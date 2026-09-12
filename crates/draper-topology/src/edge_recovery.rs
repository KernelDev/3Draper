// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Surface-surface intersection for edge recovery (Vision 2036 §1.4).
//!
//! Reconstructs LOST topological edges — edges missing from BOTH
//! adjacent faces' wires (e.g. a STEP file whose EDGE_CURVE entity was
//! dropped, truncated, or never written) — by intersecting the two
//! adjacent surfaces. This is the non-destructive counterpart to face
//! removal: instead of deleting "bad" faces or capping holes with
//! invented planar patches ([`crate::healing`]'s `fill_holes`), the
//! pass recovers the exact junction geometry:
//!
//! 1. **Gap detection** — walk every wire of every face and find
//!    consecutive coedges whose effective endpoints do NOT meet
//!    (distance `>= 2 × gap_tolerance`): the coedge that used to sit
//!    between them is missing. The generous lower bound separates true
//!    losses from tolerant-vertex mismatches (sloppy but PRESENT
//!    topology — recovering an edge there would duplicate a
//!    connection that already exists topologically).
//! 2. **Gap pairing** — a lost shared edge leaves a matching gap in
//!    BOTH adjacent faces with (nearly) identical endpoints, traversed
//!    in opposite directions for a manifold shell. Gaps are paired
//!    greedily in deterministic order (reversed orientation first —
//!    the manifold-consistent interpretation; then same-orientation
//!    for misoriented input shells).
//! 3. **SSI reconstruction** — [`crate::boolean::intersect_surfaces`]
//!    (the Vision 2036 §2.1/§2.2 machinery) returns the exact junction
//!    curve — analytic (Line/Circle/Ellipse) or a fitted B-spline —
//!    plus analytical PCURVEs in both surfaces' UV domains. The best
//!    branch is the one whose projections of BOTH gap endpoints stay
//!    within `projection_tolerance` (smallest maximum distance wins;
//!    then shortest arc; then branch index — fully deterministic).
//! 4. **Trim + insert** — the gap endpoints are projected onto the
//!    chosen branch; the segment between the projections becomes the
//!    recovered [`Edge`] (exact curve, trimmed param range,
//!    authoritative `start_vertex_point`/`end_vertex_point` overrides
//!    so the mesh edge cache discretizes it identically for both
//!    faces). One coedge is inserted into each face's wire (opposite
//!    orientations for a manifold pair), the SSI PCURVEs are attached
//!    as [`crate::entity::CoEdge::curve_2d`], and the edge joins both
//!    faces' working lists with the SAME id — `Solid::rebuild_store`
//!    dedups it into one canonical edge, so the C5 edge cache emits
//!    bit-identical vertex arrays on both sides (watertight by
//!    construction).
//!
//! # Parameter-space contract
//!
//! The SSI PCURVEs share the 3D branch curve's parameterization (the
//! Vision 2036 §2.2 derivation fits them at the branch's own sample
//! parameters), which is exactly the STEP `SURFACE_CURVE` semantics
//! the mesh edge cache validates for (identity mapping, with
//! projection fallback). Attaching them is therefore safe AND exact:
//! `compute_uvs`' candidate A evaluates `curve_2d.point_at(t)` at the
//! edge's own global curve parameters.
//!
//! # Loop-level recovery (closed lost edges)
//!
//! A lost CLOSED edge — a full circle capping a cylinder, an ellipse
//! from an oblique cut — leaves no open gap at all: the flanking
//! coedges of the neighbor face still meet exactly at the shared
//! vertex (the gap "degenerates to a point"), and the capped face's
//! wire becomes EMPTY. The pass [`recover_lost_closed_loops`] detects
//! that defect signature and reconstructs the closed edge from the
//! same SSI machinery:
//!
//! 1. **Candidate** — a face with an empty outer/inner wire whose
//!    working-list edges are not referenced by any OTHER face's wire
//!    (the orphan filter). The native cylinder's wire-less lateral
//!    face shares its circles with the disk faces' wires and is
//!    therefore NOT a candidate — an empty wire alone is a sanctioned
//!    full-surface representation (see `ShapeBuilder::make_cylinder`).
//! 2. **Neighbor search** — every other face G (deterministic face
//!    order) is intersected with the candidate's surface; a branch
//!    whose curve is CLOSED (endpoints coincide) and free of surviving
//!    edges (the closed existing-edge guard: an edge whose start,
//!    midpoint and end all lie on the closed curve is a survivor, not
//!    a loss) qualifies.
//! 3. **Junction match** — the closed edge's vertex is where G's wire
//!    has a junction (consecutive coedges meeting within gap
//!    tolerance — the degenerate gap) whose endpoints project onto the
//!    closed curve within projection tolerance.
//! 4. **Re-parameterize & insert** — the edge is anchored to G's
//!    traversal: `param_range = (t_v, t_v + period)` where `t_v` is
//!    the junction's projection (periodic curves rotate their domain
//!    so the vertex lands ON the curve — no closure kink), the
//!    vertex-point overrides are the junction's own arrive/depart
//!    points, and G's coedge is FORWARD (walk continuity through the
//!    junction) while the empty-wire face's coedge is REVERSED
//!    (manifold pair).
//!
//! # Limitations (documented, by design)
//!
//! - Loop-level recovery reconstructs ONE closed curve per empty wire.
//!   A lost loop made of several edges on DIFFERENT intersection
//!   curves (a square hole, one edge per neighbor face) is not
//!   recovered — that needs multi-neighbor loop assembly (future
//!   work). A loop whose edges all lie on one intersection curve is
//!   recovered as a single edge (heals the shell, changes the edge
//!   count).
//! - If BOTH faces' wires are empty and the shared edge survives only
//!   in the working lists (orphaned), the existing-edge guard skips
//!   recovery — that defect is edge re-referencing (stitching class),
//!   not SSI reconstruction.
//! - For periodic curves (circle/ellipse) the SHORTER arc between the
//!   two gap endpoints is chosen. A face bounded by the longer arc
//!   (e.g. a 3/4-disc "Pac-Man" face that lost its major arc) is
//!   recovered with the minor arc instead — the shell still closes,
//!   but with the wrong trimming region. Endpoint data alone cannot
//!   disambiguate this case.
//! - One-sided losses (the edge survives in ONE face's wire) are not
//!   recovered — they are stitching defects, not losses, and belong
//!   to `close_gaps`-style merging.
//! - The recovered edge's direction is anchored to gap A (face A's
//!   traversal); `param_range` may therefore be DECREASING
//!   (`t0 > t1`) — a sanctioned representation (the STEP
//!   `ORIENTED_EDGE .F.` "baked-reversed" contract) which the mesh
//!   edge cache canonicalizes via [`crate::entity::Edge::reversed`].

use crate::boolean::{create_polyline_curve, intersect_surfaces};
use crate::entity::{CoEdge, Edge, Face, Shell, TopoId, Wire};
use crate::healing::HealingParams;
use draper_geometry::{Curve2d, Curve3d, Point3d, Surface, ToleranceContext};

// ============================================================
// Parameters & report
// ============================================================

/// Parameters for the SSI edge-recovery pass.
///
/// All thresholds derive from [`HealingParams`] via
/// [`EdgeRecoveryParams::from_healing`] so the pass stays consistent
/// with the rest of the pipeline at any model scale.
#[derive(Clone, Debug)]
pub struct EdgeRecoveryParams {
    /// Base geometric tolerance (floor for the recovered edge's
    /// tolerance and the minimum segment arc length).
    pub tolerance: f64,
    /// Coincidence tolerance for gap PAIRING and the
    /// existing-edge guard. Mirrors `HealingParams::gap_tolerance`
    /// (`tolerance * gap_factor`): endpoints of a true gap pair are
    /// the SAME two vertices seen from two faces.
    pub gap_tolerance: f64,
    /// Upper bound on the 3D length of a recoverable gap (guards
    /// against pairing unrelated gaps in degenerate shells). Defaults
    /// to the shell's bounding-box diagonal.
    pub max_gap_length: f64,
    /// Maximum distance from a gap endpoint to the intersection curve
    /// for the branch to qualify.
    pub projection_tolerance: f64,
    /// Tolerance context for the SSI call. When `None`, a
    /// model-scale context is derived from the shell's extent.
    pub tolerance_context: Option<ToleranceContext>,
}

impl EdgeRecoveryParams {
    /// Derive the pass parameters from the healing pipeline's params,
    /// scaled by the shell's bounding-box diagonal (from the working
    /// lists' effective edge endpoints).
    pub fn from_healing(hp: &HealingParams, shell: &Shell, working: &[Vec<Edge>]) -> Self {
        let diag = shell_model_scale(shell, working);
        Self {
            tolerance: hp.tolerance,
            gap_tolerance: hp.gap_tolerance(),
            // Cap at the model's own diagonal: a pair of gaps longer
            // than the model is not a plausible shared-edge loss.
            max_gap_length: diag.max(2.0 * hp.gap_tolerance()),
            projection_tolerance: 10.0 * hp.tolerance.max(hp.gap_tolerance()),
            tolerance_context: hp.tolerance_context.clone(),
        }
    }
}

/// Report describing what the edge-recovery pass changed.
#[derive(Clone, Debug, Default)]
pub struct EdgeRecoveryReport {
    /// Number of open wire gaps detected (after length filtering).
    pub gaps_detected: u32,
    /// Number of lost edges reconstructed via SSI (open gaps and
    /// closed loops).
    pub edges_recovered: u32,
    /// Number of empty-wire loop-loss candidates detected (after the
    /// orphan filter).
    pub loops_detected: u32,
    /// Number of lost CLOSED edges reconstructed via loop-level SSI
    /// (counted in `edges_recovered` as well).
    pub loops_recovered: u32,
    /// Human-readable messages.
    pub messages: Vec<String>,
}

// ============================================================
// Gap model
// ============================================================

/// Identifies a wire within a face (outer or one of the inner wires).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum WireRef {
    Outer,
    Inner(usize),
}

impl WireRef {
    fn key(self) -> (u8, usize) {
        match self {
            WireRef::Outer => (0, 0),
            WireRef::Inner(i) => (1, i),
        }
    }
}

/// An open span in a wire: the coedge at index `after` ends at
/// `start`, and the next coedge (wrap-around included) starts at
/// `end`, with `start`/`end` farther apart than the detection floor —
/// the coedge that used to connect them is missing.
#[derive(Clone, Debug)]
struct WireGap {
    face: usize,
    wire: WireRef,
    /// Index of the coedge the gap FOLLOWS (insertion happens at
    /// `after + 1`).
    after: usize,
    /// Effective end point of `coedges[after]`.
    start: Point3d,
    /// Effective start point of `coedges[(after + 1) % len]`.
    end: Point3d,
}

/// A pending coedge insertion (applied after all pairing, in
/// descending positional order so pre-computed indices stay valid).
struct Insertion {
    face: usize,
    wire: WireRef,
    after: usize,
    coedge: CoEdge,
}

/// The recovered edge plus the two coedges that reference it.
struct RecoveredEdge {
    edge: Edge,
    /// Coedge for gap A's wire — always forward (the edge is built
    /// anchored to A's traversal).
    coedge_a: CoEdge,
    /// Coedge for gap B's wire — forward for same-orientation pairing,
    /// reversed for the manifold (reversed) pairing.
    coedge_b: CoEdge,
}

// ============================================================
// Entry point
// ============================================================

/// Detect open-wire gaps and reconstruct the lost shared edges via
/// surface-surface intersection (Vision 2036 §1.4), then run the
/// loop-level pass for lost CLOSED edges (empty wires + degenerate
/// junction gaps).
///
/// Mutates `shell` (coedge insertions into the gapped wires) and
/// `working` (the recovered edge joins both faces' lists with the
/// same id — `Solid::rebuild_store` unifies it downstream).
///
/// Deterministic: gap collection iterates faces/wires/coedges in
/// index order, pairing is greedy over sorted-order candidates
/// (reversed-orientation pass first), branch selection is a strict
/// lexicographic comparison, and projection is fixed-sample +
/// ternary refinement. No `HashMap` iteration order leaks.
pub fn recover_lost_edges(
    shell: &mut Shell,
    working: &mut [Vec<Edge>],
    params: &EdgeRecoveryParams,
) -> EdgeRecoveryReport {
    let mut report = EdgeRecoveryReport::default();

    if shell.faces.len() != working.len() {
        report
            .messages
            .push("face/working-list length mismatch — edge recovery skipped".to_string());
        return report;
    }

    // A gap must be long enough to be a real loss (not a tolerant
    // vertex mismatch between PRESENT coedges) and short enough to be
    // a plausible shared edge of this model.
    let min_gap = 2.0 * params.gap_tolerance;
    let gaps = collect_gaps(shell, working, min_gap, params.max_gap_length);
    report.gaps_detected = gaps.len() as u32;

    let model_scale = shell_model_scale(shell, working);

    // Existing-edge guard: if ANY edge in the shell already spans the
    // gap's endpoints, the defect is not a loss (sloppy vertices, or
    // an unreferenced surviving edge) — skip to avoid duplication.
    let existing: Vec<(Point3d, Point3d)> = working
        .iter()
        .flat_map(|list| list.iter())
        .filter_map(effective_endpoints)
        .collect();

    let pair_tol = params.gap_tolerance;
    let mut used = vec![false; gaps.len()];
    let mut insertions: Vec<Insertion> = Vec::new();
    let mut new_edges: Vec<(usize, Edge)> = Vec::new();

    // Two pairing passes: reversed endpoints first (the
    // manifold-consistent interpretation of a shared edge), then
    // same-orientation (misoriented shells). Greedy + deterministic.
    for prefer_reversed in [true, false] {
        if gaps.is_empty() {
            break;
        }
        for i in 0..gaps.len() {
            if used[i] {
                continue;
            }
            for j in (i + 1)..gaps.len() {
                if used[j] {
                    continue;
                }
                let reversed = match pair_orientation(&gaps[i], &gaps[j], pair_tol) {
                    Some(r) => r,
                    None => continue,
                };
                if reversed != prefer_reversed {
                    continue;
                }
                if let Some(rec) = try_reconstruct(
                    shell,
                    &gaps[i],
                    &gaps[j],
                    reversed,
                    params,
                    model_scale,
                    &existing,
                ) {
                    used[i] = true;
                    used[j] = true;
                    insertions.push(Insertion {
                        face: gaps[i].face,
                        wire: gaps[i].wire,
                        after: gaps[i].after,
                        coedge: rec.coedge_a,
                    });
                    insertions.push(Insertion {
                        face: gaps[j].face,
                        wire: gaps[j].wire,
                        after: gaps[j].after,
                        coedge: rec.coedge_b,
                    });
                    new_edges.push((gaps[i].face, rec.edge.clone()));
                    new_edges.push((gaps[j].face, rec.edge));
                    report.edges_recovered += 1;
                    break; // gap i consumed
                }
            }
        }
    }

    // Apply the coedge insertions in DESCENDING (face, wire, after)
    // order: inserting at `after + 1` shifts higher indices only, so
    // positions computed during detection stay valid within each wire.
    insertions.sort_by(|a, b| {
        let ka = (a.face, a.wire.key(), a.after);
        let kb = (b.face, b.wire.key(), b.after);
        kb.cmp(&ka)
    });
    for ins in insertions {
        if ins.face >= shell.faces.len() {
            continue;
        }
        let face = &mut shell.faces[ins.face];
        if let Some(wire) = wire_mut(face, ins.wire) {
            let pos = (ins.after + 1).min(wire.coedges.len());
            wire.coedges.insert(pos, ins.coedge);
            if wire.coedges.len() > 1 {
                wire.closed = true;
            }
        }
    }

    // The recovered edge joins both faces' working lists with the
    // SAME id — rebuild_store dedups it into one canonical edge.
    for (face_idx, edge) in new_edges {
        if face_idx < working.len() {
            working[face_idx].push(edge);
        }
    }

    if report.edges_recovered > 0 {
        report.messages.push(format!(
            "Recovered {} lost edge(s) via surface-surface intersection",
            report.edges_recovered
        ));
    }

    // Loop-level phase (Vision 2036 §1.4, session 34): lost CLOSED
    // edges leave empty wires and degenerate junction gaps — no open
    // gap for the pairing phase above. Runs on the post-open-gap
    // state (an open-gap insertion can shift junction indices).
    let loop_report = recover_lost_closed_loops(shell, working, params);
    report.loops_detected = loop_report.loops_detected;
    report.loops_recovered = loop_report.loops_recovered;
    report.edges_recovered += loop_report.loops_recovered;
    report.messages.extend(loop_report.messages);

    report
}

// ============================================================
// Gap detection
// ============================================================

/// Effective (start, end) points of an edge for wire-walking — the
/// authoritative vertex-point overrides win over curve evaluation
/// (bit-identical across edges sharing the same geometric vertex,
/// the session-22 watertight mechanism). Degenerate edges and edges
/// without resolvable endpoints yield `None` (treated as connected —
/// conservative, avoids false gaps).
fn effective_endpoints(edge: &Edge) -> Option<(Point3d, Point3d)> {
    if edge.degenerate {
        return None;
    }
    let start = edge.start_vertex_point.or_else(|| edge.point_at(0.0))?;
    let end = edge.end_vertex_point.or_else(|| edge.point_at(1.0))?;
    Some((start, end))
}

/// Effective (start, end) of a coedge traversal: `forward` coedges
/// walk the edge start→end, reversed coedges walk end→start.
fn coedge_span(ce: &CoEdge, endpoints: &std::collections::HashMap<TopoId, (Point3d, Point3d)>) -> Option<(Point3d, Point3d)> {
    let (s, e) = endpoints.get(&ce.edge)?;
    if ce.forward {
        Some((*s, *e))
    } else {
        Some((*e, *s))
    }
}

/// Collect all open gaps in the shell's wires, in deterministic order
/// (face idx asc, outer wire first, then inner wires in order, coedge
/// order asc). A gap qualifies when its 3D length lies within
/// `[min_gap, max_gap]`.
fn collect_gaps(
    shell: &Shell,
    working: &[Vec<Edge>],
    min_gap: f64,
    max_gap: f64,
) -> Vec<WireGap> {
    let mut gaps = Vec::new();

    for (fi, face) in shell.faces.iter().enumerate() {
        let endpoints: std::collections::HashMap<TopoId, (Point3d, Point3d)> = working
            .get(fi)
            .map(|list| {
                list.iter()
                    .filter_map(|e| effective_endpoints(e).map(|ep| (e.id, ep)))
                    .collect()
            })
            .unwrap_or_default();

        let mut scan = |wire: &Wire, wref: WireRef| {
            let n = wire.coedges.len();
            if n == 0 {
                return;
            }
            for i in 0..n {
                let ce = &wire.coedges[i];
                // Wrap-around: the gap after the LAST coedge closes
                // onto the FIRST. For a single-coedge wire this is the
                // edge's own closure (closed curves give length 0).
                let nxt = &wire.coedges[(i + 1) % n];
                // A coedge followed by ITSELF (doubled closed edge)
                // cannot flank a missing edge.
                if n > 1 && ce.edge == nxt.edge {
                    continue;
                }
                let (_, ce_end) = match coedge_span(ce, &endpoints) {
                    Some(s) => s,
                    None => continue,
                };
                let (nxt_start, _) = match coedge_span(nxt, &endpoints) {
                    Some(s) => s,
                    None => continue,
                };
                let d = ce_end.distance_to(&nxt_start);
                if d >= min_gap && d <= max_gap {
                    gaps.push(WireGap {
                        face: fi,
                        wire: wref,
                        after: i,
                        start: ce_end,
                        end: nxt_start,
                    });
                }
            }
        };

        if let Some(ref w) = face.outer_wire {
            scan(w, WireRef::Outer);
        }
        for (wi, w) in face.inner_wires.iter().enumerate() {
            scan(w, WireRef::Inner(wi));
        }
    }

    gaps
}

/// Pairing orientation between two gaps.
///
/// `Some(true)` — reversed (manifold): `a.start ≈ b.end` AND
/// `a.end ≈ b.start` — the two faces traverse the shared edge in
/// opposite directions.
/// `Some(false)` — same orientation: `a.start ≈ b.start` AND
/// `a.end ≈ b.end` — both faces traverse it identically (misoriented
/// shell; `fix_normal_orientation` addresses that defect class
/// separately).
fn pair_orientation(a: &WireGap, b: &WireGap, tol: f64) -> Option<bool> {
    if a.start.distance_to(&b.end) <= tol && a.end.distance_to(&b.start) <= tol {
        return Some(true);
    }
    if a.start.distance_to(&b.start) <= tol && a.end.distance_to(&b.end) <= tol {
        return Some(false);
    }
    None
}

/// True when some existing edge already spans (`s`, `e`) in either
/// direction — the gap is a vertex-precision defect or an
/// unreferenced surviving edge, NOT a loss.
fn existing_edge_spans(existing: &[(Point3d, Point3d)], s: &Point3d, e: &Point3d, tol: f64) -> bool {
    existing.iter().any(|(a, b)| {
        (a.distance_to(s) <= tol && b.distance_to(e) <= tol)
            || (a.distance_to(e) <= tol && b.distance_to(s) <= tol)
    })
}

// ============================================================
// SSI reconstruction
// ============================================================

/// Validate an SSI PCURVE against the identity contract before
/// attaching it to a recovered coedge: `curve_2d.point_at(t)` at the
/// edge's own curve parameters must reproduce the 3D branch points on
/// the surface — the same contract the mesh edge cache applies in its
/// `compute_uvs` (candidates identity/remap + projection fallback).
///
/// Hand-rolled analytic-arm PCURVEs parametrized over a normalized
/// `[0, 1]` domain (e.g. `Circle2d`/`Line2d` from the plane×cylinder
/// circle arm) fail here and are dropped — the mesh's projection
/// fallback then computes exact UVs anyway. PCURVEs that DO share the
/// 3D branch curve's parameterization (the §2.2 generic fits —
/// `Nurbs2d` fitted at the branch's own sample parameters) validate
/// and attach, giving exact UVs for NURBS-surface recoveries.
fn pcurve_validates(
    c2d: &Curve2d,
    curve: &Curve3d,
    t_range: (f64, f64),
    surface: &Surface,
    tol: f64,
) -> bool {
    const SAMPLES: usize = 8;
    let (t0, t1) = t_range;
    for i in 0..=SAMPLES {
        let t = t0 + (t1 - t0) * (i as f64 / SAMPLES as f64);
        let uv = c2d.point_at(t);
        let p3 = surface.point_at(uv.u, uv.v);
        if p3.distance_to(&curve.point_at(t)) > tol {
            return false;
        }
    }
    true
}

/// The best intersection branch for a gap pair (selection state).
struct BranchPick {
    /// Max of the two endpoint projection distances (primary score).
    score_dist: f64,
    /// Arc length of the trimmed segment (secondary score).
    score_arc: f64,
    branch_idx: usize,
    curve: Curve3d,
    t_range: (f64, f64),
    pcurve_a: Option<Curve2d>,
    pcurve_b: Option<Curve2d>,
    tolerance: f64,
}

/// Reconstruct the lost shared edge for a paired gap (a, b) by
/// intersecting the adjacent faces' surfaces.
///
/// `reversed` selects gap B's coedge orientation (`forward =
/// !reversed`): a manifold pair traverses the shared edge in opposite
/// directions.
fn try_reconstruct(
    shell: &Shell,
    a: &WireGap,
    b: &WireGap,
    reversed: bool,
    params: &EdgeRecoveryParams,
    model_scale: f64,
    existing: &[(Point3d, Point3d)],
) -> Option<RecoveredEdge> {
    // Guard: not a loss if an edge already spans these endpoints.
    if existing_edge_spans(existing, &a.start, &a.end, params.gap_tolerance) {
        return None;
    }

    let surf_a = shell.faces.get(a.face)?.surface.as_ref()?;
    let surf_b = shell.faces.get(b.face)?.surface.as_ref()?;

    let tol_ctx = params
        .tolerance_context
        .clone()
        .unwrap_or_else(|| ToleranceContext::from_model_scale(model_scale));

    let branches = intersect_surfaces(surf_a, surf_b, &tol_ctx);

    let mut best: Option<BranchPick> = None;
    for (bi, ic) in branches.iter().enumerate() {
        if ic.points.len() < 2 {
            continue;
        }
        let win = match branch_window(ic) {
            Some(w) => w,
            None => continue,
        };
        let (t0, d0) = project_onto_window(&win, &a.start);
        let (t1, d1) = project_onto_window(&win, &a.end);
        let max_d = d0.max(d1);
        if max_d > params.projection_tolerance {
            continue;
        }
        let mut t0 = t0;
        let mut t1 = t1;
        adjust_periodic(&win.curve, &mut t0, &mut t1);
        let arc = segment_arc_length(&win, t0, t1);
        // Degenerate (sub-tolerance) segments are not recoveries.
        if arc < params.tolerance.max(1e-12) {
            continue;
        }
        // Attach only PCURVEs that satisfy the identity parameter-space
        // contract (see `pcurve_validates`) — anything else is dropped
        // and the mesh falls back to surface projection.
        let pcurve_a = ic
            .pcurve_a
            .as_ref()
            .filter(|c| pcurve_validates(c, &win.curve, (t0, t1), surf_a, params.projection_tolerance))
            .cloned();
        let pcurve_b = ic
            .pcurve_b
            .as_ref()
            .filter(|c| pcurve_validates(c, &win.curve, (t0, t1), surf_b, params.projection_tolerance))
            .cloned();
        let better = match &best {
            None => true,
            Some(c) => (max_d, arc, bi) < (c.score_dist, c.score_arc, c.branch_idx),
        };
        if better {
            best = Some(BranchPick {
                score_dist: max_d,
                score_arc: arc,
                branch_idx: bi,
                curve: win.curve,
                t_range: (t0, t1),
                pcurve_a,
                pcurve_b,
                tolerance: ic.tolerance,
            });
        }
    }

    let pick = best?;
    let (t0, t1) = pick.t_range;

    // The edge is anchored to gap A's traversal: point_at(0) lands on
    // a.start, point_at(1) on a.end — regardless of whether the
    // trimmed range is increasing or decreasing (the baked-reversed
    // contract the mesh edge cache canonicalizes).
    let edge = Edge {
        id: TopoId::new(),
        curve: Some(pick.curve),
        param_range: (t0, t1),
        vertex_start: None,
        vertex_end: None,
        start_vertex_point: Some(a.start),
        end_vertex_point: Some(a.end),
        forward: t0 <= t1,
        tolerance: pick.tolerance.max(params.tolerance),
        degenerate: false,
        step_entity_id: None,
    };

    // Vertex-point overrides guarantee the discretized endpoints are
    // bit-identical to the flanking edges' endpoints (watertight).
    let mut coedge_a = CoEdge::new(edge.id, true);
    coedge_a.curve_2d = pick.pcurve_a;
    let mut coedge_b = CoEdge::new(edge.id, !reversed);
    coedge_b.curve_2d = pick.pcurve_b;

    Some(RecoveredEdge {
        edge,
        coedge_a,
        coedge_b,
    })
}

// ============================================================
// Curve sampling & projection
// ============================================================

/// A bounded evaluation window over an intersection branch's curve.
struct CurveWindow {
    curve: Curve3d,
    t_lo: f64,
    t_hi: f64,
}

/// Derive the sampling window for an intersection branch.
///
/// Bounded parameterizations (circle/ellipse/arc/NURBS/PCurve/
/// composite) sample their own domain. Unbounded ones (line, and the
/// rare conic sections) derive a window from the SSI's marched
/// polyline extent; conics fall back to the polyline interpolant
/// (bounded `[0, 1]`) rather than an unbounded domain.
fn branch_window(ic: &crate::boolean::IntersectionCurve) -> Option<CurveWindow> {
    match &ic.curve {
        Some(c @ Curve3d::Line(line)) => {
            // The SSI samples the line over its branch extent (e.g.
            // plane×plane marches ±1000); project those samples onto
            // the line to recover the window, padded by 5%.
            if ic.points.len() < 2 {
                return None;
            }
            let mut ts: Vec<f64> = ic
                .points
                .iter()
                .map(|p| {
                    (p.x - line.origin.x) * line.direction.x
                        + (p.y - line.origin.y) * line.direction.y
                        + (p.z - line.origin.z) * line.direction.z
                })
                .collect();
            ts.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
            let t_lo = ts[0];
            let t_hi = ts[ts.len() - 1];
            let margin = (t_hi - t_lo).max(1.0) * 0.05;
            Some(CurveWindow {
                curve: c.clone(),
                t_lo: t_lo - margin,
                t_hi: t_hi + margin,
            })
        }
        Some(_c @ Curve3d::Hyperbola(_)) | Some(_c @ Curve3d::Parabola(_)) => {
            if ic.points.len() < 2 {
                return None;
            }
            let poly = create_polyline_curve(&ic.points);
            Some(CurveWindow {
                curve: poly,
                t_lo: 0.0,
                t_hi: 1.0,
            })
        }
        Some(c) => {
            let (t_lo, t_hi) = c.param_range();
            if t_lo.is_finite() && t_hi.is_finite() && t_hi > t_lo {
                Some(CurveWindow {
                    curve: c.clone(),
                    t_lo,
                    t_hi,
                })
            } else {
                None
            }
        }
        None => {
            if ic.points.len() < 2 {
                return None;
            }
            let poly = create_polyline_curve(&ic.points);
            Some(CurveWindow {
                curve: poly,
                t_lo: 0.0,
                t_hi: 1.0,
            })
        }
    }
}

/// Project a point onto the windowed curve: dense fixed sampling for
/// the global minimum, then ternary refinement inside the bracketing
/// interval. Fully deterministic (fixed sample count and iteration
/// count — no data-dependent control flow).
///
/// Returns `(parameter, distance)`.
fn project_onto_window(win: &CurveWindow, p: &Point3d) -> (f64, f64) {
    const N: usize = 512;
    let span = win.t_hi - win.t_lo;

    let mut best_t = win.t_lo;
    let mut best_d = f64::MAX;
    for i in 0..=N {
        let t = win.t_lo + span * (i as f64 / N as f64);
        let d = win.curve.point_at(t).distance_sq_to(p);
        if d < best_d {
            best_d = d;
            best_t = t;
        }
    }

    // Ternary refinement within the neighboring sample cell.
    let cell = span / N as f64;
    let mut lo = (best_t - cell).max(win.t_lo);
    let mut hi = (best_t + cell).min(win.t_hi);
    for _ in 0..80 {
        let m1 = lo + (hi - lo) / 3.0;
        let m2 = hi - (hi - lo) / 3.0;
        let d1 = win.curve.point_at(m1).distance_sq_to(p);
        let d2 = win.curve.point_at(m2).distance_sq_to(p);
        if d1 < d2 {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    let t = (lo + hi) / 2.0;
    (t, win.curve.point_at(t).distance_to(p))
}

/// For periodic curves (circle/ellipse), wrap the segment
/// `t0 → t1` to the SHORTER arc: `t1` is adjusted so that
/// `|t1 - t0| <= period / 2` (mod the 2π period). Non-periodic curves
/// are untouched. See the module docs for the documented ambiguity.
fn adjust_periodic(curve: &Curve3d, t0: &mut f64, t1: &mut f64) {
    if !matches!(curve, Curve3d::Circle(_) | Curve3d::Ellipse(_)) {
        return;
    }
    let period = 2.0 * std::f64::consts::PI;
    let half = period / 2.0;
    let mut d = (*t1 - *t0) % period;
    if d <= -half {
        d += period;
    }
    if d > half {
        d -= period;
    }
    *t1 = *t0 + d;
}

/// Arc length of the curve segment `t0 → t1` (64-sample chord sum;
/// handles decreasing ranges).
fn segment_arc_length(win: &CurveWindow, t0: f64, t1: f64) -> f64 {
    const N: usize = 64;
    let mut len = 0.0;
    let mut prev = win.curve.point_at(t0);
    for i in 1..=N {
        let t = t0 + (t1 - t0) * (i as f64 / N as f64);
        let p = win.curve.point_at(t);
        len += prev.distance_to(&p);
        prev = p;
    }
    len
}

// ============================================================
// Loop-level recovery (closed lost edges)
// ============================================================

/// A junction between two consecutive coedges of a wire: the walk
/// arrives at `arrive` (end of `coedges[after]`) and departs from
/// `depart` (start of `coedges[(after + 1) % n]`). A lost CLOSED edge
/// leaves a junction whose endpoints still meet within gap tolerance
/// (the "degenerate gap") — the closed curve used to pass through it.
#[derive(Clone, Debug)]
struct WireJunction {
    wire: WireRef,
    /// Index of the coedge the junction FOLLOWS (insertion happens at
    /// `after + 1`).
    after: usize,
    /// Effective end point of `coedges[after]` — the arriving flank.
    arrive: Point3d,
    /// Effective start point of `coedges[(after + 1) % n]` — the
    /// departing flank.
    depart: Point3d,
}

/// An empty wire whose whole loop is lost (loop-recovery candidate).
#[derive(Clone, Copy, Debug)]
struct EmptyWire {
    face: usize,
    wire: WireRef,
}

/// The recovered closed edge plus the two coedges that reference it:
/// G's coedge is FORWARD (walk continuity through the junction —
/// arrive → depart), the empty-wire face's coedge is REVERSED (the
/// manifold opposite traversal; an empty wire has no continuity
/// constraint of its own).
struct RecoveredClosed {
    edge: Edge,
    coedge_g: CoEdge,
    coedge_f: CoEdge,
}

/// Collect empty-wire loop-loss candidates: faces whose outer/inner
/// wire exists but has no coedges, and whose working-list edges are
/// not all shared with other faces' wires.
///
/// The orphan filter is what separates a real loss from the
/// SANCTIONED empty-wire representation: the native cylinder's
/// wire-less lateral face keeps its circles in the working lists,
/// referenced by the disk faces' wires (`ShapeBuilder::make_cylinder`)
/// — not a loss. A face with an EMPTY working list, or with edges no
/// other face references, has lost its loop.
fn collect_empty_wire_candidates(shell: &Shell, working: &[Vec<Edge>]) -> Vec<EmptyWire> {
    // Edge ids referenced by any face's wires. Empty wires contribute
    // nothing, so this is exactly "referenced by OTHER faces" for
    // candidate faces.
    let mut referenced: std::collections::HashSet<TopoId> = std::collections::HashSet::new();
    for face in &shell.faces {
        if let Some(ref w) = face.outer_wire {
            for ce in &w.coedges {
                referenced.insert(ce.edge);
            }
        }
        for w in &face.inner_wires {
            for ce in &w.coedges {
                referenced.insert(ce.edge);
            }
        }
    }

    let mut out = Vec::new();
    for (fi, face) in shell.faces.iter().enumerate() {
        if face.surface.is_none() {
            continue;
        }
        // Sanctioned full-surface representation: a non-empty working
        // list whose every edge is referenced elsewhere.
        let shared = working
            .get(fi)
            .map(|list| !list.is_empty() && list.iter().all(|e| referenced.contains(&e.id)))
            .unwrap_or(false);
        if shared {
            continue;
        }
        if let Some(ref w) = face.outer_wire {
            if w.coedges.is_empty() {
                out.push(EmptyWire {
                    face: fi,
                    wire: WireRef::Outer,
                });
            }
        }
        for (wi, w) in face.inner_wires.iter().enumerate() {
            if w.coedges.is_empty() {
                out.push(EmptyWire {
                    face: fi,
                    wire: WireRef::Inner(wi),
                });
            }
        }
    }
    out
}

/// Collect the junctions of one face's wires (consecutive coedges
/// whose effective endpoints MEET within `meet_tol` — including exact
/// meetings, i.e. the degenerate gaps the open-gap pass cannot see).
/// The same edge may appear as both flanks (a seam edge referenced
/// twice — forward then reversed — puts the closed edge's vertex at
/// exactly such a junction); the dangerous doubled-CLOSED-edge case
/// is rejected downstream by the existing-edge guard. Deterministic
/// order: outer wire first, then inner wires, coedge index ascending.
fn collect_junctions(
    face: &Face,
    endpoints: &std::collections::HashMap<TopoId, (Point3d, Point3d)>,
    meet_tol: f64,
) -> Vec<WireJunction> {
    let mut out = Vec::new();
    let mut scan = |wire: &Wire, wref: WireRef| {
        let n = wire.coedges.len();
        if n < 2 {
            return;
        }
        for i in 0..n {
            let ce = &wire.coedges[i];
            let nxt = &wire.coedges[(i + 1) % n];
            let arrive = match coedge_span(ce, endpoints) {
                Some((_, e)) => e,
                None => continue,
            };
            let depart = match coedge_span(nxt, endpoints) {
                Some((s, _)) => s,
                None => continue,
            };
            if arrive.distance_to(&depart) <= meet_tol {
                out.push(WireJunction {
                    wire: wref,
                    after: i,
                    arrive,
                    depart,
                });
            }
        }
    };
    if let Some(ref w) = face.outer_wire {
        scan(w, WireRef::Outer);
    }
    for (wi, w) in face.inner_wires.iter().enumerate() {
        scan(w, WireRef::Inner(wi));
    }
    out
}

/// True when an existing edge lies ON the closed curve — its start,
/// midpoint and end all project within `tol` — the closed edge
/// survives somewhere in the working lists and recovery would
/// duplicate it. A 16-sample bounding box pre-filter rejects distant
/// edges before any projection.
fn existing_edge_on_curve(working: &[Vec<Edge>], win: &CurveWindow, tol: f64) -> bool {
    // Bounding box of the closed curve (16 samples — generous for the
    // guard's purpose; the projection test below is exact).
    let mut lo = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut hi = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    for i in 0..=16 {
        let t = win.t_lo + (win.t_hi - win.t_lo) * (i as f64 / 16.0);
        let p = win.curve.point_at(t);
        lo.x = lo.x.min(p.x);
        lo.y = lo.y.min(p.y);
        lo.z = lo.z.min(p.z);
        hi.x = hi.x.max(p.x);
        hi.y = hi.y.max(p.y);
        hi.z = hi.z.max(p.z);
    }
    let pad = tol;

    for list in working {
        for e in list {
            if e.degenerate || e.curve.is_none() {
                continue;
            }
            let mut on = true;
            for k in 0..=2usize {
                let Some(p) = e.point_at(k as f64 / 2.0) else {
                    on = false;
                    break;
                };
                // Bbox pre-filter.
                if p.x < lo.x - pad
                    || p.x > hi.x + pad
                    || p.y < lo.y - pad
                    || p.y > hi.y + pad
                    || p.z < lo.z - pad
                    || p.z > hi.z + pad
                {
                    on = false;
                    break;
                }
                let (_, d) = project_onto_window(win, &p);
                if d > tol {
                    on = false;
                    break;
                }
            }
            if on {
                return true;
            }
        }
    }
    false
}

/// Build the recovered closed edge for a matched junction: the edge
/// is anchored to G's traversal (param range starting at the junction
/// projection, vertex-point overrides = the junction's own flank
/// points), with SSI PCURVEs attached when they satisfy the identity
/// contract over the FULL closed range.
fn build_closed_recovery(
    win: &CurveWindow,
    ic: &crate::boolean::IntersectionCurve,
    junc: &WireJunction,
    surf_f: &Surface,
    surf_g: &Surface,
    params: &EdgeRecoveryParams,
) -> Option<RecoveredClosed> {
    let period = win.t_hi - win.t_lo;
    let (t0, t1) = match &win.curve {
        Curve3d::Circle(_) | Curve3d::Ellipse(_) => {
            // Periodic: rotate the domain so the vertex lands ON the
            // junction's projection — point_at(t_v) IS the vertex
            // (no closure kink from snapping an off-curve override).
            let (t_v, _) = project_onto_window(win, &junc.arrive);
            (t_v, t_v + period)
        }
        _ => {
            // Non-rotatable parameterizations (polyline/NURBS loops):
            // the junction must sit at the loop's own closure point —
            // the parameterization cannot start elsewhere.
            let (t_v, d) = project_onto_window(win, &junc.arrive);
            let at_lo = (t_v - win.t_lo).abs() <= params.gap_tolerance;
            let at_hi = (t_v - win.t_hi).abs() <= params.gap_tolerance;
            if !at_lo && !at_hi {
                return None;
            }
            let _ = d;
            (win.t_lo, win.t_hi)
        }
    };

    let edge = Edge {
        id: TopoId::new(),
        curve: Some(win.curve.clone()),
        param_range: (t0, t1),
        vertex_start: None,
        vertex_end: None,
        start_vertex_point: Some(junc.arrive),
        end_vertex_point: Some(junc.depart),
        forward: true, // increasing range by construction
        tolerance: ic.tolerance.max(params.tolerance),
        degenerate: false,
        step_entity_id: None,
    };

    // Attach only PCURVEs satisfying the identity parameter-space
    // contract over the full closed range (affine/periodic PCURVEs
    // extrapolate exactly — see `pcurve_validates`).
    let pcurve_f = ic
        .pcurve_a
        .as_ref()
        .filter(|c| pcurve_validates(c, &win.curve, (t0, t1), surf_f, params.projection_tolerance))
        .cloned();
    let pcurve_g = ic
        .pcurve_b
        .as_ref()
        .filter(|c| pcurve_validates(c, &win.curve, (t0, t1), surf_g, params.projection_tolerance))
        .cloned();

    let mut coedge_g = CoEdge::new(edge.id, true);
    coedge_g.curve_2d = pcurve_g;
    let mut coedge_f = CoEdge::new(edge.id, false);
    coedge_f.curve_2d = pcurve_f;

    Some(RecoveredClosed {
        edge,
        coedge_g,
        coedge_f,
    })
}

/// Loop-level recovery of lost CLOSED edges (Vision 2036 §1.4):
/// reconstruct the closed curve capping an empty-wire face by
/// intersecting it with a neighbor face whose wire has a matching
/// degenerate junction gap.
///
/// Deterministic: candidates in (face, wire) order, neighbors in face
/// order, branches in index order, junctions in (wire, coedge) order;
/// first match wins; one closed edge per empty wire. Junctions are
/// consumed once — two candidates cannot recover into the same wire
/// position.
fn recover_lost_closed_loops(
    shell: &mut Shell,
    working: &mut [Vec<Edge>],
    params: &EdgeRecoveryParams,
) -> EdgeRecoveryReport {
    let mut report = EdgeRecoveryReport::default();

    let candidates = collect_empty_wire_candidates(shell, working);
    report.loops_detected = candidates.len() as u32;
    if candidates.is_empty() {
        return report;
    }

    let model_scale = shell_model_scale(shell, working);

    let mut insertions: Vec<Insertion> = Vec::new();
    let mut new_edges: Vec<(usize, Edge)> = Vec::new();
    // Empty wires that received a closed coedge — their `closed` flag
    // must be set explicitly (the single-coedge len() > 1 rule of the
    // shared insertion code does not fire).
    let mut close_wires: Vec<(usize, WireRef)> = Vec::new();
    // Junctions consumed by a recovery (face, wire key, after) — a
    // second candidate must not insert into the same wire position.
    let mut used_junctions: Vec<(usize, (u8, usize), usize)> = Vec::new();

    'candidates: for cand in &candidates {
        let surf_f = match shell.faces[cand.face].surface.as_ref() {
            Some(s) => s,
            None => continue,
        };

        for (gi, g_face) in shell.faces.iter().enumerate() {
            if gi == cand.face {
                continue;
            }
            let surf_g = match g_face.surface.as_ref() {
                Some(s) => s,
                None => continue,
            };

            let endpoints: std::collections::HashMap<TopoId, (Point3d, Point3d)> = working
                .get(gi)
                .map(|list| {
                    list.iter()
                        .filter_map(|e| effective_endpoints(e).map(|ep| (e.id, ep)))
                        .collect()
                })
                .unwrap_or_default();
            let junctions = collect_junctions(g_face, &endpoints, params.gap_tolerance);
            if junctions.is_empty() {
                continue;
            }

            let tol_ctx = params
                .tolerance_context
                .clone()
                .unwrap_or_else(|| ToleranceContext::from_model_scale(model_scale));
            let branches = intersect_surfaces(surf_f, surf_g, &tol_ctx);

            for ic in branches.iter() {
                let win = match branch_window(ic) {
                    Some(w) => w,
                    None => continue,
                };
                // Closed branch only: the window's endpoints coincide
                // (full circle/ellipse, or a closed polyline loop).
                let closure = win
                    .curve
                    .point_at(win.t_lo)
                    .distance_to(&win.curve.point_at(win.t_hi));
                if closure > params.gap_tolerance {
                    continue;
                }
                // Existing-edge guard (closed variant): a surviving
                // edge on the curve means no loss.
                let guard_tol = params.gap_tolerance.max(ic.tolerance);
                if existing_edge_on_curve(working, &win, guard_tol) {
                    continue;
                }

                for junc in &junctions {
                    let jkey = (gi, junc.wire.key(), junc.after);
                    if used_junctions.contains(&jkey) {
                        continue;
                    }
                    let (_, d_arrive) = project_onto_window(&win, &junc.arrive);
                    if d_arrive > params.projection_tolerance {
                        continue;
                    }
                    let (_, d_depart) = project_onto_window(&win, &junc.depart);
                    if d_depart > params.projection_tolerance {
                        continue;
                    }
                    if let Some(rec) =
                        build_closed_recovery(&win, ic, junc, surf_f, surf_g, params)
                    {
                        used_junctions.push(jkey);
                        insertions.push(Insertion {
                            face: gi,
                            wire: junc.wire,
                            after: junc.after,
                            coedge: rec.coedge_g,
                        });
                        insertions.push(Insertion {
                            face: cand.face,
                            wire: cand.wire,
                            after: 0,
                            coedge: rec.coedge_f,
                        });
                        close_wires.push((cand.face, cand.wire));
                        new_edges.push((cand.face, rec.edge.clone()));
                        new_edges.push((gi, rec.edge));
                        report.loops_recovered += 1;
                        continue 'candidates; // one closed edge per empty wire
                    }
                }
            }
        }
    }

    // Apply the coedge insertions in DESCENDING (face, wire, after)
    // order — pre-computed positions stay valid (same contract as the
    // open-gap phase).
    insertions.sort_by(|a, b| {
        let ka = (a.face, a.wire.key(), a.after);
        let kb = (b.face, b.wire.key(), b.after);
        kb.cmp(&ka)
    });
    for ins in insertions {
        if ins.face >= shell.faces.len() {
            continue;
        }
        let face = &mut shell.faces[ins.face];
        if let Some(wire) = wire_mut(face, ins.wire) {
            let pos = (ins.after + 1).min(wire.coedges.len());
            wire.coedges.insert(pos, ins.coedge);
            if wire.coedges.len() > 1 {
                wire.closed = true;
            }
        }
    }

    // The empty wires that received a closed coedge are closed loops
    // by construction.
    for (fi, wref) in close_wires {
        if fi < shell.faces.len() {
            if let Some(wire) = wire_mut(&mut shell.faces[fi], wref) {
                if wire.coedges.len() == 1 {
                    wire.closed = true;
                }
            }
        }
    }

    // The recovered edge joins both faces' working lists with the SAME
    // id — rebuild_store dedups it into one canonical edge.
    for (face_idx, edge) in new_edges {
        if face_idx < working.len() {
            working[face_idx].push(edge);
        }
    }

    if report.loops_recovered > 0 {
        report.messages.push(format!(
            "Recovered {} lost closed edge(s) via loop-level surface-surface intersection",
            report.loops_recovered
        ));
    }

    report
}

// ============================================================
// Misc helpers
// ============================================================

fn wire_mut(face: &mut Face, w: WireRef) -> Option<&mut Wire> {
    match w {
        WireRef::Outer => face.outer_wire.as_mut(),
        WireRef::Inner(i) => face.inner_wires.get_mut(i),
    }
}

/// Model scale for the fallback SSI tolerance context: the bounding
/// box diagonal over all working-list edge endpoints (1.0 for empty
/// shells).
fn shell_model_scale(shell: &Shell, working: &[Vec<Edge>]) -> f64 {
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    let mut any = false;
    for (fi, face) in shell.faces.iter().enumerate() {
        if face.surface.is_none() {
            continue;
        }
        if let Some(list) = working.get(fi) {
            for e in list {
                if let Some((s, t)) = effective_endpoints(e) {
                    any = true;
                    min.x = min.x.min(s.x).min(t.x);
                    min.y = min.y.min(s.y).min(t.y);
                    min.z = min.z.min(s.z).min(t.z);
                    max.x = max.x.max(s.x).max(t.x);
                    max.y = max.y.max(s.y).max(t.y);
                    max.z = max.z.max(s.z).max(t.z);
                }
            }
        }
    }
    if !any {
        return 1.0;
    }
    let dx = max.x - min.x;
    let dy = max.y - min.y;
    let dz = max.z - min.z;
    (dx * dx + dy * dy + dz * dz).sqrt().max(1e-10)
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::ShapeBuilder;
    use crate::healing::{heal_solid, HealingParams};
    use crate::Solid;
    use draper_geometry::{Circle, Circle2d, CylinderSurface, Direction3d, Line2d, Plane};

    /// Recovery params for unit-scale tests.
    fn test_params() -> EdgeRecoveryParams {
        EdgeRecoveryParams {
            tolerance: 1e-6,
            gap_tolerance: 1e-5,
            max_gap_length: 10.0,
            projection_tolerance: 1e-4,
            tolerance_context: None,
        }
    }

    /// Build an open-wire face whose outer wire is the polyline chain
    /// through `pts` (line edges, forward coedges). The GAP is the
    /// wrap-around from `pts.last()` to `pts[0]`.
    fn chain_face(surface: Surface, pts: &[Point3d]) -> (Face, Vec<Edge>) {
        assert!(pts.len() >= 2);
        let mut edges = Vec::new();
        let mut coedges = Vec::new();
        for w in pts.windows(2) {
            let e = Edge::new_line(w[0], w[1]);
            coedges.push(CoEdge::new(e.id, true));
            edges.push(e);
        }
        let wire = Wire::new(coedges);
        let face = Face::new(surface, wire);
        (face, edges)
    }

    fn shell_from(parts: Vec<(Face, Vec<Edge>)>) -> (Shell, Vec<Vec<Edge>>) {
        let mut faces = Vec::with_capacity(parts.len());
        let mut working = Vec::with_capacity(parts.len());
        for (f, e) in parts {
            faces.push(f);
            working.push(e);
        }
        let shell = Shell::new(faces);
        (shell, working)
    }

    /// Maximum distance between consecutive coedge endpoints over all
    /// wires of all faces (the "openness" of the shell).
    fn max_wire_opening(shell: &Shell, working: &[Vec<Edge>]) -> f64 {
        let mut worst = 0.0_f64;
        for (fi, face) in shell.faces.iter().enumerate() {
            let endpoints: std::collections::HashMap<TopoId, (Point3d, Point3d)> = working[fi]
                .iter()
                .filter_map(|e| effective_endpoints(e).map(|ep| (e.id, ep)))
                .collect();
            for wire in face.outer_wire.iter().chain(face.inner_wires.iter()) {
                let n = wire.coedges.len();
                if n == 0 {
                    continue;
                }
                for i in 0..n {
                    let ce = &wire.coedges[i];
                    let nxt = &wire.coedges[(i + 1) % n];
                    if let (Some((_, e0)), Some((s1, _))) =
                        (coedge_span(ce, &endpoints), coedge_span(nxt, &endpoints))
                    {
                        worst = worst.max(e0.distance_to(&s1));
                    }
                }
            }
        }
        worst
    }

    /// A 2×2×2 box whose bottom-front shared edge (from (-1,-1,-1) to
    /// (1,-1,-1)) is LOST: removed from BOTH the bottom face's wire
    /// (coedge 0) and the front face's wire (coedge 3), together with
    /// their working-list entries.
    fn broken_box() -> (Shell, Vec<Vec<Edge>>) {
        let solid = ShapeBuilder::make_box(2.0, 2.0, 2.0);
        let mut shell = solid.outer_shell.clone().expect("box outer shell");
        let mut working: Vec<Vec<Edge>> = shell
            .faces
            .iter()
            .map(|f| solid.resolve_face_edges(f))
            .collect();

        let a = Point3d::new(-1.0, -1.0, -1.0);
        let b = Point3d::new(1.0, -1.0, -1.0);
        // Bottom face = rect (v0,v1,v2,v3): the v0→v1 edge is coedge 0.
        remove_coedge(&mut shell.faces[0], &mut working[0], 0, &a, &b);
        // Front face = rect (v0,v4,v5,v1): the v1→v0 edge is coedge 3.
        remove_coedge(&mut shell.faces[2], &mut working[2], 3, &a, &b);
        // Stale canonical edge_ids — from_edges_only rebuilds them.
        for f in shell.faces.iter_mut() {
            f.edge_ids.clear();
        }
        (shell, working)
    }

    fn remove_coedge(face: &mut Face, list: &mut Vec<Edge>, idx: usize, a: &Point3d, b: &Point3d) {
        let e = &list[idx];
        let (s, t) = effective_endpoints(e).expect("resolvable edge");
        let ok = (s.distance_to(a) < 1e-9 && t.distance_to(b) < 1e-9)
            || (s.distance_to(b) < 1e-9 && t.distance_to(a) < 1e-9);
        assert!(ok, "edge at {idx} does not span the expected endpoints");
        face.outer_wire.as_mut().unwrap().coedges.remove(idx);
        list.remove(idx);
    }

    /// Plane×Plane → the recovered edge is an exact LINE.
    #[test]
    fn test_recover_lost_edge_box() {
        let (mut shell, mut working) = broken_box();
        let report = recover_lost_edges(&mut shell, &mut working, &test_params());

        assert_eq!(report.edges_recovered, 1, "messages: {:?}", report.messages);
        assert_eq!(report.gaps_detected, 2);

        // Both gapped wires are closed again (4 coedges each).
        assert_eq!(shell.faces[0].outer_wire.as_ref().unwrap().coedges.len(), 4);
        assert_eq!(shell.faces[2].outer_wire.as_ref().unwrap().coedges.len(), 4);
        // The recovered edge joined both working lists (same id).
        assert_eq!(working[0].len(), 4);
        assert_eq!(working[2].len(), 4);
        assert_eq!(working[0].last().unwrap().id, working[2].last().unwrap().id);

        // Exact line geometry between the two gap endpoints.
        let e = working[0].last().unwrap();
        assert!(matches!(e.curve, Some(Curve3d::Line(_))), "curve: {:?}", e.curve);
        let a = Point3d::new(-1.0, -1.0, -1.0);
        let b = Point3d::new(1.0, -1.0, -1.0);
        assert!(e.start_vertex_point.unwrap().distance_to(&a) < 1e-9);
        assert!(e.end_vertex_point.unwrap().distance_to(&b) < 1e-9);
        let mid = e.point_at(0.5).unwrap();
        assert!(mid.distance_to(&Point3d::new(0.0, -1.0, -1.0)) < 1e-6);

        // The trimmed param range spans the edge length (2.0), in either
        // traversal direction (baked-reversed contract).
        let span = (e.param_range.1 - e.param_range.0).abs();
        assert!((span - 2.0).abs() < 1e-6, "span: {span}");

        // Watertight at the topology level: no open wires remain.
        assert!(max_wire_opening(&shell, &working) < 2.0 * test_params().gap_tolerance);
    }

    /// Plane ⊥ cylinder axis → the recovered edge is an exact CIRCLE
    /// ARC, trimmed to the gap, with both SSI PCURVEs attached and
    /// identity-consistent with the surfaces.
    #[test]
    fn test_recover_quarter_arc() {
        // Face A: plane z=1, chain open between (1,0,1) and (0,1,1) —
        // the quarter-arc gap.
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 1.0), Direction3d::Z);
        let face_a = chain_face(
            Surface::Plane(plane.clone()),
            &[
                Point3d::new(0.0, 1.0, 1.0),
                Point3d::new(-1.0, 0.0, 1.0),
                Point3d::new(0.0, -1.0, 1.0),
                Point3d::new(1.0, 0.0, 1.0),
            ],
        );
        // Face B: cylinder r=1 axis Z — reversed pairing (its gap runs
        // (0,1,1) → (1,0,1)).
        let cyl = CylinderSurface::new_z(1.0);
        let face_b = chain_face(
            Surface::Cylinder(cyl.clone()),
            &[
                Point3d::new(1.0, 0.0, 1.0),
                Point3d::new(0.0, 1.0, 3.0),
                Point3d::new(0.0, 1.0, 1.0),
            ],
        );
        let (mut shell, mut working) = shell_from(vec![face_a, face_b]);

        let report = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(report.edges_recovered, 1, "messages: {:?}", report.messages);

        // Exact circle arc trimmed to [0, π/2].
        let e = working[0].last().unwrap().clone();
        assert!(matches!(e.curve, Some(Curve3d::Circle(_))), "curve: {:?}", e.curve);
        assert!(e.param_range.0.abs() < 1e-9, "t0: {}", e.param_range.0);
        assert!(
            (e.param_range.1 - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "t1: {}",
            e.param_range.1
        );
        assert!(e.start_vertex_point.unwrap().distance_to(&Point3d::new(1.0, 0.0, 1.0)) < 1e-9);
        assert!(e.end_vertex_point.unwrap().distance_to(&Point3d::new(0.0, 1.0, 1.0)) < 1e-9);

        // Coedge orientations: A forward, B reversed (manifold pair).
        let wa = shell.faces[0].outer_wire.as_ref().unwrap();
        let wb = shell.faces[1].outer_wire.as_ref().unwrap();
        // A had 3 chain edges + 1 recovered; B had 2 chain edges + 1 recovered.
        assert_eq!(wa.coedges.len(), 4);
        assert_eq!(wb.coedges.len(), 3);
        // Insertion happened at the wrap-around position (end of chain).
        assert!(wa.coedges[3].forward);
        assert!(!wb.coedges[2].forward);

        // PCURVE contract: attached PCURVEs must be identity-consistent
        // (curve_2d.point_at(t) at the edge's own curve parameters lands
        // on the face's surface at the same 3D point — the mesh edge
        // cache's validation). PCURVEs that fail the contract are
        // dropped at recovery time and the mesh falls back to surface
        // projection, so a present PCURVE MUST validate; an absent one
        // is equally acceptable (projection path).
        let (t0, t1) = e.param_range;
        let curve3d = e.curve.clone().unwrap();
        for (coedge, surface) in [
            (&wa.coedges[3], Surface::Plane(plane.clone())),
            (&wb.coedges[2], Surface::Cylinder(cyl.clone())),
        ] {
            if let Some(c2d) = &coedge.curve_2d {
                for k in 0..=4 {
                    let t = t0 + (t1 - t0) * (k as f64 / 4.0);
                    let uv = c2d.point_at(t);
                    let p3 = surface.point_at(uv.u, uv.v);
                    let expect = curve3d.point_at(t);
                    assert!(
                        p3.distance_to(&expect) < 1e-6,
                        "attached PCURVE off by {} at t={t}",
                        p3.distance_to(&expect)
                    );
                }
            }
        }

        assert!(max_wire_opening(&shell, &working) < 2.0 * test_params().gap_tolerance);
    }

    /// Cylinder×Cylinder (parallel axes) → two exact LINE branches; the
    /// branch nearest BOTH gap endpoints wins, the far one is rejected
    /// by the projection tolerance.
    #[test]
    fn test_branch_selection_two_lines() {
        let h = (1.0 - 0.75_f64 * 0.75_f64).sqrt(); // 0.6614…
        let p_lo = Point3d::new(0.75, h, 0.0);
        let p_hi = Point3d::new(0.75, h, 2.0);

        // Face A: cylinder r=1, axis Z at origin — gap p_lo → p_hi.
        let cyl_a = CylinderSurface::new_z(1.0);
        let face_a = chain_face(
            Surface::Cylinder(cyl_a),
            &[
                Point3d::new(0.75, h, 2.0),
                Point3d::new(1.0, 0.0, 2.0),
                Point3d::new(1.0, 0.0, 0.0),
                Point3d::new(0.75, h, 0.0),
            ],
        );
        // Face B: cylinder r=1, axis Z at (1.5, 0, 0) — reversed gap.
        let cyl_b = CylinderSurface::new(
            Point3d::new(1.5, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let face_b = chain_face(
            Surface::Cylinder(cyl_b),
            &[
                Point3d::new(0.75, h, 0.0),
                Point3d::new(1.5, 1.0, 1.0),
                Point3d::new(0.75, h, 2.0),
            ],
        );
        let (mut shell, mut working) = shell_from(vec![face_a, face_b]);

        let report = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(report.edges_recovered, 1, "messages: {:?}", report.messages);

        // The recovered line lies on the NEAR branch (y = +h), not the
        // far one (y = -h).
        let e = working[0].last().unwrap();
        assert!(matches!(e.curve, Some(Curve3d::Line(_))));
        let mid = e.point_at(0.5).unwrap();
        assert!(
            mid.distance_to(&Point3d::new(0.75, h, 1.0)) < 1e-6,
            "mid: {mid:?} (expected y=+{h})"
        );
        assert!(e.start_vertex_point.unwrap().distance_to(&p_lo) < 1e-9);
        assert!(e.end_vertex_point.unwrap().distance_to(&p_hi) < 1e-9);
    }

    /// A gap with no partner gap in any other face is left alone.
    #[test]
    fn test_no_recovery_without_partner() {
        let face = chain_face(
            Surface::Plane(Plane::from_origin_and_normal(
                Point3d::ORIGIN,
                Direction3d::Z,
            )),
            &[
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(1.0, 0.0, 0.0),
                Point3d::new(1.0, 1.0, 0.0),
            ],
        );
        let (mut shell, mut working) = shell_from(vec![face]);
        let n_before = working[0].len();

        let report = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(report.gaps_detected, 1);
        assert_eq!(report.edges_recovered, 0);
        assert_eq!(shell.faces[0].outer_wire.as_ref().unwrap().coedges.len(), 2);
        assert_eq!(working[0].len(), n_before);
    }

    /// Recovery is idempotent: the output has no detectable gaps, so a
    /// second run recovers nothing and changes nothing.
    #[test]
    fn test_recovery_idempotent() {
        let (mut shell, mut working) = broken_box();
        let r1 = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(r1.edges_recovered, 1);

        let n_edges: Vec<usize> = working.iter().map(|l| l.len()).collect();
        let r2 = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(r2.gaps_detected, 0);
        assert_eq!(r2.edges_recovered, 0);
        assert_eq!(working.iter().map(|l| l.len()).collect::<Vec<_>>(), n_edges);
    }

    /// Same input → structurally identical output (curve kind, trimmed
    /// range, endpoints, orientation) — the determinism contract.
    #[test]
    fn test_recovery_deterministic() {
        fn signature(shell: &Shell, working: &[Vec<Edge>]) -> (String, (f64, f64), Point3d, Point3d, bool, bool) {
            let e = working[0].last().unwrap();
            let kind = match &e.curve {
                Some(Curve3d::Line(_)) => "line",
                Some(Curve3d::Circle(_)) => "circle",
                _ => "other",
            };
            let b_fwd = shell.faces[2].outer_wire.as_ref().unwrap().coedges[3].forward;
            (
                kind.to_string(),
                e.param_range,
                e.start_vertex_point.unwrap(),
                e.end_vertex_point.unwrap(),
                e.forward,
                b_fwd,
            )
        }

        let (mut shell1, mut working1) = broken_box();
        recover_lost_edges(&mut shell1, &mut working1, &test_params());
        let (mut shell2, mut working2) = broken_box();
        recover_lost_edges(&mut shell2, &mut working2, &test_params());

        assert_eq!(signature(&shell1, &working1), signature(&shell2, &working2));
    }

    /// End-to-end through the healing pipeline: `heal_solid` recovers
    /// the lost edge and rebuilds the store with it.
    #[test]
    fn test_heal_solid_recovers_lost_edge() {
        let (shell, working) = broken_box();
        let broken = Solid::from_edges_only(shell, working);

        let params = HealingParams {
            fix_normals: false,
            ..HealingParams::default()
        };
        let (healed, report) = heal_solid(&broken, &params);
        assert_eq!(report.edges_recovered, 1, "messages: {:?}", report.messages);

        let sh = healed.outer_shell.as_ref().expect("healed outer shell");
        assert_eq!(sh.faces[0].outer_wire.as_ref().unwrap().coedges.len(), 4);
        assert_eq!(sh.faces[2].outer_wire.as_ref().unwrap().coedges.len(), 4);

        // The recovered edge is resolvable in the store with the exact
        // lost-edge endpoints.
        let a = Point3d::new(-1.0, -1.0, -1.0);
        let b = Point3d::new(1.0, -1.0, -1.0);
        let bottom_edges = healed.resolve_face_edges(&sh.faces[0]);
        assert!(bottom_edges.iter().any(|e| {
            match effective_endpoints(e) {
                Some((s, t)) => {
                    (s.distance_to(&a) < 1e-9 && t.distance_to(&b) < 1e-9)
                        || (s.distance_to(&b) < 1e-9 && t.distance_to(&a) < 1e-9)
                }
                None => false,
            }
        }), "recovered edge not found in the healed bottom face");
    }

    // ============================================================
    // Loop-level recovery (closed lost edges)
    // ============================================================

    /// A cylinder (R=1, height 2) whose bottom cap circle edge is
    /// LOST: the cap face's wire is EMPTY and its working list is
    /// empty; the lateral face keeps the realistic seam-twice wire
    /// [seam forward, top circle, seam reversed] — the B0 junction
    /// (the seam's own start/end point, wrap position) is the
    /// degenerate gap where the bottom circle belongs.
    fn broken_cylinder_cap() -> (Shell, Vec<Vec<Edge>>) {
        // Cap face: plane z=0, empty wire, no edges.
        let cap_face = Face::new(
            Surface::Plane(Plane::from_origin_and_normal(
                Point3d::ORIGIN,
                Direction3d::Z,
            )),
            Wire::new(vec![]),
        );

        // Lateral face: cylinder R=1 axis Z; the seam edge is
        // referenced twice (forward then reversed — the native STEP
        // pattern), so no pass merges it away.
        let seam = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 2.0));
        let top_circle = Edge::new(
            Curve3d::Circle(Circle::new_xy(Point3d::new(0.0, 0.0, 2.0), 1.0)),
            (0.0, 2.0 * std::f64::consts::PI),
        );
        let lateral_wire = Wire::new(vec![
            CoEdge::new(seam.id, true),
            CoEdge::new(top_circle.id, true),
            CoEdge::new(seam.id, false),
        ]);
        let lateral_face = Face::new(Surface::Cylinder(CylinderSurface::new_z(1.0)), lateral_wire);

        (
            Shell::new(vec![cap_face, lateral_face]),
            vec![vec![], vec![seam, top_circle]],
        )
    }

    /// Lost CLOSED edge (full circle capping a cylinder): the cap's
    /// wire is empty, the lateral face's B0 junction is the degenerate
    /// gap. Loop-level recovery reconstructs the exact circle,
    /// re-parameterized to the junction vertex, with identity PCURVEs
    /// on both surfaces.
    #[test]
    fn test_recover_lost_closed_circle_cylinder_cap() {
        let (mut shell, mut working) = broken_cylinder_cap();
        let report = recover_lost_edges(&mut shell, &mut working, &test_params());

        assert_eq!(report.gaps_detected, 0);
        assert_eq!(report.loops_detected, 1, "messages: {:?}", report.messages);
        assert_eq!(report.loops_recovered, 1, "messages: {:?}", report.messages);
        assert_eq!(report.edges_recovered, 1);

        // The cap's empty wire received ONE coedge — reversed (the
        // manifold opposite of the lateral traversal) — and is a
        // closed loop.
        let cap_wire = shell.faces[0].outer_wire.as_ref().unwrap();
        assert_eq!(cap_wire.coedges.len(), 1);
        assert!(!cap_wire.coedges[0].forward);
        assert!(cap_wire.closed);

        // The lateral face's wire: seam, top circle, seam reversed,
        // plus the recovered circle at the B0 junction (wrap position).
        let lat_wire = shell.faces[1].outer_wire.as_ref().unwrap();
        assert_eq!(lat_wire.coedges.len(), 4);
        assert!(lat_wire.coedges[3].forward);

        // The recovered edge is the exact full circle, re-parameterized
        // to the junction vertex (t_v = 0: B0 IS the branch circle's
        // frame origin).
        let e = working[0].last().unwrap().clone();
        assert!(matches!(e.curve, Some(Curve3d::Circle(_))), "curve: {:?}", e.curve);
        assert!(e.param_range.0.abs() < 1e-9, "t0: {}", e.param_range.0);
        assert!(
            (e.param_range.1 - 2.0 * std::f64::consts::PI).abs() < 1e-9,
            "t1: {}",
            e.param_range.1
        );
        let b0 = Point3d::new(1.0, 0.0, 0.0);
        assert!(e.start_vertex_point.unwrap().distance_to(&b0) < 1e-12);
        assert!(e.end_vertex_point.unwrap().distance_to(&b0) < 1e-12);
        assert!(e.forward);

        // Same edge id in both working lists (rebuild_store dedups).
        assert_eq!(working[0].len(), 1);
        assert_eq!(working[1].len(), 3);
        assert_eq!(working[0].last().unwrap().id, working[1].last().unwrap().id);

        // Identity-PCURVE contract over the FULL closed range (both
        // surfaces) when attached.
        let (t0, t1) = e.param_range;
        let curve3d = e.curve.clone().unwrap();
        let plane = Plane::from_origin_and_normal(Point3d::ORIGIN, Direction3d::Z);
        let cyl = CylinderSurface::new_z(1.0);
        for (coedge, surface) in [
            (&cap_wire.coedges[0], Surface::Plane(plane)),
            (&lat_wire.coedges[3], Surface::Cylinder(cyl)),
        ] {
            if let Some(c2d) = &coedge.curve_2d {
                for k in 0..=8 {
                    let t = t0 + (t1 - t0) * (k as f64 / 8.0);
                    let uv = c2d.point_at(t);
                    let p3 = surface.point_at(uv.u, uv.v);
                    let expect = curve3d.point_at(t);
                    assert!(
                        p3.distance_to(&expect) < 1e-6,
                        "attached PCURVE off by {} at t={t}",
                        p3.distance_to(&expect)
                    );
                }
            }
        }

        // Walk closure: no open junctions remain anywhere.
        assert!(max_wire_opening(&shell, &working) < 2.0 * test_params().gap_tolerance);
    }

    /// The native cylinder's wire-less lateral face is the SANCTIONED
    /// empty-wire representation (its circles are shared with the disk
    /// faces' wires) — the orphan filter must not flag it, and nothing
    /// is recovered on a healthy solid.
    #[test]
    fn test_closed_loop_native_cylinder_not_recovered() {
        let solid = ShapeBuilder::make_cylinder(1.0, 2.0);
        let mut shell = solid.outer_shell.clone().expect("cylinder outer shell");
        let mut working: Vec<Vec<Edge>> = shell
            .faces
            .iter()
            .map(|f| solid.resolve_face_edges(f))
            .collect();

        let report = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(report.gaps_detected, 0);
        assert_eq!(report.loops_detected, 0, "messages: {:?}", report.messages);
        assert_eq!(report.loops_recovered, 0);
        assert_eq!(report.edges_recovered, 0);

        // Nothing changed.
        assert!(shell.faces[2].outer_wire.as_ref().unwrap().coedges.is_empty());
        assert_eq!(working[2].len(), 2);
    }

    /// The cap's wire is empty, but the bottom circle edge SURVIVES in
    /// the lateral face's working list (orphaned — its ORIENTED_EDGEs
    /// were dropped from both wires): the closed existing-edge guard
    /// must reject recovery instead of duplicating the edge.
    #[test]
    fn test_closed_loop_surviving_circle_guard() {
        let cap_face = Face::new(
            Surface::Plane(Plane::from_origin_and_normal(
                Point3d::ORIGIN,
                Direction3d::Z,
            )),
            Wire::new(vec![]),
        );
        let seam = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 2.0));
        let lateral_face = Face::new(
            Surface::Cylinder(CylinderSurface::new_z(1.0)),
            Wire::new(vec![CoEdge::new(seam.id, true), CoEdge::new(seam.id, false)]),
        );
        // The surviving bottom circle — orphaned in the working list.
        let bottom_circle = Edge::new(
            Curve3d::Circle(Circle::new_xy(Point3d::ORIGIN, 1.0)),
            (0.0, 2.0 * std::f64::consts::PI),
        );
        let (mut shell, mut working) = shell_from(vec![
            (cap_face, vec![]),
            (lateral_face, vec![seam, bottom_circle]),
        ]);

        let report = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(report.loops_detected, 1);
        assert_eq!(report.loops_recovered, 0, "messages: {:?}", report.messages);
        assert_eq!(report.edges_recovered, 0);
        assert_eq!(shell.faces[1].outer_wire.as_ref().unwrap().coedges.len(), 2);
        assert_eq!(working[1].len(), 2);
    }

    /// The lateral wire's seam lines end 2e-6 apart at B0 (a tolerant
    /// junction, within gap tolerance): the junction still qualifies,
    /// and the vertex-point overrides keep each flank's OWN endpoint
    /// (bit-identical discretization on both sides of the tolerant
    /// junction).
    #[test]
    fn test_closed_loop_tolerant_junction() {
        let cap_face = Face::new(
            Surface::Plane(Plane::from_origin_and_normal(
                Point3d::ORIGIN,
                Direction3d::Z,
            )),
            Wire::new(vec![]),
        );
        let seam_a = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 2.0));
        let top_circle = Edge::new(
            Curve3d::Circle(Circle::new_xy(Point3d::new(0.0, 0.0, 2.0), 1.0)),
            (0.0, 2.0 * std::f64::consts::PI),
        );
        let seam_b = Edge::new_line(
            Point3d::new(1.0, 0.0, 2.0),
            Point3d::new(1.0, 2.0e-6, 0.0),
        );
        let lateral_face = Face::new(
            Surface::Cylinder(CylinderSurface::new_z(1.0)),
            Wire::new(vec![
                CoEdge::new(seam_a.id, true),
                CoEdge::new(top_circle.id, true),
                CoEdge::new(seam_b.id, true),
            ]),
        );
        let (mut shell, mut working) = shell_from(vec![
            (cap_face, vec![]),
            (lateral_face, vec![seam_a, top_circle, seam_b]),
        ]);

        let report = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(report.loops_recovered, 1, "messages: {:?}", report.messages);

        let e = working[0].last().unwrap();
        // arrive = seam_b's end; depart = seam_a's start.
        assert!(
            e.start_vertex_point
                .unwrap()
                .distance_to(&Point3d::new(1.0, 2.0e-6, 0.0)) < 1e-12
        );
        assert!(
            e.end_vertex_point
                .unwrap()
                .distance_to(&Point3d::new(1.0, 0.0, 0.0)) < 1e-12
        );
        // The periodic domain rotated to the arrive flank's projection
        // (angle of (1, 2e-6) ≈ 2e-6 rad).
        assert!(
            (e.param_range.0 - 2.0e-6).abs() < 1e-7,
            "t0: {}",
            e.param_range.0
        );
        assert!(
            (e.param_range.1 - e.param_range.0 - 2.0 * std::f64::consts::PI).abs() < 1e-9
        );
    }

    /// Loop-level recovery is idempotent: the recovered coedge fills
    /// the empty wire, so a second run detects no candidates.
    #[test]
    fn test_closed_loop_recovery_idempotent() {
        let (mut shell, mut working) = broken_cylinder_cap();
        let r1 = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(r1.loops_recovered, 1);

        let r2 = recover_lost_edges(&mut shell, &mut working, &test_params());
        assert_eq!(r2.loops_detected, 0);
        assert_eq!(r2.loops_recovered, 0);
        assert_eq!(r2.edges_recovered, 0);
        assert_eq!(shell.faces[0].outer_wire.as_ref().unwrap().coedges.len(), 1);
        assert_eq!(shell.faces[1].outer_wire.as_ref().unwrap().coedges.len(), 4);
        assert_eq!(working[0].len(), 1);
        assert_eq!(working[1].len(), 3);
    }

    /// Same input → structurally identical output (curve kind,
    /// rotated range, vertex points, orientations, wire sizes) — the
    /// loop-level determinism contract.
    #[test]
    fn test_closed_loop_recovery_deterministic() {
        fn signature(
            shell: &Shell,
            working: &[Vec<Edge>],
        ) -> (String, (f64, f64), Point3d, Point3d, bool, bool, bool, usize, usize) {
            let e = working[0].last().unwrap();
            let kind = match &e.curve {
                Some(Curve3d::Circle(_)) => "circle",
                Some(Curve3d::Ellipse(_)) => "ellipse",
                Some(Curve3d::Line(_)) => "line",
                _ => "other",
            };
            let cap_fwd = shell.faces[0].outer_wire.as_ref().unwrap().coedges[0].forward;
            let lat = shell.faces[1].outer_wire.as_ref().unwrap();
            let lat_fwd = lat.coedges[3].forward;
            (
                kind.to_string(),
                e.param_range,
                e.start_vertex_point.unwrap(),
                e.end_vertex_point.unwrap(),
                e.forward,
                cap_fwd,
                lat_fwd,
                lat.coedges.len(),
                working[1].len(),
            )
        }

        let (mut shell1, mut working1) = broken_cylinder_cap();
        recover_lost_edges(&mut shell1, &mut working1, &test_params());
        let (mut shell2, mut working2) = broken_cylinder_cap();
        recover_lost_edges(&mut shell2, &mut working2, &test_params());

        assert_eq!(signature(&shell1, &working1), signature(&shell2, &working2));
    }

    /// End-to-end through the healing pipeline: `heal_solid` recovers
    /// the lost closed cap circle and rebuilds the store with it.
    #[test]
    fn test_heal_solid_recovers_lost_closed_loop() {
        let (shell, working) = broken_cylinder_cap();
        let broken = Solid::from_edges_only(shell, working);

        let params = HealingParams {
            fix_normals: false,
            stitch_edges: false,
            merge_faces: false,
            ..HealingParams::default()
        };
        let (healed, report) = heal_solid(&broken, &params);
        assert_eq!(report.edges_recovered, 1, "messages: {:?}", report.messages);

        let sh = healed.outer_shell.as_ref().expect("healed outer shell");
        assert_eq!(sh.faces[0].outer_wire.as_ref().unwrap().coedges.len(), 1);
        assert_eq!(sh.faces[1].outer_wire.as_ref().unwrap().coedges.len(), 4);

        // The recovered circle is resolvable in the store with the
        // seam vertex as its (closed) endpoints.
        let b0 = Point3d::new(1.0, 0.0, 0.0);
        let cap_edges = healed.resolve_face_edges(&sh.faces[0]);
        assert!(
            cap_edges.iter().any(|e| {
                matches!(e.curve, Some(Curve3d::Circle(_)))
                    && e.start_point().map(|p| p.distance_to(&b0) < 1e-9).unwrap_or(false)
                    && e.end_point().map(|p| p.distance_to(&b0) < 1e-9).unwrap_or(false)
            }),
            "recovered circle not found in the healed cap face"
        );
    }
}
