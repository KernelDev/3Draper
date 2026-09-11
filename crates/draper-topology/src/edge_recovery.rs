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
//! # Limitations (documented, by design)
//!
//! - Only OPEN gaps are recovered. A lost CLOSED edge (a full circle
//!   capping a cylinder, where the "gap" degenerates to a point and
//!   the capped face's wire becomes empty) is not detected here; that
//!   defect needs loop-level recovery (future work).
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
    /// Number of lost edges reconstructed via SSI.
    pub edges_recovered: u32,
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
/// surface-surface intersection (Vision 2036 §1.4).
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
    if gaps.is_empty() {
        return report;
    }

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
    use draper_geometry::{Circle2d, CylinderSurface, Direction3d, Line2d, Plane};

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
}
