// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Custom Constrained Delaunay Triangulation (CDT) implementation.
//!
//! Guarantees:
//! - ALL input vertices appear as triangle vertices in the output
//! - ALL constraint edges appear as edges of triangles in the output
//! - The triangulation is Delaunay except where constraint edges prevent it
//!
//! Algorithm (two-phase):
//! 1. Boundary phase: Use earcutr (mapbox/earcut) to triangulate the
//!    boundary polygon with holes. earcutr guarantees that all boundary
//!    vertices appear as triangle vertices and all boundary edges are
//!    preserved - by construction, ear-clipping cannot skip a boundary vertex.
//! 2. Interior phase: Insert interior Steiner points using Bowyer-Watson
//!    point insertion. Each insertion preserves existing edges (including
//!    constraint edges), so boundary edge integrity is maintained.
//! 3. Delaunay improvement: After all insertions, apply Lawson flips
//!    to improve triangle quality while respecting constraints.

// Production module since 2026-09-09: `triangulate_surface_consistent` and
// `triangulate_cdt` route interior Steiner points through
// `triangulate_polygon_cdt` (see the HOUSING #47598 regression).

use std::collections::{HashMap, HashSet};

/// Tolerance for geometric comparisons.
const EPS: f64 = 1e-10;

/// Triangulate a polygon with holes and optional interior Steiner points
/// using Constrained Delaunay Triangulation.
///
/// All boundary vertices are guaranteed to appear as triangle vertices.
/// All boundary edges are guaranteed to appear as triangle edges.
/// Interior Steiner points are properly integrated for surface approximation.
///
/// Returns triangle indices referencing the combined vertex array:
/// [boundary][hole0][hole1]...[interior]
pub fn triangulate_polygon_cdt(
    boundary_2d: &[[f64; 2]],
    holes_2d: &[Vec<[f64; 2]>],
    interior_2d: &[[f64; 2]],
) -> Vec<[u32; 3]> {
    let n_boundary = boundary_2d.len();
    if n_boundary < 3 {
        return Vec::new();
    }

    // Collect all 2D points: boundary + holes + interior
    let mut all_2d: Vec<[f64; 2]> = boundary_2d.to_vec();
    let mut hole_index_ranges: Vec<(usize, usize)> = Vec::new();
    for hole in holes_2d {
        let start = all_2d.len();
        all_2d.extend_from_slice(hole);
        hole_index_ranges.push((start, all_2d.len()));
    }
    let interior_start = all_2d.len();
    all_2d.extend_from_slice(interior_2d);

    // Triangulate boundary + holes using earcutr
    let mut coords: Vec<f64> = Vec::with_capacity(all_2d.len() * 2);
    for p in &all_2d[..interior_start] {
        coords.push(p[0]);
        coords.push(p[1]);
    }

    let mut earcut_hole_indices: Vec<usize> = Vec::new();
    for &(start, _end) in &hole_index_ranges {
        earcut_hole_indices.push(start);
    }

    let earcut_result = crate::earcut_adapter::triangulate_polygon_with_holes(&coords, &earcut_hole_indices);

    let mut triangles: Vec<[u32; 3]> = earcut_result.chunks(3)
        .filter_map(|chunk| {
            if chunk.len() < 3 {
                return None;
            }
            let a = chunk[0] as u32;
            let b = chunk[1] as u32;
            let c = chunk[2] as u32;
            if a == b || b == c || a == c {
                return None;
            }
            Some([a, b, c])
        })
        .collect();

    if triangles.is_empty() {
        return Vec::new();
    }

    // Repair boundary vertices that earcutr dropped (collinear rim /
    // hole points are skipped by ear-clipping — zero-area ears). Each
    // dropped vertex leaves its two ring edges missing from the
    // triangulation: the neighboring face still uses the vertex (it
    // comes from the shared edge cache), producing boundary edges in
    // the merged BREP mesh. The repair re-inserts every unused ring
    // vertex by splitting the covering edge.
    repair_unused_ring_vertices(&all_2d, &mut triangles, n_boundary, &hole_index_ranges);

    // session-64: Delaunay improvement (Lawson flips) — re-enabled with
    // the convexity guard inside lawson_flip (opposite-sides test; the
    // 2026-09-09 disable was for exactly the missing guard). Base
    // quality matters: ear-clip triangulations of elongated L-shaped
    // polygons contain long diagonal chords (Zentralstaender #1092 f29:
    // chords of UV length 12 spanning the whole stepped band) whose
    // adjacent triangles are 300:1 tents — the ~180° dihedral "folds"
    // the probe reports, and any Steiner insertion near such a chord
    // degenerates. Flipping toward Delaunay before the insertion breaks
    // the chords into well-shaped triangles; constraint (ring) edges
    // are never flipped. A second round after the insertion cleans up
    // the fan products.
    lawson_flip(&all_2d, &mut triangles, n_boundary, &hole_index_ranges);

    // Insert interior Steiner points using Bowyer-Watson
    if !interior_2d.is_empty() {
        insert_interior_points(
            &all_2d,
            &mut triangles,
            interior_start,
            interior_2d.len(),
            n_boundary,
            &hole_index_ranges,
        );
    }

    // session-64: post-insertion Delaunay round (same guarded flips) —
    // DISABLED after measurement: flipping around the inserted fan
    // triangles introduced 276 winding conflicts + 5 non-manifold edges
    // on #1092 f29 (the pre-insertion round alone is clean). The fan
    // products' local structure interacts badly with the flip winding
    // slots; re-enable only after a winding-preserving flip rewrite.
    // lawson_flip(&all_2d, &mut triangles, n_boundary, &hole_index_ranges);

    // Delaunay improvement (Lawson flips) — history: disabled 2026-09-09
    // for the missing quad-convexity guard (non-convex quads flipped to
    // overlapping triangles, stress test
    // `test_cdt_with_hole_and_grid_no_gaps`); re-enabled session-64
    // with the opposite-sides convexity guard inside lawson_flip.

    // Verify constraint edges exist (debug only)
    #[cfg(debug_assertions)]
    verify_constraints(&triangles, n_boundary, &hole_index_ranges);

    triangles
}

/// Insert interior Steiner points into an existing triangulation
/// using Bowyer-Watson point insertion.
///
/// Uses an edge-to-triangle adjacency map for O(1) neighbor lookups
/// instead of O(n) linear search.
fn insert_interior_points(
    all_2d: &[[f64; 2]],
    triangles: &mut Vec<[u32; 3]>,
    interior_start: usize,
    n_interior: usize,
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
) {
    // Build edge-to-triangle adjacency map for fast neighbor lookups
    let mut edge_map = build_edge_map(triangles);

    // RING EDGE PROTECTION (Vision 2036 watertightness / HOUSING #47598):
    // the outer rim and hole rings are CROSS-FACE contracts — the
    // neighboring face discretizes the shared topological edge into the
    // same bit-identical vertices, and every rim edge must survive this
    // face's triangulation exactly as (v_i, v_{i+1}). Splitting a ring
    // edge to insert a Steiner point would replace it with (v_i, s) +
    // (s, v_{i+1}) — a pair the neighbor does not have — punching 3
    // boundary edges into the merged BREP mesh. A Steiner landing on a
    // ring edge is redundant anyway (the rim already discretizes the
    // curve there), so the point is simply skipped.
    let ring_edges: HashSet<(u32, u32)> = build_constraint_set(n_boundary, hole_ranges);

    // s81 diagnostics: drop-reason census (env-gated)
    let dbg = std::env::var("DRAPPER_CDT_DEBUG").is_ok();
    let (mut n_ring_skip, mut n_sliver_skip, mut n_not_found) = (0usize, 0usize, 0usize);

    // session-82 LATTICE-AWARE GUARD MODE (experimental knob — the
    // default `s64` keeps the session-64 behavior bit-identical):
    //   DRAPPER_CDT_GUARD_MODE=s64     — both sliver guards always (s64)
    //   DRAPPER_CDT_GUARD_MODE=off     — no sliver guard (pre-s64)
    //   DRAPPER_CDT_GUARD_MODE=lattice — guards gated on a LONG edge
    //                                     (> LONG_EDGE_STEPS x the
    //                                     median lattice NN step)
    // The s64 thresholds were calibrated for SPARSE Steiner points
    // near long earcut chords (Z #1092 f29/f32). On DENSE interior
    // lattices (fillet/sail, 300+ points) they misfire as a CASCADE:
    // an inserted point's fan edges sit a hair from the next lattice
    // point of the same (near-)collinear row -> min_prod ~ eps ->
    // skip -> the hole grows -> more edge-adjacent points trip (s81
    // f26: 169/341 dropped, overlap 15.6%). The tents the guard kills
    // all have a LONG edge dominating the local scale; on a uniform
    // lattice no local edge exceeds a few lattice steps.
    let guard_mode = std::env::var("DRAPPER_CDT_GUARD_MODE")
        .unwrap_or_else(|_| "s64".to_string());
    let guard_mode = if guard_mode.is_empty() {
        "s64".to_string()
    } else {
        guard_mode
    };
    let lattice_guard = guard_mode == "lattice";
    let guard_off = guard_mode == "off";
    // s82 split-fallback switch. MEASURED DEFAULT OFF: the fallback
    // (split the hugged long edge instead of skipping) inserts
    // ~1000:1 UV strips — cleaner than the s64 skip's coarse residue
    // in UV terms, but they FOLD in 3D: drill HM 1315/229 → 1435/297,
    // (26,26) 25 → 67 REAL. The single-point fallback cannot make the
    // long-chord region healthy — that needs STRUCTURAL row insertion
    // (the s78 SAIL_BAND approach), s83 material. Opt-in
    // DRAPPER_CDT_SPLIT_FALLBACK=1 for reproduction.
    let split_fallback =
        std::env::var("DRAPPER_CDT_SPLIT_FALLBACK").as_deref() == Ok("1");
    // median nearest-neighbor distance among the interior points =
    // the lattice step estimate (O(n^2), capped — the corpus faces
    // sit well under 1k interior points). Computed in every mode:
    // the s82 split fallback needs it to define a LONG edge.
    let lattice_step = if (lattice_guard || split_fallback) && n_interior >= 2 && n_interior <= 2000 {
        let mut nn: Vec<f64> = Vec::with_capacity(n_interior);
        for i in 0..n_interior {
            let pi = all_2d[(interior_start + i) as usize];
            let mut best = f64::INFINITY;
            for j in 0..n_interior {
                if i == j {
                    continue;
                }
                let pj = all_2d[(interior_start + j) as usize];
                let dx = pi[0] - pj[0];
                let dy = pi[1] - pj[1];
                let d2 = dx * dx + dy * dy;
                if d2 < best {
                    best = d2;
                }
            }
            nn.push(best.sqrt());
        }
        nn.sort_by(|a, b| a.partial_cmp(b).unwrap());
        nn[nn.len() / 2]
    } else {
        0.0
    };
    const LONG_EDGE_STEPS: f64 = 4.0;
    let long_edge_sq = if lattice_guard && lattice_step > 0.0 {
        let k = LONG_EDGE_STEPS * lattice_step;
        k * k
    } else {
        f64::INFINITY
    };
    // s82: an edge worth splitting in the fallback (> LONG_SPLIT_STEPS
    // lattice steps — measured on f26's tripping parents: max edges
    // 4–17 steps; the post-refinement healthy edges sit at 1–2 steps).
    const LONG_SPLIT_STEPS: f64 = 4.0;
    let mut n_split_fallback = 0usize;

    for i in 0..n_interior {
        let point_idx = (interior_start + i) as u32;
        let p = all_2d[point_idx as usize];

        match find_containing_triangle(all_2d, triangles, p) {
            Some((tri_idx, on_edge)) => {
                if on_edge {
                    // Determine the edge the point sits on (the same
                    // min-|orient2d| choice insert_point_on_edge_fast
                    // makes) and skip the insertion when it is a ring
                    // edge — see the protection note above.
                    let edge = nearest_triangle_edge(all_2d, triangles[tri_idx], p);
                    let is_ring = edge
                        .map(|(v1, v2)| ring_edges.contains(&(v1.min(v2), v1.max(v2))))
                        .unwrap_or(false);
                    if is_ring {
                        n_ring_skip += 1;
                        continue; // redundant with the rim — skip
                    }
                    insert_point_on_edge_fast(all_2d, triangles, tri_idx, point_idx, &mut edge_map);
                } else {
                    // session-64 SLIVER GUARDS: an interior lattice point
                    // landing within a hair of the containing triangle's
                    // edge produces a fan product that is a 100:1+ sliver
                    // whose 3D normal is noise-dominated — adjacent
                    // slivers show ~180° dihedrals = the probe's
                    // FOLD-OVER pairs (Zentralstaender #1092 f29/f32).
                    // Interior Steiner points are NOT a cross-face
                    // contract (only the ring is) — skipping a
                    // degenerate insertion is always watertight-safe
                    // and costs a negligible deviation increase.
                    //
                    // Guard 1 (relative): the smallest fan product under
                    // 5% of the parent's area.
                    // Guard 2 (aspect, scale-free): the smallest product
                    // under 0.005 · (its longest edge)² — a 200:1 sliver.
                    // Guard 2 catches the case guard 1 misses: a thin
                    // PARENT (an earlier product hugging a long chord —
                    // the 10-long column chord (b63, b32)) makes the
                    // ratio look healthy while the product is still a
                    // tent flap (aspect ~5e-4).
                    let [a, b, c] = triangles[tri_idx];
                    let pa = all_2d[a as usize];
                    let pb = all_2d[b as usize];
                    let pc = all_2d[c as usize];
                    let parent_area = orient2d(pa, pb, pc).abs();
                    let prod = |q: [f64; 2], r: [f64; 2]| orient2d(p, q, r).abs();
                    let min_prod = prod(pa, pb).min(prod(pb, pc)).min(prod(pc, pa));
                    const MIN_FAN_PRODUCT_FRAC: f64 = 0.05;
                    const MIN_FAN_ASPECT: f64 = 0.005;
                    let edge_sq = |q: [f64; 2], r: [f64; 2], s: [f64; 2]| {
                        let e1 = (q[0] - r[0]) * (q[0] - r[0]) + (q[1] - r[1]) * (q[1] - r[1]);
                        let e2 = (r[0] - s[0]) * (r[0] - s[0]) + (r[1] - s[1]) * (r[1] - s[1]);
                        let e3 = (s[0] - q[0]) * (s[0] - q[0]) + (s[1] - q[1]) * (s[1] - q[1]);
                        e1.max(e2).max(e3)
                    };
                    let max_edge_sq = edge_sq(pa, pb, pc).max(edge_sq(p, pb, pc))
                        .max(edge_sq(pa, p, pc)).max(edge_sq(pa, pb, p));
                    let skip_relative = !guard_off
                        && parent_area > 0.0
                        && min_prod < MIN_FAN_PRODUCT_FRAC * parent_area;
                    // s82 lattice mode: the guards fire ONLY when the
                    // local neighborhood carries a LONG edge (> 4
                    // lattice steps) — the sparse-tent signature. On
                    // a uniform lattice every local edge is within a
                    // few steps, so insertions proceed.
                    let long_present = !lattice_guard || max_edge_sq > long_edge_sq;
                    let skip_aspect = !guard_off
                        && long_present
                        && max_edge_sq > 0.0
                        && min_prod < MIN_FAN_ASPECT * max_edge_sq;
                    let skip = (skip_relative && long_present) || skip_aspect;
                    if skip {
                        n_sliver_skip += 1;
                        // s82 diagnostics: per-skip geometry dump
                        if let Ok(path) = std::env::var("DRAPPER_CDT_SKIP_DUMP") {
                            use std::io::Write;
                            if let Ok(mut f) =
                                std::fs::OpenOptions::new().create(true).append(true).open(&path)
                            {
                                let _ = writeln!(
                                    f,
                                    "pt {} ({:.9},{:.9}) min_prod {:.3e} parent {:.3e} max_edge2 {:.3e} rel={} asp={} parent=[{},{},{}]",
                                    point_idx,
                                    p[0],
                                    p[1],
                                    min_prod,
                                    parent_area,
                                    max_edge_sq,
                                    skip_relative,
                                    skip_aspect,
                                    a,
                                    b,
                                    c
                                );
                            }
                        }
                        log::debug!(
                            "insert_interior_points: sliver guard — point {} min product {:.3e} (parent {:.3e}, max_edge² {:.3e}) skipped",
                            point_idx, min_prod, parent_area, max_edge_sq
                        );
                        // session-82 LONG-CHORD EDGE-SPLIT FALLBACK:
                        // a SKIP is only safe when the containing
                        // structure is final — on a DENSE lattice the
                        // skip deadlocks the refinement exactly where
                        // it is needed (long earcut/Delaunay chords:
                        // every lattice point inside a long-chord
                        // parent trips the aspect guard, the chord
                        // never gets split, the coarse triangle folds
                        // against the refined neighborhood — s82 f26:
                        // 205/341 dropped, the surviving 25 (26,26)
                        // REAL pairs are the UNSPLIT big triangles).
                        // Instead of skipping, SPLIT the edge the
                        // point hugs: the sub-edges halve, the new
                        // triangles carry the parent's height (healthy
                        // aspect), and the next lattice row refines
                        // further. Conditions: (a) not a ring edge
                        // (cross-face contract), (b) the projection
                        // lands strictly inside (both sub-edges ≥ 20%
                        // of the original — a point hugging a VERTEX
                        // must still skip), (c) the edge is long
                        // (> LONG_SPLIT_STEPS lattice steps — a short
                        // edge is already at the refinement scale, the
                        // split adds nothing a skip wouldn't).
                        if !guard_off && split_fallback {
                            let nearest = nearest_triangle_edge(all_2d, triangles[tri_idx], p);
                            if let Some((e1, e2)) = nearest {
                                let is_ring = ring_edges.contains(&(e1.min(e2), e1.max(e2)));
                                let pe1 = all_2d[e1 as usize];
                                let pe2 = all_2d[e2 as usize];
                                let ex = pe2[0] - pe1[0];
                                let ey = pe2[1] - pe1[1];
                                let len_sq = ex * ex + ey * ey;
                                let t = if len_sq > 0.0 {
                                    ((p[0] - pe1[0]) * ex + (p[1] - pe1[1]) * ey) / len_sq
                                } else {
                                    -1.0
                                };
                                let long_enough = lattice_step > 0.0 && {
                                    let k = LONG_SPLIT_STEPS * lattice_step;
                                    len_sq > k * k
                                };
                                // Child-quality floor: the split must
                                // not manufacture degenerate children.
                                // A needle parent (long edge, ~zero
                                // height) yields needle children —
                                // there the s64 SKIP is the right call.
                                // Require both children on this side
                                // to clear the needle census aspect
                                // (area ≥ 1e-3 · sub-edge²).
                                let child_ok = {
                                    let popp = triangles[tri_idx]
                                        .iter()
                                        .find(|&&v| v != e1 && v != e2)
                                        .map(|&v| all_2d[v as usize])
                                        .unwrap_or(p);
                                    let a1 = orient2d(popp, pe1, p).abs();
                                    let a2 = orient2d(popp, p, pe2).abs();
                                    let s1 = {
                                        let dx = pe1[0] - p[0];
                                        let dy = pe1[1] - p[1];
                                        dx * dx + dy * dy
                                    };
                                    let s2 = {
                                        let dx = pe2[0] - p[0];
                                        let dy = pe2[1] - p[1];
                                        dx * dx + dy * dy
                                    };
                                    a1 >= 2.0 * 1e-3 * s1 && a2 >= 2.0 * 1e-3 * s2
                                };
                                if !is_ring && t > 0.2 && t < 0.8 && long_enough && child_ok {
                                    n_split_fallback += 1;
                                    insert_point_on_edge_fast(
                                        all_2d,
                                        triangles,
                                        tri_idx,
                                        point_idx,
                                        &mut edge_map,
                                    );
                                    continue;
                                }
                            }
                        }
                        continue;
                    }
                    insert_point_in_triangle_fast(triangles, tri_idx, point_idx, &mut edge_map);
                }
            }
            None => {
                // Point is outside the triangulation - skip it
                n_not_found += 1;
                log::debug!("Interior point {} outside triangulation, skipping", point_idx);
            }
        }
    }
    if dbg && (n_ring_skip + n_sliver_skip + n_not_found + n_split_fallback) > 0 {
        eprintln!(
            "CDT_DEBUG: interior drops: ring_skip={} sliver_skip={} not_found={} split_fallback={} of {} (label={})",
            n_ring_skip,
            n_sliver_skip,
            n_not_found,
            n_split_fallback,
            n_interior,
            crate::parametric_domain::current_face_label()
        );
    }
}

/// The triangle edge nearest to point p (by |orient2d|), as a
/// (v1, v2) vertex pair — mirrors the edge choice of
/// `insert_point_on_edge_fast`.
fn nearest_triangle_edge(
    vertices: &[[f64; 2]],
    tri: [u32; 3],
    p: [f64; 2],
) -> Option<(u32, u32)> {
    let [a, b, c] = tri;
    let pa = vertices[a as usize];
    let pb = vertices[b as usize];
    let pc = vertices[c as usize];
    let d1 = orient2d(p, pa, pb).abs();
    let d2 = orient2d(p, pb, pc).abs();
    let d3 = orient2d(p, pc, pa).abs();
    Some(if d1 <= d2 && d1 <= d3 {
        (a, b)
    } else if d2 <= d3 {
        (b, c)
    } else {
        (c, a)
    })
}

/// Build a map from edge (min,max) → list of triangle indices that share that edge.
fn build_edge_map(triangles: &[[u32; 3]]) -> HashMap<(u32, u32), Vec<usize>> {
    let mut map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, tri) in triangles.iter().enumerate() {
        for i in 0..3 {
            let a = tri[i].min(tri[(i + 1) % 3]);
            let b = tri[i].max(tri[(i + 1) % 3]);
            map.entry((a, b)).or_default().push(ti);
        }
    }
    map
}

/// Re-insert boundary (outer rim + hole ring) vertices that earcutr
/// dropped from the triangulation.
///
/// ear-clipping skips collinear ring points (their ears have zero area),
/// so such vertices appear in no triangle and their two ring edges are
/// missing. Since production rims come from the shared edge cache (the
/// neighboring face DOES use those vertices), each dropped vertex becomes
/// dangling in the merged mesh → boundary edges.
///
/// Repair per unused ring vertex `p`:
/// 1. Find a triangle edge (a, b) whose segment geometrically contains p.
/// 2. Split (a, b) into (a, p) and (p, b) in BOTH adjacent triangles
///    (identical to `insert_point_on_edge_fast`'s edge split, but with the
///    edge located by an exact on-segment test instead of the nearest-edge
///    heuristic — the vertex may be far from the triangle's other edges).
/// 3. Repeat until every ring vertex is used (an inserted split can chain:
///    the first repair may reveal the next collinear vertex).
pub fn repair_unused_ring_vertices(
    all_2d: &[[f64; 2]],
    triangles: &mut Vec<[u32; 3]>,
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
) {
    // Ring vertex indices: outer rim 0..n_boundary, plus each hole range.
    let mut ring_vertices: Vec<usize> = (0..n_boundary).collect();
    for &(start, end) in hole_ranges {
        ring_vertices.extend(start..end);
    }

    // A repair pass can only make more vertices USED (splits never remove
    // vertex usages), so a bounded loop over the ring vertices converges.
    let mut repaired_any = true;
    let max_passes = ring_vertices.len() + 2;
    let mut pass = 0;
    while repaired_any && pass < max_passes {
        pass += 1;
        repaired_any = false;

        let mut used: Vec<bool> = vec![false; all_2d.len()];
        for tri in triangles.iter() {
            for &v in tri {
                if (v as usize) < used.len() {
                    used[v as usize] = true;
                }
            }
        }

        for &k in &ring_vertices {
            if used[k] {
                continue;
            }
            let p = all_2d[k];
            let split = find_edge_containing_point(triangles, all_2d, p, k);
            if let Some((v1, v2)) = split {
                split_edge_with_vertex(triangles, all_2d, v1, v2, k as u32);
                repaired_any = true;
                // Refresh used flags for this vertex only (cheap).
                used[k] = true;
            } else {
                log::debug!(
                    "repair_unused_ring_vertices: vertex {} at ({:.4},{:.4}) \
                     not on any triangle edge — left unused",
                    k, p[0], p[1]
                );
            }
        }
    }
}

/// Find a triangle edge whose segment geometrically contains point `p`
/// (collinear and within the segment bounds, with tolerance EPS).
/// Returns the (v1, v2) vertex pair of that edge.
fn find_edge_containing_point(
    triangles: &[[u32; 3]],
    vertices: &[[f64; 2]],
    p: [f64; 2],
    exclude: usize,
) -> Option<(u32, u32)> {
    for tri in triangles {
        for i in 0..3 {
            let a = tri[i];
            let b = tri[(i + 1) % 3];
            if a as usize == exclude || b as usize == exclude {
                continue;
            }
            let pa = vertices[a as usize];
            let pb = vertices[b as usize];
            if point_on_segment(p, pa, pb) {
                return Some((a, b));
            }
        }
    }
    None
}

/// Exact on-segment test: p collinear with (a, b) and inside the
/// axis-aligned bounding box of the segment (with EPS slack).
fn point_on_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> bool {
    let cross = orient2d(a, b, p);
    if cross.abs() > EPS {
        return false;
    }
    let (minx, maxx) = (a[0].min(b[0]), a[0].max(b[0]));
    let (miny, maxy) = (a[1].min(b[1]), a[1].max(b[1]));
    p[0] >= minx - EPS && p[0] <= maxx + EPS && p[1] >= miny - EPS && p[1] <= maxy + EPS
}

/// Split edge (v1, v2) into (v1, p) and (p, v2) in BOTH adjacent
/// triangles. A boundary edge (single adjacent triangle) is split in
/// that one triangle. Maintains the total-edge invariant: the two new
/// edges replace the old one everywhere it was used.
fn split_edge_with_vertex(
    triangles: &mut Vec<[u32; 3]>,
    vertices: &[[f64; 2]],
    v1: u32,
    v2: u32,
    p_idx: u32,
) {
    let edge_key = (v1.min(v2), v1.max(v2));

    // Collect the adjacent triangle indices (0, 1, or 2 of them).
    let adjacent: Vec<usize> = triangles
        .iter()
        .enumerate()
        .filter(|(_, tri)| {
            (0..3).any(|i| {
                let a = tri[i].min(tri[(i + 1) % 3]);
                let b = tri[i].max(tri[(i + 1) % 3]);
                (a, b) == edge_key
            })
        })
        .map(|(ti, _)| ti)
        .collect();

    for ti in adjacent {
        let [a, b, c] = triangles[ti];
        // The opposite vertex is the one that is neither v1 nor v2.
        let opposite = if a != v1 && a != v2 {
            a
        } else if b != v1 && b != v2 {
            b
        } else {
            c
        };
        // Winding-preserving split. The triangle's cyclic order is either
        // (…opp, v1, v2…) or (…opp, v2, v1…). In the first case the edge is
        // traversed v1→v2 and the sub-triangles are (opp, v1, p) and
        // (opp, p, v2); in the second it is traversed v2→v1 and the
        // sub-triangles are (opp, v2, p) and (opp, p, v1). Emitting the
        // wrong pair still tiles the same area but flips the triangle
        // winding, which would invert the face's mesh normals.
        let pos = |x: u32| -> usize {
            if x == a { 0 } else if x == b { 1 } else { 2 }
        };
        let (pv1, pv2, pop) = (pos(v1), pos(v2), pos(opposite));
        let v1_before_v2 = (pv1 + 3 - pop) % 3 < (pv2 + 3 - pop) % 3;
        let (t1, t2) = if v1_before_v2 {
            ([opposite, v1, p_idx], [opposite, p_idx, v2])
        } else {
            ([opposite, v2, p_idx], [opposite, p_idx, v1])
        };
        triangles[ti] = t1;
        triangles.push(t2);
        let _ = vertices; // reserved for diagnostics
    }
}

/// Find the triangle containing a point.
/// Returns (triangle_index, on_edge) where on_edge indicates the point is
/// exactly on one of the triangle's edges.
fn find_containing_triangle(
    vertices: &[[f64; 2]],
    triangles: &[[u32; 3]],
    p: [f64; 2],
) -> Option<(usize, bool)> {
    for (i, tri) in triangles.iter().enumerate() {
        let a = vertices[tri[0] as usize];
        let b = vertices[tri[1] as usize];
        let c = vertices[tri[2] as usize];

        let d1 = orient2d(p, a, b);
        let d2 = orient2d(p, b, c);
        let d3 = orient2d(p, c, a);

        let has_neg = d1 < -EPS || d2 < -EPS || d3 < -EPS;
        let has_pos = d1 > EPS || d2 > EPS || d3 > EPS;

        if !has_neg && !has_pos {
            return Some((i, true));
        }

        if !(has_neg && has_pos) {
            let on_edge = d1.abs() < EPS || d2.abs() < EPS || d3.abs() < EPS;
            return Some((i, on_edge));
        }
    }
    None
}

/// Insert a point inside a triangle by splitting it into 3 sub-triangles.
fn insert_point_in_triangle(
    triangles: &mut Vec<[u32; 3]>,
    tri_idx: usize,
    point_idx: u32,
) {
    let [a, b, c] = triangles[tri_idx];
    triangles[tri_idx] = [a, b, point_idx];
    triangles.push([b, c, point_idx]);
    triangles.push([c, a, point_idx]);
}

/// Insert a point inside a triangle, also updating the edge map.
fn insert_point_in_triangle_fast(
    triangles: &mut Vec<[u32; 3]>,
    tri_idx: usize,
    point_idx: u32,
    edge_map: &mut HashMap<(u32, u32), Vec<usize>>,
) {
    let [a, b, c] = triangles[tri_idx];

    // Remove old triangle's edges from map
    remove_tri_from_edge_map(edge_map, tri_idx, &[a, b, c]);

    // Create 3 new triangles
    triangles[tri_idx] = [a, b, point_idx];
    let t1 = triangles.len() as u32;
    triangles.push([b, c, point_idx]);
    let t2 = triangles.len() as u32;
    triangles.push([c, a, point_idx]);

    // Add new triangles' edges to map
    add_tri_to_edge_map(edge_map, tri_idx, &[a, b, point_idx]);
    add_tri_to_edge_map(edge_map, t1 as usize, &[b, c, point_idx]);
    add_tri_to_edge_map(edge_map, t2 as usize, &[c, a, point_idx]);
}

/// Insert a point that lies on an edge of a triangle.
fn insert_point_on_edge(
    vertices: &[[f64; 2]],
    triangles: &mut Vec<[u32; 3]>,
    tri_idx: usize,
    point_idx: u32,
) {
    let p = vertices[point_idx as usize];
    let [a, b, c] = triangles[tri_idx];

    let pa = vertices[a as usize];
    let pb = vertices[b as usize];
    let pc = vertices[c as usize];

    let d1 = orient2d(p, pa, pb).abs();
    let d2 = orient2d(p, pb, pc).abs();
    let d3 = orient2d(p, pc, pa).abs();

    // Find the edge with smallest orientation (= point is on that edge)
    let (edge_v1, edge_v2, opposite_v) = if d1 <= d2 && d1 <= d3 {
        (a, b, c)
    } else if d2 <= d3 {
        (b, c, a)
    } else {
        (c, a, b)
    };

    // Find the neighboring triangle sharing this edge
    let neighbor_idx = find_triangle_with_edge(triangles, edge_v1, edge_v2, tri_idx);

    // Split the current triangle
    triangles[tri_idx] = [opposite_v, edge_v1, point_idx];
    triangles.push([opposite_v, point_idx, edge_v2]);

    // Split the neighbor triangle
    if let Some(nbr_idx) = neighbor_idx {
        let [na, nb, nc] = triangles[nbr_idx];
        let nbr_opposite = if na != edge_v1 && na != edge_v2 {
            na
        } else if nb != edge_v1 && nb != edge_v2 {
            nb
        } else {
            nc
        };

        let (ev1_in_nbr, ev2_in_nbr) = find_edge_order_in_triangle(
            &triangles[nbr_idx], edge_v1, edge_v2,
        );

        triangles[nbr_idx] = [nbr_opposite, ev1_in_nbr, point_idx];
        triangles.push([nbr_opposite, point_idx, ev2_in_nbr]);
    }
}

/// Insert a point on an edge, also updating the edge map.
fn insert_point_on_edge_fast(
    vertices: &[[f64; 2]],
    triangles: &mut Vec<[u32; 3]>,
    tri_idx: usize,
    point_idx: u32,
    edge_map: &mut HashMap<(u32, u32), Vec<usize>>,
) {
    let p = vertices[point_idx as usize];
    let [a, b, c] = triangles[tri_idx];

    let pa = vertices[a as usize];
    let pb = vertices[b as usize];
    let pc = vertices[c as usize];

    let d1 = orient2d(p, pa, pb).abs();
    let d2 = orient2d(p, pb, pc).abs();
    let d3 = orient2d(p, pc, pa).abs();

    let (edge_v1, edge_v2, opposite_v) = if d1 <= d2 && d1 <= d3 {
        (a, b, c)
    } else if d2 <= d3 {
        (b, c, a)
    } else {
        (c, a, b)
    };

    // Find neighbor using edge map
    let edge_key = (edge_v1.min(edge_v2), edge_v1.max(edge_v2));
    let neighbor_idx = edge_map.get(&edge_key)
        .and_then(|indices| indices.iter().find(|&&i| i != tri_idx).copied());

    // Remove old triangle edges from map
    remove_tri_from_edge_map(edge_map, tri_idx, &[a, b, c]);

    // Split the current triangle
    triangles[tri_idx] = [opposite_v, edge_v1, point_idx];
    let t1 = triangles.len() as u32;
    triangles.push([opposite_v, point_idx, edge_v2]);

    // Add new triangles' edges
    add_tri_to_edge_map(edge_map, tri_idx, &[opposite_v, edge_v1, point_idx]);
    add_tri_to_edge_map(edge_map, t1 as usize, &[opposite_v, point_idx, edge_v2]);

    // Split the neighbor triangle
    if let Some(nbr_idx) = neighbor_idx {
        let [na, nb, nc] = triangles[nbr_idx];
        let nbr_opposite = if na != edge_v1 && na != edge_v2 {
            na
        } else if nb != edge_v1 && nb != edge_v2 {
            nb
        } else {
            nc
        };

        let (ev1_in_nbr, ev2_in_nbr) = find_edge_order_in_triangle(
            &triangles[nbr_idx], edge_v1, edge_v2,
        );

        // Remove old neighbor edges
        remove_tri_from_edge_map(edge_map, nbr_idx, &[na, nb, nc]);

        triangles[nbr_idx] = [nbr_opposite, ev1_in_nbr, point_idx];
        let t2 = triangles.len() as u32;
        triangles.push([nbr_opposite, point_idx, ev2_in_nbr]);

        // Add new neighbor triangles' edges
        add_tri_to_edge_map(edge_map, nbr_idx, &[nbr_opposite, ev1_in_nbr, point_idx]);
        add_tri_to_edge_map(edge_map, t2 as usize, &[nbr_opposite, point_idx, ev2_in_nbr]);
    }
}

/// Remove a triangle's edges from the edge map.
fn remove_tri_from_edge_map(
    edge_map: &mut HashMap<(u32, u32), Vec<usize>>,
    tri_idx: usize,
    tri: &[u32; 3],
) {
    for i in 0..3 {
        let a = tri[i].min(tri[(i + 1) % 3]);
        let b = tri[i].max(tri[(i + 1) % 3]);
        if let Some(indices) = edge_map.get_mut(&(a, b)) {
            indices.retain(|&i| i != tri_idx);
            if indices.is_empty() {
                edge_map.remove(&(a, b));
            }
        }
    }
}

/// Add a triangle's edges to the edge map.
fn add_tri_to_edge_map(
    edge_map: &mut HashMap<(u32, u32), Vec<usize>>,
    tri_idx: usize,
    tri: &[u32; 3],
) {
    for i in 0..3 {
        let a = tri[i].min(tri[(i + 1) % 3]);
        let b = tri[i].max(tri[(i + 1) % 3]);
        edge_map.entry((a, b)).or_default().push(tri_idx);
    }
}

/// Find a triangle (other than exclude) that contains the edge (v1, v2).
fn find_triangle_with_edge(
    triangles: &[[u32; 3]],
    v1: u32,
    v2: u32,
    exclude: usize,
) -> Option<usize> {
    for (i, tri) in triangles.iter().enumerate() {
        if i == exclude {
            continue;
        }
        let has_v1 = tri[0] == v1 || tri[1] == v1 || tri[2] == v1;
        let has_v2 = tri[0] == v2 || tri[1] == v2 || tri[2] == v2;
        if has_v1 && has_v2 {
            return Some(i);
        }
    }
    None
}

/// Find the order of edge vertices in a triangle.
fn find_edge_order_in_triangle(tri: &[u32; 3], v1: u32, v2: u32) -> (u32, u32) {
    for i in 0..3 {
        let j = (i + 1) % 3;
        if (tri[i] == v1 && tri[j] == v2) || (tri[i] == v2 && tri[j] == v1) {
            return (tri[i], tri[j]);
        }
    }
    (v1, v2)
}

/// Apply Lawson flips to improve Delaunay quality while respecting constraints.
///
/// A constraint edge is any edge of the outer boundary polygon or any hole polygon.
/// These edges must not be flipped.
///
/// Uses an edge-to-triangle adjacency map for O(1) neighbor lookups
/// instead of O(n) linear search per edge.
pub fn lawson_flip(
    vertices: &[[f64; 2]],
    triangles: &mut Vec<[u32; 3]>,
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
) {
    let constraint_edges = build_constraint_set(n_boundary, hole_ranges);

    // Build edge-to-triangle adjacency map for O(1) neighbor lookups
    let mut edge_map = build_edge_map(triangles);

    let max_iterations = triangles.len() * 3;
    let mut iteration = 0;

    loop {
        if iteration >= max_iterations {
            break;
        }
        iteration += 1;

        let mut flipped = false;
        let n_tris = triangles.len();

        for i in 0..n_tris {
            let stale_mode = std::env::var("DRAPPER_LAWSON_STALE")
                .as_deref()
                == Ok("1");
            let tri_outer = triangles[i];
            for edge_idx in 0..3 {
                // session-82 STALE-STATE FIX: re-read the CURRENT
                // triangle on every edge. A flip on an earlier edge
                // of the same triangle rewrote triangles[i], and the
                // stale vertex triple then paired DEAD vertices with
                // the LIVE neighbor — manufacturing overlapping
                // degenerate slivers on collinear rim runs (s82 f26
                // forensics: tri 116 flipped twice in one pass, first
                // [182,194,178]→[182,194,210], then the STALE copy
                // again → [194,178,180]+[194,180,182], destroying the
                // legitimate corner triangle; 18 one-sided edges + a
                // second non-manifold edge + 2% overlap).
                let tri = if stale_mode { tri_outer } else { triangles[i] };
                let ev1 = tri[edge_idx];
                let ev2 = tri[(edge_idx + 1) % 3];
                let opposite = tri[(edge_idx + 2) % 3];
                if ev1 == ev2 || ev2 == opposite || ev1 == opposite {
                    continue; // degenerate slot (should not happen)
                }

                // Skip constraint edges
                let edge_key = (ev1.min(ev2), ev1.max(ev2));
                if constraint_edges.contains(&edge_key) {
                    continue;
                }

                // Find neighboring triangle using edge map (O(1)).
                // session-82 NON-MANIFOLD GUARD: an edge shared by
                // 3+ triangles (earcut emits them around collinear
                // rim runs — f26 base: 1 nm edge) has NO valid flip
                // quad. Pairing with an arbitrary neighbor leaves the
                // remaining triangles dangling on the removed diagonal
                // — overlapping triangles + one-sided extra boundary
                // edges (s82 f26 measurement: overlap 0→15.6%, rim
                // 219→214, extra_bnd 3→22) — and the edge-map update
                // drops only the flipped pair's entries, leaving stale
                // references for the third triangle. Never flip a
                // non-manifold edge.
                let nm_guard_on = std::env::var("DRAPPER_LAWSON_NM_GUARD")
                    .as_deref()
                    != Ok("0");
                let nbr_idx = match edge_map.get(&edge_key) {
                    Some(indices) if indices.len() == 2 || !nm_guard_on => {
                        indices.iter().find(|&&idx| idx != i).copied()
                    }
                    _ => None, // 1 (boundary) or 3+ (non-manifold)
                };

                let nbr = match nbr_idx {
                    Some(idx) => triangles[idx],
                    None => continue,
                };

                // Find the opposite vertex in the neighbor
                let nbr_opposite = if nbr[0] != ev1 && nbr[0] != ev2 {
                    nbr[0]
                } else if nbr[1] != ev1 && nbr[1] != ev2 {
                    nbr[1]
                } else {
                    nbr[2]
                };

                // session-64 CONVEXITY GUARD (the reason this phase was
                // disabled 2026-09-09): a flip is geometrically valid
                // ONLY when the two apexes lie strictly on OPPOSITE
                // sides of the shared edge. For a reflex/dart quad one
                // apex lies inside the other's triangle — SAME side —
                // and the incircle test passes trivially there, so the
                // unguarded flip produced overlapping triangles (the
                // stress-test regression). Opposite sides ⟺ neither
                // apex inside the other triangle ⟺ the quad is strictly
                // convex ⟺ both new triangles are non-degenerate and
                // consistently oriented.
                let s_a = orient2d(
                    vertices[ev1 as usize],
                    vertices[ev2 as usize],
                    vertices[opposite as usize],
                );
                let s_b = orient2d(
                    vertices[ev1 as usize],
                    vertices[ev2 as usize],
                    vertices[nbr_opposite as usize],
                );
                if s_a * s_b >= 0.0 || s_a.abs() < EPS || s_b.abs() < EPS {
                    continue; // same side (reflex quad) or degenerate — never flip
                }

                // session-82 RELATIVE DEGENERACY GUARD: on
                // near-collinear rim runs (arc discretizations with
                // sag ~1e-5 between consecutive points) the absolute
                // EPS test passes ORIENTS THAT ARE PURE NOISE — the
                // quad's apexes sit a hair off the shared edge line,
                // "opposite sides" is decided by rounding, and the
                // incircle test on the degenerate quad is noise too.
                // Flipping there manufactures zero-area slivers
                // (s82 f26 bottom arc: 178/182/194/180 all within
                // 2.7e-3 of a straight line over a 0.53 span). Require
                // each apex to clear a RELATIVE height floor: |orient|
                // ≥ MIN_QUAD_ASPECT · edge_len². The s64 targets (Z
                // #1092 tents, 300:1 → rel 6.7e-3) still flip; the
                // collinear sag noise (rel ~5e-4) never does.
                {
                    let dx = vertices[ev1 as usize][0] - vertices[ev2 as usize][0];
                    let dy = vertices[ev1 as usize][1] - vertices[ev2 as usize][1];
                    let len_sq = dx * dx + dy * dy;
                    // s82: threshold env-tunable for A/B measurement.
                    // MEASURED DEFAULT OFF (0): the stale-state fix
                    // alone keeps the needle-breaking flips AND the
                    // planar integrity (f26 base: needles 136→4,
                    // overlap 0, rim 219/220); the relative floor at
                    // 1e-3 blocks legitimate needle-breaking flips
                    // and REGRESSES drill HM (1407/284 → 1662/402).
                    // Retained opt-in for future tuning.
                    let min_aspect = std::env::var("DRAPPER_LAWSON_MIN_ASPECT")
                        .ok()
                        .and_then(|v| v.parse::<f64>().ok())
                        .unwrap_or(0.0);
                    if len_sq <= 0.0
                        || (min_aspect > 0.0
                            && (s_a.abs() < min_aspect * len_sq
                                || s_b.abs() < min_aspect * len_sq))
                    {
                        continue;
                    }
                }

                // session-82 DUPLICATE-DIAGONAL GUARD: in a planar
                // mesh the new diagonal (opposite, nbr_opposite) cannot
                // pre-exist (it would cross the flipped edge). The
                // earcut base is NOT planar-clean — needle caps around
                // collinear rim runs can already carry that edge, and
                // flipping on top of it manufactures a non-manifold
                // edge (s82 f26: second nm edge + 18 one-sided boundary
                // edges + 2% overlap survived the nm guard). Skip the
                // flip when the diagonal already exists.
                let new_diag = (
                    opposite.min(nbr_opposite),
                    opposite.max(nbr_opposite),
                );
                let dup_guard_on = std::env::var("DRAPPER_LAWSON_DUP_GUARD")
                    .as_deref()
                    != Ok("0");
                if dup_guard_on && edge_map.contains_key(&new_diag) {
                    continue;
                }

                // Check if the flip would improve Delaunay quality
                if should_flip(vertices, opposite, ev1, ev2, nbr_opposite) {
                    let new_tri1 = [opposite, ev1, nbr_opposite];
                    let new_tri2 = [opposite, nbr_opposite, ev2];

                    let ccw1 = orient2d(
                        vertices[new_tri1[0] as usize],
                        vertices[new_tri1[1] as usize],
                        vertices[new_tri1[2] as usize],
                    );
                    let ccw2 = orient2d(
                        vertices[new_tri2[0] as usize],
                        vertices[new_tri2[1] as usize],
                        vertices[new_tri2[2] as usize],
                    );

                    let valid_flip = if ccw1 > 0.0 && ccw2 > 0.0 {
                        triangles[i] = new_tri1;
                        if let Some(nbr) = nbr_idx {
                            triangles[nbr] = new_tri2;
                        }
                        true
                    } else if ccw1 < 0.0 && ccw2 < 0.0 {
                        triangles[i] = [new_tri1[0], new_tri1[2], new_tri1[1]];
                        if let Some(nbr) = nbr_idx {
                            triangles[nbr] = [new_tri2[0], new_tri2[2], new_tri2[1]];
                        }
                        true
                    } else {
                        false
                    };

                    if valid_flip {
                        // s82 diagnostics: full flip log (env-gated)
                        if log::log_enabled!(log::Level::Debug) {
                            log::debug!(
                                "lawson_flip: edge ({},{}) tris {} {:?} + {} {:?} -> {:?} + {:?}",
                                ev1,
                                ev2,
                                i,
                                tri,
                                nbr_idx.unwrap_or(usize::MAX),
                                nbr,
                                triangles[i],
                                triangles.get(nbr_idx.unwrap_or(usize::MAX)).copied(),
                            );
                        }
                        // Update edge map: remove old edges, add new ones
                        if let Some(ni) = nbr_idx {
                            // Remove old triangle edges
                            remove_tri_from_edge_map(&mut edge_map, i, &tri);
                            remove_tri_from_edge_map(&mut edge_map, ni, &nbr);
                            // Add new triangle edges
                            add_tri_to_edge_map(&mut edge_map, i, &triangles[i]);
                            add_tri_to_edge_map(&mut edge_map, ni, &triangles[ni]);
                        }
                        flipped = true;
                    }
                }
            }
        }

        if !flipped {
            break;
        }
    }
}

/// Check if an edge should be flipped for Delaunay quality.
fn should_flip(
    vertices: &[[f64; 2]],
    opposite: u32,
    ev1: u32,
    ev2: u32,
    nbr_opposite: u32,
) -> bool {
    let p_opp = vertices[opposite as usize];
    let p_ev1 = vertices[ev1 as usize];
    let p_ev2 = vertices[ev2 as usize];
    let p_nbr = vertices[nbr_opposite as usize];

    circumcircle_contains_point(p_ev1, p_ev2, p_opp, p_nbr)
}

/// Build the set of constraint edges (boundary + holes).
fn build_constraint_set(
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
) -> HashSet<(u32, u32)> {
    let mut constraints = HashSet::new();

    for i in 0..n_boundary {
        let j = (i + 1) % n_boundary;
        let a = i.min(j) as u32;
        let b = i.max(j) as u32;
        constraints.insert((a, b));
    }

    for &(start, end) in hole_ranges {
        let n_hole = end - start;
        for i in 0..n_hole {
            let j = (i + 1) % n_hole;
            let a = (start + i).min(start + j) as u32;
            let b = (start + i).max(start + j) as u32;
            constraints.insert((a, b));
        }
    }

    constraints
}

/// Robust 2D orientation test.
/// Returns positive if a, b, c are counter-clockwise, negative if clockwise.
pub(crate) fn orient2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0])
}

/// Check if a point is inside the circumcircle of a triangle.
fn circumcircle_contains_point(a: [f64; 2], b: [f64; 2], c: [f64; 2], p: [f64; 2]) -> bool {
    let ax = a[0] - p[0];
    let ay = a[1] - p[1];
    let bx = b[0] - p[0];
    let by = b[1] - p[1];
    let cx = c[0] - p[0];
    let cy = c[1] - p[1];

    let det = (ax * ax + ay * ay) * (bx * cy - cx * by)
            - (bx * bx + by * by) * (ax * cy - cx * ay)
            + (cx * cx + cy * cy) * (ax * by - bx * ay);

    let orient = orient2d(a, b, c);
    if orient > 0.0 {
        det > EPS
    } else {
        det < -EPS
    }
}

/// Check if a 2D point is inside a polygon using ray casting.
pub fn point_in_polygon(p: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let n = polygon.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let xi = polygon[i][0];
        let yi = polygon[i][1];
        let xj = polygon[j][0];
        let yj = polygon[j][1];

        if ((yi > p[1]) != (yj > p[1]))
            && (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Verify that all constraint edges exist in the triangulation.
///
/// This is the PRODUCTION version (not just debug). It checks that every
/// boundary edge (and hole edge) is present as an edge of at least one
/// triangle in the mesh.
///
/// Returns a list of missing constraint edges. Empty vec = all present.
pub fn verify_constraint_edges_production(
    triangles: &[[u32; 3]],
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
) -> Vec<(u32, u32)> {
    use std::collections::HashSet;

    let mut tri_edges: HashSet<(u32, u32)> = HashSet::new();
    for tri in triangles {
        for i in 0..3 {
            let a = tri[i].min(tri[(i + 1) % 3]);
            let b = tri[i].max(tri[(i + 1) % 3]);
            tri_edges.insert((a, b));
        }
    }

    let mut missing = Vec::new();

    // Check outer boundary edges
    for i in 0..n_boundary {
        let j = (i + 1) % n_boundary;
        let a = i.min(j) as u32;
        let b = i.max(j) as u32;
        if !tri_edges.contains(&(a, b)) {
            missing.push((a, b));
        }
    }

    // Check hole edges
    for &(start, end) in hole_ranges {
        let n_hole = end - start;
        for i in 0..n_hole {
            let j = (i + 1) % n_hole;
            let a = (start + i).min(start + j) as u32;
            let b = (start + i).max(start + j) as u32;
            if !tri_edges.contains(&(a, b)) {
                missing.push((a, b));
            }
        }
    }

    missing
}

/// Verify that all constraint edges exist in the triangulation.
#[cfg(debug_assertions)]
fn verify_constraints(
    triangles: &[[u32; 3]],
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
) {
    let mut tri_edges: HashSet<(u32, u32)> = HashSet::new();
    for tri in triangles {
        for i in 0..3 {
            let a = tri[i].min(tri[(i + 1) % 3]);
            let b = tri[i].max(tri[(i + 1) % 3]);
            tri_edges.insert((a, b));
        }
    }

    for i in 0..n_boundary {
        let j = (i + 1) % n_boundary;
        let a = i.min(j) as u32;
        let b = i.max(j) as u32;
        if !tri_edges.contains(&(a, b)) {
            log::warn!("CDT constraint violation: boundary edge ({}, {}) missing", a, b);
        }
    }

    for &(start, end) in hole_ranges {
        let n_hole = end - start;
        for i in 0..n_hole {
            let j = (i + 1) % n_hole;
            let a = (start + i).min(start + j) as u32;
            let b = (start + i).max(start + j) as u32;
            if !tri_edges.contains(&(a, b)) {
                log::warn!("CDT constraint violation: hole edge ({}, {}) missing", a, b);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_square_triangulation() {
        let points = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let result = triangulate_polygon_cdt(&points, &[], &[]);
        assert!(result.len() >= 2, "Square should have at least 2 triangles, got {}", result.len());

        let mut vertex_used = [false; 4];
        for tri in &result {
            for v in tri {
                if (*v as usize) < 4 {
                    vertex_used[*v as usize] = true;
                }
            }
        }
        for (i, used) in vertex_used.iter().enumerate() {
            assert!(used, "Vertex {} should appear in triangulation", i);
        }
    }

    #[test]
    fn test_triangle_triangulation() {
        let points = [[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]];
        let result = triangulate_polygon_cdt(&points, &[], &[]);
        assert_eq!(result.len(), 1, "Triangle should have 1 triangle");
    }

    #[test]
    fn test_polygon_with_interior_point() {
        let points = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]];
        let interior = [[1.0, 1.0]];
        let result = triangulate_polygon_cdt(&points, &[], &interior);
        assert!(result.len() >= 4, "Square with interior point should have >= 4 triangles, got {}", result.len());

        let mut interior_used = false;
        for tri in &result {
            for v in tri {
                if *v == 4 {
                    interior_used = true;
                }
            }
        }
        assert!(interior_used, "Interior point should appear in triangulation");
    }

    #[test]
    fn test_polygon_with_hole() {
        let boundary = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        let hole = vec![[1.0, 1.0], [1.0, 3.0], [3.0, 3.0], [3.0, 1.0]];
        let result = triangulate_polygon_cdt(&boundary, &[hole], &[]);
        assert!(result.len() >= 4, "Square with hole should have >= 4 triangles, got {}", result.len());
    }

    #[test]
    fn test_point_in_polygon() {
        let polygon = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        assert!(point_in_polygon([0.5, 0.5], &polygon));
        assert!(!point_in_polygon([1.5, 0.5], &polygon));
    }

    #[test]
    fn test_constraint_edges_preserved() {
        let boundary = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        let interior = [[1.0, 1.0], [2.0, 2.0], [3.0, 1.0]];
        let result = triangulate_polygon_cdt(&boundary, &[], &interior);

        let mut tri_edges: HashSet<(u32, u32)> = HashSet::new();
        for tri in &result {
            for i in 0..3 {
                let a = tri[i].min(tri[(i + 1) % 3]);
                let b = tri[i].max(tri[(i + 1) % 3]);
                tri_edges.insert((a, b));
            }
        }

        for i in 0..4 {
            let j = (i + 1) % 4;
            let a = i.min(j) as u32;
            let b = i.max(j) as u32;
            assert!(tri_edges.contains(&(a, b)), "Boundary edge ({}, {}) should exist", a, b);
        }
    }

    /// Count "interior boundary edges" of a triangulation: edges between
    /// two INTERIOR (non-rim) vertices that have exactly 1 adjacent
    /// triangle. A watertight face triangulation must have ZERO of these —
    /// such edges are holes inside the face.
    fn count_interior_boundary_edges(
        triangles: &[[u32; 3]],
        n_rim: usize,
    ) -> usize {
        let mut usage: HashMap<(u32, u32), u32> = HashMap::new();
        for tri in triangles {
            for i in 0..3 {
                let a = tri[i].min(tri[(i + 1) % 3]);
                let b = tri[i].max(tri[(i + 1) % 3]);
                *usage.entry((a, b)).or_insert(0) += 1;
            }
        }
        usage
            .iter()
            .filter(|(&(a, b), &c)| {
                c == 1 && a as usize >= n_rim && b as usize >= n_rim
            })
            .count()
    }

    /// Regression: the legacy earcutr path appends interior Steiner points
    /// directly to the earcutr input (spike-chain), which produces interior
    /// holes (Steiner-to-Steiner edges with 1 adjacent triangle). The CDT
    /// path (Bowyer-Watson insertion) must produce ZERO such edges.
    ///
    /// This test reproduces the HOUSING #47598 root cause (6089 boundary
    /// edges, 57% Steiner-to-Steiner) in miniature.
    #[test]
    fn test_steiner_insertion_no_interior_gaps_vs_legacy_earcutr() {
        // 8x8 square boundary (32 rim points) + 5x5 interior grid.
        let n = 8usize;
        let mut rim: Vec<[f64; 2]> = Vec::new();
        for i in 0..n {
            rim.push([i as f64, 0.0]);
        }
        for j in 1..n {
            rim.push([n as f64, j as f64]);
        }
        for i in (0..n - 1).rev() {
            rim.push([i as f64, n as f64]);
        }
        for j in (1..n - 1).rev() {
            rim.push([0.0, j as f64]);
        }
        let n_rim = rim.len();

        let mut interior: Vec<[f64; 2]> = Vec::new();
        for i in 1..=5 {
            for j in 1..=5 {
                interior.push([i as f64, j as f64]);
            }
        }

        // ── Legacy path: append interior points to earcutr input ──────
        let mut coords: Vec<f64> = Vec::new();
        for p in rim.iter().chain(interior.iter()) {
            coords.push(p[0]);
            coords.push(p[1]);
        }
        let legacy = crate::earcut_adapter::triangulate_polygon_with_holes(&coords, &[]);
        let legacy_tris: Vec<[u32; 3]> = legacy
            .chunks(3)
            .filter_map(|c| {
                if c.len() < 3 {
                    return None;
                }
                let (a, b, cc) = (c[0] as u32, c[1] as u32, c[2] as u32);
                if a == b || b == cc || a == cc {
                    return None;
                }
                Some([a, b, cc])
            })
            .collect();

        // ── CDT path ──────────────────────────────────────────────────
        let cdt = triangulate_polygon_cdt(&rim, &[], &interior);

        let legacy_gaps = count_interior_boundary_edges(&legacy_tris, n_rim);
        let cdt_gaps = count_interior_boundary_edges(&cdt, n_rim);

        // The CDT must be hole-free in the interior.
        assert_eq!(
            cdt_gaps, 0,
            "CDT path must not leave Steiner-to-Steiner boundary edges (got {})",
            cdt_gaps
        );
        // The legacy path demonstrably leaks (this is the bug being fixed;
        // if this assertion ever fails, the legacy path was silently
        // changed and this regression guard should be revisited).
        assert!(
            legacy_gaps > 0,
            "legacy earcutr spike-chain should show interior gaps here — if it \
             no longer does, the adapter changed; re-audit the production path"
        );
    }

    /// Stress: 12x12 rim + 6x6 interior grid + square hole — the CDT must
    /// (a) have zero interior boundary edges and (b) preserve every rim and
    /// hole edge as a triangle edge (each exactly 1 adjacent triangle).
    #[test]
    fn test_cdt_with_hole_and_grid_no_gaps() {
        let n = 12usize;
        let mut rim: Vec<[f64; 2]> = Vec::new();
        for i in 0..n {
            rim.push([i as f64, 0.0]);
        }
        for j in 1..n {
            rim.push([n as f64, j as f64]);
        }
        for i in (0..n - 1).rev() {
            rim.push([i as f64, n as f64]);
        }
        for j in (1..n - 1).rev() {
            rim.push([0.0, j as f64]);
        }
        let n_rim = rim.len();

        // Hole: square [4,4]x[7,7] (16 points, CCW)
        let hole: Vec<[f64; 2]> = vec![
            [4.0, 4.0], [5.0, 4.0], [6.0, 4.0], [7.0, 4.0],
            [7.0, 5.0], [7.0, 6.0], [7.0, 7.0],
            [6.0, 7.0], [5.0, 7.0], [4.0, 7.0],
            [4.0, 6.0], [4.0, 5.0],
        ];

        // Interior grid avoiding the hole area
        let mut interior: Vec<[f64; 2]> = Vec::new();
        'outer: for i in 1..n {
            for j in 1..n {
                let p = [i as f64, j as f64];
                if p[0] >= 3.9 && p[0] <= 7.1 && p[1] >= 3.9 && p[1] <= 7.1 {
                    continue; // skip hole region (with margin)
                }
                interior.push(p);
                if interior.len() >= 60 {
                    break 'outer;
                }
            }
        }

        let tris = triangulate_polygon_cdt(&rim, &[hole.clone()], &interior);

        // (a) no interior-to-interior boundary edges
        let mut usage: HashMap<(u32, u32), u32> = HashMap::new();
        for tri in &tris {
            for i in 0..3 {
                let a = tri[i].min(tri[(i + 1) % 3]);
                let b = tri[i].max(tri[(i + 1) % 3]);
                *usage.entry((a, b)).or_insert(0) += 1;
            }
        }
        let interior_gaps = usage
            .iter()
            .filter(|(&(a, b), &c)| c == 1 && a as usize >= n_rim + hole.len() && b as usize >= n_rim + hole.len())
            .count();
        assert_eq!(interior_gaps, 0, "interior Steiner gaps: {}", interior_gaps);

        // (b) every rim edge and hole edge must exist exactly once
        let n_hole = hole.len();
        for i in 0..n_rim {
            let j = (i + 1) % n_rim;
            let key = (i.min(j) as u32, i.max(j) as u32);
            let c = usage.get(&key).copied().unwrap_or(0);
            assert_eq!(c, 1, "rim edge {:?} usage {} (must be 1)", key, c);
        }
        for k in 0..n_hole {
            let i = n_rim + k;
            let j = n_rim + (k + 1) % n_hole;
            let key = (i.min(j) as u32, i.max(j) as u32);
            let c = usage.get(&key).copied().unwrap_or(0);
            assert_eq!(c, 1, "hole edge {:?} usage {} (must be 1)", key, c);
        }
    }
}

#[cfg(test)]
mod s82_tests {
    use super::*;

    /// s82 regression: the f26 (Nurbs 4x10 fillet) class — a thin band
    /// domain whose rim arcs are near-collinear point runs. The stale
    /// `tri` local in lawson_flip paired dead vertices with live
    /// neighbors after an intra-triangle flip; on collinear runs the
    /// noise-driven "opposite sides" test then passed and the flip
    /// manufactured overlapping degenerate slivers (measured on the
    /// real face: overlap ratio 0.156, 6 rim edges lost, 22 one-sided
    /// boundary edges, a second non-manifold edge). A valid
    /// triangulation of the full polygon satisfies sum|A| == polygon
    /// area; the stale bug inflated it 15.6%.
    #[test]
    fn test_lawson_stale_state_no_overlap_on_collinear_band() {
        // Thin band 1.0 x 0.06; bottom/top rims are 21-point runs with
        // a 2e-3 sag (the collinear-run signature), sides are 3-point.
        let mut ring: Vec<[f64; 2]> = Vec::new();
        let n = 21;
        for i in 0..n {
            let x = i as f64 / (n - 1) as f64;
            ring.push([x, 0.02 * x * (1.0 - x)]); // bottom, sag 5e-3
        }
        for i in 0..n {
            let x = 1.0 - i as f64 / (n - 1) as f64;
            ring.push([x, 0.06 + 0.02 * x * (1.0 - x)]); // top
        }
        // Interior: a sparse lattice (2 rows x 9 cols).
        let mut interior: Vec<[f64; 2]> = Vec::new();
        for j in 1..=2 {
            let y = 0.02 * j as f64;
            for i in 1..=9 {
                interior.push([i as f64 / 10.0, y]);
            }
        }
        let tris = triangulate_polygon_cdt(&ring, &[], &interior);
        assert!(!tris.is_empty());

        // Planar integrity: sum of |signed areas| equals the polygon
        // area for any valid full triangulation (no overlap, no
        // spillover). The stale-state corruption measured 1.156x.
        let mut all_pts: Vec<[f64; 2]> = ring.clone();
        all_pts.extend_from_slice(&interior);
        let mut sum_abs = 0.0;
        for t in &tris {
            let s = orient2d(
                all_pts[t[0] as usize],
                all_pts[t[1] as usize],
                all_pts[t[2] as usize],
            )
            .abs();
            sum_abs += s;
        }
        let mut ring2 = 0.0;
        let n_r = ring.len();
        for i in 0..n_r {
            let j = (i + 1) % n_r;
            ring2 += ring[i][0] * ring[j][1] - ring[j][0] * ring[i][1];
        }
        let poly2 = ring2.abs();
        let ratio = sum_abs / poly2;
        assert!(
            (ratio - 1.0).abs() < 1e-9,
            "overlap ratio {} (sum|2A|={:.6} vs polygon 2A={:.6}) — the lawson round corrupted the triangulation",
            ratio,
            sum_abs,
            poly2
        );

        // Edge census: every edge used by at most 2 triangles (no
        // manufactured non-manifold edges beyond what earcut emits),
        // and every rim edge survives (constraints are never flipped).
        let mut ecount: std::collections::HashMap<(u32, u32), usize> =
            std::collections::HashMap::new();
        for t in &tris {
            for k in 0..3 {
                let a = t[k];
                let b = t[(k + 1) % 3];
                if a != b {
                    *ecount.entry((a.min(b), a.max(b))).or_default() += 1;
                }
            }
        }
        for i in 0..n_r {
            let j = (i + 1) % n_r;
            let key = ((i as u32).min(j as u32), (i as u32).max(j as u32));
            assert!(
                ecount.contains_key(&key),
                "rim edge ({},{}) missing — constraints must never flip",
                i,
                j
            );
        }
    }

    /// s82: the non-manifold guard — an edge shared by 3+ triangles
    /// (emitted by earcut around collinear rim runs) must never be
    /// flipped: pairing with an arbitrary neighbor strands the third
    /// triangle on the removed diagonal (overlap + one-sided edges).
    /// Build a fan-with-flap: 5 triangles where edge (0,2) carries 3.
    #[test]
    fn test_lawson_nm_edge_never_flipped() {
        // vertices: 0..4 on a convex arc + apex 5
        let verts: Vec<[f64; 2]> = vec![
            [0.0, 0.0],
            [1.0, 0.05],
            [2.0, 0.0],
            [3.0, 0.05],
            [1.0, 1.0],
        ];
        // Three triangles sharing edge (0,2): one apex-side fan
        // triangle plus two chord-side flaps — non-manifold by
        // construction (the earcut degenerate-run signature).
        let mut tris: Vec<[u32; 3]> = vec![
            [4, 0, 2], // apex side
            [4, 2, 3],
            [4, 3, 0],
            [1, 0, 2], // chord side, flap A
            [3, 0, 2], // chord side, flap B
        ];
        let before: Vec<[u32; 3]> = tris.clone();
        let hole_ranges: Vec<(usize, usize)> = Vec::new();
        lawson_flip(&verts, &mut tris, 5, &hole_ranges);
        // the nm edge (0,2) must still have exactly its original
        // triangle count (3) — no flip may have touched it.
        let count = |tris: &[[u32; 3]]| -> usize {
            tris.iter().filter(|t| t.contains(&0) && t.contains(&2)).count()
        };
        let c_before = count(&before);
        let c_after = count(&tris);
        assert_eq!(
            c_before, 3,
            "test setup: edge (0,2) must start non-manifold"
        );
        assert_eq!(
            c_after, 3,
            "non-manifold edge (0,2) triangle count changed {} -> {} — the nm guard must prevent flips on it",
            c_before, c_after
        );
    }
}
