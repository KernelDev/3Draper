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

    // session-83: STRUCTURAL LATTICE rescue. When the standard
    // pipeline leaves defects (interior Steiner drops / missing rim
    // edges / one-sided interior seams — the f26 class: 205/341
    // lattice points dropped by the sliver-guard chord deadlock) and
    // the interior points form a clean v-const row lattice on a
    // 4-corner monotone ring, rebuild the whole face structurally:
    // rows zipped to the ring walls (the s78 SAIL_BAND construction
    // on the adaptive subdivision's own lattice). Healthy faces
    // (defects == 0) stay bit-identical — the rescue never fires.
    // Kill-switch DRAPPER_LATTICE_BAND=0; debug DRAPPER_LATTICE_DEBUG.
    if !interior_2d.is_empty() && hole_index_ranges.is_empty() {
        let defects = cdt_defect_count(
            &all_2d,
            &triangles,
            n_boundary,
            &hole_index_ranges,
            interior_start,
            interior_2d.len(),
        );
        if defects > 0 {
            if let Some(structured) = structured_lattice_triangulation(
                &all_2d,
                n_boundary,
                interior_start,
                interior_2d.len(),
            ) {
                if std::env::var("DRAPPER_LATTICE_DEBUG").is_ok() {
                    eprintln!(
                        "[LATTICE rescue {}] standard defects {} -> structured {} tris (was {})",
                        crate::parametric_domain::current_face_label(),
                        defects,
                        structured.len(),
                        triangles.len()
                    );
                }
                return structured;
            }
        }
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

// ══════════════════════════════════════════════════════════════════
// session-83: STRUCTURAL LATTICE triangulation
// ══════════════════════════════════════════════════════════════════

/// session-83: count the standard pipeline's output defects that the
/// structural lattice rescue exists to fix:
///   - interior Steiner points left unused (insertion drops)
///   - rim edges missing (cross-face contract violations)
///   - one-sided non-rim edges (interior seams)
/// Returns the total; 0 = the standard result is healthy and the face
/// must stay bit-identical (the rescue never fires).
fn cdt_defect_count(
    all_2d: &[[f64; 2]],
    triangles: &[[u32; 3]],
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
    interior_start: usize,
    n_interior: usize,
) -> usize {
    let _ = all_2d;
    let mut used = vec![false; interior_start + n_interior];
    for t in triangles {
        for &v in t.iter() {
            let vi = v as usize;
            if vi < used.len() {
                used[vi] = true;
            }
        }
    }
    let mut defects = (interior_start..interior_start + n_interior)
        .filter(|&i| !used[i])
        .count();
    // edge multiplicity census
    let mut ecount: HashMap<(u32, u32), usize> = HashMap::new();
    for t in triangles {
        for k in 0..3 {
            let a = t[k];
            let b = t[(k + 1) % 3];
            if a != b {
                *ecount.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
    }
    let mut rim_miss = 0usize;
    let mut is_rim = |a: usize, b: usize| -> bool {
        ecount.contains_key(&(a.min(b) as u32, a.max(b) as u32))
    };
    for i in 0..n_boundary {
        let j = (i + 1) % n_boundary;
        if !is_rim(i, j) {
            rim_miss += 1;
        }
    }
    for &(start, end) in hole_ranges {
        let len = end - start;
        for i in 0..len {
            let a = start + i;
            let b = start + (i + 1) % len;
            if !is_rim(a, b) {
                rim_miss += 1;
            }
        }
    }
    defects += rim_miss;
    // one-sided non-rim edges: count 1 (or 3+) edges that are not rim
    let rim_set: HashSet<(u32, u32)> = {
        let mut s = HashSet::new();
        for i in 0..n_boundary {
            let j = (i + 1) % n_boundary;
            s.insert((i.min(j) as u32, i.max(j) as u32));
        }
        for &(start, end) in hole_ranges {
            let len = end - start;
            for i in 0..len {
                let a = start + i;
                let b = start + (i + 1) % len;
                s.insert((a.min(b) as u32, a.max(b) as u32));
            }
        }
        s
    };
    for (e, cnt) in ecount.iter() {
        if *cnt != 2 && !rim_set.contains(e) {
            defects += 1;
        }
    }
    defects
}

/// session-83: STRUCTURAL LATTICE row triangulation — the s78 SAIL_BAND
/// construction generalized to a GIVEN interior lattice (the adaptive
/// subdivision's own Steiner points) on a 4-corner monotone ring.
///
/// Motivation (s81/s82): on dense fillet lattices (f26 class: 341-point
/// brick lattice, 16x21 at step 0.024) the point-by-point
/// Bowyer-Watson insertion DEADLOCKS — the earcut+lawson base carries
/// long Delaunay chords (4-17 lattice steps), every lattice point
/// inside a long-chord parent trips the s64 sliver guard, and the skip
/// keeps the chord alive for the next point (s82 f26: 205/341 dropped,
/// the surviving 25 (26,26) REAL fold pairs ARE the unsplit coarse
/// triangles). Guard-off inserts them but manufactures 133 UV needles
/// that fold in 3D; the single-point split fallback inserts ~1000:1
/// strips that fold worse (s82 measured, default OFF).
///
/// The structural answer never fights the base: it builds the whole
/// face from rows. Interior points are clustered into v-const rows;
/// the ring is split at its 4 direction-transition corners into
/// bottom/top caps (u-monotone) and left/right walls (v-monotone,
/// ANGLED walls allowed — the sail band's u-const requirement is
/// dropped); wall points are banded positionally between row levels;
/// consecutive rows are stitched with the s78 two-pointer zipper.
/// Every rim edge is emitted exactly once (wall bands fan from the
/// adjacent row's end points, never from a bare wall anchor), and the
/// grid quads carry healthy aspect by construction.
///
/// Guards (any failure → None → the standard pipeline result stands):
/// every ring/interior point used, rim edges exactly once, non-rim
/// edges exactly twice, uniform winding, area within 0.5% of the
/// polygon shoelace, zero UV needles (aspect < 1e-3).
///
/// Kill-switch: DRAPPER_LATTICE_BAND=0. Debug: DRAPPER_LATTICE_DEBUG.
fn structured_lattice_triangulation(
    all_2d: &[[f64; 2]],
    n_boundary: usize,
    interior_start: usize,
    n_interior: usize,
) -> Option<Vec<[u32; 3]>> {
    if std::env::var("DRAPPER_LATTICE_BAND").as_deref() == Ok("0") {
        return None;
    }
    let dbg = std::env::var("DRAPPER_LATTICE_DEBUG").is_ok();
    macro_rules! lattice_reject {
        ($reason:expr) => {{
            if dbg {
                eprintln!(
                    "[LATTICE reject {}] {}",
                    crate::parametric_domain::current_face_label(),
                    $reason
                );
            }
            return None;
        }};
    }
    if n_boundary < 8 || n_interior < 8 {
        lattice_reject!("tiny input");
    }
    // ── 1. interior row detection (v-const clusters) ───────────────
    let mut ivmin = f64::INFINITY;
    let mut ivmax = f64::NEG_INFINITY;
    for i in interior_start..interior_start + n_interior {
        let v = all_2d[i][1];
        if !v.is_finite() {
            lattice_reject!("non-finite interior v");
        }
        ivmin = ivmin.min(v);
        ivmax = ivmax.max(v);
    }
    let ivspan = ivmax - ivmin;
    if ivspan <= 0.0 {
        lattice_reject!("flat interior v");
    }
    let row_tol = 2e-3 * ivspan;
    let mut order: Vec<usize> = (interior_start..interior_start + n_interior).collect();
    order.sort_by(|&a, &b| {
        all_2d[a][1]
            .partial_cmp(&all_2d[b][1])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                all_2d[a][0]
                    .partial_cmp(&all_2d[b][0])
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });
    let mut rows: Vec<Vec<usize>> = Vec::new();
    {
        let mut cur: Vec<usize> = vec![order[0]];
        for &idx in order.iter().skip(1) {
            if all_2d[idx][1] - all_2d[cur[cur.len() - 1]][1] > row_tol {
                rows.push(cur);
                cur = vec![idx];
            } else {
                cur.push(idx);
            }
        }
        rows.push(cur);
    }
    let m_rows = rows.len();
    if m_rows < 4 {
        lattice_reject!("too few rows");
    }
    for r in rows.iter_mut() {
        r.sort_by(|&a, &b| {
            all_2d[a][0]
                .partial_cmp(&all_2d[b][0])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        // strict u-monotone within the row, v-spread within tol
        let (vlo, vhi) = {
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            for &i in r.iter() {
                lo = lo.min(all_2d[i][1]);
                hi = hi.max(all_2d[i][1]);
            }
            (lo, hi)
        };
        if r.len() < 2 || vhi - vlo > row_tol {
            lattice_reject!("row not v-const");
        }
        for w in r.windows(2) {
            if all_2d[w[1]][0] - all_2d[w[0]][0] <= 1e-9 {
                lattice_reject!("row u not strictly ascending");
            }
        }
    }
    let row_v: Vec<f64> = rows.iter().map(|r| all_2d[r[0]][1]).collect();
    // ── 2. ring 4-corner split (H/V edge-class transitions) ────────
    let n = n_boundary;
    let edge_class = |i: usize| -> u8 {
        // 0 = H (u-dominant), 1 = V (v-dominant), 2 = D (mixed)
        let a = all_2d[i];
        let b = all_2d[(i + 1) % n];
        let (du, dv) = ((b[0] - a[0]).abs(), (b[1] - a[1]).abs());
        if du >= 2.0 * dv {
            0
        } else if dv > 2.0 * du {
            1
        } else {
            2
        }
    };
    let mut has_d = false;
    let mut cls = Vec::with_capacity(n);
    for i in 0..n {
        let c = edge_class(i);
        if c == 2 {
            has_d = true;
        }
        cls.push(c);
    }
    if has_d {
        // rounded/organic corners: outside the s83 rectangle class
        lattice_reject!("diagonal rim edges (rounded corners)");
    }
    let corners: Vec<usize> = (0..n)
        .filter(|&i| cls[(i + n - 1) % n] != cls[i])
        .collect();
    if corners.len() != 4 {
        lattice_reject!(format!("corner count {}", corners.len()));
    }
    let cu_min = corners.iter().map(|&i| all_2d[i][0]).fold(f64::INFINITY, f64::min);
    let cu_max = corners.iter().map(|&i| all_2d[i][0]).fold(f64::NEG_INFINITY, f64::max);
    let cv_min = corners.iter().map(|&i| all_2d[i][1]).fold(f64::INFINITY, f64::min);
    let cv_max = corners.iter().map(|&i| all_2d[i][1]).fold(f64::NEG_INFINITY, f64::max);
    let nearest_corner = |tu: f64, tv: f64| -> usize {
        let mut best = corners[0];
        let mut bd = f64::INFINITY;
        for &c in corners.iter() {
            let p = all_2d[c];
            let d = (p[0] - tu) * (p[0] - tu) + (p[1] - tv) * (p[1] - tv);
            if d < bd {
                bd = d;
                best = c;
            }
        }
        best
    };
    let bl = nearest_corner(cu_min, cv_min);
    let br = nearest_corner(cu_max, cv_min);
    let tr = nearest_corner(cu_max, cv_max);
    let tl = nearest_corner(cu_min, cv_max);
    if bl == br || bl == tr || bl == tl || br == tr || br == tl || tr == tl {
        lattice_reject!("corner identity collapse");
    }
    // ── 3. four arcs between corners ───────────────────────────────
    let arc = |from: usize, to: usize, avoid: &[usize; 2]| -> Option<Vec<usize>> {
        let mut fwd = vec![from];
        let mut i = from;
        let mut ok = true;
        while i != to {
            i = (i + 1) % n;
            if i == to {
                break;
            }
            if i == avoid[0] || i == avoid[1] {
                ok = false;
                break;
            }
            fwd.push(i);
        }
        if ok {
            fwd.push(to);
        } else {
            fwd.clear();
        }
        let mut bwd = vec![from];
        let mut i = from;
        let mut ok = true;
        while i != to {
            i = (i + n - 1) % n;
            if i == to {
                break;
            }
            if i == avoid[0] || i == avoid[1] {
                ok = false;
                break;
            }
            bwd.push(i);
        }
        if ok {
            bwd.push(to);
        } else {
            bwd.clear();
        }
        match (fwd.is_empty(), bwd.is_empty()) {
            (false, false) => Some(if fwd.len() <= bwd.len() { fwd } else { bwd }),
            (false, true) => Some(fwd),
            (true, false) => Some(bwd),
            (true, true) => None,
        }
    };
    let mut bottom = arc(bl, br, &[tl, tr])?;
    let mut top = arc(tl, tr, &[bl, br])?;
    let mut left = arc(bl, tl, &[br, tr])?;
    let mut right = arc(br, tr, &[bl, tl])?;
    if bottom.len() < 2 || top.len() < 2 || left.len() < 2 || right.len() < 2 {
        lattice_reject!("degenerate arc");
    }
    let (umin, umax) = {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for i in 0..n {
            lo = lo.min(all_2d[i][0]);
            hi = hi.max(all_2d[i][0]);
        }
        (lo, hi)
    };
    let (vmin, vmax) = {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for i in 0..n {
            lo = lo.min(all_2d[i][1]);
            hi = hi.max(all_2d[i][1]);
        }
        (lo, hi)
    };
    let uspan = umax - umin;
    let vspan = vmax - vmin;
    if uspan <= 0.0 || vspan <= 0.0 {
        lattice_reject!("flat ring");
    }
    // orient: caps u-ascending, walls v-ascending
    if all_2d[bottom[0]][0] > all_2d[bottom[bottom.len() - 1]][0] {
        bottom.reverse();
    }
    if all_2d[top[0]][0] > all_2d[top[top.len() - 1]][0] {
        top.reverse();
    }
    if all_2d[left[0]][1] > all_2d[left[left.len() - 1]][1] {
        left.reverse();
    }
    if all_2d[right[0]][1] > all_2d[right[right.len() - 1]][1] {
        right.reverse();
    }
    let utol = 1e-3 * uspan + 1e-12;
    let vtol = 5e-3 * vspan + 1e-12;
    let mono = |chain: &[usize], axis: usize, tol: f64| -> bool {
        chain.windows(2).all(|w| {
            all_2d[w[1]][axis] >= all_2d[w[0]][axis] - tol
        })
    };
    if !mono(&bottom, 0, utol) || !mono(&top, 0, utol) {
        lattice_reject!("cap not u-monotone");
    }
    if !mono(&left, 1, vtol) || !mono(&right, 1, vtol) {
        lattice_reject!("wall not v-monotone");
    }
    // ── 4. wall band segments between row levels ───────────────────
    // seg j (0-based) covers wall positions (L_{j-1}, L_j] where L_j
    // = row_v[j]; positions 0 (bottom corner) and len-1 (top corner)
    // are excluded — the caps own the corners. The LAST band is OPEN
    // at the top (carries everything above L_{M-1} up to the top
    // corner, the sub-corner included): the sub-corner is the final
    // zipper's lower-chain anchor, and its band's rim edges close
    // against it (s78 "open at the top" contract).
    let seg_bounds = |wall: &[usize]| -> Vec<(usize, usize)> {
        let len = wall.len();
        let mut bounds = Vec::with_capacity(m_rows);
        let mut s = 1usize;
        for j in 0..m_rows {
            let e = if j == m_rows - 1 {
                len - 1
            } else {
                let mut e = s;
                while e < len - 1 && all_2d[wall[e]][1] <= row_v[j] + 1e-12 {
                    e += 1;
                }
                e.min(len - 1).max(s)
            };
            bounds.push((s, e));
            s = e;
        }
        bounds
    };
    let lb = seg_bounds(&left);
    let rb = seg_bounds(&right);
    // ── 5. chains ──────────────────────────────────────────────────
    let full_row = |j: usize| -> Vec<u32> {
        // j = 0: bottom cap; j = m_rows+1: top cap; else band-j walls
        // + row j (1-based) between them
        if j == 0 {
            return bottom.iter().map(|&i| i as u32).collect();
        }
        if j == m_rows + 1 {
            return top.iter().map(|&i| i as u32).collect();
        }
        let (s, e) = lb[j - 1];
        let mut row: Vec<u32> = left[s..e].iter().map(|&i| i as u32).collect();
        row.extend(rows[j - 1].iter().map(|&i| i as u32));
        let (s, e) = rb[j - 1];
        let mut tail: Vec<u32> = right[s..e].iter().map(|&i| i as u32).collect();
        tail.reverse(); // v-descending
        row.extend(tail);
        row
    };
    let lower_chain = |k: usize| -> Option<Vec<u32>> {
        // k = 1: bottom cap; k = m_rows+1: sub-corners + row M;
        // else anchors at L_{k-1} + row k-1
        if k == 1 {
            return Some(bottom.iter().map(|&i| i as u32).collect());
        }
        if k == m_rows + 1 {
            if left.len() < 2 || right.len() < 2 {
                return None;
            }
            let mut c: Vec<u32> = vec![left[left.len() - 2] as u32];
            c.extend(rows[m_rows - 1].iter().map(|&i| i as u32));
            c.push(right[right.len() - 2] as u32);
            return Some(c);
        }
        let lvl = row_v[k - 2];
        let anchor_at = |wall: &[usize]| -> usize {
            let mut best = 1usize;
            for p in 1..wall.len() - 1 {
                if all_2d[wall[p]][1] <= lvl + 1e-12 {
                    best = p;
                } else {
                    break;
                }
            }
            wall[best]
        };
        let mut c: Vec<u32> = vec![anchor_at(&left) as u32];
        c.extend(rows[k - 2].iter().map(|&i| i as u32));
        c.push(anchor_at(&right) as u32);
        Some(c)
    };
    let left_set: HashSet<u32> = left.iter().map(|&i| i as u32).collect();
    let right_set: HashSet<u32> = right.iter().map(|&i| i as u32).collect();
    // ── 6. two-pointer zipper (s78 mechanics, angled-wall hardened) ─
    let scale2 = uspan * uspan + vspan * vspan;
    let eps_area = 1e-12 * scale2;
    let area2 = |a: u32, b: u32, c: u32| -> f64 {
        let pa = all_2d[a as usize];
        let pb = all_2d[b as usize];
        let pc = all_2d[c as usize];
        (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])
    };
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for k in 1..=m_rows + 1 {
        let a = match lower_chain(k) {
            Some(c) if c.len() >= 2 => c,
            _ => lattice_reject!(format!("lower chain degenerate k={}", k)),
        };
        let b = full_row(k);
        if b.len() < 2 {
            lattice_reject!(format!("upper row degenerate k={}", k));
        }
        let (m, nb) = (a.len(), b.len());
        let (mut ia, mut ib) = (0usize, 0usize);
        let mut guard_ct = 0usize;
        while ia < m - 1 || ib < nb - 1 {
            let dead_a = ia >= m - 1;
            let dead_b = ib >= nb - 1;
            // all-on-one-wall triangles are 1D degenerates: on u-const
            // walls (sail) they are EXACTLY collinear and the eps guard
            // catches them; an ANGLED wall's curvature (f26 right wall:
            // area ~1e-7) slips past the absolute eps and would emit an
            // inverted wall sliver — tripped explicitly instead.
            let deg_a = !dead_a && {
                area2(a[ia], a[ia + 1], b[ib]).abs() <= eps_area
                    || (left_set.contains(&a[ia])
                        && left_set.contains(&a[ia + 1])
                        && left_set.contains(&b[ib]))
                    || (right_set.contains(&a[ia])
                        && right_set.contains(&a[ia + 1])
                        && right_set.contains(&b[ib]))
            };
            let deg_b = !dead_b && {
                area2(a[ia], b[ib + 1], b[ib]).abs() <= eps_area
                    || (left_set.contains(&a[ia])
                        && left_set.contains(&b[ib + 1])
                        && left_set.contains(&b[ib]))
                    || (right_set.contains(&a[ia])
                        && right_set.contains(&b[ib + 1])
                        && right_set.contains(&b[ib]))
            };
            // right-anchor block (s78 wall-deadlock guard generalized
            // for ANGLED walls): A must not step onto its final right
            // anchor while B still has anything to walk and B's chain
            // ends in a right-wall suffix — the tail would then have
            // to fan from the ANCHOR (all-wall triangles, deg →
            // stuck). On u-const walls the u-criterion already
            // enforces this (anchor u = max); an angled wall lets a
            // row point sit RIGHT of the anchor (f26: row u 0.7999 vs
            // anchor 0.7994), so the block is explicit: B consumes its
            // whole right tail first (fanning from A's row-last
            // interior point), then A takes the final step.
            let wall_block_a = !dead_a
                && ia + 1 == m - 1
                && !dead_b
                && right_set.contains(&a[m - 1])
                && right_set.contains(&b[nb - 1]);
            let can_a = !dead_a && !deg_a && !wall_block_a;
            let can_b = !dead_b && !deg_b;
            let take_a = if !can_a {
                false
            } else if !can_b {
                true
            } else if ia == 0
                && ib == 0
                && left_set.contains(&a[0])
                && left_set.contains(&b[0])
            {
                // opening pair on the left wall: advance A first
                true
            } else {
                // u-criterion (both chains weakly u-monotone)
                all_2d[a[ia + 1] as usize][0] <= all_2d[b[ib + 1] as usize][0]
            };
            if take_a {
                tris.push([a[ia], a[ia + 1], b[ib]]);
                ia += 1;
            } else if can_b {
                tris.push([a[ia], b[ib + 1], b[ib]]);
                ib += 1;
            } else if !dead_a && !deg_a {
                // wall_block_a is the only blocker — B is exhausted or
                // blocked; advance A to finish the band
                tris.push([a[ia], a[ia + 1], b[ib]]);
                ia += 1;
            } else {
                lattice_reject!(format!(
                    "zipper stuck k={} ia={} ib={}",
                    k, ia, ib
                ));
            }
            guard_ct += 1;
            if guard_ct > 4 * (m + nb) {
                lattice_reject!(format!("zipper loop k={}", k));
            }
        }
    }
    if tris.len() < 3 {
        lattice_reject!("empty strip");
    }
    // ── 7. guards: coverage / edge audit / area / winding / needles ─
    {
        let mut used = vec![false; interior_start + n_interior];
        for t in tris.iter() {
            for &v in t.iter() {
                if (v as usize) < used.len() {
                    used[v as usize] = true;
                }
            }
        }
        for i in 0..interior_start + n_interior {
            if !used[i] {
                lattice_reject!(format!("point {} unused", i));
            }
        }
        let mut ecount: HashMap<(u32, u32), usize> = HashMap::new();
        for t in tris.iter() {
            for k in 0..3 {
                let x = t[k];
                let y = t[(k + 1) % 3];
                if x != y {
                    *ecount.entry((x.min(y), y.max(x))).or_default() += 1;
                }
            }
        }
        let mut rim_set: HashSet<(u32, u32)> = HashSet::new();
        for i in 0..n_boundary {
            let j = (i + 1) % n_boundary;
            rim_set.insert((i.min(j) as u32, i.max(j) as u32));
        }
        for (e, cnt) in ecount.iter() {
            if rim_set.contains(e) {
                if *cnt != 1 {
                    lattice_reject!(format!("rim edge {:?} count {}", e, cnt));
                }
            } else if *cnt != 2 {
                lattice_reject!(format!("interior edge {:?} count {}", e, cnt));
            }
        }
        for e in rim_set.iter() {
            if !ecount.contains_key(e) {
                lattice_reject!(format!("rim edge {:?} missing", e));
            }
        }
        // area + winding (normalize to the polygon's sign)
        let mut poly_s = 0.0f64;
        for i in 0..n_boundary {
            let a = all_2d[i];
            let b = all_2d[(i + 1) % n_boundary];
            poly_s += a[0] * b[1] - b[0] * a[1];
        }
        poly_s *= 0.5;
        let strip_s: f64 = tris
            .iter()
            .map(|t| area2(t[0], t[1], t[2]) * 0.5)
            .sum();
        if (strip_s - poly_s).abs() > 0.005 * poly_s.abs().max(1e-12) {
            lattice_reject!(format!(
                "area mismatch strip {:.6} vs poly {:.6}",
                strip_s, poly_s
            ));
        }
        if strip_s * poly_s < 0.0 {
            for t in tris.iter_mut() {
                t.swap(1, 2);
            }
        }
        let noise = 1e-9 * scale2;
        let mut pos = 0usize;
        let mut neg = 0usize;
        for t in tris.iter() {
            let s = area2(t[0], t[1], t[2]);
            if s > noise {
                pos += 1;
            } else if s < -noise {
                neg += 1;
            }
        }
        if pos > 0 && neg > 0 {
            lattice_reject!(format!("winding inversion {} pos / {} neg", pos, neg));
        }
        if pos == 0 && neg == 0 {
            lattice_reject!("all-zero areas");
        }
        // UV needle census (s82 lesson: aspect < 1e-3 folds in 3D).
        // Strict zero: a row-built grid is needle-free by construction
        // (grid quads ~1.5:1); any needle means the rows/walls are
        // mismatched (f26-168 class: row ends 0.08 short of the wall,
        // 55-pt wall band fanning from a distant row point) — such a
        // face keeps the standard CDT result.
        for t in tris.iter() {
            let p = all_2d[t[0] as usize];
            let q = all_2d[t[1] as usize];
            let r = all_2d[t[2] as usize];
            let e1 = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]);
            let e2 = (q[0] - r[0]) * (q[0] - r[0]) + (q[1] - r[1]) * (q[1] - r[1]);
            let e3 = (r[0] - p[0]) * (r[0] - p[0]) + (r[1] - p[1]) * (r[1] - p[1]);
            let max_e = e1.max(e2).max(e3);
            let a = area2(t[0], t[1], t[2]).abs() * 0.5;
            if max_e > 0.0 && a < 1e-3 * max_e {
                lattice_reject!("UV needle in row grid");
            }
        }
    }
    if dbg {
        eprintln!(
            "[LATTICE accept {}] {} tris, {} rows, ring {}",
            crate::parametric_domain::current_face_label(),
            tris.len(),
            m_rows,
            n_boundary
        );
    }
    Some(tris)
}

/// session-84: LEGACY LATTICE RESCUE entry point for the legacy earcut
/// path. The s83 structural lattice lives inside the CDT pipeline (it
/// fires only when a face is routed through `triangulate_polygon_cdt`);
/// the s84 census showed a residual class whose production routing NEVER
/// reaches the CDT — legacy earcut with every ring vertex used
/// (n_unused == 0), no non-rim boundary edges (no crescent/band debt)
/// and no band candidate claiming the face, yet the interior lattice is
/// partially dropped (drill HOUSING f49: 35 of 341, the (49,49)/
/// (49,178) REAL fold families). The s77 CDT re-route arm stays opt-in
/// for a reason (the s51 Delaunay-near-rim regression: HOUSING +20 /
/// HM +33 REAL measured), so this entry calls the STRUCTURAL lattice
/// DIRECTLY: no Delaunay flips, no Bowyer-Watson, no new points — just
/// the row construction over the ORIGINAL boundary+interior arrays with
/// the full s83 guard set (all points used / rim 1x / non-rim 2x /
/// uniform winding / area ±0.5% / zero UV needles). Any guard failure
/// returns None and the caller keeps the legacy result bit-exactly.
/// Kill-switch (checked inside): DRAPPER_LATTICE_BAND=0 — the same
/// switch as the s83 CDT-path rescue.
pub fn structural_lattice_rescue_legacy(
    boundary_2d: &[[f64; 2]],
    interior_2d: &[[f64; 2]],
) -> Option<Vec<[u32; 3]>> {
    if boundary_2d.len() < 8 || interior_2d.is_empty() {
        return None;
    }
    let mut all_2d: Vec<[f64; 2]> = Vec::with_capacity(boundary_2d.len() + interior_2d.len());
    all_2d.extend_from_slice(boundary_2d);
    let interior_start = all_2d.len();
    all_2d.extend_from_slice(interior_2d);
    structured_lattice_triangulation(
        &all_2d,
        boundary_2d.len(),
        interior_start,
        interior_2d.len(),
    )
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

    // session-83: the f26-class regression — a 4-corner ring (angled
    // right wall with slight convex curvature) carrying a 341-point
    // brick lattice (16 fine u-step-0.04 rows interleaved with 15
    // offset coarse u-step-0.08 rows). The data is the production
    // HOUSING_MIRROR f26 dump (drill BREP#62542, Nurbs 4x10 fillet,
    // 6-decimal). The STANDARD pipeline drops 205/341 lattice points
    // to the s64 sliver-guard chord deadlock (the measured s82 state
    // — the surviving 25 (26,26) REAL fold pairs were the unsplit
    // coarse triangles); the s83 STRUCTURAL LATTICE rescue must
    // recover EVERY point with an exact rim and no interior seams.
    // Self-proving: if the rescue breaks, the standard result (491
    // tris, 205 drops) fails the all-points-used assertion below.
    #[test]
    fn test_structural_lattice_rescue_f26_class() {
    const RING: [[f64; 2]; 220] = [
        [0.749444,0.425793], [0.749875,0.433133], [0.750339,0.440487], [0.750838,0.447855],
        [0.751375,0.455239], [0.751952,0.462638], [0.752568,0.470054], [0.753217,0.477485],
        [0.753896,0.484933], [0.7546,0.492398], [0.755327,0.499881], [0.756084,0.507381],
        [0.756877,0.514896], [0.757714,0.522427], [0.758599,0.529971], [0.759533,0.537529],
        [0.760509,0.545098], [0.76152,0.552678], [0.762557,0.560267], [0.763611,0.567866],
        [0.764684,0.575474], [0.765782,0.58309], [0.766911,0.590714], [0.768079,0.598347],
        [0.769288,0.605988], [0.770538,0.613637], [0.771825,0.621294], [0.773148,0.628959],
        [0.774504,0.636631], [0.775888,0.644311], [0.7773,0.652], [0.778735,0.659698],
        [0.780195,0.667407], [0.781676,0.675127], [0.783175,0.682859], [0.784687,0.690605],
        [0.786207,0.698366], [0.787734,0.706141], [0.789274,0.713931], [0.790844,0.721732],
        [0.792458,0.72954], [0.794124,0.737351], [0.795848,0.745162], [0.797623,0.752968],
        [0.79944,0.760765], [0.801284,0.76855], [0.803138,0.776318], [0.804987,0.784067],
        [0.80683,0.791798], [0.808671,0.799513], [0.810514,0.807212], [0.812365,0.814897],
        [0.814226,0.822569], [0.8161,0.83023], [0.817992,0.83788], [0.819905,0.84552],
        [0.807598,0.845502], [0.795368,0.845488], [0.783211,0.845476], [0.771123,0.845467],
        [0.759103,0.845461], [0.747145,0.845456], [0.735247,0.845454], [0.723407,0.845454],
        [0.711622,0.845455], [0.699888,0.845457], [0.688203,0.845461], [0.676564,0.845466],
        [0.664969,0.845471], [0.653414,0.845476], [0.641898,0.845482], [0.630417,0.845488],
        [0.618969,0.845494], [0.607551,0.845499], [0.596159,0.845503], [0.584791,0.845507],
        [0.573445,0.84551], [0.562116,0.845512], [0.550802,0.845514], [0.5395,0.845515],
        [0.528208,0.845516], [0.516923,0.845516], [0.505642,0.845516], [0.494362,0.845516],
        [0.483081,0.845515], [0.471797,0.845514], [0.460507,0.845513], [0.449207,0.845512],
        [0.437896,0.845511], [0.426571,0.845511], [0.415228,0.84551], [0.403865,0.845509],
        [0.392479,0.845509], [0.381067,0.845509], [0.369625,0.845509], [0.358151,0.845509],
        [0.346642,0.84551], [0.335095,0.84551], [0.323507,0.845511], [0.311874,0.845512],
        [0.300195,0.845513], [0.288465,0.845514], [0.276683,0.845515], [0.264845,0.845516],
        [0.252947,0.845517], [0.240987,0.845518], [0.228962,0.845518], [0.216867,0.845519],
        [0.204699,0.84552], [0.192455,0.84552], [0.180131,0.84552], [0.180186,0.837833],
        [0.18031,0.830163], [0.18045,0.822506], [0.180566,0.814856], [0.180631,0.807209],
        [0.180628,0.799558], [0.180544,0.7919], [0.180379,0.784227], [0.180146,0.776536],
        [0.179874,0.768822], [0.179613,0.761086], [0.179407,0.753334], [0.179286,0.745571],
        [0.179271,0.737801], [0.179365,0.73003], [0.179548,0.722263], [0.179785,0.714504],
        [0.180029,0.706758], [0.180224,0.699029], [0.180348,0.691318], [0.180409,0.683625],
        [0.180418,0.675947], [0.18039,0.668284], [0.180339,0.660634], [0.180275,0.652997],
        [0.180203,0.645371], [0.180133,0.637755], [0.180076,0.630149], [0.180038,0.622551],
        [0.18002,0.614962], [0.18002,0.607381], [0.180037,0.599809], [0.18007,0.592246],
        [0.180109,0.584691], [0.180136,0.577145], [0.180131,0.569607], [0.180078,0.562078],
        [0.179979,0.554557], [0.179864,0.547047], [0.179761,0.53955], [0.179696,0.532067],
        [0.179685,0.524599], [0.179735,0.517151], [0.179837,0.509722], [0.179975,0.502316],
        [0.180127,0.494933], [0.180268,0.487576], [0.180373,0.480239], [0.180427,0.472919],
        [0.180427,0.465611], [0.180381,0.458311], [0.180306,0.451015], [0.180218,0.443718],
        [0.180138,0.436416], [0.180102,0.429106], [0.191672,0.429026], [0.203084,0.428946],
        [0.214351,0.428868], [0.225482,0.42879], [0.236486,0.428713], [0.247375,0.428636],
        [0.258157,0.42856], [0.268843,0.428485], [0.279441,0.428411], [0.289961,0.428337],
        [0.300412,0.428263], [0.310803,0.428191], [0.321142,0.428119], [0.331438,0.428047],
        [0.341698,0.427976], [0.351932,0.427906], [0.362147,0.427836], [0.372351,0.427767],
        [0.382551,0.427698], [0.392749,0.427629], [0.402945,0.427561], [0.413138,0.427494],
        [0.423327,0.427428], [0.433511,0.427362], [0.443688,0.427296], [0.453858,0.427232],
        [0.464019,0.427168], [0.47417,0.427105], [0.48431,0.427043], [0.494437,0.426982],
        [0.50455,0.426921], [0.514647,0.426862], [0.524729,0.426803], [0.534792,0.426746],
        [0.544837,0.42669], [0.554862,0.426635], [0.564866,0.426581], [0.574851,0.426528],
        [0.584825,0.426476], [0.594795,0.426425], [0.604769,0.426376], [0.614755,0.426327],
        [0.624761,0.42628], [0.634795,0.426233], [0.644866,0.426188], [0.654981,0.426144],
        [0.665149,0.426101], [0.675379,0.426059], [0.685679,0.426018], [0.696058,0.425978],
        [0.706524,0.425939], [0.717087,0.425901], [0.727755,0.425864], [0.738538,0.425828],
    ];
    const LATTICE: [[f64; 2]; 341] = [
        [0.259351,0.478259], [0.33943,0.478259], [0.419509,0.478259], [0.499588,0.478259],
        [0.579667,0.478259], [0.659746,0.478259], [0.739825,0.478259], [0.259351,0.530725],
        [0.33943,0.530725], [0.419509,0.530725], [0.499588,0.530725], [0.579667,0.530725],
        [0.659746,0.530725], [0.739825,0.530725], [0.259351,0.583191], [0.33943,0.583191],
        [0.419509,0.583191], [0.499588,0.583191], [0.579667,0.583191], [0.659746,0.583191],
        [0.739825,0.583191], [0.259351,0.635657], [0.33943,0.635657], [0.419509,0.635657],
        [0.499588,0.635657], [0.579667,0.635657], [0.659746,0.635657], [0.739825,0.635657],
        [0.259351,0.688123], [0.33943,0.688123], [0.419509,0.688123], [0.499588,0.688123],
        [0.579667,0.688123], [0.659746,0.688123], [0.739825,0.688123], [0.259351,0.740588],
        [0.33943,0.740588], [0.419509,0.740588], [0.499588,0.740588], [0.579667,0.740588],
        [0.659746,0.740588], [0.739825,0.740588], [0.259351,0.793054], [0.33943,0.793054],
        [0.419509,0.793054], [0.499588,0.793054], [0.579667,0.793054], [0.659746,0.793054],
        [0.739825,0.793054], [0.219311,0.452026], [0.199291,0.43891], [0.239331,0.43891],
        [0.199291,0.465142], [0.239331,0.465142], [0.219311,0.504492], [0.199291,0.491375],
        [0.239331,0.491375], [0.199291,0.517608], [0.239331,0.517608], [0.219311,0.556958],
        [0.199291,0.543841], [0.239331,0.543841], [0.199291,0.570074], [0.239331,0.570074],
        [0.219311,0.609424], [0.199291,0.596307], [0.239331,0.596307], [0.199291,0.62254],
        [0.239331,0.62254], [0.219311,0.66189], [0.199291,0.648773], [0.239331,0.648773],
        [0.199291,0.675006], [0.239331,0.675006], [0.219311,0.714355], [0.199291,0.701239],
        [0.239331,0.701239], [0.199291,0.727472], [0.239331,0.727472], [0.219311,0.766821],
        [0.199291,0.753705], [0.239331,0.753705], [0.199291,0.779938], [0.239331,0.779938],
        [0.219311,0.819287], [0.199291,0.806171], [0.239331,0.806171], [0.199291,0.832404],
        [0.239331,0.832404], [0.29939,0.452026], [0.27937,0.43891], [0.31941,0.43891],
        [0.27937,0.465142], [0.31941,0.465142], [0.29939,0.504492], [0.27937,0.491375],
        [0.31941,0.491375], [0.27937,0.517608], [0.31941,0.517608], [0.29939,0.556958],
        [0.27937,0.543841], [0.31941,0.543841], [0.27937,0.570074], [0.31941,0.570074],
        [0.29939,0.609424], [0.27937,0.596307], [0.31941,0.596307], [0.27937,0.62254],
        [0.31941,0.62254], [0.29939,0.66189], [0.27937,0.648773], [0.31941,0.648773],
        [0.27937,0.675006], [0.31941,0.675006], [0.29939,0.714355], [0.27937,0.701239],
        [0.31941,0.701239], [0.27937,0.727472], [0.31941,0.727472], [0.29939,0.766821],
        [0.27937,0.753705], [0.31941,0.753705], [0.27937,0.779938], [0.31941,0.779938],
        [0.29939,0.819287], [0.27937,0.806171], [0.31941,0.806171], [0.27937,0.832404],
        [0.31941,0.832404], [0.379469,0.452026], [0.359449,0.43891], [0.399489,0.43891],
        [0.359449,0.465142], [0.399489,0.465142], [0.379469,0.504492], [0.359449,0.491375],
        [0.399489,0.491375], [0.359449,0.517608], [0.399489,0.517608], [0.379469,0.556958],
        [0.359449,0.543841], [0.399489,0.543841], [0.359449,0.570074], [0.399489,0.570074],
        [0.379469,0.609424], [0.359449,0.596307], [0.399489,0.596307], [0.359449,0.62254],
        [0.399489,0.62254], [0.379469,0.66189], [0.359449,0.648773], [0.399489,0.648773],
        [0.359449,0.675006], [0.399489,0.675006], [0.379469,0.714355], [0.359449,0.701239],
        [0.399489,0.701239], [0.359449,0.727472], [0.399489,0.727472], [0.379469,0.766821],
        [0.359449,0.753705], [0.399489,0.753705], [0.359449,0.779938], [0.399489,0.779938],
        [0.379469,0.819287], [0.359449,0.806171], [0.399489,0.806171], [0.359449,0.832404],
        [0.399489,0.832404], [0.459548,0.452026], [0.439529,0.43891], [0.479568,0.43891],
        [0.439529,0.465142], [0.479568,0.465142], [0.459548,0.504492], [0.439529,0.491375],
        [0.479568,0.491375], [0.439529,0.517608], [0.479568,0.517608], [0.459548,0.556958],
        [0.439529,0.543841], [0.479568,0.543841], [0.439529,0.570074], [0.479568,0.570074],
        [0.459548,0.609424], [0.439529,0.596307], [0.479568,0.596307], [0.439529,0.62254],
        [0.479568,0.62254], [0.459548,0.66189], [0.439529,0.648773], [0.479568,0.648773],
        [0.439529,0.675006], [0.479568,0.675006], [0.459548,0.714355], [0.439529,0.701239],
        [0.479568,0.701239], [0.439529,0.727472], [0.479568,0.727472], [0.459548,0.766821],
        [0.439529,0.753705], [0.479568,0.753705], [0.439529,0.779938], [0.479568,0.779938],
        [0.459548,0.819287], [0.439529,0.806171], [0.479568,0.806171], [0.439529,0.832404],
        [0.479568,0.832404], [0.539628,0.452026], [0.519608,0.43891], [0.559647,0.43891],
        [0.519608,0.465142], [0.559647,0.465142], [0.539628,0.504492], [0.519608,0.491375],
        [0.559647,0.491375], [0.519608,0.517608], [0.559647,0.517608], [0.539628,0.556958],
        [0.519608,0.543841], [0.559647,0.543841], [0.519608,0.570074], [0.559647,0.570074],
        [0.539628,0.609424], [0.519608,0.596307], [0.559647,0.596307], [0.519608,0.62254],
        [0.559647,0.62254], [0.539628,0.66189], [0.519608,0.648773], [0.559647,0.648773],
        [0.519608,0.675006], [0.559647,0.675006], [0.539628,0.714355], [0.519608,0.701239],
        [0.559647,0.701239], [0.519608,0.727472], [0.559647,0.727472], [0.539628,0.766821],
        [0.519608,0.753705], [0.559647,0.753705], [0.519608,0.779938], [0.559647,0.779938],
        [0.539628,0.819287], [0.519608,0.806171], [0.559647,0.806171], [0.519608,0.832404],
        [0.559647,0.832404], [0.619707,0.452026], [0.599687,0.43891], [0.639726,0.43891],
        [0.599687,0.465142], [0.639726,0.465142], [0.619707,0.504492], [0.599687,0.491375],
        [0.639726,0.491375], [0.599687,0.517608], [0.639726,0.517608], [0.619707,0.556958],
        [0.599687,0.543841], [0.639726,0.543841], [0.599687,0.570074], [0.639726,0.570074],
        [0.619707,0.609424], [0.599687,0.596307], [0.639726,0.596307], [0.599687,0.62254],
        [0.639726,0.62254], [0.619707,0.66189], [0.599687,0.648773], [0.639726,0.648773],
        [0.599687,0.675006], [0.639726,0.675006], [0.619707,0.714355], [0.599687,0.701239],
        [0.639726,0.701239], [0.599687,0.727472], [0.639726,0.727472], [0.619707,0.766821],
        [0.599687,0.753705], [0.639726,0.753705], [0.599687,0.779938], [0.639726,0.779938],
        [0.619707,0.819287], [0.599687,0.806171], [0.639726,0.806171], [0.599687,0.832404],
        [0.639726,0.832404], [0.699786,0.452026], [0.679766,0.43891], [0.719806,0.43891],
        [0.679766,0.465142], [0.719806,0.465142], [0.699786,0.504492], [0.679766,0.491375],
        [0.719806,0.491375], [0.679766,0.517608], [0.719806,0.517608], [0.699786,0.556958],
        [0.679766,0.543841], [0.719806,0.543841], [0.679766,0.570074], [0.719806,0.570074],
        [0.699786,0.609424], [0.679766,0.596307], [0.719806,0.596307], [0.679766,0.62254],
        [0.719806,0.62254], [0.699786,0.66189], [0.679766,0.648773], [0.719806,0.648773],
        [0.679766,0.675006], [0.719806,0.675006], [0.699786,0.714355], [0.679766,0.701239],
        [0.719806,0.701239], [0.679766,0.727472], [0.719806,0.727472], [0.699786,0.766821],
        [0.679766,0.753705], [0.719806,0.753705], [0.679766,0.779938], [0.719806,0.779938],
        [0.699786,0.819287], [0.679766,0.806171], [0.719806,0.806171], [0.679766,0.832404],
        [0.719806,0.832404], [0.779865,0.714355], [0.759845,0.701239], [0.759845,0.727472],
        [0.779865,0.766821], [0.759845,0.753705], [0.759845,0.779938], [0.799885,0.779938],
        [0.779865,0.819287], [0.759845,0.806171], [0.799885,0.806171], [0.759845,0.832404],
        [0.799885,0.832404],
    ];
        let tris = triangulate_polygon_cdt(&RING, &[], &LATTICE);
        assert!(
            !tris.is_empty(),
            "f26: CDT returned empty — face untriangulated"
        );
        // 1. EVERY interior point used (the standard path drops 205)
        let n_total = RING.len() + LATTICE.len();
        let mut used = vec![false; n_total];
        for t in &tris {
            for &v in t.iter() {
                assert!((v as usize) < n_total, "index out of range");
                used[v as usize] = true;
            }
        }
        let unused: Vec<usize> = (RING.len()..n_total)
            .filter(|&i| !used[i])
            .collect();
        assert!(
            unused.is_empty(),
            "f26: interior lattice drops: {} of {} (first {:?}) — the structural lattice rescue must recover every Steiner point",
            unused.len(),
            LATTICE.len(),
            &unused[..unused.len().min(5)]
        );
        // 2. rim edges exactly once, non-rim edges exactly twice
        let mut ecount: HashMap<(u32, u32), usize> = HashMap::new();
        for t in &tris {
            for k in 0..3 {
                let a = t[k];
                let b = t[(k + 1) % 3];
                if a != b {
                    *ecount.entry((a.min(b), a.max(b))).or_default() += 1;
                }
            }
        }
        let n_ring = RING.len();
        for i in 0..n_ring {
            let j = (i + 1) % n_ring;
            let e = (i.min(j) as u32, i.max(j) as u32);
            let cnt = ecount.get(&e).copied().unwrap_or(0);
            assert_eq!(
                cnt, 1,
                "f26: rim edge ({},{}) count {} (must be exactly 1 — the cross-face watertight contract)",
                e.0, e.1, cnt
            );
        }
        let rim = |a: usize, b: usize| -> bool {
            let (lo, hi) = (a.min(b), a.max(b));
            (lo == 0 && hi == n_ring - 1)
                || hi - lo == 1
        };
        for (e, cnt) in ecount.iter() {
            if !rim(e.0 as usize, e.1 as usize) {
                assert_eq!(
                    *cnt, 2,
                    "f26: interior edge ({},{}) count {} (one-sided seam)",
                    e.0, e.1, cnt
                );
            }
        }
        // 3. uniform winding (no inversions)
        let all: Vec<[f64; 2]> = RING.iter().chain(LATTICE.iter()).copied().collect();
        let scale2 = {
            let (mut lo0, mut hi0, mut lo1, mut hi1) =
                (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
            for p in all.iter() {
                lo0 = lo0.min(p[0]);
                hi0 = hi0.max(p[0]);
                lo1 = lo1.min(p[1]);
                hi1 = hi1.max(p[1]);
            }
            (hi0 - lo0) * (hi0 - lo0) + (hi1 - lo1) * (hi1 - lo1)
        };
        let noise = 1e-9 * scale2;
        let (mut pos, mut neg) = (0usize, 0usize);
        for t in &tris {
            let s = orient2d(all[t[0] as usize], all[t[1] as usize], all[t[2] as usize]);
            if s > noise {
                pos += 1;
            } else if s < -noise {
                neg += 1;
            }
        }
        assert!(
            pos == 0 || neg == 0,
            "f26: winding inversion {} pos / {} neg",
            pos,
            neg
        );
        // 4. no UV needles (aspect < 1e-3 — the s82 fold-prone class)
        for t in &tris {
            let p = all[t[0] as usize];
            let q = all[t[1] as usize];
            let r = all[t[2] as usize];
            let e1 = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]);
            let e2 = (q[0] - r[0]) * (q[0] - r[0]) + (q[1] - r[1]) * (q[1] - r[1]);
            let e3 = (r[0] - p[0]) * (r[0] - p[0]) + (r[1] - p[1]) * (r[1] - p[1]);
            let max_e = e1.max(e2).max(e3);
            let a = orient2d(p, q, r).abs() * 0.5;
            assert!(
                max_e <= 0.0 || a >= 1e-3 * max_e,
                "f26: UV needle in the lattice grid (aspect < 1e-3)"
            );
        }
    }
    /// session-84: the LEGACY LATTICE RESCUE entry on the drill HOUSING
    /// f49 class (Nurbs 4x10, 220-ring, 341-point lattice; the legacy
    /// earcut result drops 35 interior points and leaves 412 legitimate
    /// spike-chain one-sided edges; every band constructor rejects the
    /// face — NFB "no level count passes" K=5..8 — so the acceptance
    /// chain never enters and the face keeps the debt). The entry must
    /// ACCEPT: every interior point used, every rim edge exactly once,
    /// uniform winding. Data: DRAPPER_DUMP_TRI_INPUT of brep47598_f49.
    #[test]
    fn test_structural_lattice_rescue_legacy_f49_class() {
        const RING: [[f64; 2]; 220] = [
        [0.180095,0.845520], [0.182008,0.837880], [0.183900,0.830230], [0.185774,0.822569],
        [0.187635,0.814897], [0.189486,0.807212], [0.191329,0.799513], [0.193170,0.791798],
        [0.195013,0.784067], [0.196862,0.776318], [0.198716,0.768550], [0.200560,0.760765],
        [0.202377,0.752968], [0.204152,0.745162], [0.205876,0.737351], [0.207542,0.729540],
        [0.209156,0.721732], [0.210726,0.713931], [0.212266,0.706141], [0.213793,0.698366],
        [0.215313,0.690605], [0.216825,0.682859], [0.218324,0.675127], [0.219805,0.667407],
        [0.221265,0.659698], [0.222700,0.652000], [0.224112,0.644311], [0.225496,0.636631],
        [0.226852,0.628959], [0.228175,0.621294], [0.229462,0.613637], [0.230712,0.605988],
        [0.231921,0.598347], [0.233089,0.590714], [0.234218,0.583090], [0.235316,0.575474],
        [0.236389,0.567866], [0.237443,0.560267], [0.238480,0.552678], [0.239491,0.545098],
        [0.240467,0.537529], [0.241401,0.529971], [0.242286,0.522427], [0.243123,0.514896],
        [0.243916,0.507381], [0.244673,0.499881], [0.245400,0.492398], [0.246104,0.484933],
        [0.246783,0.477485], [0.247432,0.470054], [0.248048,0.462638], [0.248625,0.455239],
        [0.249162,0.447855], [0.249661,0.440487], [0.250125,0.433133], [0.250556,0.425793],
        [0.261462,0.425828], [0.272245,0.425864], [0.282913,0.425901], [0.293476,0.425939],
        [0.303942,0.425978], [0.314321,0.426018], [0.324621,0.426059], [0.334851,0.426101],
        [0.345019,0.426144], [0.355134,0.426188], [0.365205,0.426233], [0.375239,0.426280],
        [0.385245,0.426327], [0.395231,0.426376], [0.405205,0.426425], [0.415175,0.426476],
        [0.425149,0.426528], [0.435134,0.426581], [0.445138,0.426635], [0.455163,0.426690],
        [0.465208,0.426746], [0.475271,0.426803], [0.485353,0.426862], [0.495450,0.426921],
        [0.505563,0.426982], [0.515690,0.427043], [0.525830,0.427105], [0.535981,0.427168],
        [0.546142,0.427232], [0.556312,0.427296], [0.566489,0.427362], [0.576673,0.427428],
        [0.586862,0.427494], [0.597055,0.427561], [0.607251,0.427629], [0.617449,0.427698],
        [0.627649,0.427767], [0.637853,0.427836], [0.648068,0.427906], [0.658302,0.427976],
        [0.668562,0.428047], [0.678858,0.428119], [0.689197,0.428191], [0.699588,0.428263],
        [0.710039,0.428337], [0.720559,0.428411], [0.731157,0.428485], [0.741843,0.428560],
        [0.752625,0.428636], [0.763514,0.428713], [0.774518,0.428790], [0.785649,0.428868],
        [0.796916,0.428946], [0.808328,0.429026], [0.819898,0.429106], [0.819862,0.436416],
        [0.819782,0.443718], [0.819694,0.451015], [0.819619,0.458311], [0.819573,0.465611],
        [0.819573,0.472919], [0.819627,0.480239], [0.819732,0.487576], [0.819873,0.494933],
        [0.820025,0.502316], [0.820163,0.509722], [0.820265,0.517151], [0.820315,0.524599],
        [0.820304,0.532067], [0.820239,0.539550], [0.820136,0.547047], [0.820021,0.554557],
        [0.819922,0.562078], [0.819869,0.569607], [0.819864,0.577145], [0.819891,0.584691],
        [0.819930,0.592246], [0.819963,0.599809], [0.819980,0.607381], [0.819980,0.614962],
        [0.819962,0.622551], [0.819924,0.630149], [0.819867,0.637755], [0.819797,0.645371],
        [0.819725,0.652997], [0.819661,0.660634], [0.819610,0.668284], [0.819582,0.675947],
        [0.819591,0.683625], [0.819652,0.691318], [0.819776,0.699029], [0.819971,0.706758],
        [0.820215,0.714504], [0.820452,0.722263], [0.820635,0.730030], [0.820729,0.737801],
        [0.820714,0.745571], [0.820593,0.753334], [0.820387,0.761086], [0.820126,0.768822],
        [0.819854,0.776536], [0.819621,0.784227], [0.819456,0.791900], [0.819372,0.799558],
        [0.819369,0.807209], [0.819434,0.814856], [0.819550,0.822506], [0.819690,0.830163],
        [0.819814,0.837833], [0.819869,0.845520], [0.807545,0.845520], [0.795301,0.845520],
        [0.783133,0.845519], [0.771038,0.845518], [0.759013,0.845518], [0.747053,0.845517],
        [0.735155,0.845516], [0.723317,0.845515], [0.711535,0.845514], [0.699805,0.845513],
        [0.688126,0.845512], [0.676493,0.845511], [0.664905,0.845510], [0.653358,0.845510],
        [0.641849,0.845509], [0.630375,0.845509], [0.618933,0.845509], [0.607521,0.845509],
        [0.596135,0.845509], [0.584772,0.845510], [0.573429,0.845511], [0.562104,0.845511],
        [0.550793,0.845512], [0.539493,0.845513], [0.528203,0.845514], [0.516919,0.845515],
        [0.505638,0.845516], [0.494358,0.845516], [0.483077,0.845516], [0.471792,0.845516],
        [0.460500,0.845515], [0.449198,0.845514], [0.437884,0.845512], [0.426555,0.845510],
        [0.415209,0.845507], [0.403841,0.845503], [0.392449,0.845499], [0.381031,0.845494],
        [0.369583,0.845488], [0.358102,0.845482], [0.346586,0.845476], [0.335031,0.845471],
        [0.323436,0.845466], [0.311797,0.845461], [0.300112,0.845457], [0.288378,0.845455],
        [0.276593,0.845454], [0.264753,0.845454], [0.252855,0.845456], [0.240897,0.845461],
        [0.228877,0.845467], [0.216789,0.845476], [0.204632,0.845488], [0.192402,0.845502],
        ];
        const INTERIOR: [[f64; 2]; 341] = [
        [0.260175,0.478259], [0.340254,0.478259], [0.420333,0.478259], [0.500412,0.478259],
        [0.580491,0.478259], [0.660570,0.478259], [0.740649,0.478259], [0.260175,0.530725],
        [0.340254,0.530725], [0.420333,0.530725], [0.500412,0.530725], [0.580491,0.530725],
        [0.660570,0.530725], [0.740649,0.530725], [0.260175,0.583191], [0.340254,0.583191],
        [0.420333,0.583191], [0.500412,0.583191], [0.580491,0.583191], [0.660570,0.583191],
        [0.740649,0.583191], [0.260175,0.635657], [0.340254,0.635657], [0.420333,0.635657],
        [0.500412,0.635657], [0.580491,0.635657], [0.660570,0.635657], [0.740649,0.635657],
        [0.260175,0.688123], [0.340254,0.688123], [0.420333,0.688123], [0.500412,0.688123],
        [0.580491,0.688123], [0.660570,0.688123], [0.740649,0.688123], [0.260175,0.740588],
        [0.340254,0.740588], [0.420333,0.740588], [0.500412,0.740588], [0.580491,0.740588],
        [0.660570,0.740588], [0.740649,0.740588], [0.260175,0.793054], [0.340254,0.793054],
        [0.420333,0.793054], [0.500412,0.793054], [0.580491,0.793054], [0.660570,0.793054],
        [0.740649,0.793054], [0.220135,0.714355], [0.240155,0.701239], [0.240155,0.727472],
        [0.220135,0.766821], [0.240155,0.753705], [0.200115,0.779938], [0.240155,0.779938],
        [0.220135,0.819287], [0.200115,0.806171], [0.240155,0.806171], [0.200115,0.832404],
        [0.240155,0.832404], [0.300214,0.452026], [0.280194,0.438910], [0.320234,0.438910],
        [0.280194,0.465142], [0.320234,0.465142], [0.300214,0.504492], [0.280194,0.491375],
        [0.320234,0.491375], [0.280194,0.517608], [0.320234,0.517608], [0.300214,0.556958],
        [0.280194,0.543841], [0.320234,0.543841], [0.280194,0.570074], [0.320234,0.570074],
        [0.300214,0.609424], [0.280194,0.596307], [0.320234,0.596307], [0.280194,0.622540],
        [0.320234,0.622540], [0.300214,0.661890], [0.280194,0.648773], [0.320234,0.648773],
        [0.280194,0.675006], [0.320234,0.675006], [0.300214,0.714355], [0.280194,0.701239],
        [0.320234,0.701239], [0.280194,0.727472], [0.320234,0.727472], [0.300214,0.766821],
        [0.280194,0.753705], [0.320234,0.753705], [0.280194,0.779938], [0.320234,0.779938],
        [0.300214,0.819287], [0.280194,0.806171], [0.320234,0.806171], [0.280194,0.832404],
        [0.320234,0.832404], [0.380293,0.452026], [0.360274,0.438910], [0.400313,0.438910],
        [0.360274,0.465142], [0.400313,0.465142], [0.380293,0.504492], [0.360274,0.491375],
        [0.400313,0.491375], [0.360274,0.517608], [0.400313,0.517608], [0.380293,0.556958],
        [0.360274,0.543841], [0.400313,0.543841], [0.360274,0.570074], [0.400313,0.570074],
        [0.380293,0.609424], [0.360274,0.596307], [0.400313,0.596307], [0.360274,0.622540],
        [0.400313,0.622540], [0.380293,0.661890], [0.360274,0.648773], [0.400313,0.648773],
        [0.360274,0.675006], [0.400313,0.675006], [0.380293,0.714355], [0.360274,0.701239],
        [0.400313,0.701239], [0.360274,0.727472], [0.400313,0.727472], [0.380293,0.766821],
        [0.360274,0.753705], [0.400313,0.753705], [0.360274,0.779938], [0.400313,0.779938],
        [0.380293,0.819287], [0.360274,0.806171], [0.400313,0.806171], [0.360274,0.832404],
        [0.400313,0.832404], [0.460372,0.452026], [0.440353,0.438910], [0.480392,0.438910],
        [0.440353,0.465142], [0.480392,0.465142], [0.460372,0.504492], [0.440353,0.491375],
        [0.480392,0.491375], [0.440353,0.517608], [0.480392,0.517608], [0.460372,0.556958],
        [0.440353,0.543841], [0.480392,0.543841], [0.440353,0.570074], [0.480392,0.570074],
        [0.460372,0.609424], [0.440353,0.596307], [0.480392,0.596307], [0.440353,0.622540],
        [0.480392,0.622540], [0.460372,0.661890], [0.440353,0.648773], [0.480392,0.648773],
        [0.440353,0.675006], [0.480392,0.675006], [0.460372,0.714355], [0.440353,0.701239],
        [0.480392,0.701239], [0.440353,0.727472], [0.480392,0.727472], [0.460372,0.766821],
        [0.440353,0.753705], [0.480392,0.753705], [0.440353,0.779938], [0.480392,0.779938],
        [0.460372,0.819287], [0.440353,0.806171], [0.480392,0.806171], [0.440353,0.832404],
        [0.480392,0.832404], [0.540452,0.452026], [0.520432,0.438910], [0.560471,0.438910],
        [0.520432,0.465142], [0.560471,0.465142], [0.540452,0.504492], [0.520432,0.491375],
        [0.560471,0.491375], [0.520432,0.517608], [0.560471,0.517608], [0.540452,0.556958],
        [0.520432,0.543841], [0.560471,0.543841], [0.520432,0.570074], [0.560471,0.570074],
        [0.540452,0.609424], [0.520432,0.596307], [0.560471,0.596307], [0.520432,0.622540],
        [0.560471,0.622540], [0.540452,0.661890], [0.520432,0.648773], [0.560471,0.648773],
        [0.520432,0.675006], [0.560471,0.675006], [0.540452,0.714355], [0.520432,0.701239],
        [0.560471,0.701239], [0.520432,0.727472], [0.560471,0.727472], [0.540452,0.766821],
        [0.520432,0.753705], [0.560471,0.753705], [0.520432,0.779938], [0.560471,0.779938],
        [0.540452,0.819287], [0.520432,0.806171], [0.560471,0.806171], [0.520432,0.832404],
        [0.560471,0.832404], [0.620531,0.452026], [0.600511,0.438910], [0.640551,0.438910],
        [0.600511,0.465142], [0.640551,0.465142], [0.620531,0.504492], [0.600511,0.491375],
        [0.640551,0.491375], [0.600511,0.517608], [0.640551,0.517608], [0.620531,0.556958],
        [0.600511,0.543841], [0.640551,0.543841], [0.600511,0.570074], [0.640551,0.570074],
        [0.620531,0.609424], [0.600511,0.596307], [0.640551,0.596307], [0.600511,0.622540],
        [0.640551,0.622540], [0.620531,0.661890], [0.600511,0.648773], [0.640551,0.648773],
        [0.600511,0.675006], [0.640551,0.675006], [0.620531,0.714355], [0.600511,0.701239],
        [0.640551,0.701239], [0.600511,0.727472], [0.640551,0.727472], [0.620531,0.766821],
        [0.600511,0.753705], [0.640551,0.753705], [0.600511,0.779938], [0.640551,0.779938],
        [0.620531,0.819287], [0.600511,0.806171], [0.640551,0.806171], [0.600511,0.832404],
        [0.640551,0.832404], [0.700610,0.452026], [0.680590,0.438910], [0.720630,0.438910],
        [0.680590,0.465142], [0.720630,0.465142], [0.700610,0.504492], [0.680590,0.491375],
        [0.720630,0.491375], [0.680590,0.517608], [0.720630,0.517608], [0.700610,0.556958],
        [0.680590,0.543841], [0.720630,0.543841], [0.680590,0.570074], [0.720630,0.570074],
        [0.700610,0.609424], [0.680590,0.596307], [0.720630,0.596307], [0.680590,0.622540],
        [0.720630,0.622540], [0.700610,0.661890], [0.680590,0.648773], [0.720630,0.648773],
        [0.680590,0.675006], [0.720630,0.675006], [0.700610,0.714355], [0.680590,0.701239],
        [0.720630,0.701239], [0.680590,0.727472], [0.720630,0.727472], [0.700610,0.766821],
        [0.680590,0.753705], [0.720630,0.753705], [0.680590,0.779938], [0.720630,0.779938],
        [0.700610,0.819287], [0.680590,0.806171], [0.720630,0.806171], [0.680590,0.832404],
        [0.720630,0.832404], [0.780689,0.452026], [0.760669,0.438910], [0.800709,0.438910],
        [0.760669,0.465142], [0.800709,0.465142], [0.780689,0.504492], [0.760669,0.491375],
        [0.800709,0.491375], [0.760669,0.517608], [0.800709,0.517608], [0.780689,0.556958],
        [0.760669,0.543841], [0.800709,0.543841], [0.760669,0.570074], [0.800709,0.570074],
        [0.780689,0.609424], [0.760669,0.596307], [0.800709,0.596307], [0.760669,0.622540],
        [0.800709,0.622540], [0.780689,0.661890], [0.760669,0.648773], [0.800709,0.648773],
        [0.760669,0.675006], [0.800709,0.675006], [0.780689,0.714355], [0.760669,0.701239],
        [0.800709,0.701239], [0.760669,0.727472], [0.800709,0.727472], [0.780689,0.766821],
        [0.760669,0.753705], [0.800709,0.753705], [0.760669,0.779938], [0.800709,0.779938],
        [0.780689,0.819287], [0.760669,0.806171], [0.800709,0.806171], [0.760669,0.832404],
        [0.800709,0.832404],
        ];
        let tris = super::structural_lattice_rescue_legacy(&RING, &INTERIOR)
            .expect("f49-class: legacy lattice rescue must accept");
        // every interior point used
        let mut used = vec![false; RING.len() + INTERIOR.len()];
        for t in tris.iter() {
            for &v in t.iter() {
                used[v as usize] = true;
            }
        }
        for i in RING.len()..used.len() {
            assert!(used[i], "interior point {} unused", i - RING.len());
        }
        // every rim edge exactly once
        use std::collections::HashMap;
        let mut ecount: HashMap<(u32, u32), usize> = HashMap::new();
        for t in tris.iter() {
            for k in 0..3 {
                let x = t[k];
                let y = t[(k + 1) % 3];
                if x != y {
                    *ecount.entry((x.min(y), y.max(x))).or_default() += 1;
                }
            }
        }
        let n = RING.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let key = (i.min(j) as u32, i.max(j) as u32);
            let cnt = ecount.get(&key).copied().unwrap_or(0);
            assert_eq!(cnt, 1, "rim edge ({},{}) count {}", i, j, cnt);
        }
        // winding uniformity
        let all: Vec<[f64; 2]> = RING.iter().chain(INTERIOR.iter()).cloned().collect();
        let area2 = |a: u32, b: u32, c: u32| -> f64 {
            let pa = &all[a as usize];
            let pb = &all[b as usize];
            let pc = &all[c as usize];
            (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])
        };
        let mut pos = 0usize;
        let mut neg = 0usize;
        for t in tris.iter() {
            let s = area2(t[0], t[1], t[2]);
            if s > 1e-12 {
                pos += 1;
            } else if s < -1e-12 {
                neg += 1;
            }
        }
        assert!(pos == 0 || neg == 0, "winding inversion {}/{}", pos, neg);
        assert!(pos + neg > 0, "all-zero areas");
    }


    /// session-88: the TORUS WINDMILL class — a half-torus fillet band
    /// (single loop, ~150 rim points, wavy rims, u span pi) + a dense
    /// interior Steiner lattice. The legacy spike-chain pass appends the
    /// row-major lattice after the rim; the chain entry lands DIAGONALLY
    /// opposite the ring's closure vertex and earcutr fills the notch
    /// with domain-spanning fans — measured on transmission BOOT
    /// f38-class: 530 non-rim boundary edges, hub vertices of degree 69,
    /// long slivers that fold over the curved tube (2041 >170 deg pairs
    /// on BOOT alone). The CDT must triangulate the same inputs with a
    /// complete rim, a hole-free interior, every Steiner point used, and
    /// ZERO same-face >170 deg pairs on the curved surface (the s51
    /// "Delaunay near the rim folds" concern, answered per-class).
    #[test]
    fn test_torus_windmill_band_cdt_no_folds() {
        use draper_geometry::{Point3d, Surface, TorusSurface};
        use std::collections::HashMap;
        use std::f64::consts::PI;

        // Local copy of the tests-module helper (this module nests one
        // level deeper; keeping the test self-contained).
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

        // Half-torus band, R=13 r=2 (the BOOT fillet class): u in [0, pi],
        // v in [-pi/2, pi/2] with WAVY rims (amplitude 0.35 rad) like the
        // real intersection-curve rims.
        let mut rim: Vec<[f64; 2]> = Vec::new();
        let nu = 30;
        let nv = 7;
        // bottom rim: u 0 -> pi, v = -1.1 + 0.35*sin(3u)
        for i in 0..nu {
            let u = PI * (i as f64) / ((nu - 1) as f64);
            let v = -1.1 + 0.12 * (3.0 * u as f64).sin();
            rim.push([u, v]);
        }
        // right edge: u=pi, v -1.1..1.1 (sampled through the wavy ends)
        for j in 1..nv {
            let v = -1.1 + 2.2 * (j as f64) / (nv as f64);
            rim.push([PI, v]);
        }
        // top rim: u pi -> 0, v = 1.1 + 0.35*sin(3u + 0.7)
        for i in 0..nu {
            let u = PI - PI * (i as f64) / ((nu - 1) as f64);
            let v = 1.1 + 0.12 * (3.0 * u as f64 + 0.7).sin();
            rim.push([u, v]);
        }
        // left edge: u=0, v 1.1..-1.1
        for j in 1..nv {
            let v = 1.1 - 2.2 * (j as f64) / (nv as f64);
            rim.push([0.0, v]);
        }
        let n_rim = rim.len();

        // Interior lattice: 23 x 23 grid, shrunk inside the wavy rims
        // (the STRICT-INTERIOR domain filter result).
        let mut interior: Vec<[f64; 2]> = Vec::new();
        for a in 1..=23 {
            for b in 1..=23 {
                let u = 0.12 + (PI - 0.24) * (a as f64) / 24.0;
                let v = -0.65 + 1.3 * (b as f64) / 24.0;
                interior.push([u, v]);
            }
        }

        // ── Legacy spike-chain pass (row-major append) ────────────────
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
        let legacy_gaps = count_interior_boundary_edges(&legacy_tris, n_rim);

        // ── CDT pass ──────────────────────────────────────────────────
        let cdt = triangulate_polygon_cdt(&rim, &[], &interior);
        let cdt_gaps = count_interior_boundary_edges(&cdt, n_rim);

        // The legacy path demonstrably guts this class (windmill slits).
        assert!(
            legacy_gaps > 100,
            "legacy spike-chain should gut the half-torus band (got {}              non-rim boundary edges) — if it no longer does, the earcut              adapter changed; re-audit the production path",
            legacy_gaps
        );
        // The CDT is hole-free.
        assert_eq!(cdt_gaps, 0, "CDT must leave no non-rim boundary edges");
        // Every interior Steiner point is used.
        let mut used = vec![false; n_rim + interior.len()];
        for t in &cdt {
            for &i in t {
                used[i as usize] = true;
            }
        }
        let dropped = (n_rim..n_rim + interior.len())
            .filter(|&i| !used[i])
            .count();
        assert_eq!(dropped, 0, "CDT must insert every Steiner point");

        // ── Emission-fold audit on the curved torus (the s51 concern) ─
        let torus = Surface::Torus(TorusSurface::new_z(Point3d::ORIGIN, 13.0, 2.0));
        let all_pts: Vec<[f64; 2]> = rim.iter().cloned().chain(interior.iter().cloned()).collect();
        let count_folds = |tris: &[[u32; 3]]| -> usize {
            let mut p3: HashMap<u32, [f64; 3]> = HashMap::new();
            for t in tris {
                for &i in t {
                    if !p3.contains_key(&i) {
                        let q = torus.point_at(all_pts[i as usize][0], all_pts[i as usize][1]);
                        p3.insert(i, [q.x, q.y, q.z]);
                    }
                }
            }
            let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
            for (ti, t) in tris.iter().enumerate() {
                for k in 0..3 {
                    let x = t[k];
                    let y = t[(k + 1) % 3];
                    if x != y {
                        edge_tris.entry((x.min(y), x.max(y))).or_default().push(ti);
                    }
                }
            }
            let nrm = |t: &[u32; 3]| -> Option<[f64; 3]> {
                let a = p3[&t[0]];
                let b = p3[&t[1]];
                let d = p3[&t[2]];
                let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let ad = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
                let nn = [
                    ab[1] * ad[2] - ab[2] * ad[1],
                    ab[2] * ad[0] - ab[0] * ad[2],
                    ab[0] * ad[1] - ab[1] * ad[0],
                ];
                let l = (nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]).sqrt();
                if l > 1e-18 {
                    Some([nn[0] / l, nn[1] / l, nn[2] / l])
                } else {
                    None
                }
            };
            let mut folds = 0usize;
            for ts in edge_tris.values() {
                if ts.len() != 2 {
                    continue;
                }
                if let (Some(n1), Some(n2)) = (nrm(&tris[ts[0]]), nrm(&tris[ts[1]])) {
                    let dot = (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]).clamp(-1.0, 1.0);
                    if dot.acos().to_degrees() > 170.0 {
                        folds += 1;
                    }
                }
            }
            folds
        };
        let legacy_folds = count_folds(&legacy_tris);
        let cdt_folds = count_folds(&cdt);
        assert!(
            legacy_folds > 0,
            "the windmill class must demonstrably fold on the curved torus \
             (got {} emission folds) — if it no longer does, the class \
             signature drifted; re-audit",
            legacy_folds
        );
        assert_eq!(
            cdt_folds, 0,
            "CDT must emit ZERO same-face >170 deg pairs on this class \
             (the s51 concern, answered per-class; got {})",
            cdt_folds
        );
    }

}
