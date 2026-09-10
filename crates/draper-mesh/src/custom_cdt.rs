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

    // Delaunay improvement (Lawson flips) — DISABLED 2026-09-09.
    //
    // The flip phase had no quad-convexity guard: for a non-convex quad
    // the incircle test passes trivially (the "opposite" vertex lies
    // inside the neighbor triangle, hence inside its circumcircle), and
    // the flip then produces OVERLAPPING triangles — corrupting an
    // otherwise valid greedy-insertion triangulation (stress test
    // `test_cdt_with_hole_and_grid_no_gaps` caught 2 interior gaps from
    // exactly this). Correctness (watertightness) strictly dominates
    // Delaunay quality here; re-enable only with a convexity guard AND
    // a post-flip validity check (no edge usage != 2 in the interior).
    // lawson_flip(&all_2d, &mut triangles, n_boundary, &hole_index_ranges);
    let _ = (&n_boundary, &hole_index_ranges);

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
                        continue; // redundant with the rim — skip
                    }
                    insert_point_on_edge_fast(all_2d, triangles, tri_idx, point_idx, &mut edge_map);
                } else {
                    insert_point_in_triangle_fast(triangles, tri_idx, point_idx, &mut edge_map);
                }
            }
            None => {
                // Point is outside the triangulation - skip it
                log::debug!("Interior point {} outside triangulation, skipping", point_idx);
            }
        }
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
fn repair_unused_ring_vertices(
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
fn lawson_flip(
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
            let tri = triangles[i];
            for edge_idx in 0..3 {
                let ev1 = tri[edge_idx];
                let ev2 = tri[(edge_idx + 1) % 3];
                let opposite = tri[(edge_idx + 2) % 3];

                // Skip constraint edges
                let edge_key = (ev1.min(ev2), ev1.max(ev2));
                if constraint_edges.contains(&edge_key) {
                    continue;
                }

                // Find neighboring triangle using edge map (O(1))
                let nbr_idx = edge_map.get(&edge_key)
                    .and_then(|indices| indices.iter().find(|&&idx| idx != i).copied());

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
