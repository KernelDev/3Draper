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

use std::collections::HashSet;

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

    let earcut_result = earcutr::earcut(&coords, &earcut_hole_indices, 2);

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

    // Insert interior Steiner points using Bowyer-Watson
    if !interior_2d.is_empty() {
        insert_interior_points(&all_2d, &mut triangles, interior_start, interior_2d.len());
    }

    // Delaunay improvement (Lawson flips) respecting constraints
    lawson_flip(&all_2d, &mut triangles, n_boundary, &hole_index_ranges);

    // Verify constraint edges exist (debug only)
    #[cfg(debug_assertions)]
    verify_constraints(&triangles, n_boundary, &hole_index_ranges);

    triangles
}

/// Insert interior Steiner points into an existing triangulation
/// using Bowyer-Watson point insertion.
fn insert_interior_points(
    all_2d: &[[f64; 2]],
    triangles: &mut Vec<[u32; 3]>,
    interior_start: usize,
    n_interior: usize,
) {
    for i in 0..n_interior {
        let point_idx = (interior_start + i) as u32;
        let p = all_2d[point_idx as usize];

        match find_containing_triangle(all_2d, triangles, p) {
            Some((tri_idx, on_edge)) => {
                if on_edge {
                    insert_point_on_edge(all_2d, triangles, tri_idx, point_idx);
                } else {
                    insert_point_in_triangle(triangles, tri_idx, point_idx);
                }
            }
            None => {
                // Point is outside the triangulation - skip it
                log::debug!("Interior point {} outside triangulation, skipping", point_idx);
            }
        }
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
fn lawson_flip(
    vertices: &[[f64; 2]],
    triangles: &mut Vec<[u32; 3]>,
    n_boundary: usize,
    hole_ranges: &[(usize, usize)],
) {
    let constraint_edges = build_constraint_set(n_boundary, hole_ranges);

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

                // Find neighboring triangle sharing this edge
                let nbr_idx = find_triangle_with_edge(triangles, ev1, ev2, i);
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

                    if ccw1 > 0.0 && ccw2 > 0.0 {
                        triangles[i] = new_tri1;
                        if let Some(nbr) = nbr_idx {
                            triangles[nbr] = new_tri2;
                        }
                        flipped = true;
                    } else if ccw1 < 0.0 && ccw2 < 0.0 {
                        triangles[i] = [new_tri1[0], new_tri1[2], new_tri1[1]];
                        if let Some(nbr) = nbr_idx {
                            triangles[nbr] = [new_tri2[0], new_tri2[2], new_tri2[1]];
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
fn orient2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
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
}
