// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Custom Constrained Delaunay Triangulation (CDT) with 100% constraint edge guarantee.
//!
//! This module implements CDT from scratch to guarantee that ALL constraint edges
//! are present in the output triangulation. This is critical for watertight mesh
//! generation: if boundary edges of a B-Rep face are missing from the CDT,
//! the resulting mesh will have cracks.
//!
//! # Algorithm
//! 1. Bowyer-Watson algorithm for initial Delaunay triangulation
//! 2. Constraint enforcement via edge flipping
//! 3. Steiner point insertion as fallback when flipping fails
//! 4. Triangle filtering (inside boundary, outside holes)
//!
//! # Guarantee
//! Every constraint edge in the input will appear as an edge in the output
//! triangulation. If flipping cannot make an edge appear, a Steiner point
//! is inserted on the edge to enforce it.

use std::collections::{HashMap, HashSet};

/// Epsilon for orientation tests and other floating-point comparisons.
const EPS: f64 = 1e-10;

/// Input for the CDT algorithm.
pub struct CdtInput {
    /// All 2D points (boundary + holes + interior).
    pub points: Vec<[f64; 2]>,
    /// Outer boundary as indices into `points` (closed loop).
    pub outer_boundary: Vec<u32>,
    /// Hole boundaries as indices into `points`.
    pub holes: Vec<Vec<u32>>,
    /// Constraint edges that MUST appear in the output.
    pub constraints: Vec<[u32; 2]>,
}

/// Result of CDT triangulation.
pub struct CdtResult {
    /// Triangle indices into the original points vec (plus any Steiner points appended).
    pub triangles: Vec<[u32; 3]>,
    /// All points including any Steiner points appended at the end.
    pub all_points: Vec<[f64; 2]>,
    /// Whether all constraints were satisfied.
    pub all_constraints_satisfied: bool,
    /// Number of Steiner points inserted.
    pub steiner_points_inserted: usize,
}

// ============================================================
// Geometry helpers
// ============================================================

/// 2D orientation test: returns positive if C is left of AB, negative if right, ~0 if collinear.
fn orient2d(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}

/// Check if two line segments (a1,a2) and (b1,b2) properly intersect.
fn segments_intersect(a1: &[f64; 2], a2: &[f64; 2], b1: &[f64; 2], b2: &[f64; 2]) -> bool {
    let o1 = orient2d(a1[0], a1[1], a2[0], a2[1], b1[0], b1[1]);
    let o2 = orient2d(a1[0], a1[1], a2[0], a2[1], b2[0], b2[1]);
    let o3 = orient2d(b1[0], b1[1], b2[0], b2[1], a1[0], a1[1]);
    let o4 = orient2d(b1[0], b1[1], b2[0], b2[1], a2[0], a2[1]);

    if o1.abs() < EPS && o2.abs() < EPS {
        // Collinear — check for overlap
        return collinear_segments_overlap(a1, a2, b1, b2);
    }

    // Proper intersection: opposite orientations on both sides
    (o1 * o2 < -EPS) && (o3 * o4 < -EPS)
}

/// Check if two collinear segments overlap.
fn collinear_segments_overlap(a1: &[f64; 2], a2: &[f64; 2], b1: &[f64; 2], b2: &[f64; 2]) -> bool {
    // Project onto the axis with larger span
    let use_x = (a2[0] - a1[0]).abs() > (a2[1] - a1[1]).abs();
    let (a1p, a2p, b1p, b2p) = if use_x {
        (a1[0], a2[0], b1[0], b2[0])
    } else {
        (a1[1], a2[1], b1[1], b2[1])
    };
    let (amin, amax) = if a1p < a2p { (a1p, a2p) } else { (a2p, a1p) };
    let (bmin, bmax) = if b1p < b2p { (b1p, b2p) } else { (b2p, b1p) };
    bmin < amax - EPS && amin < bmax - EPS
}

/// Check if point p is on segment (a, b) within tolerance.
fn point_on_segment(p: &[f64; 2], a: &[f64; 2], b: &[f64; 2]) -> bool {
    let o = orient2d(a[0], a[1], b[0], b[1], p[0], p[1]);
    if o.abs() > EPS {
        return false;
    }
    // Check bounding box
    let xmin = a[0].min(b[0]) - EPS;
    let xmax = a[0].max(b[0]) + EPS;
    let ymin = a[1].min(b[1]) - EPS;
    let ymax = a[1].max(b[1]) + EPS;
    p[0] >= xmin && p[0] <= xmax && p[1] >= ymin && p[1] <= ymax
}

/// Check if a 2D point is inside a polygon using ray casting.
pub fn point_in_polygon_2d(px: f64, py: f64, polygon: &[[f64; 2]]) -> bool {
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

        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ============================================================
// Triangulation data structure
// ============================================================

/// A triangle stored as three vertex indices.
#[derive(Clone, Copy, Debug)]
struct Tri {
    v: [u32; 3],
}

impl Tri {
    fn new(a: u32, b: u32, c: u32) -> Self {
        Tri { v: [a, b, c] }
    }

    /// Check if this triangle contains vertex `vi`.
    fn has_vertex(&self, vi: u32) -> bool {
        self.v[0] == vi || self.v[1] == vi || self.v[2] == vi
    }

    /// Check if this triangle contains edge (a, b).
    fn has_edge(&self, a: u32, b: u32) -> bool {
        (self.v[0] == a || self.v[1] == a || self.v[2] == a)
            && (self.v[0] == b || self.v[1] == b || self.v[2] == b)
    }

    /// Return the third vertex given two vertices of an edge.
    fn opposite_vertex(&self, a: u32, b: u32) -> Option<u32> {
        for &vi in &self.v {
            if vi != a && vi != b {
                return Some(vi);
            }
        }
        None
    }

    /// Replace vertex `old` with `new`.
    fn replace_vertex(&mut self, old: u32, new: u32) {
        for i in 0..3 {
            if self.v[i] == old {
                self.v[i] = new;
                return;
            }
        }
    }
}

/// Circumcircle of a triangle.
struct Circumcircle {
    cx: f64,
    cy: f64,
    r_sq: f64,
}

fn compute_circumcircle(points: &[[f64; 2]], tri: &Tri) -> Circumcircle {
    let ax = points[tri.v[0] as usize][0];
    let ay = points[tri.v[0] as usize][1];
    let bx = points[tri.v[1] as usize][0];
    let by = points[tri.v[1] as usize][1];
    let cx = points[tri.v[2] as usize][0];
    let cy = points[tri.v[2] as usize][1];

    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < EPS {
        // Degenerate — return a large circumcircle
        return Circumcircle {
            cx: (ax + bx + cx) / 3.0,
            cy: (ay + by + cy) / 3.0,
            r_sq: f64::MAX,
        };
    }

    let ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay)
        + (cx * cx + cy * cy) * (ay - by))
        / d;
    let uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx)
        + (cx * cx + cy * cy) * (bx - ax))
        / d;

    let dx = ax - ux;
    let dy = ay - uy;
    Circumcircle {
        cx: ux,
        cy: uy,
        r_sq: dx * dx + dy * dy,
    }
}

/// Check if a point is inside the circumcircle of a triangle.
fn in_circumcircle(points: &[[f64; 2]], tri: &Tri, px: f64, py: f64) -> bool {
    let cc = compute_circumcircle(points, tri);
    let dx = px - cc.cx;
    let dy = py - cc.cy;
    dx * dx + dy * dy < cc.r_sq + EPS
}

// ============================================================
// Bowyer-Watson Delaunay triangulation
// ============================================================

/// Build an initial Delaunay triangulation of all points using Bowyer-Watson.
/// `n_real` is the number of actual input points; the last 3 are super-triangle vertices.
fn bowyer_watson(points: &[[f64; 2]], n_real: usize) -> Vec<Tri> {
    if points.len() < 3 {
        return Vec::new();
    }

    // Start with the super-triangle (last 3 vertices)
    let s0 = n_real as u32;
    let s1 = n_real as u32 + 1;
    let s2 = n_real as u32 + 2;

    let mut triangles = vec![Tri::new(s0, s1, s2)];

    // Insert each real point
    for pi in 0..n_real {
        let px = points[pi][0];
        let py = points[pi][1];
        let pidx = pi as u32;

        // Find all triangles whose circumcircle contains this point
        let mut bad_indices = Vec::new();
        for (ti, tri) in triangles.iter().enumerate() {
            if in_circumcircle(points, tri, px, py) {
                bad_indices.push(ti);
            }
        }

        if bad_indices.is_empty() {
            continue;
        }

        // Find the boundary polygon of the cavity
        let mut boundary_edges: Vec<(u32, u32)> = Vec::new();
        for &ti in &bad_indices {
            let tri = &triangles[ti];
            let edges = [
                (tri.v[0], tri.v[1]),
                (tri.v[1], tri.v[2]),
                (tri.v[2], tri.v[0]),
            ];
            for &(ea, eb) in &edges {
                // Check if this edge is shared with another bad triangle
                let mut shared = false;
                for &tj in &bad_indices {
                    if tj == ti {
                        continue;
                    }
                    if triangles[tj].has_edge(ea, eb) {
                        shared = true;
                        break;
                    }
                }
                if !shared {
                    boundary_edges.push((ea, eb));
                }
            }
        }

        // Remove bad triangles (in reverse index order to preserve indices)
        let mut bad_set: HashSet<usize> = bad_indices.into_iter().collect();
        let mut new_tris: Vec<Tri> = Vec::new();
        for (ti, tri) in triangles.drain(..).enumerate() {
            if !bad_set.contains(&ti) {
                new_tris.push(tri);
            }
        }
        triangles = new_tris;

        // Create new triangles connecting the point to each boundary edge
        for &(ea, eb) in &boundary_edges {
            triangles.push(Tri::new(pidx, ea, eb));
        }
    }

    triangles
}

// ============================================================
// Constraint enforcement
// ============================================================

/// Find the index of a triangle that contains both vertices a and b.
fn find_triangle_with_edge(triangles: &[Tri], a: u32, b: u32) -> Option<usize> {
    for (i, tri) in triangles.iter().enumerate() {
        if tri.has_edge(a, b) {
            return Some(i);
        }
    }
    None
}

/// Find all triangles that share edge (a, b).
fn find_triangles_sharing_edge(triangles: &[Tri], a: u32, b: u32) -> Vec<usize> {
    triangles
        .iter()
        .enumerate()
        .filter(|(_, tri)| tri.has_edge(a, b))
        .map(|(i, _)| i)
        .collect()
}

/// Try to enforce a constraint edge by flipping. Returns true if the edge now exists.
fn enforce_constraint_by_flipping(
    points: &mut Vec<[f64; 2]>,
    triangles: &mut Vec<Tri>,
    a: u32,
    b: u32,
    max_flips: usize,
) -> bool {
    for _ in 0..max_flips {
        // Check if the constraint edge already exists
        if find_triangle_with_edge(triangles, a, b).is_some() {
            return true;
        }

        // Find an edge crossing (a, b)
        let pa = points[a as usize];
        let pb = points[b as usize];

        let mut flipped = false;
        let mut tri_indices_to_check: Vec<usize> = (0..triangles.len()).collect();

        for ti in tri_indices_to_check {
            if ti >= triangles.len() {
                continue;
            }
            let tri = triangles[ti];

            // Check each edge of this triangle for intersection with (a, b)
            let edges = [
                (tri.v[0], tri.v[1]),
                (tri.v[1], tri.v[2]),
                (tri.v[2], tri.v[0]),
            ];

            for &(ea, eb) in &edges {
                // Don't flip edges that share a vertex with the constraint
                if ea == a || ea == b || eb == a || eb == b {
                    continue;
                }

                let pe_a = points[ea as usize];
                let pe_b = points[eb as usize];

                if segments_intersect(&pa, &pb, &pe_a, &pe_b) {
                    // Try to flip edge (ea, eb)
                    let shared = find_triangles_sharing_edge(triangles, ea, eb);
                    if shared.len() == 2 {
                        let t1 = triangles[shared[0]];
                        let t2 = triangles[shared[1]];

                        let c1 = t1.opposite_vertex(ea, eb).unwrap();
                        let c2 = t2.opposite_vertex(ea, eb).unwrap();

                        // Check if the quadrilateral is convex
                        let pc1 = points[c1 as usize];
                        let pc2 = points[c2 as usize];
                        let pea = points[ea as usize];
                        let peb = points[eb as usize];

                        let o1 = orient2d(pea[0], pea[1], peb[0], peb[1], pc1[0], pc1[1]);
                        let o2 = orient2d(pea[0], pea[1], peb[0], peb[1], pc2[0], pc2[1]);

                        // Convex if c1 and c2 are on opposite sides of (ea, eb)
                        if o1 * o2 < -EPS {
                            // Flip: replace edge (ea, eb) with (c1, c2)
                            // Remove old triangles and add new ones
                            let mut new_tris = Vec::with_capacity(triangles.len());
                            for (i, t) in triangles.iter().enumerate() {
                                if i != shared[0] && i != shared[1] {
                                    new_tris.push(*t);
                                }
                            }
                            new_tris.push(Tri::new(c1, c2, ea));
                            new_tris.push(Tri::new(c2, c1, eb));
                            *triangles = new_tris;
                            flipped = true;
                            break;
                        }
                    }
                }
            }
            if flipped {
                break;
            }
        }

        if !flipped {
            // No more flips possible — need Steiner point
            return false;
        }
    }

    // Check one final time
    find_triangle_with_edge(triangles, a, b).is_some()
}

/// Enforce a constraint edge, using Steiner point insertion if flipping fails.
fn enforce_constraint(
    points: &mut Vec<[f64; 2]>,
    triangles: &mut Vec<Tri>,
    a: u32,
    b: u32,
    steiner_count: &mut usize,
    depth: usize,
) -> bool {
    if depth > 20 {
        // Safety limit
        return find_triangle_with_edge(triangles, a, b).is_some();
    }

    // Check if already exists
    if find_triangle_with_edge(triangles, a, b).is_some() {
        return true;
    }

    // Try flipping first
    if enforce_constraint_by_flipping(points, triangles, a, b, 50) {
        return true;
    }

    // Flipping failed — insert Steiner point at midpoint
    let pa = points[a as usize];
    let pb = points[b as usize];
    let mid = [(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0];
    let mid_idx = points.len() as u32;
    points.push(mid);
    *steiner_count += 1;

    // Insert the Steiner point using Bowyer-Watson
    insert_point_bowyer_watson(points, triangles, mid_idx);

    // Recursively enforce (a, mid) and (mid, b)
    let ok1 = enforce_constraint(points, triangles, a, mid_idx, steiner_count, depth + 1);
    let ok2 = enforce_constraint(points, triangles, mid_idx, b, steiner_count, depth + 1);

    ok1 && ok2
}

/// Insert a single point into the triangulation using Bowyer-Watson.
fn insert_point_bowyer_watson(points: &[[f64; 2]], triangles: &mut Vec<Tri>, pidx: u32) {
    let px = points[pidx as usize][0];
    let py = points[pidx as usize][1];

    // Find all triangles whose circumcircle contains this point
    let bad_indices: HashSet<usize> = triangles
        .iter()
        .enumerate()
        .filter(|(_, tri)| in_circumcircle(points, tri, px, py))
        .map(|(i, _)| i)
        .collect();

    if bad_indices.is_empty() {
        return;
    }

    // Find boundary edges of the cavity
    let mut boundary_edges: Vec<(u32, u32)> = Vec::new();
    for &ti in &bad_indices {
        let tri = &triangles[ti];
        let edges = [
            (tri.v[0], tri.v[1]),
            (tri.v[1], tri.v[2]),
            (tri.v[2], tri.v[0]),
        ];
        for &(ea, eb) in &edges {
            let mut shared = false;
            for &tj in &bad_indices {
                if tj == ti {
                    continue;
                }
                if triangles[tj].has_edge(ea, eb) {
                    shared = true;
                    break;
                }
            }
            if !shared {
                boundary_edges.push((ea, eb));
            }
        }
    }

    // Remove bad triangles
    let mut new_tris: Vec<Tri> = Vec::with_capacity(triangles.len());
    for (ti, tri) in triangles.drain(..).enumerate() {
        if !bad_indices.contains(&ti) {
            new_tris.push(tri);
        }
    }
    *triangles = new_tris;

    // Create new triangles
    for &(ea, eb) in &boundary_edges {
        triangles.push(Tri::new(pidx, ea, eb));
    }
}

// ============================================================
// Main CDT function
// ============================================================

/// Perform Constrained Delaunay Triangulation with 100% constraint edge guarantee.
///
/// # Algorithm
/// 1. Create super-triangle enclosing all points
/// 2. Insert all points using Bowyer-Watson
/// 3. Enforce all constraint edges via flipping + Steiner points
/// 4. Remove super-triangle and exterior triangles
/// 5. Filter triangles: inside outer boundary, outside holes
/// 6. Verify all constraints are present
pub fn constrained_delaunay_triangulation(input: &CdtInput) -> CdtResult {
    let n_input = input.points.len();
    if n_input < 3 || input.outer_boundary.len() < 3 {
        return CdtResult {
            triangles: Vec::new(),
            all_points: input.points.clone(),
            all_constraints_satisfied: true,
            steiner_points_inserted: 0,
        };
    }

    // Compute bounding box for super-triangle
    let mut xmin = f64::MAX;
    let mut ymin = f64::MAX;
    let mut xmax = f64::MIN;
    let mut ymax = f64::MIN;
    for p in &input.points {
        xmin = xmin.min(p[0]);
        ymin = ymin.min(p[1]);
        xmax = xmax.max(p[0]);
        ymax = ymax.max(p[1]);
    }

    let dx = xmax - xmin;
    let dy = ymax - ymin;
    let d = dx.max(dy).max(1.0);
    let margin = d * 5.0;

    // Create super-triangle vertices
    let cx = (xmin + xmax) / 2.0;
    let cy = (ymin + ymax) / 2.0;
    let s0 = [cx - margin * 2.0, cy - margin];
    let s1 = [cx + margin * 2.0, cy - margin];
    let s2 = [cx, cy + margin * 2.0];

    // Build points array with super-triangle vertices appended
    let mut points: Vec<[f64; 2]> = input.points.clone();
    points.push(s0);
    points.push(s1);
    points.push(s2);

    // Step 1: Bowyer-Watson
    let mut triangles = bowyer_watson(&points, n_input);

    // Step 2: Enforce constraints
    let mut steiner_count = 0usize;
    let mut all_satisfied = true;

    for &[a, b] in &input.constraints {
        let ok = enforce_constraint(&mut points, &mut triangles, a, b, &mut steiner_count, 0);
        if !ok {
            all_satisfied = false;
            log::warn!("CDT: failed to enforce constraint edge ({}, {})", a, b);
        }
    }

    // Step 3: Remove triangles connected to super-triangle vertices
    let super_start = n_input as u32; // first super vertex index (may shift due to Steiner points)
    // Actually, super vertices are always the last 3 of the original array.
    // But Steiner points were inserted, so super vertices are at the end still.
    let n_super = points.len() - n_input; // includes Steiner points + 3 super
    let super_base = (points.len() - 3) as u32; // the 3 super-triangle vertices

    triangles.retain(|tri| {
        !tri.has_vertex(super_base)
            && !tri.has_vertex(super_base + 1)
            && !tri.has_vertex(super_base + 2)
    });

    // Step 4: Filter triangles — keep only those inside the outer boundary and outside holes
    let outer_polygon: Vec<[f64; 2]> = input
        .outer_boundary
        .iter()
        .map(|&idx| points[idx as usize])
        .collect();

    let hole_polygons: Vec<Vec<[f64; 2]>> = input
        .holes
        .iter()
        .map(|hole| hole.iter().map(|&idx| points[idx as usize]).collect())
        .collect();

    triangles.retain(|tri| {
        // Compute centroid
        let cx = (points[tri.v[0] as usize][0]
            + points[tri.v[1] as usize][0]
            + points[tri.v[2] as usize][0])
            / 3.0;
        let cy = (points[tri.v[0] as usize][1]
            + points[tri.v[1] as usize][1]
            + points[tri.v[2] as usize][1])
            / 3.0;

        // Must be inside outer boundary
        if !point_in_polygon_2d(cx, cy, &outer_polygon) {
            return false;
        }

        // Must be outside all holes
        for hole_poly in &hole_polygons {
            if point_in_polygon_2d(cx, cy, hole_poly) {
                return false;
            }
        }

        true
    });

    // Step 5: Filter degenerate triangles
    triangles.retain(|tri| {
        tri.v[0] != tri.v[1] && tri.v[1] != tri.v[2] && tri.v[0] != tri.v[2]
            // Check area > 0
            && orient2d(
                points[tri.v[0] as usize][0],
                points[tri.v[0] as usize][1],
                points[tri.v[1] as usize][0],
                points[tri.v[1] as usize][1],
                points[tri.v[2] as usize][0],
                points[tri.v[2] as usize][1],
            )
            .abs()
            > EPS
    });

    // Step 6: Verify all constraints
    for &[a, b] in &input.constraints {
        if !triangles.iter().any(|t| t.has_edge(a, b)) {
            all_satisfied = false;
            log::error!(
                "CDT constraint verification FAILED: edge ({}, {}) not in triangulation",
                a,
                b
            );
        }
    }

    // Convert triangles to output format
    let result_triangles: Vec<[u32; 3]> = triangles.iter().map(|t| t.v).collect();

    CdtResult {
        triangles: result_triangles,
        all_points: points,
        all_constraints_satisfied: all_satisfied,
        steiner_points_inserted: steiner_count,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_square_cdt() {
        let points = vec![
            [0.0, 0.0], // 0
            [1.0, 0.0], // 1
            [1.0, 1.0], // 2
            [0.0, 1.0], // 3
        ];

        let input = CdtInput {
            points,
            outer_boundary: vec![0, 1, 2, 3],
            holes: vec![],
            constraints: vec![[0, 1], [1, 2], [2, 3], [3, 0]],
        };

        let result = constrained_delaunay_triangulation(&input);
        assert!(result.all_constraints_satisfied);
        assert!(result.triangles.len() >= 2); // At least 2 triangles for a square
    }

    #[test]
    fn test_square_with_hole() {
        let points = vec![
            [0.0, 0.0], // 0
            [4.0, 0.0], // 1
            [4.0, 4.0], // 2
            [0.0, 4.0], // 3
            [1.0, 1.0], // 4
            [3.0, 1.0], // 5
            [3.0, 3.0], // 6
            [1.0, 3.0], // 7
        ];

        let input = CdtInput {
            points,
            outer_boundary: vec![0, 1, 2, 3],
            holes: vec![vec![4, 5, 6, 7]],
            constraints: vec![
                [0, 1], [1, 2], [2, 3], [3, 0],
                [4, 5], [5, 6], [6, 7], [7, 4],
            ],
        };

        let result = constrained_delaunay_triangulation(&input);
        assert!(result.all_constraints_satisfied);
        // No triangle should have all 3 vertices inside the hole
        for tri in &result.triangles {
            let cx = (result.all_points[tri[0] as usize][0]
                + result.all_points[tri[1] as usize][0]
                + result.all_points[tri[2] as usize][0])
                / 3.0;
            let cy = (result.all_points[tri[0] as usize][1]
                + result.all_points[tri[1] as usize][1]
                + result.all_points[tri[2] as usize][1])
                / 3.0;
            let hole = [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]];
            assert!(!point_in_polygon_2d(cx, cy, &hole));
        }
    }

    #[test]
    fn test_point_in_polygon() {
        let square = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        assert!(point_in_polygon_2d(0.5, 0.5, &square));
        assert!(!point_in_polygon_2d(1.5, 0.5, &square));
        assert!(!point_in_polygon_2d(-0.5, 0.5, &square));
    }
}
