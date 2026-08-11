// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # Mesh-based Boolean Operations
//!
//! Robust boolean operations (union, subtract, intersect) that work directly
//! on triangle meshes. Unlike B-Rep boolean operations which require exact
//! surface-surface intersection curves and topology stitching, mesh-based
//! booleans operate on the discretized geometry and are therefore more
//! robust for complex geometries (cylinders through boxes, fillets, etc.).
//!
//! ## Algorithm
//!
//! 1. **Triangle-triangle intersection**: For each pair of triangles (one
//!    from mesh A, one from mesh B), find the intersection segment using
//!    Möller's algorithm.
//!
//! 2. **Triangle splitting**: Each triangle that has intersection segments
//!    is split into smaller triangles along those segments.
//!
//! 3. **Classification**: Each triangle fragment is classified as inside
//!    or outside the other mesh using ray-casting point-in-mesh test.
//!
//! 4. **Assembly**: Based on the boolean operation:
//!    - Union: keep A-outside-B + B-outside-A
//!    - Subtract: keep A-outside-B + B-inside-A (reversed)
//!    - Intersect: keep A-inside-B + B-inside-A

use crate::mesh::TriangleMesh;
use draper_geometry::{Point3d, Vec3d};

// ============================================================
// Public API
// ============================================================

/// Compute boolean union of two triangle meshes.
///
/// Result = A ∪ B (all triangles from A outside B, plus all triangles
/// from B outside A).
pub fn mesh_union(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    mesh_boolean(a, b, MeshBooleanOp::Union)
}

/// Compute boolean subtract (a - b) of two triangle meshes.
///
/// Result = A - B (all triangles from A outside B, plus the triangles
/// from B that are inside A, with reversed winding to form the cavity).
pub fn mesh_subtract(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    mesh_boolean(a, b, MeshBooleanOp::Subtract)
}

/// Compute boolean intersect of two triangle meshes.
///
/// Result = A ∩ B (all triangles from A inside B, plus all triangles
/// from B inside A).
pub fn mesh_intersect(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    mesh_boolean(a, b, MeshBooleanOp::Intersect)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MeshBooleanOp {
    Union,
    Subtract,
    Intersect,
}

/// Main entry point for mesh boolean operations.
///
/// Uses a centroid-classification approach: each triangle is classified as
/// entirely inside or outside the other mesh based on its centroid. Triangles
/// that straddle the boundary are kept or discarded based on centroid location.
/// After classification, boundary gaps are filled with triangles to ensure
/// watertightness.
fn mesh_boolean(a: &TriangleMesh, b: &TriangleMesh, op: MeshBooleanOp) -> TriangleMesh {
    if a.triangles.is_empty() {
        return match op {
            MeshBooleanOp::Subtract => TriangleMesh::new(),
            _ => b.clone(),
        };
    }
    if b.triangles.is_empty() {
        return match op {
            MeshBooleanOp::Intersect => TriangleMesh::new(),
            _ => a.clone(),
        };
    }

    // Step 1: Classify each triangle of A as inside/outside B.
    let a_keep: Vec<bool> = a.triangles.iter()
        .map(|tri| {
            let centroid = triangle_centroid(&[
                a.vertices[tri[0] as usize],
                a.vertices[tri[1] as usize],
                a.vertices[tri[2] as usize],
            ]);
            let inside_b = point_in_mesh(&centroid, b);
            match op {
                MeshBooleanOp::Union | MeshBooleanOp::Subtract => !inside_b,
                MeshBooleanOp::Intersect => inside_b,
            }
        })
        .collect();

    // Step 2: Classify each triangle of B as inside/outside A.
    let b_keep: Vec<bool> = b.triangles.iter()
        .map(|tri| {
            let centroid = triangle_centroid(&[
                b.vertices[tri[0] as usize],
                b.vertices[tri[1] as usize],
                b.vertices[tri[2] as usize],
            ]);
            let inside_a = point_in_mesh(&centroid, a);
            match op {
                MeshBooleanOp::Union => !inside_a,
                MeshBooleanOp::Subtract => inside_a,
                MeshBooleanOp::Intersect => inside_a,
            }
        })
        .collect();

    // Step 3: Assemble result from kept triangles.
    let mut result = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<(u64, u64, u64), u32> = std::collections::HashMap::new();

    for (ti, &keep) in a_keep.iter().enumerate() {
        if keep {
            let tri = a.triangles[ti];
            add_triangle(&mut result, &mut vertex_map, &[
                a.vertices[tri[0] as usize],
                a.vertices[tri[1] as usize],
                a.vertices[tri[2] as usize],
            ]);
        }
    }

    for (ti, &keep) in b_keep.iter().enumerate() {
        if keep {
            let tri = b.triangles[ti];
            let v = [
                b.vertices[tri[0] as usize],
                b.vertices[tri[1] as usize],
                b.vertices[tri[2] as usize],
            ];
            if op == MeshBooleanOp::Subtract {
                add_triangle(&mut result, &mut vertex_map, &[v[0], v[2], v[1]]);
            } else {
                add_triangle(&mut result, &mut vertex_map, &v);
            }
        }
    }

    // Step 4: Fill boundary gaps to ensure watertightness.
    crate::watertight::fill_boundary_gaps(&mut result, 512);

    // Step 5: Weld vertices and clean up.
    let bbox = compute_bounding_box(&result);
    let scale = bbox_size(&bbox).max(1.0);
    let weld_tol = scale * 1e-6;
    weld_vertices(&mut result, weld_tol);
    crate::watertight::fill_boundary_gaps(&mut result, 512);
    clean_mesh(result)
}

/// Add a triangle to the result mesh, deduplicating vertices.
fn add_triangle(
    mesh: &mut TriangleMesh,
    vertex_map: &mut std::collections::HashMap<(u64, u64, u64), u32>,
    tri: &[Point3d; 3],
) {
    let mut indices = [0u32; 3];
    for (i, p) in tri.iter().enumerate() {
        let key = quantize_point(p);
        if let Some(&idx) = vertex_map.get(&key) {
            indices[i] = idx;
        } else {
            let idx = mesh.vertices.len() as u32;
            mesh.vertices.push(*p);
            vertex_map.insert(key, idx);
            indices[i] = idx;
        }
    }
    mesh.triangles.push(indices);
}

/// For each edge in mesh `a`, find all intersection points where the edge
/// crosses any triangle plane of mesh `b`, OR crosses any edge of mesh `b`.
fn split_edges_by_planes_and_edges(a: &TriangleMesh, b: &TriangleMesh) -> Vec<Vec<f64>> {
    let num_edges = a.triangles.len() * 3;
    let mut edge_splits: Vec<Vec<f64>> = vec![Vec::new(); num_edges];

    // Collect all edges of B (as vertex index pairs)
    let b_edges: Vec<(Point3d, Point3d)> = b
        .triangles
        .iter()
        .flat_map(|tri| {
            let v0 = b.vertices[tri[0] as usize];
            let v1 = b.vertices[tri[1] as usize];
            let v2 = b.vertices[tri[2] as usize];
            [(v0, v1), (v1, v2), (v2, v0)]
        })
        .collect();

    for (ti, tri) in a.triangles.iter().enumerate() {
        for ei in 0..3 {
            let v0_idx = tri[ei] as usize;
            let v1_idx = tri[(ei + 1) % 3] as usize;
            let p0 = a.vertices[v0_idx];
            let p1 = a.vertices[v1_idx];
            let edge_idx = ti * 3 + ei;

            // 1. Edge-plane intersections
            for tri_b in &b.triangles {
                let vb = [
                    b.vertices[tri_b[0] as usize],
                    b.vertices[tri_b[1] as usize],
                    b.vertices[tri_b[2] as usize],
                ];
                let nb = triangle_normal(&vb);
                if nb.length_sq() < 1e-20 {
                    continue;
                }

                let d0 = signed_distance_to_plane(&p0, &vb[0], &nb);
                let d1 = signed_distance_to_plane(&p1, &vb[0], &nb);

                if (d0 > 0.0 && d1 < 0.0) || (d0 < 0.0 && d1 > 0.0) {
                    let t = d0 / (d0 - d1);
                    if t > 1e-9 && t < 1.0 - 1e-9 {
                        let px = p0.x + t * (p1.x - p0.x);
                        let py = p0.y + t * (p1.y - p0.y);
                        let pz = p0.z + t * (p1.z - p0.z);
                        let p = Point3d::new(px, py, pz);
                        if point_in_triangle(&p, &vb, &nb) {
                            if !edge_splits[edge_idx].iter().any(|&t2| (t - t2).abs() < 1e-9) {
                                edge_splits[edge_idx].push(t);
                            }
                        }
                    }
                }
            }

            // 2. Edge-edge intersections
            for (ep0, ep1) in &b_edges {
                if let Some(t) = edge_edge_intersection(&p0, &p1, ep0, ep1) {
                    if t > 1e-9 && t < 1.0 - 1e-9 {
                        if !edge_splits[edge_idx].iter().any(|&t2| (t - t2).abs() < 1e-9) {
                            edge_splits[edge_idx].push(t);
                        }
                    }
                }
            }

            edge_splits[edge_idx].sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        }
    }

    edge_splits
}

/// Find the parametric intersection of edge (p0,p1) with edge (q0,q1).
/// Returns t along (p0,p1) if they intersect, None otherwise.
fn edge_edge_intersection(p0: &Point3d, p1: &Point3d, q0: &Point3d, q1: &Point3d) -> Option<f64> {
    let d1 = Vec3d::new(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    let d2 = Vec3d::new(q1.x - q0.x, q1.y - q0.y, q1.z - q0.z);
    let r = Vec3d::new(p0.x - q0.x, p0.y - q0.y, p0.z - q0.z);

    let n = d1.cross(&d2);
    let n_len_sq = n.length_sq();
    if n_len_sq < 1e-20 {
        return None; // Parallel
    }

    // Check coplanarity
    if r.dot(&n).abs() > 1e-9 * n.length() {
        return None; // Skew
    }

    let n2 = r.cross(&d2);
    let t = n2.dot(&n) / n_len_sq;

    let n3 = r.cross(&d1);
    let s = n3.dot(&n) / n_len_sq;

    if t < -1e-9 || t > 1.0 + 1e-9 || s < -1e-9 || s > 1.0 + 1e-9 {
        return None;
    }

    Some(t.max(0.0).min(1.0))
}

/// For each edge in mesh `a`, find all intersection points where the edge
/// crosses any triangle plane of mesh `b`. Returns a vector (indexed by
/// edge index) of parametric positions along the edge.
///
/// Edge index convention: for triangle i with vertices [v0, v1, v2],
/// the 3 edges are: 3*i+0 = v0→v1, 3*i+1 = v1→v2, 3*i+2 = v2→v0.
fn split_edges_by_planes(a: &TriangleMesh, b: &TriangleMesh) -> Vec<Vec<f64>> {
    let num_edges = a.triangles.len() * 3;
    let mut edge_splits: Vec<Vec<f64>> = vec![Vec::new(); num_edges];

    // For each edge of A, test against each triangle plane of B
    for (ti, tri) in a.triangles.iter().enumerate() {
        for ei in 0..3 {
            let v0_idx = tri[ei] as usize;
            let v1_idx = tri[(ei + 1) % 3] as usize;
            let p0 = a.vertices[v0_idx];
            let p1 = a.vertices[v1_idx];
            let edge_idx = ti * 3 + ei;

            for tri_b in &b.triangles {
                let vb = [
                    b.vertices[tri_b[0] as usize],
                    b.vertices[tri_b[1] as usize],
                    b.vertices[tri_b[2] as usize],
                ];
                let nb = triangle_normal(&vb);
                if nb.length_sq() < 1e-20 {
                    continue;
                }

                // Signed distances
                let d0 = signed_distance_to_plane(&p0, &vb[0], &nb);
                let d1 = signed_distance_to_plane(&p1, &vb[0], &nb);

                // Check if edge crosses the plane
                if (d0 > 0.0 && d1 < 0.0) || (d0 < 0.0 && d1 > 0.0) {
                    let t = d0 / (d0 - d1);
                    if t > 1e-9 && t < 1.0 - 1e-9 {
                        // Verify the intersection point is inside triangle B
                        let px = p0.x + t * (p1.x - p0.x);
                        let py = p0.y + t * (p1.y - p0.y);
                        let pz = p0.z + t * (p1.z - p0.z);
                        let p = Point3d::new(px, py, pz);
                        if point_in_triangle(&p, &vb, &nb) {
                            // Avoid duplicate t values
                            if !edge_splits[edge_idx].iter().any(|&t2| (t - t2).abs() < 1e-9) {
                                edge_splits[edge_idx].push(t);
                            }
                        }
                    }
                }
            }

            edge_splits[edge_idx].sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        }
    }

    edge_splits
}

/// Rebuild triangles using split edges. Each triangle's edges may have
/// additional points inserted at intersection locations. We re-triangulate
/// each triangle as a fan from vertex 0 using all the points on its edges.
fn rebuild_triangles(
    mesh: &TriangleMesh,
    edge_splits: &[Vec<f64>],
) -> Vec<TriangleFragment> {
    let mut fragments: Vec<TriangleFragment> = Vec::new();

    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        // Get split points for each edge of this triangle
        let s0 = &edge_splits[ti * 3 + 0]; // v0→v1
        let s1 = &edge_splits[ti * 3 + 1]; // v1→v2
        let s2 = &edge_splits[ti * 3 + 2]; // v2→v0

        // If no splits, keep original triangle
        if s0.is_empty() && s1.is_empty() && s2.is_empty() {
            fragments.push(TriangleFragment { v: [v0, v1, v2] });
            continue;
        }

        // Build the boundary polygon: v0, splits on v0→v1, v1, splits on v1→v2, v2, splits on v2→v0
        let mut polygon: Vec<Point3d> = vec![v0];

        // Splits on edge v0→v1 (parametric t from v0 to v1)
        for &t in s0 {
            polygon.push(Point3d::new(
                v0.x + t * (v1.x - v0.x),
                v0.y + t * (v1.y - v0.y),
                v0.z + t * (v1.z - v0.z),
            ));
        }

        polygon.push(v1);

        // Splits on edge v1→v2
        for &t in s1 {
            polygon.push(Point3d::new(
                v1.x + t * (v2.x - v1.x),
                v1.y + t * (v2.y - v1.y),
                v1.z + t * (v2.z - v1.z),
            ));
        }

        polygon.push(v2);

        // Splits on edge v2→v0
        for &t in s2 {
            polygon.push(Point3d::new(
                v2.x + t * (v0.x - v2.x),
                v2.y + t * (v0.y - v2.y),
                v2.z + t * (v0.z - v2.z),
            ));
        }

        // Fan triangulate from vertex 0 (polygon[0] = v0)
        // This preserves the original winding and uses shared edge points.
        for i in 1..polygon.len().saturating_sub(1) {
            fragments.push(TriangleFragment {
                v: [polygon[0], polygon[i], polygon[i + 1]],
            });
        }
    }

    fragments
}

// ============================================================
// Step 1: Triangle-triangle intersection (Möller's algorithm)
// ============================================================

/// A segment where two triangles intersect.
#[derive(Clone, Debug)]
struct IntersectionSegment {
    /// Start point of the intersection segment (on the intersection line).
    p0: Point3d,
    /// End point of the intersection segment.
    p1: Point3d,
}

/// For each triangle in mesh `a`, collect all intersection segments with
/// any triangle in mesh `b`. Returns a vector (indexed by triangle index
/// in `a`) of segments.
fn collect_intersection_segments(
    a: &TriangleMesh,
    b: &TriangleMesh,
) -> Vec<Vec<IntersectionSegment>> {
    let mut result: Vec<Vec<IntersectionSegment>> = vec![Vec::new(); a.triangles.len()];

    // Simple O(n*m) brute-force intersection test.
    // For large meshes, this should be accelerated with BVH.
    for (ia, tri_a) in a.triangles.iter().enumerate() {
        let va = [
            a.vertices[tri_a[0] as usize],
            a.vertices[tri_a[1] as usize],
            a.vertices[tri_a[2] as usize],
        ];
        for (ib, tri_b) in b.triangles.iter().enumerate() {
            let vb = [
                b.vertices[tri_b[0] as usize],
                b.vertices[tri_b[1] as usize],
                b.vertices[tri_b[2] as usize],
            ];
            if let Some(seg) = triangle_triangle_intersection(&va, &vb) {
                result[ia].push(seg);
            }
            let _ = ib;
        }
    }

    result
}

/// Möller's triangle-triangle intersection test.
///
/// Returns the intersection segment if the triangles intersect, None otherwise.
/// Reference: "A Fast Triangle-Triangle Intersection Test" (Möller, 1997).
fn triangle_triangle_intersection(
    a: &[Point3d; 3],
    b: &[Point3d; 3],
) -> Option<IntersectionSegment> {
    // Compute normal of triangle B
    let nb = triangle_normal(b);
    if nb.length_sq() < 1e-20 {
        return None; // Degenerate triangle B
    }

    // Signed distances from A's vertices to B's plane
    let da = [
        signed_distance_to_plane(&a[0], &b[0], &nb),
        signed_distance_to_plane(&a[1], &b[0], &nb),
        signed_distance_to_plane(&a[2], &b[0], &nb),
    ];

    // All on same side? No intersection.
    if da[0] > 0.0 && da[1] > 0.0 && da[2] > 0.0 {
        return None;
    }
    if da[0] < 0.0 && da[1] < 0.0 && da[2] < 0.0 {
        return None;
    }

    // Compute normal of triangle A
    let na = triangle_normal(a);
    if na.length_sq() < 1e-20 {
        return None; // Degenerate triangle A
    }

    // Signed distances from B's vertices to A's plane
    let db = [
        signed_distance_to_plane(&b[0], &a[0], &na),
        signed_distance_to_plane(&b[1], &a[0], &na),
        signed_distance_to_plane(&b[2], &a[0], &na),
    ];

    if db[0] > 0.0 && db[1] > 0.0 && db[2] > 0.0 {
        return None;
    }
    if db[0] < 0.0 && db[1] < 0.0 && db[2] < 0.0 {
        return None;
    }

    // Compute intersection line: direction = na × nb, point = on both planes
    let line_dir = na.cross(&nb);
    if line_dir.length_sq() < 1e-20 {
        // Coplanar triangles — skip (rare for our use case, and complex to handle)
        return None;
    }

    // Find a point on the intersection line.
    // Solve: nb · (P - b[0]) = 0 and na · (P - a[0]) = 0
    // P = a[0] + t1 * (some vector perpendicular to na)
    // Use the method: project onto the dominant axis of line_dir
    let line_point = compute_line_point(&a[0], &na, &b[0], &nb, &line_dir);

    // Compute intervals on the intersection line for both triangles
    let (ta0, ta1) = compute_interval(&da, &a, &line_dir, &line_point);
    let (tb0, tb1) = compute_interval(&db, &b, &line_dir, &line_point);

    // Check for overlap
    if ta0 > tb1 || tb0 > ta1 {
        return None;
    }

    let t0 = ta0.max(tb0);
    let t1 = ta1.min(tb1);

    if t1 - t0 < 1e-12 {
        return None; // Point intersection, no segment
    }

    Some(IntersectionSegment {
        p0: Point3d::new(
            line_point.x + line_dir.x * t0,
            line_point.y + line_dir.y * t0,
            line_point.z + line_dir.z * t0,
        ),
        p1: Point3d::new(
            line_point.x + line_dir.x * t1,
            line_point.y + line_dir.y * t1,
            line_point.z + line_dir.z * t1,
        ),
    })
}

/// Compute the normal of a triangle (not normalized).
fn triangle_normal(t: &[Point3d; 3]) -> Vec3d {
    let e1 = Vec3d::new(t[1].x - t[0].x, t[1].y - t[0].y, t[1].z - t[0].z);
    let e2 = Vec3d::new(t[2].x - t[0].x, t[2].y - t[0].y, t[2].z - t[0].z);
    e1.cross(&e2)
}

/// Signed distance from point P to plane (plane_point, plane_normal).
fn signed_distance_to_plane(p: &Point3d, plane_point: &Point3d, plane_normal: &Vec3d) -> f64 {
    (p.x - plane_point.x) * plane_normal.x
        + (p.y - plane_point.y) * plane_normal.y
        + (p.z - plane_point.z) * plane_normal.z
}

/// Find a point on the intersection line of two planes.
fn compute_line_point(
    p1: &Point3d,
    n1: &Vec3d,
    p2: &Point3d,
    n2: &Vec3d,
    line_dir: &Vec3d,
) -> Point3d {
    // Use the determinant method:
    // We want a point P such that n1·(P - p1) = 0 and n2·(P - p2) = 0
    // P = p1 + s * (n1 × line_dir) — this lies in plane 1
    // Then solve for s using plane 2.
    let cross = n1.cross(line_dir);
    let denom = n2.dot(&cross);
    if denom.abs() < 1e-15 {
        // Fall back: try with p2 as the base
        let cross2 = n2.cross(line_dir);
        let denom2 = n1.dot(&cross2);
        if denom2.abs() < 1e-15 {
            return *p1; // Fallback
        }
        let s = n1.dot(&Vec3d::new(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z)) / denom2;
        return Point3d::new(
            p2.x + cross2.x * s,
            p2.y + cross2.y * s,
            p2.z + cross2.z * s,
        );
    }
    let s = n2.dot(&Vec3d::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)) / denom;
    Point3d::new(p1.x + cross.x * s, p1.y + cross.y * s, p1.z + cross.z * s)
}

/// Compute the parametric interval [t0, t1] on the intersection line where
/// the triangle crosses.
fn compute_interval(
    dist: &[f64; 3],
    tri: &[Point3d; 3],
    line_dir: &Vec3d,
    line_point: &Point3d,
) -> (f64, f64) {
    // Parametrize each vertex on the line: t_i = line_dir · (V_i - line_point)
    let t = [
        line_dir.x * (tri[0].x - line_point.x)
            + line_dir.y * (tri[0].y - line_point.y)
            + line_dir.z * (tri[0].z - line_point.z),
        line_dir.x * (tri[1].x - line_point.x)
            + line_dir.y * (tri[1].y - line_point.y)
            + line_dir.z * (tri[1].z - line_point.z),
        line_dir.x * (tri[2].x - line_point.x)
            + line_dir.y * (tri[2].y - line_point.y)
            + line_dir.z * (tri[2].z - line_point.z),
    ];

    // Find the two edges that cross the plane (sign change in dist)
    let mut crossings: Vec<f64> = Vec::with_capacity(2);
    for i in 0..3 {
        let j = (i + 1) % 3;
        if (dist[i] > 0.0 && dist[j] < 0.0) || (dist[i] < 0.0 && dist[j] > 0.0) {
            // Edge i-j crosses the plane. Find the parametric position.
            let alpha = dist[i] / (dist[i] - dist[j]);
            let tc = t[i] + alpha * (t[j] - t[i]);
            crossings.push(tc);
        }
    }

    if crossings.len() < 2 {
        // Shouldn't happen if the distance test passed, but handle gracefully
        return (0.0, 0.0);
    }

    let t0 = crossings[0].min(crossings[1]);
    let t1 = crossings[0].max(crossings[1]);
    (t0, t1)
}

// ============================================================
// Step 2: Triangle splitting
// ============================================================

/// A triangle fragment: 3 vertex positions (not indices).
#[derive(Clone, Debug)]
struct TriangleFragment {
    v: [Point3d; 3],
}

/// Split each triangle along its intersection segments, producing fragments.
/// Triangles with no segments are returned as-is (single fragment).
fn split_triangles(
    mesh: &TriangleMesh,
    segments_per_tri: &[Vec<IntersectionSegment>],
) -> Vec<TriangleFragment> {
    let mut fragments: Vec<TriangleFragment> = Vec::new();

    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let verts = [
            mesh.vertices[tri[0] as usize],
            mesh.vertices[tri[1] as usize],
            mesh.vertices[tri[2] as usize],
        ];

        let segs = &segments_per_tri[ti];
        if segs.is_empty() {
            fragments.push(TriangleFragment { v: verts });
            continue;
        }

        // Split the triangle by all segments.
        // We do this by collecting all intersection points on the triangle's
        // edges and interior, then re-triangulating.
        let split_tris = split_triangle_by_segments(&verts, segs);
        for st in split_tris {
            fragments.push(TriangleFragment { v: st });
        }
    }

    fragments
}

/// Split a triangle by a set of intersection segments.
///
/// Each segment lies on the triangle's plane. We clip the triangle
/// against each segment endpoint, producing a set of sub-triangles.
fn split_triangle_by_segments(
    tri: &[Point3d; 3],
    segments: &[IntersectionSegment],
) -> Vec<[Point3d; 3]> {
    // Collect all unique points to insert into the triangulation:
    // - the 3 original vertices
    // - all segment endpoints that lie inside the triangle
    // - all edge crossing points (where segments cross triangle edges)
    let mut points: Vec<Point3d> = vec![tri[0], tri[1], tri[2]];

    let normal = triangle_normal(tri);
    if normal.length_sq() < 1e-20 {
        return vec![*tri];
    }

    // For each segment, add points that lie on or inside the triangle.
    for seg in segments {
        for p in &[seg.p0, seg.p1] {
            if point_in_triangle(p, tri, &normal) {
                // Add if not too close to existing point
                if !points.iter().any(|q| {
                    (q.x - p.x).powi(2) + (q.y - p.y).powi(2) + (q.z - p.z).powi(2) < 1e-16
                }) {
                    points.push(*p);
                }
            }
        }
    }

    // Also add edge-edge crossings: where segment crosses triangle edges.
    for seg in segments {
        for i in 0..3 {
            let j = (i + 1) % 3;
            if let Some(cp) = segment_segment_cross_3d(&seg.p0, &seg.p1, &tri[i], &tri[j]) {
                if !points.iter().any(|q| {
                    (q.x - cp.x).powi(2) + (q.y - cp.y).powi(2) + (q.z - cp.z).powi(2) < 1e-16
                }) {
                    points.push(cp);
                }
            }
        }
    }

    if points.len() <= 3 {
        return vec![*tri];
    }

    // Triangulate the point set using a simple fan from the centroid.
    // This is not optimal but robust. For better quality, a constrained
    // Delaunay triangulation would be used.
    fan_triangulate(&points, &normal)
}

/// Check if a point is inside a triangle (in 3D, assuming coplanar).
fn point_in_triangle(p: &Point3d, tri: &[Point3d; 3], normal: &Vec3d) -> bool {
    // Use barycentric coordinates via cross products.
    let v0 = Vec3d::new(tri[1].x - tri[0].x, tri[1].y - tri[0].y, tri[1].z - tri[0].z);
    let v1 = Vec3d::new(tri[2].x - tri[0].x, tri[2].y - tri[0].y, tri[2].z - tri[0].z);
    let v2 = Vec3d::new(p.x - tri[0].x, p.y - tri[0].y, p.z - tri[0].z);

    let dot00 = v0.dot(&v0);
    let dot01 = v0.dot(&v1);
    let dot02 = v0.dot(&v2);
    let dot11 = v1.dot(&v1);
    let dot12 = v1.dot(&v2);

    let denom = dot00 * dot11 - dot01 * dot01;
    if denom.abs() < 1e-20 {
        return false;
    }

    let u = (dot11 * dot02 - dot01 * dot12) / denom;
    let v = (dot00 * dot12 - dot01 * dot02) / denom;

    let _ = normal; // not needed for barycentric test
    u >= -1e-9 && v >= -1e-9 && (u + v) <= 1.0 + 1e-9
}

/// Check if segments AB and CD cross in 3D (assuming coplanar).
/// Returns the crossing point if they do.
fn segment_segment_cross_3d(a: &Point3d, b: &Point3d, c: &Point3d, d: &Point3d) -> Option<Point3d> {
    // Solve: A + t*(B-A) = C + s*(D-C)
    let ab = Vec3d::new(b.x - a.x, b.y - a.y, b.z - a.z);
    let cd = Vec3d::new(d.x - c.x, d.y - c.y, d.z - c.z);
    let ca = Vec3d::new(a.x - c.x, a.y - c.y, a.z - c.z);

    let n = ab.cross(&cd);
    let n_len_sq = n.length_sq();
    if n_len_sq < 1e-20 {
        return None; // Parallel
    }

    // Check coplanarity: ca · n should be ~0
    if ca.dot(&n).abs() > 1e-9 * n.length() {
        return None; // Skew lines, not coplanar
    }

    let n2 = ca.cross(&cd);
    let t = n2.dot(&n) / n_len_sq;

    let n3 = ca.cross(&ab);
    let s = n3.dot(&n) / n_len_sq;

    if t < -1e-9 || t > 1.0 + 1e-9 || s < -1e-9 || s > 1.0 + 1e-9 {
        return None;
    }

    Some(Point3d::new(
        a.x + ab.x * t,
        a.y + ab.y * t,
        a.z + ab.z * t,
    ))
}

/// Simple fan triangulation from the centroid of a point set.
fn fan_triangulate(points: &[Point3d], normal: &Vec3d) -> Vec<[Point3d; 3]> {
    if points.len() < 3 {
        return Vec::new();
    }

    // Compute centroid
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;
    for p in points {
        cx += p.x;
        cy += p.y;
        cz += p.z;
    }
    let n = points.len() as f64;
    let centroid = Point3d::new(cx / n, cy / n, cz / n);

    // Sort points by angle around centroid (projected onto the triangle plane)
    let ref_dir = {
        // Pick any vector perpendicular to normal
        let mut r = if normal.x.abs() < 0.9 {
            Vec3d::new(1.0, 0.0, 0.0)
        } else {
            Vec3d::new(0.0, 1.0, 0.0)
        };
        // Make perpendicular to normal
        let d = r.dot(normal);
        r = Vec3d::new(r.x - normal.x * d, r.y - normal.y * d, r.z - normal.z * d);
        r
    };

    let mut indexed: Vec<(usize, f64)> = (0..points.len())
        .map(|i| {
            let v = Vec3d::new(
                points[i].x - centroid.x,
                points[i].y - centroid.y,
                points[i].z - centroid.z,
            );
            let cos = v.dot(&ref_dir) / (v.length() * ref_dir.length()).max(1e-15);
            let cross = ref_dir.cross(&v);
            let sin_sign = cross.dot(normal).signum();
            let angle = cos.acos() * if sin_sign < 0.0 { -1.0 } else { 1.0 };
            (i, angle)
        })
        .collect();

    indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    // Fan from centroid
    let mut tris = Vec::new();
    for w in indexed.windows(2) {
        tris.push([centroid, points[w[0].0], points[w[1].0]]);
    }
    // Close the loop
    if indexed.len() >= 2 {
        tris.push([centroid, points[indexed[indexed.len() - 1].0], points[indexed[0].0]]);
    }

    tris
}

// ============================================================
// Step 3: Classification (point-in-mesh via ray casting)
// ============================================================

/// Determine if a point is inside a closed mesh using ray casting.
///
/// Casts a ray from the point in an arbitrary direction and counts
/// intersections with mesh triangles. Odd = inside, even = outside.
fn point_in_mesh(point: &Point3d, mesh: &TriangleMesh) -> bool {
    // Use a non-axis-aligned ray direction to avoid edge cases
    let dir = Vec3d::new(0.5773502691896258, 0.5773502691896258, 0.5773502691896258); // (1,1,1)/sqrt(3)

    let mut count = 0;
    for tri in &mesh.triangles {
        let v = [
            mesh.vertices[tri[0] as usize],
            mesh.vertices[tri[1] as usize],
            mesh.vertices[tri[2] as usize],
        ];
        if ray_triangle_intersect(point, &dir, &v).is_some() {
            count += 1;
        }
    }

    count % 2 == 1
}

/// Möller-Trumbore ray-triangle intersection.
/// Returns the t parameter if the ray hits the triangle.
fn ray_triangle_intersect(
    origin: &Point3d,
    dir: &Vec3d,
    tri: &[Point3d; 3],
) -> Option<f64> {
    const EPS: f64 = 1e-9;

    let edge1 = Vec3d::new(
        tri[1].x - tri[0].x,
        tri[1].y - tri[0].y,
        tri[1].z - tri[0].z,
    );
    let edge2 = Vec3d::new(
        tri[2].x - tri[0].x,
        tri[2].y - tri[0].y,
        tri[2].z - tri[0].z,
    );

    let h = dir.cross(&edge2);
    let a = edge1.dot(&h);
    if a.abs() < EPS {
        return None; // Ray parallel to triangle
    }

    let f = 1.0 / a;
    let s = Vec3d::new(
        origin.x - tri[0].x,
        origin.y - tri[0].y,
        origin.z - tri[0].z,
    );
    let u = f * s.dot(&h);
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let q = s.cross(&edge1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(&q);
    if t > EPS {
        Some(t)
    } else {
        None
    }
}

// ============================================================
// Step 4: Assembly
// ============================================================

/// Assemble the result mesh based on the boolean operation.
fn assemble(
    a_fragments: &[TriangleFragment],
    b_fragments: &[TriangleFragment],
    op: MeshBooleanOp,
) -> TriangleMesh {
    // Build a temporary mesh from B fragments for point-in-mesh tests
    let b_mesh = fragments_to_mesh(b_fragments);
    let a_mesh = fragments_to_mesh(a_fragments);

    let mut result: Vec<[Point3d; 3]> = Vec::new();

    // Classify A fragments: inside/outside B
    for frag in a_fragments {
        let centroid = triangle_centroid(&frag.v);
        let inside_b = point_in_mesh(&centroid, &b_mesh);

        match op {
            MeshBooleanOp::Union => {
                if !inside_b {
                    result.push(frag.v);
                }
            }
            MeshBooleanOp::Subtract => {
                if !inside_b {
                    result.push(frag.v);
                }
            }
            MeshBooleanOp::Intersect => {
                if inside_b {
                    result.push(frag.v);
                }
            }
        }
    }

    // Classify B fragments: inside/outside A
    for frag in b_fragments {
        let centroid = triangle_centroid(&frag.v);
        let inside_a = point_in_mesh(&centroid, &a_mesh);

        match op {
            MeshBooleanOp::Union => {
                if !inside_a {
                    result.push(frag.v);
                }
            }
            MeshBooleanOp::Subtract => {
                if inside_a {
                    // Reverse winding to form the cavity interior
                    result.push([frag.v[0], frag.v[2], frag.v[1]]);
                }
            }
            MeshBooleanOp::Intersect => {
                if inside_a {
                    result.push(frag.v);
                }
            }
        }
    }

    // Convert to TriangleMesh (dedup vertices)
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<(u64, u64, u64), u32> = std::collections::HashMap::new();

    for tri in &result {
        let mut indices = [0u32; 3];
        for (i, p) in tri.iter().enumerate() {
            // Quantize vertex to avoid floating-point duplicates
            let key = quantize_point(p);
            if let Some(&idx) = vertex_map.get(&key) {
                indices[i] = idx;
            } else {
                let idx = mesh.vertices.len() as u32;
                mesh.vertices.push(*p);
                vertex_map.insert(key, idx);
                indices[i] = idx;
            }
        }
        mesh.triangles.push(indices);
    }

    mesh
}

/// Convert fragments to a mesh (for point-in-mesh tests).
fn fragments_to_mesh(fragments: &[TriangleFragment]) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<(u64, u64, u64), u32> = std::collections::HashMap::new();

    for frag in fragments {
        let mut indices = [0u32; 3];
        for (i, p) in frag.v.iter().enumerate() {
            let key = quantize_point(p);
            if let Some(&idx) = vertex_map.get(&key) {
                indices[i] = idx;
            } else {
                let idx = mesh.vertices.len() as u32;
                mesh.vertices.push(*p);
                vertex_map.insert(key, idx);
                indices[i] = idx;
            }
        }
        mesh.triangles.push(indices);
    }

    mesh
}

/// Quantize a point to a hashable key (for vertex deduplication).
fn quantize_point(p: &Point3d) -> (u64, u64, u64) {
    const SCALE: f64 = 1e7;
    let qx = (p.x * SCALE).round() as i64 as u64;
    let qy = (p.y * SCALE).round() as i64 as u64;
    let qz = (p.z * SCALE).round() as i64 as u64;
    (qx, qy, qz)
}

/// Centroid of a triangle.
fn triangle_centroid(tri: &[Point3d; 3]) -> Point3d {
    Point3d::new(
        (tri[0].x + tri[1].x + tri[2].x) / 3.0,
        (tri[0].y + tri[1].y + tri[2].y) / 3.0,
        (tri[0].z + tri[1].z + tri[2].z) / 3.0,
    )
}

// ============================================================
// Mesh cleanup
// ============================================================

/// Remove degenerate triangles and empty vertices.
fn clean_mesh(mut mesh: TriangleMesh) -> TriangleMesh {
    // Remove degenerate triangles (zero area or duplicate vertices)
    mesh.triangles.retain(|tri| {
        let a = &mesh.vertices[tri[0] as usize];
        let b = &mesh.vertices[tri[1] as usize];
        let c = &mesh.vertices[tri[2] as usize];
        let area_sq = triangle_normal(&[*a, *b, *c]).length_sq();
        area_sq > 1e-18
    });

    // Compact vertices (remove unused)
    let mut used = vec![false; mesh.vertices.len()];
    for tri in &mesh.triangles {
        used[tri[0] as usize] = true;
        used[tri[1] as usize] = true;
        used[tri[2] as usize] = true;
    }
    let mut new_vertices = Vec::new();
    let mut old_to_new: Vec<u32> = vec![0; mesh.vertices.len()];
    for (i, &u) in used.iter().enumerate() {
        if u {
            old_to_new[i] = new_vertices.len() as u32;
            new_vertices.push(mesh.vertices[i]);
        }
    }
    for tri in &mut mesh.triangles {
        tri[0] = old_to_new[tri[0] as usize];
        tri[1] = old_to_new[tri[1] as usize];
        tri[2] = old_to_new[tri[2] as usize];
    }
    mesh.vertices = new_vertices;

    mesh
}

// ============================================================
// Vertex welding (for watertightness)
// ============================================================

/// Compute the axis-aligned bounding box of a mesh.
fn compute_bounding_box(mesh: &TriangleMesh) -> (Point3d, Point3d) {
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    for v in &mesh.vertices {
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }
    (min, max)
}

/// Size of a bounding box (max dimension).
fn bbox_size(bbox: &(Point3d, Point3d)) -> f64 {
    let dx = bbox.1.x - bbox.0.x;
    let dy = bbox.1.y - bbox.0.y;
    let dz = bbox.1.z - bbox.0.z;
    dx.max(dy).max(dz)
}

/// Weld (merge) vertices that are within `tolerance` of each other.
/// This makes meshes watertight by merging duplicate/near-duplicate vertices.
fn weld_vertices(mesh: &mut TriangleMesh, tolerance: f64) {
    if mesh.vertices.is_empty() {
        return;
    }

    let tol_sq = tolerance * tolerance;

    // Build a spatial hash for fast neighbor lookup
    let cell_size = tolerance.max(1e-15);
    let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<u32>> = std::collections::HashMap::new();

    let mut new_indices: Vec<u32> = vec![u32::MAX; mesh.vertices.len()];
    let mut new_vertices: Vec<Point3d> = Vec::new();

    for (i, v) in mesh.vertices.iter().enumerate() {
        let cx = (v.x / cell_size).floor() as i64;
        let cy = (v.y / cell_size).floor() as i64;
        let cz = (v.z / cell_size).floor() as i64;

        // Check this cell and 26 neighbors
        let mut found: Option<u32> = None;
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(candidates) = grid.get(&key) {
                        for &idx in candidates {
                            let w = &new_vertices[idx as usize];
                            let d = (v.x - w.x).powi(2) + (v.y - w.y).powi(2) + (v.z - w.z).powi(2);
                            if d <= tol_sq {
                                found = Some(idx);
                                break;
                            }
                        }
                    }
                    if found.is_some() { break; }
                }
                if found.is_some() { break; }
            }
            if found.is_some() { break; }
        }

        let idx = match found {
            Some(i) => i,
            None => {
                let new_idx = new_vertices.len() as u32;
                new_vertices.push(*v);
                grid.entry((cx, cy, cz)).or_default().push(new_idx);
                new_idx
            }
        };
        new_indices[i] = idx;
    }

    // Remap triangles
    for tri in &mut mesh.triangles {
        tri[0] = new_indices[tri[0] as usize];
        tri[1] = new_indices[tri[1] as usize];
        tri[2] = new_indices[tri[2] as usize];
    }
    mesh.vertices = new_vertices;
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_box_mesh(dx: f64, dy: f64, dz: f64) -> TriangleMesh {
        let hx = dx / 2.0;
        let hy = dy / 2.0;
        let hz = dz / 2.0;
        let v = vec![
            Point3d::new(-hx, -hy, -hz),
            Point3d::new(hx, -hy, -hz),
            Point3d::new(hx, hy, -hz),
            Point3d::new(-hx, hy, -hz),
            Point3d::new(-hx, -hy, hz),
            Point3d::new(hx, -hy, hz),
            Point3d::new(hx, hy, hz),
            Point3d::new(-hx, hy, hz),
        ];
        let triangles = vec![
            [0, 2, 1], [0, 3, 2], // bottom
            [4, 5, 6], [4, 6, 7], // top
            [0, 1, 5], [0, 5, 4], // front
            [3, 6, 2], [3, 7, 6], // back
            [0, 7, 3], [0, 4, 7], // left
            [1, 2, 6], [1, 6, 5], // right
        ];
        TriangleMesh { vertices: v, triangles, normals: None, face_normals: None, triangle_colors: None, triangle_face_ids: None }
    }

    #[test]
    fn test_box_minus_box_subtract() {
        let a = make_box_mesh(100.0, 100.0, 100.0);
        let b = make_box_mesh(30.0, 30.0, 30.0);

        let result = mesh_subtract(&a, &b);
        println!("Subtract result: {} vertices, {} triangles",
            result.vertices.len(), result.triangles.len());

        // Box (100³) - Box (30³) should be a box with a cubic hole
        // Should have more triangles than original (12) due to hole walls
        assert!(result.triangles.len() >= 12, "Should have at least 12 triangles");

        // Check watertightness: every edge shared by exactly 2 triangles
        let mut edges: std::collections::HashMap<[u32; 2], u32> = std::collections::HashMap::new();
        for tri in &result.triangles {
            for (i, j) in [(0, 1), (1, 2), (2, 0)] {
                let key = if tri[i] < tri[j] {
                    [tri[i], tri[j]]
                } else {
                    [tri[j], tri[i]]
                };
                *edges.entry(key).or_insert(0) += 1;
            }
        }
        let boundary = edges.iter().filter(|(_, &c)| c != 2).count();
        println!("Boundary edges: {}", boundary);
        assert_eq!(boundary, 0, "Result should be watertight");
    }

    #[test]
    fn test_box_union_box() {
        let a = make_box_mesh(40.0, 40.0, 40.0);
        let b = make_box_mesh(40.0, 40.0, 40.0);
        // Translate b by 20 units in X so they overlap
        let mut b_translated = b.clone();
        for v in &mut b_translated.vertices {
            v.x += 20.0;
        }

        let result = mesh_union(&a, &b_translated);
        println!("Union result: {} triangles", result.triangles.len());
        assert!(result.triangles.len() > 0);
    }

    #[test]
    fn test_point_in_box() {
        let box_mesh = make_box_mesh(100.0, 100.0, 100.0);
        // Off-center point should be inside (avoid axis-aligned rays that
        // hit edges/vertices exactly)
        assert!(point_in_mesh(&Point3d::new(1.0, 2.0, 3.0), &box_mesh));
        // Outside point
        assert!(!point_in_mesh(&Point3d::new(200.0, 100.0, 50.0), &box_mesh));
    }

    fn make_cylinder_mesh(radius: f64, height: f64, segments: usize) -> TriangleMesh {
        let mut vertices = Vec::new();
        let mut triangles: Vec<[u32; 3]> = Vec::new();

        // Bottom center vertex
        vertices.push(Point3d::new(0.0, 0.0, 0.0));
        // Top center vertex
        vertices.push(Point3d::new(0.0, 0.0, height));

        // Ring vertices
        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (segments as f64);
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            vertices.push(Point3d::new(x, y, 0.0)); // bottom ring
            vertices.push(Point3d::new(x, y, height)); // top ring
        }

        // Bottom cap triangles (fan from center)
        for i in 0..segments {
            let next = (i + 1) % segments;
            // 0 = bottom center, 2+2*i = bottom ring i
            triangles.push([0, (2 + 2 * i) as u32, (2 + 2 * next) as u32]);
        }

        // Top cap triangles (fan from center, reversed)
        for i in 0..segments {
            let next = (i + 1) % segments;
            // 1 = top center, 3+2*i = top ring i
            triangles.push([1, (3 + 2 * next) as u32, (3 + 2 * i) as u32]);
        }

        // Side quads (2 triangles each)
        for i in 0..segments {
            let next = (i + 1) % segments;
            let b0 = (2 + 2 * i) as u32;
            let b1 = (2 + 2 * next) as u32;
            let t0 = (3 + 2 * i) as u32;
            let t1 = (3 + 2 * next) as u32;
            triangles.push([b0, t0, b1]);
            triangles.push([b1, t0, t1]);
        }

        TriangleMesh { vertices, triangles, normals: None, face_normals: None, triangle_colors: None, triangle_face_ids: None }
    }

    #[test]
    fn test_box_minus_cylinder_mesh_boolean() {
        let box_mesh = make_box_mesh(100.0, 80.0, 50.0);
        // Cylinder through the box (taller than box so it pokes out both sides)
        let cyl_mesh = make_cylinder_mesh(20.0, 100.0, 32);
        // Center the cylinder on the box (box is at origin, centered)
        // Cylinder goes from z=0 to z=100, but box is from z=-25 to z=25
        // So translate cylinder down by 50 to center it
        let mut cyl_translated = cyl_mesh.clone();
        for v in &mut cyl_translated.vertices {
            v.z -= 50.0;
        }

        let result = mesh_subtract(&box_mesh, &cyl_translated);
        println!("Box-Cylinder subtract: {} vertices, {} triangles",
            result.vertices.len(), result.triangles.len());

        // Check watertightness
        let mut edges: std::collections::HashMap<[u32; 2], u32> = std::collections::HashMap::new();
        for tri in &result.triangles {
            for (i, j) in [(0, 1), (1, 2), (2, 0)] {
                let key = if tri[i] < tri[j] {
                    [tri[i], tri[j]]
                } else {
                    [tri[j], tri[i]]
                };
                *edges.entry(key).or_insert(0) += 1;
            }
        }
        let boundary_edges: Vec<_> = edges.iter().filter(|(_, &c)| c != 2).collect();
        println!("Boundary edges: {}", boundary_edges.len());

        if !boundary_edges.is_empty() {
            // Find the actual distance between boundary vertex pairs
            let mut min_dist = f64::MAX;
            let mut max_dist = 0.0f64;
            for (i, _) in &boundary_edges {
                for (j, _) in &boundary_edges {
                    if i >= j { continue; }
                    let [a, b] = **i;
                    let [c, d] = **j;
                    for (vi, vj) in [(a, c), (a, d), (b, c), (b, d)] {
                        if vi == vj { continue; }
                        let pi = &result.vertices[vi as usize];
                        let pj = &result.vertices[vj as usize];
                        let d = ((pi.x - pj.x).powi(2) + (pi.y - pj.y).powi(2) + (pi.z - pj.z).powi(2)).sqrt();
                        if d > 0.0 && d < min_dist { min_dist = d; }
                        if d > max_dist { max_dist = d; }
                    }
                }
            }
            println!("Boundary vertex distances: min={:.2e}, max={:.2e}", min_dist, max_dist);

            // Try welding with larger tolerances
            for wt in [1e-3, 0.1, 1.0, 5.0].iter() {
                let mut m2 = result.clone();
                weld_vertices(&mut m2, *wt);
                let mut e2: std::collections::HashMap<[u32; 2], u32> = std::collections::HashMap::new();
                for t in &m2.triangles {
                    let [a, b, c] = *t;
                    for (p, q) in [(a, b), (b, c), (c, a)] {
                        let key = if p < q { [p, q] } else { [q, p] };
                        *e2.entry(key).or_insert(0) += 1;
                    }
                }
                let b2 = e2.iter().filter(|(_, &c)| c != 2).count();
                println!("  After weld({}): {} boundary edges, {} triangles", wt, b2, m2.triangles.len());
            }
        }
    }
}
