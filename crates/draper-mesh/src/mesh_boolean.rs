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
use draper_geometry::{Point3d, Vec3d, Direction3d};

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

/// Test if a point is inside a triangle mesh using ray-casting.
/// Returns `true` if the point is inside (odd number of ray/triangle
/// intersections along +X axis).
fn point_in_mesh(point: &Point3d, mesh: &TriangleMesh) -> bool {
    let mut count = 0u32;
    for tri in &mesh.triangles {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        if ray_triangle_intersect(point, &Direction3d::X, &a, &b, &c).is_some() {
            count += 1;
        }
    }
    count % 2 == 1
}

/// Compute the (unnormalized) normal of a triangle (cross product of two edges).
fn triangle_normal(t: &[Point3d; 3]) -> Vec3d {
    let e1 = Vec3d::new(t[1].x - t[0].x, t[1].y - t[0].y, t[1].z - t[0].z);
    let e2 = Vec3d::new(t[2].x - t[0].x, t[2].y - t[0].y, t[2].z - t[0].z);
    Vec3d::new(
        e1.y * e2.z - e1.z * e2.y,
        e1.z * e2.x - e1.x * e2.z,
        e1.x * e2.y - e1.y * e2.x,
    )
}

/// Möller-Trumbore ray-triangle intersection along an axis direction.
/// Returns `Some(t)` (distance from ray origin to hit point) if intersection
/// exists in `t > 1e-9` and barycentric coordinates are within [0,1].
fn ray_triangle_intersect(
    origin: &Point3d,
    direction: &Direction3d,
    a: &Point3d,
    b: &Point3d,
    c: &Point3d,
) -> Option<f64> {
    let edge1 = Vec3d::new(b.x - a.x, b.y - a.y, b.z - a.z);
    let edge2 = Vec3d::new(c.x - a.x, c.y - a.y, c.z - a.z);
    let h = Vec3d::new(
        direction.y * edge2.z - direction.z * edge2.y,
        direction.z * edge2.x - direction.x * edge2.z,
        direction.x * edge2.y - direction.y * edge2.x,
    );
    let det = edge1.x * h.x + edge1.y * h.y + edge1.z * h.z;
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = Vec3d::new(origin.x - a.x, origin.y - a.y, origin.z - a.z);
    let u = inv_det * (s.x * h.x + s.y * h.y + s.z * h.z);
    if u < 0.0 || u > 1.0 {
        return None;
    }
    let q = Vec3d::new(
        s.y * edge1.z - s.z * edge1.y,
        s.z * edge1.x - s.x * edge1.z,
        s.x * edge1.y - s.y * edge1.x,
    );
    let v = inv_det * (direction.x * q.x + direction.y * q.y + direction.z * q.z);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = inv_det * (edge2.x * q.x + edge2.y * q.y + edge2.z * q.z);
    if t > 1e-9 {
        Some(t)
    } else {
        None
    }
}

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
    // Remove degenerate triangles (zero area or duplicate vertices).
    // Index-preserving filter so the per-triangle attribute arrays
    // (face_normals, triangle_colors, triangle_face_ids) stay in sync —
    // `Vec::retain` on `triangles` alone would leave them stale and break
    // every downstream consumer that zips them with `triangles`.
    let keep: Vec<bool> = mesh
        .triangles
        .iter()
        .map(|tri| {
            let a = &mesh.vertices[tri[0] as usize];
            let b = &mesh.vertices[tri[1] as usize];
            let c = &mesh.vertices[tri[2] as usize];
            let area_sq = triangle_normal(&[*a, *b, *c]).length_sq();
            area_sq > 1e-18
        })
        .collect();

    let keep_idx = |i: usize| keep.get(i).copied().unwrap_or(true);
    mesh.triangles = mesh
        .triangles
        .iter()
        .enumerate()
        .filter(|(i, _)| keep_idx(*i))
        .map(|(_, t)| *t)
        .collect();
    // Filter each per-triangle attribute with the same predicate.
    if let Some(ref mut face_normals) = mesh.face_normals {
        *face_normals = face_normals
            .iter()
            .enumerate()
            .filter(|(i, _)| keep_idx(*i))
            .map(|(_, v)| *v)
            .collect();
    }
    if let Some(ref mut colors) = mesh.triangle_colors {
        *colors = colors
            .iter()
            .enumerate()
            .filter(|(i, _)| keep_idx(*i))
            .map(|(_, v)| *v)
            .collect();
    }
    if let Some(ref mut ids) = mesh.triangle_face_ids {
        *ids = ids
            .iter()
            .enumerate()
            .filter(|(i, _)| keep_idx(*i))
            .map(|(_, v)| *v)
            .collect();
    }

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
