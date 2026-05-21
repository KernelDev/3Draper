//! Mesh-level Boolean operations.
//!
//! Implements union, intersection, and difference operations on triangle meshes.
//! Uses ray-casting for inside/outside classification.
//!
//! This is a practical approach for rendering complex CAD models where
//! full B-Rep Boolean operations (requiring surface-surface intersection)
//! are not yet available.
//!
//! Limitations:
//! - Both input meshes must be watertight (closed)
//! - Intersection edges are approximated (no exact curve computation)
//! - Small gaps may appear at intersection boundaries

use crate::triangulate::TriangleMesh;
use draper_geometry::point::Point3;

/// Result of a mesh boolean operation.
#[derive(Debug)]
pub struct BooleanResult {
    /// The resulting mesh.
    pub mesh: TriangleMesh,
    /// Number of triangles kept from shape A.
    pub kept_from_a: usize,
    /// Number of triangles kept from shape B.
    pub kept_from_b: usize,
}

/// Test if a point is inside a watertight mesh using ray casting.
///
/// Casts rays from the point in slightly perturbed directions and uses
/// majority voting for robustness. This handles edge cases where a ray
/// passes through a shared edge or vertex of adjacent triangles.
pub fn point_in_mesh(point: Point3, mesh: &TriangleMesh) -> bool {
    // Use 3 slightly perturbed ray directions to avoid hitting exact edges/vertices.
    // Pure axis-aligned rays can pass through shared triangle edges at the center
    // of symmetric meshes, giving double-counted intersections.
    let directions = [
        glam::DVec3::new(1.0, 0.0001, 0.0002), // Slightly off +X
        glam::DVec3::new(0.0003, 1.0, 0.0001), // Slightly off +Y
        glam::DVec3::new(0.0001, 0.0002, 1.0), // Slightly off +Z
    ];

    let mut inside_votes = 0;
    let origin = point.to_dvec3();

    for ray_dir in &directions {
        let ray_dir = ray_dir.normalize();
        let mut crossings = 0i32;
        for tri in mesh.indices.chunks(3) {
            let a = mesh.vertices[tri[0] as usize].to_dvec3();
            let b = mesh.vertices[tri[1] as usize].to_dvec3();
            let c = mesh.vertices[tri[2] as usize].to_dvec3();

            if ray_triangle_intersect(origin, ray_dir, a, b, c).is_some() {
                crossings += 1;
            }
        }
        if crossings % 2 == 1 {
            inside_votes += 1;
        }
    }

    inside_votes >= 2 // Majority vote: at least 2 out of 3 rays say inside
}

/// Möller–Trumbore ray-triangle intersection.
///
/// Returns the parameter t along the ray if intersection occurs, with t > 0.
fn ray_triangle_intersect(
    origin: glam::DVec3,
    dir: glam::DVec3,
    v0: glam::DVec3,
    v1: glam::DVec3,
    v2: glam::DVec3,
) -> Option<f64> {
    const EPSILON: f64 = 1e-10;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = dir.cross(edge2);
    let a = edge1.dot(h);

    if a.abs() < EPSILON {
        return None; // Ray parallel to triangle
    }

    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(h);
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let q = s.cross(edge1);
    let v = f * dir.dot(q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(q);
    if t > EPSILON {
        Some(t)
    } else {
        None
    }
}

/// Classify each triangle of mesh_a as inside or outside mesh_b.
///
/// Returns a boolean vector: true = triangle centroid is inside mesh_b.
fn classify_triangles(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> Vec<bool> {
    let mut result = Vec::with_capacity(mesh_a.triangle_count());

    for tri in mesh_a.indices.chunks(3) {
        let a = mesh_a.vertices[tri[0] as usize].to_dvec3();
        let b = mesh_a.vertices[tri[1] as usize].to_dvec3();
        let c = mesh_a.vertices[tri[2] as usize].to_dvec3();
        let centroid = Point3::from_dvec3((a + b + c) / 3.0);

        result.push(point_in_mesh(centroid, mesh_b));
    }

    result
}

/// Mesh boolean difference: A - B.
///
/// Keeps triangles of A that are outside B, plus triangles of B that are inside A (reversed).
pub fn mesh_difference(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> TriangleMesh {
    let a_inside_b = classify_triangles(mesh_a, mesh_b);
    let b_inside_a = classify_triangles(mesh_b, mesh_a);

    let mut result = TriangleMesh::new();

    // Add triangles from A that are OUTSIDE B
    let mut kept_a = 0usize;
    let base_a = 0u32;
    for (i, tri) in mesh_a.indices.chunks(3).enumerate() {
        if !a_inside_b[i] {
            let idx0 = result.add_vertex(mesh_a.vertices[tri[0] as usize]);
            let idx1 = result.add_vertex(mesh_a.vertices[tri[1] as usize]);
            let idx2 = result.add_vertex(mesh_a.vertices[tri[2] as usize]);
            result.add_triangle(idx0, idx1, idx2);
            kept_a += 1;
        }
    }

    // Add triangles from B that are INSIDE A (with reversed winding)
    let mut kept_b = 0usize;
    for (i, tri) in mesh_b.indices.chunks(3).enumerate() {
        if b_inside_a[i] {
            let idx0 = result.add_vertex(mesh_b.vertices[tri[0] as usize]);
            let idx1 = result.add_vertex(mesh_b.vertices[tri[1] as usize]);
            let idx2 = result.add_vertex(mesh_b.vertices[tri[2] as usize]);
            // Reverse winding for inside-out orientation
            result.add_triangle(idx2, idx1, idx0);
            kept_b += 1;
        }
    }

    log::info!(
        "Boolean difference: kept {} triangles from A (outside B), {} from B (inside A, reversed)",
        kept_a, kept_b
    );

    result.compute_normals();
    result
}

/// Mesh boolean union: A ∪ B.
///
/// Keeps triangles of A that are outside B, plus triangles of B that are outside A.
pub fn mesh_union(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> TriangleMesh {
    let a_inside_b = classify_triangles(mesh_a, mesh_b);
    let b_inside_a = classify_triangles(mesh_b, mesh_a);

    let mut result = TriangleMesh::new();

    // Add triangles from A that are OUTSIDE B
    for (i, tri) in mesh_a.indices.chunks(3).enumerate() {
        if !a_inside_b[i] {
            let idx0 = result.add_vertex(mesh_a.vertices[tri[0] as usize]);
            let idx1 = result.add_vertex(mesh_a.vertices[tri[1] as usize]);
            let idx2 = result.add_vertex(mesh_a.vertices[tri[2] as usize]);
            result.add_triangle(idx0, idx1, idx2);
        }
    }

    // Add triangles from B that are OUTSIDE A
    for (i, tri) in mesh_b.indices.chunks(3).enumerate() {
        if !b_inside_a[i] {
            let idx0 = result.add_vertex(mesh_b.vertices[tri[0] as usize]);
            let idx1 = result.add_vertex(mesh_b.vertices[tri[1] as usize]);
            let idx2 = result.add_vertex(mesh_b.vertices[tri[2] as usize]);
            result.add_triangle(idx0, idx1, idx2);
        }
    }

    result.compute_normals();
    result
}

/// Mesh boolean intersection: A ∩ B.
///
/// Keeps triangles of A that are inside B, plus triangles of B that are inside A.
pub fn mesh_intersection(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> TriangleMesh {
    let a_inside_b = classify_triangles(mesh_a, mesh_b);
    let b_inside_a = classify_triangles(mesh_b, mesh_a);

    let mut result = TriangleMesh::new();

    // Add triangles from A that are INSIDE B
    for (i, tri) in mesh_a.indices.chunks(3).enumerate() {
        if a_inside_b[i] {
            let idx0 = result.add_vertex(mesh_a.vertices[tri[0] as usize]);
            let idx1 = result.add_vertex(mesh_a.vertices[tri[1] as usize]);
            let idx2 = result.add_vertex(mesh_a.vertices[tri[2] as usize]);
            result.add_triangle(idx0, idx1, idx2);
        }
    }

    // Add triangles from B that are INSIDE A
    for (i, tri) in mesh_b.indices.chunks(3).enumerate() {
        if b_inside_a[i] {
            let idx0 = result.add_vertex(mesh_b.vertices[tri[0] as usize]);
            let idx1 = result.add_vertex(mesh_b.vertices[tri[1] as usize]);
            let idx2 = result.add_vertex(mesh_b.vertices[tri[2] as usize]);
            result.add_triangle(idx0, idx1, idx2);
        }
    }

    result.compute_normals();
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_in_mesh_unit_cube() {
        // Use a 10x10x10 cube to avoid numerical issues at exact boundaries
        let mut mesh = TriangleMesh::new();
        let v = [
            mesh.add_vertex(Point3::new(0.0, 0.0, 0.0)),
            mesh.add_vertex(Point3::new(10.0, 0.0, 0.0)),
            mesh.add_vertex(Point3::new(10.0, 10.0, 0.0)),
            mesh.add_vertex(Point3::new(0.0, 10.0, 0.0)),
            mesh.add_vertex(Point3::new(0.0, 0.0, 10.0)),
            mesh.add_vertex(Point3::new(10.0, 0.0, 10.0)),
            mesh.add_vertex(Point3::new(10.0, 10.0, 10.0)),
            mesh.add_vertex(Point3::new(0.0, 10.0, 10.0)),
        ];
        // Bottom (z=0)
        mesh.add_triangle(v[0], v[2], v[1]);
        mesh.add_triangle(v[0], v[3], v[2]);
        // Top (z=10)
        mesh.add_triangle(v[4], v[5], v[6]);
        mesh.add_triangle(v[4], v[6], v[7]);
        // Front (y=0)
        mesh.add_triangle(v[0], v[1], v[5]);
        mesh.add_triangle(v[0], v[5], v[4]);
        // Back (y=10)
        mesh.add_triangle(v[3], v[7], v[6]);
        mesh.add_triangle(v[3], v[6], v[2]);
        // Left (x=0)
        mesh.add_triangle(v[0], v[4], v[7]);
        mesh.add_triangle(v[0], v[7], v[3]);
        // Right (x=10)
        mesh.add_triangle(v[1], v[2], v[6]);
        mesh.add_triangle(v[1], v[6], v[5]);

        // Inside
        assert!(point_in_mesh(Point3::new(5.0, 5.0, 5.0), &mesh));
        assert!(point_in_mesh(Point3::new(1.0, 9.0, 1.0), &mesh));

        // Outside
        assert!(!point_in_mesh(Point3::new(20.0, 5.0, 5.0), &mesh));
        assert!(!point_in_mesh(Point3::new(-10.0, 5.0, 5.0), &mesh));
    }

    #[test]
    fn test_ray_triangle_intersect() {
        let v0 = glam::DVec3::new(0.0, 0.0, 0.0);
        let v1 = glam::DVec3::new(1.0, 0.0, 0.0);
        let v2 = glam::DVec3::new(0.0, 1.0, 0.0);

        // Ray from above hitting the triangle
        let origin = glam::DVec3::new(0.25, 0.25, 1.0);
        let dir = glam::DVec3::new(0.0, 0.0, -1.0);
        assert!(ray_triangle_intersect(origin, dir, v0, v1, v2).is_some());

        // Ray from below missing the triangle
        let origin_miss = glam::DVec3::new(0.25, 0.25, -1.0);
        let dir_miss = glam::DVec3::new(0.0, 0.0, -1.0);
        assert!(ray_triangle_intersect(origin_miss, dir_miss, v0, v1, v2).is_none());
    }
}
