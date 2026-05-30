// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.12/T.13 — Mesh Validity / No Zero-Area Triangles
//!
//! Comprehensive mesh validity checking: NaN/Inf detection,
//! zero-area triangle detection and filtering.

use draper_mesh::TriangleMesh;

/// Comprehensive mesh validity report.
#[derive(Debug)]
pub struct MeshValidity {
    /// Whether the mesh passes all validity checks.
    pub valid: bool,
    /// Indices of vertices with NaN or Inf coordinates.
    pub nan_vertices: Vec<u32>,
    /// Indices of triangles with NaN or Inf normals.
    pub nan_normals: Vec<u32>,
    /// Indices of zero-area triangles.
    pub zero_area_triangles: Vec<u32>,
    /// Total count of degenerate elements.
    pub degenerate_count: usize,
}

/// Find vertices with NaN or Inf coordinates.
/// Returns the indices of invalid vertices.
pub fn has_nan_vertices(mesh: &TriangleMesh) -> Vec<u32> {
    let mut result = Vec::new();
    for (i, v) in mesh.vertices.iter().enumerate() {
        if !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite() {
            result.push(i as u32);
        }
    }
    result
}

/// Find triangles with NaN or Inf face normals.
/// Returns the indices of invalid triangles.
/// If face_normals is not computed, computes them on-the-fly.
pub fn has_nan_normals(mesh: &TriangleMesh) -> Vec<u32> {
    let mut result = Vec::new();

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        let e1x = v1.x - v0.x;
        let e1y = v1.y - v0.y;
        let e1z = v1.z - v0.z;
        let e2x = v2.x - v0.x;
        let e2y = v2.y - v0.y;
        let e2z = v2.z - v0.z;

        let nx = e1y * e2z - e1z * e2y;
        let ny = e1z * e2x - e1x * e2z;
        let nz = e1x * e2y - e1y * e2x;

        if !nx.is_finite() || !ny.is_finite() || !nz.is_finite() {
            result.push(i as u32);
        }
    }

    result
}

/// Find zero-area triangles (area < tolerance²).
/// Returns the indices of degenerate triangles.
pub fn has_zero_area_triangles(mesh: &TriangleMesh) -> Vec<u32> {
    has_zero_area_triangles_with_tolerance(mesh, 1e-20)
}

/// Find zero-area triangles with a custom tolerance.
/// A triangle is considered zero-area if its area is less than `tolerance`.
pub fn has_zero_area_triangles_with_tolerance(mesh: &TriangleMesh, tolerance: f64) -> Vec<u32> {
    let mut result = Vec::new();
    for (i, tri) in mesh.triangles.iter().enumerate() {
        let area = triangle_area(mesh, tri);
        if area < tolerance {
            result.push(i as u32);
        }
    }
    result
}

/// Compute the area of a single triangle.
fn triangle_area(mesh: &TriangleMesh, tri: &[u32; 3]) -> f64 {
    let v0 = mesh.vertices[tri[0] as usize];
    let v1 = mesh.vertices[tri[1] as usize];
    let v2 = mesh.vertices[tri[2] as usize];
    let e1x = v1.x - v0.x;
    let e1y = v1.y - v0.y;
    let e1z = v1.z - v0.z;
    let e2x = v2.x - v0.x;
    let e2y = v2.y - v0.y;
    let e2z = v2.z - v0.z;
    let cx = e1y * e2z - e1z * e2y;
    let cy = e1z * e2x - e1x * e2z;
    let cz = e1x * e2y - e1y * e2x;
    (cx * cx + cy * cy + cz * cz).sqrt() * 0.5
}

/// Perform a comprehensive validity check on a mesh.
pub fn validate_mesh(mesh: &TriangleMesh) -> MeshValidity {
    let nan_verts = has_nan_vertices(mesh);
    let nan_norms = has_nan_normals(mesh);
    let zero_area = has_zero_area_triangles(mesh);

    let valid = nan_verts.is_empty() && nan_norms.is_empty() && zero_area.is_empty();
    let degenerate_count = zero_area.len();

    MeshValidity {
        valid,
        nan_vertices: nan_verts,
        nan_normals: nan_norms,
        zero_area_triangles: zero_area,
        degenerate_count,
    }
}

/// Remove zero-area triangles from a mesh.
/// Returns the number of triangles removed.
pub fn filter_zero_area_triangles(mesh: &mut TriangleMesh, tolerance: f64) -> usize {
    let original_count = mesh.triangles.len();

    let indices_to_remove: std::collections::HashSet<usize> = has_zero_area_triangles_with_tolerance(mesh, tolerance)
        .into_iter()
        .map(|i| i as usize)
        .collect();

    let mut new_triangles = Vec::with_capacity(mesh.triangles.len());
    for (i, tri) in mesh.triangles.drain(..).enumerate() {
        if !indices_to_remove.contains(&i) {
            new_triangles.push(tri);
        }
    }
    mesh.triangles = new_triangles;

    // Also filter face_normals, triangle_colors, triangle_face_ids if present
    if let Some(ref mut face_normals) = mesh.face_normals {
        let mut new_normals = Vec::with_capacity(face_normals.len());
        for (i, n) in face_normals.drain(..).enumerate() {
            if !indices_to_remove.contains(&i) {
                new_normals.push(n);
            }
        }
        *face_normals = new_normals;
    }

    if let Some(ref mut colors) = mesh.triangle_colors {
        let mut new_colors = Vec::with_capacity(colors.len());
        for (i, c) in colors.drain(..).enumerate() {
            if !indices_to_remove.contains(&i) {
                new_colors.push(c);
            }
        }
        *colors = new_colors;
    }

    if let Some(ref mut face_ids) = mesh.triangle_face_ids {
        let mut new_ids = Vec::with_capacity(face_ids.len());
        for (i, id) in face_ids.drain(..).enumerate() {
            if !indices_to_remove.contains(&i) {
                new_ids.push(id);
            }
        }
        *face_ids = new_ids;
    }

    original_count - mesh.triangles.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_mesh() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let validity = validate_mesh(&mesh);
        assert!(validity.valid, "Simple triangle mesh should be valid");
        assert!(validity.nan_vertices.is_empty());
        assert!(validity.zero_area_triangles.is_empty());
    }

    #[test]
    fn test_nan_vertex_detection() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(f64::NAN, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let nan_verts = has_nan_vertices(&mesh);
        assert_eq!(nan_verts.len(), 1);
        assert_eq!(nan_verts[0], 0);
    }

    #[test]
    fn test_inf_vertex_detection() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(f64::INFINITY, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let nan_verts = has_nan_vertices(&mesh);
        assert_eq!(nan_verts.len(), 1);
        assert_eq!(nan_verts[0], 0);
    }

    #[test]
    fn test_zero_area_triangle_detection() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // Same as vertex 0 → zero area
        mesh.add_triangle(0, 1, 2);

        let zero_area = has_zero_area_triangles(&mesh);
        assert_eq!(zero_area.len(), 1, "Should detect 1 zero-area triangle");
        assert_eq!(zero_area[0], 0);
    }

    #[test]
    fn test_filter_zero_area_triangles() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0)); // Normal
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // Degenerate
        mesh.add_triangle(0, 1, 2); // Normal
        mesh.add_triangle(0, 1, 3); // Zero-area (vertex 0 and 3 are same)

        let removed = filter_zero_area_triangles(&mut mesh, 1e-20);
        assert_eq!(removed, 1, "Should remove 1 zero-area triangle");
        assert_eq!(mesh.triangle_count(), 1, "Should have 1 triangle left");
    }

    #[test]
    fn test_validate_mesh_with_problems() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(f64::NAN, 0.0, 0.0)); // NaN vertex
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // Zero-area with v0
        mesh.add_triangle(0, 1, 2); // Has NaN vertex + is zero-area

        let validity = validate_mesh(&mesh);
        assert!(!validity.valid);
        assert_eq!(validity.nan_vertices.len(), 1);
        assert_eq!(validity.degenerate_count, 1);
    }
}
