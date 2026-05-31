// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.8 — Watertight Test
//!
//! Test mesh watertightness using `draper_mesh::manifold::ManifoldChecker`.

use draper_mesh::{TriangleMesh, check_manifold, ManifoldReport};

/// Result of a watertight test on a mesh.
#[derive(Debug)]
pub struct WatertightResult {
    /// Whether the mesh is watertight (no boundary edges).
    pub is_watertight: bool,
    /// Number of boundary edges (edges shared by only 1 triangle).
    pub boundary_edge_count: usize,
    /// Euler characteristic: V - E + F.
    pub euler_char: i64,
    /// Expected Euler characteristic for the mesh topology.
    pub expected_euler: i64,
}

/// Test a triangle mesh for watertightness.
///
/// Uses `draper_mesh::check_manifold()` internally.
/// Returns a `WatertightResult` with detailed information.
pub fn test_watertight(mesh: &TriangleMesh) -> WatertightResult {
    let report = check_manifold(mesh);
    WatertightResult {
        is_watertight: report.is_watertight(),
        boundary_edge_count: report.boundary_edge_count,
        euler_char: report.euler_characteristic,
        expected_euler: 2, // Default: genus 0 (sphere-like)
    }
}

/// Test watertightness with an expected genus.
pub fn test_watertight_with_genus(mesh: &TriangleMesh, genus: u32) -> WatertightResult {
    let report = check_manifold(mesh);
    let expected_euler = ManifoldReport::expected_euler_for_genus(genus as usize);
    WatertightResult {
        is_watertight: report.is_watertight(),
        boundary_edge_count: report.boundary_edge_count,
        euler_char: report.euler_characteristic,
        expected_euler,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
            Point3d::new(0.0, 0.0, 1.0),
            Point3d::new(1.0, 0.0, 1.0),
            Point3d::new(1.0, 1.0, 1.0),
            Point3d::new(0.0, 1.0, 1.0),
        ];
        for p in &v {
            mesh.add_vertex(*p);
        }
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        mesh.add_triangle(3, 7, 6);
        mesh.add_triangle(3, 6, 2);
        mesh.add_triangle(0, 4, 7);
        mesh.add_triangle(0, 7, 3);
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);
        mesh
    }

    #[test]
    fn test_cube_is_watertight() {
        let mesh = make_cube_mesh();
        let result = test_watertight(&mesh);
        assert!(result.is_watertight, "Cube should be watertight");
        assert_eq!(result.boundary_edge_count, 0, "Cube should have no boundary edges");
        assert_eq!(result.euler_char, 2, "Cube should have Euler characteristic 2");
    }

    #[test]
    fn test_open_mesh_not_watertight() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let result = test_watertight(&mesh);
        assert!(!result.is_watertight, "Single triangle should not be watertight");
        assert_eq!(result.boundary_edge_count, 3);
    }

    #[test]
    fn test_watertight_with_genus() {
        let mesh = make_cube_mesh();
        let result = test_watertight_with_genus(&mesh, 0);
        assert!(result.is_watertight);
        assert_eq!(result.expected_euler, 2);
        assert_eq!(result.euler_char, result.expected_euler);
    }
}
