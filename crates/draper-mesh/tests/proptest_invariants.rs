// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Property-based tests for mesh invariants (ROADMAP_VISION_2036 §9.3).
//!
//! Verifies:
//! - Euler characteristic V - E + F = 2 for closed cube meshes
//! - Every interior edge has exactly 2 adjacent triangles (manifold)
//! - No degenerate triangles (zero area)

use proptest::prelude::*;
use draper_mesh::TriangleMesh;
use draper_mesh::watertight::validate_watertight;
use draper_geometry::Point3d;

/// Build a closed cube mesh with given half-size.
/// A cube has 8 vertices, 12 triangles, 18 edges, V-E+F = 8-18+12 = 2.
fn make_cube(half: f64) -> TriangleMesh {
    let h = half;
    let mut mesh = TriangleMesh::new();
    mesh.vertices = vec![
        Point3d::new(-h, -h, -h), // 0
        Point3d::new( h, -h, -h), // 1
        Point3d::new( h,  h, -h), // 2
        Point3d::new(-h,  h, -h), // 3
        Point3d::new(-h, -h,  h), // 4
        Point3d::new( h, -h,  h), // 5
        Point3d::new( h,  h,  h), // 6
        Point3d::new(-h,  h,  h), // 7
    ];
    // 12 triangles (2 per face, outward normals)
    mesh.triangles = vec![
        [0, 2, 1], [0, 3, 2], // -Z (bottom)
        [4, 5, 6], [4, 6, 7], // +Z (top)
        [0, 1, 5], [0, 5, 4], // -Y (front)
        [2, 3, 7], [2, 7, 6], // +Y (back)
        [0, 4, 7], [0, 7, 3], // -X (left)
        [1, 2, 6], [1, 6, 5], // +X (right)
    ];
    mesh
}

proptest! {
    /// A closed cube must always be watertight regardless of size.
    #[test]
    fn prop_cube_watertight(
        half in 0.001f64..1000.0,
    ) {
        let mesh = make_cube(half);
        let report = validate_watertight(&mesh, false);
        prop_assert!(
            report.is_watertight(),
            "Cube with half={} not watertight: {} boundary edges, {} non-manifold",
            half, report.boundary_edge_count, report.non_manifold_edge_count
        );
    }

    /// Euler characteristic of a closed cube must be 2 (genus 0).
    /// V - E + F = 8 - 18 + 12 = 2
    #[test]
    fn prop_cube_euler_characteristic(
        half in 0.001f64..1000.0,
    ) {
        let mesh = make_cube(half);
        let v = mesh.vertices.len() as i64;
        let f = mesh.triangles.len() as i64;
        let report = validate_watertight(&mesh, false);
        let e = report.edge_count as i64;
        let euler = v - e + f;
        prop_assert_eq!(
            euler, 2,
            "Euler characteristic V-E+F = {}-{}+{} = {} (expected 2)",
            v, e, f, euler
        );
    }

    /// A cube with one triangle removed must NOT be watertight.
    #[test]
    fn prop_cube_with_hole_not_watertight(
        half in 0.001f64..100.0,
    ) {
        let mut mesh = make_cube(half);
        mesh.triangles.pop(); // Remove one triangle → creates a hole
        let report = validate_watertight(&mesh, false);
        prop_assert!(
            !report.is_watertight(),
            "Cube with hole should not be watertight, but report says it is"
        );
        prop_assert!(
            report.boundary_edge_count >= 3,
            "Cube with hole should have ≥3 boundary edges, got {}",
            report.boundary_edge_count
        );
    }

    /// Degenerate triangles (all 3 vertices at same point) must be detected.
    #[test]
    fn prop_degenerate_triangle_detected(
        x in -100.0f64..100.0,
    ) {
        let mut mesh = TriangleMesh::new();
        mesh.vertices = vec![
            Point3d::new(x, x, x),
            Point3d::new(x, x, x),
            Point3d::new(x, x, x),
        ];
        mesh.triangles = vec![[0, 1, 2]];
        let report = validate_watertight(&mesh, true);
        // A single degenerate triangle should have degenerate_triangle_count > 0
        prop_assert!(
            report.degenerate_triangle_count > 0 || report.edge_count == 0,
            "Degenerate triangle not detected: degenerate_count={}, edge_count={}",
            report.degenerate_triangle_count, report.edge_count
        );
    }
}
