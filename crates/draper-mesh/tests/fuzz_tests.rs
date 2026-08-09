// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! LT-4: Property-based / fuzzing tests for triangulation edge cases.
//!
//! These tests generate random meshes with controlled degeneracies
//! to verify that the triangulation pipeline handles edge cases without
//! panicking and produces valid output.

use draper_mesh::{TriangleMesh, validate_watertight, weld_boundary_edge_vertices,
                  repair_t_junctions, fill_boundary_gaps};
use draper_geometry::Point3d;

/// Simple LCG random number generator (deterministic, no external deps).
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg { state: seed }
    }

    fn next(&mut self) -> u64 {
        // Numerical Recipes LCG
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state >> 1
    }

    fn next_f64(&mut self, min: f64, max: f64) -> f64 {
        let r = self.next() as f64 / u64::MAX as f64;
        min + r * (max - min)
    }

    fn next_usize(&mut self, min: usize, max: usize) -> usize {
        min + (self.next() as usize) % (max - min + 1)
    }
}

/// Generate a random point within a bounded box.
fn random_point(lcg: &mut Lcg) -> Point3d {
    Point3d::new(
        lcg.next_f64(-10.0, 10.0),
        lcg.next_f64(-10.0, 10.0),
        lcg.next_f64(-10.0, 10.0),
    )
}

/// Generate a random triangle mesh with controlled properties.
fn generate_random_mesh(lcg: &mut Lcg, n_triangles: usize, allow_degenerate: bool) -> TriangleMesh {
    let n_vertices = n_triangles * 2; // Rough estimate
    let mut mesh = TriangleMesh::new();

    // Generate random vertices
    for _ in 0..n_vertices {
        mesh.add_vertex(random_point(lcg));
    }

    // Generate random triangles
    for _ in 0..n_triangles {
        if allow_degenerate {
            // Sometimes create degenerate triangles (collinear or duplicate vertices)
            let choice = lcg.next_usize(0, 3);
            match choice {
                0 => {
                    // Normal triangle
                    let a = lcg.next_usize(0, n_vertices - 1) as u32;
                    let b = lcg.next_usize(0, n_vertices - 1) as u32;
                    let c = lcg.next_usize(0, n_vertices - 1) as u32;
                    mesh.add_triangle(a, b, c);
                }
                1 => {
                    // Degenerate: two same vertices
                    let a = lcg.next_usize(0, n_vertices - 1) as u32;
                    let b = a;
                    let c = lcg.next_usize(0, n_vertices - 1) as u32;
                    mesh.add_triangle(a, b, c);
                }
                2 => {
                    // Degenerate: all same vertices
                    let a = lcg.next_usize(0, n_vertices - 1) as u32;
                    mesh.add_triangle(a, a, a);
                }
                _ => {
                    // Collinear vertices
                    let a = lcg.next_usize(0, n_vertices - 1) as u32;
                    let b = lcg.next_usize(0, n_vertices - 1) as u32;
                    mesh.add_triangle(a, b, a);
                }
            }
        } else {
            // Only normal triangles with distinct vertices
            let a = lcg.next_usize(0, n_vertices - 1) as u32;
            let mut b = lcg.next_usize(0, n_vertices - 1) as u32;
            while b == a {
                b = lcg.next_usize(0, n_vertices - 1) as u32;
            }
            let mut c = lcg.next_usize(0, n_vertices - 1) as u32;
            while c == a || c == b {
                c = lcg.next_usize(0, n_vertices - 1) as u32;
            }
            mesh.add_triangle(a, b, c);
        }
    }

    mesh
}

/// Property: validate_watertight should never panic on any mesh.
#[test]
fn test_fuzz_validate_watertight_no_panic() {
    let mut lcg = Lcg::new(42);

    for _ in 0..100 {
        let mesh = generate_random_mesh(&mut lcg, 20, true);
        // Should not panic
        let _report = validate_watertight(&mesh, false);
    }
}

/// Property: weld_boundary_edge_vertices should never panic.
#[test]
fn test_fuzz_weld_no_panic() {
    let mut lcg = Lcg::new(123);

    for _ in 0..50 {
        let mut mesh = generate_random_mesh(&mut lcg, 15, true);
        // Should not panic
        weld_boundary_edge_vertices(&mut mesh, 0.01);
    }
}

/// Property: repair_t_junctions should never panic.
#[test]
fn test_fuzz_repair_t_junctions_no_panic() {
    let mut lcg = Lcg::new(456);

    for _ in 0..50 {
        let mut mesh = generate_random_mesh(&mut lcg, 15, false);
        // Should not panic
        let _n = repair_t_junctions(&mut mesh, 1e-6);
    }
}

/// Property: fill_boundary_gaps should never panic.
#[test]
fn test_fuzz_fill_boundary_gaps_no_panic() {
    let mut lcg = Lcg::new(789);

    for _ in 0..50 {
        let mut mesh = generate_random_mesh(&mut lcg, 15, false);
        // Should not panic
        let _n = fill_boundary_gaps(&mut mesh, 32);
    }
}

/// Property: repair_t_junctions + fill_boundary_gaps should not create
/// more boundary edges than the input had (no regression).
#[test]
fn test_fuzz_no_boundary_edge_regression() {
    let mut lcg = Lcg::new(101112);

    for _ in 0..30 {
        let mut mesh = generate_random_mesh(&mut lcg, 10, false);
        let report_before = validate_watertight(&mesh, false);
        let boundary_before = report_before.boundary_edge_count;

        repair_t_junctions(&mut mesh, 1e-6);
        fill_boundary_gaps(&mut mesh, 32);

        let report_after = validate_watertight(&mesh, false);
        let boundary_after = report_after.boundary_edge_count;

        // Post-processing should not INCREASE boundary edges significantly.
        // Allow small increases due to T-junction splits creating new edges.
        assert!(
            boundary_after <= boundary_before + 20,
            "Boundary edges increased from {} to {} — possible regression",
            boundary_before, boundary_after,
        );
    }
}

/// Property: empty mesh should be handled gracefully by all functions.
#[test]
fn test_fuzz_empty_mesh() {
    let mut mesh = TriangleMesh::new();

    // None of these should panic
    let _ = validate_watertight(&mesh, false);
    weld_boundary_edge_vertices(&mut mesh, 0.01);
    let _ = repair_t_junctions(&mut mesh, 1e-6);
    let _ = fill_boundary_gaps(&mut mesh, 32);
}

/// Property: single triangle mesh should be handled gracefully.
#[test]
fn test_fuzz_single_triangle() {
    let mut mesh = TriangleMesh::new();
    mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
    mesh.add_triangle(0, 1, 2);

    // Should not panic, should report 3 boundary edges (open mesh)
    let report = validate_watertight(&mesh, false);
    assert_eq!(report.boundary_edge_count, 3);

    // fill_boundary_gaps should fill the single triangle loop
    let n = fill_boundary_gaps(&mut mesh, 32);
    assert!(n >= 1, "Should fill the boundary loop");
}

/// Property: mesh with NaN coordinates should not panic.
#[test]
fn test_fuzz_nan_coordinates() {
    let mut mesh = TriangleMesh::new();
    mesh.add_vertex(Point3d::new(f64::NAN, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(1.0, f64::NAN, 0.0));
    mesh.add_vertex(Point3d::new(0.5, 1.0, f64::NAN));
    mesh.add_triangle(0, 1, 2);

    // Should not panic
    let _ = validate_watertight(&mesh, false);
    let _ = repair_t_junctions(&mut mesh, 1e-6);
    let _ = fill_boundary_gaps(&mut mesh, 32);
}

/// Property: mesh with Inf coordinates should not panic.
#[test]
fn test_fuzz_inf_coordinates() {
    let mut mesh = TriangleMesh::new();
    mesh.add_vertex(Point3d::new(f64::INFINITY, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(1.0, f64::NEG_INFINITY, 0.0));
    mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
    mesh.add_triangle(0, 1, 2);

    // Should not panic
    let _ = validate_watertight(&mesh, false);
    let _ = repair_t_junctions(&mut mesh, 1e-6);
}

/// Property: mesh with very large coordinates should not panic.
#[test]
fn test_fuzz_large_coordinates() {
    let mut mesh = TriangleMesh::new();
    mesh.add_vertex(Point3d::new(1e15, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(-1e15, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(0.0, 1e15, 0.0));
    mesh.add_triangle(0, 1, 2);

    // Should not panic
    let _ = validate_watertight(&mesh, false);
    let _ = repair_t_junctions(&mut mesh, 1e-6);
}

/// Property: mesh with very small coordinates (near zero) should not panic.
#[test]
fn test_fuzz_tiny_coordinates() {
    let mut mesh = TriangleMesh::new();
    mesh.add_vertex(Point3d::new(1e-15, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(2e-15, 0.0, 0.0));
    mesh.add_vertex(Point3d::new(1.5e-15, 1e-15, 0.0));
    mesh.add_triangle(0, 1, 2);

    // Should not panic
    let _ = validate_watertight(&mesh, false);
    let _ = repair_t_junctions(&mut mesh, 1e-20);
}

/// Stress test: large random mesh should complete within reasonable time.
#[test]
fn test_fuzz_stress_large_mesh() {
    let mut lcg = Lcg::new(999);
    let mesh = generate_random_mesh(&mut lcg, 500, false);

    let start = std::time::Instant::now();
    let _ = validate_watertight(&mesh, false);
    let elapsed = start.elapsed();

    // Should complete in under 1 second for 500 triangles
    assert!(
        elapsed.as_secs() < 2,
        "validate_watertight took too long: {:?}",
        elapsed,
    );
}
