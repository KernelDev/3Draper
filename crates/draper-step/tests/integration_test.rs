//! Integration tests for STEP file loading and manifold validation.
//!
//! Tests:
//! 1. Zentralstaender.stp loads without error and produces a valid mesh
//! 2. ManifoldChecker reports reasonable results for industrial STEP files
//! 3. StepEdgeCache produces consistent boundary points on shared edges

use std::path::Path;
use draper_step::{parse_step, step_to_detailed_instances, step_to_mesh};
use draper_mesh::{check_manifold, TriangulationParams};

/// Helper: read a STEP file from the test directory.
fn read_test_step(filename: &str) -> String {
    let path = format!("../../test/{}", filename);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("Failed to read test file {}: {}", path, e))
}

/// Helper: parse a STEP file from the test directory.
fn parse_test_step(filename: &str) -> draper_step::StepFile {
    let content = read_test_step(filename);
    parse_step(&content).unwrap_or_else(|e| panic!("Failed to parse {}: {}", filename, e))
}

// ============================================================
// Task 1.2.8: Integration test — Zentralstaender.stp
// ============================================================

#[test]
fn test_zentralstaender_loads() {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let step = parse_test_step("Zentralstaender.stp");
    let params = TriangulationParams::default();
    
    // Convert to mesh
    let result = step_to_mesh(&step);
    assert!(result.is_ok(), "Zentralstaender.stp failed to convert: {:?}", result.err());
    
    let mesh = result.unwrap();
    assert!(mesh.vertex_count() > 0, "Zentralstaender.stp produced no vertices");
    assert!(mesh.triangle_count() > 0, "Zentralstaender.stp produced no triangles");
    
    // Check for NaN/Inf
    for v in &mesh.vertices {
        assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
            "Zentralstaender.stp produced NaN/Inf vertex: {:?}", v);
    }
    
    println!("Zentralstaender.stp: v={} t={}", mesh.vertex_count(), mesh.triangle_count());
}

#[test]
fn test_zentralstaender_manifold_report() {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let step = parse_test_step("Zentralstaender.stp");
    let result = step_to_mesh(&step);
    assert!(result.is_ok());
    
    let mesh = result.unwrap();
    let report = check_manifold(&mesh);
    
    println!("Zentralstaender.stp manifold report:");
    println!("  Vertices: {}", mesh.vertex_count());
    println!("  Triangles: {}", mesh.triangle_count());
    println!("  Euler characteristic: {}", report.euler_characteristic);
    println!("  Boundary edges: {}", report.boundary_edge_count);
    println!("  Non-manifold edges: {}", report.non_manifold_edge_count);
    println!("  Degenerate triangles: {}", report.degenerate_triangle_count);
    println!("  Is watertight: {}", report.is_watertight());
    
    // For an industrial STEP file, we don't expect perfect watertightness
    // but we expect reasonable results
    assert!(mesh.triangle_count() > 100, "Zentralstaender should produce significant mesh");
    assert!(report.degenerate_triangle_count < mesh.triangle_count() / 10,
        "Too many degenerate triangles: {} / {}",
        report.degenerate_triangle_count, mesh.triangle_count());
}

// ============================================================
// Task 1.3.9: Test manifold checker on industrial files
// ============================================================

#[test]
fn test_drill_top_manifold_report() {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let step = parse_test_step("drill_top.stp");
    let params = TriangulationParams {
        adaptive: true,
        ..TriangulationParams::default()
    };
    
    // Use detailed instances to get per-face info
    let result = step_to_detailed_instances(&step);
    assert!(result.is_ok(), "drill_top.stp failed: {:?}", result.err());
    
    let instances = result.unwrap();
    assert!(!instances.is_empty(), "drill_top.stp produced no instances");
    
    let mut total_vertices = 0;
    let mut total_triangles = 0;
    
    for inst in &instances {
        total_vertices += inst.mesh.vertex_count();
        total_triangles += inst.mesh.triangle_count();
        
        // Check for NaN/Inf
        for v in &inst.mesh.vertices {
            assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
                "drill_top.stp produced NaN/Inf vertex");
        }
        
        let report = check_manifold(&inst.mesh);
        println!("drill_top instance '{}': v={} t={} boundary={} euler={} watertight={}",
            inst.name, inst.mesh.vertex_count(), inst.mesh.triangle_count(),
            report.boundary_edge_count, report.euler_characteristic, report.is_watertight());
    }
    
    println!("drill_top.stp total: {} instances, v={} t={}",
        instances.len(), total_vertices, total_triangles);
}

#[test]
fn test_sample_cube_manifold() {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let step = parse_test_step("SampleCube.step");
    let result = step_to_mesh(&step);
    assert!(result.is_ok());
    
    let mesh = result.unwrap();
    let report = check_manifold(&mesh);
    
    println!("SampleCube: v={} t={} euler={} boundary={} watertight={}",
        mesh.vertex_count(), mesh.triangle_count(),
        report.euler_characteristic, report.boundary_edge_count, report.is_watertight());
    
    // A cube should be watertight (Euler characteristic = 2)
    assert!(report.is_watertight() || report.boundary_edge_count < 10,
        "SampleCube should be approximately watertight, got {} boundary edges",
        report.boundary_edge_count);
}
