// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Integration tests for STEP file loading and manifold validation.
//!
//! Tests:
//! 1. Zentralstaender.stp loads without error and produces a valid mesh
//! 2. ManifoldChecker reports reasonable results for industrial STEP files
//! 3. StepEdgeCache produces consistent boundary points on shared edges

use draper_step::{parse_step, step_to_detailed_instances, step_to_mesh, step_structure_lazy, StepConversionContext, OwnedStepConversionContext};
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
    let _params = TriangulationParams::default();
    
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
    let _params = TriangulationParams {
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

// ============================================================
// Test as1-oc-214.stp (assembly file that previously hung)
// ============================================================

#[test]
fn test_as1_oc_214_loads() {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let step = parse_test_step("as1-oc-214.stp");
    
    // Test lazy loading (the path used by the web viewer)
    let start = std::time::Instant::now();
    let (_tree, pending) = step_structure_lazy(&step);
    let lazy_time = start.elapsed();
    println!("as1-oc-214.stp: {} pending instances (lazy: {:?})", pending.len(), lazy_time);
    
    assert!(!pending.is_empty(), "as1-oc-214.stp should have at least one BREP instance");
    
    // Test progressive triangulation
    let ctx = StepConversionContext::new(&step);
    let mut total_vertices = 0;
    let mut total_triangles = 0;
    let mut ok_count = 0;
    let mut fail_count = 0;
    
    for (i, p) in pending.iter().enumerate() {
        match ctx.triangulate_pending(p) {
            Some(inst) => {
                total_vertices += inst.mesh.vertex_count();
                total_triangles += inst.mesh.triangle_count();
                ok_count += 1;
                if i < 3 || i == pending.len() - 1 {
                    println!("  [{}] {}: v={} t={}", i, inst.name, inst.mesh.vertex_count(), inst.mesh.triangle_count());
                }
            }
            None => {
                fail_count += 1;
            }
        }
    }
    
    println!("as1-oc-214.stp: {} ok, {} fail, total v={}, t={}", ok_count, fail_count, total_vertices, total_triangles);
    assert!(ok_count > 0, "as1-oc-214.stp should produce at least one mesh");
    assert!(total_triangles > 0, "as1-oc-214.stp should produce triangles");
}

// ============================================================
// Test 3.05.078.stp — rotational part with multiple cylinders + cones
// sharing the X axis. This file has HALF-cylinder and HALF-cone lateral
// faces (each covering π of angular range, with 2 half-circle arc edges
// + 2 seam line edges).
//
// Before the partial-tube-face fix, these faces fell through to earcutr
// which produced TWISTED triangles because the UV polygon self-intersects
// when the top arc is parameterized to go around the "long way" (from u=0
// through u=3π/2 to u=π, instead of the "short way" through u=π/2).
//
// After the fix, these faces are detected as partial-tube faces and
// triangulated using a grid that uses cached arc points (shared with cap
// faces) as the u-row grid points at v_min and v_max, with NO wrap-around
// in u direction.
// ============================================================

#[test]
fn test_3_05_078_loads_and_watertight() {
    let _ = env_logger::builder().is_test(true).try_init();

    let step = parse_test_step("3.05.078.stp");
    // Use LOD 1.0 (full quality) — same as the production viewer's default.
    // step_to_mesh uses TriangulationParams::default() which is much coarser
    // and produces different triangle counts.
    let (_tree, pending) = step_structure_lazy(&step);
    let mut ctx = OwnedStepConversionContext::new_with_lod(step, 1.0);

    let mut mesh = draper_mesh::TriangleMesh::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            mesh.merge(&inst.mesh);
        }
    }

    assert!(mesh.vertex_count() > 0, "3.05.078.stp produced no vertices");
    assert!(mesh.triangle_count() > 100, "3.05.078.stp produced too few triangles: {}", mesh.triangle_count());

    // Check for NaN/Inf vertices
    for v in &mesh.vertices {
        assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
            "3.05.078.stp produced NaN/Inf vertex: {:?}", v);
    }

    let report = check_manifold(&mesh);
    println!("3.05.078.stp: v={} t={} euler={} boundary={} non_manifold={} degenerate={} watertight={}",
        mesh.vertex_count(), mesh.triangle_count(),
        report.euler_characteristic, report.boundary_edge_count,
        report.non_manifold_edge_count, report.degenerate_triangle_count,
        report.is_watertight());

    // For a closed solid (rotational part), we expect:
    // - Very few boundary edges (< 5% of total edges) — partial-tube faces
    //   share vertices with cap faces via cached edge points.
    // - No degenerate triangles (the partial-tube grid produces clean quads).
    // - Reasonable triangle count.
    let total_edges = report.edge_count;
    let boundary_pct = if total_edges > 0 {
        100.0 * report.boundary_edge_count as f64 / total_edges as f64
    } else {
        0.0
    };
    assert!(boundary_pct < 5.0,
        "3.05.078.stp has too many boundary edges: {} ({:.2}%) — partial-tube fix may have regressed",
        report.boundary_edge_count, boundary_pct);
    assert_eq!(report.degenerate_triangle_count, 0,
        "3.05.078.stp has {} degenerate triangles — partial-tube grid should produce clean quads",
        report.degenerate_triangle_count);
}

#[test]
fn test_3_05_078_no_twisted_triangles() {
    let _ = env_logger::builder().is_test(true).try_init();

    let step = parse_test_step("3.05.078.stp");
    let (_tree, pending) = step_structure_lazy(&step);
    let mut ctx = OwnedStepConversionContext::new_with_lod(step, 1.0);

    let mut mesh = draper_mesh::TriangleMesh::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            mesh.merge(&inst.mesh);
        }
    }

    // Detect "twisted" triangles on cylinder/cone lateral faces.
    //
    // A twisted triangle spans a large angular range on a cylinder/cone
    // surface (e.g., connects u=0 to u=π directly, skipping u=π/2). Such
    // triangles are visually wrong (looking like spikes/lobes).
    //
    // Detection: for each triangle, find the best-fit cylinder (by checking
    // if the 3 vertices are equidistant from the X axis — which is the
    // shared axis of all cylinders/cones in this rotational part). If they
    // are, compute their u angles and check if the triangle's angular span
    // exceeds 2π/n_u * 3 (3x the expected uniform grid gap). Such triangles
    // are likely twisted.
    //
    // Note: we use a tighter r_dev threshold (1% instead of 5%) to exclude
    // cap face triangles that happen to have all 3 vertices near the outer
    // rim. Those triangles are NOT lateral face triangles and would produce
    // false positives.
    let mut twisted_count = 0usize;
    let mut total_cyl_triangles = 0usize;
    let mut max_angular_span = 0.0_f64;
    let mut twisted_examples: Vec<(usize, f64, f64, [f64; 3])> = Vec::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        // Distance from each vertex to the X axis (sqrt(y² + z²))
        let r0 = (v0.y * v0.y + v0.z * v0.z).sqrt();
        let r1 = (v1.y * v1.y + v1.z * v1.z).sqrt();
        let r2 = (v2.y * v2.y + v2.z * v2.z).sqrt();

        // Tighter threshold: 1% deviation from mean radius.
        // This excludes cap face triangles (which span radially) while
        // including lateral face triangles (which have constant radius
        // modulo floating-point rounding).
        let r_mean = (r0 + r1 + r2) / 3.0;
        if r_mean < 1.0 { continue; } // skip tiny triangles near the axis
        let r_dev = ((r0 - r_mean).abs() + (r1 - r_mean).abs() + (r2 - r_mean).abs()) / 3.0;
        if r_dev > r_mean * 0.01 { continue; } // not on a cylinder surface (too much radial variation)

        total_cyl_triangles += 1;

        // Compute u angles (around X axis): u = atan2(z, y)
        let u0 = v0.z.atan2(v0.y).rem_euclid(2.0 * std::f64::consts::PI);
        let u1 = v1.z.atan2(v1.y).rem_euclid(2.0 * std::f64::consts::PI);
        let u2 = v2.z.atan2(v2.y).rem_euclid(2.0 * std::f64::consts::PI);

        // Angular span = max pairwise angular distance (accounting for periodicity)
        let ang_dist = |a: f64, b: f64| {
            let d = (a - b).abs();
            if d > std::f64::consts::PI { 2.0 * std::f64::consts::PI - d } else { d }
        };
        let span = ang_dist(u0, u1).max(ang_dist(u1, u2)).max(ang_dist(u0, u2));
        if span > max_angular_span { max_angular_span = span; }

        // A "twisted" triangle spans more than π/2 (90°) of angular range.
        // Normal tube grid triangles span ~2π/n_u, which is typically < π/4.
        if span > std::f64::consts::PI / 2.0 {
            twisted_count += 1;
            if twisted_examples.len() < 5 {
                twisted_examples.push((tri_idx, r_mean, span, [v0.x, v1.x, v2.x]));
            }
        }
    }

    println!("3.05.078.stp twisted-triangle check: {} cylinder/cone triangles, {} twisted, max_span={:.1}°",
        total_cyl_triangles, twisted_count, max_angular_span.to_degrees());
    for (i, (idx, r, span, xs)) in twisted_examples.iter().enumerate() {
        println!("  twisted #{}: tri_idx={}, r={:.2}, span={:.1}°, x=[{:.2}, {:.2}, {:.2}]",
            i, idx, r, span.to_degrees(), xs[0], xs[1], xs[2]);
    }

    // Allow up to 2% twisted triangles (for numerical edge cases at seams
    // and cap-face artifacts that pass the 1% radius threshold).
    let max_twisted = (total_cyl_triangles / 50).max(10);
    assert!(twisted_count <= max_twisted,
        "3.05.078.stp has too many twisted triangles: {} (max allowed: {} of {} cyl/cone triangles)",
        twisted_count, max_twisted, total_cyl_triangles);
}
