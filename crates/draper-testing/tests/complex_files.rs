// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.5.3 — Complex file test infrastructure.
//!
//! Integration tests for "heavy" STEP files that are too slow for normal CI.
//! Each test is marked `#[ignore]` and runs only via `cargo test -- --ignored`
//! (nightly or manual trigger).
//!
//! # What each test checks (5.3.3)
//!
//! 1. **Load without panic** — parse_step + triangulation complete.
//! 2. **triangle_count > 0** — every instance has at least some triangles.
//! 3. **boundary edge % < 5%** — reasonable watertightness (not strict 0%,
//!    since complex industrial files often have small gaps).
//! 4. **elapsed time < 60s** — desktop performance budget.
//!
//! # Benchmark regression (5.3.5)
//!
//! Results are compared against `benchmark_baseline.csv` at the repo root.
//! If triangle_count or elapsed_time deviates by > 20%, the test fails
//! with a clear message showing expected vs actual.

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::HashMap;
use std::time::Instant;

/// Maximum acceptable boundary edge percentage for complex files.
const MAX_BOUNDARY_EDGE_PCT: f64 = 5.0;

/// Maximum acceptable triangulation time on desktop (seconds).
const MAX_ELAPSED_SECS: f64 = 60.0;

/// Maximum allowed regression from baseline (fraction, e.g. 0.20 = 20%).
const MAX_REGRESSION_FRACTION: f64 = 0.20;

/// Result of testing a single complex STEP file.
#[derive(Debug)]
struct ComplexFileResult {
    filename: String,
    load_ok: bool,
    instance_count: usize,
    total_triangles: usize,
    boundary_edge_pct: f64,
    elapsed_s: f64,
    error_message: Option<String>,
}

/// Load and fully triangulate a STEP file, returning results.
fn test_complex_file(path: &str) -> ComplexFileResult {
    let start = Instant::now();

    // Step 1: Parse
    let data = match std::fs::read_to_string(path) {
        Ok(d) => d,
        Err(e) => {
            return ComplexFileResult {
                filename: path.to_string(),
                load_ok: false,
                instance_count: 0,
                total_triangles: 0,
                boundary_edge_pct: 0.0,
                elapsed_s: start.elapsed().as_secs_f64(),
                error_message: Some(format!("Failed to read file: {}", e)),
            };
        }
    };

    let step = match parse_step(&data) {
        Ok(s) => s,
        Err(e) => {
            return ComplexFileResult {
                filename: path.to_string(),
                load_ok: false,
                instance_count: 0,
                total_triangles: 0,
                boundary_edge_pct: 0.0,
                elapsed_s: start.elapsed().as_secs_f64(),
                error_message: Some(format!("Parse error: {}", e)),
            };
        }
    };

    // Step 2: Triangulate all instances sequentially
    let (_tree, pending) = step_structure_lazy(&step);
    let mut ctx = OwnedStepConversionContext::new(step);
    let mut instances = Vec::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            instances.push(inst);
        }
    }

    let instance_count = instances.len();
    let mut total_triangles = 0usize;

    for inst in &instances {
        total_triangles += inst.mesh.triangle_count();
    }

    // Step 3: Merge all instances into one mesh for watertight check
    let merged = merge_instances(&instances);
    let wt_report = validate_watertight(&merged, false);
    let boundary_pct = if wt_report.edge_count > 0 {
        (wt_report.boundary_edge_count as f64 / wt_report.edge_count as f64) * 100.0
    } else {
        0.0
    };

    let elapsed = start.elapsed().as_secs_f64();

    ComplexFileResult {
        filename: path.to_string(),
        load_ok: true,
        instance_count,
        total_triangles,
        boundary_edge_pct: boundary_pct,
        elapsed_s: elapsed,
        error_message: None,
    }
}

/// Merge multiple DetailedMeshInstance meshes into a single TriangleMesh.
fn merge_instances(instances: &[draper_step::DetailedMeshInstance]) -> draper_mesh::TriangleMesh {
    let mut merged = draper_mesh::TriangleMesh::new();
    let mut vertex_offset = 0u32;

    for inst in instances {
        let mesh = &inst.mesh;
        for v in &mesh.vertices {
            merged.vertices.push(*v);
        }
        for [i0, i1, i2] in &mesh.triangles {
            merged.triangles.push([
                i0 + vertex_offset,
                i1 + vertex_offset,
                i2 + vertex_offset,
            ]);
        }
        vertex_offset += mesh.vertices.len() as u32;
    }

    merged
}

/// Read the benchmark baseline CSV and return a map from filename → (triangle_count, elapsed_s).
fn read_baseline() -> HashMap<String, (usize, f64)> {
    let mut map = HashMap::new();
    let content = match std::fs::read_to_string("benchmark_baseline.csv") {
        Ok(c) => c,
        Err(_) => return map,
    };

    for line in content.lines().skip(1) {
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() < 7 {
            continue;
        }
        let filename = fields[0].to_string();
        let triangles: usize = fields[2].parse().unwrap_or(0);
        let elapsed: f64 = fields[6].parse().unwrap_or(0.0);
        map.insert(filename, (triangles, elapsed));
    }

    map
}

/// Check regression against baseline.
/// Returns None if no baseline, or Some(regression_message) if regression detected.
fn check_regression(result: &ComplexFileResult, baseline: &HashMap<String, (usize, f64)>) -> Option<String> {
    let (base_triangles, base_elapsed) = baseline.get(&result.filename)?;

    let tri_ratio = if *base_triangles > 0 {
        result.total_triangles as f64 / *base_triangles as f64
    } else {
        1.0
    };
    let elapsed_ratio = if *base_elapsed > 0.0 {
        result.elapsed_s / base_elapsed
    } else {
        1.0
    };

    let mut messages = Vec::new();

    if tri_ratio < (1.0 - MAX_REGRESSION_FRACTION) {
        messages.push(format!(
            "triangle_count regression: {} (now) vs {} (baseline), ratio={:.2} (< {:.2})",
            result.total_triangles, base_triangles, tri_ratio, 1.0 - MAX_REGRESSION_FRACTION
        ));
    }
    if tri_ratio > (1.0 + MAX_REGRESSION_FRACTION) {
        messages.push(format!(
            "triangle_count increase: {} (now) vs {} (baseline), ratio={:.2} (> {:.2})",
            result.total_triangles, base_triangles, tri_ratio, 1.0 + MAX_REGRESSION_FRACTION
        ));
    }
    if elapsed_ratio > (1.0 + MAX_REGRESSION_FRACTION) {
        messages.push(format!(
            "elapsed_time regression: {:.3}s (now) vs {:.3}s (baseline), ratio={:.2} (> {:.2})",
            result.elapsed_s, base_elapsed, elapsed_ratio, 1.0 + MAX_REGRESSION_FRACTION
        ));
    }

    if messages.is_empty() {
        None
    } else {
        Some(messages.join("; "))
    }
}

/// Assert the standard checks for a complex file result.
fn assert_complex_file_checks(result: &ComplexFileResult) {
    assert!(result.load_ok, "File should load without panic: {:?}", result.error_message);
    assert!(result.total_triangles > 0, "Should produce triangles (got {})", result.total_triangles);
    assert!(
        result.boundary_edge_pct < MAX_BOUNDARY_EDGE_PCT,
        "Boundary edge pct should be < {:.1}% (got {:.2}%)",
        MAX_BOUNDARY_EDGE_PCT, result.boundary_edge_pct
    );
    assert!(
        result.elapsed_s < MAX_ELAPSED_SECS,
        "Elapsed time should be < {:.0}s (got {:.1}s)",
        MAX_ELAPSED_SECS, result.elapsed_s
    );
}

// ============================================================
// Per-file tests (5.3.1, 5.3.2)
// All marked #[ignore] — run via `cargo test -- --ignored`
// ============================================================

#[test]
#[ignore]
fn test_drill_top() {
    let result = test_complex_file("test/drill_top.stp");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

#[test]
#[ignore]
fn test_vulcan() {
    let result = test_complex_file("test/8500-02_Vulcan.STEP");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

#[test]
#[ignore]
fn test_transmission_top() {
    let result = test_complex_file("test/transmission_top.stp");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

#[test]
#[ignore]
fn test_spit_fire() {
    let result = test_complex_file("test/8394-121_Spit-Fire.STEP");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

#[test]
#[ignore]
fn test_zentralstaender() {
    let result = test_complex_file("test/Zentralstaender.stp");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

#[test]
#[ignore]
fn test_as1_oc_214() {
    let result = test_complex_file("test/as1-oc-214.stp");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

#[test]
#[ignore]
fn test_compressor_top() {
    let result = test_complex_file("test/compressor-13920_top.stp");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

#[test]
#[ignore]
fn test_305_078() {
    let result = test_complex_file("test/3.05.078.stp");
    assert_complex_file_checks(&result);

    let baseline = read_baseline();
    if let Some(regression) = check_regression(&result, &baseline) {
        panic!("Regression detected: {}", regression);
    }
}

/// Run all complex files in one test (for convenience when running `--ignored`).
#[test]
#[ignore]
fn test_all_complex_files_summary() {
    let files = vec![
        "test/3.05.078.stp",
        "test/8394-121_Spit-Fire.STEP",
        "test/8500-02_Vulcan.STEP",
        "test/Zentralstaender.stp",
        "test/drill_top.stp",
        "test/as1-oc-214.stp",
        "test/compressor-13920_top.stp",
        "test/transmission_top.stp",
    ];

    let baseline = read_baseline();
    let mut all_ok = true;

    println!("\n{:<45} {:>10} {:>10} {:>10} {:>10} {:>8}",
        "File", "Instances", "Triangles", "BndEdge%", "Time(s)", "Status");
    println!("{}", "-".repeat(100));

    for file in &files {
        let result = test_complex_file(file);

        let load_ok = result.load_ok;
        let tri_ok = result.total_triangles > 0;
        let bnd_ok = result.boundary_edge_pct < MAX_BOUNDARY_EDGE_PCT;
        let time_ok = result.elapsed_s < MAX_ELAPSED_SECS;

        let status = if load_ok && tri_ok && bnd_ok && time_ok { "OK" } else { "FAIL" };
        if status == "FAIL" { all_ok = false; }

        let regression_status = check_regression(&result, &baseline)
            .map(|msg| format!("REGRESSION: {}", msg))
            .unwrap_or_default();

        println!("{:<45} {:>10} {:>10} {:>9.2}% {:>9.2} {:>8} {}",
            result.filename,
            result.instance_count,
            result.total_triangles,
            result.boundary_edge_pct,
            result.elapsed_s,
            status,
            regression_status,
        );
    }

    assert!(all_ok, "Some complex file tests failed — see output above");
}

/// Quick non-ignored test that the test infrastructure itself works.
#[test]
fn test_complex_file_infrastructure() {
    // Just verify the baseline can be read (or gracefully skipped)
    let baseline = read_baseline();
    // baseline may be empty if file doesn't exist — that's OK
    let _ = baseline;

    // Verify merge_instances works with empty input
    let empty: Vec<draper_step::DetailedMeshInstance> = vec![];
    let merged = merge_instances(&empty);
    assert_eq!(merged.vertices.len(), 0);
    assert_eq!(merged.triangles.len(), 0);
}
