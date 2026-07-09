// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 7.1 — ABC/NIST Dataset integration test.
//!
//! Automated testing framework for STEP files from public datasets.
//! Currently uses the existing test corpus (26 files in test/ directory).
//! Can be extended with ABC dataset files when they become available.
//!
//! # Usage
//!
//! ```sh
//! # Run dataset tests (fast — only parsing, no triangulation)
//! cargo test -p draper-testing --test abc_dataset
//!
//! # Run full triangulation tests (slow)
//! cargo test -p draper-testing --test abc_dataset -- --ignored
//! ```
//!
//! # Adding files
//!
//! Place STEP files in the `test/` directory. The test discovers them
//! automatically by scanning for `*.stp` and `*.STEP` files.

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::validate_watertight;
use std::path::PathBuf;
use std::time::Instant;

/// Maximum acceptable boundary edge percentage for dataset files.
const MAX_BOUNDARY_EDGE_PCT: f64 = 5.0;

/// Maximum acceptable triangulation time per file (seconds).
const MAX_ELAPSED_SECS: f64 = 60.0;

/// Result of testing a single STEP file from the dataset.
#[derive(Debug)]
struct DatasetFileResult {
    filename: String,
    parse_ok: bool,
    instance_count: usize,
    total_triangles: usize,
    boundary_edge_pct: f64,
    elapsed_s: f64,
    error_message: Option<String>,
}

/// Discover all STEP files in the test directory.
fn discover_step_files() -> Vec<PathBuf> {
    let test_dir = PathBuf::from("../../test");
    let mut files = Vec::new();

    if let Ok(entries) = std::fs::read_dir(&test_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            let ext = path.extension()
                .and_then(|e| e.to_str())
                .map(|e| e.to_lowercase())
                .unwrap_or_default();
            if ext == "stp" || ext == "step" {
                files.push(path);
            }
        }
    }

    files.sort();
    files
}

/// Load and fully test a single STEP file.
fn test_step_file(path: &PathBuf, do_triangulate: bool) -> DatasetFileResult {
    let filename = path.file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("unknown")
        .to_string();

    let start = Instant::now();

    // Step 1: Parse
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => {
            return DatasetFileResult {
                filename,
                parse_ok: false,
                instance_count: 0,
                total_triangles: 0,
                boundary_edge_pct: 0.0,
                elapsed_s: start.elapsed().as_secs_f64(),
                error_message: Some(format!("Read error: {}", e)),
            };
        }
    };

    let step = match parse_step(&content) {
        Ok(s) => s,
        Err(e) => {
            return DatasetFileResult {
                filename,
                parse_ok: false,
                instance_count: 0,
                total_triangles: 0,
                boundary_edge_pct: 0.0,
                elapsed_s: start.elapsed().as_secs_f64(),
                error_message: Some(format!("Parse error: {}", e)),
            };
        }
    };

    if !do_triangulate {
        return DatasetFileResult {
            filename,
            parse_ok: true,
            instance_count: 0,
            total_triangles: 0,
            boundary_edge_pct: 0.0,
            elapsed_s: start.elapsed().as_secs_f64(),
            error_message: None,
        };
    }

    // Step 2: Triangulate
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
    let mut total_boundary_edges = 0usize;
    let mut total_edges = 0usize;

    for inst in &instances {
        total_triangles += inst.mesh.triangle_count();
        let report = validate_watertight(&inst.mesh, false);
        total_boundary_edges += report.boundary_edge_count;
        total_edges += report.edge_count;
    }

    let boundary_pct = if total_edges > 0 {
        (total_boundary_edges as f64 / total_edges as f64) * 100.0
    } else {
        0.0
    };

    DatasetFileResult {
        filename,
        parse_ok: true,
        instance_count,
        total_triangles,
        boundary_edge_pct: boundary_pct,
        elapsed_s: start.elapsed().as_secs_f64(),
        error_message: None,
    }
}

/// Test that ALL STEP files in the test directory can be parsed without errors.
/// This is the fast version (no triangulation).
#[test]
fn test_all_step_files_parse() {
    let files = discover_step_files();
    assert!(!files.is_empty(), "No STEP files found in test/ directory");

    let mut failed = Vec::new();
    let mut succeeded = 0usize;

    for path in &files {
        let result = test_step_file(path, false);
        if result.parse_ok {
            succeeded += 1;
        } else {
            failed.push(format!("{}: {}", result.filename,
                result.error_message.unwrap_or_default()));
        }
    }

    assert!(failed.is_empty(),
        "{} of {} files failed to parse:\n  {}",
        failed.len(), files.len(), failed.join("\n  "));

    println!("✓ All {} STEP files parsed successfully", succeeded);
}

/// Test that ALL STEP files in the test directory can be fully triangulated.
/// This is the slow version — marked #[ignore] for CI.
#[test]
#[ignore]
fn test_all_step_files_triangulate() {
    let files = discover_step_files();
    assert!(!files.is_empty(), "No STEP files found in test/ directory");

    let mut failed = Vec::new();
    let mut succeeded = 0usize;
    let mut total_triangles = 0usize;

    for path in &files {
        let result = test_step_file(path, true);

        if !result.parse_ok {
            failed.push(format!("{}: PARSE FAILED - {}", result.filename,
                result.error_message.unwrap_or_default()));
            continue;
        }

        if result.instance_count == 0 {
            failed.push(format!("{}: 0 instances produced", result.filename));
            continue;
        }

        if result.total_triangles == 0 {
            failed.push(format!("{}: 0 triangles produced", result.filename));
            continue;
        }

        if result.boundary_edge_pct > MAX_BOUNDARY_EDGE_PCT {
            failed.push(format!("{}: {:.2}% boundary edges (max {}%)",
                result.filename, result.boundary_edge_pct, MAX_BOUNDARY_EDGE_PCT));
            continue;
        }

        if result.elapsed_s > MAX_ELAPSED_SECS {
            failed.push(format!("{}: {:.1}s elapsed (max {}s)",
                result.filename, result.elapsed_s, MAX_ELAPSED_SECS));
            continue;
        }

        succeeded += 1;
        total_triangles += result.total_triangles;

        println!("  ✓ {} — {} instances, {} tris, {:.2}% boundary, {:.2}s",
            result.filename, result.instance_count, result.total_triangles,
            result.boundary_edge_pct, result.elapsed_s);
    }

    let success_rate = succeeded as f64 / files.len() as f64 * 100.0;
    println!("\n═══ Dataset Summary ═══");
    println!("  Files: {} total, {} passed, {} failed", files.len(), succeeded, failed.len());
    println!("  Success rate: {:.1}%", success_rate);
    println!("  Total triangles: {}", total_triangles);

    // Target: ≥ 95% success rate
    assert!(success_rate >= 95.0,
        "Success rate {:.1}% < 95% target. Failures:\n  {}",
        success_rate, failed.join("\n  "));
}

/// Generate a summary report for all STEP files.
/// This test always passes — it only produces output.
#[test]
fn test_dataset_summary() {
    let files = discover_step_files();
    println!("\n═══ STEP Dataset Inventory ═══");
    println!("  Total files: {}", files.len());
    for (i, path) in files.iter().enumerate() {
        let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("?");
        let size = std::fs::metadata(path)
            .map(|m| m.len())
            .unwrap_or(0);
        println!("  {:2}. {:40} {:>8} bytes", i + 1, name, size);
    }
}
