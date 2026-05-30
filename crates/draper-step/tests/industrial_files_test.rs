// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Industrial STEP files test — scans the `test/` directory for .stp/.step files
//! and attempts to parse and triangulate each one.
//!
//! This test:
//! - Scans the `test/` directory for .stp and .step files
//! - Attempts to parse and triangulate each file
//! - Reports: filename, parse result, triangle count, manifold status, warnings
//! - Does NOT panic on failure — just reports
//!
//! This partially covers roadmap task 3.3.11 (Testing on industrial file set).

use draper_step::{parse_step, step_to_mesh};
use draper_mesh::check_manifold;

/// Result of testing a single STEP file.
#[derive(Debug)]
struct FileTestResult {
    filename: String,
    parse_ok: bool,
    parse_error: Option<String>,
    triangle_count: usize,
    vertex_count: usize,
    euler_characteristic: i64,
    boundary_edge_count: usize,
    non_manifold_edge_count: usize,
    degenerate_triangle_count: usize,
    is_watertight: bool,
    nan_vertex_count: usize,
    mesh_error: Option<String>,
}

impl FileTestResult {
    fn parse_failed(filename: &str, error: String) -> Self {
        Self {
            filename: filename.to_string(),
            parse_ok: false,
            parse_error: Some(error),
            triangle_count: 0,
            vertex_count: 0,
            euler_characteristic: 0,
            boundary_edge_count: 0,
            non_manifold_edge_count: 0,
            degenerate_triangle_count: 0,
            is_watertight: false,
            nan_vertex_count: 0,
            mesh_error: None,
        }
    }

    fn mesh_failed(filename: &str, error: String) -> Self {
        Self {
            filename: filename.to_string(),
            parse_ok: true,
            parse_error: None,
            triangle_count: 0,
            vertex_count: 0,
            euler_characteristic: 0,
            boundary_edge_count: 0,
            non_manifold_edge_count: 0,
            degenerate_triangle_count: 0,
            is_watertight: false,
            nan_vertex_count: 0,
            mesh_error: Some(error),
        }
    }
}

/// Test a single STEP file and return the result.
fn test_step_file(filename: &str, content: &str) -> FileTestResult {
    // Step 1: Parse
    let step = match parse_step(content) {
        Ok(s) => s,
        Err(e) => return FileTestResult::parse_failed(filename, format!("{}", e)),
    };

    // Step 2: Convert to mesh
    let mesh = match step_to_mesh(&step) {
        Ok(m) => m,
        Err(e) => return FileTestResult::mesh_failed(filename, e),
    };

    // Step 3: Check for NaN/Inf vertices
    let nan_count = mesh.vertices.iter()
        .filter(|v| !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite())
        .count();

    // Step 4: Manifold check
    let report = check_manifold(&mesh);

    FileTestResult {
        filename: filename.to_string(),
        parse_ok: true,
        parse_error: None,
        triangle_count: mesh.triangle_count(),
        vertex_count: mesh.vertex_count(),
        euler_characteristic: report.euler_characteristic,
        boundary_edge_count: report.boundary_edge_count,
        non_manifold_edge_count: report.non_manifold_edge_count,
        degenerate_triangle_count: report.degenerate_triangle_count,
        is_watertight: report.is_watertight(),
        nan_vertex_count: nan_count,
        mesh_error: None,
    }
}

/// Scan the test directory for STEP files and return their names.
/// Optionally skip files larger than `max_bytes` to avoid very slow triangulation.
fn scan_step_files_with_limit(max_bytes: Option<u64>) -> Vec<String> {
    let test_dir = std::path::Path::new("../../test");
    if !test_dir.exists() {
        return vec![];
    }

    let mut files: Vec<String> = Vec::new();
    if let Ok(entries) = std::fs::read_dir(test_dir) {
        for entry in entries.flatten() {
            if let Some(name) = entry.file_name().to_str() {
                let name_lower = name.to_lowercase();
                if name_lower.ends_with(".stp") || name_lower.ends_with(".step") {
                    // Check file size if limit is specified
                    if let Some(limit) = max_bytes {
                        if let Ok(metadata) = entry.metadata() {
                            if metadata.len() > limit {
                                continue; // Skip large files
                            }
                        }
                    }
                    files.push(name.to_string());
                }
            }
        }
    }

    files.sort();
    files
}

/// Scan the test directory for STEP files and return their names.
/// For the report test, we skip files larger than 2MB to avoid timeouts.
fn scan_step_files() -> Vec<String> {
    scan_step_files_with_limit(Some(2_000_000))
}

/// Scan ALL step files regardless of size (for parse-only tests).
fn scan_all_step_files() -> Vec<String> {
    scan_step_files_with_limit(None)
}

#[test]
fn test_industrial_files_report() {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let step_files = scan_step_files();
    
    if step_files.is_empty() {
        println!("No STEP files found in test/ directory — skipping industrial test");
        return;
    }
    
    println!("\n=== Industrial STEP Files Test Report ===\n");
    println!("Found {} STEP files in test/ directory (files > 2MB skipped for performance)\n", step_files.len());
    
    // Also count total files (including skipped)
    let all_files = scan_all_step_files();
    if all_files.len() > step_files.len() {
        println!("Note: {} large files skipped (see scan_all_step_files for full list)\n",
            all_files.len() - step_files.len());
    }
    
    let mut results: Vec<FileTestResult> = Vec::new();
    
    for filename in &step_files {
        let path = format!("../../test/{}", filename);
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(e) => {
                println!("  {} — FILE READ ERROR: {}", filename, e);
                results.push(FileTestResult::parse_failed(filename, format!("Read error: {}", e)));
                continue;
            }
        };
        
        let result = test_step_file(filename, &content);
        results.push(result);
    }
    
    // Print detailed report
    println!("{:<35} {:<6} {:<10} {:<10} {:<5} {:<5} {:<5} {:<5} {:<5}",
        "File", "Parse", "Triangles", "Vertices", "χ", "Bnd", "NM", "Degen", "WT");
    println!("{}", "─".repeat(95));
    
    for r in &results {
        let parse_status = if r.parse_ok { "OK" } else { "FAIL" };
        let tri_str = if r.parse_ok && r.mesh_error.is_none() {
            r.triangle_count.to_string()
        } else if r.mesh_error.is_some() {
            "CONV_ERR".to_string()
        } else {
            "-".to_string()
        };
        let v_str = if r.vertex_count > 0 { r.vertex_count.to_string() } else { "-".to_string() };
        let euler_str = if r.vertex_count > 0 { r.euler_characteristic.to_string() } else { "-".to_string() };
        let bnd_str = if r.vertex_count > 0 { r.boundary_edge_count.to_string() } else { "-".to_string() };
        let nm_str = if r.vertex_count > 0 { r.non_manifold_edge_count.to_string() } else { "-".to_string() };
        let degen_str = if r.vertex_count > 0 { r.degenerate_triangle_count.to_string() } else { "-".to_string() };
        let wt_str = if r.vertex_count > 0 {
            if r.is_watertight { "Y" } else { "N" }
        } else { "-" };
        
        println!("{:<35} {:<6} {:<10} {:<10} {:<5} {:<5} {:<5} {:<5} {:<5}",
            r.filename, parse_status, tri_str, v_str, euler_str, bnd_str, nm_str, degen_str, wt_str);
        
        if let Some(ref err) = r.parse_error {
            println!("  ↳ Parse error: {}", err);
        }
        if let Some(ref err) = r.mesh_error {
            println!("  ↳ Conversion error: {}", err);
        }
        if r.nan_vertex_count > 0 {
            println!("  ↳ WARNING: {} NaN/Inf vertices!", r.nan_vertex_count);
        }
    }
    
    println!("{}", "─".repeat(95));
    
    // Summary statistics
    let total = results.len();
    let parse_ok = results.iter().filter(|r| r.parse_ok).count();
    let mesh_ok = results.iter().filter(|r| r.mesh_error.is_none() && r.triangle_count > 0).count();
    let watertight = results.iter().filter(|r| r.is_watertight).count();
    let with_nan = results.iter().filter(|r| r.nan_vertex_count > 0).count();
    let total_triangles: usize = results.iter().map(|r| r.triangle_count).sum();
    
    println!("\nSummary:");
    println!("  Total files:     {}", total);
    println!("  Parse OK:        {}/{} ({:.0}%)", parse_ok, total, if total > 0 { 100.0 * parse_ok as f64 / total as f64 } else { 0.0 });
    println!("  Mesh OK:         {}/{} ({:.0}%)", mesh_ok, total, if total > 0 { 100.0 * mesh_ok as f64 / total as f64 } else { 0.0 });
    println!("  Watertight:      {}/{} ({:.0}%)", watertight, mesh_ok, if mesh_ok > 0 { 100.0 * watertight as f64 / mesh_ok as f64 } else { 0.0 });
    println!("  With NaN/Inf:    {}/{}", with_nan, mesh_ok);
    println!("  Total triangles: {}", total_triangles);
    
    // Don't panic — just report. The test passes as long as we can scan and report.
    // Individual file failures are reported but don't fail the test.
}

#[test]
fn test_all_step_files_parse() {
    let _ = env_logger::builder().is_test(true).try_init();
    
    let step_files = scan_all_step_files();
    
    if step_files.is_empty() {
        println!("No STEP files found — skipping");
        return;
    }
    
    let mut parse_ok = 0;
    let mut parse_fail = 0;
    
    for filename in &step_files {
        let path = format!("../../test/{}", filename);
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(_) => continue,
        };
        
        match parse_step(&content) {
            Ok(step) => {
                parse_ok += 1;
                println!("  {} — parsed {} entities", filename, step.entities.len());
            }
            Err(e) => {
                parse_fail += 1;
                println!("  {} — PARSE ERROR: {}", filename, e);
            }
        }
    }
    
    println!("\nParse results: {}/{} OK", parse_ok, parse_ok + parse_fail);
    
    // At least some files should parse
    assert!(parse_ok > 0, "At least one STEP file should parse successfully");
}
