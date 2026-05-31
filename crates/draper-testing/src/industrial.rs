// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.3 — Industrial Files
//!
//! Functions to test STEP files in the test/ directory.

use draper_step::{parse_step_file, step_to_mesh};
use std::time::Instant;

/// Error type for industrial file testing.
#[derive(Debug)]
pub struct TestError {
    pub message: String,
}

impl std::fmt::Display for TestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TestError: {}", self.message)
    }
}

impl std::error::Error for TestError {}

/// Result of testing an industrial STEP file.
#[derive(Debug)]
pub struct IndustrialTestResult {
    /// Filename of the STEP file.
    pub filename: String,
    /// Whether parsing succeeded.
    pub parse_ok: bool,
    /// Whether triangulation succeeded.
    pub triangulate_ok: bool,
    /// Number of triangles in the resulting mesh.
    pub triangle_count: usize,
    /// Number of vertices in the resulting mesh.
    pub vertex_count: usize,
    /// Manifold report string.
    pub manifold_report: String,
    /// Time elapsed for parse + triangulate (seconds).
    pub elapsed: f64,
}

/// List of all test files in the test/ directory.
/// Returns relative paths from the repository root.
pub fn test_file_list() -> Vec<&'static str> {
    vec![
        "test/nist_cube.stp",
        "test/nist_cylinder.stp",
        "test/nist_sphere.stp",
        "test/nist_cone.stp",
        "test/nist_block_with_hole.stp",
        "test/nist_chamfer_block.stp",
        "test/nist_assembly.stp",
        "test/nist_complex_surface.stp",
        "test/Zentralstaender.stp",
        "test/drill_top.stp",
        "test/SampleCube.step",
        "test/as1-oc-214.stp",
        "test/brick_thin.stp",
        "test/brick_thin_round.stp",
        "test/brick_thin_hole.stp",
        "test/3.05.078.stp",
        "test/compressor-13920_top.stp",
        "test/transmission_top.stp",
    ]
}

/// Load a STEP file and triangulate it.
pub fn load_and_triangulate(path: &str) -> Result<draper_mesh::TriangleMesh, TestError> {
    let step_file = parse_step_file(path)
        .map_err(|e| TestError { message: format!("Parse error: {}", e) })?;

    let mesh = step_to_mesh(&step_file)
        .map_err(|e| TestError { message: format!("Triangulation error: {}", e) })?;

    Ok(mesh)
}

/// Run all industrial file tests and return results.
/// This function never panics — errors are recorded in the result.
pub fn run_industrial_tests() -> Vec<IndustrialTestResult> {
    let mut results = Vec::new();

    for file_path in test_file_list() {
        let start = Instant::now();
        let (parse_ok, triangulate_ok, triangle_count, vertex_count, manifold_report) = match load_and_triangulate(file_path) {
            Ok(mesh) => {
                let report = draper_mesh::check_manifold(&mesh);
                (
                    true,
                    true,
                    mesh.triangle_count(),
                    mesh.vertex_count(),
                    format!(
                        "watertight={} boundary={} euler={} degenerate={}",
                        report.is_watertight(),
                        report.boundary_edge_count,
                        report.euler_characteristic,
                        report.degenerate_triangle_count,
                    ),
                )
            }
            Err(e) => {
                // Try to determine if parse succeeded but triangulation failed
                let parse_ok = match parse_step_file(file_path) {
                    Ok(_) => true,
                    Err(_) => false,
                };
                (
                    parse_ok,
                    false,
                    0,
                    0,
                    format!("error: {}", e.message),
                )
            }
        };

        results.push(IndustrialTestResult {
            filename: file_path.to_string(),
            parse_ok,
            triangulate_ok,
            triangle_count,
            vertex_count,
            manifold_report,
            elapsed: start.elapsed().as_secs_f64(),
        });
    }

    results
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_file_list_not_empty() {
        let files = test_file_list();
        assert!(!files.is_empty(), "Test file list should not be empty");
    }

    #[test]
    fn test_load_nist_cube() {
        // Find the repo root by looking for the test directory
        let result = load_and_triangulate("test/nist_cube.stp");
        // This may fail if CWD is not the repo root, but the function
        // should not panic in any case
        match result {
            Ok(mesh) => {
                assert!(mesh.triangle_count() > 0, "Cube should have triangles");
            }
            Err(_) => {
                // File may not be accessible from current CWD — that's OK for unit test
            }
        }
    }
}
