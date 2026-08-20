// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.10 — Golden File Regression Testing (ROADMAP_VISION_2036 §9.1)
//!
//! Provides a baseline comparison system for STEP file triangulation results.
//! The baseline stores expected metrics (vertex count, triangle count, watertightness,
//! Euler characteristic) for each test file. On each test run, actual metrics are
//! compared against the baseline to detect regressions.
//!
//! ## Usage
//!
//! ```ignore
//! use draper_testing::golden::*;
//!
//! // Generate or update the baseline (run once, commit the output)
//! let baseline = generate_baseline();
//! let json = baseline.to_json();
//! std::fs::write("test/golden_baseline.json", &json).unwrap();
//!
//! // Compare actual results against the baseline
//! let baseline = GoldenBaseline::from_json(&std::fs::read_to_string("test/golden_baseline.json").unwrap()).unwrap();
//! let report = compare_to_baseline(&baseline);
//! for regression in &report.regressions {
//!     eprintln!("REGRESSION: {}", regression);
//! }
//! assert!(report.regressions.is_empty(), "Regressions detected!");
//! ```

use crate::industrial::{run_industrial_tests, IndustrialTestResult};
use std::collections::HashMap;

/// Expected metrics for a single test file.
#[derive(Clone, Debug, PartialEq)]
pub struct GoldenEntry {
    /// Filename relative to repo root.
    pub filename: String,
    /// Whether parsing should succeed.
    pub parse_ok: bool,
    /// Whether triangulation should succeed.
    pub triangulate_ok: bool,
    /// Expected vertex count (0 if triangulation fails).
    pub vertex_count: usize,
    /// Expected triangle count (0 if triangulation fails).
    pub triangle_count: usize,
    /// Whether the mesh should be watertight.
    pub watertight: bool,
    /// Expected Euler characteristic (V - E + F).
    pub euler_characteristic: i64,
}

impl GoldenEntry {
    /// Create a golden entry from an industrial test result.
    pub fn from_result(result: &IndustrialTestResult) -> Self {
        let (watertight, euler) = parse_manifold_report(&result.manifold_report);
        Self {
            filename: result.filename.clone(),
            parse_ok: result.parse_ok,
            triangulate_ok: result.triangulate_ok,
            vertex_count: result.vertex_count,
            triangle_count: result.triangle_count,
            watertight,
            euler_characteristic: euler,
        }
    }

    /// Convert to a JSON line for storage.
    pub fn to_json(&self) -> String {
        format!(
            r#"{{"filename":"{}","parse_ok":{},"triangulate_ok":{},"vertex_count":{},"triangle_count":{},"watertight":{},"euler_characteristic":{}}}"#,
            self.filename,
            self.parse_ok,
            self.triangulate_ok,
            self.vertex_count,
            self.triangle_count,
            self.watertight,
            self.euler_characteristic,
        )
    }

    /// Parse from a JSON line.
    pub fn from_json(json: &str) -> Result<Self, String> {
        let json = json.trim();
        if !json.starts_with('{') || !json.ends_with('}') {
            return Err("JSON must start with { and end with }".to_string());
        }
        let inner = &json[1..json.len() - 1];

        let filename = extract_string_field(inner, "filename")?;
        let parse_ok = extract_bool_field(inner, "parse_ok")?;
        let triangulate_ok = extract_bool_field(inner, "triangulate_ok")?;
        let vertex_count = extract_num_field(inner, "vertex_count")? as usize;
        let triangle_count = extract_num_field(inner, "triangle_count")? as usize;
        let watertight = extract_bool_field(inner, "watertight")?;
        let euler_characteristic = extract_num_field(inner, "euler_characteristic")? as i64;

        Ok(Self {
            filename,
            parse_ok,
            triangulate_ok,
            vertex_count,
            triangle_count,
            watertight,
            euler_characteristic,
        })
    }
}

/// A collection of golden file baselines.
#[derive(Clone, Debug, Default)]
pub struct GoldenBaseline {
    /// Map from filename to golden entry.
    pub entries: HashMap<String, GoldenEntry>,
}

impl GoldenBaseline {
    /// Create a new empty baseline.
    pub fn new() -> Self {
        Self::default()
    }

    /// Generate a baseline from current test results.
    /// This runs all industrial tests and records their metrics.
    pub fn generate() -> Self {
        let results = run_industrial_tests();
        let mut baseline = Self::new();
        for result in &results {
            let entry = GoldenEntry::from_result(result);
            baseline.entries.insert(entry.filename.clone(), entry);
        }
        baseline
    }

    /// Serialize the baseline to JSON format.
    pub fn to_json(&self) -> String {
        let mut entries: Vec<&GoldenEntry> = self.entries.values().collect();
        entries.sort_by(|a, b| a.filename.cmp(&b.filename));
        let lines: Vec<String> = entries.iter().map(|e| e.to_json()).collect();
        format!("[\n{}\n]\n", lines.join(",\n"))
    }

    /// Deserialize from JSON format.
    pub fn from_json(json: &str) -> Result<Self, String> {
        let json = json.trim();
        if !json.starts_with('[') || !json.ends_with(']') {
            return Err("JSON must be an array".to_string());
        }
        let inner = &json[1..json.len() - 1];
        let mut baseline = Self::new();

        // Split by lines and parse each entry
        let mut depth = 0;
        let mut start = 0;
        for (i, ch) in inner.char_indices() {
            match ch {
                '{' => {
                    if depth == 0 {
                        start = i;
                    }
                    depth += 1;
                }
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        let entry_json = &inner[start..=i];
                        let entry = GoldenEntry::from_json(entry_json)?;
                        baseline.entries.insert(entry.filename.clone(), entry);
                    }
                }
                _ => {}
            }
        }
        Ok(baseline)
    }

    /// Get an entry by filename.
    pub fn get(&self, filename: &str) -> Option<&GoldenEntry> {
        self.entries.get(filename)
    }

    /// Number of entries in the baseline.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the baseline is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// A single regression finding.
#[derive(Clone, Debug)]
pub struct Regression {
    /// Filename of the regressed test.
    pub filename: String,
    /// Description of what changed.
    pub description: String,
    /// Expected value.
    pub expected: String,
    /// Actual value.
    pub actual: String,
}

impl std::fmt::Display for Regression {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}: {} (expected: {}, got: {})",
            self.filename, self.description, self.expected, self.actual
        )
    }
}

/// Result of comparing actual test results against a golden baseline.
#[derive(Clone, Debug, Default)]
pub struct RegressionReport {
    /// List of regressions found.
    pub regressions: Vec<Regression>,
    /// Total number of files tested.
    pub total_files: usize,
    /// Number of files that passed without regression.
    pub passed: usize,
}

impl RegressionReport {
    /// Whether any regressions were found.
    pub fn has_regressions(&self) -> bool {
        !self.regressions.is_empty()
    }

    /// Summary string for display.
    pub fn summary(&self) -> String {
        format!(
            "{}/{} files passed, {} regressions",
            self.passed,
            self.total_files,
            self.regressions.len()
        )
    }
}

/// Compare actual test results against a golden baseline.
///
/// Returns a report listing all regressions found.
/// A regression is defined as:
/// - A file that was parse_ok but no longer is
/// - A file that was triangulate_ok but no longer is
/// - Triangle count changed by more than 10%
/// - Vertex count changed by more than 10%
/// - Watertightness status changed (was watertight, now not)
/// - Euler characteristic changed
pub fn compare_to_baseline(baseline: &GoldenBaseline) -> RegressionReport {
    let results = run_industrial_tests();
    let mut report = RegressionReport {
        total_files: results.len(),
        ..Default::default()
    };

    for result in &results {
        let entry = match baseline.get(&result.filename) {
            Some(e) => e,
            None => {
                // New file not in baseline — not a regression, just new
                report.passed += 1;
                continue;
            }
        };

        let mut file_regressions = Vec::new();

        // Check parse status
        if entry.parse_ok && !result.parse_ok {
            file_regressions.push(Regression {
                filename: result.filename.clone(),
                description: "parse status degraded".to_string(),
                expected: "parse_ok=true".to_string(),
                actual: "parse_ok=false".to_string(),
            });
        }

        // Check triangulation status
        if entry.triangulate_ok && !result.triangulate_ok {
            file_regressions.push(Regression {
                filename: result.filename.clone(),
                description: "triangulation status degraded".to_string(),
                expected: "triangulate_ok=true".to_string(),
                actual: "triangulate_ok=false".to_string(),
            });
        }

        // Check triangle count (if triangulation succeeded)
        if entry.triangulate_ok && result.triangulate_ok && entry.triangle_count > 0 {
            let expected = entry.triangle_count as f64;
            let actual = result.triangle_count as f64;
            let ratio = (actual - expected).abs() / expected;
            if ratio > 0.10 {
                file_regressions.push(Regression {
                    filename: result.filename.clone(),
                    description: "triangle count changed >10%".to_string(),
                    expected: format!("{}", entry.triangle_count),
                    actual: format!("{}", result.triangle_count),
                });
            }
        }

        // Check vertex count (if triangulation succeeded)
        if entry.triangulate_ok && result.triangulate_ok && entry.vertex_count > 0 {
            let expected = entry.vertex_count as f64;
            let actual = result.vertex_count as f64;
            let ratio = (actual - expected).abs() / expected;
            if ratio > 0.10 {
                file_regressions.push(Regression {
                    filename: result.filename.clone(),
                    description: "vertex count changed >10%".to_string(),
                    expected: format!("{}", entry.vertex_count),
                    actual: format!("{}", result.vertex_count),
                });
            }
        }

        // Check watertightness
        if entry.triangulate_ok && result.triangulate_ok {
            let (actual_watertight, actual_euler) =
                parse_manifold_report(&result.manifold_report);
            if entry.watertight && !actual_watertight {
                file_regressions.push(Regression {
                    filename: result.filename.clone(),
                    description: "watertightness lost".to_string(),
                    expected: "watertight=true".to_string(),
                    actual: "watertight=false".to_string(),
                });
            }

            // Check Euler characteristic
            if entry.euler_characteristic != actual_euler {
                file_regressions.push(Regression {
                    filename: result.filename.clone(),
                    description: "Euler characteristic changed".to_string(),
                    expected: format!("{}", entry.euler_characteristic),
                    actual: format!("{}", actual_euler),
                });
            }
        }

        if file_regressions.is_empty() {
            report.passed += 1;
        } else {
            report.regressions.extend(file_regressions);
        }
    }

    report
}

/// Parse the manifold report string to extract watertightness and Euler characteristic.
fn parse_manifold_report(report: &str) -> (bool, i64) {
    let watertight = report.contains("watertight=true");
    let euler = extract_euler_from_report(report);
    (watertight, euler)
}

/// Extract Euler characteristic from a manifold report string.
fn extract_euler_from_report(report: &str) -> i64 {
    // Format: "watertight=true boundary=0 euler=2 degenerate=0"
    let parts: Vec<&str> = report.split_whitespace().collect();
    for part in parts {
        if let Some(euler_str) = part.strip_prefix("euler=") {
            if let Ok(e) = euler_str.parse::<i64>() {
                return e;
            }
        }
    }
    0
}

// ── JSON field extraction helpers (minimal JSON parser) ──

fn extract_string_field(json: &str, field: &str) -> Result<String, String> {
    let needle = format!("\"{}\":\"", field);
    if let Some(start) = json.find(&needle) {
        let rest = &json[start + needle.len()..];
        if let Some(end) = rest.find('"') {
            return Ok(rest[..end].to_string());
        }
    }
    Err(format!("Field '{}' not found in JSON", field))
}

fn extract_bool_field(json: &str, field: &str) -> Result<bool, String> {
    let needle_true = format!("\"{}\":true", field);
    let needle_false = format!("\"{}\":false", field);
    if json.contains(&needle_true) {
        Ok(true)
    } else if json.contains(&needle_false) {
        Ok(false)
    } else {
        Err(format!("Boolean field '{}' not found in JSON", field))
    }
}

fn extract_num_field(json: &str, field: &str) -> Result<i64, String> {
    let needle = format!("\"{}\":", field);
    if let Some(start) = json.find(&needle) {
        let rest = &json[start + needle.len()..];
        let num_str: String = rest
            .chars()
            .take_while(|c| c.is_ascii_digit() || *c == '-')
            .collect();
        if let Ok(n) = num_str.parse::<i64>() {
            return Ok(n);
        }
    }
    Err(format!("Numeric field '{}' not found in JSON", field))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_golden_entry_json_roundtrip() {
        let entry = GoldenEntry {
            filename: "test/nist_cube.stp".to_string(),
            parse_ok: true,
            triangulate_ok: true,
            vertex_count: 8,
            triangle_count: 12,
            watertight: true,
            euler_characteristic: 2,
        };
        let json = entry.to_json();
        let parsed = GoldenEntry::from_json(&json).unwrap();
        assert_eq!(parsed, entry);
    }

    #[test]
    fn test_golden_baseline_json_roundtrip() {
        let mut baseline = GoldenBaseline::new();
        baseline.entries.insert(
            "test/nist_cube.stp".to_string(),
            GoldenEntry {
                filename: "test/nist_cube.stp".to_string(),
                parse_ok: true,
                triangulate_ok: true,
                vertex_count: 8,
                triangle_count: 12,
                watertight: true,
                euler_characteristic: 2,
            },
        );
        baseline.entries.insert(
            "test/nist_sphere.stp".to_string(),
            GoldenEntry {
                filename: "test/nist_sphere.stp".to_string(),
                parse_ok: true,
                triangulate_ok: true,
                vertex_count: 482,
                triangle_count: 960,
                watertight: true,
                euler_characteristic: 2,
            },
        );

        let json = baseline.to_json();
        let parsed = GoldenBaseline::from_json(&json).unwrap();
        assert_eq!(parsed.len(), 2);
        assert_eq!(
            parsed.get("test/nist_cube.stp").unwrap(),
            baseline.get("test/nist_cube.stp").unwrap()
        );
    }

    #[test]
    fn test_parse_manifold_report() {
        let report = "watertight=true boundary=0 euler=2 degenerate=0";
        let (wt, euler) = parse_manifold_report(report);
        assert!(wt);
        assert_eq!(euler, 2);

        let report2 = "watertight=false boundary=42 euler=0 degenerate=1";
        let (wt2, euler2) = parse_manifold_report(report2);
        assert!(!wt2);
        assert_eq!(euler2, 0);
    }

    #[test]
    fn test_extract_string_field() {
        let json = r#"{"filename":"test/cube.stp","parse_ok":true}"#;
        let filename = extract_string_field(json, "filename").unwrap();
        assert_eq!(filename, "test/cube.stp");
    }

    #[test]
    fn test_extract_bool_field() {
        let json = r#"{"parse_ok":true,"triangulate_ok":false}"#;
        assert_eq!(extract_bool_field(json, "parse_ok").unwrap(), true);
        assert_eq!(extract_bool_field(json, "triangulate_ok").unwrap(), false);
    }

    #[test]
    fn test_extract_num_field() {
        let json = r#"{"vertex_count":482,"triangle_count":960,"euler_characteristic":2}"#;
        assert_eq!(extract_num_field(json, "vertex_count").unwrap(), 482);
        assert_eq!(extract_num_field(json, "triangle_count").unwrap(), 960);
        assert_eq!(extract_num_field(json, "euler_characteristic").unwrap(), 2);
    }

    #[test]
    fn test_extract_num_field_negative() {
        let json = r#"{"euler_characteristic":-2}"#;
        assert_eq!(extract_num_field(json, "euler_characteristic").unwrap(), -2);
    }

    #[test]
    fn test_regression_display() {
        let reg = Regression {
            filename: "test/cube.stp".to_string(),
            description: "triangle count changed >10%".to_string(),
            expected: "12".to_string(),
            actual: "24".to_string(),
        };
        let s = format!("{}", reg);
        assert!(s.contains("test/cube.stp"));
        assert!(s.contains("12"));
        assert!(s.contains("24"));
    }

    #[test]
    fn test_regression_report_summary() {
        let report = RegressionReport {
            regressions: vec![],
            total_files: 10,
            passed: 10,
        };
        assert!(!report.has_regressions());
        assert_eq!(report.summary(), "10/10 files passed, 0 regressions");

        let report2 = RegressionReport {
            regressions: vec![Regression {
                filename: "test/cube.stp".to_string(),
                description: "test".to_string(),
                expected: "1".to_string(),
                actual: "2".to_string(),
            }],
            total_files: 10,
            passed: 9,
        };
        assert!(report2.has_regressions());
        assert_eq!(report2.summary(), "9/10 files passed, 1 regressions");
    }

    #[test]
    fn test_empty_baseline() {
        let baseline = GoldenBaseline::new();
        assert!(baseline.is_empty());
        assert_eq!(baseline.len(), 0);
        assert!(baseline.get("nonexistent.stp").is_none());
    }
}
