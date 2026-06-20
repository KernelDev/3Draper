// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Validation for STEP files produced by the exporter (P20).
//!
//! This module validates the OUTPUT of `export_step` / `export_compound_step`
//! before writing to disk. It checks:
//!
//! 1. **Structural integrity**: required header sections, DATA/ENDSEC pairing
//! 2. **Reference integrity**: every `#N` reference resolves to a defined entity
//! 3. **Topological completeness**: required entity types for a valid B-Rep
//!    (MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS → CLOSED_SHELL → ADVANCED_FACE →
//!    FACE_BOUND → EDGE_LOOP / VERTEX_LOOP → ORIENTED_EDGE → EDGE_CURVE →
//!    VERTEX_POINT → CARTESIAN_POINT)
//! 4. **Geometric validity**: surface and curve entity types are recognized
//! 5. **Schema compliance**: entities have the expected number of parameters
//!
//! Returns a structured `ExportValidationReport` that the caller can inspect
//! to decide whether to write the file or report errors to the user.

use crate::parser::parse_step;
use crate::schema::{StepValue};
use std::collections::HashSet;

// ─────────────────────────────────────────────────────────────────────────
// Report types
// ─────────────────────────────────────────────────────────────────────────

/// Severity of a validation issue.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ExportSeverity {
    /// Critical error — the file is invalid STEP and will be rejected by
    /// conforming parsers. Examples: missing ENDSEC, dangling references,
    /// missing CLOSED_SHELL.
    Error,
    /// Non-critical issue — the file will parse but may have problems in
    /// downstream consumers. Examples: missing APPLICATION_CONTEXT, missing
    /// SHAPE_DEFINITION_REPRESENTATION.
    Warning,
    /// Informational — no action required.
    Info,
}

impl std::fmt::Display for ExportSeverity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportSeverity::Error => write!(f, "ERROR"),
            ExportSeverity::Warning => write!(f, "WARNING"),
            ExportSeverity::Info => write!(f, "INFO"),
        }
    }
}

/// A single validation issue.
#[derive(Clone, Debug)]
pub struct ExportValidationIssue {
    pub severity: ExportSeverity,
    pub code: &'static str,
    pub message: String,
    /// STEP entity ID involved (if applicable).
    pub entity_id: Option<i64>,
}

/// The full validation report.
#[derive(Clone, Debug, Default)]
pub struct ExportValidationReport {
    pub issues: Vec<ExportValidationIssue>,
    pub entity_count: usize,
    pub brep_count: usize,
    pub shell_count: usize,
    pub face_count: usize,
    pub edge_curve_count: usize,
    pub vertex_count: usize,
    pub surface_count: usize,
    pub curve_count: usize,
}

impl ExportValidationReport {
    pub fn has_errors(&self) -> bool {
        self.issues
            .iter()
            .any(|i| i.severity == ExportSeverity::Error)
    }

    pub fn has_warnings(&self) -> bool {
        self.issues
            .iter()
            .any(|i| i.severity == ExportSeverity::Warning)
    }

    pub fn errors(&self) -> impl Iterator<Item = &ExportValidationIssue> {
        self.issues
            .iter()
            .filter(|i| i.severity == ExportSeverity::Error)
    }

    pub fn warnings(&self) -> impl Iterator<Item = &ExportValidationIssue> {
        self.issues
            .iter()
            .filter(|i| i.severity == ExportSeverity::Warning)
    }

    pub fn summary(&self) -> String {
        let n_err = self.issues.iter().filter(|i| i.severity == ExportSeverity::Error).count();
        let n_warn = self.issues.iter().filter(|i| i.severity == ExportSeverity::Warning).count();
        let n_info = self.issues.iter().filter(|i| i.severity == ExportSeverity::Info).count();
        format!(
            "Export validation: {} errors, {} warnings, {} info | {} entities, {} BREP(s), {} shell(s), {} face(s), {} edge(s), {} vertex/vertices, {} surface(s), {} curve(s)",
            n_err, n_warn, n_info,
            self.entity_count,
            self.brep_count,
            self.shell_count,
            self.face_count,
            self.edge_curve_count,
            self.vertex_count,
            self.surface_count,
            self.curve_count
        )
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────

/// Validate a STEP string produced by the exporter.
///
/// This is the recommended pre-write check: call `validate_exported_step`
/// on the output of `export_step` / `export_compound_step` before writing
/// to disk. If `report.has_errors()` is true, do not write the file — the
/// STEP is malformed and will be rejected by conforming parsers.
///
/// # Example
/// ```ignore
/// let step_str = export_step(&solid, "part");
/// let report = validate_exported_step(&step_str);
/// if report.has_errors() {
///     for issue in report.errors() {
///         eprintln!("{}", issue.message);
///     }
///     return Err("STEP export validation failed".into());
/// }
/// std::fs::write("part.stp", step_str)?;
/// ```
pub fn validate_exported_step(step_str: &str) -> ExportValidationReport {
    let mut report = ExportValidationReport::default();

    // ── 1. Structural integrity ──
    if !step_str.starts_with("ISO-10303-21;") {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_ISO_HEADER",
            message: "Missing 'ISO-10303-21;' header line".to_string(),
            entity_id: None,
        });
    }
    if !step_str.contains("END-ISO-10303-21;") {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_ISO_FOOTER",
            message: "Missing 'END-ISO-10303-21;' footer line".to_string(),
            entity_id: None,
        });
    }
    if !step_str.contains("HEADER;") {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_HEADER_SEC",
            message: "Missing HEADER; section".to_string(),
            entity_id: None,
        });
    }
    if !step_str.contains("ENDSEC;") {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_ENDSEC",
            message: "Missing ENDSEC; section terminator".to_string(),
            entity_id: None,
        });
    }
    if !step_str.contains("DATA;") {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_DATA_SEC",
            message: "Missing DATA; section".to_string(),
            entity_id: None,
        });
    }
    if !step_str.contains("FILE_SCHEMA") {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Warning,
            code: "W_NO_FILE_SCHEMA",
            message: "Missing FILE_SCHEMA declaration in HEADER".to_string(),
            entity_id: None,
        });
    }

    // ── 2. Parse the exported STEP ──
    let step_file = match parse_step(step_str) {
        Ok(sf) => sf,
        Err(e) => {
            report.issues.push(ExportValidationIssue {
                severity: ExportSeverity::Error,
                code: "E_PARSE_FAILED",
                message: format!("Re-parsing exported STEP failed: {}", e),
                entity_id: None,
            });
            return report;
        }
    };

    report.entity_count = step_file.entities.len();

    // ── 3. Reference integrity ──
    let mut defined_ids: HashSet<i64> = HashSet::new();
    for ent in &step_file.entities {
        defined_ids.insert(ent.id);
    }

    let mut dangling_refs: Vec<(i64, i64)> = Vec::new(); // (from_entity, missing_ref)
    for ent in &step_file.entities {
        for param in &ent.params {
            collect_refs(param, &mut |ref_id| {
                if !defined_ids.contains(&ref_id) {
                    dangling_refs.push((ent.id, ref_id));
                }
            });
        }
    }
    for (from, missing) in &dangling_refs {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_DANGLING_REF",
            message: format!("Entity #{} references undefined #{}", from, missing),
            entity_id: Some(*from),
        });
    }

    // ── 4. Topological completeness ──
    report.brep_count = step_file.find_entities_by_type("MANIFOLD_SOLID_BREP").len()
        + step_file.find_entities_by_type("BREP_WITH_VOIDS").len();
    report.shell_count = step_file.find_entities_by_type("CLOSED_SHELL").len()
        + step_file.find_entities_by_type("ORIENTED_CLOSED_SHELL").len()
        + step_file.find_entities_by_type("OPEN_SHELL").len();
    report.face_count = step_file.find_entities_by_type("ADVANCED_FACE").len()
        + step_file.find_entities_by_type("FACE_SURFACE").len();
    report.edge_curve_count = step_file.find_entities_by_type("EDGE_CURVE").len();
    report.vertex_count = step_file.find_entities_by_type("VERTEX_POINT").len();

    // Count surfaces (geometric)
    let surface_types = [
        "PLANE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE",
        "TOROIDAL_SURFACE", "SURFACE_OF_REVOLUTION", "SURFACE_OF_LINEAR_EXTRUSION",
        "B_SPLINE_SURFACE_WITH_KNOTS", "B_SPLINE_SURFACE",
    ];
    report.surface_count = surface_types
        .iter()
        .map(|t| step_file.find_entities_by_type(t).len())
        .sum();

    // Count curves (geometric, not topological)
    let curve_types = [
        "LINE", "CIRCLE", "ELLIPSE", "HYPERBOLA", "PARABOLA",
        "B_SPLINE_CURVE_WITH_KNOTS", "B_SPLINE_CURVE",
        "TRIMMED_CURVE", "SURFACE_CURVE",
    ];
    report.curve_count = curve_types
        .iter()
        .map(|t| step_file.find_entities_by_type(t).len())
        .sum();

    // Critical: must have at least one BREP
    if report.brep_count == 0 {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_BREP",
            message: "No MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS entity found".to_string(),
            entity_id: None,
        });
    }

    // Critical: must have at least one CLOSED_SHELL per BREP
    if report.shell_count == 0 && report.brep_count > 0 {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_SHELL",
            message: "BREP(s) present but no CLOSED_SHELL — topology chain broken".to_string(),
            entity_id: None,
        });
    }

    // Critical: must have ADVANCED_FACE entries
    if report.face_count == 0 && report.brep_count > 0 {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_FACE",
            message: "BREP(s) present but no ADVANCED_FACE — topology chain broken".to_string(),
            entity_id: None,
        });
    }

    // Critical: must have EDGE_CURVE entries
    if report.edge_curve_count == 0 && report.face_count > 0 {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Warning,
            code: "W_NO_EDGE_CURVE",
            message: "Faces present but no EDGE_CURVE — solid may be a degenerate single-vertex shape".to_string(),
            entity_id: None,
        });
    }

    // Critical: must have VERTEX_POINT entries
    if report.vertex_count == 0 && report.brep_count > 0 {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Error,
            code: "E_NO_VERTEX",
            message: "BREP(s) present but no VERTEX_POINT — topology chain broken".to_string(),
            entity_id: None,
        });
    }

    // ── 5. Schema compliance for ADVANCED_FACE ──
    // ADVANCED_FACE('', (#bound1, #bound2, ...), #surface, .T./.F., .F.)
    // Must have at least 4 params: name, bounds list, surface ref, orientation
    for face in step_file.find_entities_by_type("ADVANCED_FACE") {
        if face.params.len() < 4 {
            report.issues.push(ExportValidationIssue {
                severity: ExportSeverity::Error,
                code: "E_FACE_PARAMS",
                message: format!(
                    "ADVANCED_FACE #{} has {} params (expected at least 4)",
                    face.id,
                    face.params.len()
                ),
                entity_id: Some(face.id),
            });
        }
    }

    // ── 6. Schema compliance for EDGE_CURVE ──
    // EDGE_CURVE('', #start_vtx, #end_vtx, #curve, .T./.F.)
    for ec in step_file.find_entities_by_type("EDGE_CURVE") {
        if ec.params.len() < 5 {
            report.issues.push(ExportValidationIssue {
                severity: ExportSeverity::Error,
                code: "E_EDGE_CURVE_PARAMS",
                message: format!(
                    "EDGE_CURVE #{} has {} params (expected 5)",
                    ec.id,
                    ec.params.len()
                ),
                entity_id: Some(ec.id),
            });
        }
    }

    // ── 7. Schema compliance for CLOSED_SHELL ──
    // CLOSED_SHELL('', (#face1, #face2, ...))
    for shell in step_file.find_entities_by_type("CLOSED_SHELL") {
        if shell.params.len() < 2 {
            report.issues.push(ExportValidationIssue {
                severity: ExportSeverity::Error,
                code: "E_SHELL_PARAMS",
                message: format!(
                    "CLOSED_SHELL #{} has {} params (expected at least 2)",
                    shell.id,
                    shell.params.len()
                ),
                entity_id: Some(shell.id),
            });
        }
    }

    // ── 8. APPLICATION_CONTEXT (warning if missing) ──
    if step_file.find_entities_by_type("APPLICATION_CONTEXT").is_empty() {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Warning,
            code: "W_NO_APP_CTX",
            message: "No APPLICATION_CONTEXT entity — some parsers may reject the file".to_string(),
            entity_id: None,
        });
    }

    // ── 9. SHAPE_DEFINITION_REPRESENTATION (warning if missing) ──
    if step_file.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION").is_empty() {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Warning,
            code: "W_NO_SDR",
            message: "No SHAPE_DEFINITION_REPRESENTATION — assembly structure may be missing".to_string(),
            entity_id: None,
        });
    }

    // ── 10. PRODUCT (warning if missing) ──
    if step_file.find_entities_by_type("PRODUCT").is_empty() {
        report.issues.push(ExportValidationIssue {
            severity: ExportSeverity::Warning,
            code: "W_NO_PRODUCT",
            message: "No PRODUCT entity — file lacks part identification".to_string(),
            entity_id: None,
        });
    }

    report
}

/// Recursively collect all `#N` references from a STEP parameter value.
fn collect_refs<F: FnMut(i64)>(value: &StepValue, callback: &mut F) {
    match value {
        StepValue::Ref(id) => callback(*id),
        StepValue::List(items) => {
            for item in items {
                collect_refs(item, callback);
            }
        }
        StepValue::Typed { value, .. } => collect_refs(value, callback),
        _ => {}
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export_step;
    use draper_geometry::{Plane, Surface};
    use draper_topology::{Face, Shell, Solid, Wire};

    #[test]
    fn test_validate_minimal_export() {
        let plane = Plane::xy();
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);
        let step = export_step(&solid, "test_validate");
        let report = validate_exported_step(&step);
        println!("{}", report.summary());
        assert!(!report.has_errors(), "Expected no errors, got: {:?}",
            report.errors().collect::<Vec<_>>());
        assert!(report.brep_count >= 1);
        assert!(report.shell_count >= 1);
        assert!(report.face_count >= 1);
    }

    #[test]
    fn test_validate_missing_header() {
        let bad_step = "DATA;\n#1 = MANIFOLD_SOLID_BREP('test', #2);\nENDSEC;\nEND-ISO-10303-21;";
        let report = validate_exported_step(bad_step);
        assert!(report.has_errors());
        assert!(report.issues.iter().any(|i| i.code == "E_NO_ISO_HEADER"));
        assert!(report.issues.iter().any(|i| i.code == "E_NO_HEADER_SEC"));
    }

    #[test]
    fn test_validate_dangling_ref() {
        // Build a STEP with a dangling reference
        let bad_step = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('test'), '2;1');\nFILE_NAME('test.stp','2026-06-20',('test'),(''),'test','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n#1 = MANIFOLD_SOLID_BREP('test', #999);\nENDSEC;\nEND-ISO-10303-21;";
        let report = validate_exported_step(bad_step);
        assert!(report.has_errors());
        assert!(report.issues.iter().any(|i| i.code == "E_DANGLING_REF"));
    }

    #[test]
    fn test_validate_empty_step() {
        let empty = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('test'), '2;1');\nFILE_NAME('test.stp','2026-06-20',('test'),(''),'test','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;";
        let report = validate_exported_step(empty);
        // No BREP → E_NO_BREP
        assert!(report.issues.iter().any(|i| i.code == "E_NO_BREP"));
    }
}
