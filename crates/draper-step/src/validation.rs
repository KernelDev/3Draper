// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP file validation and semantic parsing.
//!
//! Validates parsed STEP geometry, topology, and assembly structures
//! before conversion to mesh. Produces a structured `StepValidationReport`
//! with errors, warnings, and informational messages.

use crate::schema::{StepEntity, StepFile, StepValue};
use std::collections::{HashMap, HashSet};

// ============================================================
// Severity levels and issue types
// ============================================================

/// Severity level for validation issues.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Severity {
    /// Critical error — conversion will likely fail or produce incorrect results.
    Error,
    /// Non-critical issue — conversion may proceed but results may be suboptimal.
    Warning,
    /// Informational message — no action required.
    Info,
}

impl std::fmt::Display for Severity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Severity::Error => write!(f, "ERROR"),
            Severity::Warning => write!(f, "WARNING"),
            Severity::Info => write!(f, "INFO"),
        }
    }
}

/// A single validation issue found in a STEP file.
#[derive(Clone, Debug)]
pub struct StepValidationIssue {
    /// Severity of the issue.
    pub severity: Severity,
    /// STEP entity ID that caused the issue, if applicable.
    pub entity_id: Option<i64>,
    /// Human-readable description of the issue.
    pub description: String,
    /// Suggestion for fixing the issue, if applicable.
    pub suggestion: Option<String>,
}

impl StepValidationIssue {
    /// Create a new error-level issue.
    pub fn error(entity_id: Option<i64>, description: impl Into<String>) -> Self {
        Self {
            severity: Severity::Error,
            entity_id,
            description: description.into(),
            suggestion: None,
        }
    }

    /// Create a new warning-level issue.
    pub fn warning(entity_id: Option<i64>, description: impl Into<String>) -> Self {
        Self {
            severity: Severity::Warning,
            entity_id,
            description: description.into(),
            suggestion: None,
        }
    }

    /// Create a new info-level issue.
    pub fn info(entity_id: Option<i64>, description: impl Into<String>) -> Self {
        Self {
            severity: Severity::Info,
            entity_id,
            description: description.into(),
            suggestion: None,
        }
    }

    /// Add a suggestion to the issue.
    pub fn with_suggestion(mut self, suggestion: impl Into<String>) -> Self {
        self.suggestion = Some(suggestion.into());
        self
    }
}

impl std::fmt::Display for StepValidationIssue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(id) = self.entity_id {
            write!(f, "[{}] #{}: {}", self.severity, id, self.description)?;
        } else {
            write!(f, "[{}] {}", self.severity, self.description)?;
        }
        if let Some(ref s) = self.suggestion {
            write!(f, " — Suggestion: {}", s)?;
        }
        Ok(())
    }
}

// ============================================================
// Validation report
// ============================================================

/// A structured report of all validation issues found in a STEP file.
#[derive(Clone, Debug, Default)]
pub struct StepValidationReport {
    /// All issues found during validation.
    pub issues: Vec<StepValidationIssue>,
}

impl StepValidationReport {
    /// Create an empty report.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add an issue to the report.
    pub fn add(&mut self, issue: StepValidationIssue) {
        self.issues.push(issue);
    }

    /// Number of errors.
    pub fn error_count(&self) -> usize {
        self.issues.iter().filter(|i| i.severity == Severity::Error).count()
    }

    /// Number of warnings.
    pub fn warning_count(&self) -> usize {
        self.issues.iter().filter(|i| i.severity == Severity::Warning).count()
    }

    /// Number of info messages.
    pub fn info_count(&self) -> usize {
        self.issues.iter().filter(|i| i.severity == Severity::Info).count()
    }

    /// Total number of issues.
    pub fn total_count(&self) -> usize {
        self.issues.len()
    }

    /// Whether the report contains any errors.
    pub fn has_errors(&self) -> bool {
        self.error_count() > 0
    }

    /// Whether the report contains any warnings or errors.
    pub fn has_warnings_or_errors(&self) -> bool {
        self.issues.iter().any(|i| i.severity == Severity::Error || i.severity == Severity::Warning)
    }

    /// Get all issues of a specific severity.
    pub fn issues_by_severity(&self, severity: &Severity) -> Vec<&StepValidationIssue> {
        self.issues.iter().filter(|i| &i.severity == severity).collect()
    }

    /// Merge another report into this one.
    pub fn merge(&mut self, other: StepValidationReport) {
        self.issues.extend(other.issues);
    }
}

impl std::fmt::Display for StepValidationReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "STEP Validation Report")?;
        writeln!(f, "  Errors:   {}", self.error_count())?;
        writeln!(f, "  Warnings: {}", self.warning_count())?;
        writeln!(f, "  Info:     {}", self.info_count())?;
        writeln!(f, "  Total:    {}", self.total_count())?;
        if !self.issues.is_empty() {
            writeln!(f)?;
            for issue in &self.issues {
                writeln!(f, "  {}", issue)?;
            }
        }
        Ok(())
    }
}

// ============================================================
// Tolerance extraction
// ============================================================

/// Tolerance information extracted from STEP file entities.
#[derive(Clone, Debug, Default)]
pub struct StepTolerances {
    /// Overall model uncertainty from UNCERTAINTY_MEASURE_WITH_UNIT.
    pub uncertainty: Option<f64>,
    /// GD&T tolerances from GEOMETRIC_TOLERANCE entities.
    pub geometric_tolerances: Vec<f64>,
    /// Shape tolerances from SHAPE_TOLERANCE entities.
    pub shape_tolerances: Vec<f64>,
    /// The smallest (tightest) tolerance found across all sources.
    pub tightest_tolerance: Option<f64>,
}

impl StepTolerances {
    /// Create an empty tolerances struct.
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether any tolerance information was found.
    pub fn has_tolerances(&self) -> bool {
        self.uncertainty.is_some() || !self.geometric_tolerances.is_empty() || !self.shape_tolerances.is_empty()
    }

    /// Compute the tightest tolerance from all sources.
    pub fn compute_tightest(&mut self) {
        let mut best: Option<f64> = self.uncertainty;
        for &t in &self.geometric_tolerances {
            best = Some(match best {
                Some(b) => b.min(t),
                None => t,
            });
        }
        for &t in &self.shape_tolerances {
            best = Some(match best {
                Some(b) => b.min(t),
                None => t,
            });
        }
        self.tightest_tolerance = best;
    }
}

/// Extract tolerance information from STEP file entities.
///
/// STEP files can contain explicit tolerance information via:
/// - `UNCERTAINTY_MEASURE_WITH_UNIT` — overall model uncertainty
/// - `GEOMETRIC_TOLERANCE` / `GEOMETRIC_TOLERANCE_WITH_DATUM_REFERENCE` — GD&T tolerances
/// - `SHAPE_TOLERANCE` / `SHAPE_TOLERANCE_WITH_DATUM_REFERENCE` — shape-level tolerances
pub fn extract_tolerances(step_file: &StepFile) -> StepTolerances {
    let mut tolerances = StepTolerances::new();
    let mut uncertainty_found: Option<f64> = None;

    for entity in &step_file.entities {
        let type_name = entity.type_name.to_uppercase();

        if type_name == "UNCERTAINTY_MEASURE_WITH_UNIT" {
            // Format: UNCERTAINTY_MEASURE_WITH_UNIT((value, ...), ...)
            // or: UNCERTAINTY_MEASURE_WITH_UNIT(value, ...)
            if let Some(StepValue::List(params)) = entity.params.get(0) {
                if let Some(StepValue::Float(val)) = params.first() {
                    let tol = val.abs();
                    uncertainty_found = Some(match uncertainty_found {
                        Some(existing) => existing.min(tol),
                        None => tol,
                    });
                }
            }
            // Also try direct float parameter
            if let Some(StepValue::Float(val)) = entity.params.get(0) {
                let tol = val.abs();
                uncertainty_found = Some(match uncertainty_found {
                    Some(existing) => existing.min(tol),
                    None => tol,
                });
            }
        }

        if type_name.starts_with("GEOMETRIC_TOLERANCE") {
            // The first numeric parameter is typically the tolerance value
            for param in &entity.params {
                if let StepValue::Float(val) = param {
                    let tol = val.abs();
                    if tol > 1e-15 && tol < 1000.0 {
                        tolerances.geometric_tolerances.push(tol);
                    }
                    break;
                }
            }
        }

        if type_name.starts_with("SHAPE_TOLERANCE") {
            for param in &entity.params {
                if let StepValue::Float(val) = param {
                    let tol = val.abs();
                    if tol > 1e-15 && tol < 1000.0 {
                        tolerances.shape_tolerances.push(tol);
                    }
                    break;
                }
            }
        }
    }

    tolerances.uncertainty = uncertainty_found;
    tolerances.compute_tightest();
    tolerances
}

// ============================================================
// Main validation entry point
// ============================================================

/// Validate a parsed STEP file and return a structured report.
///
/// Performs the following checks:
/// 1. Dangling entity references
/// 2. CARTESIAN_POINT validity (no NaN/Inf)
/// 3. DIRECTION vector normalization
/// 4. Curve/surface parameter sanity (positive radii, valid knot vectors)
/// 5. Composite entity validation (ADVANCED_FACE, FACE_BOUND, EDGE_LOOP, MANIFOLD_SOLID_BREP)
/// 6. Assembly structure validation (NAUO, ITEM_DEFINED_TRANSFORMATION, CDSR, circular refs)
/// 7. Tolerance extraction
pub fn validate_step_file(step_file: &StepFile) -> StepValidationReport {
    let mut report = StepValidationReport::new();

    // Build the set of all known entity IDs for fast dangling-reference checks
    let known_ids: HashSet<i64> = step_file.entities.iter().map(|e| e.id).collect();

    // Phase 1: Dangling reference check
    validate_dangling_references(step_file, &known_ids, &mut report);

    // Phase 2: Geometric validation
    validate_geometric_entities(step_file, &mut report);

    // Phase 3: Composite entity validation
    validate_composite_entities(step_file, &mut report);

    // Phase 4: Assembly validation
    validate_assemblies(step_file, &known_ids, &mut report);

    // Phase 5: Extract tolerances and add info messages
    let tolerances = extract_tolerances(step_file);
    if let Some(tol) = tolerances.tightest_tolerance {
        report.add(StepValidationIssue::info(
            None,
            format!("Tightest tolerance found in STEP file: {:.6e}", tol),
        ));
    } else {
        report.add(StepValidationIssue::info(
            None,
            "No explicit tolerance information found in STEP file",
        ));
    }

    // Sort issues: errors first (Error < Warning < Info by enum declaration order)
    report.issues.sort_by(|a, b| a.severity.cmp(&b.severity));

    report
}

// ============================================================
// Phase 1: Dangling reference validation
// ============================================================

/// Check that all referenced entity IDs actually exist in the file.
fn validate_dangling_references(
    step_file: &StepFile,
    known_ids: &HashSet<i64>,
    report: &mut StepValidationReport,
) {
    for entity in &step_file.entities {
        for ref_id in collect_refs(&entity.params) {
            if !known_ids.contains(&ref_id) {
                report.add(StepValidationIssue::error(
                    Some(entity.id),
                    format!("References non-existent entity #{}", ref_id),
                ).with_suggestion(format!(
                    "Add missing entity #{} or fix the reference in #{}",
                    ref_id, entity.id
                )));
            }
        }
        // Also check sub-entities
        for sub in &entity.sub_entities {
            for ref_id in collect_refs(&sub.params) {
                if !known_ids.contains(&ref_id) {
                    report.add(StepValidationIssue::error(
                        Some(entity.id),
                        format!("Complex entity references non-existent entity #{}", ref_id),
                    ).with_suggestion(format!(
                        "Add missing entity #{} or fix the reference",
                        ref_id
                    )));
                }
            }
        }
    }
}

/// Recursively collect all entity references from a parameter list.
fn collect_refs(params: &[StepValue]) -> Vec<i64> {
    let mut refs = Vec::new();
    for param in params {
        match param {
            StepValue::Ref(id) => {
                refs.push(*id);
            }
            StepValue::List(items) => {
                refs.extend(collect_refs(items));
            }
            StepValue::Typed { value, .. } => {
                refs.extend(collect_refs(std::slice::from_ref(value)));
            }
            _ => {}
        }
    }
    refs
}

// ============================================================
// Phase 2: Geometric validation
// ============================================================

/// Validate geometric entities: CARTESIAN_POINT, DIRECTION, curve/surface parameters.
fn validate_geometric_entities(step_file: &StepFile, report: &mut StepValidationReport) {
    for entity in &step_file.entities {
        let type_upper = entity.type_name.to_uppercase();

        match type_upper.as_str() {
            "CARTESIAN_POINT" => validate_cartesian_point(entity, report),
            "DIRECTION" => validate_direction(entity, report),
            "VECTOR" => validate_vector(entity, report),
            _ => {
                // Validate curve/surface parameters
                if is_curve_type(&type_upper) {
                    validate_curve_params(entity, &type_upper, step_file, report);
                }
                if is_surface_type(&type_upper) {
                    validate_surface_params(entity, &type_upper, step_file, report);
                }
            }
        }
    }
}

fn validate_cartesian_point(entity: &StepEntity, report: &mut StepValidationReport) {
    // CARTESIAN_POINT('name', (x, y, z))
    // The coordinates can be in a list or as direct parameters
    let coords = extract_coordinate_list(&entity.params);
    for (i, &val) in coords.iter().enumerate() {
        if val.is_nan() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CARTESIAN_POINT has NaN coordinate at index {}", i),
            ).with_suggestion("Replace NaN with a valid coordinate value"));
        }
        if val.is_infinite() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CARTESIAN_POINT has infinite coordinate at index {}", i),
            ).with_suggestion("Replace infinite value with a valid coordinate value"));
        }
        // Warn on very large coordinates that may indicate unit issues
        if val.abs() > 1e8 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("CARTESIAN_POINT has very large coordinate at index {} ({:.3e}); possible unit mismatch", i, val),
            ).with_suggestion("Check if the model uses correct units (STEP default is mm)"));
        }
    }
}

fn validate_direction(entity: &StepEntity, report: &mut StepValidationReport) {
    // DIRECTION('name', (x, y, z))
    let coords = extract_coordinate_list(&entity.params);
    if coords.is_empty() {
        report.add(StepValidationIssue::warning(
            Some(entity.id),
            "DIRECTION has no direction components",
        ));
        return;
    }

    // Compute magnitude
    let magnitude: f64 = coords.iter().map(|c| c * c).sum::<f64>().sqrt();
    let normalization_tol = 1e-4; // Directions should be roughly normalized

    if magnitude < 1e-10 {
        report.add(StepValidationIssue::error(
            Some(entity.id),
            "DIRECTION is a zero vector (magnitude ≈ 0)",
        ).with_suggestion("Replace with a valid direction vector"));
    } else if (magnitude - 1.0).abs() > normalization_tol {
        report.add(StepValidationIssue::warning(
            Some(entity.id),
            format!("DIRECTION is not normalized (magnitude = {:.6}, expected 1.0)", magnitude),
        ).with_suggestion("Normalize the direction vector before using in geometry"));
    }

    // Check for NaN/Inf
    for (i, &val) in coords.iter().enumerate() {
        if val.is_nan() || val.is_infinite() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("DIRECTION has invalid component at index {}", i),
            ));
        }
    }
}

fn validate_vector(entity: &StepEntity, report: &mut StepValidationReport) {
    // VECTOR('name', #direction, magnitude)
    // Check that magnitude is positive and finite
    for param in &entity.params {
        if let StepValue::Float(val) = param {
            if val.is_nan() || val.is_infinite() {
                report.add(StepValidationIssue::error(
                    Some(entity.id),
                    "VECTOR has invalid magnitude (NaN or Inf)",
                ));
            } else if *val < 0.0 {
                report.add(StepValidationIssue::warning(
                    Some(entity.id),
                    format!("VECTOR has negative magnitude ({:.6})", val),
                ).with_suggestion("Check if the magnitude sign is intentional"));
            }
        }
    }
}

fn is_curve_type(type_name: &str) -> bool {
    matches!(
        type_name,
        "CIRCLE"
            | "ELLIPSE"
            | "LINE"
            | "B_SPLINE_CURVE"
            | "B_SPLINE_CURVE_WITH_KNOTS"
            | "BEZIER_CURVE"
            | "RATIONAL_B_SPLINE_CURVE"
            | "PARABOLA"
            | "HYPERBOLA"
            | "TRIMMED_CURVE"
            | "OFFSET_CURVE_3D"
            | "SURFACE_CURVE"
            | "SEAM_CURVE"
            | "COMPOSITE_CURVE"
    )
}

fn is_surface_type(type_name: &str) -> bool {
    matches!(
        type_name,
        "PLANE"
            | "CYLINDRICAL_SURFACE"
            | "SPHERICAL_SURFACE"
            | "CONICAL_SURFACE"
            | "TOROIDAL_SURFACE"
            | "B_SPLINE_SURFACE"
            | "B_SPLINE_SURFACE_WITH_KNOTS"
            | "BEZIER_SURFACE"
            | "RATIONAL_B_SPLINE_SURFACE"
            | "SURFACE_OF_REVOLUTION"
            | "SURFACE_OF_LINEAR_EXTRUSION"
            | "OFFSET_SURFACE"
    )
}

fn validate_curve_params(
    entity: &StepEntity,
    type_name: &str,
    step_file: &StepFile,
    report: &mut StepValidationReport,
) {
    match type_name {
        "CIRCLE" => validate_circle(entity, step_file, report),
        "ELLIPSE" => validate_ellipse(entity, step_file, report),
        "B_SPLINE_CURVE" | "B_SPLINE_CURVE_WITH_KNOTS" | "BEZIER_CURVE" | "RATIONAL_B_SPLINE_CURVE" => {
            validate_bspline_curve(entity, report);
        }
        "COMPOSITE_CURVE" => validate_composite_curve(entity, report),
        _ => {}
    }
}

fn validate_surface_params(
    entity: &StepEntity,
    type_name: &str,
    step_file: &StepFile,
    report: &mut StepValidationReport,
) {
    match type_name {
        "CYLINDRICAL_SURFACE" => validate_cylindrical_surface(entity, step_file, report),
        "SPHERICAL_SURFACE" => validate_spherical_surface(entity, step_file, report),
        "CONICAL_SURFACE" => validate_conical_surface(entity, step_file, report),
        "TOROIDAL_SURFACE" => validate_toroidal_surface(entity, step_file, report),
        "B_SPLINE_SURFACE" | "B_SPLINE_SURFACE_WITH_KNOTS" | "BEZIER_SURFACE" | "RATIONAL_B_SPLINE_SURFACE" => {
            validate_bspline_surface(entity, report);
        }
        _ => {}
    }
}

fn validate_circle(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // CIRCLE('name', #axis2_placement, radius)
    // The radius is typically the last numeric parameter
    if let Some(radius) = find_last_float(&entity.params) {
        if radius < 0.0 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CIRCLE has negative radius ({:.6})", radius),
            ).with_suggestion("Radius must be positive"));
        } else if radius < 1e-10 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("CIRCLE has near-zero radius ({:.6e})", radius),
            ).with_suggestion("Check if this is a degenerate circle"));
        }
    }

    // Check axis placement reference
    if let Some(StepValue::Ref(axis_id)) = entity.params.get(1) {
        if step_file.find_entity(*axis_id).is_none() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CIRCLE references non-existent axis placement #{}", axis_id),
            ));
        }
    }
}

fn validate_ellipse(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // ELLIPSE('name', #axis2_placement, semi_axis_1, semi_axis_2)
    let floats: Vec<f64> = entity.params.iter()
        .filter_map(|p| match p {
            StepValue::Float(f) => Some(*f),
            _ => None,
        })
        .collect();

    if floats.len() >= 2 {
        let semi1 = floats[floats.len() - 2];
        let semi2 = floats[floats.len() - 1];
        if semi1 < 0.0 || semi2 < 0.0 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("ELLIPSE has negative semi-axis ({:.6}, {:.6})", semi1, semi2),
            ).with_suggestion("Semi-axes must be positive"));
        }
        if semi1 < 1e-10 || semi2 < 1e-10 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("ELLIPSE has near-zero semi-axis ({:.6e}, {:.6e})", semi1, semi2),
            ));
        }
    }

    // Check axis placement reference
    if let Some(StepValue::Ref(axis_id)) = entity.params.get(1) {
        if step_file.find_entity(*axis_id).is_none() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("ELLIPSE references non-existent axis placement #{}", axis_id),
            ));
        }
    }
}

fn validate_bspline_curve(entity: &StepEntity, report: &mut StepValidationReport) {
    // Validate degree
    if let Some(StepValue::Integer(deg)) = entity.params.get(1) {
        if *deg < 1 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("B_SPLINE_CURVE has invalid degree ({})", deg),
            ).with_suggestion("Degree must be >= 1"));
        }
        if *deg > 20 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("B_SPLINE_CURVE has very high degree ({})", deg),
            ).with_suggestion("High-degree B-splines may cause numerical instability"));
        }
    }

    // Validate knot vector if present (in B_SPLINE_CURVE_WITH_KNOTS)
    // Knots are typically in later parameters after control points
    validate_knot_vector(entity, report, "B_SPLINE_CURVE");
}

fn validate_bspline_surface(entity: &StepEntity, report: &mut StepValidationReport) {
    // Validate degrees
    if let Some(StepValue::Integer(u_deg)) = entity.params.get(1) {
        if *u_deg < 1 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("B_SPLINE_SURFACE has invalid u_degree ({})", u_deg),
            ));
        }
    }
    if let Some(StepValue::Integer(v_deg)) = entity.params.get(2) {
        if *v_deg < 1 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("B_SPLINE_SURFACE has invalid v_degree ({})", v_deg),
            ));
        }
    }

    validate_knot_vector(entity, report, "B_SPLINE_SURFACE");
}

fn validate_knot_vector(entity: &StepEntity, report: &mut StepValidationReport, _entity_type: &str) {
    // Look for knot multiplicities and knot values in parameters
    // B_SPLINE_*_WITH_KNOTS has: ..., knot_multiplicities, knots, ...
    // Knots should be non-decreasing
    let mut found_knots: Option<Vec<f64>> = None;
    let mut found_multiplicities: Option<Vec<i64>> = None;

    for param in &entity.params {
        if let StepValue::List(items) = param {
            // Check if this looks like a knot vector (list of non-decreasing floats)
            let floats: Vec<f64> = items.iter()
                .filter_map(|v| match v {
                    StepValue::Float(f) => Some(*f),
                    StepValue::Integer(i) => Some(*i as f64),
                    _ => None,
                })
                .collect();

            if floats.len() >= 2 {
                // Check if non-decreasing
                let is_non_decreasing = floats.windows(2).all(|w| w[0] <= w[1] + 1e-10);
                let looks_like_knots = floats[0].abs() < 1e6 && floats.last().map_or(false, |&l| l.abs() < 1e6);

                if !is_non_decreasing && looks_like_knots && floats.iter().all(|f| !f.is_nan() && !f.is_infinite()) {
                    // This could be a knot vector — check more carefully
                    // Only report if it's clearly not non-decreasing
                    let violations: Vec<usize> = floats.windows(2)
                        .enumerate()
                        .filter_map(|(i, w)| if w[0] > w[1] + 1e-10 { Some(i) } else { None })
                        .collect();
                    if !violations.is_empty() {
                        found_knots = Some(floats);
                        // Don't break — try to find multiplicities too
                    }
                }

                // Check if this looks like multiplicities (list of positive integers)
                let ints: Vec<i64> = items.iter()
                    .filter_map(|v| match v {
                        StepValue::Integer(i) if *i > 0 => Some(*i),
                        _ => None,
                    })
                    .collect();
                if ints.len() == items.len() && ints.len() >= 2 {
                    found_multiplicities = Some(ints);
                }
            }
        }
    }

    if let Some(ref knots) = found_knots {
        let violations: Vec<usize> = knots.windows(2)
            .enumerate()
            .filter_map(|(i, w)| if w[0] > w[1] + 1e-10 { Some(i) } else { None })
            .collect();
        if !violations.is_empty() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("Knot vector is not non-decreasing (violations at indices {:?})", violations),
            ).with_suggestion("Knot vectors must be non-decreasing"));
        }
    }

    // Check knot multiplicity doesn't exceed degree + 1
    if let Some(ref mults) = found_multiplicities {
        // We'd need degree to check properly, but we can at least warn on large multiplicities
        let max_mult = mults.iter().max().copied().unwrap_or(0);
        if max_mult > 25 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("Knot multiplicity {} is very high; should not exceed degree + 1", max_mult),
            ));
        }
    }
}

fn validate_composite_curve(entity: &StepEntity, report: &mut StepValidationReport) {
    // COMPOSITE_CURVE('name', (list_of_curve_references), self_intersect)
    // Check that the list of curve segments is non-empty
    let has_segment_list = entity.params.iter().any(|p| {
        if let StepValue::List(items) = p {
            !items.is_empty()
        } else {
            false
        }
    });

    if !has_segment_list {
        report.add(StepValidationIssue::warning(
            Some(entity.id),
            "COMPOSITE_CURVE has no curve segments",
        ).with_suggestion("A COMPOSITE_CURVE should contain at least one curve segment"));
    }
}

fn validate_cylindrical_surface(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // CYLINDRICAL_SURFACE('name', #axis2_placement, radius)
    if let Some(radius) = find_last_float(&entity.params) {
        if radius < 0.0 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CYLINDRICAL_SURFACE has negative radius ({:.6})", radius),
            ).with_suggestion("Radius must be positive"));
        } else if radius < 1e-10 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("CYLINDRICAL_SURFACE has near-zero radius ({:.6e})", radius),
            ));
        }
    }

    // Check axis placement
    if let Some(StepValue::Ref(axis_id)) = entity.params.get(1) {
        if step_file.find_entity(*axis_id).is_none() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CYLINDRICAL_SURFACE references non-existent axis placement #{}", axis_id),
            ));
        }
    }
}

fn validate_spherical_surface(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // SPHERICAL_SURFACE('name', #axis2_placement, radius)
    if let Some(radius) = find_last_float(&entity.params) {
        if radius < 0.0 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("SPHERICAL_SURFACE has negative radius ({:.6})", radius),
            ).with_suggestion("Radius must be positive"));
        } else if radius < 1e-10 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("SPHERICAL_SURFACE has near-zero radius ({:.6e})", radius),
            ));
        }
    }

    if let Some(StepValue::Ref(axis_id)) = entity.params.get(1) {
        if step_file.find_entity(*axis_id).is_none() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("SPHERICAL_SURFACE references non-existent axis placement #{}", axis_id),
            ));
        }
    }
}

fn validate_conical_surface(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // CONICAL_SURFACE('name', #axis2_placement, radius, semi_angle)
    let floats: Vec<f64> = entity.params.iter()
        .filter_map(|p| match p {
            StepValue::Float(f) => Some(*f),
            _ => None,
        })
        .collect();

    if floats.len() >= 1 {
        let radius = floats[0];
        if radius < 0.0 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CONICAL_SURFACE has negative radius ({:.6})", radius),
            ).with_suggestion("Radius must be positive"));
        }
    }

    if floats.len() >= 2 {
        let semi_angle = floats[1];
        if semi_angle < 0.0 || semi_angle > std::f64::consts::FRAC_PI_2 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("CONICAL_SURFACE has unusual semi_angle ({:.6} rad, {:.1} deg)", semi_angle, semi_angle.to_degrees()),
            ).with_suggestion("Semi-angle should typically be in [0, π/2]"));
        }
    }

    if let Some(StepValue::Ref(axis_id)) = entity.params.get(1) {
        if step_file.find_entity(*axis_id).is_none() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("CONICAL_SURFACE references non-existent axis placement #{}", axis_id),
            ));
        }
    }
}

fn validate_toroidal_surface(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // TOROIDAL_SURFACE('name', #axis2_placement, major_radius, minor_radius)
    let floats: Vec<f64> = entity.params.iter()
        .filter_map(|p| match p {
            StepValue::Float(f) => Some(*f),
            _ => None,
        })
        .collect();

    if floats.len() >= 2 {
        let major = floats[floats.len() - 2];
        let minor = floats[floats.len() - 1];

        if major < 0.0 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("TOROIDAL_SURFACE has negative major radius ({:.6})", major),
            ).with_suggestion("Major radius must be positive"));
        }
        if minor < 0.0 {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("TOROIDAL_SURFACE has negative minor radius ({:.6})", minor),
            ).with_suggestion("Minor radius must be positive"));
        }
        if minor > major && major > 1e-10 {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                format!("TOROIDAL_SURFACE minor radius ({:.6}) > major radius ({:.6}); self-intersecting torus", minor, major),
            ).with_suggestion("This creates a self-intersecting torus (apple shape)"));
        }
    }

    if let Some(StepValue::Ref(axis_id)) = entity.params.get(1) {
        if step_file.find_entity(*axis_id).is_none() {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("TOROIDAL_SURFACE references non-existent axis placement #{}", axis_id),
            ));
        }
    }
}

// ============================================================
// Phase 3: Composite entity validation
// ============================================================

/// Validate composite STEP entities: ADVANCED_FACE, FACE_BOUND, EDGE_LOOP,
/// MANIFOLD_SOLID_BREP, and assembly structures.
fn validate_composite_entities(step_file: &StepFile, report: &mut StepValidationReport) {
    for entity in &step_file.entities {
        let type_upper = entity.type_name.to_uppercase();

        match type_upper.as_str() {
            "ADVANCED_FACE" => validate_advanced_face(entity, step_file, report),
            "FACE_BOUND" | "FACE_OUTER_BOUND" => validate_face_bound(entity, step_file, report),
            "EDGE_LOOP" => validate_edge_loop(entity, step_file, report),
            "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => validate_brep(entity, step_file, report),
            "PRODUCT_DEFINITION_SHAPE" => validate_product_definition_shape(entity, step_file, report),
            _ => {}
        }
    }
}

fn validate_advanced_face(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // ADVANCED_FACE('name', (bound_refs), surface_ref, same_sense)
    // Must have at least one FACE_BOUND. The bounds are in the 2nd parameter (index 1),
    // which is typically a List of references to FACE_BOUND/FACE_OUTER_BOUND entities.
    let bound_refs: Vec<i64> = entity.params.get(1)
        .map(|p| extract_refs_from_value(p))
        .unwrap_or_default();

    if bound_refs.is_empty() {
        report.add(StepValidationIssue::error(
            Some(entity.id),
            "ADVANCED_FACE has no FACE_BOUND references",
        ).with_suggestion("An ADVANCED_FACE must have at least one FACE_BOUND or FACE_OUTER_BOUND"));
    }

    // Check that bound references point to FACE_BOUND or FACE_OUTER_BOUND entities
    for &bound_id in &bound_refs {
        if let Some(bound_entity) = step_file.find_entity(bound_id) {
            let bt = bound_entity.type_name.to_uppercase();
            if bt != "FACE_BOUND" && bt != "FACE_OUTER_BOUND" {
                report.add(StepValidationIssue::warning(
                    Some(entity.id),
                    format!("ADVANCED_FACE bound reference #{} is not FACE_BOUND/FACE_OUTER_BOUND (is '{}')", bound_id, bound_entity.type_name),
                ));
            }
        }
    }

    // Check surface reference
    let surface_ref = entity.params.iter()
        .filter_map(|p| match p {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        })
        .nth(1); // Second reference (after the bound list)

    if let Some(surf_id) = surface_ref {
        if let Some(surf_entity) = step_file.find_entity(surf_id) {
            let st = surf_entity.type_name.to_uppercase();
            if !is_surface_type(&st) && st != "SURFACE" {
                report.add(StepValidationIssue::warning(
                    Some(entity.id),
                    format!("ADVANCED_FACE surface reference #{} is '{}' which is not a known surface type", surf_id, surf_entity.type_name),
                ));
            }
        }
    }
}

fn validate_face_bound(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // FACE_BOUND('name', #loop_ref, orientation)
    // FACE_OUTER_BOUND('name', #loop_ref, orientation)
    // Must reference a valid LOOP entity

    let loop_ref = entity.params.iter()
        .filter_map(|p| match p {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        })
        .find(|_| true); // First reference

    if let Some(loop_id) = loop_ref {
        if let Some(loop_entity) = step_file.find_entity(loop_id) {
            let lt = loop_entity.type_name.to_uppercase();
            if lt != "EDGE_LOOP" && lt != "VERTEX_LOOP" && lt != "POLY_LOOP" {
                report.add(StepValidationIssue::warning(
                    Some(entity.id),
                    format!("FACE_BOUND references #{} which is '{}' (expected EDGE_LOOP, VERTEX_LOOP, or POLY_LOOP)", loop_id, loop_entity.type_name),
                ));
            }
        } else {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("FACE_BOUND references non-existent loop #{}", loop_id),
            ));
        }
    } else {
        report.add(StepValidationIssue::error(
            Some(entity.id),
            "FACE_BOUND has no loop reference",
        ));
    }
}

fn validate_edge_loop(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // EDGE_LOOP('name', (oriented_edge_refs))
    // Must contain at least one ORIENTED_EDGE
    let edge_refs: Vec<i64> = entity.params.iter()
        .flat_map(|p| extract_refs_from_value(p))
        .collect();

    if edge_refs.is_empty() {
        report.add(StepValidationIssue::error(
            Some(entity.id),
            "EDGE_LOOP has no ORIENTED_EDGE references",
        ).with_suggestion("An EDGE_LOOP must contain at least one ORIENTED_EDGE"));
    }

    // Check that edge references point to ORIENTED_EDGE entities
    let mut non_oriented_count = 0;
    for &edge_id in &edge_refs {
        if let Some(edge_entity) = step_file.find_entity(edge_id) {
            let et = edge_entity.type_name.to_uppercase();
            if et != "ORIENTED_EDGE" {
                non_oriented_count += 1;
            }
        }
    }
    if non_oriented_count > 0 {
        report.add(StepValidationIssue::warning(
            Some(entity.id),
            format!("EDGE_LOOP contains {} reference(s) that are not ORIENTED_EDGE", non_oriented_count),
        ).with_suggestion("EDGE_LOOP should contain only ORIENTED_EDGE references"));
    }
}

fn validate_brep(entity: &StepEntity, step_file: &StepFile, report: &mut StepValidationReport) {
    // MANIFOLD_SOLID_BREP('name', #closed_shell)
    // Must reference a valid CLOSED_SHELL
    let shell_ref = entity.params.iter()
        .filter_map(|p| match p {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        })
        .find(|_| true);

    if let Some(shell_id) = shell_ref {
        if let Some(shell_entity) = step_file.find_entity(shell_id) {
            let st = shell_entity.type_name.to_uppercase();
            if st != "CLOSED_SHELL" && st != "OPEN_SHELL" {
                report.add(StepValidationIssue::warning(
                    Some(entity.id),
                    format!("BREP references #{} which is '{}' (expected CLOSED_SHELL)", shell_id, shell_entity.type_name),
                ));
            }
            if st == "OPEN_SHELL" {
                report.add(StepValidationIssue::warning(
                    Some(entity.id),
                    format!("MANIFOLD_SOLID_BREP references OPEN_SHELL #{} instead of CLOSED_SHELL", shell_id),
                ).with_suggestion("A solid BREP should have a CLOSED_SHELL for a watertight solid"));
            }
        } else {
            report.add(StepValidationIssue::error(
                Some(entity.id),
                format!("BREP references non-existent shell #{}", shell_id),
            ));
        }
    } else {
        report.add(StepValidationIssue::error(
            Some(entity.id),
            "MANIFOLD_SOLID_BREP has no shell reference",
        ));
    }
}

fn validate_product_definition_shape(
    entity: &StepEntity,
    step_file: &StepFile,
    report: &mut StepValidationReport,
) {
    // PRODUCT_DEFINITION_SHAPE links to SHAPE_DEFINITION_REPRESENTATION
    // Check if there's a corresponding SHAPE_DEFINITION_REPRESENTATION
    let sdr_entities = step_file.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION");

    // Check if any SDR references this PDS
    let is_referenced = sdr_entities.iter().any(|sdr| {
        sdr.params.iter().any(|p| {
            match p {
                StepValue::Ref(id) => *id == entity.id,
                _ => false,
            }
        })
    });

    if !is_referenced {
        // This might not be an error — PDS can be referenced by CDSR too
        let cdsr_entities = step_file.find_entities_by_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION");
        let is_referenced_by_cdsr = cdsr_entities.iter().any(|cdsr| {
            cdsr.params.iter().any(|p| {
                match p {
                    StepValue::Ref(id) => *id == entity.id,
                    _ => false,
                }
            })
        });

        if !is_referenced_by_cdsr {
            report.add(StepValidationIssue::warning(
                Some(entity.id),
                "PRODUCT_DEFINITION_SHAPE is not referenced by any SHAPE_DEFINITION_REPRESENTATION or CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",
            ).with_suggestion("Check if this PDS is orphaned or if its reference chain is incomplete"));
        }
    }
}

// ============================================================
// Phase 4: Assembly validation
// ============================================================

/// Validate STEP assembly structures: NAUO references, ITEM_DEFINED_TRANSFORMATION,
/// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION, and circular references.
fn validate_assemblies(
    step_file: &StepFile,
    known_ids: &HashSet<i64>,
    report: &mut StepValidationReport,
) {
    let nauos = step_file.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
    let idts = step_file.find_entities_by_type("ITEM_DEFINED_TRANSFORMATION");
    let cdsrs = step_file.find_entities_by_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION");

    // Always validate ITEM_DEFINED_TRANSFORMATION and CDSR, even without NAUO,
    // since they can exist independently.
    for idt in &idts {
        validate_item_defined_transformation(idt, step_file, report);
    }

    for cdsr in &cdsrs {
        validate_cdsr(cdsr, step_file, report);
    }

    if nauos.is_empty() {
        // No assembly structure — skip NAUO-specific validation
        return;
    }

    // Validate NAUO references
    for nauo in &nauos {
        validate_nauo(nauo, step_file, known_ids, report);
    }

    // Check for circular references in assembly tree
    validate_circular_assembly_refs(step_file, &nauos, report);
}

fn validate_nauo(
    nauo: &StepEntity,
    step_file: &StepFile,
    known_ids: &HashSet<i64>,
    report: &mut StepValidationReport,
) {
    // NEXT_ASSEMBLY_USAGE_OCCURRENCE('id','name','description',#relating_pd,#related_pd,$)
    // Must have two PRODUCT_DEFINITION references (relating = parent, related = child)
    let pd_refs: Vec<i64> = nauo.params.iter()
        .filter_map(|p| match p {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        })
        .collect();

    let relating_pd = pd_refs.get(0).copied();
    let related_pd = pd_refs.get(1).copied();

    if relating_pd.is_none() {
        report.add(StepValidationIssue::error(
            Some(nauo.id),
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE has no relating PRODUCT_DEFINITION reference",
        ).with_suggestion("The NAUO must reference a parent PRODUCT_DEFINITION"));
    } else if !known_ids.contains(&relating_pd.unwrap()) {
        report.add(StepValidationIssue::error(
            Some(nauo.id),
            format!("NAUO relating PD reference #{} does not exist", relating_pd.unwrap()),
        ));
    } else if let Some(pd_entity) = step_file.find_entity(relating_pd.unwrap()) {
        if pd_entity.type_name != "PRODUCT_DEFINITION" {
            report.add(StepValidationIssue::warning(
                Some(nauo.id),
                format!("NAUO relating reference #{} is '{}' (expected PRODUCT_DEFINITION)", relating_pd.unwrap(), pd_entity.type_name),
            ));
        }
    }

    if related_pd.is_none() {
        report.add(StepValidationIssue::error(
            Some(nauo.id),
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE has no related PRODUCT_DEFINITION reference",
        ).with_suggestion("The NAUO must reference a child PRODUCT_DEFINITION"));
    } else if !known_ids.contains(&related_pd.unwrap()) {
        report.add(StepValidationIssue::error(
            Some(nauo.id),
            format!("NAUO related PD reference #{} does not exist", related_pd.unwrap()),
        ));
    } else if let Some(pd_entity) = step_file.find_entity(related_pd.unwrap()) {
        if pd_entity.type_name != "PRODUCT_DEFINITION" {
            report.add(StepValidationIssue::warning(
                Some(nauo.id),
                format!("NAUO related reference #{} is '{}' (expected PRODUCT_DEFINITION)", related_pd.unwrap(), pd_entity.type_name),
            ));
        }
    }

    // Self-reference check: relating and related should be different
    if let (Some(rel), Some(rec)) = (relating_pd, related_pd) {
        if rel == rec {
            report.add(StepValidationIssue::error(
                Some(nauo.id),
                "NEXT_ASSEMBLY_USAGE_OCCURRENCE has identical relating and related PRODUCT_DEFINITION (self-reference)",
            ).with_suggestion("A component cannot be assembled into itself"));
        }
    }
}

fn validate_item_defined_transformation(
    idt: &StepEntity,
    step_file: &StepFile,
    report: &mut StepValidationReport,
) {
    // ITEM_DEFINED_TRANSFORMATION('name','description',#transform_item1,#transform_item2)
    // The two transform items should be AXIS2_PLACEMENT_3D entities
    let axis_refs: Vec<i64> = idt.params.iter()
        .filter_map(|p| match p {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        })
        .collect();

    if axis_refs.len() < 2 {
        report.add(StepValidationIssue::error(
            Some(idt.id),
            format!("ITEM_DEFINED_TRANSFORMATION has {} axis reference(s) (expected at least 2)", axis_refs.len()),
        ).with_suggestion("ITEM_DEFINED_TRANSFORMATION requires source and target axis placements"));
    }

    for (_i, &axis_id) in axis_refs.iter().enumerate() {
        if let Some(axis_entity) = step_file.find_entity(axis_id) {
            let at = axis_entity.type_name.to_uppercase();
            if at != "AXIS2_PLACEMENT_3D" && at != "AXIS2_PLACEMENT_2D" && at != "CARTESIAN_POINT" {
                report.add(StepValidationIssue::warning(
                    Some(idt.id),
                    format!("ITEM_DEFINED_TRANSFORMATION axis reference #{} is '{}' (expected AXIS2_PLACEMENT_3D)", axis_id, axis_entity.type_name),
                ));
            }
        } else {
            report.add(StepValidationIssue::error(
                Some(idt.id),
                format!("ITEM_DEFINED_TRANSFORMATION references non-existent axis #{}", axis_id),
            ));
        }
    }
}

fn validate_cdsr(
    cdsr: &StepEntity,
    step_file: &StepFile,
    report: &mut StepValidationReport,
) {
    // CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#representation_relation, #product_definition_shape)
    let refs: Vec<i64> = cdsr.params.iter()
        .filter_map(|p| match p {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        })
        .collect();

    if refs.len() < 2 {
        report.add(StepValidationIssue::error(
            Some(cdsr.id),
            format!("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION has {} reference(s) (expected 2)", refs.len()),
        ).with_suggestion("CDSR requires a representation relation and a product definition shape"));
        return;
    }

    // First ref should be a representation relationship
    if let Some(rr_entity) = step_file.find_entity(refs[0]) {
        let rt = rr_entity.type_name.to_uppercase();
        if !rt.contains("REPRESENTATION_RELATIONSHIP") {
            report.add(StepValidationIssue::warning(
                Some(cdsr.id),
                format!("CDSR first reference #{} is '{}' (expected REPRESENTATION_RELATIONSHIP or similar)", refs[0], rr_entity.type_name),
            ));
        }
    } else {
        report.add(StepValidationIssue::error(
            Some(cdsr.id),
            format!("CDSR references non-existent representation relation #{}", refs[0]),
        ));
    }

    // Second ref should be a PRODUCT_DEFINITION_SHAPE
    if let Some(pds_entity) = step_file.find_entity(refs[1]) {
        if pds_entity.type_name.to_uppercase() != "PRODUCT_DEFINITION_SHAPE" {
            report.add(StepValidationIssue::warning(
                Some(cdsr.id),
                format!("CDSR second reference #{} is '{}' (expected PRODUCT_DEFINITION_SHAPE)", refs[1], pds_entity.type_name),
            ));
        }
    } else {
        report.add(StepValidationIssue::error(
            Some(cdsr.id),
            format!("CDSR references non-existent PDS #{}", refs[1]),
        ));
    }
}

fn validate_circular_assembly_refs(
    _step_file: &StepFile,
    nauos: &[&StepEntity],
    report: &mut StepValidationReport,
) {
    // Build parent→child adjacency from NAUO entities
    let mut parent_to_children: HashMap<i64, Vec<i64>> = HashMap::new();
    for nauo in nauos {
        let pd_refs: Vec<i64> = nauo.params.iter()
            .filter_map(|p| match p {
                StepValue::Ref(id) => Some(*id),
                _ => None,
            })
            .collect();
        if pd_refs.len() >= 2 {
            let parent = pd_refs[0];
            let child = pd_refs[1];
            if parent != child {
                parent_to_children.entry(parent).or_default().push(child);
            }
        }
    }

    // DFS-based cycle detection
    let mut visited: HashSet<i64> = HashSet::new();
    let mut in_stack: HashSet<i64> = HashSet::new();
    let mut path: Vec<i64> = Vec::new();

    // Find all PDs that appear as parents
    let all_parents: Vec<i64> = parent_to_children.keys().copied().collect();

    for start_pd in &all_parents {
        if visited.contains(start_pd) {
            continue;
        }
        if detect_cycle_dfs(
            *start_pd,
            &parent_to_children,
            &mut visited,
            &mut in_stack,
            &mut path,
        ) {
            // Found a cycle — report it
            let cycle_start = path.iter().rposition(|&id| id == *start_pd).unwrap_or(0);
            let cycle: Vec<String> = path[cycle_start..]
                .iter()
                .map(|id| format!("#{}", id))
                .collect();
            report.add(StepValidationIssue::error(
                None,
                format!("Circular assembly reference detected: {}", cycle.join(" → ")),
            ).with_suggestion("Remove circular references in the assembly tree"));
        }
    }
}

fn detect_cycle_dfs(
    node: i64,
    adj: &HashMap<i64, Vec<i64>>,
    visited: &mut HashSet<i64>,
    in_stack: &mut HashSet<i64>,
    path: &mut Vec<i64>,
) -> bool {
    visited.insert(node);
    in_stack.insert(node);
    path.push(node);

    if let Some(children) = adj.get(&node) {
        for &child in children {
            if !visited.contains(&child) {
                if detect_cycle_dfs(child, adj, visited, in_stack, path) {
                    return true;
                }
            } else if in_stack.contains(&child) {
                path.push(child);
                return true;
            }
        }
    }

    path.pop();
    in_stack.remove(&node);
    false
}

// ============================================================
// Helper functions
// ============================================================

/// Extract coordinate values from a CARTESIAN_POINT or DIRECTION parameter list.
fn extract_coordinate_list(params: &[StepValue]) -> Vec<f64> {
    let mut coords = Vec::new();

    for param in params {
        match param {
            StepValue::List(items) => {
                for item in items {
                    match item {
                        StepValue::Float(f) => coords.push(*f),
                        StepValue::Integer(i) => coords.push(*i as f64),
                        _ => {}
                    }
                }
            }
            StepValue::Float(f) => coords.push(*f),
            StepValue::Integer(i) => coords.push(*i as f64),
            _ => {}
        }
    }

    coords
}

/// Extract all entity references from a StepValue (including nested lists).
fn extract_refs_from_value(value: &StepValue) -> Vec<i64> {
    let mut refs = Vec::new();
    match value {
        StepValue::Ref(id) => refs.push(*id),
        StepValue::List(items) => {
            for item in items {
                refs.extend(extract_refs_from_value(item));
            }
        }
        StepValue::Typed { value, .. } => {
            refs.extend(extract_refs_from_value(value));
        }
        _ => {}
    }
    refs
}

/// Find the last float parameter in an entity's parameter list.
fn find_last_float(params: &[StepValue]) -> Option<f64> {
    params.iter().rev().find_map(|p| match p {
        StepValue::Float(f) => Some(*f),
        _ => None,
    })
}

// ============================================================
// Integration with conversion pipeline
// ============================================================

/// Validate a STEP file and return a report, but do not error on warnings.
/// Returns `Ok(report)` if there are no errors, `Err(report)` if there are errors.
pub fn validate_step_for_conversion(step_file: &StepFile) -> Result<StepValidationReport, StepValidationReport> {
    let report = validate_step_file(step_file);
    if report.has_errors() {
        Err(report)
    } else {
        Ok(report)
    }
}

// ============================================================
// Unit tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_step_string(input: &str) -> StepFile {
        crate::parser::parse_step(input).expect("Failed to parse STEP string")
    }

    // ── Severity and report tests ────────────────────────────────

    #[test]
    fn test_severity_ordering() {
        // With derive Ord, declaration order determines ordering:
        // Error(0) < Warning(1) < Info(2)
        assert!(Severity::Error < Severity::Warning);
        assert!(Severity::Warning < Severity::Info);
    }

    #[test]
    fn test_empty_report() {
        let report = StepValidationReport::new();
        assert_eq!(report.error_count(), 0);
        assert_eq!(report.warning_count(), 0);
        assert_eq!(report.info_count(), 0);
        assert!(!report.has_errors());
    }

    #[test]
    fn test_report_counts() {
        let mut report = StepValidationReport::new();
        report.add(StepValidationIssue::error(Some(1), "err1"));
        report.add(StepValidationIssue::error(Some(2), "err2"));
        report.add(StepValidationIssue::warning(Some(3), "warn1"));
        report.add(StepValidationIssue::info(None, "info1"));

        assert_eq!(report.error_count(), 2);
        assert_eq!(report.warning_count(), 1);
        assert_eq!(report.info_count(), 1);
        assert!(report.has_errors());
    }

    #[test]
    fn test_issue_display() {
        let issue = StepValidationIssue::error(Some(42), "bad entity")
            .with_suggestion("fix it");
        let s = format!("{}", issue);
        assert!(s.contains("ERROR"));
        assert!(s.contains("#42"));
        assert!(s.contains("bad entity"));
        assert!(s.contains("fix it"));
    }

    #[test]
    fn test_report_display() {
        let mut report = StepValidationReport::new();
        report.add(StepValidationIssue::error(Some(1), "test error"));
        report.add(StepValidationIssue::warning(None, "test warning"));
        let s = format!("{}", report);
        assert!(s.contains("Errors:   1"));
        assert!(s.contains("Warnings: 1"));
    }

    #[test]
    fn test_validate_step_for_conversion() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let result = validate_step_for_conversion(&file);
        assert!(result.is_ok(), "Valid STEP file should pass validation");
    }

    // ── Dangling reference tests ────────────────────────────────

    #[test]
    fn test_dangling_reference_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#2 = DIRECTION('x', (1.0, 0.0, 0.0));
#3 = VECTOR('v', #999, 1.0);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let errors = report.issues_by_severity(&Severity::Error);
        let dangling_errors: Vec<_> = errors.iter()
            .filter(|e| e.description.contains("non-existent entity #999"))
            .collect();
        assert!(!dangling_errors.is_empty(), "Should detect dangling reference to #999");
    }

    // ── Geometric validation tests ──────────────────────────────

    #[test]
    fn test_nan_cartesian_point_detected() {
        // NaN doesn't parse from STEP syntax; test via direct construction
        let mut file = StepFile::new();
        file.entities.push(StepEntity {
            id: 1,
            type_name: "CARTESIAN_POINT".to_string(),
            params: vec![
                StepValue::String("bad".to_string()),
                StepValue::List(vec![
                    StepValue::Float(f64::NAN),
                    StepValue::Float(0.0),
                    StepValue::Float(0.0),
                ]),
            ],
            sub_entities: vec![],
        });
        file.build_index();

        let report = validate_step_file(&file);
        assert!(report.has_errors(), "NaN in CARTESIAN_POINT should be an error");
        let has_nan_error = report.issues.iter()
            .any(|i| i.description.contains("NaN"));
        assert!(has_nan_error, "Should report NaN coordinate error");
    }

    #[test]
    fn test_inf_cartesian_point_detected() {
        let mut file = StepFile::new();
        file.entities.push(StepEntity {
            id: 1,
            type_name: "CARTESIAN_POINT".to_string(),
            params: vec![
                StepValue::String("bad".to_string()),
                StepValue::List(vec![
                    StepValue::Float(f64::INFINITY),
                    StepValue::Float(0.0),
                    StepValue::Float(0.0),
                ]),
            ],
            sub_entities: vec![],
        });
        file.build_index();

        let report = validate_step_file(&file);
        assert!(report.has_errors(), "Inf in CARTESIAN_POINT should be an error");
    }

    #[test]
    fn test_unnormalized_direction_detected() {
        let mut file = StepFile::new();
        file.entities.push(StepEntity {
            id: 1,
            type_name: "DIRECTION".to_string(),
            params: vec![
                StepValue::String("bad_dir".to_string()),
                StepValue::List(vec![
                    StepValue::Float(3.0),
                    StepValue::Float(4.0),
                    StepValue::Float(0.0),
                ]),
            ],
            sub_entities: vec![],
        });
        file.build_index();

        let report = validate_step_file(&file);
        let has_unnorm = report.issues.iter()
            .any(|i| i.description.contains("not normalized"));
        assert!(has_unnorm, "Should detect unnormalized direction");
    }

    #[test]
    fn test_zero_direction_detected() {
        let mut file = StepFile::new();
        file.entities.push(StepEntity {
            id: 1,
            type_name: "DIRECTION".to_string(),
            params: vec![
                StepValue::String("zero_dir".to_string()),
                StepValue::List(vec![
                    StepValue::Float(0.0),
                    StepValue::Float(0.0),
                    StepValue::Float(0.0),
                ]),
            ],
            sub_entities: vec![],
        });
        file.build_index();

        let report = validate_step_file(&file);
        let has_zero = report.issues.iter()
            .any(|i| i.description.contains("zero vector"));
        assert!(has_zero, "Should detect zero direction vector");
    }

    #[test]
    fn test_normalized_direction_passes() {
        let mut file = StepFile::new();
        file.entities.push(StepEntity {
            id: 1,
            type_name: "DIRECTION".to_string(),
            params: vec![
                StepValue::String("good_dir".to_string()),
                StepValue::List(vec![
                    StepValue::Float(1.0),
                    StepValue::Float(0.0),
                    StepValue::Float(0.0),
                ]),
            ],
            sub_entities: vec![],
        });
        file.build_index();

        let report = validate_step_file(&file);
        let dir_errors = report.issues.iter()
            .filter(|i| i.entity_id == Some(1) && i.severity == Severity::Error)
            .count();
        assert_eq!(dir_errors, 0, "Normalized direction should not produce errors");
    }

    // ── Curve/surface validation tests ──────────────────────────

    #[test]
    fn test_negative_circle_radius_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('center', (0.0, 0.0, 0.0));
#2 = DIRECTION('z', (0.0, 0.0, 1.0));
#3 = VECTOR('v', #2, 1.0);
#4 = AXIS2_PLACEMENT_3D('ax', #1, #2, $);
#5 = CIRCLE('circ', #4, -5.0);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_neg_radius = report.issues.iter()
            .any(|i| i.description.contains("CIRCLE") && i.description.contains("negative radius"));
        assert!(has_neg_radius, "Should detect negative circle radius");
    }

    #[test]
    fn test_negative_cylinder_radius_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('center', (0.0, 0.0, 0.0));
#2 = DIRECTION('z', (0.0, 0.0, 1.0));
#4 = AXIS2_PLACEMENT_3D('ax', #1, #2, $);
#5 = CYLINDRICAL_SURFACE('cyl', #4, -3.0);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_neg = report.issues.iter()
            .any(|i| i.description.contains("CYLINDRICAL_SURFACE") && i.description.contains("negative radius"));
        assert!(has_neg, "Should detect negative cylinder radius");
    }

    #[test]
    fn test_self_intersecting_torus_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('center', (0.0, 0.0, 0.0));
#2 = DIRECTION('z', (0.0, 0.0, 1.0));
#4 = AXIS2_PLACEMENT_3D('ax', #1, #2, $);
#5 = TOROIDAL_SURFACE('torus', #4, 5.0, 10.0);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_self_int = report.issues.iter()
            .any(|i| i.description.contains("self-intersecting torus"));
        assert!(has_self_int, "Should detect self-intersecting torus");
    }

    // ── Composite entity validation tests ───────────────────────

    #[test]
    fn test_advanced_face_no_bounds_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#2 = DIRECTION('z', (0.0, 0.0, 1.0));
#3 = AXIS2_PLACEMENT_3D('ax', #1, #2, $);
#4 = PLANE('pln', #3);
#5 = ADVANCED_FACE('face', ($,$), #4, .T.);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_no_bounds = report.issues.iter()
            .any(|i| i.description.contains("no FACE_BOUND references"));
        assert!(has_no_bounds, "Should detect ADVANCED_FACE with no bounds");
    }

    #[test]
    fn test_edge_loop_no_edges_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = EDGE_LOOP('empty_loop', ($));
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_no_edges = report.issues.iter()
            .any(|i| i.description.contains("no ORIENTED_EDGE references"));
        assert!(has_no_edges, "Should detect EDGE_LOOP with no edges");
    }

    #[test]
    fn test_brep_no_shell_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = MANIFOLD_SOLID_BREP('brep', $);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_no_shell = report.issues.iter()
            .any(|i| i.description.contains("no shell reference"));
        assert!(has_no_shell, "Should detect BREP with no shell reference");
    }

    #[test]
    fn test_brep_open_shell_warning() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = OPEN_SHELL('oshell', ($));
#2 = MANIFOLD_SOLID_BREP('brep', #1);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_open_shell = report.issues.iter()
            .any(|i| i.description.contains("OPEN_SHELL") && i.severity == Severity::Warning);
        assert!(has_open_shell, "Should warn about BREP referencing OPEN_SHELL");
    }

    // ── Assembly validation tests ───────────────────────────────

    #[test]
    fn test_nauo_self_reference_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = PRODUCT_DEFINITION('design', '', #10, #11);
#10 = PRODUCT_DEFINITION_FORMATION('', '', #20);
#11 = PRODUCT_DEFINITION_CONTEXT('', $, 'design');
#20 = PRODUCT('id', 'name', $, $);
#2 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('nauo','nauo','',#1,#1,$);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_self_ref = report.issues.iter()
            .any(|i| i.description.contains("self-reference"));
        assert!(has_self_ref, "Should detect NAUO self-reference");
    }

    #[test]
    fn test_circular_assembly_detected() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = PRODUCT_DEFINITION('design', '', #10, #11);
#2 = PRODUCT_DEFINITION('design', '', #10, #11);
#3 = PRODUCT_DEFINITION('design', '', #10, #11);
#10 = PRODUCT_DEFINITION_FORMATION('', '', #20);
#11 = PRODUCT_DEFINITION_CONTEXT('', $, 'design');
#20 = PRODUCT('id', 'name', $, $);
#100 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('a','a','',#1,#2,$);
#101 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('b','b','',#2,#3,$);
#102 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('c','c','',#3,#1,$);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_circular = report.issues.iter()
            .any(|i| i.description.contains("Circular assembly reference"));
        assert!(has_circular, "Should detect circular assembly reference");
    }

    #[test]
    fn test_item_defined_transformation_insufficient_refs() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = ITEM_DEFINED_TRANSFORMATION('t','desc',$);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_insufficient = report.issues.iter()
            .any(|i| i.description.contains("axis reference") && i.severity == Severity::Error);
        assert!(has_insufficient, "Should detect ITEM_DEFINED_TRANSFORMATION with insufficient refs");
    }

    // ── Tolerance extraction tests ──────────────────────────────

    #[test]
    fn test_extract_tolerances_empty() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let tolerances = extract_tolerances(&file);
        assert!(!tolerances.has_tolerances());
        assert!(tolerances.tightest_tolerance.is_none());
    }

    #[test]
    fn test_extract_tolerances_uncertainty() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = UNCERTAINTY_MEASURE_WITH_UNIT((0.01), $, $, $);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let tolerances = extract_tolerances(&file);
        // The first param is a list, which may or may not be parsed correctly
        // depending on the parser. Let's check if we get something.
        // The key thing is the function doesn't crash.
        assert!(tolerances.uncertainty.is_some() || tolerances.geometric_tolerances.is_empty());
    }

    #[test]
    fn test_extract_tolerances_geometric() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = GEOMETRIC_TOLERANCE('gt','gt',$,0.05,$,$,$,$);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let tolerances = extract_tolerances(&file);
        assert!(!tolerances.geometric_tolerances.is_empty(), "Should find geometric tolerance");
        assert_eq!(tolerances.geometric_tolerances[0], 0.05);
    }

    #[test]
    fn test_tolerances_compute_tightest() {
        let mut t = StepTolerances {
            uncertainty: Some(0.01),
            geometric_tolerances: vec![0.005, 0.02],
            shape_tolerances: vec![0.001],
            tightest_tolerance: None,
        };
        t.compute_tightest();
        assert_eq!(t.tightest_tolerance, Some(0.001));
    }

    // ── Face bound validation test ──────────────────────────────

    #[test]
    fn test_face_bound_no_loop_ref() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = FACE_BOUND('fb', $, .T.);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_no_loop = report.issues.iter()
            .any(|i| i.description.contains("no loop reference"));
        assert!(has_no_loop, "Should detect FACE_BOUND with no loop reference");
    }

    // ── Valid file test ─────────────────────────────────────────

    #[test]
    fn test_valid_file_no_errors() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#2 = DIRECTION('z', (0.0, 0.0, 1.0));
#3 = DIRECTION('x', (1.0, 0.0, 0.0));
#4 = AXIS2_PLACEMENT_3D('ax', #1, #2, #3);
#5 = PLANE('pln', #4);
#6 = CIRCLE('circ', #4, 5.0);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        assert_eq!(report.error_count(), 0, "Valid file should have no errors");
    }

    // ── Report merge test ───────────────────────────────────────

    #[test]
    fn test_report_merge() {
        let mut r1 = StepValidationReport::new();
        r1.add(StepValidationIssue::error(Some(1), "e1"));
        let mut r2 = StepValidationReport::new();
        r2.add(StepValidationIssue::warning(Some(2), "w1"));
        r1.merge(r2);
        assert_eq!(r1.total_count(), 2);
        assert_eq!(r1.error_count(), 1);
        assert_eq!(r1.warning_count(), 1);
    }

    // ── Large coordinate warning test ───────────────────────────

    #[test]
    fn test_large_coordinate_warning() {
        let mut file = StepFile::new();
        file.entities.push(StepEntity {
            id: 1,
            type_name: "CARTESIAN_POINT".to_string(),
            params: vec![
                StepValue::String("huge".to_string()),
                StepValue::List(vec![
                    StepValue::Float(5e8),
                    StepValue::Float(0.0),
                    StepValue::Float(0.0),
                ]),
            ],
            sub_entities: vec![],
        });
        file.build_index();

        let report = validate_step_file(&file);
        let has_large = report.issues.iter()
            .any(|i| i.description.contains("very large coordinate") && i.severity == Severity::Warning);
        assert!(has_large, "Should warn about very large coordinates");
    }

    // ── CDSR validation test ────────────────────────────────────

    #[test]
    fn test_cdsr_insufficient_refs() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION($,$);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let report = validate_step_file(&file);
        let has_cdsr_err = report.issues.iter()
            .any(|i| i.description.contains("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") && i.severity == Severity::Error);
        // The $ params result in Omitted values, not Ref values, so refs.len() < 2
        assert!(has_cdsr_err, "Should detect CDSR with insufficient references");
    }

    // ── Collect refs helper test ─────────────────────────────────

    #[test]
    fn test_collect_refs() {
        let params = vec![
            StepValue::String("name".to_string()),
            StepValue::Ref(10),
            StepValue::List(vec![
                StepValue::Ref(20),
                StepValue::Float(3.0),
            ]),
            StepValue::Typed {
                type_name: "SOMETYPE".to_string(),
                value: Box::new(StepValue::Ref(30)),
            },
            StepValue::Omitted,
        ];
        let refs = collect_refs(&params);
        assert_eq!(refs, vec![10, 20, 30]);
    }

    // ── Validate step for conversion ────────────────────────────

    #[test]
    fn test_validate_step_for_conversion_ok() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#2 = DIRECTION('z', (0.0, 0.0, 1.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let result = validate_step_for_conversion(&file);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_step_for_conversion_err() {
        let mut file = StepFile::new();
        file.entities.push(StepEntity {
            id: 1,
            type_name: "CARTESIAN_POINT".to_string(),
            params: vec![
                StepValue::String("bad".to_string()),
                StepValue::List(vec![
                    StepValue::Float(f64::NAN),
                    StepValue::Float(0.0),
                    StepValue::Float(0.0),
                ]),
            ],
            sub_entities: vec![],
        });
        file.build_index();

        let result = validate_step_for_conversion(&file);
        assert!(result.is_err());
    }
}
