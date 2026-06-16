// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! High-level GD&T (Geometric Dimensioning and Tolerancing) API for draper-core.
//!
//! This module wraps the low-level [`draper_mesh::gdt_check::GdtChecker`] and provides
//! document-level and assembly-level GD&T validation. While the mesh-level checker
//! operates on a single [`TriangleMesh`], this module orchestrates checks across
//! entire documents and assemblies, producing structured reports suitable for
//! inspection, quality assurance, and manufacturing sign-off.
//!
//! # Core Types
//!
//! - [`GdntStatus`] — Pass / Warning / Fail tri-state for individual checks
//! - [`GdntToleranceType`] — Enumeration of all 14 ASME Y14.5 tolerance types
//! - [`GdntTolerance`] — Specification of a single GD&T tolerance with datum references
//! - [`GdntFeatureReport`] — Per-feature GD&T result
//! - [`GdntReport`] — Comprehensive report for a document or assembly
//! - [`GdntChecker`] — High-level checker that drives validation
//!
//! # Presets
//!
//! Common tolerance sets are provided via [`GdntPreset`]:
//!
//! - [`GdntPreset::machining_general`] — IT7 general machining
//! - [`GdntPreset::precision_machining`] — IT5 precision
//! - [`GdntPreset::sheet_metal`] — sheet metal forming
//! - [`GdntPreset::casting`] — casting / foundry
//!
//! # Datum System
//!
//! [`DatumReferenceSystem`] establishes a datum alignment frame (A, B, C) from
//! mesh geometry, producing [`DatumResult`] with origin and orthonormal axes.
//!
//! # Batch Processing
//!
//! [`batch_check`] validates multiple meshes against the same tolerance set
//! in a single call, useful for production-line inspection.
//!
//! # Usage
//!
//! ```ignore
//! use draper_core::gdnt::{GdntChecker, GdntPreset, GdntReport};
//! use draper_core::document::Document;
//!
//! let doc = Document::new("bracket");
//! let checker = GdntChecker::new();
//! let report = checker.check_document(&doc);
//! println!("{}", report.summary());
//! ```

use crate::document::Document;
use crate::assembly::AssemblyNode;
use draper_mesh::{TriangleMesh, gdt_check::GdtChecker as MeshGdtChecker};
use draper_mesh::gdt_check::{ToleranceSpec, GdtCheckType, GdtCheckResult};
use draper_geometry::{Point3d, Vec3d};

use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};

// ============================================================
// GdntStatus — Pass / Warning / Fail
// ============================================================

/// Status of a GD&T tolerance check.
///
/// Three-valued logic that extends the mesh-level binary Pass/Fail
/// with an intermediate **Warning** state (deviation is within tolerance
/// but close to the limit, typically > 75 % of the tolerance zone).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GdntStatus {
    /// Feature is within tolerance.
    Pass,
    /// Feature is within tolerance but approaching the limit.
    Warning,
    /// Feature exceeds the specified tolerance.
    Fail,
}

impl fmt::Display for GdntStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GdntStatus::Pass => write!(f, "PASS"),
            GdntStatus::Warning => write!(f, "WARNING"),
            GdntStatus::Fail => write!(f, "FAIL"),
        }
    }
}

impl GdntStatus {
    /// Determine status from deviation relative to tolerance value.
    ///
    /// - `Pass` if deviation ≤ 75 % of tolerance
    /// - `Warning` if deviation > 75 % but ≤ 100 % of tolerance
    /// - `Fail` if deviation > tolerance
    fn from_deviation(actual: f64, tolerance: f64) -> Self {
        if actual.is_nan() || tolerance <= 0.0 {
            return GdntStatus::Fail;
        }
        if actual > tolerance {
            GdntStatus::Fail
        } else if actual > tolerance * 0.75 {
            GdntStatus::Warning
        } else {
            GdntStatus::Pass
        }
    }
}

// ============================================================
// GdntToleranceType — 14 ASME Y14.5 tolerance types
// ============================================================

/// Enumeration of all GD&T tolerance types per ASME Y14.5-2018.
///
/// Divided into the standard categories:
/// - **Form**: Flatness, Straightness, Circularity, Cylindricity
/// - **Orientation**: Parallelism, Perpendicularity, Angularity
/// - **Location**: Position, Concentricity, Symmetry
/// - **Runout**: Circular Runout, Total Runout
/// - **Profile**: Profile of a Line, Profile of a Surface
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GdntToleranceType {
    // Form tolerances
    /// Maximum deviation of a surface from a reference plane.
    Flatness,
    /// Maximum deviation of an element from a straight line.
    Straightness,
    /// Maximum deviation of a cross-section from a perfect circle.
    Circularity,
    /// Maximum deviation of a surface from a perfect cylinder.
    Cylindricity,

    // Orientation tolerances (require datum)
    /// Deviation of a feature from parallel to a datum.
    Parallelism,
    /// Deviation of a feature from 90° to a datum.
    Perpendicularity,
    /// Deviation of a feature from a specified angle to a datum.
    Angularity,

    // Location tolerances
    /// Deviation of a feature's location from its true position.
    Position,
    /// Deviation of an axis from a datum axis.
    Concentricity,
    /// Deviation of a feature's median plane from a datum axis.
    Symmetry,

    // Runout tolerances (require datum axis)
    /// Total deviation during one full rotation about a datum axis.
    CircularRunout,
    /// Total deviation across the entire surface during rotation.
    TotalRunout,

    // Profile tolerances
    /// Deviation of a line element from the true profile.
    ProfileLine,
    /// Deviation of a surface from the true profile.
    ProfileSurface,
}

impl fmt::Display for GdntToleranceType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GdntToleranceType::Flatness => write!(f, "Flatness"),
            GdntToleranceType::Straightness => write!(f, "Straightness"),
            GdntToleranceType::Circularity => write!(f, "Circularity"),
            GdntToleranceType::Cylindricity => write!(f, "Cylindricity"),
            GdntToleranceType::Parallelism => write!(f, "Parallelism"),
            GdntToleranceType::Perpendicularity => write!(f, "Perpendicularity"),
            GdntToleranceType::Angularity => write!(f, "Angularity"),
            GdntToleranceType::Position => write!(f, "Position"),
            GdntToleranceType::Concentricity => write!(f, "Concentricity"),
            GdntToleranceType::Symmetry => write!(f, "Symmetry"),
            GdntToleranceType::CircularRunout => write!(f, "Circular Runout"),
            GdntToleranceType::TotalRunout => write!(f, "Total Runout"),
            GdntToleranceType::ProfileLine => write!(f, "Profile of a Line"),
            GdntToleranceType::ProfileSurface => write!(f, "Profile of a Surface"),
        }
    }
}

impl GdntToleranceType {
    /// Whether this tolerance type requires at least one datum reference.
    pub fn requires_datum(&self) -> bool {
        matches!(
            self,
            GdntToleranceType::Parallelism
                | GdntToleranceType::Perpendicularity
                | GdntToleranceType::Angularity
                | GdntToleranceType::Position
                | GdntToleranceType::Concentricity
                | GdntToleranceType::Symmetry
                | GdntToleranceType::CircularRunout
                | GdntToleranceType::TotalRunout
        )
    }

    /// Category string for display purposes.
    pub fn category(&self) -> &'static str {
        match self {
            GdntToleranceType::Flatness
            | GdntToleranceType::Straightness
            | GdntToleranceType::Circularity
            | GdntToleranceType::Cylindricity => "Form",

            GdntToleranceType::Parallelism
            | GdntToleranceType::Perpendicularity
            | GdntToleranceType::Angularity => "Orientation",

            GdntToleranceType::Position
            | GdntToleranceType::Concentricity
            | GdntToleranceType::Symmetry => "Location",

            GdntToleranceType::CircularRunout
            | GdntToleranceType::TotalRunout => "Runout",

            GdntToleranceType::ProfileLine
            | GdntToleranceType::ProfileSurface => "Profile",
        }
    }

    /// Convert to the mesh-level [`GdtCheckType`].
    fn to_mesh_check_type(&self) -> GdtCheckType {
        match self {
            GdntToleranceType::Flatness => GdtCheckType::Flatness,
            GdntToleranceType::Straightness => GdtCheckType::Straightness,
            GdntToleranceType::Circularity => GdtCheckType::Circularity,
            GdntToleranceType::Cylindricity => GdtCheckType::Cylindricity,
            GdntToleranceType::Parallelism => GdtCheckType::Parallelism,
            GdntToleranceType::Perpendicularity => GdtCheckType::Perpendicularity,
            GdntToleranceType::Angularity => GdtCheckType::Angularity,
            GdntToleranceType::Position => GdtCheckType::Position,
            GdntToleranceType::CircularRunout => GdtCheckType::Runout,
            GdntToleranceType::TotalRunout => GdtCheckType::Runout,
            GdntToleranceType::Concentricity => {
                GdtCheckType::Unsupported("Concentricity".to_string())
            }
            GdntToleranceType::Symmetry => {
                GdtCheckType::Unsupported("Symmetry".to_string())
            }
            GdntToleranceType::ProfileLine => GdtCheckType::ProfileOfLine,
            GdntToleranceType::ProfileSurface => GdtCheckType::ProfileOfSurface,
        }
    }
}

// ============================================================
// GdntTolerance — specification of a single tolerance
// ============================================================

/// Specification of a GD&T tolerance to be checked.
///
/// Combines the tolerance type and value with optional datum references
/// and face selection. This is the input type for the high-level checker.
///
/// # Example
///
/// ```ignore
/// use draper_core::gdnt::{GdntTolerance, GdntToleranceType};
///
/// let flatness = GdntTolerance {
///     tolerance_type: GdntToleranceType::Flatness,
///     value: 0.05,
///     datum_references: vec![],
///     applies_to_faces: vec![1, 2, 3],
///     name: "Top face flatness".to_string(),
/// };
/// ```
#[derive(Clone, Debug)]
pub struct GdntTolerance {
    /// The type of GD&T tolerance.
    pub tolerance_type: GdntToleranceType,
    /// Tolerance zone width (in model units, typically mm).
    pub value: f64,
    /// Datum feature labels (e.g., "A", "B", "C").
    pub datum_references: Vec<String>,
    /// Face IDs this tolerance applies to. Empty means all faces.
    pub applies_to_faces: Vec<u64>,
    /// Human-readable name for this tolerance specification.
    pub name: String,
}

impl GdntTolerance {
    /// Create a new tolerance specification.
    ///
    /// # Arguments
    ///
    /// * `tolerance_type` — The kind of GD&T tolerance
    /// * `value` — Tolerance zone width in model units
    /// * `name` — Human-readable label
    pub fn new(tolerance_type: GdntToleranceType, value: f64, name: &str) -> Self {
        Self {
            tolerance_type,
            value,
            datum_references: Vec::new(),
            applies_to_faces: Vec::new(),
            name: name.to_string(),
        }
    }

    /// Add a datum reference to this tolerance.
    pub fn with_datum(mut self, datum: &str) -> Self {
        self.datum_references.push(datum.to_string());
        self
    }

    /// Restrict this tolerance to specific face IDs.
    pub fn with_faces(mut self, faces: Vec<u64>) -> Self {
        self.applies_to_faces = faces;
        self
    }

    /// Convert to a mesh-level [`ToleranceSpec`].
    fn to_tolerance_spec(&self) -> ToleranceSpec {
        ToleranceSpec {
            name: self.name.clone(),
            description: format!("{} (category: {})", self.tolerance_type, self.tolerance_type.category()),
            tolerance_type: self.tolerance_type.to_mesh_check_type(),
            tolerance_value: self.value,
            step_id: -1, // Not from STEP — generated from high-level API
            datum_references: self.datum_references.iter().map(|_| -1).collect(),
        }
    }
}

impl fmt::Display for GdntTolerance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {} ({:.4})", self.name, self.tolerance_type, self.value)?;
        if !self.datum_references.is_empty() {
            write!(f, " | Datum: {}", self.datum_references.join(", "))?;
        }
        Ok(())
    }
}

// ============================================================
// GdntFeatureReport — per-feature GD&T result
// ============================================================

/// GD&T check result for a single feature.
///
/// Contains the measured deviation, the pass/fail status, and metadata
/// about which faces and datums were involved.
#[derive(Clone, Debug)]
pub struct GdntFeatureReport {
    /// Name/label of the feature being checked.
    pub feature_name: String,
    /// Tolerance type (e.g., "Flatness", "Cylindricity").
    pub tolerance_type: String,
    /// The specified tolerance zone width.
    pub tolerance_value: f64,
    /// The measured actual deviation.
    pub actual_deviation: f64,
    /// Pass / Warning / Fail status.
    pub status: GdntStatus,
    /// Datum feature labels referenced by this tolerance.
    pub datum_references: Vec<String>,
    /// Mesh face IDs this feature applies to.
    pub face_ids: Vec<u64>,
}

impl GdntFeatureReport {
    /// Create a feature report from a mesh-level check result and tolerance spec.
    fn from_mesh_result(result: &GdtCheckResult, tolerance: &GdntTolerance) -> Self {
        let status = GdntStatus::from_deviation(result.actual_deviation, tolerance.value);

        Self {
            feature_name: tolerance.name.clone(),
            tolerance_type: tolerance.tolerance_type.to_string(),
            tolerance_value: tolerance.value,
            actual_deviation: result.actual_deviation,
            status,
            datum_references: tolerance.datum_references.clone(),
            face_ids: tolerance.applies_to_faces.clone(),
        }
    }

    /// The margin: how much tolerance remains (positive) or is exceeded (negative).
    pub fn margin(&self) -> f64 {
        self.tolerance_value - self.actual_deviation
    }

    /// Ratio of actual deviation to tolerance value (0.0–1.0+ if failed).
    pub fn utilization(&self) -> f64 {
        if self.tolerance_value > 0.0 {
            self.actual_deviation / self.tolerance_value
        } else {
            f64::INFINITY
        }
    }
}

impl fmt::Display for GdntFeatureReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}] {} — {}: tolerance={:.4}, actual={:.4}, margin={:.4} ({:.0}% utilized)",
            self.status,
            self.feature_name,
            self.tolerance_type,
            self.tolerance_value,
            self.actual_deviation,
            self.margin(),
            self.utilization() * 100.0,
        )
    }
}

// ============================================================
// GdntReport — comprehensive GD&T report for a document
// ============================================================

/// Comprehensive GD&T validation report for a document or assembly.
///
/// Aggregates per-feature results into an overall pass/fail determination
/// with counts and a timestamp.
#[derive(Clone, Debug)]
pub struct GdntReport {
    /// Name of the part or document.
    pub part_name: String,
    /// Per-feature GD&T results.
    pub results: Vec<GdntFeatureReport>,
    /// Overall Pass / Warning / Fail status.
    pub overall_status: GdntStatus,
    /// Number of features that passed.
    pub pass_count: usize,
    /// Number of features that failed.
    pub fail_count: usize,
    /// Number of features with warnings.
    pub warning_count: usize,
    /// ISO 8601 timestamp of when the report was generated.
    pub timestamp: String,
}

impl GdntReport {
    /// Create a new empty report for a given part.
    fn new(part_name: &str) -> Self {
        Self {
            part_name: part_name.to_string(),
            results: Vec::new(),
            overall_status: GdntStatus::Pass,
            pass_count: 0,
            fail_count: 0,
            warning_count: 0,
            timestamp: iso8601_now(),
        }
    }

    /// Add a feature report and update aggregate counts.
    fn add_result(&mut self, report: GdntFeatureReport) {
        match report.status {
            GdntStatus::Pass => self.pass_count += 1,
            GdntStatus::Warning => self.warning_count += 1,
            GdntStatus::Fail => self.fail_count += 1,
        }
        self.results.push(report);
        self.recompute_overall();
    }

    /// Recompute the overall status from individual results.
    ///
    /// The overall status is the worst individual status:
    /// - Any **Fail** → overall **Fail**
    /// - No Fail but any **Warning** → overall **Warning**
    /// - All **Pass** → overall **Pass**
    fn recompute_overall(&mut self) {
        if self.fail_count > 0 {
            self.overall_status = GdntStatus::Fail;
        } else if self.warning_count > 0 {
            self.overall_status = GdntStatus::Warning;
        } else {
            self.overall_status = GdntStatus::Pass;
        }
    }

    /// Total number of features checked.
    pub fn total_count(&self) -> usize {
        self.results.len()
    }

    /// Human-readable summary of the report.
    ///
    /// # Example output
    ///
    /// ```text
    /// GD&T Report: bracket
    /// Generated: 2026-03-04T12:34:56Z
    /// Overall: FAIL
    /// Features: 10 total (6 PASS, 2 WARNING, 2 FAIL)
    ///
    /// PASS  Top face flatness — Flatness: tol=0.0500, act=0.0123
    /// WARN  Side parallelism — Parallelism: tol=0.1000, act=0.0850
    /// FAIL  Hole position — Position: tol=0.0200, act=0.0310
    /// ```
    pub fn summary(&self) -> String {
        let mut lines = Vec::new();
        lines.push(format!("GD&T Report: {}", self.part_name));
        lines.push(format!("Generated: {}", self.timestamp));
        lines.push(format!("Overall: {}", self.overall_status));
        lines.push(format!(
            "Features: {} total ({} PASS, {} WARNING, {} FAIL)",
            self.total_count(),
            self.pass_count,
            self.warning_count,
            self.fail_count,
        ));
        lines.push(String::new());

        for report in &self.results {
            lines.push(format!("{}", report));
        }

        lines.join("\n")
    }

    /// JSON serialization (manual, no serde dependency).
    ///
    /// Produces a JSON object with all report fields. Nested
    /// `results` is a JSON array of feature report objects.
    pub fn to_json(&self) -> String {
        let mut json = String::new();
        json.push_str("{");
        json.push_str(&format!("\"part_name\":\"{}\",", escape_json(&self.part_name)));
        json.push_str(&format!("\"overall_status\":\"{}\",", self.overall_status));
        json.push_str(&format!("\"pass_count\":{},", self.pass_count));
        json.push_str(&format!("\"fail_count\":{},", self.fail_count));
        json.push_str(&format!("\"warning_count\":{},", self.warning_count));
        json.push_str(&format!("\"timestamp\":\"{}\",", escape_json(&self.timestamp)));
        json.push_str("\"results\":[");
        for (i, report) in self.results.iter().enumerate() {
            if i > 0 {
                json.push(',');
            }
            json.push_str(&report.to_json());
        }
        json.push_str("]}");
        json
    }
}

impl GdntFeatureReport {
    /// JSON serialization for a single feature report.
    fn to_json(&self) -> String {
        let mut json = String::new();
        json.push('{');
        json.push_str(&format!("\"feature_name\":\"{}\",", escape_json(&self.feature_name)));
        json.push_str(&format!("\"tolerance_type\":\"{}\",", escape_json(&self.tolerance_type)));
        json.push_str(&format!("\"tolerance_value\":{},", self.tolerance_value));
        json.push_str(&format!("\"actual_deviation\":{},", self.actual_deviation));
        json.push_str(&format!("\"status\":\"{}\",", self.status));
        json.push_str(&format!("\"margin\":{},", self.margin()));
        json.push_str(&format!("\"utilization\":{},", self.utilization()));
        json.push_str("\"datum_references\":[");
        for (i, datum) in self.datum_references.iter().enumerate() {
            if i > 0 {
                json.push(',');
            }
            json.push_str(&format!("\"{}\"", escape_json(datum)));
        }
        json.push_str("],\"face_ids\":[");
        for (i, fid) in self.face_ids.iter().enumerate() {
            if i > 0 {
                json.push(',');
            }
            json.push_str(&fid.to_string());
        }
        json.push_str("]}");
        json
    }
}

// ============================================================
// DatumResult / DatumReferenceSystem
// ============================================================

/// Result of establishing a datum from mesh geometry.
///
/// Provides an origin point and an orthonormal coordinate frame
/// aligned with the datum feature.
#[derive(Clone, Copy, Debug)]
pub struct DatumResult {
    /// Origin of the datum reference frame.
    pub origin: Point3d,
    /// X-axis of the datum frame.
    pub x_axis: Vec3d,
    /// Y-axis of the datum frame.
    pub y_axis: Vec3d,
    /// Z-axis (normal) of the datum frame.
    pub z_axis: Vec3d,
}

impl DatumResult {
    /// Create a default datum result at the origin with standard axes.
    pub fn identity() -> Self {
        Self {
            origin: Point3d::ORIGIN,
            x_axis: Vec3d::X,
            y_axis: Vec3d::Y,
            z_axis: Vec3d::Z,
        }
    }

    /// Verify that the axes form an orthonormal frame (within tolerance).
    pub fn is_orthonormal(&self) -> bool {
        let tol = 1e-6;
        let x_len = self.x_axis.length();
        let y_len = self.y_axis.length();
        let z_len = self.z_axis.length();

        if (x_len - 1.0).abs() > tol || (y_len - 1.0).abs() > tol || (z_len - 1.0).abs() > tol {
            return false;
        }
        if self.x_axis.dot(&self.y_axis).abs() > tol {
            return false;
        }
        if self.x_axis.dot(&self.z_axis).abs() > tol {
            return false;
        }
        if self.y_axis.dot(&self.z_axis).abs() > tol {
            return false;
        }
        true
    }
}

impl fmt::Display for DatumResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "DatumResult(origin={}, x={}, y={}, z={})",
            self.origin, self.x_axis, self.y_axis, self.z_axis
        )
    }
}

/// Datum reference system (DRF) for GD&T checks.
///
/// Establishes a coordinate frame from primary, secondary, and tertiary
/// datum features following ASME Y14.5 rules:
/// - **Primary datum** constrains 3 degrees of freedom (rotation + translation)
/// - **Secondary datum** constrains 2 additional DOF
/// - **Tertiary datum** constrains the final DOF
#[derive(Clone, Debug)]
pub struct DatumReferenceSystem {
    /// Primary datum feature label (e.g., "A").
    pub primary: Option<String>,
    /// Secondary datum feature label (e.g., "B").
    pub secondary: Option<String>,
    /// Tertiary datum feature label (e.g., "C").
    pub tertiary: Option<String>,
}

impl DatumReferenceSystem {
    /// Create an empty datum reference system (no datums defined).
    pub fn new() -> Self {
        Self {
            primary: None,
            secondary: None,
            tertiary: None,
        }
    }

    /// Create a datum reference system from labels.
    pub fn from_labels(primary: &str, secondary: Option<&str>, tertiary: Option<&str>) -> Self {
        Self {
            primary: Some(primary.to_string()),
            secondary: secondary.map(|s| s.to_string()),
            tertiary: tertiary.map(|s| s.to_string()),
        }
    }

    /// Establish a datum from a mesh face.
    ///
    /// Computes a coordinate frame whose Z-axis aligns with the face normal
    /// and whose origin is the face centroid. The X and Y axes are derived
    /// from the face's principal directions via PCA.
    ///
    /// # Arguments
    ///
    /// * `mesh` — The triangulated mesh containing the face
    /// * `datum_label` — Label for the datum (e.g., "A")
    /// * `face_id` — ID of the mesh face to use as the datum feature
    ///
    /// # Returns
    ///
    /// A [`DatumResult`] with origin and orthonormal axes, or a default
    /// identity frame if the face cannot be processed.
    pub fn establish_datum(
        &self,
        mesh: &TriangleMesh,
        datum_label: &str,
        face_id: u64,
    ) -> DatumResult {
        if mesh.vertices.is_empty() {
            return DatumResult::identity();
        }

        // Collect vertices belonging to the specified face.
        // If face_id doesn't match any triangle, use all vertices.
        let face_verts: Vec<Point3d> = self.collect_face_vertices(mesh, face_id);

        if face_verts.is_empty() {
            return DatumResult::identity();
        }

        // Compute centroid
        let n = face_verts.len() as f64;
        let cx = face_verts.iter().map(|p| p.x).sum::<f64>() / n;
        let cy = face_verts.iter().map(|p| p.y).sum::<f64>() / n;
        let cz = face_verts.iter().map(|p| p.z).sum::<f64>() / n;
        let origin = Point3d::new(cx, cy, cz);

        // Compute covariance matrix for PCA
        let mut xx = 0.0_f64;
        let mut xy = 0.0_f64;
        let mut xz = 0.0_f64;
        let mut yy = 0.0_f64;
        let mut yz = 0.0_f64;
        let mut zz = 0.0_f64;

        for p in &face_verts {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            xx += dx * dx;
            xy += dx * dy;
            xz += dx * dz;
            yy += dy * dy;
            yz += dy * dz;
            zz += dz * dz;
        }

        // Z-axis = normal direction (smallest eigenvector)
        let z_raw = smallest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);
        let z_len = (z_raw.0 * z_raw.0 + z_raw.1 * z_raw.1 + z_raw.2 * z_raw.2).sqrt();
        if z_len < 1e-15 {
            return DatumResult::identity();
        }
        let z_axis = Vec3d::new(z_raw.0 / z_len, z_raw.1 / z_len, z_raw.2 / z_len);

        // X-axis = direction of largest variance, projected onto the plane
        let x_raw = largest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);
        let x_len = (x_raw.0 * x_raw.0 + x_raw.1 * x_raw.1 + x_raw.2 * x_raw.2).sqrt();
        if x_len < 1e-15 {
            return DatumResult { origin, x_axis: Vec3d::X, y_axis: Vec3d::Y, z_axis };
        }
        let x_unit = Vec3d::new(x_raw.0 / x_len, x_raw.1 / x_len, x_raw.2 / x_len);
        // Remove component along Z
        let dot = x_unit.dot(&z_axis);
        let x_axis = Vec3d::new(
            x_unit.x - dot * z_axis.x,
            x_unit.y - dot * z_axis.y,
            x_unit.z - dot * z_axis.z,
        );
        let x_axis_len = x_axis.length();
        let x_axis = if x_axis_len > 1e-15 {
            Vec3d::new(x_axis.x / x_axis_len, x_axis.y / x_axis_len, x_axis.z / x_axis_len)
        } else {
            Vec3d::X
        };

        // Y-axis = Z × X (ensures right-handed frame)
        let y_axis = z_axis.cross(&x_axis);

        log::debug!(
            "Established datum '{}' from face {}: origin=({}, {}, {})",
            datum_label,
            face_id,
            origin.x,
            origin.y,
            origin.z,
        );

        DatumResult {
            origin,
            x_axis,
            y_axis,
            z_axis,
        }
    }

    /// Collect vertices that belong to a specific face ID.
    ///
    /// If the face_id is not found among triangles, all vertices are returned
    /// as a fallback (the mesh is treated as a single feature).
    fn collect_face_vertices(&self, mesh: &TriangleMesh, face_id: u64) -> Vec<Point3d> {
        let fid = face_id as usize;
        if fid < mesh.triangles.len() {
            let tri = &mesh.triangles[fid];
            vec![
                mesh.vertices[tri[0]],
                mesh.vertices[tri[1]],
                mesh.vertices[tri[2]],
            ]
        } else {
            // Fallback: use all vertices
            mesh.vertices.clone()
        }
    }
}

impl Default for DatumReferenceSystem {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// GdntPreset — preset tolerance sets
// ============================================================

/// Preset GD&T tolerance sets for common manufacturing processes.
///
/// These presets follow ISO 2768 and ASME Y14.5 conventions for
/// general tolerance grades. Values are in millimeters.
pub struct GdntPreset;

impl GdntPreset {
    /// General machining tolerances (IT7 / ISO 2768-mK).
    ///
    /// Suitable for standard CNC machining with moderate precision.
    /// - Flatness: 0.05 mm
    /// - Straightness: 0.05 mm
    /// - Circularity: 0.03 mm
    /// - Cylindricity: 0.05 mm
    /// - Parallelism: 0.05 mm (w.r.t. datum A)
    /// - Perpendicularity: 0.05 mm (w.r.t. datum A)
    /// - Position: 0.10 mm
    pub fn machining_general() -> Vec<GdntTolerance> {
        vec![
            GdntTolerance::new(GdntToleranceType::Flatness, 0.05, "Flatness — general machining")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Straightness, 0.05, "Straightness — general machining"),
            GdntTolerance::new(GdntToleranceType::Circularity, 0.03, "Circularity — general machining"),
            GdntTolerance::new(GdntToleranceType::Cylindricity, 0.05, "Cylindricity — general machining"),
            GdntTolerance::new(GdntToleranceType::Parallelism, 0.05, "Parallelism — general machining")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Perpendicularity, 0.05, "Perpendicularity — general machining")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Position, 0.10, "Position — general machining")
                .with_datum("A")
                .with_datum("B"),
        ]
    }

    /// Precision machining tolerances (IT5 / ISO 2768-fH).
    ///
    /// Suitable for high-precision CNC, grinding, and honing operations.
    /// - Flatness: 0.01 mm
    /// - Straightness: 0.01 mm
    /// - Circularity: 0.005 mm
    /// - Cylindricity: 0.01 mm
    /// - Parallelism: 0.01 mm (w.r.t. datum A)
    /// - Perpendicularity: 0.01 mm (w.r.t. datum A)
    /// - Position: 0.02 mm
    /// - Surface profile: 0.02 mm
    pub fn precision_machining() -> Vec<GdntTolerance> {
        vec![
            GdntTolerance::new(GdntToleranceType::Flatness, 0.01, "Flatness — precision machining")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Straightness, 0.01, "Straightness — precision machining"),
            GdntTolerance::new(GdntToleranceType::Circularity, 0.005, "Circularity — precision machining"),
            GdntTolerance::new(GdntToleranceType::Cylindricity, 0.01, "Cylindricity — precision machining"),
            GdntTolerance::new(GdntToleranceType::Parallelism, 0.01, "Parallelism — precision machining")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Perpendicularity, 0.01, "Perpendicularity — precision machining")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Position, 0.02, "Position — precision machining")
                .with_datum("A")
                .with_datum("B"),
            GdntTolerance::new(GdntToleranceType::ProfileSurface, 0.02, "Surface profile — precision machining")
                .with_datum("A"),
        ]
    }

    /// Sheet metal tolerances.
    ///
    /// Suitable for laser-cut, punched, and bent sheet metal parts.
    /// Relaxed flatness and position tolerances to account for
    /// springback and material behavior.
    /// - Flatness: 0.50 mm
    /// - Straightness: 0.30 mm
    /// - Parallelism: 0.30 mm (w.r.t. datum A)
    /// - Perpendicularity: 0.30 mm (w.r.t. datum A)
    /// - Position: 0.25 mm
    /// - Profile surface: 0.50 mm
    pub fn sheet_metal() -> Vec<GdntTolerance> {
        vec![
            GdntTolerance::new(GdntToleranceType::Flatness, 0.50, "Flatness — sheet metal")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Straightness, 0.30, "Straightness — sheet metal"),
            GdntTolerance::new(GdntToleranceType::Parallelism, 0.30, "Parallelism — sheet metal")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Perpendicularity, 0.30, "Perpendicularity — sheet metal")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Position, 0.25, "Position — sheet metal")
                .with_datum("A")
                .with_datum("B"),
            GdntTolerance::new(GdntToleranceType::ProfileSurface, 0.50, "Surface profile — sheet metal")
                .with_datum("A"),
        ]
    }

    /// Casting tolerances.
    ///
    /// Suitable for sand casting, investment casting, and die casting.
    /// Significantly relaxed tolerances to account for shrinkage,
    /// draft, and surface finish limitations.
    /// - Flatness: 1.00 mm
    /// - Straightness: 0.80 mm
    /// - Circularity: 0.50 mm
    /// - Cylindricity: 0.80 mm
    /// - Parallelism: 0.80 mm (w.r.t. datum A)
    /// - Perpendicularity: 0.80 mm (w.r.t. datum A)
    /// - Position: 1.00 mm
    /// - Profile surface: 1.50 mm
    pub fn casting() -> Vec<GdntTolerance> {
        vec![
            GdntTolerance::new(GdntToleranceType::Flatness, 1.00, "Flatness — casting")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Straightness, 0.80, "Straightness — casting"),
            GdntTolerance::new(GdntToleranceType::Circularity, 0.50, "Circularity — casting"),
            GdntTolerance::new(GdntToleranceType::Cylindricity, 0.80, "Cylindricity — casting"),
            GdntTolerance::new(GdntToleranceType::Parallelism, 0.80, "Parallelism — casting")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Perpendicularity, 0.80, "Perpendicularity — casting")
                .with_datum("A"),
            GdntTolerance::new(GdntToleranceType::Position, 1.00, "Position — casting")
                .with_datum("A")
                .with_datum("B"),
            GdntTolerance::new(GdntToleranceType::ProfileSurface, 1.50, "Surface profile — casting")
                .with_datum("A"),
        ]
    }
}

// ============================================================
// GdntChecker — high-level GD&T checker
// ============================================================

/// High-level GD&T checker that operates on documents and assemblies.
///
/// Wraps the mesh-level [`MeshGdtChecker`] and provides document-level
/// orchestration: triangulating solids, running checks per feature,
/// and aggregating results into a [`GdntReport`].
///
/// # Example
///
/// ```ignore
/// use draper_core::gdnt::{GdntChecker, GdntPreset};
/// use draper_core::document::Document;
///
/// let doc = Document::new("housing");
/// let checker = GdntChecker::new();
///
/// // Check the full document with a preset
/// let tolerances = GdntPreset::machining_general();
/// let report = checker.check_document(&doc);
///
/// // Or check a single mesh
/// let mesh = doc.triangulate();
/// let feature_report = checker.check_solid("housing", &mesh, &tolerances);
/// ```
pub struct GdntChecker;

impl GdntChecker {
    /// Create a new high-level GD&T checker.
    pub fn new() -> Self {
        Self
    }

    /// Check all solids in a document against default tolerances.
    ///
    /// Each solid in the document is triangulated and checked with
    /// the general machining preset. Results are aggregated into
    /// a single [`GdntReport`].
    ///
    /// # Arguments
    ///
    /// * `doc` — The CAD document to check
    ///
    /// # Returns
    ///
    /// A [`GdntReport`] containing results for all features across all solids.
    pub fn check_document(&self, doc: &Document) -> GdntReport {
        let tolerances = GdntPreset::machining_general();
        self.check_document_with_tolerances(doc, &tolerances)
    }

    /// Check all solids in a document with custom tolerances.
    ///
    /// # Arguments
    ///
    /// * `doc` — The CAD document to check
    /// * `tolerances` — Custom tolerance specifications
    pub fn check_document_with_tolerances(
        &self,
        doc: &Document,
        tolerances: &[GdntTolerance],
    ) -> GdntReport {
        let mut report = GdntReport::new(&doc.name);

        // Triangulate the document and check as a single mesh
        let mesh = doc.triangulate();

        if mesh.vertices.is_empty() {
            log::warn!("Document '{}' produced an empty mesh — skipping GD&T check", doc.name);
            return report;
        }

        for tolerance in tolerances {
            let feature_report = self.check_solid(&doc.name, &mesh, std::slice::from_ref(tolerance));
            // check_solid returns a single GdntFeatureReport, but we handle it
            // one at a time to avoid double-reporting
            report.add_result(feature_report);
        }

        report
    }

    /// Check a single solid (mesh) against a set of tolerances.
    ///
    /// Uses the mesh-level [`MeshGdtChecker`] internally. Each tolerance
    /// is checked independently and the result is mapped to a
    /// [`GdntFeatureReport`].
    ///
    /// # Arguments
    ///
    /// * `name` — Name/label for the solid
    /// * `mesh` — The triangulated mesh to check
    /// * `tolerances` — Tolerance specifications to check against
    ///
    /// # Returns
    ///
    /// A [`GdntFeatureReport`] for the first tolerance in the list.
    /// To check multiple tolerances, call this method for each one
    /// or use [`check_solid_all`].
    pub fn check_solid(
        &self,
        name: &str,
        mesh: &TriangleMesh,
        tolerances: &[GdntTolerance],
    ) -> GdntFeatureReport {
        let mesh_checker = MeshGdtChecker::new(mesh);

        // Use the first tolerance; for multiple, use check_solid_all
        if let Some(tolerance) = tolerances.first() {
            let spec = tolerance.to_tolerance_spec();
            let result = mesh_checker.check(&spec);
            GdntFeatureReport::from_mesh_result(&result, tolerance)
        } else {
            // No tolerances specified — return a passing default
            GdntFeatureReport {
                feature_name: name.to_string(),
                tolerance_type: "None".to_string(),
                tolerance_value: 0.0,
                actual_deviation: 0.0,
                status: GdntStatus::Pass,
                datum_references: Vec::new(),
                face_ids: Vec::new(),
            }
        }
    }

    /// Check a single solid against all given tolerances.
    ///
    /// Returns one [`GdntFeatureReport`] per tolerance specification.
    pub fn check_solid_all(
        &self,
        name: &str,
        mesh: &TriangleMesh,
        tolerances: &[GdntTolerance],
    ) -> Vec<GdntFeatureReport> {
        if mesh.vertices.is_empty() || tolerances.is_empty() {
            return Vec::new();
        }

        let mesh_checker = MeshGdtChecker::new(mesh);

        tolerances
            .iter()
            .map(|tolerance| {
                let spec = tolerance.to_tolerance_spec();
                let result = mesh_checker.check(&spec);
                let mut report = GdntFeatureReport::from_mesh_result(&result, tolerance);
                report.feature_name = format!("{} — {}", name, tolerance.name);
                report
            })
            .collect()
    }

    /// Check assembly-level GD&T (inter-part tolerances).
    ///
    /// Evaluates tolerances that span multiple parts in an assembly,
    /// such as position, parallelism, and perpendicularity between
    /// different components. Each mesh is checked individually first,
    /// then inter-part checks are performed for datumed tolerances.
    ///
    /// # Arguments
    ///
    /// * `nodes` — Assembly nodes (each may have a solid and transform)
    /// * `meshes` — Pre-triangulated meshes for each node
    ///
    /// # Returns
    ///
    /// A [`GdntReport`] combining per-part and inter-part results.
    pub fn check_assembly(
        &self,
        nodes: &[AssemblyNode],
        meshes: &[TriangleMesh],
    ) -> GdntReport {
        let tolerances = GdntPreset::machining_general();
        self.check_assembly_with_tolerances(nodes, meshes, &tolerances)
    }

    /// Check assembly-level GD&T with custom tolerances.
    ///
    /// # Arguments
    ///
    /// * `nodes` — Assembly nodes
    /// * `meshes` — Pre-triangulated meshes (must be same length as nodes)
    /// * `tolerances` — Custom tolerance specifications
    pub fn check_assembly_with_tolerances(
        &self,
        nodes: &[AssemblyNode],
        meshes: &[TriangleMesh],
        tolerances: &[GdntTolerance],
    ) -> GdntReport {
        let assembly_name = if nodes.is_empty() {
            "Empty Assembly".to_string()
        } else {
            nodes[0].name.clone()
        };
        let mut report = GdntReport::new(&assembly_name);

        // Per-part checks
        for (i, (node, mesh)) in nodes.iter().zip(meshes.iter()).enumerate() {
            if mesh.vertices.is_empty() {
                log::debug!("Node '{}' has empty mesh — skipping", node.name);
                continue;
            }

            let part_reports = self.check_solid_all(&node.name, mesh, tolerances);
            for part_report in part_reports {
                report.add_result(part_report);
            }

            // Inter-part checks: compare with subsequent parts
            for j in (i + 1)..nodes.len() {
                if j >= meshes.len() || meshes[j].vertices.is_empty() {
                    continue;
                }

                // Check parallelism between parts (compare principal normals)
                let inter_reports = self.check_inter_part(
                    &nodes[i].name,
                    mesh,
                    &nodes[j].name,
                    &meshes[j],
                    tolerances,
                );
                for inter_report in inter_reports {
                    report.add_result(inter_report);
                }
            }
        }

        report
    }

    /// Check inter-part GD&T between two meshes.
    ///
    /// Compares the geometric relationship of two parts, checking
    /// parallelism and position between their principal features.
    fn check_inter_part(
        &self,
        name_a: &str,
        mesh_a: &TriangleMesh,
        name_b: &str,
        mesh_b: &TriangleMesh,
        tolerances: &[GdntTolerance],
    ) -> Vec<GdntFeatureReport> {
        let mut reports = Vec::new();

        // Find orientation-type tolerances for inter-part checks
        for tol in tolerances {
            match tol.tolerance_type {
                GdntToleranceType::Parallelism => {
                    let deviation = self.compute_inter_part_parallelism(mesh_a, mesh_b);
                    reports.push(GdntFeatureReport {
                        feature_name: format!("{} ∥ {} — Parallelism", name_a, name_b),
                        tolerance_type: "Parallelism".to_string(),
                        tolerance_value: tol.value,
                        actual_deviation: deviation,
                        status: GdntStatus::from_deviation(deviation, tol.value),
                        datum_references: tol.datum_references.clone(),
                        face_ids: Vec::new(),
                    });
                }
                GdntToleranceType::Position => {
                    let deviation = self.compute_inter_part_position(mesh_a, mesh_b);
                    reports.push(GdntFeatureReport {
                        feature_name: format!("{} ⊕ {} — Position", name_a, name_b),
                        tolerance_type: "Position".to_string(),
                        tolerance_value: tol.value,
                        actual_deviation: deviation,
                        status: GdntStatus::from_deviation(deviation, tol.value),
                        datum_references: tol.datum_references.clone(),
                        face_ids: Vec::new(),
                    });
                }
                _ => {}
            }
        }

        reports
    }

    /// Compute parallelism deviation between two meshes.
    ///
    /// Measures the angular deviation between the principal normals
    /// of two mesh surfaces. Two perfectly parallel surfaces have
    /// a deviation of zero.
    fn compute_inter_part_parallelism(&self, mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> f64 {
        let normal_a = compute_mesh_normal(mesh_a);
        let normal_b = compute_mesh_normal(mesh_b);

        if normal_a.is_none() || normal_b.is_none() {
            return 0.0;
        }

        let na = normal_a.unwrap();
        let nb = normal_b.unwrap();

        // Angular deviation as tangent of angle between normals
        let dot = na.dot(&nb).abs().min(1.0);
        let angle = dot.acos();
        // Approximate linear deviation from angle over characteristic size
        let size_a = mesh_characteristic_size(mesh_a);
        angle * size_a
    }

    /// Compute position deviation between two mesh centroids.
    ///
    /// Measures the distance between the centroids of two meshes
    /// as a proxy for positional tolerance.
    fn compute_inter_part_position(&self, mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> f64 {
        let centroid_a = mesh_centroid(mesh_a);
        let centroid_b = mesh_centroid(mesh_b);
        centroid_a.distance_to(&centroid_b)
    }
}

impl Default for GdntChecker {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Batch GD&T checking
// ============================================================

/// Check multiple meshes against the same set of tolerances.
///
/// Useful for production-line inspection where many parts need to be
/// validated against the same specification. Each mesh is checked
/// independently and results are returned in order.
///
/// # Arguments
///
/// * `meshes` — Slice of (name, mesh) pairs
/// * `tolerances` — Tolerance specifications to apply to all meshes
///
/// # Returns
///
/// A vector of [`GdntFeatureReport`], one per (mesh × tolerance) combination.
///
/// # Example
///
/// ```ignore
/// use draper_core::gdnt::{batch_check, GdntPreset};
///
/// let tolerances = GdntPreset::machining_general();
/// let meshes: Vec<(String, &TriangleMesh)> = vec![
///     ("part_001".to_string(), &mesh1),
///     ("part_002".to_string(), &mesh2),
/// ];
/// let results = batch_check(&meshes, &tolerances);
/// for result in &results {
///     println!("{}", result);
/// }
/// ```
pub fn batch_check(
    meshes: &[(String, &TriangleMesh)],
    tolerances: &[GdntTolerance],
) -> Vec<GdntFeatureReport> {
    let checker = GdntChecker::new();
    let mut results = Vec::new();

    for (name, mesh) in meshes {
        let part_results = checker.check_solid_all(name, mesh, tolerances);
        results.extend(part_results);
    }

    results
}

// ============================================================
// Internal helper functions
// ============================================================

/// Generate an ISO 8601 timestamp from the current system time.
///
/// Uses `std::time::SystemTime` to avoid a `chrono` dependency.
/// Format: `YYYY-MM-DDTHH:MM:SSZ`
fn iso8601_now() -> String {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_else(|_| std::time::Duration::from_secs(0));

    let total_secs = duration.as_secs();

    // Compute date components
    let (year, month, day, hour, minute, second) = unix_time_to_date(total_secs);

    format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
        year, month, day, hour, minute, second
    )
}

/// Convert Unix timestamp to date/time components.
///
/// Implements a simplified Gregorian calendar conversion.
fn unix_time_to_date(secs: u64) -> (u64, u64, u64, u64, u64, u64) {
    let days_since_epoch = secs / 86400;
    let time_of_day = secs % 86400;
    let hour = time_of_day / 3600;
    let minute = (time_of_day % 3600) / 60;
    let second = time_of_day % 60;

    // Algorithm from http://howardhinnant.github.io/date_algorithms.html
    let z = days_since_epoch + 719468;
    let era = z / 146097;
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = y + if m <= 2 { 1 } else { 0 };

    (year, m, d, hour, minute, second)
}

/// Escape a string for JSON output (handles quotes, backslashes, control chars).
fn escape_json(s: &str) -> String {
    let mut escaped = String::with_capacity(s.len());
    for ch in s.chars() {
        match ch {
            '"' => escaped.push_str("\\\""),
            '\\' => escaped.push_str("\\\\"),
            '\n' => escaped.push_str("\\n"),
            '\r' => escaped.push_str("\\r"),
            '\t' => escaped.push_str("\\t"),
            c if c.is_control() => {
                escaped.push_str(&format!("\\u{:04x}", c as u32));
            }
            c => escaped.push(c),
        }
    }
    escaped
}

/// Compute the centroid of a mesh.
fn mesh_centroid(mesh: &TriangleMesh) -> Point3d {
    if mesh.vertices.is_empty() {
        return Point3d::ORIGIN;
    }
    let n = mesh.vertices.len() as f64;
    let cx = mesh.vertices.iter().map(|p| p.x).sum::<f64>() / n;
    let cy = mesh.vertices.iter().map(|p| p.y).sum::<f64>() / n;
    let cz = mesh.vertices.iter().map(|p| p.z).sum::<f64>() / n;
    Point3d::new(cx, cy, cz)
}

/// Compute the principal surface normal of a mesh via area-weighted average.
fn compute_mesh_normal(mesh: &TriangleMesh) -> Option<Vec3d> {
    if mesh.triangles.is_empty() || mesh.vertices.len() < 3 {
        return None;
    }

    let mut nx = 0.0_f64;
    let mut ny = 0.0_f64;
    let mut nz = 0.0_f64;

    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0]];
        let v1 = mesh.vertices[tri[1]];
        let v2 = mesh.vertices[tri[2]];

        let e1 = Vec3d::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = Vec3d::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let cross = e1.cross(&e2);

        // Area-weighted normal (cross product magnitude is 2× area)
        nx += cross.x;
        ny += cross.y;
        nz += cross.z;
    }

    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    if len < 1e-15 {
        return None;
    }

    Some(Vec3d::new(nx / len, ny / len, nz / len))
}

/// Compute a characteristic size for a mesh (cube root of bounding box volume).
fn mesh_characteristic_size(mesh: &TriangleMesh) -> f64 {
    if mesh.vertices.is_empty() {
        return 0.0;
    }

    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut min_z = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;
    let mut max_z = f64::MIN;

    for p in &mesh.vertices {
        min_x = min_x.min(p.x);
        min_y = min_y.min(p.y);
        min_z = min_z.min(p.z);
        max_x = max_x.max(p.x);
        max_y = max_y.max(p.y);
        max_z = max_z.max(p.z);
    }

    let dx = max_x - min_x;
    let dy = max_y - min_y;
    let dz = max_z - min_z;

    (dx * dy * dz).cbrt().max(1.0) // At least 1.0 to avoid zero
}

/// Compute the eigenvector corresponding to the smallest eigenvalue
/// of a 3×3 symmetric matrix (for datum normal computation).
///
/// This is the same PCA algorithm used in the mesh-level checker,
/// duplicated here to keep the core module self-contained.
fn smallest_eigenvector_3x3(
    xx: f64,
    xy: f64,
    xz: f64,
    yy: f64,
    yz: f64,
    zz: f64,
) -> (f64, f64, f64) {
    let largest = largest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);

    let lambda = largest.0 * (xx * largest.0 + xy * largest.1 + xz * largest.2)
        + largest.1 * (xy * largest.0 + yy * largest.1 + yz * largest.2)
        + largest.2 * (xz * largest.0 + yz * largest.1 + zz * largest.2);

    let len_sq = largest.0 * largest.0 + largest.1 * largest.1 + largest.2 * largest.2;
    if len_sq < 1e-30 {
        return (0.0, 0.0, 1.0);
    }

    let dxx = xx - lambda * largest.0 * largest.0 / len_sq;
    let dxy = xy - lambda * largest.0 * largest.1 / len_sq;
    let dxz = xz - lambda * largest.0 * largest.2 / len_sq;
    let dyy = yy - lambda * largest.1 * largest.1 / len_sq;
    let dyz = yz - lambda * largest.1 * largest.2 / len_sq;
    let dzz = zz - lambda * largest.2 * largest.2 / len_sq;

    let second = largest_eigenvector_3x3(dxx, dxy, dxz, dyy, dyz, dzz);

    let lambda2 = second.0 * (dxx * second.0 + dxy * second.1 + dxz * second.2)
        + second.1 * (dxy * second.0 + dyy * second.1 + dyz * second.2)
        + second.2 * (dxz * second.0 + dyz * second.1 + dzz * second.2);
    let len_sq2 = second.0 * second.0 + second.1 * second.1 + second.2 * second.2;
    if len_sq2 < 1e-30 {
        return cross_3d(largest, second);
    }

    let d2xx = dxx - lambda2 * second.0 * second.0 / len_sq2;
    let d2xy = dxy - lambda2 * second.0 * second.1 / len_sq2;
    let d2xz = dxz - lambda2 * second.0 * second.2 / len_sq2;
    let d2yy = dyy - lambda2 * second.1 * second.1 / len_sq2;
    let d2yz = dyz - lambda2 * second.1 * second.2 / len_sq2;
    let d2zz = dzz - lambda2 * second.2 * second.2 / len_sq2;

    largest_eigenvector_3x3(d2xx, d2xy, d2xz, d2yy, d2yz, d2zz)
}

/// Compute the eigenvector corresponding to the largest eigenvalue
/// of a 3×3 symmetric matrix using the power method.
fn largest_eigenvector_3x3(
    xx: f64,
    xy: f64,
    xz: f64,
    yy: f64,
    yz: f64,
    zz: f64,
) -> (f64, f64, f64) {
    let mut v = if xx >= yy && xx >= zz {
        (1.0, 0.0, 0.0)
    } else if yy >= zz {
        (0.0, 1.0, 0.0)
    } else {
        (0.0, 0.0, 1.0)
    };

    for _ in 0..20 {
        let new_v = (
            xx * v.0 + xy * v.1 + xz * v.2,
            xy * v.0 + yy * v.1 + yz * v.2,
            xz * v.0 + yz * v.1 + zz * v.2,
        );
        let len = (new_v.0 * new_v.0 + new_v.1 * new_v.1 + new_v.2 * new_v.2).sqrt();
        if len < 1e-30 {
            break;
        }
        v = (new_v.0 / len, new_v.1 / len, new_v.2 / len);
    }

    v
}

/// Cross product of two 3D vectors (tuple form).
fn cross_3d(a: (f64, f64, f64), b: (f64, f64, f64)) -> (f64, f64, f64) {
    (
        a.1 * b.2 - a.2 * b.1,
        a.2 * b.0 - a.0 * b.2,
        a.0 * b.1 - a.1 * b.0,
    )
}

// ============================================================
// Unit tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    // ----------------------------------------------------------
    // GdntStatus tests
    // ----------------------------------------------------------

    #[test]
    fn test_status_display() {
        assert_eq!(format!("{}", GdntStatus::Pass), "PASS");
        assert_eq!(format!("{}", GdntStatus::Warning), "WARNING");
        assert_eq!(format!("{}", GdntStatus::Fail), "FAIL");
    }

    #[test]
    fn test_status_from_deviation() {
        // Well within tolerance
        assert_eq!(GdntStatus::from_deviation(0.01, 0.1), GdntStatus::Pass);
        // 80% utilization → warning
        assert_eq!(GdntStatus::from_deviation(0.08, 0.1), GdntStatus::Warning);
        // Exactly at 75% → still pass (not strictly greater)
        assert_eq!(GdntStatus::from_deviation(0.075, 0.1), GdntStatus::Pass);
        // Just over 75%
        assert_eq!(GdntStatus::from_deviation(0.076, 0.1), GdntStatus::Warning);
        // Exceeds tolerance
        assert_eq!(GdntStatus::from_deviation(0.11, 0.1), GdntStatus::Fail);
        // NaN deviation
        assert_eq!(GdntStatus::from_deviation(f64::NAN, 0.1), GdntStatus::Fail);
        // Zero tolerance
        assert_eq!(GdntStatus::from_deviation(0.001, 0.0), GdntStatus::Fail);
    }

    // ----------------------------------------------------------
    // GdntToleranceType tests
    // ----------------------------------------------------------

    #[test]
    fn test_tolerance_type_display() {
        assert_eq!(format!("{}", GdntToleranceType::Flatness), "Flatness");
        assert_eq!(format!("{}", GdntToleranceType::CircularRunout), "Circular Runout");
        assert_eq!(format!("{}", GdntToleranceType::ProfileLine), "Profile of a Line");
        assert_eq!(format!("{}", GdntToleranceType::ProfileSurface), "Profile of a Surface");
    }

    #[test]
    fn test_tolerance_type_requires_datum() {
        // Form tolerances don't need datums
        assert!(!GdntToleranceType::Flatness.requires_datum());
        assert!(!GdntToleranceType::Straightness.requires_datum());
        assert!(!GdntToleranceType::Circularity.requires_datum());
        assert!(!GdntToleranceType::Cylindricity.requires_datum());

        // Orientation and location tolerances need datums
        assert!(GdntToleranceType::Parallelism.requires_datum());
        assert!(GdntToleranceType::Perpendicularity.requires_datum());
        assert!(GdntToleranceType::Angularity.requires_datum());
        assert!(GdntToleranceType::Position.requires_datum());
        assert!(GdntToleranceType::Concentricity.requires_datum());
        assert!(GdntToleranceType::Symmetry.requires_datum());
        assert!(GdntToleranceType::CircularRunout.requires_datum());
        assert!(GdntToleranceType::TotalRunout.requires_datum());

        // Profile tolerances can optionally reference datums
        assert!(!GdntToleranceType::ProfileLine.requires_datum());
        assert!(!GdntToleranceType::ProfileSurface.requires_datum());
    }

    #[test]
    fn test_tolerance_type_category() {
        assert_eq!(GdntToleranceType::Flatness.category(), "Form");
        assert_eq!(GdntToleranceType::Parallelism.category(), "Orientation");
        assert_eq!(GdntToleranceType::Position.category(), "Location");
        assert_eq!(GdntToleranceType::CircularRunout.category(), "Runout");
        assert_eq!(GdntToleranceType::ProfileLine.category(), "Profile");
    }

    #[test]
    fn test_tolerance_type_to_mesh_check_type() {
        assert_eq!(GdntToleranceType::Flatness.to_mesh_check_type(), GdtCheckType::Flatness);
        assert_eq!(GdntToleranceType::Cylindricity.to_mesh_check_type(), GdtCheckType::Cylindricity);
        assert_eq!(GdntToleranceType::Parallelism.to_mesh_check_type(), GdtCheckType::Parallelism);
        assert_eq!(GdntToleranceType::ProfileLine.to_mesh_check_type(), GdtCheckType::ProfileOfLine);
        assert_eq!(GdntToleranceType::ProfileSurface.to_mesh_check_type(), GdtCheckType::ProfileOfSurface);

        // Concentricity maps to Unsupported (not yet in mesh-level checker)
        match GdntToleranceType::Concentricity.to_mesh_check_type() {
            GdtCheckType::Unsupported(_) => {}
            other => panic!("Expected Unsupported, got {:?}", other),
        }
    }

    // ----------------------------------------------------------
    // GdntTolerance tests
    // ----------------------------------------------------------

    #[test]
    fn test_tolerance_builder() {
        let tol = GdntTolerance::new(GdntToleranceType::Flatness, 0.05, "Top face")
            .with_datum("A")
            .with_datum("B")
            .with_faces(vec![1, 2, 3]);

        assert_eq!(tol.tolerance_type, GdntToleranceType::Flatness);
        assert!((tol.value - 0.05).abs() < 1e-10);
        assert_eq!(tol.datum_references, vec!["A", "B"]);
        assert_eq!(tol.applies_to_faces, vec![1, 2, 3]);
        assert_eq!(tol.name, "Top face");
    }

    #[test]
    fn test_tolerance_display() {
        let tol = GdntTolerance::new(GdntToleranceType::Parallelism, 0.05, "Side parallelism")
            .with_datum("A");

        let display = format!("{}", tol);
        assert!(display.contains("Side parallelism"));
        assert!(display.contains("Parallelism"));
        assert!(display.contains("Datum: A"));
    }

    #[test]
    fn test_tolerance_to_spec() {
        let tol = GdntTolerance::new(GdntToleranceType::Flatness, 0.05, "Test flatness")
            .with_datum("A");
        let spec = tol.to_tolerance_spec();

        assert_eq!(spec.name, "Test flatness");
        assert_eq!(spec.tolerance_type, GdtCheckType::Flatness);
        assert!((spec.tolerance_value - 0.05).abs() < 1e-10);
    }

    // ----------------------------------------------------------
    // GdntFeatureReport tests
    // ----------------------------------------------------------

    #[test]
    fn test_feature_report_margin_and_utilization() {
        let report = GdntFeatureReport {
            feature_name: "Test".to_string(),
            tolerance_type: "Flatness".to_string(),
            tolerance_value: 0.10,
            actual_deviation: 0.06,
            status: GdntStatus::Pass,
            datum_references: vec![],
            face_ids: vec![],
        };

        assert!((report.margin() - 0.04).abs() < 1e-10);
        assert!((report.utilization() - 0.6).abs() < 1e-10);
    }

    #[test]
    fn test_feature_report_display() {
        let report = GdntFeatureReport {
            feature_name: "Top face".to_string(),
            tolerance_type: "Flatness".to_string(),
            tolerance_value: 0.10,
            actual_deviation: 0.06,
            status: GdntStatus::Pass,
            datum_references: vec![],
            face_ids: vec![],
        };

        let display = format!("{}", report);
        assert!(display.contains("PASS"));
        assert!(display.contains("Top face"));
        assert!(display.contains("Flatness"));
    }

    // ----------------------------------------------------------
    // GdntReport tests
    // ----------------------------------------------------------

    #[test]
    fn test_report_empty() {
        let report = GdntReport::new("test_part");
        assert_eq!(report.part_name, "test_part");
        assert_eq!(report.overall_status, GdntStatus::Pass);
        assert_eq!(report.pass_count, 0);
        assert_eq!(report.fail_count, 0);
        assert_eq!(report.warning_count, 0);
        assert_eq!(report.total_count(), 0);
    }

    #[test]
    fn test_report_add_results() {
        let mut report = GdntReport::new("test_part");

        report.add_result(GdntFeatureReport {
            feature_name: "F1".to_string(),
            tolerance_type: "Flatness".to_string(),
            tolerance_value: 0.05,
            actual_deviation: 0.01,
            status: GdntStatus::Pass,
            datum_references: vec![],
            face_ids: vec![],
        });

        report.add_result(GdntFeatureReport {
            feature_name: "F2".to_string(),
            tolerance_type: "Parallelism".to_string(),
            tolerance_value: 0.05,
            actual_deviation: 0.04,
            status: GdntStatus::Warning,
            datum_references: vec!["A".to_string()],
            face_ids: vec![1],
        });

        report.add_result(GdntFeatureReport {
            feature_name: "F3".to_string(),
            tolerance_type: "Position".to_string(),
            tolerance_value: 0.05,
            actual_deviation: 0.10,
            status: GdntStatus::Fail,
            datum_references: vec!["A".to_string(), "B".to_string()],
            face_ids: vec![2, 3],
        });

        assert_eq!(report.pass_count, 1);
        assert_eq!(report.warning_count, 1);
        assert_eq!(report.fail_count, 1);
        assert_eq!(report.total_count(), 3);
        assert_eq!(report.overall_status, GdntStatus::Fail);
    }

    #[test]
    fn test_report_overall_warning() {
        let mut report = GdntReport::new("test_part");
        report.add_result(GdntFeatureReport {
            feature_name: "F1".to_string(),
            tolerance_type: "Flatness".to_string(),
            tolerance_value: 0.05,
            actual_deviation: 0.01,
            status: GdntStatus::Pass,
            datum_references: vec![],
            face_ids: vec![],
        });
        report.add_result(GdntFeatureReport {
            feature_name: "F2".to_string(),
            tolerance_type: "Parallelism".to_string(),
            tolerance_value: 0.05,
            actual_deviation: 0.04,
            status: GdntStatus::Warning,
            datum_references: vec![],
            face_ids: vec![],
        });

        assert_eq!(report.overall_status, GdntStatus::Warning);
    }

    #[test]
    fn test_report_summary() {
        let mut report = GdntReport::new("bracket");
        report.add_result(GdntFeatureReport {
            feature_name: "Top face".to_string(),
            tolerance_type: "Flatness".to_string(),
            tolerance_value: 0.05,
            actual_deviation: 0.01,
            status: GdntStatus::Pass,
            datum_references: vec![],
            face_ids: vec![],
        });

        let summary = report.summary();
        assert!(summary.contains("GD&T Report: bracket"));
        assert!(summary.contains("Overall: PASS"));
        assert!(summary.contains("1 total"));
    }

    #[test]
    fn test_report_json() {
        let mut report = GdntReport::new("housing");
        report.add_result(GdntFeatureReport {
            feature_name: "Bore".to_string(),
            tolerance_type: "Cylindricity".to_string(),
            tolerance_value: 0.02,
            actual_deviation: 0.015,
            status: GdntStatus::Warning,
            datum_references: vec!["A".to_string()],
            face_ids: vec![5, 6],
        });

        let json = report.to_json();
        assert!(json.starts_with('{'));
        assert!(json.contains("\"part_name\":\"housing\""));
        assert!(json.contains("\"overall_status\":\"WARNING\""));
        assert!(json.contains("\"pass_count\":0"));
        assert!(json.contains("\"warning_count\":1"));
        assert!(json.contains("\"results\":["));
        assert!(json.contains("\"Cylindricity\""));
        assert!(json.contains("\"datum_references\":[\"A\"]"));
        assert!(json.contains("\"face_ids\":[5,6]"));
        assert!(json.ends_with("]}"));
    }

    // ----------------------------------------------------------
    // DatumReferenceSystem tests
    // ----------------------------------------------------------

    #[test]
    fn test_datum_reference_system_new() {
        let drs = DatumReferenceSystem::new();
        assert!(drs.primary.is_none());
        assert!(drs.secondary.is_none());
        assert!(drs.tertiary.is_none());
    }

    #[test]
    fn test_datum_reference_system_from_labels() {
        let drs = DatumReferenceSystem::from_labels("A", Some("B"), Some("C"));
        assert_eq!(drs.primary.as_deref(), Some("A"));
        assert_eq!(drs.secondary.as_deref(), Some("B"));
        assert_eq!(drs.tertiary.as_deref(), Some("C"));
    }

    #[test]
    fn test_datum_result_identity() {
        let dr = DatumResult::identity();
        assert_eq!(dr.origin, Point3d::ORIGIN);
        assert!(dr.is_orthonormal());
    }

    #[test]
    fn test_datum_result_display() {
        let dr = DatumResult::identity();
        let display = format!("{}", dr);
        assert!(display.contains("DatumResult"));
    }

    // ----------------------------------------------------------
    // GdntPreset tests
    // ----------------------------------------------------------

    #[test]
    fn test_preset_machining_general() {
        let tolerances = GdntPreset::machining_general();
        assert!(!tolerances.is_empty());
        assert!(tolerances.iter().any(|t| t.tolerance_type == GdntToleranceType::Flatness));
        assert!(tolerances.iter().any(|t| t.tolerance_type == GdntToleranceType::Cylindricity));
        assert!(tolerances.iter().any(|t| t.tolerance_type == GdntToleranceType::Position));
    }

    #[test]
    fn test_preset_precision_machining() {
        let tolerances = GdntPreset::precision_machining();
        assert!(!tolerances.is_empty());

        // Precision should be tighter than general
        let gen = GdntPreset::machining_general();
        let gen_flatness = gen.iter().find(|t| t.tolerance_type == GdntToleranceType::Flatness).unwrap();
        let prec_flatness = tolerances.iter().find(|t| t.tolerance_type == GdntToleranceType::Flatness).unwrap();
        assert!(prec_flatness.value < gen_flatness.value);
    }

    #[test]
    fn test_preset_sheet_metal() {
        let tolerances = GdntPreset::sheet_metal();
        assert!(!tolerances.is_empty());

        // Sheet metal should be more relaxed than machining
        let gen = GdntPreset::machining_general();
        let gen_flatness = gen.iter().find(|t| t.tolerance_type == GdntToleranceType::Flatness).unwrap();
        let sm_flatness = tolerances.iter().find(|t| t.tolerance_type == GdntToleranceType::Flatness).unwrap();
        assert!(sm_flatness.value > gen_flatness.value);
    }

    #[test]
    fn test_preset_casting() {
        let tolerances = GdntPreset::casting();
        assert!(!tolerances.is_empty());

        // Casting should be the most relaxed
        let sm = GdntPreset::sheet_metal();
        let sm_flatness = sm.iter().find(|t| t.tolerance_type == GdntToleranceType::Flatness).unwrap();
        let cast_flatness = tolerances.iter().find(|t| t.tolerance_type == GdntToleranceType::Flatness).unwrap();
        assert!(cast_flatness.value > sm_flatness.value);
    }

    // ----------------------------------------------------------
    // GdntChecker tests (with mock mesh)
    // ----------------------------------------------------------

    fn flat_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(10.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(10.0, 10.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.0, 10.0, 0.0));
        mesh.add_triangle([v0, v1, v2]);
        mesh.add_triangle([v0, v2, v3]);
        mesh
    }

    fn warped_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(10.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(10.0, 10.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.0, 10.0, 0.0));
        let v4 = mesh.add_vertex(Point3d::new(5.0, 5.0, 0.5)); // Raised center
        mesh.add_triangle([v0, v1, v4]);
        mesh.add_triangle([v1, v2, v4]);
        mesh.add_triangle([v2, v3, v4]);
        mesh.add_triangle([v3, v0, v4]);
        mesh
    }

    #[test]
    fn test_checker_new() {
        let _checker = GdntChecker::new();
    }

    #[test]
    fn test_checker_check_solid_flat() {
        let mesh = flat_mesh();
        let checker = GdntChecker::new();
        let tolerances = vec![
            GdntTolerance::new(GdntToleranceType::Flatness, 0.1, "Flatness test"),
        ];

        let report = checker.check_solid("flat_part", &mesh, &tolerances);
        assert_eq!(report.feature_name, "flat_part");
        assert!(report.actual_deviation < 1e-6, "Flat mesh should have near-zero deviation");
    }

    #[test]
    fn test_checker_check_solid_all() {
        let mesh = flat_mesh();
        let checker = GdntChecker::new();
        let tolerances = vec![
            GdntTolerance::new(GdntToleranceType::Flatness, 0.1, "Flatness"),
            GdntTolerance::new(GdntToleranceType::Straightness, 0.1, "Straightness"),
        ];

        let reports = checker.check_solid_all("flat_part", &mesh, &tolerances);
        assert_eq!(reports.len(), 2);
    }

    #[test]
    fn test_checker_check_solid_warped() {
        let mesh = warped_mesh();
        let checker = GdntChecker::new();
        let tolerances = vec![
            GdntTolerance::new(GdntToleranceType::Flatness, 0.1, "Flatness test"),
        ];

        let report = checker.check_solid("warped_part", &mesh, &tolerances);
        assert!(report.actual_deviation > 0.1, "Warped mesh should have significant deviation");
    }

    #[test]
    fn test_checker_check_solid_empty_tolerances() {
        let mesh = flat_mesh();
        let checker = GdntChecker::new();
        let report = checker.check_solid("part", &mesh, &[]);
        assert_eq!(report.status, GdntStatus::Pass);
    }

    #[test]
    fn test_checker_check_assembly() {
        let mesh_a = flat_mesh();
        let mesh_b = warped_mesh();
        let checker = GdntChecker::new();

        let nodes = vec![
            AssemblyNode::new("Part A"),
            AssemblyNode::new("Part B"),
        ];

        let report = checker.check_assembly(&nodes, &[mesh_a, mesh_b]);
        assert_eq!(report.part_name, "Part A");
    }

    // ----------------------------------------------------------
    // Batch check tests
    // ----------------------------------------------------------

    #[test]
    fn test_batch_check() {
        let mesh_a = flat_mesh();
        let mesh_b = warped_mesh();
        let tolerances = GdntPreset::machining_general();

        let meshes: Vec<(String, &TriangleMesh)> = vec![
            ("Part A".to_string(), &mesh_a),
            ("Part B".to_string(), &mesh_b),
        ];

        let results = batch_check(&meshes, &tolerances);
        // Each mesh × each tolerance
        assert_eq!(results.len(), tolerances.len() * 2);
    }

    #[test]
    fn test_batch_check_empty() {
        let tolerances = GdntPreset::machining_general();
        let meshes: Vec<(String, &TriangleMesh)> = vec![];
        let results = batch_check(&meshes, &tolerances);
        assert!(results.is_empty());
    }

    // ----------------------------------------------------------
    // Datum establishment tests
    // ----------------------------------------------------------

    #[test]
    fn test_establish_datum() {
        let mesh = flat_mesh();
        let drs = DatumReferenceSystem::from_labels("A", None, None);

        let datum = drs.establish_datum(&mesh, "A", 0);
        // The flat mesh should produce a Z-axis normal (pointing along Z)
        assert!(datum.z_axis.z.abs() > 0.9, "Flat mesh normal should be along Z, got z_axis={:?}", datum.z_axis);
    }

    #[test]
    fn test_establish_datum_empty_mesh() {
        let mesh = TriangleMesh::new();
        let drs = DatumReferenceSystem::new();
        let datum = drs.establish_datum(&mesh, "A", 0);
        // Should return identity
        assert_eq!(datum.origin, Point3d::ORIGIN);
    }

    // ----------------------------------------------------------
    // JSON escaping tests
    // ----------------------------------------------------------

    #[test]
    fn test_escape_json() {
        assert_eq!(escape_json("hello"), "hello");
        assert_eq!(escape_json("say \"hi\""), "say \\\"hi\\\"");
        assert_eq!(escape_json("line1\nline2"), "line1\\nline2");
        assert_eq!(escape_json("tab\there"), "tab\\there");
        assert_eq!(escape_json("back\\slash"), "back\\\\slash");
    }

    // ----------------------------------------------------------
    // Timestamp tests
    // ----------------------------------------------------------

    #[test]
    fn test_iso8601_format() {
        let ts = iso8601_now();
        // Should match YYYY-MM-DDTHH:MM:SSZ
        assert_eq!(ts.len(), 20);
        assert_eq!(ts.chars().nth(4), Some('-'));
        assert_eq!(ts.chars().nth(7), Some('-'));
        assert_eq!(ts.chars().nth(10), Some('T'));
        assert_eq!(ts.chars().nth(13), Some(':'));
        assert_eq!(ts.chars().nth(16), Some(':'));
        assert_eq!(ts.chars().nth(19), Some('Z'));
    }

    #[test]
    fn test_unix_time_to_date() {
        // 2024-01-01T00:00:00Z = 1704067200
        let (y, m, d, h, min, s) = unix_time_to_date(1704067200);
        assert_eq!(y, 2024);
        assert_eq!(m, 1);
        assert_eq!(d, 1);
        assert_eq!(h, 0);
        assert_eq!(min, 0);
        assert_eq!(s, 0);
    }

    // ----------------------------------------------------------
    // Mesh helper tests
    // ----------------------------------------------------------

    #[test]
    fn test_mesh_centroid() {
        let mesh = flat_mesh();
        let c = mesh_centroid(&mesh);
        // Centroid of the flat mesh should be around (5, 5, 0)
        assert!((c.x - 5.0).abs() < 1.0);
        assert!((c.y - 5.0).abs() < 1.0);
        assert!(c.z.abs() < 1e-10);
    }

    #[test]
    fn test_mesh_normal() {
        let mesh = flat_mesh();
        let normal = compute_mesh_normal(&mesh);
        assert!(normal.is_some());
        let n = normal.unwrap();
        // Flat mesh in XY plane should have normal along Z
        assert!(n.z.abs() > 0.9);
    }

    #[test]
    fn test_mesh_characteristic_size() {
        let mesh = flat_mesh();
        let size = mesh_characteristic_size(&mesh);
        assert!(size > 0.0);
    }
}
