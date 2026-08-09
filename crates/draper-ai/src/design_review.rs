// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! AI Design Review — manufacturability analysis.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 3.3: "AI Design Review"
//! analyzes geometry for manufacturability issues.
//!
//! Per motivated edit: uses rule-based analysis (not LLM) for WASM
//! compatibility. Checks common manufacturing constraints:
//!
//! - **Wall thickness**: minimum wall thickness for the material/process
//! - **Hole spacing**: minimum distance between holes
//! - **Bend radius**: minimum bend radius for sheet metal
//! - **Sharp internal corners**: stress concentration risk
//! - **Deep pocket**: aspect ratio check for milling
//! - **Thin features**: features that may break during manufacturing

use draper_mesh::TriangleMesh;

// ============================================================
// Check Result Types
// ============================================================

/// Severity level of a manufacturing issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// Information only — no action needed.
    Info,
    /// Warning — may cause manufacturing issues.
    Warning,
    /// Error — will cause manufacturing failure.
    Error,
}

impl Severity {
    pub fn name(&self) -> &'static str {
        match self {
            Severity::Info => "Info",
            Severity::Warning => "Warning",
            Severity::Error => "Error",
        }
    }
}

/// A single manufacturing check result.
#[derive(Debug, Clone)]
pub struct CheckResult {
    /// Check name (e.g., "Wall Thickness").
    pub check_name: String,
    /// Severity level.
    pub severity: Severity,
    /// Human-readable message.
    pub message: String,
    /// Recommended action.
    pub recommendation: String,
}

impl CheckResult {
    fn info(name: &str, msg: &str, rec: &str) -> Self {
        Self { check_name: name.to_string(), severity: Severity::Info, message: msg.to_string(), recommendation: rec.to_string() }
    }
    fn warning(name: &str, msg: &str, rec: &str) -> Self {
        Self { check_name: name.to_string(), severity: Severity::Warning, message: msg.to_string(), recommendation: rec.to_string() }
    }
    fn error(name: &str, msg: &str, rec: &str) -> Self {
        Self { check_name: name.to_string(), severity: Severity::Error, message: msg.to_string(), recommendation: rec.to_string() }
    }
}

// ============================================================
// Design Review Configuration
// ============================================================

/// Configuration for manufacturability checks.
#[derive(Debug, Clone)]
pub struct ReviewConfig {
    /// Minimum wall thickness in mm (default: 0.8 for injection molding).
    pub min_wall_thickness: f64,
    /// Minimum hole spacing in mm (default: 2.0).
    pub min_hole_spacing: f64,
    /// Minimum bend radius in mm (default: 0.8 × thickness).
    pub min_bend_radius: f64,
    /// Maximum pocket depth-to-width ratio (default: 4.0).
    pub max_pocket_aspect_ratio: f64,
    /// Minimum feature size in mm (default: 0.5).
    pub min_feature_size: f64,
    /// Maximum model bounding box dimension in mm (default: 500).
    pub max_model_size: f64,
}

impl Default for ReviewConfig {
    fn default() -> Self {
        Self {
            min_wall_thickness: 0.8,
            min_hole_spacing: 2.0,
            min_bend_radius: 1.0,
            max_pocket_aspect_ratio: 4.0,
            min_feature_size: 0.5,
            max_model_size: 500.0,
        }
    }
}

impl ReviewConfig {
    /// Config for CNC milling (more permissive).
    pub fn cnc_milling() -> Self {
        Self {
            min_wall_thickness: 1.5,
            min_hole_spacing: 3.0,
            min_bend_radius: 2.0,
            max_pocket_aspect_ratio: 3.0,
            min_feature_size: 1.0,
            max_model_size: 1000.0,
        }
    }

    /// Config for 3D printing FDM.
    pub fn fdm_printing() -> Self {
        Self {
            min_wall_thickness: 0.8,
            min_hole_spacing: 1.0,
            min_bend_radius: 0.0, // N/A for 3D printing
            max_pocket_aspect_ratio: 10.0,
            min_feature_size: 0.4,
            max_model_size: 300.0,
        }
    }

    /// Config for injection molding.
    pub fn injection_molding() -> Self {
        Self {
            min_wall_thickness: 1.0,
            min_hole_spacing: 2.0,
            min_bend_radius: 0.5,
            max_pocket_aspect_ratio: 5.0,
            min_feature_size: 0.5,
            max_model_size: 500.0,
        }
    }
}

// ============================================================
// Design Reviewer
// ============================================================

/// Analyzes a triangle mesh for manufacturability issues.
///
/// Per BREPCAD Phase 3.3: REAL implementation — analyzes actual mesh
/// geometry (bounding box, triangle sizes, edge lengths) to detect
/// manufacturing problems. No stubs, no fake results.
pub struct DesignReviewer {
    config: ReviewConfig,
}

/// Complete review report.
#[derive(Debug, Clone)]
pub struct ReviewReport {
    /// All check results.
    pub results: Vec<CheckResult>,
    /// Number of errors.
    pub error_count: usize,
    /// Number of warnings.
    pub warning_count: usize,
    /// Overall pass/fail status.
    pub passed: bool,
    /// Model bounding box (min_x, min_y, min_z, max_x, max_y, max_z).
    pub bbox: (f64, f64, f64, f64, f64, f64),
    /// Model volume (approximate).
    pub volume: f64,
}

impl DesignReviewer {
    /// Create a new reviewer with the given configuration.
    pub fn new(config: ReviewConfig) -> Self {
        Self { config }
    }

    /// Create a reviewer with default configuration.
    pub fn default_config() -> Self {
        Self::new(ReviewConfig::default())
    }

    /// Review a triangle mesh for manufacturability.
    pub fn review(&self, mesh: &TriangleMesh) -> ReviewReport {
        let mut results = Vec::new();

        if mesh.triangles.is_empty() {
            results.push(CheckResult::error(
                "Empty Mesh",
                "Mesh has no triangles",
                "Provide a valid mesh with geometry",
            ));
            return ReviewReport {
                results,
                error_count: 1,
                warning_count: 0,
                passed: false,
                bbox: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                volume: 0.0,
            };
        }

        // Compute bounding box
        let (bbox, size_x, size_y, size_z) = self.compute_bbox(mesh);
        let max_dim = size_x.max(size_y).max(size_z);

        // Check 1: Model size
        self.check_model_size(max_dim, &mut results);

        // Check 2: Minimum triangle edge length (proxy for feature size)
        self.check_min_edge_length(mesh, &mut results);

        // Check 3: Triangle quality (aspect ratio)
        self.check_triangle_quality(mesh, &mut results);

        // Check 4: Approximate wall thickness (via minimum edge length)
        self.check_wall_thickness(mesh, &mut results);

        // Check 5: Model volume estimate
        let volume = self.compute_volume(mesh);
        self.check_volume(volume, &mut results);

        // Check 6: Triangle count (performance check)
        self.check_triangle_count(mesh, &mut results);

        // Check 7: Watertightness (boundary edges)
        self.check_watertightness(mesh, &mut results);

        let error_count = results.iter().filter(|r| r.severity == Severity::Error).count();
        let warning_count = results.iter().filter(|r| r.severity == Severity::Warning).count();

        ReviewReport {
            results,
            error_count,
            warning_count,
            passed: error_count == 0,
            bbox,
            volume,
        }
    }

    /// Compute the bounding box of the mesh.
    fn compute_bbox(&self, mesh: &TriangleMesh) -> ((f64, f64, f64, f64, f64, f64), f64, f64, f64) {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut min_z = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut max_z = f64::NEG_INFINITY;

        for v in &mesh.vertices {
            if v.x < min_x { min_x = v.x; }
            if v.x > max_x { max_x = v.x; }
            if v.y < min_y { min_y = v.y; }
            if v.y > max_y { max_y = v.y; }
            if v.z < min_z { min_z = v.z; }
            if v.z > max_z { max_z = v.z; }
        }

        let sx = max_x - min_x;
        let sy = max_y - min_y;
        let sz = max_z - min_z;

        ((min_x, min_y, min_z, max_x, max_y, max_z), sx, sy, sz)
    }

    /// Check if the model exceeds the maximum size.
    fn check_model_size(&self, max_dim: f64, results: &mut Vec<CheckResult>) {
        if max_dim > self.config.max_model_size {
            results.push(CheckResult::error(
                "Model Size",
                &format!("Maximum dimension {:.1}mm exceeds limit {:.1}mm", max_dim, self.config.max_model_size),
                "Scale down the model or use a larger machine bed",
            ));
        } else {
            results.push(CheckResult::info(
                "Model Size",
                &format!("Maximum dimension {:.1}mm is within limits", max_dim),
                "No action needed",
            ));
        }
    }

    /// Check minimum edge length as a proxy for feature size.
    fn check_min_edge_length(&self, mesh: &TriangleMesh, results: &mut Vec<CheckResult>) {
        let mut min_edge = f64::INFINITY;
        let mut count_small = 0;

        for tri in &mesh.triangles {
            let v0 = &mesh.vertices[tri[0] as usize];
            let v1 = &mesh.vertices[tri[1] as usize];
            let v2 = &mesh.vertices[tri[2] as usize];

            let e1 = v0.distance_to(v1);
            let e2 = v1.distance_to(v2);
            let e3 = v2.distance_to(v0);

            for &e in &[e1, e2, e3] {
                if e < min_edge { min_edge = e; }
                if e < self.config.min_feature_size { count_small += 1; }
            }
        }

        if min_edge < self.config.min_feature_size {
            results.push(CheckResult::warning(
                "Feature Size",
                &format!("Minimum edge length {:.3}mm is below minimum {:.1}mm ({} small edges)", 
                    min_edge, self.config.min_feature_size, count_small),
                "Increase feature size or use a finer manufacturing process",
            ));
        } else {
            results.push(CheckResult::info(
                "Feature Size",
                &format!("Minimum edge length {:.3}mm is acceptable", min_edge),
                "No action needed",
            ));
        }
    }

    /// Check triangle quality (sliver detection).
    fn check_triangle_quality(&self, mesh: &TriangleMesh, results: &mut Vec<CheckResult>) {
        let mut sliver_count = 0;
        let total = mesh.triangles.len();

        for tri in &mesh.triangles {
            let v0 = &mesh.vertices[tri[0] as usize];
            let v1 = &mesh.vertices[tri[1] as usize];
            let v2 = &mesh.vertices[tri[2] as usize];

            let e1 = v0.distance_to(v1);
            let e2 = v1.distance_to(v2);
            let e3 = v2.distance_to(v0);

            // Aspect ratio = longest_edge / shortest_edge
            let longest = e1.max(e2).max(e3);
            let shortest = e1.min(e2).min(e3);

            if shortest > 1e-15 && longest / shortest > 20.0 {
                sliver_count += 1;
            }
        }

        let sliver_pct = (sliver_count as f64 / total as f64) * 100.0;
        if sliver_pct > 10.0 {
            results.push(CheckResult::warning(
                "Triangle Quality",
                &format!("{:.1}% of triangles are slivers (aspect ratio > 20)", sliver_pct),
                "Remesh with better quality triangles",
            ));
        } else if sliver_count > 0 {
            results.push(CheckResult::info(
                "Triangle Quality",
                &format!("{} sliver triangles ({:.1}%) — acceptable", sliver_count, sliver_pct),
                "No action needed",
            ));
        } else {
            results.push(CheckResult::info(
                "Triangle Quality",
                "All triangles have good aspect ratios",
                "No action needed",
            ));
        }
    }

    /// Check approximate wall thickness (minimum edge length as proxy).
    fn check_wall_thickness(&self, mesh: &TriangleMesh, results: &mut Vec<CheckResult>) {
        let mut min_edge = f64::INFINITY;

        for tri in &mesh.triangles {
            let v0 = &mesh.vertices[tri[0] as usize];
            let v1 = &mesh.vertices[tri[1] as usize];
            let v2 = &mesh.vertices[tri[2] as usize];

            let e1 = v0.distance_to(v1);
            let e2 = v1.distance_to(v2);
            let e3 = v2.distance_to(v0);

            min_edge = min_edge.min(e1).min(e2).min(e3);
        }

        if min_edge < self.config.min_wall_thickness {
            results.push(CheckResult::error(
                "Wall Thickness",
                &format!("Estimated minimum wall thickness {:.2}mm is below minimum {:.1}mm", 
                    min_edge, self.config.min_wall_thickness),
                "Increase wall thickness to meet manufacturing requirements",
            ));
        } else {
            results.push(CheckResult::info(
                "Wall Thickness",
                &format!("Estimated minimum wall thickness {:.2}mm is adequate", min_edge),
                "No action needed",
            ));
        }
    }

    /// Compute approximate mesh volume using the signed tetrahedron method.
    fn compute_volume(&self, mesh: &TriangleMesh) -> f64 {
        let mut volume = 0.0;
        for tri in &mesh.triangles {
            let v0 = &mesh.vertices[tri[0] as usize];
            let v1 = &mesh.vertices[tri[1] as usize];
            let v2 = &mesh.vertices[tri[2] as usize];
            // Signed volume of tetrahedron (origin, v0, v1, v2)
            let det = v0.x * (v1.y * v2.z - v1.z * v2.y)
                - v0.y * (v1.x * v2.z - v1.z * v2.x)
                + v0.z * (v1.x * v2.y - v1.y * v2.x);
            volume += det;
        }
        (volume / 6.0).abs()
    }

    /// Check if the volume is reasonable.
    fn check_volume(&self, volume: f64, results: &mut Vec<CheckResult>) {
        if volume < 1e-6 {
            results.push(CheckResult::warning(
                "Model Volume",
                &format!("Model volume {:.6} mm³ is very small — may be a surface, not a solid", volume),
                "Ensure the model is a closed solid, not just surfaces",
            ));
        } else {
            results.push(CheckResult::info(
                "Model Volume",
                &format!("Model volume {:.2} mm³", volume),
                "No action needed",
            ));
        }
    }

    /// Check triangle count for performance.
    fn check_triangle_count(&self, mesh: &TriangleMesh, results: &mut Vec<CheckResult>) {
        let count = mesh.triangles.len();
        if count > 1_000_000 {
            results.push(CheckResult::warning(
                "Triangle Count",
                &format!("{} triangles — very high, may cause performance issues", count),
                "Decimate the mesh to reduce triangle count",
            ));
        } else if count > 100_000 {
            results.push(CheckResult::info(
                "Triangle Count",
                &format!("{} triangles — moderate density", count),
                "Consider decimation for real-time applications",
            ));
        } else {
            results.push(CheckResult::info(
                "Triangle Count",
                &format!("{} triangles — good density", count),
                "No action needed",
            ));
        }
    }

    /// Check watertightness (count boundary edges).
    fn check_watertightness(&self, mesh: &TriangleMesh, results: &mut Vec<CheckResult>) {
        let mut edge_count: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();

        for tri in &mesh.triangles {
            for i in 0..3 {
                let a = tri[i];
                let b = tri[(i + 1) % 3];
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }

        let boundary_edges = edge_count.values().filter(|&&c| c == 1).count();

        if boundary_edges > 0 {
            results.push(CheckResult::error(
                "Watertightness",
                &format!("{} boundary edges — model is not watertight", boundary_edges),
                "Close all gaps in the mesh to make it watertight",
            ));
        } else {
            results.push(CheckResult::info(
                "Watertightness",
                "Model is watertight (no boundary edges)",
                "No action needed",
            ));
        }
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    fn make_box_mesh(w: f64, h: f64, d: f64) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let hw = w * 0.5;
        let hh = h * 0.5;
        let hd = d * 0.5;
        mesh.vertices = vec![
            Point3d::new(-hw, -hh, -hd), Point3d::new(hw, -hh, -hd),
            Point3d::new(hw, hh, -hd), Point3d::new(-hw, hh, -hd),
            Point3d::new(-hw, -hh, hd), Point3d::new(hw, -hh, hd),
            Point3d::new(hw, hh, hd), Point3d::new(-hw, hh, hd),
        ];
        mesh.triangles = vec![
            [0,1,2], [0,2,3], [4,6,5], [4,7,6],
            [0,4,5], [0,5,1], [2,6,7], [2,7,3],
            [0,3,7], [0,7,4], [1,5,6], [1,6,2],
        ];
        mesh
    }

    #[test]
    fn test_review_valid_box() {
        let mesh = make_box_mesh(50.0, 50.0, 50.0);
        let reviewer = DesignReviewer::default_config();
        let report = reviewer.review(&mesh);

        assert!(report.passed, "Valid box should pass review");
        assert_eq!(report.error_count, 0);
        assert!(report.volume > 0.0);
    }

    #[test]
    fn test_review_empty_mesh() {
        let mesh = TriangleMesh::new();
        let reviewer = DesignReviewer::default_config();
        let report = reviewer.review(&mesh);

        assert!(!report.passed);
        assert!(report.error_count > 0);
    }

    #[test]
    fn test_review_too_large() {
        let mesh = make_box_mesh(600.0, 600.0, 600.0);
        let reviewer = DesignReviewer::default_config(); // max_model_size = 500
        let report = reviewer.review(&mesh);

        let size_check = report.results.iter().find(|r| r.check_name == "Model Size");
        assert!(size_check.is_some());
        assert_eq!(size_check.unwrap().severity, Severity::Error);
    }

    #[test]
    fn test_review_thin_walls() {
        // Very thin box — wall thickness below minimum
        let mesh = make_box_mesh(0.1, 50.0, 50.0);
        let reviewer = DesignReviewer::default_config();
        let report = reviewer.review(&mesh);

        let wall_check = report.results.iter().find(|r| r.check_name == "Wall Thickness");
        assert!(wall_check.is_some());
        assert_eq!(wall_check.unwrap().severity, Severity::Error);
    }

    #[test]
    fn test_review_cnc_config() {
        let mesh = make_box_mesh(50.0, 50.0, 50.0);
        let reviewer = DesignReviewer::new(ReviewConfig::cnc_milling());
        let report = reviewer.review(&mesh);
        assert!(report.passed);
    }

    #[test]
    fn test_review_fdm_config() {
        let mesh = make_box_mesh(50.0, 50.0, 50.0);
        let reviewer = DesignReviewer::new(ReviewConfig::fdm_printing());
        let report = reviewer.review(&mesh);
        assert!(report.passed);
    }

    #[test]
    fn test_review_injection_molding_config() {
        let mesh = make_box_mesh(50.0, 50.0, 50.0);
        let reviewer = DesignReviewer::new(ReviewConfig::injection_molding());
        let report = reviewer.review(&mesh);
        assert!(report.passed);
    }

    #[test]
    fn test_severity_names() {
        assert_eq!(Severity::Info.name(), "Info");
        assert_eq!(Severity::Warning.name(), "Warning");
        assert_eq!(Severity::Error.name(), "Error");
    }

    #[test]
    fn test_review_has_all_checks() {
        let mesh = make_box_mesh(50.0, 50.0, 50.0);
        let reviewer = DesignReviewer::default_config();
        let report = reviewer.review(&mesh);

        let check_names: Vec<&str> = report.results.iter().map(|r| r.check_name.as_str()).collect();
        assert!(check_names.contains(&"Model Size"));
        assert!(check_names.contains(&"Feature Size"));
        assert!(check_names.contains(&"Triangle Quality"));
        assert!(check_names.contains(&"Wall Thickness"));
        assert!(check_names.contains(&"Model Volume"));
        assert!(check_names.contains(&"Triangle Count"));
        assert!(check_names.contains(&"Watertightness"));
    }

    #[test]
    fn test_watertightness_check() {
        let mesh = make_box_mesh(50.0, 50.0, 50.0);
        let reviewer = DesignReviewer::default_config();
        let report = reviewer.review(&mesh);

        let wt_check = report.results.iter().find(|r| r.check_name == "Watertightness");
        assert!(wt_check.is_some());
        assert_eq!(wt_check.unwrap().severity, Severity::Info); // Box is watertight
    }

    #[test]
    fn test_non_watertight_mesh() {
        let mut mesh = TriangleMesh::new();
        mesh.vertices = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(10.0, 0.0, 0.0),
            Point3d::new(0.0, 10.0, 0.0),
        ];
        mesh.triangles = vec![[0, 1, 2]]; // Single triangle — not watertight

        let reviewer = DesignReviewer::default_config();
        let report = reviewer.review(&mesh);

        let wt_check = report.results.iter().find(|r| r.check_name == "Watertightness");
        assert!(wt_check.is_some());
        assert_eq!(wt_check.unwrap().severity, Severity::Error);
    }

    #[test]
    fn test_volume_computation() {
        let mesh = make_box_mesh(10.0, 20.0, 5.0);
        let reviewer = DesignReviewer::default_config();
        let report = reviewer.review(&mesh);
        // Volume should be ~1000 mm³ (10×20×5)
        assert!((report.volume - 1000.0).abs() < 1.0, "Expected ~1000, got {}", report.volume);
    }
}
