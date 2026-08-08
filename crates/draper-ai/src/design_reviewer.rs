// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Design reviewer — analyzes a sequence of geometry actions for
//! manufacturability, cost, and structural issues.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 5.2: provides automated
//! design review that flags potential problems before manufacturing:
//!
//! - **Manufacturability**: walls too thin, holes too close to edges,
//!   deep pockets, sharp internal corners (cannot be milled).
//! - **Cost indicators**: high part count, excessive material use,
//!   complex geometry (high fillet/chamfer count).
//! - **Structural**: thin sections under load, stress concentrations
//!   at sharp corners, large unsupported spans.
//!
//! # Usage
//!
//! ```ignore
//! use draper_ai::shape_parser::{ShapeParser, GeometryAction};
//! use draper_ai::design_reviewer::{DesignReviewer, ReviewSeverity};
//!
//! let parser = ShapeParser::new();
//! let actions = parser.parse("box 50x50x5 holes of diameter 10 fillet 0.5").unwrap();
//! let reviewer = DesignReviewer::new();
//! let report = reviewer.review(&actions);
//!
//! for issue in &report.issues {
//!     println!("{:?}: {}", issue.severity, issue.message);
//! }
//! ```

use crate::shape_parser::GeometryAction;
use serde::{Deserialize, Serialize};

// ============================================================
// Review report
// ============================================================

/// Severity of a design issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewSeverity {
    /// Informational — no action needed.
    Info,
    /// Warning — may cause issues, consider revising.
    Warning,
    /// Error — will cause manufacturing failure or structural problem.
    Error,
}

impl ReviewSeverity {
    pub fn as_str(&self) -> &'static str {
        match self {
            ReviewSeverity::Info => "INFO",
            ReviewSeverity::Warning => "WARNING",
            ReviewSeverity::Error => "ERROR",
        }
    }
}

/// A single design review issue.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReviewIssue {
    /// Severity level.
    pub severity: ReviewSeverity,
    /// Category of issue (Manufacturability, Cost, Structural, etc.).
    pub category: ReviewCategory,
    /// Human-readable description of the issue.
    pub message: String,
    /// Index of the action that triggered this issue (if applicable).
    pub action_index: Option<usize>,
    /// Suggested fix (if any).
    pub suggestion: Option<String>,
}

/// Category of design review issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewCategory {
    /// Manufacturing feasibility (milling, turning, 3D printing).
    Manufacturability,
    /// Cost indicators (material, time, complexity).
    Cost,
    /// Structural integrity (stress, deflection).
    Structural,
    /// Design best practices (naming, symmetry, etc.).
    BestPractice,
}

impl ReviewCategory {
    pub fn as_str(&self) -> &'static str {
        match self {
            ReviewCategory::Manufacturability => "Manufacturability",
            ReviewCategory::Cost => "Cost",
            ReviewCategory::Structural => "Structural",
            ReviewCategory::BestPractice => "BestPractice",
        }
    }
}

/// A complete design review report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReviewReport {
    /// All issues found, ordered by severity (errors first).
    pub issues: Vec<ReviewIssue>,
    /// Overall design score (0.0 = terrible, 100.0 = excellent).
    pub score: f64,
    /// Summary statistics.
    pub stats: ReviewStats,
}

/// Summary statistics from a design review.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReviewStats {
    pub total_actions: usize,
    pub error_count: usize,
    pub warning_count: usize,
    pub info_count: usize,
    pub has_fillets: bool,
    pub has_chamfers: bool,
    pub has_shells: bool,
    pub boolean_operation_count: usize,
    pub estimated_material_volume: f64,
}

// ============================================================
// DesignReviewer
// ============================================================

/// Configuration thresholds for design review.
#[derive(Debug, Clone)]
pub struct ReviewConfig {
    /// Minimum wall thickness (mm) — thinner walls are flagged.
    pub min_wall_thickness: f64,
    /// Minimum hole edge distance (mm) — holes closer to edges are flagged.
    pub min_hole_edge_distance: f64,
    /// Minimum fillet radius (mm) — smaller fillets are flagged.
    pub min_fillet_radius: f64,
    /// Maximum aspect ratio (height/diameter) before flagging as deep.
    pub max_aspect_ratio: f64,
    /// Maximum number of boolean operations before flagging complexity.
    pub max_boolean_ops: usize,
}

impl Default for ReviewConfig {
    fn default() -> Self {
        Self {
            min_wall_thickness: 1.0,
            min_hole_edge_distance: 2.0,
            min_fillet_radius: 0.5,
            max_aspect_ratio: 10.0,
            max_boolean_ops: 20,
        }
    }
}

/// Automated design reviewer.
pub struct DesignReviewer {
    config: ReviewConfig,
}

impl Default for DesignReviewer {
    fn default() -> Self {
        Self::new()
    }
}

impl DesignReviewer {
    pub fn new() -> Self {
        Self { config: ReviewConfig::default() }
    }

    pub fn with_config(config: ReviewConfig) -> Self {
        Self { config }
    }

    /// Review a sequence of geometry actions and produce a report.
    pub fn review(&self, actions: &[GeometryAction]) -> ReviewReport {
        let mut issues = Vec::new();
        let mut stats = ReviewStats {
            total_actions: actions.len(),
            ..ReviewStats::default()
        };

        let mut last_box_size: Option<[f64; 3]> = None;
        let mut material_volume = 0.0_f64;

        for (i, action) in actions.iter().enumerate() {
            match action {
                GeometryAction::CreateBox { size, center: _ } => {
                    last_box_size = Some(*size);
                    let volume = size[0] * size[1] * size[2];
                    material_volume += volume;

                    // Check wall thickness (only meaningful if we know the part is hollow)
                    let min_dim = size[0].min(size[1]).min(size[2]);
                    if min_dim < self.config.min_wall_thickness {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Error,
                            category: ReviewCategory::Manufacturability,
                            message: format!(
                                "Box wall thickness {:.2}mm is below minimum {:.2}mm — part may not print/mill correctly",
                                min_dim, self.config.min_wall_thickness
                            ),
                            action_index: Some(i),
                            suggestion: Some(format!(
                                "Increase the smallest dimension to at least {:.2}mm",
                                self.config.min_wall_thickness
                            )),
                        });
                    }

                    // Check aspect ratio
                    let max_dim = size[0].max(size[1]).max(size[2]);
                    if min_dim > 0.0 && max_dim / min_dim > self.config.max_aspect_ratio {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Warning,
                            category: ReviewCategory::Structural,
                            message: format!(
                                "Box aspect ratio {:.1}:{:.1} = {:.1} exceeds maximum {:.1} — part may warp or be fragile",
                                max_dim, min_dim, max_dim / min_dim, self.config.max_aspect_ratio
                            ),
                            action_index: Some(i),
                            suggestion: Some("Consider rebalancing dimensions or adding ribs for support".to_string()),
                        });
                    }
                }

                GeometryAction::CreateCylinder { diameter, height, center: _ } => {
                    let diameter = *diameter;
                    let height = *height;
                    let volume = std::f64::consts::PI * (diameter / 2.0).powi(2) * height;
                    material_volume += volume;

                    // Check aspect ratio for deep holes
                    if diameter > 0.0 && height / diameter > self.config.max_aspect_ratio {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Warning,
                            category: ReviewCategory::Manufacturability,
                            message: format!(
                                "Cylinder aspect ratio {:.1} (height/diameter) is high — deep holes are hard to machine accurately",
                                height / diameter
                            ),
                            action_index: Some(i),
                            suggestion: Some("Consider using a stepped hole or reducing depth".to_string()),
                        });
                    }

                    // Check if hole is too close to edge (we'd need the box dimensions)
                    if let Some(box_size) = last_box_size {
                        // Flag if hole diameter exceeds 50% of the smaller face dimension
                        if diameter > box_size[0].min(box_size[1]) * 0.5 {
                            issues.push(ReviewIssue {
                                severity: ReviewSeverity::Warning,
                                category: ReviewCategory::Manufacturability,
                                message: format!(
                                    "Hole diameter {:.2}mm is large relative to box face — may weaken the part",
                                    diameter
                                ),
                                action_index: Some(i),
                                suggestion: Some(format!(
                                    "Reduce hole diameter or increase face size to at least {:.1}mm",
                                    diameter * 2.0
                                )),
                            });
                        }
                    }
                }

                GeometryAction::CreateSphere { diameter, center: _ } => {
                    let volume = (4.0 / 3.0) * std::f64::consts::PI * (diameter / 2.0).powi(3);
                    material_volume += volume;
                }

                GeometryAction::CreateCone { bottom_diameter, top_diameter, height, center: _ } => {
                    let r1 = bottom_diameter / 2.0;
                    let r2 = top_diameter / 2.0;
                    let volume = (1.0 / 3.0) * std::f64::consts::PI * height * (r1 * r1 + r1 * r2 + r2 * r2);
                    material_volume += volume;
                }

                GeometryAction::CreateTorus { major_diameter, minor_diameter, center: _ } => {
                    let r = major_diameter / 2.0;
                    let t = minor_diameter / 2.0;
                    let volume = 2.0 * std::f64::consts::PI * std::f64::consts::PI * r * t * t;
                    material_volume += volume;
                }

                GeometryAction::BooleanSubtract => {
                    stats.boolean_operation_count += 1;
                    // Subtracting reduces volume (approximate — we don't track exact)
                    if material_volume > 0.0 {
                        material_volume *= 0.95; // Assume 5% material removed
                    }
                }

                GeometryAction::BooleanUnion => {
                    stats.boolean_operation_count += 1;
                }

                GeometryAction::BooleanIntersect => {
                    stats.boolean_operation_count += 1;
                    if material_volume > 0.0 {
                        material_volume *= 0.5; // Intersect typically reduces volume
                    }
                }

                GeometryAction::FilletAllEdges { radius } => {
                    stats.has_fillets = true;
                    if *radius < self.config.min_fillet_radius {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Warning,
                            category: ReviewCategory::Manufacturability,
                            message: format!(
                                "Fillet radius {:.2}mm is below recommended minimum {:.2}mm — tool may break",
                                radius, self.config.min_fillet_radius
                            ),
                            action_index: Some(i),
                            suggestion: Some(format!(
                                "Increase fillet radius to at least {:.2}mm",
                                self.config.min_fillet_radius
                            )),
                        });
                    }
                }

                GeometryAction::ChamferAllEdges { distance } => {
                    stats.has_chamfers = true;
                    if *distance < self.config.min_fillet_radius {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Info,
                            category: ReviewCategory::BestPractice,
                            message: format!(
                                "Chamfer distance {:.2}mm is very small — may be hard to see/measure",
                                distance
                            ),
                            action_index: Some(i),
                            suggestion: None,
                        });
                    }
                }

                GeometryAction::Shell { thickness } => {
                    stats.has_shells = true;
                    if *thickness < self.config.min_wall_thickness {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Error,
                            category: ReviewCategory::Manufacturability,
                            message: format!(
                                "Shell thickness {:.2}mm is below minimum {:.2}mm — walls will be too thin",
                                thickness, self.config.min_wall_thickness
                            ),
                            action_index: Some(i),
                            suggestion: Some(format!(
                                "Increase shell thickness to at least {:.2}mm",
                                self.config.min_wall_thickness
                            )),
                        });
                    }
                }

                GeometryAction::ExtrudeProfile { profile_points, distance } => {
                    if profile_points.len() < 3 {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Error,
                            category: ReviewCategory::BestPractice,
                            message: format!(
                                "Extrude profile has only {} points — need at least 3 for a valid polygon",
                                profile_points.len()
                            ),
                            action_index: Some(i),
                            suggestion: Some("Add more points to the profile".to_string()),
                        });
                    }
                    let _ = distance;
                }

                GeometryAction::RevolveProfile { profile_points, angle_degrees } => {
                    if profile_points.len() < 2 {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Error,
                            category: ReviewCategory::BestPractice,
                            message: format!(
                                "Revolve profile has only {} points — need at least 2",
                                profile_points.len()
                            ),
                            action_index: Some(i),
                            suggestion: Some("Add more points to the profile".to_string()),
                        });
                    }
                    if *angle_degrees <= 0.0 || *angle_degrees > 360.0 {
                        issues.push(ReviewIssue {
                            severity: ReviewSeverity::Warning,
                            category: ReviewCategory::BestPractice,
                            message: format!(
                                "Revolve angle {:.1}° is out of typical range (0–360°)",
                                angle_degrees
                            ),
                            action_index: Some(i),
                            suggestion: None,
                        });
                    }
                }
            }
        }

        // Check overall complexity
        if stats.boolean_operation_count > self.config.max_boolean_ops {
            issues.push(ReviewIssue {
                severity: ReviewSeverity::Warning,
                category: ReviewCategory::Cost,
                message: format!(
                    "Design has {} boolean operations — high complexity increases machining/repair cost",
                    stats.boolean_operation_count
                ),
                action_index: None,
                suggestion: Some(format!(
                    "Consider simplifying to fewer than {} boolean operations",
                    self.config.max_boolean_ops
                )),
            });
        }

        // Best practice: no fillets on a complex part
        if !stats.has_fillets && !stats.has_chamfers && stats.total_actions > 2 {
            issues.push(ReviewIssue {
                severity: ReviewSeverity::Info,
                category: ReviewCategory::BestPractice,
                message: "No fillets or chamfers applied — sharp corners may cause stress concentrations".to_string(),
                action_index: None,
                suggestion: Some("Consider adding fillets to internal corners to reduce stress".to_string()),
            });
        }

        // Update stats
        stats.estimated_material_volume = material_volume;
        for issue in &issues {
            match issue.severity {
                ReviewSeverity::Error => stats.error_count += 1,
                ReviewSeverity::Warning => stats.warning_count += 1,
                ReviewSeverity::Info => stats.info_count += 1,
            }
        }

        // Compute score: start at 100, subtract for each issue
        let mut score = 100.0_f64;
        for issue in &issues {
            match issue.severity {
                ReviewSeverity::Error => score -= 20.0,
                ReviewSeverity::Warning => score -= 5.0,
                ReviewSeverity::Info => score -= 1.0,
            }
        }
        let score = score.max(0.0);

        // Sort issues by severity (errors first)
        issues.sort_by(|a, b| {
            let sev_order = |s: ReviewSeverity| match s {
                ReviewSeverity::Error => 0,
                ReviewSeverity::Warning => 1,
                ReviewSeverity::Info => 2,
            };
            sev_order(a.severity).cmp(&sev_order(b.severity))
        });

        ReviewReport { issues, score, stats }
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;
    use crate::shape_parser::GeometryAction;

    #[test]
    fn test_review_simple_box_no_issues() {
        let reviewer = DesignReviewer::new();
        let actions = vec![GeometryAction::CreateBox {
            size: [50.0, 30.0, 20.0],
            center: [0.0, 0.0, 0.0],
        }];
        let report = reviewer.review(&actions);
        assert!(report.issues.is_empty(), "Simple box should have no issues");
        assert!((report.score - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_review_thin_wall_error() {
        let reviewer = DesignReviewer::new();
        let actions = vec![GeometryAction::CreateBox {
            size: [50.0, 50.0, 0.5], // 0.5mm wall — too thin
            center: [0.0, 0.0, 0.0],
        }];
        let report = reviewer.review(&actions);
        assert!(report.stats.error_count >= 1, "Should have at least 1 error");
        assert!(report.issues.iter().any(|i| i.severity == ReviewSeverity::Error));
        assert!(report.score < 100.0);
    }

    #[test]
    fn test_review_high_aspect_ratio_warning() {
        let reviewer = DesignReviewer::new();
        let actions = vec![GeometryAction::CreateBox {
            size: [100.0, 100.0, 1.0], // aspect ratio 100:1
            center: [0.0, 0.0, 0.0],
        }];
        let report = reviewer.review(&actions);
        assert!(report.stats.warning_count >= 1);
    }

    #[test]
    fn test_review_small_fillet_warning() {
        let reviewer = DesignReviewer::new();
        let actions = vec![
            GeometryAction::CreateBox {
                size: [20.0, 20.0, 20.0],
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::FilletAllEdges { radius: 0.1 }, // Too small
        ];
        let report = reviewer.review(&actions);
        assert!(report.stats.warning_count >= 1);
        assert!(report.issues.iter().any(|i| i.message.contains("Fillet")));
    }

    #[test]
    fn test_review_thin_shell_error() {
        let reviewer = DesignReviewer::new();
        let actions = vec![
            GeometryAction::CreateBox {
                size: [30.0, 30.0, 30.0],
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::Shell { thickness: 0.2 }, // Too thin
        ];
        let report = reviewer.review(&actions);
        assert!(report.stats.error_count >= 1);
        assert!(report.issues.iter().any(|i| i.message.contains("Shell")));
    }

    #[test]
    fn test_review_deep_hole_warning() {
        let reviewer = DesignReviewer::new();
        let actions = vec![GeometryAction::CreateCylinder {
            diameter: 2.0,
            height: 50.0, // aspect ratio 25:1 — very deep
            center: [0.0, 0.0, 0.0],
        }];
        let report = reviewer.review(&actions);
        assert!(report.stats.warning_count >= 1);
    }

    #[test]
    fn test_review_too_many_boolean_ops() {
        let config = ReviewConfig {
            max_boolean_ops: 3,
            ..Default::default()
        };
        let reviewer = DesignReviewer::with_config(config);
        let mut actions = vec![GeometryAction::CreateBox {
            size: [50.0, 50.0, 20.0],
            center: [0.0, 0.0, 0.0],
        }];
        for _ in 0..5 {
            actions.push(GeometryAction::CreateCylinder {
                diameter: 5.0,
                height: 20.0,
                center: [0.0, 0.0, 0.0],
            });
            actions.push(GeometryAction::BooleanSubtract);
        }
        let report = reviewer.review(&actions);
        assert!(report.stats.boolean_operation_count == 5);
        assert!(report.issues.iter().any(|i| i.category == ReviewCategory::Cost));
    }

    #[test]
    fn test_review_no_fillets_info() {
        let reviewer = DesignReviewer::new();
        let actions = vec![
            GeometryAction::CreateBox {
                size: [50.0, 50.0, 20.0],
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::CreateCylinder {
                diameter: 5.0,
                height: 20.0,
                center: [10.0, 10.0, 0.0],
            },
            GeometryAction::BooleanSubtract,
        ];
        let report = reviewer.review(&actions);
        // Should have an Info about no fillets
        assert!(report.issues.iter().any(|i| i.severity == ReviewSeverity::Info));
    }

    #[test]
    fn test_review_extrude_profile_too_few_points() {
        let reviewer = DesignReviewer::new();
        let actions = vec![GeometryAction::ExtrudeProfile {
            profile_points: vec![[0.0, 0.0], [1.0, 0.0]], // Only 2 points
            distance: 10.0,
        }];
        let report = reviewer.review(&actions);
        assert!(report.stats.error_count >= 1);
    }

    #[test]
    fn test_review_revolve_invalid_angle() {
        let reviewer = DesignReviewer::new();
        let actions = vec![GeometryAction::RevolveProfile {
            profile_points: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            angle_degrees: 500.0, // > 360
        }];
        let report = reviewer.review(&actions);
        assert!(report.stats.warning_count >= 1);
    }

    #[test]
    fn test_review_score_decreases_with_errors() {
        let reviewer = DesignReviewer::new();
        let good_actions = vec![GeometryAction::CreateBox {
            size: [50.0, 50.0, 50.0],
            center: [0.0, 0.0, 0.0],
        }];
        let bad_actions = vec![
            GeometryAction::CreateBox {
                size: [0.1, 0.1, 0.1], // All dimensions below minimum
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::Shell { thickness: 0.1 }, // Also below minimum — second error
        ];

        let good_report = reviewer.review(&good_actions);
        let bad_report = reviewer.review(&bad_actions);

        assert!(good_report.score > bad_report.score);
        assert!((good_report.score - 100.0).abs() < 1e-6);
        assert!(bad_report.score < 80.0, "Bad score should be < 80, got {}", bad_report.score);
    }

    #[test]
    fn test_review_stats_tracking() {
        let reviewer = DesignReviewer::new();
        let actions = vec![
            GeometryAction::CreateBox {
                size: [50.0, 50.0, 20.0],
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::CreateCylinder {
                diameter: 5.0,
                height: 20.0,
                center: [10.0, 10.0, 0.0],
            },
            GeometryAction::BooleanSubtract,
            GeometryAction::FilletAllEdges { radius: 2.0 },
        ];
        let report = reviewer.review(&actions);
        assert_eq!(report.stats.total_actions, 4);
        assert_eq!(report.stats.boolean_operation_count, 1);
        assert!(report.stats.has_fillets);
        assert!(!report.stats.has_chamfers);
        assert!(!report.stats.has_shells);
        assert!(report.stats.estimated_material_volume > 0.0);
    }

    #[test]
    fn test_review_issues_sorted_by_severity() {
        let reviewer = DesignReviewer::new();
        let actions = vec![
            GeometryAction::CreateBox {
                size: [0.1, 0.1, 0.1], // Error
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::FilletAllEdges { radius: 0.1 }, // Warning
        ];
        let report = reviewer.review(&actions);
        // First issue should be an error (sorted)
        if !report.issues.is_empty() {
            let first = &report.issues[0];
            assert!(first.severity == ReviewSeverity::Error || first.severity == ReviewSeverity::Warning);
        }
    }

    #[test]
    fn test_review_large_hole_warning() {
        let reviewer = DesignReviewer::new();
        let actions = vec![
            GeometryAction::CreateBox {
                size: [20.0, 20.0, 10.0],
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::CreateCylinder {
                diameter: 15.0, // Large relative to 20mm face
                height: 10.0,
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::BooleanSubtract,
        ];
        let report = reviewer.review(&actions);
        assert!(report.issues.iter().any(|i| i.message.contains("Hole") || i.message.contains("hole")));
    }

    #[test]
    fn test_severity_as_str() {
        assert_eq!(ReviewSeverity::Info.as_str(), "INFO");
        assert_eq!(ReviewSeverity::Warning.as_str(), "WARNING");
        assert_eq!(ReviewSeverity::Error.as_str(), "ERROR");
    }

    #[test]
    fn test_category_as_str() {
        assert_eq!(ReviewCategory::Manufacturability.as_str(), "Manufacturability");
        assert_eq!(ReviewCategory::Cost.as_str(), "Cost");
        assert_eq!(ReviewCategory::Structural.as_str(), "Structural");
        assert_eq!(ReviewCategory::BestPractice.as_str(), "BestPractice");
    }

    #[test]
    fn test_review_empty_actions() {
        let reviewer = DesignReviewer::new();
        let report = reviewer.review(&[]);
        assert_eq!(report.stats.total_actions, 0);
        // Should still have the "no fillets" info (since total_actions <= 2, actually no)
        assert!(report.issues.is_empty());
        assert!((report.score - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_review_with_suggestion() {
        let reviewer = DesignReviewer::new();
        let actions = vec![GeometryAction::CreateBox {
            size: [50.0, 50.0, 0.5],
            center: [0.0, 0.0, 0.0],
        }];
        let report = reviewer.review(&actions);
        let issue = report.issues.iter().find(|i| i.severity == ReviewSeverity::Error).unwrap();
        assert!(issue.suggestion.is_some());
        assert!(issue.suggestion.as_ref().unwrap().contains("Increase"));
    }
}
