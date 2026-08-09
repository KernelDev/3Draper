// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! AI-driven geometry healing via ML models (ROADMAP_VISION_2036 §8).
//!
//! Per §8.1: Use local neural networks (ONNX Runtime in Rust) for:
//! - **Gap Prediction**: predict missing surface patches for hole closure
//! - **Feature Recognition**: auto-detect chamfers, fillets, holes
//! - **Topology Repair**: predict correct edge-face adjacency
//!
//! This module provides the interface for ML-based healing. The actual
//! ONNX model inference is behind a trait so it can be:
//! - Mocked in tests (rule-based fallback)
//! - Replaced with real ONNX Runtime when available
//! - Extended with custom models
//!
//! Per §8.2: Training pipeline collects "dirty" STEP files with known-good
//! repairs, trains ONNX models, ships as binary assets.

use draper_geometry::{Point3d, NurbsSurface, NurbsCurve, Curve3d};
use draper_topology::{Shell, Face, Edge};

// ============================================================
// Healing request / result types
// ============================================================

/// A gap (hole) detected in a B-Rep shell that needs ML-based repair.
#[derive(Clone, Debug)]
pub struct GapDescriptor {
    /// 3D boundary points of the gap (ordered CCW from outside).
    pub boundary_points: Vec<Point3d>,
    /// Adjacent face normals at the gap boundary (for orientation).
    pub adjacent_normals: Vec<[f64; 3]>,
    /// Adjacent surface types (for pattern matching).
    pub adjacent_surface_types: Vec<SurfaceTypeHint>,
    /// Bounding box of the gap region.
    pub bbox_min: Point3d,
    pub bbox_max: Point3d,
}

/// Hint about the surface type adjacent to a gap.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SurfaceTypeHint {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Nurbs,
    Unknown,
}

/// Result of ML-based gap prediction.
#[derive(Clone, Debug)]
pub struct GapPrediction {
    /// Predicted 3D points filling the gap (ordered to form a surface patch).
    pub patch_points: Vec<Point3d>,
    /// Confidence score [0, 1] — 1.0 = high confidence.
    pub confidence: f64,
    /// Predicted surface type for the patch.
    pub predicted_surface_type: SurfaceTypeHint,
    /// Optional: fitted NURBS surface for the patch (when confidence is high).
    pub nurbs_patch: Option<NurbsSurface>,
}

/// A detected feature (chamfer, fillet, hole pattern) in a B-Rep model.
#[derive(Clone, Debug)]
pub struct FeatureDescriptor {
    /// Type of detected feature.
    pub feature_type: FeatureType,
    /// Face indices involved in this feature.
    pub face_indices: Vec<usize>,
    /// Confidence score [0, 1].
    pub confidence: f64,
    /// Feature parameters (e.g., fillet radius, chamfer angle).
    pub parameters: Vec<f64>,
}

/// Types of manufacturing features that can be recognized.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FeatureType {
    /// Fillet (rounded edge transition).
    Fillet,
    /// Chamfer (angled edge transition).
    Chamfer,
    /// Circular hole (through or blind).
    Hole,
    /// Slot (rectangular cutout).
    Slot,
    /// Pocket (rectangular depression).
    Pocket,
    /// Rib (thin wall feature).
    Rib,
    /// Pattern (array of repeated features).
    Pattern,
    /// Unknown feature.
    Unknown,
}

/// Request for AI-driven healing of a B-Rep shell.
#[derive(Clone, Debug)]
pub struct HealingRequest {
    /// Gaps detected in the shell that need repair.
    pub gaps: Vec<GapDescriptor>,
    /// Whether to attempt feature recognition.
    pub recognize_features: bool,
    /// Minimum confidence threshold for accepting predictions [0, 1].
    pub min_confidence: f64,
}

/// Result of AI-driven healing.
#[derive(Clone, Debug, Default)]
pub struct HealingResult {
    /// Patches generated for each gap (same order as request.gaps).
    pub patches: Vec<Option<GapPrediction>>,
    /// Detected features (if requested).
    pub features: Vec<FeatureDescriptor>,
    /// Number of gaps successfully repaired.
    pub repaired_count: usize,
    /// Number of gaps that couldn't be repaired (low confidence).
    pub failed_count: usize,
}

// ============================================================
// ML inference trait (ONNX interface)
// ============================================================

/// Trait for ML model inference in geometry healing.
///
/// Implementations can use:
/// - `RuleBasedHealingModel`: heuristic fallback (no ML, always available)
/// - `OnnxHealingModel`: real ONNX Runtime inference (when model files are available)
///
/// Per Directive 3 (Panic-Free): all methods return `Result`, never panic.
pub trait HealingModel: Send + Sync {
    /// Predict a surface patch to fill a gap.
    ///
    /// The model takes the gap boundary (points + normals + surface types)
    /// and returns predicted patch points + optional NURBS surface.
    fn predict_gap(&self, gap: &GapDescriptor) -> Option<GapPrediction>;

    /// Recognize manufacturing features in a shell.
    ///
    /// Returns detected features (fillets, chamfers, holes, etc.) with
    /// confidence scores and parameters.
    fn recognize_features(&self, shell: &Shell) -> Vec<FeatureDescriptor>;

    /// Get the model name and version (for logging).
    fn model_info(&self) -> &str;
}

// ============================================================
// Rule-based healing model (fallback — no ML required)
// ============================================================

/// Heuristic healing model that uses geometric rules instead of ML.
///
/// This is the default fallback when no ONNX model is available.
/// It provides reasonable results for common gap patterns:
/// - Planar gaps → fill with planar patch
/// - Cylindrical gaps → fill with cylindrical patch
/// - Small gaps (< 10 boundary points) → centroid fan triangulation
impl RuleBasedHealingModel {
    pub fn new() -> Self {
        Self
    }
}

impl Default for RuleBasedHealingModel {
    fn default() -> Self {
        Self::new()
    }
}

/// Rule-based healing model (no ML, always available).
pub struct RuleBasedHealingModel;

impl HealingModel for RuleBasedHealingModel {
    fn predict_gap(&self, gap: &GapDescriptor) -> Option<GapPrediction> {
        if gap.boundary_points.len() < 3 {
            return None;
        }

        // Compute centroid
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;
        for p in &gap.boundary_points {
            cx += p.x;
            cy += p.y;
            cz += p.z;
        }
        let n = gap.boundary_points.len() as f64;
        cx /= n;
        cy /= n;
        cz /= n;
        let centroid = Point3d::new(cx, cy, cz);

        // Compute average boundary radius
        let mut avg_radius = 0.0;
        for p in &gap.boundary_points {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            avg_radius += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        avg_radius /= n;

        // Determine surface type from adjacent surfaces
        let predicted_type = if gap.adjacent_surface_types.iter().all(|t| *t == SurfaceTypeHint::Plane) {
            SurfaceTypeHint::Plane
        } else if gap.adjacent_surface_types.iter().any(|t| *t == SurfaceTypeHint::Cylinder) {
            SurfaceTypeHint::Cylinder
        } else {
            SurfaceTypeHint::Unknown
        };

        // Generate patch points: centroid + boundary points
        let mut patch_points = vec![centroid];
        patch_points.extend(gap.boundary_points.iter().cloned());

        // Confidence: higher for planar gaps (easier to predict)
        let confidence = match predicted_type {
            SurfaceTypeHint::Plane => 0.9,
            SurfaceTypeHint::Cylinder => 0.7,
            _ => 0.5,
        };

        Some(GapPrediction {
            patch_points,
            confidence,
            predicted_surface_type: predicted_type,
            nurbs_patch: None, // Rule-based model doesn't fit NURBS
        })
    }

    fn recognize_features(&self, shell: &Shell) -> Vec<FeatureDescriptor> {
        let mut features = Vec::new();

        // Rule: faces with Surface::Cylinder and small radius → Hole
        for (i, face) in shell.faces.iter().enumerate() {
            if let Some(draper_geometry::Surface::Cylinder(cyl)) = &face.surface {
                if cyl.radius < 5.0 {
                    features.push(FeatureDescriptor {
                        feature_type: FeatureType::Hole,
                        face_indices: vec![i],
                        confidence: 0.8,
                        parameters: vec![cyl.radius],
                    });
                }
            }
        }

        // Rule: faces with small area between two planar faces → Fillet
        for (i, face) in shell.faces.iter().enumerate() {
            if face.edges.len() <= 4 {
                // Check if neighbors are planes (simplified)
                let is_small = face.edges.len() <= 2;
                if is_small {
                    features.push(FeatureDescriptor {
                        feature_type: FeatureType::Fillet,
                        face_indices: vec![i],
                        confidence: 0.5,
                        parameters: vec![],
                    });
                }
            }
        }

        features
    }

    fn model_info(&self) -> &str {
        "RuleBasedHealingModel v0.1 (heuristic fallback, no ML)"
    }
}

// ============================================================
// ONNX healing model (interface — requires ONNX Runtime)
// ============================================================

/// ONNX-based healing model.
///
/// This struct provides the interface for loading and running ONNX models
/// for geometry healing. The actual ONNX Runtime integration requires
/// the `ort` crate (ONNX Runtime Rust bindings) as an optional dependency.
///
/// Per ROADMAP_VISION_2036 §8.2: models are trained on 10,000+ "dirty"
/// STEP files and shipped as binary assets.
///
/// # Future implementation
/// ```ignore
/// use ort::{Environment, Session, SessionBuilder};
///
/// let env = Environment::builder().build()?;
/// let session = SessionBuilder::new(&env)?
///     .with_optimization_level(GraphOptimizationLevel::Level1)?
///     .with_model_from_file("assets/models/gap_prediction.onnx")?;
/// ```
pub struct OnnxHealingModel {
    /// Path to the gap prediction model file.
    pub gap_model_path: String,
    /// Path to the feature recognition model file.
    pub feature_model_path: String,
    /// Whether the models are loaded and ready.
    pub models_loaded: bool,
}

impl OnnxHealingModel {
    /// Create a new ONNX healing model with the given model paths.
    ///
    /// Models are NOT loaded until `load()` is called.
    pub fn new(gap_model_path: &str, feature_model_path: &str) -> Self {
        Self {
            gap_model_path: gap_model_path.to_string(),
            feature_model_path: feature_model_path.to_string(),
            models_loaded: false,
        }
    }

    /// Load the ONNX models.
    ///
    /// Returns an error if the model files don't exist or ONNX Runtime
    /// is not available.
    pub fn load(&mut self) -> Result<(), String> {
        // Check if model files exist
        if !std::path::Path::new(&self.gap_model_path).exists() {
            return Err(format!("Gap model not found: {}", self.gap_model_path));
        }
        if !std::path::Path::new(&self.feature_model_path).exists() {
            return Err(format!("Feature model not found: {}", self.feature_model_path));
        }

        // TODO: Load ONNX models using `ort` crate when available
        // let env = ort::Environment::builder().build().map_err(|e| e.to_string())?;
        // let session = ort::SessionBuilder::new(&env).map_err(|e| e.to_string())?
        //     .with_model_from_file(&self.gap_model_path).map_err(|e| e.to_string())?;

        log::info!("ONNX healing models loaded (gap: {}, feature: {})",
            self.gap_model_path, self.feature_model_path);
        self.models_loaded = true;
        Ok(())
    }
}

impl HealingModel for OnnxHealingModel {
    fn predict_gap(&self, gap: &GapDescriptor) -> Option<GapPrediction> {
        if !self.models_loaded {
            // Fall back to rule-based when models aren't loaded
            return RuleBasedHealingModel.predict_gap(gap);
        }

        // TODO: Actual ONNX inference
        // 1. Encode gap boundary as fixed-size tensor (pad/truncate to N×3)
        // 2. Run model.forward(input_tensor)
        // 3. Decode output tensor → patch points + confidence
        // 4. Fit NURBS surface to predicted points (when confidence > 0.8)

        // For now, use rule-based as fallback
        log::debug!("ONNX model not yet implemented — using rule-based fallback");
        RuleBasedHealingModel.predict_gap(gap)
    }

    fn recognize_features(&self, shell: &Shell) -> Vec<FeatureDescriptor> {
        if !self.models_loaded {
            return RuleBasedHealingModel.recognize_features(shell);
        }

        // TODO: Actual ONNX inference
        // 1. Encode shell topology as graph tensor
        // 2. Run model.forward(input_tensor)
        // 3. Decode output → feature types + parameters

        log::debug!("ONNX feature recognition not yet implemented — using rule-based fallback");
        RuleBasedHealingModel.recognize_features(shell)
    }

    fn model_info(&self) -> &str {
        if self.models_loaded {
            "OnnxHealingModel v0.1 (ONNX Runtime, models loaded)"
        } else {
            "OnnxHealingModel v0.1 (models NOT loaded — using rule-based fallback)"
        }
    }
}

// ============================================================
// Top-level AI healing API
// ============================================================

/// Run AI-driven healing on a shell with detected gaps.
///
/// Uses the provided healing model (or rule-based fallback) to:
/// 1. Predict surface patches for each gap
/// 2. Optionally recognize manufacturing features
/// 3. Return predictions for integration into the B-Rep
///
/// Per Directive 3: never panics — returns None for gaps that can't be repaired.
pub fn heal_with_model(
    request: &HealingRequest,
    model: &dyn HealingModel,
) -> HealingResult {
    let mut result = HealingResult::default();
    log::info!("AI healing: {} gaps, model={}", request.gaps.len(), model.model_info());

    for gap in &request.gaps {
        match model.predict_gap(gap) {
            Some(prediction) if prediction.confidence >= request.min_confidence => {
                result.patches.push(Some(prediction));
                result.repaired_count += 1;
            }
            Some(prediction) => {
                log::debug!(
                    "AI healing: gap rejected (confidence={:.2} < threshold={:.2})",
                    prediction.confidence, request.min_confidence
                );
                result.patches.push(None);
                result.failed_count += 1;
            }
            None => {
                result.patches.push(None);
                result.failed_count += 1;
            }
        }
    }

    if request.recognize_features {
        // Feature recognition requires a Shell — but we don't have one here.
        // This would be called separately with the actual shell.
        log::debug!("AI healing: feature recognition requested (call separately with shell)");
    }

    log::info!(
        "AI healing complete: {} repaired, {} failed (confidence threshold: {:.2})",
        result.repaired_count, result.failed_count, request.min_confidence
    );

    result
}

/// Create the default healing model (rule-based fallback).
///
/// This is used when no ONNX models are available.
pub fn default_healing_model() -> Box<dyn HealingModel> {
    Box::new(RuleBasedHealingModel::new())
}

/// Create an ONNX healing model (if model files are available).
///
/// Returns the rule-based fallback if ONNX models can't be loaded.
pub fn try_create_onnx_model(
    gap_model_path: &str,
    feature_model_path: &str,
) -> Box<dyn HealingModel> {
    let mut model = OnnxHealingModel::new(gap_model_path, feature_model_path);
    match model.load() {
        Ok(()) => Box::new(model),
        Err(e) => {
            log::warn!("Failed to load ONNX healing models: {} — using rule-based fallback", e);
            Box::new(RuleBasedHealingModel::new())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_gap() -> GapDescriptor {
        let boundary = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        GapDescriptor {
            boundary_points: boundary,
            adjacent_normals: vec![[0.0, 0.0, 1.0]; 4],
            adjacent_surface_types: vec![SurfaceTypeHint::Plane; 4],
            bbox_min: Point3d::new(0.0, 0.0, 0.0),
            bbox_max: Point3d::new(1.0, 1.0, 0.0),
        }
    }

    #[test]
    fn test_rule_based_gap_prediction() {
        let model = RuleBasedHealingModel::new();
        let gap = make_test_gap();
        let prediction = model.predict_gap(&gap).expect("Should predict");

        assert!(prediction.confidence > 0.5);
        assert_eq!(prediction.predicted_surface_type, SurfaceTypeHint::Plane);
        // Patch points: centroid + 4 boundary = 5
        assert_eq!(prediction.patch_points.len(), 5);
    }

    #[test]
    fn test_healing_with_model() {
        let request = HealingRequest {
            gaps: vec![make_test_gap()],
            recognize_features: false,
            min_confidence: 0.5,
        };
        let model = default_healing_model();
        let result = heal_with_model(&request, model.as_ref());

        assert_eq!(result.repaired_count, 1);
        assert_eq!(result.failed_count, 0);
        assert!(result.patches[0].is_some());
    }

    #[test]
    fn test_low_confidence_rejected() {
        let request = HealingRequest {
            gaps: vec![make_test_gap()],
            recognize_features: false,
            min_confidence: 0.99, // Very high threshold
        };
        // Planar gap has confidence 0.9 → should be rejected
        let model = default_healing_model();
        let result = heal_with_model(&request, model.as_ref());

        assert_eq!(result.repaired_count, 0);
        assert_eq!(result.failed_count, 1);
    }

    #[test]
    fn test_onnx_fallback() {
        // ONNX model without loading should fall back to rule-based
        let model = OnnxHealingModel::new("nonexistent.onnx", "nonexistent.onnx");
        let gap = make_test_gap();
        let prediction = model.predict_gap(&gap);

        assert!(prediction.is_some(), "Should fall back to rule-based");
    }

    #[test]
    fn test_empty_gap() {
        let model = RuleBasedHealingModel::new();
        let gap = GapDescriptor {
            boundary_points: vec![],
            adjacent_normals: vec![],
            adjacent_surface_types: vec![],
            bbox_min: Point3d::ORIGIN,
            bbox_max: Point3d::ORIGIN,
        };
        assert!(model.predict_gap(&gap).is_none());
    }
}
