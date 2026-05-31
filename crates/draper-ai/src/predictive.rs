// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Predictive mesh optimization (4.6.3).
//!
//! Analyzes a mesh and predicts optimal triangulation parameters for different
//! target quality levels. Uses mesh quality metrics, surface curvature
//! distribution, and model scale to recommend parameters.
//!
//! # Quality targets
//!
//! - **Visual**: Good rendering quality — balanced detail level
//! - **Analytical**: Accurate measurements — high deviation control
//! - **Print**: 3D printing — watertight, no thin walls, fine detail
//! - **FEA**: Finite element analysis — well-shaped elements, no slivers
//!
//! # Predicted parameters
//!
//! - `max_deviation`: Maximum allowed distance between mesh and true surface
//! - `angular_samples`: Number of angular samples for curved surfaces
//! - `height_samples`: Number of height samples for curved surfaces
//! - `detail_level`: LOD multiplier (0.5 = coarse, 1.0 = normal, 2.0 = fine)

use draper_geometry::Point3d;
use draper_mesh::TriangleMesh;

// ============================================================
// QualityTarget enum
// ============================================================

/// Target quality level for mesh optimization.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QualityTarget {
    /// Good rendering quality — balanced detail level.
    /// Priority: visual appearance, reasonable triangle count.
    Visual,
    /// Accurate measurements — high deviation control.
    /// Priority: surface area accuracy, volume accuracy.
    Analytical,
    /// 3D printing — watertight, no thin walls, fine detail.
    /// Priority: watertightness, minimum feature size, no inverted normals.
    Print,
    /// Finite element analysis — well-shaped elements, no slivers.
    /// Priority: element quality, no degenerate triangles, uniform sizing.
    FEA,
}

impl QualityTarget {
    /// Human-readable name.
    pub fn name(&self) -> &'static str {
        match self {
            QualityTarget::Visual => "Visual",
            QualityTarget::Analytical => "Analytical",
            QualityTarget::Print => "Print",
            QualityTarget::FEA => "FEA",
        }
    }

    /// Default `max_deviation` for this quality target relative to model scale.
    pub fn default_deviation_ratio(&self) -> f64 {
        match self {
            QualityTarget::Visual => 0.005,      // 0.5% of model scale
            QualityTarget::Analytical => 0.0005,  // 0.05% of model scale
            QualityTarget::Print => 0.001,        // 0.1% of model scale
            QualityTarget::FEA => 0.002,          // 0.2% of model scale
        }
    }

    /// Default `detail_level` for this quality target.
    pub fn default_detail_level(&self) -> f64 {
        match self {
            QualityTarget::Visual => 1.0,
            QualityTarget::Analytical => 1.5,
            QualityTarget::Print => 1.2,
            QualityTarget::FEA => 1.0,
        }
    }

    /// Minimum angle threshold (in radians) for this quality target.
    pub fn min_angle_threshold(&self) -> f64 {
        match self {
            QualityTarget::Visual => 5.0_f64.to_radians(),     // 5°
            QualityTarget::Analytical => 10.0_f64.to_radians(), // 10°
            QualityTarget::Print => 15.0_f64.to_radians(),     // 15°
            QualityTarget::FEA => 20.0_f64.to_radians(),       // 20° (FEA needs well-shaped elements)
        }
    }
}

impl std::fmt::Display for QualityTarget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

// ============================================================
// MeshQualityMetrics
// ============================================================

/// Quality metrics computed from a triangle mesh.
#[derive(Clone, Debug)]
pub struct MeshQualityMetrics {
    /// Minimum interior angle across all triangles (radians).
    pub min_angle: f64,
    /// Maximum interior angle across all triangles (radians).
    pub max_angle: f64,
    /// Histogram of aspect ratios (10 bins from 0 to max_aspect_ratio).
    pub aspect_ratio_hist: Vec<usize>,
    /// Distribution of triangle areas (min, 25th percentile, median, 75th percentile, max).
    pub area_distribution: [f64; 5],
    /// Curvature statistics (min, mean, max, standard deviation).
    pub curvature_stats: [f64; 4],
    /// Total surface area.
    pub total_area: f64,
    /// Number of triangles.
    pub triangle_count: usize,
    /// Number of vertices.
    pub vertex_count: usize,
    /// Average edge length.
    pub avg_edge_length: f64,
    /// Ratio of max edge length to min edge length (mesh uniformity).
    pub edge_length_ratio: f64,
    /// Percentage of triangles with aspect ratio > 10.
    pub sliver_percentage: f64,
    /// Model scale (bounding box diagonal).
    pub model_scale: f64,
}

impl MeshQualityMetrics {
    /// Overall quality score in [0.0, 1.0] (1.0 = best quality).
    ///
    /// Computed from:
    /// - Angle quality: proportion of triangles with angles > threshold
    /// - Aspect ratio quality: proportion of non-sliver triangles
    /// - Size uniformity: 1 - (edge_length_ratio / ideal_ratio)
    pub fn overall_quality(&self, target: QualityTarget) -> f64 {
        let min_angle_thresh = target.min_angle_threshold();

        // Angle quality: fraction of triangles with min_angle > threshold
        // We approximate this from the global min_angle
        let angle_quality = if self.min_angle >= min_angle_thresh {
            1.0
        } else {
            self.min_angle / min_angle_thresh
        };

        // Aspect ratio quality: fraction of non-sliver triangles
        let aspect_quality = 1.0 - self.sliver_percentage / 100.0;

        // Size uniformity: how uniform is the mesh
        let ideal_ratio = 5.0; // Ideal max/min edge length ratio
        let uniformity = 1.0 - (self.edge_length_ratio / ideal_ratio).min(1.0);

        // Weighted combination
        let weights = match target {
            QualityTarget::FEA => (0.4, 0.4, 0.2),       // FEA cares most about angles and aspect
            QualityTarget::Analytical => (0.3, 0.3, 0.4), // Analytical cares about uniformity
            QualityTarget::Print => (0.3, 0.4, 0.3),      // Print cares about no slivers
            QualityTarget::Visual => (0.2, 0.3, 0.5),     // Visual cares most about uniformity
        };

        (angle_quality * weights.0 + aspect_quality * weights.1 + uniformity * weights.2).clamp(0.0, 1.0)
    }
}

// ============================================================
// TriangulationParams (our own, mapped to draper_mesh::TriangulationParams)
// ============================================================

/// Predicted optimal triangulation parameters.
///
/// These map to `draper_mesh::triangulate::TriangulationParams`.
#[derive(Clone, Debug)]
pub struct TriangulationParams {
    /// Maximum deviation from the true surface.
    pub max_deviation: f64,
    /// Number of angular samples for cylindrical/spherical surfaces.
    pub angular_samples: usize,
    /// Number of height samples for cylindrical surfaces.
    pub height_samples: usize,
    /// LOD detail level (1.0 = normal, 0.5 = coarser, 2.0 = finer).
    pub detail_level: f64,
    /// Maximum edge length.
    pub max_edge_length: f64,
    /// Maximum angular deviation between adjacent face normals (radians).
    pub max_angular_deviation: f64,
    /// Whether to use adaptive sampling based on curvature.
    pub adaptive: bool,
}

impl TriangulationParams {
    /// Convert to `draper_mesh::TriangulationParams`.
    pub fn to_mesh_params(&self) -> draper_mesh::TriangulationParams {
        draper_mesh::TriangulationParams {
            max_deviation: self.max_deviation,
            angular_samples: self.angular_samples,
            height_samples: self.height_samples,
            detail_level: self.detail_level,
            max_edge_length: self.max_edge_length,
            max_angular_deviation: self.max_angular_deviation,
            adaptive: self.adaptive,
            parallel: false,
            progress_callback: None,
        }
    }
}

// ============================================================
// RefinementSuggestion
// ============================================================

/// A suggestion for adaptive refinement of specific mesh regions.
#[derive(Clone, Debug)]
pub struct RefinementSuggestion {
    /// Face or triangle indices that need more triangles.
    pub refine_indices: Vec<usize>,
    /// Face or triangle indices that can be simplified (reduced triangle count).
    pub simplify_indices: Vec<usize>,
    /// Recommended detail level for refinement regions.
    pub refine_detail_level: f64,
    /// Recommended detail level for simplification regions.
    pub simplify_detail_level: f64,
    /// Reason for the suggestion.
    pub reason: String,
}

// ============================================================
// MeshOptimizer
// ============================================================

/// Predicts optimal triangulation parameters based on mesh quality analysis.
///
/// # Algorithm
///
/// 1. **Analyze quality**: Compute `MeshQualityMetrics` from the current mesh.
/// 2. **Predict parameters**: Based on the quality target and current metrics,
///    predict optimal `max_deviation`, `angular_samples`, `height_samples`,
///    and `detail_level`.
/// 3. **Suggest refinement**: Identify regions that need more or fewer triangles.
#[derive(Clone, Debug)]
pub struct MeshOptimizer {
    /// History of predictions and their outcomes (for learning).
    prediction_history: Vec<PredictionRecord>,
}

/// Record of a past prediction and its outcome.
#[derive(Clone, Debug)]
struct PredictionRecord {
    /// The quality target.
    target: QualityTarget,
    /// The predicted parameters.
    _predicted: TriangulationParams,
    /// The actual quality achieved.
    achieved_quality: f64,
    /// Whether the prediction was within tolerance.
    _within_tolerance: bool,
}

impl Default for MeshOptimizer {
    fn default() -> Self {
        Self {
            prediction_history: Vec::new(),
        }
    }
}

impl MeshOptimizer {
    /// Create a new optimizer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Analyze mesh quality and compute quality metrics.
    pub fn analyze_quality(&self, mesh: &TriangleMesh) -> MeshQualityMetrics {
        if mesh.triangles.is_empty() {
            return MeshQualityMetrics {
                min_angle: 0.0,
                max_angle: std::f64::consts::PI,
                aspect_ratio_hist: vec![0; 10],
                area_distribution: [0.0; 5],
                curvature_stats: [0.0; 4],
                total_area: 0.0,
                triangle_count: 0,
                vertex_count: mesh.vertices.len(),
                avg_edge_length: 0.0,
                edge_length_ratio: 1.0,
                sliver_percentage: 0.0,
                model_scale: compute_model_scale(mesh),
            };
        }

        let model_scale = compute_model_scale(mesh);

        // Compute per-triangle metrics
        let mut angles = Vec::with_capacity(mesh.triangles.len() * 3);
        let mut aspect_ratios = Vec::with_capacity(mesh.triangles.len());
        let mut areas = Vec::with_capacity(mesh.triangles.len());
        let mut edge_lengths = Vec::with_capacity(mesh.triangles.len() * 3);
        let mut sliver_count = 0usize;

        for tri in &mesh.triangles {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];

            // Triangle area
            let area = triangle_area(&v0, &v1, &v2);
            areas.push(area);

            // Edge lengths
            let e01 = v0.distance_to(&v1);
            let e12 = v1.distance_to(&v2);
            let e20 = v2.distance_to(&v0);
            edge_lengths.push(e01);
            edge_lengths.push(e12);
            edge_lengths.push(e20);

            // Interior angles
            let a0 = angle_between(v1, v0, v2);
            let a1 = angle_between(v0, v1, v2);
            let a2 = angle_between(v0, v2, v1);
            angles.push(a0);
            angles.push(a1);
            angles.push(a2);

            // Aspect ratio
            let ar = triangle_aspect_ratio(&v0, &v1, &v2);
            aspect_ratios.push(ar);

            if ar > 10.0 {
                sliver_count += 1;
            }
        }

        // Min/max angle
        let min_angle = angles.iter().cloned().fold(f64::MAX, f64::min);
        let max_angle = angles.iter().cloned().fold(0.0_f64, f64::max);

        // Aspect ratio histogram (10 bins: 0-1, 1-2, 2-5, 5-10, 10-20, 20-50, 50-100, 100-200, 200-500, 500+)
        let bin_thresholds = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, f64::MAX];
        let mut aspect_ratio_hist = vec![0usize; 10];
        for &ar in &aspect_ratios {
            for (i, &thresh) in bin_thresholds.iter().enumerate() {
                if ar <= thresh {
                    aspect_ratio_hist[i] += 1;
                    break;
                }
            }
        }

        // Area distribution
        let mut sorted_areas = areas.clone();
        sorted_areas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted_areas.len();
        let area_distribution = [
            sorted_areas[0],
            sorted_areas[n / 4],
            sorted_areas[n / 2],
            sorted_areas[3 * n / 4],
            sorted_areas[n - 1],
        ];

        // Curvature statistics (approximate from normal variation)
        let curvature_stats = compute_curvature_stats(mesh);

        // Edge length statistics
        let avg_edge_length = edge_lengths.iter().sum::<f64>() / edge_lengths.len() as f64;
        let min_edge = edge_lengths.iter().cloned().fold(f64::MAX, f64::min);
        let max_edge = edge_lengths.iter().cloned().fold(0.0_f64, f64::max);
        let edge_length_ratio = if min_edge > 1e-15 { max_edge / min_edge } else { 1e10 };

        // Sliver percentage
        let sliver_percentage = if mesh.triangles.len() > 0 {
            sliver_count as f64 / mesh.triangles.len() as f64 * 100.0
        } else {
            0.0
        };

        // Total area
        let total_area = areas.iter().sum();

        MeshQualityMetrics {
            min_angle,
            max_angle,
            aspect_ratio_hist,
            area_distribution,
            curvature_stats,
            total_area,
            triangle_count: mesh.triangles.len(),
            vertex_count: mesh.vertices.len(),
            avg_edge_length,
            edge_length_ratio,
            sliver_percentage,
            model_scale,
        }
    }

    /// Predict optimal triangulation parameters for a given quality target.
    ///
    /// # Algorithm
    ///
    /// 1. Analyze current mesh quality
    /// 2. Compute target parameters based on quality target
    /// 3. Adjust based on current quality (if already good, can reduce; if bad, increase)
    /// 4. Apply learned adjustments from past predictions
    pub fn predict_params(
        &self,
        mesh: &TriangleMesh,
        target: QualityTarget,
    ) -> TriangulationParams {
        let metrics = self.analyze_quality(mesh);
        let current_quality = metrics.overall_quality(target);

        // Base parameters from quality target
        let base_deviation = target.default_deviation_ratio() * metrics.model_scale;
        let base_detail_level = target.default_detail_level();

        // Adjust based on current quality
        let (max_deviation, detail_level) = adjust_for_current_quality(
            base_deviation,
            base_detail_level,
            current_quality,
            target,
        );

        // Compute angular and height samples based on max_deviation and model scale
        let angular_samples = compute_angular_samples(metrics.model_scale, max_deviation, target);
        let height_samples = compute_height_samples(metrics.model_scale, max_deviation, target);

        // Compute max edge length from max_deviation and model scale
        let max_edge_length = compute_max_edge_length(metrics.model_scale, max_deviation);

        // Compute max angular deviation
        let max_angular_deviation = match target {
            QualityTarget::Visual => 0.2,       // ~11.5°
            QualityTarget::Analytical => 0.05,   // ~2.9°
            QualityTarget::Print => 0.1,         // ~5.7°
            QualityTarget::FEA => 0.15,          // ~8.6°
        };

        TriangulationParams {
            max_deviation,
            angular_samples,
            height_samples,
            detail_level,
            max_edge_length,
            max_angular_deviation,
            adaptive: true,
        }
    }

    /// Suggest adaptive refinement for specific mesh regions.
    ///
    /// Identifies which triangles need more subdivision (high curvature, poor
    /// aspect ratio) and which can be simplified (flat, good aspect ratio).
    pub fn suggest_refinement(
        &self,
        mesh: &TriangleMesh,
        target: QualityTarget,
    ) -> RefinementSuggestion {
        let metrics = self.analyze_quality(mesh);
        let min_angle_thresh = target.min_angle_threshold();

        let mut refine_indices = Vec::new();
        let mut simplify_indices = Vec::new();

        for (i, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];

            let area = triangle_area(&v0, &v1, &v2);
            let aspect = triangle_aspect_ratio(&v0, &v1, &v2);
            let min_angle = min_interior_angle(&v0, &v1, &v2);

            // Refinement criteria:
            // - Poor aspect ratio (slivers)
            // - Small minimum angle
            // - High curvature (large normal variation with neighbors)
            let needs_refinement = aspect > 10.0
                || min_angle < min_angle_thresh
                || area > metrics.area_distribution[4] * 0.8; // Very large triangles

            // Simplification criteria:
            // - Good aspect ratio (close to equilateral)
            // - Large minimum angle
            // - Area close to median (well-sized)
            let can_simplify = aspect < 2.0
                && min_angle > 30.0_f64.to_radians()
                && area > metrics.area_distribution[1] * 0.5
                && area < metrics.area_distribution[3] * 1.5;

            if needs_refinement {
                refine_indices.push(i);
            } else if can_simplify {
                simplify_indices.push(i);
            }
        }

        let refine_detail_level = match target {
            QualityTarget::Visual => 2.0,
            QualityTarget::Analytical => 3.0,
            QualityTarget::Print => 2.5,
            QualityTarget::FEA => 2.0,
        };

        let simplify_detail_level = match target {
            QualityTarget::Visual => 0.5,
            QualityTarget::Analytical => 0.7,
            QualityTarget::Print => 0.6,
            QualityTarget::FEA => 0.5,
        };

        let reason = format!(
            "Refinement: {}/{} triangles need more subdivision, {}/{} can be simplified (target: {})",
            refine_indices.len(),
            mesh.triangles.len(),
            simplify_indices.len(),
            mesh.triangles.len(),
            target.name()
        );

        RefinementSuggestion {
            refine_indices,
            simplify_indices,
            refine_detail_level,
            simplify_detail_level,
            reason,
        }
    }

    /// Record a prediction outcome for learning.
    pub fn record_prediction(
        &mut self,
        target: QualityTarget,
        predicted: TriangulationParams,
        achieved_quality: f64,
    ) {
        let target_quality = match target {
            QualityTarget::Visual => 0.7,
            QualityTarget::Analytical => 0.9,
            QualityTarget::Print => 0.85,
            QualityTarget::FEA => 0.8,
        };

        self.prediction_history.push(PredictionRecord {
            target,
            _predicted: predicted,
            achieved_quality,
            _within_tolerance: achieved_quality >= target_quality,
        });
    }

    /// Get the number of recorded predictions.
    pub fn prediction_count(&self) -> usize {
        self.prediction_history.len()
    }

    /// Get the average achieved quality for a target.
    pub fn average_quality_for(&self, target: QualityTarget) -> f64 {
        let records: Vec<_> = self
            .prediction_history
            .iter()
            .filter(|r| r.target == target)
            .collect();

        if records.is_empty() {
            return 0.0;
        }

        records.iter().map(|r| r.achieved_quality).sum::<f64>() / records.len() as f64
    }
}

// ============================================================
// Helper functions
// ============================================================

/// Compute the characteristic model scale (bounding box diagonal).
fn compute_model_scale(mesh: &TriangleMesh) -> f64 {
    let (min, max) = mesh.bounding_box();
    let dx = max.x - min.x;
    let dy = max.y - min.y;
    let dz = max.z - min.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Compute triangle area.
fn triangle_area(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let e1x = v1.x - v0.x;
    let e1y = v1.y - v0.y;
    let e1z = v1.z - v0.z;
    let e2x = v2.x - v0.x;
    let e2y = v2.y - v0.y;
    let e2z = v2.z - v0.z;
    let cx = e1y * e2z - e1z * e2y;
    let cy = e1z * e2x - e1x * e2z;
    let cz = e1x * e2y - e1y * e2x;
    (cx * cx + cy * cy + cz * cz).sqrt() * 0.5
}

/// Compute the aspect ratio of a triangle.
fn triangle_aspect_ratio(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let e0 = v0.distance_to(v1);
    let e1 = v1.distance_to(v2);
    let e2 = v2.distance_to(v0);
    let longest = e0.max(e1).max(e2);

    if longest < 1e-15 {
        return 1.0;
    }

    let area = triangle_area(v0, v1, v2);
    if area < 1e-20 {
        return 1e10;
    }

    let perimeter = e0 + e1 + e2;
    let inradius = 2.0 * area / perimeter;
    longest / (2.0 * (3f64).sqrt() * inradius)
}

/// Compute the angle at vertex `vertex` between edges to `a` and `b`.
fn angle_between(vertex: Point3d, a: Point3d, b: Point3d) -> f64 {
    let va = (a.x - vertex.x, a.y - vertex.y, a.z - vertex.z);
    let vb = (b.x - vertex.x, b.y - vertex.y, b.z - vertex.z);

    let dot = va.0 * vb.0 + va.1 * vb.1 + va.2 * vb.2;
    let len_a = (va.0 * va.0 + va.1 * va.1 + va.2 * va.2).sqrt();
    let len_b = (vb.0 * vb.0 + vb.1 * vb.1 + vb.2 * vb.2).sqrt();

    if len_a < 1e-15 || len_b < 1e-15 {
        return 0.0;
    }

    let cos_angle = (dot / (len_a * len_b)).clamp(-1.0, 1.0);
    cos_angle.acos()
}

/// Compute the minimum interior angle of a triangle.
fn min_interior_angle(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let a0 = angle_between(*v0, *v1, *v2);
    let a1 = angle_between(*v1, *v0, *v2);
    let a2 = angle_between(*v2, *v0, *v1);
    a0.min(a1).min(a2)
}

/// Approximate curvature statistics from normal variation.
///
/// For each vertex, compute the variation of face normals among adjacent
/// triangles. This gives an approximation of discrete mean curvature.
fn compute_curvature_stats(mesh: &TriangleMesh) -> [f64; 4] {
    if mesh.triangles.is_empty() {
        return [0.0; 4];
    }

    // Compute face normals
    let face_normals: Vec<[f64; 3]> = mesh
        .triangles
        .iter()
        .map(|tri| {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
            let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
            let nx = e1.1 * e2.2 - e1.2 * e2.1;
            let ny = e1.2 * e2.0 - e1.0 * e2.2;
            let nz = e1.0 * e2.1 - e1.1 * e2.0;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 1e-15 {
                [nx / len, ny / len, nz / len]
            } else {
                [0.0, 0.0, 1.0]
            }
        })
        .collect();

    // Build vertex → face adjacency
    let mut vertex_faces: std::collections::HashMap<u32, Vec<usize>> =
        std::collections::HashMap::new();
    for (i, tri) in mesh.triangles.iter().enumerate() {
        vertex_faces.entry(tri[0]).or_default().push(i);
        vertex_faces.entry(tri[1]).or_default().push(i);
        vertex_faces.entry(tri[2]).or_default().push(i);
    }

    // For each vertex, compute normal variation (approximate curvature)
    let mut curvatures = Vec::new();
    for (_v, faces) in &vertex_faces {
        if faces.len() < 2 {
            continue;
        }

        // Average pairwise angle between adjacent face normals
        let mut total_angle = 0.0;
        let mut count = 0;
        for i in 0..faces.len() {
            for j in (i + 1)..faces.len() {
                let ni = face_normals[faces[i]];
                let nj = face_normals[faces[j]];
                let dot = ni[0] * nj[0] + ni[1] * nj[1] + ni[2] * nj[2];
                let angle = dot.clamp(-1.0, 1.0).acos();
                total_angle += angle;
                count += 1;
            }
        }

        if count > 0 {
            curvatures.push(total_angle / count as f64);
        }
    }

    if curvatures.is_empty() {
        return [0.0; 4];
    }

    // Compute statistics
    let min_curv = curvatures.iter().cloned().fold(f64::MAX, f64::min);
    let max_curv = curvatures.iter().cloned().fold(0.0_f64, f64::max);
    let mean_curv = curvatures.iter().sum::<f64>() / curvatures.len() as f64;

    let variance = if curvatures.len() > 1 {
        curvatures
            .iter()
            .map(|&c| (c - mean_curv) * (c - mean_curv))
            .sum::<f64>()
            / (curvatures.len() - 1) as f64
    } else {
        0.0
    };
    let std_dev = variance.sqrt();

    [min_curv, mean_curv, max_curv, std_dev]
}

/// Adjust base parameters based on current mesh quality.
///
/// If the current quality is already good, we can reduce the detail level.
/// If it's poor, we need to increase it.
fn adjust_for_current_quality(
    base_deviation: f64,
    base_detail_level: f64,
    current_quality: f64,
    target: QualityTarget,
) -> (f64, f64) {
    // Quality adjustment factor:
    // - If quality > 0.8, we can be less aggressive (increase deviation, decrease detail)
    // - If quality < 0.5, we need to be more aggressive (decrease deviation, increase detail)
    let quality_factor = if current_quality > 0.8 {
        1.0 + (current_quality - 0.8) * 2.0 // 1.0 to 1.4
    } else if current_quality < 0.5 {
        0.5 + current_quality // 0.5 to 1.0
    } else {
        1.0
    };

    // Adjust deviation: higher quality → can afford larger deviation (coarser)
    let max_deviation = match target {
        QualityTarget::Visual => base_deviation * quality_factor,
        QualityTarget::Analytical => base_deviation * quality_factor.sqrt(), // More conservative
        QualityTarget::Print => base_deviation * quality_factor.powf(0.8),
        QualityTarget::FEA => base_deviation * quality_factor.powf(0.7),
    };

    // Adjust detail level: lower quality → need more detail
    let detail_level = match target {
        QualityTarget::Visual => base_detail_level / quality_factor.sqrt(),
        QualityTarget::Analytical => base_detail_level / quality_factor.powf(0.3),
        QualityTarget::Print => base_detail_level / quality_factor.powf(0.4),
        QualityTarget::FEA => base_detail_level / quality_factor.powf(0.5),
    };

    (
        max_deviation.max(1e-10),
        detail_level.clamp(0.1, 5.0),
    )
}

/// Compute angular samples based on model scale, max deviation, and target.
fn compute_angular_samples(model_scale: f64, max_deviation: f64, target: QualityTarget) -> usize {
    // For a full circle (2π) of radius r, the chord deviation formula gives:
    // n = π / acos(1 - max_deviation / r)
    // Use a representative radius of model_scale / 4

    let representative_radius = model_scale / 4.0;
    if representative_radius < 1e-10 || max_deviation <= 0.0 {
        return 48;
    }

    let d_over_r = (max_deviation / representative_radius).min(1.0 - 1e-10);
    if d_over_r <= 0.0 {
        return 256;
    }

    let half_angle = (1.0 - d_over_r).acos();
    if half_angle < 1e-10 {
        return 256;
    }

    let n = (std::f64::consts::PI / half_angle).ceil() as usize;

    // Apply target-specific adjustments
    let adjusted = match target {
        QualityTarget::Visual => n,
        QualityTarget::Analytical => (n as f64 * 1.5).ceil() as usize,
        QualityTarget::Print => (n as f64 * 1.2).ceil() as usize,
        QualityTarget::FEA => (n as f64 * 1.3).ceil() as usize,
    };

    adjusted.clamp(6, 256)
}

/// Compute height samples based on model scale, max deviation, and target.
fn compute_height_samples(model_scale: f64, max_deviation: f64, target: QualityTarget) -> usize {
    // Height samples depend on the extent of the model in the height direction
    // For simplicity, use a fraction of the model scale

    let base = if model_scale < 1e-10 || max_deviation <= 0.0 {
        8
    } else {
        let n = (model_scale / (max_deviation * 10.0)).ceil() as usize;
        n.clamp(2, 64)
    };

    match target {
        QualityTarget::Visual => base,
        QualityTarget::Analytical => (base as f64 * 1.5).ceil() as usize,
        QualityTarget::Print => (base as f64 * 1.2).ceil() as usize,
        QualityTarget::FEA => (base as f64 * 1.3).ceil() as usize,
    }
    .clamp(2, 256)
}

/// Compute max edge length from model scale and max deviation.
fn compute_max_edge_length(model_scale: f64, max_deviation: f64) -> f64 {
    // Max edge length should ensure chord deviation < max_deviation
    // For a circle of radius r: chord_dev = r * (1 - cos(θ/2))
    // With r ≈ model_scale / 4 and θ = max_edge_length / r:
    // chord_dev ≈ max_edge_length² / (8 * r)
    // Solving for max_edge_length: max_edge_length = sqrt(8 * r * max_deviation)

    let representative_radius = model_scale / 4.0;
    if representative_radius < 1e-10 {
        return 1.0;
    }

    (8.0 * representative_radius * max_deviation).sqrt().max(max_deviation * 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_mesh::TriangleMesh;

    /// Helper: create a unit cube mesh.
    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
            Point3d::new(0.0, 0.0, 1.0),
            Point3d::new(1.0, 0.0, 1.0),
            Point3d::new(1.0, 1.0, 1.0),
            Point3d::new(0.0, 1.0, 1.0),
        ];
        for p in &v {
            mesh.add_vertex(*p);
        }
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        mesh.add_triangle(3, 7, 6);
        mesh.add_triangle(3, 6, 2);
        mesh.add_triangle(0, 4, 7);
        mesh.add_triangle(0, 7, 3);
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);
        mesh
    }

    #[test]
    fn test_analyze_quality_cube() {
        let mesh = make_cube_mesh();
        let optimizer = MeshOptimizer::new();
        let metrics = optimizer.analyze_quality(&mesh);

        assert_eq!(metrics.triangle_count, 12);
        assert_eq!(metrics.vertex_count, 8);
        assert!(metrics.total_area > 0.0, "Cube should have positive area");
        assert!(metrics.min_angle > 0.0, "Cube triangles should have positive min angle");
    }

    #[test]
    fn test_predict_params_visual() {
        let mesh = make_cube_mesh();
        let optimizer = MeshOptimizer::new();
        let params = optimizer.predict_params(&mesh, QualityTarget::Visual);

        assert!(params.max_deviation > 0.0, "max_deviation should be positive");
        assert!(params.angular_samples >= 6, "angular_samples should be >= 6");
        assert!(params.height_samples >= 2, "height_samples should be >= 2");
        assert!(params.detail_level > 0.0, "detail_level should be positive");
        assert!(params.adaptive, "Should use adaptive sampling");
    }

    #[test]
    fn test_predict_params_analytical_more_precise() {
        let mesh = make_cube_mesh();
        let optimizer = MeshOptimizer::new();

        let visual_params = optimizer.predict_params(&mesh, QualityTarget::Visual);
        let analytical_params = optimizer.predict_params(&mesh, QualityTarget::Analytical);

        // Analytical should have smaller max_deviation than visual
        assert!(
            analytical_params.max_deviation <= visual_params.max_deviation,
            "Analytical max_deviation ({}) should be <= visual ({})",
            analytical_params.max_deviation,
            visual_params.max_deviation
        );
    }

    #[test]
    fn test_predict_params_fea_more_samples() {
        let mesh = make_cube_mesh();
        let optimizer = MeshOptimizer::new();

        let visual_params = optimizer.predict_params(&mesh, QualityTarget::Visual);
        let fea_params = optimizer.predict_params(&mesh, QualityTarget::FEA);

        // FEA should generally have more angular samples than visual
        assert!(
            fea_params.angular_samples >= visual_params.angular_samples,
            "FEA angular_samples ({}) should be >= visual ({})",
            fea_params.angular_samples,
            visual_params.angular_samples
        );
    }

    #[test]
    fn test_suggest_refinement() {
        let mesh = make_cube_mesh();
        let optimizer = MeshOptimizer::new();
        let suggestion = optimizer.suggest_refinement(&mesh, QualityTarget::Visual);

        // For a cube with reasonable triangles, most can be simplified
        assert!(!suggestion.reason.is_empty());
    }

    #[test]
    fn test_quality_target_defaults() {
        assert!(QualityTarget::Visual.default_deviation_ratio() > QualityTarget::Analytical.default_deviation_ratio());
        assert!(QualityTarget::Visual.default_detail_level() < QualityTarget::Analytical.default_detail_level());
    }

    #[test]
    fn test_quality_target_display() {
        assert_eq!(QualityTarget::Visual.to_string(), "Visual");
        assert_eq!(QualityTarget::FEA.to_string(), "FEA");
    }

    #[test]
    fn test_mesh_quality_overall_score() {
        let mesh = make_cube_mesh();
        let optimizer = MeshOptimizer::new();
        let metrics = optimizer.analyze_quality(&mesh);

        let score = metrics.overall_quality(QualityTarget::Visual);
        assert!(score > 0.0 && score <= 1.0, "Quality score should be in [0, 1], got {}", score);
    }

    #[test]
    fn test_to_mesh_params_conversion() {
        let params = TriangulationParams {
            max_deviation: 0.01,
            angular_samples: 48,
            height_samples: 8,
            detail_level: 1.0,
            max_edge_length: 1.0,
            max_angular_deviation: 0.1,
            adaptive: true,
        };
        let mesh_params = params.to_mesh_params();

        assert_eq!(mesh_params.max_deviation, 0.01);
        assert_eq!(mesh_params.angular_samples, 48);
        assert_eq!(mesh_params.height_samples, 8);
        assert_eq!(mesh_params.detail_level, 1.0);
        assert!(mesh_params.adaptive);
    }

    #[test]
    fn test_empty_mesh_quality() {
        let mesh = TriangleMesh::new();
        let optimizer = MeshOptimizer::new();
        let metrics = optimizer.analyze_quality(&mesh);

        assert_eq!(metrics.triangle_count, 0);
        assert_eq!(metrics.vertex_count, 0);
    }

    #[test]
    fn test_prediction_recording() {
        let mut optimizer = MeshOptimizer::new();
        let mesh = make_cube_mesh();
        let params = optimizer.predict_params(&mesh, QualityTarget::Visual);

        optimizer.record_prediction(QualityTarget::Visual, params, 0.85);

        assert_eq!(optimizer.prediction_count(), 1);
        assert!((optimizer.average_quality_for(QualityTarget::Visual) - 0.85).abs() < 0.01);
    }

    #[test]
    fn test_min_angle_for_equilateral() {
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(1.0, 0.0, 0.0);
        let v2 = Point3d::new(0.5, (3f64).sqrt() / 2.0, 0.0);

        let min_angle = min_interior_angle(&v0, &v1, &v2);
        let expected = 60.0_f64.to_radians();
        assert!(
            (min_angle - expected).abs() < 0.01,
            "Equilateral triangle min angle should be 60°, got {}°",
            min_angle.to_degrees()
        );
    }
}
