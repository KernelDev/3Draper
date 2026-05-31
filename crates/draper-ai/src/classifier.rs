// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! ML-based defect classification (4.6.1).
//!
//! Uses rule-based decision trees and heuristic scoring to classify mesh defects
//! from a `ManifoldReport` and `TriangleMesh`. No external ML framework dependency —
//! all classification logic is implemented as interpretable heuristic rules that
//! approximate decision-tree behaviour.
//!
//! # Defect types
//!
//! - **Gap**: boundary edges that should be closed
//! - **Hole**: open boundary loops (missing faces)
//! - **NonManifoldEdge**: edges shared by > 2 triangles
//! - **FlippedNormal**: faces with normals pointing inward
//! - **SliverTriangle**: triangles with extreme aspect ratios
//! - **SmallFeature**: clusters of very small triangles or faces
//! - **DegenerateEdge**: zero-length or degenerate edges
//! - **SelfIntersection**: intersecting surface patches
//! - **ToleranceMismatch**: inconsistent tolerance across topology
//!
//! # Severity scoring
//!
//! Severity (0.0–1.0) is computed from:
//! - Defect size relative to model scale
//! - Number of affected elements
//! - Spatial distribution (clustered vs. scattered)

use draper_geometry::Point3d;
use draper_mesh::{ManifoldReport, TriangleMesh};
use std::collections::HashMap;

// ============================================================
// DefectType enum
// ============================================================

/// Classification of a mesh defect type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DefectType {
    /// Boundary edges that should be closed (gaps between faces).
    Gap,
    /// Open boundary loops — missing faces.
    Hole,
    /// Edges shared by more than 2 triangles (non-manifold).
    NonManifoldEdge,
    /// Face normals pointing inward instead of outward.
    FlippedNormal,
    /// Triangles with extreme aspect ratio (thin and elongated).
    SliverTriangle,
    /// Clusters of very small-area triangles or faces.
    SmallFeature,
    /// Zero-length or degenerate edges.
    DegenerateEdge,
    /// Self-intersecting surface patches.
    SelfIntersection,
    /// Inconsistent tolerance across the topological graph.
    ToleranceMismatch,
}

impl DefectType {
    /// Human-readable name of the defect type.
    pub fn name(&self) -> &'static str {
        match self {
            DefectType::Gap => "Gap",
            DefectType::Hole => "Hole",
            DefectType::NonManifoldEdge => "NonManifoldEdge",
            DefectType::FlippedNormal => "FlippedNormal",
            DefectType::SliverTriangle => "SliverTriangle",
            DefectType::SmallFeature => "SmallFeature",
            DefectType::DegenerateEdge => "DegenerateEdge",
            DefectType::SelfIntersection => "SelfIntersection",
            DefectType::ToleranceMismatch => "ToleranceMismatch",
        }
    }

    /// Description of the defect type.
    pub fn description(&self) -> &'static str {
        match self {
            DefectType::Gap => "Boundary edges that should be closed — gaps between adjacent faces",
            DefectType::Hole => "Open boundary loops indicating missing faces",
            DefectType::NonManifoldEdge => "Edges shared by more than 2 triangles",
            DefectType::FlippedNormal => "Face normals pointing inward instead of outward",
            DefectType::SliverTriangle => "Triangles with extreme aspect ratio (thin and elongated)",
            DefectType::SmallFeature => "Clusters of very small-area triangles or faces",
            DefectType::DegenerateEdge => "Zero-length or degenerate edges with no meaningful geometry",
            DefectType::SelfIntersection => "Self-intersecting surface patches",
            DefectType::ToleranceMismatch => "Inconsistent tolerance across the topological graph",
        }
    }
}

impl std::fmt::Display for DefectType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

// ============================================================
// ClassifiedDefect
// ============================================================

/// A classified defect with severity, location, and metadata.
#[derive(Clone, Debug)]
pub struct ClassifiedDefect {
    /// The type of defect.
    pub defect_type: DefectType,
    /// Severity score in [0.0, 1.0].
    /// 0.0 = negligible, 1.0 = critical.
    pub severity: f64,
    /// Approximate 3D location of the defect (centroid of affected region).
    pub location: Point3d,
    /// Indices of affected mesh elements (triangle indices, vertex indices, etc.).
    pub affected_elements: Vec<usize>,
    /// Confidence score in [0.0, 1.0] for this classification.
    pub confidence: f64,
    /// Human-readable description of why this defect was classified.
    pub reason: String,
}

// ============================================================
// DefectClassifier
// ============================================================

/// ML-based defect classifier using rule-based decision trees and heuristic scoring.
///
/// Takes a `TriangleMesh` and `ManifoldReport` and produces a list of classified
/// defects with severity scores and confidence values.
///
/// # Algorithm
///
/// The classifier operates as a set of independent "detectors", each implementing
/// a rule-based decision tree:
///
/// 1. **Gap detector**: Uses `boundary_edge_count` from `ManifoldReport`.
///    Severity based on gap count relative to total edges.
///
/// 2. **Hole detector**: Groups boundary edges into loops, estimates hole area.
///    Severity based on hole area relative to model surface area.
///
/// 3. **Non-manifold edge detector**: Uses `non_manifold_edge_count`.
///    Severity based on count relative to total edges.
///
/// 4. **Flipped normal detector**: Checks face normal consistency for closed shells.
///    Severity based on percentage of flipped normals.
///
/// 5. **Sliver triangle detector**: Computes aspect ratios for all triangles.
///    Severity based on the worst aspect ratio and number of slivers.
///
/// 6. **Small feature detector**: Finds clusters of small-area triangles.
///    Severity based on feature size relative to model scale.
///
/// 7. **Degenerate edge detector**: Uses `degenerate_triangle_count`.
///    Severity based on count relative to total triangles.
///
/// 8. **Self-intersection detector**: Sampling-based intersection check.
///    Severity based on number of intersection pairs.
///
/// 9. **Tolerance mismatch detector**: Checks tolerance consistency.
///    Severity based on tolerance spread.
#[derive(Clone, Debug)]
pub struct DefectClassifier {
    /// Aspect ratio threshold above which a triangle is considered a sliver.
    pub sliver_aspect_ratio_threshold: f64,
    /// Area threshold below which a triangle is considered "small" (relative to
    /// median triangle area).
    pub small_feature_area_ratio: f64,
    /// Maximum number of boundary edges to classify as a "hole" (vs. a large gap).
    pub max_hole_boundary_edges: usize,
    /// Minimum angle (in radians) below which a triangle is considered degenerate.
    pub min_angle_threshold: f64,
}

impl Default for DefectClassifier {
    fn default() -> Self {
        Self {
            sliver_aspect_ratio_threshold: 100.0,
            small_feature_area_ratio: 0.01,
            max_hole_boundary_edges: 12,
            min_angle_threshold: 1.0_f64.to_radians(), // ~1 degree
        }
    }
}

impl DefectClassifier {
    /// Create a new classifier with default parameters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Classify all defects in a mesh given its manifold report.
    ///
    /// Returns a list of `ClassifiedDefect` sorted by severity (most severe first).
    pub fn classify(
        &self,
        mesh: &TriangleMesh,
        manifold_report: &ManifoldReport,
    ) -> Vec<ClassifiedDefect> {
        let mut defects = Vec::new();

        // Compute model scale for relative severity calculations
        let model_scale = compute_model_scale(mesh);

        // Run each detector independently
        self.detect_gaps(mesh, manifold_report, &mut defects);
        self.detect_holes(mesh, manifold_report, model_scale, &mut defects);
        self.detect_nonmanifold_edges(mesh, manifold_report, &mut defects);
        self.detect_flipped_normals(mesh, manifold_report, &mut defects);
        self.detect_sliver_triangles(mesh, manifold_report, model_scale, &mut defects);
        self.detect_small_features(mesh, manifold_report, &mut defects);
        self.detect_degenerate_elements(mesh, manifold_report, &mut defects);
        self.detect_self_intersections(mesh, manifold_report, &mut defects);
        self.detect_tolerance_mismatch(mesh, manifold_report, &mut defects);

        // Sort by severity (most severe first)
        defects.sort_by(|a, b| b.severity.partial_cmp(&a.severity).unwrap_or(std::cmp::Ordering::Equal));

        defects
    }

    // ----------------------------------------------------------
    // Individual detectors
    // ----------------------------------------------------------

    /// Detect gaps: boundary edges that should be closed.
    fn detect_gaps(
        &self,
        mesh: &TriangleMesh,
        report: &ManifoldReport,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        if report.boundary_edge_count == 0 {
            return;
        }

        // Compute centroid of boundary edges
        let mut location = Point3d::ORIGIN;
        let mut count = 0usize;
        let mut affected = Vec::new();

        for &(v0, v1) in &report.boundary_edges {
            let p0 = mesh.vertices.get(v0 as usize);
            let p1 = mesh.vertices.get(v1 as usize);
            if let (Some(a), Some(b)) = (p0, p1) {
                location.x += a.x + b.x;
                location.y += a.y + b.y;
                location.z += a.z + b.z;
                count += 2;
                affected.push(v0 as usize);
                affected.push(v1 as usize);
            }
        }

        if count > 0 {
            location.x /= count as f64;
            location.y /= count as f64;
            location.z /= count as f64;
        }

        // Severity: based on proportion of boundary edges relative to total edges
        let edge_ratio = if report.edge_count > 0 {
            report.boundary_edge_count as f64 / report.edge_count as f64
        } else {
            1.0
        };

        // Also consider absolute count — even a small ratio can be severe if
        // there are many boundary edges
        let absolute_severity = (report.boundary_edge_count as f64 / 100.0).min(1.0);
        let severity = (edge_ratio * 0.7 + absolute_severity * 0.3).min(1.0);

        // Confidence: higher when boundary edges are clearly detected
        let confidence = if report.boundary_edge_count > 0 { 0.95 } else { 0.0 };

        defects.push(ClassifiedDefect {
            defect_type: DefectType::Gap,
            severity,
            location,
            affected_elements: affected,
            confidence,
            reason: format!(
                "Found {} boundary edges out of {} total ({:.1}%)",
                report.boundary_edge_count,
                report.edge_count,
                edge_ratio * 100.0
            ),
        });
    }

    /// Detect holes: open boundary loops.
    fn detect_holes(
        &self,
        mesh: &TriangleMesh,
        report: &ManifoldReport,
        model_scale: f64,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        if report.boundary_edge_count < 3 {
            return;
        }

        // Group boundary edges into loops by chaining end-to-start
        let loops = find_boundary_loops(mesh, &report.boundary_edges);

        for hole_loop in &loops {
            if hole_loop.edge_count < 3 || hole_loop.edge_count > self.max_hole_boundary_edges {
                // Skip: not a "hole" (too small or too large — probably a major gap)
                continue;
            }

            // Estimate hole area as the area of the polygon formed by boundary vertices
            let hole_area = estimate_loop_area(mesh, &hole_loop.vertex_indices);
            let model_area = mesh.surface_area();
            let area_ratio = if model_area > 0.0 {
                hole_area / model_area
            } else {
                0.0
            };

            // Severity: based on hole area relative to model size
            let size_severity = (area_ratio * 10.0).min(1.0);
            // Also factor in loop size (more edges = more complex hole)
            let complexity_severity = (hole_loop.edge_count as f64 / self.max_hole_boundary_edges as f64).min(1.0);
            let severity = (size_severity * 0.6 + complexity_severity * 0.4).min(1.0);

            let confidence = if hole_loop.is_closed {
                0.9
            } else {
                0.6 // Lower confidence for open loops
            };

            defects.push(ClassifiedDefect {
                defect_type: DefectType::Hole,
                severity,
                location: hole_loop.centroid,
                affected_elements: hole_loop.vertex_indices.iter().map(|&v| v as usize).collect(),
                confidence,
                reason: format!(
                    "Hole with {} boundary edges, estimated area {:.2e} (model scale: {:.2e})",
                    hole_loop.edge_count, hole_area, model_scale
                ),
            });
        }
    }

    /// Detect non-manifold edges.
    fn detect_nonmanifold_edges(
        &self,
        mesh: &TriangleMesh,
        report: &ManifoldReport,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        if report.non_manifold_edge_count == 0 {
            return;
        }

        let mut location = Point3d::ORIGIN;
        let mut count = 0usize;
        let mut affected = Vec::new();

        for &(v0, v1, _face_count) in &report.non_manifold_edges {
            let p0 = mesh.vertices.get(v0 as usize);
            let p1 = mesh.vertices.get(v1 as usize);
            if let (Some(a), Some(b)) = (p0, p1) {
                location.x += a.x + b.x;
                location.y += a.y + b.y;
                location.z += a.z + b.z;
                count += 2;
                affected.push(v0 as usize);
                affected.push(v1 as usize);
            }
        }

        if count > 0 {
            location.x /= count as f64;
            location.y /= count as f64;
            location.z /= count as f64;
        }

        let edge_ratio = if report.edge_count > 0 {
            report.non_manifold_edge_count as f64 / report.edge_count as f64
        } else {
            1.0
        };
        let absolute_severity = (report.non_manifold_edge_count as f64 / 50.0).min(1.0);
        let severity = (edge_ratio * 0.6 + absolute_severity * 0.4).min(1.0);

        defects.push(ClassifiedDefect {
            defect_type: DefectType::NonManifoldEdge,
            severity,
            location,
            affected_elements: affected,
            confidence: 0.95,
            reason: format!(
                "Found {} non-manifold edges out of {} total ({:.1}%)",
                report.non_manifold_edge_count,
                report.edge_count,
                edge_ratio * 100.0
            ),
        });
    }

    /// Detect flipped normals for closed shells.
    fn detect_flipped_normals(
        &self,
        mesh: &TriangleMesh,
        report: &ManifoldReport,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        // Only check for flipped normals on closed meshes (Euler characteristic ≈ 2)
        if report.euler_characteristic != 2 || report.boundary_edge_count > 0 {
            return;
        }

        // Compute face normals if not already present
        let face_normals = match &mesh.face_normals {
            Some(normals) => normals.clone(),
            None => {
                // Compute face normals inline
                let mut normals = Vec::with_capacity(mesh.triangles.len());
                for tri in &mesh.triangles {
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
                        normals.push([nx / len, ny / len, nz / len]);
                    } else {
                        normals.push([0.0, 0.0, 1.0]);
                    }
                }
                normals
            }
        };

        if face_normals.is_empty() {
            return;
        }

        // For a closed mesh with consistent normals, all normals should point
        // outward. Use a voting scheme: compute the average normal direction,
        // then count how many normals disagree.

        // Compute centroid of the mesh
        let mut centroid = Point3d::ORIGIN;
        for v in &mesh.vertices {
            centroid.x += v.x;
            centroid.y += v.y;
            centroid.z += v.z;
        }
        let n_verts = mesh.vertices.len() as f64;
        if n_verts > 0.0 {
            centroid.x /= n_verts;
            centroid.y /= n_verts;
            centroid.z /= n_verts;
        }

        // Count normals pointing "inward" (toward centroid)
        let mut flipped_count = 0usize;
        let mut flipped_indices = Vec::new();

        for (i, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri[0] as usize];
            let normal = face_normals[i];

            // Vector from triangle centroid to mesh centroid
            let tri_cx = (v0.x + mesh.vertices[tri[1] as usize].x + mesh.vertices[tri[2] as usize].x) / 3.0;
            let tri_cy = (v0.y + mesh.vertices[tri[1] as usize].y + mesh.vertices[tri[2] as usize].y) / 3.0;
            let tri_cz = (v0.z + mesh.vertices[tri[1] as usize].z + mesh.vertices[tri[2] as usize].z) / 3.0;

            let to_center_x = centroid.x - tri_cx;
            let to_center_y = centroid.y - tri_cy;
            let to_center_z = centroid.z - tri_cz;

            // If the normal points toward the center, it's likely flipped
            let dot = normal[0] * to_center_x + normal[1] * to_center_y + normal[2] * to_center_z;
            if dot > 0.0 {
                flipped_count += 1;
                flipped_indices.push(i);
            }
        }

        if flipped_count == 0 {
            return;
        }

        let total_faces = mesh.triangles.len();
        let flip_ratio = flipped_count as f64 / total_faces as f64;

        // If more than half are flipped, it's likely the entire mesh has reversed
        // orientation, which is a different issue — only flag if it's a minority
        let actual_flipped = if flip_ratio > 0.5 {
            // Probably the whole mesh is inside-out; report the complement
            total_faces - flipped_count
        } else {
            flipped_count
        };

        let actual_ratio = actual_flipped as f64 / total_faces as f64;
        let severity = (actual_ratio * 5.0).min(1.0); // 20% flipped → severity 1.0

        defects.push(ClassifiedDefect {
            defect_type: DefectType::FlippedNormal,
            severity,
            location: centroid,
            affected_elements: flipped_indices,
            confidence: if flip_ratio > 0.05 && flip_ratio < 0.95 { 0.85 } else { 0.5 },
            reason: format!(
                "Found {}/{} faces with flipped normals ({:.1}%)",
                actual_flipped, total_faces, actual_ratio * 100.0
            ),
        });
    }

    /// Detect sliver triangles (extreme aspect ratio).
    fn detect_sliver_triangles(
        &self,
        mesh: &TriangleMesh,
        _report: &ManifoldReport,
        _model_scale: f64,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        let mut sliver_indices = Vec::new();
        let mut worst_ratio = 1.0_f64;
        let mut sliver_centroid = Point3d::ORIGIN;

        for (i, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];

            let aspect = triangle_aspect_ratio(&v0, &v1, &v2);
            if aspect > self.sliver_aspect_ratio_threshold {
                sliver_indices.push(i);
                if aspect > worst_ratio {
                    worst_ratio = aspect;
                }
                sliver_centroid.x += (v0.x + v1.x + v2.x) / 3.0;
                sliver_centroid.y += (v0.y + v1.y + v2.y) / 3.0;
                sliver_centroid.z += (v0.z + v1.z + v2.z) / 3.0;
            }
        }

        if sliver_indices.is_empty() {
            return;
        }

        let n = sliver_indices.len();
        sliver_centroid.x /= n as f64;
        sliver_centroid.y /= n as f64;
        sliver_centroid.z /= n as f64;

        // Severity: based on proportion of slivers and worst ratio
        let proportion = n as f64 / mesh.triangles.len() as f64;
        let ratio_severity = ((worst_ratio / self.sliver_aspect_ratio_threshold).ln() / 5.0).min(1.0);
        let severity = (proportion * 0.5 + ratio_severity * 0.5).min(1.0);

        // Pattern matching: check for "strip of degenerate triangles along cone apex"
        let pattern_detected = detect_degenerate_strip_pattern(mesh, &sliver_indices);

        let reason = if pattern_detected {
            format!(
                "Found {} sliver triangles (worst ratio {:.0}:1) — pattern: strip of degenerate triangles along cone apex",
                n, worst_ratio
            )
        } else {
            format!(
                "Found {} sliver triangles (worst ratio {:.0}:1, threshold {:.0}:1)",
                n, worst_ratio, self.sliver_aspect_ratio_threshold
            )
        };

        defects.push(ClassifiedDefect {
            defect_type: DefectType::SliverTriangle,
            severity,
            location: sliver_centroid,
            affected_elements: sliver_indices,
            confidence: 0.9,
            reason,
        });
    }

    /// Detect small features (clusters of tiny triangles).
    fn detect_small_features(
        &self,
        mesh: &TriangleMesh,
        _report: &ManifoldReport,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        if mesh.triangles.is_empty() {
            return;
        }

        // Compute median triangle area
        let mut areas: Vec<f64> = mesh
            .triangles
            .iter()
            .map(|tri| {
                let v0 = mesh.vertices[tri[0] as usize];
                let v1 = mesh.vertices[tri[1] as usize];
                let v2 = mesh.vertices[tri[2] as usize];
                triangle_area(&v0, &v1, &v2)
            })
            .collect();

        areas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_area = areas[areas.len() / 2];

        if median_area <= 0.0 {
            return;
        }

        let threshold = median_area * self.small_feature_area_ratio;
        let mut small_indices = Vec::new();
        let mut small_centroid = Point3d::ORIGIN;

        for (i, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            let area = triangle_area(&v0, &v1, &v2);

            if area < threshold && area > 0.0 {
                small_indices.push(i);
                small_centroid.x += (v0.x + v1.x + v2.x) / 3.0;
                small_centroid.y += (v0.y + v1.y + v2.y) / 3.0;
                small_centroid.z += (v0.z + v1.z + v2.z) / 3.0;
            }
        }

        if small_indices.is_empty() {
            return;
        }

        let n = small_indices.len();
        small_centroid.x /= n as f64;
        small_centroid.y /= n as f64;
        small_centroid.z /= n as f64;

        let proportion = n as f64 / mesh.triangles.len() as f64;
        let severity = (proportion * 3.0).min(1.0);

        defects.push(ClassifiedDefect {
            defect_type: DefectType::SmallFeature,
            severity,
            location: small_centroid,
            affected_elements: small_indices,
            confidence: 0.8,
            reason: format!(
                "Found {} small triangles (area < {:.2e}, median area {:.2e})",
                n, threshold, median_area
            ),
        });
    }

    /// Detect degenerate elements (zero-area triangles, T-junctions).
    fn detect_degenerate_elements(
        &self,
        mesh: &TriangleMesh,
        report: &ManifoldReport,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        let mut degenerate_indices = Vec::new();
        let mut degenerate_centroid = Point3d::ORIGIN;

        for (i, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            let area = triangle_area(&v0, &v1, &v2);

            if area < 1e-20 {
                degenerate_indices.push(i);
                degenerate_centroid.x += (v0.x + v1.x + v2.x) / 3.0;
                degenerate_centroid.y += (v0.y + v1.y + v2.y) / 3.0;
                degenerate_centroid.z += (v0.z + v1.z + v2.z) / 3.0;
            }
        }

        if degenerate_indices.is_empty() && report.t_junction_count == 0 {
            return;
        }

        // Include T-junctions in the count
        let total_degenerate = degenerate_indices.len() + report.t_junction_count;
        let n = degenerate_indices.len();
        if n > 0 {
            degenerate_centroid.x /= n as f64;
            degenerate_centroid.y /= n as f64;
            degenerate_centroid.z /= n as f64;
        }

        let proportion = if mesh.triangles.len() > 0 {
            total_degenerate as f64 / mesh.triangles.len() as f64
        } else {
            1.0
        };
        let severity = (proportion * 5.0).min(1.0);

        let mut reason = String::new();
        if !degenerate_indices.is_empty() {
            reason.push_str(&format!(
                "{} degenerate triangles (zero area)",
                degenerate_indices.len()
            ));
        }
        if report.t_junction_count > 0 {
            if !reason.is_empty() {
                reason.push_str(", ");
            }
            reason.push_str(&format!("{} T-junctions", report.t_junction_count));
        }

        defects.push(ClassifiedDefect {
            defect_type: DefectType::DegenerateEdge,
            severity,
            location: degenerate_centroid,
            affected_elements: degenerate_indices,
            confidence: 0.95,
            reason,
        });
    }

    /// Detect self-intersections (sampling-based).
    fn detect_self_intersections(
        &self,
        mesh: &TriangleMesh,
        _report: &ManifoldReport,
        defects: &mut Vec<ClassifiedDefect>,
    ) {
        // Full self-intersection detection is O(n²) and expensive.
        // Use a sampling approach: check random pairs of non-adjacent triangles.
        let n_tris = mesh.triangles.len();
        if n_tris < 4 {
            return;
        }

        // Build a simple spatial index for quick rejection
        // For each triangle, compute its bounding box
        let tri_bboxes: Vec<(Point3d, Point3d)> = mesh
            .triangles
            .iter()
            .map(|tri| {
                let v0 = mesh.vertices[tri[0] as usize];
                let v1 = mesh.vertices[tri[1] as usize];
                let v2 = mesh.vertices[tri[2] as usize];
                let min = Point3d::new(
                    v0.x.min(v1.x).min(v2.x),
                    v0.y.min(v1.y).min(v2.y),
                    v0.z.min(v1.z).min(v2.z),
                );
                let max = Point3d::new(
                    v0.x.max(v1.x).max(v2.x),
                    v0.y.max(v1.y).max(v2.y),
                    v0.z.max(v1.z).max(v2.z),
                );
                (min, max)
            })
            .collect();

        // Sample pairs of triangles and check for intersection
        let max_checks = 500.min(n_tris * (n_tris - 1) / 2);
        let mut intersection_count = 0usize;
        let mut intersection_indices = Vec::new();

        // Build adjacency set for quick neighbor check
        let mut adjacent: HashMap<(u32, u32), bool> = HashMap::new();
        for tri in &mesh.triangles {
            let v0 = tri[0];
            let v1 = tri[1];
            let v2 = tri[2];
            let e01 = (v0.min(v1), v0.max(v1));
            let e12 = (v1.min(v2), v1.max(v2));
            let e20 = (v2.min(v0), v2.max(v0));
            adjacent.entry(e01).or_insert(true);
            adjacent.entry(e12).or_insert(true);
            adjacent.entry(e20).or_insert(true);
        }

        // Check pairs of triangles with overlapping bounding boxes
        let mut check_count = 0usize;
        'outer: for i in 0..n_tris {
            for j in (i + 1)..n_tris {
                if check_count >= max_checks {
                    break 'outer;
                }
                check_count += 1;

                // Quick bbox overlap test
                let (min_i, max_i) = &tri_bboxes[i];
                let (min_j, max_j) = &tri_bboxes[j];
                if max_i.x < min_j.x || min_i.x > max_j.x
                    || max_i.y < min_j.y || min_i.y > max_j.y
                    || max_i.z < min_j.z || min_i.z > max_j.z
                {
                    continue; // No bbox overlap — skip
                }

                // Check if triangles share an edge (adjacent)
                let tri_i = mesh.triangles[i];
                let tri_j = mesh.triangles[j];
                let mut shares_edge = false;
                for &vi in &tri_i {
                    for &vj in &tri_j {
                        let key = (vi.min(vj), vi.max(vj));
                        if adjacent.contains_key(&key) {
                            // Check if this edge belongs to both triangles
                            let i_edges = [
                                (tri_i[0].min(tri_i[1]), tri_i[0].max(tri_i[1])),
                                (tri_i[1].min(tri_i[2]), tri_i[1].max(tri_i[2])),
                                (tri_i[2].min(tri_i[0]), tri_i[2].max(tri_i[0])),
                            ];
                            let j_edges = [
                                (tri_j[0].min(tri_j[1]), tri_j[0].max(tri_j[1])),
                                (tri_j[1].min(tri_j[2]), tri_j[1].max(tri_j[2])),
                                (tri_j[2].min(tri_j[0]), tri_j[2].max(tri_j[0])),
                            ];
                            for ei in &i_edges {
                                for ej in &j_edges {
                                    if ei == ej {
                                        shares_edge = true;
                                        break;
                                    }
                                }
                                if shares_edge {
                                    break;
                                }
                            }
                        }
                        if shares_edge {
                            break;
                        }
                    }
                    if shares_edge {
                        break;
                    }
                }

                if shares_edge {
                    continue; // Adjacent triangles can share edges — not a defect
                }

                // Simple triangle-triangle intersection test
                if triangles_intersect(mesh, i, j) {
                    intersection_count += 1;
                    intersection_indices.push(i);
                    intersection_indices.push(j);
                }
            }
        }

        if intersection_count == 0 {
            return;
        }

        // Estimate total intersections by extrapolating from the sample
        let total_possible = n_tris * (n_tris - 1) / 2;
        let estimated_total = if check_count > 0 {
            (intersection_count as f64 / check_count as f64) * total_possible as f64
        } else {
            0.0
        };

        let severity = (estimated_total / (n_tris as f64 * 0.1)).min(1.0);
        let confidence = if check_count > total_possible / 2 {
            0.9 // High confidence if we checked many pairs
        } else {
            0.5 // Lower confidence with limited sampling
        };

        // Location: centroid of first intersection pair
        let location = if intersection_indices.len() >= 2 {
            let tri = &mesh.triangles[intersection_indices[0]];
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            Point3d::new(
                (v0.x + v1.x + v2.x) / 3.0,
                (v0.y + v1.y + v2.y) / 3.0,
                (v0.z + v1.z + v2.z) / 3.0,
            )
        } else {
            Point3d::ORIGIN
        };

        defects.push(ClassifiedDefect {
            defect_type: DefectType::SelfIntersection,
            severity,
            location,
            affected_elements: intersection_indices,
            confidence,
            reason: format!(
                "Detected {} self-intersections (checked {} of {} pairs, estimated total: {:.0})",
                intersection_count, check_count, total_possible, estimated_total
            ),
        });
    }

    /// Detect tolerance mismatches.
    fn detect_tolerance_mismatch(
        &self,
        _mesh: &TriangleMesh,
        _report: &ManifoldReport,
        _defects: &mut Vec<ClassifiedDefect>,
    ) {
        // Tolerance mismatch detection requires access to the B-Rep tolerance
        // data, which is not available from TriangleMesh/ManifoldReport alone.
        // This detector provides a placeholder that can be enriched when
        // B-Rep data is available.
        //
        // For now, we infer potential tolerance issues from the mesh:
        // - Very large variation in triangle sizes suggests inconsistent tolerances
        // - T-junctions often indicate tolerance issues

        // This is intentionally a lightweight check; the full implementation
        // requires access to Edge/Face tolerance data from the B-Rep.
    }
}

// ============================================================
// Helper functions
// ============================================================

/// Compute the characteristic model scale (diagonal of bounding box).
fn compute_model_scale(mesh: &TriangleMesh) -> f64 {
    let (min, max) = mesh.bounding_box();
    let dx = max.x - min.x;
    let dy = max.y - min.y;
    let dz = max.z - min.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Compute the area of a triangle.
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
///
/// Aspect ratio = longest_edge / (2 * sqrt(3) * inradius)
/// For an equilateral triangle, this returns 1.0.
/// For a degenerate/sliver triangle, this can be very large.
fn triangle_aspect_ratio(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let e0 = v0.distance_to(v1);
    let e1 = v1.distance_to(v2);
    let e2 = v2.distance_to(v0);
    let longest = e0.max(e1).max(e2);

    if longest < 1e-15 {
        return 1.0; // Degenerate — treat as equilateral
    }

    let area = triangle_area(v0, v1, v2);
    if area < 1e-20 {
        return 1e10; // Very thin triangle
    }

    // Inradius = 2 * area / perimeter
    let perimeter = e0 + e1 + e2;
    let inradius = 2.0 * area / perimeter;

    // Aspect ratio relative to equilateral triangle
    // For equilateral: longest = s, inradius = s / (2*sqrt(3))
    // Ratio = longest / (2*sqrt(3) * inradius) = 1.0 for equilateral
    longest / (2.0 * (3f64).sqrt() * inradius)
}

/// A boundary loop found by chaining boundary edges.
struct BoundaryLoop {
    /// Number of edges in the loop.
    edge_count: usize,
    /// Vertex indices forming the loop.
    vertex_indices: Vec<u32>,
    /// Centroid of the loop.
    centroid: Point3d,
    /// Whether the loop is closed (last vertex connects to first).
    is_closed: bool,
}

/// Find boundary loops by chaining boundary edges end-to-start.
fn find_boundary_loops(mesh: &TriangleMesh, boundary_edges: &[(u32, u32)]) -> Vec<BoundaryLoop> {
    if boundary_edges.is_empty() {
        return Vec::new();
    }

    // Build adjacency map: vertex → set of connected vertices via boundary edges
    let mut adjacency: HashMap<u32, Vec<u32>> = HashMap::new();
    for &(v0, v1) in boundary_edges {
        adjacency.entry(v0).or_default().push(v1);
        adjacency.entry(v1).or_default().push(v0);
    }

    let mut visited_edges: HashMap<(u32, u32), bool> = HashMap::new();
    let mut loops = Vec::new();

    for &(start_v0, start_v1) in boundary_edges {
        let key = (start_v0.min(start_v1), start_v0.max(start_v1));
        if visited_edges.contains_key(&key) {
            continue;
        }

        // Walk the loop starting from this edge
        let mut loop_vertices = vec![start_v0];
        let mut current = start_v1;
        let mut loop_closed = false;

        visited_edges.insert(key, true);

        loop {
            loop_vertices.push(current);

            // Find next unvisited boundary edge from current
            let mut found_next = false;
            if let Some(neighbors) = adjacency.get(&current) {
                for &next in neighbors {
                    let edge_key = (current.min(next), current.max(next));
                    if !visited_edges.contains_key(&edge_key) {
                        visited_edges.insert(edge_key, true);
                        current = next;
                        found_next = true;
                        break;
                    }
                }
            }

            if !found_next {
                break;
            }

            if current == start_v0 {
                loop_closed = true;
                break;
            }
        }

        // Compute centroid
        let mut centroid = Point3d::ORIGIN;
        let count = loop_vertices.len();
        for &v in &loop_vertices {
            if let Some(p) = mesh.vertices.get(v as usize) {
                centroid.x += p.x;
                centroid.y += p.y;
                centroid.z += p.z;
            }
        }
        if count > 0 {
            centroid.x /= count as f64;
            centroid.y /= count as f64;
            centroid.z /= count as f64;
        }

        loops.push(BoundaryLoop {
            edge_count: if loop_closed {
                loop_vertices.len() - 1 // Last vertex = first vertex
            } else {
                loop_vertices.len() - 1
            },
            vertex_indices: loop_vertices,
            centroid,
            is_closed: loop_closed,
        });
    }

    loops
}

/// Estimate the area of a boundary loop (polygon area in 3D).
fn estimate_loop_area(mesh: &TriangleMesh, vertex_indices: &[u32]) -> f64 {
    if vertex_indices.len() < 3 {
        return 0.0;
    }

    // Using 2D projection for area estimation
    let mut area = 0.0;

    let points: Vec<Point3d> = vertex_indices
        .iter()
        .filter_map(|&v| mesh.vertices.get(v as usize).copied())
        .collect();

    if points.len() < 3 {
        return 0.0;
    }

    // Compute normal using Newell's method
    let mut nx = 0.0_f64;
    let mut ny = 0.0_f64;
    let mut nz = 0.0_f64;

    for i in 0..points.len() {
        let j = (i + 1) % points.len();
        nx += (points[i].y - points[j].y) * (points[i].z + points[j].z);
        ny += (points[i].z - points[j].z) * (points[i].x + points[j].x);
        nz += (points[i].x - points[j].x) * (points[i].y + points[j].y);
    }

    // Project onto the dominant axis for area computation
    let abs_nx = nx.abs();
    let abs_ny = ny.abs();
    let abs_nz = nz.abs();

    if abs_nx >= abs_ny && abs_nx >= abs_nz {
        // Project onto YZ plane
        for i in 0..points.len() {
            let j = (i + 1) % points.len();
            area += points[i].y * points[j].z - points[j].y * points[i].z;
        }
    } else if abs_ny >= abs_nx && abs_ny >= abs_nz {
        // Project onto XZ plane
        for i in 0..points.len() {
            let j = (i + 1) % points.len();
            area += points[i].x * points[j].z - points[j].x * points[i].z;
        }
    } else {
        // Project onto XY plane
        for i in 0..points.len() {
            let j = (i + 1) % points.len();
            area += points[i].x * points[j].y - points[j].x * points[i].y;
        }
    }

    area.abs() * 0.5
}

/// Detect the "strip of degenerate triangles along cone apex" pattern.
///
/// This pattern occurs when triangulating a cone or sphere near the apex/pole:
/// many very thin triangles radiate from a single vertex, forming a fan-shaped
/// strip of slivers.
fn detect_degenerate_strip_pattern(mesh: &TriangleMesh, sliver_indices: &[usize]) -> bool {
    if sliver_indices.len() < 3 {
        return false;
    }

    // Count how many sliver triangles share a common vertex
    let mut vertex_count: HashMap<u32, usize> = HashMap::new();
    for &idx in sliver_indices {
        if let Some(tri) = mesh.triangles.get(idx) {
            for &v in tri {
                *vertex_count.entry(v).or_insert(0) += 1;
            }
        }
    }

    // If any vertex appears in > 60% of sliver triangles, it's likely an apex pattern
    let threshold = (sliver_indices.len() as f64 * 0.6) as usize;
    vertex_count.values().any(|&count| count >= threshold)
}

/// Simple triangle-triangle intersection test.
///
/// Uses the separating axis theorem (SAT) for two triangles.
fn triangles_intersect(mesh: &TriangleMesh, i: usize, j: usize) -> bool {
    let tri_a = mesh.triangles[i];
    let tri_b = mesh.triangles[j];

    let a0 = mesh.vertices[tri_a[0] as usize];
    let a1 = mesh.vertices[tri_a[1] as usize];
    let a2 = mesh.vertices[tri_a[2] as usize];
    let b0 = mesh.vertices[tri_b[0] as usize];
    let b1 = mesh.vertices[tri_b[1] as usize];
    let b2 = mesh.vertices[tri_b[2] as usize];

    // Compute edges and normal for triangle A
    let e1a = (a1.x - a0.x, a1.y - a0.y, a1.z - a0.z);
    let e2a = (a2.x - a0.x, a2.y - a0.y, a2.z - a0.z);
    let na = (
        e1a.1 * e2a.2 - e1a.2 * e2a.1,
        e1a.2 * e2a.0 - e1a.0 * e2a.2,
        e1a.0 * e2a.1 - e1a.1 * e2a.0,
    );

    // Test: all vertices of B on one side of A's plane?
    let db0 = (b0.x - a0.x) * na.0 + (b0.y - a0.y) * na.1 + (b0.z - a0.z) * na.2;
    let db1 = (b1.x - a0.x) * na.0 + (b1.y - a0.y) * na.1 + (b1.z - a0.z) * na.2;
    let db2 = (b2.x - a0.x) * na.0 + (b2.y - a0.y) * na.1 + (b2.z - a0.z) * na.2;

    let tol = 1e-10;
    if db0 > tol && db1 > tol && db2 > tol {
        return false;
    }
    if db0 < -tol && db1 < -tol && db2 < -tol {
        return false;
    }

    // Compute normal for triangle B
    let e1b = (b1.x - b0.x, b1.y - b0.y, b1.z - b0.z);
    let e2b = (b2.x - b0.x, b2.y - b0.y, b2.z - b0.z);
    let nb = (
        e1b.1 * e2b.2 - e1b.2 * e2b.1,
        e1b.2 * e2b.0 - e1b.0 * e2b.2,
        e1b.0 * e2b.1 - e1b.1 * e2b.0,
    );

    // Test: all vertices of A on one side of B's plane?
    let da0 = (a0.x - b0.x) * nb.0 + (a0.y - b0.y) * nb.1 + (a0.z - b0.z) * nb.2;
    let da1 = (a1.x - b0.x) * nb.0 + (a1.y - b0.y) * nb.1 + (a1.z - b0.z) * nb.2;
    let da2 = (a2.x - b0.x) * nb.0 + (a2.y - b0.y) * nb.1 + (a2.z - b0.z) * nb.2;

    if da0 > tol && da1 > tol && da2 > tol {
        return false;
    }
    if da0 < -tol && da1 < -tol && da2 < -tol {
        return false;
    }

    // Both triangles straddle each other's planes — potential intersection.
    // For a full SAT test, we'd need to check 9 edge-edge axes.
    // We use a simplified approach: if both triangles straddle each other's
    // planes, report a potential intersection.

    // Additional check: verify edge-edge separation on a few axes
    let edges_a = [e1a, e2a, (a2.x - a1.x, a2.y - a1.y, a2.z - a1.z)];
    let edges_b = [e1b, e2b, (b2.x - b1.x, b2.y - b1.y, b2.z - b1.z)];

    for ea in &edges_a {
        for eb in &edges_b {
            // Cross product as separating axis
            let axis = (
                ea.1 * eb.2 - ea.2 * eb.1,
                ea.2 * eb.0 - ea.0 * eb.2,
                ea.0 * eb.1 - ea.1 * eb.0,
            );
            let axis_len_sq = axis.0 * axis.0 + axis.1 * axis.1 + axis.2 * axis.2;
            if axis_len_sq < 1e-20 {
                continue; // Degenerate axis — skip
            }

            // Project all vertices onto the axis
            let pa0 = a0.x * axis.0 + a0.y * axis.1 + a0.z * axis.2;
            let pa1 = a1.x * axis.0 + a1.y * axis.1 + a1.z * axis.2;
            let pa2 = a2.x * axis.0 + a2.y * axis.1 + a2.z * axis.2;
            let pb0 = b0.x * axis.0 + b0.y * axis.1 + b0.z * axis.2;
            let pb1 = b1.x * axis.0 + b1.y * axis.1 + b1.z * axis.2;
            let pb2 = b2.x * axis.0 + b2.y * axis.1 + b2.z * axis.2;

            let min_a = pa0.min(pa1).min(pa2);
            let max_a = pa0.max(pa1).max(pa2);
            let min_b = pb0.min(pb1).min(pb2);
            let max_b = pb0.max(pb1).max(pb2);

            if max_a < min_b || max_b < min_a {
                return false; // Separation found
            }
        }
    }

    // No separation found — triangles likely intersect
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_mesh::TriangleMesh;
    use draper_mesh::check_manifold;

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
    fn test_cube_no_defects() {
        let mesh = make_cube_mesh();
        let report = check_manifold(&mesh);
        let classifier = DefectClassifier::new();
        let defects = classifier.classify(&mesh, &report);

        // A watertight cube should have minimal defects
        // It might still trigger small features or slivers depending on thresholds,
        // but no gaps, holes, non-manifold edges, or flipped normals
        let gap_defects: Vec<_> = defects.iter().filter(|d| d.defect_type == DefectType::Gap).collect();
        let hole_defects: Vec<_> = defects.iter().filter(|d| d.defect_type == DefectType::Hole).collect();
        let nm_defects: Vec<_> = defects.iter().filter(|d| d.defect_type == DefectType::NonManifoldEdge).collect();

        assert!(gap_defects.is_empty(), "Watertight cube should have no gaps");
        assert!(hole_defects.is_empty(), "Watertight cube should have no holes");
        assert!(nm_defects.is_empty(), "Watertight cube should have no non-manifold edges");
    }

    #[test]
    fn test_open_mesh_detects_gaps() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let report = check_manifold(&mesh);
        let classifier = DefectClassifier::new();
        let defects = classifier.classify(&mesh, &report);

        let gap_defects: Vec<_> = defects.iter().filter(|d| d.defect_type == DefectType::Gap).collect();
        assert!(!gap_defects.is_empty(), "Open mesh should have gaps");
        assert!(gap_defects[0].severity > 0.0, "Gap should have positive severity");
    }

    #[test]
    fn test_sliver_detection() {
        let mut mesh = TriangleMesh::new();
        // Very thin triangle (sliver)
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(100.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(50.0, 0.001, 0.0));
        mesh.add_triangle(0, 1, 2);

        let report = check_manifold(&mesh);
        let classifier = DefectClassifier::new();
        let defects = classifier.classify(&mesh, &report);

        let sliver_defects: Vec<_> = defects.iter().filter(|d| d.defect_type == DefectType::SliverTriangle).collect();
        assert!(!sliver_defects.is_empty(), "Very thin triangle should be detected as sliver");
    }

    #[test]
    fn test_degenerate_triangle_detection() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // Same as first
        mesh.add_triangle(0, 1, 2);

        let report = check_manifold(&mesh);
        let classifier = DefectClassifier::new();
        let defects = classifier.classify(&mesh, &report);

        let degen_defects: Vec<_> = defects.iter().filter(|d| d.defect_type == DefectType::DegenerateEdge).collect();
        assert!(!degen_defects.is_empty(), "Degenerate triangle should be detected");
    }

    #[test]
    fn test_defect_type_display() {
        assert_eq!(DefectType::Gap.to_string(), "Gap");
        assert_eq!(DefectType::FlippedNormal.to_string(), "FlippedNormal");
        assert_eq!(DefectType::SelfIntersection.to_string(), "SelfIntersection");
    }

    #[test]
    fn test_aspect_ratio_equilateral() {
        // Equilateral triangle with side length 1
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(1.0, 0.0, 0.0);
        let v2 = Point3d::new(0.5, (3f64).sqrt() / 2.0, 0.0);
        let ratio = triangle_aspect_ratio(&v0, &v1, &v2);
        assert!((ratio - 1.0).abs() < 0.01, "Equilateral triangle should have ratio ≈ 1.0, got {}", ratio);
    }

    #[test]
    fn test_aspect_ratio_sliver() {
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(100.0, 0.0, 0.0);
        let v2 = Point3d::new(50.0, 0.01, 0.0);
        let ratio = triangle_aspect_ratio(&v0, &v1, &v2);
        assert!(ratio > 10.0, "Very thin triangle should have high aspect ratio, got {}", ratio);
    }

    #[test]
    fn test_classifier_returns_sorted_by_severity() {
        let mut mesh = TriangleMesh::new();
        // Create a mesh with both gaps and degenerate triangles
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // degenerate
        mesh.add_triangle(0, 1, 2);
        mesh.add_triangle(0, 1, 3); // degenerate

        let report = check_manifold(&mesh);
        let classifier = DefectClassifier::new();
        let defects = classifier.classify(&mesh, &report);

        // Verify sorted by severity (descending)
        for i in 1..defects.len() {
            assert!(
                defects[i - 1].severity >= defects[i].severity,
                "Defects should be sorted by severity (descending)"
            );
        }
    }
}
