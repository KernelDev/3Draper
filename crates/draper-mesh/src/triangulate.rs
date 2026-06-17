// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Face triangulation — converts B-Rep faces to triangle meshes.
//!
//! Design principles:
//! 1. Edge curves are sampled at consistent parameter values so shared edges
//!    between adjacent faces produce identical 3D points (triangulation consistency).
//! 2. Planes use minimum number of triangles (ear-clipping, no interior subdivision).
//! 3. Curved surfaces use edge samples as boundary ring vertices.
//! 4. Watertight by construction — shared edges produce bit-identical vertices via the unified edge cache.
//! 5. TriangulationGuard prevents runaway computation on pathological faces.

use crate::mesh::TriangleMesh;
use crate::edge_cache::EdgeDiscretizationCache;
use draper_geometry::{
    Point3d, Point2d, Direction3d,
    Surface, Plane, CylinderSurface, SphereSurface, TorusSurface,
    ConeSurface, Curve3d,
};
use draper_topology::{Face, Wire, CoEdge, Edge, Solid, Shell, Compound, TopoId};
// WASM-compatible Instant: on native uses std::time::Instant,
// on wasm32 uses web_time::Instant (backed by performance.now()).
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

/// Guard that prevents individual face triangulation from running too long.
///
/// If a face takes longer than the configured time limit, its triangulation
/// is aborted and an empty mesh is returned. This prevents the entire
/// application from hanging on pathological geometry (e.g., 660 NURBS faces
/// in drill_top.stp).
///
/// # Usage
/// ```ignore
/// let guard = TriangulationGuard::new(std::time::Duration::from_secs(5));
/// if guard.should_abort() {
///     return TriangleMesh::new();
/// }
/// ```
#[derive(Clone, Debug)]
pub struct TriangulationGuard {
    /// Maximum time allowed for triangulation of a single face.
    time_limit: std::time::Duration,
    /// When the guard was created.
    start: Instant,
    /// Whether the guard has been triggered.
    aborted: bool,
}

impl TriangulationGuard {
    /// Create a new guard with a default time limit of 5 seconds per face.
    pub fn new() -> Self {
        Self::with_limit(std::time::Duration::from_secs(5))
    }

    /// Create a new guard with a custom time limit.
    pub fn with_limit(time_limit: std::time::Duration) -> Self {
        Self {
            time_limit,
            start: Instant::now(),
            aborted: false,
        }
    }

    /// Check if the time limit has been exceeded.
    /// Returns `true` if the triangulation should be aborted.
    pub fn should_abort(&mut self) -> bool {
        if self.aborted {
            return true;
        }
        if self.start.elapsed() > self.time_limit {
            self.aborted = true;
            true
        } else {
            false
        }
    }

    /// Check if the guard has been triggered (read-only).
    pub fn is_aborted(&self) -> bool {
        self.aborted
    }

    /// Reset the guard for a new face.
    pub fn reset(&mut self) {
        self.start = Instant::now();
        self.aborted = false;
    }

    /// Get the elapsed time since the guard was created or last reset.
    pub fn elapsed(&self) -> std::time::Duration {
        self.start.elapsed()
    }
}

impl Default for TriangulationGuard {
    fn default() -> Self {
        Self::new()
    }
}
use std::f64::consts::PI;
use std::collections::HashMap;

/// Triangulation parameters.
#[derive(Clone)]
pub struct TriangulationParams {
    /// Maximum edge length in the triangulation.
    pub max_edge_length: f64,
    /// Maximum deviation from the true surface.
    pub max_deviation: f64,
    /// Number of angular samples for cylindrical/spherical surfaces.
    pub angular_samples: usize,
    /// Number of height samples for cylindrical surfaces.
    pub height_samples: usize,
    /// Maximum angular deviation in radians between adjacent face normals.
    pub max_angular_deviation: f64,
    /// LOD detail level (1.0 = normal, 0.5 = coarser, 2.0 = finer).
    pub detail_level: f64,
    /// Whether to use adaptive sampling based on curvature.
    pub adaptive: bool,
    /// Whether to use parallel triangulation (multi-threaded via rayon).
    /// When `false` (default), uses the existing single-threaded path.
    /// When `true`, uses `rayon::par_iter()` to triangulate faces in parallel.
    pub parallel: bool,
    /// Optional progress callback invoked periodically during triangulation.
    /// Called with `(faces_completed, total_faces)`.
    /// Only used when `parallel` is `true`.
    pub progress_callback: Option<Arc<dyn Fn(usize, usize) + Send + Sync>>,
    /// Maximum number of triangles per face. If a face's grid would produce
    /// more triangles than this budget, the resolution is reduced.
    /// This prevents a single face from generating millions of triangles
    /// that freeze the browser on WASM. Default: 2000.
    pub max_face_triangles: usize,
}

impl std::fmt::Debug for TriangulationParams {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TriangulationParams")
            .field("max_edge_length", &self.max_edge_length)
            .field("max_deviation", &self.max_deviation)
            .field("angular_samples", &self.angular_samples)
            .field("height_samples", &self.height_samples)
            .field("max_angular_deviation", &self.max_angular_deviation)
            .field("detail_level", &self.detail_level)
            .field("adaptive", &self.adaptive)
            .field("parallel", &self.parallel)
            .field("max_face_triangles", &self.max_face_triangles)
            .field("progress_callback", &self.progress_callback.as_ref().map(|_| "Some(...)"))
            .finish()
    }
}

impl Default for TriangulationParams {
    fn default() -> Self {
        Self {
            max_edge_length: 1.0,
            max_deviation: 0.01,
            angular_samples: 32,
            height_samples: 8,
            max_angular_deviation: 0.1,
            detail_level: 1.0,
            adaptive: true,
            parallel: false,
            progress_callback: None,
            max_face_triangles: 8000,
        }
    }
}

impl TriangulationParams {
    /// Create parameters for a specific Level of Detail (LOD).
    ///
    /// LOD is a value in [0.0, 1.0] where:
    /// - `0.0` = coarsest (minimum triangles, for distant/preview)
    /// - `0.5` = medium (balanced quality/performance)
    /// - `1.0` = full quality (default parameters)
    ///
    /// The LOD value scales `max_deviation`, `angular_samples`,
    /// `height_samples`, and `max_face_triangles` accordingly.
    /// Lower LOD means fewer triangles (coarser mesh) and larger
    /// allowed deviation from the true surface.
    ///
    /// # Example
    /// ```ignore
    /// // Preview quality (very coarse, fast)
    /// let preview = TriangulationParams::for_lod(0.1);
    /// // Medium quality (interactive)
    /// let medium = TriangulationParams::for_lod(0.5);
    /// // Full quality
    /// let full = TriangulationParams::for_lod(1.0);
    /// ```
    pub fn for_lod(lod: f64) -> Self {
        let lod = lod.clamp(0.0, 1.0);

        // Scale max_deviation: at LOD 0.0, allow 100× more deviation;
        // at LOD 1.0, use the default 0.01.
        // The relationship is exponential: deviation = 0.01 / lod^2
        // (capped at 1.0 for very low LOD).
        let max_deviation = if lod < 0.01 {
            1.0 // Very coarse — large deviation allowed
        } else {
            0.01 / (lod * lod).min(1.0)
        };

        // Scale angular samples: at LOD 0.0 → 6 (hexagonal), at 1.0 → 32
        let angular_samples = (6.0 + 26.0 * lod).round() as usize;

        // Scale height samples: at LOD 0.0 → 2 (minimum), at 1.0 → 8
        let height_samples = (2.0 + 6.0 * lod).round() as usize;

        // Scale max_face_triangles: at LOD 0.0 → 100, at 1.0 → 8000
        let max_face_triangles = (100.0 + 7900.0 * lod).round() as usize;

        // Scale detail_level: at LOD 0.0 → 0.25, at 1.0 → 1.0
        let detail_level = 0.25 + 0.75 * lod;

        Self {
            max_edge_length: 1.0 / lod.max(0.1), // Longer edges at lower LOD
            max_deviation,
            angular_samples,
            height_samples,
            max_angular_deviation: 0.1 / lod.max(0.1), // More angular deviation at lower LOD
            detail_level,
            adaptive: true,
            parallel: false,
            progress_callback: None,
            max_face_triangles,
        }
    }

    /// Create parameters optimized for preview (LOD ≈ 0.15).
    ///
    /// Very coarse mesh suitable for:
    /// - Thumbnail generation
    /// - Distant objects in a scene
    /// - First-pass progressive loading
    pub fn preview() -> Self {
        Self::for_lod(0.15)
    }

    /// Create parameters for interactive editing (LOD ≈ 0.5).
    ///
    /// Balanced quality/performance suitable for:
    /// - Real-time rotation/pan/zoom
    /// - Model inspection at medium distance
    pub fn interactive() -> Self {
        Self::for_lod(0.5)
    }

    /// Create parameters for high-quality rendering (LOD = 1.0).
    ///
    /// Full quality suitable for:
    /// - Close-up inspection
    /// - Export to manufacturing formats
    /// - Final rendering / screenshots
    pub fn high_quality() -> Self {
        Self::for_lod(1.0)
    }

    /// Derive a coarser set of params from this one by the given LOD factor.
    ///
    /// Returns a new `TriangulationParams` with the same settings except
    /// `detail_level` scaled by `factor`. This is useful for generating
    /// multiple LOD levels from a base configuration.
    pub fn with_detail_level(&self, detail_level: f64) -> Self {
        let mut params = self.clone();
        params.detail_level = detail_level;
        // Also scale angular/height samples proportionally
        let scale = (detail_level / self.detail_level).max(0.25);
        params.angular_samples = (self.angular_samples as f64 * scale).round() as usize;
        params.height_samples = (self.height_samples as f64 * scale).round() as usize;
        params.max_face_triangles = (self.max_face_triangles as f64 * scale).round() as usize;
        params
    }
}

/// Number of samples per edge curve for boundary discretization.
/// Reduced from 48 to 20 for performance — each boundary point on a NURBS
/// face requires expensive project_point() or reproject_nurbs_point() calls.
/// For watertightness, the edge cache ensures shared edges produce identical
/// 3D points regardless of sample count. 20 samples provides sufficient
/// boundary resolution for accurate triangulation while being ~2.4x faster
/// than 48 samples.
const EDGE_SAMPLES: usize = 20;

/// Named Level-of-Detail presets for triangulation.
///
/// These provide a convenient, typed way to select LOD levels without
/// remembering the exact float values. Each variant maps to a specific
/// LOD factor that controls triangle density and surface accuracy.
///
/// # Usage
/// ```ignore
/// let params = TriangulationParams::for_lod(LodLevel::Preview.lod_factor());
/// // or directly:
/// let params = LodLevel::Interactive.params();
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LodLevel {
    /// Coarsest mesh (LOD ≈ 0.1). Suitable for thumbnails, distant objects,
    /// and initial progressive loading. Produces ~6-10 angular samples
    /// per curved face and allows 1.0 max deviation.
    Preview,
    /// Low-detail mesh (LOD ≈ 0.3). Suitable for medium-distance viewing.
    /// Better shape than Preview but still fast to compute.
    Low,
    /// Medium-detail mesh (LOD ≈ 0.5). Balanced quality/performance for
    /// interactive rotation and inspection.
    Interactive,
    /// High-detail mesh (LOD ≈ 0.75). Close inspection with good surface
    /// accuracy. Suitable for most engineering visualization.
    High,
    /// Maximum quality (LOD = 1.0). Full resolution for manufacturing
    /// export, close-up rendering, and analysis.
    Ultra,
}

impl LodLevel {
    /// Return the LOD factor as a float in [0.0, 1.0].
    pub fn lod_factor(self) -> f64 {
        match self {
            LodLevel::Preview => 0.10,
            LodLevel::Low => 0.30,
            LodLevel::Interactive => 0.50,
            LodLevel::High => 0.75,
            LodLevel::Ultra => 1.00,
        }
    }

    /// Create triangulation parameters for this LOD level.
    pub fn params(self) -> TriangulationParams {
        TriangulationParams::for_lod(self.lod_factor())
    }

    /// Get the next coarser LOD level, or `None` if already at Preview.
    pub fn coarser(self) -> Option<LodLevel> {
        match self {
            LodLevel::Preview => None,
            LodLevel::Low => Some(LodLevel::Preview),
            LodLevel::Interactive => Some(LodLevel::Low),
            LodLevel::High => Some(LodLevel::Interactive),
            LodLevel::Ultra => Some(LodLevel::High),
        }
    }

    /// Get the next finer LOD level, or `None` if already at Ultra.
    pub fn finer(self) -> Option<LodLevel> {
        match self {
            LodLevel::Preview => Some(LodLevel::Low),
            LodLevel::Low => Some(LodLevel::Interactive),
            LodLevel::Interactive => Some(LodLevel::High),
            LodLevel::High => Some(LodLevel::Ultra),
            LodLevel::Ultra => None,
        }
    }
}

/// Project a sequence of 3D points onto a NURBS surface efficiently.
///
/// Uses Newton-Raphson with UV chaining: the first point uses the expensive
/// `project_point()` (146+ evaluations), and subsequent points use
/// Newton-Raphson from the previous point's UV as starting guess (~15
/// evaluations each). This is ~10x faster than calling `project_point()`
/// on every point, and also more accurate since the initial guess is close.
fn project_points_nurbs_fast(surface: &Surface, points: &[Point3d]) -> Vec<Point2d> {
    if points.is_empty() {
        return Vec::new();
    }
    if let Surface::Nurbs(ref nurbs) = surface {
        let (u_min, u_max) = nurbs.u_range();
        let (v_min, v_max) = nurbs.v_range();
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;
        let use_chain_newton = u_range < 10.0 && v_range < 10.0;

        let mut uvs = Vec::with_capacity(points.len());
        let mut prev_u = (u_min + u_max) * 0.5;
        let mut prev_v = (v_min + v_max) * 0.5;

        for (i, p) in points.iter().enumerate() {
            let (u, v) = if use_chain_newton && i > 0 {
                crate::parametric_domain::reproject_nurbs_point(nurbs, p, prev_u, prev_v)
            } else {
                surface.project_point(p)
            };

            let proj_p = surface.point_at(u, v);
            let err = p.distance_to(&proj_p);

            if err > 1e-4 {
                let grid_size = crate::edge_cache::adaptive_grid_size(u_range, v_range);
                let (ub, vb) = crate::edge_cache::brute_force_project_point(nurbs, p, grid_size);
                let bf_p = surface.point_at(ub, vb);
                let bf_err = p.distance_to(&bf_p);

                if bf_err < err {
                    uvs.push(Point2d::new(ub, vb));
                    prev_u = ub;
                    prev_v = vb;
                } else {
                    uvs.push(Point2d::new(u, v));
                    prev_u = u;
                    prev_v = v;
                }
            } else {
                uvs.push(Point2d::new(u, v));
                prev_u = u;
                prev_v = v;
            }
        }
        uvs
    } else {
        points.iter().map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        }).collect()
    }
}

// ============================================================
// Top-level entry points
// ============================================================

/// Triangulate a solid into a triangle mesh using topology-first approach.
///
/// The unified edge cache guarantees bit-identical 3D points on shared edges,
/// making the mesh watertight **by construction** — no post-hoc vertex merging
/// or edge stitching is needed.
///
/// Post-processing steps (topology-first):
/// 1. Filter degenerate (zero-area) triangles and NaN/Inf vertices
/// 2. Validate watertightness and attempt automatic repair if needed
/// 3. Smooth normals across shared edges with adaptive crease angle
///
/// When `params.parallel` is `true`, faces are triangulated in parallel using
/// rayon and per-face meshes are merged with pre-computed vertex offsets.
pub fn triangulate_solid(solid: &Solid, params: &TriangulationParams) -> TriangleMesh {
    // Compute adaptive tolerance from the solid's bounding box
    let bbox = solid_bounding_box(solid);
    let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(
        &bbox.0, &bbox.1, 64,
    );
    triangulate_solid_with_cache(solid, params, &mut cache)
}

/// Triangulate a solid using a shared edge discretization cache.
///
/// The cache ensures that shared edges between adjacent faces produce
/// identical 3D point sequences, which is critical for watertight meshes.
pub fn triangulate_solid_with_cache(solid: &Solid, params: &TriangulationParams, cache: &mut EdgeDiscretizationCache) -> TriangleMesh {
    if params.parallel {
        // For parallel: pre-populate the cache fully, then share as immutable
        cache.pre_populate_for_solid_full(solid, EDGE_SAMPLES);
        triangulate_solid_parallel(solid, params, cache)
    } else {
        triangulate_solid_sequential(solid, params, cache)
    }
}

/// Triangulate a solid in parallel using a pre-populated, Arc-wrapped edge cache.
///
/// This is the recommended API for callers that need fine-grained control over
/// the cache lifecycle (e.g., caching across multiple solids, WASM workers).
/// The cache must be fully pre-populated (all edges + UV coordinates) before
/// calling this function — use `EdgeDiscretizationCache::pre_populate_for_solid_full()`.
///
/// # Thread safety
///
/// The `Arc<EdgeDiscretizationCache>` is shared immutably across rayon worker
/// threads. Since the cache is fully pre-populated before triangulation starts,
/// no synchronization is needed — each worker reads its own face's boundary data.
///
/// # Performance
///
/// Uses a two-level tree-reduce merge strategy:
/// 1. Faces are triangulated in parallel via `rayon::par_iter()`
/// 2. Per-face meshes are merged in parallel pairs (tree-reduce), reducing
///    the merge from O(n) sequential to O(log n) parallel depth
/// 3. Final dedup and validation runs on the single merged result
///
/// For models with 100+ faces, this provides ~2-4x speedup over sequential
/// triangulation on 8-core machines, with the merge step being ~2x faster
/// than the previous sequential merge.
pub fn triangulate_solid_parallel_arc(
    solid: &Solid,
    params: &TriangulationParams,
    cache: Arc<EdgeDiscretizationCache>,
) -> TriangleMesh {
    let start = Instant::now();

    // Collect faces into a Vec for parallel iteration
    let faces: Vec<&Face> = solid.faces();
    let total_faces = faces.len();

    if total_faces == 0 {
        return TriangleMesh::new();
    }

    // Step 1: Parallel face triangulation
    let completed_count = AtomicUsize::new(0);
    let progress_cb = params.progress_callback.clone();

    use rayon::prelude::*;

    let face_meshes: Vec<TriangleMesh> = faces
        .par_iter()
        .map(|face| {
            let mesh = triangulate_face_impl(face, params, &cache);

            // Progress reporting (lock-free)
            if progress_cb.is_some() {
                let completed = completed_count.fetch_add(1, Ordering::Relaxed) + 1;
                if let Some(ref cb) = progress_cb {
                    cb(completed, total_faces);
                }
            }

            mesh
        })
        .collect();

    let triangulate_time = start.elapsed();

    // Step 2: Parallel tree-reduce merge with tolerance-based dedup
    let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
    let merged = merge_meshes_tree_reduce(&face_meshes, adaptive_tol);
    let merge_time = start.elapsed();

    // Step 3: Post-processing (topology-first — no mandatory merge/stitch)
    let mut mesh = merged;
    filter_degenerate_triangles(&mut mesh, 1e-10);

    // Step 4: Validation
    let report = crate::watertight::validate_watertight(&mesh, false);
    if !report.is_watertight() {
        let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
        let boundary_pct = if report.edge_count > 0 {
            report.boundary_edge_count as f64 / report.edge_count as f64 * 100.0
        } else {
            0.0
        };
        log::error!(
            "BUG: Solid triangulation not watertight (parallel-arc): {} boundary edges ({:.2}%), {} non-manifold, {} degenerate (V={}, E={}, F={}, χ={}, tol={:.2e})",
            report.boundary_edge_count,
            boundary_pct,
            report.non_manifold_edge_count,
            report.degenerate_triangle_count,
            report.vertex_count,
            report.edge_count,
            report.triangle_count,
            report.euler_characteristic,
            adaptive_tol,
        );
    } else {
        log::info!("Solid is watertight ✓ (parallel-arc, {} interior edges, {} triangles, χ={})",
            report.interior_edge_count, report.triangle_count, report.euler_characteristic);
    }

    // Step 5: Smooth normals with adaptive crease angle
    crate::watertight::smooth_normals_adaptive(&mut mesh, solid);

    debug_assert_mesh_consistency(&mesh);

    let total_time = start.elapsed();
    log::info!(
        "Parallel-arc triangulation timing: triangulate={:.1}ms, merge={:.1}ms, total={:.1}ms ({} faces)",
        triangulate_time.as_secs_f64() * 1000.0,
        (merge_time - triangulate_time).as_secs_f64() * 1000.0,
        total_time.as_secs_f64() * 1000.0,
        total_faces,
    );

    mesh
}

/// Sequential (single-threaded) solid triangulation with edge cache.
///
/// Uses topology-first approach: the edge cache guarantees bit-identical
/// 3D points on shared edges, so the mesh is watertight by construction.
fn triangulate_solid_sequential(solid: &Solid, params: &TriangulationParams, cache: &mut EdgeDiscretizationCache) -> TriangleMesh {
    // Phase 1: Pre-populate edge cache with deterministic rounding
    cache.pre_populate_for_solid(solid, EDGE_SAMPLES);

    // Phase 2: Triangulate all faces using cached boundary data.
    // Use merge_deduplicating to ensure shared-edge vertices get the same
    // vertex index in the final mesh.
    //
    // Dedup strategy: bit-exact + tolerance fallback.
    // The edge cache with deterministic rounding (48-bit mantissa) produces
    // bit-identical 3D coordinates for shared STEP EDGE_CURVEs. However,
    // some edges in STEP files don't share the same EDGE_CURVE entity
    // (different STEP IDs for the same geometric boundary). In those cases,
    // the edge cache produces near-identical but not bit-identical vertices.
    //
    // The tolerance fallback catches these near-misses: vertices within
    // the adaptive merge tolerance (model_scale × 1e-6) are merged. This is
    // essential for watertightness — without it, 11-21% boundary edges remain
    // unmerged even though the edge cache is working correctly for shared edges.
    //
    // The tolerance is very small (1 PPM of model scale), so it will never
    // merge genuinely distinct features.
    let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
    let mut mesh = TriangleMesh::new();
    let mut dedup_map = crate::mesh::VertexDedupMap::with_tolerance(adaptive_tol);
    let mut total_face_vertices = 0usize;
    for face in solid.faces() {
        let face_mesh = triangulate_face_with_cache(face, params, cache);
        total_face_vertices += face_mesh.vertices.len();
        mesh.merge_deduplicating(&face_mesh, &mut dedup_map);
    }
    let deduped_vertices = total_face_vertices - mesh.vertices.len();
    if deduped_vertices > 0 {
        let (exact_hits, tolerance_hits, misses) = dedup_map.stats();
        let total_lookups = exact_hits + tolerance_hits + misses;
        let exact_pct = if total_lookups > 0 { exact_hits as f64 / total_lookups as f64 * 100.0 } else { 0.0 };
        let tol_pct = if total_lookups > 0 { tolerance_hits as f64 / total_lookups as f64 * 100.0 } else { 0.0 };
        log::info!(
            "Vertex dedup: {} face vertices → {} unique ({} shared: {:.1}% bit-exact, {:.1}% tolerance, tol={:.2e})",
            total_face_vertices, mesh.vertices.len(), deduped_vertices,
            exact_pct, tol_pct, adaptive_tol,
        );
    }

    // Phase 3: Post-processing (topology-first — no merge/stitch needed)
    // Filter degenerate triangles (zero area or NaN/Inf)
    filter_degenerate_triangles(&mut mesh, 1e-10);

    // Phase 4: Validation — log but do NOT apply repair_mesh.
    // If the mesh is not watertight, that indicates a BUG in the edge cache
    // or the face triangulation. repair_mesh/stitch_boundary_edges mask the
    // real problem by moving vertices by up to 100× base_tol, which
    // degrades mesh quality and breaks normals. Instead, we log the
    // boundary edge count as an error so the root cause can be fixed.
    let report = crate::watertight::validate_watertight(&mesh, false);
    if !report.is_watertight() {
        let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
        let boundary_pct = if report.edge_count > 0 {
            report.boundary_edge_count as f64 / report.edge_count as f64 * 100.0
        } else {
            0.0
        };
        log::error!(
            "BUG: Solid triangulation not watertight: {} boundary edges ({:.2}%), {} non-manifold, {} degenerate (V={}, E={}, F={}, χ={}, tol={:.2e})",
            report.boundary_edge_count,
            boundary_pct,
            report.non_manifold_edge_count,
            report.degenerate_triangle_count,
            report.vertex_count,
            report.edge_count,
            report.triangle_count,
            report.euler_characteristic,
            adaptive_tol,
        );
        if boundary_pct > 1.0 {
            log::error!("More than 1% boundary edges — edge cache is NOT working correctly for this solid!");
        }
        // Run edge consistency validation to diagnose the root cause
        let consistency = crate::watertight::validate_edge_consistency(&mesh, adaptive_tol);
        log::error!(
            "Edge consistency: {}/{} shared edges consistent, {} inconsistent ({:.2}%), max_dist={:.2e}",
            consistency.consistent_edges,
            consistency.shared_edges_checked,
            consistency.inconsistent_edges,
            consistency.inconsistency_rate(),
            consistency.max_vertex_distance,
        );
        for inc in &consistency.worst_inconsistencies {
            log::error!(
                "  Inconsistent edge: vertices ({}, {}), distance={:.2e}, faces={:?}",
                inc.vertex_indices.0, inc.vertex_indices.1, inc.distance, inc.face_ids,
            );
        }
        // Log edge cache statistics for debugging
        let stats = cache.stats();
        log::info!("Edge cache stats: {} entries, {} hits, {} misses, {} shared edges, hit_rate={:.1}%",
            stats.total_edges, stats.cache_hits, stats.cache_misses, stats.shared_edges,
            if stats.cache_hits + stats.cache_misses > 0 {
                stats.cache_hits as f64 / (stats.cache_hits + stats.cache_misses) as f64 * 100.0
            } else { 0.0 },
        );
    } else {
        log::info!("Solid is watertight ✓ ({} interior edges, {} triangles, χ={})",
            report.interior_edge_count, report.triangle_count, report.euler_characteristic);
    }

    // Phase 5: Smooth normals with adaptive crease angle per surface type
    crate::watertight::smooth_normals_adaptive(&mut mesh, solid);

    // Chord error refinement is handled internally by
    // refine_mesh_chord_error_uv() in triangulate_surface_consistent(),
    // so no post-hoc refinement is needed here.

    debug_assert_mesh_consistency(&mesh);
    mesh
}

// ============================================================
// Parallel triangulation (3.5)
// ============================================================

// Note: The old EdgeSampleCache and related functions (collect_face_boundary_points_cached,
// collect_face_hole_points_cached, triangulate_face_cached) have been removed.
// They were superseded by EdgeDiscretizationCache (in edge_cache.rs), which stores
// both 3D points AND per-face UV coordinates. The main pipeline uses
// EdgeDiscretizationCache with pre_populate_for_solid_full().

/// Parallel solid triangulation using rayon (topology-first approach).
///
/// # Algorithm
/// 1. **Pre-compute** edge discretizations into a read-only `EdgeDiscretizationCache`
///    with deterministic rounding (done before calling this function).
/// 2. **Parallel triangulate** each face independently using `rayon::par_iter()`.
/// 3. **Merge** per-face meshes with pre-computed vertex offsets (avoids
///    sequential `merge` calls).
/// 4. **Post-process**: filter degenerate triangles, validate watertightness,
///    adaptive repair if needed, smooth normals with adaptive crease angle.
///
/// # Thread safety
/// - The `EdgeDiscretizationCache` is shared immutably across threads (no locking needed).
/// - Each face produces its own `TriangleMesh` — no shared mutable state.
/// - The progress callback uses `AtomicUsize` for lock-free counting.
fn triangulate_solid_parallel(solid: &Solid, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    // Wrap in Arc and delegate to the Arc-based implementation
    let arc_cache = Arc::new(cache.clone());
    triangulate_solid_parallel_arc(solid, params, arc_cache)
}

/// Merge multiple per-face meshes using a two-level tree-reduce strategy.
///
/// Instead of merging N meshes sequentially (O(n) depth), we merge in pairs
/// using rayon's parallel reduce: at each level, adjacent mesh pairs are merged
/// in parallel, reducing the total depth from O(n) to O(log n). This is
/// significantly faster for models with 50+ faces.
///
/// The tree-reduce approach also improves cache locality: each merge step
/// operates on smaller meshes, keeping the dedup map in L1/L2 cache rather
/// than thrashing L3 for a single massive dedup map.
///
/// # Algorithm
/// 1. Split meshes into chunks (one per rayon worker)
/// 2. Each worker sequentially merges its chunk with a local dedup map
/// 3. The partially-merged results are then merged sequentially (small N)
///
/// # Why not full parallel reduce?
///
/// True parallel reduce would merge pairs at each level, but the merge
/// operation is not commutative in the presence of dedup (vertex indices
/// depend on insertion order). Instead, we use a chunked approach that
/// preserves deterministic ordering while parallelizing the bulk of the work.
fn merge_meshes_tree_reduce(meshes: &[TriangleMesh], tolerance: f64) -> TriangleMesh {
    use rayon::prelude::*;

    if meshes.is_empty() {
        return TriangleMesh::new();
    }
    if meshes.len() == 1 {
        return meshes[0].clone();
    }

    // For small mesh counts, sequential merge is faster (avoids rayon overhead)
    if meshes.len() < 8 {
        return merge_meshes_sequential(meshes, tolerance);
    }

    // Chunked parallel merge: split into rayon-sized chunks, merge each
    // chunk sequentially with its own dedup map, then merge the results.
    let n_chunks = rayon::current_num_threads().max(1).min(meshes.len());
    let chunk_size = (meshes.len() + n_chunks - 1) / n_chunks;

    let chunks: Vec<TriangleMesh> = meshes
        .par_chunks(chunk_size)
        .map(|chunk| merge_meshes_sequential(chunk, tolerance))
        .collect();

    // Final merge of chunk results (small N, sequential is fine)
    let merged = merge_meshes_sequential(&chunks, tolerance);

    let total_face_vertices: usize = meshes.iter().map(|m| m.vertices.len()).sum();
    let deduped_vertices = total_face_vertices - merged.vertices.len();
    if deduped_vertices > 0 {
        log::info!(
            "Parallel tree-reduce dedup: {} face vertices → {} unique ({} shared, {} chunks, tol={:.2e})",
            total_face_vertices, merged.vertices.len(), deduped_vertices, chunks.len(), tolerance,
        );
    }
    merged
}

/// Sequential merge of meshes with tolerance-based dedup (worker function).
///
/// This is the inner loop of `merge_meshes_tree_reduce` — each rayon worker
/// calls this on its assigned chunk. Uses bit-exact + tolerance dedup to catch
/// both shared STEP EDGE_CURVE vertices and near-identical vertices from
/// different STEP entities on the same geometric boundary.
fn merge_meshes_sequential(meshes: &[TriangleMesh], tolerance: f64) -> TriangleMesh {
    if meshes.is_empty() {
        return TriangleMesh::new();
    }

    let mut merged = TriangleMesh::new();
    let mut dedup_map = crate::mesh::VertexDedupMap::with_tolerance(tolerance);
    for mesh in meshes {
        merged.merge_deduplicating(mesh, &mut dedup_map);
    }
    merged
}

/// Compute the bounding box of a Solid from its face surfaces and edge vertices.
fn solid_bounding_box(solid: &Solid) -> (Point3d, Point3d) {
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    let mut has_points = false;

    for face in solid.faces() {
        for edge in &face.edges {
            if edge.degenerate { continue; }
            if let Some(p) = edge.start_point() {
                min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
                has_points = true;
            }
            if let Some(p) = edge.end_point() {
                min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
                has_points = true;
            }
        }
    }

    if !has_points {
        return (Point3d::ORIGIN, Point3d::new(1.0, 1.0, 1.0));
    }
    (min, max)
}

/// Compute the bounding box of a Shell from its face surfaces and edge vertices.
fn shell_bounding_box(shell: &Shell) -> (Point3d, Point3d) {
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    let mut has_points = false;

    for face in &shell.faces {
        for edge in &face.edges {
            if edge.degenerate { continue; }
            if let Some(p) = edge.start_point() {
                min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
                has_points = true;
            }
            if let Some(p) = edge.end_point() {
                min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
                has_points = true;
            }
        }
    }

    if !has_points {
        return (Point3d::ORIGIN, Point3d::new(1.0, 1.0, 1.0));
    }
    (min, max)
}

/// Triangulate a shell into a triangle mesh (topology-first approach).
pub fn triangulate_shell(shell: &Shell, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    // Compute adaptive tolerance from shell bounding box
    let (bmin, bmax) = shell_bounding_box(shell);
    let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(&bmin, &bmax, EDGE_SAMPLES);
    let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
    // Tolerance-based dedup: same strategy as sequential solid path.
    // Bit-exact-only dedup misses near-identical vertices from different
    // STEP EDGE_CURVEs that are geometrically the same boundary. The adaptive
    // tolerance (1 PPM of model scale) catches these near-misses without
    // collapsing genuinely distinct features.
    let mut dedup_map = crate::mesh::VertexDedupMap::with_tolerance(adaptive_tol);
    for face in &shell.faces {
        let face_mesh = triangulate_face_with_cache(face, params, &mut cache);
        mesh.merge_deduplicating(&face_mesh, &mut dedup_map);
    }
    filter_degenerate_triangles(&mut mesh, 1e-10);
    mesh
}

/// Triangulate a compound into a triangle mesh.
pub fn triangulate_compound(compound: &Compound, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for solid in &compound.solids {
        mesh.merge(&triangulate_solid(solid, params));
    }
    for sub in &compound.compounds {
        mesh.merge(&triangulate_compound(sub, params));
    }
    mesh
}

/// Triangulate a single face.
///
/// This is the backward-compatible public API that creates a local cache.
/// For better watertightness when triangulating multiple faces of a solid,
/// use `triangulate_face_with_cache` with a shared cache instead.
pub fn triangulate_face(face: &Face, params: &TriangulationParams) -> TriangleMesh {
    let mut cache = EdgeDiscretizationCache::new();
    triangulate_face_with_cache(face, params, &mut cache)
}

/// Triangulate a single face using a shared edge discretization cache.
///
/// The cache ensures that shared edges between adjacent faces produce
/// identical 3D point sequences, which is critical for watertight meshes.
/// This function pre-populates the cache for the face's edges, then
/// delegates to `triangulate_face_impl` which uses the immutable cache.
pub fn triangulate_face_with_cache(face: &Face, params: &TriangulationParams, cache: &mut EdgeDiscretizationCache) -> TriangleMesh {
    // Pre-populate edges for this face so the cache is complete
    // before we pass it as an immutable reference
    if let Some(ref surface) = face.surface {
        pre_populate_face_edges(cache, face, surface);
    }
    triangulate_face_impl(face, params, cache)
}

/// Internal implementation: triangulate a face with a read-only cache.
///
/// This is used both by `triangulate_face_with_cache` (sequential path)
/// and by the parallel path (after the cache has been fully pre-populated).
///
/// # Fallback strategy (Phase 2.2)
///
/// When the primary surface-specific triangulation produces an empty mesh
/// (e.g., NURBS with degenerate UVs, surface with no boundary edges), or
/// when `face.surface` is `None`, we apply a three-tier fallback strategy:
///
/// 1. **ApproximatePlane** — fit a plane to boundary 3D points and ear-clip
/// 2. **BoundaryFan** — fan-triangulate from centroid using boundary points only
/// 3. **SurfacePointSample** — sample surface.point_at() on a regular UV grid
///
/// Each fallback logs a warning so that pathological geometry can be diagnosed.
/// The fallback meshes are approximate but always produce a visible result
/// rather than silently dropping the face.
fn triangulate_face_impl(face: &Face, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let primary_mesh = if let Some(ref surface) = face.surface {
        match surface {
            Surface::Plane(plane) => triangulate_planar_face(face, plane, params, cache),
            Surface::Cylinder(cyl) => triangulate_cylinder_face(face, cyl, params, cache),
            Surface::Sphere(sphere) => triangulate_sphere_face(face, sphere, params, cache),
            Surface::Torus(torus) => triangulate_torus_face(face, torus, params, cache),
            Surface::Cone(cone) => triangulate_cone_face(face, cone, params, cache),
            Surface::Revolution(rev) => triangulate_revolution_face(face, rev, params, cache),
            Surface::Extrusion(ext) => triangulate_extrusion_face(face, ext, params, cache),
            Surface::Nurbs(_) => {
                triangulate_nurbs_cdt(face, surface, params, cache)
            }
        }
    } else {
        TriangleMesh::new()
    };

    // Phase 2.2: If the primary triangulation produced an empty mesh,
    // apply fallback strategies in order of decreasing quality.
    if !primary_mesh.vertices.is_empty() {
        return primary_mesh;
    }

    // All fallback strategies need boundary 3D points from the cache.
    let boundary_3d = if let Some(ref surface) = face.surface {
        collect_face_boundary_from_cache(face, cache, surface)
    } else {
        collect_face_boundary_no_surface(face, cache)
    };

    if boundary_3d.len() < 3 {
        log::debug!(
            "FallbackSurface: face {} has {} boundary points (< 3), cannot fallback",
            face.id, boundary_3d.len()
        );
        return TriangleMesh::new();
    }

    log::warn!(
        "FallbackSurface: face {} primary triangulation empty (surface={}), trying fallback strategies ({} boundary pts)",
        face.id,
        face.surface.as_ref().map_or("None", |s| s.type_name()),
        boundary_3d.len()
    );

    // Fallback tier 1: Approximate plane — best quality for near-planar faces
    if let Some(mesh) = fallback_approximate_plane(face, &boundary_3d, cache) {
        log::info!("FallbackSurface: face {} → ApproximatePlane ({} triangles)", face.id, mesh.triangles.len());
        return mesh;
    }

    // Fallback tier 2: Boundary fan — works for any face shape but may
    // produce degenerate triangles on highly concave boundaries
    if let Some(mesh) = fallback_boundary_fan(face, &boundary_3d) {
        log::info!("FallbackSurface: face {} → BoundaryFan ({} triangles)", face.id, mesh.triangles.len());
        return mesh;
    }

    // Fallback tier 3: Surface point sampling — only works when surface exists
    if let Some(ref surface) = face.surface {
        if let Some(mesh) = fallback_surface_point_sample(face, surface, &boundary_3d, params) {
            log::info!("FallbackSurface: face {} → SurfacePointSample ({} triangles)", face.id, mesh.triangles.len());
            return mesh;
        }
    }

    log::warn!(
        "FallbackSurface: face {} all fallback strategies failed — empty mesh",
        face.id
    );
    TriangleMesh::new()
}

// ============================================================
// Edge curve sampling — the foundation of consistent triangulation
// ============================================================

/// Sample points along a single edge curve.
/// Returns the sampled 3D points (not including the endpoint to avoid duplicates
/// when chaining edges into a wire).
///
/// Samples in the CANONICAL direction (from param_min to param_max) regardless
/// of the edge's stored param_range orientation. This ensures that the same
/// STEP edge_curve always produces identical 3D points when sampled, even when
/// adjacent faces have reversed Edge copies with swapped param_ranges.
/// The coedge.forward flag controls whether the result is reversed to match
/// the wire traversal direction.
pub fn sample_edge_points(edge: &Edge, n_samples: usize) -> Vec<Point3d> {
    let mut pts = Vec::with_capacity(n_samples);
    if let Some(ref curve) = edge.curve {
        let (tmin, tmax) = edge.param_range;
        // Always sample from the smaller parameter to the larger one
        // (canonical direction). This guarantees that two Edge objects
        // referencing the same STEP EDGE_CURVE produce identical point
        // sequences, even if one has a reversed param_range.
        let (pmin, pmax) = if tmin <= tmax { (tmin, tmax) } else { (tmax, tmin) };
        for i in 0..n_samples {
            let t = pmin + (i as f64 / n_samples as f64) * (pmax - pmin);
            pts.push(curve.point_at(t));
        }
    }
    // Apply deterministic rounding for consistent vertex deduplication
    for p in &mut pts {
        *p = crate::edge_cache::deterministic_round_point(*p);
    }
    pts
}

/// Fast UV projection for a sequence of 3D points on a NURBS surface.
///
/// Uses one full `project_point()` call to bootstrap, then chains
/// Newton-Raphson re-projection from each point's predecessor.
/// This is ~8× faster than calling `project_point()` for each point.
fn nurbs_uv_fast_projection(surface: &Surface, points_3d: &[Point3d]) -> Vec<Point2d> {
    if points_3d.is_empty() {
        return Vec::new();
    }

    if let Surface::Nurbs(ref nurbs) = surface {
        let (u_min, u_max) = nurbs.u_range();
        let (v_min, v_max) = nurbs.v_range();
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;
        let use_chain_newton = u_range < 10.0 && v_range < 10.0;

        let mut uvs = Vec::with_capacity(points_3d.len());
        let mut newton_failures = 0usize;

        for (i, p) in points_3d.iter().enumerate() {
            let (u, v) = if use_chain_newton && i > 0 && !uvs.is_empty() {
                let prev: Point2d = uvs[i - 1];
                crate::parametric_domain::reproject_nurbs_point(nurbs, p, prev.u, prev.v)
            } else {
                surface.project_point(p)
            };

            // Validate: check that the projected UV maps back close to the target
            let reconstructed = surface.point_at(u, v);
            let error = p.distance_to(&reconstructed);

            if error > 1e-4 {
                // Projection failed — try brute-force grid search
                let grid_size = crate::edge_cache::adaptive_grid_size(u_range, v_range);
                let (ub, vb) = crate::edge_cache::brute_force_project_point(nurbs, p, grid_size);
                let bf_p = surface.point_at(ub, vb);
                let bf_err = p.distance_to(&bf_p);

                if bf_err < error {
                    uvs.push(Point2d::new(ub, vb));
                } else {
                    uvs.push(Point2d::new(u, v));
                }
                newton_failures += 1;
            } else {
                uvs.push(Point2d::new(u, v));
            }
        }
        if newton_failures > 0 {
            log::warn!(
                "NURBS fast projection: {}/{} projections failed (u_range={:.1}, v_range={:.1})",
                newton_failures, points_3d.len(), u_range, v_range,
            );
        }
        uvs
    } else {
        // Non-NURBS: project_point() is fast, use it directly
        points_3d.iter().map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        }).collect()
    }
}

// ============================================================
// Cached boundary collection — uses EdgeDiscretizationCache
// ============================================================

/// Pre-populate the cache with all edge discretizations for a single face.
///
/// This is used in the sequential path where each face's edges are
/// lazily populated into the cache. After calling this, the cache
/// contains entries for all non-degenerate edges of this face,
/// and the face can be triangulated using only `&EdgeDiscretizationCache`.
fn pre_populate_face_edges(cache: &mut EdgeDiscretizationCache, face: &Face, surface: &Surface) {
    // Outer wire
    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            if let Some(edge) = face.edges.iter().find(|e| e.id == coedge.edge) {
                if edge.degenerate { continue; }
                cache.discretize_edge(edge, face.id, surface, EDGE_SAMPLES, coedge.curve_2d.as_ref());
            }
        }
    }
    // Inner wires
    for wire in &face.inner_wires {
        for coedge in &wire.coedges {
            if let Some(edge) = face.edges.iter().find(|e| e.id == coedge.edge) {
                if edge.degenerate { continue; }
                cache.discretize_edge(edge, face.id, surface, EDGE_SAMPLES, coedge.curve_2d.as_ref());
            }
        }
    }
}

/// Collect boundary points from a face's outer wire using the edge discretization cache.
///
/// This replaces `collect_face_boundary_points` in the main pipeline.
/// The cache ensures that shared edges between adjacent faces produce
/// identical 3D point sequences, which is critical for watertight meshes.
///
/// NOTE: This function requires that the cache has been pre-populated for
/// this face (via `pre_populate_face_edges` or `pre_populate_for_solid_full`).
fn collect_face_boundary_from_cache(
    face: &Face,
    cache: &EdgeDiscretizationCache,
    surface: &Surface,
) -> Vec<Point3d> {
    let mut points = Vec::new();

    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate { continue; }

                // Get cached discretization
                if let Some(disc) = cache.get(edge.id) {
                    let mut edge_pts = disc.points_3d.clone();

                    // Same reversal logic as collect_face_boundary_points
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts.reverse();
                    }
                    points.extend(edge_pts);
                } else {
                    // Fallback: should not happen if cache was pre-populated
                    log::warn!("Edge {} not found in cache for face {}, falling back to sample_edge_points", edge.id, face.id);
                    let mut edge_pts = sample_edge_points(edge, EDGE_SAMPLES);
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts.reverse();
                    }
                    points.extend(edge_pts);
                }
            }
        }
    }

    // Remove duplicate consecutive points (within tolerance)
    if !points.is_empty() {
        let mut unique = vec![points[0]];
        for p in &points[1..] {
            if let Some(last) = unique.last() {
                if !last.is_coincident_with(p) {
                    unique.push(*p);
                }
            }
        }
        // Also check last vs first (closed loop)
        if unique.len() > 1 {
            if let Some(last) = unique.last() {
                if last.is_coincident_with(&unique[0]) {
                    unique.pop();
                }
            }
        }
        points = unique;
    }

    points
}

/// Collect boundary points AND UV coordinates from a face's outer wire using the cache.
///
/// This replaces `collect_face_boundary_points_with_uv` in the main pipeline.
/// UV coordinates are retrieved from the cache's per-face UV map.
fn collect_face_boundary_with_uv_from_cache(
    face: &Face,
    cache: &EdgeDiscretizationCache,
    surface: &Surface,
) -> (Vec<Point3d>, Vec<Point2d>) {
    let mut points_3d = Vec::new();
    let mut points_uv = Vec::new();

    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate { continue; }

                if let Some(disc) = cache.get(edge.id) {
                    let mut edge_pts_3d = disc.points_3d.clone();

                    // Get UV for this face; if missing, compute as fallback
                    let mut edge_pts_uv = if let Some(uvs) = disc.uv_per_face.get(&face.id) {
                        uvs.clone()
                    } else {
                        // Fallback: compute UVs from the surface projection
                        log::debug!("Computing UV fallback for edge {} face {}", edge.id, face.id);
                        EdgeDiscretizationCache::compute_uvs(
                            &disc.points_3d, &disc.params, surface, coedge.curve_2d.as_ref(),
                        )
                    };

                    // Same reversal logic — both 3D and UV must be reversed together
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts_3d.reverse();
                        edge_pts_uv.reverse();
                    }

                    points_3d.extend(edge_pts_3d);
                    points_uv.extend(edge_pts_uv);
                } else {
                    // Fallback: should not happen if cache was pre-populated
                    log::warn!("Edge {} not found in cache for face {}, falling back to uncached collection", edge.id, face.id);
                    // Use the old uncached path for this edge
                    let mut edge_pts_3d = sample_edge_points(edge, EDGE_SAMPLES);
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;

                    let edge_pts_uv = nurbs_uv_fast_projection(surface, &edge_pts_3d);

                    if should_reverse {
                        edge_pts_3d.reverse();
                    }
                    let mut edge_pts_uv = edge_pts_uv;
                    if should_reverse {
                        edge_pts_uv.reverse();
                    }

                    points_3d.extend(edge_pts_3d);
                    points_uv.extend(edge_pts_uv);
                }
            }
        }
    }

    // Remove duplicate consecutive points (within tolerance) — keep 3D and UV in sync
    if !points_3d.is_empty() {
        let mut unique_3d = vec![points_3d[0]];
        let mut unique_uv = vec![points_uv[0]];
        for i in 1..points_3d.len() {
            if let Some(last) = unique_3d.last() {
                if !last.is_coincident_with(&points_3d[i]) {
                    unique_3d.push(points_3d[i]);
                    unique_uv.push(points_uv[i]);
                }
            }
        }
        // Also check last vs first (closed loop)
        if unique_3d.len() > 1 {
            if let Some(last) = unique_3d.last() {
                if last.is_coincident_with(&unique_3d[0]) {
                    unique_3d.pop();
                    unique_uv.pop();
                }
            }
        }
        points_3d = unique_3d;
        points_uv = unique_uv;
    }

    (points_3d, points_uv)
}

/// Collect hole boundary points from a face's inner wires using the cache.
///
/// This replaces `collect_face_hole_points` in the main pipeline.
fn collect_face_holes_from_cache(
    face: &Face,
    cache: &EdgeDiscretizationCache,
    surface: &Surface,
) -> Vec<Vec<Point3d>> {
    let mut holes = Vec::new();
    for wire in &face.inner_wires {
        let mut points = Vec::new();
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate { continue; }

                if let Some(disc) = cache.get(edge.id) {
                    let mut edge_pts = disc.points_3d.clone();
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts.reverse();
                    }
                    points.extend(edge_pts);
                } else {
                    let mut edge_pts = sample_edge_points(edge, EDGE_SAMPLES);
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts.reverse();
                    }
                    points.extend(edge_pts);
                }
            }
        }
        // Deduplicate
        if !points.is_empty() {
            let mut unique = vec![points[0]];
            for p in &points[1..] {
                if let Some(last) = unique.last() {
                    if !last.is_coincident_with(p) {
                        unique.push(*p);
                    }
                }
            }
            if unique.len() > 1 {
                if let Some(last) = unique.last() {
                    if last.is_coincident_with(&unique[0]) {
                        unique.pop();
                    }
                }
            }
            holes.push(unique);
        }
    }
    holes
}

/// Collect hole boundary points AND UV coordinates from a face's inner wires using the cache.
///
/// This replaces `collect_face_hole_points_with_uv` in the main pipeline.
fn collect_face_holes_with_uv_from_cache(
    face: &Face,
    cache: &EdgeDiscretizationCache,
    surface: &Surface,
) -> (Vec<Vec<Point3d>>, Vec<Vec<Point2d>>) {
    let mut holes_3d = Vec::new();
    let mut holes_uv = Vec::new();

    for wire in &face.inner_wires {
        let mut pts_3d = Vec::new();
        let mut pts_uv = Vec::new();

        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate { continue; }

                if let Some(disc) = cache.get(edge.id) {
                    let mut edge_pts_3d = disc.points_3d.clone();

                    let mut edge_pts_uv = if let Some(uvs) = disc.uv_per_face.get(&face.id) {
                        uvs.clone()
                    } else {
                        EdgeDiscretizationCache::compute_uvs(
                            &disc.points_3d, &disc.params, surface, coedge.curve_2d.as_ref(),
                        )
                    };

                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts_3d.reverse();
                        edge_pts_uv.reverse();
                    }

                    pts_3d.extend(edge_pts_3d);
                    pts_uv.extend(edge_pts_uv);
                } else {
                    // Fallback
                    let mut edge_pts_3d = sample_edge_points(edge, EDGE_SAMPLES);
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    let edge_pts_uv = nurbs_uv_fast_projection(surface, &edge_pts_3d);
                    if should_reverse {
                        edge_pts_3d.reverse();
                    }
                    let mut edge_pts_uv = edge_pts_uv;
                    if should_reverse {
                        edge_pts_uv.reverse();
                    }
                    pts_3d.extend(edge_pts_3d);
                    pts_uv.extend(edge_pts_uv);
                }
            }
        }

        // Deduplicate — keep 3D and UV in sync
        if !pts_3d.is_empty() {
            let mut unique_3d = vec![pts_3d[0]];
            let mut unique_uv = vec![pts_uv[0]];
            for i in 1..pts_3d.len() {
                if let Some(last) = unique_3d.last() {
                    if !last.is_coincident_with(&pts_3d[i]) {
                        unique_3d.push(pts_3d[i]);
                        unique_uv.push(pts_uv[i]);
                    }
                }
            }
            if unique_3d.len() > 1 {
                if let Some(last) = unique_3d.last() {
                    if last.is_coincident_with(&unique_3d[0]) {
                        unique_3d.pop();
                        unique_uv.pop();
                    }
                }
            }
            holes_3d.push(unique_3d);
            holes_uv.push(unique_uv);
        }
    }

    (holes_3d, holes_uv)
}

// ============================================================
// Planar face triangulation — minimum triangle count
// ============================================================

/// Triangulate a planar face.
/// Uses ear-clipping on the boundary polygon — this produces the minimum
/// number of triangles for a given boundary polygon (N-2 for convex).
/// Supports holes via bridge-edge technique.
fn triangulate_planar_face(face: &Face, plane: &Plane, _params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Use cached boundary collection for watertight meshes
    let surface = Surface::Plane(plane.clone());
    let boundary_3d = collect_face_boundary_from_cache(face, cache, &surface);
    if boundary_3d.is_empty() {
        return mesh;
    }

    let holes_3d = collect_face_holes_from_cache(face, cache, &surface);
    let forward = face.forward;

    // NOTE: We intentionally do NOT snap boundary points to the plane here.
    // Snapping was previously done to eliminate numerical drift, but it
    // causes boundary vertices on shared edges to have DIFFERENT 3D positions
    // between adjacent faces. The boundary points come from edge curve sampling
    // (potentially via StepEdgeCache), and snapping breaks the guarantee that
    // shared edges produce identical 3D points.

    // Project 3D boundary points onto the plane's 2D coordinate system
    let project = |p: &Point3d| -> Point2d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    };

    let points_2d: Vec<Point2d> = boundary_3d.iter().map(|p| project(p)).collect();

    if holes_3d.is_empty() {
        // No holes — simple polygon triangulation
        let is_convex = is_convex_polygon(&points_2d);

        if is_convex && boundary_3d.len() >= 3 {
            // Fan triangulation — N-2 triangles for N boundary vertices (minimum)
            for p in &boundary_3d {
                mesh.add_vertex(*p);
            }
            let n = boundary_3d.len() as u32;
            for i in 1..n - 1 {
                if forward {
                    mesh.add_triangle(0, i, i + 1);
                } else {
                    mesh.add_triangle(0, i + 1, i);
                }
            }
        } else {
            // Ear clipping for non-convex polygons
            let triangles = ear_clip(&points_2d);
            for p in &boundary_3d {
                mesh.add_vertex(*p);
            }
            for tri in &triangles {
                if forward {
                    mesh.add_triangle(tri[0], tri[1], tri[2]);
                } else {
                    mesh.add_triangle(tri[0], tri[2], tri[1]);
                }
            }
        }
    } else {
        // Has holes — use earcutr (mapbox/earcut algorithm) which natively
        // supports holes without bridge-edge tricks. The bridge-edge approach
        // fails for circular bolt holes because ear-clipping can produce
        // triangles that span across the thin bridge-edge passage.
        let holes_2d: Vec<Vec<Point2d>> = holes_3d.iter()
            .map(|h| h.iter().map(|p| project(p)).collect())
            .collect();

        match earcutr_triangulate_planar(&points_2d, &boundary_3d, &holes_2d, &holes_3d, forward, plane.normal) {
            Some(m) => return m,
            None => {
                // Fallback to bridge-edge + ear-clip if earcutr fails
                log::warn!("earcutr failed for planar face, falling back to bridge-edge ear-clip");
                let (merged_2d, merged_3d) = merge_holes_into_polygon_planar(
                    &points_2d, &boundary_3d, &holes_2d, &holes_3d,
                );
                let triangles = ear_clip(&merged_2d);
                let filtered_triangles: Vec<[u32; 3]> = triangles.iter()
                    .filter(|tri| {
                        let a = merged_2d[tri[0] as usize];
                        let b = merged_2d[tri[1] as usize];
                        let c = merged_2d[tri[2] as usize];
                        let centroid_u = (a.u + b.u + c.u) / 3.0;
                        let centroid_v = (a.v + b.v + c.v) / 3.0;
                        for hole in &holes_2d {
                            if point_in_polygon_check(&Point2d::new(centroid_u, centroid_v), hole) {
                                return false;
                            }
                        }
                        point_in_polygon_check(&Point2d::new(centroid_u, centroid_v), &points_2d)
                    })
                    .cloned()
                    .collect();
                for p in &merged_3d {
                    mesh.add_vertex(*p);
                }
                for tri in &filtered_triangles {
                    if forward {
                        mesh.add_triangle(tri[0], tri[1], tri[2]);
                    } else {
                        mesh.add_triangle(tri[0], tri[2], tri[1]);
                    }
                }
            }
        }
    }

    // Compute face normals for the planar face
    let normal = if forward {
        plane.normal
    } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };
    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);

    mesh
}

/// Triangulate a planar face with holes using the earcutr (mapbox/earcut) algorithm.
///
/// Unlike the bridge-edge + ear-clip approach, earcutr natively handles holes
/// by connecting them to the outer boundary using an optimal Z-order curve strategy.
/// This produces correct triangulations for circular bolt holes and other complex
/// hole shapes where bridge edges fail.
///
/// Returns None if the input is degenerate (too few points, zero-area polygon, etc.),
/// in which case the caller should fall back to bridge-edge ear-clip.
fn earcutr_triangulate_planar(
    outer_2d: &[Point2d],
    outer_3d: &[Point3d],
    holes_2d: &[Vec<Point2d>],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    plane_normal: Direction3d,
) -> Option<TriangleMesh> {
    if outer_2d.len() < 3 {
        return None;
    }

    let mut mesh = TriangleMesh::new();

    // Build the flat coordinate array for earcutr.
    // Layout: [outer_pts...][hole0_pts...][hole1_pts...]
    // earcutr expects coordinates as [x0,y0, x1,y1, ...] (2D flat)
    let mut coords: Vec<f64> = Vec::with_capacity((outer_2d.len() + holes_2d.iter().map(|h| h.len()).sum::<usize>()) * 2);
    let mut hole_indices: Vec<usize> = Vec::with_capacity(holes_2d.len());

    // Outer boundary points
    for p in outer_2d {
        coords.push(p.u);
        coords.push(p.v);
    }

    // Hole points — each hole starts at the current vertex count
    // Track which holes are included so the 3D vertex array stays in sync
    let mut valid_hole_indices: Vec<usize> = Vec::new();
    for (hi, hole) in holes_2d.iter().enumerate() {
        if hole.len() < 3 {
            continue;
        }
        valid_hole_indices.push(hi);
        hole_indices.push(coords.len() / 2);
        for p in hole {
            coords.push(p.u);
            coords.push(p.v);
        }
    }

    // Run earcutr triangulation
    let triangle_indices = earcutr::earcut(&coords, &hole_indices, 2);

    if triangle_indices.is_empty() {
        return None;
    }

    // Build combined 3D vertex array: outer vertices first, then valid hole vertices
    // CRITICAL: Only include holes that were also added to coords, so 3D indices
    // match earcutr's 2D indices exactly.
    let mut all_3d: Vec<Point3d> = outer_3d.to_vec();
    for &hi in &valid_hole_indices {
        all_3d.extend_from_slice(&holes_3d[hi]);
    }

    // Verify that all triangle indices are within bounds
    let n_verts = coords.len() / 2;
    for &idx in &triangle_indices {
        if idx as usize >= n_verts {
            log::warn!("earcutr produced out-of-bounds index {} (max {})", idx, n_verts - 1);
            return None;
        }
    }

    // Add vertices and triangles to the mesh
    for p in &all_3d {
        mesh.add_vertex(*p);
    }

    // earcutr produces triangles as [i0, i1, i2, i0, i1, i2, ...]
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 {
            break;
        }
        let a = chunk[0] as u32;
        let b = chunk[1] as u32;
        let c = chunk[2] as u32;

        // Skip degenerate triangles
        if a == b || b == c || a == c {
            continue;
        }

        // Verify vertices are valid
        if (a as usize) < all_3d.len() && (b as usize) < all_3d.len() && (c as usize) < all_3d.len() {
            if forward {
                mesh.add_triangle(a, b, c);
            } else {
                mesh.add_triangle(a, c, b);
            }
        }
    }

    // Compute face normals for the planar face using the analytical plane normal
    if mesh.triangles.is_empty() {
        return None;
    }

    let normal = if forward {
        plane_normal
    } else {
        Direction3d::new(-plane_normal.x, -plane_normal.y, -plane_normal.z).unwrap_or(Direction3d::Z)
    };

    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);

    Some(mesh)
}

/// Result of finding a bridge edge between outer polygon and a hole.
struct BridgeResult {
    outer_idx: usize,
    hole_idx: usize,
}

/// Merge holes into the outer polygon using the bridge-edge technique.
/// Returns the merged polygon as flat (2D, 3D) arrays suitable for ear-clipping.
///
/// This function rebuilds the polygon as a flat array after each hole insertion,
/// ensuring that bridge-edge search always operates on the current polygon and
/// indices remain consistent across multiple holes.
fn merge_holes_into_polygon_planar(
    outer_2d: &[Point2d],
    outer_3d: &[Point3d],
    holes_2d: &[Vec<Point2d>],
    holes_3d: &[Vec<Point3d>],
) -> (Vec<Point2d>, Vec<Point3d>) {
    if outer_2d.is_empty() {
        return (Vec::new(), Vec::new());
    }
    if holes_2d.is_empty() {
        return (outer_2d.to_vec(), outer_3d.to_vec());
    }

    let mut poly_2d: Vec<Point2d> = outer_2d.to_vec();
    let mut poly_3d: Vec<Point3d> = outer_3d.to_vec();

    // Sort holes by rightmost point (u-coordinate) descending
    let mut hole_indices: Vec<usize> = (0..holes_2d.len()).collect();
    hole_indices.sort_by(|&a, &b| {
        let max_u_a = holes_2d[a].iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
        let max_u_b = holes_2d[b].iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
        max_u_b.partial_cmp(&max_u_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    for hole_idx in hole_indices {
        let hole_2d = &holes_2d[hole_idx];
        let hole_3d = &holes_3d[hole_idx];
        if hole_2d.is_empty() { continue; }

        let bridge_result = find_bridge_edge(&poly_2d, hole_2d);

        // Rotate hole to start at bridge point
        let n_hole = hole_2d.len();
        let mut rotated_hole_2d = Vec::with_capacity(n_hole + 1);
        let mut rotated_hole_3d = Vec::with_capacity(n_hole + 1);
        for i in 0..=n_hole {
            let idx = (bridge_result.hole_idx + i) % n_hole;
            rotated_hole_2d.push(hole_2d[idx]);
            rotated_hole_3d.push(hole_3d[idx]);
        }

        // Insert hole into polygon at the bridge point
        let mut new_poly_2d = Vec::new();
        let mut new_poly_3d = Vec::new();

        // Part 1: outer polygon up to and including the bridge point
        for i in 0..=bridge_result.outer_idx {
            new_poly_2d.push(poly_2d[i]);
            new_poly_3d.push(poly_3d[i]);
        }

        // Part 2: bridge to hole (rightmost point)
        new_poly_2d.push(hole_2d[bridge_result.hole_idx]);
        new_poly_3d.push(hole_3d[bridge_result.hole_idx]);

        // Part 3: hole vertices starting from rightmost+1 going around back to rightmost
        for i in 1..rotated_hole_2d.len() {
            new_poly_2d.push(rotated_hole_2d[i]);
            new_poly_3d.push(rotated_hole_3d[i]);
        }

        // Part 4: bridge back to the same outer polygon point
        new_poly_2d.push(poly_2d[bridge_result.outer_idx]);
        new_poly_3d.push(poly_3d[bridge_result.outer_idx]);

        // Part 5: rest of outer polygon after bridge point
        for i in (bridge_result.outer_idx + 1)..poly_2d.len() {
            new_poly_2d.push(poly_2d[i]);
            new_poly_3d.push(poly_3d[i]);
        }

        poly_2d = new_poly_2d;
        poly_3d = new_poly_3d;
    }

    (poly_2d, poly_3d)
}

/// Find the best bridge edge between an outer polygon and a hole.
/// Uses the rightmost-hole-point / closest-visible-outer-point technique.
///
/// For non-convex polygons (like L-shapes), the simple "closest point" approach
/// can produce bridge edges that cross through the concave part of the polygon,
/// creating self-intersecting merged polygons. This function verifies that the
/// bridge edge doesn't cross any polygon edges before accepting it.
fn find_bridge_edge(outer_2d: &[Point2d], hole_2d: &[Point2d]) -> BridgeResult {
    // Guard against empty inputs
    if hole_2d.is_empty() || outer_2d.is_empty() {
        return BridgeResult { outer_idx: 0, hole_idx: 0 };
    }
    // Find rightmost point of the hole
    let mut hole_idx = 0;
    let mut max_u = hole_2d[0].u;
    for (i, p) in hole_2d.iter().enumerate() {
        if p.u > max_u {
            max_u = p.u;
            hole_idx = i;
        }
    }

    let hole_pt = hole_2d[hole_idx];

    // Sort outer polygon vertices by distance to the rightmost hole point (closest first)
    let mut candidates: Vec<(usize, f64)> = outer_2d.iter().enumerate()
        .map(|(i, p)| {
            let dx = p.u - hole_pt.u;
            let dy = p.v - hole_pt.v;
            (i, dx * dx + dy * dy)
        })
        .collect();
    candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    if candidates.is_empty() {
        return BridgeResult { outer_idx: 0, hole_idx };
    }

    // Try each candidate in order of distance — accept the first visible one
    let fallback_idx = candidates[0].0;
    let bridge = candidates.into_iter().find(|(outer_idx, _)| {
        let outer_pt = outer_2d[*outer_idx];
        is_bridge_visible(outer_2d, hole_pt, outer_pt, *outer_idx)
    });

    let outer_idx = bridge.map(|(idx, _)| idx).unwrap_or(fallback_idx);

    BridgeResult { outer_idx, hole_idx }
}

/// Check if a bridge edge from `hole_pt` to `outer_pt` (at `outer_idx`) is visible,
/// meaning it doesn't cross any edge of the outer polygon.
fn is_bridge_visible(
    outer_2d: &[Point2d],
    hole_pt: Point2d,
    outer_pt: Point2d,
    outer_idx: usize,
) -> bool {
    let n = outer_2d.len();

    // Check if the bridge edge intersects any edge of the outer polygon.
    // We skip the two edges adjacent to outer_idx since they share the endpoint.
    for i in 0..n {
        let j = (i + 1) % n;

        // Skip edges adjacent to the bridge endpoint
        if i == outer_idx || j == outer_idx {
            continue;
        }

        let a = outer_2d[i];
        let b = outer_2d[j];

        if segments_intersect(hole_pt, outer_pt, a, b) {
            return false;
        }
    }

    // Also check that the bridge edge doesn't go outside the polygon
    // by verifying the midpoint is inside the outer polygon
    let mid_u = (hole_pt.u + outer_pt.u) / 2.0;
    let mid_v = (hole_pt.v + outer_pt.v) / 2.0;
    let mid = Point2d::new(mid_u, mid_v);
    if !point_in_polygon_check(&mid, outer_2d) {
        return false;
    }

    true
}

/// Check if two line segments (p1→p2 and p3→p4) properly intersect.
/// Uses the orientation test approach.
fn segments_intersect(p1: Point2d, p2: Point2d, p3: Point2d, p4: Point2d) -> bool {
    let d1 = cross_2d(p3, p4, p1);
    let d2 = cross_2d(p3, p4, p2);
    let d3 = cross_2d(p1, p2, p3);
    let d4 = cross_2d(p1, p2, p4);

    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }

    // Check collinear cases (degenerate — treat as non-intersecting for robustness)
    false
}

/// Cross product of vectors (p2-p1) and (p3-p1) in 2D.
fn cross_2d(p1: Point2d, p2: Point2d, p3: Point2d) -> f64 {
    (p2.u - p1.u) * (p3.v - p1.v) - (p2.v - p1.v) * (p3.u - p1.u)
}

/// Point-in-polygon test using ray casting for 2D points.
/// (Used by bridge edge visibility checking for hole merging)
fn point_in_polygon_check(point: &Point2d, polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let yi = polygon[i].v;
        let yj = polygon[j].v;
        let xi = polygon[i].u;
        let xj = polygon[j].u;
        if ((yi > point.v) != (yj > point.v))
            && (point.u < (xj - xi) * (point.v - yi) / (yj - yi) + xi)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ============================================================
// Unified ring-based surface triangulation
// ============================================================

/// Information about a single ring row in a ring-based surface.
struct RingRow {
    /// Base vertex index for this row.
    base_index: u32,
    /// Number of vertices in this row (1 for degenerate/pole rows, n_u for normal rows).
    vertex_count: usize,
}

/// Triangulate a ring-based surface using a unified algorithm.
///
/// Ring-based surfaces (cylinder, cone, sphere, torus) share a common
/// parameterization: u = angle around the axis, v = position along the axis.
/// This function generates vertices in rings (one per v-step) and connects
/// them with triangle strips, handling:
/// - Periodic u (seam closing when full_circle)
/// - Degenerate rows (cone apex, sphere poles with single vertex)
/// - Partial u/v ranges
fn triangulate_ring_surface(
    n_u: usize,
    n_v: usize,
    full_circle: bool,
    forward: bool,
    point_at: impl Fn(usize, usize) -> Point3d,       // (i_u, j_v) -> Point3d
    normal_at: impl Fn(usize, usize) -> Direction3d,   // (i_u, j_v) -> Direction3d
    is_degenerate_row: impl Fn(usize) -> bool,          // j_v -> is this row degenerate (1 vertex)?
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Generate vertices for each ring row
    let mut rows: Vec<RingRow> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        if is_degenerate_row(j) {
            // Degenerate row — single vertex (pole or apex)
            let p = point_at(0, j);
            let n = normal_at(0, j);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            rows.push(RingRow { base_index: idx, vertex_count: 1 });
            total_vertices += 1;
        } else {
            // Normal ring row — n_u vertices
            let base = total_vertices;
            for i in 0..n_u {
                let p = point_at(i, j);
                let n = normal_at(i, j);
                let idx = mesh.add_vertex(p);
                mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            }
            rows.push(RingRow { base_index: base, vertex_count: n_u });
            total_vertices += n_u as u32;
        }
    }

    // Generate triangle strips between adjacent rows
    for j in 0..n_v {
        let row = &rows[j];
        let next_row = &rows[j + 1];

        if next_row.vertex_count == 1 {
            // Next row is a single vertex (pole/apex) — fan triangulation
            let apex = next_row.base_index;
            let loop_count = if full_circle { row.vertex_count } else { row.vertex_count - 1 };
            for i in 0..loop_count {
                let i_next = (i + 1) % row.vertex_count;
                let v0 = row.base_index + i as u32;
                let v1 = row.base_index + i_next as u32;
                if v0 != v1 {
                    if forward {
                        mesh.add_triangle(v0, v1, apex);
                    } else {
                        mesh.add_triangle(v1, v0, apex);
                    }
                }
            }
        } else if row.vertex_count == 1 {
            // Current row is a single vertex (pole) — fan from pole
            let pole = row.base_index;
            let loop_count = if full_circle { next_row.vertex_count } else { next_row.vertex_count - 1 };
            for i in 0..loop_count {
                let i_next = (i + 1) % next_row.vertex_count;
                let v1 = next_row.base_index + i as u32;
                let v2 = next_row.base_index + i_next as u32;
                if v1 != v2 {
                    if forward {
                        mesh.add_triangle(pole, v1, v2);
                    } else {
                        mesh.add_triangle(pole, v2, v1);
                    }
                }
            }
        } else {
            // Both rows are normal rings — quad strip
            let loop_count = if full_circle { row.vertex_count } else { row.vertex_count - 1 };
            for i in 0..loop_count {
                let i_next = (i + 1) % row.vertex_count;
                let v0 = row.base_index + i as u32;
                let v1 = row.base_index + i_next as u32;
                let v2 = next_row.base_index + i_next as u32;
                let v3 = next_row.base_index + i as u32;
                if forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }
    }

    mesh
}

// ============================================================
// Curved surface triangulation — boundary-consistent
// ============================================================

/// Triangulate a cylinder face.
/// The boundary ring vertices are sampled from edge curves (ensuring consistency
/// with adjacent faces). Interior vertices are sampled from the parametric grid.
/// Top/bottom cap rings are snapped to edge curves when available.
///
/// A cylinder is periodic in u with period 2π. When the boundary doesn't
/// constrain the u direction (u_range < ~half period), we use the full
/// u range [0, 2π] — this handles the common case of a full cylinder
/// where the boundary edges are only the top/bottom circles and seam.
fn triangulate_cylinder_face(face: &Face, cyl: &CylinderSurface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let surface = Surface::Cylinder(cyl.clone());

    // Use cached boundary collection WITH UVs for consistency.
    // This uses PCURVE-based UVs when available (more accurate than project_point)
    // and ensures UVs match the edge cache's pre-computed values.
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(face, cache, &surface);

    if boundary_3d.is_empty() {
        // No boundary edges — sample full cylinder
        return triangulate_cylinder_full(face, cyl, params);
    }

    // Detect "full U-period wrap" tube face and use grid triangulation for watertightness.
    if boundary_uvs.len() >= 4 && is_full_u_period_wrap(&boundary_uvs, 2.0 * PI) {
        let (hole_polylines, _hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);
        if hole_polylines.is_empty() {
            log::info!(
                "Cylinder face #{}: full U-period wrap detected ({} bnd pts, u-range≈2π) — using tube grid triangulation",
                face.id, boundary_3d.len()
            );
            return triangulate_cylinder_tube_from_boundary(cyl, params, &boundary_3d, face.forward);
        }
    }

    // Collect holes from inner loops — critical for faces with through-holes,
    // keyways, and other internal boundaries. Without holes, earcutr fills
    // the entire outer boundary, producing triangles where there should be voids.
    let (hole_polylines, hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);

    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        &boundary_3d,
        &boundary_uvs,
        &hole_polylines,
        &hole_uvs,
        face.forward,
        params,
    )
}

/// Detect whether boundary UVs wrap around the full U period of a periodic surface.
///
/// A "full U-period wrap" face is one whose boundary traverses the entire U range
/// of the surface (e.g., a cylindrical tube face with bottom circle + top circle +
/// 2 seam edges at u=0 and u=2π). In UV space, the boundary's U span is close to
/// the U period (within 5% tolerance).
///
/// Such faces confuse earcutr because the boundary polygon "wraps around" the seam
/// — points near u=0 and u=2π are at the same 3D location but may have different
/// u values in UV. The earcutr triangulation collapses these into too few vertices.
///
/// For these faces, a dedicated grid triangulation (e.g., `triangulate_cylinder_tube`)
/// produces a proper watertight mesh.
fn is_full_u_period_wrap(boundary_uvs: &[draper_geometry::Point2d], u_period: f64) -> bool {
    if boundary_uvs.is_empty() || u_period <= 0.0 {
        return false;
    }
    // Normalize all u values to [0, u_period) first.
    let mut us: Vec<f64> = boundary_uvs.iter().map(|p| {
        let mut u = p.u % u_period;
        if u < 0.0 { u += u_period; }
        u
    }).collect();
    us.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // The "wrapped range" is the total U span accounting for periodicity.
    // If the points wrap around, the largest gap between consecutive sorted
    // u values (including wrap from last to first+period) represents the
    // "missing" portion. The wrapped range is period - max_gap.
    let n = us.len();
    if n < 2 { return false; }
    let mut max_gap = 0.0f64;
    for i in 0..n {
        let next = if i + 1 < n { us[i + 1] } else { us[0] + u_period };
        let gap = next - us[i];
        if gap > max_gap { max_gap = gap; }
    }
    let wrapped_range = u_period - max_gap;
    let is_full = wrapped_range >= u_period * 0.95;
    log::debug!(
        "is_full_u_period_wrap: n={}, u_period={:.4}, u_min={:.4}, u_max={:.4}, max_gap={:.4}, wrapped_range={:.4} ({:.1}% of period), is_full={}",
        n, u_period, us[0], us[n - 1], max_gap, wrapped_range,
        100.0 * wrapped_range / u_period, is_full
    );
    is_full
}

/// Triangulate a cylinder face whose boundary forms a closed tube
/// (bottom circle + top circle + 2 seam edges wrapping the full U period).
///
/// This produces a watertight grid mesh using the cylinder's analytic
/// surface. The v range is computed from the boundary 3D points (which
/// include the bottom and top circles at specific axis heights).
///
/// CRITICAL for watertightness: The bottom and top ring vertices are taken
/// DIRECTLY from the cached boundary 3D points (shared with adjacent plane
/// faces via the edge cache), so the resulting mesh has bit-identical
/// vertices on shared edges.
fn triangulate_cylinder_tube_from_boundary(
    cyl: &CylinderSurface,
    params: &TriangulationParams,
    boundary_3d: &[draper_geometry::Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Compute v range from boundary 3D points projected onto cylinder axis.
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for p in boundary_3d {
        let v = (p.x - cyl.origin.x) * cyl.axis.x
              + (p.y - cyl.origin.y) * cyl.axis.y
              + (p.z - cyl.origin.z) * cyl.axis.z;
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if v_min >= v_max {
        v_min = 0.0;
        v_max = 1.0;
    }

    // Split boundary 3D points into bottom ring (v=v_min) and top ring (v=v_max),
    // using axis projection. Each ring is then sorted by angle around the axis
    // so we can use them directly as the grid's boundary rows.
    let (bottom_ring, top_ring) = split_boundary_into_rings(
        boundary_3d, &cyl.origin, &cyl.axis, &cyl.x_dir, v_min, v_max,
    );

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cylinder(cyl.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples.max(8), params.height_samples.max(2))
    };
    // Use the boundary ring point count as n_u so we can use cached points directly.
    // If the rings are too small, fall back to the requested n_u.
    let n_u = n_u.max(bottom_ring.len()).max(top_ring.len());

    // Generate vertices: n_v+1 rows (from v_min to v_max inclusive)
    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            // For boundary rows (j=0 and j=n_v), prefer cached boundary points
            // when available (for watertightness with adjacent plane faces).
            let p = if j == 0 && i < bottom_ring.len() {
                bottom_ring[i]
            } else if j == n_v && i < top_ring.len() {
                top_ring[i]
            } else {
                crate::edge_cache::deterministic_round_point(cyl.point_at(u, v))
            };
            let n = cyl.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // Generate triangles with U wrap-around (mod n_u)
    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Split a closed-tube boundary 3D point list into two sorted rings:
/// bottom (v=v_min) and top (v=v_max).
///
/// Points are classified by axis projection: those close to v_min go to
/// the bottom ring, those close to v_max go to the top ring. Points in
/// between (e.g., seam endpoints at intermediate v) are dropped.
///
/// Each ring is then sorted by angle around the axis (using x_dir as the
/// reference direction for u=0), so the resulting ring order matches the
/// cylinder's U parameterization (counter-clockwise looking down the axis).
fn split_boundary_into_rings(
    boundary_3d: &[draper_geometry::Point3d],
    origin: &draper_geometry::Point3d,
    axis: &draper_geometry::Direction3d,
    x_dir: &draper_geometry::Direction3d,
    v_min: f64,
    v_max: f64,
) -> (Vec<draper_geometry::Point3d>, Vec<draper_geometry::Point3d>) {
    let y_dir = axis.cross(x_dir);
    let v_tol = (v_max - v_min).abs() * 0.05 + 1e-9;

    let mut bottom: Vec<(f64, draper_geometry::Point3d)> = Vec::new();
    let mut top: Vec<(f64, draper_geometry::Point3d)> = Vec::new();

    for p in boundary_3d {
        let dx = p.x - origin.x;
        let dy = p.y - origin.y;
        let dz = p.z - origin.z;
        let v = dx * axis.x + dy * axis.y + dz * axis.z;
        // Compute angle around the axis (using x_dir as reference)
        let x_comp = dx * x_dir.x + dy * x_dir.y + dz * x_dir.z;
        let y_comp = dx * y_dir.x + dy * y_dir.y + dz * y_dir.z;
        let angle = y_comp.atan2(x_comp);

        if (v - v_min).abs() <= v_tol {
            bottom.push((angle, *p));
        } else if (v - v_max).abs() <= v_tol {
            top.push((angle, *p));
        }
        // else: point is at intermediate v (e.g. seam midpoint) — skip
    }

    // Sort by angle
    bottom.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    top.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Strip angles, return just points
    let bottom_pts: Vec<draper_geometry::Point3d> = bottom.into_iter().map(|(_, p)| p).collect();
    let top_pts: Vec<draper_geometry::Point3d> = top.into_iter().map(|(_, p)| p).collect();

    (bottom_pts, top_pts)
}

/// Triangulate a cone face whose boundary forms a closed tube
/// (bottom circle + top circle/apex + 2 seam edges wrapping the full U period).
fn triangulate_cone_tube_from_boundary(
    cone: &ConeSurface,
    params: &TriangulationParams,
    boundary_3d: &[draper_geometry::Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Compute v range from boundary 3D points projected onto cone axis.
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for p in boundary_3d {
        let v = (p.x - cone.origin.x) * cone.axis.x
              + (p.y - cone.origin.y) * cone.axis.y
              + (p.z - cone.origin.z) * cone.axis.z;
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if v_min >= v_max {
        v_min = 0.0;
        v_max = cone.height().min(100.0);
    }

    let apex_v = cone.height();
    let v_max = v_max.min(apex_v);
    let top_row_at_apex = apex_v.is_finite() && (v_max - apex_v).abs() < apex_v * 0.01 + 1e-6;

    // Split boundary 3D points into bottom and top rings (similar to cylinder).
    let (bottom_ring, top_ring) = split_boundary_into_rings(
        boundary_3d, &cone.origin, &cone.axis, &cone.x_dir, v_min, v_max,
    );

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cone(cone.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples.max(8), params.height_samples.max(2))
    };
    let n_u = n_u.max(bottom_ring.len()).max(top_ring.len());

    // Generate vertex grid with apex degeneracy handling
    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;

        if top_row_at_apex && j == n_v {
            // Apex row — single vertex
            let p = cone.point_at(0.0, apex_v);
            let n = cone.normal_at(0.0, apex_v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            row_vertex_offset.push(idx);
            row_vertex_count.push(1);
            total_vertices += 1;
        } else {
            let base = total_vertices;
            row_vertex_offset.push(base);
            row_vertex_count.push(n_u);
            for i in 0..n_u {
                let u = 2.0 * PI * i as f64 / n_u as f64;
                // For boundary rows, prefer cached boundary points
                let p = if j == 0 && i < bottom_ring.len() {
                    bottom_ring[i]
                } else if j == n_v && i < top_ring.len() && !top_row_at_apex {
                    top_ring[i]
                } else {
                    cone.point_at(u, v)
                };
                let n = cone.normal_at(u, v);
                let idx = mesh.add_vertex(p);
                mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            }
            total_vertices += n_u as u32;
        }
    }

    // Generate triangles with U wrap-around (mod n_u)
    for j in 0..n_v {
        let j_next = j + 1;
        let row_count = row_vertex_count[j];
        let next_row_count = row_vertex_count[j_next];
        let row_base = row_vertex_offset[j];
        let next_row_base = row_vertex_offset[j_next];

        if next_row_count == 1 {
            // Next row is apex — fan from current row ring to apex
            let apex = next_row_base;
            for i in 0..row_count {
                let i_next = (i + 1) % row_count;
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                if forward {
                    mesh.add_triangle(v0, v1, apex);
                } else {
                    mesh.add_triangle(v0, apex, v1);
                }
            }
        } else {
            for i in 0..row_count {
                let i_next = (i + 1) % row_count;
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                let v2 = next_row_base + i_next as u32;
                let v3 = next_row_base + i as u32;
                if forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }
    }

    mesh
}

/// Full cylinder triangulation (no boundary edges).
fn triangulate_cylinder_full(face: &Face, cyl: &CylinderSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (v_min, v_max) = compute_axis_v_range(face, &cyl.origin, &cyl.axis);
    let (v_min, v_max) = if v_min < v_max { (v_min, v_max) } else { (0.0, 1.0) };

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cylinder(cyl.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples, params.height_samples.max(2))
    };

    // Generate vertices: n_v+1 rows (from v_min to v_max inclusive)
    // Apply deterministic rounding for consistency with edge cache,
    // even though full cylinders have no shared edges with other faces.
    for j in 0..=n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            let p = crate::edge_cache::deterministic_round_point(cyl.point_at(u, v));
            let n = cyl.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a cone face.
///
/// A cone is periodic in u with period 2π. When the boundary doesn't
/// constrain the u direction, we use the full u range [0, 2π].
/// Handles apex degeneracy: when v_max reaches the apex height, all
/// vertices in the top row collapse to a single point.
fn triangulate_cone_face(face: &Face, cone: &ConeSurface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let surface = Surface::Cone(cone.clone());

    // Use cached boundary collection WITH UVs for consistency.
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(face, cache, &surface);

    if boundary_3d.is_empty() {
        return triangulate_cone_full(face, cone, params);
    }

    // Detect "full U-period wrap" tube face and use grid triangulation.
    if boundary_uvs.len() >= 4 && is_full_u_period_wrap(&boundary_uvs, 2.0 * PI) {
        let (hole_polylines, _hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);
        if hole_polylines.is_empty() {
            log::info!(
                "Cone face #{}: full U-period wrap detected ({} bnd pts, u-range≈2π) — using tube grid triangulation",
                face.id, boundary_3d.len()
            );
            return triangulate_cone_tube_from_boundary(cone, params, &boundary_3d, face.forward);
        }
    }

    // Collect holes from inner loops
    let (hole_polylines, hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);

    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        &boundary_3d,
        &boundary_uvs,
        &hole_polylines,
        &hole_uvs,
        face.forward,
        params,
    )
}

/// Full cone triangulation (no boundary edges).
///
/// Handles apex degeneracy: when the top row of vertices reaches the apex,
/// all vertices collapse to a single point. We generate only 1 apex vertex
/// instead of n_u to avoid degenerate (zero-area) triangles.
fn triangulate_cone_full(face: &Face, cone: &ConeSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (v_min, v_max) = compute_axis_v_range(face, &cone.origin, &cone.axis);
    let (v_min, v_max) = if v_min < v_max { (v_min, v_max) } else { (0.0, cone.height().min(100.0)) };

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cone(cone.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples, params.height_samples.max(2))
    };

    // Clamp v_max to apex height
    let apex_v = cone.height();
    let v_max = v_max.min(apex_v);

    // Check if the top row reaches the apex (radius = 0)
    let top_row_at_apex = apex_v.is_finite() && (v_max - apex_v).abs() < apex_v * 0.01 + 1e-6;

    // Generate vertex grid with apex degeneracy handling
    let mut apex_vertex: Option<u32> = None;
    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;

        if top_row_at_apex && j == n_v {
            // Apex row — single vertex
            let p = cone.point_at(0.0, apex_v);
            let n = cone.normal_at(0.0, apex_v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            apex_vertex = Some(idx);
            row_vertex_offset.push(idx);
            row_vertex_count.push(1);
            total_vertices += 1;
        } else {
            // Normal ring row
            let base = total_vertices;
            row_vertex_offset.push(base);
            row_vertex_count.push(n_u);
            for i in 0..n_u {
                let u = 2.0 * PI * i as f64 / n_u as f64;
                let p = cone.point_at(u, v);
                let n = cone.normal_at(u, v);
                let idx = mesh.add_vertex(p);
                mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            }
            total_vertices += n_u as u32;
        }
    }

    // Generate triangles
    for j in 0..n_v {
        let j_next = j + 1;
        let row_count = row_vertex_count[j];
        let next_row_count = row_vertex_count[j_next];
        let row_base = row_vertex_offset[j];
        let next_row_base = row_vertex_offset[j_next];

        if next_row_count == 1 {
            // Next row is apex — fan from current row ring to apex
            let apex = next_row_base;
            for i in 0..row_count {
                let i_next = (i + 1) % row_count;
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                if v0 != v1 {
                    if face.forward {
                        mesh.add_triangle(v0, v1, apex);
                    } else {
                        mesh.add_triangle(v1, v0, apex);
                    }
                }
            }
        } else {
            // Normal quad strip between two ring rows
            for i in 0..row_count {
                let i_next = (i + 1) % row_count;
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                let v2 = next_row_base + i_next as u32;
                let v3 = next_row_base + i as u32;
                if face.forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }
    }

    mesh
}

/// Triangulate a sphere face.
///
/// A sphere is periodic in u (period 2π) and semi-periodic in v (range [0, π]).
/// When the boundary edges don't constrain a direction, we default to the
/// full range for that direction. This handles both full spheres and partial
/// spherical faces correctly.
///
/// Pole degeneracy handling: At v=0 (north pole) and v=π (south pole),
/// all vertices in that row collapse to a single point. We merge them
/// into a single vertex to avoid degenerate (zero-area) triangles.
fn triangulate_sphere_face(face: &Face, sphere: &SphereSurface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let surface = Surface::Sphere(sphere.clone());

    // Use cached boundary collection WITH UVs for consistency.
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(face, cache, &surface);

    if boundary_3d.is_empty() {
        // No boundary edges — fall back to grid-based full sphere.
        // Since there are no shared edges, watertightness is not a concern here.
        return triangulate_sphere_full_grid(face, sphere, params);
    }

    // Collect holes from inner loops
    let (hole_polylines, hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);

    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        &boundary_3d,
        &boundary_uvs,
        &hole_polylines,
        &hole_uvs,
        face.forward,
        params,
    )
}

/// Full sphere triangulation (no boundary edges) using a simple grid approach.
/// Since there are no shared edges with other faces, watertightness is not a concern.
fn triangulate_sphere_full_grid(face: &Face, sphere: &SphereSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Sphere(sphere.clone()), 0.0, 2.0 * PI, 0.0, PI,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples, (params.angular_samples / 2).max(4))
    };
    let n_v = n_v.max(4);

    // North pole vertex
    let p_north = sphere.point_at(0.0, 0.0);
    let n_north = sphere.normal_at(0.0, 0.0);
    let north_idx = mesh.add_vertex(p_north);
    mesh.add_vertex_normal(north_idx, [n_north.x, n_north.y, n_north.z]);

    // Ring vertices (rows 1..n_v-1)
    for j in 1..n_v {
        let v = PI * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let p = sphere.point_at(u, v);
            let n = sphere.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // South pole vertex
    let p_south = sphere.point_at(0.0, PI);
    let n_south = sphere.normal_at(0.0, PI);
    let south_idx = mesh.add_vertex(p_south);
    mesh.add_vertex_normal(south_idx, [n_south.x, n_south.y, n_south.z]);

    // North pole fan
    let first_ring_base = 1u32;
    for i in 0..n_u {
        let i_next = (i + 1) % n_u;
        let v1 = first_ring_base + i as u32;
        let v2 = first_ring_base + i_next as u32;
        if face.forward {
            mesh.add_triangle(north_idx, v1, v2);
        } else {
            mesh.add_triangle(north_idx, v2, v1);
        }
    }

    // Ring strips
    for j in 1..(n_v - 1) {
        let row_base = 1 + ((j - 1) * n_u) as u32;
        let next_row_base = 1 + (j * n_u) as u32;
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = row_base + i as u32;
            let v1 = row_base + i_next as u32;
            let v2 = next_row_base + i_next as u32;
            let v3 = next_row_base + i as u32;
            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    // South pole fan
    let last_ring_base = 1 + ((n_v - 2) * n_u) as u32;
    for i in 0..n_u {
        let i_next = (i + 1) % n_u;
        let v1 = last_ring_base + i as u32;
        let v2 = last_ring_base + i_next as u32;
        if face.forward {
            mesh.add_triangle(v1, v2, south_idx);
        } else {
            mesh.add_triangle(v2, v1, south_idx);
        }
    }

    mesh
}

/// Triangulate a torus face.
///
/// A torus is periodic in both u and v (both have period 2π).
/// When the boundary edges don't constrain a periodic direction
/// (i.e. the projected boundary points span less than ~half the period),
/// we assume the face needs the full period in that direction.
/// This handles the common case of a full torus with only a single
/// v-circle boundary edge.
fn triangulate_torus_face(face: &Face, torus: &TorusSurface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let surface = Surface::Torus(torus.clone());

    // Use cached boundary collection WITH UVs for consistency.
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(face, cache, &surface);

    if boundary_3d.is_empty() {
        // No boundary edges — fall back to grid-based full torus.
        // Since there are no shared edges, watertightness is not a concern here.
        return triangulate_torus_full_grid(face, torus, params);
    }

    // A torus is closed in BOTH U and V directions. If the face's boundary
    // forms a degenerate UV polygon (all u values constant or all v values
    // constant), it means the boundary is a 1D loop wrapping around ONE
    // parametric direction, while the OTHER direction is unbounded.
    // In this case, we should triangulate the full torus surface (the
    // boundary is just the "seam" of the closed surface, not a real
    // trimming curve).
    //
    // Example: make_torus() creates a torus with a single edge — the minor
    // circle at u=0, v∈[0, 2π]. The boundary UV is {(0, v) : v ∈ [0, 2π]},
    // which has constant u=0. This is degenerate — the actual face covers
    // the entire torus surface u∈[0, 2π] × v∈[0, 2π].
    if !boundary_uvs.is_empty() {
        let u_min = boundary_uvs.iter().map(|p| p.u).fold(f64::MAX, f64::min);
        let u_max = boundary_uvs.iter().map(|p| p.u).fold(f64::MIN, f64::max);
        let v_min = boundary_uvs.iter().map(|p| p.v).fold(f64::MAX, f64::min);
        let v_max = boundary_uvs.iter().map(|p| p.v).fold(f64::MIN, f64::max);
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;
        // Torus UV range is [0, 2π] × [0, 2π]. A "real" trimmed face should
        // have non-trivial extent in BOTH directions. If extent in either
        // direction is < 1e-6, the boundary is degenerate (1D loop).
        if u_range < 1e-6 || v_range < 1e-6 {
            log::info!(
                "torus_face: degenerate UV boundary (u=[{:.4},{:.4}], v=[{:.4},{:.4}], {} pts) — using full grid",
                u_min, u_max, v_min, v_max, boundary_3d.len(),
            );
            return triangulate_torus_full_grid(face, torus, params);
        }
    }

    // Collect holes from inner loops
    let (hole_polylines, hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);

    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        &boundary_3d,
        &boundary_uvs,
        &hole_polylines,
        &hole_uvs,
        face.forward,
        params,
    )
}

/// Full torus triangulation (no boundary edges) using a doubly-periodic grid.
///
/// A torus is closed in BOTH U and V directions (both have period 2π). We must
/// generate a grid that wraps around in BOTH directions — n_u × n_v vertices
/// with NO duplicate seam vertices. Using `triangulate_ring_surface` here
/// would create n_v+1 rows (j=0..=n_v), duplicating the v=0 / v=2π seam and
/// producing 2*n_u boundary edges. This custom generator avoids that by
/// using modulo wrap-around in BOTH directions.
fn triangulate_torus_full_grid(face: &Face, torus: &TorusSurface, params: &TriangulationParams) -> TriangleMesh {
    let surface = Surface::Torus(torus.clone());
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &surface, 0.0, 2.0 * PI, 0.0, 2.0 * PI,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples.max(8), params.angular_samples.max(8))
    };

    let mut mesh = TriangleMesh::new();

    // Generate n_u × n_v vertices (NO duplicates — v wraps via modulo).
    // Vertex index = j * n_u + i, where i ∈ [0, n_u) and j ∈ [0, n_v).
    for j in 0..n_v {
        let v = 2.0 * PI * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let p = crate::edge_cache::deterministic_round_point(torus.point_at(u, v));
            let n = torus.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // Generate triangle quads with modulo wrap-around in BOTH directions.
    // This produces n_u × n_v quads = 2 × n_u × n_v triangles, with every
    // edge shared by exactly 2 triangles → fully watertight.
    for j in 0..n_v {
        let j_next = (j + 1) % n_v;
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = (j_next * n_u + i_next) as u32;
            let v3 = (j_next * n_u + i) as u32;
            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a revolution surface face.
fn triangulate_revolution_face(face: &Face, rev: &draper_geometry::RevolutionSurface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let surface = face.surface.as_ref().cloned().unwrap_or(Surface::Revolution(rev.clone()));

    // Use cached boundary collection WITH UVs for consistency.
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(face, cache, &surface);

    if boundary_3d.is_empty() {
        // No boundary edges — fallback to full revolution grid without cache snapping
        return triangulate_revolution_full(face, rev, params);
    }

    // Collect holes from inner loops
    let (hole_polylines, hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);

    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        &boundary_3d,
        &boundary_uvs,
        &hole_polylines,
        &hole_uvs,
        face.forward,
        params,
    )
}

/// Full revolution triangulation (no boundary edges) — fallback when cache is empty.
fn triangulate_revolution_full(face: &Face, rev: &draper_geometry::RevolutionSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (v_min, v_max) = rev.profile.param_range();

    let surface = face.surface.as_ref().cloned().unwrap_or(Surface::Revolution(rev.clone()));
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &surface, 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples, params.angular_samples)
    };

    for j in 0..=n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            let p = crate::edge_cache::deterministic_round_point(rev.point_at(u, v));
            let n = face.surface.as_ref().map(|s| s.normal_at(u, v)).unwrap_or(Direction3d::Z);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate an extrusion surface face.
fn triangulate_extrusion_face(face: &Face, ext: &draper_geometry::ExtrusionSurface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let surface = face.surface.as_ref().cloned().unwrap_or(Surface::Extrusion(ext.clone()));

    // Use cached boundary collection WITH UVs for consistency.
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(face, cache, &surface);

    if boundary_3d.is_empty() {
        // No boundary edges — fallback to full extrusion grid without cache snapping
        return triangulate_extrusion_full(face, ext, params);
    }

    // Collect holes from inner loops
    let (hole_polylines, hole_uvs) = collect_face_holes_with_uv_from_cache(face, cache, &surface);

    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        &boundary_3d,
        &boundary_uvs,
        &hole_polylines,
        &hole_uvs,
        face.forward,
        params,
    )
}

/// Full extrusion triangulation (no boundary edges) — fallback when cache is empty.
fn triangulate_extrusion_full(face: &Face, ext: &draper_geometry::ExtrusionSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let (v_min, v_max) = compute_extrusion_v_range(face, ext);
    let (u_min, u_max) = ext.profile.param_range();

    let surface = face.surface.as_ref().cloned().unwrap_or(Surface::Extrusion(ext.clone()));
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &surface, u_min, u_max, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples, params.height_samples.max(2))
    };

    for j in 0..=n_v {
        for i in 0..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            let p = crate::edge_cache::deterministic_round_point(ext.point_at(u, v));
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u - 1 {
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i + 1) as u32;
            let v2 = ((j + 1) * n_u + i + 1) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// NURBS surface triangulation using UV-space earcutr with Newton-Raphson re-projection.
///
/// This function produces high-quality NURBS triangulation by delegating to
/// `parametric_domain::triangulate_surface_consistent()`, which provides:
/// 1. Newton-Raphson re-projection for accurate UV coordinates
/// 2. UV normalization for periodic surfaces
/// 3. UV polygon quality validation (area ratio check)
/// 4. Knot-span subdivision for interior Steiner points
/// 5. earcutr-based triangulation (O(n log n), handles holes natively)
/// 6. Boundary 3D points used directly for watertight meshes
fn triangulate_nurbs_cdt(face: &Face, surface: &Surface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    // Collect boundary points AND UV coordinates using the edge discretization cache.
    // The cache ensures shared edges produce identical 3D points for watertight meshes.
    // UV coordinates come from the cache's per-face UV map (computed from PCURVEs
    // when available, otherwise via surface projection).
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(face, cache, surface);
    if boundary_3d.len() < 3 {
        return TriangleMesh::new();
    }

    let (holes_3d, holes_uvs) = collect_face_holes_with_uv_from_cache(face, cache, surface);

    // Check if UV projection is valid
    if boundary_uvs.iter().any(|uv| !uv.u.is_finite() || !uv.v.is_finite()) {
        log::warn!("NURBS CDT fallback: UV projection NaN/Inf, using generic surface");
        return triangulate_generic_surface(face, surface, params, cache);
    }

    // Check hole UV validity
    if holes_uvs.iter().any(|h| h.iter().any(|uv| !uv.u.is_finite() || !uv.v.is_finite())) {
        log::warn!("NURBS CDT fallback: hole UV NaN/Inf, using generic surface");
        return triangulate_generic_surface(face, surface, params, cache);
    }

    // Use the earcutr-based consistent triangulation approach.
    //
    // RADICAL FIX: The old grid-based approach (triangulate_nurbs_grid_trimmed)
    // had fundamental flaws:
    // 1. Missing triangles at corners — ray-casting containment on regular grid
    //    vertices misclassified corner grid points as "outside", causing gaps
    // 2. No boundary strip triangles — only grid vertex snapping, which doesn't
    //    guarantee coverage when boundary UVs don't align with grid points
    // 3. Over-tessellation — fixed grid resolution regardless of surface curvature
    // 4. Slow — evaluates derivatives_at() for every grid point, even flat regions
    //
    // The earcutr-based approach (triangulate_surface_consistent) fixes all of these:
    // 1. Boundary vertices are part of the triangulation input — earcutr creates
    //    triangles at every corner automatically, no containment check needed
    // 2. Boundary 3D points are used directly — bit-identical for watertight meshes
    // 3. Adaptive interior points — knot-span subdivision + curvature-based sampling
    //    gives more points where the surface curves, fewer where it's flat
    // 4. Chord-error refinement — iterative post-triangulation check ensures
    //    the mesh stays within max_deviation of the true surface
    let result = crate::parametric_domain::triangulate_surface_consistent(
        surface,
        &boundary_3d,
        &boundary_uvs,
        &holes_3d,
        &holes_uvs,
        face.forward,
        params,
    );

    // If the consistent path produced an empty mesh, fall back to generic surface
    if result.vertices.is_empty() {
        log::warn!("NURBS CDT fallback: consistent triangulation returned empty mesh, using generic surface");
        return triangulate_generic_surface(face, surface, params, cache);
    }

    result
}

/// Generate interior Steiner points for NURBS surface approximation.
///
/// NOTE: This function is kept for potential future use but is currently not
/// called from the main code path. NURBS triangulation now delegates to
/// `parametric_domain::triangulate_surface_consistent()` which uses its own
/// `generate_nurbs_interior_points` with knot-span subdivision.
#[allow(dead_code)]
fn generate_nurbs_interior_points(
    boundary_2d: &[[f64; 2]],
    surface: &Surface,
    params: &TriangulationParams,
) -> Vec<[f64; 2]> {
    let mut interior = Vec::new();

    // Compute UV bounding box from boundary
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for uv in boundary_2d.iter() {
        u_min = u_min.min(uv[0]);
        u_max = u_max.max(uv[0]);
        v_min = v_min.min(uv[1]);
        v_max = v_max.max(uv[1]);
    }

    // Use adaptive or fixed resolution
    // Increased minimum from 12/48 to 24/96 for better NURBS surface approximation.
    // Bolt threads and other cylindrical NURBS surfaces need more interior points
    // to avoid jagged, faceted results.
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples_capped(
            surface, u_min, u_max, v_min, v_max,
            params.max_deviation, params.detail_level, params.max_face_triangles,
        )
    } else {
        let n = params.angular_samples.max(24).min(96);
        (n, n)
    };

    let du = (u_max - u_min) / n_u as f64;
    let dv = (v_max - v_min) / n_v as f64;

    for i in 1..n_u {
        for j in 1..n_v {
            let u = u_min + i as f64 * du;
            let v = v_min + j as f64 * dv;

            // Check if (u, v) is inside the boundary polygon
            if !crate::custom_cdt::point_in_polygon([u, v], boundary_2d) {
                continue;
            }

            // Check if point on surface is valid
            let p3d = surface.point_at(u, v);
            if !p3d.x.is_finite() || !p3d.y.is_finite() || !p3d.z.is_finite() {
                continue;
            }

            interior.push([u, v]);
        }
    }

    interior
}

/// Generic surface triangulation by sampling on a grid.
/// For NURBS surfaces, uses the actual knot range.
fn triangulate_generic_surface(face: &Face, surface: &Surface, params: &TriangulationParams, cache: &EdgeDiscretizationCache) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let (base_u_min, base_u_max, base_v_min, base_v_max) = if let Surface::Nurbs(nurbs) = surface {
        let (u0, u1) = nurbs.u_range();
        let (v0, v1) = nurbs.v_range();
        (u0, u1, v0, v1)
    } else {
        (0.0, 2.0 * PI, 0.0, PI)
    };

    // For NURBS surfaces, we already know the exact UV range from the knot
    // vectors — there's no need to project boundary points to refine it.
    // project_point() on NURBS is catastrophically slow (32×32 grid search
    // + Newton-Raphson ≈ 1767 evaluations per point), so we avoid it entirely.
    let (u_min, u_max, v_min, v_max) = if let Surface::Nurbs(_) = surface {
        (base_u_min, base_u_max, base_v_min, base_v_max)
    } else if let Some(ref wire) = face.outer_wire {
        if wire.coedges.is_empty() {
            (base_u_min, base_u_max, base_v_min, base_v_max)
        } else {
            let boundary_pts = collect_face_boundary_from_cache(face, cache, surface);
            if !boundary_pts.is_empty() {
                let mut proj_u_min = f64::MAX;
                let mut proj_u_max = f64::MIN;
                let mut proj_v_min = f64::MAX;
                let mut proj_v_max = f64::MIN;
                for p in &boundary_pts {
                    let (u, v) = surface.project_point(p);
                    proj_u_min = proj_u_min.min(u);
                    proj_u_max = proj_u_max.max(u);
                    proj_v_min = proj_v_min.min(v);
                    proj_v_max = proj_v_max.max(v);
                }
                let u0 = proj_u_min.max(base_u_min);
                let u1 = proj_u_max.min(base_u_max);
                let v0 = proj_v_min.max(base_v_min);
                let v1 = proj_v_max.min(base_v_max);
                if u0 < u1 && v0 < v1 {
                    let margin_u = (u1 - u0) * 0.01;
                    let margin_v = (v1 - v0) * 0.01;
                    (u0 - margin_u, u1 + margin_u, v0 - margin_v, v1 + margin_v)
                } else {
                    (base_u_min, base_u_max, base_v_min, base_v_max)
                }
            } else {
                (base_u_min, base_u_max, base_v_min, base_v_max)
            }
        }
    } else {
        (base_u_min, base_u_max, base_v_min, base_v_max)
    };

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            surface, u_min, u_max, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        let n_u = if let Surface::Nurbs(_) = surface {
            params.angular_samples.max(24)
        } else {
            params.angular_samples
        };
        let n_v = if let Surface::Nurbs(_) = surface {
            params.angular_samples.max(24)
        } else {
            params.angular_samples
        };
        (n_u, n_v)
    };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = surface.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u - 1 {
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i + 1) as u32;
            let v2 = ((j + 1) * n_u + i + 1) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

// ============================================================
// Boundary-aware triangulation (new API for STEP converter)
// ============================================================

/// Triangulate a face with boundary points for proper trimming.
/// This is the preferred entry point when boundary 3D points are available
/// from STEP file topology extraction.
pub fn triangulate_face_with_boundary(
    surface: &Surface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    triangulate_face_with_boundary_and_holes(surface, boundary_points, &[], forward, params)
}

/// Triangulate a face with boundary points and optional hole polygons.
/// For curved surfaces, uses UV-space boundary trimming with proper hole exclusion.
pub fn triangulate_face_with_boundary_and_holes(
    surface: &Surface,
    boundary_points: &[Point3d],
    hole_polylines: &[Vec<Point3d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    if boundary_points.is_empty() {
        let wire = Wire::new(vec![]);
        let mut face = Face::new(surface.clone(), wire);
        face.forward = forward;
        face.edges = vec![];
        return triangulate_face(&face, params);
    }

    match surface {
        Surface::Plane(plane) => {
            if hole_polylines.is_empty() {
                // Planes without holes: simple ear-clipping
                triangulate_plane_with_boundary(plane, boundary_points, forward)
            } else {
                // Planes with holes: use earcutr which natively handles holes
                triangulate_plane_with_boundary_and_holes(plane, boundary_points, hole_polylines, forward)
            }
        }
        Surface::Cone(_) | Surface::Sphere(_) | Surface::Cylinder(_) => {
            // Curved surfaces with degeneracies or periodicity:
            // Use earcutr-based consistent triangulation which:
            // 1. Uses boundary 3D points DIRECTLY (watertight by construction)
            // 2. Handles periodicity (cylinder u-seam, cone/sphere poles)
            // 3. Handles holes natively
            // 4. Inserts Steiner points via surface.point_at() for chord error
            if boundary_points.len() < 3 {
                return TriangleMesh::new();
            }

            // Check if the boundary wraps the full u-period (full surface).
            // When a boundary spans the full period (≈2π for revolution surfaces),
            // the UV polygon degenerates because u=0 and u=2π map to the same
            // geometric line (the seam). Earcutr can't handle this — it needs
            // a non-degenerate 2D polygon. Delegate to grid-based full-surface
            // triangulation instead, which handles seams and degeneracies natively.
            let u_period = surface_u_period(surface);
            if is_full_period_boundary(surface, boundary_points, u_period) {
                log::info!(
                    "triangulate_face_with_boundary: full-period boundary detected ({} pts), delegating to grid-based path",
                    boundary_points.len(),
                );
                let wire = Wire::new(vec![]);
                let mut face = Face::new(surface.clone(), wire);
                face.forward = forward;
                face.edges = vec![];
                return triangulate_face(&face, params);
            }

            // Project boundary to UV space using CHAIN-BASED projection.
            // For periodic surfaces, project_point() may return u=0 for points
            // at u=2π, collapsing the UV polygon. Chain projection maintains
            // continuity by adjusting each UV to be close to the previous one.
            let v_period = surface_v_period(surface);
            let boundary_uvs = project_boundary_to_uv_chain(
                surface, boundary_points, u_period, v_period,
            );
            // Project hole polylines to UV (same chain-based approach)
            let hole_uvs: Vec<Vec<Point2d>> = hole_polylines.iter().map(|hole| {
                project_boundary_to_uv_chain(surface, hole, u_period, v_period)
            }).collect();
            crate::parametric_domain::triangulate_surface_consistent(
                surface,
                boundary_points,
                &boundary_uvs,
                hole_polylines,
                &hole_uvs,
                forward,
                params,
            )
        }
        _ => {
            // Other curved surfaces (Torus, Revolution, Extrusion):
            // Use earcutr-based consistent triangulation for watertightness,
            // projecting boundary 3D points to UV space.
            if boundary_points.len() < 3 {
                return TriangleMesh::new();
            }
            let u_period = surface_u_period(surface);
            let v_period = surface_v_period(surface);
            let boundary_uvs = project_boundary_to_uv_chain(
                surface, boundary_points, u_period, v_period,
            );
            let hole_uvs: Vec<Vec<Point2d>> = hole_polylines.iter().map(|hole| {
                project_boundary_to_uv_chain(surface, hole, u_period, v_period)
            }).collect();
            crate::parametric_domain::triangulate_surface_consistent(
                surface,
                boundary_points,
                &boundary_uvs,
                hole_polylines,
                &hole_uvs,
                forward,
                params,
            )
        }
    }
}

/// Triangulate a face with boundary points and optional hole polygons,
/// using **pre-computed UV coordinates** for consistent (watertight) triangulation.
///
/// This is the UV-aware variant of `triangulate_face_with_boundary_and_holes()`.
/// When UV coordinates are available (e.g., from STEP PCURVE or EdgeDiscretizationCache),
/// this function produces meshes where shared edges between adjacent faces have
/// **bit-identical** 3D vertex positions.
///
/// # Key differences from `triangulate_face_with_boundary_and_holes`:
/// - Boundary 3D points are used **directly** (not re-projected from UV),
///   ensuring bit-identical positions across shared edges.
/// - UV coordinates come from the caller (PCURVE or cache), not from
///   `surface.project_point()`, which is slow and inaccurate.
/// - For curved surfaces, uses `triangulate_surface_consistent()` which
///   includes boundary UV points as earcutr vertices directly.
///
/// # Arguments
/// * `surface` — The parametric surface.
/// * `boundary_points` — 3D boundary points (from StepEdgeCache).
/// * `boundary_uvs` — Pre-computed UV coordinates for boundary points.
/// * `hole_polylines` — 3D hole boundary points.
/// * `hole_uvs` — Pre-computed UV coordinates for hole points.
/// * `forward` — Whether face normal matches surface normal.
/// * `params` — Triangulation parameters.
pub fn triangulate_face_with_boundary_and_holes_uv(
    surface: &Surface,
    boundary_points: &[Point3d],
    boundary_uvs: &[Point2d],
    hole_polylines: &[Vec<Point3d>],
    hole_uvs: &[Vec<Point2d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    if boundary_points.is_empty() {
        return TriangleMesh::new();
    }

    // ============================================================
    // Decimate collinear boundary points
    //
    // When the edge cache discretizes shared edges, it produces many
    // intermediate points (e.g., 21 points along a 10mm edge at 0.5mm
    // intervals). For STRAIGHT edges in 3D, these intermediate points
    // are collinear and don't add geometric information.
    //
    // However, they cause watertightness issues for PLANAR faces:
    // - Planar face fan triangulation produces degenerate triangles
    //   for collinear points, which are then removed, leaving the
    //   intermediate vertices unused.
    // - The unused vertices in one face's mesh but not the other's
    //   create boundary edges.
    //
    // Solution: decimate collinear boundary points to just the "corner"
    // vertices (where the boundary changes direction). Both faces sharing
    // the edge will decimate the same way (because they share the edge
    // cache), so they'll both keep the same corner vertices.
    //
    // IMPORTANT: Only apply decimation when the boundary is LARGE (>20 points).
    // For small boundaries (e.g., chamfer faces with 8 points), the decimation
    // might remove corners that are "close to collinear" but actually needed
    // for the face topology.
    // ============================================================
    let (dec_3d, dec_uv) = if boundary_points.len() > 20 {
        decimate_collinear_boundary(boundary_points, boundary_uvs)
    } else {
        (boundary_points.to_vec(), boundary_uvs.to_vec())
    };
    let boundary_points: &[Point3d] = &dec_3d;
    let boundary_uvs: &[Point2d] = &dec_uv;

    let hole_polylines_decimated: Vec<Vec<Point3d>>;
    let hole_uvs_decimated: Vec<Vec<Point2d>>;
    let hole_polylines: &[Vec<Point3d>] = if !hole_polylines.is_empty() {
        let mut hps = Vec::with_capacity(hole_polylines.len());
        let mut huvs = Vec::with_capacity(hole_uvs.len());
        for (hp, huv) in hole_polylines.iter().zip(hole_uvs.iter()) {
            if hp.len() > 20 {
                let (d3d, d2d) = decimate_collinear_boundary(hp, huv);
                hps.push(d3d);
                huvs.push(d2d);
            } else {
                hps.push(hp.clone());
                huvs.push(huv.clone());
            }
        }
        hole_polylines_decimated = hps;
        hole_uvs_decimated = huvs;
        &hole_polylines_decimated
    } else {
        hole_polylines_decimated = Vec::new();
        hole_uvs_decimated = Vec::new();
        hole_polylines
    };
    let hole_uvs: &[Vec<Point2d>] = if !hole_uvs_decimated.is_empty() {
        &hole_uvs_decimated
    } else {
        hole_uvs
    };

    // For planar faces, we don't need UV-space triangulation —
    // the boundary 3D points are already on the plane, and ear-clipping
    // in the plane's 2D coordinate system produces consistent results
    // as long as the boundary 3D points are shared (which StepEdgeCache ensures).
    match surface {
        Surface::Plane(plane) => {
            // Planar faces: use the same ear-clipping approach as the non-UV variant,
            // since boundary 3D points are already bit-identical from the cache.
            if hole_polylines.is_empty() {
                triangulate_plane_with_boundary_and_holes_uv(
                    plane, boundary_points, boundary_uvs, &[], &[], forward,
                )
            } else {
                triangulate_plane_with_boundary_and_holes_uv(
                    plane, boundary_points, boundary_uvs, hole_polylines, hole_uvs, forward,
                )
            }
        }
        Surface::Cylinder(cyl) => {
            // Cylinder: detect full U-period wrap (tube face) and use grid
            // triangulation for watertightness. Otherwise use earcutr.
            if boundary_uvs.len() >= 4
                && hole_polylines.is_empty()
                && is_full_u_period_wrap(boundary_uvs, 2.0 * PI)
            {
                log::info!(
                    "Cylinder face: full U-period wrap detected ({} bnd pts, u-range≈2π) — using tube grid triangulation",
                    boundary_points.len()
                );
                return triangulate_cylinder_tube_from_boundary(cyl, params, boundary_points, forward);
            }
            // Cylinder: use earcutr-based consistent triangulation.
            // The grid-based approach with boundary snapping only approximates
            // boundary vertex positions (nearest grid cell), which can produce
            // gaps on shared edges with non-cylindrical adjacent faces.
            // The consistent path uses cached boundary 3D points directly.
            crate::parametric_domain::triangulate_surface_consistent(
                surface,
                boundary_points,
                boundary_uvs,
                hole_polylines,
                hole_uvs,
                forward,
                params,
            )
        }
        Surface::Cone(cone) => {
            // Cone: detect full U-period wrap (tube face) similarly
            if boundary_uvs.len() >= 4
                && hole_polylines.is_empty()
                && is_full_u_period_wrap(boundary_uvs, 2.0 * PI)
            {
                log::info!(
                    "Cone face: full U-period wrap detected ({} bnd pts, u-range≈2π) — using tube grid triangulation",
                    boundary_points.len()
                );
                return triangulate_cone_tube_from_boundary(cone, params, boundary_points, forward);
            }
            // Cone: must handle apex degeneracy
            triangulate_cone_face_with_boundary_uv(
                cone, boundary_points, boundary_uvs, hole_polylines, hole_uvs, forward, params,
            )
        }
        Surface::Sphere(sphere) => {
            // Sphere: must handle pole degeneracy
            triangulate_sphere_face_with_boundary_uv(
                sphere, boundary_points, boundary_uvs, hole_polylines, hole_uvs, forward, params,
            )
        }
        Surface::Nurbs(_) => {
            // NURBS: use earcutr-based consistent triangulation.
            //
            // The grid-based approach (triangulate_nurbs_grid_trimmed) had
            // critical flaws for watertightness:
            // 1. Only snaps grid vertices near boundary UVs — boundary points
            //    that fall between grid cells are NOT used, creating gaps
                       // 2. Creates its own grid via nurbs.derivatives_at() which
            //    produces different 3D coordinates than cached boundary points
            // 3. Containment check is vertex-based and can miss edge cases
            //
            // The earcutr approach (triangulate_surface_consistent) fixes these:
            // 1. Boundary 3D points are used DIRECTLY — bit-identical for watertightness
            // 2. earcutr creates triangles at every corner automatically
            // 3. Adaptive interior points via knot-span subdivision
            // 4. Chord-error refinement for surface accuracy
            crate::parametric_domain::triangulate_surface_consistent(
                surface,
                boundary_points,
                boundary_uvs,
                hole_polylines,
                hole_uvs,
                forward,
                params,
            )
        }
        _ => {
            // Other curved surfaces (Torus, Revolution, Extrusion):
            // use the consistent UV-space triangulation (earcutr CDT)
            crate::parametric_domain::triangulate_surface_consistent(
                surface,
                boundary_points,
                boundary_uvs,
                hole_polylines,
                hole_uvs,
                forward,
                params,
            )
        }
    }
}

/// Triangulate a planar face with pre-computed UV (2D projected) coordinates.
///
/// For planar faces, "UV" is actually the 2D projection onto the plane's
/// coordinate system. The boundary 3D points are used directly, and the
/// UV coordinates are used only for the ear-clipping/earcutr algorithm.
fn triangulate_plane_with_boundary_and_holes_uv(
    plane: &Plane,
    boundary_points: &[Point3d],
    boundary_uvs: &[Point2d],
    hole_polylines: &[Vec<Point3d>],
    hole_uvs: &[Vec<Point2d>],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_points.len() < 3 {
        return mesh;
    }

    // NOTE: We intentionally do NOT snap boundary points to the plane here.
    // Snapping was previously done to eliminate numerical drift, but it
    // causes boundary vertices on shared edges to have DIFFERENT 3D positions
    // between adjacent faces (e.g., a planar cap face and a cylinder side face
    // share a circular edge — the cap face snaps circle points to the cap plane,
    // the cylinder face doesn't — creating gaps).
    // The boundary points come from the StepEdgeCache which ensures shared
    // edges produce identical 3D points. Snapping breaks this guarantee.

    // Re-project the 3D points onto the plane's 2D coordinate system
    // (This is more accurate than using the passed-in UVs which may come from
    // a different projection method)
    let project = |p: &Point3d| -> Point2d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    };

    let points_2d: Vec<Point2d> = boundary_points.iter().map(|p| project(p)).collect();

    if hole_polylines.is_empty() {
        // No holes — simple polygon triangulation using boundary 3D points directly
        let is_convex = is_convex_polygon(&points_2d);

        if is_convex && boundary_points.len() >= 3 {
            for p in boundary_points {
                mesh.add_vertex(*p);
            }
            let n = boundary_points.len() as u32;
            for i in 1..n - 1 {
                if forward {
                    mesh.add_triangle(0, i, i + 1);
                } else {
                    mesh.add_triangle(0, i + 1, i);
                }
            }
        } else {
            let triangles = ear_clip(&points_2d);
            for p in boundary_points {
                mesh.add_vertex(*p);
            }
            for tri in &triangles {
                if forward {
                    mesh.add_triangle(tri[0], tri[1], tri[2]);
                } else {
                    mesh.add_triangle(tri[0], tri[2], tri[1]);
                }
            }
        }
    } else {
        // Has holes — use earcutr with boundary points (no snap_to_plane)
        let holes_2d: Vec<Vec<Point2d>> = hole_polylines.iter()
            .map(|hole| hole.iter().map(|p| project(p)).collect())
            .collect();

        // Try earcutr first
        if let Some(m) = earcutr_triangulate_planar(
            &points_2d, &boundary_points, &holes_2d, &hole_polylines, forward, plane.normal,
        ) {
            return m;
        }

        // Fallback to ear-clip with bridge edges
        log::warn!("earcutr failed for planar face with UV-aware API, falling back to bridge-edge ear-clip");
        let (merged_2d, merged_3d) = merge_holes_into_polygon_planar(
            &points_2d, &boundary_points, &holes_2d, &hole_polylines,
        );
        let triangles = ear_clip(&merged_2d);
        for p in &merged_3d {
            mesh.add_vertex(*p);
        }
        for tri in &triangles {
            if forward {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            } else {
                mesh.add_triangle(tri[0], tri[2], tri[1]);
            }
        }
    }

    // Compute face normals
    let normal = if forward {
        plane.normal
    } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };
    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);

    mesh
}

/// Triangulate a cone face with pre-computed UV coordinates and hole UV coordinates.
///
/// Handles apex degeneracy (all top-row vertices collapse to a single point)
/// while using the cached boundary 3D points directly.
fn triangulate_cone_face_with_boundary_uv(
    cone: &ConeSurface,
    boundary_points: &[Point3d],
    boundary_uvs: &[Point2d],
    hole_polylines: &[Vec<Point3d>],
    hole_uvs: &[Vec<Point2d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    // Use the consistent UV-aware triangulation path for cones.
    // This properly handles apex degeneracy, hole support, and
    // uses the boundary_uvs from STEP PCURVE data for accuracy.
    let surface = Surface::Cone(cone.clone());
    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        boundary_points,
        boundary_uvs,
        hole_polylines,
        hole_uvs,
        forward,
        params,
    )
}

/// Triangulate a sphere face with pre-computed UV coordinates and hole UV coordinates.
///
/// Handles pole degeneracy while using the cached boundary 3D points directly.
fn triangulate_sphere_face_with_boundary_uv(
    sphere: &SphereSurface,
    boundary_points: &[Point3d],
    boundary_uvs: &[Point2d],
    hole_polylines: &[Vec<Point3d>],
    hole_uvs: &[Vec<Point2d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    // Detect full-sphere case: the boundary UVs cover approximately the full
    // U range [0, 2π] AND the full V range [0, π]. This happens for spheres
    // defined by a degenerate boundary (equator traversed twice + meridian
    // forward + meridian backward) that traces the entire sphere.
    //
    // In this case, the boundary polygon is self-intersecting in UV space
    // (the equator traversed forward and backward creates a "fold"), and
    // earcutr cannot triangulate it correctly. We fall back to a grid-based
    // triangulation that produces a proper watertight sphere with pole
    // degeneracy handling.
    let is_full_sphere = !boundary_uvs.is_empty()
        && hole_polylines.is_empty()
        && {
            let u_min = boundary_uvs.iter().map(|p| p.u).fold(f64::MAX, f64::min);
            let u_max = boundary_uvs.iter().map(|p| p.u).fold(f64::MIN, f64::max);
            let v_min = boundary_uvs.iter().map(|p| p.v).fold(f64::MAX, f64::min);
            let v_max = boundary_uvs.iter().map(|p| p.v).fold(f64::MIN, f64::max);
            let u_range = u_max - u_min;
            let v_range = v_max - v_min;
            // Full sphere: u covers ~2π AND v covers ~π
            u_range > 1.9 * PI && v_range > 0.9 * PI
        };

    if is_full_sphere {
        log::info!(
            "Sphere face: full-sphere boundary detected ({} bnd pts) — using grid triangulation with pole handling",
            boundary_points.len()
        );
        return triangulate_sphere_full_grid_from_boundary(sphere, params, forward);
    }

    // Detect partial-sphere case where the boundary covers full U but only
    // a band of V (e.g., a sphere band between two latitudes). In this case,
    // the boundary has two ring loops at v_min and v_max connected by seam
    // edges at u=0 and u=2π. We use a tube-like grid triangulation.
    let is_full_u_band = !boundary_uvs.is_empty()
        && hole_polylines.is_empty()
        && {
            let u_min = boundary_uvs.iter().map(|p| p.u).fold(f64::MAX, f64::min);
            let u_max = boundary_uvs.iter().map(|p| p.u).fold(f64::MIN, f64::max);
            let v_min = boundary_uvs.iter().map(|p| p.v).fold(f64::MAX, f64::min);
            let v_max = boundary_uvs.iter().map(|p| p.v).fold(f64::MIN, f64::max);
            let u_range = u_max - u_min;
            let v_range = v_max - v_min;
            // Full U band: u covers ~2π, v is a small range not at the poles
            u_range > 1.9 * PI && v_range < 0.9 * PI && v_min > 0.05 && v_max < PI - 0.05
        };

    if is_full_u_band {
        log::info!(
            "Sphere face: full-U band boundary detected ({} bnd pts) — using band grid triangulation",
            boundary_points.len()
        );
        return triangulate_sphere_band_from_boundary(sphere, boundary_points, params, forward);
    }

    // Default path: use the consistent UV-aware triangulation.
    // This properly handles pole degeneracy, hole support, and
    // uses the boundary_uvs from STEP PCURVE data for accuracy.
    let surface = Surface::Sphere(sphere.clone());
    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        boundary_points,
        boundary_uvs,
        hole_polylines,
        hole_uvs,
        forward,
        params,
    )
}

/// Full sphere triangulation using a grid approach with pole degeneracy handling.
/// This is used when the boundary represents the entire sphere (full U and V range).
/// Produces a watertight mesh: 1 north pole vertex, 1 south pole vertex, and
/// (n_v - 1) rings of n_u vertices each. Triangle count = 2 * n_u * (n_v - 1).
fn triangulate_sphere_full_grid_from_boundary(
    sphere: &SphereSurface,
    params: &TriangulationParams,
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let n_u = params.angular_samples.max(8);
    let n_v = (params.angular_samples / 2).max(6);

    // North pole vertex
    let p_north = sphere.point_at(0.0, 0.0);
    let n_north = sphere.normal_at(0.0, 0.0);
    let north_idx = mesh.add_vertex(p_north);
    mesh.add_vertex_normal(north_idx, [n_north.x, n_north.y, n_north.z]);

    // Ring vertices (rows 1..n_v-1) — exclude the poles
    for j in 1..n_v {
        let v = PI * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let p = sphere.point_at(u, v);
            let n = sphere.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // South pole vertex
    let p_south = sphere.point_at(0.0, PI);
    let n_south = sphere.normal_at(0.0, PI);
    let south_idx = mesh.add_vertex(p_south);
    mesh.add_vertex_normal(south_idx, [n_south.x, n_south.y, n_south.z]);

    // North pole fan: north → ring[i] → ring[i+1]
    let first_ring_base = 1u32;  // After north pole (idx 0)
    for i in 0..n_u {
        let i_next = (i + 1) % n_u;
        let v1 = first_ring_base + i as u32;
        let v2 = first_ring_base + i_next as u32;
        if forward {
            mesh.add_triangle(north_idx, v1, v2);
        } else {
            mesh.add_triangle(north_idx, v2, v1);
        }
    }

    // Ring strips: between rows j and j+1 (j from 1 to n_v-2)
    for j in 1..(n_v - 1) {
        let row_base = first_ring_base + ((j - 1) * n_u) as u32;
        let next_row_base = row_base + n_u as u32;
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = row_base + i as u32;
            let v1 = row_base + i_next as u32;
            let v2 = next_row_base + i_next as u32;
            let v3 = next_row_base + i as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    // South pole fan: ring[i] → ring[i+1] → south
    let last_ring_base = south_idx - n_u as u32;
    for i in 0..n_u {
        let i_next = (i + 1) % n_u;
        let v0 = last_ring_base + i as u32;
        let v1 = last_ring_base + i_next as u32;
        if forward {
            mesh.add_triangle(v0, v1, south_idx);
        } else {
            mesh.add_triangle(v1, v0, south_idx);
        }
    }

    mesh
}

/// Sphere band triangulation: full U range, V range is a band not at the poles.
/// Used for sphere bands / strips between two latitudes.
/// Produces a watertight mesh: n_u vertices per ring × (n_v + 1) rings.
fn triangulate_sphere_band_from_boundary(
    sphere: &SphereSurface,
    boundary_points: &[Point3d],
    params: &TriangulationParams,
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Compute V range from boundary points
    let v_min = boundary_points.iter()
        .map(|p| sphere.project_point(p).1)
        .fold(f64::MAX, f64::min)
        .max(0.0);
    let v_max = boundary_points.iter()
        .map(|p| sphere.project_point(p).1)
        .fold(f64::MIN, f64::min)
        .min(PI);

    let n_u = params.angular_samples.max(8);
    let n_v = (params.angular_samples / 4).max(4);

    // Generate grid: (n_v + 1) rows × n_u vertices
    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let p = sphere.point_at(u, v);
            let n = sphere.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // Generate triangles between rows
    for j in 0..n_v {
        let row_base = (j * n_u) as u32;
        let next_row_base = ((j + 1) * n_u) as u32;
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;  // Wrap around for full U
            let v0 = row_base + i as u32;
            let v1 = row_base + i_next as u32;
            let v2 = next_row_base + i_next as u32;
            let v3 = next_row_base + i as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

// ============================================================
// UV-space boundary trimming for curved surfaces
// ============================================================

/// Check if a set of 3D points are approximately coplanar within a given tolerance.
/// Uses the best-fit plane and checks the maximum deviation from it.
fn is_nearly_coplanar(points: &[Point3d], tolerance: f64) -> bool {
    if points.len() < 4 {
        return true; // 3 or fewer points are always coplanar
    }

    // Compute centroid
    let n = points.len() as f64;
    let cx = points.iter().map(|p| p.x).sum::<f64>() / n;
    let cy = points.iter().map(|p| p.y).sum::<f64>() / n;
    let cz = points.iter().map(|p| p.z).sum::<f64>() / n;

    // Compute covariance matrix (3x3, symmetric)
    let mut xx = 0.0_f64; let mut xy = 0.0_f64; let mut xz = 0.0_f64;
    let mut yy = 0.0_f64; let mut yz = 0.0_f64; let mut zz = 0.0_f64;
    for p in points {
        let dx = p.x - cx;
        let dy = p.y - cy;
        let dz = p.z - cz;
        xx += dx * dx; xy += dx * dy; xz += dx * dz;
        yy += dy * dy; yz += dy * dz; zz += dz * dz;
    }

    // Find the normal of the best-fit plane by finding the eigenvector
    // of the covariance matrix corresponding to the smallest eigenvalue.
    // Use the cross product of the two rows of the covariance matrix
    // as an approximation. The normal direction is the direction of
    // minimum variance.
    let candidates = [
        [xy * yz - xz * yy, xz * yz - xy * xz, xx * yy - xy * xy], // row0 × row1
        [xy * xz - xz * yz, yy * xz - yz * xz, xy * xz - xx * yz], // row1 × row2  
        [xx * yz - xz * xy, xy * yz - yz * xz, xz * yz - xz * zz], // row0 × row2
    ];

    // Find the candidate with the largest magnitude (= best normal estimate)
    let mut best_normal = [0.0_f64, 0.0, 1.0];
    let mut max_mag = 0.0_f64;
    for c in &candidates {
        let mag = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        if mag > max_mag {
            max_mag = mag;
            best_normal = [c[0] / mag, c[1] / mag, c[2] / mag];
        }
    }

    if max_mag < 1e-20 {
        // All points are coincident — trivially coplanar
        return true;
    }

    // Check max distance from the plane through centroid with the best normal
    let mut max_dist = 0.0_f64;
    for p in points {
        let dist = ((p.x - cx) * best_normal[0] + (p.y - cy) * best_normal[1] + (p.z - cz) * best_normal[2]).abs();
        if dist > max_dist {
            max_dist = dist;
        }
    }

    max_dist < tolerance
}

/// Test if a 2D point is inside a closed polygon using ray casting.
fn point_in_polygon_2d(point: &Point2d, polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 3 { return false; }
    let mut inside = false;
    let px = point.u;
    let py = point.v;
    let mut j = n - 1;
    for i in 0..n {
        let xi = polygon[i].u;
        let yi = polygon[i].v;
        let xj = polygon[j].u;
        let yj = polygon[j].v;
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Normalize UV polygon for periodic surfaces.
/// Handles wrap-around when boundary points cross the ±π seam.
pub(crate) fn normalize_uv_polygon(boundary_uv: &mut [Point2d], u_period: Option<f64>, v_period: Option<f64>) {
    // Handle u-periodicity
    if let Some(period) = u_period {
        // Find the largest gap and normalize
        let mut us: Vec<f64> = boundary_uv.iter().map(|p| p.u).collect();
        us.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Check for wrap-around: if the range is close to the period,
        // shift values that are far from the cluster
        let u_range = us.last().copied().unwrap_or(0.0) - us.first().copied().unwrap_or(0.0);
        if u_range > period * 0.5 {
            // Find the largest gap — points on the other side of the gap
            // should be shifted by ±period
            let mut max_gap = 0.0f64;
            let mut gap_idx = 0;
            for i in 0..us.len() - 1 {
                let gap = us[i + 1] - us[i];
                if gap > max_gap {
                    max_gap = gap;
                    gap_idx = i;
                }
            }
            // Points after the gap should be shifted down by period
            // (they wrapped from +period to -period)
            let threshold = us[gap_idx];
            for p in boundary_uv.iter_mut() {
                if p.u > threshold + max_gap * 0.5 {
                    p.u -= period;
                }
            }
        }
    }

    // Handle v-periodicity
    if let Some(period) = v_period {
        let mut vs: Vec<f64> = boundary_uv.iter().map(|p| p.v).collect();
        vs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let v_range = vs.last().copied().unwrap_or(0.0) - vs.first().copied().unwrap_or(0.0);
        if v_range > period * 0.5 {
            let mut max_gap = 0.0f64;
            let mut gap_idx = 0;
            for i in 0..vs.len() - 1 {
                let gap = vs[i + 1] - vs[i];
                if gap > max_gap {
                    max_gap = gap;
                    gap_idx = i;
                }
            }
            let threshold = vs[gap_idx];
            for p in boundary_uv.iter_mut() {
                if p.v > threshold + max_gap * 0.5 {
                    p.v -= period;
                }
            }
        }
    }
}

/// Get the period of the surface's u parameter, if periodic.
fn surface_u_period(surface: &Surface) -> Option<f64> {
    match surface {
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) => {
            Some(2.0 * PI)
        }
        _ => None,
    }
}

/// Get the period of the surface's v parameter, if periodic.
fn surface_v_period(surface: &Surface) -> Option<f64> {
    match surface {
        Surface::Torus(_) => Some(2.0 * PI),
        _ => None,
    }
}

/// Project boundary 3D points to UV space using chain-based continuity.
///
/// For periodic surfaces, `surface.project_point()` may return u=0 for points
/// at u=2π (or vice versa), which collapses the UV polygon and produces
/// degenerate triangulation. This function prevents that by adjusting each
/// UV coordinate to be as close as possible to the previous one, adding or
/// subtracting periods as needed.
///
/// # Algorithm
/// 1. Project the first point normally via `surface.project_point()`
/// 2. For each subsequent point, project normally, then:
///    - If the surface is u-periodic and the u jump > period/2,
///      shift the new u by ±period to minimize the jump
///    - Same for v-periodic surfaces
/// 3. This maintains continuity along the boundary curve, even across seams
fn project_boundary_to_uv_chain(
    surface: &Surface,
    points: &[Point3d],
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Vec<Point2d> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut uvs = Vec::with_capacity(points.len());

    // First point: project normally
    let (u0, v0) = surface.project_point(&points[0]);
    uvs.push(Point2d::new(u0, v0));

    // Chain projection for subsequent points
    for i in 1..points.len() {
        let (mut u, v) = surface.project_point(&points[i]);

        // Adjust u for periodicity to maintain continuity with previous point
        if let Some(period) = u_period {
            let prev_u = uvs[i - 1].u;
            u = adjust_periodic_value(u, prev_u, period);
        }

        // Adjust v for periodicity (torus has v-period)
        let mut adjusted_v = v;
        if let Some(period) = v_period {
            let prev_v = uvs[i - 1].v;
            adjusted_v = adjust_periodic_value(v, prev_v, period);
        }

        uvs.push(Point2d::new(u, adjusted_v));
    }

    uvs
}

/// Adjust a periodic parameter value to be as close as possible to a reference,
/// by adding or subtracting the period.
///
/// For example, if `value = 0.1`, `reference = 6.2`, and `period = 2π ≈ 6.283`,
/// the adjusted value would be `0.1 + 6.283 ≈ 6.383` (closer to 6.2 than 0.1).
#[inline]
fn adjust_periodic_value(value: f64, reference: f64, period: f64) -> f64 {
    let diff = value - reference;
    if diff > period * 0.5 {
        value - period
    } else if diff < -period * 0.5 {
        value + period
    } else {
        value
    }
}

/// Detect whether a boundary wraps the full u-period of a periodic surface.
///
/// A full-period boundary means the boundary points span the entire angular range
/// (0 to 2π) on a revolution-type surface (cylinder, cone, sphere, torus).
/// When this happens, the UV polygon degenerates because u=0 and u=2π map to
/// the same geometric seam, and earcutr cannot produce a valid triangulation.
///
/// # Algorithm
/// Projects all boundary points to UV space and checks if the u-range (after
/// chain normalization) spans ≥ 90% of the period. This threshold catches both
/// exact full-wrap boundaries and near-full-wrap ones that still cause problems.
fn is_full_period_boundary(
    surface: &Surface,
    boundary_points: &[Point3d],
    u_period: Option<f64>,
) -> bool {
    let period = match u_period {
        Some(p) => p,
        None => return false, // Non-periodic surface — never full-wrap
    };

    if boundary_points.len() < 4 {
        return false;
    }

    // Project all points to UV and find the u-range using chain normalization
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut prev_u = 0.0f64;
    let mut first = true;

    for p in boundary_points {
        let (mut u, _v) = surface.project_point(p);
        if !first {
            u = adjust_periodic_value(u, prev_u, period);
        }
        first = false;
        prev_u = u;
        u_min = u_min.min(u);
        u_max = u_max.max(u);
    }

    let u_range = u_max - u_min;

    // If the u-range covers ≥90% of the period, this is a full-wrap boundary
    u_range >= period * 0.9
}

/// Triangulate a "cap" face on a curved surface — a disc-like face where the boundary
/// projects to a degenerate UV range (e.g., a circular disc on the end of a cylinder,
/// cone, or torus). The boundary points form a closed loop on the surface, and we
/// triangulate using a fan from the centroid of the boundary points, with each triangle's
/// third vertex evaluated on the surface.
fn triangulate_cap_face(
    surface: &Surface,
    boundary_points_3d: &[Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    if boundary_points_3d.len() < 3 {
        return mesh;
    }

    // Compute centroid of boundary points
    let n_pts = boundary_points_3d.len() as f64;
    let centroid = Point3d::new(
        boundary_points_3d.iter().map(|p| p.x).sum::<f64>() / n_pts,
        boundary_points_3d.iter().map(|p| p.y).sum::<f64>() / n_pts,
        boundary_points_3d.iter().map(|p| p.z).sum::<f64>() / n_pts,
    );

    // Project centroid onto the surface to get a more accurate center
    let (cu, cv) = surface.project_point(&centroid);
    let center_3d = surface.point_at(cu, cv);
    let center_normal = surface.normal_at(cu, cv);

    // Add centroid as first vertex
    let center_idx = mesh.add_vertex(center_3d);
    mesh.add_vertex_normal(center_idx, [center_normal.x, center_normal.y, center_normal.z]);

    // Add boundary vertices
    for p in boundary_points_3d {
        let (u, v) = surface.project_point(p);
        let n = surface.normal_at(u, v);
        let idx = mesh.add_vertex(*p);
        mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
    }

    // Fan triangulation: center → boundary[i] → boundary[(i+1) % n]
    let n = boundary_points_3d.len() as u32;
    for i in 0..n {
        let i_next = (i + 1) % n;
        let v1 = center_idx + 1 + i;
        let v2 = center_idx + 1 + i_next;
        if forward {
            mesh.add_triangle(center_idx, v1, v2);
        } else {
            mesh.add_triangle(center_idx, v2, v1);
        }
    }

    mesh
}

/// Triangulate a curved surface with boundary trimming in UV space.
///
/// For NURBS surfaces, delegates to `parametric_domain::triangulate_surface_consistent()`
/// which provides Newton-Raphson re-projection, UV normalization, area validation,
/// and knot-span subdivision for high-quality results.
///
/// For other curved surfaces, uses the grid-based algorithm:
/// 1. Project boundary points to UV space → UV polygon
/// 2. Normalize UV polygon for periodic surfaces
/// 3. Compute UV bounding box from the polygon
/// 4. Create a UV grid inside the bounding box
/// 5. For each grid cell, test if the center is inside the UV polygon
/// 6. Generate triangles only for inside cells
/// 7. Add boundary vertices and create boundary strip triangles
fn triangulate_surface_uv_trimmed(
    surface: &Surface,
    boundary_points_3d: &[Point3d],
    hole_polylines_3d: &[Vec<Point3d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    // For NURBS surfaces, use the consistent triangulation path which provides:
    // - Newton-Raphson re-projection for accurate UV coordinates
    // - UV normalization for periodic surfaces
    // - UV polygon quality validation (area ratio check)
    // - Knot-span subdivision for interior Steiner points
    // - earcutr-based triangulation with native hole support
    // This produces much better results than the grid-based approach below.
    if matches!(surface, Surface::Nurbs(_)) {
        if boundary_points_3d.len() < 3 {
            return TriangleMesh::new();
        }

        // Project boundary to UV space using Newton-Raphson chaining
        // (much faster than calling project_point() on every point)
        let boundary_uvs = project_points_nurbs_fast(surface, boundary_points_3d);

        // Check if UV projection is valid
        if boundary_uvs.iter().any(|uv| !uv.u.is_finite() || !uv.v.is_finite()) {
            log::warn!("NURBS UV trimmed fallback: UV projection NaN/Inf");
            return TriangleMesh::new();
        }

        // Project hole polylines to UV
        let holes_uvs: Vec<Vec<Point2d>> = hole_polylines_3d.iter().map(|hole| {
            project_points_nurbs_fast(surface, hole)
        }).collect();

        // Check hole UV validity
        if holes_uvs.iter().any(|h| h.iter().any(|uv| !uv.u.is_finite() || !uv.v.is_finite())) {
            log::warn!("NURBS UV trimmed fallback: hole UV NaN/Inf");
            return TriangleMesh::new();
        }

        let result = crate::parametric_domain::triangulate_surface_consistent(
            surface,
            boundary_points_3d,
            &boundary_uvs,
            hole_polylines_3d,
            &holes_uvs,
            forward,
            params,
        );

        return result;
    }

    let mut mesh = TriangleMesh::new();

    // 1. Project boundary to UV space
    let mut boundary_uv: Vec<Point2d> = boundary_points_3d.iter()
        .map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        })
        .collect();

    if boundary_uv.len() < 3 {
        return mesh;
    }

    // Also project hole polylines to UV
    let mut hole_uv_polylines: Vec<Vec<Point2d>> = hole_polylines_3d.iter().map(|hole| {
        hole.iter().map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        }).collect()
    }).collect();

    // 2. Normalize UV polygon for periodic surfaces
    let u_period = surface_u_period(surface);
    let v_period = surface_v_period(surface);
    normalize_uv_polygon(&mut boundary_uv, u_period, v_period);
    // Also normalize hole polygons
    for hole_uv in hole_uv_polylines.iter_mut() {
        normalize_uv_polygon(hole_uv, u_period, v_period);
    }

    // 3. Compute UV bounding box from BOTH outer and inner boundary points
    let mut u_min = f64::MAX; let mut u_max = f64::MIN;
    let mut v_min = f64::MAX; let mut v_max = f64::MIN;
    for p in &boundary_uv {
        u_min = u_min.min(p.u); u_max = u_max.max(p.u);
        v_min = v_min.min(p.v); v_max = v_max.max(p.v);
    }
    for hole_uv in &hole_uv_polylines {
        for p in hole_uv {
            u_min = u_min.min(p.u); u_max = u_max.max(p.u);
            v_min = v_min.min(p.v); v_max = v_max.max(p.v);
        }
    }

    let u_range = u_max - u_min;
    let v_range = v_max - v_min;

    // Handle degenerate UV range — this occurs when the boundary is a "cap" face
    // (a disc on a cylinder/cone/torus where all boundary points project to the
    // same v value). In this case, triangulate as a disc (fan from centroid).
    if u_range < 1e-12 && v_range < 1e-12 {
        // Both ranges degenerate — shouldn't happen
        return mesh;
    }

    // Check for cap faces: if the outer boundary projects to a degenerate UV range
    // (constant v for cylinders/cones, or constant u/v for torus), but holes provide
    // the missing range, we still need to treat this as a cap + annular ring.
    //
    // The key insight: if the outer boundary alone has a degenerate v_range (or u_range),
    // but with holes the full range becomes non-degenerate, the face is an annular band
    // (like the side of a tube between two concentric circles). We should handle this
    // with the UV grid approach.
    //
    // But if the outer boundary has a degenerate range AND holes don't help either
    // (holes are at the same v value), it's truly a flat cap disc.
    {
        let mut outer_u_min = f64::MAX; let mut outer_u_max = f64::MIN;
        let mut outer_v_min = f64::MAX; let mut outer_v_max = f64::MIN;
        for p in &boundary_uv {
            outer_u_min = outer_u_min.min(p.u); outer_u_max = outer_u_max.max(p.u);
            outer_v_min = outer_v_min.min(p.v); outer_v_max = outer_v_max.max(p.v);
        }
        let outer_u_range = outer_u_max - outer_u_min;
        let outer_v_range = outer_v_max - outer_v_min;

        if outer_u_range < 1e-8 && outer_v_range < 1e-8 {
            // Both ranges degenerate — completely degenerate face
            return mesh;
        }

        // If only one UV direction is degenerate, check if this is a truly flat
        // cap face (disc on a plane) or a curved band (cylinder/cone side face).
        // A single closed curve on a cylinder/cone that projects to constant v is
        // NOT a flat cap — it's a circular band that wraps around the surface.
        // Only treat it as a cap face if the surface is flat (a plane) or if
        // the boundary 3D points are nearly coplanar.
        if outer_u_range < 1e-8 || outer_v_range < 1e-8 {
            // Check if the 3D boundary points are approximately coplanar
            // (meaning this is a genuine flat cap disc, not a curved band)
            let is_coplanar = is_nearly_coplanar(boundary_points_3d, 1e-4);
            if is_coplanar {
                return triangulate_cap_face(surface, boundary_points_3d, forward);
            }
            // Otherwise, it's a curved band (e.g., cylinder/cone side face
            // where the outer boundary is a single circle projecting to constant v).
            // Fall through to the UV grid approach — we need to expand the UV range
            // to cover the surface region properly.
            //
            // For a cylinder/cone band: if v_range is degenerate, expand it slightly
            // using the 3D bounding box and the surface geometry.
            if outer_v_range < 1e-8 {
                // For v-periodic surfaces, a degenerate v range means the boundary
                // doesn't constrain v — use the full v period
                if let Some(period) = v_period {
                    v_min = 0.0;
                    v_max = period;
                } else {
                    // Compute a reasonable v range from the 3D bounding box
                    let mut z_min = f64::MAX; let mut z_max = f64::MIN;
                    for p in boundary_points_3d {
                        // Project onto the surface's v direction
                        let (_, v) = surface.project_point(p);
                        z_min = z_min.min(v);
                        z_max = z_max.max(v);
                    }
                    if z_max - z_min > 1e-8 {
                        v_min = z_min; v_max = z_max;
                    } else {
                        // Truly degenerate — give it a small range
                        v_min -= 0.5; v_max += 0.5;
                    }
                }
            }
            if outer_u_range < 1e-8 {
                // For periodic surfaces, a degenerate u range means the boundary
                // doesn't constrain u — use the full u period
                if let Some(period) = u_period {
                    u_min = 0.0;
                    u_max = period;
                } else {
                    let mut u_min_tmp = f64::MAX; let mut u_max_tmp = f64::MIN;
                    for p in boundary_points_3d {
                        let (u, _) = surface.project_point(p);
                        u_min_tmp = u_min_tmp.min(u);
                        u_max_tmp = u_max_tmp.max(u);
                    }
                    if u_max_tmp - u_min_tmp > 1e-8 {
                        u_min = u_min_tmp; u_max = u_max_tmp;
                    } else {
                        u_min -= 0.5; u_max += 0.5;
                    }
                }
            }
        }
    }

    // Add small margin
    // Recompute ranges (may have been modified by cap face detection)
    let u_range = u_max - u_min;
    let v_range = v_max - v_min;
    let margin_u = u_range * 0.001;
    let margin_v = v_range * 0.001;
    u_min -= margin_u; u_max += margin_u;
    v_min -= margin_v; v_max += margin_v;

    // 4. Determine grid resolution
    // For partial angular ranges, scale down n_u proportionally
    let full_circle = u_range > 1.9 * PI;
    let n_u = if full_circle {
        params.angular_samples.max(16)
    } else {
        ((params.angular_samples as f64 * u_range / (2.0 * PI)).ceil() as usize).max(8).min(params.angular_samples)
    };
    // For small v ranges, use fewer height samples
    let n_v = if v_range < 1.0 {
        ((params.height_samples as f64 * v_range).ceil() as usize).max(2)
    } else {
        params.height_samples.max(4)
    };
    let du = (u_max - u_min) / n_u as f64;
    let dv = (v_max - v_min) / n_v as f64;

    // 5. For each grid cell, check if center is inside outer polygon AND NOT inside any hole
    let mut inside = vec![vec![false; n_u]; n_v];
    let mut inside_count = 0usize;
    for j in 0..n_v {
        for i in 0..n_u {
            let cu = u_min + du * (i as f64 + 0.5);
            let cv = v_min + dv * (j as f64 + 0.5);
            let pt = Point2d::new(cu, cv);
            let in_outer = point_in_polygon_2d(&pt, &boundary_uv);
            if !in_outer {
                inside[j][i] = false;
                continue;
            }
            // Check if inside any hole — if so, exclude
            let mut in_hole = false;
            for hole_uv in &hole_uv_polylines {
                if hole_uv.len() >= 3 && point_in_polygon_2d(&pt, hole_uv) {
                    in_hole = true;
                    break;
                }
            }
            inside[j][i] = !in_hole;
            if inside[j][i] { inside_count += 1; }
        }
    }

    if inside_count == 0 {
        // No cells inside the polygon — this likely means the boundary projects
        // to a degenerate shape (a line) in UV space, or the UV polygon
        // representation doesn't match the grid (e.g., wrap-around issues).
        //
        // For periodic surfaces, the UV polygon might not correctly represent
        // the face when it wraps around. In this case, fall back to the
        // surface-type-specific triangulation which handles periodicity better.
        if u_period.is_some() || v_period.is_some() {
            // Use the Face-based triangulation path which handles periodicity
            let wire = Wire::new(vec![]);
            let mut face = Face::new(surface.clone(), wire);
            face.forward = forward;
            face.edges = vec![];
            return triangulate_face(&face, params);
        }
        return triangulate_cap_face(surface, boundary_points_3d, forward);
    }

    // 6. Generate grid vertices (only for cells that are inside or adjacent to inside)
    // Vertex grid is (n_u+1) x (n_v+1)
    let mut vertex_index = vec![vec![None::<u32>; n_u + 1]; n_v + 1];

    for j in 0..=n_v {
        for i in 0..=n_u {
            // Check if this vertex is needed (adjacent to any inside cell)
            let mut needed = false;
            for dj in 0..2usize {
                for di in 0..2usize {
                    let ci = if di == 0 { i.checked_sub(1) } else { if i < n_u { Some(i) } else { None } };
                    let cj = if dj == 0 { j.checked_sub(1) } else { if j < n_v { Some(j) } else { None } };
                    if let (Some(ci), Some(cj)) = (ci, cj) {
                        if inside[cj][ci] { needed = true; }
                    }
                }
            }

            if needed {
                let u = u_min + du * i as f64;
                let v = v_min + dv * j as f64;
                let p3d = surface.point_at(u, v);
                let normal = surface.normal_at(u, v);
                let idx = mesh.add_vertex(p3d);
                mesh.add_vertex_normal(idx, [normal.x, normal.y, normal.z]);
                vertex_index[j][i] = Some(idx);
            }
        }
    }

    // 7. Generate triangles for inside cells
    for j in 0..n_v {
        for i in 0..n_u {
            if !inside[j][i] { continue; }

            let v00 = vertex_index[j][i];
            let v10 = vertex_index[j][i + 1];
            let v01 = vertex_index[j + 1][i];
            let v11 = vertex_index[j + 1][i + 1];

            // Need at least 3 vertices to form a triangle
            match (v00, v10, v01, v11) {
                (Some(i00), Some(i10), Some(i01), Some(i11)) => {
                    // Full quad
                    if forward {
                        mesh.add_triangle(i00, i10, i11);
                        mesh.add_triangle(i00, i11, i01);
                    } else {
                        mesh.add_triangle(i00, i11, i10);
                        mesh.add_triangle(i00, i01, i11);
                    }
                }
                (Some(i00), Some(i10), Some(i01), None) => {
                    if forward {
                        mesh.add_triangle(i00, i10, i01);
                    } else {
                        mesh.add_triangle(i00, i01, i10);
                    }
                }
                (Some(i00), Some(i10), None, Some(i11)) => {
                    if forward {
                        mesh.add_triangle(i00, i10, i11);
                    } else {
                        mesh.add_triangle(i00, i11, i10);
                    }
                }
                (Some(i00), None, Some(i01), Some(i11)) => {
                    if forward {
                        mesh.add_triangle(i00, i01, i11);
                    } else {
                        mesh.add_triangle(i00, i11, i01);
                    }
                }
                (None, Some(i10), Some(i01), Some(i11)) => {
                    if forward {
                        mesh.add_triangle(i10, i11, i01);
                    } else {
                        mesh.add_triangle(i10, i01, i11);
                    }
                }
                _ => {
                    // Fewer than 3 vertices available — skip this cell
                }
            }
        }
    }

    // 8. Add boundary vertices and create boundary strip triangles
    // For NURBS surfaces, skip the expensive boundary strip — the UV grid
    // already provides adequate coverage, and project_point is very slow for NURBS.
    // Boundary strip is mainly needed for cylinder/cone/torus where the boundary
    // constrains the face tightly.
    let is_nurbs = matches!(surface, Surface::Nurbs(_));
    if is_nurbs {
        return mesh;
    }

    let n_boundary = boundary_points_3d.len();
    if n_boundary < 3 {
        return mesh;
    }

    // Reuse the already-computed boundary_uv from step 1 instead of re-projecting.
    // This avoids the expensive project_point call (especially for NURBS/Revolution/Extrusion).
    let boundary_uv_ref = &boundary_uv;

    let boundary_start = mesh.vertices.len() as u32;
    for (idx_3d, p) in boundary_points_3d.iter().enumerate() {
        // Use the pre-computed boundary UV (already normalized)
        let (nu, nv) = if idx_3d < boundary_uv_ref.len() {
            (boundary_uv_ref[idx_3d].u, boundary_uv_ref[idx_3d].v)
        } else {
            // Fallback: project the point (shouldn't normally happen)
            let (u, v) = surface.project_point(p);
            (u, v)
        };

        let (u, v) = if idx_3d < boundary_uv_ref.len() {
            (boundary_uv_ref[idx_3d].u, boundary_uv_ref[idx_3d].v)
        } else {
            surface.project_point(p)
        };

        let normal = surface.normal_at(u, v);
        let vidx = mesh.add_vertex(*p);
        mesh.add_vertex_normal(vidx, [normal.x, normal.y, normal.z]);

        // Find nearest grid vertex that is inside the polygon
        let gi_f = ((nu - u_min) / du).round() as isize;
        let gj_f = ((nv - v_min) / dv).round() as isize;

        // Search in expanding radius for an inside grid vertex
        let mut best_grid: Option<u32> = None;
        let mut best_dist = f64::MAX;
        let search_radius = 3isize;
        for dj in -search_radius..=search_radius {
            for di in -search_radius..=search_radius {
                let gi = gi_f + di;
                let gj = gj_f + dj;
                if gi < 0 || gj < 0 { continue; }
                let gi = gi as usize;
                let gj = gj as usize;
                if gi > n_u || gj > n_v { continue; }
                if let Some(vidx_grid) = vertex_index[gj][gi] {
                    let gp = &mesh.vertices[vidx_grid as usize];
                    let dx = gp.x - p.x;
                    let dy = gp.y - p.y;
                    let dz = gp.z - p.z;
                    let dist = dx * dx + dy * dy + dz * dz;
                    if dist < best_dist {
                        best_dist = dist;
                        best_grid = Some(vidx_grid);
                    }
                }
            }
        }

        // Create triangle: boundary[idx] → boundary[idx+1] → nearest_grid
        if let Some(grid_idx) = best_grid {
            let next_boundary_idx = boundary_start + ((idx_3d as u32 + 1) % n_boundary as u32);
            let cur_boundary_idx = boundary_start + idx_3d as u32;
            if forward {
                mesh.add_triangle(cur_boundary_idx, next_boundary_idx, grid_idx);
            } else {
                mesh.add_triangle(cur_boundary_idx, grid_idx, next_boundary_idx);
            }
        }
    }

    mesh
}

/// Triangulate a plane face with boundary points — minimum triangles.
fn triangulate_plane_with_boundary(
    plane: &Plane,
    boundary_points: &[Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_points.len() < 3 {
        return mesh;
    }

    // Deduplicate boundary points — close points create degenerate triangles
    let deduped_points = deduplicate_points_3d(boundary_points, 1e-6);
    if deduped_points.len() < 3 {
        return mesh;
    }

    let points_2d: Vec<Point2d> = deduped_points.iter().map(|p| {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    }).collect();

    let is_convex = is_convex_polygon(&points_2d);

    if is_convex && deduped_points.len() >= 3 {
        // Fan triangulation — N-2 triangles (minimum for convex polygon)
        for p in &deduped_points {
            mesh.add_vertex(*p);
        }
        let n = deduped_points.len() as u32;
        for i in 1..n - 1 {
            if forward {
                mesh.add_triangle(0, i, i + 1);
            } else {
                mesh.add_triangle(0, i + 1, i);
            }
        }
    } else {
        let triangles = ear_clip(&points_2d);
        for p in &deduped_points {
            mesh.add_vertex(*p);
        }
        for tri in &triangles {
            if forward {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            } else {
                mesh.add_triangle(tri[0], tri[2], tri[1]);
            }
        }
    }

    let normal = if forward {
        plane.normal
    } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };
    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);
    mesh
}

/// Triangulate a planar face with boundary and holes using earcutr.
///
/// Uses the mapbox/earcut algorithm which natively handles holes
/// without bridge-edge tricks. This produces correct triangulations
/// for circular bolt holes and other complex hole shapes.
fn triangulate_plane_with_boundary_and_holes(
    plane: &Plane,
    boundary_points: &[Point3d],
    hole_polylines: &[Vec<Point3d>],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_points.len() < 3 {
        return mesh;
    }

    // Deduplicate boundary points
    let deduped_outer = deduplicate_points_3d(boundary_points, 1e-6);
    if deduped_outer.len() < 3 {
        return mesh;
    }

    // Project to 2D
    let project = |p: &Point3d| -> Point2d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    };

    let outer_2d: Vec<Point2d> = deduped_outer.iter().map(|p| project(p)).collect();

    // Deduplicate and project hole polylines
    let mut holes_2d: Vec<Vec<Point2d>> = Vec::new();
    let mut holes_3d: Vec<Vec<Point3d>> = Vec::new();
    for hole_pts in hole_polylines {
        let deduped = deduplicate_points_3d(hole_pts, 1e-6);
        if deduped.len() < 3 { continue; }
        let h2d: Vec<Point2d> = deduped.iter().map(|p| project(p)).collect();
        holes_2d.push(h2d);
        holes_3d.push(deduped);
    }

    // Try earcutr first
    if let Some(m) = earcutr_triangulate_planar(&outer_2d, &deduped_outer, &holes_2d, &holes_3d, forward, plane.normal) {
        return m;
    }

    // Fallback to bridge-edge + ear-clip
    log::warn!("earcutr failed for planar face with holes, falling back to bridge-edge ear-clip");
    let (merged_2d, merged_3d) = merge_holes_into_polygon_planar(
        &outer_2d, &deduped_outer, &holes_2d, &holes_3d,
    );
    let triangles = ear_clip(&merged_2d);
    let filtered_triangles: Vec<[u32; 3]> = triangles.iter()
        .filter(|tri| {
            let a = merged_2d[tri[0] as usize];
            let b = merged_2d[tri[1] as usize];
            let c = merged_2d[tri[2] as usize];
            let centroid_u = (a.u + b.u + c.u) / 3.0;
            let centroid_v = (a.v + b.v + c.v) / 3.0;
            for hole in &holes_2d {
                if point_in_polygon_check(&Point2d::new(centroid_u, centroid_v), hole) {
                    return false;
                }
            }
            point_in_polygon_check(&Point2d::new(centroid_u, centroid_v), &outer_2d)
        })
        .cloned()
        .collect();
    for p in &merged_3d {
        mesh.add_vertex(*p);
    }
    for tri in &filtered_triangles {
        if forward {
            mesh.add_triangle(tri[0], tri[1], tri[2]);
        } else {
            mesh.add_triangle(tri[0], tri[2], tri[1]);
        }
    }

    let normal = if forward {
        plane.normal
    } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };
    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);
    mesh
}

/// Triangulate a cylinder face trimmed by boundary points.
fn triangulate_cylinder_with_boundary(
    cyl: &CylinderSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = cylinder_uv_range(cyl, boundary_points);

    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);

    let u_range = u_max - u_min;
    let full_circle = u_range > 1.9 * PI;
    let u_start = if full_circle { 0.0 } else { u_min };
    let u_end = if full_circle { 2.0 * PI } else { u_max };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = cyl.point_at(u, v);
            let n = cyl.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    let n_u_loop = if full_circle { n_u } else { n_u - 1 };
    for j in 0..n_v - 1 {
        for i in 0..n_u_loop {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a cone face trimmed by boundary points.
fn triangulate_cone_with_boundary(
    cone: &ConeSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = cone_uv_range(cone, boundary_points);

    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);

    let u_range = u_max - u_min;
    let full_circle = u_range > 1.9 * PI;
    let u_start = if full_circle { 0.0 } else { u_min };
    let u_end = if full_circle { 2.0 * PI } else { u_max };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = cone.point_at(u, v);
            let n = cone.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    let n_u_loop = if full_circle { n_u } else { n_u - 1 };
    for j in 0..n_v - 1 {
        for i in 0..n_u_loop {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a sphere face trimmed by boundary points.
fn triangulate_sphere_with_boundary(
    sphere: &SphereSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = sphere_uv_range(sphere, boundary_points);

    let n_u = params.angular_samples;
    let n_v = (params.angular_samples / 2).max(4);

    let u_range = u_max - u_min;
    let v_range = v_max - v_min;
    let full_u = u_range > 1.9 * PI;
    let full_v = v_range > 0.9 * PI;

    let u_start = if full_u { 0.0 } else { u_min };
    let u_end = if full_u { 2.0 * PI } else { u_max };
    let v_start = if full_v { 0.0 } else { v_min };
    let v_end = if full_v { PI } else { v_max };

    for j in 0..=n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_start + (v_end - v_start) * j as f64 / n_v as f64;
            let p = sphere.point_at(u, v);
            let n = sphere.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a cone face with boundary points and optional holes.
/// Handles apex degeneracy: when v_max reaches the apex height, all
/// vertices in the top row collapse to a single point. We merge them
/// into a single apex vertex to avoid degenerate (zero-area) triangles.
/// Also handles cones with near-zero height (very small half_angle) by
/// treating them as degenerate cylinders when appropriate.
fn triangulate_cone_face_with_boundary(
    cone: &ConeSurface,
    boundary_points: &[Point3d],
    hole_polylines: &[Vec<Point3d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_points.len() < 3 {
        return mesh;
    }

    let apex_v = cone.height();

    // Handle degenerate cone: infinite or zero height (non-expanding).
    // Expanding cones always have infinite height — that's normal for them.
    if !cone.expanding && (!apex_v.is_finite() || apex_v > 1e6) {
        // Near-cylinder: half_angle is very small, cone is essentially a cylinder.
        // Use the generic UV-trimmed path which handles cylinders.
        return triangulate_surface_uv_trimmed(
            &Surface::Cone(cone.clone()),
            boundary_points, hole_polylines, forward, params,
        );
    }

    // Project boundary to UV space
    let mut boundary_uv: Vec<Point2d> = boundary_points.iter()
        .map(|p| {
            let (u, v) = cone.project_point(p);
            Point2d::new(u, v)
        })
        .collect();

    // Normalize UV polygon for u-periodicity (2π)
    normalize_uv_polygon(&mut boundary_uv, Some(2.0 * PI), None);

    // Compute UV bounding box
    let mut u_min = f64::MAX; let mut u_max = f64::MIN;
    let mut v_min = f64::MAX; let mut v_max = f64::MIN;
    for p in &boundary_uv {
        u_min = u_min.min(p.u); u_max = u_max.max(p.u);
        v_min = v_min.min(p.v); v_max = v_max.max(p.v);
    }
    for hole in hole_polylines {
        for p in hole {
            let (u, v) = cone.project_point(p);
            u_min = u_min.min(u); u_max = u_max.max(u);
            v_min = v_min.min(v); v_max = v_max.max(v);
        }
    }

    let u_range = u_max - u_min;
    let v_range = v_max - v_min;

    if u_range < 1e-10 && v_range < 1e-10 {
        return mesh;
    }

    // Handle degenerate v range — if v range is near-zero, the cone face
    // is essentially a flat disc. Triangulate as a cap face.
    if v_range < apex_v * 0.001 + 1e-6 && !cone.expanding {
        return triangulate_cap_face(&Surface::Cone(cone.clone()), boundary_points, forward);
    }

    // Handle degenerate u range (boundary doesn't constrain u → full circle)
    let full_circle = u_range < 0.5 * PI || u_range > 1.9 * PI;
    if full_circle {
        u_min = 0.0;
        u_max = 2.0 * PI;
    }

    // Clamp v_max to apex height (only for non-expanding cones)
    if !cone.expanding {
        v_max = v_max.min(apex_v);
    }
    let top_at_apex = !cone.expanding && apex_v.is_finite() && (v_max - apex_v).abs() < apex_v * 0.05 + 1e-6;

    // Add small margin
    let margin_u = (u_max - u_min) * 0.001;
    let margin_v = (v_max - v_min) * 0.001;
    u_min -= margin_u; u_max += margin_u;
    v_min -= margin_v; v_max += margin_v;

    // Clamp v range (only lower bound for expanding cones)
    v_min = v_min.max(0.0);
    if !cone.expanding {
        v_max = v_max.min(apex_v);
    }

    // Grid resolution
    let n_u = if full_circle { params.angular_samples } else {
        ((params.angular_samples as f64 * (u_max - u_min) / (2.0 * PI)).ceil() as usize).max(8).min(params.angular_samples)
    };
    let n_v = params.height_samples.max(2);

    let du = (u_max - u_min) / n_u as f64;
    let dv = (v_max - v_min) / n_v as f64;

    // Generate vertex grid with apex degeneracy handling
    // At the apex (row n_v), all vertices collapse to a single point.
    // We generate only 1 apex vertex instead of n_u to avoid degenerate triangles.
    let mut apex_vertex: Option<u32> = None;
    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + dv * j as f64;

        if top_at_apex && j == n_v {
            // Apex row — single vertex
            let p = cone.point_at(0.0, apex_v);
            let n = cone.normal_at(0.0, apex_v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            apex_vertex = Some(idx);
            row_vertex_offset.push(idx);
            row_vertex_count.push(1);
            total_vertices += 1;
        } else {
            // Normal ring row
            let base = total_vertices;
            row_vertex_offset.push(base);
            row_vertex_count.push(n_u);
            for i in 0..n_u {
                let u = u_min + du * i as f64;
                let p = cone.point_at(u, v);
                let n = cone.normal_at(u, v);
                let idx = mesh.add_vertex(p);
                mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            }
            total_vertices += n_u as u32;
        }
    }

    // Generate triangles
    for j in 0..n_v {
        let j_next = j + 1;
        let row_count = row_vertex_count[j];
        let next_row_count = row_vertex_count[j_next];
        let row_base = row_vertex_offset[j];
        let next_row_base = row_vertex_offset[j_next];

        if row_count == 1 {
            // Current row is apex — fan from apex to next row ring
            // (shouldn't normally happen as apex is the last row)
            let apex = row_base;
            for i in 0..next_row_count {
                let i_next = if full_circle { (i + 1) % next_row_count } else { (i + 1).min(next_row_count - 1) };
                let v1 = next_row_base + i as u32;
                let v2 = next_row_base + i_next as u32;
                if v1 != v2 {
                    if forward {
                        mesh.add_triangle(apex, v1, v2);
                    } else {
                        mesh.add_triangle(apex, v2, v1);
                    }
                }
            }
        } else if next_row_count == 1 {
            // Next row is apex — fan from current row ring to apex
            let apex = next_row_base;
            for i in 0..row_count {
                let i_next = if full_circle { (i + 1) % row_count } else { (i + 1).min(row_count - 1) };
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                if v0 != v1 {
                    if forward {
                        mesh.add_triangle(v0, v1, apex);
                    } else {
                        mesh.add_triangle(v1, v0, apex);
                    }
                }
            }
        } else {
            // Normal quad strip between two ring rows
            let loop_count = if full_circle { row_count } else { row_count - 1 };
            for i in 0..loop_count {
                let i_next = if full_circle { (i + 1) % row_count } else { i + 1 };
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                let v2 = next_row_base + i_next as u32;
                let v3 = next_row_base + i as u32;
                if forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }
    }

    mesh
}

/// Triangulate a sphere face with boundary points and optional holes.
/// Handles pole degeneracy: when v_start = 0 (north pole) or v_end = π (south pole),
/// all vertices in that row collapse to a single point. We merge them
/// into a single pole vertex to avoid degenerate (zero-area) triangles.
fn triangulate_sphere_face_with_boundary(
    sphere: &SphereSurface,
    boundary_points: &[Point3d],
    hole_polylines: &[Vec<Point3d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_points.len() < 3 {
        return mesh;
    }

    // Project boundary to UV space
    let mut boundary_uv: Vec<Point2d> = boundary_points.iter()
        .map(|p| {
            let (u, v) = sphere.project_point(p);
            Point2d::new(u, v)
        })
        .collect();

    // Normalize UV polygon for u-periodicity (2π)
    normalize_uv_polygon(&mut boundary_uv, Some(2.0 * PI), None);

    // Compute UV bounding box
    let mut u_min = f64::MAX; let mut u_max = f64::MIN;
    let mut v_min = f64::MAX; let mut v_max = f64::MIN;
    for p in &boundary_uv {
        u_min = u_min.min(p.u); u_max = u_max.max(p.u);
        v_min = v_min.min(p.v); v_max = v_max.max(p.v);
    }
    for hole in hole_polylines {
        for p in hole {
            let (u, v) = sphere.project_point(p);
            u_min = u_min.min(u); u_max = u_max.max(u);
            v_min = v_min.min(v); v_max = v_max.max(v);
        }
    }

    let u_range = u_max - u_min;
    let v_range = v_max - v_min;

    if u_range < 1e-10 && v_range < 1e-10 {
        return mesh;
    }

    // Handle degenerate u range (boundary doesn't constrain u → full circle)
    let full_u = u_range < 0.5 * PI || u_range > 1.9 * PI;
    if full_u {
        u_min = 0.0;
        u_max = 2.0 * PI;
    }

    // Handle v range: if boundary doesn't constrain v, use full [0, π]
    let full_v = v_range < 0.2 || v_range > 0.9 * PI;
    if full_v {
        v_min = 0.0;
        v_max = PI;
    }

    // Check for pole degeneracy
    let at_north_pole = v_min.abs() < 0.05; // v ≈ 0 → north pole
    let at_south_pole = (v_max - PI).abs() < 0.05; // v ≈ π → south pole

    // Add small margin
    let margin_u = (u_max - u_min) * 0.001;
    let margin_v = (v_max - v_min) * 0.001;
    u_min -= margin_u; u_max += margin_u;
    v_min -= margin_v; v_max += margin_v;

    // Clamp v to valid range
    v_min = v_min.max(0.0);
    v_max = v_max.min(PI);

    // Grid resolution
    let full_u = (u_max - u_min) > 1.9 * PI;
    let n_u = if full_u { params.angular_samples } else {
        ((params.angular_samples as f64 * (u_max - u_min) / (2.0 * PI)).ceil() as usize).max(8).min(params.angular_samples)
    };
    let n_v = (params.angular_samples / 2).max(4);

    let du = (u_max - u_min) / n_u as f64;
    let dv = (v_max - v_min) / n_v as f64;

    // Generate vertex grid with pole degeneracy handling
    // At poles, all vertices collapse to a single point.
    // We generate only 1 pole vertex instead of n_u to avoid degenerate triangles.
    let mut pole_vertex_north: Option<u32> = None;
    let mut pole_vertex_south: Option<u32> = None;
    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + dv * j as f64;

        if j == 0 && at_north_pole {
            // North pole — single vertex
            let p = sphere.point_at(0.0, 0.0);
            let n = sphere.normal_at(0.0, 0.0);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            pole_vertex_north = Some(idx);
            row_vertex_offset.push(idx);
            row_vertex_count.push(1);
            total_vertices += 1;
        } else if j == n_v && at_south_pole {
            // South pole — single vertex
            let p = sphere.point_at(0.0, PI);
            let n = sphere.normal_at(0.0, PI);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            pole_vertex_south = Some(idx);
            row_vertex_offset.push(idx);
            row_vertex_count.push(1);
            total_vertices += 1;
        } else {
            // Normal ring row
            let base = total_vertices;
            row_vertex_offset.push(base);
            row_vertex_count.push(n_u);
            for i in 0..n_u {
                let u = u_min + du * i as f64;
                let p = sphere.point_at(u, v);
                let n = sphere.normal_at(u, v);
                let idx = mesh.add_vertex(p);
                mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            }
            total_vertices += n_u as u32;
        }
    }

    // Generate triangles
    for j in 0..n_v {
        let j_next = j + 1;
        let row_count = row_vertex_count[j];
        let next_row_count = row_vertex_count[j_next];
        let row_base = row_vertex_offset[j];
        let next_row_base = row_vertex_offset[j_next];

        if row_count == 1 && next_row_count > 1 {
            // North pole fan: pole → ring[i] → ring[i_next]
            let pole = row_base;
            for i in 0..next_row_count {
                let i_next = if full_u { (i + 1) % next_row_count } else { (i + 1).min(next_row_count - 1) };
                let v1 = next_row_base + i as u32;
                let v2 = next_row_base + i_next as u32;
                if v1 != v2 {
                    if forward {
                        mesh.add_triangle(pole, v1, v2);
                    } else {
                        mesh.add_triangle(pole, v2, v1);
                    }
                }
            }
        } else if row_count > 1 && next_row_count == 1 {
            // South pole fan: ring[i] → ring[i_next] → pole
            let pole = next_row_base;
            for i in 0..row_count {
                let i_next = if full_u { (i + 1) % row_count } else { (i + 1).min(row_count - 1) };
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                if v0 != v1 {
                    if forward {
                        mesh.add_triangle(v0, v1, pole);
                    } else {
                        mesh.add_triangle(v1, v0, pole);
                    }
                }
            }
        } else if row_count > 1 && next_row_count > 1 {
            // Normal quad strip between two ring rows
            let loop_count = if full_u { row_count } else { row_count - 1 };
            for i in 0..loop_count {
                let i_next = if full_u { (i + 1) % row_count } else { i + 1 };
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                let v2 = next_row_base + i_next as u32;
                let v3 = next_row_base + i as u32;
                if forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }
    }

    mesh
}

/// Triangulate a torus face trimmed by boundary points.
fn triangulate_torus_with_boundary(
    torus: &TorusSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = torus_uv_range(torus, boundary_points);

    let n_u = params.angular_samples;
    let n_v = params.angular_samples;

    let u_range = u_max - u_min;
    let v_range = v_max - v_min;
    let full_u = u_range > 1.9 * PI;
    let full_v = v_range > 1.9 * PI;

    let u_start = if full_u { 0.0 } else { u_min };
    let u_end = if full_u { 2.0 * PI } else { u_max };
    let v_start = if full_v { 0.0 } else { v_min };
    let v_end = if full_v { 2.0 * PI } else { v_max };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_start + (v_end - v_start) * j as f64 / n_v as f64;
            let p = torus.point_at(u, v);
            let n = torus.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    let u_periodic = full_u;
    let v_periodic = full_v;

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = if u_periodic { (i + 1) % n_u } else { (i + 1).min(n_u - 1) };
            let j_next = if v_periodic { (j + 1) % n_v } else { (j + 1).min(n_v - 1) };

            if (!u_periodic && i == n_u - 1) || (!v_periodic && j == n_v - 1) {
                continue;
            }

            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = (j_next * n_u + i_next) as u32;
            let v3 = (j_next * n_u + i) as u32;

            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Generic surface triangulation with boundary points.
fn triangulate_generic_with_boundary(
    surface: &Surface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let (base_u_min, base_u_max, base_v_min, base_v_max) = if let Surface::Nurbs(nurbs) = surface {
        let (u0, u1) = nurbs.u_range();
        let (v0, v1) = nurbs.v_range();
        (u0, u1, v0, v1)
    } else {
        (0.0, 2.0 * PI, 0.0, PI)
    };

    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary_points {
        let (u, v) = surface.project_point(p);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    u_min = u_min.max(base_u_min);
    u_max = u_max.min(base_u_max);
    v_min = v_min.max(base_v_min);
    v_max = v_max.min(base_v_max);

    if u_min >= u_max || v_min >= v_max {
        u_min = base_u_min;
        u_max = base_u_max;
        v_min = base_v_min;
        v_max = base_v_max;
    }

    let u_margin = (u_max - u_min) * 0.01;
    let v_margin = (v_max - v_min) * 0.01;
    u_min = (u_min - u_margin).max(base_u_min);
    u_max = (u_max + u_margin).min(base_u_max);
    v_min = (v_min - v_margin).max(base_v_min);
    v_max = (v_max + v_margin).min(base_v_max);

    let n_u = if let Surface::Nurbs(_) = surface { params.angular_samples.max(24) } else { params.angular_samples };
    let n_v = if let Surface::Nurbs(_) = surface { params.angular_samples.max(24) } else { params.angular_samples };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = surface.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u - 1 {
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i + 1) as u32;
            let v2 = ((j + 1) * n_u + i + 1) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

// ============================================================
// UV range computation for boundary-aware trimming
// ============================================================

/// Compute parametric (u, v) range from boundary points for a cylinder.
fn cylinder_uv_range(cyl: &CylinderSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = cyl.project_point(p);
        angles.push(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    let (u_min, u_max) = compute_angular_range(&angles);
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Compute parametric (u, v) range from boundary points for a cone.
fn cone_uv_range(cone: &ConeSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = cone.project_point(p);
        angles.push(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    let (u_min, u_max) = compute_angular_range(&angles);
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Compute parametric (u, v) range from boundary points for a sphere.
fn sphere_uv_range(sphere: &SphereSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = sphere.project_point(p);
        angles.push(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    let (u_min, u_max) = compute_angular_range(&angles);
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Compute parametric (u, v) range from boundary points for a torus.
fn torus_uv_range(torus: &TorusSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut u_angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    let mut v_angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = torus.project_point(p);
        u_angles.push(u);
        v_angles.push(v);
    }
    let (u_min, u_max) = compute_angular_range(&u_angles);
    let (v_min, v_max) = compute_angular_range(&v_angles);
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Compute the angular range from a list of angles, handling ±π wraparound.
fn compute_angular_range(angles: &[f64]) -> (f64, f64) {
    if angles.is_empty() {
        return (0.0, 2.0 * PI);
    }
    if angles.len() == 1 {
        return (angles[0], angles[0] + 2.0 * PI);
    }

    let mut normalized: Vec<f64> = angles.iter()
        .map(|a| ((a % (2.0 * PI)) + 2.0 * PI) % (2.0 * PI))
        .collect();
    normalized.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let n = normalized.len();
    let mut max_gap = 0.0f64;
    let mut gap_end_idx = 0;
    for i in 0..n {
        let next = if i + 1 < n { normalized[i + 1] } else { normalized[0] + 2.0 * PI };
        let gap = next - normalized[i];
        if gap > max_gap {
            max_gap = gap;
            gap_end_idx = i + 1;
        }
    }

    let start_angle = normalized[gap_end_idx % n];
    let end_angle = if gap_end_idx == 0 {
        normalized[n - 1]
    } else if gap_end_idx < n {
        normalized[gap_end_idx - 1]
    } else {
        normalized[n - 1]
    };

    let range = if gap_end_idx == 0 {
        end_angle - start_angle + 2.0 * PI
    } else {
        end_angle - start_angle
    };

    if range > 1.99 * PI {
        (0.0, 2.0 * PI)
    } else {
        (start_angle, start_angle + range)
    }
}

// ============================================================
// V-range estimation for axis-based surfaces
// ============================================================

/// Estimate the v parameter range for a face by sampling its edges
/// and projecting sample points onto the surface's axis direction.
fn estimate_v_range(face: &Face) -> Option<(f64, f64)> {
    if let Some(ref surface) = face.surface {
        match surface {
            Surface::Cylinder(cyl) => {
                let (v_min, v_max) = compute_axis_v_range(face, &cyl.origin, &cyl.axis);
                if v_min < v_max { Some((v_min, v_max)) } else { Some((0.0, 100.0)) }
            }
            Surface::Cone(cone) => {
                let (v_min, v_max) = compute_axis_v_range(face, &cone.origin, &cone.axis);
                if v_min < v_max { Some((v_min, v_max)) } else { Some((0.0, cone.height().min(100.0))) }
            }
            Surface::Revolution(rev) => Some(rev.profile.param_range()),
            Surface::Extrusion(ext) => Some(ext.profile.param_range()),
            _ => Some((0.0, 1.0)),
        }
    } else {
        None
    }
}

/// Compute the v parameter range for axis-based surfaces (Cylinder, Cone).
/// When the face has no edges (e.g., simplified primitives), falls back
/// to surface geometry to estimate the v range.
fn compute_axis_v_range(face: &Face, origin: &Point3d, axis: &Direction3d) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for edge in &face.edges {
        for i in 0..64 {
            let t = i as f64 / 63.0;
            if let Some(p) = edge.point_at(t) {
                let v = (p.x - origin.x) * axis.x
                      + (p.y - origin.y) * axis.y
                      + (p.z - origin.z) * axis.z;
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }

    if v_min >= v_max {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                if let Some(edge) = edge {
                    for i in 0..64 {
                        let t = i as f64 / 63.0;
                        let t_actual = if coedge.forward { t } else { 1.0 - t };
                        if let Some(p) = edge.point_at(t_actual) {
                            let v = (p.x - origin.x) * axis.x
                                  + (p.y - origin.y) * axis.y
                                  + (p.z - origin.z) * axis.z;
                            v_min = v_min.min(v);
                            v_max = v_max.max(v);
                        }
                    }
                }
            }
        }
    }

    if v_min >= v_max {
        // No edges or wire — use surface geometry to estimate v range
        // For a cylinder/cone centered at origin along Z axis, v range
        // corresponds to the height. Use the bounding box of the surface.
        if let Some(ref surface) = face.surface {
            match surface {
                Surface::Cylinder(cyl) => {
                    // For a full cylinder with no edges, use a default height of 1.0
                    let base_pt = cyl.point_at(0.0, 0.0);
                    let top_pt = cyl.point_at(0.0, 1.0);
                    let v_base = (base_pt.x - origin.x) * axis.x
                               + (base_pt.y - origin.y) * axis.y
                               + (base_pt.z - origin.z) * axis.z;
                    let v_top = (top_pt.x - origin.x) * axis.x
                              + (top_pt.y - origin.y) * axis.y
                              + (top_pt.z - origin.z) * axis.z;
                    v_min = v_base.min(v_top);
                    v_max = v_base.max(v_top);
                }
                Surface::Cone(cone) => {
                    let h = cone.height().min(100.0);
                    let base_pt = cone.point_at(0.0, 0.0);
                    let apex_pt = cone.point_at(0.0, h);
                    let v_base = (base_pt.x - origin.x) * axis.x
                               + (base_pt.y - origin.y) * axis.y
                               + (base_pt.z - origin.z) * axis.z;
                    let v_apex = (apex_pt.x - origin.x) * axis.x
                               + (apex_pt.y - origin.y) * axis.y
                               + (apex_pt.z - origin.z) * axis.z;
                    v_min = v_base.min(v_apex);
                    v_max = v_base.max(v_apex);
                }
                _ => {
                    return (0.0, 1.0);
                }
            }
        } else {
            return (0.0, 1.0);
        }
    }

    let margin = (v_max - v_min) * 0.001;
    (v_min - margin, v_max + margin)
}

/// Compute the v (extrusion) parameter range for an extrusion surface.
fn compute_extrusion_v_range(face: &Face, ext: &draper_geometry::ExtrusionSurface) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for edge in &face.edges {
        for i in 0..64 {
            let t = i as f64 / 63.0;
            if let Some(p) = edge.point_at(t) {
                let origin = ext.profile.point_at(0.0);
                let v = (p.x - origin.x) * ext.direction.x
                      + (p.y - origin.y) * ext.direction.y
                      + (p.z - origin.z) * ext.direction.z;
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }

    if v_min >= v_max {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                if let Some(edge) = edge {
                    for i in 0..64 {
                        let t = i as f64 / 63.0;
                        let t_actual = if coedge.forward { t } else { 1.0 - t };
                        if let Some(p) = edge.point_at(t_actual) {
                            let origin = ext.profile.point_at(0.0);
                            let v = (p.x - origin.x) * ext.direction.x
                                  + (p.y - origin.y) * ext.direction.y
                                  + (p.z - origin.z) * ext.direction.z;
                            v_min = v_min.min(v);
                            v_max = v_max.max(v);
                        }
                    }
                }
            }
        }
    }

    if v_min >= v_max { (0.0, 1.0) } else {
        let margin = (v_max - v_min) * 0.001;
        (v_min - margin, v_max + margin)
    }
}

// ============================================================
// Boundary ring snapping for curved surfaces
// ============================================================

/// Snap boundary ring vertices of a curved surface mesh to edge curve samples.
/// This is a generic helper that works for any axis-based surface (cylinder, cone).
fn snap_boundary_rings(
    mesh: &mut TriangleMesh,
    boundary_3d: &[Point3d],
    surface: &Surface,
    n_u: usize,
    n_v: usize,
    u_start: f64,
    u_end: f64,
    v_min: f64,
    v_max: f64,
    full_circle: bool,
) {
    // For each boundary ring (top and bottom rows), find edge curve points
    // at the corresponding v values and snap grid vertices to them
    for (row_j, target_v) in [(0, v_min), (n_v - 1, v_max)] {
        let v_tol = (v_max - v_min) * 0.01 + 1e-6;

        // Collect boundary points near this v value
        let mut ring_pts: Vec<(f64, Point3d)> = Vec::new();
        for p in boundary_3d {
            if let Some((u, v)) = surface.project_point_opt(p) {
                if (v - target_v).abs() < v_tol {
                    ring_pts.push((u, *p));
                }
            }
        }

        if ring_pts.is_empty() {
            continue;
        }

        // For each grid vertex in this row, find the closest boundary point
        let offset = row_j * n_u;
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let mut best_dist = f64::MAX;
            let mut best_pt = None;
            for (ru, rp) in &ring_pts {
                let du = if full_circle {
                    let diff = (u - ru).abs();
                    diff.min(2.0 * PI - diff)
                } else {
                    (u - ru).abs()
                };
                if du < best_dist {
                    best_dist = du;
                    best_pt = Some(*rp);
                }
            }
            // Only snap if the boundary point is very close in angle
            let angle_tol = (u_end - u_start) / n_u as f64 * 0.5;
            if best_dist < angle_tol {
                if let Some(pt) = best_pt {
                    mesh.vertices[offset + i] = pt;
                }
            }
        }
    }
}

/// Remove consecutive duplicate points within a tolerance, and also check
/// that the last point is not coincident with the first (for closed loops).
fn deduplicate_points_3d(points: &[Point3d], tolerance: f64) -> Vec<Point3d> {
    if points.is_empty() {
        return Vec::new();
    }

    let tol_sq = tolerance * tolerance;
    let mut unique = vec![points[0]];
    for p in &points[1..] {
        if let Some(last) = unique.last() {
            let dx = p.x - last.x;
            let dy = p.y - last.y;
            let dz = p.z - last.z;
            if dx * dx + dy * dy + dz * dz > tol_sq {
                unique.push(*p);
            }
        }
    }
    // Also check last vs first (closed loop)
    if unique.len() > 1 {
        let first = unique[0];
        if let Some(last) = unique.last() {
            let dx = first.x - last.x;
            let dy = first.y - last.y;
            let dz = first.z - last.z;
            if dx * dx + dy * dy + dz * dz <= tol_sq {
                unique.pop();
            }
        }
    }
    unique
}

// ============================================================
// Ear clipping triangulation
// ============================================================

/// Ear clipping triangulation of a 2D polygon.
/// Returns triangle indices into the original point array.
/// Produces N-2 triangles for a simple polygon with N vertices (minimum for convex).
pub fn ear_clip(points: &[Point2d]) -> Vec<[u32; 3]> {
    let n = points.len();
    if n < 3 {
        return vec![];
    }
    if n == 3 {
        return vec![[0, 1, 2]];
    }

    // Determine winding order
    let mut signed_area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        signed_area += points[i].u * points[j].v - points[j].u * points[i].v;
    }
    let ccw = signed_area > 0.0;

    let mut indices: Vec<u32> = (0..n as u32).collect();
    let mut triangles = Vec::new();

    let mut attempts = 0;
    let max_attempts = n * n;

    while indices.len() > 3 && attempts < max_attempts {
        attempts += 1;
        let len = indices.len();
        let mut found_ear = false;

        for i in 0..len {
            let i_prev = if i == 0 { len - 1 } else { i - 1 };
            let i_next = (i + 1) % len;

            let a = indices[i_prev];
            let b = indices[i];
            let c = indices[i_next];

            let pa = &points[a as usize];
            let pb = &points[b as usize];
            let pc = &points[c as usize];

            let cross = (pb.u - pa.u) * (pc.v - pa.v) - (pb.v - pa.v) * (pc.u - pa.u);
            let is_convex = if ccw { cross > 0.0 } else { cross < 0.0 };

            if !is_convex {
                continue;
            }

            let mut is_ear = true;
            for j in 0..len {
                if j == i_prev || j == i || j == i_next {
                    continue;
                }
                let p = &points[indices[j] as usize];
                if point_in_triangle(pa, pb, pc, p) {
                    is_ear = false;
                    break;
                }
            }

            if is_ear {
                triangles.push([a, b, c]);
                indices.remove(i);
                found_ear = true;
                break;
            }
        }

        if !found_ear {
            // Degenerate polygon — fan triangulate as fallback
            for i in 1..indices.len() - 1 {
                triangles.push([indices[0], indices[i], indices[i + 1]]);
            }
            break;
        }
    }

    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }

    triangles
}

/// Debug assertion to verify that all per-triangle arrays in a mesh have consistent lengths.
/// This catches bugs where post-processing removes triangles but forgets to update
/// the corresponding entries in face_normals, triangle_colors, or triangle_face_ids.
fn debug_assert_mesh_consistency(mesh: &TriangleMesh) {
    let n = mesh.triangles.len();
    if let Some(ref face_normals) = mesh.face_normals {
        debug_assert_eq!(
            face_normals.len(), n,
            "face_normals length ({}) != triangles length ({})",
            face_normals.len(), n
        );
    }
    if let Some(ref colors) = mesh.triangle_colors {
        debug_assert_eq!(
            colors.len(), n,
            "triangle_colors length ({}) != triangles length ({})",
            colors.len(), n
        );
    }
    if let Some(ref ids) = mesh.triangle_face_ids {
        debug_assert_eq!(
            ids.len(), n,
            "triangle_face_ids length ({}) != triangles length ({})",
            ids.len(), n
        );
    }
}

/// Check if a 2D point is inside a triangle.
fn point_in_triangle(a: &Point2d, b: &Point2d, c: &Point2d, p: &Point2d) -> bool {
    let d1 = sign_area_2d(p, a, b);
    let d2 = sign_area_2d(p, b, c);
    let d3 = sign_area_2d(p, c, a);
    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
    !(has_neg && has_pos)
}

/// Signed area of triangle (p, a, b) * 2.
fn sign_area_2d(p: &Point2d, a: &Point2d, b: &Point2d) -> f64 {
    (p.u - b.u) * (a.v - b.v) - (a.u - b.u) * (p.v - b.v)
}

/// Check if a 2D polygon is convex.
fn is_convex_polygon(points: &[Point2d]) -> bool {
    if points.len() < 3 {
        return false;
    }
    let n = points.len();
    let mut sign = 0i32;
    for i in 0..n {
        let a = &points[i];
        let b = &points[(i + 1) % n];
        let c = &points[(i + 2) % n];
        let cross = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
        if cross.abs() > 1e-10 {
            let s = if cross > 0.0 { 1 } else { -1 };
            if sign == 0 { sign = s; } else if sign != s { return false; }
        }
    }
    sign != 0
}

/// Decimate collinear boundary points in 3D.
///
/// For a closed boundary polyline, removes intermediate points that are
/// collinear with their neighbors (within `tolerance`). This reduces
/// the boundary to just the "corner" vertices where the polyline changes
/// direction.
///
/// # Algorithm
/// 1. For each triple of consecutive points (prev, curr, next), compute
///    the cross product of (curr - prev) × (next - curr).
/// 2. If the cross product magnitude is below the tolerance, `curr` is
///    collinear with `prev` and `next` and can be removed.
/// 3. Keep `curr` if it's a "corner" (cross product above tolerance).
///
/// # Why This Matters for Watertightness
/// When the edge cache discretizes a shared edge between two faces, it
/// produces many intermediate points. For STRAIGHT edges, these points
/// are collinear in 3D. Both faces receive the same points (from the
/// shared edge cache).
///
/// Without decimation:
/// - Planar face fan triangulation produces degenerate triangles for
///   collinear points, which are removed. The intermediate vertices
///   become unused.
/// - NURBS face earcutr may also leave intermediate points unused.
/// - Unused vertices in one face but not the other create boundary edges.
///
/// With decimation:
/// - Both faces decimate the same way (same edge cache input), keeping
///   only the corner vertices.
/// - Shared edges have only 2 vertices (the corners), no intermediate
///   boundary edges.
///
/// # Arguments
/// * `points_3d` — Closed boundary polyline in 3D
/// * `uvs` — Corresponding UV coordinates (same length as points_3d)
///
/// # Returns
/// Decimated (points_3d, uvs) with collinear intermediate points removed.
fn decimate_collinear_boundary(
    points_3d: &[Point3d],
    uvs: &[Point2d],
) -> (Vec<Point3d>, Vec<Point2d>) {
    if points_3d.len() <= 3 || uvs.len() != points_3d.len() {
        return (points_3d.to_vec(), uvs.to_vec());
    }

    // Step 1: Remove consecutive duplicate points (within tolerance).
    // This is critical because the edge cache may produce duplicate points
    // at edge endpoints (where two edges share a vertex). Without this step,
    // the collinear check below would trivially pass for all points
    // (since prev == curr or curr == next gives cross product = 0).
    let dedup_tol_sq = 1e-12; // 1e-6 squared
    let mut dedup_3d: Vec<Point3d> = Vec::with_capacity(points_3d.len());
    let mut dedup_uv: Vec<Point2d> = Vec::with_capacity(uvs.len());
    for i in 0..points_3d.len() {
        let p = &points_3d[i];
        let is_dup = dedup_3d.last().map_or(false, |last| {
            let dx = p.x - last.x;
            let dy = p.y - last.y;
            let dz = p.z - last.z;
            dx * dx + dy * dy + dz * dz < dedup_tol_sq
        });
        if !is_dup {
            dedup_3d.push(*p);
            dedup_uv.push(uvs[i]);
        }
    }
    // Also check last vs first (closed loop)
    if dedup_3d.len() > 1 {
        let first = dedup_3d[0];
        let last = dedup_3d[dedup_3d.len() - 1];
        let dx = first.x - last.x;
        let dy = first.y - last.y;
        let dz = first.z - last.z;
        if dx * dx + dy * dy + dz * dz < dedup_tol_sq {
            dedup_3d.pop();
            dedup_uv.pop();
        }
    }

    if dedup_3d.len() <= 3 {
        // After dedup, too few points — return as-is
        return (dedup_3d, dedup_uv);
    }

    let n = dedup_3d.len();
    // Tolerance: absolute perpendicular distance from `curr` to line (prev → next).
    // 1e-6 = 0.001 microns — well below manufacturing tolerance but above FP noise.
    let abs_tol = 1e-6;

    // Step 2: Identify collinear points to remove.
    let mut keep = vec![true; n];
    let mut removed_count = 0usize;

    for i in 0..n {
        let prev_idx = (i + n - 1) % n;
        let next_idx = (i + 1) % n;

        let prev = &dedup_3d[prev_idx];
        let curr = &dedup_3d[i];
        let next = &dedup_3d[next_idx];

        // Segment length (prev → next)
        let seg_dx = next.x - prev.x;
        let seg_dy = next.y - prev.y;
        let seg_dz = next.z - prev.z;
        let seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy + seg_dz * seg_dz;

        if seg_len_sq < 1e-20 {
            // prev and next are coincident — keep curr as a sanity check
            continue;
        }

        // Edge vectors
        let e1x = curr.x - prev.x;
        let e1y = curr.y - prev.y;
        let e1z = curr.z - prev.z;
        let e2x = next.x - curr.x;
        let e2y = next.y - curr.y;
        let e2z = next.z - curr.z;

        // Cross product e1 × e2 gives the area of the parallelogram.
        // Perpendicular distance from curr to line (prev → next) = |e1 × e2| / |seg|
        let cross_x = e1y * e2z - e1z * e2y;
        let cross_y = e1z * e2x - e1x * e2z;
        let cross_z = e1x * e2y - e1y * e2x;
        let cross_mag_sq = cross_x * cross_x + cross_y * cross_y + cross_z * cross_z;
        let perp_dist = (cross_mag_sq / seg_len_sq).sqrt();

        if perp_dist < abs_tol {
            // curr is collinear with prev and next — remove it
            keep[i] = false;
            removed_count += 1;
        }
    }

    if removed_count == 0 {
        return (dedup_3d, dedup_uv);
    }

    // Build decimated arrays
    let mut dec_3d = Vec::with_capacity(n - removed_count);
    let mut dec_uv = Vec::with_capacity(n - removed_count);
    for i in 0..n {
        if keep[i] {
            dec_3d.push(dedup_3d[i]);
            dec_uv.push(dedup_uv[i]);
        }
    }

    if removed_count > 0 {
        log::debug!(
            "decimate_collinear_boundary: removed {} of {} collinear points ({} remain)",
            removed_count, n, dec_3d.len(),
        );
    }

    (dec_3d, dec_uv)
}

// ============================================================
// Chord error refinement (placeholder)
// ============================================================

// Note: Chord error refinement is implemented in parametric_domain.rs as
// `refine_mesh_chord_error_uv()`, which operates on UV coordinates and
// is called during `triangulate_surface_consistent()`. No separate
// post-hoc refinement pass is needed here.

// ============================================================
// Vertex merging for watertight solids
// ============================================================

/// Merge coincident vertices in a mesh within the given tolerance.
/// This makes closed solids watertight by ensuring that shared edge
/// vertices between adjacent faces use the same vertex index.
///
/// # ⚠️ DEPRECATED — DO NOT USE in the main triangulation pipeline.
///
/// This function MASKS bugs in the edge cache by arbitrarily moving vertices
/// within `tolerance`. The unified edge cache should produce **bit-identical**
/// vertices on shared edges, making this function unnecessary. If boundary
/// edges exist after triangulation, fix the edge cache instead.
#[deprecated(
    since = "0.3.0",
    note = "Do not use in main pipeline. If mesh has boundary edges, fix the edge cache instead."
)]
pub fn merge_coincident_vertices(mesh: &mut TriangleMesh, tolerance: f64) {
    if mesh.vertices.is_empty() {
        return;
    }

    let tol_sq = tolerance * tolerance;
    let n = mesh.vertices.len();

    let mut remap: Vec<u32> = vec![0; n];
    let mut new_vertices: Vec<Point3d> = Vec::with_capacity(n);

    let cell_size = tolerance * 10.0;
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();

    for (i, v) in mesh.vertices.iter().enumerate() {
        let cx = (v.x / cell_size).floor() as i64;
        let cy = (v.y / cell_size).floor() as i64;
        let cz = (v.z / cell_size).floor() as i64;

        let mut found = false;
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(indices) = grid.get(&key) {
                        for &j in indices {
                            let ov = &new_vertices[j as usize];
                            let ddx = v.x - ov.x;
                            let ddy = v.y - ov.y;
                            let ddz = v.z - ov.z;
                            if ddx * ddx + ddy * ddy + ddz * ddz < tol_sq {
                                remap[i] = j;
                                found = true;
                                break;
                            }
                        }
                    }
                    if found { break; }
                }
                if found { break; }
            }
            if found { break; }
        }

        if !found {
            let new_idx = new_vertices.len() as u32;
            new_vertices.push(*v);
            remap[i] = new_idx;
            grid.entry((cx, cy, cz)).or_default().push(new_idx);
        }
    }

    // Apply remap to triangles and filter degenerate ones
    // Also preserve ALL per-triangle arrays in sync with the filtered triangles
    let old_triangles = std::mem::take(&mut mesh.triangles);
    let old_face_ids = mesh.triangle_face_ids.take();
    let old_face_normals = mesh.face_normals.take();
    let old_triangle_colors = mesh.triangle_colors.take();
    for (i, tri) in old_triangles.iter().enumerate() {
        let a = remap[tri[0] as usize];
        let b = remap[tri[1] as usize];
        let c = remap[tri[2] as usize];
        if a != b && b != c && a != c {
            mesh.triangles.push([a, b, c]);
            if let Some(ref ids) = old_face_ids {
                if let Some(&fid) = ids.get(i) {
                    mesh.triangle_face_ids.get_or_insert_with(Vec::new).push(fid);
                }
            }
            if let Some(ref normals) = old_face_normals {
                if let Some(&n) = normals.get(i) {
                    mesh.face_normals.get_or_insert_with(Vec::new).push(n);
                }
            }
            if let Some(ref colors) = old_triangle_colors {
                if let Some(&col) = colors.get(i) {
                    mesh.triangle_colors.get_or_insert_with(Vec::new).push(col);
                }
            }
        }
    }

    mesh.vertices = new_vertices;

    // Rebuild vertex normals by averaging normals of merged vertices.
    // Previously, normals were simply discarded (set to None), causing
    // flat shading on merged meshes. Now we average the normals of all
    // vertices that were merged together, then renormalize.
    if let Some(old_normals) = mesh.normals.take() {
        if !old_normals.is_empty() {
            let n_new = mesh.vertices.len();
            let mut new_normals: Vec<[f64; 3]> = vec![[0.0, 0.0, 0.0]; n_new];
            let mut counts: Vec<usize> = vec![0; n_new];
            for (i, &remapped) in remap.iter().enumerate() {
                if i < old_normals.len() {
                    let n = old_normals[i];
                    new_normals[remapped as usize][0] += n[0];
                    new_normals[remapped as usize][1] += n[1];
                    new_normals[remapped as usize][2] += n[2];
                    counts[remapped as usize] += 1;
                }
            }
            // Normalize the averaged normals
            for (i, n) in new_normals.iter_mut().enumerate() {
                if counts[i] > 0 {
                    *n = [n[0] / counts[i] as f64, n[1] / counts[i] as f64, n[2] / counts[i] as f64];
                    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                    if len > 1e-10 {
                        *n = [n[0] / len, n[1] / len, n[2] / len];
                    }
                }
            }
            mesh.normals = Some(new_normals);
        }
    }
}

/// Filter out degenerate and invalid triangles from the mesh.
///
/// A triangle is removed if:
/// - Any of its vertices have NaN or Inf coordinates
/// - Its area is below `tolerance²` (zero-area / degenerate triangle)
/// - Two or more vertex indices are the same (collapsed triangle)
///
/// This is called after `merge_coincident_vertices` as a final cleanup step.
pub fn filter_degenerate_triangles(mesh: &mut TriangleMesh, tolerance: f64) {
    let min_area_sq = tolerance * tolerance;
    let old_triangles = std::mem::take(&mut mesh.triangles);
    let old_face_ids = mesh.triangle_face_ids.take();
    let old_face_normals = mesh.face_normals.take();
    let old_triangle_colors = mesh.triangle_colors.take();

    for (i, tri) in old_triangles.iter().enumerate() {
        let a_idx = tri[0] as usize;
        let b_idx = tri[1] as usize;
        let c_idx = tri[2] as usize;

        // Check for collapsed triangles (duplicate vertex indices)
        if a_idx == b_idx || b_idx == c_idx || a_idx == c_idx {
            continue;
        }

        // Bounds check
        if a_idx >= mesh.vertices.len() || b_idx >= mesh.vertices.len() || c_idx >= mesh.vertices.len() {
            continue;
        }

        let a = &mesh.vertices[a_idx];
        let b = &mesh.vertices[b_idx];
        let c = &mesh.vertices[c_idx];

        // Check for NaN/Inf vertices
        if !a.x.is_finite() || !a.y.is_finite() || !a.z.is_finite() ||
           !b.x.is_finite() || !b.y.is_finite() || !b.z.is_finite() ||
           !c.x.is_finite() || !c.y.is_finite() || !c.z.is_finite() {
            continue;
        }

        // Check for zero-area triangles using cross product magnitude
        // |cross| / 2 = area, so |cross|² / 4 = area²
        // We compare |cross|² < 4 * min_area² = (2 * tolerance)²
        let e1x = b.x - a.x;
        let e1y = b.y - a.y;
        let e1z = b.z - a.z;
        let e2x = c.x - a.x;
        let e2y = c.y - a.y;
        let e2z = c.z - a.z;
        let cx = e1y * e2z - e1z * e2y;
        let cy = e1z * e2x - e1x * e2z;
        let cz = e1x * e2y - e1y * e2x;
        let cross_mag_sq = cx * cx + cy * cy + cz * cz;

        // Skip degenerate triangles (area < tolerance²)
        if cross_mag_sq < 4.0 * min_area_sq * min_area_sq {
            continue;
        }

        mesh.triangles.push(*tri);
        if let Some(ref ids) = old_face_ids {
            if let Some(&fid) = ids.get(i) {
                mesh.triangle_face_ids.get_or_insert_with(Vec::new).push(fid);
            }
        }
        if let Some(ref normals) = old_face_normals {
            if let Some(&n) = normals.get(i) {
                mesh.face_normals.get_or_insert_with(Vec::new).push(n);
            }
        }
        if let Some(ref colors) = old_triangle_colors {
            if let Some(&col) = colors.get(i) {
                mesh.triangle_colors.get_or_insert_with(Vec::new).push(col);
            }
        }
    }
}

#[cfg(test)]
mod ring_surface_tests {
    use super::*;
    use draper_geometry::{Point3d, Direction3d, Surface, Plane, CylinderSurface, SphereSurface, TorusSurface, ConeSurface};

    /// Helper: create a face with a surface but no boundary edges (full surface).
    fn make_full_face(surface: Surface) -> Face {
        Face::new_surface_only(surface)
    }

    #[test]
    fn test_full_cylinder_triangulation() {
        let cyl = CylinderSurface::new_z(5.0);
        let face = make_full_face(Surface::Cylinder(cyl));
        let params = TriangulationParams::default();
        let mesh = triangulate_face(&face, &params);
        assert!(mesh.triangles.len() > 0, "Cylinder should produce triangles");
        assert!(mesh.vertices.len() > 0, "Cylinder should produce vertices");
        for v in &mesh.vertices {
            assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
                "Cylinder vertex should be finite: {:?}", v);
        }
    }

    #[test]
    fn test_full_sphere_triangulation() {
        let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let face = make_full_face(Surface::Sphere(sphere));
        let params = TriangulationParams::default();
        let mesh = triangulate_face(&face, &params);
        assert!(mesh.triangles.len() > 0, "Sphere should produce triangles");
        assert!(mesh.vertices.len() > 0, "Sphere should produce vertices");
        for v in &mesh.vertices {
            assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
                "Sphere vertex should be finite: {:?}", v);
        }
    }

    #[test]
    fn test_full_torus_triangulation() {
        let torus = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0);
        let face = make_full_face(Surface::Torus(torus));
        let params = TriangulationParams::default();
        let mesh = triangulate_face(&face, &params);
        assert!(mesh.triangles.len() > 0, "Torus should produce triangles");
        assert!(mesh.vertices.len() > 0, "Torus should produce vertices");
        for v in &mesh.vertices {
            assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
                "Torus vertex should be finite: {:?}", v);
        }
    }

    #[test]
    fn test_cone_with_apex() {
        let cone = ConeSurface::new_z(5.0, 0.3);
        let face = make_full_face(Surface::Cone(cone));
        let params = TriangulationParams::default();
        let mesh = triangulate_face(&face, &params);
        assert!(mesh.triangles.len() > 0, "Cone should produce triangles");
        assert!(mesh.vertices.len() > 0, "Cone should produce vertices");
        for v in &mesh.vertices {
            assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite(),
                "Cone vertex should be finite: {:?}", v);
        }
    }

    #[test]
    fn test_plane_triangulation_minimal() {
        let plane = Plane::xy();
        // Face with no boundary edges produces empty mesh for planar faces
        let face = make_full_face(Surface::Plane(plane));
        let params = TriangulationParams::default();
        let mesh = triangulate_face(&face, &params);
        // Plane with no boundary points produces empty mesh — this is expected
        assert_eq!(mesh.triangles.len(), 0, "Plane with no boundary should produce no triangles");
    }

    #[test]
    fn test_ring_surface_no_degenerate_rows() {
        // Test triangulate_ring_surface directly with no degenerate rows (like a cylinder)
        let n_u = 8;
        let n_v = 4;
        let radius = 3.0;
        let mesh = triangulate_ring_surface(
            n_u, n_v, true, true,
            |i, j| {
                let u = 2.0 * std::f64::consts::PI * i as f64 / n_u as f64;
                let v = j as f64 / n_v as f64;
                Point3d::new(radius * u.cos(), radius * u.sin(), v)
            },
            |i, j| {
                let u = 2.0 * std::f64::consts::PI * i as f64 / n_u as f64;
                let _v = j as f64 / n_v as f64;
                Direction3d::new(u.cos(), u.sin(), 0.0).unwrap_or(Direction3d::Z)
            },
            |_j| false,
        );
        // Should have (n_v+1)*n_u vertices and n_v*n_u*2 triangles
        assert_eq!(mesh.vertices.len(), (n_v + 1) * n_u);
        assert_eq!(mesh.triangles.len(), n_v * n_u * 2);
    }

    #[test]
    fn test_ring_surface_with_apex() {
        // Test triangulate_ring_surface with a degenerate last row (cone apex)
        let n_u = 8;
        let n_v = 4;
        let mesh = triangulate_ring_surface(
            n_u, n_v, true, true,
            |i, j| {
                let u = 2.0 * std::f64::consts::PI * i as f64 / n_u as f64;
                let v = j as f64 / n_v as f64;
                let r = 3.0 * (1.0 - v);
                Point3d::new(r * u.cos(), r * u.sin(), v * 5.0)
            },
            |i, j| {
                let u = 2.0 * std::f64::consts::PI * i as f64 / n_u as f64;
                let v = j as f64 / n_v as f64;
                let r = 3.0 * (1.0 - v);
                if r.abs() < 1e-10 {
                    Direction3d::Z
                } else {
                    Direction3d::new(u.cos(), u.sin(), 0.0).unwrap_or(Direction3d::Z)
                }
            },
            |j| j == n_v, // Last row is degenerate (apex)
        );
        // Should have n_v*n_u + 1 vertices (n_v normal rows + 1 apex vertex)
        assert_eq!(mesh.vertices.len(), n_v * n_u + 1);
        // Triangles: (n_v-1) normal strips * n_u * 2 + 1 apex fan * n_u
        assert_eq!(mesh.triangles.len(), (n_v - 1) * n_u * 2 + n_u);
    }

    #[test]
    fn test_ring_surface_with_pole() {
        // Test triangulate_ring_surface with a degenerate first row (north pole)
        let n_u = 8;
        let n_v = 4;
        let mesh = triangulate_ring_surface(
            n_u, n_v, true, true,
            |i, j| {
                let u = 2.0 * std::f64::consts::PI * i as f64 / n_u as f64;
                let v = j as f64 / n_v as f64;
                let r = 3.0 * v.sin();
                Point3d::new(r * u.cos(), r * u.sin(), 3.0 * v.cos())
            },
            |i, j| {
                let u = 2.0 * std::f64::consts::PI * i as f64 / n_u as f64;
                let v = j as f64 / n_v as f64;
                let r = 3.0 * v.sin();
                if r.abs() < 1e-10 {
                    Direction3d::new(0.0, 0.0, v.cos().signum()).unwrap_or(Direction3d::Z)
                } else {
                    Direction3d::new(u.cos() * v.sin(), u.sin() * v.sin(), v.cos()).unwrap_or(Direction3d::Z)
                }
            },
            |j| j == 0, // First row is degenerate (north pole)
        );
        // Should have 1 + n_v*n_u vertices (1 pole + n_v normal rows)
        assert_eq!(mesh.vertices.len(), 1 + n_v * n_u);
    }
}

// ============================================================
// Parallel triangulation tests and benchmark (3.5)
// ============================================================

#[cfg(test)]
mod parallel_tests {
    use super::*;
    use draper_geometry::{Surface, CylinderSurface, SphereSurface, Plane};
    use draper_topology::{Face, Shell, Solid};
    use std::sync::{Arc, Mutex};

    /// Helper: create a Solid with multiple faces for parallel testing.
    /// Creates a solid with `n_faces` cylinder faces.
    fn make_multi_face_solid(n_faces: usize) -> Solid {
        let mut faces = Vec::with_capacity(n_faces);
        for _ in 0..n_faces {
            let cyl = CylinderSurface::new_z(5.0);
            faces.push(Face::new_surface_only(Surface::Cylinder(cyl)));
        }
        let shell = Shell::new_closed(faces);
        Solid::new(shell)
    }

    /// Helper: create a Solid with mixed surface types for parallel testing.
    fn make_mixed_solid() -> Solid {
        let mut faces = Vec::new();
        // 4 cylinders
        for _ in 0..4 {
            let cyl = CylinderSurface::new_z(5.0);
            faces.push(Face::new_surface_only(Surface::Cylinder(cyl)));
        }
        // 4 spheres
        for _ in 0..4 {
            let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
            faces.push(Face::new_surface_only(Surface::Sphere(sphere)));
        }
        // 4 planes
        for _ in 0..4 {
            faces.push(Face::new_surface_only(Surface::Plane(Plane::xy())));
        }
        let shell = Shell::new_closed(faces);
        Solid::new(shell)
    }

    #[test]
    fn test_parallel_produces_same_vertex_count() {
        let solid = make_mixed_solid();

        let mut params_seq = TriangulationParams::default();
        params_seq.parallel = false;
        let mesh_seq = triangulate_solid(&solid, &params_seq);

        let mut params_par = TriangulationParams::default();
        params_par.parallel = true;
        let mesh_par = triangulate_solid(&solid, &params_par);

        // Both should produce the same number of vertices and triangles
        // (post-processing might differ slightly, but counts should match)
        assert_eq!(
            mesh_seq.vertices.len(),
            mesh_par.vertices.len(),
            "Sequential and parallel should produce same vertex count"
        );
        assert_eq!(
            mesh_seq.triangles.len(),
            mesh_par.triangles.len(),
            "Sequential and parallel should produce same triangle count"
        );
    }

    #[test]
    fn test_parallel_with_progress_callback() {
        let solid = make_multi_face_solid(8);

        let progress_log = Arc::new(Mutex::new(Vec::new()));
        let progress_log_clone = progress_log.clone();

        let mut params = TriangulationParams::default();
        params.parallel = true;
        params.progress_callback = Some(Arc::new(move |completed, total| {
            progress_log_clone.lock().unwrap().push((completed, total));
        }));

        let mesh = triangulate_solid(&solid, &params);

        // Should have produced some triangles
        assert!(mesh.triangles.len() > 0, "Parallel triangulation should produce triangles");

        // Progress callback should have been called
        let log = progress_log.lock().unwrap();
        assert!(log.len() > 0, "Progress callback should have been called");
        // Total should match number of faces
        assert_eq!(log[0].1, 8, "Total faces should be 8");
    }

    #[test]
    fn test_parallel_empty_solid() {
        // A solid with no faces should return empty mesh
        let faces: Vec<Face> = vec![];
        let shell = Shell::new_closed(faces);
        let solid = Solid::new(shell);

        let mut params = TriangulationParams::default();
        params.parallel = true;
        let mesh = triangulate_solid(&solid, &params);

        assert_eq!(mesh.vertices.len(), 0);
        assert_eq!(mesh.triangles.len(), 0);
    }

    #[test]
    fn test_parallel_default_is_sequential() {
        let params = TriangulationParams::default();
        assert!(!params.parallel, "Default should be sequential (parallel=false)");
        assert!(params.progress_callback.is_none(), "Default should have no progress callback");
    }

    #[test]
    fn test_merge_meshes_parallel() {
        // Test the merge function directly
        let mut m1 = TriangleMesh::new();
        m1.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        m1.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        m1.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        m1.add_triangle(0, 1, 2);

        let mut m2 = TriangleMesh::new();
        m2.add_vertex(Point3d::new(0.0, 0.0, 1.0));
        m2.add_vertex(Point3d::new(1.0, 0.0, 1.0));
        m2.add_vertex(Point3d::new(0.0, 1.0, 1.0));
        m2.add_triangle(0, 1, 2);

        let merged = merge_meshes_sequential(&[m1, m2], 1e-10);

        assert_eq!(merged.vertices.len(), 6, "Should have 6 vertices");
        assert_eq!(merged.triangles.len(), 2, "Should have 2 triangles");
        // Second triangle's indices should be offset by 3
        assert_eq!(merged.triangles[1], [3, 4, 5], "Second triangle should have offset indices");
    }

    #[test]
    fn test_benchmark_sequential_vs_parallel() {
        // Simple benchmark: time both paths and print results.
        // This is a test, not a rigorous benchmark — use criterion for proper benchmarks.
        let solid = make_multi_face_solid(50);

        // Sequential
        let mut params_seq = TriangulationParams::default();
        params_seq.parallel = false;
        let start_seq = std::time::Instant::now();
        let _mesh_seq = triangulate_solid(&solid, &params_seq);
        let elapsed_seq = start_seq.elapsed();

        // Parallel
        let mut params_par = TriangulationParams::default();
        params_par.parallel = true;
        let start_par = std::time::Instant::now();
        let _mesh_par = triangulate_solid(&solid, &params_par);
        let elapsed_par = start_par.elapsed();

        eprintln!(
            "\n[3.5 Benchmark] Sequential: {:.2}ms, Parallel: {:.2}ms ({} faces)",
            elapsed_seq.as_secs_f64() * 1000.0,
            elapsed_par.as_secs_f64() * 1000.0,
            solid.faces().len(),
        );

        // Both should produce valid results (no assertion on speed — CI may have 1 core)
    }

    #[test]
    fn test_edge_sample_cache_build() {
        // Smoke test: triangulate a mixed solid to ensure the edge cache
        // (formerly EdgeSampleCache, now EdgeDiscretizationCache) builds without errors.
        let solid = make_mixed_solid();
        let params = TriangulationParams::default();
        let mesh = triangulate_solid(&solid, &params);
        // The mesh should have at least some vertices and triangles.
        assert!(mesh.vertex_count() > 0, "mesh should have vertices");
        assert!(mesh.triangle_count() > 0, "mesh should have triangles");
    }
}

// ============================================================
// Phase 1.2: Chunked BREP Triangulator
//
// Incremental triangulation that respects a time budget per frame.
// This prevents the browser from freezing when loading large STEP files.
// Designed for 60/120 FPS — each frame processes as many faces as
// possible within the time budget (default: 8ms for 120 FPS).
// ============================================================

/// Result of a single frame of chunked triangulation.
#[derive(Debug, Clone)]
pub enum ChunkResult {
    /// Triangulation is still in progress. Contains the partial mesh built so far.
    InProgress {
        /// Number of faces processed so far.
        faces_completed: usize,
        /// Total number of faces to process.
        faces_total: usize,
    },
    /// Triangulation is complete. Contains the final mesh.
    Complete(TriangleMesh),
}

/// Incremental BREP triangulator that processes faces in time-bounded chunks.
///
/// # Usage
/// ```ignore
/// let mut triangulator = ChunkedBrepTriangulator::new(solid, params, Duration::from_millis(8));
/// loop {
///     match triangulator.process_frame() {
///         ChunkResult::InProgress { faces_completed, faces_total } => {
///             // Update UI progress bar
///             update_progress(faces_completed, faces_total);
///             // Yield to the browser for rendering
///         }
///         ChunkResult::Complete(mesh) => {
///             // All faces done — display the mesh
///             display_mesh(mesh);
///             break;
///         }
///     }
/// }
/// ```
///
/// # Design
/// - **Time-budgeted**: Each `process_frame()` call processes as many faces as
///   possible within the configured time budget, then returns control.
/// - **Watertight by construction**: Uses the same edge cache and dedup strategy
///   as the sequential path, ensuring shared edges produce bit-identical vertices.
/// - **Progress reporting**: Returns the number of faces completed vs total.
/// - **No locking**: The edge cache is pre-populated before chunked processing
///   begins, so it's read-only during face triangulation.
pub struct ChunkedBrepTriangulator {
    /// The solid being triangulated.
    faces: Vec<Face>,
    /// Triangulation parameters.
    params: TriangulationParams,
    /// Pre-populated edge cache (read-only during chunked processing).
    cache: EdgeDiscretizationCache,
    /// Vertex deduplication map (accumulated across chunks, bit-exact only).
    dedup_map: crate::mesh::VertexDedupMap,
    /// Accumulated mesh from all processed faces.
    partial_mesh: TriangleMesh,
    /// Index of the next face to process.
    next_face_idx: usize,
    /// Total number of faces.
    total_faces: usize,
    /// Time budget per frame.
    time_budget: std::time::Duration,
    /// Whether triangulation is complete.
    is_complete: bool,
    /// Reference solid for smooth_normals_adaptive on finalization.
    solid: Solid,
    /// Per-frame timing statistics (last frame only).
    last_frame_time_ms: f64,
}

impl ChunkedBrepTriangulator {
    /// Create a new chunked triangulator for the given solid.
    ///
    /// Pre-populates the edge cache fully (3D points + UV coordinates) so that
    /// each `process_frame()` call only performs face triangulation — no lazy UV
    /// computation can stall the frame budget. The cache is read-only after this.
    ///
    /// # Arguments
    /// * `solid` — The solid to triangulate.
    /// * `params` — Triangulation parameters (max_deviation, detail_level, etc.).
    /// * `time_budget` — Maximum time to spend per `process_frame()` call.
    ///   Recommended: 8ms for 120 FPS, 16ms for 60 FPS.
    pub fn new(solid: &Solid, params: TriangulationParams, time_budget: std::time::Duration) -> Self {
        // Compute adaptive tolerance from the solid's bounding box
        let bbox = solid_bounding_box(solid);
        let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(
            &bbox.0, &bbox.1, EDGE_SAMPLES,
        );

        // Pre-populate the edge cache FULLY (3D + UV for all faces).
        // This ensures that process_frame() only does face triangulation,
        // never lazy UV computation that could blow the frame budget.
        cache.pre_populate_for_solid_full(solid, EDGE_SAMPLES);

        // Get tolerance before moving cache into the struct
        let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();

        // Collect faces and sort by estimated complexity (complex faces first).
        // This gives better progressive rendering: the viewer sees large curved
        // faces appear early, while small planar faces fill in last.
        let mut faces: Vec<Face> = solid.faces().into_iter().cloned().collect();
        faces.sort_by(|a, b| {
            let complexity_a = estimate_face_complexity(a);
            let complexity_b = estimate_face_complexity(b);
            complexity_b.cmp(&complexity_a) // Descending: complex first
        });

        let total_faces = faces.len();

        Self {
            faces,
            params,
            cache,
            dedup_map: crate::mesh::VertexDedupMap::with_tolerance(adaptive_tol),
            partial_mesh: TriangleMesh::new(),
            next_face_idx: 0,
            total_faces,
            time_budget,
            is_complete: false,
            solid: solid.clone(),
            last_frame_time_ms: 0.0,
        }
    }

    /// Create a new chunked triangulator with a pre-populated, Arc-wrapped cache.
    ///
    /// Use this when the cache has already been populated (e.g., by a previous
    /// parallel triangulation call). Avoids redundant cache population.
    pub fn with_cache(
        solid: &Solid,
        params: TriangulationParams,
        time_budget: std::time::Duration,
        cache: EdgeDiscretizationCache,
    ) -> Self {
        // Get tolerance before moving cache into the struct
        let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();

        let mut faces: Vec<Face> = solid.faces().into_iter().cloned().collect();
        faces.sort_by(|a, b| {
            let complexity_a = estimate_face_complexity(a);
            let complexity_b = estimate_face_complexity(b);
            complexity_b.cmp(&complexity_a)
        });

        let total_faces = faces.len();

        Self {
            faces,
            params,
            cache,
            dedup_map: crate::mesh::VertexDedupMap::with_tolerance(adaptive_tol),
            partial_mesh: TriangleMesh::new(),
            next_face_idx: 0,
            total_faces,
            time_budget,
            is_complete: false,
            solid: solid.clone(),
            last_frame_time_ms: 0.0,
        }
    }

    /// Process one frame of triangulation, respecting the time budget.
    ///
    /// Processes as many faces as possible within the time budget, then returns.
    /// Since the edge cache is fully pre-populated (3D + UV), no lazy computation
    /// can stall the frame — each face triangulation is bounded by the face's
    /// geometric complexity alone.
    pub fn process_frame(&mut self) -> ChunkResult {
        if self.is_complete {
            return ChunkResult::Complete(self.partial_mesh.clone());
        }

        let start = Instant::now();
        let mut faces_this_frame = 0usize;

        while self.next_face_idx < self.total_faces {
            // Check time budget before processing each face
            let elapsed = start.elapsed();
            if elapsed >= self.time_budget {
                self.last_frame_time_ms = elapsed.as_secs_f64() * 1000.0;
                log::trace!(
                    "Chunked frame: {} faces in {:.1}ms (budget: {:.0}ms)",
                    faces_this_frame, self.last_frame_time_ms,
                    self.time_budget.as_secs_f64() * 1000.0,
                );
                return ChunkResult::InProgress {
                    faces_completed: self.next_face_idx,
                    faces_total: self.total_faces,
                };
            }

            let face = &self.faces[self.next_face_idx];
            let face_mesh = triangulate_face_impl(face, &self.params, &self.cache);
            self.partial_mesh.merge_deduplicating(&face_mesh, &mut self.dedup_map);
            self.next_face_idx += 1;
            faces_this_frame += 1;
        }

        // All faces processed — finalize
        filter_degenerate_triangles(&mut self.partial_mesh, 1e-10);

        // Smooth normals with adaptive crease angle (same as sequential/parallel paths)
        crate::watertight::smooth_normals_adaptive(&mut self.partial_mesh, &self.solid);

        // Validate watertightness
        let report = crate::watertight::validate_watertight(&self.partial_mesh, false);
        if !report.is_watertight() {
            let adaptive_tol = self.cache.adaptive_tolerance().merge_tolerance();
            let boundary_pct = if report.edge_count > 0 {
                report.boundary_edge_count as f64 / report.edge_count as f64 * 100.0
            } else {
                0.0
            };
            log::error!(
                "BUG: Chunked triangulation not watertight: {} boundary edges ({:.2}%) (V={}, E={}, F={})",
                report.boundary_edge_count, boundary_pct,
                report.vertex_count, report.edge_count, report.triangle_count,
            );
        } else {
            log::info!(
                "Chunked triangulation watertight ✓ ({} interior edges, {} triangles)",
                report.interior_edge_count, report.triangle_count,
            );
        }

        // Log dedup stats
        let (exact_hits, _tolerance_hits, misses) = self.dedup_map.stats();
        let total_lookups = exact_hits + misses;
        if total_lookups > 0 {
            log::info!(
                "Chunked dedup: {} bit-exact [{:.1}%], {} misses",
                exact_hits, exact_hits as f64 / total_lookups as f64 * 100.0,
                misses,
            );
        }

        self.last_frame_time_ms = start.elapsed().as_secs_f64() * 1000.0;
        self.is_complete = true;
        ChunkResult::Complete(self.partial_mesh.clone())
    }

    /// Get the current progress as (faces_completed, faces_total).
    pub fn progress(&self) -> (usize, usize) {
        (self.next_face_idx, self.total_faces)
    }

    /// Check if triangulation is complete.
    pub fn is_complete(&self) -> bool {
        self.is_complete
    }

    /// Get the partial mesh built so far (for progressive rendering).
    ///
    /// Returns `None` if no faces have been processed yet.
    pub fn partial_mesh(&self) -> Option<&TriangleMesh> {
        if self.next_face_idx > 0 {
            Some(&self.partial_mesh)
        } else {
            None
        }
    }

    /// Get the time spent in the last `process_frame()` call, in milliseconds.
    ///
    /// Useful for adaptive frame budgeting: if `last_frame_time_ms` is consistently
    /// well below the budget, you can reduce the budget or process more aggressively.
    pub fn last_frame_time_ms(&self) -> f64 {
        self.last_frame_time_ms
    }

    /// Change the LOD (Level of Detail) for remaining unprocessed faces.
    ///
    /// This is useful for progressive rendering where the viewer starts at a
    /// distance (low LOD) and zooms in (high LOD). The already-processed
    /// faces keep their original LOD; only subsequent faces use the new params.
    ///
    /// # Arguments
    /// * `lod` — LOD value in [0.0, 1.0]. See [`TriangulationParams::for_lod`].
    pub fn set_lod(&mut self, lod: f64) {
        self.params = TriangulationParams::for_lod(lod);
    }

    /// Change the detail level for remaining unprocessed faces.
    ///
    /// Unlike `set_lod`, this preserves the current base parameters and only
    /// scales the `detail_level`, `angular_samples`, `height_samples`, and
    /// `max_face_triangles` proportionally. Use this when you want to keep
    /// custom `max_deviation` or other settings while adjusting detail.
    pub fn set_detail_level(&mut self, detail_level: f64) {
        self.params = self.params.with_detail_level(detail_level);
    }

    /// Get the current detail level.
    pub fn detail_level(&self) -> f64 {
        self.params.detail_level
    }

    /// Progress as a fraction in [0.0, 1.0].
    ///
    /// Useful for progress bars and adaptive quality decisions.
    /// Returns 1.0 when the triangulation is complete.
    pub fn progress_fraction(&self) -> f64 {
        if self.total_faces == 0 {
            1.0
        } else {
            self.next_face_idx as f64 / self.total_faces as f64
        }
    }

    /// Take ownership of the final mesh, consuming the triangulator.
    ///
    /// # Panics
    /// Panics if the triangulation is not yet complete.
    pub fn into_mesh(self) -> TriangleMesh {
        assert!(self.is_complete, "Triangulation not yet complete — call process_frame() until it returns ChunkResult::Complete");
        self.partial_mesh
    }

    /// Get the current time budget per frame.
    pub fn time_budget(&self) -> std::time::Duration {
        self.time_budget
    }

    /// Set a new time budget per frame.
    ///
    /// Can be called between frames to adapt to changing frame rates.
    /// For example, if the frame rate drops from 120fps to 60fps,
    /// increase the budget from 8ms to 16ms.
    pub fn set_time_budget(&mut self, budget: std::time::Duration) {
        self.time_budget = budget;
    }

    /// Get the current triangulation parameters.
    pub fn params(&self) -> &TriangulationParams {
        &self.params
    }

    /// Set new triangulation parameters for remaining faces.
    ///
    /// This is a general-purpose alternative to `set_lod()` and
    /// `set_detail_level()`. Allows changing any parameter (e.g.,
    /// `max_deviation`, `max_face_triangles`) for subsequent faces.
    pub fn set_params(&mut self, params: TriangulationParams) {
        self.params = params;
    }
}

/// Estimate the complexity of a face for sorting purposes.
///
/// Higher complexity → should be processed earlier for better progressive rendering.
/// NURBS faces are the most complex (project_point is expensive), followed by
/// curved analytic surfaces, then planes (cheapest).
fn estimate_face_complexity(face: &Face) -> u32 {
    if let Some(ref surface) = face.surface {
        match surface {
            Surface::Nurbs(_) => 100,
            Surface::Torus(_) => 60,
            Surface::Sphere(_) => 50,
            Surface::Cylinder(_) => 40,
            Surface::Cone(_) => 40,
            Surface::Revolution(_) => 70,
            Surface::Extrusion(_) => 30,
            Surface::Plane(_) => 10,
        }
    } else {
        0
    }
}

// ============================================================
// Phase 2.2: Fallback Surface Triangulation
// ============================================================
//
// When the primary surface-specific triangulation returns an empty mesh,
// these fallback strategies provide graceful degradation. The goal is
// to always produce a visible mesh rather than silently dropping a face,
// even if the mesh is only an approximation.
//
// Three-tier strategy (in order of decreasing quality):
//
// 1. ApproximatePlane — fit a plane through boundary points and ear-clip.
//    Best quality for near-planar faces. Works for faces with holes.
//
// 2. BoundaryFan — fan-triangulate from the centroid of boundary points.
//    Works for any face shape but may produce degenerate triangles on
//    highly concave boundaries. Does not handle holes.
//
// 3. SurfacePointSample — sample surface.point_at() on a regular UV grid.
//    Only works when face.surface is present. Produces a rough grid mesh
//    without proper trimming. Last resort for curved surfaces.
//
// Additionally, `collect_face_boundary_no_surface` collects boundary 3D
// points from the cache when no surface is available (face.surface = None).

/// Collect boundary 3D points from the cache without a surface reference.
///
/// This is used when `face.surface` is `None` — the cache still has
/// discretized edge points, so we can collect them for fallback strategies.
/// No UV computation is attempted since there's no surface to project onto.
fn collect_face_boundary_no_surface(face: &Face, cache: &EdgeDiscretizationCache) -> Vec<Point3d> {
    let mut points = Vec::new();

    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate { continue; }

                if let Some(disc) = cache.get(edge.id) {
                    let mut edge_pts = disc.points_3d.clone();
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts.reverse();
                    }
                    points.extend(edge_pts);
                } else {
                    let mut edge_pts = sample_edge_points(edge, EDGE_SAMPLES);
                    let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                    let should_reverse = !coedge.forward != edge_is_reversed;
                    if should_reverse {
                        edge_pts.reverse();
                    }
                    points.extend(edge_pts);
                }
            }
        }
    }

    // Remove duplicate consecutive points
    if !points.is_empty() {
        let mut unique = vec![points[0]];
        for p in &points[1..] {
            if let Some(last) = unique.last() {
                if !last.is_coincident_with(p) {
                    unique.push(*p);
                }
            }
        }
        if unique.len() > 1 {
            if let Some(last) = unique.last() {
                if last.is_coincident_with(&unique[0]) {
                    unique.pop();
                }
            }
        }
        points = unique;
    }

    points
}

/// Fallback tier 1: Approximate the face as a plane and ear-clip.
///
/// Fits a plane through the boundary points using least-squares (SVD),
/// projects the boundary points onto that plane in 2D, and ear-clips
/// the resulting polygon. Handles holes by collecting inner wire points
/// and using earcutr.
///
/// Returns `None` if:
/// - Boundary has fewer than 3 points
/// - Points are collinear (can't form a plane)
/// - Ear-clipping produces no triangles
fn fallback_approximate_plane(face: &Face, boundary_3d: &[Point3d], cache: &EdgeDiscretizationCache) -> Option<TriangleMesh> {
    if boundary_3d.len() < 3 {
        return None;
    }

    // Compute centroid
    let n = boundary_3d.len() as f64;
    let cx = boundary_3d.iter().map(|p| p.x).sum::<f64>() / n;
    let cy = boundary_3d.iter().map(|p| p.y).sum::<f64>() / n;
    let cz = boundary_3d.iter().map(|p| p.z).sum::<f64>() / n;
    let centroid = Point3d::new(cx, cy, cz);

    // Compute covariance matrix for plane fitting via SVD.
    // The eigenvector with the smallest eigenvalue is the plane normal.
    let mut cov_xx = 0.0f64;
    let mut cov_xy = 0.0f64;
    let mut cov_xz = 0.0f64;
    let mut cov_yy = 0.0f64;
    let mut cov_yz = 0.0f64;
    let mut cov_zz = 0.0f64;

    for p in boundary_3d {
        let dx = p.x - cx;
        let dy = p.y - cy;
        let dz = p.z - cz;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_xz += dx * dz;
        cov_yy += dy * dy;
        cov_yz += dy * dz;
        cov_zz += dz * dz;
    }

    // Power iteration to find the smallest eigenvector of the 3x3 covariance matrix.
    // This is the normal of the best-fit plane.
    // We iterate the inverse matrix to converge to the smallest eigenvector.
    // Simpler approach: just find the principal normal from the cross products of
    // the two largest eigenvectors, or use a few iterations of inverse power method.

    // For robustness, use the iterative approach:
    // Start with a guess, apply the covariance matrix repeatedly,
    // and the result converges to the LARGEST eigenvector.
    // Then the normal is perpendicular to the two largest eigenvectors.

    // Simpler: just compute the cross product of two edge vectors to get the normal.
    // This works well for convex polygons and is fast.
    let v1 = Point3d::new(
        boundary_3d[1].x - boundary_3d[0].x,
        boundary_3d[1].y - boundary_3d[0].y,
        boundary_3d[1].z - boundary_3d[0].z,
    );
    // Find a non-collinear edge
    let mut normal = None;
    for i in 2..boundary_3d.len() {
        let v2 = Point3d::new(
            boundary_3d[i].x - boundary_3d[0].x,
            boundary_3d[i].y - boundary_3d[0].y,
            boundary_3d[i].z - boundary_3d[0].z,
        );
        // Cross product
        let nx = v1.y * v2.z - v1.z * v2.y;
        let ny = v1.z * v2.x - v1.x * v2.z;
        let nz = v1.x * v2.y - v1.y * v2.x;
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len > 1e-10 {
            normal = Direction3d::new(nx / len, ny / len, nz / len);
            if normal.is_some() {
                break;
            }
        }
    }

    let normal = match normal {
        Some(n) => n,
        None => {
            // All points are collinear — can't form a plane
            return None;
        }
    };

    // If the points are nearly coplanar, the covariance-based normal is more robust.
    // Check: if the smallest eigenvalue / largest eigenvalue < 0.01, the points are planar.
    // For now, just use the cross-product normal — it's sufficient for the fallback case.

    let plane = Plane::from_origin_and_normal(centroid, normal);
    let surface = Surface::Plane(plane.clone());

    // Project boundary points onto the plane's 2D coordinate system
    let project = |p: &Point3d| -> Point2d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    };

    let points_2d: Vec<Point2d> = boundary_3d.iter().map(|p| project(p)).collect();

    // Collect holes
    let holes_3d: Vec<Vec<Point3d>> = if face.surface.is_some() {
        collect_face_holes_from_cache(face, cache, &surface)
    } else {
        // No surface — try to collect holes from cache using the approximate surface
        collect_face_holes_from_cache(face, cache, &surface)
    };

    let mut mesh = TriangleMesh::new();
    let forward = face.forward;

    if holes_3d.is_empty() {
        // No holes — simple polygon triangulation
        let is_convex = is_convex_polygon(&points_2d);

        if is_convex && boundary_3d.len() >= 3 {
            for p in boundary_3d {
                mesh.add_vertex(*p);
            }
            let n = boundary_3d.len() as u32;
            for i in 1..n - 1 {
                if forward {
                    mesh.add_triangle(0, i, i + 1);
                } else {
                    mesh.add_triangle(0, i + 1, i);
                }
            }
        } else {
            let triangles = ear_clip(&points_2d);
            for p in boundary_3d {
                mesh.add_vertex(*p);
            }
            for tri in &triangles {
                if forward {
                    mesh.add_triangle(tri[0], tri[1], tri[2]);
                } else {
                    mesh.add_triangle(tri[0], tri[2], tri[1]);
                }
            }
        }
    } else {
        // Has holes — use earcutr for better results
        let holes_2d: Vec<Vec<Point2d>> = holes_3d.iter()
            .map(|h| h.iter().map(|p| project(p)).collect())
            .collect();

        match earcutr_triangulate_planar(&points_2d, boundary_3d, &holes_2d, &holes_3d, forward, plane.normal) {
            Some(m) => return Some(m),
            None => {
                // Last resort: merge holes and ear-clip
                let (merged_2d, merged_3d) = merge_holes_into_polygon_planar(
                    &points_2d, boundary_3d, &holes_2d, &holes_3d,
                );
                let triangles = ear_clip(&merged_2d);
                for p in &merged_3d {
                    mesh.add_vertex(*p);
                }
                for tri in &triangles {
                    if forward {
                        mesh.add_triangle(tri[0], tri[1], tri[2]);
                    } else {
                        mesh.add_triangle(tri[0], tri[2], tri[1]);
                    }
                }
            }
        }
    }

    if mesh.triangles.is_empty() {
        None
    } else {
        Some(mesh)
    }
}

/// Fallback tier 2: Fan-triangulate from the centroid of boundary points.
///
/// This is the simplest possible triangulation: connect each boundary
/// edge to the centroid. It works for any face shape but produces
/// degenerate triangles on concave boundaries and does not handle holes.
/// Use only as a last resort before the empty-mesh fallback.
fn fallback_boundary_fan(face: &Face, boundary_3d: &[Point3d]) -> Option<TriangleMesh> {
    if boundary_3d.len() < 3 {
        return None;
    }

    // Compute centroid
    let n = boundary_3d.len() as f64;
    let cx = boundary_3d.iter().map(|p| p.x).sum::<f64>() / n;
    let cy = boundary_3d.iter().map(|p| p.y).sum::<f64>() / n;
    let cz = boundary_3d.iter().map(|p| p.z).sum::<f64>() / n;
    let centroid = crate::edge_cache::deterministic_round_point(Point3d::new(cx, cy, cz));

    let mut mesh = TriangleMesh::new();

    // Add centroid as vertex 0
    mesh.add_vertex(centroid);

    // Add boundary vertices starting from index 1
    for p in boundary_3d {
        mesh.add_vertex(*p);
    }

    let n_pts = boundary_3d.len() as u32;
    let forward = face.forward;

    // Fan triangulation: centroid → boundary[i] → boundary[i+1]
    for i in 0..n_pts {
        let i_next = (i + 1) % n_pts;
        if forward {
            mesh.add_triangle(0, 1 + i, 1 + i_next);
        } else {
            mesh.add_triangle(0, 1 + i_next, 1 + i);
        }
    }

    // Check that we produced valid triangles (non-degenerate)
    let valid_count = mesh.triangles.iter().filter(|tri| {
        tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2]
    }).count();

    if valid_count == 0 {
        None
    } else {
        Some(mesh)
    }
}

/// Fallback tier 3: Sample surface.point_at() on a regular UV grid.
///
/// This is a last-resort strategy for curved surfaces where the boundary
/// triangulation failed. It samples the surface on a regular UV grid,
/// producing an untrimmed mesh that may extend beyond the face boundary.
/// The mesh is approximate but always produces a visible result.
///
/// Returns `None` if the surface produces invalid points (NaN/Inf) on
/// the majority of grid positions.
fn fallback_surface_point_sample(
    face: &Face,
    surface: &Surface,
    boundary_3d: &[Point3d],
    params: &TriangulationParams,
) -> Option<TriangleMesh> {
    // Determine UV range from boundary points via projection
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary_3d {
        let (u, v) = surface.project_point(p);
        if u.is_finite() && v.is_finite() {
            u_min = u_min.min(u);
            u_max = u_max.max(u);
            v_min = v_min.min(v);
            v_max = v_max.max(v);
        }
    }

    if u_min >= u_max || v_min >= v_max {
        // Could not determine a valid UV range
        return None;
    }

    // Add a small margin (5%) to avoid clipping at the boundary
    let du = u_max - u_min;
    let dv = v_max - v_min;
    u_min -= du * 0.05;
    u_max += du * 0.05;
    v_min -= dv * 0.05;
    v_max += dv * 0.05;

    // Use a modest grid resolution for the fallback
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            surface, u_min, u_max, v_min, v_max,
            params.max_deviation * 2.0, // Allow 2× deviation for fallback (rough is OK)
            params.detail_level * 0.5,  // Use lower detail for speed
        )
    } else {
        (params.angular_samples.min(32).max(8), params.height_samples.min(32).max(8))
    };

    let mut mesh = TriangleMesh::new();
    let mut invalid_count = 0usize;
    let total_points = n_u * n_v;

    // Sample the surface on a regular grid
    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = surface.point_at(u, v);

            if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
                invalid_count += 1;
                // Still add a vertex to keep indexing consistent, but use centroid as placeholder
                mesh.add_vertex(crate::edge_cache::deterministic_round_point(Point3d::new(
                    (u_min + u_max) * 0.5,
                    (v_min + v_max) * 0.5,
                    0.0,
                )));
            } else {
                mesh.add_vertex(crate::edge_cache::deterministic_round_point(p));
            }
        }
    }

    // If more than 50% of points are invalid, give up
    if invalid_count > total_points / 2 {
        return None;
    }

    // Generate triangles
    let forward = face.forward;
    for j in 0..n_v - 1 {
        for i in 0..n_u - 1 {
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i + 1) as u32;
            let v2 = ((j + 1) * n_u + i + 1) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    // Filter out degenerate triangles (zero-area)
    mesh.triangles.retain(|tri| {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        let ab = Point3d::new(b.x - a.x, b.y - a.y, b.z - a.z);
        let ac = Point3d::new(c.x - a.x, c.y - a.y, c.z - a.z);
        let cross_z = ab.x * ac.y - ab.y * ac.x;
        let cross_y = ab.x * ac.z - ab.z * ac.x;
        let cross_x = ab.y * ac.z - ab.z * ac.y;
        (cross_x * cross_x + cross_y * cross_y + cross_z * cross_z) > 1e-20
    });

    if mesh.triangles.is_empty() {
        None
    } else {
        Some(mesh)
    }
}

/// Statistics for fallback surface triangulation.
///
/// Tracks how often each fallback tier was used across all faces in a solid,
/// enabling diagnosis of which surface types or geometry patterns cause
/// primary triangulation to fail.
#[derive(Clone, Debug, Default)]
pub struct FallbackStats {
    /// Number of faces that used the ApproximatePlane fallback.
    pub approximate_plane_count: usize,
    /// Number of faces that used the BoundaryFan fallback.
    pub boundary_fan_count: usize,
    /// Number of faces that used the SurfacePointSample fallback.
    pub surface_point_sample_count: usize,
    /// Number of faces where all fallback strategies failed.
    pub all_failed_count: usize,
    /// Total number of faces that triggered fallback (primary returned empty).
    pub total_fallback_count: usize,
}

impl FallbackStats {
    /// Create zero-initialized stats.
    pub fn new() -> Self {
        Self::default()
    }
}
