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
//
// Note: Several legacy helper functions (older `_with_boundary` variants, UV range
// estimators, projection helpers) are retained as reference implementations and
// for use in diagnostic tools. They are marked `#[allow(dead_code)]` below.
#![allow(dead_code)]

use crate::mesh::TriangleMesh;
use crate::edge_cache::EdgeDiscretizationCache;
use draper_geometry::{
    Point3d, Point2d, Direction3d,
    Surface, Plane, CylinderSurface, SphereSurface, TorusSurface,
    ConeSurface, NurbsSurface,
};
use draper_topology::{Face, Wire, Edge, Solid, Shell, Compound};
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

/// Steiner grid budget profile — controls the maximum grid density
/// per face based on the target platform.
///
/// Profiles were introduced after the regression caused by the global
/// caps `n_u ≤ 64, n_v ≤ 32` (commit `276735c`). Those caps were too
/// aggressive for desktop and produced visibly coarse meshes. The
/// profile system restores desktop quality while keeping mobile fast.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SteinerBudgetProfile {
    /// Desktop profile — high-quality grid.
    /// `n_u ≤ 96`, `n_v ≤ 64` for cylinder/cone.
    /// `n_u/n_v ≤ 64` for plane.
    /// Used when running on native or wide-screen WASM (≥ 1024 px).
    Desktop,
    /// Tablet profile — balanced.
    /// `n_u ≤ 64`, `n_v ≤ 32`.
    /// Used for WASM on medium screens (768–1023 px).
    Tablet,
    /// Mobile profile — coarser grid for slower CPUs.
    /// `n_u ≤ 32`, `n_v ≤ 16`.
    /// Used for WASM on narrow screens (< 768 px).
    Mobile,
}

impl SteinerBudgetProfile {
    /// Maximum number of U subdivisions for cylinder/cone Steiner grid.
    pub fn max_u_cyl(&self) -> usize {
        match self {
            Self::Desktop => 96,
            Self::Tablet => 64,
            Self::Mobile => 32,
        }
    }
    /// Maximum number of V subdivisions for cylinder/cone Steiner grid.
    pub fn max_v_cyl(&self) -> usize {
        match self {
            Self::Desktop => 64,
            Self::Tablet => 32,
            Self::Mobile => 16,
        }
    }
    /// Maximum number of U/V subdivisions for planar Steiner grid.
    pub fn max_uv_plane(&self) -> usize {
        match self {
            Self::Desktop => 64,
            Self::Tablet => 48,
            Self::Mobile => 32,
        }
    }
    /// Budget multiplier — how many candidate points to generate
    /// relative to the requested budget. Higher = more filtering work
    /// but better grid structure preservation.
    pub fn candidate_multiplier(&self) -> f64 {
        match self {
            Self::Desktop => 2.0,
            Self::Tablet => 1.5,
            Self::Mobile => 1.25,
        }
    }

    /// Adaptive per-face-area budget multiplier.
    ///
    /// Computes a multiplier for `max_face_triangles` based on the face's
    /// area relative to the total bounding box area of the part. This allows
    /// large faces to receive more Steiner points (up to 2×) while small
    /// faces use fewer (down to 0.5×), preventing budget overflow on parts
    /// with many tiny faces (e.g. fillets on drill_top.stp).
    ///
    /// # Arguments
    /// * `face_area_fraction` — face area / total bbox surface area, in [0, 1].
    ///   If the caller doesn't know the bbox area, pass 1.0 (no adjustment).
    ///
    /// # Returns
    /// Multiplier in [0.5, 2.0]:
    /// - face < 1% of bbox → 0.5× budget (small face, few triangles needed)
    /// - face 1%–25% of bbox → linear interpolation from 0.5× to 1.0×
    /// - face > 25% of bbox → 2.0× budget (large face needs more detail)
    pub fn face_area_budget_multiplier(&self, face_area_fraction: f64) -> f64 {
        // Clamp to valid range
        let f = face_area_fraction.clamp(0.0, 1.0);

        if f < 0.01 {
            // Face area < 1% of bbox → minimal budget
            0.5
        } else if f < 0.25 {
            // Linear interpolation from 0.5 at 1% to 1.0 at 25%
            // t goes from 0.0 (at 1%) to 1.0 (at 25%)
            let t = (f - 0.01) / (0.25 - 0.01);
            0.5 + t * 0.5
        } else {
            // Face area > 25% of bbox → generous budget
            // But cap at 2.0 to prevent excessive tessellation
            // Scale from 1.0 at 25% to 2.0 at 100%
            let t = ((f - 0.25) / 0.75).min(1.0);
            1.0 + t * 1.0
        }
    }
    /// Minimum floor for n_u (cylinder/cone) — prevents visually
    /// broken grids even at extreme LOD downgrades.
    pub fn min_u_cyl(&self) -> usize {
        match self {
            Self::Desktop => 12,
            Self::Tablet => 10,
            Self::Mobile => 8,
        }
    }

    // ── Sphere-specific profile methods ──────────────────────────────
    //
    // Sphere parameterization: u ∈ [0, 2π] (azimuthal), v ∈ [0, π] (polar).
    // Both directions trace great circles of radius R, so the same
    // chord-error formula `d_max = 2·acos(1 - tol/R)` applies to both.
    //
    // Caps mirror the cylinder values (same u period 2π), but v caps
    // are slightly smaller because v_span = π is half of u_span = 2π
    // (so v needs fewer subdivisions for the same chord error).

    /// Maximum number of U subdivisions for sphere Steiner grid.
    /// Same as cylinder (u period is 2π for both).
    pub fn max_u_sphere(&self) -> usize {
        match self {
            Self::Desktop => 96,
            Self::Tablet => 64,
            Self::Mobile => 32,
        }
    }
    /// Maximum number of V subdivisions for sphere Steiner grid.
    /// Slightly smaller than cylinder because v_span = π (half of u_span = 2π).
    pub fn max_v_sphere(&self) -> usize {
        match self {
            Self::Desktop => 64,
            Self::Tablet => 32,
            Self::Mobile => 16,
        }
    }
    /// Minimum floor for n_u (sphere). Same as cylinder.
    pub fn min_u_sphere(&self) -> usize {
        match self {
            Self::Desktop => 12,
            Self::Tablet => 10,
            Self::Mobile => 8,
        }
    }
    /// Minimum floor for n_v (sphere). Smaller than min_u because v_span
    /// is half of u_span — a sphere face needs at least a few polar rows
    /// to look smooth, but not as many as azimuthal.
    pub fn min_v_sphere(&self) -> usize {
        match self {
            Self::Desktop => 8,
            Self::Tablet => 6,
            Self::Mobile => 4,
        }
    }

    // ── Torus-specific profile methods ───────────────────────────────
    //
    // Torus parameterization: u ∈ [0, 2π] (around main ring, radius R),
    // v ∈ [0, 2π] (around tube, radius r). Both directions are periodic.
    //
    // Chord error in u uses radius (R + r) (worst case = outer equator).
    // Chord error in v uses radius r (the tube).
    //
    // The minimum floor for n_u/n_v is 24 on desktop — below this,
    // fillets look "faceted" instead of smooth (per UNIVERSAL_STEP_PLAN
    // task 2.3.2). Mobile/Tablet use lower floors to stay responsive.

    /// Maximum number of U subdivisions for torus Steiner grid.
    /// Same as cylinder/sphere (u period is 2π for all).
    pub fn max_u_torus(&self) -> usize {
        match self {
            Self::Desktop => 96,
            Self::Tablet => 64,
            Self::Mobile => 32,
        }
    }
    /// Maximum number of V subdivisions for torus Steiner grid.
    /// Same as max_v_cyl / max_v_sphere.
    pub fn max_v_torus(&self) -> usize {
        match self {
            Self::Desktop => 64,
            Self::Tablet => 32,
            Self::Mobile => 16,
        }
    }
    /// Minimum floor for n_u (torus). 24 on desktop per plan task 2.3.2
    /// ("min(24, n) — иначе fillet выглядит гранёным").
    pub fn min_u_torus(&self) -> usize {
        match self {
            Self::Desktop => 24,
            Self::Tablet => 20,
            Self::Mobile => 16,
        }
    }
    /// Minimum floor for n_v (torus). Same as min_u_torus — both
    /// directions need at least 24 samples to look smooth.
    pub fn min_v_torus(&self) -> usize {
        self.min_u_torus()
    }

    // ── Revolution-specific profile methods ────────────────────────────
    //
    // RevolutionSurface parameterization: u ∈ [0, 2π] (revolution angle),
    // v ∈ [v_min, v_max] (profile curve parameter). Only u is periodic.
    //
    // n_u uses the maximum revolution radius (max distance from profile
    // to axis) — the same chord-error formula as cylinder.
    // n_v depends on the profile curve type:
    //   - Line → uniform, few subdivisions (2–8)
    //   - Circle/Arc → chord-error with the circle radius
    //   - NURBS/general → adaptive sampling of profile curvature
    //
    // The minimum floor for n_u is 12 on desktop (same as cylinder) to
    // avoid faceted revolution surfaces. n_v minimum is lower (4) since
    // linear profiles need few v-samples.

    /// Maximum number of U subdivisions for revolution Steiner grid.
    /// Same as cylinder (u period is 2π).
    pub fn max_u_revolution(&self) -> usize {
        match self {
            Self::Desktop => 96,
            Self::Tablet => 64,
            Self::Mobile => 32,
        }
    }
    /// Maximum number of V subdivisions for revolution Steiner grid.
    pub fn max_v_revolution(&self) -> usize {
        match self {
            Self::Desktop => 64,
            Self::Tablet => 32,
            Self::Mobile => 16,
        }
    }
    /// Minimum floor for n_u (revolution). 12 on desktop, same as cylinder.
    pub fn min_u_revolution(&self) -> usize {
        match self {
            Self::Desktop => 12,
            Self::Tablet => 10,
            Self::Mobile => 8,
        }
    }
    /// Minimum floor for n_v (revolution). 4 on desktop — linear profiles
    /// need few v-samples, but curved profiles need more. The adaptive
    /// profile sampling will push n_v higher for curved profiles.
    pub fn min_v_revolution(&self) -> usize {
        match self {
            Self::Desktop => 4,
            Self::Tablet => 3,
            Self::Mobile => 2,
        }
    }

    // ── Extrusion-specific profile methods ─────────────────────────────
    //
    // ExtrusionSurface parameterization: u ∈ [u_min, u_max] (profile curve
    // parameter), v ∈ [v_min, v_max] (extrusion distance along direction).
    // Neither direction is periodic.
    //
    // n_u depends on the profile curve type (same logic as revolution):
    //   - Line → uniform, few subdivisions (2–8)
    //   - Circle/Arc → chord-error with the circle radius
    //   - NURBS/general → adaptive sampling of profile curvature
    //
    // n_v is almost always small (2–8) because the extrusion direction
    // is STRAIGHT (dS/dv = D = constant). The surface has zero curvature
    // in v — only the profile contributes curvature.

    /// Maximum number of U subdivisions for extrusion Steiner grid.
    /// Same as revolution (profile-dependent, not angular).
    pub fn max_u_extrusion(&self) -> usize {
        match self {
            Self::Desktop => 64,
            Self::Tablet => 48,
            Self::Mobile => 32,
        }
    }
    /// Maximum number of V subdivisions for extrusion Steiner grid.
    /// Lower than other surfaces — extrusion direction is always straight.
    pub fn max_v_extrusion(&self) -> usize {
        match self {
            Self::Desktop => 16,
            Self::Tablet => 10,
            Self::Mobile => 8,
        }
    }
    /// Minimum floor for n_u (extrusion). 6 on desktop — lower than
    /// revolution because extrusion profiles tend to be simpler.
    pub fn min_u_extrusion(&self) -> usize {
        match self {
            Self::Desktop => 6,
            Self::Tablet => 5,
            Self::Mobile => 4,
        }
    }
    /// Minimum floor for n_v (extrusion). 2 on desktop — extrusion
    /// direction is always straight, so very few v-samples needed.
    pub fn min_v_extrusion(&self) -> usize {
        match self {
            Self::Desktop => 2,
            Self::Tablet => 2,
            Self::Mobile => 2,
        }
    }

    // ── NURBS Steiner budget ──────────────────────────────────
    // NURBS surfaces are the most general case. Unlike analytic
    // surfaces (sphere, torus, cylinder), NURBS have no fixed
    // curvature formula — the grid density depends on degree,
    // control point layout, and knot vector. The budget here is
    // a per-axis CAP: the actual n_u/n_v are derived from
    // `parameter_division_2d` chord-error subdivision, then
    // densified if below the 8×8 minimum for faces with holes.

    /// Maximum number of u-subdivisions for NURBS Steiner grids.
    pub fn max_u_nurbs(&self) -> usize {
        match self {
            Self::Desktop => 96,
            Self::Tablet => 64,
            Self::Mobile => 32,
        }
    }

    /// Maximum number of v-subdivisions for NURBS Steiner grids.
    pub fn max_v_nurbs(&self) -> usize {
        match self {
            Self::Desktop => 96,
            Self::Tablet => 64,
            Self::Mobile => 32,
        }
    }

    /// Minimum number of u-subdivisions for NURBS Steiner grids.
    /// This is only used for the 8×8 densify floor (faces with holes);
    /// bilinear NURBS (deg 1×1) skip the grid entirely.
    pub fn min_u_nurbs(&self) -> usize {
        match self {
            Self::Desktop => 8,
            Self::Tablet => 6,
            Self::Mobile => 4,
        }
    }

    /// Minimum number of v-subdivisions for NURBS Steiner grids.
    pub fn min_v_nurbs(&self) -> usize {
        match self {
            Self::Desktop => 8,
            Self::Tablet => 6,
            Self::Mobile => 4,
        }
    }
}

impl Default for SteinerBudgetProfile {
    fn default() -> Self {
        // Default to Desktop — native builds and tests use this.
        // The WASM viewer overrides via `TriangulationParams::steiner_profile`.
        Self::Desktop
    }
}

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
    /// Steiner grid budget profile — controls max grid density per face.
    /// Defaults to `Desktop`. The WASM viewer sets this to `Mobile` or
    /// `Tablet` based on screen width to keep mobile devices responsive.
    pub steiner_profile: SteinerBudgetProfile,
    /// Target fraction of triangles to KEEP after post-triangulation decimation.
    /// `1.0` = no decimation (keep all triangles).
    /// `0.1` = keep only 10% of triangles (very coarse, ~90% reduction).
    /// `0.0` is invalid (treated as 1.0).
    ///
    /// Decimation is applied as a POST-PROCESS step on the final mesh:
    ///   1. Triangulate the BREP normally (per-face).
    ///   2. Weld shared vertices (so adjacent triangles actually share edges).
    ///   3. Apply shortest-edge-collapse decimation until the target triangle
    ///      count is reached OR no more collapsible edges remain.
    ///
    /// This is what makes "Preview" (LOD 0.1) visibly different from "Ultra"
    /// (LOD 1.0) on files like Zentralstaender.stp — most of whose faces are
    /// planar with linear edges, so per-face triangulation alone produces the
    /// same N-2 triangles regardless of LOD. Decimation collapses coplanar
    /// internal edges, drastically reducing the count for low LOD.
    ///
    /// When `adaptive_lod_enabled` is true, decimation is SKIPPED because
    /// the per-face triangle budget is already controlled via
    /// `target_triangles_per_face` → `max_face_triangles`.
    pub keep_ratio: f64,
    /// Target number of triangles PER FACE when adaptive LOD is enabled.
    ///
    /// When `Some(n)`, each face is triangulated with a budget of ≤ n
    /// triangles, and `max_face_triangles` is capped to this value.
    /// This replaces the old approach of triangulating every face at
    /// full quality and then decimating the combined mesh.
    ///
    /// The budget is computed as:
    ///   target_per_face = (lod × TOTAL_BUDGET) / face_count
    /// where TOTAL_BUDGET = 100 000 for LOD 1.0.
    ///
    /// Small faces naturally use fewer triangles (they have fewer boundary
    /// points), leaving budget for larger, more curved faces.
    ///
    /// When `None`, the original `max_face_triangles` from `for_lod()`
    /// is used (uniform per face, same as before).
    pub target_triangles_per_face: Option<usize>,
    /// Whether adaptive LOD is enabled.
    ///
    /// When `true`:
    /// - `target_triangles_per_face` controls the per-face triangle budget
    ///   (computed from total budget / face count × LOD factor).
    /// - Post-triangulation decimation is SKIPPED (each face already
    ///   respects its budget, so global decimation is unnecessary).
    /// - `max_face_triangles` is capped to `target_triangles_per_face`.
    ///
    /// When `false` (default):
    /// - The original uniform per-face budget is used.
    /// - Post-triangulation decimation is applied when `keep_ratio < 1.0`.
    pub adaptive_lod_enabled: bool,
    /// Total surface area of the bounding box of the part, used for
    /// adaptive per-face-area budget scaling (task 1.1.4).
    ///
    /// When `Some(area)`, each face's `max_face_triangles` is scaled by
    /// `steiner_profile.face_area_budget_multiplier(face_area / area)`.
    /// Large faces (>25% of bbox) get up to 2× budget; small faces (<1%)
    /// get 0.5× budget. This prevents budget overflow on parts with many
    /// tiny faces (e.g. fillets on drill_top.stp) while giving large
    /// curved faces enough Steiner points.
    ///
    /// When `None`, no per-face-area adjustment is applied (uniform budget).
    pub bbox_surface_area: Option<f64>,
    /// Override for the per-BREP wall-clock time limit.
    ///
    /// Defaults to `None` (use platform-specific defaults: 30s WASM, 600s native).
    /// When `Some(duration)`, the BREP session uses this value instead of the
    /// default, allowing tests to simulate timeout scenarios with very short
    /// limits (e.g. 1s).
    pub brep_time_limit_override: Option<std::time::Duration>,
    /// Override for the per-face wall-clock time limit.
    ///
    /// Defaults to `None` (use platform-specific defaults: 3s WASM, 120s native).
    /// When `Some(duration)`, each face gets this time budget instead of the
    /// default, allowing tests to force face-level timeouts.
    pub face_time_limit_override: Option<std::time::Duration>,
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
            .field("steiner_profile", &self.steiner_profile)
            .field("keep_ratio", &self.keep_ratio)
            .field("target_triangles_per_face", &self.target_triangles_per_face)
            .field("adaptive_lod_enabled", &self.adaptive_lod_enabled)
            .field("bbox_surface_area", &self.bbox_surface_area)
            .field("brep_time_limit_override", &self.brep_time_limit_override)
            .field("face_time_limit_override", &self.face_time_limit_override)
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
            steiner_profile: SteinerBudgetProfile::default(),
            keep_ratio: 1.0, // No decimation by default — preserve backward compatibility
            target_triangles_per_face: None,
            adaptive_lod_enabled: false,
            bbox_surface_area: None,
            brep_time_limit_override: None,
            face_time_limit_override: None,
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

        // Post-triangulation decimation ratio (fraction of triangles to KEEP).
        // For very low LODs, decimate aggressively (keep only ~10%).
        // For high LODs (≥ 0.75), no decimation (keep all triangles).
        // In between, interpolate smoothly.
        //
        // The mapping is:
        //   LOD 0.0  → keep 0.05 (very coarse — 5% of triangles)
        //   LOD 0.1  → keep 0.25 (Preview quality)
        //   LOD 0.3  → keep 0.60 (Low quality)
        //   LOD 0.5  → keep 0.85 (Medium quality — light decimation)
        //   LOD 0.75 → keep 1.00 (High quality — no decimation)
        //   LOD 1.0  → keep 1.00 (Ultra quality — no decimation)
        //
        // This is what makes "Preview" visibly different from "Ultra" on
        // STEP files whose faces are predominantly planar with linear edges
        // (where per-face triangulation alone gives the same N-2 triangles
        // regardless of LOD).
        let keep_ratio = if lod >= 0.75 {
            1.0 // No decimation for high LOD
        } else {
            // Linear interpolation from (lod=0.0, keep=0.05) to (lod=0.75, keep=1.0)
            // keep = 0.05 + (lod / 0.75) * 0.95
            (0.05 + (lod / 0.75) * 0.95).clamp(0.05, 1.0)
        };

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
            steiner_profile: SteinerBudgetProfile::default(),
            keep_ratio,
            target_triangles_per_face: None,
            adaptive_lod_enabled: false,
            bbox_surface_area: None,
            brep_time_limit_override: None,
            face_time_limit_override: None,
        }
    }

    /// Total triangle budget at a given LOD level.
    ///
    /// At LOD 1.0 the total budget is 100 000 triangles for the entire BREP.
    /// This scales linearly with LOD: LOD 0.5 → 50 000, LOD 0.3 → 30 000.
    ///
    /// This constant was chosen so that a typical CAD assembly (drill_top.stp,
    /// ~2971 faces) produces ~33 triangles per face at LOD 1.0 — enough
    /// for good visual quality on curved surfaces while keeping the total
    /// count under 100K for GPU efficiency.
    pub const TOTAL_TRIANGLE_BUDGET: usize = 100_000;

    /// Compute the per-face triangle budget for adaptive LOD.
    ///
    /// The formula is:
    /// ```ignore
    /// target_per_face = (lod × TOTAL_TRIANGLE_BUDGET) / face_count
    /// ```
    ///
    /// This is clamped to at least 4 triangles (minimum for a visible face)
    /// and at most `max_face_triangles` (the hard ceiling from the LOD preset).
    ///
    /// When `adaptive_lod_enabled` is false, returns `None`.
    pub fn compute_target_triangles_per_face(&self, face_count: usize) -> Option<usize> {
        if !self.adaptive_lod_enabled || face_count == 0 {
            return None;
        }
        let total_budget = (self.detail_level * Self::TOTAL_TRIANGLE_BUDGET as f64).round() as usize;
        let per_face = (total_budget / face_count).max(4);
        // Don't exceed the hard ceiling from for_lod()
        let capped = per_face.min(self.max_face_triangles);
        Some(capped)
    }

    /// Enable adaptive LOD, computing the per-face budget from face count.
    ///
    /// This replaces the old approach of triangulating every face at full
    /// quality and then decimating the combined mesh. Instead, each face
    /// gets a fair share of the total triangle budget, proportional to
    /// `detail_level` (which scales with LOD).
    ///
    /// After calling this:
    /// - `adaptive_lod_enabled` = `true`
    /// - `target_triangles_per_face` = `Some(budget)`
    /// - `max_face_triangles` is capped to `target_triangles_per_face`
    /// - `keep_ratio` is set to 1.0 (no decimation — each face already
    ///   respects its budget)
    pub fn with_adaptive_lod(&mut self, face_count: usize) {
        if face_count == 0 {
            return;
        }
        self.adaptive_lod_enabled = true;
        let total_budget = (self.detail_level * Self::TOTAL_TRIANGLE_BUDGET as f64).round() as usize;
        let per_face = (total_budget / face_count).max(4);
        // Don't exceed the hard ceiling from for_lod()
        let budget = per_face.min(self.max_face_triangles);
        self.target_triangles_per_face = Some(budget);
        self.max_face_triangles = budget;
        // No decimation needed — each face already respects its budget
        self.keep_ratio = 1.0;
        log::info!(
            "Adaptive LOD: {} faces × {} tris/face = {} total budget (LOD {:.2})",
            face_count, budget, face_count * budget, self.detail_level
        );
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
    // Override the chord tolerance with the LOD-driven max_deviation so that
    // curved edges (arcs on cylinder caps, fillets, etc.) get coarser or finer
    // subdivision depending on the user's Quality selection. Without this
    // override, the chord tolerance is derived from the bounding box and gives
    // a fixed, very fine subdivision regardless of LOD — making the Quality
    // selector appear to have no effect on cylinder caps (which look perfectly
    // circular at all LODs because their boundary arcs are always sampled
    // with ~50+ points).
    cache.set_chord_tolerance_override(Some(params.max_deviation));
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

    // Phase 2.2: If the primary triangulation produced an empty mesh OR
    // a mesh with vertices but no triangles (which can happen when earcutr
    // cannot triangulate a degenerate UV polygon and the 3D ear-clip fallback
    // also fails), apply fallback strategies in order of decreasing quality.
    //
    // CRITICAL: We check `triangles.is_empty()`, not just `vertices.is_empty()`.
    // A mesh with vertices but 0 triangles leaves a hole in the BREP that no
    // weld pass can fix — this was the root cause of Zentralstaender.stp
    // leaky watertightness (5.9% boundary edges).
    if !primary_mesh.vertices.is_empty() && !primary_mesh.triangles.is_empty() {
        return primary_mesh;
    }

    // Log if primary mesh had vertices but no triangles (diagnostic)
    if !primary_mesh.vertices.is_empty() && primary_mesh.triangles.is_empty() {
        log::warn!(
            "FallbackSurface: face {} primary triangulation produced {} vertices but 0 triangles — trying fallback strategies",
            face.id, primary_mesh.vertices.len(),
        );
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
    _surface: &Surface,
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
    _surface: &Surface,
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

    // ============================================================
    // CCW normalization (same as parametric_domain.rs Step 1.25)
    //
    // When `forward:false`, the boundary coedges are reversed,
    // producing a CW (clockwise) UV polygon. earcutr interprets
    // CW input as a hole and produces CW triangles. The subsequent
    // `forward:false` winding swap (tri[0],tri[2],tri[1]) then
    // over-corrects, resulting in normals pointing outward when
    // they should point inward.
    //
    // Fix: Detect CW polygons via signed area and reverse them to CCW.
    // This ensures earcutr always receives CCW input and produces
    // CCW triangles, so the `forward` flag's winding swap is the
    // only correction needed. The 3D boundary points must be
    // reversed in sync to maintain UV↔3D correspondence.
    // ============================================================
    let (outer_2d, outer_3d, holes_2d, holes_3d): (Vec<Point2d>, Vec<Point3d>, Vec<Vec<Point2d>>, Vec<Vec<Point3d>>) = {
        let mut area = 0.0_f64;
        for i in 0..outer_2d.len() {
            let j = (i + 1) % outer_2d.len();
            area += outer_2d[i].u * outer_2d[j].v - outer_2d[j].u * outer_2d[i].v;
        }
        let signed_area = area * 0.5;
        if signed_area < 0.0 {
            log::info!(
                "CCW normalization (planar): UV polygon has negative signed area ({:.6}) — reversing to CCW (forward={})",
                signed_area, forward
            );
            let mut rev_2d: Vec<Point2d> = outer_2d.to_vec();
            rev_2d.reverse();
            let mut rev_3d: Vec<Point3d> = outer_3d.to_vec();
            rev_3d.reverse();
            let mut rev_holes_2d = Vec::with_capacity(holes_2d.len());
            let mut rev_holes_3d = Vec::with_capacity(holes_3d.len());
            for h2d in holes_2d.iter() {
                let mut h = h2d.to_vec();
                h.reverse();
                rev_holes_2d.push(h);
            }
            for h3d in holes_3d.iter() {
                let mut h = h3d.to_vec();
                h.reverse();
                rev_holes_3d.push(h);
            }
            (rev_2d, rev_3d, rev_holes_2d, rev_holes_3d)
        } else {
            (outer_2d.to_vec(), outer_3d.to_vec(), holes_2d.to_vec(), holes_3d.to_vec())
        }
    };

    let mut mesh = TriangleMesh::new();

    // Build the flat coordinate array for earcutr.
    // Layout: [outer_pts...][hole0_pts...][hole1_pts...]
    // earcutr expects coordinates as [x0,y0, x1,y1, ...] (2D flat)
    let mut coords: Vec<f64> = Vec::with_capacity((outer_2d.len() + holes_2d.iter().map(|h| h.len()).sum::<usize>()) * 2);
    let mut hole_indices: Vec<usize> = Vec::with_capacity(holes_2d.len());

    // Outer boundary points
    for p in &outer_2d {
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

    // Run triangulation via adapter (tries earcut int, i_triangle, earcutr)
    let triangle_indices = crate::earcut_adapter::triangulate_polygon_with_holes(&coords, &hole_indices);

    if triangle_indices.is_empty() {
        return None;
    }

    // Build combined 3D vertex array: outer vertices first, then valid hole vertices
    // CRITICAL: Only include holes that were also added to coords, so 3D indices
    // match earcutr's 2D indices exactly.
    let mut all_3d: Vec<Point3d> = outer_3d.clone();
    for &hi in &valid_hole_indices {
        all_3d.extend_from_slice(&holes_3d[hi]);
    }

    // Verify that all triangle indices are within bounds
    let n_verts = coords.len() / 2;
    for &idx in &triangle_indices {
        if idx >= n_verts {
            log::warn!("earcut adapter produced out-of-bounds index {} (max {})", idx, n_verts - 1);
            return None;
        }
    }

    // Add vertices and triangles to the mesh
    for p in &all_3d {
        mesh.add_vertex(*p);
    }

    // The adapter produces triangles as [i0, i1, i2, i0, i1, i2, ...]
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
        // No outer wire → try to recover boundary points directly from face.edges.
        //
        // This happens for `make_cylinder()`'s lateral face, which has
        // `edges = [bottom_circle, top_circle]` but `outer_wire = Wire::new(vec![])`.
        // Without this recovery, we'd fall through to `triangulate_cylinder_full`
        // which generates its OWN regular grid (n_u points uniformly spaced in u),
        // giving the lateral face a DIFFERENT number of ring vertices than the
        // adjacent cap faces (which use the edge cache's adaptively-sampled points).
        //
        // The mismatch (e.g. 6 lateral ring pts vs 19 cap ring pts at LOD 0.1)
        // produces a non-watertight mesh with ~50 boundary edges and a
        // "crumpled, inconsistent" appearance.
        //
        // By recovering the cached edge points here, we ensure the lateral face
        // SHARES its bottom/top ring vertices with the cap faces → watertight.
        let mut direct_boundary: Vec<Point3d> = Vec::new();
        for edge in &face.edges {
            if edge.degenerate { continue; }
            if let Some(disc) = cache.get(edge.id) {
                direct_boundary.extend(disc.points_3d.iter().cloned());
            }
        }
        if !direct_boundary.is_empty() {
            // Deduplicate consecutive coincident points (the edge cache may
            // store the seam point at both t=0 and t=2π for a closed circle).
            let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
            let mut dedup: Vec<Point3d> = Vec::with_capacity(direct_boundary.len());
            for p in &direct_boundary {
                if let Some(last) = dedup.last() {
                    if last.distance_to(p) <= adaptive_tol { continue; }
                }
                dedup.push(*p);
            }
            // Also drop last if it coincides with first (closed loop)
            if dedup.len() > 1 {
                if let Some(last) = dedup.last() {
                    if last.distance_to(&dedup[0]) <= adaptive_tol {
                        dedup.pop();
                    }
                }
            }
            if dedup.len() >= 6 {
                log::debug!(
                    "Cylinder face #{}: no outer_wire but {} edge points recovered from cache — using tube grid triangulation",
                    face.id, dedup.len()
                );
                return triangulate_cylinder_tube_from_boundary(cyl, params, &dedup, face.forward);
            }
        }
        // No edges at all — sample full cylinder from analytic surface
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

    // Detect PARTIAL-wrap tube face: boundary has 2 arc edges (at v_min and
    // v_max) + 2 seam line edges (at u_start and u_end), covering <2π of
    // angular range. earcutr produces TWISTED triangles for these because
    // the UV polygon self-intersects when the top arc is parameterized to
    // go around the "long way" (e.g., from u=0 through u=3π/2 to u=π,
    // instead of the "short way" through u=π/2).
    //
    // Detection: no holes, both bottom_ring and top_ring have ≥3 points.
    // Triangulation: `triangulate_cylinder_tube_from_boundary` handles both
    // full and partial wrap (uses `is_full_wrap` to decide wrap-around).
    if hole_polylines.is_empty() && boundary_3d.len() >= 6 {
        let (v_min_pw, v_max_pw) = compute_axis_v_range_pts(&boundary_3d, &cyl.origin, &cyl.axis);
        if v_max_pw > v_min_pw {
            let dedup_tol = params.max_deviation.max(1e-6) * 0.5;
            let (bottom_ring, top_ring, _) = split_boundary_into_rings_with_u(
                &boundary_3d, &cyl.origin, &cyl.axis, &cyl.x_dir,
                v_min_pw, v_max_pw, dedup_tol,
            );
            if bottom_ring.len() >= 3 && top_ring.len() >= 3 {
                log::info!(
                    "Cylinder face #{}: PARTIAL tube face detected ({} bottom + {} top ring points, {} bnd pts) — using tube grid triangulation",
                    face.id, bottom_ring.len(), top_ring.len(), boundary_3d.len()
                );
                return triangulate_cylinder_tube_from_boundary(cyl, params, &boundary_3d, face.forward);
            }
        }
    }

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
    let mut second_max_gap = 0.0f64;
    for i in 0..n {
        let next = if i + 1 < n { us[i + 1] } else { us[0] + u_period };
        let gap = next - us[i];
        if gap > max_gap {
            second_max_gap = max_gap;
            max_gap = gap;
        } else if gap > second_max_gap {
            second_max_gap = gap;
        }
    }
    let wrapped_range = u_period - max_gap;
    // Full-range threshold: ≥90% of u_period. See
    // split_boundary_into_rings_with_u for the rationale.
    let is_full_range = wrapped_range >= u_period * 0.90;
    // Uniform-gap threshold using second-max gap as the arc-step reference.
    // See split_boundary_into_rings_with_u for the rationale.
    let seam_to_arc_ratio = if second_max_gap > 1e-9 { max_gap / second_max_gap } else { 1.0 };
    let is_uniform_gap = seam_to_arc_ratio <= 1.5;
    let is_full = is_full_range && is_uniform_gap;
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
/// CRITICAL for watertightness AND correct visual geometry:
/// 1. The bottom and top ring vertices are taken DIRECTLY from the cached
///    boundary 3D points (shared with adjacent plane faces via the edge
///    cache), so the mesh has bit-identical vertices on shared edges.
/// 2. The intermediate grid rows are sampled at the SAME u angles as the
///    bottom ring (using `cyl.point_at(bottom_ring.u[i], v)`), NOT at
///    uniform `2π*i/n_u` spacing. This ensures each quad of the grid
///    connects vertices at matching angular positions, producing a clean
///    cylindrical tube instead of a "twisted/lobed" appearance.
///
/// Bug history: The previous implementation sorted the boundary ring by
/// `atan2` (which returns values in (-π, π]) and then sampled intermediate
/// rows at uniform `u = 2π*i/n_u` starting from u=0. The sorted ring's
/// first element was at angle ≈ -π (i.e., u ≈ π), while the intermediate
/// row's first element was at angle 0. This ~π angular offset caused every
/// triangle to span ~180° of the cylinder, producing the "twisted, lobed,
/// spiked" appearance reported by the user.
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

    // Split boundary into bottom/top rings, each as a sorted Vec<(u, Point3d)>
    // with u in [0, 2π) (full-wrap) or unwrapped to a continuous range
    // [u_start, u_end] (partial-wrap). `is_full_wrap` indicates which.
    let dedup_tol = params.max_deviation.max(1e-6) * 0.5;
    let (bottom_ring, top_ring, is_full_wrap) = split_boundary_into_rings_with_u(
        boundary_3d, &cyl.origin, &cyl.axis, &cyl.x_dir, v_min, v_max, dedup_tol,
    );

    if bottom_ring.len() < 3 {
        // Not enough points for a proper ring — fall back to analytic full
        // triangulation. This should not happen for well-formed BREP solids
        // but is a safety net for degenerate inputs.
        log::warn!(
            "Cylinder tube face has only {} bottom ring points (need ≥3) — falling back to analytic full triangulation",
            bottom_ring.len()
        );
        return triangulate_cylinder_full_at_v_range(cyl, params, v_min, v_max, forward);
    }

    let n_u = bottom_ring.len();
    let bottom_u: Vec<f64> = bottom_ring.iter().map(|(u, _)| *u).collect();

    // Use cached top ring points when available (same count as bottom).
    // See cone tube function for full rationale.
    let use_cached_top = top_ring.len() == n_u;

    // Compute n_v from adaptive sampling (height direction only — u direction
    // is determined by the cached ring point count, not adaptive sampling).
    let n_v = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cylinder(cyl.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        ).1.max(2)
    } else {
        params.height_samples.max(2)
    };

    // When using cached top ring points, ALWAYS force n_v=1.
    // See cone tube function for full rationale.
    let n_v = if use_cached_top && top_ring.len() == n_u {
        1
    } else {
        n_v
    };

    log::debug!(
        "Cylinder tube face: n_u={}, n_v={}, full_wrap={}, use_cached_top={}, v=[{:.3},{:.3}]",
        n_u, n_v, is_full_wrap, use_cached_top, v_min, v_max
    );

    // Generate vertices: n_v+1 rows × n_u columns.
    // - j=0 (bottom): use cached bottom_ring points (shared with bottom cap face).
    // - j=n_v (top): use cached top_ring points (if use_cached_top) — shared
    //   with top cap face. Otherwise, analytic cyl.point_at(bottom_u[i], v_max).
    // - 0 < j < n_v (intermediate): analytic cyl.point_at(bottom_u[i], v_j).
    //   Using bottom_u[i] (not uniform 2π*i/n_u) ensures each column of the
    //   grid is at a single angular position, producing clean rectangular
    //   quads faces instead of twisted/spiraling triangles.
    //
    // IMPORTANT: for partial-wrap faces, bottom_u[i] is in unwrapped range
    // [u_start, u_end] (possibly > 2π). cyl.point_at(u, v) is periodic in u,
    // so passing u > 2π is fine — it produces the same point as u mod 2π.
    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = bottom_u[i];
            let p = if j == 0 {
                bottom_ring[i].1
            } else if j == n_v && use_cached_top {
                top_ring[i].1
            } else {
                crate::edge_cache::deterministic_round_point(cyl.point_at(u, v))
            };
            let n = orient_normal(cyl.normal_at(u, v), forward);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // Generate triangles.
    // v0 = (j, i), v1 = (j, i+1), v2 = (j+1, i+1), v3 = (j+1, i).
    // Forward winding: (v0,v1,v2)+(v0,v2,v3) → CCW from outside.
    //
    // For FULL-wrap: i_next = (i+1) % n_u (wrap around to close the tube).
    // Produces n_u quads per row.
    //
    // For PARTIAL-wrap: i_next = i+1, NO wrap (last column has no neighbor
    // to the right — that's where the seam line lives, shared with the
    // adjacent half-cylinder face). Produces n_u-1 quads per row.
    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = if is_full_wrap {
                (i + 1) % n_u
            } else {
                if i + 1 >= n_u { continue; }
                i + 1
            };
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
/// bottom (v=v_min) and top (v=v_max), each as a `Vec<(u, Point3d)>`.
///
/// For each point we compute:
/// - `v` = projection onto the axis (used to classify into bottom/top ring).
/// - `u` = angular position around the axis, in `[0, 2π)`, measured CCW from
///   `x_dir`. This u is consistent with `Cylinder::point_at(u, v)` and
///   `Cone::point_at(u, v)` parameterizations, so points generated by
///   `surface.point_at(bottom_u[i], v)` align perfectly with `bottom_ring[i]`.
///
/// Each ring is sorted by u ascending in [0, 2π). Consecutive coincident
/// points (e.g., seam duplicates where the cache stores the same point at
/// both u=0 and u=2π) are removed, as is the wraparound duplicate (last
/// point coincident with first).
fn split_boundary_into_rings_with_u(
    boundary_3d: &[draper_geometry::Point3d],
    origin: &draper_geometry::Point3d,
    axis: &draper_geometry::Direction3d,
    x_dir: &draper_geometry::Direction3d,
    v_min: f64,
    v_max: f64,
    dedup_tol: f64,
) -> (
    Vec<(f64, draper_geometry::Point3d)>,
    Vec<(f64, draper_geometry::Point3d)>,
    bool,
) {
    let y_dir = axis.cross(x_dir);
    let v_tol = (v_max - v_min).abs() * 0.05 + 1e-9;

    let mut bottom: Vec<(f64, draper_geometry::Point3d)> = Vec::new();
    let mut top: Vec<(f64, draper_geometry::Point3d)> = Vec::new();

    for p in boundary_3d {
        let dx = p.x - origin.x;
        let dy = p.y - origin.y;
        let dz = p.z - origin.z;
        let v = dx * axis.x + dy * axis.y + dz * axis.z;
        let x_comp = dx * x_dir.x + dy * x_dir.y + dz * x_dir.z;
        let y_comp = dx * y_dir.x + dy * y_dir.y + dz * y_dir.z;
        // atan2 returns (-π, π]; rem_euclid(2π) maps to [0, 2π).
        let u = y_comp.atan2(x_comp).rem_euclid(2.0 * PI);

        if (v - v_min).abs() <= v_tol {
            bottom.push((u, *p));
        } else if (v - v_max).abs() <= v_tol {
            top.push((u, *p));
        }
        // else: point is at intermediate v (e.g. seam midpoint) — skip
    }

    // Sort by u ascending in [0, 2π).
    bottom.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    top.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Remove consecutive coincident points (e.g., seam at u=0 and u=2π).
    let dedup_consecutive = |ring: &mut Vec<(f64, draper_geometry::Point3d)>| {
        if ring.len() < 2 {
            return;
        }
        let mut i = 1;
        while i < ring.len() {
            if ring[i - 1].1.distance_to(&ring[i].1) <= dedup_tol {
                ring.remove(i);
            } else {
                i += 1;
            }
        }
        // Also check wraparound: last vs first (closed loop)
        if ring.len() > 1 {
            if ring[0].1.distance_to(&ring[ring.len() - 1].1) <= dedup_tol {
                ring.pop();
            }
        }
    };
    dedup_consecutive(&mut bottom);
    dedup_consecutive(&mut top);

    // ============================================================
    // Detect full vs partial U-period wrap and unwrap u values.
    //
    // For a FULL-wrap face (boundary covers ≈2π of angular range), the
    // bottom_ring and top_ring points are roughly uniformly distributed
    // in [0, 2π). The largest gap between consecutive sorted u values
    // (including wraparound from last to first+2π) is small (≈2π/n_u).
    // We keep u values in [0, 2π) and use `(i+1) % n_u` for triangle
    // winding to close the loop.
    //
    // For a PARTIAL-wrap face (boundary covers <2π, e.g., a half-cylinder
    // face from a STEP file with 2 half-circle arcs + 2 seam lines), the
    // u values cluster in [u_start, u_end] with u_end - u_start < 2π. The
    // largest gap is the "missing" angular range = 2π - (u_end - u_start).
    //
    // For partial-wrap, we must UNWRAP u values to a continuous range
    // [u_start, u_end] (or [u_start, u_end + 2π] if the cluster wraps
    // around the u=0 seam). This is done by shifting points on the "low"
    // side of the largest gap by +2π, so all u values become monotonic.
    //
    // We then use `i+1` (no mod) for triangle winding, producing n_u-1
    // quads per row instead of n_u (no wraparound edge — that's where
    // the seam lines live).
    //
    // Threshold: full-wrap requires BOTH:
    //   (1) wrapped_range = 2π - max_gap >= 99% of 2π (covers ≈ entire period).
    //   (2) max_gap ≤ 2 × mean_gap (gap uniformity — partial-wrap has a
    //       distinctly larger seam gap than arc step gaps).
    //
    // The previous single-threshold `max_gap ≤ π/4 (45°)` was too loose for
    // 351°-wrap STEP faces with n_u=64: the seam gap ≈ 0.157 rad (9°) is
    // well below π/4, so the face was misclassified as full-wrap, causing
    // wrap-link triangles between the two seam endpoints across the missing
    // 9° gap — visible as twisted/spike artifacts on cone/cylinder lateral
    // faces (test/3.05.078.stp faces #78, #84 reported by user).
    // ============================================================
    let mut all_us: Vec<f64> = bottom.iter().map(|(u, _)| *u).collect();
    all_us.extend(top.iter().map(|(u, _)| *u));
    if all_us.is_empty() {
        return (bottom, top, false);
    }
    all_us.sort_by(|a, b| a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal));

    let n_all = all_us.len();
    let mut max_gap = 0.0f64;
    let mut second_max_gap = 0.0f64;
    let mut gap_idx = 0usize;
    for i in 0..n_all {
        let next = if i + 1 < n_all { all_us[i + 1] } else { all_us[0] + 2.0 * PI };
        let gap = next - all_us[i];
        if gap > max_gap {
            second_max_gap = max_gap;
            max_gap = gap;
            gap_idx = i;
        } else if gap > second_max_gap {
            second_max_gap = gap;
        }
    }
    let wrapped_range = 2.0 * PI - max_gap;
    // Full-range threshold: ≥90% of 2π. Lower than 99% because arc sampling
    // excludes the last point (sample_edge_points uses i in 0..n, so the
    // last sample is at i=n-1, not i=n). For a primitive cylinder with
    // n_u=19, the wrapped range is 18/19 × 2π ≈ 94.7%, which is below 99%
    // but above 90%.
    let is_full_range = wrapped_range >= 2.0 * PI * 0.90;
    // Uniform-gap threshold using SECOND-MAX gap as the "arc step" reference.
    // For full-wrap: max_gap ≈ second_max_gap (all gaps uniform), ratio ≈ 1.
    // For partial-wrap: max_gap is the seam gap, second_max_gap is the arc
    //   step. Seam gap > arc step, ratio > 1 (typically ≥1.5 for STEP files
    //   with partial-wrap faces like 351°-wrap cones).
    // We use ratio ≤ 1.5 as the full-wrap threshold.
    //
    // Why not use mean_gap? Because `all_us` contains BOTH bottom_ring and
    // top_ring u values, which may be duplicated (same u for both rings).
    // Duplicates make mean_gap smaller than the true arc step, inflating
    // max_gap / mean_gap ratio. Using second_max_gap avoids this — the
    // second-largest gap is the largest "non-seam" gap, which is the arc
    // step regardless of duplicates.
    let seam_to_arc_ratio = if second_max_gap > 1e-9 { max_gap / second_max_gap } else { 1.0 };
    let is_uniform_gap = seam_to_arc_ratio <= 1.5;
    let is_full_wrap = is_full_range && is_uniform_gap;
    log::debug!(
        "split_boundary wrap detection: n_all={}, gap_idx={}, max_gap={:.4}, second_max_gap={:.4}, ratio={:.2}, wrapped_range={:.4} ({:.1}%), is_full_range={}, is_uniform_gap={}, is_full_wrap={}",
        n_all, gap_idx, max_gap, second_max_gap, seam_to_arc_ratio,
        wrapped_range, 100.0 * wrapped_range / (2.0 * PI),
        is_full_range, is_uniform_gap, is_full_wrap
    );

    if !is_full_wrap {
        // Partial-wrap: unwrap u values to a continuous range.
        //
        // The largest gap is between `all_us[gap_idx]` and `all_us[gap_idx+1]`
        // (or between `all_us[n-1]` and `all_us[0]+2π` when `gap_idx == n-1`).
        //
        // Case A — gap in the MIDDLE (gap_idx < n_all-1):
        //   u values in [all_us[0], all_us[gap_idx]] are on the "low" side
        //   of the gap. Shift them by +2π so all u values become
        //   monotonically increasing in [all_us[gap_idx+1], all_us[gap_idx]+2π].
        //
        // Case B — gap at the END (gap_idx == n_all-1):
        //   u values are already sorted and continuous in
        //   [all_us[0], all_us[n_all-1]]. No unwrap needed.
        //   (Previous code incorrectly shifted ALL points by +2π here,
        //    producing a range like [0.083+2π, 6.283+2π] which broke
        //    triangulation for 351°-wrap cone faces.)
        if gap_idx < n_all - 1 {
            let u_threshold = all_us[gap_idx];
            for (u, _) in bottom.iter_mut() {
                if *u <= u_threshold {
                    *u += 2.0 * PI;
                }
            }
            for (u, _) in top.iter_mut() {
                if *u <= u_threshold {
                    *u += 2.0 * PI;
                }
            }
            // Re-sort by unwrapped u.
            bottom.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            top.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        }
    }

    (bottom, top, is_full_wrap)
}

/// Analytic fallback: triangulate a full cylinder between v_min and v_max
/// using a uniform `2π*i/n_u` angular grid. Used when `triangulate_cylinder_tube_from_boundary`
/// can't recover enough cached boundary points (e.g., degenerate input).
///
/// `n_u` is chosen as `max(params.angular_samples, 8)`; `n_v` is chosen as
/// `max(params.height_samples, 2)` or the adaptive sample count.
fn triangulate_cylinder_full_at_v_range(
    cyl: &CylinderSurface,
    params: &TriangulationParams,
    v_min: f64,
    v_max: f64,
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cylinder(cyl.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples.max(8), params.height_samples.max(2))
    };

    for j in 0..=n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            let p = crate::edge_cache::deterministic_round_point(cyl.point_at(u, v));
            let n = orient_normal(cyl.normal_at(u, v), forward);
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

/// Triangulate a cone face whose boundary forms a closed tube
/// (bottom circle + top circle/apex + 2 seam edges wrapping the full U period).
///
/// Same angular-alignment fix as `triangulate_cylinder_tube_from_boundary`:
/// intermediate grid rows are sampled at the bottom ring's actual u angles
/// (not uniform 2π*i/n_u), so each column of the grid is at a single angular
/// position. This avoids the "twisted/lobed" bug.
///
/// Apex degeneracy: when v_max reaches the cone's apex height, the top row
/// Negate a surface normal for `forward:false` faces (Bug B fix — 8.2.1/8.2.2).
/// For `forward:false`, the geometric outward normal must be flipped so it
/// points inward (toward the solid), matching the inverted triangle winding.
#[inline]
fn orient_normal(n: Direction3d, forward: bool) -> Direction3d {
    if forward {
        n
    } else {
        Direction3d::new(-n.x, -n.y, -n.z).unwrap_or(n)
    }
}

/// collapses to a single apex vertex (since `cone.point_at(u, apex_v)` is
/// the apex point for any u). Triangles connecting the bottom row to the
/// apex row are fan-triangulated.
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
        // All boundary points at the same v — use the full cone range.
        // With STEP parameterization, apex is at v_apex = -radius/tan(ha).
        // For a standard cone tube from base (v=0) to apex (v=apex_v):
        let apex_v = cone.apex_v();
        if cone.expanding {
            v_min = 0.0;
            v_max = cone.height().min(100.0);
        } else {
            v_min = apex_v;
            v_max = 0.0;
        }
    }

    // Clamp v range to exclude the apex (where radius = 0).
    // With STEP parameterization (r = radius + v*tan(ha)), the apex is at
    // v_apex = -radius/tan(ha) (negative for standard cones). Clamp v_min
    // to not go past the apex. For expanding cones, apex is at v=0.
    let apex_v = cone.apex_v();
    if !cone.expanding && apex_v.is_finite() {
        v_min = v_min.max(apex_v);
    }
    let top_row_at_apex = !cone.expanding && apex_v.is_finite() && (v_min - apex_v).abs() < apex_v.abs() * 0.01 + 1e-6;

    // Split boundary into bottom/top rings, each as a sorted Vec<(u, Point3d)>
    // with u in [0, 2π) (full-wrap) or unwrapped to a continuous range
    // [u_start, u_end] (partial-wrap). `is_full_wrap` indicates which.
    let dedup_tol = params.max_deviation.max(1e-6) * 0.5;
    let (bottom_ring, top_ring, is_full_wrap) = split_boundary_into_rings_with_u(
        boundary_3d, &cone.origin, &cone.axis, &cone.x_dir, v_min, v_max, dedup_tol,
    );

    // When bottom ring is empty but top ring has points (boundary is at v_max,
    // apex is at v_min), swap the direction so the boundary ring becomes the
    // "bottom" and the apex is at the "top" of the iteration.
    let (effective_bottom, effective_top, _effective_v_min, _effective_v_max, _swapped) =
        if bottom_ring.len() < 3 && top_ring.len() >= 3 {
            // Boundary points are at v_max (top), apex at v_min (bottom).
            // Swap: use top_ring as bottom, generate toward apex at v_min.
            (top_ring.clone(), bottom_ring.clone(), v_min, v_max, true)
        } else {
            (bottom_ring.clone(), top_ring.clone(), v_min, v_max, false)
        };

    if effective_bottom.len() < 3 {
        log::warn!(
            "Cone tube face has only {} ring points (need ≥3) — falling back to analytic full triangulation",
            effective_bottom.len()
        );
        return triangulate_cone_full_at_v_range(cone, params, v_min, v_max, forward);
    }

    let n_u = effective_bottom.len();
    let bottom_u: Vec<f64> = effective_bottom.iter().map(|(u, _)| *u).collect();

    // Use cached top ring points when available (same count as bottom).
    // When the two rings have different angular samplings (different CIRCLE
    // entities with different radii), this produces slightly non-rectangular
    // quads, but the vertices match the edge cache → watertight by construction.
    // The alternative (interpolating top ring to bottom ring's angles) would
    // produce rectangular quads but break watertightness (interpolated points
    // don't match edge cache points).
    let use_cached_top = !top_row_at_apex && effective_top.len() == n_u;

    // Compute n_v from adaptive sampling (height direction only).
    let n_v = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cone(cone.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        ).1.max(2)
    } else {
        params.height_samples.max(2)
    };

    // When using cached top ring points, ALWAYS force n_v=1 (only 2 rows:
    // bottom and top, no intermediate analytic row).
    //
    // For cone/cylinder surfaces, the axial direction is straight (zero chord
    // error), so n_v=1 is always sufficient — there's no geometric benefit
    // from intermediate rows. Using n_v=1 produces cleaner triangulation
    // that matches what other CAD applications produce: each quad connects
    // a bottom ring vertex directly to the top ring vertex at the same
    // angular position, creating a clean developable surface.
    //
    // Using n_v>1 adds an analytic middle row that creates extra facets
    // visible as a "band" on the cone surface.
    let n_v = if use_cached_top && !top_row_at_apex && effective_top.len() == n_u {
        1
    } else {
        n_v
    };

    log::debug!(
        "Cone tube face: n_u={}, n_v={}, full_wrap={}, apex={}, use_cached_top={}, v=[{:.3},{:.3}]",
        n_u, n_v, is_full_wrap, top_row_at_apex, use_cached_top, v_min, v_max
    );

    // Generate vertex grid with apex degeneracy handling.
    // - j=0 (bottom): use cached bottom_ring points (shared with bottom cap face).
    // - If apex is at v_min (j=0): single apex vertex at row 0.
    // - j=n_v (top, if apex at v_max): single apex vertex.
    // - j=n_v (top, if not apex & use_cached_top): cached top_ring points.
    // - j=n_v (top, if not apex & !use_cached_top): analytic cone.point_at(bottom_u[i], v_max).
    // - 0 < j < n_v (intermediate): analytic cone.point_at(bottom_u[i], v_j).
    
    // Determine which row the apex is at (if any)
    let apex_at_bottom = top_row_at_apex && apex_v <= v_min;
    let apex_at_top = top_row_at_apex && apex_v >= v_max;
    
    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;

        if (apex_at_bottom && j == 0) || (apex_at_top && j == n_v) {
            // Apex row — single vertex (all u values map to the apex point).
            let p = cone.point_at(0.0, apex_v);
            let n = orient_normal(cone.normal_at(0.0, apex_v), forward);
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
                let u = bottom_u[i];
                let p = if j == 0 && !apex_at_bottom {
                    effective_bottom[i].1
                } else if j == n_v && use_cached_top && !apex_at_top {
                    effective_top[i].1
                } else {
                    crate::edge_cache::deterministic_round_point(cone.point_at(u, v))
                };
                let n = orient_normal(cone.normal_at(u, v), forward);
                let idx = mesh.add_vertex(p);
                mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            }
            total_vertices += n_u as u32;
        }
    }

    // Generate triangles.
    // Apex row fans from the adjacent ring; other rows use standard quad split.
    //
    // For FULL-wrap: i_next = (i+1) % row_count (wrap around).
    // For PARTIAL-wrap: i_next = i+1, NO wrap (last column has no right
    // neighbor — that's where the seam line lives).
    for j in 0..n_v {
        let j_next = j + 1;
        let row_count = row_vertex_count[j];
        let next_row_count = row_vertex_count[j_next];
        let row_base = row_vertex_offset[j];
        let next_row_base = row_vertex_offset[j_next];

        if row_count == 1 {
            // Current row is apex — fan from apex to next row ring.
            let apex = row_base;
            for i in 0..next_row_count {
                let i_next = if is_full_wrap {
                    (i + 1) % next_row_count
                } else {
                    if i + 1 >= next_row_count { continue; }
                    i + 1
                };
                let v0 = next_row_base + i as u32;
                let v1 = next_row_base + i_next as u32;
                if forward {
                    mesh.add_triangle(apex, v0, v1);
                } else {
                    mesh.add_triangle(apex, v1, v0);
                }
            }
        } else if next_row_count == 1 {
            // Next row is apex — fan from current row ring to apex.
            let apex = next_row_base;
            for i in 0..row_count {
                let i_next = if is_full_wrap {
                    (i + 1) % row_count
                } else {
                    if i + 1 >= row_count { continue; }
                    i + 1
                };
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
                let i_next = if is_full_wrap {
                    (i + 1) % row_count
                } else {
                    if i + 1 >= row_count { continue; }
                    i + 1
                };
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

/// Analytic fallback for cone triangulation between v_min and v_max.
/// Same role as `triangulate_cylinder_full_at_v_range` but for cones.
fn triangulate_cone_full_at_v_range(
    cone: &ConeSurface,
    params: &TriangulationParams,
    v_min: f64,
    v_max: f64,
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let apex_v = cone.apex_v();
    // Clamp v_min to not go past the apex (apex is at negative v for standard cones)
    let v_min = if !cone.expanding && apex_v.is_finite() { v_min.max(apex_v) } else { v_min };
    let apex_near_v_min = !cone.expanding && apex_v.is_finite()
        && (v_min - apex_v).abs() < apex_v.abs().max(1e-6) * 0.01 + 1e-6;
    let apex_near_v_max = !cone.expanding && apex_v.is_finite()
        && (v_max - apex_v).abs() < apex_v.abs().max(1e-6) * 0.01 + 1e-6;
    let apex_at_bottom = apex_near_v_min; // apex is at j=0
    let apex_at_top    = apex_near_v_max; // apex is at j=n_v

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cone(cone.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples.max(8), params.height_samples.max(2))
    };

    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
        if (apex_at_bottom && j == 0) || (apex_at_top && j == n_v) {
            // Apex row — single vertex (all u values map to the apex point)
            let p = cone.point_at(0.0, apex_v);
            let n = orient_normal(cone.normal_at(0.0, apex_v), forward);
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
                let p = crate::edge_cache::deterministic_round_point(cone.point_at(u, v));
                let n = orient_normal(cone.normal_at(u, v), forward);
                let idx = mesh.add_vertex(p);
                mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            }
            total_vertices += n_u as u32;
        }
    }
    for j in 0..n_v {
        let j_next = j + 1;
        let row_count = row_vertex_count[j];
        let next_row_count = row_vertex_count[j_next];
        let row_base = row_vertex_offset[j];
        let next_row_base = row_vertex_offset[j_next];
        if row_count == 1 {
            // Current row is apex — fan from apex to next row ring
            let apex = row_base;
            for i in 0..next_row_count {
                let i_next = (i + 1) % next_row_count;
                let v0 = next_row_base + i as u32;
                let v1 = next_row_base + i_next as u32;
                if v0 != v1 {
                    if forward {
                        mesh.add_triangle(apex, v0, v1);
                    } else {
                        mesh.add_triangle(apex, v1, v0);
                    }
                }
            }
        } else if next_row_count == 1 {
            // Next row is apex — fan from current row ring to apex
            let apex = next_row_base;
            for i in 0..row_count {
                let i_next = (i + 1) % row_count;
                let v0 = row_base + i as u32;
                let v1 = row_base + i_next as u32;
                if v0 != v1 {
                    if forward {
                        mesh.add_triangle(v0, v1, apex);
                    } else {
                        mesh.add_triangle(v0, apex, v1);
                    }
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
    let forward = face.forward;
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
            let n = orient_normal(cyl.normal_at(u, v), forward);
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
        // No outer wire → recover boundary points directly from face.edges.
        // Same fix as triangulate_cylinder_face — see comment there for details.
        let mut direct_boundary: Vec<Point3d> = Vec::new();
        for edge in &face.edges {
            if edge.degenerate { continue; }
            if let Some(disc) = cache.get(edge.id) {
                direct_boundary.extend(disc.points_3d.iter().cloned());
            }
        }
        if !direct_boundary.is_empty() {
            let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
            let mut dedup: Vec<Point3d> = Vec::with_capacity(direct_boundary.len());
            for p in &direct_boundary {
                if let Some(last) = dedup.last() {
                    if last.distance_to(p) <= adaptive_tol { continue; }
                }
                dedup.push(*p);
            }
            if dedup.len() > 1 {
                if let Some(last) = dedup.last() {
                    if last.distance_to(&dedup[0]) <= adaptive_tol {
                        dedup.pop();
                    }
                }
            }
            if dedup.len() >= 6 {
                log::debug!(
                    "Cone face #{}: no outer_wire but {} edge points recovered from cache — using tube grid triangulation",
                    face.id, dedup.len()
                );
                return triangulate_cone_tube_from_boundary(cone, params, &dedup, face.forward);
            }
        }
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

    // Detect PARTIAL-wrap tube face (same logic as triangulate_cylinder_face).
    // See triangulate_cylinder_face for detailed comments on why partial-wrap
    // faces need this detection (earcutr produces twisted triangles for them).
    if hole_polylines.is_empty() && boundary_3d.len() >= 6 {
        let (v_min_pw, v_max_pw) = compute_axis_v_range_pts(&boundary_3d, &cone.origin, &cone.axis);
        if v_max_pw > v_min_pw {
            let dedup_tol = params.max_deviation.max(1e-6) * 0.5;
            let (bottom_ring, top_ring, _) = split_boundary_into_rings_with_u(
                &boundary_3d, &cone.origin, &cone.axis, &cone.x_dir,
                v_min_pw, v_max_pw, dedup_tol,
            );
            if bottom_ring.len() >= 3 && top_ring.len() >= 3 {
                log::info!(
                    "Cone face #{}: PARTIAL tube face detected ({} bottom + {} top ring points, {} bnd pts) — using tube grid triangulation",
                    face.id, bottom_ring.len(), top_ring.len(), boundary_3d.len()
                );
                return triangulate_cone_tube_from_boundary(cone, params, &boundary_3d, face.forward);
            }
        }
    }

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
    let forward = face.forward;
    let mut mesh = TriangleMesh::new();
    let (v_min, v_max) = compute_axis_v_range(face, &cone.origin, &cone.axis);
    // For a full cone without wires, the v range goes from the apex to the base.
    // With STEP parameterization, apex is at negative v and base is at v=0 (or wherever the edges are).
    let (mut v_min, v_max) = if v_min < v_max {
        (v_min, v_max)
    } else {
        // No edges found — use default range from apex to a reasonable height above base
        let apex_v = cone.apex_v();
        (apex_v, apex_v + cone.height().min(100.0))
    };

    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            &Surface::Cone(cone.clone()), 0.0, 2.0 * PI, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples, params.height_samples.max(2))
    };

    // Clamp v_min to apex v value (apex is at negative v for standard cones)
    let apex_v = cone.apex_v();
    if !cone.expanding && apex_v.is_finite() {
        v_min = v_min.max(apex_v);
    }

    // Check if the apex lies at one of the v-range boundaries.
    // After clamping v_min to apex_v, v_min == apex_v means the apex
    // is at the BOTTOM row (j=0).  If apex_v ≈ v_max the apex is at
    // the TOP row (j=n_v).
    let apex_near_v_min = !cone.expanding && apex_v.is_finite()
        && (v_min - apex_v).abs() < apex_v.abs().max(1e-6) * 0.01 + 1e-6;
    let apex_near_v_max = !cone.expanding && apex_v.is_finite()
        && (v_max - apex_v).abs() < apex_v.abs().max(1e-6) * 0.01 + 1e-6;
    let apex_at_bottom = apex_near_v_min; // apex is at j=0
    let apex_at_top    = apex_near_v_max; // apex is at j=n_v

    // Generate vertex grid with apex degeneracy handling
    let mut _apex_vertex: Option<u32> = None;
    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;

        if (apex_at_bottom && j == 0) || (apex_at_top && j == n_v) {
            // Apex row — single vertex (all u values map to the apex point)
            let p = cone.point_at(0.0, apex_v);
            let n = orient_normal(cone.normal_at(0.0, apex_v), forward);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            _apex_vertex = Some(idx);
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
                let n = orient_normal(cone.normal_at(u, v), forward);
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
            let apex = row_base;
            for i in 0..next_row_count {
                let i_next = (i + 1) % next_row_count;
                let v0 = next_row_base + i as u32;
                let v1 = next_row_base + i_next as u32;
                if v0 != v1 {
                    if face.forward {
                        mesh.add_triangle(apex, v0, v1);
                    } else {
                        mesh.add_triangle(apex, v1, v0);
                    }
                }
            }
        } else if next_row_count == 1 {
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
    let n_north = orient_normal(sphere.normal_at(0.0, 0.0), face.forward);
    let north_idx = mesh.add_vertex(p_north);
    mesh.add_vertex_normal(north_idx, [n_north.x, n_north.y, n_north.z]);

    // Ring vertices (rows 1..n_v-1)
    for j in 1..n_v {
        let v = PI * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let p = sphere.point_at(u, v);
            let n = orient_normal(sphere.normal_at(u, v), face.forward);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // South pole vertex
    let p_south = sphere.point_at(0.0, PI);
    let n_south = orient_normal(sphere.normal_at(0.0, PI), face.forward);
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

    // Detect "half-wrap" torus face: one axis spans >1.9π (full wrap) while
    // the other spans a partial arc. earcutr cannot handle periodic-wrap
    // polygons — boundary points at v=0 and v=2π (which are the SAME 3D
    // point) appear as different UV points, producing twisted triangles.
    //
    // Fix: UNWRAP the periodic axis so the polygon lies in a continuous
    // UV range. For each point with u (or v) > π, shift it by -2π. This
    // moves the "wrapped" half of the polygon to negative coordinates,
    // producing a continuous range like [-π, π/4] instead of [0, 2π].
    //
    // Example (drill_top.stp Face #4, STEP #803):
    //   Boundary has v values 0.7854, 0.6283, ..., 0.0393, 6.2832, 0.1178, ..., 0.7069
    //   The 6.2832 value (=2π) is the same 3D point as v=0. After unwrapping
    //   (shift v > π to v - 2π), the polygon has v ∈ [-6.16e-7, 0.7854]
    //   (continuous), and earcutr produces correct triangulation.
    let (boundary_3d_eff, boundary_uvs_eff, hole_polylines_eff, hole_uvs_eff) =
        unwrap_periodic_torus_boundary(
            &boundary_3d, &boundary_uvs, &hole_polylines, &hole_uvs,
        );

    crate::parametric_domain::triangulate_surface_consistent(
        &surface,
        &boundary_3d_eff,
        &boundary_uvs_eff,
        &hole_polylines_eff,
        &hole_uvs_eff,
        face.forward,
        params,
    )
}

/// Unwrap a torus face's periodic UV boundary so earcutr can triangulate it.
///
/// A torus is periodic in BOTH u and v (period 2π). When a face's boundary
/// spans more than ~1.9π in one axis, the polygon "wraps around" the seam
/// — points near v=0 and v=2π are at the same 3D location but have different
/// UV values. earcutr treats them as distinct points and produces twisted
/// triangulation.
///
/// This function detects the wrap and applies CHAIN-BASED adjustment:
/// each UV point is adjusted to be close to the PREVIOUS point by adding
/// or subtracting 2π. This maintains boundary continuity while unwrapping
/// the periodic axis.
///
/// If no wrap is detected, returns the inputs unchanged.
fn unwrap_periodic_torus_boundary(
    boundary_3d: &[draper_geometry::Point3d],
    boundary_uvs: &[draper_geometry::Point2d],
    hole_polylines: &[Vec<draper_geometry::Point3d>],
    hole_uvs: &[Vec<draper_geometry::Point2d>],
) -> (
    Vec<draper_geometry::Point3d>,
    Vec<draper_geometry::Point2d>,
    Vec<Vec<draper_geometry::Point3d>>,
    Vec<Vec<draper_geometry::Point2d>>,
) {
    use draper_geometry::Point2d;

    if boundary_uvs.is_empty() {
        return (
            boundary_3d.to_vec(),
            boundary_uvs.to_vec(),
            hole_polylines.to_vec(),
            hole_uvs.to_vec(),
        );
    }

    // Compute u and v ranges
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for p in boundary_uvs {
        u_min = u_min.min(p.u);
        u_max = u_max.max(p.u);
        v_min = v_min.min(p.v);
        v_max = v_max.max(p.v);
    }
    let u_range = u_max - u_min;
    let v_range = v_max - v_min;

    // Detect wrap: range > 1.9π in either axis
    let wrap_u = u_range > 1.9 * PI;
    let wrap_v = v_range > 1.9 * PI;

    log::warn!(
        "TORUS_UNWRAP_CHECK: n_bnd={}, u=[{:.4},{:.4}] (range={:.4}), v=[{:.4},{:.4}] (range={:.4}), wrap_u={}, wrap_v={}",
        boundary_uvs.len(), u_min, u_max, u_range, v_min, v_max, v_range, wrap_u, wrap_v,
    );
    eprintln!(
        "TORUS_UNWRAP_CHECK_EPRINT: n_bnd={}, u=[{:.4},{:.4}] (range={:.4}), v=[{:.4},{:.4}] (range={:.4}), wrap_u={}, wrap_v={}",
        boundary_uvs.len(), u_min, u_max, u_range, v_min, v_max, v_range, wrap_u, wrap_v,
    );

    if !wrap_u && !wrap_v {
        // No wrap — return inputs unchanged
        return (
            boundary_3d.to_vec(),
            boundary_uvs.to_vec(),
            hole_polylines.to_vec(),
            hole_uvs.to_vec(),
        );
    }

    log::warn!(
        "TORUS_UNWRAP: wrap_u={}, wrap_v={}, u=[{:.4},{:.4}], v=[{:.4},{:.4}], {} bnd pts, {} holes",
        wrap_u, wrap_v, u_min, u_max, v_min, v_max,
        boundary_uvs.len(), hole_uvs.len(),
    );

    // CHAIN-BASED adjustment: for each point, adjust u (or v) to be close to
    // the PREVIOUS point by adding/subtracting 2π. This maintains boundary
    // continuity.
    //
    // The period for both u and v on a torus is 2π.
    let period = 2.0 * PI;
    let half_period = period * 0.5;

    let chain_adjust = |uvs: &[Point2d]| -> Vec<Point2d> {
        if uvs.is_empty() {
            return Vec::new();
        }
        let mut result = Vec::with_capacity(uvs.len());
        result.push(uvs[0]);
        for i in 1..uvs.len() {
            let prev = result[i - 1];
            let mut new_u = uvs[i].u;
            let mut new_v = uvs[i].v;
            if wrap_u {
                let diff = new_u - prev.u;
                if diff > half_period {
                    new_u -= period;
                } else if diff < -half_period {
                    new_u += period;
                }
            }
            if wrap_v {
                let diff = new_v - prev.v;
                if diff > half_period {
                    new_v -= period;
                } else if diff < -half_period {
                    new_v += period;
                }
            }
            result.push(Point2d::new(new_u, new_v));
        }
        result
    };

    let new_boundary_uvs = chain_adjust(boundary_uvs);
    let new_hole_uvs: Vec<Vec<Point2d>> = hole_uvs.iter()
        .map(|hole| chain_adjust(hole))
        .collect();

    // 3D points are unchanged (UV unwrap doesn't affect geometry)
    (
        boundary_3d.to_vec(),
        new_boundary_uvs,
        hole_polylines.to_vec(),
        new_hole_uvs,
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
            let n = if face.forward {
                torus.normal_at(u, v)
            } else {
                // For forward:false, negate the outward normal so it points inward
                let n = torus.normal_at(u, v);
                draper_geometry::Direction3d::new(-n.x, -n.y, -n.z).unwrap_or(n)
            };
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
/// Try strip triangulation for a ruled NURBS surface.
///
/// A ruled NURBS surface (degree 1 in one direction) has two "rails" —
/// the two boundary edges that run along the linear direction. A strip
/// triangulation connects corresponding points on the two rails, creating
/// a ladder-like mesh that uses ALL rim edges.
///
/// This is critical for watertightness: earcutr's generic triangulation
/// often skips rim edges, creating boundary edges in the merged mesh.
/// The strip triangulation guarantees all rim edges are used.
///
/// # Requirements
/// - The boundary must have exactly 4 edges (a quadrilateral UV domain)
/// - The two rails must have the same number of points
/// - No holes
///
/// # Returns
/// Some(mesh) if strip triangulation was applied, None if not applicable.
fn try_strip_triangulation_ruled_nurbs(
    nurbs: &NurbsSurface,
    boundary_points: &[Point3d],
    boundary_uvs: &[Point2d],
    forward: bool,
) -> Option<TriangleMesh> {
    use draper_geometry::{Point3d, Point2d};

    log::warn!(
        "STRIP_ENTER: bnd={} u_deg={} v_deg={} u_range=[{:.3},{:.3}] v_range=[{:.3},{:.3}]",
        boundary_points.len(), nurbs.u_degree, nurbs.v_degree,
        nurbs.u_knots.first().copied().unwrap_or(0.0),
        nurbs.u_knots.last().copied().unwrap_or(0.0),
        nurbs.v_knots.first().copied().unwrap_or(0.0),
        nurbs.v_knots.last().copied().unwrap_or(0.0),
    );

    if boundary_points.is_empty() || boundary_uvs.len() != boundary_points.len() {
        log::warn!("STRIP_EXIT: empty boundary or length mismatch");
        return None;
    }

    // Deduplicate consecutive boundary points (3D tolerance 1e-6).
    // The converter skips dedup for NURBS, so the boundary may contain
    // duplicate points at corners and along edges. These duplicates cause
    // the strip to produce degenerate triangles (zero area, distinct indices)
    // that get skipped during merging, leaving boundary edges.
    // By deduplicating here, we ensure the strip works with unique points.
    let mut deduped_points: Vec<Point3d> = Vec::with_capacity(boundary_points.len());
    let mut deduped_uvs: Vec<Point2d> = Vec::with_capacity(boundary_uvs.len());
    let mut orig_to_dedup: Vec<usize> = Vec::with_capacity(boundary_points.len());
    for (i, p) in boundary_points.iter().enumerate() {
        let is_dup = deduped_points.iter().rposition(|q| {
            (q.x - p.x).abs() < 1e-6 && (q.y - p.y).abs() < 1e-6 && (q.z - p.z).abs() < 1e-6
        });
        match is_dup {
            Some(idx) => orig_to_dedup.push(idx),
            None => {
                orig_to_dedup.push(deduped_points.len());
                deduped_points.push(*p);
                deduped_uvs.push(boundary_uvs[i]);
            }
        }
    }
    let boundary_points: &[Point3d] = &deduped_points;
    let boundary_uvs: &[Point2d] = &deduped_uvs;
    log::warn!(
        "STRIP_DEDUP: {} → {} unique points",
        orig_to_dedup.len(), deduped_points.len()
    );

    // Identify the 4 edges of the boundary by finding "corners" — points where
    // the UV direction changes significantly.
    //
    // For a ruled NURBS with u_degree=1, the two rails are at constant u
    // (u_min and u_max), running along v. The other two edges are at constant
    // v (v_min and v_max), running along u.
    //
    // For v_degree=1, the rails are at constant v, running along u.
    let (u_min, u_max) = nurbs.u_range();
    let (v_min, v_max) = nurbs.v_range();
    let u_range = u_max - u_min;
    let v_range = v_max - v_min;

    if u_range < 1e-10 || v_range < 1e-10 {
        return None; // Degenerate surface
    }

    // Classify each boundary point by which edge it's on:
    // 0 = u_min edge (rail 0 if u_degree=1)
    // 1 = u_max edge (rail 1 if u_degree=1)
    // 2 = v_min edge
    // 3 = v_max edge
    let u_tol = u_range * 0.01; // 1% of u_range
    let v_tol = v_range * 0.01;

    let is_u_ruled = nurbs.u_degree == 1 && nurbs.v_degree > 1;
    let is_v_ruled = nurbs.v_degree == 1 && nurbs.u_degree > 1;

    if !is_u_ruled && !is_v_ruled {
        return None; // Not a ruled surface
    }

    // Find corners: points where BOTH u and v are at extremes
    let mut corner_indices: Vec<usize> = Vec::new();
    for (i, uv) in boundary_uvs.iter().enumerate() {
        let at_u_min = (uv.u - u_min).abs() < u_tol;
        let at_u_max = (uv.u - u_max).abs() < u_tol;
        let at_v_min = (uv.v - v_min).abs() < v_tol;
        let at_v_max = (uv.v - v_max).abs() < v_tol;
        if (at_u_min || at_u_max) && (at_v_min || at_v_max) {
            // Check if this point is too close to ANY existing corner.
            // Use a FIXED 3D tolerance (1e-6) instead of a parameter-space
            // tolerance — the parameter-to-3D scaling varies across surfaces,
            // making parameter-based tolerances unreliable.
            // Corners that come from the same VERTEX_POINT should be at
            // EXACTLY the same 3D position (bit-identical from edge cache).
            let mut is_duplicate = false;
            let corner_dist_tol = 1e-6; // 1 micron — corners from same vertex are bit-identical
            for &prev in &corner_indices {
                let dist = ((boundary_points[i].x - boundary_points[prev].x).powi(2)
                    + (boundary_points[i].y - boundary_points[prev].y).powi(2)
                    + (boundary_points[i].z - boundary_points[prev].z).powi(2)).sqrt();
                if dist < corner_dist_tol {
                    is_duplicate = true;
                    break;
                }
            }
            if !is_duplicate {
                corner_indices.push(i);
            }
        }
    }

    if corner_indices.len() != 4 {
        log::warn!(
            "STRIP_FAIL: need 4 corners, found {} (u_range={:.3}, v_range={:.3}, u_tol={:.4}, v_tol={:.4}) corners={:?}",
            corner_indices.len(), u_range, v_range, u_tol, v_tol, corner_indices
        );
        return None;
    }
    log::warn!("STRIP_CORNERS: found 4 corners at {:?}", corner_indices);
    for (ci, &idx) in corner_indices.iter().enumerate() {
        let p = &boundary_points[idx];
        let uv = &boundary_uvs[idx];
        log::warn!(
            "  corner {}: idx={} pos=({:.3},{:.3},{:.3}) uv=({:.3},{:.3})",
            ci, idx, p.x, p.y, p.z, uv.u, uv.v
        );
    }
    // Print y-range of boundary points to detect cross-half contamination
    let (y_min, y_max) = boundary_points.iter().fold((f64::MAX, f64::MIN), |(mn, mx), p| {
        (mn.min(p.y), mx.max(p.y))
    });
    log::warn!("  boundary y-range: [{:.3}, {:.3}]", y_min, y_max);
    // Print a few sample points from each rail
    log::warn!("  rail_a samples (edge 2 = corner2→corner3):");
    let n = boundary_points.len();
    for offset in [0, 1, n/4, n/2, 3*n/4] {
        let idx = corner_indices[2] + offset;
        if idx < n {
            let p = &boundary_points[idx];
            log::warn!("    idx={} pos=({:.3},{:.3},{:.3})", idx, p.x, p.y, p.z);
        }
    }

    // Split the boundary into 4 edges using the corners
    // The edges are: corner[0]→corner[1], corner[1]→corner[2], corner[2]→corner[3], corner[3]→corner[0]
    let n = boundary_points.len();
    let mut edges: Vec<Vec<usize>> = Vec::with_capacity(4);
    for i in 0..4 {
        let start = corner_indices[i];
        let end = corner_indices[(i + 1) % 4];
        let mut edge: Vec<usize> = Vec::new();
        let mut j = start;
        edge.push(j);
        while j != end {
            j = (j + 1) % n;
            edge.push(j);
        }
        edges.push(edge);
    }

    // Identify which edges are the rails
    // For u_ruled: rails are the edges at u_min and u_max (constant u, varying v)
    // For v_ruled: rails are the edges at v_min and v_max (constant v, varying u)
    let mut rail_a_idx: Option<usize> = None;
    let mut rail_b_idx: Option<usize> = None;
    let mut side_a_idx: Option<usize> = None;
    let mut side_b_idx: Option<usize> = None;

    for (ei, edge) in edges.iter().enumerate() {
        // Check the midpoint of this edge to classify it
        let mid_idx = edge[edge.len() / 2];
        let mid_uv = boundary_uvs[mid_idx];
        let at_u_min = (mid_uv.u - u_min).abs() < u_tol;
        let at_u_max = (mid_uv.u - u_max).abs() < u_tol;
        let at_v_min = (mid_uv.v - v_min).abs() < v_tol;
        let at_v_max = (mid_uv.v - v_max).abs() < v_tol;

        if is_u_ruled {
            if at_u_min {
                rail_a_idx = Some(ei);
            } else if at_u_max {
                rail_b_idx = Some(ei);
            } else if at_v_min || at_v_max {
                if side_a_idx.is_none() {
                    side_a_idx = Some(ei);
                } else {
                    side_b_idx = Some(ei);
                }
            }
        } else {
            // v_ruled
            if at_v_min {
                rail_a_idx = Some(ei);
            } else if at_v_max {
                rail_b_idx = Some(ei);
            } else if at_u_min || at_u_max {
                if side_a_idx.is_none() {
                    side_a_idx = Some(ei);
                } else {
                    side_b_idx = Some(ei);
                }
            }
        }
    }

    let rail_a_idx = rail_a_idx?;
    let rail_b_idx = rail_b_idx?;
    let _side_a_idx = side_a_idx?;
    let _side_b_idx = side_b_idx?;

    let rail_a = &edges[rail_a_idx];
    let rail_b = &edges[rail_b_idx];

    // ─── RESAMPLE RAILS TO COMMON COUNT ───────────────────────────────
    //
    // The edge cache discretizes each EDGE_CURVE independently using chord-error
    // adaptation. Two rails that trace geometrically equivalent curves (e.g., the
    // top and bottom edges of a cylinder's half-side) can therefore end up with
    // DIFFERENT point counts (e.g., 63 vs 61). If we just take min(na, nb) - 1
    // quads, the extra points on the longer rail become ORPHAN VERTICES that are
    // added to the mesh but never used in any triangle. Those orphans create
    // boundary edges (visible holes in the rendered mesh).
    //
    // To eliminate the orphans, we resample BOTH rails by arc length to a common
    // count = max(na, nb). The resampled rails share endpoints with the originals
    // (corners are preserved bit-identically), so watertightness with adjacent
    // faces is maintained. Interior points are interpolated along the rail
    // polyline; for NURBS surfaces the interpolation is along the surface's
    // isoparametric curve, which is the geometrically correct thing to do.
    let na_orig = rail_a.len();
    let nb_orig = rail_b.len();

    // Reverse rail_b so that rail_b[i] corresponds to rail_a[i] (going the same
    // direction along the surface).
    let rail_b_fwd: Vec<usize> = rail_b.iter().rev().copied().collect();

    // Common count = max of the two rails. We always resample to this count so
    // that quads are uniform and no vertices are orphaned.
    let n_common = na_orig.max(nb_orig);
    if n_common < 2 {
        return None;
    }

    // Resample a polyline (Vec of boundary indices) to n target points by arc length.
    // Returns a Vec of (Point3d, Point2d, original_boundary_index_or_None).
    // If original_boundary_index is None, the point was interpolated and does not
    // exist in the original boundary (we add it as a new mesh vertex).
    let resample = |rail: &[usize], n: usize| -> Vec<(Point3d, Point2d, Option<usize>)> {
        if rail.is_empty() {
            return Vec::new();
        }
        if rail.len() == 1 || n == 1 {
            let idx = rail[0];
            return vec![(boundary_points[idx], boundary_uvs[idx], Some(idx))];
        }

        // Compute cumulative arc length along the rail (in 3D).
        let mut cum_len = Vec::with_capacity(rail.len());
        cum_len.push(0.0f64);
        for i in 1..rail.len() {
            let p_prev = boundary_points[rail[i - 1]];
            let p_curr = boundary_points[rail[i]];
            let dx = p_curr.x - p_prev.x;
            let dy = p_curr.y - p_prev.y;
            let dz = p_curr.z - p_prev.z;
            let seg = (dx * dx + dy * dy + dz * dz).sqrt();
            cum_len.push(cum_len[i - 1] + seg);
        }
        let total_len = *cum_len.last().unwrap();

        let mut out: Vec<(Point3d, Point2d, Option<usize>)> = Vec::with_capacity(n);
        for k in 0..n {
            // Sample at evenly-spaced arc-length fractions.
            // For k=0 → 0.0, for k=n-1 → 1.0 (exactly hits the endpoints).
            let t = if n == 1 { 0.0 } else { k as f64 / (n - 1) as f64 };
            let target_len = t * total_len;

            // Find segment containing target_len.
            // Use binary search on cum_len for efficiency.
            let seg_idx = match cum_len.binary_search_by(|c| c.partial_cmp(&target_len).unwrap_or(std::cmp::Ordering::Equal)) {
                Ok(i) => i, // Exact match — use the point directly
                Err(i) => {
                    if i == 0 {
                        0
                    } else if i >= rail.len() {
                        rail.len() - 1
                    } else {
                        // target_len is between cum_len[i-1] and cum_len[i]
                        i - 1
                    }
                }
            };

            // If exact match (within 1e-12), use the original point
            if (cum_len[seg_idx] - target_len).abs() < 1e-12 {
                let idx = rail[seg_idx];
                out.push((boundary_points[idx], boundary_uvs[idx], Some(idx)));
                continue;
            }

            // Interpolate within segment seg_idx → seg_idx+1
            let seg_start_len = cum_len[seg_idx];
            let seg_end_len = if seg_idx + 1 < cum_len.len() { cum_len[seg_idx + 1] } else { total_len };
            let seg_len = seg_end_len - seg_start_len;
            if seg_len < 1e-15 {
                let idx = rail[seg_idx];
                out.push((boundary_points[idx], boundary_uvs[idx], Some(idx)));
                continue;
            }
            let local_t = (target_len - seg_start_len) / seg_len;
            let idx_a = rail[seg_idx];
            let idx_b = rail[(seg_idx + 1).min(rail.len() - 1)];
            let pa = boundary_points[idx_a];
            let pb = boundary_points[idx_b];
            let ua = boundary_uvs[idx_a];
            let ub = boundary_uvs[idx_b];
            let p = Point3d::new(
                pa.x + local_t * (pb.x - pa.x),
                pa.y + local_t * (pb.y - pa.y),
                pa.z + local_t * (pb.z - pa.z),
            );
            let uv = Point2d::new(
                ua.u + local_t * (ub.u - ua.u),
                ua.v + local_t * (ub.v - ua.v),
            );
            out.push((p, uv, None));
        }
        out
    };

    let rail_a_resampled = resample(rail_a, n_common);
    let rail_b_resampled = resample(&rail_b_fwd, n_common);

    log::warn!(
        "STRIP_RESAMPLE: rail_a {}→{}, rail_b {}→{} (common={})",
        na_orig, rail_a_resampled.len(), nb_orig, rail_b_resampled.len(), n_common
    );

    let n_quads = n_common.saturating_sub(1);
    if n_quads == 0 {
        return None;
    }

    let mut mesh = TriangleMesh::new();

    // Add all ORIGINAL boundary points as vertices (for watertightness with
    // adjacent faces — they share these exact vertices via the edge cache).
    let mut vertex_map: std::collections::HashMap<usize, u32> = std::collections::HashMap::new();
    for (i, p) in boundary_points.iter().enumerate() {
        let vi = mesh.add_vertex(*p);
        // Compute normal from NURBS derivatives
        let uv = boundary_uvs[i];
        let derivs = nurbs.derivatives_at(uv.u, uv.v);
        mesh.add_vertex_normal(vi, [derivs.normal().x, derivs.normal().y, derivs.normal().z]);
        vertex_map.insert(i, vi);
    }

    // Add INTERPOLATED rail points as additional vertices.
    // We do this so that the resampled rails can be referenced by mesh index.
    // For NURBS-ruling this is geometrically correct: the interpolated points
    // lie on the surface's isoparametric curve (since the rail is itself an
    // isoparametric curve at constant u or v).
    //
    // IMPORTANT: For better surface fidelity, we evaluate the NURBS surface at
    // the interpolated UV instead of just linearly interpolating the 3D point.
    // This gives us the exact surface point (no chord error).
    let mut rail_a_mesh_idx: Vec<u32> = Vec::with_capacity(n_common);
    let mut rail_b_mesh_idx: Vec<u32> = Vec::with_capacity(n_common);

    for k in 0..n_common {
        // Rail A
        let (p_a, uv_a, orig_a) = rail_a_resampled[k];
        let vi_a = if let Some(orig) = orig_a {
            *vertex_map.get(&orig).unwrap_or(&0)
        } else {
            // Evaluate the NURBS surface at the interpolated UV for exact surface point.
            let p_exact = nurbs.point_at(uv_a.u, uv_a.v);
            let vi = mesh.add_vertex(p_exact);
            let derivs = nurbs.derivatives_at(uv_a.u, uv_a.v);
            mesh.add_vertex_normal(vi, [derivs.normal().x, derivs.normal().y, derivs.normal().z]);
            // Suppress unused-variable warning
            let _ = p_a;
            vi
        };
        rail_a_mesh_idx.push(vi_a);

        // Rail B
        let (p_b, uv_b, orig_b) = rail_b_resampled[k];
        let vi_b = if let Some(orig) = orig_b {
            *vertex_map.get(&orig).unwrap_or(&0)
        } else {
            let p_exact = nurbs.point_at(uv_b.u, uv_b.v);
            let vi = mesh.add_vertex(p_exact);
            let derivs = nurbs.derivatives_at(uv_b.u, uv_b.v);
            mesh.add_vertex_normal(vi, [derivs.normal().x, derivs.normal().y, derivs.normal().z]);
            let _ = p_b;
            vi
        };
        rail_b_mesh_idx.push(vi_b);
    }

    log::warn!(
        "STRIP_BUILD: n_common={} n_quads={} rail_a_mesh[0..3]={:?} rail_b_mesh[0..3]={:?}",
        n_common, n_quads, &rail_a_mesh_idx[..3.min(rail_a_mesh_idx.len())], &rail_b_mesh_idx[..3.min(rail_b_mesh_idx.len())]
    );

    for i in 0..n_quads {
        let a0 = rail_a_mesh_idx[i];
        let a1 = rail_a_mesh_idx[i + 1];
        let b0 = rail_b_mesh_idx[i];
        let b1 = rail_b_mesh_idx[i + 1];

        // Skip degenerate triangles
        if a0 == a1 || a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1 || b0 == b1 {
            continue;
        }

        // Print first 2 and middle quad for debugging
        if i < 2 || i == n_quads / 2 {
            let p_a0 = mesh.vertices[a0 as usize];
            let p_b0 = mesh.vertices[b0 as usize];
            log::warn!(
                "  quad {}: a0(mesh={})=({:.2},{:.2},{:.2}) b0(mesh={})=({:.2},{:.2},{:.2})",
                i, a0, p_a0.x, p_a0.y, p_a0.z, b0, p_b0.x, p_b0.y, p_b0.z
            );
        }

        // Two triangles per quad:
        // (a0, a1, b1) and (a0, b1, b0)
        // The orientation depends on the surface normal direction
        if forward {
            mesh.add_triangle(a0, a1, b1);
            mesh.add_triangle(a0, b1, b0);
        } else {
            mesh.add_triangle(a0, b1, a1);
            mesh.add_triangle(a0, b0, b1);
        }
    }

    // NOTE: The side cap fan triangulation was REMOVED.
    //
    // Previously, this code added fan triangles for the side edges (the edges
    // at v_min and v_max for u_ruled surfaces, or u_min and u_max for v_ruled).
    // The fan triangulation created SPURIOUS triangles that:
    // 1. Overlapped with the strip triangles (which already cover the surface)
    // 2. Created non-manifold edges (3+ triangles sharing an edge)
    // 3. Produced inconsistent results depending on face processing order
    //
    // For ruled NURBS surfaces, the sides are STRAIGHT LINES (because the
    // surface is ruled in one direction). The strip triangulation already
    // uses the endpoints of the sides (via the rail endpoints). If the edge
    // cache produces intermediate points on the sides (which are collinear),
    // those points are added as vertices but not used in any triangle.
    //
    // This is correct behavior: the intermediate side points are collinear
    // and don't affect the surface geometry. They're shared with adjacent
    // faces via the edge cache, ensuring watertightness.

    log::info!(
        "strip_triangulation: created {} triangles from {} rail points (rails: {} and {} pts, sides: {} and {} pts)",
        mesh.triangle_count(),
        n_common,
        na_orig, nb_orig,
        edges[_side_a_idx].len(),
        edges[_side_b_idx].len(),
    );

    Some(mesh)
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

    // For RULED NURBS (degree 1 in one direction) with NO holes, try strip
    // triangulation FIRST. Strip triangulation:
    // 1. Uses ALL boundary points from the edge cache (watertight by construction)
    // 2. Evaluates the NURBS surface at rail points (proper NURBS handling)
    // 3. Creates quads between corresponding rail points (follows surface curvature)
    // 4. Does NOT add interior Steiner points (no orphan vertices, no cross-face
    //    vertex contamination that earcutr+Steiner can cause)
    //
    // This is the PREFERRED path for ruled NURBS because earcutr+Steiner can
    // produce triangles that span across the surface's intended domain when
    // the boundary UV polygon has a specific shape (e.g., a rectangle in UV
    // space that maps to a half-cylinder in 3D).
    if holes_3d.is_empty() {
        if let Surface::Nurbs(ref nurbs) = surface {
            if nurbs.u_degree == 1 || nurbs.v_degree == 1 {
                log::warn!(
                    "NURBS_CDT: trying strip for face {} (u_deg={}, v_deg={}, bnd={})",
                    face.id, nurbs.u_degree, nurbs.v_degree, boundary_3d.len()
                );
                if let Some(strip_mesh) = try_strip_triangulation_ruled_nurbs(
                    nurbs,
                    &boundary_3d,
                    &boundary_uvs,
                    face.forward,
                ) {
                    log::warn!(
                        "NURBS_CDT: strip returned {} verts, {} tris",
                        strip_mesh.vertices.len(), strip_mesh.triangles.len()
                    );
                    if !strip_mesh.vertices.is_empty() {
                        return strip_mesh;
                    }
                } else {
                    log::warn!("NURBS_CDT: strip returned None — falling back to earcutr");
                }
            }
        }
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

    // If the consistent path produced an empty mesh (or a mesh with vertices
    // but 0 triangles — which happens when the UV polygon is degenerate and
    // the 3D ear-clip fallback also fails), fall back to generic surface.
    // The generic surface path samples NURBS on a regular grid and is
    // guaranteed to produce non-empty output.
    if result.vertices.is_empty() || result.triangles.is_empty() {
        log::warn!(
            "NURBS CDT fallback: consistent triangulation produced {} verts / {} tris (empty={}), using generic surface",
            result.vertices.len(), result.triangles.len(),
            result.vertices.is_empty(),
        );
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
            let mut boundary_uvs = project_boundary_to_uv_chain(
                surface, boundary_points, u_period, v_period,
            );
            let mut hole_uvs: Vec<Vec<Point2d>> = hole_polylines.iter().map(|hole| {
                project_boundary_to_uv_chain(surface, hole, u_period, v_period)
            }).collect();

            // For torus (periodic in BOTH u and v): unwrap any remaining
            // periodic wrap that chain projection didn't resolve.
            // Chain projection adjusts each point to be close to the PREVIOUS
            // point, but if the boundary crosses the seam MULTIPLE times
            // (e.g., a half-torus face with full V wrap), the chain may
            // still produce values spanning > 1.9π in one axis.
            //
            // This detects such cases and shifts values > π by -2π to
            // produce a continuous range, allowing earcutr to triangulate
            // correctly.
            if let Surface::Torus(_) = surface {
                eprintln!("TORUS_PATH: triangulate_face_with_boundary_and_holes (non-UV) _ branch, n_bnd={}", boundary_points.len());
                let (b3d, buvs, hps, huvs) = unwrap_periodic_torus_boundary(
                    boundary_points, &boundary_uvs, hole_polylines, &hole_uvs,
                );
                // Replace with unwrapped versions
                boundary_uvs = buvs;
                hole_uvs = huvs;
                // b3d and hps are unchanged (3D points not affected by UV unwrap)
                let _ = b3d;
                let _ = hps;

                // Diagnostic: print unwrapped UV range
                let u_min = boundary_uvs.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                let u_max = boundary_uvs.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                let v_min = boundary_uvs.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                let v_max = boundary_uvs.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                eprintln!(
                    "TORUS_UNWRAPPED UVs: n={}, u=[{:.4},{:.4}], v=[{:.4},{:.4}]",
                    boundary_uvs.len(), u_min, u_max, v_min, v_max,
                );
                // Print first 10 and last 10 UVs
                for (i, uv) in boundary_uvs.iter().enumerate() {
                    if i < 10 || i >= boundary_uvs.len() - 10 {
                        eprintln!("  uv[{}]: ({:.4}, {:.4})", i, uv.u, uv.v);
                    }
                }
            }

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
    // DISABLED: decimate_collinear_boundary was removing boundary points that
    // are needed for watertightness. When two faces share an edge, both must
    // use the SAME boundary points. If one face decimates (removes intermediate
    // collinear points) and the other doesn't, the shared edge will have
    // different vertex counts, producing boundary edges.
    //
    // The decimation was originally added to reduce triangle count on planar
    // faces with many collinear boundary points. But the edge cache already
    // produces the minimum necessary points (via adaptive_discretize), so
    // decimation is not needed.
    let dec_3d = boundary_points.to_vec();
    let dec_uv = boundary_uvs.to_vec();
    let boundary_points: &[Point3d] = &dec_3d;
    let boundary_uvs: &[Point2d] = &dec_uv;

    let hole_polylines_decimated: Vec<Vec<Point3d>>;
    let hole_uvs_decimated: Vec<Vec<Point2d>>;
    let hole_polylines: &[Vec<Point3d>] = if !hole_polylines.is_empty() {
        let mut hps = Vec::with_capacity(hole_polylines.len());
        let mut huvs = Vec::with_capacity(hole_uvs.len());
        for (hp, huv) in hole_polylines.iter().zip(hole_uvs.iter()) {
            // No decimation — keep all boundary points for watertightness
            hps.push(hp.clone());
            huvs.push(huv.clone());
        }
        hole_polylines_decimated = hps;
        hole_uvs_decimated = huvs;
        &hole_polylines_decimated
    } else {
        #[allow(unused_assignments)]
        {
            hole_polylines_decimated = Vec::new();
            hole_uvs_decimated = Vec::new();
        }
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
            // Cylinder: detect tube face (full OR partial U-period wrap) and
            // use grid triangulation for watertightness. Otherwise use earcutr.
            //
            // FULL-wrap: boundary covers ≈2π of angular range (e.g., a full
            // cylinder lateral face with bottom + top full circles, no seam
            // edges). Detected by `is_full_u_period_wrap`.
            //
            // PARTIAL-wrap: boundary covers <2π of angular range (e.g., a
            // half-cylinder lateral face from a STEP file with 2 half-circle
            // arcs + 2 seam lines, covering π of angular range). The
            // `is_full_u_period_wrap` check FAILS for these, causing them to
            // fall through to earcutr which produces TWISTED triangles
            // because the UV polygon self-intersects when the top arc is
            // parameterized to go around the "long way" (e.g., from u=0
            // through u=3π/2 to u=π, instead of the "short way" through
            // u=π/2).
            //
            // For partial-wrap, we detect via `split_boundary_into_rings_with_u`:
            // if both bottom_ring and top_ring have ≥3 points (i.e., the face
            // has 2 distinct v-levels with arcs at each), it's a tube face
            // and we use `triangulate_cylinder_tube_from_boundary` which now
            // handles both full and partial wrap.
            if hole_polylines.is_empty() && boundary_points.len() >= 6 {
                if is_full_u_period_wrap(boundary_uvs, 2.0 * PI) {
                    log::info!(
                        "Cylinder face: full U-period wrap detected ({} bnd pts, u-range≈2π) — using tube grid triangulation",
                        boundary_points.len()
                    );
                    return triangulate_cylinder_tube_from_boundary(cyl, params, boundary_points, forward);
                }
                // Check for PARTIAL-wrap tube face: both bottom_ring and
                // top_ring must have ≥3 points (i.e., the boundary has arcs
                // at two distinct v-levels, plus seam lines connecting them).
                let (v_min_pw, v_max_pw) = compute_axis_v_range_pts(boundary_points, &cyl.origin, &cyl.axis);
                if v_max_pw > v_min_pw {
                    let dedup_tol = params.max_deviation.max(1e-6) * 0.5;
                    let (bottom_ring, top_ring, _) = split_boundary_into_rings_with_u(
                        boundary_points, &cyl.origin, &cyl.axis, &cyl.x_dir,
                        v_min_pw, v_max_pw, dedup_tol,
                    );
                    if bottom_ring.len() >= 3 && top_ring.len() >= 3 {
                        log::info!(
                            "Cylinder face: PARTIAL tube face detected ({} bottom + {} top ring points, {} bnd pts) — using tube grid triangulation",
                            bottom_ring.len(), top_ring.len(), boundary_points.len()
                        );
                        return triangulate_cylinder_tube_from_boundary(cyl, params, boundary_points, forward);
                    }
                }
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
            // Cone: detect tube face (full OR partial U-period wrap) similarly
            // to the cylinder case. See the cylinder arm for detailed comments.
            //
            // PARTIAL-wrap cone faces come from STEP files where a cone
            // surface is split into 2 half-cone faces (each covering π of
            // angular range, with 2 half-circle arc edges + 2 seam line
            // edges). Without this detection, they fall through to earcutr
            // which produces twisted triangles due to UV polygon self-
            // intersection when the top arc is parameterized the "long way".
            //
            // Cone apex degeneracy is handled inside
            // `triangulate_cone_tube_from_boundary` — when v_max reaches
            // the apex height, the top row collapses to a single apex vertex.
            if hole_polylines.is_empty() && boundary_points.len() >= 6 {
                if is_full_u_period_wrap(boundary_uvs, 2.0 * PI) {
                    log::info!(
                        "Cone face: full U-period wrap detected ({} bnd pts, u-range≈2π) — using tube grid triangulation",
                        boundary_points.len()
                    );
                    return triangulate_cone_tube_from_boundary(cone, params, boundary_points, forward);
                }
                // Check for PARTIAL-wrap tube face.
                let (v_min_pw, v_max_pw) = compute_axis_v_range_pts(boundary_points, &cone.origin, &cone.axis);
                if v_max_pw > v_min_pw {
                    let dedup_tol = params.max_deviation.max(1e-6) * 0.5;
                    let (bottom_ring, top_ring, _) = split_boundary_into_rings_with_u(
                        boundary_points, &cone.origin, &cone.axis, &cone.x_dir,
                        v_min_pw, v_max_pw, dedup_tol,
                    );
                    if bottom_ring.len() >= 3 && top_ring.len() >= 3 {
                        log::info!(
                            "Cone face: PARTIAL tube face detected ({} bottom + {} top ring points, {} bnd pts) — using tube grid triangulation",
                            bottom_ring.len(), top_ring.len(), boundary_points.len()
                        );
                        return triangulate_cone_tube_from_boundary(cone, params, boundary_points, forward);
                    }
                }
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
        Surface::Nurbs(ref nurbs) => {
            // NURBS triangulation strategy:
            //
            // For RULED NURBS (degree 1 in one direction), use strip triangulation
            // as the PRIMARY approach. Strip triangulation:
            // 1. Uses ALL boundary points from the edge cache (watertight by construction)
            // 2. Evaluates the NURBS surface at rail points (proper NURBS handling)
            // 3. Creates quads between corresponding rail points (follows surface curvature)
            // 4. Does NOT add interior Steiner points (no orphan vertices)
            //
            // The strip approach is "proper NURBS handling" because:
            // - It uses the actual NURBS surface evaluation (point_at) for all vertices
            // - It preserves the surface curvature (rails follow the NURBS curve)
            // - It produces deterministic, watertight results
            //
            // For NON-RULED NURBS (both degrees > 1), fall back to earcutr CDT
            // with curvature-adaptive interior Steiner points.
            //
            // If strip triangulation fails (e.g., can't find 4 corners), fall back
            // to earcutr as well.
            if nurbs.u_degree == 1 || nurbs.v_degree == 1 {
                if let Some(strip_mesh) = try_strip_triangulation_ruled_nurbs(
                    nurbs,
                    boundary_points,
                    boundary_uvs,
                    forward,
                ) {
                    return strip_mesh;
                }
            }

            // Fall back to earcutr-based consistent triangulation for non-ruled NURBS
            // or when strip triangulation fails.
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
            //
            // For torus (periodic in BOTH u and v): unwrap any periodic
            // wrap in the pre-computed UVs. PCURVE-based UVs may have
            // values spanning > 1.9π in one axis when the boundary crosses
            // the seam multiple times (e.g., half-torus with full V wrap).
            let (boundary_points_eff, boundary_uvs_eff, hole_polylines_eff, hole_uvs_eff) =
                if let Surface::Torus(_) = surface {
                    eprintln!("TORUS_PATH: triangulate_face_with_boundary_and_holes_uv (UV) _ branch, n_bnd={}", boundary_points.len());
                    let result = unwrap_periodic_torus_boundary(
                        boundary_points, boundary_uvs, hole_polylines, hole_uvs,
                    );
                    // Print unwrapped UV range
                    let u_min = result.1.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                    let u_max = result.1.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                    let v_min = result.1.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                    let v_max = result.1.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                    eprintln!(
                        "TORUS_UNWRAPPED_RESULT: n={}, u=[{:.4},{:.4}], v=[{:.4},{:.4}]",
                        result.1.len(), u_min, u_max, v_min, v_max,
                    );
                    // Print first 5 UVs
                    for (i, uv) in result.1.iter().enumerate().take(5) {
                        eprintln!("  uv[{}]: ({:.4}, {:.4})", i, uv.u, uv.v);
                    }
                    // Print UVs around index 80-90 (where the wrap might occur)
                    for i in 80..result.1.len().min(95) {
                        eprintln!("  uv[{}]: ({:.4}, {:.4})", i, result.1[i].u, result.1[i].v);
                    }
                    result
                } else {
                    (
                        boundary_points.to_vec(),
                        boundary_uvs.to_vec(),
                        hole_polylines.to_vec(),
                        hole_uvs.to_vec(),
                    )
                };
            crate::parametric_domain::triangulate_surface_consistent(
                surface,
                &boundary_points_eff,
                &boundary_uvs_eff,
                &hole_polylines_eff,
                &hole_uvs_eff,
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
    _boundary_uvs: &[Point2d],
    hole_polylines: &[Vec<Point3d>],
    _hole_uvs: &[Vec<Point2d>],
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

    // Deduplicate boundary points — close points create degenerate triangles.
    // The boundary points come from sampling multiple edges; consecutive edges
    // share endpoints, but floating-point differences can leave near-duplicates
    // (1-2 ULP apart). Without dedup, a 4-corner polygon becomes 5 vertices,
    // producing 3 triangles instead of 2 (with 2 being degenerate).
    //
    // We use 1e-9 tolerance to catch ULP-level differences without removing
    // valid vertices on small geometries. UVs are re-projected below from the
    // deduped 3D points, so we don't need to keep them in sync here.
    let boundary_points: Vec<Point3d> = deduplicate_points_3d(boundary_points, 1e-9);
    if boundary_points.len() < 3 {
        log::warn!(
            "PLANE_UV_TRI: after dedup, only {} points (< 3) — returning empty mesh",
            boundary_points.len()
        );
        return mesh;
    }

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
            for p in &boundary_points {
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
            for p in &boundary_points {
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
    let n_north = orient_normal(sphere.normal_at(0.0, 0.0), forward);
    let north_idx = mesh.add_vertex(p_north);
    mesh.add_vertex_normal(north_idx, [n_north.x, n_north.y, n_north.z]);

    // Ring vertices (rows 1..n_v-1) — exclude the poles
    for j in 1..n_v {
        let v = PI * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let p = sphere.point_at(u, v);
            let n = orient_normal(sphere.normal_at(u, v), forward);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // South pole vertex
    let p_south = sphere.point_at(0.0, PI);
    let n_south = orient_normal(sphere.normal_at(0.0, PI), forward);
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
pub(crate) fn surface_u_period(surface: &Surface) -> Option<f64> {
    match surface {
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) => {
            Some(2.0 * PI)
        }
        _ => None,
    }
}

/// Get the period of the surface's v parameter, if periodic.
pub(crate) fn surface_v_period(surface: &Surface) -> Option<f64> {
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

    // 2. Normalize UV polygon for periodic surfaces.
    // P3: Use PolyBoundary to bundle the polygon + periods together.
    // This is the same normalize_uv_polygon call as before, just packaged
    // so future callers don't have to thread u_period/v_period manually.
    let u_period = surface_u_period(surface);
    let v_period = surface_v_period(surface);
    let mut outer_pb = crate::poly_boundary::PolyBoundary::with_periods(boundary_uv, u_period, v_period);
    outer_pb.normalize();
    boundary_uv = outer_pb.into_polygon();
    // Also normalize hole polygons
    for hole_uv in hole_uv_polylines.iter_mut() {
        let mut hole_pb = crate::poly_boundary::PolyBoundary::with_periods(std::mem::take(hole_uv), u_period, v_period);
        hole_pb.normalize();
        *hole_uv = hole_pb.into_polygon();
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
            let n = orient_normal(cyl.normal_at(u, v), forward);
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
            let n = orient_normal(cone.normal_at(u, v), forward);
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

    let apex_v = cone.apex_v();

    // Handle degenerate cone: infinite or zero height (non-expanding).
    // Expanding cones always have infinite height — that's normal for them.
    if !cone.expanding && (!apex_v.is_finite() || apex_v.abs() > 1e6) {
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
    if v_range < apex_v.abs() * 0.001 + 1e-6 && !cone.expanding {
        return triangulate_cap_face(&Surface::Cone(cone.clone()), boundary_points, forward);
    }

    // Handle degenerate u range (boundary doesn't constrain u → full circle)
    let full_circle = u_range < 0.5 * PI || u_range > 1.9 * PI;
    if full_circle {
        u_min = 0.0;
        u_max = 2.0 * PI;
    }

    // Clamp v_min to apex v value (apex is at negative v for standard cones)
    if !cone.expanding && apex_v.is_finite() {
        v_min = v_min.max(apex_v);
    }
    let top_at_apex = !cone.expanding && apex_v.is_finite() && (v_min - apex_v).abs() < apex_v.abs() * 0.05 + 1e-6;

    // Add small margin
    let margin_u = (u_max - u_min) * 0.001;
    let margin_v = (v_max - v_min) * 0.001;
    u_min -= margin_u; u_max += margin_u;
    v_min -= margin_v; v_max += margin_v;

    // Clamp v range for expanding cones (v_min = 0 at apex)
    if cone.expanding {
        v_min = v_min.max(0.0);
    }
    if !cone.expanding && apex_v.is_finite() {
        v_min = v_min.max(apex_v);
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
    let mut _apex_vertex: Option<u32> = None;
    let mut row_vertex_offset: Vec<u32> = Vec::with_capacity(n_v + 1);
    let mut row_vertex_count: Vec<usize> = Vec::with_capacity(n_v + 1);
    let mut total_vertices = 0u32;

    for j in 0..=n_v {
        let v = v_min + dv * j as f64;

        if top_at_apex && j == n_v {
            // Apex row — single vertex
            let p = cone.point_at(0.0, apex_v);
            let n = orient_normal(cone.normal_at(0.0, apex_v), forward);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            _apex_vertex = Some(idx);
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
                let n = orient_normal(cone.normal_at(u, v), forward);
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
    let mut _pole_vertex_north: Option<u32> = None;
    let mut _pole_vertex_south: Option<u32> = None;
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
            _pole_vertex_north = Some(idx);
            row_vertex_offset.push(idx);
            row_vertex_count.push(1);
            total_vertices += 1;
        } else if j == n_v && at_south_pole {
            // South pole — single vertex
            let p = sphere.point_at(0.0, PI);
            let n = sphere.normal_at(0.0, PI);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
            _pole_vertex_south = Some(idx);
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
                if v_min < v_max { Some((v_min, v_max)) } else { Some((cone.apex_v(), cone.apex_v() + cone.height().min(100.0))) }
            }
            Surface::Revolution(rev) => Some(rev.profile.param_range()),
            Surface::Extrusion(ext) => Some(ext.profile.param_range()),
            _ => Some((0.0, 1.0)),
        }
    } else {
        None
    }
}

/// Compute the v parameter range from a list of 3D boundary points projected
/// onto an axis. Used by partial-tube-face detection in
/// `triangulate_face_with_boundary_and_holes_uv` to determine v_min and v_max
/// before calling `split_boundary_into_rings_with_u`.
///
/// Returns (v_min, v_max). If `points` is empty, returns (0.0, 0.0).
fn compute_axis_v_range_pts(
    points: &[Point3d],
    origin: &Point3d,
    axis: &Direction3d,
) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for p in points {
        let v = (p.x - origin.x) * axis.x
              + (p.y - origin.y) * axis.y
              + (p.z - origin.z) * axis.z;
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if v_min > v_max {
        (0.0, 0.0)
    } else {
        (v_min, v_max)
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
                    let apex_v = cone.apex_v();
                    let base_pt = cone.point_at(0.0, 0.0);
                    let apex_pt = cone.point_at(0.0, apex_v);
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
            let _adaptive_tol = self.cache.adaptive_tolerance().merge_tolerance();
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
    // NOTE: covariance matrix computed below is not currently consumed —
    // the function falls back to a simpler cross-product approach for the
    // plane normal. The covariance code is kept for future SVD-based fitting.
    #[allow(unused_assignments, unused_variables)]
    {
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
        let _ = (cov_xx, cov_xy, cov_xz, cov_yy, cov_yz, cov_zz);
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

#[cfg(test)]
mod adaptive_lod_tests {
    use super::*;

    #[test]
    fn test_adaptive_lod_budget_computation() {
        // With LOD 1.0 (detail_level=1.0) and 100 faces:
        // total_budget = 1.0 * 100_000 = 100_000
        // per_face = 100_000 / 100 = 1000
        let mut params = TriangulationParams::for_lod(1.0);
        params.adaptive_lod_enabled = true;
        let budget = params.compute_target_triangles_per_face(100);
        assert_eq!(budget, Some(1000));
    }

    #[test]
    fn test_adaptive_lod_low_detail() {
        // With LOD 0.5 (detail_level=0.625) and 200 faces:
        // total_budget = 0.625 * 100_000 = 62_500
        // per_face = 62_500 / 200 = 312
        let mut params = TriangulationParams::for_lod(0.5);
        params.adaptive_lod_enabled = true;
        let budget = params.compute_target_triangles_per_face(200);
        assert_eq!(budget, Some(312));
    }

    #[test]
    fn test_adaptive_lod_minimum_floor() {
        // With very many faces, per-face budget should never go below 4
        let mut params = TriangulationParams::for_lod(0.1);
        params.adaptive_lod_enabled = true;
        let budget = params.compute_target_triangles_per_face(1_000_000);
        assert!(budget.unwrap() >= 4, "budget should be at least 4, got {:?}", budget);
    }

    #[test]
    fn test_adaptive_lod_capped_by_max_face_triangles() {
        // With very few faces, budget should not exceed max_face_triangles
        let mut params = TriangulationParams::for_lod(1.0);
        params.adaptive_lod_enabled = true;
        let max_ft = params.max_face_triangles;
        let budget = params.compute_target_triangles_per_face(5);
        // 100_000 / 5 = 20_000, but max_face_triangles at LOD 1.0 = 8000
        assert_eq!(budget, Some(max_ft));
        assert!(budget.unwrap() <= max_ft);
    }

    #[test]
    fn test_adaptive_lod_disabled_returns_none() {
        let params = TriangulationParams::for_lod(1.0);
        // adaptive_lod_enabled = false by default
        let budget = params.compute_target_triangles_per_face(100);
        assert_eq!(budget, None);
    }

    #[test]
    fn test_with_adaptive_lod_sets_fields() {
        let mut params = TriangulationParams::for_lod(0.5);
        assert!(!params.adaptive_lod_enabled);
        assert_eq!(params.target_triangles_per_face, None);

        params.with_adaptive_lod(100);

        assert!(params.adaptive_lod_enabled);
        assert!(params.target_triangles_per_face.is_some());
        assert_eq!(params.keep_ratio, 1.0); // No decimation when adaptive
        // max_face_triangles should be capped to target
        assert_eq!(params.max_face_triangles, params.target_triangles_per_face.unwrap());
    }

    #[test]
    fn test_adaptive_lod_preview_quality() {
        // Preview (LOD 0.15, detail_level ≈ 0.36) with 50 faces:
        // total_budget ≈ 36_250, per_face ≈ 725
        let mut params = TriangulationParams::for_lod(0.15);
        params.adaptive_lod_enabled = true;
        let budget = params.compute_target_triangles_per_face(50);
        let expected = ((0.3625_f64 * 100_000.0).round() as usize / 50).max(4);
        assert_eq!(budget, Some(expected.min(params.max_face_triangles)));
    }

    #[test]
    fn test_default_params_no_adaptive_lod() {
        let params = TriangulationParams::default();
        assert!(!params.adaptive_lod_enabled);
        assert_eq!(params.target_triangles_per_face, None);
        assert_eq!(params.keep_ratio, 1.0);
    }

    #[test]
    fn test_adaptive_lod_zero_face_count() {
        let mut params = TriangulationParams::for_lod(1.0);
        params.adaptive_lod_enabled = true;
        let budget = params.compute_target_triangles_per_face(0);
        assert_eq!(budget, None); // Guard against division by zero
    }

    // ── face_area_budget_multiplier tests (task 1.1.4) ─────────────

    #[test]
    fn test_face_area_budget_tiny_face() {
        // Face < 1% of bbox → 0.5× budget
        let profile = SteinerBudgetProfile::Desktop;
        let mult = profile.face_area_budget_multiplier(0.005);
        assert!((mult - 0.5).abs() < 1e-10,
            "Tiny face (<1%) should get 0.5× budget, got {}", mult);
    }

    #[test]
    fn test_face_area_budget_medium_face() {
        // Face 1%–25% of bbox → linear interpolation from 0.5× to 1.0×
        let profile = SteinerBudgetProfile::Desktop;
        let mult_at_1pct = profile.face_area_budget_multiplier(0.01);
        let mult_at_25pct = profile.face_area_budget_multiplier(0.25);
        assert!((mult_at_1pct - 0.5).abs() < 1e-10,
            "At 1% should be 0.5×, got {}", mult_at_1pct);
        assert!((mult_at_25pct - 1.0).abs() < 1e-10,
            "At 25% should be 1.0×, got {}", mult_at_25pct);
        // Midpoint (13%) should be ~0.75
        let mult_at_13pct = profile.face_area_budget_multiplier(0.13);
        assert!((mult_at_13pct - 0.75).abs() < 0.02,
            "At 13% should be ~0.75×, got {}", mult_at_13pct);
    }

    #[test]
    fn test_face_area_budget_large_face() {
        // Face > 25% of bbox → up to 2.0× budget
        let profile = SteinerBudgetProfile::Desktop;
        let mult_at_50pct = profile.face_area_budget_multiplier(0.50);
        let mult_at_100pct = profile.face_area_budget_multiplier(1.0);
        assert!(mult_at_50pct > 1.0 && mult_at_50pct < 2.0,
            "At 50% should be between 1.0 and 2.0, got {}", mult_at_50pct);
        assert!((mult_at_100pct - 2.0).abs() < 1e-10,
            "At 100% should be 2.0×, got {}", mult_at_100pct);
    }

    #[test]
    fn test_face_area_budget_zero_and_negative() {
        let profile = SteinerBudgetProfile::Desktop;
        // Zero → 0.5× (treated as tiny face)
        assert!((profile.face_area_budget_multiplier(0.0) - 0.5).abs() < 1e-10);
        // Negative → clamped to 0 → 0.5×
        assert!((profile.face_area_budget_multiplier(-0.5) - 0.5).abs() < 1e-10);
        // > 1.0 → clamped to 1.0 → 2.0×
        assert!((profile.face_area_budget_multiplier(5.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_face_area_budget_consistent_across_profiles() {
        // The multiplier formula is profile-independent (same for Desktop/Tablet/Mobile)
        let desktop = SteinerBudgetProfile::Desktop;
        let tablet = SteinerBudgetProfile::Tablet;
        let mobile = SteinerBudgetProfile::Mobile;
        for fraction in [0.005, 0.01, 0.05, 0.13, 0.25, 0.5, 1.0] {
            assert!((desktop.face_area_budget_multiplier(fraction)
                     - tablet.face_area_budget_multiplier(fraction)).abs() < 1e-10,
                "Desktop/Tablet mismatch at fraction={}", fraction);
            assert!((desktop.face_area_budget_multiplier(fraction)
                     - mobile.face_area_budget_multiplier(fraction)).abs() < 1e-10,
                "Desktop/Mobile mismatch at fraction={}", fraction);
        }
    }

    #[test]
    fn test_bbox_surface_area_propagation() {
        // Verify that bbox_surface_area is None by default and Some after
        // being set, and that it flows through for_lod() correctly.
        let mut params = TriangulationParams::for_lod(1.0);
        assert_eq!(params.bbox_surface_area, None,
            "Default bbox_surface_area should be None");
        params.bbox_surface_area = Some(1000.0);
        assert_eq!(params.bbox_surface_area, Some(1000.0));
    }
}
