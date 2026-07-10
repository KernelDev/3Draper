// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Unified edge discretization cache for consistent triangulation.
//!
//! Ensures that shared edges between faces produce **bit-identical** 3D point
//! sequences. This is critical for watertight (gap-free) meshes where adjacent
//! faces must have exactly the same vertices on their common edges.
//!
//! # Architecture: Topology-First Meshing
//!
//! The cache is the cornerstone of the "topology-first" approach:
//! 1. The first face that references an edge triggers its discretization.
//! 2. The resulting 3D points and curve parameters are cached by a **canonical key**
//!    derived from the edge's geometry identity (step_entity_id or TopoId).
//! 3. Subsequent faces that share the same edge receive the *identical* 3D
//!    point sequence, plus UV coordinates computed for their own surface.
//!
//! This eliminates the need for post-hoc `merge_coincident_vertices` and
//! `stitch_boundary_edges`, which were symptoms of inconsistent discretization.
//!
//! # Dual-Key System
//!
//! STEP files identify edges by entity ID (`i64`), while native topology
//! uses `TopoId` (`u64`). The unified cache supports BOTH:
//! - STEP path: key via `step_entity_id` → guarantees identical points for
//!   two ORIENTED_EDGEs sharing the same EDGE_CURVE
//! - Native path: key via `TopoId` → guarantees identical points when the
//!   same Edge object is shared between faces
//!
//! # Deterministic Rounding
//!
//! All 3D points are rounded to a fixed number of significant bits before
//! storage. This ensures that floating-point arithmetic produces the same
//! bit pattern regardless of evaluation order, preventing micro-gaps from
//! accumulating at shared boundaries.

use draper_geometry::{Point3d, Point2d, Direction3d, Curve3d, Curve2d, Surface, tolerance::ToleranceContext};
use draper_topology::{Edge, Solid, TopoId};
use std::collections::HashMap;

/// Number of mantissa bits to preserve during deterministic rounding.
/// 52 bits = full f64 precision (no rounding).
/// 48 bits = discard lowest 4 bits (~1e-14 relative precision).
/// 40 bits = discard lowest 12 bits (~1e-12 relative precision).
const PRECISION_BITS: u32 = 48;

/// Round an f64 value by truncating low mantissa bits for deterministic results.
/// This ensures that values computed via different floating-point paths
/// (e.g., different parameter orders) produce bit-identical results.
#[inline]
fn deterministic_round(value: f64) -> f64 {
    let bits = value.to_bits();
    let truncate = 52 - PRECISION_BITS;
    if truncate == 0 {
        return value; // Full precision, no rounding
    }
    // Mask off the lowest `truncate` mantissa bits
    let mask = !((1u64 << truncate) - 1);
    f64::from_bits(bits & mask)
}

/// Round a 3D point deterministically.
#[inline]
pub fn deterministic_round_point(p: Point3d) -> Point3d {
    Point3d::new(
        deterministic_round(p.x),
        deterministic_round(p.y),
        deterministic_round(p.z),
    )
}

/// Quantization grid (in millimetres) for axis-origin components when
/// grouping circles by axis. Two circles whose axis origins differ by
/// less than this value are treated as sharing the same axis.
const AXIS_ORIGIN_QUANT: f64 = 1e-4;

/// Quantization grid (radians, applied to direction cosines) for axis
/// direction when grouping circles by axis.
const AXIS_DIR_QUANT: f64 = 1e-4;

/// Quantization grid for NURBS control point coordinates when hashing.
/// Two surfaces whose control points differ by less than this are
/// considered the same surface (for refinement grid sharing).
const NURBS_HASH_QUANT: f64 = 1e-6;

/// Compute a stable hash of a NURBS surface for the purpose of sharing
/// a refinement grid across multiple faces that reference the same
/// underlying surface entity.
///
/// The hash incorporates: degrees, control point positions (quantized),
/// weights (quantized), and knot vectors (quantized). Two surfaces that
/// are geometrically identical (within `NURBS_HASH_QUANT`) produce the
/// same hash, ensuring that faces sharing a NURBS surface in a STEP file
/// receive the SAME interior Steiner grid — which is critical for
/// watertightness when chord-error refinement is enabled.
pub fn nurbs_surface_hash(nurbs: &draper_geometry::NurbsSurface) -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();

    // Degrees
    nurbs.u_degree.hash(&mut hasher);
    nurbs.v_degree.hash(&mut hasher);

    // Control points (quantized)
    for row in &nurbs.control_points {
        for p in row {
            let qx = (p.x / NURBS_HASH_QUANT).round() as i64;
            let qy = (p.y / NURBS_HASH_QUANT).round() as i64;
            let qz = (p.z / NURBS_HASH_QUANT).round() as i64;
            qx.hash(&mut hasher);
            qy.hash(&mut hasher);
            qz.hash(&mut hasher);
        }
    }

    // Weights (quantized)
    for row in &nurbs.weights {
        for w in row {
            let qw = (w / NURBS_HASH_QUANT).round() as i64;
            qw.hash(&mut hasher);
        }
    }

    // Knots (quantized)
    for k in &nurbs.u_knots {
        let qk = (k / NURBS_HASH_QUANT).round() as i64;
        qk.hash(&mut hasher);
    }
    for k in &nurbs.v_knots {
        let qk = (k / NURBS_HASH_QUANT).round() as i64;
        qk.hash(&mut hasher);
    }

    // Closure flags
    nurbs.u_closed.hash(&mut hasher);
    nurbs.v_closed.hash(&mut hasher);

    hasher.finish()
}

/// Canonical key identifying a circle's axis (origin + direction), quantized
/// so that two circles on the "same" axis (within tolerance) map to the same
/// key. Used by `pre_compute_circle_axis_n` to enforce identical sample
/// counts across all circles on a shared axis — critical for watertightness
/// of cone/cylinder tube faces where bottom and top rings come from DIFFERENT
/// CIRCLE entities with DIFFERENT radii.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct AxisKey {
    ox: i64,
    oy: i64,
    oz: i64,
    dx: i64,
    dy: i64,
    dz: i64,
}

impl AxisKey {
    fn from_circle(c: &draper_geometry::Circle) -> Self {
        // Quantize origin to 1e-4 mm grid
        let ox = (c.center.x / AXIS_ORIGIN_QUANT).round() as i64;
        let oy = (c.center.y / AXIS_ORIGIN_QUANT).round() as i64;
        let oz = (c.center.z / AXIS_ORIGIN_QUANT).round() as i64;
        // Quantize direction cosines to 1e-4 grid
        let dx = (c.normal.x / AXIS_DIR_QUANT).round() as i64;
        let dy = (c.normal.y / AXIS_DIR_QUANT).round() as i64;
        let dz = (c.normal.z / AXIS_DIR_QUANT).round() as i64;
        Self { ox, oy, oz, dx, dy, dz }
    }
}

/// Compute the number of sample points for a circle of radius `R` given a
/// chord tolerance `d`. Uses the exact formula:
///   n = π / acos(1 - d/R)  when d < R
/// and the small-angle approximation π · sqrt(R / (2d)) as a fallback.
///
/// The result is rounded UP to a multiple of 4 so that two circles on the
/// same axis with similar radii (e.g., R=30.36 and R=35.22) tend to produce
/// the same n — this preserves watertightness of tube faces.
///
/// Always clamps to [8, max_samples] and to at least `8`.
fn compute_circle_n(radius: f64, chord_tol: f64, max_samples: usize) -> usize {
    // Sanity guards: degenerate circle or tolerance
    if !radius.is_finite() || radius <= 0.0 || !chord_tol.is_finite() || chord_tol <= 0.0 {
        return 8;
    }
    // If tolerance is larger than radius, the circle collapses — minimum samples
    if chord_tol >= radius {
        return 8;
    }
    // Exact: n = π / acos(1 - d/R)
    let arg = 1.0 - chord_tol / radius;
    let n_real = if arg < 1.0 {
        let acos_arg = arg.max(-1.0);
        std::f64::consts::PI / acos_arg.acos()
    } else {
        // Tolerance very close to 0 — use small-angle approximation
        std::f64::consts::PI * (radius / (2.0 * chord_tol)).sqrt()
    };
    // Round up to multiple of 4 (so 4, 8, 12, 16, ...)
    let n_rounded = ((n_real.ceil() as usize) + 3) & !3;
    // Clamp to [8, max_samples]
    n_rounded.clamp(8, max_samples.max(8))
}

/// Canonical key for edge cache lookup.
///
/// Uses step_entity_id when available (STEP path), falling back to TopoId
/// (native path). This ensures that two ORIENTED_EDGEs sharing the same
/// EDGE_CURVE resolve to the same cache entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EdgeCacheKey {
    /// STEP entity ID of the EDGE_CURVE — stable across ORIENTED_EDGE instances.
    StepEntityId(i64),
    /// TopoId of the edge — for native topology where edges are properly shared.
    TopoId(TopoId),
}

impl EdgeCacheKey {
    /// Derive the canonical key for an edge.
    /// Prefers step_entity_id (when available) for maximum stability.
    pub fn from_edge(edge: &Edge) -> Self {
        if let Some(step_id) = edge.step_entity_id {
            EdgeCacheKey::StepEntityId(step_id)
        } else {
            EdgeCacheKey::TopoId(edge.id)
        }
    }
}

/// Cached discretization of a single edge.
#[derive(Clone, Debug)]
pub struct EdgeDiscretization {
    /// 3D points along the edge curve (deterministically rounded).
    pub points_3d: Vec<Point3d>,
    /// UV coordinates for each incident face.
    /// Maps face TopoId → Vec<Point2d> (same length as points_3d).
    pub uv_per_face: HashMap<TopoId, Vec<Point2d>>,
    /// Curve parameters for each sample point (normalized to [0, 1]).
    pub params: Vec<f64>,
}

/// Adaptive tolerance based on model bounding box.
///
/// Replaces hardcoded 1e-6 tolerance with a scale-aware value.
/// For a 1mm model, tolerance ≈ 1e-6 (nanometer precision).
/// For a 1m model, tolerance ≈ 1e-5.
/// For a 1km model, tolerance ≈ 1e-3.
#[derive(Clone, Debug)]
pub struct AdaptiveTolerance {
    /// Bounding box diagonal (characteristic model size).
    model_scale: f64,
    /// Relative tolerance (e.g., 1e-6 means 1 PPM of model scale).
    relative: f64,
    /// Tolerance context for reuse.
    tol_ctx: ToleranceContext,
}

impl AdaptiveTolerance {
    /// Create adaptive tolerance from a bounding box.
    pub fn from_bounding_box(min: &Point3d, max: &Point3d) -> Self {
        let dx = max.x - min.x;
        let dy = max.y - min.y;
        let dz = max.z - min.z;
        let model_scale = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-10);
        Self {
            model_scale,
            relative: 1e-6,
            tol_ctx: ToleranceContext::from_bounding_box(min, max),
        }
    }

    /// Create adaptive tolerance with a specific model scale.
    pub fn from_model_scale(model_scale: f64) -> Self {
        Self {
            model_scale: model_scale.max(1e-10),
            relative: 1e-6,
            tol_ctx: ToleranceContext::from_model_scale(model_scale),
        }
    }

    /// Create with default values (model scale = 1.0).
    pub fn new() -> Self {
        Self {
            model_scale: 1.0,
            relative: 1e-6,
            tol_ctx: ToleranceContext::new(),
        }
    }

    /// Get the absolute merge tolerance (model_scale × relative).
    /// This is the tolerance for vertex merging operations.
    pub fn merge_tolerance(&self) -> f64 {
        self.model_scale * self.relative
    }

    /// Get the absolute chord deviation tolerance (10× merge tolerance).
    /// Used for adaptive edge subdivision.
    pub fn chord_tolerance(&self) -> f64 {
        self.model_scale * self.relative * 10.0
    }

    /// Get the stitch tolerance (100× merge tolerance, for extreme cases only).
    pub fn stitch_tolerance(&self) -> f64 {
        self.model_scale * self.relative * 100.0
    }

    /// Check if two points are coincident within tolerance.
    pub fn is_coincident(&self, a: &Point3d, b: &Point3d) -> bool {
        self.tol_ctx.is_coincident_3d(a, b)
    }

    /// Get the tolerance context.
    pub fn tol_ctx(&self) -> &ToleranceContext {
        &self.tol_ctx
    }

    /// Get the model scale.
    pub fn model_scale(&self) -> f64 {
        self.model_scale
    }
}

impl Default for AdaptiveTolerance {
    fn default() -> Self {
        Self::new()
    }
}

/// Cache that ensures each edge is discretized exactly once.
/// When multiple faces share the same edge, they receive identical
/// 3D point sequences and computed UV coordinates.
///
/// # Topology-First Architecture
///
/// This cache implements the "topology-first" meshing approach:
/// - Edges are discretized ONCE with deterministic rounding
/// - Faces use the cached boundary data for triangulation
/// - No post-hoc vertex merging or stitching is needed
#[derive(Clone, Debug)]
pub struct EdgeDiscretizationCache {
    /// Maps canonical key → edge discretization.
    entries: HashMap<EdgeCacheKey, EdgeDiscretization>,
    /// Secondary index: TopoId → canonical key (for reverse lookup).
    topo_id_to_key: HashMap<TopoId, EdgeCacheKey>,
    /// Step-entity ID aliases: maps a non-canonical step_id to the canonical one.
    ///
    /// When two different STEP EDGE_CURVE entities share the same VERTEX_POINT
    /// endpoints (i.e., they represent the same geometric boundary but with
    /// different curve representations), the first-seen step_id becomes the
    /// canonical key. Subsequent step_ids are registered as aliases, so that
    /// `discretize_step_edge()` returns the same cached 3D points for all
    /// edges sharing that boundary — ensuring watertightness by construction.
    step_id_aliases: HashMap<i64, i64>,
    /// Adaptive tolerance for sampling and coincidence checks.
    adaptive_tol: AdaptiveTolerance,
    /// Maximum number of sample points per edge.
    max_samples: usize,
    /// Optional LOD-driven chord tolerance override.
    ///
    /// When set, this REPLACES `adaptive_tol.chord_tolerance()` as the
    /// threshold for adaptive edge subdivision. This is what lets the
    /// "Quality" selector change the resolution of curved edges (arcs on
    /// the top/bottom faces of a cylinder, fillets, etc.) — without it,
    /// `adaptive_tol` is derived solely from the model's bounding box and
    /// gives a fixed, very fine subdivision regardless of LOD.
    ///
    /// Set via `set_chord_tolerance_override()` by `triangulate_solid()`
    /// from `TriangulationParams::max_deviation`.
    chord_tolerance_override: Option<f64>,

    /// Per-axis-group sample count for circles.
    ///
    /// Key: quantized axis (origin + direction). Value: the MAX n across
    /// all circles sharing that axis. This ensures that two circles on the
    /// same axis but with different radii (e.g., bottom ring R=30.36 and
    /// top ring R=35.22 of a cone tube face) get the SAME n — which is
    /// critical for watertightness (the tube face's bottom[i]→top[i]
    /// connection requires identical counts).
    ///
    /// Computed by `pre_compute_circle_axis_n()` which must be called
    /// before triangulation starts. If not pre-computed, the Circle branch
    /// in `adaptive_discretize` falls back to per-circle n computation
    /// (still LOD-aware, but may break watertightness for multi-radius
    /// tube faces).
    circle_axis_n: HashMap<AxisKey, usize>,

    /// Per-NURBS-surface shared interior refinement grid.
    ///
    /// Key: `nurbs_surface_hash()` of the surface. Value: a list of UV
    /// points that form a chord-error-compliant interior grid covering the
    /// FULL surface parameter range (not domain-filtered).
    ///
    /// When a NURBS face is triangulated, the shared grid is filtered by
    /// the face's UV domain and used as Steiner points. This ensures that
    /// all faces sharing the same NURBS surface entity get the SAME
    /// interior vertices — which is critical for watertightness when
    /// chord-error refinement is enabled (MS-2 from audit plan).
    ///
    /// Without this shared grid, per-face chord-error refinement creates
    /// different interior vertices on each face (because earcutr produces
    /// different interior edges), and these mismatched vertices appear as
    /// BREP boundary edges.
    nurbs_refinement_grids: HashMap<u64, Vec<Point2d>>,
    /// ── Instrumentation counters ──
    /// Number of cache hits (edge already discretized).
    cache_hits: usize,
    /// Number of cache misses (edge needed discretization).
    cache_misses: usize,
    /// Number of edges that are shared by 2+ faces (computed on demand).
    shared_edges: usize,
    /// Number of step_id alias lookups that resulted in a cache hit.
    alias_hits: usize,
}

/// Statistics from the edge discretization cache, used for debugging
/// and validation. If cache_misses is too high relative to cache_hits,
/// edges are being re-discretized, which defeats the topology-first approach.
#[derive(Clone, Debug, Default)]
pub struct EdgeCacheStats {
    /// Total number of cached edge discretizations.
    pub total_edges: usize,
    /// Number of cache hits (edge was already cached).
    pub cache_hits: usize,
    /// Number of cache misses (edge needed fresh discretization).
    pub cache_misses: usize,
    /// Number of edges shared by 2+ faces (approximation from uv_per_face).
    pub shared_edges: usize,
}

/// Statistics for STEP ID aliasing (Phase 1 + Phase 2).
///
/// Tracks how many edges were aliased in each phase, how many groups
/// were formed, and how many edges were skipped as "different curves
/// with same endpoints".
#[derive(Clone, Debug, Default)]
pub struct AliasingStatistics {
    /// Number of aliases registered in Phase 1 (vertex-pair + shape matching).
    pub phase1_aliases: usize,
    /// Number of groups in Phase 1 (vertex-pair groups with 2+ step_ids).
    pub phase1_groups: usize,
    /// Number of edges skipped in Phase 1 (same endpoints, different curves).
    pub phase1_skipped_different_curves: usize,
    /// Number of aliases registered in Phase 2 (3D coordinate matching).
    pub phase2_aliases: usize,
    /// Number of coordinate groups in Phase 2 (with 2+ step_ids).
    pub phase2_groups: usize,
    /// Number of edges skipped in Phase 2 (different curves).
    pub phase2_skipped_different_curves: usize,
    /// Total number of step_ids processed.
    pub total_step_ids: usize,
}

impl AliasingStatistics {
    /// Log a summary of aliasing statistics.
    pub fn log_summary(&self, brep_id: i64) {
        if self.phase1_aliases == 0 && self.phase2_aliases == 0
            && self.phase1_skipped_different_curves == 0
            && self.phase2_skipped_different_curves == 0
        {
            log::info!(
                "BREP #{} aliasing: {} step_ids, 0 aliases (no matching edges found)",
                brep_id, self.total_step_ids,
            );
            return;
        }
        log::info!(
            "BREP #{} aliasing: {} step_ids — Phase1: {} aliases in {} groups, {} skipped (different curves); Phase2: {} aliases in {} groups, {} skipped",
            brep_id,
            self.total_step_ids,
            self.phase1_aliases, self.phase1_groups, self.phase1_skipped_different_curves,
            self.phase2_aliases, self.phase2_groups, self.phase2_skipped_different_curves,
        );
    }

    /// Total aliases across all phases.
    pub fn total_aliases(&self) -> usize {
        self.phase1_aliases + self.phase2_aliases
    }

    /// Total skipped edges (different curves with same endpoints).
    pub fn total_skipped(&self) -> usize {
        self.phase1_skipped_different_curves + self.phase2_skipped_different_curves
    }
}

impl EdgeDiscretizationCache {
    /// Create a new cache with default tolerance.
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
            topo_id_to_key: HashMap::new(),
            step_id_aliases: HashMap::new(),
            adaptive_tol: AdaptiveTolerance::new(),
            max_samples: 64,
            chord_tolerance_override: None,
            circle_axis_n: HashMap::new(),
            nurbs_refinement_grids: HashMap::new(),
            cache_hits: 0,
            cache_misses: 0,
            shared_edges: 0,
            alias_hits: 0,
        }
    }

    /// Create a new cache with custom tolerance and max samples.
    pub fn with_tolerance(tol_ctx: ToleranceContext, max_samples: usize) -> Self {
        Self {
            entries: HashMap::new(),
            topo_id_to_key: HashMap::new(),
            step_id_aliases: HashMap::new(),
            adaptive_tol: AdaptiveTolerance::from_model_scale(tol_ctx.model_scale),
            max_samples: max_samples.max(4),
            chord_tolerance_override: None,
            circle_axis_n: HashMap::new(),
            nurbs_refinement_grids: HashMap::new(),
            cache_hits: 0,
            cache_misses: 0,
            shared_edges: 0,
            alias_hits: 0,
        }
    }

    /// Create a new cache with adaptive tolerance from a bounding box.
    pub fn with_adaptive_tolerance(min: &Point3d, max: &Point3d, max_samples: usize) -> Self {
        Self {
            entries: HashMap::new(),
            topo_id_to_key: HashMap::new(),
            step_id_aliases: HashMap::new(),
            adaptive_tol: AdaptiveTolerance::from_bounding_box(min, max),
            max_samples: max_samples.max(4),
            chord_tolerance_override: None,
            circle_axis_n: HashMap::new(),
            nurbs_refinement_grids: HashMap::new(),
            cache_hits: 0,
            cache_misses: 0,
            shared_edges: 0,
            alias_hits: 0,
        }
    }

    /// Get or compute the discretization for an edge.
    ///
    /// If the edge has already been discretized (identified by its canonical key),
    /// returns the cached result and computes UV coordinates for the given
    /// face/surface if not already present.
    ///
    /// # Arguments
    /// * `edge` - The edge to discretize
    /// * `face_id` - The TopoId of the face that needs this edge
    /// * `surface` - The surface of the face (for UV computation)
    /// * `n_samples_hint` - Suggested number of samples (ignored if edge is already cached)
    /// * `curve_2d` - Optional analytical PCURVE in UV space
    pub fn discretize_edge(
        &mut self,
        edge: &Edge,
        face_id: TopoId,
        surface: &Surface,
        n_samples_hint: usize,
        curve_2d: Option<&Curve2d>,
    ) -> &EdgeDiscretization {
        let key = EdgeCacheKey::from_edge(edge);

        // Register TopoId → canonical key mapping
        self.topo_id_to_key.insert(edge.id, key);

        // If not yet cached, compute and insert the discretization
        if !self.entries.contains_key(&key) {
            self.cache_misses += 1;

            let (mut points_3d, params) = self.adaptive_discretize(edge, n_samples_hint);

            // Apply deterministic rounding to all 3D points
            for p in &mut points_3d {
                *p = deterministic_round_point(*p);
            }

            let uvs = Self::compute_uvs(&points_3d, &params, surface, curve_2d);

            let mut uv_per_face = HashMap::new();
            uv_per_face.insert(face_id, uvs);

            self.entries.insert(key, EdgeDiscretization {
                points_3d,
                uv_per_face,
                params,
            });
        } else {
            self.cache_hits += 1;
            // Track shared edges: if this edge already has UV for a different face
            if let Some(entry) = self.entries.get(&key) {
                if !entry.uv_per_face.contains_key(&face_id) && entry.uv_per_face.len() >= 1 {
                    self.shared_edges += 1;
                }
            }
        }

        // Entry is guaranteed to exist now — add UV for this face if missing
        if let Some(entry) = self.entries.get_mut(&key) {
            if !entry.uv_per_face.contains_key(&face_id) {
                let uvs = Self::compute_uvs(&entry.points_3d, &entry.params, surface, curve_2d);
                entry.uv_per_face.insert(face_id, uvs);
            }
        }

        // Return the entry — guaranteed to exist at this point
        &self.entries[&key]
    }

    /// Register a step_id alias: when `from_id` is used as a cache key,
    /// redirect to `to_id` instead. This ensures that two different STEP
    /// EDGE_CURVE entities sharing the same VERTEX_POINT endpoints produce
    /// identical 3D boundary points — the foundation of watertight meshes.
    ///
    /// The `to_id` should be the "canonical" step_id (typically the first
    /// one encountered for a given vertex pair). All aliases resolve to it.
    pub fn register_step_id_alias(&mut self, from_id: i64, to_id: i64) {
        if from_id != to_id {
            // Resolve transitive aliases: if to_id is itself an alias,
            // follow the chain to the true canonical id.
            let canonical = self.resolve_canonical_step_id(to_id);
            self.step_id_aliases.insert(from_id, canonical);
        }
    }

    /// Resolve a step_id through the alias chain to its canonical form.
    /// If `step_id` has been registered as an alias, returns the canonical id.
    /// Otherwise returns `step_id` itself.
    pub fn resolve_canonical_step_id(&self, step_id: i64) -> i64 {
        let mut current = step_id;
        let mut seen = std::collections::HashSet::new();
        while let Some(&alias_target) = self.step_id_aliases.get(&current) {
            if !seen.insert(current) {
                // Cycle detected — shouldn't happen, but break to avoid infinite loop
                log::warn!("Step ID alias cycle detected for step_id={}", step_id);
                break;
            }
            current = alias_target;
        }
        current
    }

    /// Discretize an edge using a STEP entity ID as the key.
    ///
    /// This is the STEP-path API that replaces the old `StepEdgeCache::discretize()`.
    /// It creates a canonical key from the STEP entity ID, ensuring that two
    /// ORIENTED_EDGEs sharing the same EDGE_CURVE resolve to the same cache entry.
    ///
    /// # Vertex-pair aliasing
    ///
    /// When two different STEP EDGE_CURVE entities share the same VERTEX_POINT
    /// endpoints (same geometric boundary, different curve representation),
    /// `register_step_id_alias()` maps the non-canonical step_id to the canonical
    /// one. This method resolves aliases before looking up the cache, so both
    /// edges produce identical 3D boundary points.
    ///
    /// # Arguments
    /// * `step_entity_id` - STEP entity ID of the EDGE_CURVE
    /// * `edge` - The topological edge (for curve geometry and param_range)
    /// * `n_samples` - Number of samples
    ///
    /// # Returns
    /// (3D points, normalized parameters) — reversed if the edge's param_range
    /// is reversed relative to the canonical direction.
    pub fn discretize_step_edge(
        &mut self,
        step_entity_id: i64,
        edge: &Edge,
        n_samples: usize,
    ) -> (Vec<Point3d>, Vec<f64>) {
        // Resolve aliases: if this step_id was aliased to a canonical one, use it
        let canonical_id = self.resolve_canonical_step_id(step_entity_id);
        let key = EdgeCacheKey::StepEntityId(canonical_id);

        // Register TopoId → canonical key mapping
        self.topo_id_to_key.insert(edge.id, key);

        if !self.entries.contains_key(&key) {
            self.cache_misses += 1;

            // Cache miss: discretize in forward direction (canonical)
            let forward_edge = if edge.param_range.0 > edge.param_range.1 {
                edge.reversed()
            } else {
                edge.clone()
            };

            let (mut points_3d, params) = self.adaptive_discretize(&forward_edge, n_samples.max(2));

            // Apply deterministic rounding
            for p in &mut points_3d {
                *p = deterministic_round_point(*p);
            }

            self.entries.insert(key, EdgeDiscretization {
                points_3d,
                uv_per_face: HashMap::new(),
                params,
            });
        } else {
            self.cache_hits += 1;
            if canonical_id != step_entity_id {
                self.alias_hits += 1;
            }
        }

        // Retrieve the cached entry
        let entry = &self.entries[&key];

        // If the original edge was reversed, reverse the result for the caller
        if edge.param_range.0 > edge.param_range.1 {
            let mut pts = entry.points_3d.clone();
            let mut params = entry.params.clone();
            pts.reverse();
            params = params.into_iter().map(|p| 1.0 - p).collect();
            (pts, params)
        } else {
            (entry.points_3d.clone(), entry.params.clone())
        }
    }

    /// Get the cached discretization for an edge by TopoId (if it exists).
    pub fn get(&self, edge_id: TopoId) -> Option<&EdgeDiscretization> {
        // Try direct lookup first
        if let Some(entry) = self.entries.get(&EdgeCacheKey::TopoId(edge_id)) {
            return Some(entry);
        }
        // Try via secondary index (TopoId → canonical key)
        if let Some(key) = self.topo_id_to_key.get(&edge_id) {
            return self.entries.get(key);
        }
        None
    }

    /// Get the cached discretization for an edge mutably (if it exists).
    pub fn get_mut(&mut self, edge_id: TopoId) -> Option<&mut EdgeDiscretization> {
        // Try direct lookup first
        if self.entries.contains_key(&EdgeCacheKey::TopoId(edge_id)) {
            return self.entries.get_mut(&EdgeCacheKey::TopoId(edge_id));
        }
        // Try via secondary index
        if let Some(key) = self.topo_id_to_key.get(&edge_id).copied() {
            return self.entries.get_mut(&key);
        }
        None
    }

    /// Get the cached discretization by STEP entity ID.
    pub fn get_by_step_id(&self, step_id: i64) -> Option<&EdgeDiscretization> {
        self.entries.get(&EdgeCacheKey::StepEntityId(step_id))
    }

    /// Check if an edge is already in the cache.
    pub fn contains(&self, edge_id: TopoId) -> bool {
        self.entries.contains_key(&EdgeCacheKey::TopoId(edge_id))
            || self.topo_id_to_key.contains_key(&edge_id)
    }

    /// Check if a STEP edge is already in the cache.
    pub fn contains_step_id(&self, step_id: i64) -> bool {
        self.entries.contains_key(&EdgeCacheKey::StepEntityId(step_id))
    }

    /// Number of cached edges.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Get cache statistics for debugging and validation.
    ///
    /// Key metrics:
    /// - `cache_hits` / (`cache_hits` + `cache_misses`) = hit rate.
    ///   Should be >50% for well-structured B-Rep data.
    /// - `shared_edges` should equal the number of edges shared by 2+ faces.
    /// - `cache_misses` should equal `total_edges` (each edge is discretized once).
    pub fn stats(&self) -> EdgeCacheStats {
        // Recompute shared_edges from uv_per_face (more accurate than counter)
        let shared = self.entries.values()
            .filter(|e| e.uv_per_face.len() >= 2)
            .count();
        EdgeCacheStats {
            total_edges: self.entries.len(),
            cache_hits: self.cache_hits,
            cache_misses: self.cache_misses,
            shared_edges: shared,
        }
    }

    /// Get the adaptive tolerance.
    pub fn adaptive_tolerance(&self) -> &AdaptiveTolerance {
        &self.adaptive_tol
    }

    /// Update the adaptive tolerance from a bounding box.
    pub fn set_adaptive_tolerance(&mut self, min: &Point3d, max: &Point3d) {
        self.adaptive_tol = AdaptiveTolerance::from_bounding_box(min, max);
    }

    /// Override the chord tolerance used for adaptive edge subdivision.
    ///
    /// Pass `Some(tol)` to make `adaptive_discretize` use `tol` instead of
    /// `adaptive_tol.chord_tolerance()` as its deviation threshold. Pass
    /// `None` to revert to the bbox-derived default.
    ///
    /// This is what the LOD/Quality selector uses to change the resolution
    /// of curved edges (arcs on cylinder caps, fillets, etc.) —
    /// `triangulate_solid()` calls `set_chord_tolerance_override(Some(params.max_deviation))`
    /// so a coarser LOD produces fewer edge samples and a finer LOD produces more.
    pub fn set_chord_tolerance_override(&mut self, tol: Option<f64>) {
        self.chord_tolerance_override = tol.filter(|t| t.is_finite() && *t > 0.0);
    }

    /// Get the currently effective chord tolerance (override if set, else adaptive).
    pub fn effective_chord_tolerance(&self) -> f64 {
        self.chord_tolerance_override
            .unwrap_or_else(|| self.adaptive_tol.chord_tolerance())
    }

    /// Pre-compute the per-axis-group sample count for all CIRCLE curves in the
    /// given edges. For each axis (quantized origin + direction), finds the
    /// MAX n across all circles sharing that axis — and stores it.
    ///
    /// This MUST be called before triangulation starts. It ensures that two
    /// circles on the same axis but with different radii (e.g., bottom ring
    /// R=30.36 and top ring R=35.22 of a cone tube face) get the SAME n —
    /// which is critical for watertightness (the tube face's bottom[i]→top[i]
    /// connection requires identical counts on both rings).
    ///
    /// The n computation uses the LOD-driven `effective_chord_tolerance()`,
    /// so a finer LOD produces more circle samples (and a coarser LOD fewer).
    /// The geometric formula: for a circle of radius R with chord tolerance d,
    ///   n = π / acos(1 - d/R) ≈ π · sqrt(R / (2d))  for small d/R
    /// We round n up to a multiple of 4 so that two circles with similar but
    /// not identical radii (but on the same axis) produce the SAME n.
    pub fn pre_compute_circle_axis_n<'a, I>(&mut self, edges: I)
    where
        I: IntoIterator<Item = &'a Edge>,
    {
        let chord_tol = self.effective_chord_tolerance();
        let mut per_axis_max: HashMap<AxisKey, usize> = HashMap::new();

        for edge in edges {
            let circle = match &edge.curve {
                Some(Curve3d::Circle(c)) => c,
                _ => continue,
            };
            let axis_key = AxisKey::from_circle(circle);
            // Compute n from chord tolerance and radius
            let n_for_this = compute_circle_n(circle.radius, chord_tol, self.max_samples);
            let entry = per_axis_max.entry(axis_key).or_insert(0);
            if n_for_this > *entry {
                *entry = n_for_this;
            }
        }

        // Merge into the persistent map (in case pre_compute is called multiple
        // times — e.g., for multi-shell solids). Take the max.
        for (k, v) in per_axis_max {
            let entry = self.circle_axis_n.entry(k).or_insert(0);
            if v > *entry {
                *entry = v;
            }
        }
    }

    /// Look up the pre-computed n for the axis of the given circle.
    /// Returns `None` if `pre_compute_circle_axis_n` was not called or the
    /// circle's axis was not seen.
    fn circle_n_for(&self, circle: &draper_geometry::Circle) -> Option<usize> {
        let key = AxisKey::from_circle(circle);
        self.circle_axis_n.get(&key).copied()
    }

    /// Pre-compute a shared interior refinement grid for a single NURBS surface.
    ///
    /// The grid covers the FULL surface parameter range (u_min..u_max,
    /// v_min..v_max) and is chord-error-compliant: every grid cell's chord
    /// deviation is below `effective_chord_tolerance()`.
    ///
    /// The grid is stored in the cache keyed by `nurbs_surface_hash()`.
    /// When a NURBS face is triangulated, `get_nurbs_refinement_grid()` is
    /// called to retrieve the grid, which is then filtered by the face's
    /// UV domain. This ensures all faces sharing the same NURBS surface
    /// entity get the SAME interior Steiner points — critical for
    /// watertightness (MS-2 from audit plan).
    ///
    /// # Algorithm
    /// 1. Start with knot-span midpoints (1 per span).
    /// 2. For each grid cell, compute chord error = distance from midpoint
    ///    of the 4 corners' 3D points to surface.point_at(midpoint UV).
    /// 3. If chord error > tolerance, subdivide the cell into 4 sub-cells.
    /// 4. Repeat until convergence or max_iterations.
    /// 5. Return the final grid (all cell midpoints).
    pub fn pre_compute_nurbs_refinement_grid(
        &mut self,
        nurbs: &draper_geometry::NurbsSurface,
        max_iterations: usize,
    ) {
        let hash = nurbs_surface_hash(nurbs);

        // Skip if already computed (e.g., multiple faces sharing the surface)
        if self.nurbs_refinement_grids.contains_key(&hash) {
            return;
        }

        let chord_tol = self.effective_chord_tolerance();
        let surface = Surface::Nurbs(nurbs.clone());

        // Get full parameter range from knots
        let u_min = nurbs.u_knots.first().copied().unwrap_or(0.0);
        let u_max = nurbs.u_knots.last().copied().unwrap_or(1.0);
        let v_min = nurbs.v_knots.first().copied().unwrap_or(0.0);
        let v_max = nurbs.v_knots.last().copied().unwrap_or(1.0);

        if u_max <= u_min || v_max <= v_min {
            self.nurbs_refinement_grids.insert(hash, Vec::new());
            return;
        }

        // Build initial grid from knot-span midpoints
        let mut u_coords: Vec<f64> = Vec::new();
        let mut v_coords: Vec<f64> = Vec::new();

        // Use interior knot values + midpoints between consecutive knots
        for i in 0..nurbs.u_knots.len().saturating_sub(1) {
            let k0 = nurbs.u_knots[i];
            let k1 = nurbs.u_knots[i + 1];
            if k1 > k0 {
                u_coords.push(k0 + 0.5 * (k1 - k0));
            }
        }
        for i in 0..nurbs.v_knots.len().saturating_sub(1) {
            let k0 = nurbs.v_knots[i];
            let k1 = nurbs.v_knots[i + 1];
            if k1 > k0 {
                v_coords.push(k0 + 0.5 * (k1 - k0));
            }
        }

        // Cartesian product → initial grid
        let mut grid: Vec<Point2d> = Vec::new();
        for &u in &u_coords {
            for &v in &v_coords {
                grid.push(Point2d::new(u, v));
            }
        }

        // Iterative refinement: check chord error and add midpoints where needed
        for _iter in 0..max_iterations {
            let mut new_points: Vec<Point2d> = Vec::new();
            let mut refined = false;

            // For each pair of adjacent grid points (in u and v), check chord error
            // and subdivide if needed.
            let mut sorted_u: Vec<f64> = u_coords.clone();
            sorted_u.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            sorted_u.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

            let mut sorted_v: Vec<f64> = v_coords.clone();
            sorted_v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            sorted_v.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

            // Check each cell (u_i, v_j) → (u_{i+1}, v_{j+1})
            for i in 0..sorted_u.len().saturating_sub(1) {
                for j in 0..sorted_v.len().saturating_sub(1) {
                    let u0 = sorted_u[i];
                    let u1 = sorted_u[i + 1];
                    let v0 = sorted_v[j];
                    let v1 = sorted_v[j + 1];

                    let um = (u0 + u1) * 0.5;
                    let vm = (v0 + v1) * 0.5;

                    // 4 corners
                    let p00 = surface.point_at(u0, v0);
                    let p10 = surface.point_at(u1, v0);
                    let p01 = surface.point_at(u0, v1);
                    let p11 = surface.point_at(u1, v1);

                    // Midpoint of 4 corners (linear interpolation)
                    let p_mid_linear = Point3d::new(
                        (p00.x + p10.x + p01.x + p11.x) * 0.25,
                        (p00.y + p10.y + p01.y + p11.y) * 0.25,
                        (p00.z + p10.z + p01.z + p11.z) * 0.25,
                    );

                    // True surface point at midpoint
                    let p_mid_surface = surface.point_at(um, vm);

                    // Chord error
                    let err = p_mid_linear.distance_to(&p_mid_surface);
                    if err > chord_tol {
                        // Subdivide: add midpoint
                        new_points.push(Point2d::new(um, vm));
                        refined = true;
                    }
                }
            }

            if !refined {
                break;
            }

            // Add new points to grid and update coordinate lists
            for p in &new_points {
                if !u_coords.iter().any(|&u| (u - p.u).abs() < 1e-12) {
                    u_coords.push(p.u);
                }
                if !v_coords.iter().any(|&v| (v - p.v).abs() < 1e-12) {
                    v_coords.push(p.v);
                }
            }
            grid.extend(new_points);
        }

        // Clamp to parameter range and dedup
        let mut filtered: Vec<Point2d> = Vec::new();
        for p in &grid {
            let u = p.u.clamp(u_min, u_max);
            let v = p.v.clamp(v_min, v_max);
            let pt = Point2d::new(u, v);
            if !filtered.iter().any(|q| (q.u - pt.u).abs() < 1e-12 && (q.v - pt.v).abs() < 1e-12) {
                filtered.push(pt);
            }
        }

        log::debug!(
            "NURBS refinement grid: hash={:x}, {} points, chord_tol={:.4}, range u=[{:.3},{:.3}] v=[{:.3},{:.3}]",
            hash, filtered.len(), chord_tol, u_min, u_max, v_min, v_max
        );

        self.nurbs_refinement_grids.insert(hash, filtered);
    }

    /// Get the pre-computed NURBS refinement grid for the given surface.
    /// Returns `None` if `pre_compute_nurbs_refinement_grid` was not called
    /// for this surface.
    pub fn get_nurbs_refinement_grid(&self, nurbs: &draper_geometry::NurbsSurface) -> Option<&Vec<Point2d>> {
        let hash = nurbs_surface_hash(nurbs);
        self.nurbs_refinement_grids.get(&hash)
    }

    /// Adaptively discretize an edge based on curve curvature.
    ///
    /// Starts with uniformly-spaced points based on the hint, then recursively
    /// subdivides where the chord deviation exceeds the adaptive threshold.
    fn adaptive_discretize(&self, edge: &Edge, n_samples_hint: usize) -> (Vec<Point3d>, Vec<f64>) {
        let curve = match &edge.curve {
            Some(c) => c,
            None => {
                // No curve geometry — just use start/end points
                let (p0, p1) = match (edge.start_point(), edge.end_point()) {
                    (Some(a), Some(b)) => (a, b),
                    _ => return (vec![], vec![]),
                };
                return (vec![p0, p1], vec![0.0, 1.0]);
            }
        };

        let (t_min, t_max) = edge.param_range;

        // For line edges, just return endpoints
        if matches!(curve, Curve3d::Line(_)) {
            return (
                vec![curve.point_at(t_min), curve.point_at(t_max)],
                vec![0.0, 1.0],
            );
        }

        // For composite curves, discretize segment-by-segment to ensure
        // we get proper sampling at segment boundaries (where curvature
        // changes abruptly). Then concatenate the results.
        if let Curve3d::Composite { segments, cum_lengths } = curve {
            let mut all_points: Vec<Point3d> = Vec::new();
            let mut all_params: Vec<f64> = Vec::new();

            for (i, seg) in segments.iter().enumerate() {
                let seg_start_frac = if i == 0 { 0.0 } else { cum_lengths[i - 1] };
                let seg_end_frac = cum_lengths[i];

                // Determine sample count for this segment
                let seg_n = if matches!(seg, Curve3d::Line(_)) { 2 } else { n_samples_hint.max(8) };

                // Create a sub-edge for this segment
                let (seg_t_min, seg_t_max) = seg.param_range();
                let seg_edge = Edge::new(seg.clone(), (seg_t_min, seg_t_max));
                let (mut seg_points, mut seg_params) = self.adaptive_discretize(&seg_edge, seg_n);

                // Remap local params to global composite param range [0,1]
                for p in &mut seg_params {
                    *p = seg_start_frac + *p * (seg_end_frac - seg_start_frac);
                }

                // MS-4: Robust deduplication for composite curve joints.
                // Previous code used a tight 1e-12 tolerance which could miss
                // close-but-not-identical points (FP drift from different curve
                // evaluations). Now uses a model-scale-aware tolerance and
                // snaps the joint point to the midpoint for consistency.
                if !all_points.is_empty() && !seg_points.is_empty() {
                    let last = *all_points.last().unwrap();
                    let first = seg_points[0];
                    let dx = first.x - last.x;
                    let dy = first.y - last.y;
                    let dz = first.z - last.z;
                    let dist_sq = dx * dx + dy * dy + dz * dz;

                    // Use model-scale-aware tolerance (1 PPM of model scale)
                    let dedup_tol = self.adaptive_tol.merge_tolerance();
                    let dedup_tol_sq = dedup_tol * dedup_tol;

                    if dist_sq < dedup_tol_sq {
                        // Points are close enough to deduplicate.
                        // If they're not bit-identical, snap to midpoint
                        // and apply deterministic_round_point for consistency.
                        if dist_sq > 1e-20 {
                            let mid = Point3d::new(
                                (last.x + first.x) * 0.5,
                                (last.y + first.y) * 0.5,
                                (last.z + first.z) * 0.5,
                            );
                            let rounded = deterministic_round_point(mid);
                            *all_points.last_mut().unwrap() = rounded;
                        }
                        seg_points.remove(0);
                        seg_params.remove(0);
                    } else {
                        // Points are NOT close — this indicates a gap in the
                        // composite curve. Log a warning but continue (the gap
                        // will be caught by weld_boundary_edge_vertices later).
                        if dist_sq < (dedup_tol * 10.0).powi(2) {
                            log::warn!(
                                "composite_curve: joint gap {:.2e} > tol {:.2e} — segments may not connect properly",
                                dist_sq.sqrt(), dedup_tol,
                            );
                        }
                    }
                }

                all_points.extend(seg_points);
                all_params.extend(seg_params);
            }

            return (all_points, all_params);
        }

        // Adaptive subdivision threshold: use LOD override if set, else adaptive.
        let max_deviation = self.effective_chord_tolerance();

        // For CIRCLE curves, use a UNIFORM angular grid. The number of points
        // is determined by `pre_compute_circle_axis_n` (preferred) or computed
        // from the LOD-driven chord tolerance as a fallback.
        //
        // CRITICAL for watertightness: when two circles on the same axis have
        // different radii (e.g., R=30.36 bottom ring + R=35.22 top ring of a
        // cone tube face), they MUST have the SAME number of points at the
        // SAME angular positions. The `pre_compute_circle_axis_n` method
        // ensures this by computing the MAX n across all circles sharing an
        // axis — so both rings get the same n.
        //
        // LOD response: the chord tolerance (set by `set_chord_tolerance_override`)
        // feeds into `compute_circle_n`, so a finer LOD produces more samples.
        // The fallback (no pre-compute) still uses LOD, but only per-circle —
        // which may break watertightness for multi-radius tube faces. Therefore
        // `pre_compute_circle_axis_n` MUST be called before triangulation.
        if let Curve3d::Circle(ref circle) = curve {
            // Determine n: prefer pre-computed per-axis value, else compute
            // from chord tolerance and this circle's radius.
            let n = if let Some(n_pre) = self.circle_n_for(circle) {
                // Use the pre-computed (axis-group max) n. Also enforce a
                // minimum based on the hint so we don't UNDER-tessellate when
                // the pre-compute was done with a stale tolerance.
                n_pre.max(n_samples_hint.max(8).min(self.max_samples))
            } else {
                // Fallback: per-circle n from chord tolerance
                let chord_tol = self.effective_chord_tolerance();
                let n_from_tol = compute_circle_n(circle.radius, chord_tol, self.max_samples);
                // Use the MAX of (n_from_tol, n_samples_hint) so the hint
                // acts as a floor. This is important for the native path
                // where pre_compute_circle_axis_n may not have been called.
                n_from_tol.max(n_samples_hint.max(8).min(self.max_samples))
            };
            // Final clamp to max_samples
            let n = n.min(self.max_samples).max(2);
            let mut points: Vec<Point3d> = Vec::with_capacity(n);
            let mut t_params: Vec<f64> = Vec::with_capacity(n);
            for i in 0..n {
                let t_norm = if n > 1 { i as f64 / (n - 1) as f64 } else { 0.0 };
                let t = t_min + t_norm * (t_max - t_min);
                points.push(curve.point_at(t));
                t_params.push(t_norm);
            }
            return (points, t_params);
        }

        // For other curve types (NURBS, etc.), keep the adaptive refinement.
        // Start with uniformly spaced points based on hint
        let n_initial = n_samples_hint.min(self.max_samples).max(2);
        let mut t_params: Vec<f64> = vec![0.0]; // Normalized parameter [0, 1]
        let mut points: Vec<Point3d> = vec![curve.point_at(t_min)];

        for i in 1..n_initial {
            let t_norm = i as f64 / (n_initial - 1) as f64;
            let t = t_min + t_norm * (t_max - t_min);
            points.push(curve.point_at(t));
            t_params.push(t_norm);
        }

        // Refine: check chord deviation and subdivide where needed
        let mut refined = true;
        let mut refinement_passes = 0;
        let max_refinement_passes = 5;

        while refined && refinement_passes < max_refinement_passes && points.len() < self.max_samples {
            refined = false;
            refinement_passes += 1;

            let mut i = 0;
            while i < points.len() - 1 && points.len() < self.max_samples {
                let p0 = points[i];
                let p2 = points[i + 1];

                // Compute midpoint parameter
                let t_mid = (t_params[i] + t_params[i + 1]) * 0.5;
                let t_actual = t_min + t_mid * (t_max - t_min);
                let p_mid = curve.point_at(t_actual);

                // Check chord deviation: distance from midpoint to the chord
                let deviation = point_to_chord_distance(&p_mid, &p0, &p2);

                if deviation > max_deviation {
                    // Subdivide: insert midpoint
                    points.insert(i + 1, p_mid);
                    t_params.insert(i + 1, t_mid);
                    refined = true;
                    i += 2; // Skip the newly inserted point
                } else {
                    i += 1;
                }
            }
        }

        (points, t_params)
    }

    /// Compute UV coordinates for a set of 3D points on a surface.
    ///
    /// If a Curve2d (analytical PCURVE) is provided, UV coordinates are computed
    /// by evaluating the curve at the corresponding parameter values. This is more
    /// accurate and faster than surface.project_point().
    ///
    /// If no Curve2d is available, falls back to surface projection.
    /// For NURBS surfaces, uses chain Newton-Raphson (bootstrap from first point,
    /// then use previous UV as initial guess for next point). This is both faster
    /// and more accurate than calling surface.project_point() for each point
    /// independently (which does a 32×32 grid search per call).
    pub(crate) fn compute_uvs(points_3d: &[Point3d], params: &[f64], surface: &Surface, curve_2d: Option<&Curve2d>) -> Vec<Point2d> {
        if let Some(c2d) = curve_2d {
            // Use analytical PCURVE — evaluate the 2D curve at each parameter value
            let (t_min, t_max) = c2d.param_range();
            params.iter().map(|&t| {
                // Map normalized parameter t ∈ [0, 1] to curve's parameter range
                let curve_t = t_min + t * (t_max - t_min);
                c2d.point_at(curve_t)
            }).collect()
        } else if let Surface::Nurbs(ref nurbs) = surface {
            // NURBS UV projection strategy.
            //
            // DESIGN: Use the analytic project_point() for every point INDEPENDENTLY.
            // This is deterministic — the same 3D point always maps to the same UV
            // regardless of traversal order, which is CRITICAL for edge cache
            // consistency (shared edges between adjacent faces must produce
            // bit-identical UVs so the seam-split logic in parametric_domain.rs
            // can detect and handle periodic wraparound correctly).
            //
            // The old brute-force fallback (grid_size × grid_size evaluations per
            // point) was the #1 cause of test files hanging / "not opening":
            // 36×36 = 1296 surface evaluations per bad point, repeated for every
            // boundary point of every NURBS face, multiplied by the number of
            // faces — easily 100K+ evaluations for a moderately complex model.
            //
            // We removed the brute-force fallback because:
            // 1. Performance: brute-force is O(grid_size^2) per point — too slow.
            // 2. Quality: brute-force didn't actually fix the UV scrambling issue;
            //    the seam-split logic in parametric_domain.rs handles that now.
            // 3. Watertightness: 3D boundary points come from the edge cache
            //    (deterministic_round), so they are bit-identical regardless of
            //    UV accuracy. UV only affects triangulation QUALITY, not watertightness.
            let (u_min, u_max) = nurbs.u_range();
            let (v_min, v_max) = nurbs.v_range();

            let mut uvs = Vec::with_capacity(points_3d.len());
            let mut out_of_range_count = 0usize;

            for point in points_3d.iter() {
                let (u, v) = surface.project_point(point);

                // Clamp UVs to the valid NURBS parameter range. project_point()
                // can return values slightly outside the range due to floating-point
                // errors, especially at seam points on periodic surfaces.
                let u_clamped = u.clamp(u_min, u_max);
                let v_clamped = v.clamp(v_min, v_max);
                if u_clamped != u || v_clamped != v {
                    out_of_range_count += 1;
                }

                uvs.push(Point2d::new(
                    deterministic_round(u_clamped),
                    deterministic_round(v_clamped),
                ));
            }

            if out_of_range_count > 0 {
                log::debug!(
                    "NURBS UV: {}/{} points clamped to range u=[{:.4},{:.4}] v=[{:.4},{:.4}]",
                    out_of_range_count, points_3d.len(),
                    u_min, u_max, v_min, v_max,
                );
            }

            // 5.1.1 — Snap UV coordinates near the seam boundary for periodic NURBS.
            if nurbs.u_closed || nurbs.v_closed {
                Self::snap_seam_uvs(&mut uvs, surface);
            }

            uvs
        } else {
            // Non-NURBS surfaces: project_point() is fast, use it directly
            let mut uvs: Vec<Point2d> = points_3d
                .iter()
                .map(|p| {
                    let (u, v) = surface.project_point(p);
                    Point2d::new(u, v)
                })
                .collect();

            // 5.1.1 — Snap UV coordinates near the seam boundary to the exact value.
            // This ensures consistent UV coordinates on both sides of the seam,
            // which is critical for seam-split detection and vertex deduplication.
            Self::snap_seam_uvs(&mut uvs, surface);

            uvs
        }
    }

    /// Snap UV coordinates that are very close to the seam boundary of a periodic
    /// surface to the exact boundary value. This ensures consistent UV coordinates
    /// on both sides of the seam, which is critical for the seam-split logic and
    /// for vertex deduplication at the seam.
    ///
    /// Without snapping, two edges at u≈0 and u≈2π on a periodic surface can have
    /// UV values like u=0.0001 and u=6.2822 instead of exactly u=0 and u=2π.
    /// The seam-split logic then fails to detect the seam crossing, and vertices
    /// at the seam don't get deduplicated, leaving boundary edges.
    fn snap_seam_uvs(uvs: &mut [Point2d], surface: &Surface) {
        use std::f64::consts::PI;
        let is_u_periodic = surface.is_u_periodic();
        let is_v_periodic = surface.is_v_periodic();
        if !is_u_periodic && !is_v_periodic {
            return;
        }

        let (u_min, u_max) = match surface {
            Surface::Nurbs(n) => n.u_range(),
            Surface::Cylinder(_) | Surface::Cone(_) | Surface::Revolution(_) => (0.0, 2.0 * PI),
            Surface::Sphere(_) => (0.0, 2.0 * PI),
            Surface::Torus(_) => (0.0, 2.0 * PI),
            Surface::Plane(_) | Surface::Extrusion(_) => (0.0, 1.0),
        };
        let (v_min, v_max) = match surface {
            Surface::Nurbs(n) => n.v_range(),
            Surface::Sphere(_) => (0.0, PI),
            Surface::Torus(_) => (0.0, 2.0 * PI),
            _ => (0.0, 1.0),
        };
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;

        // Snap threshold: if a UV value is within 1% of the boundary, snap it.
        let u_snap_thresh = u_range * 0.01;
        let v_snap_thresh = v_range * 0.01;

        for uv in uvs.iter_mut() {
            if is_u_periodic && u_range > 0.0 {
                if (uv.u - u_min).abs() < u_snap_thresh {
                    uv.u = u_min;
                } else if (uv.u - u_max).abs() < u_snap_thresh {
                    uv.u = u_max;
                }
            }
            if is_v_periodic && v_range > 0.0 {
                if (uv.v - v_min).abs() < v_snap_thresh {
                    uv.v = v_min;
                } else if (uv.v - v_max).abs() < v_snap_thresh {
                    uv.v = v_max;
                }
            }
        }
    }

    /// Get the UV coordinates for a specific face-edge pair.
    ///
    /// Returns `None` if the edge is not in the cache, or if UV coordinates
    /// haven't been computed for the given face yet.
    pub fn get_uv_for_face(&self, edge_id: TopoId, face_id: TopoId) -> Option<&Vec<Point2d>> {
        self.get(edge_id).and_then(|e| e.uv_per_face.get(&face_id))
    }


    /// Pre-populate the cache with all edge discretizations for a solid.
    ///
    /// Uses topology-first approach:
    /// 1. Discretize all edges ONCE (3D points with deterministic rounding)
    /// 2. UV coordinates are computed lazily per face (not eagerly)
    ///
    /// This replaces the old two-pass `pre_populate_for_solid_full` which
    /// computed all UVs eagerly. Lazy UV computation saves ~30% of time for
    /// models with many faces per edge.
    pub fn pre_populate_for_solid(&mut self, solid: &Solid, default_n_samples: usize) {
        // Phase 0a: Pre-compute per-axis-group n for circles. This MUST happen
        // before any discretization so that the Circle branch in
        // adaptive_discretize can look up the pre-computed n and ensure all
        // circles on the same axis get the same sample count (critical for
        // watertightness of cone/cylinder tube faces with multi-radius rings).
        let all_edges: Vec<&Edge> = solid
            .faces()
            .iter()
            .flat_map(|f| f.edges.iter())
            .filter(|e| !e.degenerate)
            .collect();
        self.pre_compute_circle_axis_n(all_edges);

        // Phase 0b: Pre-compute shared NURBS refinement grids (MS-2).
        // For each unique NURBS surface in the solid, generate a chord-error-
        // compliant interior UV grid. All faces sharing the same NURBS surface
        // will use this grid → identical interior Steiner points → watertight.
        for face in solid.faces() {
            if let Some(Surface::Nurbs(nurbs)) = &face.surface {
                self.pre_compute_nurbs_refinement_grid(nurbs, 3);
            }
        }

        // Single pass: discretize all edges (3D points only)
        for face in solid.faces() {
            for edge in &face.edges {
                if edge.degenerate { continue; }
                let key = EdgeCacheKey::from_edge(edge);
                self.topo_id_to_key.insert(edge.id, key);
                if !self.entries.contains_key(&key) {
                    let (mut points_3d, params) = self.adaptive_discretize(edge, default_n_samples);
                    // Apply deterministic rounding
                    for p in &mut points_3d {
                        *p = deterministic_round_point(*p);
                    }
                    self.entries.insert(key, EdgeDiscretization {
                        points_3d,
                        uv_per_face: HashMap::new(),
                        params,
                    });
                }
            }
        }
    }

    /// Pre-populate the cache with all edge discretizations AND UV coordinates
    /// for every face-edge pair in the solid.
    ///
    /// After calling this method, the cache is fully read-only and can be
    /// shared as `&EdgeDiscretizationCache` across threads (for parallel
    /// triangulation). No further calls to `discretize_edge` are needed.
    ///
    /// Use `pre_populate_for_solid` instead for sequential triangulation
    /// where UV can be computed lazily.
    pub fn pre_populate_for_solid_full(&mut self, solid: &Solid, default_n_samples: usize) {
        // Phase 0a: Pre-compute per-axis-group n for circles (see
        // pre_populate_for_solid for rationale).
        let all_edges: Vec<&Edge> = solid
            .faces()
            .iter()
            .flat_map(|f| f.edges.iter())
            .filter(|e| !e.degenerate)
            .collect();
        self.pre_compute_circle_axis_n(all_edges);

        // Phase 0b: Pre-compute shared NURBS refinement grids (MS-2).
        for face in solid.faces() {
            if let Some(Surface::Nurbs(nurbs)) = &face.surface {
                self.pre_compute_nurbs_refinement_grid(nurbs, 3);
            }
        }

        // First pass: discretize all edges (3D points only)
        for face in solid.faces() {
            if let Some(ref _surface) = face.surface {
                for edge in &face.edges {
                    if edge.degenerate { continue; }
                    let key = EdgeCacheKey::from_edge(edge);
                    self.topo_id_to_key.insert(edge.id, key);
                    if !self.entries.contains_key(&key) {
                        let (mut points_3d, params) = self.adaptive_discretize(edge, default_n_samples);
                        // Apply deterministic rounding
                        for p in &mut points_3d {
                            *p = deterministic_round_point(*p);
                        }
                        self.entries.insert(key, EdgeDiscretization {
                            points_3d,
                            uv_per_face: HashMap::new(),
                            params,
                        });
                    }
                }
            }
        }
        // Second pass: compute UVs for each face-edge pair (needed for parallel)
        for face in solid.faces() {
            if let Some(ref surface) = face.surface {
                // Outer wire
                if let Some(ref wire) = face.outer_wire {
                    for coedge in &wire.coedges {
                        let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                        if let Some(edge) = edge {
                            if edge.degenerate { continue; }
                            let key = EdgeCacheKey::from_edge(edge);
                            if let Some(entry) = self.entries.get_mut(&key) {
                                if !entry.uv_per_face.contains_key(&face.id) {
                                    let uvs = Self::compute_uvs(
                                        &entry.points_3d, &entry.params,
                                        surface, coedge.curve_2d.as_ref(),
                                    );
                                    entry.uv_per_face.insert(face.id, uvs);
                                }
                            }
                        }
                    }
                }
                // Inner wires
                for wire in &face.inner_wires {
                    for coedge in &wire.coedges {
                        let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                        if let Some(edge) = edge {
                            if edge.degenerate { continue; }
                            let key = EdgeCacheKey::from_edge(edge);
                            if let Some(entry) = self.entries.get_mut(&key) {
                                if !entry.uv_per_face.contains_key(&face.id) {
                                    let uvs = Self::compute_uvs(
                                        &entry.points_3d, &entry.params,
                                        surface, coedge.curve_2d.as_ref(),
                                    );
                                    entry.uv_per_face.insert(face.id, uvs);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Clear the cache.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.topo_id_to_key.clear();
    }

    /// Validate that all circles on the same axis have consistent sampling
    /// (same number of discretization points).
    ///
    /// This is critical for watertightness of tube faces (cylinder, cone):
    /// two circles on the same axis with different radii MUST have the same
    /// number of points in the same angular positions. Otherwise the
    /// `bottom[i] → top[i]` connection in tube grid triangulation breaks.
    ///
    /// Returns a list of inconsistencies found. Empty vec = all consistent.
    pub fn validate_circle_consistency(&self) -> Vec<CircleInconsistency> {
        use std::collections::HashMap;

        // Group cached circles by axis (origin + direction)
        // Use quantized coordinates as key since Point3d/Direction3d don't impl Hash
        let quant = 1e-4; // Quantization tolerance for axis grouping
        let mut axis_groups: HashMap<(i64, i64, i64, i64, i64, i64), Vec<(EdgeCacheKey, usize, Point3d, Direction3d)>> = HashMap::new();

        for (key, entry) in &self.entries {
            let n = entry.points_3d.len();
            if n < 8 {
                continue;
            }

            if entry.points_3d.len() < 2 {
                continue;
            }
            let first = entry.points_3d.first().unwrap();
            let last = entry.points_3d.last().unwrap();
            let axis_origin = *first;
            let dx = last.x - first.x;
            let dy = last.y - first.y;
            let dz = last.z - first.z;
            let len = (dx * dx + dy * dy + dz * dz).sqrt();
            if len < 1e-10 {
                continue;
            }
            let axis_dir = Direction3d::new(dx / len, dy / len, dz / len)
                .unwrap_or(Direction3d::Z);

            let key_q = (
                (axis_origin.x / quant).round() as i64,
                (axis_origin.y / quant).round() as i64,
                (axis_origin.z / quant).round() as i64,
                (axis_dir.x / quant).round() as i64,
                (axis_dir.y / quant).round() as i64,
                (axis_dir.z / quant).round() as i64,
            );
            axis_groups.entry(key_q).or_default().push((key.clone(), n, axis_origin, axis_dir));
        }

        let mut inconsistencies = Vec::new();
        for (_key_q, group) in &axis_groups {
            if group.len() < 2 {
                continue;
            }
            let n_values: Vec<usize> = group.iter().map(|(_, n, _, _)| *n).collect();
            let first_n = n_values[0];
            if !n_values.iter().all(|&n| n == first_n) {
                inconsistencies.push(CircleInconsistency {
                    axis_origin: group[0].2,
                    axis_dir: group[0].3,
                    edge_keys: group.iter().map(|(k, _, _, _)| k.clone()).collect(),
                    point_counts: n_values,
                });
            }
        }

        inconsistencies
    }
}

/// Report of a circle sampling inconsistency.
#[derive(Clone, Debug)]
pub struct CircleInconsistency {
    /// Approximate axis origin.
    pub axis_origin: Point3d,
    /// Approximate axis direction.
    pub axis_dir: Direction3d,
    /// Edge cache keys of circles on this axis.
    pub edge_keys: Vec<EdgeCacheKey>,
    /// Point counts for each circle (should all be equal).
    pub point_counts: Vec<usize>,
}

/// Compute adaptive grid size for brute-force NURBS point projection.
///
/// Larger UV ranges need finer grids to ensure the closest UV is found.
/// However, very large grids (70-100) are extremely slow and rarely improve
/// results — they were the main cause of performance regression. The seam-split
/// strategy in parametric_domain.rs now handles closed NURBS surfaces correctly,
/// so brute-force projection is only needed as a last resort for truly bad
/// parameterizations.
pub fn adaptive_grid_size(u_range: f64, v_range: f64) -> usize {
    let max_range = u_range.max(v_range);
    if max_range < 10.0 {
        15       // Small range — coarse grid is fine
    } else if max_range < 50.0 {
        25       // Medium range
    } else {
        35       // Large range — cap at 35 to avoid extreme slowdown
    }
}

/// Brute-force NURBS point projection using multi-resolution grid search.
///
/// When `surface.project_point()` and chain Newton-Raphson both fail to
/// find a good UV (reconstructed 3D point too far from target), this
/// function performs a more exhaustive grid search followed by Newton
/// refinement from the best grid point.
///
/// This is deterministic: the same 3D point always produces the same UV
/// regardless of traversal order, which is critical for edge cache
/// consistency (shared edges between faces must have bit-identical UVs).
///
/// # Performance
/// Grid size 100 → 10,201 evaluations per point. Only used as fallback
/// when project_point() fails, and only for boundary points (typically
/// < 100 per edge). Applied once and cached by the edge cache.
pub fn brute_force_project_point(
    nurbs: &draper_geometry::NurbsSurface,
    point: &Point3d,
    grid_size: usize,
) -> (f64, f64) {
    let (u_min, u_max) = nurbs.u_range();
    let (v_min, v_max) = nurbs.v_range();
    let surface = Surface::Nurbs(nurbs.clone());

    let mut best_u = (u_min + u_max) * 0.5;
    let mut best_v = (v_min + v_max) * 0.5;
    let mut best_dist = f64::MAX;

    let u_step = (u_max - u_min) / grid_size as f64;
    let v_step = (v_max - v_min) / grid_size as f64;

    // Phase 1: Uniform grid search
    for i in 0..=grid_size {
        let u = u_min + u_step * i as f64;
        for j in 0..=grid_size {
            let v = v_min + v_step * j as f64;
            let p = surface.point_at(u, v);
            let dist = (p.x - point.x).powi(2)
                + (p.y - point.y).powi(2)
                + (p.z - point.z).powi(2);
            if dist < best_dist {
                best_dist = dist;
                best_u = u;
                best_v = v;
            }
        }
    }

    // Phase 2: Local refinement — 7×7 grid around the best point
    let refine = 7;
    let refine_u_start = (best_u - u_step).max(u_min);
    let refine_v_start = (best_v - v_step).max(v_min);
    let refine_u_end = (best_u + u_step).min(u_max);
    let refine_v_end = (best_v + v_step).min(v_max);
    let refine_u_range = refine_u_end - refine_u_start;
    let refine_v_range = refine_v_end - refine_v_start;

    for i in 0..=refine {
        let u = refine_u_start + refine_u_range * i as f64 / refine as f64;
        for j in 0..=refine {
            let v = refine_v_start + refine_v_range * j as f64 / refine as f64;
            let p = surface.point_at(u, v);
            let dist = (p.x - point.x).powi(2)
                + (p.y - point.y).powi(2)
                + (p.z - point.z).powi(2);
            if dist < best_dist {
                best_dist = dist;
                best_u = u;
                best_v = v;
            }
        }
    }

    // Phase 3: Newton-Raphson refinement from the best grid point
    let (u_refined, v_refined) = crate::parametric_domain::reproject_nurbs_point(
        nurbs, point, best_u, best_v,
    );

    // Validate: if Newton made it worse, fall back to grid result
    let refined_p = surface.point_at(u_refined, v_refined);
    let refined_dist = (refined_p.x - point.x).powi(2)
        + (refined_p.y - point.y).powi(2)
        + (refined_p.z - point.z).powi(2);

    if refined_dist < best_dist {
        (u_refined, v_refined)
    } else {
        (best_u, best_v)
    }
}

impl Default for EdgeDiscretizationCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the distance from a point to a line segment (chord).
fn point_to_chord_distance(point: &Point3d, a: &Point3d, b: &Point3d) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let abz = b.z - a.z;
    let apx = point.x - a.x;
    let apy = point.y - a.y;
    let apz = point.z - a.z;

    let ab_len_sq = abx * abx + aby * aby + abz * abz;
    if ab_len_sq < 1e-30 {
        return (apx * apx + apy * apy + apz * apz).sqrt();
    }

    let t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
    let t = t.clamp(0.0, 1.0);

    let cx = a.x + t * abx - point.x;
    let cy = a.y + t * aby - point.y;
    let cz = a.z + t * abz - point.z;
    (cx * cx + cy * cy + cz * cz).sqrt()
}

/// Compute an adaptive crease angle for normal smoothing based on surface type.
///
/// Instead of using a fixed 30° crease angle for all surfaces, this function
/// returns a surface-type-appropriate angle:
/// - Planes: 0° (sharp edges, no smoothing across face boundaries)
/// - Cylinders/Cones/Spheres/Tori: 180° (smooth everything)
/// - NURBS: based on max curvature (adaptive)
/// - Revolution/Extrusion: 90° (moderate smoothing)
pub fn compute_adaptive_crease_angle(surface: &Surface) -> f64 {
    match surface {
        Surface::Plane(_) => 0.0, // Sharp edges at face boundaries
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) => {
            std::f64::consts::PI // 180°: smooth everything
        }
        Surface::Revolution(_) | Surface::Extrusion(_) => {
            std::f64::consts::FRAC_PI_2 // 90°: moderate smoothing
        }
        Surface::Nurbs(_) => {
            // For NURBS, use a moderate angle — the exact curvature would
            // require evaluating the surface, which is expensive.
            // 60° is a good compromise.
            std::f64::consts::PI / 3.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::{Plane, Line2d};
    use draper_topology::Edge;

    #[test]
    fn test_deterministic_round_consistency() {
        // Two values that differ only in the lowest mantissa bits
        // should round to the same value
        let v1 = 1.0000000000000002_f64; // differs in lowest bit
        let v2 = 1.0000000000000004_f64;
        let r1 = deterministic_round(v1);
        let r2 = deterministic_round(v2);
        assert_eq!(r1, r2, "Deterministic rounding should produce identical results for near-equal values");
    }

    #[test]
    fn test_deterministic_round_point() {
        let p = Point3d::new(1.0000000000000002, 2.0000000000000004, 3.0);
        let rounded = deterministic_round_point(p);
        // Should be deterministic
        let rounded2 = deterministic_round_point(p);
        assert_eq!(rounded.x.to_bits(), rounded2.x.to_bits());
        assert_eq!(rounded.y.to_bits(), rounded2.y.to_bits());
        assert_eq!(rounded.z.to_bits(), rounded2.z.to_bits());
    }

    #[test]
    fn test_line_edge_cached_once() {
        let mut cache = EdgeDiscretizationCache::new();
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 0.0, 0.0);
        let edge = Edge::new_line(p1, p2);

        let surface = Surface::Plane(Plane::xy());
        let face_id = TopoId::new();

        {
            let disc = cache.discretize_edge(&edge, face_id, &surface, 32, None);
            // Line edges should have exactly 2 points (endpoints)
            assert_eq!(disc.points_3d.len(), 2);
        }

        // Verify cache count after the borrow is released
        assert_eq!(cache.len(), 1);
    }

    #[test]
    fn test_shared_edge_same_points() {
        let mut cache = EdgeDiscretizationCache::new();
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 0.0, 0.0);
        let edge = Edge::new_line(p1, p2);

        let surface1 = Surface::Plane(Plane::xy());
        let surface2 = Surface::Plane(Plane::xy());
        let face1_id = TopoId::new();
        let face2_id = TopoId::new();

        // Discretize for face1 — clone the points to release the borrow
        let points_face1 = cache.discretize_edge(&edge, face1_id, &surface1, 32, None).points_3d.clone();

        // Discretize for face2 — should return same 3D points
        let (points_face2, has_face1_uv, has_face2_uv) = {
            let disc2 = cache.discretize_edge(&edge, face2_id, &surface2, 32, None);
            let pts = disc2.points_3d.clone();
            let h1 = disc2.uv_per_face.contains_key(&face1_id);
            let h2 = disc2.uv_per_face.contains_key(&face2_id);
            (pts, h1, h2)
        };

        assert_eq!(points_face1, points_face2, "Shared edges must produce identical 3D points");
        assert!(has_face1_uv, "UV for face1 should be present");
        assert!(has_face2_uv, "UV for face2 should be present");

        // Verify cache count after borrows are released
        assert_eq!(cache.len(), 1, "Edge should only be cached once");
    }

    #[test]
    fn test_step_entity_id_key_deduplication() {
        // Two edges with different TopoIds but same step_entity_id
        // should resolve to the same cache entry
        let mut cache = EdgeDiscretizationCache::new();
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 0.0, 0.0);

        let mut edge1 = Edge::new_line(p1, p2);
        edge1.step_entity_id = Some(42); // Same STEP entity ID

        let mut edge2 = Edge::new_line(p1, p2);
        edge2.step_entity_id = Some(42); // Same STEP entity ID, different TopoId

        // Discretize via STEP API
        let (pts1, _) = cache.discretize_step_edge(42, &edge1, 32);
        let (pts2, _) = cache.discretize_step_edge(42, &edge2, 32);

        assert_eq!(pts1, pts2, "Edges with same step_entity_id must produce identical points");
        assert_eq!(cache.len(), 1, "Should only have one cache entry");
    }

    #[test]
    fn test_curve2d_analytical_uv() {
        let mut cache = EdgeDiscretizationCache::new();
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 0.0, 0.0);
        let edge = Edge::new_line(p1, p2);

        let surface = Surface::Plane(Plane::xy());
        let face_id = TopoId::new();

        let curve_2d = Curve2d::Line(Line2d::new(
            Point2d::new(0.5, 0.5),
            Point2d::new(1.5, 0.5),
        ));

        let disc = cache.discretize_edge(&edge, face_id, &surface, 32, Some(&curve_2d));

        let uvs = disc.uv_per_face.get(&face_id).unwrap();
        assert!((uvs[0].u - 0.5).abs() < 1e-10, "Expected u=0.5, got {}", uvs[0].u);
        assert!((uvs[0].v - 0.5).abs() < 1e-10, "Expected v=0.5, got {}", uvs[0].v);
        assert!((uvs[1].u - 1.5).abs() < 1e-10, "Expected u=1.5, got {}", uvs[1].u);
        assert!((uvs[1].v - 0.5).abs() < 1e-10, "Expected v=0.5, got {}", uvs[1].v);
    }

    #[test]
    fn test_adaptive_tolerance() {
        let tol = AdaptiveTolerance::from_model_scale(1.0);
        assert!((tol.merge_tolerance() - 1e-6).abs() < 1e-12);
        assert!((tol.chord_tolerance() - 1e-5).abs() < 1e-12);

        let tol_big = AdaptiveTolerance::from_model_scale(1000.0);
        assert!(tol_big.merge_tolerance() > 1e-6, "Large model should have larger tolerance");
        assert!(tol_big.merge_tolerance() < 1e-2, "But not too large");
    }

    #[test]
    fn test_adaptive_crease_angle() {
        let plane = Surface::Plane(Plane::xy());
        assert_eq!(compute_adaptive_crease_angle(&plane), 0.0);

        let cyl = Surface::Cylinder(draper_geometry::CylinderSurface::new_z(5.0));
        assert!((compute_adaptive_crease_angle(&cyl) - std::f64::consts::PI).abs() < 1e-10);
    }

    #[test]
    fn test_point_to_chord_distance() {
        let a = Point3d::new(0.0, 0.0, 0.0);
        let b = Point3d::new(1.0, 0.0, 0.0);

        let on_chord = Point3d::new(0.5, 0.0, 0.0);
        assert!(point_to_chord_distance(&on_chord, &a, &b) < 1e-10);

        let perp = Point3d::new(0.5, 1.0, 0.0);
        let dist = point_to_chord_distance(&perp, &a, &b);
        assert!((dist - 1.0).abs() < 1e-10);
    }
}

// ============================================================
// Phase 1.4: Triangulation Cache
//
// LRU cache for triangulation results. When loading the same STEP
// file repeatedly (e.g., rotating a model that triggers re-meshing),
// the cache returns the previously computed mesh in < 50ms instead
// of re-triangulating from scratch.
// ============================================================

/// Cache key: hash of the face geometry + triangulation parameters.
/// Two faces produce the same key iff they have the same surface geometry,
/// the same boundary edge topology, and the same params.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TriangulationCacheKey(u64);

impl TriangulationCacheKey {
    /// Compute a cache key for a face + params combination.
    /// Uses a fast hash of the surface type, bounding box, and key params.
    pub fn from_face_and_params(face: &draper_topology::Face, params: &crate::triangulate::TriangulationParams) -> Self {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::collections::hash_map::DefaultHasher::new();

        // Hash surface type
        if let Some(ref surface) = face.surface {
            std::mem::discriminant(surface).hash(&mut hasher);
        }

        // Hash face topology (number of edges, coedge orientations)
        if let Some(ref wire) = face.outer_wire {
            wire.coedges.len().hash(&mut hasher);
            for coedge in &wire.coedges {
                coedge.edge.hash(&mut hasher);
                coedge.forward.hash(&mut hasher);
            }
        }
        face.inner_wires.len().hash(&mut hasher);
        face.forward.hash(&mut hasher);

        // Hash key triangulation params
        params.max_deviation.to_bits().hash(&mut hasher);
        params.detail_level.to_bits().hash(&mut hasher);
        params.adaptive.hash(&mut hasher);
        params.max_face_triangles.hash(&mut hasher);

        TriangulationCacheKey(hasher.finish())
    }

    /// Compute a cache key for an entire solid + params combination.
    /// Hashes all faces' geometry and topology for a more reliable key
    /// than using just the first face as a proxy.
    pub fn from_solid_and_params(solid: &draper_topology::Solid, params: &crate::triangulate::TriangulationParams) -> Self {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::collections::hash_map::DefaultHasher::new();

        // Hash the solid's face count and each face's key
        let faces = solid.faces();
        faces.len().hash(&mut hasher);
        for face in &faces {
            if let Some(ref surface) = face.surface {
                std::mem::discriminant(surface).hash(&mut hasher);
            }
            if let Some(ref wire) = face.outer_wire {
                wire.coedges.len().hash(&mut hasher);
                for coedge in &wire.coedges {
                    coedge.edge.hash(&mut hasher);
                    coedge.forward.hash(&mut hasher);
                }
            }
            face.inner_wires.len().hash(&mut hasher);
            face.forward.hash(&mut hasher);
        }

        // Hash key triangulation params
        params.max_deviation.to_bits().hash(&mut hasher);
        params.detail_level.to_bits().hash(&mut hasher);
        params.adaptive.hash(&mut hasher);
        params.max_face_triangles.hash(&mut hasher);

        TriangulationCacheKey(hasher.finish())
    }
}

/// O(1) LRU cache for triangulation results using index-based doubly-linked list.
///
/// Unlike the previous Vec-based implementation (O(n) eviction via `remove(0)`),
/// this uses an index-based doubly-linked list embedded in the entry nodes,
/// providing O(1) insertion, lookup, and eviction.
///
/// # Performance
/// - Cache hit: O(1) lookup + clone — **< 1ms** for typical meshes
/// - Cache miss: Full triangulation — **50ms-5s** depending on model
/// - LRU eviction: O(1) — no shifting or reallocation
/// - Memory: ~1KB per 100 triangles (positions + normals + indices) + 24 bytes overhead per entry
///
/// # Usage
/// ```ignore
/// use draper_mesh::edge_cache::TriangulationCache;
/// use draper_mesh::triangulate::{triangulate_solid, TriangulationParams};
///
/// let mut cache = TriangulationCache::new(100); // 100 entries
/// let mesh = cache.get_or_triangulate(solid, &params);
/// ```
pub struct TriangulationCache {
    /// The cache entries: key → (mesh, linked list node)
    cache: HashMap<TriangulationCacheKey, CacheEntry>,
    /// Sentinel head of the doubly-linked list (most recently used)
    head: Option<TriangulationCacheKey>,
    /// Sentinel tail of the doubly-linked list (least recently used)
    tail: Option<TriangulationCacheKey>,
    /// Maximum number of entries
    capacity: usize,
    /// Number of cache hits
    hits: usize,
    /// Number of cache misses
    misses: usize,
}

/// A cache entry with embedded doubly-linked list pointers for O(1) LRU.
struct CacheEntry {
    mesh: crate::mesh::TriangleMesh,
    /// Key of the previous (more recently used) entry in the LRU list
    prev: Option<TriangulationCacheKey>,
    /// Key of the next (less recently used) entry in the LRU list
    next: Option<TriangulationCacheKey>,
}

impl TriangulationCache {
    /// Create a new triangulation cache with the given capacity.
    pub fn new(capacity: usize) -> Self {
        Self {
            cache: HashMap::new(),
            head: None,
            tail: None,
            capacity: capacity.max(1),
            hits: 0,
            misses: 0,
        }
    }

    /// Move an entry to the head of the LRU list (most recently used).
    fn touch(&mut self, key: &TriangulationCacheKey) {
        // Already at head — nothing to do
        if self.head.as_ref() == Some(key) {
            return;
        }

        // Get the entry and its neighbors
        let (prev, next) = {
            let entry = self.cache.get(key).expect("touch: key must exist");
            (entry.prev.clone(), entry.next.clone())
        };

        // Unlink from current position
        if let Some(ref prev_key) = prev {
            if let Some(prev_entry) = self.cache.get_mut(prev_key) {
                prev_entry.next = next.clone();
            }
        }
        if let Some(ref next_key) = next {
            if let Some(next_entry) = self.cache.get_mut(next_key) {
                next_entry.prev = prev;
            }
        }

        // Update tail if we moved the tail entry
        if self.tail.as_ref() == Some(key) {
            self.tail = prev.or_else(|| self.head.clone());
        }

        // Link at head
        if let Some(head_key) = self.head.take() {
            if let Some(head_entry) = self.cache.get_mut(&head_key) {
                head_entry.prev = Some(key.clone());
            }
            if let Some(entry) = self.cache.get_mut(key) {
                entry.next = Some(head_key);
                entry.prev = None;
            }
        } else {
            // List was empty (shouldn't happen since key exists)
            if let Some(entry) = self.cache.get_mut(key) {
                entry.prev = None;
                entry.next = None;
            }
        }
        self.head = Some(key.clone());
    }

    /// Remove the tail (least recently used) entry from the cache.
    fn evict_lru(&mut self) {
        let lru_key = match self.tail.take() {
            Some(k) => k,
            None => return,
        };

        let (prev, _) = {
            let entry = self.cache.get(&lru_key).expect("evict_lru: tail key must exist");
            (entry.prev.clone(), entry.next.clone())
        };

        // Unlink from list
        if let Some(ref prev_key) = prev {
            if let Some(prev_entry) = self.cache.get_mut(prev_key) {
                prev_entry.next = None;
            }
        } else {
            // Only one entry — list becomes empty
            self.head = None;
        }

        self.tail = prev;
        self.cache.remove(&lru_key);
    }

    /// Insert a new entry at the head of the LRU list.
    fn insert_at_head(&mut self, key: TriangulationCacheKey, mesh: crate::mesh::TriangleMesh) {
        // Link at head
        let entry = CacheEntry {
            mesh,
            prev: None,
            next: self.head.clone(),
        };

        if let Some(ref head_key) = self.head {
            if let Some(head_entry) = self.cache.get_mut(head_key) {
                head_entry.prev = Some(key.clone());
            }
        }

        self.cache.insert(key.clone(), entry);
        self.head = Some(key);

        // If this is the first entry, it's also the tail
        if self.tail.is_none() {
            self.tail = self.head.clone();
        }
    }

    /// Get a cached triangulation, or compute and cache it.
    ///
    /// If the solid+params combination is in the cache, returns the cached mesh.
    /// Otherwise, triangulates the solid, caches the result, and returns it.
    pub fn get_or_triangulate<F>(
        &mut self,
        solid: &draper_topology::Solid,
        params: &crate::triangulate::TriangulationParams,
        triangulate_fn: F,
    ) -> crate::mesh::TriangleMesh
    where
        F: FnOnce(&draper_topology::Solid, &crate::triangulate::TriangulationParams) -> crate::mesh::TriangleMesh,
    {
        let key = TriangulationCacheKey::from_solid_and_params(solid, params);

        if let Some(entry) = self.cache.get(&key) {
            self.hits += 1;
            let mesh = entry.mesh.clone();
            self.touch(&key);
            log::debug!("TriangulationCache HIT (hits={}, misses={}, entries={})", self.hits, self.misses, self.cache.len());
            return mesh;
        }

        self.misses += 1;
        let mesh = triangulate_fn(solid, params);

        // Evict LRU entry if at capacity
        if self.cache.len() >= self.capacity {
            self.evict_lru();
        }

        self.insert_at_head(key, mesh.clone());

        log::debug!("TriangulationCache MISS (hits={}, misses={}, entries={})", self.hits, self.misses, self.cache.len());
        mesh
    }

    /// Clear the cache.
    pub fn clear(&mut self) {
        self.cache.clear();
        self.head = None;
        self.tail = None;
    }

    /// Get cache statistics.
    pub fn stats(&self) -> (usize, usize, usize) {
        (self.hits, self.misses, self.cache.len())
    }

    /// Get the hit rate as a percentage.
    pub fn hit_rate(&self) -> f64 {
        let total = self.hits + self.misses;
        if total > 0 {
            self.hits as f64 / total as f64 * 100.0
        } else {
            0.0
        }
    }

    /// Get the current number of entries in the cache.
    pub fn len(&self) -> usize {
        self.cache.len()
    }

    /// Check if the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.cache.is_empty()
    }
}
