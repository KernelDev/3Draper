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

use draper_geometry::{Point3d, Point2d, Curve3d, Curve2d, Surface, tolerance::ToleranceContext};
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

                // Skip first point if it duplicates the last point of previous segment
                if !all_points.is_empty() && !seg_points.is_empty() {
                    let last = all_points.last().unwrap();
                    let first = &seg_points[0];
                    let dx = first.x - last.x;
                    let dy = first.y - last.y;
                    let dz = first.z - last.z;
                    if (dx * dx + dy * dy + dz * dz) < 1e-12 {
                        seg_points.remove(0);
                        seg_params.remove(0);
                    }
                }

                all_points.extend(seg_points);
                all_params.extend(seg_params);
            }

            return (all_points, all_params);
        }

        // Adaptive subdivision threshold: use LOD override if set, else adaptive.
        let max_deviation = self.effective_chord_tolerance();

        // For CIRCLE curves, use a UNIFORM angular grid — no adaptive refinement.
        //
        // This is CRITICAL for watertightness and visual quality of tube faces
        // (cones, cylinders). When two circles on the same axis have different
        // radii (e.g., a cone's bottom R=30.36 and top R=35.22), adaptive
        // refinement produces DIFFERENT angular steps for each ring (more
        // points for larger radius). Connecting bottom[i] to top[i] by index
        // then produces TWISTED/SLIVER quads because the angles don't match.
        //
        // Using a uniform grid ensures both rings have points at the SAME
        // angular positions (2π*i/n), producing rectangular quads.
        //
        // n_samples is computed from chord tolerance using the radius, then
        // rounded up to the next multiple of 4 for better sharing between
        // circles with similar (but not identical) radii.
        if let Curve3d::Circle(ref circle) = curve {
            let angle_range = (t_max - t_min).abs();
            let r = circle.radius;
            let n_from_tol = if max_deviation > 0.0 && r > 0.0 {
                let half_angle = (1.0 - max_deviation / r).clamp(-1.0, 1.0).acos();
                ((angle_range / (2.0 * half_angle)).ceil() as usize).max(4)
            } else {
                n_samples_hint.max(8)
            };
            // Round up to next multiple of 4 for better angular alignment
            // between circles with different radii on the same axis.
            let n = (((n_from_tol + 3) / 4) * 4).min(self.max_samples).max(n_samples_hint.max(8));
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
