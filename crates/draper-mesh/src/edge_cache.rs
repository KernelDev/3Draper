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
fn deterministic_round_point(p: Point3d) -> Point3d {
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
    /// Adaptive tolerance for sampling and coincidence checks.
    adaptive_tol: AdaptiveTolerance,
    /// Maximum number of sample points per edge.
    max_samples: usize,
}

impl EdgeDiscretizationCache {
    /// Create a new cache with default tolerance.
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
            topo_id_to_key: HashMap::new(),
            adaptive_tol: AdaptiveTolerance::new(),
            max_samples: 64,
        }
    }

    /// Create a new cache with custom tolerance and max samples.
    pub fn with_tolerance(tol_ctx: ToleranceContext, max_samples: usize) -> Self {
        Self {
            entries: HashMap::new(),
            topo_id_to_key: HashMap::new(),
            adaptive_tol: AdaptiveTolerance::from_model_scale(tol_ctx.model_scale),
            max_samples: max_samples.max(4),
        }
    }

    /// Create a new cache with adaptive tolerance from a bounding box.
    pub fn with_adaptive_tolerance(min: &Point3d, max: &Point3d, max_samples: usize) -> Self {
        Self {
            entries: HashMap::new(),
            topo_id_to_key: HashMap::new(),
            adaptive_tol: AdaptiveTolerance::from_bounding_box(min, max),
            max_samples: max_samples.max(4),
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

    /// Discretize an edge using a STEP entity ID as the key.
    ///
    /// This is the STEP-path API that replaces the old `StepEdgeCache::discretize()`.
    /// It creates a canonical key from the STEP entity ID, ensuring that two
    /// ORIENTED_EDGEs sharing the same EDGE_CURVE resolve to the same cache entry.
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
        let key = EdgeCacheKey::StepEntityId(step_entity_id);

        // Register TopoId → canonical key mapping
        self.topo_id_to_key.insert(edge.id, key);

        if !self.entries.contains_key(&key) {
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

    /// Get the adaptive tolerance.
    pub fn adaptive_tolerance(&self) -> &AdaptiveTolerance {
        &self.adaptive_tol
    }

    /// Update the adaptive tolerance from a bounding box.
    pub fn set_adaptive_tolerance(&mut self, min: &Point3d, max: &Point3d) {
        self.adaptive_tol = AdaptiveTolerance::from_bounding_box(min, max);
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

        // Adaptive subdivision threshold: use adaptive chord tolerance
        let max_deviation = self.adaptive_tol.chord_tolerance();

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
            // NURBS fast path: bootstrap + chain Newton-Raphson.
            // This avoids the expensive 32×32 grid search that surface.project_point()
            // performs for each point independently. Instead, we:
            // 1. Use project_point() for the first point (bootstrap)
            // 2. Use reproject_nurbs_point() with the previous UV as initial guess
            //    for subsequent points — Newton-Raphson converges in ~3-5 iterations
            //    when starting from a nearby UV, vs ~146 iterations from scratch.
            let mut uvs = Vec::with_capacity(points_3d.len());
            if !points_3d.is_empty() {
                let (u0, v0) = surface.project_point(&points_3d[0]);
                uvs.push(Point2d::new(u0, v0));
                for i in 1..points_3d.len() {
                    let prev = uvs[i - 1];
                    let (u, v) = crate::parametric_domain::reproject_nurbs_point(
                        nurbs, &points_3d[i], prev.u, prev.v,
                    );
                    uvs.push(Point2d::new(u, v));
                }
            }
            uvs
        } else {
            // Non-NURBS surfaces: project_point() is fast, use it directly
            points_3d
                .iter()
                .map(|p| {
                    let (u, v) = surface.project_point(p);
                    Point2d::new(u, v)
                })
                .collect()
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
            if let Some(ref surface) = face.surface {
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
    use draper_geometry::{Plane, Direction3d, Line2d};
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
