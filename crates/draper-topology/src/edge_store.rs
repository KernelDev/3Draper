// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Global edge store — canonical registry of B-Rep edges (C5 Stage 2).
//!
//! # The problem (C5)
//!
//! A shared edge between two adjacent faces is currently duplicated as two
//! separate `Edge` structs with **different TopoIds** (e.g. two
//! `ORIENTED_EDGE`s referencing the same `EDGE_CURVE` in a STEP file, or two
//! result faces of a boolean operation sharing a split edge). Consequences:
//!
//! - Healing that fixes an edge copy in one face does not propagate to the
//!   other face — roughly 30% of healing work exists only to compensate for
//!   this duplication.
//! - Mesh-level caches need ad-hoc alias maps (`topo_id_to_key`) to make the
//!   two copies discretize identically.
//! - Topology validation cannot detect that two "different" edges are
//!   actually the same topological entity.
//!
//! # The solution (this module)
//!
//! `EdgeStore` is the single source of truth for edge identity:
//!
//! - `edges` holds one **canonical** `Edge` per unique topological edge
//!   (deduplicated by `step_entity_id` where available).
//! - `aliases` maps every per-face *instance* TopoId to its canonical
//!   TopoId, so any code holding an instance id can resolve the shared edge:
//!   `store.get(instance_id)` transparently follows the alias.
//! - `by_step_id` indexes canonical edges by STEP entity id for O(1)
//!   STEP-level lookups.
//!
//! # Stage 2 semantics (non-breaking)
//!
//! `Face.edges: Vec<Edge>` per-face mirrors are **not removed** in this
//! stage — every existing consumer keeps working unchanged. The store is
//! populated by [`crate::entity::Solid::index_edges`] and provides:
//!
//! - identity resolution (`canonical_of`, `get` with alias following),
//! - a deduplicated iteration surface (`iter` yields each shared edge once),
//! - a target-state API (`Face.edge_ids`) that downstream stages migrate to.
//!
//! # Stage 4 semantics (2026-08-31) — store-first reads, derived mirrors
//!
//! The store is now the READ path of record: `Solid::resolve_edge` /
//! `Solid::face_edges` answer identity-aware queries with a mirror
//! fallback for un-indexed solids, and `Solid::sync_edge_mirrors`
//! reverse-propagates canonical edge fixes onto the per-face mirrors,
//! establishing the sanctioned mutation flow
//! `ensure → get_mut → sync_edge_mirrors`. Consumers are migrating
//! incrementally; the per-face `Vec<Edge>` mirrors remain as derived
//! data until the Stage 5 serde+API migration removes them entirely.

use crate::entity::{Edge, Face, Shell, Solid, TopoId};
use draper_geometry::Curve3d;
use std::collections::HashMap;

/// Statistics returned by [`Solid::index_edges`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EdgeDedupReport {
    /// Total number of edge instances scanned across all faces.
    pub total_instances: usize,
    /// Number of unique canonical edges in the resulting store.
    pub unique_edges: usize,
    /// Number of instance ids that are aliases (duplicates) of a canonical
    /// edge — i.e. `total_instances - unique_edges` minus in-face repeats.
    pub deduplicated: usize,
    /// Number of distinct STEP entity ids that had more than one instance.
    pub shared_step_edges: usize,
    /// Number of instance ids unified by GEOMETRIC identity (C5 Stage 3):
    /// native edges without `step_entity_id` that match an earlier edge's
    /// canonical geometric key (same curve + same endpoint pair).
    pub geometric_dedup: usize,
}

// ============================================================
// Geometric edge identity (C5 Stage 3)
//
// Native edges (builder primitives, boolean results) carry no
// `step_entity_id`, so Stage 2 could not unify shared instances of them.
// The geometric key below identifies an edge by its curve geometry plus
// its endpoint pair, direction-insensitively:
//
//   - Line: canonicalized (closest point to origin + sign-canonical
//     direction) — any parametrization of the same line matches.
//   - Circle: center + sign-canonical normal + radius. `x_axis` is a
//     parametrization artifact for a full circle and is excluded.
//   - Ellipse / Hyperbola / Parabola: placement + axes + scalar params.
//     `x_axis` IS geometric here (it orients the major/transverse axis).
//   - Arc: circle key + angle pair (min, max) — same arc from either
//     direction matches.
//   - NURBS: degree + quantized control points + weights + knots.
//     Reversed representations intentionally do NOT match (conservative).
//   - `None` and `PCurve` curves are excluded: endpoints alone cannot
//     distinguish e.g. two arcs of different circles joining the same
//     points (lens shapes), and pcurves live in surface-parametric space.
//
// All coordinates are quantized onto a 1e-9 absolute grid: far below the
// loosest modeling tolerance (STEP default 1e-6), far above double
// rounding noise at mm-scale coordinates — only bit-level near-identical
// geometries collide. A missed match is always safe (no dedup); a false
// match would require two *distinct* edges with identical curve geometry
// AND identical endpoint pair inside one solid.
// ============================================================

/// Quantization grid for geometric edge keys (absolute, in model units).
const GEOM_KEY_GRID: f64 = 1e-9;

/// Quantize a coordinate onto the key grid. Non-finite values collapse to 0
/// (edges with NaN/Inf geometry cannot be identified and will simply fail
/// to match anything else — the safe direction).
fn q(v: f64) -> i64 {
    if v.is_finite() {
        (v / GEOM_KEY_GRID).round() as i64
    } else {
        0
    }
}

/// Sign-canonical quantized direction: the first non-zero component is
/// positive, so `(d)` and `(-d)` hash identically.
fn canon_dir(d: &draper_geometry::Direction3d) -> (i64, i64, i64) {
    let (x, y, z) = (q(d.x), q(d.y), q(d.z));
    let flip = x < 0 || (x == 0 && (y < 0 || (y == 0 && z < 0)));
    if flip {
        (-x, -y, -z)
    } else {
        (x, y, z)
    }
}

/// Direction-insensitive geometric identity key for an edge instance.
///
/// Returns `None` for edges that cannot be identified geometrically
/// (`curve == None`, PCurve, non-finite geometry) — those never take part
/// in geometric dedup.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub struct GeomEdgeKey {
    kind: u8,
    params: Vec<i64>,
    /// Quantized (start, end) pair, lexicographically ordered so a reversed
    /// instance of the same edge produces the same key.
    endpoints: [i64; 6],
}

/// Compute the geometric identity key of an edge (see module docs).
pub fn geom_edge_key(edge: &Edge) -> Option<GeomEdgeKey> {
    let curve = edge.curve.as_ref()?;
    let (kind, params) = match curve {
        Curve3d::Line(l) => {
            // Canonical point on the line: the closest point to the origin.
            // `origin` itself is a parametrization artifact.
            let d = &l.direction;
            let o = &l.origin;
            let dot = o.x * d.x + o.y * d.y + o.z * d.z;
            let px = q(o.x - d.x * dot);
            let py = q(o.y - d.y * dot);
            let pz = q(o.z - d.z * dot);
            let (dx, dy, dz) = canon_dir(d);
            (1u8, vec![px, py, pz, dx, dy, dz])
        }
        Curve3d::Circle(c) => {
            let (nx, ny, nz) = canon_dir(&c.normal);
            (
                2u8,
                vec![q(c.center.x), q(c.center.y), q(c.center.z), nx, ny, nz, q(c.radius)],
            )
        }
        Curve3d::Ellipse(e) => {
            let (nx, ny, nz) = canon_dir(&e.normal);
            let (ax, ay, az) = canon_dir(&e.x_axis);
            (
                3u8,
                vec![
                    q(e.center.x),
                    q(e.center.y),
                    q(e.center.z),
                    nx,
                    ny,
                    nz,
                    ax,
                    ay,
                    az,
                    q(e.semi_major),
                    q(e.semi_minor),
                ],
            )
        }
        Curve3d::Arc(a) => {
            let c = &a.circle;
            let (nx, ny, nz) = canon_dir(&c.normal);
            let (ax, ay, az) = canon_dir(&c.x_axis);
            let (s, e) = (q(a.start_angle), q(a.end_angle));
            // Direction-insensitive angle pair.
            let (lo, hi) = if s <= e { (s, e) } else { (e, s) };
            (
                4u8,
                vec![
                    q(c.center.x),
                    q(c.center.y),
                    q(c.center.z),
                    nx,
                    ny,
                    nz,
                    ax,
                    ay,
                    az,
                    q(c.radius),
                    lo,
                    hi,
                ],
            )
        }
        Curve3d::Hyperbola(h) => {
            let (nx, ny, nz) = canon_dir(&h.normal);
            let (ax, ay, az) = canon_dir(&h.x_axis);
            (
                5u8,
                vec![
                    q(h.center.x),
                    q(h.center.y),
                    q(h.center.z),
                    nx,
                    ny,
                    nz,
                    ax,
                    ay,
                    az,
                    q(h.semi_real),
                    q(h.semi_imag),
                ],
            )
        }
        Curve3d::Parabola(p) => {
            let (nx, ny, nz) = canon_dir(&p.normal);
            let (ax, ay, az) = canon_dir(&p.x_axis);
            (
                6u8,
                vec![
                    q(p.vertex.x),
                    q(p.vertex.y),
                    q(p.vertex.z),
                    nx,
                    ny,
                    nz,
                    ax,
                    ay,
                    az,
                    q(p.focal_dist),
                ],
            )
        }
        Curve3d::Nurbs(n) => {
            let mut params = Vec::with_capacity(4 + 4 * n.control_points.len() + n.knots.len());
            params.push(n.degree as i64);
            for cp in &n.control_points {
                params.push(q(cp.x));
                params.push(q(cp.y));
                params.push(q(cp.z));
            }
            for w in &n.weights {
                params.push(q(*w));
            }
            for k in &n.knots {
                params.push(q(*k));
            }
            (7u8, params)
        }
        // PCurve lives in surface-parametric space — excluded (see docs).
        // Trimmed/Composite bases could in principle be keyed recursively,
        // but their parameter conventions are representation-dependent;
        // excluded in Stage 3 (conservative — a missed match is safe).
        Curve3d::PCurve { .. } | Curve3d::Trimmed { .. } | Curve3d::Composite { .. } => {
            return None
        }
    };

    // Endpoint pair: prefer authoritative VERTEX_POINT coords, fall back to
    // curve evaluation at the parametric range.
    let sp = edge
        .start_vertex_point
        .unwrap_or_else(|| curve.point_at(edge.param_range.0));
    let ep = edge
        .end_vertex_point
        .unwrap_or_else(|| curve.point_at(edge.param_range.1));
    let mut a = [q(sp.x), q(sp.y), q(sp.z)];
    let mut b = [q(ep.x), q(ep.y), q(ep.z)];
    if b < a {
        std::mem::swap(&mut a, &mut b);
    }

    Some(GeomEdgeKey {
        kind,
        params,
        endpoints: [a[0], a[1], a[2], b[0], b[1], b[2]],
    })
}

/// Canonical registry of B-Rep edges.
///
/// See the module documentation for the architecture. The store is cheap to
/// rebuild (`Solid::index_edges`).
///
/// C5 Stage 5.1 (2026-08-31): the store IS serialized on `Solid` (flat
/// format below), so canonical identity + alias mappings survive
/// serialization round-trips. Legacy payloads written before Stage 5 carry
/// no `edge_store` field; deserialization defaults it to empty and
/// `Solid::ensure_edge_store` rebuilds it from the per-face mirrors.
#[derive(Clone, Debug, Default)]
pub struct EdgeStore {
    /// Canonical edges, keyed by canonical TopoId.
    edges: HashMap<TopoId, Edge>,
    /// Instance TopoId → canonical TopoId (identity mappings are omitted).
    aliases: HashMap<TopoId, TopoId>,
    /// STEP entity id → canonical TopoId.
    by_step_id: HashMap<i64, TopoId>,
}

// ============================================================
// Serde (C5 Stage 5.1)
//
// HashMaps with `TopoId` keys cannot serialize to JSON directly (newtype
// structs are not valid JSON map keys), so the flat on-wire format uses
// sorted Vecs: edges as an array, aliases/by_step_id as (key, value) pair
// arrays. Reconstruction rebuilds the HashMaps.
// ============================================================

#[cfg(feature = "serde")]
mod serde_impl {
    use super::EdgeStore;
    use crate::entity::{Edge, TopoId};
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    /// Flat (de)serialization format for [`EdgeStore`].
    #[derive(Serialize, Deserialize)]
    struct EdgeStoreData {
        /// Canonical edges.
        edges: Vec<Edge>,
        /// (instance TopoId, canonical TopoId) alias pairs.
        aliases: Vec<(TopoId, TopoId)>,
        /// (STEP entity id, canonical TopoId) index pairs.
        by_step_id: Vec<(i64, TopoId)>,
    }

    impl From<&EdgeStore> for EdgeStoreData {
        fn from(store: &EdgeStore) -> Self {
            let mut edges: Vec<Edge> = store.edges.values().cloned().collect();
            edges.sort_by_key(|e| e.id);
            let mut aliases: Vec<(TopoId, TopoId)> =
                store.aliases.iter().map(|(&k, &v)| (k, v)).collect();
            aliases.sort();
            let mut by_step_id: Vec<(i64, TopoId)> =
                store.by_step_id.iter().map(|(&k, &v)| (k, v)).collect();
            by_step_id.sort();
            Self { edges, aliases, by_step_id }
        }
    }

    impl From<EdgeStoreData> for EdgeStore {
        fn from(data: EdgeStoreData) -> Self {
            let mut store = EdgeStore::new();
            for edge in data.edges {
                let id = edge.id;
                if let Some(step_id) = edge.step_entity_id {
                    store.by_step_id.insert(step_id, id);
                }
                store.edges.insert(id, edge);
            }
            for (instance, canonical) in data.aliases {
                if instance != canonical {
                    store.aliases.insert(instance, canonical);
                }
            }
            for (step_id, canonical) in data.by_step_id {
                store.by_step_id.entry(step_id).or_insert(canonical);
            }
            store
        }
    }

    impl Serialize for EdgeStore {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            EdgeStoreData::from(self).serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for EdgeStore {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            EdgeStoreData::deserialize(deserializer).map(EdgeStore::from)
        }
    }
}

impl EdgeStore {
    /// Create an empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a canonical edge (no dedup — the edge id becomes its own key).
    /// Returns the canonical TopoId.
    pub fn insert(&mut self, edge: Edge) -> TopoId {
        let id = edge.id;
        if let Some(step_id) = edge.step_entity_id {
            self.by_step_id.insert(step_id, id);
        }
        self.edges.insert(id, edge);
        id
    }

    /// Register `instance_id` as an alias of `canonical_id`.
    ///
    /// Idempotent; identity mappings (instance == canonical) are ignored.
    pub fn add_alias(&mut self, instance_id: TopoId, canonical_id: TopoId) {
        if instance_id != canonical_id {
            self.aliases.insert(instance_id, canonical_id);
        }
    }

    /// Resolve an edge id (instance or canonical) to its canonical TopoId.
    pub fn canonical_of(&self, id: TopoId) -> TopoId {
        match self.aliases.get(&id) {
            Some(&canonical) => canonical,
            None => id,
        }
    }

    /// Look up an edge by instance **or** canonical id (aliases are followed
    /// transparently). Returns the canonical edge.
    pub fn get(&self, id: TopoId) -> Option<&Edge> {
        let canonical = self.canonical_of(id);
        self.edges.get(&canonical)
    }

    /// Look up a canonical edge by its canonical id only (no alias following).
    pub fn get_canonical(&self, id: TopoId) -> Option<&Edge> {
        self.edges.get(&id)
    }

    /// Look up a canonical edge by STEP entity id.
    pub fn find_by_step_id(&self, step_id: i64) -> Option<&Edge> {
        self.by_step_id
            .get(&step_id)
            .and_then(|&canonical| self.edges.get(&canonical))
    }

    /// Mutable access to a canonical edge by canonical id.
    ///
    /// Modifying a canonical edge through the store is the C5-sanctioned way
    /// to edit shared edges — Stage 3 consumers migrate their in-place edits
    /// to this entry point so that fixes propagate to every incident face.
    pub fn get_mut(&mut self, id: TopoId) -> Option<&mut Edge> {
        let canonical = self.canonical_of(id);
        self.edges.get_mut(&canonical)
    }

    /// Remove a canonical edge and all its aliases.
    /// Returns the removed edge, if it existed.
    pub fn remove(&mut self, id: TopoId) -> Option<Edge> {
        let canonical = self.canonical_of(id);
        let removed = self.edges.remove(&canonical)?;
        if let Some(step_id) = removed.step_entity_id {
            self.by_step_id.remove(&step_id);
        }
        self.aliases.retain(|_, &mut c| c != canonical);
        Some(removed)
    }

    /// Iterate over canonical edges (each shared edge appears exactly once).
    pub fn iter(&self) -> impl Iterator<Item = &Edge> {
        self.edges.values()
    }

    /// Iterate over canonical ids.
    pub fn iter_ids(&self) -> impl Iterator<Item = TopoId> + '_ {
        self.edges.keys().copied()
    }

    /// Iterate over (instance id, canonical id) alias pairs.
    pub fn iter_aliases(&self) -> impl Iterator<Item = (TopoId, TopoId)> + '_ {
        self.aliases.iter().map(|(&k, &v)| (k, v))
    }

    /// Number of canonical edges.
    pub fn len(&self) -> usize {
        self.edges.len()
    }

    /// Whether the store holds no canonical edges.
    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    /// Number of alias (duplicate instance) mappings.
    pub fn alias_count(&self) -> usize {
        self.aliases.len()
    }

    /// Whether `a` and `b` resolve to the same canonical edge.
    pub fn same_edge(&self, a: TopoId, b: TopoId) -> bool {
        !self.is_empty() && self.canonical_of(a) == self.canonical_of(b)
    }
}

impl Solid {
    /// Build (or rebuild) the edge store for this solid.
    ///
    /// Scans every face edge instance across all shells, deduplicates them by
    /// `step_entity_id` (STEP path — the only duplication source with a
    /// reliable identity key today; native geometric dedup is Stage 3), and
    /// populates `self.edge_store` with canonical edges + alias mappings.
    /// `Face.edge_ids` mirrors are (re)synced to canonical ids.
    ///
    /// Non-breaking: per-face `Face.edges` mirrors are left untouched, so all
    /// existing consumers behave exactly as before.
    pub fn index_edges(&mut self) -> EdgeDedupReport {
        let mut store = EdgeStore::new();
        let mut report = EdgeDedupReport::default();
        // step_entity_id → canonical id (local, to count shared edges).
        let mut seen_step: HashMap<i64, TopoId> = HashMap::new();
        // Geometric key → canonical id for native edges (C5 Stage 3).
        let mut seen_geom: HashMap<GeomEdgeKey, TopoId> = HashMap::new();
        // Per-face canonical id lists, parallel to shells/faces order.
        let mut face_edge_ids: Vec<Vec<TopoId>> = Vec::new();

        // ── Pass 1: collect canonical edges + alias map ──────────────────
        let mut shells: Vec<&mut Shell> = Vec::new();
        if let Some(ref mut shell) = self.outer_shell {
            shells.push(shell);
        }
        for shell in &mut self.inner_shells {
            shells.push(shell);
        }
        for shell in shells.iter_mut() {
            for face in shell.faces.iter_mut() {
                let mut canonical_ids = Vec::with_capacity(face.edges.len());
                for edge in &face.edges {
                    report.total_instances += 1;
                    let canonical_id = if let Some(step_id) = edge.step_entity_id {
                        match seen_step.get(&step_id) {
                            Some(&existing) => {
                                report.deduplicated += 1;
                                existing
                            }
                            None => {
                                seen_step.insert(step_id, edge.id);
                                edge.id
                            }
                        }
                    } else if let Some(key) = geom_edge_key(edge) {
                        // C5 Stage 3: native edges unify by geometric identity
                        // (same curve + same endpoint pair, direction-insensitive).
                        match seen_geom.get(&key) {
                            Some(&existing) => {
                                report.deduplicated += 1;
                                report.geometric_dedup += 1;
                                existing
                            }
                            None => {
                                seen_geom.insert(key, edge.id);
                                edge.id
                            }
                        }
                    } else {
                        edge.id
                    };
                    store.add_alias(edge.id, canonical_id);
                    canonical_ids.push(canonical_id);
                    // Insert the canonical edge once (first occurrence wins;
                    // prefer a copy with a curve when we see one later).
                    match store.get_canonical(canonical_id) {
                        None => {
                            store.insert(edge.clone());
                        }
                        Some(existing) => {
                            if existing.curve.is_none() && edge.curve.is_some() {
                                // Upgrade the canonical copy with curve data,
                                // preserving the canonical id.
                                let mut better = edge.clone();
                                better.id = canonical_id;
                                if let Some(step_id) = better.step_entity_id {
                                    store.by_step_id.insert(step_id, canonical_id);
                                }
                                store.edges.insert(canonical_id, better);
                            }
                        }
                    }
                }
                face_edge_ids.push(canonical_ids);
            }
        }

        report.unique_edges = store.len();
        report.shared_step_edges = face_edge_ids
            .iter()
            .flatten()
            .copied()
            .collect::<std::collections::HashSet<_>>()
            .iter()
            .filter(|&&id| {
                store
                    .get_canonical(id)
                    .map(|e| {
                        e.step_entity_id
                            .map(|s| seen_step.get(&s) == Some(&id))
                            .unwrap_or(false)
                    })
                    .unwrap_or(false)
                    // shared = appears more than once among instance ids
                    && store.aliases.values().any(|&c| c == id)
            })
            .count();

        // ── Pass 2: sync Face.edge_ids mirrors to canonical ids ──────────
        let mut idx = 0usize;
        let mut shells: Vec<&mut Shell> = Vec::new();
        if let Some(ref mut shell) = self.outer_shell {
            shells.push(shell);
        }
        for shell in &mut self.inner_shells {
            shells.push(shell);
        }
        for shell in shells.iter_mut() {
            for face in shell.faces.iter_mut() {
                if idx < face_edge_ids.len() {
                    face.edge_ids = std::mem::take(&mut face_edge_ids[idx]);
                }
                idx += 1;
            }
        }

        self.edge_store = store;
        report
    }

    /// Ensure the edge store is populated (idempotent, cheap when built).
    ///
    /// Callers that receive a `&mut Solid` from deserialization or from
    /// builder paths use this before reading `solid.edge_store`.
    pub fn ensure_edge_store(&mut self) {
        if self.edge_store.is_empty() {
            self.index_edges();
        }
    }

    /// Propagate unambiguous edge fixes across instances of the same shared
    /// edge (C5 Stage 3).
    ///
    /// Healing and validation mutate per-face `Face.edges` copies
    /// independently. When two faces share one topological edge (same
    /// `step_entity_id` — e.g. after tolerant stitching — or the same
    /// geometric key), a fix applied to one copy leaves the other stale,
    /// which is precisely the cross-face inconsistency C5 eliminates.
    /// This pass groups instances by shared identity and reconciles the
    /// orientation-independent fields:
    ///
    /// - `degenerate`: OR — if one instance is degenerate, the edge is
    ///   degenerate for every incident face.
    /// - `tolerance`: MAX — tolerant-modeling semantics (the loosest
    ///   tolerance wins).
    /// - `curve`: backfilled into curve-less copies **only when their
    ///   parametric range matches the donor's range (or its swap)**, so the
    ///   backfilled geometry is guaranteed to describe the same segment.
    ///
    /// Orientation-dependent fields (`param_range`, `forward`, vertex ids,
    /// vertex points) are deliberately NOT propagated — reversed instances
    /// legitimately carry swapped values.
    ///
    /// Returns the number of instance fields updated.
    pub fn propagate_edge_fixes(&mut self) -> usize {
        /// Identity of a shared edge for the reconciliation pass.
        #[derive(Clone, PartialEq, Eq, Hash)]
        enum SharedKey {
            Step(i64),
            Geom(GeomEdgeKey),
            /// Instances that cannot be identified never share a group.
            Unique(u64),
        }

        // Pass 1 (read-only): aggregate the reconciliation values per group.
        let mut agg: HashMap<SharedKey, (bool, f64, Option<(Curve3d, (f64, f64))>)> =
            HashMap::new();
        let mut unique_counter: u64 = 0;
        {
            let mut shells: Vec<&Shell> = Vec::new();
            if let Some(ref shell) = self.outer_shell {
                shells.push(shell);
            }
            for shell in &self.inner_shells {
                shells.push(shell);
            }
            for shell in shells {
                for face in &shell.faces {
                    for edge in &face.edges {
                        let key = match (edge.step_entity_id, geom_edge_key(edge)) {
                            (Some(s), _) => SharedKey::Step(s),
                            (None, Some(g)) => SharedKey::Geom(g),
                            (None, None) => {
                                unique_counter += 1;
                                SharedKey::Unique(unique_counter)
                            }
                        };
                        let entry = agg.entry(key).or_insert((false, 0.0, None));
                        entry.0 |= edge.degenerate;
                        entry.1 = entry.1.max(edge.tolerance);
                        if entry.2.is_none() && edge.curve.is_some() {
                            entry.2 = edge.curve.clone().map(|c| (c, edge.param_range));
                        }
                    }
                }
            }
        }

        // Pass 2: apply the reconciled values to every instance.
        let mut updated = 0usize;
        let mut shells: Vec<&mut Shell> = Vec::new();
        if let Some(ref mut shell) = self.outer_shell {
            shells.push(shell);
        }
        for shell in &mut self.inner_shells {
            shells.push(shell);
        }
        for shell in shells.iter_mut() {
            for face in shell.faces.iter_mut() {
                for edge in face.edges.iter_mut() {
                    let key = match (edge.step_entity_id, geom_edge_key(edge)) {
                        (Some(s), _) => SharedKey::Step(s),
                        (None, Some(g)) => SharedKey::Geom(g),
                        // Unidentifiable instances never share a group.
                        (None, None) => continue,
                    };
                    let Some(&(degenerate, tolerance, ref donor)) = agg.get(&key) else {
                        continue;
                    };
                    if edge.degenerate != degenerate {
                        edge.degenerate = degenerate;
                        updated += 1;
                    }
                    if edge.tolerance < tolerance {
                        edge.tolerance = tolerance;
                        updated += 1;
                    }
                    if edge.curve.is_none() {
                        if let Some((ref curve, ref donor_range)) = donor {
                            let same = edge.param_range == *donor_range;
                            let swapped =
                                (edge.param_range.1, edge.param_range.0) == *donor_range;
                            if same || swapped {
                                edge.curve = Some(curve.clone());
                                updated += 1;
                            }
                        }
                    }
                }
            }
        }
        updated
    }

    /// Resolve any edge id — instance or canonical — to the canonical edge
    /// (C5 Stage 4 read API).
    ///
    /// Store-first: aliases are followed transparently, so the two per-face
    /// copies of one shared edge resolve to the SAME `&Edge`. When the store
    /// cannot answer (not yet indexed, or the edge was appended after the
    /// last `index_edges` call), the per-face mirrors are scanned as a
    /// fallback — the pre-C5 lookup semantics, now encapsulated.
    ///
    /// Consumers previously wrote `face.edges.iter().find(|e| e.id == id)`;
    /// migrating that pattern to `solid.resolve_edge(id)` is the Stage 4
    /// read-path migration step.
    pub fn resolve_edge(&self, id: TopoId) -> Option<&Edge> {
        if let Some(edge) = self.edge_store.get(id) {
            return Some(edge);
        }
        self.faces()
            .iter()
            .flat_map(|face| face.edges.iter())
            .find(|edge| edge.id == id)
    }

    /// Instance-faithful edge list of `face`, resolved through the canonical
    /// store (C5 Stage 4 read API).
    ///
    /// Element `i` corresponds to face instance `i` (parallel to
    /// `face.edges`): if the face has been indexed and the store still holds
    /// that edge, the CANONICAL edge is returned — shared edges from
    /// adjacent faces compare equal by pointer. Un-indexed or drifted
    /// entries fall back to the mirror entry, so behavior never regresses
    /// for builder-created or post-mutation faces.
    pub fn face_edges<'s>(&'s self, face: &'s Face) -> Vec<&'s Edge> {
        let store = &self.edge_store;
        let n = face.edges.len();
        let indexed = n > 0 && face.edge_ids.len() == n;
        (0..n)
            .map(|i| match indexed {
                true => store.get(face.edge_ids[i]).unwrap_or(&face.edges[i]),
                false => &face.edges[i],
            })
            .collect()
    }

    /// Propagate canonical edge field fixes onto the per-face `edges`
    /// mirrors (C5 Stage 4 — the store becomes the source of truth).
    ///
    /// The C5-sanctioned mutation flow is now:
    ///
    /// ```text
    /// solid.ensure_edge_store();
    /// if let Some(edge) = solid.edge_store.get_mut(instance_id) {
    ///     edge.tolerance = 0.5;            // fix the CANONICAL edge once
    /// }
    /// solid.sync_edge_mirrors();           // every incident face sees it
    /// ```
    ///
    /// Only orientation-INDEPENDENT fields are propagated from the canonical
    /// edge onto each instance:
    ///
    /// - `degenerate`, `tolerance`, `step_entity_id` — unconditional;
    /// - `curve` — only when the instance's `param_range` equals the
    ///   canonical range (or its swap), the same guard
    ///   [`Solid::propagate_edge_fixes`] uses to guarantee the backfilled
    ///   curve describes the same segment.
    ///
    /// Per-instance orientation fields (`id`, `param_range`, `forward`,
    /// `vertex_start`/`vertex_end`, vertex points) are never overwritten —
    /// reversed instances legitimately carry swapped values, and coedges
    /// keep referencing the instance ids.
    ///
    /// Returns the number of instance field updates applied. Idempotent: a
    /// second call without intervening store mutations returns 0.
    pub fn sync_edge_mirrors(&mut self) -> usize {
        if self.edge_store.is_empty() {
            return 0;
        }
        let store = &self.edge_store;
        let mut updated = 0usize;

        let mut shells: Vec<&mut Shell> = Vec::new();
        if let Some(ref mut shell) = self.outer_shell {
            shells.push(shell);
        }
        for shell in &mut self.inner_shells {
            shells.push(shell);
        }
        for shell in shells.iter_mut() {
            for face in shell.faces.iter_mut() {
                // `edge_ids[i]` (canonical) or `edges[i].id` (un-indexed).
                let instance_ids: Vec<TopoId> = if face.edge_ids.len() == face.edges.len()
                    && !face.edge_ids.is_empty()
                {
                    face.edge_ids.clone()
                } else {
                    face.edges.iter().map(|e| e.id).collect()
                };
                for (edge, &instance_id) in face.edges.iter_mut().zip(instance_ids.iter()) {
                    let Some(canonical) = store.get(instance_id) else {
                        continue;
                    };
                    if edge.degenerate != canonical.degenerate {
                        edge.degenerate = canonical.degenerate;
                        updated += 1;
                    }
                    if edge.tolerance != canonical.tolerance
                        && canonical.tolerance > edge.tolerance
                    {
                        edge.tolerance = canonical.tolerance;
                        updated += 1;
                    }
                    if edge.step_entity_id.is_none() && canonical.step_entity_id.is_some() {
                        edge.step_entity_id = canonical.step_entity_id;
                        updated += 1;
                    }
                    if edge.curve.is_none() && canonical.curve.is_some() {
                        let same = edge.param_range == canonical.param_range;
                        let swapped =
                            (edge.param_range.1, edge.param_range.0) == canonical.param_range;
                        if same || swapped {
                            edge.curve = canonical.curve.clone();
                            updated += 1;
                        }
                    }
                }
            }
        }
        updated
    }
}

impl Face {
    /// Canonical edge references (C5 target-state API).
    ///
    /// Parallel to `edges` but holding canonical ids from the solid's
    /// `EdgeStore` — a shared edge has the same id in every incident face.
    /// Populated by [`Solid::index_edges`]; safe to leave empty for faces
    /// that have not been indexed (falls back to `edges[i].id` semantics).
    pub fn canonical_edge_ids(&self) -> Vec<TopoId> {
        if self.edge_ids.len() == self.edges.len() && !self.edge_ids.is_empty() {
            self.edge_ids.clone()
        } else {
            self.edges.iter().map(|e| e.id).collect()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{CoEdge, Edge, Face, Wire};
    use draper_geometry::{Point3d, Surface, Plane};

    fn line_edge(from: Point3d, to: Point3d, step_id: Option<i64>) -> Edge {
        let mut edge = Edge::new_line(from, to);
        edge.step_entity_id = step_id;
        edge.start_vertex_point = Some(from);
        edge.end_vertex_point = Some(to);
        edge
    }

    fn square_face(edge_step_ids: &[Option<i64>]) -> Face {
        // Build a face with 4 line edges (ids passed through), no real wires
        // needed for store tests.
        let mut face = Face::new_surface_only(Surface::Plane(Plane::xy()));
        for (i, sid) in edge_step_ids.iter().enumerate() {
            let a = (i as f64) * 1.0;
            face.edges.push(line_edge(
                Point3d::new(a, 0.0, 0.0),
                Point3d::new(a + 1.0, 0.0, 0.0),
                *sid,
            ));
        }
        face
    }

    fn solid_of(faces: Vec<Face>) -> Solid {
        let shell = crate::entity::Shell::new_closed(faces);
        Solid::new(shell)
    }

    #[test]
    fn test_store_insert_get_alias() {
        let mut store = EdgeStore::new();
        let e = line_edge(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Some(42),
        );
        let id = e.id;
        store.insert(e);
        let other = TopoId::new();
        store.add_alias(other, id);

        assert_eq!(store.len(), 1);
        assert!(store.get(id).is_some());
        assert!(store.get(other).is_some(), "alias must resolve");
        assert_eq!(store.canonical_of(other), id);
        assert!(store.same_edge(id, other));
        assert!(store.find_by_step_id(42).is_some());
        assert_eq!(store.alias_count(), 1);
    }

    #[test]
    fn test_store_remove_drops_aliases() {
        let mut store = EdgeStore::new();
        let e = line_edge(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Some(7),
        );
        let id = e.id;
        store.insert(e);
        let alias = TopoId::new();
        store.add_alias(alias, id);

        assert!(store.remove(alias).is_some());
        assert!(store.is_empty());
        assert!(store.get(alias).is_none());
        assert_eq!(store.alias_count(), 0);
    }

    #[test]
    fn test_index_edges_dedups_shared_step_edges() {
        // Two faces sharing the same STEP EDGE_CURVE (#100) with different
        // instance TopoIds — the core C5 duplication scenario.
        let mut face_a = square_face(&[Some(100), Some(101)]);
        let mut face_b = square_face(&[Some(100), Some(102)]);
        // Give the shared edge instances different ids (as the converter does).
        assert_ne!(face_a.edges[0].id, face_b.edges[0].id);

        let mut solid = solid_of(vec![face_a, face_b]);
        let report = solid.index_edges();

        assert_eq!(report.total_instances, 4);
        assert_eq!(report.unique_edges, 3, "100 shared + 101 + 102");
        assert_eq!(report.deduplicated, 1);
        assert_eq!(solid.edge_store.len(), 3);

        // Both faces' canonical edge_ids reference the SAME id for step #100.
        let a_shared = solid.outer_shell.as_ref().unwrap().faces[0].edge_ids[0];
        let b_shared = solid.outer_shell.as_ref().unwrap().faces[1].edge_ids[0];
        assert_eq!(a_shared, b_shared, "shared edge must unify to one canonical id");

        // Per-face mirrors untouched: still 2 entries each.
        assert_eq!(solid.outer_shell.as_ref().unwrap().faces[0].edges.len(), 2);
        assert_eq!(solid.outer_shell.as_ref().unwrap().faces[1].edges.len(), 2);

        // Instance-id lookup resolves to the canonical edge.
        let inst_a = solid.outer_shell.as_ref().unwrap().faces[0].edges[0].id;
        let inst_b = solid.outer_shell.as_ref().unwrap().faces[1].edges[0].id;
        assert!(solid.edge_store.same_edge(inst_a, inst_b));
        assert!(solid.edge_store.get(inst_b).is_some());
        assert_eq!(solid.edge_store.find_by_step_id(100).unwrap().id, a_shared);
    }

    #[test]
    fn test_index_edges_keeps_seam_double_use() {
        // The same STEP edge used TWICE within ONE face (seam) must keep both
        // mirror entries — only identity is unified, not the instance count.
        let face = square_face(&[Some(200), Some(200)]);
        let mut solid = solid_of(vec![face]);
        let report = solid.index_edges();

        assert_eq!(report.total_instances, 2);
        assert_eq!(report.unique_edges, 1);
        assert_eq!(report.deduplicated, 1);
        // Both mirror entries survive (seam traversed twice).
        assert_eq!(solid.outer_shell.as_ref().unwrap().faces[0].edges.len(), 2);
        // Both edge_ids are canonical and equal.
        let f = &solid.outer_shell.as_ref().unwrap().faces[0];
        assert_eq!(f.edge_ids[0], f.edge_ids[1]);
    }

    #[test]
    fn test_index_edges_no_step_ids() {
        // C5 Stage 3: native edges without step_entity_id now unify by
        // GEOMETRIC identity. The two faces below have IDENTICAL edge
        // geometry (same lines, same endpoints) → each geometric edge
        // exists once canonically.
        let face_a = square_face(&[None, None]);
        let face_b = square_face(&[None, None]);
        let mut solid = solid_of(vec![face_a, face_b]);
        let report = solid.index_edges();

        assert_eq!(report.total_instances, 4);
        assert_eq!(report.unique_edges, 2, "two distinct geometric edges");
        assert_eq!(report.deduplicated, 2);
        assert_eq!(report.geometric_dedup, 2);
        // face_b's instances must alias face_a's canonicals.
        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        assert!(solid
            .edge_store
            .same_edge(fa.edges[0].id, fb.edges[0].id));
        assert!(solid
            .edge_store
            .same_edge(fa.edges[1].id, fb.edges[1].id));
        assert!(!solid
            .edge_store
            .same_edge(fa.edges[0].id, fa.edges[1].id));
    }

    #[test]
    fn test_geometric_dedup_direction_insensitive() {
        // A reversed instance of the same shared edge (opposite direction,
        // swapped param_range, forward=false) must unify with the original.
        let mut face_a = square_face(&[None]);
        let mut face_b = square_face(&[None]);
        // Reverse face_b's copy in place: swap endpoints + param_range.
        {
            let edge = &mut face_b.edges[0];
            let sp = edge.start_vertex_point;
            edge.start_vertex_point = edge.end_vertex_point;
            edge.end_vertex_point = sp;
            edge.param_range = (edge.param_range.1, edge.param_range.0);
            edge.forward = !edge.forward;
        }
        let mut solid = solid_of(vec![face_a, face_b]);
        let report = solid.index_edges();

        assert_eq!(report.geometric_dedup, 1);
        assert_eq!(report.unique_edges, 1);
        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        assert!(solid
            .edge_store
            .same_edge(fa.edges[0].id, fb.edges[0].id));
    }

    #[test]
    fn test_geometric_dedup_no_false_merge_lens() {
        // Two DIFFERENT arcs joining the same endpoints (lens shape) must
        // NOT merge: different circle centers → different keys.
        let mut face = Face::new_surface_only(Surface::Plane(Plane::xy()));
        let circle_a = draper_geometry::Circle::new(
            Point3d::new(0.0, 1.0, 0.0),
            draper_geometry::Direction3d::Z,
            1.0,
        );
        let circle_b = draper_geometry::Circle::new(
            Point3d::new(0.0, -1.0, 0.0),
            draper_geometry::Direction3d::Z,
            1.0,
        );
        // Both arcs pass through (0,0,0); full circles with distinct centers.
        let mut e1 = Edge::new(
            Curve3d::Circle(circle_a),
            (0.0, std::f64::consts::TAU),
        );
        e1.start_vertex_point = Some(Point3d::new(0.0, 0.0, 0.0));
        e1.end_vertex_point = Some(Point3d::new(0.0, 0.0, 0.0));
        let mut e2 = Edge::new(
            Curve3d::Circle(circle_b),
            (0.0, std::f64::consts::TAU),
        );
        e2.start_vertex_point = Some(Point3d::new(0.0, 0.0, 0.0));
        e2.end_vertex_point = Some(Point3d::new(0.0, 0.0, 0.0));
        face.edges = vec![e1, e2];

        let mut solid = solid_of(vec![face]);
        let report = solid.index_edges();
        assert_eq!(report.unique_edges, 2, "distinct circles must not merge");
        assert_eq!(report.geometric_dedup, 0);
    }

    #[test]
    fn test_geometric_dedup_excludes_curveless() {
        // Edges with curve == None never participate in geometric dedup —
        // endpoints alone cannot establish identity.
        let mut face = Face::new_surface_only(Surface::Plane(Plane::xy()));
        let mk = |from: Point3d, to: Point3d| {
            let mut e = Edge {
                curve: None,
                ..line_edge(from, to, None)
            };
            e.start_vertex_point = Some(from);
            e.end_vertex_point = Some(to);
            e
        };
        let a = mk(Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0));
        let b = mk(Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0));
        face.edges = vec![a, b];

        let mut solid = solid_of(vec![face]);
        let report = solid.index_edges();
        assert_eq!(report.unique_edges, 2);
        assert_eq!(report.geometric_dedup, 0);
    }

    #[test]
    fn test_geom_key_line_parametrization_invariant() {
        // Same geometric line, different parametrization origins and
        // opposite directions → identical keys.
        let mut e1 = Edge::new(
            Curve3d::Line(draper_geometry::Line::new(
                Point3d::new(0.0, 0.0, 0.0),
                draper_geometry::Direction3d::X,
            )),
            (0.0, 1.0),
        );
        e1.start_vertex_point = Some(Point3d::new(0.0, 0.0, 0.0));
        e1.end_vertex_point = Some(Point3d::new(1.0, 0.0, 0.0));
        let mut e2 = Edge::new(
            Curve3d::Line(draper_geometry::Line::new(
                Point3d::new(5.0, 0.0, 0.0),
                draper_geometry::Direction3d::NEG_X,
            )),
            (0.0, 1.0),
        );
        e2.start_vertex_point = Some(Point3d::new(1.0, 0.0, 0.0));
        e2.end_vertex_point = Some(Point3d::new(0.0, 0.0, 0.0));

        assert_eq!(geom_edge_key(&e1), geom_edge_key(&e2));
    }

    #[test]
    fn test_propagate_edge_fixes_shared_step_id() {
        // Two faces share EDGE_CURVE #77. Healing marked one copy degenerate
        // and bumped its tolerance; the twin in the other face is stale.
        let mut face_a = square_face(&[Some(77)]);
        let mut face_b = square_face(&[Some(77)]);
        face_a.edges[0].degenerate = true;
        face_a.edges[0].tolerance = 1e-3;
        face_b.edges[0].tolerance = 1e-6;

        let mut solid = solid_of(vec![face_a, face_b]);
        let updated = solid.propagate_edge_fixes();

        assert!(updated >= 2, "degenerate + tolerance must propagate");
        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        assert!(fb.edges[0].degenerate, "degenerate flag must propagate");
        assert!((fa.edges[0].tolerance - 1e-3).abs() < 1e-12);
        assert!((fb.edges[0].tolerance - 1e-3).abs() < 1e-12, "MAX tolerance wins");
        // Orientation-dependent fields must stay untouched.
        assert_eq!(fa.edges[0].param_range, fb.edges[0].param_range);
    }

    #[test]
    fn test_propagate_edge_fixes_curve_backfill_range_guard() {
        // A curve-less twin gets the donor curve ONLY when its param_range
        // matches the donor's (or is its swap); a mismatching range means
        // a different parametrization → no backfill.
        let mut face_a = square_face(&[None]);
        let mut face_b = square_face(&[None]);
        // face_a's copy keeps the curve and donates it; face_b's copy loses
        // its curve. Identity comes from the shared step id (a curve-less
        // edge has no geometric key of its own).
        face_a.edges[0].step_entity_id = Some(88);
        face_b.edges[0].step_entity_id = Some(88);
        let donor_range = face_a.edges[0].param_range;
        face_b.edges[0].curve = None;
        face_b.edges[0].param_range = (7.0, 9.0); // mismatching range

        let mut solid = solid_of(vec![face_a.clone(), face_b.clone()]);
        let updated = solid.propagate_edge_fixes();
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        assert!(
            fb.edges[0].curve.is_none(),
            "mismatching param_range must NOT receive a blind backfill"
        );
        assert_eq!(updated, 0);

        // Now with a matching (swapped) range the backfill fires.
        let mut solid = solid_of(vec![face_a, face_b]);
        {
            let fb = solid.outer_shell.as_mut().unwrap().faces.get_mut(1).unwrap();
            fb.edges[0].param_range = (donor_range.1, donor_range.0); // swap = OK
        }
        let updated = solid.propagate_edge_fixes();
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        assert!(fb.edges[0].curve.is_some(), "swapped range may take the donor curve");
        assert!(updated >= 1);
    }

    #[test]
    fn test_propagate_edge_fixes_geometric_group() {
        // Native (no step id) shared edges group by geometric key too.
        let mut face_a = square_face(&[None]);
        let mut face_b = square_face(&[None]);
        face_b.edges[0].tolerance = 0.5;

        let mut solid = solid_of(vec![face_a, face_b]);
        solid.propagate_edge_fixes();
        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        assert!((fa.edges[0].tolerance - 0.5).abs() < 1e-12);
    }

    #[test]
    fn test_ensure_edge_store_idempotent() {
        let face = square_face(&[Some(1)]);
        let mut solid = solid_of(vec![face]);
        solid.ensure_edge_store();
        assert_eq!(solid.edge_store.len(), 1);
        // Second call must not duplicate anything.
        solid.ensure_edge_store();
        assert_eq!(solid.edge_store.len(), 1);
    }

    #[test]
    fn test_upgrade_canonical_copy_with_curve() {
        // First instance has no curve, second does — canonical copy upgrades.
        let face = square_face(&[None]);
        let mut solid = solid_of(vec![face]);

        let no_curve = Edge {
            curve: None,
            ..line_edge(
                Point3d::new(9.0, 0.0, 0.0),
                Point3d::new(10.0, 0.0, 0.0),
                Some(300),
            )
        };
        let with_curve = line_edge(
            Point3d::new(11.0, 0.0, 0.0),
            Point3d::new(12.0, 0.0, 0.0),
            Some(300),
        );
        let id_no_curve = no_curve.id;
        solid.outer_shell.as_mut().unwrap().faces[0].edges = vec![no_curve, with_curve];
        solid.index_edges();

        let canonical = solid.edge_store.get(id_no_curve).unwrap();
        assert!(canonical.curve.is_some(), "canonical copy must carry curve data");
    }

    #[test]
    fn test_canonical_edge_ids_fallback() {
        // Un-indexed face: canonical_edge_ids falls back to instance ids.
        let face = square_face(&[Some(1), Some(2)]);
        let ids = face.canonical_edge_ids();
        assert_eq!(ids.len(), 2);
        assert_eq!(ids[0], face.edges[0].id);
        assert_eq!(ids[1], face.edges[1].id);
    }

    #[test]
    fn test_wire_coedge_ids_unchanged_by_indexing() {
        // Regression guard: index_edges must NOT rewrite coedge.edge refs —
        // per-face lookups (face.edges.find(|e| e.id == coedge.edge)) must
        // keep finding their instance.
        let mut face = square_face(&[Some(500)]);
        let e0 = face.edges[0].id;
        let coedge = CoEdge::new(e0, true);
        let wire = Wire::new(vec![coedge]);
        face.outer_wire = Some(wire);

        let mut solid = solid_of(vec![face]);
        solid.index_edges();

        let f = &solid.outer_shell.as_ref().unwrap().faces[0];
        let ce = f.outer_wire.as_ref().unwrap().coedges[0].edge;
        assert_eq!(ce, e0, "coedge refs must stay untouched in Stage 2");
        assert!(f.edges.iter().any(|e| e.id == ce));
    }

    // ── C5 Stage 4: store-first read API + mirror sync ──────────────────

    #[test]
    fn test_resolve_edge_store_first_mirror_fallback() {
        // Two faces sharing one STEP edge (same step_entity_id).
        let face_a = square_face(&[Some(10)]);
        let face_b = square_face(&[Some(10)]);
        let mut solid = solid_of(vec![face_a, face_b]);
        solid.index_edges();

        let instance_a = solid.outer_shell.as_ref().unwrap().faces[0].edges[0].id;
        let instance_b = solid.outer_shell.as_ref().unwrap().faces[1].edges[0].id;
        assert_ne!(instance_a, instance_b, "fixture: two instance copies");

        // Store path: BOTH instances resolve to the same canonical edge.
        let ra = solid.resolve_edge(instance_a).unwrap();
        let rb = solid.resolve_edge(instance_b).unwrap();
        assert!(std::ptr::eq(ra, rb), "shared instances resolve identically");

        // Mirror fallback path: a fresh (un-indexed) solid still answers.
        let solid_raw = solid_of(vec![square_face(&[Some(10)])]);
        let raw_instance = solid_raw
            .outer_shell
            .as_ref()
            .unwrap()
            .faces[0]
            .edges[0]
            .id;
        assert!(solid_raw
            .resolve_edge(raw_instance)
            .is_some(), "un-indexed solid falls back to mirror scan");
        assert!(solid_raw.resolve_edge(TopoId::new()).is_none());
    }

    #[test]
    fn test_face_edges_yields_canonical_shared_edge() {
        let face_a = square_face(&[Some(20)]);
        let face_b = square_face(&[Some(20)]);
        let mut solid = solid_of(vec![face_a, face_b]);
        solid.index_edges();

        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let ea = &solid.face_edges(fa)[0];
        let eb = &solid.face_edges(fb)[0];
        assert!(
            std::ptr::eq(*ea, *eb),
            "face_edges must return the canonical edge for both incident faces"
        );
        // Length stays instance-faithful (parallel to face.edges).
        assert_eq!(solid.face_edges(fa).len(), fa.edges.len());
    }

    #[test]
    fn test_sync_edge_mirrors_propagates_canonical_fixes() {
        let face_a = square_face(&[Some(30)]);
        let face_b = square_face(&[Some(30)]);
        let mut solid = solid_of(vec![face_a, face_b]);
        solid.index_edges();

        let instance_b = solid.outer_shell.as_ref().unwrap().faces[1].edges[0].id;
        let range_b = solid.outer_shell.as_ref().unwrap().faces[1].edges[0].param_range;

        // The C5 Stage 4 mutation flow: fix the canonical edge ONCE...
        {
            let canonical = solid.edge_store.get_mut(instance_b).unwrap();
            canonical.tolerance = 1e-2;
            canonical.degenerate = true;
        }
        // ...then sync every mirror.
        let updated = solid.sync_edge_mirrors();
        assert!(updated >= 3, "tolerance+degenerate on two mirrors, counted per field");

        for f in &solid.outer_shell.as_ref().unwrap().faces {
            assert!((f.edges[0].tolerance - 1e-2).abs() < 1e-12);
            assert!(f.edges[0].degenerate);
        }

        // Idempotent: nothing left to do.
        assert_eq!(solid.sync_edge_mirrors(), 0);

        // Orientation-dependent fields untouched.
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        assert_eq!(fb.edges[0].param_range, range_b);
        assert_eq!(fb.edges[0].id, instance_b, "instance id must never be rewritten");
    }

    #[test]
    fn test_sync_edge_mirrors_curve_range_guard() {
        let mut face_a = square_face(&[None]);
        let mut face_b = square_face(&[None]);
        // Shared identity via step id; face_b's copy is curve-less with a
        // mismatching param_range.
        face_a.edges[0].step_entity_id = Some(99);
        face_b.edges[0].step_entity_id = Some(99);
        face_b.edges[0].curve = None;
        face_b.edges[0].param_range = (5.0, 6.0);

        let mut solid = solid_of(vec![face_a, face_b]);
        solid.index_edges();

        solid.sync_edge_mirrors();
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        assert!(
            fb.edges[0].curve.is_none(),
            "sync must respect the param_range guard — no blind curve backfill"
        );
    }

    #[test]
    fn test_sync_edge_mirrors_noop_without_store() {
        let mut solid = solid_of(vec![square_face(&[Some(40)])]);
        // Store empty (never indexed) → sync is a safe no-op.
        assert_eq!(solid.sync_edge_mirrors(), 0);
    }

    #[test]
    fn test_face_edge_by_id_helpers() {
        let mut face = square_face(&[Some(50), Some(51)]);
        let id0 = face.edges[0].id;
        assert!(face.edge_by_id(id0).is_some());
        assert!(face.edge_by_id(TopoId::new()).is_none());
        face.edge_by_id_mut(id0).unwrap().tolerance = 1e-4;
        assert!((face.edges[0].tolerance - 1e-4).abs() < 1e-12);
    }

    // ============================================================
    // C5 Stage 5.1 — serde: store round-trip + legacy mirror loading
    // ============================================================

    /// Solid with one shared STEP edge (two instances, one `step_entity_id`).
    fn shared_edge_solid() -> Solid {
        // face A: edges [shared(70), a1(71)]
        let mut face_a = square_face(&[Some(70), Some(71)]);
        // face B: edges [b0(80), shared(70)] — different instance id,
        // same STEP entity id as face A's first edge.
        let mut edge_b_shared = line_edge(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Some(70),
        );
        edge_b_shared.id = TopoId::new();
        let mut b0 = line_edge(
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(2.0, 0.0, 0.0),
            Some(80),
        );
        b0.id = TopoId::new();
        let mut face_b = square_face(&[]);
        face_b.edges = vec![b0, edge_b_shared];
        solid_of(vec![face_a, face_b])
    }

    #[cfg(feature = "serde")]
    #[test]
    fn test_serde_roundtrip_preserves_store_identity() {
        let mut solid = shared_edge_solid();
        let report = solid.index_edges();
        assert_eq!(report.deduplicated, 1, "one shared STEP edge expected");

        let json = serde_json::to_string(&solid).expect("serialize solid");
        assert!(
            json.contains("\"edge_store\""),
            "Stage 5.1: the store must be serialized"
        );
        let loaded: Solid = serde_json::from_str(&json).expect("deserialize solid");

        // Store survives the round-trip with identical semantics.
        let (instance, canonical) = loaded
            .edge_store
            .iter_aliases()
            .next()
            .expect("round-trip must preserve aliases");
        assert_ne!(instance, canonical);
        // The alias and its canonical target resolve to the SAME edge.
        let via_alias = loaded.edge_store.get(instance);
        let via_canonical = loaded.edge_store.get_canonical(canonical);
        assert!(
            via_alias.is_some() && via_canonical.is_some(),
            "both alias and canonical lookups must resolve after round-trip"
        );
        assert!(
            std::ptr::eq(via_alias.unwrap(), via_canonical.unwrap()),
            "alias lookup must land on the canonical edge"
        );

        // The solid-level read API resolves the shared edge identically.
        let shared = loaded
            .edge_store
            .find_by_step_id(70)
            .expect("by_step_id index survives round-trip");
        let shared_id = shared.id;
        assert_eq!(
            loaded.edge_store.get(shared_id).map(|e| e.id),
            Some(shared_id),
            "shared id resolves to itself"
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn test_serde_legacy_payload_rebuilds_store() {
        let mut solid = shared_edge_solid();
        solid.index_edges();

        // Emulate a LEGACY payload: written before Stage 5, when
        // `edge_store` was `#[serde(skip)]` — the field is absent.
        let mut value: serde_json::Value =
            serde_json::to_value(&solid).expect("serialize to value");
        let obj = value.as_object_mut().expect("solid serializes to an object");
        obj.remove("edge_store");

        let mut loaded: Solid =
            serde_json::from_value(value).expect("legacy payload must deserialize");
        assert!(
            loaded.edge_store.is_empty(),
            "legacy payload has no store — default empty"
        );

        // Legacy loading path: rebuild from mirrors on demand.
        loaded.ensure_edge_store();
        let report = loaded.index_edges();
        assert_eq!(report.deduplicated, 1, "rebuild re-detects the shared edge");
        assert!(
            loaded.edge_store.find_by_step_id(70).is_some(),
            "rebuilt store indexes STEP ids"
        );
    }
}
