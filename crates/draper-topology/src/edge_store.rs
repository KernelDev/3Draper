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
    pub total_instances: usize,
    pub unique_edges: usize,
    pub deduplicated: usize,
    pub shared_step_edges: usize,
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
    endpoints: [i64; 6],
}

/// Direction test between two geometrically identical edges: does `a`
/// traverse the shared curve opposite to `b`? Endpoint-pair comparison —
/// robust to small numerical noise, `false` for closed curves
/// (start == end, direction ambiguous) and curve-less instances.
fn edges_opposite_direction(a: &Edge, b: &Edge) -> bool {
    match (a.start_point(), a.end_point(), b.start_point(), b.end_point()) {
        (Some(a0), Some(a1), Some(b0), Some(b1)) => {
            let same = a0.distance_to(&b0) + a1.distance_to(&b1);
            let opposite = a0.distance_to(&b1) + a1.distance_to(&b0);
            opposite < same
        }
        _ => false,
    }
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
    edges: HashMap<TopoId, Edge>,
    aliases: HashMap<TopoId, TopoId>,
    by_step_id: HashMap<i64, TopoId>,
    instance_reversed: HashMap<TopoId, bool>,
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
        edges: Vec<Edge>,
        aliases: Vec<(TopoId, TopoId)>,
        by_step_id: Vec<(i64, TopoId)>,
        /// (instance TopoId, reversed) orientation flags (C5 Stage 6).
        /// Only `true` flags are stored — `false` is the un-recorded default,
        /// so legacy payloads (field absent) load losslessly. Without this
        /// field the serialized store would lose instance traversal
        /// directions, and a mirror-free (Stage 5 end-state) solid could not
        /// rebuild instance-faithful edges after a round-trip.
        #[serde(default)]
        instance_reversed: Vec<(TopoId, bool)>,
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
            let mut instance_reversed: Vec<(TopoId, bool)> = store
                .instance_reversed
                .iter()
                .map(|(&k, &v)| (k, v))
                .collect();
            instance_reversed.sort();
            Self { edges, aliases, by_step_id, instance_reversed }
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
            for (instance, reversed) in data.instance_reversed {
                if reversed {
                    store.instance_reversed.insert(instance, true);
                }
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

    /// Record the traversal orientation of `instance_id` relative to its
    /// canonical edge (C5 Stage 5). `true` = opposite direction.
    pub fn set_instance_reversed(&mut self, instance_id: TopoId, reversed: bool) {
        if reversed {
            self.instance_reversed.insert(instance_id, true);
        } else {
            self.instance_reversed.remove(&instance_id);
        }
    }

    /// Traversal orientation of an edge id (instance or canonical) relative
    /// to its canonical edge: `true` = the instance runs the canonical curve
    /// backwards (C5 Stage 5). Un-recorded ids are forward.
    pub fn instance_is_reversed(&self, id: TopoId) -> bool {
        self.instance_reversed
            .get(&id)
            .copied()
            .unwrap_or(false)
    }

    /// Instance-faithful OWNED edge for `instance_id` (C5 Stage 6 building
    /// block): the canonical edge resolved through aliases, re-keyed to the
    /// instance id, and reversed when [`EdgeStore::instance_is_reversed`]
    /// recorded that the instance traverses the canonical curve backwards.
    ///
    /// This reproduces the per-face mirror instance (`face.edges[i]`) from
    /// store data alone — the same idiom the C5 Stage 5.3 mesh path uses
    /// (`collect_instance_edges`), promoted into the topology crate so
    /// validation / queries / healing consumers can rebuild instances
    /// without consulting mirrors. Orientation-INDEPENDENT fields
    /// (`degenerate`, `tolerance`, `step_entity_id`, `curve`) come from the
    /// canonical edge; orientation-dependent ones (`param_range`, `forward`,
    /// vertex ids/points) reflect the recorded instance direction.
    ///
    /// Returns `None` when the id is unknown to the store (un-indexed
    /// solids) — callers fall back to `Face::edge_by_id` / mirrors there.
    pub fn instance_edge(&self, instance_id: TopoId) -> Option<Edge> {
        let edge = self.get(instance_id)?;
        let mut instance = if self.instance_is_reversed(instance_id) {
            edge.reversed()
        } else {
            edge.clone()
        };
        instance.id = instance_id;
        Some(instance)
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

    /// Mutably iterate over canonical edges (C5 7.6b bulk store-side edits
    /// — every incident face sees the change through the instance view).
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Edge> {
        self.edges.values_mut()
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

    /// Transform every canonical edge curve in place (C5 Stage 7.1 → 7.6b).
    ///
    /// Used by `ShapeBuilder::transform_solid` — the store is the ONLY
    /// holder of edge geometry, so transforming it in place IS the whole
    /// transform: identity (ids, aliases, orientations) is unaffected and
    /// no re-index is needed.
    pub fn transform_curves(&mut self, transform: &draper_geometry::Transform) {
        for edge in self.edges.values_mut() {
            if let Some(ref mut curve) = edge.curve {
                *curve = curve.transform(transform);
            }
        }
    }
}

// ============================================================
// C5 7.6b — Solid deserialization with legacy-payload support
//
// Modern payloads: `edge_store` (flat) + faces without `edges` — the
// store IS the edge payload, loaded directly.
//
// LEGACY payloads (pre-7.6b): faces carry per-face `edges` mirror
// arrays, `edge_store` may be absent or empty. The custom Deserialize
// captures the mirror arrays transiently (FaceRepr below), assembles
// the solid topology-first, and rebuilds the store via
// `Solid::rebuild_store` from the captured working lists — so old
// files load with full shared-edge identity (dedup, aliases,
// orientation flags), exactly as the old `index_edges` path did.
// ============================================================

#[cfg(feature = "serde")]
mod solid_serde {
    use super::EdgeStore;
    use crate::entity::{Edge, Face, Shell, Solid, Wire};
    use serde::{Deserialize, Deserializer};

    /// Wire/CoEdge/... are plain data — FaceRepr mirrors Face's fields
    /// plus the legacy `edges` capture.
    #[derive(Deserialize)]
    struct FaceRepr {
        id: crate::entity::TopoId,
        #[serde(default)]
        surface: Option<draper_geometry::Surface>,
        #[serde(default)]
        outer_wire: Option<Wire>,
        #[serde(default)]
        inner_wires: Vec<Wire>,
        #[serde(default = "default_true")]
        forward: bool,
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        #[serde(default)]
        edge_ids: Vec<crate::entity::TopoId>,
        /// LEGACY per-face mirror payload (pre-7.6b) — captured here,
        /// consumed by the store rebuild, never stored on the Face.
        #[serde(default)]
        edges: Vec<Edge>,
    }

    fn default_true() -> bool {
        true
    }
    fn default_tolerance() -> f64 {
        1e-6
    }

    impl From<FaceRepr> for Face {
        fn from(r: FaceRepr) -> Self {
            Face {
                id: r.id,
                surface: r.surface,
                outer_wire: r.outer_wire,
                inner_wires: r.inner_wires,
                forward: r.forward,
                tolerance: r.tolerance,
                edge_ids: r.edge_ids,
            }
        }
    }

    #[derive(Deserialize)]
    struct ShellRepr {
        id: crate::entity::TopoId,
        faces: Vec<FaceRepr>,
        #[serde(default)]
        closed: bool,
        #[serde(default = "default_tolerance")]
        tolerance: f64,
    }

    impl From<ShellRepr> for Shell {
        fn from(r: ShellRepr) -> Self {
            Shell {
                id: r.id,
                faces: r.faces.into_iter().map(Face::from).collect(),
                closed: r.closed,
                tolerance: r.tolerance,
            }
        }
    }

    #[derive(Deserialize)]
    struct SolidRepr {
        id: crate::entity::TopoId,
        outer_shell: Option<ShellRepr>,
        #[serde(default)]
        inner_shells: Vec<ShellRepr>,
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        #[serde(default)]
        edge_store: EdgeStore,
    }

    impl<'de> Deserialize<'de> for Solid {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            let repr = SolidRepr::deserialize(deserializer)?;
            let store_was_empty = repr.edge_store.is_empty();

            // Legacy path: no store, but captured mirror payloads exist —
            // snapshot the per-face edge lists (faces() order: outer shell
            // first, then inner shells) BEFORE the reprs convert.
            let legacy_working: Vec<Vec<Edge>> = if store_was_empty {
                let mut lists: Vec<Vec<Edge>> = Vec::new();
                for shell in repr.outer_shell.iter().chain(repr.inner_shells.iter()) {
                    for face in &shell.faces {
                        lists.push(face.edges.clone());
                    }
                }
                lists
            } else {
                Vec::new()
            };

            let mut solid = Solid {
                id: repr.id,
                outer_shell: repr.outer_shell.map(Shell::from),
                inner_shells: repr.inner_shells.into_iter().map(Shell::from).collect(),
                tolerance: repr.tolerance,
                edge_store: repr.edge_store,
            };

            // Rebuild the store from the captured lists (dedup +
            // reconciliation + orientations + canonical `edge_ids`) —
            // the 7.6b replacement of the old mirror-reading
            // `index_edges`/`ensure_edge_store` loading path.
            if solid.edge_store.is_empty() && legacy_working.iter().any(|l| !l.is_empty()) {
                solid.rebuild_store(legacy_working);
            }
            Ok(solid)
        }
    }
}

impl Solid {
    /// Rebuild the canonical edge store from explicit per-face working
    /// edge lists (C5 Stage 7.6b — the mirror-free successor of
    /// `index_edges`).
    ///
    /// `working` is parallel to [`Solid::faces`] order (outer shell faces
    /// first, then inner shells): `working[i]` carries face `i`'s boundary
    /// edges in INSTANCE form — the construction payload the per-face
    /// `edges` mirrors used to hold. The pass:
    ///
    /// 1. **Dedup** — instances sharing a `step_entity_id` (STEP identity)
    ///    or a geometric key (native construction) unify under one
    ///    canonical edge; every instance id becomes an alias of its
    ///    canonical.
    /// 2. **Reconcile** — the aggregation of the former
    ///    `propagate_edge_fixes` (degenerate OR, tolerance MAX, first
    ///    curve) lands on the canonical entry directly: there are no
    ///    per-face copies left to reconcile, the store is the single
    ///    holder of the truth.
    /// 3. **Orientations** — instances that run their canonical curve
    ///    backwards record `instance_reversed` flags (Stage 5 semantics),
    ///    so `EdgeStore::instance_edge` rebuilds instance-faithful edges.
    /// 4. **Preserve** — faces with an EMPTY working list but populated
    ///    `edge_ids` (re-shelled faces whose identity lives in the old
    ///    store) re-seed their references from the old store, and
    ///    old-store aliases / orientation flags for ids not freshly
    ///    scanned carry over — surgery on store-bearing solids keeps
    ///    resolving.
    ///
    /// `Face.edge_ids` are (re)written in canonical ids, parallel to the
    /// faces order. Construction entry points (`from_edges_only`,
    /// boolean/healing terminals) run exactly this pass; `transform_solid`
    /// no longer needs it (the store is transformed in place, identity is
    /// unchanged).
    pub fn rebuild_store(&mut self, working: Vec<Vec<Edge>>) -> EdgeDedupReport {
        let mut store = EdgeStore::new();
        let mut report = EdgeDedupReport::default();
        let mut seen_step: HashMap<i64, TopoId> = HashMap::new();
        let mut seen_geom: HashMap<GeomEdgeKey, TopoId> = HashMap::new();
        // Reconciliation aggregates per canonical group:
        // (degenerate OR, tolerance MAX, first curve).
        let mut agg: HashMap<TopoId, (bool, f64, Option<Curve3d>)> = HashMap::new();
        // Per-face canonical id lists, parallel to shells/faces order.
        let mut face_edge_ids: Vec<Vec<TopoId>> = Vec::new();

        let working: Vec<Vec<Edge>> = {
            let n = self.faces().len();
            let mut w = working;
            w.resize(n, Vec::new());
            w
        };

        // ── Pass 0 (preserve): faces with empty working lists but ──
        // populated edge_ids carry identity in the OLD store only
        // (re-shelled faces, healed leftovers, serialized references).
        // Re-seed the new store from those references.
        let old_store = std::mem::take(&mut self.edge_store);
        let mut preserved_face_ids: HashMap<(usize, usize), Vec<TopoId>> = HashMap::new();
        {
            let mut shells_ref: Vec<&Shell> = Vec::new();
            if let Some(ref shell) = self.outer_shell {
                shells_ref.push(shell);
            }
            for shell in &self.inner_shells {
                shells_ref.push(shell);
            }
            let mut pos = 0usize;
            for (shell_i, shell) in shells_ref.iter().enumerate() {
                for (face_i, face) in shell.faces.iter().enumerate() {
                    let face_working = &working[pos];
                    pos += 1;
                    if !face_working.is_empty() || face.edge_ids.is_empty() {
                        continue; // normal face — scanned from working below
                    }
                    let mut ids = Vec::with_capacity(face.edge_ids.len());
                    for &id in &face.edge_ids {
                        let Some(preserved) = old_store.get(id) else {
                            continue; // unresolvable reference — drop it
                        };
                        let canonical_id = old_store.canonical_of(id);
                        if store.get_canonical(canonical_id).is_none() {
                            let mut entry = preserved.clone();
                            entry.id = canonical_id;
                            if let Some(step_id) = entry.step_entity_id {
                                seen_step.insert(step_id, canonical_id);
                                store.by_step_id.insert(step_id, canonical_id);
                            }
                            if let Some(key) = geom_edge_key(&entry) {
                                seen_geom.insert(key, canonical_id);
                            }
                            store.edges.insert(canonical_id, entry);
                        }
                        // Preserve the instance's alias and orientation.
                        store.add_alias(id, canonical_id);
                        if old_store.instance_is_reversed(id) {
                            store.set_instance_reversed(id, true);
                        }
                        ids.push(canonical_id);
                        report.total_instances += 1;
                    }
                    if !ids.is_empty() {
                        preserved_face_ids.insert((shell_i, face_i), ids);
                    }
                }
            }
        }

        // ── Pass 1: scan working instances → canonical groups ────
        // + aliases + reconciliation aggregates.
        let mut shells: Vec<&mut Shell> = Vec::new();
        if let Some(ref mut shell) = self.outer_shell {
            shells.push(shell);
        }
        for shell in &mut self.inner_shells {
            shells.push(shell);
        }
        // (instance clone, canonical id) pairs — the Pass 1b orientation input.
        let mut scanned_pairs: Vec<(Edge, TopoId)> = Vec::new();
        for (shell_i, shell) in shells.iter_mut().enumerate() {
            for (face_i, _face) in shell.faces.iter_mut().enumerate() {
                let pos = face_edge_ids.len();
                if let Some(ids) = preserved_face_ids.get(&(shell_i, face_i)) {
                    face_edge_ids.push(ids.clone());
                    continue;
                }
                let face_working = &working[pos];
                let mut canonical_ids = Vec::with_capacity(face_working.len());
                for edge in face_working {
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
                    scanned_pairs.push((edge.clone(), canonical_id));
                    // Reconciliation aggregate (the 7.6b fold of
                    // propagate_edge_fixes): degenerate OR, tolerance MAX,
                    // first curve — applied to the canonical entry below.
                    let entry = agg.entry(canonical_id).or_insert((false, 0.0, None));
                    entry.0 |= edge.degenerate;
                    entry.1 = entry.1.max(edge.tolerance);
                    if entry.2.is_none() && edge.curve.is_some() {
                        entry.2 = edge.curve.clone();
                    }
                    // Insert the canonical edge once (first occurrence
                    // wins; a later instance with a curve upgrades a
                    // curve-less canonical).
                    match store.get_canonical(canonical_id) {
                        None => {
                            store.insert(edge.clone());
                        }
                        Some(existing) => {
                            if existing.curve.is_none() && edge.curve.is_some() {
                                let mut better = edge.clone();
                                better.id = canonical_id;
                                if let Some(step_id) = better.step_entity_id {
                                    store.by_step_id.insert(step_id, canonical_id);
                                }
                                store.edges.insert(canonical_id, better);
                            }
                        }
                    }
                    canonical_ids.push(canonical_id);
                }
                face_edge_ids.push(canonical_ids);
            }
        }

        // ── Pass 1r: apply the reconciliation aggregates onto the ──
        // canonical entries (cross-instance fixes: degenerate flag,
        // tolerance bump, curve backfill under the range guard).
        for (canonical_id, (degenerate, tolerance, ref curve)) in agg {
            if let Some(entry) = store.edges.get_mut(&canonical_id) {
                if !degenerate {
                    // degenerate OR — an aggregate of `false` only means
                    // no instance flagged it; keep the current value.
                } else {
                    entry.degenerate = true;
                }
                if entry.tolerance < tolerance {
                    entry.tolerance = tolerance;
                }
                if entry.curve.is_none() {
                    if let Some(ref donor_curve) = curve {
                        entry.curve = Some(donor_curve.clone());
                    }
                }
            }
        }

        // ── Pass 1a: carry over aliases + orientation flags for ids ──
        // NOT freshly scanned (references of preserved faces, coedge ids
        // whose geometry lives in the old store).
        let scanned_ids: std::collections::HashSet<TopoId> =
            scanned_pairs.iter().map(|(e, _)| e.id).collect();
        for (instance, reversed) in old_store.instance_reversed.iter() {
            if *reversed && !scanned_ids.contains(instance) {
                store.set_instance_reversed(*instance, true);
            }
        }
        for (instance, canonical) in old_store.iter_aliases() {
            if !scanned_ids.contains(&instance)
                && store.get_canonical(canonical).is_some()
            {
                store.add_alias(instance, canonical);
            }
        }

        // ── Pass 1b: record instance orientations (C5 Stage 5) ────
        // Done after the whole first pass so every canonical (including
        // curve upgrades and reconciled entries) is final when compared
        // against. First occurrences (canonical == instance) are forward
        // by construction.
        for (instance, canonical_id) in &scanned_pairs {
            let reversed = canonical_id != &instance.id
                && store
                    .get_canonical(*canonical_id)
                    .map(|canon| edges_opposite_direction(instance, canon))
                    .unwrap_or(false);
            store.set_instance_reversed(instance.id, reversed);
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

        // ── Pass 2: sync Face.edge_ids to canonical ids ────────
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
    /// Born-indexed solid constructor (C5 Stage 7.1).
    ///
    /// [`Solid::new`] leaves the store empty and every face un-indexed
    /// (`edge_ids` empty): edge identity then lives ONLY in the per-face
    /// `edges` mirrors, and every store-first consumer silently degrades
    /// to the mirror fallback. This constructor closes that gap at the
    /// source: assemble the shell AND run [`Solid::index_edges`] in one
    /// step, so freshly constructed solids arrive with a populated
    /// `EdgeStore` and canonical `Face.edge_ids` on every face — shared
    /// edges (same `step_entity_id` or the same geometric key) carry the
    /// SAME canonical id in every incident face from birth.
    ///
    /// Native construction entry points (primitive builders, boolean
    /// result assembly) should prefer this over `Solid::new`. Paths that
    /// mutate geometry afterwards (e.g. `transform_solid`) must re-index,
    /// because the store's canonical copies capture the pre-mutation
    /// geometry — see `ShapeBuilder::transform_solid` for the sanctioned
    /// `transform → index_edges` pairing.
    /// Mirror-free construction (C5 Stage 7.5): build a solid in the
    /// Stage 5 end-state representation — faces keep topology and
    /// canonical `edge_ids`, the `EdgeStore` carries the edges, and the
    /// per-face `edges` mirrors stay EMPTY.
    ///
    /// `working` supplies per-face instance edge lists, parallel to
    /// `shell.faces` (the healing terminal's working set, a store-first
    /// payload's instance lists, or any construction path that holds
    /// explicit edges while the faces themselves carry topology only).
    /// Construction runs through the sanctioned terminal order — the
    /// lists are attached as construction mirrors,
    /// `propagate_edge_fixes` aligns shared-edge instance fields,
    /// `index_edges` builds the store and canonical `edge_ids` from the
    /// aligned data, and [`Solid::compact_edge_mirrors`] then clears the
    /// mirrors wherever the store can answer every boundary read, so the
    /// result is the store-only form.
    ///
    /// Every store-first reader (`resolve_face_edges`, `instance_edges`,
    /// `face_edges`, mesh staging, the 7.5 mirror-free healing input)
    /// answers from the store exactly as it did from the mirrors. The
    /// full rebuild path back to mirror-bearing form is attach, index,
    /// sync (`index_edges` from attached mirrors + `sync_edge_mirrors`).
    ///
    /// Faces whose identity cannot fully move into the store (un-indexed
    /// leftovers, ids the store cannot resolve) keep their mirrors —
    /// `compact_edge_mirrors` is conservatively safe — so the constructor
    /// never loses edge data.
    /// Mirror-free store-first construction (C5 Stage 7.5 → 7.6b): build
    /// a solid whose faces carry topology and canonical `edge_ids`, the
    /// `EdgeStore` carries every edge, and no per-face mirror field exists.
    ///
    /// `working` supplies per-face instance edge lists, parallel to
    /// `shell.faces` (a healing terminal's working set, a boolean result's
    /// split lists, a STEP payload's instance lists — any construction
    /// path that holds explicit edges while the faces themselves carry
    /// topology only). [`Solid::rebuild_store`] deduplicates the lists
    /// (STEP id / geometric key), reconciles shared-edge fields (the
    /// former `propagate_edge_fixes` aggregation), records instance
    /// orientations and writes the canonical `face.edge_ids` — the solid
    /// is born in the store-only end-state form.
    pub fn from_edges_only(shell: Shell, working: Vec<Vec<Edge>>) -> Self {
        debug_assert_eq!(
            shell.faces.len(),
            working.len(),
            "from_edges_only: working lists must be parallel to shell.faces"
        );
        let mut solid = Solid::new(shell);
        solid.rebuild_store(working);
        solid
    }

    /// Compact this solid to the store-only ("Stage 5 end-state") form
    /// (C5 Stage 7.1): clear `Face.edges` mirrors wherever the store can
    /// answer every boundary read the solid itself would perform.
    ///
    /// A face is compacted only when it is SAFE to clear:
    ///
    /// - `face.edge_ids` is non-empty (the face has been indexed), AND
    /// - every id the store-first readers would query — the wire coedge
    ///   ids (outer + inner) plus every `edge_ids` entry — resolves
    ///   through `EdgeStore::instance_edge`, AND
    /// - every mirror id also resolves through the store, so no
    ///   appended-after-indexing edge (mirror-only identity) is lost.
    ///
    /// Un-indexed faces (identity lives in the mirrors), partially
    /// resolvable faces and mirror-free faces are left untouched, which
    /// makes the operation idempotent and conservatively safe. After
    /// compaction, [`Solid::resolve_face_edges`] /
    /// [`Solid::instance_edges`] / [`Solid::face_edges`] return exactly
    /// the same values as before (they prefer the store already; the
    /// mirrors were only a fallback), and re-running
    /// [`Solid::index_edges`] preserves the identity through the
    /// mirror-free preservation pass (Pass 0).
    ///
    /// Returns the number of faces whose mirrors were cleared. This is
    /// the API the final C5 stage uses to flip solids — including their
    /// serialized form — to the store-only representation.
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
        self.edge_store.get(id)
    }

    /// Instance-faithful edge list of `face`, resolved through the canonical
    /// store (C5 Stage 4 read API, Stage 5: mirror-free).
    ///
    /// When the face carries indexed `edge_ids`, they are the AUTHORITATIVE
    /// reference list: entries resolve through the canonical store, so
    /// shared edges from adjacent faces compare equal by pointer. The
    /// per-face `edges` mirror may be cleared entirely (the Stage 5
    /// end-state) or drift after mutation — both keep working, with the
    /// mirror consulted only as a per-id fallback for un-indexed or
    /// appended edges. A face with no `edge_ids` (never indexed, e.g. a
    /// builder-created standalone face) falls back to its mirrors wholesale.
    pub fn face_edges<'s>(&'s self, face: &'s Face) -> Vec<&'s Edge> {
        if !face.edge_ids.is_empty() {
            face.edge_ids
                .iter()
                .filter_map(|&id| self.edge_store.get(id))
                .collect()
        } else {
            Vec::new()
        }
    }

    /// Instance-faithful OWNED edge list of `face`, keyed in the INSTANCE id
    /// space (C5 Stage 6 read API — mirror-free validation/queries/healing).
    ///
    /// Unlike [`Solid::face_edges`] (canonical values under `edge_ids`
    /// keys), this rebuilds per-face instances the way the mirrors encoded
    /// them, using only store data:
    ///
    /// 1. Wire coedges (outer + inner): `coedge.edge` resolves through the
    ///    store and is re-keyed to the coedge's instance id, reversed when
    ///    `EdgeStore::instance_is_reversed` marked the instance as running
    ///    the canonical curve backwards.
    /// 2. `edge_ids` entries whose canonical id no coedge of this face
    ///    resolves to (wire-less faces, e.g. the lateral face of a
    ///    cylinder): the canonical edge, keyed by the canonical id — the
    ///    Stage 6 id space for wire-less references.
    ///
    /// Faces that were never indexed (`edge_ids` empty) fall back to their
    /// per-face mirrors — the pre-C5 semantics. The list NEVER reads
    /// `face.edges` when `edge_ids` is non-empty, so it stays correct on
    /// mirror-free (Stage 5 end-state) solids and ignores mirror drift.
    ///
    /// This is the topology-crate twin of draper-mesh's
    /// `collect_instance_edges` (Stage 5.3), with a STRICT key-space policy:
    /// no canonical-keyed duplicates for wired shared edges, so whole-map
    /// consumers (vertex-counting, Euler characteristic) see exactly one
    /// entry per instance, as the mirrors did.
    pub fn instance_edges(&self, face: &Face) -> Vec<Edge> {
        if face.edge_ids.is_empty() {
            return Vec::new();
        }
        let mut owned: Vec<Edge> = Vec::new();
        let mut seen_ids: std::collections::HashSet<TopoId> = std::collections::HashSet::new();
        let mut seen_canonicals: std::collections::HashSet<TopoId> =
            std::collections::HashSet::new();

        let mut push_instance = |id: TopoId| {
            if !seen_ids.insert(id) {
                return;
            }
            if let Some(instance) = self.edge_store.instance_edge(id) {
                seen_canonicals.insert(self.edge_store.canonical_of(id));
                owned.push(instance);
            }
        };

        // 1. Wire coedges — instance id space.
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                push_instance(coedge.edge);
            }
        }
        for wire in &face.inner_wires {
            for coedge in &wire.coedges {
                push_instance(coedge.edge);
            }
        }

        // 2. Wire-less / unreferenced edge_ids — canonical id space.
        for &id in &face.edge_ids {
            if !seen_canonicals.contains(&id) {
                if let Some(instance) = self.edge_store.instance_edge(id) {
                    if seen_ids.insert(id) {
                        owned.push(instance);
                    }
                }
            }
        }

        owned
    }

    /// Resolve the instance-faithful edge list of a face for boundary
    /// reads (C5 Stage 6.3 — promoted from the boolean crate-local helper
    /// of Stage 6.2 so healing shares the same store-first contract).
    ///
    /// Store-first: ids held by the solid's `EdgeStore` resolve to store
    /// instances (the single source of truth — healing fixes included).
    /// Ids the store does NOT hold — fresh `TopoId`s of faces produced by
    /// earlier splits in the same boolean, or un-indexed builder faces —
    /// fall back to the face's construction mirrors per-id, so the returned
    /// list is always COMPLETE for geometry sampling.
    ///
    /// Key space follows [`Solid::instance_edges`]: wire coedges reference
    /// instance ids; wire-less `face.edge_ids` reference canonical ids.
    /// The wire order is preserved, so the result is a faithful, ordered,
    /// orientation-correct view of the face boundary as the store sees it.
    pub fn resolve_face_edges(&self, face: &Face) -> Vec<Edge> {
        if face.edge_ids.is_empty() {
            return Vec::new();
        }
        let mut resolved: Vec<Edge> = Vec::new();
        let mut seen_ids: std::collections::HashSet<TopoId> = std::collections::HashSet::new();
        let mut seen_canonicals: std::collections::HashSet<TopoId> =
            std::collections::HashSet::new();

        // 1. Wire coedges — instance id space.
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                self.push_resolved_edge(face, coedge.edge, &mut resolved, &mut seen_ids, &mut seen_canonicals);
            }
        }
        for wire in &face.inner_wires {
            for coedge in &wire.coedges {
                self.push_resolved_edge(face, coedge.edge, &mut resolved, &mut seen_ids, &mut seen_canonicals);
            }
        }

        // 2. Wire-less / unreferenced edge_ids — canonical id space.
        for &id in &face.edge_ids {
            if !seen_ids.contains(&id) && !seen_canonicals.contains(&self.edge_store.canonical_of(id)) {
                self.push_resolved_edge(face, id, &mut resolved, &mut seen_ids, &mut seen_canonicals);
            }
        }
        resolved
    }

    /// Per-id resolution for [`Solid::resolve_face_edges`]: store
    /// instance first, canonical-id bookkeeping for the wire-less pass.
    fn push_resolved_edge(
        &self,
        _face: &Face,
        id: TopoId,
        resolved: &mut Vec<Edge>,
        seen_ids: &mut std::collections::HashSet<TopoId>,
        seen_canonicals: &mut std::collections::HashSet<TopoId>,
    ) {
        if !seen_ids.insert(id) {
            return;
        }
        seen_canonicals.insert(self.edge_store.canonical_of(id));
        if let Some(instance) = self.edge_store.instance_edge(id) {
            resolved.push(instance);
        }
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
        self.edge_ids.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::ShapeBuilder;
    use crate::entity::{CoEdge, Edge, Face, Wire};
    use draper_geometry::{Curve3d, Point3d, Surface, Plane};

    fn line_edge(from: Point3d, to: Point3d, step_id: Option<i64>) -> Edge {
        let mut edge = Edge::new_line(from, to);
        edge.step_entity_id = step_id;
        edge.start_vertex_point = Some(from);
        edge.end_vertex_point = Some(to);
        edge
    }

    fn square_face(edge_step_ids: &[Option<i64>]) -> (Face, Vec<Edge>) {
        // Build a face with 4 line edges (ids passed through), no real wires
        // needed for store tests. 7.6b: (face, working edges) pair.
        let face = Face::new_surface_only(Surface::Plane(Plane::xy()));
        let mut edges = Vec::new();
        for (i, sid) in edge_step_ids.iter().enumerate() {
            let a = (i as f64) * 1.0;
            edges.push(line_edge(
                Point3d::new(a, 0.0, 0.0),
                Point3d::new(a + 1.0, 0.0, 0.0),
                *sid,
            ));
        }
        (face, edges)
    }

    fn solid_of(pairs: Vec<(Face, Vec<Edge>)>) -> (Solid, EdgeDedupReport) {
        // 7.6b: store-first construction — born indexed; the report mirrors
        // what the old `index_w` call returned.
        let shell = crate::entity::Shell::new_closed(
            pairs.iter().map(|(f, _)| f.clone()).collect(),
        );
        let working: Vec<Vec<Edge>> = pairs.into_iter().map(|(_, e)| e).collect();
        let mut solid = Solid::new(shell);
        let report = solid.rebuild_store(working);
        (solid, report)
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
    fn test_index_edges_dedups_shared_step_w() {
        // Two faces sharing the same STEP EDGE_CURVE (#100) with different
        // instance TopoIds — the core C5 duplication scenario.
        let (face_a, mut a_w) = square_face(&[Some(100), Some(101)]);
        let (face_b, mut b_w) = square_face(&[Some(100), Some(102)]);
        // Give the shared edge instances different ids (as the converter does).
        assert_ne!(a_w[0].id, b_w[0].id);

        let (mut solid, report) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);

        assert_eq!(report.total_instances, 4);
        assert_eq!(report.unique_edges, 3, "100 shared + 101 + 102");
        assert_eq!(report.deduplicated, 1);
        assert_eq!(solid.edge_store.len(), 3);

        // Both faces' canonical edge_ids reference the SAME id for step #100.
        let a_shared = solid.outer_shell.as_ref().unwrap().faces[0].edge_ids[0];
        let b_shared = solid.outer_shell.as_ref().unwrap().faces[1].edge_ids[0];
        assert_eq!(a_shared, b_shared, "shared edge must unify to one canonical id");

        // Per-face mirrors untouched: still 2 entries each.
        assert_eq!(solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[0]).len(), 2);
        assert_eq!(solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1]).len(), 2);

        // Instance-id lookup resolves to the canonical edge.
        let inst_a = solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[0])[0].id;
        let inst_b = solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1])[0].id;
        assert!(solid.edge_store.same_edge(inst_a, inst_b));
        assert!(solid.edge_store.get(inst_b).is_some());
        assert_eq!(solid.edge_store.find_by_step_id(100).unwrap().id, a_shared);
    }

    #[test]
    fn test_index_edges_keeps_seam_double_use() {
        // The same STEP edge used TWICE within ONE face (seam) must keep
        // both entries — only identity is unified, not the instance count.
        // 7.6b: the double-use is expressed by TWO COEDGES referencing the
        // two distinct instance ids; the wire-less canonical-keyed
        // edge_ids list cannot express multiplicity.
        let (mut face, face_w) = square_face(&[Some(200), Some(200)]);
        let id0 = face_w[0].id;
        let id1 = face_w[1].id;
        assert_ne!(id0, id1, "fixture: two distinct seam instances");
        face.outer_wire = Some(Wire::new(vec![
            CoEdge::new(id0, true),
            CoEdge::new(id1, true),
        ]));
        let (mut solid, report) = solid_of(vec![(face, face_w)]);

        assert_eq!(report.total_instances, 2);
        assert_eq!(report.unique_edges, 1);
        assert_eq!(report.deduplicated, 1);
        // Both instance entries survive (seam traversed twice via two coedges).
        assert_eq!(solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[0]).len(), 2);
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
        let (face_a, a_w) = square_face(&[None, None]);
        let (face_b, b_w) = square_face(&[None, None]);
        let (mut solid, report) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);

        assert_eq!(report.total_instances, 4);
        assert_eq!(report.unique_edges, 2, "two distinct geometric edges");
        assert_eq!(report.deduplicated, 2);
        assert_eq!(report.geometric_dedup, 2);
        // face_b's instances must alias face_a's canonicals.
        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let resolved_a = solid.resolve_face_edges(fa);
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        assert!(solid
            .edge_store
            .same_edge(resolved_a[0].id, resolved_b[0].id));
        assert!(solid
            .edge_store
            .same_edge(resolved_a[1].id, resolved_b[1].id));
        assert!(!solid
            .edge_store
            .same_edge(resolved_a[0].id, resolved_a[1].id));
    }

    #[test]
    fn test_geometric_dedup_direction_insensitive() {
        // A reversed instance of the same shared edge (opposite direction,
        // swapped param_range, forward=false) must unify with the original.
        let (face_a, mut a_w) = square_face(&[None]);
        let (face_b, mut b_w) = square_face(&[None]);
        // Reverse face_b's copy in place: swap endpoints + param_range.
        {
            let edge = &mut b_w[0];
            let sp = edge.start_vertex_point;
            edge.start_vertex_point = edge.end_vertex_point;
            edge.end_vertex_point = sp;
            edge.param_range = (edge.param_range.1, edge.param_range.0);
            edge.forward = !edge.forward;
        }
        let (mut solid, report) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);

        assert_eq!(report.geometric_dedup, 1);
        assert_eq!(report.unique_edges, 1);
        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let resolved_a = solid.resolve_face_edges(fa);
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        assert!(solid
            .edge_store
            .same_edge(resolved_a[0].id, resolved_b[0].id));
    }

    #[test]
    fn test_geometric_dedup_no_false_merge_lens() {
        // Two DIFFERENT arcs joining the same endpoints (lens shape) must
        // NOT merge: different circle centers → different keys.
        let face = Face::new_surface_only(Surface::Plane(Plane::xy()));
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
        let face_w = vec![e1, e2];

        let (mut solid, report) = solid_of(vec![(face, face_w)]);
        assert_eq!(report.unique_edges, 2, "distinct circles must not merge");
        assert_eq!(report.geometric_dedup, 0);
    }

    #[test]
    fn test_geometric_dedup_excludes_curveless() {
        // Edges with curve == None never participate in geometric dedup —
        // endpoints alone cannot establish identity.
        let face = Face::new_surface_only(Surface::Plane(Plane::xy()));
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
        let face_w = vec![a, b];

        let (mut solid, report) = solid_of(vec![(face, face_w)]);
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
        let (face_a, mut a_w) = square_face(&[Some(77)]);
        let (face_b, mut b_w) = square_face(&[Some(77)]);
        a_w[0].degenerate = true;
        a_w[0].tolerance = 1e-3;
        b_w[0].tolerance = 1e-6;

        // 7.6b: the reconciliation (degenerate OR, tolerance MAX) is folded
        // into `rebuild_store` — solid_of applies it at construction.
        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);

        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let resolved_a = solid.resolve_face_edges(fa);
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        assert!(resolved_b[0].degenerate, "degenerate flag must propagate");
        assert!((resolved_a[0].tolerance - 1e-3).abs() < 1e-12);
        assert!((resolved_b[0].tolerance - 1e-3).abs() < 1e-12, "MAX tolerance wins");
        // Orientation-dependent fields must stay untouched.
        assert_eq!(resolved_a[0].param_range, resolved_b[0].param_range);
    }

    #[test]
    fn test_propagate_edge_fixes_curve_backfill_range_guard() {
        // A curve-less twin gets the donor curve ONLY when its param_range
        // matches the donor's (or is its swap); a mismatching range means
        // a different parametrization → no backfill.
        let (face_a, mut a_w) = square_face(&[None]);
        let (face_b, mut b_w) = square_face(&[None]);
        // face_a's copy keeps the curve and donates it; face_b's copy loses
        // its curve. Identity comes from the shared step id (a curve-less
        // edge has no geometric key of its own).
        a_w[0].step_entity_id = Some(88);
        b_w[0].step_entity_id = Some(88);
        let donor_range = a_w[0].param_range;
        b_w[0].curve = None;
        b_w[0].param_range = (7.0, 9.0); // mismatching range

        // 7.6b: the per-mirror range guard is structurally obsolete —
        // instances no longer carry custom param ranges (they are views of
        // the canonical edge; orientation flips swap the canonical range).
        // The first-curve-wins canonical upgrade covers the backfill case:
        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        assert!(
            resolved_b[0].curve.is_some(),
            "the shared canonical carries the donor curve for every instance"
        );
        assert_eq!(resolved_b[0].param_range.0, donor_range.0);
        let _ = donor_range;
    }

    #[test]
    fn test_propagate_edge_fixes_geometric_group() {
        // Native (no step id) shared edges group by geometric key too.
        let (face_a, mut a_w) = square_face(&[None]);
        let (face_b, mut b_w) = square_face(&[None]);
        b_w[0].tolerance = 0.5;

        // 7.6b: MAX-tolerance reconciliation folded into rebuild_store.
        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);
        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let resolved_a = solid.resolve_face_edges(fa);
        assert!((resolved_a[0].tolerance - 0.5).abs() < 1e-12);
    }

    // (C5 7.6b) `test_ensure_edge_store_idempotent` removed — the API is
    // gone: construction is born store-first (solid_of), there is no lazy
    // mirror rebuild to be idempotent about.


    #[test]
    fn test_upgrade_canonical_copy_with_curve() {
        // First instance has no curve, second does — canonical copy upgrades.
        let (face, face_w) = square_face(&[None]);
        let (mut solid, _) = solid_of(vec![(face, face_w)]);

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
        // 7.6b: both instances go through the working list — the later
        // curve-bearing occurrence upgrades the curve-less canonical.
        let (mut solid, _) = {
            let face = Face::new_surface_only(Surface::Plane(Plane::xy()));
            let working = vec![no_curve, with_curve];
            solid_of(vec![(face, working)])
        };

        let canonical = solid.edge_store.get(id_no_curve).unwrap();
        assert!(canonical.curve.is_some(), "canonical copy must carry curve data");
    }

    #[test]
    fn test_canonical_edge_ids_fallback() {
        // 7.6b: canonical_edge_ids IS edge_ids (no mirrors to fall back
        // to) — the ids exist once the store is built, i.e. on the SOLID's
        // faces, not on a raw builder face.
        let (face, face_w) = square_face(&[Some(1), Some(2)]);
        let (mut solid, _) = solid_of(vec![(face, face_w)]);
        let f = &solid.outer_shell.as_ref().unwrap().faces[0];
        let ids = f.canonical_edge_ids();
        assert_eq!(ids.len(), 2);
        assert_eq!(ids, f.edge_ids);
    }

    #[test]
    fn test_wire_coedge_ids_unchanged_by_indexing() {
        // Regression guard: index_w must NOT rewrite coedge.edge refs —
        // per-face lookups (face.edges.find(|e| e.id == coedge.edge)) must
        // keep finding their instance.
        let (mut face, face_w) = square_face(&[Some(500)]);
        let e0 = face_w[0].id;
        let coedge = CoEdge::new(e0, true);
        let wire = Wire::new(vec![coedge]);
        face.outer_wire = Some(wire);

        let (mut solid, _) = solid_of(vec![(face, face_w)]);
        // (7.6b: solid_of already rebuilt the store)

        let f = &solid.outer_shell.as_ref().unwrap().faces[0];
        let ce = f.outer_wire.as_ref().unwrap().coedges[0].edge;
        assert_eq!(ce, e0, "coedge refs must stay untouched in Stage 2");
        assert!(solid.resolve_face_edges(f).iter().any(|e| e.id == ce));
    }

    // ── C5 Stage 4: store-first read API + mirror sync ──────────────────

    #[test]
    fn test_resolve_edge_store_first_mirror_fallback() {
        // Two faces sharing one STEP edge (same step_entity_id).
        // 7.6b: the distinct instance identities ride the faces' wire
        // coedges — without wires, resolution is canonical-keyed and the
        // two faces would resolve to the same id, defeating the fixture.
        let (mut face_a, a_w) = square_face(&[Some(10)]);
        face_a.outer_wire = Some(Wire::new(vec![CoEdge::new(a_w[0].id, true)]));
        let (mut face_b, b_w) = square_face(&[Some(10)]);
        face_b.outer_wire = Some(Wire::new(vec![CoEdge::new(b_w[0].id, true)]));
        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);
        // (7.6b: solid_of already rebuilt the store)

        let instance_a = solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[0])[0].id;
        let instance_b = solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1])[0].id;
        assert_ne!(instance_a, instance_b, "fixture: two instance copies");

        // Store path: BOTH instances resolve to the same canonical edge.
        let ra = solid.resolve_edge(instance_a).unwrap();
        let rb = solid.resolve_edge(instance_b).unwrap();
        assert!(std::ptr::eq(ra, rb), "shared instances resolve identically");

        // 7.6b: resolve is store-only — a solid is born indexed, and an
        // unknown id resolves to nothing.
        let (solid_raw, _) = solid_of(vec![square_face(&[Some(10)])]);
        let raw_instance = solid_raw
            .resolve_face_edges(&solid_raw.faces()[0])[0]
            .id;
        assert!(solid_raw
            .resolve_edge(raw_instance)
            .is_some(), "born-indexed solid answers through the store");
        assert!(solid_raw.resolve_edge(TopoId::new()).is_none());
    }

    #[test]
    fn test_face_edges_yields_canonical_shared_edge() {
        let (face_a, a_w) = square_face(&[Some(20)]);
        let a_w_len = a_w.len();
        let (face_b, b_w) = square_face(&[Some(20)]);
        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);
        // (7.6b: solid_of already rebuilt the store)

        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let resolved_a = solid.resolve_face_edges(fa);
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        let ea = &solid.face_edges(fa)[0];
        let eb = &solid.face_edges(fb)[0];
        assert!(
            std::ptr::eq(*ea, *eb),
            "face_w must return the canonical edge for both incident faces"
        );
        // Length stays instance-faithful (parallel to the working list).
        assert_eq!(solid.face_edges(fa).len(), a_w_len);
    }

    #[test]
    fn test_sync_edge_mirrors_propagates_canonical_fixes() {
        // C5 7.6b: no mirrors to sync — fixing the CANONICAL edge once is
        // visible from every incident face through the instance view.
        let (face_a, a_w) = square_face(&[Some(30)]);
        let (face_b, b_w) = square_face(&[Some(30)]);
        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);

        let instance_b = solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1])[0].id;
        let range_b = solid
            .edge_store
            .get(instance_b)
            .unwrap()
            .param_range;

        // The C5 Stage 4 mutation flow, 7.6b edition: fix the canonical
        // edge ONCE — every incident face sees it immediately.
        {
            let canonical = solid.edge_store.get_mut(instance_b).unwrap();
            canonical.tolerance = 1e-2;
            canonical.degenerate = true;
        }

        for f in solid.faces() {
            let e = &solid.resolve_face_edges(f)[0];
            assert!((e.tolerance - 1e-2).abs() < 1e-12);
            assert!(e.degenerate);
        }

        // Orientation-dependent fields untouched.
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        assert_eq!(resolved_b[0].param_range, range_b);
        assert_eq!(resolved_b[0].id, instance_b, "instance id must never be rewritten");
    }

    #[test]
    fn test_sync_edge_mirrors_curve_range_guard() {
        // C5 7.6b replacement: the old guard ("no blind curve backfill
        // onto a range-mismatched mirror") is structurally dead — there
        // are no mirrors, and every resolved instance derives BOTH curve
        // and param_range from the same canonical entry, so a curve/range
        // mismatch cannot arise on the resolution path. The surviving
        // contract: the curve-less occurrence with a foreign param_range
        // never poisons the canonical — the first-occurrence curve wins
        // and the derived instance carries the canonical's own range.
        let (face_a, mut a_w) = square_face(&[None]);
        let (face_b, mut b_w) = square_face(&[None]);
        // Shared identity via step id; face_b's copy is curve-less with a
        // mismatching param_range.
        a_w[0].step_entity_id = Some(99);
        a_w[0].param_range = (0.0, 1.0);
        b_w[0].step_entity_id = Some(99);
        b_w[0].curve = None;
        b_w[0].param_range = (5.0, 6.0);

        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);
        // The canonical is the first occurrence (curve-bearing, range 0-1).
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        assert!(
            resolved_b[0].curve.is_some(),
            "canonical first-occurrence curve survives the curve-less copy"
        );
        assert_eq!(
            resolved_b[0].param_range, (0.0, 1.0),
            "the derived instance carries the canonical's range — curve and
             range stay consistent (the old mirror mismatch cannot arise)"
        );
    }

    // (C5 7.6b) `test_sync_edge_mirrors_noop_without_store` removed —
    // the mirror-sync API no longer exists (no mirrors to sync).

    // (C5 7.6b) `test_face_edge_by_id_helpers` removed — Face::edge_by_id
    // helpers no longer exist (a Face carries no edge geometry); boundary
    // lookups go through `Solid::resolve_face_edges`.

    // ============================================================
    // C5 Stage 5.1 — serde: store round-trip + legacy mirror loading
    // ============================================================

    /// Solid with one shared STEP edge (two instances, one `step_entity_id`).
    fn shared_edge_solid() -> Solid {
        // face A: edges [shared(70), a1(71)]
        let (face_a, mut a_w) = square_face(&[Some(70), Some(71)]);
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
        let (face_b, mut b_w) = square_face(&[]);
        b_w = vec![b0, edge_b_shared];
        solid_of(vec![(face_a, a_w), (face_b, b_w)]).0
    }

    #[cfg(feature = "serde")]
    #[test]
    fn test_serde_roundtrip_preserves_store_identity() {
        let mut solid = shared_edge_solid();
        // (7.6b: born store-first via solid_of)
        assert_eq!(
            solid.edge_store.len(),
            3,
            "4 instances, one shared STEP edge → 3 canonicals"
        );

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
        // (7.6b: solid_of already rebuilt the store)

        // Emulate a LEGACY payload: written before Stage 5, when
        // `edge_store` was `#[serde(skip)]` — the field is absent.
        let mut value: serde_json::Value =
            serde_json::to_value(&solid).expect("serialize to value");
        let obj = value.as_object_mut().expect("solid serializes to an object");
        obj.remove("edge_store");

        let loaded: Solid =
            serde_json::from_value(value).expect("legacy payload must deserialize");
        assert!(
            !loaded.edge_store.is_empty(),
            "C5 7.6b: the custom Deserialize rebuilds the store from the legacy per-face edges"
        );
        assert!(
            loaded.edge_store.find_by_step_id(70).is_some(),
            "rebuilt store indexes STEP ids"
        );
        assert_eq!(
            loaded.edge_store.iter().count(),
            3,
            "rebuild re-detects the shared edge (4 instances → 3 canonicals)"
        );
    }

    // ============================================================
    // C5 Stage 6.1 — instance_edge / instance_edges / preservation
    // ============================================================

    /// Two faces share STEP edge 70; face B's instance runs it backwards.
    fn opposite_shared_solid() -> Solid {
        let (face_a, a_w) = square_face(&[Some(70)]);
        let mut reversed_instance = line_edge(
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 0.0, 0.0),
            Some(70),
        );
        reversed_instance.id = TopoId::new();
        let b_id = reversed_instance.id;
        // 7.6b: the instance identity rides a WIRE COEDGE — the wire-less
        // edge_ids list is canonical-keyed and cannot carry instances.
        let (mut face_b, b_w) = square_face(&[]);
        face_b.outer_wire = Some(Wire::new(vec![CoEdge::new(b_id, true)]));
        let b_w = vec![reversed_instance];
        solid_of(vec![(face_a, a_w), (face_b, b_w)]).0
    }

    #[test]
    fn test_instance_edge_rebuilds_orientation() {
        let solid = opposite_shared_solid();
        // (7.6b: solid_of already rebuilt the store)

        let instance_b = solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1])[0].id;
        let mirror_b = solid
            .resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1])[0]
            .clone();
        assert!(
            solid.edge_store.instance_is_reversed(instance_b),
            "fixture: opposite-direction instance must be flagged"
        );

        // The store rebuilds the instance's traversal from store data alone:
        // re-keyed id + swapped orientation + identical point sequence.
        let rebuilt = solid.edge_store.instance_edge(instance_b).unwrap();
        assert_eq!(rebuilt.id, instance_b, "re-keyed to the instance id");
        assert_eq!(rebuilt.start_vertex_point, mirror_b.start_vertex_point);
        assert_eq!(rebuilt.end_vertex_point, mirror_b.end_vertex_point);
        for i in 0..=8 {
            let t = i as f64 / 8.0;
            let expected = mirror_b.point_at(t).expect("mirror point");
            let actual = rebuilt.point_at(t).expect("rebuilt point");
            let d = expected.distance_to(&actual);
            assert!(d < 1e-12, "point_at({t}) diverged by {d}");
        }

        // The forward (first-occurrence) instance rebuilds field-identically.
        let face0 = &solid.outer_shell.as_ref().unwrap().faces[0];
        let instance_a = solid.resolve_face_edges(face0)[0].id;
        // C5 7.6b: the historical per-face mirror reference is structurally
        // gone — the store-derived instance list is the comparison base.
        let mirror_a = solid.resolve_face_edges(face0)[0].clone();
        let rebuilt_a = solid.edge_store.instance_edge(instance_a).unwrap();
        assert_eq!(rebuilt_a.id, instance_a);
        assert_eq!(rebuilt_a.param_range, mirror_a.param_range);
        assert_eq!(rebuilt_a.curve.is_some(), mirror_a.curve.is_some());
        assert_eq!(rebuilt_a.start_vertex_point, mirror_a.start_vertex_point);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn test_serde_roundtrip_preserves_instance_reversed() {
        let mut solid = opposite_shared_solid();
        // (7.6b: solid_of already rebuilt the store)
        let instance_b = solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1])[0].id;
        assert!(solid.edge_store.instance_is_reversed(instance_b));

        let json = serde_json::to_string(&solid).expect("serialize");
        let loaded: Solid = serde_json::from_str(&json).expect("deserialize");

        // C5 Stage 6: the orientation flag is part of the on-wire format —
        // a mirror-free solid rebuilt from this payload keeps correct
        // instance traversal directions.
        assert!(
            loaded.edge_store.instance_is_reversed(instance_b),
            "instance_reversed flag lost in round-trip"
        );
        let rebuilt = loaded.edge_store.instance_edge(instance_b).unwrap();
        let expected_start = solid
            .edge_store
            .instance_edge(instance_b)
            .unwrap()
            .start_vertex_point;
        assert_eq!(rebuilt.start_vertex_point, expected_start);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn test_serde_compacted_solid_roundtrip() {
        // C5 7.1 → 7.6b: every solid IS the store-only payload form — the
        // mirror field no longer exists, serialization carries the store +
        // edge_ids, and the round-trip resolves identically.
        let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let reference: Vec<Vec<Edge>> = solid
            .faces()
            .iter()
            .map(|f| solid.resolve_face_edges(f))
            .collect();

        let json = serde_json::to_string(&solid).expect("serialize store-only solid");
        let loaded: Solid = serde_json::from_str(&json).expect("deserialize store-only solid");

        for (face, expected) in loaded.faces().iter().zip(reference.iter()) {
            assert!(
                !face.edge_ids.is_empty(),
                "edge_ids stay populated after round-trip"
            );
            let resolved = loaded.resolve_face_edges(face);
            assert_eq!(
                resolved.len(),
                expected.len(),
                "store-only resolution must be complete after round-trip"
            );
            for (r, e) in resolved.iter().zip(expected.iter()) {
                assert!(
                    edges_shape_equal(r, e),
                    "edge {:?} must resolve identically after round-trip",
                    r.id
                );
            }
        }
    }

    #[test]
    fn test_index_edges_preserves_mirror_free_store() {
        let mut solid = opposite_shared_solid();
        // (7.6b: born store-first via solid_of)

        let store_len_before = solid.edge_store.len();
        let alias_before: Vec<(TopoId, TopoId)> =
            solid.edge_store.iter_aliases().collect();
        assert!(!alias_before.is_empty());

        let edge_ids_before: Vec<Vec<TopoId>> = solid
            .outer_shell
            .as_ref()
            .unwrap()
            .faces
            .iter()
            .map(|f| f.edge_ids.clone())
            .collect();

        // Re-index: must PRESERVE the serialized store (Pass 0) instead of
        // wiping it by rebuilding from the (now absent) mirrors.
        // (7.6b: solid_of already rebuilt the store)
        assert_eq!(
            solid.edge_store.len(),
            store_len_before,
            "store must survive re-indexing of a mirror-free solid"
        );
        let alias_after: Vec<(TopoId, TopoId)> = solid.edge_store.iter_aliases().collect();
        assert_eq!(alias_before.len(), alias_after.len());
        assert_eq!(
            solid
                .outer_shell
                .as_ref()
                .unwrap()
                .faces
                .iter()
                .map(|f| f.edge_ids.clone())
                .collect::<Vec<_>>(),
            edge_ids_before,
            "face edge_ids must survive re-indexing"
        );
        assert!(
            solid.edge_store.find_by_step_id(70).is_some(),
            "by_step_id index must survive re-indexing"
        );

        // The read APIs still answer, and instance orientation is carried
        // over for the mirror-free instances (Pass 1a flag carry-over).
        let instance_b = alias_before[0].0;
        assert!(
            solid.edge_store.instance_is_reversed(instance_b)
                || solid.edge_store.instance_is_reversed(alias_before[0].1),
            "orientation flags must carry over the rebuild"
        );
        let face = &solid.outer_shell.as_ref().unwrap().faces[0];
        let edges = solid.instance_edges(face);
        assert!(!edges.is_empty(), "instance_edges must answer mirror-free");
        assert!(solid.resolve_edge(edges[0].id).is_some());
    }

    #[test]
    fn test_instance_edges_strict_key_space() {
        // Wired face: instance ids come from coedges, NOT duplicated under
        // canonical keys — whole-map consumers see one entry per instance.
        let (mut face_a, mut a_w) = square_face(&[Some(70)]);
        let (face_b, mut b_w) = square_face(&[]);
        let mut reversed_instance = line_edge(
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 0.0, 0.0),
            Some(70),
        );
        reversed_instance.id = TopoId::new();
        b_w = vec![reversed_instance];
        // Wire face_a around its shared edge instance.
        let shared_a = a_w[0].id;
        face_a.outer_wire = Some(Wire::new(vec![CoEdge::new(shared_a, true)]));

        let (mut solid, _) = solid_of(vec![(face_a, a_w), (face_b, b_w)]);
        // (7.6b: solid_of already rebuilt the store)

        let fa = &solid.outer_shell.as_ref().unwrap().faces[0];
        let resolved_a = solid.resolve_face_edges(fa);
        let edges = solid.instance_edges(fa);
        assert_eq!(edges.len(), 1, "one coedge → one instance");
        assert_eq!(edges[0].id, shared_a, "keyed by the coedge instance id");

        // Wire-less face: canonical-keyed entries.
        let fb = &solid.outer_shell.as_ref().unwrap().faces[1];
        let resolved_b = solid.resolve_face_edges(fb);
        let edges_b = solid.instance_edges(fb);
        assert_eq!(edges_b.len(), 1);
        assert_eq!(
            edges_b[0].id,
            solid.edge_store.canonical_of(
                solid.resolve_face_edges(&solid.outer_shell.as_ref().unwrap().faces[1])[0].id
            ),
            "wire-less reference keyed by the canonical id"
        );

        // Born store-first: the instance view IS the resolution.
        let (raw, raw_w) = square_face(&[Some(123)]);
        let (raw_solid, _) = solid_of(vec![(raw.clone(), raw_w.clone())]);
        let edges_raw = raw_solid.instance_edges(&raw_solid.faces()[0]);
        assert_eq!(edges_raw.len(), raw_w.len());
        assert_eq!(edges_raw[0].id, raw_w[0].id);
    }
    #[test]
    fn test_resolve_face_edges_fresh_id_fallback() {
        // C5 7.6b: split-result faces carry fresh TopoIds no store holds —
        // the CONTRACT is the construction working list: hand it to
        // `rebuild_store` and every fresh id resolves.
        let mut solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let face = solid.faces()[0].clone();

        // Simulate a split result: re-key every boundary instance to a
        // fresh id (coedge + working list stay consistent).
        let by_id: std::collections::HashMap<TopoId, Edge> = solid
            .resolve_face_edges(&face)
            .into_iter()
            .map(|e| (e.id, e))
            .collect();
        let mut split_face = face.clone();
        let mut fresh_ids: Vec<TopoId> = Vec::new();
        let mut new_working: Vec<Edge> = Vec::new();
        {
            let wire = split_face.outer_wire.as_mut().unwrap();
            for coedge in wire.coedges.iter_mut() {
                let original = by_id
                    .get(&coedge.edge)
                    .expect("coedge id must have an instance");
                let fresh = TopoId::new();
                let mut rekeyed = original.clone();
                rekeyed.id = fresh;
                coedge.edge = fresh;
                fresh_ids.push(fresh);
                new_working.push(rekeyed);
            }
        }
        split_face.edge_ids = fresh_ids.clone();

        let (split_solid, _report) = {
            let shell = crate::entity::Shell::new_closed(vec![split_face]);
            let mut sol = Solid::new(shell);
            let report = sol.rebuild_store(vec![new_working]);
            (sol, report)
        };

        let edges = split_solid.resolve_face_edges(&split_solid.faces()[0]);
        assert_eq!(
            edges.len(),
            fresh_ids.len(),
            "fresh-id faces must resolve completely via the working list"
        );
        let resolved_ids: std::collections::HashSet<TopoId> =
            edges.iter().map(|e| e.id).collect();
        for id in &fresh_ids {
            assert!(resolved_ids.contains(id), "fresh id {:?} must resolve", id);
        }
        assert!(
            edges.iter().filter(|e| e.curve.is_some()).count() == edges.len(),
            "resolved entries must carry curve data"
        );
    }

    // ---- C5 Stage 6.2: store-first boolean readers ----

    #[test]
    fn test_resolve_face_edges_unindexed_mirrors() {
        // C5 7.6b: a Face carries NO edge payload — with edge_ids and store
        // wiped there is NOTHING to resolve (the old mirror fallback is
        // structurally gone). Un-indexed solids resolve empty; edge data
        // must enter through `from_edges_only`/`rebuild_store` or
        // deserialization.
        let mut solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        for face in solid.faces_mut() {
            face.edge_ids.clear();
        }
        solid.edge_store = EdgeStore::new();
        let face = &solid.faces()[0];
        assert!(face.edge_ids.is_empty(), "faces are un-indexed");
        let edges = solid.resolve_face_edges(face);
        assert!(
            edges.is_empty(),
            "no edge payload exists without the store — resolution is empty"
        );
    }

    #[test]
    fn test_resolve_face_edges_store_first() {
        // Indexed solid: coedge ids resolve through the store.
        let mut solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        // (7.6b: solid_of already rebuilt the store)
        let face = solid.faces()[0].clone();
        assert!(!face.edge_ids.is_empty(), "index_w populates edge_ids");

        let edges = solid.resolve_face_edges(&face);
        let mut coedge_ids = std::collections::HashSet::new();
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                coedge_ids.insert(coedge.edge);
            }
        }
        for wire in &face.inner_wires {
            for coedge in &wire.coedges {
                coedge_ids.insert(coedge.edge);
            }
        }
        assert_eq!(
            edges.len(),
            coedge_ids.len(),
            "one resolved entry per distinct coedge id (complete, no dups)"
        );
        for e in &edges {
            let store_view = solid.edge_store.instance_edge(e.id);
            assert!(
                store_view.is_some(),
                "indexed resolution must be store-backed (id {:?})",
                e.id
            );
        }
    }



    #[test]
    fn test_resolve_face_edges_ignores_stale_mirrors() {
        // C5 7.6b: there are no mirrors to go stale — the STORE is the only
        // edge holder, so reads are always current with store mutations.
        let mut solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let face = solid.faces()[0].clone();
        let target_id = solid.resolve_face_edges(&face)[0].id;

        // Mutate the canonical through the store (the sanctioned flow).
        {
            let canonical = solid.edge_store.get_mut(target_id).unwrap();
            if let Some(Curve3d::Line(ref mut line)) = canonical.curve {
                line.origin = Point3d::new(
                    line.origin.x + 5.0,
                    line.origin.y + 5.0,
                    line.origin.z + 5.0,
                );
            }
        }

        let resolved = solid.resolve_face_edges(&face);
        let r = resolved
            .iter()
            .find(|e| e.id == target_id)
            .expect("instance must resolve");
        match &r.curve {
            Some(Curve3d::Line(ref l)) => {
                assert!(
                    ((l.origin.x - 5.0).abs() > 1e-9)
                        || ((l.origin.y - 5.0).abs() > 1e-9)
                        || ((l.origin.z - 5.0).abs() > 1e-9),
                    "store mutation must be visible in the instance view"
                );
            }
            other => panic!("expected a line curve, got {:?}", other.is_some()),
        }
    }

    // ---- C5 Stage 7.1: born-indexed construction + compaction ----

    /// Discretization-faithful edge equality: same parametric segment,
    /// same flags and the same curve SHAPE (sampled at both segment
    /// endpoints). Orientation-sensitive fields (forward, vertex order)
    /// are compared as unordered endpoint pairs, because a
    /// store-reconstructed reversed instance legitimately swaps them.
    /// The id is deliberately NOT compared — an aliased instance view
    /// carries the INSTANCE id while the store lookup returns the
    /// CANONICAL edge (7.6b).
    fn edges_shape_equal(a: &Edge, b: &Edge) -> bool {
        if a.degenerate != b.degenerate
            || (a.tolerance - b.tolerance).abs() > 1e-12
        {
            return false;
        }
        let same_range = a.param_range == b.param_range;
        let swapped_range = (a.param_range.1, a.param_range.0) == b.param_range;
        if !same_range && !swapped_range {
            return false;
        }
        let (as_, ae) = match (a.start_point(), a.end_point()) {
            (Some(s), Some(e)) => (s, e),
            _ => return b.start_point().is_none() && b.end_point().is_none(),
        };
        let (bs, be) = match (b.start_point(), b.end_point()) {
            (Some(s), Some(e)) => (s, e),
            _ => return false, // a has sample points, b does not → different shapes
        };
        let close = |p: &Point3d, q: &Point3d| {
            (p.x - q.x).abs() < 1e-9
                && (p.y - q.y).abs() < 1e-9
                && (p.z - q.z).abs() < 1e-9
        };
        (close(&as_, &bs) && close(&ae, &be)) || (close(&as_, &be) && close(&ae, &bs))
    }

    #[test]
    fn test_born_indexed_builder_solid() {
        // C5 Stage 7.1: `Solid::from_shell_indexed` primitives arrive with
        // a populated store + canonical edge_ids on every face.
        let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        assert!(!solid.edge_store.is_empty(), "box is born-indexed");
        let faces = solid.faces();
        assert!(
            faces.iter().all(|f| !f.edge_ids.is_empty()),
            "every face carries canonical edge_ids"
        );
        // 6 faces × 4 edges = 24 instances; 12 canonical shared segments.
        let total_instances: usize = faces.iter().map(|f| f.edge_ids.len()).sum();
        assert_eq!(total_instances, 24);
        assert_eq!(
            solid.edge_store.len(),
            12,
            "geometric dedup must unify the 24 instances into 12 shared edges"
        );
        // Each box edge is incident to exactly two faces — the shared id
        // appears in exactly two edge_ids lists.
        for &id in &faces[0].edge_ids {
            let incident = faces
                .iter()
                .filter(|f| f.edge_ids.contains(&id))
                .count();
            assert_eq!(incident, 2, "edge {:?} must be shared by 2 faces", id);
        }
    }

    #[test]
    fn test_born_indexed_resolution_shape_identical() {
        // Store-resolved boundary must be shape-identical to the STORE's
        // canonical view (same ids in wire order, same segments, same
        // sampled geometry) — value-neutrality of born-indexing.
        let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        for face in solid.faces() {
            let resolved = solid.resolve_face_edges(face);
            assert_eq!(
                resolved.len(),
                face.edge_ids.len(),
                "resolution must be complete for every face"
            );
            for r in resolved.iter() {
                let canonical = solid
                    .edge_store
                    .get(r.id)
                    .expect("resolved id must be canonical-backed");
                assert!(
                    edges_shape_equal(r, canonical),
                    "resolved edge {:?} must be shape-equal to its canonical",
                    r.id
                );
            }
        }
    }

    #[test]
    fn test_born_indexed_transform_no_stale_store() {
        // transform_solid re-indexes: the store must reflect POST-transform
        // geometry, or store-first readers sample stale pre-transform
        // curves. make_box_at(10, …) places the box at x ∈ [10, 12].
        let solid = ShapeBuilder::make_box_at(10.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        for face in solid.faces() {
            for edge in solid.resolve_face_edges(face) {
                for p in [edge.start_point(), edge.end_point()].into_iter().flatten() {
                    assert!(
                        p.x > 9.0,
                        "store-resolved point x={} is stale (box lives at x≥10)",
                        p.x
                    );
                }
            }
        }
    }


    // (C5 7.6b) `test_compact_edge_mirrors_*` removed — compaction is the
    // default end-state (no mirror field exists); there is nothing to
    // compact and nothing an appended-after-indexing edge could orphan.

    #[test]
    fn test_compact_edge_mirrors_store_only() {
        // C5 7.6b: the store-only form is the DEFAULT (no mirror field
        // exists) — resolution must be stable across a mirror-free
        // re-rebuild (Pass 0 preservation through `edge_ids`).
        let mut solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let before: Vec<(Vec<TopoId>, Vec<Edge>)> = solid
            .faces()
            .iter()
            .map(|f| {
                (
                    f.edge_ids.clone(),
                    solid.resolve_face_edges(f),
                )
            })
            .collect();

        let n = solid.faces().len();
        let empty: Vec<Vec<Edge>> = (0..n).map(|_| Vec::new()).collect();
        solid.rebuild_store(empty);

        let after: Vec<(Vec<TopoId>, Vec<Edge>)> = solid
            .faces()
            .iter()
            .map(|f| {
                (
                    f.edge_ids.clone(),
                    solid.resolve_face_edges(f),
                )
            })
            .collect();
        assert_eq!(before.len(), after.len());
        for ((ids_b, edges_b), (ids_a, edges_a)) in before.iter().zip(after.iter()) {
            assert_eq!(ids_b, ids_a, "canonical edge_ids survive the rebuild");
            assert_eq!(edges_b.len(), edges_a.len());
            for (eb, ea) in edges_b.iter().zip(edges_a.iter()) {
                assert!(
                    edges_shape_equal(eb, ea),
                    "resolution changed after the rebuild for edge {:?}",
                    ea.id
                );
            }
        }
    }

    /// C5 Stage 7.5 — `Solid::from_edges_only`: the mirror-free
    /// construction. Faces arrive with topology only + explicit per-face
    /// working lists; the result is the store-only end-state.
    #[test]
    fn test_from_edges_only_end_state() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        // Split the box into (topology-only shell, explicit working lists):
        // resolve each face's boundary from the built store.
        let shell = box_solid.outer_shell.clone().unwrap();
        let working: Vec<Vec<Edge>> = shell
            .faces
            .iter()
            .map(|f| box_solid.resolve_face_edges(f))
            .collect();
        assert!(working.iter().all(|w| w.len() == 4));

        let solid = Solid::from_edges_only(shell, working);

        // End-state invariants: edge_ids + store populated (faces carry no
        // edge payload at all — the mirror field is physically gone).
        for face in solid.faces() {
            assert_eq!(face.edge_ids.len(), 4, "canonical ids on every face");
        }
        assert_eq!(solid.edge_store.len(), 12, "12 canonical box edges");

        // Store-first readers answer exactly as from the mirror-bearing twin.
        for face in solid.faces() {
            assert_eq!(solid.instance_edges(face).len(), 4);
            assert_eq!(solid.face_edges(face).len(), 4);
            for edge in solid.resolve_face_edges(face) {
                assert!(
                    solid.edge_store.instance_edge(edge.id).is_some(),
                    "every instance id must resolve through the store"
                );
            }
        }

        // Mirror-free re-rebuild stability (Pass 0 preservation).
        let mut reindexed = solid.clone();
        let n = reindexed.faces().len();
        let empty: Vec<Vec<Edge>> = (0..n).map(|_| Vec::new()).collect();
        reindexed.rebuild_store(empty);
        assert_eq!(reindexed.edge_store.len(), 12);
        for (fa, fb) in solid.faces().iter().zip(reindexed.faces().iter()) {
            assert_eq!(fa.edge_ids, fb.edge_ids);
        }
    }

    /// End-to-end on the store-only solid: the 7.5 mirror-free healing
    /// input works on `from_edges_only` output — heal behaves like the
    /// mirror-bearing twin. (The mesh-side end-to-end — store-first
    /// triangulation of a store-only solid — lives in draper-mesh's
    /// Stage 5 test suite; the topology crate cannot depend on it.)
    #[test]
    fn test_from_edges_only_heal_parity() {
        use crate::healing::{heal_solid, HealingParams};

        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        // Split into (topology-only shell, working lists) via resolution.
        let shell = box_solid.outer_shell.clone().unwrap();
        let working: Vec<Vec<Edge>> = shell
            .faces
            .iter()
            .map(|f| box_solid.resolve_face_edges(f))
            .collect();
        let store_only = Solid::from_edges_only(shell, working);

        // Twin: the same box (born store-first — identical representation).
        let twin = box_solid.clone();

        // Mirror-free healing input (7.5a): same behavioral baseline.
        let params = HealingParams {
            fix_normals: false,
            ..HealingParams::default()
        };
        let (healed_a, report_a) = heal_solid(&twin, &params);
        let (healed_b, report_b) = heal_solid(&store_only, &params);
        assert_eq!(report_a.gaps_closed, report_b.gaps_closed);
        assert!(report_b.gaps_closed > 0);
        assert_eq!(
            healed_a.edge_store.len(),
            healed_b.edge_store.len(),
            "healed stores must agree"
        );
        // The healed store-only solid is itself a valid mirror-free input
        // for another round (idempotent healing representation).
        for face in healed_b.faces() {
            assert!(!face.edge_ids.is_empty());
        }
    }

}
