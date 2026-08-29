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
//! Stage 3+ will migrate consumers to store lookups and finally remove the
//! per-face `Vec<Edge>` mirrors entirely.

use crate::entity::{Edge, Face, Shell, Solid, TopoId};
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
}

/// Canonical registry of B-Rep edges.
///
/// See the module documentation for the architecture. The store is cheap to
/// rebuild (`Solid::index_edges`) and is `#[serde(skip)]`-ped on `Solid` —
/// serialization keeps the per-face mirrors, deserialization rebuilds the
/// store on demand via `Solid::ensure_edge_store`.
#[derive(Clone, Debug, Default)]
pub struct EdgeStore {
    /// Canonical edges, keyed by canonical TopoId.
    edges: HashMap<TopoId, Edge>,
    /// Instance TopoId → canonical TopoId (identity mappings are omitted).
    aliases: HashMap<TopoId, TopoId>,
    /// STEP entity id → canonical TopoId.
    by_step_id: HashMap<i64, TopoId>,
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
        // Native edges (no step_entity_id) are all canonical — no dedup yet
        // (geometric dedup is Stage 3).
        let face_a = square_face(&[None, None]);
        let face_b = square_face(&[None, None]);
        let mut solid = solid_of(vec![face_a, face_b]);
        let report = solid.index_edges();

        assert_eq!(report.total_instances, 4);
        assert_eq!(report.unique_edges, 4);
        assert_eq!(report.deduplicated, 0);
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
}
