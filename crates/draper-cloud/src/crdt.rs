// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! CRDT (Conflict-free Replicated Data Type) for collaborative B-Rep editing.
//!
//! Per ROADMAP_VISION_2036.md §5 (Phase 5): CRDT for topology enables
//! simultaneous editing of B-Rep models by multiple users without conflicts.
//!
//! Unlike Operational Transform (OT) which requires central coordination,
//! CRDTs guarantee convergence: any two replicas that have received the
//! same set of operations (in any order) will have identical state.
//!
//! This module implements a **last-writer-wins (LWW) map CRDT** for
//! B-Rep topology entities (faces, edges, vertices):
//! - Each entity has a unique ID (Lamport timestamp)
//! - Each modification includes a timestamp → latest wins
//! - Deletions are tombstones (not physical removal)
//! - Merge is commutative and idempotent

use std::collections::HashMap;
use serde::{Deserialize, Serialize};

// ============================================================
// Lamport timestamps (logical clocks)
// ============================================================

/// Lamport timestamp for causal ordering in distributed systems.
///
/// Format: (sequence_number, replica_id).
/// Comparison: (s1, r1) < (s2, r2) iff s1 < s2, or s1 == s2 and r1 < r2.
/// This gives a total order that is consistent across all replicas.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize, PartialOrd, Ord)]
pub struct LamportTimestamp {
    pub sequence: u64,
    pub replica_id: String,
}

impl LamportTimestamp {
    pub fn new(sequence: u64, replica_id: &str) -> Self {
        Self {
            sequence,
            replica_id: replica_id.to_string(),
        }
    }

    /// Create a timestamp that is strictly greater than both inputs.
    /// Used when merging operations from different replicas.
    pub fn merge(&self, other: &Self) -> Self {
        Self {
            sequence: self.sequence.max(other.sequence) + 1,
            replica_id: self.replica_id.clone(),
        }
    }
}

// ============================================================
// CRDT operations on B-Rep entities
// ============================================================

/// A B-Rep entity ID — unique across all replicas.
pub type EntityId = u64;

/// A CRDT operation on a B-Rep entity.
///
/// All operations are commutative: applying them in any order gives
/// the same result. This is the key property that enables conflict-free
/// collaborative editing.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CrdtOp {
    /// Create or update a face with the given geometry.
    PutFace {
        entity_id: EntityId,
        timestamp: LamportTimestamp,
        /// Serialized face geometry (surface + boundary edges).
        geometry: Vec<u8>,
        /// Face label (e.g., "FRONT", "TOP").
        label: Option<String>,
        /// Face color as RGBA.
        color: [f32; 4],
    },

    /// Delete a face (tombstone — not physical removal).
    DeleteFace {
        entity_id: EntityId,
        timestamp: LamportTimestamp,
    },

    /// Create or update an edge.
    PutEdge {
        entity_id: EntityId,
        timestamp: LamportTimestamp,
        geometry: Vec<u8>,
    },

    /// Delete an edge.
    DeleteEdge {
        entity_id: EntityId,
        timestamp: LamportTimestamp,
    },

    /// Create or update a vertex.
    PutVertex {
        entity_id: EntityId,
        timestamp: LamportTimestamp,
        position: [f64; 3],
    },

    /// Delete a vertex.
    DeleteVertex {
        entity_id: EntityId,
        timestamp: LamportTimestamp,
    },
}

impl CrdtOp {
    /// Get the entity ID affected by this operation.
    pub fn entity_id(&self) -> EntityId {
        match self {
            CrdtOp::PutFace { entity_id, .. } => *entity_id,
            CrdtOp::DeleteFace { entity_id, .. } => *entity_id,
            CrdtOp::PutEdge { entity_id, .. } => *entity_id,
            CrdtOp::DeleteEdge { entity_id, .. } => *entity_id,
            CrdtOp::PutVertex { entity_id, .. } => *entity_id,
            CrdtOp::DeleteVertex { entity_id, .. } => *entity_id,
        }
    }

    /// Get the timestamp of this operation.
    pub fn timestamp(&self) -> &LamportTimestamp {
        match self {
            CrdtOp::PutFace { timestamp, .. } => timestamp,
            CrdtOp::DeleteFace { timestamp, .. } => timestamp,
            CrdtOp::PutEdge { timestamp, .. } => timestamp,
            CrdtOp::DeleteEdge { timestamp, .. } => timestamp,
            CrdtOp::PutVertex { timestamp, .. } => timestamp,
            CrdtOp::DeleteVertex { timestamp, .. } => timestamp,
        }
    }

    /// Check if this is a delete operation.
    pub fn is_delete(&self) -> bool {
        matches!(self, CrdtOp::DeleteFace { .. } | CrdtOp::DeleteEdge { .. } | CrdtOp::DeleteVertex { .. })
    }
}

// ============================================================
// CRDT replica state
// ============================================================

/// State of a single B-Rep entity in the CRDT.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EntityState {
    /// The latest operation applied to this entity.
    pub last_op: CrdtOp,
    /// Whether this entity is deleted (tombstone).
    pub deleted: bool,
}

/// A CRDT replica — local state of the B-Rep model for one user.
///
/// Each collaborator has their own replica. Changes are exchanged as
/// `CrdtOp` messages. Merge is automatic and conflict-free.
pub struct CrdtReplica {
    /// Unique identifier for this replica (user ID).
    pub replica_id: String,
    /// Lamport clock — incremented for each local operation.
    pub clock: u64,
    /// Entity states, keyed by entity ID.
    pub entities: HashMap<EntityId, EntityState>,
}

impl CrdtReplica {
    /// Create a new replica with the given user ID.
    pub fn new(replica_id: &str) -> Self {
        Self {
            replica_id: replica_id.to_string(),
            clock: 0,
            entities: HashMap::new(),
        }
    }

    /// Get the next Lamport timestamp for a local operation.
    fn next_timestamp(&mut self) -> LamportTimestamp {
        self.clock += 1;
        LamportTimestamp::new(self.clock, &self.replica_id)
    }

    /// Create a PutFace operation (local edit).
    pub fn put_face(&mut self, entity_id: EntityId, geometry: Vec<u8>, label: Option<String>, color: [f32; 4]) -> CrdtOp {
        let ts = self.next_timestamp();
        let op = CrdtOp::PutFace { entity_id, timestamp: ts, geometry, label, color };
        self.apply_local(&op);
        op
    }

    /// Create a DeleteFace operation (local edit).
    pub fn delete_face(&mut self, entity_id: EntityId) -> CrdtOp {
        let ts = self.next_timestamp();
        let op = CrdtOp::DeleteFace { entity_id, timestamp: ts };
        self.apply_local(&op);
        op
    }

    /// Create a PutVertex operation (local edit).
    pub fn put_vertex(&mut self, entity_id: EntityId, position: [f64; 3]) -> CrdtOp {
        let ts = self.next_timestamp();
        let op = CrdtOp::PutVertex { entity_id, timestamp: ts, position };
        self.apply_local(&op);
        op
    }

    /// Apply a local operation (from this replica).
    fn apply_local(&mut self, op: &CrdtOp) {
        let entity_id = op.entity_id();
        self.entities.insert(entity_id, EntityState {
            last_op: op.clone(),
            deleted: op.is_delete(),
        });
    }

    /// Apply a remote operation (from another replica).
    ///
    /// This is the core CRDT merge logic:
    /// - If the remote op's timestamp is NEWER than the local state → apply
    /// - If the remote op's timestamp is OLDER → ignore (stale)
    /// - If timestamps are equal → use replica_id as tiebreaker
    ///
    /// This guarantees convergence: all replicas that receive the same
    /// set of ops will end up in the same state, regardless of order.
    pub fn apply_remote(&mut self, op: &CrdtOp) -> bool {
        let entity_id = op.entity_id();

        match self.entities.get(&entity_id) {
            None => {
                // Entity doesn't exist locally → create it
                self.entities.insert(entity_id, EntityState {
                    last_op: op.clone(),
                    deleted: op.is_delete(),
                });
                true
            }
            Some(existing) => {
                // Compare timestamps: latest wins (LWW — Last Writer Wins)
                let local_ts = existing.last_op.timestamp();
                let remote_ts = op.timestamp();

                if remote_ts > local_ts {
                    // Remote is newer → apply
                    self.entities.insert(entity_id, EntityState {
                        last_op: op.clone(),
                        deleted: op.is_delete(),
                    });
                    true
                } else {
                    // Remote is older or equal → ignore (stale)
                    false
                }
            }
        }
    }

    /// Merge another replica's state into this one.
    ///
    /// Applies all operations from the other replica. This is equivalent
    /// to calling `apply_remote()` for each entity in the other replica.
    pub fn merge(&mut self, other: &CrdtReplica) -> usize {
        let mut applied = 0;
        for (entity_id, state) in &other.entities {
            if self.apply_remote(&state.last_op) {
                applied += 1;
            }
        }
        // Update clock to be ahead of both replicas
        self.clock = self.clock.max(other.clock);
        applied
    }

    /// Get all active (non-deleted) faces.
    pub fn active_faces(&self) -> Vec<(EntityId, &CrdtOp)> {
        self.entities.iter()
            .filter(|(_, state)| !state.deleted)
            .filter_map(|(id, state)| {
                if let CrdtOp::PutFace { .. } = &state.last_op {
                    Some((*id, &state.last_op))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get all active vertices.
    pub fn active_vertices(&self) -> Vec<(EntityId, [f64; 3])> {
        self.entities.iter()
            .filter(|(_, state)| !state.deleted)
            .filter_map(|(id, state)| {
                if let CrdtOp::PutVertex { position, .. } = &state.last_op {
                    Some((*id, *position))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Total number of entities (including tombstones).
    pub fn total_entities(&self) -> usize {
        self.entities.len()
    }

    /// Number of active (non-deleted) entities.
    pub fn active_count(&self) -> usize {
        self.entities.values().filter(|s| !s.deleted).count()
    }

    /// Number of deleted entities (tombstones).
    pub fn tombstone_count(&self) -> usize {
        self.entities.values().filter(|s| s.deleted).count()
    }

    /// Export all operations for synchronization.
    ///
    /// Returns the full state as a list of operations. A new replica
    /// can be initialized by applying all these operations.
    pub fn export_state(&self) -> Vec<CrdtOp> {
        self.entities.values()
            .map(|s| s.last_op.clone())
            .collect()
    }

    /// Import state from a list of operations (e.g., from server sync).
    pub fn import_state(&mut self, ops: &[CrdtOp]) -> usize {
        let mut applied = 0;
        for op in ops {
            if self.apply_remote(op) {
                applied += 1;
            }
        }
        applied
    }
}

// ============================================================
// Collaborative session
// ============================================================

/// A collaborative editing session with multiple replicas.
///
/// Manages the set of connected replicas and broadcasts operations
/// between them. In a real implementation, this would use WebSocket
/// or WebRTC for transport.
pub struct CollabSession {
    /// Session ID.
    pub session_id: String,
    /// The local replica.
    pub local: CrdtReplica,
    /// Connected remote replica IDs.
    pub connected_replicas: Vec<String>,
    /// Pending operations to broadcast (not yet sent to network).
    pub pending_broadcast: Vec<CrdtOp>,
}

impl CollabSession {
    /// Create a new collaborative session.
    pub fn new(session_id: &str, user_id: &str) -> Self {
        Self {
            session_id: session_id.to_string(),
            local: CrdtReplica::new(user_id),
            connected_replicas: Vec::new(),
            pending_broadcast: Vec::new(),
        }
    }

    /// Record a local face edit and queue it for broadcast.
    pub fn edit_face(&mut self, entity_id: EntityId, geometry: Vec<u8>, label: Option<String>, color: [f32; 4]) {
        let op = self.local.put_face(entity_id, geometry, label, color);
        self.pending_broadcast.push(op);
    }

    /// Record a local face deletion and queue it for broadcast.
    pub fn delete_face(&mut self, entity_id: EntityId) {
        let op = self.local.delete_face(entity_id);
        self.pending_broadcast.push(op);
    }

    /// Receive a remote operation from another replica.
    pub fn receive_remote(&mut self, op: &CrdtOp) -> bool {
        self.local.apply_remote(op)
    }

    /// Synchronize with another replica's full state.
    pub fn sync_with(&mut self, other: &CrdtReplica) -> usize {
        self.local.merge(other)
    }

    /// Get pending operations to broadcast and clear the queue.
    pub fn drain_pending(&mut self) -> Vec<CrdtOp> {
        std::mem::take(&mut self.pending_broadcast)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lamport_timestamp_ordering() {
        let ts1 = LamportTimestamp::new(1, "alice");
        let ts2 = LamportTimestamp::new(2, "alice");
        let ts3 = LamportTimestamp::new(1, "bob");
        assert!(ts1 < ts2);
        assert!(ts1 < ts3); // Same sequence, "alice" < "bob"
        assert!(ts3 < ts2); // seq 1 < 2
    }

    #[test]
    fn test_lamport_merge() {
        let ts1 = LamportTimestamp::new(5, "alice");
        let ts2 = LamportTimestamp::new(3, "bob");
        let merged = ts1.merge(&ts2);
        assert_eq!(merged.sequence, 6); // max(5, 3) + 1
    }

    #[test]
    fn test_crdt_put_and_get_face() {
        let mut replica = CrdtReplica::new("alice");
        replica.put_face(1, vec![0x01], Some("TOP".to_string()), [1.0, 0.0, 0.0, 1.0]);
        let faces = replica.active_faces();
        assert_eq!(faces.len(), 1);
        assert_eq!(faces[0].0, 1);
    }

    #[test]
    fn test_crdt_delete_face() {
        let mut replica = CrdtReplica::new("alice");
        replica.put_face(1, vec![0x01], None, [1.0, 0.0, 0.0, 1.0]);
        assert_eq!(replica.active_count(), 1);
        replica.delete_face(1);
        assert_eq!(replica.active_count(), 0);
        assert_eq!(replica.tombstone_count(), 1);
    }

    #[test]
    fn test_crdt_merge_convergence() {
        // Two replicas start empty, make concurrent edits, then merge
        let mut alice = CrdtReplica::new("alice");
        let mut bob = CrdtReplica::new("bob");

        // Alice adds face 1
        alice.put_face(1, vec![0x01], None, [1.0, 0.0, 0.0, 1.0]);
        // Bob adds face 2
        bob.put_face(2, vec![0x02], None, [0.0, 1.0, 0.0, 1.0]);

        // Merge: Alice receives Bob's state
        alice.merge(&bob);
        assert_eq!(alice.active_faces().len(), 2);

        // Merge: Bob receives Alice's state
        bob.merge(&alice);
        assert_eq!(bob.active_faces().len(), 2);

        // Both replicas should have the same state (convergence)
        assert_eq!(alice.active_count(), bob.active_count());
    }

    #[test]
    fn test_crdt_last_writer_wins() {
        let mut alice = CrdtReplica::new("alice");
        let mut bob = CrdtReplica::new("bob");

        // Both edit the same face, but Bob's edit is later
        alice.put_face(1, vec![0xAA], Some("Alice's version".to_string()), [1.0, 0.0, 0.0, 1.0]);
        // Bob's clock is ahead (simulated)
        bob.clock = 10;
        bob.put_face(1, vec![0xBB], Some("Bob's version".to_string()), [0.0, 1.0, 0.0, 1.0]);

        // Alice receives Bob's edit
        let applied = alice.merge(&bob);
        assert_eq!(applied, 1);

        // Alice should now have Bob's version (LWW)
        let faces = alice.active_faces();
        assert_eq!(faces.len(), 1);
        if let CrdtOp::PutFace { label, .. } = faces[0].1 {
            assert_eq!(label.as_deref(), Some("Bob's version"));
        }
    }

    #[test]
    fn test_crdt_export_import() {
        let mut alice = CrdtReplica::new("alice");
        alice.put_face(1, vec![0x01], None, [1.0, 0.0, 0.0, 1.0]);
        alice.put_face(2, vec![0x02], None, [0.0, 1.0, 0.0, 1.0]);
        alice.put_vertex(10, [1.0, 2.0, 3.0]);

        let ops = alice.export_state();
        assert_eq!(ops.len(), 3);

        // Import into a new replica
        let mut bob = CrdtReplica::new("bob");
        let applied = bob.import_state(&ops);
        assert_eq!(applied, 3);
        assert_eq!(bob.active_faces().len(), 2);
        assert_eq!(bob.active_vertices().len(), 1);
    }

    #[test]
    fn test_collab_session() {
        let mut session = CollabSession::new("session1", "alice");
        session.edit_face(1, vec![0x01], Some("TOP".to_string()), [1.0, 0.0, 0.0, 1.0]);
        session.edit_face(2, vec![0x02], Some("FRONT".to_string()), [0.0, 1.0, 0.0, 1.0]);

        // Pending broadcast should have 2 ops
        let pending = session.drain_pending();
        assert_eq!(pending.len(), 2);

        // After drain, no pending
        let pending2 = session.drain_pending();
        assert_eq!(pending2.len(), 0);

        // Local state should have both faces
        assert_eq!(session.local.active_faces().len(), 2);
    }

    #[test]
    fn test_concurrent_delete_and_edit() {
        // Alice deletes face 1, Bob edits face 1 concurrently
        let mut alice = CrdtReplica::new("alice");
        let mut bob = CrdtReplica::new("bob");

        // Both start with face 1
        alice.put_face(1, vec![0x01], None, [1.0, 0.0, 0.0, 1.0]);
        bob.import_state(&alice.export_state());

        // Alice deletes, Bob edits (concurrent — both at clock=1)
        alice.delete_face(1);
        bob.clock = 2; // Bob's edit is later
        bob.put_face(1, vec![0x02], Some("edited".to_string()), [0.0, 1.0, 0.0, 1.0]);

        // Merge: Alice receives Bob's edit (timestamp 3 > 1 → Bob wins)
        alice.merge(&bob);

        // Face should exist (Bob's edit won because timestamp was later)
        assert_eq!(alice.active_faces().len(), 1);
        assert_eq!(alice.tombstone_count(), 0);
    }
}
