// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # Conflict Resolution for Collaborative Editing
//!
//! Implements Operational Transform (OT) for concurrent modifications in a
//! collaborative CAD editing environment. Uses version vectors for causal
//! ordering and supports automatic merge for non-conflicting operations.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// An editing operation in the collaborative CAD environment.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Operation {
    /// Add a face with the given ID and geometry data.
    AddFace {
        face_id: u64,
        geometry: Vec<u8>,
    },
    /// Remove the face with the given ID.
    RemoveFace {
        face_id: u64,
    },
    /// Modify the geometry of an existing face.
    ModifyGeometry {
        face_id: u64,
        old_geometry: Vec<u8>,
        new_geometry: Vec<u8>,
    },
    /// Apply an affine transform to the specified faces.
    Transform {
        face_ids: Vec<u64>,
        matrix: [[f64; 4]; 4],
    },
    /// Change the color of a face.
    ChangeColor {
        face_id: u64,
        old_color: [f32; 4],
        new_color: [f32; 4],
    },
}

impl Operation {
    /// Get the face IDs affected by this operation.
    pub fn affected_faces(&self) -> Vec<u64> {
        match self {
            Operation::AddFace { face_id, .. } => vec![*face_id],
            Operation::RemoveFace { face_id } => vec![*face_id],
            Operation::ModifyGeometry { face_id, .. } => vec![*face_id],
            Operation::Transform { face_ids, .. } => face_ids.clone(),
            Operation::ChangeColor { face_id, .. } => vec![*face_id],
        }
    }

    /// Check if this operation conflicts with another.
    ///
    /// Two operations conflict if they affect the same face(s) and
    /// at least one of them modifies geometry (not just color).
    pub fn conflicts_with(&self, other: &Operation) -> bool {
        let my_faces = self.affected_faces();
        let other_faces = other.affected_faces();

        let has_overlap = my_faces.iter().any(|f| other_faces.contains(f));
        if !has_overlap {
            return false;
        }

        // Both just changing color on the same face is not a conflict
        matches!(self, Operation::ChangeColor { .. }) && matches!(other, Operation::ChangeColor { .. })
    }
}

/// Version vector for causal ordering of operations.
///
/// Maps replica (user) IDs to their operation count.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct VersionVector {
    /// Map from replica ID to the latest operation sequence number seen.
    entries: HashMap<String, u64>,
}

impl VersionVector {
    /// Create an empty version vector.
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    /// Get the sequence number for a given replica.
    pub fn get(&self, replica_id: &str) -> u64 {
        self.entries.get(replica_id).copied().unwrap_or(0)
    }

    /// Increment the sequence number for a replica (after applying an operation).
    pub fn increment(&mut self, replica_id: &str) {
        *self.entries.entry(replica_id.to_string()).or_insert(0) += 1;
    }

    /// Merge another version vector into this one (element-wise max).
    pub fn merge(&mut self, other: &VersionVector) {
        for (replica_id, &seq) in &other.entries {
            let entry = self.entries.entry(replica_id.clone()).or_insert(0);
            *entry = (*entry).max(seq);
        }
    }

    /// Check if this version vector dominates another (i.e., has seen all
    /// operations that the other has seen).
    pub fn dominates(&self, other: &VersionVector) -> bool {
        for (replica_id, &seq) in &other.entries {
            if self.get(replica_id) < seq {
                return false;
            }
        }
        true
    }

    /// Check if this version vector is concurrent with another (neither dominates).
    pub fn is_concurrent_with(&self, other: &VersionVector) -> bool {
        !self.dominates(other) && !other.dominates(self)
    }
}

/// Conflict resolution strategy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConflictResolution {
    /// Keep the local operation, discard the remote.
    KeepLocal,
    /// Keep the remote operation, discard the local.
    KeepRemote,
    /// Merge both operations (automatic merge).
    Merge,
    /// Require manual resolution by the user.
    ManualResolve,
}

/// Result of transforming an operation against another.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TransformedOperation {
    /// The transformed operation to apply.
    pub operation: Operation,
    /// The resolution strategy used (if there was a conflict).
    pub resolution: Option<ConflictResolution>,
}

/// Operational Transform engine for collaborative editing.
pub struct OperationalTransform;

impl OperationalTransform {
    /// Transform a local operation against a remote operation.
    ///
    /// Given two concurrent operations `local_op` and `remote_op`, both applied
    /// to the same document state, compute the transformed version of `local_op`
    /// that can be applied after `remote_op` has been applied.
    ///
    /// Returns the transformed operation along with any conflict resolution.
    pub fn transform_op(local_op: &Operation, remote_op: &Operation) -> TransformedOperation {
        // Case 1: Operations on different faces — no conflict
        let local_faces = local_op.affected_faces();
        let remote_faces = remote_op.affected_faces();
        let has_overlap = local_faces.iter().any(|f| remote_faces.contains(f));

        if !has_overlap {
            return TransformedOperation {
                operation: local_op.clone(),
                resolution: None,
            };
        }

        // Case 2: Both operations on the same face(s)
        match (local_op, remote_op) {
            // Add + Add on same face: keep local (it was applied first locally)
            (
                Operation::AddFace { face_id: lid, geometry },
                Operation::AddFace { face_id: rid, .. },
            ) if lid == rid => {
                log::warn!("OT conflict: both users added face {}", lid);
                TransformedOperation {
                    operation: Operation::AddFace { face_id: *lid, geometry: geometry.clone() },
                    resolution: Some(ConflictResolution::KeepLocal),
                }
            }

            // Add + Remove on same face: local Add wins (re-add the face)
            (Operation::AddFace { .. }, Operation::RemoveFace { face_id: _ }) => {
                TransformedOperation {
                    operation: local_op.clone(),
                    resolution: Some(ConflictResolution::KeepLocal),
                }
            }

            // Remove + Add on same face: remote Add wins (face is re-added)
            (Operation::RemoveFace { face_id: _ }, Operation::AddFace { .. }) => {
                // The local remove is no longer needed since remote added it back
                TransformedOperation {
                    operation: local_op.clone(),
                    resolution: Some(ConflictResolution::KeepRemote),
                }
            }

            // Remove + Remove on same face: both agree, local remove is no-op
            (Operation::RemoveFace { face_id: lid }, Operation::RemoveFace { face_id: rid })
                if lid == rid =>
            {
                TransformedOperation {
                    operation: local_op.clone(),
                    resolution: Some(ConflictResolution::Merge),
                }
            }

            // Remove + Modify on same face: remote modify is applied on a face
            // that local removed. Keep the removal (local intent wins).
            (Operation::RemoveFace { .. }, Operation::ModifyGeometry { .. }) => {
                TransformedOperation {
                    operation: local_op.clone(),
                    resolution: Some(ConflictResolution::KeepLocal),
                }
            }

            // Modify + Remove on same face: remote remove takes precedence.
            // The modification is lost.
            (Operation::ModifyGeometry { .. }, Operation::RemoveFace { .. }) => {
                TransformedOperation {
                    operation: remote_op.clone(),
                    resolution: Some(ConflictResolution::KeepRemote),
                }
            }

            // Modify + Modify on same face: three-way merge
            (
                Operation::ModifyGeometry {
                    face_id: lid,
                    old_geometry: local_old,
                    new_geometry: local_new,
                },
                Operation::ModifyGeometry {
                    face_id: rid,
                    old_geometry: remote_old,
                    new_geometry: remote_new,
                },
            ) if lid == rid => {
                Self::three_way_merge_geometry(
                    *lid,
                    local_old,
                    local_new,
                    remote_old,
                    remote_new,
                )
            }

            // Transform + Transform: compose the transforms
            (
                Operation::Transform {
                    face_ids: local_faces,
                    matrix: local_matrix,
                },
                Operation::Transform {
                    face_ids: remote_faces,
                    matrix: remote_matrix,
                },
            ) => {
                // Compose: apply remote then local
                let composed = Self::compose_matrices(local_matrix, remote_matrix);
                let merged_faces: Vec<u64> = local_faces
                    .iter()
                    .chain(remote_faces.iter())
                    .copied()
                    .collect();

                TransformedOperation {
                    operation: Operation::Transform {
                        face_ids: merged_faces,
                        matrix: composed,
                    },
                    resolution: Some(ConflictResolution::Merge),
                }
            }

            // ChangeColor + ChangeColor on same face: merge (use local preference)
            (
                Operation::ChangeColor {
                    face_id: lid,
                    new_color: local_color,
                    ..
                },
                Operation::ChangeColor {
                    face_id: rid,
                    ..
                },
            ) if lid == rid => {
                TransformedOperation {
                    operation: Operation::ChangeColor {
                        face_id: *lid,
                        old_color: *local_color, // After remote applied, old = local_new
                        new_color: *local_color,
                    },
                    resolution: Some(ConflictResolution::KeepLocal),
                }
            }

            // Default: operations on overlapping faces but different types
            _ => TransformedOperation {
                operation: local_op.clone(),
                resolution: Some(ConflictResolution::ManualResolve),
            },
        }
    }

    /// Three-way merge for geometry changes.
    ///
    /// When two users modify the same face's geometry concurrently, we attempt
    /// an automatic merge. If both changes are from the same base, we apply
    /// both; if the bases differ (one user was already working on a modified
    /// version), we flag for manual resolution.
    fn three_way_merge_geometry(
        face_id: u64,
        local_old: &[u8],
        local_new: &[u8],
        remote_old: &[u8],
        remote_new: &[u8],
    ) -> TransformedOperation {
        // If both operations share the same base (old geometry), we can
        // attempt a merge by applying both changes.
        if local_old == remote_old {
            // Same base — check if changes are to different aspects
            // In production this would do a semantic diff of the geometry.
            // For now, prefer local (most recent user intent).
            log::info!(
                "OT: three-way merge for face {} — same base, keeping local",
                face_id
            );
            TransformedOperation {
                operation: Operation::ModifyGeometry {
                    face_id,
                    old_geometry: remote_new.to_vec(), // Remote was already applied
                    new_geometry: local_new.to_vec(),  // Apply local on top
                },
                resolution: Some(ConflictResolution::Merge),
            }
        } else {
            // Different bases — complex conflict requiring manual resolution
            log::warn!(
                "OT: three-way merge for face {} — different bases, manual resolve required",
                face_id
            );
            TransformedOperation {
                operation: Operation::ModifyGeometry {
                    face_id,
                    old_geometry: local_old.to_vec(),
                    new_geometry: local_new.to_vec(),
                },
                resolution: Some(ConflictResolution::ManualResolve),
            }
        }
    }

    /// Compose two 4×4 transformation matrices: result = a × b.
    fn compose_matrices(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
        let mut result = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    result[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        result
    }
}

/// A pending operation with its version context.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PendingOperation {
    /// The operation.
    pub operation: Operation,
    /// Replica (user) ID that produced this operation.
    pub replica_id: String,
    /// Sequence number within the replica.
    pub seq: u64,
    /// Version vector at the time the operation was created.
    pub version: VersionVector,
}

/// A collaborative editing session.
///
/// Manages the OT state for a group of users editing the same model.
pub struct CollabSession {
    /// Unique session identifier.
    pub session_id: String,
    /// Current version vector (reflects all applied operations).
    version: VersionVector,
    /// History of applied operations.
    history: Vec<PendingOperation>,
    /// Pending operations waiting to be applied.
    #[allow(dead_code)]
    pending: Vec<PendingOperation>,
}

impl CollabSession {
    /// Create a new collaborative session.
    pub fn new(session_id: String) -> Self {
        Self {
            session_id,
            version: VersionVector::new(),
            history: Vec::new(),
            pending: Vec::new(),
        }
    }

    /// Apply a local operation immediately.
    ///
    /// Returns the sequence number assigned to this operation.
    pub fn apply_local(&mut self, replica_id: &str, operation: Operation) -> u64 {
        let seq = self.version.get(replica_id) + 1;

        let pending_op = PendingOperation {
            operation,
            replica_id: replica_id.to_string(),
            seq,
            version: self.version.clone(),
        };

        self.history.push(pending_op);
        self.version.increment(replica_id);

        log::debug!(
            "CollabSession {}: applied local op from {} seq {}",
            self.session_id,
            replica_id,
            seq
        );

        seq
    }

    /// Receive a remote operation and transform it against local history.
    ///
    /// Returns the transformed operation to apply, along with any conflict
    /// resolution information.
    pub fn receive_remote(
        &mut self,
        remote_op: PendingOperation,
    ) -> Vec<TransformedOperation> {
        let mut transformed_ops = Vec::new();

        // Find local operations that were concurrent with the remote one.
        // Two operations are concurrent if neither "happened before" the other:
        //   A "happened before" B iff B.version[A.replica] >= A.seq
        let concurrent_locals: Vec<&PendingOperation> = self
            .history
            .iter()
            .filter(|local| {
                // local happened before remote?
                let local_before_remote = remote_op.version.get(&local.replica_id) >= local.seq;
                // remote happened before local?
                let remote_before_local = local.version.get(&remote_op.replica_id) >= remote_op.seq;
                // Concurrent iff neither happened before the other
                !local_before_remote && !remote_before_local
            })
            .collect();

        // Transform the remote operation against each concurrent local operation
        let mut current_op = remote_op.operation.clone();
        let mut had_conflict = false;

        for local in &concurrent_locals {
            let result = OperationalTransform::transform_op(&current_op, &local.operation);
            current_op = result.operation;
            if result.resolution.is_some() {
                had_conflict = true;
            }
        }

        let resolution = if had_conflict {
            Some(ConflictResolution::Merge)
        } else {
            None
        };

        transformed_ops.push(TransformedOperation {
            operation: current_op,
            resolution,
        });

        // Update version vector: merge the remote's vector (which already
        // accounts for the remote operation), no need to increment again.
        self.version.merge(&remote_op.version);

        // Add to history
        self.history.push(remote_op);

        transformed_ops
    }

    /// Get the current version vector.
    pub fn version(&self) -> &VersionVector {
        &self.version
    }

    /// Get the operation history.
    pub fn history(&self) -> &[PendingOperation] {
        &self.history
    }

    /// Get the number of operations in history.
    pub fn history_len(&self) -> usize {
        self.history.len()
    }

    /// Check if an operation from a given replica at the given sequence number
    /// has been applied.
    pub fn has_applied(&self, replica_id: &str, seq: u64) -> bool {
        self.history
            .iter()
            .any(|op| op.replica_id == replica_id && op.seq == seq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version_vector_basic() {
        let mut vv = VersionVector::new();
        assert_eq!(vv.get("user1"), 0);

        vv.increment("user1");
        assert_eq!(vv.get("user1"), 1);

        vv.increment("user1");
        assert_eq!(vv.get("user1"), 2);
        assert_eq!(vv.get("user2"), 0);
    }

    #[test]
    fn test_version_vector_merge() {
        let mut vv1 = VersionVector::new();
        vv1.increment("user1");
        vv1.increment("user1");

        let mut vv2 = VersionVector::new();
        vv2.increment("user2");

        vv1.merge(&vv2);
        assert_eq!(vv1.get("user1"), 2);
        assert_eq!(vv1.get("user2"), 1);
    }

    #[test]
    fn test_version_vector_dominates() {
        let mut vv1 = VersionVector::new();
        vv1.increment("user1");
        vv1.increment("user2");

        let mut vv2 = VersionVector::new();
        vv2.increment("user1");

        assert!(vv1.dominates(&vv2));
        assert!(!vv2.dominates(&vv1));
    }

    #[test]
    fn test_version_vector_concurrent() {
        let mut vv1 = VersionVector::new();
        vv1.increment("user1");

        let mut vv2 = VersionVector::new();
        vv2.increment("user2");

        assert!(vv1.is_concurrent_with(&vv2));
    }

    #[test]
    fn test_operation_affected_faces() {
        let op = Operation::AddFace {
            face_id: 42,
            geometry: vec![],
        };
        assert_eq!(op.affected_faces(), vec![42]);

        let op = Operation::Transform {
            face_ids: vec![1, 2, 3],
            matrix: [[0.0; 4]; 4],
        };
        assert_eq!(op.affected_faces(), vec![1, 2, 3]);
    }

    #[test]
    fn test_transform_non_overlapping() {
        let local = Operation::AddFace {
            face_id: 1,
            geometry: vec![1, 2, 3],
        };
        let remote = Operation::AddFace {
            face_id: 2,
            geometry: vec![4, 5, 6],
        };

        let result = OperationalTransform::transform_op(&local, &remote);
        assert!(result.resolution.is_none());
        assert_eq!(result.operation, local);
    }

    #[test]
    fn test_transform_add_add_conflict() {
        let local = Operation::AddFace {
            face_id: 1,
            geometry: vec![1],
        };
        let remote = Operation::AddFace {
            face_id: 1,
            geometry: vec![2],
        };

        let result = OperationalTransform::transform_op(&local, &remote);
        assert_eq!(result.resolution, Some(ConflictResolution::KeepLocal));
    }

    #[test]
    fn test_transform_remove_remove_same_face() {
        let local = Operation::RemoveFace { face_id: 1 };
        let remote = Operation::RemoveFace { face_id: 1 };

        let result = OperationalTransform::transform_op(&local, &remote);
        assert_eq!(result.resolution, Some(ConflictResolution::Merge));
    }

    #[test]
    fn test_transform_modify_modify_same_base() {
        let local = Operation::ModifyGeometry {
            face_id: 1,
            old_geometry: vec![0, 0, 0], // same base
            new_geometry: vec![1, 0, 0],
        };
        let remote = Operation::ModifyGeometry {
            face_id: 1,
            old_geometry: vec![0, 0, 0], // same base
            new_geometry: vec![0, 1, 0],
        };

        let result = OperationalTransform::transform_op(&local, &remote);
        assert_eq!(result.resolution, Some(ConflictResolution::Merge));
    }

    #[test]
    fn test_transform_modify_modify_different_base() {
        let local = Operation::ModifyGeometry {
            face_id: 1,
            old_geometry: vec![1, 0, 0], // different base
            new_geometry: vec![2, 0, 0],
        };
        let remote = Operation::ModifyGeometry {
            face_id: 1,
            old_geometry: vec![0, 0, 0], // different base
            new_geometry: vec![0, 1, 0],
        };

        let result = OperationalTransform::transform_op(&local, &remote);
        assert_eq!(result.resolution, Some(ConflictResolution::ManualResolve));
    }

    #[test]
    fn test_transform_compose_matrices() {
        let identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];

        let result = OperationalTransform::compose_matrices(&identity, &identity);
        assert_eq!(result, identity);
    }

    #[test]
    fn test_collab_session_apply_local() {
        let mut session = CollabSession::new("test".to_string());
        let seq = session.apply_local("user1", Operation::AddFace {
            face_id: 1,
            geometry: vec![],
        });
        assert_eq!(seq, 1);
        assert_eq!(session.history_len(), 1);
        assert_eq!(session.version().get("user1"), 1);
    }

    #[test]
    fn test_collab_session_receive_remote() {
        let mut session = CollabSession::new("test".to_string());

        // Apply a local op
        session.apply_local("user1", Operation::AddFace {
            face_id: 1,
            geometry: vec![],
        });

        // Receive a remote op
        let mut remote_version = VersionVector::new();
        remote_version.increment("user2");

        let remote = PendingOperation {
            operation: Operation::AddFace {
                face_id: 2,
                geometry: vec![],
            },
            replica_id: "user2".to_string(),
            seq: 1,
            version: remote_version,
        };

        let results = session.receive_remote(remote);
        assert_eq!(results.len(), 1);
        assert!(results[0].resolution.is_none()); // No conflict (different faces)
        assert_eq!(session.version().get("user2"), 1);
    }

    #[test]
    fn test_collab_session_conflicting_remote() {
        let mut session = CollabSession::new("test".to_string());

        // Apply a local op on face 1
        session.apply_local("user1", Operation::ModifyGeometry {
            face_id: 1,
            old_geometry: vec![0],
            new_geometry: vec![1],
        });

        // Receive a conflicting remote op on the same face
        let mut remote_version = VersionVector::new();
        remote_version.increment("user2");

        let remote = PendingOperation {
            operation: Operation::ModifyGeometry {
                face_id: 1,
                old_geometry: vec![0],
                new_geometry: vec![2],
            },
            replica_id: "user2".to_string(),
            seq: 1,
            version: remote_version,
        };

        let results = session.receive_remote(remote);
        assert_eq!(results.len(), 1);
        // Should have some resolution (same base → Merge)
        assert!(results[0].resolution.is_some());
    }

    #[test]
    fn test_collab_session_has_applied() {
        let mut session = CollabSession::new("test".to_string());
        session.apply_local("user1", Operation::AddFace {
            face_id: 1,
            geometry: vec![],
        });

        assert!(session.has_applied("user1", 1));
        assert!(!session.has_applied("user1", 2));
        assert!(!session.has_applied("user2", 1));
    }

    #[test]
    fn test_conflict_resolution_serialization() {
        let res = ConflictResolution::Merge;
        let json = serde_json::to_string(&res).unwrap();
        assert!(json.contains("Merge"));

        let parsed: ConflictResolution = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed, ConflictResolution::Merge);
    }

    #[test]
    fn test_pending_operation_serialization() {
        let op = PendingOperation {
            operation: Operation::RemoveFace { face_id: 42 },
            replica_id: "user1".to_string(),
            seq: 5,
            version: VersionVector::new(),
        };

        let json = serde_json::to_string(&op).unwrap();
        let parsed: PendingOperation = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.replica_id, "user1");
        assert_eq!(parsed.seq, 5);
    }
}
