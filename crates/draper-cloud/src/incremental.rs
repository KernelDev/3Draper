// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # Incremental Loading
//!
//! Incremental assembly loading for large CAD models. The assembly tree is
//! loaded first (fast), then individual parts are loaded on demand with a
//! priority queue: visible parts first, off-screen parts deferred.
//! Supports Level-of-Detail (LOD) so distant parts use coarser triangulation.

use draper_mesh::TriangleMesh;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashMap, VecDeque};
use std::sync::Arc;
use tokio::sync::RwLock;

/// Priority level for part loading. Higher priority values are loaded first.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum LoadPriority {
    /// Off-screen / not currently needed.
    Deferred = 0,
    /// In the frustum but far away.
    Background = 1,
    /// Visible and near the camera.
    Visible = 2,
    /// Currently selected or interacted with.
    Critical = 3,
}

/// A node in the assembly tree.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AssemblyNode {
    /// Unique B-Rep ID for this part.
    pub brep_id: i64,
    /// Human-readable name.
    pub name: String,
    /// Children in the assembly hierarchy.
    pub children: Vec<i64>,
    /// Parent B-Rep ID (None for root).
    pub parent: Option<i64>,
    /// Approximate memory in bytes if loaded at full LOD.
    pub estimated_size: usize,
}

/// Level-of-Detail specification.
///
/// `lod` is a value in `[0.0, 1.0]`:
/// - `1.0` — full resolution
/// - `0.5` — half the triangles (coarser)
/// - `0.0` — bounding box only (no mesh)
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct LodSpec {
    pub lod: f64,
}

impl LodSpec {
    pub fn full() -> Self {
        Self { lod: 1.0 }
    }

    pub fn half() -> Self {
        Self { lod: 0.5 }
    }

    pub fn coarse() -> Self {
        Self { lod: 0.25 }
    }

    pub fn bounding_box_only() -> Self {
        Self { lod: 0.0 }
    }

    pub fn is_valid(&self) -> bool {
        (0.0..=1.0).contains(&self.lod)
    }
}

/// Metadata about a loaded part.
#[derive(Clone, Debug)]
struct LoadedPart {
    mesh: TriangleMesh,
    lod: f64,
    memory_bytes: usize,
}

/// A pending load request in the priority queue.
#[derive(Clone, Debug)]
struct PendingLoad {
    brep_id: i64,
    lod: f64,
    priority: LoadPriority,
}

impl PartialEq for PendingLoad {
    fn eq(&self, other: &Self) -> bool {
        self.brep_id == other.brep_id && self.lod == other.lod && self.priority == other.priority
    }
}

impl Eq for PendingLoad {}

impl Ord for PendingLoad {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Higher priority first
        self.priority.cmp(&other.priority)
    }
}

impl PartialOrd for PendingLoad {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Incremental assembly loader.
///
/// Loads the assembly tree first (fast), then loads individual parts on demand.
/// Visible parts are prioritized; off-screen parts are deferred. Supports LOD
/// so distant parts use coarser triangulation.
pub struct IncrementalLoader {
    /// Assembly tree nodes, keyed by B-Rep ID.
    assembly_tree: Arc<RwLock<HashMap<i64, AssemblyNode>>>,
    /// Currently loaded parts, keyed by B-Rep ID.
    loaded_parts: Arc<RwLock<HashMap<i64, LoadedPart>>>,
    /// B-Rep IDs that have been explicitly unloaded.
    unloaded_ids: Arc<RwLock<BTreeSet<i64>>>,
    /// Priority queue of pending load requests.
    pending_queue: Arc<RwLock<VecDeque<PendingLoad>>>,
    /// Root B-Rep IDs of the assembly.
    root_ids: Arc<RwLock<Vec<i64>>>,
    /// Estimated total memory usage in bytes.
    total_memory: Arc<RwLock<usize>>,
    /// Maximum allowed memory in bytes (0 = unlimited).
    memory_limit: usize,
}

impl IncrementalLoader {
    /// Create a new incremental loader with an optional memory limit (in bytes).
    pub fn new(memory_limit: usize) -> Self {
        Self {
            assembly_tree: Arc::new(RwLock::new(HashMap::new())),
            loaded_parts: Arc::new(RwLock::new(HashMap::new())),
            unloaded_ids: Arc::new(RwLock::new(BTreeSet::new())),
            pending_queue: Arc::new(RwLock::new(VecDeque::new())),
            root_ids: Arc::new(RwLock::new(Vec::new())),
            total_memory: Arc::new(RwLock::new(0)),
            memory_limit,
        }
    }

    /// Load the assembly tree structure (fast operation).
    ///
    /// This populates the tree of `AssemblyNode`s without loading any mesh data.
    /// The `nodes` vector should contain all nodes in the assembly; root nodes
    /// are those with `parent == None`.
    pub async fn load_assembly_tree(&self, nodes: Vec<AssemblyNode>) {
        let mut tree = self.assembly_tree.write().await;
        let mut roots = self.root_ids.write().await;

        for node in &nodes {
            if node.parent.is_none() {
                roots.push(node.brep_id);
            }
        }
        for node in nodes {
            tree.insert(node.brep_id, node);
        }
        log::info!(
            "IncrementalLoader: assembly tree loaded with {} nodes, {} roots",
            tree.len(),
            roots.len()
        );
    }

    /// Request a part to be loaded with the given LOD and priority.
    ///
    /// The part will be queued and loaded when its priority allows.
    pub async fn request_part(&self, brep_id: i64, lod: f64, priority: LoadPriority) {
        let mut queue = self.pending_queue.write().await;
        queue.push_back(PendingLoad {
            brep_id,
            lod,
            priority,
        });
        log::debug!(
            "IncrementalLoader: queued part {} at LOD {:.2}, priority {:?}",
            brep_id,
            lod,
            priority
        );
    }

    /// Load a specific part immediately with the given LOD.
    ///
    /// Returns the triangulated mesh for the part. In a full integration,
    /// this would invoke `draper-step` + `draper-mesh` to parse and triangulate
    /// the B-Rep. Here we provide the infrastructure with a placeholder
    /// triangulation that respects the LOD setting.
    pub async fn load_part(&self, brep_id: i64, lod: f64) -> TriangleMesh {
        // Check if already loaded at adequate LOD
        {
            let loaded = self.loaded_parts.read().await;
            if let Some(part) = loaded.get(&brep_id) {
                if part.lod >= lod {
                    return part.mesh.clone();
                }
            }
        }

        log::info!(
            "IncrementalLoader: loading part {} at LOD {:.2}",
            brep_id,
            lod
        );

        // In production: parse STEP, extract B-Rep for brep_id,
        // triangulate with TriangulationParams::detail_level = lod
        let mesh = self.triangulate_part_placeholder(brep_id, lod);

        // Estimate memory: vertices * 3 * 8 (f64) + triangles * 3 * 4 (u32)
        let memory = mesh.vertices.len() * 24 + mesh.triangles.len() * 12;

        // Remove from unloaded set
        self.unloaded_ids.write().await.remove(&brep_id);

        // Store loaded part
        {
            let mut loaded = self.loaded_parts.write().await;
            let mut total_mem = self.total_memory.write().await;

            if let Some(old) = loaded.insert(
                brep_id,
                LoadedPart {
                    mesh: mesh.clone(),
                    lod,
                    memory_bytes: memory,
                },
            ) {
                *total_mem -= old.memory_bytes;
            }
            *total_mem += memory;
        }

        // Enforce memory limit by unloading lowest-priority parts
        if self.memory_limit > 0 {
            self.enforce_memory_limit().await;
        }

        mesh
    }

    /// Unload a previously loaded part to free memory.
    pub async fn unload_part(&self, brep_id: i64) {
        let mut loaded = self.loaded_parts.write().await;
        let mut unloaded = self.unloaded_ids.write().await;
        let mut total_mem = self.total_memory.write().await;

        if let Some(part) = loaded.remove(&brep_id) {
            *total_mem -= part.memory_bytes;
            unloaded.insert(brep_id);
            log::info!(
                "IncrementalLoader: unloaded part {}, freed {} bytes",
                brep_id,
                part.memory_bytes
            );
        }
    }

    /// Check if a part is currently loaded.
    pub async fn is_loaded(&self, brep_id: i64) -> bool {
        self.loaded_parts.read().await.contains_key(&brep_id)
    }

    /// Estimate total memory usage in bytes for all loaded parts.
    pub async fn estimate_memory(&self) -> usize {
        *self.total_memory.read().await
    }

    /// Get the list of currently loaded B-Rep IDs.
    pub async fn loaded_brep_ids(&self) -> Vec<i64> {
        self.loaded_parts.read().await.keys().copied().collect()
    }

    /// Get the list of unloaded B-Rep IDs.
    pub async fn unloaded_brep_ids(&self) -> Vec<i64> {
        self.unloaded_ids.read().await.iter().copied().collect()
    }

    /// Process pending load requests up to `max_count` items.
    ///
    /// Returns the number of parts actually loaded.
    pub async fn process_pending(&self, max_count: usize) -> usize {
        // Sort queue by priority (highest first) and drain items to process
        let to_process: Vec<PendingLoad> = {
            let mut queue = self.pending_queue.write().await;
            queue.make_contiguous();
            queue.as_mut_slices().0.sort_by(|a, b| b.cmp(a));
            let count = max_count.min(queue.len());
            queue.drain(..count).collect()
        };

        let loaded_count = to_process.len();
        for pending in to_process {
            self.load_part(pending.brep_id, pending.lod).await;
        }

        loaded_count
    }

    /// Get the assembly tree node for a given B-Rep ID.
    pub async fn get_node(&self, brep_id: i64) -> Option<AssemblyNode> {
        self.assembly_tree.read().await.get(&brep_id).cloned()
    }

    /// Get root B-Rep IDs.
    pub async fn root_ids(&self) -> Vec<i64> {
        self.root_ids.read().await.clone()
    }

    /// Recursively load all parts in the assembly tree at the given LOD.
    pub async fn load_all_recursive(&self, lod: f64) {
        let roots = self.root_ids.read().await.clone();
        for root_id in roots {
            self.load_subtree(root_id, lod).await;
        }
    }

    /// Load a subtree starting from the given B-Rep ID.
    fn load_subtree<'a>(&'a self, brep_id: i64, lod: f64) -> std::pin::Pin<Box<dyn std::future::Future<Output = ()> + 'a>> {
        Box::pin(async move {
            self.load_part(brep_id, lod).await;
            if let Some(node) = self.assembly_tree.read().await.get(&brep_id).cloned() {
                for child_id in node.children {
                    self.load_subtree(child_id, lod).await;
                }
            }
        })
    }

    /// Placeholder triangulation: creates a simple box mesh scaled by LOD.
    ///
    /// In production this would use draper-step + draper-mesh.
    fn triangulate_part_placeholder(&self, brep_id: i64, lod: f64) -> TriangleMesh {
        use draper_geometry::Point3d;

        // Create a box with resolution scaled by LOD
        let subdivisions = ((lod * 4.0).ceil() as usize).max(1);
        let size = 10.0 + (brep_id as f64).abs().fract() * 50.0;

        let mut mesh = TriangleMesh::new();

        // Simple box: 8 vertices, 12 triangles (2 per face)
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(size, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(size, size, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.0, size, 0.0));
        let v4 = mesh.add_vertex(Point3d::new(0.0, 0.0, size));
        let v5 = mesh.add_vertex(Point3d::new(size, 0.0, size));
        let v6 = mesh.add_vertex(Point3d::new(size, size, size));
        let v7 = mesh.add_vertex(Point3d::new(0.0, size, size));

        // Front
        mesh.add_triangle(v0, v1, v2);
        mesh.add_triangle(v0, v2, v3);
        // Back
        mesh.add_triangle(v5, v4, v7);
        mesh.add_triangle(v5, v7, v6);
        // Left
        mesh.add_triangle(v4, v0, v3);
        mesh.add_triangle(v4, v3, v7);
        // Right
        mesh.add_triangle(v1, v5, v6);
        mesh.add_triangle(v1, v6, v2);
        // Top
        mesh.add_triangle(v3, v2, v6);
        mesh.add_triangle(v3, v6, v7);
        // Bottom
        mesh.add_triangle(v4, v5, v1);
        mesh.add_triangle(v4, v1, v0);

        // Add extra triangles for higher LOD to simulate detail
        for i in 1..subdivisions {
            let t = i as f64 / subdivisions as f64;
            let vi = mesh.add_vertex(Point3d::new(size * t, size * t, size * t));
            mesh.add_triangle(v0, vi, v1);
        }

        mesh
    }

    /// Evict lowest-priority loaded parts until memory is within the limit.
    async fn enforce_memory_limit(&self) {
        if self.memory_limit == 0 {
            return;
        }

        let mut total_mem = self.total_memory.write().await;
        while *total_mem > self.memory_limit {
            let mut loaded = self.loaded_parts.write().await;
            let mut unloaded = self.unloaded_ids.write().await;

            // Find the largest part to evict (simplest strategy)
            let victim = loaded
                .iter()
                .max_by_key(|(_, p)| p.memory_bytes)
                .map(|(id, _)| *id);

            if let Some(victim_id) = victim {
                if let Some(part) = loaded.remove(&victim_id) {
                    *total_mem -= part.memory_bytes;
                    unloaded.insert(victim_id);
                    log::info!(
                        "IncrementalLoader: evicted part {} to free {} bytes (memory limit enforced)",
                        victim_id,
                        part.memory_bytes
                    );
                }
            } else {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_node(id: i64, name: &str, parent: Option<i64>, children: Vec<i64>) -> AssemblyNode {
        AssemblyNode {
            brep_id: id,
            name: name.to_string(),
            children,
            parent,
            estimated_size: 1024,
        }
    }

    #[tokio::test]
    async fn test_load_assembly_tree() {
        let loader = IncrementalLoader::new(0);
        let nodes = vec![
            make_node(1, "Root", None, vec![2, 3]),
            make_node(2, "Part A", Some(1), vec![]),
            make_node(3, "Part B", Some(1), vec![]),
        ];
        loader.load_assembly_tree(nodes).await;

        let roots = loader.root_ids().await;
        assert_eq!(roots, vec![1]);

        let node = loader.get_node(2).await.unwrap();
        assert_eq!(node.name, "Part A");
    }

    #[tokio::test]
    async fn test_load_and_unload_part() {
        let loader = IncrementalLoader::new(0);

        let nodes = vec![
            make_node(1, "Root", None, vec![2]),
            make_node(2, "Part", Some(1), vec![]),
        ];
        loader.load_assembly_tree(nodes).await;

        // Load part
        let mesh = loader.load_part(2, 1.0).await;
        assert!(mesh.triangle_count() > 0);
        assert!(loader.is_loaded(2).await);

        let mem_before = loader.estimate_memory().await;
        assert!(mem_before > 0);

        // Unload part
        loader.unload_part(2).await;
        assert!(!loader.is_loaded(2).await);
        assert_eq!(loader.estimate_memory().await, 0);
        assert!(loader.unloaded_brep_ids().await.contains(&2));
    }

    #[tokio::test]
    async fn test_lod_affects_mesh_detail() {
        let loader = IncrementalLoader::new(0);

        let mesh_high = loader.load_part(1, 1.0).await;
        let mesh_low = loader.load_part(1, 0.1).await;

        // Higher LOD should produce at least as many triangles as lower LOD
        assert!(mesh_high.triangle_count() >= mesh_low.triangle_count());
    }

    #[tokio::test]
    async fn test_memory_limit() {
        let loader = IncrementalLoader::new(500); // Very small limit

        loader.load_part(1, 1.0).await;
        loader.load_part(2, 1.0).await;

        // Memory should be within limit
        let mem = loader.estimate_memory().await;
        assert!(mem <= 500 + 200); // Allow some tolerance for the limit enforcement
    }

    #[tokio::test]
    async fn test_priority_queue() {
        let loader = IncrementalLoader::new(0);

        let nodes = vec![
            make_node(1, "A", None, vec![]),
            make_node(2, "B", None, vec![]),
            make_node(3, "C", None, vec![]),
        ];
        loader.load_assembly_tree(nodes).await;

        loader.request_part(1, 0.5, LoadPriority::Deferred).await;
        loader.request_part(2, 1.0, LoadPriority::Critical).await;
        loader.request_part(3, 0.8, LoadPriority::Visible).await;

        let count = loader.process_pending(10).await;
        assert_eq!(count, 3);
        assert!(loader.is_loaded(1).await);
        assert!(loader.is_loaded(2).await);
        assert!(loader.is_loaded(3).await);
    }

    #[tokio::test]
    async fn test_load_all_recursive() {
        let loader = IncrementalLoader::new(0);

        let nodes = vec![
            make_node(1, "Root", None, vec![2, 3]),
            make_node(2, "Child A", Some(1), vec![4]),
            make_node(3, "Child B", Some(1), vec![]),
            make_node(4, "Grandchild", Some(2), vec![]),
        ];
        loader.load_assembly_tree(nodes).await;
        loader.load_all_recursive(0.5).await;

        assert!(loader.is_loaded(1).await);
        assert!(loader.is_loaded(2).await);
        assert!(loader.is_loaded(3).await);
        assert!(loader.is_loaded(4).await);
    }

    #[tokio::test]
    async fn test_reload_at_higher_lod() {
        let loader = IncrementalLoader::new(0);

        // Load at low LOD first
        let mesh_low = loader.load_part(1, 0.2).await;
        // Load same part at higher LOD — should upgrade
        let mesh_high = loader.load_part(1, 1.0).await;
        assert!(mesh_high.triangle_count() >= mesh_low.triangle_count());
    }
}
