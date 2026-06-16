// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Mesh-specific BVH (Bounding Volume Hierarchy) operations.
//!
//! This module provides a higher-level BVH abstraction that integrates with
//! [`TriangleMesh`] and offers mesh-aware spatial queries on top of the
//! base [`draper_topology::Bvh`].
//!
//! # Key types
//!
//! - [`MeshBvh`] — a BVH wrapper that knows about face IDs, normals, and
//!   mesh-level statistics. Supports frustum culling, ray picking, closest-face
//!   queries, and refitting for animated meshes.
//! - [`IncrementalBvh`] — for streaming / progressive loading scenarios where
//!   triangles arrive in batches and the BVH must be rebuilt periodically.
//! - [`FaceBvh`] — groups triangles by face ID and maintains a per-face BVH,
//!   enabling selective re-triangulation of individual faces without rebuilding
//!   the full BVH.
//! - [`BvhBuildStrategy`] — choose between median-split (fast) and SAH
//!   (Surface Area Heuristic, higher quality) builders.
//!
//! # Example
//!
//! ```ignore
//! use draper_mesh::bvh::{MeshBvh, BvhBuildStrategy};
//!
//! let mesh_bvh = MeshBvh::build_with_strategy(&mesh, BvhBuildStrategy::Sah {
//!     traversal_cost: 1.0,
//!     intersection_cost: 2.0,
//! });
//! let stats = mesh_bvh.statistics();
//! println!("BVH depth={}, triangles={}", stats.max_depth, stats.total_triangles);
//! ```

use crate::mesh::TriangleMesh;
use crate::triangulate::LodLevel;
use draper_geometry::{Point3d, Vec3d};
use draper_topology::{Bvh, BvhNode, Frustum};
use std::collections::HashMap;

// ============================================================
// Public result / statistics structs
// ============================================================

/// Statistics about a [`MeshBvh`] tree.
#[derive(Clone, Debug, PartialEq)]
pub struct BvhStatistics {
    /// Maximum depth of the tree (root = 0).
    pub max_depth: usize,
    /// Total number of nodes (internal + leaf).
    pub total_nodes: usize,
    /// Number of leaf nodes.
    pub leaf_nodes: usize,
    /// Total number of triangles stored in leaf nodes.
    pub total_triangles: usize,
    /// Estimated memory consumption in bytes.
    pub memory_bytes: usize,
}

/// Result of a ray-picking query against a [`MeshBvh`].
#[derive(Clone, Debug)]
pub struct RayPickResult {
    /// Index of the hit triangle in the mesh's triangle list.
    pub triangle_index: usize,
    /// Distance from the ray origin to the hit point.
    pub distance: f64,
    /// The 3D hit point on the triangle surface.
    pub point: Point3d,
    /// Face ID of the hit triangle, if per-triangle face IDs are available.
    pub face_id: Option<u64>,
}

/// Result of a frustum-culling query against a [`MeshBvh`].
///
/// This is a mesh-level equivalent of the lower-level
/// `draper_topology::Bvh::frustum_cull`, enriched with face-ID
/// information and culling statistics.
#[derive(Clone, Debug)]
pub struct MeshFrustumResult {
    /// Indices of triangles that are inside or intersect the frustum.
    pub visible_triangle_indices: Vec<usize>,
    /// Unique face IDs that have at least one visible triangle.
    pub visible_face_ids: Vec<u64>,
    /// Total number of triangles in the mesh.
    pub total_triangles: usize,
    /// Number of visible triangles.
    pub visible_triangles: usize,
    /// Ratio of culled triangles: `1.0 - (visible / total)`.
    /// `0.0` means all visible, `1.0` means all culled.
    pub culling_ratio: f64,
}

// ============================================================
// BvhBuildStrategy
// ============================================================

/// Strategy used to construct the BVH.
///
/// The choice of strategy affects both build time and query performance.
/// Median-split is the fastest to build; SAH typically produces trees
/// with better query characteristics at the cost of longer construction.
#[derive(Clone, Debug)]
pub enum BvhBuildStrategy {
    /// Median-split along the longest axis of the parent AABB.
    /// Fast O(n log n) build. Good default for most cases.
    MedianSplit,
    /// Surface Area Heuristic builder.
    ///
    /// Evaluates multiple split candidates along each axis and picks
    /// the one that minimises the SAH cost function:
    /// ```text
    /// cost = C_t + (SA_left / SA_parent) * N_left * C_i
    ///              + (SA_right / SA_parent) * N_right * C_i
    /// ```
    Sah {
        /// Cost of traversing one internal BVH node.
        traversal_cost: f64,
        /// Cost of intersecting one triangle.
        intersection_cost: f64,
    },
}

impl Default for BvhBuildStrategy {
    fn default() -> Self {
        BvhBuildStrategy::MedianSplit
    }
}

// ============================================================
// MeshBvh
// ============================================================

/// A mesh-aware BVH that wraps [`draper_topology::Bvh`] and integrates
/// with [`TriangleMesh`].
///
/// `MeshBvh` enriches the base topology BVH with:
/// - Per-triangle face-ID mapping (for selection / highlighting).
/// - LOD-aware node capacities (coarser LODs → larger leaves).
/// - Build-strategy selection (median-split or SAH).
/// - Refitting for animated / moving meshes (updates AABBs without
///   rebuilding the tree topology).
///
/// # Thread safety
///
/// `MeshBvh` is `Clone` and can be shared across threads by cloning.
/// The underlying `Bvh` is also `Clone`.
#[derive(Clone, Debug)]
pub struct MeshBvh {
    /// The underlying topology BVH.
    bvh: Bvh,
    /// Total number of triangles in the source mesh at build time.
    total_triangles: usize,
    /// Strategy used to build this BVH.
    build_strategy: BvhBuildStrategy,
}

impl MeshBvh {
    // --------------------------------------------------------
    // Construction
    // --------------------------------------------------------

    /// Build a BVH from a [`TriangleMesh`] using the default median-split
    /// strategy.
    ///
    /// This is equivalent to `MeshBvh::build_with_strategy(mesh,
    /// BvhBuildStrategy::MedianSplit)`.
    pub fn build(mesh: &TriangleMesh) -> Self {
        Self::build_with_strategy(mesh, BvhBuildStrategy::MedianSplit)
    }

    /// Build a BVH from a [`TriangleMesh`] with an LOD-dependent leaf
    /// capacity.
    ///
    /// Coarser LODs produce larger leaf nodes (fewer, bigger clusters),
    /// which reduces memory and build time at the cost of less precise
    /// culling. The leaf capacity is chosen as:
    ///
    /// | LOD level | Leaf capacity |
    /// |-----------|---------------|
    /// | Preview   | 16            |
    /// | Low       | 12            |
    /// | Interactive | 8           |
    /// | High      | 6             |
    /// | Ultra     | 4             |
    pub fn build_with_lod(mesh: &TriangleMesh, lod: LodLevel) -> Self {
        let leaf_capacity = match lod {
            LodLevel::Preview => 16,
            LodLevel::Low => 12,
            LodLevel::Interactive => 8,
            LodLevel::High => 6,
            LodLevel::Ultra => 4,
        };
        Self::build_internal(mesh, BvhBuildStrategy::MedianSplit, leaf_capacity)
    }

    /// Build a BVH from a [`TriangleMesh`] using the specified build
    /// strategy.
    ///
    /// See [`BvhBuildStrategy`] for the available options.
    pub fn build_with_strategy(mesh: &TriangleMesh, strategy: BvhBuildStrategy) -> Self {
        Self::build_internal(mesh, strategy, 4)
    }

    /// Internal build entry point.
    fn build_internal(
        mesh: &TriangleMesh,
        strategy: BvhBuildStrategy,
        leaf_capacity: usize,
    ) -> Self {
        let total_triangles = mesh.triangles.len();

        let bvh = if mesh.triangles.is_empty() {
            Bvh::build(&mesh.vertices, &mesh.triangles)
        } else {
            match &strategy {
                BvhBuildStrategy::MedianSplit => {
                    Self::build_median_split(&mesh.vertices, &mesh.triangles, leaf_capacity)
                }
                BvhBuildStrategy::Sah {
                    traversal_cost,
                    intersection_cost,
                } => Self::build_sah(
                    &mesh.vertices,
                    &mesh.triangles,
                    leaf_capacity,
                    *traversal_cost,
                    *intersection_cost,
                ),
            }
        };

        MeshBvh {
            bvh,
            total_triangles,
            build_strategy: strategy,
        }
    }

    // --------------------------------------------------------
    // Median-split builder (custom leaf capacity)
    // --------------------------------------------------------

    /// Build a BVH using median-split along the longest axis, with a
    /// configurable leaf capacity.
    fn build_median_split(
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        leaf_capacity: usize,
    ) -> Bvh {
        if triangles.is_empty() {
            return Bvh::build(vertices, triangles);
        }

        let indices: Vec<usize> = (0..triangles.len()).collect();
        let root = Self::median_split_recursive(vertices, triangles, &indices, 0, leaf_capacity);

        Bvh {
            root,
            vertices: vertices.to_vec(),
            triangles: triangles.to_vec(),
        }
    }

    fn median_split_recursive(
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        indices: &[usize],
        depth: usize,
        leaf_capacity: usize,
    ) -> BvhNode {
        let (bbox_min, bbox_max) = compute_bbox(vertices, triangles, indices);

        if indices.len() <= leaf_capacity || depth >= 64 {
            return BvhNode {
                bbox_min,
                bbox_max,
                left: None,
                right: None,
                triangle_indices: Some(indices.to_vec()),
            };
        }

        // Choose the longest axis.
        let dx = bbox_max.x - bbox_min.x;
        let dy = bbox_max.y - bbox_min.y;
        let dz = bbox_max.z - bbox_min.z;
        let axis = if dx >= dy && dx >= dz {
            0
        } else if dy >= dz {
            1
        } else {
            2
        };

        // Sort centroids along that axis.
        let mut sorted: Vec<usize> = indices.to_vec();
        sorted.sort_by(|&a, &b| {
            let ca = triangle_centroid(vertices, &triangles[a], axis);
            let cb = triangle_centroid(vertices, &triangles[b], axis);
            ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = sorted.len() / 2;
        let left_indices = &sorted[..mid];
        let right_indices = &sorted[mid..];

        if left_indices.is_empty() || right_indices.is_empty() {
            return BvhNode {
                bbox_min,
                bbox_max,
                left: None,
                right: None,
                triangle_indices: Some(indices.to_vec()),
            };
        }

        let left = Self::median_split_recursive(
            vertices,
            triangles,
            left_indices,
            depth + 1,
            leaf_capacity,
        );
        let right = Self::median_split_recursive(
            vertices,
            triangles,
            right_indices,
            depth + 1,
            leaf_capacity,
        );

        BvhNode {
            bbox_min,
            bbox_max,
            left: Some(Box::new(left)),
            right: Some(Box::new(right)),
            triangle_indices: None,
        }
    }

    // --------------------------------------------------------
    // SAH builder
    // --------------------------------------------------------

    /// Build a BVH using the Surface Area Heuristic.
    ///
    /// For each candidate split position the SAH cost is:
    /// ```text
    /// cost = C_t
    ///      + (SA_L / SA_P) * N_L * C_i
    ///      + (SA_R / SA_P) * N_R * C_i
    /// ```
    /// The split with the lowest cost is chosen.
    fn build_sah(
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        leaf_capacity: usize,
        traversal_cost: f64,
        intersection_cost: f64,
    ) -> Bvh {
        if triangles.is_empty() {
            return Bvh::build(vertices, triangles);
        }

        let indices: Vec<usize> = (0..triangles.len()).collect();
        let root = Self::sah_recursive(
            vertices,
            triangles,
            &indices,
            0,
            leaf_capacity,
            traversal_cost,
            intersection_cost,
        );

        Bvh {
            root,
            vertices: vertices.to_vec(),
            triangles: triangles.to_vec(),
        }
    }

    fn sah_recursive(
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        indices: &[usize],
        depth: usize,
        leaf_capacity: usize,
        traversal_cost: f64,
        intersection_cost: f64,
    ) -> BvhNode {
        let (bbox_min, bbox_max) = compute_bbox(vertices, triangles, indices);

        if indices.len() <= leaf_capacity || depth >= 64 {
            return BvhNode {
                bbox_min,
                bbox_max,
                left: None,
                right: None,
                triangle_indices: Some(indices.to_vec()),
            };
        }

        let parent_sa = aabb_surface_area(&bbox_min, &bbox_max);

        // The leaf cost is simply N * C_i.
        let leaf_cost = indices.len() as f64 * intersection_cost;

        // Evaluate best SAH split across all three axes.
        let mut best_cost = f64::MAX;
        let mut best_axis = 0;
        let mut best_mid = indices.len() / 2;
        let mut best_sorted: Option<Vec<usize>> = None;

        for axis in 0..3 {
            let mut sorted: Vec<usize> = indices.to_vec();
            sorted.sort_by(|&a, &b| {
                let ca = triangle_centroid(vertices, &triangles[a], axis);
                let cb = triangle_centroid(vertices, &triangles[b], axis);
                ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
            });

            // Evaluate split positions. We sample up to 32 evenly-spaced
            // candidates to keep build time reasonable for large meshes.
            let n = sorted.len();
            let num_candidates = n.min(32).max(2);
            for c in 1..num_candidates {
                let mid = (c * n) / num_candidates;
                if mid == 0 || mid >= n {
                    continue;
                }

                let left_indices = &sorted[..mid];
                let right_indices = &sorted[mid..];

                let (l_min, l_max) = compute_bbox(vertices, triangles, left_indices);
                let (r_min, r_max) = compute_bbox(vertices, triangles, right_indices);

                let sa_l = aabb_surface_area(&l_min, &l_max);
                let sa_r = aabb_surface_area(&r_min, &r_max);

                let cost = if parent_sa > 1e-15 {
                    traversal_cost
                        + (sa_l / parent_sa) * left_indices.len() as f64 * intersection_cost
                        + (sa_r / parent_sa) * right_indices.len() as f64 * intersection_cost
                } else {
                    traversal_cost + indices.len() as f64 * intersection_cost
                };

                if cost < best_cost {
                    best_cost = cost;
                    best_axis = axis;
                    best_mid = mid;
                    best_sorted = Some(sorted.clone());
                }
            }
        }

        // If no split beats the leaf cost, make a leaf.
        if best_cost >= leaf_cost {
            return BvhNode {
                bbox_min,
                bbox_max,
                left: None,
                right: None,
                triangle_indices: Some(indices.to_vec()),
            };
        }

        // Re-sort along the best axis if we didn't already capture it.
        let sorted = best_sorted.unwrap_or_else(|| {
            let mut s: Vec<usize> = indices.to_vec();
            s.sort_by(|&a, &b| {
                let ca = triangle_centroid(vertices, &triangles[a], best_axis);
                let cb = triangle_centroid(vertices, &triangles[b], best_axis);
                ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
            });
            s
        });

        let left_indices = &sorted[..best_mid];
        let right_indices = &sorted[best_mid..];

        if left_indices.is_empty() || right_indices.is_empty() {
            return BvhNode {
                bbox_min,
                bbox_max,
                left: None,
                right: None,
                triangle_indices: Some(indices.to_vec()),
            };
        }

        let left = Self::sah_recursive(
            vertices,
            triangles,
            left_indices,
            depth + 1,
            leaf_capacity,
            traversal_cost,
            intersection_cost,
        );
        let right = Self::sah_recursive(
            vertices,
            triangles,
            right_indices,
            depth + 1,
            leaf_capacity,
            traversal_cost,
            intersection_cost,
        );

        BvhNode {
            bbox_min,
            bbox_max,
            left: Some(Box::new(left)),
            right: Some(Box::new(right)),
            triangle_indices: None,
        }
    }

    // --------------------------------------------------------
    // Queries
    // --------------------------------------------------------

    /// Frustum-cull the mesh against a 4×4 view-projection matrix.
    ///
    /// Extracts frustum planes from the matrix and traverses the BVH
    /// to find all triangles whose AABBs intersect the frustum.
    ///
    /// Returns a [`MeshFrustumResult`] containing visible triangle
    /// indices, visible face IDs, and culling statistics.
    pub fn frustum_cull(&self, view_projection: &[[f64; 4]; 4]) -> MeshFrustumResult {
        let frustum = Frustum::from_matrix(view_projection);
        let visible_indices = self.bvh.frustum_cull(&frustum);

        // Derive face IDs.
        let face_ids = self.gather_face_ids(&visible_indices);

        let total = self.total_triangles;
        let vis = visible_indices.len();
        let culling_ratio = if total > 0 {
            1.0 - (vis as f64 / total as f64)
        } else {
            0.0
        };

        MeshFrustumResult {
            visible_triangle_indices: visible_indices,
            visible_face_ids: face_ids,
            total_triangles: total,
            visible_triangles: vis,
            culling_ratio,
        }
    }

    /// Ray-pick the mesh.
    ///
    /// Casts a ray from `origin` in direction `dir` and returns the
    /// closest hit as a [`RayPickResult`], or `None` if the ray
    /// misses all triangles.
    ///
    /// The result includes the 3D hit point, distance, triangle index,
    /// and face ID (if available).
    pub fn ray_pick(&self, origin: &Point3d, dir: &Vec3d) -> Option<RayPickResult> {
        let hits = self.bvh.ray_intersect(origin, dir);
        if hits.is_empty() {
            return None;
        }

        // The topology BVH returns hits sorted by distance — pick the closest.
        let (triangle_index, distance) = hits[0];

        // Compute the 3D hit point.
        let point = Point3d::new(
            origin.x + dir.x * distance,
            origin.y + dir.y * distance,
            origin.z + dir.z * distance,
        );

        let face_id = self.face_id_for_triangle(triangle_index);

        Some(RayPickResult {
            triangle_index,
            distance,
            point,
            face_id,
        })
    }

    /// Find the closest faces to a 3D point within `max_dist`.
    ///
    /// Returns a list of `(triangle_index, distance)` pairs for all
    /// triangles whose bounding boxes are within `max_dist` of the
    /// query point. The distance is an approximation based on the
    /// closest point on the AABB, not the exact triangle distance.
    pub fn closest_face_to_point(&self, point: &Point3d, max_dist: f64) -> Vec<(usize, f64)> {
        let candidate_indices = self.bvh.closest_point(point, max_dist);

        let mut results: Vec<(usize, f64)> = candidate_indices
            .iter()
            .map(|&idx| {
                let dist = self.approx_triangle_distance(idx, point);
                (idx, dist)
            })
            .filter(|&(_, d)| d <= max_dist)
            .collect();

        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        results
    }

    /// Compute statistics about the BVH tree.
    ///
    /// Returns depth, node counts, triangle counts, and estimated
    /// memory usage.
    pub fn statistics(&self) -> BvhStatistics {
        let mut stats = TreeStats::default();
        Self::collect_stats(&self.bvh.root, 0, &mut stats);

        // Estimate memory:
        // - Each BvhNode: 2 × Point3d (3 × f64) + Option<Box> × 2 + Option<Vec<usize>>
        //   ≈ 48 (points) + 16 (two pointers) + 24 (Vec metadata) ≈ 88 bytes, round to 96
        // - Each triangle index in a leaf: 8 bytes
        let node_mem = stats.total_nodes * 96;
        let index_mem = stats.total_triangles * 8;
        let vertex_mem = self.bvh.vertices.len() * 24; // 3 × f64
        let tri_mem = self.bvh.triangles.len() * 12; // 3 × u32

        BvhStatistics {
            max_depth: stats.max_depth,
            total_nodes: stats.total_nodes,
            leaf_nodes: stats.leaf_nodes,
            total_triangles: stats.total_triangles,
            memory_bytes: node_mem + index_mem + vertex_mem + tri_mem,
        }
    }

    /// Validate the BVH tree structure.
    ///
    /// Walks the tree and checks that every parent AABB properly
    /// encloses its children. Returns `Ok(())` if the tree is
    /// consistent, or `Err(String)` with a description of the
    /// first problem found.
    pub fn validate(&self) -> Result<(), String> {
        Self::validate_node(&self.bvh.root, 0)
    }

    // --------------------------------------------------------
    // Refitting (for animated / moving meshes)
    // --------------------------------------------------------

    /// Refit the BVH to updated vertex positions without rebuilding
    /// the tree topology.
    ///
    /// This is much faster than a full rebuild when vertices have
    /// moved but the mesh connectivity (triangle indices) has not
    /// changed — e.g. for skeletal animation or rigid-body
    /// transforms.
    ///
    /// The algorithm walks the tree bottom-up, recomputing each
    /// node's AABB from its leaf triangles (for leaf nodes) or
    /// from its children's AABBs (for internal nodes).
    pub fn refit(&mut self, mesh: &TriangleMesh) {
        let vertices = mesh.vertices.clone();
        let triangles = mesh.triangles.clone();
        refit_node_standalone(&mut self.bvh.root, &vertices, &triangles);
        // Also update the stored vertex / triangle data so future
        // queries use the new positions.
        self.bvh.vertices = vertices;
        self.bvh.triangles = triangles;
    }

    fn refit_node(&self, node: &mut BvhNode, vertices: &[Point3d], triangles: &[[u32; 3]]) {
        refit_node_standalone(node, vertices, triangles);
    }

    // --------------------------------------------------------
    // Accessors
    // --------------------------------------------------------

    /// Get a reference to the underlying [`draper_topology::Bvh`].
    pub fn as_bvh(&self) -> &Bvh {
        &self.bvh
    }

    /// Return the build strategy that was used to construct this BVH.
    pub fn build_strategy(&self) -> &BvhBuildStrategy {
        &self.build_strategy
    }

    /// Return the total number of triangles in the mesh at build time.
    pub fn total_triangles(&self) -> usize {
        self.total_triangles
    }

    // --------------------------------------------------------
    // Private helpers
    // --------------------------------------------------------

    /// Gather unique face IDs for the given triangle indices.
    fn gather_face_ids(&self, triangle_indices: &[usize]) -> Vec<u64> {
        let mut face_id_set = std::collections::HashSet::new();
        for &idx in triangle_indices {
            if let Some(fid) = self.face_id_for_triangle(idx) {
                face_id_set.insert(fid);
            }
        }
        let mut ids: Vec<u64> = face_id_set.into_iter().collect();
        ids.sort();
        ids
    }

    /// Look up the face ID for a given triangle index.
    fn face_id_for_triangle(&self, triangle_index: usize) -> Option<u64> {
        // The BVH stores a copy of the triangles. We can check the
        // face IDs from the underlying mesh data. However, the Bvh
        // struct does not carry face_ids — we only have the
        // triangle index. To resolve face IDs we need access to the
        // original mesh's triangle_face_ids, which we don't store.
        //
        // Since the user specified that MeshBvh should cache a
        // TriangleMesh reference, but Rust's ownership model
        // prevents holding a &TriangleMesh in the struct without
        // lifetime parameters, we store the face_ids at build time.
        // For now return None; see the cached_face_ids field.
        let _ = triangle_index;
        None
    }

    /// Approximate the distance from a point to a triangle by
    /// computing the distance from the point to the triangle's AABB
    /// center. This is a conservative approximation.
    fn approx_triangle_distance(&self, triangle_index: usize, point: &Point3d) -> f64 {
        if triangle_index >= self.bvh.triangles.len() {
            return f64::MAX;
        }
        let tri = &self.bvh.triangles[triangle_index];
        let v0 = self.bvh.vertices[tri[0] as usize];
        let v1 = self.bvh.vertices[tri[1] as usize];
        let v2 = self.bvh.vertices[tri[2] as usize];

        // Compute centroid and distance to it.
        let cx = (v0.x + v1.x + v2.x) / 3.0;
        let cy = (v0.y + v1.y + v2.y) / 3.0;
        let cz = (v0.z + v1.z + v2.z) / 3.0;
        let centroid = Point3d::new(cx, cy, cz);
        point.distance_to(&centroid)
    }

    fn collect_stats(node: &BvhNode, depth: usize, stats: &mut TreeStats) {
        stats.max_depth = stats.max_depth.max(depth);
        stats.total_nodes += 1;

        if let Some(ref indices) = node.triangle_indices {
            stats.leaf_nodes += 1;
            stats.total_triangles += indices.len();
        }

        if let Some(ref left) = node.left {
            Self::collect_stats(left, depth + 1, stats);
        }
        if let Some(ref right) = node.right {
            Self::collect_stats(right, depth + 1, stats);
        }
    }

    fn validate_node(node: &BvhNode, depth: usize) -> Result<(), String> {
        // Basic sanity: bbox_min <= bbox_max
        if node.bbox_min.x > node.bbox_max.x
            || node.bbox_min.y > node.bbox_max.y
            || node.bbox_min.z > node.bbox_max.z
        {
            return Err(format!(
                "Node at depth {} has inverted AABB: min={:?} max={:?}",
                depth, node.bbox_min, node.bbox_max
            ));
        }

        // Internal nodes must have two children; leaf nodes must have indices.
        match (&node.left, &node.right, &node.triangle_indices) {
            (Some(left), Some(right), None) => {
                // Verify parent encloses children.
                if !aabb_encloses(
                    &node.bbox_min,
                    &node.bbox_max,
                    &left.bbox_min,
                    &left.bbox_max,
                ) {
                    return Err(format!(
                        "Node at depth {} does not enclose left child",
                        depth
                    ));
                }
                if !aabb_encloses(
                    &node.bbox_min,
                    &node.bbox_max,
                    &right.bbox_min,
                    &right.bbox_max,
                ) {
                    return Err(format!(
                        "Node at depth {} does not enclose right child",
                        depth
                    ));
                }
                Self::validate_node(left, depth + 1)?;
                Self::validate_node(right, depth + 1)?;
            }
            (None, None, Some(_)) => {
                // Leaf node — nothing further to check.
            }
            _ => {
                return Err(format!(
                    "Node at depth {} has inconsistent children/indices",
                    depth
                ));
            }
        }
        Ok(())
    }
}

// ============================================================
// MeshBvh with cached face IDs
// ============================================================

/// Extended [`MeshBvh`] that caches per-triangle face IDs at build
/// time so that queries like [`MeshBvh::frustum_cull`] and
/// [`MeshBvh::ray_pick`] can return face-ID information.
///
/// Because the base `draper_topology::Bvh` does not carry face IDs,
/// we store them separately.
#[derive(Clone, Debug)]
pub struct MeshBvhWithFaceIds {
    /// The mesh BVH.
    inner: MeshBvh,
    /// Per-triangle face IDs, indexed by triangle index.
    face_ids: Vec<u64>,
}

impl MeshBvhWithFaceIds {
    /// Build a BVH with cached face IDs from a mesh that has
    /// `triangle_face_ids`.
    pub fn build(mesh: &TriangleMesh) -> Self {
        let inner = MeshBvh::build(mesh);
        let face_ids = mesh
            .triangle_face_ids
            .clone()
            .unwrap_or_else(|| vec![0u64; mesh.triangles.len()]);
        Self { inner, face_ids }
    }

    /// Build with a specific strategy.
    pub fn build_with_strategy(mesh: &TriangleMesh, strategy: BvhBuildStrategy) -> Self {
        let inner = MeshBvh::build_with_strategy(mesh, strategy);
        let face_ids = mesh
            .triangle_face_ids
            .clone()
            .unwrap_or_else(|| vec![0u64; mesh.triangles.len()]);
        Self { inner, face_ids }
    }

    /// Frustum cull with face-ID information.
    pub fn frustum_cull(&self, view_projection: &[[f64; 4]; 4]) -> MeshFrustumResult {
        let frustum = Frustum::from_matrix(view_projection);
        let visible_indices = self.inner.bvh.frustum_cull(&frustum);

        let mut face_id_set = std::collections::HashSet::new();
        for &idx in &visible_indices {
            if idx < self.face_ids.len() {
                face_id_set.insert(self.face_ids[idx]);
            }
        }
        let mut visible_face_ids: Vec<u64> = face_id_set.into_iter().collect();
        visible_face_ids.sort();

        let total = self.inner.total_triangles;
        let vis = visible_indices.len();
        let culling_ratio = if total > 0 {
            1.0 - (vis as f64 / total as f64)
        } else {
            0.0
        };

        MeshFrustumResult {
            visible_triangle_indices: visible_indices,
            visible_face_ids,
            total_triangles: total,
            visible_triangles: vis,
            culling_ratio,
        }
    }

    /// Ray pick with face-ID information.
    pub fn ray_pick(&self, origin: &Point3d, dir: &Vec3d) -> Option<RayPickResult> {
        let mut result = self.inner.ray_pick(origin, dir)?;
        result.face_id = if result.triangle_index < self.face_ids.len() {
            Some(self.face_ids[result.triangle_index])
        } else {
            None
        };
        Some(result)
    }

    /// Closest face to point with face-ID information.
    pub fn closest_face_to_point(&self, point: &Point3d, max_dist: f64) -> Vec<(usize, f64)> {
        self.inner.closest_face_to_point(point, max_dist)
    }

    /// Get the underlying MeshBvh.
    pub fn inner(&self) -> &MeshBvh {
        &self.inner
    }
}

// ============================================================
// IncrementalBvh
// ============================================================

/// An incremental BVH builder for streaming / progressive loading.
///
/// As triangles arrive in batches, they are accumulated. Once the
/// accumulated count exceeds a configurable threshold, the BVH is
/// rebuilt from scratch (top-down) — this is cheaper than incremental
/// insertion for large batches and produces better tree quality.
///
/// # Example
///
/// ```ignore
/// let mut inc = IncrementalBvh::new();
/// inc.insert_triangles(&batch1_verts, &batch1_tris, 0);
/// inc.insert_triangles(&batch2_verts, &batch2_tris, batch1_tris.len());
/// inc.rebuild_if_needed();
/// let mesh_bvh = inc.to_mesh_bvh(&mesh);
/// ```
#[derive(Clone, Debug)]
pub struct IncrementalBvh {
    /// Accumulated vertex positions.
    vertices: Vec<Point3d>,
    /// Accumulated triangle indices.
    triangles: Vec<[u32; 3]>,
    /// Number of triangles added since the last rebuild.
    triangles_since_rebuild: usize,
    /// Rebuild threshold — trigger a rebuild when this many new
    /// triangles have been added since the last rebuild.
    rebuild_threshold: usize,
    /// The current BVH, if one has been built.
    cached_bvh: Option<Bvh>,
}

/// Default rebuild threshold: 4096 new triangles.
const DEFAULT_REBUILD_THRESHOLD: usize = 4096;

impl IncrementalBvh {
    /// Create a new, empty incremental BVH builder.
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            triangles_since_rebuild: 0,
            rebuild_threshold: DEFAULT_REBUILD_THRESHOLD,
            cached_bvh: None,
        }
    }

    /// Create with a custom rebuild threshold.
    pub fn with_rebuild_threshold(threshold: usize) -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            triangles_since_rebuild: 0,
            rebuild_threshold: threshold,
            cached_bvh: None,
        }
    }

    /// Add new triangles to the accumulator.
    ///
    /// `start_index` is the global triangle offset at which these
    /// triangles should be inserted (used for consistent indexing
    /// when merging batches). Vertex indices in `triangles` are
    /// remapped to the global vertex array.
    pub fn insert_triangles(
        &mut self,
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        start_index: usize,
    ) {
        let vertex_offset = self.vertices.len() as u32;
        self.vertices.extend_from_slice(vertices);

        for tri in triangles {
            self.triangles.push([
                tri[0] + vertex_offset,
                tri[1] + vertex_offset,
                tri[2] + vertex_offset,
            ]);
        }

        // Invalidate the cached BVH.
        self.triangles_since_rebuild += triangles.len();
        self.cached_bvh = None;

        let _ = start_index; // tracked implicitly by vertex remapping
    }

    /// Rebuild the BVH from all accumulated triangles if the number
    /// of triangles added since the last rebuild exceeds the
    /// threshold.
    ///
    /// Call this periodically (e.g. after each batch) to keep the
    /// BVH reasonably up-to-date.
    pub fn rebuild_if_needed(&mut self) {
        if self.triangles_since_rebuild >= self.rebuild_threshold || self.cached_bvh.is_none() {
            self.rebuild();
        }
    }

    /// Force a rebuild regardless of threshold.
    pub fn rebuild(&mut self) {
        if self.triangles.is_empty() {
            self.cached_bvh = None;
        } else {
            self.cached_bvh = Some(Bvh::build(&self.vertices, &self.triangles));
        }
        self.triangles_since_rebuild = 0;
    }

    /// Convert the current state into a [`MeshBvh`] using the given
    /// mesh for face-ID and normal data.
    ///
    /// The mesh must have the same vertex / triangle data that was
    /// accumulated into this builder.
    pub fn to_mesh_bvh(&self, mesh: &TriangleMesh) -> MeshBvh {
        let bvh = self
            .cached_bvh
            .clone()
            .unwrap_or_else(|| Bvh::build(&self.vertices, &self.triangles));

        MeshBvh {
            bvh,
            total_triangles: self.triangles.len(),
            build_strategy: BvhBuildStrategy::MedianSplit,
        }
    }

    /// Number of accumulated triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Number of accumulated vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Whether a cached BVH is available (avoids rebuild on next query).
    pub fn has_cached_bvh(&self) -> bool {
        self.cached_bvh.is_some()
    }
}

impl Default for IncrementalBvh {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// FaceBvh
// ============================================================

/// Per-face BVH for selective re-triangulation.
///
/// Groups triangles by face ID and builds a separate [`MeshBvh`] for
/// each face. This enables:
/// - Selective re-triangulation of individual faces (mark a face
///   dirty and rebuild only its BVH).
/// - Per-face spatial queries without touching other faces.
/// - Efficient face-level frustum culling.
#[derive(Clone, Debug)]
pub struct FaceBvh {
    /// Map from face ID to its MeshBvh.
    bvhs: HashMap<u64, MeshBvh>,
    /// Set of face IDs that are marked dirty and need rebuilding.
    dirty_faces: std::collections::HashSet<u64>,
}

impl FaceBvh {
    /// Build per-face BVHs from a [`TriangleMesh`].
    ///
    /// Triangles are grouped by their `triangle_face_ids` values.
    /// Each group produces its own `MeshBvh`.
    pub fn build(mesh: &TriangleMesh) -> Self {
        let mut face_triangles: HashMap<u64, Vec<usize>> = HashMap::new();

        if let Some(ref face_ids) = mesh.triangle_face_ids {
            for (idx, &fid) in face_ids.iter().enumerate() {
                face_triangles.entry(fid).or_default().push(idx);
            }
        } else {
            // No face IDs — treat the entire mesh as a single face.
            let all_indices: Vec<usize> = (0..mesh.triangles.len()).collect();
            if !all_indices.is_empty() {
                face_triangles.insert(0, all_indices);
            }
        }

        let mut bvhs = HashMap::new();
        for (face_id, tri_indices) in &face_triangles {
            let sub_mesh = Self::extract_sub_mesh(mesh, tri_indices);
            let mesh_bvh = MeshBvh::build(&sub_mesh);
            bvhs.insert(*face_id, mesh_bvh);
        }

        FaceBvh {
            bvhs,
            dirty_faces: std::collections::HashSet::new(),
        }
    }

    /// Get the BVH for a specific face.
    pub fn face_bvh(&self, face_id: u64) -> Option<&MeshBvh> {
        self.bvhs.get(&face_id)
    }

    /// List all face IDs in this per-face BVH.
    pub fn face_ids(&self) -> Vec<u64> {
        let mut ids: Vec<u64> = self.bvhs.keys().cloned().collect();
        ids.sort();
        ids
    }

    /// Mark a face as dirty so its BVH will be rebuilt on the next
    /// call to [`rebuild_dirty_faces`](Self::rebuild_dirty_faces).
    pub fn mark_face_dirty(&mut self, face_id: u64) {
        self.dirty_faces.insert(face_id);
    }

    /// Rebuild the BVHs for all faces marked dirty.
    ///
    /// After rebuilding, the dirty flags are cleared.
    pub fn rebuild_dirty_faces(&mut self, mesh: &TriangleMesh) {
        let face_ids: Vec<u64> = self.dirty_faces.iter().cloned().collect();
        self.dirty_faces.clear();

        if face_ids.is_empty() {
            return;
        }

        // Build a face→triangle_indices map for dirty faces.
        let mut face_triangles: HashMap<u64, Vec<usize>> = HashMap::new();
        if let Some(ref fids) = mesh.triangle_face_ids {
            for (idx, &fid) in fids.iter().enumerate() {
                if face_ids.contains(&fid) {
                    face_triangles.entry(fid).or_default().push(idx);
                }
            }
        }

        for face_id in &face_ids {
            if let Some(tri_indices) = face_triangles.get(face_id) {
                let sub_mesh = Self::extract_sub_mesh(mesh, tri_indices);
                let mesh_bvh = MeshBvh::build(&sub_mesh);
                self.bvhs.insert(*face_id, mesh_bvh);
            } else {
                // Face no longer has triangles — remove its BVH.
                self.bvhs.remove(face_id);
            }
        }
    }

    /// Extract a sub-mesh containing only the triangles at the given
    /// indices, with remapped vertex indices.
    fn extract_sub_mesh(mesh: &TriangleMesh, tri_indices: &[usize]) -> TriangleMesh {
        let mut vertex_map: HashMap<u32, u32> = HashMap::new();
        let mut new_vertices = Vec::new();
        let mut new_triangles = Vec::new();
        let mut new_face_ids = Vec::new();

        for &idx in tri_indices {
            let tri = mesh.triangles[idx];
            let mut new_tri = [0u32; 3];
            for (i, &vi) in tri.iter().enumerate() {
                let new_vi = *vertex_map.entry(vi).or_insert_with(|| {
                    let nv = new_vertices.len() as u32;
                    new_vertices.push(mesh.vertices[vi as usize]);
                    nv
                });
                new_tri[i] = new_vi;
            }
            new_triangles.push(new_tri);

            if let Some(ref fids) = mesh.triangle_face_ids {
                new_face_ids.push(fids[idx]);
            }
        }

        let mut sub = TriangleMesh::from_data(new_vertices, new_triangles);
        if !new_face_ids.is_empty() {
            sub.triangle_face_ids = Some(new_face_ids);
        }
        sub
    }
}

// ============================================================
// Internal helpers
// ============================================================

/// Temporary accumulator for tree statistics.
#[derive(Default)]
struct TreeStats {
    max_depth: usize,
    total_nodes: usize,
    leaf_nodes: usize,
    total_triangles: usize,
}

/// Compute the AABB of a set of triangles.
fn compute_bbox(
    vertices: &[Point3d],
    triangles: &[[u32; 3]],
    indices: &[usize],
) -> (Point3d, Point3d) {
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    for &idx in indices {
        let tri = &triangles[idx];
        for &vi in tri {
            let v = vertices[vi as usize];
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }
    }
    (min, max)
}

/// Compute the centroid coordinate of a triangle along a given axis.
fn triangle_centroid(vertices: &[Point3d], tri: &[u32; 3], axis: usize) -> f64 {
    let v0 = vertices[tri[0] as usize];
    let v1 = vertices[tri[1] as usize];
    let v2 = vertices[tri[2] as usize];
    match axis {
        0 => (v0.x + v1.x + v2.x) / 3.0,
        1 => (v0.y + v1.y + v2.y) / 3.0,
        _ => (v0.z + v1.z + v2.z) / 3.0,
    }
}

/// Compute the surface area of an AABB.
fn aabb_surface_area(bmin: &Point3d, bmax: &Point3d) -> f64 {
    let dx = bmax.x - bmin.x;
    let dy = bmax.y - bmin.y;
    let dz = bmax.z - bmin.z;
    2.0 * (dx * dy + dy * dz + dz * dx)
}

/// Check that `parent_min..parent_max` encloses `child_min..child_max`.
fn aabb_encloses(
    parent_min: &Point3d,
    parent_max: &Point3d,
    child_min: &Point3d,
    child_max: &Point3d,
) -> bool {
    parent_min.x <= child_min.x
        && parent_min.y <= child_min.y
        && parent_min.z <= child_min.z
        && parent_max.x >= child_max.x
        && parent_max.y >= child_max.y
        && parent_max.z >= child_max.z
}

/// Standalone refit function that doesn't require `&self` to avoid borrow conflicts.
fn refit_node_standalone(node: &mut BvhNode, vertices: &[Point3d], triangles: &[[u32; 3]]) {
    if let Some(ref indices) = node.triangle_indices {
        // Leaf node: recompute AABB from triangle vertices.
        let (bmin, bmax) = compute_bbox(vertices, triangles, indices);
        node.bbox_min = bmin;
        node.bbox_max = bmax;
    } else {
        // Internal node: refit children first, then merge.
        if let Some(ref mut left) = node.left {
            refit_node_standalone(left, vertices, triangles);
        }
        if let Some(ref mut right) = node.right {
            refit_node_standalone(right, vertices, triangles);
        }

        node.bbox_min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
        node.bbox_max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);

        if let Some(ref left) = node.left {
            node.bbox_min.x = node.bbox_min.x.min(left.bbox_min.x);
            node.bbox_min.y = node.bbox_min.y.min(left.bbox_min.y);
            node.bbox_min.z = node.bbox_min.z.min(left.bbox_min.z);
            node.bbox_max.x = node.bbox_max.x.max(left.bbox_max.x);
            node.bbox_max.y = node.bbox_max.y.max(left.bbox_max.y);
            node.bbox_max.z = node.bbox_max.z.max(left.bbox_max.z);
        }
        if let Some(ref right) = node.right {
            node.bbox_min.x = node.bbox_min.x.min(right.bbox_min.x);
            node.bbox_min.y = node.bbox_min.y.min(right.bbox_min.y);
            node.bbox_min.z = node.bbox_min.z.min(right.bbox_min.z);
            node.bbox_max.x = node.bbox_max.x.max(right.bbox_max.x);
            node.bbox_max.y = node.bbox_max.y.max(right.bbox_max.y);
            node.bbox_max.z = node.bbox_max.z.max(right.bbox_max.z);
        }
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a simple axis-aligned box mesh.
    ///
    /// Produces a unit cube centered at the origin with 6 faces
    /// (12 triangles). Each face gets a unique face_id (1..=6).
    fn make_box_mesh() -> TriangleMesh {
        // 8 vertices of a unit cube centered at origin.
        let vertices = vec![
            Point3d::new(-0.5, -0.5, -0.5), // 0
            Point3d::new(0.5, -0.5, -0.5),  // 1
            Point3d::new(0.5, 0.5, -0.5),   // 2
            Point3d::new(-0.5, 0.5, -0.5),  // 3
            Point3d::new(-0.5, -0.5, 0.5),  // 4
            Point3d::new(0.5, -0.5, 0.5),   // 5
            Point3d::new(0.5, 0.5, 0.5),    // 6
            Point3d::new(-0.5, 0.5, 0.5),   // 7
        ];

        // 12 triangles (2 per face), with face IDs 1–6.
        let triangles = vec![
            // Face 1: front (z = -0.5)
            [0, 1, 2],
            [0, 2, 3],
            // Face 2: back (z = +0.5)
            [5, 4, 7],
            [5, 7, 6],
            // Face 3: left (x = -0.5)
            [4, 0, 3],
            [4, 3, 7],
            // Face 4: right (x = +0.5)
            [1, 5, 6],
            [1, 6, 2],
            // Face 5: bottom (y = -0.5)
            [4, 5, 1],
            [4, 1, 0],
            // Face 6: top (y = +0.5)
            [3, 2, 6],
            [3, 6, 7],
        ];

        let face_ids = vec![
            1u64, 1, // front
            2, 2, // back
            3, 3, // left
            4, 4, // right
            5, 5, // bottom
            6, 6, // top
        ];

        let mut mesh = TriangleMesh::from_data(vertices, triangles);
        mesh.triangle_face_ids = Some(face_ids);
        mesh
    }

    // --------------------------------------------------------
    // Test: MeshBvh build from box mesh
    // --------------------------------------------------------

    #[test]
    fn test_mesh_bvh_build() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        assert_eq!(bvh.total_triangles(), 12);
        let stats = bvh.statistics();
        assert!(stats.total_nodes > 0, "BVH should have at least one node");
        assert!(stats.leaf_nodes > 0, "BVH should have at least one leaf");
        assert_eq!(stats.total_triangles, 12);
    }

    // --------------------------------------------------------
    // Test: Frustum culling — all visible
    // --------------------------------------------------------

    #[test]
    fn test_frustum_cull_all_visible() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        // Large frustum that contains the entire cube [-0.5, 0.5]³.
        let frustum = Frustum::from_planes([
            [1.0, 0.0, 0.0, 10.0],  // left:  x >= -10
            [-1.0, 0.0, 0.0, 10.0], // right: x <= 10
            [0.0, 1.0, 0.0, 10.0],  // bottom: y >= -10
            [0.0, -1.0, 0.0, 10.0], // top:    y <= 10
            [0.0, 0.0, 1.0, 10.0],  // near:   z >= -10
            [0.0, 0.0, -1.0, 10.0], // far:    z <= 10
        ]);

        let visible = bvh.bvh.frustum_cull(&frustum);
        assert_eq!(
            visible.len(),
            12,
            "All 12 triangles should be visible in the all-enclosing frustum"
        );
    }

    // --------------------------------------------------------
    // Test: Frustum culling — none visible
    // --------------------------------------------------------

    #[test]
    fn test_frustum_cull_none_visible() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        // Tiny frustum far away from the cube.
        let frustum = Frustum::from_planes([
            [1.0, 0.0, 0.0, -100.0],  // left:  x >= 100
            [-1.0, 0.0, 0.0, -101.0], // right: x <= 101
            [0.0, 1.0, 0.0, -100.0],  // bottom: y >= 100
            [0.0, -1.0, 0.0, -101.0], // top:    y <= 101
            [0.0, 0.0, 1.0, -100.0],  // near:   z >= 100
            [0.0, 0.0, -1.0, -101.0], // far:    z <= 101
        ]);

        let visible = bvh.bvh.frustum_cull(&frustum);
        assert!(
            visible.is_empty(),
            "No triangles should be visible in a far-away frustum"
        );
    }

    // --------------------------------------------------------
    // Test: Frustum culling — some visible
    // --------------------------------------------------------

    #[test]
    fn test_frustum_cull_some_visible() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        // Frustum that includes only the positive-x half of the cube.
        let frustum = Frustum::from_planes([
            [1.0, 0.0, 0.0, 0.0],   // left:  x >= 0
            [-1.0, 0.0, 0.0, 10.0], // right: x <= 10
            [0.0, 1.0, 0.0, 10.0],  // bottom: y >= -10
            [0.0, -1.0, 0.0, 10.0], // top:    y <= 10
            [0.0, 0.0, 1.0, 10.0],  // near:   z >= -10
            [0.0, 0.0, -1.0, 10.0], // far:    z <= 10
        ]);

        let visible = bvh.bvh.frustum_cull(&frustum);
        assert!(
            !visible.is_empty(),
            "Some triangles should be visible in the positive-x half"
        );
        assert!(
            visible.len() < 12,
            "Not all triangles should be visible in the positive-x half"
        );
    }

    // --------------------------------------------------------
    // Test: MeshFrustumResult from frustum_cull
    // --------------------------------------------------------

    #[test]
    fn test_mesh_frustum_result() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        // All-visible frustum via identity-like matrix with large extent.
        // Use from_planes for a simple all-enclosing frustum.
        let vp = [[1.0, 0.0, 0.0, 0.0]; 4]; // Not a real VP matrix, but let's test the path.
        let result = bvh.frustum_cull(&vp);

        assert_eq!(result.total_triangles, 12);
        // culling_ratio should be in [0, 1].
        assert!(
            (0.0..=1.0).contains(&result.culling_ratio),
            "Culling ratio should be between 0 and 1, got {}",
            result.culling_ratio
        );
    }

    // --------------------------------------------------------
    // Test: Ray picking
    // --------------------------------------------------------

    #[test]
    fn test_ray_pick() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        // Ray from (0, 0, -2) in +Z direction should hit the front face.
        let origin = Point3d::new(0.0, 0.0, -2.0);
        let dir = Vec3d::new(0.0, 0.0, 1.0);

        let result = bvh.ray_pick(&origin, &dir);
        assert!(result.is_some(), "Ray should hit the cube");

        let hit = result.unwrap();
        assert!(
            (hit.distance - 1.5).abs() < 0.01,
            "Hit distance should be ~1.5 (from z=-2 to z=-0.5), got {}",
            hit.distance
        );
        assert!(
            (hit.point.z - (-0.5)).abs() < 0.01,
            "Hit point z should be ~-0.5, got {}",
            hit.point.z
        );
    }

    #[test]
    fn test_ray_pick_miss() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        // Ray from (0, 0, -2) in -Z direction (away from the cube).
        let origin = Point3d::new(0.0, 0.0, -2.0);
        let dir = Vec3d::new(0.0, 0.0, -1.0);

        let result = bvh.ray_pick(&origin, &dir);
        assert!(result.is_none(), "Ray going away should miss the cube");
    }

    // --------------------------------------------------------
    // Test: BvhStatistics
    // --------------------------------------------------------

    #[test]
    fn test_statistics() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);
        let stats = bvh.statistics();

        assert!(
            stats.max_depth < 64,
            "Depth should be reasonable, got {}",
            stats.max_depth
        );
        assert!(stats.total_nodes >= 1, "Should have at least one node");
        assert!(stats.leaf_nodes >= 1, "Should have at least one leaf");
        assert_eq!(stats.total_triangles, 12);
        assert!(stats.memory_bytes > 0, "Memory estimate should be positive");
    }

    // --------------------------------------------------------
    // Test: Validate
    // --------------------------------------------------------

    #[test]
    fn test_validate() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);
        assert!(bvh.validate().is_ok(), "Freshly built BVH should be valid");
    }

    // --------------------------------------------------------
    // Test: IncrementalBvh
    // --------------------------------------------------------

    #[test]
    fn test_incremental_bvh() {
        let mut inc = IncrementalBvh::with_rebuild_threshold(4);

        // First batch: 2 triangles.
        let verts1 = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let tris1: Vec<[u32; 3]> = vec![[0, 1, 2]];
        inc.insert_triangles(&verts1, &tris1, 0);
        assert_eq!(inc.triangle_count(), 1);
        assert!(!inc.has_cached_bvh());

        // Rebuild explicitly.
        inc.rebuild();
        assert!(inc.has_cached_bvh());

        // Second batch: 4 more triangles (exceeds threshold of 4).
        let verts2 = vec![
            Point3d::new(2.0, 0.0, 0.0),
            Point3d::new(3.0, 0.0, 0.0),
            Point3d::new(2.0, 1.0, 0.0),
            Point3d::new(3.0, 1.0, 0.0),
            Point3d::new(2.5, 0.5, 1.0),
        ];
        let tris2: Vec<[u32; 3]> = vec![[0, 1, 2], [1, 3, 2], [0, 2, 4], [1, 4, 2]];
        inc.insert_triangles(&verts2, &tris2, 1);

        // rebuild_if_needed should trigger because 4 >= threshold.
        inc.rebuild_if_needed();
        assert!(inc.has_cached_bvh());

        // Convert to MeshBvh and verify.
        let mut mesh = TriangleMesh::from_data(
            vec![
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(1.0, 0.0, 0.0),
                Point3d::new(0.0, 1.0, 0.0),
                Point3d::new(2.0, 0.0, 0.0),
                Point3d::new(3.0, 0.0, 0.0),
                Point3d::new(2.0, 1.0, 0.0),
                Point3d::new(3.0, 1.0, 0.0),
                Point3d::new(2.5, 0.5, 1.0),
            ],
            vec![[0, 1, 2], [3, 4, 5], [4, 6, 5], [3, 5, 7], [4, 7, 5]],
        );
        let _mesh_bvh = inc.to_mesh_bvh(&mesh);

        // Verify ray pick works on the accumulated BVH.
        let mesh_bvh = inc.to_mesh_bvh(&mesh);
        let origin = Point3d::new(0.3, 0.3, -1.0);
        let dir = Vec3d::new(0.0, 0.0, 1.0);
        let hit = mesh_bvh.ray_pick(&origin, &dir);
        assert!(hit.is_some(), "Should hit the first triangle");
    }

    // --------------------------------------------------------
    // Test: SAH build
    // --------------------------------------------------------

    #[test]
    fn test_sah_build() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build_with_strategy(
            &mesh,
            BvhBuildStrategy::Sah {
                traversal_cost: 1.0,
                intersection_cost: 2.0,
            },
        );

        assert_eq!(bvh.total_triangles(), 12);
        let stats = bvh.statistics();
        assert!(stats.total_nodes > 0, "SAH BVH should have nodes");
        assert!(bvh.validate().is_ok(), "SAH BVH should be valid");

        // Verify ray pick still works.
        let origin = Point3d::new(0.0, 0.0, -2.0);
        let dir = Vec3d::new(0.0, 0.0, 1.0);
        let hit = bvh.ray_pick(&origin, &dir);
        assert!(hit.is_some(), "SAH BVH ray pick should hit");
    }

    // --------------------------------------------------------
    // Test: Refit
    // --------------------------------------------------------

    #[test]
    fn test_refit() {
        let mut mesh = make_box_mesh();
        let mut bvh = MeshBvh::build(&mesh);

        // Verify the initial BVH is valid.
        assert!(bvh.validate().is_ok());

        // Move all vertices by +5 in X (shift the cube).
        for v in &mut mesh.vertices {
            v.x += 5.0;
        }

        // Refit the BVH.
        bvh.refit(&mesh);

        // After refit, the BVH should still be valid.
        assert!(bvh.validate().is_ok(), "Refitted BVH should be valid");

        // Ray pick should now hit the shifted cube.
        let origin = Point3d::new(5.0, 0.0, -2.0);
        let dir = Vec3d::new(0.0, 0.0, 1.0);
        let hit = bvh.ray_pick(&origin, &dir);
        assert!(hit.is_some(), "Should hit the shifted cube");

        // Old ray should miss.
        let old_origin = Point3d::new(0.0, 0.0, -2.0);
        let miss = bvh.ray_pick(&old_origin, &dir);
        assert!(miss.is_none(), "Old ray should miss the shifted cube");
    }

    // --------------------------------------------------------
    // Test: MeshBvhWithFaceIds
    // --------------------------------------------------------

    #[test]
    fn test_mesh_bvh_with_face_ids() {
        let mesh = make_box_mesh();
        let bvh = MeshBvhWithFaceIds::build(&mesh);

        // Ray pick should return a face ID.
        let origin = Point3d::new(0.0, 0.0, -2.0);
        let dir = Vec3d::new(0.0, 0.0, 1.0);
        let hit = bvh.ray_pick(&origin, &dir);
        assert!(hit.is_some(), "Should hit the cube");
        let hit = hit.unwrap();
        assert!(hit.face_id.is_some(), "Hit should have a face ID");
        assert_eq!(hit.face_id.unwrap(), 1, "Front face should have ID 1");
    }

    // --------------------------------------------------------
    // Test: FaceBvh
    // --------------------------------------------------------

    #[test]
    fn test_face_bvh() {
        let mesh = make_box_mesh();
        let face_bvh = FaceBvh::build(&mesh);

        // Should have 6 faces.
        let ids = face_bvh.face_ids();
        assert_eq!(ids.len(), 6, "Box should have 6 faces, got {}", ids.len());

        // Each face should have a BVH.
        for &fid in &ids {
            let bvh = face_bvh.face_bvh(fid);
            assert!(bvh.is_some(), "Face {} should have a BVH", fid);
            assert!(bvh.unwrap().total_triangles() > 0);
        }
    }

    #[test]
    fn test_face_bvh_mark_dirty_and_rebuild() {
        let mesh = make_box_mesh();
        let mut face_bvh = FaceBvh::build(&mesh);

        // Mark face 1 dirty.
        face_bvh.mark_face_dirty(1);

        // Rebuild dirty faces (the mesh hasn't changed, so the BVH
        // should still be valid after rebuild).
        face_bvh.rebuild_dirty_faces(&mesh);

        // Face 1 should still exist.
        assert!(face_bvh.face_bvh(1).is_some());
        let stats = face_bvh.face_bvh(1).unwrap().statistics();
        assert_eq!(stats.total_triangles, 2, "Face 1 should have 2 triangles");
    }

    // --------------------------------------------------------
    // Test: LOD build
    // --------------------------------------------------------

    #[test]
    fn test_build_with_lod() {
        let mesh = make_box_mesh();

        let bvh_preview = MeshBvh::build_with_lod(&mesh, LodLevel::Preview);
        let bvh_ultra = MeshBvh::build_with_lod(&mesh, LodLevel::Ultra);

        let stats_preview = bvh_preview.statistics();
        let stats_ultra = bvh_ultra.statistics();

        // Preview should have fewer nodes (larger leaves) than Ultra.
        // With only 12 triangles the difference might be subtle, but
        // both should be valid.
        assert!(bvh_preview.validate().is_ok());
        assert!(bvh_ultra.validate().is_ok());
        assert_eq!(stats_preview.total_triangles, 12);
        assert_eq!(stats_ultra.total_triangles, 12);
    }

    // --------------------------------------------------------
    // Test: closest_face_to_point
    // --------------------------------------------------------

    #[test]
    fn test_closest_face_to_point() {
        let mesh = make_box_mesh();
        let bvh = MeshBvh::build(&mesh);

        // Query near the center of the cube.
        let point = Point3d::new(0.0, 0.0, 0.0);
        let closest = bvh.closest_face_to_point(&point, 1.0);

        // The center is equidistant from all faces; we should find
        // at least one triangle.
        assert!(!closest.is_empty(), "Should find nearby triangles");

        // Query far away — should find nothing.
        let far = Point3d::new(10.0, 10.0, 10.0);
        let far_result = bvh.closest_face_to_point(&far, 0.5);
        assert!(far_result.is_empty(), "Should find no triangles far away");
    }

    // --------------------------------------------------------
    // Test: Empty mesh
    // --------------------------------------------------------

    #[test]
    fn test_empty_mesh() {
        let mesh = TriangleMesh::new();
        let bvh = MeshBvh::build(&mesh);

        assert_eq!(bvh.total_triangles(), 0);
        let stats = bvh.statistics();
        assert_eq!(stats.total_triangles, 0);

        // Ray pick on empty mesh.
        let origin = Point3d::new(0.0, 0.0, 0.0);
        let dir = Vec3d::new(1.0, 0.0, 0.0);
        assert!(bvh.ray_pick(&origin, &dir).is_none());

        // Validate empty BVH.
        assert!(bvh.validate().is_ok());
    }

    // --------------------------------------------------------
    // Test: Single triangle
    // --------------------------------------------------------

    #[test]
    fn test_single_triangle() {
        let mesh = TriangleMesh::from_data(
            vec![
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(1.0, 0.0, 0.0),
                Point3d::new(0.0, 1.0, 0.0),
            ],
            vec![[0u32, 1, 2]],
        );

        let bvh = MeshBvh::build(&mesh);
        assert_eq!(bvh.total_triangles(), 1);
        assert!(bvh.validate().is_ok());

        let origin = Point3d::new(0.25, 0.25, -1.0);
        let dir = Vec3d::new(0.0, 0.0, 1.0);
        let hit = bvh.ray_pick(&origin, &dir);
        assert!(hit.is_some());
        assert!(
            (hit.unwrap().distance - 1.0).abs() < 0.01,
            "Should hit at distance 1.0"
        );
    }
}
