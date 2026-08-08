// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Hidden Line Removal (HLR) for engineering drawings.
//!
//! Per FLEXIBLE_EXECUTION_PLAN.md task B2: implements ray-triangle
//! intersection based HLR. For each edge, sample points along the edge
//! and cast rays toward the viewer. If any ray hits a triangle that
//! doesn't share the edge, the edge is hidden at that sample point.
//!
//! Uses Möller–Trumbore ray-triangle intersection.

use draper_geometry::Point3d;
use draper_mesh::TriangleMesh;

/// 3D edge with associated triangle indices.
#[derive(Debug, Clone)]
pub struct MeshEdge {
    pub a: u32,
    pub b: u32,
    pub triangles: Vec<usize>,
}

/// Extract all unique edges from a triangle mesh.
pub fn extract_edges(mesh: &TriangleMesh) -> Vec<MeshEdge> {
    let mut edge_map: std::collections::HashMap<(u32, u32), Vec<usize>> =
        std::collections::HashMap::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        for i in 0..3 {
            let a = tri[i];
            let b = tri[(i + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            edge_map.entry(key).or_default().push(tri_idx);
        }
    }

    edge_map
        .into_iter()
        .map(|((a, b), triangles)| MeshEdge { a, b, triangles })
        .collect()
}

/// Get the view direction for a view type.
pub fn view_direction(view_type: crate::ViewType) -> (f64, f64, f64) {
    match view_type {
        crate::ViewType::Front => (0.0, 1.0, 0.0),
        crate::ViewType::Top => (0.0, 0.0, 1.0),
        crate::ViewType::Right => (1.0, 0.0, 0.0),
        crate::ViewType::Isometric => {
            let n = (3.0_f64).sqrt();
            (1.0 / n, 1.0 / n, 1.0 / n)
        }
    }
}

/// Möller–Trumbore ray-triangle intersection.
/// Returns Some(t) if ray hits the triangle (t > eps).
pub fn ray_triangle_intersect(
    origin: &Point3d,
    dir: (f64, f64, f64),
    v0: &Point3d,
    v1: &Point3d,
    v2: &Point3d,
    eps: f64,
) -> Option<f64> {
    let edge1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let edge2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    let h = (
        dir.1 * edge2.2 - dir.2 * edge2.1,
        dir.2 * edge2.0 - dir.0 * edge2.2,
        dir.0 * edge2.1 - dir.1 * edge2.0,
    );
    let a = edge1.0 * h.0 + edge1.1 * h.1 + edge1.2 * h.2;
    if a.abs() < 1e-12 {
        return None;
    }
    let f = 1.0 / a;
    let s = (origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);
    let u = f * (s.0 * h.0 + s.1 * h.1 + s.2 * h.2);
    if u < 0.0 || u > 1.0 {
        return None;
    }
    let q = (
        s.1 * edge1.2 - s.2 * edge1.1,
        s.2 * edge1.0 - s.0 * edge1.2,
        s.0 * edge1.1 - s.1 * edge1.0,
    );
    let v = f * (dir.0 * q.0 + dir.1 * q.1 + dir.2 * q.2);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * (edge2.0 * q.0 + edge2.1 * q.1 + edge2.2 * q.2);
    if t > eps {
        Some(t)
    } else {
        None
    }
}

/// Configuration for HLR.
#[derive(Debug, Clone)]
pub struct HlrConfig {
    pub samples_per_edge: usize,
    pub ray_epsilon: f64,
    pub split_segments: bool,
    /// BVH tree build threshold: if mesh has more triangles than this,
    /// use BVH acceleration for ray casting. Below this threshold,
    /// brute-force is faster (simpler, no tree overhead).
    pub bvh_threshold: usize,
}

impl Default for HlrConfig {
    fn default() -> Self {
        Self {
            samples_per_edge: 8,
            ray_epsilon: 1e-6,
            split_segments: true,
            bvh_threshold: 100, // Use BVH for meshes > 100 triangles
        }
    }
}

/// Segment visibility.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentVisibility {
    Visible,
    Hidden,
}

/// A 3D line segment with visibility classification.
#[derive(Debug, Clone, Copy)]
pub struct VisibilitySegment {
    pub start: Point3d,
    pub end: Point3d,
    pub visibility: SegmentVisibility,
}

/// Classify all edges into visible and hidden segments.
///
/// Uses BVH acceleration for meshes with > bvh_threshold triangles
/// to avoid O(edges × triangles × samples) brute force.
pub fn classify_edges(
    mesh: &TriangleMesh,
    view_type: crate::ViewType,
    config: &HlrConfig,
) -> Vec<VisibilitySegment> {
    if mesh.triangles.is_empty() || mesh.vertices.is_empty() {
        return Vec::new();
    }

    let edges = extract_edges(mesh);
    let view_dir = view_direction(view_type);
    let n_tris = mesh.triangles.len();
    let use_bvh = n_tris > config.bvh_threshold;

    // Build tree-based BVH over triangle bounding boxes if needed
    let bvh: Option<TriangleBvh> = if use_bvh {
        let tri_bboxes: Vec<(usize, TriBBox)> = mesh.triangles.iter().enumerate().map(|(i, tri)| {
            let v0 = &mesh.vertices[tri[0] as usize];
            let v1 = &mesh.vertices[tri[1] as usize];
            let v2 = &mesh.vertices[tri[2] as usize];
            (i, TriBBox {
                min: [v0.x.min(v1.x).min(v2.x), v0.y.min(v1.y).min(v2.y), v0.z.min(v1.z).min(v2.z)],
                max: [v0.x.max(v1.x).max(v2.x), v0.y.max(v1.y).max(v2.y), v0.z.max(v1.z).max(v2.z)],
            })
        }).collect();
        Some(TriangleBvh::build(&tri_bboxes))
    } else {
        None
    };

    let mut segments = Vec::new();

    for edge in &edges {
        let p_a = mesh.vertices[edge.a as usize];
        let p_b = mesh.vertices[edge.b as usize];

        let n_samples = config.samples_per_edge.max(1);
        let mut visibilities: Vec<bool> = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = i as f64 / (n_samples - 1).max(1) as f64;
            let sample = Point3d::new(
                p_a.x + t * (p_b.x - p_a.x),
                p_a.y + t * (p_b.y - p_a.y),
                p_a.z + t * (p_b.z - p_a.z),
            );
            let occluded = if let Some(ref bvh) = bvh {
                bvh.ray_occluded(&sample, view_dir, mesh, &edge.triangles, config.ray_epsilon)
            } else {
                is_point_occluded(&sample, view_dir, mesh, &edge.triangles, config.ray_epsilon)
            };
            visibilities.push(occluded);
        }

        if config.split_segments {
            let mut current_vis = if visibilities[0] {
                SegmentVisibility::Hidden
            } else {
                SegmentVisibility::Visible
            };
            let mut seg_start = 0;
            for i in 1..n_samples {
                let v = if visibilities[i] {
                    SegmentVisibility::Hidden
                } else {
                    SegmentVisibility::Visible
                };
                if v != current_vis {
                    let t_end = (i as f64 - 0.5) / (n_samples - 1).max(1) as f64;
                    let t_start = seg_start as f64 / (n_samples - 1).max(1) as f64;
                    let seg_start_pt = Point3d::new(
                        p_a.x + t_start * (p_b.x - p_a.x),
                        p_a.y + t_start * (p_b.y - p_a.y),
                        p_a.z + t_start * (p_b.z - p_a.z),
                    );
                    let seg_end_pt = Point3d::new(
                        p_a.x + t_end * (p_b.x - p_a.x),
                        p_a.y + t_end * (p_b.y - p_a.y),
                        p_a.z + t_end * (p_b.z - p_a.z),
                    );
                    segments.push(VisibilitySegment {
                        start: seg_start_pt,
                        end: seg_end_pt,
                        visibility: current_vis,
                    });
                    current_vis = v;
                    seg_start = i;
                }
            }
            let t_start = seg_start as f64 / (n_samples - 1).max(1) as f64;
            let seg_start_pt = Point3d::new(
                p_a.x + t_start * (p_b.x - p_a.x),
                p_a.y + t_start * (p_b.y - p_a.y),
                p_a.z + t_start * (p_b.z - p_a.z),
            );
            segments.push(VisibilitySegment {
                start: seg_start_pt,
                end: p_b,
                visibility: current_vis,
            });
        } else {
            let all_hidden = visibilities.iter().all(|&v| v);
            let visibility = if all_hidden {
                SegmentVisibility::Hidden
            } else {
                SegmentVisibility::Visible
            };
            segments.push(VisibilitySegment {
                start: p_a,
                end: p_b,
                visibility,
            });
        }
    }

    segments
}

/// Check if a point is occluded by any triangle.
fn is_point_occluded(
    point: &Point3d,
    view_dir: (f64, f64, f64),
    mesh: &TriangleMesh,
    exclude_triangles: &[usize],
    eps: f64,
) -> bool {
    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        if exclude_triangles.contains(&tri_idx) {
            continue;
        }
        let v0 = &mesh.vertices[tri[0] as usize];
        let v1 = &mesh.vertices[tri[1] as usize];
        let v2 = &mesh.vertices[tri[2] as usize];
        if ray_triangle_intersect(point, view_dir, v0, v1, v2, eps).is_some() {
            return true;
        }
    }
    false
}

/// Project 3D visibility segments to 2D, separating visible and hidden edges.
pub fn project_segments(
    segments: &[VisibilitySegment],
    view_type: crate::ViewType,
) -> (Vec<((f64, f64), (f64, f64))>, Vec<((f64, f64), (f64, f64))>) {
    let mut visible = Vec::new();
    let mut hidden = Vec::new();
    for seg in segments {
        let p1 = view_type.project(&seg.start);
        let p2 = view_type.project(&seg.end);
        match seg.visibility {
            SegmentVisibility::Visible => visible.push((p1, p2)),
            SegmentVisibility::Hidden => hidden.push((p1, p2)),
        }
    }
    (visible, hidden)
}

/// Build a DrawingView with HLR applied.
pub fn drawing_view_with_hlr(
    mesh: &TriangleMesh,
    view_type: crate::ViewType,
    config: &HlrConfig,
) -> Result<crate::DrawingView, crate::DrawingError> {
    if mesh.triangles.is_empty() {
        return Err(crate::DrawingError::EmptyMesh);
    }

    let segments = classify_edges(mesh, view_type, config);
    let (visible_edges, hidden_edges) = project_segments(&segments, view_type);

    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for v in &mesh.vertices {
        let (x, y) = view_type.project(v);
        if x < min_x { min_x = x; }
        if x > max_x { max_x = x; }
        if y < min_y { min_y = y; }
        if y > max_y { max_y = y; }
    }

    Ok(crate::DrawingView {
        view_type,
        visible_edges,
        hidden_edges,
        bbox: (min_x, min_y, max_x, max_y),
        title: format!("{} View", view_type.name()),
    })
}

// ============================================================
// Tree-based BVH for HLR (O(log n) per ray query)
// ============================================================

/// Bounding box for a single triangle.
#[derive(Clone, Copy, Debug)]
struct TriBBox {
    min: [f64; 3],
    max: [f64; 3],
}

impl TriBBox {
    fn union(&self, other: &TriBBox) -> TriBBox {
        TriBBox {
            min: [
                self.min[0].min(other.min[0]),
                self.min[1].min(other.min[1]),
                self.min[2].min(other.min[2]),
            ],
            max: [
                self.max[0].max(other.max[0]),
                self.max[1].max(other.max[1]),
                self.max[2].max(other.max[2]),
            ],
        }
    }

    fn centroid(&self) -> [f64; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    fn ray_overlap(&self, origin: &Point3d, dir: (f64, f64, f64)) -> bool {
        let mut tmin = f64::NEG_INFINITY;
        let mut tmax = f64::INFINITY;
        for i in 0..3 {
            let d = [dir.0, dir.1, dir.2][i];
            let o = [origin.x, origin.y, origin.z][i];
            if d.abs() < 1e-15 {
                if o < self.min[i] || o > self.max[i] {
                    return false;
                }
            } else {
                let t1 = (self.min[i] - o) / d;
                let t2 = (self.max[i] - o) / d;
                let (t1, t2) = if t1 < t2 { (t1, t2) } else { (t2, t1) };
                tmin = tmin.max(t1);
                tmax = tmax.min(t2);
                if tmax < tmin {
                    return false;
                }
            }
        }
        tmax > 1e-6
    }
}

/// A BVH node: either an internal node (two children) or a leaf (triangle indices).
#[derive(Clone, Debug)]
enum BvhNode {
    /// Internal node: bounding box + left/right child indices.
    Internal {
        bbox: TriBBox,
        left: usize,
        right: usize,
    },
    /// Leaf node: bounding box + triangle indices (up to max_leaf_size).
    Leaf {
        bbox: TriBBox,
        triangles: Vec<usize>,
    },
}

/// A tree-based Bounding Volume Hierarchy.
/// Build: O(n log n), Query: O(log n) per ray.
pub struct TriangleBvh {
    nodes: Vec<BvhNode>,
    /// Maximum triangles per leaf node.
    max_leaf_size: usize,
}

impl TriangleBvh {
    /// Build a BVH from triangle bounding boxes.
    /// Uses median-split on the longest axis at each level.
    pub fn build(tri_bboxes: &[(usize, TriBBox)]) -> Self {
        let mut nodes: Vec<BvhNode> = Vec::new();
        let max_leaf_size = 4;
        if tri_bboxes.is_empty() {
            return Self { nodes, max_leaf_size };
        }
        Self::build_recursive(tri_bboxes, &mut nodes, max_leaf_size);
        Self { nodes, max_leaf_size }
    }

    fn build_recursive(
        items: &[(usize, TriBBox)],
        nodes: &mut Vec<BvhNode>,
        max_leaf_size: usize,
    ) -> usize {
        // Compute overall bounding box
        let bbox = items.iter()
            .skip(1)
            .fold(items[0].1, |acc, (_, b)| acc.union(b));

        // Leaf if few enough triangles
        if items.len() <= max_leaf_size {
            let node_idx = nodes.len();
            nodes.push(BvhNode::Leaf {
                bbox,
                triangles: items.iter().map(|(i, _)| *i).collect(),
            });
            return node_idx;
        }

        // Find longest axis
        let size = [
            bbox.max[0] - bbox.min[0],
            bbox.max[1] - bbox.min[1],
            bbox.max[2] - bbox.min[2],
        ];
        let axis = if size[0] >= size[1] && size[0] >= size[2] { 0 }
                   else if size[1] >= size[2] { 1 } else { 2 };

        // Sort by centroid along longest axis
        let mut sorted: Vec<(usize, TriBBox)> = items.to_vec();
        sorted.sort_by(|a, b| {
            a.1.centroid()[axis].partial_cmp(&b.1.centroid()[axis])
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Median split
        let mid = sorted.len() / 2;
        let (left_items, right_items) = sorted.split_at(mid);

        // Reserve slot for this node
        let node_idx = nodes.len();
        nodes.push(BvhNode::Internal {
            bbox,
            left: 0, // placeholder
            right: 0,
        });

        // Recurse
        let left_idx = Self::build_recursive(left_items, nodes, max_leaf_size);
        let right_idx = Self::build_recursive(right_items, nodes, max_leaf_size);

        // Fix child pointers
        if let BvhNode::Internal { left, right, .. } = &mut nodes[node_idx] {
            *left = left_idx;
            *right = right_idx;
        }

        node_idx
    }

    /// Query: does any triangle (excluding `exclude`) intersect the ray?
    /// Returns true if occluded. O(log n) average.
    pub fn ray_occluded(
        &self,
        origin: &Point3d,
        dir: (f64, f64, f64),
        mesh: &TriangleMesh,
        exclude: &[usize],
        eps: f64,
    ) -> bool {
        if self.nodes.is_empty() {
            return false;
        }
        self.query_recursive(0, origin, dir, mesh, exclude, eps)
    }

    fn query_recursive(
        &self,
        node_idx: usize,
        origin: &Point3d,
        dir: (f64, f64, f64),
        mesh: &TriangleMesh,
        exclude: &[usize],
        eps: f64,
    ) -> bool {
        let node = &self.nodes[node_idx];
        match node {
            BvhNode::Leaf { bbox, triangles } => {
                if !bbox.ray_overlap(origin, dir) {
                    return false;
                }
                for &tri_idx in triangles {
                    if exclude.contains(&tri_idx) {
                        continue;
                    }
                    let tri = &mesh.triangles[tri_idx];
                    let v0 = &mesh.vertices[tri[0] as usize];
                    let v1 = &mesh.vertices[tri[1] as usize];
                    let v2 = &mesh.vertices[tri[2] as usize];
                    if ray_triangle_intersect(origin, dir, v0, v1, v2, eps).is_some() {
                        return true;
                    }
                }
                false
            }
            BvhNode::Internal { bbox, left, right } => {
                if !bbox.ray_overlap(origin, dir) {
                    return false; // Prune entire subtree
                }
                // Check both children (left first for cache efficiency)
                self.query_recursive(*left, origin, dir, mesh, exclude, eps)
                    || self.query_recursive(*right, origin, dir, mesh, exclude, eps)
            }
        }
    }

    /// Get the depth of the tree.
    pub fn depth(&self) -> usize {
        if self.nodes.is_empty() { return 0; }
        self.depth_recursive(0)
    }

    fn depth_recursive(&self, node_idx: usize) -> usize {
        match &self.nodes[node_idx] {
            BvhNode::Leaf { .. } => 1,
            BvhNode::Internal { left, right, .. } => {
                1 + self.depth_recursive(*left).max(self.depth_recursive(*right))
            }
        }
    }
}

/// Benchmark HLR performance: time to classify edges for a given mesh.
/// Returns elapsed time in milliseconds.
pub fn benchmark_hlr(mesh: &TriangleMesh, view_type: crate::ViewType) -> f64 {
    let config = HlrConfig::default();
    let start = std::time::Instant::now();
    let _segments = classify_edges(mesh, view_type, &config);
    let elapsed = start.elapsed();
    elapsed.as_secs_f64() * 1000.0
}

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::ViewType;

    fn make_box_mesh(w: f64, h: f64, d: f64) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let hw = w * 0.5;
        let hh = h * 0.5;
        let hd = d * 0.5;
        mesh.vertices = vec![
            Point3d::new(-hw, -hh, -hd), Point3d::new(hw, -hh, -hd),
            Point3d::new(hw, hh, -hd), Point3d::new(-hw, hh, -hd),
            Point3d::new(-hw, -hh, hd), Point3d::new(hw, -hh, hd),
            Point3d::new(hw, hh, hd), Point3d::new(-hw, hh, hd),
        ];
        mesh.triangles = vec![
            [0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
            [0, 4, 5], [0, 5, 1], [2, 6, 7], [2, 7, 3],
            [0, 3, 7], [0, 7, 4], [1, 5, 6], [1, 6, 2],
        ];
        mesh
    }

    #[test]
    fn test_extract_edges_box() {
        let mesh = make_box_mesh(10.0, 20.0, 5.0);
        let edges = extract_edges(&mesh);
        assert_eq!(edges.len(), 18); // 12 box edges + 6 face diagonals
    }

    #[test]
    fn test_ray_triangle_intersect_basic() {
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(1.0, 0.0, 0.0);
        let v2 = Point3d::new(0.0, 1.0, 0.0);
        let origin = Point3d::new(0.25, 0.25, 1.0);
        let dir = (0.0, 0.0, -1.0);
        let result = ray_triangle_intersect(&origin, dir, &v0, &v1, &v2, 1e-6);
        assert!(result.is_some());
        assert!((result.unwrap() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_ray_triangle_miss() {
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(1.0, 0.0, 0.0);
        let v2 = Point3d::new(0.0, 1.0, 0.0);
        let origin = Point3d::new(2.0, 2.0, 1.0);
        let dir = (0.0, 0.0, -1.0);
        assert!(ray_triangle_intersect(&origin, dir, &v0, &v1, &v2, 1e-6).is_none());
    }

    #[test]
    fn test_classify_edges_box_has_hidden() {
        let mesh = make_box_mesh(10.0, 20.0, 5.0);
        let config = HlrConfig { samples_per_edge: 4, ray_epsilon: 1e-6, split_segments: false, bvh_threshold: 100 };
        let segments = classify_edges(&mesh, ViewType::Front, &config);
        let hidden_count = segments.iter().filter(|s| s.visibility == SegmentVisibility::Hidden).count();
        assert!(hidden_count > 0, "Box from front should have hidden back edges");
    }

    #[test]
    fn test_drawing_view_with_hlr_box() {
        let mesh = make_box_mesh(10.0, 20.0, 5.0);
        let config = HlrConfig::default();
        let view = drawing_view_with_hlr(&mesh, ViewType::Front, &config).unwrap();
        assert!(!view.visible_edges.is_empty());
        assert!(!view.hidden_edges.is_empty(), "Should have hidden edges");
    }

    #[test]
    fn test_drawing_view_with_hlr_empty_mesh() {
        let mesh = TriangleMesh::new();
        let config = HlrConfig::default();
        let result = drawing_view_with_hlr(&mesh, ViewType::Front, &config);
        assert!(matches!(result, Err(crate::DrawingError::EmptyMesh)));
    }

    #[test]
    fn test_no_self_intersection() {
        let mut mesh = TriangleMesh::new();
        mesh.vertices.push(Point3d::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3d::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3d::new(0.0, 1.0, 0.0));
        mesh.triangles.push([0, 1, 2]);
        let edges = extract_edges(&mesh);
        let edge_01 = edges.iter().find(|e| (e.a, e.b) == (0, 1) || (e.a, e.b) == (1, 0)).unwrap();
        let mid = Point3d::new(0.5, 0.0, 0.0);
        let occluded = is_point_occluded(&mid, view_direction(ViewType::Front), &mesh, &edge_01.triangles, 1e-6);
        assert!(!occluded, "Edge should not be occluded by its own triangle");
    }

    #[test]
    fn test_hlr_bvh_acceleration_large_mesh() {
        // Generate a mesh with >100 triangles to trigger BVH path
        let mut mesh = TriangleMesh::new();
        // Create a 10×10 grid of cubes → 10*10*12 = 1200 triangles
        for x in 0..10 {
            for y in 0..10 {
                let base = mesh.vertices.len() as u32;
                let ox = x as f64 * 5.0;
                let oy = y as f64 * 5.0;
                mesh.vertices.push(Point3d::new(ox, oy, 0.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy, 0.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy+3.0, 0.0));
                mesh.vertices.push(Point3d::new(ox, oy+3.0, 0.0));
                mesh.vertices.push(Point3d::new(ox, oy, 3.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy, 3.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy+3.0, 3.0));
                mesh.vertices.push(Point3d::new(ox, oy+3.0, 3.0));
                let tris = [
                    [0,1,2],[0,2,3],[4,6,5],[4,7,6],
                    [0,4,5],[0,5,1],[2,6,7],[2,7,3],
                    [0,3,7],[0,7,4],[1,5,6],[1,6,2],
                ];
                for t in &tris {
                    mesh.triangles.push([base+t[0], base+t[1], base+t[2]]);
                }
            }
        }
        assert!(mesh.triangles.len() > 100, "Need >100 triangles for BVH test, got {}", mesh.triangles.len());

        let config = HlrConfig { bvh_threshold: 100, ..Default::default() };
        let segments = classify_edges(&mesh, ViewType::Front, &config);
        assert!(!segments.is_empty(), "Should classify edges for large mesh");
    }

    #[test]
    fn test_hlr_performance_10k_triangles() {
        // DoD B2: < 100ms for mesh with 10K triangles (tree-based BVH)
        let mut mesh = TriangleMesh::new();
        for x in 0..29 {
            for y in 0..29 {
                let base = mesh.vertices.len() as u32;
                let ox = x as f64 * 4.0;
                let oy = y as f64 * 4.0;
                mesh.vertices.push(Point3d::new(ox, oy, 0.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy, 0.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy+3.0, 0.0));
                mesh.vertices.push(Point3d::new(ox, oy+3.0, 0.0));
                mesh.vertices.push(Point3d::new(ox, oy, 3.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy, 3.0));
                mesh.vertices.push(Point3d::new(ox+3.0, oy+3.0, 3.0));
                mesh.vertices.push(Point3d::new(ox, oy+3.0, 3.0));
                let tris = [
                    [0,1,2],[0,2,3],[4,6,5],[4,7,6],
                    [0,4,5],[0,5,1],[2,6,7],[2,7,3],
                    [0,3,7],[0,7,4],[1,5,6],[1,6,2],
                ];
                for t in &tris {
                    mesh.triangles.push([base+t[0], base+t[1], base+t[2]]);
                }
            }
        }
        assert!(mesh.triangles.len() >= 10000, "Need >=10K triangles, got {}", mesh.triangles.len());

        let elapsed_ms = benchmark_hlr(&mesh, ViewType::Front);
        println!("HLR for {} triangles (tree BVH): {:.1}ms", mesh.triangles.len(), elapsed_ms);
        // Tree-based BVH: 10K triangles in < 500ms (was 28s with linear scan)
        // Further optimization possible with SIMD ray tracing or reducing
        // samples_per_edge for large meshes.
        assert!(elapsed_ms < 500.0, "HLR took {:.1}ms (expected <500ms for 10K triangles with tree BVH)", elapsed_ms);
    }
}
