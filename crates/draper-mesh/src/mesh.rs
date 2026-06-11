// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Mesh data structures.

use draper_geometry::Point3d;
use std::collections::HashMap;
use std::fmt;

/// A bit-exact hash key for a 3D point, used for vertex deduplication.
///
/// Since the edge cache applies deterministic rounding (48-bit mantissa precision),
/// shared-edge vertices produce bit-identical f64 values. Using the raw bit
/// representation as a hash key is both correct and fast — no epsilon comparison
/// is needed because the rounding already guarantees consistent bit patterns.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VertexKey(u64, u64, u64);

impl VertexKey {
    /// Create a bit-exact key from a Point3d.
    #[inline]
    pub fn from_point(p: &Point3d) -> Self {
        VertexKey(p.x.to_bits(), p.y.to_bits(), p.z.to_bits())
    }
}

/// A vertex deduplication map that tracks which 3D points have already been
/// added to a mesh, mapping them to their vertex indices.
///
/// Used during the "topology-first" merge step to ensure that shared-edge
/// vertices from different faces get the same vertex index in the final mesh,
/// making it watertight by construction.
pub type VertexDedupMap = HashMap<VertexKey, u32>;

/// A 3D triangle mesh.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TriangleMesh {
    /// Vertex positions.
    pub vertices: Vec<Point3d>,
    /// Triangle indices (3 vertex indices per triangle).
    pub triangles: Vec<[u32; 3]>,
    /// Optional vertex normals.
    pub normals: Option<Vec<[f64; 3]>>,
    /// Optional triangle normals.
    pub face_normals: Option<Vec<[f64; 3]>>,
    /// Optional per-triangle RGBA colors (0..1 range).
    pub triangle_colors: Option<Vec<[f32; 4]>>,
    /// Optional per-triangle face ID (TopoId of the source BRep face).
    /// Used for selection, highlighting, and UV grid display.
    pub triangle_face_ids: Option<Vec<u64>>,
}

impl TriangleMesh {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            normals: None,
            face_normals: None,
            triangle_colors: None,
            triangle_face_ids: None,
        }
    }

    /// Create from vertices and triangle indices.
    pub fn from_data(vertices: Vec<Point3d>, triangles: Vec<[u32; 3]>) -> Self {
        Self {
            vertices,
            triangles,
            normals: None,
            face_normals: None,
            triangle_colors: None,
            triangle_face_ids: None,
        }
    }

    /// Add a vertex and return its index.
    pub fn add_vertex(&mut self, p: Point3d) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(p);
        idx
    }

    /// Add a vertex normal. Call after add_vertex with the returned index.
    pub fn add_vertex_normal(&mut self, _idx: u32, normal: [f64; 3]) {
        if self.normals.is_none() {
            self.normals = Some(vec![[0.0, 0.0, 1.0]; self.vertices.len() - 1]);
        }
        if let Some(ref mut normals) = self.normals {
            normals.push(normal);
        }
    }

    /// Add a triangle.
    pub fn add_triangle(&mut self, i: u32, j: u32, k: u32) {
        self.triangles.push([i, j, k]);
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Compute face normals.
    pub fn compute_face_normals(&mut self) {
        let mut normals = Vec::with_capacity(self.triangles.len());
        for tri in &self.triangles {
            let v0 = self.vertices[tri[0] as usize];
            let v1 = self.vertices[tri[1] as usize];
            let v2 = self.vertices[tri[2] as usize];

            let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
            let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

            let nx = e1.1 * e2.2 - e1.2 * e2.1;
            let ny = e1.2 * e2.0 - e1.0 * e2.2;
            let nz = e1.0 * e2.1 - e1.1 * e2.0;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 1e-15 {
                normals.push([nx / len, ny / len, nz / len]);
            } else {
                normals.push([0.0, 0.0, 1.0]);
            }
        }
        self.face_normals = Some(normals);
    }

    /// Merge another mesh into this one.
    pub fn merge(&mut self, other: &TriangleMesh) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for tri in &other.triangles {
            self.triangles.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
        }
        // Merge vertex normals
        match (&mut self.normals, &other.normals) {
            (Some(ref mut self_normals), Some(ref other_normals)) => {
                self_normals.extend(other_normals.iter().cloned());
            }
            (None, Some(ref other_normals)) => {
                // We need to fill in default normals for existing vertices
                let mut combined = vec![[0.0, 0.0, 1.0]; self.vertices.len() - other.vertices.len()];
                combined.extend(other_normals.iter().cloned());
                self.normals = Some(combined);
            }
            _ => {}
        }
        // Merge face normals (per-triangle)
        if self.face_normals.is_none() && other.face_normals.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.face_normals = Some(vec![[0.0, 0.0, 1.0]; existing_count]);
        }
        match (&mut self.face_normals, &other.face_normals) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            _ => {}
        }
        // Merge triangle colors (per-triangle)
        if self.triangle_colors.is_none() && other.triangle_colors.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.triangle_colors = Some(vec![[0.62, 0.65, 0.70, 1.0]; existing_count]);
        }
        match (&mut self.triangle_colors, &other.triangle_colors) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            _ => {}
        }
        // Merge face IDs
        if self.triangle_face_ids.is_none() && other.triangle_face_ids.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.triangle_face_ids = Some(vec![0; existing_count]);
        }
        match (&mut self.triangle_face_ids, &other.triangle_face_ids) {
            (Some(ref mut ids), Some(ref other_ids)) => {
                ids.extend(other_ids.iter().cloned());
            }
            _ => {}
        }
    }

    /// Merge another mesh into this one with vertex deduplication.
    ///
    /// This is the **topology-first** merge: when two faces share an edge,
    /// their boundary vertices have identical 3D coordinates (guaranteed by
    /// the `EdgeDiscretizationCache` with deterministic rounding). Instead of
    /// blindly appending all vertices with an offset (like `merge()`), this
    /// method reuses existing vertex indices for points that are already
    /// present in the accumulated mesh.
    ///
    /// # How it works
    ///
    /// 1. For each vertex in `other`, compute a `VertexKey` from its bit-exact
    ///    coordinates.
    /// 2. If the key exists in `dedup_map`, reuse the existing vertex index.
    /// 3. Otherwise, add the vertex and store its index in `dedup_map`.
    /// 4. Remap triangle indices from `other`'s local indices to the
    ///    (deduplicated) global indices.
    ///
    /// # Arguments
    ///
    /// * `other` — The per-face mesh to merge in.
    /// * `dedup_map` — A mutable vertex deduplication map that persists
    ///   across calls. Must be created once before the first call and reused
    ///   for all subsequent face merges in the same solid.
    ///
    /// # Why this matters
    ///
    /// Without deduplication, shared-edge vertices get different indices in
    /// the final mesh, producing boundary edges. The mesh is NOT watertight
    /// even though the edge cache guarantees bit-identical 3D coordinates.
    /// With deduplication, shared vertices get the same index, making the
    /// mesh watertight **by construction** — no post-hoc repair needed.
    pub fn merge_deduplicating(&mut self, other: &TriangleMesh, dedup_map: &mut VertexDedupMap) {
        // Build index remapping: other's local vertex index → global index
        let mut index_map: Vec<u32> = Vec::with_capacity(other.vertices.len());

        for vertex in &other.vertices {
            let key = VertexKey::from_point(vertex);
            if let Some(&existing_idx) = dedup_map.get(&key) {
                // Vertex already exists — reuse its index
                index_map.push(existing_idx);
            } else {
                // New vertex — add to mesh and record in dedup map
                let new_idx = self.vertices.len() as u32;
                self.vertices.push(*vertex);
                dedup_map.insert(key, new_idx);
                index_map.push(new_idx);
            }
        }

        // Add triangles with remapped indices
        for tri in &other.triangles {
            self.triangles.push([
                index_map[tri[0] as usize],
                index_map[tri[1] as usize],
                index_map[tri[2] as usize],
            ]);
        }

        // Handle vertex normals: when deduplicating, the first face's normal
        // wins for shared vertices. This is acceptable because normals are
        // later smoothed by `smooth_normals_adaptive` which computes proper
        // averaged normals across shared edges.
        match (&mut self.normals, &other.normals) {
            (Some(ref mut self_normals), Some(ref other_normals)) => {
                // For new vertices, add their normals. For reused vertices,
                // skip (keep the first face's normal).
                for (i, vertex) in other.vertices.iter().enumerate() {
                    let key = VertexKey::from_point(vertex);
                    let global_idx = index_map[i] as usize;
                    // Only add normal if this is a new vertex (global_idx == current length before push)
                    if global_idx >= self_normals.len() {
                        self_normals.push(other_normals[i]);
                    }
                }
            }
            (None, Some(ref other_normals)) => {
                // Fill default normals for existing vertices, then add new ones
                let mut combined = vec![[0.0, 0.0, 1.0]; self.vertices.len() - other.vertices.len()];
                for (i, vertex) in other.vertices.iter().enumerate() {
                    let key = VertexKey::from_point(vertex);
                    let global_idx = index_map[i] as usize;
                    if global_idx >= combined.len() {
                        combined.push(other_normals[i]);
                    }
                }
                self.normals = Some(combined);
            }
            _ => {}
        }

        // Merge face normals (per-triangle, no deduplication needed)
        if self.face_normals.is_none() && other.face_normals.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.face_normals = Some(vec![[0.0, 0.0, 1.0]; existing_count]);
        }
        match (&mut self.face_normals, &other.face_normals) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            _ => {}
        }

        // Merge triangle colors (per-triangle, no deduplication needed)
        if self.triangle_colors.is_none() && other.triangle_colors.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.triangle_colors = Some(vec![[0.62, 0.65, 0.70, 1.0]; existing_count]);
        }
        match (&mut self.triangle_colors, &other.triangle_colors) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            _ => {}
        }

        // Merge face IDs (per-triangle, no deduplication needed)
        if self.triangle_face_ids.is_none() && other.triangle_face_ids.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.triangle_face_ids = Some(vec![0; existing_count]);
        }
        match (&mut self.triangle_face_ids, &other.triangle_face_ids) {
            (Some(ref mut ids), Some(ref other_ids)) => {
                ids.extend(other_ids.iter().cloned());
            }
            _ => {}
        }
    }

    /// Merge another mesh with a uniform color applied to all its triangles.
    pub fn merge_with_color(&mut self, other: &TriangleMesh, color: [f32; 4]) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for tri in &other.triangles {
            self.triangles.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
        }
        if self.triangle_colors.is_none() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.triangle_colors = Some(vec![[0.62, 0.65, 0.70, 1.0]; existing_count]);
        }
        if let Some(ref mut colors) = self.triangle_colors {
            for _ in 0..other.triangles.len() {
                colors.push(color);
            }
        }
        // Merge vertex normals
        match (&mut self.normals, &other.normals) {
            (Some(ref mut self_normals), Some(ref other_normals)) => {
                self_normals.extend(other_normals.iter().cloned());
            }
            (None, Some(ref other_normals)) => {
                // Fill in default normals for existing vertices
                let mut combined = vec![[0.0, 0.0, 1.0]; self.vertices.len() - other.vertices.len()];
                combined.extend(other_normals.iter().cloned());
                self.normals = Some(combined);
            }
            _ => {}
        }
        // Merge face normals (per-triangle)
        if self.face_normals.is_none() && other.face_normals.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.face_normals = Some(vec![[0.0, 0.0, 1.0]; existing_count]);
        }
        match (&mut self.face_normals, &other.face_normals) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            _ => {}
        }
        // Merge face IDs
        if self.triangle_face_ids.is_none() && other.triangle_face_ids.is_some() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.triangle_face_ids = Some(vec![0; existing_count]);
        }
        match (&mut self.triangle_face_ids, &other.triangle_face_ids) {
            (Some(ref mut ids), Some(ref other_ids)) => {
                ids.extend(other_ids.iter().cloned());
            }
            _ => {}
        }
    }

    /// Ensure triangle_colors matches triangles length, filling with default color if needed.
    pub fn ensure_colors(&mut self, default: [f32; 4]) {
        if self.triangle_colors.is_none() {
            self.triangle_colors = Some(vec![default; self.triangles.len()]);
        } else if let Some(ref mut colors) = self.triangle_colors {
            while colors.len() < self.triangles.len() {
                colors.push(default);
            }
        }
    }

    /// Compute bounding box.
    pub fn bounding_box(&self) -> (Point3d, Point3d) {
        if self.vertices.is_empty() {
            return (Point3d::ORIGIN, Point3d::ORIGIN);
        }
        let mut min = self.vertices[0];
        let mut max = self.vertices[0];
        for v in &self.vertices[1..] {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }
        (min, max)
    }

    /// Total surface area.
    pub fn surface_area(&self) -> f64 {
        let mut area = 0.0;
        for tri in &self.triangles {
            let v0 = self.vertices[tri[0] as usize];
            let v1 = self.vertices[tri[1] as usize];
            let v2 = self.vertices[tri[2] as usize];
            // Cross product of two edges / 2
            let e1x = v1.x - v0.x;
            let e1y = v1.y - v0.y;
            let e1z = v1.z - v0.z;
            let e2x = v2.x - v0.x;
            let e2y = v2.y - v0.y;
            let e2z = v2.z - v0.z;
            let cx = e1y * e2z - e1z * e2y;
            let cy = e1z * e2x - e1x * e2z;
            let cz = e1x * e2y - e1y * e2x;
            area += (cx * cx + cy * cy + cz * cz).sqrt() * 0.5;
        }
        area
    }

    /// Transform all vertices and normals.
    ///
    /// Vertices are transformed by the full 4×4 matrix (including translation).
    /// Normals are transformed by the inverse-transpose of the upper-left 3×3
    /// submatrix — this preserves correct lighting/backface-culling for
    /// non-uniform scaling and reflection transforms.
    pub fn transform(&mut self, m: &[[f64; 4]; 4]) {
        for v in &mut self.vertices {
            *v = v.transform(m);
        }
        // Transform normals by inverse-transpose of 3×3 rotation
        if let Some(ref mut normals) = self.normals {
            let inv_transpose = compute_normal_transform(m);
            for n in normals.iter_mut() {
                let nx = inv_transpose[0][0] * n[0] + inv_transpose[0][1] * n[1] + inv_transpose[0][2] * n[2];
                let ny = inv_transpose[1][0] * n[0] + inv_transpose[1][1] * n[1] + inv_transpose[1][2] * n[2];
                let nz = inv_transpose[2][0] * n[0] + inv_transpose[2][1] * n[1] + inv_transpose[2][2] * n[2];
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    *n = [nx / len, ny / len, nz / len];
                }
            }
        }
        // Face normals also need to be transformed
        if let Some(ref mut face_normals) = self.face_normals {
            let inv_transpose = compute_normal_transform(m);
            for n in face_normals.iter_mut() {
                let nx = inv_transpose[0][0] * n[0] + inv_transpose[0][1] * n[1] + inv_transpose[0][2] * n[2];
                let ny = inv_transpose[1][0] * n[0] + inv_transpose[1][1] * n[1] + inv_transpose[1][2] * n[2];
                let nz = inv_transpose[2][0] * n[0] + inv_transpose[2][1] * n[1] + inv_transpose[2][2] * n[2];
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    *n = [nx / len, ny / len, nz / len];
                }
            }
        }
    }
}

/// A 2D point for triangulation (in parametric space).
#[derive(Clone, Copy, Debug)]
pub struct Point2dForTriangulation {
    pub x: f64,
    pub y: f64,
    pub original_index: usize,
}

/// Edge constraint for triangulation.
#[derive(Clone, Copy, Debug)]
pub struct ConstraintEdge {
    pub start: usize,
    pub end: usize,
}

/// Compute the inverse-transpose of the upper-left 3×3 submatrix of a 4×4 matrix.
///
/// This is used for transforming normals: if vertices are transformed by M,
/// then normals must be transformed by (M⁻¹)ᵀ to remain correct under
/// non-uniform scaling and reflection transforms.
fn compute_normal_transform(m: &[[f64; 4]; 4]) -> [[f64; 3]; 3] {
    // Extract 3×3 submatrix
    let a = m[0][0]; let b = m[0][1]; let c = m[0][2];
    let d = m[1][0]; let e = m[1][1]; let f = m[1][2];
    let g = m[2][0]; let h = m[2][1]; let i = m[2][2];

    // Compute determinant of 3×3
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

    if det.abs() < 1e-15 {
        // Degenerate matrix — return identity
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }

    let inv_det = 1.0 / det;

    // Compute inverse of 3×3 (cofactor matrix transposed, divided by det)
    let inv = [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ];

    // Transpose the inverse to get (M⁻¹)ᵀ
    [
        [inv[0][0], inv[1][0], inv[2][0]],
        [inv[0][1], inv[1][1], inv[2][1]],
        [inv[0][2], inv[1][2], inv[2][2]],
    ]
}
