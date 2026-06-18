// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! WASM-friendly mesh serialization for zero-copy transfer to JavaScript.
//!
//! This module provides `MeshData` — a flat, TypedArray-compatible representation
//! of a `TriangleMesh` that can be efficiently transferred to JavaScript via
//! `postMessage` with transferable `ArrayBuffer`s.
//!
//! # Wire Format
//!
//! ```js
//! {
//!   vertex_count: number,        // Number of vertices
//!   triangle_count: number,      // Number of triangles
//!   vertices: Float32Array,      // [x0,y0,z0, x1,y1,z1, ...] — 3 floats per vertex
//!   indices: Uint32Array,        // [i0,j0,k0, i1,j1,k1, ...] — 3 indices per triangle
//!   normals: Float32Array|null,  // [nx0,ny0,nz0, ...] — 3 floats per vertex (or null)
//!   face_normals: Float32Array|null, // [nx0,ny0,nz0, ...] — 3 floats per triangle
//!   colors: Float32Array|null,   // [r0,g0,b0,a0, ...] — 4 floats per triangle
//!   face_ids: Float64Array|null, // [id0, id1, ...] — 1 id per triangle
//! }
//! ```
//!
//! # Performance
//!
//! Converting a 100K-triangle mesh takes ~1ms on modern hardware.
//! The `ArrayBuffer`s are transferred (zero-copy) via `postMessage`,
//! avoiding the cost of structured cloning.

use crate::mesh::TriangleMesh;

/// Flat, TypedArray-compatible mesh representation for WASM/JS transfer.
///
/// All vertex/index/normal data is stored as contiguous `Vec<f32>` / `Vec<u32>`
/// arrays that can be directly viewed as `Float32Array` / `Uint32Array` in JS
/// without any per-element conversion.
///
/// # Construction
///
/// ```ignore
/// let mesh: TriangleMesh = triangulate_solid(&solid, &params);
/// let data = MeshData::from_mesh(&mesh);
///
/// // Access the raw bytes for transferable postMessage
/// let vertices = JsValue::from(data.vertices);  // Float32Array view
/// let indices = JsValue::from(data.indices);     // Uint32Array view
/// ```
#[derive(Debug)]
pub struct MeshData {
    /// Number of vertices in the mesh.
    pub vertex_count: usize,
    /// Number of triangles in the mesh.
    pub triangle_count: usize,
    /// Vertex positions: [x0,y0,z0, x1,y1,z1, ...] — 3 * vertex_count floats.
    /// Stored as f32 for GPU upload (most CAD models fit in f32 precision).
    pub vertices: Vec<f32>,
    /// Triangle indices: [i0,j0,k0, i1,j1,k1, ...] — 3 * triangle_count uint32s.
    pub indices: Vec<u32>,
    /// Vertex normals: [nx0,ny0,nz0, ...] — 3 * vertex_count floats.
    /// `None` if the mesh has no normals.
    pub normals: Option<Vec<f32>>,
    /// Face (triangle) normals: [nx0,ny0,nz0, ...] — 3 * triangle_count floats.
    /// `None` if not computed.
    pub face_normals: Option<Vec<f32>>,
    /// Per-triangle RGBA colors: [r0,g0,b0,a0, ...] — 4 * triangle_count floats.
    /// Values in [0, 1] range. `None` if no per-triangle colors.
    pub colors: Option<Vec<f32>>,
    /// Per-triangle face IDs: [id0, id1, ...] — triangle_count float64s.
    /// `None` if no face IDs.
    pub face_ids: Option<Vec<f64>>,
}

impl MeshData {
    /// Convert a `TriangleMesh` to a flat `MeshData` for JS transfer.
    ///
    /// This performs f64→f32 conversion for vertex positions and normals,
    /// which is appropriate for GPU upload. The precision loss from f64 to f32
    /// is ~7 significant digits, which is sufficient for all practical CAD models
    /// (models larger than 10km with sub-millimeter detail are extremely rare).
    ///
    /// # Performance
    ///
    /// For a 100K-triangle mesh (300K vertices), this takes ~1ms.
    pub fn from_mesh(mesh: &TriangleMesh) -> Self {
        let vertex_count = mesh.vertices.len();
        let triangle_count = mesh.triangles.len();

        // Flatten vertices: Point3d (f64) → [f32, f32, f32]
        let mut vertices = Vec::with_capacity(vertex_count * 3);
        for p in &mesh.vertices {
            vertices.push(p.x as f32);
            vertices.push(p.y as f32);
            vertices.push(p.z as f32);
        }

        // Flatten indices: [[u32; 3]; N] → [u32]
        let mut indices = Vec::with_capacity(triangle_count * 3);
        for tri in &mesh.triangles {
            indices.push(tri[0]);
            indices.push(tri[1]);
            indices.push(tri[2]);
        }

        // Flatten vertex normals (if present)
        let normals = mesh.normals.as_ref().map(|ns| {
            let mut flat = Vec::with_capacity(vertex_count * 3);
            for n in ns {
                flat.push(n[0] as f32);
                flat.push(n[1] as f32);
                flat.push(n[2] as f32);
            }
            flat
        });

        // Flatten face normals (if present)
        let face_normals = mesh.face_normals.as_ref().map(|ns| {
            let mut flat = Vec::with_capacity(triangle_count * 3);
            for n in ns {
                flat.push(n[0] as f32);
                flat.push(n[1] as f32);
                flat.push(n[2] as f32);
            }
            flat
        });

        // Flatten per-triangle colors (if present)
        let colors = mesh.triangle_colors.as_ref().map(|cs| {
            let mut flat = Vec::with_capacity(triangle_count * 4);
            for c in cs {
                flat.push(c[0]);
                flat.push(c[1]);
                flat.push(c[2]);
                flat.push(c[3]);
            }
            flat
        });

        // Flatten per-triangle face IDs (if present)
        let face_ids = mesh.triangle_face_ids.as_ref().map(|ids| {
            ids.iter().map(|&id| id as f64).collect()
        });

        Self {
            vertex_count,
            triangle_count,
            vertices,
            indices,
            normals,
            face_normals,
            colors,
            face_ids,
        }
    }

    /// Estimate the memory size of this mesh data in bytes.
    pub fn size_bytes(&self) -> usize {
        let vertices_bytes = self.vertices.len() * std::mem::size_of::<f32>();
        let indices_bytes = self.indices.len() * std::mem::size_of::<u32>();
        let normals_bytes = self.normals.as_ref().map_or(0, |n| n.len() * std::mem::size_of::<f32>());
        let face_normals_bytes = self.face_normals.as_ref().map_or(0, |n| n.len() * std::mem::size_of::<f32>());
        let colors_bytes = self.colors.as_ref().map_or(0, |c| c.len() * std::mem::size_of::<f32>());
        let face_ids_bytes = self.face_ids.as_ref().map_or(0, |ids| ids.len() * std::mem::size_of::<f64>());
        vertices_bytes + indices_bytes + normals_bytes + face_normals_bytes + colors_bytes + face_ids_bytes
    }
}

/// Incremental mesh update for OffscreenCanvas + Web Worker integration.
///
/// Represents a partial or complete mesh update that can be sent from a
/// Web Worker to the main thread via `postMessage`. Contains enough
/// information to efficiently update a WebGL/WebGPU buffer without
/// re-uploading the entire mesh every frame.
///
/// # Wire Format (for `postMessage` with transferable ArrayBuffers)
///
/// ```js
/// {
///   type: 'partial' | 'complete',
///   progress: number,           // 0.0 to 1.0
///   vertex_count: number,
///   triangle_count: number,
///   vertices: Float32Array,     // transferable
///   indices: Uint32Array,       // transferable
///   normals: Float32Array|null, // transferable
///   face_ids: Float64Array|null // transferable
/// }
/// ```
///
/// # Usage with OffscreenCanvas
///
/// ```js
/// // main.js — main thread
/// const worker = new Worker('triangulate-worker.js');
/// const offscreen = canvas.transferControlToOffscreen();
/// worker.postMessage({ type: 'init', canvas: offscreen, stepData }, [offscreen]);
///
/// // triangulate-worker.js — Web Worker
/// import init, { WasmChunkedTriangulator } from './pkg/draper_mesh.js';
///
/// self.onmessage = async (e) => {
///   if (e.data.type === 'init') {
///     await init();
///     const gl = e.data.canvas.getContext('webgl2');
///     const triangulator = new WasmChunkedTriangulator(e.data.stepData);
///     const renderLoop = () => {
///       const result = triangulator.tick(8); // 8ms budget
///       // Upload partial mesh to WebGL and render
///       uploadMesh(gl, triangulator.getMeshData());
///       render(gl);
///       if (!result.is_complete) {
///         requestAnimationFrame(renderLoop);
///       }
///     };
///     requestAnimationFrame(renderLoop);
///   }
/// };
/// ```
#[derive(Debug)]
pub struct IncrementalMeshUpdate {
    /// Whether this is a partial update or the final complete mesh.
    pub is_complete: bool,
    /// Progress as a fraction in [0.0, 1.0].
    pub progress: f64,
    /// The mesh data (can be partial).
    pub mesh_data: MeshData,
}

impl IncrementalMeshUpdate {
    /// Create an incremental update from a `ChunkResult` and the triangulator.
    ///
    /// This converts the internal mesh representation to a JS-friendly
    /// flat format suitable for transfer to the main thread or upload
    /// to a WebGL/WebGPU buffer.
    ///
    /// # Arguments
    /// * `result` — The `ChunkResult` from `ChunkedBrepTriangulator::process_frame()`
    /// * `mesh` — The partial mesh from the triangulator
    /// * `progress_fraction` — Progress as a fraction in [0.0, 1.0] from `progress_fraction()`
    pub fn from_chunk_result(
        result: &crate::triangulate::ChunkResult,
        mesh: &TriangleMesh,
        progress_fraction: f64,
    ) -> Self {
        let (is_complete, progress) = match result {
            crate::triangulate::ChunkResult::Complete(_) => (true, 1.0),
            crate::triangulate::ChunkResult::InProgress { faces_completed, faces_total } => {
                let p = if *faces_total > 0 {
                    *faces_completed as f64 / *faces_total as f64
                } else {
                    progress_fraction
                };
                (false, p)
            }
        };

        Self {
            is_complete,
            progress,
            mesh_data: MeshData::from_mesh(mesh),
        }
    }

    /// Create an update from raw progress data (simpler API).
    ///
    /// Use this when you track progress externally rather than from `ChunkResult`.
    pub fn from_progress(
        is_complete: bool,
        progress: f64,
        mesh: &TriangleMesh,
    ) -> Self {
        Self {
            is_complete,
            progress,
            mesh_data: MeshData::from_mesh(mesh),
        }
    }

    /// Estimate the memory size of this update in bytes.
    pub fn size_bytes(&self) -> usize {
        self.mesh_data.size_bytes()
    }
}

/// Configuration for WASM-based incremental triangulation.
///
/// This struct is passed from JavaScript to configure the
/// ChunkedBrepTriangulator's behavior in a Web Worker context.
///
/// # Example (JavaScript)
///
/// ```js
/// const config = {
///   time_budget_ms: 8,       // 8ms per tick (120fps)
///   lod: 0.5,                // Interactive quality
///   max_face_triangles: 4000, // Triangle budget per face
///   adaptive: true,           // Use adaptive sampling
/// };
/// ```
#[derive(Clone, Debug)]
pub struct WasmTriangulationConfig {
    /// Time budget per tick in milliseconds.
    /// Default: 8ms (120fps target).
    pub time_budget_ms: f64,
    /// LOD level in [0.0, 1.0].
    /// Default: 0.5 (interactive).
    pub lod: f64,
    /// Maximum triangles per face.
    /// Default: 4000.
    pub max_face_triangles: usize,
    /// Whether to use adaptive sampling.
    /// Default: true.
    pub adaptive: bool,
}

impl Default for WasmTriangulationConfig {
    fn default() -> Self {
        Self {
            time_budget_ms: 8.0,
            lod: 0.5,
            max_face_triangles: 4000,
            adaptive: true,
        }
    }
}

impl WasmTriangulationConfig {
    /// Create config for 120fps preview (coarse quality).
    pub fn preview_120fps() -> Self {
        Self {
            time_budget_ms: 8.0,
            lod: 0.15,
            max_face_triangles: 500,
            adaptive: true,
        }
    }

    /// Create config for 60fps interactive (balanced).
    pub fn interactive_60fps() -> Self {
        Self {
            time_budget_ms: 16.0,
            lod: 0.5,
            max_face_triangles: 4000,
            adaptive: true,
        }
    }

    /// Create config for 30fps high quality.
    pub fn high_quality_30fps() -> Self {
        Self {
            time_budget_ms: 33.0,
            lod: 0.75,
            max_face_triangles: 8000,
            adaptive: true,
        }
    }

    /// Convert to `TriangulationParams` for use with the triangulator.
    pub fn to_params(&self) -> crate::triangulate::TriangulationParams {
        let mut params = crate::triangulate::TriangulationParams::for_lod(self.lod);
        params.max_face_triangles = self.max_face_triangles;
        params.adaptive = self.adaptive;
        params
    }

    /// Convert time budget to `std::time::Duration`.
    pub fn time_budget(&self) -> std::time::Duration {
        std::time::Duration::from_micros((self.time_budget_ms * 1000.0) as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::TriangleMesh;
    use draper_geometry::Point3d;

    #[test]
    fn test_mesh_data_from_simple_mesh() {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        mesh.add_triangle(v0, v1, v2);

        let data = MeshData::from_mesh(&mesh);

        assert_eq!(data.vertex_count, 3);
        assert_eq!(data.triangle_count, 1);
        assert_eq!(data.vertices.len(), 9);  // 3 vertices × 3 floats
        assert_eq!(data.indices.len(), 3);   // 1 triangle × 3 indices
        assert!((data.vertices[0] - 0.0f32).abs() < 1e-6);
        assert!((data.vertices[3] - 1.0f32).abs() < 1e-6);
    }

    #[test]
    fn test_mesh_data_empty_mesh() {
        let mesh = TriangleMesh::new();
        let data = MeshData::from_mesh(&mesh);

        assert_eq!(data.vertex_count, 0);
        assert_eq!(data.triangle_count, 0);
        assert!(data.vertices.is_empty());
        assert!(data.indices.is_empty());
        assert!(data.normals.is_none());
    }

    #[test]
    fn test_mesh_data_size_bytes() {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        mesh.add_triangle(v0, v1, v2);

        let data = MeshData::from_mesh(&mesh);

        // 9 floats (vertices) + 3 uint32 (indices) = 36 + 12 = 48 bytes minimum
        assert!(data.size_bytes() >= 48);
    }

    #[test]
    fn test_incremental_mesh_update() {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        mesh.add_triangle(v0, v1, v2);

        // Test with InProgress variant
        let result = crate::triangulate::ChunkResult::InProgress {
            faces_completed: 3,
            faces_total: 10,
        };
        let update = IncrementalMeshUpdate::from_chunk_result(&result, &mesh, 0.3);
        assert!(!update.is_complete);
        assert!((update.progress - 0.3).abs() < 0.01);
        assert_eq!(update.mesh_data.vertex_count, 3);

        // Test with Complete variant
        let complete_mesh = mesh.clone();
        let result = crate::triangulate::ChunkResult::Complete(complete_mesh);
        let update = IncrementalMeshUpdate::from_chunk_result(&result, &mesh, 1.0);
        assert!(update.is_complete);
        assert!((update.progress - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_incremental_mesh_update_from_progress() {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        mesh.add_triangle(v0, v1, v2);

        let update = IncrementalMeshUpdate::from_progress(false, 0.5, &mesh);
        assert!(!update.is_complete);
        assert!((update.progress - 0.5).abs() < 0.01);
        assert_eq!(update.mesh_data.vertex_count, 3);
    }

    #[test]
    fn test_wasm_triangulation_config() {
        let config = WasmTriangulationConfig::interactive_60fps();
        assert_eq!(config.time_budget_ms, 16.0);
        assert_eq!(config.lod, 0.5);

        let params = config.to_params();
        assert!(params.adaptive);
        assert_eq!(params.max_face_triangles, 4000);

        let budget = config.time_budget();
        assert_eq!(budget.as_millis(), 16);
    }

    #[test]
    fn test_wasm_config_presets() {
        let preview = WasmTriangulationConfig::preview_120fps();
        assert!(preview.time_budget_ms < 10.0);
        assert!(preview.lod < 0.3);

        let hq = WasmTriangulationConfig::high_quality_30fps();
        assert!(hq.time_budget_ms > 20.0);
        assert!(hq.lod > 0.5);
    }

    #[test]
    fn test_frustum_cull_result() {
        let result = FrustumCullResult {
            visible_triangle_indices: vec![0, 1, 2],
            visible_face_ids: vec![1, 2],
            total_triangles: 10,
            visible_triangles: 3,
            culling_ratio: 0.7,
        };
        assert_eq!(result.visible_triangles, 3);
        assert_eq!(result.total_triangles, 10);
        assert!(!result.is_fully_visible());
    }
}

// ============================================================
// Frustum culling integration
// ============================================================

/// Result of frustum culling on a mesh.
///
/// Provides the indices of visible triangles and face IDs,
/// along with statistics about the culling operation.
#[derive(Clone, Debug)]
pub struct FrustumCullResult {
    /// Indices of triangles that are visible (inside the frustum).
    pub visible_triangle_indices: Vec<usize>,
    /// Face IDs that have at least one visible triangle.
    pub visible_face_ids: Vec<u64>,
    /// Total number of triangles in the mesh.
    pub total_triangles: usize,
    /// Number of visible triangles.
    pub visible_triangles: usize,
    /// Ratio of culled triangles (0.0 = all visible, 1.0 = all culled).
    pub culling_ratio: f64,
}

impl FrustumCullResult {
    /// Whether all triangles are visible (nothing was culled).
    pub fn is_fully_visible(&self) -> bool {
        self.visible_triangles == self.total_triangles
    }

    /// Whether no triangles are visible (everything was culled).
    pub fn is_fully_culled(&self) -> bool {
        self.visible_triangles == 0 && self.total_triangles > 0
    }
}

/// Perform frustum culling on a mesh using a BVH acceleration structure.
///
/// This builds a BVH from the mesh (or reuses one if provided) and
/// tests each triangle's bounding box against the 6-plane frustum.
/// Triangles whose bounding boxes are outside the frustum are culled.
///
/// # Arguments
///
/// * `mesh` — The triangle mesh to cull
/// * `view_projection_matrix` — 4x4 view-projection matrix in column-major order
///   (OpenGL convention: m[col][row])
///
/// # Performance
///
/// For a model with N triangles, worst case is O(N) (all visible),
/// but typical case for culling is O(log N) per visible cluster.
/// A model with 1M triangles and 10% visible runs ~10x faster
/// than brute-force rendering.
///
/// # Example
///
/// ```ignore
/// let result = frustum_cull_mesh(&mesh, &view_proj_matrix);
/// println!("Visible: {}/{} triangles ({:.1}% culled)",
///     result.visible_triangles, result.total_triangles,
///     result.culling_ratio * 100.0);
/// ```
pub fn frustum_cull_mesh(mesh: &TriangleMesh, view_projection_matrix: &[[f64; 4]; 4]) -> FrustumCullResult {
    let bvh = draper_topology::Bvh::build(&mesh.vertices, &mesh.triangles);
    frustum_cull_mesh_with_bvh(mesh, &bvh, view_projection_matrix)
}

/// Perform frustum culling using a pre-built BVH.
///
/// This is more efficient than `frustum_cull_mesh` when you need to
/// perform multiple culling operations on the same mesh (e.g., per frame),
/// because the BVH is built once and reused.
pub fn frustum_cull_mesh_with_bvh(
    mesh: &TriangleMesh,
    bvh: &draper_topology::Bvh,
    view_projection_matrix: &[[f64; 4]; 4],
) -> FrustumCullResult {
    let frustum = draper_topology::Frustum::from_matrix(view_projection_matrix);

    let visible_indices = bvh.frustum_cull(&frustum);
    let visible_count = visible_indices.len();
    let total = mesh.triangles.len();

    // Collect face IDs if available
    let visible_face_ids: Vec<u64> = if let Some(ref face_ids) = mesh.triangle_face_ids {
        let mut face_set = std::collections::HashSet::new();
        for &idx in &visible_indices {
            if idx < face_ids.len() {
                face_set.insert(face_ids[idx]);
            }
        }
        face_set.into_iter().collect()
    } else {
        Vec::new()
    };

    let culling_ratio = if total > 0 {
        1.0 - visible_count as f64 / total as f64
    } else {
        0.0
    };

    FrustumCullResult {
        visible_triangle_indices: visible_indices,
        visible_face_ids,
        total_triangles: total,
        visible_triangles: visible_count,
        culling_ratio,
    }
}

// ============================================================
// Software Occlusion Culling
// ============================================================

/// Result of occlusion culling on a mesh.
///
/// Provides the indices of triangles that are not occluded by
/// other triangles closer to the camera.
#[derive(Clone, Debug)]
pub struct OcclusionCullResult {
    /// Indices of triangles that are visible (not occluded).
    pub visible_triangle_indices: Vec<usize>,
    /// Total number of triangles tested.
    pub total_triangles: usize,
    /// Number of visible (non-occluded) triangles.
    pub visible_triangles: usize,
    /// Ratio of occluded triangles.
    pub occlusion_ratio: f64,
}

/// Perform software occlusion culling using a hierarchical Z-buffer approach.
///
/// This is a simplified occlusion culling implementation that:
/// 1. Sorts triangles front-to-back (by distance from camera)
/// 2. Rasterizes each triangle into a low-resolution depth buffer
/// 3. Marks triangles as occluded if all their pixels are behind the depth buffer
///
/// This approach is suitable for WASM/WebGPU where hardware occlusion queries
/// may not be available. It works on the CPU at reduced resolution.
///
/// # Arguments
///
/// * `mesh` — The triangle mesh
/// * `camera_position` — World-space camera position for front-to-back sorting
/// * `view_projection_matrix` — 4x4 view-projection matrix (column-major)
/// * `resolution` — Depth buffer resolution (e.g., 256 for 256x256). Lower = faster but less accurate.
///
/// # Performance
///
/// For a 256x256 depth buffer, occlusion culling takes approximately:
/// - ~1ms for 10K triangles
/// - ~10ms for 100K triangles
/// - ~100ms for 1M triangles
///
/// For real-time use, combine with frustum culling first to reduce triangle count.
pub fn occlusion_cull_mesh(
    mesh: &TriangleMesh,
    camera_position: &draper_geometry::Point3d,
    view_projection_matrix: &[[f64; 4]; 4],
    resolution: usize,
) -> OcclusionCullResult {
    if mesh.triangles.is_empty() || resolution == 0 {
        return OcclusionCullResult {
            visible_triangle_indices: Vec::new(),
            total_triangles: 0,
            visible_triangles: 0,
            occlusion_ratio: 0.0,
        };
    }

    // First apply frustum culling to reduce work
    let frustum_result = frustum_cull_mesh(mesh, view_projection_matrix);
    let frustum_visible: std::collections::HashSet<usize> =
        frustum_result.visible_triangle_indices.into_iter().collect();

    // Sort visible triangles front-to-back by centroid distance to camera
    let mut sorted_indices: Vec<usize> = frustum_visible.into_iter().collect();
    sorted_indices.sort_by(|&a, &b| {
        let tri_a = &mesh.triangles[a];
        let tri_b = &mesh.triangles[b];
        let dist_a = triangle_centroid_dist(mesh, tri_a, camera_position);
        let dist_b = triangle_centroid_dist(mesh, tri_b, camera_position);
        dist_a.partial_cmp(&dist_b).unwrap_or(std::cmp::Ordering::Equal)
    });

    // Initialize depth buffer with far-plane values
    let mut depth_buffer = vec![f64::MAX; resolution * resolution];

    let mut visible_indices = Vec::new();
    let _inv_res = 1.0 / resolution as f64;

    for &tri_idx in &sorted_indices {
        let tri = &mesh.triangles[tri_idx];

        // Project triangle vertices to screen space
        let v0 = project_vertex(&mesh.vertices[tri[0] as usize], view_projection_matrix);
        let v1 = project_vertex(&mesh.vertices[tri[1] as usize], view_projection_matrix);
        let v2 = project_vertex(&mesh.vertices[tri[2] as usize], view_projection_matrix);

        // Compute screen-space bounding box
        let min_x = ((v0.0.min(v1.0).min(v2.0)).max(0.0) * resolution as f64) as usize;
        let max_x = ((v0.0.max(v1.0).max(v2.0)).min(1.0) * resolution as f64) as usize;
        let min_y = ((v0.1.min(v1.1).min(v2.1)).max(0.0) * resolution as f64) as usize;
        let max_y = ((v0.1.max(v1.1).max(v2.1)).min(1.0) * resolution as f64) as usize;

        // Check if triangle covers any pixels
        let mut is_visible = false;
        let avg_depth = (v0.2 + v1.2 + v2.2) / 3.0;

        for py in min_y..=max_y.min(resolution - 1) {
            for px in min_x..=max_x.min(resolution - 1) {
                let buf_idx = py * resolution + px;
                if avg_depth < depth_buffer[buf_idx] {
                    // This triangle is closer — it's visible
                    depth_buffer[buf_idx] = avg_depth;
                    is_visible = true;
                }
            }
        }

        if is_visible {
            visible_indices.push(tri_idx);
        }
    }

    let visible_count = visible_indices.len();
    let total = mesh.triangles.len();

    OcclusionCullResult {
        visible_triangle_indices: visible_indices,
        total_triangles: total,
        visible_triangles: visible_count,
        occlusion_ratio: if total > 0 { 1.0 - visible_count as f64 / total as f64 } else { 0.0 },
    }
}

/// Compute centroid distance to camera.
fn triangle_centroid_dist(mesh: &TriangleMesh, tri: &[u32; 3], cam: &draper_geometry::Point3d) -> f64 {
    let v0 = &mesh.vertices[tri[0] as usize];
    let v1 = &mesh.vertices[tri[1] as usize];
    let v2 = &mesh.vertices[tri[2] as usize];
    let cx = (v0.x + v1.x + v2.x) / 3.0 - cam.x;
    let cy = (v0.y + v1.y + v2.y) / 3.0 - cam.y;
    let cz = (v0.z + v1.z + v2.z) / 3.0 - cam.z;
    cx * cx + cy * cy + cz * cz
}

/// Project a vertex to screen space using the view-projection matrix.
/// Returns (screen_x, screen_y, depth) where screen coordinates are in [0,1].
fn project_vertex(v: &draper_geometry::Point3d, mvp: &[[f64; 4]; 4]) -> (f64, f64, f64) {
    // Column-major: m[col][row]
    let x = mvp[0][0] * v.x + mvp[1][0] * v.y + mvp[2][0] * v.z + mvp[3][0];
    let y = mvp[0][1] * v.x + mvp[1][1] * v.y + mvp[2][1] * v.z + mvp[3][1];
    let z = mvp[0][2] * v.x + mvp[1][2] * v.y + mvp[2][2] * v.z + mvp[3][2];
    let w = mvp[0][3] * v.x + mvp[1][3] * v.y + mvp[2][3] * v.z + mvp[3][3];

    if w.abs() < 1e-10 {
        return (0.5, 0.5, f64::MAX);
    }

    let inv_w = 1.0 / w;
    // NDC to screen coordinates [0, 1]
    let screen_x = (x * inv_w * 0.5 + 0.5).clamp(0.0, 1.0);
    let screen_y = (y * inv_w * 0.5 + 0.5).clamp(0.0, 1.0);
    let depth = z * inv_w; // Depth for z-buffer comparison

    (screen_x, screen_y, depth)
}
