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
}
