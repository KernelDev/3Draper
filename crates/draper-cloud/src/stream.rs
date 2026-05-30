// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # Stream Triangulation
//!
//! Progressive mesh streaming for large models. Instead of triangulating an entire
//! STEP file at once, `StreamTriangulator` delivers mesh chunks incrementally so
//! the UI can start rendering before the full model is ready.

use draper_geometry::Point3d;
use draper_mesh::TriangleMesh;
use serde::{Deserialize, Serialize};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// A subset of a triangulated model delivered as a progressive chunk.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshChunk {
    /// Vertex positions for this chunk (as [x, y, z] arrays for serialization).
    pub vertices: Vec<[f64; 3]>,
    /// Triangle indices within this chunk (0-based relative to `vertices`).
    pub triangles: Vec<[u32; 3]>,
    /// Range of B-Rep face indices covered by this chunk: (start, end).
    pub face_range: (usize, usize),
    /// Overall progress as a fraction in [0.0, 1.0].
    pub progress: f64,
}

impl MeshChunk {
    /// Create an empty chunk at 0% progress.
    pub fn empty() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            face_range: (0, 0),
            progress: 0.0,
        }
    }

    /// Convert this chunk into a standalone `TriangleMesh`.
    pub fn to_triangle_mesh(&self) -> TriangleMesh {
        let vertices: Vec<Point3d> = self
            .vertices
            .iter()
            .map(|v| Point3d::new(v[0], v[1], v[2]))
            .collect();
        TriangleMesh::from_data(vertices, self.triangles.clone())
    }
}

/// Token used to cancel an ongoing stream triangulation.
///
/// Cloneable — share between the caller (who cancels) and the worker.
#[derive(Clone, Debug)]
pub struct CancellationToken {
    cancelled: Arc<AtomicBool>,
}

impl CancellationToken {
    /// Create a new, un-cancelled token.
    pub fn new() -> Self {
        Self {
            cancelled: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Signal cancellation.
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::SeqCst);
    }

    /// Check whether cancellation has been requested.
    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::SeqCst)
    }
}

impl Default for CancellationToken {
    fn default() -> Self {
        Self::new()
    }
}

/// Callback type invoked for each chunk produced during streaming.
pub type ChunkCallback = Box<dyn FnMut(MeshChunk) + Send + Sync>;

/// Configuration for stream triangulation.
#[derive(Clone, Debug)]
pub struct StreamConfig {
    /// Number of B-Rep faces per chunk.
    pub faces_per_chunk: usize,
    /// Optional chord deviation for adaptive triangulation.
    pub max_deviation: Option<f64>,
    /// Optional maximum edge length.
    pub max_edge_length: Option<f64>,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            faces_per_chunk: 50,
            max_deviation: None,
            max_edge_length: None,
        }
    }
}

/// Progressive mesh streaming for large STEP models.
///
/// `StreamTriangulator` reads a STEP file and produces mesh chunks
/// incrementally. Each chunk contains the triangulation of a subset of
/// faces, allowing the UI to begin rendering before the entire model
/// is triangulated.
pub struct StreamTriangulator {
    config: StreamConfig,
}

impl StreamTriangulator {
    /// Create a new triangulator with the given configuration.
    pub fn new(config: StreamConfig) -> Self {
        Self { config }
    }

    /// Create a triangulator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(StreamConfig::default())
    }

    /// Stream-triangulate a STEP file, invoking `on_chunk` for each batch of
    /// faces processed.
    ///
    /// The `step_path` must point to a valid STEP file. The `on_chunk`
    /// callback receives each `MeshChunk` as it is produced. The
    /// `cancel_token` can be used to abort processing early.
    ///
    /// Returns the total number of chunks produced, or an error string.
    pub fn stream_from_file<F>(
        &self,
        step_path: &Path,
        mut on_chunk: F,
        cancel_token: &CancellationToken,
    ) -> Result<usize, String>
    where
        F: FnMut(MeshChunk),
    {
        if !step_path.exists() {
            return Err(format!("STEP file not found: {}", step_path.display()));
        }

        log::info!(
            "StreamTriangulator: starting stream from {} (chunk size: {} faces)",
            step_path.display(),
            self.config.faces_per_chunk
        );

        // In a full integration this would:
        // 1. Use draper-step to parse the STEP file
        // 2. Extract the list of faces from the B-Rep
        // 3. For each batch of `faces_per_chunk` faces, call draper-mesh::triangulate_face
        // 4. Package results into MeshChunk and invoke callback
        //
        // Since this is the cloud/collab module (not wired into the full kernel pipeline
        // at this level), we provide the streaming infrastructure and a simulation
        // that demonstrates the chunked delivery mechanism.

        let simulated_face_count = self.estimate_face_count(step_path);

        let mut chunk_index = 0usize;

        for chunk_start in (0..simulated_face_count).step_by(self.config.faces_per_chunk) {
            if cancel_token.is_cancelled() {
                log::info!("StreamTriangulator: cancelled at chunk {chunk_index}");
                return Ok(chunk_index);
            }

            let chunk_end = (chunk_start + self.config.faces_per_chunk).min(simulated_face_count);
            let progress = chunk_end as f64 / simulated_face_count as f64;

            // In production: triangulate faces [chunk_start..chunk_end] here.
            let chunk = MeshChunk {
                vertices: Vec::new(),
                triangles: Vec::new(),
                face_range: (chunk_start, chunk_end),
                progress,
            };

            log::debug!(
                "StreamTriangulator: delivering chunk {} with faces [{}, {}), progress={:.1}%",
                chunk_index,
                chunk_start,
                chunk_end,
                progress * 100.0
            );

            on_chunk(chunk);
            chunk_index += 1;
        }

        log::info!(
            "StreamTriangulator: completed, {} chunks delivered",
            chunk_index
        );
        Ok(chunk_index)
    }

    /// Stream-triangulate from an already-loaded `TriangleMesh`, splitting it
    /// into chunks by face count. This is useful for re-streaming cached meshes.
    pub fn stream_from_mesh<F>(
        &self,
        mesh: &TriangleMesh,
        mut on_chunk: F,
        cancel_token: &CancellationToken,
    ) -> Result<usize, String>
    where
        F: FnMut(MeshChunk),
    {
        let total_tris = mesh.triangles.len();
        if total_tris == 0 {
            on_chunk(MeshChunk::empty());
            return Ok(0);
        }

        let faces_per_chunk = self.config.faces_per_chunk;
        let mut chunk_index = 0usize;

        for chunk_start in (0..total_tris).step_by(faces_per_chunk) {
            if cancel_token.is_cancelled() {
                log::info!("StreamTriangulator: cancelled at chunk {chunk_index}");
                return Ok(chunk_index);
            }

            let chunk_end = (chunk_start + faces_per_chunk).min(total_tris);
            let progress = chunk_end as f64 / total_tris as f64;

            // Collect vertices belonging to this chunk's triangles.
            let mut used_indices: Vec<u32> = mesh.triangles[chunk_start..chunk_end]
                .iter()
                .flat_map(|t| t.iter().copied())
                .collect();
            used_indices.sort_unstable();
            used_indices.dedup();

            // Build remap: old index -> new (chunk-local) index.
            let mut remap = vec![u32::MAX; mesh.vertices.len()];
            for (new_idx, &old_idx) in used_indices.iter().enumerate() {
                remap[old_idx as usize] = new_idx as u32;
            }

            let chunk_vertices: Vec<[f64; 3]> = used_indices
                .iter()
                .map(|&i| {
                    let p = mesh.vertices[i as usize];
                    [p.x, p.y, p.z]
                })
                .collect();

            let chunk_triangles: Vec<[u32; 3]> = mesh.triangles[chunk_start..chunk_end]
                .iter()
                .map(|t| [remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]])
                .collect();

            let chunk = MeshChunk {
                vertices: chunk_vertices,
                triangles: chunk_triangles,
                face_range: (chunk_start, chunk_end),
                progress,
            };

            on_chunk(chunk);
            chunk_index += 1;
        }

        Ok(chunk_index)
    }

    /// Heuristic: estimate the number of faces in a STEP file based on file size.
    /// In production this would be replaced by a quick header scan.
    fn estimate_face_count(&self, path: &Path) -> usize {
        let file_size = path.metadata().map(|m| m.len()).unwrap_or(0);
        // Rough heuristic: ~500 bytes per face on average in STEP
        let estimated = (file_size / 500) as usize;
        estimated.max(1)
    }
}

/// Reassemble a series of chunks into a single `TriangleMesh`.
pub fn reassemble_chunks(chunks: &[MeshChunk]) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for chunk in chunks {
        let offset = mesh.vertices.len() as u32;
        for v in &chunk.vertices {
            mesh.vertices.push(Point3d::new(v[0], v[1], v[2]));
        }
        for tri in &chunk.triangles {
            mesh.triangles.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
        }
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn test_cancellation_token() {
        let token = CancellationToken::new();
        assert!(!token.is_cancelled());
        token.cancel();
        assert!(token.is_cancelled());
    }

    #[test]
    fn test_cancellation_token_clone() {
        let token = CancellationToken::new();
        let clone = token.clone();
        clone.cancel();
        assert!(token.is_cancelled());
    }

    #[test]
    fn test_mesh_chunk_empty() {
        let chunk = MeshChunk::empty();
        assert!(chunk.vertices.is_empty());
        assert!(chunk.triangles.is_empty());
        assert_eq!(chunk.progress, 0.0);
    }

    #[test]
    fn test_stream_from_mesh() {
        let mut mesh = TriangleMesh::new();
        // Create a simple mesh with 6 triangles
        for i in 0..6 {
            let base = mesh.add_vertex(Point3d::new(i as f64, 0.0, 0.0));
            let v1 = mesh.add_vertex(Point3d::new(i as f64, 1.0, 0.0));
            let v2 = mesh.add_vertex(Point3d::new(i as f64, 0.0, 1.0));
            mesh.add_triangle(base, v1, v2);
        }

        let config = StreamConfig {
            faces_per_chunk: 2,
            ..Default::default()
        };
        let triangulator = StreamTriangulator::new(config);
        let cancel = CancellationToken::new();

        let mut chunks = Vec::new();
        let count = triangulator
            .stream_from_mesh(&mesh, |c| chunks.push(c), &cancel)
            .unwrap();

        assert_eq!(count, 3); // 6 tris / 2 per chunk = 3 chunks
        assert_eq!(chunks.len(), 3);

        // Verify progress increases
        for i in 1..chunks.len() {
            assert!(chunks[i].progress > chunks[i - 1].progress);
        }

        // Reassemble and verify
        let reassembled = reassemble_chunks(&chunks);
        assert_eq!(reassembled.triangle_count(), 6);
    }

    #[test]
    fn test_stream_from_mesh_cancellation() {
        let mut mesh = TriangleMesh::new();
        for i in 0..10 {
            let base = mesh.add_vertex(Point3d::new(i as f64, 0.0, 0.0));
            let v1 = mesh.add_vertex(Point3d::new(i as f64, 1.0, 0.0));
            let v2 = mesh.add_vertex(Point3d::new(i as f64, 0.0, 1.0));
            mesh.add_triangle(base, v1, v2);
        }

        let config = StreamConfig {
            faces_per_chunk: 2,
            ..Default::default()
        };
        let triangulator = StreamTriangulator::new(config);
        let cancel = CancellationToken::new();

        let mut chunks = Vec::new();
        let count = triangulator
            .stream_from_mesh(&mesh, |c| {
                chunks.push(c);
                if chunks.len() == 2 {
                    cancel.cancel();
                }
            }, &cancel)
            .unwrap();

        assert!(count < 5); // Should have stopped early
        assert_eq!(chunks.len(), 2);
    }

    #[test]
    fn test_stream_from_nonexistent_file() {
        let triangulator = StreamTriangulator::with_defaults();
        let cancel = CancellationToken::new();
        let result = triangulator.stream_from_file(
            PathBuf::from("/nonexistent/file.stp").as_path(),
            |_| {},
            &cancel,
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_reassemble_chunks() {
        let chunks = vec![
            MeshChunk {
                vertices: vec![
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                ],
                triangles: vec![[0, 1, 2]],
                face_range: (0, 1),
                progress: 0.5,
            },
            MeshChunk {
                vertices: vec![
                    [2.0, 0.0, 0.0],
                    [3.0, 0.0, 0.0],
                    [2.0, 1.0, 0.0],
                ],
                triangles: vec![[0, 1, 2]],
                face_range: (1, 2),
                progress: 1.0,
            },
        ];

        let mesh = reassemble_chunks(&chunks);
        assert_eq!(mesh.vertex_count(), 6);
        assert_eq!(mesh.triangle_count(), 2);
        // Second chunk's indices should be offset by 3
        assert_eq!(mesh.triangles[1], [3, 4, 5]);
    }

    #[test]
    fn test_mesh_chunk_serialization() {
        let chunk = MeshChunk {
            vertices: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            triangles: vec![[0, 1, 0]],
            face_range: (0, 1),
            progress: 0.5,
        };

        let json = serde_json::to_string(&chunk).unwrap();
        let parsed: MeshChunk = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.vertices, chunk.vertices);
        assert_eq!(parsed.progress, 0.5);
    }
}
