// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! CDT-based solid triangulation with tolerance-aware vertex unification.
//!
//! This module provides watertight mesh generation by:
//! 1. Triangulating each face independently using the standard pipeline
//! 2. Unifying vertices across ALL faces using 3D spatial hashing with tolerance
//! 3. Remapping all triangle vertex indices to the unified vertex set
//! 4. Stitching remaining boundary edges with a progressive tolerance sweep
//!
//! The key insight: by unifying boundary vertices AFTER per-face triangulation,
//! shared edges between adjacent faces automatically get identical vertex indices.
//! Interior vertices of each face are preserved (not discarded), and only
//! boundary vertices near shared edges are merged.

use crate::mesh::TriangleMesh;
use crate::triangulate::{
    TriangulationParams, triangulate_face, filter_degenerate_triangles,
};
use crate::watertight::stitch_boundary_edges;
use draper_geometry::Point3d;
use draper_topology::{Solid, TopoId};
use std::collections::HashMap;

/// A unified vertex pool that merges vertices across all faces of a solid.
///
/// When two faces share an edge, their boundary vertices should be the same.
/// This pool ensures that by:
/// 1. Inserting all vertices with 3D spatial hashing
/// 2. Merging vertices within the given tolerance
/// 3. Mapping each face's local vertex indices to unified indices
#[derive(Clone, Debug)]
pub struct UnifiedVertexPool {
    /// All unique vertices (after merging).
    pub vertices: Vec<Point3d>,
    /// Spatial hash grid for fast near-neighbor lookup.
    grid: HashMap<(i64, i64, i64), Vec<u32>>,
    /// Cell size for the spatial hash grid.
    cell_size: f64,
    /// Tolerance for vertex merging (squared).
    tolerance_sq: f64,
}

impl UnifiedVertexPool {
    /// Create a new vertex pool with the given merge tolerance.
    pub fn new(tolerance: f64) -> Self {
        Self {
            vertices: Vec::new(),
            grid: HashMap::new(),
            cell_size: tolerance * 10.0,
            tolerance_sq: tolerance * tolerance,
        }
    }

    /// Insert a vertex and return its unified index.
    ///
    /// If a vertex within `tolerance` already exists, returns the existing index.
    /// Otherwise, adds the vertex and returns a new index.
    pub fn insert(&mut self, point: Point3d) -> u32 {
        let cx = (point.x / self.cell_size).floor() as i64;
        let cy = (point.y / self.cell_size).floor() as i64;
        let cz = (point.z / self.cell_size).floor() as i64;

        // Check neighboring cells for existing vertex within tolerance
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(indices) = self.grid.get(&key) {
                        for &idx in indices {
                            let existing = &self.vertices[idx as usize];
                            let ddx = point.x - existing.x;
                            let ddy = point.y - existing.y;
                            let ddz = point.z - existing.z;
                            if ddx * ddx + ddy * ddy + ddz * ddz < self.tolerance_sq {
                                return idx;
                            }
                        }
                    }
                }
            }
        }

        // No match found — add new vertex
        let new_idx = self.vertices.len() as u32;
        self.vertices.push(point);
        self.grid.entry((cx, cy, cz)).or_default().push(new_idx);
        new_idx
    }

    /// Number of unique vertices.
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Whether the pool is empty.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }
}

/// Triangulate a solid into a watertight mesh using unified vertex merging
/// and boundary edge stitching.
///
/// # Algorithm
///
/// **Phase 1 — Per-face triangulation**: Each face is triangulated independently
/// using the standard `triangulate_face()` pipeline. This produces correct
/// per-face meshes with boundary vertices sampled from edge curves.
///
/// **Phase 2 — Vertex unification**: All vertices from all face meshes are
/// inserted into a `UnifiedVertexPool` with the given `merge_tolerance`.
/// Boundary vertices on shared edges between adjacent faces get the same
/// unified index, producing watertight edges by construction. Interior
/// vertices (inside a face, not on any boundary) get their own unique indices.
///
/// **Phase 3 — Mesh assembly**: All face meshes are combined using the
/// unified vertex indices. Degenerate triangles are filtered.
///
/// **Phase 4 — Edge stitching**: Remaining boundary edges (caused by vertices
/// that are slightly beyond the merge tolerance) are closed by progressively
/// increasing the stitch tolerance.
///
/// # Arguments
/// * `solid` — The B-Rep solid to triangulate.
/// * `params` — Triangulation parameters.
/// * `merge_tolerance` — 3D distance tolerance for merging boundary vertices.
///   Should be proportional to the model's bounding box diagonal.
///   Typical values: 1e-3 to 5e-3 for models with diagonal ~200.
pub fn triangulate_solid_watertight(
    solid: &Solid,
    params: &TriangulationParams,
    merge_tolerance: f64,
) -> TriangleMesh {
    let faces = solid.faces();
    if faces.is_empty() {
        return TriangleMesh::new();
    }

    // ============================================================
    // Phase 1: Triangulate each face independently
    // ============================================================

    let mut face_meshes: Vec<TriangleMesh> = Vec::with_capacity(faces.len());
    let mut face_ids: Vec<Option<TopoId>> = Vec::with_capacity(faces.len());

    for face in &faces {
        let face_mesh = triangulate_face(face, params);
        face_meshes.push(face_mesh);
        face_ids.push(Some(face.id));
    }

    // ============================================================
    // Phase 2: Build unified vertex pool from ALL face meshes
    // ============================================================

    let mut pool = UnifiedVertexPool::new(merge_tolerance);

    // For each face mesh, map local vertex indices → unified vertex indices
    let mut face_remaps: Vec<Vec<u32>> = Vec::with_capacity(faces.len());

    for (face_idx, mesh) in face_meshes.iter().enumerate() {
        let mut remap = Vec::with_capacity(mesh.vertices.len());

        for vertex in &mesh.vertices {
            let unified_idx = pool.insert(*vertex);
            remap.push(unified_idx);
        }

        face_remaps.push(remap);
    }

    log::info!(
        "triangulate_solid_watertight: {} faces, {} total vertices → {} unified vertices (merge_tol={:.6})",
        faces.len(),
        face_meshes.iter().map(|m| m.vertices.len()).sum::<usize>(),
        pool.len(),
        merge_tolerance,
    );

    // ============================================================
    // Phase 3: Assemble the final mesh using unified indices
    // ============================================================

    let mut mesh = TriangleMesh::new();

    // Add all unified vertices
    for &point in &pool.vertices {
        mesh.add_vertex(point);
    }

    // Add triangles from each face with remapped indices
    for (face_idx, face_mesh) in face_meshes.iter().enumerate() {
        let remap = &face_remaps[face_idx];
        let face_id = face_ids[face_idx].map(|id| id.to_u64()).unwrap_or(0);

        for (tri_idx, tri) in face_mesh.triangles.iter().enumerate() {
            let a = remap[tri[0] as usize];
            let b = remap[tri[1] as usize];
            let c = remap[tri[2] as usize];

            // Skip degenerate triangles (collapsed after vertex merging)
            if a != b && b != c && a != c {
                // Bounds check
                let max_idx = pool.len() as u32;
                if a < max_idx && b < max_idx && c < max_idx {
                    mesh.add_triangle(a, b, c);

                    // Track face ID for per-face analysis
                    mesh.triangle_face_ids
                        .get_or_insert_with(Vec::new)
                        .push(face_id);

                    // Preserve face normals if available
                    if let Some(ref face_normals) = face_mesh.face_normals {
                        if let Some(&n) = face_normals.get(tri_idx) {
                            mesh.face_normals
                                .get_or_insert_with(Vec::new)
                                .push(n);
                        }
                    }
                }
            }
        }

        // Initialize face_ids for faces that were skipped earlier
        // (ensures all per-triangle arrays stay in sync)
    }

    // Ensure face_normals and triangle_face_ids lengths match triangles
    if let Some(ref mut ids) = mesh.triangle_face_ids {
        while ids.len() < mesh.triangles.len() {
            ids.push(0);
        }
    }
    if let Some(ref mut normals) = mesh.face_normals {
        while normals.len() < mesh.triangles.len() {
            normals.push([0.0, 0.0, 1.0]);
        }
    }

    // Filter degenerate triangles
    filter_degenerate_triangles(&mut mesh, 1e-10);

    // ============================================================
    // Phase 4: Progressive edge stitching for remaining gaps
    // ============================================================

    // Start with merge_tolerance and progressively increase
    let stitch_tolerances = [
        merge_tolerance * 2.0,
        merge_tolerance * 5.0,
        merge_tolerance * 10.0,
        merge_tolerance * 50.0,
    ];

    for &stitch_tol in &stitch_tolerances {
        let report = crate::watertight::validate_watertight(&mesh, false);
        if report.is_watertight() {
            break;
        }
        if report.boundary_edge_count == 0 {
            break;
        }
        log::info!(
            "Edge stitching: {} boundary edges remaining, trying stitch_tol={:.6}",
            report.boundary_edge_count, stitch_tol,
        );
        stitch_boundary_edges(&mut mesh, stitch_tol, 3);
        filter_degenerate_triangles(&mut mesh, 1e-10);
    }

    // Final validation
    let report = crate::watertight::validate_watertight(&mesh, false);
    if !report.is_watertight() {
        log::warn!(
            "triangulate_solid_watertight: mesh still has {} boundary edges, {} non-manifold edges after stitching",
            report.boundary_edge_count, report.non_manifold_edge_count,
        );
    }

    mesh
}

/// Compute an adaptive merge tolerance based on the solid's bounding box.
///
/// The tolerance is set to `diagonal * factor`, clamped to [min, max].
/// This ensures that the tolerance is proportional to the model size,
/// which is critical for merging boundary vertices that differ due to
/// floating-point precision in curve parameterization.
///
/// Typical factor: 1e-4 (0.01% of the diagonal)
pub fn adaptive_merge_tolerance(
    solid: &Solid,
    factor: f64,
    min_tol: f64,
    max_tol: f64,
) -> f64 {
    // Compute bounding box from all face vertices
    let faces = solid.faces();
    let mut min_pt = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max_pt = Point3d::new(f64::MIN, f64::MIN, f64::MIN);

    for face in &faces {
        for edge in &face.edges {
            if let Some(ref curve) = edge.curve {
                let (tmin, tmax) = edge.param_range;
                let (pmin, pmax) = if tmin <= tmax { (tmin, tmax) } else { (tmax, tmin) };
                let p0 = curve.point_at(pmin);
                let p1 = curve.point_at(pmax);
                let pm = curve.point_at((pmin + pmax) * 0.5);

                for p in &[p0, p1, pm] {
                    min_pt.x = min_pt.x.min(p.x);
                    min_pt.y = min_pt.y.min(p.y);
                    min_pt.z = min_pt.z.min(p.z);
                    max_pt.x = max_pt.x.max(p.x);
                    max_pt.y = max_pt.y.max(p.y);
                    max_pt.z = max_pt.z.max(p.z);
                }
            }
        }
    }

    let dx = max_pt.x - min_pt.x;
    let dy = max_pt.y - min_pt.y;
    let dz = max_pt.z - min_pt.z;
    let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();

    let tol = diagonal * factor;
    tol.clamp(min_tol, max_tol)
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    #[test]
    fn test_unified_vertex_pool_merge() {
        let mut pool = UnifiedVertexPool::new(1e-3);

        let a = pool.insert(Point3d::new(0.0, 0.0, 0.0));
        let b = pool.insert(Point3d::new(1.0, 0.0, 0.0));
        // Close to a — should merge
        let c = pool.insert(Point3d::new(0.0001, 0.0001, 0.0001));

        assert_eq!(pool.len(), 2, "Should have 2 unique vertices (a≈c merged)");
        assert_eq!(a, c, "Close vertices should get the same index");
        assert_ne!(a, b, "Distinct vertices should get different indices");
    }

    #[test]
    fn test_unified_vertex_pool_no_false_merge() {
        let mut pool = UnifiedVertexPool::new(1e-3);

        let a = pool.insert(Point3d::new(0.0, 0.0, 0.0));
        let b = pool.insert(Point3d::new(0.1, 0.0, 0.0)); // 0.1 apart — should NOT merge at 1e-3

        assert_eq!(pool.len(), 2, "Should have 2 unique vertices");
        assert_ne!(a, b, "Vertices 0.1 apart should not merge at 1e-3 tolerance");
    }
}
