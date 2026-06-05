// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Watertight mesh validation for B-Rep solid triangulation.
//!
//! After triangulating all faces of a solid and merging them into a single mesh,
//! this module checks that the result is **watertight**: every edge must be shared
//! by exactly 2 triangles. If any edge has count != 2, the mesh has gaps (count=1)
//! or self-intersections (count>2).
//!
//! # Usage
//! ```rust,ignore
//! use draper_mesh::watertight::validate_watertight;
//!
//! let report = validate_watertight(&merged_mesh, true);
//! if report.is_watertight() {
//!     println!("Mesh is watertight!");
//! } else {
//!     println!("Mesh has {} boundary edges, {} non-manifold edges",
//!              report.boundary_edge_count, report.non_manifold_edge_count);
//! }
//! ```

use crate::mesh::TriangleMesh;
use draper_geometry::Point3d;
use std::collections::HashMap;

/// Result of watertight validation on a merged solid mesh.
#[derive(Clone, Debug)]
pub struct WatertightReport {
    /// Total number of unique edges in the mesh.
    pub edge_count: usize,
    /// Number of edges shared by exactly 2 triangles (good — interior edges).
    pub interior_edge_count: usize,
    /// Number of edges with only 1 adjacent triangle (boundary/gap edges).
    pub boundary_edge_count: usize,
    /// Number of edges with more than 2 adjacent triangles (non-manifold).
    pub non_manifold_edge_count: usize,
    /// Total number of triangles.
    pub triangle_count: usize,
    /// Total number of vertices.
    pub vertex_count: usize,
    /// Euler characteristic: V - E + F.
    pub euler_characteristic: i64,
    /// Number of degenerate triangles (zero area).
    pub degenerate_triangle_count: usize,
    /// Number of duplicate triangles (identical vertex sets).
    pub duplicate_triangle_count: usize,
    /// Boundary edges as vertex pairs (for debugging).
    pub boundary_edges: Vec<(u32, u32)>,
    /// Non-manifold edges as (vertex_a, vertex_b, face_count).
    pub non_manifold_edges: Vec<(u32, u32, u32)>,
    /// Per-face-id watertight summary (if face_ids are available).
    pub per_face_summary: HashMap<u64, FaceWatertightSummary>,
}

/// Watertight summary for a single face's triangles.
#[derive(Clone, Debug, Default)]
pub struct FaceWatertightSummary {
    /// Number of triangles in this face.
    pub triangle_count: usize,
    /// Number of boundary edges (edges only in this face, not shared with another face).
    pub boundary_edge_count: usize,
}

impl WatertightReport {
    /// Check if the mesh is watertight: no boundary edges, no non-manifold edges.
    pub fn is_watertight(&self) -> bool {
        self.boundary_edge_count == 0 && self.non_manifold_edge_count == 0
    }

    /// Check if the mesh is a 2-manifold: no non-manifold edges (boundary is OK).
    pub fn is_manifold(&self) -> bool {
        self.non_manifold_edge_count == 0
    }
}

/// Validate that a merged solid mesh is watertight.
///
/// For a closed solid, every edge should be shared by exactly 2 triangles.
/// This function counts the number of adjacent triangles for each edge
/// and classifies edges as:
/// - Interior (count == 2): correct for a closed solid
/// - Boundary (count == 1): gap/crack — mesh is not watertight
/// - Non-manifold (count > 2): self-intersection or T-junction
///
/// # Arguments
/// * `mesh` — The merged triangle mesh from all faces of the solid.
/// * `verbose` — If true, collect per-face summaries and boundary edge lists
///   (slightly slower due to extra bookkeeping).
pub fn validate_watertight(mesh: &TriangleMesh, verbose: bool) -> WatertightReport {
    let vertex_count = mesh.vertices.len();
    let triangle_count = mesh.triangles.len();

    // Build edge → (face_count, list of face_ids) map
    // Key: canonical edge (smaller vertex index first)
    let mut edge_face_count: HashMap<(u32, u32), EdgeInfo> = HashMap::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let v0 = tri[0];
        let v1 = tri[1];
        let v2 = tri[2];

        let edges = [
            (v0.min(v1), v0.max(v1)),
            (v1.min(v2), v1.max(v2)),
            (v2.min(v0), v2.max(v0)),
        ];

        let face_id = mesh.triangle_face_ids.as_ref()
            .and_then(|ids| ids.get(tri_idx).copied())
            .unwrap_or(0);

        for edge in &edges {
            let info = edge_face_count.entry(*edge).or_insert(EdgeInfo::default());
            info.count += 1;
            if verbose && face_id != 0 {
                info.face_ids.push(face_id);
            }
        }
    }

    // Count degenerate triangles
    let mut degenerate_triangle_count = 0;
    for tri in &mesh.triangles {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
            degenerate_triangle_count += 1;
            continue;
        }
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let area = triangle_area_3d(&v0, &v1, &v2);
        if area < 1e-20 {
            degenerate_triangle_count += 1;
        }
    }

    // Count duplicate triangles
    let mut duplicate_triangle_count = 0;
    {
        let mut tri_set: HashMap<[u32; 3], usize> = HashMap::new();
        for tri in &mesh.triangles {
            let mut sorted = [tri[0], tri[1], tri[2]];
            sorted.sort();
            *tri_set.entry(sorted).or_insert(0) += 1;
        }
        for &count in tri_set.values() {
            if count > 1 {
                duplicate_triangle_count += count - 1;
            }
        }
    }

    // Classify edges
    let edge_count = edge_face_count.len();
    let mut interior_edge_count = 0;
    let mut boundary_edge_count = 0;
    let mut non_manifold_edge_count = 0;
    let mut boundary_edges = Vec::new();
    let mut non_manifold_edges = Vec::new();

    for (edge, info) in &edge_face_count {
        match info.count {
            1 => {
                boundary_edge_count += 1;
                if verbose {
                    boundary_edges.push(*edge);
                }
            }
            2 => {
                interior_edge_count += 1;
            }
            _ => {
                non_manifold_edge_count += 1;
                if verbose {
                    non_manifold_edges.push((edge.0, edge.1, info.count));
                }
            }
        }
    }

    // Euler characteristic: V - E + F
    let euler = vertex_count as i64 - edge_count as i64 + triangle_count as i64;

    // Per-face summary (if face_ids available)
    let per_face_summary = if verbose && mesh.triangle_face_ids.is_some() {
        compute_per_face_summary(mesh, &edge_face_count)
    } else {
        HashMap::new()
    };

    WatertightReport {
        edge_count,
        interior_edge_count,
        boundary_edge_count,
        non_manifold_edge_count,
        triangle_count,
        vertex_count,
        euler_characteristic: euler,
        degenerate_triangle_count,
        duplicate_triangle_count,
        boundary_edges,
        non_manifold_edges,
        per_face_summary,
    }
}

/// Internal edge info during validation.
#[derive(Clone, Debug, Default)]
struct EdgeInfo {
    count: u32,
    face_ids: Vec<u64>,
}

/// Compute per-face watertight summary.
///
/// For each face, counts how many of its edges are boundary edges
/// (not shared with another face). A fully watertight face should have
/// all its edges shared with at least one other face.
fn compute_per_face_summary(
    mesh: &TriangleMesh,
    edge_face_count: &HashMap<(u32, u32), EdgeInfo>,
) -> HashMap<u64, FaceWatertightSummary> {
    let mut summary: HashMap<u64, FaceWatertightSummary> = HashMap::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let face_id = mesh.triangle_face_ids.as_ref()
            .and_then(|ids| ids.get(tri_idx).copied())
            .unwrap_or(0);

        if face_id == 0 {
            continue;
        }

        let entry = summary.entry(face_id).or_default();
        entry.triangle_count += 1;

        // Check each edge of this triangle
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[2].min(tri[0]), tri[2].max(tri[0])),
        ];

        for edge in &edges {
            if let Some(info) = edge_face_count.get(edge) {
                // An edge is a "boundary edge for this face" if it only touches
                // triangles from this same face (count == 1 or all from same face_id).
                // For a true boundary edge (count==1), it's definitely not shared.
                if info.count == 1 {
                    entry.boundary_edge_count += 1;
                }
                // Note: We don't count count==2 edges where both faces have the same
                // face_id as boundary (that would be a self-fold, very rare).
            }
        }
    }

    summary
}

/// Compute the 3D area of a triangle.
fn triangle_area_3d(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let e1x = v1.x - v0.x;
    let e1y = v1.y - v0.y;
    let e1z = v1.z - v0.z;
    let e2x = v2.x - v0.x;
    let e2y = v2.y - v0.y;
    let e2z = v2.z - v0.z;
    let cx = e1y * e2z - e1z * e2y;
    let cy = e1z * e2x - e1x * e2z;
    let cz = e1x * e2y - e1y * e2x;
    (cx * cx + cy * cy + cz * cz).sqrt() * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a closed cube mesh (8 vertices, 12 triangles)
    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v = [
            Point3d::new(0.0, 0.0, 0.0), // 0
            Point3d::new(1.0, 0.0, 0.0), // 1
            Point3d::new(1.0, 1.0, 0.0), // 2
            Point3d::new(0.0, 1.0, 0.0), // 3
            Point3d::new(0.0, 0.0, 1.0), // 4
            Point3d::new(1.0, 0.0, 1.0), // 5
            Point3d::new(1.0, 1.0, 1.0), // 6
            Point3d::new(0.0, 1.0, 1.0), // 7
        ];
        for p in &v {
            mesh.add_vertex(*p);
        }
        // Bottom (z=0)
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        // Top (z=1)
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        // Front (y=0)
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        // Back (y=1)
        mesh.add_triangle(3, 7, 6);
        mesh.add_triangle(3, 6, 2);
        // Left (x=0)
        mesh.add_triangle(0, 4, 7);
        mesh.add_triangle(0, 7, 3);
        // Right (x=1)
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);
        mesh
    }

    #[test]
    fn test_cube_watertight() {
        let mesh = make_cube_mesh();
        let report = validate_watertight(&mesh, true);
        assert!(report.is_watertight(),
            "Cube should be watertight, but has {} boundary edges, {} non-manifold edges",
            report.boundary_edge_count, report.non_manifold_edge_count);
        assert_eq!(report.euler_characteristic, 2,
            "Cube Euler characteristic should be 2");
        assert_eq!(report.degenerate_triangle_count, 0);
        assert_eq!(report.duplicate_triangle_count, 0);
    }

    #[test]
    fn test_open_mesh_has_boundary() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let report = validate_watertight(&mesh, false);
        assert!(!report.is_watertight());
        assert_eq!(report.boundary_edge_count, 3);
        assert_eq!(report.interior_edge_count, 0);
    }

    #[test]
    fn test_non_manifold_edge_detected() {
        // Create a non-manifold configuration: two triangles sharing an edge
        // with a third triangle also sharing that edge
        let mut mesh = TriangleMesh::new();
        // 4 vertices: two triangles sharing edge 0-1, and a third also sharing it
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));  // 0
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));  // 1
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));  // 2
        mesh.add_vertex(Point3d::new(0.5, -1.0, 0.0)); // 3

        mesh.add_triangle(0, 1, 2); // Edge 0-1 shared by all 3
        mesh.add_triangle(0, 1, 3);
        // This creates a non-manifold edge: 0-1 has count=2 (still OK)
        // Let's add a third triangle to make it non-manifold
        mesh.add_vertex(Point3d::new(0.5, 0.5, 1.0)); // 4
        mesh.add_triangle(0, 1, 4); // Now edge 0-1 has count=3

        let report = validate_watertight(&mesh, true);
        assert!(!report.is_manifold());
        assert!(report.non_manifold_edge_count > 0);
    }

    #[test]
    fn test_tetrahedron_watertight() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(1.0, 1.0, 1.0));
        mesh.add_vertex(Point3d::new(1.0, -1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, 1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, -1.0, 1.0));
        mesh.add_triangle(0, 1, 2);
        mesh.add_triangle(0, 1, 3);
        mesh.add_triangle(0, 2, 3);
        mesh.add_triangle(1, 2, 3);

        let report = validate_watertight(&mesh, false);
        assert!(report.is_watertight(), "Tetrahedron should be watertight");
        assert_eq!(report.euler_characteristic, 2);
    }

    #[test]
    fn test_per_face_summary() {
        let mut mesh = make_cube_mesh();
        // Assign face IDs: 2 triangles per face, 6 faces
        mesh.triangle_face_ids = Some(vec![1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6]);

        let report = validate_watertight(&mesh, true);
        assert!(report.is_watertight());
        // Each face should have 2 triangles and 0 boundary edges
        for (&face_id, summary) in &report.per_face_summary {
            assert_eq!(summary.triangle_count, 2,
                "Face {} should have 2 triangles, got {}", face_id, summary.triangle_count);
        }
    }

    #[test]
    fn test_degenerate_triangle_detected() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // Same as vertex 0
        mesh.add_triangle(0, 1, 2);

        let report = validate_watertight(&mesh, false);
        assert!(report.degenerate_triangle_count > 0);
    }
}
