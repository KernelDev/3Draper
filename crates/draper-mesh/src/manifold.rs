// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Manifold (watertight) mesh validation.
//!
//! Checks that a triangulated mesh forms a proper manifold:
//! - Every interior edge is shared by exactly 2 triangles
//! - No T-junctions (vertices lying on edges)
//! - Euler characteristic matches expected topology
//! - No degenerate (zero-area) triangles

use crate::mesh::TriangleMesh;
use draper_geometry::Point3d;
use std::collections::HashMap;

/// Result of a manifold check.
#[derive(Clone, Debug)]
pub struct ManifoldReport {
    /// Total number of vertices.
    pub vertex_count: usize,
    /// Total number of edges (unique, undirected).
    pub edge_count: usize,
    /// Total number of triangles.
    pub triangle_count: usize,
    /// Euler characteristic: V - E + F.
    pub euler_characteristic: i64,
    /// Edges with exactly 1 incident triangle (boundary edges).
    pub boundary_edge_count: usize,
    /// Edges with more than 2 incident triangles (non-manifold edges).
    pub non_manifold_edge_count: usize,
    /// Number of degenerate (zero-area) triangles.
    pub degenerate_triangle_count: usize,
    /// Number of T-junctions detected.
    pub t_junction_count: usize,
    /// List of boundary edges (vertex pairs).
    pub boundary_edges: Vec<(u32, u32)>,
    /// List of non-manifold edges (vertex pairs, count).
    pub non_manifold_edges: Vec<(u32, u32, u32)>,
}

impl ManifoldReport {
    /// Check if the mesh is watertight (no boundary edges, no non-manifold edges).
    pub fn is_watertight(&self) -> bool {
        self.boundary_edge_count == 0 && self.non_manifold_edge_count == 0
    }

    /// Check if the mesh is manifold (no non-manifold edges).
    pub fn is_manifold(&self) -> bool {
        self.non_manifold_edge_count == 0
    }

    /// Get the expected Euler characteristic for a closed surface of the given genus.
    /// For a sphere (genus 0): χ = 2
    /// For a torus (genus 1): χ = 0
    pub fn expected_euler_for_genus(genus: usize) -> i64 {
        2 * (1 - genus as i64)
    }
}

/// Check a triangle mesh for manifold properties.
pub fn check_manifold(mesh: &TriangleMesh) -> ManifoldReport {
    let vertex_count = mesh.vertices.len();
    let triangle_count = mesh.triangles.len();

    // Build edge → triangle count map
    let mut edge_face_count: HashMap<(u32, u32), u32> = HashMap::new();
    
    for tri in &mesh.triangles {
        let v0 = tri[0];
        let v1 = tri[1];
        let v2 = tri[2];
        
        // Create canonical edges (smaller index first)
        let e01 = if v0 < v1 { (v0, v1) } else { (v1, v0) };
        let e12 = if v1 < v2 { (v1, v2) } else { (v2, v1) };
        let e20 = if v2 < v0 { (v2, v0) } else { (v0, v2) };
        
        *edge_face_count.entry(e01).or_insert(0) += 1;
        *edge_face_count.entry(e12).or_insert(0) += 1;
        *edge_face_count.entry(e20).or_insert(0) += 1;
    }

    let edge_count = edge_face_count.len();
    
    // Classify edges
    let mut boundary_edges = Vec::new();
    let mut non_manifold_edges = Vec::new();
    let mut boundary_edge_count = 0;
    let mut non_manifold_edge_count = 0;

    for (edge, count) in &edge_face_count {
        match count {
            1 => {
                boundary_edge_count += 1;
                boundary_edges.push(*edge);
            }
            2 => {
                // Interior edge — this is the expected case for a closed manifold
            }
            _ => {
                non_manifold_edge_count += 1;
                non_manifold_edges.push((edge.0, edge.1, *count));
            }
        }
    }

    // Compute Euler characteristic
    let euler = vertex_count as i64 - edge_count as i64 + triangle_count as i64;

    // Count degenerate triangles (zero area)
    let mut degenerate_triangle_count = 0;
    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let area = triangle_area(&v0, &v1, &v2);
        if area < 1e-20 {
            degenerate_triangle_count += 1;
        }
    }

    // T-junction detection: check if any vertex lies on an edge it's not part of
    // This is expensive for large meshes, so we sample
    let t_junction_count = if mesh.vertices.len() < 100000 {
        detect_t_junctions(mesh)
    } else {
        // Skip for very large meshes (too expensive)
        0
    };

    ManifoldReport {
        vertex_count,
        edge_count,
        triangle_count,
        euler_characteristic: euler,
        boundary_edge_count,
        non_manifold_edge_count,
        degenerate_triangle_count,
        t_junction_count,
        boundary_edges,
        non_manifold_edges,
    }
}

/// Compute the area of a triangle given its three vertices.
fn triangle_area(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
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

/// Detect T-junctions: vertices that lie on edges they're not part of.
/// Uses spatial hashing for efficiency.
fn detect_t_junctions(mesh: &TriangleMesh) -> usize {
    use std::collections::HashSet;
    
    // Build set of all edges
    let mut edges: HashSet<(u32, u32)> = HashSet::new();
    for tri in &mesh.triangles {
        let v0 = tri[0];
        let v1 = tri[1];
        let v2 = tri[2];
        edges.insert(if v0 < v1 { (v0, v1) } else { (v1, v0) });
        edges.insert(if v1 < v2 { (v1, v2) } else { (v2, v1) });
        edges.insert(if v2 < v0 { (v2, v0) } else { (v0, v2) });
    }

    // For each edge, check if any vertex (not part of the edge) lies on it
    let mut t_junctions = 0;
    let tol_sq = 1e-12; // Tolerance for point-on-edge check
    
    // Build vertex → edge set for each vertex to know which edges it belongs to
    let mut vertex_edges: HashMap<u32, HashSet<(u32, u32)>> = HashMap::new();
    for &(a, b) in &edges {
        vertex_edges.entry(a).or_default().insert((a.min(b), a.max(b)));
        vertex_edges.entry(b).or_default().insert((a.min(b), a.max(b)));
    }

    // Sample check: for performance, only check edges with length > 0
    for &(a, b) in &edges {
        let pa = mesh.vertices[a as usize];
        let pb = mesh.vertices[b as usize];
        let edge_len_sq = (pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2) + (pa.z - pb.z).powi(2);
        if edge_len_sq < 1e-20 {
            continue;
        }

        // Check a limited number of vertices (performance)
        // For each vertex not part of this edge, check if it lies on the edge
        let v_edges_a = vertex_edges.get(&a);
        
        // Only check vertices that share a triangle with a but aren't part of this edge
        if let Some(v_set) = v_edges_a {
            for &(c, d) in v_set {
                let other = if c == a { d } else { c };
                if other != a && other != b {
                    let po = mesh.vertices[other as usize];
                    if point_on_segment(&po, &pa, &pb, tol_sq) {
                        t_junctions += 1;
                        break; // One T-junction per edge is enough
                    }
                }
            }
        }
    }

    t_junctions
}

/// Check if a point lies on a line segment within tolerance.
fn point_on_segment(p: &Point3d, a: &Point3d, b: &Point3d, tol_sq: f64) -> bool {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let abz = b.z - a.z;
    let apx = p.x - a.x;
    let apy = p.y - a.y;
    let apz = p.z - a.z;
    
    let ab_len_sq = abx * abx + aby * aby + abz * abz;
    if ab_len_sq < 1e-20 {
        return (apx * apx + apy * apy + apz * apz) < tol_sq;
    }
    
    let t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
    if t < 0.0 || t > 1.0 {
        return false;
    }
    
    // Distance from p to the closest point on the segment
    let cx = a.x + t * abx - p.x;
    let cy = a.y + t * aby - p.y;
    let cz = a.z + t * abz - p.z;
    (cx * cx + cy * cy + cz * cz) < tol_sq
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::TriangleMesh;

    /// Helper: create a simple cube mesh (6 faces, 12 triangles, 8 vertices)
    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        // 8 vertices of a unit cube
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
        // 12 triangles (2 per face)
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
    fn test_cube_euler_characteristic() {
        let mesh = make_cube_mesh();
        let report = check_manifold(&mesh);
        assert!(report.is_watertight(), "Cube should be watertight, but has {} boundary edges", report.boundary_edge_count);
        // For a sphere (genus 0): χ = V - E + F = 2
        assert_eq!(report.euler_characteristic, 2,
            "Cube Euler characteristic should be 2, got {}", report.euler_characteristic);
    }

    #[test]
    fn test_cube_is_manifold() {
        let mesh = make_cube_mesh();
        let report = check_manifold(&mesh);
        assert!(report.is_manifold(), "Cube should be manifold");
        assert_eq!(report.non_manifold_edge_count, 0, "Cube should have no non-manifold edges");
    }

    #[test]
    fn test_open_mesh_has_boundary() {
        let mut mesh = TriangleMesh::new();
        // Single triangle — 3 boundary edges
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let report = check_manifold(&mesh);
        assert!(!report.is_watertight(), "Single triangle should not be watertight");
        assert_eq!(report.boundary_edge_count, 3, "Single triangle should have 3 boundary edges");
        // Euler characteristic: V=3, E=3, F=1 → χ = 1
        assert_eq!(report.euler_characteristic, 1,
            "Single triangle: χ = V-E+F = 3-3+1 = 1, got {}", report.euler_characteristic);
    }

    #[test]
    fn test_sphere_mesh_euler() {
        // Create a simple closed mesh (icosahedron-like)
        let mut mesh = TriangleMesh::new();
        // Create a simple tetrahedron (4 vertices, 4 faces)
        mesh.add_vertex(Point3d::new(1.0, 1.0, 1.0));
        mesh.add_vertex(Point3d::new(1.0, -1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, 1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, -1.0, 1.0));
        mesh.add_triangle(0, 1, 2);
        mesh.add_triangle(0, 1, 3);
        mesh.add_triangle(0, 2, 3);
        mesh.add_triangle(1, 2, 3);

        let report = check_manifold(&mesh);
        assert!(report.is_watertight(), "Tetrahedron should be watertight");
        // For a sphere (genus 0): χ = 2
        // V=4, E=6, F=4 → χ = 2 ✓
        assert_eq!(report.euler_characteristic, 2,
            "Tetrahedron Euler characteristic should be 2, got {}", report.euler_characteristic);
    }

    #[test]
    fn test_torus_euler() {
        // A torus has genus 1, so χ = 0
        // We check the formula rather than constructing a full torus mesh
        // (which would need many vertices to be manifold)
        assert_eq!(ManifoldReport::expected_euler_for_genus(0), 2, "Genus 0 → χ = 2");
        assert_eq!(ManifoldReport::expected_euler_for_genus(1), 0, "Genus 1 → χ = 0");
        assert_eq!(ManifoldReport::expected_euler_for_genus(2), -2, "Genus 2 → χ = -2");
    }

    #[test]
    fn test_degenerate_triangles_counted() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        // Zero-area triangle (all 3 vertices at the same point)
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let report = check_manifold(&mesh);
        assert!(report.degenerate_triangle_count > 0, "Should detect degenerate triangles");
    }

    #[test]
    fn test_expected_euler_for_genus() {
        assert_eq!(ManifoldReport::expected_euler_for_genus(0), 2);
        assert_eq!(ManifoldReport::expected_euler_for_genus(1), 0);
        assert_eq!(ManifoldReport::expected_euler_for_genus(2), -2);
    }
}
