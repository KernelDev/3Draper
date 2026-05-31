// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.9 — Euler Characteristic Test
//!
//! Compute and verify Euler characteristic: χ = V - E + F = 2(1 - genus)

use draper_mesh::TriangleMesh;
use std::collections::HashSet;

/// Compute the Euler characteristic of a mesh: χ = V - E + F.
pub fn compute_euler(mesh: &TriangleMesh) -> i64 {
    let v = mesh.vertices.len() as i64;
    let f = mesh.triangles.len() as i64;

    // Count unique edges
    let mut edges: HashSet<(u32, u32)> = HashSet::new();
    for tri in &mesh.triangles {
        let v0 = tri[0];
        let v1 = tri[1];
        let v2 = tri[2];
        edges.insert(if v0 < v1 { (v0, v1) } else { (v1, v0) });
        edges.insert(if v1 < v2 { (v1, v2) } else { (v2, v1) });
        edges.insert(if v2 < v0 { (v2, v0) } else { (v0, v2) });
    }
    let e = edges.len() as i64;

    v - e + f
}

/// Expected Euler characteristic for a closed surface of the given genus.
/// χ = 2(1 - genus)
pub fn expected_euler(genus: u32) -> i64 {
    2 * (1 - genus as i64)
}

/// Test if a mesh's Euler characteristic matches the expected genus.
pub fn test_euler(mesh: &TriangleMesh, expected_genus: u32) -> bool {
    let actual = compute_euler(mesh);
    let expected = expected_euler(expected_genus);
    actual == expected
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
            Point3d::new(0.0, 0.0, 1.0),
            Point3d::new(1.0, 0.0, 1.0),
            Point3d::new(1.0, 1.0, 1.0),
            Point3d::new(0.0, 1.0, 1.0),
        ];
        for p in &v {
            mesh.add_vertex(*p);
        }
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        mesh.add_triangle(3, 7, 6);
        mesh.add_triangle(3, 6, 2);
        mesh.add_triangle(0, 4, 7);
        mesh.add_triangle(0, 7, 3);
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);
        mesh
    }

    #[test]
    fn test_cube_euler() {
        let mesh = make_cube_mesh();
        let euler = compute_euler(&mesh);
        assert_eq!(euler, 2, "Cube (genus 0) should have χ = 2");
    }

    #[test]
    fn test_cube_genus_0() {
        let mesh = make_cube_mesh();
        assert!(test_euler(&mesh, 0), "Cube should match genus 0");
    }

    #[test]
    fn test_expected_euler_values() {
        assert_eq!(expected_euler(0), 2, "Genus 0 → χ = 2");
        assert_eq!(expected_euler(1), 0, "Genus 1 (torus) → χ = 0");
        assert_eq!(expected_euler(2), -2, "Genus 2 (double torus) → χ = -2");
        assert_eq!(expected_euler(3), -4, "Genus 3 → χ = -4");
    }

    #[test]
    fn test_tetrahedron_euler() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(1.0, 1.0, 1.0));
        mesh.add_vertex(Point3d::new(1.0, -1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, 1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, -1.0, 1.0));
        mesh.add_triangle(0, 1, 2);
        mesh.add_triangle(0, 1, 3);
        mesh.add_triangle(0, 2, 3);
        mesh.add_triangle(1, 2, 3);

        assert_eq!(compute_euler(&mesh), 2, "Tetrahedron should have χ = 2");
        assert!(test_euler(&mesh, 0), "Tetrahedron should match genus 0");
    }
}
