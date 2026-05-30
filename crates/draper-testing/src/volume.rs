// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.11 — Volume Test
//!
//! Compare mesh volume with analytical values using divergence theorem.

use draper_mesh::TriangleMesh;
use std::f64::consts::PI;

/// Compute the volume of a closed triangle mesh using the divergence theorem.
///
/// For a closed surface mesh, the signed volume can be computed as:
/// V = (1/6) Σ (v0 · (v1 × v2))
///
/// where the sum is over all triangles with vertices (v0, v1, v2).
/// This is equivalent to integrating the divergence of F(x,y,z) = (x,y,z)/3
/// over the volume enclosed by the mesh.
pub fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let mut volume = 0.0;
    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        // Signed volume contribution from triangle
        // V += (1/6) * v0 · (v1 × v2)
        let cross_x = v1.y * v2.z - v1.z * v2.y;
        let cross_y = v1.z * v2.x - v1.x * v2.z;
        let cross_z = v1.x * v2.y - v1.y * v2.x;
        volume += v0.x * cross_x + v0.y * cross_y + v0.z * cross_z;
    }
    volume / 6.0
}

/// Analytical volume of a sphere: (4/3)πr³.
pub fn analytical_sphere_volume(radius: f64) -> f64 {
    (4.0 / 3.0) * PI * radius * radius * radius
}

/// Analytical volume of a cube: s³.
pub fn analytical_cube_volume(side: f64) -> f64 {
    side * side * side
}

/// Analytical volume of a cylinder: πr²h.
pub fn analytical_cylinder_volume(radius: f64, height: f64) -> f64 {
    PI * radius * radius * height
}

/// Compute percentage deviation between mesh volume and analytical volume.
/// Returns |mesh - analytical| / |analytical| * 100.
pub fn volume_deviation(mesh_vol: f64, analytical_vol: f64) -> f64 {
    if analytical_vol.abs() < 1e-15 {
        return 0.0;
    }
    (mesh_vol - analytical_vol).abs() / analytical_vol.abs() * 100.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    #[test]
    fn test_analytical_sphere_volume() {
        let vol = analytical_sphere_volume(1.0);
        assert!((vol - (4.0 / 3.0) * PI).abs() < 1e-10);
    }

    #[test]
    fn test_analytical_cube_volume() {
        let vol = analytical_cube_volume(1.0);
        assert!((vol - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_analytical_cylinder_volume() {
        let vol = analytical_cylinder_volume(1.0, 2.0);
        assert!((vol - 2.0 * PI).abs() < 1e-10);
    }

    #[test]
    fn test_volume_deviation_zero() {
        let dev = volume_deviation(1.0, 1.0);
        assert!(dev.abs() < 1e-10);
    }

    #[test]
    fn test_cube_mesh_volume() {
        let mut mesh = TriangleMesh::new();
        // Unit cube from (0,0,0) to (1,1,1)
        // Triangles must be oriented outward for correct signed volume
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
        // Bottom (z=0) — normal pointing down (-Z)
        mesh.add_triangle(0, 1, 2);
        mesh.add_triangle(0, 2, 3);
        // Top (z=1) — normal pointing up (+Z)
        mesh.add_triangle(4, 6, 5);
        mesh.add_triangle(4, 7, 6);
        // Front (y=0) — normal pointing -Y
        mesh.add_triangle(0, 5, 1);
        mesh.add_triangle(0, 4, 5);
        // Back (y=1) — normal pointing +Y
        mesh.add_triangle(3, 2, 6);
        mesh.add_triangle(3, 6, 7);
        // Left (x=0) — normal pointing -X
        mesh.add_triangle(0, 3, 7);
        mesh.add_triangle(0, 7, 4);
        // Right (x=1) — normal pointing +X
        mesh.add_triangle(1, 5, 6);
        mesh.add_triangle(1, 6, 2);

        let vol = mesh_volume(&mesh);
        let analytical = analytical_cube_volume(1.0);
        let dev = volume_deviation(vol, analytical);
        assert!(dev < 5.0, "Cube mesh volume should be within 5% of analytical, got {}% deviation (mesh={}, analytical={})",
            dev, vol, analytical);
    }

    #[test]
    fn test_tetrahedron_volume() {
        let mut mesh = TriangleMesh::new();
        // Regular tetrahedron with vertices at:
        // (1,1,1), (1,-1,-1), (-1,1,-1), (-1,-1,1)
        // Volume = 8/3
        mesh.add_vertex(Point3d::new(1.0, 1.0, 1.0));
        mesh.add_vertex(Point3d::new(1.0, -1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, 1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, -1.0, 1.0));
        // Outward-oriented faces
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        mesh.add_triangle(0, 1, 3);
        mesh.add_triangle(1, 2, 3);

        let vol = mesh_volume(&mesh).abs();
        let expected = 8.0 / 3.0;
        let dev = volume_deviation(vol, expected);
        assert!(dev < 1.0, "Tetrahedron volume should match analytical, got {}% deviation", dev);
    }
}
