// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.10 — Surface Area Test
//!
//! Compare mesh surface area with analytical values.

use draper_mesh::TriangleMesh;
use std::f64::consts::PI;

/// Compute the total surface area of a triangle mesh.
/// Sum of all triangle areas.
pub fn mesh_surface_area(mesh: &TriangleMesh) -> f64 {
    mesh.surface_area()
}

/// Analytical surface area of a sphere: 4πr².
pub fn analytical_sphere_area(radius: f64) -> f64 {
    4.0 * PI * radius * radius
}

/// Analytical surface area of a cylinder: 2πrh + 2πr²
/// (lateral area + top and bottom caps).
pub fn analytical_cylinder_area(radius: f64, height: f64) -> f64 {
    2.0 * PI * radius * height + 2.0 * PI * radius * radius
}

/// Analytical surface area of a cube: 6s².
pub fn analytical_cube_area(side: f64) -> f64 {
    6.0 * side * side
}

/// Analytical surface area of a cone: πr(r + √(r² + h²))
/// (base + lateral).
pub fn analytical_cone_area(radius: f64, height: f64) -> f64 {
    let slant = (radius * radius + height * height).sqrt();
    PI * radius * (radius + slant)
}

/// Compute percentage deviation between mesh area and analytical area.
/// Returns |mesh - analytical| / analytical * 100.
pub fn area_deviation(mesh_area: f64, analytical_area: f64) -> f64 {
    if analytical_area.abs() < 1e-15 {
        return 0.0;
    }
    (mesh_area - analytical_area).abs() / analytical_area * 100.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analytical_sphere_area() {
        let area = analytical_sphere_area(1.0);
        assert!((area - 4.0 * PI).abs() < 1e-10, "Sphere r=1 area should be 4π");
    }

    #[test]
    fn test_analytical_cylinder_area() {
        let area = analytical_cylinder_area(1.0, 2.0);
        let expected = 2.0 * PI * 1.0 * 2.0 + 2.0 * PI * 1.0;
        assert!((area - expected).abs() < 1e-10, "Cylinder r=1,h=2 area");
    }

    #[test]
    fn test_analytical_cube_area() {
        let area = analytical_cube_area(1.0);
        assert!((area - 6.0).abs() < 1e-10, "Cube s=1 area should be 6");
    }

    #[test]
    fn test_area_deviation_zero() {
        let dev = area_deviation(6.0, 6.0);
        assert!(dev.abs() < 1e-10, "Zero deviation when areas match");
    }

    #[test]
    fn test_area_deviation_5_percent() {
        let dev = area_deviation(105.0, 100.0);
        assert!((dev - 5.0).abs() < 1e-10, "5% deviation");
    }

    #[test]
    fn test_cube_mesh_area() {
        use draper_mesh::TriangleMesh;
        use draper_geometry::Point3d;

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

        let mesh_area = mesh_surface_area(&mesh);
        let analytical = analytical_cube_area(1.0);
        let dev = area_deviation(mesh_area, analytical);
        assert!(dev < 1.0, "Cube mesh area should be within 1% of analytical, got {}% deviation", dev);
    }
}
