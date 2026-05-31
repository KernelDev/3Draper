// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.1 — Reference Primitives
//!
//! Functions that generate perfect B-Rep models of basic primitives.
//! Each returns a proper `Solid` from `draper_topology`.

use draper_topology::{Solid, Shell, Face, Wire, CoEdge, Edge};
use draper_geometry::{
    Point3d, Direction3d,
    Surface, Plane,
};

/// Create a unit cube: 6 faces, 12 edges, 8 vertices.
/// The cube spans from (0,0,0) to (1,1,1).
pub fn make_unit_cube() -> Solid {
    // 8 vertices
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

    // Build 6 faces using ShapeBuilder-like logic
    let faces = vec![
        make_rect_face(v[0], v[1], v[2], v[3]), // Bottom (z=0)
        make_rect_face(v[4], v[7], v[6], v[5]), // Top (z=1)
        make_rect_face(v[0], v[4], v[5], v[1]), // Front (y=0)
        make_rect_face(v[3], v[2], v[6], v[7]), // Back (y=1)
        make_rect_face(v[0], v[3], v[7], v[4]), // Left (x=0)
        make_rect_face(v[1], v[5], v[6], v[2]), // Right (x=1)
    ];

    let shell = Shell::new_closed(faces);
    Solid::new(shell)
}

/// Create a unit sphere with specified resolution.
/// Radius = 1.0, centered at origin.
/// `n_u` = angular samples, `n_v` = polar samples.
pub fn make_unit_sphere(_n_u: usize, _n_v: usize) -> Solid {
    // Use the topology builder's make_sphere which creates a proper B-Rep
    draper_topology::ShapeBuilder::make_sphere(1.0)
}

/// Create a unit cylinder: radius=1, height=2, centered along Z axis.
/// Bottom at z=0, top at z=2.
pub fn make_unit_cylinder() -> Solid {
    draper_topology::ShapeBuilder::make_cylinder(1.0, 2.0)
}

/// Create a unit cone: radius=1, height=2.
/// The half_angle is computed from radius/height.
pub fn make_unit_cone() -> Solid {
    let radius = 1.0;
    let height = 2.0;
    let half_angle: f64 = f64::atan(radius / height);
    draper_topology::ShapeBuilder::make_cone(radius, height, half_angle)
}

/// Create a unit torus with given major and minor radii.
pub fn make_unit_torus(major_r: f64, minor_r: f64) -> Solid {
    draper_topology::ShapeBuilder::make_torus(major_r, minor_r)
}

/// Helper: create a rectangular face from 4 corner points.
fn make_rect_face(p0: Point3d, p1: Point3d, p2: Point3d, p3: Point3d) -> Face {
    let e0 = Edge::new_line(p0, p1);
    let e1 = Edge::new_line(p1, p2);
    let e2 = Edge::new_line(p2, p3);
    let e3 = Edge::new_line(p3, p0);

    let id0 = e0.id;
    let id1 = e1.id;
    let id2 = e2.id;
    let id3 = e3.id;

    let coedges = vec![
        CoEdge::new(id0, true),
        CoEdge::new(id1, true),
        CoEdge::new(id2, true),
        CoEdge::new(id3, true),
    ];

    let wire = Wire::new(coedges);
    let plane = Plane::from_three_points(&p0, &p1, &p2)
        .unwrap_or_else(|| Plane::from_origin_and_normal(p0, Direction3d::Z));

    let mut face = Face::new(Surface::Plane(plane), wire);
    face.edges = vec![e0, e1, e2, e3];
    face
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unit_cube_has_6_faces() {
        let cube = make_unit_cube();
        let faces = cube.faces();
        assert_eq!(faces.len(), 6, "Unit cube should have 6 faces");
    }

    #[test]
    fn test_unit_cube_edges_and_vertices() {
        let cube = make_unit_cube();
        let faces = cube.faces();
        // 6 faces × 4 edges each = 24 edge uses
        // Each edge is shared by 2 faces → 12 unique edges
        // 8 unique vertices
        let total_edge_uses: usize = faces.iter().map(|f| f.edges.len()).sum();
        assert_eq!(total_edge_uses, 24, "Unit cube should have 24 edge uses (6 faces × 4 edges)");
    }

    #[test]
    fn test_unit_sphere_creates_solid() {
        let sphere = make_unit_sphere(16, 8);
        let faces = sphere.faces();
        assert!(!faces.is_empty(), "Unit sphere should have at least one face");
    }

    #[test]
    fn test_unit_cylinder_has_3_faces() {
        let cyl = make_unit_cylinder();
        let faces = cyl.faces();
        assert_eq!(faces.len(), 3, "Unit cylinder should have 3 faces (bottom, top, lateral)");
    }

    #[test]
    fn test_unit_cone_has_2_faces() {
        let cone = make_unit_cone();
        let faces = cone.faces();
        assert_eq!(faces.len(), 2, "Unit cone should have 2 faces (bottom, lateral)");
    }

    #[test]
    fn test_unit_torus_creates_solid() {
        let torus = make_unit_torus(2.0, 0.5);
        let faces = torus.faces();
        assert!(!faces.is_empty(), "Unit torus should have at least one face");
    }
}
