// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.2 — Reference Combinations
//!
//! Functions that generate B-Rep models of combined primitives.

use draper_topology::Solid;
use crate::primitives;

/// Create a cylinder sitting on the XY plane.
/// Radius=1, height=2, bottom at z=0, top at z=2.
pub fn make_cylinder_on_plane() -> Solid {
    primitives::make_unit_cylinder()
}

/// Create a cone sitting on the XY plane.
/// Radius=1, height=2, base at z=0, apex at z=2.
pub fn make_cone_on_plane() -> Solid {
    primitives::make_unit_cone()
}

/// Create a rectangular block with a cylindrical through-hole.
/// Block: 4×4×2, centered at origin.
/// Hole: cylinder of radius 1 along Z axis through the full height.
///
/// This is constructed as a simplified B-Rep with a block face
/// that has an inner wire for the hole.
pub fn make_block_with_hole() -> Solid {
    use draper_topology::{Shell, Face, Wire, CoEdge, Edge, TopoId};
    use draper_geometry::{
        Point3d, Direction3d, Curve3d, Surface,
        Plane, CylinderSurface, Circle,
    };
    use std::f64::consts::PI;

    // Block dimensions
    let hx = 2.0; // half-x = 2 (full width = 4)
    let hy = 2.0; // half-y = 2 (full depth = 4)
    let hz = 1.0; // half-z = 1 (full height = 2)
    let hole_radius = 1.0;

    // 8 corners of the block
    let v = [
        Point3d::new(-hx, -hy, -hz), // 0
        Point3d::new( hx, -hy, -hz), // 1
        Point3d::new( hx,  hy, -hz), // 2
        Point3d::new(-hx,  hy, -hz), // 3
        Point3d::new(-hx, -hy,  hz), // 4
        Point3d::new( hx, -hy,  hz), // 5
        Point3d::new( hx,  hy,  hz), // 6
        Point3d::new(-hx,  hy,  hz), // 7
    ];

    // Helper to make a rectangular face
    fn make_rect(p0: Point3d, p1: Point3d, p2: Point3d, p3: Point3d) -> Face {
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

    // Top face with hole
    let top_outer_edges = vec![
        Edge::new_line(v[4], v[5]),
        Edge::new_line(v[5], v[6]),
        Edge::new_line(v[6], v[7]),
        Edge::new_line(v[7], v[4]),
    ];
    let top_outer_coedges: Vec<CoEdge> = top_outer_edges.iter().map(|e| CoEdge::new(e.id, true)).collect();
    let top_outer_wire = Wire::new(top_outer_coedges);

    // Inner hole circle on top face
    let top_hole_circle = Circle::new_xy(Point3d::new(0.0, 0.0, hz), hole_radius);
    let top_hole_edge = Edge {
        id: TopoId::new(),
        curve: Some(Curve3d::Circle(top_hole_circle)),
        param_range: (0.0, 2.0 * PI),
        vertex_start: None,
        vertex_end: None,
        forward: true,
        tolerance: 1e-6,
        degenerate: false,
    };
    let top_hole_coedge = CoEdge::new(top_hole_edge.id, false); // Reversed for inner wire
    let top_hole_wire = Wire::new(vec![top_hole_coedge]);

    let mut top_face = Face::new(
        Surface::Plane(Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, hz),
            Direction3d::Z,
        )),
        top_outer_wire,
    );
    top_face.add_hole(top_hole_wire);
    top_face.edges = {
        let mut e = top_outer_edges;
        e.push(top_hole_edge);
        e
    };

    // Bottom face with hole (same structure, reversed normal)
    let bottom_outer_edges = vec![
        Edge::new_line(v[0], v[3]),
        Edge::new_line(v[3], v[2]),
        Edge::new_line(v[2], v[1]),
        Edge::new_line(v[1], v[0]),
    ];
    let bottom_outer_coedges: Vec<CoEdge> = bottom_outer_edges.iter().map(|e| CoEdge::new(e.id, true)).collect();
    let bottom_outer_wire = Wire::new(bottom_outer_coedges);

    let bottom_hole_circle = Circle::new_xy(Point3d::new(0.0, 0.0, -hz), hole_radius);
    let bottom_hole_edge = Edge {
        id: TopoId::new(),
        curve: Some(Curve3d::Circle(bottom_hole_circle)),
        param_range: (0.0, 2.0 * PI),
        vertex_start: None,
        vertex_end: None,
        forward: true,
        tolerance: 1e-6,
        degenerate: false,
    };
    let bottom_hole_coedge = CoEdge::new(bottom_hole_edge.id, false);
    let bottom_hole_wire = Wire::new(vec![bottom_hole_coedge]);

    let mut bottom_face = Face::new(
        Surface::Plane(Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, -hz),
            Direction3d::new(0.0, 0.0, -1.0).unwrap_or(Direction3d::Z),
        )),
        bottom_outer_wire,
    );
    bottom_face.add_hole(bottom_hole_wire);
    bottom_face.edges = {
        let mut e = bottom_outer_edges;
        e.push(bottom_hole_edge);
        e
    };

    // 4 side faces
    let side_faces = vec![
        make_rect(v[0], v[1], v[5], v[4]), // Front
        make_rect(v[3], v[7], v[6], v[2]), // Back
        make_rect(v[0], v[4], v[7], v[3]), // Left
        make_rect(v[1], v[2], v[6], v[5]), // Right
    ];

    // Inner cylinder surface (hole wall)
    let cyl_surface = CylinderSurface::new_z(hole_radius);
    let cyl_edge_bottom = Edge {
        id: TopoId::new(),
        curve: Some(Curve3d::Circle(Circle::new_xy(Point3d::new(0.0, 0.0, -hz), hole_radius))),
        param_range: (0.0, 2.0 * PI),
        vertex_start: None,
        vertex_end: None,
        forward: true,
        tolerance: 1e-6,
        degenerate: false,
    };
    let cyl_edge_top = Edge {
        id: TopoId::new(),
        curve: Some(Curve3d::Circle(Circle::new_xy(Point3d::new(0.0, 0.0, hz), hole_radius))),
        param_range: (0.0, 2.0 * PI),
        vertex_start: None,
        vertex_end: None,
        forward: true,
        tolerance: 1e-6,
        degenerate: false,
    };

    let cyl_coedge_bottom = CoEdge::new(cyl_edge_bottom.id, true);
    let cyl_wire = Wire::new(vec![cyl_coedge_bottom]);
    let mut cyl_face = Face::new(Surface::Cylinder(cyl_surface), cyl_wire);
    cyl_face.edges = vec![cyl_edge_bottom, cyl_edge_top];

    let mut all_faces: Vec<Face> = vec![bottom_face, top_face, cyl_face];
    all_faces.extend(side_faces);

    let shell = Shell::new_closed(all_faces);
    Solid::new(shell)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cylinder_on_plane_creates_solid() {
        let solid = make_cylinder_on_plane();
        assert!(!solid.faces().is_empty());
    }

    #[test]
    fn test_cone_on_plane_creates_solid() {
        let solid = make_cone_on_plane();
        assert!(!solid.faces().is_empty());
    }

    #[test]
    fn test_block_with_hole_creates_solid() {
        let solid = make_block_with_hole();
        let faces = solid.faces();
        // 6 block faces + 1 cylinder face = 7
        assert!(faces.len() >= 7, "Block with hole should have at least 7 faces, got {}", faces.len());
    }
}
