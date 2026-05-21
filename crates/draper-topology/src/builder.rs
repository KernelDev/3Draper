//! Shape builder utilities for common operations.

use crate::entity::*;
use crate::shape::Shape;
use draper_geometry::direction::Axis2Placement3D;
use draper_geometry::point::Point3;
use draper_geometry::surface::{CylindricalSurface, Plane, Surface};

/// Builder for creating common shapes.
pub struct ShapeBuilder;

impl ShapeBuilder {
    /// Create a rectangular box (brick).
    pub fn make_box(shape: &mut Shape, dx: f64, dy: f64, dz: f64) -> TopoId {
        // Create 8 vertices
        let v = [
            shape.add_vertex(Point3::new(0.0, 0.0, 0.0)),
            shape.add_vertex(Point3::new(dx, 0.0, 0.0)),
            shape.add_vertex(Point3::new(dx, dy, 0.0)),
            shape.add_vertex(Point3::new(0.0, dy, 0.0)),
            shape.add_vertex(Point3::new(0.0, 0.0, dz)),
            shape.add_vertex(Point3::new(dx, 0.0, dz)),
            shape.add_vertex(Point3::new(dx, dy, dz)),
            shape.add_vertex(Point3::new(0.0, dy, dz)),
        ];

        // Create 12 edges
        let edges = [
            // Bottom face
            shape.add_edge(None, v[0], v[1], None),
            shape.add_edge(None, v[1], v[2], None),
            shape.add_edge(None, v[2], v[3], None),
            shape.add_edge(None, v[3], v[0], None),
            // Top face
            shape.add_edge(None, v[4], v[5], None),
            shape.add_edge(None, v[5], v[6], None),
            shape.add_edge(None, v[6], v[7], None),
            shape.add_edge(None, v[7], v[4], None),
            // Vertical edges
            shape.add_edge(None, v[0], v[4], None),
            shape.add_edge(None, v[1], v[5], None),
            shape.add_edge(None, v[2], v[6], None),
            shape.add_edge(None, v[3], v[7], None),
        ];

        // Create 6 wires (one per face)
        let wires = [
            shape.add_wire(vec![
                OrientedEdge { edge_id: edges[0], orientation: true },
                OrientedEdge { edge_id: edges[1], orientation: true },
                OrientedEdge { edge_id: edges[2], orientation: true },
                OrientedEdge { edge_id: edges[3], orientation: true },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: edges[4], orientation: true },
                OrientedEdge { edge_id: edges[5], orientation: true },
                OrientedEdge { edge_id: edges[6], orientation: true },
                OrientedEdge { edge_id: edges[7], orientation: true },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: edges[0], orientation: true },
                OrientedEdge { edge_id: edges[9], orientation: true },
                OrientedEdge { edge_id: edges[4], orientation: false },
                OrientedEdge { edge_id: edges[8], orientation: false },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: edges[1], orientation: true },
                OrientedEdge { edge_id: edges[10], orientation: true },
                OrientedEdge { edge_id: edges[5], orientation: false },
                OrientedEdge { edge_id: edges[9], orientation: false },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: edges[2], orientation: true },
                OrientedEdge { edge_id: edges[11], orientation: true },
                OrientedEdge { edge_id: edges[6], orientation: false },
                OrientedEdge { edge_id: edges[10], orientation: false },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: edges[3], orientation: true },
                OrientedEdge { edge_id: edges[8], orientation: true },
                OrientedEdge { edge_id: edges[7], orientation: false },
                OrientedEdge { edge_id: edges[11], orientation: false },
            ]),
        ];

        // Create 6 faces
        let faces = [
            shape.add_face(Some(Surface::Plane(Plane::new(Axis2Placement3D::new(
                Point3::new(0.0, 0.0, 0.0),
                draper_geometry::direction::Direction3::NEG_Z,
                None,
            ))))),
            shape.add_face(Some(Surface::Plane(Plane::new(Axis2Placement3D::new(
                Point3::new(0.0, 0.0, dz),
                draper_geometry::direction::Direction3::Z,
                None,
            ))))),
            shape.add_face(Some(Surface::Plane(Plane::new(Axis2Placement3D::new(
                Point3::new(0.0, 0.0, 0.0),
                draper_geometry::direction::Direction3::NEG_Y,
                None,
            ))))),
            shape.add_face(Some(Surface::Plane(Plane::new(Axis2Placement3D::new(
                Point3::new(dx, 0.0, 0.0),
                draper_geometry::direction::Direction3::Y,
                None,
            ))))),
            shape.add_face(Some(Surface::Plane(Plane::new(Axis2Placement3D::new(
                Point3::new(dx, dy, 0.0),
                draper_geometry::direction::Direction3::X,
                None,
            ))))),
            shape.add_face(Some(Surface::Plane(Plane::new(Axis2Placement3D::new(
                Point3::new(0.0, dy, 0.0),
                draper_geometry::direction::Direction3::NEG_X,
                None,
            ))))),
        ];

        for i in 0..6 {
            shape.set_face_outer_wire(faces[i], wires[i]);
        }

        // Create shell
        let shell = shape.add_shell(faces.to_vec());

        // Create solid
        shape.add_solid(shell)
    }

    /// Create a cylinder along the Z axis.
    pub fn make_cylinder(shape: &mut Shape, radius: f64, height: f64) -> TopoId {
        // Simplified cylinder: create the B-rep structure
        let center_bottom = Point3::new(0.0, 0.0, 0.0);
        let center_top = Point3::new(0.0, 0.0, height);

        // Vertices
        let _v_center_b = shape.add_vertex(center_bottom);
        let _v_center_t = shape.add_vertex(center_top);

        // We need at least 4 points on each circle to approximate
        let segments = 16;
        let mut bottom_verts = Vec::new();
        let mut top_verts = Vec::new();

        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            bottom_verts.push(shape.add_vertex(Point3::new(x, y, 0.0)));
            top_verts.push(shape.add_vertex(Point3::new(x, y, height)));
        }

        // Create edges
        let mut bottom_edges = Vec::new();
        let mut top_edges = Vec::new();
        let mut vertical_edges = Vec::new();

        for i in 0..segments {
            let next = (i + 1) % segments;
            bottom_edges.push(shape.add_edge(None, bottom_verts[i], bottom_verts[next], None));
            top_edges.push(shape.add_edge(None, top_verts[i], top_verts[next], None));
            vertical_edges.push(shape.add_edge(None, bottom_verts[i], top_verts[i], None));
        }

        // Create bottom wire
        let bottom_wire = shape.add_wire(
            bottom_edges.iter().map(|&e| OrientedEdge { edge_id: e, orientation: true }).collect()
        );

        // Create top wire
        let top_wire = shape.add_wire(
            top_edges.iter().map(|&e| OrientedEdge { edge_id: e, orientation: true }).collect()
        );

        // Create side wires (one per segment)
        let mut side_wires = Vec::new();
        for i in 0..segments {
            let next = (i + 1) % segments;
            let wire = shape.add_wire(vec![
                OrientedEdge { edge_id: vertical_edges[i], orientation: true },
                OrientedEdge { edge_id: top_edges[i], orientation: true },
                OrientedEdge { edge_id: vertical_edges[next], orientation: false },
                OrientedEdge { edge_id: bottom_edges[i], orientation: false },
            ]);
            side_wires.push(wire);
        }

        // Create faces
        let bottom_face = shape.add_face(Some(Surface::Plane(Plane::new(
            Axis2Placement3D::new(center_bottom, draper_geometry::direction::Direction3::NEG_Z, None),
        ))));
        shape.set_face_outer_wire(bottom_face, bottom_wire);

        let top_face = shape.add_face(Some(Surface::Plane(Plane::new(
            Axis2Placement3D::new(center_top, draper_geometry::direction::Direction3::Z, None),
        ))));
        shape.set_face_outer_wire(top_face, top_wire);

        let lateral_face = shape.add_face(Some(Surface::CylindricalSurface(CylindricalSurface::new(
            Axis2Placement3D::new(center_bottom, draper_geometry::direction::Direction3::Z, None),
            radius,
        ))));

        let mut all_faces = vec![bottom_face, top_face, lateral_face];
        for wire in side_wires {
            let face = shape.add_face(None);
            shape.set_face_outer_wire(face, wire);
            all_faces.push(face);
        }

        // Create shell and solid
        let shell = shape.add_shell(all_faces);
        shape.add_solid(shell)
    }
}
