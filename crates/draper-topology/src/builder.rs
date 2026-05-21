//! Shape builder utilities for common operations.
//!
//! Provides parametric creation of primitive solids:
//! - Box, Cylinder, Cone, Sphere, Torus
//! - Cylinder with arbitrary axis placement
//! - Truncated cone
//!
//! All builders create complete B-Rep topology: vertices, edges (with 3D curves),
//! wires, faces (with surface geometry), shells, and solids.

use crate::entity::*;
use crate::shape::Shape;
use draper_geometry::curve::{Circle, Curve, Line};
use draper_geometry::direction::Axis2Placement3D;
use draper_geometry::point::Point3;
use draper_geometry::surface::{
    ConicalSurface, CylindricalSurface, Plane, SphericalSurface, Surface, ToroidalSurface,
};

/// Builder for creating common shapes.
pub struct ShapeBuilder;

impl ShapeBuilder {
    /// Create a rectangular box (brick) aligned with axes, corner at origin.
    pub fn make_box(shape: &mut Shape, dx: f64, dy: f64, dz: f64) -> TopoId {
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

        let edges = [
            // Bottom face
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(0.0, 0.0, 0.0), draper_geometry::direction::Direction3::X))),
                v[0], v[1], Some((0.0, dx)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(dx, 0.0, 0.0), draper_geometry::direction::Direction3::Y))),
                v[1], v[2], Some((0.0, dy)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(dx, dy, 0.0), draper_geometry::direction::Direction3::NEG_X))),
                v[2], v[3], Some((0.0, dx)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(0.0, dy, 0.0), draper_geometry::direction::Direction3::NEG_Y))),
                v[3], v[0], Some((0.0, dy)),
            ),
            // Top face
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(0.0, 0.0, dz), draper_geometry::direction::Direction3::X))),
                v[4], v[5], Some((0.0, dx)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(dx, 0.0, dz), draper_geometry::direction::Direction3::Y))),
                v[5], v[6], Some((0.0, dy)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(dx, dy, dz), draper_geometry::direction::Direction3::NEG_X))),
                v[6], v[7], Some((0.0, dx)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(0.0, dy, dz), draper_geometry::direction::Direction3::NEG_Y))),
                v[7], v[4], Some((0.0, dy)),
            ),
            // Vertical edges
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(0.0, 0.0, 0.0), draper_geometry::direction::Direction3::Z))),
                v[0], v[4], Some((0.0, dz)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(dx, 0.0, 0.0), draper_geometry::direction::Direction3::Z))),
                v[1], v[5], Some((0.0, dz)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(dx, dy, 0.0), draper_geometry::direction::Direction3::Z))),
                v[2], v[6], Some((0.0, dz)),
            ),
            shape.add_edge(
                Some(Curve::Line(Line::new(Point3::new(0.0, dy, 0.0), draper_geometry::direction::Direction3::Z))),
                v[3], v[7], Some((0.0, dz)),
            ),
        ];

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

        let shell = shape.add_shell(faces.to_vec());
        shape.add_solid(shell)
    }

    /// Create a cylinder along the Z axis, centered at origin on bottom face.
    pub fn make_cylinder(shape: &mut Shape, radius: f64, height: f64) -> TopoId {
        Self::make_cylinder_at(shape, radius, height,
            Axis2Placement3D::new(Point3::new(0.0, 0.0, 0.0), draper_geometry::direction::Direction3::Z, None),
        )
    }

    /// Create a cylinder along an arbitrary axis.
    ///
    /// The axis defines the cylinder's position, direction, and reference direction.
    /// The cylinder extends from the axis location along the axis direction for `height`.
    pub fn make_cylinder_at(shape: &mut Shape, radius: f64, height: f64, axis: Axis2Placement3D) -> TopoId {
        let segments = 24;
        let center_bottom = axis.location;
        let center_top = center_bottom + axis.axis.to_dvec3() * height;

        let x_dir = axis.ref_direction.to_dvec3();
        let y_dir = axis.y_direction().to_dvec3();
        let z_dir = axis.axis.to_dvec3();

        // Create circle edges (3D curves)
        let bottom_circle = Circle::new(
            Axis2Placement3D::new(center_bottom, axis.axis, Some(axis.ref_direction)),
            radius,
        );
        let top_circle = Circle::new(
            Axis2Placement3D::new(center_top, axis.axis, Some(axis.ref_direction)),
            radius,
        );

        // Create vertices on bottom and top circles
        let mut bottom_verts = Vec::with_capacity(segments);
        let mut top_verts = Vec::with_capacity(segments);

        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            let pt = center_bottom.to_dvec3()
                + x_dir * (radius * angle.cos())
                + y_dir * (radius * angle.sin());
            bottom_verts.push(shape.add_vertex(Point3::from_dvec3(pt)));

            let pt_top = center_top.to_dvec3()
                + x_dir * (radius * angle.cos())
                + y_dir * (radius * angle.sin());
            top_verts.push(shape.add_vertex(Point3::from_dvec3(pt_top)));
        }

        // Bottom circle edges
        let mut bottom_edges = Vec::with_capacity(segments);
        for i in 0..segments {
            let next = (i + 1) % segments;
            let t1 = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            let t2 = 2.0 * std::f64::consts::PI * (i + 1) as f64 / segments as f64;
            bottom_edges.push(shape.add_edge(
                Some(Curve::Circle(Circle::new(
                    Axis2Placement3D::new(center_bottom, axis.axis, Some(axis.ref_direction)),
                    radius,
                ))),
                bottom_verts[i],
                bottom_verts[next],
                Some((t1, t2)),
            ));
        }

        // Top circle edges
        let mut top_edges = Vec::with_capacity(segments);
        for i in 0..segments {
            let next = (i + 1) % segments;
            let t1 = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            let t2 = 2.0 * std::f64::consts::PI * (i + 1) as f64 / segments as f64;
            top_edges.push(shape.add_edge(
                Some(Curve::Circle(Circle::new(
                    Axis2Placement3D::new(center_top, axis.axis, Some(axis.ref_direction)),
                    radius,
                ))),
                top_verts[i],
                top_verts[next],
                Some((t1, t2)),
            ));
        }

        // Vertical edges (generators)
        let mut vertical_edges = Vec::with_capacity(segments);
        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            let pt_bottom = center_bottom.to_dvec3()
                + x_dir * (radius * angle.cos())
                + y_dir * (radius * angle.sin());
            vertical_edges.push(shape.add_edge(
                Some(Curve::Line(Line::new(
                    Point3::from_dvec3(pt_bottom),
                    draper_geometry::direction::Direction3::new(z_dir.x, z_dir.y, z_dir.z)
                        .unwrap_or(draper_geometry::direction::Direction3::Z),
                ))),
                bottom_verts[i],
                top_verts[i],
                Some((0.0, height)),
            ));
        }

        // Bottom wire (CCW when viewed from below = CW in Z direction)
        let bottom_wire = shape.add_wire(
            bottom_edges.iter().map(|&e| OrientedEdge { edge_id: e, orientation: true }).collect()
        );

        // Top wire (CCW when viewed from above)
        let top_wire = shape.add_wire(
            top_edges.iter().map(|&e| OrientedEdge { edge_id: e, orientation: true }).collect()
        );

        // Lateral faces (one per segment) with cylindrical surface
        let mut lateral_wires = Vec::new();
        for i in 0..segments {
            let next = (i + 1) % segments;
            let wire = shape.add_wire(vec![
                OrientedEdge { edge_id: vertical_edges[i], orientation: true },
                OrientedEdge { edge_id: top_edges[i], orientation: true },
                OrientedEdge { edge_id: vertical_edges[next], orientation: false },
                OrientedEdge { edge_id: bottom_edges[i], orientation: false },
            ]);
            lateral_wires.push(wire);
        }

        // Create faces
        let bottom_face = shape.add_face(Some(Surface::Plane(Plane::new(
            Axis2Placement3D::new(center_bottom, draper_geometry::direction::Direction3::new(-z_dir.x, -z_dir.y, -z_dir.z)
                .unwrap_or(draper_geometry::direction::Direction3::NEG_Z), None),
        ))));
        shape.set_face_outer_wire(bottom_face, bottom_wire);

        let top_face = shape.add_face(Some(Surface::Plane(Plane::new(
            Axis2Placement3D::new(center_top, draper_geometry::direction::Direction3::new(z_dir.x, z_dir.y, z_dir.z)
                .unwrap_or(draper_geometry::direction::Direction3::Z), None),
        ))));
        shape.set_face_outer_wire(top_face, top_wire);

        // Lateral face — one cylindrical surface for the whole lateral
        let lateral_face = shape.add_face(Some(Surface::CylindricalSurface(CylindricalSurface::new(
            axis.clone(),
            radius,
        ))));

        let mut all_faces = vec![bottom_face, top_face, lateral_face];
        for wire in lateral_wires {
            let face = shape.add_face(None);
            shape.set_face_outer_wire(face, wire);
            all_faces.push(face);
        }

        let shell = shape.add_shell(all_faces);
        shape.add_solid(shell)
    }

    /// Create a cone (or truncated cone) along the Z axis.
    ///
    /// `radius_bottom` — radius at the base (z = 0)
    /// `radius_top` — radius at the top (z = height). Set to 0.0 for a full cone.
    /// `height` — height of the cone
    pub fn make_cone(shape: &mut Shape, radius_bottom: f64, radius_top: f64, height: f64) -> TopoId {
        let segments = 24;
        let center_bottom = Point3::new(0.0, 0.0, 0.0);
        let center_top = Point3::new(0.0, 0.0, height);

        let semi_angle = if radius_bottom > 0.0 && height > 0.0 {
            (radius_bottom - radius_top).atan2(height)
        } else {
            0.0
        };

        // Vertices
        let mut bottom_verts = Vec::with_capacity(segments);
        let mut top_verts = Vec::with_capacity(segments);

        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            bottom_verts.push(shape.add_vertex(Point3::new(
                radius_bottom * angle.cos(),
                radius_bottom * angle.sin(),
                0.0,
            )));
            top_verts.push(shape.add_vertex(Point3::new(
                radius_top * angle.cos(),
                radius_top * angle.sin(),
                height,
            )));
        }

        // Bottom edges
        let mut bottom_edges = Vec::new();
        for i in 0..segments {
            let next = (i + 1) % segments;
            bottom_edges.push(shape.add_edge(
                Some(Curve::Circle(Circle::new(
                    Axis2Placement3D::new(center_bottom, draper_geometry::direction::Direction3::Z, None),
                    radius_bottom,
                ))),
                bottom_verts[i], bottom_verts[next],
                Some((2.0 * std::f64::consts::PI * i as f64 / segments as f64,
                      2.0 * std::f64::consts::PI * (i + 1) as f64 / segments as f64)),
            ));
        }

        // Top edges (only if radius_top > 0)
        let mut top_edges = Vec::new();
        if radius_top > 1e-10 {
            for i in 0..segments {
                let next = (i + 1) % segments;
                top_edges.push(shape.add_edge(
                    Some(Curve::Circle(Circle::new(
                        Axis2Placement3D::new(center_top, draper_geometry::direction::Direction3::Z, None),
                        radius_top,
                    ))),
                    top_verts[i], top_verts[next],
                    Some((2.0 * std::f64::consts::PI * i as f64 / segments as f64,
                          2.0 * std::f64::consts::PI * (i + 1) as f64 / segments as f64)),
                ));
            }
        }

        // Vertical (slant) edges
        let mut slant_edges = Vec::new();
        for i in 0..segments {
            slant_edges.push(shape.add_edge(None, bottom_verts[i], top_verts[i], None));
        }

        // Wires
        let bottom_wire = shape.add_wire(
            bottom_edges.iter().map(|&e| OrientedEdge { edge_id: e, orientation: true }).collect()
        );

        let mut top_wire_id = None;
        if radius_top > 1e-10 {
            top_wire_id = Some(shape.add_wire(
                top_edges.iter().map(|&e| OrientedEdge { edge_id: e, orientation: true }).collect()
            ));
        }

        // Lateral wires
        let mut lateral_wires = Vec::new();
        for i in 0..segments {
            let next = (i + 1) % segments;
            let mut wire_edges = vec![
                OrientedEdge { edge_id: slant_edges[i], orientation: true },
            ];
            if radius_top > 1e-10 {
                wire_edges.push(OrientedEdge { edge_id: top_edges[i], orientation: true });
                wire_edges.push(OrientedEdge { edge_id: slant_edges[next], orientation: false });
            } else {
                wire_edges.push(OrientedEdge { edge_id: slant_edges[next], orientation: false });
            }
            wire_edges.push(OrientedEdge { edge_id: bottom_edges[i], orientation: false });
            lateral_wires.push(shape.add_wire(wire_edges));
        }

        // Faces
        let bottom_face = shape.add_face(Some(Surface::Plane(Plane::new(
            Axis2Placement3D::new(center_bottom, draper_geometry::direction::Direction3::NEG_Z, None),
        ))));
        shape.set_face_outer_wire(bottom_face, bottom_wire);

        let mut all_faces = vec![bottom_face];

        if let Some(tw) = top_wire_id {
            let top_face = shape.add_face(Some(Surface::Plane(Plane::new(
                Axis2Placement3D::new(center_top, draper_geometry::direction::Direction3::Z, None),
            ))));
            shape.set_face_outer_wire(top_face, tw);
            all_faces.push(top_face);
        }

        // Conical lateral face
        let conical_face = shape.add_face(Some(Surface::ConicalSurface(ConicalSurface::new(
            Axis2Placement3D::new(center_bottom, draper_geometry::direction::Direction3::Z, None),
            radius_bottom,
            semi_angle,
        ))));
        all_faces.push(conical_face);

        for wire in lateral_wires {
            let face = shape.add_face(None);
            shape.set_face_outer_wire(face, wire);
            all_faces.push(face);
        }

        let shell = shape.add_shell(all_faces);
        shape.add_solid(shell)
    }

    /// Create a sphere centered at the origin.
    pub fn make_sphere(shape: &mut Shape, radius: f64) -> TopoId {
        let segments_u = 24; // Azimuthal (longitude)
        let segments_v = 12; // Polar (latitude)

        let center = Point3::new(0.0, 0.0, 0.0);

        // Create vertices on a latitude-longitude grid
        // Index: v_row * segments_u + u_col
        // Rows: 0 = south pole, segments_v = north pole
        let mut grid_verts = Vec::with_capacity((segments_v + 1) * segments_u);
        for j in 0..=segments_v {
            let v = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * j as f64 / segments_v as f64;
            for i in 0..segments_u {
                let u = 2.0 * std::f64::consts::PI * i as f64 / segments_u as f64;
                let x = radius * v.cos() * u.cos();
                let y = radius * v.cos() * u.sin();
                let z = radius * v.sin();
                grid_verts.push(shape.add_vertex(Point3::new(x, y, z)));
            }
        }

        // Create edges: latitude circles and longitude arcs
        // Latitude edges (horizontal circles)
        let mut lat_edges = Vec::with_capacity((segments_v + 1) * segments_u);
        for j in 0..=segments_v {
            let v = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * j as f64 / segments_v as f64;
            let row_start = j * segments_u;
            for i in 0..segments_u {
                let next_i = (i + 1) % segments_u;
                let idx_a = row_start + i;
                let idx_b = row_start + next_i;
                let t1 = 2.0 * std::f64::consts::PI * i as f64 / segments_u as f64;
                let t2 = 2.0 * std::f64::consts::PI * (i + 1) as f64 / segments_u as f64;

                let circle_r = radius * v.cos();
                let circle_z = radius * v.sin();
                let edge_curve = if circle_r > 1e-10 {
                    Some(Curve::Circle(Circle::new(
                        Axis2Placement3D::new(
                            Point3::new(0.0, 0.0, circle_z),
                            draper_geometry::direction::Direction3::Z,
                            None,
                        ),
                        circle_r,
                    )))
                } else {
                    None
                };
                lat_edges.push(shape.add_edge(
                    edge_curve,
                    grid_verts[idx_a], grid_verts[idx_b],
                    Some((t1, t2)),
                ));
            }
        }

        // Longitude edges (vertical arcs)
        let mut lon_edges = Vec::with_capacity(segments_v * segments_u);
        for i in 0..segments_u {
            for j in 0..segments_v {
                let idx_a = j * segments_u + i;
                let idx_b = (j + 1) * segments_u + i;
                lon_edges.push(shape.add_edge(None, grid_verts[idx_a], grid_verts[idx_b], None));
            }
        }

        // Create faces (quads split into 2 triangles each)
        let spherical_surface = Surface::SphericalSurface(SphericalSurface::new(
            Axis2Placement3D::new(center, draper_geometry::direction::Direction3::Z, None),
            radius,
        ));

        let mut all_faces = Vec::new();
        for j in 0..segments_v {
            for i in 0..segments_u {
                let next_i = (i + 1) % segments_u;

                // Indices
                let bl = j * segments_u + i;        // bottom-left
                let br = j * segments_u + next_i;   // bottom-right
                let tl = (j + 1) * segments_u + i;  // top-left
                let tr = (j + 1) * segments_u + next_i; // top-right

                // Edges
                let e_bottom = lat_edges[j * segments_u + i];        // bottom latitude edge
                let e_top = lat_edges[(j + 1) * segments_u + i];    // top latitude edge
                let e_left = lon_edges[j * segments_u + i];          // left longitude edge
                let e_right = lon_edges[j * segments_u + next_i];    // right longitude edge

                // Wire (quad)
                let wire = shape.add_wire(vec![
                    OrientedEdge { edge_id: e_bottom, orientation: true },
                    OrientedEdge { edge_id: e_right, orientation: true },
                    OrientedEdge { edge_id: e_top, orientation: false },
                    OrientedEdge { edge_id: e_left, orientation: false },
                ]);

                let face = shape.add_face(Some(spherical_surface.clone()));
                shape.set_face_outer_wire(face, wire);
                all_faces.push(face);
            }
        }

        let shell = shape.add_shell(all_faces);
        shape.add_solid(shell)
    }

    /// Create a torus centered at the origin with the main axis along Z.
    ///
    /// `major_radius` — distance from center of tube to center of torus
    /// `minor_radius` — radius of the tube
    pub fn make_torus(shape: &mut Shape, major_radius: f64, minor_radius: f64) -> TopoId {
        let segments_u = 24; // Around main axis
        let segments_v = 12; // Around tube

        let center = Point3::new(0.0, 0.0, 0.0);

        // Create vertices
        let mut grid_verts = Vec::with_capacity(segments_u * segments_v);
        for i in 0..segments_u {
            let u = 2.0 * std::f64::consts::PI * i as f64 / segments_u as f64;
            for j in 0..segments_v {
                let v = 2.0 * std::f64::consts::PI * j as f64 / segments_v as f64;
                let x = (major_radius + minor_radius * v.cos()) * u.cos();
                let y = (major_radius + minor_radius * v.cos()) * u.sin();
                let z = minor_radius * v.sin();
                grid_verts.push(shape.add_vertex(Point3::new(x, y, z)));
            }
        }

        // U-direction edges (around main axis)
        let mut u_edges = Vec::with_capacity(segments_u * segments_v);
        for i in 0..segments_u {
            let next_i = (i + 1) % segments_u;
            for j in 0..segments_v {
                let idx_a = i * segments_v + j;
                let idx_b = next_i * segments_v + j;
                u_edges.push(shape.add_edge(None, grid_verts[idx_a], grid_verts[idx_b], None));
            }
        }

        // V-direction edges (around tube)
        let mut v_edges = Vec::with_capacity(segments_u * segments_v);
        for i in 0..segments_u {
            for j in 0..segments_v {
                let next_j = (j + 1) % segments_v;
                let idx_a = i * segments_v + j;
                let idx_b = i * segments_v + next_j;
                v_edges.push(shape.add_edge(None, grid_verts[idx_a], grid_verts[idx_b], None));
            }
        }

        // Create faces
        let toroidal_surface = Surface::ToroidalSurface(ToroidalSurface::new(
            Axis2Placement3D::new(center, draper_geometry::direction::Direction3::Z, None),
            major_radius,
            minor_radius,
        ));

        let mut all_faces = Vec::new();
        for i in 0..segments_u {
            for j in 0..segments_v {
                let next_j = (j + 1) % segments_v;

                let e_u0 = u_edges[i * segments_v + j];
                let e_u1 = u_edges[i * segments_v + next_j];
                let e_v0 = v_edges[i * segments_v + j];
                let e_v1_next = v_edges[((i + 1) % segments_u) * segments_v + j];

                let wire = shape.add_wire(vec![
                    OrientedEdge { edge_id: e_v0, orientation: true },
                    OrientedEdge { edge_id: e_u1, orientation: true },
                    OrientedEdge { edge_id: e_v1_next, orientation: false },
                    OrientedEdge { edge_id: e_u0, orientation: false },
                ]);

                let face = shape.add_face(Some(toroidal_surface.clone()));
                shape.set_face_outer_wire(face, wire);
                all_faces.push(face);
            }
        }

        let shell = shape.add_shell(all_faces);
        shape.add_solid(shell)
    }

    /// Create a box with cylindrical holes (for engine block cylinder bores).
    ///
    /// The box extends from (0,0,0) to (dx, dy, dz).
    /// Each bore is defined by (x_center, y_center, radius) and extends
    /// through the full height of the box (along Z axis).
    pub fn make_box_with_cylinder_holes(
        shape: &mut Shape,
        dx: f64, dy: f64, dz: f64,
        bores: &[(f64, f64, f64)], // (x_center, y_center, radius)
    ) -> TopoId {
        if bores.is_empty() {
            return Self::make_box(shape, dx, dy, dz);
        }

        let segments = 24;

        // Create 8 box vertices
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

        // Box edges
        let box_edges = [
            shape.add_edge(None, v[0], v[1], None),
            shape.add_edge(None, v[1], v[2], None),
            shape.add_edge(None, v[2], v[3], None),
            shape.add_edge(None, v[3], v[0], None),
            shape.add_edge(None, v[4], v[5], None),
            shape.add_edge(None, v[5], v[6], None),
            shape.add_edge(None, v[6], v[7], None),
            shape.add_edge(None, v[7], v[4], None),
            shape.add_edge(None, v[0], v[4], None),
            shape.add_edge(None, v[1], v[5], None),
            shape.add_edge(None, v[2], v[6], None),
            shape.add_edge(None, v[3], v[7], None),
        ];

        // Create bore circle vertices and edges
        let mut bore_data = Vec::new();
        for &(cx, cy, radius) in bores {
            let mut bottom_verts = Vec::with_capacity(segments);
            let mut top_verts = Vec::with_capacity(segments);
            let mut bottom_edges = Vec::new();
            let mut top_edges = Vec::new();
            let mut vert_edges = Vec::new();

            for i in 0..segments {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
                let x = cx + radius * angle.cos();
                let y = cy + radius * angle.sin();
                bottom_verts.push(shape.add_vertex(Point3::new(x, y, 0.0)));
                top_verts.push(shape.add_vertex(Point3::new(x, y, dz)));
            }

            for i in 0..segments {
                let next = (i + 1) % segments;
                bottom_edges.push(shape.add_edge(
                    Some(Curve::Circle(Circle::new(
                        Axis2Placement3D::new(Point3::new(cx, cy, 0.0), draper_geometry::direction::Direction3::Z, None),
                        radius,
                    ))),
                    bottom_verts[i], bottom_verts[next],
                    Some((2.0 * std::f64::consts::PI * i as f64 / segments as f64,
                          2.0 * std::f64::consts::PI * (i + 1) as f64 / segments as f64)),
                ));
                top_edges.push(shape.add_edge(
                    Some(Curve::Circle(Circle::new(
                        Axis2Placement3D::new(Point3::new(cx, cy, dz), draper_geometry::direction::Direction3::Z, None),
                        radius,
                    ))),
                    top_verts[i], top_verts[next],
                    Some((2.0 * std::f64::consts::PI * i as f64 / segments as f64,
                          2.0 * std::f64::consts::PI * (i + 1) as f64 / segments as f64)),
                ));
                vert_edges.push(shape.add_edge(None, bottom_verts[i], top_verts[i], None));
            }

            bore_data.push((bottom_verts, top_verts, bottom_edges, top_edges, vert_edges, cx, cy, radius));
        }

        // Build wires and faces

        // Bottom face — outer wire (CW when viewed from below, i.e., CCW in +Z)
        let bottom_wire = shape.add_wire(vec![
            OrientedEdge { edge_id: box_edges[0], orientation: true },
            OrientedEdge { edge_id: box_edges[1], orientation: true },
            OrientedEdge { edge_id: box_edges[2], orientation: true },
            OrientedEdge { edge_id: box_edges[3], orientation: true },
        ]);

        // Inner wires for bottom face (hole boundaries — CW when viewed from above for holes)
        let mut bottom_inner_wires = Vec::new();
        for bd in &bore_data {
            let inner_wire = shape.add_wire(
                bd.2.iter().map(|&e| OrientedEdge { edge_id: e, orientation: false }).collect()
            );
            bottom_inner_wires.push(inner_wire);
        }

        let bottom_face = shape.add_face(Some(Surface::Plane(Plane::new(
            Axis2Placement3D::new(Point3::new(0.0, 0.0, 0.0), draper_geometry::direction::Direction3::NEG_Z, None),
        ))));
        shape.set_face_outer_wire(bottom_face, bottom_wire);
        for wire in &bottom_inner_wires {
            shape.add_face_inner_wire(bottom_face, *wire);
        }

        // Top face — outer wire + inner wires
        let top_wire = shape.add_wire(vec![
            OrientedEdge { edge_id: box_edges[4], orientation: true },
            OrientedEdge { edge_id: box_edges[5], orientation: true },
            OrientedEdge { edge_id: box_edges[6], orientation: true },
            OrientedEdge { edge_id: box_edges[7], orientation: true },
        ]);

        let mut top_inner_wires = Vec::new();
        for bd in &bore_data {
            let inner_wire = shape.add_wire(
                bd.3.iter().map(|&e| OrientedEdge { edge_id: e, orientation: true }).collect()
            );
            top_inner_wires.push(inner_wire);
        }

        let top_face = shape.add_face(Some(Surface::Plane(Plane::new(
            Axis2Placement3D::new(Point3::new(0.0, 0.0, dz), draper_geometry::direction::Direction3::Z, None),
        ))));
        shape.set_face_outer_wire(top_face, top_wire);
        for wire in &top_inner_wires {
            shape.add_face_inner_wire(top_face, *wire);
        }

        // Side faces (4 box sides)
        let side_wires = [
            shape.add_wire(vec![
                OrientedEdge { edge_id: box_edges[0], orientation: true },
                OrientedEdge { edge_id: box_edges[9], orientation: true },
                OrientedEdge { edge_id: box_edges[4], orientation: false },
                OrientedEdge { edge_id: box_edges[8], orientation: false },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: box_edges[1], orientation: true },
                OrientedEdge { edge_id: box_edges[10], orientation: true },
                OrientedEdge { edge_id: box_edges[5], orientation: false },
                OrientedEdge { edge_id: box_edges[9], orientation: false },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: box_edges[2], orientation: true },
                OrientedEdge { edge_id: box_edges[11], orientation: true },
                OrientedEdge { edge_id: box_edges[6], orientation: false },
                OrientedEdge { edge_id: box_edges[10], orientation: false },
            ]),
            shape.add_wire(vec![
                OrientedEdge { edge_id: box_edges[3], orientation: true },
                OrientedEdge { edge_id: box_edges[8], orientation: true },
                OrientedEdge { edge_id: box_edges[7], orientation: false },
                OrientedEdge { edge_id: box_edges[11], orientation: false },
            ]),
        ];

        let side_faces = [
            shape.add_face(Some(Surface::Plane(Plane::new(
                Axis2Placement3D::new(Point3::new(0.0, 0.0, 0.0), draper_geometry::direction::Direction3::NEG_Y, None),
            )))),
            shape.add_face(Some(Surface::Plane(Plane::new(
                Axis2Placement3D::new(Point3::new(dx, 0.0, 0.0), draper_geometry::direction::Direction3::Y, None),
            )))),
            shape.add_face(Some(Surface::Plane(Plane::new(
                Axis2Placement3D::new(Point3::new(dx, dy, 0.0), draper_geometry::direction::Direction3::X, None),
            )))),
            shape.add_face(Some(Surface::Plane(Plane::new(
                Axis2Placement3D::new(Point3::new(0.0, dy, 0.0), draper_geometry::direction::Direction3::NEG_X, None),
            )))),
        ];

        for i in 0..4 {
            shape.set_face_outer_wire(side_faces[i], side_wires[i]);
        }

        let mut all_faces = vec![bottom_face, top_face, side_faces[0], side_faces[1], side_faces[2], side_faces[3]];

        // Cylindrical bore faces
        for bd in &bore_data {
            let (_, _, _, top_edges, vert_edges, cx, cy, radius) = bd;

            // Build lateral faces for each bore segment
            for i in 0..segments {
                let next = (i + 1) % segments;
                let wire = shape.add_wire(vec![
                    OrientedEdge { edge_id: vert_edges[i], orientation: true },
                    OrientedEdge { edge_id: top_edges[i], orientation: true },
                    OrientedEdge { edge_id: vert_edges[next], orientation: false },
                    OrientedEdge { edge_id: bd.2[i], orientation: false },
                ]);
                let face = shape.add_face(None);
                shape.set_face_outer_wire(face, wire);
                all_faces.push(face);
            }

            // One cylindrical surface face for the whole bore
            let cyl_face = shape.add_face(Some(Surface::CylindricalSurface(CylindricalSurface::new(
                Axis2Placement3D::new(Point3::new(*cx, *cy, 0.0), draper_geometry::direction::Direction3::Z, None),
                *radius,
            ))));
            all_faces.push(cyl_face);
        }

        let shell = shape.add_shell(all_faces);
        shape.add_solid(shell)
    }
}
