//! B-Rep shape builders — high-level functions to create primitive shapes.

use crate::entity::*;
use draper_geometry::{
    Point3d, Point2d, Direction3d, Vec3d,
    Curve3d, Line, Circle, Arc,
    Surface, Plane, CylinderSurface, SphereSurface, ConeSurface, TorusSurface,
    RevolutionSurface, ExtrusionSurface,
    Transform,
};
use std::f64::consts::PI;

/// Builder for creating B-Rep shapes.
pub struct ShapeBuilder;

impl ShapeBuilder {
    /// Create a box (parallelepiped) centered at the origin.
    /// The box spans from (-dx/2, -dy/2, -dz/2) to (dx/2, dy/2, dz/2).
    pub fn make_box(dx: f64, dy: f64, dz: f64) -> Solid {
        let hx = dx / 2.0;
        let hy = dy / 2.0;
        let hz = dz / 2.0;

        // 8 vertices of the box
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

        // Create 6 faces
        let faces = vec![
            // Bottom (-Z)
            Self::make_rect_face(v[0], v[1], v[2], v[3], Plane::xz()), // Bottom face
            // Top (+Z)
            Self::make_rect_face(v[4], v[7], v[6], v[5], Plane::xz()), // Top face
            // Front (-Y)
            Self::make_rect_face(v[0], v[4], v[5], v[1], Plane::xy()), // Front face
            // Back (+Y)
            Self::make_rect_face(v[3], v[2], v[6], v[7], Plane::xy()), // Back face
            // Left (-X)
            Self::make_rect_face(v[0], v[3], v[7], v[4], Plane::yz()), // Left face
            // Right (+X)
            Self::make_rect_face(v[1], v[5], v[6], v[2], Plane::yz()), // Right face
        ];

        let shell = Shell::new_closed(faces);
        Solid::new(shell)
    }

    /// Create a box at a specific position (min corner).
    pub fn make_box_at(x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64) -> Solid {
        let mut box_solid = Self::make_box(dx, dy, dz);
        // Translate
        Self::transform_solid(&mut box_solid, &Transform::translation(
            x + dx / 2.0, y + dy / 2.0, z + dz / 2.0
        ));
        box_solid
    }

    /// Create a rectangular face from 4 corner points.
    fn make_rect_face(p0: Point3d, p1: Point3d, p2: Point3d, p3: Point3d, _plane: Plane) -> Face {
        // Create 4 edges
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
        // Store edges in face so triangulation can sample edge curves
        face.edges = vec![e0, e1, e2, e3];
        face
    }

    /// Create a cylinder along the Z axis.
    /// height: total height along Z
    /// radius: cylinder radius
    ///
    /// Uses three faces: bottom disk, top disk, and lateral surface.
    /// The lateral surface stores bottom and top circle edges so that
    /// the triangulation can determine the height range.
    pub fn make_cylinder(radius: f64, height: f64) -> Solid {
        let cyl_surface = CylinderSurface::new_z(radius);
        let bottom_center = Point3d::new(0.0, 0.0, 0.0);
        let top_center = Point3d::new(0.0, 0.0, height);

        // === Bottom face (disk) ===
        let bottom_circle = Circle::new_xy(bottom_center, radius);
        let bottom_edge = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(bottom_circle)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
        };
        let bottom_coedge = CoEdge::new(bottom_edge.id, false); // Reversed for bottom (looking from -Z)
        let bottom_wire = Wire::new(vec![bottom_coedge]);
        let mut bottom_face = Face::new(Surface::Plane(Plane::xy()), bottom_wire);
        bottom_face.edges = vec![bottom_edge.clone()];

        // === Top face (disk) ===
        let top_circle = Circle::new_xy(top_center, radius);
        let top_edge = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(top_circle)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
        };
        let top_coedge = CoEdge::new(top_edge.id, true); // Forward for top (looking from +Z)
        let top_wire = Wire::new(vec![top_coedge]);
        let mut top_face = Face::new(
            Surface::Plane(Plane::from_origin_and_normal(top_center, Direction3d::Z)),
            top_wire,
        );
        top_face.edges = vec![top_edge.clone()];

        // === Lateral face (cylinder surface) ===
        // Store bottom and top circle edges so compute_axis_v_range can
        // determine the height range. Wire is empty — the triangulation
        // uses the full cylinder path.
        let lateral_wire = Wire::new(vec![]);
        let mut lateral_face = Face::new(Surface::Cylinder(cyl_surface), lateral_wire);
        lateral_face.edges = vec![bottom_edge, top_edge];

        let shell = Shell::new_closed(vec![bottom_face, top_face, lateral_face]);
        Solid::new(shell)
    }

    /// Create a cylinder at a specific position.
    pub fn make_cylinder_at(x: f64, y: f64, z: f64, radius: f64, height: f64) -> Solid {
        let mut cyl = Self::make_cylinder(radius, height);
        Self::transform_solid(&mut cyl, &Transform::translation(x, y, z));
        cyl
    }

    /// Create a sphere.
    ///
    /// Uses a single face with no boundary edges. The triangulation
    /// code handles the full sphere via the UV parameterization
    /// u ∈ [0, 2π], v ∈ [0, π] with proper pole handling.
    pub fn make_sphere(radius: f64) -> Solid {
        let sphere_surface = SphereSurface::new(Point3d::ORIGIN, radius);

        // Single face — no boundary edges; triangulation uses the full
        // sphere path which correctly handles pole degeneracy.
        let wire = Wire::new(vec![]);
        let face = Face::new(Surface::Sphere(sphere_surface), wire);

        let shell = Shell::new_closed(vec![face]);
        Solid::new(shell)
    }

    /// Create a cone.
    ///
    /// Uses two faces: bottom disk and lateral cone surface.
    /// The lateral face stores the bottom circle edge so that
    /// the triangulation can determine the height range.
    /// Handles apex degeneracy (all vertices collapse to a single
    /// point at the apex) via the cone surface parameterization.
    pub fn make_cone(radius: f64, _height: f64, half_angle: f64) -> Solid {
        let cone_surface = ConeSurface::new_z(radius, half_angle);

        // Bottom disk face
        let bottom_circle = Circle::new_xy(Point3d::ORIGIN, radius);
        let bottom_edge = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(bottom_circle)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
        };
        let bottom_coedge = CoEdge::new(bottom_edge.id, false);
        let bottom_wire = Wire::new(vec![bottom_coedge]);
        let mut bottom_face = Face::new(Surface::Plane(Plane::xy()), bottom_wire);
        bottom_face.edges = vec![bottom_edge.clone()];

        // Lateral cone face — store bottom circle edge so compute_axis_v_range
        // can determine the height range. Wire is empty — triangulation uses
        // the full cone path with apex degeneracy handling.
        let lateral_wire = Wire::new(vec![]);
        let mut lateral_face = Face::new(Surface::Cone(cone_surface), lateral_wire);
        lateral_face.edges = vec![bottom_edge];

        let shell = Shell::new_closed(vec![bottom_face, lateral_face]);
        Solid::new(shell)
    }

    /// Create a torus.
    pub fn make_torus(major_radius: f64, minor_radius: f64) -> Solid {
        let torus_surface = TorusSurface::new_z(Point3d::ORIGIN, major_radius, minor_radius);

        // Simplified: single face torus
        let circle_v = Circle::new_xy(
            Point3d::new(major_radius, 0.0, 0.0),
            minor_radius,
        );

        let edge_v = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(circle_v)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
        };

        let coedges = vec![CoEdge::new(edge_v.id, true)];
        let wire = Wire::new(coedges);
        let mut face = Face::new(Surface::Torus(torus_surface), wire);
        face.edges = vec![edge_v];

        let shell = Shell::new_closed(vec![face]);
        Solid::new(shell)
    }

    /// Create a solid of revolution by revolving a profile curve around the Z axis.
    pub fn make_revolution(profile: Curve3d, angle: f64) -> Solid {
        let rev_surface = Surface::Revolution(draper_geometry::RevolutionSurface {
            profile,
            axis: Direction3d::Z,
            origin: Point3d::ORIGIN,
        });

        // Simplified: single face revolution
        let wire = Wire::new(vec![]);
        let face = Face::new(rev_surface, wire);
        let shell = Shell::new_closed(vec![face]);
        Solid::new(shell)
    }

    /// Create a solid by extruding a profile curve along a direction.
    pub fn make_extrusion(profile: Curve3d, direction: Direction3d, distance: f64) -> Solid {
        let ext_surface = Surface::Extrusion(draper_geometry::ExtrusionSurface {
            profile,
            direction,
        });

        // Simplified: single face extrusion
        let wire = Wire::new(vec![]);
        let face = Face::new(ext_surface, wire);
        let shell = Shell::new_closed(vec![face]);
        Solid::new(shell)
    }

    /// Transform a solid (apply transformation to all geometry).
    pub fn transform_solid(solid: &mut Solid, transform: &Transform) {
        if let Some(ref mut shell) = solid.outer_shell {
            for face in &mut shell.faces {
                // Transform surface
                if let Some(ref mut surface) = face.surface {
                    *surface = surface.transform(transform);
                }
                // Transform edge curves stored in the face
                for edge in &mut face.edges {
                    if let Some(ref mut curve) = edge.curve {
                        *curve = curve.transform(transform);
                    }
                }
            }
        }
    }

    /// Create a polygonal face from a list of 3D points.
    pub fn make_polygon_face(points: &[Point3d]) -> Option<Face> {
        if points.len() < 3 {
            return None;
        }

        let mut edges = Vec::new();
        let n = points.len();
        for i in 0..n {
            let j = (i + 1) % n;
            edges.push(Edge::new_line(points[i], points[j]));
        }

        let coedges: Vec<CoEdge> = edges.iter().map(|e| CoEdge::new(e.id, true)).collect();
        let wire = Wire::new(coedges);

        let plane = Plane::from_three_points(&points[0], &points[1], &points[2])?;
        let mut face = Face::new(Surface::Plane(plane), wire);
        face.edges = edges;
        Some(face)
    }

    /// Create a circular disk face.
    pub fn make_disk(center: Point3d, normal: Direction3d, radius: f64) -> Face {
        let circle = Circle::new(center, normal, radius);
        let edge = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(circle)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
        };
        let coedge = CoEdge::new(edge.id, true);
        let wire = Wire::new(vec![coedge]);
        let plane = Plane::from_origin_and_normal(center, normal);
        let mut face = Face::new(Surface::Plane(plane), wire);
        face.edges = vec![edge];
        face
    }
}
