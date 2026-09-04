// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! B-Rep shape builders — high-level functions to create primitive shapes.

use crate::entity::*;
use draper_geometry::{
    Point3d, Direction3d, Vec3d,
    Curve3d, Circle,
    Surface, Plane, CylinderSurface, SphereSurface, ConeSurface, TorusSurface,
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

        // Create 6 faces (each with its boundary edges — 7.6b working lists)
        let rect_specs = [
            (v[0], v[1], v[2], v[3]), // Bottom (-Z)
            (v[4], v[7], v[6], v[5]), // Top (+Z)
            (v[0], v[4], v[5], v[1]), // Front (-Y)
            (v[3], v[2], v[6], v[7]), // Back (+Y)
            (v[0], v[3], v[7], v[4]), // Left (-X)
            (v[1], v[5], v[6], v[2]), // Right (+X)
        ];
        let mut faces = Vec::with_capacity(6);
        let mut working = Vec::with_capacity(6);
        for (p0, p1, p2, p3) in rect_specs {
            let (face, edges) = Self::make_rect_face(p0, p1, p2, p3);
            faces.push(face);
            working.push(edges);
        }

        let shell = Shell::new_closed(faces);
        // C5 7.6b: born store-first — the store and canonical edge_ids
        // exist from construction (shared box edges dedup by geometric key).
        Solid::from_edges_only(shell, working)
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

    /// Create a rectangular face from 4 corner points, together with its
    /// boundary edge list (C5 7.6b — construction payload, the caller
    /// hands it to `Solid::from_edges_only`).
    fn make_rect_face(p0: Point3d, p1: Point3d, p2: Point3d, p3: Point3d) -> (Face, Vec<Edge>) {
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

        let face = Face::new(Surface::Plane(plane), wire);
        (face, vec![e0, e1, e2, e3])
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
            start_vertex_point: None,
            vertex_end: None,
            end_vertex_point: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
            step_entity_id: None,
        };
        let bottom_coedge = CoEdge::new(bottom_edge.id, false); // Reversed for bottom (looking from -Z)
        let bottom_wire = Wire::new(vec![bottom_coedge]);
        let bottom_face = Face::new(Surface::Plane(Plane::xy()), bottom_wire);

        // === Top face (disk) ===
        let top_circle = Circle::new_xy(top_center, radius);
        let top_edge = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(top_circle)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            start_vertex_point: None,
            vertex_end: None,
            end_vertex_point: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
            step_entity_id: None,
        };
        let top_coedge = CoEdge::new(top_edge.id, true); // Forward for top (looking from +Z)
        let top_wire = Wire::new(vec![top_coedge]);
        let top_face = Face::new(
            Surface::Plane(Plane::from_origin_and_normal(top_center, Direction3d::Z)),
            top_wire,
        );

        // === Lateral face (cylinder surface) ===
        // Store bottom and top circle edges so compute_axis_v_range can
        // determine the height range. Wire is empty — the triangulation
        // uses the full cylinder path.
        //
        // When a boolean operation adds an intersection curve as an inner
        // wire (hole), the triangulation's fallback path handles it.
        let lateral_wire = Wire::new(vec![]);
        let lateral_face = Face::new(Surface::Cylinder(cyl_surface), lateral_wire);

        let shell = Shell::new_closed(vec![bottom_face, top_face, lateral_face]);
        // C5 7.6b: born store-first — bottom/top circle edges are shared
        // between the disk faces and the (wire-less) lateral face.
        let working = vec![
            vec![bottom_edge.clone()],
            vec![top_edge.clone()],
            vec![bottom_edge, top_edge],
        ];
        Solid::from_edges_only(shell, working)
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
        // Wire-less single face — no edge payload; store-first construction
        // with empty working lists is equivalent to the plain constructor.
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
            start_vertex_point: None,
            vertex_end: None,
            end_vertex_point: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
            step_entity_id: None,
        };
        let bottom_coedge = CoEdge::new(bottom_edge.id, false);
        let bottom_wire = Wire::new(vec![bottom_coedge]);
        let bottom_face = Face::new(Surface::Plane(Plane::xy()), bottom_wire);

        // Lateral cone face — store bottom circle edge so compute_axis_v_range
        // can determine the height range. Wire is empty — triangulation uses
        // the full cone path with apex degeneracy handling.
        let lateral_wire = Wire::new(vec![]);
        let lateral_face = Face::new(Surface::Cone(cone_surface), lateral_wire);

        let shell = Shell::new_closed(vec![bottom_face, lateral_face]);
        // C5 7.6b: born store-first — the bottom circle edge is shared
        // between the disk face and the (wire-less) lateral face.
        let working = vec![vec![bottom_edge.clone()], vec![bottom_edge]];
        Solid::from_edges_only(shell, working)
    }

    /// Create a torus.
    pub fn make_torus(major_radius: f64, minor_radius: f64) -> Solid {
        let torus_surface = TorusSurface::new_z(Point3d::ORIGIN, major_radius, minor_radius);

        // The boundary edge is the MINOR circle of the torus at u=0,
        // which lies in the XZ plane (containing the torus axis).
        //
        // IMPORTANT: This circle must be a constant-u curve on the torus
        // (i.e., project_point must return u=0 for every point on it) so
        // that the UV boundary is degenerate (u_range ≈ 0) and
        // `triangulate_torus_face` routes to `triangulate_torus_full_grid`
        // which generates a proper doubly-periodic grid.
        //
        // The previous code used Circle::new_xy which creates a circle in
        // the XY plane — that is NOT a constant-u curve on the torus
        // (project_point returns u varying in [-asin(r/R), +asin(r/R)]),
        // so the UV boundary was non-degenerate and got routed to
        // `triangulate_surface_consistent` which produced a terrible
        // self-intersecting UV polygon.
        //
        // For the circle to evaluate (R + r*cos(t), 0, r*sin(t)) matching
        // torus.point_at(0, t), we need:
        //   center = (R, 0, 0), normal = +Y, x_axis = +X
        //   y_axis = normal × x_axis = Y × X = -Z
        //   point_at(t) = center + r*(cos(t)*X + sin(t)*(-Z)) = (R+r*cos(t), 0, -r*sin(t))
        //
        // That gives the wrong sign on z. Flip normal to -Y:
        //   x_axis = (-Y) × Z = -X (wrong, we want +X).
        //
        // So we construct the Circle directly with the fields we need:
        //   normal = -Y, x_axis = +X
        //   y_axis = (-Y) × X = +(Y × X) wait no, (-Y)×X = -(Y×X) = -(-Z) = Z
        //   point_at(t) = center + r*(cos(t)*X + sin(t)*Z) = (R+r*cos(t), 0, r*sin(t)) ✓
        let circle_v = Circle {
            center: Point3d::new(major_radius, 0.0, 0.0),
            normal: Direction3d::new(0.0, -1.0, 0.0).unwrap_or(Direction3d::Y),
            radius: minor_radius,
            x_axis: Direction3d::X,
        };

        let edge_v = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(circle_v)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            start_vertex_point: None,
            vertex_end: None,
            end_vertex_point: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
            step_entity_id: None,
        };

        let coedges = vec![CoEdge::new(edge_v.id, true)];
        let wire = Wire::new(coedges);
        let face = Face::new(Surface::Torus(torus_surface), wire);

        let shell = Shell::new_closed(vec![face]);
        Solid::from_edges_only(shell, vec![vec![edge_v]])
    }

    /// Create a solid of revolution by revolving a profile curve around the Z axis.
    pub fn make_revolution(profile: Curve3d, _angle: f64) -> Solid {
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
    pub fn make_extrusion(profile: Curve3d, direction: Direction3d, _distance: f64) -> Solid {
        let ext_surface = Surface::Extrusion(draper_geometry::ExtrusionSurface {
            profile,
            direction,
        });

        // Simplified: single face extrusion (wire-less — no edge payload)
        let wire = Wire::new(vec![]);
        let face = Face::new(ext_surface, wire);
        let shell = Shell::new_closed(vec![face]);
        Solid::new(shell)
    }

    /// Transform a solid (apply transformation to all geometry).
    ///
    /// C5 Stage 7.1: transforms the surface, the mirror edge curves AND the
    /// canonical store curves, then re-indexes. The store pass is what keeps
    /// MIRROR-FREE (compacted) faces correct: their edge payload lives only
    /// in the store, and `index_edges` Pass 0 re-seeds the rebuilt store
    /// from these already-transformed canonical copies. Re-indexing is not
    /// optional for mirror-carrying faces either — born-indexed primitives
    /// hold pre-transform canonicals in the store, and without the rebuild
    /// every store-first reader (mesh, queries, exporters) would sample
    /// stale curves after a transform.
    pub fn transform_solid(solid: &mut Solid, transform: &Transform) {
        if let Some(ref mut shell) = solid.outer_shell {
            for face in &mut shell.faces {
                // Transform surface
                if let Some(ref mut surface) = face.surface {
                    *surface = surface.transform(transform);
                }
            }
        }
        // Transform the canonical store curves — the store is the ONLY
        // holder of edge geometry (C5 7.6b), so this IS the whole edge
        // transform: identity (ids, aliases, orientations) is unaffected.
        solid.edge_store.transform_curves(transform);
    }

    /// Create a polygonal face from a list of 3D points, together with
    /// its boundary edge list (C5 7.6b — construction payload for
    /// `Solid::from_edges_only` / `rebuild_store`).
    pub fn make_polygon_face(points: &[Point3d]) -> Option<(Face, Vec<Edge>)> {
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

        // Try to build a plane from three points; if that fails (collinear),
        // try other point triples or construct from a best-fit normal.
        let plane = Plane::from_three_points(&points[0], &points[1], &points[2])
            .or_else(|| {
                // Try points 0, 2, 3 (skip the middle one)
                if points.len() >= 4 {
                    Plane::from_three_points(&points[0], &points[2], &points[3])
                } else {
                    None
                }
            })
            .or_else(|| {
                // Last resort: construct a plane from the best-fit normal
                // computed via cross product of non-parallel edges.
                let mut normal = Vec3d::new(0.0, 0.0, 1.0);
                let mut found = false;
                for i in 1..n {
                    let e1 = Vec3d::new(
                        points[i].x - points[0].x,
                        points[i].y - points[0].y,
                        points[i].z - points[0].z,
                    );
                    for j in (i + 1)..n {
                        let e2 = Vec3d::new(
                            points[j].x - points[0].x,
                            points[j].y - points[0].y,
                            points[j].z - points[0].z,
                        );
                        let cross = Vec3d::new(
                            e1.y * e2.z - e1.z * e2.y,
                            e1.z * e2.x - e1.x * e2.z,
                            e1.x * e2.y - e1.y * e2.x,
                        );
                        let len = (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z).sqrt();
                        if len > 1e-10 {
                            normal = Vec3d::new(cross.x / len, cross.y / len, cross.z / len);
                            found = true;
                            break;
                        }
                    }
                    if found {
                        break;
                    }
                }
                if found {
                    // Construct a plane with this normal through points[0]
                    Plane::from_normal_and_point(
                        &draper_geometry::Direction3d::new(normal.x, normal.y, normal.z)?,
                        &points[0],
                    )
                } else {
                    None
                }
            })?;

        let face = Face::new(Surface::Plane(plane), wire);
        Some((face, edges))
    }

    /// Create a circular disk face together with its boundary circle
    /// edge (C5 7.6b — construction payload for `Solid::from_edges_only`).
    pub fn make_disk(center: Point3d, normal: Direction3d, radius: f64) -> (Face, Vec<Edge>) {
        let circle = Circle::new(center, normal, radius);
        let edge = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(circle)),
            param_range: (0.0, 2.0 * PI),
            vertex_start: None,
            start_vertex_point: None,
            vertex_end: None,
            end_vertex_point: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
            step_entity_id: None,
        };
        let coedge = CoEdge::new(edge.id, true);
        let wire = Wire::new(vec![coedge]);
        let plane = Plane::from_origin_and_normal(center, normal);
        let face = Face::new(Surface::Plane(plane), wire);
        (face, vec![edge])
    }
}
