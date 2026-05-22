//! STEP entity resolution and conversion to triangle mesh.
//!
//! This module resolves the flat STEP entity list into a connected graph,
//! then converts the geometric and topological entities into a triangle mesh
//! that can be directly rendered.
//!
//! The conversion pipeline:
//! 1. Build entity lookup HashMap for fast reference resolution
//! 2. Find MANIFOLD_SOLID_BREP → CLOSED_SHELL → ADVANCED_FACEs
//! 3. For each face: resolve surface geometry and boundary vertices
//! 4. Triangulate each face (planar ear-clipping or parametric tessellation)
//! 5. Merge all face meshes into a single TriangleMesh

use std::collections::HashMap;
use std::f64::consts::PI;

use crate::schema::{StepEntity, StepFile, StepValue};
use draper_geometry::{
    Point3d, Point2d, Direction3d,
    Curve3d, Line, Circle, Surface, Plane, CylinderSurface, ConeSurface,
};
use draper_mesh::{TriangleMesh, TriangulationParams};

/// Result of STEP file analysis for diagnostics.
#[derive(Clone, Debug)]
pub struct StepDiagnostics {
    pub entity_count: usize,
    pub point_count: usize,
    pub face_count: usize,
    pub shell_count: usize,
    pub brep_count: usize,
    pub surface_types: Vec<String>,
    pub vertex_count: usize,
    pub triangle_count: usize,
}

/// STEP entity resolver and converter.
pub struct StepConverter<'a> {
    /// Fast entity lookup by ID.
    entities: HashMap<i64, &'a StepEntity>,
    /// Diagnostics: surface types found.
    surface_types: Vec<String>,
}

impl<'a> StepConverter<'a> {
    /// Create a new converter from a parsed STEP file.
    pub fn new(step_file: &'a StepFile) -> Self {
        let entities: HashMap<i64, &'a StepEntity> = step_file.entities
            .iter()
            .map(|e| (e.id, e))
            .collect();
        Self {
            entities,
            surface_types: Vec::new(),
        }
    }

    // ── Entity lookup helpers ──────────────────────────────

    fn entity(&self, id: i64) -> Option<&'a StepEntity> {
        self.entities.get(&id).copied()
    }

    fn get_float(val: &StepValue) -> Option<f64> {
        match val {
            StepValue::Float(f) => Some(*f),
            StepValue::Integer(i) => Some(*i as f64),
            _ => None,
        }
    }

    fn get_ref(val: &StepValue) -> Option<i64> {
        if let StepValue::Ref(id) = val { Some(*id) } else { None }
    }

    // ── Geometry extraction ────────────────────────────────

    /// Extract a 3D point from a CARTESIAN_POINT entity.
    fn get_cartesian_point(&self, id: i64) -> Option<Point3d> {
        let e = self.entity(id)?;
        // CARTESIAN_POINT(name, (x, y, z))
        if e.params.len() < 2 { return None; }
        if let StepValue::List(coords) = &e.params[1] {
            if coords.len() >= 3 {
                let x = Self::get_float(&coords[0])?;
                let y = Self::get_float(&coords[1])?;
                let z = Self::get_float(&coords[2])?;
                return Some(Point3d::new(x, y, z));
            }
        }
        None
    }

    /// Extract a direction from a DIRECTION entity.
    fn get_direction(&self, id: i64) -> Option<Direction3d> {
        let e = self.entity(id)?;
        if e.params.len() < 2 { return None; }
        if let StepValue::List(coords) = &e.params[1] {
            if coords.len() >= 3 {
                let dx = Self::get_float(&coords[0]).unwrap_or(0.0);
                let dy = Self::get_float(&coords[1]).unwrap_or(0.0);
                let dz = Self::get_float(&coords[2]).unwrap_or(0.0);
                let len = (dx * dx + dy * dy + dz * dz).sqrt();
                if len < 1e-10 {
                    return Some(Direction3d::Z);
                }
                return Direction3d::new(dx, dy, dz);
            }
        }
        None
    }

    /// Extract AXIS2_PLACEMENT_3D: (origin, axis_z, ref_dir_x).
    fn get_axis2_placement(&self, id: i64) -> Option<(Point3d, Direction3d, Direction3d)> {
        let e = self.entity(id)?;
        // AXIS2_PLACEMENT_3D(name, #point, #axis, #ref_dir)
        if e.params.len() < 2 { return None; }

        let point_id = Self::get_ref(&e.params[1])?;
        let origin = self.get_cartesian_point(point_id)?;

        let axis = if e.params.len() >= 3 {
            Self::get_ref(&e.params[2])
                .and_then(|id| self.get_direction(id))
                .unwrap_or(Direction3d::Z)
        } else { Direction3d::Z };

        let default_ref = if axis.is_parallel_to(&Direction3d::Y) {
            axis.cross(&Direction3d::X)
        } else {
            axis.cross(&Direction3d::Y)
        };

        let ref_dir = if e.params.len() >= 4 {
            Self::get_ref(&e.params[3])
                .and_then(|id| self.get_direction(id))
                .unwrap_or(default_ref)
        } else { default_ref };

        Some((origin, axis, ref_dir))
    }

    /// Convert a STEP surface entity to a draper Surface.
    fn convert_surface(&mut self, surface_id: i64) -> Option<Surface> {
        let e = self.entity(surface_id)?;
        match e.type_name.as_str() {
            "PLANE" => {
                // PLANE(name, #axis2) — params[0]=name, params[1]=axis2_ref
                let axis2_id = Self::get_ref(e.params.get(1)?)?;
                let (origin, axis, ref_dir) = self.get_axis2_placement(axis2_id)?;
                let v_dir = axis.cross(&ref_dir);
                self.surface_types.push("PLANE".to_string());
                Some(Surface::Plane(Plane {
                    origin, u_dir: ref_dir, v_dir, normal: axis,
                }))
            }
            "CYLINDRICAL_SURFACE" => {
                // CYLINDRICAL_SURFACE(name, #axis2, radius)
                let axis2_id = Self::get_ref(e.params.get(1)?)?;
                let radius = Self::get_float(e.params.get(2)?)?;
                let (origin, axis, _ref_dir) = self.get_axis2_placement(axis2_id)?;
                self.surface_types.push("CYLINDRICAL_SURFACE".to_string());
                Some(Surface::Cylinder(CylinderSurface { origin, axis, radius }))
            }
            "CONICAL_SURFACE" => {
                // CONICAL_SURFACE(name, #axis2, radius, semi_angle)
                let axis2_id = Self::get_ref(e.params.get(1)?)?;
                let radius = Self::get_float(e.params.get(2)?)?;
                let semi_angle = e.params.get(3)
                    .and_then(|v| Self::get_float(v))
                    .unwrap_or(0.0);
                let (origin, axis, _ref_dir) = self.get_axis2_placement(axis2_id)?;
                let tan_a = semi_angle.tan();
                let apex = if tan_a.abs() > 1e-10 {
                    Point3d::new(
                        origin.x - (radius / tan_a) * axis.x,
                        origin.y - (radius / tan_a) * axis.y,
                        origin.z - (radius / tan_a) * axis.z,
                    )
                } else { origin };
                self.surface_types.push("CONICAL_SURFACE".to_string());
                Some(Surface::Cone(ConeSurface { apex, axis, half_angle: semi_angle, radius }))
            }
            "SPHERICAL_SURFACE" => {
                // SPHERICAL_SURFACE(name, #axis2, radius)
                let axis2_id = Self::get_ref(e.params.get(1)?)?;
                let radius = Self::get_float(e.params.get(2)?)?;
                let (center, _axis, _ref_dir) = self.get_axis2_placement(axis2_id)?;
                self.surface_types.push("SPHERICAL_SURFACE".to_string());
                Some(Surface::Sphere(draper_geometry::SphereSurface { center, radius }))
            }
            "TOROIDAL_SURFACE" => {
                // TOROIDAL_SURFACE(name, #axis2, major_radius, minor_radius)
                let axis2_id = Self::get_ref(e.params.get(1)?)?;
                let major_radius = Self::get_float(e.params.get(2)?)?;
                let minor_radius = e.params.get(3)
                    .and_then(|v| Self::get_float(v))
                    .unwrap_or(0.0);
                let (center, axis, _ref_dir) = self.get_axis2_placement(axis2_id)?;
                self.surface_types.push("TOROIDAL_SURFACE".to_string());
                Some(Surface::Torus(draper_geometry::TorusSurface {
                    center, axis, major_radius, minor_radius,
                }))
            }
            other => {
                self.surface_types.push(other.to_string());
                log::warn!("Unsupported STEP surface type: {}", other);
                None
            }
        }
    }

    /// Convert a STEP curve entity to a draper Curve3d.
    fn convert_curve(&self, curve_id: i64) -> Option<Curve3d> {
        let e = self.entity(curve_id)?;
        match e.type_name.as_str() {
            "LINE" => {
                if e.params.len() < 3 { return None; }
                let point_id = Self::get_ref(&e.params[1])?;
                let origin = self.get_cartesian_point(point_id)?;

                let vector_id = Self::get_ref(&e.params[2])?;
                let vector_ent = self.entity(vector_id)?;
                if vector_ent.params.len() < 3 { return None; }
                let dir_id = Self::get_ref(&vector_ent.params[1])?;
                let dir = self.get_direction(dir_id)?;
                let magnitude = Self::get_float(&vector_ent.params[2]).unwrap_or(1.0);

                let direction = Direction3d::new(
                    dir.x * magnitude, dir.y * magnitude, dir.z * magnitude,
                ).unwrap_or(dir);

                Some(Curve3d::Line(Line { origin, direction }))
            }
            "CIRCLE" => {
                if e.params.len() < 3 { return None; }
                let axis2_id = Self::get_ref(&e.params[1])?;
                let (center, normal, x_axis) = self.get_axis2_placement(axis2_id)?;
                let radius = Self::get_float(&e.params[2])?;
                Some(Curve3d::Circle(Circle { center, normal, radius, x_axis }))
            }
            "ELLIPSE" => {
                if e.params.len() < 4 { return None; }
                let axis2_id = Self::get_ref(&e.params[1])?;
                let (center, normal, x_axis) = self.get_axis2_placement(axis2_id)?;
                let semi_major = Self::get_float(&e.params[2])?;
                let semi_minor = Self::get_float(&e.params[3])?;
                Some(Curve3d::Ellipse(draper_geometry::Ellipse {
                    center, normal, semi_major, semi_minor, x_axis,
                }))
            }
            other => {
                log::warn!("Unsupported STEP curve type: {}", other);
                None
            }
        }
    }

    // ── Topology extraction ────────────────────────────────

    /// Resolve VERTEX_POINT → CARTESIAN_POINT → Point3d.
    fn get_vertex_point(&self, vp_id: i64) -> Option<Point3d> {
        let vp_ent = self.entity(vp_id)?;
        // VERTEX_POINT(name, #point) — params[0]=name, params[1]=point_ref
        if vp_ent.params.len() < 2 { return None; }
        let cp_id = Self::get_ref(&vp_ent.params[1])?;
        self.get_cartesian_point(cp_id)
    }

    /// Get start and end points of an EDGE_CURVE.
    fn get_edge_curve_endpoints(&self, ec_id: i64) -> Option<(Point3d, Point3d)> {
        let ec_ent = self.entity(ec_id)?;
        if ec_ent.params.len() < 3 { return None; }
        let v1_id = Self::get_ref(&ec_ent.params[1])?;
        let v2_id = Self::get_ref(&ec_ent.params[2])?;
        let start_pt = self.get_vertex_point(v1_id)?;
        let end_pt = self.get_vertex_point(v2_id)?;
        Some((start_pt, end_pt))
    }

    /// Get the curve geometry and its parametric range from an EDGE_CURVE.
    fn get_edge_curve_info(&self, ec_id: i64) -> (Option<Curve3d>, (f64, f64)) {
        let ec_ent = match self.entity(ec_id) {
            Some(e) => e,
            None => return (None, (0.0, 1.0)),
        };
        if ec_ent.params.len() < 4 {
            return (None, (0.0, 1.0));
        }

        let curve_id = match Self::get_ref(&ec_ent.params[3]) {
            Some(id) => id,
            None => return (None, (0.0, 1.0)),
        };

        let curve = self.convert_curve(curve_id);

        // Determine param range
        let param_range = if let Some(ref c) = curve {
            match c {
                Curve3d::Line(_) => {
                    // For lines, the param range depends on the edge
                    if let Some((start, end)) = self.get_edge_curve_endpoints(ec_id) {
                        (0.0, start.distance_to(&end))
                    } else {
                        (0.0, 1.0)
                    }
                }
                Curve3d::Circle(_) => (0.0, 2.0 * PI),
                Curve3d::Ellipse(_) => (0.0, 2.0 * PI),
                _ => (0.0, 1.0),
            }
        } else {
            (0.0, 1.0)
        };

        (curve, param_range)
    }

    /// Parse ORIENTED_EDGE: returns (edge_curve_id, forward).
    fn parse_oriented_edge(&self, oe_ent: &StepEntity) -> (Option<i64>, bool) {
        let mut ec_id = None;
        let mut forward = true;

        for param in &oe_ent.params {
            match param {
                StepValue::Ref(id) => {
                    if let Some(e) = self.entity(*id) {
                        if e.type_name == "EDGE_CURVE" {
                            ec_id = Some(*id);
                        }
                    }
                }
                StepValue::Enum(s) => { forward = s == "T"; }
                _ => {}
            }
        }
        (ec_id, forward)
    }

    /// Get ordered boundary vertices from an EDGE_LOOP.
    fn get_loop_vertices(&self, loop_id: i64) -> Vec<Point3d> {
        let loop_ent = match self.entity(loop_id) {
            Some(e) => e,
            None => return Vec::new(),
        };

        // EDGE_LOOP(name, (#oe1, ...)) — params[0]=name, params[1]=list
        let oriented_edge_ids: Vec<i64> = match loop_ent.params.get(1) {
            Some(StepValue::List(refs)) => refs.iter().filter_map(|v| Self::get_ref(v)).collect(),
            _ => return Vec::new(),
        };

        let mut vertices = Vec::new();

        for oe_id in oriented_edge_ids {
            let oe_ent = match self.entity(oe_id) {
                Some(e) => e,
                None => continue,
            };

            let (ec_id_opt, forward) = self.parse_oriented_edge(oe_ent);

            if let Some(ec_id) = ec_id_opt {
                if let Some((start_pt, end_pt)) = self.get_edge_curve_endpoints(ec_id) {
                    if forward {
                        vertices.push(start_pt);
                    } else {
                        vertices.push(end_pt);
                    }
                }
            }
        }

        // Close the loop: if last vertex coincides with first, remove it
        if vertices.len() > 2 {
            if let (Some(first), Some(last)) = (vertices.first(), vertices.last()) {
                if first.is_coincident_with(last) {
                    vertices.pop();
                }
            }
        }

        vertices
    }

    /// Get ordered boundary edge curves from an EDGE_LOOP.
    /// Returns (curve, param_range, forward) for each edge in the loop.
    fn get_loop_curves(&self, loop_id: i64) -> Vec<(Option<Curve3d>, (f64, f64), bool)> {
        let loop_ent = match self.entity(loop_id) {
            Some(e) => e,
            None => return Vec::new(),
        };

        // EDGE_LOOP(name, (#oe1, ...)) — params[0]=name, params[1]=list
        let oriented_edge_ids: Vec<i64> = match loop_ent.params.get(1) {
            Some(StepValue::List(refs)) => refs.iter().filter_map(|v| Self::get_ref(v)).collect(),
            _ => return Vec::new(),
        };

        let mut curves = Vec::new();

        for oe_id in oriented_edge_ids {
            let oe_ent = match self.entity(oe_id) {
                Some(e) => e,
                None => continue,
            };

            let (ec_id_opt, forward) = self.parse_oriented_edge(oe_ent);

            if let Some(ec_id) = ec_id_opt {
                let (curve, param_range) = self.get_edge_curve_info(ec_id);
                curves.push((curve, param_range, forward));
            }
        }

        curves
    }

    /// Extract face bounds (outer and inner loop IDs) from an ADVANCED_FACE.
    fn get_face_bounds(&self, face_id: i64) -> (Vec<i64>, Vec<i64>) {
        let face_ent = match self.entity(face_id) {
            Some(e) => e,
            None => return (Vec::new(), Vec::new()),
        };

        let mut outer = Vec::new();
        let mut inner = Vec::new();

        if face_ent.params.is_empty() { return (outer, inner); }

        let bound_ids: Vec<i64> = match face_ent.params.get(1) {
            Some(StepValue::List(refs)) => refs.iter().filter_map(|v| Self::get_ref(v)).collect(),
            _ => return (outer, inner),
        };

        for bound_id in bound_ids {
            if let Some(bound_ent) = self.entity(bound_id) {
                if bound_ent.params.len() < 2 { continue; }
                if let Some(loop_id) = Self::get_ref(&bound_ent.params[1]) {
                    match bound_ent.type_name.as_str() {
                        "FACE_OUTER_BOUND" => outer.push(loop_id),
                        "FACE_BOUND" => inner.push(loop_id),
                        _ => {}
                    }
                }
            }
        }

        (outer, inner)
    }

    /// Get surface for an ADVANCED_FACE.
    fn get_face_surface(&mut self, face_id: i64) -> Option<Surface> {
        let face_ent = self.entity(face_id)?;
        if face_ent.params.len() < 3 { return None; }
        let surface_id = Self::get_ref(&face_ent.params[2])?;
        self.convert_surface(surface_id)
    }

    /// Get face orientation.
    fn get_face_orientation(&self, face_id: i64) -> bool {
        let face_ent = match self.entity(face_id) {
            Some(e) => e,
            None => return true,
        };
        if face_ent.params.len() >= 4 {
            match &face_ent.params[3] {
                StepValue::Enum(s) => s == "T",
                _ => true,
            }
        } else { true }
    }

    /// Find all ADVANCED_FACE IDs in the shell.
    fn find_shell_faces(&self) -> Vec<i64> {
        // Find MANIFOLD_SOLID_BREP
        let brep_ent = self.entities.values()
            .find(|e| e.type_name == "MANIFOLD_SOLID_BREP");

        let brep_ent = match brep_ent {
            Some(e) => e,
            None => return Vec::new(),
        };

        if brep_ent.params.len() < 2 { return Vec::new(); }
        let shell_id = match Self::get_ref(&brep_ent.params[1]) {
            Some(id) => id,
            None => return Vec::new(),
        };

        let shell_ent = match self.entity(shell_id) {
            Some(e) => e,
            None => return Vec::new(),
        };

        if shell_ent.params.is_empty() { return Vec::new(); }

        // CLOSED_SHELL(name, (#face1, ...)) — params[0]=name, params[1]=list
        match shell_ent.params.get(1) {
            Some(StepValue::List(refs)) => refs.iter().filter_map(|v| Self::get_ref(v)).collect(),
            _ => Vec::new(),
        }
    }

    // ── Triangulation ──────────────────────────────────────

    /// Triangulate a planar face using ear clipping.
    fn triangulate_planar_face(
        &self,
        boundary: &[Point3d],
        plane: &Plane,
        forward: bool,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        if boundary.len() < 3 {
            return mesh;
        }

        // Project 3D boundary points onto the plane's 2D coordinate system
        let points_2d: Vec<Point2d> = boundary.iter().map(|p| {
            let dx = p.x - plane.origin.x;
            let dy = p.y - plane.origin.y;
            let dz = p.z - plane.origin.z;
            Point2d::new(
                dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
                dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
            )
        }).collect();

        // Ear clipping
        let triangles = ear_clip(&points_2d);

        // Add vertices and triangles
        for p in boundary {
            mesh.add_vertex(*p);
        }
        for tri in &triangles {
            if forward {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            } else {
                mesh.add_triangle(tri[0], tri[2], tri[1]);
            }
        }

        mesh
    }

    /// Triangulate a cylindrical face using parametric tessellation.
    fn triangulate_cylindrical_face(
        &self,
        boundary: &[Point3d],
        cyl: &CylinderSurface,
        forward: bool,
        params: &TriangulationParams,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        if boundary.is_empty() {
            return mesh;
        }

        // Determine v range from boundary points projected onto the cylinder axis
        let (v_min, v_max) = compute_cylinder_v_range(boundary, cyl);

        let n_u = params.angular_samples.max(16);
        let n_v = params.height_samples.max(2);

        // Sample the cylinder surface
        for j in 0..n_v {
            for i in 0..n_u {
                let u = 2.0 * PI * i as f64 / n_u as f64;
                let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
                mesh.add_vertex(cyl.point_at(u, v));
            }
        }

        for j in 0..n_v - 1 {
            for i in 0..n_u {
                let i_next = (i + 1) % n_u;
                let v0 = (j * n_u + i) as u32;
                let v1 = (j * n_u + i_next) as u32;
                let v2 = ((j + 1) * n_u + i_next) as u32;
                let v3 = ((j + 1) * n_u + i) as u32;

                if forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }

        mesh
    }

    /// Triangulate a conical face using parametric tessellation.
    fn triangulate_conical_face(
        &self,
        boundary: &[Point3d],
        cone: &ConeSurface,
        forward: bool,
        params: &TriangulationParams,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        if boundary.is_empty() {
            return mesh;
        }

        // Determine v range from boundary points
        let (v_min, v_max) = compute_cone_v_range(boundary, cone);

        let n_u = params.angular_samples.max(16);
        let n_v = params.height_samples.max(4);

        for j in 0..n_v {
            for i in 0..n_u {
                let u = 2.0 * PI * i as f64 / n_u as f64;
                let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
                mesh.add_vertex(cone.point_at(u, v));
            }
        }

        for j in 0..n_v - 1 {
            for i in 0..n_u {
                let i_next = (i + 1) % n_u;
                let v0 = (j * n_u + i) as u32;
                let v1 = (j * n_u + i_next) as u32;
                let v2 = ((j + 1) * n_u + i_next) as u32;
                let v3 = ((j + 1) * n_u + i) as u32;

                if forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }

        mesh
    }

    /// Triangulate a planar face with an inner hole (ring-shaped).
    fn triangulate_planar_face_with_hole(
        &self,
        outer: &[Point3d],
        inner: &[Point3d],
        plane: &Plane,
        forward: bool,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        if outer.len() < 3 || inner.is_empty() {
            return self.triangulate_planar_face(outer, plane, forward);
        }

        // Simple approach: connect inner boundary to outer boundary using triangles
        // This is a basic bridge triangulation between two loops

        // Add all outer vertices
        for p in outer {
            mesh.add_vertex(*p);
        }
        // Add all inner vertices
        let inner_offset = mesh.vertices.len();
        for p in inner {
            mesh.add_vertex(*p);
        }

        let n_outer = outer.len() as u32;
        let n_inner = inner.len() as u32;

        // Find closest points on each loop to bridge
        let mut inner_start = 0u32;
        let mut min_dist = f64::MAX;
        for (i, ip) in inner.iter().enumerate() {
            let d = ip.distance_to(&outer[0]);
            if d < min_dist {
                min_dist = d;
                inner_start = i as u32;
            }
        }

        // Bridge triangulation: connect outer[i] → inner[(i*scale + inner_start) % n_inner]
        for i in 0..n_outer {
            let i_next = (i + 1) % n_outer;
            let j = ((i as f64 * n_inner as f64 / n_outer as f64) as u32 + inner_start) % n_inner;
            let j_next = ((i_next as f64 * n_inner as f64 / n_outer as f64) as u32 + inner_start) % n_inner;

            let outer_i = i;
            let outer_i_next = i_next;
            let inner_j = inner_offset as u32 + j;
            let inner_j_next = inner_offset as u32 + j_next;

            if forward {
                mesh.add_triangle(outer_i, outer_i_next, inner_j);
                mesh.add_triangle(outer_i_next, inner_j_next, inner_j);
            } else {
                mesh.add_triangle(outer_i, inner_j, outer_i_next);
                mesh.add_triangle(outer_i_next, inner_j, inner_j_next);
            }
        }

        mesh
    }

    /// Triangulate a planar disk face bounded by a circular edge.
    /// Uses fan triangulation from center.
    fn triangulate_disk_face(
        &self,
        boundary: &[Point3d],
        center: Point3d,
        normal: Direction3d,
        forward: bool,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        if boundary.is_empty() {
            return mesh;
        }

        // Add center vertex
        let center_idx = mesh.add_vertex(center);

        // Add boundary vertices
        for p in boundary {
            mesh.add_vertex(*p);
        }

        let n = boundary.len() as u32;
        for i in 0..n {
            let j = (i + 1) % n;
            if forward {
                mesh.add_triangle(center_idx, center_idx + 1 + i, center_idx + 1 + j);
            } else {
                mesh.add_triangle(center_idx, center_idx + 1 + j, center_idx + 1 + i);
            }
        }

        mesh
    }

    // ── Main conversion entry point ────────────────────────

    /// Convert STEP file to a triangle mesh.
    pub fn to_mesh(&mut self, params: &TriangulationParams) -> TriangleMesh {
        let face_ids = self.find_shell_faces();
        log::info!("STEP converter: found {} faces in shell", face_ids.len());

        if face_ids.is_empty() {
            // Debug: what entity types do we have?
            let mut type_counts = HashMap::new();
            for e in self.entities.values() {
                *type_counts.entry(e.type_name.as_str()).or_insert(0usize) += 1;
            }
            let types_str: Vec<String> = type_counts.iter()
                .map(|(k, &v)| format!("{}:{}", k, v))
                .collect();
            log::warn!("STEP converter: no faces found. Entity types: {}", types_str.join(", "));

            // Try to find faces directly
            let direct_faces: Vec<i64> = self.entities.values()
                .filter(|e| e.type_name == "ADVANCED_FACE")
                .map(|e| e.id)
                .collect();
            log::warn!("STEP converter: found {} ADVANCED_FACE entities directly", direct_faces.len());

            if !direct_faces.is_empty() {
                return self.to_mesh_from_faces(&direct_faces, params);
            }
        }

        self.to_mesh_from_faces(&face_ids, params)
    }

    /// Convert a list of ADVANCED_FACE IDs to mesh.
    fn to_mesh_from_faces(&mut self, face_ids: &[i64], params: &TriangulationParams) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        for face_id in face_ids {
            let face_mesh = self.triangulate_face(*face_id, params);
            log::info!("STEP face #{}: {} vertices, {} triangles", face_id, face_mesh.vertex_count(), face_mesh.triangle_count());
            mesh.merge(&face_mesh);
        }

        mesh
    }

    /// Triangulate a single STEP face.
    fn triangulate_face(&mut self, face_id: i64, params: &TriangulationParams) -> TriangleMesh {
        let forward = self.get_face_orientation(face_id);

        // Get boundary vertices
        let (outer_loop_ids, inner_loop_ids) = self.get_face_bounds(face_id);
        log::debug!("Face #{}: {} outer loops, {} inner loops, forward={}", face_id, outer_loop_ids.len(), inner_loop_ids.len(), forward);

        let outer_boundary = if let Some(loop_id) = outer_loop_ids.first() {
            // Get vertices from the loop
            let verts = self.get_loop_vertices(*loop_id);
            log::debug!("Face #{}: {} boundary vertices from loop #{}", face_id, verts.len(), loop_id);

            // Also try to get more detailed boundary by sampling edge curves
            if verts.len() < 3 {
                Vec::new()
            } else {
                // For curved boundaries, sample edge curves for more points
                let curves = self.get_loop_curves(*loop_id);
                let sampled = self.sample_loop_boundary(&curves, &verts);
                if sampled.len() >= verts.len() {
                    sampled
                } else {
                    verts
                }
            }
        } else {
            log::debug!("Face #{}: no outer loop found", face_id);
            Vec::new()
        };

        // Get inner boundaries (holes)
        let inner_boundaries: Vec<Vec<Point3d>> = inner_loop_ids.iter().map(|id| {
            self.get_loop_vertices(*id)
        }).collect();

        if outer_boundary.is_empty() {
            log::debug!("Face #{}: empty boundary, skipping", face_id);
            return TriangleMesh::new();
        }

        // Get surface
        let surface = self.get_face_surface(face_id);

        match surface {
            Some(Surface::Plane(plane)) => {
                if inner_boundaries.is_empty() {
                    self.triangulate_planar_face(&outer_boundary, &plane, forward)
                } else if let Some(inner) = inner_boundaries.first() {
                    // Check if the face is a disk (circular boundary with a hole)
                    // For now, use bridge triangulation
                    self.triangulate_planar_face_with_hole(
                        &outer_boundary, inner, &plane, forward
                    )
                } else {
                    self.triangulate_planar_face(&outer_boundary, &plane, forward)
                }
            }
            Some(Surface::Cylinder(cyl)) => {
                self.triangulate_cylindrical_face(&outer_boundary, &cyl, forward, params)
            }
            Some(Surface::Cone(cone)) => {
                self.triangulate_conical_face(&outer_boundary, &cone, forward, params)
            }
            Some(Surface::Sphere(sphere)) => {
                // For now, use generic sphere tessellation
                self.triangulate_sphere_face_simple(&sphere, forward, params)
            }
            Some(Surface::Torus(torus)) => {
                self.triangulate_torus_face_simple(&torus, forward, params)
            }
            None => {
                // Fallback: try to triangulate as a planar face using boundary points
                if outer_boundary.len() >= 3 {
                    let plane = Plane::from_three_points(
                        &outer_boundary[0],
                        &outer_boundary[1],
                        &outer_boundary[2],
                    ).unwrap_or_else(|| Plane::from_origin_and_normal(outer_boundary[0], Direction3d::Z));
                    self.triangulate_planar_face(&outer_boundary, &plane, forward)
                } else {
                    TriangleMesh::new()
                }
            }
            _ => {
                // For other surface types, fall back to planar triangulation
                if outer_boundary.len() >= 3 {
                    let plane = Plane::from_three_points(
                        &outer_boundary[0],
                        &outer_boundary[1],
                        &outer_boundary[2],
                    ).unwrap_or_else(|| Plane::from_origin_and_normal(outer_boundary[0], Direction3d::Z));
                    self.triangulate_planar_face(&outer_boundary, &plane, forward)
                } else {
                    TriangleMesh::new()
                }
            }
        }
    }

    /// Sample edge curves in a loop to get more boundary points (for curved edges).
    fn sample_loop_boundary(
        &self,
        curves: &[(Option<Curve3d>, (f64, f64), bool)],
        vertices: &[Point3d],
    ) -> Vec<Point3d> {
        let mut points = Vec::new();

        for (i, (curve_opt, (t_min, t_max), forward)) in curves.iter().enumerate() {
            if let Some(curve) = curve_opt {
                match curve {
                    Curve3d::Line(_) => {
                        // Just use the start vertex
                        if i < vertices.len() {
                            points.push(vertices[i]);
                        }
                    }
                    Curve3d::Circle(_) => {
                        // Sample the circle arc between start and end vertices
                        let n_samples = 32;
                        let start = if i < vertices.len() { vertices[i] } else { continue };
                        let end = if i + 1 < vertices.len() { vertices[i + 1] } else { vertices[0] };

                        // Determine if this is a full circle or an arc
                        let is_full_circle = start.is_coincident_with(&end);
                        let (arc_start, arc_end) = if is_full_circle {
                            (0.0, 2.0 * PI)
                        } else {
                            // Find angles for start and end points on the circle
                            let angles = find_angles_on_circle(curve, &start, &end);
                            (angles.0, angles.1)
                        };

                        for j in 0..n_samples {
                            let t = if is_full_circle {
                                2.0 * PI * j as f64 / n_samples as f64
                            } else {
                                arc_start + (arc_end - arc_start) * j as f64 / n_samples as f64
                            };
                            let p = curve.point_at(t);
                            if *forward {
                                points.push(p);
                            } else {
                                // For reversed edges, sample in reverse order
                                points.insert(points.len().saturating_sub(0), p);
                            }
                        }
                    }
                    _ => {
                        // For other curves, just use the vertex
                        if i < vertices.len() {
                            points.push(vertices[i]);
                        }
                    }
                }
            } else {
                // No curve geometry — just use the vertex
                if i < vertices.len() {
                    points.push(vertices[i]);
                }
            }
        }

        // Remove duplicate consecutive points
        if points.len() > 1 {
            let mut deduped = vec![points[0]];
            for p in &points[1..] {
                if !p.is_coincident_with(deduped.last().unwrap()) {
                    deduped.push(*p);
                }
            }
            // Check wrap-around
            if deduped.len() > 1 && deduped.first().unwrap().is_coincident_with(deduped.last().unwrap()) {
                deduped.pop();
            }
            points = deduped;
        }

        points
    }

    /// Simple sphere face tessellation.
    fn triangulate_sphere_face_simple(
        &self,
        sphere: &draper_geometry::SphereSurface,
        forward: bool,
        params: &TriangulationParams,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let n_u = params.angular_samples;
        let n_v = params.angular_samples / 2;

        for j in 0..=n_v {
            for i in 0..n_u {
                let u = 2.0 * PI * i as f64 / n_u as f64;
                let v = PI * j as f64 / n_v as f64;
                mesh.add_vertex(sphere.point_at(u, v));
            }
        }

        for j in 0..n_v {
            for i in 0..n_u {
                let i_next = (i + 1) % n_u;
                let v0 = (j * n_u + i) as u32;
                let v1 = (j * n_u + i_next) as u32;
                let v2 = ((j + 1) * n_u + i_next) as u32;
                let v3 = ((j + 1) * n_u + i) as u32;

                if j == 0 {
                    if forward { mesh.add_triangle(v0, v2, v3); }
                    else { mesh.add_triangle(v0, v3, v2); }
                } else if j == n_v - 1 {
                    if forward { mesh.add_triangle(v0, v1, v2); }
                    else { mesh.add_triangle(v0, v2, v1); }
                } else {
                    if forward {
                        mesh.add_triangle(v0, v1, v2);
                        mesh.add_triangle(v0, v2, v3);
                    } else {
                        mesh.add_triangle(v0, v2, v1);
                        mesh.add_triangle(v0, v3, v2);
                    }
                }
            }
        }

        mesh
    }

    /// Simple torus face tessellation.
    fn triangulate_torus_face_simple(
        &self,
        torus: &draper_geometry::TorusSurface,
        forward: bool,
        params: &TriangulationParams,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let n_u = params.angular_samples;
        let n_v = params.angular_samples;

        for j in 0..n_v {
            for i in 0..n_u {
                let u = 2.0 * PI * i as f64 / n_u as f64;
                let v = 2.0 * PI * j as f64 / n_v as f64;
                mesh.add_vertex(torus.point_at(u, v));
            }
        }

        for j in 0..n_v {
            for i in 0..n_u {
                let i_next = (i + 1) % n_u;
                let j_next = (j + 1) % n_v;
                let v0 = (j * n_u + i) as u32;
                let v1 = (j * n_u + i_next) as u32;
                let v2 = (j_next * n_u + i_next) as u32;
                let v3 = (j_next * n_u + i) as u32;

                if forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }

        mesh
    }

    /// Get diagnostics about the STEP file.
    pub fn diagnostics(&self) -> StepDiagnostics {
        let mut point_count = 0;
        let mut face_count = 0;
        let mut shell_count = 0;
        let mut brep_count = 0;

        for e in self.entities.values() {
            match e.type_name.as_str() {
                "CARTESIAN_POINT" => point_count += 1,
                "ADVANCED_FACE" | "FACE_OUTER_BOUND" | "FACE_BOUND" => face_count += 1,
                "CLOSED_SHELL" | "OPEN_SHELL" => shell_count += 1,
                "MANIFOLD_SOLID_BREP" => brep_count += 1,
                _ => {}
            }
        }

        StepDiagnostics {
            entity_count: self.entities.len(),
            point_count,
            face_count,
            shell_count,
            brep_count,
            surface_types: self.surface_types.clone(),
            vertex_count: 0,
            triangle_count: 0,
        }
    }

    /// Get surface types found during conversion.
    pub fn surface_types(&self) -> &[String] {
        &self.surface_types
    }
}

// ── Helper functions ───────────────────────────────────────

/// Compute the v (height) range for a cylinder from boundary points.
fn compute_cylinder_v_range(boundary: &[Point3d], cyl: &CylinderSurface) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary {
        // Project point onto cylinder axis
        let dx = p.x - cyl.origin.x;
        let dy = p.y - cyl.origin.y;
        let dz = p.z - cyl.origin.z;
        let v = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    if v_min == f64::MAX {
        (0.0, 1.0)
    } else {
        (v_min, v_max)
    }
}

/// Compute the v range for a cone from boundary points.
fn compute_cone_v_range(boundary: &[Point3d], cone: &ConeSurface) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary {
        // Project point onto cone axis from apex
        let dx = p.x - cone.apex.x;
        let dy = p.y - cone.apex.y;
        let dz = p.z - cone.apex.z;
        let v = dx * cone.axis.x + dy * cone.axis.y + dz * cone.axis.z;
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    if v_min == f64::MAX {
        (0.0, 1.0)
    } else {
        (v_min, v_max)
    }
}

/// Find the start and end angles for two points on a circle.
fn find_angles_on_circle(curve: &Curve3d, start: &Point3d, end: &Point3d) -> (f64, f64) {
    if let Curve3d::Circle(circle) = curve {
        // Compute angle of start point relative to circle center
        let dx_s = start.x - circle.center.x;
        let dy_s = start.y - circle.center.y;
        let dz_s = start.z - circle.center.z;
        let y_axis = circle.normal.cross(&circle.x_axis);
        let start_angle = (dx_s * circle.x_axis.x + dy_s * circle.x_axis.y + dz_s * circle.x_axis.z)
            .atan2(dx_s * y_axis.x + dy_s * y_axis.y + dz_s * y_axis.z);

        let dx_e = end.x - circle.center.x;
        let dy_e = end.y - circle.center.y;
        let dz_e = end.z - circle.center.z;
        let end_angle = (dx_e * circle.x_axis.x + dy_e * circle.x_axis.y + dz_e * circle.x_axis.z)
            .atan2(dx_e * y_axis.x + dy_e * y_axis.y + dz_e * y_axis.z);

        // Ensure end > start for forward direction
        let mut end_angle = end_angle;
        while end_angle <= start_angle {
            end_angle += 2.0 * PI;
        }

        (start_angle, end_angle)
    } else {
        (0.0, 2.0 * PI)
    }
}

/// Ear clipping triangulation of a 2D polygon.
fn ear_clip(points: &[Point2d]) -> Vec<[u32; 3]> {
    let n = points.len();
    if n < 3 { return vec![]; }
    if n == 3 { return vec![[0, 1, 2]]; }

    // Determine winding order
    let mut signed_area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        signed_area += points[i].u * points[j].v - points[j].u * points[i].v;
    }
    let ccw = signed_area > 0.0;

    let mut indices: Vec<u32> = (0..n as u32).collect();
    let mut triangles = Vec::new();

    let max_attempts = n * n;
    let mut attempts = 0;

    while indices.len() > 3 && attempts < max_attempts {
        attempts += 1;
        let len = indices.len();
        let mut found_ear = false;

        for i in 0..len {
            let i_prev = if i == 0 { len - 1 } else { i - 1 };
            let i_next = (i + 1) % len;

            let a = indices[i_prev];
            let b = indices[i];
            let c = indices[i_next];

            let pa = &points[a as usize];
            let pb = &points[b as usize];
            let pc = &points[c as usize];

            let cross = (pb.u - pa.u) * (pc.v - pa.v) - (pb.v - pa.v) * (pc.u - pa.u);
            let is_convex = if ccw { cross > 0.0 } else { cross < 0.0 };

            if !is_convex { continue; }

            let mut is_ear = true;
            for j in 0..len {
                if j == i_prev || j == i || j == i_next { continue; }
                let p = &points[indices[j] as usize];
                if point_in_triangle(pa, pb, pc, p) {
                    is_ear = false;
                    break;
                }
            }

            if is_ear {
                triangles.push([a, b, c]);
                indices.remove(i);
                found_ear = true;
                break;
            }
        }

        if !found_ear {
            // Degenerate — fan triangulate
            for i in 1..indices.len() - 1 {
                triangles.push([indices[0], indices[i], indices[i + 1]]);
            }
            break;
        }
    }

    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }

    triangles
}

/// Check if a 2D point is inside a triangle.
fn point_in_triangle(a: &Point2d, b: &Point2d, c: &Point2d, p: &Point2d) -> bool {
    let d1 = sign_area_2d(p, a, b);
    let d2 = sign_area_2d(p, b, c);
    let d3 = sign_area_2d(p, c, a);
    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
    !(has_neg && has_pos)
}

fn sign_area_2d(p: &Point2d, a: &Point2d, b: &Point2d) -> f64 {
    (p.u - b.u) * (a.v - b.v) - (a.u - b.u) * (p.v - b.v)
}

/// High-level function: parse a STEP file and convert it to a triangle mesh.
pub fn step_file_to_mesh(path: &str, params: &TriangulationParams) -> Result<(TriangleMesh, StepDiagnostics), String> {
    let step_file = crate::parse_step_file(path)
        .map_err(|e| format!("STEP parse error: {}", e))?;

    let mut converter = StepConverter::new(&step_file);
    let mut diag = converter.diagnostics();

    let mesh = converter.to_mesh(params);

    diag.vertex_count = mesh.vertex_count();
    diag.triangle_count = mesh.triangle_count();
    diag.surface_types = converter.surface_types().to_vec();

    Ok((mesh, diag))
}
