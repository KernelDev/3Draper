//! STEP-to-mesh converter.
//!
//! Converts parsed STEP entities into a triangle mesh by:
//! 1. Resolving entity references to build geometry
//! 2. Creating B-Rep faces from STEP surface entities
//! 3. Triangulating using the existing mesh pipeline
//!
//! Supported surface types:
//! - PLANE
//! - CYLINDRICAL_SURFACE
//! - SPHERICAL_SURFACE
//! - CONICAL_SURFACE
//! - TOROIDAL_SURFACE
//! - SURFACE_OF_REVOLUTION
//! - SURFACE_OF_LINEAR_EXTRUSION
//! - B_SPLINE_SURFACE_WITH_KNOTS / B_SPLINE_SURFACE / BEZIER_SURFACE
//!
//! Full B-Rep topology reconstruction with trimming is not yet implemented —
//! surfaces are rendered with extents estimated from the model bounding box.

use crate::schema::{StepFile, StepValue};
use draper_geometry::{
    Point3d, Direction3d, Surface, Plane, CylinderSurface, SphereSurface,
    ConeSurface, TorusSurface, RevolutionSurface, ExtrusionSurface,
    NurbsSurface, Curve3d, Line, Circle, NurbsCurve,
};
use draper_mesh::{TriangleMesh, TriangulationParams, triangulate_solid, triangulate_face};
use draper_topology::{Solid, Shell, Face, Wire};
use std::collections::HashMap;
use std::f64::consts::PI;

/// Convert a parsed STEP file to a triangle mesh.
pub fn step_to_mesh(step_file: &StepFile) -> Result<TriangleMesh, String> {
    let converter = StepConverter::new(step_file);
    converter.convert()
}

struct StepConverter<'a> {
    step: &'a StepFile,
    entity_map: HashMap<i64, usize>,
}

impl<'a> StepConverter<'a> {
    fn new(step: &'a StepFile) -> Self {
        let entity_map: HashMap<i64, usize> = step.entities.iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect();
        Self { step, entity_map }
    }

    fn convert(&self) -> Result<TriangleMesh, String> {
        let mut mesh = TriangleMesh::new();
        let params = TriangulationParams::default();

        // Compute a model bounding box from all CARTESIAN_POINT entities
        // This is used to estimate surface extents for untrimmed surfaces.
        let bbox = self.compute_bounding_box();

        // Strategy 1: Find MANIFOLD_SOLID_BREP → CLOSED_SHELL → ADVANCED_FACE → surface
        let breps = self.step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        let mut faces_converted = 0;
        let mut unsupported_types: Vec<String> = Vec::new();

        if !breps.is_empty() {
            for brep in &breps {
                // BREP typically: #N = MANIFOLD_SOLID_BREP('name', #shell_ref);
                // The shell reference is the 2nd parameter (index 1)
                let shell_id = self.find_shell_ref(brep);
                if let Some(shell_id) = shell_id {
                    if let Some(surfaces) = self.extract_shell_faces(shell_id) {
                        for surface in surfaces {
                            let face_mesh = self.surface_to_mesh(&surface, &params, &bbox);
                            mesh.merge(&face_mesh);
                            faces_converted += 1;
                        }
                    }
                }
            }
        }

        // Strategy 2: Try FACETED_BREP (some STEP files use this)
        if faces_converted == 0 {
            let faceted = self.step.find_entities_by_type("FACETED_BREP");
            for fb in &faceted {
                let shell_id = self.find_shell_ref(fb);
                if let Some(shell_id) = shell_id {
                    if let Some(surfaces) = self.extract_shell_faces(shell_id) {
                        for surface in surfaces {
                            let face_mesh = self.surface_to_mesh(&surface, &params, &bbox);
                            mesh.merge(&face_mesh);
                            faces_converted += 1;
                        }
                    }
                }
            }
        }

        // Strategy 3: Try ADVANCED_BREP_SHAPE_REPRESENTATION
        if faces_converted == 0 {
            let abrep = self.step.find_entities_by_type("ADVANCED_BREP_SHAPE_REPRESENTATION");
            for ab in &abrep {
                // Find MANIFOLD_SOLID_BREP referenced from this
                for param in &ab.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name == "MANIFOLD_SOLID_BREP" {
                                let shell_id = self.find_shell_ref(entity);
                                if let Some(shell_id) = shell_id {
                                    if let Some(surfaces) = self.extract_shell_faces(shell_id) {
                                        for surface in surfaces {
                                            let face_mesh = self.surface_to_mesh(&surface, &params, &bbox);
                                            mesh.merge(&face_mesh);
                                            faces_converted += 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    // Also check inside lists
                    if let StepValue::List(items) = param {
                        for item in items {
                            if let Some(ref_id) = self.get_ref(item) {
                                if let Some(entity) = self.step.find_entity(ref_id) {
                                    if entity.type_name == "MANIFOLD_SOLID_BREP" {
                                        let shell_id = self.find_shell_ref(entity);
                                        if let Some(shell_id) = shell_id {
                                            if let Some(surfaces) = self.extract_shell_faces(shell_id) {
                                                for surface in surfaces {
                                                    let face_mesh = self.surface_to_mesh(&surface, &params, &bbox);
                                                    mesh.merge(&face_mesh);
                                                    faces_converted += 1;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Strategy 4: If no BREP structure found, try to extract surfaces directly
        if faces_converted == 0 {
            let surface_types = [
                "PLANE",
                "CYLINDRICAL_SURFACE",
                "SPHERICAL_SURFACE",
                "CONICAL_SURFACE",
                "TOROIDAL_SURFACE",
                "SURFACE_OF_REVOLUTION",
                "SURFACE_OF_LINEAR_EXTRUSION",
                "B_SPLINE_SURFACE_WITH_KNOTS",
                "B_SPLINE_SURFACE",
                "BEZIER_SURFACE",
            ];

            for type_name in &surface_types {
                for entity in self.step.find_entities_by_type(type_name) {
                    match self.extract_surface(entity.id) {
                        Some(surface) => {
                            let face_mesh = self.surface_to_mesh(&surface, &params, &bbox);
                            mesh.merge(&face_mesh);
                            faces_converted += 1;
                        }
                        None => {
                            unsupported_types.push(type_name.to_string());
                        }
                    }
                }
            }
        }

        // Strategy 5: Last resort — try to create a mesh from all point/vertex data
        if faces_converted == 0 {
            // Collect all CARTESIAN_POINT entities and create a point cloud mesh
            let points: Vec<Point3d> = self.step.find_entities_by_type("CARTESIAN_POINT")
                .iter()
                .filter_map(|e| self.resolve_cartesian_point(e.id))
                .collect();

            if points.len() >= 3 {
                // Create a convex hull approximation — just fan-triangulate the points
                // This is a rough approximation but better than nothing
                let mut pt_mesh = TriangleMesh::new();
                for p in &points {
                    pt_mesh.add_vertex(*p);
                }
                // Simple fan triangulation from first point
                for i in 1..points.len().saturating_sub(1) {
                    pt_mesh.add_triangle(0, i as u32, (i + 1) as u32);
                }
                if pt_mesh.triangle_count() > 0 {
                    mesh.merge(&pt_mesh);
                    faces_converted += pt_mesh.triangle_count() as usize;
                }
            }
        }

        if faces_converted == 0 {
            // List what entity types exist so the user can report them
            let type_summary: Vec<String> = {
                let mut types: HashMap<String, usize> = HashMap::new();
                for e in &self.step.entities {
                    *types.entry(e.type_name.clone()).or_insert(0) += 1;
                }
                let mut v: Vec<_> = types.into_iter().collect();
                v.sort_by(|a, b| b.1.cmp(&a.1));
                v.iter().take(15).map(|(t, c)| format!("{}({})", t, c)).collect()
            };
            return Err(format!(
                "No convertible surface geometry found in STEP file. Top entity types: {}",
                type_summary.join(", ")
            ));
        }

        Ok(mesh)
    }

    /// Find the shell reference from a BREP entity.
    fn find_shell_ref(&self, brep: &crate::schema::StepEntity) -> Option<i64> {
        // MANIFOLD_SOLID_BREP('name', #shell_ref) — shell ref is usually 2nd param
        // But some files have it as the first Ref parameter
        for param in &brep.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name.contains("SHELL") {
                        return Some(ref_id);
                    }
                }
            }
        }
        // If not found by type name, try the second parameter
        if let Some(param) = brep.params.get(1) {
            if let Some(ref_id) = self.get_ref(param) {
                return Some(ref_id);
            }
        }
        None
    }

    /// Compute a bounding box from all CARTESIAN_POINT entities.
    fn compute_bounding_box(&self) -> Option<(Point3d, Point3d)> {
        let points: Vec<Point3d> = self.step.find_entities_by_type("CARTESIAN_POINT")
            .iter()
            .filter_map(|e| self.resolve_cartesian_point(e.id))
            .collect();

        if points.is_empty() {
            return None;
        }

        let mut min = points[0];
        let mut max = points[0];
        for p in &points[1..] {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }

        // Expand the box slightly
        let margin = 0.001;
        min.x -= margin; min.y -= margin; min.z -= margin;
        max.x += margin; max.y += margin; max.z += margin;

        Some((min, max))
    }

    /// Extract surfaces from a CLOSED_SHELL or OPEN_SHELL entity.
    fn extract_shell_faces(&self, shell_id: i64) -> Option<Vec<Surface>> {
        let shell = self.step.find_entity(shell_id)?;
        let mut surfaces = Vec::new();

        // CLOSED_SHELL('', (#face1, #face2, ...))
        for param in &shell.params {
            match param {
                StepValue::List(items) => {
                    for item in items {
                        if let Some(face_id) = self.get_ref(item) {
                            if let Some(surface) = self.extract_face_surface(face_id) {
                                surfaces.push(surface);
                            }
                        }
                    }
                }
                StepValue::Ref(face_id) => {
                    if let Some(surface) = self.extract_face_surface(*face_id) {
                        surfaces.push(surface);
                    }
                }
                _ => {}
            }
        }

        if surfaces.is_empty() { None } else { Some(surfaces) }
    }

    /// Extract the surface geometry from an ADVANCED_FACE or FACE_SURFACE entity.
    fn extract_face_surface(&self, face_id: i64) -> Option<Surface> {
        let face = self.step.find_entity(face_id)?;

        match face.type_name.as_str() {
            "ADVANCED_FACE" | "FACE_SURFACE" => {
                // Format: #N = ADVANCED_FACE('', (bounds), #surface_ref, .T.);
                // The surface reference is typically the 3rd parameter (index 2).
                // But bounds can be complex (lists of lists), so we need to be smart.

                // Try parameter index 2 first (the typical position for surface ref)
                if let Some(param) = face.params.get(2) {
                    if let Some(surface_id) = self.get_ref(param) {
                        if let Some(surface) = self.extract_surface(surface_id) {
                            return Some(surface);
                        }
                    }
                }

                // If index 2 didn't work, scan all params for the surface ref
                // Skip the first param (usually a string name)
                for (i, param) in face.params.iter().enumerate() {
                    if i == 0 { continue; } // Skip name
                    if let Some(surface_id) = self.get_ref(param) {
                        // Check if this ref points to a surface entity (not a bound)
                        if let Some(entity) = self.step.find_entity(surface_id) {
                            let is_surface = matches!(
                                entity.type_name.as_str(),
                                "PLANE" | "CYLINDRICAL_SURFACE" | "SPHERICAL_SURFACE" |
                                "CONICAL_SURFACE" | "TOROIDAL_SURFACE" |
                                "SURFACE_OF_REVOLUTION" | "SURFACE_OF_LINEAR_EXTRUSION" |
                                "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" |
                                "BEZIER_SURFACE" | "RECTANGULAR_TRIMMED_SURFACE" |
                                "OFFSET_SURFACE" | "SWEPT_SURFACE"
                            );
                            if is_surface {
                                if let Some(surface) = self.extract_surface(surface_id) {
                                    return Some(surface);
                                }
                            }
                        }
                    }
                }
            }
            _ => {
                // Try to extract directly as a surface
                if let Some(surface) = self.extract_surface(face_id) {
                    return Some(surface);
                }
            }
        }

        None
    }

    /// Extract a Surface from a STEP surface entity.
    fn extract_surface(&self, surface_id: i64) -> Option<Surface> {
        let entity = self.step.find_entity(surface_id)?;
        match entity.type_name.as_str() {
            "PLANE" => self.extract_plane(entity),
            "CYLINDRICAL_SURFACE" => self.extract_cylinder(entity),
            "SPHERICAL_SURFACE" => self.extract_sphere(entity),
            "CONICAL_SURFACE" => self.extract_cone(entity),
            "TOROIDAL_SURFACE" => self.extract_torus(entity),
            "SURFACE_OF_REVOLUTION" => self.extract_revolution(entity),
            "SURFACE_OF_LINEAR_EXTRUSION" => self.extract_extrusion(entity),
            "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" | "BEZIER_SURFACE" => {
                self.extract_bspline_surface(entity)
            }
            "RECTANGULAR_TRIMMED_SURFACE" => self.extract_trimmed_surface(entity),
            "SWEPT_SURFACE" => self.extract_swept_surface(entity),
            _ => None,
        }
    }

    /// Extract a PLANE surface.
    fn extract_plane(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (origin, normal, u_dir) = self.resolve_axis2(axis2_id)?;
        let v_dir = normal.cross(&u_dir);
        Some(Surface::Plane(Plane { origin, u_dir, v_dir, normal }))
    }

    /// Extract a CYLINDRICAL_SURFACE.
    fn extract_cylinder(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (origin, axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.get_float(entity.params.get(1)?)?;
        Some(Surface::Cylinder(CylinderSurface::new(origin, axis, radius)))
    }

    /// Extract a SPHERICAL_SURFACE.
    fn extract_sphere(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (center, _axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.get_float(entity.params.get(1)?)?;
        Some(Surface::Sphere(SphereSurface::new(center, radius)))
    }

    /// Extract a CONICAL_SURFACE.
    fn extract_cone(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (origin, axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.get_float(entity.params.get(1)?)?;
        let half_angle = self.get_float(entity.params.get(2)?)?;
        Some(Surface::Cone(ConeSurface {
            origin,
            axis,
            half_angle: half_angle.abs(),
            radius,
        }))
    }

    /// Extract a TOROIDAL_SURFACE.
    fn extract_torus(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (center, axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let major_radius = self.get_float(entity.params.get(1)?)?;
        let minor_radius = self.get_float(entity.params.get(2)?)?;
        Some(Surface::Torus(TorusSurface {
            center,
            axis,
            major_radius,
            minor_radius,
        }))
    }

    /// Extract a SURFACE_OF_REVOLUTION.
    /// Format: #N = SURFACE_OF_REVOLUTION('', #profile_curve, #axis2_placement);
    fn extract_revolution(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Find the profile curve (2nd param, index 1)
        let profile_id = self.find_curve_ref(entity, 1)?;
        let profile = self.resolve_curve(profile_id)?;

        // Find the axis placement (3rd param, index 2)
        let axis2_id = self.find_param_ref(entity, 2)?;
        let (origin, axis, _u_dir) = self.resolve_axis2(axis2_id)?;

        Some(Surface::Revolution(RevolutionSurface {
            profile,
            axis,
            origin,
        }))
    }

    /// Extract a SURFACE_OF_LINEAR_EXTRUSION.
    /// Format: #N = SURFACE_OF_LINEAR_EXTRUSION('', #profile_curve, #direction);
    fn extract_extrusion(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Find the profile curve
        let profile_id = self.find_curve_ref(entity, 1)?;
        let profile = self.resolve_curve(profile_id)?;

        // Find the extrusion direction (3rd param, index 2)
        let dir_id = self.find_param_ref(entity, 2)?;
        let direction = self.resolve_direction(dir_id)?;

        Some(Surface::Extrusion(ExtrusionSurface {
            profile,
            direction,
        }))
    }

    /// Extract a B_SPLINE_SURFACE_WITH_KNOTS.
    /// Format: #N = B_SPLINE_SURFACE_WITH_KNOTS(degree_u, degree_v,
    ///   ((cp_list_row1), (cp_list_row2), ...), .UNSPECIFIED., .F., .F., .F.,
    ///   knot_count_u, knot_count_v, (knots_u), (knots_v), .UNSPECIFIED.);
    fn extract_bspline_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Degree u and v
        let u_degree = self.get_float(entity.params.first()?).unwrap_or(1.0) as usize;
        let v_degree = self.get_float(entity.params.get(1)?).unwrap_or(1.0) as usize;

        // Control points: 3rd param is a list of lists
        let mut control_points: Vec<Vec<Point3d>> = Vec::new();
        if let Some(StepValue::List(rows)) = entity.params.get(2) {
            for row in rows {
                if let StepValue::List(cols) = row {
                    let mut row_pts = Vec::new();
                    for col in cols {
                        if let StepValue::List(coords) = col {
                            // [x, y, z]
                            let x = coords.get(0).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                            let y = coords.get(1).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                            let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                            row_pts.push(Point3d::new(x, y, z));
                        } else if let Some(ref_id) = self.get_ref(col) {
                            // Reference to CARTESIAN_POINT
                            if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                                row_pts.push(pt);
                            }
                        }
                    }
                    if !row_pts.is_empty() {
                        control_points.push(row_pts);
                    }
                }
            }
        }

        if control_points.is_empty() {
            return None;
        }

        // Weights: default to 1.0 for each control point (rational = false)
        let n_u = control_points.len();
        let n_v = control_points[0].len();
        let weights = vec![vec![1.0; n_v]; n_u];

        // Find knot vectors — search remaining params for lists that could be knot vectors
        // The knot vectors are typically the 10th and 11th parameters for B_SPLINE_SURFACE_WITH_KNOTS
        // but for simpler B_SPLINE_SURFACE, they may be omitted
        let (u_knots, v_knots) = self.extract_bspline_knots(entity, n_u, n_v, u_degree, v_degree);

        Some(Surface::Nurbs(NurbsSurface {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        }))
    }

    /// Extract knot vectors from a B_SPLINE_SURFACE_WITH_KNOTS entity.
    fn extract_bspline_knots(
        &self,
        entity: &crate::schema::StepEntity,
        n_u: usize,
        n_v: usize,
        u_degree: usize,
        v_degree: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        // For B_SPLINE_SURFACE_WITH_KNOTS, the knot vectors are typically at
        // params[9] and params[10] (0-indexed), or params[10] and params[11]
        // depending on the exact format.
        // We'll search for them by looking for lists of numbers in the later params.

        let mut knot_lists: Vec<Vec<f64>> = Vec::new();
        for param in entity.params.iter().skip(3) {
            if let StepValue::List(items) = param {
                // Check if this looks like a knot vector (list of floats)
                let floats: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if floats.len() >= 2 && floats.iter().all(|f| f.is_finite()) {
                    knot_lists.push(floats);
                }
            }
        }

        let u_knots = if knot_lists.len() >= 2 {
            // The two longest lists are likely the knot vectors
            knot_lists.sort_by(|a, b| b.len().cmp(&a.len()));
            let expected_u_knots = n_u + u_degree + 1;
            let expected_v_knots = n_v + v_degree + 1;
            // Try to match by expected length
            if let Some(k) = knot_lists.iter().find(|l| l.len() == expected_u_knots) {
                k.clone()
            } else {
                knot_lists[0].clone()
            }
        } else {
            // Generate uniform knot vector
            let n = n_u + u_degree + 1;
            (0..n).map(|i| i as f64 / (n - 1).max(1) as f64).collect()
        };

        let v_knots = if knot_lists.len() >= 2 {
            let expected_v_knots = n_v + v_degree + 1;
            if let Some(k) = knot_lists.iter().find(|l| l.len() == expected_v_knots) {
                k.clone()
            } else if knot_lists.len() >= 2 {
                knot_lists[1].clone()
            } else {
                let n = n_v + v_degree + 1;
                (0..n).map(|i| i as f64 / (n - 1).max(1) as f64).collect()
            }
        } else {
            let n = n_v + v_degree + 1;
            (0..n).map(|i| i as f64 / (n - 1).max(1) as f64).collect()
        };

        (u_knots, v_knots)
    }

    /// Extract a RECTANGULAR_TRIMMED_SURFACE (wrapper around another surface).
    fn extract_trimmed_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // RECTANGULAR_TRIMMED_SURFACE(#basis_surface, u1, u2, v1, v2, .T., .T.)
        let basis_id = self.get_ref(entity.params.first()?)?;
        self.extract_surface(basis_id)
    }

    /// Extract a SWEPT_SURFACE (surface created by sweeping a curve).
    fn extract_swept_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // SWEPT_SURFACE('', #profile_curve, #swept_curve_or_direction)
        // This is a base type for SURFACE_OF_REVOLUTION and SURFACE_OF_LINEAR_EXTRUSION
        // Try to extract the profile curve and create a revolution or extrusion
        let profile_id = self.find_curve_ref(entity, 1)?;

        // Look for the 3rd param — could be a direction or axis placement
        if let Some(param) = entity.params.get(2) {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(dir_entity) = self.step.find_entity(ref_id) {
                    if dir_entity.type_name == "DIRECTION" {
                        // It's a linear extrusion
                        let profile = self.resolve_curve(profile_id)?;
                        let direction = self.resolve_direction(ref_id)?;
                        return Some(Surface::Extrusion(ExtrusionSurface {
                            profile,
                            direction,
                        }));
                    } else if dir_entity.type_name.contains("AXIS2_PLACEMENT") {
                        // It's a revolution
                        let profile = self.resolve_curve(profile_id)?;
                        let (origin, axis, _u_dir) = self.resolve_axis2(ref_id)?;
                        return Some(Surface::Revolution(RevolutionSurface {
                            profile,
                            axis,
                            origin,
                        }));
                    }
                }
            }
        }
        None
    }

    /// Find a curve reference from an entity's parameters.
    /// Handles both direct references and indirect references through
    /// entities like DEFINITIONAL_REPRESENTATION, GEOMETRIC_REPRESENTATION_ITEM, etc.
    fn find_curve_ref(&self, entity: &crate::schema::StepEntity, param_index: usize) -> Option<i64> {
        if let Some(param) = entity.params.get(param_index) {
            if let Some(ref_id) = self.get_ref(param) {
                // Direct reference — check if it's a curve entity
                if let Some(curve_entity) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&curve_entity.type_name) {
                        return Some(ref_id);
                    }
                    // Indirect reference — try to find a curve within this entity
                    return self.find_nested_curve(curve_entity);
                }
            }
        }
        None
    }

    /// Find a curve reference nested inside an entity (e.g., through
    /// DEFINITIONAL_REPRESENTATION, GEOMETRIC_REPRESENTATION_ITEM, etc.)
    fn find_nested_curve(&self, entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(nested) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&nested.type_name) {
                        return Some(ref_id);
                    }
                    // Go one level deeper
                    let deeper = self.find_nested_curve(nested);
                    if deeper.is_some() {
                        return deeper;
                    }
                }
            }
            // Also check lists
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(nested) = self.step.find_entity(ref_id) {
                            if self.is_curve_type(&nested.type_name) {
                                return Some(ref_id);
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Check if a type name is a known curve type.
    fn is_curve_type(&self, type_name: &str) -> bool {
        matches!(
            type_name,
            "LINE" | "CIRCLE" | "ELLIPSE" | "B_SPLINE_CURVE_WITH_KNOTS" |
            "B_SPLINE_CURVE" | "BEZIER_CURVE" | "POLYLINE" | "TRIMMED_CURVE" |
            "COMPOSITE_CURVE" | "COMPOSITE_CURVE_SEGMENT" | "OFFSET_CURVE_3D" |
            "HYPERBOLA" | "PARABOLA" | "RATIONAL_B_SPLINE_CURVE"
        )
    }

    /// Find a reference in the entity's parameters at a specific index.
    fn find_param_ref(&self, entity: &crate::schema::StepEntity, index: usize) -> Option<i64> {
        if let Some(param) = entity.params.get(index) {
            self.get_ref(param)
        } else {
            None
        }
    }

    /// Resolve a STEP curve entity to a Curve3d.
    fn resolve_curve(&self, curve_id: i64) -> Option<Curve3d> {
        let entity = self.step.find_entity(curve_id)?;
        match entity.type_name.as_str() {
            "LINE" => self.resolve_line_curve(entity),
            "CIRCLE" => self.resolve_circle_curve(entity),
            "ELLIPSE" => self.resolve_ellipse_curve(entity),
            "B_SPLINE_CURVE_WITH_KNOTS" | "B_SPLINE_CURVE" | "BEZIER_CURVE" |
            "RATIONAL_B_SPLINE_CURVE" => self.resolve_bspline_curve(entity),
            "POLYLINE" => self.resolve_polyline_curve(entity),
            "TRIMMED_CURVE" => self.resolve_trimmed_curve(entity),
            "COMPOSITE_CURVE" => self.resolve_composite_curve(entity),
            _ => None,
        }
    }

    /// Resolve a LINE curve entity.
    fn resolve_line_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // LINE(#point, #direction)
        let point_id = self.get_ref(entity.params.first()?)?;
        let origin = self.resolve_cartesian_point(point_id)?;
        let dir_id = self.get_ref(entity.params.get(1)?)?;
        let direction = self.resolve_direction(dir_id)?;
        Some(Curve3d::Line(Line::new(origin, direction)))
    }

    /// Resolve a CIRCLE curve entity.
    fn resolve_circle_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // CIRCLE(#axis2_placement, radius)
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (center, normal, x_axis) = self.resolve_axis2(axis2_id)?;
        let radius = self.get_float(entity.params.get(1)?)?;
        Some(Curve3d::Circle(Circle {
            center,
            normal,
            radius,
            x_axis,
        }))
    }

    /// Resolve an ELLIPSE curve entity.
    fn resolve_ellipse_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // ELLIPSE(#axis2_placement, semi_major, semi_minor)
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (center, normal, x_axis) = self.resolve_axis2(axis2_id)?;
        let semi_major = self.get_float(entity.params.get(1)?)?;
        let semi_minor = self.get_float(entity.params.get(2)?)?;
        Some(Curve3d::Ellipse(draper_geometry::Ellipse::new_xy(center, semi_major, semi_minor)))
        // Note: we use new_xy for simplicity; a proper implementation would use the axis2
    }

    /// Resolve a B_SPLINE_CURVE_WITH_KNOTS entity.
    fn resolve_bspline_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let degree = self.get_float(entity.params.first()?).unwrap_or(1.0) as usize;

        // Control points: 2nd param is a list of points
        let mut control_points = Vec::new();
        if let Some(StepValue::List(items)) = entity.params.get(1) {
            for item in items {
                if let Some(ref_id) = self.get_ref(item) {
                    if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                        control_points.push(pt);
                    }
                } else if let StepValue::List(coords) = item {
                    let x = coords.get(0).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                    let y = coords.get(1).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                    let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                    control_points.push(Point3d::new(x, y, z));
                }
            }
        }

        if control_points.is_empty() {
            return None;
        }

        // Default weights (uniform)
        let weights = vec![1.0; control_points.len()];

        // Find knots in remaining params
        let n = control_points.len();
        let knot_count = n + degree + 1;
        let knots: Vec<f64> = (0..knot_count).map(|i| i as f64 / (knot_count - 1).max(1) as f64).collect();

        Some(Curve3d::Nurbs(NurbsCurve {
            degree,
            control_points,
            weights,
            knots,
        }))
    }

    /// Resolve a POLYLINE entity — return as a line segment approximation.
    fn resolve_polyline_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // POLYLINE('', (#pt1, #pt2, ...))
        // Use the first and last points to create a line
        let mut points = Vec::new();
        for param in &entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                            points.push(pt);
                        }
                    }
                }
            }
        }

        if points.len() >= 2 {
            let line = Line::through_points(points[0], *points.last().unwrap())?;
            Some(Curve3d::Line(line))
        } else {
            None
        }
    }

    /// Resolve a TRIMMED_CURVE entity.
    fn resolve_trimmed_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // TRIMMED_CURVE(#basis_curve, #trim1, #trim2, .T., .T., .CARTESIAN., .CARTESIAN.)
        let basis_id = self.get_ref(entity.params.first()?)?;
        self.resolve_curve(basis_id)
    }

    /// Resolve a COMPOSITE_CURVE entity.
    fn resolve_composite_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // COMPOSITE_CURVE('', (#segment1, #segment2, ...), .U.)
        // Use the first segment as a representative curve
        for param in &entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        // Each segment is a COMPOSITE_CURVE_SEGMENT
                        if let Some(seg_entity) = self.step.find_entity(ref_id) {
                            if seg_entity.type_name == "COMPOSITE_CURVE_SEGMENT" {
                                // The 2nd param is the parent curve
                                if let Some(curve_id) = self.find_param_ref(seg_entity, 1) {
                                    if let Some(curve) = self.resolve_curve(curve_id) {
                                        return Some(curve);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Convert a surface to a mesh by creating a Face and triangulating it.
    fn surface_to_mesh(
        &self,
        surface: &Surface,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> TriangleMesh {
        // Create a Face with an empty wire
        let wire = Wire::new(vec![]);
        let mut face = Face::new(surface.clone(), wire);
        face.forward = true;
        face.edges = vec![];

        // For flat planes without bounds, we need to create a finite surface
        // Use the model bounding box to determine extents
        if let Surface::Plane(ref plane) = surface {
            return self.triangulate_bounded_plane(plane, params, bbox);
        }

        // For other surfaces, use the standard triangulation
        triangulate_face(&face, params)
    }

    /// Triangulate a PLANE surface with a finite extent derived from the bounding box.
    fn triangulate_bounded_plane(
        &self,
        plane: &Plane,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        // Determine the plane's extent from the bounding box
        let (size, center) = if let Some((bmin, bmax)) = bbox {
            let cx = (bmin.x + bmax.x) / 2.0;
            let cy = (bmin.y + bmax.y) / 2.0;
            let cz = (bmin.z + bmax.z) / 2.0;
            let sx = (bmax.x - bmin.x).max(1.0);
            let sy = (bmax.y - bmin.y).max(1.0);
            let sz = (bmax.z - bmin.z).max(1.0);
            let max_dim = sx.max(sy).max(sz);
            (max_dim, Point3d::new(cx, cy, cz))
        } else {
            (100.0, plane.origin)
        };

        // Create a grid of points on the plane
        let n = 2; // 2x2 grid for a simple quad
        let half = size * 0.5;

        for j in 0..=n {
            for i in 0..=n {
                let u = -half + size * i as f64 / n as f64;
                let v = -half + size * j as f64 / n as f64;
                let p = Point3d::new(
                    center.x + u * plane.u_dir.x + v * plane.v_dir.x,
                    center.y + u * plane.u_dir.y + v * plane.v_dir.y,
                    center.z + u * plane.u_dir.z + v * plane.v_dir.z,
                );
                mesh.add_vertex(p);
            }
        }

        let cols = n + 1;
        for j in 0..n {
            for i in 0..n {
                let v0 = (j * cols + i) as u32;
                let v1 = (j * cols + i + 1) as u32;
                let v2 = ((j + 1) * cols + i + 1) as u32;
                let v3 = ((j + 1) * cols + i) as u32;
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            }
        }

        mesh
    }

    /// Resolve an AXIS2_PLACEMENT_3D entity to (origin, z_direction, x_direction).
    fn resolve_axis2(&self, axis2_id: i64) -> Option<(Point3d, Direction3d, Direction3d)> {
        let entity = self.step.find_entity(axis2_id)?;
        let point_id = self.get_ref(entity.params.first()?)?;
        let origin = self.resolve_cartesian_point(point_id)?;

        let z_dir = if let Some(dir_param) = entity.params.get(1) {
            if let Some(dir_id) = self.get_ref(dir_param) {
                self.resolve_direction(dir_id).unwrap_or(Direction3d::Z)
            } else {
                Direction3d::Z
            }
        } else {
            Direction3d::Z
        };

        let x_dir = if let Some(dir_param) = entity.params.get(2) {
            if let Some(dir_id) = self.get_ref(dir_param) {
                self.resolve_direction(dir_id).unwrap_or_else(|| {
                    Self::default_x_dir(&z_dir)
                })
            } else {
                Self::default_x_dir(&z_dir)
            }
        } else {
            Self::default_x_dir(&z_dir)
        };

        Some((origin, z_dir, x_dir))
    }

    /// Compute a default x direction given a z direction.
    fn default_x_dir(z_dir: &Direction3d) -> Direction3d {
        if z_dir.is_parallel_to(&Direction3d::Y) {
            Direction3d::X
        } else {
            z_dir.cross(&Direction3d::Y)
        }
    }

    /// Resolve a CARTESIAN_POINT entity.
    fn resolve_cartesian_point(&self, point_id: i64) -> Option<Point3d> {
        let entity = self.step.find_entity(point_id)?;
        for param in &entity.params {
            if let StepValue::List(coords) = param {
                let x = self.get_float(coords.get(0)?)?;
                let y = self.get_float(coords.get(1)?)?;
                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                return Some(Point3d::new(x, y, z));
            }
        }
        None
    }

    /// Resolve a DIRECTION entity.
    fn resolve_direction(&self, dir_id: i64) -> Option<Direction3d> {
        let entity = self.step.find_entity(dir_id)?;
        for param in &entity.params {
            if let StepValue::List(coords) = param {
                let x = self.get_float(coords.get(0)?)?;
                let y = self.get_float(coords.get(1)?)?;
                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                return Direction3d::new(x, y, z);
            }
        }
        None
    }

    // Helper methods

    fn get_ref(&self, value: &StepValue) -> Option<i64> {
        match value {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        }
    }

    fn get_float(&self, value: &StepValue) -> Option<f64> {
        match value {
            StepValue::Float(f) => Some(*f),
            StepValue::Integer(i) => Some(*i as f64),
            _ => None,
        }
    }
}
