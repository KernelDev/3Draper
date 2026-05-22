//! STEP-to-mesh converter.
//!
//! Converts parsed STEP entities into a triangle mesh by:
//! 1. Resolving entity references to build geometry
//! 2. Creating B-Rep faces from STEP surface entities
//! 3. Triangulating using the existing mesh pipeline
//!
//! This is a basic implementation that handles the most common STEP surface types
//! (PLANE, CYLINDRICAL_SURFACE, SPHERICAL_SURFACE, CONICAL_SURFACE, TOROIDAL_SURFACE).
//! Full B-Rep topology reconstruction with trimming is not yet implemented.

use crate::schema::{StepFile, StepValue};
use draper_geometry::{
    Point3d, Direction3d, Surface, Plane, CylinderSurface, SphereSurface,
    ConeSurface, TorusSurface,
};
use draper_mesh::{TriangleMesh, TriangulationParams, triangulate_solid};
use draper_topology::{Solid, Shell, Face, Wire, CoEdge, Edge};
use std::collections::HashMap;
use std::f64::consts::PI;

/// Convert a parsed STEP file to a triangle mesh.
///
/// This is a basic converter that extracts surface geometry from STEP entities
/// and creates approximate mesh representations. Full B-Rep topology reconstruction
/// with proper trimming is not yet implemented — surfaces are rendered untrimmed.
pub fn step_to_mesh(step_file: &StepFile) -> Result<TriangleMesh, String> {
    let converter = StepConverter::new(step_file);
    converter.convert()
}

struct StepConverter<'a> {
    step: &'a StepFile,
    entity_map: HashMap<i64, usize>, // id → index in entities vec
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

        // Strategy: find all MANIFOLD_SOLID_BREP entities and extract their faces
        let breps = self.step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        let mut faces_converted = 0;

        if !breps.is_empty() {
            for brep in &breps {
                // BREP references a CLOSED_SHELL
                if let Some(shell_ref) = brep.params.first() {
                    if let Some(shell_id) = self.get_ref(shell_ref) {
                        if let Some(shell_faces) = self.extract_shell_faces(shell_id) {
                            for surface in shell_faces {
                                let face_mesh = self.surface_to_mesh(&surface, &params);
                                mesh.merge(&face_mesh);
                                faces_converted += 1;
                            }
                        }
                    }
                }
            }
        }

        // If no BREP structure found, try to extract surfaces directly
        if faces_converted == 0 {
            let surface_entities = [
                ("PLANE", "plane"),
                ("CYLINDRICAL_SURFACE", "cylinder"),
                ("SPHERICAL_SURFACE", "sphere"),
                ("CONICAL_SURFACE", "cone"),
                ("TOROIDAL_SURFACE", "torus"),
            ];

            for (type_name, _label) in &surface_entities {
                for entity in self.step.find_entities_by_type(type_name) {
                    if let Some(surface) = self.extract_surface(entity.id) {
                        let face_mesh = self.surface_to_mesh(&surface, &params);
                        mesh.merge(&face_mesh);
                        faces_converted += 1;
                    }
                }
            }
        }

        if faces_converted == 0 {
            return Err("No convertible surface geometry found in STEP file".to_string());
        }

        Ok(mesh)
    }

    /// Extract faces from a CLOSED_SHELL or OPEN_SHELL entity.
    fn extract_shell_faces(&self, shell_id: i64) -> Option<Vec<Surface>> {
        let shell = self.step.find_entity(shell_id)?;
        // Shell has a list of face references
        let mut surfaces = Vec::new();

        // The shell entity format: #N = CLOSED_SHELL('',(#f1,#f2,...));
        // or #N = OPEN_SHELL('',(#f1,#f2,...));
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

    /// Extract the surface geometry from an ADVANCED_FACE entity.
    fn extract_face_surface(&self, face_id: i64) -> Option<Surface> {
        let face = self.step.find_entity(face_id)?;
        // ADVANCED_FACE format: #N = ADVANCED_FACE('',(bounds),#surface_ref,orientation);
        // The surface reference is typically the 3rd parameter (index 2)
        for param in &face.params {
            if let Some(surface_id) = self.get_ref(param) {
                if let Some(surface) = self.extract_surface(surface_id) {
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
            "SURFACE_OF_REVOLUTION" => None, // TODO
            "SURFACE_OF_LINEAR_EXTRUSION" => None, // TODO
            "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" => None, // TODO
            _ => None,
        }
    }

    /// Extract a PLANE surface.
    fn extract_plane(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // PLANE(#axis2_placement)
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (origin, normal, u_dir) = self.resolve_axis2(axis2_id)?;
        let v_dir = normal.cross(&u_dir);
        Some(Surface::Plane(Plane { origin, u_dir, v_dir, normal }))
    }

    /// Extract a CYLINDRICAL_SURFACE.
    fn extract_cylinder(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // CYLINDRICAL_SURFACE(#axis2_placement, radius)
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (origin, axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.get_float(entity.params.get(1)?)?;
        Some(Surface::Cylinder(CylinderSurface::new(origin, axis, radius)))
    }

    /// Extract a SPHERICAL_SURFACE.
    fn extract_sphere(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // SPHERICAL_SURFACE(#axis2_placement, radius)
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (center, _axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.get_float(entity.params.get(1)?)?;
        Some(Surface::Sphere(SphereSurface::new(center, radius)))
    }

    /// Extract a CONICAL_SURFACE.
    fn extract_cone(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // CONICAL_SURFACE(#axis2_placement, radius, semi_angle)
        let axis2_id = self.get_ref(entity.params.first()?)?;
        let (origin, axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.get_float(entity.params.get(1)?)?;
        let half_angle = self.get_float(entity.params.get(2)?)?;
        Some(Surface::Cone(ConeSurface {
            origin,
            axis,
            half_angle: half_angle.abs(), // Ensure positive
            radius,
        }))
    }

    /// Extract a TOROIDAL_SURFACE.
    fn extract_torus(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // TOROIDAL_SURFACE(#axis2_placement, major_radius, minor_radius)
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

    /// Resolve an AXIS2_PLACEMENT_3D entity to (origin, z_direction, x_direction).
    fn resolve_axis2(&self, axis2_id: i64) -> Option<(Point3d, Direction3d, Direction3d)> {
        let entity = self.step.find_entity(axis2_id)?;
        // AXIS2_PLACEMENT_3D(#point, #direction_z, #direction_x)
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
                    if z_dir.is_parallel_to(&Direction3d::Y) {
                        Direction3d::X
                    } else {
                        z_dir.cross(&Direction3d::Y)
                    }
                })
            } else {
                if z_dir.is_parallel_to(&Direction3d::Y) {
                    Direction3d::X
                } else {
                    z_dir.cross(&Direction3d::Y)
                }
            }
        } else {
            if z_dir.is_parallel_to(&Direction3d::Y) {
                Direction3d::X
            } else {
                z_dir.cross(&Direction3d::Y)
            }
        };

        Some((origin, z_dir, x_dir))
    }

    /// Resolve a CARTESIAN_POINT entity.
    fn resolve_cartesian_point(&self, point_id: i64) -> Option<Point3d> {
        let entity = self.step.find_entity(point_id)?;
        // CARTESIAN_POINT('', (x, y, z))
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
        // DIRECTION('', (x, y, z))
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

    /// Convert a surface to a mesh by creating a Face and triangulating it.
    fn surface_to_mesh(&self, surface: &Surface, params: &TriangulationParams) -> TriangleMesh {
        // Create a Face with an empty wire — the triangulation functions
        // will generate the full surface mesh without trimming
        let wire = Wire::new(vec![]);
        let mut face = Face::new(surface.clone(), wire);
        face.forward = true;
        face.edges = vec![];

        // Create a minimal solid to triangulate
        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);
        triangulate_solid(&solid, params)
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
