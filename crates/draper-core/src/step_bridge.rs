//! Bridge between STEP AST and B-rep Shape.
//!
//! Converts STEP entity instances into topological and geometric objects.

use draper_geometry::curve::{Circle, Curve, Line};
use draper_geometry::direction::{Axis2Placement3D, Direction3};
use draper_geometry::point::Point3;
use draper_geometry::surface::{CylindricalSurface, Plane, SphericalSurface, Surface};
use draper_step::ast::{Parameter, StepDocument, StepEntity};
use draper_topology::entity::*;
use draper_topology::shape::Shape;

use crate::error::{CoreError, CoreResult};

/// Convert a STEP document to a B-rep Shape.
pub fn step_to_shape(doc: &StepDocument, shape: &mut Shape) {
    let ctx = &mut BridgeContext { doc, shape, id_map: Default::default() };

    // First pass: create all geometry (points, directions, curves, surfaces)
    for id in &doc.entity_order {
        if let Some(entity) = doc.get_entity(*id) {
            ctx.process_geometry_entity(entity);
        }
    }

    // Second pass: create topology (vertices, edges, wires, faces, shells, solids)
    for id in &doc.entity_order {
        if let Some(entity) = doc.get_entity(*id) {
            ctx.process_topology_entity(entity);
        }
    }

    // Find root shapes
    let roots: Vec<TopoId> = shape.solids().iter().map(|s| s.id).collect();
    let compound_roots: Vec<TopoId> = shape.find_by_type(ShapeType::Compound)
        .iter().map(|s| s.id()).collect();
    
    let all_roots = [roots, compound_roots].concat();
    if !all_roots.is_empty() {
        shape.set_roots(all_roots);
    }
}

/// Convert a B-rep Shape back to a STEP document.
pub fn shape_to_step(shape: &Shape) -> CoreResult<StepDocument> {
    let mut doc = StepDocument::new();

    // Set header
    doc.header.file_description.description.push("3Draper exported file".to_string());
    doc.header.file_description.implementation_level = "2;1".to_string();
    doc.header.file_name.name = "3draper_export.stp".to_string();
    doc.header.file_name.time_stamp = chrono::Local::now().format("%Y-%m-%dT%H:%M:%S").to_string();
    doc.header.file_name.author.push("3Draper".to_string());
    doc.header.file_name.organization.push("3Draper".to_string());
    doc.header.file_name.preprocessor_version = "3Draper 0.1".to_string();
    doc.header.file_name.originating_system = "3Draper".to_string();
    doc.header.file_name.authorization = "".to_string();
    doc.header.file_schema.schemas.push("AUTOMOTIVE_DESIGN".to_string());

    let mut next_id = 1u64;

    // Write all entities
    // This is a simplified exporter — a full one would need to handle all entity types
    for vertex in shape.vertices() {
        let id = next_id;
        next_id += 1;

        let pt_id = next_id;
        next_id += 1;
        doc.entity_order.push(pt_id);
        doc.entities.insert(pt_id, StepEntity {
            id: pt_id,
            type_name: "CARTESIAN_POINT".to_string(),
            parameters: vec![
                Parameter::String(String::new()),
                Parameter::List(vec![
                    Parameter::Real(vertex.point.x),
                    Parameter::Real(vertex.point.y),
                    Parameter::Real(vertex.point.z),
                ]),
            ],
        });

        doc.entity_order.push(id);
        doc.entities.insert(id, StepEntity {
            id,
            type_name: "VERTEX_POINT".to_string(),
            parameters: vec![
                Parameter::String(String::new()),
                Parameter::Reference(pt_id),
            ],
        });
    }

    Ok(doc)
}

struct BridgeContext<'a> {
    doc: &'a StepDocument,
    shape: &'a mut Shape,
    /// Maps STEP entity IDs to TopoIds.
    id_map: std::collections::HashMap<u64, TopoId>,
}

impl<'a> BridgeContext<'a> {
    fn process_geometry_entity(&mut self, entity: &StepEntity) {
        match entity.type_name.as_str() {
            "CARTESIAN_POINT" => {
                if let Some(point) = self.parse_cartesian_point(entity) {
                    let id = self.shape.add_vertex(point);
                    self.id_map.insert(entity.id, id);
                }
            }
            "DIRECTION" => {
                // Directions are stored in the id_map for reference by other entities
                // We don't create topological entities for them
                let id = self.shape.alloc_id_internal();
                self.id_map.insert(entity.id, id);
            }
            "VECTOR" => {
                let id = self.shape.alloc_id_internal();
                self.id_map.insert(entity.id, id);
            }
            "AXIS2_PLACEMENT_3D" => {
                let id = self.shape.alloc_id_internal();
                self.id_map.insert(entity.id, id);
            }
            "LINE" | "CIRCLE" | "ELLIPSE" | "B_SPLINE_CURVE_WITH_KNOTS" | "B_SPLINE_CURVE" => {
                let id = self.shape.alloc_id_internal();
                self.id_map.insert(entity.id, id);
            }
            "PLANE" | "CYLINDRICAL_SURFACE" | "CONICAL_SURFACE" | "SPHERICAL_SURFACE"
            | "TOROIDAL_SURFACE" | "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE"
            | "SURFACE_OF_REVOLUTION" | "SURFACE_OF_LINEAR_EXTRUSION" => {
                let id = self.shape.alloc_id_internal();
                self.id_map.insert(entity.id, id);
            }
            _ => {}
        }
    }

    fn process_topology_entity(&mut self, entity: &StepEntity) {
        match entity.type_name.as_str() {
            "VERTEX_POINT" => {
                // The vertex was already created in the geometry pass if it has a CARTESIAN_POINT
                // But we need to handle the case where it references an existing point
                if let Some(ref_id) = entity.ref_param(1) {
                    if let Some(&topo_id) = self.id_map.get(&ref_id) {
                        self.id_map.insert(entity.id, topo_id);
                    }
                }
            }
            "EDGE_CURVE" => {
                self.process_edge_curve(entity);
            }
            "ORIENTED_EDGE" => {
                // Oriented edges reference an EDGE_CURVE with an orientation flag
                if let Some(edge_ref) = entity.ref_param(4) {
                    if let Some(&topo_id) = self.id_map.get(&edge_ref) {
                        self.id_map.insert(entity.id, topo_id);
                    }
                }
            }
            "EDGE_LOOP" => {
                self.process_edge_loop(entity);
            }
            "FACE_OUTER_BOUND" | "FACE_BOUND" => {
                if let Some(ref_id) = entity.ref_param(1) {
                    if let Some(&topo_id) = self.id_map.get(&ref_id) {
                        self.id_map.insert(entity.id, topo_id);
                    }
                }
            }
            "ADVANCED_FACE" => {
                self.process_advanced_face(entity);
            }
            "CLOSED_SHELL" | "OPEN_SHELL" => {
                self.process_shell(entity);
            }
            "MANIFOLD_SOLID_BREP" => {
                self.process_solid(entity);
            }
            _ => {
                // Store a mapping for unknown types so references can be resolved
                if !self.id_map.contains_key(&entity.id) {
                    let id = self.shape.alloc_id_internal();
                    self.id_map.insert(entity.id, id);
                }
            }
        }
    }

    fn parse_cartesian_point(&self, entity: &StepEntity) -> Option<Point3> {
        // CARTESIAN_POINT(name, (x, y, z))
        let coords = entity.list_param(1)?;
        match coords.len() {
            3 => Some(Point3::new(
                self.param_real(&coords[0])?,
                self.param_real(&coords[1])?,
                self.param_real(&coords[2])?,
            )),
            2 => Some(Point3::new(
                self.param_real(&coords[0])?,
                self.param_real(&coords[1])?,
                0.0,
            )),
            _ => None,
        }
    }

    fn parse_direction(&self, entity: &StepEntity) -> Option<Direction3> {
        // DIRECTION(name, (x, y, z))
        let coords = entity.list_param(1)?;
        Direction3::new(
            self.param_real(coords.get(0)?)?,
            self.param_real(coords.get(1)?)?,
            if coords.len() > 2 { self.param_real(coords.get(2)?)? } else { 0.0 },
        )
    }

    fn parse_axis2_placement(&self, entity: &StepEntity) -> Option<Axis2Placement3D> {
        // AXIS2_PLACEMENT_3D(name, location_ref, axis_ref, ref_direction_ref)
        let location_ref = entity.ref_param(1)?;
        let location_entity = self.doc.get_entity(location_ref)?;
        let location = self.parse_cartesian_point(location_entity)?;

        let axis = if let Some(axis_ref) = entity.ref_param(2) {
            self.doc.get_entity(axis_ref)
                .and_then(|e| self.parse_direction(e))
                .unwrap_or(Direction3::Z)
        } else {
            Direction3::Z
        };

        let ref_direction = if let Some(ref_dir_ref) = entity.ref_param(3) {
            self.doc.get_entity(ref_dir_ref)
                .and_then(|e| self.parse_direction(e))
        } else {
            None
        };

        Some(Axis2Placement3D::new(location, axis, ref_direction))
    }

    fn process_edge_curve(&mut self, entity: &StepEntity) {
        // EDGE_CURVE(name, start_vertex_ref, end_vertex_ref, curve_ref, same_sense)
        let start_ref = entity.ref_param(1).unwrap_or(0);
        let end_ref = entity.ref_param(2).unwrap_or(0);
        let curve_ref = entity.ref_param(3);

        let start_topo = self.id_map.get(&start_ref).copied();
        let end_topo = self.id_map.get(&end_ref).copied();

        let curve = curve_ref.and_then(|r| self.doc.get_entity(r))
            .and_then(|e| self.parse_curve(e));

        if let (Some(sv), Some(ev)) = (start_topo, end_topo) {
            let id = self.shape.add_edge(curve, sv, ev, None);
            self.id_map.insert(entity.id, id);
        }
    }

    fn process_edge_loop(&mut self, entity: &StepEntity) {
        // EDGE_LOOP(name, (oriented_edge_list))
        let edge_list = entity.list_param(1);
        if let Some(edges) = edge_list {
            let oriented_edges: Vec<OrientedEdge> = edges
                .iter()
                .filter_map(|p| {
                    if let Parameter::Reference(ref_id) = p {
                        // The oriented edge itself has orientation info
                        if let Some(oe_entity) = self.doc.get_entity(*ref_id) {
                            let edge_ref = oe_entity.ref_param(4).unwrap_or(0);
                            let orientation = match oe_entity.param(5) {
                                Some(Parameter::Enumeration(s)) => s != "F",
                                Some(Parameter::Omitted) => true,
                                _ => true,
                            };
                            if let Some(&edge_topo) = self.id_map.get(&edge_ref) {
                                return Some(OrientedEdge {
                                    edge_id: edge_topo,
                                    orientation,
                                });
                            }
                        }
                    }
                    None
                })
                .collect();

            if !oriented_edges.is_empty() {
                let id = self.shape.add_wire(oriented_edges);
                self.id_map.insert(entity.id, id);
            }
        }
    }

    fn process_advanced_face(&mut self, entity: &StepEntity) {
        // ADVANCED_FACE(name, (bound_list), surface_ref, same_sense)
        let surface_ref = entity.ref_param(2);
        let surface = surface_ref
            .and_then(|r| self.doc.get_entity(r))
            .and_then(|e| self.parse_surface(e));

        let face_id = self.shape.add_face(surface);

        // Set boundaries
        if let Some(bound_list) = entity.list_param(1) {
            for bound_param in bound_list {
                if let Parameter::Reference(bound_ref) = bound_param {
                    if let Some(&wire_topo) = self.id_map.get(bound_ref) {
                        // Determine if this is outer or inner bound
                        if let Some(bound_entity) = self.doc.get_entity(*bound_ref) {
                            if bound_entity.type_name == "FACE_OUTER_BOUND" {
                                self.shape.set_face_outer_wire(face_id, wire_topo);
                            } else {
                                self.shape.add_face_inner_wire(face_id, wire_topo);
                            }
                        }
                    }
                }
            }
        }

        self.id_map.insert(entity.id, face_id);
    }

    fn process_shell(&mut self, entity: &StepEntity) {
        // CLOSED_SHELL(name, (face_list))
        let face_list = entity.list_param(0);
        if let Some(faces) = face_list {
            let face_ids: Vec<TopoId> = faces
                .iter()
                .filter_map(|p| {
                    if let Parameter::Reference(ref_id) = p {
                        self.id_map.get(ref_id).copied()
                    } else {
                        None
                    }
                })
                .collect();

            if !face_ids.is_empty() {
                let id = self.shape.add_shell(face_ids);
                self.id_map.insert(entity.id, id);
            }
        }
    }

    fn process_solid(&mut self, entity: &StepEntity) {
        // MANIFOLD_SOLID_BREP(name, shell_ref)
        let shell_ref = entity.ref_param(1).unwrap_or(0);
        if let Some(&shell_topo) = self.id_map.get(&shell_ref) {
            let id = self.shape.add_solid(shell_topo);
            self.id_map.insert(entity.id, id);
        }
    }

    fn parse_curve(&self, entity: &StepEntity) -> Option<Curve> {
        match entity.type_name.as_str() {
            "LINE" => {
                // LINE(name, point_ref, vector_ref)
                let point_ref = entity.ref_param(1)?;
                let point_entity = self.doc.get_entity(point_ref)?;
                let origin = self.parse_cartesian_point(point_entity)?;

                // Vector has a direction and magnitude
                let vector_ref = entity.ref_param(2)?;
                let vector_entity = self.doc.get_entity(vector_ref)?;
                let dir_ref = vector_entity.ref_param(1)?;
                let dir_entity = self.doc.get_entity(dir_ref)?;
                let direction = self.parse_direction(dir_entity)?;

                Some(Curve::Line(Line::new(origin, direction)))
            }
            "CIRCLE" => {
                let axis_ref = entity.ref_param(1)?;
                let axis_entity = self.doc.get_entity(axis_ref)?;
                let axis = self.parse_axis2_placement(axis_entity)?;
                let radius = entity.real_param(2)?;

                Some(Curve::Circle(Circle::new(axis, radius)))
            }
            _ => None,
        }
    }

    fn parse_surface(&self, entity: &StepEntity) -> Option<Surface> {
        match entity.type_name.as_str() {
            "PLANE" => {
                let axis_ref = entity.ref_param(1)?;
                let axis_entity = self.doc.get_entity(axis_ref)?;
                let axis = self.parse_axis2_placement(axis_entity)?;
                Some(Surface::Plane(Plane::new(axis)))
            }
            "CYLINDRICAL_SURFACE" => {
                let axis_ref = entity.ref_param(1)?;
                let axis_entity = self.doc.get_entity(axis_ref)?;
                let axis = self.parse_axis2_placement(axis_entity)?;
                let radius = entity.real_param(2)?;
                Some(Surface::CylindricalSurface(CylindricalSurface::new(axis, radius)))
            }
            "SPHERICAL_SURFACE" => {
                let center_ref = entity.ref_param(1)?;
                let center_entity = self.doc.get_entity(center_ref)?;
                let axis = self.parse_axis2_placement(center_entity)?;
                let radius = entity.real_param(2)?;
                Some(Surface::SphericalSurface(SphericalSurface::new(axis, radius)))
            }
            _ => None,
        }
    }

    fn param_real(&self, param: &Parameter) -> Option<f64> {
        match param {
            Parameter::Real(v) => Some(*v),
            Parameter::Integer(v) => Some(*v as f64),
            _ => None,
        }
    }

    fn param_real_owned(&self, param: Parameter) -> Option<f64> {
        self.param_real(&param)
    }
}

// Internal helper for Shape
trait ShapeInternal {
    fn alloc_id_internal(&mut self) -> TopoId;
}

impl ShapeInternal for Shape {
    fn alloc_id_internal(&mut self) -> TopoId {
        // Use a high ID range to avoid conflicts with real topological entities
        // This is a bit of a hack — in a production system we'd have a cleaner approach
        static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1_000_000);
        COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst)
    }
}
