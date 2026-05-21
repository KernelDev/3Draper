//! Bridge between step-io's StepModel and our B-rep Shape.
//!
//! Converts step-io's typed arena-based IR (StepModel with GeometryPool and
//! TopologyPool) into our custom B-rep Shape with topological entities.

use draper_geometry::curve::{Circle, Curve, Ellipse, Line};
use draper_geometry::direction::{Axis2Placement3D, Direction3};
use draper_geometry::point::Point3;
use draper_geometry::surface::{
    ConicalSurface, CylindricalSurface, Plane, SphericalSurface, Surface,
    SurfaceOfLinearExtrusion, SurfaceOfRevolution, ToroidalSurface,
};
use draper_step::{
    Curve as StepCurve, CurveId, Direction3 as StepDir3,
    NurbsCurve, Orientation,
    Point3 as StepPt3, Placement3dId,
    StepModel, Surface as StepSurface,
};
use draper_topology::entity::*;
use draper_topology::shape::Shape;

/// Convert a step-io StepModel to a B-rep Shape.
///
/// This is the primary conversion pipeline: StepModel (typed arenas) → Shape (B-rep).
/// The conversion resolves all geometry references through the arenas and
/// creates our custom topological entities with embedded geometric data.
pub fn step_model_to_shape(model: &StepModel) -> Shape {
    let mut ctx = ConversionContext::new(model);
    ctx.convert();
    ctx.shape
}

/// Convert a step-io StepModel back to a StepDocument (for write support).
/// This is a placeholder — step-io has its own writer.
pub fn shape_to_step_model(_shape: &Shape) -> StepModel {
    let model = StepModel::default();
    log::warn!("shape_to_step_model: conversion not yet implemented, returning empty model");
    model
}

struct ConversionContext<'a> {
    model: &'a StepModel,
    shape: Shape,
    /// Maps step-io VertexId to our TopoId
    vertex_map: std::collections::HashMap<u32, TopoId>,
    /// Maps step-io EdgeId to our TopoId
    edge_map: std::collections::HashMap<u32, TopoId>,
    /// Maps step-io WireId to our TopoId
    wire_map: std::collections::HashMap<u32, TopoId>,
    /// Maps step-io FaceId to our TopoId
    face_map: std::collections::HashMap<u32, TopoId>,
    /// Maps step-io ShellId to our TopoId
    shell_map: std::collections::HashMap<u32, TopoId>,
    /// Scale factor based on units (convert to mm)
    scale: f64,
}

impl<'a> ConversionContext<'a> {
    fn new(model: &'a StepModel) -> Self {
        let scale = model.units.as_ref().map_or(1.0, |u| match u.length {
            draper_step::LengthUnit::Millimetre => 1.0,
            draper_step::LengthUnit::Metre => 1000.0,
            draper_step::LengthUnit::Centimetre => 10.0,
            draper_step::LengthUnit::Inch => 25.4,
            draper_step::LengthUnit::Foot => 304.8,
        });

        log::info!("Unit scale factor: {} (converting to mm)", scale);

        Self {
            model,
            shape: Shape::new(),
            vertex_map: std::collections::HashMap::new(),
            edge_map: std::collections::HashMap::new(),
            wire_map: std::collections::HashMap::new(),
            face_map: std::collections::HashMap::new(),
            shell_map: std::collections::HashMap::new(),
            scale,
        }
    }

    fn convert(&mut self) {
        // Phase 1: Convert vertices (CARTESIAN_POINT → Vertex with Point3)
        self.convert_vertices();

        // Phase 2: Convert edges (EDGE_CURVE → Edge with Curve)
        self.convert_edges();

        // Phase 3: Convert wires (EDGE_LOOP → Wire with OrientedEdges)
        self.convert_wires();

        // Phase 4: Convert faces (ADVANCED_FACE → Face with Surface)
        self.convert_faces();

        // Phase 5: Convert shells (CLOSED_SHELL → Shell)
        self.convert_shells();

        // Phase 6: Convert solids (MANIFOLD_SOLID_BREP → Solid)
        self.convert_solids();

        log::info!(
            "Conversion complete: {} vertices, {} edges, {} faces, {} shells, {} solids",
            self.shape.vertices().len(),
            self.shape.edges().len(),
            self.shape.faces().len(),
            self.shape.shells().len(),
            self.shape.solids().len(),
        );
    }

    /// Convert step-io vertices to our vertices.
    fn convert_vertices(&mut self) {
        for (i, vertex) in self.model.topology.vertices.iter().enumerate() {
            let step_point = &self.model.geometry.points[vertex.point];
            let point = self.convert_point(*step_point);
            let topo_id = self.shape.add_vertex(point);
            self.vertex_map.insert(i as u32, topo_id);
        }
        log::debug!("Converted {} vertices", self.model.topology.vertices.len());
    }

    /// Convert step-io edges to our edges.
    fn convert_edges(&mut self) {
        for (i, edge) in self.model.topology.edges.iter().enumerate() {
            let curve = self.convert_curve(edge.curve);

            let (start_vid, end_vid) = edge.vertices;
            let start_topo = self.vertex_map.get(&start_vid.0).copied();
            let end_topo = self.vertex_map.get(&end_vid.0).copied();

            match (start_topo, end_topo) {
                (Some(sv), Some(ev)) => {
                    let param_range = if edge.trim != (0.0, 0.0) {
                        Some(edge.trim)
                    } else {
                        None
                    };
                    let id = self.shape.add_edge(curve, sv, ev, param_range);
                    self.edge_map.insert(i as u32, id);
                }
                _ => {
                    log::warn!(
                        "Edge #{}: missing vertices (start={:?}, end={:?})",
                        i, start_topo, end_topo
                    );
                }
            }
        }
        log::debug!("Converted {} edges", self.model.topology.edges.len());
    }

    /// Convert step-io wires to our wires.
    ///
    /// IMPORTANT: STEP files may list oriented edges in non-consecutive order
    /// within an edge_loop. We must reorder them so that the end vertex of
    /// edge[i] matches the start vertex of edge[i+1], forming a proper
    /// closed loop. Without this, the wire traversal produces zigzag paths
    /// that cause CDT triangulation failures.
    fn convert_wires(&mut self) {
        for (i, wire) in self.model.topology.wires.iter().enumerate() {
            let oriented_edges: Vec<OrientedEdge> = wire
                .edges
                .iter()
                .filter_map(|oe| {
                    self.edge_map.get(&oe.edge.0).copied().map(|edge_id| {
                        let orientation = matches!(oe.orientation, Orientation::Forward);
                        OrientedEdge {
                            edge_id,
                            orientation,
                        }
                    })
                })
                .collect();

            if !oriented_edges.is_empty() {
                // Reorder edges to form a consecutive path
                let ordered_edges = reorder_wire_edges(&oriented_edges, &self.shape);
                let id = self.shape.add_wire(ordered_edges);
                self.wire_map.insert(i as u32, id);
            } else if wire.vertex.is_some() {
                // Degenerate wire (VERTEX_LOOP) — create a single-vertex wire
                let empty_wire_id = self.shape.add_wire(Vec::new());
                self.wire_map.insert(i as u32, empty_wire_id);
            }
        }
        log::debug!("Converted {} wires", self.model.topology.wires.len());
    }

    /// Convert step-io faces to our faces.
    fn convert_faces(&mut self) {
        for (i, face) in self.model.topology.faces.iter().enumerate() {
            let surface = self.convert_surface(face.surface);

            let face_id = self.shape.add_face(surface);

            // Set face orientation
            if let Some(TopoShape::Face(f)) = self.shape.get_mut(face_id) {
                f.orientation = matches!(face.orientation, Orientation::Forward);
            }

            // Set boundaries
            let mut first_bound = true;
            for &wire_id in &face.bounds {
                if let Some(wire_topo) = self.wire_map.get(&wire_id.0).copied() {
                    // Determine if this is the outer or inner bound
                    let step_wire = &self.model.topology.wires[wire_id];
                    let is_outer = step_wire.is_outer;

                    if is_outer && first_bound {
                        self.shape.set_face_outer_wire(face_id, wire_topo);
                    } else {
                        self.shape.add_face_inner_wire(face_id, wire_topo);
                    }
                    first_bound = false;
                }
            }

            self.face_map.insert(i as u32, face_id);
        }
        log::debug!("Converted {} faces", self.model.topology.faces.len());
    }

    /// Convert step-io shells to our shells.
    fn convert_shells(&mut self) {
        for (i, shell) in self.model.topology.shells.iter().enumerate() {
            let face_ids: Vec<TopoId> = shell
                .faces
                .iter()
                .filter_map(|fid| self.face_map.get(&fid.0).copied())
                .collect();

            if !face_ids.is_empty() {
                let id = self.shape.add_shell(face_ids);
                self.shell_map.insert(i as u32, id);
            }
        }
        log::debug!("Converted {} shells", self.model.topology.shells.len());
    }

    /// Convert step-io solids to our solids.
    fn convert_solids(&mut self) {
        for (i, solid) in self.model.topology.solids.iter().enumerate() {
            // shells[0] is the outer shell
            if let Some(outer_shell_id) = solid.shells.first() {
                if let Some(shell_topo) = self.shell_map.get(&outer_shell_id.0).copied() {
                    let solid_id = self.shape.add_solid(shell_topo);

                    // Add inner shells (voids) if any
                    for inner_shell_id in solid.shells.iter().skip(1) {
                        if let Some(inner_topo) = self.shell_map.get(&inner_shell_id.0).copied() {
                            if let Some(TopoShape::Solid(s)) = self.shape.get_mut(solid_id) {
                                s.inner_shells.push(inner_topo);
                            }
                        }
                    }

                    log::trace!(
                        "Solid #{}: outer_shell={} ({} shells total)",
                        i, shell_topo, solid.shells.len()
                    );
                }
            }
        }
        log::debug!("Converted {} solids", self.model.topology.solids.len());
    }

    // ---- Geometry conversion helpers ----

    fn convert_point(&self, p: StepPt3) -> Point3 {
        Point3::new(p.x * self.scale, p.y * self.scale, p.z * self.scale)
    }

    fn convert_direction(&self, d: StepDir3) -> Direction3 {
        Direction3::new(d.x, d.y, d.z).unwrap_or(Direction3::Z)
    }

    fn convert_axis2_placement(&self, placement_id: Placement3dId) -> Axis2Placement3D {
        let placement = &self.model.geometry.placements[placement_id];
        let location = self.convert_point(self.model.geometry.points[placement.location]);

        let axis = placement
            .axis
            .map(|dir_id| self.convert_direction(self.model.geometry.directions[dir_id]))
            .unwrap_or(Direction3::Z);

        let ref_direction = placement
            .ref_direction
            .map(|dir_id| self.convert_direction(self.model.geometry.directions[dir_id]));

        Axis2Placement3D::new(location, axis, ref_direction)
    }

    fn convert_curve(&self, curve_id: CurveId) -> Option<Curve> {
        let curve = &self.model.geometry.curves[curve_id];
        match curve {
            StepCurve::Line(line) => {
                let origin = self.convert_point(self.model.geometry.points[line.point]);
                let direction = self.convert_direction(self.model.geometry.directions[line.direction]);
                Some(Curve::Line(Line::new(origin, direction)))
            }
            StepCurve::Circle(circle) => {
                let axis = self.convert_axis2_placement(circle.position);
                let radius = circle.radius * self.scale;
                Some(Curve::Circle(Circle::new(axis, radius)))
            }
            StepCurve::Ellipse(ellipse) => {
                let axis = self.convert_axis2_placement(ellipse.position);
                let semi_axis_1 = ellipse.semi_axis_1 * self.scale;
                let semi_axis_2 = ellipse.semi_axis_2 * self.scale;
                log::debug!(
                    "Converting ellipse (semi_axis_1={}, semi_axis_2={})",
                    semi_axis_1, semi_axis_2
                );
                Some(Curve::Ellipse(Ellipse::new(axis, semi_axis_1, semi_axis_2)))
            }
            StepCurve::Nurbs(nurbs) => {
                // For now, approximate NURBS curve as a line between first and last control points
                // TODO: Implement proper NURBS curve evaluation
                self.approximate_nurbs_curve(nurbs)
            }
        }
    }

    fn convert_surface(&self, surface_id: draper_step::SurfaceId) -> Option<Surface> {
        let surface = &self.model.geometry.surfaces[surface_id];
        match surface {
            StepSurface::Plane(plane) => {
                let axis = self.convert_axis2_placement(plane.position);
                Some(Surface::Plane(Plane::new(axis)))
            }
            StepSurface::Cylinder(cyl) => {
                let axis = self.convert_axis2_placement(cyl.position);
                let radius = cyl.radius * self.scale;
                Some(Surface::CylindricalSurface(CylindricalSurface::new(axis, radius)))
            }
            StepSurface::Cone(cone) => {
                let axis = self.convert_axis2_placement(cone.position);
                let radius = cone.radius * self.scale;
                let semi_angle = cone.semi_angle;
                Some(Surface::ConicalSurface(ConicalSurface::new(axis, radius, semi_angle)))
            }
            StepSurface::Sphere(sphere) => {
                let axis = self.convert_axis2_placement(sphere.position);
                let radius = sphere.radius * self.scale;
                Some(Surface::SphericalSurface(SphericalSurface::new(axis, radius)))
            }
            StepSurface::Torus(torus) => {
                let axis = self.convert_axis2_placement(torus.position);
                let major_radius = torus.major_radius * self.scale;
                let minor_radius = torus.minor_radius * self.scale;
                Some(Surface::ToroidalSurface(ToroidalSurface::new(axis, major_radius, minor_radius)))
            }
            StepSurface::Revolution(rev) => {
                // Surface of revolution: a curve swept around an axis
                let generatrix = match self.convert_curve(rev.swept_curve) {
                    Some(c) => c,
                    None => {
                        log::warn!("SurfaceOfRevolution: failed to convert swept curve, using faceted fallback");
                        return None;
                    }
                };
                let axis_placement = &self.model.geometry.placements_1d[rev.axis_placement];
                let location = self.convert_point(self.model.geometry.points[axis_placement.location]);
                let axis_dir = self.convert_direction(self.model.geometry.directions[axis_placement.axis]);

                // Build an Axis2Placement3D from the axis1 placement.
                // The axis direction is the Z-axis; we need to compute a ref_direction
                // perpendicular to it for the X-axis.
                let ref_dir = if axis_dir.dot(draper_geometry::direction::Direction3::X).abs() < 0.9 {
                    axis_dir.cross(draper_geometry::direction::Direction3::X)
                } else {
                    axis_dir.cross(draper_geometry::direction::Direction3::Y)
                };
                let axis2 = draper_geometry::direction::Axis2Placement3D::new(location, axis_dir, Some(ref_dir));

                log::debug!(
                    "SurfaceOfRevolution: generatrix={:?}, axis_loc=({:.3},{:.3},{:.3})",
                    std::mem::discriminant(&generatrix),
                    location.x, location.y, location.z
                );

                Some(Surface::SurfaceOfRevolution(SurfaceOfRevolution {
                    generatrix,
                    axis: axis2,
                }))
            }
            StepSurface::Extrusion(ext) => {
                // Surface of linear extrusion: a curve swept along a direction
                let generatrix = match self.convert_curve(ext.swept_curve) {
                    Some(c) => c,
                    None => {
                        log::warn!("SurfaceOfLinearExtrusion: failed to convert swept curve, using faceted fallback");
                        return None;
                    }
                };
                let direction = self.convert_direction(self.model.geometry.directions[ext.extrusion_direction]);

                log::debug!(
                    "SurfaceOfLinearExtrusion: generatrix={:?}, dir=({:.3},{:.3},{:.3}), depth={:.3}",
                    std::mem::discriminant(&generatrix),
                    direction.x, direction.y, direction.z,
                    ext.depth * self.scale
                );

                Some(Surface::SurfaceOfLinearExtrusion(
                    SurfaceOfLinearExtrusion {
                        generatrix,
                        direction,
                    }
                ))
            }
            StepSurface::Nurbs(nurbs) => {
                log::debug!(
                    "NURBS surface: not yet supported, using faceted fallback ({}x{} control points)",
                    nurbs.u_degree, nurbs.v_degree
                );
                None
            }
        }
    }

    /// Approximate a NURBS curve by sampling control points.
    fn approximate_nurbs_curve(&self, nurbs: &NurbsCurve) -> Option<Curve> {
        if nurbs.control_points.len() < 2 {
            return None;
        }

        // Use first and last control points to create a line approximation
        let first = self.convert_point(self.model.geometry.points[nurbs.control_points[0]]);
        let last = self.convert_point(self.model.geometry.points[*nurbs.control_points.last()?]);

        if nurbs.control_points.len() == 2 {
            let direction = Direction3::new(
                last.x - first.x,
                last.y - first.y,
                last.z - first.z,
            )?;
            Some(Curve::Line(Line::new(first, direction)))
        } else {
            log::debug!(
                "Approximating NURBS curve (degree={}, {} control points) as line",
                nurbs.degree,
                nurbs.control_points.len()
            );
            let direction = Direction3::new(
                last.x - first.x,
                last.y - first.y,
                last.z - first.z,
            )?;
            Some(Curve::Line(Line::new(first, direction)))
        }
    }
}

/// Reorder oriented edges in a wire so they form a consecutive closed path.
///
/// In STEP files, oriented edges within an EDGE_LOOP may be listed in arbitrary
/// order. This function reorders them so that the end vertex of edge[i] equals
/// the start vertex of edge[i+1], forming a proper closed loop suitable for
/// boundary traversal and CDT triangulation.
///
/// Algorithm:
/// 1. Build a connectivity map: from_vertex → (edge_index, to_vertex)
/// 2. Start from the first edge's start vertex
/// 3. Walk the chain: from current vertex, find the next edge that starts here
/// 4. If the chain is complete (all edges used), return the ordered list
/// 5. If not, fall back to the original order
fn reorder_wire_edges(edges: &[OrientedEdge], shape: &Shape) -> Vec<OrientedEdge> {
    if edges.len() <= 2 {
        return edges.to_vec();
    }

    use std::collections::HashMap;

    // Build connectivity: start_vertex → Vec<(edge_index, end_vertex)>
    let mut from_map: HashMap<TopoId, Vec<(usize, TopoId)>> = HashMap::new();

    for (i, oe) in edges.iter().enumerate() {
        let edge = match shape.get(oe.edge_id) {
            Some(TopoShape::Edge(e)) => e,
            _ => continue,
        };
        let (start, end) = if oe.orientation {
            (edge.start_vertex, edge.end_vertex)
        } else {
            (edge.end_vertex, edge.start_vertex)
        };
        from_map.entry(start).or_default().push((i, end));
    }

    // Find a good starting edge: pick the first edge that has a unique start vertex
    // (i.e., only one edge starts from that vertex). This avoids ambiguity.
    let mut start_edge_idx = 0;
    for (i, oe) in edges.iter().enumerate() {
        let edge = match shape.get(oe.edge_id) {
            Some(TopoShape::Edge(e)) => e,
            _ => continue,
        };
        let start = if oe.orientation {
            edge.start_vertex
        } else {
            edge.end_vertex
        };
        if let Some(candidates) = from_map.get(&start) {
            if candidates.len() == 1 {
                start_edge_idx = i;
                break;
            }
        }
    }

    // Get the starting vertex
    let first_edge = &edges[start_edge_idx];
    let first_edge_data = match shape.get(first_edge.edge_id) {
        Some(TopoShape::Edge(e)) => e,
        _ => return edges.to_vec(),
    };
    let start_vertex = if first_edge.orientation {
        first_edge_data.start_vertex
    } else {
        first_edge_data.end_vertex
    };

    // Walk the chain
    let mut ordered = Vec::with_capacity(edges.len());
    let mut used = vec![false; edges.len()];
    let mut current_vertex = start_vertex;

    for _ in 0..edges.len() {
        // Find an unused edge that starts from current_vertex
        if let Some(candidates) = from_map.get(&current_vertex) {
            let mut found = false;
            for &(idx, to_v) in candidates {
                if !used[idx] {
                    ordered.push(edges[idx]);
                    used[idx] = true;
                    current_vertex = to_v;
                    found = true;
                    break;
                }
            }
            if !found {
                log::warn!(
                    "Wire reordering: no edge starting from vertex {} at step {}",
                    current_vertex,
                    ordered.len()
                );
                break;
            }
        } else {
            log::warn!(
                "Wire reordering: no edges from vertex {} at step {}",
                current_vertex,
                ordered.len()
            );
            break;
        }
    }

    if ordered.len() == edges.len() {
        log::trace!(
            "Wire reordering: successfully reordered {} edges",
            ordered.len()
        );
        ordered
    } else {
        log::warn!(
            "Wire reordering: only ordered {}/{} edges, using original order",
            ordered.len(),
            edges.len()
        );
        edges.to_vec()
    }
}
