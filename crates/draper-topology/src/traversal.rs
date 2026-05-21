//! Shape traversal — iterate over the B-rep hierarchy.

use crate::entity::*;
use crate::shape::Shape;

/// Visitor pattern for traversing a shape's topology.
pub trait TopoVisitor {
    fn visit_vertex(&mut self, _vertex: &Vertex, _shape: &Shape) {}
    fn visit_edge(&mut self, _edge: &Edge, _shape: &Shape) {}
    fn visit_wire(&mut self, _wire: &Wire, _shape: &Shape) {}
    fn visit_face(&mut self, _face: &Face, _shape: &Shape) {}
    fn visit_shell(&mut self, _shell: &Shell, _shape: &Shape) {}
    fn visit_solid(&mut self, _solid: &Solid, _shape: &Shape) {}
    fn visit_compound(&mut self, _compound: &Compound, _shape: &Shape) {}
}

/// Traverse the entire shape, calling the visitor for each entity.
pub fn traverse_shape(shape: &Shape, visitor: &mut dyn TopoVisitor) {
    for entity in shape.entities.values() {
        match entity {
            TopoShape::Vertex(v) => visitor.visit_vertex(v, shape),
            TopoShape::Edge(e) => visitor.visit_edge(e, shape),
            TopoShape::Wire(w) => visitor.visit_wire(w, shape),
            TopoShape::Face(f) => visitor.visit_face(f, shape),
            TopoShape::Shell(s) => visitor.visit_shell(&s, shape),
            TopoShape::Solid(s) => visitor.visit_solid(&s, shape),
            TopoShape::Compound(c) => visitor.visit_compound(&c, shape),
        }
    }
}

/// Traverse the shape starting from a specific entity, following references.
pub fn traverse_from(shape: &Shape, id: TopoId, visitor: &mut dyn TopoVisitor) {
    if let Some(entity) = shape.get(id) {
        match entity {
            TopoShape::Solid(solid) => {
                visitor.visit_solid(solid, shape);
                if let Some(TopoShape::Shell(shell)) = shape.get(solid.outer_shell) {
                    visitor.visit_shell(shell, shape);
                    for &face_id in &shell.faces {
                        if let Some(TopoShape::Face(face)) = shape.get(face_id) {
                            visitor.visit_face(face, shape);
                            if let Some(wire_id) = face.outer_wire {
                                traverse_wire(shape, wire_id, visitor);
                            }
                            for &wire_id in &face.inner_wires {
                                traverse_wire(shape, wire_id, visitor);
                            }
                        }
                    }
                }
            }
            TopoShape::Compound(compound) => {
                visitor.visit_compound(compound, shape);
                for &child_id in &compound.children {
                    traverse_from(shape, child_id, visitor);
                }
            }
            _ => {}
        }
    }
}

fn traverse_wire(shape: &Shape, wire_id: TopoId, visitor: &mut dyn TopoVisitor) {
    if let Some(TopoShape::Wire(wire)) = shape.get(wire_id) {
        visitor.visit_wire(wire, shape);
        for oriented_edge in &wire.edges {
            if let Some(TopoShape::Edge(edge)) = shape.get(oriented_edge.edge_id) {
                visitor.visit_edge(edge, shape);
                if let Some(TopoShape::Vertex(v)) = shape.get(edge.start_vertex) {
                    visitor.visit_vertex(v, shape);
                }
                if let Some(TopoShape::Vertex(v)) = shape.get(edge.end_vertex) {
                    visitor.visit_vertex(v, shape);
                }
            }
        }
    }
}

/// A simple visitor that counts each type of entity.
#[derive(Debug, Default)]
pub struct CountVisitor {
    pub vertices: usize,
    pub edges: usize,
    pub wires: usize,
    pub faces: usize,
    pub shells: usize,
    pub solids: usize,
    pub compounds: usize,
}

impl TopoVisitor for CountVisitor {
    fn visit_vertex(&mut self, _: &Vertex, _: &Shape) {
        self.vertices += 1;
    }
    fn visit_edge(&mut self, _: &Edge, _: &Shape) {
        self.edges += 1;
    }
    fn visit_wire(&mut self, _: &Wire, _: &Shape) {
        self.wires += 1;
    }
    fn visit_face(&mut self, _: &Face, _: &Shape) {
        self.faces += 1;
    }
    fn visit_shell(&mut self, _: &Shell, _: &Shape) {
        self.shells += 1;
    }
    fn visit_solid(&mut self, _: &Solid, _: &Shape) {
        self.solids += 1;
    }
    fn visit_compound(&mut self, _: &Compound, _: &Shape) {
        self.compounds += 1;
    }
}

/// A visitor that collects all vertex points.
#[derive(Debug, Default)]
pub struct VertexCollector {
    pub points: Vec<draper_geometry::point::Point3>,
}

impl TopoVisitor for VertexCollector {
    fn visit_vertex(&mut self, vertex: &Vertex, _: &Shape) {
        self.points.push(vertex.point);
    }
}
