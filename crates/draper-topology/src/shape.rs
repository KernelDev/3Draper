//! Shape — the top-level container for a B-rep model.
//!
//! A Shape holds all topological entities and provides methods for
//! creating, querying, and modifying the model.

use crate::entity::*;
use draper_geometry::curve::Curve;
use draper_geometry::point::{BoundingBox3, Point3};
use draper_geometry::surface::Surface;
use std::collections::HashMap;

/// The top-level B-rep shape container.
#[derive(Debug, Clone)]
pub struct Shape {
    /// All topological entities indexed by ID.
    pub entities: HashMap<TopoId, TopoShape>,
    /// Next available ID.
    next_id: TopoId,
    /// Root shapes (top-level solids, compounds).
    roots: Vec<TopoId>,
}

impl Shape {
    /// Create an empty shape.
    pub fn new() -> Self {
        Self {
            entities: HashMap::new(),
            next_id: 1,
            roots: Vec::new(),
        }
    }

    /// Allocate a new unique ID.
    fn alloc_id(&mut self) -> TopoId {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    // ---- Creation methods ----

    /// Add a vertex at the given point.
    pub fn add_vertex(&mut self, point: Point3) -> TopoId {
        let id = self.alloc_id();
        let vertex = Vertex::new(id, point);
        self.entities.insert(id, TopoShape::Vertex(vertex));
        id
    }

    /// Add an edge between two vertices.
    pub fn add_edge(
        &mut self,
        curve: Option<Curve>,
        start_vertex: TopoId,
        end_vertex: TopoId,
        parameter_range: Option<(f64, f64)>,
    ) -> TopoId {
        let id = self.alloc_id();
        let edge = Edge::new(id, curve, start_vertex, end_vertex, parameter_range);
        self.entities.insert(id, TopoShape::Edge(edge));
        id
    }

    /// Add a wire from oriented edges.
    pub fn add_wire(&mut self, edges: Vec<OrientedEdge>) -> TopoId {
        let id = self.alloc_id();
        let wire = Wire::new(id, edges);
        self.entities.insert(id, TopoShape::Wire(wire));
        id
    }

    /// Add a face with an optional surface.
    pub fn add_face(&mut self, surface: Option<Surface>) -> TopoId {
        let id = self.alloc_id();
        let face = Face::new(id, surface);
        self.entities.insert(id, TopoShape::Face(face));
        id
    }

    /// Add a shell from faces.
    pub fn add_shell(&mut self, faces: Vec<TopoId>) -> TopoId {
        let id = self.alloc_id();
        let shell = Shell::new(id, faces);
        self.entities.insert(id, TopoShape::Shell(shell));
        id
    }

    /// Add a solid with an outer shell.
    pub fn add_solid(&mut self, outer_shell: TopoId) -> TopoId {
        let id = self.alloc_id();
        let solid = Solid::new(id, outer_shell);
        self.entities.insert(id, TopoShape::Solid(solid));
        self.roots.push(id);
        id
    }

    /// Add a compound shape.
    pub fn add_compound(&mut self, children: Vec<TopoId>) -> TopoId {
        let id = self.alloc_id();
        let compound = Compound::new(id, children);
        self.entities.insert(id, TopoShape::Compound(compound));
        self.roots.push(id);
        id
    }

    // ---- Query methods ----

    /// Get a shape by ID.
    pub fn get(&self, id: TopoId) -> Option<&TopoShape> {
        self.entities.get(&id)
    }

    /// Get a mutable shape by ID.
    pub fn get_mut(&mut self, id: TopoId) -> Option<&mut TopoShape> {
        self.entities.get_mut(&id)
    }

    /// Get all root shapes.
    pub fn roots(&self) -> &[TopoId] {
        &self.roots
    }

    /// Set the root shapes.
    pub fn set_roots(&mut self, roots: Vec<TopoId>) {
        self.roots = roots;
    }

    /// Get all entities of a given type.
    pub fn find_by_type(&self, shape_type: ShapeType) -> Vec<&TopoShape> {
        self.entities
            .values()
            .filter(|s| s.shape_type() == shape_type)
            .collect()
    }

    /// Get all vertices.
    pub fn vertices(&self) -> Vec<&Vertex> {
        self.entities
            .values()
            .filter_map(|s| {
                if let TopoShape::Vertex(v) = s {
                    Some(v)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get all edges.
    pub fn edges(&self) -> Vec<&Edge> {
        self.entities
            .values()
            .filter_map(|s| {
                if let TopoShape::Edge(e) = s {
                    Some(e)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get all faces.
    pub fn faces(&self) -> Vec<&Face> {
        self.entities
            .values()
            .filter_map(|s| {
                if let TopoShape::Face(f) = s {
                    Some(f)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get all shells.
    pub fn shells(&self) -> Vec<&Shell> {
        self.entities
            .values()
            .filter_map(|s| {
                if let TopoShape::Shell(sh) = s {
                    Some(sh)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get all solids.
    pub fn solids(&self) -> Vec<&Solid> {
        self.entities
            .values()
            .filter_map(|s| {
                if let TopoShape::Solid(solid) = s {
                    Some(solid)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Compute the bounding box of the entire shape.
    pub fn bounding_box(&self) -> BoundingBox3 {
        let mut bb = BoundingBox3::empty();
        for vertex in self.vertices() {
            bb.extend(vertex.point);
        }
        bb
    }

    /// Get the number of entities.
    pub fn len(&self) -> usize {
        self.entities.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entities.is_empty()
    }

    /// Update the outer wire of a face.
    pub fn set_face_outer_wire(&mut self, face_id: TopoId, wire_id: TopoId) {
        if let Some(TopoShape::Face(face)) = self.entities.get_mut(&face_id) {
            face.outer_wire = Some(wire_id);
        }
    }

    /// Add an inner wire to a face.
    pub fn add_face_inner_wire(&mut self, face_id: TopoId, wire_id: TopoId) {
        if let Some(TopoShape::Face(face)) = self.entities.get_mut(&face_id) {
            face.inner_wires.push(wire_id);
        }
    }
}

impl Default for Shape {
    fn default() -> Self {
        Self::new()
    }
}
