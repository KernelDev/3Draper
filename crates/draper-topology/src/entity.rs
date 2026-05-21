//! Topological entities — the core of the B-rep model.
//!
//! Each entity has a unique ID and references geometric data.

use draper_geometry::curve::Curve;
use draper_geometry::point::{BoundingBox3, Point3};
use draper_geometry::surface::Surface;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Unique identifier for a topological entity.
pub type TopoId = u64;

/// A vertex — a point in 3D space.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vertex {
    pub id: TopoId,
    pub point: Point3,
    /// Tolerance for vertex position.
    pub tolerance: f64,
}

/// An oriented edge — a curve bounded by two vertices.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    pub id: TopoId,
    /// The geometric curve supporting this edge.
    pub curve: Option<Curve>,
    /// Start vertex.
    pub start_vertex: TopoId,
    /// End vertex.
    pub end_vertex: TopoId,
    /// Parameter range on the curve [t1, t2].
    pub parameter_range: Option<(f64, f64)>,
    /// Orientation: true = same as curve direction, false = reversed.
    pub orientation: bool,
    /// Tolerance.
    pub tolerance: f64,
}

/// An oriented edge reference within a wire.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OrientedEdge {
    pub edge_id: TopoId,
    /// Orientation within the wire.
    pub orientation: bool,
}

/// A wire — a sequence of connected edges forming a closed or open loop.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wire {
    pub id: TopoId,
    /// Ordered list of oriented edges.
    pub edges: Vec<OrientedEdge>,
    /// Whether the wire is closed.
    pub closed: bool,
}

/// A face — a portion of a surface bounded by wires.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Face {
    pub id: TopoId,
    /// The geometric surface supporting this face.
    pub surface: Option<Surface>,
    /// Outer boundary wire (required).
    pub outer_wire: Option<TopoId>,
    /// Inner boundary wires (holes).
    pub inner_wires: Vec<TopoId>,
    /// Orientation: true = normal points outward, false = reversed.
    pub orientation: bool,
}

/// A shell — a collection of connected faces.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Shell {
    pub id: TopoId,
    /// Faces in this shell.
    pub faces: Vec<TopoId>,
    /// Whether the shell is closed (forms a solid boundary).
    pub closed: bool,
}

/// A solid — a closed shell defining a volume.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Solid {
    pub id: TopoId,
    /// The outer shell.
    pub outer_shell: TopoId,
    /// Inner shells (voids/cavities).
    pub inner_shells: Vec<TopoId>,
}

/// A compound — a collection of shapes (for assemblies).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Compound {
    pub id: TopoId,
    /// Child shape IDs.
    pub children: Vec<TopoId>,
}

/// The type of a topological shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ShapeType {
    Vertex,
    Edge,
    Wire,
    Face,
    Shell,
    Solid,
    Compound,
}

/// A topological shape — wrapper around any topological entity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TopoShape {
    Vertex(Vertex),
    Edge(Edge),
    Wire(Wire),
    Face(Face),
    Shell(Shell),
    Solid(Solid),
    Compound(Compound),
}

impl TopoShape {
    pub fn shape_type(&self) -> ShapeType {
        match self {
            TopoShape::Vertex(_) => ShapeType::Vertex,
            TopoShape::Edge(_) => ShapeType::Edge,
            TopoShape::Wire(_) => ShapeType::Wire,
            TopoShape::Face(_) => ShapeType::Face,
            TopoShape::Shell(_) => ShapeType::Shell,
            TopoShape::Solid(_) => ShapeType::Solid,
            TopoShape::Compound(_) => ShapeType::Compound,
        }
    }

    pub fn id(&self) -> TopoId {
        match self {
            TopoShape::Vertex(v) => v.id,
            TopoShape::Edge(e) => e.id,
            TopoShape::Wire(w) => w.id,
            TopoShape::Face(f) => f.id,
            TopoShape::Shell(s) => s.id,
            TopoShape::Solid(s) => s.id,
            TopoShape::Compound(c) => c.id,
        }
    }
}

impl Vertex {
    pub fn new(id: TopoId, point: Point3) -> Self {
        Self {
            id,
            point,
            tolerance: 1e-7,
        }
    }
}

impl Edge {
    pub fn new(
        id: TopoId,
        curve: Option<Curve>,
        start_vertex: TopoId,
        end_vertex: TopoId,
        parameter_range: Option<(f64, f64)>,
    ) -> Self {
        Self {
            id,
            curve,
            start_vertex,
            end_vertex,
            parameter_range,
            orientation: true,
            tolerance: 1e-7,
        }
    }

    /// Create a seam edge (start and end vertex are the same, e.g., on a cylinder).
    pub fn seam(id: TopoId, curve: Option<Curve>, vertex: TopoId, parameter_range: (f64, f64)) -> Self {
        Self {
            id,
            curve,
            start_vertex: vertex,
            end_vertex: vertex,
            parameter_range: Some(parameter_range),
            orientation: true,
            tolerance: 1e-7,
        }
    }

    /// Get the vertices of this edge in the correct order based on orientation.
    pub fn vertices(&self) -> (TopoId, TopoId) {
        if self.orientation {
            (self.start_vertex, self.end_vertex)
        } else {
            (self.end_vertex, self.start_vertex)
        }
    }
}

impl Wire {
    pub fn new(id: TopoId, edges: Vec<OrientedEdge>) -> Self {
        let closed = !edges.is_empty(); // Wires in STEP are typically closed
        Self { id, edges, closed }
    }

    /// Get the number of edges in this wire.
    pub fn len(&self) -> usize {
        self.edges.len()
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }
}

impl Face {
    pub fn new(id: TopoId, surface: Option<Surface>) -> Self {
        Self {
            id,
            surface,
            outer_wire: None,
            inner_wires: Vec::new(),
            orientation: true,
        }
    }

    pub fn with_outer_wire(mut self, wire_id: TopoId) -> Self {
        self.outer_wire = Some(wire_id);
        self
    }

    pub fn with_inner_wire(mut self, wire_id: TopoId) -> Self {
        self.inner_wires.push(wire_id);
        self
    }
}

impl Shell {
    pub fn new(id: TopoId, faces: Vec<TopoId>) -> Self {
        Self {
            id,
            faces,
            closed: true,
        }
    }
}

impl Solid {
    pub fn new(id: TopoId, outer_shell: TopoId) -> Self {
        Self {
            id,
            outer_shell,
            inner_shells: Vec::new(),
        }
    }
}

impl Compound {
    pub fn new(id: TopoId, children: Vec<TopoId>) -> Self {
        Self { id, children }
    }
}
