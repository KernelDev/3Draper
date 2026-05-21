//! Topological entities — the core of the B-rep model.
//!
//! Each entity has a unique ID and references geometric data.
//! The topology follows the STEP B-rep hierarchy:
//! Vertex → Edge → Wire → Face → Shell → Solid → Compound
//!
//! Key design principles (from the triangulation guide):
//! - Edges carry both 3D curves AND 2D pcurves (one per face)
//! - Seam edges are explicitly marked for periodic surfaces
//! - Face carries UV domain bounds for proper parameterization
//! - Tolerance is tracked at every level for healing/validation

use draper_geometry::curve::Curve;
use draper_geometry::pcurve::PCurveOnFace;
use draper_geometry::point::{Point2, Point3};
use draper_geometry::surface::Surface;
use serde::{Deserialize, Serialize};

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
///
/// An edge may be shared by multiple faces. For each face it belongs to,
/// it has a corresponding pcurve (2D curve in the face's UV parameter space).
/// This is critical for constrained Delaunay triangulation in UV space.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    pub id: TopoId,
    /// The 3D geometric curve supporting this edge.
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
    /// Whether this edge is a seam on a periodic surface.
    /// A seam edge connects the same vertex at u_min and u_max (or v_min/v_max).
    /// In UV space, it appears as two separate curves at opposite boundaries.
    pub is_seam: bool,
    /// PCurves for this edge, one per face it borders.
    /// Key = face_id, Value = pcurve information.
    pub pcurves: Vec<PCurveOnFace>,
}

/// An oriented edge reference within a wire.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OrientedEdge {
    pub edge_id: TopoId,
    /// Orientation within the wire.
    pub orientation: bool,
}

/// A wire — a sequence of connected edges forming a closed or open loop.
///
/// Wires represent boundaries of faces. The outer wire goes CCW,
/// inner wires (holes) go CW when viewed from outside the face.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wire {
    pub id: TopoId,
    /// Ordered list of oriented edges.
    pub edges: Vec<OrientedEdge>,
    /// Whether the wire is closed.
    pub closed: bool,
    /// Whether this wire is a seam wire (on a periodic surface boundary).
    pub is_seam: bool,
}

/// A face — a portion of a surface bounded by wires.
///
/// The face carries both the 3D surface and UV domain information.
/// The outer wire defines the outer boundary; inner wires define holes.
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
    /// UV parameter bounds for the face's surface.
    /// This defines the valid parameter domain for triangulation.
    pub uv_bounds: Option<UVBounds>,
    /// Whether this face is on a periodic surface (has seam edges).
    pub has_seam: bool,
}

/// UV parameter bounds for a face.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct UVBounds {
    pub u_min: f64,
    pub u_max: f64,
    pub v_min: f64,
    pub v_max: f64,
}

impl UVBounds {
    pub fn new(u_min: f64, u_max: f64, v_min: f64, v_max: f64) -> Self {
        Self { u_min, u_max, v_min, v_max }
    }

    pub fn u_range(&self) -> f64 {
        self.u_max - self.u_min
    }

    pub fn v_range(&self) -> f64 {
        self.v_max - self.v_min
    }

    /// Check if a UV point is within these bounds.
    pub fn contains(&self, u: f64, v: f64) -> bool {
        u >= self.u_min && u <= self.u_max && v >= self.v_min && v <= self.v_max
    }

    /// Compute the center of the UV domain.
    pub fn center(&self) -> Point2 {
        Point2::new(
            (self.u_min + self.u_max) / 2.0,
            (self.v_min + self.v_max) / 2.0,
        )
    }
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
            is_seam: false,
            pcurves: Vec::new(),
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
            is_seam: true,
            pcurves: Vec::new(),
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

    /// Add a pcurve for a specific face.
    pub fn add_pcurve(&mut self, pcurve: PCurveOnFace) {
        // Remove existing pcurve for the same face if present
        self.pcurves.retain(|pc| pc.face_id != pcurve.face_id);
        self.pcurves.push(pcurve);
    }

    /// Get the pcurve for a specific face.
    pub fn get_pcurve(&self, face_id: TopoId) -> Option<&PCurveOnFace> {
        self.pcurves.iter().find(|pc| pc.face_id == face_id)
    }

    /// Check if this edge is degenerate (zero length).
    pub fn is_degenerate(&self) -> bool {
        self.start_vertex == self.end_vertex
    }
}

impl Wire {
    pub fn new(id: TopoId, edges: Vec<OrientedEdge>) -> Self {
        let closed = !edges.is_empty(); // Wires in STEP are typically closed
        Self { id, edges, closed, is_seam: false }
    }

    /// Create a seam wire.
    pub fn seam_wire(id: TopoId, edges: Vec<OrientedEdge>) -> Self {
        Self { id, edges, closed: true, is_seam: true }
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
            uv_bounds: None,
            has_seam: false,
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

    pub fn with_uv_bounds(mut self, bounds: UVBounds) -> Self {
        self.uv_bounds = Some(bounds);
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
