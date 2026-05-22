//! B-Rep topological entities.
//!
//! The topology hierarchy:
//! - Solid (collection of shells)
//!   - Shell (closed collection of faces)
//!     - Face (region of a surface bounded by wires)
//!       - Wire (ordered sequence of coedges)
//!         - CoEdge (oriented edge use within a wire)
//!           - Edge (curve segment between two vertices)
//!             - Vertex (point in 3D space)

use draper_geometry::{Point3d, Point2d, Curve3d, Surface};
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};

/// Global ID counter for topological entities.
static NEXT_ID: AtomicU64 = AtomicU64::new(1);

fn next_id() -> u64 {
    NEXT_ID.fetch_add(1, Ordering::Relaxed)
}

/// Unique identifier for a topological entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TopoId(u64);

impl TopoId {
    pub fn new() -> Self {
        TopoId(next_id())
    }
}

impl fmt::Display for TopoId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "#{}", self.0)
    }
}

// ============================================================
// Vertex
// ============================================================

/// A vertex — a point in 3D space.
#[derive(Clone, Debug)]
pub struct Vertex {
    pub id: TopoId,
    pub point: Point3d,
    /// Tolerance for merging vertices.
    pub tolerance: f64,
}

impl Vertex {
    pub fn new(point: Point3d) -> Self {
        Self {
            id: TopoId::new(),
            point,
            tolerance: 1e-6,
        }
    }
}

// ============================================================
// Edge
// ============================================================

/// An edge — a curve segment between two vertices.
#[derive(Clone, Debug)]
pub struct Edge {
    pub id: TopoId,
    /// The 3D curve geometry.
    pub curve: Option<Curve3d>,
    /// Parametric range on the curve [t_min, t_max].
    pub param_range: (f64, f64),
    /// Start vertex.
    pub vertex_start: Option<TopoId>,
    /// End vertex.
    pub vertex_end: Option<TopoId>,
    /// Whether the edge orientation matches the curve direction.
    pub forward: bool,
    /// Tolerance.
    pub tolerance: f64,
}

impl Edge {
    /// Create a new edge with a curve and parametric range.
    pub fn new(curve: Curve3d, param_range: (f64, f64)) -> Self {
        Self {
            id: TopoId::new(),
            curve: Some(curve),
            param_range,
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
        }
    }

    /// Create a linear edge between two points.
    pub fn new_line(p1: Point3d, p2: Point3d) -> Self {
        let curve = Curve3d::Line(draper_geometry::Line::through_points(p1, p2).unwrap());
        let mut edge = Self::new(curve, (0.0, 1.0));
        edge.vertex_start = Some(TopoId::new());
        edge.vertex_end = Some(TopoId::new());
        edge
    }

    /// Evaluate the edge at parameter t in [0, 1].
    pub fn point_at(&self, t: f64) -> Option<Point3d> {
        self.curve.as_ref().map(|c| {
            let (tmin, tmax) = self.param_range;
            let param = tmin + t * (tmax - tmin);
            c.point_at(param)
        })
    }

    /// Start point of the edge.
    pub fn start_point(&self) -> Option<Point3d> {
        self.point_at(0.0)
    }

    /// End point of the edge.
    pub fn end_point(&self) -> Option<Point3d> {
        self.point_at(1.0)
    }

    /// Reversed edge (same geometry, opposite direction).
    pub fn reversed(&self) -> Edge {
        Edge {
            id: self.id,
            curve: self.curve.clone(),
            param_range: (self.param_range.1, self.param_range.0),
            vertex_start: self.vertex_end,
            vertex_end: self.vertex_start,
            forward: !self.forward,
            tolerance: self.tolerance,
        }
    }
}

// ============================================================
// CoEdge (Oriented Edge)
// ============================================================

/// A co-edge — an oriented use of an edge within a wire.
/// Stores the 2D pcurve (parametric curve on the face's surface).
#[derive(Clone, Debug)]
pub struct CoEdge {
    pub id: TopoId,
    /// Reference to the parent edge.
    pub edge: TopoId,
    /// Whether the coedge orientation matches the edge orientation.
    pub forward: bool,
    /// 2D pcurve in the parametric space of the face's surface.
    pub pcurve: Option<Pcurve>,
}

impl CoEdge {
    pub fn new(edge: TopoId, forward: bool) -> Self {
        Self {
            id: TopoId::new(),
            edge,
            forward,
            pcurve: None,
        }
    }
}

/// A 2D parametric curve on a surface (pcurve).
#[derive(Clone, Debug)]
pub struct Pcurve {
    /// 2D polyline approximation in (u, v) space.
    pub polyline_2d: Vec<Point2d>,
}

impl Pcurve {
    pub fn new(polyline: Vec<Point2d>) -> Self {
        Self { polyline_2d: polyline }
    }

    /// Create a linear pcurve between two 2D points.
    pub fn linear(p1: Point2d, p2: Point2d) -> Self {
        Self { polyline_2d: vec![p1, p2] }
    }
}

// ============================================================
// Wire
// ============================================================

/// A wire — an ordered sequence of coedges forming a closed or open loop.
#[derive(Clone, Debug)]
pub struct Wire {
    pub id: TopoId,
    /// Ordered coedges.
    pub coedges: Vec<CoEdge>,
    /// Whether the wire is a closed loop.
    pub closed: bool,
}

impl Wire {
    pub fn new(coedges: Vec<CoEdge>) -> Self {
        let closed = coedges.len() > 1; // Will be validated later
        Self {
            id: TopoId::new(),
            coedges,
            closed,
        }
    }

    /// Number of edges in the wire.
    pub fn len(&self) -> usize {
        self.coedges.len()
    }

    pub fn is_empty(&self) -> bool {
        self.coedges.is_empty()
    }
}

// ============================================================
// Face
// ============================================================

/// A face — a region of a surface bounded by wires.
#[derive(Clone, Debug)]
pub struct Face {
    pub id: TopoId,
    /// The surface geometry.
    pub surface: Option<Surface>,
    /// Outer boundary wire (required).
    pub outer_wire: Option<Wire>,
    /// Inner boundary wires (holes).
    pub inner_wires: Vec<Wire>,
    /// Whether the face normal matches the surface normal.
    pub forward: bool,
    /// Tolerance.
    pub tolerance: f64,
    /// Cached 3D boundary points for triangulation.
    /// Populated by builders that have access to edge geometry.
    pub boundary_points: Vec<draper_geometry::Point3d>,
}

impl Face {
    /// Create a face with a surface and outer wire.
    pub fn new(surface: Surface, outer_wire: Wire) -> Self {
        Self {
            id: TopoId::new(),
            surface: Some(surface),
            outer_wire: Some(outer_wire),
            inner_wires: Vec::new(),
            forward: true,
            tolerance: 1e-6,
            boundary_points: Vec::new(),
        }
    }

    /// Create a planar face from a surface only (no wires — infinite face).
    pub fn new_surface_only(surface: Surface) -> Self {
        Self {
            id: TopoId::new(),
            surface: Some(surface),
            outer_wire: None,
            inner_wires: Vec::new(),
            forward: true,
            tolerance: 1e-6,
            boundary_points: Vec::new(),
        }
    }

    /// Add an inner wire (hole).
    pub fn add_hole(&mut self, wire: Wire) {
        self.inner_wires.push(wire);
    }

    /// Reversed face (normal points inward).
    pub fn reversed(&self) -> Face {
        Face {
            id: self.id,
            surface: self.surface.clone(),
            outer_wire: self.outer_wire.clone(),
            inner_wires: self.inner_wires.clone(),
            forward: !self.forward,
            tolerance: self.tolerance,
            boundary_points: self.boundary_points.clone(),
        }
    }
}

// ============================================================
// Shell
// ============================================================

/// A shell — a connected set of faces forming a closed or open surface.
#[derive(Clone, Debug)]
pub struct Shell {
    pub id: TopoId,
    /// Faces in the shell.
    pub faces: Vec<Face>,
    /// Whether the shell is closed (forms a solid boundary).
    pub closed: bool,
}

impl Shell {
    pub fn new(faces: Vec<Face>) -> Self {
        Self {
            id: TopoId::new(),
            faces,
            closed: false,
        }
    }

    /// Create a closed shell.
    pub fn new_closed(faces: Vec<Face>) -> Self {
        Self {
            id: TopoId::new(),
            faces,
            closed: true,
        }
    }

    pub fn len(&self) -> usize {
        self.faces.len()
    }

    pub fn is_empty(&self) -> bool {
        self.faces.is_empty()
    }
}

// ============================================================
// Solid
// ============================================================

/// A solid — a closed 3D region bounded by shells.
#[derive(Clone, Debug)]
pub struct Solid {
    pub id: TopoId,
    /// Outer shell.
    pub outer_shell: Option<Shell>,
    /// Inner shells (voids/cavities).
    pub inner_shells: Vec<Shell>,
}

impl Solid {
    pub fn new(shell: Shell) -> Self {
        Self {
            id: TopoId::new(),
            outer_shell: Some(shell),
            inner_shells: Vec::new(),
        }
    }

    /// Add an inner shell (void/cavity).
    pub fn add_void(&mut self, shell: Shell) {
        self.inner_shells.push(shell);
    }

    /// Get all faces from all shells.
    pub fn faces(&self) -> Vec<&Face> {
        let mut faces = Vec::new();
        if let Some(ref shell) = self.outer_shell {
            faces.extend(shell.faces.iter());
        }
        for shell in &self.inner_shells {
            faces.extend(shell.faces.iter());
        }
        faces
    }

    /// Get all faces mutably.
    pub fn faces_mut(&mut self) -> Vec<&mut Face> {
        let mut faces = Vec::new();
        if let Some(ref mut shell) = self.outer_shell {
            faces.extend(shell.faces.iter_mut());
        }
        for shell in &mut self.inner_shells {
            faces.extend(shell.faces.iter_mut());
        }
        faces
    }
}

// ============================================================
// Compound
// ============================================================

/// A compound — a collection of solids (assembly).
#[derive(Clone, Debug)]
pub struct Compound {
    pub id: TopoId,
    pub solids: Vec<Solid>,
    pub compounds: Vec<Compound>,
}

impl Compound {
    pub fn new() -> Self {
        Self {
            id: TopoId::new(),
            solids: Vec::new(),
            compounds: Vec::new(),
        }
    }

    pub fn add_solid(&mut self, solid: Solid) {
        self.solids.push(solid);
    }

    pub fn add_compound(&mut self, compound: Compound) {
        self.compounds.push(compound);
    }
}

/// Top-level shape that can contain any topological entity.
#[derive(Clone, Debug)]
pub enum Shape {
    Vertex(Vertex),
    Edge(Edge),
    Wire(Wire),
    Face(Face),
    Shell(Shell),
    Solid(Solid),
    Compound(Compound),
}
