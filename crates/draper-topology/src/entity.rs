// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
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

use draper_geometry::{Point3d, Point2d, Curve3d, Curve2d, Surface};
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};

/// Global ID counter for topological entities.
static NEXT_ID: AtomicU64 = AtomicU64::new(1);

fn next_id() -> u64 {
    NEXT_ID.fetch_add(1, Ordering::Relaxed)
}

/// Unique identifier for a topological entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TopoId(u64);

impl TopoId {
    pub fn new() -> Self {
        TopoId(next_id())
    }

    /// Get the raw u64 value of this ID.
    pub fn to_u64(self) -> u64 {
        self.0
    }

    /// Reconstruct an ID from its raw u64 value (C5 Stage 4).
    ///
    /// Callers that receive an edge id as a plain number (CLI parameters,
    /// fillet/chamfer selection) use this to resolve the id through the
    /// owning solid's `EdgeStore` — shared-edge instances carry different
    /// instance TopoIds, so a numeric id may name an alias rather than the
    /// canonical edge.
    pub fn from_u64(raw: u64) -> Self {
        TopoId(raw)
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
    /// Authoritative 3D coordinate of the start vertex (from VERTEX_POINT).
    /// When set, overrides curve.point_at(t_min) for the first discretized point.
    /// This ensures bit-identical start points across edges sharing the same
    /// geometric vertex but with different curve parameterizations.
    pub start_vertex_point: Option<Point3d>,
    /// Authoritative 3D coordinate of the end vertex (from VERTEX_POINT).
    pub end_vertex_point: Option<Point3d>,
    /// Whether the edge orientation matches the curve direction.
    pub forward: bool,
    /// Tolerance.
    pub tolerance: f64,
    /// Whether this edge is degenerate (zero-length or degenerate curve).
    /// Set during validation when the edge's curve is found to be degenerate.
    pub degenerate: bool,
    /// STEP entity ID of the underlying EDGE_CURVE.
    /// When two ORIENTED_EDGEs share the same EDGE_CURVE, they get
    /// different TopoIds but the same step_entity_id. This field enables
    /// the unified edge cache to guarantee bit-identical 3D points on
    /// shared edges regardless of which face triggers the discretization.
    /// Set to `None` for edges not originating from STEP files.
    pub step_entity_id: Option<i64>,
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
            start_vertex_point: None,
            end_vertex_point: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
            step_entity_id: None,
        }
    }

    /// Create a linear edge between two points.
    ///
    /// The param_range is set to `(0.0, distance)` where `distance` is the
    /// Euclidean distance between `p1` and `p2`. This ensures that
    /// `point_at(0.0) == p1` and `point_at(1.0) == p2`, because
    /// `Line::through_points` stores a **normalized** direction vector and
    /// `Line::point_at(t)` moves `t` units along that direction.
    pub fn new_line(p1: Point3d, p2: Point3d) -> Self {
        let distance = p1.distance_to(&p2);
        let line = draper_geometry::Line::through_points(p1, p2)
            .unwrap_or_else(|| {
                // Fallback for coincident points: use an arbitrary direction
                // This creates a degenerate edge (zero-length), which will be
                // detected by edge validation later.
                draper_geometry::Line::new(p1, draper_geometry::Direction3d::X)
            });
        let curve = Curve3d::Line(line);
        let mut edge = Self::new(curve, (0.0, distance));
        edge.vertex_start = Some(TopoId::new());
        edge.vertex_end = Some(TopoId::new());
        edge.step_entity_id = None;
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
            start_vertex_point: self.end_vertex_point,
            end_vertex_point: self.start_vertex_point,
            forward: !self.forward,
            tolerance: self.tolerance,
            degenerate: self.degenerate,
            step_entity_id: self.step_entity_id,
        }
    }
}

// ============================================================
// CoEdge (Oriented Edge)
// ============================================================

/// A co-edge — an oriented use of an edge within a wire.
/// Stores the 2D pcurve (parametric curve on the face's surface).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CoEdge {
    pub id: TopoId,
    /// Reference to the parent edge.
    pub edge: TopoId,
    /// Whether the coedge orientation matches the edge orientation.
    pub forward: bool,
    /// 2D pcurve in the parametric space of the face's surface.
    pub pcurve: Option<Pcurve>,
    /// Analytical PCURVE in UV space (if available from STEP).
    /// When present, this is used instead of surface.project_point()
    /// for computing UV coordinates during triangulation.
    pub curve_2d: Option<Curve2d>,
}

impl CoEdge {
    pub fn new(edge: TopoId, forward: bool) -> Self {
        Self {
            id: TopoId::new(),
            edge,
            forward,
            pcurve: None,
            curve_2d: None,
        }
    }
}

/// A 2D parametric curve on a surface (pcurve).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
    /// Canonical edge references into the owning Solid's `EdgeStore` (C5).
    /// Populated by store-first construction (`Solid::from_edges_only`) —
    /// a shared edge carries the SAME canonical id in every incident face.
    /// Instance-faithful edge geometry resolves through
    /// [`Solid::resolve_face_edges`] / [`Solid::instance_edges`] at read
    /// time; a `Face` alone carries NO edge geometry (C5 Stage 7.6b — the
    /// per-face `edges` mirror field is physically removed).
    #[cfg_attr(feature = "serde", serde(default))]
    pub edge_ids: Vec<TopoId>,
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
            edge_ids: Vec::new(),
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
            edge_ids: Vec::new(),
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
            edge_ids: self.edge_ids.clone(),
        }
    }
}

// ============================================================
// Shell
// ============================================================

/// A shell — a connected set of faces forming a closed or open surface.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Shell {
    pub id: TopoId,
    /// Faces in the shell.
    pub faces: Vec<Face>,
    /// Whether the shell is closed (forms a solid boundary).
    pub closed: bool,
    /// Tolerance for the shell (max of all face tolerances).
    ///
    /// Audit item 2.2 (2026-07-19): Added for hierarchical tolerant modeling.
    /// This is the maximum tolerance of any face in the shell, used for
    /// shell-level coincidence checks (e.g., when stitching shells).
    pub tolerance: f64,
}

impl Shell {
    pub fn new(faces: Vec<Face>) -> Self {
        let tolerance = faces.iter().map(|f| f.tolerance).fold(0.0_f64, f64::max);
        Self {
            id: TopoId::new(),
            faces,
            closed: false,
            tolerance,
        }
    }

    /// Create a closed shell.
    pub fn new_closed(faces: Vec<Face>) -> Self {
        let tolerance = faces.iter().map(|f| f.tolerance).fold(0.0_f64, f64::max);
        Self {
            id: TopoId::new(),
            faces,
            closed: true,
            tolerance,
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

/// A solid — a closed shell plus zero or more void shells.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Solid {
    pub id: TopoId,
    /// Outer shell.
    pub outer_shell: Option<Shell>,
    /// Inner shells (voids/cavities).
    pub inner_shells: Vec<Shell>,
    /// Tolerance for the solid (max of all shell tolerances).
    ///
    /// Audit item 2.2 (2026-07-19): Added for hierarchical tolerant modeling.
    pub tolerance: f64,
    /// Canonical edge registry (C5 Stage 2) — single source of truth for
    /// edge identity. Deduplicated by `step_entity_id`; alias mappings let
    /// any instance TopoId resolve to its canonical shared edge.
    ///
    /// C5 Stage 5.1 (2026-08-31): the store is SERIALIZED (flat format,
    /// see `edge_store::serde_impl`) so shared-edge identity survives
    /// round-trips.
    ///
    /// C5 7.6b: the store is the ONLY edge holder (the per-face mirror
    /// field is physically removed). LEGACY payloads (pre-7.6b) carry
    /// per-face `edges` arrays instead — the custom `Deserialize`
    /// implementation (see `edge_store::solid_serde`) captures them
    /// transiently and rebuilds the store via `Solid::rebuild_store`, so
    /// old files load with full edge identity.
    #[cfg_attr(feature = "serde", serde(default))]
    pub edge_store: crate::edge_store::EdgeStore,
}

impl Solid {
    pub fn new(shell: Shell) -> Self {
        let tolerance = shell.tolerance;
        Self {
            id: TopoId::new(),
            outer_shell: Some(shell),
            inner_shells: Vec::new(),
            tolerance,
            edge_store: crate::edge_store::EdgeStore::new(),
        }
    }

    /// Add an inner shell (void/cavity).
    pub fn add_void(&mut self, shell: Shell) {
        self.tolerance = self.tolerance.max(shell.tolerance);
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Shape {
    Vertex(Vertex),
    Edge(Edge),
    Wire(Wire),
    Face(Face),
    Shell(Shell),
    Solid(Solid),
    Compound(Compound),
}
