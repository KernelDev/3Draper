// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Specialized workspaces — Phase 9.
//!
//! Visual Programming, Surface Modeling, Sheet Metal, CAM, FEA, Drawing,
//! Assembly, Point Cloud, Mold, AI.

use std::collections::HashMap;

// ============================================================
// 9.1. Visual Programming (Node Graph) — Grasshopper-Inspired
// ============================================================

/// Typed data that flows between VP nodes.
/// Each port on a node carries one of these types.
#[derive(Clone, Debug)]
pub enum VpData {
    /// 3D solid geometry (box, sphere, boolean result, etc.)
    Geometry(Box<draper_topology::Solid>),
    /// Triangle mesh (from mesh boolean operations)
    Mesh(Box<draper_mesh::TriangleMesh>),
    /// A 2D/3D polyline curve (sampled points)
    Curve(Vec<draper_geometry::Point3d>),
    /// Parametric curve — preserves Line/Circle/Arc/NurbsCurve type info.
    Curve3d(Box<draper_geometry::Curve3d>),
    /// Parametric surface — Plane/Cylinder/Sphere/NurbsSurface/etc.
    Surface(Box<draper_geometry::Surface>),
    /// Infinite plane reference (origin + normal + x_dir).
    PlaneRef(Box<draper_geometry::Plane>),
    /// 4×4 transformation matrix.
    Transform(Box<draper_geometry::Transform>),
    /// Floating-point number
    Number(f64),
    /// Whole number
    Integer(i64),
    /// True/False
    Boolean(bool),
    /// 3D point [x, y, z]
    Point([f64; 3]),
    /// 3D vector [x, y, z]
    Vector([f64; 3]),
    /// Text string
    String(String),
    /// Interval [min, max]
    Domain { min: f64, max: f64 },
    /// RGBA color
    Color([f64; 4]),
    /// List of items (data tree leaf)
    List(Vec<VpData>),
    /// No data yet (not computed)
    Empty,
}

/// Port data type — used for type-checking connections.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PortType {
    Geometry,
    Curve,
    Curve3d,
    Surface,
    PlaneRef,
    Mesh,
    Transform,
    Number,
    Integer,
    Boolean,
    Point,
    Vector,
    String,
    Domain,
    Color,
    List,
    Any, // Accepts any type (for Bake, Panel, etc.)
}

impl PortType {
    /// Check if this port type can accept data of another type.
    pub fn accepts(&self, other: &PortType) -> bool {
        if *self == PortType::Any || *other == PortType::Any { return true; }
        if *self == *other { return true; }
        // Number accepts Integer (promotion)
        if *self == PortType::Number && *other == PortType::Integer { return true; }
        if *self == PortType::Integer && *other == PortType::Number { return true; }
        // Point accepts Vector and vice versa
        if *self == PortType::Point && *other == PortType::Vector { return true; }
        if *self == PortType::Vector && *other == PortType::Point { return true; }
        // Curve3d accepts Curve (sampled) and vice versa
        if *self == PortType::Curve3d && *other == PortType::Curve { return true; }
        if *self == PortType::Curve && *other == PortType::Curve3d { return true; }
        // Surface accepts PlaneRef
        if *self == PortType::Surface && *other == PortType::PlaneRef { return true; }
        // Geometry accepts Surface, Mesh
        if *self == PortType::Geometry && (*other == PortType::Surface || *other == PortType::Mesh) { return true; }
        // List accepts anything
        if *self == PortType::List { return true; }
        if *other == PortType::List { return true; }
        false
    }

    pub fn color(&self) -> egui::Color32 {
        match self {
            PortType::Geometry => egui::Color32::from_rgb(0x89, 0xb4, 0xfa),
            PortType::Curve => egui::Color32::from_rgb(0xa6, 0xe3, 0xa1),
            PortType::Curve3d => egui::Color32::from_rgb(0xa6, 0xe3, 0xa1),
            PortType::Surface => egui::Color32::from_rgb(0x94, 0xe2, 0xd5),
            PortType::PlaneRef => egui::Color32::from_rgb(0x94, 0xe2, 0xd5),
            PortType::Mesh => egui::Color32::from_rgb(0x89, 0xd8, 0xb4),
            PortType::Transform => egui::Color32::from_rgb(0xf5, 0xc2, 0xe7),
            PortType::Number => egui::Color32::from_rgb(0xf9, 0xe2, 0xaf),
            PortType::Integer => egui::Color32::from_rgb(0xeb, 0xa0, 0xac),
            PortType::Boolean => egui::Color32::from_rgb(0xf5, 0xc2, 0xe7),
            PortType::Point => egui::Color32::from_rgb(0xfa, 0xb3, 0x87),
            PortType::Vector => egui::Color32::from_rgb(0x94, 0xe2, 0xd5),
            PortType::String => egui::Color32::from_rgb(0xba, 0xc2, 0xde),
            PortType::Domain => egui::Color32::from_rgb(0xf9, 0xe2, 0xaf),
            PortType::Color => egui::Color32::from_rgb(0xf5, 0xa0, 0xa0),
            PortType::List => egui::Color32::from_rgb(0xcb, 0xa6, 0xf7),
            PortType::Any => egui::Color32::from_rgb(0x6c, 0x70, 0x86),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            PortType::Geometry => "Geometry",
            PortType::Curve => "Curve",
            PortType::Curve3d => "Curve3d",
            PortType::Surface => "Surface",
            PortType::PlaneRef => "Plane",
            PortType::Mesh => "Mesh",
            PortType::Transform => "Transform",
            PortType::Number => "Number",
            PortType::Integer => "Integer",
            PortType::Boolean => "Boolean",
            PortType::Point => "Point",
            PortType::Vector => "Vector",
            PortType::String => "String",
            PortType::Domain => "Domain",
            PortType::Color => "Color",
            PortType::List => "List",
            PortType::Any => "Any",
        }
    }
}

/// Mirror plane for the Mirror transform node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MirrorPlane {
    /// Mirror across the YZ plane (negate X).
    YZ,
    /// Mirror across the XZ plane (negate Y).
    XZ,
    /// Mirror across the XY plane (negate Z).
    XY,
}

impl MirrorPlane {
    pub fn label(&self) -> &'static str {
        match self {
            MirrorPlane::YZ => "YZ",
            MirrorPlane::XZ => "XZ",
            MirrorPlane::XY => "XY",
        }
    }
}

/// Port descriptor — name + type.
#[derive(Clone, Debug)]
pub struct PortDesc {
    pub name: &'static str,
    pub port_type: PortType,
}

/// Node type for the visual programming editor.
#[derive(Clone, Debug)]
pub enum NodeType {
    // ─── Params (Input Parameters) ───
    /// Number slider — draggable float value.
    NumberSlider { value: f64, min: f64, max: f64 },
    /// Integer input.
    IntegerInput { value: i64 },
    /// Boolean toggle.
    BooleanToggle { value: bool },
    /// 3D Point parameter.
    PointInput { x: f64, y: f64, z: f64 },
    /// 3D Vector parameter.
    VectorInput { x: f64, y: f64, z: f64 },
    /// Text panel — displays data.
    Panel,

    // ─── Maths ───
    Add,
    Subtract,
    Multiply,
    Divide,
    Sin,
    Cos,
    Tan,
    Abs,
    Sqrt,
    Pow,
    Round,
    Min,
    Max,
    Average,
    Expression { expr: String },

    // ─── Sets (List Operations) ───
    Series { start: f64, step: f64, count: u32 },
    Range { domain_min: f64, domain_max: f64, count: u32 },
    ListLength,
    ListItem,
    Reverse,
    Sort,
    CullPattern,

    // ─── Primitives (Geometry Creation) ───
    /// Box primitive.
    Box { width: f64, height: f64, depth: f64 },
    /// Sphere primitive.
    Sphere { radius: f64 },
    /// Cylinder primitive.
    Cylinder { radius: f64, height: f64 },
    /// Cone primitive.
    Cone { bottom_radius: f64, top_radius: f64, height: f64 },
    /// Torus primitive.
    Torus { major_radius: f64, minor_radius: f64 },

    // ─── Curve ───
    Line,
    Circle { radius: f64 },
    DivideCurve { count: u32 },
    EvaluateCurve,
    CurveLength,

    // ─── Transform ───
    /// Move (translate) geometry by a vector. Default X=0, Y=0, Z=0.
    Move { x: f64, y: f64, z: f64 },
    /// Rotate geometry by Euler angles (X, Y, Z) in degrees.
    Rotate { x_deg: f64, y_deg: f64, z_deg: f64 },
    /// Scale geometry by non-uniform factors (X, Y, Z).
    Scale { x: f64, y: f64, z: f64 },
    /// Mirror geometry across a plane (XY, XZ, or YZ).
    Mirror { plane: MirrorPlane },
    LinearArray { count: u32, spacing: f64 },
    CircularArray { count: u32, angle: f64 },

    // ─── Intersect (Boolean) ───
    /// Boolean union.
    BooleanUnion,
    /// Boolean subtract.
    BooleanSubtract,
    /// Boolean intersect.
    BooleanIntersect,

    // ─── Modify ───
    /// Fillet operation.
    Fillet { radius: f64 },
    /// Chamfer operation.
    Chamfer { distance: f64 },

    // ─── Data Tree Operations (Phase 6) ───
    /// Graft: wraps each item in a list into its own single-item list.
    Graft,
    /// Flatten: concatenates all sub-lists into a single flat list.
    Flatten,
    /// Cross-reference: cartesian product of two lists.
    CrossRef,
    /// Shift List: shifts list items by N positions.
    ShiftList { amount: i32 },
    /// Subset: extracts a sub-list from start to end index.
    Subset { start: u32, count: u32 },
    /// Dispatch: splits a list into two based on a boolean pattern.
    Dispatch,
    /// Weave: interleaves two lists into one.
    Weave,
    /// Concat: concatenates two lists.
    Concat,

    // ─── Output ───
    /// Bake to document.
    BakeToDoc,

    // ───── Phase A: New Param Nodes ─────
    /// Plane input (origin + normal).
    PlaneInput { ox: f64, oy: f64, oz: f64, nx: f64, ny: f64, nz: f64 },
    /// Domain input [min, max].
    DomainInput { min: f64, max: f64 },
    /// String text input.
    StringInput { text: String },

    // ───── Phase B: Analysis Nodes ─────
    /// Compute volume of a solid.
    Volume,
    /// Compute surface area of a solid.
    SurfaceArea,
    /// Compute centroid of a solid.
    Centroid,
    /// Compute bounding box of geometry.
    BoundingBox,
    /// Distance between two points.
    Distance,
    /// Angle between two vectors.
    Angle,
    /// Mass properties (volume, area, centroid, moments).
    MassProperties,

    // ───── Phase C: Vector & Math Nodes ─────
    /// Vector cross product.
    Cross,
    /// Vector dot product.
    Dot,
    /// Vector length.
    VectorLength,
    /// Vector unit (normalize).
    Unit,
    /// Vector reverse (negate).
    Negative,
    /// Reciprocal (1/x).
    Reciprocal,
    /// Arcsin.
    Asin,
    /// Arccos.
    Acos,
    /// Arctan.
    Atan,
    /// Arctan2(y, x).
    Atan2,
    /// Logarithm (base 10).
    Log,
    /// Natural logarithm.
    Ln,
    /// Exponential (e^x).
    Exp,
    /// Modulus (a % b).
    Modulus,
    /// Map value from one domain to another.
    MapDomain { source_min: f64, source_max: f64, target_min: f64, target_max: f64 },
    /// Point from two points (midpoint).
    PointMidpoint,
    /// Linear interpolation between two points.
    PointLerp { t: f64 },
    /// Vector from two points.
    Vector2pt,

    // ───── Phase D: Surface Creation Nodes ─────
    /// Extrude a curve along a vector to create a solid.
    Extrude { distance: f64 },
    /// Revolve a curve around an axis to create a solid.
    Revolve { angle_deg: f64 },
    /// Loft through multiple curves.
    Loft,
    /// Sweep a profile along a path.
    Sweep,
    /// Create a ruled surface between two curves.
    RuledSurface,
    /// Create a plane surface (finite patch).
    PlaneSurface,
    /// Extrude to a point (tapered).
    ExtrudePoint,
    /// Extrude with taper angle.
    ExtrudeTapered { distance: f64, taper_deg: f64 },

    // ───── Phase E: Surface Eval & Modify ─────
    /// Evaluate surface at (u, v).
    EvaluateSurface,
    /// Surface normal at (u, v).
    SurfaceNormal,
    /// Offset a solid by a distance (shell/thicken).
    Shell { thickness: f64 },
    /// Thicken a surface into a solid.
    Thicken { thickness: f64 },
    /// Offset solid surfaces.
    OffsetSolid { distance: f64 },
    /// Split solid with a plane.
    SplitSolid,
    /// Trim solid with a cutting surface.
    TrimSolid,
    /// Create a hole in a solid.
    Hole { radius: f64, depth: f64 },

    // ───── Phase E (extended): Surface Evaluation & Modification ─────
    /// Local frame at (u, v): origin + 3 axes from surface derivatives.
    SurfaceFrame,
    /// Principal curvatures & directions at (u, v).
    SurfaceCurvature,
    /// Surface area via numerical quadrature over UV domain.
    SurfaceAreaUV { u_samples: u32, v_samples: u32 },
    /// Extract a sub-patch of a surface [u0, u1] × [v0, v1].
    IsoTrim { u0: f64, u1: f64, v0: f64, v1: f64 },
    /// Grid of points/normals on surface.
    DivideSurface { u_count: u32, v_count: u32 },
    /// Find UV of closest point on surface to a 3D point.
    SurfaceClosestPoint,
    /// Project points onto surface along a direction.
    SurfaceProjectPoint { direction: [f64; 3] },
    /// Split surface at U or V iso-curve at given parameter.
    SurfaceSplit { parameter: f64 },
    /// Reverse surface normal direction.
    SurfaceFlip,
    /// Refit surface to a NURBS of given degree/counts.
    SurfaceRebuild { u_degree: u32, v_degree: u32, u_count: u32, v_count: u32 },
    /// Build NURBS surface from a control-point grid.
    SurfaceFromPoints { u_count: u32, v_count: u32, degree: u32 },
    /// Extract an iso-curve from a surface at given U or V parameter.
    SurfaceIsocurve { direction: u32 }, // 0 = U-iso, 1 = V-iso
    /// Remove all trims from a surface (return underlying surface).
    SurfaceUntrim,
    /// Trim a surface with closed curves in UV space.
    SurfaceTrim,

    // ───── Phase E (extended): Solid Modification ─────
    /// Apply draft angle to a face (useful for molded parts).
    DraftFace { angle_deg: f64 },
    /// Translate a planar face of a solid by a vector.
    MoveFace,
    /// Offset a planar face of a solid by a distance.
    OffsetFace { distance: f64 },
    /// Replace a planar face of a solid with a different surface.
    ReplaceFace,
    /// Drill a circular hole at point C along axis N.
    HoleCircular { radius: f64 },
    /// Add a rib feature: extrude profile, union with base.
    Rib { thickness: f64 },
    /// Fillet specific edges (by index) of a solid.
    FilletEdge { radius: f64 },
    /// Chamfer specific edges (by index) of a solid.
    ChamferEdge { distance: f64 },
    /// Fillet with per-edge radius.
    FilletVariable,

    // ───── Phase F: Curve Nodes ─────
    /// Arc by center, radius, start/end angles.
    Arc { radius: f64, start_angle: f64, end_angle: f64 },
    /// Arc by 3 points.
    Arc3pt,
    /// Ellipse by radii.
    Ellipse { rx: f64, ry: f64 },
    /// Polyline from points.
    Polyline,
    /// NURBS curve from control points.
    NurbsCurve,
    /// NURBS curve through points (interpolation).
    NurbsCurveInterp,
    /// Join multiple curves into one.
    JoinCurves,
    /// Offset a curve.
    CurveOffset { distance: f64 },
    /// Extend a curve.
    Extend { length: f64 },
    /// Flip curve direction.
    Flip,
    /// Rebuild curve with different point count.
    Rebuild { count: u32 },
    /// Point on curve at parameter t.
    PointAt,
    /// Tangent vector on curve at t.
    Tangent,
    /// Curve curvature at t.
    Curvature,
    /// Nearest point on curve to a test point.
    NearestPoint,
    /// Curve endpoints (start + end).
    EndPoints,
    /// Split curve at parameter.
    SplitCurve { t: f64 },

    // ───── Phase G: Transform Nodes ─────
    /// Rotate around arbitrary axis.
    RotateAxis { angle_deg: f64 },
    /// Mirror across an arbitrary plane.
    MirrorPlane,
    /// Orient geometry from source plane to target plane.
    Orient,
    /// Project geometry onto a plane.
    Project,
    /// Array along a curve.
    ArrayAlongCurve { count: u32 },
    /// Array in a box pattern (Nx × Ny × Nz).
    ArrayBox { nx: u32, ny: u32, nz: u32 },
    /// Array on a surface (Nx × Ny).
    ArrayOnSurface { nx: u32, ny: u32 },
    /// Compose multiple transforms into one.
    ComposeTransform,
    /// Invert a transform.
    InvertTransform,

    // ───── Phase H: Intersect Nodes ─────
    /// Curve-curve intersection.
    CurveCurveIntersect,
    /// Curve-surface intersection.
    CurveSurfaceIntersect,
    /// Surface-surface intersection.
    SurfaceSurfaceIntersect,
    /// Plane-plane intersection (line).
    PlanePlaneIntersect,
    /// Curve-plane intersection.
    CurvePlaneIntersect,
    /// Solid-plane intersection (section).
    SolidPlaneIntersect,
    /// Boolean split (A split by B → above + below).
    BooleanSplit,

    // ───── Phase I: Mesh Nodes ─────
    /// Convert geometry to mesh.
    ToMesh,
    /// Mesh from box/sphere/cylinder.
    MeshPrimitive,
    /// Mesh area.
    MeshArea,
    /// Mesh volume.
    MeshVolume,
    /// Flip mesh normals.
    MeshFlip,
    /// Weld mesh vertices.
    MeshWeld { tolerance: f64 },
    /// Subdivide mesh.
    MeshSubdivide { iterations: u32 },
    /// Decimate (simplify) mesh.
    MeshDecimate { target_ratio: f64 },
    /// Smooth mesh.
    MeshSmooth { iterations: u32 },

    // ───── Phase H (extended): Sets / Tree Operations ─────
    /// Remove items at the given indices.
    CullIndex { wrap: bool },
    /// Split list into chunks of `size`.
    Partition { size: u32 },
    /// Replace items at indices `I` with values `V`.
    ReplaceItems,
    /// Like Dispatch but retains index positions in each output.
    Sift,
    /// Alternating interleave from two lists (vs. Concat which appends).
    Combine,
    /// Replicate a single item N times.
    Duplicate { count: u32 },
    /// Emit a null placeholder for list-padding operations.
    NullItem,
    /// Remap data-tree paths.
    PathMapper { target_path: Vec<i32> },
    /// Extract a single branch by path (index in list).
    TreeBranch,
    /// List all paths and item counts in a tree (list).
    TreeStatistics,
    /// Strip null items and empty branches.
    CleanTree { remove_nulls: bool, remove_empty: bool },
    /// Convert a tree (list of lists) into N separate list outputs.
    ExplodeTree,

    // ───── Phase H (extended): Output — Bake / Export ─────
    /// Bake geometry into a named layer with optional color override.
    BakeToLayer { layer_name: String, color: Option<[f64; 4]> },
    /// Bake a mesh into the document.
    BakeMesh { layer_name: String },
    /// Bake a curve as a wire in the document.
    BakeCurve { layer_name: String },
    /// Export geometry to STEP file.
    ExportSTEP { path: String },
    /// Export geometry/mesh to STL file.
    ExportSTL { path: String, binary: bool },
    /// Export geometry/mesh to OBJ file.
    ExportOBJ { path: String },
    /// Group multiple solids into one named collection.
    Group { name: String },
    /// Wrap a sub-graph as a reusable node.
    Cluster { graph_path: String },
}

impl NodeType {
    pub fn label(&self) -> &'static str {
        match self {
            // Params
            NodeType::NumberSlider { .. } => "Number",
            NodeType::IntegerInput { .. } => "Integer",
            NodeType::BooleanToggle { .. } => "Boolean",
            NodeType::PointInput { .. } => "Point",
            NodeType::VectorInput { .. } => "Vector",
            NodeType::Panel => "Panel",
            // Maths
            NodeType::Add => "Add (+)",
            NodeType::Subtract => "Subtract (-)",
            NodeType::Multiply => "Multiply (*)",
            NodeType::Divide => "Divide (/)",
            NodeType::Sin => "Sin",
            NodeType::Cos => "Cos",
            NodeType::Tan => "Tan",
            NodeType::Abs => "Abs",
            NodeType::Pow => "Pow",
            NodeType::Round => "Round",
            NodeType::Sqrt => "Sqrt",
            NodeType::Min => "Min",
            NodeType::Max => "Max",
            NodeType::Average => "Average",
            NodeType::Expression { .. } => "Expression",
            // Sets
            NodeType::Series { .. } => "Series",
            NodeType::Range { .. } => "Range",
            NodeType::ListLength => "List Length",
            NodeType::ListItem => "List Item",
            NodeType::Reverse => "Reverse",
            NodeType::Sort => "Sort",
            NodeType::CullPattern => "Cull Pattern",
            NodeType::Sort => "Sort",
            NodeType::CullPattern => "Cull Pattern",
            // Primitives
            NodeType::Box { .. } => "Box",
            NodeType::Sphere { .. } => "Sphere",
            NodeType::Cylinder { .. } => "Cylinder",
            NodeType::Cone { .. } => "Cone",
            NodeType::Torus { .. } => "Torus",
            // Curve
            NodeType::Line => "Line",
            NodeType::Circle { .. } => "Circle",
            NodeType::DivideCurve { .. } => "Divide Curve",
            NodeType::EvaluateCurve => "Evaluate Curve",
            NodeType::CurveLength => "Curve Length",
            NodeType::EvaluateCurve => "Evaluate Curve",
            NodeType::CurveLength => "Curve Length",
            // Transform
            NodeType::Move { .. } => "Move",
            NodeType::Rotate { .. } => "Rotate",
            NodeType::Scale { .. } => "Scale",
            NodeType::Mirror { .. } => "Mirror",
            NodeType::LinearArray { .. } => "Linear Array",
            NodeType::CircularArray { .. } => "Circular Array",
            // Boolean
            NodeType::BooleanUnion => "Union",
            NodeType::BooleanSubtract => "Subtract",
            NodeType::BooleanIntersect => "Intersect",
            // Modify
            NodeType::Fillet { .. } => "Fillet",
            NodeType::Chamfer { .. } => "Chamfer",
            // Data Tree
            NodeType::Graft => "Graft",
            NodeType::Flatten => "Flatten",
            NodeType::CrossRef => "Cross Reference",
            NodeType::ShiftList { .. } => "Shift List",
            NodeType::Subset { .. } => "Subset",
            NodeType::Dispatch => "Dispatch",
            NodeType::Weave => "Weave",
            NodeType::Concat => "Concat",
            // Output
            NodeType::BakeToDoc => "Bake to Doc",
            // New nodes
            NodeType::PlaneInput { .. } => "Plane",
            NodeType::DomainInput { .. } => "Domain",
            NodeType::StringInput { .. } => "String",
            NodeType::Volume => "Volume",
            NodeType::SurfaceArea => "Surface Area",
            NodeType::Centroid => "Centroid",
            NodeType::BoundingBox => "Bounding Box",
            NodeType::Distance => "Distance",
            NodeType::Angle => "Angle",
            NodeType::MassProperties => "Mass Properties",
            NodeType::Cross => "Cross",
            NodeType::Dot => "Dot",
            NodeType::VectorLength => "Vector Length",
            NodeType::Unit => "Unit",
            NodeType::Negative => "Negative",
            NodeType::Reciprocal => "Reciprocal",
            NodeType::Asin => "Asin",
            NodeType::Acos => "Acos",
            NodeType::Atan => "Atan",
            NodeType::Atan2 => "Atan2",
            NodeType::Log => "Log",
            NodeType::Ln => "Ln",
            NodeType::Exp => "Exp",
            NodeType::Modulus => "Modulus",
            NodeType::MapDomain { .. } => "Map Domain",
            NodeType::PointMidpoint => "Midpoint",
            NodeType::PointLerp { .. } => "Point Lerp",
            NodeType::Vector2pt => "Vector 2pt",
            NodeType::Extrude { .. } => "Extrude",
            NodeType::Revolve { .. } => "Revolve",
            NodeType::Loft => "Loft",
            NodeType::Sweep => "Sweep",
            NodeType::RuledSurface => "Ruled Surface",
            NodeType::PlaneSurface => "Plane Surface",
            NodeType::ExtrudePoint => "Extrude Point",
            NodeType::ExtrudeTapered { .. } => "Extrude Tapered",
            NodeType::EvaluateSurface => "Evaluate Surface",
            NodeType::SurfaceNormal => "Surface Normal",
            NodeType::Shell { .. } => "Shell",
            NodeType::Thicken { .. } => "Thicken",
            NodeType::OffsetSolid { .. } => "Offset Solid",
            NodeType::SplitSolid => "Split Solid",
            NodeType::TrimSolid => "Trim Solid",
            NodeType::Hole { .. } => "Hole",
            // Phase E (extended): Surface Eval & Modify
            NodeType::SurfaceFrame => "Surface Frame",
            NodeType::SurfaceCurvature => "Surface Curvature",
            NodeType::SurfaceAreaUV { .. } => "Surface Area (UV)",
            NodeType::IsoTrim { .. } => "Iso Trim",
            NodeType::DivideSurface { .. } => "Divide Surface",
            NodeType::SurfaceClosestPoint => "Surface Closest Point",
            NodeType::SurfaceProjectPoint { .. } => "Surface Project Point",
            NodeType::SurfaceSplit { .. } => "Surface Split",
            NodeType::SurfaceFlip => "Surface Flip",
            NodeType::SurfaceRebuild { .. } => "Surface Rebuild",
            NodeType::SurfaceFromPoints { .. } => "Surface From Points",
            NodeType::SurfaceIsocurve { .. } => "Surface Isocurve",
            NodeType::SurfaceUntrim => "Surface Untrim",
            NodeType::SurfaceTrim => "Surface Trim",
            // Phase E (extended): Solid Modification
            NodeType::DraftFace { .. } => "Draft Face",
            NodeType::MoveFace => "Move Face",
            NodeType::OffsetFace { .. } => "Offset Face",
            NodeType::ReplaceFace => "Replace Face",
            NodeType::HoleCircular { .. } => "Hole Circular",
            NodeType::Rib { .. } => "Rib",
            NodeType::FilletEdge { .. } => "Fillet Edge",
            NodeType::ChamferEdge { .. } => "Chamfer Edge",
            NodeType::FilletVariable => "Fillet Variable",
            NodeType::Arc { .. } => "Arc",
            NodeType::Arc3pt => "Arc 3pt",
            NodeType::Ellipse { .. } => "Ellipse",
            NodeType::Polyline => "Polyline",
            NodeType::NurbsCurve => "NURBS Curve",
            NodeType::NurbsCurveInterp => "NURBS Interp",
            NodeType::JoinCurves => "Join Curves",
            NodeType::CurveOffset { .. } => "Curve Offset",
            NodeType::Extend { .. } => "Extend",
            NodeType::Flip => "Flip",
            NodeType::Rebuild { .. } => "Rebuild",
            NodeType::PointAt => "Point At",
            NodeType::Tangent => "Tangent",
            NodeType::Curvature => "Curvature",
            NodeType::NearestPoint => "Nearest Point",
            NodeType::EndPoints => "End Points",
            NodeType::SplitCurve { .. } => "Split Curve",
            NodeType::RotateAxis { .. } => "Rotate Axis",
            NodeType::MirrorPlane => "Mirror Plane",
            NodeType::Orient => "Orient",
            NodeType::Project => "Project",
            NodeType::ArrayAlongCurve { .. } => "Array Along Curve",
            NodeType::ArrayBox { .. } => "Array Box",
            NodeType::ArrayOnSurface { .. } => "Array on Surface",
            NodeType::ComposeTransform => "Compose",
            NodeType::InvertTransform => "Invert Transform",
            NodeType::CurveCurveIntersect => "CCX",
            NodeType::CurveSurfaceIntersect => "CSX",
            NodeType::SurfaceSurfaceIntersect => "SSX",
            NodeType::PlanePlaneIntersect => "Plane-Plane",
            NodeType::CurvePlaneIntersect => "Curve-Plane",
            NodeType::SolidPlaneIntersect => "Solid-Plane",
            NodeType::BooleanSplit => "Boolean Split",
            NodeType::ToMesh => "To Mesh",
            NodeType::MeshPrimitive => "Mesh Primitive",
            NodeType::MeshArea => "Mesh Area",
            NodeType::MeshVolume => "Mesh Volume",
            NodeType::MeshFlip => "Mesh Flip",
            NodeType::MeshWeld { .. } => "Mesh Weld",
            NodeType::MeshSubdivide { .. } => "Mesh Subdivide",
            NodeType::MeshDecimate { .. } => "Mesh Decimate",
            NodeType::MeshSmooth { .. } => "Mesh Smooth",
            // Phase H (extended): Sets/Tree
            NodeType::CullIndex { .. } => "Cull Index",
            NodeType::Partition { .. } => "Partition",
            NodeType::ReplaceItems => "Replace Items",
            NodeType::Sift => "Sift",
            NodeType::Combine => "Combine",
            NodeType::Duplicate { .. } => "Duplicate",
            NodeType::NullItem => "Null",
            NodeType::PathMapper { .. } => "Path Mapper",
            NodeType::TreeBranch => "Tree Branch",
            NodeType::TreeStatistics => "Tree Statistics",
            NodeType::CleanTree { .. } => "Clean Tree",
            NodeType::ExplodeTree => "Explode Tree",
            // Phase H (extended): Output
            NodeType::BakeToLayer { .. } => "Bake To Layer",
            NodeType::BakeMesh { .. } => "Bake Mesh",
            NodeType::BakeCurve { .. } => "Bake Curve",
            NodeType::ExportSTEP { .. } => "Export STEP",
            NodeType::ExportSTL { .. } => "Export STL",
            NodeType::ExportOBJ { .. } => "Export OBJ",
            NodeType::Group { .. } => "Group",
            NodeType::Cluster { .. } => "Cluster",
        }
    }

    pub fn category(&self) -> &'static str {
        match self {
            NodeType::NumberSlider { .. } | NodeType::IntegerInput { .. } |
            NodeType::BooleanToggle { .. } | NodeType::PointInput { .. } |
            NodeType::VectorInput { .. } | NodeType::Panel => "Params",
            NodeType::Add | NodeType::Subtract | NodeType::Multiply | NodeType::Divide |
            NodeType::Sin | NodeType::Cos | NodeType::Tan | NodeType::Abs |
            NodeType::Sqrt | NodeType::Pow | NodeType::Round |
            NodeType::Min | NodeType::Max | NodeType::Average | NodeType::Expression { .. } => "Maths",
            NodeType::Series { .. } | NodeType::Range { .. } | NodeType::ListLength |
            NodeType::ListItem | NodeType::Reverse | NodeType::Sort | NodeType::CullPattern => "Sets",
            NodeType::Box { .. } | NodeType::Sphere { .. } | NodeType::Cylinder { .. } |
            NodeType::Cone { .. } | NodeType::Torus { .. } => "Primitives",
            NodeType::Line | NodeType::Circle { .. } | NodeType::DivideCurve { .. } |
            NodeType::EvaluateCurve | NodeType::CurveLength => "Curve",
            NodeType::Move { .. } | NodeType::Rotate { .. } | NodeType::Scale { .. } |
            NodeType::Mirror { .. } | NodeType::LinearArray { .. } | NodeType::CircularArray { .. } => "Transform",
            NodeType::BooleanUnion | NodeType::BooleanSubtract | NodeType::BooleanIntersect => "Boolean",
            NodeType::Fillet { .. } | NodeType::Chamfer { .. } => "Modify",
            NodeType::Graft | NodeType::Flatten | NodeType::CrossRef |
            NodeType::ShiftList { .. } | NodeType::Subset { .. } |
            NodeType::Dispatch | NodeType::Weave | NodeType::Concat => "Tree",
            NodeType::BakeToDoc => "Output",
            // New categories
            NodeType::PlaneInput { .. } | NodeType::DomainInput { .. } | NodeType::StringInput { .. } => "Params",
            NodeType::Volume | NodeType::SurfaceArea | NodeType::Centroid | NodeType::BoundingBox |
            NodeType::Distance | NodeType::Angle | NodeType::MassProperties => "Analysis",
            NodeType::Cross | NodeType::Dot | NodeType::VectorLength | NodeType::Unit |
            NodeType::Negative | NodeType::Reciprocal | NodeType::Asin | NodeType::Acos |
            NodeType::Atan | NodeType::Atan2 | NodeType::Log | NodeType::Ln |
            NodeType::Exp | NodeType::Modulus | NodeType::MapDomain { .. } |
            NodeType::PointMidpoint | NodeType::PointLerp { .. } | NodeType::Vector2pt => "Maths",
            NodeType::Extrude { .. } | NodeType::Revolve { .. } | NodeType::Loft |
            NodeType::Sweep | NodeType::RuledSurface | NodeType::PlaneSurface |
            NodeType::ExtrudePoint | NodeType::ExtrudeTapered { .. } => "Surface",
            NodeType::EvaluateSurface | NodeType::SurfaceNormal |
            NodeType::Shell { .. } | NodeType::Thicken { .. } | NodeType::OffsetSolid { .. } |
            NodeType::SplitSolid | NodeType::TrimSolid | NodeType::Hole { .. } |
            // Phase E (extended): Surface Eval & Modify
            NodeType::SurfaceFrame | NodeType::SurfaceCurvature |
            NodeType::SurfaceAreaUV { .. } | NodeType::IsoTrim { .. } |
            NodeType::DivideSurface { .. } | NodeType::SurfaceClosestPoint |
            NodeType::SurfaceProjectPoint { .. } | NodeType::SurfaceSplit { .. } |
            NodeType::SurfaceFlip | NodeType::SurfaceRebuild { .. } |
            NodeType::SurfaceFromPoints { .. } | NodeType::SurfaceIsocurve { .. } |
            NodeType::SurfaceUntrim | NodeType::SurfaceTrim |
            // Phase E (extended): Solid Modification
            NodeType::DraftFace { .. } | NodeType::MoveFace |
            NodeType::OffsetFace { .. } | NodeType::ReplaceFace |
            NodeType::HoleCircular { .. } | NodeType::Rib { .. } |
            NodeType::FilletEdge { .. } | NodeType::ChamferEdge { .. } |
            NodeType::FilletVariable => "Modify",
            NodeType::Arc { .. } | NodeType::Arc3pt | NodeType::Ellipse { .. } |
            NodeType::Polyline | NodeType::NurbsCurve | NodeType::NurbsCurveInterp |
            NodeType::JoinCurves | NodeType::CurveOffset { .. } | NodeType::Extend { .. } |
            NodeType::Flip | NodeType::Rebuild { .. } | NodeType::PointAt |
            NodeType::Tangent | NodeType::Curvature | NodeType::NearestPoint |
            NodeType::EndPoints | NodeType::SplitCurve { .. } => "Curve",
            NodeType::RotateAxis { .. } | NodeType::MirrorPlane | NodeType::Orient |
            NodeType::Project | NodeType::ArrayAlongCurve { .. } | NodeType::ArrayBox { .. } |
            NodeType::ArrayOnSurface { .. } | NodeType::ComposeTransform | NodeType::InvertTransform => "Transform",
            NodeType::CurveCurveIntersect | NodeType::CurveSurfaceIntersect |
            NodeType::SurfaceSurfaceIntersect | NodeType::PlanePlaneIntersect |
            NodeType::CurvePlaneIntersect | NodeType::SolidPlaneIntersect | NodeType::BooleanSplit => "Intersect",
            NodeType::ToMesh | NodeType::MeshPrimitive | NodeType::MeshArea | NodeType::MeshVolume |
            NodeType::MeshFlip | NodeType::MeshWeld { .. } | NodeType::MeshSubdivide { .. } |
            NodeType::MeshDecimate { .. } | NodeType::MeshSmooth { .. } => "Mesh",
            // Phase H (extended): Sets/Tree categories
            NodeType::CullIndex { .. } | NodeType::Partition { .. } | NodeType::ReplaceItems |
            NodeType::Sift | NodeType::Combine | NodeType::Duplicate { .. } | NodeType::NullItem |
            NodeType::PathMapper { .. } | NodeType::TreeBranch | NodeType::TreeStatistics |
            NodeType::CleanTree { .. } | NodeType::ExplodeTree => "Tree",
            // Phase H (extended): Output categories
            NodeType::BakeToLayer { .. } | NodeType::BakeMesh { .. } | NodeType::BakeCurve { .. } |
            NodeType::ExportSTEP { .. } | NodeType::ExportSTL { .. } | NodeType::ExportOBJ { .. } |
            NodeType::Group { .. } | NodeType::Cluster { .. } => "Output",
        }
    }

    pub fn icon(&self) -> &'static str {
        match self {
            NodeType::NumberSlider { .. } => "#",
            NodeType::IntegerInput { .. } => "I",
            NodeType::BooleanToggle { .. } => "T/F",
            NodeType::PointInput { .. } => "P",
            NodeType::VectorInput { .. } => "V",
            NodeType::Panel => "[]",
            NodeType::Add => "+",
            NodeType::Subtract => "-",
            NodeType::Multiply => "*",
            NodeType::Divide => "/",
            NodeType::Sin | NodeType::Cos | NodeType::Tan | NodeType::Abs | NodeType::Sqrt | NodeType::Pow | NodeType::Round => "f(x)",
            NodeType::Min | NodeType::Max | NodeType::Average => "min",
            NodeType::Expression { .. } => "expr",
            NodeType::Series { .. } => "S",
            NodeType::Range { .. } => "Rng",
            NodeType::ListLength => "len",
            NodeType::ListItem => "[]i",
            NodeType::Reverse => "rev",
            NodeType::Sort => "sort",
            NodeType::CullPattern => "cull",
            NodeType::Box { .. } => "B",
            NodeType::Sphere { .. } => "S",
            NodeType::Cylinder { .. } => "C",
            NodeType::Cone { .. } => "Co",
            NodeType::Torus { .. } => "T",
            NodeType::Line => "L",
            NodeType::Circle { .. } => "O",
            NodeType::DivideCurve { .. } => "Div",
            NodeType::EvaluateCurve => "Eval",
            NodeType::CurveLength => "Len",
            NodeType::Move { .. } => "Mv",
            NodeType::Rotate { .. } => "Rot",
            NodeType::Scale { .. } => "Sc",
            NodeType::Mirror { .. } => "Mir",
            NodeType::LinearArray { .. } => "Arr",
            NodeType::CircularArray { .. } => "Cir",
            NodeType::BooleanUnion => "U",
            NodeType::BooleanSubtract => "Sub",
            NodeType::BooleanIntersect => "Int",
            NodeType::Fillet { .. } => "Fil",
            NodeType::Chamfer { .. } => "Chm",
            NodeType::Graft => "Gr",
            NodeType::Flatten => "Fl",
            NodeType::CrossRef => "XR",
            NodeType::ShiftList { .. } => "Sh",
            NodeType::Subset { .. } => "Sub",
            NodeType::Dispatch => "Dp",
            NodeType::Weave => "Wv",
            NodeType::Concat => "Cat",
            NodeType::BakeToDoc => "Bake",
            _ => "?",
        }
    }

    /// Input port descriptors (name + type) for this node.
    pub fn input_ports(&self) -> Vec<PortDesc> {
        match self {
            // Params — no inputs
            NodeType::NumberSlider { .. } | NodeType::IntegerInput { .. } |
            NodeType::BooleanToggle { .. } | NodeType::PointInput { .. } |
            NodeType::VectorInput { .. } | NodeType::Series { .. } | NodeType::Range { .. } |
            NodeType::Box { .. } | NodeType::Sphere { .. } | NodeType::Cylinder { .. } |
            NodeType::Cone { .. } | NodeType::Torus { .. } | NodeType::Circle { .. } => vec![],
            // Panel — accepts any
            NodeType::Panel => vec![PortDesc { name: "D", port_type: PortType::Any }],
            // Maths — 2 Number inputs
            NodeType::Add | NodeType::Subtract | NodeType::Multiply | NodeType::Divide |
            NodeType::Min | NodeType::Max => vec![
                PortDesc { name: "A", port_type: PortType::Number },
                PortDesc { name: "B", port_type: PortType::Number },
            ],
            // Single-input maths
            NodeType::Sin | NodeType::Cos | NodeType::Tan | NodeType::Abs |
            NodeType::Sqrt | NodeType::Round => vec![
                PortDesc { name: "X", port_type: PortType::Number },
            ],
            NodeType::Pow => vec![
                PortDesc { name: "B", port_type: PortType::Number },
                PortDesc { name: "E", port_type: PortType::Number },
            ],
            NodeType::Average => vec![PortDesc { name: "L", port_type: PortType::List }],
            NodeType::Expression { .. } => vec![PortDesc { name: "X", port_type: PortType::Number }],
            // Sets
            NodeType::ListLength => vec![PortDesc { name: "L", port_type: PortType::List }],
            NodeType::ListItem => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "I", port_type: PortType::Integer },
            ],
            NodeType::Reverse | NodeType::Sort => vec![PortDesc { name: "L", port_type: PortType::List }],
            NodeType::CullPattern => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "P", port_type: PortType::List },
            ],
            // Curve
            NodeType::Line => vec![
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "B", port_type: PortType::Point },
            ],
            NodeType::DivideCurve { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "N", port_type: PortType::Integer },
            ],
            NodeType::EvaluateCurve => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "T", port_type: PortType::Number },
            ],
            NodeType::CurveLength => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            // Transform — 1 Geometry + optional Vector/Number
            NodeType::Move { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "V", port_type: PortType::Vector },
            ],
            NodeType::Rotate { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "X", port_type: PortType::Number },
                PortDesc { name: "Y", port_type: PortType::Number },
                PortDesc { name: "Z", port_type: PortType::Number },
            ],
            NodeType::Scale { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "X", port_type: PortType::Number },
                PortDesc { name: "Y", port_type: PortType::Number },
                PortDesc { name: "Z", port_type: PortType::Number },
            ],
            NodeType::Mirror { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::LinearArray { .. } | NodeType::CircularArray { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            // Boolean — 2 Geometry inputs
            NodeType::BooleanUnion | NodeType::BooleanSubtract | NodeType::BooleanIntersect => vec![
                PortDesc { name: "A", port_type: PortType::Geometry },
                PortDesc { name: "B", port_type: PortType::Geometry },
            ],
            // Modify — 1 Geometry
            NodeType::Fillet { .. } | NodeType::Chamfer { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            // Data Tree
            NodeType::Graft | NodeType::Flatten => vec![
                PortDesc { name: "L", port_type: PortType::List },
            ],
            NodeType::CrossRef | NodeType::Weave | NodeType::Concat => vec![
                PortDesc { name: "A", port_type: PortType::List },
                PortDesc { name: "B", port_type: PortType::List },
            ],
            NodeType::ShiftList { .. } => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "N", port_type: PortType::Integer },
            ],
            NodeType::Subset { .. } => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "S", port_type: PortType::Integer },
                PortDesc { name: "C", port_type: PortType::Integer },
            ],
            NodeType::Dispatch => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "P", port_type: PortType::List },
            ],
            // Output
            NodeType::BakeToDoc => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
            // ── Params ──
            NodeType::PlaneInput { .. } | NodeType::DomainInput { .. } | NodeType::StringInput { .. } => vec![],
            // ── Analysis ──
            NodeType::Volume | NodeType::SurfaceArea | NodeType::Centroid |
            NodeType::BoundingBox | NodeType::MassProperties => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::Distance => vec![
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "B", port_type: PortType::Point },
            ],
            NodeType::Angle => vec![
                PortDesc { name: "A", port_type: PortType::Vector },
                PortDesc { name: "B", port_type: PortType::Vector },
            ],
            // ── Vector & Math ──
            NodeType::Cross => vec![
                PortDesc { name: "A", port_type: PortType::Vector },
                PortDesc { name: "B", port_type: PortType::Vector },
            ],
            NodeType::Dot => vec![
                PortDesc { name: "A", port_type: PortType::Vector },
                PortDesc { name: "B", port_type: PortType::Vector },
            ],
            NodeType::VectorLength | NodeType::Unit | NodeType::Negative => vec![
                PortDesc { name: "V", port_type: PortType::Vector },
            ],
            NodeType::Reciprocal | NodeType::Asin | NodeType::Acos | NodeType::Atan |
            NodeType::Log | NodeType::Ln | NodeType::Exp | NodeType::Negative => vec![
                PortDesc { name: "X", port_type: PortType::Number },
            ],
            NodeType::Atan2 | NodeType::Modulus => vec![
                PortDesc { name: "A", port_type: PortType::Number },
                PortDesc { name: "B", port_type: PortType::Number },
            ],
            NodeType::MapDomain { .. } => vec![
                PortDesc { name: "V", port_type: PortType::Number },
            ],
            NodeType::PointMidpoint => vec![
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "B", port_type: PortType::Point },
            ],
            NodeType::PointLerp { .. } => vec![
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "B", port_type: PortType::Point },
            ],
            NodeType::Vector2pt => vec![
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "B", port_type: PortType::Point },
            ],
            // ── Surface Creation ──
            NodeType::Extrude { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "V", port_type: PortType::Vector },
            ],
            NodeType::Revolve { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "D", port_type: PortType::Vector },
            ],
            NodeType::Loft => vec![
                PortDesc { name: "C", port_type: PortType::List },
            ],
            NodeType::Sweep => vec![
                PortDesc { name: "P", port_type: PortType::Curve },
                PortDesc { name: "R", port_type: PortType::Curve },
            ],
            NodeType::RuledSurface => vec![
                PortDesc { name: "A", port_type: PortType::Curve },
                PortDesc { name: "B", port_type: PortType::Curve },
            ],
            NodeType::PlaneSurface => vec![
                PortDesc { name: "P", port_type: PortType::PlaneRef },
            ],
            NodeType::ExtrudePoint => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "P", port_type: PortType::Point },
            ],
            NodeType::ExtrudeTapered { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "V", port_type: PortType::Vector },
            ],
            // ── Surface Eval & Modify ──
            NodeType::EvaluateSurface | NodeType::SurfaceNormal => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "U", port_type: PortType::Number },
                PortDesc { name: "V", port_type: PortType::Number },
            ],
            NodeType::Shell { .. } | NodeType::Thicken { .. } | NodeType::OffsetSolid { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::SplitSolid => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "P", port_type: PortType::PlaneRef },
            ],
            NodeType::TrimSolid => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::Hole { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "P", port_type: PortType::Point },
                PortDesc { name: "D", port_type: PortType::Vector },
            ],
            // ── Phase E (extended): Surface Eval ──
            NodeType::SurfaceFrame => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "U", port_type: PortType::Number },
                PortDesc { name: "V", port_type: PortType::Number },
            ],
            NodeType::SurfaceCurvature => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "U", port_type: PortType::Number },
                PortDesc { name: "V", port_type: PortType::Number },
            ],
            NodeType::SurfaceAreaUV { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::IsoTrim { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "D", port_type: PortType::Domain },
            ],
            NodeType::DivideSurface { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceClosestPoint => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "P", port_type: PortType::Point },
            ],
            NodeType::SurfaceProjectPoint { .. } => vec![
                PortDesc { name: "P", port_type: PortType::List },
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceSplit { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceFlip => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceRebuild { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceFromPoints { .. } => vec![
                PortDesc { name: "P", port_type: PortType::List },
            ],
            NodeType::SurfaceIsocurve { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "T", port_type: PortType::Number },
            ],
            NodeType::SurfaceUntrim => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceTrim => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "C", port_type: PortType::List },
            ],
            // ── Phase E (extended): Solid Modification ──
            NodeType::DraftFace { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "F", port_type: PortType::Integer },
                PortDesc { name: "D", port_type: PortType::Vector },
            ],
            NodeType::MoveFace => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "F", port_type: PortType::Integer },
                PortDesc { name: "V", port_type: PortType::Vector },
            ],
            NodeType::OffsetFace { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "F", port_type: PortType::Integer },
            ],
            NodeType::ReplaceFace => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "F", port_type: PortType::Integer },
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::HoleCircular { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "C", port_type: PortType::Point },
                PortDesc { name: "N", port_type: PortType::Vector },
            ],
            NodeType::Rib { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "P", port_type: PortType::Curve },
                PortDesc { name: "D", port_type: PortType::Vector },
            ],
            NodeType::FilletEdge { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "E", port_type: PortType::List },
            ],
            NodeType::ChamferEdge { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "E", port_type: PortType::List },
            ],
            NodeType::FilletVariable => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "E", port_type: PortType::List },
            ],
            // ── Curve ──
            NodeType::Arc { .. } | NodeType::Ellipse { .. } => vec![],
            NodeType::Arc3pt => vec![
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "B", port_type: PortType::Point },
                PortDesc { name: "C", port_type: PortType::Point },
            ],
            NodeType::Polyline => vec![
                PortDesc { name: "P", port_type: PortType::List },
            ],
            NodeType::NurbsCurve | NodeType::NurbsCurveInterp => vec![
                PortDesc { name: "P", port_type: PortType::List },
            ],
            NodeType::JoinCurves => vec![
                PortDesc { name: "C", port_type: PortType::List },
            ],
            NodeType::CurveOffset { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "P", port_type: PortType::PlaneRef },
            ],
            NodeType::Extend { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::Flip | NodeType::Rebuild { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::PointAt | NodeType::Tangent | NodeType::Curvature => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "T", port_type: PortType::Number },
            ],
            NodeType::NearestPoint => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "P", port_type: PortType::Point },
            ],
            NodeType::EndPoints => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::SplitCurve { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "T", port_type: PortType::Number },
            ],
            // ── Transform ──
            NodeType::RotateAxis { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "D", port_type: PortType::Vector },
            ],
            NodeType::MirrorPlane => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "P", port_type: PortType::PlaneRef },
            ],
            NodeType::Orient => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "S", port_type: PortType::PlaneRef },
                PortDesc { name: "T", port_type: PortType::PlaneRef },
            ],
            NodeType::Project => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "P", port_type: PortType::PlaneRef },
                PortDesc { name: "D", port_type: PortType::Vector },
            ],
            NodeType::ArrayAlongCurve { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::ArrayBox { .. } | NodeType::ArrayOnSurface { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::ComposeTransform => vec![
                PortDesc { name: "A", port_type: PortType::Transform },
                PortDesc { name: "B", port_type: PortType::Transform },
            ],
            NodeType::InvertTransform => vec![
                PortDesc { name: "T", port_type: PortType::Transform },
            ],
            // ── Intersect ──
            NodeType::CurveCurveIntersect => vec![
                PortDesc { name: "A", port_type: PortType::Curve },
                PortDesc { name: "B", port_type: PortType::Curve },
            ],
            NodeType::CurveSurfaceIntersect => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceSurfaceIntersect => vec![
                PortDesc { name: "A", port_type: PortType::Surface },
                PortDesc { name: "B", port_type: PortType::Surface },
            ],
            NodeType::PlanePlaneIntersect => vec![
                PortDesc { name: "A", port_type: PortType::PlaneRef },
                PortDesc { name: "B", port_type: PortType::PlaneRef },
            ],
            NodeType::CurvePlaneIntersect => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "P", port_type: PortType::PlaneRef },
            ],
            NodeType::SolidPlaneIntersect => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "P", port_type: PortType::PlaneRef },
            ],
            NodeType::BooleanSplit => vec![
                PortDesc { name: "A", port_type: PortType::Geometry },
                PortDesc { name: "B", port_type: PortType::Geometry },
            ],
            // ── Mesh ──
            NodeType::ToMesh => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::MeshPrimitive => vec![],
            NodeType::MeshArea | NodeType::MeshVolume | NodeType::MeshFlip |
            NodeType::MeshWeld { .. } | NodeType::MeshSubdivide { .. } |
            NodeType::MeshDecimate { .. } | NodeType::MeshSmooth { .. } => vec![
                PortDesc { name: "M", port_type: PortType::Mesh },
            ],
            // Phase H (extended): Sets/Tree inputs
            NodeType::CullIndex { .. } => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "I", port_type: PortType::List },
            ],
            NodeType::Partition { .. } => vec![
                PortDesc { name: "L", port_type: PortType::List },
            ],
            NodeType::ReplaceItems => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "I", port_type: PortType::List },
                PortDesc { name: "V", port_type: PortType::List },
            ],
            NodeType::Sift => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "P", port_type: PortType::List },
            ],
            NodeType::Combine => vec![
                PortDesc { name: "A", port_type: PortType::List },
                PortDesc { name: "B", port_type: PortType::List },
            ],
            NodeType::Duplicate { .. } => vec![
                PortDesc { name: "D", port_type: PortType::Any },
            ],
            NodeType::NullItem => vec![],
            NodeType::PathMapper { .. } => vec![
                PortDesc { name: "T", port_type: PortType::List },
            ],
            NodeType::TreeBranch => vec![
                PortDesc { name: "T", port_type: PortType::List },
                PortDesc { name: "P", port_type: PortType::Integer },
            ],
            NodeType::TreeStatistics => vec![
                PortDesc { name: "T", port_type: PortType::List },
            ],
            NodeType::CleanTree { .. } => vec![
                PortDesc { name: "T", port_type: PortType::List },
            ],
            NodeType::ExplodeTree => vec![
                PortDesc { name: "T", port_type: PortType::List },
            ],
            // Phase H (extended): Output inputs
            NodeType::BakeToLayer { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::BakeMesh { .. } => vec![
                PortDesc { name: "M", port_type: PortType::Mesh },
            ],
            NodeType::BakeCurve { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::ExportSTEP { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::ExportSTL { .. } | NodeType::ExportOBJ { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Any },
            ],
            NodeType::Group { .. } => vec![
                PortDesc { name: "G", port_type: PortType::List },
            ],
            NodeType::Cluster { .. } => vec![
                PortDesc { name: "I", port_type: PortType::List },
            ],
        }
    }

    /// Output port descriptors for this node.
    pub fn output_ports(&self) -> Vec<PortDesc> {
        match self {
            NodeType::NumberSlider { .. } => vec![PortDesc { name: "V", port_type: PortType::Number }],
            NodeType::IntegerInput { .. } => vec![PortDesc { name: "V", port_type: PortType::Integer }],
            NodeType::BooleanToggle { .. } => vec![PortDesc { name: "V", port_type: PortType::Boolean }],
            NodeType::PointInput { .. } => vec![PortDesc { name: "P", port_type: PortType::Point }],
            NodeType::VectorInput { .. } => vec![PortDesc { name: "V", port_type: PortType::Vector }],
            NodeType::Panel => vec![PortDesc { name: "D", port_type: PortType::Any }],
            // Data Tree
            NodeType::Graft | NodeType::Flatten | NodeType::CrossRef |
            NodeType::ShiftList { .. } | NodeType::Subset { .. } |
            NodeType::Weave | NodeType::Concat => vec![
                PortDesc { name: "L", port_type: PortType::List },
            ],
            NodeType::Dispatch => vec![
                PortDesc { name: "A", port_type: PortType::List },
                PortDesc { name: "B", port_type: PortType::List },
            ],
            NodeType::BakeToDoc => vec![],
            NodeType::Series { .. } | NodeType::Range { .. } => vec![PortDesc { name: "L", port_type: PortType::List }],
            NodeType::ListLength => vec![PortDesc { name: "N", port_type: PortType::Integer }],
            NodeType::ListItem => vec![PortDesc { name: "I", port_type: PortType::Any }],
            NodeType::Reverse | NodeType::Sort | NodeType::CullPattern => vec![PortDesc { name: "L", port_type: PortType::List }],
            // Math outputs are Number
            NodeType::Add | NodeType::Subtract | NodeType::Multiply | NodeType::Divide |
            NodeType::Sin | NodeType::Cos | NodeType::Tan | NodeType::Abs |
            NodeType::Sqrt | NodeType::Pow | NodeType::Round |
            NodeType::Min | NodeType::Max | NodeType::Average | NodeType::Expression { .. } => vec![
                PortDesc { name: "R", port_type: PortType::Number },
            ],
            // Geometry outputs
            NodeType::Box { .. } | NodeType::Sphere { .. } | NodeType::Cylinder { .. } |
            NodeType::Cone { .. } | NodeType::Torus { .. } |
            NodeType::Move { .. } | NodeType::Rotate { .. } | NodeType::Scale { .. } |
            NodeType::Mirror { .. } | NodeType::LinearArray { .. } | NodeType::CircularArray { .. } |
            NodeType::BooleanUnion | NodeType::BooleanSubtract | NodeType::BooleanIntersect |
            NodeType::Fillet { .. } | NodeType::Chamfer { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            // Curve outputs
            NodeType::Line | NodeType::Circle { .. } => vec![PortDesc { name: "C", port_type: PortType::Curve }],
            NodeType::DivideCurve { .. } => vec![PortDesc { name: "P", port_type: PortType::List }],
            NodeType::EvaluateCurve => vec![PortDesc { name: "P", port_type: PortType::Point }],
            NodeType::CurveLength => vec![PortDesc { name: "L", port_type: PortType::Number }],
            // ── Params ──
            NodeType::PlaneInput { .. } => vec![PortDesc { name: "P", port_type: PortType::PlaneRef }],
            NodeType::DomainInput { .. } => vec![PortDesc { name: "D", port_type: PortType::Domain }],
            NodeType::StringInput { .. } => vec![PortDesc { name: "S", port_type: PortType::String }],
            // ── Analysis ──
            NodeType::Volume => vec![PortDesc { name: "V", port_type: PortType::Number }],
            NodeType::SurfaceArea => vec![PortDesc { name: "A", port_type: PortType::Number }],
            NodeType::Centroid => vec![PortDesc { name: "C", port_type: PortType::Point }],
            NodeType::BoundingBox => vec![
                PortDesc { name: "Min", port_type: PortType::Point },
                PortDesc { name: "Max", port_type: PortType::Point },
            ],
            NodeType::Distance => vec![PortDesc { name: "D", port_type: PortType::Number }],
            NodeType::Angle => vec![PortDesc { name: "A", port_type: PortType::Number }],
            NodeType::MassProperties => vec![
                PortDesc { name: "V", port_type: PortType::Number },
                PortDesc { name: "C", port_type: PortType::Point },
            ],
            // ── Vector & Math ──
            NodeType::Cross => vec![PortDesc { name: "V", port_type: PortType::Vector }],
            NodeType::Dot => vec![PortDesc { name: "R", port_type: PortType::Number }],
            NodeType::VectorLength => vec![PortDesc { name: "L", port_type: PortType::Number }],
            NodeType::Unit => vec![PortDesc { name: "V", port_type: PortType::Vector }],
            NodeType::Negative => vec![PortDesc { name: "R", port_type: PortType::Any }],
            NodeType::Reciprocal | NodeType::Asin | NodeType::Acos | NodeType::Atan |
            NodeType::Log | NodeType::Ln | NodeType::Exp => vec![
                PortDesc { name: "R", port_type: PortType::Number },
            ],
            NodeType::Atan2 | NodeType::Modulus => vec![
                PortDesc { name: "R", port_type: PortType::Number },
            ],
            NodeType::MapDomain { .. } => vec![PortDesc { name: "R", port_type: PortType::Number }],
            NodeType::PointMidpoint => vec![PortDesc { name: "P", port_type: PortType::Point }],
            NodeType::PointLerp { .. } => vec![PortDesc { name: "P", port_type: PortType::Point }],
            NodeType::Vector2pt => vec![PortDesc { name: "V", port_type: PortType::Vector }],
            // ── Surface Creation ──
            NodeType::Extrude { .. } | NodeType::Revolve { .. } | NodeType::Loft |
            NodeType::Sweep | NodeType::ExtrudePoint | NodeType::ExtrudeTapered { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::RuledSurface | NodeType::PlaneSurface => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            // ── Surface Eval & Modify ──
            NodeType::EvaluateSurface => vec![PortDesc { name: "P", port_type: PortType::Point }],
            NodeType::SurfaceNormal => vec![PortDesc { name: "N", port_type: PortType::Vector }],
            NodeType::Shell { .. } | NodeType::Thicken { .. } | NodeType::OffsetSolid { .. } |
            NodeType::SplitSolid | NodeType::TrimSolid | NodeType::Hole { .. } |
            // Phase E (extended): Solid Modification
            NodeType::DraftFace { .. } | NodeType::MoveFace |
            NodeType::OffsetFace { .. } | NodeType::ReplaceFace |
            NodeType::HoleCircular { .. } | NodeType::Rib { .. } |
            NodeType::FilletEdge { .. } | NodeType::ChamferEdge { .. } |
            NodeType::FilletVariable => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            // Phase E (extended): Surface Eval outputs
            NodeType::SurfaceFrame => vec![PortDesc { name: "F", port_type: PortType::PlaneRef }],
            NodeType::SurfaceCurvature => vec![
                PortDesc { name: "K1", port_type: PortType::Number },
                PortDesc { name: "K2", port_type: PortType::Number },
                PortDesc { name: "D1", port_type: PortType::Vector },
                PortDesc { name: "D2", port_type: PortType::Vector },
            ],
            NodeType::SurfaceAreaUV { .. } => vec![PortDesc { name: "A", port_type: PortType::Number }],
            NodeType::IsoTrim { .. } | NodeType::SurfaceFlip |
            NodeType::SurfaceRebuild { .. } | NodeType::SurfaceUntrim | NodeType::SurfaceTrim => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::DivideSurface { .. } => vec![
                PortDesc { name: "P", port_type: PortType::List },
                PortDesc { name: "N", port_type: PortType::List },
                PortDesc { name: "F", port_type: PortType::List },
            ],
            NodeType::SurfaceClosestPoint => vec![
                PortDesc { name: "U", port_type: PortType::Number },
                PortDesc { name: "V", port_type: PortType::Number },
                PortDesc { name: "Q", port_type: PortType::Point },
            ],
            NodeType::SurfaceProjectPoint { .. } => vec![
                PortDesc { name: "Q", port_type: PortType::List },
                PortDesc { name: "U", port_type: PortType::List },
                PortDesc { name: "V", port_type: PortType::List },
            ],
            NodeType::SurfaceSplit { .. } => vec![
                PortDesc { name: "A", port_type: PortType::Surface },
                PortDesc { name: "B", port_type: PortType::Surface },
            ],
            NodeType::SurfaceFromPoints { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::SurfaceIsocurve { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            // ── Curve ──
            NodeType::Arc { .. } | NodeType::Arc3pt | NodeType::Ellipse { .. } |
            NodeType::Polyline | NodeType::NurbsCurve | NodeType::NurbsCurveInterp |
            NodeType::JoinCurves | NodeType::CurveOffset { .. } | NodeType::Extend { .. } |
            NodeType::Flip | NodeType::Rebuild { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::PointAt => vec![PortDesc { name: "P", port_type: PortType::Point }],
            NodeType::Tangent => vec![PortDesc { name: "V", port_type: PortType::Vector }],
            NodeType::Curvature => vec![PortDesc { name: "C", port_type: PortType::Number }],
            NodeType::NearestPoint => vec![PortDesc { name: "P", port_type: PortType::Point }],
            NodeType::EndPoints => vec![
                PortDesc { name: "S", port_type: PortType::Point },
                PortDesc { name: "E", port_type: PortType::Point },
            ],
            NodeType::SplitCurve { .. } => vec![
                PortDesc { name: "A", port_type: PortType::Curve },
                PortDesc { name: "B", port_type: PortType::Curve },
            ],
            // ── Transform ──
            NodeType::RotateAxis { .. } | NodeType::MirrorPlane | NodeType::Orient |
            NodeType::Project | NodeType::ArrayAlongCurve { .. } |
            NodeType::ArrayBox { .. } | NodeType::ArrayOnSurface { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::ComposeTransform | NodeType::InvertTransform => vec![
                PortDesc { name: "T", port_type: PortType::Transform },
            ],
            // ── Intersect ──
            NodeType::CurveCurveIntersect | NodeType::CurveSurfaceIntersect |
            NodeType::SurfaceSurfaceIntersect | NodeType::CurvePlaneIntersect |
            NodeType::SolidPlaneIntersect => vec![
                PortDesc { name: "P", port_type: PortType::List },
            ],
            NodeType::PlanePlaneIntersect => vec![
                PortDesc { name: "L", port_type: PortType::Curve },
            ],
            NodeType::BooleanSplit => vec![
                PortDesc { name: "A", port_type: PortType::Geometry },
                PortDesc { name: "B", port_type: PortType::Geometry },
            ],
            // ── Mesh ──
            NodeType::ToMesh => vec![PortDesc { name: "M", port_type: PortType::Mesh }],
            NodeType::MeshPrimitive => vec![PortDesc { name: "M", port_type: PortType::Mesh }],
            NodeType::MeshArea => vec![PortDesc { name: "A", port_type: PortType::Number }],
            NodeType::MeshVolume => vec![PortDesc { name: "V", port_type: PortType::Number }],
            NodeType::MeshFlip | NodeType::MeshWeld { .. } | NodeType::MeshSubdivide { .. } |
            NodeType::MeshDecimate { .. } | NodeType::MeshSmooth { .. } => vec![
                PortDesc { name: "M", port_type: PortType::Mesh },
            ],
            // Phase H (extended): Sets/Tree outputs
            NodeType::CullIndex { .. } | NodeType::Partition { .. } |
            NodeType::ReplaceItems | NodeType::Combine | NodeType::Duplicate { .. } |
            NodeType::PathMapper { .. } | NodeType::CleanTree { .. } |
            NodeType::TreeBranch | NodeType::ExplodeTree => vec![
                PortDesc { name: "R", port_type: PortType::List },
            ],
            NodeType::Sift => vec![
                PortDesc { name: "T", port_type: PortType::List },
                PortDesc { name: "F", port_type: PortType::List },
            ],
            NodeType::NullItem => vec![PortDesc { name: "N", port_type: PortType::Any }],
            NodeType::TreeStatistics => vec![
                PortDesc { name: "Paths", port_type: PortType::List },
                PortDesc { name: "Counts", port_type: PortType::List },
            ],
            // Phase H (extended): Output outputs
            NodeType::BakeToLayer { .. } | NodeType::BakeMesh { .. } | NodeType::BakeCurve { .. } => vec![],
            NodeType::ExportSTEP { .. } | NodeType::ExportSTL { .. } | NodeType::ExportOBJ { .. } => vec![
                PortDesc { name: "S", port_type: PortType::String },
            ],
            NodeType::Group { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::Cluster { .. } => vec![
                PortDesc { name: "O", port_type: PortType::List },
            ],
        }
    }

    /// Number of input ports.
    pub fn input_count(&self) -> usize {
        self.input_ports().len()
    }

    /// Number of output ports.
    pub fn output_count(&self) -> usize {
        self.output_ports().len()
    }
}

/// A node in the graph.
#[derive(Clone, Debug)]
pub struct VpNode {
    pub id: u64,
    pub node_type: NodeType,
    pub x: f32,
    pub y: f32,
}

/// A connection between two nodes.
#[derive(Clone, Debug)]
pub struct VpConnection {
    pub from_node: u64,
    pub from_port: usize,
    pub to_node: u64,
    pub to_port: usize,
}

/// Visual programming graph.
#[derive(Clone, Debug, Default)]
pub struct VpGraph {
    pub nodes: Vec<VpNode>,
    pub connections: Vec<VpConnection>,
    pub next_id: u64,
    pub live_preview: bool,
}

impl VpGraph {
    pub fn new() -> Self {
        Self { live_preview: true, ..Default::default() }
    }

    pub fn add_node(&mut self, node_type: NodeType, x: f32, y: f32) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        self.nodes.push(VpNode { id, node_type, x, y });
        id
    }

    pub fn connect(&mut self, from: u64, from_port: usize, to: u64, to_port: usize) {
        self.connections.push(VpConnection {
            from_node: from, from_port, to_node: to, to_port,
        });
    }

    pub fn node_count(&self) -> usize { self.nodes.len() }
    pub fn connection_count(&self) -> usize { self.connections.len() }
}

// ============================================================
// 9.2. Surface Modeling
// ============================================================

/// Continuity level for surface operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Continuity {
    G0, // Position
    G1, // Tangent
    G2, // Curvature
}

impl Continuity {
    pub fn label(&self) -> &'static str {
        match self {
            Continuity::G0 => "G0 (Position)",
            Continuity::G1 => "G1 (Tangent)",
            Continuity::G2 => "G2 (Curvature)",
        }
    }
}

/// Surface operation parameters.
#[derive(Clone, Debug)]
pub enum SurfaceOp {
    /// Loft through 2+ profiles.
    Loft {
        profiles: Vec<String>, // Profile names/IDs
        continuity: Continuity,
        closed: bool,
    },
    /// Sweep a profile along a path.
    Sweep {
        profile: String,
        path: String,
        guide_rails: Vec<String>,
        twist_angle: f64,
    },
    /// Boundary surface from 4 edges.
    Boundary {
        edge1: String,
        edge2: String,
        edge3: String,
        edge4: String,
        continuity: Continuity,
    },
    /// Fill an n-sided patch.
    Fill {
        edges: Vec<String>,
        continuity: Continuity,
    },
    /// Network surface from UV grid of curves.
    Network {
        u_curves: Vec<String>,
        v_curves: Vec<String>,
    },
}

// ============================================================
// 9.3. Sheet Metal
// ============================================================

/// Bend type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BendType {
    Bend,
    Hem,
    Jog,
    Lofted,
}

/// Relief type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReliefType {
    Rectangular,
    Tear,
    Obround,
}

/// Sheet metal parameters.
#[derive(Clone, Debug)]
pub struct SheetMetalParams {
    pub thickness: f64,
    pub bend_radius: f64,
    pub k_factor: f64,
    pub bend_angle: f64, // degrees
    pub relief_type: ReliefType,
    pub material: String,
}

impl Default for SheetMetalParams {
    fn default() -> Self {
        Self {
            thickness: 1.5,
            bend_radius: 2.0,
            k_factor: 0.44,
            bend_angle: 90.0,
            relief_type: ReliefType::Rectangular,
            material: "Steel 1.5mm".to_string(),
        }
    }
}

impl SheetMetalParams {
    /// Calculate bend allowance.
    /// BA = (π/180) × angle × (r + k × t)
    pub fn bend_allowance(&self) -> f64 {
        let angle_rad = self.bend_angle.to_radians();
        angle_rad * (self.bend_radius + self.k_factor * self.thickness)
    }
}

/// Sheet metal operation.
#[derive(Clone, Debug)]
pub enum SheetMetalOp {
    BaseFlange { params: SheetMetalParams, sketch_id: u64 },
    EdgeFlange { params: SheetMetalParams, edge_id: u64, length: f64, angle: f64 },
    Bend { params: SheetMetalParams, edge_id: u64 },
    Hem { edge_id: u64, hem_length: f64 },
    Jog { edge_id: u64, jog_length: f64, jog_angle: f64 },
    Unfold { body_id: u64 },
    Fold { body_id: u64, bend_line: u64 },
    FlatPattern { body_id: u64 },
}

// ============================================================
// 9.4. CAM
// ============================================================

/// Tool type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolType {
    FlatEndMill,
    BallEndMill,
    BullNose,
    Drill,
    FaceMill,
    Tap,
    Reamer,
}

impl ToolType {
    pub fn label(&self) -> &'static str {
        match self {
            ToolType::FlatEndMill => "Flat End Mill",
            ToolType::BallEndMill => "Ball End Mill",
            ToolType::BullNose => "Bull Nose",
            ToolType::Drill => "Drill",
            ToolType::FaceMill => "Face Mill",
            ToolType::Tap => "Tap",
            ToolType::Reamer => "Reamer",
        }
    }
}

/// Cutting tool.
#[derive(Clone, Debug)]
pub struct Tool {
    pub id: u32,
    pub name: String,
    pub tool_type: ToolType,
    pub diameter: f64,
    pub length: f64,
    pub flutes: u32,
    pub feed_rate: f64,    // mm/min
    pub spindle_speed: f64, // RPM
}

/// CAM operation type.
#[derive(Clone, Debug)]
pub enum CamOperation {
    Facing { tool_id: u32, stepover: f64, depth_per_pass: f64 },
    Profile { tool_id: u32, depth_per_pass: f64, finish_passes: u32 },
    Pocket { tool_id: u32, stepover: f64, depth_per_pass: f64 },
    Drilling { tool_id: u32, depth: f64, peck: f64 },
    Engraving { tool_id: u32, depth: f64 },
    Surface3D { tool_id: u32, stepover: f64, tolerance: f64 },
    FiveAxisSWARF { tool_id: u32 },
    FiveAxisMorph { tool_id: u32 },
}

/// CAM operation status.
#[derive(Clone, Debug, PartialEq)]
pub enum OpStatus {
    Queued,
    Running { progress: f32 },
    Done { duration_sec: f64 },
    Failed { error_code: u32 },
}

/// G-code post-processor dialect.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PostDialect {
    Fanuc,
    Siemens,
    Haas,
    Heidenhain,
    Mach3,
    LinuxCNC,
    GRBL,
}

impl PostDialect {
    pub fn label(&self) -> &'static str {
        match self {
            PostDialect::Fanuc => "Fanuc",
            PostDialect::Siemens => "Siemens 840D",
            PostDialect::Haas => "Haas",
            PostDialect::Heidenhain => "Heidenhain",
            PostDialect::Mach3 => "Mach3",
            PostDialect::LinuxCNC => "LinuxCNC",
            PostDialect::GRBL => "GRBL",
        }
    }
}

/// CAM setup.
#[derive(Clone, Debug, Default)]
pub struct CamSetup {
    pub tools: Vec<Tool>,
    pub operations: Vec<(CamOperation, OpStatus)>,
    pub stock_bbox: ([f64; 3], [f64; 3]),
    pub post_dialect: Option<PostDialect>,
    pub g_code: Option<String>,
}

// ============================================================
// 9.5. FEA / Simulation
// ============================================================

/// Element type for mesh generation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ElementType {
    Tet4,
    Tet10,
    Hex8,
    Hex20,
}

impl ElementType {
    pub fn label(&self) -> &'static str {
        match self {
            ElementType::Tet4 => "Tet4 (4-node tetrahedron)",
            ElementType::Tet10 => "Tet10 (10-node tetrahedron)",
            ElementType::Hex8 => "Hex8 (8-node hexahedron)",
            ElementType::Hex20 => "Hex20 (20-node hexahedron)",
        }
    }
}

/// Study type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StudyType {
    Static,
    Modal,
    Thermal,
    Buckling,
    Fatigue,
    Nonlinear,
    CFD,
    Electromagnetic,
    Optimization,
}

impl StudyType {
    pub fn label(&self) -> &'static str {
        match self {
            StudyType::Static => "Static Structural",
            StudyType::Modal => "Modal (Frequency)",
            StudyType::Thermal => "Thermal",
            StudyType::Buckling => "Buckling",
            StudyType::Fatigue => "Fatigue",
            StudyType::Nonlinear => "Nonlinear",
            StudyType::CFD => "CFD (Fluid)",
            StudyType::Electromagnetic => "Electromagnetic",
            StudyType::Optimization => "Topology Optimization",
        }
    }
}

/// Boundary condition.
#[derive(Clone, Debug)]
pub enum BoundaryCondition {
    Fixed { entity_ids: Vec<u64> },
    Force { entity_ids: Vec<u64>, fx: f64, fy: f64, fz: f64 },
    Pressure { entity_ids: Vec<u64>, magnitude: f64 },
    Displacement { entity_ids: Vec<u64>, dx: f64, dy: f64, dz: f64 },
    Thermal { entity_ids: Vec<u64>, temperature: f64 },
    HeatFlux { entity_ids: Vec<u64>, flux: f64 },
}

/// FEA study.
#[derive(Clone, Debug)]
pub struct FeaStudy {
    pub name: String,
    pub study_type: StudyType,
    pub element_type: ElementType,
    pub mesh_size: f64,
    pub boundary_conditions: Vec<BoundaryCondition>,
    pub material: String,
    pub solved: bool,
    pub max_von_mises: Option<f64>,
    pub max_displacement: Option<f64>,
    pub iterations: Option<u32>,
    pub converged: Option<bool>,
}

impl Default for FeaStudy {
    fn default() -> Self {
        Self {
            name: "Static Analysis".to_string(),
            study_type: StudyType::Static,
            element_type: ElementType::Tet4,
            mesh_size: 2.0,
            boundary_conditions: Vec::new(),
            material: "Steel AISI 1045".to_string(),
            solved: false,
            max_von_mises: None,
            max_displacement: None,
            iterations: None,
            converged: None,
        }
    }
}

// ============================================================
// 9.6. Drawing
// ============================================================

/// Paper size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PaperSize {
    A0, A1, A2, A3, A4, AnsiB, Custom,
}

impl PaperSize {
    pub fn dimensions_mm(&self) -> (f64, f64) {
        match self {
            PaperSize::A0 => (841.0, 1189.0),
            PaperSize::A1 => (594.0, 841.0),
            PaperSize::A2 => (420.0, 594.0),
            PaperSize::A3 => (297.0, 420.0),
            PaperSize::A4 => (210.0, 297.0),
            PaperSize::AnsiB => (279.0, 432.0),
            PaperSize::Custom => (500.0, 500.0),
        }
    }
}

/// Drawing view type.
#[derive(Clone, Debug)]
pub enum DrawingView {
    Standard { orientation: String, scale: f64 },
    Section { cut_plane: [f64; 4], scale: f64 },
    Detail { center: [f64; 2], radius: f64, scale: f64 },
    Projected { direction: String, scale: f64 },
    BrokenOut { cut_depth: f64, scale: f64 },
    Crop { boundary: Vec<[f64; 2]>, scale: f64 },
    Auxiliary { direction: [f64; 3], scale: f64 },
    Exploded { offset: f64, scale: f64 },
}

/// Title block field.
#[derive(Clone, Debug)]
pub struct TitleBlockField {
    pub label: String,
    pub value: String,
    pub custom: bool,
}

/// Drawing sheet.
#[derive(Clone, Debug)]
pub struct DrawingSheet {
    pub paper: PaperSize,
    pub views: Vec<DrawingView>,
    pub title_block: Vec<TitleBlockField>,
    pub revisions: Vec<RevisionEntry>,
}

/// Revision table entry.
#[derive(Clone, Debug)]
pub struct RevisionEntry {
    pub rev: String,
    pub description: String,
    pub date: String,
    pub approved_by: String,
}

impl Default for DrawingSheet {
    fn default() -> Self {
        Self {
            paper: PaperSize::A3,
            views: Vec::new(),
            title_block: vec![
                TitleBlockField { label: "Title".into(), value: "".into(), custom: false },
                TitleBlockField { label: "Drawing No.".into(), value: "".into(), custom: false },
                TitleBlockField { label: "Scale".into(), value: "1:1".into(), custom: false },
                TitleBlockField { label: "Material".into(), value: "".into(), custom: false },
                TitleBlockField { label: "Designer".into(), value: "".into(), custom: false },
                TitleBlockField { label: "Date".into(), value: "".into(), custom: false },
            ],
            revisions: Vec::new(),
        }
    }
}

// ============================================================
// 9.7. Assembly
// ============================================================

/// Mate type.
#[derive(Clone, Debug)]
pub enum MateType {
    Coincident,
    Concentric,
    Distance(f64),
    Angle(f64),
    Parallel,
    Perpendicular,
    Tangent,
    Width,
    Symmetric,
}

/// Assembly mate.
#[derive(Clone, Debug)]
pub struct Mate {
    pub component_a: u64,
    pub entity_a: u64,
    pub component_b: u64,
    pub entity_b: u64,
    pub mate_type: MateType,
}

/// Assembly component instance.
#[derive(Clone, Debug)]
pub struct AssemblyComponent {
    pub id: u64,
    pub name: String,
    pub source_file: String,
    pub transform: [[f64; 4]; 4],
    pub visible: bool,
    pub fixed: bool,
}

/// Assembly.
#[derive(Clone, Debug, Default)]
pub struct Assembly {
    pub components: Vec<AssemblyComponent>,
    pub mates: Vec<Mate>,
    pub next_component_id: u64,
}

impl Assembly {
    pub fn add_component(&mut self, name: &str, source: &str) -> u64 {
        let id = self.next_component_id;
        self.next_component_id += 1;
        self.components.push(AssemblyComponent {
            id,
            name: name.to_string(),
            source_file: source.to_string(),
            transform: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            visible: true,
            fixed: false,
        });
        id
    }

    pub fn add_mate(&mut self, mate: Mate) {
        self.mates.push(mate);
    }

    /// BOM (Bill of Materials).
    pub fn bom(&self) -> Vec<(String, u32)> {
        let mut counts: HashMap<String, u32> = HashMap::new();
        for c in &self.components {
            *counts.entry(c.name.clone()).or_default() += 1;
        }
        let mut bom: Vec<(String, u32)> = counts.into_iter().collect();
        bom.sort_by(|a, b| a.0.cmp(&b.0));
        bom
    }
}

// ============================================================
// 9.8. Point Cloud & Reverse Engineering
// ============================================================

/// Point cloud format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointCloudFormat {
    PLY,
    XYZ,
    LAS,
}

/// Point cloud.
#[derive(Clone, Debug)]
pub struct PointCloud {
    pub points: Vec<[f64; 3]>,
    pub format: PointCloudFormat,
    pub normals: Vec<[f64; 3]>,
    pub colors: Vec<[u8; 3]>,
}

/// Reverse engineering step.
#[derive(Clone, Debug)]
pub enum ReStep {
    Import { file_path: String },
    Denoise { threshold: f64 },
    Segment { min_cluster_size: usize },
    DetectShapes { tolerance: f64 },
    FitPrimitives { tolerance: f64 },
    CreateSolid { merge_tolerance: f64 },
}

// ============================================================
// 9.9. Mold Design
// ============================================================

/// Mold base standard.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoldStandard {
    Misumi,
    Hasco,
    Dme,
    Lkm,
}

impl MoldStandard {
    pub fn label(&self) -> &'static str {
        match self {
            MoldStandard::Misumi => "Misumi",
            MoldStandard::Hasco => "HASCO",
            MoldStandard::Dme => "DME",
            MoldStandard::Lkm => "LKM",
        }
    }
}

/// Mold design project.
#[derive(Clone, Debug, Default)]
pub struct MoldProject {
    pub standard: Option<MoldStandard>,
    pub mold_base: Option<String>,
    pub runner_type: Option<String>,
    pub cooling_channels: Vec<String>,
    pub ejection_pins: Vec<String>,
    pub cavity: Option<u64>,
    pub core: Option<u64>,
    pub flow_analysis: Option<String>,
    pub cooling_analysis: Option<String>,
    pub warpage_analysis: Option<String>,
    pub estimated_cost: Option<f64>,
    pub cycle_time_sec: Option<f64>,
}

// ============================================================
// 9.10. AI Features
// ============================================================

/// AI feature type.
#[derive(Clone, Debug)]
pub enum AiFeature {
    /// Text → 3D shape generation.
    ShapeFromText { prompt: String },
    /// AI assistant chat.
    Assistant { mode: AiAssistantMode, message: String },
    /// Smart auto-fillet.
    AutoFillet { max_radius: f64 },
    /// Auto-pattern detection.
    AutoPattern { sensitivity: f64 },
    /// Auto-repair geometry.
    AutoRepair { tolerance: f64 },
    /// Auto-dimension sketch.
    AutoDimension,
    /// Auto-constrain sketch.
    AutoConstrain,
    /// Generative design variant.
    GenerateVariant { constraints: String, variant: char },
    /// Topology optimization.
    Optimize { preset: OptimizePreset },
}

/// AI assistant mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AiAssistantMode {
    Chat,
    DesignReview,
    CostEstimate,
    SuggestFeature,
}

impl AiAssistantMode {
    pub fn label(&self) -> &'static str {
        match self {
            AiAssistantMode::Chat => "Chat",
            AiAssistantMode::DesignReview => "Design Review (DRC)",
            AiAssistantMode::CostEstimate => "Cost Estimate",
            AiAssistantMode::SuggestFeature => "Suggest Feature",
        }
    }
}

/// Optimization preset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OptimizePreset {
    Lightweight,
    Stiff,
    Balanced,
}

impl OptimizePreset {
    pub fn label(&self) -> &'static str {
        match self {
            OptimizePreset::Lightweight => "Lightweight",
            OptimizePreset::Stiff => "Stiff",
            OptimizePreset::Balanced => "Balanced",
        }
    }
}
