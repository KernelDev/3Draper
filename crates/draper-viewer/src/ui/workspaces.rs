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

    // ───── Phase I (Optional/Advanced): Primitives ─────
    /// Rectangular planar solid (finite plane patch).
    PlanePrimitive { width: f64, height: f64 },
    /// Regular n-gonal prism.
    PolygonPrism { sides: u32, radius: f64, height: f64 },
    /// Pipe along a curve (sweep a circle along path).
    Tube { radius: f64 },
    /// Helical curve.
    Helix { radius: f64, pitch: f64, turns: f64 },
    /// Wedge / pie-shape solid.
    Wedge { radius: f64, angle_deg: f64, height: f64 },
    /// Tetrahedron (Platonic solid).
    Tetra { radius: f64 },
    /// Octahedron (Platonic solid).
    Octa { radius: f64 },
    /// Icosahedron (Platonic solid).
    Icosa { radius: f64 },

    // ───── Phase I (Optional/Advanced): Curves ─────
    /// Hyperbola (conic section).
    Hyperbola { semi_real: f64, semi_imag: f64 },
    /// Parabola (conic section).
    Parabola { focal_dist: f64 },

    // ───── Phase I (Optional/Advanced): Transforms ─────
    /// Shear transform (6 shear factors).
    Shear { xy: f64, xz: f64, yx: f64, yz: f64, zx: f64, zy: f64 },
    /// Apply draft angle to solid (useful for molded parts).
    Taper { draft_angle_deg: f64, height: f64 },
    /// Apply an authored Transform to geometry.
    ApplyTransform,
    /// Polar array around arbitrary axis.
    ArrayPolar { count: u32, angle_deg: f64 },
    /// Offset solid along a direction (creates a "thickened" copy).
    Offset { distance: f64, both_sides: bool },

    // ───── Extended Curve Nodes ─────
    /// Boolean union of planar closed curves.
    CurveBooleanUnion,
    /// Boolean subtract of planar closed curves.
    CurveBooleanSubtract,
    /// Boolean intersect of planar closed curves.
    CurveBooleanIntersect,
    /// Shatter curves at every discontinuity.
    CurveShatter,
    /// Find points where continuity drops below level.
    CurveDiscontinuity { level: u32 },
    /// Perpendicular frame at parameter (Frenet frame).
    CurveFrame,
    /// Normal vector at parameter (in plane perpendicular to tangent).
    CurveNormal,
    /// Project curve onto plane (drop normal component).
    ProjectCurveToPlane,
    /// Project curve onto surface along direction; result is a PCurve.
    ProjectCurveToSurface { direction: Option<[f64; 3]> },
    /// For closed curves, change the start point.
    CurveSeam,

    // ───── Extended Surface Creation Nodes ─────
    /// Cylinder surface (lateral only).
    CylinderSurface { radius: f64, height: f64 },
    /// Cone surface (lateral).
    ConeSurface { radius: f64, height: f64 },
    /// Sphere surface.
    SphereSurface { radius: f64 },
    /// Torus surface.
    TorusSurface { major: f64, minor: f64 },
    /// Offset surface along normals.
    OffsetSurface { distance: f64, both_sides: bool },

    // ───── Extended Intersect Nodes ─────
    /// Intersection of two solids → curves + points.
    BrepBrepIntersect,
    /// Mesh-mesh intersection → polylines.
    MeshMeshIntersect,
    /// Closest points between two lines.
    LineLineClosestPoint,
    /// Point-in-solid test.
    SolidInclusion,
    /// Collision check between two solids.
    CollisionCheck,
    /// Trim solid with curve-defined knives.
    BooleanTrim,
    /// Mesh boolean union.
    MeshBooleanUnion,
    /// Mesh boolean subtract.
    MeshBooleanSubtract,
    /// Mesh boolean intersect.
    MeshBooleanIntersect,

    // ───── Extended Analysis Nodes ─────
    /// Moments of inertia (6 components).
    MomentsOfInertia,
    /// Curve curvature comb data (samples).
    CurveCurvatureAnalysis { samples: u32 },
    /// Surface curvature heatmap data.
    SurfaceCurvatureAnalysis { u_samples: u32, v_samples: u32 },
    /// Point-in-curve (closed) test.
    PointInCurve,
    /// Closest point on surface (with distance output).
    ClosestPointOnSurface,
    /// Closest point on curve (with distance output).
    ClosestPointOnCurve,
    /// Detect self-intersections.
    SelfIntersect,
    /// Test whether curve/point set is planar.
    Planar,
    /// Test whether curve/surface is closed.
    Closed,

    // ───── Extended Params Nodes ─────
    /// Author a 4x4 transform directly.
    TransformInput { translation: [f64; 3], rotation_deg: [f64; 3], scale: [f64; 3] },
    /// RGBA color swatch.
    ColorInput { r: f32, g: f32, b: f32, a: f32 },
    /// Load a CAD file (STEP/IGES) as a parameter.
    FileInput { path: String },
    /// Data-tree path for PathMapper, TreeBranch.
    PathInput { branch: u32, indices: Vec<i32> },

    // ───── Sub-graph Operations ─────
    /// Apply a list of operations to every item in the input list.
    /// Operations: negate/abs/sqrt/sin/cos/tan/log10/ln/exp/reciprocal/double/
    /// halve/square/cube/radians/degrees/add:N/mul:N/pow:N/normalize/length/
    /// scale:N/upper/lower/trim/len. Separated by `;`.
    ListMap { operations: String },

    // ───── Kernel API Wrappers (new VP nodes for existing core API) ─────
    /// Heal a solid (fix gaps, stitch faces, remove small features).
    HealSolid,
    /// Stitch two solids' shells within tolerance.
    StitchSolids { tolerance: f64 },
    /// Validate solid topology (Euler characteristic, manifold check).
    ValidateSolid,
    /// Export mesh to glTF format.
    ExportGLTF { path: String },
    /// Export mesh to USD format.
    ExportUSD { path: String },
    /// Export mesh to 3MF format.
    Export3MF { path: String },
    /// Export mesh to PLY format.
    ExportPLY { path: String },
    /// Export mesh to DXF format.
    ExportDXF { path: String },
    /// Import STL file as a mesh.
    ImportSTL { path: String },
    /// Solve a 2D sketch constraint system.
    SketchSolve,
    /// Solve a 3D assembly constraint system (6-DOF).
    AssemblySolve,
    /// Create a 2D drawing from a 3D mesh with HLR.
    CreateDrawing { view_type: u32 }, // 0=front, 1=top, 2=side, 3=iso
    /// Export drawing to PDF.
    ExportPDF { path: String },
    /// Export drawing to SVG.
    ExportSVG { path: String },
    /// Run FEA analysis on a solid.
    FEASolve { density: f64 },
    /// Generate CAM toolpath from a solid.
    GenerateToolpath { tool_diameter: f64 },
    /// Export CAM toolpath to G-code.
    ExportGCode { path: String },
    /// Unfold a sheet metal solid.
    UnfoldSheet { k_factor: f64 },
    /// Convert a solid to an implicit SDF representation.
    ToImplicit,
    /// Boolean union of two implicit solids (SDF).
    SdfUnion,
    /// Boolean subtract of two implicit solids (SDF).
    SdfSubtract,
    /// Boolean intersect of two implicit solids (SDF).
    SdfIntersect,
    /// Generate mesh from implicit solid via Dual Contouring.
    DualContourMesh { voxel_size: f64 },
    /// Subdivide a mesh using Catmull-Clark scheme.
    SubDSubdivide { levels: u32 },
    /// Convert SubD mesh to NURBS patches.
    SubDToNurbs,
    /// Evaluate NURBS surface on GPU (WebGPU compute shader).
    GPUEvalNurbs,
    /// Compute geometry hash (quantum-resistant).
    GeometryHash,
    /// Verify geometry hash against expected value.
    MerkleVerify,
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
            // Phase I (Optional/Advanced): Primitives
            NodeType::PlanePrimitive { .. } => "Plane Primitive",
            NodeType::PolygonPrism { .. } => "Polygon Prism",
            NodeType::Tube { .. } => "Tube",
            NodeType::Helix { .. } => "Helix",
            NodeType::Wedge { .. } => "Wedge",
            NodeType::Tetra { .. } => "Tetrahedron",
            NodeType::Octa { .. } => "Octahedron",
            NodeType::Icosa { .. } => "Icosahedron",
            // Phase I: Curves
            NodeType::Hyperbola { .. } => "Hyperbola",
            NodeType::Parabola { .. } => "Parabola",
            // Phase I: Transforms
            NodeType::Shear { .. } => "Shear",
            NodeType::Taper { .. } => "Taper",
            NodeType::ApplyTransform => "Apply Transform",
            NodeType::ArrayPolar { .. } => "Array Polar",
            NodeType::Offset { .. } => "Offset",
            // Extended Curve
            NodeType::CurveBooleanUnion => "Curve Boolean Union",
            NodeType::CurveBooleanSubtract => "Curve Boolean Subtract",
            NodeType::CurveBooleanIntersect => "Curve Boolean Intersect",
            NodeType::CurveShatter => "Curve Shatter",
            NodeType::CurveDiscontinuity { .. } => "Curve Discontinuity",
            NodeType::CurveFrame => "Curve Frame",
            NodeType::CurveNormal => "Curve Normal",
            NodeType::ProjectCurveToPlane => "Project Curve to Plane",
            NodeType::ProjectCurveToSurface { .. } => "Project Curve to Surface",
            NodeType::CurveSeam => "Curve Seam",
            // Extended Surface
            NodeType::CylinderSurface { .. } => "Cylinder Surface",
            NodeType::ConeSurface { .. } => "Cone Surface",
            NodeType::SphereSurface { .. } => "Sphere Surface",
            NodeType::TorusSurface { .. } => "Torus Surface",
            NodeType::OffsetSurface { .. } => "Offset Surface",
            // Extended Intersect
            NodeType::BrepBrepIntersect => "Brep | Brep Intersect",
            NodeType::MeshMeshIntersect => "Mesh | Mesh Intersect",
            NodeType::LineLineClosestPoint => "Line-Line Closest Point",
            NodeType::SolidInclusion => "Solid Inclusion",
            NodeType::CollisionCheck => "Collision Check",
            NodeType::BooleanTrim => "Boolean Trim",
            NodeType::MeshBooleanUnion => "Mesh Boolean Union",
            NodeType::MeshBooleanSubtract => "Mesh Boolean Subtract",
            NodeType::MeshBooleanIntersect => "Mesh Boolean Intersect",
            // Extended Analysis
            NodeType::MomentsOfInertia => "Moments of Inertia",
            NodeType::CurveCurvatureAnalysis { .. } => "Curve Curvature Analysis",
            NodeType::SurfaceCurvatureAnalysis { .. } => "Surface Curvature Analysis",
            NodeType::PointInCurve => "Point in Curve",
            NodeType::ClosestPointOnSurface => "Closest Point on Surface",
            NodeType::ClosestPointOnCurve => "Closest Point on Curve",
            NodeType::SelfIntersect => "Self Intersect",
            NodeType::Planar => "Planar",
            NodeType::Closed => "Closed",
            // Extended Params
            NodeType::TransformInput { .. } => "Transform Input",
            NodeType::ColorInput { .. } => "Color Input",
            NodeType::FileInput { .. } => "File Input",
            NodeType::PathInput { .. } => "Path Input",
            NodeType::ListMap { .. } => "List Map",
            // Kernel API Wrappers
            NodeType::HealSolid => "Heal Solid",
            NodeType::StitchSolids { .. } => "Stitch Solids",
            NodeType::ValidateSolid => "Validate Solid",
            NodeType::ExportGLTF { .. } => "Export glTF",
            NodeType::ExportUSD { .. } => "Export USD",
            NodeType::Export3MF { .. } => "Export 3MF",
            NodeType::ExportPLY { .. } => "Export PLY",
            NodeType::ExportDXF { .. } => "Export DXF",
            NodeType::ImportSTL { .. } => "Import STL",
            NodeType::SketchSolve => "Sketch Solve",
            NodeType::AssemblySolve => "Assembly Solve",
            NodeType::CreateDrawing { .. } => "Create Drawing",
            NodeType::ExportPDF { .. } => "Export PDF",
            NodeType::ExportSVG { .. } => "Export SVG",
            NodeType::FEASolve { .. } => "FEA Solve",
            NodeType::GenerateToolpath { .. } => "Generate Toolpath",
            NodeType::ExportGCode { .. } => "Export G-Code",
            NodeType::UnfoldSheet { .. } => "Unfold Sheet",
            NodeType::ToImplicit => "To Implicit",
            NodeType::SdfUnion => "SDF Union",
            NodeType::SdfSubtract => "SDF Subtract",
            NodeType::SdfIntersect => "SDF Intersect",
            NodeType::DualContourMesh { .. } => "Dual Contour Mesh",
            NodeType::SubDSubdivide { .. } => "SubD Subdivide",
            NodeType::SubDToNurbs => "SubD To NURBS",
            NodeType::GPUEvalNurbs => "GPU Eval NURBS",
            NodeType::GeometryHash => "Geometry Hash",
            NodeType::MerkleVerify => "Merkle Verify",
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
            // Phase I (Optional/Advanced): Primitives & Curves & Transforms
            NodeType::PlanePrimitive { .. } | NodeType::PolygonPrism { .. } |
            NodeType::Tube { .. } | NodeType::Wedge { .. } |
            NodeType::Tetra { .. } | NodeType::Octa { .. } | NodeType::Icosa { .. } => "Primitives",
            NodeType::Helix { .. } | NodeType::Hyperbola { .. } | NodeType::Parabola { .. } => "Curve",
            NodeType::Shear { .. } | NodeType::Taper { .. } | NodeType::ApplyTransform |
            NodeType::ArrayPolar { .. } | NodeType::Offset { .. } => "Transform",
            // Extended Curve
            NodeType::CurveBooleanUnion | NodeType::CurveBooleanSubtract |
            NodeType::CurveBooleanIntersect | NodeType::CurveShatter |
            NodeType::CurveDiscontinuity { .. } | NodeType::CurveFrame |
            NodeType::CurveNormal | NodeType::ProjectCurveToPlane |
            NodeType::ProjectCurveToSurface { .. } | NodeType::CurveSeam => "Curve",
            // Extended Surface
            NodeType::CylinderSurface { .. } | NodeType::ConeSurface { .. } |
            NodeType::SphereSurface { .. } | NodeType::TorusSurface { .. } |
            NodeType::OffsetSurface { .. } => "Surface",
            // Extended Intersect
            NodeType::BrepBrepIntersect | NodeType::MeshMeshIntersect |
            NodeType::LineLineClosestPoint | NodeType::SolidInclusion |
            NodeType::CollisionCheck | NodeType::BooleanTrim |
            NodeType::MeshBooleanUnion | NodeType::MeshBooleanSubtract |
            NodeType::MeshBooleanIntersect => "Intersect",
            // Extended Analysis
            NodeType::MomentsOfInertia | NodeType::CurveCurvatureAnalysis { .. } |
            NodeType::SurfaceCurvatureAnalysis { .. } | NodeType::PointInCurve |
            NodeType::ClosestPointOnSurface | NodeType::ClosestPointOnCurve |
            NodeType::SelfIntersect | NodeType::Planar | NodeType::Closed => "Analysis",
            // Extended Params
            NodeType::TransformInput { .. } | NodeType::ColorInput { .. } |
            NodeType::FileInput { .. } | NodeType::PathInput { .. } => "Params",
            NodeType::ListMap { .. } => "Tree",
            // Kernel API Wrappers
            NodeType::HealSolid | NodeType::StitchSolids { .. } |
            NodeType::ValidateSolid => "Modify",
            NodeType::ExportGLTF { .. } | NodeType::ExportUSD { .. } |
            NodeType::Export3MF { .. } | NodeType::ExportPLY { .. } |
            NodeType::ExportDXF { .. } | NodeType::ExportPDF { .. } |
            NodeType::ExportSVG { .. } | NodeType::ExportGCode { .. } |
            NodeType::ImportSTL { .. } | NodeType::CreateDrawing { .. } => "Output",
            NodeType::SketchSolve | NodeType::AssemblySolve |
            NodeType::FEASolve { .. } | NodeType::GenerateToolpath { .. } |
            NodeType::UnfoldSheet { .. } | NodeType::GeometryHash |
            NodeType::MerkleVerify => "Analysis",
            NodeType::ToImplicit | NodeType::SdfUnion | NodeType::SdfSubtract |
            NodeType::SdfIntersect | NodeType::DualContourMesh { .. } => "Implicit",
            NodeType::SubDSubdivide { .. } | NodeType::SubDToNurbs => "SubD",
            NodeType::GPUEvalNurbs => "Compute",
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
            // Phase I (Optional/Advanced): Primitives
            NodeType::PlanePrimitive { .. } => vec![],
            NodeType::PolygonPrism { .. } => vec![],
            NodeType::Tube { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::Helix { .. } => vec![],
            NodeType::Wedge { .. } => vec![],
            NodeType::Tetra { .. } | NodeType::Octa { .. } | NodeType::Icosa { .. } => vec![],
            // Phase I: Curves
            NodeType::Hyperbola { .. } | NodeType::Parabola { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Point },
                PortDesc { name: "N", port_type: PortType::Vector },
            ],
            // Phase I: Transforms
            NodeType::Shear { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::Taper { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "A", port_type: PortType::Vector },
            ],
            NodeType::ApplyTransform => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "T", port_type: PortType::Transform },
            ],
            NodeType::ArrayPolar { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "C", port_type: PortType::Point },
                PortDesc { name: "A", port_type: PortType::Vector },
            ],
            NodeType::Offset { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "N", port_type: PortType::Vector },
            ],
            // Extended Curve inputs
            NodeType::CurveBooleanUnion | NodeType::CurveBooleanSubtract |
            NodeType::CurveBooleanIntersect => vec![
                PortDesc { name: "A", port_type: PortType::List },
                PortDesc { name: "B", port_type: PortType::List },
            ],
            NodeType::CurveShatter | NodeType::CurveDiscontinuity { .. } |
            NodeType::ProjectCurveToPlane | NodeType::CurveSeam => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::CurveFrame | NodeType::CurveNormal => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "T", port_type: PortType::Number },
            ],
            NodeType::ProjectCurveToSurface { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            // Extended Surface inputs
            NodeType::CylinderSurface { .. } | NodeType::ConeSurface { .. } |
            NodeType::TorusSurface { .. } => vec![
                PortDesc { name: "P", port_type: PortType::PlaneRef },
            ],
            NodeType::SphereSurface { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Point },
            ],
            NodeType::OffsetSurface { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            // Extended Intersect inputs
            NodeType::BrepBrepIntersect | NodeType::MeshMeshIntersect |
            NodeType::LineLineClosestPoint | NodeType::CollisionCheck => vec![
                PortDesc { name: "A", port_type: PortType::Any },
                PortDesc { name: "B", port_type: PortType::Any },
            ],
            NodeType::SolidInclusion | NodeType::PointInCurve => vec![
                PortDesc { name: "G", port_type: PortType::Any },
                PortDesc { name: "P", port_type: PortType::Point },
            ],
            NodeType::BooleanTrim => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "C", port_type: PortType::List },
            ],
            NodeType::MeshBooleanUnion | NodeType::MeshBooleanSubtract |
            NodeType::MeshBooleanIntersect => vec![
                PortDesc { name: "A", port_type: PortType::Mesh },
                PortDesc { name: "B", port_type: PortType::Mesh },
            ],
            // Extended Analysis inputs
            NodeType::MomentsOfInertia => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::CurveCurvatureAnalysis { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            NodeType::SurfaceCurvatureAnalysis { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::ClosestPointOnSurface => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
                PortDesc { name: "P", port_type: PortType::Point },
            ],
            NodeType::ClosestPointOnCurve => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "P", port_type: PortType::Point },
            ],
            NodeType::SelfIntersect => vec![
                PortDesc { name: "C", port_type: PortType::Any },
            ],
            NodeType::Planar | NodeType::Closed => vec![
                PortDesc { name: "C", port_type: PortType::Any },
            ],
            // Extended Params inputs (none — these are pure parameter nodes)
            NodeType::TransformInput { .. } | NodeType::ColorInput { .. } |
            NodeType::FileInput { .. } | NodeType::PathInput { .. } => vec![],
            NodeType::ListMap { .. } => vec![
                PortDesc { name: "L", port_type: PortType::List },
            ],
            // Kernel API Wrappers — inputs
            NodeType::HealSolid => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
            NodeType::StitchSolids { .. } => vec![
                PortDesc { name: "A", port_type: PortType::Geometry },
                PortDesc { name: "B", port_type: PortType::Geometry },
            ],
            NodeType::ValidateSolid => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
            NodeType::ExportGLTF { .. } | NodeType::ExportUSD { .. } |
            NodeType::Export3MF { .. } | NodeType::ExportPLY { .. } |
            NodeType::ExportDXF { .. } => vec![PortDesc { name: "M", port_type: PortType::Mesh }],
            NodeType::ImportSTL { .. } => vec![],
            NodeType::SketchSolve => vec![PortDesc { name: "S", port_type: PortType::Any }],
            NodeType::AssemblySolve => vec![PortDesc { name: "A", port_type: PortType::Any }],
            NodeType::CreateDrawing { .. } => vec![PortDesc { name: "M", port_type: PortType::Mesh }],
            NodeType::ExportPDF { .. } | NodeType::ExportSVG { .. } => vec![
                PortDesc { name: "D", port_type: PortType::Any },
            ],
            NodeType::FEASolve { .. } => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
            NodeType::GenerateToolpath { .. } => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
            NodeType::ExportGCode { .. } => vec![PortDesc { name: "T", port_type: PortType::Any }],
            NodeType::UnfoldSheet { .. } => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
            NodeType::ToImplicit => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
            NodeType::SdfUnion | NodeType::SdfSubtract | NodeType::SdfIntersect => vec![
                PortDesc { name: "A", port_type: PortType::Any },
                PortDesc { name: "B", port_type: PortType::Any },
            ],
            NodeType::DualContourMesh { .. } => vec![PortDesc { name: "S", port_type: PortType::Any }],
            NodeType::SubDSubdivide { .. } => vec![PortDesc { name: "M", port_type: PortType::Mesh }],
            NodeType::SubDToNurbs => vec![PortDesc { name: "M", port_type: PortType::Mesh }],
            NodeType::GPUEvalNurbs => vec![PortDesc { name: "S", port_type: PortType::Surface }],
            NodeType::GeometryHash | NodeType::MerkleVerify => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
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
            // Phase I (Optional/Advanced): Primitives
            NodeType::PlanePrimitive { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::PolygonPrism { .. } | NodeType::Tube { .. } | NodeType::Wedge { .. } |
            NodeType::Tetra { .. } | NodeType::Octa { .. } | NodeType::Icosa { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::Helix { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            // Phase I: Curves
            NodeType::Hyperbola { .. } | NodeType::Parabola { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
            ],
            // Phase I: Transforms
            NodeType::Shear { .. } | NodeType::Taper { .. } | NodeType::ApplyTransform |
            NodeType::ArrayPolar { .. } | NodeType::Offset { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            // Extended Curve outputs
            NodeType::CurveBooleanUnion | NodeType::CurveBooleanSubtract |
            NodeType::CurveBooleanIntersect | NodeType::CurveShatter |
            NodeType::CurveDiscontinuity { .. } | NodeType::ProjectCurveToPlane |
            NodeType::ProjectCurveToSurface { .. } | NodeType::CurveSeam => vec![
                PortDesc { name: "R", port_type: PortType::Curve },
            ],
            NodeType::CurveFrame => vec![
                PortDesc { name: "F", port_type: PortType::PlaneRef },
            ],
            NodeType::CurveNormal => vec![
                PortDesc { name: "N", port_type: PortType::Vector },
            ],
            // Extended Surface outputs
            NodeType::CylinderSurface { .. } | NodeType::ConeSurface { .. } |
            NodeType::SphereSurface { .. } | NodeType::TorusSurface { .. } |
            NodeType::OffsetSurface { .. } => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            // Extended Intersect outputs
            NodeType::BrepBrepIntersect => vec![
                PortDesc { name: "C", port_type: PortType::List },
                PortDesc { name: "P", port_type: PortType::List },
            ],
            NodeType::MeshMeshIntersect => vec![
                PortDesc { name: "L", port_type: PortType::List },
            ],
            NodeType::LineLineClosestPoint => vec![
                PortDesc { name: "PA", port_type: PortType::Point },
                PortDesc { name: "PB", port_type: PortType::Point },
                PortDesc { name: "D", port_type: PortType::Number },
            ],
            NodeType::SolidInclusion | NodeType::PointInCurve | NodeType::CollisionCheck |
            NodeType::Planar | NodeType::Closed | NodeType::SelfIntersect => vec![
                PortDesc { name: "B", port_type: PortType::Boolean },
            ],
            NodeType::BooleanTrim => vec![
                PortDesc { name: "R", port_type: PortType::Geometry },
            ],
            NodeType::MeshBooleanUnion | NodeType::MeshBooleanSubtract |
            NodeType::MeshBooleanIntersect => vec![
                PortDesc { name: "M", port_type: PortType::Mesh },
            ],
            // Extended Analysis outputs
            NodeType::MomentsOfInertia => vec![
                PortDesc { name: "Ixx", port_type: PortType::Number },
                PortDesc { name: "Iyy", port_type: PortType::Number },
                PortDesc { name: "Izz", port_type: PortType::Number },
                PortDesc { name: "Ixy", port_type: PortType::Number },
                PortDesc { name: "Ixz", port_type: PortType::Number },
                PortDesc { name: "Iyz", port_type: PortType::Number },
            ],
            NodeType::CurveCurvatureAnalysis { .. } => vec![
                PortDesc { name: "T", port_type: PortType::List },
                PortDesc { name: "K", port_type: PortType::List },
                PortDesc { name: "R", port_type: PortType::List },
            ],
            NodeType::SurfaceCurvatureAnalysis { .. } => vec![
                PortDesc { name: "K1", port_type: PortType::List },
                PortDesc { name: "K2", port_type: PortType::List },
                PortDesc { name: "G", port_type: PortType::List },
                PortDesc { name: "M", port_type: PortType::List },
            ],
            NodeType::ClosestPointOnSurface | NodeType::ClosestPointOnCurve => vec![
                PortDesc { name: "Q", port_type: PortType::Point },
                PortDesc { name: "T", port_type: PortType::Number },
                PortDesc { name: "D", port_type: PortType::Number },
            ],
            // Extended Params outputs
            NodeType::TransformInput { .. } => vec![
                PortDesc { name: "T", port_type: PortType::Transform },
            ],
            NodeType::ColorInput { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Color },
            ],
            NodeType::FileInput { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::PathInput { .. } => vec![
                PortDesc { name: "P", port_type: PortType::Any },
            ],
            NodeType::ListMap { .. } => vec![
                PortDesc { name: "R", port_type: PortType::List },
            ],
            // Kernel API Wrappers — outputs
            NodeType::HealSolid | NodeType::StitchSolids { .. } |
            NodeType::UnfoldSheet { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            NodeType::ValidateSolid => vec![
                PortDesc { name: "B", port_type: PortType::Boolean },
            ],
            NodeType::ExportGLTF { .. } | NodeType::ExportUSD { .. } |
            NodeType::Export3MF { .. } | NodeType::ExportPLY { .. } |
            NodeType::ExportDXF { .. } | NodeType::ExportPDF { .. } |
            NodeType::ExportSVG { .. } | NodeType::ExportGCode { .. } => vec![
                PortDesc { name: "S", port_type: PortType::String },
            ],
            NodeType::ImportSTL { .. } => vec![
                PortDesc { name: "M", port_type: PortType::Mesh },
            ],
            NodeType::SketchSolve => vec![
                PortDesc { name: "S", port_type: PortType::Any },
            ],
            NodeType::AssemblySolve => vec![
                PortDesc { name: "A", port_type: PortType::Any },
            ],
            NodeType::CreateDrawing { .. } => vec![
                PortDesc { name: "D", port_type: PortType::Any },
            ],
            NodeType::FEASolve { .. } => vec![
                PortDesc { name: "R", port_type: PortType::List },
            ],
            NodeType::GenerateToolpath { .. } => vec![
                PortDesc { name: "T", port_type: PortType::Any },
            ],
            NodeType::ToImplicit | NodeType::SdfUnion | NodeType::SdfSubtract |
            NodeType::SdfIntersect => vec![
                PortDesc { name: "I", port_type: PortType::Any },
            ],
            NodeType::DualContourMesh { .. } => vec![
                PortDesc { name: "M", port_type: PortType::Mesh },
            ],
            NodeType::SubDSubdivide { .. } => vec![
                PortDesc { name: "M", port_type: PortType::Mesh },
            ],
            NodeType::SubDToNurbs => vec![
                PortDesc { name: "S", port_type: PortType::Surface },
            ],
            NodeType::GPUEvalNurbs => vec![
                PortDesc { name: "P", port_type: PortType::List },
            ],
            NodeType::GeometryHash => vec![
                PortDesc { name: "H", port_type: PortType::String },
            ],
            NodeType::MerkleVerify => vec![
                PortDesc { name: "B", port_type: PortType::Boolean },
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

// ============================================================
// VP Graph Serialization (Save / Load / Import / Export)
// ============================================================
//
// JSON format (human-readable, no serde dependency):
//
// ```json
// {
//   "version": 1,
//   "next_id": 42,
//   "nodes": [
//     { "id": 0, "type": "NumberSlider", "fields": {"value": 42.0, "min": 0.0, "max": 100.0}, "x": 0.0, "y": 0.0 },
//     { "id": 1, "type": "Box", "fields": {"width": 10.0, "height": 5.0, "depth": 3.0}, "x": 100.0, "y": 0.0 }
//   ],
//   "connections": [
//     { "from_node": 0, "from_port": 0, "to_node": 1, "to_port": 0 }
//   ]
// }
// ```

impl VpGraph {
    /// Serialize the entire graph to a JSON string.
    pub fn to_json(&self) -> String {
        let mut s = String::with_capacity(4096);
        s.push_str("{\n");
        s.push_str(&format!("  \"version\": 1,\n"));
        s.push_str(&format!("  \"next_id\": {},\n", self.next_id));

        // Nodes
        s.push_str("  \"nodes\": [\n");
        for (i, node) in self.nodes.iter().enumerate() {
            s.push_str("    {");
            s.push_str(&format!("\"id\": {}, ", node.id));
            let (type_name, fields_json) = node_type_to_json(&node.node_type);
            s.push_str(&format!("\"type\": \"{}\", ", type_name));
            s.push_str(&format!("\"fields\": {}, ", fields_json));
            s.push_str(&format!("\"x\": {}, \"y\": {}", node.x, node.y));
            s.push_str("}");
            if i < self.nodes.len() - 1 { s.push(','); }
            s.push('\n');
        }
        s.push_str("  ],\n");

        // Connections
        s.push_str("  \"connections\": [\n");
        for (i, conn) in self.connections.iter().enumerate() {
            s.push_str("    {");
            s.push_str(&format!(
                "\"from_node\": {}, \"from_port\": {}, \"to_node\": {}, \"to_port\": {}",
                conn.from_node, conn.from_port, conn.to_node, conn.to_port
            ));
            s.push_str("}");
            if i < self.connections.len() - 1 { s.push(','); }
            s.push('\n');
        }
        s.push_str("  ]\n");
        s.push_str("}\n");
        s
    }

    /// Deserialize a graph from a JSON string.
    pub fn from_json(json: &str) -> Result<Self, String> {
        let mut graph = VpGraph::new();

        // Parse next_id
        if let Some(val) = extract_json_value(json, "\"next_id\"") {
            graph.next_id = val.trim().trim_end_matches(',').parse::<u64>().unwrap_or(0);
        }

        // Parse nodes
        let nodes_str = extract_json_array(json, "\"nodes\"").unwrap_or_default();
        for node_str in split_json_objects(&nodes_str) {
            let id = extract_json_value(&node_str, "\"id\"")
                .and_then(|v| v.trim().trim_end_matches(',').parse::<u64>().ok())
                .ok_or("Missing node id")?;
            let type_name = extract_json_string(&node_str, "\"type\"")
                .ok_or("Missing node type")?;
            let fields_str = extract_json_object(&node_str, "\"fields\"")
                .unwrap_or_default();
            let x = extract_json_value(&node_str, "\"x\"")
                .and_then(|v| v.trim().trim_end_matches(',').parse::<f32>().ok())
                .unwrap_or(0.0);
            let y = extract_json_value(&node_str, "\"y\"")
                .and_then(|v| v.trim().trim_end_matches(',').parse::<f32>().ok())
                .unwrap_or(0.0);
            let node_type = node_type_from_json(&type_name, &fields_str)?;
            graph.nodes.push(VpNode { id, node_type, x, y });
        }

        // Parse connections
        let conns_str = extract_json_array(json, "\"connections\"").unwrap_or_default();
        for conn_str in split_json_objects(&conns_str) {
            let from_node = extract_json_value(&conn_str, "\"from_node\"")
                .and_then(|v| v.trim().trim_end_matches(',').parse::<u64>().ok())
                .ok_or("Missing from_node")?;
            let from_port = extract_json_value(&conn_str, "\"from_port\"")
                .and_then(|v| v.trim().trim_end_matches(',').parse::<usize>().ok())
                .ok_or("Missing from_port")?;
            let to_node = extract_json_value(&conn_str, "\"to_node\"")
                .and_then(|v| v.trim().trim_end_matches(',').parse::<u64>().ok())
                .ok_or("Missing to_node")?;
            let to_port = extract_json_value(&conn_str, "\"to_port\"")
                .and_then(|v| v.trim().trim_end_matches(',').parse::<usize>().ok())
                .ok_or("Missing to_port")?;
            graph.connections.push(VpConnection {
                from_node, from_port, to_node, to_port,
            });
        }

        Ok(graph)
    }

    /// Save the entire graph to a file.
    pub fn save_to_file(&self, path: &str) -> Result<(), String> {
        let json = self.to_json();
        std::fs::write(path, json).map_err(|e| format!("Failed to write file: {}", e))
    }

    /// Load a graph from a file. Replaces the current graph.
    pub fn load_from_file(path: &str) -> Result<Self, String> {
        let json = std::fs::read_to_string(path).map_err(|e| format!("Failed to read file: {}", e))?;
        Self::from_json(&json)
    }

    /// Export selected nodes (and their internal connections) to a JSON string.
    ///
    /// Node IDs are remapped to start from 0. Connections between selected
    /// nodes are preserved; connections to unselected nodes are dropped.
    pub fn export_selected(&self, selected_ids: &[u64]) -> String {
        let selected_set: std::collections::HashSet<u64> = selected_ids.iter().copied().collect();

        // Build ID remap (old → new)
        let mut id_remap: std::collections::HashMap<u64, u64> = std::collections::HashMap::new();
        for (i, &old_id) in selected_ids.iter().enumerate() {
            id_remap.insert(old_id, i as u64);
        }

        let mut s = String::with_capacity(2048);
        s.push_str("{\n");
        s.push_str("  \"version\": 1,\n");
        s.push_str(&format!("  \"next_id\": {},\n", selected_ids.len()));

        // Nodes (remapped)
        s.push_str("  \"nodes\": [\n");
        for (i, &old_id) in selected_ids.iter().enumerate() {
            if let Some(node) = self.nodes.iter().find(|n| n.id == old_id) {
                let new_id = id_remap[&old_id];
                let (type_name, fields_json) = node_type_to_json(&node.node_type);
                s.push_str("    {");
                s.push_str(&format!("\"id\": {}, ", new_id));
                s.push_str(&format!("\"type\": \"{}\", ", type_name));
                s.push_str(&format!("\"fields\": {}, ", fields_json));
                s.push_str(&format!("\"x\": {}, \"y\": {}", node.x, node.y));
                s.push_str("}");
                if i < selected_ids.len() - 1 { s.push(','); }
                s.push('\n');
            }
        }
        s.push_str("  ],\n");

        // Connections (only between selected nodes, remapped)
        s.push_str("  \"connections\": [\n");
        let filtered_conns: Vec<&VpConnection> = self.connections.iter()
            .filter(|c| selected_set.contains(&c.from_node) && selected_set.contains(&c.to_node))
            .collect();
        for (i, conn) in filtered_conns.iter().enumerate() {
            let new_from = id_remap[&conn.from_node];
            let new_to = id_remap[&conn.to_node];
            s.push_str("    {");
            s.push_str(&format!(
                "\"from_node\": {}, \"from_port\": {}, \"to_node\": {}, \"to_port\": {}",
                new_from, conn.from_port, new_to, conn.to_port
            ));
            s.push_str("}");
            if i < filtered_conns.len() - 1 { s.push(','); }
            s.push('\n');
        }
        s.push_str("  ]\n");
        s.push_str("}\n");
        s
    }

    /// Import nodes from a JSON string into this graph.
    ///
    /// Imported nodes get new IDs (appended to the current graph's ID space).
    /// Connections are remapped to the new IDs.
    /// Returns the list of new node IDs.
    pub fn import_from_json(&mut self, json: &str) -> Result<Vec<u64>, String> {
        let sub_graph = VpGraph::from_json(json)?;

        // Build ID remap (old sub_graph ID → new ID in self)
        let mut id_remap: std::collections::HashMap<u64, u64> = std::collections::HashMap::new();
        let mut new_ids = Vec::new();

        for node in &sub_graph.nodes {
            let new_id = self.next_id;
            self.next_id += 1;
            id_remap.insert(node.id, new_id);
            new_ids.push(new_id);
            self.nodes.push(VpNode {
                id: new_id,
                node_type: node.node_type.clone(),
                x: node.x,
                y: node.y,
            });
        }

        // Import connections with remapped IDs
        for conn in &sub_graph.connections {
            if let (Some(&new_from), Some(&new_to)) = (
                id_remap.get(&conn.from_node),
                id_remap.get(&conn.to_node),
            ) {
                self.connections.push(VpConnection {
                    from_node: new_from,
                    from_port: conn.from_port,
                    to_node: new_to,
                    to_port: conn.to_port,
                });
            }
        }

        Ok(new_ids)
    }

    /// Get all node IDs.
    pub fn all_node_ids(&self) -> Vec<u64> {
        self.nodes.iter().map(|n| n.id).collect()
    }

    /// Remove a node and all its connections by ID.
    pub fn remove_node(&mut self, id: u64) -> bool {
        let before = self.nodes.len();
        self.nodes.retain(|n| n.id != id);
        let removed = self.nodes.len() < before;
        if removed {
            self.connections.retain(|c| c.from_node != id && c.to_node != id);
        }
        removed
    }

    /// Clear the entire graph.
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.connections.clear();
        self.next_id = 0;
    }
}

// ── JSON helpers ──

/// Extract a value string after a key in JSON.
/// Returns the raw text between the colon after the key and the next comma/newline.
fn extract_json_value(json: &str, key: &str) -> Option<String> {
    let pos = json.find(key)?;
    let rest = &json[pos + key.len()..];
    let colon_pos = rest.find(':')?;
    let after_colon = &rest[colon_pos + 1..];
    let trimmed = after_colon.trim_start();
    let end = trimmed.find(|c: char| c == ',' || c == '\n' || c == '}').unwrap_or(trimmed.len());
    Some(trimmed[..end].trim().to_string())
}

/// Extract a quoted string value after a key.
fn extract_json_string(json: &str, key: &str) -> Option<String> {
    let val = extract_json_value(json, key)?;
    let val = val.trim();
    if val.starts_with('"') {
        // Find the closing quote (not the opening one)
        let after_open = &val[1..];
        if let Some(end) = after_open.find('"') {
            return Some(after_open[..end].to_string());
        }
    }
    None
}

/// Extract the content of a JSON array after a key.
/// Returns the text between the opening [ and matching ].
fn extract_json_array(json: &str, key: &str) -> Option<String> {
    let pos = json.find(key)?;
    let rest = &json[pos + key.len()..];
    let bracket_pos = rest.find('[')?;
    let after_open = &rest[bracket_pos + 1..];
    // Find matching ] (accounting for nesting)
    let mut depth = 1;
    let mut end = 0;
    for (i, ch) in after_open.char_indices() {
        match ch {
            '[' => depth += 1,
            ']' => {
                depth -= 1;
                if depth == 0 { end = i; break; }
            }
            _ => {}
        }
    }
    Some(after_open[..end].to_string())
}

/// Extract the content of a JSON object after a key.
/// Returns the text between the opening { and matching }.
fn extract_json_object(json: &str, key: &str) -> Option<String> {
    let pos = json.find(key)?;
    let rest = &json[pos + key.len()..];
    let colon_pos = rest.find(':')?;
    let after_colon = &rest[colon_pos + 1..].trim_start();
    if !after_colon.starts_with('{') { return None; }
    let after_open = &after_colon[1..];
    let mut depth = 1;
    let mut end = 0;
    for (i, ch) in after_open.char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 { end = i; break; }
            }
            _ => {}
        }
    }
    Some(after_open[..end].to_string())
}

/// Split a JSON array body into individual object strings.
fn split_json_objects(array_body: &str) -> Vec<String> {
    let mut objects = Vec::new();
    let mut depth = 0;
    let mut start = 0;
    for (i, ch) in array_body.char_indices() {
        match ch {
            '{' => {
                if depth == 0 { start = i; }
                depth += 1;
            }
            '}' => {
                depth -= 1;
                if depth == 0 {
                    objects.push(array_body[start..=i].to_string());
                }
            }
            _ => {}
        }
    }
    objects
}

/// Extract a float field from a JSON object string.
fn json_float(obj: &str, key: &str) -> Option<f64> {
    let needle = format!("\"{}\"", key);
    let pos = obj.find(&needle)?;
    let rest = &obj[pos + needle.len()..];
    let colon = rest.find(':')?;
    let val = &rest[colon + 1..];
    let trimmed = val.trim_start();
    let end = trimmed.find(|c: char| c == ',' || c == '}').unwrap_or(trimmed.len());
    trimmed[..end].trim().parse::<f64>().ok()
}

/// Extract a u32 field from a JSON object string.
fn json_u32(obj: &str, key: &str) -> Option<u32> {
    json_float(obj, key).map(|v| v as u32)
}

/// Extract a bool field from a JSON object string.
fn json_bool(obj: &str, key: &str) -> Option<bool> {
    let needle = format!("\"{}\"", key);
    let pos = obj.find(&needle)?;
    let rest = &obj[pos + needle.len()..];
    let colon = rest.find(':')?;
    let val = &rest[colon + 1..].trim_start();
    if val.starts_with("true") { Some(true) }
    else if val.starts_with("false") { Some(false) }
    else { None }
}

/// Extract a string field from a JSON object string.
fn json_string(obj: &str, key: &str) -> Option<String> {
    let needle = format!("\"{}\"", key);
    let pos = obj.find(&needle)?;
    let rest = &obj[pos + needle.len()..];
    let colon = rest.find(':')?;
    let val = &rest[colon + 1..].trim_start();
    if val.starts_with('"') {
        let inner = &val[1..];
        if let Some(end) = inner.find('"') {
            return Some(inner[..end].to_string());
        }
    }
    None
}

/// Extract an i64 field from a JSON object string.
fn json_i64(obj: &str, key: &str) -> Option<i64> {
    json_float(obj, key).map(|v| v as i64)
}

// ── NodeType serialization ──

/// Convert a NodeType to (type_name, fields_json).
fn node_type_to_json(nt: &NodeType) -> (&'static str, String) {
    match nt {
        NodeType::NumberSlider { value, min, max } => ("NumberSlider", format!(
            "{{\"value\":{},\"min\":{},\"max\":{}}}", value, min, max)),
        NodeType::IntegerInput { value } => ("IntegerInput", format!("{{\"value\":{}}}", value)),
        NodeType::BooleanToggle { value } => ("BooleanToggle", format!("{{\"value\":{}}}", value)),
        NodeType::PointInput { x, y, z } => ("PointInput", format!("{{\"x\":{},\"y\":{},\"z\":{}}}", x, y, z)),
        NodeType::VectorInput { x, y, z } => ("VectorInput", format!("{{\"x\":{},\"y\":{},\"z\":{}}}", x, y, z)),
        NodeType::Panel => ("Panel", "{}".to_string()),
        NodeType::Add => ("Add", "{}".to_string()),
        NodeType::Subtract => ("Subtract", "{}".to_string()),
        NodeType::Multiply => ("Multiply", "{}".to_string()),
        NodeType::Divide => ("Divide", "{}".to_string()),
        NodeType::Sin => ("Sin", "{}".to_string()),
        NodeType::Cos => ("Cos", "{}".to_string()),
        NodeType::Tan => ("Tan", "{}".to_string()),
        NodeType::Abs => ("Abs", "{}".to_string()),
        NodeType::Sqrt => ("Sqrt", "{}".to_string()),
        NodeType::Pow => ("Pow", "{}".to_string()),
        NodeType::Round => ("Round", "{}".to_string()),
        NodeType::Min => ("Min", "{}".to_string()),
        NodeType::Max => ("Max", "{}".to_string()),
        NodeType::Average => ("Average", "{}".to_string()),
        NodeType::Expression { expr } => ("Expression", format!("{{\"expr\":\"{}\"}}", escape_json(&expr))),
        NodeType::Series { start, step, count } => ("Series", format!(
            "{{\"start\":{},\"step\":{},\"count\":{}}}", start, step, count)),
        NodeType::Range { domain_min, domain_max, count } => ("Range", format!(
            "{{\"domain_min\":{},\"domain_max\":{},\"count\":{}}}", domain_min, domain_max, count)),
        NodeType::ListLength => ("ListLength", "{}".to_string()),
        NodeType::ListItem => ("ListItem", "{}".to_string()),
        NodeType::Reverse => ("Reverse", "{}".to_string()),
        NodeType::Sort => ("Sort", "{}".to_string()),
        NodeType::CullPattern => ("CullPattern", "{}".to_string()),
        NodeType::Box { width, height, depth } => ("Box", format!(
            "{{\"width\":{},\"height\":{},\"depth\":{}}}", width, height, depth)),
        NodeType::Sphere { radius } => ("Sphere", format!("{{\"radius\":{}}}", radius)),
        NodeType::Cylinder { radius, height } => ("Cylinder", format!("{{\"radius\":{},\"height\":{}}}", radius, height)),
        NodeType::Cone { bottom_radius, top_radius, height } => ("Cone", format!(
            "{{\"bottom_radius\":{},\"top_radius\":{},\"height\":{}}}", bottom_radius, top_radius, height)),
        NodeType::Torus { major_radius, minor_radius } => ("Torus", format!(
            "{{\"major_radius\":{},\"minor_radius\":{}}}", major_radius, minor_radius)),
        NodeType::Line => ("Line", "{}".to_string()),
        NodeType::Circle { radius } => ("Circle", format!("{{\"radius\":{}}}", radius)),
        NodeType::DivideCurve { count } => ("DivideCurve", format!("{{\"count\":{}}}", count)),
        NodeType::EvaluateCurve => ("EvaluateCurve", "{}".to_string()),
        NodeType::CurveLength => ("CurveLength", "{}".to_string()),
        NodeType::Move { x, y, z } => ("Move", format!("{{\"x\":{},\"y\":{},\"z\":{}}}", x, y, z)),
        NodeType::Rotate { x_deg, y_deg, z_deg } => ("Rotate", format!(
            "{{\"x_deg\":{},\"y_deg\":{},\"z_deg\":{}}}", x_deg, y_deg, z_deg)),
        NodeType::Scale { x, y, z } => ("Scale", format!("{{\"x\":{},\"y\":{},\"z\":{}}}", x, y, z)),
        NodeType::Mirror { plane } => ("Mirror", format!(
            "{{\"plane\":\"{}\"}}", match plane {
                MirrorPlane::XY => "XY", MirrorPlane::XZ => "XZ", MirrorPlane::YZ => "YZ",
            })),
        NodeType::LinearArray { count, spacing } => ("LinearArray", format!(
            "{{\"count\":{},\"spacing\":{}}}", count, spacing)),
        NodeType::CircularArray { count, angle } => ("CircularArray", format!(
            "{{\"count\":{},\"angle\":{}}}", count, angle)),
        NodeType::BooleanUnion => ("BooleanUnion", "{}".to_string()),
        NodeType::BooleanSubtract => ("BooleanSubtract", "{}".to_string()),
        NodeType::BooleanIntersect => ("BooleanIntersect", "{}".to_string()),
        NodeType::Fillet { radius } => ("Fillet", format!("{{\"radius\":{}}}", radius)),
        NodeType::Chamfer { distance } => ("Chamfer", format!("{{\"distance\":{}}}", distance)),
        NodeType::Graft => ("Graft", "{}".to_string()),
        NodeType::Flatten => ("Flatten", "{}".to_string()),
        NodeType::CrossRef => ("CrossRef", "{}".to_string()),
        NodeType::ShiftList { amount } => ("ShiftList", format!("{{\"amount\":{}}}", amount)),
        NodeType::Subset { start, count } => ("Subset", format!(
            "{{\"start\":{},\"count\":{}}}", start, count)),
        NodeType::Dispatch => ("Dispatch", "{}".to_string()),
        NodeType::Weave => ("Weave", "{}".to_string()),
        NodeType::Concat => ("Concat", "{}".to_string()),
        NodeType::BakeToDoc => ("BakeToDoc", "{}".to_string()),
        // Phase A-I: Extended nodes — serialize with fields
        NodeType::PlaneInput { ox, oy, oz, nx, ny, nz } => ("PlaneInput", format!(
            "{{\"ox\":{},\"oy\":{},\"oz\":{},\"nx\":{},\"ny\":{},\"nz\":{}}}", ox, oy, oz, nx, ny, nz)),
        NodeType::DomainInput { min, max } => ("DomainInput", format!("{{\"min\":{},\"max\":{}}}", min, max)),
        NodeType::StringInput { text } => ("StringInput", format!("{{\"text\":\"{}\"}}", escape_json(text))),
        NodeType::Volume => ("Volume", "{}".to_string()),
        NodeType::SurfaceArea => ("SurfaceArea", "{}".to_string()),
        NodeType::Centroid => ("Centroid", "{}".to_string()),
        NodeType::BoundingBox => ("BoundingBox", "{}".to_string()),
        NodeType::Distance => ("Distance", "{}".to_string()),
        NodeType::Angle => ("Angle", "{}".to_string()),
        NodeType::MassProperties => ("MassProperties", "{}".to_string()),
        NodeType::Cross => ("Cross", "{}".to_string()),
        NodeType::Dot => ("Dot", "{}".to_string()),
        NodeType::VectorLength => ("VectorLength", "{}".to_string()),
        NodeType::Unit => ("Unit", "{}".to_string()),
        NodeType::Negative => ("Negative", "{}".to_string()),
        NodeType::Reciprocal => ("Reciprocal", "{}".to_string()),
        NodeType::Asin => ("Asin", "{}".to_string()),
        NodeType::Acos => ("Acos", "{}".to_string()),
        NodeType::Atan => ("Atan", "{}".to_string()),
        NodeType::Atan2 => ("Atan2", "{}".to_string()),
        NodeType::Log => ("Log", "{}".to_string()),
        NodeType::Ln => ("Ln", "{}".to_string()),
        NodeType::Exp => ("Exp", "{}".to_string()),
        NodeType::Modulus => ("Modulus", "{}".to_string()),
        NodeType::MapDomain { source_min, source_max, target_min, target_max } => ("MapDomain",
            format!("{{\"source_min\":{},\"source_max\":{},\"target_min\":{},\"target_max\":{}}}",
                source_min, source_max, target_min, target_max)),
        NodeType::PointMidpoint => ("PointMidpoint", "{}".to_string()),
        NodeType::PointLerp { t } => ("PointLerp", format!("{{\"t\":{}}}", t)),
        NodeType::Vector2pt => ("Vector2pt", "{}".to_string()),
        NodeType::Extrude { distance } => ("Extrude", format!("{{\"distance\":{}}}", distance)),
        NodeType::Revolve { angle_deg } => ("Revolve", format!("{{\"angle_deg\":{}}}", angle_deg)),
        NodeType::Loft => ("Loft", "{}".to_string()),
        NodeType::Sweep => ("Sweep", "{}".to_string()),
        NodeType::RuledSurface => ("RuledSurface", "{}".to_string()),
        NodeType::PlaneSurface => ("PlaneSurface", "{}".to_string()),
        NodeType::ExtrudePoint => ("ExtrudePoint", "{}".to_string()),
        NodeType::ExtrudeTapered { distance, taper_deg } => ("ExtrudeTapered",
            format!("{{\"distance\":{},\"taper_deg\":{}}}", distance, taper_deg)),
        NodeType::EvaluateSurface => ("EvaluateSurface", "{}".to_string()),
        NodeType::SurfaceNormal => ("SurfaceNormal", "{}".to_string()),
        NodeType::Shell { thickness } => ("Shell", format!("{{\"thickness\":{}}}", thickness)),
        NodeType::Thicken { thickness } => ("Thicken", format!("{{\"thickness\":{}}}", thickness)),
        NodeType::OffsetSolid { distance } => ("OffsetSolid", format!("{{\"distance\":{}}}", distance)),
        NodeType::SplitSolid => ("SplitSolid", "{}".to_string()),
        NodeType::TrimSolid => ("TrimSolid", "{}".to_string()),
        NodeType::Hole { radius, depth } => ("Hole", format!("{{\"radius\":{},\"depth\":{}}}", radius, depth)),
        // Default for all remaining variants — store as type name only
        other => {
            let label = other.label();
            (label, "{}".to_string())
        }
    }
}

/// Parse a NodeType from (type_name, fields_json).
fn node_type_from_json(type_name: &str, fields: &str) -> Result<NodeType, String> {
    Ok(match type_name {
        "NumberSlider" => NodeType::NumberSlider {
            value: json_float(fields, "value").unwrap_or(0.0),
            min: json_float(fields, "min").unwrap_or(0.0),
            max: json_float(fields, "max").unwrap_or(100.0),
        },
        "IntegerInput" => NodeType::IntegerInput { value: json_i64(fields, "value").unwrap_or(0) },
        "BooleanToggle" => NodeType::BooleanToggle { value: json_bool(fields, "value").unwrap_or(false) },
        "PointInput" => NodeType::PointInput {
            x: json_float(fields, "x").unwrap_or(0.0),
            y: json_float(fields, "y").unwrap_or(0.0),
            z: json_float(fields, "z").unwrap_or(0.0),
        },
        "VectorInput" => NodeType::VectorInput {
            x: json_float(fields, "x").unwrap_or(0.0),
            y: json_float(fields, "y").unwrap_or(0.0),
            z: json_float(fields, "z").unwrap_or(0.0),
        },
        "Panel" => NodeType::Panel,
        "Add" => NodeType::Add,
        "Subtract" => NodeType::Subtract,
        "Multiply" => NodeType::Multiply,
        "Divide" => NodeType::Divide,
        "Sin" => NodeType::Sin,
        "Cos" => NodeType::Cos,
        "Tan" => NodeType::Tan,
        "Abs" => NodeType::Abs,
        "Sqrt" => NodeType::Sqrt,
        "Pow" => NodeType::Pow,
        "Round" => NodeType::Round,
        "Min" => NodeType::Min,
        "Max" => NodeType::Max,
        "Average" => NodeType::Average,
        "Expression" => NodeType::Expression { expr: json_string(fields, "expr").unwrap_or_default() },
        "Series" => NodeType::Series {
            start: json_float(fields, "start").unwrap_or(0.0),
            step: json_float(fields, "step").unwrap_or(1.0),
            count: json_u32(fields, "count").unwrap_or(10),
        },
        "Range" => NodeType::Range {
            domain_min: json_float(fields, "domain_min").unwrap_or(0.0),
            domain_max: json_float(fields, "domain_max").unwrap_or(1.0),
            count: json_u32(fields, "count").unwrap_or(10),
        },
        "ListLength" => NodeType::ListLength,
        "ListItem" => NodeType::ListItem,
        "Reverse" => NodeType::Reverse,
        "Sort" => NodeType::Sort,
        "CullPattern" => NodeType::CullPattern,
        "Box" => NodeType::Box {
            width: json_float(fields, "width").unwrap_or(1.0),
            height: json_float(fields, "height").unwrap_or(1.0),
            depth: json_float(fields, "depth").unwrap_or(1.0),
        },
        "Sphere" => NodeType::Sphere { radius: json_float(fields, "radius").unwrap_or(1.0) },
        "Cylinder" => NodeType::Cylinder {
            radius: json_float(fields, "radius").unwrap_or(1.0),
            height: json_float(fields, "height").unwrap_or(1.0),
        },
        "Cone" => NodeType::Cone {
            bottom_radius: json_float(fields, "bottom_radius").unwrap_or(1.0),
            top_radius: json_float(fields, "top_radius").unwrap_or(0.0),
            height: json_float(fields, "height").unwrap_or(1.0),
        },
        "Torus" => NodeType::Torus {
            major_radius: json_float(fields, "major_radius").unwrap_or(1.0),
            minor_radius: json_float(fields, "minor_radius").unwrap_or(0.1),
        },
        "Line" => NodeType::Line,
        "Circle" => NodeType::Circle { radius: json_float(fields, "radius").unwrap_or(1.0) },
        "DivideCurve" => NodeType::DivideCurve { count: json_u32(fields, "count").unwrap_or(10) },
        "EvaluateCurve" => NodeType::EvaluateCurve,
        "CurveLength" => NodeType::CurveLength,
        "Move" => NodeType::Move {
            x: json_float(fields, "x").unwrap_or(0.0),
            y: json_float(fields, "y").unwrap_or(0.0),
            z: json_float(fields, "z").unwrap_or(0.0),
        },
        "Rotate" => NodeType::Rotate {
            x_deg: json_float(fields, "x_deg").unwrap_or(0.0),
            y_deg: json_float(fields, "y_deg").unwrap_or(0.0),
            z_deg: json_float(fields, "z_deg").unwrap_or(0.0),
        },
        "Scale" => NodeType::Scale {
            x: json_float(fields, "x").unwrap_or(1.0),
            y: json_float(fields, "y").unwrap_or(1.0),
            z: json_float(fields, "z").unwrap_or(1.0),
        },
        "Mirror" => NodeType::Mirror {
            plane: match json_string(fields, "plane").as_deref() {
                Some("XZ") => MirrorPlane::XZ,
                Some("YZ") => MirrorPlane::YZ,
                _ => MirrorPlane::XY,
            },
        },
        "LinearArray" => NodeType::LinearArray {
            count: json_u32(fields, "count").unwrap_or(2),
            spacing: json_float(fields, "spacing").unwrap_or(1.0),
        },
        "CircularArray" => NodeType::CircularArray {
            count: json_u32(fields, "count").unwrap_or(4),
            angle: json_float(fields, "angle").unwrap_or(360.0),
        },
        "BooleanUnion" => NodeType::BooleanUnion,
        "BooleanSubtract" => NodeType::BooleanSubtract,
        "BooleanIntersect" => NodeType::BooleanIntersect,
        "Fillet" => NodeType::Fillet { radius: json_float(fields, "radius").unwrap_or(1.0) },
        "Chamfer" => NodeType::Chamfer { distance: json_float(fields, "distance").unwrap_or(1.0) },
        "Graft" => NodeType::Graft,
        "Flatten" => NodeType::Flatten,
        "CrossRef" => NodeType::CrossRef,
        "ShiftList" => NodeType::ShiftList { amount: json_i64(fields, "amount").unwrap_or(0) as i32 },
        "Subset" => NodeType::Subset {
            start: json_u32(fields, "start").unwrap_or(0),
            count: json_u32(fields, "count").unwrap_or(1),
        },
        "Dispatch" => NodeType::Dispatch,
        "Weave" => NodeType::Weave,
        "Concat" => NodeType::Concat,
        "BakeToDoc" => NodeType::BakeToDoc,
        "PlaneInput" => NodeType::PlaneInput {
            ox: json_float(fields, "ox").unwrap_or(0.0),
            oy: json_float(fields, "oy").unwrap_or(0.0),
            oz: json_float(fields, "oz").unwrap_or(0.0),
            nx: json_float(fields, "nx").unwrap_or(0.0),
            ny: json_float(fields, "ny").unwrap_or(0.0),
            nz: json_float(fields, "nz").unwrap_or(1.0),
        },
        "DomainInput" => NodeType::DomainInput {
            min: json_float(fields, "min").unwrap_or(0.0),
            max: json_float(fields, "max").unwrap_or(1.0),
        },
        "StringInput" => NodeType::StringInput { text: json_string(fields, "text").unwrap_or_default() },
        "Volume" => NodeType::Volume,
        "SurfaceArea" => NodeType::SurfaceArea,
        "Centroid" => NodeType::Centroid,
        "BoundingBox" => NodeType::BoundingBox,
        "Distance" => NodeType::Distance,
        "Angle" => NodeType::Angle,
        "MassProperties" => NodeType::MassProperties,
        "Cross" => NodeType::Cross,
        "Dot" => NodeType::Dot,
        "VectorLength" => NodeType::VectorLength,
        "Unit" => NodeType::Unit,
        "Negative" => NodeType::Negative,
        "Reciprocal" => NodeType::Reciprocal,
        "Asin" => NodeType::Asin,
        "Acos" => NodeType::Acos,
        "Atan" => NodeType::Atan,
        "Atan2" => NodeType::Atan2,
        "Log" => NodeType::Log,
        "Ln" => NodeType::Ln,
        "Exp" => NodeType::Exp,
        "Modulus" => NodeType::Modulus,
        "MapDomain" => NodeType::MapDomain {
            source_min: json_float(fields, "source_min").unwrap_or(0.0),
            source_max: json_float(fields, "source_max").unwrap_or(1.0),
            target_min: json_float(fields, "target_min").unwrap_or(0.0),
            target_max: json_float(fields, "target_max").unwrap_or(1.0),
        },
        "PointMidpoint" => NodeType::PointMidpoint,
        "PointLerp" => NodeType::PointLerp { t: json_float(fields, "t").unwrap_or(0.5) },
        "Vector2pt" => NodeType::Vector2pt,
        "Extrude" => NodeType::Extrude { distance: json_float(fields, "distance").unwrap_or(1.0) },
        "Revolve" => NodeType::Revolve { angle_deg: json_float(fields, "angle_deg").unwrap_or(360.0) },
        "Loft" => NodeType::Loft,
        "Sweep" => NodeType::Sweep,
        "RuledSurface" => NodeType::RuledSurface,
        "PlaneSurface" => NodeType::PlaneSurface,
        "ExtrudePoint" => NodeType::ExtrudePoint,
        "ExtrudeTapered" => NodeType::ExtrudeTapered {
            distance: json_float(fields, "distance").unwrap_or(1.0),
            taper_deg: json_float(fields, "taper_deg").unwrap_or(0.0),
        },
        "EvaluateSurface" => NodeType::EvaluateSurface,
        "SurfaceNormal" => NodeType::SurfaceNormal,
        "Shell" => NodeType::Shell { thickness: json_float(fields, "thickness").unwrap_or(1.0) },
        "Thicken" => NodeType::Thicken { thickness: json_float(fields, "thickness").unwrap_or(1.0) },
        "OffsetSolid" => NodeType::OffsetSolid { distance: json_float(fields, "distance").unwrap_or(1.0) },
        "SplitSolid" => NodeType::SplitSolid,
        "TrimSolid" => NodeType::TrimSolid,
        "Hole" => NodeType::Hole {
            radius: json_float(fields, "radius").unwrap_or(1.0),
            depth: json_float(fields, "depth").unwrap_or(1.0),
        },
        "ToMesh" => NodeType::ToMesh,
        "MeshArea" => NodeType::MeshArea,
        "MeshVolume" => NodeType::MeshVolume,
        "MeshFlip" => NodeType::MeshFlip,
        // Kernel API Wrappers
        "Heal Solid" => NodeType::HealSolid,
        "Stitch Solids" => NodeType::StitchSolids { tolerance: json_float(fields, "tolerance").unwrap_or(0.01) },
        "Validate Solid" => NodeType::ValidateSolid,
        "Export glTF" => NodeType::ExportGLTF { path: json_string(fields, "path").unwrap_or_default() },
        "Export USD" => NodeType::ExportUSD { path: json_string(fields, "path").unwrap_or_default() },
        "Export 3MF" => NodeType::Export3MF { path: json_string(fields, "path").unwrap_or_default() },
        "Export PLY" => NodeType::ExportPLY { path: json_string(fields, "path").unwrap_or_default() },
        "Export DXF" => NodeType::ExportDXF { path: json_string(fields, "path").unwrap_or_default() },
        "Import STL" => NodeType::ImportSTL { path: json_string(fields, "path").unwrap_or_default() },
        "Sketch Solve" => NodeType::SketchSolve,
        "Assembly Solve" => NodeType::AssemblySolve,
        "Create Drawing" => NodeType::CreateDrawing { view_type: json_u32(fields, "view_type").unwrap_or(0) },
        "Export PDF" => NodeType::ExportPDF { path: json_string(fields, "path").unwrap_or_default() },
        "Export SVG" => NodeType::ExportSVG { path: json_string(fields, "path").unwrap_or_default() },
        "FEA Solve" => NodeType::FEASolve { density: json_float(fields, "density").unwrap_or(7850.0) },
        "Generate Toolpath" => NodeType::GenerateToolpath { tool_diameter: json_float(fields, "tool_diameter").unwrap_or(6.0) },
        "Export G-Code" => NodeType::ExportGCode { path: json_string(fields, "path").unwrap_or_default() },
        "Unfold Sheet" => NodeType::UnfoldSheet { k_factor: json_float(fields, "k_factor").unwrap_or(0.33) },
        "To Implicit" => NodeType::ToImplicit,
        "SDF Union" => NodeType::SdfUnion,
        "SDF Subtract" => NodeType::SdfSubtract,
        "SDF Intersect" => NodeType::SdfIntersect,
        "Dual Contour Mesh" => NodeType::DualContourMesh { voxel_size: json_float(fields, "voxel_size").unwrap_or(0.5) },
        "SubD Subdivide" => NodeType::SubDSubdivide { levels: json_u32(fields, "levels").unwrap_or(1) },
        "SubD To NURBS" => NodeType::SubDToNurbs,
        "GPU Eval NURBS" => NodeType::GPUEvalNurbs,
        "Geometry Hash" => NodeType::GeometryHash,
        "Merkle Verify" => NodeType::MerkleVerify,
        // Fallback: try to match by label
        other => {
            // For unknown types, create a Panel as placeholder
            log::warn!("Unknown NodeType '{}' during deserialization, creating Panel", other);
            NodeType::Panel
        }
    })
}

/// Escape special JSON characters in a string.
fn escape_json(s: &str) -> String {
    s.replace('\\', "\\\\")
     .replace('"', "\\\"")
     .replace('\n', "\\n")
     .replace('\r', "\\r")
     .replace('\t', "\\t")
}

#[cfg(test)]
mod vp_serialize_tests {
    use super::*;

    #[test]
    fn test_graph_json_roundtrip() {
        let mut g = VpGraph::new();
        let a = g.add_node(NodeType::NumberSlider { value: 42.0, min: 0.0, max: 100.0 }, 0.0, 0.0);
        let b = g.add_node(NodeType::Box { width: 10.0, height: 5.0, depth: 3.0 }, 100.0, 0.0);
        let c = g.add_node(NodeType::BooleanUnion, 200.0, 0.0);
        g.connect(a, 0, c, 0);
        g.connect(b, 0, c, 1);

        let json = g.to_json();
        let g2 = VpGraph::from_json(&json).unwrap();

        assert_eq!(g2.node_count(), 3);
        assert_eq!(g2.connection_count(), 2);
        assert_eq!(g2.next_id, 3);

        // Check first node
        match &g2.nodes[0].node_type {
            NodeType::NumberSlider { value, min, max } => {
                assert!((value - 42.0).abs() < 1e-10);
                assert!((min - 0.0).abs() < 1e-10);
                assert!((max - 100.0).abs() < 1e-10);
            }
            other => panic!("Expected NumberSlider, got {:?}", other),
        }
    }

    #[test]
    fn test_export_selected() {
        let mut g = VpGraph::new();
        let a = g.add_node(NodeType::Box { width: 5.0, height: 5.0, depth: 5.0 }, 0.0, 0.0);
        let b = g.add_node(NodeType::Sphere { radius: 3.0 }, 100.0, 0.0);
        let _c = g.add_node(NodeType::BooleanUnion, 200.0, 0.0);

        let json = g.export_selected(&[a, b]);
        let g2 = VpGraph::from_json(&json).unwrap();
        assert_eq!(g2.node_count(), 2);
        // IDs should be remapped to 0 and 1
        assert_eq!(g2.nodes[0].id, 0);
        assert_eq!(g2.nodes[1].id, 1);
    }

    #[test]
    fn test_import_into_graph() {
        let mut g1 = VpGraph::new();
        let a = g1.add_node(NodeType::Box { width: 5.0, height: 5.0, depth: 5.0 }, 0.0, 0.0);
        let b = g1.add_node(NodeType::Sphere { radius: 3.0 }, 100.0, 0.0);
        let c = g1.add_node(NodeType::BooleanUnion, 200.0, 0.0);
        g1.connect(a, 0, c, 0);
        g1.connect(b, 0, c, 1);

        let json = g1.to_json();

        let mut g2 = VpGraph::new();
        g2.add_node(NodeType::Cylinder { radius: 2.0, height: 10.0 }, 0.0, 0.0); // existing node
        let new_ids = g2.import_from_json(&json).unwrap();

        assert_eq!(new_ids.len(), 3);
        assert_eq!(g2.node_count(), 4); // 1 existing + 3 imported
        assert_eq!(g2.connection_count(), 2); // 2 connections imported
    }

    #[test]
    fn test_empty_graph_serialize() {
        let g = VpGraph::new();
        let json = g.to_json();
        let g2 = VpGraph::from_json(&json).unwrap();
        assert_eq!(g2.node_count(), 0);
        assert_eq!(g2.connection_count(), 0);
    }

    #[test]
    fn test_all_field_types_roundtrip() {
        let mut g = VpGraph::new();
        g.add_node(NodeType::NumberSlider { value: 5.0, min: 0.0, max: 10.0 }, 0.0, 0.0);
        g.add_node(NodeType::BooleanToggle { value: true }, 0.0, 50.0);
        g.add_node(NodeType::PointInput { x: 1.0, y: 2.0, z: 3.0 }, 0.0, 100.0);
        g.add_node(NodeType::VectorInput { x: 0.0, y: 1.0, z: 0.0 }, 0.0, 150.0);
        g.add_node(NodeType::Fillet { radius: 2.5 }, 0.0, 200.0);
        g.add_node(NodeType::Series { start: 0.0, step: 0.5, count: 20 }, 0.0, 250.0);

        let json = g.to_json();
        let g2 = VpGraph::from_json(&json).unwrap();

        assert_eq!(g2.node_count(), 6);

        // Check Fillet
        match &g2.nodes[4].node_type {
            NodeType::Fillet { radius } => assert!((radius - 2.5).abs() < 1e-10),
            other => panic!("Expected Fillet, got {:?}", other),
        }

        // Check Series
        match &g2.nodes[5].node_type {
            NodeType::Series { start, step, count } => {
                assert!((start - 0.0).abs() < 1e-10);
                assert!((step - 0.5).abs() < 1e-10);
                assert_eq!(*count, 20);
            }
            other => panic!("Expected Series, got {:?}", other),
        }
    }

    #[test]
    fn test_save_load_file() {
        let mut g = VpGraph::new();
        g.add_node(NodeType::Box { width: 5.0, height: 5.0, depth: 5.0 }, 0.0, 0.0);

        let path = "/tmp/vp_test_graph.json";
        g.save_to_file(path).unwrap();
        let g2 = VpGraph::load_from_file(path).unwrap();
        assert_eq!(g2.node_count(), 1);

        // Cleanup
        std::fs::remove_file(path).ok();
    }

    #[test]
    fn test_remove_node() {
        let mut g = VpGraph::new();
        let a = g.add_node(NodeType::Box { width: 5.0, height: 5.0, depth: 5.0 }, 0.0, 0.0);
        let b = g.add_node(NodeType::Sphere { radius: 3.0 }, 100.0, 0.0);
        let c = g.add_node(NodeType::BooleanUnion, 200.0, 0.0);
        g.connect(a, 0, c, 0);
        g.connect(b, 0, c, 1);

        assert!(g.remove_node(b));
        assert_eq!(g.node_count(), 2);
        assert_eq!(g.connection_count(), 1); // Only a→c remains
    }

    #[test]
    fn test_clear_graph() {
        let mut g = VpGraph::new();
        g.add_node(NodeType::Box { width: 5.0, height: 5.0, depth: 5.0 }, 0.0, 0.0);
        g.add_node(NodeType::Sphere { radius: 3.0 }, 100.0, 0.0);
        g.clear();
        assert_eq!(g.node_count(), 0);
        assert_eq!(g.next_id, 0);
    }
}
