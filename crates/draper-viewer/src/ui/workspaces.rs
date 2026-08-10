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
    /// A 2D/3D polyline curve
    Curve(Vec<draper_geometry::Point3d>),
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
    Number,
    Integer,
    Boolean,
    Point,
    Vector,
    String,
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
        // Integer accepts Number (truncation)
        if *self == PortType::Integer && *other == PortType::Number { return true; }
        // Point accepts Vector and vice versa
        if *self == PortType::Point && *other == PortType::Vector { return true; }
        if *self == PortType::Vector && *other == PortType::Point { return true; }
        // List accepts anything (wrap in single-element list)
        if *self == PortType::List { return true; }
        // Any type accepts List (take first element)
        if *other == PortType::List { return true; }
        false
    }

    /// Display color for this port type (for visual identification).
    pub fn color(&self) -> egui::Color32 {
        match self {
            PortType::Geometry => egui::Color32::from_rgb(0x89, 0xb4, 0xfa), // blue
            PortType::Curve => egui::Color32::from_rgb(0xa6, 0xe3, 0xa1),    // green
            PortType::Number => egui::Color32::from_rgb(0xf9, 0xe2, 0xaf),   // yellow
            PortType::Integer => egui::Color32::from_rgb(0xeb, 0xa0, 0xac),   // red
            PortType::Boolean => egui::Color32::from_rgb(0xf5, 0xc2, 0xe7),  // pink
            PortType::Point => egui::Color32::from_rgb(0xfab3, 0x87, 0x95),  // peach
            PortType::Vector => egui::Color32::from_rgb(0x94, 0xe2, 0xd5),   // teal
            PortType::String => egui::Color32::from_rgb(0xba, 0xc2, 0xde),   // lavender
            PortType::List => egui::Color32::from_rgb(0xcb, 0xa6, 0xf7),     // purple
            PortType::Any => egui::Color32::from_rgb(0x6c, 0x70, 0x86),      // gray
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
    Abs,
    Sqrt,
    Min,
    Max,
    Average,
    Expression { expr: String },

    // ─── Sets (List Operations) ───
    Series { start: f64, step: f64, count: u32 },
    ListLength,
    ListItem,
    Reverse,

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

    // ─── Transform ───
    Move,
    Rotate { angle_deg: f64 },
    Scale { factor: f64 },
    Mirror,
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

    // ─── Output ───
    /// Bake to document.
    BakeToDoc,
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
            NodeType::Abs => "Abs",
            NodeType::Sqrt => "Sqrt",
            NodeType::Min => "Min",
            NodeType::Max => "Max",
            NodeType::Average => "Average",
            NodeType::Expression { .. } => "Expression",
            // Sets
            NodeType::Series { .. } => "Series",
            NodeType::ListLength => "List Length",
            NodeType::ListItem => "List Item",
            NodeType::Reverse => "Reverse",
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
            // Transform
            NodeType::Move => "Move",
            NodeType::Rotate { .. } => "Rotate",
            NodeType::Scale { .. } => "Scale",
            NodeType::Mirror => "Mirror",
            NodeType::LinearArray { .. } => "Linear Array",
            NodeType::CircularArray { .. } => "Circular Array",
            // Boolean
            NodeType::BooleanUnion => "Union",
            NodeType::BooleanSubtract => "Subtract",
            NodeType::BooleanIntersect => "Intersect",
            // Modify
            NodeType::Fillet { .. } => "Fillet",
            NodeType::Chamfer { .. } => "Chamfer",
            // Output
            NodeType::BakeToDoc => "Bake to Doc",
        }
    }

    pub fn category(&self) -> &'static str {
        match self {
            NodeType::NumberSlider { .. } | NodeType::IntegerInput { .. } |
            NodeType::BooleanToggle { .. } | NodeType::PointInput { .. } |
            NodeType::VectorInput { .. } | NodeType::Panel => "Params",
            NodeType::Add | NodeType::Subtract | NodeType::Multiply | NodeType::Divide |
            NodeType::Sin | NodeType::Cos | NodeType::Abs | NodeType::Sqrt |
            NodeType::Min | NodeType::Max | NodeType::Average | NodeType::Expression { .. } => "Maths",
            NodeType::Series { .. } | NodeType::ListLength | NodeType::ListItem | NodeType::Reverse => "Sets",
            NodeType::Box { .. } | NodeType::Sphere { .. } | NodeType::Cylinder { .. } |
            NodeType::Cone { .. } | NodeType::Torus { .. } => "Primitives",
            NodeType::Line | NodeType::Circle { .. } | NodeType::DivideCurve { .. } => "Curve",
            NodeType::Move | NodeType::Rotate { .. } | NodeType::Scale { .. } |
            NodeType::Mirror | NodeType::LinearArray { .. } | NodeType::CircularArray { .. } => "Transform",
            NodeType::BooleanUnion | NodeType::BooleanSubtract | NodeType::BooleanIntersect => "Boolean",
            NodeType::Fillet { .. } | NodeType::Chamfer { .. } => "Modify",
            NodeType::BakeToDoc => "Output",
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
            NodeType::Sin | NodeType::Cos | NodeType::Abs | NodeType::Sqrt => "f(x)",
            NodeType::Min | NodeType::Max | NodeType::Average => "min",
            NodeType::Expression { .. } => "expr",
            NodeType::Series { .. } => "S",
            NodeType::ListLength => "len",
            NodeType::ListItem => "[]i",
            NodeType::Reverse => "rev",
            NodeType::Box { .. } => "B",
            NodeType::Sphere { .. } => "S",
            NodeType::Cylinder { .. } => "C",
            NodeType::Cone { .. } => "Co",
            NodeType::Torus { .. } => "T",
            NodeType::Line => "L",
            NodeType::Circle { .. } => "O",
            NodeType::DivideCurve { .. } => "Div",
            NodeType::Move => "Mv",
            NodeType::Rotate { .. } => "Rot",
            NodeType::Scale { .. } => "Sc",
            NodeType::Mirror => "Mir",
            NodeType::LinearArray { .. } => "Arr",
            NodeType::CircularArray { .. } => "Cir",
            NodeType::BooleanUnion => "U",
            NodeType::BooleanSubtract => "Sub",
            NodeType::BooleanIntersect => "Int",
            NodeType::Fillet { .. } => "Fil",
            NodeType::Chamfer { .. } => "Chm",
            NodeType::BakeToDoc => "Bake",
        }
    }

    /// Input port descriptors (name + type) for this node.
    pub fn input_ports(&self) -> Vec<PortDesc> {
        match self {
            // Params — no inputs
            NodeType::NumberSlider { .. } | NodeType::IntegerInput { .. } |
            NodeType::BooleanToggle { .. } | NodeType::PointInput { .. } |
            NodeType::VectorInput { .. } | NodeType::Series { .. } |
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
            NodeType::Sin | NodeType::Cos | NodeType::Abs | NodeType::Sqrt => vec![
                PortDesc { name: "X", port_type: PortType::Number },
            ],
            NodeType::Average => vec![PortDesc { name: "L", port_type: PortType::List }],
            NodeType::Expression { .. } => vec![PortDesc { name: "X", port_type: PortType::Number }],
            // Sets
            NodeType::ListLength => vec![PortDesc { name: "L", port_type: PortType::List }],
            NodeType::ListItem => vec![
                PortDesc { name: "L", port_type: PortType::List },
                PortDesc { name: "I", port_type: PortType::Integer },
            ],
            NodeType::Reverse => vec![PortDesc { name: "L", port_type: PortType::List }],
            // Curve
            NodeType::Line => vec![
                PortDesc { name: "A", port_type: PortType::Point },
                PortDesc { name: "B", port_type: PortType::Point },
            ],
            NodeType::DivideCurve { .. } => vec![
                PortDesc { name: "C", port_type: PortType::Curve },
                PortDesc { name: "N", port_type: PortType::Integer },
            ],
            // Transform — 1 Geometry + optional Vector/Number
            NodeType::Move => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "V", port_type: PortType::Vector },
            ],
            NodeType::Rotate { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "A", port_type: PortType::Number },
            ],
            NodeType::Scale { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
                PortDesc { name: "F", port_type: PortType::Number },
            ],
            NodeType::Mirror => vec![
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
            // Output
            NodeType::BakeToDoc => vec![PortDesc { name: "G", port_type: PortType::Geometry }],
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
            NodeType::Panel | NodeType::BakeToDoc => vec![],
            NodeType::Series { .. } => vec![PortDesc { name: "L", port_type: PortType::List }],
            NodeType::ListLength => vec![PortDesc { name: "N", port_type: PortType::Integer }],
            NodeType::ListItem => vec![PortDesc { name: "I", port_type: PortType::Any }],
            NodeType::Reverse => vec![PortDesc { name: "L", port_type: PortType::List }],
            // Math outputs are Number
            NodeType::Add | NodeType::Subtract | NodeType::Multiply | NodeType::Divide |
            NodeType::Sin | NodeType::Cos | NodeType::Abs | NodeType::Sqrt |
            NodeType::Min | NodeType::Max | NodeType::Average | NodeType::Expression { .. } => vec![
                PortDesc { name: "R", port_type: PortType::Number },
            ],
            // Geometry outputs
            NodeType::Box { .. } | NodeType::Sphere { .. } | NodeType::Cylinder { .. } |
            NodeType::Cone { .. } | NodeType::Torus { .. } |
            NodeType::Move | NodeType::Rotate { .. } | NodeType::Scale { .. } |
            NodeType::Mirror | NodeType::LinearArray { .. } | NodeType::CircularArray { .. } |
            NodeType::BooleanUnion | NodeType::BooleanSubtract | NodeType::BooleanIntersect |
            NodeType::Fillet { .. } | NodeType::Chamfer { .. } => vec![
                PortDesc { name: "G", port_type: PortType::Geometry },
            ],
            // Curve outputs
            NodeType::Line | NodeType::Circle { .. } => vec![PortDesc { name: "C", port_type: PortType::Curve }],
            NodeType::DivideCurve { .. } => vec![PortDesc { name: "P", port_type: PortType::List }],
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
