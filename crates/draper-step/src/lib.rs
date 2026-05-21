//! # draper-step
//!
//! STEP (ISO 10303-21 / ISO 10303-203, -214, -242) file parser for 3Draper.
//!
//! Powered by the `step-io` crate for robust parsing of real-world STEP files.
//! This crate wraps step-io and provides:
//! - High-level parsing API (`parse_step`)
//! - Backward-compatible AST types (`StepDocument`, `StepEntity`, `StructureNode`)
//! - Direct access to step-io's typed IR (`StepModel`, `GeometryPool`, `TopologyPool`)
//! - STEP file writing support

pub mod ast;
pub mod bridge;
pub mod error;

// Re-export step-io types for direct access by downstream crates.
// Only export types that step_io exposes at its root level.
pub use step_io::{
    // IR model types
    Arena, GeometryPool, StepModel, TopologyPool, UnitContext,
    LengthUnit, AngleUnit, SolidAngleUnit,
    // IR geometry types
    Axis1Placement, Axis2Placement2d, Axis2Placement3d,
    Circle2, Circle3, ConicalSurface, Curve, Curve2d, CurveForm, CurveId,
    CylindricalSurface, Direction2, Direction2dId, Direction3, DirectionId,
    Edge, EdgeId, Ellipse2, Ellipse3,
    Face, FaceId,
    Line2, Line3,
    NurbsCurve, NurbsCurve2d, NurbsSurface,
    Orientation, OrientedEdge, Pcurve,
    Placement1dId, Placement2dId, Placement3dId,
    Plane3, Point2, Point2dId, Point3, PointId,
    Shell, ShellId, Solid, SolidId, SphericalSurface,
    Surface, SurfaceForm, SurfaceId,
    SurfaceOfLinearExtrusion, SurfaceOfRevolution,
    ToroidalSurface, Transform3d,
    Vertex, VertexId, Wire, WireId,
    // IR assembly types
    AssemblyTree, Instance, Product, ProductContent, ProductId,
    // Parser types
    Attribute, EntityGraph, ParseError, RawEntity, RawEntityPart,
    SchemaClass, StepSchema,
    parse as parse_step_raw,
    // Writer types
    WriteError,
    // Error types
    ConvertError,
};

pub use ast::*;
pub use bridge::*;
pub use error::*;
