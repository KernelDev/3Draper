//! # draper-core
//!
//! The high-level API for 3Draper — combines STEP I/O, geometry,
//! topology, mesh generation, and programmatic modeling into a unified interface.

pub mod document;
pub mod engine;
pub mod error;
pub mod scene;
pub mod step_bridge;

pub use document::*;
pub use engine::*;
pub use error::*;
pub use scene::*;
pub use step_bridge::*;

// Re-export key types from sub-crates
pub use draper_geometry::{
    curve::Curve,
    direction::{Axis2Placement3D, Direction3},
    point::{BoundingBox3, Point2, Point3},
    surface::Surface,
    transform::Transform3,
};
pub use draper_mesh::triangulate::TriangleMesh;
pub use draper_step::ast::{StepDocument, StepEntity, StructureNode};
pub use draper_topology::{
    builder::ShapeBuilder,
    entity::*,
    shape::Shape,
};
