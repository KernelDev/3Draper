//! # draper-topology
//! B-Rep (Boundary Representation) topology for the 3Draper kernel.
//!
//! Topology hierarchy: Solid → Shell → Face → Wire → CoEdge → Edge → Vertex
//! Each topological entity has a reference to its underlying geometry.

pub mod entity;
pub mod shape;
pub mod builder;
pub mod traversal;
pub mod validation;
pub mod healing;

pub use entity::*;
pub use shape::*;
pub use builder::*;
pub use traversal::*;
pub use validation::{ValidationError, validate_solid, validate_solid_readonly, validate_shell};
pub use healing::*;
