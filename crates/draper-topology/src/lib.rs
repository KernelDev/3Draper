//! # draper-topology
//!
//! B-rep (Boundary Representation) topology kernel for 3Draper.
//!
//! Implements the standard topological hierarchy:
//! - Vertex → Edge → Wire → Face → Shell → Solid → Compound
//!
//! Each topological entity references its geometric counterpart
//! (e.g., Edge references a Curve, Face references a Surface).

pub mod builder;
pub mod entity;
pub mod shape;
pub mod traversal;

pub use builder::*;
pub use entity::*;
pub use shape::*;
pub use traversal::*;
