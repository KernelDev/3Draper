//! # draper-topology
//!
//! B-rep (Boundary Representation) topology kernel for 3Draper.
//!
//! Implements the standard topological hierarchy:
//! - Vertex → Edge → Wire → Face → Shell → Solid → Compound
//!
//! Each topological entity references its geometric counterpart
//! (e.g., Edge references a Curve, Face references a Surface).
//!
//! Additional modules:
//! - `healing`: Validation and repair of B-rep topology
//! - `discretize`: Consistent edge discretization for triangulation

pub mod builder;
pub mod discretize;
pub mod entity;
pub mod healing;
pub mod shape;
pub mod traversal;

pub use builder::*;
pub use discretize::*;
pub use entity::*;
pub use healing::*;
pub use shape::*;
pub use traversal::*;
