//! # draper-step
//!
//! Custom STEP (ISO 10303-21 / ISO 10303-203, -214, -242) file parser.
//!
//! Parses the exchange structure format into an entity tree that can be
//! consumed by higher-level crates (topology, geometry).

pub mod parser;
pub mod ast;
pub mod error;

pub use ast::*;
pub use error::*;
pub use parser::parse_step;
