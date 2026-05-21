//! # draper-mesh
//!
//! Mesh generation and triangulation for 3Draper.
//!
//! Converts B-rep topology into triangle meshes for rendering.
//! Uses the `spade` crate for Delaunay triangulation.

pub mod earcut;
pub mod generate;
pub mod triangulate;

pub use generate::*;
pub use triangulate::*;
