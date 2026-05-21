//! # draper-mesh
//!
//! Mesh generation and triangulation for 3Draper.
//!
//! Converts B-rep topology into triangle meshes for rendering.
//! Uses the `spade` crate for Constrained Delaunay Triangulation (CDT)
//! and custom ear-clipping as a fallback.

pub mod delaunay;
pub mod earcut;
pub mod generate;
pub mod triangulate;

pub use generate::*;
pub use triangulate::*;
