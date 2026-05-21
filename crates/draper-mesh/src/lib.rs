//! # draper-mesh
//!
//! Mesh generation and triangulation for 3Draper.
//!
//! Converts B-rep topology into triangle meshes for rendering.
//! Uses the `spade` crate for Constrained Delaunay Triangulation (CDT)
//! and custom ear-clipping as a fallback.
//!
//! Production pipeline:
//! 1. Topological validation and healing
//! 2. Consistent edge discretization (curvature-based)
//! 3. Surface metric analysis and adaptive interior points
//! 4. CDT in UV space with boundary constraints
//! 5. 3D mapping and quality control
//! 6. Iterative refinement

pub mod boolean;
pub mod delaunay;
pub mod earcut;
pub mod generate;
pub mod pipeline;
pub mod quality;
pub mod triangulate;

pub use generate::*;
pub use triangulate::*;
