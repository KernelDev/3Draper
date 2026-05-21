//! Mesh generation from B-rep topology.
//!
//! This module provides the main entry point for mesh generation.
//! The actual implementation is in the `pipeline` module, which follows
//! the production triangulation pipeline from the guide.

// Re-export the pipeline function as the main generate_mesh
pub use crate::pipeline::generate_mesh;
