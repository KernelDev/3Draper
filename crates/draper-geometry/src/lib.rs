//! # draper-geometry
//! Core geometric primitives for the 3Draper kernel.
//!
//! Provides points, vectors, transformations, parametric curves and surfaces.

pub mod point;
pub mod direction;
pub mod vector;
pub mod transform;
pub mod curve;
pub mod surface;
pub mod intersection;
pub mod tolerance;

pub use point::*;
pub use direction::*;
pub use vector::*;
pub use transform::*;
pub use curve::*;
pub use surface::*;
pub use intersection::*;
pub use tolerance::*;
