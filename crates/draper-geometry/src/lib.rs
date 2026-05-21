//! # draper-geometry
//!
//! Custom geometry kernel for 3Draper.
//!
//! Provides fundamental geometric primitives:
//! - Points, vectors, directions
//! - Curves: Line, Circle, Ellipse, BSpline
//! - Surfaces: Plane, Cylinder, Cone, Sphere, Torus, BSpline Surface
//! - Coordinate systems and transformations
//! - Intersection and projection utilities

pub mod curve;
pub mod direction;
pub mod intersection;
pub mod point;
pub mod surface;
pub mod transform;

pub use curve::*;
pub use direction::*;
pub use intersection::*;
pub use point::*;
pub use surface::*;
pub use transform::*;
