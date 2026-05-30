// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-geometry
//! Core geometric primitives for the 3Draper kernel.
//!
//! Provides points, vectors, transformations, parametric curves and surfaces.

pub mod point;
pub mod direction;
pub mod vector;
pub mod transform;
pub mod curve;
pub mod curve2d;
pub mod surface;
pub mod intersection;
pub mod tolerance;

pub use point::*;
pub use direction::*;
pub use vector::*;
pub use transform::*;
pub use curve::*;
pub use curve2d::*;
pub use surface::*;
pub use intersection::*;
pub use tolerance::*;
