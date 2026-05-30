// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-mesh
//! Mesh generation from B-Rep topology.
//!
//! Provides constrained Delaunay triangulation of B-Rep faces
//! and mesh output in various formats.

#![warn(clippy::unwrap_used)]

pub mod mesh;
pub mod triangulate;
pub mod stl;
pub mod manifold;
pub mod edge_cache;
pub mod adaptive;
pub mod parametric_domain;

pub use mesh::*;
pub use triangulate::*;
pub use stl::*;
pub use manifold::*;
pub use edge_cache::*;
