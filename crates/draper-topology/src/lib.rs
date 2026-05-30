// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-topology
//! B-Rep (Boundary Representation) topology for the 3Draper kernel.
//!
//! Topology hierarchy: Solid → Shell → Face → Wire → CoEdge → Edge → Vertex
//! Each topological entity has a reference to its underlying geometry.

#![warn(clippy::unwrap_used)]

pub mod entity;
pub mod shape;
pub mod builder;
pub mod traversal;
pub mod validation;
pub mod healing;
pub mod boolean;
pub mod queries;
pub mod operations;

pub use entity::*;
pub use shape::*;
pub use builder::*;
pub use traversal::*;
pub use validation::{
    ValidationError, validate_solid, validate_solid_readonly, validate_shell,
    Severity, ValidationIssue, TopologyValidationConfig, TopologyValidationReport,
    validate_topology,
};
pub use healing::*;
pub use boolean::*;
pub use queries::{
    solid_volume, solid_surface_area, solid_center_of_mass, point_in_solid,
    solid_moments_of_inertia, InertiaTensor,
    Bvh, BvhNode,
};
pub use operations::*;
