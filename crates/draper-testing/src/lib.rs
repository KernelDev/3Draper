// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-testing
//! Test infrastructure for the 3Draper geometric kernel.
//!
//! Provides reference primitives, industrial file testing, fuzzing,
//! manifold/watertight checks, Euler characteristic, surface area/volume
//! comparison, NaN/Inf detection, zero-area triangle filtering, and
//! flipped normal detection.

pub mod primitives;
pub mod combinations;
pub mod industrial;
pub mod problematic;
pub mod fuzzing;
pub mod nist;
pub mod cad_compat;
pub mod watertight;
pub mod euler;
pub mod area;
pub mod volume;
pub mod validity;
pub mod normals;

pub use primitives::*;
pub use combinations::*;
pub use industrial::*;
pub use problematic::*;
pub use fuzzing::*;
pub use nist::*;
pub use cad_compat::*;
pub use watertight::*;
pub use euler::*;
pub use area::*;
pub use volume::*;
pub use validity::*;
pub use normals::*;
