// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-core
//! High-level API for the 3Draper kernel.
//!
//! Provides document management, modeling operations, and pipeline orchestration.

pub mod document;
pub mod operations;
pub mod boolean;
pub mod assembly;
pub mod engine;
pub mod error;

pub use document::*;
pub use operations::*;
pub use boolean::*;
pub use assembly::*;
pub use engine::*;
pub use error::*;
