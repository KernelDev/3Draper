// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-json
//! JSON import/export and API interface for the 3Draper kernel.
//!
//! Provides:
//! - `JsonModel` — a serializable model representation (mesh + assembly + metadata)
//! - Export: STEP/BRep → JSON
//! - Import: JSON → mesh/renderable data
//! - JSON API for programmatic kernel access
//!
//! ## Example (export)
//! ```ignore
//! use draper_json::JsonModel;
//! let model = JsonModel::from_step_content(&step_text);
//! let json = serde_json::to_string_pretty(&model).unwrap();
//! ```
//!
//! ## Example (import)
//! ```ignore
//! let model: JsonModel = serde_json::from_str(&json).unwrap();
//! let mesh = model.to_triangle_mesh();
//! ```

pub mod model;
pub mod api;
pub mod commands;

#[cfg(feature = "http-server")]
pub mod server;

pub use model::*;
pub use api::*;
pub use commands::*;
