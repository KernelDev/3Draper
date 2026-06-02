// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! JSON command definitions for the 3Draper kernel API.
//!
//! These are standalone serializable command structures that can be
//! used to build API requests without the full `ApiRequest` enum.
//! Useful for WASM bindings and HTTP endpoints.

use serde::{Deserialize, Serialize};

/// Command to load a STEP file.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoadStepCommand {
    /// STEP file text content.
    pub content: String,
    /// Whether to apply healing.
    #[serde(default = "default_true")]
    pub heal: bool,
}

/// Command to export a model to JSON.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExportJsonCommand {
    /// Whether to include STEP source text.
    #[serde(default)]
    pub include_step_source: bool,
    /// Whether to pretty-print.
    #[serde(default = "default_true")]
    pub pretty: bool,
}

/// Command to import a model from JSON.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ImportJsonCommand {
    /// JSON string of the model.
    pub json: String,
}

/// Command to get mesh data.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GetMeshCommand {
    /// Optional instance index. If None, returns merged mesh.
    pub instance_index: Option<usize>,
}

/// Command to transform an instance.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransformInstanceCommand {
    /// Instance index (0-based).
    pub instance_index: usize,
    /// 4x4 transform matrix (row-major).
    pub transform: [[f64; 4]; 4],
}

/// Command to set instance color.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ColorInstanceCommand {
    /// Instance index (0-based).
    pub instance_index: usize,
    /// RGBA color (0..1 range).
    pub color: [f32; 4],
}

/// Command to get face information.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GetFacesCommand {
    /// Instance index (0-based).
    pub instance_index: usize,
}

fn default_true() -> bool {
    true
}
