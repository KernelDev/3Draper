// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! JSON API for programmatic kernel access.
//!
//! The `JsonApi` struct provides a simple request/response interface for
//! working with the 3Draper kernel via JSON. This can be used from:
//! - JavaScript/WASM via the exposed bindings
//! - HTTP server endpoints
//! - Command-line tools
//!
//! ## API Commands
//! - `load_step` — Load a STEP file from text
//! - `export_json` — Export current model to JSON
//! - `import_json` — Import model from JSON
//! - `get_mesh` — Get mesh data (vertices, triangles)
//! - `get_assembly` — Get assembly tree
//! - `get_instances` — Get all mesh instances
//! - `get_bbox` — Get bounding box
//! - `get_stats` — Get model statistics
//! - `transform_instance` — Apply transform to an instance
//! - `color_instance` — Set instance color
//! - `triangulate` — Re-triangulate the model with parameters

use serde::{Deserialize, Serialize};
use crate::model::JsonModel;
use draper_step::StepConversionConfig;

// ============================================================
// API Request/Response types
// ============================================================

/// A JSON API request.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum ApiRequest {
    /// Load a STEP file from text content.
    LoadStep {
        /// The STEP file text content.
        content: String,
        /// Whether to apply healing (default: true).
        #[serde(default = "default_true")]
        heal: bool,
    },

    /// Export the current model to JSON.
    ExportJson {
        /// Whether to include STEP source text.
        #[serde(default)]
        include_step_source: bool,
        /// Whether to pretty-print the JSON.
        #[serde(default = "default_true")]
        pretty: bool,
    },

    /// Import a model from JSON string.
    ImportJson {
        /// The JSON string.
        json: String,
    },

    /// Get mesh data for the entire model or a specific instance.
    GetMesh {
        /// Instance index (0-based). If None, returns merged mesh.
        #[serde(skip_serializing_if = "Option::is_none")]
        instance_index: Option<usize>,
    },

    /// Get the assembly tree structure.
    GetAssembly,

    /// Get all mesh instances with metadata.
    GetInstances,

    /// Get the bounding box of the model.
    GetBbox,

    /// Get model statistics.
    GetStats,

    /// Apply a transform to a specific instance.
    TransformInstance {
        /// Instance index (0-based).
        instance_index: usize,
        /// 4x4 transform matrix (row-major).
        transform: [[f64; 4]; 4],
    },

    /// Set the color of a specific instance.
    ColorInstance {
        /// Instance index (0-based).
        instance_index: usize,
        /// RGBA color (0..1 range).
        color: [f32; 4],
    },

    /// Get per-face information for a specific instance.
    GetFaces {
        /// Instance index (0-based).
        instance_index: usize,
    },

    /// Get a specific instance's detailed info.
    GetInstance {
        /// Instance index (0-based).
        instance_index: usize,
    },

    /// List all commands.
    Help,
}

/// A JSON API response.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApiResponse {
    /// Whether the command succeeded.
    pub success: bool,
    /// Human-readable status message.
    pub message: String,
    /// The response data (command-specific).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<serde_json::Value>,
}

impl ApiResponse {
    /// Create a success response.
    pub fn ok(message: &str, data: serde_json::Value) -> Self {
        ApiResponse {
            success: true,
            message: message.to_string(),
            data: Some(data),
        }
    }

    /// Create a success response with no data.
    pub fn ok_msg(message: &str) -> Self {
        ApiResponse {
            success: true,
            message: message.to_string(),
            data: None,
        }
    }

    /// Create an error response.
    pub fn err(message: &str) -> Self {
        ApiResponse {
            success: false,
            message: message.to_string(),
            data: None,
        }
    }
}

// ============================================================
// API Engine
// ============================================================

/// The JSON API engine that holds model state and processes commands.
pub struct JsonApi {
    /// The current loaded model (if any).
    model: Option<JsonModel>,
}

impl JsonApi {
    /// Create a new empty API engine.
    pub fn new() -> Self {
        JsonApi { model: None }
    }

    /// Create an API engine with a pre-loaded model.
    pub fn with_model(model: JsonModel) -> Self {
        JsonApi { model: Some(model) }
    }

    /// Get a reference to the current model.
    pub fn model(&self) -> Option<&JsonModel> {
        self.model.as_ref()
    }

    /// Get a mutable reference to the current model.
    pub fn model_mut(&mut self) -> Option<&mut JsonModel> {
        self.model.as_mut()
    }

    /// Process an API request and return a response.
    pub fn execute(&mut self, request: ApiRequest) -> ApiResponse {
        match request {
            ApiRequest::LoadStep { content, heal } => {
                self.cmd_load_step(&content, heal)
            }
            ApiRequest::ExportJson { include_step_source, pretty } => {
                self.cmd_export_json(include_step_source, pretty)
            }
            ApiRequest::ImportJson { json } => {
                self.cmd_import_json(&json)
            }
            ApiRequest::GetMesh { instance_index } => {
                self.cmd_get_mesh(instance_index)
            }
            ApiRequest::GetAssembly => {
                self.cmd_get_assembly()
            }
            ApiRequest::GetInstances => {
                self.cmd_get_instances()
            }
            ApiRequest::GetBbox => {
                self.cmd_get_bbox()
            }
            ApiRequest::GetStats => {
                self.cmd_get_stats()
            }
            ApiRequest::TransformInstance { instance_index, transform } => {
                self.cmd_transform_instance(instance_index, transform)
            }
            ApiRequest::ColorInstance { instance_index, color } => {
                self.cmd_color_instance(instance_index, color)
            }
            ApiRequest::GetFaces { instance_index } => {
                self.cmd_get_faces(instance_index)
            }
            ApiRequest::GetInstance { instance_index } => {
                self.cmd_get_instance(instance_index)
            }
            ApiRequest::Help => {
                self.cmd_help()
            }
        }
    }

    /// Process a raw JSON request string.
    pub fn execute_json(&mut self, json: &str) -> String {
        let request: Result<ApiRequest, _> = serde_json::from_str(json);
        match request {
            Ok(req) => {
                let response = self.execute(req);
                serde_json::to_string(&response).unwrap_or_else(|e| {
                    serde_json::to_string(&ApiResponse::err(&format!("Serialization error: {}", e)))
                        .unwrap_or_else(|_| r#"{"success":false,"message":"Internal error"}"#.to_string())
                })
            }
            Err(e) => {
                let response = ApiResponse::err(&format!("Invalid request: {}", e));
                serde_json::to_string(&response).unwrap_or_else(|_| {
                    r#"{"success":false,"message":"Invalid JSON"}"#.to_string()
                })
            }
        }
    }

    // ============================================================
    // Command implementations
    // ============================================================

    fn cmd_load_step(&mut self, content: &str, heal: bool) -> ApiResponse {
        let config = StepConversionConfig { heal };
        match JsonModel::from_step_content_with_config(content, &config) {
            Ok(model) => {
                let stats = format!(
                    "Loaded {} instances, {} vertices, {} triangles",
                    model.metadata.instance_count,
                    model.metadata.total_vertices,
                    model.metadata.total_triangles,
                );
                let data = serde_json::to_value(&model.metadata)
                    .unwrap_or(serde_json::Value::Null);
                self.model = Some(model);
                ApiResponse::ok(&stats, data)
            }
            Err(e) => ApiResponse::err(&format!("Failed to load STEP: {}", e)),
        }
    }

    fn cmd_export_json(&self, include_step_source: bool, pretty: bool) -> ApiResponse {
        match &self.model {
            Some(model) => {
                // If not including step source, create a copy without it
                let export_model = if include_step_source {
                    model.clone()
                } else {
                    let mut m = model.clone();
                    m.step_source = None;
                    m
                };

                let json = if pretty {
                    export_model.to_json_pretty()
                } else {
                    export_model.to_json()
                };

                match json {
                    Ok(j) => ApiResponse::ok("Model exported to JSON", serde_json::Value::String(j)),
                    Err(e) => ApiResponse::err(&format!("JSON export failed: {}", e)),
                }
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_import_json(&mut self, json: &str) -> ApiResponse {
        match JsonModel::from_json(json) {
            Ok(model) => {
                let stats = format!(
                    "Imported {} instances, {} vertices, {} triangles",
                    model.metadata.instance_count,
                    model.metadata.total_vertices,
                    model.metadata.total_triangles,
                );
                let data = serde_json::to_value(&model.metadata)
                    .unwrap_or(serde_json::Value::Null);
                self.model = Some(model);
                ApiResponse::ok(&stats, data)
            }
            Err(e) => ApiResponse::err(&format!("JSON import failed: {}", e)),
        }
    }

    fn cmd_get_mesh(&self, instance_index: Option<usize>) -> ApiResponse {
        match &self.model {
            Some(model) => {
                match instance_index {
                    None => {
                        // Return merged mesh
                        let mesh = model.to_triangle_mesh();
                        let data = mesh_to_json_value(&mesh);
                        ApiResponse::ok("Merged mesh data", data)
                    }
                    Some(idx) => {
                        if idx < model.instances.len() {
                            let inst = &model.instances[idx];
                            let mesh = inst.to_triangle_mesh();
                            let data = mesh_to_json_value(&mesh);
                            ApiResponse::ok(&format!("Mesh for instance {} ({})", idx, inst.name), data)
                        } else {
                            ApiResponse::err(&format!("Instance index {} out of range (0..{})", idx, model.instances.len()))
                        }
                    }
                }
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_get_assembly(&self) -> ApiResponse {
        match &self.model {
            Some(model) => {
                let data = serde_json::to_value(&model.assembly)
                    .unwrap_or(serde_json::Value::Null);
                ApiResponse::ok("Assembly tree", data)
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_get_instances(&self) -> ApiResponse {
        match &self.model {
            Some(model) => {
                let summary: Vec<serde_json::Value> = model.instances.iter().enumerate()
                    .map(|(i, inst)| serde_json::json!({
                        "index": i,
                        "name": inst.name,
                        "brep_id": inst.brep_id,
                        "vertex_count": inst.vertices.len() / 3,
                        "triangle_count": inst.triangles.len() / 3,
                        "has_color": inst.color.is_some(),
                        "has_transform": inst.transform.is_some(),
                    }))
                    .collect();
                ApiResponse::ok(&format!("{} instances", model.instances.len()), serde_json::Value::Array(summary))
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_get_bbox(&self) -> ApiResponse {
        match &self.model {
            Some(model) => {
                let data = serde_json::json!({
                    "min": model.metadata.bbox_min,
                    "max": model.metadata.bbox_max,
                });
                ApiResponse::ok("Bounding box", data)
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_get_stats(&self) -> ApiResponse {
        match &self.model {
            Some(model) => {
                let data = serde_json::to_value(&model.metadata)
                    .unwrap_or(serde_json::Value::Null);
                ApiResponse::ok("Model statistics", data)
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_transform_instance(&mut self, instance_index: usize, transform: [[f64; 4]; 4]) -> ApiResponse {
        match &mut self.model {
            Some(model) => {
                if instance_index < model.instances.len() {
                    let inst = &mut model.instances[instance_index];
                    // Transform all vertices
                    let mut mesh = inst.to_triangle_mesh();
                    mesh.transform(&transform);
                    // Write back
                    inst.vertices = mesh.vertices.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
                    inst.triangles = mesh.triangles.iter().flat_map(|t| [t[0], t[1], t[2]]).collect();
                    inst.normals = mesh.normals.as_ref().map(|n| n.iter().flat_map(|v| [v[0], v[1], v[2]]).collect());
                    // Update stored transform (compose)
                    inst.transform = Some(match inst.transform {
                        Some(existing) => compose_transforms(&existing, &transform),
                        None => transform,
                    });
                    ApiResponse::ok_msg(&format!("Transformed instance {}", instance_index))
                } else {
                    ApiResponse::err(&format!("Instance index {} out of range", instance_index))
                }
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_color_instance(&mut self, instance_index: usize, color: [f32; 4]) -> ApiResponse {
        match &mut self.model {
            Some(model) => {
                if instance_index < model.instances.len() {
                    model.instances[instance_index].color = Some(color);
                    ApiResponse::ok_msg(&format!("Set color for instance {}", instance_index))
                } else {
                    ApiResponse::err(&format!("Instance index {} out of range", instance_index))
                }
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_get_faces(&self, instance_index: usize) -> ApiResponse {
        match &self.model {
            Some(model) => {
                if instance_index < model.instances.len() {
                    let inst = &model.instances[instance_index];
                    let data = serde_json::to_value(&inst.faces)
                        .unwrap_or(serde_json::Value::Null);
                    ApiResponse::ok(&format!("{} faces for instance {}", inst.faces.len(), instance_index), data)
                } else {
                    ApiResponse::err(&format!("Instance index {} out of range", instance_index))
                }
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_get_instance(&self, instance_index: usize) -> ApiResponse {
        match &self.model {
            Some(model) => {
                if instance_index < model.instances.len() {
                    let data = serde_json::to_value(&model.instances[instance_index])
                        .unwrap_or(serde_json::Value::Null);
                    ApiResponse::ok(&format!("Instance {}", instance_index), data)
                } else {
                    ApiResponse::err(&format!("Instance index {} out of range", instance_index))
                }
            }
            None => ApiResponse::err("No model loaded"),
        }
    }

    fn cmd_help(&self) -> ApiResponse {
        let commands = serde_json::json!([
            {"command": "load_step", "description": "Load a STEP file from text content", "params": ["content: String", "heal: bool (default true)"]},
            {"command": "export_json", "description": "Export current model to JSON", "params": ["include_step_source: bool", "pretty: bool"]},
            {"command": "import_json", "description": "Import model from JSON string", "params": ["json: String"]},
            {"command": "get_mesh", "description": "Get mesh data (merged or per-instance)", "params": ["instance_index: Option<usize>"]},
            {"command": "get_assembly", "description": "Get assembly tree structure", "params": []},
            {"command": "get_instances", "description": "Get all mesh instances summary", "params": []},
            {"command": "get_bbox", "description": "Get model bounding box", "params": []},
            {"command": "get_stats", "description": "Get model statistics", "params": []},
            {"command": "transform_instance", "description": "Apply transform to an instance", "params": ["instance_index: usize", "transform: [[f64;4];4]"]},
            {"command": "color_instance", "description": "Set instance color", "params": ["instance_index: usize", "color: [f32;4]"]},
            {"command": "get_faces", "description": "Get per-face info for an instance", "params": ["instance_index: usize"]},
            {"command": "get_instance", "description": "Get detailed instance info", "params": ["instance_index: usize"]},
            {"command": "help", "description": "List all available commands", "params": []},
        ]);
        ApiResponse::ok("Available commands", commands)
    }
}

// ============================================================
// Helper functions
// ============================================================

fn mesh_to_json_value(mesh: &draper_mesh::TriangleMesh) -> serde_json::Value {
    let vertices: Vec<f64> = mesh.vertices.iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .collect();
    let triangles: Vec<u32> = mesh.triangles.iter()
        .flat_map(|t| [t[0], t[1], t[2]])
        .collect();

    serde_json::json!({
        "vertex_count": mesh.vertex_count(),
        "triangle_count": mesh.triangle_count(),
        "vertices": vertices,
        "triangles": triangles,
        "has_normals": mesh.normals.is_some(),
        "has_face_normals": mesh.face_normals.is_some(),
        "has_colors": mesh.triangle_colors.is_some(),
        "has_face_ids": mesh.triangle_face_ids.is_some(),
    })
}

fn compose_transforms(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut result = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    result
}

fn default_true() -> bool {
    true
}
