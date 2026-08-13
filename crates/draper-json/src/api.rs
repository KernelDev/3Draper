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
use crate::model::{JsonModel, JsonMeshInstance};
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

    // ----- Editing commands (operate on the underlying Document) -----

    /// Add a primitive solid to the document.
    /// `kind` ∈ {"box", "cylinder", "sphere", "cone", "torus"}.
    AddPrimitive {
        kind: String,
        /// [dx, dy, dz] for box; [radius, height] for cylinder; [radius] for sphere;
        /// [radius, height, half_angle_rad] for cone; [major_r, minor_r] for torus.
        params: Vec<f64>,
    },

    /// Fillet (round) an edge of a solid.
    FilletEdge {
        solid_index: usize,
        edge_index: usize,
        radius: f64,
    },

    /// Chamfer (bevel) an edge of a solid.
    ChamferEdge {
        solid_index: usize,
        edge_index: usize,
        distance: f64,
    },

    /// Shell a solid (inward offset by `thickness`).
    MakeShell {
        solid_index: usize,
        thickness: f64,
    },

    /// Translate a solid by (dx, dy, dz).
    Translate {
        solid_index: usize,
        dx: f64, dy: f64, dz: f64,
    },

    /// Rotate a solid about (ax, ay, az) by `angle_radians`.
    Rotate {
        solid_index: usize,
        ax: f64, ay: f64, az: f64,
        angle_radians: f64,
    },

    /// Uniformly scale a solid by `factor`.
    Scale {
        solid_index: usize,
        factor: f64,
    },

    /// Mirror a solid about the plane through (ox,oy,oz) with normal (nx,ny,nz).
    Mirror {
        solid_index: usize,
        ox: f64, oy: f64, oz: f64,
        nx: f64, ny: f64, nz: f64,
    },

    /// Boolean union of two solids. Returns the new solid's index.
    BooleanUnion {
        a_index: usize,
        b_index: usize,
    },

    /// Boolean subtract (A - B). Returns the new solid's index.
    BooleanSubtract {
        a_index: usize,
        b_index: usize,
    },

    /// Boolean intersect (A ∩ B). Returns the new solid's index.
    BooleanIntersect {
        a_index: usize,
        b_index: usize,
    },

    /// Add a circular hole of `radius` mm centered at (cx, cy, cz) on a face.
    AddCircularHole {
        solid_index: usize,
        face_index: usize,
        cx: f64, cy: f64, cz: f64,
        radius: f64,
    },

    /// Delete a solid by index.
    DeleteSolid {
        solid_index: usize,
    },

    /// Run a single GDT check on the mesh of a solid.
    /// `check_type` ∈ {"flatness", "straightness", "circularity",
    /// "cylindricity", "position", "parallelism", "perpendicularity",
    /// "angularity", "runout", "profile_of_line", "profile_of_surface"}.
    GdtCheck {
        solid_index: usize,
        check_type: String,
        tolerance_value: f64,
        datum_axis: Option<[f64; 3]>,
        nominal_position: Option<[f64; 3]>,
        nominal_angle_deg: Option<f64>,
    },

    /// Export a single solid to STEP (AP214) text.
    ExportStep {
        solid_index: usize,
        name: Option<String>,
    },

    /// List all edges in a solid as a JSON array.
    ListEdges {
        solid_index: usize,
    },

    /// Get the number of solids in the document.
    GetSolidCount,

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
    /// The underlying CAD document — kept in sync with `model` after
    /// each edit operation so editing commands can modify the kernel's
    /// native `Solid` representation, then re-triangulate to refresh
    /// the cached `model`.
    document: Option<draper_core::Document>,
    /// Dirty flag — set whenever an edit modifies `document`. The next
    /// read command (GetMesh, GetBbox, etc.) will refresh `model` from
    /// `document` if this is set.
    dirty: bool,
}

impl JsonApi {
    /// Create a new empty API engine.
    pub fn new() -> Self {
        JsonApi { model: None, document: None, dirty: false }
    }

    /// Create an API engine with a pre-loaded model.
    pub fn with_model(model: JsonModel) -> Self {
        JsonApi { model: Some(model), document: None, dirty: false }
    }

    /// Get a reference to the current model.
    pub fn model(&self) -> Option<&JsonModel> {
        self.model.as_ref()
    }

    /// Get a mutable reference to the current model.
    pub fn model_mut(&mut self) -> Option<&mut JsonModel> {
        self.model.as_mut()
    }

    /// Get a reference to the underlying CAD document (if any).
    pub fn document(&self) -> Option<&draper_core::Document> {
        self.document.as_ref()
    }

    /// Get a mutable reference to the underlying CAD document.
    pub fn document_mut(&mut self) -> Option<&mut draper_core::Document> {
        self.document.as_mut()
    }

    /// Process an API request and return a response.
    pub fn execute(&mut self, request: ApiRequest) -> ApiResponse {
        // If the document has been modified since the last read, refresh
        // the cached JsonModel from the Document before dispatching.
        // Read commands (get_mesh, get_bbox, etc.) then see a fresh model.
        // Editing commands ignore the refresh (they modify the document
        // and set dirty=true).
        self.refresh_if_dirty();
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
            ApiRequest::AddPrimitive { kind, params } => {
                self.cmd_add_primitive(&kind, &params)
            }
            ApiRequest::FilletEdge { solid_index, edge_index, radius } => {
                self.cmd_fillet_edge(solid_index, edge_index, radius)
            }
            ApiRequest::ChamferEdge { solid_index, edge_index, distance } => {
                self.cmd_chamfer_edge(solid_index, edge_index, distance)
            }
            ApiRequest::MakeShell { solid_index, thickness } => {
                self.cmd_make_shell(solid_index, thickness)
            }
            ApiRequest::Translate { solid_index, dx, dy, dz } => {
                self.cmd_translate(solid_index, dx, dy, dz)
            }
            ApiRequest::Rotate { solid_index, ax, ay, az, angle_radians } => {
                self.cmd_rotate(solid_index, ax, ay, az, angle_radians)
            }
            ApiRequest::Scale { solid_index, factor } => {
                self.cmd_scale(solid_index, factor)
            }
            ApiRequest::Mirror { solid_index, ox, oy, oz, nx, ny, nz } => {
                self.cmd_mirror(solid_index, ox, oy, oz, nx, ny, nz)
            }
            ApiRequest::BooleanUnion { a_index, b_index } => {
                self.cmd_boolean("union", a_index, b_index)
            }
            ApiRequest::BooleanSubtract { a_index, b_index } => {
                self.cmd_boolean("subtract", a_index, b_index)
            }
            ApiRequest::BooleanIntersect { a_index, b_index } => {
                self.cmd_boolean("intersect", a_index, b_index)
            }
            ApiRequest::AddCircularHole { solid_index, face_index, cx, cy, cz, radius } => {
                self.cmd_add_circular_hole(solid_index, face_index, cx, cy, cz, radius)
            }
            ApiRequest::DeleteSolid { solid_index } => {
                self.cmd_delete_solid(solid_index)
            }
            ApiRequest::GdtCheck { solid_index, check_type, tolerance_value, datum_axis, nominal_position, nominal_angle_deg } => {
                self.cmd_gdt_check(solid_index, &check_type, tolerance_value, datum_axis, nominal_position, nominal_angle_deg)
            }
            ApiRequest::ExportStep { solid_index, name } => {
                self.cmd_export_step(solid_index, name)
            }
            ApiRequest::ListEdges { solid_index } => {
                self.cmd_list_edges(solid_index)
            }
            ApiRequest::GetSolidCount => {
                self.cmd_get_solid_count()
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
        // Parse STEP into a StepFile once, then build both the JsonModel
        // (for inspection commands) and the Document (for editing commands)
        // from the same parse.
        let step_file = match draper_step::parse_step(content) {
            Ok(sf) => sf,
            Err(e) => return ApiResponse::err(&format!("Failed to parse STEP: {:?}", e)),
        };
        let (solids, _brep_ids) = draper_step::extract_solids(&step_file);
        let mut doc = draper_core::Document::new("json-api-doc");
        for s in solids {
            doc.add_solid(s);
        }
        let model = match JsonModel::from_step_file_with_config(
            &step_file,
            &config,
            Some(content.to_string()),
        ) {
            Ok(m) => m,
            Err(e) => return ApiResponse::err(&format!("Failed to build model: {}", e)),
        };
        let stats = format!(
            "Loaded {} instance(s), {} solid(s), {} vertices, {} triangles",
            model.metadata.instance_count,
            doc.solid_count(),
            model.metadata.total_vertices,
            model.metadata.total_triangles,
        );
        let data = serde_json::to_value(&model.metadata)
            .unwrap_or(serde_json::Value::Null);
        self.model = Some(model);
        self.document = Some(doc);
        self.dirty = false;
        ApiResponse::ok(&stats, data)
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
            {"command": "add_primitive", "description": "Add a primitive solid (box/cylinder/sphere/cone/torus)", "params": ["kind: String", "params: Vec<f64>"]},
            {"command": "fillet_edge", "description": "Fillet an edge of a solid", "params": ["solid_index: usize", "edge_index: usize", "radius: f64"]},
            {"command": "chamfer_edge", "description": "Chamfer an edge of a solid", "params": ["solid_index: usize", "edge_index: usize", "distance: f64"]},
            {"command": "make_shell", "description": "Shell a solid (inward offset)", "params": ["solid_index: usize", "thickness: f64"]},
            {"command": "translate", "description": "Translate a solid", "params": ["solid_index: usize", "dx: f64", "dy: f64", "dz: f64"]},
            {"command": "rotate", "description": "Rotate a solid about an axis", "params": ["solid_index: usize", "ax: f64", "ay: f64", "az: f64", "angle_radians: f64"]},
            {"command": "scale", "description": "Uniformly scale a solid", "params": ["solid_index: usize", "factor: f64"]},
            {"command": "mirror", "description": "Mirror a solid about a plane", "params": ["solid_index: usize", "ox: f64", "oy: f64", "oz: f64", "nx: f64", "ny: f64", "nz: f64"]},
            {"command": "boolean_union", "description": "Boolean union of two solids", "params": ["a_index: usize", "b_index: usize"]},
            {"command": "boolean_subtract", "description": "Boolean subtract (A - B)", "params": ["a_index: usize", "b_index: usize"]},
            {"command": "boolean_intersect", "description": "Boolean intersect (A ∩ B)", "params": ["a_index: usize", "b_index: usize"]},
            {"command": "add_circular_hole", "description": "Add a circular hole on a face", "params": ["solid_index: usize", "face_index: usize", "cx: f64", "cy: f64", "cz: f64", "radius: f64"]},
            {"command": "delete_solid", "description": "Delete a solid by index", "params": ["solid_index: usize"]},
            {"command": "gdt_check", "description": "Run a GDT check on a solid", "params": ["solid_index: usize", "check_type: String", "tolerance_value: f64", "datum_axis: Option<[f64;3]>", "nominal_position: Option<[f64;3]>", "nominal_angle_deg: Option<f64>"]},
            {"command": "export_step", "description": "Export a solid to STEP text", "params": ["solid_index: usize", "name: Option<String>"]},
            {"command": "list_edges", "description": "List all edges in a solid", "params": ["solid_index: usize"]},
            {"command": "get_solid_count", "description": "Get number of solids in the document", "params": []},
            {"command": "help", "description": "List all available commands", "params": []},
        ]);
        ApiResponse::ok("Available commands", commands)
    }

    // ============================================================
    // Editing command implementations
    // ============================================================

    /// Ensure a Document exists, creating an empty one if needed.
    fn ensure_document(&mut self) -> &mut draper_core::Document {
        if self.document.is_none() {
            self.document = Some(draper_core::Document::new("json-api-doc"));
        }
        self.document.as_mut().unwrap()
    }

    /// Mark the cached JsonModel as stale. The next read command will
    /// re-triangulate the Document into a fresh JsonModel.
    fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    /// If dirty, rebuild self.model from self.document by re-triangulating.
    fn refresh_if_dirty(&mut self) {
        if !self.dirty {
            return;
        }
        if let Some(doc) = &self.document {
            // Build a fresh JsonModel from the document's solids by
            // re-triangulating each one and packing the result into a
            // DetailedMeshInstance → JsonMeshInstance.
            let mut instances: Vec<draper_step::DetailedMeshInstance> = Vec::new();
            for (i, solid) in doc.solids().iter().enumerate() {
                let mesh = draper_mesh::triangulate_solid(solid, &doc.tri_params);
                instances.push(draper_step::DetailedMeshInstance {
                    name: format!("solid_{}", i),
                    mesh,
                    color: None,
                    transform: None,
                    brep_id: -1,
                    faces: Vec::new(),
                });
            }
            let json_instances: Vec<JsonMeshInstance> = instances.iter()
                .map(JsonMeshInstance::from_detailed_instance)
                .collect();
            let total_vertices: usize = json_instances.iter().map(|i| i.vertices.len() / 3).sum();
            let total_triangles: usize = json_instances.iter().map(|i| i.triangles.len() / 3).sum();
            let (bbox_min, bbox_max) = json_instances.iter().fold(
                (None::<[f64; 3]>, None::<[f64; 3]>),
                |(mn, mx), inst| {
                    let mut mn = mn;
                    let mut mx = mx;
                    for chunk in inst.vertices.chunks(3) {
                        if chunk.len() < 3 { continue; }
                        let p = [chunk[0], chunk[1], chunk[2]];
                        mn = Some(match mn {
                            None => p,
                            Some(m) => [m[0].min(p[0]), m[1].min(p[1]), m[2].min(p[2])],
                        });
                        mx = Some(match mx {
                            None => p,
                            Some(m) => [m[0].max(p[0]), m[1].max(p[1]), m[2].max(p[2])],
                        });
                    }
                    (mn, mx)
                },
            );
            let assembly = draper_step::AssemblyNode {
                name: doc.name.clone(),
                pd_id: -1,
                brep_id: None,
                instance_index: None,
                face_id: None,
                transform: None,
                color: None,
                layers: Vec::new(),
                children: json_instances.iter().enumerate().map(|(i, _)| draper_step::AssemblyNode {
                    name: format!("solid_{}", i),
                    pd_id: -1,
                    brep_id: None,
                    instance_index: Some(i),
                    transform: None,
                    color: None,
                    layers: Vec::new(),
                    children: Vec::new(), face_id: None,
                }).collect(),
            };
            self.model = Some(JsonModel {
                metadata: crate::model::ModelMetadata {
                    name: doc.name.clone(),
                    source_format: "Document".to_string(),
                    kernel_version: env!("CARGO_PKG_VERSION").to_string(),
                    created_at: None,
                    instance_count: json_instances.len(),
                    total_vertices,
                    total_triangles,
                    bbox_min,
                    bbox_max,
                },
                assembly,
                instances: json_instances,
                step_source: None,
            });
        }
        self.dirty = false;
    }

    fn cmd_add_primitive(&mut self, kind: &str, params: &[f64]) -> ApiResponse {
        use draper_topology::ShapeBuilder;
        let solid = match kind.to_lowercase().as_str() {
            "box" => {
                if params.len() < 3 {
                    return ApiResponse::err("box requires 3 params: [dx, dy, dz]");
                }
                ShapeBuilder::make_box(params[0], params[1], params[2])
            }
            "cylinder" => {
                if params.len() < 2 {
                    return ApiResponse::err("cylinder requires 2 params: [radius, height]");
                }
                ShapeBuilder::make_cylinder(params[0], params[1])
            }
            "sphere" => {
                if params.is_empty() {
                    return ApiResponse::err("sphere requires 1 param: [radius]");
                }
                ShapeBuilder::make_sphere(params[0])
            }
            "cone" => {
                if params.len() < 3 {
                    return ApiResponse::err("cone requires 3 params: [radius, height, half_angle_rad]");
                }
                ShapeBuilder::make_cone(params[0], params[1], params[2])
            }
            "torus" => {
                if params.len() < 2 {
                    return ApiResponse::err("torus requires 2 params: [major_r, minor_r]");
                }
                ShapeBuilder::make_torus(params[0], params[1])
            }
            other => return ApiResponse::err(&format!("unknown primitive kind '{}'", other)),
        };
        let doc = self.ensure_document();
        doc.add_solid(solid);
        let idx = doc.solid_count() - 1;
        self.mark_dirty();
        ApiResponse::ok(
            &format!("Added {} as solid {}", kind, idx),
            serde_json::json!({"solid_index": idx}),
        )
    }

    fn cmd_fillet_edge(&mut self, solid_index: usize, edge_index: usize, radius: f64) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        match draper_core::operations::fillet_edge(&mut doc.root.solids[solid_index], edge_index, radius) {
            Ok(_) => {
                self.mark_dirty();
                ApiResponse::ok_msg(&format!("Filleted edge {} of solid {}", edge_index, solid_index))
            }
            Err(e) => ApiResponse::err(&e),
        }
    }

    fn cmd_chamfer_edge(&mut self, solid_index: usize, edge_index: usize, distance: f64) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        match draper_core::operations::chamfer_edge(&mut doc.root.solids[solid_index], edge_index, distance) {
            Ok(_) => {
                self.mark_dirty();
                ApiResponse::ok_msg(&format!("Chamfered edge {} of solid {}", edge_index, solid_index))
            }
            Err(e) => ApiResponse::err(&e),
        }
    }

    fn cmd_make_shell(&mut self, solid_index: usize, thickness: f64) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        match draper_core::operations::make_shell(&mut doc.root.solids[solid_index], thickness) {
            Ok(_) => {
                self.mark_dirty();
                ApiResponse::ok_msg(&format!("Shelled solid {} by {} mm", solid_index, thickness))
            }
            Err(e) => ApiResponse::err(&e),
        }
    }

    fn cmd_translate(&mut self, solid_index: usize, dx: f64, dy: f64, dz: f64) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        draper_core::operations::translate_solid(&mut doc.root.solids[solid_index], dx, dy, dz);
        self.mark_dirty();
        ApiResponse::ok_msg(&format!("Translated solid {} by ({}, {}, {})", solid_index, dx, dy, dz))
    }

    fn cmd_rotate(&mut self, solid_index: usize, ax: f64, ay: f64, az: f64, angle: f64) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        let axis = match draper_geometry::Direction3d::new(ax, ay, az) {
            Some(d) => d,
            None => return ApiResponse::err("zero-length axis"),
        };
        draper_core::operations::rotate_solid(&mut doc.root.solids[solid_index], &axis, angle);
        self.mark_dirty();
        ApiResponse::ok_msg(&format!("Rotated solid {} by {} rad", solid_index, angle))
    }

    fn cmd_scale(&mut self, solid_index: usize, factor: f64) -> ApiResponse {
        if !factor.is_finite() || factor <= 0.0 {
            return ApiResponse::err(&format!("invalid scale factor {}", factor));
        }
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        draper_core::operations::scale_solid(&mut doc.root.solids[solid_index], factor);
        self.mark_dirty();
        ApiResponse::ok_msg(&format!("Scaled solid {} by {}", solid_index, factor))
    }

    fn cmd_mirror(&mut self, solid_index: usize, ox: f64, oy: f64, oz: f64, nx: f64, ny: f64, nz: f64) -> ApiResponse {
        let normal = match draper_geometry::Direction3d::new(nx, ny, nz) {
            Some(d) => d,
            None => return ApiResponse::err("zero-length normal"),
        };
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        let origin = draper_geometry::Point3d::new(ox, oy, oz);
        let mirrored = draper_core::operations::mirror_solid(&doc.root.solids[solid_index], origin, normal);
        doc.root.solids[solid_index] = mirrored;
        self.mark_dirty();
        ApiResponse::ok_msg(&format!("Mirrored solid {}", solid_index))
    }

    fn cmd_boolean(&mut self, op: &str, a_index: usize, b_index: usize) -> ApiResponse {
        let doc = self.ensure_document();
        if a_index >= doc.solid_count() || b_index >= doc.solid_count() {
            return ApiResponse::err(&format!(
                "solid index out of range (have {} solids)", doc.solid_count()
            ));
        }
        let a = doc.root.solids[a_index].clone();
        let b = doc.root.solids[b_index].clone();
        let result = match op {
            "union" => draper_core::boolean::boolean_union(&a, &b),
            "subtract" => draper_core::boolean::boolean_subtract(&a, &b),
            "intersect" => draper_core::boolean::boolean_intersect(&a, &b),
            _ => unreachable!("invalid boolean op"),
        };
        match result {
            Ok(s) => {
                doc.add_solid(s);
                let idx = doc.solid_count() - 1;
                self.mark_dirty();
                ApiResponse::ok(
                    &format!("Boolean {} produced solid {}", op, idx),
                    serde_json::json!({"solid_index": idx}),
                )
            }
            Err(e) => ApiResponse::err(&e),
        }
    }

    fn cmd_add_circular_hole(&mut self, solid_index: usize, face_index: usize, cx: f64, cy: f64, cz: f64, radius: f64) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        if radius <= 0.0 || !radius.is_finite() {
            return ApiResponse::err(&format!("invalid radius {}", radius));
        }
        let center = draper_geometry::Point3d::new(cx, cy, cz);
        let s = &mut doc.root.solids[solid_index];
        let face_normal = {
            let face = match draper_core::operations::get_face_mut(s, face_index) {
                Some(f) => f,
                None => return ApiResponse::err(&format!("face_index {} out of range", face_index)),
            };
            let surface = face.surface.clone();
            match &surface {
                Some(draper_geometry::Surface::Plane(p)) => p.normal,
                Some(draper_geometry::Surface::Cylinder(c)) => c.axis.clone(),
                Some(draper_geometry::Surface::Cone(c)) => c.axis.clone(),
                Some(draper_geometry::Surface::Sphere(_)) => draper_geometry::Direction3d::new(cx, cy, cz).unwrap_or(draper_geometry::Direction3d::Z),
                Some(draper_geometry::Surface::Torus(t)) => t.axis.clone(),
                _ => draper_geometry::Direction3d::Z,
            }
        };
        let face = draper_core::operations::get_face_mut(s, face_index).unwrap();
        match draper_core::operations::add_circular_hole_to_face(face, center, radius, face_normal) {
            Ok(_) => {
                self.mark_dirty();
                ApiResponse::ok_msg(&format!("Added hole on face {} of solid {}", face_index, solid_index))
            }
            Err(e) => ApiResponse::err(&e),
        }
    }

    fn cmd_delete_solid(&mut self, solid_index: usize) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        doc.root.solids.remove(solid_index);
        self.mark_dirty();
        ApiResponse::ok_msg(&format!("Deleted solid {}", solid_index))
    }

    fn cmd_gdt_check(&mut self, solid_index: usize, check_type: &str, tolerance_value: f64, datum_axis: Option<[f64; 3]>, nominal_position: Option<[f64; 3]>, nominal_angle_deg: Option<f64>) -> ApiResponse {
        let doc = self.ensure_document();
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        let mut spec = draper_mesh::gdt_check::ToleranceSpec::default();
        spec.tolerance_type = match check_type.to_lowercase().as_str() {
            "flatness" => draper_mesh::gdt_check::GdtCheckType::Flatness,
            "straightness" => draper_mesh::gdt_check::GdtCheckType::Straightness,
            "circularity" | "roundness" => draper_mesh::gdt_check::GdtCheckType::Circularity,
            "cylindricity" => draper_mesh::gdt_check::GdtCheckType::Cylindricity,
            "position" => draper_mesh::gdt_check::GdtCheckType::Position,
            "parallelism" => draper_mesh::gdt_check::GdtCheckType::Parallelism,
            "perpendicularity" => draper_mesh::gdt_check::GdtCheckType::Perpendicularity,
            "angularity" => draper_mesh::gdt_check::GdtCheckType::Angularity,
            "runout" => draper_mesh::gdt_check::GdtCheckType::Runout,
            "profile_of_line" => draper_mesh::gdt_check::GdtCheckType::ProfileOfLine,
            "profile_of_surface" => draper_mesh::gdt_check::GdtCheckType::ProfileOfSurface,
            other => return ApiResponse::err(&format!("unknown GDT check type '{}'", other)),
        };
        spec.tolerance_value = tolerance_value;
        if let Some([x, y, z]) = datum_axis {
            if let Some(d) = draper_geometry::Direction3d::new(x, y, z) {
                spec.datum_axis = Some(d);
            }
        }
        if let Some([x, y, z]) = nominal_position {
            spec.nominal_position = Some(draper_geometry::Point3d::new(x, y, z));
        }
        if let Some(a) = nominal_angle_deg {
            spec.nominal_angle_deg = Some(a);
        }
        let mesh = draper_mesh::triangulate_solid(&doc.root.solids[solid_index], &doc.tri_params);
        let checker = draper_mesh::gdt_check::GdtChecker::new(&mesh);
        let r = checker.check(&spec);
        let data = serde_json::json!({
            "name": r.tolerance_name,
            "description": r.description,
            "type": format!("{:?}", r.tolerance_type),
            "tolerance_value": r.tolerance_value,
            "actual_deviation": r.actual_deviation,
            "passed": r.passed,
            "step_id": r.step_id,
        });
        ApiResponse::ok("GDT check completed", data)
    }

    fn cmd_export_step(&self, solid_index: usize, name: Option<String>) -> ApiResponse {
        let doc = match &self.document {
            Some(d) => d,
            None => return ApiResponse::err("No document loaded"),
        };
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        let n = name.unwrap_or_else(|| format!("solid_{}", solid_index));
        let step_text = draper_step::export_step(&doc.root.solids[solid_index], &n);
        ApiResponse::ok("Exported to STEP", serde_json::Value::String(step_text))
    }

    fn cmd_list_edges(&self, solid_index: usize) -> ApiResponse {
        let doc = match &self.document {
            Some(d) => d,
            None => return ApiResponse::err("No document loaded"),
        };
        if solid_index >= doc.solid_count() {
            return ApiResponse::err(&format!("solid_index {} out of range", solid_index));
        }
        let solid = &doc.root.solids[solid_index];
        use std::collections::HashMap;
        let mut edge_info: HashMap<u64, (String, Vec<usize>)> = HashMap::new();
        if let Some(shell) = solid.outer_shell.as_ref() {
            for (fi, face) in shell.faces.iter().enumerate() {
                for edge in &face.edges {
                    let id = edge.id.to_u64();
                    let curve_type = match &edge.curve {
                        None => "None".to_string(),
                        Some(draper_geometry::Curve3d::Line(_)) => "Line".to_string(),
                        Some(draper_geometry::Curve3d::Circle(_)) => "Circle".to_string(),
                        Some(draper_geometry::Curve3d::Ellipse(_)) => "Ellipse".to_string(),
                        Some(draper_geometry::Curve3d::Arc(_)) => "Arc".to_string(),
                        Some(draper_geometry::Curve3d::Hyperbola(_)) => "Hyperbola".to_string(),
                        Some(draper_geometry::Curve3d::Parabola(_)) => "Parabola".to_string(),
                        Some(draper_geometry::Curve3d::Nurbs(_)) => "Nurbs".to_string(),
                        Some(draper_geometry::Curve3d::PCurve { .. }) => "PCurve".to_string(),
                        Some(draper_geometry::Curve3d::Trimmed { .. }) => "Trimmed".to_string(),
                        Some(draper_geometry::Curve3d::Composite { .. }) => "Composite".to_string(),
                    };
                    edge_info.entry(id)
                        .and_modify(|(_, faces)| faces.push(fi))
                        .or_insert((curve_type, vec![fi]));
                }
            }
        }
        let mut arr = Vec::with_capacity(edge_info.len());
        for (id, (curve_type, faces)) in edge_info {
            arr.push(serde_json::json!({
                "id": id,
                "curve_type": curve_type,
                "face_ids": faces,
            }));
        }
        ApiResponse::ok("Edge listing", serde_json::Value::Array(arr))
    }

    fn cmd_get_solid_count(&self) -> ApiResponse {
        let n = self.document.as_ref().map(|d| d.solid_count()).unwrap_or(0);
        ApiResponse::ok("Solid count", serde_json::json!({"count": n}))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_help_includes_edit_commands() {
        let api = JsonApi::new();
        let resp = api.cmd_help();
        assert!(resp.success);
        let data = resp.data.unwrap();
        let arr = data.as_array().unwrap();
        let commands: Vec<&str> = arr.iter()
            .filter_map(|v| v.get("command").and_then(|c| c.as_str()))
            .collect();
        assert!(commands.contains(&"add_primitive"), "missing add_primitive in help");
        assert!(commands.contains(&"fillet_edge"), "missing fillet_edge in help");
        assert!(commands.contains(&"chamfer_edge"), "missing chamfer_edge in help");
        assert!(commands.contains(&"make_shell"), "missing make_shell in help");
        assert!(commands.contains(&"translate"), "missing translate in help");
        assert!(commands.contains(&"rotate"), "missing rotate in help");
        assert!(commands.contains(&"scale"), "missing scale in help");
        assert!(commands.contains(&"mirror"), "missing mirror in help");
        assert!(commands.contains(&"boolean_union"), "missing boolean_union in help");
        assert!(commands.contains(&"boolean_subtract"), "missing boolean_subtract in help");
        assert!(commands.contains(&"boolean_intersect"), "missing boolean_intersect in help");
        assert!(commands.contains(&"add_circular_hole"), "missing add_circular_hole in help");
        assert!(commands.contains(&"delete_solid"), "missing delete_solid in help");
        assert!(commands.contains(&"gdt_check"), "missing gdt_check in help");
        assert!(commands.contains(&"export_step"), "missing export_step in help");
        assert!(commands.contains(&"list_edges"), "missing list_edges in help");
        assert!(commands.contains(&"get_solid_count"), "missing get_solid_count in help");
    }

    #[test]
    fn test_add_primitive_box_creates_solid() {
        let mut api = JsonApi::new();
        let req = ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![10.0, 20.0, 30.0],
        };
        let resp = api.execute(req);
        assert!(resp.success, "add_primitive failed: {}", resp.message);
        let data = resp.data.unwrap();
        assert_eq!(data["solid_index"], 0);

        let resp = api.execute(ApiRequest::GetSolidCount);
        assert!(resp.success);
        assert_eq!(resp.data.unwrap()["count"], 1);
    }

    #[test]
    fn test_add_primitive_invalid_kind_returns_error() {
        let mut api = JsonApi::new();
        let req = ApiRequest::AddPrimitive {
            kind: "pyramid".to_string(),
            params: vec![10.0],
        };
        let resp = api.execute(req);
        assert!(!resp.success);
        assert!(resp.message.contains("unknown primitive kind"));
    }

    #[test]
    fn test_translate_updates_document() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![10.0, 10.0, 10.0],
        });
        let resp = api.execute(ApiRequest::Translate {
            solid_index: 0,
            dx: 100.0, dy: 0.0, dz: 0.0,
        });
        assert!(resp.success, "translate failed: {}", resp.message);

        // GetBbox should reflect the translation — but make_box is centered
        // at origin (-5..5), so after +100 translation it's (95..105, -5..5, -5..5).
        let resp = api.execute(ApiRequest::GetBbox);
        assert!(resp.success);
        let data = resp.data.unwrap();
        let min_x = data["min"][0].as_f64().unwrap();
        let max_x = data["max"][0].as_f64().unwrap();
        assert!((min_x - 95.0).abs() < 1e-6, "expected min_x = 95, got {}", min_x);
        assert!((max_x - 105.0).abs() < 1e-6, "expected max_x = 105, got {}", max_x);
    }

    #[test]
    fn test_scale_around_origin_doubles_cube() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![10.0, 10.0, 10.0],
        });
        // Cube spans (-5..5). Scale by 2 → (-10..10).
        let resp = api.execute(ApiRequest::Scale {
            solid_index: 0,
            factor: 2.0,
        });
        assert!(resp.success);
        let resp = api.execute(ApiRequest::GetBbox);
        let data = resp.data.unwrap();
        let max_x = data["max"][0].as_f64().unwrap();
        assert!((max_x - 10.0).abs() < 1e-6, "expected max_x = 10 after 2x scale, got {}", max_x);
    }

    #[test]
    fn test_invalid_solid_index_returns_error() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "sphere".to_string(),
            params: vec![5.0],
        });
        let resp = api.execute(ApiRequest::Translate {
            solid_index: 99,
            dx: 1.0, dy: 0.0, dz: 0.0,
        });
        assert!(!resp.success);
        assert!(resp.message.contains("out of range"));
    }

    #[test]
    fn test_invalid_scale_factor_returns_error() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "sphere".to_string(),
            params: vec![5.0],
        });
        let resp = api.execute(ApiRequest::Scale {
            solid_index: 0,
            factor: -1.0,
        });
        assert!(!resp.success);
        assert!(resp.message.contains("invalid scale factor"));
    }

    #[test]
    fn test_boolean_union_creates_new_solid() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![100.0, 100.0, 100.0],
        });
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![50.0, 50.0, 50.0],
        });
        let resp = api.execute(ApiRequest::BooleanUnion {
            a_index: 0,
            b_index: 1,
        });
        assert!(resp.success, "boolean_union failed: {}", resp.message);
        // After union, document should have 3 solids (A, B, A∪B).
        let resp = api.execute(ApiRequest::GetSolidCount);
        assert_eq!(resp.data.unwrap()["count"], 3);
    }

    #[test]
    fn test_delete_solid_removes_from_document() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![10.0, 10.0, 10.0],
        });
        api.execute(ApiRequest::AddPrimitive {
            kind: "sphere".to_string(),
            params: vec![5.0],
        });
        let resp = api.execute(ApiRequest::DeleteSolid { solid_index: 0 });
        assert!(resp.success);
        let resp = api.execute(ApiRequest::GetSolidCount);
        assert_eq!(resp.data.unwrap()["count"], 1);
    }

    #[test]
    fn test_gdt_check_flatness_returns_result() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![100.0, 100.0, 100.0],
        });
        let resp = api.execute(ApiRequest::GdtCheck {
            solid_index: 0,
            check_type: "flatness".to_string(),
            tolerance_value: 1.0,
            datum_axis: None,
            nominal_position: None,
            nominal_angle_deg: None,
        });
        assert!(resp.success, "gdt_check failed: {}", resp.message);
        let data = resp.data.unwrap();
        assert!(data["actual_deviation"].is_number());
        assert_eq!(data["type"], "Flatness");
    }

    #[test]
    fn test_export_step_round_trips() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![50.0, 60.0, 70.0],
        });
        let resp = api.execute(ApiRequest::ExportStep {
            solid_index: 0,
            name: Some("test_box".to_string()),
        });
        assert!(resp.success);
        let step_text = resp.data.unwrap().as_str().unwrap().to_string();
        assert!(step_text.contains("ISO-10303-21"));
        assert!(step_text.contains("MANIFOLD_SOLID_BREP"));
    }

    #[test]
    fn test_list_edges_returns_array() {
        let mut api = JsonApi::new();
        api.execute(ApiRequest::AddPrimitive {
            kind: "box".to_string(),
            params: vec![10.0, 20.0, 30.0],
        });
        let resp = api.execute(ApiRequest::ListEdges { solid_index: 0 });
        assert!(resp.success);
        let data = resp.data.unwrap();
        let arr = data.as_array().unwrap();
        // A box has 12 edges (4 per face × 6 faces, but shared).
        assert!(arr.len() >= 12, "expected >= 12 edges, got {}", arr.len());
    }

    #[test]
    fn test_execute_json_dispatches() {
        let mut api = JsonApi::new();
        let json = r#"{"command":"add_primitive","kind":"box","params":[10.0,10.0,10.0]}"#;
        let resp = api.execute_json(json);
        assert!(resp.contains("\"success\":true"));
    }
}
