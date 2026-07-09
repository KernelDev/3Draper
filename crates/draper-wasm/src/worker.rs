// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-wasm worker module
//!
//! Lightweight WASM module for the STEP triangulation Web Worker.
//!
//! This module exposes only the kernel functions needed for off-main-thread
//! STEP parsing and BREP triangulation — no eframe/egui/wgpu dependencies.
//! It is compiled as a **separate** WASM binary (`draper-worker.wasm`) that
//! the worker loads via `importScripts` / ES module import.
//!
//! # Exports
//!
//! - `worker_init()` — panic hook + logger (must call first)
//! - `parse_step_worker(content)` — parse STEP text → `ParseResult`
//! - `triangulate_brep_worker(ctx_id, brep_id, name, transform_json, lod, profile)` → `MeshDataResult`
//! - `drop_parse_context(ctx_id)` — free the stored `OwnedStepConversionContext`
//!
//! # Design
//!
//! The worker holds one `OwnedStepConversionContext` at a time, referenced by
//! an opaque `ctx_id` (currently always 0 — we only parse one file at a time).
//! After `parse_step_worker`, the context is stored in a thread-local `RefCell`.
//! Subsequent `triangulate_brep_worker` calls use it. When done, `drop_parse_context`
//! frees the memory.

#![allow(clippy::unused_unit)]

use draper_mesh::triangulate::SteinerBudgetProfile;
use draper_mesh::wasm_api::MeshData;
use draper_step::{
    parse_step, step_structure_lazy, OwnedStepConversionContext, PendingBrepInstance,
};
use wasm_bindgen::prelude::*;

// ============================================================
// State: stored parse context
// ============================================================

use std::cell::RefCell;

thread_local! {
    /// The single active conversion context. Only one STEP file can be
    /// processed at a time in the worker — this matches the serial message
    /// protocol of `worker.js`.
    static CONTEXT: RefCell<Option<WorkerState>> = RefCell::new(None);
}

/// Internal state kept after `parse_step_worker`.
struct WorkerState {
    context: OwnedStepConversionContext,
    pending_breps: Vec<PendingBrepInstance>,
}

// ============================================================
// Initialization
// ============================================================

/// Initialize the worker WASM module — installs panic hook and logger.
/// Must be called once before any other function (the worker.js `initWasm`
/// does this automatically after importing the module).
#[wasm_bindgen(start)]
pub fn worker_init() {
    #[cfg(target_arch = "wasm32")]
    {
        console_error_panic_hook::set_once();
        let _ = console_log::init_with_level(log::Level::Info);
    }
}

// ============================================================
// Parse result
// ============================================================

/// Result of STEP parsing, returned to JS.
#[wasm_bindgen]
pub struct ParseResult {
    entity_count: usize,
    face_count: usize,
    brep_count: usize,
    shell_count: usize,
    /// JSON-encoded array of pending BREP descriptors.
    /// Each entry: { name, brep_id, transform: [[f64;4];4] | null, color: [f32;4] | null }
    pending_breps_json: String,
    /// JSON-encoded assembly tree.
    assembly_tree_json: String,
}

#[wasm_bindgen]
impl ParseResult {
    /// Total STEP entity count.
    #[wasm_bindgen(getter)]
    pub fn entity_count(&self) -> usize {
        self.entity_count
    }

    /// Estimated face count.
    #[wasm_bindgen(getter)]
    pub fn face_count(&self) -> usize {
        self.face_count
    }

    /// Number of BREP instances to triangulate.
    #[wasm_bindgen(getter)]
    pub fn brep_count(&self) -> usize {
        self.brep_count
    }

    /// Shell count.
    #[wasm_bindgen(getter)]
    pub fn shell_count(&self) -> usize {
        self.shell_count
    }

    /// JSON array of pending BREP descriptors.
    #[wasm_bindgen(getter)]
    pub fn pending_breps_json(&self) -> String {
        self.pending_breps_json.clone()
    }

    /// JSON assembly tree.
    #[wasm_bindgen(getter)]
    pub fn assembly_tree_json(&self) -> String {
        self.assembly_tree_json.clone()
    }
}

// ============================================================
// Mesh result
// ============================================================

/// Triangulated mesh data returned to JS — flat arrays suitable for
/// `Float32Array` / `Uint32Array` views and zero-copy transfer.
#[wasm_bindgen]
pub struct MeshDataResult {
    vertex_count: usize,
    triangle_count: usize,
    vertices: Vec<f32>,
    indices: Vec<u32>,
    normals: Option<Vec<f32>>,
    face_normals: Option<Vec<f32>>,
    colors: Option<Vec<f32>>,
}

#[wasm_bindgen]
impl MeshDataResult {
    #[wasm_bindgen(getter)]
    pub fn vertex_count(&self) -> usize {
        self.vertex_count
    }

    #[wasm_bindgen(getter)]
    pub fn triangle_count(&self) -> usize {
        self.triangle_count
    }

    /// Flat vertex positions as Float32Array: [x0,y0,z0, x1,y1,z1, ...].
    pub fn vertices(&self) -> js_sys::Float32Array {
        js_sys::Float32Array::from(&self.vertices[..])
    }

    /// Flat triangle indices as Uint32Array: [i0,j0,k0, ...].
    pub fn indices(&self) -> js_sys::Uint32Array {
        js_sys::Uint32Array::from(&self.indices[..])
    }

    /// Vertex normals or empty Float32Array.
    pub fn normals(&self) -> js_sys::Float32Array {
        match &self.normals {
            Some(n) => js_sys::Float32Array::from(&n[..]),
            None => js_sys::Float32Array::new(&JsValue::from(0)),
        }
    }

    /// Face normals or empty Float32Array.
    pub fn face_normals(&self) -> js_sys::Float32Array {
        match &self.face_normals {
            Some(n) => js_sys::Float32Array::from(&n[..]),
            None => js_sys::Float32Array::new(&JsValue::from(0)),
        }
    }

    /// Per-triangle RGBA colors or empty Float32Array.
    pub fn colors(&self) -> js_sys::Float32Array {
        match &self.colors {
            Some(c) => js_sys::Float32Array::from(&c[..]),
            None => js_sys::Float32Array::new(&JsValue::from(0)),
        }
    }
}

// ============================================================
// STEP parsing
// ============================================================

/// Serialize a PendingBrepInstance to a JSON-compatible value.
#[derive(serde::Serialize)]
struct PendingBrepJson {
    name: String,
    brep_id: i64,
    transform: Option<[[f64; 4]; 4]>,
    color: Option<[f32; 4]>,
}

/// Serialize AssemblyNode to JSON (simplified — just name, brep_id, children).
#[derive(serde::Serialize)]
struct AssemblyNodeJson {
    name: String,
    brep_id: Option<i64>,
    instance_index: Option<usize>,
    children: Vec<AssemblyNodeJson>,
}

fn assembly_node_to_json(node: &draper_step::AssemblyNode) -> AssemblyNodeJson {
    AssemblyNodeJson {
        name: node.name.clone(),
        brep_id: node.brep_id,
        instance_index: node.instance_index,
        children: node.children.iter().map(assembly_node_to_json).collect(),
    }
}

/// Parse a STEP file and prepare for progressive triangulation.
///
/// This function:
/// 1. Parses the STEP text into an in-memory `StepFile`
/// 2. Calls `step_structure_lazy` to get assembly tree + pending BREP descriptors
/// 3. Creates an `OwnedStepConversionContext` (with LOD + profile)
/// 4. Stores the context + pending list in thread-local state
/// 5. Returns parse stats + JSON descriptors to JS
///
/// After calling this, the JS worker can call `triangulate_brep_worker`
/// repeatedly to process each BREP.
#[wasm_bindgen]
pub fn parse_step_worker(
    content: &str,
    name: &str,
    lod: f64,
    profile_name: &str,
) -> Result<ParseResult, JsValue> {
    let profile = match profile_name {
        "Mobile" => SteinerBudgetProfile::Mobile,
        "Tablet" => SteinerBudgetProfile::Tablet,
        _ => SteinerBudgetProfile::Desktop,
    };

    // Parse STEP text
    let step_file = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| parse_step(content)))
        .map_err(|_| JsValue::from_str("Panic during STEP parsing"))?;
    let step_file = step_file.map_err(|e| JsValue::from_str(&format!("STEP parse error: {}", e)))?;

    // Build assembly structure (fast — no triangulation)
    let (assembly_tree, pending_breps) =
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| step_structure_lazy(&step_file)))
            .map_err(|_| JsValue::from_str("Panic during structure extraction"))?;

    // Count stats
    let entity_count = step_file.entity_index_ref().len();
    let brep_count = pending_breps.len();
    let face_count: usize = pending_breps
        .iter()
        .map(|p| p.face_count_estimate.unwrap_or(0))
        .sum();
    let shell_count = 0; // Not easily available without full conversion

    // Serialize pending BREP list as JSON
    let pending_json: Vec<PendingBrepJson> = pending_breps
        .iter()
        .map(|p| PendingBrepJson {
            name: p.name.clone(),
            brep_id: p.brep_id,
            transform: p.transform,
            color: p.color,
        })
        .collect();
    let pending_breps_json = serde_json::to_string(&pending_json)
        .unwrap_or_else(|_| "[]".to_string());

    // Serialize assembly tree as JSON
    let tree_json = assembly_node_to_json(&assembly_tree);
    let assembly_tree_json = serde_json::to_string(&tree_json)
        .unwrap_or_else(|_| "{}".to_string());

    // Create the conversion context (this is the expensive part —
    // builds entity maps, computes bounding box)
    let context = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        OwnedStepConversionContext::new_with_lod_and_profile(step_file, lod, profile)
    }))
    .map_err(|_| JsValue::from_str("Panic during conversion context creation"))?;

    // Store in thread-local state
    CONTEXT.with(|c| {
        *c.borrow_mut() = Some(WorkerState {
            context,
            pending_breps,
        });
    });

    log::info!(
        "[Worker] Parsed '{}': {} entities, {} BREP instances, ~{} faces",
        name,
        entity_count,
        brep_count,
        face_count
    );

    Ok(ParseResult {
        entity_count,
        face_count,
        brep_count,
        shell_count,
        pending_breps_json,
        assembly_tree_json,
    })
}

// ============================================================
// BREP triangulation
// ============================================================

/// Triangulate a single BREP instance by its index in the pending list.
///
/// `brep_index` is the 0-based index into the pending BREP list returned
/// by `parse_step_worker`. After triangulation, the BREP is removed from
/// the pending list.
///
/// Returns `MeshDataResult` with flat arrays for zero-copy transfer,
/// or an error string if triangulation fails.
#[wasm_bindgen]
pub fn triangulate_brep_worker(brep_index: usize) -> Result<JsValue, JsValue> {
    let result = CONTEXT.with(|c| {
        let mut borrow = c.borrow_mut();
        let state = borrow.as_mut().ok_or_else(|| {
            JsValue::from_str("No parse context — call parse_step_worker first")
        })?;

        if brep_index >= state.pending_breps.len() {
            return Err(JsValue::from_str(&format!(
                "brep_index {} out of range (0..{})",
                brep_index,
                state.pending_breps.len()
            )));
        }

        // Remove the BREP from the pending list
        let pending = state.pending_breps.remove(brep_index);

        // Triangulate using the stored context
        let instance = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            state.context.triangulate_pending(&pending)
        }))
        .map_err(|_| {
            JsValue::from_str(&format!(
                "Panic during triangulation of '{}' (BREP #{})",
                pending.name, pending.brep_id
            ))
        })?;

        match instance {
            Some(inst) => {
                let _vcount = inst.mesh.vertex_count();
                let _tcount = inst.mesh.triangle_count();

                if _tcount == 0 {
                    log::warn!(
                        "[Worker] '{}' (BREP #{}): empty mesh — skipping",
                        inst.name,
                        inst.brep_id
                    );
                    Ok(JsValue::from_str(&format!(
                        "{{\"empty\":true,\"name\":\"{}\",\"brep_id\":{}}}",
                        inst.name, inst.brep_id
                    )))
                } else {
                    // Convert to flat MeshData — actual data returned
                    // via triangulate_brep_structured instead
                    let _mesh_data = MeshData::from_mesh(&inst.mesh);
                    let _color = inst.color;
                    Ok(JsValue::NULL) // Placeholder
                }
            }
            None => {
                log::warn!(
                    "[Worker] '{}' (BREP #{}): triangulation failed",
                    pending.name,
                    pending.brep_id
                );
                Ok(JsValue::from_str(&format!(
                    "{{\"error\":true,\"name\":\"{}\",\"brep_id\":{}}}",
                    pending.name, pending.brep_id
                )))
            }
        }
    });

    result
}

/// Triangulate a single BREP instance and return structured mesh data.
///
/// This is the main entry point called from `worker.js`. It:
/// 1. Removes the BREP at `brep_index` from the pending list
/// 2. Triangulates it using the stored `OwnedStepConversionContext`
/// 3. Returns `MeshDataResult` with flat arrays for zero-copy transfer
///
/// On error (no context, out-of-range index, triangulation failure),
/// returns a JS string starting with "error:".
#[wasm_bindgen]
pub fn triangulate_brep_structured(brep_index: usize) -> Result<MeshDataResult, JsValue> {
    CONTEXT.with(|c| {
        let mut borrow = c.borrow_mut();
        let state = borrow.as_mut().ok_or_else(|| {
            JsValue::from_str("error:No parse context — call parse_step_worker first")
        })?;

        if brep_index >= state.pending_breps.len() {
            return Err(JsValue::from_str(&format!(
                "error:brep_index {} out of range (0..{})",
                brep_index,
                state.pending_breps.len()
            )));
        }

        // Remove the BREP from the pending list
        let pending = state.pending_breps.remove(brep_index);
        let brep_id = pending.brep_id;
        let name = pending.name.clone();

        // Triangulate using the stored context
        let instance = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            state.context.triangulate_pending(&pending)
        }))
        .map_err(|_| {
            JsValue::from_str(&format!(
                "error:Panic during triangulation of '{}' (BREP #{})",
                name, brep_id
            ))
        })?;

        match instance {
            Some(inst) => {
                let vcount = inst.mesh.vertex_count();
                let tcount = inst.mesh.triangle_count();

                if tcount == 0 {
                    log::warn!(
                        "[Worker] '{}' (BREP #{}): empty mesh — skipping",
                        inst.name, inst.brep_id
                    );
                    // Return an empty MeshDataResult
                    Ok(MeshDataResult {
                        vertex_count: 0,
                        triangle_count: 0,
                        vertices: Vec::new(),
                        indices: Vec::new(),
                        normals: None,
                        face_normals: None,
                        colors: None,
                    })
                } else {
                    let mesh_data = MeshData::from_mesh(&inst.mesh);
                    log::info!(
                        "[Worker] '{}' (BREP #{}): {} vertices, {} triangles",
                        inst.name, inst.brep_id, vcount, tcount
                    );
                    Ok(MeshDataResult {
                        vertex_count: mesh_data.vertex_count,
                        triangle_count: mesh_data.triangle_count,
                        vertices: mesh_data.vertices,
                        indices: mesh_data.indices,
                        normals: mesh_data.normals,
                        face_normals: mesh_data.face_normals,
                        colors: mesh_data.colors,
                    })
                }
            }
            None => {
                log::warn!(
                    "[Worker] '{}' (BREP #{}): triangulation failed",
                    name, brep_id
                );
                Ok(MeshDataResult {
                    vertex_count: 0,
                    triangle_count: 0,
                    vertices: Vec::new(),
                    indices: Vec::new(),
                    normals: None,
                    face_normals: None,
                    colors: None,
                })
            }
        }
    })
}

// ============================================================
// Context management
// ============================================================

/// Number of remaining pending BREPs in the current context.
#[wasm_bindgen]
pub fn pending_brep_count() -> usize {
    CONTEXT.with(|c| {
        c.borrow()
            .as_ref()
            .map(|s| s.pending_breps.len())
            .unwrap_or(0)
    })
}

/// Get the BREP ID and name for a pending BREP by index.
///
/// Returns a JSON string: `{"name":"...","brep_id":123}` or `"null"`.
#[wasm_bindgen]
pub fn get_pending_brep_info(brep_index: usize) -> String {
    CONTEXT.with(|c| {
        c.borrow()
            .as_ref()
            .and_then(|s| s.pending_breps.get(brep_index))
            .map(|p| {
                serde_json::to_string(&PendingBrepJson {
                    name: p.name.clone(),
                    brep_id: p.brep_id,
                    transform: p.transform,
                    color: p.color,
                })
                .unwrap_or_else(|_| "null".to_string())
            })
            .unwrap_or_else(|| "null".to_string())
    })
}

/// Drop the stored parse context and free WASM memory.
///
/// Call this when triangulation is complete or cancelled.
#[wasm_bindgen]
pub fn drop_parse_context() {
    CONTEXT.with(|c| {
        *c.borrow_mut() = None;
    });
    log::info!("[Worker] Parse context dropped, memory freed");
}

/// Cancel the current chunked triangulation session (if any)
/// and drop the context.
#[wasm_bindgen]
pub fn cancel_triangulation() {
    CONTEXT.with(|c| {
        let mut borrow = c.borrow_mut();
        if let Some(ref mut state) = *borrow {
            state.context.abort_active_session();
        }
        *borrow = None;
    });
    log::info!("[Worker] Triangulation cancelled, context dropped");
}
