// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Web Worker exports for offloading STEP parsing and triangulation.
//!
//! These `wasm-bindgen` functions are called from `worker.js` running in a
//! background Web Worker thread. The worker loads the same WASM binary as
//! the main thread but only invokes these functions — it never touches egui
//! or wgpu, which only exist on the main thread.
//!
//! # State management
//!
//! Worker state is stored in a `thread_local!` so each worker thread gets its
//! own independent instance. This is safe because each Web Worker has exactly
//! one thread.
//!
//! # Protocol
//!
//! 1. Main thread sends `parse` message with STEP file text
//! 2. Worker calls `worker_parse_step()` → stores parsed state, returns stats
//! 3. Main thread sends `triangulate_next` message
//! 4. Worker calls `worker_triangulate_brep()` → returns mesh data
//! 5. Repeat step 3-4 until all BREPs are done
//! 6. Main thread sends `cancel` → worker calls `worker_cancel()`

use wasm_bindgen::prelude::*;
use draper_step::{StepFile, step_structure_lazy, OwnedStepConversionContext, PendingBrepInstance};
use draper_mesh::triangulate::SteinerBudgetProfile;
use draper_mesh::wasm_api::MeshData;

// ─── Worker State ──────────────────────────────────────────────────────

/// Persistent state for the worker thread.
/// Stored in thread_local so each worker gets its own instance.
struct WorkerState {
    /// The parsed STEP file (kept alive for triangulation).
    /// After the conversion context is created, ownership of the StepFile
    /// is transferred into the context, so this becomes an empty placeholder.
    step_file: StepFile,
    /// The conversion context (created on first triangulate call).
    /// Takes ownership of the StepFile.
    ctx: Option<OwnedStepConversionContext>,
    /// Pending BREP instances to triangulate.
    pending_breps: Vec<PendingBrepInstance>,
    /// Assembly tree serialized as JSON (sent to main thread once).
    assembly_tree_json: String,
    /// LOD value for triangulation.
    lod: f64,
    /// Whether this is a mobile device (affects Steiner profile).
    is_mobile: bool,
}

thread_local! {
    static WORKER_STATE: std::cell::RefCell<Option<WorkerState>> = std::cell::RefCell::new(None);
}

// ─── Parse Result (returned to JS) ─────────────────────────────────────

/// Result of `worker_parse_step` — contains file statistics and pending BREP descriptors.
#[wasm_bindgen]
pub struct WorkerParseResult {
    entity_count: usize,
    face_count: usize,
    brep_count: usize,
    shell_count: usize,
    pending_breps_json: String,
    assembly_tree_json: String,
    error: String,
}

#[wasm_bindgen]
impl WorkerParseResult {
    #[wasm_bindgen(getter)]
    pub fn entity_count(&self) -> usize { self.entity_count }

    #[wasm_bindgen(getter)]
    pub fn face_count(&self) -> usize { self.face_count }

    #[wasm_bindgen(getter)]
    pub fn brep_count(&self) -> usize { self.brep_count }

    #[wasm_bindgen(getter)]
    pub fn shell_count(&self) -> usize { self.shell_count }

    #[wasm_bindgen(getter)]
    pub fn pending_breps_json(&self) -> String { self.pending_breps_json.clone() }

    #[wasm_bindgen(getter)]
    pub fn assembly_tree_json(&self) -> String { self.assembly_tree_json.clone() }

    #[wasm_bindgen(getter)]
    pub fn error(&self) -> String { self.error.clone() }
}

/// Manual JSON serialization for pending BREP descriptors.
/// We avoid depending on the `serde` feature of draper-step to keep
/// the dependency tree minimal.
fn serialize_pending_breps_json(pending: &[PendingBrepInstance]) -> String {
    let items: Vec<String> = pending.iter().map(|p| {
        let transform_json = match &p.transform {
            Some(tf) => {
                let vals: Vec<String> = tf.iter()
                    .flat_map(|row| row.iter())
                    .map(|v| format!("{}", v))
                    .collect();
                format!("[{}]", vals.join(","))
            }
            None => "null".to_string(),
        };
        let color_json = match &p.color {
            Some(c) => format!("[{},{},{},{}]", c[0], c[1], c[2], c[3]),
            None => "null".to_string(),
        };
        format!(
            r#"{{"name":"{}","brep_id":{},"transform":{},"color":{}}}"#,
            p.name.replace('"', "\\\"").replace('\\', "\\\\"),
            p.brep_id,
            transform_json,
            color_json
        )
    }).collect();
    format!("[{}]", items.join(","))
}

/// Manual JSON serialization for the assembly tree.
/// Only serializes the fields the main thread needs: name, brep_id,
/// instance_index, and children (recursive).
fn serialize_assembly_tree_json(tree: &draper_step::AssemblyNode) -> String {
    let brep_id_json = match tree.brep_id {
        Some(id) => format!("{}", id),
        None => "null".to_string(),
    };
    let instance_index_json = match tree.instance_index {
        Some(idx) => format!("{}", idx),
        None => "null".to_string(),
    };
    let children_json: Vec<String> = tree.children.iter()
        .map(|c| serialize_assembly_tree_json(c))
        .collect();
    format!(
        r#"{{"name":"{}","brep_id":{},"instance_index":{},"children":[{}]}}"#,
        tree.name.replace('"', "\\\"").replace('\\', "\\\\"),
        brep_id_json,
        instance_index_json,
        children_json.join(",")
    )
}

// ─── Triangulate Result (returned to JS) ───────────────────────────────

/// Result of `worker_triangulate_brep` — contains mesh data for one BREP.
#[wasm_bindgen]
pub struct WorkerTriangulateResult {
    name: String,
    brep_id: i64,
    color: Vec<f32>,
    vertices: Vec<f32>,
    indices: Vec<u32>,
    normals: Vec<f32>,
    face_normals: Vec<f32>,
    colors: Vec<f32>,
    remaining: usize,
    error: String,
}

#[wasm_bindgen]
impl WorkerTriangulateResult {
    #[wasm_bindgen(getter)]
    pub fn name(&self) -> String { self.name.clone() }

    #[wasm_bindgen(getter)]
    pub fn brep_id(&self) -> i64 { self.brep_id }

    #[wasm_bindgen(getter)]
    pub fn color(&self) -> Vec<f32> { self.color.clone() }

    #[wasm_bindgen(getter)]
    pub fn vertices(&self) -> Vec<f32> { self.vertices.clone() }

    #[wasm_bindgen(getter)]
    pub fn indices(&self) -> Vec<u32> { self.indices.clone() }

    #[wasm_bindgen(getter)]
    pub fn normals(&self) -> Vec<f32> { self.normals.clone() }

    #[wasm_bindgen(getter)]
    pub fn face_normals(&self) -> Vec<f32> { self.face_normals.clone() }

    #[wasm_bindgen(getter)]
    pub fn colors(&self) -> Vec<f32> { self.colors.clone() }

    #[wasm_bindgen(getter)]
    pub fn remaining(&self) -> usize { self.remaining }

    #[wasm_bindgen(getter)]
    pub fn error(&self) -> String { self.error.clone() }
}

// ─── Exported Functions ────────────────────────────────────────────────

/// Parse a STEP file in the worker thread.
///
/// This is called from `worker.js` when the main thread sends a `parse` message.
/// The STEP file text is parsed into a `StepFile`, and the assembly tree +
/// pending BREP instances are extracted (but no triangulation happens yet).
///
/// Returns a `WorkerParseResult` with file statistics and pending BREP descriptors.
/// The parsed state is stored in thread-local storage for subsequent
/// `worker_triangulate_brep` calls.
#[wasm_bindgen]
pub fn worker_parse_step(content: &str, name: &str, lod: f64, is_mobile: bool) -> WorkerParseResult {
    let mut result = WorkerParseResult {
        entity_count: 0,
        face_count: 0,
        brep_count: 0,
        shell_count: 0,
        pending_breps_json: String::new(),
        assembly_tree_json: String::new(),
        error: String::new(),
    };

    // Parse STEP file using the draper-step parser
    let step_file = match draper_step::parse_step(content) {
        Ok(sf) => sf,
        Err(e) => {
            result.error = format!("STEP parse error: {}", e);
            return result;
        }
    };

    // Count entities
    let mut face_count = 0;
    let mut shell_count = 0;
    let mut brep_count = 0;
    for entity in &step_file.entities {
        match entity.type_name.as_str() {
            "ADVANCED_FACE" | "FACE_OUTER_BOUND" | "FACE_BOUND" => face_count += 1,
            "CLOSED_SHELL" | "OPEN_SHELL" => shell_count += 1,
            "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => brep_count += 1,
            _ => {}
        }
    }

    result.entity_count = step_file.entities.len();
    result.face_count = face_count;
    result.brep_count = brep_count;
    result.shell_count = shell_count;

    // Extract assembly tree and pending BREP instances
    let (tree, pending) = step_structure_lazy(&step_file);

    // Serialize pending BREP descriptors to JSON (manual, no serde dep)
    result.pending_breps_json = serialize_pending_breps_json(&pending);

    // Serialize assembly tree to JSON
    result.assembly_tree_json = serialize_assembly_tree_json(&tree);

    // Store state in thread-local
    let assembly_tree_json = result.assembly_tree_json.clone();
    WORKER_STATE.with(|ws| {
        *ws.borrow_mut() = Some(WorkerState {
            step_file,
            ctx: None,
            pending_breps: pending,
            assembly_tree_json,
            lod,
            is_mobile,
        });
    });

    log::info!("[Worker] Parsed STEP '{}' — {} entities, {} BREPs, {} faces",
        name, result.entity_count, brep_count, face_count);

    result
}

/// Triangulate the next pending BREP in the worker thread.
///
/// This is called from `worker.js` when the main thread sends a `triangulate_next`
/// message. It processes one BREP at a time, returning the mesh data.
///
/// If no more BREPs are pending, returns a result with empty vertices/indices
/// and `remaining = 0`.
#[wasm_bindgen]
pub fn worker_triangulate_brep() -> WorkerTriangulateResult {
    let mut out = WorkerTriangulateResult {
        name: String::new(),
        brep_id: 0,
        color: Vec::new(),
        vertices: Vec::new(),
        indices: Vec::new(),
        normals: Vec::new(),
        face_normals: Vec::new(),
        colors: Vec::new(),
        remaining: 0,
        error: String::new(),
    };

    WORKER_STATE.with(|ws| {
        let mut ws_ref = ws.borrow_mut();
        let state = match ws_ref.as_mut() {
            Some(s) => s,
            None => {
                out.error = "No STEP file loaded in worker".to_string();
                return;
            }
        };

        if state.pending_breps.is_empty() {
            out.remaining = 0;
            return;
        }

        // Create conversion context on first call (lazy, same as main thread).
        // OwnedStepConversionContext takes ownership of the StepFile, so we
        // replace it with an empty placeholder after transfer.
        if state.ctx.is_none() {
            let profile = if state.is_mobile {
                SteinerBudgetProfile::Mobile
            } else {
                SteinerBudgetProfile::Desktop
            };
            let step_file = std::mem::replace(&mut state.step_file, StepFile::new());
            let lod = state.lod;
            match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                OwnedStepConversionContext::new_with_lod_and_profile(step_file, lod, profile)
            })) {
                Ok(ctx) => {
                    log::info!("[Worker] Conversion context created (LOD={:.2}, profile={})",
                        lod, if state.is_mobile { "Mobile" } else { "Desktop" });
                    state.ctx = Some(ctx);
                }
                Err(_) => {
                    out.error = "Panic during conversion context creation".to_string();
                    state.pending_breps.clear();
                    return;
                }
            }
        }

        // Take the next pending BREP
        let pending = state.pending_breps.remove(0);
        out.name = pending.name.clone();
        out.brep_id = pending.brep_id;
        if let Some(c) = pending.color {
            out.color = c.to_vec();
        }

        // Triangulate this BREP (full, no chunking — worker doesn't need
        // to yield to the browser event loop).
        let ctx = state.ctx.as_mut().unwrap();
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            ctx.triangulate_pending(&pending)
        })) {
            Ok(Some(inst)) => {
                if inst.mesh.triangle_count() > 0 {
                    let mesh_data = MeshData::from_mesh(&inst.mesh);
                    out.vertices = mesh_data.vertices;
                    out.indices = mesh_data.indices;
                    out.normals = mesh_data.normals.unwrap_or_default();
                    out.face_normals = mesh_data.face_normals.unwrap_or_default();
                    out.colors = mesh_data.colors.unwrap_or_default();

                    log::info!("[Worker] Triangulated '{}' (BREP #{}) — {} vertices, {} triangles",
                        out.name, out.brep_id,
                        mesh_data.vertex_count, mesh_data.triangle_count);
                } else {
                    log::warn!("[Worker] '{}' (BREP #{}) produced empty mesh", out.name, out.brep_id);
                }
            }
            Ok(None) => {
                log::warn!("[Worker] '{}' (BREP #{}) — triangulation returned None", out.name, out.brep_id);
            }
            Err(_) => {
                out.error = format!("Panic during triangulation of '{}' (BREP #{})", out.name, out.brep_id);
                ctx.abort_active_session();
                log::error!("[Worker] {}", out.error);
            }
        }

        out.remaining = state.pending_breps.len();
    });

    out
}

/// Cancel all pending work in the worker and release resources.
#[wasm_bindgen]
pub fn worker_cancel() {
    WORKER_STATE.with(|ws| {
        let mut ws_ref = ws.borrow_mut();
        if let Some(state) = ws_ref.as_mut() {
            if let Some(ref mut ctx) = state.ctx {
                ctx.abort_active_session();
            }
            state.pending_breps.clear();
        }
        *ws_ref = None;
    });
    log::info!("[Worker] Cancelled and cleared state");
}

/// Get the assembly tree JSON from the worker state.
/// Called after `worker_parse_step` to retrieve the tree.
#[wasm_bindgen]
pub fn worker_get_assembly_tree() -> String {
    WORKER_STATE.with(|ws| {
        ws.borrow().as_ref()
            .map(|s| s.assembly_tree_json.clone())
            .unwrap_or_default()
    })
}
