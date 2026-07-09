// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Integration between the Rust viewer app and the Web Worker.
//!
//! This module provides the WASM-side interface for the viewer to:
//! 1. Try to create a Web Worker for STEP processing
//! 2. Send STEP file content to the worker
//! 3. Receive triangulated mesh results from the worker
//! 4. Fall back to main-thread chunked processing if the worker is unavailable
//!
//! The JavaScript bridge is loaded from worker-bridge.js.

/// Global worker bridge instance (JS object).
/// Set by the JavaScript initialization code in index.html.
/// If null, the worker is not available and we fall back to main-thread processing.

// ─── Worker Bridge (WASM only) ─────────────────────────────────────────

/// Try to create a Web Worker and return whether it's available.
/// Called once during viewer initialization.
#[cfg(target_arch = "wasm32")]
pub fn try_create_worker() -> bool {
    use wasm_bindgen::prelude::*;
    use wasm_bindgen::JsCast;

    // Call a JavaScript function to create the worker bridge.
    // The function is defined in worker-bridge.js and exposed globally.
    let window = match web_sys::window() {
        Some(w) => w,
        None => return false,
    };

    // Check if the global draperWorkerBridge exists
    let bridge_exists = js_sys::Reflect::has(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(false);
    if !bridge_exists {
        log::info!("[Viewer] Worker bridge not found — using main-thread processing");
        return false;
    }

    // Check if the worker is ready
    let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(JsValue::NULL);
    if bridge.is_null() || bridge.is_undefined() {
        return false;
    }

    let is_ready = js_sys::Reflect::get(&bridge, &JsValue::from_str("isReady")).unwrap_or(JsValue::FALSE);
    if is_ready.as_bool().unwrap_or(false) {
        log::info!("[Viewer] Worker bridge ready — using Web Worker for triangulation");
        return true;
    }

    // Worker is initializing but not ready yet — we'll check again later
    log::info!("[Viewer] Worker bridge found but not ready yet — will try on next load");
    false
}

/// Send a STEP file to the worker for parsing and triangulation.
/// The worker will post results back to the main thread via the bridge.
#[cfg(target_arch = "wasm32")]
pub fn worker_send_step(content: &str, name: &str, lod: f64, is_mobile: bool) -> bool {
    use wasm_bindgen::prelude::*;
    use wasm_bindgen::JsCast;

    let window = match web_sys::window() {
        Some(w) => w,
        None => return false,
    };

    let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(JsValue::NULL);
    if bridge.is_null() || bridge.is_undefined() {
        return false;
    }

    // Call bridge.parseStep(content, name, lod, is_mobile)
    let parse_fn = js_sys::Reflect::get(&bridge, &JsValue::from_str("parseStep")).unwrap_or(JsValue::UNDEFINED);
    if parse_fn.is_undefined() {
        return false;
    }

    let parse_fn: js_sys::Function = match parse_fn.dyn_into() {
        Ok(f) => f,
        Err(_) => return false,
    };

    let result = parse_fn.call3(
        &bridge,
        &JsValue::from_str(content),
        &JsValue::from_str(name),
        &JsValue::from_f64(lod),
    );

    match result {
        Ok(_) => {
            log::info!("[Viewer] Sent STEP file '{}' to worker (LOD={:.2}, mobile={})", name, lod, is_mobile);
            true
        }
        Err(e) => {
            log::error!("[Viewer] Failed to send STEP to worker: {:?}", e);
            false
        }
    }
}

/// Request triangulation of the next BREP from the worker.
#[cfg(target_arch = "wasm32")]
pub fn worker_request_next() -> bool {
    use wasm_bindgen::prelude::*;

    let window = match web_sys::window() {
        Some(w) => w,
        None => return false,
    };

    let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(JsValue::NULL);
    if bridge.is_null() {
        return false;
    }

    let fn_val = js_sys::Reflect::get(&bridge, &JsValue::from_str("triangulateNext")).unwrap_or(JsValue::UNDEFINED);
    if fn_val.is_undefined() {
        return false;
    }

    let fn_val: js_sys::Function = match fn_val.dyn_into() {
        Ok(f) => f,
        Err(_) => return false,
    };

    fn_val.call0(&bridge).is_ok()
}

/// Cancel the worker's current processing.
#[cfg(target_arch = "wasm32")]
pub fn worker_cancel() {
    use wasm_bindgen::prelude::*;

    let window = match web_sys::window() {
        Some(w) => w,
        None => return,
    };

    let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(JsValue::NULL);
    if bridge.is_null() {
        return;
    }

    let fn_val = js_sys::Reflect::get(&bridge, &JsValue::from_str("cancel")).unwrap_or(JsValue::UNDEFINED);
    if fn_val.is_undefined() {
        return;
    }

    if let Ok(fn_val) = fn_val.dyn_into::<js_sys::Function>() {
        let _ = fn_val.call0(&bridge);
    }
}

/// Check if the worker is still processing (has BREPs in flight).
#[cfg(target_arch = "wasm32")]
pub fn worker_is_processing() -> bool {
    use wasm_bindgen::prelude::*;

    let window = match web_sys::window() {
        Some(w) => w,
        None => return false,
    };

    let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(JsValue::NULL);
    if bridge.is_null() {
        return false;
    }

    let processing = js_sys::Reflect::get(&bridge, &JsValue::from_str("isProcessing")).unwrap_or(JsValue::FALSE);
    processing.as_bool().unwrap_or(false)
}

/// Check if there are pending mesh results from the worker.
/// Returns the number of pending results.
#[cfg(target_arch = "wasm32")]
pub fn worker_pending_count() -> usize {
    use wasm_bindgen::prelude::*;

    let window = match web_sys::window() {
        Some(w) => w,
        None => return 0,
    };

    let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(JsValue::NULL);
    if bridge.is_null() {
        return 0;
    }

    let count = js_sys::Reflect::get(&bridge, &JsValue::from_str("pendingMeshCount")).unwrap_or(JsValue::from_f64(0.0));
    count.as_f64().unwrap_or(0.0) as usize
}

/// Drain pending mesh results from the worker bridge.
/// Returns a Vec of WorkerMeshResult.
#[cfg(target_arch = "wasm32")]
pub fn worker_drain_results() -> Vec<crate::app::WorkerMeshResult> {
    use wasm_bindgen::prelude::*;

    let window = match web_sys::window() {
        Some(w) => w,
        None => return Vec::new(),
    };

    let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge")).unwrap_or(JsValue::NULL);
    if bridge.is_null() {
        return Vec::new();
    }

    let fn_val = js_sys::Reflect::get(&bridge, &JsValue::from_str("drainResults")).unwrap_or(JsValue::UNDEFINED);
    if fn_val.is_undefined() {
        return Vec::new();
    }

    let fn_val: js_sys::Function = match fn_val.dyn_into() {
        Ok(f) => f,
        Err(_) => return Vec::new(),
    };

    let result = match fn_val.call0(&bridge) {
        Ok(r) => r,
        Err(_) => return Vec::new(),
    };

    // The JS function returns an array of mesh result objects
    let arr: js_sys::Array = match result.dyn_into() {
        Ok(a) => a,
        Err(_) => return Vec::new(),
    };

    let mut results = Vec::new();
    for i in 0..arr.length() {
        let item = arr.get(i);
        if let Some(wmr) = js_to_worker_mesh_result(&item) {
            results.push(wmr);
        }
    }
    results
}

/// Convert a JS mesh result object to a WorkerMeshResult.
#[cfg(target_arch = "wasm32")]
fn js_to_worker_mesh_result(obj: &wasm_bindgen::JsValue) -> Option<crate::app::WorkerMeshResult> {
    use wasm_bindgen::JsCast;

    // Extract fields from the JS object
    let name = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("name"))
        .ok().and_then(|v| v.as_string()).unwrap_or_default();

    let brep_id = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("brep_id"))
        .ok().and_then(|v| v.as_f64()).unwrap_or(0.0) as usize;

    let color = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("color"))
        .ok().and_then(|v| {
            if v.is_null() || v.is_undefined() { return None; }
            let arr: js_sys::Array = v.dyn_into().ok()?;
            if arr.length() >= 4 {
                Some([
                    arr.get(0).as_f64()? as f32,
                    arr.get(1).as_f64()? as f32,
                    arr.get(2).as_f64()? as f32,
                    arr.get(3).as_f64()? as f32,
                ])
            } else { None }
        });

    let vertices = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("vertices"))
        .ok().and_then(|v| {
            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
            Some(arr.to_vec())
        }).unwrap_or_default();

    let indices = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("indices"))
        .ok().and_then(|v| {
            let arr: js_sys::Uint32Array = v.dyn_into().ok()?;
            Some(arr.to_vec())
        }).unwrap_or_default();

    let normals = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("normals"))
        .ok().and_then(|v| {
            if v.is_null() || v.is_undefined() { return None; }
            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
            let vec = arr.to_vec();
            if vec.is_empty() { None } else { Some(vec) }
        });

    let face_normals = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("face_normals"))
        .ok().and_then(|v| {
            if v.is_null() || v.is_undefined() { return None; }
            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
            let vec = arr.to_vec();
            if vec.is_empty() { None } else { Some(vec) }
        });

    let colors = js_sys::Reflect::get(obj, &wasm_bindgen::JsValue::from_str("colors"))
        .ok().and_then(|v| {
            if v.is_null() || v.is_undefined() { return None; }
            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
            let vec = arr.to_vec();
            if vec.is_empty() { None } else { Some(vec) }
        });

    Some(crate::app::WorkerMeshResult {
        name,
        brep_id,
        color,
        vertices,
        indices,
        normals,
        face_normals,
        colors,
    })
}
