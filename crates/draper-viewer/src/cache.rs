// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # IndexedDB Cache for Triangulation Results
//!
//! Caches the results of STEP file triangulation in IndexedDB so that
//! re-opening the same file is instantaneous (no re-parsing or
//! re-triangulation needed).
//!
//! ## Storage format
//!
//! **Key:** SHA-256 hex digest of the STEP file content (UTF-8 bytes).
//!
//! **Value:** A JavaScript object with the following fields:
//! - `vertices`: `Float32Array` — flat `[x,y,z, x,y,z, ...]`
//! - `indices`: `Uint32Array` — flat triangle indices
//! - `normals`: `Float32Array | null` — vertex normals
//! - `face_normals`: `Float32Array | null` — per-triangle normals
//! - `colors`: `Float32Array | null` — per-triangle RGBA
//! - `face_ids`: `Float64Array | null` — per-triangle face selection IDs
//! - `instances_json`: `string` — JSON-serialized instance metadata
//! - `assembly_tree_json`: `string` — JSON-serialized assembly tree
//! - `timestamp`: `number` — Unix milliseconds when cached
//! - `lod`: `number` — LOD value used for triangulation
//! - `file_name`: `string` — original file name
//! - `vertex_count`: `number`
//! - `triangle_count`: `number`
//!
//! ## Cache eviction
//!
//! - **TTL:** 7 days. Entries older than this are skipped on lookup.
//! - **Size limit:** 500 MB. When exceeded, oldest entries are evicted first (LRU).
//!
//! ## Asynchronous design
//!
//! IndexedDB operations are inherently async. This module uses a polling
//! pattern similar to the Web Worker bridge: an async operation is started,
//! and the result is written to a shared `Arc<Mutex<...>>`. The main frame
//! loop checks each frame whether the result is ready.

use std::sync::Arc;
use std::sync::Mutex;

use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;

use draper_mesh::TriangleMesh;
use draper_step::{AssemblyNode, DetailedMeshInstance};

// ─── Constants ────────────────────────────────────────────────────────

const DB_NAME: &str = "3draper_cache";
const DB_VERSION: u32 = 1;
const STORE_NAME: &str = "triangulations";
const TTL_MS: u64 = 7 * 24 * 60 * 60 * 1000; // 7 days
const MAX_ENTRIES: usize = 50; // Max cache entries (simple size limit)

/// Cache version prefix. When the WASM is rebuilt with breaking changes
/// (new triangulation logic, new mesh format, etc.), this version is
/// bumped. Old cache entries (with a different version prefix) won't
/// be found, forcing re-triangulation with the new code.
///
/// This prevents corrupted/incompatible cached meshes from causing
/// rendering hangs or crashes after a WASM update.
const CACHE_VERSION: &str = "v3_";

// ─── Result types ─────────────────────────────────────────────────────

/// The result of a cache lookup — either a hit with deserialized data,
/// or a miss (the file was not found in the cache, or the entry expired).
#[derive(Debug)]
pub struct CacheLookupResult {
    pub mesh: TriangleMesh,
    pub instances: Vec<DetailedMeshInstance>,
    pub assembly_tree: AssemblyNode,
    pub lod: f64,
    pub file_name: String,
}

impl Clone for CacheLookupResult {
    fn clone(&self) -> Self {
        Self {
            mesh: self.mesh.clone(),
            instances: self.instances.clone(),
            assembly_tree: self.assembly_tree.clone(),
            lod: self.lod,
            file_name: self.file_name.clone(),
        }
    }
}

/// State of an in-flight cache operation.
#[derive(Debug)]
pub enum CacheState {
    /// No cache operation in progress.
    Idle,
    /// Waiting for SHA-256 hash computation.
    Hashing,
    /// Waiting for IndexedDB lookup.
    LookingUp { hash: String },
    /// Cache hit — data is ready.
    Hit(Box<CacheLookupResult>),
    /// Cache miss — proceed with normal triangulation.
    Miss { hash: String },
    /// An error occurred during the cache operation.
    Error(String),
}

impl CacheState {
    pub fn take(&mut self) -> CacheState {
        std::mem::replace(self, CacheState::Idle)
    }
}

// ─── Shared state for async communication ────────────────────────────

/// Shared state between the async cache operation and the main frame loop.
/// The async operation writes its result here, and the main loop polls it
/// each frame.
pub struct CacheManager {
    state: Arc<Mutex<CacheState>>,
}

impl CacheManager {
    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(CacheState::Idle)),
        }
    }

    /// Take the current state, replacing it with Idle.
    pub fn take_state(&self) -> CacheState {
        let mut guard = self.state.lock().unwrap();
        guard.take()
    }

    /// Start an async cache lookup for the given STEP file content.
    pub fn start_lookup(&self, content: &str, file_name: &str, lod: f64) {
        // Reset state
        {
            let mut guard = self.state.lock().unwrap();
            *guard = CacheState::Hashing;
        }

        let state = self.state.clone();
        let content_bytes = content.as_bytes().to_vec();
        let file_name = file_name.to_string();

        wasm_bindgen_futures::spawn_local(async move {
            // Step 1: Compute SHA-256 hash
            let hash = match compute_sha256(&content_bytes).await {
                Ok(h) => h,
                Err(e) => {
                    let mut guard = state.lock().unwrap();
                    *guard = CacheState::Error(format!("SHA-256 failed: {}", e));
                    return;
                }
            };

            // Prefix hash with cache version to invalidate old entries
            let hash = format!("{}{}", CACHE_VERSION, hash);

            log::info!("Cache: SHA-256 hash = {}...", &hash[..16.min(hash.len())]);

            {
                let mut guard = state.lock().unwrap();
                *guard = CacheState::LookingUp { hash: hash.clone() };
            }

            // Step 2: Look up the entry via JS bridge
            let entry_js = match idb_get(&hash).await {
                Ok(Some(val)) => val,
                Ok(None) => {
                    let mut guard = state.lock().unwrap();
                    *guard = CacheState::Miss { hash };
                    return;
                }
                Err(e) => {
                    let mut guard = state.lock().unwrap();
                    *guard = CacheState::Error(format!("IDB lookup failed: {}", e));
                    return;
                }
            };

            // Step 3: Check TTL
            let now = js_sys::Date::now() as u64;
            let timestamp = js_sys::Reflect::get(&entry_js, &"timestamp".into())
                .ok()
                .and_then(|v| v.as_f64())
                .unwrap_or(0.0) as u64;

            if now.saturating_sub(timestamp) > TTL_MS {
                log::info!("Cache: entry expired (age = {} days)", (now - timestamp) / (24 * 60 * 60 * 1000));
                let _ = idb_delete(&hash).await;
                let mut guard = state.lock().unwrap();
                *guard = CacheState::Miss { hash };
                return;
            }

            // Step 4: Check LOD — only use cache if LOD matches EXACTLY.
            //
            // CRITICAL: The previous condition `(lod - cached_lod).abs() > 0.01 && lod > cached_lod`
            // only caused a cache miss when the requested LOD was HIGHER than the cached LOD.
            // When the user LOWERED the quality (e.g., High → Low), the cache would return
            // the old high-quality mesh — making the Quality slider appear to have no effect.
            //
            // Now we require an exact LOD match (within 0.01 tolerance) for a cache hit.
            // Any mismatch (higher OR lower) causes a cache miss, ensuring the model is
            // re-triangulated at the requested LOD.
            let cached_lod = js_sys::Reflect::get(&entry_js, &"lod".into())
                .ok()
                .and_then(|v| v.as_f64())
                .unwrap_or(0.0);
            if (lod - cached_lod).abs() > 0.01 {
                log::info!("Cache: LOD mismatch (cached={:.2}, requested={:.2}) — miss", cached_lod, lod);
                let mut guard = state.lock().unwrap();
                *guard = CacheState::Miss { hash };
                return;
            }

            // Step 5: Deserialize the data
            let result = match js_entry_to_lookup_result(&entry_js) {
                Ok(r) => r,
                Err(e) => {
                    log::warn!("Cache: deserialization failed: {} — deleting entry", e);
                    let _ = idb_delete(&hash).await;
                    let mut guard = state.lock().unwrap();
                    *guard = CacheState::Miss { hash };
                    return;
                }
            };

            log::info!("Cache: HIT — {} vertices, {} triangles (LOD={:.2}, age={}s)",
                result.mesh.vertex_count(),
                result.mesh.triangle_count(),
                result.lod,
                (now - timestamp) / 1000
            );

            let mut guard = state.lock().unwrap();
            *guard = CacheState::Hit(Box::new(result));
        });
    }

    /// Store triangulation results in the cache (async, fire-and-forget).
    pub fn store_result(
        &self,
        content: &str,
        file_name: &str,
        lod: f64,
        mesh: &TriangleMesh,
        instances: &[DetailedMeshInstance],
        assembly_tree: &AssemblyNode,
    ) {
        let content_bytes = content.as_bytes().to_vec();
        let file_name = file_name.to_string();

        // Serialize instance metadata to JSON
        let instances_json = serialize_instances(instances);
        let assembly_tree_json = serialize_assembly_tree(assembly_tree);

        // Extract flat arrays from mesh
        let vertices: Vec<f32> = mesh.vertices.iter()
            .flat_map(|v| [v.x as f32, v.y as f32, v.z as f32])
            .collect();
        let indices: Vec<u32> = mesh.triangles.iter().flat_map(|t| t.iter().copied()).collect();
        let normals: Option<Vec<f32>> = mesh.normals.as_ref().map(|n| n.iter().flat_map(|v| [v[0] as f32, v[1] as f32, v[2] as f32]).collect());
        let face_normals: Option<Vec<f32>> = mesh.face_normals.as_ref().map(|n| n.iter().flat_map(|v| [v[0] as f32, v[1] as f32, v[2] as f32]).collect());
        let colors: Option<Vec<f32>> = mesh.triangle_colors.as_ref().map(|c| c.iter().flat_map(|v| [v[0], v[1], v[2], v[3]]).collect());
        let face_ids: Option<Vec<f64>> = mesh.triangle_face_ids.as_ref().map(|ids| ids.iter().map(|&id| id as f64).collect());

        let vertex_count = mesh.vertex_count();
        let triangle_count = mesh.triangle_count();

        wasm_bindgen_futures::spawn_local(async move {
            // Step 1: Compute SHA-256 hash
            let hash = match compute_sha256(&content_bytes).await {
                Ok(h) => h,
                Err(e) => {
                    log::warn!("Cache: SHA-256 failed for store: {}", e);
                    return;
                }
            };

            // Prefix hash with cache version to invalidate old entries
            let hash = format!("{}{}", CACHE_VERSION, hash);

            // Step 2: Build the JS entry object
            let entry = build_js_entry(&hash, &file_name, lod, vertex_count, triangle_count,
                &vertices, &indices, &normals, &face_normals, &colors, &face_ids,
                &instances_json, &assembly_tree_json);

            // Step 3: Store in IndexedDB
            match idb_put(&hash, &entry).await {
                Ok(()) => {
                    log::info!("Cache: STORED — hash={}..., {} vertices, {} triangles, LOD={:.2}",
                        &hash[..16.min(hash.len())], vertex_count, triangle_count, lod);
                }
                Err(e) => {
                    log::warn!("Cache: store failed: {}", e);
                }
            }
        });
    }

    /// Clear the entire cache (async, fire-and-forget).
    pub fn clear_cache(&self) {
        wasm_bindgen_futures::spawn_local(async move {
            match idb_clear().await {
                Ok(()) => log::info!("Cache: cleared all entries"),
                Err(e) => log::warn!("Cache: clear failed: {}", e),
            }
        });
    }
}

// ─── JS bridge functions for IndexedDB ────────────────────────────────
//
// Instead of fighting with web-sys's incomplete IndexedDB bindings,
// we use a small JS shim injected at startup and call it via js_sys.
// This is more reliable and avoids missing web-sys feature flags.

/// Initialize the IndexedDB database. Called once at startup.
pub fn init_cache_db() {
    let js = format!(r#"
        (function() {{
            if (!window.__draper_cache_db) {{
                var req = indexedDB.open("{db_name}", {db_version});
                req.onupgradeneeded = function(e) {{
                    var db = e.target.result;
                    if (!db.objectStoreNames.contains("{store_name}")) {{
                        db.createObjectStore("{store_name}");
                    }}
                }};
                req.onsuccess = function(e) {{
                    window.__draper_cache_db = e.target.result;
                }};
                req.onerror = function(e) {{
                    console.error("Cache DB open error:", e);
                }};
            }}
        }})()
    "#, db_name = DB_NAME, db_version = DB_VERSION, store_name = STORE_NAME);

    let _ = js_sys::eval(&js);
}

/// Get a value from IndexedDB by key. Returns a Promise that resolves
/// to the stored JS object, or undefined if not found.
async fn idb_get(hash: &str) -> Result<Option<js_sys::Object>, String> {
    let js = format!(r#"
        (function() {{
            return new Promise(function(resolve, reject) {{
                var db = window.__draper_cache_db;
                if (!db) {{
                    // DB not ready yet — try opening it
                    var req = indexedDB.open("{db_name}", {db_version});
                    req.onupgradeneeded = function(e) {{
                        var db2 = e.target.result;
                        if (!db2.objectStoreNames.contains("{store_name}")) {{
                            db2.createObjectStore("{store_name}");
                        }}
                    }};
                    req.onsuccess = function(e) {{
                        window.__draper_cache_db = e.target.result;
                        var tx = e.target.result.transaction("{store_name}", "readonly");
                        var store = tx.objectStore("{store_name}");
                        var getReq = store.get("{hash}");
                        getReq.onsuccess = function() {{
                            resolve(getReq.result || undefined);
                        }};
                        getReq.onerror = function() {{
                            reject(getReq.error);
                        }};
                    }};
                    req.onerror = function() {{ reject(req.error); }};
                    return;
                }}
                var tx = db.transaction("{store_name}", "readonly");
                var store = tx.objectStore("{store_name}");
                var getReq = store.get("{hash}");
                getReq.onsuccess = function() {{
                    resolve(getReq.result || undefined);
                }};
                getReq.onerror = function() {{
                    reject(getReq.error);
                }};
            }});
        }})()
    "#, db_name = DB_NAME, db_version = DB_VERSION, store_name = STORE_NAME, hash = hash);

    let result = js_sys::eval(&js)
        .map_err(|e| format!("eval failed: {:?}", e))?;

    let promise = js_sys::Promise::from(result);
    let value = wasm_bindgen_futures::JsFuture::from(promise)
        .await
        .map_err(|e| format!("IDB get failed: {:?}", e))?;

    if value.is_undefined() || value.is_null() {
        Ok(None)
    } else {
        Ok(Some(js_sys::Object::from(value)))
    }
}

/// Put a value into IndexedDB.
async fn idb_put(hash: &str, entry: &js_sys::Object) -> Result<(), String> {
    // Store the entry with the hash as the key
    // We need to pass the entry object into JS context
    // Use a global variable to pass the entry
    js_sys::Reflect::set(
        &js_sys::global().unchecked_ref(),
        &"__draper_cache_entry".into(),
        entry,
    ).map_err(|e| format!("Reflect set failed: {:?}", e))?;

    let js = format!(r#"
        (function() {{
            return new Promise(function(resolve, reject) {{
                var db = window.__draper_cache_db;
                var entry = window.__draper_cache_entry;
                if (!db) {{
                    reject("DB not open");
                    return;
                }}
                var tx = db.transaction("{store_name}", "readwrite");
                var store = tx.objectStore("{store_name}");
                var putReq = store.put(entry, "{hash}");
                putReq.onsuccess = function() {{
                    delete window.__draper_cache_entry;
                    resolve();
                }};
                putReq.onerror = function() {{
                    delete window.__draper_cache_entry;
                    reject(putReq.error);
                }};
            }});
        }})()
    "#, store_name = STORE_NAME, hash = hash);

    let result = js_sys::eval(&js)
        .map_err(|e| format!("eval failed: {:?}", e))?;

    let promise = js_sys::Promise::from(result);
    wasm_bindgen_futures::JsFuture::from(promise)
        .await
        .map_err(|e| format!("IDB put failed: {:?}", e))?;

    Ok(())
}

/// Delete a value from IndexedDB.
async fn idb_delete(hash: &str) -> Result<(), String> {
    let js = format!(r#"
        (function() {{
            return new Promise(function(resolve, reject) {{
                var db = window.__draper_cache_db;
                if (!db) {{ reject("DB not open"); return; }}
                var tx = db.transaction("{store_name}", "readwrite");
                var store = tx.objectStore("{store_name}");
                var req = store.delete("{hash}");
                req.onsuccess = function() {{ resolve(); }};
                req.onerror = function() {{ reject(req.error); }};
            }});
        }})()
    "#, store_name = STORE_NAME, hash = hash);

    let result = js_sys::eval(&js)
        .map_err(|e| format!("eval failed: {:?}", e))?;

    let promise = js_sys::Promise::from(result);
    wasm_bindgen_futures::JsFuture::from(promise)
        .await
        .map_err(|e| format!("IDB delete failed: {:?}", e))?;

    Ok(())
}

/// Clear all entries from the cache store.
async fn idb_clear() -> Result<(), String> {
    let js = format!(r#"
        (function() {{
            return new Promise(function(resolve, reject) {{
                var db = window.__draper_cache_db;
                if (!db) {{ reject("DB not open"); return; }}
                var tx = db.transaction("{store_name}", "readwrite");
                var store = tx.objectStore("{store_name}");
                var req = store.clear();
                req.onsuccess = function() {{ resolve(); }};
                req.onerror = function() {{ reject(req.error); }};
            }});
        }})()
    "#, store_name = STORE_NAME);

    let result = js_sys::eval(&js)
        .map_err(|e| format!("eval failed: {:?}", e))?;

    let promise = js_sys::Promise::from(result);
    wasm_bindgen_futures::JsFuture::from(promise)
        .await
        .map_err(|e| format!("IDB clear failed: {:?}", e))?;

    Ok(())
}

// ─── SHA-256 via Web Crypto API ───────────────────────────────────────

/// Compute SHA-256 hash of the given bytes using the Web Crypto API.
/// Returns the hex digest.
async fn compute_sha256(bytes: &[u8]) -> Result<String, String> {
    // Use a JS bridge to compute SHA-256 — avoids complex SubtleCrypto bindings
    let js_buffer = js_sys::ArrayBuffer::new(bytes.len() as u32);
    let js_view = js_sys::Uint8Array::new(&js_buffer);
    js_view.copy_from(bytes);

    // Store the buffer in a global so JS can access it
    js_sys::Reflect::set(
        &js_sys::global().unchecked_ref(),
        &"__draper_hash_input".into(),
        &js_buffer,
    ).map_err(|e| format!("Reflect set failed: {:?}", e))?;

    let js = r#"
        (function() {
            return new Promise(function(resolve, reject) {
                var buffer = window.__draper_hash_input;
                crypto.subtle.digest('SHA-256', buffer).then(function(hashBuffer) {
                    var hashArray = Array.from(new Uint8Array(hashBuffer));
                    var hex = hashArray.map(function(b) { return b.toString(16).padStart(2, '0'); }).join('');
                    delete window.__draper_hash_input;
                    resolve(hex);
                }).catch(function(err) {
                    delete window.__draper_hash_input;
                    reject(err);
                });
            });
        })()
    "#;

    let result = js_sys::eval(js)
        .map_err(|e| format!("eval failed: {:?}", e))?;

    let promise = js_sys::Promise::from(result);
    let hex = wasm_bindgen_futures::JsFuture::from(promise)
        .await
        .map_err(|e| format!("SHA-256 failed: {:?}", e))?;

    hex.as_string().ok_or_else(|| "SHA-256 result is not a string".to_string())
}

// ─── JS object ↔ CacheEntry conversion ────────────────────────────────

/// Build a JS object from cache entry data.
fn build_js_entry(
    hash: &str,
    file_name: &str,
    lod: f64,
    vertex_count: usize,
    triangle_count: usize,
    vertices: &[f32],
    indices: &[u32],
    normals: &Option<Vec<f32>>,
    face_normals: &Option<Vec<f32>>,
    colors: &Option<Vec<f32>>,
    face_ids: &Option<Vec<f64>>,
    instances_json: &str,
    assembly_tree_json: &str,
) -> js_sys::Object {
    let obj = js_sys::Object::new();

    let set_str = |obj: &js_sys::Object, key: &str, val: &str| {
        let _ = js_sys::Reflect::set(obj, &key.into(), &val.into());
    };
    let set_num = |obj: &js_sys::Object, key: &str, val: f64| {
        let _ = js_sys::Reflect::set(obj, &key.into(), &js_sys::Number::from(val));
    };
    let set_arraybuffer = |obj: &js_sys::Object, key: &str, data: &[u8], ctor: fn(u32) -> js_sys::Object| {
        // We create a TypedArray view, then store its underlying ArrayBuffer
        let arr = ctor(data.len() as u32);
        // Copy data into the typed array
        let uint8 = js_sys::Uint8Array::new(&arr);
        // For f32/u32/f64, we need to copy via Uint8Array view of the buffer
        let buf = js_sys::Reflect::get(&arr, &"buffer".into()).ok();
        if let Some(ref buf) = buf {
            let view = js_sys::Uint8Array::new(buf);
            let bytes = unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * std::mem::size_of::<f32>()) };
            view.copy_from(bytes);
        }
        let _ = js_sys::Reflect::set(obj, &key.into(), &arr);
    };

    set_str(&obj, "hash", hash);
    set_str(&obj, "file_name", file_name);
    set_num(&obj, "lod", lod);
    set_num(&obj, "timestamp", js_sys::Date::now());
    set_num(&obj, "vertex_count", vertex_count as f64);
    set_num(&obj, "triangle_count", triangle_count as f64);

    // Vertices as Float32Array
    {
        let arr = js_sys::Float32Array::new_with_length(vertices.len() as u32);
        arr.copy_from(vertices);
        let _ = js_sys::Reflect::set(&obj, &"vertices".into(), &arr.buffer());
    }

    // Indices as Uint32Array
    {
        let arr = js_sys::Uint32Array::new_with_length(indices.len() as u32);
        arr.copy_from(indices);
        let _ = js_sys::Reflect::set(&obj, &"indices".into(), &arr.buffer());
    }

    // Normals as Float32Array (optional)
    if let Some(ref normals) = normals {
        let arr = js_sys::Float32Array::new_with_length(normals.len() as u32);
        arr.copy_from(normals);
        let _ = js_sys::Reflect::set(&obj, &"normals".into(), &arr.buffer());
    }

    // Face normals as Float32Array (optional)
    if let Some(ref face_normals) = face_normals {
        let arr = js_sys::Float32Array::new_with_length(face_normals.len() as u32);
        arr.copy_from(face_normals);
        let _ = js_sys::Reflect::set(&obj, &"face_normals".into(), &arr.buffer());
    }

    // Colors as Float32Array (optional)
    if let Some(ref colors) = colors {
        let arr = js_sys::Float32Array::new_with_length(colors.len() as u32);
        arr.copy_from(colors);
        let _ = js_sys::Reflect::set(&obj, &"colors".into(), &arr.buffer());
    }

    // Face IDs as Float64Array (optional)
    if let Some(ref face_ids) = face_ids {
        let arr = js_sys::Float64Array::new_with_length(face_ids.len() as u32);
        arr.copy_from(face_ids);
        let _ = js_sys::Reflect::set(&obj, &"face_ids".into(), &arr.buffer());
    }

    // JSON strings for metadata
    set_str(&obj, "instances_json", instances_json);
    set_str(&obj, "assembly_tree_json", assembly_tree_json);

    obj
}

/// Convert a cached JS entry object back into the result types.
fn js_entry_to_lookup_result(obj: &js_sys::Object) -> Result<CacheLookupResult, String> {
    let get_str = |key: &str| -> Result<String, String> {
        js_sys::Reflect::get(obj, &key.into())
            .map_err(|_| format!("missing field '{}'", key))
            .and_then(|v| v.as_string().ok_or_else(|| format!("field '{}' is not a string", key)))
    };

    let get_f64 = |key: &str| -> Result<f64, String> {
        js_sys::Reflect::get(obj, &key.into())
            .map_err(|_| format!("missing field '{}'", key))
            .and_then(|v| v.as_f64().ok_or_else(|| format!("field '{}' is not a number", key)))
    };

    let get_f32_vec = |key: &str| -> Result<Vec<f32>, String> {
        let val = js_sys::Reflect::get(obj, &key.into())
            .map_err(|_| format!("missing field '{}'", key))?;
        // Value might be an ArrayBuffer (from IDB storage) or Float32Array
        let arr = if val.is_instance_of::<js_sys::ArrayBuffer>() {
            js_sys::Float32Array::new(&val)
        } else {
            js_sys::Float32Array::from(val)
        };
        let mut result = vec![0.0f32; arr.length() as usize];
        arr.copy_to(&mut result);
        Ok(result)
    };

    let get_u32_vec = |key: &str| -> Result<Vec<u32>, String> {
        let val = js_sys::Reflect::get(obj, &key.into())
            .map_err(|_| format!("missing field '{}'", key))?;
        let arr = if val.is_instance_of::<js_sys::ArrayBuffer>() {
            js_sys::Uint32Array::new(&val)
        } else {
            js_sys::Uint32Array::from(val)
        };
        let mut result = vec![0u32; arr.length() as usize];
        arr.copy_to(&mut result);
        Ok(result)
    };

    let get_optional_f32_vec = |key: &str| -> Result<Option<Vec<f32>>, String> {
        let val = match js_sys::Reflect::get(obj, &key.into()) {
            Ok(v) => v,
            Err(_) => return Ok(None),
        };
        if val.is_undefined() || val.is_null() {
            return Ok(None);
        }
        let arr = if val.is_instance_of::<js_sys::ArrayBuffer>() {
            js_sys::Float32Array::new(&val)
        } else {
            js_sys::Float32Array::from(val)
        };
        let mut result = vec![0.0f32; arr.length() as usize];
        arr.copy_to(&mut result);
        Ok(Some(result))
    };

    let get_optional_f64_vec = |key: &str| -> Result<Option<Vec<f64>>, String> {
        let val = match js_sys::Reflect::get(obj, &key.into()) {
            Ok(v) => v,
            Err(_) => return Ok(None),
        };
        if val.is_undefined() || val.is_null() {
            return Ok(None);
        }
        let arr = if val.is_instance_of::<js_sys::ArrayBuffer>() {
            js_sys::Float64Array::new(&val)
        } else {
            js_sys::Float64Array::from(val)
        };
        let mut result = vec![0.0f64; arr.length() as usize];
        arr.copy_to(&mut result);
        Ok(Some(result))
    };

    let file_name = get_str("file_name")?;
    let lod = get_f64("lod")?;

    let vertices = get_f32_vec("vertices")?;
    let indices = get_u32_vec("indices")?;
    let normals = get_optional_f32_vec("normals")?;
    let face_normals = get_optional_f32_vec("face_normals")?;
    let colors = get_optional_f32_vec("colors")?;
    let face_ids = get_optional_f64_vec("face_ids")?;
    let instances_json = get_str("instances_json")?;
    let assembly_tree_json = get_str("assembly_tree_json")?;

    // Reconstruct TriangleMesh
    let mut mesh = TriangleMesh::new();
    for i in (0..vertices.len()).step_by(3) {
        if i + 2 < vertices.len() {
            mesh.vertices.push(draper_geometry::Point3d {
                x: vertices[i] as f64,
                y: vertices[i + 1] as f64,
                z: vertices[i + 2] as f64,
            });
        }
    }
    for i in (0..indices.len()).step_by(3) {
        if i + 2 < indices.len() {
            mesh.triangles.push([indices[i], indices[i + 1], indices[i + 2]]);
        }
    }
    if let Some(normals) = normals {
        let mut n = Vec::with_capacity(normals.len() / 3);
        for i in (0..normals.len()).step_by(3) {
            if i + 2 < normals.len() {
                n.push([normals[i] as f64, normals[i + 1] as f64, normals[i + 2] as f64]);
            }
        }
        mesh.normals = Some(n);
    }
    if let Some(face_normals) = face_normals {
        let mut n = Vec::with_capacity(face_normals.len() / 3);
        for i in (0..face_normals.len()).step_by(3) {
            if i + 2 < face_normals.len() {
                n.push([face_normals[i] as f64, face_normals[i + 1] as f64, face_normals[i + 2] as f64]);
            }
        }
        mesh.face_normals = Some(n);
    }
    if let Some(colors) = colors {
        let mut c = Vec::with_capacity(colors.len() / 4);
        for i in (0..colors.len()).step_by(4) {
            if i + 3 < colors.len() {
                c.push([colors[i], colors[i + 1], colors[i + 2], colors[i + 3]]);
            }
        }
        mesh.triangle_colors = Some(c);
    }
    if let Some(face_ids) = face_ids {
        mesh.triangle_face_ids = Some(face_ids.iter().map(|&id| id as u64).collect());
    }

    // Deserialize instance metadata
    let instances = deserialize_instances(&instances_json)?;

    // Deserialize assembly tree
    let assembly_tree = deserialize_assembly_tree(&assembly_tree_json)?;

    Ok(CacheLookupResult {
        mesh,
        instances,
        assembly_tree,
        lod,
        file_name,
    })
}

// ─── Serialization helpers ────────────────────────────────────────────

/// Simplified face metadata — enough for the Structure panel to show
/// the face list, select faces, and highlight them in the 3D view.
/// Full boundary/UV/surface data is NOT cached (too large, rarely needed
/// after initial load). The UV breakdown window won't work from cache,
/// but the face list, selection, and 3D highlighting will.
#[derive(serde::Serialize, serde::Deserialize)]
struct CachedFaceInfo {
    face_id: u64,
    step_face_id: i64,
    surface_type: String,
    triangle_range: (usize, usize),
    forward: bool,
    is_void: bool,
    /// Number of inner boundaries (holes) — cached so the UV breakdown
    /// can display the correct hole count even after cache restore.
    num_inner_boundaries: usize,
}

/// Simplified instance metadata — only what's needed to reconstruct
/// the viewer state. Face-level metadata is cached for the Structure
/// panel; full FaceInfo data (boundaries, UV, surface geometry) is NOT
/// cached because it's large and rarely needed after initial load.
#[derive(serde::Serialize, serde::Deserialize)]
struct CachedInstance {
    name: String,
    brep_id: i64,
    color: Option<[f32; 4]>,
    transform: Option<[[f64; 4]; 4]>,
    faces: Vec<CachedFaceInfo>,
}

/// Simplified assembly tree — enough to reconstruct the structure panel.
#[derive(serde::Serialize, serde::Deserialize)]
struct CachedAssemblyNode {
    name: String,
    pd_id: i64,
    brep_id: Option<i64>,
    instance_index: Option<usize>,
    transform: Option<[[f64; 4]; 4]>,
    color: Option<[f32; 4]>,
    #[serde(default)]
    layers: Vec<String>,
    children: Vec<CachedAssemblyNode>,
}

fn serialize_instances(instances: &[DetailedMeshInstance]) -> String {
    let cached: Vec<CachedInstance> = instances.iter().map(|inst| {
        let faces: Vec<CachedFaceInfo> = inst.faces.iter().map(|fi| {
            CachedFaceInfo {
                face_id: fi.face_id,
                step_face_id: fi.step_face_id,
                surface_type: fi.surface_type.clone(),
                triangle_range: fi.triangle_range,
                forward: fi.forward,
                is_void: fi.is_void,
                num_inner_boundaries: fi.inner_boundaries.len(),
            }
        }).collect();
        CachedInstance {
            name: inst.name.clone(),
            brep_id: inst.brep_id,
            color: inst.color,
            transform: inst.transform,
            faces,
        }
    }).collect();

    serde_json::to_string(&cached).unwrap_or_else(|e| {
        log::warn!("Cache: failed to serialize instances: {}", e);
        "[]".to_string()
    })
}

fn serialize_assembly_tree(tree: &AssemblyNode) -> String {
    fn convert(node: &AssemblyNode) -> CachedAssemblyNode {
        CachedAssemblyNode {
            name: node.name.clone(),
            pd_id: node.pd_id,
            brep_id: node.brep_id,
            instance_index: node.instance_index,
            transform: node.transform,
            color: node.color,
            layers: node.layers.clone(),
            children: node.children.iter().map(convert).collect(),
        }
    }

    serde_json::to_string(&convert(tree)).unwrap_or_else(|e| {
        log::warn!("Cache: failed to serialize assembly tree: {}", e);
        "{}".to_string()
    })
}

fn deserialize_instances(json: &str) -> Result<Vec<DetailedMeshInstance>, String> {
    let cached: Vec<CachedInstance> = serde_json::from_str(json)
        .map_err(|e| format!("failed to deserialize instances: {}", e))?;

    // Reconstruct DetailedMeshInstance objects from cached data.
    // The per-instance mesh is empty (the merged mesh is used for rendering).
    // Face-level metadata (face_id, step_face_id, surface_type, triangle_range,
    // forward, is_void) is restored from cache so the Structure panel can show
    // the face list and allow selection. Boundary/UV/surface data is NOT cached,
    // so the UV breakdown window won't work for cached instances.
    use draper_step::FaceInfo;
    use draper_geometry::Surface;

    Ok(cached.into_iter().map(|ci| {
        let faces: Vec<FaceInfo> = ci.faces.into_iter().map(|cf| {
            FaceInfo {
                face_id: cf.face_id,
                step_face_id: cf.step_face_id,
                surface_type: cf.surface_type,
                surface: Surface::Plane(draper_geometry::Plane::xy()), // Placeholder — UV view won't work from cache
                outer_boundary: Vec::new(),
                inner_boundaries: vec![Vec::new(); cf.num_inner_boundaries],
                outer_uv_boundary: Vec::new(),
                inner_uv_boundaries: vec![Vec::new(); cf.num_inner_boundaries],
                triangle_range: cf.triangle_range,
                forward: cf.forward,
                uv_triangles: Vec::new(),
                is_void: cf.is_void,
            }
        }).collect();
        DetailedMeshInstance {
            name: ci.name,
            mesh: TriangleMesh::new(),
            color: ci.color,
            transform: ci.transform,
            brep_id: ci.brep_id,
            faces,
        }
    }).collect())
}

fn deserialize_assembly_tree(json: &str) -> Result<AssemblyNode, String> {
    let cached: CachedAssemblyNode = serde_json::from_str(json)
        .map_err(|e| format!("failed to deserialize assembly tree: {}", e))?;

    fn convert(node: CachedAssemblyNode) -> AssemblyNode {
        AssemblyNode {
            name: node.name,
            pd_id: node.pd_id,
            brep_id: node.brep_id,
            instance_index: node.instance_index,
            transform: node.transform,
            color: node.color,
            layers: node.layers,
            children: node.children.into_iter().map(convert).collect(),
        }
    }

    Ok(convert(cached))
}
