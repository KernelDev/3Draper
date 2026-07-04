// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Web Worker for offloading STEP parsing and triangulation from the main thread.
//!
//! This worker runs the heavy computational tasks (STEP file parsing, B-Rep
//! triangulation, mesh generation) in a background thread, keeping the main
//! thread free for UI rendering at 60+ FPS.
//!
//! # Architecture
//!
//! Main Thread (UI):
//!   - Handles user input, egui/wgpu rendering
//!   - Sends STEP file content to worker via postMessage
//!   - Receives triangulated mesh data back via onmessage
//!
//! Worker Thread:
//!   - Initializes lightweight WASM module (draper-worker.wasm)
//!   - Parses STEP files into B-Rep topology
//!   - Triangulates B-Reps one at a time
//!   - Sends completed mesh vertices/indices back to main thread
//!
//! # Message Protocol
//!
//! Main → Worker:
//!   { type: "init" }                           — Initialize WASM (auto-sent on worker start)
//!   { type: "parse", id, content, name, lod, profile }  — Parse STEP file content
//!   { type: "triangulate_next", id }           — Triangulate next pending BREP
//!   { type: "triangulate_batch", id, max_count } — Triangulate batch of BREPs
//!   { type: "cancel" }                         — Cancel current processing
//!   { type: "get_structure", id }              — Get assembly structure tree
//!   { type: "get_pending_count", id }          — Get number of remaining BREPs
//!
//! Worker → Main:
//!   { type: "ready" }                          — Worker initialized and ready
//!   { type: "parse_complete", id, stats, pending_breps_json, assembly_tree_json }
//!   { type: "triangulate_result", id, mesh, brep_id, name, color, elapsed_ms, progress }
//!   { type: "triangulate_error", id, name, brep_id, error, progress }
//!   { type: "all_complete", id, summary }
//!   { type: "structure", id, tree }
//!   { type: "pending_count", id, count }
//!   { type: "error", message }

// State for the current STEP file being processed
let wasmExports = null;
let wasmInitialized = false;
let pendingBreps = [];           // Array of pending BREP descriptors from parse
let triangulatedCount = 0;
let totalBrepCount = 0;
let assemblyTree = null;
let requestId = 0;               // ID of the current request (to handle cancellation)

// ─── WASM Initialization ───────────────────────────────────────────────
//
// The worker loads a LIGHTWEIGHT WASM module (draper-worker.wasm) that
// contains only STEP parsing and triangulation — no eframe/egui/wgpu.
// This is much smaller and faster to load than the full viewer WASM.
//
// IMPORTANT: The worker resolves the WASM module path relative to its own
// location. The build system places draper-worker.js and draper-worker_bg.wasm
// in the same directory as index.html.

async function initWasm() {
    try {
        // Resolve the WASM module path relative to this worker script.
        const baseUrl = self.location.href.replace(/\/worker\.js$/, '/');
        const wasmJsUrl = baseUrl + 'draper-worker.js';

        // Import the wasm-bindgen generated JS glue for the worker module
        const module = await import(wasmJsUrl);

        // Initialize the WASM module
        if (module.default) {
            await module.default();
        }

        wasmExports = module;
        wasmInitialized = true;

        self.postMessage({ type: 'ready' });
        console.log('[3Draper Worker] WASM initialized successfully (draper-worker.wasm)');
    } catch (err) {
        console.error('[3Draper Worker] WASM initialization failed:', err);
        self.postMessage({
            type: 'error',
            message: `WASM init failed: ${err.message}. ` +
                     'The worker cannot load the WASM module. Falling back to main-thread processing.'
        });
    }
}

// Start initialization immediately on worker startup
initWasm();

// ─── Message Handler ───────────────────────────────────────────────────

self.onmessage = async function(e) {
    const msg = e.data;

    if (!wasmInitialized && msg.type !== 'init') {
        self.postMessage({ type: 'error', message: 'Worker not initialized yet' });
        return;
    }

    switch (msg.type) {
        case 'init':
            if (!wasmInitialized) {
                await initWasm();
            }
            break;

        case 'parse':
            handleParse(msg);
            break;

        case 'triangulate_next':
            handleTriangulateNext(msg);
            break;

        case 'triangulate_batch':
            handleTriangulateBatch(msg);
            break;

        case 'cancel':
            handleCancel();
            break;

        case 'get_structure':
            handleGetStructure(msg);
            break;

        case 'get_pending_count':
            handleGetPendingCount(msg);
            break;

        default:
            console.warn('[3Draper Worker] Unknown message type:', msg.type);
    }
};

// ─── STEP Parsing ──────────────────────────────────────────────────────
//
// The STEP parsing happens entirely in WASM (Rust code). The worker
// sends the file content to the Rust parse_step_worker function and
// receives back a structured result with:
//   - Entity counts (faces, breps, shells, etc.)
//   - Pending BREP descriptors (JSON, for progressive triangulation)
//   - Assembly tree structure (JSON)
//
// After parsing, the worker stores the pending BREP list locally and
// the WASM context is stored in thread-local Rust state. Subsequent
// triangulate_next calls use the stored context.

function handleParse(msg) {
    const { id, content, name, lod, profile } = msg;
    requestId = id;

    try {
        // LOD defaults to 1.0 (full quality) if not specified
        const lodValue = lod !== undefined ? lod : 1.0;
        // Profile defaults to "Desktop" if not specified
        const profileName = profile || 'Desktop';

        // Call the Rust parse_step_worker function via wasm-bindgen
        const result = wasmExports.parse_step_worker(content, name, lodValue, profileName);

        // Parse the JSON strings returned by Rust
        pendingBreps = JSON.parse(result.pending_breps_json);
        assemblyTree = JSON.parse(result.assembly_tree_json);
        totalBrepCount = pendingBreps.length;
        triangulatedCount = 0;

        self.postMessage({
            type: 'parse_complete',
            id: id,
            stats: {
                name: name,
                entity_count: result.entity_count,
                face_count: result.face_count,
                brep_count: totalBrepCount,
                shell_count: result.shell_count,
            },
            pending_breps_json: result.pending_breps_json,
            assembly_tree_json: result.assembly_tree_json,
        });

    } catch (err) {
        console.error('[3Draper Worker] Parse error:', err);
        self.postMessage({
            type: 'error',
            message: `Parse failed: ${err.message}`
        });
    }
}

// ─── Triangulation ─────────────────────────────────────────────────────
//
// Progressive triangulation: process one BREP at a time, sending
// the resulting mesh data back to the main thread for rendering.
// This keeps the worker responsive and allows the UI to show progress.

function handleTriangulateNext(msg) {
    // Check if there are BREPs left to triangulate
    const remainingCount = wasmExports.pending_brep_count();

    if (remainingCount === 0) {
        self.postMessage({
            type: 'all_complete',
            id: requestId,
            summary: {
                triangulated: triangulatedCount,
                total: totalBrepCount,
            }
        });
        return;
    }

    try {
        const startTime = performance.now();

        // Always triangulate the BREP at index 0 (the worker removes it from the list)
        const meshResult = wasmExports.triangulate_brep_structured(0);
        const elapsed = performance.now() - startTime;

        // Get the info for the BREP we just processed (from our local pending list)
        const brep = pendingBreps.shift();

        if (meshResult && meshResult.vertex_count > 0) {
            triangulatedCount++;
            const meshData = {
                vertex_count: meshResult.vertex_count,
                triangle_count: meshResult.triangle_count,
                vertices: meshResult.vertices(),
                indices: meshResult.indices(),
                normals: meshResult.normals(),
                face_normals: meshResult.face_normals(),
                colors: meshResult.colors(),
            };
            self.postMessage({
                type: 'triangulate_result',
                id: requestId,
                mesh: meshData,
                brep_id: brep ? brep.brep_id : -1,
                name: brep ? brep.name : 'unknown',
                color: brep ? brep.color : null,
                elapsed_ms: elapsed,
                progress: {
                    completed: triangulatedCount,
                    total: totalBrepCount,
                }
            }, getTransferables(meshData));
        } else {
            triangulatedCount++;
            self.postMessage({
                type: 'triangulate_error',
                id: requestId,
                name: brep ? brep.name : 'unknown',
                brep_id: brep ? brep.brep_id : -1,
                error: 'Empty mesh produced',
                progress: {
                    completed: triangulatedCount,
                    total: totalBrepCount,
                }
            });
        }
    } catch (err) {
        console.error('[3Draper Worker] Triangulation error:', err);
        const brep = pendingBreps.shift(); // Remove it even on error
        triangulatedCount++;
        self.postMessage({
            type: 'triangulate_error',
            id: requestId,
            name: brep ? brep.name : 'unknown',
            brep_id: brep ? brep.brep_id : -1,
            error: err.message,
            progress: {
                completed: triangulatedCount,
                total: totalBrepCount,
            }
        });
    }

    // Check if all done
    const stillRemaining = wasmExports.pending_brep_count();
    if (stillRemaining === 0) {
        self.postMessage({
            type: 'all_complete',
            id: requestId,
            summary: {
                triangulated: triangulatedCount,
                total: totalBrepCount,
            }
        });
    }
}

function handleTriangulateBatch(msg) {
    const { max_count } = msg;
    let count = 0;
    const maxBatch = max_count || 1;

    while (wasmExports.pending_brep_count() > 0 && count < maxBatch) {
        handleTriangulateNext(msg);
        count++;
    }
}

function handleCancel() {
    // Tell the WASM module to abort and free the context
    if (wasmExports && wasmExports.cancel_triangulation) {
        try {
            wasmExports.cancel_triangulation();
        } catch (err) {
            console.warn('[3Draper Worker] Cancel error:', err);
        }
    }
    pendingBreps = [];
    assemblyTree = null;
    triangulatedCount = 0;
    totalBrepCount = 0;
    requestId = 0;
}

function handleGetStructure(msg) {
    self.postMessage({
        type: 'structure',
        id: msg.id,
        tree: assemblyTree,
    });
}

function handleGetPendingCount(msg) {
    const count = wasmExports ? wasmExports.pending_brep_count() : 0;
    self.postMessage({
        type: 'pending_count',
        id: msg.id,
        count: count,
    });
}

// ─── Transferable Objects ──────────────────────────────────────────────
//
// Use transferable ArrayBuffers for zero-copy mesh data transfer.
// When an ArrayBuffer is transferred, the sender loses access to it,
// but the receiver gets it instantly without copying.

function getTransferables(meshData) {
    const transferables = [];
    if (meshData.vertices instanceof Float32Array && meshData.vertices.buffer) {
        transferables.push(meshData.vertices.buffer);
    }
    if (meshData.indices instanceof Uint32Array && meshData.indices.buffer) {
        transferables.push(meshData.indices.buffer);
    }
    if (meshData.normals instanceof Float32Array && meshData.normals && meshData.normals.buffer) {
        transferables.push(meshData.normals.buffer);
    }
    if (meshData.face_normals instanceof Float32Array && meshData.face_normals && meshData.face_normals.buffer) {
        transferables.push(meshData.face_normals.buffer);
    }
    if (meshData.colors instanceof Float32Array && meshData.colors && meshData.colors.buffer) {
        transferables.push(meshData.colors.buffer);
    }
    return transferables;
}
