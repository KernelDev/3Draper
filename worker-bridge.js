// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! High-level API for communicating with the 3Draper Web Worker.
//!
//! This module provides the bridge between the Rust viewer (via wasm-bindgen)
//! and the background Web Worker. It is exposed as `window.draperWorkerBridge`
//! for the Rust code to call into.
//!
//! # Usage from Rust
//!
//! ```rust
//! // In app.rs (WASM only):
//! let bridge_exists = js_sys::Reflect::has(&window, &JsValue::from_str("draperWorkerBridge"));
//! let bridge = js_sys::Reflect::get(&window, &JsValue::from_str("draperWorkerBridge"));
//! bridge.parseStep(content, name, lod, is_mobile);
//! ```

class DraperWorkerBridge {
    constructor(workerUrl = 'worker.js') {
        this.worker = null;
        this.isReady = false;
        this.pendingMeshCount = 0;
        this.completedMeshes = [];  // Results waiting to be consumed by Rust
        this.parseResolve = null;
        this.parseReject = null;
        this.assemblyTree = null;
        this.currentBrepCount = 0;
        this.triangulatedBrepCount = 0;
        this.isProcessing = false;

        try {
            this.worker = new Worker(workerUrl, { type: 'module' });
            this.worker.onmessage = (e) => this._handleMessage(e.data);
            this.worker.onerror = (e) => this._handleError(e);
            console.log('[DraperWorkerBridge] Worker created');
        } catch (err) {
            console.warn('[DraperWorkerBridge] Failed to create worker:', err);
            this.worker = null;
        }
    }

    // ─── Lifecycle ──────────────────────────────────────────────────────

    async ready() {
        // Wait for the worker to signal readiness (WASM initialized)
        if (this.isReady) return;
        return new Promise((resolve) => {
            const check = () => {
                if (this.isReady) { resolve(); return; }
                setTimeout(check, 100);
            };
            check();
        });
    }

    terminate() {
        if (this.worker) {
            this.worker.terminate();
            this.worker = null;
        }
        this.isReady = false;
        this.completedMeshes = [];
        this.pendingMeshCount = 0;
    }

    // ─── STEP Parsing ───────────────────────────────────────────────────

    parseStep(content, name, lod, is_mobile) {
        if (!this.worker || !this.isReady) {
            console.warn('[DraperWorkerBridge] Worker not ready for parseStep');
            return false;
        }

        this.completedMeshes = [];
        this.pendingMeshCount = 0;
        this.currentBrepCount = 0;
        this.triangulatedBrepCount = 0;
        this.isProcessing = true;

        this.worker.postMessage({
            type: 'parse',
            id: 1,
            content: content,
            name: name,
            lod: lod,
            is_mobile: is_mobile,
        });

        return true;
    }

    // ─── Triangulation ──────────────────────────────────────────────────

    triangulateNext() {
        if (!this.worker || !this.isReady || !this.isProcessing) {
            return false;
        }

        this.worker.postMessage({
            type: 'triangulate_next',
            id: 2,
        });

        return true;
    }

    cancel() {
        if (this.worker) {
            this.worker.postMessage({ type: 'cancel' });
        }
        this.completedMeshes = [];
        this.pendingMeshCount = 0;
        this.isProcessing = false;
    }

    // ─── Result Retrieval (called from Rust) ────────────────────────────

    drainResults() {
        const results = this.completedMeshes;
        this.completedMeshes = [];
        this.pendingMeshCount = 0;
        return results;
    }

    // ─── Internal ───────────────────────────────────────────────────────

    _handleMessage(data) {
        const { type } = data;

        switch (type) {
            case 'ready':
                this.isReady = true;
                console.log('[DraperWorkerBridge] Worker ready');
                break;

            case 'parse_complete':
                this.currentBrepCount = data.stats.brep_count || 0;
                this.assemblyTree = data.stats;
                console.log(`[DraperWorkerBridge] Parse complete — ${this.currentBrepCount} BREPs`);
                // Start requesting triangulation results
                if (this.currentBrepCount > 0) {
                    this._requestMore();
                }
                break;

            case 'triangulate_result':
                this.triangulatedBrepCount++;
                this.completedMeshes.push(data.mesh);
                this.pendingMeshCount = this.completedMeshes.length;
                console.log(`[DraperWorkerBridge] Got mesh: '${data.name}' — ${data.mesh.vertex_count} verts, ${data.mesh.triangle_count} tris (${data.elapsed_ms.toFixed(0)}ms) [${this.triangulatedBrepCount}/${this.currentBrepCount}]`);
                // Request more after a small delay to allow the main thread to process
                if (data.progress && data.progress.completed < data.progress.total) {
                    this._requestMore();
                }
                break;

            case 'triangulate_error':
                this.triangulatedBrepCount++;
                console.warn(`[DraperWorkerBridge] Triangulation error for '${data.name}': ${data.error}`);
                if (data.progress && data.progress.completed < data.progress.total) {
                    this._requestMore();
                }
                break;

            case 'all_complete':
                this.isProcessing = false;
                console.log(`[DraperWorkerBridge] All BREPs complete — ${this.triangulatedBrepCount}/${this.currentBrepCount}`);
                break;

            case 'error':
                console.error(`[DraperWorkerBridge] Worker error: ${data.message}`);
                this.isProcessing = false;
                break;

            default:
                console.warn('[DraperWorkerBridge] Unknown message:', type);
        }
    }

    _handleError(event) {
        console.error('[DraperWorkerBridge] Worker error:', event.message);
        this.isProcessing = false;
    }

    _requestMore() {
        // Use setTimeout(0) to yield to the browser before requesting more.
        // This allows the Rust side to drain completed meshes before we
        // fill up the queue again.
        setTimeout(() => {
            if (this.isProcessing && this.worker) {
                this.triangulateNext();
            }
        }, 0);
    }
}

// ─── Global Instance ───────────────────────────────────────────────────
// Create the bridge instance and expose it globally so the Rust viewer
// can call into it via js_sys::Reflect.

try {
    window.draperWorkerBridge = new DraperWorkerBridge('worker.js');
    console.log('[DraperWorkerBridge] Global bridge instance created');
} catch (err) {
    console.warn('[DraperWorkerBridge] Failed to create bridge:', err);
    window.draperWorkerBridge = null;
}
