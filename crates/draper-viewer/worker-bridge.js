// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! High-level API for communicating with the 3Draper Web Worker.
//!
//! This module provides a promise-based interface for the main thread to
//! offload STEP parsing and triangulation to a background Web Worker,
//! keeping the UI thread responsive during heavy computation.
//!
//! # Usage
//!
//! ```js
//! import { DraperWorkerBridge } from './worker-bridge.js';
//!
//! const bridge = new DraperWorkerBridge();
//! await bridge.ready();
//!
//! // Parse a STEP file
//! const parseResult = await bridge.parseStep(fileContent, 'model.stp');
//!
//! // Triangulate BREPs one at a time (progressive)
//! while (bridge.hasPending()) {
//!     const result = await bridge.triangulateNext();
//!     if (result.mesh) {
//!         addMeshToScene(result.mesh);
//!     }
//!     updateProgressBar(result.progress);
//! }
//! ```

export class DraperWorkerBridge {
    constructor(workerUrl = 'worker.js') {
        this.worker = new Worker(workerUrl, { type: 'module' });
        this.requestId = 0;
        this.pendingCallbacks = new Map();  // id → { resolve, reject }
        this.eventHandlers = new Map();     // event type → callback[]
        this.isReady = false;
        this.readyPromise = null;

        // Worker message routing
        this.worker.onmessage = (e) => this._handleMessage(e.data);
        this.worker.onerror = (e) => this._handleError(e);

        // Create ready promise
        this.readyPromise = new Promise((resolve) => {
            this._readyResolve = resolve;
        });

        // Listen for the 'ready' event
        this.on('ready', () => {
            this.isReady = true;
            this._readyResolve();
        });
    }

    // ─── Lifecycle ──────────────────────────────────────────────────────

    /**
     * Wait for the worker to be ready (WASM initialized).
     * @returns {Promise<void>}
     */
    async ready() {
        return this.readyPromise;
    }

    /**
     * Terminate the worker and clean up resources.
     */
    terminate() {
        this.worker.terminate();
        this.pendingCallbacks.clear();
        this.eventHandlers.clear();
    }

    // ─── STEP Parsing ───────────────────────────────────────────────────

    /**
     * Parse a STEP file in the worker.
     * @param {string} content - The STEP file content (text)
     * @param {string} name - File name for logging
     * @param {number} [lod=1.0] - Level of detail (0.0=coarse, 1.0=full quality)
     * @param {string} [profile='Desktop'] - Steiner budget profile: 'Desktop', 'Tablet', 'Mobile'
     * @returns {Promise<{stats: object, pending_breps_json: string, assembly_tree_json: string}>} Parse result
     */
    async parseStep(content, name = 'unknown.stp', lod = 1.0, profile = 'Desktop') {
        const id = ++this.requestId;
        return new Promise((resolve, reject) => {
            this.pendingCallbacks.set(id, { resolve, reject });
            this.worker.postMessage({
                type: 'parse',
                id,
                content,
                name,
                lod,
                profile,
            });
        });
    }

    // ─── Triangulation ──────────────────────────────────────────────────

    /**
     * Triangulate the next pending BREP in the worker.
     * @returns {Promise<{mesh: object|null, name: string, brep_id: number, progress: object}>}
     */
    async triangulateNext() {
        const id = ++this.requestId;
        return new Promise((resolve, reject) => {
            this.pendingCallbacks.set(id, { resolve, reject });
            this.worker.postMessage({
                type: 'triangulate_next',
                id,
            });
        });
    }

    /**
     * Triangulate multiple BREPs in one message round-trip.
     * @param {number} maxCount - Maximum number of BREPs to process
     * @returns {Promise<void>}
     */
    async triangulateBatch(maxCount = 4) {
        const id = ++this.requestId;
        return new Promise((resolve, reject) => {
            this.pendingCallbacks.set(id, { resolve, reject });
            this.worker.postMessage({
                type: 'triangulate_batch',
                id,
                max_count: maxCount,
            });
        });
    }

    /**
     * Cancel all pending triangulation.
     */
    cancel() {
        this.worker.postMessage({ type: 'cancel' });
        // Reject all pending callbacks
        for (const [id, { reject }] of this.pendingCallbacks) {
            reject(new Error('Operation cancelled'));
        }
        this.pendingCallbacks.clear();
    }

    /**
     * Get the assembly structure tree.
     * @returns {Promise<object>} Assembly tree
     */
    async getStructure() {
        const id = ++this.requestId;
        return new Promise((resolve, reject) => {
            this.pendingCallbacks.set(id, { resolve, reject });
            this.worker.postMessage({
                type: 'get_structure',
                id,
            });
        });
    }

    // ─── Event Handling ─────────────────────────────────────────────────

    /**
     * Register an event handler for worker events.
     * Events: 'ready', 'parse_complete', 'triangulate_result',
     *         'triangulate_error', 'all_complete', 'error'
     *
     * @param {string} eventType - Event type
     * @param {function} callback - Event handler
     */
    on(eventType, callback) {
        if (!this.eventHandlers.has(eventType)) {
            this.eventHandlers.set(eventType, []);
        }
        this.eventHandlers.get(eventType).push(callback);
    }

    /**
     * Remove an event handler.
     * @param {string} eventType - Event type
     * @param {function} callback - Event handler to remove
     */
    off(eventType, callback) {
        const handlers = this.eventHandlers.get(eventType);
        if (handlers) {
            const idx = handlers.indexOf(callback);
            if (idx >= 0) handlers.splice(idx, 1);
        }
    }

    // ─── Internal ───────────────────────────────────────────────────────

    _handleMessage(data) {
        const { type, id } = data;

        // Resolve pending promise if this is a response to a request
        if (id && this.pendingCallbacks.has(id)) {
            const { resolve, reject } = this.pendingCallbacks.get(id);
            this.pendingCallbacks.delete(id);

            if (type === 'error') {
                reject(new Error(data.message));
            } else {
                resolve(data);
            }
        }

        // Fire event handlers
        const handlers = this.eventHandlers.get(type);
        if (handlers) {
            for (const handler of handlers) {
                try {
                    handler(data);
                } catch (err) {
                    console.error('[DraperWorkerBridge] Event handler error:', err);
                }
            }
        }
    }

    _handleError(event) {
        console.error('[DraperWorkerBridge] Worker error:', event.message);

        // Reject all pending callbacks
        for (const [id, { reject }] of this.pendingCallbacks) {
            reject(new Error(`Worker error: ${event.message}`));
        }
        this.pendingCallbacks.clear();

        // Fire error handlers
        const handlers = this.eventHandlers.get('error');
        if (handlers) {
            for (const handler of handlers) {
                handler({ message: event.message });
            }
        }
    }
}

// ─── COOP/COEP Detection ───────────────────────────────────────────────
// SharedArrayBuffer (required for WASM parallel threading) is only available
// when the page is served with COOP/COEP headers. This utility detects
// whether the browser has the required security context.

export function isSharedArrayBufferAvailable() {
    try {
        return typeof SharedArrayBuffer !== 'undefined';
    } catch {
        return false;
    }
}

/**
 * Check if the browser supports the requirements for WASM parallel threading.
 * Returns an object with:
 *   - supported: boolean
 *   - reason: string explaining why not supported (if applicable)
 */
export function checkParallelSupport() {
    if (!isSharedArrayBufferAvailable()) {
        return {
            supported: false,
            reason: 'SharedArrayBuffer not available — requires Cross-Origin-Isolation (COOP/COEP headers). ' +
                    'See: https://web.dev/coop-coep/'
        };
    }

    if (!window.crossOriginIsolated) {
        return {
            supported: false,
            reason: 'Page is not cross-origin isolated. Add these HTTP headers:\n' +
                    '  Cross-Origin-Opener-Policy: same-origin\n' +
                    '  Cross-Origin-Embedder-Policy: require-corp'
        };
    }

    return { supported: true, reason: '' };
}
