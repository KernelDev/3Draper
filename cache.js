// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
/**
 * DraperCache — IndexedDB-backed triangulation cache for 3Draper WASM viewer.
 *
 * Caches the results of STEP → mesh triangulation so that re-opening the same
 * file at the same LOD level loads instantly (< 1s) instead of re-triangulating.
 *
 * Cache key:  SHA-256(fileContent) + LOD value
 * Cache value: JSON-serialised mesh data (vertices, indices, normals, etc.)
 *              plus instance metadata and assembly tree.
 *
 * Features:
 *   - 7-day TTL (entries older than 7 days are evicted on next cleanup)
 *   - 500 MB size cap with LRU-style eviction (oldest entries evicted first)
 *   - Clear-cache button in Settings
 *   - "Loaded from cache" log message on cache hit
 */

class DraperCache {
    constructor() {
        this.db = null;
        this._initPromise = this._init();
    }

    async _init() {
        return new Promise((resolve) => {
            const request = indexedDB.open('3draper_cache', 1);
            request.onupgradeneeded = (e) => {
                const db = e.target.result;
                if (!db.objectStoreNames.contains('meshes')) {
                    db.createObjectStore('meshes');
                }
            };
            request.onsuccess = (e) => {
                this.db = e.target.result;
                console.log('[Cache] IndexedDB opened successfully');
                resolve(true);
            };
            request.onerror = (e) => {
                console.error('[Cache] IndexedDB open failed:', e.target.error);
                resolve(false);
            };
        });
    }

    /**
     * Wait for the database to be ready.
     */
    async ready() {
        return this._initPromise;
    }

    /**
     * Compute SHA-256 hash of a string (UTF-8 encoded).
     */
    async hashContent(content) {
        const encoder = new TextEncoder();
        const data = encoder.encode(content);
        const hashBuffer = await crypto.subtle.digest('SHA-256', data);
        const hashArray = Array.from(new Uint8Array(hashBuffer));
        return hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
    }

    /**
     * Look up a cached triangulation result.
     * @param {string} hash - SHA-256 hash of the STEP file content
     * @param {number} lod - LOD value (e.g., 0.5, 1.0)
     * @returns {Object|null} Cached data or null if not found / expired
     */
    async lookup(hash, lod) {
        if (!this.db) {
            console.warn('[Cache] DB not ready for lookup');
            return null;
        }

        const key = `${hash}_lod${lod.toFixed(2)}`;
        return new Promise((resolve) => {
            try {
                const tx = this.db.transaction('meshes', 'readonly');
                const store = tx.objectStore('meshes');
                const request = store.get(key);
                request.onsuccess = () => {
                    const data = request.result;
                    if (!data) {
                        resolve(null);
                        return;
                    }

                    // Check TTL (7 days)
                    const now = Date.now();
                    const age = now - (data.timestamp || 0);
                    const sevenDays = 7 * 24 * 60 * 60 * 1000;
                    if (age > sevenDays) {
                        console.log(`[Cache] Entry '${key}' expired (age: ${(age / 1000 / 3600).toFixed(1)}h) — evicting`);
                        this._evict(key);
                        resolve(null);
                        return;
                    }

                    console.log(`[Cache] Cache HIT for '${key}' (age: ${(age / 1000).toFixed(0)}s, instances: ${data.instances?.length || 0})`);
                    resolve(data);
                };
                request.onerror = () => {
                    console.error('[Cache] Lookup error:', request.error);
                    resolve(null);
                };
            } catch (e) {
                console.error('[Cache] Lookup exception:', e);
                resolve(null);
            }
        });
    }

    /**
     * Store a triangulation result in the cache.
     * @param {string} hash - SHA-256 hash of the STEP file content
     * @param {number} lod - LOD value
     * @param {Object} data - Cache data to store
     */
    async store(hash, lod, data) {
        if (!this.db) {
            console.warn('[Cache] DB not ready for store');
            return;
        }

        const key = `${hash}_lod${lod.toFixed(2)}`;
        data.timestamp = Date.now();
        data.hash = hash;
        data.lod = lod;

        // Estimate size (rough: each f32 = 4 bytes, u32 = 4 bytes)
        let estimatedSize = 0;
        if (data.instances) {
            for (const inst of data.instances) {
                estimatedSize += (inst.vertices?.length || 0) * 4;
                estimatedSize += (inst.indices?.length || 0) * 4;
                estimatedSize += (inst.normals?.length || 0) * 4;
                estimatedSize += (inst.face_normals?.length || 0) * 4;
                estimatedSize += (inst.colors?.length || 0) * 4;
            }
        }
        data.estimatedSize = estimatedSize;

        return new Promise((resolve) => {
            try {
                const tx = this.db.transaction('meshes', 'readwrite');
                const store = tx.objectStore('meshes');
                store.put(data, key);
                tx.oncomplete = () => {
                    console.log(`[Cache] Stored '${key}' (~${(estimatedSize / 1024 / 1024).toFixed(2)} MB)`);
                    // Schedule cleanup in background
                    this._cleanupIfNeeded();
                    resolve();
                };
                tx.onerror = () => {
                    console.error('[Cache] Store error:', tx.error);
                    resolve();
                };
            } catch (e) {
                console.error('[Cache] Store exception:', e);
                resolve();
            }
        });
    }

    /**
     * Evict a single entry by key.
     */
    async _evict(key) {
        if (!this.db) return;
        return new Promise((resolve) => {
            try {
                const tx = this.db.transaction('meshes', 'readwrite');
                const store = tx.objectStore('meshes');
                store.delete(key);
                tx.oncomplete = () => resolve();
                tx.onerror = () => resolve();
            } catch (e) {
                resolve();
            }
        });
    }

    /**
     * Clean up old entries if the total cache size exceeds 500 MB.
     * Evicts oldest entries first (LRU-style).
     */
    async _cleanupIfNeeded() {
        if (!this.db) return;

        const maxSizeBytes = 500 * 1024 * 1024; // 500 MB

        return new Promise((resolve) => {
            try {
                const tx = this.db.transaction('meshes', 'readonly');
                const store = tx.objectStore('meshes');
                const request = store.openCursor();
                const entries = [];

                request.onsuccess = (e) => {
                    const cursor = e.target.result;
                    if (cursor) {
                        entries.push({
                            key: cursor.key,
                            timestamp: cursor.value.timestamp || 0,
                            estimatedSize: cursor.value.estimatedSize || 0,
                        });
                        cursor.continue();
                    } else {
                        // All entries collected — check total size
                        let totalSize = 0;
                        for (const entry of entries) {
                            totalSize += entry.estimatedSize;
                        }

                        if (totalSize > maxSizeBytes) {
                            // Sort by timestamp ascending (oldest first)
                            entries.sort((a, b) => a.timestamp - b.timestamp);

                            // Evict oldest entries until under the limit
                            const toEvict = [];
                            let evictSize = 0;
                            for (const entry of entries) {
                                if (totalSize - evictSize <= maxSizeBytes) break;
                                toEvict.push(entry.key);
                                evictSize += entry.estimatedSize;
                            }

                            if (toEvict.length > 0) {
                                console.log(`[Cache] Evicting ${toEvict.length} entries (~${(evictSize / 1024 / 1024).toFixed(1)} MB) to stay under 500 MB limit`);
                                const delTx = this.db.transaction('meshes', 'readwrite');
                                const delStore = delTx.objectStore('meshes');
                                for (const key of toEvict) {
                                    delStore.delete(key);
                                }
                                delTx.oncomplete = () => resolve();
                                delTx.onerror = () => resolve();
                                return;
                            }
                        }
                        resolve();
                    }
                };
                request.onerror = () => resolve();
            } catch (e) {
                resolve();
            }
        });
    }

    /**
     * Clear all cached data.
     */
    async clear() {
        if (!this.db) return;
        return new Promise((resolve) => {
            try {
                const tx = this.db.transaction('meshes', 'readwrite');
                const store = tx.objectStore('meshes');
                store.clear();
                tx.oncomplete = () => {
                    console.log('[Cache] All entries cleared');
                    resolve();
                };
                tx.onerror = () => {
                    console.error('[Cache] Clear error:', tx.error);
                    resolve();
                };
            } catch (e) {
                console.error('[Cache] Clear exception:', e);
                resolve();
            }
        });
    }

    /**
     * Get estimated total cache size in bytes.
     */
    async getSize() {
        if (!this.db) return 0;

        return new Promise((resolve) => {
            try {
                const tx = this.db.transaction('meshes', 'readonly');
                const store = tx.objectStore('meshes');
                const request = store.openCursor();
                let totalSize = 0;
                let entryCount = 0;

                request.onsuccess = (e) => {
                    const cursor = e.target.result;
                    if (cursor) {
                        totalSize += cursor.value.estimatedSize || 0;
                        entryCount += 1;
                        cursor.continue();
                    } else {
                        resolve({ size: totalSize, count: entryCount });
                    }
                };
                request.onerror = () => resolve({ size: 0, count: 0 });
            } catch (e) {
                resolve({ size: 0, count: 0 });
            }
        });
    }
}

// Create global instance
window.draperCache = new DraperCache();
