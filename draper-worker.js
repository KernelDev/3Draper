/* @ts-self-types="./draper_worker.d.ts" */

/**
 * Triangulated mesh data returned to JS — flat arrays suitable for
 * `Float32Array` / `Uint32Array` views and zero-copy transfer.
 */
export class MeshDataResult {
    static __wrap(ptr) {
        const obj = Object.create(MeshDataResult.prototype);
        obj.__wbg_ptr = ptr;
        MeshDataResultFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        MeshDataResultFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_meshdataresult_free(ptr, 0);
    }
    /**
     * Per-triangle RGBA colors or empty Float32Array.
     * @returns {Float32Array}
     */
    colors() {
        const ret = wasm.meshdataresult_colors(this.__wbg_ptr);
        return ret;
    }
    /**
     * Face normals or empty Float32Array.
     * @returns {Float32Array}
     */
    face_normals() {
        const ret = wasm.meshdataresult_face_normals(this.__wbg_ptr);
        return ret;
    }
    /**
     * Flat triangle indices as Uint32Array: [i0,j0,k0, ...].
     * @returns {Uint32Array}
     */
    indices() {
        const ret = wasm.meshdataresult_indices(this.__wbg_ptr);
        return ret;
    }
    /**
     * Vertex normals or empty Float32Array.
     * @returns {Float32Array}
     */
    normals() {
        const ret = wasm.meshdataresult_normals(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get triangle_count() {
        const ret = wasm.meshdataresult_triangle_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {number}
     */
    get vertex_count() {
        const ret = wasm.meshdataresult_vertex_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Flat vertex positions as Float32Array: [x0,y0,z0, x1,y1,z1, ...].
     * @returns {Float32Array}
     */
    vertices() {
        const ret = wasm.meshdataresult_vertices(this.__wbg_ptr);
        return ret;
    }
}
if (Symbol.dispose) MeshDataResult.prototype[Symbol.dispose] = MeshDataResult.prototype.free;

/**
 * Result of STEP parsing, returned to JS.
 */
export class ParseResult {
    static __wrap(ptr) {
        const obj = Object.create(ParseResult.prototype);
        obj.__wbg_ptr = ptr;
        ParseResultFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        ParseResultFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_parseresult_free(ptr, 0);
    }
    /**
     * JSON assembly tree.
     * @returns {string}
     */
    get assembly_tree_json() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.parseresult_assembly_tree_json(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Number of BREP instances to triangulate.
     * @returns {number}
     */
    get brep_count() {
        const ret = wasm.parseresult_brep_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Total STEP entity count.
     * @returns {number}
     */
    get entity_count() {
        const ret = wasm.parseresult_entity_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Estimated face count.
     * @returns {number}
     */
    get face_count() {
        const ret = wasm.parseresult_face_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * JSON array of pending BREP descriptors.
     * @returns {string}
     */
    get pending_breps_json() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.parseresult_pending_breps_json(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Shell count.
     * @returns {number}
     */
    get shell_count() {
        const ret = wasm.parseresult_shell_count(this.__wbg_ptr);
        return ret >>> 0;
    }
}
if (Symbol.dispose) ParseResult.prototype[Symbol.dispose] = ParseResult.prototype.free;

/**
 * Cancel the current chunked triangulation session (if any)
 * and drop the context.
 */
export function cancel_triangulation() {
    wasm.cancel_triangulation();
}

/**
 * Drop the stored parse context and free WASM memory.
 *
 * Call this when triangulation is complete or cancelled.
 */
export function drop_parse_context() {
    wasm.drop_parse_context();
}

/**
 * Get the BREP ID and name for a pending BREP by index.
 *
 * Returns a JSON string: `{"name":"...","brep_id":123}` or `"null"`.
 * @param {number} brep_index
 * @returns {string}
 */
export function get_pending_brep_info(brep_index) {
    let deferred1_0;
    let deferred1_1;
    try {
        const ret = wasm.get_pending_brep_info(brep_index);
        deferred1_0 = ret[0];
        deferred1_1 = ret[1];
        return getStringFromWasm0(ret[0], ret[1]);
    } finally {
        wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
    }
}

/**
 * Parse a STEP file and prepare for progressive triangulation.
 *
 * This function:
 * 1. Parses the STEP text into an in-memory `StepFile`
 * 2. Calls `step_structure_lazy` to get assembly tree + pending BREP descriptors
 * 3. Creates an `OwnedStepConversionContext` (with LOD + profile)
 * 4. Stores the context + pending list in thread-local state
 * 5. Returns parse stats + JSON descriptors to JS
 *
 * After calling this, the JS worker can call `triangulate_brep_worker`
 * repeatedly to process each BREP.
 * @param {string} content
 * @param {string} name
 * @param {number} lod
 * @param {string} profile_name
 * @returns {ParseResult}
 */
export function parse_step_worker(content, name, lod, profile_name) {
    const ptr0 = passStringToWasm0(content, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len0 = WASM_VECTOR_LEN;
    const ptr1 = passStringToWasm0(name, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len1 = WASM_VECTOR_LEN;
    const ptr2 = passStringToWasm0(profile_name, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len2 = WASM_VECTOR_LEN;
    const ret = wasm.parse_step_worker(ptr0, len0, ptr1, len1, lod, ptr2, len2);
    if (ret[2]) {
        throw takeFromExternrefTable0(ret[1]);
    }
    return ParseResult.__wrap(ret[0]);
}

/**
 * Number of remaining pending BREPs in the current context.
 * @returns {number}
 */
export function pending_brep_count() {
    const ret = wasm.pending_brep_count();
    return ret >>> 0;
}

/**
 * Triangulate a single BREP instance and return structured mesh data.
 *
 * This is the main entry point called from `worker.js`. It:
 * 1. Removes the BREP at `brep_index` from the pending list
 * 2. Triangulates it using the stored `OwnedStepConversionContext`
 * 3. Returns `MeshDataResult` with flat arrays for zero-copy transfer
 *
 * On error (no context, out-of-range index, triangulation failure),
 * returns a JS string starting with "error:".
 * @param {number} brep_index
 * @returns {MeshDataResult}
 */
export function triangulate_brep_structured(brep_index) {
    const ret = wasm.triangulate_brep_structured(brep_index);
    if (ret[2]) {
        throw takeFromExternrefTable0(ret[1]);
    }
    return MeshDataResult.__wrap(ret[0]);
}

/**
 * Triangulate a single BREP instance by its index in the pending list.
 *
 * `brep_index` is the 0-based index into the pending BREP list returned
 * by `parse_step_worker`. After triangulation, the BREP is removed from
 * the pending list.
 *
 * Returns `MeshDataResult` with flat arrays for zero-copy transfer,
 * or an error string if triangulation fails.
 * @param {number} brep_index
 * @returns {any}
 */
export function triangulate_brep_worker(brep_index) {
    const ret = wasm.triangulate_brep_worker(brep_index);
    if (ret[2]) {
        throw takeFromExternrefTable0(ret[1]);
    }
    return takeFromExternrefTable0(ret[0]);
}

/**
 * Initialize the worker WASM module — installs panic hook and logger.
 * Must be called once before any other function (the worker.js `initWasm`
 * does this automatically after importing the module).
 */
export function worker_init() {
    wasm.worker_init();
}
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg___wbindgen_is_undefined_67b456be8673d3d7: function(arg0) {
            const ret = arg0 === undefined;
            return ret;
        },
        __wbg___wbindgen_throw_1506f2235d1bdba0: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbg_debug_78b457f1effb3792: function(arg0) {
            console.debug(arg0);
        },
        __wbg_error_78ff5b3a29b770e0: function(arg0) {
            console.error(arg0);
        },
        __wbg_error_a6fa202b58aa1cd3: function(arg0, arg1) {
            let deferred0_0;
            let deferred0_1;
            try {
                deferred0_0 = arg0;
                deferred0_1 = arg1;
                console.error(getStringFromWasm0(arg0, arg1));
            } finally {
                wasm.__wbindgen_free(deferred0_0, deferred0_1, 1);
            }
        },
        __wbg_info_af7f45292ba9b0ea: function(arg0) {
            console.info(arg0);
        },
        __wbg_log_cf2e968649f3384e: function(arg0) {
            console.log(arg0);
        },
        __wbg_new_227d7c05414eb861: function() {
            const ret = new Error();
            return ret;
        },
        __wbg_new_2c48d7fdccf94f7a: function(arg0) {
            const ret = new Float32Array(arg0);
            return ret;
        },
        __wbg_new_from_slice_47be4219028de35d: function(arg0, arg1) {
            const ret = new Uint32Array(getArrayU32FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_new_from_slice_956df4f769fb782c: function(arg0, arg1) {
            const ret = new Float32Array(getArrayF32FromWasm0(arg0, arg1));
            return ret;
        },
        __wbg_now_e7c6795a7f81e10f: function(arg0) {
            const ret = arg0.now();
            return ret;
        },
        __wbg_performance_3fcf6e32a7e1ed0a: function(arg0) {
            const ret = arg0.performance;
            return ret;
        },
        __wbg_stack_3b0d974bbf31e44f: function(arg0, arg1) {
            const ret = arg1.stack;
            const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len1 = WASM_VECTOR_LEN;
            getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
            getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
        },
        __wbg_static_accessor_GLOBAL_9d53f2689e622ca1: function() {
            const ret = typeof global === 'undefined' ? null : global;
            return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
        },
        __wbg_static_accessor_GLOBAL_THIS_a1a35cec07001a8a: function() {
            const ret = typeof globalThis === 'undefined' ? null : globalThis;
            return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
        },
        __wbg_static_accessor_SELF_4c59f6c7ea29a144: function() {
            const ret = typeof self === 'undefined' ? null : self;
            return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
        },
        __wbg_static_accessor_WINDOW_e70ae9f2eb052253: function() {
            const ret = typeof window === 'undefined' ? null : window;
            return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
        },
        __wbg_warn_410c3261e3c6d686: function(arg0) {
            console.warn(arg0);
        },
        __wbindgen_cast_0000000000000001: function(arg0) {
            // Cast intrinsic for `F64 -> Externref`.
            const ret = arg0;
            return ret;
        },
        __wbindgen_cast_0000000000000002: function(arg0, arg1) {
            // Cast intrinsic for `Ref(String) -> Externref`.
            const ret = getStringFromWasm0(arg0, arg1);
            return ret;
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./draper_worker_bg.js": import0,
    };
}

const MeshDataResultFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_meshdataresult_free(ptr, 1));
const ParseResultFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_parseresult_free(ptr, 1));

function addToExternrefTable0(obj) {
    const idx = wasm.__externref_table_alloc();
    wasm.__wbindgen_externrefs.set(idx, obj);
    return idx;
}

function getArrayF32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayU32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

let cachedDataViewMemory0 = null;
function getDataViewMemory0() {
    if (cachedDataViewMemory0 === null || cachedDataViewMemory0.buffer.detached === true || (cachedDataViewMemory0.buffer.detached === undefined && cachedDataViewMemory0.buffer !== wasm.memory.buffer)) {
        cachedDataViewMemory0 = new DataView(wasm.memory.buffer);
    }
    return cachedDataViewMemory0;
}

let cachedFloat32ArrayMemory0 = null;
function getFloat32ArrayMemory0() {
    if (cachedFloat32ArrayMemory0 === null || cachedFloat32ArrayMemory0.byteLength === 0) {
        cachedFloat32ArrayMemory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint32ArrayMemory0 = null;
function getUint32ArrayMemory0() {
    if (cachedUint32ArrayMemory0 === null || cachedUint32ArrayMemory0.byteLength === 0) {
        cachedUint32ArrayMemory0 = new Uint32Array(wasm.memory.buffer);
    }
    return cachedUint32ArrayMemory0;
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function isLikeNone(x) {
    return x === undefined || x === null;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

function takeFromExternrefTable0(idx) {
    const value = wasm.__wbindgen_externrefs.get(idx);
    wasm.__externref_table_dealloc(idx);
    return value;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    };
}

let WASM_VECTOR_LEN = 0;

let wasmModule, wasmInstance, wasm;
function __wbg_finalize_init(instance, module) {
    wasmInstance = instance;
    wasm = instance.exports;
    wasmModule = module;
    cachedDataViewMemory0 = null;
    cachedFloat32ArrayMemory0 = null;
    cachedUint32ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('draper_worker_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };
