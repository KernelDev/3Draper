/* tslint:disable */
/* eslint-disable */

/**
 * Result of `worker_parse_step` — contains file statistics and pending BREP descriptors.
 */
export class WorkerParseResult {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    readonly assembly_tree_json: string;
    readonly brep_count: number;
    readonly entity_count: number;
    readonly error: string;
    readonly face_count: number;
    readonly pending_breps_json: string;
    readonly shell_count: number;
}

/**
 * Result of `worker_triangulate_brep` — contains mesh data for one BREP.
 */
export class WorkerTriangulateResult {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    readonly brep_id: bigint;
    readonly color: Float32Array;
    readonly colors: Float32Array;
    readonly error: string;
    readonly face_normals: Float32Array;
    readonly indices: Uint32Array;
    readonly name: string;
    readonly normals: Float32Array;
    readonly remaining: number;
    readonly vertices: Float32Array;
}

/**
 * This is the entry point for the web version.
 * It is called automatically when the wasm module is loaded.
 */
export function start(): Promise<void>;

/**
 * Cancel all pending work in the worker and release resources.
 */
export function worker_cancel(): void;

/**
 * Get the assembly tree JSON from the worker state.
 * Called after `worker_parse_step` to retrieve the tree.
 */
export function worker_get_assembly_tree(): string;

/**
 * Parse a STEP file in the worker thread.
 *
 * This is called from `worker.js` when the main thread sends a `parse` message.
 * The STEP file text is parsed into a `StepFile`, and the assembly tree +
 * pending BREP instances are extracted (but no triangulation happens yet).
 *
 * Returns a `WorkerParseResult` with file statistics and pending BREP descriptors.
 * The parsed state is stored in thread-local storage for subsequent
 * `worker_triangulate_brep` calls.
 */
export function worker_parse_step(content: string, name: string, lod: number, is_mobile: boolean): WorkerParseResult;

/**
 * Triangulate the next pending BREP in the worker thread.
 *
 * This is called from `worker.js` when the main thread sends a `triangulate_next`
 * message. It processes one BREP at a time, returning the mesh data.
 *
 * If no more BREPs are pending, returns a result with empty vertices/indices
 * and `remaining = 0`.
 */
export function worker_triangulate_brep(): WorkerTriangulateResult;

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly start: () => void;
    readonly __wbg_workerparseresult_free: (a: number, b: number) => void;
    readonly __wbg_workertriangulateresult_free: (a: number, b: number) => void;
    readonly worker_cancel: () => void;
    readonly worker_get_assembly_tree: () => [number, number];
    readonly worker_parse_step: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly worker_triangulate_brep: () => number;
    readonly workerparseresult_assembly_tree_json: (a: number) => [number, number];
    readonly workerparseresult_brep_count: (a: number) => number;
    readonly workerparseresult_entity_count: (a: number) => number;
    readonly workerparseresult_error: (a: number) => [number, number];
    readonly workerparseresult_face_count: (a: number) => number;
    readonly workerparseresult_pending_breps_json: (a: number) => [number, number];
    readonly workerparseresult_shell_count: (a: number) => number;
    readonly workertriangulateresult_brep_id: (a: number) => bigint;
    readonly workertriangulateresult_color: (a: number) => [number, number];
    readonly workertriangulateresult_colors: (a: number) => [number, number];
    readonly workertriangulateresult_error: (a: number) => [number, number];
    readonly workertriangulateresult_face_normals: (a: number) => [number, number];
    readonly workertriangulateresult_indices: (a: number) => [number, number];
    readonly workertriangulateresult_name: (a: number) => [number, number];
    readonly workertriangulateresult_normals: (a: number) => [number, number];
    readonly workertriangulateresult_remaining: (a: number) => number;
    readonly workertriangulateresult_vertices: (a: number) => [number, number];
    readonly main: (a: number, b: number) => number;
    readonly wasm_bindgen__convert__closures_____invoke__h925e8d6377294b5d: (a: number, b: number, c: any) => [number, number];
    readonly wasm_bindgen__convert__closures_____invoke__h02c6bc4fe7ddc8b1: (a: number, b: number, c: any) => void;
    readonly wasm_bindgen__convert__closures_____invoke__hc904ec35267f910e: (a: number, b: number, c: any) => void;
    readonly wasm_bindgen__convert__closures_____invoke__hc904ec35267f910e_3: (a: number, b: number, c: any) => void;
    readonly wasm_bindgen__convert__closures_____invoke__hbca3773cac796aac: (a: number, b: number) => [number, number];
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __externref_table_alloc: () => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_exn_store: (a: number) => void;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_destroy_closure: (a: number, b: number) => void;
    readonly __externref_table_dealloc: (a: number) => void;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
