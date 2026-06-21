// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//
// JavaScript bindings for the 3Draper kernel.
//
// Two integration paths are supported:
//
// 1. **WASM** — when the 3Draper library is compiled to WebAssembly via
//    `wasm-bindgen` (see the `draper-wasm` crate). This is the
//    recommended path for browser use; all kernel features are exposed
//    through the `DraperDocument`, `Mesh`, `GdtResult`, and `GdtType`
//    classes below.
//
// 2. **Native addon** — when the shared library is loaded as a Node.js
//    native addon via N-API or `ffi-napi`. The same API surface is
//    exposed; the WASM module is replaced by C FFI calls.
//
// In both cases, call `init(wasmModule)` once before using any class.

"use strict";

// ============================================================
// Error codes (must match DraperResult in draper-ffi/src/lib.rs)
// ============================================================

const DraperResult = Object.freeze({
  SUCCESS:              0,
  INVALID_ARGUMENT:    -1,
  FILE_NOT_FOUND:      -2,
  PARSE_ERROR:         -3,
  GEOMETRY_ERROR:      -4,
  TOPOLOGY_ERROR:      -5,
  TRIANGULATION_ERROR: -6,
  OUT_OF_MEMORY:       -7,
  UNKNOWN_ERROR:      -99,
});

const RESULT_MESSAGES = {
  [DraperResult.SUCCESS]:              "Success",
  [DraperResult.INVALID_ARGUMENT]:     "Invalid argument",
  [DraperResult.FILE_NOT_FOUND]:      "File not found",
  [DraperResult.PARSE_ERROR]:         "Parse error",
  [DraperResult.GEOMETRY_ERROR]:      "Geometry error",
  [DraperResult.TOPOLOGY_ERROR]:      "Topology error",
  [DraperResult.TRIANGULATION_ERROR]: "Triangulation error",
  [DraperResult.OUT_OF_MEMORY]:       "Out of memory",
  [DraperResult.UNKNOWN_ERROR]:       "Unknown error",
};

class DraperError extends Error {
  constructor(code, message) {
    const msg = message || RESULT_MESSAGES[code] || `Unknown code: ${code}`;
    super(`DraperError(${code}): ${msg}`);
    this.name = "DraperError";
    this.code = code;
  }
}

// ============================================================
// GDT type enum (mirrors draper_mesh::gdt_check::GdtCheckType)
// ============================================================

const GdtType = Object.freeze({
  FLATNESS:          0,
  STRAIGHTNESS:      1,
  CIRCULARITY:       2,
  CYLINDRICITY:      3,
  POSITION:          4,
  PARALLELISM:       5,
  PERPENDICULARITY:  6,
  ANGULARITY:        7,
  RUNOUT:            8,
  PROFILE_OF_LINE:   9,
  PROFILE_OF_SURFACE: 10,
});

// ============================================================
// WASM module loader
// ============================================================

let _wasm = null;

/**
 * Initialise the WASM module.
 *
 * Pass the wasm-bindgen-generated module object (the default export of
 * `draper_wasm.js`). After this resolves, all classes below are usable.
 *
 * @param {object} wasmModule - The wasm-bindgen-generated init object.
 * @returns {Promise<void>}
 */
async function init(wasmModule) {
  if (typeof wasmModule === "function") {
    // wasm-bindgen style: default export is an async init function.
    _wasm = await wasmModule();
  } else if (wasmModule && wasmModule.DraperDocument) {
    // Already-initialized module.
    _wasm = wasmModule;
  } else {
    throw new DraperError(
      DraperResult.INVALID_ARGUMENT,
      "init: expected a wasm-bindgen module or init function"
    );
  }
}

function _requireWasm() {
  if (!_wasm) {
    throw new DraperError(
      DraperResult.UNKNOWN_ERROR,
      "WASM module not loaded — call init() first"
    );
  }
  return _wasm;
}

// ============================================================
// Version & feature detection
// ============================================================

/**
 * Returns the library version as a string (e.g. "0.1.0").
 * @returns {string}
 */
function version() {
  if (_wasm && _wasm.version) {
    return _wasm.version();
  }
  return "0.1.0";
}

/**
 * Returns the library version as [major, minor, patch].
 * @returns {number[]}
 */
function versionTuple() {
  const v = version().split(".").map((n) => parseInt(n, 10) || 0);
  while (v.length < 3) v.push(0);
  return v.slice(0, 3);
}

/**
 * Check whether the library supports a named feature.
 *
 * Feature names: step_import, step_export, stl_export, gltf_export,
 * obj_export, 3mf_export, usda_export, boolean_ops, healing,
 * validation, analytical_queries, bvh, editing, gdt_checks, modeling,
 * patterns.
 *
 * @param {string} feature
 * @returns {boolean}
 */
function hasFeature(feature) {
  if (_wasm && _wasm.has_feature) {
    return _wasm.has_feature(feature);
  }
  const known = [
    "step_import", "step_export", "stl_export", "gltf_export",
    "obj_export", "3mf_export", "usda_export", "boolean_ops",
    "healing", "validation", "analytical_queries", "bvh",
    "editing", "gdt_checks", "modeling", "patterns",
  ];
  return known.includes(feature);
}

// ============================================================
// Mesh class — wraps draper_wasm::Mesh
// ============================================================

class Mesh {
  constructor(handle) {
    this._handle = handle;
  }

  /** Number of vertices. */
  get vertexCount() {
    return this._handle ? this._handle.vertex_count() : 0;
  }

  /** Number of triangles. */
  get triangleCount() {
    return this._handle ? this._handle.triangle_count() : 0;
  }

  /**
   * Get vertex positions as a Float32Array (x,y,z triplets).
   * @returns {Float32Array}
   */
  getVertices() {
    return this._handle ? this._handle.vertices() : new Float32Array(0);
  }

  /**
   * Get triangle indices as a Uint32Array (i,j,k triplets).
   * @returns {Uint32Array}
   */
  getTriangles() {
    return this._handle ? this._handle.triangles() : new Uint32Array(0);
  }

  /**
   * Get per-vertex normals as a Float32Array, or empty array if not present.
   * @returns {Float32Array}
   */
  getNormals() {
    return this._handle ? this._handle.normals() : new Float32Array(0);
  }

  /**
   * Get per-triangle RGBA colors as a Float32Array (4 per triangle).
   * @returns {Float32Array}
   */
  getColors() {
    return this._handle ? this._handle.colors() : new Float32Array(0);
  }

  /**
   * Export mesh to binary STL. Returns a Uint8Array.
   * @returns {Uint8Array}
   */
  exportStlBinary() {
    return this._handle ? this._handle.export_stl_binary() : new Uint8Array(0);
  }

  /**
   * Export mesh to ASCII STL. Returns a UTF-8 string.
   * @returns {string}
   */
  exportStlAscii() {
    return this._handle ? this._handle.export_stl_ascii() : "";
  }

  /**
   * Export mesh to binary glTF (GLB). Returns a Uint8Array.
   * @returns {Promise<Uint8Array>}
   */
  async exportGltf() {
    if (!this._handle) return new Uint8Array(0);
    return await this._handle.export_gltf();
  }

  /**
   * Export mesh to Wavefront OBJ. Returns a UTF-8 string.
   * @returns {string}
   */
  exportObj() {
    return this._handle ? this._handle.export_obj() : "";
  }

  /**
   * Export mesh to 3MF. Returns a Uint8Array.
   * @returns {Promise<Uint8Array>}
   */
  async export3mf() {
    if (!this._handle) return new Uint8Array(0);
    return await this._handle.export_3mf();
  }

  /** Free the underlying handle. */
  free() {
    if (this._handle) {
      this._handle.free();
      this._handle = null;
    }
  }
}

// ============================================================
// GdtResult class — wraps draper_wasm::GdtResult
// ============================================================

class GdtResult {
  constructor(handle) {
    this._handle = handle;
  }

  get toleranceValue() { return this._handle ? this._handle.tolerance_value : 0; }
  get actualDeviation() { return this._handle ? this._handle.actual_deviation : NaN; }
  get passed() { return this._handle ? this._handle.passed : false; }
  get status() { return this._handle ? this._handle.status() : "FAIL"; }

  free() {
    if (this._handle) {
      this._handle.free();
      this._handle = null;
    }
  }
}

// ============================================================
// Document class — wraps draper_wasm::DraperDocument
// ============================================================

class Document {
  /**
   * Create a new 3Draper document.
   * @param {string} [name="Untitled"]
   */
  constructor(name = "Untitled") {
    const wasm = _requireWasm();
    this._handle = new wasm.DraperDocument(name);
  }

  // ----------------------------------------------------------
  // Properties
  // ----------------------------------------------------------

  /** Number of solids in the document. */
  get solidCount() {
    return this._handle ? this._handle.solid_count() : 0;
  }

  // ----------------------------------------------------------
  // Shape builders
  // ----------------------------------------------------------

  /**
   * Add a box primitive. Returns the new solid's index.
   * @param {number} dx
   * @param {number} dy
   * @param {number} dz
   * @returns {number}
   */
  addBox(dx, dy, dz) {
    return this._handle.add_box(dx, dy, dz);
  }

  /**
   * Add a cylinder primitive. Returns the new solid's index.
   * @param {number} radius
   * @param {number} height
   * @returns {number}
   */
  addCylinder(radius, height) {
    return this._handle.add_cylinder(radius, height);
  }

  /**
   * Add a sphere primitive. Returns the new solid's index.
   * @param {number} radius
   * @returns {number}
   */
  addSphere(radius) {
    return this._handle.add_sphere(radius);
  }

  /**
   * Add a cone primitive. Returns the new solid's index.
   * @param {number} radius
   * @param {number} height
   * @param {number} halfAngle
   * @returns {number}
   */
  addCone(radius, height, halfAngle) {
    return this._handle.add_cone(radius, height, halfAngle);
  }

  /**
   * Add a torus primitive. Returns the new solid's index.
   * @param {number} majorRadius
   * @param {number} minorRadius
   * @returns {number}
   */
  addTorus(majorRadius, minorRadius) {
    return this._handle.add_torus(majorRadius, minorRadius);
  }

  /**
   * Load a STEP file from text content. Appends all solids.
   * @param {string} content
   * @returns {Promise<number>}
   */
  async loadStep(content) {
    return await this._handle.load_step(content);
  }

  // ----------------------------------------------------------
  // Editing operations
  // ----------------------------------------------------------

  /**
   * Fillet (round) an edge of a solid.
   * @param {number} solidIndex
   * @param {number} edgeIndex - TopoId of the edge (0 = auto-pick first manifold edge).
   * @param {number} radius - Fillet radius in mm.
   */
  async filletEdge(solidIndex, edgeIndex, radius) {
    await this._handle.fillet_edge(solidIndex, edgeIndex, radius);
  }

  /**
   * Chamfer (bevel) an edge of a solid.
   * @param {number} solidIndex
   * @param {number} edgeIndex - TopoId of the edge (0 = auto-pick).
   * @param {number} distance - Chamfer distance in mm.
   */
  async chamferEdge(solidIndex, edgeIndex, distance) {
    await this._handle.chamfer_edge(solidIndex, edgeIndex, distance);
  }

  /**
   * Shell a solid (inward offset by `thickness`).
   * @param {number} solidIndex
   * @param {number} thickness
   */
  async makeShell(solidIndex, thickness) {
    await this._handle.make_shell(solidIndex, thickness);
  }

  // ----------------------------------------------------------
  // Transform operations
  // ----------------------------------------------------------

  /**
   * Translate a solid by (dx, dy, dz).
   */
  async translate(solidIndex, dx, dy, dz) {
    await this._handle.translate(solidIndex, dx, dy, dz);
  }

  /**
   * Rotate a solid about (ax, ay, az) by `angleRadians`.
   */
  async rotate(solidIndex, ax, ay, az, angleRadians) {
    await this._handle.rotate(solidIndex, ax, ay, az, angleRadians);
  }

  /**
   * Uniformly scale a solid by `factor`.
   */
  async scale(solidIndex, factor) {
    await this._handle.scale(solidIndex, factor);
  }

  /**
   * Mirror a solid about the plane through (ox,oy,oz) with normal (nx,ny,nz).
   * Replaces the solid in-place with its mirror image.
   */
  async mirror(solidIndex, ox, oy, oz, nx, ny, nz) {
    await this._handle.mirror(solidIndex, ox, oy, oz, nx, ny, nz);
  }

  // ----------------------------------------------------------
  // Pattern operations
  // ----------------------------------------------------------

  /**
   * Create a circular pattern: `count` copies of the solid, evenly
   * spaced around (ax, ay, az). Copies are appended.
   * @returns {Promise<number>} number of copies added
   */
  async circularPattern(solidIndex, count, ax, ay, az) {
    return await this._handle.circular_pattern(solidIndex, count, ax, ay, az);
  }

  /**
   * Create a linear pattern: `count` copies of the solid, each translated
   * by `step` along (dx, dy, dz). Copies are appended.
   * @returns {Promise<number>} number of copies added
   */
  async linearPattern(solidIndex, count, dx, dy, dz, step) {
    return await this._handle.linear_pattern(solidIndex, count, dx, dy, dz, step);
  }

  // ----------------------------------------------------------
  // Boolean operations
  // ----------------------------------------------------------

  /**
   * Boolean union of two solids. Returns the index of the new solid.
   * @returns {Promise<number>}
   */
  async booleanUnion(aIndex, bIndex) {
    return await this._handle.boolean_union(aIndex, bIndex);
  }

  /**
   * Boolean subtract (A - B). Returns the index of the new solid.
   * @returns {Promise<number>}
   */
  async booleanSubtract(aIndex, bIndex) {
    return await this._handle.boolean_subtract(aIndex, bIndex);
  }

  /**
   * Boolean intersect (A ∩ B). Returns the index of the new solid.
   * @returns {Promise<number>}
   */
  async booleanIntersect(aIndex, bIndex) {
    return await this._handle.boolean_intersect(aIndex, bIndex);
  }

  /**
   * Delete a solid by index.
   */
  async deleteSolid(index) {
    await this._handle.delete_solid(index);
  }

  // ----------------------------------------------------------
  // Holes
  // ----------------------------------------------------------

  /**
   * Add a circular hole of `radius` mm centered at (cx, cy, cz) on the
   * face at `faceIndex` of solid `solidIndex`.
   */
  async addCircularHole(solidIndex, faceIndex, cx, cy, cz, radius) {
    await this._handle.add_circular_hole(solidIndex, faceIndex, cx, cy, cz, radius);
  }

  // ----------------------------------------------------------
  // GDT checks
  // ----------------------------------------------------------

  /**
   * Run a single GDT check on the mesh of solid `solidIndex`.
   * @param {number} solidIndex
   * @param {number} checkType - One of GdtType.* constants.
   * @param {number} toleranceValue - Tolerance in mm.
   * @param {number[]|null} [datumAxis] - Optional [x,y,z] datum axis.
   * @param {number[]|null} [nominalPosition] - Optional [x,y,z] nominal position.
   * @param {number|null} [nominalAngleDeg] - Optional nominal angle (for angularity).
   * @returns {Promise<GdtResult>}
   */
  async gdtCheck(solidIndex, checkType, toleranceValue, datumAxis, nominalPosition, nominalAngleDeg) {
    const handle = await this._handle.gdt_check(
      solidIndex,
      checkType,
      toleranceValue,
      datumAxis ? arrayToJs(datumAxis) : null,
      nominalPosition ? arrayToJs(nominalPosition) : null,
      nominalAngleDeg ?? null,
    );
    return new GdtResult(handle);
  }

  /**
   * Run all GDT checks specified as a JSON array. Each entry is an object:
   * `{ "type": "flatness", "value": 0.05, "datum_axis": [x,y,z], ... }`.
   * @param {number} solidIndex
   * @param {string} jsonSpecs
   * @returns {Promise<string>} JSON results string
   */
  async gdtCheckAll(solidIndex, jsonSpecs) {
    return await this._handle.gdt_check_all(solidIndex, jsonSpecs);
  }

  // ----------------------------------------------------------
  // Edge listing
  // ----------------------------------------------------------

  /**
   * List all edges in a solid as a JSON array.
   * Each entry: `{ "id": N, "curve_type": "Line"|"Circle"|..., "face_ids": [a, b] }`.
   * @param {number} solidIndex
   * @returns {Promise<object[]>}
   */
  async listEdges(solidIndex) {
    const json = await this._handle.list_edges(solidIndex);
    return JSON.parse(json);
  }

  // ----------------------------------------------------------
  // Triangulation & analysis
  // ----------------------------------------------------------

  /**
   * Triangulate the document and return a Mesh object.
   * @returns {Mesh}
   */
  triangulate() {
    const handle = this._handle.triangulate();
    return new Mesh(handle);
  }

  /**
   * Compute the axis-aligned bounding box.
   * @returns {Promise<Float64Array>} 6-element array: [min_x, min_y, min_z, max_x, max_y, max_z]
   */
  async boundingBox() {
    return await this._handle.bounding_box();
  }

  /** Compute total volume of all solids in mm³. */
  volume() {
    return this._handle ? this._handle.volume() : 0.0;
  }

  /** Compute total surface area of all solids in mm². */
  surfaceArea() {
    return this._handle ? this._handle.surface_area() : 0.0;
  }

  // ----------------------------------------------------------
  // STEP → USDA
  // ----------------------------------------------------------

  /**
   * Convert STEP text content to USDA (USD ASCII) text.
   * Static method — does not require a document instance.
   * @param {string} content - STEP file text.
   * @param {number} chordTolerance - Triangulation chord error in mm.
   * @param {boolean} smoothNormals - Whether to compute smooth vertex normals.
   * @returns {Promise<string>} USDA text
   */
  static async stepToUsda(content, chordTolerance, smoothNormals) {
    const wasm = _requireWasm();
    return await wasm.DraperDocument.step_to_usda(content, chordTolerance, smoothNormals);
  }

  // ----------------------------------------------------------
  // Cleanup
  // ----------------------------------------------------------

  /** Free the underlying handle. */
  free() {
    if (this._handle) {
      this._handle.free();
      this._handle = null;
    }
  }
}

/** Helper: convert a JS array to a wasm-bindgen JsArray (js_sys::Array). */
function arrayToJs(arr) {
  // wasm-bindgen accepts plain JS arrays directly via Option<Array>.
  return arr;
}

// ============================================================
// Exports
// ============================================================

module.exports = {
  // Classes
  Document,
  Mesh,
  GdtResult,

  // Enums / constants
  DraperResult,
  GdtType,

  // Functions
  init,
  version,
  versionTuple,
  hasFeature,
};
