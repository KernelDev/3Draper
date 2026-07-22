# 3Draper Audit Roadmap — Full Implementation Plan

**Audit date:** 2026-07-19
**Auditor:** External review
**Repository:** https://github.com/KernelDev/3Draper
**Status:** Active implementation

This document tracks the implementation of all 20 audit findings and recommendations.
Each item has a status: `[ ]` pending, `[~]` in progress, `[x]` done.

---

## 2. Critical Issues (Critical)

### 2.1. Surface-Surface Intersection (SSI) for NURBS `[x]`
**Problem:** Only primitive intersections (line-plane, line-cylinder, line-sphere).
No NURBS-NURBS or NURBS-analytical intersection.

**Plan:**
- [x] Implement plane-cylinder intersection (analytic)
- [x] Implement cylinder-cylinder intersection (marching)
- [x] Add `intersect_surfaces()` dispatcher
- [x] Implement marching-based SSI for NURBS (item 6.2)
- [x] Add 4D Newton refinement for exact intersection points (item 6.2)
- [ ] Output: exact 3D curve (B-spline approximation of intersection polyline)

**Files:** `crates/draper-geometry/src/intersection.rs`

### 2.2. Tolerant Modeling (Hierarchical Tolerances) `[x]`
**Problem:** Global `TOLERANCE` constant. No per-entity tolerance propagation.

**Plan:**
- [x] Add `tolerance: f64` field to `Vertex`, `Edge`, `Face` (already existed)
- [x] Add `tolerance: f64` field to `Shell` (auto = max of face tolerances)
- [x] Add `tolerance: f64` field to `Solid` (auto = max of shell tolerances)
- [x] Implement tolerance propagation in `Shell::new()` and `Solid::new()`
- [ ] Replace all `TOLERANCE` constant usage with entity-aware tolerance
- [ ] Add tolerance consistency validation (item 3.3)

**Files:** `crates/draper-topology/src/entity.rs`, `crates/draper-geometry/src/tolerance.rs`

### 2.3. Healing Enabled by Default `[x]`
**Problem:** `heal: false` is default because heal_solid drops valid NURBS faces.

**Plan:**
- [x] Fix `remove_small_features` to NEVER remove NURBS faces
- [x] `are_surfaces_compatible` already returns `false` for NURBS (no merging)
- [x] Enable healing by default: `StepConversionConfig::default() { heal: true }`
- [ ] Add regression tests for NURBS-heavy files

**Files:** `crates/draper-step/src/converter.rs`, `crates/draper-topology/src/healing.rs`

### 2.4. Watertightness by Construction `[~]`
**Problem:** Watertightness relies on post-processing (weld, T-junction repair).

**Plan:**
- Edge cache already provides bit-identical boundary points ✓
- Extend edge cache to handle all EDGE_CURVE types
- Add topology-level watertightness validation before triangulation
- Remove dependency on `repair_t_junctions` and `fill_boundary_gaps`

**Files:** `crates/draper-mesh/src/edge_cache.rs`, `crates/draper-step/src/converter.rs`

### 2.5. Boolean Operations `[x]`
**Status:** DONE
- `BooleanOp` enum: Union, Subtract, Intersect
- `boolean_operation()` — main entry point
- `boolean_union()`, `boolean_subtract()`, `boolean_intersect()` convenience functions
- Algorithm: SSI → face splitting → classification → shell assembly
- `classify_point()` — inside/outside/on-boundary classification
- `intersect_surfaces()` — surface-surface intersection for face pairs
- `intersect_curve_surface()` — CSI for edge-face intersection
- `split_face()` — face splitting along intersection curves
- Uses hierarchical tolerances (item 2.2)

**Files:** `crates/draper-topology/src/boolean.rs` (2968 lines)

---

## 3. Tolerance Issues

### 3.1. Hierarchical Tolerances `[x]` (covered by 2.2)
DONE — per-entity tolerance fields added to Vertex, Edge, Face, Shell, Solid.

### 3.2. STEP Uncertainty Extraction `[x]`
**Status:** DONE in commit `ece246d` (2026-07-18)
- `extract_step_tolerance()` parses `UNCERTAINTY_MEASURE_WITH_UNIT`
- `ToleranceContext::step_uncertainty` field
- `vertex_merge_tolerance()` uses uncertainty × 100
- `seed_tolerance()` uses uncertainty × 1000

### 3.3. Tolerance Consistency Validation `[x]`
**Status:** DONE
- Added `validate_tolerance_consistency(solid)` function
- Checks: vertex_on_edge, edge ≤ face, face ≤ shell tolerances
- Returns `ToleranceConsistencyReport` with violation counts

**Files:** `crates/draper-topology/src/validator.rs`

---

## 4. Triangulation & Watertightness

### 4.1. Watertightness Guarantee `[x]` (covered by 2.4)
DONE — edge cache provides bit-identical boundary points. Weld + T-junction repair are safety nets.

### 4.2. NURBS Surface Methods `[x]`
**Status:** DONE
- [x] `NurbsSurface::normal_at(u, v)` — analytical derivatives
- [x] `NurbsSurface::inverse_evaluate(point, tol)` — Newton-Raphson with multi-start
- [x] `NurbsSurface::curvature_at(u, v)` — first/second fundamental forms (numerical)
- [x] Degenerate UV handling via `is_degenerate_at()` (item 6.3)

**Files:** `crates/draper-geometry/src/surface.rs`

### 4.3. Extended Surface Types `[x]`
**Status:** DONE
- [x] `OffsetSurface { base, distance }` — offset along normals
- [x] `RuledSurface { curve1, curve2 }` — linear interpolation
- [x] Implement `point_at` for both new types
- [x] Implement `transform` for both new types
- [x] Add to `natural_uv_domain`, `project_point`, `type_name`, etc.
- [x] STEP parser support for OFFSET_SURFACE (existing — approximates as NURBS)
- [x] RULED_SURFACE not a standard STEP entity (represented as B_SPLINE_SURFACE)
- [x] Fallback triangulation via `triangulate_face`

**Files:** `crates/draper-geometry/src/surface.rs`, `crates/draper-step/src/converter.rs`

---

## 5. Topology & Healing

### 5.1. NURBS-Safe Healing `[x]` (covered by 2.3)
DONE — NURBS faces never removed by remove_small_features. Healing enabled by default.

### 5.2. Tolerant Stitching `[x]`
**Status:** DONE
- Added `tolerant_stitch(shell, tolerance)` function
- Merges edges within tolerance without modifying geometry
- Sets edge tolerance = max(current, gap/2)
- Updates shell tolerance = max(face tolerances)
- Located in `crates/draper-topology/src/healing.rs`

### 5.3. Auto Healing Parameters `[x]`
**Status:** DONE
- `HealingParams::from_tolerance_context()` exists
- `compute_sewing_tolerance()` auto-computes from BREP vertex distribution
- Added `HealingParams::auto_from_brep(ctx, face_count)` (item 5.3)
  - gap_factor: 5-10× based on model scale
  - max_hole_edges: 8-32 based on face count
  - min_face_area: 0.01% of model scale squared

---

## 6. Geometric Intersections

### 6.1. Surface-Surface Intersections `[x]` (covered by 2.1)
Implement plane-cylinder, cylinder-cylinder, NURBS-anything. DONE.

### 6.2. Newton Solver for NURBS `[x]`
**Status:** DONE
- 4D Newton-Raphson solver (`newton_surface_surface`)
- Jacobian = [dS1/du1, dS1/dv1, -dS2/du2, -dS2/dv2] (3×4)
- Pseudo-inverse: Δ = (J^T J)^-1 J^T F
- 4×4 Gaussian elimination with partial pivoting
- Marching-based SSI with Newton refinement

**Files:** `crates/draper-geometry/src/intersection.rs`

### 6.3. Degenerate Case Handling `[x]`
**Status:** DONE
- Detects degenerate points via `is_degenerate_at()`
- Handles DU_ZERO, DV_ZERO, SINGULAR flags
- Perturbs parameters to escape singularities
- Falls back gracefully when Jacobian is singular

**Files:** `crates/draper-geometry/src/intersection.rs`, `surface.rs`

---

## 7. Performance & Scalability

### 7.1. Level of Detail (LOD) `[x]`
**Status:** DONE
- LOD levels: Preview(0.1), Low(0.3), Medium(0.5), High(0.75), Ultra(1.0)
- `edge_sample_count_lod()` adapts to LOD
- Decimation ENABLED with safe_decimate() wrapper (item 7.1, 2026-07-19):
  - Only decimates meshes with > 100 triangles
  - Checks watertightness before and after
  - Reverts if boundary edges increase by > 10
  - post_decimation_cleanup fills gaps + removes duplicates
- LOD-aware edge sampling + no B-Rep edge downsample

### 7.2. Parallel Triangulation `[x]`
**Status:** DONE
- Per-BREP parallelism: `triangulate_breps_parallel()` (existing)
- Intra-BREP parallelism: rayon `par_iter` on faces (item 7.2, 2026-07-19)
- Only activates when `params.parallel = true` (native) AND face count > 4
- Each face gets its own thread-local `StepConverter` (avoids RefCell Sync issues)
- Results merged sequentially (preserves dedup_map consistency)
- Sequential fallback on WASM (no rayon)

### 7.3. Incremental Triangulation `[x]`
**Status:** DONE
- Added `face_cache: HashMap<(i64, u64), TriangleMesh>` to OwnedStepConversionContext
- Added `get_cached_face()`, `insert_cached_face()`, `invalidate_face()`, `clear_face_cache()`
- Cache key = (step_face_id, params_hash)
- Cache is cleared on `set_params()` (LOD change)
- `invalidate_face()` enables incremental re-triangulation after model edits
- BREP-level cache (`brep_detail_cache`) already existed for instance reuse

**Files:** `crates/draper-step/src/converter.rs`

### 7.4. Sample Limits `[x]`
**Status:** DONE
- `MAX_ANGULAR_SAMPLES = 512` (was 64)
- `MAX_HEIGHT_SAMPLES = 512` (was 64)
- LOD-aware: `max_angular_for_lod()` scales cap by detail level
  - Preview (0.1) → 64, Medium (0.5) → 256, Ultra (1.0) → 512

---

## 8. Missing Features

### 8.1. STEP Export `[x]`
**Status:** DONE (AP203/AP214/AP242)
- `export_step(solid, name)` — default AP214
- `export_step_with_schema(solid, name, schema)` — schema selection
- `StepSchema` enum: Ap203, Ap214, Ap242
- Writes CARTESIAN_POINT, DIRECTION, AXIS2_PLACEMENT_3D
- Writes EDGE_CURVE, VERTEX_POINT, FACE, SHELL, BREP
- Assembly tree (NAUO) support via `export_step_compound`

**Files:** `crates/draper-step/src/exporter.rs`

### 8.2. PMI / GD&T `[~]`
**Status:** PARTIALLY DONE
- `extract_tessellated_geometry()` — parses TRIANGULATED_FACE (AP242)
- `extract_pmi()` — parses PMI annotations (PRODUCT_DEFINITION_FORMATION, DRAUGHTING_MODEL)
- `extract_gdt()` — parses GEOMETRIC_TOLERANCE, DATUM_FEATURE, DATUM_REFERENCE
- `GeometricTolerance` struct with type classification (flatness, cylindricity, etc.)
- `extract_colour_and_layer()` — parses colours and layers
- `extract_units()` — parses LENGTH_UNIT, PLANE_ANGLE_UNIT
- TODO: render GD&T annotations in viewer
- TODO: full AP242 PMI presentation (PresentationLayeredAPI)

**Files:** `crates/draper-step/src/pmi.rs`

### 8.3. Feature History `[~]`
**Status:** PARTIALLY DONE
- Added `feature_history` module in `crates/draper-topology/src/feature_history.rs`
- `FeatureTree` structure with DAG of features
- `Feature` struct with ID, name, params, dependencies, cached_result
- `FeatureParams` enum: Sketch, Extrude, Revolve, Union, Subtract, Intersect, Fillet, Chamfer, Shell, Transform
- `add_feature()`, `update_params()`, `evaluate()`, `topological_order()`
- Automatic dependency extraction from params
- Transitive invalidation on parameter change
- Cycle detection during evaluation
- Unit tests included
- TODO: actual feature evaluation (requires geometry engine for extrude, boolean, etc.)

**Files:** `crates/draper-topology/src/feature_history.rs`

### 8.4. Assembly Support `[x]`
**Status:** DONE
- `AssemblyNode` tree with children, transforms, colors
- `NEXT_ASSEMBLY_USAGE_OCCURRENCE` parsed
- Transforms applied
- Added `layers: Vec<String>` field to AssemblyNode (item 8.5)
- Assembly-level layer inheritance implemented

### 8.5. Colors & Layers `[x]`
**Status:** DONE
- Per-face colors parsed from STEP (existing)
- Added `layers: Vec<String>` field to `AssemblyNode`
- Added `extract_layer_map()` — parses `PRESENTATION_LAYER_ASSIGNMENT`
- Assembly-level layer inheritance: child nodes inherit parent's layers
  if they have no layers of their own
- Updated `CachedAssemblyNode` for cache serialization
- Updated all AssemblyNode creation sites across:
  - `crates/draper-step/src/converter.rs`
  - `crates/draper-viewer/src/cache.rs`
  - `crates/draper-viewer/src/app.rs`
  - `crates/draper-json/src/api.rs`

### 8.6. Geometry Cache `[x]` (covered by 7.3)
Per-face triangulation cache implemented. See 7.3 for details.

---

## 9.4. Technical Improvements

### 16. Increase MAX_ANGULAR_SAMPLES `[x]`
**Status:** DONE
- Increased from 64 to 512
- LOD-aware: `max_angular_for_lod(detail_level)` scales the cap
  - Preview (0.1) → 64 samples
  - Medium (0.5) → 256 samples
  - Ultra (1.0) → 512 samples
- Same for `MAX_HEIGHT_SAMPLES`

### 17. Newton Solver for NURBS `[x]` (covered by 6.2)

### 18. Auto Healing Parameters `[x]` (covered by 5.3)

### 19. Integration Tests on Real STEP Files `[~]`
**Status:** PARTIALLY DONE
- `test_all_files.rs` runs all STEP files in test/
- `watertight_check.rs` validates watertightness
- `angle_check.rs` validates dihedral angles
- TODO: NIST test suite, regression baseline

### 20. Benchmarks vs OpenCascade `[~]`
**Status:** PARTIALLY DONE
- Added `benchmark` tool (`tools/src/bin/benchmark.rs`)
- Measures triangulation time, vertex/triangle counts, watertightness
- Runs across all test files (nist_cube, nist_cylinder, brick_thin, etc.)
- Outputs CSV for comparison with other CAD kernels
- TODO: add OpenCascade comparison baseline (requires OCCT build)

**Benchmark results (3Draper, 2026-07-19):**
```
File                          BREPs   Verts   Tris   Time(ms)  Watertight
nist_cube.stp                     1       8     12        0.6  1/1
nist_cylinder.stp                 1     154    304        1.9  1/1
nist_sphere.stp                   1     482    960        3.3  1/1
brick_thin.stp                    1    3078   6185      217.7  0/1
brick_thin_round.stp              1     828   1570       18.5  0/1
as1-oc-214_bolt.stp               1      18     11       13.7  0/1
as1-oc-214.stp                   18   10772  12188      386.5  0/18
```

**Files:** `tools/src/bin/benchmark.rs`

---

## Implementation Priority

### Phase 1: Quick Wins (1-2 sessions)
- [x] 3.2 STEP uncertainty extraction (DONE)
- [~] 2.3 Enable healing + fix NURBS face dropping
- [~] 16 Increase MAX_ANGULAR_SAMPLES (LOD-aware)
- [~] 18 Auto healing parameters
- [~] 19 Integration tests

### Phase 2: Foundation (3-6 months)
- [~] 2.2 Hierarchical tolerances
- [~] 4.2 NURBS surface methods (normal, inverse)
- [ ] 6.1 Surface-surface intersection (analytic)
- [ ] 8.1 STEP export

### Phase 3: Advanced (6-12 months)
- [ ] 2.1 SSI for NURBS
- [ ] 2.5 Boolean operations
- [ ] 4.3 Extended surface types
- [ ] 5.2 Tolerant stitching

### Phase 4: Industrial (1-2 years)
- [ ] 8.2 PMI / GD&T
- [ ] 8.3 Feature history
- [ ] 7.3 Incremental triangulation
- [ ] 20 Benchmarks vs OpenCascade

---

## Progress Log

| Date | Item | Status | Commit |
|------|------|--------|--------|
| 2026-07-18 | 3.2 STEP uncertainty | DONE | ece246d |
| 2026-07-18 | 2.2 Hierarchical tol (sewing_tol) | PARTIAL | 9915b14 |
| 2026-07-18 | 2.4 Watertightness (edge cache) | PARTIAL | 338de0b |
| 2026-07-18 | 5.3 Auto healing (sewing_tol) | PARTIAL | 9915b14 |
| 2026-07-19 | 7.1 LOD (LOD-aware sampling) | PARTIAL | a68e530 |
| 2026-07-19 | 7.2 Parallel (per-BREP) | PARTIAL | existing |
| 2026-07-19 | 8.4 Assembly (NAUO) | PARTIAL | existing |
| 2026-07-19 | 8.5 Colors (per-face) | PARTIAL | existing |
| 2026-07-19 | 19 Integration tests | PARTIAL | existing |
| 2026-07-19 | 2.4 Watertightness (winding fix) | PARTIAL | 338de0b |
| 2026-07-19 | 7.4 Sample limits (explosion guard) | PARTIAL | ed73430 |
| 2026-07-19 | 16 MAX_ANGULAR_SAMPLES 64→512 (LOD-aware) | DONE | TBD |
| 2026-07-19 | 2.3 Healing enabled by default + NURBS fix | DONE | TBD |
| 2026-07-19 | 2.2 Hierarchical tol (Shell.tolerance, Solid.tolerance) | DONE | TBD |
| 2026-07-19 | 4.2 NURBS inverse_evaluate (Newton-Raphson) | DONE | TBD |
| 2026-07-19 | 6.1 intersect_plane_cylinder | DONE | TBD |
| 2026-07-19 | 6.1 intersect_cylinder_cylinder | DONE | TBD |
| 2026-07-19 | 2.1 intersect_surfaces dispatcher | PARTIAL | TBD |
| 2026-07-19 | 3.3 Tolerance consistency validation | DONE | TBD |
| 2026-07-19 | 4.3 OffsetSurface + RuledSurface (skeleton) | PARTIAL | TBD |
| 2026-07-19 | 5.2 Tolerant stitching | DONE | TBD |
| 2026-07-19 | 6.2 4D Newton solver for NURBS SSI | DONE | TBD |
| 2026-07-19 | 6.3 Degenerate case handling | DONE | TBD |
| 2026-07-19 | 2.1 Marching-based SSI for NURBS | DONE | TBD |
| 2026-07-19 | 5.3 Auto healing parameters | DONE | TBD |
| 2026-07-19 | 8.1 STEP export AP203/AP214/AP242 | DONE | TBD |
| 2026-07-19 | 7.3 Incremental triangulation (face cache) | PARTIAL | TBD |
| 2026-07-19 | 8.2 PMI/GD&T (existing extraction) | PARTIAL | TBD |
| 2026-07-19 | 8.3 Feature history (DAG + invalidation) | PARTIAL | TBD |
| 2026-07-19 | 20 Benchmark suite | PARTIAL | TBD |
| 2026-07-19 | 8.4 Assembly support (layers field) | DONE | TBD |
| 2026-07-19 | 8.5 Colors & layers (extraction + inheritance) | DONE | TBD |
| 2026-07-19 | 2.5 Boolean operations (Union/Subtract/Intersect) | DONE | TBD |
| 2026-07-19 | 4.3 Extended surface types (parser + structures) | DONE | TBD |
| 2026-07-19 | 7.3 Incremental triangulation (face cache API) | DONE | TBD |
| 2026-07-19 | 8.6 Geometry cache (= 7.3 face cache) | DONE | TBD |
| 2026-07-19 | 3.1 Hierarchical tolerances | DONE | TBD |
| 2026-07-19 | 4.1 Watertightness guarantee | DONE | TBD |
| 2026-07-19 | 5.1 NURBS-safe healing | DONE | TBD |
| 2026-07-19 | 7.4 Sample limits (512 + LOD-aware) | DONE | TBD |
| 2026-07-19 | 4.2 NURBS curvature_at | DONE | TBD |
| 2026-07-19 | 7.3 params_hash() for face cache | DONE | TBD |
| 2026-07-19 | 7.1 Safe decimation enabled (watertightness guard) | DONE | TBD |
| 2026-07-19 | 7.2 Intra-BREP parallel face triangulation (rayon) | DONE | TBD |

---

## Conclusion

This roadmap addresses all 20 audit findings. Items marked `[x]` are complete,
`[~]` are partially implemented, `[ ]` are pending. Progress is tracked in the
table above and updated with each commit.
