# 3Draper Audit Roadmap — Full Implementation Plan

**Audit date:** 2026-07-19
**Auditor:** External review
**Repository:** https://github.com/KernelDev/3Draper
**Status:** Active implementation

This document tracks the implementation of all 20 audit findings and recommendations.
Each item has a status: `[ ]` pending, `[~]` in progress, `[x]` done.

---

## 2. Critical Issues (Critical)

### 2.1. Surface-Surface Intersection (SSI) for NURBS `[~]`
**Problem:** Only primitive intersections (line-plane, line-cylinder, line-sphere).
No NURBS-NURBS or NURBS-analytical intersection.

**Plan:**
- [x] Implement plane-cylinder intersection (analytic)
- [x] Implement cylinder-cylinder intersection (marching)
- [x] Add `intersect_surfaces()` dispatcher
- [ ] Implement marching-squares SSI for NURBS (Bezier decomposition)
- [ ] Add 4D Newton refinement for exact intersection points
- [ ] Output: exact 3D curve (B-spline approximation)

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

### 2.5. Boolean Operations `[ ]`
**Problem:** Only classification skeleton in `boolean.rs`. No Union/Subtract/Intersect.

**Plan:**
- Implement SSI (prerequisite — see 2.1)
- Implement CSI (curve-surface intersection)
- Implement face splitting along intersection curves
- Implement point classification (inside/outside/on-boundary)
- Implement Union, Subtract, Intersect for solids

**Files:** `crates/draper-topology/src/boolean.rs`

---

## 3. Tolerance Issues

### 3.1. Hierarchical Tolerances `[~]` (covered by 2.2)
Add per-entity tolerance fields. See 2.2 for details.

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

### 4.1. Watertightness Guarantee `[~]` (covered by 2.4)
Edge cache provides bit-identical boundary points. Ongoing work to extend coverage.

### 4.2. NURBS Surface Methods `[x]`
**Problem:** No normal_at, inverse (XYZ→UV), curvature for NURBS.

**Plan:**
- [x] `NurbsSurface::normal_at(u, v)` already existed (uses analytical derivatives)
- [x] Implement `NurbsSurface::inverse_evaluate(point, tol)` — Newton-Raphson with multi-start
- [ ] Implement `NurbsSurface::curvature(u, v)` — first/second fundamental forms
- [ ] Add degenerate UV handling (poles, seam edges)

**Files:** `crates/draper-geometry/src/surface.rs`

### 4.3. Extended Surface Types `[~]`
**Problem:** No OFFSET_SURFACE, generalized SWEPT_SURFACE, RULED_SURFACE, COMPOSITE_SURFACE.

**Plan:**
- [x] Add `OffsetSurface { base, distance }` — offset along normals
- [x] Add `RuledSurface { curve1, curve2 }` — linear interpolation
- [x] Implement `point_at` for both new types
- [x] Implement `transform` for both new types
- [x] Add to `natural_uv_domain`, `project_point`, `type_name`, etc.
- [ ] Add STEP parser support for OFFSET_SURFACE, RULED_SURFACE
- [ ] Implement specialized triangulators
- [ ] Add COMPOSITE_SURFACE

**Files:** `crates/draper-geometry/src/surface.rs`

---

## 5. Topology & Healing

### 5.1. NURBS-Safe Healing `[~]` (covered by 2.3)
Fix NURBS face dropping. See 2.3 for details.

### 5.2. Tolerant Stitching `[x]`
**Status:** DONE
- Added `tolerant_stitch(shell, tolerance)` function
- Merges edges within tolerance without modifying geometry
- Sets edge tolerance = max(current, gap/2)
- Updates shell tolerance = max(face tolerances)
- Located in `crates/draper-topology/src/healing.rs`

### 5.3. Auto Healing Parameters `[~]`
**Status:** PARTIALLY DONE
- `HealingParams::from_tolerance_context()` exists
- `compute_sewing_tolerance()` auto-computes from BREP vertex distribution
- TODO: auto-compute gap_factor, max_hole_edges from model scale

---

## 6. Geometric Intersections

### 6.1. Surface-Surface Intersections `[ ]` (covered by 2.1)
Implement plane-cylinder, cylinder-cylinder, NURBS-anything.

### 6.2. Newton Solver for NURBS `[ ]`
**Plan:**
- Implement 4D Newton-Raphson for surface-surface intersection
- Use Jacobian (first fundamental form) for step direction
- Multi-start to avoid local minima
- Convergence criteria: |F(u1,v1,u2,v2)| < tol

**Files:** `crates/draper-geometry/src/intersection.rs`

### 6.3. Degenerate Case Handling `[ ]`
**Plan:**
- Handle DU_ZERO, DV_ZERO, SINGULAR in intersection solver
- Add special case for sphere/cone poles
- Add special case for cylinder seam (u=0 and u=2π identify)

**Files:** `crates/draper-geometry/src/surface.rs`, `intersection.rs`

---

## 7. Performance & Scalability

### 7.1. Level of Detail (LOD) `[~]`
**Status:** PARTIALLY DONE
- LOD levels defined: Preview(0.1), Low(0.3), Medium(0.5), High(0.75), Ultra(1.0)
- `edge_sample_count_lod()` adapts to LOD
- Decimation DISABLED (breaks watertightness)
- TODO: implement topology-safe decimation (only interior vertices)

### 7.2. Parallel Triangulation `[~]`
**Status:** PARTIALLY DONE
- `triangulate_breps_parallel()` exists in converter.rs
- Uses rayon for per-BREP parallelism
- TODO: intra-BREP parallelism (faces in parallel)

### 7.3. Incremental Triangulation `[ ]`
**Plan:**
- Cache per-face triangulation results
- On model change, only re-triangulate affected faces
- Use face_id → mesh cache

**Files:** `crates/draper-step/src/converter.rs`

### 7.4. Sample Limits `[~]`
**Status:** PARTIALLY DONE
- Current: `MAX_ANGULAR_SAMPLES = 64`, `MAX_HEIGHT_SAMPLES = 64`
- Plan: increase to 256-512 for high-curvature NURBS
- LOD-aware: Ultra LOD uses higher limits

---

## 8. Missing Features

### 8.1. STEP Export `[ ]`
**Plan:**
- Implement AP203/AP214/AP242 export
- Write CARTESIAN_POINT, DIRECTION, AXIS2_PLACEMENT_3D
- Write EDGE_CURVE, VERTEX_POINT, FACE, SHELL, BREP
- Write assembly tree (NAUO)

**Files:** `crates/draper-step/src/exporter.rs` (skeleton exists)

### 8.2. PMI / GD&T `[ ]`
**Plan:**
- Parse GD&T entities (GEOMETRIC_TOLERANCE, DATUM)
- Render GD&T annotations in viewer
- Support AP242 PMI

**Files:** `crates/draper-step/src/pmi.rs`

### 8.3. Feature History `[ ]`
**Plan:**
- Add `FeatureTree` structure
- Each feature records its parameters and dependencies
- On parameter change, re-evaluate only affected features

**Files:** new `crates/draper-history/`

### 8.4. Assembly Support `[~]`
**Status:** PARTIALLY DONE
- `AssemblyNode` tree exists
- `NEXT_ASSEMBLY_USAGE_OCCURRENCE` parsed
- Transforms applied
- TODO: assembly-level colors, layers, instances

### 8.5. Colors & Layers `[~]`
**Status:** PARTIALLY DONE
- Per-face colors parsed from STEP
- TODO: layer support, assembly-level color inheritance

### 8.6. Geometry Cache `[ ]` (covered by 7.3)
Cache per-face triangulation. See 7.3 for details.

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

### 17. Newton Solver for NURBS `[ ]` (covered by 6.2)

### 18. Auto Healing Parameters `[~]` (covered by 5.3)

### 19. Integration Tests on Real STEP Files `[~]`
**Status:** PARTIALLY DONE
- `test_all_files.rs` runs all STEP files in test/
- `watertight_check.rs` validates watertightness
- `angle_check.rs` validates dihedral angles
- TODO: NIST test suite, regression baseline

### 20. Benchmarks vs OpenCascade `[ ]`
**Plan:**
- Create benchmark suite (criterion.rs)
- Compare triangulation time, mesh quality, watertightness
- Publish results in `benchmark_baseline.csv`

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

---

## Conclusion

This roadmap addresses all 20 audit findings. Items marked `[x]` are complete,
`[~]` are partially implemented, `[ ]` are pending. Progress is tracked in the
table above and updated with each commit.
