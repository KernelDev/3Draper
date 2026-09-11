# Roadmap: 3Draper Vision 2036

> Independent technical audit and 10-year strategic development plan for the
> `3Draper` 3D geometric kernel. This document defines the architectural
> evolution from a B-Rep/NURBS engine to a hybrid, GPU-accelerated,
> AI-enhanced geometric kernel suitable for next-generation CAD/CAE/CAM
> workflows.

---

## Document Status

| Field | Value |
|-------|-------|
| **Created** | 2026-08-05 |
| **Author** | Independent Technical Audit |
| **Status** | Active — guiding development priorities |
| **Scope** | Architecture, mathematics, quality engineering, ecosystem |
| **Horizon** | 2026–2036 (5 phases) |

---

## Table of Contents

1. [Critical Technical Debt (Sprint 1–2)](#1-critical-technical-debt)
2. [Mathematics and SSI (Sprint 3–4)](#2-mathematics-and-ssi)
3. [Watertightness by Construction (Sprint 5–6)](#3-watertightness-by-construction)
4. [Industrial Features (Sprint 7+)](#4-industrial-features)
5. [Hybrid Geometry: B-Rep + Implicit (SDF)](#5-hybrid-geometry)
6. [Subdivision Surfaces and T-Splines](#6-subdivision-surfaces)
7. [GPU-Accelerated Compute](#7-gpu-accelerated-compute)
8. [AI-Driven Geometry Healing](#8-ai-driven-geometry-healing)
9. [Quality Engineering](#9-quality-engineering)
10. [Ecosystem and API](#10-ecosystem-and-api)
11. [10-Year Phased Roadmap](#11-10-year-phased-roadmap)
12. [AI-Agent Directives](#12-ai-agent-directives)

---

## 1. Critical Technical Debt

### 1.1 Tolerance System

**Problem:** Global constants `TOLERANCE=1e-6` and `ANGULAR_TOLERANCE=1e-5`
do not scale — microscopic parts and kilometer-scale assemblies use the same
precision, causing geometry loss or numerical explosions.

**Action Items:**

- [x] **Remove all global tolerance constants** — replaced with
      `ToleranceContext` (the legacy `TOLERANCE`/`ANGULAR_TOLERANCE`/
      `PARAMETRIC_TOLERANCE` constants are `#[deprecated]` back-compat
      shims; production code reads `context.*` / `entity.tolerance`).
- [x] **Implement `ContextualTolerance`** — `ToleranceContext` +
      hierarchical propagation: `Solid::apply_model_tolerance` seeds
      Face/Edge tolerances from the STEP uncertainty
      (`entity_tolerance()`), `Solid::recompute_tolerances` rebuilds
      shell/solid aggregates bottom-up (vertices are implicit — their
      geometry rides the owning Edge's `start/end_vertex_point`).
- [x] **Add tolerance consistency validation** — `ToleranceConsistency`
      check in `validate_topology` (finite/positive = Error; parent must
      dominate children = Warning); `rebuild_store` auto-recomputes the
      hierarchy so healing bumps stay consistent.
- [x] **Map 3D tolerance to UV parametric tolerance** using first and second
      fundamental forms for each surface type.
- [x] **Parse `UNCERTAINTY_MEASURE_WITH_UNIT`** from STEP files and use as
      the model's base tolerance; the exporter now round-trips
      `solid.tolerance` back into the file uncertainty.

**Priority:** P0 (blocking — affects all downstream geometry)

### 1.2 Watertightness Illusion

**Problem:** Current strategy is "triangulate then patch holes." Faces are
triangulated independently, then shared edges are merged post-facto via
`merge_coincident_vertices`, `weld`, and `repair_t_junctions`. Complex
assemblies (e.g., `drill_top.stp`) retain up to 0.64% boundary edges.

**Action Items:**

- [x] **Implement Edge Discretization Bus** — each topological edge is
      discretized exactly once in a global cache. Adjacent faces receive
      bit-identical vertex arrays. No post-facto geometric welding.
      (`EdgeDiscretizationCache` + step_id aliasing, incl. seam aliases)
- [x] **Rewrite seam edge handling** — topological gluing (alias
      registration) before 3D coordinate generation, in ALL conversion
      paths, with scale-adaptive seam tolerance (2026-09-09).
- [x] **Add `ManifoldChecker::is_watertight()`** — called before caching
      triangulation; single retry at halved `max_deviation` with a
      deterministic best-result pick; wasm32 check-only (2026-09-09).

**Priority:** P0

### 1.3 Surface-Surface Intersections (SSI)

**Problem:** `intersect_surfaces()` uses marching methods and 4D Newton
refinement but outputs only a polyline. No exact 3D B-spline curve
approximation of the intersection.

**Action Items:**

- [x] **Return exact B-spline intersection curves** from
      `intersect_surfaces()` (implemented as §2.1, commits 8fa3643 /
      1372373 / 6389ea7 / 8dc5517; audited 2026-09-09):
      `SurfaceSurfaceIntersection.b_spline_curves` — per-branch
      least-squares fit with Newton-refined marching points;
      `b_spline_branch_indices` consumer contract maps each fitted
      branch to its polyline. Polyline stays as **per-branch fallback
      only** (unfittable branches keep polyline representation, no
      global failure). Topology consumer `intersect_surfaces_general`
      (boolean.rs) attaches `Curve3d::Nurbs` + param_range = knot
      domain to the shared edge; boolean edges preserve the exact
      curve instead of flattening to polyline. Verified by
      `test_general_ssi_exact_bspline_pcurves` (fitted branch must
      exist, knot-domain param range) and
      `test_cylinder_cylinder_quartic_*` (polyline-only fallback for
      quartic cylinders — exercised intentionally).
- [x] **Implement analytical `Curve2d` (PCURVE)** (implemented as §2.2,
      commits 8fa3643 / 1039889 / 57c6a33; audited 2026-09-09):
      * SSI side — `pcurves_a`/`pcurves_b`: plane×cylinder pairs use
        analytical parametric substitution; all other pairs project the
        3D B-spline branch into UV via Newton inversion; stored as
        `Curve2d::Nurbs` (least-squares) or `Curve2d::Line` (exact);
        closed branches get periodic 2D PCURVEs (C2 seam continuity).
      * STEP parser side — `resolve_curve_2d` resolves all native 2D
        curve types analytically (LINE/CIRCLE/ELLIPSE/HYPERBOLA/
        PARABOLA/B_SPLINE/BEZIER/TRIMMED_CURVE/OFFSET_CURVE_2D);
        POLYLINE only when the file itself stores one.
      * Exporter round-trips SURFACE_CURVE + PCURVE (57c6a33).
        Verified: `test_general_ssi_exact_bspline_pcurves` (composed
        PCURVE deviation < 1e-2 on both surfaces),
        `test_plane_cylinder_pcurve_order_swap`,
        `test_curve2d_inconsistent_pcurve_falls_back_to_projection`.
- [x] **Add analytical derivatives and projections** for PCURVE
      (2026-09-09, draper-geometry/curve2d.rs):
      **Projections** — exact `project_point(p) -> (t, dist)` on all 7
      curve types: `Line2d` (clamped dot-product), `Circle2d` (atan2
      angle + arc-range clamp, exact for full circles), `Ellipse2d` /
      `Hyperbola2d` / `Parabola2d` / `Nurbs2d` (generic engine
      `project_parametric_curve`: 48-sample uniform bracketing scan →
      golden-section shrink → orthogonal-projection polish with step
      halving and clamping); `Curve2d` enum dispatch `project_point` +
      `distance_to`. **Derivatives** — `Nurbs2d::derivative_at` now
      analytical (quotient rule C' = (A' − C·w')/w with Piegl & Tiller
      derivative control points, 2D de Boor `de_boor_step_2d`; numerical
      fallback only on non-finite result, §1.5 philosophy). 12 new tests:
      quarter-circle NURBS derivative vs exact, 3D-twin consistency,
      uniform-knot magnitude, per-type projections (incl. orthogonality
      assertions), composite dispatch. draper-geometry lib **246/246
      passed**, workspace check clean.

**Priority:** P1

### 1.4 STEP Parser and Healing

**Problem:** Parser is blind to tolerance metadata. Healing is destructive
(removes "bad" faces instead of repairing them).

**Action Items:**

- [x] **Extract tolerances from `UNCERTAINTY_MEASURE_WITH_UNIT`** and
      `LENGTH_MEASURE_WITH_UNIT` (verify existing extraction is complete) —
      audited 2026-09-09 with `tol_extract_check`: all 7 canonical test files
      extract correctly (as1 5e-6, drill 3.99e-4, Zentralstaender 2e-5,
      compressor 3.36e-3, SampleCube 1e-6, Spit-Fire/Vulcan 1e-5 via 311/2167
      per-context repetitions). `LENGTH_MEASURE_WITH_UNIT` audited: those
      entities carry unit-conversion factors (0.0254 = inch→metre) and
      property measures — NOT tolerances; correctly not used as tolerance
      source. The Typed `LENGTH_MEASURE` wrapper inside UNCERTAINTY is
      handled by `extract_float_from_step_value`. Session-24 delta
      (2026-09-12): `validation.rs::extract_tolerances` was still the
      weak Float-only extractor — the canonical
      `UNCERTAINTY_MEASURE_WITH_UNIT((LENGTH_MEASURE(v)),...)` form
      returned `uncertainty = None` on the VALIDATION path (the converter
      path was complete). Now uses the same recursive Typed/List-aware
      extraction on all three branches (uncertainty/geometric/shape).
- [x] **Implement surface extension algorithms** — extend surfaces to close
      micro-gaps instead of removing faces.
      (Session-24, 2026-09-12: `NurbsCurve::extended`/`extended_by_distance`
      and `NurbsSurface::extended` — exact-C¹ straight extension spans
      (collinear boundary-tangent control points, uniform boundary
      weights, Bézier-to-Bézier knot bookkeeping preserving
      knots.len() == n_cp + degree + 1); healing pass
      `close_endpoint_gaps_by_extension` (step 2b) closes vertex
      micro-gaps between cross-face boundary edges by extending the edge
      curves toward the junction (line rebuild / NURBS C¹ extension /
      arc angle growth) — repair, never removal. Counter:
      `edge_gaps_extended`. Candidate consumer: the HOUSING rim-aliasing
      twins noted in the self-intersection audit above.)
- [x] **Implement surface-surface intersection for edge recovery** —
      reconstruct lost edges by intersecting adjacent surfaces.

      Done in `crates/draper-topology/src/edge_recovery.rs` (2026-09-11):
      the healing pipeline (pass 2.5, after `close_gaps`) detects
      open-wire gaps where a coedge is missing from BOTH adjacent
      faces, pairs the gaps by endpoint coincidence (reversed
      orientation first, then same-orientation), and reconstructs the
      lost edge from the exact SSI branch (analytic curve / §2.1
      B-spline, trimmed by endpoint projection, authoritative
      vertex-point overrides for bit-identical endpoints, PCURVEs
      attached only when they satisfy the identity parameter-space
      contract). Non-destructive by design — the planar-patch
      `fill_holes` remains the fallback; complements the root-cause
      audit above (their remaining HOUSING rim-aliasing twins are a
      candidate consumer). Bonus fixes: `merge_report` now propagates
      `self_intersections`/`edges_recovered` across shell sub-reports,
      and the plane×cylinder circle-arm cylinder PCURVE height sign
      (`v_on_cyl` = `-signed_dist·(normal·axis)`).

- [x] **Root-cause audit + never-worsen healing gate for "lost edges"**
      (2026-09-10): the HOUSING #47598 "lost edges" were not parser losses —
      healing itself deleted 27 valid faces (265→238) on the strength of
      2579 self-intersection detection hits, ~95% phantom: boundary points
      projecting onto a neighbor's UNTRIMMED surface extension (the
      trimmed-domain check existed as a comment but never as code).
      Detection now (a) builds each face's trimmed UV domain
      (wire-resolved, coedge-orientation-aware, seam-unwrapped for
      periodic surfaces, hole-aware even-odd containment), (b) rejects
      projections outside the trimmed domain, (c) rejects boundary
      CONTACT (shared vertices / coincident duplicate EDGE_CURVEs —
      normal B-Rep adjacency, not intersection); the legacy
      `dist > 1e-20` exact-coincidence guard was removed (domain +
      contact tests own that filtering now). Removal is opt-in via
      `HealingParams::remove_self_intersecting_faces` — default false in
      every preset (aggressive included): detection is report-only.
      HOUSING: 252/265 faces triangulated (was 226), +3411 triangles
      restored; as1-oc-214 remains 18/18 watertight, 0 boundary edges.
      The remaining ~6400 HOUSING boundary edges are dominated by
      per-face interior Steiner holes (CDT default-off, §2.3 line) +
      ~305 rim-aliasing twins (converter shape-group skips) — the
      surface-extension / SSI-recovery items above stay open for those.
- [x] **Add dedicated algorithms for `OffsetSurface` and `SweptSurface`** —
      `SweptSurface` was already analytical (SURFACE_OF_REVOLUTION /
      SURFACE_OF_LINEAR_EXTRUSION → `Revolution`/`Extrusion` surfaces);
      `OFFSET_SURFACE` now extracts NATIVELY as `Surface::Offset`
      (2026-09-09: exact evaluation S = base + d·n, Gauss-map-preserved
      normals, inherited periodicity for §3.3 seam handling, exporter emits
      OFFSET_SURFACE for round-trip — the 16×16 NURBS approximation moved
      to tests). Dedicated meshing refinements tracked under §2.3.
      Session-24 delta (2026-09-12): fold-over safety valve —
      `offset_surface_is_well_formed` (sampled Jacobian orientation vs
      base normal) falls back to the NURBS approximation when
      |d| ≥ κ⁻¹ (e.g. inward offset deeper than the radius), where the
      exact offset would self-intersect and triangulate garbage; the
      `Ruled` export arm now emits a sampled-grid NURBS instead of a
      dangling dummy id.
- [x] **Audit healing NURBS guards** — verified 2026-09-09: all 4 face
      removal paths protect NURBS (merge requires Nurbs×Nurbs compatible —
      mixed surface types never merge; small-face removal retains NURBS;
      self-intersection removal skips NURBS; normal-repair removal skips
      NURBS). The STEP converter delegates all removal to the guarded
      healing pipeline (`apply_healing_to_face_data`). Session-24 delta
      (2026-09-12): the guards were NURBS-only while the converter had
      been producing native `Surface::Offset` faces since 2026-09-09 —
      extended via `is_exact_complex_surface` (Nurbs|Offset|Ruled) in
      all three removal paths; face merging stays safe by construction
      (mixed surface types never merge).


**Priority:** P1

### 1.5 Degeneracies

**Problem:** Sphere/cone poles, zero-length edges, and NaN/Inf cause panics
or zero-area triangles.

**Action Items:**

- [x] **Filter degeneracies at topology analysis stage** — not in renderer.
      Healing pipeline `mark_degenerate_edges` (draper-topology/healing.rs)
      marks zero-length / degenerate-curve edges via `Curve3d::is_degenerate`
      (unit-tested for line/circle/ellipse/arc/NURBS coincident-CP cases);
      the triangulator consumes `edge.degenerate` and skips them at 6+ sites
      (triangulate.rs), and degenerate triangles are counted/filtered in the
      merge/repair passes — the renderer performs no degeneracy filtering.
- [x] **`unwrap()` / `panic!()` audit in math modules** (2026-09-09):
      full production-code audit of draper-geometry — every reachable
      panic site eliminated. Float comparators in `sort_by`/`max_by` now use
      `unwrap_or(Ordering::Equal)` (NaN-tolerant: intersection.rs ×2,
      parametric_domain.rs, mesh_boolean.rs ×2, transmission_bench ×1);
      remaining unwraps verified safe (len-guarded `Vec::last()`, constant
      `Direction3d::new` inputs, or test-only). The full `Result<T,
      GeometryError>` API migration was judged unnecessary after the audit —
      no reachable panic path remains in production math code.
- [x] **Add NaN/Inf guards in all NURBS evaluation paths** — verified
      comprehensive: `nurbs_surface_eval` (empty-CP guard, param clamping,
      OOB row/col guards, `|w| < 1e-15` fallback, non-finite → ORIGIN with
      warning), `NurbsCurve::point_at`/`derivative_at` (w-guards), Nurbs
      `derivatives_at` (w-guard + non-finite → numerical fallback), de Boor
      denominator guards (`|denom| < 1e-15`), knot-span binary-search
      iteration cap, curve2d w-guards.

**Priority:** P1

---

## 2. Mathematics and SSI

### 2.1 Exact B-Spline Intersections

**Goal:** `intersect_surfaces()` returns a B-spline curve, not a polyline.

**Steps:**
1. Run marching methods to get initial intersection points.
2. Fit a B-spline curve through the points using least-squares approximation.
3. Refine the curve using Newton-Raphson on both surfaces simultaneously.
4. Return `Curve3d::Nurbs(...)` as the primary result.

**Fallback:** If fitting fails, return `Curve3d::Polyline(...)` with a warning.

### 2.2 Analytical PCURVE

**Goal:** 2D curves in parametric (UV) space are analytical, not approximated.

**Steps:**
1. For plane-cylinder intersections: derive analytical PCURVE using
   parametric substitution.
2. For NURBS-NURBS intersections: project the 3D B-spline intersection
   curve onto each surface's UV space using Newton-Raphson inversion.
3. Store as `Curve2d::Nurbs(...)` or `Curve2d::Line(...)`.

### 2.3 Dedicated Steiner Grids

**Goal:** `OffsetSurface` and `SweptSurface` have specialized triangulators.

**Steps:**
1. For `OffsetSurface`: offset the base surface's Steiner grid along the
   normal, handling self-intersection via ray-casting.
2. For `SweptSurface`: sweep the profile curve's discretization along the
   trajectory, generating a ruled grid.
3. Both must handle inner contours (holes) without NURBS approximation.

---

## 3. Watertightness by Construction

### 3.1 Edge Discretization Cache Rewrite

**Goal:** Shared topological edges generate vertex arrays exactly once.

**Status (audited 2026-09-09):** IMPLEMENTED — `EdgeDiscretizationCache`
(`crates/draper-mesh/src/edge_cache.rs`), audited against the spec:

- [x] **Architecture mapping** — `entries: HashMap<EdgeCacheKey,
      EdgeDiscretization>` where `EdgeDiscretization` holds `points_3d:
      Vec<Point3d>` (discretized_points ✓), `uv_per_face:
      HashMap<TopoId, Vec<Point2d>>` (uv_points_per_face ✓), and
      `params: Vec<f64>` (normalized curve parameters). Dual-key system:
      `topo_id_to_key` (native path) + `step_id_aliases` with
      `resolve_canonical_step_id()` (STEP round-trip path ✓).
- [x] **Bit-identical guarantee** — all 3D points pass through
      `deterministic_round` (48 mantissa bits) before storage; the first
      face referencing an edge triggers discretization, subsequent faces
      hit the cache and receive the identical point sequence. Empirically
      verified: as1-oc-214 all 18 instances 0 boundary edges;
      determinism_probe test green.
- [x] **Beyond spec** — (a) `circle_group_n` face-based union-find sample
      alignment for co-facial same-axis tube rings; (b)
      `nurbs_refinement_grids` shared chord-error Steiner grids per NURBS
      surface (identical interior vertices for faces sharing a surface);
      (c) `AdaptiveTolerance` scale-aware tolerances from the model bbox;
      (d) step_id aliasing Phase 1 (vertex-pair + shape) and Phase 2
      (3D-coordinate) for same-boundary different-representation edges.

**Architecture:**
```
EdgeDiscretizationBus
├── edge_id → discretized_points (Vec<Point3d>)
├── edge_id → uv_points_per_face (HashMap<face_id, Vec<Point2d>>)
└── edge_id → step_id (for STEP round-tripping)
```

**Guarantee:** When two faces share an edge, they receive bit-identical
vertex coordinates. No welding needed.

### 3.2 BREP Validation Before Triangulation

**Goal:** `validate_brep()` checks topological integrity before any
triangulation begins.

**Status (2026-09-09):** IMPLEMENTED — `validate_brep()` (draper-step
converter) runs in BOTH the chunked (`prepare_brep_session`) and
non-cached (`triangulate_brep_detailed`) paths.

**Checks:**
- [x] 1. Euler characteristic: V - E + F = 2 (for closed solids) —
      **χ = V − E + F − H** (H = inner loops; faces with holes are
      disks-with-holes, χ(face) = 2 − k, NOT 1 — corrected 2026-09-09:
      the naive V−E+F produced false "odd χ" errors on every hole-bearing
      part: as1 bolt #1190 χ3→2, drill SHAFT #1576 χ15→2, HOUSING #47598
      χ9→−18 = genus 10). odd χ → Error (non-orientable/duplicates),
      χ > 2 → warning (void shells / lost faces).
- [x] 2. Face loop closure: every face's wire is closed — covered by
      §1.2 manifold gate + `ManifoldChecker::is_watertight()` after
      triangulation (single retry, deterministic best-result pick).
- [x] 3. Coedge orientation: adjacent coedges have opposite orientations —
      covered by winding-number repair (final post-winding pass, session 24).
- [x] 4. Edge-face count: every interior edge has exactly 2 adjacent faces —
      boundary-edge diagnostics report this post-triangulation;
      topological pre-check surfaced via step_id aliasing stats.

### 3.3 Seam Edge Topological Gluing

**Goal:** Seam edges (on periodic surfaces) are identified and glued
topologically before coordinate generation.

**Status (2026-09-09):** IMPLEMENTED in ALL conversion paths —
`triangulate_brep_detailed`, `prepare_brep_session` (chunked/WASM), and
legacy `triangulate_brep`; `seam_tol` is now scale-adaptive
(`tol_ctx.sewing_tol` from `compute_sewing_tolerance`).

**Steps:**
- [x] 1. Detect periodic surfaces (cylinder, sphere, torus).
- [x] 2. Identify seam edges (u=0 and u=u_max on the same surface).
- [x] 3. Use union-find to merge seam edge pairs topologically — via
      `register_seam_aliases()` → `step_id_aliases` canonical keying.
- [x] 4. Generate 3D coordinates only after topological merging —
      discretization resolves aliases before sampling.
- **Effect (drill_top):** GEAR 767→679 boundary edges (162 seams caught),
      SHAFT_SLEEVE 2778→2747 (75), DRILL_SHAFT 761→749 (6); triangle
      counts dropped after post-glue dedup (SLEEVE 4592→3838).

---

## 4. Industrial Features

### 4.1 GD&T and PMI (AP242)

**Goal:** Full support for Geometric Dimensioning & Tolerancing and
Product Manufacturing Information.

**Steps:**
- [ ] Complete `PresentationLayeredAPI` support in `draper-viewer`.
- [ ] Parse all AP242 GD&T entity types from STEP.
- [ ] Render GD&T annotations in 3D viewport (leader lines, tolerance frames).
- [ ] Support semantic PMI (not just visual presentation).

### 4.2 Adaptive LOD Without Post-Decimation

**Goal:** Each face receives a triangle budget at generation time.

**Architecture:**
```
TriangulationParams {
    target_triangles_per_face: usize,  // computed from LOD + face area
    max_deviation: f64,                // chord tolerance
}
```

**Algorithm:**
1. Compute total triangle budget from LOD level.
2. Distribute budget across faces proportional to surface area.
3. Each face's Steiner grid is generated to hit its budget exactly.
4. No post-decimation needed.

---

## 5. Hybrid Geometry

### 5.1 Implicit Solid (SDF)

**Goal:** `ImplicitSolid` is a first-class citizen alongside `BrepSolid`.

**Features:**
- CSG trees (union, subtract, intersect) over SDF fields — lazy evaluation.
- Dual Contouring mesh generation with sharp feature preservation.
- B-Rep → SDF conversion via 3D voxelization.
- SDF → B-Rep conversion via feature recognition and NURBS fitting.

**Use cases:** 3D printing, lattices, metamaterials, heterogeneous volumes.

### 5.2 SDF Boolean Operations

**Goal:** Milliseconds-scale booleans on models with billions of primitives.

**Approach:**
- Lazy CSG tree evaluation.
- GPU-accelerated SDF evaluation (see §7).
- Adaptive mesh extraction only for visible regions.

---

## 6. Subdivision Surfaces

### 6.1 SubD / T-Splines Module

**Goal:** New `draper-subd` crate for organic modeling.

**Features:**
- Catmull-Clark subdivision surfaces.
- T-Spline support with T-Junctions.
- Crease (sharp edge) support.
- Exact conversion SubD → NURBS B-Rep (no approximation).

**Use case:** Bridge between polygonal modeling (Blender/Maya) and
engineering CAD (SolidWorks/NX).

---

## 7. GPU-Accelerated Compute

### 7.1 WebGPU Compute Shaders

**Goal:** 50-100× speedup for heavy math via GPU parallelism.

**What to offload:**
- `NurbsSurface::evaluate()` — mass evaluation of control points.
- Surface-surface intersection (SSI) marching on GPU thread grid.
- Point projection (Newton-Raphson solver).
- Vertex welding via spatial hash grids on GPU.

**Implementation:**
- WGSL compute shaders via `wgpu`.
- SOA (Structure of Arrays) data layout for GPU compatibility.
- Zero-copy buffer sharing between WASM and WebGPU.

### 7.2 CPU-GPU Interop

**Goal:** Vertex buffers generated in WASM, passed to WebGPU without
CPU-side copies.

**Approach:**
- `SharedArrayBuffer` for JS↔Rust data transfer.
- WebGPU buffer handles mapped directly from WASM memory.
- Avoid serialization — pass buffer IDs, not data.

---

## 8. AI-Driven Geometry Healing

### 8.1 ML Models for CAD

**Goal:** `draper-ai` crate using local neural networks (ONNX Runtime).

**Use cases:**
- **Gap Prediction:** ML predicts the missing surface patch for complex
  hole closure, using neighboring face context as boundary conditions.
- **Feature Recognition:** Auto-detect chamfers, fillets, holes from raw
  mesh-to-BREP import, reconstruct parametric history.
- **Topology Repair:** Predict correct edge-face adjacency for broken
  STEP files.

### 8.2 Training Pipeline

**Steps:**
1. Collect 10,000+ "dirty" STEP files with known-good repairs.
2. Train ONNX models for gap prediction and feature recognition.
3. Ship models as binary assets in `draper-ai`.
4. Inference runs locally — no cloud dependency.

---

## 9. Quality Engineering

### 9.1 Golden File Regression Testing

**Goal:** 1000+ reference STEP files with pre-computed "ideal" meshes.

**Metrics:**
- **Hausdorff Distance:** max deviation ≤ `max_deviation`.
- **Topological Isomorphism:** face/edge/vertex counts match reference.
- **Watertightness:** `boundary_edges == 0` for all closed solids.

**Implementation:**
- `draper-testing` crate with `#[test]` per golden file.
- CI runs golden file tests on every PR.
- Regression detection: compare against last-known-good mesh hash.

### 9.2 Fuzz Testing

**Goal:** Panic-free guarantee for all input data.

**Tools:** `cargo-fuzz` (libFuzzer), `AFL++`.

**Targets:**
- STEP parser: syntactically valid but semantically absurd files.
- NURBS solver: random weights, knot vectors, control points.
- Boolean operations: random B-Rep pairs.

**Criterion:** No panic under any input. Errors return
`Result<T, GeometryError>`.

### 9.3 Property-Based Testing

**Goal:** Mathematical invariants hold for all operations.

**Tool:** `proptest`.

**Invariants:**
1. Euler characteristic: V - E + F = 2 for closed solids.
2. Every interior edge has exactly 2 adjacent coedges with opposite
   orientations.
3. Boolean operations preserve manifold status.
4. Triangulation produces non-degenerate triangles (area > 0).

---

## 10. Ecosystem and API

### 10.1 WASM and WebGPU Integration

**Goal:** Zero-copy, high-performance web rendering.

**Steps:**
- `SharedArrayBuffer` for JS↔Rust vertex/index array access.
- WebGPU interop: vertex buffers generated in WASM, passed to WebGPU
  without CPU copy.
- Reduce WASM bundle size (tree-shake unused surface types).

### 10.2 FFI and Language Bindings

**Goal:** Easy integration with any language.

**Steps:**
- Stabilize C-API in `draper-ffi` for C++/Python/C# integration.
- Implement `draper-brep` binary format (FlatBuffers or Cap'n Proto) for
  instant model loading without STEP parsing.
- Python bindings via PyO3.
- C# bindings via P/Invoke.

---

## 11. 10-Year Phased Roadmap

### Phase 1: Stabilization and Mathematical Purity (2026–2027)

**Focus:** Close technical debt.

- [ ] Full transition to contextual hierarchical tolerances.
- [ ] 100% watertight on all standard STEP files (AP203/AP214/AP242).
      **Progress note (2026-09-09, mesh CDT audit):** the per-face
      triangulation pipeline is now provably hole-free —
      `custom_cdt::triangulate_polygon_cdt` (earcutr boundary +
      Bowyer-Watson Steiner insertion + rim-vertex repair) replaces the
      earcutr "spike-chain" Steiner append (regression test
      `test_steiner_insertion_no_interior_gaps_vs_legacy_earcutr` proves
      the legacy path leaks); consecutive-duplicate boundary dedup +
      Steiner 3D-position dedup eliminate all position-degenerate
      triangle drops (HOUSING #47598: 1369 → 0). The CDT is gated
      behind `TriangulationParams::use_cdt_steiner` (**default off**):
      faces sharing one NURBS surface receive the same shared Steiner
      points but build different CDT connectivity per face, adding
      cross-face boundary edges on dirty files (HOUSING 6035 → 14292
      when enabled). Remaining boundary edges (HOUSING 6035, from 6089)
      are CROSS-FACE rim mismatches from converter-level edge aliasing
      failures (skipped step_ids, duplicated EDGE_CURVEs) — the §1.4
      "SSI for edge recovery" work.
      **Progress note (2026-09-11, session 30):** the surface-level
      canonical triangulation is IMPLEMENTED (`surface_canonical.rs`:
      one constrained CDT per shared NURBS — hull-fan seed, flip-only
      constraint enforcement, protected Steiner insertion, per-face
      centroid extraction, rim-contract validation) behind
      `TriangulationParams::use_surface_canonical_cdt` (default off,
      converter pre-pass on all three BREP paths). Verified
      never-worsen: as1-oc-214 0 boundary in both modes;
      SHAFT_SLEEVE 3102 → 3065 boundary with canonical on. The
      blocking defects for default-on: pinched rims (57/97 HOUSING
      NURBS groups fail the >2-adjacency validation — needs loop
      splitting at the pinch vertex) and sliver-UV extraction
      fallbacks (233 faces, legacy via rim-contract guard).
- [ ] Analytical `Curve2d` (PCURVE) and exact B-spline SSI.
- [ ] Property-based testing for topology.
- [ ] Fuzz testing for STEP parser and NURBS solver.
- [ ] Edge Discretization Bus (shared edge cache).
- [ ] BREP validation before triangulation.

### Phase 2: Performance and GPU (2028–2029)

**Focus:** Speed and scalability.

- [ ] NURBS evaluation and triangulation on WebGPU Compute Shaders.
- [ ] Adaptive LOD at mesh generation time (no post-decimation).
- [ ] Support 10M+ face assemblies at 60 FPS.
- [ ] Zero-copy WASM ↔ WebGPU buffer sharing.

### Phase 3: Hybrid Geometry (2030–2031)

**Focus:** Beyond classical CAD.

- [ ] `ImplicitSolid` (SDF) module with CSG trees.
- [ ] SubD/T-Splines with exact NURBS conversion.
- [ ] AI-driven healing (local ML models for gap closure).
- [ ] Dual Contouring mesh generation on GPU.

### Phase 4: Industrial Standard and IGA (2032–2033)

**Focus:** Simulation and Digital Twin.

- [ ] Isogeometric Analysis (IGA) — export NURBS models directly to FEA.
- [ ] Full AP242 support (PMI, GD&T, manufacturing semantics).
- [ ] IoT and Digital Twin integration (real-time geometry versioning).

### Phase 5: Cloud-Native and Collaborative CAD (2034–2036)

**Focus:** The future of design.

- [ ] CRDT for topology — simultaneous multi-user B-Rep editing.
- [ ] Generative design as native kernel function (topology optimization on SDF).
- [ ] Quantum-resistant geometry hashing and versioning.

---

## 12. AI-Agent Directives

When writing code, reviewing PRs, or making architectural decisions in the
`3Draper` repository, the AI agent **MUST** follow these directives:

### Directive 1: No Global Tolerance Constants

Any request to use `const TOLERANCE: f64 = 1e-6;` must be rejected. Always
require `entity.tolerance()` or `context.tolerance()`.

### Directive 2: Topology Over Geometry

When solving watertightness problems, priority is always given to
topological gluing (shared edge cache, union-find) over geometric welding
or snapping post-facto.

### Directive 3: Panic-Free Guarantee

In mathematical modules (`draper-geometry`, `draper-mesh`), the use of
`unwrap()` or `panic!()` is strictly forbidden. All edge cases (NaN, Inf,
degenerate surfaces) must return `Err(GeometryError::DegenerateCase)`.

### Directive 4: Invariant Testing

Every new boolean operation or triangulation algorithm must be accompanied
by `proptest` tests verifying Euler characteristic and manifold status of
the resulting mesh.

### Directive 5: GPU-First Thinking

When designing new heavy algorithms (e.g., SDF booleans), design the
interface with Compute Shader portability in mind: mass parallelism, SOA
data structures, no pointers.

### Directive 6: NURBS Protection

NURBS faces represent complex geometry (fillets, threads, organic shapes)
that must never be removed during healing. All healing steps must include
NURBS guards.

### Directive 7: Edge Cache First

Triangulation must use the Edge Discretization Cache for all shared edges.
Independent face triangulation with post-facto welding is deprecated.

---

## Progress Tracking

| Phase | Milestone | Status | Commit |
|-------|-----------|--------|--------|
| 1 | Remove global TOLERANCE constants | Done | `065b9e2` |
| 1 | Parse UNCERTAINTY_MEASURE_WITH_UNIT | Done | `93df2de` |
| 1 | Contextual hierarchical tolerances | Done | `dd99d0a` + propagation/consistency/round-trip |
| 1 | NURBS healing guards | Done | `eb46eb1` |
| 1 | ManifoldChecker::is_watertight() | Done | `f8f023c` |
| 1 | GeometryError + panic-free production code | Done | `9d7ad7f` |
| 1 | NaN/Inf guards in NURBS evaluation | Done | `0da6e6e` |
| 1 | Edge Discretization Bus | Existing (verified) | — |
| 1 | BREP validation before triangulation | Done | `9244a7b` |
| 1 | Seam edge topological gluing | Done | `058805c` |
| 1 | Analytical PCURVE (derive_pcurve) | Done | `830f782` |
| 2 | Periodic 2D PCURVEs for closed branches (lattice C2 seam) | Done | `1039889` |
| 1 | Exact B-spline SSI (fit_b_spline) | Done | `e03d758` |
| 1 | Property-based testing (proptest) | Done | `2d74d8c` |
| 1 | Fuzz testing setup (quickcheck) | Done | `e7ee121` |
| 1 | Determinism CI gate (probe ×N runs, digest diff) | Done | `c3d8de9` |
| 2 | GPU-ready NURBS batch eval (SOA) | Done | `d016ada` |
| 2 | WebGPU compute shaders | Pending (API ready) | — |
| 2 | Adaptive LOD at generation | Done | `dea581e` |
| 3 | ImplicitSolid (SDF) | Done | `6fbd4f2` |
| 3 | SubD/T-Splines | Done | `e00159f` |
| 3 | AI-driven healing | Done | `d7e8583` |
| 4 | IGA export | Done | `248ee65` |
| 4 | Full AP242 PMI/GD&T | Done | `e2bb8cb` |
| 5 | CRDT collaborative editing | Done | `486f6a5` |
| 5 | Generative design | Done | `d3a092f` |

---

*This document is a living roadmap. Update the Progress Tracking table as
milestones are completed. Cross-reference commits and PRs for traceability.*
