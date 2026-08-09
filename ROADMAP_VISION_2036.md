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

- [ ] **Remove all global tolerance constants** — replace `const TOLERANCE: f64 = 1e-6`
      with `entity.tolerance()` or `context.tolerance()`.
- [ ] **Implement `ContextualTolerance`** — hierarchical tolerance context
      that propagates from Solid → Shell → Face → Edge → Vertex.
- [ ] **Add tolerance consistency validation** — ensure child tolerances do
      not contradict parent tolerances.
- [ ] **Map 3D tolerance to UV parametric tolerance** using first and second
      fundamental forms for each surface type.
- [ ] **Parse `UNCERTAINTY_MEASURE_WITH_UNIT`** from STEP files and use as
      the model's base tolerance (already partially done — verify completeness).

**Priority:** P0 (blocking — affects all downstream geometry)

### 1.2 Watertightness Illusion

**Problem:** Current strategy is "triangulate then patch holes." Faces are
triangulated independently, then shared edges are merged post-facto via
`merge_coincident_vertices`, `weld`, and `repair_t_junctions`. Complex
assemblies (e.g., `drill_top.stp`) retain up to 0.64% boundary edges.

**Action Items:**

- [ ] **Implement Edge Discretization Bus** — each topological edge is
      discretized exactly once in a global cache. Adjacent faces receive
      bit-identical vertex arrays. No post-facto geometric welding.
- [ ] **Rewrite seam edge handling** — use topological gluing (union-find)
      before generating 3D coordinates, not geometric `weld` after.
- [ ] **Add `ManifoldChecker::is_watertight()`** — called before caching
      triangulation. If mesh is not manifold, retry with reduced
      `max_deviation`.

**Priority:** P0

### 1.3 Surface-Surface Intersections (SSI)

**Problem:** `intersect_surfaces()` uses marching methods and 4D Newton
refinement but outputs only a polyline. No exact 3D B-spline curve
approximation of the intersection.

**Action Items:**

- [ ] **Return exact B-spline intersection curves** from `intersect_surfaces()`.
      Polyline should be fallback only when Newton iteration fails.
- [ ] **Implement analytical `Curve2d` (PCURVE)** — remove polyline
      approximation of UV-space curves.
- [ ] **Add analytical derivatives and projections** for PCURVE.

**Priority:** P1

### 1.4 STEP Parser and Healing

**Problem:** Parser is blind to tolerance metadata. Healing is destructive
(removes "bad" faces instead of repairing them).

**Action Items:**

- [ ] **Extract tolerances from `UNCERTAINTY_MEASURE_WITH_UNIT`** and
      `LENGTH_MEASURE_WITH_UNIT` (verify existing extraction is complete).
- [ ] **Implement surface extension algorithms** — extend surfaces to close
      micro-gaps instead of removing faces.
- [ ] **Implement surface-surface intersection for edge recovery** —
      reconstruct lost edges by intersecting adjacent surfaces.
- [ ] **Add dedicated algorithms for `OffsetSurface` and `SweptSurface`** —
      stop force-approximating them as NURBS.
- [ ] **Audit healing NURBS guards** — verify all healing steps protect
      NURBS faces from removal (done in commit `eb46eb1`, verify coverage).

**Priority:** P1

### 1.5 Degeneracies

**Problem:** Sphere/cone poles, zero-length edges, and NaN/Inf cause panics
or zero-area triangles.

**Action Items:**

- [ ] **Filter degeneracies at topology analysis stage** — not in renderer.
- [ ] **Replace all `unwrap()` / `panic!()` in math modules** with
      `Result<T, GeometryError>`.
- [ ] **Add NaN/Inf guards** in all NURBS evaluation paths.

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

**Checks:**
1. Euler characteristic: V - E + F = 2 (for closed solids)
2. Face loop closure: every face's wire is closed
3. Coedge orientation: adjacent coedges have opposite orientations
4. Edge-face count: every interior edge has exactly 2 adjacent faces

### 3.3 Seam Edge Topological Gluing

**Goal:** Seam edges (on periodic surfaces) are identified and glued
topologically before coordinate generation.

**Steps:**
1. Detect periodic surfaces (cylinder, sphere, torus).
2. Identify seam edges (u=0 and u=u_max on the same surface).
3. Use union-find to merge seam edge pairs topologically.
4. Generate 3D coordinates only after topological merging.

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
| 1 | Contextual hierarchical tolerances | In Progress | `dd99d0a` |
| 1 | NURBS healing guards | Done | `eb46eb1` |
| 1 | ManifoldChecker::is_watertight() | Done | `f8f023c` |
| 1 | GeometryError + panic-free production code | Done | `9d7ad7f` |
| 1 | NaN/Inf guards in NURBS evaluation | Done | `0da6e6e` |
| 1 | Edge Discretization Bus | Existing (verified) | — |
| 1 | BREP validation before triangulation | Done | `9244a7b` |
| 1 | Seam edge topological gluing | Done | `058805c` |
| 1 | Analytical PCURVE (derive_pcurve) | Done | `830f782` |
| 1 | Exact B-spline SSI (fit_b_spline) | Done | `e03d758` |
| 1 | Property-based testing (proptest) | Done | `2d74d8c` |
| 1 | Fuzz testing setup (quickcheck) | Done | `e7ee121` |
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
