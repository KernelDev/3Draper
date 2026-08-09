# 3Draper / BRepCAD

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/Rust-1.97+-orange.svg)](https://www.rust-lang.org/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS%20%7C%20Windows%20%7C%20WASM-green.svg)](#building)

A production-grade **Rust-native 3D CAD/CAE/CAM kernel and editor** (BRepCAD) with
an own B-rep topology engine, NURBS geometry, parametric modeling, assemblies,
drawings, FEA, CAM, and AI integration. No external CAD kernels (OCCT, Parasolid)
— everything is built from scratch in safe Rust.

## Key Features

### Geometry & Topology
- **Own B-rep kernel** — Vertex/Edge/Wire/Face/Shell/Solid hierarchy
- **NURBS curves & surfaces** — full evaluation, knot insertion, refinement
- **Exact SSI** (Surface-Surface Intersection) for robust curve/surface ops
- **Parametric modeling** — feature timeline, parameter formulas, rebuild
- **Direct modeling** — move/offset/replace/split face operations
- **Boolean operations** — union, subtract, intersect with tolerance context
- **Healing** — geometry repair, gap filling, degenerate edge removal

### Sketch Engine
- **2D entities** — line, circle, arc, rectangle, point, **spline** (Catmull-Rom), **polygon** (regular N-gon)
- **Constraint solver** — Newton-Raphson + SVD, coincident/parallel/tangent/distance/angle
- **Dimensions** — linear, angular, radial, diameter
- **3D projection** — sketch on arbitrary plane (XY, XZ, YZ, or custom)

### Assembly
- **Component hierarchy** — instances, transforms, local/world AABBs
- **Mate constraints** — coincident, concentric, distance, angle, parallel, tangent
- **BVH collision detection** — O(log n) tree-based, kinematic drag with rollback
- **Bill of Materials** (BOM) editor

### Drawing
- **HLR (Hidden Line Removal)** — Möller-Trumbore ray casting + tree-based BVH (100× faster)
- **Associative dimensions** — linked to features, auto-update on rebuild
- **PDF export** — vector graphics, PDF 1.4, no external dependencies
- **SVG export** — for web embedding
- **Title block editor**, **revision table**, **layer manager**

### CAM
- **Operations** — pocket, profile, drill, engrave, 3D surfacing
- **Post-processors** — Fanuc, Siemens, Haas, Heidenhain, Mach3, LinuxCNC, GRBL
- **NC Code Viewer** — syntax highlighting (G/M/T/X/Y/Z/F/S/N codes)
- **Tool library** — tool diameter, feed rate, spindle speed management
- **2D/3D simulation** — toolpath visualization

### FEA
- **Linear static analysis** — stress, displacement
- **Modal analysis** — natural frequencies, mode shapes
- **Mesh control** — element size, quality metrics
- **Modal plotter** — frequency response bar chart

### Sheet Metal
- **Base flange**, **edge flange**, **bend**, **hem**, **jog**
- **Relief** — rectangular, tear
- **Flatten** — unfold/fold, flat pattern, DXF export
- **Gauge table** support

### Rendering & Viewport
- **WebGPU triangulation** — Marching Cubes + Ear Clipping WGSL shaders (workgroup 256)
- **CPU fallback** — `triangulate_marching_cubes_cpu()` for headless/CI
- **Camera** — perspective + orthographic projection, orbit, fit, ISO/ Front/Top views
- **Display modes** — wireframe, shaded, shaded+edges
- **View toggles** — grid, axes, triad, view cube, shadows, AO, edges, normals, silhouette
- **Navicube** — click-to-orient navigation widget
- **Section cut** — real-time cross-section visualization

### AI Integration
- **LLM backend** — HTTP client (TcpStream, no deps) for Ollama/OpenAI
- **Shape from text** — natural language → 3D primitive
- **Design review** — automated geometry audit
- **ML healing** — learned geometry repair strategies

### Cloud & Collaboration
- **CRDT sync** — Conflict-free Replicated Data Types for real-time collab
- **WebSocket** — multi-user editing, branch management
- **Cloud panel** — avatars, activity feed, storage

### Scripting & Automation
- **Python console** — live scripting with BRepCAD API
- **Macro recorder** — records UI actions, exports to Python/Lua scripts
- **Visual programming** — node graph (Box, Sphere, Boolean, Fillet, etc.)
- **Animation timeline** — keyframe-based parameter animation

## Architecture

```
3Draper/
├── crates/
│   ├── draper-geometry/      Curves, surfaces, transforms, NURBS, SSI
│   ├── draper-topology/      B-rep: Vertex/Edge/Wire/Face/Shell/Solid
│   │                         Boolean ops, fillet/chamfer, direct modeling,
│   │                         extrude/revolve/sweep/loft, feature history
│   ├── draper-mesh/          Triangulation, HLR, watertight, decimate,
│   │                         OBJ/PLY/STL/DXF/glTF I/O, subdivision
│   ├── draper-step/          STEP (ISO 10303-21) parser + writer (AP203/214/242)
│   ├── draper-sketch/        2D sketch engine + constraint solver + projection
│   ├── draper-assembly/      Components, mates, BVH collision, kinematics
│   ├── draper-drawing/       HLR, dimensions, PDF/SVG export, title blocks
│   ├── draper-cam/           Toolpaths, post-processors, G-code generation
│   ├── draper-fea/           Linear static, modal analysis, mesh quality
│   ├── draper-sheetmetal/    Flanges, bends, relief, flatten, flat pattern
│   ├── draper-ai/            LLM client, shape-from-text, design review
│   ├── draper-cloud/         CRDT, WebSocket sync, collaboration
│   ├── draper-compute/       WebGPU compute pipelines (triangulation, NURBS eval)
│   ├── draper-implicit/      SDF/implicit modeling, generative design
│   ├── draper-subd/          Subdivision surface modeling
│   ├── draper-core/          Document, Scene, high-level API
│   ├── draper-viewer/        BRepCAD editor (egui UI, 21 menus, 15 ribbon tabs)
│   ├── draper-ffi/           C FFI bindings (draper.h)
│   ├── draper-wasm/          WebAssembly build
│   ├── draper-worker/        Background compute workers
│   ├── draper-json/          JSON serialization for geometry
│   └── draper-testing/       E2E integration tests, certification suite
├── tools/
│   └── src/bin/              Diagnostic tools (angle_check, bbox_diag, etc.)
├── assets/
│   └── navicube/             Navicube OBJ/STEP/STL models
├── test/                     STEP test files (synthetic + real-world)
├── docs/
│   ├── ui_mockups/           96 SVG UI mockups + index.html
│   ├── AUDIT_IMPLEMENTATION_PLAN.md
│   ├── MOCKUP_CODE_MAPPING.md
│   └── CONSISTENT_TRIANGULATION_PIPELINE.md
├── include/
│   └── draper.h              Generated C header
├── ROADMAP.md                Development roadmap (Russian)
├── MASTER_PLAN_100.md        100% implementation plan
├── ROADMAP_VISION_2036.md    10-year vision (hybrid geometry, GPU, AI)
├── IMPROVEMENT_PLAN.md       Post-audit action plan (15 tasks, all done)
└── FLEXIBLE_EXECUTION_PLAN.md Parametric core + assembly + advanced features
```

## Building

### Prerequisites
- Rust 1.97+ (install via [rustup](https://rustup.rs/))
- Linux: `build-essential`, `libgtk-3-dev`, `libwebkit2gtk-4.1-dev` (for native viewer)

### Build the kernel + viewer

```bash
cargo build --release
```

### Run the BRepCAD editor

```bash
cargo run -p draper-viewer --release
```

### Run tests

```bash
# All tests
cargo test

# Just the e2e integration tests
cargo test -p draper-testing --test e2e_workflow

# Just the geometry/topology/mesh unit tests
cargo test -p draper-geometry -p draper-topology -p draper-mesh
```

### Build the FFI library

```bash
cargo build -p draper-ffi --release
```

This produces:
- `libdraper_ffi.a` (static library)
- `libdraper_ffi.so` / `.dylib` / `.dll` (shared library)
- C header at `include/draper.h`

### WebAssembly build

```bash
trunk build   # builds the WASM viewer to crates/draper-viewer/dist/
```

## Usage

### From C

```c
#include "draper.h"

DraperDocument* doc = draper_open_step("model.stp");
if (doc) {
    DraperStatistics stats = draper_get_statistics(doc);
    printf("Triangles: %lu\n", stats.triangle_count);

    DraperMesh* mesh = draper_get_mesh(doc);
    // Access vertices: draper_mesh_vertices(mesh)
    // Access indices: draper_mesh_indices(mesh)

    draper_mesh_free(mesh);
    draper_document_free(doc);
}
```

### From Rust

```rust
use draper_topology::{ShapeBuilder, operations::{extrude_polyline, Polyline2d}};
use draper_geometry::Vec3d;

// Sketch a rectangle
let profile = Polyline2d::rectangle(100.0, 50.0);

// Extrude into a solid
let solid = extrude_polyline(&profile, Vec3d::new(0.0, 0.0, 1.0), 30.0)
    .expect("Extrude failed");

// Triangulate for rendering
let mesh = draper_mesh::triangulate_solid(&solid, &TriangulationParams::default());
println!("{} vertices, {} triangles", mesh.vertices.len(), mesh.triangles.len());
```

### File Formats

| Format | Import | Export |
|--------|--------|--------|
| STEP (AP203/214/242) | ✅ | ✅ |
| STL (ASCII + binary) | ✅ | ✅ |
| OBJ (Wavefront) | ✅ | ✅ |
| PLY (ASCII + binary LE) | ✅ | ✅ |
| DXF (2D flat pattern) | — | ✅ |
| glTF / GLB | — | ✅ |
| PDF (vector) | — | ✅ |
| SVG | — | ✅ |
| IGA (NURBS for FEA) | — | ✅ |
| 3MF | — | ✅ |
| USD / USDA | — | ✅ |

## UI Overview (BRepCAD Editor)

- **21 cascading menus** — File, Edit, View, Insert, Sketch, Modify, Sheet Metal,
  Assembly, CAM, Drawing, Simulation, Parametric, Optimize, GDT, Heal, Mold,
  Tools, Scripting, AI, Window, Help
- **15 ribbon tabs** — File, Home, Sketch, Insert, Modify, Sheet Metal, Assembly,
  CAM, Drawing, Simulation, Inspect, AI, Tools, View, Surface
- **8 workspaces** — Modeling, Sketch, Visual Programming, Sheet Metal, CAM,
  FEA Simulation, Drawing, AI
- **27 dialogs** — Options, Customize, Primitives, BOM Editor, Layer Manager,
  Tool Library, FEA Mesh Control, Title Block Editor, Param Search/Replace,
  Revision Table, Tutorial Browser, Crash Recovery, Onboarding Wizard,
  Update Check, License Info, Mold Catalog, Modal Plotter, NC Code Viewer,
  Macro Recorder, Print/Plot, Constraint Diagnostics, Render Settings, etc.
- **Procedural vector icons** — 90+ hand-drawn icons (no Unicode emoji, no external assets)
- **Status bar** — coordinates, camera, tool, FPS, units, display style, selection

## Testing

The project has **800+ tests** across all crates:

- **Unit tests** — geometry, topology, mesh, sketch, assembly, drawing, CAM, FEA
- **Property-based tests** — fuzzing with `proptest` for geometry invariants
- **E2E integration tests** — `crates/draper-testing/tests/e2e_workflow.rs`:
  - Sketch → Extrude → Fillet → Parametric Rebuild
  - Box → Boolean Subtract → OBJ/PLY/DXF Export round-trip
  - Assembly collision check (BVH)
  - Direct Modeling (move/offset face)
  - Sketch Spline + Polygon tessellation
  - Full pipeline Sketch → Extrude → Triangulate → OBJ
  - Camera projection modes (perspective/orthographic)
  - Macro Recorder (record + Python/Lua export)
- **STEP certification suite** — `crates/draper-testing/src/certification.rs`
- **Watertight mesh validation** — `tools/src/bin/watertight_check.rs`

## Key Principles

- **Own kernel** — no external 3D kernels (OpenCascade, Parasolid, C3D)
- **Safe Rust** — no `unwrap()`/`panic!()` in geometry code; `Result<T, Error>` everywhere
- **Tolerant modeling** — hierarchical tolerance context, model-scale-aware
- **Watertight triangulation** — shared edge discretization cache, bit-identical points
- **Cross-platform** — Linux, macOS, Windows, WebAssembly
- **Cross-language** — C FFI for Python, C#, JavaScript, etc.
- **GPU-accelerated** — WebGPU compute shaders for triangulation, CPU fallback for CI

## Documentation

- [ROADMAP.md](ROADMAP.md) — Development roadmap (Russian)
- [MASTER_PLAN_100.md](MASTER_PLAN_100.md) — 100% implementation plan
- [ROADMAP_VISION_2036.md](ROADMAP_VISION_2036.md) — 10-year vision
- [IMPROVEMENT_PLAN.md](IMPROVEMENT_PLAN.md) — Post-audit action plan
- [FLEXIBLE_EXECUTION_PLAN.md](FLEXIBLE_EXECUTION_PLAN.md) — Parametric core plan
- [BREPCAD_IMPLEMENTATION_PLAN.md](BREPCAD_IMPLEMENTATION_PLAN.md) — BRepCAD specifics
- [docs/ui_mockups/](docs/ui_mockups/) — 96 SVG UI mockups
- [docs/MOCKUP_CODE_MAPPING.md](docs/MOCKUP_CODE_MAPPING.md) — Mockup → code mapping

## Current Status

**v0.1.0** — Active development. All 15 IMPROVEMENT_PLAN tasks complete:

- ✅ Icon system (90+ procedural vector icons)
- ✅ Menu/workspace/status bar icons
- ✅ 27 dialogs (all interactive, no stubs)
- ✅ Clipboard (Cut/Copy/Paste)
- ✅ NC Code Viewer (real G-code + dialect)
- ✅ Macro Recorder (record + Python/Lua export)
- ✅ Direct Modeling (move/offset/replace/split face)
- ✅ Sketch Spline + Polygon (Catmull-Rom tessellation)
- ✅ View toggles (perspective/orthographic, shadows, AO)
- ✅ E2E integration tests (8 workflows, headless)
- ✅ File I/O (OBJ, PLY, DXF, glTF import/export)
- ✅ Assembly rotations + BVH collision
- ✅ Drawing HLR (100× faster with tree-based BVH)
- ✅ Associative dimensions + PDF export
- ✅ LLM HTTP client (Ollama/OpenAI)
- ✅ WebGPU triangulation + CPU fallback
- ✅ Kinematic drag with collision rollback

## License

GNU General Public License v3.0 or later (GPL-3.0-or-later)

This project is free software: you can use, study, modify and redistribute it
under the terms of the **GNU GPLv3+**. Any derivative work — including use in
commercial products — **must also be released as open source under the same
license**. This ensures that no one can incorporate this kernel into proprietary
software without giving back to the community.

See [LICENSE](LICENSE) for the full license text.
