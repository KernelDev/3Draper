# BRepCAD — 3Draper-powered CAD/CAE/CAM

BRepCAD is a full-featured CAD/CAE/CAM application built on top of the
[3Draper](https://github.com/KernelDev/3Draper) 3D geometric kernel.
It runs both as a native desktop application and in the browser via WebAssembly.

## Live Demo

**BRepCAD on GitHub Pages:** https://kerneldev.github.io/3Draper/brepcad.html

**3Draper Viewer (original):** https://kerneldev.github.io/3Draper/

Both apps are deployed from the same `dist/` directory. A floating link in the
top-right corner lets you switch between them.

## Quick Start

### Native (Desktop)

```bash
cargo run --bin brepcad-shell
```

This launches BRepCAD with the full 21-menu bar + 15-tab ribbon UI,
using the existing 3Draper wgpu/GL renderer for 3D viewport.

### WASM (Browser)

#### Prerequisites
- Rust 1.97+ with `wasm32-unknown-unknown` target:
  ```bash
  rustup target add wasm32-unknown-unknown
  ```
- [Trunk](https://trunkrs.dev/) 0.21+:
  ```bash
  cargo install trunk --version "^0.21"
  ```

#### Development server

```bash
trunk serve --config crates/draper-viewer/Trunk.brepcad.toml crates/draper-viewer/brepcad.html
```

Then open `http://127.0.0.1:8081` in your browser.

#### Production build

```bash
trunk build --config crates/draper-viewer/Trunk.brepcad.toml crates/draper-viewer/brepcad.html --release
```

Output is in `crates/draper-viewer/dist-brepcad/`. Deploy this directory to any
static web host (GitHub Pages, Netlify, Cloudflare Pages, etc.).

#### Manual WASM build (without Trunk)

```bash
cargo build --bin brepcad-shell \
    --target wasm32-unknown-unknown \
    --no-default-features --features web-deploy \
    --release
```

This produces `target/wasm32-unknown-unknown/release/brepcad-shell.wasm`
(~13 MB). You then need `wasm-bindgen` to generate the JS glue:

```bash
wasm-bindgen --target web --out-dir www \
    target/wasm32-unknown-unknown/release/brepcad-shell.wasm
```

## Architecture

BRepCAD is a **thin wrapper** around the existing `draper-viewer::ViewerApp`:

```
┌─────────────────────────────────────────────────────────┐
│  brepcad_shell.rs (native main + WASM start)            │
│  - Creates ViewerApp::new(cc)                           │
│  - Sets app.enable_brepcad_ui = true                    │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│  ViewerApp (app.rs)                                     │
│  - update(): if enable_brepcad_ui {                     │
│      render_menu_bar() → handle_brepcad_action()        │
│      render_ribbon()    → handle_brepcad_action()       │
│    } else { original File/View menu }                   │
│  - Full wgpu/GL renderer (SceneCallback)                │
│  - Structure panel, face info, instance selection       │
│  - NURBS gallery, GD&T, UV breakdown                    │
│  - Progressive triangulation, web worker, IndexedDB     │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│  UI modules (ui/*.rs)                                   │
│  - menubar.rs: 21 top-level menus → MenuAction enum     │
│  - ribbon.rs: 15 ribbon tabs → MenuAction               │
│  - dialogs.rs: 8 dialogs (Options/About/Insert/Plugins) │
│  - command_palette.rs: 50+ commands                     │
│  - view_modes.rs, context_menus.rs, panels.rs           │
│  - core_engine.rs, sketch.rs, workspaces.rs             │
└─────────────────────────────────────────────────────────┘
```

## Features

- **3D Viewport**: Full wgpu/GL rendering with wireframe, shaded, and
  shaded+edges display modes. Same quality as 3Draper Viewer.
- **Menu Bar**: 21 menus (File, Edit, View, Insert, Sketch, Modify, Sheet Metal,
  Assembly, CAM, Drawing, Simulation, Parametric, Optimize, GD&T, Heal, Mold,
  Tools, Scripting, AI, Window, Help) with 280+ actions.
- **Ribbon**: 15 tabs (File, Home, Sketch, Insert, Modify, SheetMetal, Assembly,
  CAM, Drawing, Simulation, Inspect, AI, Tools, View, Surface).
- **Command Palette**: Ctrl+Shift+P for fuzzy command search (50+ commands).
- **Dialogs**: Options, About, Insert Primitive, Plugins, Performance, etc.
- **Marking Menu**: Space key for quick view switching.
- **View Cube**: 8 orientation presets (ISO, Front, Back, Top, Bottom, Left, Right, Dimetric).
- **Sketch Engine**: 2D canvas with 13 drawing tools, 9 constraints, 4 dimensions, solver.
- **Workspaces**: Visual Programming, Surface, Sheet Metal, CAM, FEA, Drawing,
  Assembly, Point Cloud, Mold, AI (data models).
- **All 3Draper features**: STEP/STL/OBJ/PLY import/export, NURBS gallery,
  structure panel, face info, instance selection, GD&T, UV breakdown,
  manifold checks, progressive triangulation, mobile UI, web worker mode,
  IndexedDB cache, LOD selector.

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+Shift+P | Command palette |
| Ctrl+, | Options dialog |
| Ctrl+N | New document |
| Ctrl+O | Open file |
| Ctrl+S | Save file |
| Ctrl+D | Duplicate solid |
| Ctrl+Z | Undo |
| Ctrl+Shift+Z | Redo |
| F | Fit to view |
| S | Sketch mode |
| Space | Marking menu |
| 1-5 | Sketch tools (in sketch mode) |

## WASM Deployment Notes

### Cross-Origin Isolation

For WASM parallel threading (wasm-bindgen-rayon), the web server must send
these headers:

```
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```

Without these headers, the triangulation falls back to single-threaded mode
(still functional, just slower for large STEP files).

For GitHub Pages (which doesn't support custom headers), use the `web-deploy`
feature (no `wasm-parallel`), which works without Cross-Origin Isolation.

### Browser Support

- Chrome 113+ (WebGPU + WebGL2)
- Edge 113+ (WebGPU + WebGL2)
- Firefox with WebGL2 (WebGPU experimental)
- Safari 15+ (WebGL2 only)

### Build Verification

Both targets have been verified to build successfully:

- **Native**: `cargo build --bin brepcad-shell` — OK
- **WASM debug**: `cargo build --bin brepcad-shell --target wasm32-unknown-unknown --no-default-features --features web-deploy` — OK (~218 MB)
- **WASM release**: `cargo build --bin brepcad-shell --target wasm32-unknown-unknown --no-default-features --features web-deploy --release` — OK (~13 MB)
