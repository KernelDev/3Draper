# BRepCAD UI Implementation Plan

**Based on:** 96 SVG mockups in `docs/ui_mockups/`
**Application name:** BRepCAD (3Draper-based)
**Platform:** Desktop (Windows/Linux/macOS), Web (WASM)
**Theme:** Catppuccin Mocha dark (#1e1e2e base, #89b4fa accent)

**Live:** https://kerneldev.github.io/3Draper/brepcad.html

---

## Architecture (current)

```
brepcad-shell (binary, native + WASM)
  └─ ViewerApp::new(cc) + enable_brepcad_ui = true
       ├─ update():
       │    1. apply_brepcad_theme(ctx)          — Catppuccin Mocha
       │    2. render_menu_bar(ctx)              — 21 menus → MenuAction
       │    3. render_ribbon(ctx, &tab)          — 15 tabs → MenuAction
       │    4. Browser panel (left)              — Tree/Layers/Selection + filter
       │    5. Properties panel (right)          — Props/Constraints/Dims/Material
       │    6. Status bar (bottom)               — X/Y/Z, D, Tool, FPS, Display, View
       │    7. CentralPanel (3D viewport)        — wgpu SceneCallback (GL)
       │    8. View Cube + Display style switcher
       │    9. Command palette + dialogs + status toast
       └─ handle_brepcad_action()                — delegates to ViewerApp methods
```

**Key files:**
- `crates/draper-viewer/src/app.rs` — ViewerApp (12000+ lines), BRepCAD layout + theme
- `crates/draper-viewer/src/bin/brepcad_shell.rs` — thin wrapper (native + WASM)
- `crates/draper-viewer/src/ui/menubar.rs` — 21 menus, MenuAction enum
- `crates/draper-viewer/src/ui/ribbon.rs` — 15 ribbon tabs
- `crates/draper-viewer/src/ui/dialogs.rs` — 5 dialogs
- `crates/draper-viewer/src/ui/command_palette.rs` — 38 commands
- `crates/draper-viewer/src/ui/view_modes.rs` — View Cube + Display style

---

## Phase 0: Application Shell ✅ DONE

**Mockup:** `01_main_window.svg`

- [x] Title bar with document name + window controls
- [x] Menu bar (21 menus)
- [x] Ribbon tabs (15 tabs)
- [x] Quick Access Toolbar (Undo/Redo/Save in ribbon Home tab)
- [x] Command palette (Ctrl+Shift+P)
- [x] Left dock: Browser (Tree/Layers/Selection + filter)
- [x] Center viewport: 3D GL view (wgpu SceneCallback)
- [x] Right dock: Properties (Props/Constraints/Dimensions/Material)
- [x] Bottom status bar: X/Y/Z, D, Tool, FPS, Units, Display, View, Sel
- [x] Catppuccin Mocha dark theme
- [x] View Cube (8 orientations)
- [x] Display style switcher (Wireframe/Shaded/Shaded+Edges)

---

## Phase 1: Menu Bar (21 menus) ✅ DONE (form) / 🔄 Наполнение (in progress)

All 21 menus implemented. Status of each menu's functional wiring:

### 1.1. File Menu `07_menu_file.svg` ✅ DONE
- [x] New → load_box() (default model)
- [x] Open → rfd file dialog (STEP/STL/JSON)
- [x] Save / Save As → export_step()
- [x] Import STEP/STL/OBJ → import_step_file/import_stl_file
- [x] Export STEP/STL/OBJ → export_step/export_stl_binary
- [ ] Import PLY/DXF/Point Cloud — not yet implemented
- [ ] Export GLTF/PDF/DXF — not yet implemented
- [ ] Recent files list — not yet implemented
- [ ] Print/Plot dialog — not yet implemented

### 1.2. Edit Menu `08_menu_edit.svg` ✅ DONE
- [x] Undo/Redo — stub (returns "not yet implemented")
- [x] Duplicate — works (translate +20 X, push_undo)
- [ ] Cut/Copy/Paste — clipboard not yet implemented
- [ ] Find/Replace in parameters — not yet implemented
- [ ] History branching (snapshot/branch/diff/tree) — not yet implemented

### 1.3. View Menu `09_menu_view.svg` ✅ DONE
- [x] 8 orientations (ISO/Front/Back/Top/Bottom/Left/Right/Dimetric)
- [x] Fit/Zoom In/Out
- [x] Wireframe/Shaded/Shaded+Edges
- [x] Toggle Grid/Axis/Edges
- [ ] Zoom Window/Selection — not yet implemented
- [ ] Toggle Shadows/AO/AA/Normals/Silhouette — not yet implemented
- [ ] Perspective/Orthographic toggle — not yet implemented
- [ ] Save/Load Layout — not yet implemented

### 1.4. Insert Menu `10_menu_insert.svg` ✅ DONE
- [x] Box/Sphere/Cylinder/Cone/Torus → InsertPrimitive dialog → load_*
- [ ] Reference Geometry (Plane/Axis/Point/CS) — not yet implemented
- [ ] Sketch — not yet implemented (press S for sketch mode)
- [ ] Mesh operations (Import/From Solid/Remesh) — not yet implemented
- [ ] Component insertion — not yet implemented
- [ ] Pattern (Linear/Circular/Mirror) — Circular works, Linear/Mirror not yet

### 1.5. Sketch Menu `11_menu_sketch.svg` ⬜ NOT STARTED
- [ ] Enter/Exit sketch mode
- [ ] Draw tools: Line/Circle/Arc/Rectangle/Spline/Polygon/Point
- [ ] Constraints: Coincident/Collinear/Concentric/Parallel/Perpendicular/Tangent/Horizontal/Vertical/Equal
- [ ] Dimensions: Linear/Angular/Radial/Diameter
- [ ] Modify: Trim/Extend/Split/Offset/Mirror/Pattern/Fillet
- [ ] Sketch engine exists (ui/sketch.rs) but not wired to viewport

### 1.6. Modify Menu `12_menu_modify.svg` 🔄 PARTIAL
- [x] Boolean Union/Subtract/Intersect → model_boolean_*
- [x] Fillet/Chamfer → model_fillet_edge/model_chamfer_edge
- [x] Move/Rotate/Scale → model_translate/rotate/scale
- [x] Circular Pattern → model_circular_pattern
- [ ] Linear Pattern/Mirror — not yet implemented
- [ ] Loft/Sweep — not yet implemented (requires sketch profiles)
- [ ] Direct Modeling (Move/Offset/Delete/Replace/Split/Merge/Simplify/Thicken) — not yet
- [ ] Deform (Bend/Twist/Taper/Stretch) — not yet

### 1.7. Sheet Metal Menu `13_menu_sheetmetal.svg` ⬜ NOT STARTED
- [ ] Base/Edge Flange, Bend, Hem, Jog
- [ ] Relief (Rectangular/Tear)
- [ ] Unfold/Fold/Flat Pattern
- [ ] Gauge Table, K-Factor
- [ ] DXF export of flat pattern

### 1.8. Assembly Menu `14_menu_assembly.svg` ⬜ NOT STARTED
- [ ] Add Component (STEP/IGES/STL)
- [ ] Mate types: Coincident/Concentric/Distance/Angle/Parallel/Perpendicular/Tangent/Width/Symmetric
- [ ] Mate solver (3D constraint solving)
- [ ] BOM editor
- [ ] Exploded view
- [ ] Motion study (interference detection)

### 1.9. CAM Menu `15_menu_cam.svg` ⬜ NOT STARTED
- [ ] Stock Setup wizard
- [ ] Tool Library
- [ ] Operations: Facing/Profile/Pocket/Drilling/Engraving/3D Surfacing
- [ ] Simulation (2D/3D)
- [ ] G-code post-processors (7 dialects)

### 1.10. Drawing Menu `16_menu_drawing.svg` ⬜ NOT STARTED
- [ ] New Sheet (A0-A4)
- [ ] Views: Standard/Section/Detail/Projected/Broken-out/Crop/Auxiliary/Exploded
- [ ] Dimensions: Linear/Angular/Radial/Diameter/Ordinate
- [ ] Annotations: Note/Balloon/Surface Finish/Welding/Datum/Tolerance
- [ ] Title block editor, Revision table
- [ ] Export: PDF/DXF/DWG/SVG

### 1.11. Simulation Menu `17_menu_simulation.svg` ⬜ NOT STARTED
- [ ] Mesh generation (Tet4/Tet10/Hex8/Hex20)
- [ ] Study types: Static/Modal/Thermal/Buckling/Fatigue/Nonlinear/CFD/EM/Optimization
- [ ] Boundary conditions, Loads
- [ ] Solver (CG/direct)
- [ ] Results: Von Mises/Displacement/Strain/Stress/Animate

### 1.12. Parametric Menu `18_menu_parametric.svg` ⬜ NOT STARTED
- [ ] Parameters table (named params + formulas)
- [ ] Equations, Design Table
- [ ] Dependency Graph
- [ ] Variants
- [ ] Parameter-driven feature re-evaluation

### 1.13. Optimize Menu `19_menu_optimize_generate.svg` ⬜ NOT STARTED
- [ ] Topology Optimization (Lightweight/Stiff/Balanced)
- [ ] Generative Design (4 variants)

### 1.14. GD&T Menu `20_menu_gdt.svg` ⬜ NOT STARTED
- [ ] Datum, Form, Orientation, Position, Profile, Runout
- [ ] Analyze, Reports, Stackup Analysis

### 1.15. Heal Menu `21_menu_heal_inspect.svg` 🔄 PARTIAL
- [x] Heal (Stitch/Gap Fill/Remove Duplicates/Fix Orientation/Fix Degenerate/Simplify/Remove Sliver/Close Holes/Repair T-Junctions) → validation::heal_solid
- [x] Measure Area/Volume — computed from mesh
- [x] Analysis Watertight/Manifold — from manifold_report
- [ ] Measure Distance/Angle/Length/Diameter/Radius/Center — not yet
- [ ] Analysis Curvature/Draft/Thickness/Interference/Edge Consistency/Gaussian Curvature — not yet

### 1.16. Mold Menu `22_menu_mold.svg` ⬜ NOT STARTED
- [ ] Mold Base Catalog, Runner/Cooling/Ejection, Cavity/Core
- [ ] Flow/Cooling/Warpage Analysis

### 1.17. Tools Menu `23_menu_tools.svg` 🔄 PARTIAL
- [x] Options dialog (stub)
- [x] Plugins Manager dialog (stub)
- [x] Performance Monitor dialog (stub)
- [ ] Customize dialog — not yet
- [ ] Scripting console — not yet
- [ ] AI Settings — not yet
- [ ] Macro Recorder — not yet
- [ ] Theme switching — not yet
- [ ] UI Layout editor — not yet

### 1.18. Scripting Menu `24_menu_scripting.svg` ⬜ NOT STARTED
- [ ] Script List, Load Script, Record Macro, Run with Parameters
- [ ] Debug Step, Profile, Library Browser, API Reference

### 1.19. AI Menu `25_menu_ai.svg` ⬜ NOT STARTED
- [ ] Shape from Text, AI Assistant (Chat/Review/Cost/Suggest)
- [ ] Smart features (Auto-Fillet/Pattern/Repair/Dimension/Constrain)
- [ ] Generative design (4 variants)
- [ ] Topology optimization (4 presets)

### 1.20. Window Menu `26_menu_window.svg` ⬜ NOT STARTED
- [ ] Multiple document tabs, Close All/Cascade/Tile H/V
- [ ] Next/Prev tab, Save Layout

### 1.21. Help Menu `27_menu_help.svg` 🔄 PARTIAL
- [x] About dialog
- [ ] Check for Updates, Documentation, Forum, Report Bug
- [ ] Tutorials, Examples

---

## Phase 2: Ribbon (15 tabs) ✅ DONE (form) / 🔄 Наполнение (in progress)

All 15 ribbon tabs implemented. Buttons emit MenuAction.

### 2.1. File `28_ribbon_file.svg` ✅ DONE
- [x] New/Open/Save, Import STEP/STL, Export STEP/STL/OBJ

### 2.2. Home `29_ribbon_home.svg` ✅ DONE
- [x] Undo/Redo, Fit/ISO, Wireframe/Shaded/Shaded+Edges

### 2.3. Sketch `30_ribbon_sketch.svg` ⬜ NOT STARTED
- [ ] Enter/Exit, Line/Circle/Arc/Rect/Spline
- [ ] Constraints: Coincident/Parallel/Perpendicular/Tangent/H/V
- [ ] Dimensions: Linear/Angular/Radial

### 2.4. Insert `31_ribbon_insert.svg` ✅ DONE
- [x] Box/Sphere/Cylinder/Cone/Torus, Plane/Axis/Point, Import/Remesh, Pattern

### 2.5. Modify `32_ribbon_modify.svg` ✅ DONE
- [x] Union/Subtract/Intersect, Fillet/Chamfer, Move/Rotate/Scale, Direct Face ops

### 2.6. Sheet Metal `33_ribbon_sheetmetal.svg` ⬜ NOT STARTED
- [ ] Base/Edge Flange, Bend/Hem/Jog, Unfold/Fold/Flat/DXF, Gauge/K-Factor

### 2.7. Assembly `34_ribbon_assembly.svg` ⬜ NOT STARTED
- [ ] Insert/Replace Component, Mate Coincident/Concentric, Solve/Diagnose
- [ ] Explode/Motion, BOM

### 2.8. CAM `35_ribbon_cam.svg` ⬜ NOT STARTED
- [ ] Stock/Origin, Tool Library, 2.5-Axis ops, Simulate, G-code/NC View

### 2.9. Drawing `36_ribbon_drawing.svg` ⬜ NOT STARTED
- [ ] New Sheet, Views, Dimensions, Annotations, Export PDF/DXF

### 2.10. Simulation `37_ribbon_simulation.svg` ⬜ NOT STARTED
- [ ] Mesh/Material/BC/Load, Study types, Solve/Validate, Results/Animate

### 2.11. Inspect `38_ribbon_inspect.svg` 🔄 PARTIAL
- [x] Distance/Angle/Length/Area/Volume
- [x] Watertight/Manifold
- [ ] Curvature/Draft/Thickness — not yet
- [ ] Section/Compare/Heal — Heal works, Section/Compare not yet

### 2.12. AI `39_ribbon_ai.svg` ⬜ NOT STARTED
- [ ] Shape from Text, Variants, Optimize, Assistant, Smart

### 2.13. Tools `40_ribbon_tools.svg` ✅ DONE
- [x] Options/Customize/Plugins/Theme, Console/Macro, Monitor, Layout

### 2.14. View `41_ribbon_view.svg` ✅ DONE
- [x] ISO/Front/Top/Right, Fit/In/Out, Wireframe/Shaded/Edges, Persp/Ortho

### 2.15. Surface `95_ribbon_surface.svg` ⬜ NOT STARTED
- [ ] Loft/Sweep/Boundary/Fill/Network, G0/G1/G2 continuity

---

## Phase 3: View Modes ✅ DONE

- [x] Wireframe `42_mode_wireframe.svg`
- [x] Shaded `43_mode_shaded.svg`
- [x] Shaded + Edges `44_mode_shaded_edges.svg`
- [ ] Direct Modeling mode `45_mode_direct_modeling.svg`
- [ ] Drawing mode `46_mode_drawing.svg`
- [ ] Walkthrough mode `93_mode_walkthrough.svg`
- [ ] VR/AR mode `94_mode_vr_ar.svg`

---

## Phase 4: Sketch Engine ⬜ NOT STARTED

**Mockups:** `02_sketch_mode.svg`, `11_menu_sketch.svg`, `30_ribbon_sketch.svg`, `49_context_menu_sketch.svg`

**Existing code:** `ui/sketch.rs` (566 lines) — SketchEntity, DrawTool, Constraint, Dimension, Sketch, DrawState. NOT wired to viewport.

### 4.1. Sketch Mode Entry/Exit
- [ ] Press S or menu/ribbon to enter sketch mode
- [ ] Select sketch plane (XY/XZ/YZ/Custom)
- [ ] 2D canvas overlay on 3D viewport
- [ ] ESC to exit

### 4.2. Drawing Tools (8)
- [ ] Line (2 clicks)
- [ ] Circle (2 clicks: center + radius)
- [ ] Arc 3-Point (3 clicks)
- [ ] Arc Tangent
- [ ] Rectangle (2 clicks)
- [ ] Spline (N clicks, finish on Enter)
- [ ] Polygon (N sides)
- [ ] Point (1 click)

### 4.3. Constraints (9)
- [ ] Coincident, Collinear, Concentric
- [ ] Parallel, Perpendicular, Tangent
- [ ] Horizontal, Vertical, Equal
- [ ] Constraint solver (simplified: sequential)

### 4.4. Dimensions (4)
- [ ] Linear, Angular, Radial, Diameter
- [ ] Dimension display + editing

### 4.5. Modify (7)
- [ ] Trim, Extend, Split, Offset, Mirror, Pattern, Fillet

### 4.6. Context Menu `49_context_menu_sketch.svg`
- [ ] Right-click in sketch → context menu

---

## Phase 5: Panels (12) 🔄 PARTIAL

### 5.1. Browser `63_panel_browser.svg` ✅ DONE
- [x] 3 tabs: Tree/Layers/Selection
- [x] Filter input
- [x] Model tree (assembly tree + detailed_instances)
- [x] Instance selection (click → 3D highlight)
- [x] Instance visibility (eye icon)
- [x] Instance isolate (◎ button)
- [x] Face list under selected instance
- [x] Face visibility toggle
- [x] Face selection
- [ ] Right-click context menu `48_context_menu_browser.svg`

### 5.2. Properties `64_panel_properties.svg` ✅ DONE
- [x] 4 tabs: Props/Constraints/Dimensions/Material
- [x] Face properties (surface type, triangles, void flag)
- [x] Instance properties (name, BREP ID, faces)
- [x] Model info (name, vertices, triangles)
- [ ] Material assignment (stub only)

### 5.3. Timeline `65_panel_timeline.svg` ⬜ NOT STARTED
- [ ] Feature history timeline
- [ ] Rollback bar
- [ ] Reorder features

### 5.4. Measure `66_panel_measure.svg` 🔄 PARTIAL
- [x] Area/Volume (computed from mesh)
- [ ] Distance/Angle/Length (requires 2-point picking)
- [ ] Persistent measurement display

### 5.5. Section `67_panel_section.svg` ⬜ NOT STARTED
- [ ] Section plane (X/Y/Z/custom)
- [ ] Section cap fill
- [ ] Section measurements

### 5.6. AI Chat `68_panel_ai_chat.svg` ⬜ NOT STARTED
- [ ] Chat interface
- [ ] Design review, cost estimate

### 5.7. Scripting Console `69_panel_scripting_console.svg` ⬜ NOT STARTED
- [ ] Python/Lua console
- [ ] Command history
- [ ] Script execution

### 5.8. Performance Monitor `71_panel_performance_monitor.svg` ✅ DONE (stub)
- [x] FPS, draw calls, vertices, triangles (dialog)
- [ ] Real-time graphs

### 5.9. Cloud Collaboration `73_panel_cloud_collaboration.svg` ⬜ NOT STARTED
- [ ] Multi-user editing
- [ ] Comments, versions

### 5.10. FEA Mesh Control `85_panel_fea_mesh_control.svg` ⬜ NOT STARTED
- [ ] Element size, type, refinement

### 5.11. Animation Timeline `92_panel_animation_timeline.svg` ⬜ NOT STARTED
- [ ] Keyframe animation
- [ ] Play/pause/scrub

---

## Phase 6: Dialogs (26) 🔄 PARTIAL

### 6.1. Options `51_dialog_options.svg` ✅ DONE (stub)
- [x] 10 sections (General/Display/File Locations/Hotkeys/Theme/Advanced/Plugins/AI/Performance/Cloud)
- [ ] Settings persistence (JSON file)

### 6.2. Customize `52_dialog_customize.svg` ⬜ NOT STARTED
- [ ] Ribbon customization, shortcut editor, command reassignment

### 6.3. Insert Primitives `53_dialog_primitives.svg` ✅ DONE (custom dimensions)
- [x] Box/Sphere/Cylinder/Cone/Torus with parameters
- [x] Custom dimensions (sliders persist via egui memory)

### 6.4. Shortcut Editor `54_dialog_shortcut_editor.svg` ⬜ NOT STARTED
- [ ] 245+ commands, editable shortcuts

### 6.5. Command Search `55_dialog_command_search.svg` ✅ DONE
- [x] Fuzzy search palette (Ctrl+Shift+P)

### 6.6. Plugin Manager `56_dialog_plugin_manager.svg` ✅ DONE (stub)
- [x] Installed/Marketplace/Settings tabs
- [ ] Plugin loading/management

### 6.7. About `57_dialog_about.svg` ✅ DONE
- [x] Version, engine info

### 6.8. Check for Updates `58_dialog_update.svg` ⬜ NOT STARTED
- [ ] Version check, download

### 6.9. Material Editor `59_dialog_material_editor.svg` ⬜ NOT STARTED
- [ ] Density, Young's modulus, Poisson's ratio, color, thermal
- [ ] Material library import/export

### 6.10. Constraint Diagnostics `60_dialog_constraint_diagnostics.svg` ⬜ NOT STARTED
- [ ] Sketch constraint conflicts

### 6.11. Mold Catalog `61_dialog_mold_catalog.svg` ⬜ NOT STARTED
- [ ] Misumi/HASCO/DME/LKM

### 6.12. Render Settings `62_dialog_render_settings.svg` ⬜ NOT STARTED
- [ ] POV-Ray/Blender/LuxCore/Octane integration

### 6.13. Macro Recorder `70_dialog_macro_recorder.svg` ⬜ NOT STARTED
- [ ] Record/playback macros

### 6.14. Tutorial Browser `72_dialog_tutorial_browser.svg` ⬜ NOT STARTED
- [ ] Getting Started, Sketch, Assembly tutorials

### 6.15. Print/Plot `74_dialog_print_plot.svg` ⬜ NOT STARTED
- [ ] Print preview, paper size, orientation

### 6.16. License `75_dialog_license.svg` ⬜ NOT STARTED
- [ ] License management

### 6.17. Crash Recovery `76_dialog_crash_recovery.svg` ⬜ NOT STARTED
- [ ] Auto-save recovery

### 6.18. Onboarding `77_dialog_onboarding.svg` ⬜ NOT STARTED
- [ ] First-run wizard

### 6.19. CAM Stock Setup `82_wizard_cam_stock.svg` ⬜ NOT STARTED
- [ ] Stock dimensions, material

### 6.20. Tool Library `83_dialog_tool_library.svg` ⬜ NOT STARTED
- [ ] Tool types, parameters

### 6.21. NC Code Viewer `84_dialog_nc_code_viewer.svg` ⬜ NOT STARTED
- [ ] G-code display, syntax highlight

### 6.22. Modal Plotter `86_dialog_modal_plotter.svg` ⬜ NOT STARTED
- [ ] Frequency/mode visualization

### 6.23. Title Block Editor `87_dialog_title_block_editor.svg` ⬜ NOT STARTED
- [ ] Drawing title block fields

### 6.24. Revision Table `88_dialog_revision_table.svg` ⬜ NOT STARTED
- [ ] Drawing revision history

### 6.25. Layer Manager `89_dialog_layer_manager.svg` ✅ DONE (stub)
- [x] Layer list in Browser panel
- [ ] Full manager dialog

### 6.26. BOM Editor `90_dialog_bom_editor.svg` ⬜ NOT STARTED
- [ ] Bill of materials

### 6.27. Parameter Search/Replace `91_dialog_param_search_replace.svg` ⬜ NOT STARTED
- [ ] Find/replace in parameter table

---

## Phase 7: Context Menus (4) ⬜ NOT STARTED

### 7.1. Viewport `47_context_menu_viewport.svg`
- [ ] Right-click in 3D → Select/Isolate/Hide/Fit/View orientation

### 7.2. Browser `48_context_menu_browser.svg`
- [ ] Right-click tree node → Rename/Delete/Hide/Isolate/Copy

### 7.3. Sketch `49_context_menu_sketch.svg`
- [ ] Right-click in sketch → constraint/dimension options

### 7.4. Marking Menu `50_marking_menu.svg` ✅ DONE
- [x] Space key → 8-direction radial menu (stub with view options)

---

## Phase 8: Core Engine Features 🔄 PARTIAL

### 8.1. Selection System ✅ DONE
- [x] Click in 3D → select instance (Ctrl+click → select face)
- [x] Tree selection syncs with 3D
- [x] Face selection syncs with tree
- [ ] Box select (drag rectangle)
- [ ] Select by type (all faces, all edges)

### 8.2. Undo/Redo ✅ DONE
- [x] Snapshot-based undo (Vec<(Solid, name)> stack, 50 max)
- [ ] Linear history with branching
- [ ] History panel with diff

### 8.3. Parameter System ⬜ NOT STARTED
- [ ] Named parameters (length=100, radius=5)
- [ ] Formula support (length = width * 2)
- [ ] Design Table
- [ ] Parameter-driven feature re-evaluation

### 8.4. Material System 🔄 PARTIAL
- [x] Material struct (density, color, thermal, mechanical)
- [ ] Material assignment to faces/bodies
- [ ] Material library import/export

### 8.5. Layer System 🔄 PARTIAL
- [x] Layer list in Browser panel (stub)
- [ ] Per-layer visibility/color/line-weight
- [ ] Layer assignment

### 8.6. Plugin System ⬜ NOT STARTED
- [ ] Plugin API (Rust + Python + Lua)
- [ ] Plugin sandboxing

### 8.7. Theme System ✅ DONE
- [x] Catppuccin Mocha dark theme
- [ ] Light theme
- [ ] Custom theme editor

---

## Phase 9: Specialized Workspaces ⬜ NOT STARTED

### 9.1. Visual Programming `03_visual_programming.svg`
- [ ] Node graph editor (Grasshopper-style)

### 9.2. Surface Modeling `78_workspace_surface_modeling.svg`
- [ ] Loft/Sweep/Boundary/Fill/Network

### 9.3. Sheet Metal `05_sheetmetal_cam.svg`
- [ ] Flange/Bend/Hem/Jog/Relief/Unfold/Fold/Flat

### 9.4. CAM `05_sheetmetal_cam.svg`
- [ ] Stock/Tools/Operations/Simulate/Post

### 9.5. FEA `04_fea_analysis.svg`
- [ ] Mesh/Study/BC/Load/Solve/Results

### 9.6. Drawing `06_drawing_assembly.svg`
- [ ] Sheet/Views/Dimensions/Annotations/Export

### 9.7. Assembly `06_drawing_assembly.svg`
- [ ] Components/Mates/Solve/BOM/Explode/Motion

### 9.8. Point Cloud & RE `80_workflow_point_cloud.svg`, `81_workflow_reverse_engineering.svg`
- [ ] Import .ply/.xyz/.las, RANSAC, fit primitives, mesh from cloud

### 9.9. Mold Design `22_menu_mold.svg`
- [ ] Mold base, runner, cooling, ejection, cavity/core

### 9.10. AI Features `25_menu_ai.svg`
- [ ] Shape from Text, AI Assistant, Smart features, Generative design

---

## Phase 10: Implementation Priority

### Tier 1: Core CAD (NEXT — highest priority)
1. **Undo/Redo** — snapshot-based (8.2)
2. **Sketch mode** — drawing tools + constraints (Phase 4)
3. **Parameter system** — named params + formulas (8.3)
4. **Insert dialog dimensions** — custom sizes (6.3)
5. **Measure tools** — distance/angle/length (5.4)
6. **Section cut** — plane section (5.5)
7. **Feature timeline** — history + rollback (5.3)

### Tier 2: Advanced CAD
8. **Pattern operations** — Linear/Mirror (1.6)
9. **Loft/Sweep** — requires sketch (1.6)
10. **Reference geometry** — Plane/Axis/Point (1.4)
11. **Material assignment** — to faces/bodies (8.4)
12. **Context menus** — viewport/browser (Phase 7)

### Tier 3: Drawing & Assembly
13. **Drawing module** — sheets, views, dimensions (1.10)
14. **Assembly module** — components, mates (1.8)
15. **BOM editor** (6.26)

### Tier 4: CAE/CAM
16. **FEA** — mesh, study, solve, results (1.11)
17. **CAM** — toolpaths, G-code (1.9)
18. **Sheet Metal** — flange, bend, flat (1.7)

### Tier 5: Ecosystem
19. **Plugin system** (8.6)
20. **Scripting** (1.18)
21. **AI features** (1.19)
22. **Cloud collaboration** (5.9)
23. **VR/AR** (Phase 3)

---

## Summary

| Phase | Description | Status |
|-------|-------------|--------|
| 0 | Application Shell | ✅ DONE |
| 1 | Menu Bar (21 menus) | 🔄 PARTIAL (12/21 functional) |
| 2 | Ribbon (15 tabs) | 🔄 PARTIAL (8/15 functional) |
| 3 | View Modes | ✅ DONE (3/7 modes) |
| 4 | Sketch Engine | ⬜ NOT STARTED |
| 5 | Panels (12) | 🔄 PARTIAL (3/12 functional) |
| 6 | Dialogs (27) | 🔄 PARTIAL (6/27 functional) |
| 7 | Context Menus (4) | 🔄 PARTIAL (1/4 — marking menu) |
| 8 | Core Engine | 🔄 PARTIAL (3/7 features) |
| 9 | Workspaces (10) | ⬜ NOT STARTED |
| 10 | Tier 1 Priority | 🔄 IN PROGRESS |

**Overall: ~35% of forms have functional наполнение**
