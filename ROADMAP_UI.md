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
       │    9. Section Cut panel (floating)
       │   10. Sketch overlay (grid + entities)
       │   11. Measure overlay (result text)
       │   12. Command palette + dialogs + status toast
       │   13. Parameter dialog
       │   14. Feature Timeline panel
       └─ handle_brepcad_action()                — delegates to ViewerApp methods
```

**Key files:**
- `crates/draper-viewer/src/app.rs` — ViewerApp (14000+ lines), BRepCAD layout + theme + all features
- `crates/draper-viewer/src/bin/brepcad_shell.rs` — thin wrapper (native + WASM)
- `crates/draper-viewer/src/ui/menubar.rs` — 21 menus, MenuAction enum
- `crates/draper-viewer/src/ui/ribbon.rs` — 15 ribbon tabs
- `crates/draper-viewer/src/ui/dialogs.rs` — 5 dialogs with custom primitive dimensions
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

## Phase 1: Menu Bar (21 menus) ✅ DONE (form) / 🔄 Наполнение

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
- [x] Undo/Redo — snapshot-based (Vec<(Solid, name)> stack, 50 max)
- [x] Duplicate — works (translate +20 X, push_undo)
- [ ] Cut/Copy/Paste — clipboard not yet implemented
- [ ] Find/Replace in parameters — not yet implemented
- [ ] History branching (snapshot/branch/diff/tree) — not yet implemented

### 1.3. View Menu `09_menu_view.svg` ✅ DONE
- [x] 8 orientations (ISO/Front/Back/Top/Bottom/Left/Right/Dimetric)
- [x] Fit/Zoom In/Out
- [x] Wireframe/Shaded/Shaded+Edges
- [x] Toggle Grid/Axis/Edges
- [x] Section Cut (plane X/Y/Z + position slider)
- [x] Feature Timeline panel
- [ ] Zoom Window/Selection — not yet implemented
- [ ] Toggle Shadows/AO/AA/Normals/Silhouette — not yet implemented
- [ ] Perspective/Orthographic toggle — not yet implemented
- [ ] Save/Load Layout — not yet implemented

### 1.4. Insert Menu `10_menu_insert.svg` ✅ DONE
- [x] Box/Sphere/Cylinder/Cone/Torus → InsertPrimitive dialog with custom dimensions
- [ ] Reference Geometry (Plane/Axis/Point/CS) — not yet implemented
- [x] Sketch → enters sketch mode
- [ ] Mesh operations (Import/From Solid/Remesh) — not yet implemented
- [ ] Component insertion — not yet implemented
- [x] Linear Pattern → model_linear_pattern
- [x] Circular Pattern → model_circular_pattern
- [x] Mirror → model_mirror

### 1.5. Sketch Menu `11_menu_sketch.svg` ✅ DONE
- [x] Enter/Exit sketch mode (S key or menu)
- [x] 3 sketch planes (XY/XZ/YZ) with camera auto-align
- [x] Line (2 clicks)
- [x] Circle (2 clicks: center + radius)
- [x] Arc 3-Point (3 clicks)
- [x] Rectangle (2 clicks)
- [x] Point (1 click)
- [x] Grid overlay + snap to grid
- [x] Entity rendering (green lines, yellow endpoints, blue pending)
- [x] Extrude Sketch → 3D solid (Rectangle→Box, Circle→Cylinder)
- [ ] Spline/Polygon — not yet implemented
- [x] Horizontal/Vertical constraints (basic solver: forces Y/X equal)
- [x] Linear dimension (measures entity, adds to dimension list)
- [ ] Modify (Trim/Extend/Split/Offset/Mirror/Pattern/Fillet) — not yet

### 1.6. Modify Menu `12_menu_modify.svg` ✅ DONE
- [x] Boolean Union/Subtract/Intersect → model_boolean_*
- [x] Fillet/Chamfer → model_fillet_edge/model_chamfer_edge
- [x] Move/Rotate/Scale → model_translate/rotate/scale
- [x] Linear Pattern → model_linear_pattern (count copies + 50mm spacing)
- [x] Circular Pattern → model_circular_pattern
- [x] Mirror → model_mirror (about plane through origin)
- [x] Sweep = Extrude sketch (with distance dialog, Rectangle→Box, Circle→Cylinder)
- [ ] Direct Modeling (Move/Offset/Delete/Replace/Split/Merge/Simplify/Thicken) — not yet
- [ ] Deform (Bend/Twist/Taper/Stretch) — not yet

### 1.7. Sheet Metal Menu `13_menu_sheetmetal.svg` ⬜ NOT STARTED
- [ ] Base/Edge Flange, Bend, Hem, Jog
- [ ] Relief (Rectangular/Tear)
- [ ] Unfold/Fold/Flat Pattern
- [ ] Gauge Table, K-Factor
- [ ] DXF export of flat pattern

### 1.8. Assembly Menu `14_menu_assembly.svg` ✅ DONE
- [x] Add Component (STEP/STL) — imports file, adds to assembly list + BOM
- [x] Mate types: 9 mate types (select 2 entities, then apply)
- [ ] Mate solver (3D constraint solving) — mate selection UI
- [x] BOM editor — table with qty/material/weight, CSV export, auto-generate
- [x] Exploded view — toggle on/off
- [ ] Motion study (interference detection)

### 1.9. CAM Menu `15_menu_cam.svg` ⬜ NOT STARTED
- [ ] Stock Setup wizard
- [ ] Tool Library
- [ ] Operations: Facing/Profile/Pocket/Drilling/Engraving/3D Surfacing
- [ ] Simulation (2D/3D)
- [ ] G-code post-processors (7 dialects)

### 1.10. Drawing Menu `16_menu_drawing.svg` ✅ DONE
- [x] New Sheet (A0-A4) with sheet border + title block
- [x] Views: Standard(ISO+Front)/Section/Detail/Projected/Exploded
- [x] Dimensions: Linear(W,D,H from bbox)/Radial/Diameter/Angular/Ordinate
- [x] Annotations: Note/Balloon/Datum/Tolerance
- [x] Title block (model name, verts, tris, sheet size)
- [x] Export SVG (full sheet with views+dims+annotations)

### 1.11. Simulation Menu `17_menu_simulation.svg` ⬜ NOT STARTED
- [ ] Mesh generation (Tet4/Tet10/Hex8/Hex20)
- [ ] Study types: Static/Modal/Thermal/Buckling/Fatigue/Nonlinear/CFD/EM/Optimization
- [ ] Boundary conditions, Loads
- [ ] Solver (CG/direct)
- [ ] Results: Von Mises/Displacement/Strain/Stress/Animate

### 1.12. Parametric Menu `18_menu_parametric.svg` ✅ DONE
- [x] Parameters table (named params + formulas) — dialog with add/edit/delete
- [x] Formula evaluator (recursive descent: +,-,*,/, parentheses, param refs)
- [x] "Add Defaults" (Width, Height, Depth, Radius, Diameter=Radius*2, Volume=W*H*D)
- [x] Re-evaluate formulas on change
- [ ] Equations, Design Table — not yet
- [ ] Dependency Graph — not yet
- [ ] Variants — not yet
- [ ] Parameter-driven feature re-evaluation — not yet

### 1.13. Optimize Menu `19_menu_optimize_generate.svg` ⬜ NOT STARTED
- [ ] Topology Optimization (Lightweight/Stiff/Balanced)
- [ ] Generative Design (4 variants)

### 1.14. GD&T Menu `20_menu_gdt.svg` ⬜ NOT STARTED
- [ ] Datum, Form, Orientation, Position, Profile, Runout
- [ ] Analyze, Reports, Stackup Analysis

### 1.15. Heal Menu `21_menu_heal_inspect.svg` ✅ DONE
- [x] Heal (Stitch/Gap Fill/Remove Duplicates/Fix Orientation/Fix Degenerate/Simplify/Remove Sliver/Close Holes/Repair T-Junctions) → validation::heal_solid
- [x] Measure Distance/Angle/Length — 2-point/3-point picking in viewport
- [x] Measure Area/Volume — computed from mesh
- [x] Analysis Watertight/Manifold — from manifold_report
- [ ] Measure Diameter/Radius/Center — not yet
- [ ] Analysis Curvature/Draft/Thickness/Interference/Edge Consistency/Gaussian Curvature — not yet

### 1.16. Mold Menu `22_menu_mold.svg` ⬜ NOT STARTED
- [ ] Mold Base Catalog, Runner/Cooling/Ejection, Cavity/Core
- [ ] Flow/Cooling/Warpage Analysis

### 1.17. Tools Menu `23_menu_tools.svg` 🔄 PARTIAL
- [x] Options dialog (10 sections)
- [x] Plugins Manager dialog (stub)
- [x] Performance Monitor dialog (stub)
- [ ] Customize dialog — not yet
- [ ] Scripting console — not yet
- [ ] AI Settings — not yet
- [ ] Macro Recorder — not yet
- [ ] Theme switching — not yet
- [ ] UI Layout editor — not yet

### 1.18. Scripting Menu `24_menu_scripting.svg` ⬜ NOT STARTED
### 1.19. AI Menu `25_menu_ai.svg` ⬜ NOT STARTED
### 1.20. Window Menu `26_menu_window.svg` ⬜ NOT STARTED

### 1.21. Help Menu `27_menu_help.svg` 🔄 PARTIAL
- [x] About dialog
- [ ] Check for Updates, Documentation, Forum, Report Bug
- [ ] Tutorials, Examples

---

## Phase 2: Ribbon (15 tabs) ✅ DONE (form) / 🔄 Наполнение

### 2.1. File `28` ✅ DONE
### 2.2. Home `29` ✅ DONE
### 2.3. Sketch `30` ✅ DONE (5 tools + enter/exit)
### 2.4. Insert `31` ✅ DONE (primitives + patterns + mirror)
### 2.5. Modify `32` ✅ DONE (boolean + fillet/chamfer + transform + patterns)
### 2.6. Sheet Metal `33` ⬜ NOT STARTED
### 2.7. Assembly `34` ⬜ NOT STARTED
### 2.8. CAM `35` ⬜ NOT STARTED
### 2.9. Drawing `36` ⬜ NOT STARTED
### 2.10. Simulation `37` ⬜ NOT STARTED
### 2.11. Inspect `38` ✅ DONE (measure + watertight/manifold + heal + section)
### 2.12. AI `39` ⬜ NOT STARTED
### 2.13. Tools `40` ✅ DONE
### 2.14. View `41` ✅ DONE
### 2.15. Surface `95` ⬜ NOT STARTED

---

## Phase 3: View Modes ✅ DONE

- [x] Wireframe `42`
- [x] Shaded `43`
- [x] Shaded + Edges `44`
- [ ] Direct Modeling mode `45`
- [ ] Drawing mode `46`
- [ ] Walkthrough mode `93`
- [ ] VR/AR mode `94`

---

## Phase 4: Sketch Engine ✅ DONE (core)

**Mockups:** `02_sketch_mode.svg`, `11_menu_sketch.svg`, `30_ribbon_sketch.svg`, `49_context_menu_sketch.svg`

### 4.1. Sketch Mode Entry/Exit ✅ DONE
- [x] Press S or menu/ribbon to enter sketch mode
- [x] Select sketch plane (XY/XZ/YZ)
- [x] Camera auto-aligns to sketch plane
- [x] 2D canvas overlay on 3D viewport
- [x] ESC to exit

### 4.2. Drawing Tools ✅ DONE (5/8)
- [x] Line (2 clicks)
- [x] Circle (2 clicks: center + radius)
- [x] Arc 3-Point (3 clicks)
- [x] Rectangle (2 clicks)
- [x] Point (1 click)
- [ ] Spline (N clicks, finish on Enter)
- [ ] Polygon (N sides)
- [ ] Arc Tangent

### 4.3. Grid + Snap ✅ DONE
- [x] Grid overlay (20×20 lines, 10mm step)
- [x] Snap to grid (configurable)
- [x] Entity rendering (green lines, yellow endpoints, blue pending)

### 4.4. Extrude ✅ DONE
- [x] Rectangle → Box
- [x] Circle → Cylinder
- [x] Positions on sketch plane (XY/XZ/YZ)

### 4.5. Constraints ⬜ NOT STARTED
- [ ] Coincident, Collinear, Concentric
- [ ] Parallel, Perpendicular, Tangent
- [ ] Horizontal, Vertical, Equal
- [ ] Constraint solver

### 4.6. Dimensions ⬜ NOT STARTED
- [ ] Linear, Angular, Radial, Diameter

### 4.7. Modify ⬜ NOT STARTED
- [ ] Trim, Extend, Split, Offset, Mirror, Pattern, Fillet

---

## Phase 5: Panels (12) 🔄 PARTIAL

### 5.1. Browser `63` ✅ DONE
- [x] 3 tabs: Tree/Layers/Selection
- [x] Filter input
- [x] Model tree (assembly tree + detailed_instances)
- [x] Instance selection (click → 3D highlight)
- [x] Instance visibility (eye icon)
- [x] Instance isolate (◎ button)
- [x] Face list under selected instance
- [x] Face visibility toggle
- [x] Face selection
- [x] Right-click context menu `48` — viewport context menu done

### 5.2. Properties `64` ✅ DONE
- [x] 4 tabs: Props/Constraints/Dimensions/Material
- [x] Face properties (surface type, triangles, void flag)
- [x] Instance properties (name, BREP ID, faces)
- [x] Model info (name, vertices, triangles)
- [x] Material assignment — 10 presets, real colors applied to mesh, remove/assign, per-instance

### 5.3. Timeline `65` ✅ DONE
- [x] Feature history timeline (named operations + solid snapshots)
- [x] Rollback bar (click ↩ to restore any point)
- [x] Visual state (● active, ○ rolled-back)
- [x] Clear timeline
- [ ] Reorder features

### 5.4. Measure `66` ✅ DONE
- [x] Distance (2-point picking)
- [x] Angle (3-point picking: vertex + 2 points)
- [x] Length (2-point)
- [x] Area (computed from mesh)
- [x] Volume (computed from mesh)
- [x] Result overlay in viewport
- [x] ESC to cancel

### 5.5. Section `67` ✅ DONE
- [x] Section plane (X/Y/Z)
- [x] Position slider (bbox min..max)
- [x] Fit button (center)
- [x] Real-time triangle filtering
- [x] Floating panel UI
- [ ] Section cap fill
- [ ] Section measurements

### 5.6. AI Chat `68` ⬜ NOT STARTED
### 5.7. Scripting Console `69` ⬜ NOT STARTED
### 5.8. Performance Monitor `71` ✅ DONE (stub)
### 5.9. Cloud Collaboration `73` ⬜ NOT STARTED
### 5.10. FEA Mesh Control `85` ⬜ NOT STARTED
### 5.11. Animation Timeline `92` ⬜ NOT STARTED

---

## Phase 6: Dialogs (27) 🔄 PARTIAL

### 6.1. Options `51` ✅ DONE (stub, 10 sections)
### 6.2. Customize `52` ⬜ NOT STARTED
### 6.3. Insert Primitives `53` ✅ DONE (custom dimensions, persistent sliders)
### 6.4. Shortcut Editor `54` ⬜ NOT STARTED
### 6.5. Command Search `55` ✅ DONE (Ctrl+Shift+P)
### 6.6. Plugin Manager `56` ✅ DONE (stub)
### 6.7. About `57` ✅ DONE
### 6.8-6.27: ⬜ NOT STARTED (Update/Material/Constraint/Mold/Render/Macro/Tutorial/Print/License/Crash/Onboarding/CAM Stock/Tool Library/NC Viewer/Modal Plotter/Title Block/Revision Table/Layer Manager/BOM/Param Search)

---

## Phase 7: Context Menus (4) ⬜ NOT STARTED

### 7.1. Viewport `47` ✅ DONE
### 7.2. Browser `48` ⬜ NOT STARTED
### 7.3. Sketch `49` ⬜ NOT STARTED
### 7.4. Marking Menu `50` ✅ DONE (Space key, view options)

---

## Phase 8: Core Engine Features 🔄 PARTIAL

### 8.1. Selection System ✅ DONE
- [x] Click in 3D → select instance (Ctrl+click → select face)
- [x] Tree selection syncs with 3D
- [x] Face selection syncs with tree
- [x] Face visibility toggle from tree
- [ ] Box select (drag rectangle)
- [ ] Select by type (all faces, all edges)

### 8.2. Undo/Redo ✅ DONE
- [x] Snapshot-based undo (Vec<(Solid, name)> stack, 50 max)
- [x] Named operations for timeline
- [x] Ctrl+Z / Ctrl+Shift+Z
- [ ] Linear history with branching
- [ ] History panel with diff

### 8.3. Parameter System ✅ DONE
- [x] Named parameters (name, value, formula, unit)
- [x] Formula evaluator (recursive descent: +,-,*,/, parentheses, param refs)
- [x] Parameter dialog (add/edit/delete, defaults, clear)
- [x] Re-evaluate formulas on change
- [ ] Design Table
- [ ] Dependency Graph
- [ ] Parameter-driven feature re-evaluation

### 8.4. Material System ✅ DONE
- [x] Material struct (density, color, thermal, mechanical)
- [x] Material assignment to instances + single solid
- [x] Material library (10 presets: Steel, Aluminum, Copper, Brass, Titanium, ABS, Nylon, Glass, Wood, Ceramic)

### 8.5. Layer System 🔄 PARTIAL
- [x] Layer list in Browser panel (stub)
- [ ] Per-layer visibility/color/line-weight
- [ ] Layer assignment

### 8.6. Plugin System ⬜ NOT STARTED
### 8.7. Theme System ✅ DONE (Catppuccin Mocha dark)

---

## Phase 9: Specialized Workspaces ⬜ NOT STARTED

### 9.1. Visual Programming `03` ⬜ NOT STARTED
### 9.2. Surface Modeling `78` ⬜ NOT STARTED
### 9.3. Sheet Metal `05` ⬜ NOT STARTED
### 9.4. CAM `05` ⬜ NOT STARTED
### 9.5. FEA `04` ⬜ NOT STARTED
### 9.6. Drawing `06` ⬜ NOT STARTED
### 9.7. Assembly `06` ⬜ NOT STARTED
### 9.8. Point Cloud & RE `80`,`81` ⬜ NOT STARTED
### 9.9. Mold Design `22` ⬜ NOT STARTED
### 9.10. AI Features `25` ⬜ NOT STARTED

---

## Phase 10: Implementation Priority

### Tier 1: Core CAD ✅ ALL DONE
1. ✅ Undo/Redo — snapshot-based (8.2)
2. ✅ Sketch mode — 5 drawing tools + grid + extrude (Phase 4)
3. ✅ Parameter system — named params + formulas (8.3)
4. ✅ Insert dialog dimensions — custom sizes (6.3)
5. ✅ Measure tools — distance/angle/length (5.4)
6. ✅ Section cut — plane X/Y/Z + slider (5.5)
7. ✅ Feature timeline — history + rollback (5.3)

### Tier 2: Advanced CAD 🔄 PARTIAL
8. ✅ Pattern operations — Linear/Circular/Mirror (1.6)
9. ✅ Extrude Sketch — Rectangle→Box, Circle→Cylinder (Phase 4)
10. [ ] Reference geometry — Plane/Axis/Point (1.4)
11. [ ] Material assignment — to faces/bodies (8.4)
12. [ ] Context menus — viewport/browser (Phase 7)
13. [ ] Loft/Sweep — requires sketch profiles (1.6)

### Tier 3: Drawing & Assembly ⬜ NOT STARTED
14. [ ] Drawing module — sheets, views, dimensions (1.10)
15. [ ] Assembly module — components, mates (1.8)
16. [ ] BOM editor (6.26)

### Tier 4: CAE/CAM ⬜ NOT STARTED
17. [ ] FEA — mesh, study, solve, results (1.11)
18. [ ] CAM — toolpaths, G-code (1.9)
19. [ ] Sheet Metal — flange, bend, flat (1.7)

### Tier 5: Ecosystem ⬜ NOT STARTED
20. [ ] Plugin system (8.6)
21. [ ] Scripting (1.18)
22. [ ] AI features (1.19)
23. [ ] Cloud collaboration (5.9)
24. [ ] VR/AR (Phase 3)

---

## Summary

| Phase | Description | Status |
|-------|-------------|--------|
| 0 | Application Shell | ✅ DONE |
| 1 | Menu Bar (21 menus) | ✅ 15/21 functional |
| 2 | Ribbon (15 tabs) | ✅ 10/15 functional |
| 3 | View Modes | ✅ 3/7 modes |
| 4 | Sketch Engine | ✅ DONE (5 tools + grid + extrude) |
| 5 | Panels (12) | ✅ 5/12 functional |
| 6 | Dialogs (27) | ✅ 6/27 functional |
| 7 | Context Menus (4) | 🔄 1/4 (marking menu) |
| 8 | Core Engine | ✅ 5/7 features |
| 9 | Workspaces (10) | ⬜ NOT STARTED |
| 10 | Tier 1 | ✅ ALL DONE |
| 10 | Tier 2 | 🔄 2/5 done |

**Overall: ~50% of forms have functional наполнение**
