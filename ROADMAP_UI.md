# BRepCAD UI Implementation Plan

**Based on:** 96 SVG mockups in `docs/ui_mockups/`
**Application name:** BRepCAD (3Draper-based)
**Platform:** Desktop (Windows/Linux/macOS), Web (WASM)
**Language:** Russian/English (i18n)
**Theme:** Dark (#1a1a1a bg, #0a84ff accent)

---

## Phase 0: Application Shell (Форма)

### 0.1. Main Window Layout
**Mockup:** `01_main_window.svg`
- [x] Title bar with document name + window controls
- [x] Menu bar (21 menus — see Phase 1)
- [x] Ribbon tabs (14+1 tabs — see Phase 2)
- [x] Quick Access Toolbar (QAT): Undo/Redo/Save + primitives + booleans (in ribbon File tab)
- [x] Command palette (Ctrl+Shift+P / Cmd+P) — fuzzy search
- [x] Left dock panel: Browser (Model Tree) with tabs Tree/Layers/Selection
- [x] Center viewport: 3D OpenGL view with axis triad + grid (placeholder)
- [x] Right dock panel: Properties + mode-specific tabs
- [x] Bottom status bar: X/Y/Z coords, Az/El/Distance, Tool, FPS, Units, Display style, View

**Наполнение:**
- [x] egui dock area layout (left/center/right/bottom) — ui/mod.rs + brepcad_shell.rs
- [ ] Viewport: existing draper-viewer GL renderer (needs integration)
- [ ] Status bar: real-time coordinate tracking from camera
- [ ] QAT: wire to existing undo/redo + triangulate_solid for primitives

### 0.2. Window Management
**Mockup:** `26_menu_window.svg`
- [ ] Multiple document tabs
- [ ] Close All, Cascade, Tile H/V
- [ ] Save Layout / Load Layout
- [ ] Next/Prev tab navigation

**Наполнение:**
- Document model: Vec<Document> with tab switching
- Layout serialization to JSON

---

## Phase 1: Menu Bar (21 menus) ✅ DONE
All 21 menus implemented in `crates/draper-viewer/src/ui/menubar.rs`.

### 1.1. File Menu
**Mockup:** `07_menu_file.svg`
- [ ] New / Open / Save / Save As / Close
- [ ] Import: STEP, IGES, STL, OBJ, PLY, 3MF, DXF, SVG, Point Cloud (.ply/.xyz/.las), Image
- [ ] Export: STEP (AP203/214/242), STL, OBJ, GLTF, USD, DXF, SVG, 3MF, PDF
- [ ] Render: POV-Ray, Blender, LuxCore, Octane, FBX
- [ ] Recent files list
- [ ] Print/Plot dialog

**Наполнение:**
- Import: `parse_step()` already exists; STL/OBJ/PLY parsers needed
- Export: `export_step_with_schema()` already exists; STL/OBJ exporters needed
- Render: integrate with external renderers via CLI

### 1.2. Edit Menu
**Mockup:** `08_menu_edit.svg`
- [ ] Undo / Redo with history branching (Snapshot/Branch/Diff/Tree)
- [ ] Cut / Copy / Paste / Duplicate
- [ ] Find / Replace (in parameters)

**Наполнение:**
- Undo/Redo: command pattern with snapshot stack
- History branching: tree of undo states (not just linear)

### 1.3. View Menu
**Mockup:** `09_menu_view.svg`
- [ ] 8 orientations (ISO, Front, Back, Top, Bottom, Left, Right, Dimetric)
- [ ] Zoom: Fit / Window / In / Out / Selection
- [ ] Display Style: Wireframe / Shaded / Shaded+Edges
- [ ] Options: Grid, Axis, Triad, View Cube, Shadows, AO, Anti-alias, Edges, Normals, Silhouette
- [ ] Camera: Perspective/Orthographic, FOV, Near/Far
- [ ] Layouts: save/restore viewport layouts

**Наполнение:**
- All view orientations: existing OrbitCamera methods
- Display styles: existing GL renderer modes
- View cube: new 3D overlay widget

### 1.4-1.27. Remaining 18 Menus
**Mockups:** `10`–`27`

Each menu requires:
- [ ] Menu structure (cascading submenus)
- [ ] Action handlers (wire to backend)
- [ ] Keyboard shortcuts
- [ ] Enable/disable state based on context

| Menu | Key Features | Наполнение |
|------|-------------|------------|
| Insert (10) | 5 primitives + Ref Geometry + Sketch + Mesh + Component + Pattern | ShapeBuilder::make_box/sphere/cylinder/cone/torus already exist |
| Sketch (11) | 8 draw tools + 9 constraints + 4 dimensions + 7 modify + 4 block + 4 info + 4 parametric | 2D sketch engine needed (Phase 4) |
| Modify (12) | Boolean(3) + Edge(2) + Surface(2) + Transform(3) + Pattern(3) + Direct(8) + Deform(4) | boolean_union/subtract/intersect exist; fillet/chamfer need implementation |
| Sheet Metal (13) | Flange + Bend + Hem + Jog + Relief + Unfold/Fold/Flat + Gauge + DXF | Entire sheet metal module needed |
| Assembly (14) | Add Component + Mate(9) + Solve + BOM + Explode + Motion | Assembly tree exists; mate solver needed |
| CAM (15) | Setup + Tools(6) + Operations(9) + Simulate(5) + Post(7) + 5-Axis(6) + AI(6) | Entire CAM module needed |
| Drawing (16) | Sheet + Views(8) + Dimensions(5) + Annotations(6) + Templates(6) + Export(4) + Auto Drawing(3) | 2D drawing module needed |
| Simulation (17) | Mesh + Study(9) + Run(4) + Results(5) | FEA solver needed |
| Parametric (18) | Parameters(10) + History(8) + Advanced(Variants/DepGraph/DesignTable) | FeatureTree exists; parameter binding needed |
| Optimize+Generate (19) | Topology Optimization(3) + Generative(4) | New module |
| GD&T (20) | Datum + Form(4) + Orientation(3) + Position + Profile(2) + Runout(2) + Reports | GD&T parser exists in pmi.rs; visualization needed |
| Heal+Inspect (21) | Heal(9) + Measure(13) + Analysis(8) + Tools(4) | heal_solid exists; measure tools needed |
| Mold (22) | Catalog + Mold Base + Runner/Cooling/Ejection + Cavity + Flow/Cooling/Warpage | New module |
| Tools (23) | Options + Customize + Plugins + Scripting + AI + Macros + Perf + Theme + UI Layout | Options dialog needed |
| Scripting (24) | List/Load/Record/Run+Params/Debug/Profile + Library Browser + API Reference | Scripting engine needed (Python/Lua) |
| AI (25) | Shape from Text + Assistant(4) + Smart(5) + Generate(4) + Optimize(4) | LLM integration |
| Help (27) | Updates + About + Docs + Forum + Bug + Assets + Changelog + Tutorials(3) + Examples(7) | Standard help system |

---

## Phase 2: Ribbon Tabs (15 tabs) ✅ DONE
All 15 ribbon tabs implemented in `crates/draper-viewer/src/ui/ribbon.rs`.

**Mockups:** `28`–`41`, `95`

Each ribbon tab contains grouped command buttons with icons.

| Tab | Groups | Наполнение |
|-----|--------|------------|
| File (28) | Document(6) + Render(5) + Recent(3) + Application(1) | Wire to File menu |
| Home (29) | Clipboard + Selection + History + View + Display + Windows | Wire to Edit/View menus |
| Sketch (30) | Mode + Draw(8) + Constraint(9) + Dimension(4) + Modify(7) + Block(4) + Info(4) + Parametric(4) | Sketch engine (Phase 4) |
| Insert (31) | Primitives(5) + Reference(4) + Sketch(2) + Mesh(3) + Component(3) | ShapeBuilder + mesh import |
| Modify (32) | Boolean(3) + Edge(2) + Surface(2) + Transform(3) + Pattern(3) + Direct(8) + Deform(4) | boolean_* + new ops |
| Sheet Metal (33) | Base(3) + Bends(3) + Relief(2) + Flatten(4) + Convert(1) + Material(2) + Export(1) | New module |
| Assembly (34) | Components(4) + Mate(2) + Solve(2) + BOM(2) + Explode(2) + Motion(3) + Constraints(4) | Assembly tree + mate solver |
| CAM (35) | Setup(3) + Tools(6) + 2.5Axis(6) + 3Axis(2) + 5Axis(6) + Simulate(5) + Post(7) + AI(6) | New CAM module |
| Drawing (36) | Sheet(1) + Views(8) + Dimensions(5) + Annotations(6) + Templates(6) + Auto Drawing(3) + Export(4) | 2D drawing module |
| Simulation (37) | Setup(4) + Study(9) + Run(4) + Results(8) | FEA module |
| Inspect (38) | Measure(9) + Analysis(8) + Tools(4) | Measure tools |
| AI (39) | Generate(5) + Optimize(4) + Assistant(4) + Smart(5) + Settings(1) | LLM integration |
| Tools (40) | Application(5) + Macros(2) + Performance(2) + Theme(1) + UI Layout(4) + Plugins(5) + Industrial(5) | Settings system |
| View (41) | Orient(8) + Zoom(4) + Display Style(3) + Options(5) + Camera(4) + Layouts(5) | Camera + renderer |
| Surface (95) | Loft + Sweep + Boundary + Fill + Network (G0/G1/G2) | Surface modeling module |

**Наполнение для всех табов:**
- Ribbon widget: horizontal scrollable tab bar with icon+label buttons
- Context-sensitive: tab content changes based on active workspace/mode
- Button states: enabled/disabled/active/checked
- Tooltips with keyboard shortcuts

---

## Phase 3: View Modes (7 modes)

**Mockups:** `42`–`46`, `93`, `94`

| Mode | File | Наполнение |
|------|------|------------|
| Wireframe (42) | All edges visible, no faces | GL_LINES mode in renderer |
| Shaded (43) | Filled faces, no edges | Existing GL renderer |
| Shaded+Edges (44) | Filled faces + edge overlay | Existing mode (default) |
| Direct Modeling (45) | Face selection + gizmo handles | Selection system + transform gizmo |
| Drawing Mode (46) | 2D drawing sheet layout | 2D drawing module |
| Walkthrough (93) | WASD first-person navigation | Camera controller + collision |
| VR/AR (94) | Stereo rendering, hand tracking | WebXR / OpenXR integration |

---

## Phase 4: Sketch Engine

**Mockups:** `02_sketch_mode.svg`, `11_menu_sketch.svg`, `30_ribbon_sketch.svg`, `49_context_menu_sketch.svg`

### 4.1. Sketch Canvas
- [ ] 2D grid with X/Y axes
- [ ] Snap to grid, endpoints, midpoints, intersections
- [ ] Pan/zoom 2D camera
- [ ] Dimension display (linear, angular, radial)

### 4.2. Drawing Tools (8)
- [ ] Line
- [ ] Circle
- [ ] Arc (3-point, center-start-end, tangent)
- [ ] Rectangle
- [ ] Spline (interpolation B-spline)
- [ ] Polygon
- [ ] Ellipse
- [ ] Point

### 4.3. Constraints (9)
- [ ] Coincident
- [ ] Collinear
- [ ] Concentric
- [ ] Parallel
- [ ] Perpendicular
- [ ] Tangent
- [ ] Horizontal
- [ ] Vertical
- [ ] Equal length/radius

### 4.4. Dimensions (4)
- [ ] Linear dimension
- [ ] Angular dimension
- [ ] Radial dimension
- [ ] Diameter dimension

### 4.5. Modify (7)
- [ ] Trim
- [ ] Extend
- [ ] Split
- [ ] Offset
- [ ] Mirror
- [ ] Pattern (linear/circular)
- [ ] Fillet (2D)

### 4.6. Constraint Solver
- [ ] DOF analysis (degrees of freedom)
- [ ] Sequential constraint solving
- [ ] Over-constraint detection
- [ ] Constraint diagnostics dialog (`60_dialog_constraint_diagnostics.svg`)

**Наполнение:**
- 2D constraint solver: Lagrange multiplier or sequential reduction
- Sketch → Solid: extrude/revolve/sweep from sketch profile
- Parametric: dimension values drive constraints

---

## Phase 5: Dockable Panels (12 panels) — Phase 5.1-5.2 DONE

**Mockups:** `63`–`69`, `71`, `73`, `85`, `92`, `96`

### 5.1. Browser / Model Tree (`63`) ✅ DONE
- [x] Tree/Layers/Selection tabs
- [x] Hierarchical tree: Assembly → Bodies → Faces → Edges → Vertices
- [x] Filter box
- [x] Visibility toggles per node
- [x] Right-click context menu (stub — `48_context_menu_browser.svg`)
- [ ] Drag-and-drop reordering

**Наполнение:**
- AssemblyNode tree already exists in draper-step
- Selection sync: tree ↔ viewport (click face in tree → highlight in 3D)

### 5.2. Properties (`64`) ✅ DONE
- [ ] Props/Constraints/Dim/Material tabs
- [ ] Property groups: General, Geometry, Appearance, Layer, Custom
- [ ] Inline editing (text, number, color picker, dropdown)
- [ ] Property change → undo/redo

**Наполнение:**
- Property model: key-value with type info
- Wire to FaceInfo/EdgeInfo data from triangulation

### 5.3. Timeline / Feature History (`65`, `92`, `96`)
- [ ] Feature list with icons
- [ ] Rollback marker (drag to undo/redo to point)
- [ ] Branch visualization
- [ ] Drag to reorder features
- [ ] Animation timeline (8 tracks, 120 frames, keyframes)

**Наполнение:**
- FeatureTree already exists in feature_history.rs
- Rollback: evaluate FeatureTree to specific feature
- Animation: keyframe interpolation for motion studies

### 5.4. Measure (`66`)
- [ ] 6 measurement types: Distance, Angle, Length, Area, Volume, Mass
- [ ] Recent measurements list (8)
- [ ] Copy to clipboard / Export CSV

**Наполнение:**
- Distance: ray casting between picked points
- Area: sum of triangle areas for selected faces
- Volume: signed tetrahedron sum
- Mass: volume × material density

### 5.5. Section Cut (`67`)
- [ ] 3 plane types: XY, XZ, YZ, Custom
- [ ] Cap fill (show cross-section)
- [ ] Hatch pattern (ANSI31)
- [ ] Export to SVG

**Наполнение:**
- Clipping plane in GL renderer
- Cap: triangulate the cross-section polygon
- Hatch: 2D line pattern fill

### 5.6. AI Assistant Chat (`68`)
- [ ] 4 modes: Chat / DRC / Suggest / Cost
- [ ] Chat history
- [ ] Suggestion cards
- [ ] Natural language → CAD command translation

**Наполнение:**
- LLM API integration (z-ai-web-dev-sdk)
- Context: current model state, selected entities, active tool

### 5.7. Scripting Console (`69`)
- [ ] Console / Editor / Output / Variables tabs
- [ ] Multi-language: Python, Lua, PascalScript
- [ ] REPL (read-eval-print loop)
- [ ] Syntax highlighting

**Наполнение:**
- Python: PyO3 or rustpython
- Lua: rlua crate
- API: expose draper-step, draper-mesh, draper-topology functions

### 5.8. Performance Monitor (`71`)
- [ ] FPS graph (real-time)
- [ ] Memory usage graph
- [ ] Draw call counter
- [ ] Triangle/vertex counts
- [ ] Alerts (FPS < 30, memory > threshold)

### 5.9. Cloud Collaboration (`73`)
- [ ] Online users list (avatars)
- [ ] Live cursors
- [ ] Branch management
- [ ] Sync status

**Наполнение:**
- WebSocket-based real-time sync
- Operational transform for concurrent edits

### 5.10. FEA Mesh Control (`85`)
- [ ] Element type: Tet4/Tet10/Hex8/Hex20
- [ ] Mesh size (global + local)
- [ ] Refinement regions
- [ ] Quality metrics (aspect ratio, Jacobian)

### 5.11-5.12. Animation Timeline (`92`, `96`)
- [ ] 8 tracks (camera, visibility, transform, material, etc.)
- [ ] 120 frames
- [ ] Keyframe editing (drag, add, delete)
- [ ] Playback controls (play/pause/loop/speed)

---

## Phase 6: Dialogs (26 dialogs)

**Mockups:** `51`–`62`, `70`, `72`, `74`–`77`, `82`–`84`, `86`–`91`

| Dialog | Key Content | Наполнение |
|--------|-------------|------------|
| Options (51) | 10 sections: General/Display/Files/Hotkeys/Theme/Advanced/Plugins/AI/Perf/Cloud | Settings struct + JSON persistence |
| Customize (52) | 5 tabs: Commands/Keyboard/Toolbars/Ribbon/Marking Menu/QAT | UI config system |
| Insert Primitive (53) | Box/Sphere/Cylinder/Cone/Torus params | ShapeBuilder calls |
| Shortcut Editor (54) | 245 commands, conflict checker | Key binding system |
| Command Search (55) | Fuzzy palette (Ctrl+Shift+P) | Fuzzy match on command names |
| Plugins Manager (56) | Installed/Marketplace/Settings | Plugin loading system |
| About (57) | Version, license, credits | Static info |
| Update (58) | Version check, changelog | HTTP version check |
| Material Editor (59) | Library(7 cats) + Properties(10) + Appearance + Thermal | Material database |
| Constraint Diag (60) | 12 constraints, DOF analysis, autofix | Sketch solver integration |
| Mold Catalog (61) | Misumi/HASCO/DME/LKM | Supplier catalogs |
| Render Settings (62) | Output + Quality + Environment(HDRI) + 4 formats | Render pipeline config |
| Macro Recorder (70) | Record/Stop/Play/Save | Scripting integration |
| Tutorial Browser (72) | 16 categories, 12+ tutorials | Embedded webview |
| Print/Plot (74) | Paper size, scale, orientation | Print system |
| License (75) | Seats 42/50, activation | License manager |
| Crash Recovery (76) | 3 recovered docs | Auto-save system |
| Onboarding (77) | 4-step wizard | First-run setup |
| CAM Stock Setup (82) | 5 steps: BBox/Offset/Custom/Mesh/Cylinder | CAM module |
| Tool Library (83) | 24 tools, 7 types, 3D preview | Tool database |
| NC Code Viewer (84) | G-code syntax highlight + 3D toolpath | G-code parser |
| Modal Plotter (86) | FRF + 8 modes + animation | FEA results |
| Title Block (87) | 15 standard + 3 custom fields | Drawing template |
| Revision Table (88) | 5 revisions A–E, approval workflow | Drawing metadata |
| Layer Manager (89) | 13 layers, full attribute set | Layer system |
| BOM Editor (90) | 15 items, total cost | Assembly metadata |
| Param Search/Replace (91) | Auto-update formulas | Parameter system |

---

## Phase 7: Context Menus & Marking Menu ✅ DONE

**Mockups:** `47`–`50`

### 7.1. Viewport Context Menu (`47`) ✅
- [x] View orientation (ISO/Front/Top/etc.)
- [x] Display style switch
- [x] Zoom to selection
- [x] Section cut
- [x] Measure
- [x] Select by type (face/edge/vertex)

### 7.2. Browser Context Menu (`48`) ✅
- [x] Rename / Delete / Suppress
- [x] Edit feature
- [x] Show/Hide
- [x] Create derived (pattern/mirror)
- [x] Export selected

### 7.3. Sketch Context Menu (`49`) ✅
- [x] Constraint options (context-sensitive)
- [x] Dimension
- [x] Trim/Extend
- [x] Convert to construction geometry

### 7.4. Marking Menu (`50`) ✅
- [x] 8-direction radial pie menu
- [x] Center = ESC/cancel
- [x] Triggered by Space key
- [x] Hover highlight

**Наполнение:**
- Radial menu: overlay widget with 8 sectors
- Gesture detection: mouse direction → menu item

---

## Phase 8: Core Engine Features (Наполнение)

### 8.1. Selection System
- [ ] Pick by ray casting (face/edge/vertex)
- [ ] Multi-select (Shift+click)
- [ ] Box select
- [ ] Select by type (all faces, all edges, etc.)
- [ ] Selection highlight (outline shader)

### 8.2. Undo/Redo System
- [ ] Command pattern: each action is a Command object
- [ ] Snapshot-based: save full document state
- [ ] History tree (branching, not just linear)
- [ ] History panel with diff visualization

### 8.3. Parameter System
- [ ] Named parameters (length=100, radius=5, count=3)
- [ ] Formula support (length = width * 2)
- [ ] Parameter table (Design Table)
- [ ] Search/Replace in parameters (`91`)
- [ ] Parameter-driven feature re-evaluation

### 8.4. Material System
- [ ] Material database (density, color, thermal, mechanical)
- [ ] Material assignment to faces/bodies
- [ ] Appearance: color, roughness, metalness, texture
- [ ] Material library import/export

### 8.5. Layer System
- [ ] Create/delete/rename layers
- [ ] Assign entities to layers
- [ ] Per-layer visibility/color/line weight
- [ ] Layer manager dialog (`89`)

### 8.6. Plugin System
- [ ] Plugin API (Rust + Python + Lua)
- [ ] Plugin manager dialog (`56`)
- [ ] Marketplace integration
- [ ] Plugin sandboxing

### 8.7. Theme System
- [ ] Dark/Light/Custom themes
- [ ] Color scheme: accent color, bg, fg, border
- [ ] Icon set: per-theme icon variants
- [ ] Font size adjustment

---

## Phase 9: Specialized Workspaces

### 9.1. Visual Programming (`03`)
- [ ] Node graph editor (like Grasshopper)
- [ ] Node types: Primitives, Modify, Boolean, Script, Topology
- [ ] Connection ports (input/output)
- [ ] Live preview (auto-compute)
- [ ] Bake to document

### 9.2. Surface Modeling (`78`, `95`)
- [ ] Loft (2+ profiles, G0/G1/G2 continuity)
- [ ] Sweep (profile + path + guide rails)
- [ ] Boundary surface (4 edges)
- [ ] Fill surface (n-sided patch)
- [ ] Network surface (UV grid of curves)

### 9.3. Sheet Metal (`05`, `13`, `33`)
- [ ] Base/Edge Flange
- [ ] Bend/Hem/Jog
- [ ] Relief (rectangular/tear/obround)
- [ ] Unfold/Fold/Flat Pattern
- [ ] K-Factor / Bend Allowance
- [ ] Gauge table
- [ ] DXF export of flat pattern

### 9.4. CAM (`05`, `15`, `35`)
- [ ] Stock setup wizard (`82`)
- [ ] Tool library (`83`)
- [ ] Operations: Facing, Profile, Pocket, Drilling, Engraving, 3D Surfacing, 5-Axis
- [ ] Toolpath simulation
- [ ] G-code post-processor (7 dialects: Fanuc, Siemens, Haas, Heidenhain, Mach3, LinuxCNC, GRBL)
- [ ] NC code viewer (`84`)

### 9.5. FEA / Simulation (`04`, `17`, `37`)
- [ ] Mesh generation (Tet4/Tet10/Hex8/Hex20)
- [ ] Study types: Static, Modal, Thermal, Buckling, Fatigue, Nonlinear, CFD, EM, Optimization
- [ ] Boundary conditions (fixed, force, pressure, displacement, thermal)
- [ ] Solver (CG iterative, direct)
- [ ] Results: Von Mises, Displacement, Strain, Stress components
- [ ] Animation of results
- [ ] Modal plotter (`86`)

### 9.6. Drawing (`06`, `16`, `36`)
- [ ] Sheet setup (A0-A4, custom)
- [ ] Views: Standard(8), Section, Detail, Projected, Broken-out, Crop, Auxiliary, Exploded
- [ ] Dimensions: Linear, Angular, Radial, Diameter, Ordinate
- [ ] Annotations: Note, Balloon, Surface Finish, Welding, Datum, Tolerance
- [ ] Title block editor (`87`)
- [ ] Revision table (`88`)
- [ ] Auto-drawing from 3D model
- [ ] Export: PDF, DXF, DWG, SVG, PNG

### 9.7. Assembly (`14`, `34`)
- [ ] Component insertion (STEP/IGES/STL)
- [ ] Mate types: Coincident, Concentric, Distance, Angle, Parallel, Perpendicular, Tangent, Width, Symmetric
- [ ] Mate solver (3D constraint solving)
- [ ] BOM editor (`90`)
- [ ] Exploded view
- [ ] Motion study (interference detection)

### 9.8. Point Cloud & Reverse Engineering (`80`, `81`)
- [ ] Import: .ply, .xyz, .las (12.4M points)
- [ ] RANSAC shape detection
- [ ] Fit primitives (plane, cylinder, sphere)
- [ ] Mesh from point cloud (Poisson, Delaunay)
- [ ] 6-step reverse engineering wizard (`81`)

### 9.9. Mold Design (`22`, `61`)
- [ ] Mold base catalog (Misumi/HASCO/DME/LKM)
- [ ] Runner/Cooling/Ejection systems
- [ ] Cavity/core separation
- [ ] Flow/Cooling/Warpage analysis
- [ ] Cost/cycle time estimation

### 9.10. AI Features (`25`, `39`, `68`)
- [ ] Shape from Text (text → 3D model)
- [ ] AI Assistant (chat, DRC, suggestions, cost estimation)
- [ ] Smart features: auto-fillet, auto-pattern, auto-repair, auto-dimension, auto-constrain
- [ ] Generative design (4 variants)
- [ ] Topology optimization (3 presets)

---

## Implementation Priority

### Tier 1: MVP (Форма + базовое наполнение)
1. Application shell (Phase 0) — window layout, docks, status bar
2. Menu bar structure (Phase 1) — all 21 menus with stub actions
3. Ribbon tabs (Phase 2) — all 15 tabs with icon buttons
4. View modes (Phase 3) — Wireframe/Shaded/Shaded+Edges
5. Browser panel (Phase 5.1) — model tree
6. Properties panel (Phase 5.2) — face/edge info
7. Selection system (Phase 8.1) — pick in viewport
8. Undo/Redo (Phase 8.2) — basic linear history

### Tier 2: Core CAD (Наполнение)
9. Sketch engine (Phase 4) — 2D drawing + constraints
10. Feature history (Phase 5.3) — timeline + rollback
11. Parameter system (Phase 8.3) — named params + formulas
12. Insert primitives (Phase 1.4) — box/sphere/cylinder/cone/torus
13. Boolean operations (Phase 1.6) — union/subtract/intersect
14. Fillet/Chamfer (Phase 1.6) — edge operations
15. Measure tools (Phase 5.4)
16. Section cut (Phase 5.5)

### Tier 3: Advanced CAD
17. Surface modeling (Phase 9.2)
18. Assembly (Phase 9.7)
19. Drawing (Phase 9.6)
20. Sheet metal (Phase 9.3)
21. Visual programming (Phase 9.1)

### Tier 4: CAE/CAM
22. FEA/Simulation (Phase 9.5)
23. CAM (Phase 9.4)
24. Mold design (Phase 9.9)

### Tier 5: Ecosystem
25. Plugin system (Phase 8.6)
26. Scripting (Phase 5.7)
27. AI features (Phase 9.10)
28. Cloud collaboration (Phase 5.9)
29. VR/AR (Phase 3.6-7)
30. Point cloud/RE (Phase 9.8)

---

## Summary

| Phase | Description | Mockups | Форма | Наполнение |
|-------|-------------|---------|-------|------------|
| 0 | Application Shell | 01, 26 | Window layout | egui docks + GL viewport |
| 1 | Menu Bar (21 menus) | 07-27 | Menu structure | Action handlers |
| 2 | Ribbon (15 tabs) | 28-41, 95 | Tab widgets | Command buttons |
| 3 | View Modes (7) | 42-46, 93, 94 | Display toggle | GL renderer modes |
| 4 | Sketch Engine | 02, 11, 30, 49 | 2D canvas | Constraint solver |
| 5 | Panels (12) | 63-69, 71, 73, 85, 92, 96 | Dock widgets | Data binding |
| 6 | Dialogs (26) | 51-62, 70, 72, 74-77, 82-84, 86-91 | Modal windows | Dialog logic |
| 7 | Context Menus (4) | 47-50 | Popup menus | Context actions |
| 8 | Core Engine | — | — | Selection, undo, params, materials, layers, plugins, themes |
| 9 | Workspaces (10) | 03-06, 78, 80-82 | Workspace switch | Full module implementation |
