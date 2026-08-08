# BRepCAD UI Audit — Action Plan

**Created:** 2026-08-08
**Based on:** Full audit of 96 UI mockups vs implementation
**Status:** In progress

## Audit Summary

| Metric | Value |
|---|---|
| Total mockups | 96 |
| Implemented (✅) | 60 (62.5%) |
| Partial (⚠️) | 12 (12.5%) |
| Missing (❌) | 24 (25.0%) |
| MenuAction variants | ~348 |
| Functional | ~146 (42%) |
| Stubs | ~190 (55%) |

## Critical Issues

### Issue #1: `dispatcher.rs` is dead code (CRITICAL)
- **File:** `crates/draper-viewer/src/ui/dispatcher.rs` (1568 lines)
- **Problem:** Contains real FEA solver, CAM toolpaths, AI parsing, drawing generation — but 0 callers. Runtime uses simpler stubs in `app.rs`.
- **Fix:** Wire `dispatch_menu_action` into `handle_brepcad_action` or merge implementations.

### Issue #2: Tools→Options menu broken (BUG)
- **Problem:** `ToolsOptions`, `ToolsPlugins`, `ToolsPerformance`, `ToolsAiSettings` fall through to catch-all `_ =>` and return "not yet implemented". Ctrl+, works, menu click doesn't.
- **Fix:** Add explicit handlers that set `self.brepcad_dialog`.

## Execution Plan (Priority Order)

### Phase 1: Quick Fixes (1-2 hours)

#### Task 1.1: Fix Tools→Options bug
- Add explicit handlers for `ToolsOptions`, `ToolsPlugins`, `ToolsPerformance`, `ToolsAiSettings`
- Each should set `self.brepcad_dialog` to the appropriate `DialogType`
- **Files:** `crates/draper-viewer/src/app.rs`
- **Test:** Click Tools→Options in menu → dialog opens

#### Task 1.2: Wire dispatcher.rs FEA into app.rs
- Replace analytical formula in `SimSolve` with real `FeaSolver::solve()`
- Replace fake mesh count in `SimMesh` with real `TetMesh::from_triangle_mesh`
- **Files:** `crates/draper-viewer/src/app.rs` (SimMesh, SimSolve handlers)
- **Test:** Sim→Mesh shows real tet count; Sim→Solve shows real displacement

#### Task 1.3: Wire dispatcher.rs CAM into app.rs
- Replace generic G-code with real `CamOperation::Contour`/`Pocket`/`Drill` toolpaths
- **Files:** `crates/draper-viewer/src/app.rs` (CamProfile, CamPocket, CamDrilling handlers)
- **Test:** CAM→Profile generates real contour toolpath

#### Task 1.4: Wire dispatcher.rs AI into app.rs
- Replace generic box in `AiShapeFromText` with real `AiShapeParser` + `actions_to_solids`
- **Files:** `crates/draper-viewer/src/app.rs` (AiShapeFromText handler)
- **Test:** AI→Shape from Text creates parsed shape

### Phase 2: CAM Postprocessor Differentiation (2-3 hours)

#### Task 2.1: Implement dialect-specific G-code
- Fanuc: standard G1/G2/G3, modal groups
- Siemens: G1/G2/G3 with advanced cycles
- Haas: similar to Fanuc
- Heidenhain: L instead of G1, different format
- Mach3/LinuxCnc: G-code standard
- GRBL: simplified subset
- **Files:** `crates/draper-viewer/src/app.rs` (CAM post handlers)
- **Test:** Different dialects produce different G-code

### Phase 3: Visual Compliance with Mockup 01 (2-3 hours)

#### Task 3.1: Add Quick Access Toolbar (QAT)
- Undo/Redo/Save buttons at right of ribbon tab strip
- **Files:** `crates/draper-viewer/src/ui/ribbon.rs`
- **Test:** QAT buttons visible and functional

#### Task 3.2: Add HUD overlay in viewport
- Move X/Y/Z, Az/El/D, Tool, FPS, Display from status bar to viewport top-left
- **Files:** `crates/draper-viewer/src/app.rs` (viewport rendering)
- **Test:** HUD overlay visible in viewport corner

#### Task 3.3: Add command search in ribbon
- Embed `egui::TextEdit` in ribbon for command search
- On Enter, open command palette with pre-filled search
- **Files:** `crates/draper-viewer/src/ui/ribbon.rs`
- **Test:** Search box visible in ribbon

### Phase 4: Missing Dialogs (3-4 hours)

#### Task 4.1: Render Settings dialog (mockup 62)
- Add `DialogType::RenderSettings`
- Fields: quality, shadows, AO, AA, background color
- **Files:** `crates/draper-viewer/src/ui/dialogs.rs`

#### Task 4.2: Customize dialog (mockup 52)
- Add `DialogType::Customize`
- Tabs: Ribbon customization, Shortcut editor, Toolbar layout
- **Files:** `crates/draper-viewer/src/ui/dialogs.rs`

#### Task 4.3: NC Code Viewer dialog (mockup 84)
- Add `DialogType::NcCodeViewer`
- Shows generated G-code with syntax highlighting
- **Files:** `crates/draper-viewer/src/ui/dialogs.rs`

#### Task 4.4: Print/Plot dialog (mockup 74)
- Add `DialogType::PrintPlot`
- Paper size, orientation, scale, preview
- **Files:** `crates/draper-viewer/src/ui/dialogs.rs`

### Phase 5: Sketch Constraints (4-5 hours)

#### Task 5.1: Implement constraint solver
- Coincident, Collinear, Concentric, Parallel, Perpendicular, Tangent, Equal
- Use Newton-Raphson (already in `draper-sketch`)
- **Files:** `crates/draper-viewer/src/ui/sketch.rs`, `crates/draper-sketch/src/`

#### Task 5.2: Wire constraint diagnostics dialog (mockup 60)
- Shows constraint status, violations, DOF count
- **Files:** `crates/draper-viewer/src/ui/dialogs.rs`

### Phase 6: Animation Timeline (3-4 hours)

#### Task 6.1: Animation timeline panel (mockups 92, 96)
- Keyframe data structure: `Vec<(track_id, frame, value, easing)>`
- Playback controls: Play/Pause/Record/Loop
- Timeline grid with frame markers
- **Files:** `crates/draper-viewer/src/ui/panels.rs` or new `animation.rs`

### Phase 7: Scripting & Plugins (3-4 hours)

#### Task 7.1: Scripting console (mockup 69)
- Embed `rhai` scripting engine (Rust-native, no external deps)
- Console with command input/output
- **Files:** `crates/draper-viewer/src/ui/panels.rs` or new `scripting.rs`

#### Task 7.2: Plugin manager interactivity (mockup 56)
- Enable/disable plugins
- Load from `plugins/` directory
- **Files:** `crates/draper-viewer/src/ui/dialogs.rs`

### Phase 8: Cloud Collab Panel Enhancement (2-3 hours)

#### Task 8.1: Extend collab panel UI (mockup 73)
- User avatars with online/offline status
- Activity feed
- Branch management
- Storage usage indicator
- **Files:** `crates/draper-viewer/src/ui/collab_panel.rs`

## Progress Tracking

- [x] Phase 1 Task 1.1: Fix Tools→Options bug
- [x] Phase 1 Task 1.2: Wire FEA (real FeaSolver::solve)
- [x] Phase 1 Task 1.3: Wire CAM (dialect-specific G-code)
- [x] Phase 1 Task 1.4: Wire AI (real AiShapeParser)
- [x] Phase 2: CAM postprocessor differentiation (Heidenhain/GRBL/Siemens/Fanuc)
- [x] Phase 3: Visual compliance (QAT, HUD overlay, search in ribbon)
- [ ] Phase 4: Missing dialogs
- [ ] Phase 5: Sketch constraints
- [ ] Phase 6: Animation timeline
- [ ] Phase 7: Scripting & plugins
- [ ] Phase 8: Cloud collab enhancement

## Notes

- Each phase should be committed separately with descriptive messages
- Run `cargo test -p draper-viewer --no-default-features --features native --lib` after each change
- Push to `origin/main` after each phase completes
- Target: reduce stub count from ~190 to <100, increase mockup coverage from 62.5% to >80%
