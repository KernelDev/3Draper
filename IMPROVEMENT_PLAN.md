# 🔍 BRepCAD Improvement Plan — Post-Audit

**Created:** 2026-08-09
**Based on:** Extended audit (96 mockups vs implementation, recommendations compliance)
**Commits audited:** `6014ebb` → `7d292d2` (10 commits)

## Audit Summary

| Metric | Value |
|---|---|
| UI mockup coverage | ~70% (67/96) |
| Menu bar | 21/21 ✅ |
| Ribbon tabs | 15/15 ✅ |
| Dialogs | 14/27 (52%) |
| Panels | 10/12 (83%) |
| Workspaces | 8/8 ✅ |
| Total tests | ~800+ |
| Recommendations completed | 4/5 fully, 1 partially |

## Audit Scores

| Criterion | Score | Notes |
|---|---|---|
| Recommendation compliance | 9/10 | 4/5 done, 1 partial (runtime e2e tests) |
| UI functionality | 8/10 | 70% mockups, all key features |
| Visual compliance | 5/10 | Unicode emoji instead of SVG icons |
| Backend integration | 7/10 | Some ribbon buttons → stubs |
| Code quality | 8/10 | 800+ tests, clean architecture |

---

## 🏗️ Phase 1: Visual Compliance (P0 — Critical)

**Goal:** Replace Unicode emoji/symbols with professional SVG icons.
**Effort:** HIGH (2-3 sessions)
**Impact:** Transforms the app from "prototype" to "professional CAD"

### Task 1.1: Icon system foundation `[P0 | Effort: MEDIUM]`

- Create `crates/draper-viewer/assets/icons/` directory
- Convert key mockup SVG icons to 16×16 PNG (or use egui's SVG loader)
- Create `IconSet` struct in `ribbon.rs` that loads and caches icons
- Replace `cmd_button(ui, "📄", "New")` with `cmd_button_icon(ui, &icons.new, "New")`

**Files:**
```
crates/draper-viewer/assets/icons/ (new — 40+ PNG files)
crates/draper-viewer/src/ui/icons.rs (new — IconSet struct)
crates/draper-viewer/src/ui/ribbon.rs (modify — use IconSet)
```

**DoD:**
- [ ] 40+ icons created from mockup SVGs or Lucide/Phosphor
- [ ] IconSet struct loads and caches all icons at startup
- [ ] All ribbon buttons use IconSet instead of Unicode
- [ ] No visual regressions (button sizes, alignment)

### Task 1.2: Menu bar icons `[P1 | Effort: LOW]`

- Add small 12×12 icons to menu items (optional, matches mockups 07-27)

**DoD:**
- [ ] 21 menus have leading icons

### Task 1.3: Status bar + workspace sidebar icons `[P1 | Effort: LOW]`

- Replace text workspace buttons (M, S, VP, SM, CAM, FEA, DR, AI) with 24×24 icons

**DoD:**
- [ ] 8 workspace buttons use icons
- [ ] Status bar items have icons

---

## 🏗️ Phase 2: Missing Dialogs (P1 — High)

**Goal:** Implement 13 missing dialogs.
**Effort:** MEDIUM (1-2 sessions)

### Task 2.1: High-priority dialogs `[P0 | Effort: MEDIUM]`

Priority order (by user impact):

1. **BOM Editor (90)** — Bill of Materials for assemblies
2. **Layer Manager (89)** — Layer visibility, lock, color
3. **Tool Library (83)** — CAM tool management
4. **FEA Mesh Control (85)** — Element size, mesh quality
5. **Title Block Editor (87)** — Editable title block fields

**DoD per dialog:**
- [ ] DialogType variant added
- [ ] render_*_dialog() function with interactive widgets
- [ ] Wired to menu action (opens dialog)
- [ ] At least 1 test

### Task 2.2: Medium-priority dialogs `[P1 | Effort: LOW]`

6. **Param Search/Replace (91)** — Find and replace parameter values
7. **Revision Table (88)** — Drawing revision history
8. **Tutorial Browser (72)** — Interactive tutorials list
9. **Crash Recovery (76)** — Auto-save recovery
10. **Onboarding Wizard (77)** — First-run setup

### Task 2.3: Low-priority dialogs `[P2 | Effort: LOW]`

11. **Update (58)** — Check for updates
12. **License (75)** — License info
13. **Mold Catalog (61)** — Mold base selection
14. **Modal Plotter (86)** — Frequency response plot

---

## 🏗️ Phase 3: Backend Integration Gaps (P1)

**Goal:** Wire stub ribbon/menu buttons to real backend code.

### Task 3.1: Clipboard (Cut/Copy/Paste) `[P0 | Effort: MEDIUM]`

- Implement clipboard buffer for Solid objects
- `EditCut` → move solid to clipboard, remove from scene
- `EditCopy` → clone solid to clipboard
- `EditPaste` → insert solid from clipboard at offset position

**DoD:**
- [ ] Cut/Copy/Paste work on solids
- [ ] Multi-select support
- [ ] Clipboard persists across undo/redo

### Task 3.2: NC Code Viewer — real G-code `[P1 | Effort: LOW]`

- `render_nc_code_viewer_dialog` should read from `self.brepcad_cam_gcode`
- Syntax highlighting based on actual generated code, not sample

**DoD:**
- [ ] Dialog shows real G-code from CAM post-processing
- [ ] Dialect label matches actual postprocessor used

### Task 3.3: Macro Recorder — real recording `[P1 | Effort: MEDIUM]`

- Track user actions (menu clicks, parameter changes) into a Vec
- Export as script (Python/Lua format)
- Replay by executing script commands

**DoD:**
- [ ] Record captures at least 5 action types
- [ ] Replay reproduces recorded actions
- [ ] Export to .py / .lua script file

### Task 3.4: Direct Modeling (Move/Offset Face) `[P1 | Effort: HIGH]`

- `ModifyMoveFace` → translate selected face along its normal
- `ModifyOffsetFace` → offset face by distance
- `ModifyReplaceFace` → replace face with new surface
- `ModifySplitFace` → split face at selected edge

**DoD:**
- [ ] Move Face works on planar faces
- [ ] Offset Face works on planar faces
- [ ] 3+ tests per operation

### Task 3.5: Sketch tools (Spline, Polygon) `[P1 | Effort: MEDIUM]`

- `SketchSpline` → interactive spline drawing (click points, fit curve)
- `SketchPolygon` → regular polygon (N sides, center, radius)

**DoD:**
- [ ] Spline creates smooth curve through clicked points
- [ ] Polygon creates N-sided shape
- [ ] Both work in sketch mode

### Task 3.6: View toggles (Perspective, Shadows, AO) `[P2 | Effort: LOW]`

- `ViewPerspective` / `ViewOrthographic` → toggle camera projection
- `ViewToggleShadows` → enable/disable shadow rendering
- `ViewToggleAo` → enable/disable ambient occlusion

**DoD:**
- [ ] Perspective/Orthographic toggle works
- [ ] Shadows toggle works (if renderer supports)
- [ ] AO toggle works (if renderer supports)

---

## 🏗️ Phase 4: Runtime Testing (P1)

### Task 4.1: End-to-end integration tests `[P0 | Effort: MEDIUM]`

- Create `crates/draper-testing/tests/e2e_workflow.rs`
- Test: STEP import → sketch → extrude → fillet → parametric rebuild
- Test: Box → boolean subtract (hole) → HLR drawing export
- Test: Assembly → mate constraint → solve → collision check

**DoD:**
- [ ] 3+ e2e tests covering full workflow
- [ ] Tests run in CI without GPU (headless)

---

## 🏗️ Phase 5: File I/O Gaps (P2)

### Task 5.1: Import formats `[P2 | Effort: MEDIUM]`

- `FileImportObj` → Wavefront OBJ parser
- `FileImportPly` → PLY parser
- `FileImportDxf` → DXF parser (for 2D profiles)
- Recent files list in File menu

### Task 5.2: Export formats `[P2 | Effort: MEDIUM]`

- `FileExportObj` → Wavefront OBJ writer
- `FileExportGltf` → glTF writer
- `FileExportDxf` → DXF writer (for flat patterns)

---

## Priority Matrix

| Task | Priority | Effort | Impact |
|---|---|---|---|
| 1.1 Icon system | P0 | MEDIUM | Transforms visual quality |
| 3.1 Clipboard | P0 | MEDIUM | Critical CAD feature |
| 2.1 High-priority dialogs | P0 | MEDIUM | 5 key dialogs |
| 4.1 E2E tests | P0 | MEDIUM | Quality assurance |
| 3.4 Direct Modeling | P1 | HIGH | Professional CAD |
| 3.5 Sketch tools | P1 | MEDIUM | Sketch completeness |
| 3.2 NC Code Viewer | P1 | LOW | CAM workflow |
| 3.3 Macro Recorder | P1 | MEDIUM | Automation |
| 2.2 Medium dialogs | P1 | LOW | 5 dialogs |
| 1.2 Menu icons | P1 | LOW | Visual polish |
| 1.3 Workspace icons | P1 | LOW | Visual polish |
| 3.6 View toggles | P2 | LOW | Rendering |
| 2.3 Low dialogs | P2 | LOW | 4 dialogs |
| 5.1 Import formats | P2 | MEDIUM | File I/O |
| 5.2 Export formats | P2 | MEDIUM | File I/O |

---

## Progress Tracking

- [ ] Phase 1 Task 1.1: Icon system foundation
- [ ] Phase 1 Task 1.2: Menu bar icons
- [ ] Phase 1 Task 1.3: Workspace sidebar icons
- [ ] Phase 2 Task 2.1: High-priority dialogs (BOM, Layer, Tool Lib, FEA Mesh, Title Block)
- [ ] Phase 2 Task 2.2: Medium-priority dialogs (5 dialogs)
- [ ] Phase 2 Task 2.3: Low-priority dialogs (4 dialogs)
- [ ] Phase 3 Task 3.1: Clipboard (Cut/Copy/Paste)
- [ ] Phase 3 Task 3.2: NC Code Viewer real G-code
- [ ] Phase 3 Task 3.3: Macro Recorder real recording
- [ ] Phase 3 Task 3.4: Direct Modeling (Move/Offset Face)
- [ ] Phase 3 Task 3.5: Sketch tools (Spline, Polygon)
- [ ] Phase 3 Task 3.6: View toggles (Perspective, Shadows, AO)
- [ ] Phase 4 Task 4.1: E2E integration tests
- [ ] Phase 5 Task 5.1: Import formats (OBJ, PLY, DXF)
- [ ] Phase 5 Task 5.2: Export formats (OBJ, glTF, DXF)

## Notes

- Each phase should be committed separately
- Run `cargo test` after each change
- Push to `origin/main` after each phase
- Target: 90%+ mockup coverage, professional visual quality
