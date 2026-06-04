
---
Task ID: text-holes-1
Agent: Main
Task: Add 3Draper text holes to primitives + STEP timeout
Stage Summary: cut_text_holes_in_mesh() implemented, viewer updated, deployed to GitHub Pages

---
Task ID: fix-holes-step-2
Agent: Main
Task: Fix hole cutting and STEP file positioning bugs

Work Log:
- Diagnosed root cause of "3" hole not rendering: text scale 0.5 made glyph 5x7 units, but mesh triangles were ~12x10 units each — no centroid fell inside the hole contour
- Rewrote cut_text_holes_in_mesh() with adaptive mesh refinement: subdivide_and_classify() recursively splits large triangles near hole boundaries until they're small enough for centroid-based classification
- Increased text scales from 0.3-0.5 to 2.0-3.0 for all primitives so "3" is visible on surfaces
- Added CARTESIAN_TRANSFORMATION_OPERATOR_3D support in both static and non-static STEP transform extraction
- Fixed AXIS2_PLACEMENT_3D ref_direction projection (must be projected onto plane perpendicular to axis per STEP spec)
- Fixed TriangleMesh::transform() to also transform normals using inverse-transpose of 3x3 rotation
- Added compute_normal_transform() helper function for correct normal transformation
- All changes compiled and 75/77 tests passed (2 pre-existing PMI test failures unrelated to changes)
- Pushed to GitHub commit d2f1784

Stage Summary:
- Hole cutting now works with adaptive refinement (subdivide triangles near holes)
- STEP CTO3D entities now supported for assembly transforms
- AXIS2_PLACEMENT_3D now correctly projects ref_direction
- Mesh normals now correctly transformed with vertices

---
Task ID: fix-visibility-toggle
Agent: Main
Task: Fix visibility toggle for assembly instances - mesh not hiding, edges inconsistent

Work Log:
- Diagnosed root cause: line 3141 in app.rs was overwriting `instance_triangle_ranges` with GPU output ranges from `mesh_to_gpu_data()`
- The GPU output ranges map instance indices to triangle ranges in the GPU buffer (with hidden instances removed, indices shift)
- But `self.mesh.triangles` still contains ALL triangles including hidden ones
- All subsequent operations (`build_wireframe_overlay_vertices()`, `pick_at()`, future `mesh_to_gpu_data()`) iterate over `self.mesh.triangles` and need the ORIGINAL ranges
- After first visibility toggle, the ranges were corrupted → mesh surfaces didn't hide, edges hid inconsistently
- Fix: removed `self.instance_triangle_ranges = new_ranges;` line, added detailed comment explaining why
- Built native binary successfully
- Built WASM with wasi-sdk-24.0 and Trunk
- Deployed to GitHub Pages (gh-pages branch)
- Pushed commit 27a06c6 to main branch

Stage Summary:
- Root cause: `instance_triangle_ranges` corruption after GPU output range overwrite
- Fix: preserve original mesh triangle ranges, don't overwrite with GPU output ranges
- Both mesh surfaces and edges should now correctly hide/show per instance
- Deployed to https://kerneldev.github.io/3Draper/

---
Task ID: fix-planar-holes-3
Agent: Main
Task: Fix planar face triangulation with holes + UV triangle visualization + face picking bug

Work Log:
- Diagnosed issue: plate_1 F#1 (STEP#3815) Plane has 6 bolt holes but they were not cut from surface
- Root cause 1: bridge-edge + ear-clip triangulation produces triangles spanning across holes, especially for circular bolt holes where bridge edges don't fully separate the hole
- Root cause 2: `merge_coincident_vertices()` and `filter_degenerate_triangles()` were removing triangles without updating `triangle_face_ids`, causing face picking to always select face #1
- Added post-filter in `triangulate_planar_face_with_holes_cached()` and `triangulate_planar_face_with_holes()` to remove triangles whose centroids fall inside any hole
- Added same post-filter in `triangulate_planar_face()` in draper-mesh
- Fixed `merge_coincident_vertices()` to preserve `triangle_face_ids` in sync with filtered triangles
- Fixed `filter_degenerate_triangles()` to preserve `triangle_face_ids` in sync with filtered triangles
- Added `uv_triangles: Vec<[Point2d; 3]>` field to `FaceInfo` struct
- Added UV triangle projection in converter.rs (project each triangle's vertices to UV space)
- Added UV triangle rendering in SVG export: valid triangles in blue, triangles inside holes in red
- Added UV triangle rendering in egui UV grid panel with same color coding
- Fixed `JsonFaceInfo::to_face_info()` to include new `uv_triangles` field
- Pushed commits 8e26df1 and 681cfbb to main
- GitHub Actions build succeeded, deployed to GitHub Pages

Stage Summary:
- Bolt holes in plate_1 now properly cut from surface via centroid-based triangle filtering
- Face picking (Ctrl+click) now correctly identifies which face was clicked (no longer always F#1)
- UV triangle visualization added to both SVG export and egui panel for debugging triangulation
- triangle_face_ids now correctly maintained through vertex merging and degenerate filtering
