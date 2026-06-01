
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
