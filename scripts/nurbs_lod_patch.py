#!/usr/bin/env python3
"""Patch app.rs to make NURBS gallery surfaces LOD-aware.

For each `load_nurbs_*` function:
  - Replace `self.build_nurbs_surface_mesh(nurbs_surface, 30)` (or 20)
    with `self.build_nurbs_surface_mesh(nurbs_surface, self.steps_for_lod())`.
  - Insert `self.current_nurbs_surface = Some(nurbs_surface.clone());` BEFORE
    the build call so that retriangulate_for_lod can rebuild the mesh.

Also, in every non-NURBS `load_*` function that sets current_solid,
insert `self.current_nurbs_surface = None;` to clear stale NURBS state.
"""
import re
from pathlib import Path

PATH = Path("/home/z/my-project/crates/draper-viewer/src/app.rs")
src = PATH.read_text()
orig = src

# ─── 1. Replace all NURBS gallery loaders ───────────────────────────────
# Pattern: build_nurbs_surface_mesh(nurbs_surface, NN) → save surface + steps_for_lod()
nurbs_pattern = re.compile(
    r"        let \(mesh, nurbs_solid\) = self\.build_nurbs_surface_mesh\(nurbs_surface, (?:\d+)\);"
)

def nurbs_replace(m):
    return (
        "        // Save the surface so `retriangulate_for_lod()` can rebuild\n"
        "        // the grid mesh with a different LOD step count.\n"
        "        self.current_nurbs_surface = Some(nurbs_surface.clone());\n"
        "        let steps = self.steps_for_lod();\n"
        "        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);"
    )

src = nurbs_pattern.sub(nurbs_replace, src)

# ─── 2. Clear current_nurbs_surface in primitive loaders ───────────────
# Find all `self.current_solid = Some(solid);` lines that are NOT followed by
# a `self.current_nurbs_surface = ...` assignment (NURBS loaders use
# `self.current_solid = Some(nurbs_solid);`).
clear_pattern = re.compile(
    r"(        self\.current_solid = Some\(solid\);\n)"
    r"(?!        self\.current_nurbs_surface = None;\n)"
)

def clear_replace(m):
    return m.group(1) + "        self.current_nurbs_surface = None;\n"

src = clear_pattern.sub(clear_replace, src)

# Also clear in STEP loader and JSON/STL loaders — they set current_solid via
# different variable names or in different ways. Search for `current_solid = Some(s)`
# and `current_solid = Some(solid.clone())` etc.
extra_patterns = [
    (r"(        self\.current_solid = Some\(s\);\n)(?!        self\.current_nurbs_surface = None;\n)",
     r"\1        self.current_nurbs_surface = None;\n"),
    (r"(        self\.current_solid = Some\(s\.clone\(\)\);\n)(?!        self\.current_nurbs_surface = None;\n)",
     r"\1        self.current_nurbs_surface = None;\n"),
    (r"(        self\.current_solid = Some\(copies\.into_iter\(\)\.next\(\)\.unwrap_or_else\(\|\| s\.clone\(\)\)\);\n)(?!        self\.current_nurbs_surface = None;\n)",
     r"\1        self.current_nurbs_surface = None;\n"),
]
for pat, repl in extra_patterns:
    src = re.sub(pat, repl, src)

# ─── 3. Also clear in retriangulate_for_lod path ───────────────────────
# (We'll handle that separately in retriangulate_for_lod itself.)

# ─── 4. Verify ─────────────────────────────────────────────────────────
if src == orig:
    print("WARNING: No changes made!")
else:
    print(f"Made changes. New file size: {len(src)} chars (was {len(orig)} chars)")

PATH.write_text(src)
print(f"Wrote {PATH}")

# Count how many NURBS patches we made
nurbs_count = src.count("self.current_nurbs_surface = Some(nurbs_surface.clone());")
clear_count = src.count("self.current_nurbs_surface = None;")
print(f"NURBS surface saves: {nurbs_count}")
print(f"NURBS surface clears: {clear_count}")
