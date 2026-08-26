# BRepCAD VP Graph Examples

This directory contains ready-to-load Visual Programming graph files for
BRepCAD. Each `.vp.json` file is a self-contained JSON graph that can be
loaded into the BRepCAD VP workspace.

## How to load

1. Launch BRepCAD (`cargo run --release --bin brepcad-shell`).
2. Switch to the **Visual Programming** workspace (icon in the left ribbon).
3. Click **📂 Load…** in the VP toolbar (above the node canvas), or use
   **File → 📂 Load VP Graph…** in the menu bar.
4. Pick any `.vp.json` file from this directory.
5. The graph appears in the canvas. Enable **Live Preview** (default: on)
   to see the resulting solid in the 3D viewport.
6. Click **Bake to Document** to commit the solid to the document tree
   (it can then be exported to STL/STEP/OBJ).

## Example catalogue

| # | File                              | What it builds                                                        |
|---|-----------------------------------|-----------------------------------------------------------------------|
| 1 | `01_box_with_hole.vp.json`        | 10×10×10 box with a Ø3 cylinder hole through it.                      |
| 2 | `02_simple_gear_blank.vp.json`    | Flat disk (Ø40, h=8) with two concentric holes — gear blank.         |
| 3 | `03_sphere_cube_union_fillet.vp.json` | Sphere (r=5) ∪ Cube (4³) with 0.5 fillet on all edges.            |
| 4 | `04_parametric_cylinder_array.vp.json` | Cylinder whose r/h are driven by NumberSliders, then arrayed ×5.  |
| 5 | `05_engine_bracket.vp.json`       | Bracket plate (20×4×8) with 4 bolt holes, fillet + chamfer.          |
| 6 | `06_flywheel.vp.json`             | Torus + hub + 8 spokes (circular array) — classic flywheel.          |
| 7 | `07_spherical_lattice.vp.json`    | Sphere with 6×4 subtracted small spheres — lattice shell.            |
| 8 | `08_full_parametric_pipe.vp.json` | Linear × circular array of cylinders — full parametric pipeline.      |
| 9 | `09_piston_liner_assembly.vp.json`| Hollow liner (cyl − cyl) + 4 box fins on circular array.              |
|10 | `10_perforated_panel.vp.json`     | Plate with linear array of holes, mirrored across XZ.                 |

## File format

```json
{
  "version": 1,
  "next_id": <next unique node id>,
  "nodes": [
    {
      "id": <u64>,
      "type": "<NodeType label>",
      "fields": { ... type-specific parameters ... },
      "x": <f32 canvas position>,
      "y": <f32 canvas position>
    }
  ],
  "connections": [
    {
      "from_node": <u64>,
      "from_port": <usize>,
      "to_node":   <u64>,
      "to_port":   <usize>
    }
  ]
}
```

`type` must match the `label()` of a `NodeType` variant. See
`crates/draper-viewer/src/ui/workspaces.rs::node_type_from_json` for the
exact strings recognized by the parser.

## Save your own graphs

- **💾 Save…** in the VP toolbar saves the entire graph.
- **⬆ Export Selected…** saves only the currently selected node (and its
  downstream subgraph) as a reusable component.
- **⬇ Import…** merges an exported subgraph into the current graph,
  remapping IDs to avoid collisions.

## Round-trip test

The Rust test suite in
`crates/draper-viewer/src/ui/workspaces.rs::tests` verifies that every
graph in this directory round-trips through `to_json` / `from_json`
without loss. Run with:

```bash
cargo test -p draper-viewer --lib vp_graph_
```
