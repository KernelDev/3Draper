# BRepCAD VP — Node Examples & Usage Guide

This directory contains SVG diagrams showing how to use every VP node type and common combinations.

## Files

| File | Description |
|------|-------------|
| `01_parametric_box.svg` | Basic: Box → Bake (simplest graph) |
| `02_parametric_chain.svg` | Slider × Slider → Multiply → drives Box dimensions |
| `03_boolean_subtract.svg` | Box - Cylinder = hole (boolean operation) |
| `04_3d_transform_chain.svg` | Box → Move(X,Y,Z) → Rotate(X°,Y°,Z°) → Scale(X,Y,Z) → Bake |
| `05_linear_array.svg` | Box replicated N times in a row |
| `06_series_math.svg` | Series generates list → ListItem extracts → use as dimension |
| `07_mirror_fillet.svg` | Mirror across YZ plane + Fillet edges |
| `08_full_catalog.svg` | **All 58 node types** organized by category (10 categories) |
| `09_complex_gear.svg` | Complex: Cylinder + CircularArray(Box) → Union - hole = gear |

## How to Connect Nodes

There are two ways to connect nodes in the VP canvas:

### Method 1: Click-Click
1. Click the **output port** (colored circle on the right edge of a node)
2. Move the mouse to the **input port** (colored circle on the left edge of another node)
3. Click the input port — the connection is created

### Method 2: Drag
1. Press and hold on the **output port**
2. Drag to the **input port**
3. Release — the connection is created

### Type Checking
Connections are type-checked. The port color indicates the data type:
- **Blue** = Geometry (Solids, BReps)
- **Green** = Curve
- **Yellow** = Number (f64)
- **Red** = Integer (i64)
- **Pink** = Boolean
- **Peach** = Point [x, y, z]
- **Teal** = Vector [x, y, z]
- **Purple** = List
- **Gray** = Any (accepts any type)

Compatible types auto-connect (e.g., Integer → Number works via promotion).
Incompatible types are rejected with a status message.

## Node Categories (10)

### 1. Params (6 nodes) — green
Input parameters: Number Slider, Integer, Boolean, Point, Vector, Panel

### 2. Maths (12 nodes) — yellow
Math operations: Add, Subtract, Multiply, Divide, Sin, Cos, Tan, Abs, Sqrt, Pow, Min, Max, Average, Expression

### 3. Sets (8 nodes) — purple
List operations: Series, Range, List Length, List Item, Reverse, Sort, Cull Pattern

### 4. Primitives (5 nodes) — blue
3D shapes: Box, Sphere, Cylinder, Cone, Torus

### 5. Curve (5 nodes) — teal
Curve operations: Line, Circle, Divide Curve, Evaluate Curve, Curve Length

### 6. Transform (6 nodes) — peach
3D transforms (all support XYZ):
- **Move** — translation X, Y, Z (or connect a Vector)
- **Rotate** — Euler angles X°, Y°, Z° (applied in order X→Y→Z)
- **Scale** — non-uniform X, Y, Z factors
- **Mirror** — across YZ, XZ, or XY plane
- **Linear Array** — replicate N times along X with spacing S
- **Circular Array** — replicate N times around Z with total angle A°

### 7. Boolean (3 nodes) — lavender
Boolean operations: Union, Subtract, Intersect

### 8. Modify (2 nodes) — yellow
Edge modifications: Fillet (round), Chamfer (bevel)

### 9. Data Tree (8 nodes) — purple
Data tree manipulation: Graft, Flatten, CrossRef, ShiftList, Subset, Dispatch, Weave, Concat

### 10. Output (1 node) — pink
Bake to Doc — commits the VP result to the document tree

## 3D Transform Details

### Rotate (Euler XYZ)
The Rotate node applies rotations in **X→Y→Z order** (intrinsic rotations):
```
R = Rz · Ry · Rx
```
Each angle is in **degrees** (-360 to 360). You can either:
- Set inline values (X°, Y°, Z°)
- Connect Number nodes to the X/Y/Z input ports (overrides inline values)

### Move (Translation)
The Move node translates geometry by (X, Y, Z). You can either:
- Set inline values (X, Y, Z)
- Connect a Vector node to the V input port (overrides inline values)

### Scale (Non-Uniform)
The Scale node scales geometry by (X, Y, Z) factors. You can either:
- Set inline values (X, Y, Z)
- Connect Number nodes to the X/Y/Z input ports (overrides inline values)

### Mirror
The Mirror node reflects geometry across a plane:
- **YZ** plane: negate X coordinate
- **XZ** plane: negate Y coordinate
- **XY** plane: negate Z coordinate

## References

- [Grasshopper Complete Index](https://grasshopperdocs.com/completeIndex.html)
- [Rhino Developer Guides](https://developer.rhino3d.com/guides/grasshopper/)
