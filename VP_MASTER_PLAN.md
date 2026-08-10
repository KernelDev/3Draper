# VP Master Plan — Grasshopper-Inspired Visual Programming

**Created:** 2026-08-10
**Based on:** [grasshopperdocs.com](https://grasshopperdocs.com), [developer.rhino3d.com/guides/grasshopper](https://developer.rhino3d.com/guides/grasshopper/)

## Design Principles (from Grasshopper)

1. **Typed data ports** — each port carries a specific data type (Geometry, Number, Integer, Boolean, Vector, Point, Curve, Surface, Mesh). Connections are type-checked.
2. **Data trees** — ports can carry lists of items (not just single values). Operations apply to each item in the list (implicit `map`).
3. **Parameter nodes** — standalone nodes that hold data (Number Slider, Integer, Boolean Toggle, Point, etc.). Users set the value, it flows downstream.
4. **Live preview** — every parameter change triggers graph re-evaluation and viewport update.
5. **Bake to document** — converts the VP graph result into a permanent Solid in the document tree.

## Node Categories (Grasshopper-Inspired)

### 1. Params (Input Parameters)
| Node | Description | Ports Out |
|------|-------------|-----------|
| **Number Slider** | Drag to set a float value | Number |
| **Integer** | Set an integer | Integer |
| **Boolean Toggle** | True/False | Boolean |
| **Point** | XYZ coordinates | Point |
| **Vector** | Direction + magnitude | Vector |
| **Panel** | Display data (text) | (input only) |

### 2. Maths (Mathematical Operations)
| Node | Inputs | Output |
|------|--------|--------|
| **Add** | A:Number, B:Number | Number |
| **Subtract** | A, B | Number |
| **Multiply** | A, B | Number |
| **Divide** | A, B | Number |
| **Sin / Cos / Tan** | X:Number | Number |
| **Min / Max** | A, B | Number |
| **Average** | A, B, ... | Number |
| **Abs / Sqrt / Pow** | X | Number |
| **Round** | X | Integer |

### 3. Sets (List Operations)
| Node | Inputs | Output |
|------|--------|--------|
| **Series** | Start, Step, Count | List<Number> |
| **Range** | Domain, Count | List<Number> |
| **List Item** | List, Index | Item |
| **List Length** | List | Integer |
| **Cull Pattern** | List, Boolean[] | List |
| **Reverse** | List | List |
| **Sort** | List | List |

### 4. Primitives (Geometry Creation)
| Node | Params | Output |
|------|--------|--------|
| **Box** | W, H, D | Geometry(Solid) |
| **Sphere** | R | Geometry |
| **Cylinder** | R, H | Geometry |
| **Cone** | R1, R2, H | Geometry |
| **Torus** | R, r | Geometry |
| **Plane (ref)** | Size | Geometry |

### 5. Curve
| Node | Inputs | Output |
|------|--------|--------|
| **Line** | A:Point, B:Point | Curve |
| **Polyline** | Points[] | Curve |
| **Circle** | Center:Point, R | Curve |
| **Arc** | Center, R, A1, A2 | Curve |
| **Divide Curve** | Curve, N | Points[] |
| **Evaluate Curve** | Curve, T | Point |
| **Curve Length** | Curve | Number |

### 6. Surface
| Node | Inputs | Output |
|------|--------|--------|
| **Extrude** | Profile:Curve, Dir:Vector, Dist:Number | Surface/Solid |
| **Sweep** | Profile, Path:Curve | Solid |
| **Loft** | Sections[] | Surface |
| **Revolve** | Profile, Axis, Angle | Solid |
| **Offset Surface** | Surface, Dist | Surface |

### 7. Transform
| Node | Inputs | Output |
|------|--------|--------|
| **Move** | Geometry, Vector | Geometry |
| **Rotate** | Geometry, Axis, Angle | Geometry |
| **Scale** | Geometry, Factor | Geometry |
| **Mirror** | Geometry, Plane | Geometry |
| **Linear Array** | Geometry, Count, Dir, Spacing | Geometry[] |
| **Circular Array** | Geometry, Count, Axis, Angle | Geometry[] |

### 8. Intersect
| Node | Inputs | Output |
|------|--------|--------|
| **Boolean Union** | A, B | Geometry |
| **Boolean Subtract** | A, B | Geometry |
| **Boolean Intersect** | A, B | Geometry |
| **Boolean Split** | A, B | Geometry[] |

### 9. Modify
| Node | Inputs | Output |
|------|--------|--------|
| **Fillet** | Geometry, Radius, Edges | Geometry |
| **Chamfer** | Geometry, Distance | Geometry |
| **Shell** | Geometry, Thickness | Geometry |

### 10. Output
| Node | Inputs | Output |
|------|--------|--------|
| **Bake to Document** | Geometry | (bakes to tree) |

## Data Types

```
VpData enum:
  Geometry(Solid)     — 3D solid (box, sphere, boolean result)
  Curve(Polyline)     — 2D/3D curve
  Surface(NurbsSurface) — parametric surface
  Mesh(TriangleMesh)  — triangulated mesh
  Number(f64)         — floating point
  Integer(i64)        — whole number
  Boolean(bool)       — true/false
  Point([f64; 3])     — 3D point
  Vector([f64; 3])    — direction + magnitude
  String(String)      — text
  List(Vec<VpData>)   — multiple items (data tree leaf)
  Empty               — no data (not yet computed)
```

## Connection Rules

- **Geometry** → Geometry, Curve, Surface, Mesh (implicit conversion)
- **Number** → Number, Integer (truncation), Point (all coords), Vector
- **Integer** → Integer, Number (promotion)
- **Boolean** → Boolean only
- **Point** → Point, Vector (implicit)
- **Vector** → Vector, Point (implicit)
- **List<T>** → T (take first), List<T>
- **T** → List<T> (wrap in single-element list)

## Implementation Phases

### Phase 1: Core Data Types + Parameter Nodes (Current → Next)
- [ ] Define `VpData` enum with typed ports
- [ ] Replace `NodeType` params with `VpData` inputs/outputs
- [ ] Add Number Slider node (with min/max/value)
- [ ] Add Integer, Boolean Toggle, Point, Vector parameter nodes
- [ ] Type-check connections (only compatible types connect)

### Phase 2: Math Nodes
- [ ] Add, Subtract, Multiply, Divide
- [ ] Sin, Cos, Tan, Abs, Sqrt, Pow
- [ ] Min, Max, Average, Round
- [ ] Expression node (evaluate math expression string)

### Phase 3: Sets (List Operations)
- [ ] Series (start, step, count → list)
- [ ] Range (domain, count → list)
- [ ] List Item, List Length
- [ ] Cull Pattern, Reverse, Sort

### Phase 4: Curve Nodes
- [ ] Line (2 points → curve)
- [ ] Circle (center + radius → curve)
- [ ] Divide Curve (curve + N → points)
- [ ] Evaluate Curve, Curve Length

### Phase 5: Transform Nodes
- [ ] Move (geometry + vector)
- [ ] Rotate (geometry + axis + angle)
- [ ] Scale (geometry + factor)
- [ ] Mirror (geometry + plane)
- [ ] Linear/Circular Array (with count input from Number node)

### Phase 6: Data Trees (Multi-item Lists)
- [ ] Port carries `Vec<VpData>` instead of single `VpData`
- [ ] Operations auto-map over list items
- [ ] Graft/Flatten nodes
- [ ] Cross-reference (cartesian product) for multi-list ops

## DoD

- [ ] Every node type has inline parameter editing (DragValue/Slider/Checkbox)
- [ ] Changing any parameter triggers live preview (no manual Bake needed)
- [ ] Connections are type-checked — incompatible types can't connect
- [ ] Math nodes work with Number Slider inputs (parametric chains)
- [ ] Transform nodes accept geometry from any source
- [ ] Bake creates proper tree entry with face list
- [ ] At least 30 node types implemented
