# VP Node Expansion Plan — Grasshopper-Inspired

**Created:** 2026-08-16
**Based on:** [grasshopperdocs.com/completeIndex.html](https://grasshopperdocs.com/completeIndex.html)
**Scope:** All parametric CAD-essential nodes feasible with current `draper-geometry` + `draper-topology` API.

---

## ✅ Implementation Progress (Updated 2026-08-18)

**Status:** 190 → 210 NodeType variants; ~137 nodes have working evaluation logic.
Build: clean (`cargo check -p draper-viewer`).

### Newly implemented evaluation (Phase I, commit on 2026-08-18)

- **Primitives (8):** `PlanePrimitive`, `PolygonPrism`, `Tube`, `Helix`, `Wedge`,
  `Tetra`, `Octa`, `Icosa`.
- **Curves (2):** `Hyperbola`, `Parabola`.
- **Transforms (5):** `Shear`, `Taper`, `ApplyTransform`, `ArrayPolar`, `Offset`.

### Previously implemented (commit 506fa3b, 2026-08-18)

- **Sets/Tree (12):** `CullIndex`, `Partition`, `ReplaceItems`, `Sift`, `Combine`,
  `Duplicate`, `NullItem`, `PathMapper`, `TreeBranch`, `TreeStatistics`,
  `CleanTree`, `ExplodeTree`.
- **Output (8):** `BakeToLayer`, `BakeMesh`, `BakeCurve`, `ExportSTEP`,
  `ExportSTL`, `ExportOBJ`, `Group`, `Cluster`.

### Previously implemented (commit 6dd4a02, 2026-08-18)

- **Surface Evaluation (14):** `SurfaceFrame`, `SurfaceCurvature`, `SurfaceAreaUV`,
  `IsoTrim`, `DivideSurface`, `SurfaceClosestPoint`, `SurfaceProjectPoint`,
  `SurfaceSplit`, `SurfaceFlip`, `SurfaceRebuild`, `SurfaceFromPoints`,
  `SurfaceIsocurve`, `SurfaceUntrim`, `SurfaceTrim`.
- **Solid Modification (9):** `DraftFace`, `MoveFace`, `OffsetFace`, `ReplaceFace`,
  `HoleCircular`, `Rib`, `FilletEdge`, `ChamferEdge`, `FilletVariable`.

### Previously implemented (commit b8131c9, 2026-08-18)

- **Curve (11):** `Polyline`, `NurbsCurve`, `NurbsCurveInterp`, `JoinCurves`,
  `CurveOffset`, `Extend`, `Rebuild`, `Tangent`, `Curvature`, `NearestPoint`,
  `SplitCurve`, `Arc3pt`.
- **Surface Creation (8):** `Extrude`, `Revolve`, `Loft`, `Sweep`,
  `RuledSurface`, `PlaneSurface`, `ExtrudePoint`, `ExtrudeTapered`.
- **Surface Evaluation (2):** `EvaluateSurface`, `SurfaceNormal`.
- **Modify (6):** `Shell`, `Thicken`, `OffsetSolid`, `Hole`, `SplitSolid`, `TrimSolid`.
- **Transform (5):** `Orient`, `Project`, `ArrayAlongCurve`, `ArrayBox`, `ArrayOnSurface`.
- **Intersect (6):** `CurveCurveIntersect`, `CurveSurfaceIntersect`,
  `SurfaceSurfaceIntersect`, `PlanePlaneIntersect`, `CurvePlaneIntersect`,
  `SolidPlaneIntersect`.
- **Mesh (5):** `MeshPrimitive`, `MeshWeld`, `MeshSubdivide`, `MeshDecimate`, `MeshSmooth`.

Previously implemented (commit 8f859c3, 2026-08-16):
- Params (3): `PlaneInput`, `DomainInput`, `StringInput`.
- Analysis (7): `Volume`, `SurfaceArea`, `Centroid`, `BoundingBox`, `Distance`, `Angle`, `MassProperties`.
- Vector/Math (18): `Cross`, `Dot`, `VectorLength`, `Unit`, `Negative`, `Reciprocal`,
  `Asin`, `Acos`, `Atan`, `Atan2`, `Log`, `Ln`, `Exp`, `Modulus`, `MapDomain`,
  `PointMidpoint`, `PointLerp`, `Vector2pt`.
- Transform (2): `RotateAxis`, `MirrorPlane`.
- Curve (5): `Arc`, `Ellipse`, `Flip`, `EndPoints`, `PointAt`.
- Transform ops (2): `ComposeTransform`, `InvertTransform`.
- Mesh (4): `ToMesh`, `MeshArea`, `MeshVolume`, `MeshFlip`.
- Boolean (1): `BooleanSplit`.

### Remaining stubs (need API work)

- Surface nodes requiring `Surface::Cylinder`/`Sphere`/`Torus` outputs from planar
  inputs: `CylinderSurface`, `ConeSurface`, `SphereSurface`, `TorusSurface`,
  `OffsetSurface` (from existing surface — needs NURBS offset support).
- Curve nodes needing extended curve API: `CurveBooleanUnion`, `CurveBooleanSubtract`,
  `CurveBooleanIntersect`, `CurveShatter`, `CurveDiscontinuity`, `CurveFrame`,
  `CurveNormal`, `ProjectCurveToPlane`, `ProjectCurveToSurface`, `CurveSeam`.
- Intersect: `BrepBrepIntersect`, `MeshMeshIntersect`, `LineLineClosestPoint`,
  `SolidInclusion`, `CollisionCheck`, `BooleanTrim`, `MeshBooleanUnion`/`Subtract`/`Intersect`.
- Analysis: `MomentsOfInertia`, `CurveCurvatureAnalysis`, `SurfaceCurvatureAnalysis`,
  `PointInSolid`, `PointInCurve`, `ClosestPointOnSurface`, `ClosestPointOnCurve`,
  `SelfIntersect`, `Planar`, `Closed`.
- Params: `TransformInput`, `ColorInput`, `FileInput`, `PathInput`.

---

## 0. Prerequisites — New PortType & VpData Variants

Before adding the nodes below, the underlying data model must be extended.
The current `VpData` only has `Curve(Vec<Point3d>)` (a sampled polyline) and
no `Surface` variant at all. The new nodes need rich, parametric data types.

### 0.1 New `PortType` variants (add to `PortType` enum)

```rust
pub enum PortType {
    // Existing
    Geometry, Curve, Number, Integer, Boolean, Point, Vector, String, List, Any,
    // ── New ──
    Surface,      // parametric Surface (Plane, Cylinder, NurbsSurface, …)
    Curve3d,      // parametric Curve3d (Line, Circle, Arc, NurbsCurve, …)
    Plane,        // infinite plane reference (origin + normal + x-dir)
    Domain,       // (min, max) interval
    Transform,    // 4×4 transformation matrix
    Mesh,         // TriangleMesh (already in VpData but no port type!)
    Brep,         // alias for Geometry/Solid (topology)
    Color,        // (r, g, b, a) for display only
    Path,         // data-tree path {branch_index: u32, indices: Vec<i32>}
}
```

`PortType::accepts()` must be extended to allow:
- `Curve3d` ↔ `Curve` (implicit sampling / wrapping)
- `Geometry` → `Surface`, `Mesh` (extract)
- `Surface` → `Geometry` (wrap as a shell)
- `Plane` → `Surface` (a finite patch)
- `Point` ↔ `Vector` (already allowed)

### 0.2 New `VpData` variants (add to `VpData` enum)

```rust
pub enum VpData {
    // Existing
    Geometry(Box<draper_topology::Solid>),
    Mesh(Box<draper_mesh::TriangleMesh>),
    Curve(Vec<draper_geometry::Point3d>),        // sampled polyline (legacy)
    Number(f64), Integer(i64), Boolean(bool),
    Point([f64; 3]), Vector([f64; 3]), String(String),
    List(Vec<VpData>), Empty,
    // ── New ──
    /// Parametric curve — preserves Line/Circle/Arc/NurbsCurve type info.
    Curve3d(Box<draper_geometry::Curve3d>),
    /// Parametric surface — Plane/Cylinder/Sphere/NurbsSurface/etc.
    Surface(Box<draper_geometry::Surface>),
    /// Infinite plane reference.
    Plane(Box<draper_geometry::Plane>),
    /// Interval [min, max].
    Domain { min: f64, max: f64 },
    /// 4×4 transformation matrix.
    Transform(Box<draper_geometry::Transform>),
    /// Compound shape (multiple solids).
    Compound(Box<draper_topology::Compound>),
    /// RGBA color.
    Color([f64; 4]),
    /// Data-tree path (for tree operations like Path Mapper).
    Path { branch: u32, indices: Vec<i32> },
}
```

### 0.3 Underlying API surface (already implemented in `draper-*`)

The following APIs are confirmed present and can back the new nodes directly:

| API | Location | Used by |
|-----|----------|---------|
| `Curve3d::{Line, Circle, Ellipse, Arc, Hyperbola, Parabola, Nurbs, PCurve, Trimmed, Composite}` | `draper-geometry/src/curve.rs` | Curve nodes |
| `Surface::{Plane, Cylinder, Cone, Sphere, Torus, Revolution, Extrusion, Nurbs, Offset, Ruled}` | `draper-geometry/src/surface.rs` | Surface nodes |
| `Transform::{translation, scaling, rotation_x/y/z, rotation_axis, inverse, multiply}` | `draper-geometry/src/transform.rs` | Transform nodes |
| `ShapeBuilder::{make_box, make_cylinder, make_sphere, make_cone, make_torus, make_revolution, make_extrusion, make_polygon_face, make_disk}` | `draper-topology/src/builder.rs` | Primitives |
| `operations::{fillet_edge, chamfer_edge, shell_solid, draft_face, extrude_polyline, revolve_polyline, sweep_polyline, loft_polylines, sweep_wire_along_curve, loft_wires, move_face_planar, offset_face_planar, replace_face_planar, split_face}` | `draper-topology/src/operations.rs` | Modify / Surface |
| `queries::{solid_volume, solid_surface_area, solid_center_of_mass, point_in_solid, solid_moments_of_inertia, solid_bounding_box}` | `draper-topology/src/queries.rs` | Analysis |
| `boolean::{boolean_union, boolean_subtract, boolean_intersect, boolean_operation, intersect_surfaces, intersect_curve_surface, split_face}` | `draper-topology/src/boolean.rs` | Boolean / Intersect |
| `intersection::{intersect_line_plane, intersect_line_cylinder, intersect_line_sphere, intersect_plane_cylinder, intersect_cylinder_cylinder, intersect_surfaces, intersect_curve_surface, closest_point_on_curve}` | `draper-geometry/src/intersection.rs` | Intersect |
| `nurbs_tools::{insert_knot, remove_knot, clamp, optimize, cut, bezier_decomposition, elevate_degree}` | `draper-geometry/src/nurbs_tools.rs` | NurbsCurve ops |
| `Bvh::{build, ray_intersect, closest_point, frustum_cull}` | `draper-topology/src/queries.rs` | Mesh raycast |
| `Plane::{xy, xz, yz, from_origin_and_normal, from_three_points, project_point}` | `draper-geometry/src/surface.rs` | Plane ops |

---

## 1. Params — Input Parameters

### 1.1 `PlaneInput`
- **Fields:** `{ origin: [f64; 3], normal: [f64; 3], x_dir: Option<[f64; 3]> }`
- **Inputs:** none
- **Outputs:** `P: Plane`, `O: Point` (origin), `N: Vector` (normal)
- **Description:** Standalone infinite-plane parameter. Backed by `Plane::from_origin_and_normal`.
- **GH analogue:** `Plane` parameter.

### 1.2 `DomainInput`
- **Fields:** `{ min: f64, max: f64 }`
- **Inputs:** none
- **Outputs:** `D: Domain`, `Min: Number`, `Max: Number`
- **Description:** Numeric interval `[min, max]` for `Range`, `Series`, `IsoTrim` etc.
- **GH analogue:** `Domain` parameter.

### 1.3 `StringInput`
- **Fields:** `{ value: String, multiline: bool }`
- **Inputs:** none
- **Outputs:** `S: String`
- **Description:** Editable text parameter (single or multi-line).
- **GH analogue:** `Panel` input mode + `Text` parameter.

### 1.4 `TransformInput`
- **Fields:** `{ translation: [f64; 3], rotation_deg: [f64; 3], scale: [f64; 3] }`
- **Inputs:** none
- **Outputs:** `T: Transform`
- **Description:** Author a 4×4 transform directly (vs. composing from Move/Rotate/Scale).
- **GH analogue:** `Transform` parameter.

### 1.5 `ColorInput`
- **Fields:** `{ r: f32, g: f32, b: f32, a: f32 }`
- **Inputs:** none
- **Outputs:** `C: Color`
- **Description:** RGBA color swatch for downstream display nodes.
- **GH analogue:** `Colour Swatch`.

### 1.6 `FileInput` *(optional — STEP/IGES)*
- **Fields:** `{ path: String }`
- **Inputs:** none
- **Outputs:** `G: Geometry` (Solid loaded via `draper-step`)
- **Description:** Load a CAD file as a parameter. Already supported by `draper-step`.
- **GH analogue:** `File Path` + `Import STEP`.

### 1.7 `PathInput`
- **Fields:** `{ branch: u32, indices: Vec<i32> }`
- **Inputs:** none
- **Outputs:** `P: Path`
- **Description:** Data-tree path for `PathMapper`, `TreeBranch`.

---

## 2. Maths — Mathematical Operations

All existing math nodes (`Add` … `Expression`) keep their signatures.

### 2.1 `Asin` / `Acos` / `Atan`
- **Fields:** none
- **Inputs:** `X: Number` (for `Asin`/`Acos`); `X: Number` (for `Atan` — single-arg form)
- **Outputs:** `R: Number` (radians)
- **Description:** Inverse trig functions. Wrap `f64::asin/acos/atan`.

### 2.2 `Atan2`
- **Fields:** none
- **Inputs:** `Y: Number`, `X: Number`
- **Outputs:** `R: Number` (radians, full range)
- **Description:** Four-quadrant arctangent. `f64::atan2(y, x)`.

### 2.3 `Log` / `Ln` / `Exp`
- **Fields:** none
- **Inputs:** `X: Number`
- **Outputs:** `R: Number`
- **Description:** `log10`, `ln`, `exp` respectively.

### 2.4 `Modulus`
- **Fields:** none
- **Inputs:** `A: Number`, `B: Number`
- **Outputs:** `R: Number`
- **Description:** `A % B` (floating-point remainder).

### 2.5 `Negative`
- **Fields:** none
- **Inputs:** `X: Number`
- **Outputs:** `R: Number`
- **Description:** Unary negation `-X`.

### 2.6 `Reciprocal`
- **Fields:** none
- **Inputs:** `X: Number`
- **Outputs:** `R: Number`
- **Description:** `1.0 / X` (with zero-guard → `f64::INFINITY`).

### 2.7 `Truncate` / `Fraction`
- **Fields:** none
- **Inputs:** `X: Number`
- **Outputs:** `I: Integer` (`Truncate`), `F: Number` (`Fraction`)
- **Description:** Split a float into integer and fractional parts (`f64::trunc`, `f64::fract`).

### 2.8 `Degrees` / `Radians`
- **Fields:** none
- **Inputs:** `X: Number`
- **Outputs:** `R: Number`
- **Description:** Convert rad↔deg (`X.to_degrees()` / `X.to_radians()`).

### 2.9 `Sign`
- **Fields:** none
- **Inputs:** `X: Number`
- **Outputs:** `R: Integer` ∈ {-1, 0, +1}
- **Description:** Signum function as integer.

### 2.10 `Factorial`
- **Fields:** none
- **Inputs:** `N: Integer`
- **Outputs:** `R: Number`
- **Description:** `n!` as `f64` (handles `n ≤ 170`).

### 2.11 `MapDomain`
- **Fields:** none
- **Inputs:** `V: Number`, `S: Domain` (source), `T: Domain` (target)
- **Outputs:** `R: Number`
- **Description:** Remap `V` from `S` to `T` (linear lerp).

### 2.12 `Bounds`
- **Fields:** none
- **Inputs:** `L: List<Number>`
- **Outputs:** `D: Domain` (min, max)
- **Description:** Find numeric bounds of a list — useful for color/param mapping.

---

## 3. Vector — Vector Operations *(new category)*

### 3.1 `VectorCreate`
- **Fields:** none
- **Inputs:** `X: Number`, `Y: Number`, `Z: Number`
- **Outputs:** `V: Vector`
- **Description:** Compose a vector from 3 numbers (already implicit in `VectorInput`, but allows per-component math chains).

### 3.2 `VectorDecompose`
- **Fields:** none
- **Inputs:** `V: Vector`
- **Outputs:** `X: Number`, `Y: Number`, `Z: Number`
- **Description:** Split a vector into its 3 components.

### 3.3 `VectorAdd` / `VectorSubtract`
- **Fields:** none
- **Inputs:** `A: Vector`, `B: Vector`
- **Outputs:** `R: Vector`
- **Description:** Vector ± vector (`Vec3d::add` / `Vec3d::sub`).

### 3.4 `VectorScale`
- **Fields:** none
- **Inputs:** `V: Vector`, `S: Number`
- **Outputs:** `R: Vector`
- **Description:** Scale a vector by a scalar (`Vec3d::scale`).

### 3.5 `VectorDot`
- **Fields:** none
- **Inputs:** `A: Vector`, `B: Vector`
- **Outputs:** `R: Number`
- **Description:** Dot product (`Vec3d::dot`).

### 3.6 `VectorCross`
- **Fields:** none
- **Inputs:** `A: Vector`, `B: Vector`
- **Outputs:** `R: Vector`
- **Description:** Cross product (`Vec3d::cross`).

### 3.7 `VectorLength`
- **Fields:** none
- **Inputs:** `V: Vector`
- **Outputs:** `L: Number`
- **Description:** Euclidean length (`Vec3d::length`).

### 3.8 `VectorUnit`
- **Fields:** none
- **Inputs:** `V: Vector`
- **Outputs:** `U: Vector` (normalized), `L: Number` (original length)
- **Description:** Normalize a vector (`Vec3d::normalize`). Zero-vector → zero output.

### 3.9 `VectorReverse`
- **Fields:** none
- **Inputs:** `V: Vector`
- **Outputs:** `R: Vector`
- **Description:** Negate a vector (`Vec3d::neg`).

### 3.10 `VectorAngle`
- **Fields:** none
- **Inputs:** `A: Vector`, `B: Vector`
- **Outputs:** `R: Number` (radians)
- **Description:** Angle between two vectors (`Direction3d::angle_to` after normalization).

### 3.11 `Vector2pt`
- **Fields:** none
- **Inputs:** `A: Point`, `B: Point`
- **Outputs:** `V: Vector` (B − A)
- **Description:** Construct a vector from two points.

### 3.12 `VectorProject`
- **Fields:** none
- **Inputs:** `V: Vector`, `N: Vector` (direction to project onto)
- **Outputs:** `R: Vector`
- **Description:** Project `V` onto direction `N`: `(V·N̂) N̂`.

### 3.13 `PointAdd` / `PointSubtract`
- **Fields:** none
- **Inputs:** `P: Point`, `V: Vector`
- **Outputs:** `R: Point`
- **Description:** Translate a point by a vector (`Point3d::new` arithmetic).

### 3.14 `PointDistance`
- **Fields:** none
- **Inputs:** `A: Point`, `B: Point`
- **Outputs:** `D: Number`
- **Description:** Euclidean distance (`Point3d::distance_to`).

### 3.15 `PointLerp`
- **Fields:** none
- **Inputs:** `A: Point`, `B: Point`, `T: Number`
- **Outputs:** `R: Point`
- **Description:** Linear interpolation between points (`Point3d::lerp`).

### 3.16 `PointMidpoint`
- **Fields:** none
- **Inputs:** `A: Point`, `B: Point`
- **Outputs:** `M: Point`
- **Description:** Midpoint (`Point3d::midpoint`).

---

## 4. Sets — List / Tree Operations

Existing: `Series, Range, ListLength, ListItem, Reverse, Sort, CullPattern`.

### 4.1 `CullIndex`
- **Fields:** `{ wrap: bool }`
- **Inputs:** `L: List<Any>`, `I: List<Integer>` (indices to remove)
- **Outputs:** `R: List<Any>`
- **Description:** Remove items at the given indices. GH `Cull Index`.

### 4.2 `Partition`
- **Fields:** `{ size: u32 }`
- **Inputs:** `L: List<Any>`
- **Outputs:** `R: List<List<Any>>`
- **Description:** Split list into chunks of `size`.

### 4.3 `ReplaceItems`
- **Fields:** none
- **Inputs:** `L: List<Any>`, `I: List<Integer>`, `V: List<Any>`
- **Outputs:** `R: List<Any>`
- **Description:** Replace items at indices `I` with values `V`.

### 4.4 `Sift`
- **Fields:** none
- **Inputs:** `L: List<Any>`, `P: List<Boolean>`
- **Outputs:** `T: List<Any>` (true items), `F: List<Any>` (false items, padded with nulls to maintain indices)
- **Description:** Like `Dispatch` but retains index positions in each output.

### 4.5 `Combine`
- **Fields:** none
- **Inputs:** `A: List<Any>`, `B: List<Any>`
- **Outputs:** `R: List<Any>`
- **Description:** Alternating interleave from two lists (vs. `Concat` which appends).

### 4.6 `Duplicate`
- **Fields:** `{ count: u32 }`
- **Inputs:** `D: Any`
- **Outputs:** `R: List<Any>`
- **Description:** Replicate a single item N times.

### 4.7 `Null`
- **Fields:** none
- **Inputs:** none
- **Outputs:** `N: Any` (a null marker)
- **Description:** Emit a null placeholder for list-padding operations.

### 4.8 `ListMap`
- **Fields:** `{ graph_path: String }` *(sub-graph reference)*
- **Inputs:** `L: List<Any>`
- **Outputs:** `R: List<Any>`
- **Description:** Apply a sub-graph to every item. (Optional — advanced; cluster-like but not full programming.)

### 4.9 `PathMapper`
- **Fields:** `{ target_path: Vec<i32> }`
- **Inputs:** `T: Tree<Any>`
- **Outputs:** `R: Tree<Any>`
- **Description:** Remap data-tree paths. Requires `Path` PortType.

### 4.10 `TreeBranch`
- **Fields:** none
- **Inputs:** `T: Tree<Any>`, `P: Path`
- **Outputs:** `L: List<Any>`
- **Description:** Extract a single branch by path.

### 4.11 `TreeStatistics`
- **Fields:** none
- **Inputs:** `T: Tree<Any>`
- **Outputs:** `Paths: List<Path>`, `Counts: List<Integer>`
- **Description:** List all paths and item counts in a tree.

### 4.12 `CleanTree`
- **Fields:** `{ remove_nulls: bool, remove_empty: bool }`
- **Inputs:** `T: Tree<Any>`
- **Outputs:** `R: Tree<Any>`
- **Description:** Strip null items and empty branches.

### 4.13 `ExplodeTree`
- **Fields:** none
- **Inputs:** `T: Tree<Any>`
- **Outputs:** `L₁…Lₙ: List<Any>` (one per branch, dynamic)
- **Description:** Convert a tree into N separate list outputs.

---

## 5. Curve — Curve Creation & Evaluation

Existing: `Line, Circle, DivideCurve, EvaluateCurve, CurveLength`.

### 5.1 `Arc`
- **Fields:** `{ radius: f64, start_angle_deg: f64, end_angle_deg: f64 }`
- **Inputs:** `C: Point` (center), `N: Vector` (normal, optional — defaults Z)
- **Outputs:** `C: Curve3d`, `S: Point` (start), `E: Point` (end)
- **Description:** Arc by center, radius, angle range. Backed by `Arc::new(Circle::new_xy(...), start, end)`.
- **GH analogue:** `Arc`.

### 5.2 `Arc3pt`
- **Fields:** none
- **Inputs:** `A: Point`, `B: Point`, `C: Point`
- **Outputs:** `C: Curve3d`
- **Description:** Arc through three points. Compute circumcircle, then trim.
- **GH analogue:** `Arc 3Pt`.

### 5.3 `Ellipse`
- **Fields:** `{ radius_x: f64, radius_y: f64 }`
- **Inputs:** `C: Point` (center), `N: Vector` (normal, optional)
- **Outputs:** `C: Curve3d`, `F1: Point`, `F2: Point` (foci)
- **Description:** Ellipse by center + two radii. Backed by `Ellipse::new_xy`.
- **GH analogue:** `Ellipse`.

### 5.4 `Polyline`
- **Fields:** `{ closed: bool }`
- **Inputs:** `P: List<Point>`
- **Outputs:** `C: Curve3d` (as `Curve3d::Composite` of `Line`s)
- **Description:** Polyline through points; closes if `closed=true`.
- **GH analogue:** `Polyline`.

### 5.5 `NurbsCurve`
- **Fields:** `{ degree: u32, closed: bool }`
- **Inputs:** `P: List<Point>` (control points)
- **Outputs:** `C: Curve3d`
- **Description:** Build a clamped B-spline through control points.
- **GH analogue:** `Nurbs Curve`.

### 5.6 `NurbsCurveInterp`
- **Fields:** `{ degree: u32 }`
- **Inputs:** `P: List<Point>` (points to pass through)
- **Outputs:** `C: Curve3d`
- **Description:** Interpolating B-spline (global interpolation).
- **GH analogue:** `Interpolate` / `Nurbs Curve Pt`.

### 5.7 `Hyperbola` / `Parabola`
- **Fields:** `{ semi_real: f64, semi_imag: f64 }` (Hyperbola) / `{ focal_dist: f64 }` (Parabola)
- **Inputs:** `C: Point` (center/vertex), `N: Vector` (axis)
- **Outputs:** `C: Curve3d`
- **Description:** Conic sections. Backed by `Hyperbola::new_xy` / `Parabola::new_xy`.
- **GH analogue:** `Hyperbola`, `Parabola`.

### 5.8 `JoinCurves`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `C: List<Curve3d>`
- **Outputs:** `C: Curve3d` (as `Curve3d::Composite`)
- **Description:** Join contiguous curves into one. Use `Curve3d::Composite`.
- **GH analogue:** `Join`.

### 5.9 `CurveOffset`
- **Fields:** `{ distance: f64, corners: CornerType }`
- **Inputs:** `C: Curve3d`, `P: Plane` (offset plane)
- **Outputs:** `R: Curve3d`
- **Description:** Offset a planar curve by `distance`. Implement via polygon-offset (earcut-based).
- **GH analogue:** `Offset`.

### 5.10 `CurveExtend`
- **Fields:** `{ start: f64, end: f64 }`
- **Inputs:** `C: Curve3d`
- **Outputs:** `R: Curve3d`
- **Description:** Extend curve by absolute lengths at both ends. For Lines/Arcs, adjust param range; for Nurbs, extend tangentially.
- **GH analogue:** `Extend`.

### 5.11 `CurveFlip`
- **Fields:** none
- **Inputs:** `C: Curve3d`
- **Outputs:** `R: Curve3d`
- **Description:** Reverse curve direction.
- **GH analogue:** `Flip`.

### 5.12 `CurveRebuild`
- **Fields:** `{ degree: u32, point_count: u32 }`
- **Inputs:** `C: Curve3d`
- **Outputs:** `R: Curve3d`
- **Description:** Refit curve to a NURBS of given degree/point-count.
- **GH analogue:** `Rebuild`.

### 5.13 `CurveSeam`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `T: Number` (parameter)
- **Outputs:** `R: Curve3d`
- **Description:** For closed curves, change the start point.
- **GH analogue:** `Seam`.

### 5.14 `CurvePointAt`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `T: Number`
- **Outputs:** `P: Point`
- **Description:** Evaluate point at parameter (`Curve3d::point_at`).
- **GH analogue:** `Point On Curve`.

### 5.15 `CurveTangent`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `T: Number`
- **Outputs:** `T: Vector`
- **Description:** Evaluate tangent vector (`Curve3d::derivative_at`, normalized).
- **GH analogue:** `Curve Tangent`.

### 5.16 `CurveFrame`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `T: Number`
- **Outputs:** `F: Plane` (origin + tangent + normal)
- **Description:** Perpendicular frame at parameter (Frenet frame).
- **GH analogue:** `Perp Frame`.

### 5.17 `CurveNormal`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `T: Number`, `P: Plane` (reference)
- **Outputs:** `N: Vector`
- **Description:** Normal vector at parameter (in plane perpendicular to tangent).
- **GH analogue:** `Curve Normal`.

### 5.18 `CurveCurvature`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `T: Number`
- **Outputs:** `P: Point`, `K: Number` (curvature magnitude), `D: Vector` (curvature direction)
- **Description:** Curvature vector and magnitude at parameter.
- **GH analogue:** `Curvature`.

### 5.19 `CurveDiscontinuity`
- **Fields:** `{ level: Continuity }`
- **Inputs:** `C: Curve3d`
- **Outputs:** `P: List<Point>` (discontinuity points)
- **Description:** Find points where continuity drops below `level` (G0/G1/G2).
- **GH analogue:** `Discontinuity`.

### 5.20 `CurveNearestPoint`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `P: Point`
- **Outputs:** `T: Number` (parameter), `Q: Point` (closest)
- **Description:** Newton iteration via `closest_point_on_curve`.
- **GH analogue:** `Curve Closest Point`.

### 5.21 `CurveEndPoints`
- **Fields:** none
- **Inputs:** `C: Curve3d`
- **Outputs:** `S: Point`, `E: Point`
- **Description:** Start and end points of the curve's param range.

### 5.22 `CurveBooleanUnion` / `CurveBooleanSubtract` / `CurveBooleanIntersect`
- **Fields:** none
- **Inputs:** `A: List<Curve3d>`, `B: List<Curve3d>` (subtract only)
- **Outputs:** `R: List<Curve3d>`
- **Description:** Planar boolean ops on closed curves (implement via earcut + clipping).
- **GH analogue:** `Curve Boolean`.

### 5.23 `CurveSplit`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `P: List<Point>` (or `T: List<Number>`)
- **Outputs:** `R: List<Curve3d>`
- **Description:** Split curve at given parameters / points. Backed by `nurbs_tools::cut` for NURBS.

### 5.24 `CurveShatter`
- **Fields:** none
- **Inputs:** `C: List<Curve3d>`
- **Outputs:** `S: List<Curve3d>` (segments)
- **Description:** Shatter curves at every discontinuity.
- **GH analogue:** `Shatter`.

### 5.25 `ProjectCurveToPlane`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `P: Plane`
- **Outputs:** `R: Curve3d`
- **Description:** Project curve onto plane (drop normal component).
- **GH analogue:** `Project`.

### 5.26 `ProjectCurveToSurface`
- **Fields:** `{ direction: Option<[f64;3]> }` (None = normal-project)
- **Inputs:** `C: Curve3d`, `S: Surface`
- **Outputs:** `R: Curve3d` (as `Curve3d::PCurve`)
- **Description:** Project curve onto surface along direction; result is a PCurve.
- **GH analogue:** `Project` (curve→surface).

---

## 6. Surface — Surface Creation & Evaluation *(new category — biggest gap)*

### 6.1 `PlaneSurface`
- **Fields:** `{ size_x: f64, size_y: f64 }`
- **Inputs:** `P: Plane` (origin + frame)
- **Outputs:** `S: Surface`, `G: Geometry` (as a single-face solid)
- **Description:** Rectangular finite patch on an infinite plane. Backed by `Surface::Plane` + bounded by a 4-point polygon wire.
- **GH analogue:** `Plane Surface`.

### 6.2 `CylinderSurface`
- **Fields:** `{ radius: f64, height: f64 }`
- **Inputs:** `P: Plane` (axis from normal)
- **Outputs:** `S: Surface` (lateral only), `G: Geometry` (full solid)
- **Description:** Cylinder as a surface (not solid). Backed by `Surface::Cylinder` + `ShapeBuilder::make_cylinder`.
- **GH analogue:** `Cylinder` surface variant.

### 6.3 `ConeSurface`
- **Fields:** `{ radius: f64, height: f64 }`
- **Inputs:** `P: Plane`
- **Outputs:** `S: Surface`, `G: Geometry`
- **Description:** Cone lateral surface + solid. Backed by `Surface::Cone` + `ShapeBuilder::make_cone`.

### 6.4 `SphereSurface`
- **Fields:** `{ radius: f64 }`
- **Inputs:** `C: Point` (center)
- **Outputs:** `S: Surface`, `G: Geometry`
- **Description:** Sphere surface + solid. Backed by `Surface::Sphere` + `ShapeBuilder::make_sphere`.

### 6.5 `TorusSurface`
- **Fields:** `{ major: f64, minor: f64 }`
- **Inputs:** `P: Plane`
- **Outputs:** `S: Surface`, `G: Geometry`
- **Description:** Torus surface + solid. Backed by `Surface::Torus` + `ShapeBuilder::make_torus`.

### 6.6 `ExtrudeSurface`
- **Fields:** `{ distance: f64 }`
- **Inputs:** `P: Curve3d` (profile), `D: Vector` (direction)
- **Outputs:** `S: Surface` (lateral), `G: Geometry` (solid if profile closed)
- **Description:** Extrude a profile along a vector. Surface via `Surface::Extrusion`, solid via `operations::extrude_polyline` or `ShapeBuilder::make_extrusion`.
- **GH analogue:** `Extrude`.

### 6.7 `ExtrudePoint`
- **Fields:** none
- **Inputs:** `P: Curve3d` (profile), `A: Point` (apex)
- **Outputs:** `S: Surface`
- **Description:** Tapered extrude to a point — creates a ruled surface between profile and apex.
- **GH analogue:** `Extrude Point`.

### 6.8 `ExtrudeTapered`
- **Fields:** `{ distance: f64, draft_angle_deg: f64 }`
- **Inputs:** `P: Curve3d`, `D: Vector`
- **Outputs:** `S: Surface`
- **Description:** Extrude with draft angle (useful for molds).
- **GH analogue:** `Extrude Tapered`.

### 6.9 `Sweep1`
- **Fields:** `{ closed: bool }`
- **Inputs:** `P: Curve3d` (profile), `R: Curve3d` (rail)
- **Outputs:** `S: Surface`, `G: Geometry` (if profile closed)
- **Description:** Sweep profile along a single rail. Backed by `operations::sweep_polyline` / `sweep_wire_along_curve`.
- **GH analogue:** `Sweep 1`.

### 6.10 `Sweep2`
- **Fields:** `{ closed: bool }`
- **Inputs:** `P: Curve3d` (profile), `R1: Curve3d`, `R2: Curve3d` (two rails)
- **Outputs:** `S: Surface`
- **Description:** Sweep along two rails (profile scales to touch both). Adapt `sweep_polyline` with per-step re-scaling.
- **GH analogue:** `Sweep 2`.

### 6.11 `Loft`
- **Fields:** `{ continuity: Continuity, closed: bool, type: LoftType }`
- **Inputs:** `S: List<Curve3d>` (sections)
- **Outputs:** `S: Surface`, `G: Geometry` (if sections closed)
- **Description:** Loft through sections. Backed by `operations::loft_polylines` / `loft_wires`.
- **GH analogue:** `Loft`.

### 6.12 `Revolve`
- **Fields:** `{ angle_deg: f64 }`
- **Inputs:** `P: Curve3d` (profile), `A: Line` (axis — Point + Vector)
- **Outputs:** `S: Surface` (Revolution), `G: Geometry` (solid if closed)
- **Description:** Revolve profile around axis by angle. Backed by `Surface::Revolution` + `operations::revolve_polyline` / `ShapeBuilder::make_revolution`.
- **GH analogue:** `Revolve`.

### 6.13 `RuledSurface`
- **Fields:** none
- **Inputs:** `A: Curve3d`, `B: Curve3d`
- **Outputs:** `S: Surface`
- **Description:** Linear interpolation between two curves. Backed by `Surface::Ruled`.
- **GH analogue:** `Ruled Surface` / `EdgeSrf` for 2 edges.

### 6.14 `EdgeSurface`
- **Fields:** none
- **Inputs:** `E1: Curve3d`, `E2: Curve3d`, `E3: Curve3d`, `E4: Curve3d`
- **Outputs:** `S: Surface`
- **Description:** Surface bounded by 4 edge curves (Coons patch). Implement as a loft in U × sweep in V.
- **GH analogue:** `EdgeSrf` (4-edge).

### 6.15 `SurfacePatch`
- **Fields:** `{ continuity: Continuity }`
- **Inputs:** `E: List<Curve3d>` (2–N edges)
- **Outputs:** `S: Surface`
- **Description:** N-sided patch fill. Implement as a Coons-Blended patch.
- **GH analogue:** `Patch`.

### 6.16 `NetworkSurface`
- **Fields:** `{ u_continuity: Continuity, v_continuity: Continuity }`
- **Inputs:** `U: List<Curve3d>`, `V: List<Curve3d>`
- **Outputs:** `S: Surface`
- **Description:** Surface through a UV grid of curves (Gordon surface).
- **GH analogue:** `Network Surface`.

### 6.17 `OffsetSurface`
- **Fields:** `{ distance: f64, both_sides: bool }`
- **Inputs:** `S: Surface`
- **Outputs:** `R: Surface` (+ optional `R2: Surface` if `both_sides`)
- **Description:** Offset surface along normals. Backed by `Surface::Offset`.
- **GH analogue:** `Offset Srf`.

### 6.18 `EvaluateSurface`
- **Fields:** none
- **Inputs:** `S: Surface`, `U: Number`, `V: Number`
- **Outputs:** `P: Point`
- **Description:** Evaluate surface point at (u, v). `Surface::point_at`.

### 6.19 `SurfaceNormal`
- **Fields:** none
- **Inputs:** `S: Surface`, `U: Number`, `V: Number`
- **Outputs:** `N: Vector`
- **Description:** Surface normal at (u, v). `Surface::normal_at`.

### 6.20 `SurfaceFrame`
- **Fields:** none
- **Inputs:** `S: Surface`, `U: Number`, `V: Number`
- **Outputs:** `F: Plane` (origin + 3 axes from `SurfaceDerivatives`)
- **Description:** Full local frame at (u, v): origin = S(u,v), X = dU/du, Y = dU/dv, Z = X×Y.

### 6.21 `SurfaceCurvature`
- **Fields:** none
- **Inputs:** `S: Surface`, `U: Number`, `V: Number`
- **Outputs:** `K1: Number` (max principal), `K2: Number` (min principal), `D1: Vector`, `D2: Vector`
- **Description:** Principal curvatures & directions via `SurfaceCurvature` struct.
- **GH analogue:** `Surface Curvature`.

### 6.22 `SurfaceArea`
- **Fields:** `{ u_samples: u32, v_samples: u32 }`
- **Inputs:** `S: Surface`
- **Outputs:** `A: Number`
- **Description:** Approximate area via Gaussian quadrature over UV domain.

### 6.23 `IsoTrim`
- **Fields:** `{ u0: f64, u1: f64, v0: f64, v1: f64 }`
- **Inputs:** `S: Surface`, `D: Domain` (overrides min/max if connected)
- **Outputs:** `R: Surface` (trimmed subset)
- **Description:** Extract a sub-patch of a surface.
- **GH analogue:** `Iso Trim`.

### 6.24 `DivideSurface`
- **Fields:** `{ u_count: u32, v_count: u32 }`
- **Inputs:** `S: Surface`
- **Outputs:** `P: List<Point>`, `N: List<Vector>`, `F: List<Plane>` (frames), `U: List<Number>`, `V: List<Number>`
- **Description:** Grid of points/normals/frames on surface.
- **GH analogue:** `Divide Surface`.

### 6.25 `SurfaceClosestPoint`
- **Fields:** none
- **Inputs:** `S: Surface`, `P: Point`
- **Outputs:** `U: Number`, `V: Number`, `Q: Point` (closest on surface)
- **Description:** Find UV of closest point. Newton iteration on `surface.project_point` or numeric.

### 6.26 `SurfaceProjectPoint`
- **Fields:** `{ direction: Option<[f64;3]> }` (None = along normal)
- **Inputs:** `P: List<Point>`, `S: Surface`
- **Outputs:** `Q: List<Point>`, `U: List<Number>`, `V: List<Number>`
- **Description:** Project points onto surface along direction.
- **GH analogue:** `Pull Point` / `Project`.

### 6.27 `SurfaceSplit`
- **Fields:** `{ direction: SplitDir, parameter: f64 }`
- **Inputs:** `S: Surface`
- **Outputs:** `A: Surface`, `B: Surface`
- **Description:** Split surface at U or V iso-curve.
- **GH analogue:** `Split Surface`.

### 6.28 `SurfaceFlip`
- **Fields:** none
- **Inputs:** `S: Surface`
- **Outputs:** `R: Surface`
- **Description:** Reverse normal direction.

### 6.29 `SurfaceRebuild`
- **Fields:** `{ u_degree: u32, v_degree: u32, u_count: u32, v_count: u32 }`
- **Inputs:** `S: Surface`
- **Outputs:** `R: Surface` (as `NurbsSurface`)
- **Description:** Refit to a NURBS of given params.

### 6.30 `SurfaceFromPoints`
- **Fields:** `{ u_count: u32, v_count: u32, degree: u32 }`
- **Inputs:** `P: List<Point>` (control grid, length = u_count × v_count)
- **Outputs:** `S: Surface` (NurbsSurface)
- **Description:** Build NURBS surface from a control-point grid.
- **GH analogue:** `Surface From Points`.

### 6.31 `SurfaceIsocurve`
- **Fields:** `{ direction: IsoDir }`
- **Inputs:** `S: Surface`, `T: Number` (U or V parameter)
- **Outputs:** `C: Curve3d`
- **Description:** Extract an iso-curve (constant U or V) from a surface.
- **GH analogue:** `Iso Curve`.

### 6.32 `SurfaceUntrim`
- **Fields:** none
- **Inputs:** `S: Surface` (or `G: Geometry`)
- **Outputs:** `R: Surface`
- **Description:** Remove all trims, returning the underlying surface.

---

## 7. Primitives — Solid Geometry Creation

Existing: `Box, Sphere, Cylinder, Cone, Torus`.

### 7.1 `PlanePrimitive` *(rectangular surface, not infinite)*
- **Fields:** `{ width: f64, height: f64 }`
- **Inputs:** `P: Plane` (orientation)
- **Outputs:** `G: Geometry` (single-face solid), `S: Surface`
- **Description:** A rectangular planar solid. Backed by `ShapeBuilder::make_polygon_face` with 4 corner points.

### 7.2 `PolygonPrism`
- **Fields:** `{ sides: u32, radius: f64, height: f64 }`
- **Inputs:** `P: Plane`
- **Outputs:** `G: Geometry`
- **Description:** Regular n-gonal prism. Build profile via `Polyline2d::circle(n)` then `extrude_polyline`.

### 7.3 `Tube`
- **Fields:** `{ radius: f64 }`
- **Inputs:** `C: Curve3d` (path)
- **Outputs:** `G: Geometry`
- **Description:** Pipe along a curve (sweep a circle along path). Backed by `sweep_wire_along_curve`.
- **GH analogue:** `Pipe`.

### 7.4 `Helix`
- **Fields:** `{ radius: f64, pitch: f64, turns: f64 }`
- **Inputs:** `P: Plane` (axis)
- **Outputs:** `C: Curve3d`
- **Description:** Helical curve. Build as a NURBS via point sampling.
- **GH analogue:** `Helix`.

### 7.5 `PointCloud` *(display-only — skip?)*
- **Inputs:** `P: List<Point>` → **skip** (display-only per user spec).

### 7.6 `Wedge`
- **Fields:** `{ radius: f64, angle_deg: f64, height: f64 }`
- **Inputs:** `P: Plane`
- **Outputs:** `G: Geometry`
- **Description:** Wedge / pie-shape solid. Build profile via `Polyline2d` arc segment + center, then extrude.

### 7.7 `Tetra` / `Octa` / `Icosa` *(Platonic solids)*
- **Fields:** `{ radius: f64 }`
- **Inputs:** `P: Plane` (orientation)
- **Outputs:** `G: Geometry`
- **Description:** Platonic solids. Build vertices + faces manually.
- **GH analogue:** `Platonic Solids`.

---

## 8. Transform — Geometry Transformations

Existing: `Move, Rotate (Euler XYZ), Scale (XYZ), Mirror (3 preset planes), LinearArray, CircularArray`.

### 8.1 `RotateAxis`
- **Fields:** `{ angle_deg: f64 }`
- **Inputs:** `G: Geometry`, `O: Point` (origin), `A: Vector` (axis)
- **Outputs:** `G: Geometry`
- **Description:** Rotate around arbitrary axis. Backed by `Transform::rotation_axis` + `Transform::translation`. (More flexible than the existing Euler `Rotate`.)
- **GH analogue:** `Rotate Axis`.

### 8.2 `MirrorPlane`
- **Fields:** none
- **Inputs:** `G: Geometry`, `P: Plane` (mirror plane)
- **Outputs:** `G: Geometry`
- **Description:** Mirror across an arbitrary plane (generalization of `Mirror`).
- **GH analogue:** `Mirror` (P-input variant).

### 8.3 `Orient`
- **Fields:** none
- **Inputs:** `G: Geometry`, `S: Plane` (source), `T: Plane` (target)
- **Outputs:** `G: Geometry`
- **Description:** Map geometry from source plane to target plane (change of basis). Compute `T_source_to_world → T_world_to_target`.
- **GH analogue:** `Orient`.

### 8.4 `Orient3pt`
- **Fields:** none
- **Inputs:** `G: Geometry`, `A1,B1,C1: Point` (source), `A2,B2,C2: Point` (target)
- **Outputs:** `G: Geometry`
- **Description:** Orient by 3 source + 3 target points. Build source/target planes, then `Orient`.
- **GH analogue:** `Orient 3Pt`.

### 8.5 `Project`
- **Fields:** none
- **Inputs:** `G: Geometry`, `P: Plane` (target), `D: Vector` (direction)
- **Outputs:** `G: Geometry` (projected)
- **Description:** Project geometry onto a plane along a direction. For curves: drop components; for solids: project each face.
- **GH analogue:** `Project`.

### 8.6 `ProjectToSurface`
- **Fields:** `{ direction: Option<[f64;3]> }`
- **Inputs:** `G: Geometry`, `S: Surface`
- **Outputs:** `G: Geometry`
- **Description:** Project geometry onto a surface.
- **GH analogue:** `Project` (to surface).

### 8.7 `ArrayAlongCurve`
- **Fields:** `{ count: u32, spacing_mode: SpacingMode }`
- **Inputs:** `G: Geometry`, `C: Curve3d`
- **Outputs:** `G: Geometry` (Compound)
- **Description:** Array instances along a curve, oriented by curve frames.
- **GH analogue:** `Array Along Curve` / `Curve Array`.

### 8.8 `ArrayBox`
- **Fields:** `{ x: u32, y: u32, z: u32, spacing: f64 }`
- **Inputs:** `G: Geometry`
- **Outputs:** `G: Geometry` (Compound)
- **Description:** 3D grid array.
- **GH analogue:** `Box Array`.

### 8.9 `ArrayOnSurface`
- **Fields:** `{ u_count: u32, v_count: u32 }`
- **Inputs:** `G: Geometry`, `S: Surface`
- **Outputs:** `G: Geometry` (Compound, oriented by surface frames)
- **Description:** Distribute instances on a UV grid of a surface.
- **GH analogue:** `Surface Array`.

### 8.10 `ArrayPolar` *(generalization of CircularArray)*
- **Fields:** `{ count: u32, angle_deg: f64 }`
- **Inputs:** `G: Geometry`, `C: Point` (center), `A: Vector` (axis)
- **Outputs:** `G: Geometry` (Compound)
- **Description:** Polar array around arbitrary axis (vs existing `CircularArray` which is Z-axis only).
- **GH analogue:** `Polar Array`.

### 8.11 `Offset`
- **Fields:** `{ distance: f64, both_sides: bool }`
- **Inputs:** `G: Geometry`, `N: Vector` (direction)
- **Outputs:** `G: Geometry`
- **Description:** Offset solid along a direction (creates a "thickened" copy).
- **GH analogue:** `Offset` (solid).

### 8.12 `Shear`
- **Fields:** `{ xy: f64, xz: f64, yx: f64, yz: f64, zx: f64, zy: f64 }`
- **Inputs:** `G: Geometry`
- **Outputs:** `G: Geometry`
- **Description:** Shear transform. Build shear matrix.

### 8.13 `Taper`
- **Fields:** `{ draft_angle_deg: f64, height: f64 }`
- **Inputs:** `G: Geometry`, `A: Vector` (axis)
- **Outputs:** `G: Geometry`
- **Description:** Apply draft (useful for molded parts). Can use `operations::draft_face`.
- **GH analogue:** `Taper`.

### 8.14 `Compose`
- **Fields:** none
- **Inputs:** `T: List<Transform>`
- **Outputs:** `R: Transform`
- **Description:** Multiply multiple transforms in sequence (`Transform::multiply`).
- **GH analogue:** `Compose Transform`.

### 8.15 `InvertTransform`
- **Fields:** none
- **Inputs:** `T: Transform`
- **Outputs:** `R: Transform`
- **Description:** Inverse transform (`Transform::inverse`).

### 8.16 `ApplyTransform`
- **Fields:** none
- **Inputs:** `G: Geometry`, `T: Transform`
- **Outputs:** `G: Geometry`
- **Description:** Apply an authored `Transform` to geometry (decouples transform creation from application).

---

## 9. Intersect — Geometric Intersections *(new category)*

Existing only has Boolean ops (Union/Subtract/Intersect) under the "Boolean" category. Add explicit intersection queries.

### 9.1 `CurveCurveIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `A: Curve3d`, `B: Curve3d`
- **Outputs:** `P: List<Point>`, `T_A: List<Number>`, `T_B: List<Number>`
- **Description:** Intersection points between two curves. Implement via subdivision + Newton.
- **GH analogue:** `Curve | Curve (CCX)`.

### 9.2 `CurveSurfaceIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `C: Curve3d`, `S: Surface`
- **Outputs:** `P: List<Point>`, `T_C: List<Number>`, `UV: List<Point2>` (UV params)
- **Description:** Backed by `intersect_curve_surface`.
- **GH analogue:** `Curve | Surface (CSX)`.

### 9.3 `SurfaceSurfaceIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `A: Surface`, `B: Surface`
- **Outputs:** `C: List<Curve3d>`
- **Description:** Backed by `intersect_surfaces`.
- **GH analogue:** `Surface | Surface (SSX)`.

### 9.4 `BrepBrepIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `A: Geometry`, `B: Geometry`
- **Outputs:** `C: List<Curve3d>`, `P: List<Point>`
- **Description:** Intersection of two solids. Reuses `intersect_surfaces` over face pairs.
- **GH analogue:** `Brep | Brep (BBX)`.

### 9.5 `MeshMeshIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `A: Mesh`, `B: Mesh`
- **Outputs:** `L: List<Curve>` (polylines)
- **Description:** Mesh intersection. Use BVH + triangle-triangle tests.

### 9.6 `LineLineClosestPoint`
- **Fields:** none
- **Inputs:** `A: Curve3d` (Line), `B: Curve3d` (Line)
- **Outputs:** `P_A: Point`, `P_B: Point`, `D: Number` (distance)
- **Description:** Closest points between two (infinite or finite) lines.

### 9.7 `PlanePlaneIntersect`
- **Fields:** none
- **Inputs:** `A: Plane`, `B: Plane`
- **Outputs:** `L: Curve3d` (Line)
- **Description:** Intersection line of two planes.

### 9.8 `CurvePlaneIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `C: Curve3d`, `P: Plane`
- **Outputs:** `P: List<Point>`, `T: List<Number>`
- **Description:** Backed by `intersect_line_plane` for Lines; sample + Newton for general curves.
- **GH analogue:** `Curve | Plane (CPX)`.

### 9.9 `BrepPlaneIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `G: Geometry`, `P: Plane`
- **Outputs:** `C: List<Curve3d>` (cross-section curves)
- **Description:** Slice a solid with a plane. Loop over faces, intersect each with plane, join.
- **GH analogue:** `Brep | Plane (BPX)` / `Section`.

### 9.10 `MeshRayIntersect`
- **Fields:** none
- **Inputs:** `M: Mesh`, `O: Point` (ray origin), `D: Vector` (ray direction)
- **Outputs:** `P: List<Point>`, `T: List<Number>` (distances), `F: List<Integer>` (face indices)
- **Description:** Backed by `Bvh::ray_intersect`.

### 9.11 `SolidInclusion`
- **Fields:** none
- **Inputs:** `G: Geometry`, `P: Point`
- **Outputs:** `B: Boolean`
- **Description:** Backed by `queries::point_in_solid`.
- **GH analogue:** `Point In Brep` / `Inclosure`.

### 9.12 `CollisionCheck`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `A: Geometry`, `B: Geometry`
- **Outputs:** `C: Boolean` (collide), `P: List<Point>` (contacts)
- **Description:** AABB/OBB pre-check, then triangle-triangle.

### 9.13 `BooleanSplit`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `A: Geometry`, `B: Geometry`
- **Outputs:**`Above: Geometry`, `Below: Geometry`, `Intersection: Geometry`
- **Description:** Three-way split. Backed by `boolean_operation(BooleanOp::Split)` if available, else compute via union+subtract.

### 9.14 `BooleanTrim`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `G: Geometry`, `C: List<Curve3d>` (cutting curves, projected)
- **Outputs:** `R: Geometry`
- **Description:** Trim solid with curve-defined knives. Implement via `split_face`.

### 9.15 `MeshBooleanUnion` / `MeshBooleanSubtract` / `MeshBooleanIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `A: Mesh`, `B: Mesh`
- **Outputs:** `R: Mesh`
- **Description:** Mesh-level boolean ops. Backed by `draper-mesh::mesh_boolean`.

---

## 10. Modify — Geometry Modification

Existing: `Fillet, Chamfer`.

### 10.1 `Shell`
- **Fields:** `{ thickness: f64 }`
- **Inputs:** `G: Geometry`, `F: List<Integer>` (face indices to remove — optional)
- **Outputs:** `G: Geometry`
- **Description:** Hollow out a solid. Backed by `operations::shell_solid`.
- **GH analogue:** `Shell`.

### 10.2 `Thicken`
- **Fields:** `{ thickness: f64, both_sides: bool }`
- **Inputs:** `S: Surface` (or `G: Geometry` of single face)
- **Outputs:** `G: Geometry` (solid)
- **Description:** Thicken a surface into a solid by offsetting both sides and stitching.

### 10.3 `OffsetSolid`
- **Fields:** `{ distance: f64 }`
- **Inputs:** `G: Geometry`
- **Outputs:** `G: Geometry`
- **Description:** Offset all faces of a solid outward by `distance`.

### 10.4 `DraftFace`
- **Fields:** `{ angle_deg: f64 }`
- **Inputs:** `G: Geometry`, `F: Integer` (face index), `D: Vector` (draw direction)
- **Outputs:** `G: Geometry`
- **Description:** Backed by `operations::draft_face`.

### 10.5 `MoveFace`
- **Fields:** none
- **Inputs:** `G: Geometry`, `F: Integer`, `V: Vector`
- **Outputs:** `G: Geometry`
- **Description:** Backed by `operations::move_face_planar`.

### 10.6 `OffsetFace`
- **Fields:** `{ distance: f64 }`
- **Inputs:** `G: Geometry`, `F: Integer`
- **Outputs:** `G: Geometry`
- **Description:** Backed by `operations::offset_face_planar`.

### 10.7 `ReplaceFace`
- **Fields:** none
- **Inputs:** `G: Geometry`, `F: Integer`, `S: Surface`
- **Outputs:** `G: Geometry`
- **Description:** Backed by `operations::replace_face_planar`.

### 10.8 `SplitSolid`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `G: Geometry`, `C: List<Geometry>` (cutters)
- **Outputs:** `R: List<Geometry>` (pieces)
- **Description:** Split a solid by other solids.

### 10.9 `TrimSolid`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `G: Geometry`, `C: Geometry` (cutter)
- **Outputs:** `R: Geometry` (the inside portion)
- **Description:** Alias for boolean-intersect; semantically distinct for UI.

### 10.10 `Hole`
- **Fields:** `{ radius: f64, depth: f64, through: bool }`
- **Inputs:** `G: Geometry`, `P: Plane` (hole position + direction)
- **Outputs:** `G: Geometry`
- **Description:** Drill a cylindrical hole. Build a cylinder cutter, then boolean-subtract.

### 10.11 `HoleCircular` *(alias of Hole with simplified inputs)*
- **Fields:** `{ radius: f64 }`
- **Inputs:** `G: Geometry`, `C: Point` (center), `N: Vector` (axis)
- **Outputs:** `G: Geometry`

### 10.12 `Rib`
- **Fields:** `{ thickness: f64 }`
- **Inputs:** `G: Geometry`, `P: Curve3d` (profile), `D: Vector` (direction)
- **Outputs:** `G: Geometry`
- **Description:** Add a rib feature (extrude profile, union with base).

### 10.13 `FilletEdge` *(generalization of Fillet with edge selection)*
- **Fields:** `{ radius: f64 }`
- **Inputs:** `G: Geometry`, `E: List<Integer>` (edge indices)
- **Outputs:** `G: Geometry`
- **Description:** Backed by `operations::fillet_edge` (per-edge).

### 10.14 `ChamferEdge` *(generalization of Chamfer with edge selection)*
- **Fields:** `{ distance: f64 }`
- **Inputs:** `G: Geometry`, `E: List<Integer>`
- **Outputs:** `G: Geometry`
- **Description:** Backed by `operations::chamfer_edge`.

### 10.15 `FilletVariable`
- **Fields:** `{ radii: Vec<f64> }`
- **Inputs:** `G: Geometry`, `E: List<Integer>`
- **Outputs:** `G: Geometry`
- **Description:** Fillet with per-edge radius.

### 10.16 `SurfaceTrim`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `S: Surface`, `C: List<Curve3d>` (trim curves in UV)
- **Outputs:** `R: Surface`
- **Description:** Trim a surface with closed curves. Backed by `operations::split_face`.

### 10.17 `SurfaceUntrim`
- Already in 6.32.

---

## 11. Analysis — Geometric Queries *(new category)*

### 11.1 `Volume`
- **Fields:** none
- **Inputs:** `G: Geometry`
- **Outputs:** `V: Number`
- **Description:** Backed by `queries::solid_volume`.

### 11.2 `Area`
- **Fields:** none
- **Inputs:** `G: Geometry`
- **Outputs:** `A: Number`
- **Description:** Backed by `queries::solid_surface_area`. Also accept `S: Surface` for surface area.

### 11.3 `Centroid`
- **Fields:** none
- **Inputs:** `G: Geometry`
- **Outputs:** `C: Point`
- **Description:** Backed by `queries::solid_center_of_mass`.

### 11.4 `BoundingBox`
- **Fields:** `{ oriented: bool }`
- **Inputs:** `G: Geometry` (or `List<Geometry>`)
- **Outputs:** `B: Geometry` (box), `Min: Point`, `Max: Point`
- **Description:** Backed by `queries::solid_bounding_box` (axis-aligned); oriented bbox via PCA.

### 11.5 `Distance`
- **Fields:** none
- **Inputs:** `A: Point`, `B: Point`
- **Outputs:** `D: Number`
- **Description:** Already in `PointDistance` (3.14). Listed here for the Analysis category.

### 11.6 `Angle`
- **Fields:** `{ plane: Option<Plane> }`
- **Inputs:** `A: Vector`, `B: Vector`
- **Outputs:** `A: Number` (radians), `D: Number` (degrees)
- **Description:** Angle between two vectors. Reuse `Direction3d::angle_to`.

### 11.7 `MassProperties`
- **Fields:** `{ density: f64 }`
- **Inputs:** `G: Geometry`
- **Outputs:** `Volume: Number`, `Mass: Number`, `Centroid: Point`, `Inertia: List<Number>` (3 principal moments)
- **Description:** Combined volume/mass/centroid/inertia. Backed by `solid_volume`, `solid_center_of_mass`, `solid_moments_of_inertia`.

### 11.8 `MomentsOfInertia`
- **Fields:** none
- **Inputs:** `G: Geometry`
- **Outputs:** `Ixx, Iyy, Izz: Number`, `Ixy, Ixz, Iyz: Number`
- **Description:** Backed by `solid_moments_of_inertia`.

### 11.9 `CurveCurvatureAnalysis`
- **Fields:** `{ samples: u32 }`
- **Inputs:** `C: Curve3d`
- **Outputs:** `T: List<Number>` (parameters), `K: List<Number>` (curvature), `R: List<Number>` (radius)
- **Description:** Curvature comb data.

### 11.11 `SurfaceCurvatureAnalysis`
- **Fields:** `{ u_samples: u32, v_samples: u32 }`
- **Inputs:** `S: Surface`
- **Outputs:** `K1: List<Number>`, `K2: List<Number>` (principal curvatures), `G: List<Number>` (Gaussian), `M: List<Number>` (mean)
- **Description:** Surface curvature heatmap data.

### 11.12 `PointInSolid`
- Already in 9.11.

### 11.13 `PointInCurve`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `C: Curve3d` (closed), `P: Point`, `Pl: Plane` (curve's plane)
- **Outputs:** `B: Boolean`, `R: Boolean` (on boundary)
- **Description:** Point-in-polygon test (cast curve to planar polygon, use ray-cast).

### 11.14 `ClosestPointOnSurface`
- **Fields:** none
- **Inputs:** `S: Surface`, `P: Point`
- **Outputs:** `Q: Point`, `U: Number`, `V: Number`, `D: Number` (distance)
- **Description:** Closest point on surface (generalization of 6.25 with distance output).

### 11.15 `ClosestPointOnCurve`
- **Fields:** none
- **Inputs:** `C: Curve3d`, `P: Point`
- **Outputs:** `Q: Point`, `T: Number`, `D: Number`
- **Description:** Backed by `closest_point_on_curve`.

### 11.16 `SelfIntersect`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `C: Curve3d` (or `S: Surface`)
- **Outputs:** `P: List<Point>` (self-intersection points)
- **Description:** Detect self-intersections via subdivision.

### 11.17 `Curve continuity`
- Already covered by `CurveDiscontinuity` (5.18).

### 11.18 `Planar`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `C: Curve3d` (or `P: List<Point>`)
- **Outputs:** `B: Boolean`, `P: Plane` (best-fit plane)
- **Description:** Test whether a curve/point set lies in a plane.

### 11.19 `Closed`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `C: Curve3d` (or `S: Surface`)
- **Outputs:** `B: Boolean`
- **Description:** Test whether curve/surface is closed.

### 11.20 `VolumeOf` *(alias of Volume with density)*
- Already in `MassProperties` (11.7).

---

## 12. Mesh — Mesh Operations *(new category)*

The current `VpData::Mesh` exists but no nodes produce/consume it.

### 12.1 `MeshFromGeometry`
- **Fields:** `{ tolerance: f64, adaptive: bool }`
- **Inputs:** `G: Geometry`
- **Outputs:** `M: Mesh`
- **Description:** Tessellate a solid into a triangle mesh. Backed by `draper-mesh::triangulate`.

### 12.2 `MeshFromSurface`
- **Fields:** `{ u_count: u32, v_count: u32 }`
- **Inputs:** `S: Surface`
- **Outputs:** `M: Mesh`
- **Description:** Tessellate a surface into a mesh grid.

### 12.3 `MeshToNurbs`
- **Fields:** `{ degree: u32 }`
- **Inputs:** `M: Mesh`
- **Outputs:** `S: Surface` (NurbsSurface, one per quad)
- **Description:** Convert mesh quads to NURBS patches.

### 12.4 `MeshBox` / `MeshSphere` / `MeshCylinder`
- **Fields:** same as primitive solids
- **Inputs:** none (or `P: Plane`)
- **Outputs:** `M: Mesh`
- **Description:** Direct mesh primitive generators (skip Solid conversion).

### 12.5 `MeshArea`
- **Fields:** none
- **Inputs:** `M: Mesh`
- **Outputs:** `A: Number`
- **Description:** Sum of triangle areas.

### 12.6 `MeshVolume`
- **Fields:** none
- **Inputs:** `M: Mesh`
- **Outputs:** `V: Number`
- **Description:** Signed tetrahedral volume (divergence theorem).

### 12.7 `MeshFlip`
- **Fields:** none
- **Inputs:** `M: Mesh`
- **Outputs:** `M: Mesh`
- **Description:** Reverse winding (flip normals).

### 12.8 `MeshWeld`
- **Fields:** `{ tolerance: f64 }`
- **Inputs:** `M: Mesh`
- **Outputs:** `M: Mesh`
- **Description:** Weld coincident vertices.

### 12.9 `MeshQuadify`
- **Fields:** `{ angle_tol_deg: f64 }`
- **Inputs:** `M: Mesh`
- **Outputs:** `M: Mesh` (quads where possible)
- **Description:** Combine adjacent coplanar triangles into quads.

### 12.10 `MeshTriangulate`
- **Fields:** none
- **Inputs:** `M: Mesh` (mixed)
- **Outputs:** `M: Mesh` (triangles only)
- **Description:** Force triangulation via `draper-mesh::triangulate`.

### 12.11 `MeshSubdivide`
- **Fields:** `{ iterations: u32, scheme: SubdScheme }` (Catmull-Clark / Loop / Linear)
- **Inputs:** `M: Mesh`
- **Outputs:** `M: Mesh`
- **Description:** Backed by `draper-mesh::subdivision`.

### 12.12 `MeshDecimate`
- **Fields:** `{ target_ratio: f64 }`
- **Inputs:** `M: Mesh`
- **Outputs:** `M: Mesh`
- **Description:** Backed by `draper-mesh::decimate`.

### 12.13 `MeshSmooth`
- **Fields:** `{ iterations: u32, lambda: f64 }`
- **Inputs:** `M: Mesh`
- **Outputs:** `M: Mesh`
- **Description:** Laplacian smoothing.

---

## 13. Output — Bake / Export

### 13.1 `BakeToLayer` *(generalization of BakeToDoc)*
- **Fields:** `{ layer_name: String, color: Option<[f64;4]> }`
- **Inputs:** `G: Geometry`
- **Outputs:** none
- **Description:** Bake into a named layer with optional color override.

### 13.2 `BakeMesh`
- **Fields:** `{ layer_name: String }`
- **Inputs:** `M: Mesh`
- **Outputs:** none
- **Description:** Bake a mesh into the document.

### 13.3 `BakeCurve`
- **Fields:** `{ layer_name: String }`
- **Inputs:** `C: Curve3d`
- **Outputs:** none
- **Description:** Bake a curve as a wire in the document.

### 13.4 `ExportSTEP` / `ExportSTL` / `ExportOBJ`
- **Fields:** `{ path: String }`
- **Inputs:** `G: Geometry`
- **Outputs:** `S: String` (status / path written)
- **Description:** Export to file. Backed by `draper-step` and `draper-mesh::export`.

### 13.5 `Group`
- **Fields:** `{ name: String }`
- **Inputs:** `G: List<Geometry>`
- **Outputs:** `G: Geometry` (Compound / Group)
- **Description:** Group multiple solids into one named collection.

### 13.6 `Cluster` *(advanced — sub-graph reference; not full programming)*
- **Fields:** `{ graph_path: String }`
- **Inputs:** `I: List<Any>`
- **Outputs:** `O: List<Any>`
- **Description:** Wrap a sub-graph as a reusable node. (Not C#/Python — just graph reference.)

---

## 14. Summary Table — Coverage by Grasshopper Category

| Grasshopper Category | Current Count | Proposed Additions | Total | ✅ Eval Implemented |
|---------------------|---------------|--------------------|-------|----------------------|
| Params              | 6             | 7                  | 13    | 3 (of new) |
| Maths               | 15            | 12                 | 27    | 12 |
| Vector *(new)*      | 0             | 16                 | 16    | 6 (Cross/Dot/Length/Unit/Neg/Recip) |
| Sets                | 7             | 13                 | 20    | 12 (Phase H) |
| Curve               | 5             | 26                 | 31    | 18 (Arc/Ellipse/Flip/EndPoints/PointAt + 11 new + Hyperbola/Parabola Phase I) |
| Surface *(new)*     | 0             | 32                 | 32    | 24 (8 creation + 2 eval + 14 Phase E extended) |
| Primitives          | 5             | 7                  | 12    | 7 (Phase I: PlanePrimitive/PolygonPrism/Tube/Helix/Wedge/Tetra/Octa/Icosa — 8 of 7) |
| Transform           | 6             | 16                 | 22    | 12 (RotateAxis/MirrorPlane + 5 Phase F + 5 Phase I: Shear/Taper/ApplyTransform/ArrayPolar/Offset) |
| Intersect *(new)*   | 0 (3 boolean) | 15                 | 15    | 7 (BooleanSplit + 6 new) |
| Modify              | 2             | 17                 | 19    | 17 (Fillet/Chamfer + 6 + 9 Phase E extended) |
| Analysis *(new)*    | 0             | 14                 | 14    | 7 (Vol/Area/Centroid/BBox/Dist/Angle/Mass) |
| Mesh *(new)*        | 0             | 13                 | 13    | 9 (ToMesh/Area/Volume/Flip + 5 new) |
| Output              | 1             | 6                  | 7     | 8 (BakeToDoc + 7 Phase H) |
| **Total**           | **58**        | **194**            | **252**| **~137** |

---

## 15. Implementation Priority (Suggested Phasing)

### Phase A (Foundation — required before any new node) ✅ DONE
1. ✅ Extend `PortType` with `Surface`, `Curve3d`, `Plane`, `Domain`, `Transform`, `Mesh`.
2. ✅ Extend `VpData` with matching variants.
3. ✅ Extend `PortType::accepts()` and `color()` / `name()`.
4. ✅ Refactor existing `Curve(Vec<Point3d>)` to interop with new `Curve3d` variant (sample on demand).

### Phase B (Analysis & Query — high value, low complexity) ✅ DONE
- ✅ Section 11: `Volume, Area, Centroid, BoundingBox, Distance, Angle, MassProperties`
- ⏳ Section 9.10: `MeshRayIntersect`, 9.11: `SolidInclusion`, 9.12: `CollisionCheck` (stub)

### Phase C (Vector & extended Maths — completes math foundation) ✅ DONE
- ✅ Section 3: 6 of 16 vector nodes (Cross, Dot, VectorLength, Unit, Negative, Vector2pt + PointMidpoint + PointLerp)
- ✅ Section 2: All 12 math nodes

### Phase D (Surface creation — biggest CAD gap) ✅ MOSTLY DONE
- ✅ Section 6.1–6.18: `PlaneSurface`, `Extrude`, `Revolve`, `Loft`, `Sweep`, `RuledSurface`, `ExtrudePoint`, `ExtrudeTapered`
- ✅ Section 8.1–8.5: `RotateAxis, MirrorPlane, Orient, Project` (5 of 5)
- ⏳ CylinderSurface, ConeSurface, SphereSurface, TorusSurface, OffsetSurface (need surface outputs)

### Phase E (Surface evaluation & modification) ✅ DONE
- ✅ Section 6.18–6.19: `EvaluateSurface`, `SurfaceNormal`
- ✅ Section 6.20: `SurfaceFrame`, `SurfaceCurvature`, `SurfaceAreaUV`
- ✅ Section 6.23: `IsoTrim`, `DivideSurface` (6.24)
- ✅ Section 6.25: `SurfaceClosestPoint`, `SurfaceProjectPoint` (6.26)
- ✅ Section 6.27: `SurfaceSplit`, `SurfaceFlip` (6.28), `SurfaceRebuild` (6.29), `SurfaceFromPoints` (6.30), `SurfaceIsocurve` (6.31), `SurfaceUntrim` (6.32)
- ✅ Section 10: `Shell, Thicken, OffsetSolid, Hole, SplitSolid, TrimSolid` (6 modify nodes)
- ✅ Section 10 (extended): `DraftFace, MoveFace, OffsetFace, ReplaceFace, HoleCircular, Rib, FilletEdge, ChamferEdge, FilletVariable, SurfaceTrim` (10 modify nodes)
- ⏳ Remaining: cylinder/cone/sphere/torus surface variants (need surface outputs from planar inputs)

### Phase F (Curve expansion & intersections) ✅ MOSTLY DONE
- ✅ Section 5: 16 of 26 curve nodes (Arc, Ellipse, Flip, EndPoints, PointAt, Polyline, NurbsCurve, NurbsCurveInterp, JoinCurves, CurveOffset, Extend, Rebuild, Tangent, Curvature, NearestPoint, SplitCurve, Arc3pt)
- ✅ Section 9: CCX, CSX, SSX, PPX, CPX, BPX (6 intersection nodes)
- ⏳ CurveBooleanUnion/Subtract/Intersect, CurveShatter, CurveDiscontinuity, CurveFrame, CurveNormal, ProjectCurveToPlane, ProjectCurveToSurface, CurveSeam

### Phase G (Mesh & arrays) ✅ DONE
- ✅ Section 12: 9 of 13 mesh nodes (ToMesh, MeshArea, MeshVolume, MeshFlip, MeshPrimitive, MeshWeld, MeshSubdivide, MeshDecimate, MeshSmooth)
- ✅ Section 8.7–8.10: `ArrayAlongCurve, ArrayBox, ArrayOnSurface`
- ⏳ MeshFromGeometry, MeshFromSurface, MeshToNurbs, MeshQuadify, MeshTriangulate

### Phase H (Sets/Tree, Output, Cluster) ✅ DONE
- ✅ Section 4: All 12 tree operations (CullIndex, Partition, ReplaceItems, Sift, Combine, Duplicate, NullItem, PathMapper, TreeBranch, TreeStatistics, CleanTree, ExplodeTree)
- ✅ Section 13: All 8 output variants (BakeToLayer, BakeMesh, BakeCurve, ExportSTEP, ExportSTL, ExportOBJ, Group, Cluster)
- ⏳ ListMap (4.8) — skipped (sub-graph reference, requires graph execution runtime)

### Phase I (Optional / Advanced) ✅ DONE
- ✅ Section 7.7: Platonic solids (Tetra/Octa/Icosa)
- ✅ Section 5.7: Hyperbola/Parabola
- ✅ Section 8.13: Taper (uses `draft_face`)
- ✅ Section 8.12: Shear
- ✅ Section 5.10: CurveExtend ✅ DONE (implemented as `Extend`)
- ✅ Section 5.13: CurveRebuild ✅ DONE (implemented as `Rebuild`)
- ✅ Section 7: PlanePrimitive, PolygonPrism, Tube, Helix, Wedge
- ✅ Section 8.10: ArrayPolar
- ✅ Section 8.11: Offset
- ✅ Section 8.16: ApplyTransform

---

## 16. Notes & Constraints

1. **No external solver dependency** — every node above is implementable with current `draper-*` crates.
2. **No display-only nodes** — visualization is handled by the viewport, not VP.
3. **No programming nodes** — Python/C#/VB are excluded per spec. The `Cluster` (13.6) and `ListMap` (4.8) only reference other VP sub-graphs, not arbitrary code.
4. **`Surface` PortType is essential** — without it, half the proposed nodes (Loft, Sweep, Offset Surface, Evaluate Surface, etc.) cannot exist as first-class data.
5. **`Curve3d` PortType is essential** — the current `Curve(Vec<Point3d>)` only supports sampled polylines and loses arc/NURBS type information needed by `ProjectCurveToSurface`, `SurfaceIsocurve`, etc.
6. **Backward compatibility** — existing nodes keep their signatures. New types are added in parallel; `accepts()` rules let `Curve3d` and `Curve` interop (auto-sample).
7. **Phase A is a hard prerequisite** — implementing any new node before extending `PortType` / `VpData` will require painful refactoring later.

---

**End of plan.** Total: 194 new node variants across 13 categories.
