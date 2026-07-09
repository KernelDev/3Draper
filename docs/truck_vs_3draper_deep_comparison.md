# truck vs 3Draper — Глубокое сравнительное исследование

**Дата:** 2026-06-20
**Метод:** полный обход `src/*.rs` каждого крейта в `/tmp/truck/` и `/home/z/my-project/crates/`.
**Цель:** найти, что есть в truck, но отсутствует или слабее в 3Draper — особенно по STEP, поверхностям и кривым.

---

## TL;DR — 5 главных находок

1. **3Draper сильнее в pipeline/экспорте/PMI/AM-сертификации**, но **значительно слабее в геометрическом ядре**. truck имеет аналитический NURBS-тулкит (knot insertion, degree elevation, Bézier decomposition, quadratic/cubic approximation, `make_locally_injective`), а 3Draper — только de Boor evaluation + численные производные.

2. **3Draper теряет STEP-данные при парсинге**: `HYPERBOLA`, `PARABOLA`, внутренние оболочки `BREP_WITH_VOIDS`, `VERTEX_LOOP` — всё silent drop. truck парсит HYPERBOLA, PARABOLA, BREP_WITH_VOIDS (хотя voids не доходит до CompressedSolid — известная ограниченность).

3. **3Draper использует hand-rolled string parser без schema-validation**. truck использует `ruststep v0.4.0` — полноценный EXPRESS-схема-валидирующий парсер.

4. **3Draper не имеет `PCurve` как аналитической кривой-на-поверхности** (только polyline-приближение), не имеет `IntersectionCurve` как first-class типа (только дискретные полилинии в boolean.rs), не имеет `ToSameGeometry`/`IncludeCurve`/`Concat`/`Cut`/`Processor` trait'ов — всё это базис truck-геометрии.

5. **3Draper теряет аналитические производные** для Revolution/Extrusion (central differences) и для NURBS-кривых на dispatch-уровне (`Curve3d::derivative_at` для NURBS падает в numerical). truck имеет full analytical derivatives через chain-rule + rational-derivative machinery (`rat_der`, `rat_ders`).

---

## 1. КРИВЫЕ — что есть в truck, но нет в 3Draper

### 1.1 Отсутствующие типы кривых

| Кривая | truck | 3Draper | Влияние |
|---|---|---|---|
| `Line` | ✅ | ✅ | — |
| `Circle` / `UnitCircle` | ✅ | ✅ | — |
| `Ellipse` | ✅ (через `Processor<TrimmedCurve<UnitCircle>, M>`) | ✅ (нативный) | — |
| **`Hyperbola`** | ✅ `UnitHyperbola` (нативный, analytical quartic solver) | ❌ **silently dropped в STEP parser** | STEP-файлы с гиперболами теряют edge geometry |
| **`Parabola`** | ✅ `UnitParabola` (нативный) | ❌ **silently dropped в STEP parser** | STEP-файлы с параболами теряют edge geometry |
| `BSplineCurve` | ✅ full toolkit | ✅ eval + derivative only |详见 1.2 |
| `NurbsCurve` | ✅ | ✅ eval + analytical derivative | — |
| `BezierCurve` | ✅ (через `KnotVec::bezier_knot`) | ⚠️ только как subtype of B_SPLINE_CURVE | нет first-class Bezier API |
| `PolylineCurve` | ✅ first-class (используется как leader IntersectionCurve) | ✅ как Curve3d::Nurbs (degree 1) | — |
| `TrimmedCurve<C>` | ✅ generic decorator | ⚠️ только Circle→Arc; для остальных — untrimmed basis | TRIMMED_CURVE ellipses/NURBS теряют trim |
| **`PCurve<C, S>`** | ✅ **analytical chain-rule derivatives** (SurfaceDers::composite_ders) | ❌ только polyline approximation в topology | теряется точность surface-on-surface |
| **`IntersectionCurve<C, S0, S1>`** | ✅ first-class с 4D Newton (`double_projection`) | ❌ только дискретные полилинии в `boolean.rs` | нет аналитической SSI-кривой |
| `Offset<C, N>` | ✅ generic vector-field offset | ⚠️ только OFFSET_CURVE_3D sampled→NURBS | — |
| `NormalField<C, F>` | ✅ normal-direction offset | ❌ | — |
| `RbfContactCurve` | ✅ orbit of rolling-ball fillet contact | ❌ | — |

### 1.2 NURBS-тулкит — gap критический

| Операция | truck (`BSplineCurve`) | 3Draper (`NurbsCurve`) |
|---|---|---|
| `point_at` (de Boor) | ✅ | ✅ |
| `derivative_at` | ✅ analytical quotient rule | ⚠️ analytical в `NurbsCurve::derivative_at`, **но `Curve3d::derivative_at` fallback на central differences** (curve.rs:423-449) |
| `der_n` (n-th order, до 10) | ✅ `CurveDers<V>` | ❌ |
| `rats_ders` (rational derivatives) | ✅ `multi_rat_ders` | ❌ |
| **`add_knot(x)`** (Boehm insertion) | ✅ | ❌ |
| **`remove_knot` / `try_remove_knot`** (Tiller) | ✅ | ❌ |
| **`elevate_degree()`** (Prautzsch) | ✅ | ❌ |
| **`clamp()`** | ✅ | ❌ |
| **`optimize()`** (redundant knot removal) | ✅ | ❌ |
| **`syncro_degree(other)` / `syncro_knots(other)`** | ✅ | ❌ |
| **`bezier_decomposition()` → Vec<BSplineCurve>** | ✅ | ❌ |
| **`make_locally_injective()`** | ✅ | ❌ |
| **`quadratic_approximation<C>`** (least-squares fit) | ✅ | ❌ |
| **`cubic_approximation<C>`** | ✅ | ❌ |
| **`try_interpole(points)`** | ✅ | ❌ |
| **`cut(t)`** (split at parameter) | ✅ | ❌ |
| **`is_arc_of(curve, hint)`** | ✅ | ❌ |
| `roughly_bounding_box` | ✅ | ❌ |
| `near_as_curve` | ✅ | ❌ |
| **`SearchParameter<SPHint1D>`** (Newton in 1D) | ✅ generic | ⚠️ только `closest_point_on_curve` (numerical) |
| **`SearchNearestParameter<SPHint1D>`** | ✅ generic | ❌ |

### 1.3 Trait-уровень абстракции кривых

| Trait | truck | 3Draper |
|---|---|---|
| `ParametricCurve` | ✅ | ❌ (только enum match) |
| `BoundedCurve` | ✅ | ❌ |
| `ParameterDivision1D` (recursive bisection с hash-perturbed midpoint) | ✅ | ⚠️ в P0 заимствован только 2D-вариант |
| **`Concat<Rhs>`** (join two BoundedCurves) | ✅ + `CurveCollector<C>` | ❌ |
| **`Cut`** (split at parameter) | ✅ | ❌ |
| **`Invertible`** | ✅ | ⚠️ только `.reversed()` метод |
| **`Transformed<T>`** | ✅ | ⚠️ только `.transform()` метод |
| **`ToSameGeometry<T>`** (конверсия представлений) | ✅ | ❌ |
| **`IncludeCurve<C>`** (verify curve on surface) | ✅ — precondition для triangulation | ❌ |
| **`SearchParameter<Dim>`** / **`SearchNearestParameter<Dim>`** | ✅ generic over D1/D2 | ❌ |
| **`ParameterTransform`** (affine reparameterization) | ✅ | ❌ |

---

## 2. ПОВЕРХНОСТИ — что есть в truck, но нет в 3Draper

### 2.1 Отсутствующие типы поверхностей

| Поверхность | truck | 3Draper | Влияние |
|---|---|---|---|
| `Plane` | ✅ | ✅ | — |
| `Sphere` | ✅ + STEP lat/long swap | ✅ | 3Draper может иметь STEP interop bug — нужно проверить |
| `Torus` | ✅ | ✅ | — |
| `Cylinder` (нативный) | ⚠️ через `RevolutedCurve<Line<Point3>>` | ✅ native | 3Draper сильнее |
| `Cone` (нативный) | ⚠️ через `RevolutedCurve<Line<Point3>>` | ✅ native | 3Draper сильнее |
| `BSplineSurface<P>` | ✅ full toolkit | ✅ eval + analytical derivative |详见 2.2 |
| `NurbsSurface<V>` | ✅ | ✅ eval + analytical derivative | — |
| `RevolutedCurve<C>` | ✅ **analytical derivatives** (chain rule) | ⚠️ **numerical central differences** (surface.rs:1292-1301) | точность, скорость |
| `ExtrudedCurve<C, V>` | ✅ **analytical** | ⚠️ **numerical** | точность, скорость |
| `HomotopySurface<C0, C1>` | ✅ linear interpolation | ❌ | нет skinning между двумя кривыми |
| `Processor<E, T>` (transform wrapper) | ✅ — **не переписывает control points** | ❌ — transform мутирует control points | производительность |
| **`EdgeBlendSurface`** (cubic Bezier cross-section blend) | ✅ | ❌ | нет smooth blend face |
| **`RbfSurface`** (rolling-ball fillet surface) | ✅ | ❌ | нет точного fillet-face |
| **`ApproxFilletSurface`** (NURBS approximation of fillet) | ✅ | ❌ | — |
| **`Offset<S, N>`** | ✅ generic vector-field offset | ⚠️ только OFFSET_SURFACE sampled→NURBS | потеря исходной semantics |

### 2.2 NURBS-тулкит поверхностей — gap критический

| Операция | truck (`BSplineSurface`) | 3Draper (`NurbsSurface`) |
|---|---|---|
| `point_at` (de Boor tensor) | ✅ | ✅ |
| `derivatives_at` (analytical quotient rule) | ✅ | ✅ + numerical fallback |
| `der_mn` (до 10-го порядка) | ✅ `SurfaceDers<V>` | ❌ |
| `normal_at` | ✅ analytical (через `der_mn(1,1)`) | ✅ via `derivatives_at().normal()` |
| `curvature_at` | ✅ analytical (через 2nd fundamental form) | ⚠️ **9-point numerical stencil** (surface.rs:1397-1476) | медленно, менее точно |
| **`add_uknot` / `add_vknot`** | ✅ | ❌ |
| **`try_remove_uknot` / `try_remove_vknot`** | ✅ | ❌ |
| **`elevate_udegree` / `elevate_vdegree`** | ✅ | ❌ |
| **`syncro_uvdegrees` / `syncro_uvknots`** | ✅ | ❌ |
| **`ucut(u)` / `vcut(v)`** (split at param) | ✅ | ❌ |
| **`sectional_curve(bounding_box)`** | ✅ | ❌ |
| **`homotopy(curve0, curve1)`** (skin two curves) | ✅ | ❌ |
| **`by_boundary(curves: [C; 4])`** (Coons patch) | ✅ | ❌ |
| **`swap_axes`** | ✅ | ❌ |
| **`column_curve(row_idx)` / `row_curve(col_idx)`** | ✅ iso-curve extraction | ❌ |
| `IncludeCurve<BSplineCurve>` (verify curve lies on surface) | ✅ — использует knot insertion | ❌ |
| `SearchParameter<SPHint2D>` | ✅ generic Newton 2D | ⚠️ NURBS-специфичный Newton 11×11+5×5+15iter |
| `SearchNearestParameter<SPHint2D>` | ✅ generic Newton 3D (SsnpVector trick) | ❌ |

### 2.3 Trait-уровень поверхностей

| Trait | truck | 3Draper |
|---|---|---|
| `ParametricSurface` | ✅ | ❌ (enum match) |
| `BoundedSurface` | ✅ | ❌ |
| `ParameterDivision2D` (adaptive bisection с hash-perturbed midpoint) | ✅ original | ✅ **заимствован в P0** |
| `SurfaceParameterDivision` | ✅ | ❌ |
| `ParametricSurface3D` (with `normal`, `normal_uder`, `normal_vder`) | ✅ default impls | ❌ |
| **`IncludeCurve<C>`** | ✅ | ❌ |
| **`ToSameGeometry<T>`** | ✅ (Plane→BSplineSurface и т.д.) | ❌ |

---

## 3. STEP I/O — что есть в truck, но нет в 3Draper

### 3.1 Архитектура парсера

| Аспект | truck | 3Draper |
|---|---|---|
| STEP-парсер | ✅ `ruststep v0.4.0` (внешний crate, schema-aware) | ❌ hand-rolled streaming text parser |
| Schema validation | ✅ через ruststep | ❌ нет |
| Типизированные Holder'ы | ✅ один Holder struct на каждый entity | ❌ generic `StepEntity {id, type_name, params, sub_entities}` |
| Case-sensitivity | ✅ соответствует spec (uppercase) | ⚠️ требует uppercase (практически OK, но не по spec) |
| Complex entity parsing | ✅ через ruststep | ✅ ручной код |
| Multi-line string literals | ✅ | ✅ |
| Block comments | ✅ | ✅ |
| Performance | ✅ streaming + zero-copy где возможно | ✅ 8MB BufReader, lazy indices |

### 3.2 STEP entity coverage

| Entity | truck parse | 3Draper parse | Notes |
|---|---|---|---|
| `CARTESIAN_POINT`, `DIRECTION`, `VECTOR` | ✅ | ✅ | — |
| `AXIS1_PLACEMENT`, `AXIS2_PLACEMENT_2D/3D` | ✅ | ✅ | — |
| `LINE`, `POLYLINE` | ✅ | ✅ | — |
| **`CIRCLE`** | ✅ | ✅ | — |
| **`ELLIPSE`** | ✅ | ✅ | — |
| **`HYPERBOLA`** | ✅ → `UnitHyperbola` | ❌ **silent drop** | edge geometry теряется |
| **`PARABOLA`** | ✅ → `UnitParabola` | ❌ **silent drop** | edge geometry теряется |
| `B_SPLINE_CURVE_WITH_KNOTS`, `BEZIER_CURVE`, `UNIFORM_CURVE`, `QUASI_UNIFORM_CURVE` | ✅ | ✅ | — |
| `RATIONAL_B_SPLINE_CURVE` | ✅ | ✅ | — |
| `PCURVE`, `SURFACE_CURVE`, `SEAM_CURVE` | ✅ | ✅ | — |
| `PLANE`, `SPHERICAL_SURFACE`, `CYLINDRICAL_SURFACE`, `TOROIDAL_SURFACE`, `CONICAL_SURFACE` | ✅ | ✅ | — |
| `SURFACE_OF_LINEAR_EXTRUSION`, `SURFACE_OF_REVOLUTION` | ✅ | ✅ | — |
| `B_SPLINE_SURFACE_WITH_KNOTS`, `BEZIER_SURFACE`, `UNIFORM_SURFACE`, `QUASI_UNIFORM_SURFACE` | ✅ | ✅ | — |
| `RATIONAL_B_SPLINE_SURFACE` | ✅ | ✅ | — |
| **`RECTANGULAR_TRIMMED_SURFACE`** | ⚠️ неявно | ✅ recursive | — |
| **`OFFSET_SURFACE`** | ❌ | ⚠️ approximated as NURBS | 3Draper сильнее |
| **`OFFSET_CURVE_3D`** | ❌ | ⚠️ approximated | 3Draper сильнее |
| **`CURVE_BOUNDED_SURFACE`** | ❌ | ❌ | — |
| **`MANIFOLD_SOLID_BREP`** | ✅ | ✅ | — |
| **`FACETED_BREP`** | ❌ | ✅ | 3Draper сильнее |
| **`BREP_WITH_VOIDS`** | ⚠️ voids хранятся, но **не propagируются** в `CompressedSolid` | ❌ **voids silently dropped** — `find_shell_ref` возвращает только первую shell | оба имеют gap, 3Draper хуже |
| **`CLOSED_SHELL`**, `OPEN_SHELL`, `ORIENTED_*_SHELL`, `SHELL_BASED_SURFACE_MODEL` | ✅ | ✅ | — |
| **`ADVANCED_FACE`**, `FACE_SURFACE`, `FACE_BOUND`, `FACE_OUTER_BOUND` | ✅ | ✅ | — |
| **`EDGE_LOOP`**, `ORIENTED_EDGE`, `EDGE_CURVE`, `VERTEX_POINT` | ✅ | ✅ | — |
| **`VERTEX_LOOP`** (для degenerate faces — sphere poles, cone apex) | ❌ (writer emits, parser doesn't accept) | ❌ **не handled в `resolve_face_bound_with_step_ids`** | оба имеют gap; 3Draper теряет boundary целиком |
| Assembly: `NAUO`, `ITEM_DEFINED_TRANSFORMATION`, `CDSR`, `SHAPE_REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` | ✅ | ✅ | — |
| PMI / GD&T (14 ASME Y14.5 tolerance types, DATUM_FEATURE, etc.) | ❌ | ✅ **полная extracción** | **3Draper кратно сильнее** |
| AP242 `TESSELLATED_SHAPE_REPRESENTATION`, `TRIANGULATED_FACE` | ❌ | ✅ | **3Draper сильнее** |
| Color / layer / units | ❌ | ✅ | **3Draper сильнее** |

### 3.3 STEP export

| Аспект | truck | 3Draper |
|---|---|---|
| `LINE` корректность | ✅ | ❌ **баг: не пишет reference point** (exporter.rs:200-205) |
| `NURBS` curves export | ✅ как `B_SPLINE_CURVE_WITH_KNOTS` + `RATIONAL_B_SPLINE_CURVE` | ❌ **падает в placeholder Plane** (exporter.rs:396-417) |
| `NURBS` surfaces export | ✅ | ❌ placeholder Plane |
| `Revolution`, `Extrusion` surfaces export | ✅ | ❌ placeholder Plane |
| `BREP_WITH_VOIDS` export | ✅ (когда >1 shell) | ❌ только outer shell, inner shells игнорируются |
| Assembly structure export | ✅ | ❌ |
| PMI / GD&T / color / units export | ❌ | ❌ |

---

## 4. TOPOLOGY — сравнение

### 4.1 B-rep структура

| Элемент | truck | 3Draper |
|---|---|---|
| `Vertex<P>` | ✅ `Arc<Mutex<P>>` + `VertexID` (pointer-based ID) | ✅ `Vertex {id: TopoId, point, tolerance}` (atomic counter) |
| `Edge<P, C>` | ✅ `(vertices, orientation, curve: Arc<Mutex<C>>)` + `EdgeID` orientation-independent | ✅ `Edge {id, curve, param_range, vertex_start/end, forward, step_entity_id}` — **`step_entity_id` для bit-exact shared-edge dedup** (3Draper сильнее для STEP round-trip) |
| `Wire` | ✅ `VecDeque<Edge>` + `is_continuous`/`is_cyclic`/`is_closed`/`is_simple` | ✅ `Wire {coedges: Vec<CoEdge>, closed}` — **CoEdge layer** (3Draper сильнее — имеет pcurve per coedge) |
| `Face` | ✅ `boundaries: Vec<Wire>`, `orientation`, `surface` | ✅ `Face {surface, outer_wire, inner_wires, forward, edges}` |
| `Shell` | ✅ + `shell_condition()` enum (Irregular/Regular/Oriented/Closed) | ✅ `Shell {faces, closed}` |
| `Solid` | ✅ outer + voids | ✅ outer + inner_shells |
| `Compound` | ❌ | ✅ recursive (Solids+Compounds) |
| **`CompressedShell`** | ✅ — indexed arrays (dedup vertices/edges/faces), lingua franca для STEP/healing | ❌ |
| **`ShapeStorage`** | ❌ | ⚠️ есть, underused |

### 4.2 Boolean operations

| Op | truck | 3Draper |
|---|---|---|
| **AND (intersection)** | ✅ transversal only (тangencies не поддержаны); mesh-mesh interference → polyline → `IntersectionCurve` → face splitting → classification | ✅ **`draper-topology::boolean::boolean_intersect`** — аналитика для plane/plane, plane/cylinder, plane/sphere, plane/cone, cylinder/cylinder, cylinder/sphere; general subdivision для остальных |
| **OR (union)** | ✅ | ✅ `boolean_union` |
| **SUB (subtract)** | ❌ (но через `and(solid0, not(solid1))`) | ✅ `boolean_subtract` (нативно) |
| Tangency handling | ❌ | ⚠️ partial (через general subdivision) |
| Robustness approach | mesh-guided (tessellate → collide → reconstruct) | analytic-first с subdivision fallback |

**Verdict:** 3Draper сильнее в analytic cases (6 пар поверхностей закрыто формулами), truck сильнее в robustness (mesh-guided всегда находит пересечение, даже если analytic формулы нет).

### 4.3 Euler operations

| Op | truck | 3Draper |
|---|---|---|
| `Face::cut_by_edge` / `cut_by_wire` | ✅ | ⚠️ partial (`split_face` в boolean.rs) |
| `Face::glue_at_boundaries` | ✅ | ❌ |
| `Shell::cut_edge` | ✅ | ❌ |
| `Shell::remove_vertex_by_concat_edges` | ✅ | ❌ |
| `Shell::connected_components` | ✅ | ❌ |
| `Shell::singular_vertices` (manifold check) | ✅ | ❌ |
| `Shell::face_adjacency` (common-edge counts) | ✅ | ❌ |

### 4.4 Healing

| Op | truck | 3Draper |
|---|---|---|
| `split_closed_edges` | ✅ (только для cylinders) | ❌ |
| `split_closed_faces` | ✅ | ❌ |
| `RobustSplitClosedEdgesAndFaces` | ✅ | ❌ |
| Healing pipeline (gaps, holes, stitching, normals, small features, slivers, face merging, self-intersections, tolerance propagation) | ❌ | ✅ **`healing.rs` (3144 LOC, 10 операций, 3 preset)** — 3Draper кратно сильнее |

---

## 5. MESH — сравнение

### 5.1 Pipeline

| Этап | truck | 3Draper |
|---|---|---|
| Edge discretization | ✅ `PolylineCurve::from_curve` (ParameterDivision1D) | ✅ `EdgeDiscretizationCache` с **48-bit deterministic rounding** (3Draper сильнее для bit-exact dedup) |
| Boundary → UV projection | ✅ `by_search_parameter` (Newton) или `by_search_nearest_parameter` (fallback) | ✅ chain Newton-Raphson + brute-force fallback |
| **`PolyBoundary` seam handling** | ✅ **`PolyBoundaryPiece::try_new`**: проверяет `u_period`/`v_period`, shift'ит UV на integer multiples of period, **detect pole degeneracy** (когда `uder` near zero — sphere north pole), **inserts intermediate points** для избежания jagged UV | ⚠️ есть seam handling, но **менее robust** — нет pole-degenerate special case |
| UV triangulation | ✅ `spade::ConstrainedDelaunayTriangulation` | ✅ `spade` CDT + `earcutr` (двойная реализация) |
| Watertightness producer | ✅ `OptimizingFilter::put_together_same_attrs(tol)` | ✅ `ConsistentEdgeCache` + `UnifiedVertexPool` + iterative stitch/non-manifold resolve |
| Mesh repair / hole filling | ❌ | ✅ **`healing.rs`** |
| Watertightness check | ✅ `Topology::shell_condition() == Closed` | ✅ `validate_watertight` + `check_manifold` (3Draper подробнее — count per face) |

### 5.2 Mesh algorithms

| Algo | truck | 3Draper |
|---|---|---|
| **Loop subdivision** (extended, triangle meshes) | ✅ | ❌ |
| **Quadrangulation** (pair triangles into quads, coplanarity check) | ✅ | ❌ |
| **Triangle→n-gon** structuring | ✅ | ❌ |
| **`remove_degenerate_faces`** | ✅ | ✅ |
| **`remove_unused_attrs`** | ✅ | ✅ `compact_vertices` |
| **Smooth normals with crease detection** (`add_smooth_normals(tol_ang, overwrite)`) | ✅ clusters adjacent face normals по углу | ✅ `smooth_normals` + `smooth_normals_adaptive` (per-surface-type adaptive crease angle — 3Draper умнее) |
| **`make_face_orientation_consistent_with_normals`** | ✅ | ✅ |
| **Mesh-mesh collision (sweep-and-prune)** | ✅ `Collision::extract_interference` — O(n log n) | ⚠️ BVH-based (тоже O(n log n), но другой подход) |
| **Point-in-domain (ray-casting, signed crossing count)** | ✅ `IncludingPointInDomain::signed_crossing_faces` | ✅ `point_in_solid` |
| **`HashedPointCloud`** (uniform 3D grid for point-mesh distance) | ✅ | ❌ |
| **`is_clung_to_by` / `neighborhood_include` / `collide_with_neighborhood_of`** | ✅ | ❌ |
| **Volume** (divergence theorem, multi-shell) | ✅ | ✅ |
| **Center of gravity** | ✅ | ✅ |
| **Gaussian curvature clustering** | ✅ `ExperimentalSplitters::clustering_faces_by_gcurvature` | ❌ |
| **BVH with SAH** | ❌ (sweep-and-prune only) | ✅ **`bvh.rs` (1851 LOC, Median + SAH)** — 3Draper кратно сильнее |
| **Incremental BVH** (streaming/progressive) | ❌ | ✅ `IncrementalBvh` |
| **Per-face BVH** (selective re-triangulation) | ❌ | ✅ `FaceBvh` |
| **Ray pick / frustum cull / occlusion cull** | ❌ | ✅ в `wasm_api.rs` |
| **Loop subdivision** | ✅ | ❌ |
| **OBJ / STL I/O** | ✅ | ✅ + **glTF/GLB, USDA, 3MF** (3Draper кратно сильнее по экспорту) |
| **VTK output** | ✅ (через `vtkio`) | ❌ |

### 5.3 3Draper-специфичные strong features

| Feature | 3Draper | truck |
|---|---|---|
| **AM certification** (wall thickness, self-intersections, mesh quality, additive manufacturing) | ✅ `certification.rs` (1941 LOC) | ❌ |
| **PMI display** (leader lines, dimension lines, arrows, 3D text) | ✅ `pmi_display.rs` + `text3d.rs` | ❌ |
| **GD&T checker** (mesh vs STEP-extracted GD&T, 14 ASME Y14.5 types) | ✅ `gdt_check.rs` | ❌ |
| **Defect classifier** (9 defect types, rule-based heuristic DT) | ✅ `draper-ai/classifier.rs` | ❌ |
| **Healing strategy selector** (predictive) | ✅ `draper-ai/strategy.rs` | ❌ |
| **WASM chunked triangulation** (8ms frame budget) | ✅ `ChunkedBrepTriangulator` | ❌ (есть `truck-js`, но без chunked budget) |
| **Triangulation guard** (5 sec per-face timeout) | ✅ | ❌ |
| **Cloud streaming / incremental loading / collaborative editing (OT)** | ✅ `draper-cloud` (4 модуля, ~2000 LOC) | ❌ |
| **Edge `step_entity_id` для bit-exact shared-edge dedup через STEP round-trip** | ✅ | ❌ |
| **48-bit deterministic FP rounding** для shared-edge vertex dedup | ✅ | ❌ |

---

## 6. SUMMARY — что критично перенять из truck

### Tier 1 — КРИТИЧНО (теряем данные STEP-файлов)

1. **`HYPERBOLA` + `PARABOLA` curve types** + STEP parser handlers. Сейчас silently dropped.
2. **`BREP_WITH_VOIDS` inner shell extraction**. Сейчас теряем voids целиком.
3. **`VERTEX_LOOP` handling** в STEP input. Сейчас degenerate faces (sphere poles, cone apex) получают empty boundary.
4. **TRIMMED_CURVE proper handling** для ellipses/NURBS (не только для Circle).

### Tier 2 — ВАЖНО (геометрическое ядро слабее)

5. **NURBS toolkit: knot insertion, degree elevation, Bézier decomposition, optimize, syncro** — фундамент для будущего NURBS-work.
6. **Analytical derivatives для Revolution/Extrusion** (chain rule вместо central differences).
7. **NURBS surface analytical curvature** (вместо 9-point numerical stencil).
8. **`PCurve` как аналитическая curve-on-surface** (chain-rule derivatives через `SurfaceDers::composite_ders`).
9. **`IntersectionCurve<C, S0, S1>` как first-class тип** с 4D Newton `double_projection` — даёт аналитическую SSI-кривую вместо дискретной полилинии.
10. **`PolyBoundary` pole-degenerate handling** для sphere/torus (когда `uder` near zero).

### Tier 3 — ПОЛЕЗНО (code quality / API)

11. **`ToSameGeometry` trait** — конверсия Line→BSplineCurve, Plane→BSplineSurface, Circle→NurbsCurve для unified NURBS-algorithms.
12. **`IncludeCurve<C>` trait** — precondition-check для robust triangulation.
13. **`Concat<Rhs>` + `CurveCollector<C>`** для joining curves.
14. **`Cut` trait** для splitting curves/surfaces at parameter.
15. **`Processor<E, T>` transform wrapper** — не переписывает control points при transform.
16. **`ParameterDivision1D`** (1D adaptive bisection с hash-perturbed midpoint) — сейчас в P0 заимствован только 2D.
17. **Loop subdivision + quadrangulation** mesh algorithms.
18. **`ApproxFilletSurface`** + **`RbfSurface`** для точных fillet-face.
19. **`HomotopySurface`** (linear interpolation между двумя кривыми) + **`BSplineSurface::by_boundary([C; 4])`** (Coons patch) — skinning.

### Tier 4 — НИЗКИЙ ПРИОРИТЕТ (большой refactor, малая выгода)

20. **Migration на `ruststep` crate** вместо hand-rolled parser — schema validation, меньше maintenance, но 20+ часов работы.
21. **`SearchParameter<Dim>` / `SearchNearestParameter<Dim>` generic trait system** — красивая абстракция, но massive refactor.
22. **Derive macros (truck-derivers)** — auto-generate trait impls для enums/structs.
23. **`CompressedShell`** indexed-array representation — lingua franca для STEP/healing.
24. **Mesh-mesh collision via sweep-and-prune** (вместо BVH) — другой подход, не обязательно лучше.

---

## 7. ОБНОВЛЁННЫЙ ПРИОРИТИЗИРОВАННЫЙ ПЛАН P2–P10

### P2 (быстрый win, ~2 часа) — `step_id_aliases` midpoint comparison
*Из оригинального плана. Сравнение 5 midpoint'ов вдоль кривой для более точного aliasing.*
**Ожидаемая выгода:** transmission_top 5.0%→~2%, Zentralstaender 5.0%→~3% boundary edges.

### P3 (~3-4 часа) — HYPERBOLA + PARABOLA + ARC handlers в STEP parser
**Files:** `crates/draper-step/src/converter.rs`, `crates/draper-geometry/src/curve.rs`
1. Добавить `Hyperbola` и `Parabola` варианты в `Curve3d` enum.
2. Реализовать `point_at`, `derivative_at`, `param_range`, `is_degenerate`, `transform` — аналитически.
3. Добавить match-arm'ы в `resolve_curve`: `HYPERBOLA` → `Curve3d::Hyperbola`, `PARABOLA` → `Curve3d::Parabola`, `ARC` (если встречается) → `Curve3d::Arc`.
4. Triangulation fallback: так как эти кривые встречаются редко, можно обработать через generic `triangulate_generic_surface` + edge discretization cache.
5. Тест на NIST STEP suite + ABC dataset (если есть).

### P4 (~4-6 часов) — BREP_WITH_VOIDS + VERTEX_LOOP в STEP parser
**Files:** `crates/draper-step/src/converter.rs`, `crates/draper-topology/src/entity.rs`
1. Расширить `find_shell_ref` для извлечения ВСЕХ shell-refs из BREP_WITH_VOIDS.
2. Первая shell → `outer_shell`, остальные → `inner_shells` (voids).
3. В `resolve_face_bound_with_step_ids` добавить match-arm для `VERTEX_LOOP`: создавать Wire с одной CoEdge, degenerate=true, и vertex повторяется.
4. Тест на STEP-файлах с internal cavities ( compressor, transmission_top могут иметь voids).

### P5 (~6-8 часов) — NURBS toolkit: knot insertion + degree elevation
**Files:** новый `crates/draper-geometry/src/nurbs_tools.rs`
1. `insert_knot(curve, u, times)` — Boehm insertion algorithm.
2. `insert_knot_surface(surface, u_or_v, param, times)`.
3. `remove_knot(curve, idx, times, tol)` — Tiller removal.
4. `elevate_degree(curve, n)` — Prautzsch algorithm.
5. `clamp(curve)`, `optimize(curve)`.
6. `bezier_decomposition(curve) -> Vec<NurbsCurve>`.
7. Unit-тесты на каждом шаге — сравнить с truck-эталонами.
**Применение:** потом использовать в `triangulate_nurbs_cdt` для адаптивного refinement, в `intersect_surfaces_general` для subdivision-based SSI.

### P6 (~4-6 часов) — Analytical derivatives для Revolution + Extrusion
**Files:** `crates/draper-geometry/src/surface.rs`
1. Заменить numerical central differences в `RevolutionSurface::normal_at` и `ExtrusionSurface::normal_at` на analytical chain-rule.
2. Для Revolution: `S(u,v) = O + cos(u)·(P(v)−O)_perp + sin(u)·(A × (P(v)−O)_perp)` — partial derivatives explicit.
3. Для Extrusion: `S(u,v) = P(u) + v·D` — u-derivatives от profile, v-derivatives = D.
4. Тест: сравнить analytical vs numerical для нескольких surfaces — должны совпадать в пределах 1e-10.

### P7 (~8-10 часов) — PolyBoundary pole-degenerate handling
**Files:** `crates/draper-mesh/src/parametric_domain.rs`, `crates/draper-mesh/src/triangulate.rs`
1. В функции проекции boundary 3D→UV для sphere/torus: если `uder(u,v)` near zero (sphere pole), вставить intermediate UV point `(u, v0)` затем `(u, v)` для избежания jagged polygon.
2. Аналогично для `vder` near zero.
3. Тест на NIST sphere + torus STEP files — должны остаться watertight.

### P8 (~10-15 часов) — PCurve как аналитическая curve-on-surface
**Files:** `crates/draper-geometry/src/curve.rs` (новый вариант), `crates/draper-geometry/src/surface.rs`
1. Добавить `Curve3d::PCurve { curve_2d: Curve2d, surface: Surface }`.
2. Реализовать `point_at(t) = surface.point_at(curve_2d.point_at(t).u, curve_2d.point_at(t).v)`.
3. Derivatives через chain rule: `S_u·C'_u + S_v·C'_v` для 1st order; full 2nd order через `SurfaceDers`-style composite.
4. Использовать в topology `CoEdge::curve_2d` — сейчас там polyline approximation.
5. Тест: точность projection pcurve на surface должна быть < 1e-10.

### P9 (~15-20 часов) — IntersectionCurve first-class
**Files:** новый `crates/draper-geometry/src/intersection_curve.rs`
1. `IntersectionCurve<C, S0, S1>` где `C: ParametricCurve<Point=Point3>` (leader).
2. `point_at(t) = leader.point_at(t)` (3D точка).
3. `derivative_at(t)`: решить 3×3 систему `(n0, n1, leader'(t))ᵀ · c'(t) = (n0·Σ, n1·Σ, leader'(t)·Σ)`.
4. `search_triple(t, trials)`: 4D Newton `double_projection` над `(u0, v0, u1, v1)` с constraint `S0−S1 ∥ leader'(t)`.
5. Использовать в `draper-topology::boolean::intersect_surfaces` для general case — теперь возвращает `IntersectionCurve` вместо дискретной полилинии.
6. Затем boolean ops могут точно резать face по этой кривой.

### P10 (~5-8 часов) — Mesh algorithms: Loop subdivision + quadrangulation
**Files:** новый `crates/draper-mesh/src/subdivision.rs`
1. `loop_subdivision(mesh)` — extended Loop mask (1/8·sum_of_neighbors for edge verts, beta-weighted average for interior), split each triangle into 4.
2. `quadrangulate(mesh, plane_tol, score_tol)` — pair triangles into quads greedily by coplanarity + score.
3. Тесты на octahedron + icosahedron.

### P11 (~2-4 часа) — TRIMMED_CURVE proper handling for ellipses/NURBS
**Files:** `crates/draper-step/src/converter.rs`, `crates/draper-geometry/src/curve.rs`
1. Добавить `Curve3d::Trimmed { basis: Box<Curve3d>, start: f64, end: f64 }` variant.
2. `point_at(t) = basis.point_at(start + t·(end−start))`.
3. Derivatives через chain rule с scaling factor `(end−start)`.
4. В `resolve_trimmed_curve` возвращать `Trimmed` для любых basis curves, не только Circle.

### Отложенные / опциональные

- **Migration на ruststep** — отложено, слишком большой refactor.
- **ToSameGeometry / IncludeCurve / Concat / Cut / Processor trait'ы** — отложено, massive API refactor.
- **`CompressedShell`** — отложено, unused.
- **Derive macros** — отложено.
- **ABC dataset benchmark** — отложено, не критично.
- **Refactor `triangulate.rs` (8308 строк)** — отложено, опасно без покрытия тестами.
- **Dead code cleanup (custom_cdt, robust_cdt)** — отложено, не критично.

---

## 8. Рекомендация по порядку выполнения

Сразу после **P2** (step_id_aliases) рекомендую переключиться на **P3 → P4** (HYPERBOLA/PARABOLA/ARC + BREP_WITH_VOIDS/VERTEX_LOOP) — это **быстрые wins** (~6-10 часов суммарно), которые **закрывают silent data loss в STEP parser**. Это напрямую влияет на industrial pipeline.

Далее **P5 (NURBS toolkit)** и **P6 (analytical derivatives)** — это **фундамент** для всех будущих geometry improvements. Без knot insertion невозможно сделать robust NURBS intersection; без analytical derivatives — robust Boolean ops для curved surfaces.

**P7 (pole-degenerate)** можно делать параллельно с P5/P6 — это independent fix.

**P8 (PCurve)** и **P9 (IntersectionCurve)** — это крупные architectural changes, делать после P5/P6.

**P10 (Loop subdivision)** и **P11 (TRIMMED_CURVE)** — низкий приоритет, можно отложить.
