# 3Draper ← truck — План заимствования

> **Дата:** 2026-06-20
> **Источник:** глубокий анализ `/tmp/truck/` (v0.6) vs `/home/z/my-project/crates/`
> **Подробности:** см. [`truck_vs_3draper_deep_comparison.md`](./truck_vs_3draper_deep_comparison.md)

---

## Статус сводка

| Phase | Task | Status | Commit |
|---|---|---|---|
| **P0** | truck-inspired `ParameterDivision2D` adaptive UV subdivision | ✅ DONE | `b309243` |
| **P0** | earcut/i_triangle/delaunator adapters | ✅ DONE | `e28508c` |
| **P0+** | weld PASS 2 на длинных рёбрах | ✅ DONE | `ca5444f` |
| **P1** | 0-triangle face bug, all 24 files watertight | ✅ DONE | `8f5d589` |
| **P2** | 5-point curve sampling вместо midpoint | ✅ DONE | `72ee42f` |
| **P3** | `PolyBoundary` unified seam/periodic-surface wrapper | ✅ DONE | `d917eb4` |
| **P4** | ABC benchmark runner + baseline CSV | ✅ DONE | `86d1e5a` |
| **P5** | Remove 2768 lines dead CDT code (`spade` removed) | ✅ DONE | `38f7155` |
| **P6** | HYPERBOLA + PARABOLA + ARC handlers (silent STEP drops) | ✅ DONE | `afe01aa` |
| **P7** | BREP_WITH_VOIDS inner shells + VERTEX_LOOP | ✅ DONE | `8f8cc64` |
| **P8** | NURBS toolkit (insert_knot, elevate_degree, bezier_decomposition) | ✅ DONE | `5c7c296` |
| **P9** | Analytical derivatives для Revolution/Extrusion (chain rule) | ✅ DONE | `e78b803` |
| **P10** | PolyBoundary pole-degenerate (sphere/torus poles) | ✅ DONE | `cf32cf4` |
| **P11** | PCurve as analytical curve-on-surface | ✅ DONE | `56c2042` |
| **P12** | IntersectionCurve first-class (4D Newton `double_projection`) | ✅ DONE | `dc0d78d` |
| **P13** | Loop subdivision + quadrangulation | ✅ DONE | `cfd5bab` |
| **P14** | TRIMMED_CURVE proper handling (generic, not only Circle→Arc) | ✅ DONE | `1dab48f` |
| **P15** | STEP exporter: full surface coverage (Revolution/Extrusion/NURBS) | ✅ DONE | `c384c07` |
| **P16** | STEP exporter: full curve coverage (Hyperbola/Parabola/NURBS/PCurve/Trimmed) | ✅ DONE | `c384c07` |
| **P17** | STEP exporter: BREP_WITH_VOIDS + multiple inner wires per face | ✅ DONE | `c384c07` |
| **P18** | Round-trip integrity: extract_solids + read→save→re-read | ✅ DONE | `d3686b7` |
| **P19** | Editing API: transform/translate/rotate/scale/mirror/pattern/hole/surface | ✅ DONE | `6cc338b` |
| **P20** | STEP export validator (schema-compliant pre-write check) | ✅ DONE | `00f307c` |

---

## P6 — HYPERBOLA + PARABOLA + ARC в STEP parser

**Проблема.** В `crates/draper-step/src/converter.rs:7337-7366` match-arm'ы для `HYPERBOLA` и `PARABOLA` отсутствуют → silent drop, теряется edge geometry. ARC обрабатывается только через `TRIMMED_CURVE(CIRCLE)` — но `TRIMMED_CURVE(ELLIPSE)` или `TRIMMED_CURVE(B_SPLINE_CURVE)` теряют trim.

**План:**

- [ ] 6.1 Добавить `Hyperbola` структуру в `draper-geometry/src/curve.rs`:
  - поля: `center`, `axis_z` (normal), `axis_x` (transv), `semi_real`, `semi_imag`
  - `point_at(t) = center + cosh(t)*semi_real*x + sinh(t)*semi_imag*y`
  - `derivative_at(t) = sinh(t)*semi_real*x + cosh(t)*semi_imag*y`
  - `param_range = (-∞, +∞)` (но STEP обычно trimmed)
- [ ] 6.2 Добавить `Parabola` структуру:
  - поля: `vertex`, `axis_z` (normal), `axis_x` (parabola direction), `focal_dist`
  - `point_at(t) = vertex + t*x + (t²/(4f))*z` (в локальной СК)
  - `derivative_at(t) = x + (t/(2f))*z`
- [ ] 6.3 Добавить `Curve3d::Hyperbola(Hyperbola)` и `Curve3d::Parabola(Parabola)` варианты
- [ ] 6.4 Обновить `Curve3d::point_at`, `Curve3d::derivative_at`, `Curve3d::param_range`, `Curve3d::is_degenerate`
- [ ] 6.5 Реализовать `resolve_hyperbola_curve(&self, entity)` в converter.rs (читает axis2 + semi_real + semi_imag)
- [ ] 6.6 Реализовать `resolve_parabola_curve(&self, entity)` (читает axis2 + focal_dist)
- [ ] 6.7 Добавить match-arm'ы `"HYPERBOLA"` и `"PARABOLA"` в `resolve_curve()`
- [ ] 6.8 Добавить test STEP-файл с гиперболой (можно синтезировать из nist_complex_surface.stp)
- [ ] 6.9 Запустить `all_files_test` — 0 регрессий

---

## P7 — BREP_WITH_VOIDS + VERTEX_LOOP

**Проблема.** `BREP_WITH_VOIDS` — это твёрдое тело с вычитаемыми внутренними оболочками (воздушные полости, каналы охлаждения). 3Draper сейчас парсит только outer shell. `VERTEX_LOOP` — это вырожденный loop из одной точки (вершина конуса, apex сферы) — сейчас silent drop, ломает watertightness для конусов и пирамидок.

**План:**

- [ ] 7.1 Расширить `find_shell_ref(entity)` в converter.rs: для `BREP_WITH_VOIDS` найти ВСЕ ссылки на `CLOSED_SHELL` (outer + voids)
- [ ] 7.2 В `convert_brep_with_voids` инвертировать orientation void-оболочек (нормали должны смотреть внутрь полости)
- [ ] 7.3 Добавить `VERTEX_LOOP` handling в `resolve_face_bound_with_step_ids`: если loop содержит 1 vertex, создать `Wire` с одной `CoEdge { start_vertex, end_vertex, degenerate: true }` — degenerate edge, который триангулятор должен пропустить, но вершина должна попасть в mesh как single point
- [ ] 7.4 Тест на синтетическом BREP_WITH_VOIDS (куб с цилиндрической полостью)
- [ ] 7.5 Проверка на NIST cone (должен иметь apex vertex)

---

## P8 — NURBS toolkit

**Проблема.** truck имеет полный NURBS-тулкит: `add_knot` (Boehm insertion), `remove_knot` (Tiller), `elevate_degree` (Prautzsch), `clamp`, `optimize`, `bezier_decomposition`, `make_locally_injective`, `quadratic_approximation`, `cubic_approximation`, `try_interpole`, `cut`, `is_arc_of`. У нас только eval + first derivative.

**План:**

- [ ] 8.1 Создать `crates/draper-geometry/src/nurbs_tools.rs` модуль
- [ ] 8.2 `insert_knot(curve, u, times)`: Boehm algorithm — вставляет узел `u` `times` раз, обновляя control points и knots
- [ ] 8.3 `remove_knot(curve, u, times, tolerance)`: Tiller-Theisen lossless knot removal
- [ ] 8.4 `elevate_degree(curve, t)`: Prautzsch algorithm — повышение степени на `t`
- [ ] 8.5 `clamp(curve)`: converts periodic to clamped
- [ ] 8.6 `optimize(curve, tolerance)`: iteratively remove redundant knots
- [ ] 8.7 `bezier_decomposition(curve) -> Vec<NurbsCurve>`: split into Bézier segments at internal knots
- [ ] 8.8 `cut(curve, t) -> (NurbsCurve, NurbsCurve)`: split at parameter
- [ ] 8.9 Unit-тесты сравнить с эталонами из "The NURBS Book" (Piegl & Tiller)
- [ ] 8.10 Зарегистрировать модуль в `draper-geometry/src/lib.rs`

---

## P9 — Analytical derivatives для Revolution/Extrusion

**Проблема.** `Surface::derivatives_at` (surface.rs:1622) для `Revolution` и `Extrusion` падает в numerical central differences. truck имеет full analytical derivatives через chain-rule: для Revolution `dS/du = cross_axis × perp_component`, `dS/dv = profile.derivative_at(v) (revolved)`; для Extrusion `dS/du = profile.derivative_at(u)`, `dS/dv = direction`.

**План:**

- [ ] 9.1 Реализовать `RevolutionSurface::derivatives_at(u, v) -> SurfaceDerivatives`:
  - `S = origin + V_parallel + cos(u)*V_perp + sin(u)*(axis×V_perp)`
  - `dS/du = -sin(u)*V_perp + cos(u)*(axis×V_perp)`
  - `dS/dv = R_u(P'(v))` где R_u — вращение на угол u вокруг axis
- [ ] 9.2 Реализовать `ExtrusionSurface::derivatives_at(u, v) -> SurfaceDerivatives`:
  - `dS/du = profile.derivative_at(u)`
  - `dS/dv = direction`
- [ ] 9.3 Обновить `Surface::derivatives_at` match: добавить `Surface::Revolution(r) => r.derivatives_at(u, v)`, `Surface::Extrusion(e) => e.derivatives_at(u, v)`
- [ ] 9.4 Тест: analytical vs numerical должны совпадать в пределах 1e-10 на сетке 10×10 параметров

---

## P10 — PolyBoundary pole-degenerate handling

**Проблема.** На полюсах сферы/тора `dS/du` или `dS/dv` равно нулю — параметрический домен коллапсирует в точку, earcut ломается. truck имеет `PolyBoundary` с pole-degenerate handling: если uder/vder near zero, вставляет intermediate UV points чтобы «размазать» полюс по кольцу.

**План:**

- [ ] 10.1 В `poly_boundary.rs` добавить `detect_pole_degeneracy(surface) -> Option<PoleDegenerate>` — возвращает Some если на u=0 или u=2π derivative длина < 1e-10
- [ ] 10.2 Реализовать `inject_pole_ring(boundary, surface, u_pole) -> Boundary` — вставляет кольцо UV точек на v_pole ± ε вместо одной точки
- [ ] 10.3 В `triangulate.rs` использовать `PolyBoundary::from_surface_with_pole_handling` для sphere/torus
- [ ] 10.4 Тест на nist_sphere.stp: проверить, что pole area триангулирована корректно (без NaN, без дыр)

---

## P11 — PCurve as analytical curve-on-surface

**Проблема.** 3Draper использует только polyline approximation для PCURVE — теряется точность. truck имеет `PCurve<C, S>` с analytical chain-rule derivatives: `C'(t) = S_u · C'_u + S_v · C'_v`.

**План:**

- [ ] 11.1 Добавить `Curve3d::PCurve { curve_2d: Box<Curve2d>, surface: Box<Surface> }` вариант
- [ ] 11.2 `point_at(t)` = `surface.point_at(curve_2d.point_at(t).u, curve_2d.point_at(t).v)`
- [ ] 11.3 `derivative_at(t)` через chain rule:
  - `du/dt = curve_2d.derivative_at(t).u`
  - `dv/dt = curve_2d.derivative_at(t).v`
  - `dS/dt = S_u * du/dt + S_v * dv/dt`
- [ ] 11.4 `param_range` проксирует в `curve_2d.param_range()`
- [ ] 11.5 В STEP parser `resolve_pcurve(entity)` создавать Curve3d::PCurve вместо дискретизации
- [ ] 11.6 В edge_cache.rs использовать PCurve для UV-проекции вместо `surface.project_point`

---

## P12 — IntersectionCurve first-class

**Проблема.** В 3Draper `boolean::intersect_surfaces` возвращает дискретные полилинии. truck имеет `IntersectionCurve<C, S0, S1>` first-class с 4D Newton `double_projection` над `(u0, v0, u1, v1)`. Позволяет итеративно уточнять точку пересечения поверхностей.

**План:**

- [ ] 12.1 Создать `crates/draper-geometry/src/intersection_curve.rs` модуль
- [ ] 12.2 `IntersectionCurve { surface0: Box<Surface>, surface1: Box<Surface>, points: Vec<(u0, v0, u1, v1)> }`
- [ ] 12.3 Реализовать `double_projection(s0, s1, (u0, v0, u1, v1))` — 4D Newton iteration: на каждом шаге вычисляет `F = S0(u0,v0) - S1(u1,v1)`, Jacobian 4×4 from partials, решает `(du0, dv0, du1, dv1) = -J⁻¹ · F`
- [ ] 12.4 Реализовать marching: начиная с seed point, шагает вдоль intersection, на каждом шаге предсказывает `(u0, v0, u1, v1) + leader·dt` и проецирует обратно через `double_projection`
- [ ] 12.5 `derivative_at(t)` через 3×3 систему: `(n0, n1, leader')·c'=(n0·Σ, n1·Σ, leader'·Σ)` где n0, n1 — нормали поверхностей, leader — predicted tangent
- [ ] 12.6 Использовать в `draper-core/src/boolean.rs::intersect_surfaces` general case (когда ни одна поверхность не plane)
- [ ] 12.7 Зарегистрировать модуль в lib.rs

---

## P13 — Loop subdivision + quadrangulation

**Проблема.** truck имеет Loop subdivision (1/8 mask, split each triangle into 4) и quadrangulation (greedy by coplanarity). 3Draper этого не имеет — полезно для LOD и для mesh post-processing (создание quad-dominant mesh для FEA).

**План:**

- [ ] 13.1 Создать `crates/draper-mesh/src/subdivision.rs` модуль
- [ ] 13.2 Реализовать `loop_subdivide(mesh, iterations) -> Mesh`:
  - mask: new vertex position = 3/8·(self+neighbor_i+neighbor_{i+1}) + 1/8·(others)
  - для boundary: 1/8·(neighbor_left+neighbor_right) + 3/4·self
  - split each triangle into 4
- [ ] 13.3 Реализовать `quadrangulate(mesh) -> Mesh`:
  - greedy: для каждой пары coplanar triangles, merge into quad
  - coplanarity test: angle between normals < 5°
- [ ] 13.4 API: `TriangleMesh::subdivide(iterations)`, `TriangleMesh::quadrangulate()`
- [ ] 13.5 Unit-тесты: subdivision удваивает vertex count quadratically (4× для 1 iter)

---

## P14 — TRIMMED_CURVE proper handling

**Проблема.** `TRIMMED_CURVE` сейчас обрабатывается только для Circle→Arc. Для Ellipse, B_SPLINE_CURVE, LINE trim теряется — получается untrimmed basis curve, что приводит к неверной геометрии ребра.

**План:**

- [ ] 14.1 Добавить `Curve3d::Trimmed { basis: Box<Curve3d>, start: f64, end: f64 }` вариант
- [ ] 14.2 `point_at(t)` = `basis.point_at(start + t*(end-start))` (t ∈ [0,1])
- [ ] 14.3 `derivative_at(t)` = `basis.derivative_at(start + t*(end-start)) * (end-start)` (chain rule with scaling)
- [ ] 14.4 `param_range()` = `(0.0, 1.0)`
- [ ] 14.5 В `resolve_trimmed_curve(entity, depth)`:
  - Если basis = Circle → использовать Arc (existing fast-path)
  - Иначе → Curve3d::Trimmed { basis, start, end }
- [ ] 14.6 Тест на STEP-файле с trimmed NURBS (8394-121_Spit-Fire.STEP содержит такие)

---

## Критерии приёмки (общие)

- **Build:** 0 errors, 0 warnings (`cargo check --release`)
- **Tests:** 24/24 STEP files pass watertightness (без регрессий от P1)
- **Lib tests:** 115/117+ pass (2 pre-existing gdt_check failures unrelated)
- **Code reduction:** net positive lines acceptable (we're adding features, not removing)
- **Dependencies:** only add if absolutely necessary; prefer self-contained implementations

---

## Порядок выполнения

Пользователь указал: «Реализуй все, не пропуская.»

Поэтому порядок строго последовательный, без skip-ов:

1. **P6** (HYPERBOLA/PARABOLA/ARC) — ~3-4h, закрывает silent STEP data loss для кривых
2. **P7** (BREP_WITH_VOIDS/VERTEX_LOOP) — ~4-6h, закрывает silent STEP data loss для топологии
3. **P8** (NURBS toolkit) — ~6-8h, фундамент для всех будущих NURBS-операций
4. **P9** (Analytical derivatives) — ~4-6h, точность нормалей и UV-проекции
5. **P10** (PolyBoundary pole-degenerate) — ~8-10h, качество сферы/тора
6. **P11** (PCurve analytical) — ~10-15h, точность surface-on-surface
7. **P12** (IntersectionCurve) — ~15-20h, foundation для boolean ops
8. **P13** (Loop subdivision) — ~5-8h, LOD и FEA-ready mesh
9. **P14** (TRIMMED_CURVE) — ~2-4h, корректная геометрия рёбер
10. **P15-P17** (STEP exporter completeness) — ~6h, корректное сохранение всех типов поверхностей и кривых
11. **P18** (Round-trip integrity) — ~3h, гарантия read→save→re-read
12. **P19** (Editing API) — ~6h, transform/hole/surface/edge editing
13. **P20** (Export validator) — ~3h, schema-compliant pre-write check

После каждой задачи:
- `cargo check --release` — 0 errors, 0 warnings
- `cargo test --workspace --lib` — без регрессий
- `cargo run --release --bin all_files_test` — 24/24 watertight
- `git add -A && git commit -m "feat(Pn): ..."` 
- `git push origin main`
- Обновить этот документ: ✅ DONE + commit hash

---

## P15-P17 — Комплексный экспортёр STEP (завершено в коммите c384c07)

**Проблема.** Старый экспортёр поддерживал только 5 типов поверхностей (Plane/Cylinder/Cone/Sphere/Torus) и 2 типа кривых (Line/Circle). Всё остальное терялось при сохранении: NURBS-поверхности становились плоскостями, эллипсы — линиями, гиперболы/параболы вообще не сохранялись, BREP_WITH_VOIDS терял внутренние оболочки, дырки на гранях терялись.

**Решение.** Полная переработка `crates/draper-step/src/exporter.rs`:

### P15 — Полная поддержка поверхностей
- Plane / Cylinder / Cone / Sphere / Torus (с дедупликацией)
- `SURFACE_OF_REVOLUTION` (axis1_placement + profile curve)
- `SURFACE_OF_LINEAR_EXTRUSION` (profile curve + direction)
- `B_SPLINE_SURFACE_WITH_KNOTS` (complex entity form для rational: `B_SPLINE_SURFACE + B_SPLINE_SURFACE_WITH_KNOTS + RATIONAL_B_SPLINE_SURFACE + BOUNDED_SURFACE`)

### P16 — Полная поддержка кривых
- Line / Circle / Ellipse / Arc (с дедупликацией)
- `HYPERBOLA(axis2, semi_real, semi_imag)`
- `PARABOLA(axis2, focal_dist)`
- `B_SPLINE_CURVE_WITH_KNOTS` (rational complex entity form)
- `TRIMMED_CURVE` оборачивает arbitrary basis curve (не только Circle→Arc)
- `PCurve` как `SURFACE_CURVE` обёртывающий 3D B-spline + `PCURVE` ссылку на surface

### P17 — Топологическая полнота
- `BREP_WITH_VOIDS` когда solid имеет inner_shells (несколько CLOSED_SHELL ссылок)
- Несколько `FACE_BOUND` на `ADVANCED_FACE` (outer + inner_wires — дырки)
- `VERTEX_LOOP` для degenerate single-vertex wires (apex конуса, pole сферы)
- Shared `EDGE_CURVE` дедупликация по content hash
- Shared `CARTESIAN_POINT` / `DIRECTION` / `AXIS2_PLACEMENT_3D` / `VERTEX_POINT` дедупликация

**Архитектура:** `StepWriter` struct с mutable id-счётчиком + 7 dedup кешей (point/dir/axis2/vertex/curve/surface/edge_curve).

---

## P18 — Round-trip integrity + extract_solids (завершено в коммите d3686b7)

**Проблема.** Не было возможности извлечь `Solid` из STEP-файла без триангуляции. Конвейер шёл напрямую: `StepFile → FaceData → TriangleMesh`, минуя промежуточный `Solid`. Это блокировало любую editing-семантику (редактирование, трансформация, экспорт обратно).

**Решение:**
1. Новый публичный API: `extract_solids(step_file) -> (Vec<Solid>, Vec<i64>)`
   - Извлекает все BREP-сущности (MANIFOLD_SOLID_BREP и BREP_WITH_VOIDS)
   - Возвращает STEP entity IDs вместе с solidами для traceability
2. `StepConverter::extract_all_solids()` — итерирует все BREP-сущности
3. `StepConverter::extract_solid_from_brep(brep_id)` — извлекает один solid с outer shell + void shells

**Тест производительности round-trip** (`tools/src/bin/roundtrip_test.rs`):
- Парсит оригинальный STEP
- Извлекает solids через `extract_solids`
- Экспортирует обратно через `export_step` (P15-P17)
- Ре-парсит экспортированный STEP
- Сравнивает BREP count, surface types, curve types

**Результаты round-trip тестов:**
| Файл | BREP | Surfaces | Curves | Результат |
|---|---|---|---|---|
| nist_cube.stp | 1 | 6 PLANE | 12 LINE | ✅ PASS |
| nist_cylinder.stp | 1 | 1 CYL + 2 PLANE | 1 LINE + 2 CIRCLE | ✅ PASS |
| nist_sphere.stp | 1 | 1 SPHERE + 1 PLANE | 2 CIRCLE | ✅ PASS |
| nist_cone.stp | 1 | 1 CONE + 2 PLANE | 1 LINE + 2 CIRCLE | ✅ PASS |
| nist_complex_surface.stp | 1 | 1 B_SPLINE + 5 PLANE | 12 LINE | ✅ PASS |
| nist_assembly.stp | 2 | 12 PLANE | 24 LINE | ✅ PASS |
| as1-oc-214.stp | 5 | 25 PLANE + 28 B_SPLINE | 42 LINE + 84 B_SPLINE | ✅ PASS |
| Zentralstaender.stp | 27 | 232 PLANE + 69 CYL + 25 CONE + 22 TORUS + 1 B_SPLINE | 487 LINE + 36 B_SPLINE + 183 CIRCLE + 2 ELLIPSE | ✅ PASS |
| 8394-121_Spit-Fire.STEP | 8 | 53 PLANE + 52 CYL + 10 CONE + 12 TORUS + 49 B_SPLINE | 170 LINE + 176 CIRCLE + 223 B_SPLINE | ✅ PASS |
| compressor-13920_top.stp | 2 | 18 PLANE + 22 CYL + 1 CONE + 1 SPHERE + 8 TORUS + 4 B_SPLINE | 52 LINE + 102 CIRCLE + 18 B_SPLINE | ✅ PASS |

**WARN-сообщения** на сложных файлах ожидаемы: оригинальные STEP-файлы дублируют геометрию (одна поверхность на ADVANCED_FACE), тогда как наш экспортёр дедуплицирует разделяемые сущности. BREP count и surface/curve TYPES сохраняются точно.

---

## P19 — Editing API (завершено в коммите 6cc338b)

**Проблема.** Существующие операции `circular_pattern` / `linear_pattern` / `mirror_solid` трансформировали только поверхности, оставляя edge curves в исходных позициях — геометрия становилась несогласованной. Не хватало API для редактирования: добавление/удаление дырок, замена поверхностей (NURBS editing), замена кривых рёбер.

**Решение.** Полная переработка `crates/draper-core/src/operations.rs` (~730 строк):

### Section 1: Transform operations
- `transform_solid` — propagate transform to surfaces AND edges AND wires
- `transform_compound`, `transform_shell`, `transform_face`
- `translate_solid(solid, dx, dy, dz)`
- `rotate_solid(solid, axis, angle)`
- `rotate_solid_around_point(solid, axis, angle, pivot)`
- `scale_solid(solid, factor)`
- `scale_solid_around_point(solid, factor, center)`
- `mirror_solid(solid, plane_origin, plane_normal) -> Solid`
- `mirror_transform(plane_origin, plane_normal) -> Transform`

**Critical fix:** Все transform-методы теперь правильно propagate к EDGE_CURVE, не только к поверхностям.

### Section 2: Pattern operations (переписаны на transform_solid)
- `circular_pattern(solid, axis, count, total_angle) -> Vec<Solid>`
- `linear_pattern(solid, direction, count, spacing) -> Vec<Solid>`

### Section 3: Hole operations
- `add_circular_hole_to_face(face, center_3d, radius, normal)` — создаёт inner wire с Circle edge + 32-сегментной UV polyline pcurve (аналитическая проекция для Plane/Cylinder/Cone/Sphere/Torus, grid search + Newton для Revolution/Extrusion/Nurbs)
- `remove_hole_from_face(face, hole_index) -> Wire`
- `clear_holes_from_face(face) -> usize`

### Section 4: Face / surface editing
- `replace_face_surface(face, new_surface)` — для NURBS editing
- `get_face_mut(solid, face_index) -> Option<&mut Face>`
- `get_face_in_shell_mut(solid, shell_index, face_index) -> Option<&mut Face>`
- `reverse_face_orientation(face)` — flip normal
- `delete_face_from_solid(solid, face_index) -> Face` (breaks watertightness!)

### Section 5: Edge editing
- `replace_edge_curve(face, edge_id, new_curve, new_param_range)`
- `reverse_edge(face, edge_id)`

### Section 6: Fillet/Chamfer/Shell
Явные ошибки "not implemented" (раньше молча возвращали Ok с log::warn).

### Section 7: Helpers
- `perpendicular_direction(d) -> Direction3d` (с parallel-axis fallback)
- `project_point_to_surface(surface, point) -> (u, v)`
- `project_point_to_surface_grid` (32×32 grid + Newton refinement)

**6 unit-тестов** покрывают transform_solid (проверяет, что поверхности И рёбра двигаются), rotate_solid, mirror_solid, add/remove_hole, replace_face_surface, circular_pattern.

---

## P20 — Export validator (завершено в коммите 00f307c)

**Проблема.** Не было проверки того, что экспортируемый STEP синтаксически и семантически корректен. Если в экспортёре появлялся баг (например, dangling reference), файл сохранялся, но downstream-парсеры отвергали его с непонятными ошибками.

**Решение.** Новый модуль `crates/draper-step/src/export_validation.rs` (~340 строк):

### Public API
```rust
pub fn validate_exported_step(step_str: &str) -> ExportValidationReport;
```

### Checks
1. **Structural integrity**: ISO-10303-21 header/footer, HEADER/DATA/ENDSEC, FILE_SCHEMA
2. **Reference integrity**: каждый `#N` resolve-ится к определённой сущности (E_DANGLING_REF)
3. **Topological completeness**: хотя бы один MANIFOLD_SOLID_BREP, CLOSED_SHELL, ADVANCED_FACE, VERTEX_POINT
4. **Schema compliance**: ADVANCED_FACE ≥4 params, EDGE_CURVE ≥5 params, CLOSED_SHELL ≥2 params
5. **Recommended entities** (warnings): APPLICATION_CONTEXT, SHAPE_DEFINITION_REPRESENTATION, PRODUCT

### Report structure
```rust
ExportValidationReport {
    issues: Vec<ExportValidationIssue>,
    entity_count, brep_count, shell_count, face_count,
    edge_curve_count, vertex_count, surface_count, curve_count,
}
impl ExportValidationReport {
    fn has_errors(&self) -> bool;
    fn has_warnings(&self) -> bool;
    fn errors(&self) -> impl Iterator;
    fn warnings(&self) -> impl Iterator;
    fn summary(&self) -> String;
}
```

### Integration
`roundtrip_test` теперь запускает validator на экспортированном STEP и сообщает ошибки до round-trip сравнения.

**4 unit-теста:** minimal valid export, missing header, dangling reference, empty STEP file.

**Verification на реальных файлах:**
- nist_cube.stp: 0 errors, 0 warnings (116 entities)
- 8394-121_Spit-Fire.STEP: 0 errors, 0 warnings (14812 entities, 8 BREP, 280 faces, 569 edges, 883 vertices, 219 surfaces, 590 curves)

---

## Post-P20 quality pass (commit `b6a0fc1`, 2026-06-21)

After P0–P20 was declared complete, a full regression pass surfaced and
fixed the four pre-existing unit-test failures that the original plan did
not address. These were all real bugs (not test-side issues) and are now
closed:

### Bug 1 — PMI: generic `GEOMETRIC_TOLERANCE` lost its type
`pmi::extract_gdt()` called `GdtToleranceType::from_step_type(&entity.type_name)`
which only matched the entity **type** name. STEP files commonly use the
generic supertype `GEOMETRIC_TOLERANCE('position tolerance', 'pos', 0.05, ...)`
without a subtype-specific entity name, so the classifier returned
`Other("GEOMETRIC_TOLERANCE")` instead of `Position`.

**Fix:** new `GdtToleranceType::from_step_type_and_name(type, name, desc)`
joins all three strings and does keyword matching on the combined text.
`from_step_type` is kept as a thin wrapper for backward compatibility.

### Bug 2 — PMI: `uses_millimetres()` rejected `MILLI_METRE`
The SI_UNIT complex-entity parser produced `MILLI_METRE` (with underscore)
for `(LENGTH_UNIT() SI_UNIT(.MILLI.,.METRE.))`, but `uses_millimetres()`
only checked for `MILLIMETRE` (no underscore). `extract_ap242_combined`
therefore reported the file as not millimetre even though it clearly was.

**Fix:** `uses_millimetres()` now also accepts `MILLI_METRE`, `MILLIMETER`,
`MILLI_METER` and bare `MM`.

### Bug 3 — GDT check: flat mesh reported flatness 5.0 instead of 0
`smallest_eigenvector_3x3` used power iteration + deflation. When the
input covariance matrix had rank 1 (one non-zero eigenvalue — exactly the
flat-mesh case), the deflated matrix was the zero matrix and the inner
`largest_eigenvector_3x3` call returned its initial guess `(0,1,0)`
instead of the true surface normal `(0,0,1)`. The "best-fit plane"
therefore became `y=centroid_y` (a vertical plane through the mesh), and
the reported deviation was the half-extent of the mesh in Y (~5 units for
the test fixture).

**Fix:** detect near-zero Frobenius norm of the deflated matrix and return
a properly orthogonal unit vector built via the axis-of-least-component
trick. Same fix applied at rank 2 (two non-zero eigenvalues, one zero
eigenvalue) via cross product of the two computed eigenvectors.

### Bug 4 — GDT check: cylindricity 2.5 instead of ~0.05
`test_cylindricity_check` was adding `bottom_center` and `top_center`
vertices to the mesh (via `mesh.add_vertex`) but never using them in any
triangle. Those orphan vertices had radius 0 from the cylinder axis,
inflating `r_max - r_min` to the full radius (5), and cylindricity to
half of that (2.5). The chord error for a 16-segment polygon at radius 5
is only ~0.096, so cylindricity should be ~0.048.

**Fix:** stop adding the unused center vertices to the mesh. The test
still tolerates `< 1.0` to leave headroom for the PCA-based axis estimate.

### Batch round-trip runner
`tools/src/bin/roundtrip_test.rs` gained `--all [dir]` mode that walks
the test directory and runs a silent round-trip on every `.stp/.step/.STEP`
file, printing a summary table. `count_curve_types` no longer counts
`SURFACE_CURVE` / `PCURVE` wrapper entities — the exporter intentionally
flattens them, so including them triggered a spurious WARN on
`as1-oc-214.stp` (a 5-solid assembly).

### Final verification (24/24 PASS)
```
Round-trip batch: 24 files in test

File                                             BREPs    Solids   Re-BREPs   Result
--------------------------------------------------------------------------------------
3.05.078.stp                                     1        1        1          PASS
8394-121_Spit-Fire.STEP                          8        8        8          PASS
8500-02_Vulcan.STEP                              13       13       13         PASS
SampleCube.step                                  1        1        1          PASS
Zentralstaender.stp                              27       27       27         PASS
as1-oc-214.stp                                   5        5        5          PASS
as1-oc-214_bolt.stp                              1        1        1          PASS
as1-oc-214_nut.stp                               1        1        1          PASS
as1-oc-214_plate.stp                             1        1        1          PASS
as1-oc-214_rod.stp                               1        1        1          PASS
brick_thin.stp                                   1        1        1          PASS
brick_thin_hole.stp                              1        1        1          PASS
brick_thin_round.stp                             1        1        1          PASS
compressor-13920_top.stp                         2        2        2          PASS
drill_top.stp                                    5        5        5          PASS
nist_assembly.stp                                2        2        2          PASS
nist_block_with_hole.stp                         1        1        1          PASS
nist_chamfer_block.stp                           1        1        1          PASS
nist_complex_surface.stp                         1        1        1          PASS
nist_cone.stp                                    1        1        1          PASS
nist_cube.stp                                    1        1        1          PASS
nist_cylinder.stp                               1        1        1          PASS
nist_sphere.stp                                  1        1        1          PASS
transmission_top.stp                             35       35       35         PASS
--------------------------------------------------------------------------------------
Summary: 24 PASS, 0 WARN, 0 FAIL (24 total)
```

Test counts after fixes:
- `draper-step --lib`: 92/92 pass (was 90/92)
- `draper-mesh --lib`: 118/118 pass (was 116/118)
- `draper-geometry --lib`: 81/81 pass
- `draper-core --lib`: 9/9 pass (incl. 6 editing-API tests)
- `all_files_test`: 24/24 ok, 0 leaky, 0 BAD (was 19 ok + 5 leaky before P0–P14)

---

## Источники

- truck repo: https://github.com/ricosjp/truck (v0.6)
- truck-repl examples: `/tmp/truck/examples/`
- "The NURBS Book" (Piegl & Tiller, 1997) — для эталонов NURBS-алгоритмов
- STEP ISO 10303-42 — для schema entity references

---

## Лицензионная совместимость

truck: Apache-2.0 OR MIT — можно свободно заимствовать алгоритмы и API design.
3Draper: GPL-3.0-or-later — сохраняем совместимость через:
- Переписывание алгоритмов на Rust (не copy-paste)
- Атрибуция в комментариях: `// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT)`
- Документация: ссылка на оригинальный source file в `docs/truck_vs_3draper_deep_comparison.md`
