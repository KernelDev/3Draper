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
| **P6** | HYPERBOLA + PARABOLA + ARC handlers (silent STEP drops) | ⏳ TODO | — |
| **P7** | BREP_WITH_VOIDS inner shells + VERTEX_LOOP | ⏳ TODO | — |
| **P8** | NURBS toolkit (insert_knot, elevate_degree, bezier_decomposition) | ⏳ TODO | — |
| **P9** | Analytical derivatives для Revolution/Extrusion (chain rule) | ⏳ TODO | — |
| **P10** | PolyBoundary pole-degenerate (sphere/torus poles) | ⏳ TODO | — |
| **P11** | PCurve as analytical curve-on-surface | ⏳ TODO | — |
| **P12** | IntersectionCurve first-class (4D Newton `double_projection`) | ⏳ TODO | — |
| **P13** | Loop subdivision + quadrangulation | ⏳ TODO | — |
| **P14** | TRIMMED_CURVE proper handling (generic, not only Circle→Arc) | ⏳ TODO | — |

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

После каждой задачи:
- `cargo check --release` — 0 errors, 0 warnings
- `cargo test --workspace --lib` — без регрессий
- `cargo run --release --bin all_files_test` — 24/24 watertight
- `git add -A && git commit -m "feat(Pn): ..."` 
- `git push origin main`
- Обновить этот документ: ✅ DONE + commit hash

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
