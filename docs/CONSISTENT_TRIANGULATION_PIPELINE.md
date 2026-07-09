# Полный pipeline согласованной триангуляции 3Draper

## Обзор архитектуры

Pipeline построен вокруг **топология-first принципа**: общие рёбра между гранями дискретизируются один раз и кэшируются. Все грани, ссылающиеся на одно ребро, получают **бит-идентичные** 3D-точки, что делает watertightness конструктивным свойством, а не пост-обработкой.

```
┌─────────────────────────────────────────────────────────────────────────┐
│  ФАЗА 1: Подготовка кэша рёбер                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │ EdgeDiscretizationCache (max_samples=256)                       │    │
│  │  ├── Phase 1 aliasing: vertex-pair + curve shape matching       │    │
│  │  ├── Phase 2 aliasing: 3D coordinate matching                   │    │
│  │  ├── adaptive_discretize (LINE/CIRCLE/NURBS)                    │    │
│  │  └── compute_uvs (PCURVE или project_point + snap_seam)         │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  ФАЗА 2: Триангуляция граней (per-face)                                 │
│  ┌──────────────┬──────────────┬──────────────┬──────────────────┐      │
│  │   Plane      │  Cylinder/   │  Ruled NURBS │  NURBS CDT       │      │
│  │  (earcutr)   │  Cone tube   │  (strip)     │  (earcutr +      │      │
│  │              │              │              │   Steiner)       │      │
│  └──────────────┴──────────────┴──────────────┴──────────────────┘      │
└─────────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  ФАЗА 3: Сборка BREP                                                    │
│  ├── merge_deduplicating (per-face, cross-face duplicates KEEP)         │
│  └── filter_degenerate_triangles                                        │
└─────────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  ФАЗА 4: Пост-обработка watertightness                                  │
│  ├── weld_boundary_edge_vertices (PASS 1/2/3 + face-aware guard)        │
│  ├── remove_duplicate_triangles (face-aware, cross-face KEEP)           │
│  ├── repair_t_junctions (CDT incremental insertion, only if non-manifold)│
│  ├── remove_duplicate_triangles (повторно)                              │
│  ├── smooth_normals (45° crease angle)                                  │
│  └── validate_watertight (logging only, НЕ применяет repair_mesh)       │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## ФАЗА 1 — Подготовка кэша рёбер

### 1.1. Создание кэша (`crates/draper-step/src/converter.rs:3926`)

```rust
let mut edge_cache = EdgeDiscretizationCache::with_tolerance(tol_ctx.clone(), 256);
edge_cache.set_chord_tolerance_override(Some(params.max_deviation));
```

- `max_samples = 256` — потолок числа точек на ребро (не 64 по умолчанию).
- `chord_tolerance_override` передаёт LOD-толеранс в кэш, чтобы Quality slider влиял на density кривых.

### 1.2. Двухфазный aliasing STEP ID (`converter.rs:3933–4052`)

**Проблема**: STEP-файлы могут использовать разные `EDGE_CURVE` для одного геометрического boundary (например, Plane использует LINE, NURBS-грань использует NURBS-кривую). Кэш по entity ID их не сольёт.

#### Phase 1 — vertex-pair + shape matching (`converter.rs:3938–3982`)

1. Группирует `step_id` по парам вершин `(start_vertex, end_vertex)`.
2. Внутри каждой группы: `group_step_ids_by_curve_shape` — сэмплирует 5 точек на каждой кривой, проверяет что формы совпадают в пределах `shape_tol = model_scale × 2e-3`. Это не сливает полуокружности полной окружности (у них одинаковые endpoints).
3. Canonical = max `edge_curve_complexity_score` (densest representation). Остальные алиасятся на canonical через `register_step_id_alias`.

#### Phase 2 — 3D coordinate matching (`converter.rs:3996–4052`)

1. Для не-алиасированных step_ids вычисляет start/end 3D точки.
2. Quantize к grid cells с `coord_tol = model_scale × 2e-3`.
3. Группирует по canonical key `(s.x,s.y,s.z, e.x,e.y,e.z)` с учётом reversed endpoint order.
4. Алиасинг на max complexity.

### 1.3. Discretize ребра (`crates/draper-mesh/src/edge_cache.rs:435–490`)

`discretize_step_edge(step_id)`:

1. `resolve_canonical_step_id` — проходит по цепочке алиасов (с защитой от циклов).
2. Если canonical уже в кэше → вернуть его.
3. Иначе `adaptive_discretize(edge, n_samples_hint=20)`.

### 1.4. `adaptive_discretize` (`edge_cache.rs:598–745`) — ГДЕ ПОЯВЛЯЮТСЯ НОВЫЕ ТОЧКИ

| Тип кривой | Поведение | Новые точки |
|---|---|---|
| **Line** | Только 2 endpoints | ✗ |
| **Composite** | Segment-by-segment + дедупликация стыков | На стыках сегментов (если не дедуплицированы) |
| **Circle** | **UNIFORM** угловая сетка `n = n_samples_hint.max(8).min(256)`. НЕ адаптивная! | `n` точек на окружности |
| **NURBS/прочее** | Uniform grid → до 5 проходов адаптивного подразбиения по `effective_chord_tolerance()`, пока `points.len() < 256` | Адаптивно, на участках с высокой кривизной |

**КРИТИЧЕСКОЕ РЕШЕНИЕ для Circle** (`edge_cache.rs:668–683`): uniform-сетка выбрана специально для watertightness tube-граней. Два круга на одной оси с разными радиусами (R=30.36 vs R=35.22) ДОЛЖНЫ иметь одинаковое `n` в одинаковых угловых позициях. Адаптивное подразбиение дало бы разный `n` для разных радиусов, ломая `bottom[i] → top[i]` соединение в tube-гранях.

### 1.5. Deterministic rounding (`edge_cache.rs:64–70`)

```rust
fn deterministic_round_point(p: Point3d) -> Point3d {
    // Обрезает 4 младших бита мантиссы f64
    // ~1e-14 относительная точность
}
```

Это гарантирует, что одна и та же 3D-точка, вычисленная разными путями (PCURVE vs project_point), даёт бит-идентичный результат.

### 1.6. UV per face (`edge_cache.rs:758–845`)

`compute_uvs(edge, surface, face_id)`:

- Если есть `Curve2d` (PCURVE) → аналитически `c2d.point_at(t)`. Точнее и быстрее.
- Если NURBS без PCURVE → `surface.project_point(p)` для каждой точки (детерминированно).
- `snap_seam_uvs` (`edge_cache.rs:856–899`) — для периодических поверхностей: если UV в пределах 1% от периода, приравнивается точно к границе. Критично для seam-split логики в parametric_domain.rs.

### 1.7. Толеранс

`AdaptiveTolerance` (`edge_cache.rs:115–194`):

- `merge_tolerance()` = `model_scale × 1e-6` (1 PPM)
- `chord_tolerance()` = `model_scale × 1e-5` (10 PPM)
- `stitch_tolerance()` = `model_scale × 1e-4` (100 PPM)

`effective_chord_tolerance()` (`edge_cache.rs:589–592`) возвращает override если установлен, иначе `adaptive_tol.chord_tolerance()`.

---

## ФАЗА 2 — Триангуляция граней

### 2.1. Диспетчер поверхностей (`crates/draper-mesh/src/triangulate.rs:1508`)

`triangulate_face_impl` маршрутизрует по типу поверхности:

| Surface | Функция | Строки |
|---|---|---|
| Plane | `triangulate_planar_face` | 2047 |
| Cylinder | `triangulate_cylinder_face` | 2698 |
| Cone | `triangulate_cone_face` | 3726 |
| Sphere | `triangulate_sphere_face` | — |
| Torus | `triangulate_torus_face` | — |
| NURBS (ruled, no holes) | `try_strip_triangulation_ruled_nurbs` | 4526 |
| NURBS (general) | `triangulate_nurbs_cdt` → CDT | 5183 |

**Fallback стратегия (3 уровня, `triangulate.rs:1526–1594`)**:

1. **ApproximatePlane** — фитирование плоскости к boundary 3D-точкам, ear-clip.
2. **BoundaryFan** — fan-триангуляция от центроида.
3. **SurfacePointSample** — регулярная UV-сетка через `surface.point_at`.

Каждый fallback логирует warning. Это гарантирует, что даже патологические грани не дают пустой меш (который оставил бы дыру в BREP).

### 2.2. Планарные грани (`triangulate.rs:2047`)

**Без дыр**:
- Convex → fan-триангуляция (N−2 треугольника).
- Non-convex → `ear_clip(&points_2d)`.

**С дырами** → `earcutr_triangulate_planar` (`triangulate.rs:2179`):

1. CCW normalization (если signed area < 0 → реверс outer + holes синхронно).
2. earcutr получает `[outer_pts..., hole0_pts..., hole1_pts...]` + `hole_indices`.
3. earcutr natively сохраняет boundary edges (ear-clipping не может пропустить boundary vertex).

**ГДЕ ПОЯВЛЯЮТСЯ НОВЫЕ ТОЧКИ**: только boundary 3D-точки из кэша. **Никаких interior Steiner points** — это гарантирует watertightness.

### 2.3. Цилиндр/конус tube (`triangulate.rs:2900, 3317`)

`triangulate_cylinder_tube_from_boundary`:

1. `split_boundary_into_rings_with_u` — разделяет boundary на bottom_ring и top_ring, сортировка по `u ∈ [0, 2π)`.
2. `n_u = bottom_ring.len()` (из кэша).
3. `use_cached_top = top_ring.len() == n_u`.
4. **Если `use_cached_top`**: `n_v = 1` (только 2 ряда: bottom + top, никаких промежуточных).
5. **Генерация вершин**:
   - `j=0` (bottom): cached.
   - `j=n_v` (top, если cached): cached.
   - `0 < j < n_v` (intermediate, только если `!use_cached_top`): `cyl.point_at(u, v)` с `deterministic_round_point`. **ЗДЕСЬ ПОЯВЛЯЮТСЯ НОВЫЕ ТОЧКИ**.
6. Quads → 2 треугольника, winding по `forward`.

**Конус** дополнительно: apex degeneracy — если `v_min ≈ apex_v`, bottom row collapses в одну apex vertex (fan-triangulation вместо quad-strip).

### 2.4. Ruled NURBS strip (`triangulate.rs:4526`)

Для NURBS с `u_degree == 1 XOR v_degree == 1`, без дыр.

1. **Find 4 corners** — точки, где оба UV на границах (u_min/u_max, v_min/v_max) в пределах 1%.
2. **Split boundary into 4 edges** — walk от corner к corner.
3. **Identify rails** (2 рёбра на u_min/u_max для u_ruled, иначе v_min/v_max).
4. **RESAMPLE RAILS TO COMMON COUNT** (`triangulate.rs:4715–4995`):
   - `n_common = max(na_orig, nb_orig)`.
   - `resample(rail, n_common)` — arc-length-parametrized linear interpolation.
   - Возвращает `Vec<(Point3d, Point2d, Option<usize>)>` — `Some(orig_idx)` если точка совпала с оригинальной, `None` если интерполирована.
5. **Добавление вершин в меш**:
   - Все оригинальные boundary points добавляются как mesh vertices (для watertightness).
   - Interpolated rail points используют **LINEARLY INTERPOLATED 3D точку**, НЕ `nurbs.point_at(uv)`. Комментарий (`triangulate.rs:4966–4975`): `nurbs.point_at(uv)` даёт разные f64 bits для разных NURBS-поверхностей, разделяющих одно ребро.
6. Quads → 2 треугольника, `n_quads = n_common − 1`.
7. **BOUNDARY EDGE ENFORCEMENT** (`triangulate.rs:5097–5181`) — gap fill:
   - Для каждой consecutive пары boundary points проверяет, есть ли ребро в `mesh_edges`.
   - Если missing → находит ближайший interior vertex к midpoint, добавляет fill triangle `(va, vb, vc)`.

**ГДЕ ПОЯВЛЯЮТСЯ НОВЫЕ ТОЧКИ**: interpolated rail points когда `na_orig != nb_orig`. Linear interpolation 3D от cached boundary points (bit-identical между гранями). **Никаких interior Steiner points**.

### 2.5. NURBS CDT (`triangulate.rs:5183` → `parametric_domain.rs:3776`)

`triangulate_nurbs_cdt`:

1. Если ruled NURBS без дыр → пытается strip path первым.
2. Иначе → `triangulate_surface_consistent`.

`triangulate_surface_consistent` — полный CDT pipeline:

| Step | Описание | Строки |
|---|---|---|
| 0 | Validate NURBS UVs; если >50% NaN/Inf → return empty | 3826–3951 |
| 1 | `normalize_uv_polygon` для периодических поверхностей | 4001–4014 |
| 1.25 | CCW normalization (если signed area < 0 → реверс outer + 3D синхронно) | 4062–4095 |
| 1.5 | Area-ratio check: если uv_area < 0.001 × boundary_3d_area → re-project | 4041–4060 |
| 1.55 | **proactive_seam_split** для периодических: split на midpoint u, recursive depth ≤ 2 | 4097–4147 |
| 1.6 | UV self-intersection check → `try_split_at_seam` или 3D ear-clip fallback | 4149–4329 |
| 2 | Degenerate UV (zero range) → fan triangulation от центроида | 4331–4400 |
| 2.7 | >50% boundary degenerate (cone apex, sphere pole) → fan от apex | 4418–4476 |
| 3 | **Interior Steiner points generation** (см. ниже) | 4700+ |
| 4 | earcutr triangulation | ~5090 |
| 5 | Build 3D mesh: boundary из кэша напрямую, interior через `surface.point_at(uv)` с `deterministic_round_point` | 5100–5200 |
| 5.5 | **GAP FILLING**: для каждого missing boundary edge — найти common neighbor, проверить centroid ∈ domain | 5202–5455 |
| 6 | **chord-error refinement**: NURBS — 0 iterations (комментарий: новые interior vertices не бит-идентичны). Other curved — 2 iterations | 5457–5510 |

### 2.6. Newton-Raphson re-projection (`parametric_domain.rs:319`)

`reproject_nurbs_point(nurbs, point, init_u, init_v)`:

- 15 итераций Newton-Raphson.
- Вычисляет `derivatives_at(best_u, best_v)` (point + du + dv + 2nd derivatives).
- Гессиан: `hu_u, hu_v, hv_v` — матрица 2×2.
- Шаг ограничен 10% диапазона параметров (clamp).
- Break если improvement < 1e-12 × best_dist (convergence).
- Break если new_dist > best_dist (divergence).

### 2.7. Interior Steiner points (`parametric_domain.rs:1667`)

`generate_nurbs_interior_points(domain, u_knots, v_knots, n_sub)`:

1. Фильтрует knots в range `(u_min, u_max)`.
2. Для каждого knot span `[span_lo, span_hi]`: midpoint или `j/n_sub` fractions.
3. Декартово произведение `u_grid × v_grid`.
4. **STRICT INTERIOR check**:
   - `domain.contains(pt)` — внутри полигона.
   - `!is_point_on_boundary(domain.outer_boundary, pt, tol)` — не на outer boundary.
   - `!domain.holes.iter().any(|h| is_point_on_boundary(h, pt, tol))` — не на hole boundary.

Комментарий (`parametric_domain.rs:1670–1681`): phantom Steiner points на boundary создают T-junctions с соседней planar гранью.

### 2.8. Boundary 3D points used directly (`parametric_domain.rs:5100–5200`)

При построении mesh:

- Если `idx < n_boundary_and_holes_actual` → boundary/hole vertex: берётся `all_boundary_3d[idx_usize]` напрямую из кэша.
- Иначе → interior vertex: `deterministic_round_point(surface.point_at(uv.u, uv.v))`.
- Для NURBS используется `nurbs.derivatives_at(uv.u, uv.v)` (87 de Boor iterations) — получает и point, и normal одним вызовом.
- Position-based dedup `position_map: HashMap<[u64; 3], u32>` — две UV indices с одинаковой 3D позицией получают один mesh vertex.
- Пропуск position-degenerate triangles: если `ab < 1e-20 || bc < 1e-20 || ac < 1e-20` — пропускается, иначе создаст phantom edges.

---

## ФАЗА 3 — Сборка BREP

### 3.1. Face loop (`converter.rs:4053–4107`)

```rust
let merge_tol = (tol_ctx.model_scale * 5e-3).max(0.005);
let mut dedup_map = VertexDedupMap::with_tolerance(merge_tol);
for face_data in face_data_list {
    let face_mesh = self.surface_to_mesh_cached(face_data, params, bbox, &mut edge_cache);
    mesh.merge_deduplicating(&face_mesh, &mut dedup_map);
}
```

`merge_deduplicating` (`crates/draper-mesh/src/mesh.rs:812`):

- Для каждой вершины other: если уже есть в `dedup_map` (бит-идентично или в tolerance) → reuse index; иначе добавить.
- **Cross-face duplicate triangles (одинаковые vertex indices из разных граней) — KEEP**, чтобы не было дыр.
- Same-face duplicates — remove.

### 3.2. Пути сборки

- `triangulate_brep` (`converter.rs:3870`) — non-detailed path.
- `triangulate_brep_detailed` (`converter.rs:4326`) — с `FaceInfo` и `triangle_face_ids`.
- `BrepSession::finalize` (`converter.rs:1763`) — chunked/WASM path.

### 3.3. Пост-обработка (порядок критичен)

| Шаг | Функция | Толеранс | Назначение |
|---|---|---|---|
| 1 | `filter_degenerate_triangles` | 1e-10 | Удалить zero-area, NaN/Inf, collapsed-index |
| 2 | `merge_deduplicating` (per-face) | model_scale × 5e-3 | Vertex dedup при накоплении |
| 3 | `weld_boundary_edge_vertices` | model_scale × 1e-4 | Сшить short boundary edges (FP drift) |
| 4 | `remove_duplicate_triangles` | — | Face-aware dedup (cross-face KEEP) |
| 5 | `repair_t_junctions` | model_scale × 1e-9 | CDT-style edge splitting |
| 6 | `remove_duplicate_triangles` | — | Повторно, после T-junction split |
| 7 | `smooth_normals` | 0.785 rad (45°) | Сглаживание нормалей по crease angle |
| 8 | `validate_watertight` | — | Logging only (НЕ применяет repair_mesh) |

---

## ФАЗА 4 — Пост-обработка watertightness

### 4.1. `weld_boundary_edge_vertices` (`crates/draper-mesh/src/watertight.rs:615`)

#### Face-aware guard (`watertight.rs:624–689`)

- Для каждой вершины precompute `HashSet<u64>` face IDs.
- `shares_face(a, b)` — true если множества пересекаются.
- **Правило**: отказать в weld, если оба vertex разделяют ANY face ID, **UNLESS** расстояние < `pass2_tol_sq_for_pass1` (то есть FP drift, а не разные фичи).
- Защита от collapse thin annulus где `R_outer − R_inner < weld_tolerance`.

#### PASS 1 — SHORT boundary edges (length < `weld_tolerance`)

- Spatial hash с cell_size = weld_tolerance, проверяет 3×3×3 соседние ячейки.
- Только candidates из `boundary_vertices` (НЕ interior!).
- Сравнивает расстояние, выбирает ближайшего.
- Apply через union-find: `parent[root_v1] = root_target`.

#### PASS 2 — boundary vertices на LONG edges

- `pass2_tolerance = min(weld_tolerance × 0.01, 1e-3)` — гораздо жёстче.
- Отдельный spatial hash с cell_size = pass2_tolerance.
- Skip vertices already processed in PASS 1.

#### PASS 3 — seam-specific

- `pass3_tolerance = min(weld_tolerance × 0.1, 0.01).max(1e-5)` — промежуточный.
- Только для вершин, ещё не сваренных после PASS 1+2.
- Ловит seam mismatches на периодических поверхностях (u=0 vs u=2π).

#### Apply welds (`watertight.rs:1137–1228`)

- `root_map[v] = find(parent, v)`.
- Remap всех triangle indices.
- Filter degenerate triangles (a==b, b==c, a==c).
- Same-face duplicate triangles — remove. Cross-face duplicates — KEEP.
- `compact_vertices(mesh)` — удалить неиспользуемые вершины.

### 4.2. `repair_t_junctions` (`watertight.rs:1263`) — CDT с incremental insertion

Итеративный (max 8 iterations, skip если > 500k vertices):

1. **Build edge→triangle map** (`watertight.rs:1284–1293`).
2. **Spatial hash** с cell_size = 4 × tolerance (`watertight.rs:1294–1302`).
3. **Find T-junctions** (`watertight.rs:1307–1403`): для каждого edge (a, b), ищет vertices в соседних ячейках, лежащие на segment от a до b (`point_on_segment_3d`). Параметр `t = (ap · ab) / |ab|²` должен быть в `(1e-9, 1−1e-9)` (строго interior ребра). Для очень длинных рёбер (> 8000 ячеек) — linear scan fallback.
4. **Group splits by triangle** (`watertight.rs:1422–1464`) — критическая оптимизация. Если у triangle 2+ T-junctions на разных рёбрах, обрабатывается ONE раз, а не несколько.
5. **`incremental_insert_t_junctions`** (`watertight.rs:1583`) — re-triangulation affected triangle:
   - Start with `[A, B, C]`.
   - Для каждой T-junction vertex: найти треугольник с этим ребром → split на 2.
   - Гарантирует, что **новые T-junctions не создаются** (в отличие от ear-clipping или fan).
6. **Apply** (`watertight.rs:1529–1568`): rebuild triangle list, keep unaffected + new.
7. Повторять до конвергенции или max_iterations.
8. `compact_vertices`, `filter_degenerate_triangles_in_place`.

**Только при `non_manifold_edge_count > 0`** — иначе не запускается.

### 4.3. `validate_watertight` (`watertight.rs:373`) — метрики

`WatertightReport`:

- `interior_edge_count` — рёбра с count == 2 (хорошие).
- `boundary_edge_count` — count == 1 (gap).
- `non_manifold_edge_count` — count > 2 (self-intersection или T-junction).
- `euler_characteristic = V − E + F`.
- `degenerate_triangle_count`, `duplicate_triangle_count`.
- `per_face_summary` — если `triangle_face_ids` доступны.

`is_watertight()` = `boundary_edge_count == 0 && non_manifold_edge_count == 0`.

Дополнительно: `validate_edge_consistency` (`watertight.rs:146`) — проверяет, что interior edges используют бит-идентичные vertex indices. Реальная диагностика edge cache: если boundary edges > 1% — edge cache работает неправильно.

---

## Качество (LOD/adaptive)

### `TriangulationParams` (`triangulate.rs:474`)

| Поле | Default | LOD-зависимость |
|---|---|---|
| `max_deviation` | 0.01 | `0.01 / (lod²).min(1.0)` |
| `angular_samples` | 32 | `(6.0 + 26.0 × lod).round()` |
| `max_face_triangles` | 8000 | `(100.0 + 7900.0 × lod).round()` |
| `detail_level` | 1.0 | `0.25 + 0.75 × lod` |
| `keep_ratio` | 1.0 | `0.05 + (lod / 0.75) × 0.95` для lod<0.75 |

`TOTAL_TRIANGLE_BUDGET = 100_000` (`triangulate.rs:734`) — для LOD 1.0.

### `SteinerBudgetProfile` (`triangulate.rs:121`)

| Профиль | max_u_cyl | max_v_cyl | max_uv_plane | min_u_cyl |
|---|---|---|---|---|
| Desktop | 96 | 64 | 64 | 12 |
| Tablet | 64 | 32 | 48 | 10 |
| Mobile | 32 | 16 | 32 | 8 |

Специальные методы для sphere, torus, revolution, extrusion (`triangulate.rs:220–410`). Например, `min_u_torus() = 24` для desktop — иначе fillet выглядит гранёным.

### `face_area_budget_multiplier` (`triangulate.rs:190`)

- `fraction < 0.01` → 0.5× (маленькая грань, минимум треугольников)
- `0.01 ≤ fraction < 0.25` → linear 0.5 → 1.0
- `fraction ≥ 0.25` → linear 1.0 → 2.0 (большая грань)

### Как LOD влияет на density

- **Edge sampling**: `set_chord_tolerance_override(Some(params.max_deviation))` → `adaptive_discretize` использует LOD-driven tolerance для subdivision NURBS curves.
- **Steiner points**: `max_face_triangles` cap'ит общее число треугольников; `steiner_profile` cap'ит n_u, n_v.
- **Face budgets**: `face_area_budget_multiplier` адаптирует budget под относительный размер грани (маленькие грани получают 0.5×, большие до 2.0×).
- **Decimation**: `keep_ratio < 1.0` → `decimate_mesh` post-process. Skip если `adaptive_lod_enabled` (each face уже respect budget).

---

## Карбирование (резервирование) рёбер

### Constraint edges в CDT (`crates/draper-mesh/src/custom_cdt.rs:541`)

`build_constraint_set(n_boundary, hole_ranges)` строит `HashSet<(u32, u32)>`:

- Outer boundary: `(i, i+1 mod n)` для `i ∈ 0..n_boundary`.
- Holes: `(start+i, start+(i+1 mod n_hole))` для каждой дыры.

`verify_constraints` (debug-only, `custom_cdt.rs:620`) проверяет после триангуляции, что все constraint edges присутствуют в `tri_edges`.

**Реальный CDT**: earcutr natively сохраняет boundary edges (получает их как вход + hole_indices), поэтому custom constraint insertion не используется в production path. Custom CDT functions (`insert_point_in_triangle`, `insert_point_on_edge`) существуют, но используются только в test-modes.

### Gap-fill механизмы

| Механизм | Где | Что делает |
|---|---|---|
| Strip BOUNDARY EDGE ENFORCEMENT | `triangulate.rs:5097–5181` | После strip: для missing boundary edge находит ближайший interior vertex, добавляет fill triangle |
| GAP FILLING (CDT) | `parametric_domain.rs:5202–5455` | После earcutr: для missing boundary edge находит common neighbor, проверяет centroid ∈ domain |
| `weld_boundary_edge_vertices` PASS 1/2/3 | `watertight.rs:615–1232` | Post-hoc сшивка FP drift |
| `repair_t_junctions` | `watertight.rs:1263` | CDT-style edge splitting для T-junctions |
| `merge_deduplicating` cross-face keep | `mesh.rs:812` | Сохраняет cross-face duplicate triangles (НЕ удаляет) |
| `remove_duplicate_triangles` face-aware | `mesh.rs:297` | Удаляет только same-face duplicates, cross-face keep |

### Inner loops (дыры) для planar_face_with_holes

`triangulate_planar_face` (`triangulate.rs:2047`) с `holes_3d.is_empty() == false`:

1. Делегирует в `earcutr_triangulate_planar` (`triangulate.rs:2120`).
2. Строит flat coordinates: `[outer_pts..., hole0_pts..., hole1_pts...]`.
3. `hole_indices: Vec<usize>` — start index каждой дыры.
4. CCW normalization (если signed area < 0 → реверс outer + holes).
5. earcutr triangulation.
6. **Fallback**: `merge_holes_into_polygon_planar` — bridge-edge + ear-clip + filter по попаданию центроида в полигон.

В `triangulate_surface_consistent`:
- Holes передаются как `hole_polylines_3d` и `hole_uvs`.
- `domain = ParametricDomain::new(outer_uv, ...).with_holes_from(normalized_holes_uv)`.
- `domain.init_containment_grid()` — для быстрых containment checks.
- В GAP FILLING: для каждого missing boundary edge проверяет centroid внутри domain (через `domain.contains_ray(&centroid_uv)`) — если внутри дыры, fill triangle не добавляется.

---

## Сводная таблица: где появляются новые точки

| Путь | Interior Steiner | Analytic intermediate | Boundary from cache |
|---|---|---|---|
| Planar (no holes) | ✗ | ✗ | ✓ |
| Planar (with holes) | ✗ | ✗ | ✓ (outer + holes) |
| Cylinder tube (cached top) | ✗ | ✗ | ✓ (bottom + top) |
| Cylinder tube (no cached top) | ✗ | ✓ (intermediate rows `cyl.point_at`) | ✓ (bottom) |
| Cone tube | ✓ (apex, 1 шт.) | ✓ (intermediate rows) | ✓ |
| Ruled NURBS strip | ✓ (interpolated rail, linear 3D) | ✗ | ✓ (corners + rail endpoints) |
| NURBS CDT | ✓ (knot-span Steiner) | ✗ | ✓ (boundary 3D directly) |
| CDT chord-refine | ✓ (interior midpoint, non-NURBS only) | ✗ | ✓ (boundary edges never split) |

**Watertightness invariant**: все boundary 3D-точки берутся напрямую из `EdgeDiscretizationCache` (с `deterministic_round_point` — 48-bit mantissa). Interior points вычисляются per-face и НЕ обязаны быть бит-идентичны между гранями — но они никогда не лежат на shared edges.

---

## Ключевые файлы

| Файл | Назначение | Строк |
|---|---|---|
| `crates/draper-mesh/src/edge_cache.rs` | Кэш рёбер, aliasing, discretization | 1659 |
| `crates/draper-mesh/src/triangulate.rs` | Диспетчер поверхностей, planar/cylinder/cone/strip | 10314 |
| `crates/draper-mesh/src/parametric_domain.rs` | NURBS CDT, Newton-Raphson, gap fill | 8822 |
| `crates/draper-mesh/src/watertight.rs` | Weld, T-junction repair, validation | 2352 |
| `crates/draper-mesh/src/custom_cdt.rs` | Constraint edges (debug) | 741 |
| `crates/draper-mesh/src/mesh.rs` | merge_deduplicating, remove_duplicate_triangles | — |
| `crates/draper-step/src/converter.rs` | BREP assembly, post-processing pipeline | 15299 |

---

## Известные ограничения

1. **Оставшиеся 4 boundary edges на болте** (as1-oc-214.stp BREP#1190) — это **реальные missing triangles**, не T-junctions. Требуют gap-filling алгоритма, который ещё не реализован.

2. **NURBS chord-error refinement = 0 iterations** — чтобы не создавать non-bit-identical interior vertices. Качество NURBS-граней зависит только от Steiner budget, не от iterative refinement.

3. **Large meshes (> 500K vertices)** — `repair_t_junctions` пропускается (performance guard).

4. **Composite curves** — дедупликация стыков сегментов может пропускать близкие, но не идентичные точки.
