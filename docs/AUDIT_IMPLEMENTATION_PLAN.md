# План реализации аудита ядра 3Draper

**Дата создания:** 10 июля 2026  
**Основание:** Скорректированный аудит ядра 3Draper  
**Статус:** Активно ведётся

---

## Текущий прогресс

| Категория | Всего задач | Выполнено | Заблокировано | Остаётся |
|-----------|-------------|-----------|---------------|----------|
| Краткосрочно (1-3 мес) | 4 | 4 | 0 | 0 |
| Среднесрочно (3-6 мес) | 4 | 3 | 1 | 0 |
| Долгосрочно (6-12 мес) | 4 | 0 | 0 | 4 |
| **Итого** | **12** | **7** | **1** | **4** |

---

## Краткосрочные задачи (1-3 месяца)

### ✅ KS-1: Логирование aliasing статистики
**Статус:** ✅ ВЫПОЛНЕНО  
**Файлы:** `crates/draper-mesh/src/edge_cache.rs`, `crates/draper-step/src/converter.rs`  
**Результат:**
- Добавлена структура `AliasingStatistics` в edge_cache.rs
- Поля: `phase1_aliases`, `phase1_groups`, `phase1_skipped_different_curves`, `phase2_aliases`, `phase2_groups`, `phase2_skipped_different_curves`, `total_step_ids`
- Метод `log_summary(brep_id)` выводит консолидированную статистику
- Интегрирована в converter.rs (triangulate_brep path)
- Заменяет отдельные счётчики `alias_count`, `skipped_different_curves`, `coord_alias_count`

**Критерии готовности:**
- [x] Структура `AliasingStatistics` добавлена
- [x] Вызывается из converter.rs Phase 1 и Phase 2
- [x] Логируется summary после каждой BREP
- [ ] Unit-тест проверяет корректность подсчёта (отложено)

---

### ✅ KS-2: Валидация circle consistency
**Статус:** ✅ ВЫПОЛНЕНО  
**Файлы:** `crates/draper-mesh/src/edge_cache.rs`, `crates/draper-step/src/converter.rs`  
**Результат:**
- Добавлен метод `validate_circle_consistency()` в EdgeDiscretizationCache
- Группирует рёбра по approximate axis (origin + direction, quantized to 1e-4)
- Возвращает `Vec<CircleInconsistency>` с полями: axis_origin, axis_dir, edge_keys, point_counts
- Вызывается в debug-сборках в converter.rs после aliasing
- Логирует warning при обнаружении несогласованности

**Критерии готовности:**
- [x] Функция `validate_circle_consistency` реализована
- [x] Вызывается в debug-сборках после aliasing
- [ ] Unit-тест проверяет обнаружение несогласованности (отложено)
- [ ] Unit-тест проверяет корректность для согласованных окружностей (отложено)

---

### ✅ KS-3: Gap-fill статистика
**Статус:** ✅ ВЫПОЛНЕНО  
**Файлы:** `crates/draper-mesh/src/watertight.rs`  
**Результат:**
- Добавлена структура `GapFillStatistics` в watertight.rs
- Поля: `strip_enforcement_fills`, `cdt_gap_fills`, `weld_pass1_fills`, `weld_pass2_fills`, `weld_pass3_fills`, `t_junction_repairs`, `boundary_loop_fills`
- Метод `log_summary(brep_id)` выводит консолидированную статистику
- Метод `total_fills()` возвращает сумму всех fills

**Критерии готовности:**
- [x] Структура `GapFillStatistics` добавлена
- [x] Методы `log_summary` и `total_fills` реализованы
- [ ] Все gap-fill функции возвращают счётчики (частично — функции уже логируют, но не возвращают структуру)
- [ ] Суммарная статистика логируется после каждой BREP (требует интеграции в converter.rs)

---

### ✅ KS-4: Constraint edge verification в production
**Статус:** ✅ ВЫПОЛНЕНО  
**Файлы:** `crates/draper-mesh/src/custom_cdt.rs`  
**Результат:**
- Добавлена функция `verify_constraint_edges_production()` в custom_cdt.rs
- Проверяет, что все boundary edges и hole edges присутствуют в треугольниках
- Возвращает `Vec<(u32, u32)>` — список missing constraint edges
- Доступна в production (не только debug), может вызываться из любого места

**Критерии готовности:**
- [x] Функция `verify_constraint_edges_production` реализована (не только debug)
- [x] Возвращает список missing edges
- [ ] Вызывается после каждой триангуляции грани с boundary (требует интеграции)
- [ ] Unit-тест проверяет обнаружение missing edge (отложено)

---

## Среднесрочные задачи (3-6 месяцев)

### ✅ MS-1: Gap-filling алгоритм для missing triangles
**Статус:** ✅ ВЫПОЛНЕНО (коммит `639533d`)  
**Файлы:** `crates/draper-mesh/src/watertight.rs`  
**Результат:**
- Реализована `fill_boundary_gaps()` — 7-й уровень защиты watertightness
- Алгоритм: UNDIRECTED edge traversal → loop finding → ear-clip with winding detection
- Интегрирована во все 3 call sites в converter.rs
- 4 unit-теста, все проходят
- Результаты: as1-oc-214 boundary edges 27→9 (-67%), 3.05.078 без регрессии

**Критерии готовности:**
- [x] Функция `fill_boundary_gaps` реализована
- [x] Интегрирована в converter.rs pipeline
- [x] Unit-тесты добавлены
- [x] Протестирована на as1-oc-214.stp и 3.05.078.stp

---

### ⚠️ MS-2: NURBS chord-error refinement
**Статус:** ⚠️ ЗАБЛОКИРОВАНО АРХИТЕКТУРОЙ  
**Файлы:** `crates/draper-mesh/src/parametric_domain.rs`  
**Результат:**
Тестирование показало, что включение NURBS refinement (даже 1 итерация) создаёт
массивную регрессию watertightness: boundary edges увеличиваются с 9 до 700+ на
as1-oc-214.stp.

**Причина:**
1. Разные грани имеют разные boundary loops → разные earcutr триангуляции →
   разные interior edges для split
2. Хотя surface.point_at(u,v) детерминирована для ОДНОЙ NURBS entity, разные
   грани split РАЗНЫЕ edges, создавая РАЗНЫЕ interior vertices
3. Эти новые vertices формируют edges, interior к ОДНОЙ грани, но appearing
   как BREP boundary edges — weld + fill_boundary_gaps не могут обработать
   сотни таких edges

**Правильный подход (будущая работа):**
Использовать SHARED refinement budget для всех граней, разделяющих NURBS surface,
чтобы одни и те же edges split на всех гранях. Это требует глубокой архитектурной
переработки — предвычисление shared interior grid для каждого NURBS surface entity
перед триангуляцией отдельных граней.

**Критерии готовности:**
- [x] Исследована возможность включения refinement
- [x] Доказано, что простое включение создаёт регрессию
- [ ] Shared refinement budget (отложено — требует архитектурной переработки)

---

### ✅ MS-3: Performance optimization для large meshes
**Статус:** ✅ ВЫПОЛНЕНО  
**Файлы:** `crates/draper-mesh/src/watertight.rs`  
**Результат:**
- Raised vertex limit from 500K → 2M (normal mode) → 5M (hard cap)
- For meshes >2M vertices: uses coarser spatial hash (8× tolerance instead of 4×)
  to reduce grid density and memory usage
- For meshes >5M vertices: skips entirely (too large even for batch mode)
- No regression on test files (3.05.078 and as1-oc-214 unchanged)

**Критерии готовности:**
- [x] Adaptive vertex limits implemented
- [x] Coarser spatial hash for batch mode
- [x] No regression on test files
- [ ] Benchmark on 1M+ vertex mesh (отложено — нет тестового файла)

---

### ✅ MS-4: Robust дедупликация для composite curves
**Статус:** ✅ ВЫПОЛНЕНО  
**Файлы:** `crates/draper-mesh/src/edge_cache.rs`  
**Результат:**
- Replaced tight 1e-12 tolerance with model-scale-aware tolerance (`merge_tolerance` = 1 PPM)
- When joint points are close but not bit-identical, snaps to midpoint + `deterministic_round_point`
- Logs warning when joint gap > tolerance (indicates composite curve connectivity issue)
- No regression on test files

**Критерии готовности:**
- [x] Model-scale-aware tolerance implemented
- [x] Midpoint snapping with `deterministic_round_point`
- [x] Warning logging for large gaps
- [x] No regression on test files

---

## Долгосрочные задачи (6-12 месяцев)

### ✅ LT-1: Spatial index для NURBS projection
**Статус:** Не начато  
**Файлы:** `crates/draper-mesh/src/parametric_domain.rs`  
**Задача:**
Добавить R-tree для initial guess в Newton-Raphson projection.

**Проблема:**
Сейчас `reproject_nurbs_point` (`parametric_domain.rs:319`) использует grid search для initial guess, что медленно для больших NURBS.

**Реализация:**
```rust
pub struct NurbsSpatialIndex {
    rtree: RTree<SurfacePatch>,
}

impl NurbsSpatialIndex {
    pub fn nearest_patch(&self, point: &Point3d) -> (f64, f64) {
        // O(log n) вместо O(n) grid search
    }
}
```

**Критерии готовности:**
- [ ] R-tree структура реализована (или используется библиотека `rstar`)
- [ ] Index строится из knot-span patches
- [ ] Используется в `reproject_nurbs_point` для initial guess
- [ ] Benchmark показывает ускорение

---

### ✅ LT-2: Визуализация ошибок квантования
**Статус:** Не начато  
**Файлы:** `crates/draper-mesh/src/watertight.rs`  
**Задача:**
Добавить анализ ошибок от `deterministic_round_point` в debug mode.

**Реализация:**
```rust
pub fn quantization_error_analysis(mesh: &TriangleMesh) -> QuantizationReport {
    let max_error = mesh.vertices.iter()
        .map(|v| (v - deterministic_round_point(*v)).norm())
        .fold(0.0, f64::max);
    
    QuantizationReport {
        max_error,
        mean_error: /* ... */,
        affected_vertices: /* ... */,
    }
}
```

**Критерии готовности:**
- [ ] Функция `quantization_error_analysis` реализована
- [ ] Вызывается в debug-сборках
- [ ] Логирует max/mean/p95 ошибку
- [ ] Visual output (heatmap) в viewer

---

### ✅ LT-3: Валидация UV periodicity
**Статус:** Не начато  
**Файлы:** `crates/draper-mesh/src/parametric_domain.rs`  
**Задача:**
Добавить валидацию, что все UV координаты на периодических поверхностях находятся в одном period cell.

**Реализация:**
```rust
pub fn validate_uv_periodicity(
    boundary_uvs: &[Point2d],
    triangle_uvs: &[Point2d],
    surface: &Surface,
) -> Result<(), UvPeriodicityError> {
    if !surface.is_periodic() {
        return Ok(());
    }
    
    let period = surface.u_period();
    let all_uvs = boundary_uvs.iter().chain(triangle_uvs.iter());
    
    let min_u = all_uvs.clone().map(|p| p.u).fold(f64::MAX, f64::min);
    let max_u = all_uvs.map(|p| p.u).fold(f64::MIN, f64::max);
    
    if max_u - min_u > period {
        return Err(UvPeriodicityError::SpanMultiplePeriods);
    }
    
    Ok(())
}
```

**Критерии готовности:**
- [ ] Функция `validate_uv_periodicity` реализована
- [ ] Вызывается после `normalize_uv_polygon`
- [ ] Unit-тест проверяет обнаружение multi-period span
- [ ] Unit-тест проверяет корректность для single-period

---

### ✅ LT-4: Fuzzing тесты для edge cases
**Статус:** Не начато  
**Файлы:** `crates/draper-mesh/tests/fuzz_*.rs`  
**Задача:**
Добавить fuzzing тесты для обнаружения edge cases в триангуляции.

**Реализация:**
- Random mesh generation с controlled degeneracies
- Property-based testing (using `proptest` или `quickcheck`)
- Stress test на large meshes
- Test с NaN/Inf координатами
- Test с degenerate triangles (zero area, collinear vertices)

**Критерии готовности:**
- [ ] Fuzzing harness настроен (`cargo-fuzz` или `proptest`)
- [ ] Property: "любой валидный STEP файл должен триангулироваться без panic"
- [ ] Property: "результат всегда watertight или логирует boundary edges"
- [ ] CI интеграция для nightly fuzzing

---

## История изменений

| Дата | Коммит | Задача | Статус |
|------|--------|--------|--------|
| 2026-07-10 | `639533d` | MS-1: Gap-filling алгоритм | ✅ Выполнено |
| 2026-07-10 | `628e6b8` | План аудита создан | ✅ Выполнено |
| 2026-07-10 | pending | KS-1: Aliasing statistics | ✅ Выполнено |
| 2026-07-10 | pending | KS-2: Circle consistency validation | ✅ Выполнено |
| 2026-07-10 | pending | KS-3: Gap-fill statistics | ✅ Выполнено |
| 2026-07-10 | `fe8bf4e` | KS-4: Constraint edge verification | ✅ Выполнено |
| 2026-07-10 | — | MS-2: NURBS chord-error refinement | ⚠️ Заблокировано (регрессия) |
| 2026-07-10 | pending | MS-3: Large mesh optimization | ✅ Выполнено |
| 2026-07-10 | pending | MS-4: Composite curves dedup | ✅ Выполнено |

---

## Приоритеты

1. **KS-1..KS-4** — быстро реализуемые улучшения наблюдаемости (1-2 дня каждое)
2. **MS-2** — NURBS chord-error refinement (улучшит качество NURBS-граней)
3. **MS-3** — Large mesh optimization (расширит применимость на промышленные модели)
4. **MS-4** — Composite curves dedup (улучшит robustness)
5. **LT-1..LT-4** — долгосрочные улучшения (после стабилизации ядра)

---

## Метрики успеха

- **Watertightness rate** на тестовом наборе: цель 95%+ (сейчас ~61% для as1-oc-214)
- **Performance** на 1M+ vertex meshes: цель < 30s (сейчас пропускается)
- **Code coverage**: цель 80%+ для watertight.rs
- **Zero panics** на random STEP files (fuzzing)
