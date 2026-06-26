# 3Draper — Универсальный план поддержки всех типов STEP и любых поверхностей с отверстиями

> **Версия документа:** 1.0
> **Дата создания:** 2026-06-26
> **Владелец:** KernelDev / 3Draper
> **Главная задача:** Открывать **все** типы STEP-файлов, поддерживать **все** кривые и поверхности, обрабатывать файлы **любой сложности** — быстро и качественно.
> **Принцип работы:** отметь пункт `[x]` при завершении; `[~]` — в процессе; `[ ]` — не начато; `[!]` — блокировка.

---

## Легенда статусов

| Символ | Значение |
|--------|----------|
| `[ ]` | Не начато |
| `[~]` | В процессе |
| `[x]` | Выполнено |
| `[!]` | Блокировка / требует обсуждения |
| `[−]` | Отложено |

---

## 0. Контекст и постановка проблемы

### 0.1 Текущее состояние (июнь 2026)

3Draper уже способен загружать большинство STEP-файлов из тестового корпуса (24/24 файлов проходят бенчмарк `benchmark_baseline.csv`). Однако пользовательские сценарии на реальных промышленных сборках (drill_top.stp, 8500-02_Vulcan.STEP, transmission_top.stp) выявили три критических проблемы:

1. **Качество триангуляции упало** — после введения `Steiner grid caps` (commit `276735c`) и агрессивного mobile-LOD downgrade (Ultra→Medium, High→Low) сетки выглядят разреженными, особенно на криволинейных гранях с отверстиями.
2. **Прогрессивная триангуляция таймаутит** — при 120с mobile timeout один из 5 BREP drill_top.stp не успевает завершиться и полностью исчезает из результата ("исчез один элемент сборки").
3. **Триангуляция остаётся негерметичной** — 22807 вершин / 29756 треугольников на drill_top.stp (LOD 0.30) дают 0.64% boundary edges, но визуально видны щели на сложных гранях.

### 0.2 Что нужно достичь

| Критерий | Целевое значение |
|---|---|
| Все 24 тестовых файла открываются | 100% |
| Watertight (0 boundary edges) на простых деталях | 100% |
| Watertight на сборках (drill_top, Vulcan, transmission) | ≥ 95% граней |
| Время загрузки на desktop (4 ядра, 16 ГБ) | < 30s для drill_top.stp |
| Время загрузки на мобильном (Snapdragon 8 Gen 2) | < 90s для drill_top.stp |
| UI отзывчив во время загрузки | Да (Repaint каждые 500 мс) |
| Поддержка типов поверхностей | Plane, Cylinder, Cone, Sphere, Torus, Revolution, Extrusion, NURBS, BSpline-with-knots, Bezier, Offset, Swept, RectangularTrimmed, BoundedSurface |
| Поддержка типов кривых | Line, Circle, Ellipse, Hyperbola, Parabola, Arc, BSplineCurve, BezierCurve, Polyline, TrimmedCurve, CompositeCurve, CompositeCurveOnSurface, OffsetCurve3D, PCurve |
| Поддержка отверстий | Любое количество внутренних контуров на грани любого типа поверхности |

---

## 1. Фаза 0 — Стабилизация (немедленные приоритеты, 1–2 недели)

> **Цель:** Устранить регрессии качества, исправить исчезновение элементов сборки, восстановить доверие к триангуляции.

### 1.1 Восстановить качество триангуляции после Steiner grid caps

**Проблема:** Commit `276735c` ограничил `n_u ≤ 64, n_v ≤ 32` для `generate_cylinder_or_cone_steiner_grid` и `n_u/n_v ≤ 32` для `generate_planar_steiner_grid`. Это сократило количество кандидатов в 2–4 раза, но также вдвое снизило визуальную плотность сетки на криволинейных гранях. Пользователь жалуется: "скорость лучше но сильно хуже чем было раньше".

**Зачем:** Скорость без качества бесполезна — пользователь предпочёл бы подождать на 10 секунд дольше, но получить чистую сетку. Текущие caps слишком агрессивны для desktop; на mobile их нужно применять точечно, а не глобально.

**Задачи:**

- [x] 1.1.1 Ввести `SteinerBudgetProfile` enum с тремя профилями: `Desktop { max_u: 96, max_v: 64 }`, `Tablet { max_u: 64, max_v: 32 }`, `Mobile { max_u: 32, max_v: 16 }`. Профиль выбирается в viewer по `is_mobile` и `screen_width`.
- [x] 1.1.2 Передавать `SteinerBudgetProfile` через `TriangulationParams` (новое поле `steiner_profile: SteinerBudgetProfile`). Обновить сигнатуры `generate_planar_steiner_grid`, `generate_cylinder_or_cone_steiner_grid` так, чтобы они принимали профиль вместо хардкода.
- [x] 1.1.3 Вернуть desktop-плотность к значениям до `276735c`: `max_u: 96, max_v: 64` для цилиндра/конуса, `max_u: 64, max_v: 64` для плоскости. На mobile сохранить `32/16` (оправдано медленным CPU).
- [ ] 1.1.4 Заменить хардкоженный budget cap `1.5× max_budget` на адаптивный: если грань имеет площадь < 1% bbox детали → `0.5× budget`; если > 25% → `2× budget`. Это позволяет крупным граням получать больше Steiner-точек без переполнения бюджета мелких. **Частично:** добавлен `candidate_multiplier()` (Desktop 2.0, Tablet 1.5, Mobile 1.25), но per-face-area adaptive budget не реализован — отложено в Phase 1.
- [ ] 1.1.5 Добавить unit-тест: цилиндр R=10, H=50, с 3 отверстиями — проверка что `n_u × n_v ≥ 48 × 24` на desktop-профиле, и `≥ 16 × 8` на mobile-профиле.
- [ ] 1.1.6 Визуальный тест (manual): загрузить `test/nist_cylinder.stp` и `test/brick_thin_hole.stp` на desktop + mobile, сравнить качество сетки с состоянием до `276735c`.

**Критерий приёмки:** На desktop качество сетки цилиндра/плоскости с отверстиями визуально идентично состоянию до `276735c`, а на mobile остаётся приемлемым (видны очертания отверстий, нет "провалов").

---

### 1.2 Исправить исчезновение элемента сборки при timeout

**Проблема:** В `crates/draper-viewer/src/app.rs:4085` при глобальном timeout (120с mobile / 300с desktop) код вызывает `self.is_loading = false` и показывает `self.mesh.clone()` — но если BREP ещё в `pending_breps[0]` и его chunked session ещё активна, частичный результат этого BREP **не попадает** в `self.mesh`. Пользователь видит: 4 из 5 элементов сборки + пустое место там, где должен быть 5-й.

**Зачем:** "Исчез один элемент сборки" — это самая заметная регрессия для пользователя. Даже если BREP не завершился, его частичная триангуляция (несколько сотен граней из тысяч) должна быть видна, а не потеряна.

**Задачи:**

- [x] 1.2.1 В `crates/draper-step/src/converter.rs` добавить метод `OwnedStepConversionContext::take_partial_active_session(&mut self) -> Option<(TriangleMesh, Vec<FaceInfo>)>` — извлекает текущий `active_session`, вызывает `finalize` на нём, возвращает частичный mesh + face infos. Если session пуста — возвращает `None`. **Реализовано** с сигнатурой `take_partial_active_session(&mut self, pending: &PendingBrepInstance) -> Option<(TriangleMesh, Vec<FaceInfo>, usize, usize)>` — дополнительно применяет transform и decimation из pending instance.
- [x] 1.2.2 В `app.rs:process_pending_breps` перед вызовом `self.is_loading = false` при timeout добавить: если есть активная session → вызвать `take_partial_active_session`, смерджить partial mesh в `self.mesh`, увеличить `self.triangulated_count`, записать warning в лог "BREP #X partial: only Y of Z faces triangulated".
- [x] 1.2.3 Если partial mesh пуст (0 треугольников) — добавить placeholder-маркер в лог: "BREP #X produced 0 triangles (face time limit too tight?)". Это поможет диагностировать, когда проблема не в timeout, а в самой триангуляции.
- [~] 1.2.4 В UI: при показе "Loading timed out after Xs" добавить строку "Partial: Y of Z instances loaded, N faces triangulated". Это информирует пользователя, что частичный результат — это намеренно. **Частично:** warning логируется в консоль, но отдельная UI-строка в баннере не добавлена — отложено.
- [ ] 1.2.5 Тест: симулировать timeout в `draper-testing` — загрузить drill_top.stp, искусственно установить BREP time limit = 1с, проверить что все 5 элементов появляются (4 полных + 1 partial).

**Критерий приёмки:** При любом timeout (mobile 120с, desktop 300с, или ручной Cancel) `self.mesh` содержит все элементы, которые успели начаться, даже если некоторые — частичные.

---

### 1.3 Пересмотреть mobile LOD downgrade стратегию

**Проблема:** Текущая стратегия (commit `4a843f6`) даёт: Ultra→Medium, High→Low, Medium→Low. Это означает что пользователь с Ultra-качеством на мобильном получает LOD 0.30 — что эквивалентно ~10% от полной сетки. Сравните с desktop-LOD 1.0 — разница в 10 раз. Это слишком агрессивно.

**Зачем:** Пользователь жалуется "сильно хуже чем было раньше" — именно из-за этого downgrade. Нужно найти баланс: mobile не должен зависать, но и не должен выглядеть как "low-poly preview".

**Задачи:**

- [x] 1.3.1 Уменьшить агрессивность downgrade: Ultra→High (вместо Medium), High→Medium (вместо Low), Medium→остаётся Medium. LOD 0.50 вместо 0.30.
- [x] 1.3.2 Добавить эвристику по сложности файла: если BREP count ≤ 2 и faces ≤ 500 → не понижать LOD (мобильный CPU справится). Если > 2000 faces → понижать на 2 ступени (Ultra→Medium). Это избегает излишнего downgrade для простых деталей. **Реализовано** через `pending.len() * 500 > 2000` (5+ BREP / 2500+ faces proxy); `PendingBrepInstance::face_count_estimate` поле добавлено, но пока всегда None (BREP count как proxy).
- [ ] 1.3.3 Сохранить выбранное mobile LOD в `localStorage` — чтобы при следующей загрузке пользователь не видел "качество скачет". Ключ: `3draper_mobile_lod`.
- [ ] 1.3.4 В UI явно показывать "Quality: Medium (auto-downgraded from Ultra for mobile)" — чтобы пользователь понимал причину и мог поднять вручную.
- [ ] 1.3.5 Тест: загрузить `nist_cylinder.stp` на mobile — должно завершиться за < 15с при LOD 0.5, без freeze. Загрузить `drill_top.stp` — должно завершиться за < 90с при LOD 0.5 (с chunking).

**Критерий приёмки:** Mobile LOD downgrade активируется только для действительно больших файлов (> 2000 faces), и downgrade на одну ступень вместо двух.

---

## 2. Фаза 1 — Универсальная поддержка поверхностей с отверстиями (2–4 недели)

> **Цель:** Каждый тип поверхности должен корректно обрабатывать любое количество отверстий (внутренних контуров), используя dedicated Steiner grid + earcutr.

### 2.1 Аудит текущего состояния по типам поверхностей

| Surface type | Has dedicated Steiner grid? | Handles holes? | Статус |
|---|---|---|---|
| Plane | ✅ `generate_planar_steiner_grid` | ✅ Да | OK |
| Cylinder | ✅ `generate_cylinder_or_cone_steiner_grid` | ✅ Да | OK |
| Cone | ✅ `generate_cylinder_or_cone_steiner_grid` | ✅ Да | OK |
| Sphere | ❌ Только generic `parameter_division_2d` | ⚠️ Частично | **Требует работы** |
| Torus | ❌ Только generic | ⚠️ Частично | **Требует работы** |
| Revolution | ❌ Только generic | ⚠️ Частично | **Требует работы** |
| Extrusion | ❌ Только generic | ⚠️ Частично | **Требует работы** |
| NURBS | ❌ Только generic + `triangulate_nurbs_cdt` | ⚠️ Частично | **Требует работы** |
| OffsetSurface | ❌ Аппроксимируется NURBS | ⚠️ Через NURBS | **Требует работы** |
| SweptSurface | ❌ Аппроксимируется Extrusion/NURBS | ⚠️ Через approx | **Требует работы** |
| RectangularTrimmedSurface | ✅ Делегирует basis | ✅ Через basis | OK |
| BoundedSurface (complex entity) | ✅ Через B_SPLINE_SURFACE keyword | ✅ Через NURBS | OK |

**Вывод:** 5 из 8 первичных типов поверхностей не имеют dedicated Steiner grid — они падают в generic `parameter_division_2d`, который возвращает только boundary knots без учёта отверстий. Это причина "неверной триангуляции" на сложных гранях.

---

### 2.2 Dedicated Steiner grid для Sphere

**Проблема:** Сфера параметризуется (u, v) ∈ [0, 2π] × [0, π]. Generic `parameter_division_2d` возвращает u-knots по chord-error в горизонтальном сечении, но v-knots только на границах (так как сфера линейна по v в случае partial band). При наличии отверстий earcutr получает разреженный grid и создаёт длинные треугольники через отверстия.

**Зачем:** Сферы с отверстиями — типичная деталь аэрокосмической промышленности (lubrication holes на spherical bearing). Без dedicated grid они визуально "проваливаются".

**Задачи:**

- [ ] 2.2.1 Создать функцию `generate_sphere_steiner_grid(surface: &SphereSurface, domain: &ParametricDomain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>` в `parametric_domain.rs`.
- [ ] 2.2.2 Алгоритм: n_u из chord-error tolerance (обычно 24–48), n_v — то же. Для каждого узла (i, j) вычислить UV, проверить `domain.contains` (cached grid O(1)) + `is_point_on_boundary` (только для граничных точек). Если точка валидна — добавить в результат.
- [ ] 2.2.3 Special case: pole regions (v ≈ 0 или v ≈ π) — Steiner-точки там вырождаются в одну 3D-точку. Пропускать кандидатов с `v < ε` или `v > π - ε` (ε = (v_max - v_min) × 0.001). Это уже делается в `triangulate_sphere_face`, но не в generic grid.
- [ ] 2.2.4 Special case: full sphere (v ∈ [0, π]) — добавить экваториальный ring (v = π/2) как обязательные Steiner-точки, даже если budget tight. Это предотвращает "collapsing" сферы в один полюс.
- [ ] 2.2.5 Интегрировать в `triangulate_surface_consistent`: добавить ветку `else if matches!(surface, Surface::Sphere(_))` перед generic branch, вызывающую `generate_sphere_steiner_grid`.
- [ ] 2.2.6 Тест: сфера R=10 с 4 отверстиями (квадратными 2×2) — проверить что все 4 отверстия видны как отдельные "дыры" в сетке, а не заклеены треугольниками.
- [ ] 2.2.7 Тест: `test/nist_sphere.stp` — проверить watertight и визуальное качество.

**Критерий приёмки:** `test/nist_sphere.stp` с добавленными отверстиями триангулируется с тем же качеством, что и цилиндр с отверстиями.

---

### 2.3 Dedicated Steiner grid для Torus

**Проблема:** Тор параметризуется (u, v) ∈ [0, 2π] × [0, 2π]. В drill_top.stp 90 тороидальных граней (fillets на пересечении цилиндров). Они обычно small-area, но с высокой кривизной по обоим направлениям. Generic grid даёт 4×4 или 6×6 — слишком мало для плавного fillet.

**Зачем:** Fillets — самая визуально заметная часть CAD-модели. Плохая триангуляция fillet выглядит как "гранёный" край вместо гладкого перехода.

**Задачи:**

- [ ] 2.3.1 Создать `generate_torus_steiner_grid(surface: &TorusSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`.
- [ ] 2.3.2 Алгоритм: n_u и n_v оба вычисляются по chord-error, но с минимальным порогом `min(24, n)` для каждого (иначе fillet выглядит гранёным).
- [ ] 2.3.3 Special case: partial torus (u или v < 2π) — grid ограничен实际 range, без wrap-around.
- [ ] 2.3.4 Special case: degenerate torus (minor_radius ≈ 0 → sphere-like) — делегировать в `generate_sphere_steiner_grid` с эквивалентной сферой.
- [ ] 2.3.5 Интегрировать в `triangulate_surface_consistent` аналогично 2.2.5.
- [ ] 2.3.6 Тест: тор (R=10, r=2) с цилиндрическим отверстием сквозь tube — проверить что отверстие видно.
- [ ] 2.3.7 Тест: drill_top.stp — проверить что fillet грани выглядят гладкими (отсутствие "фасеточной" структуры).

**Критерий приёмки:** Torus fillets в drill_top.stp визуально гладкие на desktop (96 Steiner points minimum per fillet).

---

### 2.4 Dedicated Steiner grid для Revolution

**Проблема:** `RevolutionSurface` parameterizes (u, v) ∈ [0, 2π] × [0, 1], где u — угол вращения, v — параметр вдоль profile curve. Generic grid даёт u-knots по chord-error, но v-knots только на границах. Если profile — сложная кривая (e.g., NURBS с изгибами), это приводит к потере деталей.

**Зачем:** Surface of revolution — основной способ описания тел вращения в STEP (VALVE, BOLT, GEAR). Без dedicated grid детализация теряется.

**Задачи:**

- [ ] 2.4.1 Создать `generate_revolution_steiner_grid(surface: &RevolutionSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`.
- [ ] 2.4.2 Алгоритм: n_u по chord-error (24–48), n_v — по адаптивной дискретизации profile curve (использовать `adaptive_curve_discretization` из `edge_cache.rs`).
- [ ] 2.4.3 Special case: если profile — line → v grid равномерный (аналогично цилиндру).
- [ ] 2.4.4 Special case: если profile — circle arc → torus-like grid (делегировать в `generate_torus_steiner_grid` после преобразования).
- [ ] 2.4.5 Интегрировать в `triangulate_surface_consistent`.
- [ ] 2.4.6 Тест: bottle-neck profile (line + arc + line) с отверстием в widest section.

**Критерий приёмки:** RevolutionSurface с нелинейным profile триангулируется с сохранением деталей профиля.

---

### 2.5 Dedicated Steiner grid для Extrusion

**Проблема:** `ExtrusionSurface` parameterizes (u, v) ∈ [0, 1] × [0, 1], где u — параметр вдоль profile, v — вдоль extrusion direction. Аналогично revolution, generic grid не учитывает кривизну profile.

**Зачем:** Extrusion — второй по частоте способ описания тел в STEP (after B-REP with planar faces). Часто встречается в architectural / structural parts.

**Задачи:**

- [ ] 2.5.1 Создать `generate_extrusion_steiner_grid(surface: &ExtrusionSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`.
- [ ] 2.5.2 Алгоритм: n_u по адаптивной дискретизации profile, n_v по chord-error (обычно 2–8 для linear extrusion).
- [ ] 2.5.3 Special case: если extrusion direction не perpendicular profile → параметризация искажена, использовать Jacobian-based chord tol.
- [ ] 2.5.4 Интегрировать в `triangulate_surface_consistent`.
- [ ] 2.5.5 Тест: extrusion of a NURBS profile (S-curve) with 2 holes.

**Критерий приёмки:** ExtrusionSurface с NURBS profile корректно триангулируется с учётом кривизны profile.

---

### 2.6 Dedicated Steiner grid для NURBS

**Проблема:** NURBS — самый сложный случай. Текущий путь: `triangulate_nurbs_cdt` → generic `parameter_division_2d` → sparse grid. При наличии отверстий результат часто "проваливается" или имеет перекрученные треугольники. В drill_top.stp 102 NURBS surfaces with knots + 186 bounded B-spline surfaces = 288 NURBS-подобных граней.

**Зачем:** NURBS — это "последняя миля" универсальности. Без корректной поддержки NURBS с отверстиями 3Draper не может претендовать на universal STEP viewer.

**Задачи:**

- [ ] 2.6.1 Создать `generate_nurbs_steiner_grid(surface: &NurbsSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`.
- [ ] 2.6.2 Алгоритм: использовать существующий `parameter_division_2d` для начальных u/v knots, НО применить densify: если грид < 8×8 — увеличить до 8×8 (минимальный порог для earcutr с отверстиями).
- [ ] 2.6.3 Добавить curvature-adaptive refinement: для каждого sub-rectangle grid оценить Gauss curvature (через 2nd derivatives NURBS), если > threshold — subdivить. Это концентрирует Steiner points там, где они нужны.
- [ ] 2.6.4 Special case: bilinear NURBS (u_degree ≤ 1 && v_degree ≤ 1) — пропустить, обрабатывается как plane.
- [ ] 2.6.5 Special case: ruled NURBS (одна степень = 1) — использовать densify только в нелинейном направлении.
- [ ] 2.6.6 Special case: NURBS с periodic knots (closed surface) — wrap-around grid, не добавлять Steiner points на seam (u = u_max).
- [ ] 2.6.7 Заменить вызов `triangulate_nurbs_cdt` в `triangulate_face_impl` (triangulate.rs:1013) на вызов `triangulate_surface_consistent` с NURBS-aware grid. Сохранить `triangulate_nurbs_cdt` как fallback.
- [ ] 2.6.8 Тест: NURBS surface (degree 3×3, 8×8 control points) с 3 круглыми отверстиями — проверить watertight и визуальное качество.
- [ ] 2.6.9 Тест: `test/nist_complex_surface.stp` — проверить что отсутствуют "провалы" и перекрученные треугольники.

**Критерий приёмки:** NURBS surfaces с отверстиями триангулируются без визуальных артефактов; минимум 8×8 Steiner grid для любого NURBS face с отверстиями.

---

### 2.7 Edge case: вырожденные поверхности

**Проблема:** Некоторые грани в STEP имеют вырожденную параметризацию: sphere near poles (v ≈ 0 или π), cone at apex (radius = 0), torus with minor_radius = 0, NURBS with collapsed boundary. Текущий код частично обрабатывает это, но не систематически.

**Зачем:** Без явной обработки вырождений earcutr получает NaN/Inf UV координаты и падает или возвращает пустую сетку.

**Задачи:**

- [ ] 2.7.1 Создать функцию `is_degenerate_uv(surface, u, v, tol) -> bool` в `parametric_domain.rs`. Возвращает true если (u, v) близко к полюсу/apex/colapsed-edge.
- [ ] 2.7.2 В каждом `generate_*_steiner_grid` фильтровать Steiner-кандидатов через `is_degenerate_uv` перед добавлением в результат.
- [ ] 2.7.3 В `triangulate_surface_consistent` добавить pre-check: если > 50% boundary points вырождены → использовать fan triangulation из apex (для cone) или skip face (для fully degenerate).
- [ ] 2.7.4 Тест: cone with apex (radius=0, half_angle=π/4) — проверить что apex не дублируется в сетке как 100 вершин.
- [ ] 2.7.5 Тест: sphere cap (v ∈ [0, π/4]) — проверить что pole vertex единственный.

**Критерий приёмки:** Ни одна грань не возвращает пустой mesh из-за вырождения; apex/pole vertices не дублируются.

---

## 3. Фаза 2 — Универсальная поддержка кривых (2–3 недели)

> **Цель:** Все типы STEP-кривых корректно парсятся, дискретизируются и используются в PCURVE.

### 3.1 Аудит текущего покрытия кривых

| Curve type | Parsing | Discretization | PCURVE (2D) | Статус |
|---|---|---|---|---|
| LINE | ✅ | ✅ | ✅ | OK |
| CIRCLE | ✅ | ✅ | ✅ | OK |
| ELLIPSE | ✅ | ✅ | ✅ | OK |
| HYPERBOLA | ✅ (P6) | ✅ | ❌ | **Требует PCURVE** |
| PARABOLA | ✅ (P6) | ✅ | ❌ | **Требует PCURVE** |
| ARC | ✅ (через TRIMMED_CURVE) | ✅ | ⚠️ | OK |
| B_SPLINE_CURVE | ✅ | ✅ | ✅ | OK |
| BEZIER_CURVE | ✅ | ✅ | ✅ | OK |
| POLYLINE | ✅ | ✅ | ✅ | OK |
| TRIMMED_CURVE | ✅ (P14) | ✅ | ⚠️ | OK |
| COMPOSITE_CURVE | ✅ | ✅ | ⚠️ | OK |
| COMPOSITE_CURVE_ON_SURFACE | ✅ | ✅ | ⚠️ | OK |
| OFFSET_CURVE_3D | ✅ | ✅ | ❌ | **Требует PCURVE** |
| OFFSET_CURVE_2D | ❌ | ❌ | ❌ | **Не реализован** |
| PCURVE | ✅ (P11) | ✅ | ✅ | OK |
| BOUNDED_CURVE | ✅ (через nesting) | ✅ | ⚠️ | OK |
| CURVE_ON_SURFACE | ✅ | ✅ | ⚠️ | OK |
| SURFACE_CURVE | ✅ | ✅ | ✅ | OK |
| INTERSECTION_CURVE | ✅ (P12) | ✅ | ⚠️ | OK |

**Вывод:** 4 проблемных типа: HYPERBOLA, PARABOLA, OFFSET_CURVE_3D, OFFSET_CURVE_2D — у них нет 2D-представления, что критично для PCURVE-based граней.

---

### 3.2 Hyperbola / Parabola PCURVE

**Проблема:** `resolve_hyperbola_curve` и `resolve_parabola_curve` (converter.rs:8572, 8598) возвращают `Curve3d::Hyperbola` / `Curve3d::Parabola`. Но в PCURVE path (`extract_pcurve_for_surface`) вызывается `resolve_pcurve_to_curve2d`, который не имеет case для Hyperbola/Parabola → возвращает `None` → грань использует fallback `surface.project_point()` (медленно и неточно).

**Зачем:** Hyperbola и Parabola встречаются в conic-section edges на поверхностях (e.g., intersection curve of plane and cone). Без 2D-представления PCURVE эти грани теряют точность.

**Задачи:**

- [ ] 3.2.1 Добавить `Curve2d::Hyperbola(Hyperbola2d)` и `Curve2d::Parabola(Parabola2d)` варианты в `crates/draper-geometry/src/curve2d.rs`. Поля: center, semi_real, semi_imag (hyperbola) или focal_length (parabola), axis_x, axis_y.
- [ ] 3.2.2 Реализовать `point_at(t)`, `derivative_at(t)`, `parameter_range()` для обоих.
- [ ] 3.2.3 В `resolve_pcurve_to_curve2d` (converter.rs:7204) добавить handling для Hyperbola/Parabola PCURVE — извлечь axis2 frame + параметры, преобразовать в 2D.
- [ ] 3.2.4 В `resolve_curve_2d` (converter.rs:7240) добавить cases `"HYPERBOLA" | "PARABOLA"` для явных 2D-кривых.
- [ ] 3.2.5 Тест: STEP файл с conic curve на cone surface — проверить что PCURVE извлечён и edge UV корректны.

**Критерий приёмки:** `test/` файл с HYPERBOLA или PARABOLA PCURVE больше не использует `surface.project_point()` fallback.

---

### 3.3 Offset Curve 2D и 3D PCURVE

**Проблема:** `OFFSET_CURVE_3D` парсится и аппроксимируется NURBS (converter.rs:9091), но `OFFSET_CURVE_2D` вообще не обрабатывается. PCURVE с offset basis теряется.

**Зачем:** Offset curves используются для describing parallel edges (e.g., cutouts with constant offset). Без поддержки 2D-offset PCURVE эти грани теряют точность.

**Задачи:**

- [ ] 3.3.1 Реализовать `resolve_offset_curve_2d` в converter.rs — аналогично `resolve_offset_curve_3d`, но возвращает `Curve2d` (NURBS-аппроксимация offset от 2D basis curve).
- [ ] 3.3.2 В `resolve_curve_2d` добавить case `"OFFSET_CURVE_2D"` → вызов `resolve_offset_curve_2d`.
- [ ] 3.3.3 В `resolve_pcurve_to_curve2d` — если PCURVE ссылается на offset curve 3D, извлечь basis curve 2D и применить offset в UV-пространстве (с учётом Jacobian для корректного расстояния).
- [ ] 3.3.4 Тест: STEP файл с offset curve (отступ от line) — проверить что PCURVE корректен.

**Критерий приёмки:** `OFFSET_CURVE_2D` и `OFFSET_CURVE_3D` PCURVE извлекаются и используются в триангуляции.

---

### 3.4 CompositeCurveOnSurface сmixed curve types

**Проблема:** `COMPOSITE_CURVE_ON_SURFACE` может содержать сегменты разных типов (Line + Circle + BSpline). Текущий `resolve_composite_curve` (converter.rs:8927) конкатенирует их в одну `Curve3d::Nurbs` (через approximation). Но это теряет аналитическую структуру и PCURVE для каждого сегмента.

**Зачем:** CompositeCurveOnSurface — частый паттерн для сложных кромок (e.g., профиль лопатки турбины). Потеря сегментов = потеря точности.

**Задачи:**

- [ ] 3.4.1 Добавить `Curve3d::Composite { segments: Vec<Curve3d> }` вариант. Реализовать `point_at(t)` с правильным segment-routing по параметру.
- [ ] 3.4.2 Добавить `Curve2d::Composite { segments: Vec<Curve2d> }` аналог.
- [ ] 3.4.3 Обновить `resolve_composite_curve` возвращать `Curve3d::Composite` вместо NURBS-аппроксимации (сохранить approximation как fallback для legacy API).
- [ ] 3.4.4 В `EdgeDiscretizationCache::discretize_edge` — обрабатывать `Curve3d::Composite` сегмент-за-сегментом, конкатенируя точки (с deduplication на стыках).
- [ ] 3.4.5 Тест: composite curve из 3 сегментов (line + arc + line) — проверить что точки корректны на стыках.

**Критерий приёмки:** CompositeCurveOnSurface сохраняет аналитическую структуру сегментов в дискретизации.

---

## 4. Фаза 3 — Производительность для файлов любой сложности (2–3 недели)

> **Цель:** drill_top.stp (2971 грань, 5 BREP) загружается < 30с desktop / < 90с mobile; Vulcan (15MB STEP) < 60с desktop / < 180с mobile. Без UI freeze.

### 4.1 Web Worker для триангуляции (WASM)

**Проблема:** Текущая chunked триангуляция (500ms chunks) работает на main thread. Между chunks браузер repaint, но input events (scroll, click) всё ещё лагают. Для действительно больших файлов это неприемлемо.

**Зачем:** Web Worker позволяет полностью вынести триангуляцию в background thread, оставив main thread для UI. Это industry-standard подход для CAD viewers (Autodesk Viewer, Onshape).

**Задачи:**

- [ ] 4.1.1 Создать `crates/draper-wasm/src/worker.rs` — отдельный WASM модуль, экспортирующий `triangulate_step(step_bytes: &[u8], lod: f64) -> TriangulationResult`.
- [ ] 4.1.2 Настроить `Trunk.toml` / `wasm-bindgen` для сборки двух артефактов: main (UI) + worker (triangulation). Worker загружается через `Worker::new("/draper-worker.js")`.
- [ ] 4.1.3 В viewer: при загрузке STEP postMessage `{type: 'triangulate', bytes, lod}` в worker. Worker возвращает `{type: 'progress', faces_done, faces_total}` периодически и `{type: 'done', mesh_bytes}` в конце.
- [ ] 4.1.4 Передача mesh: serialize `TriangleMesh` в flatbuffers или bincode, передать как `Transferable` (ArrayBuffer) для zero-copy.
- [ ] 4.1.5 Cancel: postMessage `{type: 'cancel'}` в worker; worker aborts session.
- [ ] 4.1.6 Fallback: если Worker creation fails (CSP, old browser) — использовать текущий main-thread chunked path.
- [ ] 4.1.7 Тест: загрузить drill_top.stp — UI должен оставаться полностью responsive (scroll, click, zoom) во время триангуляции.

**Критерий приёмки:** UI framerate ≥ 60fps во время загрузки любого файла. Cancel button реагирует мгновенно.

---

### 4.2 Адаптивный LOD по сложности грани

**Проблема:** Текущий LOD применяется глобально через `keep_ratio` (decimate post-triangulation). Это означает что все грани триангулируются в full quality, потом decimate. Для drill_top.stp это ~3M triangles → decimate до 300K = waste of time.

**Зачем:** Adaptive LOD — триангулировать каждую грань сразу в нужном разрешении, без decimation. Это даёт 2–5× ускорение на больших сборках.

**Задачи:**

- [ ] 4.2.1 В `TriangulationParams` добавить `target_triangles_per_face: Option<usize>`. Если `Some(n)` — каждая грань стремится к ≤ n треугольников (адаптивный Steiner budget).
- [ ] 4.2.2 Алгоритм budget: `target_per_face = total_budget / face_count`, где `total_budget = lod × 100_000` (для LOD 1.0 = 100K triangles total).
- [ ] 4.2.3 В каждом `generate_*_steiner_grid` использовать `target_triangles_per_face` как верхний budget cap (вместо хардкоженного `max_face_triangles`).
- [ ] 4.2.4 Удалить post-triangulation decimation path (или сделать опциональным для legacy LOD). Для новых LOD value < 1.0 — использовать adaptive budget; для value = 1.0 — full quality.
- [ ] 4.2.5 Тест: drill_top.stp при LOD 0.5 — должно быть ~150K triangles (вместо 3M → 150K через decimate). Время триангуляции должно упасть с 23с до ~8с.

**Критерий приёмки:** LOD 0.5 на drill_top.stp триангулируется за < 10с desktop без decimation step.

---

### 4.3 Параллельная триангуляция BREP (native)

**Проблема:** На native (desktop) BREPs обрабатываются последовательно по 8 per frame. Для файлов с 50+ BREPs (as1-oc-214) это медленно. WASM пока single-threaded (wasm-threads experimental).

**Зачем:** Desktop users expect parallelism. 4-core CPU должно давать ~3× speedup.

**Задачи:**

- [ ] 4.3.1 В `crates/draper-step/src/converter.rs` добавить `triangulate_breps_parallel(breps: Vec<PendingBrepInstance>, params) -> Vec<DetailedMeshInstance>` используя `rayon::scope`.
- [ ] 4.3.2 Каждый BREP триангулируется в отдельном rayon task. Shared `EdgeDiscretizationCache` защищён `RwLock` (или каждый BREP имеет свой cache — проще, но теряет edge sharing).
- [ ] 4.3.3 В viewer native path: если `pending_breps.len() > 4` → использовать parallel path. Иначе sequential (avoid overhead).
- [ ] 4.3.4 Тест: as1-oc-214.stp (12 BREPs) — должно быть ~2.5× быстрее на 4-core CPU.

**Критерий приёмки:** Native загрузка as1-oc-214.stp ускорена в 2×+ на multi-core CPU.

---

### 4.4 Кэширование триангуляции в IndexedDB (WASM)

**Проблема:** Пользователь открывает drill_top.stp второй раз — снова 30с ожидания. Нет кэша.

**Зачем:** Кэш в IndexedDB даёт мгновенную загрузку при повторном открытии. Это critical UX для CAD viewer.

**Задачи:**

- [ ] 4.4.1 Создать `crates/draper-viewer/src/cache.rs` — обёртка над IndexedDB через `web-sys::IdbDatabase`.
- [ ] 4.4.2 Key: SHA-256 of STEP file bytes. Value: serialized `TriangleMesh` + `Vec<FaceInfo>` + `assembly_tree` (bincode).
- [ ] 4.4.3 При загрузке STEP: hash → lookup in IDB → если hit, deserialize и показать мгновенно; если miss, triangulate и store.
- [ ] 4.4.4 Кэш имеет TTL (7 дней) и size limit (500MB, LRU eviction).
- [ ] 4.4.5 UI: показать "Loaded from cache" в логе при cache hit.
- [ ] 4.4.6 Настройка: кнопка "Clear cache" в Settings.

**Критерий приёмки:** Повторное открытие drill_top.stp загружается < 1с (cache hit).

---

## 5. Фаза 4 — Качество и надёжность (2 недели)

> **Цель:** Гарантировать watertight для всех тестовых файлов; явная обработка всех вырожденных случаев.

### 5.1 Явная обработка seam edges для периодических поверхностей

**Проблема:** Cylinder/Sphere/Torus/Revolution имеют periodic parameterization (u ∈ [0, 2π]). Boundary edges на u=0 и u=2π — это одна и та же 3D-кривая (seam). Текущий код частично обрабатывает это (weld PASS 2, seam-split recursion в `triangulate_surface_consistent`), но всё ещё бывают mismatch.

**Зачем:** Seam mismatch = visible gap на цилиндре/сфере. Это самая частая причина "негерметичности".

**Задачи:**

- [ ] 5.1.1 В `EdgeDiscretizationCache::discretize_edge` — для seam edges (detected по `edge.is_seam = true` flag из STEP OR по geometric coincidence of two edges in face loop) гарантировать идентичность точек на обоих сторонах.
- [ ] 5.1.2 В `triangulate_surface_consistent` — для periodic surfaces всегда применять seam-split (разделение UV polygon на две половины по seam), даже если boundary не вырождена. Это предотвращает earcutr от "склеивания" точек с двух сторон seam.
- [ ] 5.1.3 В `BrepSession::finalize` — добавить post-weld pass для seam endpoints: если две вершины в mesh ближе чем `tol_ctx.absolute() × 0.1` и обе лежат на seam (детекция по UV координатам ± π) → merge.
- [ ] 5.1.4 Тест: cylinder with 2 holes at u=π/2 и u=3π/2 (symmetric relative to seam) — проверить watertight.
- [ ] 5.1.5 Тест: torus (full, both directions periodic) — проверить watertight.

**Критерий приёмки:** Cylinder/Sphere/Torus/Revolution surfaces на тестовых файлах имеют 0 boundary edges (100% watertight).

---

### 5.2 Валидация топологии BREP перед триангуляцией

**Проблема:** Текущий pipeline сразу триангулирует, без проверки что face loop замкнут, что каждое ребро имеет ровно 2 coedges (для internal) или 1 (для boundary), что orientation согласована.

**Зачем:** Невалидная топология = unpredictable triangulation. Лучше detect и heal на раннем этапе.

**Задачи:**

- [ ] 5.2.1 Создать `crates/draper-topology/src/validator.rs` с функцией `validate_brep(brep: &Brep) -> TopologyReport`.
- [ ] 5.2.2 Проверки: (a) каждый face имеет ≥ 1 outer loop; (b) каждый edge в loop имеет корректную orientation; (c) каждое internal edge имеет 2 coedges (для solid) или 1 (для sheet); (d) эйлерова характеристика для closed solid = 2.
- [ ] 5.2.3 В `prepare_brep_session` (converter.rs) вызывать `validate_brep` и логировать warnings. Не блокировать триангуляцию — только диагностика.
- [ ] 5.2.4 Если найдены dangling edges (1 coedge для internal position) — попытаться heal: найти matching edge по geometry и слить.
- [ ] 5.2.5 Тест: искусственно сломанный BREP (face missing) — проверить что validator находит проблему.

**Критерий приёмки:** Все 24 тестовых файла проходят `validate_brep` без warnings.

---

### 5.3 Тестовая инфраструктура для сложных файлов

**Проблема:** Тесты в `draper-testing` покрывают простые детали. Нет автоматизированных тестов на drill_top.stp, Vulcan, transmission (боятся долгого выполнения).

**Зачем:** Регрессии на сложных файлах обнаруживаются пользователем, а не CI. Это неприемлемо.

**Задачи:**

- [ ] 5.3.1 Создать `crates/draper-testing/tests/complex_files.rs` с тестами для каждого "тяжёлого" файла (drill_top, Vulcan, transmission, 8394-121_Spit-Fire, Zentralstaender).
- [ ] 5.3.2 Каждый тест помечен `#[ignore]` по умолчанию (быстрый CI). Запускаются через `cargo test -- --ignored` (nightly или manual).
- [ ] 5.3.3 Каждый тест проверяет: (a) load без panic; (b) triangle_count > 0 для каждого instance; (c) boundary edge % < 5%; (d) elapsed time < 60s desktop.
- [ ] 5.3.4 CI workflow (`.github/workflows/ci.yml`) — nightly job с `--ignored` tests. Результаты в GitHub Actions artifacts.
- [ ] 5.3.5 Benchmark regression: сравнить triangle_count и elapsed_time с `benchmark_baseline.csv`. Если regression > 20% — fail.

**Критерий приёмки:** Nightly CI ловит регрессии на сложных файлах до того, как они попадают к пользователю.

---

## 6. Фаза 5 — Расширение покрытия STEP-сущностей (2 недели)

> **Цель:** Поддержка всех сущностей, встречающихся в реальных STEP AP203/AP214/AP242 файлах.

### 6.1 Полная поддержка AP242 (manufacturing semantics)

**Проблема:** AP242 добавляет GD&T (Geometric Dimensioning and Tolerancing), PMI (Product Manufacturing Information), kinematics. 3Draper частично поддерживает PMI display (`pmi_display.rs`), но не GD&T semantics.

**Зачем:** AP242 — современный стандарт для aerospace/automotive. Без его поддержки 3Draper не может использоваться в этих индустриях.

**Задачи:**

- [ ] 6.1.1 Расширить парсер для AP242-specific entities: `SHAPE_ASPECT`, `DIMENSIONAL_LOCATION`, `GEOMETRIC_TOLERANCE` (already partial), `DATUM_FEATURE`, `DATUM_REFERENCE`.
- [ ] 6.1.2 В UI добавить панель "GD&T" — список всех tolerances с привязкой к faces.
- [ ] 6.1.3 В 3D-сцене — аннотации для GD&T (display tolerance frame над соответствующей face).
- [ ] 6.1.4 Тест: AP242 sample file с GD&T (найти в NIST или STEP forum).

**Критерий приёмки:** AP242 файл с GD&T открывается, tolerances видны в UI и 3D-сцене.

---

### 6.2 Поддержка сборок высшего уровня (subassemblies)

**Проблема:** `NEXT_ASSEMBLY_USAGE_OCCURRENCE` (NAUO) поддерживается (converter.rs has `nauo_transform_map`), но вложенные subassemblies (subassembly внутри subassembly) могут терять transforms.

**Зачем:** Реальные сборки имеют 5–10 уровней вложенности (engine → cylinder head → valve assembly → valve + spring + retainer).

**Задачи:**

- [ ] 6.2.1 Аудит `nauo_transform_map` — рекурсивно ли применяются transforms для nested NAUOs?
- [ ] 6.2.2 Тест: assembly с 3+ уровнями nesting (создать синтетический STEP).
- [ ] 6.2.3 В UI tree — показать иерархию subassemblies, не только flat instance list.

**Критерий приёмки:** 3-уровневая assembly корректно отображается с правильными transforms.

---

### 6.3 Поддержка BREP_WITH_VOIDS (внутренние полости)

**Проблема:** `BREP_WITH_VOIDS` парсится (P7), но voids могут теряться если их shells имеют complex orientation.

**Зачем:** Voids — критично для casting/forging parts (внутренние каналы для lubrication, weight reduction).

**Задачи:**

- [ ] 6.3.1 Аудит текущего void handling в `triangulate_brep_detailed`.
- [ ] 6.3.2 Тест: cube с цилиндрическим void (внутренний канал).
- [ ] 6.3.3 Если void shells теряются — добавить explicit branch в triangulation для void shells (negative orientation).

**Критерий приёмки:** BREP_WITH_VOIDS отображается с внутренними полостями.

---

## 7. Фаза 6 — Универсальный тестовый стенд (1 неделя)

> **Цель:** Автоматизированная проверка: любой STEP-файл из public sources открывается без ошибок.

### 7.1 ABC Dataset integration

**Проблема:** ABC dataset (Autodesk Benchmark Collection, ~1M CAD models) — industry standard для testing geometry kernels. 3Draper не тестируется на нём.

**Зачем:** ABC testing выявит edge cases, отсутствующие в 24 текущих тестах.

**Задачи:**

- [ ] 7.1.1 Скачать curated STEP subset (~100 файлов) из ABC.
- [ ] 7.1.2 Создать `crates/draper-testing/tests/abc_dataset.rs` — для каждого файла: load, triangulate, check watertight, measure time.
- [ ] 7.1.3 Отчёт: % successful loads, % watertight, avg time, max time. Сохранить в `docs/abc_baseline.md`.
- [ ] 7.1.4 Если файл fail — добавить его в bug list (отдельный раздел в этом плане).

**Критерий приёмки:** ≥ 95% ABC dataset файлов открываются без ошибок.

---

### 7.2 NIST CTS-2 test suite

**Проблема:** NIST CTS-2 (Conformance Test Suite) — официальный STEP AP203/AP214 conformance test. Не используется.

**Зачем:** CTS certification — это industry-recognized признак качества STEP parser.

**Задачи:**

- [ ] 7.2.1 Скачать NIST CTS-2 sample files.
- [ ] 7.2.2 Прогнать через 3Draper, отчёт по каждому файлу.
- [ ] 7.2.3 Для каждого fail — создать issue с описанием проблемы.

**Критерий приёмки:** ≥ 90% NIST CTS-2 файлов проходят.

---

## 8. Известные bug'и и edge cases

> Список конкретных проблем, обнаруженных в пользовательских сценариях. По мере исправления — отмечать `[x]`.

### 8.1 drill_top.stp

- [ ] 8.1.1 Step#803 триангуляция верхней грани — видны артефакты.
- [ ] 8.1.2 Пропавший 5-й элемент сборки (timeout) — фиксиится в 1.2.
- [ ] 8.1.3 Torus fillets выглядят гранёными на mobile LOD — фиксиится в 2.3.

### 8.2 3.05.078.stp

- [ ] 8.2.1 Cone face ID 8 (forward:false) триангулируется некорректно — проверить orientation handling.
- [ ] 8.2.2 Cone face ID 2 — та же проблема.

### 8.3 UV visualization

- [ ] 8.3.1 UV-разложение цилиндра: масштабирование оси некорректно.
- [ ] 8.3.2 UV-разложение плоскости: mapping инвертирован.

### 8.4 8394-121_Spit-Fire.STEP

- [ ] 8.4.1 NURBS surfaces с отверстиями — проверка качества.

### 8.5 8500-02_Vulcan.STEP (15MB)

- [ ] 8.5.1 Время загрузки > 60s desktop — оптимизация через 4.2 (adaptive LOD).
- [ ] 8.5.2 Memory usage > 1GB — проверка leaks.

---

## 9. Метрики успеха (KPIs)

| Метрика | Текущее значение | Цель (3 месяца) | Цель (6 месяцев) |
|---|---|---|---|
| Тестовых файлов проходят | 24/24 (100%) | 24/24 + ABC ≥ 95% | 24/24 + ABC ≥ 99% + NIST CTS-2 ≥ 95% |
| drill_top.stp desktop load time | ~24s | < 15s | < 10s |
| drill_top.stp mobile load time | ~120s (timeout) | < 90s | < 60s |
| Vulcan STEP desktop load time | неизвестно | < 60s | < 30s |
| Watertight (drill_top) | ~99.36% | 99.8% | 99.95% |
| UI responsiveness during load | 500ms chunks | Web Worker (60fps) | Web Worker (60fps) |
| Поддержка surface types | 8/13 (62%) | 13/13 (100%) | 13/13 + special cases |
| Поддержка curve types (PCURVE) | 14/18 (78%) | 18/18 (100%) | 18/18 + special cases |
| Cache hit повторное открытие | N/A | < 1s | < 1s |
| Regression detection | Manual | Nightly CI | Nightly CI + ABC + NIST |

---

## 10. Хронология исполнения

> Обновлять по мере завершения задач. Каждая запись: дата + commit + что сделано.

| Дата | Commit | Что сделано |
|---|---|---|
| 2026-06-26 | — | План создан (этот документ) |
| 2026-06-27 | `552a7ff` (main) · `feec20b` (gh-pages) | **Phase 0 стабилизация:** 1.1 `SteinerBudgetProfile` enum (Desktop 96×64 / Tablet 64×32 / Mobile 32×16) + `candidate_multiplier()`; 1.2 `take_partial_active_session` salvage при timeout/Cancel + warning logs; 1.3 mobile LOD downgrade смягчён до 1 ступени (Ultra→High), 2 ступени только при 5+ BREP. Частично: 1.1.4 (per-face adaptive budget), 1.2.4 (UI-баннер partial), 1.2.5 (auto-тест), 1.3.3 (localStorage), 1.3.4 (UI quality badge), 1.3.5 (mobile auto-тест). Все 180 mesh + 97 step + 7 integration тестов проходят; WASM release собран и задеплоен на gh-pages. |

---

## 11. Ссылки

- **ROADMAP.md** — долгосрочная стратегия (Phases 1–5, 24 месяца)
- **docs/TRUCK_BORROW_PLAN.md** — что заимствовано из truck
- **docs/truck_vs_3draper_deep_comparison.md** — детальное сравнение
- **docs/benchmark_baseline.md** — текущий benchmark
- **worklog_new.md** — почасовой журнал работы
- **Live demo:** https://kerneldev.github.io/3Draper/

---

## 12. Принцип работы с этим планом

1. **Перед началом задачи** — отметить `[~]` в соответствующем пункте.
2. **После завершения** — отметить `[x]`, добавить commit hash в раздел 10.
3. **Если заблокировано** — отметить `[!]`, описать причину в комментарии.
4. **Если обнаружен новый bug** — добавить в раздел 8.
5. **Если обнаружена новая задача** — добавить в соответствующую фазу.
6. **Раз в неделю** — ревью плана: что сделано, что отложено, что добавлено.
7. **После каждого commit** — push в `origin/main` и deploy на gh-pages.

---

*Этот документ — живой план. Он обновляется по мере работы. Главная цель — каждый STEP-файл открывается быстро, качественно, без сюрпризов.*
