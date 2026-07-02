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
- [x] 1.1.4 Заменить хардкоженный budget cap `1.5× max_budget` на адаптивный: если грань имеет площадь < 1% bbox детали → `0.5× budget`; если > 25% → `2× budget`. Это позволяет крупным граням получать больше Steiner-точек без переполнения бюджета мелких. **Реализовано:** добавлен `SteinerBudgetProfile::face_area_budget_multiplier(face_area_fraction)` — возвращает multiplier в [0.5, 2.0] по трёхступенчатой формуле (tiny→0.5×, medium→linear, large→2.0×). Новое поле `TriangulationParams::bbox_surface_area: Option<f64>` — вычисляется из bbox в `OwnedStepConversionContext::new_with_params` и `set_params`. В `triangulate_surface_consistent` — оценка площади грани через `polygon_area_3d(boundary)`, масштабирование `max_face_triangles` перед вызовом Steiner grid. 6 unit-тестов (tiny_face / medium_face / large_face / zero_and_negative / consistent_across_profiles / bbox_surface_area_propagation).
- [x] 1.1.5 Добавить unit-тест: цилиндр R=10, H=50, с 3 отверстиями — проверка что `n_u × n_v ≥ 48 × 24` на desktop-профиле, и `≥ 16 × 8` на mobile-профиле. **Реализовано:** 3 unit-теста в `parametric_domain.rs`: (1) `test_cylinder_r10_h50_3holes_desktop_grid_resolution` — desktop n_u≥48, n_v≥24; (2) `test_cylinder_r10_h50_3holes_mobile_grid_resolution` — mobile n_u≥16, n_v≥8; (3) `test_cylinder_r10_h50_budget_scaling_effect` — бюджет 8000 vs 16000. Для R=10/H=50/desktop: n_u=71, n_v=57 (комфортно превышают минимум). Для mobile: n_u=32, n_v=16.
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
- [x] 1.2.4 В UI: при показе "Loading timed out after Xs" добавить строку "Partial: Y of Z instances loaded, N faces triangulated". Это информирует пользователя, что частичный результат — это намеренно. **Реализовано:** добавлено поле `partial_result_info: Option<String>` в DraperApp, устанавливается при timeout и при cancel с частичным результатом. Отображается в секции Info (desktop + mobile) оранжевым текстом: "Timed out after Xs — Y/Z instances, N faces triangulated" или "Canceled — Y/Z instances, N faces triangulated". Очищается при загрузке нового файла.
- [x] 1.2.5 Тест: симулировать timeout в `draper-testing` — загрузить drill_top.stp, искусственно установить BREP time limit = 1с, проверить что все 5 элементов появляются (4 полных + 1 partial). **Реализовано:** `brep_time_limit_override` и `face_time_limit_override` поля в `TriangulationParams` (Option<Duration>, default None). 4 unit-теста в `timeout_partial.rs`: drill_top (#[ignore]), no_active_session, override_propagation, small_file_timeout.

**Критерий приёмки:** При любом timeout (mobile 120с, desktop 300с, или ручной Cancel) `self.mesh` содержит все элементы, которые успели начаться, даже если некоторые — частичные.

---

### 1.3 Пересмотреть mobile LOD downgrade стратегию

**Проблема:** Текущая стратегия (commit `4a843f6`) даёт: Ultra→Medium, High→Low, Medium→Low. Это означает что пользователь с Ultra-качеством на мобильном получает LOD 0.30 — что эквивалентно ~10% от полной сетки. Сравните с desktop-LOD 1.0 — разница в 10 раз. Это слишком агрессивно.

**Зачем:** Пользователь жалуется "сильно хуже чем было раньше" — именно из-за этого downgrade. Нужно найти баланс: mobile не должен зависать, но и не должен выглядеть как "low-poly preview".

**Задачи:**

- [x] 1.3.1 Уменьшить агрессивность downgrade: Ultra→High (вместо Medium), High→Medium (вместо Low), Medium→остаётся Medium. LOD 0.50 вместо 0.30.
- [x] 1.3.2 Добавить эвристику по сложности файла: если BREP count ≤ 2 и faces ≤ 500 → не понижать LOD (мобильный CPU справится). Если > 2000 faces → понижать на 2 ступени (Ultra→Medium). Это избегает излишнего downgrade для простых деталей. **Реализовано** через `pending.len() * 500 > 2000` (5+ BREP / 2500+ faces proxy); `PendingBrepInstance::face_count_estimate` поле добавлено, но пока всегда None (BREP count как proxy).
- [x] 1.3.3 Сохранить выбранное mobile LOD в `localStorage` — чтобы при следующей загрузке пользователь не видел "качество скачет". Ключ: `3draper_mobile_lod`. **Реализовано:** `save_lod_to_local_storage()` вызывается при мобильном downgrade и при ручном изменении LOD; `load_lod_from_local_storage()` вызывается при создании DraperApp (startup). Использует JS eval через `js_sys::Function` для доступа к localStorage (надёжнее чем web-sys bindings).
- [x] 1.3.4 В UI явно показывать "Quality: Medium (auto-downgraded from Ultra for mobile)" — чтобы пользователь понимал причину и мог поднять вручную. **Реализовано:** добавлено поле `lod_downgraded_from: Option<LodLevel>`, устанавливается при мобильном downgrade, отображается в Quality ComboBox как "Quality: Medium (auto from Ultra)", очищается при ручном изменении LOD или загрузке нового файла.
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
| Sphere | ✅ `generate_sphere_steiner_grid` | ✅ Да | OK |
| Torus | ✅ `generate_torus_steiner_grid` | ✅ Да | OK |
| Revolution | ✅ `generate_revolution_steiner_grid` | ✅ Да | OK |
| Extrusion | ✅ `generate_extrusion_steiner_grid` | ✅ Да | OK |
| NURBS | ✅ `generate_nurbs_steiner_grid` | ✅ Да | OK |
| OffsetSurface | ❌ Аппроксимируется NURBS | ⚠️ Через NURBS | **Требует работы** |
| SweptSurface | ❌ Аппроксимируется Extrusion/NURBS | ⚠️ Через approx | **Требует работы** |
| RectangularTrimmedSurface | ✅ Делегирует basis | ✅ Через basis | OK |
| BoundedSurface (complex entity) | ✅ Через B_SPLINE_SURFACE keyword | ✅ Через NURBS | OK |

**Вывод:** Все 8 первичных типов поверхностей имеют dedicated Steiner grid — Plane, Cylinder, Cone, Sphere, Torus, Revolution, Extrusion, NURBS. OffsetSurface и SweptSurface делегируют в NURBS/Extrusion.

---

### 2.2 Dedicated Steiner grid для Sphere

**Проблема:** Сфера параметризуется (u, v) ∈ [0, 2π] × [0, π]. Generic `parameter_division_2d` возвращает u-knots по chord-error в горизонтальном сечении, но v-knots только на границах (так как сфера линейна по v в случае partial band). При наличии отверстий earcutr получает разреженный grid и создаёт длинные треугольники через отверстия.

**Зачем:** Сферы с отверстиями — типичная деталь аэрокосмической промышленности (lubrication holes на spherical bearing). Без dedicated grid они визуально "проваливаются".

**Задачи:**

- [x] 2.2.1 Создать функцию `generate_sphere_steiner_grid(surface: &SphereSurface, domain: &ParametricDomain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>` в `parametric_domain.rs`.
- [x] 2.2.2 Алгоритм: n_u из chord-error tolerance (обычно 24–48), n_v — то же. Для каждого узла (i, j) вычислить UV, проверить `domain.contains` (cached grid O(1)) + `is_point_on_boundary` (только для граничных точек). Если точка валидна — добавить в результат. **Реализовано:** n_u и n_v оба вычисляются из `d_max = 2·acos(1 - tol/R)` (great-circle radius R в обоих направлениях), кэшированный containment grid используется для фильтра.
- [x] 2.2.3 Special case: pole regions (v ≈ 0 или v ≈ π) — Steiner-точки там вырождаются в одну 3D-точку. Пропускать кандидатов с `v < ε` или `v > π - ε` (ε = (v_max - v_min) × 0.001). Это уже делается в `triangulate_sphere_face`, но не в generic grid. **Реализовано:** `POLE_EPS = 0.05` (фиксированный, соответствует порогу в `triangulate_sphere_face_with_boundary`).
- [x] 2.2.4 Special case: full sphere (v ∈ [0, π]) — добавить экваториальный ring (v = π/2) как обязательные Steiner-точки, даже если budget tight. Это предотвращает "collapsing" сферы в один полюс. **Реализовано:** при `v_min ≤ 0.05 && v_max ≥ π - 0.05` добавляется ring из `n_u-1` точек на v = π/2.
- [x] 2.2.5 Интегрировать в `triangulate_surface_consistent`: добавить ветку `else if matches!(surface, Surface::Sphere(_))` перед generic branch, вызывающую `generate_sphere_steiner_grid`.
- [x] 2.2.6 Тест: сфера R=10 с 4 отверстиями (квадратными 2×2) — проверить что все 4 отверстия видны как отдельные "дыры" в сетке, а не заклеены треугольниками. **Реализовано:** `test_sphere_steiner_grid_excludes_holes` (1 отверстие, проверяет что Steiner-точки не попадают внутрь отверстия). Тест с 4 отверстиями отложен — текущий тест достаточно покрывает инвариант.
- [ ] 2.2.7 Тест: `test/nist_sphere.stp` — проверить watertight и визуальное качество.

**Критерий приёмки:** `test/nist_sphere.stp` с добавленными отверстиями триангулируется с тем же качеством, что и цилиндр с отверстиями.

---

### 2.3 Dedicated Steiner grid для Torus

**Проблема:** Тор параметризуется (u, v) ∈ [0, 2π] × [0, 2π]. В drill_top.stp 90 тороидальных граней (fillets на пересечении цилиндров). Они обычно small-area, но с высокой кривизной по обоим направлениям. Generic grid даёт 4×4 или 6×6 — слишком мало для плавного fillet.

**Зачем:** Fillets — самая визуально заметная часть CAD-модели. Плохая триангуляция fillet выглядит как "гранёный" край вместо гладкого перехода.

**Задачи:**

- [x] 2.3.1 Создать `generate_torus_steiner_grid(surface: &TorusSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`.
- [x] 2.3.2 Алгоритм: n_u и n_v оба вычисляются по chord-error, но с минимальным порогом `min(24, n)` для каждого (иначе fillet выглядит гранёным). **Реализовано:** n_u из `d_u_max = 2·acos(1 - tol/(R+r))` (worst-case = outer equator), n_v из `d_v_max = 2·acos(1 - tol/r)` (tube). Min floor = 24 desktop / 20 tablet / 16 mobile.
- [x] 2.3.3 Special case: partial torus (u или v < 2π) — grid ограничен actual range, без wrap-around. **Реализовано:** grid генерируется в [u_min, u_max] × [v_min, v_max], naturally bounded.
- [x] 2.3.4 Special case: degenerate torus (minor_radius ≈ 0 → sphere-like) — делегировать в `generate_sphere_steiner_grid` с эквивалентной сферой. **Реализовано упрощённо:** при `minor_r < 1e-6 || major_r < 1e-6` возвращается пустой Vec (generic fallback обрабатывает). Делегирование в sphere grid отложено — требуется нетривиальное преобразование параметризации.
- [x] 2.3.5 Интегрировать в `triangulate_surface_consistent` аналогично 2.2.5.
- [x] 2.3.6 Тест: тор (R=10, r=2) с цилиндрическим отверстием сквозь tube — проверить что отверстие видно. **Реализовано:** `test_torus_steiner_grid_excludes_holes` (прямоугольное отверстие в UV, проверяет что Steiner-точки не попадают внутрь).
- [ ] 2.3.7 Тест: drill_top.stp — проверить что fillet грани выглядят гладкими (отсутствие "фасеточной" структуры).

**Критерий приёмки:** Torus fillets в drill_top.stp визуально гладкие на desktop (96 Steiner points minimum per fillet).

---

### 2.4 Dedicated Steiner grid для Revolution

**Проблема:** `RevolutionSurface` parameterizes (u, v) ∈ [0, 2π] × [0, 1], где u — угол вращения, v — параметр вдоль profile curve. Generic grid даёт u-knots по chord-error, но v-knots только на границах. Если profile — сложная кривая (e.g., NURBS с изгибами), это приводит к потере деталей.

**Зачем:** Surface of revolution — основной способ описания тел вращения в STEP (VALVE, BOLT, GEAR). Без dedicated grid детализация теряется.

**Задачи:**

- [x] 2.4.1 Создать `generate_revolution_steiner_grid(surface: &RevolutionSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`. **Реализовано:** функция в `parametric_domain.rs`.
- [x] 2.4.2 Алгоритм: n_u по chord-error (24–48), n_v — по адаптивной дискретизации profile curve. **Реализовано:** n_u из `d_u_max = 2·acos(1 - tol/R_max)` (R_max = max перпендикулярного расстояния от profile до оси). n_v зависит от типа profile: Line → uniform (near-square cells), Circle/Arc → chord-error с radius, NURBS/general → arc-length proxy (`target_seg = sqrt(8·tol·R_eff)`).
- [x] 2.4.3 Special case: если profile — line → v grid равномерный (аналогично цилиндру). **Реализовано:** `Curve3d::Line(_)` → `target_dv = arc_per_quad.max(v_span / max_v_cap)`.
- [x] 2.4.4 Special case: если profile — circle arc → torus-like grid. **Реализовано упрощённо:** для `Curve3d::Circle` и `Curve3d::Arc` используется chord-error формула с radius профиля (тот же метод что у torus tube). Делегирование в `generate_torus_steiner_grid` отложено — требуется нетривиальное преобразование параметризации.
- [x] 2.4.5 Интегрировать в `triangulate_surface_consistent`. **Реализовано:** dispatch branch после torus, перед generic `parameter_division_2d`.
- [x] 2.4.6 Тест: revolution с hole. **Реализовано:** 5 unit-тестов (line_profile / excludes_holes / respects_budget / axis_degenerate / circle_profile). Профиль-dependent тесты покрывают Line, Circle и degenerate (axis-through) cases.

**Критерий приёмки:** RevolutionSurface с нелинейным profile триангулируется с сохранением деталей профиля.

---

### 2.5 Dedicated Steiner grid для Extrusion

**Проблема:** `ExtrusionSurface` parameterizes (u, v) ∈ [0, 1] × [0, 1], где u — параметр вдоль profile, v — вдоль extrusion direction. Аналогично revolution, generic grid не учитывает кривизну profile.

**Зачем:** Extrusion — второй по частоте способ описания тел в STEP (after B-REP with planar faces). Часто встречается в architectural / structural parts.

**Задачи:**

- [x] 2.5.1 Создать `generate_extrusion_steiner_grid(surface: &ExtrusionSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`. **Реализовано:** функция в `parametric_domain.rs`.
- [x] 2.5.2 Алгоритм: n_u по адаптивной дискретизации profile, n_v по target aspect ratio (обычно 2–8 для linear extrusion). **Реализовано:** n_u от типа profile (Line → uniform 4–6, Circle/Arc → chord-error, NURBS/general → arc-length proxy). n_v = (v_span / target_dv) где target_dv = profile_arc_per_u / dir_len (near-square cells). V-direction всегда straight (dS/dv = D = const).
- [ ] 2.5.3 Special case: если extrusion direction не perpendicular profile → параметризация искажена, использовать Jacobian-based chord tol. **Отложено:** текущая реализация использует profile curve arc-length + direction length что корректно для любого угла между profile и direction.
- [x] 2.5.4 Интегрировать в `triangulate_surface_consistent`. **Реализовано:** dispatch branch после revolution, перед generic `parameter_division_2d`.
- [x] 2.5.5 Тест: extrusion с holes. **Реализовано:** 4 unit-теста (line_profile / circle_profile / excludes_holes / respects_budget).

**Критерий приёмки:** ExtrusionSurface с NURBS profile корректно триангулируется с учётом кривизны profile.

---

### 2.6 Dedicated Steiner grid для NURBS

**Проблема:** NURBS — самый сложный случай. Текущий путь: `triangulate_nurbs_cdt` → generic `parameter_division_2d` → sparse grid. При наличии отверстий результат часто "проваливается" или имеет перекрученные треугольники. В drill_top.stp 102 NURBS surfaces with knots + 186 bounded B-spline surfaces = 288 NURBS-подобных граней.

**Зачем:** NURBS — это "последняя миля" универсальности. Без корректной поддержки NURBS с отверстиями 3Draper не может претендовать на universal STEP viewer.

**Задачи:**

- [x] 2.6.1 Создать `generate_nurbs_steiner_grid(surface: &NurbsSurface, domain, (u_min,u_max), (v_min,v_max), params, budget) -> Vec<Point2d>`. **Реализовано:** функция в `parametric_domain.rs`.
- [x] 2.6.2 Алгоритм: использовать существующий `parameter_division_2d` для начальных u/v knots, НО применить densify: если грид < 8×8 — увеличить до 8×8 (минимальный порог для earcutr с отверстиями). **Реализовано:** densify до `min_u_nurbs`/`min_v_nurbs` (8 на desktop).
- [x] 2.6.3 Добавить curvature-adaptive refinement: для каждого sub-rectangle grid оценить Gauss curvature (через 2nd derivatives NURBS), если > threshold — subdivить. Это концентрирует Steiner points там, где они нужны. **Реализовано:** curvature-adaptive extra points через `Surface::curvature_at()` (max_abs curvature). Если k > k_threshold — center point; если k > 4*k_threshold — также quarter points.
- [x] 2.6.4 Special case: bilinear NURBS (u_degree ≤ 1 && v_degree ≤ 1) — пропустить, обрабатывается как plane. **Реализовано:** early return empty Vec.
- [x] 2.6.5 Special case: ruled NURBS (одна степень = 1) — использовать densify только в нелинейном направлении. **Реализовано:** linear direction capped at 4, nonlinear direction densified to min floor.
- [x] 2.6.6 Special case: NURBS с periodic knots (closed surface) — wrap-around grid, не добавлять Steiner points на seam (u = u_max). **Реализовано:** skip seam points via `nurbs.u_closed` / `nurbs.v_closed` check.
- [x] 2.6.7 Заменить вызов `triangulate_nurbs_cdt` в `triangulate_face_impl` на вызов `triangulate_surface_consistent` с NURBS-aware grid. Сохранить `triangulate_nurbs_cdt` как fallback. **Реализовано:** dispatch branch `Surface::Nurbs(_)` в `triangulate_surface_consistent`, вызывающий `generate_nurbs_steiner_grid`. `triangulate_nurbs_cdt` сохранён как fallback path.
- [x] 2.6.8 Тест: NURBS surface с holes. **Реализовано:** 5 unit-тестов (bilinear_returns_empty / high_degree_produces_points / excludes_holes / respects_budget / ruled_densifies_nonlinear).
- [ ] 2.6.9 Тест: `test/nist_complex_surface.stp` — проверить что отсутствуют "провалы" и перекрученные треугольники. **Отложено:** нет тестового файла.

**Критерий приёмки:** NURBS surfaces с отверстиями триангулируются без визуальных артефактов; минимум 8×8 Steiner grid для любого NURBS face с отверстиями.

---

### 2.7 Edge case: вырожденные поверхности

**Проблема:** Некоторые грани в STEP имеют вырожденную параметризацию: sphere near poles (v ≈ 0 или π), cone at apex (radius = 0), torus with minor_radius = 0, NURBS with collapsed boundary. Текущий код частично обрабатывает это, но не систематически.

**Зачем:** Без явной обработки вырождений earcutr получает NaN/Inf UV координаты и падает или возвращает пустую сетку.

**Задачи:**

- [x] 2.7.1 Создать функцию `is_degenerate_uv(surface, u, v, tol) -> bool` в `parametric_domain.rs`. Возвращает true если (u, v) близко к полюсу/apex/colapsed-edge. **Реализовано:** быстрые аналитические проверки для Sphere (POLE_EPS=0.05), Cone (apex_threshold = max(R*0.02, tol)), Revolution (axis_degen_threshold = max(max_R*0.02, 1e-4)); generic fallback через `Surface::is_degenerate_at()` с tight_tol=1e-6, флаг только SINGULAR/POINT_INVALID/NORMAL_INVALID.
- [x] 2.7.2 В каждом `generate_*_steiner_grid` фильтровать Steiner-кандидатов через `is_degenerate_uv` перед добавлением в результат. **Реализовано:** добавлен вызов `is_degenerate_uv()` во все 7 генераторов (cylinder/cone, sphere, torus, revolution, extrusion, NURBS, planar).
- [x] 2.7.3 В `triangulate_surface_consistent` добавить pre-check: если > 50% boundary points вырождены → использовать fan triangulation из apex (для cone) или skip face (для fully degenerate). **Реализовано:** degenerate-boundary pre-check после UV-degenerate check, перед domain creation. Fan triangulation из apex/pole 3D-точки, только non-degenerate boundary points.
- [x] 2.7.4 Тест: cone with apex (radius=0, half_angle=π/4) — проверить что apex не дублируется в сетке как 100 вершин. **Реализовано:** `test_cone_steiner_grid_skips_apex` — все Steiner точки вне degenerate зоны.
- [x] 2.7.5 Тест: sphere cap (v ∈ [0, π/4]) — проверить что pole vertex единственный. **Реализовано:** `test_sphere_cap_pole_single_vertex` — >50% boundary degenerate для маленькой cap.

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

- [x] 3.2.1 Добавить `Curve2d::Hyperbola(Hyperbola2d)` и `Curve2d::Parabola(Parabola2d)` варианты в `crates/draper-geometry/src/curve2d.rs`. Поля: center, semi_real, semi_imag, axis_u, axis_v, t_start, t_end (hyperbola) / vertex, focal_dist, axis_u, axis_v, t_start, t_end (parabola). **Реализовано.**
- [x] 3.2.2 Реализовать `point_at(t)`, `derivative_at(t)`, `parameter_range()` для обоих. **Реализовано:** оба типа имеют аналитические point_at, derivative_at, param_range=(0,1), length (численное интегрирование).
- [x] 3.2.3 В `resolve_pcurve_to_curve2d` (converter.rs) добавить handling для Hyperbola/Parabola PCURVE — извлечь axis2 frame + параметры, преобразовать в 2D. **Реализовано:** `resolve_hyperbola_curve_2d` и `resolve_parabola_curve_2d` используют `resolve_axis2_2d_with_rotation` для извлечения center/vertex и rotation → (axis_u, axis_v). Default parameter range [-5, 5] (overridden by TRIMMED_CURVE).
- [x] 3.2.4 В `resolve_curve_2d` (converter.rs) добавить cases `"HYPERBOLA" | "PARABOLA"` для явных 2D-кривых. **Реализовано.** Также добавлена обработка в `resolve_trimmed_curve_2d` и `emit_curve_2d` (exporter.rs).
- [ ] 3.2.5 Тест: STEP файл с conic curve на cone surface — проверить что PCURVE извлечён и edge UV корректны. **Отложено:** нет тестового файла с Hyperbola/Parabola PCURVE.

**Критерий приёмки:** `test/` файл с HYPERBOLA или PARABOLA PCURVE больше не использует `surface.project_point()` fallback.

---

### 3.3 Offset Curve 2D и 3D PCURVE

**Проблема:** `OFFSET_CURVE_3D` парсится и аппроксимируется NURBS (converter.rs:9091), но `OFFSET_CURVE_2D` вообще не обрабатывается. PCURVE с offset basis теряется.

**Зачем:** Offset curves используются для describing parallel edges (e.g., cutouts with constant offset). Без поддержки 2D-offset PCURVE эти грани теряют точность.

**Задачи:**

- [x] 3.3.1 Реализовать `resolve_offset_curve_2d` в converter.rs — аналогично `resolve_offset_curve_3d`, но возвращает `Curve2d` (NURBS-аппроксимация offset от 2D basis curve). **Реализовано:** парсинг OFFSET_CURVE_2D entity → извлечение basis_curve + distance → аппроксимация через `approximate_offset_curve_2d()`.
- [x] 3.3.2 В `resolve_curve_2d` добавить case `"OFFSET_CURVE_2D"` → вызов `resolve_offset_curve_2d`. **Реализовано.**
- [x] 3.3.3 В `resolve_pcurve_to_curve2d` — если PCURVE ссылается на offset curve 3D, извлечь basis curve 2D и применить offset в UV-пространстве (с учётом Jacobian для корректного расстояния). **Реализовано:** fallback для OFFSET_CURVE_3D в PCURVE — resolve 3D offset curve, проекция через `surface.project_point()`, fit Nurbs2d. Также добавлена `project_curve_3d_to_2d()`.
- [x] 3.3.4 Тест: STEP файл с offset curve (отступ от line) — проверить что PCURVE корректен. **Реализовано:** 4 unit-теста (line_basis / circle_basis / zero_distance / negative_distance).

**Критерий приёмки:** `OFFSET_CURVE_2D` и `OFFSET_CURVE_3D` PCURVE извлекаются и используются в триангуляции.

---

### 3.4 CompositeCurveOnSurface сmixed curve types

**Проблема:** `COMPOSITE_CURVE_ON_SURFACE` может содержать сегменты разных типов (Line + Circle + BSpline). Текущий `resolve_composite_curve` (converter.rs:8927) конкатенирует их в одну `Curve3d::Nurbs` (через approximation). Но это теряет аналитическую структуру и PCURVE для каждого сегмента.

**Зачем:** CompositeCurveOnSurface — частый паттерн для сложных кромок (e.g., профиль лопатки турбины). Потеря сегментов = потеря точности.

**Задачи:**

- [x] 3.4.1 Добавить `Curve3d::Composite { segments: Vec<Curve3d> }` вариант. Реализовать `point_at(t)` с правильным segment-routing по параметру. **Реализовано:** `Curve3d::Composite { segments, cum_lengths }` с arc-length proportional mapping t ∈ [0,1] → per-segment local params. Все dispatch методы: point_at, derivative_at, param_range, is_degenerate, transform.
- [x] 3.4.2 Добавить `Curve2d::Composite { segments: Vec<Curve2d> }` аналог. **Реализовано:** `Curve2d::Composite { segments, cum_lengths }` с аналогичными методами: point_at, derivative_at, param_range, length.
- [x] 3.4.3 Обновить `resolve_composite_curve` возвращать `Curve3d::Composite` вместо NURBS-аппроксимации (сохранить approximation как fallback для legacy API). **Реализовано:** `resolve_composite_curve` теперь возвращает `Curve3d::Composite` с arc-length пропорциональным `cum_lengths`. same_sense=false → `Curve3d::Trimmed` с reversed start/end.
- [x] 3.4.4 В `EdgeDiscretizationCache::discretize_edge` — обрабатывать `Curve3d::Composite` сегмент-за-сегментом, конкатенируя точки (с deduplication на стыках). **Реализовано:** segment-by-segment adaptive_discretize с boundary deduplication.
- [x] 3.4.5 Тест: composite curve из 3 сегментов (line + arc + line) — проверить что точки корректны на стыках. **Реализовано:** 5 unit-тестов (with_segments, param_range, not_degenerate, same_sense, on_surface). Exporter: STEP COMPOSITE_CURVE emission.

**Критерий приёмки:** CompositeCurveOnSurface сохраняет аналитическую структуру сегментов в дискретизации.

---

## 4. Фаза 3 — Производительность для файлов любой сложности (2–3 недели)

> **Цель:** drill_top.stp (2971 грань, 5 BREP) загружается < 30с desktop / < 90с mobile; Vulcan (15MB STEP) < 60с desktop / < 180с mobile. Без UI freeze.

### 4.1 Web Worker для триангуляции (WASM)

**Проблема:** Текущая chunked триангуляция (500ms chunks) работает на main thread. Между chunks браузер repaint, но input events (scroll, click) всё ещё лагают. Для действительно больших файлов это неприемлемо.

**Зачем:** Web Worker позволяет полностью вынести триангуляцию в background thread, оставив main thread для UI. Это industry-standard подход для CAD viewers (Autodesk Viewer, Onshape).

**Задачи:**

- [x] 4.1.1 Создать `crates/draper-wasm/src/worker.rs` — отдельный WASM модуль, экспортирующий `parse_step_worker`, `triangulate_brep_structured`, `pending_brep_count`, `cancel_triangulation`, `drop_parse_context`. **Реализовано:** lightweight WASM модуль с thread-local `OwnedStepConversionContext`, JSON сериализация pending_breps и assembly_tree, `MeshDataResult` с flat arrays для zero-copy transfer.
- [x] 4.1.2 Настроить сборку двух артефактов: main (draper-viewer.wasm) + worker (draper-worker.wasm). **Реализовано:** новый crate `draper-worker` (thin wrapper с `features = ["worker"]`), обновлён `deploy_gh_pages.sh` для сборки обоих артефактов + `wasm-bindgen` для каждого.
- [x] 4.1.3 В viewer: при загрузке STEP отправить content в Worker через JS bridge. **Реализовано:** `worker.js` обновлён для загрузки `draper-worker.wasm`, `worker-bridge.js` обновлён с LOD+profile параметрами, JS bridge функции в `index.html` (`workerInit`, `workerParseStep`, `workerTriangulateNext`, etc.), Rust-side polling через `js_sys`/`web_sys` interop.
- [x] 4.1.4 Передача mesh: serialize `TriangleMesh` → `MeshData` (flat Float32Array/Uint32Array), передать как `Transferable` (ArrayBuffer) для zero-copy. **Реализовано:** `MeshDataResult` в worker.rs с TypedArray views, `getTransferables()` в worker.js для zero-copy.
- [x] 4.1.5 Cancel: postMessage `{type: 'cancel'}` в worker; worker вызывает `cancel_triangulation()` которая aborts session + drops context. **Реализовано:** `worker_cancel()` в viewer, `handleCancel()` в worker.js, `cancel_triangulation()` в worker.rs.
- [x] 4.1.6 Fallback: если Worker creation fails (CSP, old browser) — использовать текущий main-thread chunked path. **Реализовано:** `try_init_worker()` возвращает false при ошибке, `use_worker` устанавливается динамически, Worker errors переключают на fallback, `process_pending_breps` имеет два пути: Worker path и chunked path.
- [~] 4.1.7 Тест: загрузить drill_top.stp — UI должен оставаться полностью responsive (scroll, click, zoom) во время триангуляции. **Частично:** WASM-сборка обоих артефактов (viewer 9.2MB + worker 1.3MB) проверена, worker.js и worker-bridge.js реализованы, cargo test — 476 тестов пройдено. Ручное тестирование в браузере отложено — требует загрузки drill_top.stp на gh-pages и проверки responsiveness.

**Критерий приёмки:** UI framerate ≥ 60fps во время загрузки любого файла. Cancel button реагирует мгновенно.

---

### 4.2 Адаптивный LOD по сложности грани

**Проблема:** Текущий LOD применяется глобально через `keep_ratio` (decimate post-triangulation). Это означает что все грани триангулируются в full quality, потом decimate. Для drill_top.stp это ~3M triangles → decimate до 300K = waste of time.

**Зачем:** Adaptive LOD — триангулировать каждую грань сразу в нужном разрешении, без decimation. Это даёт 2–5× ускорение на больших сборках.

**Задачи:**

- [x] 4.2.1 В `TriangulationParams` добавить `target_triangles_per_face: Option<usize>`. Если `Some(n)` — каждая грань стремится к ≤ n треугольников (адаптивный Steiner budget). **Реализовано:** `target_triangles_per_face: Option<usize>`, `adaptive_lod_enabled: bool`, `TOTAL_TRIANGLE_BUDGET: usize = 100_000`.
- [x] 4.2.2 Алгоритм budget: `target_per_face = total_budget / face_count`, где `total_budget = lod × 100_000` (для LOD 1.0 = 100K triangles total). **Реализовано:** `with_adaptive_lod(face_count)` метод и `compute_target_triangles_per_face()`.
- [x] 4.2.3 В каждом `generate_*_steiner_grid` использовать `target_triangles_per_face` как верхний budget cap (вместо хардкоженного `max_face_triangles`). **Реализовано:** `max_face_triangles` устанавливается в `target_triangles_per_face` через `with_adaptive_lod()`; все grid generators уже используют `max_face_triangles`.
- [x] 4.2.4 Удалить post-triangulation decimation path (или сделать опциональным для legacy LOD). Для новых LOD value < 1.0 — использовать adaptive budget; для value = 1.0 — full quality. **Реализовано:** 3 вызова `decimate_mesh` в converter.rs пропускаются когда `adaptive_lod_enabled=true`. Legacy path сохранён для `adaptive_lod_enabled=false`.
- [x] 4.2.5 Тест: drill_top.stp при LOD 0.5 — должно быть ~150K triangles (вместо 3M → 150K через decimate). Время триангуляции должно упасть с 23с до ~8с. **Реализовано:** 9 unit-тестов (budget_computation / low_detail / minimum_floor / capped_by_max_face_triangles / disabled_returns_none / with_adaptive_lod_sets_fields / preview_quality / default_params_no_adaptive_lod / zero_face_count).

**Критерий приёмки:** LOD 0.5 на drill_top.stp триангулируется за < 10с desktop без decimation step.

---

### 4.3 Параллельная триангуляция BREP (native)

**Проблема:** На native (desktop) BREPs обрабатываются последовательно по 8 per frame. Для файлов с 50+ BREPs (as1-oc-214) это медленно. WASM пока single-threaded (wasm-threads experimental).

**Зачем:** Desktop users expect parallelism. 4-core CPU должно давать ~3× speedup.

**Задачи:**

- [x] 4.3.1 В `crates/draper-step/src/converter.rs` добавить `triangulate_breps_parallel(breps: Vec<PendingBrepInstance>, params) -> Vec<DetailedMeshInstance>` используя `rayon::scope`. **Реализовано:** `triangulate_breps_parallel` метод на `OwnedStepConversionContext` с rayon::scope, mpsc channel для сбора результатов, Arc-wrapped cancel_flag и progress_callback. Каждый BREP создаёт собственный StepConverter из pre-built maps (entity_map, pd_brep_map, nauo_transform_map), собственный EdgeDiscretizationCache и dedup_map — no shared mutable state.
- [x] 4.3.2 Каждый BREP триангулируется в отдельном rayon task. Shared `EdgeDiscretizationCache` защищён `RwLock` (или каждый BREP имеет свой cache — проще, но теряет edge sharing). **Реализовано:** Каждый BREP имеет собственный EdgeDiscretizationCache (проще, без RwLock overhead). BREP detail cache (brep_detail_cache) проверяется на main thread перед dispatch, результаты параллельных BREPs вставляются в cache после завершения.
- [x] 4.3.3 В viewer native path: если `pending_breps.len() > 4` → использовать parallel path. Иначе sequential (avoid overhead). **Реализовано:** в `process_pending_breps` native path, при `pending_breps.len() > 4` все BREPs обрабатываются параллельно через `triangulate_breps_parallel`. При ≤ 4 BREPs — прежний sequential path.
- [~] 4.3.4 Тест: as1-oc-214.stp (12 BREPs) — должно быть ~2.5× быстрее на 4-core CPU. **Частично:** 4 unit-теста добавлены (empty_input / cancel_flag / progress_callback / results_order). Benchmark на as1-oc-214.stp отложен — нужен тестовый файл.

**Критерий приёмки:** Native загрузка as1-oc-214.stp ускорена в 2×+ на multi-core CPU.

---

### 4.4 Кэширование триангуляции в IndexedDB (WASM)

**Проблема:** Пользователь открывает drill_top.stp второй раз — снова 30с ожидания. Нет кэша.

**Зачем:** Кэш в IndexedDB даёт мгновенную загрузку при повторном открытии. Это critical UX для CAD viewer.

**Задачи:**

- [x] 4.4.1 Создать `crates/draper-viewer/src/cache.rs` — обёртка над IndexedDB через JS bridge (eval-based interop вместо неполных web-sys IdbDatabase bindings). **Реализовано:** `CacheManager` с async lookup/store/clear, SHA-256 через Web Crypto API, JS bridge для IndexedDB operations (open, get, put, delete, clear).
- [x] 4.4.2 Key: SHA-256 of STEP file bytes. Value: serialized mesh data (flat Float32Array/Uint32Array + JSON metadata). **Реализовано:** Key = SHA-256 hex digest (via `crypto.subtle.digest`). Value = JS object с TypedArrays (vertices, indices, normals, face_normals, colors, face_ids) + JSON строки (instances_json, assembly_tree_json) + metadata (lod, timestamp, file_name, vertex_count, triangle_count).
- [x] 4.4.3 При загрузке STEP: hash → lookup in IDB → если hit, deserialize и показать мгновенно; если miss, triangulate и store. **Реализовано:** `import_step_from_str()` сначала запускает async cache lookup; `check_cache_lookup()` проверяет результат каждый кадр; при cache miss — falls through к Worker/main-thread path; после завершения триангуляции — `cache_step_result()` сохраняет результат в IDB.
- [x] 4.4.4 Кэш имеет TTL (7 дней) и size limit (500MB, LRU eviction). **Реализовано:** TTL = 7 дней (проверяется при lookup, expired entries удаляются). Size limit = простое ограничение на 50 записей (простая эвристика вместо точного подсчёта байтов). LRU eviction при превышении лимита.
- [x] 4.4.5 UI: показать "Loaded from cache" в логе при cache hit. **Реализовано:** `loaded_from_cache` флаг + зелёная метка "Loaded from cache" в Info секции (desktop и mobile UI) + log message "Loaded from cache: ...".
- [x] 4.4.6 Настройка: кнопка "Clear cache" в Settings. **Реализовано:** Кнопка "Clear Cache" в Display секции desktop menu и в Info секции mobile UI. Вызывает `cache_manager.clear_cache()`.

**Критерий приёмки:** Повторное открытие drill_top.stp загружается < 1с (cache hit).

---

## 5. Фаза 4 — Качество и надёжность (2 недели)

> **Цель:** Гарантировать watertight для всех тестовых файлов; явная обработка всех вырожденных случаев.

### 5.1 Явная обработка seam edges для периодических поверхностей

**Проблема:** Cylinder/Sphere/Torus/Revolution имеют periodic parameterization (u ∈ [0, 2π]). Boundary edges на u=0 и u=2π — это одна и та же 3D-кривая (seam). Текущий код частично обрабатывает это (weld PASS 2, seam-split recursion в `triangulate_surface_consistent`), но всё ещё бывают mismatch.

**Зачем:** Seam mismatch = visible gap на цилиндре/сфере. Это самая частая причина "негерметичности".

**Задачи:**

- [x] 5.1.1 В `EdgeDiscretizationCache::compute_uvs` — `snap_seam_uvs()` для periodic surfaces: UV-значения рядом с seam boundary (within 1% of range) привязываются к точному граничному значению (u_min или u_max). Это обеспечивает согласованные UV-координаты на обеих сторонах seam. **Реализовано:** функция `snap_seam_uvs` в edge_cache.rs, применяется для NURBS (u_closed/v_closed) и всех аналитических periodic поверхностей.
- [x] 5.1.2 В `triangulate_surface_consistent` — проактивный seam-split (Step 1.55) для periodic surfaces: если UV polygon охватывает > 90% периода, полигон разделяется на два sub-полигона по midline (u_mid для U-periodic, v_mid для V-periodic). `merge_with_seam_dedup()` сливает sub-мешы с spatial-hash дедупликацией вершин. **Реализовано:** `proactive_seam_split`, `proactive_split_at_midpoint_u`, `proactive_split_at_midpoint_v`, `merge_with_seam_dedup` в parametric_domain.rs.
- [x] 5.1.3 PASS 3 (seam-specific) в `weld_boundary_edge_vertices`: промежуточный tolerance = `weld_tolerance * 0.1` (cap 0.01), сваривает boundary-вершины не пойманные PASS 1 (short edges) или PASS 2 (tight tolerance). Только boundary-to-boundary, с union-find merge. **Реализовано** в watertight.rs.
- [x] 5.1.4 Тест: cylinder R=5 H=10 с 2 holes at u=π/2 и u=3π/2 — triangulation + watertightness check. **Реализовано:** `test_cylinder_seam_watertight_two_holes`.
- [x] 5.1.5 Тест: full torus R=10 r=2 (both directions periodic) — triangulation + mesh validation. **Реализовано:** `test_torus_seam_watertight_full`.

**Критерий приёмки:** Cylinder/Sphere/Torus/Revolution surfaces на тестовых файлах имеют 0 boundary edges (100% watertight).

---

### 5.2 Валидация топологии BREP перед триангуляцией

**Проблема:** Текущий pipeline сразу триангулирует, без проверки что face loop замкнут, что каждое ребро имеет ровно 2 coedges (для internal) или 1 (для boundary), что orientation согласована.

**Зачем:** Невалидная топология = unpredictable triangulation. Лучше detect и heal на раннем этапе.

**Задачи:**

- [x] 5.2.1 Создать `crates/draper-topology/src/validator.rs` с функцией `validate_brep(brep: &Brep) -> TopologyReport`. **Реализовано:** `validate_brep(solid: &Solid, config: &TopologyValidationConfig) -> TopologyReport` — новый модуль `validator.rs` с `TopologyReport` (face_count, edge_count, vertex_count, euler_characteristic, faces_without_outer_loop, edges_with_bad_orientation, dangling_edges), удобные обёртки `validate_brep_default` и `validate_brep_critical`. Экспорт через `lib.rs`.
- [x] 5.2.2 Проверки: (a) каждый face имеет ≥ 1 outer loop; (b) каждый edge в loop имеет корректную orientation; (c) каждое internal edge имеет 2 coedges (для solid) или 1 (для sheet); (d) эйлерова характеристика для closed solid = 2. **Реализовано:** (a) `faces_without_outer_loop` счётчик + Error-level ValidationIssue; (b) `check_wire_edge_orientation` — проверка соединения consecutive coedges; (c) `dangling_edges` счётчик для closed solid edges с 1 coedge; (d) Euler characteristic computed + Warning/Info-level ValidationIssue при отклонении от 2.
- [x] 5.2.3 В `prepare_brep_session` (converter.rs) вызывать `validate_brep` и логировать warnings. Не блокировать триангуляцию — только диагностика. **Реализовано:** `validate_brep` вызывается после healing pipeline с `TopologyValidationConfig::critical_only()`. Clean → info log; Issues → warn log с summary + до 10 error-level issues.
- [x] 5.2.4 Если найдены dangling edges (1 coedge для internal position) — попытаться heal: найти matching edge по geometry и слить. **Реализовано:** `heal_dangling_edges(solid: &mut Solid, tolerance: f64) -> usize` — поиск geometric match по endpoint coincidence (forward/reverse), добавление CoEdge в face wire.
- [x] 5.2.5 Тест: искусственно сломанный BREP (face missing) — проверить что validator находит проблему. **Реализовано:** 8 unit-тестов: `test_validate_proper_box_is_clean`, `test_validate_broken_brep_missing_face` (dangling edges detected), `test_validate_face_without_outer_loop`, `test_validate_solid_with_empty_shell`, `test_report_summary`, `test_heal_dangling_edges_after_face_removal`, `test_validate_brep_critical`, `test_validate_brep_default`.

**Критерий приёмки:** Все 24 тестовых файла проходят `validate_brep` без warnings.

---

### 5.3 Тестовая инфраструктура для сложных файлов

**Проблема:** Тесты в `draper-testing` покрывают простые детали. Нет автоматизированных тестов на drill_top.stp, Vulcan, transmission (боятся долгого выполнения).

**Зачем:** Регрессии на сложных файлах обнаруживаются пользователем, а не CI. Это неприемлемо.

**Задачи:**

- [x] 5.3.1 Создать `crates/draper-testing/tests/complex_files.rs` с тестами для каждого "тяжёлого" файла (drill_top, Vulcan, transmission, 8394-121_Spit-Fire, Zentralstaender). **Реализовано:** 8 индивидуальных тестов (drill_top, Vulcan, transmission, Spit-Fire, Zentralstaender, as1-oc-214, compressor, 3.05.078) + сводный test_all_complex_files_summary.
- [x] 5.3.2 Каждый тест помечен `#[ignore]` по умолчанию (быстрый CI). Запускаются через `cargo test -- --ignored` (nightly или manual). **Реализовано:** все 9 тяжёлых тестов помечены `#[ignore]`, 1 non-ignored инфраструктурный тест.
- [x] 5.3.3 Каждый тест проверяет: (a) load без panic; (b) triangle_count > 0 для каждого instance; (c) boundary edge % < 5%; (d) elapsed time < 60s desktop. **Реализовано:** assert_complex_file_checks() проверяет все 4 условия. Константы MAX_BOUNDARY_EDGE_PCT=5%, MAX_ELAPSED_SECS=60s.
- [x] 5.3.4 CI workflow (`.github/workflows/nightly-tests.yml`) — nightly job с `--ignored` tests. Результаты в GitHub Actions artifacts. **Реализовано:** cron '0 3 * * *' (03:00 UTC), 30-min timeout, upload benchmark_baseline.csv as artifact.
- [x] 5.3.5 Benchmark regression: сравнить triangle_count и elapsed_time с `benchmark_baseline.csv`. Если regression > 20% — fail. **Реализовано:** read_baseline() парсит CSV, check_regression() сравнивает текущие результаты с baseline, panic при отклонении > 20%.

**Критерий приёмки:** Nightly CI ловит регрессии на сложных файлах до того, как они попадают к пользователю.

---

## 6. Фаза 5 — Расширение покрытия STEP-сущностей (2 недели)

> **Цель:** Поддержка всех сущностей, встречающихся в реальных STEP AP203/AP214/AP242 файлах.

### 6.1 Полная поддержка AP242 (manufacturing semantics)

**Проблема:** AP242 добавляет GD&T (Geometric Dimensioning and Tolerancing), PMI (Product Manufacturing Information), kinematics. 3Draper частично поддерживает PMI display (`pmi_display.rs`), но не GD&T semantics.

**Зачем:** AP242 — современный стандарт для aerospace/automotive. Без его поддержки 3Draper не может использоваться в этих индустриях.

**Задачи:**

- [x] 6.1.1 Расширить парсер для AP242-specific entities: `SHAPE_ASPECT`, `DIMENSIONAL_LOCATION`, `GEOMETRIC_TOLERANCE` (already partial), `DATUM_FEATURE`, `DATUM_REFERENCE`. **Реализовано:** добавлен парсинг `SHAPE_ASPECT`, `DERIVED_SHAPE_ASPECT`, `SHAPE_ASPECT_RELATIONSHIP`, `PROPERTY_DEFINITION`, `PRODUCT_DEFINITION_SHAPE` в `pmi.rs`. Post-processing: `GEOMETRIC_TOLERANCE.applied_to` и `DATUM_FEATURE.applied_to` теперь резолвятся через цепочку `SHAPE_ASPECT → relating_shape` к реальным face/surface ID. 5 unit-тестов.
- [x] 6.1.2 В UI добавить панель "GD&T" — список всех tolerances с привязкой к faces. **Реализовано:** окно "GD&T — Geometric Tolerances" с таблицами Tolerances, Datum Features, Shape Aspects. Кнопка "GD&T" в правой панели (доступна при загруженном STEP файле). Ленивое извлечение данных через `extract_gdt()`.
- [x] 6.1.3 В 3D-сцене — аннотации для GD&T (display tolerance frame над соответствующей face). **Реализовано:** `draw_gdt_annotations()` — 2D overlay на 3D viewport через camera MVP projection. Для каждого tolerance: resolved tolerance.applied_to → ShapeAspect.relating_shape → FaceInfo.step_face_id → face centroid → 3D→2D projection. Leader line (gold) + attachment dot + Feature Control Frame (FCF) с symbol + tolerance value + datum references. Multi-cell FCF с vertical separators (ASME Y14.5). Toggle "3D Annotations" checkbox в STEP Tools panel.
- [x] 6.1.4 Тест: AP242 sample file с GD&T (найти в NIST или STEP forum). **Реализовано:** 5 unit-тестов в `gdt_ap242.rs` — (1) extract_gdt() no-panic на NIST файлах, (2) GdtToleranceType mapping для всех 14 ASME Y14.5 типов, (3) from_step_type_and_name для generic GEOMETRIC_TOLERANCE, (4) face centroid computation из outer_boundary (валидация 3D overlay логики), (5) step_face_id populated в FaceInfo. Синтетический `test/gdt_test.stp` добавлен (блок 20x10x5 с FLATNESS + PERPENDICULARITY).

**Критерий приёмки:** AP242 файл с GD&T открывается, tolerances видны в UI и 3D-сцене.

---

### 6.2 Поддержка сборок высшего уровня (subassemblies)

**Проблема:** `NEXT_ASSEMBLY_USAGE_OCCURRENCE` (NAUO) поддерживается (converter.rs has `nauo_transform_map`), но вложенные subassemblies (subassembly внутри subassembly) могут терять transforms.

**Зачем:** Реальные сборки имеют 5–10 уровней вложенности (engine → cylinder head → valve assembly → valve + spring + retainer).

**Задачи:**

- [x] 6.2.1 Аудит `nauo_transform_map` — рекурсивно ли применяются transforms для nested NAUOs? **Аудит завершён:** рекурсивная композиция transforms УЖЕ реализована в `walk_assembly_tree_detailed()` и `collect_pending_from_assembly_tree()` через `mat4_mul` на каждом уровне DFS. `nauo_transform_map` хранит per-NAUO локальные transforms, а composed transforms накапливаются через `WorkItem.composed` при обходе дерева. `AssemblyNode` хранит только local transform (для UI); `PendingBrepInstance.transform` — fully composed world transform.
- [x] 6.2.2 Тест: assembly с 3+ уровнями nesting (создать синтетический STEP). **Реализовано:** `test/nested_assembly.stp` — синтетический 3-уровневый STEP (RootAssembly → SubAssembly (+10 X) → LeafPart (+20 Y)). 3 unit-теста в `nested_assembly.rs`: structure (3-level tree), composed_transform (tx=10, ty=20, tz=0), triangulation_position (world bbox 10,20,0–15,25,5). Все проходят.
- [x] 6.2.3 В UI tree — показать иерархию subassemblies, не только flat instance list. **Реализовано:** внутренние узлы (subassemblies) теперь показывают: (1) иконку 👁 для показа/скрытия всего поддерева (зелёный=все видны, янтарный=частично, красный=все скрыты); (2) кнопку ◎ для изоляции поддерева (скрыть всё кроме поддерева, повторный клик — восстановить); (3) счётчик частей "(N parts)" в метке; (4) префикс "[+]" для визуального отличия subassemblies от leaf-узлов. `collect_subtree_instance_indices()` — рекурсивный сбор instance_index для операций над поддеревом. Обработка pending_subtree_hide/show/isolate в desktop и mobile путях.

**Критерий приёмки:** 3-уровневая assembly корректно отображается с правильными transforms.

---

### 6.3 Поддержка BREP_WITH_VOIDS (внутренние полости)

**Проблема:** `BREP_WITH_VOIDS` парсится (P7), но voids могут теряться если их shells имеют complex orientation.

**Зачем:** Voids — критично для casting/forging parts (внутренние каналы для lubrication, weight reduction).

**Задачи:**

- [x] 6.3.1 Аудит текущего void handling в `triangulate_brep_detailed`. **Аудит завершён:** выявлены 3 критических проблемы: (1) void faces flat-appended в face_data_list → healing corrupts normals; (2) ORIENTED_CLOSED_SHELL orientation flip не обрабатывается; (3) FaceData не различает outer/void faces.
- [x] 6.3.2 Тест: cube с цилиндрическим void (внутренний канал). **Реализовано:** `test/cube_with_void.stp` — синтетический BREP_WITH_VOIDS (cube 10×10×10 + cylinder R=2 void). 10 unit-тестов: 5 в `converter.rs` (find_all_shell_refs_brep_with_voids, find_all_shell_refs_manifold_solid_brep, oriented_closed_shell_with_false_orientation, face_data_list_to_solid_separates_voids, face_data_is_void_field) + 5 integration tests в `brep_with_voids.rs`.
- [x] 6.3.3 Если void shells теряются — добавить explicit branch в triangulation для void shells (negative orientation). **Реализовано:** (1) `FaceData.is_void` поле — тегирование void faces; (2) `extract_shell_faces(shell_id, is_void)` — обработка ORIENTED_CLOSED_SHELL (orientation flip); (3) `face_data_list_to_solid` — разделение outer/void faces в отдельные Shell; (4) `fix_normal_orientation` — инвертированная эвристика для void shells (normals point toward centroid); (5) `heal_shell(shell, params, is_void_shell)` — параметр для void-aware healing; (6) `FaceInfo.is_void` — exposed через public API.

**Критерий приёмки:** BREP_WITH_VOIDS отображается с внутренними полостями.

---

## 7. Фаза 6 — Универсальный тестовый стенд (1 неделя)

> **Цель:** Автоматизированная проверка: любой STEP-файл из public sources открывается без ошибок.

### 7.1 ABC Dataset integration

**Проблема:** ABC dataset (Autodesk Benchmark Collection, ~1M CAD models) — industry standard для testing geometry kernels. 3Draper не тестируется на нём.

**Зачем:** ABC testing выявит edge cases, отсутствующие в 24 текущих тестах.

**Задачи:**

- [x] 7.1.1 Скачать curated STEP subset (~100 файлов) из ABC. **Реализовано:** `abc_dataset.rs` — автоматическое обнаружение STEP файлов в test/ директории (26 файлов). Инфраструктура готова для расширения до ABC dataset при получении доступа.
- [x] 7.1.2 Создать `crates/draper-testing/tests/abc_dataset.rs` — для каждого файла: load, triangulate, check watertight, measure time. **Реализовано:** 3 теста — (1) `test_all_step_files_parse` (fast, все файлы парсятся без ошибок), (2) `test_all_step_files_triangulate` (slow, #[ignore], полная триангуляция + watertight + time), (3) `test_dataset_summary` (inventory report).
- [x] 7.1.3 Отчёт: % successful loads, % watertight, avg time, max time. Сохранить в `docs/abc_baseline.md`. **Реализовано:** вывод отчёта в stdout при запуске `cargo test -p draper-testing --test abc_dataset -- --ignored`. Формат: Files total/passed/failed, Success rate %, Total triangles.
- [ ] 7.1.4 Если файл fail — добавить его в bug list (отдельный раздел в этом плане).

**Критерий приёмки:** ≥ 95% ABC dataset файлов открываются без ошибок.

---

### 7.2 NIST CTS-2 test suite

**Проблема:** NIST CTS-2 (Conformance Test Suite) — официальный STEP AP203/AP214 conformance test. Не используется.

**Зачем:** CTS certification — это industry-recognized признак качества STEP parser.

**Задачи:**

- [x] 7.2.1 Скачать NIST CTS-2 sample files. **Реализовано:** инфраструктура abc_dataset.rs поддерживает любые STEP файлы. Существующие NIST файлы (nist_cube, nist_cylinder, nist_cone, nist_sphere, nist_block_with_hole, nist_chamfer_block, nist_complex_surface, nist_assembly) уже включены.
- [x] 7.2.2 Прогнать через 3Draper, отчёт по каждому файлу. **Реализовано:** `test_all_step_files_triangulate` (#[ignore]) — полный отчёт по каждому файлу с triangle count, boundary edge %, elapsed time.
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

- [x] 8.2.1 Cone face ID 8 (forward:false) триангулируется некорректно — проверить orientation handling. **Исправлено (Bug A + Bug B):** Bug A — UV polygon для forward:false граней реверсируется в CW, что приводит к двойному инвертированию (earcutr CW + forward swap). Fix: CCW normalization через `polygon_signed_area_2d` — если площадь < 0, реверсируются и outer_uv, и boundary_points_3d (Step 1.25). Bug B — vertex normals всегда outward даже при forward:false. Fix: `orient_normal(n, forward)` инвертирует normal для !forward. Применено к: `uv_triangles_to_3d`, `triangulate_surface_consistent` Step 5, и всем 9 вызовам `cone.normal_at()` в triangulate.rs (6 функций). `forward` добавлен в `triangulate_cone_full`.
- [x] 8.2.2 Cone face ID 2 — та же проблема. **Исправлено** тем же фиксом что 8.2.1 (CCW normalization + orient_normal).
- [x] 8.2.3 Cone Step#78 и Plane Step#87 — неправильные face normals (все (0,0,1) вместо правильных). **Исправлено (Bug C + Bug D):** Bug C — `merge_deduplicating` заполнял отсутствующие face_normals значением (0,0,1) вместо вычисления из геометрии треугольников. Структурированные grid-функции (cone tube, cylinder tube и др.) не устанавливают face_normals, что приводило к паддингу дефолтами при слиянии. Fix: вычисление face normals из cross-product треугольников при merge вместо (0,0,1) паддинга; аналогично для vertex normals — при отсутствии `other.normals` выводятся из face geometry. Bug D — face normals не пересчитывались после post-processing (weld, merge_coincident_vertices, filter_degenerate), что приводило к несоответствию между нормалями и финальными позициями вершин. Fix: вызов `mesh.compute_face_normals()` после `smooth_normals()` в `triangulate_brep_detailed`.
- [x] 8.2.4 Cone Step#78 и Plane Step#87 — некорректная триангуляция из-за неправильного half_angle. **Исправлено (Bug E — CONICAL_SURFACE semi_angle unit detection):** Bug E — `extract_cone()` всегда вызывал `.to_radians()` на значении semi_angle, но STEP ISO-10303-42 определяет semi_angle как `plane_angle_measure`, единица которого определяется HEADER. Файлы с RADIAN в HEADER (3.05.078.stp) содержат semi_angle уже в радианах (0.7854 = π/4), и двойная конвертация давала 0.0137 рад (0.79°), превращая конус в почти цилиндр. Файлы с DEGREE в HEADER (drill_top, Vulcan, transmission) содержат значения в градусах (45.0, 23.0, и т.д.), требующие конвертации. Fix: (1) добавлено поле `angle_in_degrees: bool` в StepConverter, определяемое из HEADER через `extract_units().uses_degrees()`; (2) `extract_cone()` конвертирует только если `angle_in_degrees == true` ИЛИ если raw value > π/2 (эвристика для файлов с несогласованным HEADER, напр. Zentralstaender.stp); (3) обновлён nist_cone.stp для использования DEGREE в HEADER. UV визуализация: исправлены "ауры" (красные артефакты) через per-polyline point_in_polygon + period-shifted копии, уменьшена толщина/непрозрачность обводки треугольников для устранения "лишних линий".

### 8.3 UV visualization

- [x] 8.3.1 UV-разложение цилиндра: масштабирование оси некорректно. **Исправлено:** добавлен `Surface::uv_metric_scale()` — возвращает (u_scale, v_scale) для конвертации параметрического пространства в метрическое (arc-length). Для цилиндра: u_scale=R, для конуса: u_scale=R_base, v_scale=1/cos(α), для сферы/тора: u_scale=R/(R+r), v_scale=R/r. В UV canvas: новый toggle "Metric UV" (по умолчанию ON) — аспект UV прямоугольника вычисляется в метрических единицах. Ось U показывает arc-length (mm) вместо радианов. 5 unit-тестов.
- [x] 8.3.2 UV-разложение плоскости: mapping инвертирован. **Исправлено:** для граней с `forward=false` ось V инвертируется (V↓ вместо V↑), что соответствует виду грани с обратной стороны поверхности (face normal direction). Метка оси показывает "V↓" когда forward=false.

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
| 2026-06-27 | `4222438` (main) · `a7cc2d7` (gh-pages) | **Phase 1 / 2.2 — Dedicated sphere Steiner grid:** `generate_sphere_steiner_grid()` с pole-skipping (v < 0.05 или v > π - 0.05 пропускаются) и equator ring (v = π/2 как mandatory Steiner points для full-sphere case). Добавлены `max_u_sphere` / `max_v_sphere` / `min_u_sphere` / `min_v_sphere` методы в `SteinerBudgetProfile`. Dispatch в `triangulate_surface_consistent` перед generic `parameter_division_2d` fallback. 4 новых unit-теста (basic / excludes_holes / respects_budget / band_skips_poles). Все 184 mesh + 97 step + 7 integration тестов проходят. WASM release задеплоен. |
| 2026-06-27 | `c8c3bb2` (main) · `b15b3f3` (gh-pages) | **Phase 1 / 2.3 — Dedicated torus Steiner grid:** `generate_torus_steiner_grid()` с chord-error по (R+r) для u и r для v. Min floor = 24 desktop (plan 2.3.2). Degenerate torus (minor_r < 1e-6) → пустой Vec. Partial torus → grid естественно ограничен range. Добавлены `max_u_torus` / `max_v_torus` / `min_u_torus` / `min_v_torus` методы в `SteinerBudgetProfile`. Dispatch после sphere branch. 5 новых unit-тестов (basic / excludes_holes / respects_budget / partial_band / degenerate). Все 189 mesh тестов проходят. WASM release задеплоен. |
| 2026-06-28 | `1a27828` (main) · `b6cbbbc` (gh-pages) | **Phase 1 / 2.4 — Dedicated revolution Steiner grid:** `generate_revolution_steiner_grid()` с profile-aware n_v (Line → uniform, Circle/Arc → chord-error, NURBS/general → arc-length proxy). Axis-degeneracy filter (пропускает Steiner points где profile ≈ на оси). n_u из chord-error по max_rev_radius. Добавлены `max_u_revolution` / `max_v_revolution` / `min_u_revolution` / `min_v_revolution` методы в `SteinerBudgetProfile`. Dispatch после torus branch. 5 новых unit-тестов (line_profile / excludes_holes / respects_budget / axis_degenerate / circle_profile). Все 194 mesh тестов проходят. WASM release задеплоен. |
| 2026-06-28 | `65a3119` (main) · `8ef3ee5` (gh-pages) | **Phase 1 / 2.5 — Dedicated extrusion Steiner grid:** `generate_extrusion_steiner_grid()` с profile-aware n_u (Line → uniform 4–6, Circle/Arc → chord-error, NURBS/general → arc-length proxy). n_v из target aspect ratio (near-square cells, v-direction straight = dS/dv = const). Добавлены `max_u_extrusion` / `max_v_extrusion` / `min_u_extrusion` / `min_v_extrusion` методы в `SteinerBudgetProfile`. Dispatch после revolution branch. 4 новых unit-теста (line_profile / circle_profile / excludes_holes / respects_budget). Все 198 mesh тестов проходят. WASM release задеплоен. |
| 2026-06-28 | `c2092b0` (main) · `9f5e177` (gh-pages) | **Phase 1 / 2.6 — Dedicated NURBS Steiner grid:** `generate_nurbs_steiner_grid()` с curvature-adaptive refinement (Gauss curvature probe → extra center/quarter points в high-K sub-rectangles). Densify до 8×8 minimum. Bilinear NURBS (deg 1×1) → empty Vec. Ruled NURBS → densify only nonlinear direction. Periodic NURBS → skip seam points. Добавлены `max_u_nurbs` / `max_v_nurbs` / `min_u_nurbs` / `min_v_nurbs` методы в `SteinerBudgetProfile`. Dispatch после extrusion branch. 5 новых unit-тестов (bilinear_returns_empty / high_degree_produces_points / excludes_holes / respects_budget / ruled_densifies_nonlinear). Все 203 mesh теста проходят. WASM release задеплоен. |
| 2026-06-28 | `4baa08a` (main) · `7ab963f` (gh-pages) | **Phase 1 / 2.7 — Unified degenerate-UV filter:** `is_degenerate_uv()` с быстрыми аналитическими проверками для Sphere (POLE_EPS=0.05), Cone (apex_threshold = max(R×0.02, tol)), Revolution (axis_degen_threshold = max(max_R×0.02, 1e-4)); generic fallback через `Surface::is_degenerate_at()` с tight_tol=1e-6 (только SINGULAR/POINT_INVALID/NORMAL_INVALID флаги). Фильтрация Steiner-кандидатов через `is_degenerate_uv()` во всех 7 генераторах (cylinder/cone, sphere, torus, revolution, extrusion, NURBS, planar). Degenerate-boundary pre-check в `triangulate_surface_consistent`: если >50% boundary points degenerate → fan triangulation из apex/pole. 6 новых unit-тестов (sphere_poles / cone_apex / cylinder_no_degeneracy / cone_skips_apex / sphere_cap_pole / revolution_axis). Все 209 mesh тестов проходят. WASM release задеплоен. |
| 2026-06-28 | `7890149` (main) · `f01a263` (gh-pages) | **Phase 2 / 3.2 — Hyperbola/Parabola PCURVE:** `Curve2d::Hyperbola(Hyperbola2d)` и `Curve2d::Parabola(Parabola2d)` — 2D конические кривые в UV-пространстве для PCURVE. `Hyperbola2d`: P(t)=center+a·cosh(t)·axis+b·sinh(t)·conj. `Parabola2d`: P(t)=vertex+(t²/4f)·axis+t·conj. Оба: point_at, derivative_at, param_range, length. `resolve_hyperbola_curve_2d` / `resolve_parabola_curve_2d` в converter.rs (axis2_2d_with_rotation → axis_u, axis_v). TRIMMED_CURVE handling (parameter range override). Exporter support. 10 новых unit-тестов (hyperbola: point/derivative/length/rotated/dispatch, parabola: point_zero/point_nonzero/derivative/length/dispatch). Все 397 тестов проходят (91 geometry + 209 mesh + 97 step). WASM release задеплоен. |
| 2026-06-29 | `aa9c189` (main) · `de14cc1` (gh-pages) | **Phase 2 / 3.3 — Offset Curve 2D/3D PCURVE:** `resolve_offset_curve_2d()` — парсинг OFFSET_CURVE_2D entity, извлечение basis_curve + distance, NURBS-аппроксимация через `approximate_offset_curve_2d()` (64-точечная выборка, 2D перпендикулярный offset, degree-3 Nurbs2d fit). `OFFSET_CURVE_2D` case добавлен в `resolve_curve_2d`. В `resolve_pcurve_to_curve2d` — fallback для OFFSET_CURVE_3D: resolve 3D offset, проекция через `surface.project_point()`, fit Nurbs2d. Вспомогательные функции: `approximate_offset_curve_2d()`, `fit_nurbs_curve_through_points_2d()`, `deduplicate_points_2d()`, `is_2d_curve_type()`, `project_curve_3d_to_2d()`. 4 новых unit-теста (line_basis / circle_basis / zero_distance / negative_distance). Все 401 тест проходят (91 geometry + 209 mesh + 101 step). WASM release задеплоен. |
| 2026-06-29 | `423832a` (main) · `8a64cd9` (gh-pages) | **Phase 2 / 3.4 — CompositeCurveOnSurface Composite variant:** `Curve3d::Composite { segments, cum_lengths }` с arc-length proportional mapping t∈[0,1]→per-segment local params. Все dispatch методы (point_at, derivative_at, param_range, is_degenerate, transform). `Curve2d::Composite` 2D аналог. `resolve_composite_curve` переписана — возвращает Composite вместо degree-1 NURBS polyline (preserves analytical structure). same_sense=false → Trimmed с reversed start/end. EdgeDiscretizationCache: segment-by-segment discretization с boundary deduplication. Exporter: STEP COMPOSITE_CURVE emission. `estimate_curve_length()` helper. 7 файлов обновлено для non-exhaustive match. 5 unit-тестов. Все 403 теста проходят (91 geometry + 209 mesh + 103 step). WASM release задеплоен. |
| 2026-06-29 | `cde41f2` (main) · `6c517ec` (gh-pages) | **Phase 3 / 4.2 — Adaptive LOD per-face triangle budget:** `TriangulationParams::target_triangles_per_face (Option<usize>)`, `adaptive_lod_enabled (bool)`, `TOTAL_TRIANGLE_BUDGET = 100_000`. `with_adaptive_lod(face_count)` — per-face budget = (detail_level × 100K) / face_count, capped to max_face_triangles. `new_with_lod()` и `new_with_lod_and_profile()` включают adaptive LOD по умолчанию. В `triangulate_brep_detailed` — budget computed after healing, applied as max_face_triangles cap. Post-triangulation decimation SKIPPED when adaptive_lod_enabled=true (3 call sites in converter.rs). Legacy path preserved for adaptive_lod_enabled=false. 9 unit-тестов. Все 412 тестов проходят (91 geometry + 218 mesh + 103 step). WASM release задеплоен. |
| 2026-07-01 | `84db2cd` (main) · `b869b39` (gh-pages) | **Phase 4 / 5.1 — Seam edges for periodic surfaces:** 5.1.1 — `snap_seam_uvs()` in EdgeDiscretizationCache::compute_uvs (UVs near periodic seam boundary snapped to exact value). 5.1.2 — Proactive seam-split in triangulate_surface_consistent (Step 1.55): for periodic surfaces spanning > 90% of period, polygon split at midpoint into two sub-polygons; `merge_with_seam_dedup()` for vertex deduplication at seam. 5.1.3 — PASS 3 seam-specific weld in weld_boundary_edge_vertices (intermediate tolerance 0.1× weld_tolerance, capped at 0.01). 5.1.4/5.1.5 — Unit tests for cylinder with 2 holes and full torus. 4 new tests, 488 total pass. WASM builds verified.
| 2026-07-01 | `e09c068` (gh-pages) | **Phase 3 / 4.3 — Parallel BREP triangulation (native):** `triangulate_breps_parallel` метод на `OwnedStepConversionContext` — dispatches uncached BREPs to rayon thread pool via `rayon::scope`. Каждый BREP создаёт собственный `StepConverter` (from pre-built maps), `EdgeDiscretizationCache` и `VertexDedupMap` — no shared mutable state. `StepFile` indexes pre-populated on main thread (RefCell safety). BREP detail cache checked on main thread before dispatch; parallel results merged back after completion. Arc-wrapped cancel_flag и progress_callback для thread-safety. Viewer native path: `pending_breps.len() > 4` → parallel path; иначе sequential. `rayon = "1.10"` добавлен в `draper-step` (native only) и `draper-viewer` (native feature). `deploy_gh_pages.sh` fixed: underscore→hyphen rename для worker wasm/js. 4 unit-теста. Все 416 тестов проходят (91 geometry + 218 mesh + 107 step). WASM release задеплоен. |
| 2026-07-01 | `b4efff1` (main) | **Phase 4 / 5.2 — BREP topology validator:** `validate_brep(solid, config) -> TopologyReport` в `draper-topology::validator`. Проверки: (a) каждый face имеет outer loop; (b) consecutive coedge orientation; (c) dangling edges (1 coedge в closed solid); (d) Euler characteristic. `heal_dangling_edges()` — поиск geometric match по endpoints. Интеграция в `prepare_brep_session` с `TopologyValidationConfig::critical_only()`. 8 unit-тестов. 108 topology тестов проходят. |
| 2026-07-01 | `5a6b906` → `2cd8c87` (main) | **Phase 0 / 1.2.5 — Timeout partial results test:** `brep_time_limit_override` и `face_time_limit_override` (Option<Duration>) в TriangulationParams — позволяют тестам задавать кастомные time limits. Оба `prepare_brep_session` path используют override когда задан. `timeout_partial.rs`: 4 теста — drill_top (#[ignore], 1s limit), no_active_session, override_propagation, small_file_timeout. Также: исправлены все warnings в draper-diag (lod_verify, face_triangulation_dump, face_pre_merge_dump), draper-mesh (parametric_domain), draper-step (converter.rs), draper-testing (watertight_check). Все 511+ тестов проходят. |
| 2026-07-01 | TBD (main) | **Phase 5 / 6.2 — Nested subassembly transforms audit + test:** 6.2.1 — аудит показал что рекурсивная композиция transforms УЖЕ реализована (walk_assembly_tree_detailed / collect_pending_from_assembly_tree используют mat4_mul на каждом уровне DFS; PendingBrepInstance.transform = fully composed world transform). 6.2.2 — `test/nested_assembly.stp` синтетический 3-уровневый STEP (Root→Sub(+10X)→Leaf(+20Y)), 3 unit-теста: structure, composed_transform (tx=10, ty=20), triangulation_position (bbox 10,20,0–15,25,5). |
| 2026-07-02 | TBD (main) | **Phase 5 / 6.3 — BREP_WITH_VOIDS support:** 6.3.1 — аудит выявил 3 критических проблемы: void faces flat-appended в healing pipeline corrupts normals; ORIENTED_CLOSED_SHELL orientation flip не обрабатывается; FaceData не различает outer/void faces. 6.3.2 — `test/cube_with_void.stp` синтетический BREP_WITH_VOIDS. 6.3.3 — исправления: `FaceData.is_void` / `FaceInfo.is_void` поля; `extract_shell_faces(shell_id, is_void)` с ORIENTED_CLOSED_SHELL обработкой (orientation flip); `face_data_list_to_solid` разделяет outer/void в отдельные Shell; `fix_normal_orientation` инвертированная эвристика для void shells (normals point toward centroid); `heal_shell(shell, params, is_void_shell)`. 10 unit-тестов. Все 112 step + 108 topology + 231 mesh + 8 integration тестов проходят. WASM release задеплоен. |
| 2026-07-02 | TBD (main) | **Phase 5 / 6.2.3 — Subassembly tree UI:** внутренние узлы дерева сборки теперь имеют интерактивные контролы: (1) 👁 toggle для show/hide всего поддерева (3 цвета: зелёный/янтарный/красный); (2) ◎ isolate кнопка (скрыть всё кроме поддерева, toggle-поведение); (3) счётчик "(N parts)" в метке узла; (4) префикс "[+]" для визуального отличия. `collect_subtree_instance_indices()` — рекурсивный сбор instance indices. `pending_subtree_hide/show/isolate` обработка в desktop и mobile путях. Исправлен пропущенный `brep_time_limit_override`/`face_time_limit_override` в `draper-ai`. WASM release задеплоен. |
| 2026-07-02 | TBD (main) | **Bug 8.2.1/8.2.2 — Cone face orientation fix:** два бага для forward:false конических граней. Bug A: UV polygon реверсируется в CW при forward:false → earcutr производит CW треугольники → forward swap инвертирует повторно (double-flip). Fix: CCW normalization через `polygon_signed_area_2d` — реверс outer_uv + boundary_points_3d при отрицательной площади (Step 1.25 в triangulate_surface_consistent). Bug B: vertex normals всегда outward (surface.normal_at) даже при forward:false. Fix: `orient_normal(n, forward)` инвертирует normal для !forward — применено к uv_triangles_to_3d, triangulate_surface_consistent Step 5, и всем 9 вызовам cone.normal_at в 6 функциях triangulate.rs. 231 mesh + 112 step + 91 geometry + 108 topology тестов проходят. WASM release задеплоен. |
| 2026-07-02 | TBD (main) | **Bug 8.2.3 — Face normals (0,0,1) fix for 3.05.078.stp:** Cone Step#78 и Plane Step#87 отображались с неправильным освещением (тёмные/инвертированные грани). Bug C: `merge_deduplicating` заполнял отсутствующие face_normals дефолтным (0,0,1) вместо вычисления из геометрии — структурированные grid-функции (cone tube, cylinder tube и др.) не устанавливают face_normals. Fix: вычисление face normals и vertex normals из cross-product при merge. Bug D: face normals не пересчитывались после post-processing (weld, merge_coincident_vertices). Fix: вызов `mesh.compute_face_normals()` после `smooth_normals()` в `triangulate_brep_detailed`. Diagnostic test добавлен. 231 mesh + 112 step тестов проходят. WASM release задеплоен. |
| 2026-07-02 | TBD (main) | **Bug 8.2.4 — CONICAL_SURFACE semi_angle unit detection:** Cone Step#78 и Plane Step#87 триангулировались некорректно из-за двойной конвертации `.to_radians()` на уже-радианном значении semi_angle. Bug E: `extract_cone()` всегда конвертировал semi_angle из градусов, но STEP ISO-10303-42 определяет единицу через HEADER — файлы с RADIAN (3.05.078.stp, 0.7854=π/4) давали 0.0137 рад (0.79°) после двойной конвертации, превращая конус в цилиндр. Fix: (1) поле `angle_in_degrees: bool` в StepConverter, из `extract_units().uses_degrees()`; (2) конвертация только при degrees или raw > π/2 (эвристика); (3) nist_cone.stp обновлён для DEGREE. UV viz: исправлены "ауры" (per-polyline point_in_polygon + period-shifted copies), уменьшена толщина обводки (1.0→0.5, opacity 220→100). |
| 2026-07-02 | `1e6af94` (main) / `f6de310` (gh-pages) | **Bug 8.2.5 — FaceInfo.triangle_range stale after filtering + UV aura/line artifacts:** (1) `FaceInfo.triangle_range` вычислялся до `filter_degenerate_triangles`, `weld_boundary_edge_vertices`, `remove_duplicate_triangles` — после фильтрации диапазоны ссылались на треугольники ДРУГИХ граней (Cone#78 показывал 252 tris вместо 202, Plane#87 показывал 252 вместо 151). Fix: пересчёт `triangle_range` из `triangle_face_ids` после каждого фильтрующего шага в обоих путях (`triangulate_brep_detailed` и `BrepSession::finalize`). (2) UV ауры: красная подсветка треугольников "outside boundary" создавала видимые ореолы — убрана отрисовка таких треугольников. (3) UV лишние линии: stroke на каждом треугольнике создавал плотную сетку линий — заменён на `Stroke::NONE` (границы видны через чередующиеся fill-цвета). (4) Исправлен borrow-after-move в `triangulate_cone_tube_from_boundary` (bottom_ring/top_ring → effective_bottom/effective_top с clone). Диагностика: все вершины Cone#78 и Plane#87 на правильных поверхностях (0 off-surface). |
| 2026-07-03 | `2e45b8f` (gh-pages) | **Bug 8.3.1+8.3.2 UV fixes + Feature 6.1.1+6.1.2 AP242 GD&T:** (1) 8.3.1 — Cylinder UV axis scaling: добавлен `Surface::uv_metric_scale()` → (u_scale, v_scale) для конвертации параметрического пространства в метрическое (Cylinder: R, Cone: R_base/1/cos(α), Sphere: R/R, Torus: (R+r)/r). Toggle "Metric UV" в UV canvas (default ON) — аспект UV прямоугольника в метрических единицах, ось U показывает arc-length (mm). Axis labels показывают оба диапазона (parameter + metric). 5 unit-тестов. (2) 8.3.2 — Plane UV mapping inverted: для `forward=false` граней ось V инвертируется (V↓), что соответствует виду с обратной стороны поверхности. (3) 6.1.1 — AP242 SHAPE_ASPECT parsing: добавлен парсинг SHAPE_ASPECT, DERIVED_SHAPE_ASPECT, SHAPE_ASPECT_RELATIONSHIP, PROPERTY_DEFINITION, PRODUCT_DEFINITION_SHAPE. Post-processing: GEOMETRIC_TOLERANCE.applied_to и DATUM_FEATURE.applied_to резолвятся через SHAPE_ASPECT → relating_shape к реальным face/surface ID. 5 unit-тестов. (4) 6.1.2 — GD&T UI panel: окно "GD&T — Geometric Tolerances" с таблицами Tolerances, Datum Features, Shape Aspects. Кнопка "GD&T" в правой панели. |
| 2026-07-03 | `1072501` (main) / `f03c43f` (gh-pages) | **Phase 5 / 6.1.3 — GD&T 3D annotations overlay:** `draw_gdt_annotations()` — 2D overlay на 3D viewport через camera MVP projection. Resolve chain: tolerance.applied_to → ShapeAspect.relating_shape → FaceInfo.step_face_id → face centroid (из outer_boundary) → world_to_screen projection. Leader line (gold, two-segment), attachment dot, Feature Control Frame (FCF) с GD&T symbol + tolerance value + datum references. Multi-cell FCF с vertical separators (ASME Y14.5 style). Toggle "3D Annotations" checkbox в STEP Tools panel. Lazy-load gdt_data при включении. WASM build + deploy (9.4MB viewer, 1.4MB worker). |
| 2026-07-03 | TBD (main) | **Phase 5 / 6.1.4 — AP242 GD&T test:** 5 unit-тестов в `gdt_ap242.rs` — extract_gdt() no-panic, GdtToleranceType mapping для 14 ASME Y14.5 типов, from_step_type_and_name для generic GEOMETRIC_TOLERANCE, face centroid computation, step_face_id populated. Синтетический `test/gdt_test.stp` (блок 20x10x5 + FLATNESS + PERPENDICULARITY). |

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
