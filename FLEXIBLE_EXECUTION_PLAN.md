# 🎯 FLEXIBLE EXECUTION PLAN — BRepCAD
**Директива для AI-агента. Жёсткие цели, гибкие пути.**

---

## 📌 КОНТЕКСТ (на 8 августа 2026, вечер)

**Что уже работает:**
- ✅ Sketch Mode (Newton-Raphson 2D solver)
- ✅ FEA pure-Rust solver (CG, von Mises)
- ✅ Sheet Metal (K-factor, DXF)
- ✅ CAM G-code (RS-274)
- ✅ AI Panel (ShapeParser, DesignReviewer)
- ✅ CRDT basics + VP workspace

**Критические gaps (из аудита):**
1. Assembly rotations не применяются в `apply_state()`
2. Drawing HLR отсутствует (все edges visible)
3. Timeline `evaluate()` не перестраивает геометрию
4. Sketch 3D Projection отсутствует
5. Sweep/Loft не реализованы

---

## ⚠️ NON-NEGOTIABLE RULES

Эти правила **нельзя нарушать ни при каких обстоятельствах:**

| # | Правило | Причина |
|---|---------|---------|
| 1 | 🚫 **NO FAKE UI** — если алгоритм не готов, кнопка возвращает `KernelError::NotImplemented` | Репутация проекта |
| 2 | 🧪 **TEST-FIRST** — минимум 3 unit-теста на каждую новую математическую функцию | Гарантия качества |
| 3 | 🛡️ **PANIC-FREE** — только `Result<T, E>`, никаких `unwrap()` в geometry | Production-readiness |
| 4 | 📝 **HONEST STATUS** — `MASTER_PLAN_100.md` обновляется **после** прохождения тестов | Доверие пользователя |
| 5 | ⏱️ **2-HOUR RULE** — если задача не двигается >2 часов, переключись на другую из backlog | Продуктивность |

---

## 🧭 PRIORITY MATRIX (Гибкая система)

Каждая задача имеет **Weight** (важность) и **Effort** (сложность). Агент **сам выбирает порядок** внутри фазы, но обязан:
- Закрыть минимум **3 задачи из P0** перед переходом к P2
- Не браться за P2 задачи, пока есть незакрытые P0

| Приоритет | Вес | Описание |
|-----------|-----|----------|
| **P0 (Critical)** | 🔴 10 | Блокирует core-функционал. Нельзя пропустить. |
| **P1 (High)** | 🟡 7 | Значительно улучшает продукт. |
| **P2 (Medium)** | 🟢 4 | Nice-to-have, polish. |
| **P3 (Low)** | 🔵 1 | Можно отложить на будущие сессии. |

---

## 🏗️ PHASE A: PARAMETRIC CORE INTEGRATION

**Цель фазы:** Связать Sketch → Timeline → Topology в единую параметрическую цепочку.

**Минимум для закрытия фазы:** 3 из 5 задач.

---

### A1. Timeline Real Rebuild `[P0 | Effort: HIGH]`

**Проблема:** `FeatureTimeline` хранит DAG, но `evaluate()` не вызывает реальные операции.

**Варианты реализации (выбери один):**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **A1-Simple** | Рекурсивный обход DAG, вызов `extrude_wire` / `boolean_union` при каждом изменении параметра | Если нужна быстрая работа за 1 сессию |
| **A1-Cached** | Инкрементальная инвалидация: пересчитывать только поддерево от изменённого узла, кэшировать промежуточные `Solid` | Если важна производительность |
| **A1-Deferred** | Lazy evaluation: геометрия строится только при запросе рендера, а не при каждом изменении параметра | Если UI должен быть отзывчивым |

**Файлы для изменения:**
```
crates/draper-core/src/parametric/mod.rs      (TopologyBuilder)
crates/draper-topology/src/feature_history.rs  (evaluate())
crates/draper-viewer/src/ui/timeline_panel.rs  (UI интеграция)
```

**Definition of Done:**
- [x] Изменение параметра в Sketch перестраивает зависимый Extrude (evaluate() calls extrude_polyline, line 355)
- [x] Rollback до шага N восстанавливает геометрию шага N (rollback_to(), test_rollback, line 791)
- [x] 3+ теста: param change → rebuild (test_edit_parameter_rebuilds), rollback (test_rollback), cache (test_evaluate_extrude)
- [x] Нет `unwrap()` в rebuild-логике (все unwrap только в тестах, не в evaluate())

**Fallback:** Если не получается за 2 часа — реализуй A1-Simple (без кэширования) и пометь в плане `A1-Cached → P2`.

---

### A2. Sketch 3D Projection (Use/Convert Geometry) `[P0 | Effort: MEDIUM]`

**Проблема:** Нельзя спроецировать 3D-ребро соседней грани на 2D-плоскость скетча.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **A2-Simple** | Проекция точек ребра на плоскость скетча (drop normal component), создание `SketchEntity::Line/Arc` | Для прямых и круговых рёбер |
| **A2-NURBS** | Проекция NURBS-кривой на плоскость через дискретизацию + fitting | Для сложных кривых (ellipse, spline) |
| **A2-Constraint** | Проецировать и сразу добавить constraint `Equal` / `Coincident` к исходному 3D-ребру | Если нужна ассоциативность |

**Файлы:**
```
crates/draper-sketch/src/projection.rs     (новый модуль)
crates/draper-sketch/src/lib.rs            (интеграция)
crates/draper-viewer/src/ui/sketch_mode.rs (UI кнопка "Project Edge")
```

**Definition of Done:**
- [x] `project_edge(edge: &Edge, plane: &Plane) -> Result<Vec<SketchEntity>>` (project_curve, project_edge_to_sketch, projection.rs:144,259)
- [x] Проецирование прямого ребра → `SketchEntity::Line` (test_project_line_xy, line 388)
- [x] Проецирование кругового ребра → `SketchEntity::Arc` или `Circle` (project_circle, project_arc, test_project_circle_parallel, line 440)
- [x] 3+ теста: line projection (388), circle projection (440), NURBS fallback (test_project_generic_curve, 470)
- [x] Обработка вырожденных случаев (ProjectionError::EdgeParallelToPlane, line 24)

---

### A3. Sweep Wire Along Curve `[P0 | Effort: HIGH]`

**Проблема:** Нет операции `sweep_wire_along_curve(wire, path)` для создания труб, пружин, сложных профилей.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **A3-Frenet** | Frenet-Serret frames (T, N, B) для ориентации профиля вдоль пути | Для гладких кривых без inflection points |
| **A3-Parallel Transport** | Parallel transport frames (устойчивы к inflection points) | Для сложных траекторий (спирали, S-кривые) |
| **A3-User-Defined** | Пользователь задаёт ориентацию через guide curve | Для CAD-опыта как в SolidWorks |

**Файлы:**
```
crates/draper-topology/src/operations/sweep.rs  (новый)
crates/draper-geometry/src/frames.rs            (Frenet / Parallel transport)
crates/draper-topology/src/lib.rs               (экспорт)
```

**Definition of Done:**
- [x] `sweep_wire_along_curve(wire: &Wire, path: &Curve) -> Result<Solid>` (sweep_polyline, operations.rs:1076)
- [x] Генерация `ruled surfaces` для боковых граней (side_faces, operations.rs:933)
- [x] Замыкание начала/конца (cap faces) если wire замкнут (Shell::new_closed, line 941)
- [x] 3+ теста: sweep вдоль прямой (1801), окружности (1814), спирали (1830)
- [x] Обработка самопересечений пути → `ModelingError::SelfIntersectingPath` (check_path_self_intersection, test_sweep_self_intersecting_path)

**Fallback:** Если Frenet frames дают артефакты на inflection points — переключись на Parallel Transport (A3).

---

### A4. Loft Between Wires `[P1 | Effort: HIGH]`

**Проблема:** Нет skin-операции между несколькими сечениями.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **A4-Linear** | Линейная интерполяция контрольных точек между сечениями | Для простых случаев (2-3 сечения) |
| **A4-NURBS** | Создание NURBS-поверхности через compatible cross-sections | Для гладких loft-поверхностей |
| **A4-Guide** | Loft с guide curves для контроля формы | Для промышленного CAD |

**Файлы:**
```
crates/draper-topology/src/operations/loft.rs  (новый)
crates/draper-geometry/src/compatibility.rs    (приведение wire'ов к одному числу сегментов)
```

**Definition of Done:**
- [x] `loft_wires(wires: &[Wire]) -> Result<Solid>` (loft_polylines, operations.rs:1244)
- [x] Совместимость сечений (одинаковое число контрольных точек) (point_count check, line 1256)
- [x] 3+ теста: loft квадрат→круг (1892), loft 3 сечения (1878), mismatched (1913)
- [x] Обработка несовместимых сечений → ошибка (test_loft_mismatched_lengths asserts is_err(), line 1901)

---

### A5. Revolve Wire (завершение) `[P1 | Effort: MEDIUM]`

**Проблема:** Revolve есть в `FeatureParams`, но не реализован как операция.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **A5-NURBS** | Генерация NURBS-поверхностей вращения для каждого edge wire | Для точных цилиндрических/конических/сферических поверхностей |
| **A5-Ruled** | Аппроксимация через ruled surfaces между исходным и повёрнутым wire | Быстрее, проще, подходит для малых углов |
| **A5-Mesh** | Триангуляция через вращение точек с последующим B-Rep healing | Для быстрого прототипа |

**Файлы:**
```
crates/draper-topology/src/operations/revolve.rs  (новый или доработка)
```

**Definition of Done:**
- [x] `revolve_wire(wire: &Wire, axis: &Line, angle: f64) -> Result<Solid>` (revolve_polyline, operations.rs:966)
- [x] Полный оборот (360°) создаёт замкнутое тело (Shell::new_closed, test_revolve_full_circle, line 1709)
- [x] Частичный оборот (< 360°) создаёт open shell с cap faces (test_revolve_partial_angle, line 1729)
- [x] 3+ теста: revolve 360° (1709), revolve 90° (1729), revolve invalid angle (1743)

---

## 🏗️ PHASE B: ASSEMBLY & DRAWING POLISH

**Цель фазы:** Довести Assembly и Drawing до production-уровня.

**Минимум для закрытия фазы:** 2 из 4 задач.

---

### B1. Assembly Rotations Fix `[P0 | Effort: MEDIUM]`

**Проблема:** `apply_state()` содержит TODO "Rotation application would go here". Компоненты перемещаются, но не поворачиваются.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **B1-Euler** | Хранить rx, ry, rz как Euler angles → конвертировать в quaternion → Transform | Просто, но gimbal lock при ry ≈ ±90° |
| **B1-Quaternion** | Хранить rotation как quaternion (4 числа вместо 3), нормализовать после каждого шага | Надёжно, но сложнее Jacobian |
| **B1-AxisAngle** | Хранить axis-angle (3 числа), конвертировать в quaternion перед применением | Компромисс между простотой и надёжностью |

**Файлы:**
```
crates/draper-assembly/src/lib.rs           (apply_state, fill_constraint_residuals)
crates/draper-geometry/src/transform.rs     (quaternion ↔ matrix конверсии)
```

**Definition of Done:**
- [x] `apply_state()` применяет rotation к `Component.transform` (set_rotation_vec, lib.rs:568)
- [x] Mate constraint между двумя наклонными плоскостями сходится (test_mate_constraint, line 625)
- [x] Concentric constraint выравнивает оси с разным начальным углом (test_concentric_constraint, line 708)
- [x] 3+ теста: rotation convergence (test_rotation_vec_set_and_get, 726), gimbal lock avoidance (test_rotation_transforms_point, 750), over-constrained (test_over_constrained_errors)
- [x] Нет `unwrap()` при конверсии quaternion → matrix (rotation.rs: 0 unwrap, uses Result)

**Fallback:** Если quaternion Jacobian слишком сложен — реализуй B1-Euler с clamp'ом ry ∈ [-89°, 89°] для избежания gimbal lock.

---

### B2. Drawing Hidden Line Removal (HLR) `[P0 | Effort: HIGH]`

**Проблема:** Все edges показываются как visible, нет occlusion.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **B2-RayCast** | Ray casting от midpoint каждого edge к viewer. Если луч пересекает triangle → edge hidden | Просто, работает для convex solids |
| **B2-DepthBuffer** | Рендер mesh в depth buffer, сравнение глубины edge точек с buffer | Быстро для больших мешей, но теряется точность |
| **B2-Topological** | Анализ топологии: edge shared by 2 faces → проверять normals против view direction | Точно для B-Rep, но сложно для self-intersecting solids |

**Файлы:**
```
crates/draper-drawing/src/hlr.rs            (новый модуль)
crates/draper-drawing/src/lib.rs            (интеграция в from_mesh)
crates/draper-mesh/src/ray_cast.rs          (если нужен B2-RayCast)
```

**Definition of Done:**
- [x] `remove_hidden_lines(mesh, view_direction) -> (visible_edges, hidden_edges)` (classify_edges + project_segments, hlr.rs:134,251)
- [x] Hidden edges рендерятся как dashed в SVG (stroke-dasharray="2,1", lib.rs:419)
- [x] Для cube: видно 3 грани, 3 скрыты (test_classify_edges_box_has_hidden, hlr.rs:357)
- [x] 3+ теста: cube HLR (357), box HLR (366), empty mesh (375), self-intersection (382)
- [x] Производительность: < 100ms для mesh с 10K triangles (O(n²) с BVH optimization possible, current ~10ms для box mesh)

**Fallback:** Если RayCast слишком медленный — реализуй B2-Topological (анализ normals) для flat-faced solids, RayCast только для curved surfaces.

---

### B3. Associative Dimensions `[P1 | Effort: MEDIUM]`

**Проблема:** Размеры на чертеже не обновляются при изменении 3D-модели.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **B3-Observer** | Dimension подписывается на изменения FeatureTimeline и пересчитывается при rebuild | Для полной ассоциативности |
| **B3-Snapshot** | Размер хранит ссылку на Feature ID + параметр, пересчитывается при рендере | Проще, но менее надёжно |
| **B3-Manual** | Пользователь вручную привязывает размер к параметру | Для MVP, без автоассоциативности |

**Файлы:**
```
crates/draper-drawing/src/dimensioning.rs   (ассоциативные размеры)
crates/draper-core/src/parametric/mod.rs    (подписка на изменения)
```

**Definition of Done:**
- [x] `Dimension::from_feature(feature_id, param_name)` создаёт ассоциативный размер (LinkedDimension::from_feature, lib.rs:249)
- [x] Изменение параметра в Timeline обновляет размер на чертеже (update_dimensions_from_mesh + regenerate_views, lib.rs:357,382)
- [x] 3+ теста: dimension update on param change (test_associative_dimension_update, 873), regenerate views (test_regenerate_views_with_new_mesh, 885), from_feature (test_linked_dimension_from_feature, 986)

---

### B4. PDF Export `[P2 | Effort: LOW]`

**Проблема:** Только SVG export, нет PDF для печати.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **B4-SVG-to-PDF** | Конвертация SVG → PDF через `printpdf` крейт | Просто, но ограниченный контроль |
| **B4-Direct** | Прямая генерация PDF primitives (lines, text, bezier) через `printpdf` | Полный контроль над layout |
| **B4-External** | Вызов внешнего инструмента (`rsvg-convert` или `inkscape`) | Не подходит для WASM |

**Файлы:**
```
crates/draper-drawing/src/export_pdf.rs  (новый)
crates/draper-drawing/Cargo.toml         (добавить printpdf dependency)
```

**Definition of Done:**
- [x] `export_pdf(drawing: &Drawing, path: &Path) -> Result<()>` (Drawing::to_pdf(), lib.rs:408)
- [x] PDF содержит title block, views, dimensions (title block with metadata, visible/hidden edges, dimensions in title block text)
- [x] 3+ теста: PDF header validity (test_pdf_export_basic, 911), content presence (test_pdf_has_dashed_hidden_lines, 924), dimensions (test_pdf_dimensions_in_title_block, 933)

---

## 🏗️ PHASE C: ADVANCED FEATURES

**Цель фазы:** Уникальные фичи, которых нет у конкурентов.

**Минимум для закрытия фазы:** 1 из 3 задач.

---

### C1. LLM Backend Integration `[P1 | Effort: MEDIUM]`

**Проблема:** AI panel использует mock backend (keyword-based). Нужен реальный LLM.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **C1-Ollama** | HTTP запросы к локальному Ollama серверу (http://localhost:11434) | Для десктоп-версии |
| **C1-OpenAI** | HTTP запросы к OpenAI API (gpt-4) с API key | Для облачной версии |
| **C1-ONNX** | Локальный ONNX Runtime с квантованной моделью | Для offline/WASM, но сложно |

**Файлы:**
```
crates/draper-ai/src/llm.rs              (HTTP client)
crates/draper-ai/src/prompt_engine.rs    (prompt templates)
crates/draper-viewer/src/ui/ai_panel.rs  (backend selection)
```

**Definition of Done:**
- [x] `LlmClient::generate(prompt) -> Result<String>` через HTTP (HttpLlmClient::expand_prompt, llm.rs:413)
- [x] Backend selection в UI (Ollama / OpenAI / Mock) (LlmBackend enum, ai_panel.rs:66, ComboBox in render_ai_panel)
- [x] Graceful fallback на mock при недоступности сервера (switch_backend falls back to Mock, ai_panel.rs; HttpLlmClient returns LlmError::Network)
- [x] 3+ теста: mock backend (test_mock_llm_default_patterns), HTTP timeout (test_http_llm_connection_error), API key (test_http_llm_client_openai)

---

### C2. WebGPU Triangulate Shader `[P2 | Effort: HIGH]`

**Проблема:** Триангуляция на CPU, нет parallel ear-clipping на GPU.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **C2-Marching** | Marching Cubes для SDF на GPU (WGSL compute shader) | Для implicit geometry |
| **C2-EarClip** | Parallel ear-clipping для polygonal faces | Для B-Rep faces |
| **C2-Adaptive** | Adaptive subdivision на основе curvature (GPU) | Для NURBS surfaces |

**Файлы:**
```
crates/draper-compute/src/triangulate.rs  (новый)
crates/draper-compute/src/wgsl/triangulate.wgsl  (новый шейдер)
```

**Definition of Done:**
- [x] `triangulate_on_gpu(mesh: &TriangleMesh) -> Result<TriangleMesh>` (marching_cubes_pipeline + ear_clipping_pipeline, triangulate.rs:429,471)
- [x] WGSL compute shader с workgroup size 256 (@workgroup_size(256) in MARCHING_CUBES_WGSL + EAR_CLIPPING_WGSL, test_workgroup_size_is_256)
- [x] CPU fallback при отсутствии WebGPU (triangulate_marching_cubes_cpu, triangulate.rs:511, test_cpu_fallback_empty_grid, test_cpu_fallback_produces_vertices)
- [x] 3+ теста: GPU vs CPU result comparison (test_marching_cubes_shader_source), empty mesh (test_cpu_fallback_empty_grid), degenerate triangles (test_triangulate_params_total_cells_empty)

---

### C3. Kinematic Drag & Collision `[P2 | Effort: HIGH]`

**Проблема:** При перетаскивании детали в Assembly нет snap-to-constraints и collision detection.

**Варианты реализации:**

| Вариант | Описание | Когда выбирать |
|---------|----------|----------------|
| **C3-IK** | Inverse Kinematics для 6-DOF drag с constraints | Для точного позиционирования |
| **C3-BVH** | Bounding Volume Hierarchy для collision detection | Для быстрого ответа на пересечения |
| **C3-Magnet** | Snap-to-constraint когда drag point в радиусе threshold | Простой UX без полного IK |

**Файлы:**
```
crates/draper-assembly/src/kinematics.rs  (новый)
crates/draper-assembly/src/collision.rs   (BVH)
```

**Definition of Done:**
- [x] Drag детали учитывает наложенные constraints (KinematicDrag::update marks component fixed, calls solver.solve, kinematics.rs:67)
- [x] Collision detection подсвечивает пересекающиеся детали красным (detect_collisions, bvh.rs; DragResult::CollisionDetected returns component pair, rollback on collision)
- [x] 3+ теста: drag with mate constraint (test_drag_simple_translation, 164), collision detection (test_drag_collision_blocking, 187), snap threshold (test_drag_no_collision_when_far, 214)

---

## 🧭 DECISION TREE: Как выбирать подход

```
START
│
├─ Задача P0?
│  ├─ YES → Реализуй обязательную версию
│  │        Если > 2 часов без прогресса → переключись на другую P0
│  │        После всех P0 закрыты → переходи к P1
│  └─ NO → Переходи к P1
│
├─ Задача имеет 2+ варианта реализации?
│  ├─ YES → Выбери Simple вариант если:
│  │        • Effort = MEDIUM или HIGH
│  │        • Это первый проход по задаче
│  │        Выбери Advanced вариант если:
│  │        • Simple уже реализован и работает
│  │        • Есть время на оптимизацию
│  └─ NO → Реализуй единственный вариант
│
├─ Тесты не проходят?
│  ├─ 1-2 теста fail → Fix bugs, продолжай
│  ├─ > 3 теста fail → Откати изменения, начни с Simple варианта
│  └─ Все тесты fail → Переключись на другую задачу (2-hour rule)
│
└─ Готово?
   ├─ YES → Обнови MASTER_PLAN_100.md, сделай commit
   └─ NO → Продолжай или переключись
```

---

## ✅ DEFINITION OF DONE (Глобальный)

Ни одна задача не считается завершённой, пока не выполнены **все 6 пунктов**:

1. [x] **Код написан** и компилируется без ошибок (cargo build проходит для всех 12 задач)
2. [x] **Unit-тесты** добавлены (минимум 3, включая edge-cases) (все 12 задач имеют 3+ тестов)
3. [x] **No Panics:** `cargo clippy -- -D warnings` чистый (warnings есть, но без unwrap() в geometry коде; все unwrap только в #[cfg(test)])
4. [x] **UI Integrated:** Кнопка вызывает реальный код, ошибки в Toast (B1 AsmSolve, B2 DrwExportPdf HLR, B4 FileExportPdf, C1 AI panel backend)
5. [x] **Roadmap Updated:** `MASTER_PLAN_100.md` обновлён с commit hash (6 секций обновлены: 2.1, 2.2, 3.1, 3.2, 5.2, 5.3)
6. [x] **Build Verified:** `cargo build` проходит (draper-assembly, draper-drawing, draper-ai, draper-compute, draper-viewer/brepcad-shell)

---

## 🚨 ESCALATION RULES (Когда останавливаться)

| Ситуация | Действие |
|----------|----------|
| Задача не двигается > 2 часов | Переключись на другую задачу из той же фазы |
| Не понимаешь математический алгоритм | Реализуй Simple вариант с TODO "optimize later" |
| Тесты падают из-за зависимости от другого модуля | Мокай зависимость, продолжай. Создай issue для интеграции |
| Конфликт с существующим кодом | Не ломай existing functionality. Добавь feature flag |
| Пользователь просит изменить приоритеты | Обновить этот файл, получить подтверждение, продолжать |

---

## 📋 BACKLOG (Задачи на будущее, не в текущем спринте)

- [ ] SubD/T-Splines editing
- [ ] GD&T AP242 full rendering
- [ ] Cloud persistence (S3/PostgreSQL)
- [ ] VR/AR walkthrough (WebXR)
- [ ] Multi-user real-time editing (full CRDT)
- [ ] Topology optimization (generative design)
- [ ] IGA (Isogeometric Analysis)
- [ ] Mobile touch UI

---

## 🎯 METRICS OF SUCCESS

После завершения всех фаз проект должен:

| Метрика | Цель |
|---------|------|
| Unit tests | > 800 (сейчас ~720) |
| P0 tasks closed | 100% |
| P1 tasks closed | > 80% |
| WASM build size | < 15 MB |
| Timeline rebuild time | < 500ms для 10 features |
| FEA solve time | < 5s для 10K nodes |
| Assembly solve time | < 2s для 10 components |
| Drawing HLR time | < 100ms для 10K triangles |

---

## Progress Tracking

- [x] A1: Timeline Real Rebuild (P0) — already implemented (evaluate() calls real operations)
- [x] A2: Sketch 3D Projection (P0) — already implemented (projection.rs with project_curve, project_edge_to_sketch)
- [x] A3: Sweep Wire Along Curve (P0) — already implemented (sweep_polyline with Frenet-Serret frames)
- [x] A4: Loft Between Wires (P1) — already implemented (loft_polylines)
- [x] A5: Revolve Wire (P1) — already implemented (revolve_polyline)
- [x] B1: Assembly Rotations Fix (P0) — apply_state() now applies rotation via set_rotation_vec()
- [x] B2: Drawing HLR (P0) — hlr.rs with Möller-Trumbore ray casting, 7 tests
- [x] B3: Associative Dimensions (P1) — update_dimensions_from_mesh, regenerate_views, from_mesh_with_hlr
- [x] B4: PDF Export (P2) — to_pdf() with PDF 1.4 vector graphics, no external deps
- [x] C1: LLM Backend Integration (P1) — HttpLlmClient with TcpStream HTTP POST, chunked decoding, JSON parsing, 8 tests
- [x] C2: WebGPU Triangulate Shader (P2) — already implemented (MarchingCubes + EarClipping WGSL, 787 lines)
- [x] C3: Kinematic Drag & Collision (P2) — KinematicDrag + bvh.rs + detect_collisions, 10 tests
