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
- [ ] Изменение параметра в Sketch перестраивает зависимый Extrude
- [ ] Rollback до шага N восстанавливает геометрию шага N
- [ ] 3+ теста: param change → rebuild, rollback, cache invalidation
- [ ] Нет `unwrap()` в rebuild-логике

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
- [ ] `project_edge(edge: &Edge, plane: &Plane) -> Result<Vec<SketchEntity>>`
- [ ] Проецирование прямого ребра → `SketchEntity::Line`
- [ ] Проецирование кругового ребра → `SketchEntity::Arc` или `Circle`
- [ ] 3+ теста: line projection, circle projection, NURBS fallback
- [ ] Обработка вырожденных случаев (ребро параллельно нормали плоскости)

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
- [ ] `sweep_wire_along_curve(wire: &Wire, path: &Curve) -> Result<Solid>`
- [ ] Генерация `ruled surfaces` для боковых граней
- [ ] Замыкание начала/конца (cap faces) если wire замкнут
- [ ] 3+ теста: sweep вдоль прямой (extrude), sweep вдоль окружности (torus-like), sweep вдоль спирали
- [ ] Обработка самопересечений пути → `KernelError::SelfIntersectingPath`

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
- [ ] `loft_wires(wires: &[Wire]) -> Result<Solid>`
- [ ] Совместимость сечений (одинаковое число контрольных точек)
- [ ] 3+ теста: loft квадрат→круг, loft 3 сечения, loft с guide curve
- [ ] Обработка несовместимых сечений → `KernelError::IncompatibleSections`

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
- [ ] `revolve_wire(wire: &Wire, axis: &Line, angle: f64) -> Result<Solid>`
- [ ] Полный оборот (360°) создаёт замкнутое тело
- [ ] Частичный оборот (< 360°) создаёт open shell с cap faces
- [ ] 3+ теста: revolve 360° (sphere/torus), revolve 90°, revolve с self-intersection detection

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
- [ ] `apply_state()` применяет rotation к `Component.transform`
- [ ] Mate constraint между двумя наклонными плоскостями сходится (угол < 1e-6)
- [ ] Concentric constraint выравнивает оси с разным начальным углом
- [ ] 3+ теста: rotation convergence, gimbal lock avoidance, over-constrained with rotations
- [ ] Нет `unwrap()` при конверсии quaternion → matrix

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
- [ ] `remove_hidden_lines(mesh, view_direction) -> (visible_edges, hidden_edges)`
- [ ] Hidden edges рендерятся как dashed в SVG
- [ ] Для cube: видно 3 грани, 3 скрыты (dashed)
- [ ] 3+ теста: cube HLR, cylinder HLR, concave shape HLR
- [ ] Производительность: < 100ms для mesh с 10K triangles

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
- [ ] `Dimension::from_feature(feature_id, param_name)` создаёт ассоциативный размер
- [ ] Изменение параметра в Timeline обновляет размер на чертеже
- [ ] 3+ теста: dimension update on param change, dimension on rollback, dimension on deleted feature

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
- [ ] `export_pdf(drawing: &Drawing, path: &Path) -> Result<()>`
- [ ] PDF содержит title block, views, dimensions
- [ ] 3+ теста: PDF header validity, content presence, multi-page support

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
- [ ] `LlmClient::generate(prompt) -> Result<String>` через HTTP
- [ ] Backend selection в UI (Ollama / OpenAI / Mock)
- [ ] Graceful fallback на mock при недоступности сервера
- [ ] 3+ теста: mock backend, HTTP timeout handling, API key validation

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
- [ ] `triangulate_on_gpu(mesh: &TriangleMesh) -> Result<TriangleMesh>`
- [ ] WGSL compute shader с workgroup size 256
- [ ] CPU fallback при отсутствии WebGPU
- [ ] 3+ теста: GPU vs CPU result comparison, empty mesh, degenerate triangles

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
- [ ] Drag детали учитывает наложенные constraints
- [ ] Collision detection подсвечивает пересекающиеся детали красным
- [ ] 3+ теста: drag with mate constraint, collision detection, snap threshold

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

1. [ ] **Код написан** и компилируется без ошибок
2. [ ] **Unit-тесты** добавлены (минимум 3, включая edge-cases)
3. [ ] **No Panics:** `cargo clippy -- -D warnings` чистый
4. [ ] **UI Integrated:** Кнопка вызывает реальный код, ошибки в Toast
5. [ ] **Roadmap Updated:** `MASTER_PLAN_100.md` обновлён с commit hash
6. [ ] **Build Verified:** `cargo build --release` и `trunk build` (WASM) проходят

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
- [ ] C1: LLM Backend Integration (P1) — MockLlmClient + trait + HttpLlmConfig exist, HTTP client not wired
- [x] C2: WebGPU Triangulate Shader (P2) — already implemented (MarchingCubes + EarClipping WGSL, 787 lines)
- [ ] C3: Kinematic Drag & Collision (P2) — not started
