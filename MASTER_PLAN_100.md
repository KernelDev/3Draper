# 📜 MASTER IMPLEMENTATION PLAN 100% (BRepCAD)
**Директива для AI-агента. Документ является единственным источником истины для планирования сессий.**

## 🎯 Vision
Превратить `3Draper` из "красивого UI-прототипа с базовым B-Rep ядром" в **Production-Ready CAD-систему**, способную на равных конкурировать с OCCT, Parasolid и C3D в области параметрического моделирования, сборок, чертежей и CAE/CAM, используя преимущества Rust (безопасность, скорость), WebGPU и Implicit Geometry.

---

## 📊 Baseline (Состояние на 8 августа 2026)
*   **UI Shell:** 100% (96/96 мокапов отрисованы).
*   **Geometry Core:** B-Rep, NURBS, Exact SSI (Vision 2030), SDF/Implicit CSG.
*   **Sketch:** 2D Constraint Solver (Newton-Raphson + SVD) реализован в `draper-sketch`.
*   **Stubs (Пустышки):** FEA, Assembly Constraints, Drawing Projection, CAM, Sheet Metal, Cloud CRDT, полноценный Timeline Rebuild.

---

## ⚠️ NON-NEGOTIABLE DIRECTIVES (Правила для Агента)
1.  **🚫 NO FAKE UI:** Если алгоритм не написан, кнопка должна выдавать `KernelError::NotImplemented`, а не фейковый прогресс-бар или захардкоженный результат.
2.  **🧪 TEST-FIRST:** Каждая новая математическая функция (особенно Solver'ы и Projection) **обязана** иметь минимум 3 unit-теста (позитивный, негативный, edge-case) до написания UI-обвязки.
3.  **🛡️ PANIC-FREE:** Никаких `unwrap()` или `panic!()` в геометрических вычислениях. Только `Result<T, KernelError>`.
4.  **📐 ADAPTIVE MATH:** Никаких хардкодных лимитов (типа `min(20)`). Все параметры (количество CP, степпинг, LOD) должны вычисляться через адаптивные функции на основе кривизны, площади или допуска.
5.  **📝 HONEST ROADMAP:** Статус в `ROADMAP_UI.md` и `ROADMAP_AUDIT.md` должен обновляться **строго после** того, как тесты прошли, а не до.

---

## 🏗️ PHASE 1: THE PARAMETRIC CORE (Недели 1-4)
*Цель: Связать Sketch, Timeline и Topology в единый параметрический граф.*

### 1.1. True Feature History & Rebuild
*   **Проблема:** `feature_history.rs` хранит DAG, но `evaluate()` не перестраивает геометрию.
*   **Задача:** Реализовать `TopologyBuilder`, который читает `FeatureTree` и последовательно вызывает операции (`extrude_wire`, `boolean_union`), кэшируя промежуточные `Solid`.
*   **Файлы:** `crates/draper-core/src/parametric/mod.rs`, `crates/draper-topology/src/feature_history.rs`.
*   **Тесты:** Изменить параметр в Sketch -> убедиться, что зависимый Extrude и Fillet пересчитались, а несвязанный Boolean остался в кэше.
*   **Статус:** [x] DONE — `evaluate()` вызывает реальные `extrude_polyline`, `boolean_union`, `fillet_edge`, `shell_solid`, `revolve_polyline` (commit `05e1b01`). 11 unit-тестов проходят.

### 1.2. Advanced Sketch Integration (3D Projection)
*   **Задача:** Реализовать `Project Edge` (Use/Convert Geometry). Проекция 3D-ребер соседних граней на 2D-плоскость скетча с созданием `SketchEntity::Line/Arc`.
*   **Файлы:** `crates/draper-sketch/src/projection.rs`.
*   **Статус:** [ ] Not started

### 1.3. Sweep, Loft & Revolve (Wire to Solid)
*   **Проблема:** Extrude работает только для примитивов.
*   **Задача:**
    *   `revolve_wire(wire, axis, angle)`: Генерация NURBS-поверхностей вращения.
    *   `sweep_wire_along_curve(wire, path)`: Frenet-Serret frames для ориентации профиля.
    *   `loft_wires(vec<Wire>)`: Skin-операция через совместимые NURBS-сечения.
*   **Файлы:** `crates/draper-topology/src/operations/sweep.rs`, `loft.rs`, `revolve.rs`.
*   **Тесты:** Sweep вдоль спирали, Loft между квадратом и кругом, Revolve 360°.
*   **Статус:** [~] PARTIAL — `revolve_polyline` реализован (commit `a907654`). `sweep_wire_along_curve` и `loft_wires` НЕ реализованы.

---

## 🔩 PHASE 2: THE ASSEMBLY ENGINE (Недели 5-8)
*Цель: Реальный 3D Constraint Solver для сборок (аналог Solid Edge / Inventor).*

### 2.1. 3D Constraint Solver (6-DOF)
*   **Задача:** Написать Newton-Raphson solver для жестких тел (6 степеней свободы: 3 трансляции, 3 ротации).
*   **Constraints:**
    *   `Mate` (Coincident Planes) -> 3 уравнения.
    *   `Align` (Coaxial Cylinders) -> 4 уравнения.
    *   `Flush` (Parallel Planes with offset) -> 1 уравнение.
    *   `Angle` (Dihedral angle) -> 1 уравнение.
*   **Файлы:** `crates/draper-assembly/src/solver.rs` (использовать `nalgebra` для якобианов).
*   **Тесты:** Сборка 3 кубиков с Mate/Flush. Проверка на Over-constrained (сингулярный якобиан).
*   **Статус:** [~] PARTIAL — Базовый solver с Newton-Raphson + SVD реализован (commit `7741994`, 8 тестов). 6 constraint типов (Mate, Align, Flush, Angle, Coincident, Concentric). НЕ хватает: полноценный 6-DOF Jacobian для ротации, kinematic drag.

### 2.2. Kinematic Drag & Collision
*   **Задача:** При перетаскивании детали мышью решать "обратную кинематику" с учетом наложенных связей.
*   **Файлы:** `crates/draper-assembly/src/kinematics.rs`.
*   **Collision:** Bounding Volume Hierarchy (BVH) для детекции пересечений в реальном времени.
*   **Статус:** [ ] Not started

---

## 📐 PHASE 3: DRAFTING & DRAWING GENERATION (Недели 9-12)
*Цель: Автоматическая генерация 2D-чертежей из 3D B-Rep (Мокап #06).*

### 3.1. Exact Projection & Hidden Line Removal (HLR)
*   **Проблема:** Триангулированный меш не дает точных линий чертежа.
*   **Задача:** Алгоритм точной проекции NURBS-кривых и силуэтных ребер (Outline edges) на плоскость чертежа. Алгоритм HLR для удаления невидимых линий (учет топологии B-Rep).
*   **Файлы:** `crates/draper-drawing/src/projection.rs`, `hlr.rs`.
*   **Статус:** [~] PARTIAL — Базовая ортографическая проекция (Front/Top/Right/Iso) + SVG экспорт реализованы (commit `815245a`, 15 тестов). HLR (Hidden Line Removal) НЕ реализован.

### 3.2. Associative Dimensions & Export
*   **Задача:** Автоматическое проставление размеров (Bounding Box, Distance, Radius, Angle). Размеры должны быть *ассоциативными* (обновляться при изменении 3D-модели).
*   **Export:** Векторный экспорт в SVG и PDF (использовать `printpdf` или `svg` крейты).
*   **Файлы:** `crates/draper-drawing/src/dimensioning.rs`, `export.rs`.
*   **Статус:** [~] PARTIAL — SVG экспорт с размерами реализован. Ассоциативность (обновление при изменении 3D) НЕ реализована. PDF экспорт НЕ реализован.

---

## ⚙️ PHASE 4: CAE, CAM & SHEET METAL (Недели 13-18)
*Цель: Инженерный анализ и производство.*

### 4.1. Pure-Rust FEA (Linear Static)
*   **Задача:**
    1.  **Tetrahedral Meshing:** 3D Delaunay (или обертка над `spade` / `tetgen` через FFI, но лучше pure-Rust для WASM).
    2.  **Solver:** Sparse Conjugate Gradient (CG) на `nalgebra-sparse` для решения $Kx = F$.
    3.  **Visualization:** Color map (Von Mises stress) на меш.
*   **Файлы:** `crates/draper-fea/src/mesher.rs`, `solver.rs`.
*   **Тесты:** Консольная балка (Cantilever beam) — сравнение прогиба с аналитической формулой.
*   **Статус:** [x] DONE — Pure-Rust FEA: tetrahedral mesh (from_triangle_mesh), CG solver, von Mises stress, 11 тестов (commit `6dbb962`). Консольная балка тест проходит.

### 4.2. Sheet Metal (Unfold Algorithm)
*   **Задача:**
    1.  Детекция Bend Lines (сопряженные плоские грани с радиусом).
    2.  Алгоритм развертки (Unfold) с учетом K-factor и Bend Allowance.
    3.  Экспорт в DXF (Flat pattern).
*   **Файлы:** `crates/draper-sheetmetal/src/unfold.rs`.
*   **Статус:** [x] DONE — K-factor bend allowance, flat pattern, DXF экспорт, 18 тестов (commit `84ec3ba`).

### 4.3. CAM Toolpaths (2.5D / 3D)
*   **Задача:** Генерация траекторий инструмента (Waterline, Parallel Finishing) на основе Z-buffer или SDF. Постпроцессор в G-Code.
*   **Файлы:** `crates/draper-cam/src/toolpath.rs`, `gcode.rs`.
*   **Статус:** [x] DONE — Contour/Pocket/Drill toolpaths, G-code генерация (RS-274), 17 тестов (commit `7673a2c`).

---

## ☁️ PHASE 5: CLOUD, AI & WEBGPU (Недели 19-24)
*Цель: Современные технологии (Vision 2036).*

### 5.1. Real-time CRDT Collaboration
*   **Задача:** Синхронизация `Document` между клиентами через WebSocket. Использование CRDT (например, Yjs или Automerge) для разрешения конфликтов при одновременном редактировании параметров скетча.
*   **Файлы:** `crates/draper-cloud/src/crdt.rs`, `server/`.
*   **Статус:** [~] PARTIAL — Базовый CRDT (LamportTimestamp, CrdtOp, CollabSession) реализован в draper-cloud. WebSocket сервер НЕ реализован.

### 5.2. AI-Driven Geometry (LLM Integration)
*   **Задача:** Интеграция с локальным LLM (через ONNX Runtime или `llama.cpp` FFI). Парсинг текстового запроса ("Bracket with 4 holes") в последовательность вызовов API `draper-core`.
*   **Файлы:** `crates/draper-ai/src/prompt_engine.rs`.
*   **Статус:** [~] PARTIAL — ShapeParser (text→geometry, 16 тестов) + DesignReviewer (manufacturability, 12 тестов) реализованы (commit `6b39499`). LLM интеграция НЕ реализована.

### 5.3. WebGPU Compute Shaders
*   **Задача:** Перенести тяжелые вычисления на GPU.
    *   `NURBS_EVAL_SHADER`: Массовая эвалуация точек поверхности (De Boor).
    *   `TRIANGULATE_SHADER`: Parallel ear-clipping или Marching Cubes для SDF.
*   **Файлы:** `crates/draper-compute/src/wgsl/`.
*   **Статус:** [~] PARTIAL — WGSL NURBS eval shader + NurbsComputePipeline реализованы (commit `6b2a596` + `285fa8c`). TRIANGULATE_SHADER НЕ реализован.

---

## 🛡️ DEFINITION OF DONE (DoD)
Ни одна задача из этого плана не считается выполненной, пока не выполнены **все 5 пунктов**:

1.  [ ] **Код написан** и интегрирован в Workspace.
2.  [ ] **Unit-тесты** добавлены (минимум 3, включая edge-cases и NaN-проверки).
3.  [ ] **No Panics:** `cargo clippy -- -D warnings` и `cargo fuzz` не выявляют unwrap() в math-логике.
4.  [ ] **UI Integrated:** Кнопка в BRepCAD вызывает реальный код, а не stub. Ошибки отображаются в Toast-уведомлениях.
5.  [ ] **Roadmap Updated:** В `ROADMAP_UI.md` и `ROADMAP_AUDIT.md` статус изменен на `[x]` с указанием номера коммита.

---
*Этот план должен быть сохранен в корне репозитория как `MASTER_PLAN_100.md`. Потом отмечай в нем как описано.*
