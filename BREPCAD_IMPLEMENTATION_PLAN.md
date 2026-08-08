# 🎯 ДИРЕКТИВА: ПОЛНАЯ РАЛИЗАЦИЯ ФУНКЦИОНАЛЬНОСТИ BRepCAD

**Дата:** 7 августа 2026
**Статус:** ОБЯЗАТЕЛЬНАЯ К ИСПОЛНЕНИЮ
**Цель:** Превратить BRepCAD из UI-прототипа в полнофункциональный CAD-редактор

---

## ⚠️ ПРАВКИ АГЕНТА (мотивированные)

> **Agent note (2026-08-07):** Изучив текущую кодовую базу, внёс следующие
> мотивированные правки в исходную директиву. Правки отмечены `[AGENT EDIT]`.

### Правка 1: API signatures — индикативные, не буквальные

Исходный план приводит конкретные сигнатуры (`Wire::is_closed()`,
`Face::from_wire()`, `Surface::ruled()`, `Shell::from_faces()`). Проверка
кода показала, что `Wire`, `Face`, `Shell`, `Solid` существуют в
`draper-topology/src/entity.rs`, но многие методы (`from_wire`,
`from_faces`, `is_closed`) **не реализованы**.

**Решение:** API signatures в плане трактуются как индикативные
(целевые). Реализация будет адаптирована к существующим структурам
данных — при необходимости методы будут добавлены в `entity.rs` или
`builder.rs` по мере необходимости. Это не меняет функциональных
требований, только уточняет путь реализации.

### Правка 2: FEA — pure-Rust вместо C++ зависимостей

Исходный план предлагает `tetgen` или `netgen` для tetrahedral mesh.
Это C++ библиотеки, требующие отдельной сборки и FFI bindings, что
усложняет WASM-совместимость и CI.

**Решение:** Для Фазы 2 FEA использовать pure-Rust подход:
- Tetrahedral mesh generation через упрощённый алгоритм
  (Delaunay на основе существующего `delaunator` crate + 3D extrusion).
- Linear static solver — собственная реализация на `nalgebra`
  (sparse CG solver), без внешних FEA библиотек.
- Это сохраняет WASM-совместимость и упрощает сборку.

### Правка 3: AI integration — reuse draper-ai, не llama.cpp/Ollama

Исходный план предлагает `llama.cpp` или `Ollama` для Phase 3 AI.
Это тяжёлые зависимости (multi-GB models), несовместимые с WASM.

**Решение:** Использовать существующий `draper-ai` crate (ONNX-based)
для всех AI функций. Для "Shape from Text" — lightweight rule-based
parser + ONNX model для intent classification. Это сохраняет
архитектурную согласованность с Vision 2030/2031 работой.

### Правка 4: Timeline — индикативные недели

"Недели 1-12" — это полный roadmap. Каждая сессия агента фокусируется
на **инкрементальных deliverables** (один подмодуль за раз), не пытаясь
реализовать всю Фазу 1 за один проход. Критерии приёмки остаются
теми же — но темп определяется фактическим прогрессом.

### Правка 5: draper-sketch crate — новый, не в existing geometry

План предлагает отдельный `draper-sketch` crate. Это правильно —
2D sketch geometry + constraint solver концептуально отделены от
3D B-Rep. Создаём новый crate.

---

## 📋 ОБЗОР ТЕКУЩЕГО СОСТОЯНИЯ

**Проблема:** 96 UI-мокапов отрисованы, но только ~40% имеют реальную
функциональность. Остальные 60% — это stubs (UI-пустышки), которые
создают иллюзию работы.

**Задача:** Реализовать **реальную функциональность** для всех ключевых
компонентов, превратив BRepCAD в production-ready CAD-приложение.

---

## 🏗️ АРХИТЕКТУРНЫЕ ПРИНЦИПЫ

### Принцип 1: Никаких Stubs

❌ **ПЛОХО:** фейковый прогресс-бар, захардкоженные результаты.
✅ **ХОРОШО:** реальный вызов solver, реальные данные, реальные ошибки.

### Принцип 2: Test-Driven Development

Для каждой фичи — минимум 3 unit-теста + 1 integration-тест.

### Принцип 3: Graceful Degradation

Если фича не может быть выполнена полностью — честно сообщать об этом,
а не показывать фейк.

---

## 🎯 ФАЗА 1: КРИТИЧЕСКИЙ ФУНДАМЕНТ

### 1.1 Sketch Mode с Constraint Solver (#02)

**Текущий статус:** UI есть, но нет 2D geometry kernel и constraint solver.

**Требования:**

#### Создать новый крейт `draper-sketch`:

```rust
pub struct Sketch2d {
    pub entities: Vec<SketchEntity>,
    pub constraints: Vec<Constraint>,
    pub parameters: HashMap<String, f64>,
}

pub enum SketchEntity {
    Point { id: u64, x: f64, y: f64 },
    Line { id: u64, start: u64, end: u64 },
    Circle { id: u64, center: u64, radius_param: String },
    Arc { id: u64, center: u64, start: u64, end: u64 },
}

pub enum Constraint {
    Coincident { p1: u64, p2: u64 },
    Distance { p1: u64, p2: u64, value: String },
    Horizontal { line: u64 },
    Vertical { line: u64 },
    Parallel { l1: u64, l2: u64 },
    Perpendicular { l1: u64, l2: u64 },
    Tangent { e1: u64, e2: u64 },
    Equal { e1: u64, e2: u64 },
    Fixed { entity: u64 },
}
```

#### Реализовать Constraint Solver:

Newton-Raphson с SVD для устойчивости. Якобиан строится из
частных производных constraints по координатам точек.

#### Интеграция в UI:

SidePanel с инструментами (Line, Circle, Rectangle), constraints
(Coincident, Dimension), кнопка Solve. CentralPanel для рисования.

**Критерии приёмки:**
- [ ] Можно создать точку, линию, окружность кликом мыши
- [ ] Можно добавить constraint (coincident, distance, parallel)
- [ ] Кнопка "Solve" реально решает систему (Newton-Raphson)
- [ ] При изменении параметра геометрия перестраивается
- [ ] Under-constrained sketch показывает предупреждение
- [ ] Over-constrained sketch показывает ошибку конфликта

### 1.2 Extrude / Revolve из Sketch (#31, #95)

**Текущий статус:** Нет операции превращения 2D-скетча в 3D-тело.
`extrude_wire` и `revolve_wire` отсутствуют в `draper-topology/src/operations.rs`.

**Требования:**

Реализовать `extrude_wire(wire, direction, distance) -> Result<Solid>`
и `revolve_wire(wire, axis, angle) -> Result<Solid>` в
`draper-topology/src/operations.rs`. Алгоритм:
1. Проверить, что wire замкнут
2. Создать base face из wire (planar)
3. Создать боковые грани (ruled surfaces) для каждого edge
4. Создать top face (translated base)
5. Сшить в Shell → Solid

**Критерии приёмки:**
- [ ] Замкнутый скетч можно extrude в 3D-тело
- [ ] Extrude работает в любом направлении
- [ ] Revolve создает тело вращения
- [ ] Revolve на 360° — полное тело, <360° — с торцами

### 1.3 Parametric Timeline (#65)

**Текущий статус:** Timeline UI есть, но нет реальной параметрической
истории. `feature_history.rs` в draper-topology существует, но не
интегрирован с UI.

**Требования:**

`FeatureTimeline` с rebuild-логикой. Каждая операция (Sketch, Extrude,
Fillet, Boolean) сохраняется. Rollback откатывает до шага. Изменение
параметра перестраивает модель.

**Критерии приёмки:**
- [ ] Каждая операция сохраняется в timeline
- [ ] Клик на "Rollback" откатывает модель
- [ ] Двойной клик открывает диалог редактирования параметров
- [ ] Изменение параметра перестраивает модель
- [ ] Timeline можно экспортировать/импортировать как JSON

### 1.4 Properties Panel (#64)

**Текущий статус:** Панель есть, но не отображает параметры.

**Требования:**

При выборе грани показываются её параметры (radius, degree, etc.).
Параметры можно редактировать через UI. Изменение модифицирует геометрию.

---

## 🎯 ФАЗА 2: ПРОДВИНУТЫЕ ФУНКЦИИ

### 2.1 Assembly Constraints (#14, #34)

Constraint solver для сборок (Mate, Align, Flush, Angle).
Drag-and-drop с snap. Проверка conflicts.

### 2.2 Drawing Generation (#06, #16, #36)

Проекционный алгоритм (orthographic, isometric).
Автоматическое создание видов. Размерные линии.
Title block. Экспорт в PDF/SVG.

### 2.3 Basic FEA Analysis (#04, #17, #37)

**[AGENT EDIT]** Pure-Rust tetrahedral mesh generation (Delaunay-based).
Linear static solver на nalgebra (sparse CG).
Визуализация результатов (color map).
Граничные условия (fixed faces, forces, pressures).

---

## 🎯 ФАЗА 3: ПРОМЫШЛЕННЫЕ ФУНКЦИИ

### 3.1 Sheet Metal (#05, #13, #33)

Bend algorithm (K-factor), flat pattern, bend allowance, DXF export.

### 3.2 CAM Toolpath Generation (#15, #35)

2.5D milling (pocket, contour, drill). Tool library. G-code. Simulation.

### 3.3 Real AI Integration (#25, #39, #68)

**[AGENT EDIT]** Использовать существующий `draper-ai` crate (ONNX-based).
"Shape from Text" — rule-based parser + ONNX intent classifier.
"AI Design Review" — manufacturability analysis.
"AI Auto-Repair" — geometry healing.

---

## 📋 ЧЕК-ЛИСТ КАЧЕСТВА

Для каждой фичи:
- [ ] **Unit-тесты:** Минимум 3 теста (happy path, edge case, error case)
- [ ] **Integration-тест:** End-to-end тест
- [ ] **Documentation:** Doc-comments для всех public функций
- [ ] **Error Handling:** Все `Result` обрабатываются
- [ ] **UI Feedback:** Пользователь видит прогресс, ошибки, успех
- [ ] **Performance:** Операции <2 секунд для типичных моделей

### Definition of Done:

1. ✅ Код написан и проходит все тесты
2. ✅ UI полностью функционален (не stub)
3. ✅ Документация обновлена
4. ✅ ROADMAP_UI.md отражает реальный статус
5. ✅ Пользователь может выполнить реальный workflow без ошибок

---

## 🚨 КРИТИЧЕСКИЕ ПРАВИЛА

### Правило 1: Никаких Фейков

Все результаты должны быть реальными. Никаких захардкоженных значений.

### Правило 2: Честные Stub-сообщения

Если фича не реализована — честно сказать "Not implemented yet",
НЕ показывать фейковый результат.

### Правило 3: Test First

Перед написанием кода — написать тест.

---

## 📊 ПЛАН РАБОТ

### Неделя 1-2: Sketch Mode
- Создать `draper-sketch` crate
- Constraint Solver (Newton-Raphson + SVD)
- UI для рисования и constraints
- 10+ unit-тестов для solver

### Неделя 3-4: Extrude/Revolve + Timeline
- `extrude_wire()` и `revolve_wire()`
- `FeatureTimeline` с rebuild
- UI timeline с rollback
- Properties Panel

### Неделя 5-6: Assembly + Drawing
### Неделя 7-8: FEA + Mesh
### Неделя 9-12: Промышленные фичи

---

## 💬 ФИНАЛЬНАЯ ДИРЕКТИВА

**Приоритет #1:** Sketch Mode с реальным constraint solver.
**Приоритет #2:** Extrude/Revolve + Parametric Timeline.
**Приоритет #3:** Убрать все stubs и фейки.

**Метрика успеха:** Пользователь может создать деталь с нуля:
Sketch → Extrude → Fillet → Boolean → Drawing. Без фейков.

**Не начинай Фазу 2, пока не завершена Фаза 1.**

---

*Этот файл — контракт между агентом и пользователем.*
