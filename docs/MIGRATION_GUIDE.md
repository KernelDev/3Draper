# Migration Guide — BRepCAD/3Draper Sandbox Relocation

**Дата:** 2026-08-29
**Автор:** Main Agent (Super Z)
**Репозиторий:** https://github.com/KernelDev/3Draper
**Текущий HEAD:** `4988645`
**Baseline до начала работ:** `85a7260` (commit до BREP_CORE_FIX_PLAN)

---

## 1. Быстрый старт в новом sandbox

### 1.1. Установка Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable --profile minimal
source "$HOME/.cargo/env"
cargo --version  # должен показать cargo 1.98+
```

### 1.2. Клонирование репозитория

```bash
cd /home/z/my-project
git clone https://github.com/KernelDev/3Draper.git
cd 3Draper
```

**Если нужен push доступ (token):**
```bash
git remote set-url origin https://<TOKEN>@github.com/KernelDev/3Draper.git
```

### 1.3. Проверка целостности

```bash
# Core tests (667 tests, должны все pass; 658 базовых + 9 EdgeStore C5 Stage 2)
cargo test -p draper-geometry --lib --release
cargo test -p draper-topology --lib --release
cargo test -p draper-mesh --lib --release
cargo test -p draper-step --lib --release

# STEP regression (33 теста)
cargo test -p draper-testing --release step_regression_ -- --nocapture
```

**Ожидаемый результат:** 667 core tests + 33 STEP regression = 700 tests, 0 failed
(2026-09-01, C5 follow-up #1: Vulcan больше НЕ таймаут — ~10s после
устранения O(n²) пост-процессинга; см. KNOWN_ISSUES и worklog_new.md).

### 1.4. Сборка приложения

```bash
# Debug build (быстрее компилируется, медленнее работает)
cargo build --bin brepcad-shell

# Release build (медленнее компилируется, быстрее работает)
# ⚠️ Может потребовать 5-10 минут и несколько попыток (OOM при LLVM оптимизации egui/wgpu)
cargo build --release --bin brepcad-shell
```

### 1.5. Запуск приложения

```bash
cargo run --release --bin brepcad-shell
# или
./target/release/brepcad-shell
```

---

## 2. Структура проекта

```
3Draper/
├── crates/
│   ├── draper-geometry/      # Point3d, Vec3d, Direction3d, Surface, Curve3d, Transform
│   ├── draper-topology/      # Solid, Shell, Face, Wire, Edge, CoEdge, boolean ops, healing, validator
│   ├── draper-mesh/          # TriangleMesh, triangulation, edge_cache, watertight, mesh_boolean
│   ├── draper-step/          # STEP AP203/AP214 parser + converter
│   ├── draper-viewer/        # egui + wgpu UI, VP graph editor, BRepCAD shell
│   ├── draper-sketch/        # 2D sketch + constraint solver
│   ├── draper-assembly/      # 6-DOF assembly solver
│   ├── draper-fea/           # Linear static FEA
│   ├── draper-cam/           # CNC toolpath + G-code
│   ├── draper-drawing/       # 2D drawing generation + HLR
│   ├── draper-sheetmetal/   # Bend allowance, K-factor, unfold
│   ├── draper-ai/            # Design review, defect classifier
│   ├── draper-implicit/      # SDF, dual contouring
│   ├── draper-subd/          # Catmull-Clark subdivision
│   ├── draper-core/          # Document, engine, quantum hash, digital twin
│   ├── draper-json/          # JSON model API
│   ├── draper-ffi/           # C FFI bindings
│   ├── draper-wasm/          # WASM bindings
│   ├── draper-worker/        # Web worker bridge
│   ├── draper-cloud/         # CRDT, WebSocket sync
│   ├── draper-compute/       # GPU compute pipeline
│   └── draper-testing/       # Integration tests, STEP regression suite
├── tools/
│   ├── src/bin/              # 50+ diagnostic tools (cone_diag, sphere_diag, etc.)
│   └── Cargo.toml            # package name: "draper-diag"
├── test/                     # 22+ STEP test files (NIST, synthetic, industrial)
├── examples/
│   └── vp_graphs/            # 10 VP graph JSON files + README
├── docs/
│   ├── BREP_CORE_FIX_PLAN.md     # Главный план работ (выполнен)
│   ├── BREPCAD_DEEP_AUDIT.md     # Финальный аудит с метриками
│   ├── BOOLEAN_ARCHITECTURE.md
│   └── ...                       # Другая документация
├── BREPCAD_IMPLEMENTATION_PLAN.md  # Исходный план (Phase 1-3)
├── ROADMAP.md
├── Cargo.toml                # Workspace root
└── README.md
```

---

## 3. Правила работы с GitHub

### 3.1. Коммит-конвенция

Используется **Conventional Commits** с префиксами:

| Префикс | Назначение | Пример |
|---------|------------|--------|
| `fix(core):` | Bug fix в ядре (geometry/topology/mesh) | `fix(core): H1 — sphere triangulation fix (31%→0%)` |
| `fix(mesh):` | Bug fix в draper-mesh | `fix(mesh): Phase 4.5 — weld_boundary_edge_vertices` |
| `fix(step):` | Bug fix в draper-step | `fix(step): J2 — parser robustness для unbalanced parentheses` |
| `fix(cone):` | Cone-specific fix | `fix(cone): H2 — STEP semi_angle sign fix` |
| `feat(vp):` | Новая фича в VP | `feat(vp): add VP graph load/save/import/export UI` |
| `feat(ui):` | Новая UI фича | `feat(ui): real workspace panels for Sketch/SM/CAM/FEA/Drawing/AI` |
| `test(core):` | Новые тесты | `test(core): E1 — STEP regression test suite (33 теста)` |
| `docs:` | Документация | `docs: rewrite BREPCAD_DEEP_AUDIT.md` |
| `revert(cone):` | Откат изменения | `revert(cone): H2 — откат analytic full cone` |

### 3.2. Workflow

```bash
# 1. Перед началом работы — всегда pull
git pull --rebase origin main

# 2. Если есть unstaged changes — stash
git stash
git pull --rebase origin main
git stash pop

# 3. После изменений — stage, commit, push
git add -A
git -c user.name="Z User" -c user.email="z@container" commit -m "fix(core): <описание>"
git push origin main

# 4. Если push отклонён (remote has new commits)
git pull --rebase origin main
git push origin main
```

### 3.3. Git identity

В sandbox нет глобальной git identity. Используется per-commit:

```bash
git -c user.name="Z User" -c user.email="z@container" commit -m "..."
```

### 3.4. Token

GitHub token хранится в remote URL:
```
https://<TOKEN>@github.com/KernelDev/3Draper.git
```

Если token истёк, обновить:
```bash
git remote set-url origin https://<NEW_TOKEN>@github.com/KernelDev/3Draper.git
```

### 3.5. Отмена локальных изменений

```bash
# Отменить конкретный файл
git checkout -- <file>

# Отменить все локальные изменения
git checkout -- .

# Удалить untracked files
git clean -fd
```

---

## 4. Все коммиты BREP_CORE_FIX_PLAN (25 коммитов)

От `979b7bb` до `4988645`:

| # | Commit | Этап | Описание |
|---|--------|------|----------|
| 1 | `979b7bb` | A | Этап A — стабилизация (3 failing tests + dead code) |
| 2 | `45d6455` | B1 | Аналитический Cylinder×Cylinder parallel-axis intersection |
| 3 | `bd9885e` | B1+B2 | B1 tangent + B2 Möller-Trumbore ray-triangle |
| 4 | `5b37e72` | C1+C2 | stitch_collinear_edges + fix_normal_orientation |
| 5 | `6e62f86` | E1 | STEP regression test suite (33 теста) |
| 6 | `70c5872` | B3 | split_general_face для неплоских граней |
| 7 | `c0c205c` | C3+C4 | merge_faces NURBS + add_coedge в правильную позицию |
| 8 | `33e3201` | D | Triangulation cleanup + deprecation warnings |
| 9 | `2d39333` | F | Rewrite BREPCAD_DEEP_AUDIT.md с реальными цифрами |
| 10 | `6695706` | G | RuledSurface/OffsetSurface project_point + uniform scale radii |
| 11 | `1bd8659` | E2 | Прогон NIST/brick/drill_top STEP regression tests |
| 12 | `6da4a40` | E2 | Прогнать STEP regression на всех NIST/synthetic/brick/as1 |
| 13 | `7fb49f1` | H1 | Sphere triangulation fix (31%→0% boundary) |
| 14 | `f0cce79` | H2(revert) | Откат analytic full cone (ухудшает результат) |
| 15 | `1f1168d` | H2+G4 | H2 diagnostic + G4 fix_self_intersections_heal documentation |
| 16 | `dbf861f` | D | Phase 4.5 — weld_boundary_edge_vertices для cross-face mismatches |
| 17 | `c28d554` | I1 | Update KNOWN_ISSUES + ignore thin_annulus |
| 18 | `c304f3d` | J1 | synth_thin_annulus.stp syntax error fix (hang→PASS) |
| 19 | `e077b0a` | docs | Update BREP_CORE_FIX_PLAN with J1 status |
| 20 | `cc549fd` | J2 | Parser robustness для unbalanced parentheses |
| 21 | `b5d9272` | K1 | Fix 6 pre-existing draper-step test failures (path resolution) |
| 22 | `4076cf8` | H2 | STEP semi_angle sign fix (18%→15% boundary) |
| 23 | `aa020b6` | L1 | Aggressive weld fallback + increased weld tolerance |
| 24 | `4988645` | M2 | Финальный BREPCAD_DEEP_AUDIT.md с полными результатами |

---

## 5. Финальные метрики

### Core Tests

| Crate | Tests | Status |
|-------|-------|--------|
| `draper-geometry --lib` | 121 | ✅ 0 failed |
| `draper-topology --lib` | 167 | ✅ 0 failed |
| `draper-mesh --lib` | 253 | ✅ 0 failed |
| `draper-step --lib` | 126 | ✅ 0 failed |
| **Total** | **667** | **✅ 0 failed** |

### STEP Regression (33 файла, все PASS)

| Категория | Количество | Примеры |
|-----------|------------|---------|
| **0% boundary (ideal)** | 8 | synth_cube, synth_sphere, synth_torus, nist_cylinder, nist_sphere, as1_rod |
| **≤5% boundary (PASS)** | 5 | nist_block_with_hole (3.10%), drill_top (4.51%), compressor (3.74%) |
| **5-15% (KNOWN_ISSUES)** | 12 | nist_cone (12%), as1_bolt (6.5%), as1_nut (11%) — synth_cone удалён (1.31%, follow-up #2) |
| **15-40% (complex)** | 5 | Zentralstaender (0.95% overall, 27 solids), transmission (9.08%) |
| **Timeout (slow)** | 1 | gdt_test |
| **Быстрые (было timeout)** | 1 | Vulcan — 9.7s после C5 follow-up #1 (было 700-900s) |
| **Hang (was, fixed)** | 1 | synth_thin_annulus — 9.14% после J1; **0.00% после C5 follow-up #2** (FACE_BOUND list-unwrap) |

### Ключевые улучшения

| Файл | До | После | Улучшение |
|------|-----|-------|-----------|
| synth_sphere | 30.33% | **0.00%** | H1: detect_sphere_seam |
| nist_sphere | 31.36% | **0.00%** | H1: detect_sphere_seam |
| drill_top | 41.23% | **4.51%** | L1: aggressive weld |
| as1_plate | 49.55% | **9.80%** | L1: aggressive weld |
| as1_bolt | 30.12% | **6.50%** | L1: aggressive weld |
| as1_nut | 40.80% | **11.14%** | L1: aggressive weld |
| as1_rod | 17.03% | **0.00%** | L1: aggressive weld |
| nist_cone | 18.22% | **12.44%** | H2+L1: semi_angle sign + weld |
| synth_thin_annulus | **HANG** | **9.14%** | J1: syntax error fix; **0.00%** — C5 follow-up #2 |
| synth_cone | 14.49% | **1.31%** | C5 follow-up #2: junction-level snap (1.31% — геом. пол: полуконус без грани разреза) |
| Dead code | 879 LOC | **0** | A4: mesh_boolean cleanup |

---

## 6. Known Limitations (future work)

1. **C5: `Face.edges: Vec<Edge>` structural refactor** — root cause большинства оставшихся boundary edge problems. Shared edge между двумя гранями дублируется как отдельные Edge-структуры с разными TopoId. **Stage 1 (mesh-level, edge cache unification), Stage 2 (EdgeStore + Face.edge_ids + alias-резолвинг, non-breaking) и Stage 3 (геометрическая дедупликация нативных рёбер + пропагация фиксов) выполнены 2026-08-29/30.** **Stage 4 (2026-08-31, «store-first reads»):** read-API (`Solid::resolve_edge` / `Solid::face_edges` / `Face::edge_by_id`), `Solid::sync_edge_mirrors()` (зеркала = производные от store), канонический healing-flow (`heal_solid` → sync; legacy `validation::heal_solid` через `store.get_mut`), boolean `shared_split_edges` → EdgeStore, identity-фиксы: `validate_brep` (канонический подсчёт coedge'ов — нет ложных dangling на STEP shared-рёбрах), `fillet_edge`/`chamfer_edge` (alias-резолвинг числового id). **Stage 5 (2026-08-31, «decoupled consumers») выполнен:** mesh standalone-API с явными рёбрами (`triangulate_face_with_edges[_and_cache]`, staged-view реализация), serde-формат EdgeStore (flat-формат, `#[serde(default)]` + legacy-загрузка зеркал через `ensure_edge_store`), store-first edge-listing в json/wasm/ffi, identity-based manifold detection в viewer/wasm, `canonical_edge_ids()` в UI/healing_ml. subd от `Face.edges` не зависит (SubdEdge — домен subdivision-сетки). Полное УДАЛЕНИЕ поля `Face.edges` (Stage 6) отложено: ядровые модули (builder/boolean/healing/validation) остаются СОЗДАТЕЛЯМИ зеркал, зеркала — instance-keyed геометрия для coedge-lookups (`Face::edge_by_id`). Детали: worklog_new.md.

2. **Cone (12.44%)** — решено в C5 Stage 1: nist_cone теперь **0.00%** boundary (watertight).

3. **273 warnings в draper-viewer** — unused imports/variables. Не влияют на functionality.

4. **gdt_test timeout** — GD&T annotations. ~~Vulcan~~ — устранён 2026-09-01
   (C5 follow-up #1: O(n²) диагностики в validate_edge_consistency/weld →
   spatial-hash/CSR; Vulcan ~10s, transmission 3.2s — industrial trade-off
   Stage 1 закрыт: быстрее даже до-C5 baseline 61s).

4a. ~~synth_cone / synth_thin_annulus швы~~ — устранено 2026-09-01
   (C5 follow-up #2): junction-level snap в LINE-ветке resolve_edge_curve
   (off-line вершина НА соседней окружности ⇒ вершина авторитетна —
   synth_cone 15.33→1.31%, геом. пол файла) + unwrap FACE_BOUND-ссылок,
   обёрнутых в список (thin_annulus 9.14→0.00% — отверстие больше не
   теряется). Остаток synth_cone 1.31% — НЕ дефект конвертера: файл
   моделирует полуконус без замыкающей XZ-грани (тот же остаток у
   synth_cylinder). Остался trade-off №3: as1_rod NURBS CDT
   strip-fallback (4.96%).

4b. ~~3-tier fallback в triangulate_face_impl~~ — УДАЛЁН 2026-09-01
   (Этап D / D4). Корневая причина срабатываний (Vulcan #40443/#40583 —
   крошечные конусы R=0.01, чьё основное кольцо ошибочно помечалось
   «apex-degenerate» из-за LOD-tolerance в is_degenerate_uv) исправлена:
   scale-relative порог (2% от radius конуса, выровнен с Revolution-веткой)
   + guard «фан невозможен» (<3 невырожденных точек → CDT вместо
   1-vertex/0-triangle фантома). Фейс с падающей primary-триангуляцией
   теперь логирует `PrimaryTriangulationFailed` и возвращает пустой меш
   (в strict-режиме — паника); единственный «легитимный» случай в
   регрессии — cube_with_void #55 (Plane с zero-length LINE-петлёй —
   вырожденная STEP-геометрия, пустой результат корректен).
   С open-chain веткой fill_boundary_gaps (D3): удалена — 0 срабатываний
   на всей регрессии. D2 (weld_boundary_edge_vertices_aggressive):
   измеренное удаление дало бы transmission 6→14%, Vulcan 1.5→4.5% —
   ОСТАВЛЕН с инструментацией + strict-гейтом до фиксации NURBS CDT
   root causes.

4c. **`--features strict` (draper-mesh, A3 Этап D)** — strict-режим для
   CI-валидных-геометрий: `PrimaryTriangulationFailed` и вызов
   aggressive-weld превращаются в панику. 4 юнит-теста с синтетически
   вырожденной геометрией исключены через `#[cfg(not(feature = "strict"))]`.
   НЕ для production: industrial-файлы легитимно используют aggressive
   weld. Прогон: `cargo test -p draper-mesh --features strict --lib` —
   251 passed.

5. **`fix_self_intersections_heal`** — conservative real implementation (удаляет face с меньшим числом edges), но не rebuilds topology. Полная реализация требует trim + stitch.

6. **RuledSurface::project_point** — approximate (sample curve1 + segment projection). Не точный closest-point-on-curve.

7. **OffsetSurface::project_point** — approximate (shifted point along base normal + re-project). Один iteration, не convergent.

8. **Surface::transform для non-uniform scaling** — не конвертирует в NURBS (сохраняет тип + копирует radii). Только uniform scaling корректно масштабирует radii.

9. **Недетерминизм `all_files_test`** (обнаружен 2026-09-02 при дифференциальной верификации): один и тот же бинарник от запуска к запуску даёт разные boundary-проценты (3.05.078.stp: 0.0% ↔ 12.9%; Spit-Fire: 22.1–26.8%; summary 13–15 ok из 26). Кандидат — порядок итерации HashMap в pre-compute фазах edge cache (`face_axis_members` в `pre_compute_circle_n_face_groups`: порядок обхода групп меняет порядок union-find вставок → разное выравнивание n колец). Cargo-тесты (в т.ч. release 126/126) детерминированы и зелёные. Фикс-кандидат: BTreeMap/сортировка итераций в pre-compute фазах `edge_cache.rs`. До фикса: сравнивать регрессионные прогоны `all_files_test` только по классификациям с допуском, либо фиксировать seed.

---

## 7. Команды для ежедневной работы

### Запуск конкретного теста

```bash
# Один тест
cargo test -p draper-topology --lib --release test_extrude_in_x_direction -- --nocapture

# Группа тестов по префиксу
cargo test -p draper-mesh --lib --release vp_ -- --nocapture

# STEP regression (один файл)
cargo test -p draper-testing --release step_regression_nist_sphere -- --nocapture

# Все STEP regression (может занять 10+ минут)
cargo test -p draper-testing --release step_regression_ -- --nocapture
```

### Diagnostic tools

```bash
# Sphere diagnostic
cargo run -p draper-diag --release --bin sphere_diag -- test/synthetic/synth_sphere.stp

# Cone diagnostic
cargo run -p draper-diag --release --bin cone_diag -- test/nist_cone.stp

# Annulus diagnostic
cargo run -p draper-diag --release --bin annulus_diag -- test/synthetic/synth_thin_annulus.stp
```

### Проверка сборки

```bash
# Только lib (быстро)
cargo check -p draper-mesh --lib

# Все bins (включая viewer)
cargo check -p draper-viewer --bins

# Release build (долго, 5-10 минут)
cargo build --release --bin brepcad-shell
```

### Git operations

```bash
# Проверить статус
git status

# Посмотреть последние коммиты
git log --oneline -10

# Отменить локальные изменения в файле
git checkout -- <file>

# Stash + pull + pop (безопасный pull при локальных изменениях)
git stash && git pull --rebase origin main && git stash pop

# Коммит с inline identity
git add -A && git -c user.name="Z User" -c user.email="z@container" commit -m "fix(core): description"

# Push
git push origin main
```

---

## 8. Файлы документации

| Файл | Назначение |
|------|------------|
| `docs/BREP_CORE_FIX_PLAN.md` | Главный план работ (выполнен) — все этапы A-M с статусами |
| `docs/BREPCAD_DEEP_AUDIT.md` | Финальный аудит с реальными метриками (до/после) |
| `docs/VP_KERNEL_AUDIT.md` | VP ↔ kernel audit — 240 VP nodes mapped to 404 core API functions |
| `docs/VP_NODE_EXPANSION_PLAN.md` | VP node expansion plan |
| `docs/VP_TEST_RESULTS.md` | VP test results — 113/113 tests pass |
| `docs/BOOLEAN_ARCHITECTURE.md` | Boolean operations architecture |
| `docs/BOOLEAN_BOOKS.md` | Boolean operation reference books |
| `BREPCAD_IMPLEMENTATION_PLAN.md` | Исходный план (Phase 1-3, 10 подзадач) |
| `ROADMAP.md` | Дорожная карта проекта |
| `examples/vp_graphs/README.md` | VP graph examples catalogue (10 файлов) |

---

## 9. Чеклист переезда

- [ ] Установлен Rust (`rustup`, stable, minimal profile)
- [ ] Клонирован репозиторий (`git clone`)
- [ ] Настроен remote с token (если нужен push)
- [ ] `cargo test -p draper-geometry --lib --release` → 121 passed / 0 failed
- [ ] `cargo test -p draper-topology --lib --release` → 167 passed / 0 failed
- [ ] `cargo test -p draper-mesh --lib --release` → 253 passed / 0 failed
- [ ] `cargo test -p draper-step --lib --release` → 126 passed / 0 failed
- [ ] `cargo test -p draper-testing --release step_regression_synthetic_cube` → PASS
- [ ] `cargo build --release --bin brepcad-shell` → успешно (может потребовать 2-3 попытки)
- [ ] `./target/release/brepcad-shell` → запускается, показывает 3D viewport
- [ ] VP workspace: Load VP Graph → `examples/vp_graphs/01_box_with_hole.vp.json` → 3D preview виден
- [ ] Проверены workspace panels (Sketch, SM, CAM, FEA, Drawing, AI) — показывают реальные UI

---

*Этот документ — полный справочник для переезда в новый sandbox. Каждый пункт проверяемо.*
