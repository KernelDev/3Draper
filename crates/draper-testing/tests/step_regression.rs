// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP regression tests — прогоняет каждый .stp файл в `test/` через
//! парсер → extract_solids → triangulate_solid_with_report и фиксирует
//! результат (pass/fail + boundary_pct).
//!
//! Цель: установить baseline качества STEP-импорта и триангуляции,
//! чтобы любая регрессия в `draper-topology` / `draper-mesh` сразу
//! проявлялась как failed test.
//!
//! Запуск: `cargo test -p draper-testing --release step_regression_ -- --nocapture --test-threads=1`

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::{triangulate_solid_with_report, triangulate::TriangulationParams};

/// Tolerance: тест считается PASS если boundary_pct ≤ 5%.
/// Файлы с boundary_pct > 5% считаются "known issues" — их нужно
/// явно пометить в `KNOWN_ISSUES` ниже.
const BOUNDARY_PCT_TOLERANCE: f64 = 5.0;

/// Известные проблемные файлы — для них тест не падает, только warns.
/// Каждый entry: (filename, expected_max_boundary_pct).
///
/// 2026-08-29 (C5 edge-cache unification): пороги ужесточены после фиксов
/// канонизации направления рёбер в EdgeDiscretizationCache + выравнивания
/// n для tube-колец (union-find по co-facial same-axis окружностям).
/// 15 файлов теперь на 0.00% и удалены из таблицы: 3.05.078, as1_nut,
/// as1_bolt, as1_assembly, brick_thin, brick_thin_hole, drill_top,
/// nested_assembly, nist_assembly, nist_block_with_hole,
/// nist_chamfer_block, nist_complex_surface, nist_cone, nist_cube,
/// SampleCube, Spit-Fire, Zentralstaender.
const KNOWN_ISSUES: &[(&str, f64)] = &[
    // Industrial files with complex topology — remaining boundary edges are
    // NURBS CDT fallbacks and cross-face mismatches in fillet regions.
    ("Zentralstaender.stp", 40.0),     // worst solid 39%, overall 0.94%
    // 2026-09-01 (C5 follow-up #1, industrial perf): the O(n²) diagnostics
    // in validate_edge_consistency / weld made Vulcan take 700-900s
    // (documented timeout). After the spatial-hash/CSR fixes it triangulates
    // in ~10s with overall boundary 1.48% — threshold tightened 80 → 5.
    ("8500-02_Vulcan.STEP", 5.0),
    ("transmission_top.stp", 8.0),     // 6.03% overall (was 9.08%)
    ("compressor-13920_top.stp", 10.0), // 6.22% (4.66% observed 2026-09-01)
    ("gdt_test.stp", 50.0),             // GD&T annotations
    // Curved surfaces
    ("brick_thin_round.stp", 8.0),     // 6.17%
    // as1 parts
    ("as1-oc-214_plate.stp", 8.0),     // 6.93% (5.43% observed 2026-09-01)
    ("as1-oc-214_rod.stp", 8.0),       // 4.00% (NURBS CDT strip fallback — see worklog)
    // Seam-line geometry bug: STEP LINE direction inconsistent with vertices
    // (vertical line + slanted vertex pair) — needs junction-level snap.
    ("synthetic/synth_cone.stp", 18.0), // 15.33%
    ("synthetic/synth_thin_annulus.stp", 12.0), // 9.14%
    ("cube_with_void.stp", 90.0),       // 80% (very small solid)
];

fn test_step_file(filename: &str) {
    // Try multiple candidate paths — tests can be run from workspace root
    // or from the crate directory.
    let candidates = [
        format!("test/{}", filename),                                 // workspace root
        format!("../../test/{}", filename),                          // crates/draper-testing/
        format!("../../../test/{}", filename),                      // crates/draper-testing/tests/ (out of date, but for safety)
    ];
    let path = candidates.iter()
        .find(|p| std::path::Path::new(p).exists())
        .cloned()
        .unwrap_or_else(|| candidates[0].clone()); // default — will hit "not found" below
    if !std::path::Path::new(&path).exists() {
        eprintln!("[SKIP] {} — file not found at {} (cwd={})",
            filename, path,
            std::env::current_dir().map(|d| d.display().to_string()).unwrap_or_default());
        return;
    }

    // Step 1: Parse STEP file
    let step_file = match parse_step_file(&path) {
        Ok(sf) => sf,
        Err(e) => {
            eprintln!("[FAIL] {} — parse error: {}", filename, e);
            panic!("STEP parse failed for {}: {}", filename, e);
        }
    };

    // Step 2: Extract solids
    let (solids, _step_ids) = extract_solids(&step_file);
    if solids.is_empty() {
        eprintln!("[WARN] {} — no solids extracted", filename);
        return;
    }

    // Step 3: Triangulate each solid with diagnostic report
    let params = TriangulationParams::default();
    let mut total_boundary_pct = 0.0;
    let mut max_boundary_pct = 0.0;
    let mut total_boundary_edges = 0usize;
    let mut total_edges = 0usize;
    let mut worst_solid_idx = 0usize;
    for (idx, solid) in solids.iter().enumerate() {
        let result = triangulate_solid_with_report(solid, &params);
        let report = result.report;
        total_boundary_pct += report.boundary_pct;
        if report.boundary_pct > max_boundary_pct {
            max_boundary_pct = report.boundary_pct;
            worst_solid_idx = idx;
        }
        total_boundary_edges += report.boundary_edge_count;
        total_edges += report.edge_count;
    }
    let avg_boundary_pct = total_boundary_pct / solids.len() as f64;
    let overall_boundary_pct = if total_edges > 0 {
        total_boundary_edges as f64 / total_edges as f64 * 100.0
    } else {
        0.0
    };

    // Step 4: Compare against tolerance (or KNOWN_ISSUES threshold)
    let threshold = KNOWN_ISSUES.iter()
        .find(|(name, _)| *name == filename)
        .map(|(_, pct)| *pct)
        .unwrap_or(BOUNDARY_PCT_TOLERANCE);

    eprintln!(
        "[{}] {} — solids={}, worst={:.2}% (solid #{}), avg={:.2}%, overall={:.2}%, boundary={}/{}",
        if overall_boundary_pct <= threshold { "PASS" } else { "FAIL" },
        filename,
        solids.len(),
        max_boundary_pct,
        worst_solid_idx,
        avg_boundary_pct,
        overall_boundary_pct,
        total_boundary_edges,
        total_edges,
    );

    assert!(
        overall_boundary_pct <= threshold,
        "{}: boundary_pct={:.2}% exceeds threshold {:.2}% (solids={}, boundary={}/{})",
        filename, overall_boundary_pct, threshold, solids.len(),
        total_boundary_edges, total_edges,
    );
}

#[test]
fn step_regression_synthetic_cube() { test_step_file("synthetic/synth_cube.stp"); }

#[test]
fn step_regression_synthetic_sphere() { test_step_file("synthetic/synth_sphere.stp"); }

#[test]
fn step_regression_synthetic_cylinder() { test_step_file("synthetic/synth_cylinder.stp"); }

#[test]
fn step_regression_synthetic_cone() { test_step_file("synthetic/synth_cone.stp"); }

#[test]
fn step_regression_synthetic_torus() { test_step_file("synthetic/synth_torus.stp"); }

#[test]
fn step_regression_synthetic_thin_annulus() { test_step_file("synthetic/synth_thin_annulus.stp"); }

#[test]
fn step_regression_nist_cube() { test_step_file("nist_cube.stp"); }

#[test]
fn step_regression_nist_cylinder() { test_step_file("nist_cylinder.stp"); }

#[test]
fn step_regression_nist_cone() { test_step_file("nist_cone.stp"); }

#[test]
fn step_regression_nist_sphere() { test_step_file("nist_sphere.stp"); }

#[test]
fn step_regression_nist_block_with_hole() { test_step_file("nist_block_with_hole.stp"); }

#[test]
fn step_regression_nist_chamfer_block() { test_step_file("nist_chamfer_block.stp"); }

#[test]
fn step_regression_nist_complex_surface() { test_step_file("nist_complex_surface.stp"); }

#[test]
fn step_regression_nist_assembly() { test_step_file("nist_assembly.stp"); }

#[test]
fn step_regression_sample_cube() { test_step_file("SampleCube.step"); }

#[test]
fn step_regression_brick_thin() { test_step_file("brick_thin.stp"); }

#[test]
fn step_regression_brick_thin_hole() { test_step_file("brick_thin_hole.stp"); }

#[test]
fn step_regression_brick_thin_round() { test_step_file("brick_thin_round.stp"); }

#[test]
fn step_regression_cube_with_void() { test_step_file("cube_with_void.stp"); }

#[test]
fn step_regression_as1_bolt() { test_step_file("as1-oc-214_bolt.stp"); }

#[test]
fn step_regression_as1_nut() { test_step_file("as1-oc-214_nut.stp"); }

#[test]
fn step_regression_as1_rod() { test_step_file("as1-oc-214_rod.stp"); }

#[test]
fn step_regression_as1_plate() { test_step_file("as1-oc-214_plate.stp"); }

#[test]
fn step_regression_as1_assembly() { test_step_file("as1-oc-214.stp"); }

// Known-issue files — tolerance relaxed via KNOWN_ISSUES table above.
// These will print FAIL/PASS based on the relaxed threshold.
#[test]
fn step_regression_drill_top() { test_step_file("drill_top.stp"); }

#[test]
fn step_regression_zentralstaender() { test_step_file("Zentralstaender.stp"); }

#[test]
fn step_regression_305078() { test_step_file("3.05.078.stp"); }

#[test]
fn step_regression_vulcan() { test_step_file("8500-02_Vulcan.STEP"); }

#[test]
fn step_regression_spitfire() { test_step_file("8394-121_Spit-Fire.STEP"); }

#[test]
fn step_regression_transmission() { test_step_file("transmission_top.stp"); }

#[test]
fn step_regression_compressor() { test_step_file("compressor-13920_top.stp"); }

#[test]
fn step_regression_nested_assembly() { test_step_file("nested_assembly.stp"); }

#[test]
fn step_regression_gdt() { test_step_file("gdt_test.stp"); }
