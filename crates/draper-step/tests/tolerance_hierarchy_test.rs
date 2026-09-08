// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Vision 2036 §1.1 — hierarchical tolerance propagation, STEP round-trip.
//!
//! Covers the full chain:
//! 1. Import: UNCERTAINTY_MEASURE_WITH_UNIT seeds every Face/Edge tolerance
//!    (via `ToleranceContext::entity_tolerance` → `Solid::apply_model_tolerance`),
//!    and shell/solid aggregates are rebuilt bottom-up.
//! 2. Export: `solid.tolerance` is written back as the file uncertainty
//!    (was: hardcoded 1e-6) — tolerance metadata survives the round-trip.
//! 3. Consistency: `validate_topology` with `check_tolerance_consistency`
//!    passes on propagated solids.

use draper_step::{export_step, extract_solids, parse_step};
use draper_topology::builder::ShapeBuilder;
use draper_topology::validation::{TopologyValidationConfig, validate_topology};

/// Export a box seeded with 0.01 tolerance, re-import it, and verify the
/// tolerance survived as entity tolerances AND as the exported uncertainty.
#[test]
fn test_tolerance_step_round_trip() {
    // ── Build + seed ──
    let mut solid = ShapeBuilder::make_box(2.0, 2.0, 2.0);
    let changed = solid.apply_model_tolerance(0.01);
    assert!(changed > 0, "seeding must bump entities (box has 6 faces + 12 edges)");
    assert!((solid.tolerance - 0.01).abs() < 1e-15,
        "solid aggregate must be 0.01 after seeding, got {}", solid.tolerance);

    // ── Export: uncertainty comes from solid.tolerance ──
    let step_text = export_step(&solid, "TOLBOX");
    assert!(
        step_text.contains("LENGTH_MEASURE(0.01)"),
        "exported uncertainty must be 0.01, got snippet: {}",
        step_text
            .lines()
            .find(|l| l.contains("UNCERTAINTY_MEASURE"))
            .unwrap_or("<no uncertainty entity>")
    );

    // ── Re-import: entity tolerances seeded from the file uncertainty ──
    let step_file = parse_step(&step_text).expect("round-tripped STEP must parse");
    let (solids, _) = extract_solids(&step_file);
    assert_eq!(solids.len(), 1, "one BREP expected in the round-trip");
    let reimported = &solids[0];

    let shell = reimported.outer_shell.as_ref().expect("outer shell");
    for face in &shell.faces {
        assert!(
            (face.tolerance - 0.01).abs() < 1e-9,
            "re-imported face tolerance must be 0.01, got {}",
            face.tolerance
        );
    }
    let n_edges = reimported.edge_store.len();
    assert!(n_edges > 0, "box must carry canonical edges");
    for edge in reimported.edge_store.iter() {
        assert!(
            (edge.tolerance - 0.01).abs() < 1e-9,
            "re-imported edge tolerance must be 0.01, got {}",
            edge.tolerance
        );
    }
    assert!(
        (reimported.tolerance - 0.01).abs() < 1e-9,
        "re-imported solid aggregate must be 0.01, got {}",
        reimported.tolerance
    );

    // ── The hierarchy is consistent after the round-trip ──
    let report = validate_topology(
        reimported,
        &TopologyValidationConfig {
            check_tolerance_consistency: true,
            ..TopologyValidationConfig::none()
        },
    );
    assert!(
        report.issues_for_check("ToleranceConsistency").is_empty(),
        "round-tripped solid must pass the tolerance-hierarchy check: {:?}",
        report.issues_for_check("ToleranceConsistency")
    );
}

/// Default-built solids keep the 1e-6 legacy behavior: export writes 1e-6,
/// import seeds ~coincidence-level tolerances (behavior unchanged).
#[test]
fn test_default_tolerance_unchanged() {
    let solid = ShapeBuilder::make_box(1.0, 1.0, 1.0);
    assert!((solid.tolerance - 1e-6).abs() < 1e-15,
        "builder default must stay 1e-6, got {}", solid.tolerance);

    let step_text = export_step(&solid, "DEFCUBE");
    assert!(
        step_text.contains("LENGTH_MEASURE(0.000001)"),
        "default export must write 1e-6, got snippet: {}",
        step_text
            .lines()
            .find(|l| l.contains("UNCERTAINTY_MEASURE"))
            .unwrap_or("<no uncertainty entity>")
    );

    let step_file = parse_step(&step_text).expect("default STEP must parse");
    let (solids, _) = extract_solids(&step_file);
    assert_eq!(solids.len(), 1);
    let reimported = &solids[0];
    // 1e-6 uncertainty → entity tolerance = 1e-6 (the seed equals it).
    let shell = reimported.outer_shell.as_ref().expect("outer shell");
    for face in &shell.faces {
        assert!(
            (face.tolerance - 1e-6).abs() < 1e-9,
            "default face tolerance must stay 1e-6, got {}",
            face.tolerance
        );
    }
}

/// Handwritten STEP with a coarse uncertainty (0.05) — the classic bolt
/// scenario (as1-oc-214 declares 0.01) — seeds exactly that value.
#[test]
fn test_coarse_uncertainty_import() {
    // Minimal STEP: a manifold BREP is required for extract_solids; reuse
    // the exporter to make a well-formed body, then patch the uncertainty.
    let solid = ShapeBuilder::make_box(2.0, 2.0, 2.0);
    let step_text = export_step(&solid, "COARSE");
    let patched = step_text.replace("LENGTH_MEASURE(0.000001)", "LENGTH_MEASURE(0.05)");

    let step_file = parse_step(&patched).expect("patched STEP must parse");
    let (solids, _) = extract_solids(&step_file);
    assert_eq!(solids.len(), 1);
    let imported = &solids[0];

    let shell = imported.outer_shell.as_ref().expect("outer shell");
    for face in &shell.faces {
        assert!(
            (face.tolerance - 0.05).abs() < 1e-9,
            "coarse uncertainty 0.05 must seed faces, got {}",
            face.tolerance
        );
    }
    assert!(
        (imported.tolerance - 0.05).abs() < 1e-9,
        "coarse uncertainty 0.05 must reach the solid aggregate, got {}",
        imported.tolerance
    );

    // A coarse-but-real precision is NOT capped away (model_scale is
    // several mm; 0.05 < scale).
    let report = validate_topology(
        imported,
        &TopologyValidationConfig {
            check_tolerance_consistency: true,
            ..TopologyValidationConfig::none()
        },
    );
    assert!(report.issues_for_check("ToleranceConsistency").is_empty());
}
