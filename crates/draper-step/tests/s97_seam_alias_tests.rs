// Copyright (c) 2026 KernelDev
//! Session-97 regression tests — the CONCENTRIC-CIRCLE seam-alias guard.
//!
//! Root cause chain (measured on drill_top.stp GEAR BREP#16033): the gear
//! tooth-space fillet cascade puts three same-axis arc families within one
//! vertex-pair tolerance: root-cylinder arcs (r=.1525, y=.01), cone-flank
//! arcs (r=.15957, y=.00293) and end-plane arcs (r=.16664, y=0/.23). The
//! cone flank face contains BOTH the root arc (#15817) and the cone arc
//! (#15832) — their vertex pairs are 0.01 apart (< sewing tol .0153) with
//! diameter-long chords, so the s95 relative-offset guard (chord vs off)
//! and the s80 deviation guard (5-pt dev .01 < 2×tol .0306) both pass
//! them. The alias chain then substituted the end-plane polyline for the
//! root arcs, and the half-cylinder band faces (STEP#15826/#15893 — the
//! two halves of the root band split at the antipodal generatrices u=0
//! and u=π) were fed boundary rings at the WRONG radius/height: their
//! parametric v-range widened from the STEP domain [.01, .22] to [0, .23]
//! and the seam flap triangles crossed into each other at 180° — the
//! (421,426)×2 FOLD-OVER REAL family (plus the (20,20)/(75,75) same-face
//! debt fed by the same poisoned rings).
//!
//! The guard: for circle-vs-circle alias candidates, require the SAME
//! circle (center + radius within 5% of the sewing tolerance). Genuine
//! seam twins and slit lips are the same circle (only the angular
//! extent/offset differs); the fillet cascade is co-axial circles of
//! DIFFERENT radii.
//!
//! Tests:
//! 1. Default (guard ON): the band faces' uv emission stays within the
//!    STEP v-domain [.01, .22] (+2e-3 slack) — no poisoned extension.
//! 2. Kill-switch (DRAPPER_SEAM_CIRCLE_GUARD=0): the legacy poisoned
//!    v-extension (≥ .23) reappears — proving the guard gates exactly
//!    this behavior.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::sync::Mutex;

/// Env-var flips are process-global — serialize the kill-switch test
/// against the default-config test (the s89 FLAP_ENV_LOCK pattern).
static S97_ENV_LOCK: Mutex<()> = Mutex::new(());

/// Load the drill assembly and triangulate ONLY the GEAR BREP (#16033).
fn gear_instance() -> Option<draper_step::DetailedMeshInstance> {
    let candidates = [
        "test/drill_top.stp".to_string(),
        "../../test/drill_top.stp".to_string(),
    ];
    let path = candidates
        .iter()
        .find(|p| std::path::Path::new(p).exists())
        .unwrap_or_else(|| panic!("test file not found: drill_top.stp"));
    let content = std::fs::read_to_string(path).unwrap();
    let step = parse_step(&content).unwrap();
    let (_tree, pending) = step_structure_lazy(&step);
    let p = pending
        .iter()
        .find(|p| p.brep_id == 16033)
        .expect("GEAR BREP #16033 not found in drill assembly");
    let ctx = StepConversionContext::new(&step);
    ctx.triangulate_pending(p)
}

/// The (v_min, v_max) extent of a face's UV emission (uv_triangles).
fn face_uv_v_extent(
    inst: &draper_step::DetailedMeshInstance,
    step_face_id: i64,
) -> (f64, f64) {
    let f = inst
        .faces
        .iter()
        .find(|f| f.step_face_id == step_face_id)
        .unwrap_or_else(|| panic!("face STEP#{} not found", step_face_id));
    let mut vmin = f64::MAX;
    let mut vmax = f64::MIN;
    for t in &f.uv_triangles {
        for p in t {
            vmin = vmin.min(p.v);
            vmax = vmax.max(p.v);
        }
    }
    assert!(
        !f.uv_triangles.is_empty(),
        "face STEP#{} emitted no uv triangles",
        step_face_id
    );
    (vmin, vmax)
}

#[test]
fn s97_concentric_guard_keeps_band_emission_in_step_domain() {
    let _guard = S97_ENV_LOCK.lock().unwrap();
    let inst = gear_instance().expect("GEAR BREP#16033 triangulation failed");
    // The two half-cylinder band faces of the root circle (r=.1525,
    // y∈[.01,.22]) — STEP#15826 owns u∈[π,2π], STEP#15893 owns u∈[0,π].
    for sid in [15826i64, 15893] {
        let (vmin, vmax) = face_uv_v_extent(&inst, sid);
        assert!(
            vmin >= 0.01 - 2e-3,
            "STEP#{}: emission v_min {:.4} below the STEP domain 0.01 — the concentric-circle alias chain is feeding the end-plane arcs again",
            sid, vmin
        );
        assert!(
            vmax <= 0.22 + 2e-3,
            "STEP#{}: emission v_max {:.4} above the STEP domain 0.22 — the concentric-circle alias chain is feeding the end-plane arcs again",
            sid, vmax
        );
    }
}

#[test]
fn s97_concentric_guard_killswitch_reproduces_poisoned_extension() {
    let _guard = S97_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_SEAM_CIRCLE_GUARD", "0");
    let inst = gear_instance();
    std::env::remove_var("DRAPPER_SEAM_CIRCLE_GUARD");
    let inst = inst.expect("GEAR BREP#16033 triangulation failed (kill-switch leg)");
    // With the guard off, the legacy alias chain substitutes the end-plane
    // arcs (y=0/.23) for the root arcs — the band faces' v-extent widens
    // past the STEP domain. Measured baseline: v∈[0.0000, 0.2300].
    let (vmin, vmax) = face_uv_v_extent(&inst, 15826);
    assert!(
        vmax > 0.225,
        "kill-switch leg: expected the poisoned v-extension beyond 0.225 (measured baseline 0.23), got v∈[{:.4},{:.4}]",
        vmin, vmax
    );
}
