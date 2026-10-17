// Copyright (c) 2026 KernelDev
//! Session-98 regression tests — the CASTELLATION extension cascade
//! (2-pt plateaus / v-flip + ring-reversal / fine level tolerance /
//! relaxed tall-band H-window + earcut tilted teeth).
//!
//! Root cause chain (measured on drill_top.stp SLEEVE BREP#32629): the
//! spiral-groove wall faces (f45 STEP#25173, f93 STEP#26243, f147
//! STEP#27631, f152 STEP#27996) are 3-level castellations the s86
//! structure detection REJECTS: their tooth tops are 2-POINT plateau
//! runs (the s95 decimation keeps only the endpoints of each tooth-top
//! micro-edge — s86 required ≥3 pts), f93/f152 carry the dominant rim
//! at the domain TOP with teeth hanging BELOW (the mirrored class —
//! v-flip + ring reversal canonicalizes), and the short teeth (0.08
//! param) merge under the 0.08×vspan cluster tolerance on the tall
//! (1.13 vspan) band. The legacy earcutr/CDT emission then fills the
//! ring with domain-spanning corner fans (u-widths up to 2π) whose
//! overlapping blades are the 5 REAL census pairs: (93,93)/(147,147)/
//! (152,152) same-face FOLD-OVER + (45,46)×2 cross-face.
//!
//! The cascade (deterministic order, first success wins): the s86
//! original pass first (bit-identity for f39/f41/f43-class faces),
//! then (id, tol .02, min-run 2) for the f45/f147 class (vertical
//! walls, 2-pt tops), then (flip+rev, tol .005, min-run 2) for the
//! f93/f152 mirrored class. Tilted-wall teeth (rounded corners
//! decimated to inward diagonals) are earcut as simple polygons —
//! the u-criterion zipper fans across the base on the angularly
//! non-monotone path (measured +1.3-2.6% ABS area). A 3D same-face
//! fold guard rejects any residual fold (measured margin: max normal
//! angle 5-7.5° on all four faces).
//!
//! Tests:
//! 1. Default (cascade ON): the four groove faces + their plane
//!    neighbor f46 have ZERO >170° pairs in the final SLEEVE mesh
//!    (baseline: 6 — the 5 REAL + 1 sub-tol (152,152)).
//! 2. Kill-switch (DRAPPER_CAST_EXTEND=0): the fold pairs reappear
//!    (≥5) — proving the extension gates exactly this behavior.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

/// Env-var flips are process-global — serialize the kill-switch test
/// against the default-config test (the s89 FLAP_ENV_LOCK pattern).
static S98_ENV_LOCK: Mutex<()> = Mutex::new(());

/// Load the drill assembly and triangulate ONLY the SLEEVE BREP (#32629).
fn sleeve_instance() -> Option<draper_step::DetailedMeshInstance> {
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
        .find(|p| p.brep_id == 32629)
        .expect("SLEEVE BREP #32629 not found in drill assembly");
    let ctx = StepConversionContext::new(&step);
    ctx.triangulate_pending(p)
}

/// Count >170° triangle-normal pairs (the census metric) in the final
/// mesh where BOTH triangles belong to the groove-wall faces
/// {f45, f46, f93, f147, f152} (mesh face ids = triangle_face_ids).
fn groove_fold_pairs(inst: &draper_step::DetailedMeshInstance) -> usize {
    let mesh = &inst.mesh;
    let fids = mesh
        .triangle_face_ids
        .as_ref()
        .expect("SLEEVE mesh has no triangle_face_ids");
    let targets: HashSet<u64> = [45u64, 46, 93, 147, 152].into_iter().collect();
    let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            let a = t[k].min(t[(k + 1) % 3]);
            let b = t[k].max(t[(k + 1) % 3]);
            if a != b {
                edge_tris.entry((a, b)).or_default().push(ti);
            }
        }
    }
    let normal = |t: &[u32; 3]| -> Option<[f64; 3]> {
        let p = |i: u32| mesh.vertices[i as usize];
        let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
        let e1 = [b.x - a.x, b.y - a.y, b.z - a.z];
        let e2 = [c.x - a.x, c.y - a.y, c.z - a.z];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if l > 1e-15 {
            Some([n[0] / l, n[1] / l, n[2] / l])
        } else {
            None
        }
    };
    let mut count = 0usize;
    for ts in edge_tris.values() {
        if ts.len() != 2 {
            continue;
        }
        let f0 = fids[ts[0]];
        let f1 = fids[ts[1]];
        if !targets.contains(&f0) || !targets.contains(&f1) {
            continue;
        }
        if let (Some(n0), Some(n1)) = (normal(&mesh.triangles[ts[0]]), normal(&mesh.triangles[ts[1]]))
        {
            let dot = (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]).clamp(-1.0, 1.0);
            if dot.acos().to_degrees() > 170.0 {
                count += 1;
            }
        }
    }
    count
}

#[test]
fn s98_cast_extension_kills_sleeve_groove_folds() {
    let _guard = S98_ENV_LOCK.lock().unwrap();
    let inst = sleeve_instance().expect("SLEEVE BREP#32629 triangulation failed");
    let folds = groove_fold_pairs(&inst);
    assert_eq!(
        folds, 0,
        "groove-wall faces f45/f46/f93/f147/f152 still carry {} >170° pairs — the castellation extension cascade is not cleaning the spiral-groove walls",
        folds
    );
}

#[test]
fn s98_cast_extension_killswitch_reproduces_groove_folds() {
    let _guard = S98_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_CAST_EXTEND", "0");
    // s100 layer isolation (the s91 "enumerate LAYERS" discipline): the
    // seam born-fold split also unwelds the (45,46)x2 CROSS-face groove
    // pairs when the s98 cascade is off (measured: 6 -> 4 — defense in
    // depth, a feature not a bug). This leg pins the s98 cascade alone,
    // so the s100 pass must be off here too.
    std::env::set_var("DRAPPER_SEAM_FOLD_SPLIT", "0");
    let inst = sleeve_instance();
    std::env::remove_var("DRAPPER_CAST_EXTEND");
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let inst = inst.expect("SLEEVE BREP#32629 triangulation failed (kill-switch leg)");
    let folds = groove_fold_pairs(&inst);
    assert!(
        folds >= 5,
        "kill-switch leg: expected the groove fold pairs back (measured baseline: 6 — the 5 REAL + 1 sub-tol), got {}",
        folds
    );
}
