// Copyright (c) 2026 KernelDev
//! Session-108 regression tests — the DEGENERATE WALL-FAN SUBDIVISION
//! (`repair_degenerate_wall_fan`, called from the nfb band emitter in
//! `parametric_domain::nurbs_fillet_band_strip`).
//!
//! Root cause (the s107 residue, fully attributed there): the drill
//! HOUSING_MIRROR (3,243) Plane×Nurbs chronic — the last drill REAL
//! fold. f243's right wall is a STRAIGHT trim (u=1.0048, 91 ring pts,
//! 1e-8 jitter): every right-wall band fan emits as a zero-area UV
//! sliver ([anchor, w[k-1], w[k]] with the apex ON the wall), so the
//! 68 wall ring edges ride on phantom triangles whose UV normals are
//! garbage — the postweld v5 vote reads them as None (the sliver is
//! UV-degenerate), the anti-wound seam state (f3 corner-fan × f243
//! band, opp_trav + 0.00°) survives, and the winding BFS
//! "harmonizes" it by flipping one side → the 180° cross-face pair.
//!
//! Fix (s108): per degenerate fan, the ONE zipper triangle carrying
//! the wall chord (anchor → the section's ceiling) is subdivided
//! along the chord through the interior wall points, fanning from
//! the chord tri's OFF-wall vertex — the new triangles have real UV
//! area, the vote is no longer None, the anti-wound state is
//! measured and killed at the emission level. Edge accounting is
//! conserved exactly (the proof is on the helper's doc comment).
//!
//! Tests:
//! 1. Default (repair ON): the (3,243) cross-face pair family in the
//!    HM (BREP#62542) final mesh is ZERO (baseline: 1 — the chronic).
//! 2. Kill-switch (DRAPPER_DEGEN_FAN_SPLIT=0): the pair reappears
//!    (≥1) — proving the subdivision gates exactly the (3,243) class.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

/// Env-var flips are process-global — serialize the kill-switch test
/// against the default-config test (the s89 FLAP_ENV_LOCK pattern).
static S108_ENV_LOCK: Mutex<()> = Mutex::new(());

fn brep_instance(file: &str, brep_id: i64) -> Option<draper_step::DetailedMeshInstance> {
    let candidates = [
        format!("test/{}", file),
        format!("../../test/{}", file),
    ];
    let path = candidates
        .iter()
        .find(|p| std::path::Path::new(p).exists())
        .unwrap_or_else(|| panic!("test file not found: {}", file));
    let content = std::fs::read_to_string(path).unwrap();
    let step = parse_step(&content).unwrap();
    let (_tree, pending) = step_structure_lazy(&step);
    let p = pending
        .iter()
        .find(|p| p.brep_id == brep_id)
        .unwrap_or_else(|| panic!("BREP #{} not found in {}", brep_id, file));
    let ctx = StepConversionContext::new(&step);
    ctx.triangulate_pending(p)
}

/// Count >170° triangle-normal pairs (the census metric) in the final
/// mesh where the two triangles of the shared edge belong to faces
/// `fa` and `fb` — the cross-face anti-wound family (the probe
/// census `faces=(fa,fb)` refers to the same mesh face ids).
fn cross_face_fold_pairs(inst: &draper_step::DetailedMeshInstance, fa: u64, fb: u64) -> usize {
    let mesh = &inst.mesh;
    let fids = mesh
        .triangle_face_ids
        .as_ref()
        .expect("mesh has no triangle_face_ids");
    let _zone: HashSet<u64> = [fa, fb].into_iter().collect();
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
        if !((f0 == fa && f1 == fb) || (f0 == fb && f1 == fa)) {
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
fn s108_degen_wall_fan_kills_anti_wound_pair() {
    let _guard = S108_ENV_LOCK.lock().unwrap();
    // drill HOUSING_MIRROR (#62542): cross-face (3,243) Plane×Nurbs —
    // the f243 straight-trim (u=1.0048) anti-wound chronic, the last
    // drill REAL fold of the s107 baseline.
    let inst = brep_instance("drill_top.stp", 62542).expect("HM BREP#62542 triangulation failed");
    let folds = cross_face_fold_pairs(&inst, 3, 243);
    assert_eq!(
        folds, 0,
        "HM cross-face (3,243) still carries {} >170° anti-wound pairs — \
         the degenerate wall-fan subdivision is not curing the straight-trim class",
        folds
    );
}

#[test]
fn s108_degen_fan_killswitch_reproduces_pair() {
    let _guard = S108_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_DEGEN_FAN_SPLIT", "0");
    // NOTE: no other layer gates the (3,243) family (s107 measured:
    // v5 vote None by design on UV-degenerate slivers, bornfold
    // fold_proof rejects the inverted pair BY DESIGN) — the OFF leg
    // disables ONLY this switch and must reproduce the chronic.
    let drill = brep_instance("drill_top.stp", 62542);
    std::env::remove_var("DRAPPER_DEGEN_FAN_SPLIT");
    let drill = drill.expect("HM BREP#62542 triangulation failed (kill-switch leg)");
    let folds = cross_face_fold_pairs(&drill, 3, 243);
    assert!(
        folds >= 1,
        "kill-switch leg must reproduce the (3,243) chronic (got {})",
        folds
    );
}
