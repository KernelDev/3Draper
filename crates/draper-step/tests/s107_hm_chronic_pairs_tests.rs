// Copyright (c) 2026 KernelDev
//! Session-107 regression tests — two independent fixes kill 2 of the
//! 3 chronic HOUSING_MIRROR REAL fold pairs of drill_top.stp:
//!
//! 1. EFF_TOL RULER DRIFT (the (3,3) Plane COINCIDENT pair, h=0.030262):
//!    `triangulate_brep_detailed_gated` returns the FIRST pass's mesh
//!    (the halved-deviation retry is rejected) but `last_brep_eff_tol`
//!    kept the RETRY's value — the per-BREP snapshot
//!    (`brep_eff_tol_cache`, read by the census sub-tol classifier)
//!    judged the returned mesh with a ruler of a mesh that was thrown
//!    away. Measured: main-pass weld max 0.030286 vs leaked retry
//!    snapshot 0.0302xx; the (3,3) pair sat in the 6e-5 gap —
//!    free-skipped by the v5 fixpoint (correct for its own mesh:
//!    0.030262 < 0.030286) yet REAL by the census (h ≥ the leaked
//!    smaller ruler). Fix: capture the first pass's Cell value and
//!    restore it on every first-wins return path.
//!    Kill-switch: DRAPPER_EFF_TOL_RESTORE=0.
//!
//! 2. BORNFOLD WINDING RECORD (the (30,30) Nurbs same-face pair,
//!    h=0.0451): `fix_inconsistent_winding`'s greedy flood-fill
//!    manufactures 180° pairs as a side effect of enforcing manifold
//!    edge traversal (the RELABEL class: same-side apex overlap reads
//!    as same-direction traversal; the flip makes it a fold). The
//!    s100 bornfold split's phase-1 record is captured PRE-winding, so
//!    these pairs were never eligible. Fix: union the POST-winding
//!    census into the record — phase-2 still rescans the LIVE mesh
//!    post-v5 and splits only survivors (the same 2-phase safety the
//!    s100 design gives emission-folds).
//!    Kill-switches: DRAPPER_BORNFOLD_WINDING_RECORD=0 (this layer),
//!    DRAPPER_SEAM_FOLD_SPLIT=0 (both phases).
//!
//! Tests:
//! 1. Default: BOTH chronic pairs are gone from the drill HM census
//!    (REAL-rule pair count with the census's own eff_tol ruler = 0).
//! 2. DRAPPER_EFF_TOL_RESTORE=0: the (3,3) pair is REAL again (≥1) —
//!    proving the ruler fix gates exactly the COINCIDENT class.
//! 3. DRAPPER_BORNFOLD_WINDING_RECORD=0: the (30,30) pair is REAL
//!    again (≥1) — proving the record layer gates exactly the
//!    BFS-relabel class.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

/// Env-var flips are process-global — serialize the kill-switch tests
/// against the default-config test (the s89 FLAP_ENV_LOCK pattern).
static S107_ENV_LOCK: Mutex<()> = Mutex::new(());

/// Load drill_top.stp, triangulate the requested BREP, and count the
/// census-REAL pairs for the given face pair (the probe's rule: >170°
/// between the two triangle normals AND max apex height over the
/// shared edge line ≥ the BREP's effective tolerance). Everything
/// (context, triangulation, eff_tol lookup) lives in ONE scope so the
/// context's caches are consistent.
fn hm_real_pairs(brep_id: i64, face_a: u64, face_b: u64) -> usize {
    let candidates = ["test/drill_top.stp", "../../test/drill_top.stp"];
    let path = candidates
        .iter()
        .find(|p| std::path::Path::new(p).exists())
        .unwrap_or_else(|| panic!("test file not found: drill_top.stp"));
    let content = std::fs::read_to_string(path).unwrap();
    let step = parse_step(&content).unwrap();
    let (_tree, pending) = step_structure_lazy(&step);
    let p = pending
        .iter()
        .find(|p| p.brep_id == brep_id)
        .unwrap_or_else(|| panic!("BREP #{} not found in drill_top.stp", brep_id));
    let ctx = StepConversionContext::new(&step);
    let inst = ctx
        .triangulate_pending(p)
        .expect("triangulate_pending returned None");
    real_pairs_for_faces(&inst, &ctx, brep_id, face_a, face_b)
}

/// Count the census-REAL pairs (the probe's rule: >170° between the
/// two triangle normals AND max apex height over the shared edge line
/// ≥ the BREP's effective tolerance) whose two faces are exactly
/// (face_a, face_b) in either order — the census `faces=(a,b)` line.
fn real_pairs_for_faces(
    inst: &draper_step::DetailedMeshInstance,
    ctx: &StepConversionContext,
    brep_id: i64,
    face_a: u64,
    face_b: u64,
) -> usize {
    let mesh = &inst.mesh;
    let fids = mesh
        .triangle_face_ids
        .as_ref()
        .expect("mesh has no triangle_face_ids");
    let tol = ctx
        .brep_eff_tol(brep_id)
        .expect("BREP eff_tol snapshot missing");
    let mut edge_map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            let a = tri[k];
            let b = tri[(k + 1) % 3];
            if a != b {
                let key = if a < b { (a, b) } else { (b, a) };
                edge_map.entry(key).or_default().push(ti);
            }
        }
    }
    let nrm = |t: &[u32; 3]| -> Option<[f64; 3]> {
        let a = &mesh.vertices[t[0] as usize];
        let b = &mesh.vertices[t[1] as usize];
        let c = &mesh.vertices[t[2] as usize];
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
    let mut edges: Vec<_> = edge_map.iter().collect();
    edges.sort_unstable_by_key(|(e, _)| **e); // determinism
    for ((ea, eb), owners) in edges {
        if owners.len() != 2 {
            continue;
        }
        let (t0, t1) = (owners[0], owners[1]);
        let (f0, f1) = (fids[t0], fids[t1]);
        let faces_match = (f0 == face_a && f1 == face_b) || (f0 == face_b && f1 == face_a);
        if !faces_match {
            continue;
        }
        let (Some(n0), Some(n1)) = (nrm(&mesh.triangles[t0]), nrm(&mesh.triangles[t1])) else {
            continue;
        };
        let dot = (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]).clamp(-1.0, 1.0);
        if dot.acos().to_degrees() <= 170.0 {
            continue;
        }
        // census h: apex height over the shared-edge LINE
        let pa = &mesh.vertices[*ea as usize];
        let pb = &mesh.vertices[*eb as usize];
        let e = [pb.x - pa.x, pb.y - pa.y, pb.z - pa.z];
        let base = (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt();
        if base <= 1e-15 {
            continue;
        }
        let h = |t: &[u32; 3]| -> f64 {
            let third = t.iter().find(|&&v| v != *ea && v != *eb).copied();
            match third {
                Some(v) => {
                    let q = &mesh.vertices[v as usize];
                    let d = [q.x - pa.x, q.y - pa.y, q.z - pa.z];
                    let cr = [
                        d[1] * e[2] - d[2] * e[1],
                        d[2] * e[0] - d[0] * e[2],
                        d[0] * e[1] - d[1] * e[0],
                    ];
                    (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt() / base
                }
                None => f64::INFINITY,
            }
        };
        let (h0, h1) = (h(&mesh.triangles[t0]), h(&mesh.triangles[t1]));
        if h0 < tol && h1 < tol {
            continue; // sub-tol weld noise (the census rule)
        }
        count += 1;
    }
    count
}

const HM_BREP: i64 = 62542; // HOUSING_MIRROR of drill_top.stp

#[test]
fn s107_default_both_chronic_hm_pairs_dead() {
    let _g = S107_ENV_LOCK.lock().unwrap();
    let pair_33 = hm_real_pairs(HM_BREP, 3, 3);
    let pair_3030 = hm_real_pairs(HM_BREP, 30, 30);
    assert_eq!(
        pair_33, 0,
        "s107: HM (3,3) COINCIDENT pair must be census-subtol by default (eff_tol ruler fix)"
    );
    assert_eq!(
        pair_3030, 0,
        "s107: HM (30,30) BFS-relabel pair must be split by default (bornfold winding record)"
    );
}

#[test]
fn s107_killswitch_eff_tol_restore_reintroduces_33() {
    let _g = S107_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_EFF_TOL_RESTORE", "0");
    let pair_33 = hm_real_pairs(HM_BREP, 3, 3);
    std::env::remove_var("DRAPPER_EFF_TOL_RESTORE");
    assert!(
        pair_33 >= 1,
        "s107 OFF-leg: with DRAPPER_EFF_TOL_RESTORE=0 the retry eff_tol leaks into the snapshot and the (3,3) pair (h=0.030262 between the rulers) is census-REAL again"
    );
}

#[test]
fn s107_killswitch_winding_record_reintroduces_3030() {
    let _g = S107_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_BORNFOLD_WINDING_RECORD", "0");
    let pair_3030 = hm_real_pairs(HM_BREP, 30, 30);
    std::env::remove_var("DRAPPER_BORNFOLD_WINDING_RECORD");
    assert!(
        pair_3030 >= 1,
        "s107 OFF-leg: with DRAPPER_BORNFOLD_WINDING_RECORD=0 the BFS-relabel pair is never recorded and survives as a census-REAL fold"
    );
}
