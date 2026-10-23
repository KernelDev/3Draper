// Copyright (c) 2026 KernelDev
//! Session-106 regression tests — the CYLINDER BAND GRID (boundary
//! half-step rows/columns in `generate_cylinder_or_cone_steiner_grid`,
//! seam-safe scoping).
//!
//! Root cause chain (measured on the s105-final baseline): the
//! exclusive-indexed cylinder Steiner grid leaves the boundary strips
//! of PARTIAL-WRAP cylinder faces a FULL cell away from any interior
//! support. The legacy earcutr spike-chain pass then fills those
//! strips with 3-on-bnd CHORD FANS — height-spanning needles:
//!   * compressor COLLECTOR face-7 (#2429, half-pipe [0,π]×[7.33,9.33],
//!     n_u=23/n_v=8, last column a FULL cell (π/23 ≈ 0.137) left of the
//!     u=π cut edge): 9 needles around the top-rim apex poly#94, pairs
//!     on the bottom rim, max-edge 2.07 rad;
//!   * drill HOUSING face-37 (#39570, half-pipe [π/2,3π/2], 7 holes):
//!     the rescue-CDT's sliver guard skipped 46/98 interior points (the
//!     s82 long-chord deadlock) leaving a 42-tri fan around poly#8,
//!     max-edge 2.0 rad.
//! After the s105 sphere fix removed the cross-face material, the
//! winding BFS re-arbitrated the net-zero and manufactured the SAME-FACE
//! needle folds: (7,7) around weld apex 559 (comp) and (37,37) around
//! weld apex 5092 (drill) — the item-3 class.
//!
//! Fix: two axial rows (v = v_min+h_v / v_max−h_v) and two angular
//! columns (u = u_min+h_u / u_max−h_u) — ONLY on seam-safe faces
//! (partial wrap, no seam multiple strictly inside the u-range; the
//! seam-crossing class #33417 measurably degrades with ANY band points:
//! Hamiltonian chain re-routes through the s104 near-duplicate zone →
//! primary earcutr drops ring verts → rescue CDT skips 252/273 →
//! emission collapses 1326→216 tris and manufactures the (13,114)
//! h=7.0 cross-face fold).
//!
//! Tests:
//! 1. Default (band ON): the same-face needle folds — (7,7) in the
//!    COLLECTOR mesh and (37,37) in the drill HOUSING mesh — are ZERO
//!    (baseline: 1 each, the s105 residues).
//! 2. Kill-switch (DRAPPER_CYL_BAND_GRID=0, sphere band stays ON):
//!    both needle folds reappear (≥1) — proving the cylinder band
//!    gates exactly the needle behavior.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

/// Env-var flips are process-global — serialize the kill-switch test
/// against the default-config test (the s89 FLAP_ENV_LOCK pattern).
static S106_ENV_LOCK: Mutex<()> = Mutex::new(());

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
/// mesh where BOTH triangles belong to `face_id` — the same-face needle
/// family the band grid is supposed to kill (mesh face ids =
/// triangle_face_ids; the probe census `faces=(f,f)` refers to the same
/// ids).
fn same_face_needle_pairs(inst: &draper_step::DetailedMeshInstance, face_id: u64) -> usize {
    let mesh = &inst.mesh;
    let fids = mesh
        .triangle_face_ids
        .as_ref()
        .expect("mesh has no triangle_face_ids");
    let zone: HashSet<u64> = [face_id].into_iter().collect();
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
        if !(zone.contains(&f0) && zone.contains(&f1)) {
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
fn s106_cyl_band_grid_kills_needle_folds() {
    let _guard = S106_ENV_LOCK.lock().unwrap();
    // comp COLLECTOR (#2860): same-face (7,7) — the apex-559 needle pair.
    let inst = brep_instance("compressor-13920_top.stp", 2860)
        .expect("COLLECTOR BREP#2860 triangulation failed");
    let folds = same_face_needle_pairs(&inst, 7);
    assert_eq!(
        folds, 0,
        "COLLECTOR same-face (7,7) still carries {} >170° needle pairs — the cylinder band grid is not supporting the partial-wrap boundary strips",
        folds
    );
    // drill HOUSING (#47598): same-face (37,37) — the apex-5092 needle pair.
    let inst = brep_instance("drill_top.stp", 47598)
        .expect("HOUSING BREP#47598 triangulation failed");
    let folds = same_face_needle_pairs(&inst, 37);
    assert_eq!(
        folds, 0,
        "HOUSING same-face (37,37) still carries {} >170° needle pairs — the cylinder band grid is not supporting the partial-wrap boundary strips",
        folds
    );
}

#[test]
fn s106_cyl_band_killswitch_reproduces_needles() {
    let _guard = S106_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_CYL_BAND_GRID", "0");
    // NOTE: the sphere band (s105) stays ON — the OFF leg reproduces the
    // s105-final state, where both needle folds were the census residue.
    let comp = brep_instance("compressor-13920_top.stp", 2860);
    let drill = brep_instance("drill_top.stp", 47598);
    std::env::remove_var("DRAPPER_CYL_BAND_GRID");
    let comp = comp.expect("COLLECTOR BREP#2860 triangulation failed (kill-switch leg)");
    let drill = drill.expect("HOUSING BREP#47598 triangulation failed (kill-switch leg)");
    let folds7 = same_face_needle_pairs(&comp, 7);
    assert!(
        folds7 >= 1,
        "kill-switch leg: expected the COLLECTOR (7,7) needle fold back (measured s105 baseline: 1 — the apex-559 pair), got {}",
        folds7
    );
    let folds37 = same_face_needle_pairs(&drill, 37);
    assert!(
        folds37 >= 1,
        "kill-switch leg: expected the HOUSING (37,37) needle fold back (measured s105 baseline: 1 — the apex-5092 pair), got {}",
        folds37
    );
}
