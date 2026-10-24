// Copyright (c) 2026 KernelDev
//! Session-105 regression tests — the SPHERE BAND GRID (boundary
//! half-step rows/columns in `generate_sphere_steiner_grid`).
//!
//! Root cause chain (measured on compressor-13920_top.stp COLLECTOR
//! BREP#2860): the sphere face-8 (#2442, the lune [π,2π]×[0.0334,π/2])
//! gets a 23×11 exclusive-indexed Steiner grid — the first/last rows
//! sit a FULL cell (0.14 rad) away from the UV bbox edges, and the
//! first/last columns 0.137 away from the meridians. The un-supported
//! boundary strips are filled by earcutr/rescue-CDT with 3-on-bnd
//! CHORD FANS (up to 1.34-rad flat chords through the dome): the
//! π-rim corner fan (40 tris, apex (π,1.5040), pairs on the rim arc
//! up to u=4.48) plus the 2π-meridian fan (21 tris) and the pole
//! corner fans. At merge, the winding BFS flips the chord-fan unit
//! as a whole (+y→−y: the chords are consistent with the face-7
//! cylinder needles but not with face-8's own dome) — manufacturing
//! the 12 REAL MERGE-ADJACENCY rim-zipper folds (cross-face (7,8)
//! Cylinder#2429 × Sphere#2442, all 12 pairs one defect, s101).
//!
//! Fix: two latitudinal rows at v=v_min+h_v / v_max−h_v and two
//! meridional columns at u=u_min+h_u / u_max−h_u (half a cell from
//! the bbox edges, POLE_EPS-pole rule respected) give the boundary
//! strip a support line, so the strip tiles with surface-hugging
//! triangles instead of flat chord fans; the winding BFS then has
//! no fan unit to flip (the fold is manufactured, net-zero — s101:
//! the ROOTS are at the emission level).
//!
//! Tests:
//! 1. Default (band ON): the rim-zipper family — cross-face (7,8)
//!    >170° pairs in the final COLLECTOR mesh — drops to ZERO
//!    (baseline: 12, all one defect).
//! 2. Kill-switch (DRAPPER_SPHERE_BAND_GRID=0): the cross-face (7,8)
//!    fold pairs reappear (≥10) — proving the band gates exactly
//!    the rim-zipper behavior.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

/// Env-var flips are process-global — serialize the kill-switch test
/// against the default-config test (the s89 FLAP_ENV_LOCK pattern).
static S105_ENV_LOCK: Mutex<()> = Mutex::new(());

/// Load the compressor assembly and triangulate ONLY the COLLECTOR
/// BREP (#2860) — the s101/s102/s104 target of the rim-zipper defect.
fn collector_instance() -> Option<draper_step::DetailedMeshInstance> {
    let candidates = [
        "test/compressor-13920_top.stp".to_string(),
        "../../test/compressor-13920_top.stp".to_string(),
    ];
    let path = candidates
        .iter()
        .find(|p| std::path::Path::new(p).exists())
        .unwrap_or_else(|| panic!("test file not found: compressor-13920_top.stp"));
    let content = std::fs::read_to_string(path).unwrap();
    let step = parse_step(&content).unwrap();
    let (_tree, pending) = step_structure_lazy(&step);
    let p = pending
        .iter()
        .find(|p| p.brep_id == 2860)
        .expect("COLLECTOR BREP #2860 not found in compressor assembly");
    let ctx = StepConversionContext::new(&step);
    ctx.triangulate_pending(p)
}

/// Count >170° triangle-normal pairs (the census metric) in the final
/// COLLECTOR mesh where the pair is CROSS-FACE between the rim faces
/// {7 (Cylinder #2429), 8 (Sphere #2442)} — the rim-zipper family the
/// band grid is supposed to kill (mesh face ids = triangle_face_ids;
/// the probe census `faces=(7,8)` refers to the same ids).
fn rim_zipper_pairs(inst: &draper_step::DetailedMeshInstance) -> usize {
    let mesh = &inst.mesh;
    let fids = mesh
        .triangle_face_ids
        .as_ref()
        .expect("COLLECTOR mesh has no triangle_face_ids");
    let z7: HashSet<u64> = [7u64].into_iter().collect();
    let z8: HashSet<u64> = [8u64].into_iter().collect();
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
        // cross-face (7,8) only — the rim-zipper family; same-face
        // (7,7)/(8,8) residue is the item-3 needle class (s106), not
        // this fix's contract.
        let cross = (z7.contains(&f0) && z8.contains(&f1)) || (z8.contains(&f0) && z7.contains(&f1));
        if !cross {
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
fn s105_band_grid_kills_collector_rim_zipper() {
    let _guard = S105_ENV_LOCK.lock().unwrap();
    let inst = collector_instance().expect("COLLECTOR BREP#2860 triangulation failed");
    let folds = rim_zipper_pairs(&inst);
    assert_eq!(
        folds, 0,
        "cross-face (7,8) rim edges still carry {} >170° pairs — the sphere band grid is not collapsing the chord-fan corner pockets near the lune rim",
        folds
    );
}

#[test]
fn s105_band_grid_killswitch_reproduces_rim_zipper() {
    let _guard = S105_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_SPHERE_BAND_GRID", "0");
    // s106: the cylinder band grid (DRAPPER_CYL_BAND_GRID, default ON)
    // independently kills the SAME rim-zipper family (face-7's emission
    // needles were the BFS flip material) — with only the sphere switch
    // off, the (7,8) count stays 0 and this leg can no longer observe
    // the pre-s105 state. Set BOTH kill-switches so the OFF leg isolates
    // exactly the sphere band's contribution (pre-s105 baseline: 12).
    std::env::set_var("DRAPPER_CYL_BAND_GRID", "0");
    // s107: the bornfold WINDING RECORD layer (DRAPPER_BORNFOLD_
    // WINDING_RECORD, default ON) ALSO gates the same zipper family —
    // the (7,8) rim-zipper pairs are BFS-manufactured (the s101
    // net-zero exchange: the winding BFS flips the chord fan against
    // the dome), so the post-winding record now catches them and
    // phase-2 splits the survivors. Third switch joined the family.
    std::env::set_var("DRAPPER_BORNFOLD_WINDING_RECORD", "0");
    // s108: the DEGENERATE WALL-FAN SUBDIVISION (DRAPPER_DEGEN_FAN_SPLIT,
    // default ON) ALSO gates the same zipper family — the chord-fan
    // corner pockets near the lune rim ride on zero-area wall fans
    // (straight u=const trim sections), and the subdivided emission
    // removes the BFS flip material at the source. Fourth switch
    // joined the family.
    std::env::set_var("DRAPPER_DEGEN_FAN_SPLIT", "0");
    let inst = collector_instance();
    std::env::remove_var("DRAPPER_SPHERE_BAND_GRID");
    std::env::remove_var("DRAPPER_CYL_BAND_GRID");
    std::env::remove_var("DRAPPER_BORNFOLD_WINDING_RECORD");
    std::env::remove_var("DRAPPER_DEGEN_FAN_SPLIT");
    let inst = inst.expect("COLLECTOR BREP#2860 triangulation failed (kill-switch leg)");
    let folds = rim_zipper_pairs(&inst);
    assert!(
        folds >= 10,
        "kill-switch leg: expected the cross-face (7,8) rim-zipper pairs back (measured baseline: 12 — one defect), got {}",
        folds
    );
}
