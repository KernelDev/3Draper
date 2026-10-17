// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Session-100 regression tests — seam born-fold split (2-phase,
//! cross-face + same-face).
//!
//! The assembly-level fold census for pairs BORN folded (s99
//! root_cause_map): the cross-face EMISSION-FOLD classes + the same-face
//! MERGE-DEFORM class:
//!
//! 1. **TANGENT-STRIP FOLD** ((3,235)/(3,248)): opposite traversal,
//!    same-side apexes, fat anchor, BETWEEN the s89 flapclean's ear/dup
//!    removal classes → SPLIT (face-scoped vertex clone).
//! 2. **SAME-TRAVERSAL degenerate ear** ((6,265)): the sliver's apex on
//!    the edge line, same directed traversal → SPLIT.
//! 3. **SAME-FACE MERGE-DEFORM** ((62,62)/(206,206)): thin blades clean
//!    at emission, folded by the merge's near-coincident ring-vert snap,
//!    usage-blocked from the winding's own flap removal → SPLIT (per-tri
//!    vertex clone in the flap). Planar faces are excluded: on a plane a
//!    same-face >170° pair is ALWAYS winding/duplicate debt (the (3,3)
//!    fixpoint class, s100 item 3), never a deformation fold.
//!
//! Negative legs: the legit two-sided thin wall (same traversal, clearly
//! opposite-side apexes), planar same-face pairs, the s94 knife-fin
//! pinch, sub-tol weld noise, and the kill-switch.

use draper_geometry::Point3d;
use draper_mesh::{seam_bornfold_record, seam_bornfold_split, TriangleMesh};
use std::collections::HashSet;

/// Serializes env-var mutation (cargo test runs threads in-process).
static S100_ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn v(x: f64, y: f64, z: f64) -> Point3d {
    Point3d { x, y, z }
}

fn build(verts: Vec<Point3d>, tris: Vec<[u32; 3]>, fids: Vec<u64>) -> TriangleMesh {
    let mut m = TriangleMesh::from_data(verts, tris);
    m.triangle_face_ids = Some(fids);
    m
}

fn no_planar() -> HashSet<u64> {
    HashSet::new()
}

fn planar(fids: &[u64]) -> HashSet<u64> {
    fids.iter().copied().collect()
}

/// Count shared (2-owner) edges in the mesh.
fn shared_edges(mesh: &TriangleMesh) -> usize {
    use std::collections::HashMap;
    let mut em: HashMap<(u32, u32), usize> = HashMap::new();
    for t in &mesh.triangles {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let k = if a < b { (a, b) } else { (b, a) };
            *em.entry(k).or_default() += 1;
        }
    }
    em.values().filter(|&&c| c == 2).count()
}

// ── TANGENT-STRIP FOLD (the (3,235) class): cross-face split fires ──
// Edge B(0,0,0)→C(1,0,0); both triangles in the z=0 plane, anti-wound,
// apexes on the SAME side (y<0), areas far apart (no s89 class), apexes
// far apart (no knife-fin), heights 5 / 0.25 over res_tol 0.1.
#[test]
fn s100_tangent_strip_fold_splits() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let mesh = build(
        vec![v(0.5, -5.0, 0.0), v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.55, -0.25, 0.0)],
        vec![[0, 1, 2], [3, 2, 1]],
        vec![1, 2],
    );
    let rec = seam_bornfold_record(&mesh, 0.1, &no_planar(), 0.1);
    assert_eq!(rec.len(), 1, "tangent-strip pair must be recorded");

    let mut m = mesh.clone();
    let n_tris_before = m.triangles.len();
    let n_verts_before = m.vertices.len();
    let n_shared_before = shared_edges(&m);
    let n = seam_bornfold_split(&mut m, 0.1, &rec, &no_planar(), 0.1);
    assert_eq!(n, 1, "exactly one split");
    assert_eq!(m.triangles.len(), n_tris_before, "no triangle added/removed");
    assert_eq!(m.vertices.len(), n_verts_before + 1, "one cloned vertex");
    assert_eq!(
        shared_edges(&m),
        n_shared_before - 1,
        "the fold edge is no longer shared"
    );
    // The two faces no longer share ANY edge.
    let fids = m.triangle_face_ids.as_ref().unwrap();
    use std::collections::HashMap;
    let mut em: HashMap<(u32, u32), Vec<u64>> = HashMap::new();
    for (ti, t) in m.triangles.iter().enumerate() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let k = if a < b { (a, b) } else { (b, a) };
            em.entry(k).or_default().push(fids[ti]);
        }
    }
    for owners in em.values() {
        let cross = owners.iter().any(|f| *f == 1) && owners.iter().any(|f| *f == 2);
        assert!(!cross, "faces 1/2 must not share an edge after the split");
    }
}

// ── SAME-TRAVERSAL degenerate ear (the (6,265) class): split fires ──
// Fat needle (apex 2 below the edge) + a nearly-collinear sliver whose
// apex lies on the edge line, welded with the SAME directed traversal.
#[test]
fn s100_same_traversal_degenerate_ear_splits() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let mesh = build(
        vec![v(0.5, -2.0, 0.0), v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(1.5, 0.001, 0.0)],
        vec![[0, 1, 2], [3, 1, 2]],
        vec![1, 2],
    );
    let rec = seam_bornfold_record(&mesh, 0.1, &no_planar(), 0.1);
    assert_eq!(
        rec.len(),
        1,
        "same-traversal degenerate-ear pair must be recorded"
    );
    let mut m = mesh.clone();
    let n = seam_bornfold_split(&mut m, 0.1, &rec, &no_planar(), 0.1);
    assert_eq!(n, 1);
    assert_eq!(m.vertices.len(), mesh.vertices.len() + 1);
}

// ── SAME-FACE MERGE-DEFORM (the (62,62) class): per-tri split fires ──
// Same geometry as the tangent-strip case but both triangles on ONE
// CURVED face (not in the planar set): the smaller-area flap gets a
// per-tri vertex clone — the pair detaches, the rest of the face is
// untouched.
#[test]
fn s100_same_face_curved_merge_deform_splits() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let mesh = build(
        vec![v(0.5, -5.0, 0.0), v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.55, -0.25, 0.0)],
        vec![[0, 1, 2], [3, 2, 1]],
        vec![7, 7],
    );
    let rec = seam_bornfold_record(&mesh, 0.1, &no_planar(), 0.1);
    assert_eq!(rec.len(), 1, "same-face curved merge-deform pair must be recorded");

    let mut m = mesh.clone();
    let n_shared_before = shared_edges(&m);
    let n = seam_bornfold_split(&mut m, 0.1, &rec, &no_planar(), 0.1);
    assert_eq!(n, 1, "exactly one split");
    assert_eq!(m.vertices.len(), mesh.vertices.len() + 1, "one cloned vertex");
    assert_eq!(m.triangles.len(), 2, "no triangle added/removed");
    assert_eq!(shared_edges(&m), n_shared_before - 1, "fold edge unshared");
    // The FLAP (smaller-area tri 1) is the remapped one: it uses the
    // cloned vertex (index 4); the fat tri 0 keeps the original verts.
    assert!(m.triangles[1].contains(&(4 as u32)), "flap remapped to the clone");
    assert!(!m.triangles[0].contains(&(4 as u32)), "fat tri untouched");
}

// ── SAME-FACE on a PLANAR face: never recorded (the (3,3) class) ──
// On a plane every same-face >170° pair is winding/duplicate debt, not a
// deformation fold — the s100 item-3 territory, deferred.
#[test]
fn s100_same_face_planar_not_recorded() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let mesh = build(
        vec![v(0.5, -5.0, 0.0), v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.55, -0.25, 0.0)],
        vec![[0, 1, 2], [3, 2, 1]],
        vec![7, 7],
    );
    let rec = seam_bornfold_record(&mesh, 0.1, &planar(&[7]), 0.1);
    assert!(
        rec.is_empty(),
        "planar same-face pairs are winding/dup debt (item 3), not ours"
    );
}

// ── Legit two-sided thin wall: never recorded ────────────────────
// Same traversal + 180° normals but clearly OPPOSITE-side apexes (the
// zero-thickness plate rim) — excluded by both fold-proof branches.
#[test]
fn s100_thin_wall_rim_not_recorded() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let mesh = build(
        vec![v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.5, 0.0, 1.0), v(0.5, 0.0, -1.0)],
        vec![[0, 1, 2], [0, 1, 3]],
        vec![1, 2],
    );
    let rec = seam_bornfold_record(&mesh, 0.1, &no_planar(), 0.1);
    assert!(rec.is_empty(), "thin-wall rim must not be recorded");
}

// ── Knife-fin pinch (s94 exempt class): never recorded ────────────
// Same-side apexes that nearly coincide (offset copies), both
// perpendicular gaps >= res_tol, wall-normal offset.
#[test]
fn s100_knife_fin_pinch_not_recorded() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let g = 0.05; // wall gap, res_tol = 0.01
    let mesh = build(
        vec![v(0.5, g, 1.0), v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.5, 0.0, 1.0)],
        vec![[0, 1, 2], [3, 2, 1]],
        vec![1, 2],
    );
    let rec = seam_bornfold_record(&mesh, 0.01, &no_planar(), 0.01);
    assert!(
        rec.is_empty(),
        "knife-fin pinch (offset copies across a resolvable gap) must not be recorded"
    );
}

// ── Sub-tol weld noise (both apex heights under res_tol): never recorded ──
#[test]
fn s100_subtol_not_recorded() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
    let mesh = build(
        vec![v(0.5, -0.005, 0.0), v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.55, -0.004, 0.0)],
        vec![[0, 1, 2], [3, 2, 1]],
        vec![1, 2],
    );
    let rec = seam_bornfold_record(&mesh, 0.1, &no_planar(), 0.1);
    assert!(rec.is_empty(), "both-thin pairs are sub-tol weld noise");
}

// ── Kill-switch: both phases disabled ─────────────────────────────
#[test]
fn s100_killswitch_disables_both_phases() {
    let _g = S100_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_SEAM_FOLD_SPLIT", "0");
    let mesh = build(
        vec![v(0.5, -5.0, 0.0), v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.55, -0.25, 0.0)],
        vec![[0, 1, 2], [3, 2, 1]],
        vec![1, 2],
    );
    let rec = seam_bornfold_record(&mesh, 0.1, &no_planar(), 0.1);
    assert!(rec.is_empty(), "kill-switch: record must be empty");
    let mut m = mesh.clone();
    let n = seam_bornfold_split(&mut m, 0.1, &[[1, 2, 0, 3]], &no_planar(), 0.1);
    assert_eq!(n, 0, "kill-switch: split must be a no-op");
    assert_eq!(m.vertices.len(), mesh.vertices.len());
    std::env::remove_var("DRAPPER_SEAM_FOLD_SPLIT");
}
