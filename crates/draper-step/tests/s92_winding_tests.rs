// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Session-92 regression tests — Nurbs voting + coplanar thin-flip (v6).
//!
//! Two new layers in `postweld_component_winding_audit` (each with its
//! own kill-switch — the s91 lesson: enumerate LAYERS, never count
//! them):
//!
//! 1. **Nurbs faces join the vote** (`DRAPPER_POSTWELD_NURBS_VOTE=0`
//!    disables): every Nurbs-face triangle is voted by 3/3 vertex
//!    disagreement against the analytic du×dv normal, inverted
//!    per-vertex through the NurbsNormalOracle (coarse grid +
//!    Gauss-Newton, cached by vertex). The emission reference is
//!    consistent across all tessellation paths (canonical CDT and
//!    the legacy grid both emit CCW-in-UV for forward faces).
//!    Measured: SPEEDOMETER 27→1, drill SLEEVE/HOUSING/HM ladders.
//!
//! 2. **Coplanar same-face thin exemption**
//!    (`DRAPPER_POSTWELD_PLANAR_THIN_FLIP=0` disables): an all-thin
//!    bad cluster confined to ONE PLANAR face is flipped — every
//!    interior edge it shares with the face's good triangles is
//!    coplanar, so the flip takes the as-wound dihedral from 180° to
//!    exactly 0° (the pair disappears; no relabeling is possible on a
//!    plane). The s90 "thin flip = relabel" lesson stays TRUE for
//!    curved surfaces — the all-thin skip remains for those.
//!
//! 3. **h-aligned free-skip in the fixpoint**: the free-relabeling
//!    skip now requires BOTH apex heights over the SHARED EDGE under
//!    tolerance (the census's own sub-tol semantics) — a thin strip
//!    with a long shared base is thin by 2·area/shortest but FAT by
//!    h, and skipping it manufactured +7 non-subtol pairs (drill,
//!    measured) which the h-gate eliminated.
//!
//! The synthetic meshes exercise the audit function DIRECTLY (the
//! weld noise that creates real inversions needs the full converter
//! pipeline; the corpus A/B gates cover the end-to-end path).

use draper_geometry::{CylinderSurface, Direction3d, NurbsSurface, Plane, Point3d, Surface};
use draper_mesh::TriangleMesh;
use draper_step::postweld_component_winding_audit;
use std::collections::HashMap;

/// Serializes env-var mutation (cargo test runs threads in-process).
static S92_ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

// ── mesh helpers (same census/area helpers as s91) ──────────────

fn winding_census_z(mesh: &TriangleMesh) -> (usize, usize) {
    let mut pos = 0usize;
    let mut neg = 0usize;
    for t in &mesh.triangles {
        let (a, b, c) = (
            &mesh.vertices[t[0] as usize],
            &mesh.vertices[t[1] as usize],
            &mesh.vertices[t[2] as usize],
        );
        let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
        let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
        let nz = e1.0 * e2.1 - e1.1 * e2.0;
        if nz > 0.0 {
            pos += 1;
        } else if nz < 0.0 {
            neg += 1;
        }
    }
    (pos, neg)
}

/// A bilinear (degree 1×1) NURBS patch over the unit square in the
/// z=0 plane: S(u,v) = (u, v, 0). du×dv = +z everywhere — the same
/// reference the canonical CDT emits CCW-in-UV triangles against.
fn unit_bilinear_nurbs() -> NurbsSurface {
    NurbsSurface::from_v_rows(
        1,
        1,
        vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0)],
            vec![Point3d::new(0.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 0.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        false,
        false,
    )
}

/// 4-vertex planar grid on z=0 (unit quad), one face_id, triangles
/// emitted CCW (+z) — `inverted` swaps the second one.
fn planar_two_tri_mesh(invert_second: bool) -> TriangleMesh {
    let vertices: Vec<Point3d> = vec![
        Point3d::new(0.0, 0.0, 0.0),
        Point3d::new(1.0, 0.0, 0.0),
        Point3d::new(0.0, 1.0, 0.0),
        Point3d::new(1.0, 1.0, 0.0),
    ];
    let t1 = if invert_second {
        [0, 3, 1] // INVERTED (winding −z)
    } else {
        [0, 1, 3] // correct CCW
    };
    let mut mesh = TriangleMesh::from_data(vertices, vec![[0, 1, 3], t1]);
    mesh.triangle_face_ids = Some(vec![1, 1]);
    mesh
}

fn nurbs_face_surf() -> HashMap<u64, (&'static Surface, bool, &'static str)> {
    static NURBS: std::sync::OnceLock<Surface> = std::sync::OnceLock::new();
    let nurbs = NURBS.get_or_init(|| Surface::Nurbs(unit_bilinear_nurbs()));
    let mut m: HashMap<u64, (&Surface, bool, &str)> = HashMap::new();
    m.insert(1, (nurbs, true, "Nurbs"));
    m
}

fn plane_face_surf() -> HashMap<u64, (&'static Surface, bool, &'static str)> {
    static PLANE: std::sync::OnceLock<Surface> = std::sync::OnceLock::new();
    let plane = PLANE.get_or_init(|| Surface::Plane(Plane::xy()));
    let mut m: HashMap<u64, (&Surface, bool, &str)> = HashMap::new();
    m.insert(1, (plane, true, "Plane"));
    m
}

// ── 1. Nurbs voting ─────────────────────────────────────────────

#[test]
fn s92_nurbs_vote_flips_inverted_nurbs_triangle() {
    let mut mesh = planar_two_tri_mesh(true);
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = nurbs_face_surf();
    // res_tol far below the unit-scale thickness — solid anchor.
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 0.001, 42);
    assert_eq!(
        flipped, 1,
        "the inverted Nurbs-face triangle must be voted bad and flipped"
    );
    let (pos, neg) = winding_census_z(&mesh);
    assert_eq!((pos, neg), (2, 0), "all triangles must wind +z after the flip");
}

#[test]
fn s92_nurbs_vote_killswitch_disables_layer() {
    let _guard = S92_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_POSTWELD_NURBS_VOTE", "0");
    let mut mesh = planar_two_tri_mesh(true);
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = nurbs_face_surf();
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 0.001, 42);
    std::env::remove_var("DRAPPER_POSTWELD_NURBS_VOTE");
    assert_eq!(flipped, 0, "NURBS_VOTE=0 must restore the v5 None-vote layer");
    let (pos, neg) = winding_census_z(&mesh);
    assert_eq!((pos, neg), (1, 1), "windings must be untouched");
}

#[test]
fn s92_nurbs_vote_forward_false_expects_opposite() {
    // !forward face: the expected winding is −du×dv — an emission that
    // winds +z is BAD against the flipped reference and must flip.
    let mut mesh = planar_two_tri_mesh(false); // both CCW (+z)
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let nurbs = nurbs_face_surf();
    let mut face_surf: HashMap<u64, (&Surface, bool, &str)> = HashMap::new();
    let (s, _, t) = *nurbs.get(&1).unwrap();
    face_surf.insert(1, (s, false, t));
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 0.001, 42);
    assert_eq!(flipped, 2, "a whole !forward face emitted CCW must flip wholesale");
    let (pos, neg) = winding_census_z(&mesh);
    assert_eq!((pos, neg), (0, 2), "all triangles must wind −z (!forward)");
}

// ── 2. Coplanar same-face thin exemption ────────────────────────

#[test]
fn s92_planar_allthin_cluster_flips() {
    let mut mesh = planar_two_tri_mesh(true);
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = plane_face_surf();
    // res_tol ABOVE every thickness (1.0): all-thin — but the cluster
    // sits on ONE PLANAR face, so the coplanar exemption applies: the
    // flip takes the shared-edge dihedral from 180° to exactly 0°.
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 10.0, 42);
    assert_eq!(
        flipped, 1,
        "a coplanar same-face all-thin cluster must flip (the s91 skip was for weld noise on CURVED pairs)"
    );
    let (pos, neg) = winding_census_z(&mesh);
    assert_eq!((pos, neg), (2, 0));
}

#[test]
fn s92_planar_allthin_killswitch_restores_v5_skip() {
    let _guard = S92_ENV_LOCK.lock().unwrap();
    std::env::set_var("DRAPPER_POSTWELD_PLANAR_THIN_FLIP", "0");
    let mut mesh = planar_two_tri_mesh(true);
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = plane_face_surf();
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 10.0, 42);
    std::env::remove_var("DRAPPER_POSTWELD_PLANAR_THIN_FLIP");
    assert_eq!(flipped, 0, "PLANAR_THIN_FLIP=0 must restore the v5 all-thin skip");
    let (pos, neg) = winding_census_z(&mesh);
    assert_eq!((pos, neg), (1, 1));
}

#[test]
fn s92_cylinder_allthin_cluster_still_skipped() {
    // The s90 "thin flip = relabel" lesson stays true for CURVED
    // surfaces: an all-thin bad cluster on a cylinder is weld noise —
    // flipping it relabels the pair class (dihedral stays ~180° on a
    // curved pair), so the solid-anchor gate must keep skipping it.
    static CYL: std::sync::OnceLock<Surface> = std::sync::OnceLock::new();
    let cyl = CYL.get_or_init(|| {
        Surface::Cylinder(CylinderSurface {
            origin: Point3d::ORIGIN,
            axis: Direction3d::Z,
            x_dir: Direction3d::X,
            radius: 1.0,
        })
    });
    // Two thin triangles on the cylinder wall near (1,0,0): the quad
    // (1,0,0)-(1,0,0.1)-(cos ε, sin ε, 0)-(cos ε, sin ε, 0.1) split
    // into two thin slivers, one inverted. Both are far under the
    // res_tol below (thin), the surface is curved → skip.
    let eps: f64 = 0.05;
    let vertices: Vec<Point3d> = vec![
        Point3d::new(1.0, 0.0, 0.0),
        Point3d::new(1.0, 0.0, 0.1),
        Point3d::new(eps.cos(), eps.sin(), 0.0),
        Point3d::new(eps.cos(), eps.sin(), 0.1),
    ];
    let mut mesh = TriangleMesh::from_data(vertices, vec![[0, 1, 3], [0, 3, 2]]);
    // Invert BOTH (a connected 2-triangle bad cluster on the wall):
    mesh.triangles[0] = [0, 3, 1];
    mesh.triangles[1] = [0, 2, 3];
    mesh.triangle_face_ids = Some(vec![1, 1]);
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let mut face_surf: HashMap<u64, (&Surface, bool, &str)> = HashMap::new();
    face_surf.insert(1, (cyl, true, "Cylinder"));
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 10.0, 42);
    assert_eq!(
        flipped, 0,
        "an all-thin cluster on a CURVED face must stay skipped (relabel lesson)"
    );
}

// ── 3. h-aligned free-skip (fixpoint) ────────────────────────────

#[test]
fn s92_hgate_blocks_thin_strip_with_long_base() {
    // A thin STRIP with a LONG shared base: thin by 2·area/shortest
    // (the s91 thickness gate would skip it as free) but FAT by the
    // apex height over the shared edge (h = 1.0 >> res_tol). The
    // good neighbor's vote must BLOCK the flip — the s92 h-gate.
    //
    // Layout (z=0 plane, face 1): t0 = the long base strip
    // [A=(0,0), B=(10,0), C=(5,1)] — thickness ≈ 2·area/shortest is
    // small (area 5, shortest side ≈ 9.1 → ≈1.1... use res_tol 2.0 so
    // it is thin), h over AB = 1.0. Its neighbor t1 = [A, C', B] with
    // C'=(5,-1) BELOW the base, correctly wound — a genuine fold-over
    // geometry the flip would EXPOSE. t0 is emitted inverted (bad).
    let vertices: Vec<Point3d> = vec![
        Point3d::new(0.0, 0.0, 0.0),  // 0 A
        Point3d::new(10.0, 0.0, 0.0), // 1 B
        Point3d::new(5.0, 1.0, 0.0),  // 2 C (t0's apex)
        Point3d::new(5.0, -1.0, 0.0), // 3 C' (t1's apex)
    ];
    // t0 INVERTED (bad), t1 correct — they share edge (0,1) in the
    // SAME direction (a pre-existing WINDING-FLIP pair, h = 1.0 both
    // sides, non-subtol at res_tol=2? h(1.0) < 2.0 → subtol...).
    let mut mesh = TriangleMesh::from_data(
        vertices,
        vec![[0, 1, 2], [0, 3, 1]], // t0 bad-ish, t1 good
    );
    mesh.triangle_face_ids = Some(vec![1, 1]);
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = plane_face_surf();
    // res_tol 2.0: both triangles thin (thickness < 2.0), h = 1.0 < 2.0
    // → still free-skip. Use res_tol 0.5 instead: thickness thin,
    // h = 1.0 ≥ 0.5 → NOT free → vote-good neighbor BLOCKS t0's flip.
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 0.5, 42);
    assert_eq!(
        flipped, 0,
        "thin-but-h-fat strip against a good neighbor must be BLOCKED, not skipped"
    );
}
