// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Session-91 regression tests — post-weld COMPONENT winding audit (v5).
//!
//! The s90 per-pair flip repaired one mixed edge of a WINDING-FLIP pair
//! and manufactured new mixed edges on the flipped member's OTHER edges
//! whenever the inverted region was larger than one triangle (measured
//! drill: audit layer +23 REAL — HOUSING +12, HM +12). The v5 audit
//! flips whole connected components of analytically-bad triangles,
//! gated on a per-component solid anchor, with a boundary fixpoint that
//! absorbs unreliable-winding neighbors and blocks flips that would
//! expose hidden genuine folds.
//!
//! These tests exercise the audit function DIRECTLY on synthetic
//! meshes (the emission-level inversion is not reproducible from a
//! synthetic STEP file — the weld noise that creates it needs the full
//! converter pipeline; the corpus A/B gates cover the end-to-end path).
//!
//! 1. **Cluster semantics**: a 2-triangle inverted cluster inside a
//!    correct plane fan flips ENTIRELY (both members) — the s90
//!    per-pair behavior would have flipped only one and broken its
//!    neighbor edge.
//! 2. **All-thin clusters are skipped** (solid-anchor gate): weld-noise
//!    clusters whose every member is under res_tol are left alone —
//!    flipping them just relabels pair classes (s90 lesson).
//! 3. **Idempotence + topology preservation**: a second audit run is a
//!    no-op; triangle/vertex counts and total area are preserved.

use draper_geometry::{Point3d, Surface};
use draper_mesh::TriangleMesh;
use draper_step::postweld_component_winding_audit;
use std::collections::HashMap;

/// 4-triangle planar grid on z=0 (two unit quads), one shared face_id,
/// all triangles emitted CCW (+z normals) — then t2/t3 INVERTED,
/// forming a connected 2-triangle bad cluster adjacent to t0/t1.
fn synthetic_cluster_mesh() -> TriangleMesh {
    let vertices: Vec<Point3d> = vec![
        Point3d::new(0.0, 0.0, 0.0), // 0
        Point3d::new(1.0, 0.0, 0.0), // 1
        Point3d::new(2.0, 0.0, 0.0), // 2
        Point3d::new(0.0, 1.0, 0.0), // 3
        Point3d::new(1.0, 1.0, 0.0), // 4
        Point3d::new(2.0, 1.0, 0.0), // 5
    ];
    // Correct CCW emission (viewed from +z):
    //   t0 = 0-1-4, t1 = 0-4-3 (left quad), t2 = 1-2-5, t3 = 1-5-4 (right quad).
    // t2/t3 inverted (swap 1,2) — the weld-inverted cluster analog.
    let triangles: Vec<[u32; 3]> = vec![
        [0, 1, 4], // t0 correct
        [0, 4, 3], // t1 correct
        [1, 5, 2], // t2 INVERTED
        [1, 4, 5], // t3 INVERTED
    ];
    let mut mesh = TriangleMesh::from_data(vertices, triangles);
    mesh.triangle_face_ids = Some(vec![1, 1, 1, 1]);
    mesh
}

fn plane_face_surf() -> HashMap<u64, (&'static Surface, bool, &'static str)> {
    // A static Plane so the map can hold a reference.
    static PLANE: std::sync::OnceLock<Surface> = std::sync::OnceLock::new();
    let plane = PLANE.get_or_init(|| Surface::Plane(draper_geometry::Plane::xy()));
    let mut m: HashMap<u64, (&Surface, bool, &str)> = HashMap::new();
    m.insert(1, (plane, true, "Plane"));
    m
}

/// (n_pos_z, n_neg_z) winding census of the mesh triangles.
fn winding_census(mesh: &TriangleMesh) -> (usize, usize) {
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

fn total_area(mesh: &TriangleMesh) -> f64 {
    mesh.triangles
        .iter()
        .map(|t| {
            let (a, b, c) = (
                &mesh.vertices[t[0] as usize],
                &mesh.vertices[t[1] as usize],
                &mesh.vertices[t[2] as usize],
            );
            let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
            let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
            let n = (
                e1.1 * e2.2 - e1.2 * e2.1,
                e1.2 * e2.0 - e1.0 * e2.2,
                e1.0 * e2.1 - e1.1 * e2.0,
            );
            0.5 * (n.0 * n.0 + n.1 * n.1 + n.2 * n.2).sqrt()
        })
        .sum()
}

#[test]
fn s91_component_flip_repairs_whole_cluster() {
    let mut mesh = synthetic_cluster_mesh();
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = plane_face_surf();
    // res_tol far below the unit-scale triangle thickness (1.0) — solid.
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 0.001, 42);
    // BOTH cluster members flipped — the s90 per-pair audit would have
    // flipped only the member of the pair it looked at (and broken the
    // shared cluster edge, the +23 regression mechanism).
    assert_eq!(flipped, 2, "expected the whole 2-triangle cluster to flip, got {flipped}");
    let (pos, neg) = winding_census(&mesh);
    assert_eq!((pos, neg), (4, 0), "all triangles must wind +z after the cluster flip");
}

#[test]
fn s91_allthin_cluster_is_skipped() {
    let mut mesh = synthetic_cluster_mesh();
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = plane_face_surf();
    // res_tol ABOVE every triangle's thickness (1.0): the bad cluster is
    // all-thin weld noise — the solid-anchor gate must skip it.
    let flipped = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 10.0, 42);
    assert_eq!(flipped, 0, "an all-thin (weld-noise) cluster must NOT be flipped");
    let (pos, neg) = winding_census(&mesh);
    assert_eq!((pos, neg), (2, 2), "windings must be untouched");
}

#[test]
fn s91_audit_idempotent_and_topology_preserving() {
    let mut mesh = synthetic_cluster_mesh();
    let fids = mesh.triangle_face_ids.clone().unwrap();
    let face_surf = plane_face_surf();
    let area_before = total_area(&mesh);
    let n_tris = mesh.triangles.len();
    let n_verts = mesh.vertices.len();

    let first = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 0.001, 42);
    assert_eq!(first, 2);
    assert_eq!(mesh.triangles.len(), n_tris, "triangle count must be preserved");
    assert_eq!(mesh.vertices.len(), n_verts, "vertex count must be preserved");
    assert!(
        (total_area(&mesh) - area_before).abs() < 1e-12,
        "total area must be preserved (winding flip is area-neutral)"
    );

    // Second run: every triangle now votes good — no bad components, no
    // absorptions possible (the neighbors are vote-good with >10°
    // as-wound angles). Must be a no-op.
    let second = postweld_component_winding_audit(&mut mesh, &fids, &face_surf, 0.001, 42);
    assert_eq!(second, 0, "the audit must be idempotent");
    let (pos, neg) = winding_census(&mesh);
    assert_eq!((pos, neg), (4, 0));
}
