// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! C5 follow-up #2 — seam/loop regression tests.
//!
//! Covers two STEP-converter robustness fixes:
//!
//! 1. **Junction-level snap** (`resolve_edge_curve` LINE branch):
//!    when exactly one vertex lies off the LINE and the line-to-vertex
//!    angle is small, the old heuristic kept the (broken) line. The new
//!    check looks at the junction neighbors: if the off-line vertex lies
//!    ON a neighboring edge's circle, the vertex is authoritative and the
//!    line is replaced with the chord through both vertices.
//!      - synth_cone.stp: seam LINE #42 is vertical but its end vertex
//!        (3,0,10) lies ON the top circle (r=3 @ z=10) → snapped.
//!      - nist_cylinder.stp: the off-line vertex (0,0,10) is the CENTER
//!        of the adjacent circle (degenerate-vertex convention) → line
//!        kept as-is.
//!
//! 2. **List-wrapped FACE_BOUND loop reference**: some exporters emit
//!    `FACE_BOUND('', (#90), .T.)` instead of the standard direct ref
//!    `FACE_BOUND('', #90, .T.)`. `get_ref` on a List returns None, so
//!    the bound was silently dropped — for an inner bound that meant a
//!    lost hole (the face triangulated as a full disk, covering the hole).
//!      - synth_thin_annulus.stp: top annulus face hole (#92) is
//!        list-wrapped; bottom annulus hole (#85) is a direct ref.

use draper_geometry::Surface;
use draper_geometry::Point3d;
use draper_mesh::triangulate::TriangulationParams;
use draper_mesh::triangulate_solid_with_report;
use draper_step::{extract_solids, parse_step};

/// Read + parse a STEP file from the workspace test directory.
fn load_solids(filename: &str) -> Vec<draper_topology::Solid> {
    let candidates = [
        format!("test/{}", filename),       // workspace root
        format!("../../test/{}", filename), // crate dir
    ];
    let path = candidates
        .iter()
        .find(|p| std::path::Path::new(p).exists())
        .unwrap_or_else(|| panic!("test file not found: {} (cwd={:?})", filename, std::env::current_dir()));
    let content =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("failed to read {}: {}", filename, e));
    let step = parse_step(&content).unwrap_or_else(|e| panic!("failed to parse {}: {}", filename, e));
    let (solids, _) = extract_solids(&step);
    assert!(!solids.is_empty(), "{}: no solids extracted", filename);
    solids
}

fn dist(a: &Point3d, b: &Point3d) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

/// Does `face` (as owned by `solid`) have an edge whose evaluated
/// endpoints are (unordered) approximately equal to the given pair?
///
/// C5 Stage 7.2: STEP-loaded solids arrive with compacted (empty) `edges`
/// mirrors, so the edge set is resolved through the solid's `EdgeStore`
/// (`Solid::instance_edges` — the mirror re-derivation). Un-indexed faces
/// fall back to their mirrors inside the same call, so both payload shapes
/// answer identically.
fn face_has_edge_between(
    solid: &draper_topology::Solid,
    face: &draper_topology::Face,
    p: &Point3d,
    q: &Point3d,
    tol: f64,
) -> bool {
    for e in solid.instance_edges(face) {
        if let (Some(sp), Some(ep)) = (e.start_point(), e.end_point()) {
            if dist(&sp, p) <= tol && dist(&ep, q) <= tol {
                return true;
            }
            if dist(&sp, q) <= tol && dist(&ep, p) <= tol {
                return true;
            }
        }
    }
    false
}

// ============================================================
// 1. Junction-level snap: synth_cone.stp
// ============================================================

#[test]
fn synth_cone_seam_snapped_to_vertex_chord() {
    let solids = load_solids("synthetic/synth_cone.stp");
    let solid = &solids[0];

    // The conical side face must own a seam edge running from (5,0,0) to
    // (3,0,10) — the cone generator through the two vertices. Before the
    // junction-level snap this edge kept the (broken) vertical LINE
    // geometry (5,0,0)→(5,0,10), which misses the top circle by 2.0.
    let conical: Vec<&draper_topology::Face> = solid
        .faces()
        .into_iter()
        .filter(|f| matches!(f.surface.as_ref(), Some(Surface::Cone(_))))
        .collect();
    assert!(!conical.is_empty(), "synth_cone: no conical face found");

    let p_bottom = Point3d::new(5.0, 0.0, 0.0);
    let p_top = Point3d::new(3.0, 0.0, 10.0);
    let p_broken = Point3d::new(5.0, 0.0, 10.0);

    let mut has_snapped_seam = false;
    let mut has_broken_vertical = false;
    for face in &conical {
        if face_has_edge_between(solid, face, &p_bottom, &p_top, 1e-6) {
            has_snapped_seam = true;
        }
        if face_has_edge_between(solid, face, &p_bottom, &p_broken, 0.1) {
            has_broken_vertical = true;
        }
    }
    assert!(has_snapped_seam, "synth_cone: seam edge was not snapped to the vertex chord (5,0,0)->(3,0,10)");
    assert!(!has_broken_vertical, "synth_cone: seam edge still follows the broken vertical line to (5,0,10)");
}

#[test]
fn synth_cone_boundary_at_geometric_floor() {
    let solids = load_solids("synthetic/synth_cone.stp");
    let params = TriangulationParams::default();
    let result = triangulate_solid_with_report(&solids[0], &params);
    // The synthetic file models a HALF cone without the closing XZ-plane
    // face, so exactly 4 boundary edges (1.31%) remain around the missing
    // face — the same pre-existing floor as synth_cylinder. Before the
    // junction-level snap this was 15.00% (63/420).
    assert!(
        result.report.boundary_pct <= 2.0,
        "synth_cone: boundary_pct={} exceeds the 2.0% geometric floor",
        result.report.boundary_pct
    );
    // No mesh vertex may sit near (5,0,10) — the broken seam's old top
    // endpoint, which lies 2.0 off the cone surface.
    let bad = result
        .mesh
        .vertices
        .iter()
        .any(|v| dist(v, &Point3d::new(5.0, 0.0, 10.0)) < 0.5);
    assert!(!bad, "synth_cone: mesh contains a vertex near the broken seam top (5,0,10)");
}

// ============================================================
// 2. Degenerate center-vertex must NOT be snapped: nist_cylinder.stp
// ============================================================

#[test]
fn nist_cylinder_center_vertex_line_is_kept() {
    let solids = load_solids("nist_cylinder.stp");
    let solid = &solids[0];

    // The cylinder seam edge must keep the vertical LINE geometry
    // (5,0,0)→(5,0,10): its second vertex (0,0,10) is the CENTER of the
    // adjacent top circle (degenerate-vertex convention for full-circle
    // edges), NOT a point on the circle — so the junction-level snap must
    // not fire.
    let cylindrical: Vec<&draper_topology::Face> = solid
        .faces()
        .into_iter()
        .filter(|f| matches!(f.surface.as_ref(), Some(Surface::Cylinder(_))))
        .collect();
    assert!(!cylindrical.is_empty(), "nist_cylinder: no cylindrical face found");

    let p_bottom = Point3d::new(5.0, 0.0, 0.0);
    let p_top = Point3d::new(5.0, 0.0, 10.0);
    let p_center = Point3d::new(0.0, 0.0, 10.0);

    let mut has_vertical_seam = false;
    let mut has_snapped_to_center = false;
    for face in &cylindrical {
        if face_has_edge_between(solid, face, &p_bottom, &p_top, 1e-6) {
            has_vertical_seam = true;
        }
        // A wrong snap would bend the seam toward the circle center.
        if face_has_edge_between(solid, face, &p_bottom, &p_center, 0.1) {
            has_snapped_to_center = true;
        }
    }
    assert!(has_vertical_seam, "nist_cylinder: vertical seam line was lost");
    assert!(!has_snapped_to_center, "nist_cylinder: seam wrongly snapped toward the circle center");

    let params = TriangulationParams::default();
    let result = triangulate_solid_with_report(&solids[0], &params);
    assert_eq!(
        result.report.boundary_pct, 0.0,
        "nist_cylinder: boundary_pct={} — mesh no longer watertight",
        result.report.boundary_pct
    );
}

// ============================================================
// 3. List-wrapped FACE_BOUND loop reference: synth_thin_annulus.stp
// ============================================================

#[test]
fn synth_thin_annulus_top_face_hole_preserved() {
    let solids = load_solids("synthetic/synth_thin_annulus.stp");
    let solid = &solids[0];

    // Both planar annulus faces (z=0 and z=1) must carry one inner wire
    // (the r=4.9 hole). Before the fix the top face's hole was silently
    // dropped because its FACE_BOUND loop ref was list-wrapped:
    //   #92 = FACE_BOUND('', (#90), .T.);
    let mut checked_planes = 0;
    for face in solid.faces() {
        if let Some(Surface::Plane(p)) = face.surface.as_ref() {
            if (p.origin.z - 1.0).abs() < 1e-6 {
                checked_planes += 1;
                assert_eq!(
                    face.inner_wires.len(),
                    1,
                    "thin_annulus top plane: expected 1 inner wire (hole), got {}",
                    face.inner_wires.len()
                );
            }
        }
    }
    assert!(checked_planes >= 1, "thin_annulus: no top plane face found");
}

#[test]
fn synth_thin_annulus_watertight() {
    let solids = load_solids("synthetic/synth_thin_annulus.stp");
    let params = TriangulationParams::default();
    let result = triangulate_solid_with_report(&solids[0], &params);
    // Before the fix: 9.14% (51/558) — the top annulus covered the hole
    // and the inner cylinder's top ring stayed open. Now the hole is cut
    // and the mesh is fully watertight.
    assert_eq!(
        result.report.boundary_pct, 0.0,
        "thin_annulus: boundary_pct={} — hole mismatch on inner ring",
        result.report.boundary_pct
    );
}
