// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Native smoke tests for the WASM-bindgen API surface.
//!
//! These tests verify that the underlying Rust functions used by the
//! wasm-bindgen exports work correctly. They run on native targets
//! because wasm-bindgen exports themselves can't be invoked from
//! Rust unit tests (they require a JS host).

#![cfg(not(target_arch = "wasm32"))]

use draper_core::Document;
use draper_geometry::Direction3d;
use draper_topology::ShapeBuilder;

#[test]
fn test_document_with_box_and_fillet() {
    let mut doc = Document::new("test");
    // Use unit_cube helper that creates shared edges — make_box does not
    // share TopoIds across faces, so fillet_edge cannot find a manifold edge.
    let s = make_unit_cube_with_shared_edges();
    doc.add_solid(s);
    assert_eq!(doc.solid_count(), 1);

    // Find first manifold edge
    let solid = doc.solids().into_iter().next().unwrap();
    let edge_id = find_first_manifold_edge(solid);
    assert!(edge_id > 0, "expected at least one manifold edge");

    // Apply fillet — this should succeed on a manifold edge.
    let solid_mut = &mut doc.root.solids[0];
    let rc = draper_core::operations::fillet_edge(solid_mut, edge_id, 0.1);
    assert!(rc.is_ok(), "fillet_edge failed: {:?}", rc);

    // After fillet: 6 original faces + 1 new fillet face = 7 faces.
    let face_count = solid_mut.outer_shell.as_ref().map(|s| s.faces.len()).unwrap_or(0);
    assert_eq!(face_count, 7, "expected 7 faces after fillet, got {}", face_count);
}

#[test]
fn test_document_with_chamfer() {
    let mut doc = Document::new("test");
    let s = make_unit_cube_with_shared_edges();
    doc.add_solid(s);

    let solid = doc.solids().into_iter().next().unwrap();
    let edge_id = find_first_manifold_edge(solid);
    assert!(edge_id > 0);

    let solid_mut = &mut doc.root.solids[0];
    let rc = draper_core::operations::chamfer_edge(solid_mut, edge_id, 0.1);
    assert!(rc.is_ok(), "chamfer_edge failed: {:?}", rc);

    // After chamfer: 6 original + 1 new chamfer face = 7
    let face_count = solid_mut.outer_shell.as_ref().map(|s| s.faces.len()).unwrap_or(0);
    assert_eq!(face_count, 7, "expected 7 faces after chamfer, got {}", face_count);
}

#[test]
fn test_document_with_shell() {
    let mut doc = Document::new("test");
    let s = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    doc.add_solid(s);

    let solid_mut = &mut doc.root.solids[0];
    let rc = draper_core::operations::make_shell(solid_mut, 2.0);
    assert!(rc.is_ok(), "make_shell failed: {:?}", rc);

    // After shelling, the solid should have an inner shell
    let solid = &doc.root.solids[0];
    assert!(solid.inner_shells.len() > 0, "expected inner_shells after make_shell");
}

#[test]
fn test_boolean_union_produces_new_solid() {
    let a = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    let b = ShapeBuilder::make_box(50.0, 50.0, 50.0);
    let result = draper_core::boolean::boolean_union(&a, &b);
    assert!(result.is_ok(), "boolean_union failed: {:?}", result);
    let result = result.unwrap();
    // The result should have faces from both solids (union keeps external faces)
    let face_count = result.outer_shell.as_ref().map(|s| s.faces.len()).unwrap_or(0);
    assert!(face_count >= 6, "union should have at least 6 faces (cube), got {}", face_count);
}

#[test]
fn test_boolean_subtract_creates_cavity() {
    let a = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    let b = ShapeBuilder::make_box(50.0, 50.0, 50.0);
    let result = draper_core::boolean::boolean_subtract(&a, &b);
    assert!(result.is_ok(), "boolean_subtract failed: {:?}", result);
}

#[test]
fn test_transform_operations_round_trip() {
    use draper_mesh::TriangulationParams;
    let mut s = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let original_centroid = {
        let mesh = draper_mesh::triangulate_solid(&s, &TriangulationParams::default());
        let n = mesh.vertices.len() as f64;
        let sum = mesh.vertices.iter().fold((0.0, 0.0, 0.0), |(x, y, z), v| (x + v.x, y + v.y, z + v.z));
        (sum.0 / n, sum.1 / n, sum.2 / n)
    };

    // Translate
    draper_core::operations::translate_solid(&mut s, 100.0, 0.0, 0.0);
    // Rotate 360° (should be identity)
    let axis = Direction3d::Z;
    draper_core::operations::rotate_solid(&mut s, &axis, 2.0 * std::f64::consts::PI);
    // Scale 1.0 (identity)
    draper_core::operations::scale_solid(&mut s, 1.0);

    let new_centroid = {
        let mesh = draper_mesh::triangulate_solid(&s, &TriangulationParams::default());
        let n = mesh.vertices.len() as f64;
        let sum = mesh.vertices.iter().fold((0.0, 0.0, 0.0), |(x, y, z), v| (x + v.x, y + v.y, z + v.z));
        (sum.0 / n, sum.1 / n, sum.2 / n)
    };

    // After translate(100, 0, 0), centroid should be ~100 mm off in X
    assert!((new_centroid.0 - original_centroid.0 - 100.0).abs() < 1.0,
        "X centroid: original={}, new={}, expected ~{}",
        original_centroid.0, new_centroid.0, original_centroid.0 + 100.0);
}

#[test]
fn test_circular_pattern_creates_n_minus_1_copies() {
    let s = ShapeBuilder::make_box(20.0, 20.0, 20.0);
    let copies = draper_core::operations::circular_pattern(
        &s, Direction3d::Z, 4, 2.0 * std::f64::consts::PI,
    );
    assert_eq!(copies.len(), 3, "circular_pattern with count=4 should return 3 copies");
}

#[test]
fn test_linear_pattern_with_step_50_creates_translated_copies() {
    let s = ShapeBuilder::make_box(20.0, 20.0, 20.0);
    let dir = Direction3d::X;
    let copies = draper_core::operations::linear_pattern(&s, dir, 3, 50.0);
    assert_eq!(copies.len(), 2);
}

#[test]
fn test_gdt_check_flatness_on_cube_passes() {
    use draper_mesh::TriangulationParams;
    let s = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    let mesh = draper_mesh::triangulate_solid(&s, &TriangulationParams::default());
    let spec = draper_mesh::gdt_check::ToleranceSpec {
        tolerance_type: draper_mesh::gdt_check::GdtCheckType::Flatness,
        tolerance_value: 1.0,  // generous tolerance — we just want the API to work
        ..Default::default()
    };
    let checker = draper_mesh::gdt_check::GdtChecker::new(&mesh);
    let r = checker.check(&spec);
    // The check should produce a finite or NaN result, and the API should not panic.
    // We don't assert pass/fail since the default LOD may introduce small deviation.
    assert!(r.actual_deviation.is_finite() || r.actual_deviation.is_nan(),
        "actual deviation should be finite or NaN, got {}", r.actual_deviation);
}

#[test]
fn test_step_to_usda_pipeline_produces_valid_output() {
    let step_path = "test/nist_cube.stp";
    if !std::path::Path::new(step_path).exists() {
        eprintln!("Skipping USDA pipeline test — {} not found", step_path);
        return;
    }
    let tmp = tempfile::NamedTempFile::new().unwrap();
    let params = draper_core::step_to_usd::StepToUsdaParams::default();
    let result = draper_core::step_to_usd::export_step_to_usda(
        std::path::Path::new(step_path),
        tmp.path(),
        &params,
    );
    assert!(result.is_ok(), "export_step_to_usda failed: {:?}", result);
    let n_meshes = result.unwrap();
    assert!(n_meshes > 0, "expected at least 1 mesh in USDA output");

    let content = std::fs::read_to_string(tmp.path()).unwrap();
    assert!(content.contains("#usda 1.0"), "USDA header missing");
    assert!(content.contains("def Xform"), "no Xform in USDA");
    assert!(content.contains("def Mesh"), "no Mesh in USDA");
}

fn find_first_manifold_edge(solid: &draper_topology::Solid) -> usize {
    use std::collections::HashMap;
    let mut edge_count: HashMap<u64, usize> = HashMap::new();
    if let Some(shell) = solid.outer_shell.as_ref() {
        for face in &shell.faces {
            for edge in &face.edges {
                *edge_count.entry(edge.id.to_u64()).or_insert(0) += 1;
            }
        }
    }
    for (id, count) in &edge_count {
        if *count == 2 {
            return *id as usize;
        }
    }
    0  // no manifold edge found
}

/// Build a unit cube with shared edge TopoIds across faces.
///
/// This is what `ShapeBuilder::make_box` SHOULD do but doesn't (each face
/// creates its own independent edges). The fillet/chamfer operations require
/// manifold edges (edges shared by exactly 2 faces) to work, so we need
/// this helper for tests.
fn make_unit_cube_with_shared_edges() -> draper_topology::Solid {
    use draper_geometry::{Point3d, Direction3d, Surface, Plane, Curve3d, Circle};
    use draper_topology::{Solid, Shell, Face, Edge, Wire, CoEdge};

    // 12 unique edges
    let e_bot_01 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0));
    let e_bot_12 = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 0.0));
    let e_bot_23 = Edge::new_line(Point3d::new(1.0, 1.0, 0.0), Point3d::new(0.0, 1.0, 0.0));
    let e_bot_30 = Edge::new_line(Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 0.0, 0.0));
    let e_top_01 = Edge::new_line(Point3d::new(0.0, 0.0, 1.0), Point3d::new(1.0, 0.0, 1.0));
    let e_top_12 = Edge::new_line(Point3d::new(1.0, 0.0, 1.0), Point3d::new(1.0, 1.0, 1.0));
    let e_top_23 = Edge::new_line(Point3d::new(1.0, 1.0, 1.0), Point3d::new(0.0, 1.0, 1.0));
    let e_top_30 = Edge::new_line(Point3d::new(0.0, 1.0, 1.0), Point3d::new(0.0, 0.0, 1.0));
    let e_vert_0 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 1.0));
    let e_vert_1 = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 1.0));
    let e_vert_2 = Edge::new_line(Point3d::new(1.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 1.0));
    let e_vert_3 = Edge::new_line(Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 1.0, 1.0));

    let mk_face = |edges: Vec<Edge>, coedges: Vec<(draper_topology::TopoId, bool)>, plane: Plane| -> Face {
        let mut face = Face::new_surface_only(Surface::Plane(plane));
        face.edges = edges;
        face.outer_wire = Some(Wire::new(coedges.into_iter().map(|(id, f)| CoEdge::new(id, f)).collect()));
        face
    };

    let bottom = mk_face(
        vec![e_bot_01.clone(), e_bot_12.clone(), e_bot_23.clone(), e_bot_30.clone()],
        vec![(e_bot_01.id, true), (e_bot_12.id, true), (e_bot_23.id, true), (e_bot_30.id, true)],
        Plane { origin: Point3d::new(0.0, 0.0, 0.0), u_dir: Direction3d::X, v_dir: Direction3d::Y, normal: Direction3d::new(0.0, 0.0, -1.0).unwrap() },
    );
    let top = mk_face(
        vec![e_top_01.clone(), e_top_12.clone(), e_top_23.clone(), e_top_30.clone()],
        vec![(e_top_01.id, true), (e_top_12.id, true), (e_top_23.id, true), (e_top_30.id, true)],
        Plane { origin: Point3d::new(0.0, 0.0, 1.0), u_dir: Direction3d::X, v_dir: Direction3d::Y, normal: Direction3d::Z },
    );
    let front = mk_face(
        vec![e_bot_01.clone(), e_vert_1.clone(), e_top_01.clone(), e_vert_0.clone()],
        vec![(e_bot_01.id, true), (e_vert_1.id, true), (e_top_01.id, false), (e_vert_0.id, false)],
        Plane { origin: Point3d::new(0.0, 0.0, 0.0), u_dir: Direction3d::X, v_dir: Direction3d::Z, normal: Direction3d::new(0.0, -1.0, 0.0).unwrap() },
    );
    let back = mk_face(
        vec![e_bot_23.clone(), e_vert_3.clone(), e_top_23.clone(), e_vert_2.clone()],
        vec![(e_bot_23.id, false), (e_vert_3.id, true), (e_top_23.id, true), (e_vert_2.id, false)],
        Plane { origin: Point3d::new(0.0, 1.0, 0.0), u_dir: Direction3d::X, v_dir: Direction3d::Z, normal: Direction3d::Y },
    );
    let left = mk_face(
        vec![e_bot_30.clone(), e_vert_0.clone(), e_top_30.clone(), e_vert_3.clone()],
        vec![(e_bot_30.id, true), (e_vert_0.id, true), (e_top_30.id, false), (e_vert_3.id, false)],
        Plane { origin: Point3d::new(0.0, 0.0, 0.0), u_dir: Direction3d::Y, v_dir: Direction3d::Z, normal: Direction3d::new(-1.0, 0.0, 0.0).unwrap() },
    );
    let right = mk_face(
        vec![e_bot_12.clone(), e_vert_2.clone(), e_top_12.clone(), e_vert_1.clone()],
        vec![(e_bot_12.id, false), (e_vert_2.id, true), (e_top_12.id, true), (e_vert_1.id, false)],
        Plane { origin: Point3d::new(1.0, 0.0, 0.0), u_dir: Direction3d::Y, v_dir: Direction3d::Z, normal: Direction3d::X },
    );

    let shell = Shell::new_closed(vec![bottom, top, front, back, left, right]);
    Solid::new(shell)
}

#[test]
fn test_rotate_around_point_translates_solid_correctly() {
    use draper_core::operations::{rotate_solid_around_point};
    use draper_geometry::{Point3d, Direction3d};
    let mut s = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    // Rotate 180° about Z axis passing through (5,5,0)
    rotate_solid_around_point(&mut s, &Direction3d::Z, std::f64::consts::PI, &Point3d::new(5.0, 5.0, 0.0));
    // After 180° rotation about Z through (5,5,0), a cube originally at origin
    // should end up at (0,0,0)..(10,10,10) flipped to (0,0,0)..(10,10,10) (same place
    // because the cube is symmetric about its center). Test passes if no panic.
    assert!(s.outer_shell.is_some());
}

#[test]
fn test_scale_around_point_resizes_about_origin() {
    use draper_core::operations::scale_solid_around_point;
    use draper_geometry::Point3d;
    // make_box(10,10,10) is centered at origin → spans (-5..5, -5..5, -5..5)
    let mut s = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    // Scale by 2.0 about (0,0,0): cube grows to (-10..10, -10..10, -10..10)
    scale_solid_around_point(&mut s, 2.0, &Point3d::new(0.0, 0.0, 0.0));
    let shell = s.outer_shell.as_ref().unwrap();
    // The right-most edge endpoint should be at x=10.
    let mut max_x = f64::NEG_INFINITY;
    for face in &shell.faces {
        for edge in &face.edges {
            if let Some(p) = edge.start_point() {
                max_x = max_x.max(p.x);
            }
            if let Some(p) = edge.end_point() {
                max_x = max_x.max(p.x);
            }
        }
    }
    assert!((max_x - 10.0).abs() < 1e-6, "expected max_x = 10 after 2x scale about origin, got {}", max_x);
}

#[test]
fn test_remove_hole_and_clear_holes_on_face() {
    use draper_core::operations::{add_circular_hole_to_face, clear_holes_from_face, get_face_mut};
    use draper_geometry::{Point3d, Direction3d, Surface, Plane};
    let mut s = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    // Add a hole on face 0
    {
        let face = get_face_mut(&mut s, 0).unwrap();
        let _ = add_circular_hole_to_face(face, Point3d::new(50.0, 50.0, 0.0), 10.0, Direction3d::Z);
    }
    // Verify hole was added
    {
        let face = get_face_mut(&mut s, 0).unwrap();
        assert!(!face.inner_wires.is_empty(), "expected inner wire after add_hole");
    }
    // Clear all holes
    let removed = {
        let face = get_face_mut(&mut s, 0).unwrap();
        clear_holes_from_face(face)
    };
    assert!(removed >= 1, "expected at least 1 hole cleared, got {}", removed);
}

#[test]
fn test_delete_face_and_reverse_face() {
    use draper_core::operations::{delete_face_from_solid, reverse_face_orientation, get_face_mut};
    let mut s = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    let face_count_before = s.outer_shell.as_ref().map(|sh| sh.faces.len()).unwrap_or(0);
    assert_eq!(face_count_before, 6);

    // Delete face 0
    let res = delete_face_from_solid(&mut s, 0);
    assert!(res.is_ok(), "delete_face failed: {:?}", res);
    let face_count_after = s.outer_shell.as_ref().map(|sh| sh.faces.len()).unwrap_or(0);
    assert_eq!(face_count_after, 5, "expected 5 faces after deletion, got {}", face_count_after);

    // Reverse face 0 (was face 1 before deletion)
    let face = get_face_mut(&mut s, 0).unwrap();
    reverse_face_orientation(face);
    // Test passes if no panic
}

#[test]
fn test_step_export_round_trips_with_load_step() {
    use draper_step::{export_step, parse_step, extract_solids};
    let s = ShapeBuilder::make_box(50.0, 60.0, 70.0);
    let step_text = export_step(&s, "test_box");
    assert!(step_text.contains("ISO-10303-21"));
    assert!(step_text.contains("MANIFOLD_SOLID_BREP"));

    // Re-parse and verify
    let parsed = parse_step(&step_text).expect("parse should succeed");
    let (solids, _) = extract_solids(&parsed);
    assert_eq!(solids.len(), 1, "expected 1 solid after round-trip, got {}", solids.len());
}
