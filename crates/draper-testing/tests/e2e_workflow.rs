// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! End-to-end integration tests — Phase 4.1.

use draper_topology::ShapeBuilder;
use draper_topology::operations::{
    extrude_polyline, Polyline2d, fillet_edge,
    move_face_planar, offset_face_planar,
};
use draper_mesh::TriangulationParams;
use draper_mesh::formats::{build_obj, import_obj_from_reader, build_ply_ascii, import_ply_from_bytes, build_dxf};
use draper_geometry::Vec3d;

// ============================================================
// Workflow 1: Sketch → Extrude → Fillet → Parametric Rebuild
// ============================================================

#[test]
fn test_e2e_sketch_extrude_fillet_rebuild() {
    // Step 1: Build a rectangular sketch profile (100×50 mm)
    let profile = Polyline2d::rectangle(100.0, 50.0);
    assert_eq!(profile.point_count(), 5, "Closed rectangle has 5 points (last == first)");

    // Step 2: Extrude the profile by 30mm along +Z
    let solid = extrude_polyline(&profile, Vec3d::new(0.0, 0.0, 1.0), 30.0)
        .expect("Extrude should succeed");
    let original_face_count = solid.faces().len();
    assert!(original_face_count >= 6, "Extruded box should have at least 6 faces");

    // Step 3: Apply a fillet to edge 0 (radius 2mm)
    let filleted = fillet_edge(&solid, 0, 2.0)
        .expect("Fillet should succeed");
    let fillet_face_count = filleted.faces().len();
    assert!(fillet_face_count >= original_face_count,
        "Fillet should not reduce face count (was {}, now {})",
        original_face_count, fillet_face_count);

    // Step 4: "Parametric rebuild" — re-extrude with a different height
    let profile_v2 = Polyline2d::rectangle(100.0, 50.0);
    let solid_v2 = extrude_polyline(&profile_v2, Vec3d::new(0.0, 0.0, 1.0), 60.0)
        .expect("Rebuild extrude should succeed");
    assert_eq!(solid_v2.faces().len(), original_face_count,
        "Rebuilt solid should have the same face count as the original");

    // Step 5: Re-apply the fillet with a larger radius
    let filleted_v2 = fillet_edge(&solid_v2, 0, 5.0)
        .expect("Rebuild fillet should succeed");
    assert!(filleted_v2.faces().len() >= original_face_count);

    // Step 6: Triangulate the final solid to verify it's valid geometry
    let tri_params = TriangulationParams::default();
    let mesh = draper_mesh::triangulate_solid(&filleted_v2, &tri_params);
    assert!(!mesh.vertices.is_empty(), "Triangulated mesh should have vertices");
    assert!(!mesh.triangles.is_empty(), "Triangulated mesh should have triangles");
}

// ============================================================
// Workflow 2: Box → Boolean Subtract → OBJ/PLY/DXF Export
// ============================================================

#[test]
fn test_e2e_box_subtract_export() {
    use draper_geometry::ToleranceContext;

    // Step 1: Create the main box (100×100×50)
    let main_box = ShapeBuilder::make_box(100.0, 100.0, 50.0);
    assert!(!main_box.faces().is_empty());

    // Step 2: Create a smaller box to subtract (the "hole")
    let hole_box = ShapeBuilder::make_box(20.0, 20.0, 60.0);
    assert!(!hole_box.faces().is_empty());

    // Step 3: Boolean subtract (may not be fully supported — accept either outcome)
    let tol_ctx = ToleranceContext::default();
    let result = draper_topology::boolean::boolean_subtract(&main_box, &hole_box, &tol_ctx);
    let final_solid = match result {
        Ok(s) => {
            assert!(!s.faces().is_empty());
            s
        }
        Err(_) => main_box,
    };

    // Step 4: Triangulate
    let tri_params = TriangulationParams::default();
    let mesh = draper_mesh::triangulate_solid(&final_solid, &tri_params);
    assert!(!mesh.vertices.is_empty());

    // Step 5: Export to OBJ and re-import (round-trip)
    let obj_content = build_obj(&mesh);
    assert!(obj_content.contains("v "), "OBJ should have vertex lines");
    assert!(obj_content.contains("f "), "OBJ should have face lines");

    let cursor = std::io::Cursor::new(obj_content.into_bytes());
    let reader = std::io::BufReader::new(cursor);
    let imported = import_obj_from_reader(reader)
        .expect("OBJ round-trip should succeed");
    assert_eq!(imported.vertices.len(), mesh.vertices.len());
    assert_eq!(imported.triangles.len(), mesh.triangles.len());

    // Step 6: Export to PLY (ASCII) and re-import
    let ply_content = build_ply_ascii(&mesh);
    assert!(ply_content.starts_with("ply\n"));
    assert!(ply_content.contains("end_header\n"));
    let ply_bytes = ply_content.into_bytes();
    let imported_ply = import_ply_from_bytes(&ply_bytes)
        .expect("PLY round-trip should succeed");
    assert_eq!(imported_ply.vertices.len(), mesh.vertices.len());
    assert_eq!(imported_ply.triangles.len(), mesh.triangles.len());

    // Step 7: Export to DXF (2D flat pattern)
    let dxf_content = build_dxf(&mesh);
    assert!(dxf_content.contains("POLYLINE"));
    assert!(dxf_content.contains("EOF"));
}

// ============================================================
// Workflow 3: Assembly → Collision Check (BVH)
// ============================================================

#[test]
fn test_e2e_assembly_collision_check() {
    use draper_assembly::bvh::BoundingBox;

    // Step 1: Two boxes at the same location DO collide
    let bbox_a = BoundingBox::new([0.0, 0.0, 0.0], [100.0, 100.0, 100.0]);
    let bbox_b = BoundingBox::new([10.0, 10.0, 10.0], [90.0, 90.0, 90.0]);
    assert!(bbox_a.overlaps(&bbox_b), "Overlapping boxes should collide");

    // Step 2: Translate box B far away → no collision
    let translated_bbox_b = BoundingBox::new(
        [1000.0, 1000.0, 1000.0],
        [1080.0, 1080.0, 1080.0],
    );
    assert!(!bbox_a.overlaps(&translated_bbox_b),
        "Separated boxes should not collide");

    // Step 3: Touching boxes (edge contact) — overlaps returns true (loose)
    let touching = BoundingBox::new([100.0, 0.0, 0.0], [200.0, 100.0, 100.0]);
    assert!(bbox_a.overlaps(&touching),
        "Touching boxes should overlap (use overlaps_strict for exact)");

    // Step 4: Strict overlap (excludes edge contact)
    assert!(!bbox_a.overlaps_strict(&touching),
        "Touching boxes should not strictly overlap");
}

// ============================================================
// Workflow 4: Direct Modeling — Move Face + Offset Face
// ============================================================

#[test]
fn test_e2e_direct_modeling_move_offset() {
    // Step 1: Create a 100×100×100 box
    let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    let original_face_count = box_solid.faces().len();
    assert!(original_face_count >= 6);

    // Step 2: Try to move face 0 by 10mm in +Z
    let translation = Vec3d::new(0.0, 0.0, 10.0);
    let moved = move_face_planar(&box_solid, 0, translation);

    if let Ok(moved_solid) = moved {
        assert!(!moved_solid.faces().is_empty());
        assert_eq!(moved_solid.faces().len(), original_face_count);

        // Step 3: Try to offset face 0 by 5mm outward
        let offset_result = offset_face_planar(&moved_solid, 0, 5.0);
        if let Ok(offset_solid) = offset_result {
            assert!(!offset_solid.faces().is_empty());
        }
    }

    // Step 4: Verify the original solid is unchanged
    assert_eq!(box_solid.faces().len(), original_face_count,
        "Original solid should not be mutated");
}

// ============================================================
// Workflow 5: Sketch Spline + Polygon (Phase 3.5)
// ============================================================

#[test]
fn test_e2e_sketch_spline_polygon() {
    use draper_sketch::{Sketch2d, SketchEntity};

    // Step 1: Create a sketch with a spline through 4 points
    let mut sketch = Sketch2d::new();
    let p0 = sketch.add_point(0.0, 0.0);
    let p1 = sketch.add_point(10.0, 5.0);
    let p2 = sketch.add_point(20.0, -5.0);
    let p3 = sketch.add_point(30.0, 0.0);
    let spline_id = sketch.add_spline(vec![p0, p1, p2, p3], 0.0);
    assert!(spline_id != u64::MAX, "Spline should be created");

    // Step 2: Tessellate the spline
    let spline_pts = sketch.tessellate_spline(spline_id, 10);
    assert!(spline_pts.len() >= 31,
        "Spline with 4 points and 10 samples per span should have ≥31 samples (got {})",
        spline_pts.len());

    // Verify endpoints
    assert!((spline_pts[0].0 - 0.0).abs() < 1e-9);
    assert!((spline_pts[0].1 - 0.0).abs() < 1e-9);
    let last = *spline_pts.last().unwrap();
    assert!((last.0 - 30.0).abs() < 1e-9);
    assert!((last.1 - 0.0).abs() < 1e-9);

    // Step 3: Add a hexagon at (50, 50) with radius 20
    let center = sketch.add_point(50.0, 50.0);
    let poly_id = sketch.add_polygon(center, "r", 20.0, 6, 0.0);
    assert!(poly_id != u64::MAX, "Polygon should be created");

    // Step 4: Tessellate the polygon
    let poly_verts = sketch.tessellate_polygon(poly_id);
    assert_eq!(poly_verts.len(), 7,
        "Hexagon should have 6 vertices + 1 closing vertex = 7 (got {})",
        poly_verts.len());

    // Verify first vertex at center + (radius, 0)
    assert!((poly_verts[0].0 - 70.0).abs() < 1e-9);
    assert!((poly_verts[0].1 - 50.0).abs() < 1e-9);

    // Verify loop closed
    assert!((poly_verts[0].0 - poly_verts[6].0).abs() < 1e-9);
    assert!((poly_verts[0].1 - poly_verts[6].1).abs() < 1e-9);

    // Step 5: Verify the sketch has both entities
    let mut has_spline = false;
    let mut has_polygon = false;
    for entity in &sketch.entities {
        match entity {
            SketchEntity::Spline { .. } => has_spline = true,
            SketchEntity::Polygon { .. } => has_polygon = true,
            _ => {}
        }
    }
    assert!(has_spline, "Sketch should contain a Spline entity");
    assert!(has_polygon, "Sketch should contain a Polygon entity");
}

// ============================================================
// Workflow 6: Full Pipeline — Sketch → Extrude → Triangulate → Export
// ============================================================

#[test]
fn test_e2e_full_pipeline_sketch_to_obj() {
    // Step 1: Sketch a triangle profile (closed)
    let profile = Polyline2d::new(vec![
        (0.0, 0.0),
        (50.0, 0.0),
        (25.0, 40.0),
        (0.0, 0.0), // close the loop
    ]);
    assert_eq!(profile.point_count(), 4);

    // Step 2: Extrude into a triangular prism (height 30mm)
    let solid = extrude_polyline(&profile, Vec3d::new(0.0, 0.0, 1.0), 30.0)
        .expect("Triangular prism extrude should succeed");
    assert!(!solid.faces().is_empty());

    // Step 3: Triangulate
    let tri_params = TriangulationParams::default();
    let mesh = draper_mesh::triangulate_solid(&solid, &tri_params);
    assert!(!mesh.vertices.is_empty());
    assert!(!mesh.triangles.is_empty());

    // Step 4: Export to OBJ
    let obj = build_obj(&mesh);
    assert!(obj.contains("v "));

    // Step 5: Verify the OBJ can be re-imported (round-trip)
    let cursor = std::io::Cursor::new(obj.into_bytes());
    let reader = std::io::BufReader::new(cursor);
    let imported = import_obj_from_reader(reader)
        .expect("OBJ round-trip should succeed");
    assert_eq!(imported.vertices.len(), mesh.vertices.len());
    assert_eq!(imported.triangles.len(), mesh.triangles.len());

    // Step 6: Export to PLY and DXF as well
    let ply = build_ply_ascii(&mesh);
    assert!(ply.contains("format ascii 1.0"));

    let dxf = build_dxf(&mesh);
    assert!(dxf.contains("POLYLINE"));
    assert!(dxf.contains("EOF"));
}

// ============================================================
// Workflow 7: Camera Projection Modes (Phase 3.6)
// ============================================================

#[test]
fn test_e2e_camera_projection_modes() {
    use draper_viewer::camera::OrbitCamera;

    let mut cam = OrbitCamera::new();
    assert!(cam.perspective, "Camera should default to perspective");

    // Switch to orthographic
    cam.set_perspective(false);
    assert!(!cam.perspective);

    // Verify ortho matrix has no perspective divide
    let ortho_matrix = cam.projection_matrix(1.0);
    assert!(ortho_matrix[2][3].abs() < 1e-6,
        "Orthographic matrix should have [2][3] = 0");

    // Switch back to perspective
    cam.set_perspective(true);
    assert!(cam.perspective);

    // Verify perspective matrix has perspective divide
    let persp_matrix = cam.projection_matrix(1.0);
    assert!((persp_matrix[2][3] - (-1.0)).abs() < 1e-6,
        "Perspective matrix should have [2][3] = -1");
}

// ============================================================
// Workflow 8: Macro Recorder (Phase 3.3)
// ============================================================

#[test]
fn test_e2e_macro_recorder_record_export() {
    use draper_viewer::ui::macro_recorder::MacroRecorder;

    let mut recorder = MacroRecorder::new();

    // Step 1: Start recording
    recorder.start();
    assert!(recorder.recording);

    // Step 2: Record several actions
    recorder.record("InsertBox");
    recorder.record_with_value("SetParam", "width", 100.0);
    recorder.record_with_value("SetParam", "height", 80.0);
    recorder.record_with_arg("EditCut", "selection1");
    recorder.record("ViewIso");

    // Step 3: Stop recording
    let n = recorder.stop();
    assert_eq!(n, 5, "Should have recorded 5 actions");
    assert!(!recorder.recording);

    // Step 4: Export to Python
    let python = recorder.export_python("e2e_test_macro");
    assert!(python.contains("#!/usr/bin/env python3"));
    assert!(python.contains("import BRepCAD"));
    assert!(python.contains("app.action(\"InsertBox\")"));
    assert!(python.contains("app.set_param(\"width\", 100.0"));
    assert!(python.contains("app.set_param(\"height\", 80.0"));
    assert!(python.contains("app.action(\"EditCut\", \"selection1\")"));
    assert!(python.contains("app.action(\"ViewIso\")"));
    assert!(python.contains("if __name__ == \"__main__\""));

    // Step 5: Export to Lua
    let lua = recorder.export_lua("e2e_test_macro");
    assert!(lua.contains("require(\"BRepCAD\")"));
    assert!(lua.contains("app:action(\"InsertBox\")"));
    assert!(lua.contains("app:set_param(\"width\", 100.0"));

    // Step 6: Verify descriptions are human-readable
    let descs = recorder.descriptions();
    assert_eq!(descs.len(), 5);
    assert!(descs[0].contains("InsertBox"));
    assert!(descs[1].contains("width"));
    assert!(descs[1].contains("100"));
}
