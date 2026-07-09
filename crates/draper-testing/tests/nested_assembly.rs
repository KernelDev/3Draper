// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.6.2 — Nested subassembly transform tests.
//!
//! Tests that the converter correctly composes transforms for multi-level
//! NAUO assemblies (3+ levels of nesting).
//!
//! Uses `test/nested_assembly.stp` — a synthetic 3-level assembly:
//!   RootAssembly → SubAssembly (translate +10 X) → LeafPart (translate +20 Y)
//!
//! Expected composed transform for LeafPart: translation (10, 20, 0).

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};

/// Helper: extract the translation component from a 4×4 transform matrix.
/// Returns (tx, ty, tz) from the last column of the matrix.
fn extract_translation(tf: &[[f64; 4]; 4]) -> (f64, f64, f64) {
    (tf[0][3], tf[1][3], tf[2][3])
}

/// Helper: check that a 4×4 matrix is approximately a pure translation.
fn is_pure_translation(tf: &[[f64; 4]; 4], tol: f64) -> bool {
    // Check that the 3×3 rotation part is identity
    for i in 0..3 {
        for j in 0..3 {
            let expected = if i == j { 1.0 } else { 0.0 };
            if (tf[i][j] - expected).abs() > tol {
                return false;
            }
        }
    }
    // Check that bottom row is [0, 0, 0, 1]
    tf[3][0].abs() < tol && tf[3][1].abs() < tol && tf[3][2].abs() < tol
        && (tf[3][3] - 1.0).abs() < tol
}

fn load_nested_assembly() -> Option<(draper_step::StepFile, draper_step::AssemblyNode, Vec<draper_step::PendingBrepInstance>)> {
    let project_root = std::env::var("CARGO_MANIFEST_DIR")
        .map(|d| format!("{}/../..", d))
        .unwrap_or(".".to_string());
    let path = format!("{}/test/nested_assembly.stp", project_root);
    let content = std::fs::read_to_string(&path).ok()?;
    let step_file = parse_step(&content).ok()?;
    let (tree, pending) = step_structure_lazy(&step_file);
    Some((step_file, tree, pending))
}

/// Load nested_assembly.stp and verify the assembly structure.
#[test]
fn test_nested_assembly_structure() {
    let (_, tree, pending) = match load_nested_assembly() {
        Some(v) => v,
        None => {
            eprintln!("Skipping: nested_assembly.stp not found");
            return;
        }
    };

    // Should have at least 1 pending instance (the leaf cube)
    assert!(!pending.is_empty(), "Expected at least 1 pending instance, got 0");

    println!("Root node: '{}' with {} children", tree.name, tree.children.len());

    // Root should have 1 child (SubAssembly)
    assert_eq!(tree.children.len(), 1,
        "Root should have 1 child (SubAssembly), got {}", tree.children.len());
    let sub_node = &tree.children[0];
    println!("  SubAssembly node: '{}' with {} children", sub_node.name, sub_node.children.len());

    // SubAssembly should have 1 child (LeafPart)
    assert_eq!(sub_node.children.len(), 1,
        "SubAssembly should have 1 child (LeafPart), got {}", sub_node.children.len());
    let leaf_node = &sub_node.children[0];
    println!("    LeafPart node: '{}'", leaf_node.name);
}

/// Verify that the composed transform for the leaf part in a 3-level assembly
/// is correct: Root (identity) × SubAssembly (+10 X) × LeafPart (+20 Y)
/// = translation (10, 20, 0).
#[test]
fn test_nested_assembly_composed_transform() {
    let (_, _, pending) = match load_nested_assembly() {
        Some(v) => v,
        None => {
            eprintln!("Skipping: nested_assembly.stp not found");
            return;
        }
    };

    // There should be exactly 1 leaf instance (the cube)
    assert!(!pending.is_empty(), "Expected at least 1 pending instance");

    // Find the leaf instance — it should have a composed transform
    let leaf = &pending[0];
    println!("Leaf instance: name='{}', brep_id={}, transform={:?}",
        leaf.name, leaf.brep_id, leaf.transform);

    // The leaf must have a composed transform
    assert!(leaf.transform.is_some(),
        "Leaf instance '{}' should have a composed transform", leaf.name);

    let tf = leaf.transform.unwrap();

    // Verify it's a pure translation
    assert!(is_pure_translation(&tf, 1e-6),
        "Composed transform should be a pure translation, got: {:?}", tf);

    // Verify the translation is (10, 20, 0)
    let (tx, ty, tz) = extract_translation(&tf);
    let tol = 1e-4;
    assert!((tx - 10.0).abs() < tol,
        "Expected tx=10.0, got {}", tx);
    assert!((ty - 20.0).abs() < tol,
        "Expected ty=20.0, got {}", ty);
    assert!(tz.abs() < tol,
        "Expected tz=0.0, got {}", tz);

    println!("Composed transform verified: translation ({}, {}, {})", tx, ty, tz);
}

/// Verify that triangulation produces the leaf cube at the correct world position.
#[test]
fn test_nested_assembly_triangulation_position() {
    let (step_file, _, pending) = match load_nested_assembly() {
        Some(v) => v,
        None => {
            eprintln!("Skipping: nested_assembly.stp not found");
            return;
        }
    };

    let mut ctx = OwnedStepConversionContext::new(step_file);
    let mut all_vertices_min = (f64::MAX, f64::MAX, f64::MAX);
    let mut all_vertices_max = (f64::MIN, f64::MIN, f64::MIN);

    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let mesh = &inst.mesh;
            assert!(mesh.triangle_count() > 0, "Leaf instance should produce triangles");

            for v in &mesh.vertices {
                all_vertices_min.0 = all_vertices_min.0.min(v.x);
                all_vertices_min.1 = all_vertices_min.1.min(v.y);
                all_vertices_min.2 = all_vertices_min.2.min(v.z);
                all_vertices_max.0 = all_vertices_max.0.max(v.x);
                all_vertices_max.1 = all_vertices_max.1.max(v.y);
                all_vertices_max.2 = all_vertices_max.2.max(v.z);
            }
        }
    }

    // The cube's local bbox is (0,0,0)-(5,5,5).
    // With composed transform (+10 X, +20 Y), world bbox should be (10,20,0)-(15,25,5).
    let tol = 0.1;
    println!("World bbox: ({:.1},{:.1},{:.1}) to ({:.1},{:.1},{:.1})",
        all_vertices_min.0, all_vertices_min.1, all_vertices_min.2,
        all_vertices_max.0, all_vertices_max.1, all_vertices_max.2);

    assert!((all_vertices_min.0 - 10.0).abs() < tol,
        "Expected min_x ≈ 10.0, got {:.2}", all_vertices_min.0);
    assert!((all_vertices_min.1 - 20.0).abs() < tol,
        "Expected min_y ≈ 20.0, got {:.2}", all_vertices_min.1);
    assert!((all_vertices_min.2 - 0.0).abs() < tol,
        "Expected min_z ≈ 0.0, got {:.2}", all_vertices_min.2);
    assert!((all_vertices_max.0 - 15.0).abs() < tol,
        "Expected max_x ≈ 15.0, got {:.2}", all_vertices_max.0);
    assert!((all_vertices_max.1 - 25.0).abs() < tol,
        "Expected max_y ≈ 25.0, got {:.2}", all_vertices_max.1);
    assert!((all_vertices_max.2 - 5.0).abs() < tol,
        "Expected max_z ≈ 5.0, got {:.2}", all_vertices_max.2);
}
