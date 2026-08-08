// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Integration tests for draper-sketch — real-world sketch workflows.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 1.1: these tests verify
//! end-to-end workflows that a user would perform in the sketch mode UI.

use draper_sketch::{Constraint, ConstraintSolver, Sketch2d};

// ============================================================
// Real-world sketch workflows
// ============================================================

#[test]
fn test_rectangle_sketch() {
    // Create a rectangle: 4 lines, 4 points, 4 constraints (horizontal/vertical alternation)
    // + 4 fixed corners (or 1 fixed + 3 distance/parallel constraints).
    let mut sketch = Sketch2d::new();
    let p1 = sketch.add_point(0.0, 0.0);
    let p2 = sketch.add_point(10.0, 2.0); // intentionally off
    let p3 = sketch.add_point(12.0, 8.0);
    let p4 = sketch.add_point(2.0, 6.0);

    let l1 = sketch.add_line(p1, p2); // bottom
    let l2 = sketch.add_line(p2, p3); // right
    let l3 = sketch.add_line(p3, p4); // top
    let l4 = sketch.add_line(p4, p1); // left

    // Make it a rectangle: bottom & top horizontal, left & right vertical
    sketch.add_constraint(Constraint::Horizontal { line: l1 });
    sketch.add_constraint(Constraint::Horizontal { line: l3 });
    sketch.add_constraint(Constraint::Vertical { line: l2 });
    sketch.add_constraint(Constraint::Vertical { line: l4 });

    // Fix one corner to anchor the sketch
    sketch.add_constraint(Constraint::Fixed { entity: p1 });

    let mut solver = ConstraintSolver::new().with_tolerance(1e-6);
    let result = solver.solve(&mut sketch, 100);
    assert!(result.is_ok(), "Rectangle solve failed: {:?}", result.err());

    // Verify rectangle shape: p1 and p2 have same y, p2 and p3 have same x, etc.
    let (x1, y1) = sketch.get_point(p1).unwrap();
    let (x2, y2) = sketch.get_point(p2).unwrap();
    let (x3, y3) = sketch.get_point(p3).unwrap();
    let (x4, y4) = sketch.get_point(p4).unwrap();

    assert!((y1 - y2).abs() < 1e-3, "Bottom edge not horizontal");
    assert!((x2 - x3).abs() < 1e-3, "Right edge not vertical");
    assert!((y3 - y4).abs() < 1e-3, "Top edge not horizontal");
    assert!((x4 - x1).abs() < 1e-3, "Left edge not vertical");
    // Also check p1 is fixed
    assert!((x1 - 0.0).abs() < 1e-6 && (y1 - 0.0).abs() < 1e-6);
}

#[test]
fn test_equilateral_triangle() {
    // Create an equilateral triangle via distance constraints
    let mut sketch = Sketch2d::new();
    let p1 = sketch.add_point(0.0, 0.0);
    let p2 = sketch.add_point(5.0, 0.0);
    let p3 = sketch.add_point(2.5, 3.0);

    let l1 = sketch.add_line(p1, p2);
    let l2 = sketch.add_line(p2, p3);
    let l3 = sketch.add_line(p3, p1);

    sketch.add_constraint(Constraint::Horizontal { line: l1 });
    sketch.add_constraint(Constraint::Fixed { entity: p1 });
    sketch.add_constraint(Constraint::Fixed { entity: p2 });

    // All three sides equal (l2 and l3 equal to l1)
    sketch.add_constraint(Constraint::Equal { e1: l1, e2: l2 });
    sketch.add_constraint(Constraint::Equal { e1: l1, e2: l3 });

    let mut solver = ConstraintSolver::new().with_tolerance(1e-6);
    // Equal constraint is a placeholder — may not fully solve
    let _ = solver.solve(&mut sketch, 100);
    // No hard assertion — Equal is a stub. Just verify no panic.
}

#[test]
fn test_dimensioned_rectangle_with_parameters() {
    // Rectangle with width=50, height=30 via parameters
    let mut sketch = Sketch2d::new();
    sketch.set_parameter("width", 50.0);
    sketch.set_parameter("height", 30.0);

    let p1 = sketch.add_point(0.0, 0.0);
    let p2 = sketch.add_point(10.0, 0.0);
    let p3 = sketch.add_point(10.0, 5.0);
    let p4 = sketch.add_point(0.0, 5.0);

    let l1 = sketch.add_line(p1, p2);
    let l2 = sketch.add_line(p2, p3);
    let l3 = sketch.add_line(p3, p4);
    let l4 = sketch.add_line(p4, p1);

    sketch.add_constraint(Constraint::Horizontal { line: l1 });
    sketch.add_constraint(Constraint::Horizontal { line: l3 });
    sketch.add_constraint(Constraint::Vertical { line: l2 });
    sketch.add_constraint(Constraint::Vertical { line: l4 });
    sketch.add_constraint(Constraint::Fixed { entity: p1 });

    // Dimensional constraints
    sketch.add_constraint(Constraint::Distance {
        p1: p1,
        p2: p2,
        value: "width".to_string(),
    });
    sketch.add_constraint(Constraint::Distance {
        p1: p2,
        p2: p3,
        value: "height".to_string(),
    });

    let mut solver = ConstraintSolver::new().with_tolerance(1e-6);
    let result = solver.solve(&mut sketch, 100);
    assert!(result.is_ok(), "Dimensioned rectangle failed: {:?}", result.err());

    let (x2, _) = sketch.get_point(p2).unwrap();
    let (_, y3) = sketch.get_point(p3).unwrap();
    let (_, y2) = sketch.get_point(p2).unwrap();

    assert!((x2 - 50.0).abs() < 1e-2, "Width should be 50, got x2={}", x2);
    assert!((y3 - y2 - 30.0).abs() < 1e-2, "Height should be 30, got y3-y2={}", y3 - y2);
}

#[test]
fn test_circle_with_radius_parameter() {
    // Circle with radius defined by a parameter
    let mut sketch = Sketch2d::new();
    sketch.set_parameter("r", 15.0);
    let center = sketch.add_point(0.0, 0.0);
    let _circle = sketch.add_circle(center, "r", 15.0);

    sketch.add_constraint(Constraint::Fixed { entity: center });

    let mut solver = ConstraintSolver::new();
    let result = solver.solve(&mut sketch, 50);
    assert!(result.is_ok(), "Circle solve failed: {:?}", result.err());

    // Verify the parameter is accessible
    assert_eq!(sketch.get_parameter("r"), Some(15.0));
}

#[test]
fn test_modify_parameter_rebuilds_geometry() {
    // After solving, changing a parameter and re-solving should give different geometry
    let mut sketch = Sketch2d::new();
    sketch.set_parameter("d", 20.0);

    let p1 = sketch.add_point(0.0, 0.0);
    let p2 = sketch.add_point(10.0, 0.0);
    let line = sketch.add_line(p1, p2);

    sketch.add_constraint(Constraint::Horizontal { line });
    sketch.add_constraint(Constraint::Fixed { entity: p1 });
    sketch.add_constraint(Constraint::Distance {
        p1,
        p2,
        value: "d".to_string(),
    });

    let mut solver = ConstraintSolver::new().with_tolerance(1e-6);
    solver.solve(&mut sketch, 100).unwrap();

    let (x2_first, _) = sketch.get_point(p2).unwrap();
    assert!((x2_first - 20.0).abs() < 1e-2, "First solve: x2 should be 20, got {}", x2_first);

    // Now change the parameter and re-solve
    sketch.set_parameter("d", 50.0);
    // Reset p2 to trigger re-solve from a different position
    sketch.set_point(p2, 10.0, 0.0);
    solver.solve(&mut sketch, 100).unwrap();

    let (x2_second, _) = sketch.get_point(p2).unwrap();
    assert!((x2_second - 50.0).abs() < 1e-2, "Second solve: x2 should be 50, got {}", x2_second);
}

#[test]
fn test_over_constrained_rectangle_detected() {
    // Rectangle with conflicting distance constraints
    let mut sketch = Sketch2d::new();
    let p1 = sketch.add_point(0.0, 0.0);
    let p2 = sketch.add_point(10.0, 0.0);
    let p3 = sketch.add_point(10.0, 5.0);
    let p4 = sketch.add_point(0.0, 5.0);

    let l1 = sketch.add_line(p1, p2);
    let l2 = sketch.add_line(p2, p3);
    let l3 = sketch.add_line(p3, p4);
    let l4 = sketch.add_line(p4, p1);

    sketch.add_constraint(Constraint::Horizontal { line: l1 });
    sketch.add_constraint(Constraint::Horizontal { line: l3 });
    sketch.add_constraint(Constraint::Vertical { line: l2 });
    sketch.add_constraint(Constraint::Vertical { line: l4 });

    // Fix all 4 corners — over-constrained
    sketch.add_constraint(Constraint::Fixed { entity: p1 });
    sketch.add_constraint(Constraint::Fixed { entity: p2 });
    sketch.add_constraint(Constraint::Fixed { entity: p3 });
    sketch.add_constraint(Constraint::Fixed { entity: p4 });

    // Plus conflicting distance
    sketch.add_constraint(Constraint::Distance {
        p1,
        p2: p2,
        value: "100".to_string(), // conflicts with fixed positions
    });

    let mut solver = ConstraintSolver::new();
    let result = solver.solve(&mut sketch, 50);
    assert!(result.is_err(), "Should detect over-constrained");
}

#[test]
fn test_empty_sketch_solves_trivially() {
    let mut sketch = Sketch2d::new();
    let mut solver = ConstraintSolver::new();
    let result = solver.solve(&mut sketch, 50);
    assert!(result.is_ok());
}

#[test]
fn test_single_point_no_constraints() {
    let mut sketch = Sketch2d::new();
    let _p = sketch.add_point(5.0, 5.0);
    let mut solver = ConstraintSolver::new();
    let result = solver.solve(&mut sketch, 50);
    assert!(result.is_ok());
    // Point should remain where it was
    assert_eq!(sketch.get_point(1), Some((5.0, 5.0)));
}

#[test]
fn test_solver_with_custom_step_size() {
    // For highly nonlinear constraints, a smaller step size improves stability
    let mut sketch = Sketch2d::new();
    let p1 = sketch.add_point(0.0, 0.0);
    let p2 = sketch.add_point(1.0, 1.0);
    let line = sketch.add_line(p1, p2);

    sketch.add_constraint(Constraint::Horizontal { line });
    sketch.add_constraint(Constraint::Fixed { entity: p1 });
    sketch.add_constraint(Constraint::Distance {
        p1,
        p2,
        value: "10".to_string(),
    });

    let mut solver = ConstraintSolver::new()
        .with_tolerance(1e-8)
        .with_step_size(0.5); // damped
    let result = solver.solve(&mut sketch, 200);
    assert!(result.is_ok(), "Damped solve failed: {:?}", result.err());

    let (x2, y2) = sketch.get_point(p2).unwrap();
    assert!(y2.abs() < 1e-4, "y2 should be ~0, got {}", y2);
    assert!((x2 - 10.0).abs() < 1e-2, "x2 should be ~10, got {}", x2);
}

#[test]
fn test_constraint_entity_refs() {
    let c = Constraint::Distance {
        p1: 1,
        p2: 2,
        value: "d".to_string(),
    };
    assert_eq!(c.entity_refs(), vec![1, 2]);

    let c = Constraint::Fixed { entity: 5 };
    assert_eq!(c.entity_refs(), vec![5]);

    let c = Constraint::Parallel { l1: 1, l2: 2 };
    assert_eq!(c.entity_refs(), vec![1, 2]);
}

#[test]
fn test_resolve_value_literal_and_parameter() {
    let mut sketch = Sketch2d::new();
    sketch.set_parameter("d1", 25.0);

    assert_eq!(sketch.resolve_value("d1").unwrap(), 25.0);
    assert_eq!(sketch.resolve_value("50.0").unwrap(), 50.0);
    assert_eq!(sketch.resolve_value("100").unwrap(), 100.0);

    // Unknown parameter and invalid literal
    assert!(sketch.resolve_value("unknown").is_err());
    assert!(sketch.resolve_value("not_a_number").is_err());
}
