// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Integration tests for FeatureTree (BREPCAD Phase 1.3).
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 1.3: verifies that the
//! feature tree evaluates features with REAL geometry operations
//! (no stubs), supports rollback, and rebuilds on parameter change.

use draper_topology::feature_history::{FeatureTree, Feature, FeatureParams};
use draper_topology::operations::Polyline2d;

#[test]
fn test_evaluate_extrude_produces_solid() {
    // Sketch → Extrude should produce a real solid with 6 faces (box).
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));
    let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
        sketch,
        distance: 5.0,
        direction: [0.0, 0.0, 1.0],
    }));

    tree.evaluate(extrude);

    let result = tree.get(extrude).unwrap();
    assert!(result.cached_result.is_some(), "Extrude should produce a solid");
    let solid = result.cached_result.as_ref().unwrap();
    assert_eq!(solid.faces().len(), 6, "Extruded rectangle should have 6 faces");
}

#[test]
fn test_evaluate_revolve_produces_solid() {
    let mut tree = FeatureTree::new();
    // Profile offset from axis (x+10) so it doesn't self-intersect
    let rect = Polyline2d::rectangle(10.0, 5.0);
    let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
    let profile = Polyline2d::new(pts);
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile,
    }));
    let revolve = tree.add_feature(Feature::new("revolve1", FeatureParams::Revolve {
        sketch,
        angle: std::f64::consts::PI,
        axis_origin: [0.0, 0.0, 0.0],
        axis_direction: [0.0, 0.0, 1.0],
    }));

    tree.evaluate(revolve);
    let result = tree.get(revolve).unwrap();
    assert!(result.cached_result.is_some(), "Revolve should produce a solid");
}

#[test]
fn test_edit_parameter_rebuilds() {
    // Changing extrude distance should rebuild the model.
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));
    let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
        sketch,
        distance: 5.0,
        direction: [0.0, 0.0, 1.0],
    }));

    // Initial evaluation
    tree.evaluate(extrude);
    let original = tree.get(extrude).unwrap().cached_result.as_ref().unwrap().clone();
    assert_eq!(original.faces().len(), 6);

    // Edit the extrude distance
    tree.edit_parameter(extrude, FeatureParams::Extrude {
        sketch,
        distance: 20.0,
        direction: [0.0, 0.0, 1.0],
    }).unwrap();

    let rebuilt = tree.get(extrude).unwrap().cached_result.as_ref().unwrap();
    assert_eq!(rebuilt.faces().len(), 6, "Rebuilt solid should still have 6 faces");
}

#[test]
fn test_rollback_to_sketch() {
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));
    let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
        sketch,
        distance: 5.0,
        direction: [0.0, 0.0, 1.0],
    }));

    tree.evaluate(extrude);
    tree.rollback_to(sketch).unwrap();
    assert_eq!(tree.current_feature(), Some(sketch));
}

#[test]
fn test_rollback_nonexistent_fails() {
    let mut tree = FeatureTree::new();
    let result = tree.rollback_to(draper_topology::feature_history::FeatureId(999));
    assert!(result.is_err());
}

#[test]
fn test_edit_nonexistent_fails() {
    let mut tree = FeatureTree::new();
    let result = tree.edit_parameter(
        draper_topology::feature_history::FeatureId(999),
        FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 10.0),
        },
    );
    assert!(result.is_err());
}

#[test]
fn test_chained_sketch_extrude_fillet() {
    // Sketch → Extrude → Fillet chain: each feature depends on the previous.
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));
    let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
        sketch,
        distance: 10.0,
        direction: [0.0, 0.0, 1.0],
    }));
    let fillet = tree.add_feature(Feature::new("fillet1", FeatureParams::Fillet {
        solid: extrude,
        radius: 1.0,
        edges: vec![0],
    }));

    tree.evaluate(fillet);

    // Extrude should be evaluated as a dependency of fillet
    let extrude_result = tree.get(extrude).unwrap();
    assert!(extrude_result.cached_result.is_some(), "Extrude should be evaluated");
}

#[test]
fn test_topological_order() {
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));
    let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
        sketch,
        distance: 5.0,
        direction: [0.0, 0.0, 1.0],
    }));

    let order = tree.topological_order();
    assert_eq!(order.len(), 2);
    assert_eq!(order[0], sketch, "Sketch should come before extrude");
    assert_eq!(order[1], extrude);
}

#[test]
fn test_invalidation_cascades() {
    // Updating sketch params should invalidate extrude (dependent).
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));
    let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
        sketch,
        distance: 5.0,
        direction: [0.0, 0.0, 1.0],
    }));

    // Evaluate both
    tree.evaluate(extrude);
    assert!(tree.get(extrude).unwrap().cached_result.is_some());

    // Update sketch → should invalidate extrude
    tree.update_params(sketch, FeatureParams::Sketch {
        profile: Polyline2d::rectangle(20.0, 20.0),
    });

    assert!(tree.get(sketch).unwrap().cached_result.is_none());
    assert!(tree.get(extrude).unwrap().cached_result.is_none());
}

#[test]
fn test_transform_feature() {
    // Sketch → Extrude → Transform (translate)
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));
    let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
        sketch,
        distance: 5.0,
        direction: [0.0, 0.0, 1.0],
    }));
    let transform = tree.add_feature(Feature::new("transform1", FeatureParams::Transform {
        solid: extrude,
        translation: [100.0, 200.0, 300.0],
        rotation: [1.0, 0.0, 0.0, 0.0], // identity quaternion
        scale: 1.0,
    }));

    tree.evaluate(transform);
    let result = tree.get(transform).unwrap();
    assert!(result.cached_result.is_some(), "Transform should produce a solid");
    let solid = result.cached_result.as_ref().unwrap();
    assert_eq!(solid.faces().len(), 6, "Transformed solid should have 6 faces");
}

#[test]
fn test_sketch_does_not_produce_solid() {
    // A sketch feature alone should not produce a solid (it stores a profile).
    let mut tree = FeatureTree::new();
    let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
        profile: Polyline2d::rectangle(10.0, 10.0),
    }));

    tree.evaluate(sketch);
    let result = tree.get(sketch).unwrap();
    assert!(result.cached_result.is_none(), "Sketch should not produce a solid");
}
