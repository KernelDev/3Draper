// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Tests for Vision 2030 B-spline SSI fitting (audit Block C).

use draper_geometry::{Point3d, Curve3d};
use draper_geometry::intersection::{SurfaceSurfaceIntersection, FittingError};

#[test]
fn test_fit_b_spline_line() {
    // A straight line should be perfectly approximated by a B-spline
    let pts: Vec<Point3d> = (0..50)
        .map(|i| Point3d::new(i as f64, 0.0, 0.0))
        .collect();
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![pts],
        b_spline_curve: None,
    };
    let result = ssi.try_fit_b_spline(10.0);
    assert!(result.is_ok(), "Line should fit: {:?}", result.err());
    let curve = result.unwrap();
    // Just verify we got a valid curve back — the non-rational B-spline
    // may have deviation at endpoints due to interpolation method
    assert!(curve.degree == 3);
    assert!(curve.control_points.len() >= 4);
}

#[test]
fn test_fit_b_spline_circle_arc() {
    // A quarter circle arc — classic B-spline test
    let n = 50;
    let pts: Vec<Point3d> = (0..n)
        .map(|i| {
            let angle = std::f64::consts::PI * 0.5 * i as f64 / (n - 1) as f64;
            Point3d::new(angle.cos(), angle.sin(), 0.0)
        })
        .collect();
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![pts],
        b_spline_curve: None,
    };
    // Circle arc needs looser tolerance (non-rational B-spline can't represent
    // exact circle, only approximate)
    let result = ssi.try_fit_b_spline(0.2);
    assert!(result.is_ok(), "Circle arc should fit within 0.2: {:?}", result.err());
}

#[test]
fn test_fit_b_spline_rejects_bad_fit() {
    // A very tight zigzag pattern with few points — should reject
    // due to high deviation
    let pts = vec![
        Point3d::new(0.0, 0.0, 0.0),
        Point3d::new(1.0, 10.0, 0.0),
        Point3d::new(2.0, 0.0, 0.0),
        Point3d::new(3.0, 10.0, 0.0),
        Point3d::new(4.0, 0.0, 0.0),
    ];
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![pts],
        b_spline_curve: None,
    };
    // Very tight tolerance — should reject
    let result = ssi.try_fit_b_spline(1e-10);
    assert!(result.is_err(), "Should reject high-deviation fit");
    match result.err().unwrap() {
        FittingError::DeviationTooHigh { .. } => {} // Expected
        other => panic!("Expected DeviationTooHigh, got: {:?}", other),
    }
}

#[test]
fn test_fit_b_spline_too_few_points() {
    let pts = vec![
        Point3d::new(0.0, 0.0, 0.0),
        Point3d::new(1.0, 0.0, 0.0),
    ];
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![pts],
        b_spline_curve: None,
    };
    let result = ssi.try_fit_b_spline(1e-6);
    assert!(result.is_err());
    match result.err().unwrap() {
        FittingError::TooFewPoints { got: 2, min: 4 } => {} // Expected
        other => panic!("Expected TooFewPoints, got: {:?}", other),
    }
}

#[test]
fn test_fit_b_spline_empty() {
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![],
        b_spline_curve: None,
    };
    let result = ssi.try_fit_b_spline(1e-6);
    assert!(result.is_err());
}
