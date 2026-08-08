// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Integration tests for extrude/revolve operations (BREPCAD Phase 1.2).

use draper_topology::operations::{extrude_polyline, revolve_polyline, Polyline2d, ModelingError};
use draper_geometry::Vec3d;
use std::f64::consts::PI;

fn count_faces(solid: &draper_topology::entity::Solid) -> usize {
    solid.faces().len()
}

#[test]
fn test_extrude_rectangle_creates_box() {
    let rect = Polyline2d::rectangle(10.0, 20.0);
    let solid = extrude_polyline(&rect, Vec3d::new(0.0, 0.0, 1.0), 5.0).unwrap();
    assert_eq!(count_faces(&solid), 6);
}

#[test]
fn test_extrude_circle_creates_cylinder() {
    let circ = Polyline2d::circle(5.0, 16);
    let solid = extrude_polyline(&circ, Vec3d::new(0.0, 0.0, 1.0), 10.0).unwrap();
    assert_eq!(count_faces(&solid), 18);
}

#[test]
fn test_extrude_open_wire_fails() {
    let open = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
    let result = extrude_polyline(&open, Vec3d::new(0.0, 0.0, 1.0), 1.0);
    assert!(matches!(result, Err(ModelingError::OpenWire)));
}

#[test]
fn test_extrude_too_few_points_fails() {
    let too_few = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0)]);
    let result = extrude_polyline(&too_few, Vec3d::new(0.0, 0.0, 1.0), 1.0);
    assert!(matches!(result, Err(ModelingError::TooFewPoints(_))));
}

#[test]
fn test_extrude_zero_direction_fails() {
    let rect = Polyline2d::rectangle(10.0, 10.0);
    let result = extrude_polyline(&rect, Vec3d::new(0.0, 0.0, 0.0), 1.0);
    assert!(matches!(result, Err(ModelingError::ZeroDirection)));
}

#[test]
fn test_extrude_in_z_direction_negative() {
    // Extrude in -Z direction (downward)
    let rect = Polyline2d::rectangle(10.0, 10.0);
    let solid = extrude_polyline(&rect, Vec3d::new(0.0, 0.0, -1.0), 5.0).unwrap();
    assert_eq!(count_faces(&solid), 6);
}

#[test]
fn test_revolve_full_circle() {
    let rect = Polyline2d::rectangle(10.0, 5.0);
    let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
    let profile = Polyline2d::new(pts);
    let solid = revolve_polyline(&profile, 2.0 * PI).unwrap();
    // 24 segments × 4 edges = 96 side faces, 0 caps
    assert_eq!(count_faces(&solid), 96);
}

#[test]
fn test_revolve_partial_angle() {
    let rect = Polyline2d::rectangle(10.0, 5.0);
    let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
    let profile = Polyline2d::new(pts);
    let solid = revolve_polyline(&profile, PI).unwrap();
    // 12 segments × 4 + 2 caps = 50
    assert_eq!(count_faces(&solid), 50);
}

#[test]
fn test_revolve_quarter_angle() {
    let rect = Polyline2d::rectangle(10.0, 5.0);
    let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
    let profile = Polyline2d::new(pts);
    let solid = revolve_polyline(&profile, PI / 2.0).unwrap();
    // 90° → ceil(90/15)=6, but max(8)=8 segments → 8×4=32 + 2 caps = 34
    assert_eq!(count_faces(&solid), 34);
}

#[test]
fn test_revolve_invalid_angle_zero() {
    let rect = Polyline2d::rectangle(10.0, 5.0);
    let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
    let profile = Polyline2d::new(pts);
    assert!(matches!(revolve_polyline(&profile, 0.0), Err(ModelingError::InvalidAngle(_))));
}

#[test]
fn test_revolve_invalid_angle_negative() {
    let rect = Polyline2d::rectangle(10.0, 5.0);
    let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
    let profile = Polyline2d::new(pts);
    assert!(matches!(revolve_polyline(&profile, -1.0), Err(ModelingError::InvalidAngle(_))));
}

#[test]
fn test_revolve_invalid_angle_too_large() {
    let rect = Polyline2d::rectangle(10.0, 5.0);
    let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
    let profile = Polyline2d::new(pts);
    assert!(matches!(revolve_polyline(&profile, 7.0), Err(ModelingError::InvalidAngle(_))));
}

#[test]
fn test_polyline_is_closed_detection() {
    let closed = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]);
    assert!(closed.is_closed());
    assert_eq!(closed.point_count(), 3);

    let open = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
    assert!(!open.is_closed());
}

#[test]
fn test_polyline_rectangle_constructor() {
    let rect = Polyline2d::rectangle(10.0, 20.0);
    assert!(rect.is_closed());
    assert_eq!(rect.point_count(), 4);
    assert_eq!(rect.points[0], (-5.0, -10.0));
    assert_eq!(rect.points[2], (5.0, 10.0));
}

#[test]
fn test_polyline_circle_constructor() {
    let circ = Polyline2d::circle(5.0, 8);
    assert!(circ.is_closed());
    assert_eq!(circ.point_count(), 8);
    assert!((circ.points[0].0 - 5.0).abs() < 1e-10);
}

#[test]
fn test_extrude_triangle() {
    let tri = Polyline2d::new(vec![
        (0.0, 0.0),
        (10.0, 0.0),
        (5.0, 8.0),
        (0.0, 0.0),
    ]);
    let solid = extrude_polyline(&tri, Vec3d::new(0.0, 0.0, 1.0), 5.0).unwrap();
    assert_eq!(count_faces(&solid), 5);
}

#[test]
fn test_extrude_hexagon() {
    let mut pts = Vec::new();
    for i in 0..6 {
        let angle = 2.0 * PI * i as f64 / 6.0;
        pts.push((5.0 * angle.cos(), 5.0 * angle.sin()));
    }
    pts.push(pts[0]);
    let hex = Polyline2d::new(pts);
    let solid = extrude_polyline(&hex, Vec3d::new(0.0, 0.0, 1.0), 3.0).unwrap();
    assert_eq!(count_faces(&solid), 8);
}
