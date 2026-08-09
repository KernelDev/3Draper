// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Tests for Vision 2030 SDF primitives (audit Block C).

use draper_implicit::{Sdf, SphereSdf, BoxSdf, GyroidSdf, CsgNode, ImplicitSolid};
use draper_geometry::Point3d;

#[test]
fn test_gyroid_periodicity() {
    let g = GyroidSdf::new(1.0, 0.1, [50.0, 50.0, 50.0]);
    let p1 = Point3d::new(0.5, 0.3, 0.7);
    let p2 = Point3d::new(0.5 + 2.0 * std::f64::consts::PI, 0.3, 0.7);
    let d1 = g.signed_distance(&p1);
    let d2 = g.signed_distance(&p2);
    assert!(
        (d1 - d2).abs() < 1e-8,
        "Gyroid periodicity: d1={}, d2={}, diff={}", d1, d2, (d1 - d2).abs()
    );
}

#[test]
fn test_gyroid_bounding_box_clip() {
    let g = GyroidSdf::new(1.0, 0.1, [5.0, 5.0, 5.0]);
    let outside = Point3d::new(100.0, 100.0, 100.0);
    let d = g.signed_distance(&outside);
    assert!(d > 0.0, "Point outside bbox should be positive: {}", d);
}

#[test]
fn test_gyroid_zero_at_surface() {
    let g = GyroidSdf::new(1.0, 0.0, [50.0, 50.0, 50.0]);
    let d = g.signed_distance(&Point3d::new(0.0, 0.0, 0.0));
    assert!(d.abs() < 0.1, "Origin should be near surface: d={}", d);
}

#[test]
fn test_csg_union_two_spheres() {
    let a = ImplicitSolid::sphere(Point3d::new(-0.5, 0.0, 0.0), 1.0);
    let b = ImplicitSolid::sphere(Point3d::new(0.5, 0.0, 0.0), 1.0);
    let union = a.union(b);

    let da = SphereSdf { center: Point3d::new(-0.5, 0.0, 0.0), radius: 1.0 }.signed_distance(&Point3d::ORIGIN);
    let db = SphereSdf { center: Point3d::new(0.5, 0.0, 0.0), radius: 1.0 }.signed_distance(&Point3d::ORIGIN);
    assert!(da < 0.0 && db < 0.0, "Both spheres contain origin");

    let du = union.signed_distance(&Point3d::ORIGIN);
    assert!(du < 0.0, "Union should contain origin: du={}", du);
    assert!((du - da.min(db)).abs() < 1e-10, "Union = min(da, db)");
}

#[test]
fn test_csg_subtract_sphere_from_box() {
    let box_solid = ImplicitSolid::box_solid(Point3d::ORIGIN, 1.0, 1.0, 1.0);
    let sphere = ImplicitSolid::sphere(Point3d::ORIGIN, 0.5);
    let result = box_solid.subtract(sphere);

    let d = result.signed_distance(&Point3d::ORIGIN);
    assert!(d > 0.0, "Center should be outside after subtraction: d={}", d);
}

#[test]
fn test_csg_intersect_two_boxes() {
    let a = ImplicitSolid::box_solid(Point3d::ORIGIN, 1.0, 1.0, 1.0);
    let b = ImplicitSolid::box_solid(Point3d::new(0.5, 0.0, 0.0), 1.0, 1.0, 1.0);
    let result = a.intersect(b);

    let d = result.signed_distance(&Point3d::new(0.5, 0.0, 0.0));
    assert!(d < 0.0, "Intersection center should be inside: d={}", d);

    let d2 = result.signed_distance(&Point3d::new(1.5, 0.0, 0.0));
    assert!(d2 > 0.0, "Outside A should be outside intersection: d={}", d2);
}

#[test]
fn test_csg_bounding_box_union() {
    let a = ImplicitSolid::sphere(Point3d::new(0.0, 0.0, 0.0), 1.0);
    let b = ImplicitSolid::sphere(Point3d::new(5.0, 0.0, 0.0), 1.0);
    let union = a.union(b);
    let (bmin, bmax) = union.bounding_box();
    assert!((bmin.x - (-1.0)).abs() < 1e-10);
    assert!((bmax.x - 6.0).abs() < 1e-10);
}
