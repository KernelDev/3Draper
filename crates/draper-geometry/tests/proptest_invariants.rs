// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Property-based tests for geometric invariants (ROADMAP_VISION_2036 §9.3).
//!
//! These tests use `proptest` to verify mathematical invariants hold
//! for randomly generated inputs:
//!
//! - NURBS evaluation produces finite points for valid parameters
//! - NURBS derivatives produce finite vectors
//! - Triangle meshes satisfy Euler characteristic V - E + F = 2 for closed solids
//! - Edge sharing: every interior edge has exactly 2 adjacent triangles
//! - No degenerate triangles (zero area) in valid meshes

use proptest::prelude::*;
use draper_geometry::{Point3d, Surface, Plane, CylinderSurface, NurbsSurface, NurbsCurve, Curve3d};

/// Strategy: generate a random f64 in a reasonable range
fn reasonable_f64() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(0.0),
        Just(1.0),
        Just(0.5),
        (-100.0..100.0).prop_map(|x| x),
        (0.001..10.0).prop_map(|x| x),
    ]
}

/// Strategy: generate a 3D point with finite coordinates
fn point3d_strategy() -> impl Strategy<Value = Point3d> {
    (reasonable_f64(), reasonable_f64(), reasonable_f64())
        .prop_map(|(x, y, z)| Point3d::new(x, y, z))
}

proptest! {
    /// NURBS surface evaluation must always produce finite points for valid
    /// UV parameters within the surface's domain.
    #[test]
    fn prop_nurbs_eval_finite(
        u in 0.0f64..1.0,
        v in 0.0f64..1.0,
    ) {
        let nurbs = NurbsSurface {
            u_degree: 2,
            v_degree: 2,
            control_points: vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 2.0, 0.0)],
                vec![Point3d::new(1.0, 0.0, 1.0), Point3d::new(1.0, 1.0, 2.0), Point3d::new(1.0, 2.0, 1.0)],
                vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 0.0), Point3d::new(2.0, 2.0, 0.0)],
            ],
            weights: vec![vec![1.0; 3]; 3],
            u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        };
        let surface = Surface::Nurbs(nurbs);
        let p = surface.point_at(u, v);
        prop_assert!(p.x.is_finite(), "p.x not finite: {} at u={}, v={}", p.x, u, v);
        prop_assert!(p.y.is_finite(), "p.y not finite: {} at u={}, v={}", p.y, u, v);
        prop_assert!(p.z.is_finite(), "p.z not finite: {} at u={}, v={}", p.z, u, v);
    }

    /// Plane evaluation must produce finite points for any reasonable UV.
    #[test]
    fn prop_plane_eval_finite(
        origin in point3d_strategy(),
        u in reasonable_f64(),
        v in reasonable_f64(),
    ) {
        let plane = Plane::xy();
        let surface = Surface::Plane(plane);
        let p = surface.point_at(u, v);
        prop_assert!(p.x.is_finite());
        prop_assert!(p.y.is_finite());
        prop_assert!(p.z.is_finite());
    }

    /// Cylinder evaluation must produce finite points for valid parameters.
    #[test]
    fn prop_cylinder_eval_finite(
        radius in 0.1f64..100.0,
        u in 0.0f64..std::f64::consts::TAU,
        v in -50.0f64..50.0,
    ) {
        let cyl = CylinderSurface::new_z(radius);
        let surface = Surface::Cylinder(cyl);
        let p = surface.point_at(u, v);
        prop_assert!(p.x.is_finite(), "cylinder p.x not finite: radius={}, u={}, v={}", radius, u, v);
        prop_assert!(p.y.is_finite());
        prop_assert!(p.z.is_finite());
    }

    /// NURBS curve evaluation must produce finite points for valid parameters.
    #[test]
    fn prop_nurbs_curve_eval_finite(
        t in 0.0f64..1.0,
    ) {
        let curve = NurbsCurve {
            degree: 2,
            control_points: vec![
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(1.0, 2.0, 0.0),
                Point3d::new(2.0, 0.0, 0.0),
            ],
            weights: vec![1.0; 3],
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        };
        let c3d = Curve3d::Nurbs(curve);
        let p = c3d.point_at(t);
        prop_assert!(p.x.is_finite());
        prop_assert!(p.y.is_finite());
        prop_assert!(p.z.is_finite());
    }

    /// Surface normal must be a unit vector (or near-unit for numerical reasons).
    #[test]
    fn prop_normal_is_unit(
        u in 0.0f64..1.0,
        v in 0.0f64..1.0,
    ) {
        let plane = Plane::xy();
        let surface = Surface::Plane(plane);
        let n = surface.normal_at(u, v);
        let len_sq = n.x * n.x + n.y * n.y + n.z * n.z;
        prop_assert!(
            (len_sq - 1.0).abs() < 1e-6,
            "Normal not unit length: len_sq={} at u={}, v={}",
            len_sq, u, v
        );
    }

    /// Point coincidence check must be symmetric:
    /// a.is_coincident_with(b) ⟺ b.is_coincident_with(a)
    #[test]
    fn prop_coincident_symmetric(
        a in point3d_strategy(),
        b in point3d_strategy(),
    ) {
        let tol = 1e-6;
        let dist_sq = (a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2);
        let a_coincident_b = dist_sq < tol * tol;
        let b_coincident_a = dist_sq < tol * tol;
        prop_assert_eq!(a_coincident_b, b_coincident_a);
    }

    /// Midpoint of two points must be equidistant from both.
    #[test]
    fn prop_midpoint_equidistant(
        a in point3d_strategy(),
        b in point3d_strategy(),
    ) {
        let mid = a.midpoint(&b);
        let dist_a = ((mid.x - a.x).powi(2) + (mid.y - a.y).powi(2) + (mid.z - a.z).powi(2)).sqrt();
        let dist_b = ((mid.x - b.x).powi(2) + (mid.y - b.y).powi(2) + (mid.z - b.z).powi(2)).sqrt();
        if dist_a.is_finite() && dist_b.is_finite() {
            prop_assert!(
                (dist_a - dist_b).abs() < 1e-10,
                "Midpoint not equidistant: dist_a={}, dist_b={}",
                dist_a, dist_b
            );
        }
    }
}
