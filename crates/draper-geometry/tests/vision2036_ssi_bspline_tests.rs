// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Vision 2036 §2.1 — exact B-spline intersection output.
//!
//! Covers the three quality pillars of the §2.1 pipeline:
//! 1. True global least-squares fitting quality (vs. the old subsampling).
//! 2. Newton-Raphson refinement of the fitted curve on BOTH surfaces
//!    (marching noise is removed from the curve).
//! 3. Multi-branch coverage: every intersection branch gets its own
//!    B-spline curve via `intersect_surfaces`.

use draper_geometry::Curve3d;
use draper_geometry::Point3d;
use draper_geometry::NurbsCurve;
use draper_geometry::Surface;
use draper_geometry::Plane;
use draper_geometry::CylinderSurface;
use draper_geometry::TorusSurface;
use draper_geometry::Direction3d;
use draper_geometry::intersection::{intersect_surfaces, SurfaceSurfaceIntersection};

/// Max distance of `n_samples` uniform curve samples to the unit circle
/// (radial + out-of-plane deviation combined).
fn circle_deviation(curve: &NurbsCurve, n_samples: usize) -> f64 {
    let eval = Curve3d::Nurbs(curve.clone());
    let mut max_dev = 0.0_f64;
    for i in 0..n_samples {
        let t = i as f64 / (n_samples - 1) as f64;
        let p = eval.point_at(t);
        let radial = (p.x * p.x + p.y * p.y).sqrt();
        let dev = ((radial - 1.0).powi(2) + p.z * p.z).sqrt();
        if dev > max_dev {
            max_dev = dev;
        }
    }
    max_dev
}

/// Max distance of `n_samples` uniform curve samples to the line through
/// `a` and `b`.
fn line_deviation(curve: &NurbsCurve, a: &Point3d, b: &Point3d, n_samples: usize) -> f64 {
    let eval = Curve3d::Nurbs(curve.clone());
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let dz = b.z - a.z;
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-12 {
        return f64::MAX;
    }
    let (ux, uy, uz) = (dx / len, dy / len, dz / len);
    let mut max_dev = 0.0_f64;
    for i in 0..n_samples {
        let t = i as f64 / (n_samples - 1) as f64;
        let p = eval.point_at(t);
        let (ax, ay, az) = (p.x - a.x, p.y - a.y, p.z - a.z);
        // Cross product magnitude / len = point-to-line distance.
        let cx = ay * uz - az * uy;
        let cy = az * ux - ax * uz;
        let cz = ax * uy - ay * ux;
        let dev = (cx * cx + cy * cy + cz * cz).sqrt() / len;
        if dev > max_dev {
            max_dev = dev;
        }
    }
    max_dev
}

// ---------------------------------------------------------------
// 1. Least-squares fitting quality
// ---------------------------------------------------------------

#[test]
fn test_lsq_quarter_circle_quality() {
    // 100 points on the exact unit quarter arc — the LSQ fitter must
    // reproduce it far better than the legacy control-point subsampling.
    let n = 100;
    let pts: Vec<Point3d> = (0..n)
        .map(|i| {
            let a = std::f64::consts::FRAC_PI_2 * i as f64 / (n - 1) as f64;
            Point3d::new(a.cos(), a.sin(), 0.0)
        })
        .collect();
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![pts],
        b_spline_curve: None,
        b_spline_curves: Vec::new(),
        pcurves_a: Vec::new(),
        pcurves_b: Vec::new(),
    };
    let curve = ssi.try_fit_b_spline(1e-3).expect("quarter arc must LSQ-fit");
    let dev = circle_deviation(&curve, 200);
    assert!(
        dev < 1e-4,
        "LSQ quarter-arc deviation {:.3e} exceeds 1e-4",
        dev
    );
}

#[test]
fn test_lsq_line_exact() {
    // A straight line is exactly representable — deviation must be at
    // rounding level even under a tight tolerance gate.
    let n = 50;
    let pts: Vec<Point3d> = (0..n)
        .map(|i| {
            let s = i as f64;
            Point3d::new(0.5 * s, 2.0 - 0.25 * s, 1.0 + 0.1 * s)
        })
        .collect();
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![pts.clone()],
        b_spline_curve: None,
        b_spline_curves: Vec::new(),
        pcurves_a: Vec::new(),
        pcurves_b: Vec::new(),
    };
    let curve = ssi.try_fit_b_spline(1e-6).expect("line must LSQ-fit");
    let dev = line_deviation(&curve, &pts[0], &pts[n - 1], 100);
    assert!(dev < 1e-9, "LSQ line deviation {:.3e} exceeds 1e-9", dev);
}

// ---------------------------------------------------------------
// 2. Newton-Raphson refinement (§2.1 step 3)
// ---------------------------------------------------------------

#[test]
fn test_newton_refinement_improves_plane_cylinder() {
    // Plane z = 0 ∩ cylinder R = 1 (axis z) = unit circle. The "marching"
    // data carries deterministic sinusoidal noise (amplitude 1e-2); the
    // refinement must snap the fitted curve back onto the exact circle.
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::ORIGIN,
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));

    let n = 128;
    let noisy: Vec<Point3d> = (0..n)
        .map(|i| {
            let th = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
            let r = 1.0 + 0.01 * (3.0 * th + 0.7).sin();
            let z = 0.01 * (2.0 * th + 0.3).cos();
            Point3d::new(r * th.cos(), r * th.sin(), z)
        })
        .collect();
    let ssi = SurfaceSurfaceIntersection {
        polylines: vec![noisy],
        b_spline_curve: None,
        b_spline_curves: Vec::new(),
        pcurves_a: Vec::new(),
        pcurves_b: Vec::new(),
    };

    // BEFORE: pure LSQ fit (no refinement) — tracks the noisy data.
    let before = ssi.try_fit_b_spline(0.05).expect("noisy circle must LSQ-fit");
    let dev_before = circle_deviation(&before, 128);

    // AFTER: full §2.1 pipeline with Newton refinement on both surfaces.
    let curves = ssi.try_fit_b_splines_on_surfaces(&plane, &cyl, 0.05);
    assert_eq!(curves.len(), 1, "single branch must produce one curve");
    let dev_after = circle_deviation(&curves[0], 128);

    assert!(
        dev_after < dev_before / 2.0,
        "refinement must halve the deviation: before {:.3e}, after {:.3e}",
        dev_before,
        dev_after
    );
    assert!(
        dev_after < 5e-3,
        "refined deviation {:.3e} exceeds 5e-3",
        dev_after
    );
}

// ---------------------------------------------------------------
// 3. Multi-branch coverage through the public dispatcher
// ---------------------------------------------------------------

#[test]
fn test_multibranch_torus_plane_two_circles() {
    // Torus (R = 10, r = 2, axis z) ∩ plane z = 1 → two latitude circles
    // with rho = 10 ± sqrt(3). Both branches must be fitted.
    let torus = Surface::Torus(TorusSurface::new_z(Point3d::ORIGIN, 10.0, 2.0));
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 1.0),
        Direction3d::Z,
    ));
    let out = intersect_surfaces(&torus, &plane, 1e-6);
    assert_eq!(
        out.polylines.len(),
        2,
        "torus(R=10,r=2) ∩ plane z=1 must produce two latitude circles"
    );
    assert_eq!(
        out.b_splines().len(),
        2,
        "both branches must be B-spline-fitted"
    );
    assert!(out.b_spline().is_some(), "legacy primary accessor is set");

    // Match each fitted curve to its expected analytic latitude radius;
    // the two branches must pair up one-to-one with the two radii.
    let sqrt3 = 3.0_f64.sqrt();
    let mut expected = vec![10.0 + sqrt3, 10.0 - sqrt3];
    assert_eq!(expected.len(), out.b_splines().len());
    for curve in out.b_splines() {
        let eval = Curve3d::Nurbs(curve.clone());
        // Mean radial distance identifies the branch.
        let mut mean_radial = 0.0_f64;
        for i in 0..64 {
            let p = eval.point_at(i as f64 / 63.0);
            mean_radial += (p.x * p.x + p.y * p.y).sqrt();
        }
        mean_radial /= 64.0;
        // Closest expected radius (manual argmin — deterministic).
        let mut best_idx = 0usize;
        let mut best_err = f64::MAX;
        for (i, &rho) in expected.iter().enumerate() {
            let e = (mean_radial - rho).abs();
            if e < best_err {
                best_err = e;
                best_idx = i;
            }
        }
        let rho = expected.remove(best_idx);
        assert!(
            (mean_radial - rho).abs() < 1e-4,
            "branch radial {mean_radial:.6} does not match latitude radius {rho:.6}"
        );
        // Per-sample deviation: radial error and z = 1 exactly.
        let mut max_err = 0.0_f64;
        for i in 0..64 {
            let p = eval.point_at(i as f64 / 63.0);
            let radial = (p.x * p.x + p.y * p.y).sqrt();
            let err = ((radial - rho).powi(2) + (p.z - 1.0).powi(2)).sqrt();
            if err > max_err {
                max_err = err;
            }
        }
        assert!(
            max_err < 1e-4,
            "latitude circle deviation {:.3e} exceeds 1e-4",
            max_err
        );
    }
}

#[test]
fn test_intersect_surfaces_plane_cylinder_bspline_primary() {
    // End-to-end: the dispatcher itself must return a B-spline as the
    // primary result (§2.1 goal), not only polylines.
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::ORIGIN,
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-4);
    assert!(!out.polylines.is_empty(), "circle intersection expected");
    assert!(
        !out.b_splines().is_empty(),
        "§2.1: primary B-spline output must be available"
    );
    let dev = circle_deviation(&out.b_splines()[0], 128);
    assert!(dev < 5e-4, "primary curve deviation {:.3e} exceeds 5e-4", dev);
}
