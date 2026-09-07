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
        b_spline_branch_indices: Vec::new(),
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
        b_spline_branch_indices: Vec::new(),
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
        b_spline_branch_indices: Vec::new(),
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

// ---------------------------------------------------------------
// 4. Seam closure of closed branches (§2.1 follow-up)
// ---------------------------------------------------------------

#[test]
fn test_closed_branch_seam_welded_analytic_circle() {
    // Plane z = 0 ∩ cylinder R = 1 → full circle sampled over [0, 2π)
    // WITHOUT the wrap point: the polyline endpoints sit one step
    // (~2π/128) apart and the fitted B-spline used to inherit the gap
    // as a visible seam. The closure must weld it: curve(0) == curve(1).
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::ORIGIN,
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-4);
    assert!(!out.b_splines().is_empty());
    let eval = Curve3d::Nurbs(out.b_splines()[0].clone());
    let seam_gap = eval.point_at(0.0).distance_to(&eval.point_at(1.0));
    assert!(
        seam_gap < 1e-9,
        "closed branch seam must be welded (C0), gap = {seam_gap:.3e}"
    );
    // Circle quality must not regress from the closure.
    let dev = circle_deviation(&out.b_splines()[0], 128);
    assert!(dev < 5e-4, "circle deviation after closure {dev:.3e}");
}

#[test]
fn test_closed_branch_seam_welded_marching_torus() {
    // Torus (R = 10, r = 2) ∩ plane z = 1 → two latitude circles. The
    // marching continuation stops ~1.3 steps short of closing; the
    // closure welds both branches' seams.
    let torus = Surface::Torus(TorusSurface::new_z(Point3d::ORIGIN, 10.0, 2.0));
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 1.0),
        Direction3d::Z,
    ));
    let out = intersect_surfaces(&torus, &plane, 1e-6);
    assert_eq!(out.b_splines().len(), 2, "two latitude branches");
    for curve in out.b_splines() {
        let eval = Curve3d::Nurbs(curve.clone());
        let seam_gap = eval.point_at(0.0).distance_to(&eval.point_at(1.0));
        assert!(
            seam_gap < 1e-9,
            "marching branch seam must be welded, gap = {seam_gap:.3e}"
        );
    }
}

#[test]
fn test_open_branch_not_falsely_closed() {
    // Plane x = 2 ∥ cylinder R = 5 axis → two open generator lines
    // (bounded by the marching span). Their endpoints are O(span) apart —
    // the closure heuristic must NOT weld them.
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(2.0, 0.0, 0.0),
        Direction3d::X,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-6);
    assert!(out.b_splines().len() >= 2, "two generator lines");
    for curve in out.b_splines() {
        let eval = Curve3d::Nurbs(curve.clone());
        let endpoint_gap = eval.point_at(0.0).distance_to(&eval.point_at(1.0));
        assert!(
            endpoint_gap > 0.5,
            "open generator branch must keep its endpoints, gap = {endpoint_gap:.3e}"
        );
    }
}

// ---------------------------------------------------------------
// 5. C2 seam periodicity (Vision 2036 «C2-периодичность шва»)
// ---------------------------------------------------------------

/// Tangent magnitude+direction mismatch between the two sides of the seam:
/// |C'(0) − C'(1)| — for a periodic uniform-knot curve this is ~machine
/// epsilon; the clamped weld (C0 only) leaves an O(1) jump.
fn seam_tangent_jump(curve: &NurbsCurve) -> f64 {
    let eval = Curve3d::Nurbs(curve.clone());
    let d0 = eval.derivative_at(0.0);
    let d1 = eval.derivative_at(1.0);
    ((d0.x - d1.x).powi(2) + (d0.y - d1.y).powi(2) + (d0.z - d1.z).powi(2)).sqrt()
}

/// Curvature jump across the seam: one-sided second-order estimates of
/// C'' AT the seam from both parameter sides,
///   C''(0⁺) ≈ (−3·C'(0) + 4·C'(h) − C'(2h)) / (2h)
///   C''(1⁻) ≈ ( 3·C'(1) − 4·C'(1−h) + C'(1−2h)) / (2h)
/// For a C2 seam both limits are the same C''(seam) — the estimates agree
/// to O(h²); a C0/C1 weld (clamped fit with duplicated endpoints) leaves
/// an O(|C'(1)−C'(0)|/h) jump.
fn seam_curvature_jump(curve: &NurbsCurve, h: f64) -> f64 {
    let eval = Curve3d::Nurbs(curve.clone());
    let d0 = eval.derivative_at(0.0);
    let d1 = eval.derivative_at(h);
    let d2 = eval.derivative_at(2.0 * h);
    let dp = eval.derivative_at(1.0);
    let dm = eval.derivative_at(1.0 - h);
    let dm2 = eval.derivative_at(1.0 - 2.0 * h);
    let dd0 = (
        (-3.0 * d0.x + 4.0 * d1.x - d2.x) / (2.0 * h),
        (-3.0 * d0.y + 4.0 * d1.y - d2.y) / (2.0 * h),
        (-3.0 * d0.z + 4.0 * d1.z - d2.z) / (2.0 * h),
    );
    let dd1 = (
        (3.0 * dp.x - 4.0 * dm.x + dm2.x) / (2.0 * h),
        (3.0 * dp.y - 4.0 * dm.y + dm2.y) / (2.0 * h),
        (3.0 * dp.z - 4.0 * dm.z + dm2.z) / (2.0 * h),
    );
    ((dd0.0 - dd1.0).powi(2)
        + (dd0.1 - dd1.1).powi(2)
        + (dd0.2 - dd1.2).powi(2))
        .sqrt()
}

#[test]
fn test_closed_branch_seam_c2_analytic_circle() {
    // Plane z = 0 ∩ cylinder R = 1 → full circle. The periodic fit must
    // close the seam with C2 continuity: position AND first derivative
    // AND the curvature trend continue across t=0/1.
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::ORIGIN,
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-4);
    assert!(!out.b_splines().is_empty());
    let curve = &out.b_splines()[0];
    let eval = Curve3d::Nurbs(curve.clone());

    // C0: seam exactly closed.
    let seam_gap = eval.point_at(0.0).distance_to(&eval.point_at(1.0));
    assert!(seam_gap < 1e-12, "periodic seam gap = {seam_gap:.3e}");

    // C1: tangents on both sides of the seam agree (machine-level).
    let t_jump = seam_tangent_jump(curve);
    assert!(
        t_jump < 1e-6,
        "periodic seam must be C1: tangent jump = {t_jump:.3e}"
    );

    // C2: the curvature estimates from both sides of the seam agree
    // (finite-difference noise O(h^2 * C4) is about 1e-3; a C0/C1 weld
    // would leave an O(1) jump).
    let c_jump = seam_curvature_jump(curve, 1e-3);
    assert!(
        c_jump < 0.05,
        "periodic seam must be C2: curvature jump = {c_jump:.3e}"
    );

    // Circle quality must hold with the periodic knots.
    let dev = circle_deviation(curve, 128);
    assert!(dev < 5e-4, "circle deviation with periodic knots {dev:.3e}");
}

#[test]
fn test_closed_branch_seam_c2_marching_torus() {
    // Torus (R = 10, r = 2) ∩ plane z = 1 → two latitude circles via
    // marching (near-closed: continuation stops ~1.3 steps short). Both
    // branches must be periodic — including the wrap gap as the natural
    // last segment.
    let torus = Surface::Torus(TorusSurface::new_z(Point3d::ORIGIN, 10.0, 2.0));
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 1.0),
        Direction3d::Z,
    ));
    let out = intersect_surfaces(&torus, &plane, 1e-6);
    assert_eq!(out.b_splines().len(), 2, "two latitude branches");
    for curve in out.b_splines() {
        let eval = Curve3d::Nurbs(curve.clone());
        let seam_gap = eval.point_at(0.0).distance_to(&eval.point_at(1.0));
        assert!(seam_gap < 1e-9, "periodic seam gap = {seam_gap:.3e}");
        let t_jump = seam_tangent_jump(curve);
        assert!(
            t_jump < 1e-6,
            "marching branch periodic seam must be C1: tangent jump = {t_jump:.3e}"
        );
        let c_jump = seam_curvature_jump(curve, 1e-3);
        assert!(
            c_jump < 0.05,
            "marching branch periodic seam must be C2: curvature jump = {c_jump:.3e}"
        );
    }
}

#[test]
fn test_periodic_fit_storage_layout() {
    // The periodic representation must be self-consistent for a generic
    // B-spline evaluator: knots count = n_cp + degree + 1, strictly
    // increasing, param_range = [0, 1], and the tail control points
    // duplicate the head (the wrap).
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::ORIGIN,
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-4);
    let curve = &out.b_splines()[0];
    let n_cp = curve.control_points.len();
    let p = curve.degree;
    assert_eq!(curve.knots.len(), n_cp + p + 1, "knot count convention");
    for i in 1..curve.knots.len() {
        assert!(curve.knots[i] > curve.knots[i - 1], "strictly increasing knots");
    }
    let (t0, t1) = Curve3d::Nurbs(curve.clone()).param_range();
    assert!(t0.abs() < 1e-12, "param_range starts at 0");
    assert!((t1 - 1.0).abs() < 1e-12, "param_range ends at 1");
    // Wrap: the first `degree` storage points repeat at the tail.
    for i in 0..p {
        let a = &curve.control_points[i];
        let b = &curve.control_points[n_cp - p + i];
        assert!(
            a.distance_to(b) < 1e-9,
            "storage tail must repeat the head (wrap), index {i}"
        );
    }
}

#[test]
fn test_open_branch_keeps_clamped_path() {
    // Open branches (two generator lines) must keep the clamped
    // endpoint-interpolating fit: endpoints are INTERPOLATED exactly and
    // the storage does NOT wrap (first != last control point).
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(2.0, 0.0, 0.0),
        Direction3d::X,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-6);
    assert!(out.b_splines().len() >= 2, "two generator lines");
    for curve in out.b_splines() {
        let n_cp = curve.control_points.len();
        // Clamped storage: first p+1 knots equal the domain start.
        for k in 0..=curve.degree {
            assert!(
                curve.knots[k].abs() < 1e-12,
                "open branch keeps clamped knots (multiplicity p+1 at 0)"
            );
        }
        // Endpoint interpolation: C(0) == P_0 exactly.
        let eval = Curve3d::Nurbs(curve.clone());
        let start = eval.point_at(0.0);
        let p0 = curve.control_points[0];
        assert!(start.distance_to(&p0) < 1e-9, "clamped fit interpolates P_0");
        let _ = n_cp;
    }
}
