// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Vision 2036 §2.2 — analytical PCURVE tests.
//!
//! Covers the three pillars of the §2.2 pipeline:
//! 1. Plane×cylinder analytical derivation via parametric substitution
//!    (tilted / perpendicular / parallel-to-axis configurations).
//! 2. Newton-Raphson inversion of the 3D B-spline branch onto a NURBS
//!    surface's UV space (generic projection path).
//! 3. Multi-branch coverage and positional correspondence between
//!    `b_spline_curves` and `pcurves_a`/`pcurves_b`.

use draper_geometry::Curve2d;
use draper_geometry::Curve3d;
use draper_geometry::CylinderSurface;
use draper_geometry::Direction3d;
use draper_geometry::NurbsSurface;
use draper_geometry::Plane;
use draper_geometry::Point3d;
use draper_geometry::Surface;
use draper_geometry::TorusSurface;
use draper_geometry::intersection::intersect_surfaces;

/// Sample a `Curve2d` at `n` uniform parameters of its own range.
fn sample_curve2d(curve: &Curve2d, n: usize) -> Vec<draper_geometry::Point2d> {
    let (t0, t1) = curve.param_range();
    (0..n)
        .map(|i| curve.point_at(t0 + (t1 - t0) * i as f64 / (n - 1) as f64))
        .collect()
}

// ---------------------------------------------------------------
// 1. Plane × cylinder: analytical parametric substitution
// ---------------------------------------------------------------

#[test]
fn test_pcurves_tilted_plane_cylinder_analytic() {
    // Cylinder R = 10 (axis z, x_dir = X) ∩ plane tilted 30° about Y
    // through the origin. Substituting the cylinder parametrization into
    // the plane equation gives v(u) = (d − R·α·cos u − R·β·sin u)/γ with
    // d = 0, α = sin 30° = 0.5, β = 0, γ = cos 30°.
    let tilt = std::f64::consts::FRAC_PI_6; // 30°
    let normal = Direction3d::new(tilt.sin(), 0.0, tilt.cos()).expect("unit normal");
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::ORIGIN,
        normal,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(10.0));

    let out = intersect_surfaces(&plane, &cyl, 1e-6);
    assert_eq!(out.b_splines().len(), 1, "tilted plane ∩ cylinder: one ellipse branch");
    assert_eq!(out.pcurves_a().len(), 1, "one PCURVE on the plane side");
    assert_eq!(out.pcurves_b().len(), 1, "one PCURVE on the cylinder side");

    // ── Cylinder-side PCURVE: the analytic relation must hold ──
    // v(u) = (0 − 10·0.5·cos u − 0) / cos 30° = −5·cos u / cos 30°.
    let gamma = tilt.cos();
    let v_expected = |u: f64| (-10.0 * 0.5 * u.cos()) / gamma;
    let cyl_pcurve = &out.pcurves_b()[0];
    let samples = sample_curve2d(cyl_pcurve, 64);
    let mut max_resid = 0.0_f64;
    for p in &samples {
        let resid = (p.v - v_expected(p.u)).abs();
        if resid > max_resid {
            max_resid = resid;
        }
    }
    // UV gate is tolerance/metric = 1e-6/10 = 1e-7; allow an order of margin.
    assert!(
        max_resid < 1e-5,
        "cylinder-side v(u) substitution residual {:.3e} exceeds 1e-5",
        max_resid
    );

    // Composed: S_cyl(pcurve(t)) must lie ON the plane (the substitution
    // satisfies the plane equation exactly at every fitted sample).
    let mut max_plane_dev = 0.0_f64;
    for p in &samples {
        let q = cyl.point_at(p.u, p.v);
        let signed = q.x * normal.x + q.y * normal.y + q.z * normal.z;
        if signed.abs() > max_plane_dev {
            max_plane_dev = signed.abs();
        }
    }
    assert!(
        max_plane_dev < 1e-5,
        "composed cylinder points off the plane by {:.3e}",
        max_plane_dev
    );

    // ── Plane-side PCURVE: composed points must lie ON the cylinder ──
    let plane_pcurve = &out.pcurves_a()[0];
    let mut max_cyl_dev = 0.0_f64;
    for p in sample_curve2d(plane_pcurve, 64) {
        let q = plane.point_at(p.u, p.v);
        let radial = (q.x * q.x + q.y * q.y).sqrt();
        let dev = (radial - 10.0).abs();
        if dev > max_cyl_dev {
            max_cyl_dev = dev;
        }
    }
    assert!(
        max_cyl_dev < 1e-4,
        "composed plane points off the cylinder by {:.3e}",
        max_cyl_dev
    );

    // Storage types per §2.2 step 3: a tilted ellipse (plane side) and a
    // sinusoid (cylinder side) are both curved — Nurbs, not Line.
    assert!(
        matches!(cyl_pcurve, Curve2d::Nurbs(_)),
        "cylinder-side pcurve must be Nurbs, got {:?}",
        std::mem::discriminant(cyl_pcurve)
    );
    assert!(
        matches!(plane_pcurve, Curve2d::Nurbs(_)),
        "plane-side pcurve must be Nurbs"
    );

    // Parameter contract: the pcurve shares the branch parameter t ∈ [0,1].
    assert_eq!(cyl_pcurve.param_range(), (0.0, 1.0));
    assert_eq!(plane_pcurve.param_range(), (0.0, 1.0));
}

#[test]
fn test_pcurves_perpendicular_plane_cylinder_circle() {
    // Plane z = 2 ∩ cylinder R = 5 (axis z) = circle at height 2.
    // Substitution: α = β = 0, γ = 1 → v(u) = d = 2 (constant).
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 2.0),
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));

    let out = intersect_surfaces(&cyl, &plane, 1e-6);
    assert!(
        out.b_splines().len() >= 1,
        "perpendicular plane ∩ cylinder must yield the circle branch"
    );
    assert_eq!(out.pcurves_a().len(), out.b_splines().len());
    assert_eq!(out.pcurves_b().len(), out.b_splines().len());

    // Cylinder-side PCURVE (surface A here): v ≡ 2, u spans the full circle.
    let cyl_pcurve = &out.pcurves_a()[0];
    let samples = sample_curve2d(cyl_pcurve, 64);
    let mut max_v_dev = 0.0_f64;
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    for p in &samples {
        max_v_dev = max_v_dev.max((p.v - 2.0).abs());
        u_min = u_min.min(p.u);
        u_max = u_max.max(p.u);
    }
    assert!(
        max_v_dev < 1e-6,
        "cylinder-side v must be constant 2 (deviation {:.3e})",
        max_v_dev
    );
    assert!(
        u_max - u_min > std::f64::consts::PI,
        "u must sweep a substantial arc, got span {:.3}",
        u_max - u_min
    );

    // Plane-side PCURVE: exact circle of radius 5 around the axis point.
    let plane_pcurve = &out.pcurves_b()[0];
    let mut max_r_dev = 0.0_f64;
    for p in sample_curve2d(plane_pcurve, 64) {
        let q = plane.point_at(p.u, p.v);
        let radial = (q.x * q.x + q.y * q.y).sqrt();
        max_r_dev = max_r_dev.max((radial - 5.0).abs());
    }
    assert!(
        max_r_dev < 1e-4,
        "plane-side circle radius deviation {:.3e}",
        max_r_dev
    );
}

#[test]
fn test_pcurves_parallel_plane_cylinder_generator_lines() {
    // Plane x = 2 (parallel to the cylinder axis) ∩ cylinder R = 5 → two
    // generator lines at cos u = 2/5. The substitution degenerates (γ = 0),
    // so the generic projection path represents the lines — and straight
    // UV images are stored as Curve2d::Line (§2.2 step 3).
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(2.0, 0.0, 0.0),
        Direction3d::X,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));

    let out = intersect_surfaces(&cyl, &plane, 1e-6);
    assert!(
        out.b_splines().len() >= 2,
        "parallel plane ∩ cylinder must yield two generator lines, got {}",
        out.b_splines().len()
    );
    assert_eq!(out.pcurves_a().len(), out.b_splines().len());
    assert_eq!(out.pcurves_b().len(), out.b_splines().len());

    // cos u = 0.4 → u = ±arccos(0.4) (the second in canonical [0, 2π)).
    let u_expected = 0.4_f64.acos();
    let mut matched_lines = 0usize;
    for pcurve in out.pcurves_a() {
        // Cylinder-side: every branch must be a Line at constant u.
        if let Curve2d::Line(line) = pcurve {
            let du = (line.end.u - line.start.u).abs();
            assert!(du < 1e-9, "generator line must hold u constant, du = {du:.3e}");
            let u_mid = 0.5 * (line.start.u + line.end.u);
            let u_canon = if u_mid < 0.0 { u_mid + 2.0 * std::f64::consts::PI } else { u_mid };
            let dist = (u_canon - u_expected).abs()
                .min((u_canon - (2.0 * std::f64::consts::PI - u_expected)).abs());
            assert!(
                dist < 1e-6,
                "generator u {u_canon:.6} does not match ±arccos(0.4)"
            );
            matched_lines += 1;
        }
    }
    assert_eq!(
        matched_lines,
        out.b_splines().len(),
        "all cylinder-side pcurves must be stored as Line"
    );

    // Plane-side: straight generator segments — also Line.
    for pcurve in out.pcurves_b() {
        assert!(
            matches!(pcurve, Curve2d::Line(_)),
            "plane-side generator pcurve must be Line, got {:?}",
            std::mem::discriminant(pcurve)
        );
    }
}

// ---------------------------------------------------------------
// 2. NURBS surface: Newton-Raphson inversion (generic path)
// ---------------------------------------------------------------

#[test]
fn test_pcurves_nurbs_plane_newton_inversion() {
    // Bilinear NURBS patch (z = 0, spanning [0,10]×[0,10]) ∩ cylinder
    // R = 3 centered at (5,5) → circle of radius 3. The NURBS-side PCURVE
    // comes from Newton-Raphson inversion (§2.2 step 2).
    let patch = Surface::Nurbs(NurbsSurface::from_v_rows(
        1,
        1,
        vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(10.0, 0.0, 0.0)],
            vec![Point3d::new(0.0, 10.0, 0.0), Point3d::new(10.0, 10.0, 0.0)],
        ],
        vec![vec![1.0; 2]; 2],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        false,
        false,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new(
        Point3d::new(5.0, 5.0, 0.0),
        Direction3d::Z,
        3.0,
    ));

    let out = intersect_surfaces(&patch, &cyl, 1e-5);
    assert!(
        !out.b_splines().is_empty(),
        "NURBS patch ∩ cylinder must produce at least one fitted branch"
    );
    assert_eq!(out.pcurves_a().len(), out.b_splines().len());
    assert_eq!(out.pcurves_b().len(), out.b_splines().len());
    assert!(out.pcurve_a().is_some(), "first-branch accessor is set");

    // NURBS-side PCURVE: composed points must reproduce the 3D branch.
    let branch = &out.b_splines()[0];
    let eval = Curve3d::Nurbs(branch.clone());
    let nurbs_pcurve = out.pcurve_a().unwrap();
    let mut max_dev = 0.0_f64;
    for p in sample_curve2d(nurbs_pcurve, 64) {
        let q = patch.point_at(p.u, p.v);
        // Distance to the cylinder axis circle: radial ≈ 3, z ≈ 0.
        let radial = ((q.x - 5.0) * (q.x - 5.0) + (q.y - 5.0) * (q.y - 5.0)).sqrt();
        let dev = ((radial - 3.0) * (radial - 3.0) + q.z * q.z).sqrt();
        if dev > max_dev {
            max_dev = dev;
        }
    }
    assert!(
        max_dev < 1e-2,
        "NURBS-side composed deviation from the intersection circle {:.3e}",
        max_dev
    );

    // Cylinder-side PCURVE: composed points on the circle as well.
    let mut max_cyl_dev = 0.0_f64;
    for p in sample_curve2d(&out.pcurves_b()[0], 64) {
        let q = cyl.point_at(p.u, p.v);
        let radial = ((q.x - 5.0) * (q.x - 5.0) + (q.y - 5.0) * (q.y - 5.0)).sqrt();
        let dev = ((radial - 3.0) * (radial - 3.0) + q.z * q.z).sqrt();
        if dev > max_cyl_dev {
            max_cyl_dev = dev;
        }
    }
    assert!(
        max_cyl_dev < 1e-2,
        "cylinder-side composed deviation {:.3e}",
        max_cyl_dev
    );
    let _ = eval;
}

// ---------------------------------------------------------------
// 3. Multi-branch coverage and API contracts
// ---------------------------------------------------------------
#[test]
fn test_pcurves_multibranch_torus_plane() {
    // Torus (R = 10, r = 2) ∩ plane z = 1 → two latitude circles at
    // v = π/6 and v = 5π/6 (sin v = 1/2). Both branches get PCURVEs on
    // both surfaces; the torus u-seam (2π periodic) is crossed by both.
    let torus = Surface::Torus(TorusSurface::new_z(Point3d::ORIGIN, 10.0, 2.0));
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 1.0),
        Direction3d::Z,
    ));
    let out = intersect_surfaces(&torus, &plane, 1e-6);
    assert_eq!(out.b_splines().len(), 2, "two latitude branches");
    assert_eq!(out.pcurves_a().len(), 2);
    assert_eq!(out.pcurves_b().len(), 2);

    // Torus-side PCURVEs: composed points lie on the plane z = 1 and on
    // one of the two latitude radii 10 ± √3.
    let sqrt3 = 3.0_f64.sqrt();
    for pcurve in out.pcurves_a() {
        let mut max_dev = 0.0_f64;
        for p in sample_curve2d(pcurve, 64) {
            let q = torus.point_at(p.u, p.v);
            let radial = (q.x * q.x + q.y * q.y).sqrt();
            let dev = ((radial - (10.0 + sqrt3)) * (radial - (10.0 + sqrt3))
                + (q.z - 1.0) * (q.z - 1.0))
                .sqrt()
                .min(
                    ((radial - (10.0 - sqrt3)) * (radial - (10.0 - sqrt3))
                        + (q.z - 1.0) * (q.z - 1.0))
                        .sqrt(),
                );
            if dev > max_dev {
                max_dev = dev;
            }
        }
        assert!(
            max_dev < 1e-2,
            "torus-side composed deviation from the latitude circles {:.3e}",
            max_dev
        );
    }

    // Plane-side PCURVEs: composed points lie on the torus (checked via
    // the plane's own coordinates reproducing a latitude circle).
    for pcurve in out.pcurves_b() {
        let mut max_dev = 0.0_f64;
        for p in sample_curve2d(pcurve, 64) {
            let q = plane.point_at(p.u, p.v);
            let radial = (q.x * q.x + q.y * q.y).sqrt();
            let dev = ((radial - (10.0 + sqrt3)) * (radial - (10.0 + sqrt3)))
                .sqrt()
                .min(((radial - (10.0 - sqrt3)) * (radial - (10.0 - sqrt3))).sqrt());
            if dev > max_dev {
                max_dev = dev;
            }
        }
        assert!(
            max_dev < 1e-2,
            "plane-side composed deviation from the latitude circles {:.3e}",
            max_dev
        );
    }
}

#[test]
fn test_pcurve_accessors_and_correspondence() {
    // API contract: positional correspondence with b_spline_curves, [0,1]
    // parameter ranges, and the immutable try_fit variant agrees with the
    // stored pcurves.
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 2.0),
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));
    let out = intersect_surfaces(&plane, &cyl, 1e-6);

    assert_eq!(out.b_splines().len(), out.pcurves_a().len());
    assert_eq!(out.b_splines().len(), out.pcurves_b().len());
    assert!(out.pcurve_a().is_some() == out.pcurves_a().is_empty().not());
    assert!(out.pcurve_b().is_some());

    // Every stored pcurve (Nurbs or Line) is parameterized over [0, 1].
    for pcurve in out.pcurves_a().iter().chain(out.pcurves_b().iter()) {
        let (t0, t1) = pcurve.param_range();
        assert!((t0 - 0.0).abs() < 1e-12, "pcurve range start {t0}");
        assert!((t1 - 1.0).abs() < 1e-12, "pcurve range end {t1}");
    }

    // Immutable recomputation yields the same branch count.
    let (pa, pb) = out.try_fit_pcurves_on_surfaces(&plane, &cyl, 1e-6);
    assert_eq!(pa.len(), out.pcurves_a().len());
    assert_eq!(pb.len(), out.pcurves_b().len());
}

/// Minimal helper trait for readability in the accessor test.
trait BoolExt {
    fn not(self) -> bool;
}
impl BoolExt for bool {
    fn not(self) -> bool {
        !self
    }
}

// ---------------------------------------------------------------
// 4. Periodic PCURVEs for closed branches (§2.2 periodic extension —
//    the 2D mirror of the §2.1 C2-seam periodicity)
// ---------------------------------------------------------------

use draper_geometry::Nurbs2d;
use draper_geometry::Point2d;

/// One-sided derivative estimates AT the seam (4-point one-sided
/// stencils — exact for cubic polynomials, and each stencil lies within a
/// single cubic span of the fitted curve, so the estimates are exact up
/// to roundoff):
///   C'(0⁺) ≈ (−11·C(0) + 18·C(h) − 9·C(2h) + 2·C(3h)) / (6h)
///   C'(1⁻) ≈ ( 11·C(1) − 18·C(1−h) + 9·C(1−2h) − 2·C(1−3h)) / (6h)
/// For the periodic storage the last span is the first span translated
/// by the lattice, and the uniform knot multiplicity makes the curve C1
/// across the seam knot — the exact piecewise estimates agree to machine
/// level. A clamped weld (C0 only) leaves an O(|C'|) jump.
fn seam_tangent_jump_2d(curve: &Nurbs2d, h: f64) -> f64 {
    let c0 = curve.point_at(0.0);
    let ch = curve.point_at(h);
    let c2h = curve.point_at(2.0 * h);
    let c3h = curve.point_at(3.0 * h);
    let c1 = curve.point_at(1.0);
    let cm = curve.point_at(1.0 - h);
    let cm2 = curve.point_at(1.0 - 2.0 * h);
    let cm3 = curve.point_at(1.0 - 3.0 * h);
    let d0 = (
        (-11.0 * c0.u + 18.0 * ch.u - 9.0 * c2h.u + 2.0 * c3h.u) / (6.0 * h),
        (-11.0 * c0.v + 18.0 * ch.v - 9.0 * c2h.v + 2.0 * c3h.v) / (6.0 * h),
    );
    let d1 = (
        (11.0 * c1.u - 18.0 * cm.u + 9.0 * cm2.u - 2.0 * cm3.u) / (6.0 * h),
        (11.0 * c1.v - 18.0 * cm.v + 9.0 * cm2.v - 2.0 * cm3.v) / (6.0 * h),
    );
    ((d0.0 - d1.0).powi(2) + (d0.1 - d1.1).powi(2)).sqrt()
}

/// One-sided curvature (C'') estimates AT the seam from 4-point one-sided
/// stencils — exact for cubic polynomials (each stencil lies within a
/// single cubic span); the lattice offset is a CONSTANT translation over
/// the tail spans, so it cancels in the differences and both sides
/// compare directly. For a C2 seam (uniform knots, multiplicity 1) the
/// exact piecewise estimates agree to roundoff; a C0/C1 weld leaves an
/// O(|ΔC'|/h) jump.
fn seam_curvature_jump_2d(curve: &Nurbs2d, h: f64) -> f64 {
    let c0 = curve.point_at(0.0);
    let ch = curve.point_at(h);
    let c2h = curve.point_at(2.0 * h);
    let c3h = curve.point_at(3.0 * h);
    let c1 = curve.point_at(1.0);
    let cm = curve.point_at(1.0 - h);
    let cm2 = curve.point_at(1.0 - 2.0 * h);
    let cm3 = curve.point_at(1.0 - 3.0 * h);
    let dd0 = (
        (2.0 * c0.u - 5.0 * ch.u + 4.0 * c2h.u - c3h.u) / (h * h),
        (2.0 * c0.v - 5.0 * ch.v + 4.0 * c2h.v - c3h.v) / (h * h),
    );
    let dd1 = (
        (2.0 * c1.u - 5.0 * cm.u + 4.0 * cm2.u - cm3.u) / (h * h),
        (2.0 * c1.v - 5.0 * cm.v + 4.0 * cm2.v - cm3.v) / (h * h),
    );
    ((dd0.0 - dd1.0).powi(2) + (dd0.1 - dd1.1).powi(2)).sqrt()
}

/// Lattice closure of a fitted pcurve: C(1) − C(0). For a Line this is
/// end − start; for a Nurbs, point_at at both domain ends.
fn pcurve_closure(curve: &Curve2d) -> Point2d {
    match curve {
        Curve2d::Line(l) => Point2d::new(l.end.u - l.start.u, l.end.v - l.start.v),
        Curve2d::Nurbs(n) => {
            let (t0, t1) = n.param_range();
            let a = n.point_at(t0);
            let b = n.point_at(t1);
            Point2d::new(b.u - a.u, b.v - a.v)
        }
        _ => Point2d::ORIGIN,
    }
}

#[test]
fn test_pcurve_closed_periodic_analytic_circle() {
    // Plane z = 2 ∩ cylinder R = 5 (axis z) → full circle. The analytic
    // plane×cylinder path must produce:
    //   cylinder side — exact straight UV image (v ≡ 2, u sweeps ±2π):
    //     the closure is exactly one lattice period in u;
    //   plane side — periodic Nurbs (knots[0] < 0, tail = head + lattice):
    //     C(1) == C(0) exactly and C1/C2 continue across the seam.
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 2.0),
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));

    let out = intersect_surfaces(&cyl, &plane, 1e-6);
    assert_eq!(out.b_splines().len(), 1);

    // ── Cylinder side (surface A): straight UV image, lattice closure ──
    let cyl_pcurve = &out.pcurves_a()[0];
    let closure = pcurve_closure(cyl_pcurve);
    let two_pi = 2.0 * std::f64::consts::PI;
    let k = (closure.u / two_pi).round();
    assert!(
        (closure.u - k * two_pi).abs() < 1e-9 && k.abs() >= 1.0,
        "cylinder-side closure must be a non-trivial u lattice multiple, got ({:.6},{:.6})",
        closure.u,
        closure.v
    );
    assert!(
        closure.v.abs() < 1e-9,
        "cylinder-side v closure must vanish, got {:.3e}",
        closure.v
    );
    // v stays constant over the whole curve (the exact analytic image).
    let mut max_v_dev = 0.0_f64;
    for p in sample_curve2d(cyl_pcurve, 64) {
        max_v_dev = max_v_dev.max((p.v - 2.0).abs());
    }
    assert!(max_v_dev < 1e-6, "cylinder-side v dev {max_v_dev:.3e}");

    // ── Plane side (surface B): periodic Nurbs with C2 seam ──
    let plane_pcurve = &out.pcurves_b()[0];
    let nurbs = match plane_pcurve {
        Curve2d::Nurbs(n) => n,
        other => panic!("plane-side circle must be a Nurbs, got {other:?}"),
    };
    // Periodic storage signature: uniform knots start BELOW the domain
    // (knots[0] = −p/n_cp); a clamped fit has knots[0] == 0.
    assert!(
        nurbs.knots[0] < -1e-9,
        "periodic knots must start below the domain, knots[0] = {:.6}",
        nurbs.knots[0]
    );
    // C0: positionally closed (lattice (0,0)) — exact.
    let gap = pcurve_closure(plane_pcurve);
    assert!(
        gap.u.abs() < 1e-9 && gap.v.abs() < 1e-9,
        "plane-side periodic closure must be exact, got ({:.3e},{:.3e})",
        gap.u,
        gap.v
    );
    // C1: tangents continue across the seam.
    let t_jump = seam_tangent_jump_2d(nurbs, 1e-3);
    assert!(
        t_jump < 5e-3,
        "plane-side seam must be C1: tangent jump = {t_jump:.3e}"
    );
    // C2: curvature estimates agree from both sides (a clamped weld would
    // leave an O(|ΔC'|/h) ≈ O(10³) jump).
    let c_jump = seam_curvature_jump_2d(nurbs, 1e-3);
    assert!(
        c_jump < 0.05,
        "plane-side seam must be C2: curvature jump = {c_jump:.3e}"
    );
    // Composed circle quality is unchanged by the periodic knots.
    let mut max_r_dev = 0.0_f64;
    for p in sample_curve2d(plane_pcurve, 64) {
        let q = plane.point_at(p.u, p.v);
        let radial = (q.x * q.x + q.y * q.y).sqrt();
        max_r_dev = max_r_dev.max((radial - 5.0).abs());
    }
    assert!(max_r_dev < 1e-4, "plane-side circle radius dev {max_r_dev:.3e}");
}

#[test]
fn test_pcurve_closed_periodic_torus_plane_marching() {
    // Torus (R=10, r=2) ∩ plane z=1 → two latitude circles (marching
    // path, generic projection). Both sides must close modulo their
    // surface's lattice with a C2-continuous seam: torus side wraps u by
    // exactly ±2π (or degenerates to the exact straight image), plane
    // side closes positionally (periodic Nurbs).
    let torus = Surface::Torus(TorusSurface::new_z(Point3d::ORIGIN, 10.0, 2.0));
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 1.0),
        Direction3d::Z,
    ));
    let out = intersect_surfaces(&torus, &plane, 1e-6);
    assert_eq!(out.b_splines().len(), 2, "two latitude branches");

    let two_pi = 2.0 * std::f64::consts::PI;
    // ── Torus side: lattice closure in u (v may drift only within fit
    //    noise — v is aperiodic in u-wrapping branches... torus v IS
    //    periodic, but a latitude circle keeps v constant, so both
    //    closures must be lattice vectors) ──
    for pcurve in out.pcurves_a() {
        let closure = pcurve_closure(pcurve);
        let ku = (closure.u / two_pi).round();
        let kv = (closure.v / two_pi).round();
        assert!(
            (closure.u - ku * two_pi).abs() < 1e-6 && (closure.v - kv * two_pi).abs() < 1e-6,
            "torus-side closure must be a UV lattice vector, got ({:.6},{:.6})",
            closure.u,
            closure.v
        );
        if let Curve2d::Nurbs(n) = pcurve {
            // C2 seam in the quotient (the lattice is a pure translation:
            // it cancels in the finite differences).
            let t_jump = seam_tangent_jump_2d(n, 1e-3);
            assert!(t_jump < 5e-3, "torus-side seam C1: {t_jump:.3e}");
            let c_jump = seam_curvature_jump_2d(n, 1e-3);
            assert!(c_jump < 0.05, "torus-side seam C2: {c_jump:.3e}");
        }
        // A Line is the exact straight image (v ≈ const) — equally valid.
    }

    // ── Plane side: periodic Nurbs, positionally closed ──
    for pcurve in out.pcurves_b() {
        let nurbs = match pcurve {
            Curve2d::Nurbs(n) => n,
            other => panic!("plane-side latitude circle must be a Nurbs, got {other:?}"),
        };
        assert!(
            nurbs.knots[0] < -1e-9,
            "periodic knots must start below the domain, knots[0] = {:.6}",
            nurbs.knots[0]
        );
        let gap = pcurve_closure(pcurve);
        assert!(
            gap.u.abs() < 1e-9 && gap.v.abs() < 1e-9,
            "plane-side periodic closure must be exact, got ({:.3e},{:.3e})",
            gap.u,
            gap.v
        );
        let t_jump = seam_tangent_jump_2d(nurbs, 1e-3);
        assert!(t_jump < 5e-3, "plane-side seam C1: {t_jump:.3e}");
        let c_jump = seam_curvature_jump_2d(nurbs, 1e-3);
        assert!(c_jump < 0.05, "plane-side seam C2: {c_jump:.3e}");
    }
}

#[test]
fn test_pcurve_periodic_storage_layout() {
    // Self-consistency of the periodic Nurbs2d storage (mirror of the
    // §2.1 layout test): knots strictly increasing, count = n_cp + p + 1
    // over the STORED points, domain [0, 1] = one period, tail control
    // points = head + lattice, C(1) − C(0) = lattice exactly.
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(0.0, 0.0, 2.0),
        Direction3d::Z,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-6);

    let nurbs = match &out.pcurves_b()[0] {
        Curve2d::Nurbs(n) => n,
        other => panic!("plane-side circle must be a Nurbs, got {other:?}"),
    };
    let p = nurbs.degree;
    let n_store = nurbs.control_points.len();
    assert!(n_store > p, "stored control points must exceed degree");
    assert_eq!(
        nurbs.knots.len(),
        n_store + p + 1,
        "knot count = n_store + degree + 1"
    );
    for w in nurbs.knots.windows(2) {
        assert!(w[1] > w[0], "knots strictly increasing");
    }
    let (t0, t1) = nurbs.param_range();
    assert!((t0 - 0.0).abs() < 1e-12, "domain start {t0}");
    assert!((t1 - 1.0).abs() < 1e-12, "domain end {t1}");

    // Period structure: n_cp = n_store − degree distinct points; tail
    // degree points duplicate the head (lattice (0,0) here).
    let n_cp = n_store - p;
    for i in 0..p {
        let head = &nurbs.control_points[i];
        let tail = &nurbs.control_points[n_cp + i];
        assert!(
            (head.u - tail.u).abs() < 1e-12 && (head.v - tail.v).abs() < 1e-12,
            "tail control point {i} must wrap the head (lattice (0,0)): ({:.6},{:.6}) vs ({:.6},{:.6})",
            tail.u,
            tail.v,
            head.u,
            head.v
        );
    }
    // Exact closure via evaluation.
    let a = nurbs.point_at(0.0);
    let b = nurbs.point_at(1.0);
    assert!(
        (a.u - b.u).abs() < 1e-12 && (a.v - b.v).abs() < 1e-12,
        "C(1) == C(0) exactly, got Δ = ({:.3e},{:.3e})",
        b.u - a.u,
        b.v - a.v
    );
}

#[test]
fn test_pcurve_open_branch_not_periodic() {
    // Plane x = 2 ∥ cylinder R=5 axis → two open generator lines. Their
    // UV images are exact straight segments — Line2d, NOT periodic Nurbs
    // (the closed-branch machinery must not touch open branches).
    let plane = Surface::Plane(Plane::from_origin_and_normal(
        Point3d::new(2.0, 0.0, 0.0),
        Direction3d::X,
    ));
    let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));
    let out = intersect_surfaces(&cyl, &plane, 1e-6);
    assert!(out.b_splines().len() >= 2, "two generator lines");
    assert_eq!(out.pcurves_a().len(), out.b_splines().len());
    assert_eq!(out.pcurves_b().len(), out.b_splines().len());

    // The 3D branches stay open (endpoint gap O(span)) — and their
    // PCURVEs are straight segments in both UV spaces.
    for (i, branch) in out.b_splines().iter().enumerate() {
        let eval = Curve3d::Nurbs(branch.clone());
        let gap = eval.point_at(0.0).distance_to(&eval.point_at(1.0));
        assert!(gap > 0.5, "branch {i} must stay open, gap = {gap:.3e}");
        for pcurve in [&out.pcurves_a()[i], &out.pcurves_b()[i]] {
            assert!(
                matches!(pcurve, Curve2d::Line(_)),
                "open branch {i} pcurve must be a straight UV line, got {:?}",
                std::mem::discriminant(pcurve)
            );
        }
    }
}
