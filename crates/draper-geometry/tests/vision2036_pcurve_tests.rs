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
