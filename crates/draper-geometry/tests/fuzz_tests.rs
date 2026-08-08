// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Fuzz tests for panic-free guarantee (ROADMAP_VISION_2036 §9.2, Directive 3).
//!
//! Uses `quickcheck` to generate random inputs and verify that geometric
//! operations NEVER panic — they return finite results or fall back
//! gracefully.
//!
//! Targets:
//! - NURBS solver: random weights, knot vectors, control points
//! - Surface evaluation: random UV parameters
//! - Curve evaluation: random parameters

use quickcheck::{Arbitrary, Gen, Testable};
use draper_geometry::{Point3d, Surface, Plane, NurbsSurface, NurbsCurve, Curve3d, CylinderSurface, SphereSurface};

/// Random NURBS-like surface parameters that are valid but potentially
/// pathological (zero weights, duplicate control points, etc.)
#[derive(Clone, Debug)]
struct RandomNurbsInput {
    u: f64,
    v: f64,
    weights: Vec<f64>,
    cps: Vec<Point3d>,
}

impl Arbitrary for RandomNurbsInput {
    fn arbitrary(g: &mut Gen) -> Self {
        let n = usize::arbitrary(g) % 8 + 4; // 4..11 control points
        let u = f64::arbitrary(g) % 1.0;
        let v = f64::arbitrary(g) % 1.0;
        let weights: Vec<f64> = (0..n)
            .map(|_| {
                let w = f64::arbitrary(g);
                // Allow zero, negative, and very large weights
                if w.is_nan() || w.is_infinite() { 1.0 } else { w }
            })
            .collect();
        let cps: Vec<Point3d> = (0..n)
            .map(|_| Point3d::new(
                f64::arbitrary(g) % 100.0,
                f64::arbitrary(g) % 100.0,
                f64::arbitrary(g) % 100.0,
            ))
            .collect();
        RandomNurbsInput { u, v, weights, cps }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quickcheck::quickcheck;

    /// Fuzz: NURBS surface evaluation must never panic, even with
    /// pathological control points and weights.
    quickcheck! {
        fn fuzz_nurbs_eval_no_panic(input: RandomNurbsInput) -> bool {
            // Build a degree-1 (linear) NURBS curve — simplest valid NURBS
            if input.cps.len() < 2 {
                return true; // Skip — not enough control points
            }
            let degree = 1;
            let n = input.cps.len();
            let n_knots = n + degree + 1;
            let mut knots = vec![0.0; n_knots];
            for i in 0..n_knots {
                if i <= degree {
                    knots[i] = 0.0;
                } else if i >= n {
                    knots[i] = 1.0;
                } else {
                    knots[i] = (i - degree) as f64 / (n - degree) as f64;
                }
            }
            let weights = if input.weights.len() >= n {
                input.weights[..n].to_vec()
            } else {
                vec![1.0; n]
            };

            let curve = NurbsCurve {
                degree,
                control_points: input.cps.clone(),
                weights,
                knots,
            };
            let c3d = Curve3d::Nurbs(curve);
            // This must NOT panic — if the input is invalid, it should
            // return some point (possibly ORIGIN for NaN/Inf cases)
            let p = c3d.point_at(input.u.clamp(0.0, 1.0));
            true // If we got here, no panic occurred
        }
    }

    /// Fuzz: Surface evaluation must never panic for any parameter.
    quickcheck! {
        fn fuzz_surface_eval_no_panic(u: f64, v: f64) -> bool {
            let u = u.rem_euclid(1.0);
            let v = v.rem_euclid(1.0);

            // Plane
            let plane = Surface::Plane(Plane::xy());
            let _ = plane.point_at(u * 100.0, v * 100.0);

            // Cylinder
            let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0 + (u * 10.0).abs()));
            let _ = cyl.point_at(u * std::f64::consts::TAU, v * 50.0);

            // Sphere
            let sphere = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 1.0 + (v * 10.0).abs()));
            let _ = sphere.point_at(u * std::f64::consts::TAU, v * std::f64::consts::PI);

            true // No panic
        }
    }

    /// Fuzz: Surface normal must never panic and must return a finite direction.
    quickcheck! {
        fn fuzz_normal_no_panic(u: f64, v: f64) -> bool {
            let u = u.rem_euclid(1.0);
            let v = v.rem_euclid(1.0);

            let plane = Surface::Plane(Plane::xy());
            let n = plane.normal_at(u, v);
            // Normal should be finite (not NaN/Inf)
            n.x.is_finite() && n.y.is_finite() && n.z.is_finite()
        }
    }

    /// Fuzz: Derivative evaluation must never panic.
    quickcheck! {
        fn fuzz_derivative_no_panic(t: f64) -> bool {
            let t = t.rem_euclid(1.0);
            let curve = Curve3d::Line(draper_geometry::Line::new(
                Point3d::new(0.0, 0.0, 0.0),
                draper_geometry::Direction3d::new(1.0, 0.0, 0.0).unwrap(),
            ));
            let d = curve.derivative_at(t);
            d.x.is_finite() && d.y.is_finite() && d.z.is_finite()
        }
    }

    /// Fuzz: Intersection functions must never panic.
    /// NOTE: This test also validates that Direction3d::new() doesn't panic
    /// on edge cases — it should return None, not panic.
    quickcheck! {
        fn fuzz_intersection_no_panic(ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64) -> bool {
            // Normalize direction
            let len = (dx*dx + dy*dy + dz*dz).sqrt();
            if len < 1e-10 || !len.is_finite() {
                return true; // Skip degenerate/overflow direction
            }
            // Direction3d::new returns None for near-zero vectors —
            // this is correct behavior (panic-free), we just skip
            let dir = match draper_geometry::Direction3d::new(dx/len, dy/len, dz/len) {
                Some(d) => d,
                None => return true, // Not a panic — Direction3d correctly rejected it
            };
            let line = draper_geometry::Line::new(
                Point3d::new(ox, oy, oz),
                dir,
            );
            let plane = Plane::xy();
            let ctx = draper_geometry::ToleranceContext::default();
            let _ = draper_geometry::intersect_line_plane_with_tolerance(&line, &plane, &ctx);
            true
        }
    }
}
