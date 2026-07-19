// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! IntersectionCurve — analytical curve representing the intersection of two surfaces.
//!
//! Uses a 4D Newton iteration ("double_projection") to find points that lie on
//! both surfaces simultaneously. The leader curve (typically a polyline) provides
//! an initial guess for the Newton iteration.
//!
//! Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT),
//! specifically the `double_projection` and `IntersectionCurve` implementations
//! in `truck-geometry/src/decorators/intersection_curve.rs`.

use crate::{Point2d, Point3d, Vec3d, Direction3d};
use crate::surface::Surface;
use crate::curve::Curve3d;

/// A 4D vector (u0, v0, u1, v1) used in the Newton iteration.
#[derive(Clone, Copy, Debug)]
struct Vec4 {
    x: f64, // u0
    y: f64, // v0
    z: f64, // u1
    w: f64, // v1
}

/// A 4x4 matrix used in the Newton iteration Jacobian.
#[derive(Clone, Copy, Debug)]
struct Mat4 {
    m: [[f64; 4]; 4], // row-major
}

impl Mat4 {
    fn from_cols(c0: [f64; 4], c1: [f64; 4], c2: [f64; 4], c3: [f64; 4]) -> Self {
        Self {
            m: [
                [c0[0], c1[0], c2[0], c3[0]],
                [c0[1], c1[1], c2[1], c3[1]],
                [c0[2], c1[2], c2[2], c3[2]],
                [c0[3], c1[3], c2[3], c3[3]],
            ],
        }
    }

    /// Solve the linear system M * x = b using Gaussian elimination with partial pivoting.
    /// Returns None if the matrix is singular.
    fn solve(&self, b: [f64; 4]) -> Option<[f64; 4]> {
        let mut a = self.m;
        let mut rhs = b;

        // Forward elimination with partial pivoting
        for col in 0..4 {
            // Find pivot
            let mut max_row = col;
            let mut max_val = a[col][col].abs();
            for row in (col + 1)..4 {
                if a[row][col].abs() > max_val {
                    max_val = a[row][col].abs();
                    max_row = row;
                }
            }
            if max_val < 1e-15 {
                return None; // Singular
            }
            // Swap rows
            if max_row != col {
                a.swap(col, max_row);
                rhs.swap(col, max_row);
            }
            // Eliminate
            for row in (col + 1)..4 {
                let factor = a[row][col] / a[col][col];
                for k in col..4 {
                    a[row][k] -= factor * a[col][k];
                }
                rhs[row] -= factor * rhs[col];
            }
        }

        // Back substitution
        let mut x = [0.0; 4];
        for i in (0..4).rev() {
            let mut sum = rhs[i];
            for k in (i + 1)..4 {
                sum -= a[i][k] * x[k];
            }
            x[i] = sum / a[i][i];
        }
        Some(x)
    }
}

/// An intersection curve between two surfaces.
///
/// The curve is defined by a "leader" curve (typically a polyline approximating
/// the intersection) and the two surfaces. The exact 3D point at parameter t is
/// found via 4D Newton iteration, projecting the leader's point onto both surfaces
/// simultaneously.
#[derive(Clone, Debug)]
pub struct IntersectionCurve {
    /// The first surface.
    pub surface0: Box<Surface>,
    /// The second surface.
    pub surface1: Box<Surface>,
    /// The leader curve — provides initial guesses for the Newton iteration.
    /// Typically a polyline through sample points on the intersection.
    pub leader: Box<Curve3d>,
}

/// The result of a search_triple call: (3D point, UV on surface0, UV on surface1).
pub type Triple = (Point3d, Point2d, Point2d);

impl IntersectionCurve {
    /// Construct a new IntersectionCurve.
    pub fn new(surface0: Surface, surface1: Surface, leader: Curve3d) -> Self {
        Self {
            surface0: Box::new(surface0),
            surface1: Box::new(surface1),
            leader: Box::new(leader),
        }
    }

    /// Search for the triple (3D point, UV0, UV1) at parameter t on the leader curve.
    ///
    /// Uses 4D Newton iteration ("double_projection") to find (u0, v0, u1, v1)
    /// such that S0(u0, v0) ≈ S1(u1, v1) ≈ leader.point_at(t).
    ///
    /// The function value is:
    ///   F = (S0(u0,v0) - S1(u1,v1), n·((S0+S1)/2 - plane_point))
    /// where n = leader'(t) (the leader's tangent) and plane_point = leader(t).
    ///
    /// The 4×4 Jacobian is built from the surface partial derivatives.
    ///
    /// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
    pub fn search_triple(&self, t: f64, trials: usize) -> Option<Triple> {
        let plane_point = self.leader.point_at(t);
        let plane_normal = self.leader.derivative_at(t);

        // Get initial hints by projecting plane_point onto each surface
        let hint0 = self.surface0.project_point_opt(&plane_point);
        let hint1 = self.surface1.project_point_opt(&plane_point);

        double_projection(
            &self.surface0,
            hint0,
            &self.surface1,
            hint1,
            plane_point,
            plane_normal,
            trials,
        )
    }

    /// Evaluate the 3D point at parameter t.
    ///
    /// Calls search_triple and returns the 3D point.
    pub fn point_at(&self, t: f64) -> Point3d {
        self.search_triple(t, 50)
            .map(|(p, _, _)| p)
            .unwrap_or_else(|| self.leader.point_at(t))
    }

    /// Evaluate the first derivative (tangent vector) at parameter t.
    ///
    /// Uses the formula:
    ///   n0 = surface0.normal(uv0)
    ///   n1 = surface1.normal(uv1)
    ///   n = n0 × n1  (direction of the intersection curve)
    ///   k = (|leader'|² - (c - leader)·leader'') / (n · leader')
    ///   c' = n * k
    ///
    /// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        let leader_pt = self.leader.point_at(t);
        let leader_der = self.leader.derivative_at(t);

        // Numerical second derivative of the leader (central differences)
        let dt = 1e-7;
        let leader_der_plus = self.leader.derivative_at(t + dt);
        let leader_der_minus = self.leader.derivative_at(t - dt);
        let leader_der2 = Vec3d::new(
            (leader_der_plus.x - leader_der_minus.x) / (2.0 * dt),
            (leader_der_plus.y - leader_der_minus.y) / (2.0 * dt),
            (leader_der_plus.z - leader_der_minus.z) / (2.0 * dt),
        );

        let (c, uv0, uv1) = match self.search_triple(t, 50) {
            Some(triple) => triple,
            None => return leader_der, // Fallback to leader derivative
        };

        // Surface normals at the intersection point
        let n0 = self.surface0.derivatives_at(uv0.u, uv0.v).normal();
        let n1 = self.surface1.derivatives_at(uv1.u, uv1.v).normal();

        // Direction of the intersection curve: n = n0 × n1
        let n = n0.cross(&n1);
        let n_dir = match Direction3d::new(n.x, n.y, n.z) {
            Some(d) => d,
            None => return leader_der, // Normals are parallel — intersection is degenerate
        };

        // Compute the scaling factor k
        let leader_der_sq = leader_der.x * leader_der.x
            + leader_der.y * leader_der.y
            + leader_der.z * leader_der.z;
        let diff = Vec3d::new(c.x - leader_pt.x, c.y - leader_pt.y, c.z - leader_pt.z);
        let diff_dot_der2 = diff.x * leader_der2.x + diff.y * leader_der2.y + diff.z * leader_der2.z;
        let n_dot_leader_der = n_dir.x * leader_der.x + n_dir.y * leader_der.y + n_dir.z * leader_der.z;

        if n_dot_leader_der.abs() < 1e-15 {
            return leader_der; // Avoid division by zero
        }

        let k = (leader_der_sq - diff_dot_der2) / n_dot_leader_der;
        Vec3d::new(n_dir.x * k, n_dir.y * k, n_dir.z * k)
    }

    /// Get the parameter range of the leader curve.
    pub fn param_range(&self) -> (f64, f64) {
        self.leader.param_range()
    }
}

/// 4D Newton iteration to find (u0, v0, u1, v1) such that S0(u0,v0) ≈ S1(u1,v1) ≈ plane_point.
///
/// The function value is a 4D vector:
///   F[0..3] = S0(u0,v0) - S1(u1,v1)  (the 3D difference)
///   F[3] = n · ((S0 + S1)/2 - plane_point)  (projection onto the leader's tangent)
///
/// The 4×4 Jacobian columns are:
///   col0 = (S0_u, n·S0_u/2)        (derivative w.r.t. u0)
///   col1 = (S0_v, n·S0_v/2)        (derivative w.r.t. v0)
///   col2 = (-S1_u, -n·S1_u/2)      (derivative w.r.t. u1)
///   col3 = (-S1_v, -n·S1_v/2)      (derivative w.r.t. v1)
///
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
fn double_projection(
    surface0: &Surface,
    hint0: Option<(f64, f64)>,
    surface1: &Surface,
    hint1: Option<(f64, f64)>,
    plane_point: Point3d,
    plane_normal: Vec3d,
    trials: usize,
) -> Option<Triple> {
    // Initialize (u0, v0, u1, v1) from hints or surface midpoints
    let (u0_min, u0_max) = surface_param_range_u(surface0);
    let (v0_min, v0_max) = surface_param_range_v(surface0);
    let (u1_min, u1_max) = surface_param_range_u(surface1);
    let (v1_min, v1_max) = surface_param_range_v(surface1);

    let mut x = Vec4 {
        x: hint0.map(|(u, _)| u).unwrap_or((u0_min + u0_max) * 0.5),
        y: hint0.map(|(_, v)| v).unwrap_or((v0_min + v0_max) * 0.5),
        z: hint1.map(|(u, _)| u).unwrap_or((u1_min + u1_max) * 0.5),
        w: hint1.map(|(_, v)| v).unwrap_or((v1_min + v1_max) * 0.5),
    };

    let n = plane_normal;

    for _ in 0..trials {
        // Evaluate surfaces and their derivatives at the current iterate
        let ders0 = surface0.derivatives_at(x.x, x.y);
        let pt0 = ders0.point;
        let uder0 = ders0.du;
        let vder0 = ders0.dv;

        let ders1 = surface1.derivatives_at(x.z, x.w);
        let pt1 = ders1.point;
        let uder1 = ders1.du;
        let vder1 = ders1.dv;

        // Function value
        let mid = Point3d::new(
            (pt0.x + pt1.x) * 0.5,
            (pt0.y + pt1.y) * 0.5,
            (pt0.z + pt1.z) * 0.5,
        );
        let mid_minus_pp = Vec3d::new(
            mid.x - plane_point.x,
            mid.y - plane_point.y,
            mid.z - plane_point.z,
        );
        let f = [
            pt0.x - pt1.x,
            pt0.y - pt1.y,
            pt0.z - pt1.z,
            n.x * mid_minus_pp.x + n.y * mid_minus_pp.y + n.z * mid_minus_pp.z,
        ];

        // Check convergence
        let f_norm = f[0] * f[0] + f[1] * f[1] + f[2] * f[2] + f[3] * f[3];
        if f_norm.sqrt() < 1e-10 {
            let point = Point3d::new(
                (pt0.x + pt1.x) * 0.5,
                (pt0.y + pt1.y) * 0.5,
                (pt0.z + pt1.z) * 0.5,
            );
            return Some((point, Point2d::new(x.x, x.y), Point2d::new(x.z, x.w)));
        }

        // Jacobian (4x4)
        let jacobian = Mat4::from_cols(
            [uder0.x, uder0.y, uder0.z, (n.x * uder0.x + n.y * uder0.y + n.z * uder0.z) * 0.5],
            [vder0.x, vder0.y, vder0.z, (n.x * vder0.x + n.y * vder0.y + n.z * vder0.z) * 0.5],
            [-uder1.x, -uder1.y, -uder1.z, -(n.x * uder1.x + n.y * uder1.y + n.z * uder1.z) * 0.5],
            [-vder1.x, -vder1.y, -vder1.z, -(n.x * vder1.x + n.y * vder1.y + n.z * vder1.z) * 0.5],
        );

        // Solve J * delta = -F
        let neg_f = [-f[0], -f[1], -f[2], -f[3]];
        let delta = match jacobian.solve(neg_f) {
            Some(d) => d,
            None => return None, // Singular Jacobian
        };

        // Update x
        x.x += delta[0];
        x.y += delta[1];
        x.z += delta[2];
        x.w += delta[3];

        // Clamp to parameter ranges
        x.x = x.x.clamp(u0_min, u0_max);
        x.y = x.y.clamp(v0_min, v0_max);
        x.z = x.z.clamp(u1_min, u1_max);
        x.w = x.w.clamp(v1_min, v1_max);
    }

    // Did not converge — return None
    None
}

/// Get the u-parameter range of a surface (for clamping during Newton iteration).
fn surface_param_range_u(surface: &Surface) -> (f64, f64) {
    use std::f64::consts::PI;
    match surface {
        Surface::Plane(_) | Surface::Extrusion(_) => (-1e6, 1e6),
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) |
        Surface::Torus(_) | Surface::Revolution(_) => (0.0, 2.0 * PI),
        Surface::Nurbs(n) => n.u_range(),
        Surface::Offset(o) => surface_param_range_u(&o.base),
        Surface::Ruled(_) => (-1e6, 1e6),
    }
}

/// Get the v-parameter range of a surface.
fn surface_param_range_v(surface: &Surface) -> (f64, f64) {
    use std::f64::consts::PI;
    match surface {
        Surface::Plane(_) | Surface::Cylinder(_) | Surface::Extrusion(_) |
        Surface::Revolution(_) => (-1e6, 1e6),
        Surface::Cone(_) => (0.0, 1e6),
        Surface::Sphere(_) => (0.0, PI),
        Surface::Torus(_) => (0.0, 2.0 * PI),
        Surface::Nurbs(n) => n.v_range(),
        Surface::Offset(o) => surface_param_range_v(&o.base),
        Surface::Ruled(_) => (0.0, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::{Plane, Surface};
    use crate::curve::Line;
    use crate::{Point3d, Direction3d, Vec3d};

    #[test]
    fn test_mat4_solve_identity() {
        let m = Mat4::from_cols(
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        );
        let b = [1.0, 2.0, 3.0, 4.0];
        let x = m.solve(b).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-12);
        assert!((x[1] - 2.0).abs() < 1e-12);
        assert!((x[2] - 3.0).abs() < 1e-12);
        assert!((x[3] - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_mat4_solve_singular() {
        let m = Mat4::from_cols(
            [1.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0], // col1 is 2x col0 → singular
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        );
        let b = [1.0, 2.0, 3.0, 4.0];
        assert!(m.solve(b).is_none(), "singular matrix should return None");
    }

    #[test]
    fn test_double_projection_two_planes() {
        // Two planes intersecting along the Z axis.
        // Plane0: the XZ plane (y=0)
        // Plane1: the YZ plane (x=0)
        // Intersection: the Z axis (x=0, y=0)
        let plane0 = Surface::Plane(Plane::xz());  // XZ plane (y=0)
        let plane1 = Surface::Plane(Plane::yz());  // YZ plane (x=0)

        // The leader is a line along the Z axis (the actual intersection direction)
        let leader = Curve3d::Line(Line::new(
            Point3d::new(0.0, 0.0, -1.0),
            Direction3d::new(0.0, 0.0, 1.0).unwrap(),
        ));

        let ic = IntersectionCurve::new(plane0, plane1, leader);

        // At t=0.5, the leader point is (0, 0, -0.5), which is ON the intersection line.
        // The search_triple should return a point very close to (0, 0, -0.5).
        let triple = ic.search_triple(0.5, 50);
        assert!(triple.is_some(), "search_triple should find a point on both planes");

        let (p, uv0, uv1) = triple.unwrap();
        // The point should be on the Z axis: (0, 0, z) for some z near -0.5
        assert!(p.x.abs() < 1e-6, "p.x should be 0, got {}", p.x);
        assert!(p.y.abs() < 1e-6, "p.y should be 0, got {}", p.y);
        assert!((p.z - (-0.5)).abs() < 1e-3, "p.z should be ~-0.5, got {}", p.z);

        // The UV on plane0 (XZ plane) for point (0, 0, -0.5) is (u=0, v=-0.5)
        assert!(uv0.u.abs() < 1e-6, "uv0.u should be 0, got {}", uv0.u);
        assert!((uv0.v - (-0.5)).abs() < 1e-3, "uv0.v should be ~-0.5, got {}", uv0.v);

        // The UV on plane1 (YZ plane) for point (0, 0, -0.5) is (u=0, v=-0.5)
        assert!(uv1.u.abs() < 1e-6, "uv1.u should be 0, got {}", uv1.u);
        assert!((uv1.v - (-0.5)).abs() < 1e-3, "uv1.v should be ~-0.5, got {}", uv1.v);
    }

    #[test]
    fn test_intersection_curve_point_at_fallback() {
        // When search_triple fails, point_at falls back to leader.point_at
        let plane = Surface::Plane(Plane::xy());
        // Use the same plane for both surfaces (degenerate — no intersection curve)
        let leader = Curve3d::Line(Line::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::new(1.0, 0.0, 0.0).unwrap(),
        ));
        let ic = IntersectionCurve::new(plane.clone(), plane, leader.clone());

        // point_at should return something (either the projection or the leader fallback)
        let p = ic.point_at(0.5);
        // The leader at t=0.5 is (0.5, 0, 0), which is ON the plane. So search_triple
        // should succeed and return approximately (0.5, 0, 0).
        assert!((p.x - 0.5).abs() < 1e-3, "p.x should be ~0.5, got {}", p.x);
    }

    #[test]
    fn test_intersection_curve_derivative_at() {
        // Two planes intersecting along the Z axis.
        // The derivative should be along Z (the intersection direction).
        let plane0 = Surface::Plane(Plane::xz());  // XZ plane
        let plane1 = Surface::Plane(Plane::yz());  // YZ plane
        // Leader along the Z axis
        let leader = Curve3d::Line(Line::new(
            Point3d::new(0.0, 0.0, -1.0),
            Direction3d::new(0.0, 0.0, 1.0).unwrap(),
        ));
        let ic = IntersectionCurve::new(plane0, plane1, leader);

        let der = ic.derivative_at(0.5);
        // The derivative should be along Z (the intersection direction)
        assert!(der.x.abs() < 1e-3, "der.x should be ~0, got {}", der.x);
        assert!(der.y.abs() < 1e-3, "der.y should be ~0, got {}", der.y);
        // der.z should be non-zero (along the intersection direction)
        assert!(der.z.abs() > 0.1, "der.z should be non-zero, got {}", der.z);
    }
}
