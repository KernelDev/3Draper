// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 2D parametric curves in UV parameter space.
//!
//! Curve2d represents a curve in the 2D parametric domain of a surface.
//! This is used for PCURVE representation — when a B-Rep edge lies on a
//! surface, its PCURVE defines the exact path in UV space.
//!
//! Supported types:
//! - Line2d: straight line in UV space (from PCURVE LINE in STEP)
//! - Circle2d: circular arc in UV space (from PCURVE CIRCLE in STEP)
//! - Ellipse2d: elliptical arc in UV space (from PCURVE ELLIPSE in STEP)
//! - Hyperbola2d: hyperbolic arc in UV space (from PCURVE HYPERBOLA in STEP)
//! - Parabola2d: parabolic arc in UV space (from PCURVE PARABOLA in STEP)
//! - Nurbs2d: NURBS curve in UV space (from PCURVE B_SPLINE_CURVE in STEP)

use crate::Point2d;
use std::f64::consts::PI;

/// A 2D parametric curve in UV parameter space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Curve2d {
    /// A straight line segment in UV space.
    Line(Line2d),
    /// A circular arc in UV space.
    Circle(Circle2d),
    /// An elliptical arc in UV space.
    Ellipse(Ellipse2d),
    /// A hyperbolic arc in UV space.
    Hyperbola(Hyperbola2d),
    /// A parabolic arc in UV space.
    Parabola(Parabola2d),
    /// A NURBS curve in UV space.
    Nurbs(Nurbs2d),
    /// Composite 2D curve: a sequence of curve segments joined end-to-end.
    ///
    /// The global parameter `t ∈ [0, 1]` is mapped to per-segment local
    /// parameters using arc-length proportional mapping, analogous to
    /// `Curve3d::Composite`.
    Composite {
        segments: Vec<Curve2d>,
        /// Cumulative arc-length fractions [0..1].
        cum_lengths: Vec<f64>,
    },
}

/// A straight line in UV space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Line2d {
    /// Start point in UV space.
    pub start: Point2d,
    /// End point in UV space.
    pub end: Point2d,
}

impl Line2d {
    /// Create a new line from start to end.
    pub fn new(start: Point2d, end: Point2d) -> Self {
        Self { start, end }
    }

    /// Evaluate the line at parameter t ∈ [0, 1].
    pub fn point_at(&self, t: f64) -> Point2d {
        Point2d::new(
            self.start.u + t * (self.end.u - self.start.u),
            self.start.v + t * (self.end.v - self.start.v),
        )
    }

    /// Derivative at parameter t.
    pub fn derivative_at(&self, _t: f64) -> (f64, f64) {
        (self.end.u - self.start.u, self.end.v - self.start.v)
    }

    /// Parameter range.
    pub fn param_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    /// Arc length of the line.
    pub fn length(&self) -> f64 {
        let du = self.end.u - self.start.u;
        let dv = self.end.v - self.start.v;
        (du * du + dv * dv).sqrt()
    }

    /// Closest-point projection of a UV point onto the line segment
    /// (Vision 2036 §1.3 «analytical projections for PCURVE»).
    ///
    /// Returns `(t, dist)`: the parameter `t ∈ [0, 1]` of the closest
    /// point and the Euclidean distance from `p` to it. Exact — one
    /// clamped dot-product projection, no iteration.
    pub fn project_point(&self, p: &Point2d) -> (f64, f64) {
        let du = self.end.u - self.start.u;
        let dv = self.end.v - self.start.v;
        let len_sq = du * du + dv * dv;
        if len_sq < 1e-30 {
            // Degenerate (zero-length) line: the start point is the only candidate.
            return (0.0, uv_distance(p, &self.start));
        }
        let t = (((p.u - self.start.u) * du + (p.v - self.start.v) * dv) / len_sq)
            .clamp(0.0, 1.0);
        let closest = self.point_at(t);
        (t, uv_distance(p, &closest))
    }
}

/// A circular arc in UV space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle2d {
    /// Center of the circle in UV space.
    pub center: Point2d,
    /// Radius of the circle.
    pub radius: f64,
    /// Start angle in radians.
    pub start_angle: f64,
    /// End angle in radians.
    pub end_angle: f64,
}

impl Circle2d {
    /// Create a full circle.
    pub fn new_full(center: Point2d, radius: f64) -> Self {
        Self {
            center,
            radius,
            start_angle: 0.0,
            end_angle: 2.0 * PI,
        }
    }

    /// Create a circular arc from start_angle to end_angle.
    pub fn new_arc(center: Point2d, radius: f64, start_angle: f64, end_angle: f64) -> Self {
        Self { center, radius, start_angle, end_angle }
    }

    /// Evaluate at parameter t ∈ [0, 1].
    pub fn point_at(&self, t: f64) -> Point2d {
        let angle = self.start_angle + t * (self.end_angle - self.start_angle);
        Point2d::new(
            self.center.u + self.radius * angle.cos(),
            self.center.v + self.radius * angle.sin(),
        )
    }

    /// Derivative at parameter t.
    pub fn derivative_at(&self, t: f64) -> (f64, f64) {
        let angle = self.start_angle + t * (self.end_angle - self.start_angle);
        let dangle_dt = self.end_angle - self.start_angle;
        (
            -self.radius * angle.sin() * dangle_dt,
             self.radius * angle.cos() * dangle_dt,
        )
    }

    /// Parameter range.
    pub fn param_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    /// Arc length.
    pub fn length(&self) -> f64 {
        self.radius * (self.end_angle - self.start_angle).abs()
    }

    /// Closest-point projection of a UV point onto the circular arc
    /// (Vision 2036 §1.3 «analytical projections for PCURVE»).
    ///
    /// Returns `(t, dist)`: the parameter `t ∈ [0, 1]` (angle =
    /// `start_angle + t·span`) of the closest point and the Euclidean
    /// distance from `p` to it. Exact — the angular coordinate of the
    /// closest point on the full circle is `atan2`, then constrained to
    /// the arc range; for a point at the center the arc start is returned
    /// (every angle is equally close).
    pub fn project_point(&self, p: &Point2d) -> (f64, f64) {
        let span = self.end_angle - self.start_angle;
        if span.abs() < 1e-15 {
            return (0.0, uv_distance(p, &self.point_at(0.0)));
        }
        let du = p.u - self.center.u;
        let dv = p.v - self.center.v;
        let r_p = (du * du + dv * dv).sqrt();
        // Angular coordinate of p around the center; at the center every
        // angle is equally close — fall back to the arc start ray.
        let theta = if r_p < 1e-15 { self.start_angle } else { dv.atan2(du) };
        // Distance from p to the point of the FULL circle at angle `theta`
        // (radial difference — exact when that point lies on the arc).
        let dist_radial = (r_p - self.radius).abs();
        let is_full = span >= 2.0 * PI - 1e-9;
        if is_full {
            // The radial point is always on the curve: exact global minimum.
            let t = ((theta - self.start_angle) / span).rem_euclid(1.0);
            return (t, dist_radial);
        }
        // Arc: the constrained angular minimum is the clamped candidate
        // (distance² = r_p² + r² − 2·r_p·r·cos(angle−θ) is minimized by the
        // closest allowed angle); endpoints are evaluated for tie-safety.
        let mut best_t = 0.0;
        let mut best_d = uv_distance(p, &self.point_at(0.0));
        let d_end = uv_distance(p, &self.point_at(1.0));
        if d_end < best_d {
            best_t = 1.0;
            best_d = d_end;
        }
        let t_theta = ((theta - self.start_angle) / span).clamp(0.0, 1.0);
        if (t_theta - 0.0).abs() > 1e-15
            && (t_theta - 1.0).abs() > 1e-15
            && dist_radial < best_d
        {
            best_t = t_theta;
            best_d = dist_radial;
        }
        (best_t, best_d)
    }
}

/// An elliptical arc in UV space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse2d {
    /// Center of the ellipse in UV space.
    pub center: Point2d,
    /// Semi-major axis length.
    pub semi_major: f64,
    /// Semi-minor axis length.
    pub semi_minor: f64,
    /// Rotation angle of the major axis in radians.
    pub rotation: f64,
    /// Start angle in radians.
    pub start_angle: f64,
    /// End angle in radians.
    pub end_angle: f64,
}

impl Ellipse2d {
    /// Create a full ellipse.
    pub fn new_full(center: Point2d, semi_major: f64, semi_minor: f64, rotation: f64) -> Self {
        Self {
            center,
            semi_major,
            semi_minor,
            rotation,
            start_angle: 0.0,
            end_angle: 2.0 * PI,
        }
    }

    /// Create an elliptical arc from start_angle to end_angle.
    pub fn new_arc(center: Point2d, semi_major: f64, semi_minor: f64, rotation: f64, start_angle: f64, end_angle: f64) -> Self {
        Self { center, semi_major, semi_minor, rotation, start_angle, end_angle }
    }

    /// Evaluate at parameter t ∈ [0, 1].
    pub fn point_at(&self, t: f64) -> Point2d {
        let angle = self.start_angle + t * (self.end_angle - self.start_angle);
        let cos_r = self.rotation.cos();
        let sin_r = self.rotation.sin();
        let x = self.semi_major * angle.cos();
        let y = self.semi_minor * angle.sin();
        Point2d::new(
            self.center.u + x * cos_r - y * sin_r,
            self.center.v + x * sin_r + y * cos_r,
        )
    }

    /// Derivative at parameter t.
    pub fn derivative_at(&self, t: f64) -> (f64, f64) {
        let angle = self.start_angle + t * (self.end_angle - self.start_angle);
        let dangle_dt = self.end_angle - self.start_angle;
        let cos_r = self.rotation.cos();
        let sin_r = self.rotation.sin();
        let dx = -self.semi_major * angle.sin() * dangle_dt;
        let dy =  self.semi_minor * angle.cos() * dangle_dt;
        (dx * cos_r - dy * sin_r, dx * sin_r + dy * cos_r)
    }

    /// Parameter range.
    pub fn param_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    /// Approximate arc length using numerical integration.
    pub fn length(&self) -> f64 {
        let n = 100;
        let mut length = 0.0;
        let mut prev = self.point_at(0.0);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let curr = self.point_at(t);
            let du = curr.u - prev.u;
            let dv = curr.v - prev.v;
            length += (du * du + dv * dv).sqrt();
            prev = curr;
        }
        length
    }

    /// Closest-point projection of a UV point onto the elliptical arc
    /// (Vision 2036 §1.3 «analytical projections for PCURVE»).
    ///
    /// Returns `(t, dist)`: the parameter `t ∈ [0, 1]` of the closest
    /// point and the Euclidean distance from `p` to it. Uses the robust
    /// scan → golden-section → orthogonal-polish pipeline
    /// (`project_parametric_curve`); the parameterization is smooth and
    /// unimodal near the minimum, so the polish converges to the exact
    /// stationary point (residual `(p − E(t))·E'(t) ≈ 0`).
    pub fn project_point(&self, p: &Point2d) -> (f64, f64) {
        project_parametric_curve(p, 0.0, 1.0, &|t| self.point_at(t), &|t| self.derivative_at(t))
    }
}

/// A hyperbolic arc in UV space.
///
/// Standard form: u²/a² - v²/b² = 1, where a = semi_real, b = semi_imag.
/// The hyperbola lies in UV space with its center at `center`,
/// its transverse axis along (axis_u, axis_v), and conjugate axis
/// perpendicular to that (rotated 90° CCW).
///
/// Parametric form:
///   P(t) = center + a·cosh(t)·(axis_u, axis_v) + b·sinh(t)·(-axis_v, axis_u)
///   P'(t) = a·sinh(t)·(axis_u, axis_v) + b·cosh(t)·(-axis_v, axis_u)
///
/// The parameter t ∈ [t_start, t_end] maps to the trimmed portion.
/// STEP TRIMMED_CURVE provides the bounds.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Hyperbola2d {
    /// Center of the hyperbola in UV space.
    pub center: Point2d,
    /// Semi-real axis length (a in u²/a² - v²/b² = 1).
    pub semi_real: f64,
    /// Semi-imaginary axis length (b in u²/a² - v²/b² = 1).
    pub semi_imag: f64,
    /// Direction of the transverse axis (unit vector in UV space).
    pub axis_u: f64,
    pub axis_v: f64,
    /// Start parameter value (typically from TRIMMED_CURVE).
    pub t_start: f64,
    /// End parameter value (typically from TRIMMED_CURVE).
    pub t_end: f64,
}

impl Hyperbola2d {
    /// Create a full hyperbola with given parameter range.
    pub fn new(center: Point2d, semi_real: f64, semi_imag: f64, axis_u: f64, axis_v: f64, t_start: f64, t_end: f64) -> Self {
        Self { center, semi_real, semi_imag, axis_u, axis_v, t_start, t_end }
    }

    /// Evaluate at parameter t ∈ [0, 1] (maps to [t_start, t_end]).
    pub fn point_at(&self, t: f64) -> Point2d {
        let s = self.t_start + t * (self.t_end - self.t_start);
        let ch = s.cosh();
        let sh = s.sinh();
        // Conjugate axis direction is 90° CCW from transverse axis
        let conj_u = -self.axis_v;
        let conj_v = self.axis_u;
        Point2d::new(
            self.center.u + self.semi_real * ch * self.axis_u + self.semi_imag * sh * conj_u,
            self.center.v + self.semi_real * ch * self.axis_v + self.semi_imag * sh * conj_v,
        )
    }

    /// Derivative at parameter t ∈ [0, 1].
    pub fn derivative_at(&self, t: f64) -> (f64, f64) {
        let dt = self.t_end - self.t_start;
        let s = self.t_start + t * dt;
        let sh = s.sinh();
        let ch = s.cosh();
        let conj_u = -self.axis_v;
        let conj_v = self.axis_u;
        let du = (self.semi_real * sh * self.axis_u + self.semi_imag * ch * conj_u) * dt;
        let dv = (self.semi_real * sh * self.axis_v + self.semi_imag * ch * conj_v) * dt;
        (du, dv)
    }

    /// Parameter range is always [0, 1] (canonical).
    pub fn param_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    /// Approximate arc length using numerical integration.
    pub fn length(&self) -> f64 {
        let n = 100;
        let mut length = 0.0;
        let mut prev = self.point_at(0.0);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let curr = self.point_at(t);
            let du = curr.u - prev.u;
            let dv = curr.v - prev.v;
            length += (du * du + dv * dv).sqrt();
            prev = curr;
        }
        length
    }

    /// Closest-point projection of a UV point onto the hyperbolic arc
    /// (Vision 2036 §1.3 «analytical projections for PCURVE»).
    ///
    /// Returns `(t, dist)`: the canonical parameter `t ∈ [0, 1]` (mapped
    /// through `[t_start, t_end]`) of the closest point and the Euclidean
    /// distance from `p` to it — via `project_parametric_curve`.
    pub fn project_point(&self, p: &Point2d) -> (f64, f64) {
        project_parametric_curve(p, 0.0, 1.0, &|t| self.point_at(t), &|t| self.derivative_at(t))
    }
}

/// A parabolic arc in UV space.
///
/// Standard form: u = v²/(4f), where f = focal_dist.
/// The parabola opens along (axis_u, axis_v) direction, with vertex at `vertex`.
///
/// Parametric form (parameter t = coordinate along conjugate axis):
///   P(t) = vertex + (t²/(4f))·(axis_u, axis_v) + t·(-axis_v, axis_u)
///   P'(t) = (t/(2f))·(axis_u, axis_v) + (-axis_v, axis_u)
///
/// The parameter t ∈ [t_start, t_end] maps to the trimmed portion.
/// STEP TRIMMED_CURVE provides the bounds.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Parabola2d {
    /// Vertex of the parabola in UV space.
    pub vertex: Point2d,
    /// Focal distance f > 0.
    pub focal_dist: f64,
    /// Direction the parabola opens (unit vector in UV space).
    pub axis_u: f64,
    pub axis_v: f64,
    /// Start parameter value (typically from TRIMMED_CURVE).
    pub t_start: f64,
    /// End parameter value (typically from TRIMMED_CURVE).
    pub t_end: f64,
}

impl Parabola2d {
    /// Create a parabola with given parameter range.
    pub fn new(vertex: Point2d, focal_dist: f64, axis_u: f64, axis_v: f64, t_start: f64, t_end: f64) -> Self {
        Self { vertex, focal_dist, axis_u, axis_v, t_start, t_end }
    }

    /// Evaluate at parameter t ∈ [0, 1] (maps to [t_start, t_end]).
    pub fn point_at(&self, t: f64) -> Point2d {
        let s = self.t_start + t * (self.t_end - self.t_start);
        let f = if self.focal_dist.abs() < 1e-15 { 1e-15 } else { self.focal_dist };
        let along = s * s / (4.0 * f);
        // Conjugate direction is 90° CCW from axis direction
        let conj_u = -self.axis_v;
        let conj_v = self.axis_u;
        Point2d::new(
            self.vertex.u + along * self.axis_u + s * conj_u,
            self.vertex.v + along * self.axis_v + s * conj_v,
        )
    }

    /// Derivative at parameter t ∈ [0, 1].
    pub fn derivative_at(&self, t: f64) -> (f64, f64) {
        let dt = self.t_end - self.t_start;
        let s = self.t_start + t * dt;
        let f = if self.focal_dist.abs() < 1e-15 { 1e-15 } else { self.focal_dist };
        let d_along = s / (2.0 * f);
        let conj_u = -self.axis_v;
        let conj_v = self.axis_u;
        let du = (d_along * self.axis_u + conj_u) * dt;
        let dv = (d_along * self.axis_v + conj_v) * dt;
        (du, dv)
    }

    /// Parameter range is always [0, 1] (canonical).
    pub fn param_range(&self) -> (f64, f64) {
        (0.0, 1.0)
    }

    /// Approximate arc length using numerical integration.
    pub fn length(&self) -> f64 {
        let n = 100;
        let mut length = 0.0;
        let mut prev = self.point_at(0.0);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let curr = self.point_at(t);
            let du = curr.u - prev.u;
            let dv = curr.v - prev.v;
            length += (du * du + dv * dv).sqrt();
            prev = curr;
        }
        length
    }

    /// Closest-point projection of a UV point onto the parabolic arc
    /// (Vision 2036 §1.3 «analytical projections for PCURVE»).
    ///
    /// Returns `(t, dist)`: the canonical parameter `t ∈ [0, 1]` (mapped
    /// through `[t_start, t_end]`) of the closest point and the Euclidean
    /// distance from `p` to it — via `project_parametric_curve`.
    pub fn project_point(&self, p: &Point2d) -> (f64, f64) {
        project_parametric_curve(p, 0.0, 1.0, &|t| self.point_at(t), &|t| self.derivative_at(t))
    }
}

/// A NURBS curve in UV space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Nurbs2d {
    /// Degree of the NURBS curve.
    pub degree: usize,
    /// 2D control points in UV space.
    pub control_points: Vec<Point2d>,
    /// Weights for rational NURBS.
    pub weights: Vec<f64>,
    /// Knot vector.
    pub knots: Vec<f64>,
}

impl Nurbs2d {
    /// Parameter range.
    pub fn param_range(&self) -> (f64, f64) {
        let p = self.degree;
        if self.knots.len() > p {
            (self.knots[p], self.knots[self.knots.len() - p - 1])
        } else {
            (0.0, 1.0)
        }
    }

    /// Evaluate at parameter t using de Boor's algorithm.
    pub fn point_at(&self, t: f64) -> Point2d {
        let n = self.control_points.len();
        if n == 0 {
            return Point2d::ORIGIN;
        }
        if n == 1 {
            let w = self.weights.get(0).copied().unwrap_or(1.0);
            if w.abs() < 1e-15 {
                return Point2d::ORIGIN;
            }
            return Point2d::new(self.control_points[0].u, self.control_points[0].v);
        }

        let p = self.degree;

        // Clamp to valid knot range
        let (t_min, t_max) = self.param_range();
        let t = t.clamp(t_min, t_max);

        // Find knot span
        let k = find_knot_span_2d(&self.knots, p, t, n);

        // De Boor's algorithm
        let mut pts: Vec<Point2d> = Vec::with_capacity(p + 1);
        let mut wts: Vec<f64> = Vec::with_capacity(p + 1);

        for i in 0..=p {
            let idx = k - p + i;
            if idx < n {
                pts.push(Point2d::new(
                    self.control_points[idx].u * self.weights[idx],
                    self.control_points[idx].v * self.weights[idx],
                ));
                wts.push(self.weights[idx]);
            } else {
                pts.push(Point2d::new(0.0, 0.0));
                wts.push(1.0);
            }
        }

        for r in 1..=p {
            for j in (r..=p).rev() {
                let i = k - p + j;
                let alpha = if i + p + 1 - r < self.knots.len() && i < self.knots.len() {
                    let denom = self.knots[i + p + 1 - r] - self.knots[i];
                    if denom.abs() < 1e-15 { 0.0 } else { (t - self.knots[i]) / denom }
                } else {
                    0.0
                };

                let beta = 1.0 - alpha;
                pts[j] = Point2d::new(
                    alpha * pts[j].u + beta * pts[j - 1].u,
                    alpha * pts[j].v + beta * pts[j - 1].v,
                );
                wts[j] = alpha * wts[j] + beta * wts[j - 1];
            }
        }

        if wts[p].abs() < 1e-15 {
            Point2d::new(0.0, 0.0)
        } else {
            Point2d::new(pts[p].u / wts[p], pts[p].v / wts[p])
        }
    }

    /// Derivative at parameter t — analytical (Vision 2036 §1.3
    /// «analytical derivatives for PCURVE»).
    ///
    /// Uses the quotient rule for rational B-splines:
    ///   C(t) = A(t) / w(t)
    ///   C'(t) = (A'(t) − C(t)·w'(t)) / w(t)
    /// with derivative control points from Piegl & Tiller («The NURBS
    /// Book», Algorithm A2.5 / Eq. 3.7):
    ///   Q_j = p·(P_{j+1} − P_j) / (u_{j+p+1} − u_{j+1}),  j = 0..n−2
    /// — the same index-corrected form as the 3D `NurbsCurve::derivative_at`
    /// (validated by the periodic uniform-knot SSI curves, «C2-периодичность
    /// шва»). Falls back to central differences only when the analytical
    /// result is non-finite (malformed-data guard, §1.5 philosophy).
    pub fn derivative_at(&self, t: f64) -> (f64, f64) {
        let analytic = self.derivative_at_analytic(t);
        if analytic.0.is_finite() && analytic.1.is_finite() {
            return analytic;
        }
        // §1.5 guard: numerical fallback for malformed knots/weights.
        let eps = 1e-7;
        let p0 = self.point_at(t - eps);
        let p1 = self.point_at(t + eps);
        ((p1.u - p0.u) / (2.0 * eps), (p1.v - p0.v) / (2.0 * eps))
    }

    /// Analytical rational B-spline derivative (quotient rule).
    /// Returns `(NaN, NaN)` for structurally malformed input so that the
    /// public `derivative_at` can take the numerical fallback.
    fn derivative_at_analytic(&self, t: f64) -> (f64, f64) {
        let n = self.control_points.len();
        if n < 2 {
            return (0.0, 0.0);
        }
        let p = self.degree;
        if p == 0 || self.knots.len() < n + p + 1 {
            return (f64::NAN, f64::NAN);
        }
        let (t_min, t_max) = self.param_range();
        let t_c = t.clamp(t_min, t_max);
        let k = find_knot_span_2d(&self.knots, p, t_c, n);

        // p+1 weighted control points (u·w, v·w, w) of the local span.
        // `pts` stays ORIGINAL (the derivative control points below are
        // differences of these); the de Boor evaluation runs on a copy.
        let mut pts: Vec<(f64, f64, f64)> = Vec::with_capacity(p + 1);
        for i in 0..=p {
            let idx = k - p + i;
            let idx = if idx >= n { n - 1 } else { idx };
            let w = self.weights.get(idx).copied().unwrap_or(1.0);
            let cp = &self.control_points[idx];
            pts.push((cp.u * w, cp.v * w, w));
        }

        // A(t), w(t) via de Boor (degree p, span k).
        let mut pts_eval = pts.clone();
        de_boor_step_2d(&mut pts_eval, &self.knots, p, k, t_c);
        let (au, av, w) = match pts_eval.last() {
            Some(&r) => r,
            None => return (f64::NAN, f64::NAN),
        };
        if w.abs() < 1e-15 {
            return (f64::NAN, f64::NAN);
        }
        let cu = au / w;
        let cv = av / w;

        // Derivative control points Q_j (degree p−1), j = k−p .. k−1:
        //   Q_j = p·(P_{j+1} − P_j) / (u_{j+p+1} − u_{j+1})
        // The span-k de Boor recursion below with the ORIGINAL knot vector
        // is index-equivalent to the standard degree-(p−1) evaluation on
        // the shifted knot vector with span k−1 (see the 3D twin).
        let mut dpts: Vec<(f64, f64, f64)> = Vec::with_capacity(p);
        for i in 0..p {
            let j = k - p + i;
            let denom = if j + p + 1 < self.knots.len() && j + 1 < self.knots.len() {
                let d = self.knots[j + p + 1] - self.knots[j + 1];
                if d.abs() < 1e-15 { 0.0 } else { p as f64 / d }
            } else {
                0.0
            };
            let next = pts[i + 1];
            let curr = pts[i];
            dpts.push((
                denom * (next.0 - curr.0),
                denom * (next.1 - curr.1),
                denom * (next.2 - curr.2),
            ));
        }

        // A'(t), w'(t) via de Boor on the derivative control points.
        if p > 1 {
            de_boor_step_2d(&mut dpts, &self.knots, p - 1, k, t_c);
        }
        let (dau, dav, dw) = match dpts.last() {
            Some(&r) => r,
            None => return (f64::NAN, f64::NAN),
        };

        // C'(t) = (A'(t) − C(t)·w'(t)) / w(t)
        let du = (dau - cu * dw) / w;
        let dv = (dav - cv * dw) / w;
        (du, dv)
    }

    /// Approximate arc length using numerical integration.
    pub fn length(&self) -> f64 {
        let (t_min, t_max) = self.param_range();
        let n = 100;
        let mut length = 0.0;
        let mut prev = self.point_at(t_min);
        for i in 1..=n {
            let t = t_min + (t_max - t_min) * i as f64 / n as f64;
            let curr = self.point_at(t);
            let du = curr.u - prev.u;
            let dv = curr.v - prev.v;
            length += (du * du + dv * dv).sqrt();
            prev = curr;
        }
        length
    }

    /// Closest-point projection of a UV point onto the NURBS curve
    /// (Vision 2036 §1.3 «analytical projections for PCURVE»).
    ///
    /// Returns `(t, dist)`: the knot-space parameter (within
    /// `param_range()`) of the closest point and the Euclidean distance
    /// from `p` to it. Uses the robust scan → golden-section →
    /// orthogonal-polish pipeline with the ANALYTICAL derivative
    /// (`derivative_at`) in the polish step.
    pub fn project_point(&self, p: &Point2d) -> (f64, f64) {
        let (t_min, t_max) = self.param_range();
        project_parametric_curve(p, t_min, t_max, &|t| self.point_at(t), &|t| self.derivative_at(t))
    }
}

/// Find the knot span for a given parameter value.
fn find_knot_span_2d(knots: &[f64], degree: usize, t: f64, n: usize) -> usize {
    // Binary search for knot span
    let p = degree;
    if t >= knots[n] { return n - 1; }
    if t <= knots[p] { return p; }

    let mut lo = p;
    let mut hi = n;
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// de Boor recursion for 2D weighted control points `(u·w, v·w, w)`
/// — the 2D twin of `de_boor_step_curve` (curve.rs, truck-geometry-
/// adapted). Evaluates the local span in place: on return, the last entry
/// holds `(A_u(t), A_v(t), w(t))`. Used by the analytical
/// `Nurbs2d::derivative_at` (Vision 2036 §1.3).
fn de_boor_step_2d(pts: &mut [(f64, f64, f64)], knots: &[f64], degree: usize, k: usize, t: f64) {
    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let i = k - degree + j;
            let alpha = if i + degree + 1 - r < knots.len() && i < knots.len() {
                let denom = knots[i + degree + 1 - r] - knots[i];
                if denom.abs() < 1e-15 { 0.0 } else { (t - knots[i]) / denom }
            } else {
                0.0
            };
            let beta = 1.0 - alpha;
            pts[j] = (
                alpha * pts[j].0 + beta * pts[j - 1].0,
                alpha * pts[j].1 + beta * pts[j - 1].1,
                alpha * pts[j].2 + beta * pts[j - 1].2,
            );
        }
    }
}

impl Curve2d {
    /// For a Composite curve, find which segment and local parameter
    /// correspond to the given global parameter `t ∈ [0, 1]`.
    fn composite_segment_at(&self, t: f64) -> (usize, f64) {
        if let Curve2d::Composite { segments, cum_lengths } = self {
            if segments.is_empty() || cum_lengths.is_empty() {
                return (0, t);
            }
            let t = t.clamp(0.0, 1.0);
            let mut seg_idx = 0;
            for (i, &cum) in cum_lengths.iter().enumerate() {
                if t <= cum || i == cum_lengths.len() - 1 {
                    seg_idx = i;
                    break;
                }
            }
            let t_start = if seg_idx == 0 { 0.0 } else { cum_lengths[seg_idx - 1] };
            let t_end = cum_lengths[seg_idx];
            let seg_span = t_end - t_start;
            let local_frac = if seg_span > 1e-15 {
                (t - t_start) / seg_span
            } else {
                0.5
            };
            let (p_min, p_max) = segments[seg_idx].param_range();
            let local_t = p_min + local_frac * (p_max - p_min);
            (seg_idx, local_t)
        } else {
            (0, t)
        }
    }

    /// Evaluate the curve at parameter t.
    pub fn point_at(&self, t: f64) -> Point2d {
        match self {
            Curve2d::Line(l) => l.point_at(t),
            Curve2d::Circle(c) => c.point_at(t),
            Curve2d::Ellipse(e) => e.point_at(t),
            Curve2d::Hyperbola(h) => h.point_at(t),
            Curve2d::Parabola(p) => p.point_at(t),
            Curve2d::Nurbs(n) => n.point_at(t),
            Curve2d::Composite { .. } => {
                let (seg_idx, local_t) = self.composite_segment_at(t);
                if let Curve2d::Composite { segments, .. } = self {
                    segments[seg_idx].point_at(local_t)
                } else {
                    Point2d::new(0.0, 0.0)
                }
            }
        }
    }

    /// Derivative at parameter t.
    pub fn derivative_at(&self, t: f64) -> (f64, f64) {
        match self {
            Curve2d::Line(l) => l.derivative_at(t),
            Curve2d::Circle(c) => c.derivative_at(t),
            Curve2d::Ellipse(e) => e.derivative_at(t),
            Curve2d::Hyperbola(h) => h.derivative_at(t),
            Curve2d::Parabola(p) => p.derivative_at(t),
            Curve2d::Nurbs(n) => n.derivative_at(t),
            Curve2d::Composite { .. } => {
                let (seg_idx, local_t) = self.composite_segment_at(t);
                if let Curve2d::Composite { segments, cum_lengths } = self {
                    let seg = &segments[seg_idx];
                    let (du, dv) = seg.derivative_at(local_t);
                    let t_start = if seg_idx == 0 { 0.0 } else { cum_lengths[seg_idx - 1] };
                    let t_end = cum_lengths[seg_idx];
                    let seg_span = t_end - t_start;
                    let (p_min, p_max) = seg.param_range();
                    let param_span = p_max - p_min;
                    let scale = if param_span > 1e-15 && seg_span > 1e-15 {
                        param_span / seg_span
                    } else {
                        1.0
                    };
                    (du * scale, dv * scale)
                } else {
                    (0.0, 0.0)
                }
            }
        }
    }

    /// Parameter range.
    pub fn param_range(&self) -> (f64, f64) {
        match self {
            Curve2d::Line(l) => l.param_range(),
            Curve2d::Circle(c) => c.param_range(),
            Curve2d::Ellipse(e) => e.param_range(),
            Curve2d::Hyperbola(h) => h.param_range(),
            Curve2d::Parabola(p) => p.param_range(),
            Curve2d::Nurbs(n) => n.param_range(),
            Curve2d::Composite { .. } => (0.0, 1.0),
        }
    }

    /// Arc length.
    pub fn length(&self) -> f64 {
        match self {
            Curve2d::Line(l) => l.length(),
            Curve2d::Circle(c) => c.length(),
            Curve2d::Ellipse(e) => e.length(),
            Curve2d::Hyperbola(h) => h.length(),
            Curve2d::Parabola(p) => p.length(),
            Curve2d::Nurbs(n) => n.length(),
            Curve2d::Composite { segments, .. } => {
                segments.iter().map(|s| s.length()).sum()
            }
        }
    }

    /// Sample the curve at n_samples points (including endpoints).
    pub fn sample(&self, n_samples: usize) -> Vec<Point2d> {
        if n_samples == 0 {
            return vec![];
        }
        if n_samples == 1 {
            return vec![self.point_at(0.0)];
        }
        let (t_min, t_max) = self.param_range();
        (0..n_samples)
            .map(|i| {
                let t = t_min + (t_max - t_min) * i as f64 / (n_samples - 1) as f64;
                self.point_at(t)
            })
            .collect()
    }

    /// Closest-point projection of a UV-space point onto this curve
    /// (Vision 2036 §1.3 «analytical projections for PCURVE»).
    ///
    /// Returns `(t, dist)` where `t` is the curve parameter — in THIS
    /// curve's `param_range()`; `Line/Circle/Ellipse/Hyperbola/Parabola`
    /// and `Composite` use the canonical `[0, 1]`, `Nurbs` uses its knot
    /// range — of the closest curve point `C(t)`, and `dist` is the
    /// Euclidean UV distance `|p − C(t)|`.
    ///
    /// Per-variant strategy:
    /// * `Line` — exact clamped dot-product projection;
    /// * `Circle` — exact `atan2` angular projection (wrap-aware for full
    ///   circles, endpoint-guarded for arcs);
    /// * `Ellipse/Hyperbola/Parabola/Nurbs` — uniform scan brackets the
    ///   global minimum, golden-section shrink refines the bracket,
    ///   orthogonal-projection polish (with the analytical derivative)
    ///   pins the stationary point: `(p − C(t))·C'(t) ≈ 0`;
    /// * `Composite` — per-segment projection, best segment wins; the
    ///   local parameter is mapped back through the composite
    ///   parameterization (inverse of `composite_segment_at`).
    ///
    /// Typical uses: snapping noisy UV samples onto the exact PCURVE,
    /// re-associating edge endpoints during healing, and deviation
    /// checks against a tolerance gate.
    pub fn project_point(&self, point: &Point2d) -> (f64, f64) {
        match self {
            Curve2d::Line(l) => l.project_point(point),
            Curve2d::Circle(c) => c.project_point(point),
            Curve2d::Ellipse(e) => e.project_point(point),
            Curve2d::Hyperbola(h) => h.project_point(point),
            Curve2d::Parabola(p) => p.project_point(point),
            Curve2d::Nurbs(n) => n.project_point(point),
            Curve2d::Composite { segments, cum_lengths } => {
                if segments.is_empty() {
                    return (0.0, f64::INFINITY);
                }
                let mut best: Option<(f64, f64)> = None; // (global t, dist)
                for (i, seg) in segments.iter().enumerate() {
                    let (t_local, d) = seg.project_point(point);
                    // Inverse of `composite_segment_at`: map the segment's
                    // local parameter back to the composite parameter.
                    let cum_start = if i == 0 { 0.0 } else { cum_lengths[i - 1] };
                    let cum_end = cum_lengths.get(i).copied().unwrap_or(1.0);
                    let seg_span = cum_end - cum_start;
                    let (p_min, p_max) = seg.param_range();
                    let param_span = p_max - p_min;
                    let t_global = if seg_span > 1e-15 && param_span > 1e-15 {
                        cum_start + (t_local - p_min) / param_span * seg_span
                    } else {
                        // Degenerate segment: the mid parameter is well-defined.
                        0.5 * (cum_start + cum_end)
                    }
                    .clamp(0.0, 1.0);
                    if best.map(|(_, bd)| d < bd).unwrap_or(true) {
                        best = Some((t_global, d));
                    }
                }
                best.unwrap_or((0.0, f64::INFINITY))
            }
        }
    }

    /// Euclidean distance from a UV point to the closest point of the
    /// curve (convenience wrapper over `project_point`).
    pub fn distance_to(&self, point: &Point2d) -> f64 {
        self.project_point(point).1
    }
}

/// Euclidean distance between two UV points.
#[inline]
fn uv_distance(p: &Point2d, q: &Point2d) -> f64 {
    let du = p.u - q.u;
    let dv = p.v - q.v;
    (du * du + dv * dv).sqrt()
}

/// Robust closest-point projection of `p` onto a parametric curve
/// `C: [t_min, t_max] → UV`, given evaluators for `C(t)` and `C'(t)`
/// (Vision 2036 §1.3 «analytical projections for PCURVE» — generic
/// engine behind `Ellipse/Hyperbola/Parabola/Nurbs` projections).
///
/// Pipeline:
/// 1. Uniform scan (48 samples including both endpoints) brackets the
///    global minimum of `|p − C(t)|` — immune to non-unimodal distance
///    profiles (inflection, flat spans, far-away query points).
/// 2. Golden-section shrink of the bracketing interval (~60 evaluations)
///    — derivative-free, converges linearly but unconditionally within
///    the bracket.
/// 3. Orthogonal-projection polish: `t ← t + ((p − C)·C')/|C'|²` with
///    accept-if-improves step halving and clamping to the domain (8
///    steps) — pins the exact stationary point to machine precision.
///
/// Returns `(t*, dist)` — the minimizing parameter and the Euclidean
/// distance `|p − C(t*)|`. No panics: non-finite derivative components
/// are treated as zero, degenerate spans return the endpoint projection.
fn project_parametric_curve(
    p: &Point2d,
    t_min: f64,
    t_max: f64,
    point_at: &dyn Fn(f64) -> Point2d,
    deriv_at: &dyn Fn(f64) -> (f64, f64),
) -> (f64, f64) {
    let span = t_max - t_min;
    if !span.is_finite() || span <= 0.0 {
        return (t_min, uv_distance(p, &point_at(t_min)));
    }

    // 1) Uniform scan — brackets the global minimum.
    const N_SCAN: usize = 48;
    let mut best_i = 0usize;
    let mut best_d = f64::INFINITY;
    for i in 0..=N_SCAN {
        let t = t_min + span * (i as f64) / (N_SCAN as f64);
        let d = uv_distance(p, &point_at(t));
        if d < best_d {
            best_d = d;
            best_i = i;
        }
    }
    let _ = best_d; // Read below only via the golden-section seeds.

    // 2) Golden-section shrink of the bracketing interval.
    const INV_PHI: f64 = 0.6180339887498949; // (√5 − 1)/2
    let sample_t = |i: usize| t_min + span * (i as f64) / (N_SCAN as f64);
    let mut a = if best_i == 0 { t_min } else { sample_t(best_i - 1) };
    let mut b = if best_i == N_SCAN { t_max } else { sample_t(best_i + 1) };
    let mut x = b - INV_PHI * (b - a);
    let mut y = a + INV_PHI * (b - a);
    let mut fx = uv_distance(p, &point_at(x));
    let mut fy = uv_distance(p, &point_at(y));
    for _ in 0..60 {
        if (b - a) <= 1e-13 * span {
            break;
        }
        if fx <= fy {
            b = y;
            y = x;
            fy = fx;
            x = b - INV_PHI * (b - a);
            fx = uv_distance(p, &point_at(x));
        } else {
            a = x;
            x = y;
            fx = fy;
            y = a + INV_PHI * (b - a);
            fy = uv_distance(p, &point_at(y));
        }
    }
    // The golden-section result seeds the polish phase below; the final
    // distance is recomputed from the polished parameter.
    let mut best_t = if fx <= fy { x } else { y };

    // 3) Orthogonal-projection polish with the (analytical) derivative.
    //    Objective: the stationarity residual (p − C(t))·C'(t) → 0 — NOT
    //    the distance value, whose acceptance stalls at a √ulp-level
    //    parameter error because the distance is second-order flat at the
    //    optimum (a distance-based gate cannot distinguish t-errors below
    //    ~1e-8). Accept a step iff the residual magnitude shrinks; the
    //    golden-section result guarantees we start inside the global
    //    minimum's basin, so the polish only travels the last ~1e-8.
    for _ in 0..12 {
        let c = point_at(best_t);
        let (mut du, mut dv) = deriv_at(best_t);
        if !du.is_finite() || !dv.is_finite() {
            du = 0.0;
            dv = 0.0;
        }
        let g2 = du * du + dv * dv;
        if g2 < 1e-30 {
            break; // Stationary derivative — nothing to polish.
        }
        let dot = (p.u - c.u) * du + (p.v - c.v) * dv;
        // Converged: residual orthogonal, scaled by curve speed and
        // query distance (dot has units of length²).
        let scale = g2.sqrt() * (1.0 + uv_distance(p, &c));
        if dot.abs() < 1e-13 * scale {
            break;
        }
        let step = dot / g2;
        if !step.is_finite() || step.abs() < 1e-16 * (1.0 + span) {
            break;
        }
        // Full step, then halvings; accept the first that shrinks the
        // residual magnitude (boundary-clamped steps that return the same
        // point are rejected → the loop terminates at the boundary).
        let mut s = step;
        let mut accepted = false;
        for _ in 0..4 {
            let t_try = (best_t + s).clamp(t_min, t_max);
            let c_try = point_at(t_try);
            let (mut tu, mut tv) = deriv_at(t_try);
            if !tu.is_finite() || !tv.is_finite() {
                tu = 0.0;
                tv = 0.0;
            }
            let g2t = tu * tu + tv * tv;
            let dot_try = if g2t < 1e-30 {
                f64::INFINITY // Cannot evaluate the residual — reject.
            } else {
                (p.u - c_try.u) * tu + (p.v - c_try.v) * tv
            };
            if dot_try.abs() < dot.abs() {
                best_t = t_try;
                accepted = true;
                break;
            }
            s *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    let best_d = uv_distance(p, &point_at(best_t));
    (best_t, best_d)
}

/// Derive a PCURVE (Curve2d) from a 3D curve and a surface.
///
/// Per ROADMAP_VISION_2036 §2.2: When no analytical PCURVE is available
/// from the STEP file, derive one by projecting the 3D curve's sample
/// points onto the surface's UV space.
///
/// For simple cases (line on plane, circle on cylinder), this returns
/// an analytical Curve2d. For complex cases (NURBS on NURBS), it
/// returns a Nurbs2d fitted to the projected UV points.
///
/// Returns None if projection fails for any sample point.
pub fn derive_pcurve(
    curve_3d: &crate::Curve3d,
    surface: &crate::Surface,
    n_samples: usize,
) -> Option<Curve2d> {
    let n = n_samples.max(4).min(64);

    // Sample the 3D curve
    let (t_min, t_max) = curve_3d.param_range();
    let mut points_3d = Vec::with_capacity(n);
    let mut params = Vec::with_capacity(n);
    for i in 0..n {
        let t = t_min + (t_max - t_min) * i as f64 / (n - 1) as f64;
        points_3d.push(curve_3d.point_at(t));
        params.push(t);
    }

    // Project each 3D point onto the surface's UV space
    let mut uv_points = Vec::with_capacity(n);
    for p in &points_3d {
        let (u, v) = surface.project_point(p);
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        uv_points.push(Point2d::new(u, v));
    }

    // Try to detect simple analytical cases
    // Check if UV points form a line (all collinear within tolerance)
    if n >= 2 {
        let is_line = check_collinear(&uv_points, 1e-6);
        if is_line {
            return Some(Curve2d::Line(Line2d::new(uv_points[0], uv_points[n - 1])));
        }
    }

    // Check if UV points form a circle (constant radius from a center)
    if n >= 4 {
        if let Some(circle) = check_circular(&uv_points) {
            return Some(Curve2d::Circle(circle));
        }
    }

    // Fall back to Nurbs2d: fit a B-spline curve in UV space
    // using the projected points as control points
    let degree = 3.min(n - 1);
    let control_points = uv_points.clone();
    let weights = vec![1.0; control_points.len()];
    let n_cp = control_points.len();
    let n_knots = n_cp + degree + 1;
    let mut knots = vec![0.0; n_knots];
    for i in 0..n_knots {
        if i <= degree {
            knots[i] = 0.0;
        } else if i >= n_cp {
            knots[i] = 1.0;
        } else {
            knots[i] = (i - degree) as f64 / (n_cp - degree) as f64;
        }
    }

    Some(Curve2d::Nurbs(Nurbs2d {
        degree,
        control_points,
        weights,
        knots,
    }))
}

/// Check if a set of 2D points are collinear within tolerance.
fn check_collinear(points: &[Point2d], tol: f64) -> bool {
    if points.len() < 2 {
        return true;
    }
    let p0 = points[0];
    let p1 = points[points.len() - 1];
    let dx = p1.u - p0.u;
    let dy = p1.v - p0.v;
    let len = (dx * dx + dy * dy).sqrt();
    if len < tol {
        return false;
    }
    let nx = -dy / len;
    let ny = dx / len;
    for p in points {
        let dist = ((p.u - p0.u) * nx + (p.v - p0.v) * ny).abs();
        if dist > tol {
            return false;
        }
    }
    true
}

/// Check if 2D points form a circle. Returns the circle if detected.
fn check_circular(points: &[Point2d]) -> Option<Circle2d> {
    if points.len() < 4 {
        return None;
    }
    // Compute centroid as center estimate
    let mut cu = 0.0;
    let mut cv = 0.0;
    for p in points {
        cu += p.u;
        cv += p.v;
    }
    cu /= points.len() as f64;
    cv /= points.len() as f64;

    // Check if all points are equidistant from center
    let mut radii = Vec::with_capacity(points.len());
    for p in points {
        let du = p.u - cu;
        let dv = p.v - cv;
        radii.push((du * du + dv * dv).sqrt());
    }
    let avg_r = radii.iter().sum::<f64>() / radii.len() as f64;
    let max_dev = radii.iter().map(|r| (r - avg_r).abs()).fold(0.0_f64, f64::max);
    if max_dev > avg_r * 0.01 {
        return None; // Not circular
    }

    // Determine start/end angles
    let start_angle = {
        let du = points[0].u - cu;
        let dv = points[0].v - cv;
        dv.atan2(du)
    };
    let end_angle = {
        let du = points[points.len() - 1].u - cu;
        let dv = points[points.len() - 1].v - cv;
        dv.atan2(du)
    };

    Some(Circle2d {
        center: Point2d::new(cu, cv),
        radius: avg_r,
        start_angle,
        end_angle,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line2d() {
        let line = Line2d::new(Point2d::new(0.0, 0.0), Point2d::new(1.0, 2.0));
        let p = line.point_at(0.5);
        assert!((p.u - 0.5).abs() < 1e-10);
        assert!((p.v - 1.0).abs() < 1e-10);
        assert!((line.length() - 5.0_f64.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_circle2d() {
        let circle = Circle2d::new_full(Point2d::new(0.0, 0.0), 1.0);
        let p0 = circle.point_at(0.0);
        assert!((p0.u - 1.0).abs() < 1e-10);
        assert!(p0.v.abs() < 1e-10);
        assert!((circle.length() - 2.0 * PI).abs() < 1e-6);
    }

    #[test]
    fn test_ellipse2d() {
        let ellipse = Ellipse2d::new_full(Point2d::new(0.0, 0.0), 2.0, 1.0, 0.0);
        let p0 = ellipse.point_at(0.0);
        assert!((p0.u - 2.0).abs() < 1e-10, "Expected u=2.0, got {}", p0.u);
        assert!(p0.v.abs() < 1e-10, "Expected v=0.0, got {}", p0.v);
        // Circumference of ellipse with a=2, b=1 is approximately 9.688
        let len = ellipse.length();
        assert!(len > 9.0 && len < 10.5, "Expected ~9.688, got {}", len);
    }

    #[test]
    fn test_nurbs2d_line() {
        // A NURBS that represents a straight line from (0,0) to (1,1)
        let nurbs = Nurbs2d {
            degree: 1,
            control_points: vec![Point2d::new(0.0, 0.0), Point2d::new(1.0, 1.0)],
            weights: vec![1.0, 1.0],
            knots: vec![0.0, 0.0, 1.0, 1.0],
        };
        let p = nurbs.point_at(0.5);
        assert!((p.u - 0.5).abs() < 1e-10);
        assert!((p.v - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_curve2d_dispatch() {
        let curve = Curve2d::Line(Line2d::new(Point2d::new(0.0, 0.0), Point2d::new(2.0, 0.0)));
        let p = curve.point_at(0.5);
        assert!((p.u - 1.0).abs() < 1e-10);
        assert!(p.v.abs() < 1e-10);
        assert!((curve.length() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_circle2d_arc() {
        // Quarter circle from 0 to π/2
        let arc = Circle2d::new_arc(Point2d::new(0.0, 0.0), 1.0, 0.0, PI / 2.0);
        let p_start = arc.point_at(0.0);
        let p_end = arc.point_at(1.0);
        assert!((p_start.u - 1.0).abs() < 1e-10, "Start point u should be 1.0");
        assert!(p_start.v.abs() < 1e-10, "Start point v should be 0.0");
        assert!(p_end.u.abs() < 1e-10, "End point u should be 0.0");
        assert!((p_end.v - 1.0).abs() < 1e-10, "End point v should be 1.0");
        assert!((arc.length() - PI / 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_curve2d_sample() {
        let curve = Curve2d::Line(Line2d::new(Point2d::new(0.0, 0.0), Point2d::new(10.0, 0.0)));
        let samples = curve.sample(11);
        assert_eq!(samples.len(), 11);
        assert!((samples[0].u - 0.0).abs() < 1e-10);
        assert!((samples[5].u - 5.0).abs() < 1e-10);
        assert!((samples[10].u - 10.0).abs() < 1e-10);
    }

    // ── Hyperbola2d tests ─────────────────────────────────────

    #[test]
    fn test_hyperbola2d_point_at_zero() {
        // Hyperbola centered at origin, a=2, b=1, axis along +U.
        // At t=0 (mapped to s = t_start + 0 * (t_end - t_start) = t_start),
        // P(s) = center + a*cosh(s)*axis + b*sinh(s)*conj.
        // With t_start=-1, t_end=1: at canonical t=0.5, s=0.
        // P(0) = (0,0) + 2*cosh(0)*(1,0) + 1*sinh(0)*(0,1) = (2, 0)
        let hyp = Hyperbola2d::new(
            Point2d::new(0.0, 0.0),
            2.0, 1.0,
            1.0, 0.0,  // axis along +U
            -1.0, 1.0, // t ∈ [-1, 1]
        );
        let p = hyp.point_at(0.5); // s = 0
        assert!((p.u - 2.0).abs() < 1e-10, "Expected u=2.0, got {}", p.u);
        assert!(p.v.abs() < 1e-10, "Expected v=0.0, got {}", p.v);
    }

    #[test]
    fn test_hyperbola2d_derivative_at_zero() {
        // At s=0: P'(0) = a*sinh(0)*axis + b*cosh(0)*conj = 0 + 1*(0,1) = (0, dt)
        // dt = t_end - t_start = 2, so P'(0.5) = (0, 2)
        let hyp = Hyperbola2d::new(
            Point2d::new(0.0, 0.0),
            2.0, 1.0,
            1.0, 0.0,
            -1.0, 1.0,
        );
        let (du, dv) = hyp.derivative_at(0.5);
        assert!(du.abs() < 1e-10, "Expected du≈0, got {}", du);
        assert!((dv - 2.0).abs() < 1e-10, "Expected dv=2.0, got {}", dv);
    }

    #[test]
    fn test_hyperbola2d_length_positive() {
        let hyp = Hyperbola2d::new(
            Point2d::new(0.0, 0.0),
            2.0, 1.0,
            1.0, 0.0,
            -1.0, 1.0,
        );
        assert!(hyp.length() > 0.0, "Hyperbola arc length should be positive");
    }

    #[test]
    fn test_hyperbola2d_rotated_axis() {
        // Hyperbola with axis rotated 90° (along +V)
        let hyp = Hyperbola2d::new(
            Point2d::new(1.0, 1.0),
            3.0, 2.0,
            0.0, 1.0,  // axis along +V
            -0.5, 0.5,
        );
        let p = hyp.point_at(0.5); // s = 0
        // P(0) = (1,1) + 3*cosh(0)*(0,1) + 2*sinh(0)*(-1,0) = (1, 4)
        assert!((p.u - 1.0).abs() < 1e-10, "Expected u=1.0, got {}", p.u);
        assert!((p.v - 4.0).abs() < 1e-10, "Expected v=4.0, got {}", p.v);
    }

    // ── Parabola2d tests ──────────────────────────────────────

    #[test]
    fn test_parabola2d_point_at_zero() {
        // Parabola with vertex at origin, f=1, axis along +U.
        // At s=0 (canonical t such that s = t_start + t*(t_end-t_start)):
        // P(0) = vertex + 0²/(4f)*axis + 0*conj = vertex
        let par = Parabola2d::new(
            Point2d::new(0.0, 0.0),
            1.0,
            1.0, 0.0,  // axis along +U
            -2.0, 2.0,
        );
        let p = par.point_at(0.5); // s = 0
        assert!(p.u.abs() < 1e-10, "Expected u=0.0, got {}", p.u);
        assert!(p.v.abs() < 1e-10, "Expected v=0.0, got {}", p.v);
    }

    #[test]
    fn test_parabola2d_point_at_nonzero() {
        // Parabola with vertex at origin, f=1, axis along +U.
        // t_start=0, t_end=2. At canonical t=0.5, s=1.
        // P(1) = (0,0) + 1²/(4*1)*(1,0) + 1*(0,1) = (0.25, 1.0)
        let par = Parabola2d::new(
            Point2d::new(0.0, 0.0),
            1.0,
            1.0, 0.0,
            0.0, 2.0,
        );
        let p = par.point_at(0.5); // s = 1
        assert!((p.u - 0.25).abs() < 1e-10, "Expected u=0.25, got {}", p.u);
        assert!((p.v - 1.0).abs() < 1e-10, "Expected v=1.0, got {}", p.v);
    }

    #[test]
    fn test_parabola2d_derivative() {
        // At s=1, f=1: d_along = 1/(2*1) = 0.5
        // P'(1) = (0.5*axis + conj) * dt, where dt = t_end - t_start = 2
        // P'(1) = (0.5*(1,0) + (0,1)) * 2 = (1.0, 2.0)
        let par = Parabola2d::new(
            Point2d::new(0.0, 0.0),
            1.0,
            1.0, 0.0,
            0.0, 2.0,
        );
        let (du, dv) = par.derivative_at(0.5);
        assert!((du - 1.0).abs() < 1e-10, "Expected du=1.0, got {}", du);
        assert!((dv - 2.0).abs() < 1e-10, "Expected dv=2.0, got {}", dv);
    }

    #[test]
    fn test_parabola2d_length_positive() {
        let par = Parabola2d::new(
            Point2d::new(0.0, 0.0),
            1.0,
            1.0, 0.0,
            -2.0, 2.0,
        );
        assert!(par.length() > 0.0, "Parabola arc length should be positive");
    }

    #[test]
    fn test_curve2d_hyperbola_dispatch() {
        let hyp = Hyperbola2d::new(
            Point2d::new(0.0, 0.0),
            2.0, 1.0,
            1.0, 0.0,
            -1.0, 1.0,
        );
        let curve = Curve2d::Hyperbola(hyp);
        let p = curve.point_at(0.5);
        assert!((p.u - 2.0).abs() < 1e-10);
        assert!(p.v.abs() < 1e-10);
        assert!(curve.length() > 0.0);
    }

    #[test]
    fn test_curve2d_parabola_dispatch() {
        let par = Parabola2d::new(
            Point2d::new(0.0, 0.0),
            1.0,
            1.0, 0.0,
            0.0, 2.0,
        );
        let curve = Curve2d::Parabola(par);
        let p = curve.point_at(0.5);
        assert!((p.u - 0.25).abs() < 1e-10);
        assert!((p.v - 1.0).abs() < 1e-10);
        assert!(curve.length() > 0.0);
    }

    // ── Vision 2036 §1.3: analytical derivatives for PCURVE ──────

    /// Rational quadratic Bézier exact quarter unit circle:
    /// P0=(1,0), P1=(1,1), P2=(0,1), w=(1, √2/2, 1), knots [0,0,0,1,1,1].
    fn quarter_circle_nurbs2d() -> Nurbs2d {
        let r2 = 2.0_f64.sqrt();
        Nurbs2d {
            degree: 2,
            control_points: vec![
                Point2d::new(1.0, 0.0),
                Point2d::new(1.0, 1.0),
                Point2d::new(0.0, 1.0),
            ],
            weights: vec![1.0, r2 / 2.0, 1.0],
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        }
    }

    #[test]
    fn test_nurbs2d_derivative_analytical_quarter_circle() {
        let n = quarter_circle_nurbs2d();
        let r2 = 2.0_f64.sqrt();

        // Exact endpoint tangents: C'(0) = (0, √2), C'(1) = (−√2, 0)
        // (quotient rule on the homogeneous quadratic Bézier).
        let (du, dv) = n.derivative_at(0.0);
        assert!(du.abs() < 1e-12, "C'(0).u should be 0, got {du}");
        assert!((dv - r2).abs() < 1e-12, "C'(0).v should be √2, got {dv}");
        let (du, dv) = n.derivative_at(1.0);
        assert!((du + r2).abs() < 1e-12, "C'(1).u should be −√2, got {du}");
        assert!(dv.abs() < 1e-12, "C'(1).v should be 0, got {dv}");

        // Interior: C'(t) must be tangent to the circle (perpendicular to
        // the radius) — the analytic property the numerical difference
        // only approximates.
        for &t in &[0.15_f64, 0.3, 0.5, 0.7, 0.85] {
            let c = n.point_at(t);
            let (du, dv) = n.derivative_at(t);
            // C is on the unit circle: |C| = 1 (within Bézier exactness).
            assert!((c.u * c.u + c.v * c.v - 1.0).abs() < 1e-9,
                "quarter circle should stay on the unit circle at t={t}");
            // Tangency: C·C' = 0 exactly for a circle.
            let dot = c.u * du + c.v * dv;
            assert!(dot.abs() < 1e-9,
                "C·C' should vanish (tangency) at t={t}, got {dot}");
        }

        // Analytic vs central differences: agreement to ~1e-6 (the
        // finite difference itself carries O(eps) truncation error).
        let eps = 1e-7;
        for &t in &[0.1_f64, 0.25, 0.5, 0.75, 0.9] {
            let (du, dv) = n.derivative_at(t);
            let p0 = n.point_at(t - eps);
            let p1 = n.point_at(t + eps);
            let nu = (p1.u - p0.u) / (2.0 * eps);
            let nv = (p1.v - p0.v) / (2.0 * eps);
            assert!((du - nu).abs() < 1e-6, "u-derivative mismatch at t={t}: {du} vs {nu}");
            assert!((dv - nv).abs() < 1e-6, "v-derivative mismatch at t={t}: {dv} vs {nv}");
        }
    }

    #[test]
    fn test_nurbs2d_derivative_matches_3d_twin() {
        use crate::curve::NurbsCurve;
        use crate::point::Point3d;
        let n2 = quarter_circle_nurbs2d();
        let n3 = NurbsCurve {
            degree: n2.degree,
            control_points: n2
                .control_points
                .iter()
                .map(|p| Point3d::new(p.u, p.v, 0.0))
                .collect(),
            weights: n2.weights.clone(),
            knots: n2.knots.clone(),
        };
        for &t in &[0.0_f64, 0.2, 0.5, 0.8, 1.0] {
            let (du, dv) = n2.derivative_at(t);
            let d3 = n3.derivative_at(t);
            assert!((du - d3.x).abs() < 1e-12,
                "2D/3D derivative u mismatch at t={t}: {du} vs {}", d3.x);
            assert!((dv - d3.y).abs() < 1e-12,
                "2D/3D derivative v mismatch at t={t}: {dv} vs {}", d3.y);
        }
    }

    #[test]
    fn test_nurbs2d_derivative_uniform_knots_magnitude() {
        // Uniform (non-clamped-interior) knots — the case that exposed the
        // wrong denominator in the 3D twin («C2-периодичность шва»):
        // degree-2, 5 control points, uniform interior knots.
        // Analytic derivative must match central differences.
        let n = Nurbs2d {
            degree: 2,
            control_points: vec![
                Point2d::new(0.0, 0.0),
                Point2d::new(1.0, 2.0),
                Point2d::new(2.0, -1.0),
                Point2d::new(3.0, 2.0),
                Point2d::new(4.0, 0.0),
            ],
            weights: vec![1.0; 5],
            knots: vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        };
        let eps = 1e-7;
        for &t in &[0.05_f64, 0.25, 0.5, 0.75, 0.95] {
            let (du, dv) = n.derivative_at(t);
            let p0 = n.point_at(t - eps);
            let p1 = n.point_at(t + eps);
            let nu = (p1.u - p0.u) / (2.0 * eps);
            let nv = (p1.v - p0.v) / (2.0 * eps);
            assert!((du - nu).abs() < 1e-6, "u mismatch at t={t}: {du} vs {nu}");
            assert!((dv - nv).abs() < 1e-6, "v mismatch at t={t}: {dv} vs {nv}");
        }
    }

    // ── Vision 2036 §1.3: projections for PCURVE ──────────────────

    #[test]
    fn test_line2d_project_point() {
        let line = Line2d::new(Point2d::new(0.0, 0.0), Point2d::new(10.0, 0.0));
        // Interior: orthogonal foot.
        let (t, d) = line.project_point(&Point2d::new(5.0, 3.0));
        assert!((t - 0.5).abs() < 1e-12);
        assert!((d - 3.0).abs() < 1e-12);
        // Clamped to start.
        let (t, d) = line.project_point(&Point2d::new(-2.0, 1.0));
        assert!(t.abs() < 1e-12);
        assert!((d - 5.0_f64.sqrt()).abs() < 1e-12);
        // Clamped to end.
        let (t, d) = line.project_point(&Point2d::new(12.0, 4.0));
        assert!((t - 1.0).abs() < 1e-12);
        assert!((d - 20.0_f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn test_circle2d_project_point_full() {
        let circle = Circle2d::new_full(Point2d::new(1.0, 1.0), 2.0);
        // Point ON the circle at angle −π/2: wraps to t = 3/4.
        let (t, d) = circle.project_point(&Point2d::new(1.0, -1.0));
        assert!((t - 0.75).abs() < 1e-12, "t should wrap to 0.75, got {t}");
        assert!(d.abs() < 1e-12);
        let c = circle.point_at(t);
        assert!((c.u - 1.0).abs() < 1e-12 && (c.v + 1.0).abs() < 1e-12);
        // Point ON the circle at angle 0: dist = 0, t = 0.
        let (t, d) = circle.project_point(&Point2d::new(3.0, 1.0));
        assert!(t.abs() < 1e-12, "t should be 0 (angle 0), got {t}");
        assert!(d.abs() < 1e-12, "point on circle: dist 0, got {d}");
        let (t, d) = circle.project_point(&Point2d::new(5.0, 1.0));
        assert!((t - 0.0).abs() < 1e-9, "t should be 0 (angle 0), got {t}");
        assert!((d - 2.0).abs() < 1e-12, "dist should be 2.0, got {d}");
        // Point at the center: any angle is closest; dist = radius.
        let (_t, d) = circle.project_point(&Point2d::new(1.0, 1.0));
        assert!((d - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_circle2d_project_point_arc() {
        // Quarter arc [0, π/2], unit radius at origin.
        let arc = Circle2d::new_arc(Point2d::new(0.0, 0.0), 1.0, 0.0, PI / 2.0);
        // Radial point at 45° (inside the angular range): exact t = 0.5.
        let p = Point2d::new(1.2 * (PI / 4.0).cos(), 1.2 * (PI / 4.0).sin());
        let (t, d) = arc.project_point(&p);
        assert!((t - 0.5).abs() < 1e-12, "t should be 0.5, got {t}");
        assert!((d - 0.2).abs() < 1e-12, "dist should be 0.2, got {d}");
        // Angle π is outside the arc: closest is the endpoint at π/2.
        let (t, d) = arc.project_point(&Point2d::new(-1.0, 0.0));
        assert!((t - 1.0).abs() < 1e-12, "t should clamp to 1.0, got {t}");
        assert!((d - 2.0_f64.sqrt()).abs() < 1e-12, "dist should be √2, got {d}");
    }

    #[test]
    fn test_ellipse2d_project_point_on_curve() {
        let ellipse = Ellipse2d::new_full(Point2d::new(0.0, 0.0), 2.0, 1.0, 0.0);
        // Point on the ellipse at parametric angle π/3 → t = 1/12.
        let on_curve = ellipse.point_at(1.0 / 12.0);
        let (t, d) = ellipse.project_point(&on_curve);
        assert!(d < 1e-9, "on-curve projection should be exact, dist={d}");
        assert!((t - 1.0 / 12.0).abs() < 1e-6, "t should recover 1/12, got {t}");
        // Off-curve: orthogonality of the residual (interior minimum).
        let p = Point2d::new(3.0, 0.5);
        let (t, d) = ellipse.project_point(&p);
        let c = ellipse.point_at(t);
        let (du, dv) = ellipse.derivative_at(t);
        let dot = (p.u - c.u) * du + (p.v - c.v) * dv;
        assert!(dot.abs() < 1e-9, "residual must be orthogonal to tangent, dot={dot}");
        assert!(d < 1.5, "distance sanity, got {d}");
    }

    #[test]
    fn test_hyperbola2d_project_point() {
        // P(s) = (2·cosh s, sinh s) for s ∈ [−1, 1].
        let hyp = Hyperbola2d::new(
            Point2d::new(0.0, 0.0),
            2.0, 1.0,
            1.0, 0.0,
            -1.0, 1.0,
        );
        // On-curve point at s = 0.5 → canonical t = 0.75.
        let on_curve = hyp.point_at(0.75);
        let (t, d) = hyp.project_point(&on_curve);
        assert!(d < 1e-9, "on-curve projection should be exact, dist={d}");
        assert!((t - 0.75).abs() < 1e-6, "t should recover 0.75, got {t}");
        // Interior minimum: for p = (3, 0) the stationarity condition
        // sinh s·(10·cosh s − 12) = 0 has interior roots s ≈ ±0.622 —
        // the residual must be orthogonal to the tangent there.
        let p = Point2d::new(3.0, 0.0);
        let (t, d) = hyp.project_point(&p);
        let c = hyp.point_at(t);
        let (du, dv) = hyp.derivative_at(t);
        let dot = (p.u - c.u) * du + (p.v - c.v) * dv;
        assert!(dot.abs() < 1e-9,
            "residual must be orthogonal to tangent at interior minimum, dot={dot}");
        assert!(t > 1e-6 && t < 1.0 - 1e-6, "interior minimum expected, got t={t}");
        // Boundary minimum: for p = (4, 0.5) the distance decreases all
        // the way to s = 1 (f'(1) < 0) — no orthogonality at the boundary,
        // but the projection must still find the global minimum.
        let p = Point2d::new(4.0, 0.5);
        let (t, d) = hyp.project_point(&p);
        assert!((t - 1.0).abs() < 1e-9, "boundary minimum expected, got t={t}");
        for i in 0..2000 {
            let ts = i as f64 / 1999.0;
            let cs = hyp.point_at(ts);
            let ds = ((p.u - cs.u).powi(2) + (p.v - cs.v).powi(2)).sqrt();
            assert!(d <= ds + 1e-9,
                "sample at t={ts} is closer ({ds} < {d}) — projection missed the global minimum");
        }
    }

    #[test]
    fn test_parabola2d_project_point_orthogonality() {
        // P(s) = (s²/2, s) for s ∈ [−2, 2] (f = 0.5, axis +U).
        let par = Parabola2d::new(Point2d::new(0.0, 0.0), 0.5, 1.0, 0.0, -2.0, 2.0);
        // On-curve point at s = 0.4 → canonical t = 0.6.
        let on_curve = par.point_at(0.6);
        let (t, d) = par.project_point(&on_curve);
        assert!(d < 1e-9, "on-curve projection should be exact, dist={d}");
        assert!((t - 0.6).abs() < 1e-6, "t should recover 0.6, got {t}");
        // Off-curve: orthogonality of the residual (interior minimum).
        let p = Point2d::new(0.0, 0.5);
        let (t, d) = par.project_point(&p);
        assert!(t > 1e-6 && t < 1.0 - 1e-6, "interior minimum expected, got t={t}");
        let c = par.point_at(t);
        let (du, dv) = par.derivative_at(t);
        let dot = (p.u - c.u) * du + (p.v - c.v) * dv;
        assert!(dot.abs() < 1e-9, "residual must be orthogonal to tangent, dot={dot}");
        // Brute-force check: no sample beats the found minimum.
        for i in 0..2000 {
            let ts = i as f64 / 1999.0;
            let cs = par.point_at(ts);
            let ds = ((p.u - cs.u).powi(2) + (p.v - cs.v).powi(2)).sqrt();
            assert!(d <= ds + 1e-9,
                "sample at t={ts} is closer ({ds} < {d}) — projection missed the global minimum");
        }
    }

    #[test]
    fn test_nurbs2d_project_point_quarter_circle() {
        let n = quarter_circle_nurbs2d();
        // On-curve point at 30°: projection must land ON the circle.
        let p = Point2d::new((PI / 6.0).cos(), (PI / 6.0).sin());
        let (t, d) = n.project_point(&p);
        assert!(d < 1e-9, "on-curve projection should be exact, dist={d}");
        let c = n.point_at(t);
        assert!((c.u - p.u).abs() < 1e-9 && (c.v - p.v).abs() < 1e-9);
        // Radial point at 30°, 1.3× radius: dist = 0.3, foot on the circle.
        let p = Point2d::new(1.3 * (PI / 6.0).cos(), 1.3 * (PI / 6.0).sin());
        let (t, d) = n.project_point(&p);
        assert!((d - 0.3).abs() < 1e-9, "radial distance should be 0.3, got {d}");
        let c = n.point_at(t);
        assert!((c.u - (PI / 6.0).cos()).abs() < 1e-9, "foot should be at 30°");
        assert!((c.v - (PI / 6.0).sin()).abs() < 1e-9);
        // Orthogonality of the residual (tangent ⟂ radius).
        let (du, dv) = n.derivative_at(t);
        let dot = (p.u - c.u) * du + (p.v - c.v) * dv;
        assert!(dot.abs() < 1e-9, "residual must be orthogonal to tangent, dot={dot}");
    }

    #[test]
    fn test_curve2d_composite_project_point() {
        // L-shape: (0,0)→(1,0) then (1,0)→(1,1); equal lengths.
        let composite = Curve2d::Composite {
            segments: vec![
                Curve2d::Line(Line2d::new(Point2d::new(0.0, 0.0), Point2d::new(1.0, 0.0))),
                Curve2d::Line(Line2d::new(Point2d::new(1.0, 0.0), Point2d::new(1.0, 1.0))),
            ],
            cum_lengths: vec![0.5, 1.0],
        };
        // Near the vertical segment: local t = 0.49 → global 0.5 + 0.49·0.5.
        let (t, d) = composite.project_point(&Point2d::new(1.2, 0.49));
        assert!((t - 0.745).abs() < 1e-12, "global t should be 0.745, got {t}");
        assert!((d - 0.2).abs() < 1e-12, "dist should be 0.2, got {d}");
        // Round-trip: the global t must map back to the same closest point.
        let c = composite.point_at(t);
        assert!((c.u - 1.0).abs() < 1e-12 && (c.v - 0.49).abs() < 1e-12);
        // Near the horizontal segment: local t = 0.4 → global 0.2.
        let (t, d) = composite.project_point(&Point2d::new(0.4, -0.3));
        assert!((t - 0.2).abs() < 1e-12, "global t should be 0.2, got {t}");
        assert!((d - 0.3).abs() < 1e-12, "dist should be 0.3, got {d}");
    }

    #[test]
    fn test_curve2d_distance_to_dispatch() {
        let curve = Curve2d::Circle(Circle2d::new_full(Point2d::new(0.0, 0.0), 1.0));
        let d = curve.distance_to(&Point2d::new(3.0, 0.0));
        assert!((d - 2.0).abs() < 1e-12);
        let (t, d2) = curve.project_point(&Point2d::new(3.0, 0.0));
        assert!((d - d2).abs() < 1e-15);
        assert!(t.abs() < 1e-12);
    }
}
