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

    /// Derivative at parameter t (numerical).
    pub fn derivative_at(&self, t: f64) -> (f64, f64) {
        let eps = 1e-7;
        let p0 = self.point_at(t - eps);
        let p1 = self.point_at(t + eps);
        ((p1.u - p0.u) / (2.0 * eps), (p1.v - p0.v) / (2.0 * eps))
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

impl Curve2d {
    /// Evaluate the curve at parameter t.
    pub fn point_at(&self, t: f64) -> Point2d {
        match self {
            Curve2d::Line(l) => l.point_at(t),
            Curve2d::Circle(c) => c.point_at(t),
            Curve2d::Ellipse(e) => e.point_at(t),
            Curve2d::Hyperbola(h) => h.point_at(t),
            Curve2d::Parabola(p) => p.point_at(t),
            Curve2d::Nurbs(n) => n.point_at(t),
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
}
