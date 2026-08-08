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
}
