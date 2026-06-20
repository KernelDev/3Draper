// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Parametric curves in 3D space.

use crate::{Direction3d, Point3d, Vec3d, Transform};

/// Parametric range [u_min, u_max].
pub type ParamRange = (f64, f64);

/// A parametric curve in 3D space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Curve3d {
    /// Line: P(t) = origin + t * direction
    Line(Line),
    /// Circle in 3D space defined by center, normal, radius
    Circle(Circle),
    /// Ellipse in 3D space
    Ellipse(Ellipse),
    /// Arc (trimmed circle segment)
    Arc(Arc),
    /// Hyperbola in 3D space (algorithm adapted from truck-geometry v0.6,
    /// ricosjp/truck, Apache-2.0 OR MIT).
    Hyperbola(Hyperbola),
    /// Parabola in 3D space (algorithm adapted from truck-geometry v0.6,
    /// ricosjp/truck, Apache-2.0 OR MIT).
    Parabola(Parabola),
    /// NURBS curve
    Nurbs(NurbsCurve),
}

/// A line in 3D.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Line {
    pub origin: Point3d,
    pub direction: Direction3d,
}

impl Line {
    pub fn new(origin: Point3d, direction: Direction3d) -> Self {
        Self { origin, direction }
    }

    /// Create a line through two points.
    pub fn through_points(p1: Point3d, p2: Point3d) -> Option<Self> {
        let dir = Direction3d::new(
            p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z,
        )?;
        Some(Self { origin: p1, direction: dir })
    }

    /// Evaluate point at parameter t.
    pub fn point_at(&self, t: f64) -> Point3d {
        Point3d::new(
            self.origin.x + t * self.direction.x,
            self.origin.y + t * self.direction.y,
            self.origin.z + t * self.direction.z,
        )
    }

    /// Evaluate derivative at parameter t.
    pub fn derivative_at(&self, _t: f64) -> Vec3d {
        Vec3d::new(self.direction.x, self.direction.y, self.direction.z)
    }

    /// Check if this line is degenerate (zero direction vector).
    pub fn is_degenerate(&self, tolerance: f64) -> bool {
        let len_sq = self.direction.x * self.direction.x
            + self.direction.y * self.direction.y
            + self.direction.z * self.direction.z;
        len_sq < tolerance * tolerance
    }
}

/// A circle in 3D space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle {
    pub center: Point3d,
    pub normal: Direction3d,
    pub radius: f64,
    /// X-axis of the circle's local coordinate system
    pub x_axis: Direction3d,
}

impl Circle {
    /// Create a circle in the XY plane.
    pub fn new_xy(center: Point3d, radius: f64) -> Self {
        Self {
            center,
            normal: Direction3d::Z,
            radius,
            x_axis: Direction3d::X,
        }
    }

    /// Create a circle with arbitrary orientation.
    pub fn new(center: Point3d, normal: Direction3d, radius: f64) -> Self {
        let x_axis = if normal.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            normal.cross(&Direction3d::Z)
        };
        Self { center, normal, radius, x_axis }
    }

    /// Evaluate point at angle t (radians).
    pub fn point_at(&self, t: f64) -> Point3d {
        let y_axis = self.normal.cross(&self.x_axis);
        Point3d::new(
            self.center.x + self.radius * (t.cos() * self.x_axis.x + t.sin() * y_axis.x),
            self.center.y + self.radius * (t.cos() * self.x_axis.y + t.sin() * y_axis.y),
            self.center.z + self.radius * (t.cos() * self.x_axis.z + t.sin() * y_axis.z),
        )
    }

    /// Evaluate first derivative at angle t.
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        let y_axis = self.normal.cross(&self.x_axis);
        Vec3d::new(
            self.radius * (-t.sin() * self.x_axis.x + t.cos() * y_axis.x),
            self.radius * (-t.sin() * self.x_axis.y + t.cos() * y_axis.y),
            self.radius * (-t.sin() * self.x_axis.z + t.cos() * y_axis.z),
        )
    }

    /// Circumference.
    pub fn circumference(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.radius
    }
}

/// An ellipse in 3D space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse {
    pub center: Point3d,
    pub normal: Direction3d,
    pub semi_major: f64,
    pub semi_minor: f64,
    pub x_axis: Direction3d,
}

impl Ellipse {
    pub fn new_xy(center: Point3d, semi_major: f64, semi_minor: f64) -> Self {
        Self {
            center,
            normal: Direction3d::Z,
            semi_major,
            semi_minor,
            x_axis: Direction3d::X,
        }
    }

    pub fn point_at(&self, t: f64) -> Point3d {
        let y_axis = self.normal.cross(&self.x_axis);
        Point3d::new(
            self.center.x + self.semi_major * t.cos() * self.x_axis.x + self.semi_minor * t.sin() * y_axis.x,
            self.center.y + self.semi_major * t.cos() * self.x_axis.y + self.semi_minor * t.sin() * y_axis.y,
            self.center.z + self.semi_major * t.cos() * self.x_axis.z + self.semi_minor * t.sin() * y_axis.z,
        )
    }

    /// Evaluate the first derivative (tangent vector) at parameter t.
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        let y_axis = self.normal.cross(&self.x_axis);
        Vec3d::new(
            -self.semi_major * t.sin() * self.x_axis.x + self.semi_minor * t.cos() * y_axis.x,
            -self.semi_major * t.sin() * self.x_axis.y + self.semi_minor * t.cos() * y_axis.y,
            -self.semi_major * t.sin() * self.x_axis.z + self.semi_minor * t.cos() * y_axis.z,
        )
    }
}

/// An arc (trimmed circle segment).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Arc {
    pub circle: Circle,
    pub start_angle: f64,
    pub end_angle: f64,
}

impl Arc {
    pub fn new(circle: Circle, start_angle: f64, end_angle: f64) -> Self {
        Self { circle, start_angle, end_angle }
    }

    pub fn point_at(&self, t: f64) -> Point3d {
        // t in [0, 1] maps to [start_angle, end_angle]
        let angle = self.start_angle + t * (self.end_angle - self.start_angle);
        self.circle.point_at(angle)
    }

    pub fn start_point(&self) -> Point3d {
        self.circle.point_at(self.start_angle)
    }

    pub fn end_point(&self) -> Point3d {
        self.circle.point_at(self.end_angle)
    }

    /// Evaluate the first derivative (tangent vector) at parameter t.
    ///
    /// The parameter t ∈ [0, 1] maps to [start_angle, end_angle].
    /// The derivative includes the chain rule factor dθ/dt = end_angle - start_angle.
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        let d_theta = self.end_angle - self.start_angle;
        let angle = self.start_angle + t * d_theta;
        let d_circle = self.circle.derivative_at(angle);
        Vec3d::new(
            d_circle.x * d_theta,
            d_circle.y * d_theta,
            d_circle.z * d_theta,
        )
    }
}

/// A hyperbola in 3D space.
///
/// Standard form: x²/a² - y²/b² = 1, where a = semi_real, b = semi_imag.
/// The hyperbola lies in the plane defined by `normal`, with its center at `center`,
/// its transverse axis along `x_axis`, and conjugate axis along `y_axis = normal × x_axis`.
///
/// Parametric form:
///   P(t) = center + a·cosh(t)·x_axis + b·sinh(t)·y_axis
///   P'(t) = a·sinh(t)·x_axis + b·cosh(t)·y_axis
///
/// Parameter range: (-∞, +∞). In practice, STEP TRIMMED_CURVE provides bounds.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Hyperbola {
    /// Center of the hyperbola (midpoint between the two vertices).
    pub center: Point3d,
    /// Normal to the plane of the hyperbola (axis_z of the STEP AXIS2_PLACEMENT_3D).
    pub normal: Direction3d,
    /// Transverse axis direction (along which the two branches open).
    /// Corresponds to ref_direction of the STEP AXIS2_PLACEMENT_3D.
    pub x_axis: Direction3d,
    /// Semi-axis length along x_axis (a in x²/a² - y²/b² = 1).
    pub semi_real: f64,
    /// Semi-imaginary axis length along y_axis (b in x²/a² - y²/b² = 1).
    pub semi_imag: f64,
}

impl Hyperbola {
    /// Construct a hyperbola in the XY plane centered at the given point.
    pub fn new_xy(center: Point3d, semi_real: f64, semi_imag: f64) -> Self {
        Self {
            center,
            normal: Direction3d::Z,
            x_axis: Direction3d::X,
            semi_real,
            semi_imag,
        }
    }

    /// Evaluate the point at parameter t.
    pub fn point_at(&self, t: f64) -> Point3d {
        let y_axis = self.normal.cross(&self.x_axis);
        let ch = t.cosh();
        let sh = t.sinh();
        Point3d::new(
            self.center.x + self.semi_real * ch * self.x_axis.x + self.semi_imag * sh * y_axis.x,
            self.center.y + self.semi_real * ch * self.x_axis.y + self.semi_imag * sh * y_axis.y,
            self.center.z + self.semi_real * ch * self.x_axis.z + self.semi_imag * sh * y_axis.z,
        )
    }

    /// Evaluate the first derivative (tangent vector) at parameter t.
    ///
    /// dP/dt = a·sinh(t)·x_axis + b·cosh(t)·y_axis
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        let y_axis = self.normal.cross(&self.x_axis);
        let ch = t.cosh();
        let sh = t.sinh();
        Vec3d::new(
            self.semi_real * sh * self.x_axis.x + self.semi_imag * ch * y_axis.x,
            self.semi_real * sh * self.x_axis.y + self.semi_imag * ch * y_axis.y,
            self.semi_real * sh * self.x_axis.z + self.semi_imag * ch * y_axis.z,
        )
    }
}

/// A parabola in 3D space.
///
/// Standard form: x = y²/(4f), where f = focal_dist. The parabola opens along +x_axis,
/// with its vertex at `vertex` and focus at `vertex + f·x_axis`.
///
/// Parametric form (parameter t = y):
///   P(t) = vertex + (t²/(4f))·x_axis + t·y_axis
///   P'(t) = (t/(2f))·x_axis + y_axis
///
/// Parameter range: (-∞, +∞). In practice, STEP TRIMMED_CURVE provides bounds.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Parabola {
    /// Vertex of the parabola (the point of zero curvature).
    pub vertex: Point3d,
    /// Normal to the plane of the parabola (axis_z of the STEP AXIS2_PLACEMENT_3D).
    pub normal: Direction3d,
    /// Direction in which the parabola opens (toward the focus).
    /// Corresponds to ref_direction of the STEP AXIS2_PLACEMENT_3D.
    pub x_axis: Direction3d,
    /// Focal distance f > 0. Focus is at vertex + f·x_axis.
    pub focal_dist: f64,
}

impl Parabola {
    /// Construct a parabola in the XY plane opening along +X.
    pub fn new_xy(vertex: Point3d, focal_dist: f64) -> Self {
        Self {
            vertex,
            normal: Direction3d::Z,
            x_axis: Direction3d::X,
            focal_dist,
        }
    }

    /// Evaluate the point at parameter t.
    pub fn point_at(&self, t: f64) -> Point3d {
        let y_axis = self.normal.cross(&self.x_axis);
        let f = if self.focal_dist.abs() < 1e-15 { 1e-15 } else { self.focal_dist };
        let x_comp = t * t / (4.0 * f);
        Point3d::new(
            self.vertex.x + x_comp * self.x_axis.x + t * y_axis.x,
            self.vertex.y + x_comp * self.x_axis.y + t * y_axis.y,
            self.vertex.z + x_comp * self.x_axis.z + t * y_axis.z,
        )
    }

    /// Evaluate the first derivative (tangent vector) at parameter t.
    ///
    /// dP/dt = (t/(2f))·x_axis + y_axis
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        let y_axis = self.normal.cross(&self.x_axis);
        let f = if self.focal_dist.abs() < 1e-15 { 1e-15 } else { self.focal_dist };
        let x_comp = t / (2.0 * f);
        Vec3d::new(
            x_comp * self.x_axis.x + y_axis.x,
            x_comp * self.x_axis.y + y_axis.y,
            x_comp * self.x_axis.z + y_axis.z,
        )
    }
}

/// NURBS curve representation.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsCurve {
    pub degree: usize,
    pub control_points: Vec<Point3d>,
    pub weights: Vec<f64>,
    pub knots: Vec<f64>,
}

impl NurbsCurve {
    /// Evaluate the first derivative of the NURBS curve at parameter t.
    ///
    /// Uses the quotient rule for rational B-splines:
    ///   C(t) = A(t) / w(t)
    ///   C'(t) = (A'(t) - C(t) * w'(t)) / w(t)
    ///
    /// where A(t) is the weighted numerator curve and w(t) is the weight function.
    ///
    /// Returns the tangent vector at t.
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        let n = self.control_points.len();
        if n == 0 {
            return Vec3d::new(0.0, 0.0, 0.0);
        }

        let p = self.degree;
        let t_min = if self.knots.len() > p { self.knots[p] } else { 0.0 };
        let t_max = if self.knots.len() > p { self.knots[self.knots.len() - p - 1] } else { 1.0 };
        let t_c = t.clamp(t_min, t_max);

        let k = find_knot_span_curve(&self.knots, p, t_c, n);

        // Collect p+1 weighted control points
        let mut pts: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(p + 1);
        for i in 0..=p {
            let idx = k - p + i;
            let idx = if idx >= n { n - 1 } else { idx };
            let w = self.weights.get(idx).copied().unwrap_or(1.0);
            let cp = &self.control_points[idx];
            pts.push((cp.x * w, cp.y * w, cp.z * w, w));
        }

        // Evaluate A(t) and w(t) using de Boor
        let mut pts_eval = pts.clone();
        de_boor_step_curve(&mut pts_eval, &self.knots, p, k, t_c);
        let a_result = match pts_eval.last() {
            Some(&r) => r,
            None => return Vec3d::new(0.0, 0.0, 0.0),
        };
        let w = a_result.3;
        if w.abs() < 1e-15 {
            return Vec3d::new(0.0, 0.0, 0.0);
        }
        let cx = a_result.0 / w;
        let cy = a_result.1 / w;
        let cz = a_result.2 / w;

        // Compute derivative of the weighted numerator A'(t) and weight derivative w'(t)
        // Using the derivative of the B-spline basis functions
        let mut dpts: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(p);
        for i in 0..p {
            let idx_curr = i;
            let idx_next = i + 1;

            let k_low = k - p + idx_curr;
            let k_high = k - p + idx_next;

            let denom1 = if k_low + p + 1 < self.knots.len() && k_low < self.knots.len() {
                let d = self.knots[k_low + p + 1] - self.knots[k_low];
                if d.abs() < 1e-15 { 0.0 } else { p as f64 / d }
            } else {
                0.0
            };

            let denom2 = if k_high + p + 1 < self.knots.len() && k_high < self.knots.len() {
                let d = self.knots[k_high + p + 1] - self.knots[k_high];
                if d.abs() < 1e-15 { 0.0 } else { p as f64 / d }
            } else {
                0.0
            };

            // d[i] = denom1 * pts[i+1] - denom2 * pts[i]
            dpts.push((
                denom1 * pts[idx_next].0 - denom2 * pts[idx_curr].0,
                denom1 * pts[idx_next].1 - denom2 * pts[idx_curr].1,
                denom1 * pts[idx_next].2 - denom2 * pts[idx_curr].2,
                denom1 * pts[idx_next].3 - denom2 * pts[idx_curr].3,
            ));
        }

        // Evaluate A'(t) using de Boor on the derivative control points (degree p-1)
        if p > 1 {
            de_boor_step_curve(&mut dpts, &self.knots, p - 1, k, t_c);
        }

        if dpts.is_empty() {
            return Vec3d::new(0.0, 0.0, 0.0);
        }

        let da = match dpts.last() {
            Some(&r) => r,
            None => return Vec3d::new(0.0, 0.0, 0.0),
        };
        let dw = da.3;

        // C'(t) = (A'(t) - C(t) * w'(t)) / w(t)
        let dx = (da.0 - cx * dw) / w;
        let dy = (da.1 - cy * dw) / w;
        let dz = (da.2 - cz * dw) / w;

        Vec3d::new(dx, dy, dz)
    }
}

impl Curve3d {
    /// Check if the curve is degenerate (zero length or zero radius).
    ///
    /// A degenerate curve has no meaningful geometric extent. This can happen
    /// when an edge's start and end points are coincident (zero-length edge),
    /// or when a circle/ellipse has zero radius.
    ///
    /// # Arguments
    /// * `tolerance` - The geometric tolerance for coincidence checks. Use
    ///   `ToleranceContext::coincidence_tolerance()` for model-scale-aware checks.
    pub fn is_degenerate(&self, tolerance: f64) -> bool {
        match self {
            Curve3d::Line(line) => {
                // A line is degenerate if its direction has zero length
                // (this shouldn't happen with Direction3d, but check anyway)
                let d = line.direction;
                let len_sq = d.x * d.x + d.y * d.y + d.z * d.z;
                len_sq < tolerance * tolerance
            }
            Curve3d::Circle(circle) => {
                // A circle is degenerate if its radius is smaller than tolerance
                circle.radius < tolerance
            }
            Curve3d::Ellipse(ellipse) => {
                // An ellipse is degenerate if both semi-axes are smaller than tolerance
                ellipse.semi_major < tolerance && ellipse.semi_minor < tolerance
            }
            Curve3d::Arc(arc) => {
                // An arc is degenerate if its underlying circle is degenerate
                // OR if the arc length is effectively zero (start and end are coincident)
                if arc.circle.radius < tolerance {
                    return true;
                }
                // Check if start and end points are coincident
                let start = arc.start_point();
                let end = arc.end_point();
                let dx = start.x - end.x;
                let dy = start.y - end.y;
                let dz = start.z - end.z;
                (dx * dx + dy * dy + dz * dz) < tolerance * tolerance
            }
            Curve3d::Hyperbola(h) => {
                // A hyperbola is degenerate if both semi-axes are below tolerance.
                // Hyperbolas always have non-zero extent in theory, but a STEP file
                // may contain a malformed entity with zero axes.
                h.semi_real.abs() < tolerance && h.semi_imag.abs() < tolerance
            }
            Curve3d::Parabola(p) => {
                // A parabola is degenerate if focal_dist is below tolerance.
                p.focal_dist.abs() < tolerance
            }
            Curve3d::Nurbs(nurbs) => {
                // A NURBS curve is degenerate if:
                // 1. It has fewer than 2 control points
                // 2. All control points are coincident within tolerance
                // 3. The knot vector is degenerate (empty or zero-length domain)
                if nurbs.control_points.len() < 2 {
                    return true;
                }

                // Check knot vector domain
                let n = nurbs.knots.len();
                let t_min = if n > nurbs.degree { nurbs.knots[nurbs.degree] } else { 0.0 };
                let t_max = if n > nurbs.degree { nurbs.knots[n - nurbs.degree - 1] } else { 1.0 };
                if (t_max - t_min).abs() < tolerance * 1e-8 {
                    return true;
                }

                // Check if all control points are coincident
                let first = &nurbs.control_points[0];
                let all_coincident = nurbs.control_points.iter().skip(1).all(|p| {
                    let dx = p.x - first.x;
                    let dy = p.y - first.y;
                    let dz = p.z - first.z;
                    (dx * dx + dy * dy + dz * dz) < tolerance * tolerance
                });
                all_coincident
            }
        }
    }

    /// Evaluate the curve at parameter t.
    pub fn point_at(&self, t: f64) -> Point3d {
        match self {
            Curve3d::Line(line) => line.point_at(t),
            Curve3d::Circle(circle) => circle.point_at(t),
            Curve3d::Ellipse(ellipse) => ellipse.point_at(t),
            Curve3d::Arc(arc) => arc.point_at(t),
            Curve3d::Hyperbola(h) => h.point_at(t),
            Curve3d::Parabola(p) => p.point_at(t),
            Curve3d::Nurbs(nurbs) => nurbs_eval(nurbs, t),
        }
    }

    /// Evaluate the first derivative (tangent vector) at parameter t.
    ///
    /// For analytical curves (Line, Circle, Ellipse, Arc, Hyperbola, Parabola, NURBS),
    /// uses exact analytical derivatives. NURBS uses the quotient rule for rational
    /// B-splines (algorithm adapted from truck-geometry v0.6, ricosjp/truck, Apache-2.0 OR MIT).
    pub fn derivative_at(&self, t: f64) -> Vec3d {
        match self {
            Curve3d::Line(line) => line.derivative_at(t),
            Curve3d::Circle(circle) => circle.derivative_at(t),
            Curve3d::Ellipse(ellipse) => ellipse.derivative_at(t),
            Curve3d::Arc(arc) => arc.derivative_at(t),
            Curve3d::Hyperbola(h) => h.derivative_at(t),
            Curve3d::Parabola(p) => p.derivative_at(t),
            Curve3d::Nurbs(nurbs) => {
                // Use the analytical NurbsCurve::derivative_at (quotient rule).
                // Falls back to numerical central differences only if the analytical
                // result is non-finite (NaN/Inf) — this should not happen for well-formed
                // NURBS but guards against malformed STEP data.
                let analytical = nurbs.derivative_at(t);
                if analytical.x.is_finite() && analytical.y.is_finite() && analytical.z.is_finite() {
                    let len_sq = analytical.x * analytical.x
                        + analytical.y * analytical.y
                        + analytical.z * analytical.z;
                    if len_sq > 1e-30 {
                        return analytical;
                    }
                }
                // Numerical fallback using central differences
                let (t_min, t_max) = {
                    let n = nurbs.knots.len();
                    if n > nurbs.degree {
                        (nurbs.knots[nurbs.degree], nurbs.knots[n - nurbs.degree - 1])
                    } else {
                        (0.0, 1.0)
                    }
                };
                let dt = (t_max - t_min).max(1e-12) * 1e-7;
                let t_lo = (t - dt).max(t_min);
                let t_hi = (t + dt).min(t_max);
                let p_lo = nurbs_eval(nurbs, t_lo);
                let p_hi = nurbs_eval(nurbs, t_hi);
                let actual_dt = t_hi - t_lo;
                if actual_dt.abs() < 1e-30 {
                    Vec3d::ZERO
                } else {
                    Vec3d::new(
                        (p_hi.x - p_lo.x) / actual_dt,
                        (p_hi.y - p_lo.y) / actual_dt,
                        (p_hi.z - p_lo.z) / actual_dt,
                    )
                }
            }
        }
    }

    /// Get the parametric range of the curve.
    pub fn param_range(&self) -> ParamRange {
        match self {
            Curve3d::Line(_) => (-f64::MAX, f64::MAX),
            Curve3d::Circle(_) => (0.0, 2.0 * std::f64::consts::PI),
            Curve3d::Ellipse(_) => (0.0, 2.0 * std::f64::consts::PI),
            Curve3d::Arc(arc) => (arc.start_angle, arc.end_angle),
            Curve3d::Hyperbola(_) => (-f64::MAX, f64::MAX),
            Curve3d::Parabola(_) => (-f64::MAX, f64::MAX),
            Curve3d::Nurbs(nurbs) => {
                let n = nurbs.knots.len();
                if n > nurbs.degree {
                    (nurbs.knots[nurbs.degree], nurbs.knots[n - nurbs.degree - 1])
                } else {
                    (0.0, 1.0)
                }
            }
        }
    }

    /// Transform the curve.
    pub fn transform(&self, t: &Transform) -> Curve3d {
        match self {
            Curve3d::Line(line) => Curve3d::Line(Line {
                origin: t.transform_point(&line.origin),
                direction: t.transform_direction(&line.direction),
            }),
            Curve3d::Circle(circle) => Curve3d::Circle(Circle {
                center: t.transform_point(&circle.center),
                normal: t.transform_direction(&circle.normal),
                radius: circle.radius, // Approximate for non-uniform scaling
                x_axis: t.transform_direction(&circle.x_axis),
            }),
            Curve3d::Ellipse(ellipse) => Curve3d::Ellipse(Ellipse {
                center: t.transform_point(&ellipse.center),
                normal: t.transform_direction(&ellipse.normal),
                semi_major: ellipse.semi_major,
                semi_minor: ellipse.semi_minor,
                x_axis: t.transform_direction(&ellipse.x_axis),
            }),
            Curve3d::Arc(arc) => Curve3d::Arc(Arc {
                circle: Circle {
                    center: t.transform_point(&arc.circle.center),
                    normal: t.transform_direction(&arc.circle.normal),
                    radius: arc.circle.radius,
                    x_axis: t.transform_direction(&arc.circle.x_axis),
                },
                start_angle: arc.start_angle,
                end_angle: arc.end_angle,
            }),
            Curve3d::Hyperbola(h) => Curve3d::Hyperbola(Hyperbola {
                center: t.transform_point(&h.center),
                normal: t.transform_direction(&h.normal),
                x_axis: t.transform_direction(&h.x_axis),
                semi_real: h.semi_real,
                semi_imag: h.semi_imag,
            }),
            Curve3d::Parabola(p) => Curve3d::Parabola(Parabola {
                vertex: t.transform_point(&p.vertex),
                normal: t.transform_direction(&p.normal),
                x_axis: t.transform_direction(&p.x_axis),
                focal_dist: p.focal_dist,
            }),
            Curve3d::Nurbs(nurbs) => Curve3d::Nurbs(NurbsCurve {
                degree: nurbs.degree,
                control_points: nurbs.control_points.iter().map(|p| t.transform_point(p)).collect(),
                weights: nurbs.weights.clone(),
                knots: nurbs.knots.clone(),
            }),
        }
    }
}

/// Evaluate a NURBS curve at parameter t using de Boor's algorithm.
fn nurbs_eval(nurbs: &NurbsCurve, t: f64) -> Point3d {
    let n = nurbs.control_points.len();
    if n == 0 {
        return Point3d::ORIGIN;
    }
    if n == 1 {
        let w = nurbs.weights.get(0).copied().unwrap_or(1.0);
        if w.abs() < 1e-15 {
            return Point3d::ORIGIN;
        }
        let cp = &nurbs.control_points[0];
        return Point3d::new(cp.x, cp.y, cp.z);
    }

    let p = nurbs.degree;

    // Clamp to valid knot range
    let t_min = if nurbs.knots.len() > p { nurbs.knots[p] } else { 0.0 };
    let t_max = if nurbs.knots.len() > p { nurbs.knots[nurbs.knots.len() - p - 1] } else { 1.0 };
    let t_c = t.clamp(t_min, t_max);

    // Find knot span: T[k] <= t_c < T[k+1]
    let k = find_knot_span_curve(&nurbs.knots, p, t_c, n);

    // Collect p+1 weighted control points
    let mut pts: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(p + 1);
    for i in 0..=p {
        let idx = k - p + i;
        let idx = if idx >= n { n - 1 } else { idx };
        let w = nurbs.weights.get(idx).copied().unwrap_or(1.0);
        let cp = &nurbs.control_points[idx];
        pts.push((cp.x * w, cp.y * w, cp.z * w, w));
    }

    // De Boor's algorithm (standard)
    de_boor_step_curve(&mut pts, &nurbs.knots, p, k, t_c);

    if let Some(&result) = pts.last() {
        let w = result.3;
        if w.abs() < 1e-15 {
            Point3d::ORIGIN
        } else {
            Point3d::new(result.0 / w, result.1 / w, result.2 / w)
        }
    } else {
        Point3d::ORIGIN
    }
}

/// Find knot span for curve: T[k] <= t < T[k+1]
fn find_knot_span_curve(knots: &[f64], degree: usize, t: f64, n_control_points: usize) -> usize {
    if t >= knots[n_control_points] {
        return n_control_points - 1;
    }
    let mut lo = degree;
    let mut hi = n_control_points;
    let mut mid = (lo + hi) / 2;
    let mut iterations = 0;
    let max_iterations = knots.len(); // Safety guard
    while t < knots[mid] || t >= knots[mid + 1] {
        iterations += 1;
        if iterations > max_iterations {
            break; // Safety: prevent infinite loop with degenerate knot vectors
        }
        if t < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// De Boor step for NURBS curves (4-tuple: wx, wy, wz, w).
/// Implements the standard de Boor algorithm:
///   for r = 1 .. degree:
///     for j = degree down to r:
///       i = k - degree + j
///       alpha = (t - knots[i]) / (knots[i + degree + 1 - r] - knots[i])
///       d[j] = alpha * d[j] + (1-alpha) * d[j-1]
fn de_boor_step_curve(pts: &mut [(f64, f64, f64, f64)], knots: &[f64], degree: usize, k: usize, t: f64) {
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
                alpha * pts[j].3 + beta * pts[j - 1].3,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_not_degenerate() {
        let line = Line::through_points(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
        ).unwrap();
        let curve = Curve3d::Line(line);
        assert!(!curve.is_degenerate(1e-6), "A line between distinct points should not be degenerate");
    }

    #[test]
    fn test_circle_degenerate_zero_radius() {
        let circle = Circle::new_xy(Point3d::ORIGIN, 0.0);
        let curve = Curve3d::Circle(circle);
        assert!(curve.is_degenerate(1e-6), "A circle with zero radius should be degenerate");
    }

    #[test]
    fn test_circle_not_degenerate() {
        let circle = Circle::new_xy(Point3d::ORIGIN, 10.0);
        let curve = Curve3d::Circle(circle);
        assert!(!curve.is_degenerate(1e-6), "A circle with nonzero radius should not be degenerate");
    }

    #[test]
    fn test_ellipse_degenerate() {
        let ellipse = Ellipse::new_xy(Point3d::ORIGIN, 0.0, 0.0);
        let curve = Curve3d::Ellipse(ellipse);
        assert!(curve.is_degenerate(1e-6), "An ellipse with zero semi-axes should be degenerate");
    }

    #[test]
    fn test_arc_degenerate_zero_angle() {
        let circle = Circle::new_xy(Point3d::ORIGIN, 10.0);
        // Arc from 0 to 0 — start and end at same point
        let arc = Arc::new(circle, 0.0, 0.0);
        let curve = Curve3d::Arc(arc);
        assert!(curve.is_degenerate(1e-6), "An arc with zero angle should be degenerate");
    }

    #[test]
    fn test_nurbs_degenerate_coincident_points() {
        // All control points at the same location
        let pts = vec![
            Point3d::new(1.0, 2.0, 3.0),
            Point3d::new(1.0, 2.0, 3.0),
            Point3d::new(1.0, 2.0, 3.0),
        ];
        let nurbs = NurbsCurve {
            degree: 2,
            control_points: pts,
            weights: vec![1.0, 1.0, 1.0],
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        };
        let curve = Curve3d::Nurbs(nurbs);
        assert!(curve.is_degenerate(1e-6), "A NURBS curve with coincident control points should be degenerate");
    }

    #[test]
    fn test_nurbs_not_degenerate() {
        let pts = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(5.0, 5.0, 0.0),
            Point3d::new(10.0, 0.0, 0.0),
        ];
        let nurbs = NurbsCurve {
            degree: 2,
            control_points: pts,
            weights: vec![1.0, 1.0, 1.0],
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        };
        let curve = Curve3d::Nurbs(nurbs);
        assert!(!curve.is_degenerate(1e-6), "A NURBS curve with distinct control points should not be degenerate");
    }

    #[test]
    fn test_edge_zero_length_degenerate() {
        // Create an edge where start == end
        let p = Point3d::new(1.0, 2.0, 3.0);
        let line = Line::through_points(p, p);
        // Line::through_points returns None for coincident points
        assert!(line.is_none(), "Line through identical points should return None");
    }

    // ─── Hyperbola tests ────────────────────────────────────────────────

    #[test]
    fn test_hyperbola_point_at_origin() {
        // At t=0, hyperbola should be at center + (a, 0, 0) in local frame.
        let h = Hyperbola::new_xy(Point3d::ORIGIN, 2.0, 1.5);
        let p = h.point_at(0.0);
        assert!((p.x - 2.0).abs() < 1e-12, "x = a*cosh(0) = a, got {}", p.x);
        assert!(p.y.abs() < 1e-12, "y = b*sinh(0) = 0, got {}", p.y);
        assert!(p.z.abs() < 1e-12, "z = 0, got {}", p.z);
    }

    #[test]
    fn test_hyperbola_satisfies_equation() {
        // x²/a² - y²/b² = 1 for all t.
        let h = Hyperbola::new_xy(Point3d::ORIGIN, 3.0, 2.0);
        for &t in &[0.5_f64, 1.0, 1.5, 2.0, -0.7, -1.3] {
            let p = h.point_at(t);
            let lhs = p.x * p.x / (3.0 * 3.0) - p.y * p.y / (2.0 * 2.0);
            assert!((lhs - 1.0).abs() < 1e-9,
                "At t={}, x²/a² - y²/b² = {}, expected 1.0", t, lhs);
        }
    }

    #[test]
    fn test_hyperbola_derivative_matches_numerical() {
        // dP/dt = a·sinh(t)·x + b·cosh(t)·y
        let h = Hyperbola::new_xy(Point3d::ORIGIN, 2.0, 1.0);
        let dt = 1e-7;
        for &t in &[0.3_f64, 1.0, 2.0, -1.5] {
            let analytical = h.derivative_at(t);
            let p_plus = h.point_at(t + dt);
            let p_minus = h.point_at(t - dt);
            let numerical_x = (p_plus.x - p_minus.x) / (2.0 * dt);
            let numerical_y = (p_plus.y - p_minus.y) / (2.0 * dt);
            let numerical_z = (p_plus.z - p_minus.z) / (2.0 * dt);
            assert!((analytical.x - numerical_x).abs() < 1e-6,
                "dx/dt mismatch at t={}: analytical={}, numerical={}", t, analytical.x, numerical_x);
            assert!((analytical.y - numerical_y).abs() < 1e-6,
                "dy/dt mismatch at t={}: analytical={}, numerical={}", t, analytical.y, numerical_y);
            assert!((analytical.z - numerical_z).abs() < 1e-6,
                "dz/dt mismatch at t={}: analytical={}, numerical={}", t, analytical.z, numerical_z);
        }
    }

    #[test]
    fn test_hyperbola_degenerate_zero_axes() {
        let h = Hyperbola::new_xy(Point3d::ORIGIN, 0.0, 0.0);
        let curve = Curve3d::Hyperbola(h);
        assert!(curve.is_degenerate(1e-6),
            "A hyperbola with both axes below tolerance should be degenerate");
    }

    #[test]
    fn test_hyperbola_not_degenerate() {
        let h = Hyperbola::new_xy(Point3d::ORIGIN, 5.0, 3.0);
        let curve = Curve3d::Hyperbola(h);
        assert!(!curve.is_degenerate(1e-6),
            "A hyperbola with non-zero axes should not be degenerate");
    }

    // ─── Parabola tests ────────────────────────────────────────────────

    #[test]
    fn test_parabola_point_at_vertex() {
        // At t=0, parabola should be at vertex.
        let p_curve = Parabola::new_xy(Point3d::new(1.0, 2.0, 3.0), 2.0);
        let p = p_curve.point_at(0.0);
        assert!((p.x - 1.0).abs() < 1e-12, "At t=0, x should equal vertex.x");
        assert!((p.y - 2.0).abs() < 1e-12, "At t=0, y should equal vertex.y");
        assert!((p.z - 3.0).abs() < 1e-12, "At t=0, z should equal vertex.z");
    }

    #[test]
    fn test_parabola_satisfies_equation() {
        // x = y²/(4f) → y² = 4*f*x for all t (where t = y).
        let f = 2.5_f64;
        let p_curve = Parabola::new_xy(Point3d::ORIGIN, f);
        for &t in &[-2.0_f64, -1.0, 0.5, 1.5, 3.0] {
            let p = p_curve.point_at(t);
            // y == t (since y_axis is +Y in XY plane)
            assert!((p.y - t).abs() < 1e-12, "At t={}, y should equal t", t);
            // x = t²/(4f)
            let expected_x = t * t / (4.0 * f);
            assert!((p.x - expected_x).abs() < 1e-12,
                "At t={}, x = {}, expected {}", t, p.x, expected_x);
        }
    }

    #[test]
    fn test_parabola_focus_position() {
        // The focus should be at vertex + f·x_axis.
        // We can verify: at t = 2f, x = f, y = 2f.
        // The focus is at (f, 0, 0) in local frame.
        let f = 3.0_f64;
        let p_curve = Parabola::new_xy(Point3d::ORIGIN, f);
        // At t=0, the tangent should be along y_axis (since dx/dt = t/(2f) = 0).
        let d = p_curve.derivative_at(0.0);
        // The derivative at t=0 is (0, 1, 0) in XY plane (y_axis direction).
        assert!(d.x.abs() < 1e-12, "dx/dt at vertex should be 0, got {}", d.x);
        assert!((d.y - 1.0).abs() < 1e-12, "dy/dt at vertex should be 1, got {}", d.y);
        assert!(d.z.abs() < 1e-12, "dz/dt at vertex should be 0, got {}", d.z);
    }

    #[test]
    fn test_parabola_derivative_matches_numerical() {
        let p_curve = Parabola::new_xy(Point3d::ORIGIN, 2.0);
        let dt = 1e-7;
        for &t in &[0.5_f64, 1.0, 2.0, -1.5] {
            let analytical = p_curve.derivative_at(t);
            let p_plus = p_curve.point_at(t + dt);
            let p_minus = p_curve.point_at(t - dt);
            let numerical_x = (p_plus.x - p_minus.x) / (2.0 * dt);
            let numerical_y = (p_plus.y - p_minus.y) / (2.0 * dt);
            let numerical_z = (p_plus.z - p_minus.z) / (2.0 * dt);
            assert!((analytical.x - numerical_x).abs() < 1e-6,
                "dx/dt mismatch at t={}: analytical={}, numerical={}", t, analytical.x, numerical_x);
            assert!((analytical.y - numerical_y).abs() < 1e-6,
                "dy/dt mismatch at t={}: analytical={}, numerical={}", t, analytical.y, numerical_y);
            assert!((analytical.z - numerical_z).abs() < 1e-6,
                "dz/dt mismatch at t={}: analytical={}, numerical={}", t, analytical.z, numerical_z);
        }
    }

    #[test]
    fn test_parabola_degenerate_zero_focal() {
        let p_curve = Parabola::new_xy(Point3d::ORIGIN, 0.0);
        let curve = Curve3d::Parabola(p_curve);
        assert!(curve.is_degenerate(1e-6),
            "A parabola with zero focal distance should be degenerate");
    }

    #[test]
    fn test_parabola_not_degenerate() {
        let p_curve = Parabola::new_xy(Point3d::ORIGIN, 5.0);
        let curve = Curve3d::Parabola(p_curve);
        assert!(!curve.is_degenerate(1e-6),
            "A parabola with non-zero focal distance should not be degenerate");
    }

    // ─── Curve3d::derivative_at dispatch test ──────────────────────────

    #[test]
    fn test_curve3d_derivative_at_uses_analytical_for_nurbs() {
        // This is a regression test: Curve3d::derivative_at for Nurbs should
        // now use the analytical quotient-rule implementation instead of
        // falling back to numerical central differences.
        let pts = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 2.0, 0.0),
            Point3d::new(2.0, 0.0, 0.0),
        ];
        let nurbs = NurbsCurve {
            degree: 2,
            control_points: pts,
            weights: vec![1.0, 1.0, 1.0],
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        };
        let curve = Curve3d::Nurbs(nurbs);

        // At t=0.5 (midpoint), derivative should match NurbsCurve::derivative_at
        let via_curve3d = curve.derivative_at(0.5);
        if let Curve3d::Nurbs(n) = &curve {
            let via_nurbs = n.derivative_at(0.5);
            assert!((via_curve3d.x - via_nurbs.x).abs() < 1e-12);
            assert!((via_curve3d.y - via_nurbs.y).abs() < 1e-12);
            assert!((via_curve3d.z - via_nurbs.z).abs() < 1e-12);
        } else {
            panic!("Expected Nurbs variant");
        }
    }
}
