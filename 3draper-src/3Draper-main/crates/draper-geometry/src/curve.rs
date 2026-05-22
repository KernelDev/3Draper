//! Parametric curves in 3D space.

use crate::{Direction3d, Point3d, Point2d, Vec3d, Transform};
use std::fmt;

/// Parametric range [u_min, u_max].
pub type ParamRange = (f64, f64);

/// A parametric curve in 3D space.
#[derive(Clone, Debug)]
pub enum Curve3d {
    /// Line: P(t) = origin + t * direction
    Line(Line),
    /// Circle in 3D space defined by center, normal, radius
    Circle(Circle),
    /// Ellipse in 3D space
    Ellipse(Ellipse),
    /// Arc (trimmed circle segment)
    Arc(Arc),
    /// NURBS curve
    Nurbs(NurbsCurve),
}

/// A line in 3D.
#[derive(Clone, Debug)]
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
}

/// A circle in 3D space.
#[derive(Clone, Debug)]
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
}

/// An arc (trimmed circle segment).
#[derive(Clone, Debug)]
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
}

/// NURBS curve representation.
#[derive(Clone, Debug)]
pub struct NurbsCurve {
    pub degree: usize,
    pub control_points: Vec<Point3d>,
    pub weights: Vec<f64>,
    pub knots: Vec<f64>,
}

impl Curve3d {
    /// Evaluate the curve at parameter t.
    pub fn point_at(&self, t: f64) -> Point3d {
        match self {
            Curve3d::Line(line) => line.point_at(t),
            Curve3d::Circle(circle) => circle.point_at(t),
            Curve3d::Ellipse(ellipse) => ellipse.point_at(t),
            Curve3d::Arc(arc) => arc.point_at(t),
            Curve3d::Nurbs(nurbs) => nurbs_eval(nurbs, t),
        }
    }

    /// Get the parametric range of the curve.
    pub fn param_range(&self) -> ParamRange {
        match self {
            Curve3d::Line(_) => (-f64::MAX, f64::MAX),
            Curve3d::Circle(_) => (0.0, 2.0 * std::f64::consts::PI),
            Curve3d::Ellipse(_) => (0.0, 2.0 * std::f64::consts::PI),
            Curve3d::Arc(arc) => (arc.start_angle, arc.end_angle),
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
        return nurbs.control_points[0];
    }

    let p = nurbs.degree;
    // Find knot span
    let mut k = p;
    while k < nurbs.knots.len() - 1 && nurbs.knots[k + 1] < t {
        k += 1;
    }
    k = k.min(n - 1);

    // De Boor's algorithm
    let mut pts: Vec<(f64, f64, f64)> = Vec::with_capacity(p + 1);
    for i in 0..=p {
        let idx = (k - p + i).min(n - 1);
        let w = nurbs.weights[idx];
        pts.push((
            nurbs.control_points[idx].x * w,
            nurbs.control_points[idx].y * w,
            nurbs.control_points[idx].z * w,
        ));
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let i0 = k - p + j;
            let i1 = i0 + 1;
            let a = if i1 < nurbs.knots.len() && i0 + p - r + 1 < nurbs.knots.len() {
                let d = nurbs.knots[i0 + p - r + 1] - nurbs.knots[i1];
                if d.abs() < 1e-15 { 0.0 } else { (t - nurbs.knots[i1]) / d }
            } else {
                0.0
            };
            let b = 1.0 - a;
            pts[j] = (
                a * pts[j].0 + b * pts[j - 1].0,
                a * pts[j].1 + b * pts[j - 1].1,
                a * pts[j].2 + b * pts[j - 1].2,
            );
        }
    }

    let w = pts[p].0.hypot(pts[p].1.hypot(pts[p].2));
    if w.abs() < 1e-15 {
        Point3d::ORIGIN
    } else {
        Point3d::new(pts[p].0 / w, pts[p].1 / w, pts[p].2 / w)
    }
}
