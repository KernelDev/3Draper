//! Curve primitives — parametric curves in 3D space.

use crate::direction::Direction3;
use crate::point::Point3;
use crate::transform::Transform3;
use serde::{Deserialize, Serialize};

/// A parametric curve in 3D.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Curve {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
    BSplineCurve(BSplineCurve),
    OffsetCurve(OffsetCurve),
    TrimmedCurve(TrimmedCurve),
}

impl Curve {
    /// Evaluate the curve at parameter t.
    pub fn point_at(&self, t: f64) -> Point3 {
        match self {
            Curve::Line(c) => c.point_at(t),
            Curve::Circle(c) => c.point_at(t),
            Curve::Ellipse(c) => c.point_at(t),
            Curve::BSplineCurve(c) => c.point_at(t),
            Curve::OffsetCurve(c) => c.point_at(t),
            Curve::TrimmedCurve(c) => c.basis_curve.point_at(
                c.trim1 + t * (c.trim2 - c.trim1),
            ),
        }
    }

    /// Get the bounding box of this curve by sampling.
    pub fn bounding_box(&self, samples: usize) -> crate::point::BoundingBox3 {
        let mut bb = crate::point::BoundingBox3::empty();
        for i in 0..=samples {
            let t = i as f64 / samples as f64;
            bb.extend(self.point_at(t));
        }
        bb
    }

    pub fn transform(&self, tf: &Transform3) -> Curve {
        match self {
            Curve::Line(c) => Curve::Line(c.transform(tf)),
            Curve::Circle(c) => Curve::Circle(c.transform(tf)),
            Curve::Ellipse(c) => Curve::Ellipse(c.transform(tf)),
            Curve::BSplineCurve(c) => Curve::BSplineCurve(c.transform(tf)),
            Curve::OffsetCurve(c) => Curve::OffsetCurve(c.transform(tf)),
            Curve::TrimmedCurve(c) => Curve::TrimmedCurve(TrimmedCurve {
                basis_curve: Box::new(c.basis_curve.transform(tf)),
                trim1: c.trim1,
                trim2: c.trim2,
                sense: c.sense,
            }),
        }
    }
}

/// An infinite line through a point in a given direction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Line {
    pub origin: Point3,
    pub direction: Direction3,
}

impl Line {
    pub fn new(origin: Point3, direction: Direction3) -> Self {
        Self { origin, direction }
    }

    /// Parameter t gives: origin + t * direction
    pub fn point_at(&self, t: f64) -> Point3 {
        self.origin + self.direction.to_dvec3() * t
    }

    pub fn transform(&self, tf: &Transform3) -> Line {
        Line {
            origin: tf.transform_point(self.origin),
            direction: tf.transform_direction(self.direction),
        }
    }
}

/// A circle in 3D space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circle {
    /// Position: center is at axis.location, normal is axis.axis, 
    /// ref_direction defines where parameter 0 is.
    pub axis: crate::direction::Axis2Placement3D,
    pub radius: f64,
}

impl Circle {
    pub fn new(axis: crate::direction::Axis2Placement3D, radius: f64) -> Self {
        Self { axis, radius }
    }

    /// Evaluate at parameter t (0 to 2*PI).
    pub fn point_at(&self, t: f64) -> Point3 {
        let x_dir = self.axis.ref_direction.to_dvec3();
        let y_dir = self.axis.y_direction().to_dvec3();
        let center = self.axis.location.to_dvec3();

        let pt = center + x_dir * (self.radius * t.cos()) + y_dir * (self.radius * t.sin());
        Point3::from_dvec3(pt)
    }

    pub fn transform(&self, tf: &Transform3) -> Circle {
        Circle {
            axis: crate::direction::Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
            radius: self.radius * tf.scale(),
        }
    }
}

/// An ellipse in 3D space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ellipse {
    pub axis: crate::direction::Axis2Placement3D,
    pub semi_axis_1: f64,
    pub semi_axis_2: f64,
}

impl Ellipse {
    pub fn new(axis: crate::direction::Axis2Placement3D, semi_axis_1: f64, semi_axis_2: f64) -> Self {
        Self { axis, semi_axis_1, semi_axis_2 }
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        let x_dir = self.axis.ref_direction.to_dvec3();
        let y_dir = self.axis.y_direction().to_dvec3();
        let center = self.axis.location.to_dvec3();

        let pt = center
            + x_dir * (self.semi_axis_1 * t.cos())
            + y_dir * (self.semi_axis_2 * t.sin());
        Point3::from_dvec3(pt)
    }

    pub fn transform(&self, tf: &Transform3) -> Ellipse {
        Ellipse {
            axis: crate::direction::Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
            semi_axis_1: self.semi_axis_1 * tf.scale(),
            semi_axis_2: self.semi_axis_2 * tf.scale(),
        }
    }
}

/// A B-Spline curve.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BSplineCurve {
    /// Control points.
    pub poles: Vec<Point3>,
    /// Knot vector.
    pub knots: Vec<f64>,
    /// Multiplicity of each knot.
    pub multiplicities: Vec<u32>,
    /// Degree of the curve.
    pub degree: u32,
    /// Whether the curve is periodic.
    pub periodic: bool,
}

impl BSplineCurve {
    /// Evaluate the B-Spline curve at parameter t using De Boor's algorithm.
    pub fn point_at(&self, t: f64) -> Point3 {
        if self.poles.is_empty() {
            return Point3::ORIGIN;
        }

        // Clamp t to knot range
        let knot_min = self.knots.first().copied().unwrap_or(0.0);
        let knot_max = self.knots.last().copied().unwrap_or(1.0);
        let t = t.clamp(knot_min, knot_max);

        // Simple evaluation: for a minimal implementation, use linear interpolation
        // between control points when degree is low, or De Boor for higher degrees.
        if self.degree == 1 {
            // Linear B-spline: piecewise linear
            let n = self.poles.len();
            if n == 1 {
                return self.poles[0];
            }
            // Find the segment
            let knot_range = knot_max - knot_min;
            let seg_t = if knot_range > 0.0 { (t - knot_min) / knot_range } else { 0.0 };
            let idx = ((seg_t * (n - 1) as f64).floor() as usize).min(n - 2);
            let local_t = seg_t * (n - 1) as f64 - idx as f64;
            return self.poles[idx].lerp(self.poles[idx + 1], local_t);
        }

        // For higher degrees, use De Boor's algorithm
        self.de_boor(t)
    }

    fn de_boor(&self, t: f64) -> Point3 {
        let degree = self.degree as usize;
        // Find the knot span
        let k = self.find_knot_span(t);
        
        // Create a copy of relevant control points
        let mut pts: Vec<glam::DVec3> = (0..=degree)
            .map(|i| self.poles[k - degree + i].to_dvec3())
            .collect();

        // De Boor recursion
        for r in 1..=degree {
            for j in (r..=degree).rev() {
                let idx = k - degree + j;
                let knot_i = self.knot_at(idx);
                let knot_ipr = self.knot_at(idx + degree + 1 - r);
                let denom = knot_ipr - knot_i;
                let alpha = if denom.abs() < 1e-10 {
                    0.0
                } else {
                    (t - knot_i) / denom
                };
                pts[j] = pts[j - 1].lerp(pts[j], alpha);
            }
        }

        Point3::from_dvec3(pts[degree])
    }

    fn find_knot_span(&self, t: f64) -> usize {
        let n = self.poles.len();
        let degree = self.degree as usize;
        
        // Expand knots with multiplicities
        let expanded = self.expanded_knots();
        
        // Binary search for the knot span
        let mut lo = degree;
        let mut hi = n;
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if expanded[mid] <= t {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    fn knot_at(&self, idx: usize) -> f64 {
        let expanded = self.expanded_knots();
        expanded.get(idx).copied().unwrap_or(expanded.last().copied().unwrap_or(0.0))
    }

    fn expanded_knots(&self) -> Vec<f64> {
        let mut result = Vec::new();
        for (knot, &mult) in self.knots.iter().zip(self.multiplicities.iter()) {
            for _ in 0..mult {
                result.push(*knot);
            }
        }
        result
    }

    pub fn transform(&self, tf: &Transform3) -> BSplineCurve {
        BSplineCurve {
            poles: self.poles.iter().map(|p| tf.transform_point(*p)).collect(),
            knots: self.knots.clone(),
            multiplicities: self.multiplicities.clone(),
            degree: self.degree,
            periodic: self.periodic,
        }
    }
}

/// An offset curve: a curve at a constant distance from a basis curve.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffsetCurve {
    pub basis_curve: Box<Curve>,
    pub distance: f64,
    pub direction: Direction3,
}

impl OffsetCurve {
    pub fn point_at(&self, t: f64) -> Point3 {
        let basis_pt = self.basis_curve.point_at(t);
        basis_pt + self.direction.to_dvec3() * self.distance
    }

    pub fn transform(&self, tf: &Transform3) -> OffsetCurve {
        OffsetCurve {
            basis_curve: Box::new(self.basis_curve.transform(tf)),
            distance: self.distance * tf.scale(),
            direction: tf.transform_direction(self.direction),
        }
    }
}

/// A trimmed curve: a curve limited to a parameter range.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimmedCurve {
    pub basis_curve: Box<Curve>,
    pub trim1: f64,
    pub trim2: f64,
    pub sense: bool, // true = same direction as basis
}
