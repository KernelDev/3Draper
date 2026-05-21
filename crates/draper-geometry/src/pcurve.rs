//! Parametric curves in 2D (UV space) — pcurves.
//!
//! A pcurve is a 2D curve in the parameter space (u,v) of a surface.
//! Every edge that bounds a face has a pcurve that maps the edge's 3D curve
//! into the face's surface parameter space. This is essential for:
//! - Constrained Delaunay Triangulation in UV space
//! - Seam detection on periodic surfaces
//! - Consistent boundary representation across shared edges

use crate::point::Point2;
use serde::{Deserialize, Serialize};

/// A 2D parametric curve in UV space (pcurve).
///
/// Represents the projection of a 3D edge curve into a surface's parameter space.
/// Each edge can have multiple pcurves — one per face it borders.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PCurve {
    /// A straight line segment in UV space.
    Line(PCurveLine),
    /// A circular arc in UV space.
    Circle(PCurveCircle),
    /// An elliptical arc in UV space.
    Ellipse(PCurveEllipse),
    /// A B-spline curve in UV space.
    BSpline(PCurveBSpline),
    /// A polyline — a sequence of UV points.
    Polyline(Vec<Point2>),
}

impl PCurve {
    /// Evaluate the pcurve at parameter t (0..=1).
    pub fn point_at(&self, t: f64) -> Point2 {
        match self {
            PCurve::Line(l) => l.point_at(t),
            PCurve::Circle(c) => c.point_at(t),
            PCurve::Ellipse(e) => e.point_at(t),
            PCurve::BSpline(b) => b.point_at(t),
            PCurve::Polyline(pts) => {
                if pts.len() < 2 {
                    return pts.first().copied().unwrap_or(Point2::ORIGIN);
                }
                let n = pts.len() - 1;
                let seg = (t * n as f64).min(n as f64 - 1e-10);
                let i = seg.floor() as usize;
                let local_t = seg - i as f64;
                let i = i.min(n - 1);
                Point2::new(
                    pts[i].u + local_t * (pts[i + 1].u - pts[i].u),
                    pts[i].v + local_t * (pts[i + 1].v - pts[i].v),
                )
            }
        }
    }

    /// Evaluate the tangent direction at parameter t.
    pub fn tangent_at(&self, t: f64) -> (f64, f64) {
        let eps = 1e-7;
        let p0 = self.point_at(t - eps);
        let p1 = self.point_at(t + eps);
        (p1.u - p0.u, p1.v - p0.v)
    }

    /// Compute the length of the pcurve by sampling.
    pub fn length(&self, samples: usize) -> f64 {
        if samples < 2 {
            return 0.0;
        }
        let mut len = 0.0;
        let mut prev = self.point_at(0.0);
        for i in 1..=samples {
            let t = i as f64 / samples as f64;
            let cur = self.point_at(t);
            let du = cur.u - prev.u;
            let dv = cur.v - prev.v;
            len += (du * du + dv * dv).sqrt();
            prev = cur;
        }
        len
    }

    /// Sample the pcurve at regular parameter intervals.
    pub fn sample(&self, n: usize) -> Vec<Point2> {
        (0..=n).map(|i| self.point_at(i as f64 / n as f64)).collect()
    }
}

/// A line segment in UV space from `start` to `end`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PCurveLine {
    pub start: Point2,
    pub end: Point2,
}

impl PCurveLine {
    pub fn new(start: Point2, end: Point2) -> Self {
        Self { start, end }
    }

    pub fn point_at(&self, t: f64) -> Point2 {
        Point2::new(
            self.start.u + t * (self.end.u - self.start.u),
            self.start.v + t * (self.end.v - self.start.v),
        )
    }
}

/// A circular arc in UV space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PCurveCircle {
    pub center: Point2,
    pub radius: f64,
    /// Start angle in radians.
    pub start_angle: f64,
    /// End angle in radians.
    pub end_angle: f64,
}

impl PCurveCircle {
    pub fn point_at(&self, t: f64) -> Point2 {
        let angle = self.start_angle + t * (self.end_angle - self.start_angle);
        Point2::new(
            self.center.u + self.radius * angle.cos(),
            self.center.v + self.radius * angle.sin(),
        )
    }
}

/// An elliptical arc in UV space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PCurveEllipse {
    pub center: Point2,
    pub semi_axis_u: f64,
    pub semi_axis_v: f64,
    pub start_angle: f64,
    pub end_angle: f64,
}

impl PCurveEllipse {
    pub fn point_at(&self, t: f64) -> Point2 {
        let angle = self.start_angle + t * (self.end_angle - self.start_angle);
        Point2::new(
            self.center.u + self.semi_axis_u * angle.cos(),
            self.center.v + self.semi_axis_v * angle.sin(),
        )
    }
}

/// A B-spline curve in UV space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PCurveBSpline {
    pub poles: Vec<Point2>,
    pub knots: Vec<f64>,
    pub multiplicities: Vec<u32>,
    pub degree: u32,
}

impl PCurveBSpline {
    /// Simplified evaluation — for now, use piecewise linear through poles.
    pub fn point_at(&self, t: f64) -> Point2 {
        if self.poles.is_empty() {
            return Point2::ORIGIN;
        }
        if self.poles.len() == 1 {
            return self.poles[0];
        }

        // Expand knots and find span
        let expanded = self.expanded_knots();
        let knot_min = expanded.first().copied().unwrap_or(0.0);
        let knot_max = expanded.last().copied().unwrap_or(1.0);
        let t = t.clamp(knot_min, knot_max);

        // De Boor evaluation (simplified for degree 1, basic for higher)
        if self.degree <= 1 {
            let n = self.poles.len();
            let range = knot_max - knot_min;
            if range.abs() < 1e-10 {
                return self.poles[0];
            }
            let seg_t = (t - knot_min) / range;
            let idx = ((seg_t * (n - 1) as f64).floor() as usize).min(n - 2);
            let local_t = seg_t * (n - 1) as f64 - idx as f64;
            return Point2::new(
                self.poles[idx].u + local_t * (self.poles[idx + 1].u - self.poles[idx].u),
                self.poles[idx].v + local_t * (self.poles[idx + 1].v - self.poles[idx].v),
            );
        }

        // Higher degree: use De Boor
        self.de_boor(t)
    }

    fn de_boor(&self, t: f64) -> Point2 {
        let degree = self.degree as usize;
        let expanded = self.expanded_knots();
        let n = self.poles.len();

        // Find knot span
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
        let k = lo;

        // Create copy of relevant poles
        let mut pts: Vec<Point2> = (0..=degree)
            .map(|i| self.poles[k - degree + i])
            .collect();

        // De Boor recursion
        for r in 1..=degree {
            for j in (r..=degree).rev() {
                let idx = k - degree + j;
                let knot_i = expanded.get(idx).copied().unwrap_or(0.0);
                let knot_ipr = expanded.get(idx + degree + 1 - r).copied().unwrap_or(0.0);
                let denom = knot_ipr - knot_i;
                let alpha = if denom.abs() < 1e-10 {
                    0.0
                } else {
                    (t - knot_i) / denom
                };
                pts[j] = Point2::new(
                    pts[j - 1].u + alpha * (pts[j].u - pts[j - 1].u),
                    pts[j - 1].v + alpha * (pts[j].v - pts[j - 1].v),
                );
            }
        }

        pts[degree]
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
}

/// Information about a pcurve associated with an edge on a specific face.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PCurveOnFace {
    /// The pcurve in UV space.
    pub pcurve: PCurve,
    /// The face this pcurve belongs to.
    pub face_id: u64,
    /// Whether this edge is a seam on the surface.
    pub is_seam: bool,
    /// UV range for the pcurve parameterization.
    pub uv_range: Option<(Point2, Point2)>,
}
