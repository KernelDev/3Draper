//! 2D and 3D point types.

use crate::tolerance::{is_coincident, TOLERANCE};
use nalgebra::{Point3, Vector3};
use std::fmt;

/// A point in 3D space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3d {
    pub const ORIGIN: Point3d = Point3d { x: 0.0, y: 0.0, z: 0.0 };

    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Distance to another point.
    #[inline]
    pub fn distance_to(&self, other: &Point3d) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx.hypot(dy.hypot(dz))
    }

    /// Squared distance (avoids sqrt).
    #[inline]
    pub fn distance_sq_to(&self, other: &Point3d) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Check if two points are within geometric tolerance.
    #[inline]
    pub fn is_coincident_with(&self, other: &Point3d) -> bool {
        self.distance_sq_to(other) < TOLERANCE * TOLERANCE
    }

    /// Midpoint between two points.
    #[inline]
    pub fn midpoint(&self, other: &Point3d) -> Point3d {
        Point3d {
            x: (self.x + other.x) * 0.5,
            y: (self.y + other.y) * 0.5,
            z: (self.z + other.z) * 0.5,
        }
    }

    /// Convert to nalgebra Point3.
    #[inline]
    pub fn to_na(&self) -> Point3<f64> {
        Point3::new(self.x, self.y, self.z)
    }

    /// Convert from nalgebra Point3.
    #[inline]
    pub fn from_na(p: &Point3<f64>) -> Self {
        Self { x: p.x, y: p.y, z: p.z }
    }

    /// Barycentric combination of two points.
    #[inline]
    pub fn lerp(&self, other: &Point3d, t: f64) -> Point3d {
        Point3d {
            x: self.x * (1.0 - t) + other.x * t,
            y: self.y * (1.0 - t) + other.y * t,
            z: self.z * (1.0 - t) + other.z * t,
        }
    }

    /// Apply a transformation matrix (4x4 homogeneous).
    pub fn transform(&self, m: &[[f64; 4]; 4]) -> Point3d {
        let x = m[0][0] * self.x + m[0][1] * self.y + m[0][2] * self.z + m[0][3];
        let y = m[1][0] * self.x + m[1][1] * self.y + m[1][2] * self.z + m[1][3];
        let z = m[2][0] * self.x + m[2][1] * self.y + m[2][2] * self.z + m[2][3];
        let w = m[3][0] * self.x + m[3][1] * self.y + m[3][2] * self.z + m[3][3];
        if w.abs() > 1e-15 {
            Point3d::new(x / w, y / w, z / w)
        } else {
            Point3d::new(x, y, z)
        }
    }
}

impl fmt::Display for Point3d {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}

/// A point in 2D parametric space (u, v).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point2d {
    pub u: f64,
    pub v: f64,
}

impl Point2d {
    pub const ORIGIN: Point2d = Point2d { u: 0.0, v: 0.0 };

    #[inline]
    pub fn new(u: f64, v: f64) -> Self {
        Self { u, v }
    }

    #[inline]
    pub fn distance_to(&self, other: &Point2d) -> f64 {
        let du = self.u - other.u;
        let dv = self.v - other.v;
        du.hypot(dv)
    }

    #[inline]
    pub fn distance_sq_to(&self, other: &Point2d) -> f64 {
        let du = self.u - other.u;
        let dv = self.v - other.v;
        du * du + dv * dv
    }

    #[inline]
    pub fn is_coincident_with(&self, other: &Point2d) -> bool {
        self.distance_sq_to(other) < TOLERANCE * TOLERANCE
    }

    #[inline]
    pub fn lerp(&self, other: &Point2d, t: f64) -> Point2d {
        Point2d {
            u: self.u * (1.0 - t) + other.u * t,
            v: self.v * (1.0 - t) + other.v * t,
        }
    }
}

impl fmt::Display for Point2d {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.u, self.v)
    }
}
