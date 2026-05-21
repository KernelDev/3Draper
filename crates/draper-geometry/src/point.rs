//! 3D Point and Vector primitives.

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// A 3D point in space.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    pub const ORIGIN: Point3 = Point3 { x: 0.0, y: 0.0, z: 0.0 };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn from_dvec3(v: DVec3) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }

    pub fn to_dvec3(self) -> DVec3 {
        DVec3::new(self.x, self.y, self.z)
    }

    pub fn distance_to(self, other: Point3) -> f64 {
        (self.to_dvec3() - other.to_dvec3()).length()
    }

    pub fn midpoint(self, other: Point3) -> Point3 {
        Point3::from_dvec3((self.to_dvec3() + other.to_dvec3()) * 0.5)
    }

    /// Linearly interpolate between this point and another.
    pub fn lerp(self, other: Point3, t: f64) -> Point3 {
        Point3::from_dvec3(self.to_dvec3().lerp(other.to_dvec3(), t))
    }
}

impl std::ops::Add<glam::DVec3> for Point3 {
    type Output = Point3;
    fn add(self, rhs: glam::DVec3) -> Self::Output {
        Point3::from_dvec3(self.to_dvec3() + rhs)
    }
}

impl std::ops::Sub for Point3 {
    type Output = glam::DVec3;
    fn sub(self, rhs: Self) -> Self::Output {
        self.to_dvec3() - rhs.to_dvec3()
    }
}

/// A 2D point (for UV parameter space).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point2 {
    pub u: f64,
    pub v: f64,
}

impl Point2 {
    pub const ORIGIN: Point2 = Point2 { u: 0.0, v: 0.0 };

    pub fn new(u: f64, v: f64) -> Self {
        Self { u, v }
    }

    pub fn distance_to(self, other: Point2) -> f64 {
        ((self.u - other.u).powi(2) + (self.v - other.v).powi(2)).sqrt()
    }
}

/// A bounded 3D box.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

impl BoundingBox3 {
    pub fn new(min: Point3, max: Point3) -> Self {
        Self { min, max }
    }

    pub fn empty() -> Self {
        Self {
            min: Point3::new(f64::MAX, f64::MAX, f64::MAX),
            max: Point3::new(f64::MIN, f64::MIN, f64::MIN),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }

    pub fn extend(&mut self, point: Point3) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    pub fn union(&self, other: &BoundingBox3) -> BoundingBox3 {
        let mut result = *self;
        result.extend(other.min);
        result.extend(other.max);
        result
    }

    pub fn center(&self) -> Point3 {
        Point3::from_dvec3((self.min.to_dvec3() + self.max.to_dvec3()) * 0.5)
    }

    pub fn size(&self) -> glam::DVec3 {
        self.max.to_dvec3() - self.min.to_dvec3()
    }

    pub fn diagonal(&self) -> f64 {
        self.size().length()
    }
}
