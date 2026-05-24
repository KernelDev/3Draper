//! 3D Vector type (non-unit-length).

use std::fmt;

/// A vector in 3D space (not necessarily unit length).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3d {
    pub const ZERO: Vec3d = Vec3d { x: 0.0, y: 0.0, z: 0.0 };
    pub const X: Vec3d = Vec3d { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Vec3d = Vec3d { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Vec3d = Vec3d { x: 0.0, y: 0.0, z: 1.0 };

    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn length(&self) -> f64 {
        self.x.hypot(self.y.hypot(self.z))
    }

    #[inline]
    pub fn length_sq(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline]
    pub fn dot(&self, other: &Vec3d) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    pub fn cross(&self, other: &Vec3d) -> Vec3d {
        Vec3d {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    #[inline]
    pub fn normalize(&self) -> Option<crate::direction::Direction3d> {
        crate::direction::Direction3d::new(self.x, self.y, self.z)
    }

    #[inline]
    pub fn scale(&self, s: f64) -> Vec3d {
        Vec3d { x: self.x * s, y: self.y * s, z: self.z * s }
    }

    #[inline]
    pub fn add(&self, other: &Vec3d) -> Vec3d {
        Vec3d { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }

    #[inline]
    pub fn sub(&self, other: &Vec3d) -> Vec3d {
        Vec3d { x: self.x - other.x, y: self.y - other.y, z: self.z - other.z }
    }

    #[inline]
    pub fn neg(&self) -> Vec3d {
        Vec3d { x: -self.x, y: -self.y, z: -self.z }
    }
}

impl fmt::Display for Vec3d {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "vec({}, {}, {})", self.x, self.y, self.z)
    }
}

/// 2D vector for parametric space computations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2d {
    pub u: f64,
    pub v: f64,
}

impl Vec2d {
    pub const ZERO: Vec2d = Vec2d { u: 0.0, v: 0.0 };

    #[inline]
    pub fn new(u: f64, v: f64) -> Self {
        Self { u, v }
    }

    #[inline]
    pub fn length(&self) -> f64 {
        self.u.hypot(self.v)
    }

    #[inline]
    pub fn dot(&self, other: &Vec2d) -> f64 {
        self.u * other.u + self.v * other.v
    }

    #[inline]
    pub fn cross(&self, other: &Vec2d) -> f64 {
        self.u * other.v - self.v * other.u
    }
}
