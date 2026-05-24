//! Direction (unit vector) in 3D space.

use crate::tolerance::is_zero;
use std::fmt;

/// A unit direction vector in 3D space. Always normalized.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Direction3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Direction3d {
    pub const X: Direction3d = Direction3d { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Direction3d = Direction3d { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Direction3d = Direction3d { x: 0.0, y: 0.0, z: 1.0 };
    pub const NEG_X: Direction3d = Direction3d { x: -1.0, y: 0.0, z: 0.0 };
    pub const NEG_Y: Direction3d = Direction3d { x: 0.0, y: -1.0, z: 0.0 };
    pub const NEG_Z: Direction3d = Direction3d { x: 0.0, y: 0.0, z: -1.0 };

    /// Create a direction from components, normalizing the result.
    /// Returns None if the vector has zero length.
    pub fn new(x: f64, y: f64, z: f64) -> Option<Self> {
        let len = (x * x + y * y + z * z).sqrt();
        if is_zero(len) {
            None
        } else {
            Some(Self { x: x / len, y: y / len, z: z / len })
        }
    }

    /// Create a direction without normalization (caller guarantees unit length).
    #[inline]
    pub const fn new_unchecked(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Cross product with another direction.
    #[inline]
    pub fn cross(&self, other: &Direction3d) -> Direction3d {
        let cx = self.y * other.z - self.z * other.y;
        let cy = self.z * other.x - self.x * other.z;
        let cz = self.x * other.y - self.y * other.x;
        // Result should be unit length since inputs are unit
        let len = (cx * cx + cy * cy + cz * cz).sqrt();
        if is_zero(len) {
            Direction3d::Z // Fallback
        } else {
            Direction3d { x: cx / len, y: cy / len, z: cz / len }
        }
    }

    /// Dot product with another direction.
    #[inline]
    pub fn dot(&self, other: &Direction3d) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Angle between two directions in radians [0, pi].
    pub fn angle_to(&self, other: &Direction3d) -> f64 {
        let d = self.dot(other).clamp(-1.0, 1.0);
        d.acos()
    }

    /// Negate the direction.
    #[inline]
    pub fn neg(&self) -> Direction3d {
        Direction3d { x: -self.x, y: -self.y, z: -self.z }
    }

    /// Check if two directions are parallel (same or opposite).
    pub fn is_parallel_to(&self, other: &Direction3d) -> bool {
        let d = self.dot(other).abs();
        d > 1.0 - 1e-10
    }

    /// Check if two directions are perpendicular.
    pub fn is_perpendicular_to(&self, other: &Direction3d) -> bool {
        self.dot(other).abs() < 1e-10
    }
}

impl fmt::Display for Direction3d {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "dir({}, {}, {})", self.x, self.y, self.z)
    }
}
