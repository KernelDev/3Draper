//! Direction and axis primitives.

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// A unit direction vector in 3D.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Direction3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Direction3 {
    pub const X: Direction3 = Direction3 { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Direction3 = Direction3 { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Direction3 = Direction3 { x: 0.0, y: 0.0, z: 1.0 };
    pub const NEG_X: Direction3 = Direction3 { x: -1.0, y: 0.0, z: 0.0 };
    pub const NEG_Y: Direction3 = Direction3 { x: 0.0, y: -1.0, z: 0.0 };
    pub const NEG_Z: Direction3 = Direction3 { x: 0.0, y: 0.0, z: -1.0 };

    pub fn new(x: f64, y: f64, z: f64) -> Option<Self> {
        let len = (x * x + y * y + z * z).sqrt();
        if len < 1e-10 {
            None
        } else {
            Some(Self { x: x / len, y: y / len, z: z / len })
        }
    }

    /// Create without normalizing (assume already unit length).
    pub fn new_unchecked(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn to_dvec3(self) -> DVec3 {
        DVec3::new(self.x, self.y, self.z)
    }

    pub fn from_dvec3(v: DVec3) -> Option<Self> {
        Self::new(v.x, v.y, v.z)
    }

    pub fn cross(self, other: Direction3) -> Direction3 {
        let v = self.to_dvec3().cross(other.to_dvec3());
        Direction3::new(v.x, v.y, v.z).unwrap_or(Direction3::Z)
    }

    pub fn dot(self, other: Direction3) -> f64 {
        self.to_dvec3().dot(other.to_dvec3())
    }
}

/// A coordinate system defined by an origin and three orthogonal axes.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Axis2Placement3D {
    /// Origin point.
    pub location: crate::point::Point3,
    /// Z-axis direction (normal).
    pub axis: Direction3,
    /// X-axis direction (ref direction).
    pub ref_direction: Direction3,
}

impl Axis2Placement3D {
    pub fn new(
        location: crate::point::Point3,
        axis: Direction3,
        ref_direction: Option<Direction3>,
    ) -> Self {
        let z = axis;
        let x = ref_direction.unwrap_or_else(|| {
            // Compute a perpendicular direction
            let zv = z.to_dvec3();
            let perp = if zv.dot(DVec3::X).abs() < 0.9 {
                zv.cross(DVec3::X)
            } else {
                zv.cross(DVec3::Y)
            };
            Direction3::new(perp.x, perp.y, perp.z).unwrap_or(Direction3::X)
        });

        // Ensure orthogonality
        let xv = x.to_dvec3();
        let zv = z.to_dvec3();
        let yv = zv.cross(xv);
        let xv = yv.cross(zv);
        let x = Direction3::new(xv.x, xv.y, xv.z).unwrap_or(Direction3::X);

        Self { location, axis: z, ref_direction: x }
    }

    pub fn y_direction(&self) -> Direction3 {
        self.axis.cross(self.ref_direction)
    }

    pub fn to_transform(self) -> crate::transform::Transform3 {
        crate::transform::Transform3::from_axis2_placement(self)
    }
}

/// A 2D direction (in UV parameter space).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Direction2 {
    pub u: f64,
    pub v: f64,
}

impl Direction2 {
    pub const U: Direction2 = Direction2 { u: 1.0, v: 0.0 };
    pub const V: Direction2 = Direction2 { u: 0.0, v: 1.0 };

    pub fn new(u: f64, v: f64) -> Option<Self> {
        let len = (u * u + v * v).sqrt();
        if len < 1e-10 {
            None
        } else {
            Some(Self { u: u / len, v: v / len })
        }
    }

    pub fn new_unchecked(u: f64, v: f64) -> Self {
        Self { u, v }
    }
}
