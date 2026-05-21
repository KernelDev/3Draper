//! 3D Transformation matrix.

use crate::direction::Direction3;
use crate::point::Point3;
use glam::{DAffine3, DVec3, EulerRot};
use serde::{Deserialize, Serialize};

/// A 3D affine transformation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform3 {
    /// The 4x3 affine matrix (rotation + translation).
    matrix: DAffine3,
}

impl Transform3 {
    pub const IDENTITY: Transform3 = Transform3 {
        matrix: DAffine3::IDENTITY,
    };

    pub fn new(matrix: DAffine3) -> Self {
        Self { matrix }
    }

    pub fn from_translation(tx: f64, ty: f64, tz: f64) -> Self {
        Self {
            matrix: DAffine3::from_translation(DVec3::new(tx, ty, tz)),
        }
    }

    pub fn from_scale(s: f64) -> Self {
        Self {
            matrix: DAffine3::from_scale(DVec3::new(s, s, s)),
        }
    }

    pub fn from_rotation_x(angle: f64) -> Self {
        Self {
            matrix: DAffine3::from_rotation_x(angle),
        }
    }

    pub fn from_rotation_y(angle: f64) -> Self {
        Self {
            matrix: DAffine3::from_rotation_y(angle),
        }
    }

    pub fn from_rotation_z(angle: f64) -> Self {
        Self {
            matrix: DAffine3::from_rotation_z(angle),
        }
    }

    /// Create from an Axis2Placement3D.
    pub fn from_axis2_placement(axis: crate::direction::Axis2Placement3D) -> Self {
        let x = axis.ref_direction.to_dvec3();
        let y = axis.y_direction().to_dvec3();
        let z = axis.axis.to_dvec3();
        let t = axis.location.to_dvec3();

        Self {
            matrix: DAffine3::from_cols(
                DVec3::new(x.x, y.x, z.x),
                DVec3::new(x.y, y.y, z.y),
                DVec3::new(x.z, y.z, z.z),
                t,
            ),
        }
    }

    pub fn transform_point(&self, point: Point3) -> Point3 {
        Point3::from_dvec3(self.matrix.transform_point3(point.to_dvec3()))
    }

    pub fn transform_direction(&self, dir: Direction3) -> Direction3 {
        let v = self.matrix.transform_vector3(dir.to_dvec3());
        Direction3::new(v.x, v.y, v.z).unwrap_or(dir)
    }

    pub fn inverse(&self) -> Transform3 {
        Transform3 {
            matrix: self.matrix.inverse(),
        }
    }

    pub fn then(&self, other: &Transform3) -> Transform3 {
        Transform3 {
            matrix: self.matrix * other.matrix,
        }
    }

    /// Get the uniform scale factor (from the matrix diagonal).
    pub fn scale(&self) -> f64 {
        let x_len = DVec3::new(self.matrix.matrix3.x_axis.x, self.matrix.matrix3.y_axis.x, self.matrix.matrix3.z_axis.x).length();
        x_len
    }

    pub fn to_daffine3(&self) -> DAffine3 {
        self.matrix
    }

    pub fn from_daffine3(m: DAffine3) -> Self {
        Self { matrix: m }
    }
}

impl Default for Transform3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl std::ops::Mul for Transform3 {
    type Output = Transform3;
    fn mul(self, rhs: Transform3) -> Self::Output {
        self.then(&rhs)
    }
}
