//! 4x4 homogeneous transformation matrix.

use crate::{Direction3d, Point3d, Vec3d};

/// 4x4 homogeneous transformation matrix stored in row-major order.
#[derive(Clone, Debug)]
pub struct Transform {
    /// Row-major 4x4 matrix.
    pub m: [[f64; 4]; 4],
}

impl Transform {
    /// Identity transformation.
    pub const IDENTITY: Transform = Transform {
        m: [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    };

    /// Create identity transform.
    pub fn identity() -> Self {
        Self::IDENTITY
    }

    /// Create a translation transform.
    pub fn translation(dx: f64, dy: f64, dz: f64) -> Self {
        Transform {
            m: [
                [1.0, 0.0, 0.0, dx],
                [0.0, 1.0, 0.0, dy],
                [0.0, 0.0, 1.0, dz],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create a scaling transform.
    pub fn scaling(sx: f64, sy: f64, sz: f64) -> Self {
        Transform {
            m: [
                [sx, 0.0, 0.0, 0.0],
                [0.0, sy, 0.0, 0.0],
                [0.0, 0.0, sz, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create a uniform scaling transform.
    pub fn uniform_scaling(s: f64) -> Self {
        Self::scaling(s, s, s)
    }

    /// Create rotation around X axis by angle (radians).
    pub fn rotation_x(angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Transform {
            m: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, c, -s, 0.0],
                [0.0, s, c, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create rotation around Y axis by angle (radians).
    pub fn rotation_y(angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Transform {
            m: [
                [c, 0.0, s, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [-s, 0.0, c, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create rotation around Z axis by angle (radians).
    pub fn rotation_z(angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Transform {
            m: [
                [c, -s, 0.0, 0.0],
                [s, c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create rotation around an arbitrary axis.
    pub fn rotation_axis(axis: &Direction3d, angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        let t = 1.0 - c;
        let x = axis.x;
        let y = axis.y;
        let z = axis.z;
        Transform {
            m: [
                [t * x * x + c,     t * x * y - s * z, t * x * z + s * y, 0.0],
                [t * x * y + s * z, t * y * y + c,     t * y * z - s * x, 0.0],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c,     0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Compose two transforms: self * other.
    pub fn multiply(&self, other: &Transform) -> Transform {
        let mut result = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    result[i][j] += self.m[i][k] * other.m[k][j];
                }
            }
        }
        Transform { m: result }
    }

    /// Transform a point.
    pub fn transform_point(&self, p: &Point3d) -> Point3d {
        p.transform(&self.m)
    }

    /// Transform a direction (no translation).
    pub fn transform_direction(&self, d: &Direction3d) -> Direction3d {
        let x = self.m[0][0] * d.x + self.m[0][1] * d.y + self.m[0][2] * d.z;
        let y = self.m[1][0] * d.x + self.m[1][1] * d.y + self.m[1][2] * d.z;
        let z = self.m[2][0] * d.x + self.m[2][1] * d.y + self.m[2][2] * d.z;
        Direction3d::new(x, y, z).unwrap_or(*d)
    }

    /// Compute the inverse transform.
    pub fn inverse(&self) -> Option<Transform> {
        // Full 4x4 inverse using cofactor method
        let m = &self.m;
        let mut inv = [[0.0; 4]; 4];

        inv[0][0] = m[1][1]*m[2][2]*m[3][3] - m[1][1]*m[2][3]*m[3][2] - m[1][2]*m[2][1]*m[3][3]
            + m[1][2]*m[2][3]*m[3][1] + m[1][3]*m[2][1]*m[3][2] - m[1][3]*m[2][2]*m[3][1];
        inv[0][1] = -m[0][1]*m[2][2]*m[3][3] + m[0][1]*m[2][3]*m[3][2] + m[0][2]*m[2][1]*m[3][3]
            - m[0][2]*m[2][3]*m[3][1] - m[0][3]*m[2][1]*m[3][2] + m[0][3]*m[2][2]*m[3][1];
        inv[0][2] = m[0][1]*m[1][2]*m[3][3] - m[0][1]*m[1][3]*m[3][2] - m[0][2]*m[1][1]*m[3][3]
            + m[0][2]*m[1][3]*m[3][1] + m[0][3]*m[1][1]*m[3][2] - m[0][3]*m[1][2]*m[3][1];
        inv[0][3] = -m[0][1]*m[1][2]*m[2][3] + m[0][1]*m[1][3]*m[2][2] + m[0][2]*m[1][1]*m[2][3]
            - m[0][2]*m[1][3]*m[2][1] - m[0][3]*m[1][1]*m[2][2] + m[0][3]*m[1][2]*m[2][1];

        let det = m[0][0]*inv[0][0] + m[0][1]*inv[0][2+0] + m[0][2]*inv[0][2] + m[0][3]*inv[0][3]; // Fix: use proper indexing
        // Actually let me use a cleaner approach
        let det = m[0][0] * inv[0][0]
            + m[0][1] * (-(m[1][0]*m[2][2]*m[3][3] - m[1][0]*m[2][3]*m[3][2] - m[1][2]*m[2][0]*m[3][3]
                + m[1][2]*m[2][3]*m[3][0] + m[1][3]*m[2][0]*m[3][2] - m[1][3]*m[2][2]*m[3][0]))
            + m[0][2] * (m[1][0]*m[2][1]*m[3][3] - m[1][0]*m[2][3]*m[3][1] - m[1][1]*m[2][0]*m[3][3]
                + m[1][1]*m[2][3]*m[3][0] + m[1][3]*m[2][0]*m[3][1] - m[1][3]*m[2][1]*m[3][0])
            + m[0][3] * (-(m[1][0]*m[2][1]*m[3][2] - m[1][0]*m[2][2]*m[3][1] - m[1][1]*m[2][0]*m[3][2]
                + m[1][1]*m[2][2]*m[3][0] + m[1][2]*m[2][0]*m[3][1] - m[1][2]*m[2][1]*m[3][0]));

        if det.abs() < 1e-15 {
            return None;
        }

        let inv_det = 1.0 / det;

        inv[1][0] = -(m[1][0]*m[2][2]*m[3][3] - m[1][0]*m[2][3]*m[3][2] - m[1][2]*m[2][0]*m[3][3]
            + m[1][2]*m[2][3]*m[3][0] + m[1][3]*m[2][0]*m[3][2] - m[1][3]*m[2][2]*m[3][0]);
        inv[1][1] = m[0][0]*m[2][2]*m[3][3] - m[0][0]*m[2][3]*m[3][2] - m[0][2]*m[2][0]*m[3][3]
            + m[0][2]*m[2][3]*m[3][0] + m[0][3]*m[2][0]*m[3][2] - m[0][3]*m[2][2]*m[3][0];
        inv[1][2] = -(m[0][0]*m[1][2]*m[3][3] - m[0][0]*m[1][3]*m[3][2] - m[0][2]*m[1][0]*m[3][3]
            + m[0][2]*m[1][3]*m[3][0] + m[0][3]*m[1][0]*m[3][2] - m[0][3]*m[1][2]*m[3][0]);
        inv[1][3] = m[0][0]*m[1][2]*m[2][3] - m[0][0]*m[1][3]*m[2][2] - m[0][2]*m[1][0]*m[2][3]
            + m[0][2]*m[1][3]*m[2][0] + m[0][3]*m[1][0]*m[2][2] - m[0][3]*m[1][2]*m[2][0];

        inv[2][0] = m[1][0]*m[2][1]*m[3][3] - m[1][0]*m[2][3]*m[3][1] - m[1][1]*m[2][0]*m[3][3]
            + m[1][1]*m[2][3]*m[3][0] + m[1][3]*m[2][0]*m[3][1] - m[1][3]*m[2][1]*m[3][0];
        inv[2][1] = -(m[0][0]*m[2][1]*m[3][3] - m[0][0]*m[2][3]*m[3][1] - m[0][1]*m[2][0]*m[3][3]
            + m[0][1]*m[2][3]*m[3][0] + m[0][3]*m[2][0]*m[3][1] - m[0][3]*m[2][1]*m[3][0]);
        inv[2][2] = m[0][0]*m[1][1]*m[3][3] - m[0][0]*m[1][3]*m[3][1] - m[0][1]*m[1][0]*m[3][3]
            + m[0][1]*m[1][3]*m[3][0] + m[0][3]*m[1][0]*m[3][1] - m[0][3]*m[1][1]*m[3][0];
        inv[2][3] = -(m[0][0]*m[1][1]*m[2][3] - m[0][0]*m[1][3]*m[2][1] - m[0][1]*m[1][0]*m[2][3]
            + m[0][1]*m[1][3]*m[2][0] + m[0][3]*m[1][0]*m[2][1] - m[0][3]*m[1][1]*m[2][0]);

        inv[3][0] = -(m[1][0]*m[2][1]*m[3][2] - m[1][0]*m[2][2]*m[3][1] - m[1][1]*m[2][0]*m[3][2]
            + m[1][1]*m[2][2]*m[3][0] + m[1][2]*m[2][0]*m[3][1] - m[1][2]*m[2][1]*m[3][0]);
        inv[3][1] = m[0][0]*m[2][1]*m[3][2] - m[0][0]*m[2][2]*m[3][1] - m[0][1]*m[2][0]*m[3][2]
            + m[0][1]*m[2][2]*m[3][0] + m[0][2]*m[2][0]*m[3][1] - m[0][2]*m[2][1]*m[3][0];
        inv[3][2] = -(m[0][0]*m[1][1]*m[3][2] - m[0][0]*m[1][2]*m[3][1] - m[0][1]*m[1][0]*m[3][2]
            + m[0][1]*m[1][2]*m[3][0] + m[0][2]*m[1][0]*m[3][1] - m[0][2]*m[1][1]*m[3][0]);
        inv[3][3] = m[0][0]*m[1][1]*m[2][2] - m[0][0]*m[1][2]*m[2][1] - m[0][1]*m[1][0]*m[2][2]
            + m[0][1]*m[1][2]*m[2][0] + m[0][2]*m[1][0]*m[2][1] - m[0][2]*m[1][1]*m[2][0];

        for i in 0..4 {
            for j in 0..4 {
                inv[i][j] *= inv_det;
            }
        }

        Some(Transform { m: inv })
    }
}

impl std::ops::Mul for &Transform {
    type Output = Transform;

    fn mul(self, other: &Transform) -> Transform {
        self.multiply(other)
    }
}
