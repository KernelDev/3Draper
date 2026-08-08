// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Rotation utilities for the assembly solver.
//!
//! Uses rotation vector (axis × angle) representation: a 3-vector
//! whose direction is the rotation axis and magnitude is the angle.
//! Converts to/from rotation matrix via Rodrigues formula.

use draper_geometry::Direction3d;

/// A 3×3 matrix stored as `[[row0], [row1], [row2]]`.
pub type Mat3 = [[f64; 3]; 3];

/// A rotation vector (axis × angle).
#[derive(Clone, Copy, Debug)]
pub struct RotationVec {
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
}

impl RotationVec {
    pub fn new(rx: f64, ry: f64, rz: f64) -> Self {
        Self { rx, ry, rz }
    }

    /// Convert to a 3×3 rotation matrix via Rodrigues formula.
    pub fn to_matrix(&self) -> Mat3 {
        let theta = (self.rx * self.rx + self.ry * self.ry + self.rz * self.rz).sqrt();
        if theta < 1e-15 {
            return [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ];
        }
        let ux = self.rx / theta;
        let uy = self.ry / theta;
        let uz = self.rz / theta;
        let s = theta.sin();
        let c = theta.cos();
        let t = 1.0 - c;

        let uut = [
            [ux * ux, ux * uy, ux * uz],
            [uy * ux, uy * uy, uy * uz],
            [uz * ux, uz * uy, uz * uz],
        ];
        let skew = [
            [0.0, -uz, uy],
            [uz, 0.0, -ux],
            [-uy, ux, 0.0],
        ];

        let mut r = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                r[i][j] = c * (if i == j { 1.0 } else { 0.0 }) + s * skew[i][j] + t * uut[i][j];
            }
        }
        r
    }
}

/// Convert a 4×4 transform matrix to a rotation vector.
pub fn rotation_matrix_to_vec(m: &[[f64; 4]; 4]) -> (f64, f64, f64) {
    let r = [
        [m[0][0], m[0][1], m[0][2]],
        [m[1][0], m[1][1], m[1][2]],
        [m[2][0], m[2][1], m[2][2]],
    ];
    rotation_mat3_to_vec(&r)
}

/// Convert a 3×3 rotation matrix to a rotation vector.
pub fn rotation_mat3_to_vec(r: &Mat3) -> (f64, f64, f64) {
    let trace = r[0][0] + r[1][1] + r[2][2];
    let cos_theta = (trace - 1.0) / 2.0;
    let cos_theta = cos_theta.clamp(-1.0, 1.0);
    let theta = cos_theta.acos();

    if theta.abs() < 1e-10 {
        let rx = (r[2][1] - r[1][2]) / 2.0;
        let ry = (r[0][2] - r[2][0]) / 2.0;
        let rz = (r[1][0] - r[0][1]) / 2.0;
        return (rx, ry, rz);
    }

    if (theta - std::f64::consts::PI).abs() < 1e-6 {
        let m00 = r[0][0] + 1.0;
        let m11 = r[1][1] + 1.0;
        let m22 = r[2][2] + 1.0;
        let (ux, uy, uz);
        if m00 >= m11 && m00 >= m22 {
            ux = m00.sqrt();
            uy = r[0][1] / ux;
            uz = r[0][2] / ux;
        } else if m11 >= m00 && m11 >= m22 {
            uy = m11.sqrt();
            ux = r[0][1] / uy;
            uz = r[1][2] / uy;
        } else {
            uz = m22.sqrt();
            ux = r[0][2] / uz;
            uy = r[1][2] / uz;
        }
        let len = (ux * ux + uy * uy + uz * uz).sqrt().max(1e-15);
        return (ux / len * theta, uy / len * theta, uz / len * theta);
    }

    let sin_theta = theta.sin();
    let ux = (r[2][1] - r[1][2]) / (2.0 * sin_theta);
    let uy = (r[0][2] - r[2][0]) / (2.0 * sin_theta);
    let uz = (r[1][0] - r[0][1]) / (2.0 * sin_theta);
    (ux * theta, uy * theta, uz * theta)
}

/// Compute the skew-symmetric matrix `[v]×` of a 3-vector.
pub fn skew_symmetric_vec(v: &Direction3d) -> Mat3 {
    [
        [0.0, -v.z, v.y],
        [v.z, 0.0, -v.x],
        [-v.y, v.x, 0.0],
    ]
}

/// Multiply two 3×3 matrices.
pub fn mat_mul_mat(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

/// Negate a 3×3 matrix.
pub fn neg_mat(a: &Mat3) -> Mat3 {
    [
        [-a[0][0], -a[0][1], -a[0][2]],
        [-a[1][0], -a[1][1], -a[1][2]],
        [-a[2][0], -a[2][1], -a[2][2]],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_identity_rotation_vec() {
        let r = RotationVec::new(0.0, 0.0, 0.0);
        let m = r.to_matrix();
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(m[i][j], if i == j { 1.0 } else { 0.0 }, epsilon = 1e-15);
            }
        }
    }

    #[test]
    fn test_rotation_z_90() {
        let r = RotationVec::new(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let m = r.to_matrix();
        assert_relative_eq!(m[0][0], 0.0, epsilon = 1e-15);
        assert_relative_eq!(m[0][1], -1.0, epsilon = 1e-15);
        assert_relative_eq!(m[1][0], 1.0, epsilon = 1e-15);
        assert_relative_eq!(m[1][1], 0.0, epsilon = 1e-15);
    }

    #[test]
    fn test_round_trip_general() {
        let (rx, ry, rz) = (0.3, -0.7, 1.2);
        let r = RotationVec::new(rx, ry, rz);
        let m = r.to_matrix();
        let (rx2, ry2, rz2) = rotation_mat3_to_vec(&m);
        let r2 = RotationVec::new(rx2, ry2, rz2);
        let m2 = r2.to_matrix();
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(m[i][j], m2[i][j], epsilon = 1e-10);
            }
        }
    }
}
