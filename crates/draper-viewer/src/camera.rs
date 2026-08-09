// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
#![allow(dead_code)]
//! Orbit camera with perspective projection — quaternion-based rotation.
//!
//! Uses a unit quaternion to store the camera orientation so that
//! rotation is free of gimbal lock.  The camera always orbits around
//! a `target` point at a configurable `distance`.

// ─── Minimal inline quaternion helpers ───────────────────────────────────
// We avoid pulling in a full math crate just for quaternion ops.

/// Quaternion represented as [w, x, y, z].
pub type Quat = [f32; 4];

#[inline]
fn quat_identity() -> Quat {
    [1.0, 0.0, 0.0, 0.0]
}

/// Create a quaternion from an axis (must be normalized) and angle in radians.
#[inline]
fn quat_from_axis_angle(axis: [f32; 3], angle: f32) -> Quat {
    let half = angle * 0.5;
    let s = half.sin();
    [half.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
}

/// Hamilton product q1 * q2.
#[inline]
fn quat_mul(q1: &Quat, q2: &Quat) -> Quat {
    let (w1, x1, y1, z1) = (q1[0], q1[1], q1[2], q1[3]);
    let (w2, x2, y2, z2) = (q2[0], q2[1], q2[2], q2[3]);
    [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]
}

/// Conjugate (for unit quats same as inverse).
#[inline]
fn quat_conj(q: &Quat) -> Quat {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Normalize a quaternion in-place, return normalized.
#[inline]
fn quat_normalize(q: &Quat) -> Quat {
    let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if len < 1e-12 {
        return quat_identity();
    }
    let inv = 1.0 / len;
    [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv]
}

/// Rotate a 3D vector by a unit quaternion: q * v * q^-1.
#[inline]
fn quat_rotate_vec(q: &Quat, v: [f32; 3]) -> [f32; 3] {
    // Pure quaternion p = [0, v]
    let p: Quat = [0.0, v[0], v[1], v[2]];
    let qc = quat_conj(q);
    let r = quat_mul(q, &quat_mul(&p, &qc));
    [r[1], r[2], r[3]]
}

/// Extract the "right" (local +X), "up" (local +Y), and "forward" (local -Z)
/// basis vectors from the quaternion.
#[inline]
fn quat_basis(q: &Quat) -> ([f32; 3], [f32; 3], [f32; 3]) {
    // In our convention the camera looks along local -Z.
    // Right = q * [1,0,0] * q^-1
    // Up    = q * [0,1,0] * q^-1
    // Fwd   = q * [0,0,-1] * q^-1  (the direction the camera looks at)
    let right = quat_rotate_vec(q, [1.0, 0.0, 0.0]);
    let up = quat_rotate_vec(q, [0.0, 1.0, 0.0]);
    let fwd = quat_rotate_vec(q, [0.0, 0.0, -1.0]);
    (right, up, fwd)
}

/// Cross product.
#[inline]
fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Dot product.
#[inline]
fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Normalize a 3D vector.
#[inline]
fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        return [0.0, 0.0, 0.0];
    }
    let inv = 1.0 / len;
    [v[0] * inv, v[1] * inv, v[2] * inv]
}

// ─── OrbitCamera ─────────────────────────────────────────────────────────

/// Orbit camera that rotates freely around a target point using a quaternion.
///
/// Unlike Euler-angle based cameras this has **no gimbal lock** — you can
/// spin freely in any direction and the rotation is always smooth.
#[derive(Clone, Debug)]
pub struct OrbitCamera {
    /// Target point the camera orbits around (model center).
    pub target: [f32; 3],
    /// Quaternion encoding the camera orientation.
    /// Default orientation: looking along world -Z (forward), local +Y is up.
    pub orientation: Quat,
    /// Distance from target to camera.
    pub distance: f32,
    /// Field of view in degrees.
    pub fov: f32,
    /// Near plane distance.
    pub near: f32,
    /// Far plane distance.
    pub far: f32,
    /// Max dimension of the current model's bounding box.
    /// Used to scale zoom limits so small models can be zoomed in
    /// and large models can be zoomed out.
    model_size: f32,
    /// Projection mode: true = perspective (default), false = orthographic.
    /// Phase 3.6: toggled via ViewPerspective / ViewOrthographic menu actions.
    pub perspective: bool,
    /// Orthographic half-height (world units). Used only when `perspective` is false.
    pub ortho_half_height: f32,
}

impl OrbitCamera {
    pub fn new() -> Self {
        // Start with an isometric-ish view: azimuth -45°, elevation 30°
        let azimuth = -45.0_f32.to_radians();
        let elevation = 30.0_f32.to_radians();
        let q_azimuth = quat_from_axis_angle([0.0, 1.0, 0.0], azimuth);
        let q_elevation = quat_from_axis_angle([1.0, 0.0, 0.0], -elevation);
        let orientation = quat_normalize(&quat_mul(&q_azimuth, &q_elevation));

        Self {
            target: [0.0, 0.0, 0.0],
            orientation,
            distance: 500.0,
            fov: 45.0,
            near: 0.1,
            far: 100000.0,
            model_size: 100.0,
            perspective: true,
            ortho_half_height: 100.0,
        }
    }

    /// Auto-fit camera to show the entire bounding box.
    pub fn fit_to_bounding_box(&mut self, bbox_min: [f32; 3], bbox_max: [f32; 3]) {
        let center = [
            (bbox_min[0] + bbox_max[0]) * 0.5,
            (bbox_min[1] + bbox_max[1]) * 0.5,
            (bbox_min[2] + bbox_max[2]) * 0.5,
        ];
        let size = [
            bbox_max[0] - bbox_min[0],
            bbox_max[1] - bbox_min[1],
            bbox_max[2] - bbox_min[2],
        ];
        let max_dim = size[0].max(size[1]).max(size[2]).max(0.001);

        self.model_size = max_dim;
        self.target = center;
        let fov_rad = self.fov.to_radians();
        // Use 2.0x margin (was 1.5x) to ensure the entire model is visible
        // even with perspective distortion at the edges of the viewport.
        self.distance = max_dim / (2.0 * (fov_rad * 0.5).tan()) * 2.0;
        self.distance = self.distance.max(max_dim * 0.5);
        // Adjust near/far planes to model size
        self.near = (max_dim * 0.001).max(0.0001);
        self.far = (max_dim * 1000.0).max(100000.0);
    }

    /// Reset orientation to the default isometric view (azimuth -45°, elevation 30°)
    /// while keeping the current target and distance.
    ///
    /// This is used after loading a new model so the user always sees it
    /// from a recognizable 3/4 perspective angle, regardless of any previous
    /// rotation they may have applied.
    pub fn reset_orientation_to_isometric(&mut self) {
        let azimuth = -45.0_f32.to_radians();
        let elevation = 30.0_f32.to_radians();
        let q_azimuth = quat_from_axis_angle([0.0, 1.0, 0.0], azimuth);
        let q_elevation = quat_from_axis_angle([1.0, 0.0, 0.0], -elevation);
        self.orientation = quat_normalize(&quat_mul(&q_azimuth, &q_elevation));
    }

    /// Convenience: fit to bbox AND reset orientation in one call.
    /// Useful after loading a new model so the user sees it properly framed.
    pub fn fit_and_reset_orientation(&mut self, bbox_min: [f32; 3], bbox_max: [f32; 3]) {
        self.fit_to_bounding_box(bbox_min, bbox_max);
        self.reset_orientation_to_isometric();
    }

    /// Get camera position in world space.
    pub fn position(&self) -> [f32; 3] {
        let (_right, _up, fwd) = quat_basis(&self.orientation);
        // Camera is positioned at target + distance * opposite_of_forward
        // forward is the direction camera looks, so camera pos = target - distance * fwd
        [
            self.target[0] - self.distance * fwd[0],
            self.target[1] - self.distance * fwd[1],
            self.target[2] - self.distance * fwd[2],
        ]
    }

    /// Get the forward direction (from camera toward target), normalized.
    pub fn forward(&self) -> [f32; 3] {
        let (_right, _up, fwd) = quat_basis(&self.orientation);
        fwd
    }

    /// Get the right direction (camera local +X), normalized.
    pub fn right(&self) -> [f32; 3] {
        let (right, _up, _fwd) = quat_basis(&self.orientation);
        right
    }

    /// Get the up direction (camera local +Y), normalized.
    pub fn up(&self) -> [f32; 3] {
        let (_right, up, _fwd) = quat_basis(&self.orientation);
        up
    }

    /// Compute view matrix (column-major 4x4).
    pub fn view_matrix(&self) -> [[f32; 4]; 4] {
        let pos = self.position();
        let (right, up, fwd) = quat_basis(&self.orientation);

        // View matrix = inverse of camera transform
        let tx = -(right[0] * pos[0] + right[1] * pos[1] + right[2] * pos[2]);
        let ty = -(up[0] * pos[0] + up[1] * pos[1] + up[2] * pos[2]);
        let tz = fwd[0] * pos[0] + fwd[1] * pos[1] + fwd[2] * pos[2];

        [
            [right[0], up[0], -fwd[0], 0.0],
            [right[1], up[1], -fwd[1], 0.0],
            [right[2], up[2], -fwd[2], 0.0],
            [tx, ty, tz, 1.0],
        ]
    }

    /// Compute projection matrix (column-major 4x4).
    ///
    /// Phase 3.6: dispatches to perspective or orthographic based on
    /// the `perspective` field. Uses wgpu/Vulkan Z convention [0, 1].
    pub fn projection_matrix(&self, aspect: f32) -> [[f32; 4]; 4] {
        if self.perspective {
            self.perspective_matrix(aspect)
        } else {
            self.orthographic_matrix(aspect)
        }
    }

    /// Perspective projection matrix.
    fn perspective_matrix(&self, aspect: f32) -> [[f32; 4]; 4] {
        let fov_rad = self.fov.to_radians();
        let f = 1.0 / (fov_rad * 0.5).tan();
        let z_range = self.near - self.far;

        [
            [f / aspect, 0.0, 0.0, 0.0],
            [0.0, f, 0.0, 0.0],
            [0.0, 0.0, self.far / z_range, -1.0],
            [0.0, 0.0, self.near * self.far / z_range, 0.0],
        ]
    }

    /// Orthographic projection matrix.
    /// Width derived from `ortho_half_height * aspect`.
    fn orthographic_matrix(&self, aspect: f32) -> [[f32; 4]; 4] {
        let t = self.ortho_half_height;
        let b = -self.ortho_half_height;
        let r = self.ortho_half_height * aspect;
        let l = -r;
        let n = self.near;
        let f = self.far;
        [
            [2.0 / (r - l), 0.0, 0.0, 0.0],
            [0.0, 2.0 / (t - b), 0.0, 0.0],
            [0.0, 0.0, 1.0 / (f - n), 0.0],
            [-(r + l) / (r - l), -(t + b) / (t - b), -n / (f - n), 1.0],
        ]
    }

    /// Toggle between perspective and orthographic.
    /// When switching to ortho, sync `ortho_half_height` with current distance/fov.
    pub fn set_perspective(&mut self, perspective: bool) {
        if perspective == self.perspective { return; }
        if !perspective {
            let fov_rad = self.fov.to_radians();
            self.ortho_half_height = (fov_rad * 0.5).tan() * self.distance;
        }
        self.perspective = perspective;
    }

    /// Rotate the camera by the given screen-space deltas (orbit around target).
    ///
    /// `delta_x` — horizontal drag (pixels): rotates around the camera's **up** axis.
    /// `delta_y` — vertical drag (pixels): rotates around the camera's **right** axis.
    ///
    /// No gimbal lock — quaternion composition is always smooth.
    pub fn rotate(&mut self, delta_x: f32, delta_y: f32) {
        let sensitivity = 0.01;

        // Yaw: rotate around the camera's local up axis
        let yaw_angle = -delta_x * sensitivity;
        let (_, up, _) = quat_basis(&self.orientation);
        let q_yaw = quat_from_axis_angle(up, yaw_angle);

        // Pitch: rotate around the camera's local right axis
        let pitch_angle = -delta_y * sensitivity;
        let (right, _, _) = quat_basis(&self.orientation);
        let q_pitch = quat_from_axis_angle(right, pitch_angle);

        // Apply: first pitch, then yaw (order matters but both are small increments)
        self.orientation = quat_normalize(&quat_mul(&q_yaw, &quat_mul(&q_pitch, &self.orientation)));
    }

    /// Rotate around the world Y axis only (used for view presets).
    pub fn rotate_world_yaw(&mut self, angle_rad: f32) {
        let q = quat_from_axis_angle([0.0, 1.0, 0.0], angle_rad);
        self.orientation = quat_normalize(&quat_mul(&q, &self.orientation));
    }

    /// Rotate around the camera's local right axis only (used for view presets).
    pub fn rotate_local_pitch(&mut self, angle_rad: f32) {
        let (right, _, _) = quat_basis(&self.orientation);
        let q = quat_from_axis_angle(right, angle_rad);
        self.orientation = quat_normalize(&quat_mul(&q, &self.orientation));
    }

    /// Set orientation to look from a given direction toward the target.
    /// `direction` is the direction FROM the camera TO the target (does not need to be normalized).
    pub fn look_from_direction(&mut self, direction: [f32; 3]) {
        let fwd = normalize(direction);
        // Build a quaternion that rotates [0,0,-1] to fwd
        // Using the "from-to" rotation approach.
        let default_fwd: [f32; 3] = [0.0, 0.0, -1.0];
        let default_up: [f32; 3] = [0.0, 1.0, 0.0];
        // Check if fwd is nearly parallel or antiparallel to default_fwd
        let d = dot(default_fwd, fwd);
        if d > 0.9999 {
            // Already looking along -Z, identity rotation
            self.orientation = quat_identity();
            return;
        }
        if d < -0.9999 {
            // Looking along +Z (opposite of default), flip 180° around Y
            self.orientation = quat_from_axis_angle([0.0, 1.0, 0.0], std::f32::consts::PI);
            return;
        }

        // General case: rotation axis = default_fwd × fwd
        let axis = normalize(cross(default_fwd, fwd));
        let angle = d.acos();
        let q_forward = quat_from_axis_angle(axis, angle);

        // Now apply the up-vector correction.
        // The camera's local up after q_forward might not be world up.
        let rotated_up = quat_rotate_vec(&q_forward, default_up);
        let world_up: [f32; 3] = [0.0, 1.0, 0.0];

        // Project world_up onto the camera's right axis plane
        let right = normalize(cross(fwd, world_up));
        let projected_up = normalize(cross(right, fwd));

        // If the projected up is nearly zero, the camera is looking along world up/down
        let up_dot = dot(rotated_up, projected_up);
        if up_dot.abs() < 0.9999 {
            // Need to roll the camera to align up with projected world up
            let roll_axis = fwd; // roll around the forward axis
            let roll_angle = dot(rotated_up, projected_up).acos()
                * if dot(cross(rotated_up, projected_up), fwd) >= 0.0 { 1.0 } else { -1.0 };
            let q_roll = quat_from_axis_angle(roll_axis, roll_angle);
            self.orientation = quat_normalize(&quat_mul(&q_roll, &q_forward));
        } else {
            self.orientation = q_forward;
        }
    }

    /// Set the camera orientation directly from a quaternion.
    /// Used by the ViewCube for smooth slerp animation: the ViewCube computes
    /// a target quaternion and the app interpolates toward it.
    pub fn set_orientation(&mut self, q: Quat) {
        self.orientation = quat_normalize(&q);
    }

    /// Spherical linear interpolation between two quaternions.
    /// `t` is in [0, 1] where 0 returns `from` and 1 returns `to`.
    /// Uses the shortest-arc path (negates `to` if dot < 0).
    /// Quat layout: [w, x, y, z].
    pub fn slerp_quat(from: &Quat, to: &Quat, t: f32) -> Quat {
        let mut cos_half_theta = from[0]*to[0] + from[1]*to[1] + from[2]*to[2] + from[3]*to[3];
        let mut to2 = *to;
        // If dot < 0, negate `to` to take the shorter arc
        if cos_half_theta < 0.0 {
            to2 = [-to[0], -to[1], -to[2], -to[3]];
            cos_half_theta = -cos_half_theta;
        }
        // If quaternions are very close, use linear interpolation to avoid NaN
        if cos_half_theta >= 1.0 {
            return quat_normalize(&[
                from[0] + t * (to2[0] - from[0]),
                from[1] + t * (to2[1] - from[1]),
                from[2] + t * (to2[2] - from[2]),
                from[3] + t * (to2[3] - from[3]),
            ]);
        }
        let half_theta = cos_half_theta.acos();
        let sin_half_theta = (1.0 - cos_half_theta * cos_half_theta).sqrt().max(1e-10);
        let ratio_a = ((1.0 - t) * half_theta).sin() / sin_half_theta;
        let ratio_b = (t * half_theta).sin() / sin_half_theta;
        [
            from[0] * ratio_a + to2[0] * ratio_b,
            from[1] * ratio_a + to2[1] * ratio_b,
            from[2] * ratio_a + to2[2] * ratio_b,
            from[3] * ratio_a + to2[3] * ratio_b,
        ]
    }

    /// Compute the target quaternion for looking from a given direction
    /// (without mutating the camera). Used by the ViewCube to compute the
    /// slerp target before interpolation starts.
    pub fn orientation_for_direction(direction: [f32; 3]) -> Quat {
        let fwd = normalize(direction);
        let default_fwd: [f32; 3] = [0.0, 0.0, -1.0];
        let default_up: [f32; 3] = [0.0, 1.0, 0.0];
        let d = dot(default_fwd, fwd);
        if d > 0.9999 {
            return quat_identity();
        }
        if d < -0.9999 {
            return quat_from_axis_angle([0.0, 1.0, 0.0], std::f32::consts::PI);
        }
        let axis = normalize(cross(default_fwd, fwd));
        let angle = d.acos();
        let q_forward = quat_from_axis_angle(axis, angle);
        let rotated_up = quat_rotate_vec(&q_forward, default_up);
        let world_up: [f32; 3] = [0.0, 1.0, 0.0];
        let right = normalize(cross(fwd, world_up));
        let projected_up = normalize(cross(right, fwd));
        let up_dot = dot(rotated_up, projected_up);
        if up_dot.abs() < 0.9999 {
            let roll_axis = fwd;
            let roll_angle = dot(rotated_up, projected_up).acos()
                * if dot(cross(rotated_up, projected_up), fwd) >= 0.0 { 1.0 } else { -1.0 };
            let q_roll = quat_from_axis_angle(roll_axis, roll_angle);
            quat_normalize(&quat_mul(&q_roll, &q_forward))
        } else {
            q_forward
        }
    }

    /// Zoom the camera by the given delta.
    /// When `mouse_norm` is Some([nx, ny]), zoom toward the point under the cursor
    /// in normalized device coordinates (-1 to 1). When None, zoom toward target center.
    pub fn zoom(&mut self, delta: f32, mouse_norm: Option<[f32; 2]>) {
        let factor = 1.0 - delta * 0.001;
        // Min distance scales with model size — allows zooming into very
        // small models (e.g., brick_thin.stp with 0.5mm dimensions).
        // Max distance allows zooming out from very large models.
        let min_dist = (self.model_size * 0.01).max(0.001);
        let max_dist = (self.model_size * 100.0).max(100000.0);
        let new_distance = (self.distance * factor).max(min_dist).min(max_dist);
        let zoom_ratio = new_distance / self.distance;

        if let Some([nx, ny]) = mouse_norm {
            let (right, up, _fwd) = quat_basis(&self.orientation);

            let fov_rad = self.fov.to_radians();
            let half_height = self.distance * (fov_rad * 0.5).tan();

            let offset_x = nx * half_height;
            let offset_y = ny * half_height;

            let cursor_world = [
                self.target[0] + right[0] * offset_x + up[0] * offset_y,
                self.target[1] + right[1] * offset_x + up[1] * offset_y,
                self.target[2] + right[2] * offset_x + up[2] * offset_y,
            ];

            let blend = 1.0 - zoom_ratio;
            self.target[0] += (cursor_world[0] - self.target[0]) * blend;
            self.target[1] += (cursor_world[1] - self.target[1]) * blend;
            self.target[2] += (cursor_world[2] - self.target[2]) * blend;
        }

        self.distance = new_distance;
    }

    /// Pan the camera by the given screen-space deltas.
    pub fn pan(&mut self, delta_x: f32, delta_y: f32, _viewport_width: f32, _viewport_height: f32) {
        let pan_speed = self.distance * 0.002;
        let dx = -delta_x * pan_speed;
        let dy = delta_y * pan_speed;

        let (right, up, _) = quat_basis(&self.orientation);

        self.target[0] += right[0] * dx + up[0] * dy;
        self.target[1] += right[1] * dx + up[1] * dy;
        self.target[2] += right[2] * dx + up[2] * dy;
    }

    /// Compute a world-space ray (origin, direction) for a given screen position.
    ///
    /// `screen_pos` — pixel coordinates within the viewport (origin at top-left).
    /// `viewport` — (x, y, width, height) of the viewport rect in pixels.
    ///
    /// Returns `(ray_origin, ray_direction)` where direction is normalized.
    pub fn screen_to_ray(&self, screen_pos: [f32; 2], viewport: (f32, f32, f32, f32)) -> ([f32; 3], [f32; 3]) {
        let (_vx, _vy, vw, vh) = viewport;

        // Normalized device coordinates [-1, 1]
        let ndc_x = 2.0 * screen_pos[0] / vw - 1.0;
        let ndc_y = 1.0 - 2.0 * screen_pos[1] / vh; // flip Y

        let aspect = vw / vh;
        let fov_rad = self.fov.to_radians();
        let half_h = (fov_rad * 0.5).tan();

        // Ray direction in camera space
        let dir_cam = normalize([ndc_x * half_h * aspect, ndc_y * half_h, -1.0]);

        // Transform to world space using camera orientation
        // View matrix 3x3 is [right, up, -fwd] (columns).
        // Inverse rotation (camera-to-world) is the transpose:
        //   Row 0 = right, Row 1 = up, Row 2 = -fwd
        // So: world = right*cx + up*cy + (-fwd)*cz
        // For cz = -1.0 (camera looks along -Z = +fwd in world):
        //   world = right*nx + up*ny + (-fwd)*(-1) = right*nx + up*ny + fwd
        let (right, up, fwd) = quat_basis(&self.orientation);

        let dir_world = normalize([
            right[0] * dir_cam[0] + up[0] * dir_cam[1] + (-fwd[0]) * dir_cam[2],
            right[1] * dir_cam[0] + up[1] * dir_cam[1] + (-fwd[1]) * dir_cam[2],
            right[2] * dir_cam[0] + up[2] * dir_cam[1] + (-fwd[2]) * dir_cam[2],
        ]);

        let origin = self.position();
        (origin, dir_world)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perspective_default() {
        let cam = OrbitCamera::new();
        assert!(cam.perspective);
    }

    #[test]
    fn test_switch_to_orthographic() {
        let mut cam = OrbitCamera::new();
        cam.distance = 200.0;
        cam.fov = 45.0;
        cam.set_perspective(false);
        assert!(!cam.perspective);
        let expected = (45.0_f32.to_radians() * 0.5).tan() * 200.0;
        assert!((cam.ortho_half_height - expected).abs() < 0.01);
    }

    #[test]
    fn test_switch_back_to_perspective() {
        let mut cam = OrbitCamera::new();
        cam.set_perspective(false);
        cam.set_perspective(true);
        assert!(cam.perspective);
    }

    #[test]
    fn test_perspective_matrix_aspect() {
        let cam = OrbitCamera::new();
        let m1 = cam.projection_matrix(1.0);
        let m2 = cam.projection_matrix(2.0);
        assert!((m1[0][0] / m2[0][0] - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_orthographic_matrix() {
        let mut cam = OrbitCamera::new();
        cam.set_perspective(false);
        let m = cam.projection_matrix(1.0);
        assert!((m[3][3] - 1.0).abs() < 0.01, "Ortho [3][3] should be 1.0");
        assert!(m[2][3].abs() < 0.01, "Ortho [2][3] should be 0 (no perspective divide)");
    }

    #[test]
    fn test_set_perspective_idempotent() {
        let mut cam = OrbitCamera::new();
        let original_ortho = cam.ortho_half_height;
        cam.set_perspective(true);
        assert_eq!(cam.ortho_half_height, original_ortho);
    }
}
