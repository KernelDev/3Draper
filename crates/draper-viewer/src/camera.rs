//! Orbit camera with perspective projection — quaternion-based rotation.
//!
//! Uses a unit quaternion to store the camera orientation so that
//! rotation is free of gimbal lock.  The camera always orbits around
//! a `target` point at a configurable `distance`.

// ─── Minimal inline quaternion helpers ───────────────────────────────────
// We avoid pulling in a full math crate just for quaternion ops.

/// Quaternion represented as [w, x, y, z].
type Quat = [f32; 4];

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
    orientation: Quat,
    /// Distance from target to camera.
    pub distance: f32,
    /// Field of view in degrees.
    pub fov: f32,
    /// Near plane distance.
    pub near: f32,
    /// Far plane distance.
    pub far: f32,
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
        let max_dim = size[0].max(size[1]).max(size[2]).max(1.0);

        self.target = center;
        let fov_rad = self.fov.to_radians();
        self.distance = max_dim / (2.0 * (fov_rad * 0.5).tan()) * 1.5;
        self.distance = self.distance.max(max_dim * 0.5);
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

    /// Compute perspective projection matrix (column-major 4x4).
    ///
    /// Uses wgpu/Vulkan Z range convention [0, 1] (NOT OpenGL [-1, 1]).
    pub fn projection_matrix(&self, aspect: f32) -> [[f32; 4]; 4] {
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

    /// Zoom the camera by the given delta.
    /// When `mouse_norm` is Some([nx, ny]), zoom toward the point under the cursor
    /// in normalized device coordinates (-1 to 1). When None, zoom toward target center.
    pub fn zoom(&mut self, delta: f32, mouse_norm: Option<[f32; 2]>) {
        let factor = 1.0 - delta * 0.001;
        let new_distance = (self.distance * factor).max(1.0).min(100000.0);
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
}
