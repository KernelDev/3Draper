//! Orbit camera with perspective projection.

/// Orbit camera that rotates around a target point.
#[derive(Clone, Debug)]
pub struct OrbitCamera {
    /// Target point the camera orbits around (model center).
    pub target: [f32; 3],
    /// Azimuth angle (rotation around Y axis) in radians.
    pub azimuth: f32,
    /// Elevation angle (rotation around X axis) in radians.
    pub elevation: f32,
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
        Self {
            target: [0.0, 0.0, 0.0],
            azimuth: -45.0_f32.to_radians(),
            elevation: 30.0_f32.to_radians(),
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
        // Distance so the model fills ~70% of the viewport
        let fov_rad = self.fov.to_radians();
        self.distance = max_dim / (2.0 * (fov_rad * 0.5).tan()) * 1.5;
        // Ensure minimum distance
        self.distance = self.distance.max(max_dim * 0.5);
    }

    /// Get camera position in world space.
    pub fn position(&self) -> [f32; 3] {
        let cos_elev = self.elevation.cos();
        let sin_elev = self.elevation.sin();
        let cos_azim = self.azimuth.cos();
        let sin_azim = self.azimuth.sin();

        [
            self.target[0] + self.distance * cos_elev * sin_azim,
            self.target[1] + self.distance * sin_elev,
            self.target[2] + self.distance * cos_elev * cos_azim,
        ]
    }

    /// Get the forward direction (from camera toward target), normalized.
    pub fn forward(&self) -> [f32; 3] {
        let pos = self.position();
        let fwd = [
            self.target[0] - pos[0],
            self.target[1] - pos[1],
            self.target[2] - pos[2],
        ];
        let len = (fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]).sqrt();
        if len > 1e-10 {
            [fwd[0] / len, fwd[1] / len, fwd[2] / len]
        } else {
            [0.0, 0.0, -1.0]
        }
    }

    /// Compute view matrix (column-major 4x4).
    pub fn view_matrix(&self) -> [[f32; 4]; 4] {
        let pos = self.position();

        // Forward direction (from camera to target)
        let fwd = self.forward();

        // Right = fwd cross (0,1,0)
        let up_world = [0.0_f32, 1.0, 0.0];
        let right = [
            fwd[1] * up_world[2] - fwd[2] * up_world[1],
            fwd[2] * up_world[0] - fwd[0] * up_world[2],
            fwd[0] * up_world[1] - fwd[1] * up_world[0],
        ];
        let right_len = (right[0] * right[0] + right[1] * right[1] + right[2] * right[2]).sqrt();
        let right = [right[0] / right_len, right[1] / right_len, right[2] / right_len];

        // Up = right x forward
        let up = [
            right[1] * fwd[2] - right[2] * fwd[1],
            right[2] * fwd[0] - right[0] * fwd[2],
            right[0] * fwd[1] - right[1] * fwd[0],
        ];

        // View matrix = inverse of camera transform
        // Translation component
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
        // z_range is negative (near < far), which is correct for the formula
        let z_range = self.near - self.far;

        [
            [f / aspect, 0.0, 0.0, 0.0],
            [0.0, f, 0.0, 0.0],
            [0.0, 0.0, self.far / z_range, -1.0],
            [0.0, 0.0, self.near * self.far / z_range, 0.0],
        ]
    }

    /// Rotate the camera by the given deltas (orbit around target).
    pub fn rotate(&mut self, delta_x: f32, delta_y: f32) {
        self.azimuth += delta_x * 0.01;
        self.elevation += delta_y * 0.01;
        // Clamp elevation to avoid flipping
        self.elevation = self.elevation.clamp(-89.0_f32.to_radians(), 89.0_f32.to_radians());
    }

    /// Zoom the camera by the given delta.
    /// When `mouse_norm` is Some([nx, ny]), zoom toward the point under the cursor
    /// in normalized device coordinates (-1 to 1). When None, zoom toward target center.
    pub fn zoom(&mut self, delta: f32, mouse_norm: Option<[f32; 2]>) {
        // Exponential zoom for consistent feel
        let factor = 1.0 - delta * 0.001;
        let new_distance = (self.distance * factor).max(1.0).min(100000.0);
        let zoom_ratio = new_distance / self.distance;

        if let Some([nx, ny]) = mouse_norm {
            // Zoom toward the mouse cursor position
            // Compute the world-space point under the cursor at the target depth
            let _pos = self.position();
            let fwd = self.forward();

            // Right and up vectors
            let up_world = [0.0_f32, 1.0, 0.0];
            let right = [
                fwd[1] * up_world[2] - fwd[2] * up_world[1],
                fwd[2] * up_world[0] - fwd[0] * up_world[2],
                fwd[0] * up_world[1] - fwd[1] * up_world[0],
            ];
            let right_len = (right[0] * right[0] + right[1] * right[1] + right[2] * right[2]).sqrt();
            let right = [right[0] / right_len, right[1] / right_len, right[2] / right_len];

            let up = [
                right[1] * fwd[2] - right[2] * fwd[1],
                right[2] * fwd[0] - right[0] * fwd[2],
                right[0] * fwd[1] - right[1] * fwd[0],
            ];

            // Scale nx, ny by the viewport half-size at the target distance
            let fov_rad = self.fov.to_radians();
            let half_height = self.distance * (fov_rad * 0.5).tan();
            let aspect = 1.0; // We'll use normalized coords directly

            // The point on the target plane under the mouse cursor (relative to target)
            let offset_x = nx * half_height * aspect;
            let offset_y = ny * half_height;

            // World-space point under cursor at target depth
            let cursor_world = [
                self.target[0] + right[0] * offset_x + up[0] * offset_y,
                self.target[1] + right[1] * offset_x + up[1] * offset_y,
                self.target[2] + right[2] * offset_x + up[2] * offset_y,
            ];

            // Move target toward cursor_world by (1 - zoom_ratio)
            let blend = 1.0 - zoom_ratio;
            self.target[0] += (cursor_world[0] - self.target[0]) * blend;
            self.target[1] += (cursor_world[1] - self.target[1]) * blend;
            self.target[2] += (cursor_world[2] - self.target[2]) * blend;
        }

        self.distance = new_distance;
    }

    /// Pan the camera by the given screen-space deltas.
    pub fn pan(&mut self, delta_x: f32, delta_y: f32, _viewport_width: f32, _viewport_height: f32) {
        // Pan speed proportional to distance
        let pan_speed = self.distance * 0.002;
        let dx = -delta_x * pan_speed;
        let dy = delta_y * pan_speed;

        // Move target in camera's right/up directions
        let fwd = self.forward();

        let up_world = [0.0_f32, 1.0, 0.0];
        let right = [
            fwd[1] * up_world[2] - fwd[2] * up_world[1],
            fwd[2] * up_world[0] - fwd[0] * up_world[2],
            fwd[0] * up_world[1] - fwd[1] * up_world[0],
        ];
        let right_len = (right[0] * right[0] + right[1] * right[1] + right[2] * right[2]).sqrt();
        let right = [right[0] / right_len, right[1] / right_len, right[2] / right_len];

        let up = [
            right[1] * fwd[2] - right[2] * fwd[1],
            right[2] * fwd[0] - right[0] * fwd[2],
            right[0] * fwd[1] - right[1] * fwd[0],
        ];

        self.target[0] += right[0] * dx + up[0] * dy;
        self.target[1] += right[1] * dx + up[1] * dy;
        self.target[2] += right[2] * dx + up[2] * dy;
    }
}
