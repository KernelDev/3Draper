// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # Server-side Rendering
//!
//! Infrastructure for server-side rendering of 3D models. Uses WebSocket-based
//! communication and a headless rendering pipeline. Supports session management
//! for multiple concurrent users with frame throttling and quality adaptation.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::RwLock;
use uuid::Uuid;

/// Camera parameters for rendering.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CameraState {
    /// Eye position in world space [x, y, z].
    pub eye: [f64; 3],
    /// Look-at target [x, y, z].
    pub target: [f64; 3],
    /// Up vector [x, y, z].
    pub up: [f64; 3],
    /// Field of view in degrees.
    pub fov: f64,
    /// Near clip plane distance.
    pub near: f64,
    /// Far clip plane distance.
    pub far: f64,
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            eye: [0.0, 0.0, 100.0],
            target: [0.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            fov: 45.0,
            near: 0.1,
            far: 10000.0,
        }
    }
}

/// Viewport dimensions.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Viewport {
    pub width: u32,
    pub height: u32,
}

impl Default for Viewport {
    fn default() -> Self {
        Self {
            width: 1920,
            height: 1080,
        }
    }
}

/// A render request from a client.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RenderRequest {
    /// Client camera state.
    pub camera: CameraState,
    /// Target viewport dimensions.
    pub viewport: Viewport,
    /// Model ID to render.
    pub model_id: String,
    /// Level of detail (0.0–1.0).
    pub lod: f64,
}

/// A render response sent back to the client.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RenderResponse {
    /// Rendered image data (PNG bytes).
    pub image_data: Vec<u8>,
    /// Time taken to render in milliseconds.
    pub render_time_ms: f64,
}

/// Bandwidth estimation for quality adaptation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BandwidthClass {
    /// Low bandwidth: reduce quality, smaller viewport.
    Low,
    /// Medium bandwidth: moderate quality.
    Medium,
    /// High bandwidth: full quality.
    High,
}

impl BandwidthClass {
    /// Derive bandwidth class from estimated throughput in kbps.
    pub fn from_kbps(kbps: u32) -> Self {
        if kbps < 500 {
            BandwidthClass::Low
        } else if kbps < 2000 {
            BandwidthClass::Medium
        } else {
            BandwidthClass::High
        }
    }

    /// Get the recommended LOD for this bandwidth class.
    pub fn recommended_lod(&self) -> f64 {
        match self {
            BandwidthClass::Low => 0.25,
            BandwidthClass::Medium => 0.6,
            BandwidthClass::High => 1.0,
        }
    }

    /// Get the maximum frame rate for this bandwidth class.
    pub fn max_fps(&self) -> u32 {
        match self {
            BandwidthClass::Low => 10,
            BandwidthClass::Medium => 24,
            BandwidthClass::High => 60,
        }
    }
}

/// A rendering session for a single user.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RenderSession {
    /// Unique session identifier.
    pub session_id: String,
    /// Current camera state.
    pub camera_state: CameraState,
    /// Models loaded in this session.
    pub loaded_models: Vec<String>,
    /// Estimated bandwidth class.
    pub bandwidth: BandwidthClass,
    /// Current quality LOD.
    pub current_lod: f64,
    /// Last render timestamp.
    pub last_render_time: Option<f64>,
}

impl RenderSession {
    /// Create a new session with a generated UUID.
    pub fn new() -> Self {
        Self {
            session_id: Uuid::new_v4().to_string(),
            camera_state: CameraState::default(),
            loaded_models: Vec::new(),
            bandwidth: BandwidthClass::Medium,
            current_lod: 0.6,
            last_render_time: None,
        }
    }

    /// Create a new session with specific bandwidth.
    pub fn with_bandwidth(bandwidth: BandwidthClass) -> Self {
        let mut session = Self::new();
        session.bandwidth = bandwidth;
        session.current_lod = bandwidth.recommended_lod();
        session
    }
}

/// Frame throttling state for a session.
struct ThrottleState {
    /// Minimum interval between renders.
    min_interval: Duration,
    /// Last render instant.
    last_render: Option<Instant>,
}

impl ThrottleState {
    fn new(bandwidth: BandwidthClass) -> Self {
        let max_fps = bandwidth.max_fps().max(1);
        Self {
            min_interval: Duration::from_millis(1000 / max_fps as u64),
            last_render: None,
        }
    }

    /// Check if enough time has elapsed since the last render.
    fn should_render(&self) -> bool {
        match self.last_render {
            Some(last) => last.elapsed() >= self.min_interval,
            None => true,
        }
    }

    /// Record that a render just occurred.
    fn record_render(&mut self) {
        self.last_render = Some(Instant::now());
    }

    /// Update throttle based on new bandwidth class.
    fn update_bandwidth(&mut self, bandwidth: BandwidthClass) {
        let max_fps = bandwidth.max_fps().max(1);
        self.min_interval = Duration::from_millis(1000 / max_fps as u64);
    }
}

/// Server-side rendering infrastructure.
///
/// `RenderServer` manages WebSocket-based communication, headless rendering,
/// session management, frame throttling, and quality adaptation.
pub struct RenderServer {
    /// Active sessions.
    sessions: Arc<RwLock<HashMap<String, RenderSession>>>,
    /// Throttle state per session.
    throttle_states: Arc<RwLock<HashMap<String, ThrottleState>>>,
    /// Loaded models (model_id → mesh data placeholder).
    models: Arc<RwLock<HashMap<String, Vec<u8>>>>,
}

impl RenderServer {
    /// Create a new render server.
    pub fn new() -> Self {
        Self {
            sessions: Arc::new(RwLock::new(HashMap::new())),
            throttle_states: Arc::new(RwLock::new(HashMap::new())),
            models: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Create a new rendering session.
    ///
    /// Returns the session ID.
    pub async fn create_session(&self) -> String {
        self.create_session_with_bandwidth(BandwidthClass::Medium).await
    }

    /// Create a new rendering session with specified bandwidth.
    ///
    /// Returns the session ID.
    pub async fn create_session_with_bandwidth(&self, bandwidth: BandwidthClass) -> String {
        let session = RenderSession::with_bandwidth(bandwidth);
        let session_id = session.session_id.clone();

        let throttle = ThrottleState::new(bandwidth);

        self.sessions.write().await.insert(session_id.clone(), session);
        self.throttle_states.write().await.insert(session_id.clone(), throttle);

        log::info!("RenderServer: created session {}", session_id);
        session_id
    }

    /// Destroy a rendering session.
    pub async fn destroy_session(&self, session_id: &str) {
        self.sessions.write().await.remove(session_id);
        self.throttle_states.write().await.remove(session_id);
        log::info!("RenderServer: destroyed session {}", session_id);
    }

    /// Get a session by ID.
    pub async fn get_session(&self, session_id: &str) -> Option<RenderSession> {
        self.sessions.read().await.get(session_id).cloned()
    }

    /// Update the camera state for a session.
    pub async fn update_camera(&self, session_id: &str, camera: CameraState) {
        if let Some(session) = self.sessions.write().await.get_mut(session_id) {
            session.camera_state = camera;
        }
    }

    /// Add a model to a session.
    pub async fn add_model_to_session(&self, session_id: &str, model_id: &str) {
        if let Some(session) = self.sessions.write().await.get_mut(session_id) {
            if !session.loaded_models.contains(&model_id.to_string()) {
                session.loaded_models.push(model_id.to_string());
            }
        }
    }

    /// Load a model into the server (stored as raw bytes for the headless renderer).
    pub async fn load_model(&self, model_id: &str, data: Vec<u8>) {
        let len = data.len();
        self.models.write().await.insert(model_id.to_string(), data);
        log::info!("RenderServer: loaded model {} ({} bytes)", model_id, len);
    }

    /// Unload a model from the server.
    pub async fn unload_model(&self, model_id: &str) {
        self.models.write().await.remove(model_id);
    }

    /// Process a render request from a client.
    ///
    /// Applies frame throttling and quality adaptation. Returns `None` if
    /// the frame should be skipped due to throttling.
    pub async fn render(&self, session_id: &str, request: RenderRequest) -> Option<RenderResponse> {
        // Check throttle
        {
            let throttle = self.throttle_states.read().await;
            if let Some(state) = throttle.get(session_id) {
                if !state.should_render() {
                    log::debug!("RenderServer: throttled frame for session {}", session_id);
                    return None;
                }
            } else {
                return None; // Unknown session
            }
        }

        // Adapt quality based on bandwidth
        let effective_lod = {
            let sessions = self.sessions.read().await;
            if let Some(session) = sessions.get(session_id) {
                request.lod.min(session.bandwidth.recommended_lod())
            } else {
                return None;
            }
        };

        let start = Instant::now();

        // In production: headless rendering pipeline using wgpu
        // This would:
        // 1. Set up a wgpu headless surface with the requested viewport
        // 2. Load the model's mesh data into GPU buffers
        // 3. Set up camera uniforms from the request
        // 4. Render with quality adapted by effective_lod
        // 5. Read back the rendered image as PNG bytes
        let image_data = self.headless_render_placeholder(&request, effective_lod);

        let render_time = start.elapsed().as_secs_f64() * 1000.0;

        // Record throttle
        {
            let mut throttle = self.throttle_states.write().await;
            if let Some(state) = throttle.get_mut(session_id) {
                state.record_render();
            }
        }

        // Update session
        {
            let mut sessions = self.sessions.write().await;
            if let Some(session) = sessions.get_mut(session_id) {
                session.last_render_time = Some(render_time);
                session.camera_state = request.camera.clone();
            }
        }

        log::debug!(
            "RenderServer: rendered frame for session {} in {:.1}ms (LOD: {:.2})",
            session_id,
            render_time,
            effective_lod
        );

        Some(RenderResponse {
            image_data,
            render_time_ms: render_time,
        })
    }

    /// Update the bandwidth estimation for a session.
    pub async fn update_bandwidth(&self, session_id: &str, bandwidth: BandwidthClass) {
        let mut sessions = self.sessions.write().await;
        let mut throttle = self.throttle_states.write().await;

        if let Some(session) = sessions.get_mut(session_id) {
            session.bandwidth = bandwidth;
            session.current_lod = bandwidth.recommended_lod();
        }
        if let Some(state) = throttle.get_mut(session_id) {
            state.update_bandwidth(bandwidth);
        }
    }

    /// Get the number of active sessions.
    pub async fn session_count(&self) -> usize {
        self.sessions.read().await.len()
    }

    /// Get the number of loaded models.
    pub async fn model_count(&self) -> usize {
        self.models.read().await.len()
    }

    /// Placeholder for the headless rendering pipeline.
    ///
    /// In production this would use wgpu to render the model off-screen.
    /// For now, returns a minimal 1×1 PNG as proof of concept.
    fn headless_render_placeholder(&self, _request: &RenderRequest, _lod: f64) -> Vec<u8> {
        // Minimal valid PNG: 1x1 pixel, RGBA
        // This is a 1x1 white pixel PNG
        vec![
            0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, // PNG signature
            0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52, // IHDR chunk
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, // 1x1
            0x08, 0x02, 0x00, 0x00, 0x00, 0x90, 0x77, 0x53, // 8-bit RGB
            0xDE, 0x00, 0x00, 0x00, 0x0C, 0x49, 0x44, 0x41, // IDAT chunk
            0x54, 0x08, 0xD7, 0x63, 0xF8, 0xCF, 0xC0, 0x00, // compressed data
            0x00, 0x00, 0x02, 0x00, 0x01, 0xE2, 0x21, 0xBC, // CRC
            0x33, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, // IEND chunk
            0x44, 0xAE, 0x42, 0x60, 0x82,                     // CRC
        ]
    }
}

impl Default for RenderServer {
    fn default() -> Self {
        Self::new()
    }
}

/// WebSocket message types for the render server protocol.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum WsMessage {
    /// Client requests a new session.
    CreateSession { bandwidth_kbps: Option<u32> },
    /// Server responds with session info.
    SessionCreated { session_id: String, lod: f64 },
    /// Client sends a render request.
    RenderRequest { session_id: String, request: RenderRequest },
    /// Server sends a render response.
    RenderResponse { session_id: String, response: RenderResponse },
    /// Frame was throttled (skipped).
    FrameThrottled { session_id: String },
    /// Client updates bandwidth estimate.
    UpdateBandwidth { session_id: String, bandwidth_kbps: u32 },
    /// Client destroys session.
    DestroySession { session_id: String },
    /// Error message.
    Error { message: String },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_create_and_destroy_session() {
        let server = RenderServer::new();
        let session_id = server.create_session().await;

        assert!(server.get_session(&session_id).await.is_some());
        assert_eq!(server.session_count().await, 1);

        server.destroy_session(&session_id).await;
        assert!(server.get_session(&session_id).await.is_none());
        assert_eq!(server.session_count().await, 0);
    }

    #[tokio::test]
    async fn test_session_with_bandwidth() {
        let server = RenderServer::new();
        let session_id = server.create_session_with_bandwidth(BandwidthClass::High).await;

        let session = server.get_session(&session_id).await.unwrap();
        assert_eq!(session.bandwidth, BandwidthClass::High);
        assert!((session.current_lod - 1.0).abs() < 0.01);
    }

    #[tokio::test]
    async fn test_render_request() {
        let server = RenderServer::new();
        let session_id = server.create_session().await;

        let request = RenderRequest {
            camera: CameraState::default(),
            viewport: Viewport::default(),
            model_id: "test_model".to_string(),
            lod: 1.0,
        };

        let response = server.render(&session_id, request).await;
        assert!(response.is_some());

        let resp = response.unwrap();
        assert!(resp.render_time_ms >= 0.0);
        assert!(!resp.image_data.is_empty());
    }

    #[tokio::test]
    async fn test_render_unknown_session() {
        let server = RenderServer::new();
        let request = RenderRequest {
            camera: CameraState::default(),
            viewport: Viewport::default(),
            model_id: "test".to_string(),
            lod: 1.0,
        };

        let response = server.render("nonexistent", request).await;
        assert!(response.is_none());
    }

    #[tokio::test]
    async fn test_load_model() {
        let server = RenderServer::new();
        server.load_model("cube", vec![1, 2, 3]).await;
        assert_eq!(server.model_count().await, 1);

        server.unload_model("cube").await;
        assert_eq!(server.model_count().await, 0);
    }

    #[tokio::test]
    async fn test_add_model_to_session() {
        let server = RenderServer::new();
        let session_id = server.create_session().await;

        server.add_model_to_session(&session_id, "cube").await;
        server.add_model_to_session(&session_id, "cube").await; // Duplicate should not be added

        let session = server.get_session(&session_id).await.unwrap();
        assert_eq!(session.loaded_models.len(), 1);
    }

    #[tokio::test]
    async fn test_update_camera() {
        let server = RenderServer::new();
        let session_id = server.create_session().await;

        let new_camera = CameraState {
            eye: [10.0, 20.0, 30.0],
            ..Default::default()
        };
        server.update_camera(&session_id, new_camera.clone()).await;

        let session = server.get_session(&session_id).await.unwrap();
        assert_eq!(session.camera_state.eye, [10.0, 20.0, 30.0]);
    }

    #[tokio::test]
    async fn test_update_bandwidth() {
        let server = RenderServer::new();
        let session_id = server.create_session().await;

        server.update_bandwidth(&session_id, BandwidthClass::Low).await;

        let session = server.get_session(&session_id).await.unwrap();
        assert_eq!(session.bandwidth, BandwidthClass::Low);
        assert!((session.current_lod - 0.25).abs() < 0.01);
    }

    #[test]
    fn test_bandwidth_class_from_kbps() {
        assert_eq!(BandwidthClass::from_kbps(100), BandwidthClass::Low);
        assert_eq!(BandwidthClass::from_kbps(1000), BandwidthClass::Medium);
        assert_eq!(BandwidthClass::from_kbps(5000), BandwidthClass::High);
    }

    #[test]
    fn test_ws_message_serialization() {
        let msg = WsMessage::CreateSession {
            bandwidth_kbps: Some(1500),
        };
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("CreateSession"));

        let parsed: WsMessage = serde_json::from_str(&json).unwrap();
        if let WsMessage::CreateSession { bandwidth_kbps } = parsed {
            assert_eq!(bandwidth_kbps, Some(1500));
        } else {
            panic!("Wrong message type");
        }
    }
}
