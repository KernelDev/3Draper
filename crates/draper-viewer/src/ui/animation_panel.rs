// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Animation Timeline panel (mockups 92, 96).
//!
//! Per Phase 6: provides a timeline panel for creating and playing
//! animations of assembly motion, camera paths, and parameter changes.
//!
//! # Layout
//!
//! ```text
//! ┌── Animation Timeline ──────────────────────────────┐
//! │ [▶ Play] [⏺ Rec] [⏸ Pause] [⟲ Loop] [⇄ Ping-pong] │
//! │                                                      │
//! │ Frame: [====●===========] 45 / 120  (24 fps)        │
//! │                                                      │
//! │ Tracks:                                              │
//! │ Camera    │────●─────────────●──────────│           │
//! │ Solid_1   │●─────────●─────────────●────│           │
//! │ Solid_2   │────●─────────●──────────●───│           │
//! │ Param_W   │────────●──────────●─────────│           │
//! │                                                      │
//! │ [Add Track] [Add Keyframe] [Delete] [Clear All]     │
//! └──────────────────────────────────────────────────────┘
//! ```

use eframe::egui;

/// A single keyframe in the animation.
#[derive(Clone, Debug)]
pub struct Keyframe {
    /// Frame number (0-based).
    pub frame: u32,
    /// Value at this keyframe (interpretation depends on track type).
    pub value: f32,
    /// Easing type.
    pub easing: Easing,
}

/// Easing function for interpolation between keyframes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Easing {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
    Step,
}

impl Easing {
    pub fn label(&self) -> &'static str {
        match self {
            Easing::Linear => "Linear",
            Easing::EaseIn => "Ease In",
            Easing::EaseOut => "Ease Out",
            Easing::EaseInOut => "Ease In-Out",
            Easing::Step => "Step",
        }
    }

    pub fn all() -> [Easing; 5] {
        [Easing::Linear, Easing::EaseIn, Easing::EaseOut, Easing::EaseInOut, Easing::Step]
    }

    /// Apply easing to a normalized time t ∈ [0, 1].
    pub fn apply(&self, t: f32) -> f32 {
        match self {
            Easing::Linear => t,
            Easing::EaseIn => t * t,
            Easing::EaseOut => 1.0 - (1.0 - t) * (1.0 - t),
            Easing::EaseInOut => if t < 0.5 {
                2.0 * t * t
            } else {
                1.0 - 2.0 * (1.0 - t) * (1.0 - t)
            },
            Easing::Step => if t >= 1.0 { 1.0 } else { 0.0 },
        }
    }
}

/// A track in the animation timeline.
#[derive(Clone, Debug)]
pub struct AnimationTrack {
    pub id: u32,
    pub name: String,
    pub keyframes: Vec<Keyframe>,
    pub color: [u8; 3],
}

/// Playback state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaybackState {
    Stopped,
    Playing,
    Paused,
    Recording,
}

/// Animation timeline state.
#[derive(Clone, Debug)]
pub struct AnimationTimelineState {
    /// All tracks.
    pub tracks: Vec<AnimationTrack>,
    /// Current frame.
    pub current_frame: u32,
    /// Total number of frames.
    pub total_frames: u32,
    /// Frames per second.
    pub fps: f32,
    /// Playback state.
    pub playback: PlaybackState,
    /// Loop mode.
    pub loop_mode: LoopMode,
    /// Next track ID.
    next_track_id: u32,
    /// Whether the panel is visible.
    pub visible: bool,
}

/// Loop mode for playback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoopMode {
    NoLoop,
    Loop,
    PingPong,
}

impl LoopMode {
    pub fn label(&self) -> &'static str {
        match self {
            LoopMode::NoLoop => "No Loop",
            LoopMode::Loop => "Loop",
            LoopMode::PingPong => "Ping-Pong",
        }
    }
}

impl Default for AnimationTimelineState {
    fn default() -> Self {
        Self {
            tracks: vec![
                AnimationTrack {
                    id: 0,
                    name: "Camera".to_string(),
                    keyframes: vec![
                        Keyframe { frame: 0, value: 0.0, easing: Easing::Linear },
                        Keyframe { frame: 60, value: 45.0, easing: Easing::EaseInOut },
                        Keyframe { frame: 120, value: 0.0, easing: Easing::EaseOut },
                    ],
                    color: [137, 180, 250], // Blue
                },
                AnimationTrack {
                    id: 1,
                    name: "Solid_1".to_string(),
                    keyframes: vec![
                        Keyframe { frame: 0, value: 0.0, easing: Easing::Linear },
                        Keyframe { frame: 40, value: 50.0, easing: Easing::EaseIn },
                        Keyframe { frame: 80, value: 50.0, easing: Easing::Step },
                        Keyframe { frame: 120, value: 0.0, easing: Easing::EaseOut },
                    ],
                    color: [166, 227, 161], // Green
                },
            ],
            current_frame: 0,
            total_frames: 120,
            fps: 24.0,
            playback: PlaybackState::Stopped,
            loop_mode: LoopMode::Loop,
            next_track_id: 2,
            visible: false,
        }
    }
}

impl AnimationTimelineState {
    /// Advance one frame.
    pub fn advance_frame(&mut self) {
        if self.playback != PlaybackState::Playing {
            return;
        }
        self.current_frame += 1;
        if self.current_frame >= self.total_frames {
            match self.loop_mode {
                LoopMode::NoLoop => {
                    self.current_frame = self.total_frames - 1;
                    self.playback = PlaybackState::Stopped;
                }
                LoopMode::Loop => {
                    self.current_frame = 0;
                }
                LoopMode::PingPong => {
                    // Ping-pong would require tracking direction; simplified to loop
                    self.current_frame = 0;
                }
            }
        }
    }

    /// Toggle play/pause.
    pub fn toggle_play(&mut self) {
        match self.playback {
            PlaybackState::Playing => self.playback = PlaybackState::Paused,
            PlaybackState::Paused => self.playback = PlaybackState::Playing,
            PlaybackState::Stopped => {
                if self.current_frame >= self.total_frames - 1 {
                    self.current_frame = 0;
                }
                self.playback = PlaybackState::Playing;
            }
            PlaybackState::Recording => self.playback = PlaybackState::Paused,
        }
    }

    /// Start recording.
    pub fn toggle_record(&mut self) {
        match self.playback {
            PlaybackState::Recording => self.playback = PlaybackState::Stopped,
            _ => self.playback = PlaybackState::Recording,
        }
    }

    /// Add a new track.
    pub fn add_track(&mut self, name: &str) {
        let colors = [[137, 180, 250], [166, 227, 161], [249, 226, 175], [203, 166, 247], [243, 139, 168]];
        let color = colors[(self.next_track_id as usize) % colors.len()];
        self.tracks.push(AnimationTrack {
            id: self.next_track_id,
            name: name.to_string(),
            keyframes: Vec::new(),
            color,
        });
        self.next_track_id += 1;
    }

    /// Add a keyframe to the last track at the current frame.
    pub fn add_keyframe(&mut self, value: f32) {
        if let Some(track) = self.tracks.last_mut() {
            // Remove existing keyframe at current frame if any
            track.keyframes.retain(|kf| kf.frame != self.current_frame);
            track.keyframes.push(Keyframe {
                frame: self.current_frame,
                value,
                easing: Easing::EaseInOut,
            });
            track.keyframes.sort_by_key(|kf| kf.frame);
        }
    }

    /// Clear all keyframes from all tracks.
    pub fn clear_all(&mut self) {
        for track in &mut self.tracks {
            track.keyframes.clear();
        }
        self.current_frame = 0;
        self.playback = PlaybackState::Stopped;
    }

    /// Get interpolated value for a track at the current frame.
    pub fn get_value(&self, track_id: u32) -> Option<f32> {
        let track = self.tracks.iter().find(|t| t.id == track_id)?;
        if track.keyframes.is_empty() {
            return None;
        }
        if track.keyframes.len() == 1 {
            return Some(track.keyframes[0].value);
        }

        // Find surrounding keyframes
        let frame = self.current_frame;
        let mut prev = &track.keyframes[0];
        let mut next = &track.keyframes[0];

        for (i, kf) in track.keyframes.iter().enumerate() {
            if kf.frame <= frame {
                prev = kf;
                next = if i + 1 < track.keyframes.len() { &track.keyframes[i + 1] } else { kf };
            }
        }

        if next.frame == prev.frame {
            return Some(prev.value);
        }

        let t = (frame - prev.frame) as f32 / (next.frame - prev.frame) as f32;
        let eased_t = next.easing.apply(t);
        Some(prev.value + (next.value - prev.value) * eased_t)
    }
}

/// Render the animation timeline panel.
pub fn render_animation_timeline(ui: &mut egui::Ui, state: &mut AnimationTimelineState) {
    ui.heading(egui::RichText::new("Animation Timeline").size(13.0).strong());
    ui.separator();

    // === Playback controls ===
    ui.horizontal(|ui| {
        let play_label = match state.playback {
            PlaybackState::Playing => "⏸ Pause",
            PlaybackState::Stopped | PlaybackState::Paused => "▶ Play",
            PlaybackState::Recording => "⏹ Stop",
        };
        if ui.button(play_label).clicked() {
            state.toggle_play();
        }
        if ui.button("⏺ Rec").clicked() {
            state.toggle_record();
        }
        if ui.button("⏮ Start").clicked() {
            state.current_frame = 0;
        }
        if ui.button("⏭ End").clicked() {
            state.current_frame = state.total_frames - 1;
        }
        ui.separator();
        // Loop mode
        egui::ComboBox::from_id_salt("loop_mode_combo")
            .selected_text(state.loop_mode.label())
            .show_ui(ui, |ui| {
                for mode in [LoopMode::NoLoop, LoopMode::Loop, LoopMode::PingPong] {
                    ui.selectable_value(&mut state.loop_mode, mode, mode.label());
                }
            });
    });

    ui.separator();

    // === Frame slider ===
    ui.horizontal(|ui| {
        ui.label("Frame:");
        let mut frame = state.current_frame as i32;
        ui.add(egui::Slider::new(&mut frame, 0..=(state.total_frames as i32 - 1)));
        state.current_frame = frame.max(0) as u32;
        ui.label(format!("/ {} ({} fps)", state.total_frames, state.fps));
    });

    // Advance frame if playing
    state.advance_frame();

    ui.separator();

    // === Tracks ===
    ui.label(egui::RichText::new("Tracks:").size(11.0).strong());

    let total_frames = state.total_frames;
    let timeline_width = ui.available_width() - 100.0; // Leave space for track name

    egui::ScrollArea::vertical()
        .max_height(150.0)
        .show(ui, |ui| {
            for track in &state.tracks {
                ui.horizontal(|ui| {
                    // Track name
                    ui.label(egui::RichText::new(&track.name)
                        .size(10.0)
                        .color(egui::Color32::from_rgb(track.color[0], track.color[1], track.color[2])));
                    ui.label(" ");

                    // Timeline area
                    let (rect, _) = ui.allocate_exact_size(
                        egui::vec2(timeline_width, 16.0),
                        egui::Sense::click(),
                    );

                    // Draw timeline background
                    ui.painter().rect_filled(
                        rect,
                        2.0_f32,
                        egui::Color32::from_rgb(0x18, 0x18, 0x25),
                    );

                    // Draw keyframes
                    for kf in &track.keyframes {
                        let x = rect.min.x + (kf.frame as f32 / total_frames as f32) * rect.width();
                        let pos = egui::pos2(x, rect.center().y);
                        ui.painter().circle_filled(
                            pos,
                            4.0_f32,
                            egui::Color32::from_rgb(track.color[0], track.color[1], track.color[2]),
                        );
                    }

                    // Draw current frame indicator
                    let cx = rect.min.x + (state.current_frame as f32 / total_frames as f32) * rect.width();
                    ui.painter().line_segment(
                        [egui::pos2(cx, rect.min.y), egui::pos2(cx, rect.max.y)],
                        egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(0xf3, 0x8b, 0xa8)),
                    );
                });
            }
        });

    ui.separator();

    // === Track management ===
    ui.horizontal(|ui| {
        if ui.button("Add Track").clicked() {
            let name = format!("Track_{}", state.next_track_id);
            state.add_track(&name);
        }
        if ui.button("Add Keyframe").clicked() {
            state.add_keyframe(0.0);
        }
        if ui.button("Delete Last Keyframe").clicked() {
            if let Some(track) = state.tracks.last_mut() {
                track.keyframes.pop();
            }
        }
        if ui.button("Clear All").clicked() {
            state.clear_all();
        }
    });

    // === Current values ===
    ui.separator();
    ui.label(egui::RichText::new("Current Values:").size(10.0).weak());
    for track in &state.tracks {
        if let Some(value) = state.get_value(track.id) {
            ui.label(egui::RichText::new(format!("  {}: {:.2}", track.name, value))
                .size(10.0)
                .color(egui::Color32::from_rgb(track.color[0], track.color[1], track.color[2])));
        }
    }
}

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_default_state() {
        let state = AnimationTimelineState::default();
        assert_eq!(state.tracks.len(), 2);
        assert_eq!(state.total_frames, 120);
        assert_eq!(state.fps, 24.0);
        assert_eq!(state.playback, PlaybackState::Stopped);
        assert_eq!(state.loop_mode, LoopMode::Loop);
    }

    #[test]
    fn test_toggle_play() {
        let mut state = AnimationTimelineState::default();
        assert_eq!(state.playback, PlaybackState::Stopped);
        state.toggle_play();
        assert_eq!(state.playback, PlaybackState::Playing);
        state.toggle_play();
        assert_eq!(state.playback, PlaybackState::Paused);
        state.toggle_play();
        assert_eq!(state.playback, PlaybackState::Playing);
    }

    #[test]
    fn test_advance_frame() {
        let mut state = AnimationTimelineState::default();
        state.playback = PlaybackState::Playing;
        state.current_frame = 50;
        state.advance_frame();
        assert_eq!(state.current_frame, 51);
    }

    #[test]
    fn test_advance_frame_loop() {
        let mut state = AnimationTimelineState::default();
        state.playback = PlaybackState::Playing;
        state.loop_mode = LoopMode::Loop;
        state.current_frame = state.total_frames - 1;
        state.advance_frame();
        assert_eq!(state.current_frame, 0);
    }

    #[test]
    fn test_advance_frame_no_loop() {
        let mut state = AnimationTimelineState::default();
        state.playback = PlaybackState::Playing;
        state.loop_mode = LoopMode::NoLoop;
        state.current_frame = state.total_frames - 1;
        state.advance_frame();
        assert_eq!(state.playback, PlaybackState::Stopped);
    }

    #[test]
    fn test_add_track() {
        let mut state = AnimationTimelineState::default();
        let initial_count = state.tracks.len();
        state.add_track("TestTrack");
        assert_eq!(state.tracks.len(), initial_count + 1);
        assert_eq!(state.tracks.last().unwrap().name, "TestTrack");
    }

    #[test]
    fn test_add_keyframe() {
        let mut state = AnimationTimelineState::default();
        state.current_frame = 30;
        let initial_kf_count = state.tracks.last().unwrap().keyframes.len();
        state.add_keyframe(42.0);
        assert_eq!(state.tracks.last().unwrap().keyframes.len(), initial_kf_count + 1);
        // Find the keyframe at frame 30 (sorted, not necessarily last)
        let kf_at_30 = state.tracks.last().unwrap().keyframes.iter()
            .find(|kf| kf.frame == 30)
            .expect("Should find keyframe at frame 30");
        assert!((kf_at_30.value - 42.0).abs() < 1e-6);
    }

    #[test]
    fn test_add_keyframe_replaces_existing() {
        let mut state = AnimationTimelineState::default();
        state.current_frame = 0;
        state.add_keyframe(10.0);
        state.add_keyframe(20.0); // Same frame, should replace
        let track = &state.tracks[1];
        let kfs_at_0: Vec<_> = track.keyframes.iter().filter(|kf| kf.frame == 0).collect();
        assert_eq!(kfs_at_0.len(), 1);
        assert!((kfs_at_0[0].value - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_clear_all() {
        let mut state = AnimationTimelineState::default();
        state.clear_all();
        for track in &state.tracks {
            assert!(track.keyframes.is_empty());
        }
        assert_eq!(state.current_frame, 0);
        assert_eq!(state.playback, PlaybackState::Stopped);
    }

    #[test]
    fn test_get_value_interpolation() {
        let state = AnimationTimelineState::default();
        // Camera track has keyframes at 0 (0.0), 60 (45.0), 120 (0.0)
        // At frame 30 (halfway between 0 and 60), linear interpolation = 22.5
        let value = state.get_value(0).unwrap();
        let mut state = state;
        state.current_frame = 30;
        let value = state.get_value(0).unwrap();
        // With EaseInOut, at t=0.5: 2*0.5*0.5 = 0.5, so value = 0 + 45*0.5 = 22.5
        assert!((value - 22.5).abs() < 1.0, "Expected ~22.5, got {}", value);
    }

    #[test]
    fn test_easing_linear() {
        assert!((Easing::Linear.apply(0.0) - 0.0).abs() < 1e-6);
        assert!((Easing::Linear.apply(0.5) - 0.5).abs() < 1e-6);
        assert!((Easing::Linear.apply(1.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_easing_ease_in() {
        assert!((Easing::EaseIn.apply(0.0) - 0.0).abs() < 1e-6);
        assert!((Easing::EaseIn.apply(0.5) - 0.25).abs() < 1e-6);
        assert!((Easing::EaseIn.apply(1.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_easing_ease_out() {
        assert!((Easing::EaseOut.apply(0.0) - 0.0).abs() < 1e-6);
        assert!((Easing::EaseOut.apply(0.5) - 0.75).abs() < 1e-6);
        assert!((Easing::EaseOut.apply(1.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_easing_step() {
        assert!((Easing::Step.apply(0.0) - 0.0).abs() < 1e-6);
        assert!((Easing::Step.apply(0.99) - 0.0).abs() < 1e-6);
        assert!((Easing::Step.apply(1.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_loop_mode_labels() {
        assert_eq!(LoopMode::NoLoop.label(), "No Loop");
        assert_eq!(LoopMode::Loop.label(), "Loop");
        assert_eq!(LoopMode::PingPong.label(), "Ping-Pong");
    }

    #[test]
    fn test_toggle_record() {
        let mut state = AnimationTimelineState::default();
        assert_eq!(state.playback, PlaybackState::Stopped);
        state.toggle_record();
        assert_eq!(state.playback, PlaybackState::Recording);
        state.toggle_record();
        assert_eq!(state.playback, PlaybackState::Stopped);
    }
}
