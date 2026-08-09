// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! AI panel — integrates draper-ai (ShapeParser, DesignReviewer, LLM) into the UI.
//!
//! Per Phase 5 UI integration: provides an interactive panel where the user
//! can type a natural-language prompt ("box 50x30x20 with 4 holes"), see
//! the parsed geometry actions, run a design review, and (optionally)
//! expand vague prompts via an LLM.
//!
//! # Layout
//!
//! ```text
//! ┌── AI Assistant ──────────────────────┐
//! │ Prompt: [____________________] [Go]  │
//! │                                       │
//! │ Parsed Actions:                       │
//! │   • Create box 50×30×20 at (0,0,0)    │
//! │   • Create cylinder Ø5×20 at (...)    │
//! │   • Subtract last shape               │
//! │   • ...                               │
//! │                                       │
//! │ [Apply to Scene] [Review] [Clear]     │
//! │                                       │
//! │ Design Review:                        │
//! │   Score: 85/100                       │
//! │   ⚠ Hole diameter 15mm is large...    │
//! │   ℹ No fillets applied...             │
//! │                                       │
//! │ LLM Backend: [Mock ▼]                 │
//! └───────────────────────────────────────┘
//! ```

use eframe::egui;
use draper_ai::{
    AiDesignReviewer as DesignReviewer, GeometryAction, MockLlmClient, ShapeParser,
    parse_with_llm, LlmClient, ReviewReport, ReviewSeverity,
};
use std::sync::Arc;

/// State for the AI panel.
pub struct AiPanelState {
    /// Current text in the prompt input.
    pub prompt: String,
    /// Last parsed actions (if parsing succeeded).
    pub actions: Vec<GeometryAction>,
    /// Last parse error (if parsing failed).
    pub parse_error: Option<String>,
    /// Last design review report (if review was run).
    pub review: Option<ReviewReport>,
    /// Selected LLM backend name.
    pub llm_backend: LlmBackend,
    /// Whether an LLM expansion is in progress.
    pub llm_loading: bool,
    /// The shape parser instance.
    pub parser: ShapeParser,
    /// The design reviewer instance.
    pub reviewer: DesignReviewer,
    /// The LLM client (wrapped in Arc for trait-object sharing).
    pub llm_client: Arc<dyn LlmClient>,
    /// Status message (shown at bottom of panel).
    pub status: String,
}

/// Available LLM backends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LlmBackend {
    /// Mock LLM — returns canned responses based on keywords.
    Mock,
    /// HTTP LLM — calls an OpenAI-compatible API (not yet wired).
    Http,
}

impl LlmBackend {
    pub fn label(&self) -> &'static str {
        match self {
            LlmBackend::Mock => "Mock (offline)",
            LlmBackend::Http => "HTTP (Ollama/OpenAI)",
        }
    }

    pub fn all() -> [LlmBackend; 2] {
        [LlmBackend::Mock, LlmBackend::Http]
    }
}

impl Default for AiPanelState {
    fn default() -> Self {
        Self {
            prompt: String::new(),
            actions: Vec::new(),
            parse_error: None,
            review: None,
            llm_backend: LlmBackend::Mock,
            llm_loading: false,
            parser: ShapeParser::new(),
            reviewer: DesignReviewer::new(),
            llm_client: Arc::new(MockLlmClient::new()),
            status: "Type a prompt and press Go".to_string(),
        }
    }
}

impl AiPanelState {
    /// Parse the current prompt using the rule-based parser.
    pub fn parse_prompt(&mut self) {
        if self.prompt.trim().is_empty() {
            self.status = "Prompt is empty".to_string();
            return;
        }
        match self.parser.parse(&self.prompt) {
            Ok(actions) => {
                self.actions = actions;
                self.parse_error = None;
                self.status = format!("Parsed {} action(s)", self.actions.len());
            }
            Err(e) => {
                self.parse_error = Some(format!("{}", e));
                self.actions.clear();
                self.status = format!("Parse error: {}", e);
            }
        }
        // Clear any stale review
        self.review = None;
    }

    /// Expand the prompt via the LLM, then parse.
    ///
    /// This is async in principle, but since egui is single-threaded we
    /// block on it. For the MockLlmClient this is instant; for a real
    /// HTTP client we'd need to spawn a background task.
    pub fn expand_with_llm(&mut self) {
        if self.prompt.trim().is_empty() {
            self.status = "Prompt is empty".to_string();
            return;
        }
        self.llm_loading = true;
        self.status = "Expanding prompt via LLM...".to_string();

        // For now, we use the blocking approach since MockLlmClient is instant.
        // A real implementation would use egui's async facilities or a
        // background thread + channel.
        let parser = &self.parser;
        let llm = self.llm_client.as_ref();
        // We can't easily call async code here without a runtime, so we
        // use a small inline runtime for the mock client.
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build();
        match rt {
            Ok(runtime) => {
                let result = runtime.block_on(parse_with_llm(parser, llm, &self.prompt));
                match result {
                    Ok(actions) => {
                        self.actions = actions;
                        self.parse_error = None;
                        self.status = format!("LLM expanded → {} action(s)", self.actions.len());
                    }
                    Err(e) => {
                        self.parse_error = Some(format!("{}", e));
                        self.actions.clear();
                        self.status = format!("LLM parse error: {}", e);
                    }
                }
            }
            Err(_) => {
                // Fallback: try direct parsing without LLM
                self.parse_prompt();
            }
        }
        self.llm_loading = false;
        self.review = None;
    }

    /// Run design review on the current actions.
    pub fn run_review(&mut self) {
        if self.actions.is_empty() {
            self.status = "No actions to review — parse a prompt first".to_string();
            return;
        }
        self.review = Some(self.reviewer.review(&self.actions));
        let score = self.review.as_ref().map(|r| r.score).unwrap_or(0.0);
        self.status = format!("Review complete — score {:.0}/100", score);
    }

    /// Clear all state.
    pub fn clear(&mut self) {
        self.prompt.clear();
        self.actions.clear();
        self.parse_error = None;
        self.review = None;
        self.status = "Cleared".to_string();
    }

    /// Switch the LLM backend.
    pub fn switch_backend(&mut self, backend: LlmBackend) {
        self.llm_backend = backend;
        self.llm_client = match backend {
            LlmBackend::Mock => Arc::new(MockLlmClient::new()),
            LlmBackend::Http => {
                // HTTP backend not yet implemented — fall back to mock
                self.status = "HTTP LLM not yet implemented, using Mock".to_string();
                Arc::new(MockLlmClient::new())
            }
        };
    }
}

/// Render the AI panel.
///
/// Returns `Some(action)` if the user clicked "Apply to Scene", containing
/// the parsed actions. The caller (ViewerApp) is responsible for converting
/// these actions into actual solids and adding them to the scene.
pub fn render_ai_panel(ui: &mut egui::Ui, state: &mut AiPanelState) -> Option<Vec<GeometryAction>> {
    let mut apply_clicked = false;

    ui.heading(egui::RichText::new("AI Assistant").size(14.0).strong());
    ui.separator();

    // === Prompt input ===
    ui.label(egui::RichText::new("Natural-language prompt:").size(11.0));
    ui.horizontal(|ui| {
        let resp = ui.add(
            egui::TextEdit::singleline(&mut state.prompt)
                .hint_text("e.g., 'box 50x30x20 with 4 holes'")
                .desired_width(ui.available_width() - 120.0)
                .code_editor(),
        );
        if ui.button("Go").clicked() || (resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter))) {
            state.parse_prompt();
        }
    });

    // === Quick examples ===
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new("Examples:").size(10.0).weak());
        for example in &["box 50x30x20", "cylinder 10x50", "sphere 20", "box 50x50x5 holes of diameter 5 fillet 2"] {
            if ui.small_button(*example).clicked() {
                state.prompt = example.to_string();
                state.parse_prompt();
            }
        }
    });

    ui.add_space(4.0);

    // === Parsed actions ===
    if !state.actions.is_empty() {
        ui.label(egui::RichText::new(format!("Parsed Actions ({}):", state.actions.len())).size(11.0).strong());
        egui::ScrollArea::vertical()
            .max_height(150.0)
            .show(ui, |ui| {
                for (i, action) in state.actions.iter().enumerate() {
                    let icon = match action {
                        GeometryAction::CreateBox { .. } => "▭",
                        GeometryAction::CreateCylinder { .. } => "⬭",
                        GeometryAction::CreateSphere { .. } => "◯",
                        GeometryAction::CreateCone { .. } => "△",
                        GeometryAction::CreateTorus { .. } => "◎",
                        GeometryAction::BooleanSubtract => "∖",
                        GeometryAction::BooleanUnion => "∪",
                        GeometryAction::BooleanIntersect => "∩",
                        GeometryAction::FilletAllEdges { .. } => "◜",
                        GeometryAction::ChamferAllEdges { .. } => "◹",
                        GeometryAction::Shell { .. } => "⬚",
                        GeometryAction::ExtrudeProfile { .. } => "⬆",
                        GeometryAction::RevolveProfile { .. } => "↻",
                    };
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(format!("{}. {} {}", i + 1, icon, action.describe())).size(10.0));
                    });
                }
            });
    } else if let Some(err) = &state.parse_error {
        ui.label(egui::RichText::new(format!("Error: {}", err)).size(11.0).color(egui::Color32::from_rgb(220, 80, 80)));
    }

    // === Action buttons ===
    ui.horizontal(|ui| {
        if ui.add_enabled(!state.actions.is_empty(), egui::Button::new("Apply to Scene")).clicked() {
            apply_clicked = true;
        }
        if ui.add_enabled(!state.actions.is_empty(), egui::Button::new("Review")).clicked() {
            state.run_review();
        }
        if ui.button("Expand via LLM").clicked() {
            state.expand_with_llm();
        }
        if ui.button("Clear").clicked() {
            state.clear();
        }
    });

    ui.add_space(4.0);

    // === Design review report ===
    if let Some(report) = &state.review {
        ui.label(egui::RichText::new("Design Review:").size(11.0).strong());
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(format!("Score: {:.0}/100", report.score)).size(12.0).strong());
            let color = if report.score >= 80.0 {
                egui::Color32::from_rgb(80, 180, 80)
            } else if report.score >= 50.0 {
                egui::Color32::from_rgb(220, 180, 80)
            } else {
                egui::Color32::from_rgb(220, 80, 80)
            };
            ui.add(egui::ProgressBar::new((report.score / 100.0) as f32).fill(color).desired_width(100.0));
        });
        ui.label(egui::RichText::new(format!(
            "{} error(s), {} warning(s), {} info",
            report.stats.error_count, report.stats.warning_count, report.stats.info_count
        )).size(10.0).weak());

        egui::ScrollArea::vertical()
            .max_height(120.0)
            .show(ui, |ui| {
                for issue in &report.issues {
                    let (icon, color) = match issue.severity {
                        ReviewSeverity::Error => ("ERROR", egui::Color32::from_rgb(220, 80, 80)),
                        ReviewSeverity::Warning => ("WARN", egui::Color32::from_rgb(220, 180, 80)),
                        ReviewSeverity::Info => ("INFO", egui::Color32::from_rgb(100, 160, 220)),
                    };
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(format!("[{}]", icon)).size(10.0).color(color));
                        ui.label(egui::RichText::new(&issue.message).size(10.0));
                    });
                    if let Some(suggestion) = &issue.suggestion {
                        ui.horizontal(|ui| {
                            ui.add_space(20.0);
                            ui.label(egui::RichText::new(format!("→ {}", suggestion)).size(9.0).weak().color(egui::Color32::from_rgb(120, 200, 120)));
                        });
                    }
                }
            });
    }

    ui.add_space(4.0);

    // === LLM backend selector ===
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("LLM:").size(10.0).weak());
        egui::ComboBox::from_id_salt("llm_backend_combo")
            .selected_text(state.llm_backend.label())
            .show_ui(ui, |ui| {
                for backend in LlmBackend::all() {
                    if ui.selectable_label(state.llm_backend == backend, backend.label()).clicked() {
                        state.switch_backend(backend);
                    }
                }
            });
        if state.llm_loading {
            ui.spinner();
        }
    });

    // === Status bar ===
    ui.add_space(2.0);
    ui.separator();
    ui.label(egui::RichText::new(&state.status).size(10.0).weak());

    if apply_clicked {
        Some(state.actions.clone())
    } else {
        None
    }
}

/// Convert geometry actions into solids and add them to the scene.
///
/// This is a helper that the ViewerApp can call when the user clicks
/// "Apply to Scene". It processes the actions sequentially, building up
/// a list of solids.
///
/// Returns the list of created solids (the last one is the "final" result
/// after all boolean operations).
pub fn actions_to_solids(actions: &[GeometryAction]) -> Vec<draper_topology::Solid> {
    use draper_geometry::Transform;
    use draper_topology::{ShapeBuilder, boolean};
    let mut solids: Vec<draper_topology::Solid> = Vec::new();
    let tol_ctx = draper_geometry::ToleranceContext::default();

    for action in actions {
        match action {
            GeometryAction::CreateBox { size, center } => {
                let mut solid = ShapeBuilder::make_box(size[0], size[1], size[2]);
                if center != &[0.0, 0.0, 0.0] {
                    ShapeBuilder::transform_solid(&mut solid, &Transform::translation(center[0], center[1], center[2]));
                }
                solids.push(solid);
            }
            GeometryAction::CreateCylinder { diameter, height, center } => {
                let mut solid = ShapeBuilder::make_cylinder(*diameter / 2.0, *height);
                if center != &[0.0, 0.0, 0.0] {
                    ShapeBuilder::transform_solid(&mut solid, &Transform::translation(center[0], center[1], center[2]));
                }
                solids.push(solid);
            }
            GeometryAction::CreateSphere { diameter, center } => {
                let mut solid = ShapeBuilder::make_sphere(*diameter / 2.0);
                if center != &[0.0, 0.0, 0.0] {
                    ShapeBuilder::transform_solid(&mut solid, &Transform::translation(center[0], center[1], center[2]));
                }
                solids.push(solid);
            }
            GeometryAction::CreateCone { bottom_diameter, top_diameter, height, center } => {
                // make_cone takes (radius, height, half_angle) — compute half_angle from top/bottom
                let half_angle = if *height > 0.0 {
                    ((bottom_diameter - top_diameter) / 2.0 / height).atan()
                } else {
                    0.0
                };
                let mut solid = ShapeBuilder::make_cone(*bottom_diameter / 2.0, *height, half_angle);
                if center != &[0.0, 0.0, 0.0] {
                    ShapeBuilder::transform_solid(&mut solid, &Transform::translation(center[0], center[1], center[2]));
                }
                solids.push(solid);
            }
            GeometryAction::CreateTorus { major_diameter, minor_diameter, center } => {
                let mut solid = ShapeBuilder::make_torus(*major_diameter / 2.0, *minor_diameter / 2.0);
                if center != &[0.0, 0.0, 0.0] {
                    ShapeBuilder::transform_solid(&mut solid, &Transform::translation(center[0], center[1], center[2]));
                }
                solids.push(solid);
            }
            GeometryAction::BooleanSubtract => {
                if solids.len() >= 2 {
                    if let Some(tool) = solids.pop() {
                        if let Some(target) = solids.last().cloned() {
                            if let Ok(result) = boolean::boolean_subtract(&target, &tool, &tol_ctx) {
                                if let Some(last) = solids.last_mut() {
                                    *last = result;
                                }
                            }
                        }
                    }
                }
            }
            GeometryAction::BooleanUnion => {
                if solids.len() >= 2 {
                    if let Some(tool) = solids.pop() {
                        if let Some(target) = solids.last().cloned() {
                            if let Ok(result) = boolean::boolean_union(&target, &tool, &tol_ctx) {
                                if let Some(last) = solids.last_mut() {
                                    *last = result;
                                }
                            }
                        }
                    }
                }
            }
            GeometryAction::BooleanIntersect => {
                if solids.len() >= 2 {
                    if let Some(tool) = solids.pop() {
                        if let Some(target) = solids.last().cloned() {
                            if let Ok(result) = boolean::boolean_intersect(&target, &tool, &tol_ctx) {
                                if let Some(last) = solids.last_mut() {
                                    *last = result;
                                }
                            }
                        }
                    }
                }
            }
            GeometryAction::FilletAllEdges { radius: _ } => {
                // fillet_edge takes (solid, edge_index, radius) — would need to iterate all edges
                // For now, skip (requires knowing edge count)
                log::warn!("FilletAllEdges not yet fully supported in actions_to_solids");
            }
            GeometryAction::ChamferAllEdges { distance: _ } => {
                log::warn!("ChamferAllEdges not yet fully supported in actions_to_solids");
            }
            GeometryAction::Shell { thickness } => {
                if let Some(target) = solids.last().cloned() {
                    if let Ok(result) = draper_topology::shell_solid(&target, *thickness) {
                        if let Some(last) = solids.last_mut() {
                            *last = result;
                        }
                    }
                }
            }
            GeometryAction::ExtrudeProfile { .. } | GeometryAction::RevolveProfile { .. } => {
                // These would require sketch profile conversion — skip for now
                log::warn!("ExtrudeProfile/RevolveProfile not yet supported in actions_to_solids");
            }
        }
    }

    solids
}

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_ai_panel_default_state() {
        let state = AiPanelState::default();
        assert!(state.prompt.is_empty());
        assert!(state.actions.is_empty());
        assert!(state.parse_error.is_none());
        assert!(state.review.is_none());
        assert_eq!(state.llm_backend, LlmBackend::Mock);
    }

    #[test]
    fn test_parse_prompt_simple_box() {
        let mut state = AiPanelState {
            prompt: "box 50x30x20".to_string(),
            ..Default::default()
        };
        state.parse_prompt();
        assert_eq!(state.actions.len(), 1);
        assert!(state.parse_error.is_none());
    }

    #[test]
    fn test_parse_prompt_error() {
        let mut state = AiPanelState {
            prompt: "gibberish".to_string(),
            ..Default::default()
        };
        state.parse_prompt();
        assert!(state.actions.is_empty());
        assert!(state.parse_error.is_some());
    }

    #[test]
    fn test_parse_prompt_empty() {
        let mut state = AiPanelState {
            prompt: "".to_string(),
            ..Default::default()
        };
        state.parse_prompt();
        assert!(state.actions.is_empty());
        assert!(state.status.contains("empty"));
    }

    #[test]
    fn test_run_review_no_actions() {
        let mut state = AiPanelState::default();
        state.run_review();
        assert!(state.review.is_none());
        assert!(state.status.contains("No actions"));
    }

    #[test]
    fn test_run_review_with_actions() {
        let mut state = AiPanelState {
            prompt: "box 50x30x20".to_string(),
            ..Default::default()
        };
        state.parse_prompt();
        state.run_review();
        assert!(state.review.is_some());
        assert!(state.status.contains("Review complete"));
    }

    #[test]
    fn test_clear() {
        let mut state = AiPanelState {
            prompt: "box 50x30x20".to_string(),
            ..Default::default()
        };
        state.parse_prompt();
        state.run_review();
        state.clear();
        assert!(state.prompt.is_empty());
        assert!(state.actions.is_empty());
        assert!(state.review.is_none());
        assert!(state.parse_error.is_none());
    }

    #[test]
    fn test_switch_backend() {
        let mut state = AiPanelState::default();
        assert_eq!(state.llm_backend, LlmBackend::Mock);
        state.switch_backend(LlmBackend::Http);
        assert_eq!(state.llm_backend, LlmBackend::Http);
        // Should fall back to Mock since HTTP is not implemented
        // (we can't easily verify the Arc<dyn> here, but the status should be set)
        assert!(state.status.contains("HTTP LLM not yet implemented") || state.status.contains("Mock"));
    }

    #[test]
    fn test_llm_backend_labels() {
        assert_eq!(LlmBackend::Mock.label(), "Mock (offline)");
        assert_eq!(LlmBackend::Http.label(), "HTTP (Ollama/OpenAI)");
        assert_eq!(LlmBackend::all().len(), 2);
    }

    #[test]
    fn test_expand_with_llm_mock() {
        let mut state = AiPanelState {
            prompt: "I need a bracket".to_string(),
            ..Default::default()
        };
        state.expand_with_llm();
        // MockLlmClient returns "box 50x30x5 holes of diameter 5 fillet 2" for "bracket"
        assert!(!state.actions.is_empty());
        assert!(state.parse_error.is_none());
        assert!(state.status.contains("LLM expanded"));
    }

    #[test]
    fn test_actions_to_solids_box() {
        let actions = vec![GeometryAction::CreateBox {
            size: [10.0, 20.0, 30.0],
            center: [0.0, 0.0, 0.0],
        }];
        let solids = actions_to_solids(&actions);
        assert_eq!(solids.len(), 1);
    }

    #[test]
    fn test_actions_to_solids_boolean_subtract() {
        let actions = vec![
            GeometryAction::CreateBox {
                size: [50.0, 50.0, 20.0],
                center: [0.0, 0.0, 0.0],
            },
            GeometryAction::CreateCylinder {
                diameter: 5.0,
                height: 20.0,
                center: [10.0, 10.0, 0.0],
            },
            GeometryAction::BooleanSubtract,
        ];
        let solids = actions_to_solids(&actions);
        // After subtract, we should have 1 solid (the box with the hole)
        assert_eq!(solids.len(), 1);
    }

    #[test]
    fn test_actions_to_solids_empty() {
        let solids = actions_to_solids(&[]);
        assert!(solids.is_empty());
    }
}
