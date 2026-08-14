// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Status bar — bottom panel (Phase 1.3: leading icons).

use eframe::egui;
use super::UiState;
use super::icons::draw_icon_in_rect;

pub fn render_status_bar(ctx: &egui::Context, state: &UiState) {
    egui::TopBottomPanel::bottom("status_bar")
        .exact_height(26.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                let fg = ui.style().visuals.widgets.noninteractive.fg_stroke.color;
                status_field(ui, "status_coords", &format!(
                    "X {:.2}  Y {:.2}  Z {:.2}",
                    state.mouse_world[0], state.mouse_world[1], state.mouse_world[2]
                ), fg);
                ui.separator();
                status_field(ui, "status_camera", &format!(
                    "Az {:.1}°  El {:.1}°  D {:.1}",
                    state.camera_info[0], state.camera_info[1], state.camera_info[2]
                ), fg);
                ui.separator();
                status_field(ui, "status_tool", &format!("Tool: {}", state.active_tool), fg);
                ui.separator();
                status_field(ui, "status_fps", &format!("FPS: {:.0}", state.fps), fg);
                ui.separator();
                status_field(ui, "status_units", &format!("{}", state.units), fg);
                ui.separator();
                let style_label = match state.display_style {
                    super::DisplayStyle::Wireframe => "Wireframe",
                    super::DisplayStyle::Shaded => "Shaded",
                    super::DisplayStyle::ShadedWithEdges => "Shaded + Edges",
                    super::DisplayStyle::EdgesOnly => "Edges Only",
                    super::DisplayStyle::ShadedWithMesh => "Shaded + Mesh",
                    super::DisplayStyle::EdgesWithMesh => "Edges + Mesh",
                    super::DisplayStyle::ShadedWithEdgesAndMesh => "All",
                };
                status_field(ui, "iso", style_label, fg);
                ui.separator();
                status_field(ui, "iso", &state.view_orientation, fg);
                ui.separator();
                if state.selection_count > 0 {
                    let sel_text = format!("Sel: {} {}",
                        state.selection_count,
                        if state.selection_count == 1 { "item" } else { "items" });
                    status_field(ui, "status_selection", &sel_text, fg);
                } else {
                    status_field(ui, "check", "Ready", fg);
                }
            });
        });
}

fn status_field(ui: &mut egui::Ui, icon: &str, label: &str, fg: egui::Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
    draw_icon_in_rect(ui.painter(), icon, rect, fg);
    ui.label(label);
}
