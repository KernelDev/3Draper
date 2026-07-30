// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Status bar — bottom panel showing coordinates, tool, FPS, units.

use eframe::egui;
use super::UiState;

/// Render the status bar at the bottom of the window.
pub fn render_status_bar(ctx: &egui::Context, state: &UiState) {
    egui::TopBottomPanel::bottom("status_bar")
        .exact_height(24.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "X {:.2}  Y {:.2}  Z {:.2}",
                    state.mouse_world[0], state.mouse_world[1], state.mouse_world[2]
                ));
                ui.separator();
                ui.label(format!(
                    "Az {:.1}°  El {:.1}°  D {:.1}",
                    state.camera_info[0], state.camera_info[1], state.camera_info[2]
                ));
                ui.separator();
                ui.label(format!("Tool: {}", state.active_tool));
                ui.separator();
                ui.label(format!("FPS: {:.0}", state.fps));
                ui.separator();
                ui.label(format!("{}", state.units));
                ui.separator();
                ui.label(format!("{}", match state.display_style {
                    super::DisplayStyle::Wireframe => "Wireframe",
                    super::DisplayStyle::Shaded => "Shaded",
                    super::DisplayStyle::ShadedWithEdges => "Shaded + Edges",
                }));
                ui.separator();
                ui.label(&state.view_orientation);
                ui.separator();
                if state.selection_count > 0 {
                    ui.label(format!("Sel: {} {}", state.selection_count,
                        if state.selection_count == 1 { "item" } else { "items" }));
                } else {
                    ui.label("Ready");
                }
            });
        });
}
