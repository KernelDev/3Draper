// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! View modes — Phase 3.
//!
//! Controls display style (Wireframe/Shaded/Shaded+Edges) and
//! view orientation for the 3D viewport.

use eframe::egui;
use super::DisplayStyle;

/// View orientation presets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewOrientation {
    Iso,
    Front,
    Back,
    Top,
    Bottom,
    Left,
    Right,
    Dimetric,
}

impl ViewOrientation {
    pub fn label(&self) -> &'static str {
        match self {
            ViewOrientation::Iso => "ISO",
            ViewOrientation::Front => "Front",
            ViewOrientation::Back => "Back",
            ViewOrientation::Top => "Top",
            ViewOrientation::Bottom => "Bottom",
            ViewOrientation::Left => "Left",
            ViewOrientation::Right => "Right",
            ViewOrientation::Dimetric => "Dimetric",
        }
    }

    /// Camera angles (azimuth, elevation) in degrees.
    pub fn camera_angles(&self) -> (f32, f32) {
        match self {
            ViewOrientation::Iso => (35.0, 25.0),
            ViewOrientation::Front => (0.0, 0.0),
            ViewOrientation::Back => (180.0, 0.0),
            ViewOrientation::Top => (0.0, 90.0),
            ViewOrientation::Bottom => (0.0, -90.0),
            ViewOrientation::Left => (-90.0, 0.0),
            ViewOrientation::Right => (90.0, 0.0),
            ViewOrientation::Dimetric => (20.0, 15.0),
        }
    }

    /// Camera direction vector (looking FROM this direction TO origin).
    /// Used by OrbitCamera::look_from_direction().
    pub fn direction(&self) -> [f32; 3] {
        match self {
            ViewOrientation::Iso => {
                let d = 45.0_f32.to_radians();
                let e = 30.0_f32.to_radians();
                [-e.cos() * d.sin(), -e.sin(), e.cos() * d.cos()]
            }
            ViewOrientation::Front => [0.0, 0.0, 1.0],
            ViewOrientation::Back => [0.0, 0.0, -1.0],
            ViewOrientation::Top => [0.0, -1.0, 0.0],
            ViewOrientation::Bottom => [0.0, 1.0, 0.0],
            ViewOrientation::Left => [1.0, 0.0, 0.0],
            ViewOrientation::Right => [-1.0, 0.0, 0.0],
            ViewOrientation::Dimetric => {
                let d = 20.0_f32.to_radians();
                let e = 15.0_f32.to_radians();
                [-e.cos() * d.sin(), -e.sin(), e.cos() * d.cos()]
            }
        }
    }

    pub const ALL: &'static [ViewOrientation] = &[
        ViewOrientation::Iso,
        ViewOrientation::Front,
        ViewOrientation::Back,
        ViewOrientation::Top,
        ViewOrientation::Bottom,
        ViewOrientation::Left,
        ViewOrientation::Right,
        ViewOrientation::Dimetric,
    ];
}

/// Render a floating view orientation widget (top-right corner of viewport).
/// Returns the selected orientation if clicked.
pub fn render_view_cube(ctx: &egui::Context) -> Option<ViewOrientation> {
    let mut selected = None;

    egui::Area::new(egui::Id::new("view_cube"))
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 10.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::new()
                .fill(egui::Color32::from_black_alpha(150))
                .corner_radius(6.0)
                .inner_margin(egui::Margin::symmetric(8, 6))
                .show(ui, |ui| {
                    ui.vertical(|ui| {
                        for orient in ViewOrientation::ALL {
                            if ui.small_button(orient.label()).clicked() {
                                selected = Some(*orient);
                            }
                        }
                    });
                });
        });

    selected
}

/// Render a floating display style switcher (bottom-right corner of viewport).
pub fn render_display_style_switcher(ctx: &egui::Context, style: &mut DisplayStyle) {
    egui::Area::new(egui::Id::new("display_style_switcher"))
        .anchor(egui::Align2::RIGHT_BOTTOM, egui::vec2(-10.0, -30.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::new()
                .fill(egui::Color32::from_black_alpha(150))
                .corner_radius(6.0)
                .inner_margin(egui::Margin::symmetric(8, 4))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(style, DisplayStyle::Wireframe, "Wireframe");
                        ui.selectable_value(style, DisplayStyle::Shaded, "Shaded");
                        ui.selectable_value(style, DisplayStyle::ShadedWithEdges, "Shaded+Edges");
                    });
                });
        });
}
