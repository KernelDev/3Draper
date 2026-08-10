// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Context menus and marking menu — Phase 7.

use eframe::egui;

/// Actions emitted by context menus and marking menu.
#[derive(Clone, Debug)]
pub enum ContextAction {
    Orient(String),
    DisplayStyle(String),
    ZoomSelection,
    ZoomFit,
    SelectByType(String),
    InvertSelection,
    ClearSelection,
    SectionCut,
    Measure,
    Rename,
    Delete,
    Suppress,
    EditFeature,
    ToggleVisibility,
    CreatePattern(String),
    Mirror,
    ExportSelected,
    AddConstraint(String),
    AddDimension(String),
    Trim,
    Extend,
    Split,
    Offset,
    ToggleConstruction,
    MarkingMenu(String),
}

/// Viewport right-click context menu.
pub fn viewport_context_menu(_ui: &mut egui::Ui, response: &egui::Response) -> Option<ContextAction> {
    let mut action = None;
    response.context_menu(|ui| {
        ui.menu_button("Orient", |ui| {
            for o in &["ISO", "Front", "Back", "Top", "Bottom", "Left", "Right"] {
                if ui.button(*o).clicked() {
                    action = Some(ContextAction::Orient(o.to_string()));
                    ui.close_menu();
                }
            }
        });

        ui.menu_button("Display", |ui| {
            if ui.button("Wireframe").clicked() { action = Some(ContextAction::DisplayStyle("Wireframe".into())); ui.close_menu(); }
            if ui.button("Shaded").clicked() { action = Some(ContextAction::DisplayStyle("Shaded".into())); ui.close_menu(); }
            if ui.button("Shaded + Edges").clicked() { action = Some(ContextAction::DisplayStyle("Shaded+Edges".into())); ui.close_menu(); }
        });

        ui.separator();

        if ui.button("Zoom to Selection").clicked() { action = Some(ContextAction::ZoomSelection); ui.close_menu(); }
        if ui.button("Zoom to Fit").clicked() { action = Some(ContextAction::ZoomFit); ui.close_menu(); }

        ui.separator();

        ui.menu_button("Select", |ui| {
            if ui.button("All Faces").clicked() { action = Some(ContextAction::SelectByType("face".into())); ui.close_menu(); }
            if ui.button("All Edges").clicked() { action = Some(ContextAction::SelectByType("edge".into())); ui.close_menu(); }
            if ui.button("All Vertices").clicked() { action = Some(ContextAction::SelectByType("vertex".into())); ui.close_menu(); }
            if ui.button("Invert Selection").clicked() { action = Some(ContextAction::InvertSelection); ui.close_menu(); }
            if ui.button("Clear Selection").clicked() { action = Some(ContextAction::ClearSelection); ui.close_menu(); }
        });

        ui.separator();

        if ui.button("Section Cut…").clicked() { action = Some(ContextAction::SectionCut); ui.close_menu(); }
        if ui.button("Measure…").clicked() { action = Some(ContextAction::Measure); ui.close_menu(); }
    });
    action
}

/// Browser tree right-click context menu.
pub fn browser_context_menu(_ui: &mut egui::Ui, response: &egui::Response) -> Option<ContextAction> {
    let mut action = None;
    response.context_menu(|ui| {
        if ui.button("Rename").clicked() { action = Some(ContextAction::Rename); ui.close_menu(); }
        if ui.button("Delete").clicked() { action = Some(ContextAction::Delete); ui.close_menu(); }
        if ui.button("Suppress").clicked() { action = Some(ContextAction::Suppress); ui.close_menu(); }

        ui.separator();

        if ui.button("Edit Feature").clicked() { action = Some(ContextAction::EditFeature); ui.close_menu(); }
        if ui.button("Show/Hide").clicked() { action = Some(ContextAction::ToggleVisibility); ui.close_menu(); }

        ui.separator();

        ui.menu_button("Create Derived", |ui| {
            if ui.button("Linear Pattern…").clicked() { action = Some(ContextAction::CreatePattern("linear".into())); ui.close_menu(); }
            if ui.button("Circular Pattern…").clicked() { action = Some(ContextAction::CreatePattern("circular".into())); ui.close_menu(); }
            if ui.button("Mirror…").clicked() { action = Some(ContextAction::Mirror); ui.close_menu(); }
        });

        ui.separator();

        if ui.button("Export Selected…").clicked() { action = Some(ContextAction::ExportSelected); ui.close_menu(); }
    });
    action
}

/// Sketch entity right-click context menu.
pub fn sketch_context_menu(_ui: &mut egui::Ui, response: &egui::Response) -> Option<ContextAction> {
    let mut action = None;
    response.context_menu(|ui| {
        ui.menu_button("Constrain", |ui| {
            for c in &["Coincident", "Collinear", "Concentric", "Parallel", "Perpendicular", "Tangent", "Horizontal", "Vertical", "Equal"] {
                if ui.button(*c).clicked() { action = Some(ContextAction::AddConstraint(c.to_string())); ui.close_menu(); }
            }
        });

        ui.menu_button("Dimension", |ui| {
            for d in &["Linear", "Angular", "Radial", "Diameter"] {
                if ui.button(*d).clicked() { action = Some(ContextAction::AddDimension(d.to_string())); ui.close_menu(); }
            }
        });

        ui.separator();

        if ui.button("Trim").clicked() { action = Some(ContextAction::Trim); ui.close_menu(); }
        if ui.button("Extend").clicked() { action = Some(ContextAction::Extend); ui.close_menu(); }
        if ui.button("Split").clicked() { action = Some(ContextAction::Split); ui.close_menu(); }
        if ui.button("Offset").clicked() { action = Some(ContextAction::Offset); ui.close_menu(); }

        ui.separator();

        if ui.button("Construction Geometry").clicked() { action = Some(ContextAction::ToggleConstruction); ui.close_menu(); }
    });
    action
}

/// Marking menu (radial pie menu) triggered by Space key.
pub fn marking_menu(ctx: &egui::Context, show: &mut bool) -> Option<ContextAction> {
    let mut action = None;

    if *show {
        let screen_rect = ctx.screen_rect();
        let center = screen_rect.center();
        let radius = 100.0_f32;
        let mut close = false;

        egui::Area::new(egui::Id::new("marking_menu"))
            .fixed_pos(egui::pos2(center.x - 130.0, center.y - 130.0))
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                ui.set_min_size(egui::vec2(260.0, 260.0));

                // Background circle
                ui.painter().circle_filled(center, radius + 10.0, egui::Color32::from_black_alpha(200));
                ui.painter().circle_stroke(center, radius, egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(10, 132, 255)));

                let items = [
                    ("N",  0.0_f32, -1.0_f32, "Fillet"),
                    ("NE", 0.7, -0.7, "Chamfer"),
                    ("E",  1.0,  0.0, "Extrude"),
                    ("SE", 0.7,  0.7, "Revolve"),
                    ("S",  0.0,  1.0, "Delete"),
                    ("SW",-0.7,  0.7, "Mirror"),
                    ("W", -1.0,  0.0, "Pattern"),
                    ("NW",-0.7, -0.7, "Measure"),
                ];

                for (dir, dx, dy, label) in &items {
                    let pos = egui::pos2(center.x + dx * radius * 0.6, center.y + dy * radius * 0.6);
                    let btn_rect = egui::Rect::from_center_size(pos, egui::vec2(50.0, 50.0));
                    let resp = ui.allocate_rect(btn_rect, egui::Sense::click());

                    let bg = if resp.hovered() { egui::Color32::from_rgb(10, 132, 255) } else { egui::Color32::from_rgb(40, 50, 60) };
                    ui.painter().rect_filled(btn_rect, 6.0_f32, bg);
                    ui.painter().text(pos, egui::Align2::CENTER_CENTER, format!("{}\n{}", dir, label), egui::FontId::proportional(11.0), egui::Color32::WHITE);

                    if resp.clicked() {
                        action = Some(ContextAction::MarkingMenu(label.to_string()));
                        close = true;
                    }
                }

                // Center: ESC
                let center_rect = egui::Rect::from_center_size(center, egui::vec2(50.0, 50.0));
                let center_resp = ui.allocate_rect(center_rect, egui::Sense::click());
                ui.painter().circle_filled(center, 25.0_f32, egui::Color32::from_rgb(80, 30, 30));
                ui.painter().text(center, egui::Align2::CENTER_CENTER, "ESC", egui::FontId::proportional(14.0), egui::Color32::WHITE);
                if center_resp.clicked() { close = true; }
            });

        if close || ctx.input(|i| i.key_pressed(egui::Key::Escape)) || ctx.input(|i| i.key_pressed(egui::Key::Space)) {
            *show = false;
        }
    }

    action
}
