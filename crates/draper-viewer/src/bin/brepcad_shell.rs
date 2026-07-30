// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! BRepCAD UI Shell — demonstrates the new menu bar + ribbon + panels + status bar.
//!
//! Roadmap UI Phase 0-2: Application shell with full UI structure.

use eframe::egui;
use draper_viewer::ui;

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 900.0])
            .with_title("BRepCAD — UI Shell Demo"),
        ..Default::default()
    };

    eframe::run_native(
        "BRepCAD UI Shell",
        options,
        Box::new(|_cc| Ok(Box::new(BrepCadApp::default()))),
    )
}

#[derive(Default)]
struct BrepCadApp {
    ui_state: ui::UiState,
    left_tab: ui::panels::LeftPanelTab,
    right_tab: ui::panels::RightPanelTab,
    sample_tree: Vec<ui::panels::TreeNode>,
}

impl BrepCadApp {
    fn ensure_tree(&mut self) {
        if self.sample_tree.is_empty() {
            self.sample_tree = vec![
                ui::panels::TreeNode {
                    name: "model.step".to_string(),
                    node_type: "assembly".to_string(),
                    visible: true,
                    selected: false,
                    children: vec![
                        ui::panels::TreeNode {
                            name: "Solid_1".to_string(),
                            node_type: "body".to_string(),
                            visible: true,
                            selected: false,
                            children: vec![
                                ui::panels::TreeNode {
                                    name: "Face_1".to_string(),
                                    node_type: "face".to_string(),
                                    visible: true,
                                    selected: true,
                                    children: vec![],
                                },
                                ui::panels::TreeNode {
                                    name: "Face_2".to_string(),
                                    node_type: "face".to_string(),
                                    visible: true,
                                    selected: false,
                                    children: vec![],
                                },
                                ui::panels::TreeNode {
                                    name: "Edge_1".to_string(),
                                    node_type: "edge".to_string(),
                                    visible: true,
                                    selected: false,
                                    children: vec![],
                                },
                            ],
                        },
                        ui::panels::TreeNode {
                            name: "Solid_2".to_string(),
                            node_type: "body".to_string(),
                            visible: true,
                            selected: false,
                            children: vec![],
                        },
                        ui::panels::TreeNode {
                            name: "Sketch_1".to_string(),
                            node_type: "sketch".to_string(),
                            visible: true,
                            selected: false,
                            children: vec![],
                        },
                        ui::panels::TreeNode {
                            name: "XY Plane".to_string(),
                            node_type: "plane".to_string(),
                            visible: true,
                            selected: false,
                            children: vec![],
                        },
                    ],
                },
            ];
        }
    }
}

impl eframe::App for BrepCadApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.ensure_tree();

        // Phase 1: Menu bar (21 menus)
        let _action = ui::menubar::render_menu_bar(ctx);

        // Phase 2: Ribbon bar (15 tabs)
        ui::ribbon::render_ribbon(ctx, &mut self.ui_state.active_ribbon);

        // Phase 5: Left dock panel (Browser/Model Tree)
        ui::panels::render_left_panel(ctx, &mut self.left_tab, &self.sample_tree);

        // Phase 5: Right dock panel (Properties)
        ui::panels::render_right_panel(ctx, &mut self.right_tab);

        // Phase 0: Center viewport (placeholder with grid + axis triad)
        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(10, 16, 20));

            // Grid lines
            let grid_color = egui::Color32::from_rgb(30, 40, 50);
            let step = 50.0_f32;
            let mut x = rect.left();
            while x < rect.right() {
                ui.painter().line_segment(
                    [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
                    egui::Stroke::new(1.0, grid_color));
                x += step;
            }
            let mut y = rect.top();
            while y < rect.bottom() {
                ui.painter().line_segment(
                    [egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)],
                    egui::Stroke::new(1.0, grid_color));
                y += step;
            }

            // Axis triad (bottom-left corner)
            let origin = egui::pos2(rect.left() + 60.0, rect.bottom() - 60.0);
            let len = 40.0_f32;
            ui.painter().line_segment(
                [origin, egui::pos2(origin.x + len, origin.y)],
                egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 80, 80)));
            ui.painter().text(
                egui::pos2(origin.x + len + 5.0, origin.y - 8.0),
                egui::Align2::LEFT_TOP, "X",
                egui::FontId::proportional(14.0), egui::Color32::from_rgb(255, 80, 80));
            ui.painter().line_segment(
                [origin, egui::pos2(origin.x, origin.y - len)],
                egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 255, 80)));
            ui.painter().text(
                egui::pos2(origin.x - 15.0, origin.y - len - 5.0),
                egui::Align2::LEFT_TOP, "Y",
                egui::FontId::proportional(14.0), egui::Color32::from_rgb(80, 255, 80));

            // Placeholder text
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "3D Viewport\n(load STEP file to see model)\n\nCtrl+Shift+P = Command Palette\nSpace = Marking Menu",
                egui::FontId::proportional(20.0),
                egui::Color32::from_rgb(80, 90, 100));

            // Space key toggles marking menu
            if ui.input(|i| i.key_pressed(egui::Key::Space)) {
                self.ui_state.marking_menu_visible = !self.ui_state.marking_menu_visible;
            }
        });

        // Phase 7: Command palette (Ctrl+Shift+P)
        if let Some(cmd) = ui::command_palette::render_command_palette(ctx, &mut self.ui_state.command_palette) {
            eprintln!("Command selected: {}", cmd);
        }

        // Phase 7: Marking menu (Space key)
        if let Some(action) = ui::context_menus::marking_menu(ctx, &mut self.ui_state.marking_menu_visible) {
            eprintln!("Marking menu action: {:?}", action);
        }

        // Phase 0: Status bar
        ui::statusbar::render_status_bar(ctx, &self.ui_state);
    }
}
