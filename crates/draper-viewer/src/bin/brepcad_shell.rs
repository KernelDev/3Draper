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
    active_dialog: ui::dialogs::DialogType,
    // Phase 8: Core engine
    selection: ui::core_engine::SelectionManager,
    undo: ui::core_engine::UndoManager,
    params: ui::core_engine::ParameterTable,
    materials: ui::core_engine::MaterialLibrary,
    layers: ui::core_engine::LayerManager,
    // Phase 4: Sketch engine
    sketch: ui::sketch::Sketch,
    draw_state: ui::sketch::DrawState,
    in_sketch_mode: bool,
    // Phase 8+: Document + dispatcher
    doc: ui::dispatcher::Document,
    status_msg: String,
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

        // Phase 1: Menu bar (21 menus) — dispatch actions to backend
        if let Some(action) = ui::menubar::render_menu_bar(ctx) {
            self.status_msg = ui::dispatcher::dispatch_menu_action(
                &action, &mut self.doc, &mut self.selection, &mut self.undo,
            );
            eprintln!("Menu: {}", self.status_msg);
        }

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
            if self.in_sketch_mode {
                // Phase 4: Sketch mode canvas
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    &format!(
                        "Sketch Mode — {}\nEntities: {} | Constraints: {} | DOF: {} | {}\nTool: {}\nClick to draw, ESC to exit",
                        self.sketch.plane.label(),
                        self.sketch.entity_count(),
                        self.sketch.constraint_count(),
                        self.sketch.degrees_of_freedom(),
                        self.sketch.status(),
                        self.draw_state.tool.label(),
                    ),
                    egui::FontId::proportional(18.0),
                    egui::Color32::from_rgb(100, 200, 100));
            } else {
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "3D Viewport\n(load STEP file to see model)\n\nCtrl+Shift+P = Command Palette\nSpace = Marking Menu\nS = Toggle Sketch Mode",
                    egui::FontId::proportional(20.0),
                    egui::Color32::from_rgb(80, 90, 100));
            }

            // Space key toggles marking menu
            if ui.input(|i| i.key_pressed(egui::Key::Space)) {
                self.ui_state.marking_menu_visible = !self.ui_state.marking_menu_visible;
            }

            // S key toggles sketch mode
            if ui.input(|i| i.key_pressed(egui::Key::S) && !i.modifiers.ctrl && !i.modifiers.command) {
                self.in_sketch_mode = !self.in_sketch_mode;
                self.draw_state.reset();
                self.ui_state.active_tool = if self.in_sketch_mode {
                    "Sketch".to_string()
                } else {
                    "Select".to_string()
                };
            }

            // ESC exits sketch mode
            if self.in_sketch_mode && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                self.in_sketch_mode = false;
                self.draw_state.reset();
                self.ui_state.active_tool = "Select".to_string();
            }

            // Number keys select draw tools in sketch mode
            if self.in_sketch_mode {
                if ui.input(|i| i.key_pressed(egui::Key::Num1)) { self.draw_state.tool = ui::sketch::DrawTool::Line; self.ui_state.active_tool = "Line".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num2)) { self.draw_state.tool = ui::sketch::DrawTool::Circle; self.ui_state.active_tool = "Circle".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num3)) { self.draw_state.tool = ui::sketch::DrawTool::Rectangle; self.ui_state.active_tool = "Rectangle".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num4)) { self.draw_state.tool = ui::sketch::DrawTool::Point; self.ui_state.active_tool = "Point".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num5)) { self.draw_state.tool = ui::sketch::DrawTool::Arc3Point; self.ui_state.active_tool = "Arc".into(); }
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

        // Phase 3: View mode widgets (view cube + display style switcher)
        if let Some(orient) = ui::view_modes::render_view_cube(ctx) {
            let (az, el) = orient.camera_angles();
            self.ui_state.camera_info[0] = az;
            self.ui_state.camera_info[1] = el;
            self.ui_state.view_orientation = orient.label().to_string();
        }
        ui::view_modes::render_display_style_switcher(ctx, &mut self.ui_state.display_style);

        // Phase 6: Dialogs
        // Keyboard shortcuts for dialogs
        if ctx.input(|i| i.key_pressed(egui::Key::Comma) && (i.modifiers.ctrl || i.modifiers.command)) {
            self.active_dialog = ui::dialogs::DialogType::Options;
        }
        if let Some(action) = ui::dialogs::render_dialog(ctx, &mut self.active_dialog) {
            self.status_msg = ui::dispatcher::dispatch_dialog_action(
                &action, &mut self.doc, &mut self.undo,
            );
            eprintln!("Dialog: {}", self.status_msg);
        }

        // Phase 8: Undo/Redo keyboard shortcuts
        if ctx.input(|i| i.key_pressed(egui::Key::Z) && (i.modifiers.ctrl || i.modifiers.command) && !i.modifiers.shift) {
            if let Some(desc) = self.undo.undo() { eprintln!("{}", desc); }
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Z) && (i.modifiers.ctrl || i.modifiers.command) && i.modifiers.shift) {
            if let Some(desc) = self.undo.redo() { eprintln!("{}", desc); }
        }

        // Update selection count in UI state
        self.ui_state.selection_count = self.selection.count();

        // Show status message in status bar area
        if !self.status_msg.is_empty() {
            egui::Area::new(egui::Id::new("status_toast"))
                .anchor(egui::Align2::CENTER_BOTTOM, [0.0, -30.0])
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    egui::Frame::new()
                        .fill(egui::Color32::from_black_alpha(200))
                        .corner_radius(6.0)
                        .inner_margin(egui::Margin::symmetric(12, 6))
                        .show(ui, |ui| {
                            ui.label(&self.status_msg);
                        });
                });
        }

        // Phase 0: Status bar
        ui::statusbar::render_status_bar(ctx, &self.ui_state);
    }
}
