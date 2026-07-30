// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! BRepCAD UI Shell — full UI with backend wiring.
//!
//! Roadmap UI Phase 0-9 + backend wiring:
//! - 21-menu bar dispatches real actions to dispatcher.rs → 3Draper engine
//! - 15 ribbon tabs emit MenuAction for each button
//! - Viewport renders doc.mesh triangles (wireframe/shaded/shaded+edges)
//! - View cube updates camera orientation
//! - Sketch mode: click to draw lines/circles/rectangles/arcs/points
//! - Command palette → MenuAction mapping
//! - Snapshot-based undo/redo

use eframe::egui;
use draper_viewer::ui;
use draper_viewer::ui::dispatcher::{Document, dispatch_menu_action, dispatch_dialog_action};
use draper_viewer::ui::menubar::MenuAction;
use draper_viewer::ui::sketch::{Point2D, DrawTool};

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 900.0])
            .with_title("BRepCAD — 3Draper-powered CAD/CAE/CAM"),
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
    doc: Document,
    status_msg: String,
    /// Pending action that requires showing a dialog (e.g. Insert → InsertPrimitive dialog)
    pending_dialog: Option<ui::dialogs::DialogType>,
    /// Pan offset (in screen px)
    pan_offset: egui::Vec2,
    /// Mouse position in screen coords
    last_mouse_pos: Option<egui::Pos2>,
}

impl BrepCadApp {
    fn ensure_tree(&mut self) {
        if self.sample_tree.is_empty() {
            self.sample_tree = vec![
                ui::panels::TreeNode {
                    name: format!("{}.step", self.doc.name),
                    node_type: "assembly".to_string(),
                    visible: true,
                    selected: false,
                    children: vec![
                        ui::panels::TreeNode {
                            name: format!("Solid_1 ({} tris)", self.doc.mesh.triangle_count()),
                            node_type: "body".to_string(),
                            visible: true,
                            selected: false,
                            children: vec![],
                        },
                    ],
                },
            ];
        }
    }

    /// Update the model tree based on the current document state.
    fn refresh_tree(&mut self) {
        self.sample_tree.clear();
        let body_count = self.doc.solids.len().max(1);
        let mut children = Vec::new();
        for i in 0..body_count {
            let name = if self.doc.solids.is_empty() {
                format!("Mesh_1 ({} tris)", self.doc.mesh.triangle_count())
            } else {
                format!("Solid_{} ({} tris)", i + 1, self.doc.mesh.triangle_count())
            };
            children.push(ui::panels::TreeNode {
                name,
                node_type: "body".to_string(),
                visible: true,
                selected: false,
                children: vec![],
            });
        }
        self.sample_tree.push(ui::panels::TreeNode {
            name: format!("{}.step", self.doc.name),
            node_type: "assembly".to_string(),
            visible: true,
            selected: false,
            children,
        });
    }

    /// Dispatch an action and update status.
    fn do_action(&mut self, action: &MenuAction) {
        // Some actions need to open a dialog instead
        match action {
            MenuAction::InsertBox => {
                self.pending_dialog = Some(ui::dialogs::DialogType::InsertPrimitive(ui::dialogs::PrimitiveType::Box));
                return;
            }
            MenuAction::InsertSphere => {
                self.pending_dialog = Some(ui::dialogs::DialogType::InsertPrimitive(ui::dialogs::PrimitiveType::Sphere));
                return;
            }
            MenuAction::InsertCylinder => {
                self.pending_dialog = Some(ui::dialogs::DialogType::InsertPrimitive(ui::dialogs::PrimitiveType::Cylinder));
                return;
            }
            MenuAction::InsertCone => {
                self.pending_dialog = Some(ui::dialogs::DialogType::InsertPrimitive(ui::dialogs::PrimitiveType::Cone));
                return;
            }
            MenuAction::InsertTorus => {
                self.pending_dialog = Some(ui::dialogs::DialogType::InsertPrimitive(ui::dialogs::PrimitiveType::Torus));
                return;
            }
            MenuAction::ToolsOptions => {
                self.active_dialog = ui::dialogs::DialogType::Options;
                return;
            }
            MenuAction::ToolsPlugins => {
                self.active_dialog = ui::dialogs::DialogType::Plugins;
                return;
            }
            MenuAction::ToolsPerformance => {
                self.active_dialog = ui::dialogs::DialogType::Performance;
                return;
            }
            MenuAction::SketchEnter => {
                self.in_sketch_mode = true;
                self.draw_state.reset();
                self.ui_state.active_tool = "Sketch".to_string();
                self.status_msg = "Sketch mode entered (1-5 for tools, ESC to exit)".to_string();
                return;
            }
            MenuAction::SketchExit => {
                self.in_sketch_mode = false;
                self.draw_state.reset();
                self.ui_state.active_tool = "Select".to_string();
                self.status_msg = "Sketch mode exited".to_string();
                return;
            }
            MenuAction::SketchLine => {
                if self.in_sketch_mode {
                    self.draw_state.tool = DrawTool::Line;
                    self.ui_state.active_tool = "Line".to_string();
                    self.status_msg = "Line tool selected".to_string();
                    return;
                }
            }
            MenuAction::SketchCircle => {
                if self.in_sketch_mode {
                    self.draw_state.tool = DrawTool::Circle;
                    self.ui_state.active_tool = "Circle".to_string();
                    self.status_msg = "Circle tool selected".to_string();
                    return;
                }
            }
            MenuAction::SketchArc3 => {
                if self.in_sketch_mode {
                    self.draw_state.tool = DrawTool::Arc3Point;
                    self.ui_state.active_tool = "Arc".to_string();
                    self.status_msg = "Arc tool selected".to_string();
                    return;
                }
            }
            MenuAction::SketchRectangle => {
                if self.in_sketch_mode {
                    self.draw_state.tool = DrawTool::Rectangle;
                    self.ui_state.active_tool = "Rectangle".to_string();
                    self.status_msg = "Rectangle tool selected".to_string();
                    return;
                }
            }
            MenuAction::SketchSpline => {
                if self.in_sketch_mode {
                    self.draw_state.tool = DrawTool::Spline;
                    self.ui_state.active_tool = "Spline".to_string();
                    self.status_msg = "Spline tool selected (click 5 points)".to_string();
                    return;
                }
            }
            MenuAction::SketchPoint => {
                if self.in_sketch_mode {
                    self.draw_state.tool = DrawTool::Point;
                    self.ui_state.active_tool = "Point".to_string();
                    self.status_msg = "Point tool selected".to_string();
                    return;
                }
            }
            MenuAction::HelpAbout => {
                self.active_dialog = ui::dialogs::DialogType::About;
                return;
            }
            _ => {}
        }

        let msg = dispatch_menu_action(action, &mut self.doc, &mut self.selection, &mut self.undo);
        if !msg.is_empty() {
            self.status_msg = msg;
        }
        // Sync display style
        self.ui_state.display_style = self.doc.display_style;
        self.refresh_tree();
    }

    /// Project a 3D point to screen using the document's camera (orthographic with simple rotation).
    fn project_3d(&self, p: [f32; 3], rect: &egui::Rect) -> egui::Pos2 {
        let cx = self.doc.camera_target[0];
        let cy = self.doc.camera_target[1];
        let cz = self.doc.camera_target[2];
        let az = self.doc.camera_az.to_radians();
        let el = self.doc.camera_el.to_radians();
        // Translate to target
        let dx = p[0] - cx;
        let dy = p[1] - cy;
        let dz = p[2] - cz;
        // Rotate around Z (azimuth)
        let x1 = dx * az.cos() + dy * az.sin();
        let y1 = -dx * az.sin() + dy * az.cos();
        let z1 = dz;
        // Rotate around X (elevation)
        let x2 = x1;
        let y2 = y1 * el.cos() - z1 * el.sin();
        let z2 = y1 * el.sin() + z1 * el.cos();
        // Project (orthographic: drop y, use z for depth)
        let scale = rect.size().min_elem() / (self.doc.camera_dist.max(1.0) * 2.0);
        let sx = rect.center().x + self.pan_offset.x + x2 * scale;
        let sy = rect.center().y + self.pan_offset.y - z2 * scale;
        egui::pos2(sx, sy)
    }

    /// Inverse-project a screen point to 2D sketch plane (XY).
    fn unproject_to_sketch(&self, screen: egui::Pos2, rect: &egui::Rect) -> Point2D {
        // For sketch mode, we use an orthographic top-down view (camera_az=0, camera_el=90)
        // Simple inverse of project_3d for the XY plane.
        let scale = rect.size().min_elem() / (self.doc.camera_dist.max(1.0) * 2.0);
        let x = (screen.x - rect.center().x - self.pan_offset.x) / scale;
        let y = -(screen.y - rect.center().y - self.pan_offset.y) / scale;
        // Snap to grid (Point2D uses f64)
        let grid = self.sketch.grid_size as f32;
        let sx = (x / grid).round() * grid;
        let sy = (y / grid).round() * grid;
        Point2D::new(sx as f64, sy as f64)
    }
}

impl eframe::App for BrepCadApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.ensure_tree();

        // Handle pending dialog open
        if let Some(dialog) = self.pending_dialog.take() {
            self.active_dialog = dialog;
        }

        // Phase 1: Menu bar — dispatch actions to backend
        if let Some(action) = ui::menubar::render_menu_bar(ctx) {
            self.do_action(&action);
        }

        // Phase 2: Ribbon bar — dispatch actions
        if let Some(action) = ui::ribbon::render_ribbon(ctx, &mut self.ui_state.active_ribbon) {
            self.do_action(&action);
        }

        // Phase 5: Left dock panel (Browser/Model Tree)
        ui::panels::render_left_panel(ctx, &mut self.left_tab, &self.sample_tree);

        // Phase 5: Right dock panel (Properties)
        ui::panels::render_right_panel(ctx, &mut self.right_tab);

        // Phase 0: Center viewport — render doc.mesh
        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, resp) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(10, 16, 20));

            // Track mouse for hover/preview
            let mouse_pos = resp.hover_pos();
            if let Some(pos) = mouse_pos {
                self.last_mouse_pos = Some(pos);
            }

            // Pan with middle mouse drag
            if resp.dragged_by(egui::PointerButton::Middle) {
                if let Some(delta) = resp.drag_delta().try_into().ok().or_else(|| Some(egui::Vec2::ZERO)) {
                    self.pan_offset += delta;
                }
            }

            // Zoom with scroll
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.01 {
                self.doc.camera_dist = (self.doc.camera_dist * (1.0 - scroll * 0.001)).max(10.0);
            }

            // Grid lines (if enabled)
            if self.doc.show_grid {
                let grid_color = egui::Color32::from_rgb(30, 40, 50);
                let step = 50.0_f32;
                let mut x = rect.left();
                while x < rect.right() {
                    ui.painter().line_segment(
                        [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
                        egui::Stroke::new(1.0_f32, grid_color));
                    x += step;
                }
                let mut y = rect.top();
                while y < rect.bottom() {
                    ui.painter().line_segment(
                        [egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)],
                        egui::Stroke::new(1.0_f32, grid_color));
                    y += step;
                }
            }

            // Axis triad (bottom-left corner)
            if self.doc.show_triad {
                let origin = egui::pos2(rect.left() + 60.0, rect.bottom() - 60.0);
                let len = 40.0_f32;
                ui.painter().line_segment(
                    [origin, egui::pos2(origin.x + len, origin.y)],
                    egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(255, 80, 80)));
                ui.painter().text(
                    egui::pos2(origin.x + len + 5.0, origin.y - 8.0),
                    egui::Align2::LEFT_TOP, "X",
                    egui::FontId::proportional(14.0), egui::Color32::from_rgb(255, 80, 80));
                ui.painter().line_segment(
                    [origin, egui::pos2(origin.x, origin.y - len)],
                    egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(80, 255, 80)));
                ui.painter().text(
                    egui::pos2(origin.x - 15.0, origin.y - len - 5.0),
                    egui::Align2::LEFT_TOP, "Y",
                    egui::FontId::proportional(14.0), egui::Color32::from_rgb(80, 255, 80));
                ui.painter().line_segment(
                    [origin, egui::pos2(origin.x - len * 0.7, origin.y + len * 0.7)],
                    egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(80, 180, 255)));
                ui.painter().text(
                    egui::pos2(origin.x - len * 0.7 - 15.0, origin.y + len * 0.7 + 5.0),
                    egui::Align2::LEFT_TOP, "Z",
                    egui::FontId::proportional(14.0), egui::Color32::from_rgb(80, 180, 255));
            }

            // Render the mesh
            if !self.doc.mesh.vertices.is_empty() && !self.in_sketch_mode {
                render_mesh(ui, &self.doc, rect, self.pan_offset);
            }

            // Sketch mode rendering & interaction
            if self.in_sketch_mode {
                // Update preview from hover position
                if let Some(hover) = mouse_pos {
                    let pt = self.unproject_to_sketch(hover, &rect);
                    self.draw_state.update_preview(pt);
                }

                // Handle clicks
                if resp.clicked() {
                    if let Some(click_pos) = resp.interact_pointer_pos() {
                        let pt = self.unproject_to_sketch(click_pos, &rect);
                        if let Some(_id) = self.draw_state.click(pt, &mut self.sketch) {
                            self.status_msg = format!("Entity added (total: {})", self.sketch.entity_count());
                        }
                    }
                }

                // Render sketch entities
                render_sketch(ui, &self.sketch, &self.draw_state, &rect, &self.doc, self.pan_offset);

                // Sketch mode info
                ui.painter().text(
                    egui::pos2(rect.left() + 10.0, rect.top() + 10.0),
                    egui::Align2::LEFT_TOP,
                    &format!(
                        "Sketch Mode — {}\nEntities: {} | Constraints: {} | DOF: {} | {}\nTool: {} (1-5 to switch)",
                        self.sketch.plane.label(),
                        self.sketch.entity_count(),
                        self.sketch.constraint_count(),
                        self.sketch.degrees_of_freedom(),
                        self.sketch.status(),
                        self.draw_state.tool.label(),
                    ),
                    egui::FontId::proportional(14.0),
                    egui::Color32::from_rgb(100, 200, 100));
            } else {
                // 3D viewport info
                ui.painter().text(
                    egui::pos2(rect.left() + 10.0, rect.top() + 10.0),
                    egui::Align2::LEFT_TOP,
                    &format!(
                        "{}  |  {}\nAz {:.0}° El {:.0}° D {:.0}mm  |  {}",
                        self.doc.name,
                        self.doc.stats(),
                        self.doc.camera_az,
                        self.doc.camera_el,
                        self.doc.camera_dist,
                        if self.doc.perspective { "Perspective" } else { "Orthographic" },
                    ),
                    egui::FontId::proportional(14.0),
                    egui::Color32::from_rgb(160, 170, 180));

                if self.doc.mesh.vertices.is_empty() {
                    ui.painter().text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        "3D Viewport\n(load STEP file: File → Open, or Insert → Primitives)\n\nCtrl+Shift+P = Command Palette\nS = Sketch Mode",
                        egui::FontId::proportional(20.0),
                        egui::Color32::from_rgb(80, 90, 100));
                }
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
                if ui.input(|i| i.key_pressed(egui::Key::Num1)) { self.draw_state.tool = DrawTool::Line; self.ui_state.active_tool = "Line".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num2)) { self.draw_state.tool = DrawTool::Circle; self.ui_state.active_tool = "Circle".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num3)) { self.draw_state.tool = DrawTool::Rectangle; self.ui_state.active_tool = "Rectangle".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num4)) { self.draw_state.tool = DrawTool::Point; self.ui_state.active_tool = "Point".into(); }
                if ui.input(|i| i.key_pressed(egui::Key::Num5)) { self.draw_state.tool = DrawTool::Arc3Point; self.ui_state.active_tool = "Arc".into(); }
            }

            // Camera rotate with right mouse drag
            if resp.dragged_by(egui::PointerButton::Primary) {
                let delta = resp.drag_delta();
                self.doc.camera_az += delta.x * 0.5;
                self.doc.camera_el = (self.doc.camera_el + delta.y * 0.5).max(-89.0).min(89.0);
            }
        });

        // Phase 7: Command palette (Ctrl+Shift+P)
        if let Some(cmd) = ui::command_palette::render_command_palette(ctx, &mut self.ui_state.command_palette) {
            // Map command name to MenuAction
            if let Some(action) = command_name_to_action(&cmd) {
                self.do_action(&action);
            }
        }

        // Phase 7: Marking menu (Space key)
        if ctx.input(|i| i.key_pressed(egui::Key::Space)) {
            self.ui_state.marking_menu_visible = !self.ui_state.marking_menu_visible;
        }
        if let Some(action) = ui::context_menus::marking_menu(ctx, &mut self.ui_state.marking_menu_visible) {
            let _ = action;
        }

        // Phase 3: View mode widgets (view cube + display style switcher)
        if self.doc.show_view_cube {
            if let Some(orient) = ui::view_modes::render_view_cube(ctx) {
                let (az, el) = orient.camera_angles();
                self.doc.camera_az = az;
                self.doc.camera_el = el;
                self.doc.fit_view();
                self.ui_state.camera_info[0] = az;
                self.ui_state.camera_info[1] = el;
                self.ui_state.view_orientation = orient.label().to_string();
                self.status_msg = format!("View: {}", orient.label());
            }
        }
        // Sync display style
        let mut style = self.ui_state.display_style;
        ui::view_modes::render_display_style_switcher(ctx, &mut style);
        if style != self.ui_state.display_style {
            self.ui_state.display_style = style;
            self.doc.display_style = style;
        }

        // Phase 6: Dialogs
        // Keyboard shortcuts for dialogs
        if ctx.input(|i| i.key_pressed(egui::Key::Comma) && (i.modifiers.ctrl || i.modifiers.command)) {
            self.active_dialog = ui::dialogs::DialogType::Options;
        }
        if let Some(action) = ui::dialogs::render_dialog(ctx, &mut self.active_dialog) {
            let msg = dispatch_dialog_action(&action, &mut self.doc, &mut self.undo);
            if !msg.is_empty() {
                self.status_msg = msg;
            }
            self.refresh_tree();
        }

        // Phase 8: Undo/Redo keyboard shortcuts — restore snapshot from doc
        if ctx.input(|i| i.key_pressed(egui::Key::Z) && (i.modifiers.ctrl || i.modifiers.command) && !i.modifiers.shift) {
            if let Some(desc) = self.doc.undo() {
                self.status_msg = desc;
                self.undo.undo(); // also pop from text history for UI display
                self.refresh_tree();
            } else {
                self.status_msg = "Nothing to undo".to_string();
            }
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Z) && (i.modifiers.ctrl || i.modifiers.command) && i.modifiers.shift) {
            if let Some(desc) = self.doc.redo() {
                self.status_msg = desc;
                self.undo.redo();
                self.refresh_tree();
            } else {
                self.status_msg = "Nothing to redo".to_string();
            }
        }
        // Ctrl+S = Save, Ctrl+O = Open, Ctrl+N = New, Ctrl+D = Duplicate
        if ctx.input(|i| i.key_pressed(egui::Key::S) && (i.modifiers.ctrl || i.modifiers.command)) {
            self.do_action(&MenuAction::FileSave);
        }
        if ctx.input(|i| i.key_pressed(egui::Key::O) && (i.modifiers.ctrl || i.modifiers.command)) {
            self.do_action(&MenuAction::FileOpen);
        }
        if ctx.input(|i| i.key_pressed(egui::Key::N) && (i.modifiers.ctrl || i.modifiers.command)) {
            self.do_action(&MenuAction::FileNew);
        }
        if ctx.input(|i| i.key_pressed(egui::Key::D) && (i.modifiers.ctrl || i.modifiers.command)) {
            self.do_action(&MenuAction::EditDuplicate);
        }
        // F = Fit
        if ctx.input(|i| i.key_pressed(egui::Key::F) && !i.modifiers.ctrl && !i.modifiers.command) {
            self.do_action(&MenuAction::ViewFit);
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
        self.ui_state.camera_info[0] = self.doc.camera_az;
        self.ui_state.camera_info[1] = self.doc.camera_el;
        self.ui_state.camera_info[2] = self.doc.camera_dist;
        self.ui_state.fps = ctx.input(|i| i.time).abs() as f32; // placeholder
        ui::statusbar::render_status_bar(ctx, &self.ui_state);
    }
}

/// Render the document mesh in the viewport.
fn render_mesh(ui: &mut egui::Ui, doc: &Document, rect: egui::Rect, pan_offset: egui::Vec2) {
    let mesh = &doc.mesh;
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return;
    }

    // Pre-compute screen positions for all vertices
    let mut screen_pts: Vec<egui::Pos2> = Vec::with_capacity(mesh.vertices.len());
    for v in &mesh.vertices {
        let p = [v.x as f32, v.y as f32, v.z as f32];
        // Inline projection for performance
        let cx = doc.camera_target[0];
        let cy = doc.camera_target[1];
        let cz = doc.camera_target[2];
        let az = doc.camera_az.to_radians();
        let el = doc.camera_el.to_radians();
        let dx = p[0] - cx;
        let dy = p[1] - cy;
        let dz = p[2] - cz;
        let x1 = dx * az.cos() + dy * az.sin();
        let y1 = -dx * az.sin() + dy * az.cos();
        let z1 = dz;
        let y2 = y1 * el.cos() - z1 * el.sin();
        let z2 = y1 * el.sin() + z1 * el.cos();
        let scale = rect.size().min_elem() / (doc.camera_dist.max(1.0) * 2.0);
        let sx = rect.center().x + pan_offset.x + x1 * scale;
        let sy = rect.center().y + pan_offset.y - z2 * scale;
        screen_pts.push(egui::pos2(sx, sy));
    }

    match doc.display_style {
        ui::DisplayStyle::Wireframe => {
            // Render only triangle edges
            let edge_color = egui::Color32::from_rgb(180, 200, 220);
            let stroke = egui::Stroke::new(0.5_f32, edge_color);
            for tri in &mesh.triangles {
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;
                if i0 < screen_pts.len() && i1 < screen_pts.len() && i2 < screen_pts.len() {
                    let p0 = screen_pts[i0];
                    let p1 = screen_pts[i1];
                    let p2 = screen_pts[i2];
                    // Skip degenerate triangles
                    if (p0 - p1).length_sq() < 0.1 { continue; }
                    ui.painter().line_segment([p0, p1], stroke);
                    ui.painter().line_segment([p1, p2], stroke);
                    ui.painter().line_segment([p2, p0], stroke);
                }
            }
        }
        ui::DisplayStyle::Shaded | ui::DisplayStyle::ShadedWithEdges => {
            // Render filled triangles with simple shading
            // Compute per-triangle normal and lighting
            let light_dir = [0.5_f32, -0.5, 0.7];
            let light_len = (light_dir[0] * light_dir[0] + light_dir[1] * light_dir[1] + light_dir[2] * light_dir[2]).sqrt();
            let light = [
                light_dir[0] / light_len,
                light_dir[1] / light_len,
                light_dir[2] / light_len,
            ];

            // Sort triangles back-to-front (painter's algorithm) using mean z after rotation
            let mut tri_order: Vec<(usize, f32)> = Vec::with_capacity(mesh.triangles.len());
            for (i, tri) in mesh.triangles.iter().enumerate() {
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;
                if i0 >= mesh.vertices.len() || i1 >= mesh.vertices.len() || i2 >= mesh.vertices.len() {
                    continue;
                }
                let v0 = &mesh.vertices[i0];
                let v1 = &mesh.vertices[i1];
                let v2 = &mesh.vertices[i2];
                let cx = doc.camera_target[0] as f64;
                let cy = doc.camera_target[1] as f64;
                let cz = doc.camera_target[2] as f64;
                let az = doc.camera_az.to_radians() as f64;
                let el = doc.camera_el.to_radians() as f64;
                let proj_z = |v: &draper_geometry::Point3d| {
                    let dx = v.x - cx;
                    let dy = v.y - cy;
                    let dz = v.z - cz;
                    let y1 = -dx * az.sin() + dy * az.cos();
                    let z1 = dz;
                    y1 * el.sin() + z1 * el.cos()
                };
                let z_mean = (proj_z(v0) + proj_z(v1) + proj_z(v2)) / 3.0;
                tri_order.push((i, z_mean as f32));
            }
            // Sort: larger z (farther) first
            tri_order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

            for (i, _z) in &tri_order {
                let tri = &mesh.triangles[*i];
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;
                if i0 >= screen_pts.len() || i1 >= screen_pts.len() || i2 >= screen_pts.len() {
                    continue;
                }
                let p0 = screen_pts[i0];
                let p1 = screen_pts[i1];
                let p2 = screen_pts[i2];

                // Compute face normal in world space
                let v0 = mesh.vertices[i0];
                let v1 = mesh.vertices[i1];
                let v2 = mesh.vertices[i2];
                let ax = v1.x - v0.x; let ay = v1.y - v0.y; let az_ = v1.z - v0.z;
                let bx = v2.x - v0.x; let by = v2.y - v0.y; let bz = v2.z - v0.z;
                let nx = ay * bz - az_ * by;
                let ny = az_ * bx - ax * bz;
                let nz = ax * by - ay * bx;
                let nlen = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-9);
                let nx = nx / nlen;
                let ny = ny / nlen;
                let nz = nz / nlen;

                // Lambert lighting
                let dot = (nx * light[0] as f64 + ny * light[1] as f64 + nz * light[2] as f64).abs();
                let intensity = (0.3 + 0.7 * dot).min(1.0) as f32;
                let r = (90.0 * intensity) as u8;
                let g = (130.0 * intensity) as u8;
                let b = (180.0 * intensity) as u8;
                let color = egui::Color32::from_rgb(r, g, b);

                let points = [p0, p1, p2];
                ui.painter().add(egui::Shape::convex_polygon(
                    points.to_vec(),
                    color,
                    egui::Stroke::NONE,
                ));

                if doc.display_style == ui::DisplayStyle::ShadedWithEdges && doc.show_edges {
                    let edge_color = egui::Color32::from_rgb(40, 50, 60);
                    let stroke = egui::Stroke::new(0.5_f32, edge_color);
                    ui.painter().line_segment([p0, p1], stroke);
                    ui.painter().line_segment([p1, p2], stroke);
                    ui.painter().line_segment([p2, p0], stroke);
                }
            }
        }
    }
}

/// Render sketch entities in the viewport.
fn render_sketch(
    ui: &mut egui::Ui,
    sketch: &ui::sketch::Sketch,
    draw_state: &ui::sketch::DrawState,
    rect: &egui::Rect,
    doc: &Document,
    pan_offset: egui::Vec2,
) {
    use ui::sketch::SketchEntity;
    let entity_color = egui::Color32::from_rgb(255, 220, 100);
    let preview_color = egui::Color32::from_rgb(255, 200, 50);
    let point_color = egui::Color32::from_rgb(255, 100, 100);
    let stroke = egui::Stroke::new(1.5_f32, entity_color);
    let preview_stroke = egui::Stroke::new(1.5_f32, preview_color);

    // Project a 2D sketch point to screen (using current camera, treating sketch on XY plane)
    let project_sketch_pt = |p: &Point2D| -> egui::Pos2 {
        let pt = [p.x as f32, p.y as f32, 0.0];
        let cx = doc.camera_target[0];
        let cy = doc.camera_target[1];
        let cz = doc.camera_target[2];
        let az = doc.camera_az.to_radians();
        let el = doc.camera_el.to_radians();
        let dx = pt[0] - cx;
        let dy = pt[1] - cy;
        let dz = pt[2] - cz;
        let x1 = dx * az.cos() + dy * az.sin();
        let y1 = -dx * az.sin() + dy * az.cos();
        let z1 = dz;
        let y2 = y1 * el.cos() - z1 * el.sin();
        let z2 = y1 * el.sin() + z1 * el.cos();
        let scale = rect.size().min_elem() / (doc.camera_dist.max(1.0) * 2.0);
        let sx = rect.center().x + pan_offset.x + x1 * scale;
        let sy = rect.center().y + pan_offset.y - z2 * scale;
        egui::pos2(sx, sy)
    };

    for (_id, entity) in &sketch.entities {
        match entity {
            SketchEntity::Line { p1, p2, .. } => {
                let s1 = project_sketch_pt(p1);
                let s2 = project_sketch_pt(p2);
                ui.painter().line_segment([s1, s2], stroke);
                // Endpoints
                ui.painter().circle_filled(s1, 3.0, point_color);
                ui.painter().circle_filled(s2, 3.0, point_color);
            }
            SketchEntity::Circle { center, radius, .. } => {
                let sc = project_sketch_pt(center);
                // Convert radius from world to screen
                let sp = project_sketch_pt(&Point2D::new(center.x + radius, center.y));
                let r = (sp - sc).length();
                ui.painter().circle_stroke(sc, r, stroke);
                ui.painter().circle_filled(sc, 3.0, point_color);
            }
            SketchEntity::Arc { center, p1, p2, .. } => {
                let sc = project_sketch_pt(center);
                let sp1 = project_sketch_pt(p1);
                let sp2 = project_sketch_pt(p2);
                let r = (sp1 - sc).length();
                ui.painter().circle_stroke(sc, r, stroke);
                ui.painter().circle_filled(sp1, 3.0, point_color);
                ui.painter().circle_filled(sp2, 3.0, point_color);
            }
            SketchEntity::Rectangle { p1, p2, .. } => {
                let s1 = project_sketch_pt(p1);
                let s2 = project_sketch_pt(p2);
                let p_a = s1;
                let p_b = egui::pos2(s2.x, s1.y);
                let p_c = s2;
                let p_d = egui::pos2(s1.x, s2.y);
                ui.painter().line_segment([p_a, p_b], stroke);
                ui.painter().line_segment([p_b, p_c], stroke);
                ui.painter().line_segment([p_c, p_d], stroke);
                ui.painter().line_segment([p_d, p_a], stroke);
            }
            SketchEntity::Spline { points, .. } => {
                for window in points.windows(2) {
                    let s1 = project_sketch_pt(&window[0]);
                    let s2 = project_sketch_pt(&window[1]);
                    ui.painter().line_segment([s1, s2], stroke);
                }
            }
            SketchEntity::Point { p, .. } => {
                let s = project_sketch_pt(p);
                ui.painter().circle_filled(s, 4.0, point_color);
            }
        }
    }

    // Render preview entity
    if let Some(preview) = &draw_state.preview {
        match preview {
            SketchEntity::Line { p1, p2, .. } => {
                let s1 = project_sketch_pt(p1);
                let s2 = project_sketch_pt(p2);
                ui.painter().line_segment([s1, s2], preview_stroke);
            }
            SketchEntity::Circle { center, radius, .. } => {
                let sc = project_sketch_pt(center);
                let sp = project_sketch_pt(&Point2D::new(center.x + radius, center.y));
                let r = (sp - sc).length();
                ui.painter().circle_stroke(sc, r, preview_stroke);
            }
            SketchEntity::Rectangle { p1, p2, .. } => {
                let s1 = project_sketch_pt(p1);
                let s2 = project_sketch_pt(p2);
                let p_a = s1;
                let p_b = egui::pos2(s2.x, s1.y);
                let p_c = s2;
                let p_d = egui::pos2(s1.x, s2.y);
                ui.painter().line_segment([p_a, p_b], preview_stroke);
                ui.painter().line_segment([p_b, p_c], preview_stroke);
                ui.painter().line_segment([p_c, p_d], preview_stroke);
                ui.painter().line_segment([p_d, p_a], preview_stroke);
            }
            _ => {}
        }
    }

    // Draw current click points
    for p in &draw_state.points {
        let s = project_sketch_pt(p);
        ui.painter().circle_filled(s, 4.0, egui::Color32::from_rgb(255, 255, 0));
    }
}

/// Map a command palette name to a MenuAction.
fn command_name_to_action(name: &str) -> Option<MenuAction> {
    match name {
        "New" => Some(MenuAction::FileNew),
        "Open…" => Some(MenuAction::FileOpen),
        "Save" => Some(MenuAction::FileSave),
        "Export STEP" => Some(MenuAction::FileExportStep),
        "Export STL" => Some(MenuAction::FileExportStl),
        "Import STEP" => Some(MenuAction::FileImportStep),
        "Undo" => Some(MenuAction::EditUndo),
        "Redo" => Some(MenuAction::EditRedo),
        "Cut" => Some(MenuAction::EditCut),
        "Copy" => Some(MenuAction::EditCopy),
        "Paste" => Some(MenuAction::EditPaste),
        "Duplicate" => Some(MenuAction::EditDuplicate),
        "Fit to View" => Some(MenuAction::ViewFit),
        "ISO View" => Some(MenuAction::ViewIso),
        "Front View" => Some(MenuAction::ViewFront),
        "Top View" => Some(MenuAction::ViewTop),
        "Right View" => Some(MenuAction::ViewRight),
        "Wireframe" => Some(MenuAction::ViewWireframe),
        "Shaded" => Some(MenuAction::ViewShaded),
        "Shaded + Edges" => Some(MenuAction::ViewShadedEdges),
        "Insert Box" => Some(MenuAction::InsertBox),
        "Insert Sphere" => Some(MenuAction::InsertSphere),
        "Insert Cylinder" => Some(MenuAction::InsertCylinder),
        "Insert Cone" => Some(MenuAction::InsertCone),
        "Insert Torus" => Some(MenuAction::InsertTorus),
        "Insert Sketch" => Some(MenuAction::SketchEnter),
        "Boolean Union" => Some(MenuAction::ModifyUnion),
        "Boolean Subtract" => Some(MenuAction::ModifySubtract),
        "Boolean Intersect" => Some(MenuAction::ModifyIntersect),
        "Fillet" => Some(MenuAction::ModifyFillet),
        "Chamfer" => Some(MenuAction::ModifyChamfer),
        "Move" => Some(MenuAction::ModifyMove),
        "Rotate" => Some(MenuAction::ModifyRotate),
        "Scale" => Some(MenuAction::ModifyScale),
        "Sketch Mode" => Some(MenuAction::SketchEnter),
        "Line" => Some(MenuAction::SketchLine),
        "Circle" => Some(MenuAction::SketchCircle),
        "Arc" => Some(MenuAction::SketchArc3),
        "Rectangle" => Some(MenuAction::SketchRectangle),
        "Dimension" => Some(MenuAction::SketchDimLinear),
        "Options…" => Some(MenuAction::ToolsOptions),
        "Customize…" => Some(MenuAction::ToolsCustomize),
        "Plugins Manager…" => Some(MenuAction::ToolsPlugins),
        "Scripting Console" => Some(MenuAction::ToolsScriptingConsole),
        "Performance Monitor" => Some(MenuAction::ToolsPerformance),
        "Measure Distance" => Some(MenuAction::MeasureDistance),
        "Measure Angle" => Some(MenuAction::MeasureAngle),
        "Measure Area" => Some(MenuAction::MeasureArea),
        "Measure Volume" => Some(MenuAction::MeasureVolume),
        "Heal: Stitch" => Some(MenuAction::HealStitch),
        "Heal: Gap Fill" => Some(MenuAction::HealGapFill),
        "Heal: Fix Orientation" => Some(MenuAction::HealFixOrientation),
        "Watertight Check" => Some(MenuAction::AnalysisWatertight),
        "Manifold Check" => Some(MenuAction::AnalysisManifold),
        _ => None,
    }
}
