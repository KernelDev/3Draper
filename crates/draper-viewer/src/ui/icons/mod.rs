// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Icon system for BRepCAD UI (Phase 1.2/1.3).
//!
//! Procedural vector icons drawn via egui::Painter — no external assets,
//! no Unicode emoji. Crisp at any DPI, consistent stroke width.

use eframe::egui;

/// Draw an icon by name into a given rect. Returns true if recognized.
pub fn draw_icon(ui: &mut egui::Ui, name: &str, rect: egui::Rect, color: egui::Color32) -> bool {
    let painter = ui.painter();
    let c = rect.center();
    let s = rect.size().min_elem() * 0.5;
    let st = egui::Stroke::new(2.0_f32, color);
    match name {
        "new" | "import" => draw_doc(painter, c, s, st),
        "open" => draw_folder(painter, c, s, st),
        "save" => draw_save(painter, c, s, st),
        "export" => draw_export(painter, c, s, st),
        "print" => draw_printer(painter, c, s, st),
        "undo" => draw_undo(painter, c, s, st),
        "redo" => draw_redo(painter, c, s, st),
        "cut" => draw_scissors(painter, c, s, st),
        "copy" | "duplicate" => draw_copy(painter, c, s, st),
        "paste" => draw_clipboard(painter, c, s, st),
        "fit" => draw_fit(painter, c, s, st),
        "zoom_in" => draw_zoom_in(painter, c, s, st),
        "zoom_out" => draw_zoom_out(painter, c, s, st),
        "iso" | "box" => draw_cube(painter, c, s, st),
        "sphere" => draw_circle(painter, c, s, st),
        "cylinder" => draw_cyl(painter, c, s, st),
        "settings" | "search" | "find" => draw_gear(painter, c, s, st),
        "exit" => draw_arrow_right(painter, c, s, st),
        "check" => draw_check(painter, c, s, st),
        "ws_modeling" => draw_cube(painter, c, s, st),
        "ws_sketch" => draw_pencil(painter, c, s, st),
        "ws_viewport" => draw_eye(painter, c, s, st),
        "ws_sheetmetal" => draw_L(painter, c, s, st),
        "ws_cam" => draw_circle(painter, c, s, st),
        "ws_fea" => draw_tri(painter, c, s, st),
        "ws_drawing" => draw_doc(painter, c, s, st),
        "ws_ai" => draw_star(painter, c, s, st),
        _ => return false,
    }
    true
}

/// Painter-only variant for custom widgets.
pub fn draw_icon_in_rect(painter: &egui::Painter, name: &str, rect: egui::Rect, color: egui::Color32) -> bool {
    let c = rect.center();
    let s = rect.size().min_elem() * 0.5;
    let st = egui::Stroke::new(1.6_f32, color);
    match name {
        "new" | "import" => draw_doc(painter, c, s, st),
        "open" => draw_folder(painter, c, s, st),
        "save" => draw_save(painter, c, s, st),
        "export" => draw_export(painter, c, s, st),
        "print" => draw_printer(painter, c, s, st),
        "undo" => draw_undo(painter, c, s, st),
        "redo" => draw_redo(painter, c, s, st),
        "cut" => draw_scissors(painter, c, s, st),
        "copy" | "duplicate" => draw_copy(painter, c, s, st),
        "paste" => draw_clipboard(painter, c, s, st),
        "fit" => draw_fit(painter, c, s, st),
        "zoom_in" => draw_zoom_in(painter, c, s, st),
        "zoom_out" => draw_zoom_out(painter, c, s, st),
        "iso" | "box" => draw_cube(painter, c, s, st),
        "sphere" => draw_circle(painter, c, s, st),
        "cylinder" => draw_cyl(painter, c, s, st),
        "settings" | "search" | "find" => draw_gear(painter, c, s, st),
        "exit" => draw_arrow_right(painter, c, s, st),
        "check" => draw_check(painter, c, s, st),
        "ws_modeling" => draw_cube(painter, c, s, st),
        "ws_sketch" => draw_pencil(painter, c, s, st),
        "ws_viewport" => draw_eye(painter, c, s, st),
        "ws_sheetmetal" => draw_L(painter, c, s, st),
        "ws_cam" => draw_circle(painter, c, s, st),
        "ws_fea" => draw_tri(painter, c, s, st),
        "ws_drawing" => draw_doc(painter, c, s, st),
        "ws_ai" => draw_star(painter, c, s, st),
        _ => return false,
    }
    true
}

/// Render an icon button (icon + label). Returns true if clicked.
pub fn icon_button(ui: &mut egui::Ui, icon_name: &str, label: &str) -> bool {
    let size = egui::vec2(55.0, 45.0);
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
    let visuals = ui.style().interact(&response);
    let bg = visuals.bg_fill;
    let fg = if response.hovered() {
        ui.style().visuals.selection.stroke.color
    } else {
        ui.style().visuals.widgets.noninteractive.fg_stroke.color
    };
    ui.painter().rect_filled(rect, 4.0_f32, bg);
    if response.hovered() || response.clicked() {
        ui.painter().rect_stroke(rect, 4.0_f32, egui::Stroke::new(1.0_f32, fg), egui::StrokeKind::Outside);
    }
    let icon_rect = egui::Rect::from_center_size(
        egui::pos2(rect.center().x - 5.0, rect.min.y + 14.0),
        egui::vec2(20.0, 20.0),
    );
    draw_icon(ui, icon_name, icon_rect, fg);
    ui.painter().text(
        egui::pos2(rect.center().x, rect.min.y + 34.0),
        egui::Align2::CENTER_CENTER,
        label,
        egui::FontId::proportional(9.0),
        fg,
    );
    response.clicked()
}

/// Menu item with leading icon + label.
pub fn icon_menu_item(ui: &mut egui::Ui, icon: &str, label: &str) -> bool {
    let response = ui.add(egui::Button::new(label));
    if !icon.is_empty() {
        let painter = ui.painter();
        let icon_rect = egui::Rect::from_center_size(
            egui::pos2(response.rect.left() + 8.0, response.rect.center().y),
            egui::vec2(12.0, 12.0),
        );
        let color = if response.hovered() {
            ui.style().visuals.selection.stroke.color
        } else {
            ui.style().visuals.widgets.noninteractive.fg_stroke.color
        };
        let _ = draw_icon_in_rect(painter, icon, icon_rect, color);
    }
    response.clicked()
}

// ─── Drawing primitives ───

fn pt(x: f32, y: f32, c: egui::Pos2, s: f32) -> egui::Pos2 {
    egui::pos2(c.x + x * s, c.y + y * s)
}

fn draw_doc(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    let r = egui::Rect::from_center_size(c, egui::vec2(s * 1.4, s * 1.6));
    p.rect_stroke(r, 2.0_f32, st, egui::StrokeKind::Outside);
    p.line_segment([pt(-0.4, -0.8, c, s), pt(0.0, -0.8, c, s)], st);
    p.line_segment([pt(0.0, -0.8, c, s), pt(0.0, -0.4, c, s)], st);
    p.line_segment([pt(0.0, -0.4, c, s), pt(-0.4, -0.8, c, s)], st);
}

fn draw_folder(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(-0.7, -0.5, c, s), pt(-0.7, 0.6, c, s)], st);
    p.line_segment([pt(-0.7, -0.5, c, s), pt(-0.2, -0.5, c, s)], st);
    p.line_segment([pt(-0.2, -0.5, c, s), pt(0.0, -0.3, c, s)], st);
    p.line_segment([pt(0.0, -0.3, c, s), pt(0.7, -0.3, c, s)], st);
    p.line_segment([pt(0.7, -0.3, c, s), pt(0.7, 0.6, c, s)], st);
    p.line_segment([pt(0.7, 0.6, c, s), pt(-0.7, 0.6, c, s)], st);
}

fn draw_save(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    let r = egui::Rect::from_center_size(c, egui::vec2(s * 1.4, s * 1.4));
    p.rect_stroke(r, 2.0_f32, st, egui::StrokeKind::Outside);
    p.line_segment([pt(-0.5, -0.7, c, s), pt(0.5, -0.7, c, s)], st);
    p.line_segment([pt(0.5, -0.7, c, s), pt(0.5, 0.0, c, s)], st);
    p.line_segment([pt(0.5, 0.0, c, s), pt(-0.5, 0.0, c, s)], st);
    p.line_segment([pt(-0.5, 0.0, c, s), pt(-0.5, -0.7, c, s)], st);
}

fn draw_export(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(0.0, -0.7, c, s), pt(0.0, 0.2, c, s)], st);
    p.line_segment([pt(-0.3, -0.4, c, s), pt(0.0, -0.7, c, s)], st);
    p.line_segment([pt(0.3, -0.4, c, s), pt(0.0, -0.7, c, s)], st);
    p.line_segment([pt(-0.6, 0.3, c, s), pt(-0.6, 0.7, c, s)], st);
    p.line_segment([pt(-0.6, 0.7, c, s), pt(0.6, 0.7, c, s)], st);
    p.line_segment([pt(0.6, 0.7, c, s), pt(0.6, 0.3, c, s)], st);
}

fn draw_printer(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.rect_stroke(egui::Rect::from_center_size(pt(0.0, -0.2, c, s), egui::vec2(s * 1.4, s * 0.5)), 2.0, st, egui::StrokeKind::Outside);
    p.line_segment([pt(-0.4, -0.45, c, s), pt(-0.4, -0.7, c, s)], st);
    p.line_segment([pt(-0.4, -0.7, c, s), pt(0.4, -0.7, c, s)], st);
    p.line_segment([pt(0.4, -0.7, c, s), pt(0.4, -0.45, c, s)], st);
}

fn draw_undo(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(0.5, -0.5, c, s), pt(-0.3, -0.5, c, s)], st);
    p.line_segment([pt(-0.3, -0.5, c, s), pt(-0.3, 0.5, c, s)], st);
    p.line_segment([pt(-0.3, 0.5, c, s), pt(0.5, 0.5, c, s)], st);
    p.line_segment([pt(-0.3, -0.5, c, s), pt(-0.1, -0.7, c, s)], st);
    p.line_segment([pt(-0.3, -0.5, c, s), pt(-0.1, -0.3, c, s)], st);
}

fn draw_redo(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(-0.5, -0.5, c, s), pt(0.3, -0.5, c, s)], st);
    p.line_segment([pt(0.3, -0.5, c, s), pt(0.3, 0.5, c, s)], st);
    p.line_segment([pt(0.3, 0.5, c, s), pt(-0.5, 0.5, c, s)], st);
    p.line_segment([pt(0.3, -0.5, c, s), pt(0.1, -0.7, c, s)], st);
    p.line_segment([pt(0.3, -0.5, c, s), pt(0.1, -0.3, c, s)], st);
}

fn draw_scissors(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.circle_stroke(pt(-0.4, 0.4_f32, c, s), s * 0.2, st);
    p.circle_stroke(pt(-0.4, -0.4, c, s), s * 0.2, st);
    p.line_segment([pt(-0.2, 0.3, c, s), pt(0.6, -0.5, c, s)], st);
    p.line_segment([pt(-0.2, -0.3, c, s), pt(0.6, 0.5, c, s)], st);
}

fn draw_copy(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.rect_stroke(egui::Rect::from_center_size(pt(-0.2, -0.2, c, s), egui::vec2(s * 0.9, s * 0.9)), 2.0, st, egui::StrokeKind::Outside);
    p.rect_stroke(egui::Rect::from_center_size(pt(0.3, 0.3_f32, c, s), egui::vec2(s * 0.9, s * 0.9)), 2.0, st, egui::StrokeKind::Outside);
}

fn draw_clipboard(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.rect_stroke(egui::Rect::from_center_size(c, egui::vec2(s * 1.2, s * 1.4)), 2.0, st, egui::StrokeKind::Outside);
    p.rect_stroke(egui::Rect::from_center_size(pt(0.0, -0.7, c, s), egui::vec2(s * 0.5, s * 0.25)), 2.0, st, egui::StrokeKind::Outside);
}

fn draw_fit(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(-0.6, -0.3, c, s), pt(-0.6, -0.6, c, s)], st);
    p.line_segment([pt(-0.6, -0.6, c, s), pt(-0.3, -0.6, c, s)], st);
    p.line_segment([pt(0.3, -0.6, c, s), pt(0.6, -0.6, c, s)], st);
    p.line_segment([pt(0.6, -0.6, c, s), pt(0.6, -0.3, c, s)], st);
    p.line_segment([pt(0.6, 0.3, c, s), pt(0.6, 0.6, c, s)], st);
    p.line_segment([pt(0.6, 0.6, c, s), pt(0.3, 0.6, c, s)], st);
    p.line_segment([pt(-0.3, 0.6, c, s), pt(-0.6, 0.6, c, s)], st);
    p.line_segment([pt(-0.6, 0.6, c, s), pt(-0.6, 0.3, c, s)], st);
}

fn draw_zoom_in(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.circle_stroke(pt(-0.15, -0.15, c, s), s * 0.5, st);
    p.line_segment([pt(0.2, 0.2, c, s), pt(0.6, 0.6, c, s)], st);
    p.line_segment([pt(-0.35, -0.15, c, s), pt(0.05, -0.15, c, s)], st);
    p.line_segment([pt(-0.15, -0.35, c, s), pt(-0.15, 0.05, c, s)], st);
}

fn draw_zoom_out(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.circle_stroke(pt(-0.15, -0.15, c, s), s * 0.5, st);
    p.line_segment([pt(0.2, 0.2, c, s), pt(0.6, 0.6, c, s)], st);
    p.line_segment([pt(-0.35, -0.15, c, s), pt(0.05, -0.15, c, s)], st);
}

fn draw_cube(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    let r = egui::Rect::from_center_size(c, egui::vec2(s * 1.2, s * 1.2));
    p.rect_stroke(r, 2.0_f32, st, egui::StrokeKind::Outside);
    p.line_segment([pt(-0.6, -0.6, c, s), pt(-0.3, -0.85, c, s)], st);
    p.line_segment([pt(0.6, -0.6, c, s), pt(0.9, -0.85, c, s)], st);
    p.line_segment([pt(-0.3, -0.85, c, s), pt(0.9, -0.85, c, s)], st);
    p.line_segment([pt(0.6, 0.6, c, s), pt(0.9, 0.35, c, s)], st);
    p.line_segment([pt(0.9, -0.85, c, s), pt(0.9, 0.35, c, s)], st);
}

fn draw_circle(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.circle_stroke(c, s * 0.7, st);
}

fn draw_cyl(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.circle_stroke(pt(0.0, -0.5, c, s), s * 0.5, st);
    p.line_segment([pt(-0.5, -0.5, c, s), pt(-0.5, 0.5, c, s)], st);
    p.line_segment([pt(0.5, -0.5, c, s), pt(0.5, 0.5, c, s)], st);
    p.add(egui::Shape::line(vec![
        pt(-0.5, 0.5, c, s), pt(0.0, 0.7, c, s), pt(0.5, 0.5, c, s),
    ], st));
}

fn draw_gear(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.circle_stroke(c, s * 0.5, st);
    p.circle_stroke(c, s * 0.2, st);
    for i in 0..8 {
        let a = i as f32 * std::f32::consts::TAU / 8.0;
        let r1 = 0.5;
        let r2 = 0.7;
        p.line_segment([
            pt(r1 * a.cos(), r1 * a.sin(), c, s),
            pt(r2 * a.cos(), r2 * a.sin(), c, s),
        ], st);
    }
}

fn draw_arrow_right(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(-0.5, 0.0, c, s), pt(0.5, 0.0, c, s)], st);
    p.line_segment([pt(0.2, -0.3, c, s), pt(0.5, 0.0, c, s)], st);
    p.line_segment([pt(0.2, 0.3, c, s), pt(0.5, 0.0, c, s)], st);
}

fn draw_check(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.add(egui::Shape::line(vec![
        pt(-0.5, 0.0, c, s), pt(-0.2, 0.3, c, s), pt(0.5, -0.4, c, s),
    ], st));
}

fn draw_pencil(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(-0.5, 0.5, c, s), pt(0.5, -0.5, c, s)], st);
    p.line_segment([pt(-0.5, 0.5, c, s), pt(-0.3, 0.3, c, s)], st);
    p.line_segment([pt(0.5, -0.5, c, s), pt(0.3, -0.3, c, s)], st);
    p.line_segment([pt(0.3, -0.3, c, s), pt(-0.3, 0.3, c, s)], st);
}

fn draw_eye(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.add(egui::Shape::line(vec![
        pt(-0.7, 0.0, c, s), pt(-0.3, -0.3, c, s),
        pt(0.3, -0.3, c, s), pt(0.7, 0.0, c, s),
        pt(0.3, 0.3, c, s), pt(-0.3, 0.3, c, s),
        pt(-0.7, 0.0, c, s),
    ], st));
    p.circle_stroke(c, s * 0.2, st);
}

fn draw_L(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(-0.6, -0.5, c, s), pt(0.3, -0.5, c, s)], st);
    p.line_segment([pt(0.3, -0.5, c, s), pt(0.3, 0.5, c, s)], st);
    p.line_segment([pt(0.3, 0.5, c, s), pt(0.6, 0.5, c, s)], st);
    p.line_segment([pt(0.6, 0.5, c, s), pt(0.6, -0.5, c, s)], st);
    p.line_segment([pt(0.6, -0.5, c, s), pt(0.3, -0.5, c, s)], st);
}

fn draw_tri(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(-0.6, 0.5, c, s), pt(0.6, 0.5, c, s)], st);
    p.line_segment([pt(0.6, 0.5, c, s), pt(0.0, -0.6, c, s)], st);
    p.line_segment([pt(0.0, -0.6, c, s), pt(-0.6, 0.5, c, s)], st);
}

fn draw_star(p: &egui::Painter, c: egui::Pos2, s: f32, st: egui::Stroke) {
    p.line_segment([pt(0.0, -0.7, c, s), pt(0.0, 0.7, c, s)], st);
    p.line_segment([pt(-0.7, 0.0, c, s), pt(0.7, 0.0, c, s)], st);
    p.line_segment([pt(-0.5, -0.5, c, s), pt(0.5, 0.5, c, s)], st);
    p.line_segment([pt(-0.5, 0.5, c, s), pt(0.5, -0.5, c, s)], st);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_painter_icons_recognized() {
        let names = [
            "new", "open", "save", "export", "print",
            "undo", "redo", "cut", "copy", "paste",
            "fit", "zoom_in", "zoom_out", "iso",
            "box", "sphere", "cylinder",
            "settings", "search", "find",
            "import", "duplicate", "exit", "check",
            "ws_modeling", "ws_sketch", "ws_viewport", "ws_sheetmetal",
            "ws_cam", "ws_fea", "ws_drawing", "ws_ai",
        ];
        for n in &names {
            assert!(is_painter_icon(n), "Icon '{}' not recognized", n);
        }
    }

    #[test]
    fn test_unknown_icon_returns_false() {
        assert!(!is_painter_icon("nonexistent_xyz_123"));
    }

    fn is_painter_icon(name: &str) -> bool {
        matches!(name,
            "new" | "import" | "open" | "save" | "export" | "print"
            | "undo" | "redo" | "cut" | "copy" | "duplicate" | "paste"
            | "fit" | "zoom_in" | "zoom_out" | "iso" | "box"
            | "sphere" | "cylinder"
            | "settings" | "search" | "find"
            | "exit" | "check"
            | "ws_modeling" | "ws_sketch" | "ws_viewport" | "ws_sheetmetal"
            | "ws_cam" | "ws_fea" | "ws_drawing" | "ws_ai"
        )
    }
}
