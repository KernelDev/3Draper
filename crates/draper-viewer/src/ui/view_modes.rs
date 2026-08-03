// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! View modes — multi-functional View Cube + display style switcher.
//! Rendered INSIDE the viewport (using viewport rect coordinates).

use eframe::egui;
use super::DisplayStyle;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewOrientation {
    Iso, Front, Back, Top, Bottom, Left, Right, Dimetric,
}

impl ViewOrientation {
    pub fn label(&self) -> &'static str {
        match self {
            ViewOrientation::Iso => "ISO", ViewOrientation::Front => "Front",
            ViewOrientation::Back => "Back", ViewOrientation::Top => "Top",
            ViewOrientation::Bottom => "Bottom", ViewOrientation::Left => "Left",
            ViewOrientation::Right => "Right", ViewOrientation::Dimetric => "Dimetric",
        }
    }
    pub fn direction(&self) -> [f32; 3] {
        match self {
            ViewOrientation::Iso => { let d=45.0_f32.to_radians(); let e=30.0_f32.to_radians(); [-e.cos()*d.sin(), -e.sin(), e.cos()*d.cos()] }
            ViewOrientation::Front => [0.0, 0.0, 1.0],
            ViewOrientation::Back => [0.0, 0.0, -1.0],
            ViewOrientation::Top => [0.0, -1.0, 0.0],
            ViewOrientation::Bottom => [0.0, 1.0, 0.0],
            ViewOrientation::Left => [1.0, 0.0, 0.0],
            ViewOrientation::Right => [-1.0, 0.0, 0.0],
            ViewOrientation::Dimetric => { let d=20.0_f32.to_radians(); let e=15.0_f32.to_radians(); [-e.cos()*d.sin(), -e.sin(), e.cos()*d.cos()] }
        }
    }
    pub const ALL: &'static [ViewOrientation] = &[
        ViewOrientation::Iso, ViewOrientation::Front, ViewOrientation::Back,
        ViewOrientation::Top, ViewOrientation::Bottom, ViewOrientation::Left,
        ViewOrientation::Right, ViewOrientation::Dimetric,
    ];
}

/// Render multi-functional View Cube inside viewport (top-right corner).
/// 3D cube with clickable faces (T/F/R) and corners (ISO).
/// Compass buttons below for Back/Front/Left/Right.
pub fn render_view_cube_in_viewport(ui: &mut egui::Ui, viewport_rect: &egui::Rect) -> Option<ViewOrientation> {
    let mut selected = None;
    let cube_size = 80.0_f32;
    let margin = 10.0_f32;
    let cube_rect = egui::Rect::from_min_size(
        egui::pos2(viewport_rect.right() - cube_size - margin, viewport_rect.top() + margin),
        egui::vec2(cube_size, cube_size + 20.0), // extra for compass buttons
    );

    // Allocate cube area + compass buttons (all mutable borrows first)
    let cube_resp = ui.allocate_rect(
        egui::Rect::from_min_size(cube_rect.min, egui::vec2(cube_size, cube_size)),
        egui::Sense::click(),
    );

    let btn_y = cube_rect.top() + cube_size + 2.0;
    let btn_w = 16.0_f32;
    let compass = [
        ("L", ViewOrientation::Left, cube_rect.left() + 4.0),
        ("B", ViewOrientation::Back, cube_rect.left() + 4.0 + btn_w),
        ("F", ViewOrientation::Front, cube_rect.left() + 4.0 + btn_w * 2.0),
        ("R", ViewOrientation::Right, cube_rect.left() + 4.0 + btn_w * 3.0),
    ];
    let mut btn_results: Vec<(egui::Rect, bool, ViewOrientation, &str)> = Vec::new();
    for (label, orient, x) in &compass {
        let br = egui::Rect::from_min_size(egui::pos2(*x, btn_y), egui::vec2(btn_w, 16.0));
        let resp = ui.allocate_rect(br, egui::Sense::click());
        btn_results.push((br, resp.hovered(), *orient, *label));
        if resp.clicked() { selected = Some(*orient); }
    }

    // Handle cube click
    if cube_resp.clicked() {
        let mp = ui.input(|i| i.pointer.latest_pos());
        if let Some(mp) = mp {
            let cx = cube_rect.left() + cube_size / 2.0;
            let cy = cube_rect.top() + cube_size / 2.0;
            let center = egui::pos2(cx, cy);
            let dist = (mp - center).length();
            if dist < 12.0 {
                selected = Some(ViewOrientation::Iso);
            } else if mp.y < cy - 5.0 {
                selected = Some(ViewOrientation::Top);
            } else if mp.x > cx {
                selected = Some(ViewOrientation::Right);
            } else {
                selected = Some(ViewOrientation::Front);
            }
        }
    }

    // Now draw everything (immutable borrow of painter)
    let painter = ui.painter();

    // Background
    let bg_rect = egui::Rect::from_min_size(cube_rect.min, egui::vec2(cube_size, cube_size));
    painter.rect_filled(bg_rect, 6.0, egui::Color32::from_black_alpha(180));

    // 3D cube via isometric projection
    let cx = bg_rect.center().x;
    let cy = bg_rect.center().y;
    let s = 22.0_f32;
    let iso_az = 45.0_f32.to_radians();
    let iso_el = 30.0_f32.to_radians();
    let project = |x: f32, y: f32, z: f32| -> egui::Pos2 {
        let x1 = x * iso_az.cos() + z * iso_az.sin();
        let z1 = -x * iso_az.sin() + z * iso_az.cos();
        let y2 = y * iso_el.cos() - z1 * iso_el.sin();
        egui::pos2(cx + x1 * s, cy - y2 * s)
    };

    let v = [
        project(-1.0, 1.0, -1.0), project(1.0, 1.0, -1.0),
        project(1.0, 1.0, 1.0), project(-1.0, 1.0, 1.0),
        project(-1.0, -1.0, -1.0), project(1.0, -1.0, -1.0),
        project(1.0, -1.0, 1.0), project(-1.0, -1.0, 1.0),
    ];

    let edge = egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(0x89, 0xb4, 0xfa));
    let text_c = egui::Color32::from_rgb(0xcd, 0xd6, 0xf4);
    let corner_c = egui::Color32::from_rgb(0xa6, 0xe3, 0xa1);

    // Top face
    painter.add(egui::Shape::convex_polygon(vec![v[0], v[1], v[2], v[3]],
        egui::Color32::from_rgb(0x45, 0x47, 0x5a), edge));
    // Front face
    painter.add(egui::Shape::convex_polygon(vec![v[3], v[2], v[6], v[7]],
        egui::Color32::from_rgb(0x31, 0x32, 0x44), edge));
    // Right face
    painter.add(egui::Shape::convex_polygon(vec![v[1], v[2], v[6], v[5]],
        egui::Color32::from_rgb(0x1e, 0x1e, 0x2e), edge));

    // Face labels
    painter.text(egui::pos2((v[0].x + v[2].x) / 2.0, (v[0].y + v[2].y) / 2.0),
        egui::Align2::CENTER_CENTER, "T", egui::FontId::proportional(10.0), text_c);
    painter.text(egui::pos2((v[3].x + v[6].x) / 2.0, (v[3].y + v[6].y) / 2.0),
        egui::Align2::CENTER_CENTER, "F", egui::FontId::proportional(10.0), text_c);
    painter.text(egui::pos2((v[1].x + v[6].x) / 2.0, (v[1].y + v[6].y) / 2.0),
        egui::Align2::CENTER_CENTER, "R", egui::FontId::proportional(10.0), text_c);

    // Corner dots (green) for ISO view
    for &c in &[v[2], v[3], v[6], v[7]] {
        painter.circle_filled(c, 3.0, corner_c);
    }

    // Compass buttons
    for (br, hovered, _orient, label) in &btn_results {
        let bg = if *hovered { egui::Color32::from_rgb(0x45, 0x47, 0x5a) } else { egui::Color32::TRANSPARENT };
        painter.rect_filled(*br, 2.0, bg);
        painter.text(br.center(), egui::Align2::CENTER_CENTER, *label,
            egui::FontId::proportional(9.0), text_c);
    }

    selected
}

/// Display style switcher inside viewport (bottom-right corner).
pub fn render_display_style_in_viewport(ui: &mut egui::Ui, viewport_rect: &egui::Rect, style: &mut DisplayStyle) {
    let margin = 10.0_f32;
    let pw = 180.0_f32;
    let ph = 28.0_f32;
    let pr = egui::Rect::from_min_size(
        egui::pos2(viewport_rect.right() - pw - margin, viewport_rect.bottom() - ph - margin),
        egui::vec2(pw, ph),
    );

    let btn_w = pw / 3.0;
    let labels = [(" Wire", DisplayStyle::Wireframe), (" Solid", DisplayStyle::Shaded), (" Both", DisplayStyle::ShadedWithEdges)];

    // Allocate buttons first (mutable borrow)
    let mut btns: Vec<(egui::Rect, bool, &str, DisplayStyle)> = Vec::new();
    for (i, (label, ds)) in labels.iter().enumerate() {
        let br = egui::Rect::from_min_size(
            egui::pos2(pr.left() + i as f32 * btn_w, pr.top()),
            egui::vec2(btn_w, ph),
        );
        let resp = ui.allocate_rect(br, egui::Sense::click());
        if resp.clicked() { *style = *ds; }
        btns.push((br, resp.hovered(), *label, *ds));
    }

    // Draw (immutable borrow)
    let painter = ui.painter();
    painter.rect_filled(pr, 6.0, egui::Color32::from_black_alpha(180));
    for (br, hovered, label, ds) in &btns {
        let active = *style == *ds;
        let bg = if *hovered { egui::Color32::from_rgb(0x45, 0x47, 0x5a) }
            else if active { egui::Color32::from_rgb(0x09, 0x47, 0x71) }
            else { egui::Color32::TRANSPARENT };
        painter.rect_filled(*br, 4.0, bg);
        let tc = if active { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) } else { egui::Color32::from_rgb(0xcd, 0xd6, 0xf4) };
        painter.text(br.center(), egui::Align2::CENTER_CENTER, *label, egui::FontId::proportional(10.0), tc);
    }
}

// Old API for backward compat (used by 3Draper Viewer, not BRepCAD)
pub fn render_view_cube(ctx: &egui::Context) -> Option<ViewOrientation> {
    let mut selected = None;
    egui::Area::new(egui::Id::new("view_cube"))
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 10.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::new().fill(egui::Color32::from_black_alpha(150))
                .corner_radius(6.0).inner_margin(egui::Margin::symmetric(8, 6))
                .show(ui, |ui| {
                    ui.vertical(|ui| {
                        for orient in ViewOrientation::ALL {
                            if ui.small_button(orient.label()).clicked() { selected = Some(*orient); }
                        }
                    });
                });
        });
    selected
}

pub fn render_display_style_switcher(ctx: &egui::Context, style: &mut DisplayStyle) {
    egui::Area::new(egui::Id::new("display_style_switcher"))
        .anchor(egui::Align2::RIGHT_BOTTOM, egui::vec2(-10.0, -30.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::new().fill(egui::Color32::from_black_alpha(150))
                .corner_radius(6.0).inner_margin(egui::Margin::symmetric(8, 4))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(style, DisplayStyle::Wireframe, "Wireframe");
                        ui.selectable_value(style, DisplayStyle::Shaded, "Shaded");
                        ui.selectable_value(style, DisplayStyle::ShadedWithEdges, "Shaded+Edges");
                    });
                });
        });
}
