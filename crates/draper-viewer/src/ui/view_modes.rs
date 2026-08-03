// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! ViewCube — Autodesk-style 3D orientation controller.
//! White cube with blue hover, full face labels (TOP/FRONT/RIGHT),
//! compass disc with N/E/S/W, Home button, drag-to-rotate.

use eframe::egui;
use super::DisplayStyle;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewOrientation {
    Iso, Front, Back, Top, Bottom, Left, Right, Dimetric,
}

impl ViewOrientation {
    pub fn label(&self) -> &'static str {
        match self {
            ViewOrientation::Iso => "ISO", ViewOrientation::Front => "FRONT",
            ViewOrientation::Back => "BACK", ViewOrientation::Top => "TOP",
            ViewOrientation::Bottom => "BOTTOM", ViewOrientation::Left => "LEFT",
            ViewOrientation::Right => "RIGHT", ViewOrientation::Dimetric => "DIM",
        }
    }
    pub fn direction(&self) -> [f32; 3] {
        match self {
            ViewOrientation::Iso => { let d=45.0_f32.to_radians(); let e=35.264_f32.to_radians(); [-e.cos()*d.sin(), -e.sin(), e.cos()*d.cos()] }
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

#[derive(Clone, Debug, Default)]
pub struct ViewCubeState {
    pub azimuth: f32,
    pub elevation: f32,
    pub dragging: bool,
}

impl ViewCubeState {
    pub fn new() -> Self {
        Self { azimuth: 45.0, elevation: 35.264, dragging: false }
    }
}

pub fn render_view_cube_in_viewport(
    ui: &mut egui::Ui,
    viewport_rect: &egui::Rect,
    state: &mut ViewCubeState,
) -> Option<ViewOrientation> {
    let mut selected = None;

    // Layout
    let ring_r = 52.0_f32;
    let margin = 12.0_f32;
    let cube_half = 22.0_f32;
    let center = egui::pos2(
        viewport_rect.right() - ring_r - margin,
        viewport_rect.top() + ring_r + margin,
    );

    // Allocate interaction areas
    let cube_rect = egui::Rect::from_center_size(center, egui::vec2(cube_half * 3.0, cube_half * 3.0));
    let cube_resp = ui.allocate_rect(cube_rect, egui::Sense::click_and_drag());
    let ring_rect = egui::Rect::from_center_size(center, egui::vec2(ring_r * 2.2, ring_r * 2.2));
    let ring_resp = ui.allocate_rect(ring_rect, egui::Sense::click_and_drag());
    let home_rect = egui::Rect::from_center_size(
        egui::pos2(center.x, center.y + ring_r + 14.0),
        egui::vec2(55.0, 18.0),
    );
    let home_resp = ui.allocate_rect(home_rect, egui::Sense::click());

    // Compass buttons
    let compass_r = ring_r - 4.0;
    let compass = [
        ("N", ViewOrientation::Back,  0.0_f32),
        ("E", ViewOrientation::Right, 90.0_f32),
        ("S", ViewOrientation::Front, 180.0_f32),
        ("W", ViewOrientation::Left,  270.0_f32),
    ];
    let mut compass_results: Vec<(egui::Rect, bool, ViewOrientation, &str)> = Vec::new();
    for (label, orient, angle) in &compass {
        let rad = angle.to_radians();
        let px = center.x + rad.sin() * compass_r;
        let py = center.y - rad.cos() * compass_r;
        let br = egui::Rect::from_center_size(egui::pos2(px, py), egui::vec2(14.0, 14.0));
        let resp = ui.allocate_rect(br, egui::Sense::click());
        compass_results.push((br, resp.hovered(), *orient, *label));
        if resp.clicked() { selected = Some(*orient); }
    }

    // Handle drag
    let drag_resp = if cube_resp.dragged_by(egui::PointerButton::Primary) { Some(&cube_resp) }
        else if ring_resp.dragged_by(egui::PointerButton::Primary) { Some(&ring_resp) }
        else { None };
    if let Some(dr) = drag_resp {
        let delta = dr.drag_delta();
        if delta.length_sq() > 0.5 {
            state.azimuth += delta.x * 0.8;
            state.elevation = (state.elevation - delta.y * 0.8).max(-89.0).min(89.0);
            state.dragging = true;
        }
    }

    // Mouse position for hover
    let mouse_pos = ui.input(|i| i.pointer.latest_pos());

    // ─── DRAWING ───
    let painter = ui.painter();

    // Background panel (dark, rounded)
    let bg_rect = egui::Rect::from_center_size(center, egui::vec2(ring_r * 2.0 + 12.0, ring_r * 2.0 + 12.0));
    painter.rect_filled(bg_rect, ring_r + 6.0, egui::Color32::from_black_alpha(170));

    // Compass disc (grey gradient effect — just solid grey)
    painter.circle_filled(center, ring_r, egui::Color32::from_rgb(0x3a, 0x3a, 0x40));
    painter.circle_stroke(center, ring_r, egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
    painter.circle_stroke(center, ring_r - 10.0, egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(0x45, 0x47, 0x5a)));

    // Compass tick marks
    for i in 0..4 {
        let angle = (i as f32 * 90.0).to_radians();
        let x1 = center.x + angle.sin() * (ring_r - 10.0);
        let y1 = center.y - angle.cos() * (ring_r - 10.0);
        let x2 = center.x + angle.sin() * (ring_r - 2.0);
        let y2 = center.y - angle.cos() * (ring_r - 2.0);
        painter.line_segment([egui::pos2(x1, y1), egui::pos2(x2, y2)],
            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
    }

    // Compass buttons (N/E/S/W)
    for (br, hovered, _orient, label) in &compass_results {
        let bg = if *hovered { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
            else { egui::Color32::from_rgb(0x2a, 0x2a, 0x35) };
        painter.rect_filled(*br, 3.0, bg);
        let tc = if *hovered { egui::Color32::from_rgb(0x1e, 0x1e, 0x2e) }
            else { egui::Color32::WHITE };
        painter.text(br.center(), egui::Align2::CENTER_CENTER, *label,
            egui::FontId::proportional(8.0), tc);
    }

    // ─── 3D CUBE ───
    let az = state.azimuth.to_radians();
    let el = state.elevation.to_radians();

    let project = |x: f32, y: f32, z: f32| -> egui::Pos2 {
        let x1 = x * az.cos() + z * az.sin();
        let z1 = -x * az.sin() + z * az.cos();
        let y2 = y * el.cos() - z1 * el.sin();
        egui::pos2(center.x + x1 * cube_half, center.y - y2 * cube_half)
    };

    // 8 vertices
    let v = [
        project(-1.0,  1.0, -1.0), project( 1.0,  1.0, -1.0),
        project( 1.0,  1.0,  1.0), project(-1.0,  1.0,  1.0),
        project(-1.0, -1.0, -1.0), project( 1.0, -1.0, -1.0),
        project( 1.0, -1.0,  1.0), project(-1.0, -1.0,  1.0),
    ];

    // Colors — WHITE cube (like reference)
    let c_face    = egui::Color32::from_rgb(0xe8, 0xe8, 0xec); // white-ish
    let c_top     = egui::Color32::from_rgb(0xf0, 0xf0, 0xf4); // brighter white for top
    let c_hidden  = egui::Color32::from_rgb(0x2a, 0x2a, 0x35); // dark for hidden
    let c_edge    = egui::Color32::from_rgb(0x4a, 0x4a, 0x55); // dark grey edges
    let c_hover   = egui::Color32::from_rgb(0x6c, 0xb4, 0xe8); // light blue hover
    let c_hover_a = egui::Color32::from_rgba_premultiplied(0x6c, 0xb4, 0xe8, 180); // semi-transparent blue
    let c_corner  = egui::Color32::from_rgb(0xa6, 0xe3, 0xa1); // green
    let c_text    = egui::Color32::from_rgb(0x2a, 0x2a, 0x35); // dark text on white
    let c_text_h  = egui::Color32::WHITE; // white text on blue hover
    let edge_stroke = egui::Stroke::new(1.0_f32, c_edge);

    // Hidden faces (drawn first)
    let hidden = [
        ([4, 5, 6, 7], "BOTTOM"),
        ([0, 1, 5, 4], "BACK"),
        ([0, 3, 7, 4], "LEFT"),
    ];
    for (idx, _label) in &hidden {
        let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
        painter.add(egui::Shape::convex_polygon(pts, c_hidden,
            egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(0x31, 0x32, 0x44))));
    }

    // Visible faces with hover detection
    let visible = [
        ([0, 1, 2, 3], "TOP",   c_top,   ViewOrientation::Top,   0usize),
        ([3, 2, 6, 7], "FRONT", c_face,  ViewOrientation::Front, 1),
        ([2, 1, 5, 6], "RIGHT", c_face,  ViewOrientation::Right, 2),
    ];

    let mut hovered_face: Option<usize> = None;
    if let Some(mp) = mouse_pos {
        if cube_rect.contains(mp) && !state.dragging {
            for (_, _, _, _, fi) in visible.iter().rev() {
                let face = &visible[*fi];
                let pts: Vec<egui::Pos2> = face.0.iter().map(|&i| v[i]).collect();
                if point_in_polygon(mp, &pts) {
                    hovered_face = Some(face.4);
                    break;
                }
            }
        }
    }

    // Draw visible faces
    for (idx, label, color, orient, fi) in &visible {
        let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
        let fill = if hovered_face == Some(*fi) { c_hover_a } else { *color };
        painter.add(egui::Shape::convex_polygon(pts.clone(), fill, edge_stroke));
        // Label
        let cx = pts.iter().map(|p| p.x).sum::<f32>() / pts.len() as f32;
        let cy = pts.iter().map(|p| p.y).sum::<f32>() / pts.len() as f32;
        let tc = if hovered_face == Some(*fi) { c_text_h } else { c_text };
        painter.text(egui::pos2(cx, cy), egui::Align2::CENTER_CENTER,
            *label, egui::FontId::proportional(8.0), tc);
    }

    // Draw edges
    let edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)];
    for &(a, b) in &edges {
        painter.line_segment([v[a], v[b]], edge_stroke);
    }

    // Corner dots (green) on visible corners
    let vis_corners = [2, 3, 6, 7];
    for &ci in &vis_corners {
        let is_hover = mouse_pos.map(|mp| (mp - v[ci]).length() < 7.0).unwrap_or(false);
        let r = if is_hover { 4.5 } else { 3.0 };
        let color = if is_hover { egui::Color32::WHITE } else { c_corner };
        painter.circle_filled(v[ci], r, color);
        if is_hover {
            painter.circle_stroke(v[ci], 6.0, egui::Stroke::new(1.5_f32, egui::Color32::WHITE));
        }
    }

    // Handle clicks
    if cube_resp.clicked() && !state.dragging {
        if let Some(mp) = mouse_pos {
            // Check corners first
            for &ci in &vis_corners {
                if (mp - v[ci]).length() < 8.0 {
                    selected = Some(ViewOrientation::Iso);
                    state.azimuth = 45.0;
                    state.elevation = 35.264;
                    return selected;
                }
            }
            // Check faces
            for (_, _, _, orient, fi) in &visible {
                if hovered_face == Some(*fi) {
                    selected = Some(*orient);
                    break;
                }
            }
        }
    }

    // Home button
    let home_bg = if home_resp.hovered() { egui::Color32::from_rgb(0x45, 0x47, 0x5a) }
        else { egui::Color32::from_rgb(0x2a, 0x2a, 0x35) };
    painter.rect_filled(home_rect, 4.0, home_bg);
    let home_tc = if home_resp.hovered() { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
        else { egui::Color32::from_rgb(0xc8, 0xc8, 0xd0) };
    // House icon (simple triangle + square)
    let hx = home_rect.left() + 10.0;
    let hy = home_rect.center().y;
    painter.text(egui::pos2(hx, hy), egui::Align2::CENTER_CENTER, "\u{2302}",
        egui::FontId::proportional(12.0), home_tc);
    painter.text(egui::pos2(hx + 14.0, hy), egui::Align2::LEFT_CENTER, "Home",
        egui::FontId::proportional(9.0), home_tc);
    if home_resp.clicked() {
        selected = Some(ViewOrientation::Iso);
        state.azimuth = 45.0;
        state.elevation = 35.264;
    }

    // Reset dragging
    if !cube_resp.dragged() && !ring_resp.dragged() {
        state.dragging = false;
    }

    selected
}

fn point_in_polygon(p: egui::Pos2, polygon: &[egui::Pos2]) -> bool {
    let n = polygon.len();
    if n < 3 { return false; }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = polygon[i];
        let pj = polygon[j];
        if ((pi.y > p.y) != (pj.y > p.y))
            && (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y + 1e-10) + pi.x) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

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
    let mut btns: Vec<(egui::Rect, bool, &str, DisplayStyle)> = Vec::new();
    for (i, (label, ds)) in labels.iter().enumerate() {
        let br = egui::Rect::from_min_size(egui::pos2(pr.left() + i as f32 * btn_w, pr.top()), egui::vec2(btn_w, ph));
        let resp = ui.allocate_rect(br, egui::Sense::click());
        if resp.clicked() { *style = *ds; }
        btns.push((br, resp.hovered(), *label, *ds));
    }
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
