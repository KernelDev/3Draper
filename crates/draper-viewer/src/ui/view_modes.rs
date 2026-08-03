// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! ViewCube — Autodesk-style 3D orientation controller.
//! White cube, blue hover, compass disc, Home, drag-to-rotate.

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
        Self { azimuth: 35.0, elevation: 25.0, dragging: false }
    }
}

pub fn render_view_cube_in_viewport(
    ui: &mut egui::Ui,
    viewport_rect: &egui::Rect,
    state: &mut ViewCubeState,
) -> Option<ViewOrientation> {
    let mut selected = None;

    // Layout
    let ring_r = 55.0_f32;
    let margin = 12.0_f32;
    let cube_half = 26.0_f32; // bigger cube
    let center = egui::pos2(
        viewport_rect.right() - ring_r - margin,
        viewport_rect.top() + ring_r + margin,
    );

    // Allocate interaction areas.
    //
    // IMPORTANT: allocate the LARGER ring_resp FIRST and the smaller cube_resp
    // SECOND. In egui, a later `allocate_rect` is layered ON TOP and claims
    // pointer interaction over earlier ones. Previously the order was reversed
    // (cube first, then ring), so the ring — which fully contains the cube —
    // was on top and intercepted every click over the cube. That made
    // `cube_resp.clicked()` always return false, so clicking a face did
    // nothing. Allocating cube_resp last lets it receive clicks over its area,
    // while ring_resp still gets drags starting in the ring (outside the cube).
    let cube_rect = egui::Rect::from_center_size(center, egui::vec2(cube_half * 3.0, cube_half * 3.0));
    let ring_rect = egui::Rect::from_center_size(center, egui::vec2(ring_r * 2.2, ring_r * 2.2));
    let ring_resp = ui.allocate_rect(ring_rect, egui::Sense::click_and_drag());
    let cube_resp = ui.allocate_rect(cube_rect, egui::Sense::click_and_drag());
    let home_rect = egui::Rect::from_center_size(
        egui::pos2(center.x, center.y + ring_r + 16.0),
        egui::vec2(55.0, 18.0),
    );
    let home_resp = ui.allocate_rect(home_rect, egui::Sense::click());

    // Compass buttons
    let compass_r = ring_r - 5.0;
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
            state.azimuth += delta.x * 0.7;
            state.elevation = (state.elevation + delta.y * 0.7).max(-85.0).min(85.0);
            state.dragging = true;
        }
    }

    let mouse_pos = ui.input(|i| i.pointer.latest_pos());

    // ─── DRAWING ───
    let painter = ui.painter();

    // Background panel
    let bg_rect = egui::Rect::from_center_size(center, egui::vec2(ring_r * 2.0 + 16.0, ring_r * 2.0 + 16.0));
    painter.rect_filled(bg_rect, ring_r + 8.0, egui::Color32::from_black_alpha(180));

    // Compass disc
    painter.circle_filled(center, ring_r, egui::Color32::from_rgb(0x38, 0x38, 0x3e));
    painter.circle_stroke(center, ring_r, egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
    painter.circle_stroke(center, ring_r - 12.0, egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(0x45, 0x47, 0x5a)));

    // Compass ticks
    for i in 0..4 {
        let angle = (i as f32 * 90.0).to_radians();
        let x1 = center.x + angle.sin() * (ring_r - 12.0);
        let y1 = center.y - angle.cos() * (ring_r - 12.0);
        let x2 = center.x + angle.sin() * (ring_r - 2.0);
        let y2 = center.y - angle.cos() * (ring_r - 2.0);
        painter.line_segment([egui::pos2(x1, y1), egui::pos2(x2, y2)],
            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
    }

    // Compass buttons
    for (br, hovered, _orient, label) in &compass_results {
        let bg = if *hovered { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
            else { egui::Color32::from_rgb(0x2a, 0x2a, 0x35) };
        painter.rect_filled(*br, 3.0, bg);
        let tc = if *hovered { egui::Color32::from_rgb(0x1e, 0x1e, 0x2e) }
            else { egui::Color32::WHITE };
        painter.text(br.center(), egui::Align2::CENTER_CENTER, *label,
            egui::FontId::proportional(8.0), tc);
    }

    // ─── 3D CUBE (correct isometric projection) ───
    // Use rotation matrices: first rotate around Y (azimuth), then around X (elevation)
    let az_rad = state.azimuth.to_radians();
    let el_rad = state.elevation.to_radians();

    // Proper 3D rotation: Y-azimuth then X-elevation, then orthographic project (drop Z)
    let project = |x: f32, y: f32, z: f32| -> egui::Pos2 {
        // Rotate around Y axis (azimuth)
        let cos_a = az_rad.cos();
        let sin_a = az_rad.sin();
        let x1 = x * cos_a + z * sin_a;
        let z1 = -x * sin_a + z * cos_a;
        let y1 = y;

        // Rotate around X axis (elevation)
        let cos_e = el_rad.cos();
        let sin_e = el_rad.sin();
        let y2 = y1 * cos_e - z1 * sin_e;
        let z2 = y1 * sin_e + z1 * cos_e;
        let x2 = x1;

        // Orthographic projection: x → screen_x, y → screen_y (z is depth, ignored)
        egui::pos2(center.x + x2 * cube_half, center.y - y2 * cube_half)
    };

    // 8 cube vertices
    let v = [
        project(-1.0,  1.0, -1.0), // 0: top-left-back
        project( 1.0,  1.0, -1.0), // 1: top-right-back
        project( 1.0,  1.0,  1.0), // 2: top-right-front
        project(-1.0,  1.0,  1.0), // 3: top-left-front
        project(-1.0, -1.0, -1.0), // 4: bot-left-back
        project( 1.0, -1.0, -1.0), // 5: bot-right-back
        project( 1.0, -1.0,  1.0), // 6: bot-right-front
        project(-1.0, -1.0,  1.0), // 7: bot-left-front
    ];

    // Colors — WHITE cube
    let c_face    = egui::Color32::from_rgb(0xe0, 0xe0, 0xe6);
    let c_top     = egui::Color32::from_rgb(0xf2, 0xf2, 0xf6);
    let c_hidden  = egui::Color32::from_rgb(0x2a, 0x2a, 0x30);
    let c_edge    = egui::Color32::from_rgb(0x50, 0x50, 0x5a);
    let c_hover_a = egui::Color32::from_rgba_premultiplied(108, 180, 232, 160);
    let c_text    = egui::Color32::from_rgb(0x2a, 0x2a, 0x30);
    let c_text_h  = egui::Color32::WHITE;
    let edge_stroke = egui::Stroke::new(1.5_f32, c_edge);

    // Determine which faces are visible based on current rotation.
    // The camera sits at +Z looking toward -Z (orthographic projection drops Z).
    // A face is visible (front-facing) iff its rotated normal points TOWARD the
    // camera, i.e. the normal's rotated Z-component is POSITIVE (z2 > 0).
    // Face normals (before rotation):
    // Top: (0,1,0), Bottom: (0,-1,0), Front: (0,0,1), Back: (0,0,-1), Left: (-1,0,0), Right: (1,0,0)
    let face_visibility = |nx: f32, ny: f32, nz: f32| -> bool {
        // Rotate normal the same way as vertices (Y-azimuth then X-elevation)
        let cos_a = az_rad.cos();
        let sin_a = az_rad.sin();
        let x1 = nx * cos_a + nz * sin_a;
        let z1 = -nx * sin_a + nz * cos_a;
        let y1 = ny;
        let cos_e = el_rad.cos();
        let sin_e = el_rad.sin();
        let z2 = y1 * sin_e + z1 * cos_e;
        // Camera at +Z: visible iff normal's Z-component points toward +Z (z2 > 0).
        // The previous `z2 < 0.0` was inverted, which caused the cube to render
        // "inside-out" — back faces were drawn with labels/hover while front
        // faces were treated as hidden, making the cube appear transparent and
        // clicks miss the visible faces.
        z2 > 0.0
    };

    let faces = [
        ([0, 1, 2, 3], "TOP",   c_top,  ViewOrientation::Top,    0.0, 1.0, 0.0),
        ([4, 5, 6, 7], "BOT",   c_face, ViewOrientation::Bottom, 0.0, -1.0, 0.0),
        ([3, 2, 6, 7], "FRONT", c_face, ViewOrientation::Front,  0.0, 0.0, 1.0),
        ([1, 0, 4, 5], "BACK",  c_face, ViewOrientation::Back,   0.0, 0.0, -1.0),
        ([0, 3, 7, 4], "LEFT",  c_face, ViewOrientation::Left,  -1.0, 0.0, 0.0),
        ([2, 1, 5, 6], "RIGHT", c_face, ViewOrientation::Right,  1.0, 0.0, 0.0),
    ];

    // Determine visible faces and sort by depth (back to front)
    let mut visible_faces: Vec<(usize, [usize; 4], &'static str, egui::Color32, ViewOrientation, f32)> = Vec::new();
    for (i, (idx, label, color, orient, nx, ny, nz)) in faces.iter().enumerate() {
        if face_visibility(*nx, *ny, *nz) {
            // Compute average depth (z2) for sorting
            let cos_a = az_rad.cos();
            let sin_a = az_rad.sin();
            let cos_e = el_rad.cos();
            let sin_e = el_rad.sin();
            let mut avg_z = 0.0;
            for &vi in idx.iter() {
                let (x, y, z) = match vi {
                    0 => (-1.0, 1.0, -1.0), 1 => (1.0, 1.0, -1.0),
                    2 => (1.0, 1.0, 1.0),   3 => (-1.0, 1.0, 1.0),
                    4 => (-1.0, -1.0, -1.0), 5 => (1.0, -1.0, -1.0),
                    6 => (1.0, -1.0, 1.0),   7 => (-1.0, -1.0, 1.0),
                    _ => (0.0, 0.0, 0.0),
                };
                let x1 = x * cos_a + z * sin_a;
                let z1 = -x * sin_a + z * cos_a;
                let y1 = y;
                let z2 = y1 * sin_e + z1 * cos_e;
                avg_z += z2;
            }
            avg_z /= 4.0;
            visible_faces.push((i, *idx, *label, *color, *orient, avg_z));
        }
    }
    // Sort visible faces ASCENDING by depth (z2) so the FARTHEST face is drawn
    // first and the NEAREST face is drawn LAST (painter's algorithm).
    // Camera is at +Z, so higher z2 = closer to camera. Drawing far-first lets
    // near faces correctly overlay far faces. The previous descending sort
    // drew near faces first, causing them to be covered by far faces.
    visible_faces.sort_by(|a, b| a.5.partial_cmp(&b.5).unwrap_or(std::cmp::Ordering::Equal));

    // Draw hidden faces first (dimmed)
    let hidden_indices: Vec<usize> = faces.iter().enumerate()
        .filter(|(_, (_, _, _, _, nx, ny, nz))| !face_visibility(*nx, *ny, *nz))
        .map(|(i, _)| i)
        .collect();
    for &hi in &hidden_indices {
        let (idx, _label, _color, _orient, _, _, _) = &faces[hi];
        let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
        painter.add(egui::Shape::convex_polygon(pts, c_hidden,
            egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(0x31, 0x32, 0x44))));
    }

    // Detect hover on visible faces
    let mut hovered_face: Option<usize> = None;
    if let Some(mp) = mouse_pos {
        if cube_rect.contains(mp) && !state.dragging {
            // Check front-most face first (last in sorted list = front)
            for (orig_idx, idx, _, _, _, _) in visible_faces.iter().rev() {
                let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
                if point_in_polygon(mp, &pts) {
                    hovered_face = Some(*orig_idx);
                    break;
                }
            }
        }
    }

    // Draw visible faces (back to front)
    for (orig_idx, idx, label, color, orient, _depth) in &visible_faces {
        let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
        let fill = if hovered_face == Some(*orig_idx) { c_hover_a } else { *color };
        painter.add(egui::Shape::convex_polygon(pts.clone(), fill, edge_stroke));
        // Label
        let cx = pts.iter().map(|p| p.x).sum::<f32>() / pts.len() as f32;
        let cy = pts.iter().map(|p| p.y).sum::<f32>() / pts.len() as f32;
        let tc = if hovered_face == Some(*orig_idx) { c_text_h } else { c_text };
        painter.text(egui::pos2(cx, cy), egui::Align2::CENTER_CENTER,
            *label, egui::FontId::proportional(8.0), tc);
    }

    // Draw all edges
    let edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)];
    for &(a, b) in &edges {
        painter.line_segment([v[a], v[b]], edge_stroke);
    }

    // Handle clicks
    if cube_resp.clicked() && !state.dragging {
        if let Some(mp) = mouse_pos {
            // Check visible faces (front first)
            for (orig_idx, idx, _, _, orient, _) in visible_faces.iter().rev() {
                let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
                if point_in_polygon(mp, &pts) {
                    selected = Some(*orient);
                    state.azimuth = 35.0;
                    state.elevation = 25.0;
                    break;
                }
            }
        }
    }

    // Home button
    let home_bg = if home_resp.hovered() { egui::Color32::from_rgb(0x45, 0x47, 0x5a) }
        else { egui::Color32::from_rgb(0x2a, 0x2a, 0x30) };
    painter.rect_filled(home_rect, 4.0, home_bg);
    let home_tc = if home_resp.hovered() { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
        else { egui::Color32::from_rgb(0xc8, 0xc8, 0xd0) };
    painter.text(egui::pos2(home_rect.left() + 12.0, home_rect.center().y),
        egui::Align2::CENTER_CENTER, "\u{2302}",
        egui::FontId::proportional(12.0), home_tc);
    painter.text(egui::pos2(home_rect.left() + 24.0, home_rect.center().y),
        egui::Align2::LEFT_CENTER, "Home",
        egui::FontId::proportional(9.0), home_tc);
    if home_resp.clicked() {
        selected = Some(ViewOrientation::Iso);
        state.azimuth = 35.0;
        state.elevation = 25.0;
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
