// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! ViewCube — 3D orientation indicator and controller (Autodesk-style).
//!
//! Features:
//! - Large 3D cube with 3 visible faces (Top/Front/Right)
//! - Right face is light-blue, Top is dark-grey, Front is near-black
//! - Green corner dots on visible vertices
//! - Compass ring with N/S/W/E in rounded-rect badges
//! - Home button (□ Home) below ring
//! - DRAG cube or ring → orbit camera (live rotation)
//! - Click face → snap to that view
//! - Click corner → ISO view
//! - Click compass → rotate to that cardinal direction

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

/// Persistent state for ViewCube drag rotation.
#[derive(Clone, Debug, Default)]
pub struct ViewCubeState {
    /// Current azimuth angle (degrees) — rotated by dragging.
    pub azimuth: f32,
    /// Current elevation angle (degrees).
    pub elevation: f32,
    /// Is the cube being dragged?
    pub dragging: bool,
    /// Last drag position.
    pub last_pos: Option<egui::Pos2>,
}

impl ViewCubeState {
    pub fn new() -> Self {
        Self {
            azimuth: 45.0,
            elevation: 35.264,
            dragging: false,
            last_pos: None,
        }
    }
}

/// Render the ViewCube inside the viewport (top-right corner).
/// Supports drag-to-rotate on both cube and compass ring.
pub fn render_view_cube_in_viewport(
    ui: &mut egui::Ui,
    viewport_rect: &egui::Rect,
    state: &mut ViewCubeState,
) -> Option<ViewOrientation> {
    let mut selected = None;

    // Layout — cube is large, ring around it
    let ring_radius = 55.0_f32; // outer ring radius
    let margin = 10.0_f32;
    let total_size = ring_radius * 2.0 + 10.0;
    let home_h = 20.0_f32;

    let center = egui::pos2(
        viewport_rect.right() - ring_radius - margin - 5.0,
        viewport_rect.top() + ring_radius + margin + 5.0,
    );

    let cube_half = 24.0_f32; // half-edge of cube in pixels (large!)

    // Allocate interaction areas
    // 1. Cube area (for drag + click)
    let cube_rect = egui::Rect::from_center_size(center, egui::vec2(cube_half * 2.5, cube_half * 2.5));
    let cube_resp = ui.allocate_rect(cube_rect, egui::Sense::click_and_drag());

    // 2. Ring area (for drag + compass clicks)
    let ring_rect = egui::Rect::from_center_size(center, egui::vec2(ring_radius * 2.0, ring_radius * 2.0));
    let ring_resp = ui.allocate_rect(ring_rect, egui::Sense::click_and_drag());

    // 3. Home button
    let home_rect = egui::Rect::from_center_size(
        egui::pos2(center.x, center.y + ring_radius + 12.0),
        egui::vec2(60.0, home_h),
    );
    let home_resp = ui.allocate_rect(home_rect, egui::Sense::click());

    // 4. Compass buttons (N/S/W/E)
    let compass_r = ring_radius - 6.0;
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
        let br = egui::Rect::from_center_size(egui::pos2(px, py), egui::vec2(16.0, 16.0));
        let resp = ui.allocate_rect(br, egui::Sense::click());
        compass_results.push((br, resp.hovered(), *orient, *label));
        if resp.clicked() { selected = Some(*orient); }
    }

    // Handle drag rotation (cube or ring)
    let drag_resp = if cube_resp.dragged_by(egui::PointerButton::Primary) {
        Some(&cube_resp)
    } else if ring_resp.dragged_by(egui::PointerButton::Primary) {
        Some(&ring_resp)
    } else {
        None
    };

    if let Some(dr) = drag_resp {
        let delta = dr.drag_delta();
        if delta.length_sq() > 0.5 {
            state.azimuth += delta.x * 0.8;
            state.elevation = (state.elevation - delta.y * 0.8).max(-89.0).min(89.0);
            state.dragging = true;
        }
    }

    // Handle cube click (not drag)
    if cube_resp.clicked() && !state.dragging {
        // Determine which face was clicked based on mouse position relative to cube center
        let mp = ui.input(|i| i.pointer.latest_pos());
        if let Some(mp) = mp {
            let dx = mp.x - center.x;
            let dy = mp.y - center.y;
            // Project mouse onto cube faces using current azimuth/elevation
            // Simplified: check which region of the cube was clicked
            if dy < -cube_half * 0.5 {
                selected = Some(ViewOrientation::Top);
            } else if dx > cube_half * 0.3 {
                selected = Some(ViewOrientation::Right);
            } else if dx < -cube_half * 0.3 {
                selected = Some(ViewOrientation::Left);
            } else {
                selected = Some(ViewOrientation::Front);
            }
        }
    }

    // Reset dragging state when drag ends
    if !cube_resp.dragged() && !ring_resp.dragged() {
        state.dragging = false;
    }

    // ─── DRAWING ───
    let painter = ui.painter();

    // Background circle (dark, semi-transparent)
    let bg_rect = egui::Rect::from_center_size(center, egui::vec2(ring_radius * 2.0 + 8.0, ring_radius * 2.0 + 8.0));
    painter.rect_filled(bg_rect, ring_radius + 4.0, egui::Color32::from_black_alpha(160));

    // Compass ring (thin dark circle)
    painter.circle_stroke(center, ring_radius, egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(0x45, 0x47, 0x5a)));
    painter.circle_stroke(center, ring_radius - 12.0, egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(0x31, 0x32, 0x44)));

    // Compass tick marks (every 90°)
    for i in 0..4 {
        let angle = (i as f32 * 90.0).to_radians();
        let x1 = center.x + angle.sin() * (ring_radius - 12.0);
        let y1 = center.y - angle.cos() * (ring_radius - 12.0);
        let x2 = center.x + angle.sin() * ring_radius;
        let y2 = center.y - angle.cos() * ring_radius;
        painter.line_segment(
            [egui::pos2(x1, y1), egui::pos2(x2, y2)],
            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(0x6c, 0x70, 0x86)),
        );
    }

    // Compass buttons (N/S/W/E in rounded rects)
    for (br, hovered, _orient, label) in &compass_results {
        let bg = if *hovered {
            egui::Color32::from_rgb(0x89, 0xb4, 0xfa)
        } else {
            egui::Color32::from_rgb(0x31, 0x32, 0x44)
        };
        painter.rect_filled(*br, 4.0, bg);
        let tc = if *hovered { egui::Color32::from_rgb(0x1e, 0x1e, 0x2e) } else { egui::Color32::WHITE };
        painter.text(br.center(), egui::Align2::CENTER_CENTER, *label,
            egui::FontId::proportional(9.0), tc);
    }

    // ─── 3D CUBE ───
    // Use current azimuth/elevation from state (allows drag rotation)
    let az = state.azimuth.to_radians();
    let el = state.elevation.to_radians();

    let project = |x: f32, y: f32, z: f32| -> egui::Pos2 {
        // Rotate around Y (azimuth), then around X (elevation)
        let x1 = x * az.cos() + z * az.sin();
        let z1 = -x * az.sin() + z * az.cos();
        let y2 = y * el.cos() - z1 * el.sin();
        egui::pos2(center.x + x1 * cube_half, center.y - y2 * cube_half)
    };

    // 8 cube vertices
    let v = [
        project(-1.0,  1.0, -1.0), // 0: top-left-back
        project( 1.0,  1.0, -1.0), // 1: top-right-back
        project( 1.0,  1.0,  1.0), // 2: top-right-front
        project(-1.0,  1.0,  1.0), // 3: top-left-front
        project(-1.0, -1.0, -1.0), // 4: bottom-left-back
        project( 1.0, -1.0, -1.0), // 5: bottom-right-back
        project( 1.0, -1.0,  1.0), // 6: bottom-right-front
        project(-1.0, -1.0,  1.0), // 7: bottom-left-front
    ];

    // Colors matching the reference image
    let c_right  = egui::Color32::from_rgb(0x6c, 0xb4, 0xe8); // light blue (azure)
    let c_top    = egui::Color32::from_rgb(0x2a, 0x2a, 0x35); // dark grey
    let c_front  = egui::Color32::from_rgb(0x1a, 0x1a, 0x22); // near black
    let c_hidden = egui::Color32::from_rgb(0x11, 0x11, 0x16); // very dark (hidden faces)
    let c_edge   = egui::Color32::from_rgb(0x6c, 0x70, 0x86); // edge lines
    let c_corner = egui::Color32::from_rgb(0xa6, 0xe3, 0xa1); // green corners
    let c_hover  = egui::Color32::from_rgb(0x89, 0xb4, 0xfa); // blue hover
    let c_text   = egui::Color32::WHITE;

    let edge_stroke = egui::Stroke::new(1.5_f32, c_edge);

    // Determine visible faces based on azimuth/elevation
    // We check the normal of each face after projection
    // For simplicity, always draw in order: hidden first, then visible
    // Visible faces (when az=45, el=35): Top (0,1,2,3), Front (3,2,6,7), Right (2,1,5,6)

    // Hidden faces (drawn first, very dark)
    let hidden_faces = [
        ([4, 5, 6, 7], "Bot"),   // Bottom
        ([0, 1, 5, 4], "Back"),  // Back
        ([0, 3, 7, 4], "Left"),  // Left
    ];
    for (idx, _label) in &hidden_faces {
        let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
        painter.add(egui::Shape::convex_polygon(
            pts,
            c_hidden,
            egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(0x31, 0x32, 0x44)),
        ));
    }

    // Detect hover on visible faces
    let mouse_pos = ui.input(|i| i.pointer.latest_pos());
    let mut hovered_face: Option<usize> = None; // 0=Top, 1=Front, 2=Right

    let visible_faces = [
        ([0, 1, 2, 3], "Top",   c_top,   ViewOrientation::Top,   0),
        ([3, 2, 6, 7], "Front", c_front, ViewOrientation::Front, 1),
        ([2, 1, 5, 6], "Right", c_right, ViewOrientation::Right, 2),
    ];

    if let Some(mp) = mouse_pos {
        if cube_rect.contains(mp) && !state.dragging {
            // Check faces in reverse order (front-most first)
            for (_, _, _, _, fi) in visible_faces.iter().rev() {
                let face = &visible_faces[*fi];
                let pts: Vec<egui::Pos2> = face.0.iter().map(|&i| v[i]).collect();
                if point_in_polygon(mp, &pts) {
                    hovered_face = Some(face.4);
                    break;
                }
            }
        }
    }

    // Draw visible faces
    for (idx, label, color, orient, _fi) in &visible_faces {
        let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
        let fill = if hovered_face == Some(*_fi) { c_hover } else { *color };
        painter.add(egui::Shape::convex_polygon(
            pts.clone(),
            fill,
            edge_stroke,
        ));
        // Face label (centered on face)
        let cx = pts.iter().map(|p| p.x).sum::<f32>() / pts.len() as f32;
        let cy = pts.iter().map(|p| p.y).sum::<f32>() / pts.len() as f32;
        let tc = if hovered_face == Some(*_fi) {
            egui::Color32::from_rgb(0x1e, 0x1e, 0x2e)
        } else {
            c_text
        };
        painter.text(egui::pos2(cx, cy), egui::Align2::CENTER_CENTER,
            *label, egui::FontId::proportional(9.0), tc);
    }

    // Draw edges (all 12, visible ones thicker)
    let edges = [
        (0, 1), (1, 2), (2, 3), (3, 0), // top
        (4, 5), (5, 6), (6, 7), (7, 4), // bottom
        (0, 4), (1, 5), (2, 6), (3, 7), // verticals
    ];
    for &(a, b) in &edges {
        painter.line_segment([v[a], v[b]], edge_stroke);
    }

    // Draw corner dots (green) on visible corners
    // Visible corners: 2 (TRF), 3 (TLF), 6 (BRF), 7 (BLF)
    let visible_corners = [2, 3, 6, 7];
    for &ci in &visible_corners {
        let is_hover = mouse_pos.map(|mp| (mp - v[ci]).length() < 8.0).unwrap_or(false);
        let r = if is_hover { 5.0 } else { 3.5 };
        let color = if is_hover { egui::Color32::WHITE } else { c_corner };
        painter.circle_filled(v[ci], r, color);
        if is_hover {
            painter.circle_stroke(v[ci], 7.0, egui::Stroke::new(2.0_f32, egui::Color32::WHITE));
        }
    }

    // Handle corner click
    if cube_resp.clicked() && !state.dragging {
        if let Some(mp) = mouse_pos {
            for &ci in &visible_corners {
                if (mp - v[ci]).length() < 8.0 {
                    selected = Some(ViewOrientation::Iso);
                    break;
                }
            }
        }
    }

    // Home button
    let home_bg = if home_resp.hovered() {
        egui::Color32::from_rgb(0x45, 0x47, 0x5a)
    } else {
        egui::Color32::from_rgb(0x31, 0x32, 0x44)
    };
    painter.rect_filled(home_rect, 4.0, home_bg);
    let home_tc = if home_resp.hovered() {
        egui::Color32::from_rgb(0x89, 0xb4, 0xfa)
    } else {
        egui::Color32::from_rgb(0xcd, 0xd6, 0xf4)
    };
    // Draw □ symbol + "Home" text
    let symbol_rect = egui::Rect::from_center_size(
        egui::pos2(home_rect.left() + 12.0, home_rect.center().y),
        egui::vec2(8.0, 8.0),
    );
    painter.rect_stroke(symbol_rect, 1.0, egui::Stroke::new(1.0_f32, home_tc), egui::StrokeKind::Outside);
    painter.text(
        egui::pos2(home_rect.left() + 20.0, home_rect.center().y),
        egui::Align2::LEFT_CENTER,
        "Home",
        egui::FontId::proportional(10.0),
        home_tc,
    );
    if home_resp.clicked() {
        selected = Some(ViewOrientation::Iso);
        // Reset drag state
        state.azimuth = 45.0;
        state.elevation = 35.264;
    }

    selected
}

/// Point-in-polygon test (ray casting).
fn point_in_polygon(p: egui::Pos2, polygon: &[egui::Pos2]) -> bool {
    let n = polygon.len();
    if n < 3 { return false; }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = polygon[i];
        let pj = polygon[j];
        if ((pi.y > p.y) != (pj.y > p.y))
            && (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y + 1e-10) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
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

// Old API for backward compat (3Draper Viewer)
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
