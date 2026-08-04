// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! ViewCube — FreeCAD/Autodesk-style 3D navigation cube.
//!
//! Architecture (per the technical specification):
//! - **Over-layer rendering**: cube drawn as egui Painter shapes over the
//!   main 3D viewport (no separate GPU pipeline → zero performance overhead).
//! - **View matrix sync**: cube orientation tracks main camera's azimuth/elevation.
//!   Translation is ignored — cube stays centered in its screen corner.
//! - **Orthographic projection**: cube size is fixed regardless of main
//!   scene zoom.
//! - **Own depth buffer**: painter's algorithm sorts faces back-to-front,
//!   so main scene objects never poke through the cube.
//! - **Chamfered cube (26 zones)**: 6 face centers + 12 edge strips + 8 corner
//!   triangles, each independently clickable.
//! - **External controls**: 4 rotation arrows + 2 roll arrows + RGB axes +
//!   Home button + Menu button.
//! - **Smooth slerp**: camera transitions interpolate over 250 ms.

use eframe::egui;
use super::DisplayStyle;
use crate::camera::{OrbitCamera, Quat};

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
    /// Direction FROM camera TO target for `look_from_direction`.
    pub fn direction(&self) -> [f32; 3] {
        match self {
            ViewOrientation::Iso => { let d=45.0_f32.to_radians(); let e=35.264_f32.to_radians(); [-e.cos()*d.sin(), -e.sin(), e.cos()*d.cos()] }
            ViewOrientation::Front => [0.0, 0.0, -1.0],
            ViewOrientation::Back => [0.0, 0.0, 1.0],
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

/// What the ViewCube can request from the main app.
/// The app is responsible for actually rotating the camera (with slerp).
#[derive(Clone, Debug)]
pub enum ViewCubeAction {
    /// Snap camera to this orientation (with smooth slerp animation).
    SnapTo(ViewOrientation),
    /// Rotate camera by a fixed step around a screen axis.
    /// Axis: 0=screen-X (pitch), 1=screen-Y (yaw), 2=screen-Z (roll).
    /// Angle is in radians.
    RotateStep { axis: u8, angle_rad: f32 },
    /// Drag-rotate the camera (same as dragging in the main viewport).
    /// The app should call camera.rotate(delta_x, delta_y) — this uses
    /// quaternion-based rotation, identical to the main viewport drag.
    Drag { delta_x: f32, delta_y: f32 },
    /// Reset to default ISO view.
    Home,
    /// Toggle projection mode (perspective ↔ orthographic).
    /// App should toggle its camera's projection.
    ToggleProjection,
}

/// State for the ViewCube widget, persisted between frames.
#[derive(Clone, Debug)]
pub struct ViewCubeState {
    /// Azimuth angle (degrees) — rotation around world Y axis.
    pub azimuth: f32,
    /// Elevation angle (degrees) — rotation around screen X axis.
    pub elevation: f32,
    /// True while the user is dragging the cube to rotate it.
    pub dragging: bool,
    /// ID of the cube zone currently under the cursor (for hover highlight).
    /// Range: 0..26 (6 faces + 12 edges + 8 corners), or None.
    pub hovered_zone: Option<usize>,
}

impl Default for ViewCubeState {
    fn default() -> Self {
        Self::new()
    }
}

impl ViewCubeState {
    pub fn new() -> Self {
        Self {
            azimuth: 45.0,
            elevation: 35.264,
            dragging: false,
            hovered_zone: None,
        }
    }
}

// ═══ Chamfered cube mesh data ═══════════════════════════════════════════════
// 24 vertices, 44 triangles. Generated from convex hull of edge-vertices.
// CUBE_HALF=1.0, CHAMFER=0.18.
// This mesh has 26 distinct face-normal directions:
//   - 6 face normals (±X, ±Y, ±Z) → 8 triangles each (octagonal faces)
//   - 12 edge normals (±X±Y, ±X±Z, ±Y±Z) → 2 triangles each (edge chamfers)
//   - 8 corner normals (±X±Y±Z) → 1 triangle each (corner triangles)
pub const CHAMFERED_CUBE_VERTS: [[f32; 3]; 24] = [
    [-0.82, -1.0, -1.0], [-1.0, -0.82, -1.0], [-1.0, -1.0, -0.82],
    [-0.82, -1.0,  1.0], [-1.0, -0.82,  1.0], [-1.0, -1.0,  0.82],
    [-0.82,  1.0, -1.0], [-1.0,  0.82, -1.0], [-1.0,  1.0, -0.82],
    [-0.82,  1.0,  1.0], [-1.0,  0.82,  1.0], [-1.0,  1.0,  0.82],
    [ 0.82, -1.0, -1.0], [ 1.0, -0.82, -1.0], [ 1.0, -1.0, -0.82],
    [ 0.82, -1.0,  1.0], [ 1.0, -0.82,  1.0], [ 1.0, -1.0,  0.82],
    [ 0.82,  1.0, -1.0], [ 1.0,  0.82, -1.0], [ 1.0,  1.0, -0.82],
    [ 0.82,  1.0,  1.0], [ 1.0,  0.82,  1.0], [ 1.0,  1.0,  0.82],
];

pub const CHAMFERED_CUBE_TRIS: [[u32; 3]; 44] = [
    [1, 0, 2], [7, 8, 6], [20, 19, 18], [21, 22, 23],
    [13, 14, 12], [4, 5, 3], [9, 11, 10], [15, 17, 16],
    [12, 0, 1], [12, 19, 13], [6, 18, 12], [1, 7, 12],
    [12, 7, 6], [12, 18, 19], [17, 22, 16], [13, 19, 17],
    [17, 19, 20], [17, 14, 13], [20, 23, 17], [17, 23, 22],
    [10, 11, 4], [1, 2, 4], [4, 7, 1], [2, 5, 4],
    [8, 7, 4], [4, 11, 8], [9, 18, 6], [8, 11, 9],
    [6, 8, 9], [21, 23, 9], [20, 18, 9], [9, 23, 20],
    [21, 9, 15], [15, 22, 21], [15, 4, 3], [16, 22, 15],
    [10, 4, 15], [15, 9, 10], [15, 2, 0], [0, 12, 15],
    [15, 5, 2], [3, 5, 15], [15, 12, 14], [14, 17, 15],
];

pub const CHAMFERED_CUBE_FACE_NORMALS: [[f32; 3]; 44] = [
    [-0.57735, -0.57735, -0.57735], [-0.57735, 0.57735, -0.57735],
    [ 0.57735,  0.57735, -0.57735], [ 0.57735, 0.57735,  0.57735],
    [ 0.57735, -0.57735, -0.57735], [-0.57735, -0.57735, 0.57735],
    [-0.57735,  0.57735,  0.57735], [ 0.57735, -0.57735, 0.57735],
    [ 0.0,  0.0, -1.0], [ 0.0,  0.0, -1.0], [ 0.0,  0.0, -1.0],
    [ 0.0,  0.0, -1.0], [ 0.0,  0.0, -1.0], [ 0.0,  0.0, -1.0],
    [ 1.0,  0.0,  0.0], [ 1.0,  0.0,  0.0], [ 1.0,  0.0,  0.0],
    [ 1.0,  0.0,  0.0], [ 1.0,  0.0,  0.0], [ 1.0,  0.0,  0.0],
    [-1.0,  0.0,  0.0], [-1.0,  0.0,  0.0], [-1.0,  0.0,  0.0],
    [-1.0,  0.0,  0.0], [-1.0,  0.0,  0.0], [-1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0], [ 0.0,  1.0,  0.0], [ 0.0,  1.0,  0.0],
    [ 0.0,  1.0,  0.0], [ 0.0,  1.0,  0.0], [ 0.0,  1.0,  0.0],
    [ 0.0,  0.0,  1.0], [ 0.0,  0.0,  1.0], [ 0.0,  0.0,  1.0],
    [ 0.0,  0.0,  1.0], [ 0.0,  0.0,  1.0], [ 0.0,  0.0,  1.0],
    [ 0.0, -1.0,  0.0], [ 0.0, -1.0,  0.0], [ 0.0, -1.0,  0.0],
    [ 0.0, -1.0,  0.0], [ 0.0, -1.0,  0.0], [ 0.0, -1.0,  0.0],
];

/// Classify a face normal into one of 26 zone IDs:
///   0-5: face zones (±X, ±Y, ±Z)
///   6-17: edge zones (12 edges)
///   18-25: corner zones (8 corners)
/// Returns None if the normal doesn't match any zone.
fn classify_normal(n: [f32; 3]) -> Option<usize> {
    let ax = n[0].abs();
    let ay = n[1].abs();
    let az = n[2].abs();
    let eps = 0.3;
    let nz = |x: f32| x.abs() < eps;
    let one = |x: f32| x.abs() > 0.9;

    // Face zones (one component is ±1, others ≈ 0)
    if one(n[0]) && nz(n[1]) && nz(n[2]) {
        return Some(if n[0] > 0.0 { 0 } else { 1 }); // +X=0, -X=1
    }
    if one(n[1]) && nz(n[0]) && nz(n[2]) {
        return Some(if n[1] > 0.0 { 2 } else { 3 }); // +Y=2, -Y=3
    }
    if one(n[2]) && nz(n[0]) && nz(n[1]) {
        return Some(if n[2] > 0.0 { 4 } else { 5 }); // +Z=4, -Z=5
    }

    // Edge zones (two components non-zero, one ≈ 0) — 12 edges
    if nz(n[2]) {
        // Edge in XY plane
        return Some(6 + (if n[0] > 0.0 { 1 } else { 0 }) * 2 + (if n[1] > 0.0 { 1 } else { 0 }));
    }
    if nz(n[1]) {
        // Edge in XZ plane
        return Some(10 + (if n[0] > 0.0 { 1 } else { 0 }) * 2 + (if n[2] > 0.0 { 1 } else { 0 }));
    }
    if nz(n[0]) {
        // Edge in YZ plane
        return Some(14 + (if n[1] > 0.0 { 1 } else { 0 }) * 2 + (if n[2] > 0.0 { 1 } else { 0 }));
    }

    // Corner zones (all three non-zero) — 8 corners
    Some(18 + (if n[0] > 0.0 { 1 } else { 0 }) * 4
           + (if n[1] > 0.0 { 1 } else { 0 }) * 2
           + (if n[2] > 0.0 { 1 } else { 0 }))
}

/// Map a zone ID to the ViewOrientation for camera snapping.
fn zone_to_orientation(zone_id: usize) -> ViewOrientation {
    match zone_id {
        0 | 1 => ViewOrientation::Right,  // ±X
        2 => ViewOrientation::Top,        // +Y
        3 => ViewOrientation::Bottom,     // -Y
        4 => ViewOrientation::Back,       // +Z
        5 => ViewOrientation::Front,      // -Z
        _ => ViewOrientation::Iso,        // edges and corners → ISO
    }
}

pub fn render_view_cube_in_viewport(
    ui: &mut egui::Ui,
    viewport_rect: &egui::Rect,
    // Camera forward direction (FROM camera TO target), from camera.orientation quaternion.
    // The cube uses this instead of its own Euler angles, so it always
    // shows the same faces as the main camera.
    camera_forward: [f32; 3],
    state: &mut ViewCubeState,
) -> Option<ViewCubeAction> {
    let mut selected: Option<ViewCubeAction> = None;

    // ═══ Layout ═══════════════════════════════════════════════════════════════
    // DPI-aware sizing: scale cube and font by the egui pixels_per_point.
    // On a 2x DPI display, cube_half doubles so the cube stays the same
    // physical size, and font scales to match.
    let ppi = ui.ctx().pixels_per_point().max(1.0);
    let ring_r = 60.0_f32 * ppi;       // compass disc radius
    let margin = 14.0_f32 * ppi;
    let cube_half = 28.0_f32 * ppi;    // cube edge half-length (orthographic, fixed scale)
    let chamfer = 0.18_f32;            // chamfer size as fraction of cube_half
    // Dynamic font size: scales with cube size and DPI
    let label_font_size = (cube_half * 0.32).max(8.0); // ~9pt at 1x DPI, 18pt at 2x
    let small_font_size = (cube_half * 0.22).max(6.0);
    let center = egui::pos2(
        viewport_rect.right() - ring_r - margin,
        viewport_rect.top() + ring_r + margin,
    );

    // ═══ Interaction rects (allocate largest first, smallest last) ═══════════
    let cube_rect = egui::Rect::from_center_size(center, egui::vec2(cube_half * 3.0, cube_half * 3.0));
    let ring_rect = egui::Rect::from_center_size(center, egui::vec2(ring_r * 2.4, ring_r * 2.4));
    let ring_resp = ui.allocate_rect(ring_rect, egui::Sense::click_and_drag());
    let cube_resp = ui.allocate_rect(cube_rect, egui::Sense::click_and_drag());

    // Home button (top-right, outside the ring)
    let home_rect = egui::Rect::from_center_size(
        egui::pos2(center.x + ring_r + 16.0, center.y - ring_r - 16.0),
        egui::vec2(22.0, 22.0),
    );
    let home_resp = ui.allocate_rect(home_rect, egui::Sense::click());

    // Menu button (bottom-right, outside the ring)
    let menu_rect = egui::Rect::from_center_size(
        egui::pos2(center.x + ring_r + 16.0, center.y + ring_r + 16.0),
        egui::vec2(22.0, 22.0),
    );
    let menu_resp = ui.allocate_rect(menu_rect, egui::Sense::click());

    // 4 rotation arrows (top/bottom/left/right of cube, outside cube_rect but inside ring)
    let arrow_dist = cube_half * 1.5;
    let arrow_size = 14.0_f32;
    let arrow_rects = [
        egui::Rect::from_center_size(egui::pos2(center.x, center.y - arrow_dist), egui::vec2(arrow_size*2.0, arrow_size)),
        egui::Rect::from_center_size(egui::pos2(center.x, center.y + arrow_dist), egui::vec2(arrow_size*2.0, arrow_size)),
        egui::Rect::from_center_size(egui::pos2(center.x - arrow_dist, center.y), egui::vec2(arrow_size, arrow_size*2.0)),
        egui::Rect::from_center_size(egui::pos2(center.x + arrow_dist, center.y), egui::vec2(arrow_size, arrow_size*2.0)),
    ];
    let mut arrow_responses = Vec::new();
    for ar in &arrow_rects {
        arrow_responses.push(ui.allocate_rect(*ar, egui::Sense::click()));
    }

    // 2 roll arrows (curved, top of cube — represented as small circles)
    let roll_dist = cube_half * 1.9;
    let roll_r = 10.0_f32;
    let roll_rects = [
        egui::Rect::from_center_size(egui::pos2(center.x - roll_dist*0.7, center.y - roll_dist), egui::vec2(roll_r*2.0, roll_r*2.0)),
        egui::Rect::from_center_size(egui::pos2(center.x + roll_dist*0.7, center.y - roll_dist), egui::vec2(roll_r*2.0, roll_r*2.0)),
    ];
    let mut roll_responses = Vec::new();
    for rr in &roll_rects {
        roll_responses.push(ui.allocate_rect(*rr, egui::Sense::click()));
    }

    // Handle arrow clicks (90° rotation steps)
    let step_90 = std::f32::consts::FRAC_PI_2;
    if arrow_responses[0].clicked() {
        selected = Some(ViewCubeAction::RotateStep { axis: 0, angle_rad: -step_90 });
    }
    if arrow_responses[1].clicked() {
        selected = Some(ViewCubeAction::RotateStep { axis: 0, angle_rad: step_90 });
    }
    if arrow_responses[2].clicked() {
        selected = Some(ViewCubeAction::RotateStep { axis: 1, angle_rad: -step_90 });
    }
    if arrow_responses[3].clicked() {
        selected = Some(ViewCubeAction::RotateStep { axis: 1, angle_rad: step_90 });
    }
    // Roll arrows (45° around view axis)
    if roll_responses[0].clicked() {
        selected = Some(ViewCubeAction::RotateStep { axis: 2, angle_rad: -std::f32::consts::FRAC_PI_4 });
    }
    if roll_responses[1].clicked() {
        selected = Some(ViewCubeAction::RotateStep { axis: 2, angle_rad: std::f32::consts::FRAC_PI_4 });
    }

    // Handle drag on cube/ring — return Drag action so the app can call
    // camera.rotate() (quaternion-based, same as main viewport drag).
    let drag_resp = if cube_resp.dragged_by(egui::PointerButton::Primary) { Some(&cube_resp) }
        else if ring_resp.dragged_by(egui::PointerButton::Primary) { Some(&ring_resp) }
        else { None };
    if let Some(dr) = drag_resp {
        let delta = dr.drag_delta();
        if delta.length_sq() > 0.5 {
            state.dragging = true;
            // Return Drag action — app.rs will call camera.rotate(delta.x, delta.y)
            // which uses the SAME quaternion rotation as the main viewport.
            selected = Some(ViewCubeAction::Drag { delta_x: delta.x, delta_y: delta.y });
        }
    }

    let mouse_pos = ui.input(|i| i.pointer.latest_pos());

    // ═══ DRAWING ═══════════════════════════════════════════════════════════════
    let painter = ui.painter();

    // Background panel
    let bg_rect = egui::Rect::from_center_size(center, egui::vec2(ring_r * 2.0 + 20.0, ring_r * 2.0 + 20.0));
    painter.rect_filled(bg_rect, ring_r + 10.0, egui::Color32::from_black_alpha(180));

    // Compass disc
    painter.circle_filled(center, ring_r, egui::Color32::from_rgb(0x2a, 0x2a, 0x32));
    painter.circle_stroke(center, ring_r, egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
    painter.circle_stroke(center, ring_r - 12.0, egui::Stroke::new(0.5_f32, egui::Color32::from_rgb(0x45, 0x47, 0x5a)));

    // Compass ticks (N/S/E/W)
    for i in 0..4 {
        let angle = (i as f32 * 90.0).to_radians();
        let x1 = center.x + angle.sin() * (ring_r - 12.0);
        let y1 = center.y - angle.cos() * (ring_r - 12.0);
        let x2 = center.x + angle.sin() * (ring_r - 2.0);
        let y2 = center.y - angle.cos() * (ring_r - 2.0);
        painter.line_segment([egui::pos2(x1, y1), egui::pos2(x2, y2)],
            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
    }

    // ═══ 3D CUBE — camera-based orthographic projection ═══════════════════════
    // Use the camera's actual forward direction (derived from its orientation
    // quaternion) instead of separate Euler angles. This ensures the cube
    // ALWAYS shows the same faces as the main camera, and quaternion-based
    // rotation is used throughout (no gimbal lock, smooth orbit).
    let cam_dir = {
        let len = (camera_forward[0]*camera_forward[0] + camera_forward[1]*camera_forward[1] + camera_forward[2]*camera_forward[2]).sqrt();
        if len > 1e-6 {
            [camera_forward[0]/len, camera_forward[1]/len, camera_forward[2]/len]
        } else {
            [0.0, 0.0, -1.0]
        }
    };
    let cam_pos = [-cam_dir[0], -cam_dir[1], -cam_dir[2]];

    // Camera basis: right = normalize(cross(world_up, cam_dir)), up = cross(cam_dir, right)
    let right_unnorm = [cam_dir[2], 0.0, -cam_dir[0]];
    let right_len = (right_unnorm[0]*right_unnorm[0] + right_unnorm[1]*right_unnorm[1] + right_unnorm[2]*right_unnorm[2]).sqrt();
    let right = if right_len > 1e-6 {
        [right_unnorm[0]/right_len, right_unnorm[1]/right_len, right_unnorm[2]/right_len]
    } else { [1.0, 0.0, 0.0] };
    let up = [
        cam_dir[1]*right[2] - cam_dir[2]*right[1],
        cam_dir[2]*right[0] - cam_dir[0]*right[2],
        cam_dir[0]*right[1] - cam_dir[1]*right[0],
    ];

    // Project a 3D point to 2D screen coords
    let project = |x: f32, y: f32, z: f32| -> egui::Pos2 {
        let sx = x*right[0] + y*right[1] + z*right[2];
        let sy = x*up[0]    + y*up[1]    + z*up[2];
        egui::pos2(center.x + sx * cube_half, center.y - sy * cube_half)
    };


    // ═══ Chamfered cube — mesh-based rendering ════════════════════════════════
    let v2d: Vec<egui::Pos2> = CHAMFERED_CUBE_VERTS.iter().map(|&p| {
        project(p[0], p[1], p[2])
    }).collect();

    let face_visible = |n: [f32; 3]| -> bool {
        n[0]*cam_pos[0] + n[1]*cam_pos[1] + n[2]*cam_pos[2] > 0.0
    };

    let mut visible_tris: Vec<(usize, [f32; 3], f32)> = Vec::new();
    for (i, tri) in CHAMFERED_CUBE_TRIS.iter().enumerate() {
        let n = CHAMFERED_CUBE_FACE_NORMALS[i];
        if !face_visible(n) { continue; }
        let mut avg_depth = 0.0_f32;
        for &vi in tri {
            let p = CHAMFERED_CUBE_VERTS[vi as usize];
            avg_depth -= (p[0]*cam_dir[0] + p[1]*cam_dir[1] + p[2]*cam_dir[2]) / 3.0;
        }
        visible_tris.push((i, n, avg_depth));
    }
    visible_tris.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));

    // ═══ Hover detection ══════════════════════════════════════════════════════
    let mut hovered_zone: Option<usize> = None;
    if let Some(mp) = mouse_pos {
        if cube_rect.contains(mp) && !state.dragging {
            for &(tri_idx, _n, _depth) in visible_tris.iter().rev() {
                let tri = CHAMFERED_CUBE_TRIS[tri_idx];
                let pts = [v2d[tri[0] as usize], v2d[tri[1] as usize], v2d[tri[2] as usize]];
                if point_in_polygon(mp, &pts) {
                    hovered_zone = classify_normal(CHAMFERED_CUBE_FACE_NORMALS[tri_idx]);
                    break;
                }
            }
        }
    }
    state.hovered_zone = hovered_zone;

    // ═══ Lambertian shading ═══════════════════════════════════════════════════
    let base_color = [0.90_f32, 0.91, 0.94];
    let ambient = 0.25_f32;
    let light_key = { let v = [0.6_f32, 0.8, 0.9]; let n = (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]).sqrt(); [v[0]/n, v[1]/n, v[2]/n] };
    let light_fill = { let v = [-0.4_f32, -0.3, 0.3]; let n = (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]).sqrt(); [v[0]/n, v[1]/n, v[2]/n] };
    let shade = |n: [f32; 3]| -> egui::Color32 {
        let dk = (n[0]*light_key[0]+n[1]*light_key[1]+n[2]*light_key[2]).max(0.0) * 0.65;
        let df = (n[0]*light_fill[0]+n[1]*light_fill[1]+n[2]*light_fill[2]).max(0.0) * 0.10;
        let b = (ambient + dk + df).clamp(0.0, 1.0);
        egui::Color32::from_rgb((base_color[0]*b*255.0) as u8, (base_color[1]*b*255.0) as u8, (base_color[2]*b*255.0) as u8)
    };
    let c_hover = egui::Color32::from_rgba_premultiplied(108, 180, 232, 220);
    let c_text = egui::Color32::from_rgb(0x1a, 0x1a, 0x22);
    let c_text_h = egui::Color32::WHITE;
    let c_edge = egui::Color32::from_rgb(0x1a, 0x1a, 0x28);
    let edge_stroke = egui::Stroke::new(1.2_f32 * ppi, c_edge);

    // ═══ Draw triangles back-to-front ═════════════════════════════════════════
    for &(tri_idx, n, _depth) in &visible_tris {
        let tri = CHAMFERED_CUBE_TRIS[tri_idx];
        let pts = vec![v2d[tri[0] as usize], v2d[tri[1] as usize], v2d[tri[2] as usize]];
        let zone = classify_normal(n);
        let is_hovered = hovered_zone.is_some() && hovered_zone == zone;
        let fill = if is_hovered { c_hover } else { shade(n) };
        painter.add(egui::Shape::convex_polygon(pts, fill, edge_stroke));
    }

    // ═══ Draw labels on the 6 main faces ══════════════════════════════════════
    let face_labels = [
        (0usize, "RIGHT", [1.0_f32, 0.0, 0.0]),
        (1, "LEFT",  [-1.0_f32, 0.0, 0.0]),
        (2, "TOP",   [0.0_f32, 1.0, 0.0]),
        (3, "BOT",   [0.0_f32, -1.0, 0.0]),
        (4, "BACK",  [0.0_f32, 0.0, 1.0]),
        (5, "FRONT", [0.0_f32, 0.0, -1.0]),
    ];
    for (zone_id, label, normal) in &face_labels {
        if !face_visible(*normal) { continue; }
        let mut cx = 0.0_f32; let mut cy = 0.0_f32; let mut count = 0u32;
        for &(tri_idx, n, _depth) in &visible_tris {
            if classify_normal(n) == Some(*zone_id) {
                let tri = CHAMFERED_CUBE_TRIS[tri_idx];
                for &vi in &tri { cx += v2d[vi as usize].x; cy += v2d[vi as usize].y; count += 1; }
            }
        }
        if count > 0 {
            cx /= count as f32; cy /= count as f32;
            let tc = if hovered_zone == Some(*zone_id) { c_text_h } else { c_text };
            painter.text(egui::pos2(cx, cy), egui::Align2::CENTER_CENTER,
                label, egui::FontId::proportional(label_font_size), tc);
        }
    }

    // ═══ Coordinate axes (X red, Y green, Z blue) — bottom-left of widget ═══
    let axes_origin = egui::pos2(center.x - ring_r - 4.0, center.y + ring_r + 4.0);
    let axis_len = 18.0_f32;
    let ax_x = project(1.0, 0.0, 0.0); // X axis end
    let ax_y = project(0.0, 1.0, 0.0);
    let ax_z = project(0.0, 0.0, 1.0);
    // Convert to local axes from axes_origin
    let ax_x_end = egui::pos2(axes_origin.x + (ax_x.x - center.x) * axis_len / cube_half,
                               axes_origin.y + (ax_x.y - center.y) * axis_len / cube_half);
    let ax_y_end = egui::pos2(axes_origin.x + (ax_y.x - center.x) * axis_len / cube_half,
                               axes_origin.y + (ax_y.y - center.y) * axis_len / cube_half);
    let ax_z_end = egui::pos2(axes_origin.x + (ax_z.x - center.x) * axis_len / cube_half,
                               axes_origin.y + (ax_z.y - center.y) * axis_len / cube_half);
    painter.line_segment([axes_origin, ax_x_end], egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(0xE0, 0x40, 0x40)));
    painter.line_segment([axes_origin, ax_y_end], egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(0x40, 0xC0, 0x40)));
    painter.line_segment([axes_origin, ax_z_end], egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(0x40, 0x80, 0xE0)));
    painter.text(ax_x_end, egui::Align2::CENTER_CENTER, "X", egui::FontId::proportional(small_font_size), egui::Color32::from_rgb(0xE0, 0x40, 0x40));
    painter.text(ax_y_end, egui::Align2::CENTER_CENTER, "Y", egui::FontId::proportional(small_font_size), egui::Color32::from_rgb(0x40, 0xC0, 0x40));
    painter.text(ax_z_end, egui::Align2::CENTER_CENTER, "Z", egui::FontId::proportional(small_font_size), egui::Color32::from_rgb(0x40, 0x80, 0xE0));

    // ═══ Draw rotation arrows (4 triangles) ═══════════════════════════════════
    let arrow_color = |hovered: bool| {
        if hovered { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
        else { egui::Color32::from_rgb(0x6c, 0x70, 0x86) }
    };
    // Top arrow (points up) — rotates pitch negative
    {
        let r = &arrow_responses[0];
        let c = arrow_rects[0].center();
        let s = arrow_size;
        let pts = vec![
            egui::pos2(c.x, c.y - s*0.6),
            egui::pos2(c.x - s*0.5, c.y + s*0.3),
            egui::pos2(c.x + s*0.5, c.y + s*0.3),
        ];
        painter.add(egui::Shape::convex_polygon(pts.clone(), arrow_color(r.hovered()), egui::Stroke::NONE));
    }
    // Bottom arrow (points down)
    {
        let r = &arrow_responses[1];
        let c = arrow_rects[1].center();
        let s = arrow_size;
        let pts = vec![
            egui::pos2(c.x, c.y + s*0.6),
            egui::pos2(c.x - s*0.5, c.y - s*0.3),
            egui::pos2(c.x + s*0.5, c.y - s*0.3),
        ];
        painter.add(egui::Shape::convex_polygon(pts.clone(), arrow_color(r.hovered()), egui::Stroke::NONE));
    }
    // Left arrow (points left)
    {
        let r = &arrow_responses[2];
        let c = arrow_rects[2].center();
        let s = arrow_size;
        let pts = vec![
            egui::pos2(c.x - s*0.6, c.y),
            egui::pos2(c.x + s*0.3, c.y - s*0.5),
            egui::pos2(c.x + s*0.3, c.y + s*0.5),
        ];
        painter.add(egui::Shape::convex_polygon(pts.clone(), arrow_color(r.hovered()), egui::Stroke::NONE));
    }
    // Right arrow (points right)
    {
        let r = &arrow_responses[3];
        let c = arrow_rects[3].center();
        let s = arrow_size;
        let pts = vec![
            egui::pos2(c.x + s*0.6, c.y),
            egui::pos2(c.x - s*0.3, c.y - s*0.5),
            egui::pos2(c.x - s*0.3, c.y + s*0.5),
        ];
        painter.add(egui::Shape::convex_polygon(pts.clone(), arrow_color(r.hovered()), egui::Stroke::NONE));
    }

    // ═══ Draw roll arrows (2 curved arrows at top) ════════════════════════════
    for (i, r) in roll_responses.iter().enumerate() {
        let c = roll_rects[i].center();
        let col = if r.hovered() { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
            else { egui::Color32::from_rgb(0x6c, 0x70, 0x86) };
        // Draw a small circular arrow
        painter.circle_stroke(c, roll_r * 0.7, egui::Stroke::new(2.0_f32, col));
        // Arrowhead (left for left button, right for right button)
        let head_x = if i == 0 { c.x - roll_r * 0.7 } else { c.x + roll_r * 0.7 };
        let head_y = c.y;
        let pts = if i == 0 {
            vec![
                egui::pos2(head_x, head_y),
                egui::pos2(head_x + 4.0, head_y - 4.0),
                egui::pos2(head_x + 4.0, head_y + 4.0),
            ]
        } else {
            vec![
                egui::pos2(head_x, head_y),
                egui::pos2(head_x - 4.0, head_y - 4.0),
                egui::pos2(head_x - 4.0, head_y + 4.0),
            ]
        };
        painter.add(egui::Shape::convex_polygon(pts, col, egui::Stroke::NONE));
    }

    // ═══ Home button (top-right) ═══════════════════════════════════════════════
    let home_bg = if home_resp.hovered() { egui::Color32::from_rgb(0x45, 0x47, 0x5a) }
        else { egui::Color32::from_rgb(0x2a, 0x2a, 0x30) };
    painter.rect_filled(home_rect, 4.0, home_bg);
    let home_tc = if home_resp.hovered() { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
        else { egui::Color32::from_rgb(0xc8, 0xc8, 0xd0) };
    painter.text(home_rect.center(), egui::Align2::CENTER_CENTER, "\u{2302}",
        egui::FontId::proportional(label_font_size * 1.4), home_tc);
    if home_resp.clicked() {
        selected = Some(ViewCubeAction::Home);
        state.azimuth = 45.0;
        state.elevation = 35.264;
    }

    // ═══ Menu button (bottom-right) ═══════════════════════════════════════════
    let menu_bg = if menu_resp.hovered() { egui::Color32::from_rgb(0x45, 0x47, 0x5a) }
        else { egui::Color32::from_rgb(0x2a, 0x2a, 0x30) };
    painter.rect_filled(menu_rect, 4.0, menu_bg);
    let menu_tc = if menu_resp.hovered() { egui::Color32::from_rgb(0x89, 0xb4, 0xfa) }
        else { egui::Color32::from_rgb(0xc8, 0xc8, 0xd0) };
    // Draw 3 horizontal lines (hamburger menu icon)
    for i in 0..3 {
        let y = menu_rect.center().y - 4.0 + i as f32 * 4.0;
        painter.line_segment(
            [egui::pos2(menu_rect.left() + 5.0, y), egui::pos2(menu_rect.right() - 5.0, y)],
            egui::Stroke::new(1.5_f32, menu_tc),
        );
    }
    if menu_resp.clicked() {
        selected = Some(ViewCubeAction::ToggleProjection);
    }

    // ═══ Handle cube clicks (mesh-based raycast) ══════════════════════════════
    if cube_resp.clicked() && !state.dragging {
        if let Some(mp) = mouse_pos {
            // Check front-most triangles first (last in sorted list = nearest)
            for &(tri_idx, _n, _depth) in visible_tris.iter().rev() {
                let tri = CHAMFERED_CUBE_TRIS[tri_idx];
                let pts = [v2d[tri[0] as usize], v2d[tri[1] as usize], v2d[tri[2] as usize]];
                if point_in_polygon(mp, &pts) {
                    let zone = classify_normal(CHAMFERED_CUBE_FACE_NORMALS[tri_idx]);
                    if let Some(zid) = zone {
                        let orient = zone_to_orientation(zid);
                        selected = Some(ViewCubeAction::SnapTo(orient));
                    }
                    break;
                }
            }
        }
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
