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

pub fn render_view_cube_in_viewport(
    ui: &mut egui::Ui,
    viewport_rect: &egui::Rect,
    state: &mut ViewCubeState,
) -> Option<ViewCubeAction> {
    let mut selected: Option<ViewCubeAction> = None;

    // ═══ Layout ═══════════════════════════════════════════════════════════════
    let ring_r = 60.0_f32;          // compass disc radius
    let margin = 14.0_f32;
    let cube_half = 28.0_f32;       // cube edge half-length (orthographic, fixed scale)
    let chamfer = 0.18_f32;         // chamfer size as fraction of cube_half
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

    // Handle drag on cube/ring to rotate the widget
    let drag_resp = if cube_resp.dragged_by(egui::PointerButton::Primary) { Some(&cube_resp) }
        else if ring_resp.dragged_by(egui::PointerButton::Primary) { Some(&ring_resp) }
        else { None };
    if let Some(dr) = drag_resp {
        let delta = dr.drag_delta();
        if delta.length_sq() > 0.5 {
            state.azimuth += delta.x * 0.7;
            state.elevation = (state.elevation - delta.y * 0.7).max(-85.0).min(85.0);
            state.dragging = true;
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
    let az_rad = state.azimuth.to_radians();
    let el_rad = state.elevation.to_radians();
    let cos_a = az_rad.cos();
    let sin_a = az_rad.sin();
    let cos_e = el_rad.cos();
    let sin_e = el_rad.sin();

    // cam_dir = direction FROM camera TO target (target at origin)
    let cam_dir = [-cos_e * sin_a, -sin_e, cos_e * cos_a];
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

    // ═══ Chamfered cube geometry ═════════════════════════════════════════════
    // A chamfered cube is a cube with each edge cut at 45°.
    // Vertices: 8 corners × 3 edge-vertices per corner = 24 vertices.
    // Faces: 6 octagons (main faces, with corners cut) + 12 rectangles (edge
    // chamfers) + 8 triangles (corner chamfers) = 26 faces.
    //
    // The 26 faces map directly to the 26 interactive zones:
    //   - 6 face octagons → ortho views (TOP/BOT/FRONT/BACK/LEFT/RIGHT)
    //   - 12 edge rectangles → 2-face ISO views
    //   - 8 corner triangles → 3-face ISO views
    let h = 1.0_f32;
    let c = chamfer;

    // Generate 24 vertices: for each of 8 corners, 3 vertices (one per edge direction)
    // Corner (sx,sy,sz) where sx,sy,sz ∈ {-1,+1}:
    //   v_x = (sx*h - sx*c, sy*h, sz*h) — moved along X edge
    //   v_y = (sx*h, sy*h - sy*c, sz*h) — moved along Y edge
    //   v_z = (sx*h, sy*h, sz*h - sz*c) — moved along Z edge
    let mut verts_3d: Vec<[f32; 3]> = Vec::with_capacity(24);
    for sx in [-1.0_f32, 1.0] {
        for sy in [-1.0_f32, 1.0] {
            for sz in [-1.0_f32, 1.0] {
                verts_3d.push([sx*h - sx*c, sy*h, sz*h]);   // along X
                verts_3d.push([sx*h, sy*h - sy*c, sz*h]);   // along Y
                verts_3d.push([sx*h, sy*h, sz*h - sz*c]);   // along Z
            }
        }
    }
    let v2d: Vec<egui::Pos2> = verts_3d.iter().map(|&p| project(p[0], p[1], p[2])).collect();

    // Helper: vertex index for corner (sx,sy,sz) along axis (0=X, 1=Y, 2=Z)
    // Corner order in verts_3d: outer loop sx(-1,+1), middle sy(-1,+1), inner sz(-1,+1)
    // Each corner generates 3 vertices in order X, Y, Z.
    let corner_idx = |sx: f32, sy: f32, sz: f32| -> usize {
        let i = if sx < 0.0 { 0 } else { 1 };
        let j = if sy < 0.0 { 0 } else { 1 };
        let k = if sz < 0.0 { 0 } else { 1 };
        (i * 4 + j * 2 + k) * 3
    };
    // Returns the 3 vertex indices for corner (sx,sy,sz): [v_x, v_y, v_z]
    let corner_verts = |sx: f32, sy: f32, sz: f32| -> [usize; 3] {
        let base = corner_idx(sx, sy, sz);
        [base, base + 1, base + 2]
    };

    // ═══ Build 26 face zones ═════════════════════════════════════════════════
    // Each zone: (vertex indices in CCW order from outside, normal, label_or_None,
    //             ViewOrientation_for_snap)
    #[derive(Clone)]
    struct Zone {
        verts: Vec<usize>,
        normal: [f32; 3],
        label: Option<&'static str>,
        snap: ViewOrientation,
        // Zone ID 0..25 for hover tracking
        id: usize,
    }

    let mut zones: Vec<Zone> = Vec::with_capacity(26);
    let mut next_id = 0usize;

    // ── 6 face octagons ──
    // TOP face (y=+1): 8 vertices from 4 corners' X and Z edge-vertices
    // Corners of top face: (-1,+1,-1), (+1,+1,-1), (+1,+1,+1), (-1,+1,+1)
    // For each corner, we take the X-edge and Z-edge vertices (not Y-edge, which goes down)
    {
        let c1 = corner_verts(-1.0, 1.0, -1.0); // [-1,+1,-1]
        let c2 = corner_verts( 1.0, 1.0, -1.0); // [+1,+1,-1]
        let c3 = corner_verts( 1.0, 1.0,  1.0); // [+1,+1,+1]
        let c4 = corner_verts(-1.0, 1.0,  1.0); // [-1,+1,+1]
        // Octagon vertices in CCW order (viewed from +Y, looking down -Y):
        // c1.x, c1.z, c2.z, c2.x, c3.x, c3.z, c4.z, c4.x
        let oct = vec![c1[0], c1[2], c2[2], c2[0], c3[0], c3[2], c4[2], c4[0]];
        zones.push(Zone { verts: oct, normal: [0.0, 1.0, 0.0], label: Some("TOP"), snap: ViewOrientation::Top, id: next_id }); next_id += 1;
    }
    {
        // BOTTOM (y=-1): mirror of top
        let c1 = corner_verts(-1.0, -1.0, -1.0);
        let c2 = corner_verts( 1.0, -1.0, -1.0);
        let c3 = corner_verts( 1.0, -1.0,  1.0);
        let c4 = corner_verts(-1.0, -1.0,  1.0);
        // CCW from -Y (looking up +Y): reverse of top
        let oct = vec![c1[0], c1[2], c4[2], c4[0], c3[0], c3[2], c2[2], c2[0]];
        zones.push(Zone { verts: oct, normal: [0.0, -1.0, 0.0], label: Some("BOT"), snap: ViewOrientation::Bottom, id: next_id }); next_id += 1;
    }
    {
        // FRONT (z=+1)
        let c1 = corner_verts(-1.0, -1.0, 1.0);
        let c2 = corner_verts( 1.0, -1.0, 1.0);
        let c3 = corner_verts( 1.0,  1.0, 1.0);
        let c4 = corner_verts(-1.0,  1.0, 1.0);
        // CCW from +Z: take X-edge and Y-edge vertices
        let oct = vec![c1[0], c1[1], c2[1], c2[0], c3[0], c3[1], c4[1], c4[0]];
        zones.push(Zone { verts: oct, normal: [0.0, 0.0, 1.0], label: Some("FRONT"), snap: ViewOrientation::Front, id: next_id }); next_id += 1;
    }
    {
        // BACK (z=-1)
        let c1 = corner_verts(-1.0, -1.0, -1.0);
        let c2 = corner_verts( 1.0, -1.0, -1.0);
        let c3 = corner_verts( 1.0,  1.0, -1.0);
        let c4 = corner_verts(-1.0,  1.0, -1.0);
        let oct = vec![c1[0], c1[1], c4[1], c4[0], c3[0], c3[1], c2[1], c2[0]];
        zones.push(Zone { verts: oct, normal: [0.0, 0.0, -1.0], label: Some("BACK"), snap: ViewOrientation::Back, id: next_id }); next_id += 1;
    }
    {
        // LEFT (x=-1)
        let c1 = corner_verts(-1.0, -1.0, -1.0);
        let c2 = corner_verts(-1.0, -1.0,  1.0);
        let c3 = corner_verts(-1.0,  1.0,  1.0);
        let c4 = corner_verts(-1.0,  1.0, -1.0);
        // CCW from -X: take Y-edge and Z-edge vertices
        let oct = vec![c1[1], c1[2], c2[2], c2[1], c3[1], c3[2], c4[2], c4[1]];
        zones.push(Zone { verts: oct, normal: [-1.0, 0.0, 0.0], label: Some("LEFT"), snap: ViewOrientation::Left, id: next_id }); next_id += 1;
    }
    {
        // RIGHT (x=+1)
        let c1 = corner_verts( 1.0, -1.0, -1.0);
        let c2 = corner_verts( 1.0, -1.0,  1.0);
        let c3 = corner_verts( 1.0,  1.0,  1.0);
        let c4 = corner_verts( 1.0,  1.0, -1.0);
        let oct = vec![c1[1], c1[2], c4[2], c4[1], c3[1], c3[2], c2[2], c2[1]];
        zones.push(Zone { verts: oct, normal: [1.0, 0.0, 0.0], label: Some("RIGHT"), snap: ViewOrientation::Right, id: next_id }); next_id += 1;
    }

    // ── 12 edge rectangles ──
    // Each edge is between two adjacent corners along one axis.
    // The chamfer rectangle uses the edge-vertices from both corners.
    // Edge rectangles snap to ISO (2-face view).
    //
    // Helper macro to add an edge zone.
    // (corner_a, corner_b, axis, snap_orient)
    // axis = the axis the edge runs along (0=X, 1=Y, 2=Z)
    // For an X-edge between (sx1,sy,sz) and (sx2,sy,sz) where sx1≠sx2:
    //   the chamfer face is on the side facing the edge direction (perpendicular to Y or Z)
    //   Actually each edge has ONE chamfer face perpendicular to the edge axis,
    //   oriented at 45° between the two adjacent main faces.
    //
    // We'll add all 12 edges:
    // 4 edges along X (top-front, top-back, bot-front, bot-back)
    // 4 edges along Y (left-front, right-front, left-back, right-back)
    // 4 edges along Z (top-left, top-right, bot-left, bot-right)

    // X-direction edges (varying sx, fixed sy, sz)
    for sy in [-1.0_f32, 1.0] {
        for sz in [-1.0_f32, 1.0] {
            // Two corners: (-1, sy, sz) and (+1, sy, sz)
            // The X-edge chamfer face has normal in (0, sy_sign, sz_sign) direction (45°)
            let c_neg = corner_verts(-1.0, sy, sz);
            let c_pos = corner_verts( 1.0, sy, sz);
            // The chamfer face is a rectangle formed by the X-edge vertices of both corners
            // X-edge vertex of corner (sx,sy,sz) is at index 0 of corner_verts()
            // Rectangle: c_neg[0], c_pos[0], c_pos[0], c_neg[0] — but we need 4 distinct verts
            // Actually the chamfer face is the beveled strip between the two main faces.
            // For an X-edge at (sy, sz), the chamfer face normal is (0, sy, sz)/sqrt(2).
            // Vertices: the two X-edge vertices from both corners + the X-edge vertices
            // but that's only 2 unique points (the edge itself).
            //
            // Wait — a chamfered cube's edge face is a RECTANGLE perpendicular to the
            // edge direction, sitting where the original edge was. The 4 vertices of
            // this rectangle are the 4 edge-vertices created by cutting the corner.
            // For X-edge at fixed (sy, sz): the rectangle is in the Y-Z plane (mostly),
            // with vertices at:
            //   (0, sy*h, sz*h - sz*c), (0, sy*h - sy*c, sz*h),  — corner (-1,sy,sz) X-vert and Y-vert... no
            //
            // Let me reconsider. The chamfer cut at corner (sx,sy,sz) creates 3 new vertices:
            //   v_x = (sx*h - sx*c, sy*h, sz*h)  — moved in X
            //   v_y = (sx*h, sy*h - sy*c, sz*h)  — moved in Y
            //   v_z = (sx*h, sy*h, sz*h - sz*c)  — moved in Z
            // The 3 new vertices form a triangle (the corner cut).
            // The edge between v_x and v_y is the new edge between the X-face and Y-face chamfers.
            //
            // For an X-direction edge (between corners (-1,sy,sz) and (+1,sy,sz)):
            // The original edge ran along X at (sy, sz).
            // After chamfering, this edge becomes a RECTANGLE face with 4 vertices:
            //   corner (-1,sy,sz): v_y and v_z (the two edge-vertices NOT on the X-edge)
            //   corner (+1,sy,sz): v_y and v_z
            // Wait, that's not right either. Let me think again.
            //
            // A chamfered cube edge face is the rectangle that replaces the original edge.
            // For the X-edge at (sy, sz):
            //   The original edge had 2 endpoints: (-h, sy*h, sz*h) and (+h, sy*h, sz*h).
            //   After chamfering, each endpoint is split into 3 vertices (one per axis cut).
            //   The edge face uses:
            //     from corner (-1,sy,sz): v_y (-h, sy*h - sy*c, sz*h) and v_z (-h, sy*h, sz*h - sz*c)
            //     from corner (+1,sy,sz): v_y (+h, sy*h - sy*c, sz*h) and v_z (+h, sy*h, sz*h - sz*c)
            //   These 4 points form a rectangle.
            //   The normal of this face is in direction (0, sy, sz) normalized (45° between Y and Z faces).
            let v1 = c_neg[1]; // (-1,sy,sz).v_y
            let v2 = c_neg[2]; // (-1,sy,sz).v_z
            let v3 = c_pos[2]; // (+1,sy,sz).v_z
            let v4 = c_pos[1]; // (+1,sy,sz).v_y
            let nx = 0.0_f32;
            let ny = sy;
            let nz = sz;
            let nlen = (ny*ny + nz*nz).sqrt();
            let normal = [nx, ny/nlen, nz/nlen];
            zones.push(Zone {
                verts: vec![v1, v2, v3, v4],
                normal,
                label: None,
                snap: ViewOrientation::Iso,
                id: next_id,
            });
            next_id += 1;
        }
    }

    // Y-direction edges (fixed sx, sz; varying sy)
    for sx in [-1.0_f32, 1.0] {
        for sz in [-1.0_f32, 1.0] {
            let c_neg = corner_verts(sx, -1.0, sz);
            let c_pos = corner_verts(sx,  1.0, sz);
            // Y-edge face uses v_x and v_z from each corner
            let v1 = c_neg[0]; // v_x
            let v2 = c_neg[2]; // v_z
            let v3 = c_pos[2];
            let v4 = c_pos[0];
            let nx = sx;
            let nz = sz;
            let nlen = (nx*nx + nz*nz).sqrt();
            zones.push(Zone {
                verts: vec![v1, v2, v3, v4],
                normal: [nx/nlen, 0.0, nz/nlen],
                label: None,
                snap: ViewOrientation::Iso,
                id: next_id,
            });
            next_id += 1;
        }
    }

    // Z-direction edges (fixed sx, sy; varying sz)
    for sx in [-1.0_f32, 1.0] {
        for sy in [-1.0_f32, 1.0] {
            let c_neg = corner_verts(sx, sy, -1.0);
            let c_pos = corner_verts(sx, sy,  1.0);
            // Z-edge face uses v_x and v_y from each corner
            let v1 = c_neg[0]; // v_x
            let v2 = c_neg[1]; // v_y
            let v3 = c_pos[1];
            let v4 = c_pos[0];
            let nx = sx;
            let ny = sy;
            let nlen = (nx*nx + ny*ny).sqrt();
            zones.push(Zone {
                verts: vec![v1, v2, v3, v4],
                normal: [nx/nlen, ny/nlen, 0.0],
                label: None,
                snap: ViewOrientation::Iso,
                id: next_id,
            });
            next_id += 1;
        }
    }

    // ── 8 corner triangles ──
    // Each corner triangle uses the 3 edge-vertices (v_x, v_y, v_z) of that corner.
    // Normal is in direction (sx, sy, sz) normalized.
    for sx in [-1.0_f32, 1.0] {
        for sy in [-1.0_f32, 1.0] {
            for sz in [-1.0_f32, 1.0] {
                let cv = corner_verts(sx, sy, sz);
                let nx = sx;
                let ny = sy;
                let nz = sz;
                let nlen = (nx*nx + ny*ny + nz*nz).sqrt();
                zones.push(Zone {
                    verts: vec![cv[0], cv[1], cv[2]],
                    normal: [nx/nlen, ny/nlen, nz/nlen],
                    label: None,
                    snap: ViewOrientation::Iso,
                    id: next_id,
                });
                next_id += 1;
            }
        }
    }

    debug_assert_eq!(zones.len(), 26, "Expected 26 zones, got {}", zones.len());

    // ═══ Visibility test + depth sort (painter's algorithm = own depth buffer) ═
    let face_visible = |n: [f32; 3]| -> bool {
        n[0]*cam_pos[0] + n[1]*cam_pos[1] + n[2]*cam_pos[2] > 0.0
    };

    // Compute average depth for each visible zone (for back-to-front sorting)
    let mut visible_zones: Vec<(usize, &[usize], [f32; 3], Option<&'static str>, ViewOrientation, f32)> = Vec::new();
    for z in &zones {
        if face_visible(z.normal) {
            let mut avg_depth = 0.0_f32;
            for &vi in &z.verts {
                let p = verts_3d[vi];
                let depth = -(p[0]*cam_dir[0] + p[1]*cam_dir[1] + p[2]*cam_dir[2]);
                avg_depth += depth;
            }
            avg_depth /= z.verts.len() as f32;
            visible_zones.push((z.id, &z.verts, z.normal, z.label, z.snap, avg_depth));
        }
    }
    // Sort ASCENDING by depth (farthest first = drawn first)
    visible_zones.sort_by(|a, b| a.5.partial_cmp(&b.5).unwrap_or(std::cmp::Ordering::Equal));

    // ═══ Hover detection (raycast via point-in-polygon on projected zones) ═══
    let mut hovered_zone_id: Option<usize> = None;
    if let Some(mp) = mouse_pos {
        if cube_rect.contains(mp) && !state.dragging {
            // Check nearest zones first (last in sorted list)
            for (zid, vidx, _n, _lbl, _snap, _depth) in visible_zones.iter().rev() {
                let pts: Vec<egui::Pos2> = vidx.iter().map(|&i| v2d[i]).collect();
                if point_in_polygon(mp, &pts) {
                    hovered_zone_id = Some(*zid);
                    break;
                }
            }
        }
    }
    state.hovered_zone = hovered_zone_id;

    // ═══ Lambertian shading ═══════════════════════════════════════════════════
    let base_color = [0.90_f32, 0.91, 0.94];
    let ambient = 0.25_f32;
    let light_key = {
        let v = [0.6_f32, 0.8, 0.9];
        let n = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
        [v[0]/n, v[1]/n, v[2]/n]
    };
    let light_fill = {
        let v = [-0.4_f32, -0.3, 0.3];
        let n = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
        [v[0]/n, v[1]/n, v[2]/n]
    };
    let shade = |n: [f32; 3]| -> egui::Color32 {
        let d_key = (n[0]*light_key[0] + n[1]*light_key[1] + n[2]*light_key[2]).max(0.0) * 0.65;
        let d_fill = (n[0]*light_fill[0] + n[1]*light_fill[1] + n[2]*light_fill[2]).max(0.0) * 0.10;
        let b = (ambient + d_key + d_fill).clamp(0.0, 1.0);
        egui::Color32::from_rgb(
            (base_color[0] * b * 255.0) as u8,
            (base_color[1] * b * 255.0) as u8,
            (base_color[2] * b * 255.0) as u8,
        )
    };
    let c_hover = egui::Color32::from_rgba_premultiplied(108, 180, 232, 220);
    let c_text = egui::Color32::from_rgb(0x1a, 0x1a, 0x22);
    let c_text_h = egui::Color32::WHITE;
    let c_edge = egui::Color32::from_rgb(0x2a, 0x2a, 0x3a);
    let edge_stroke = egui::Stroke::new(1.0_f32, c_edge);

    // ═══ Draw zones back-to-front ═════════════════════════════════════════════
    for (zid, vidx, n, label, _snap, _depth) in &visible_zones {
        let pts: Vec<egui::Pos2> = vidx.iter().map(|&i| v2d[i]).collect();
        let base_fill = shade(*n);
        let fill = if hovered_zone_id == Some(*zid) { c_hover } else { base_fill };
        painter.add(egui::Shape::convex_polygon(pts.clone(), fill, egui::Stroke::NONE));
        // Draw label only on face zones (octagons), not on edge/corner chamfers
        if let Some(lbl) = label {
            let cx = pts.iter().map(|p| p.x).sum::<f32>() / pts.len() as f32;
            let cy = pts.iter().map(|p| p.y).sum::<f32>() / pts.len() as f32;
            let tc = if hovered_zone_id == Some(*zid) { c_text_h } else { c_text };
            painter.text(egui::pos2(cx, cy), egui::Align2::CENTER_CENTER,
                lbl, egui::FontId::proportional(9.0), tc);
        }
        // Draw thin edge around each zone
        for w in pts.windows(2) {
            painter.line_segment([w[0], w[1]], edge_stroke);
        }
        // Close the loop
        if pts.len() >= 2 {
            painter.line_segment([pts[pts.len()-1], pts[0]], edge_stroke);
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
    painter.text(ax_x_end, egui::Align2::CENTER_CENTER, "X", egui::FontId::proportional(8.0), egui::Color32::from_rgb(0xE0, 0x40, 0x40));
    painter.text(ax_y_end, egui::Align2::CENTER_CENTER, "Y", egui::FontId::proportional(8.0), egui::Color32::from_rgb(0x40, 0xC0, 0x40));
    painter.text(ax_z_end, egui::Align2::CENTER_CENTER, "Z", egui::FontId::proportional(8.0), egui::Color32::from_rgb(0x40, 0x80, 0xE0));

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
        egui::FontId::proportional(14.0), home_tc);
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

    // ═══ Handle cube clicks (26-zone raycast) ═════════════════════════════════
    if cube_resp.clicked() && !state.dragging {
        if let Some(mp) = mouse_pos {
            // Check nearest zones first (front-most)
            for (_zid, vidx, _n, _lbl, snap, _depth) in visible_zones.iter().rev() {
                let pts: Vec<egui::Pos2> = vidx.iter().map(|&i| v2d[i]).collect();
                if point_in_polygon(mp, &pts) {
                    selected = Some(ViewCubeAction::SnapTo(*snap));
                    // Snap widget to match main camera after click
                    state.azimuth = 45.0;
                    state.elevation = 35.264;
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
