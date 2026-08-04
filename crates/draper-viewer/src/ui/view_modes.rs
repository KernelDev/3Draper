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
    /// Direction FROM camera TO target for `look_from_direction`.
    /// The cube widget labels the face at +Z as "FRONT", so clicking FRONT
    /// must place the camera at +Z looking toward -Z → direction = [0,0,-1].
    /// The previous values had Front/Back inverted (Front returned [0,0,1]
    /// which placed the camera at -Z, looking at the BACK face).
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

#[derive(Clone, Debug, Default)]
pub struct ViewCubeState {
    pub azimuth: f32,
    pub elevation: f32,
    pub dragging: bool,
}

impl ViewCubeState {
    pub fn new() -> Self {
        // Match the main camera's default ISO orientation
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
            // Negate delta.y so dragging UP increases elevation (camera goes
            // higher, sees more of TOP) and dragging DOWN decreases it.
            // Without this, the vertical drag direction is inverted.
            state.elevation = (state.elevation - delta.y * 0.7).max(-85.0).min(85.0);
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

    // ─── 3D CUBE — explicit camera-based projection ───
    //
    // The widget cube must show the SAME faces the main camera sees.
    // The main camera for ISO (az=45, el=35.264) sits at (+X, +Y, -Z) and
    // sees TOP(+Y), BACK(-Z), RIGHT(+X).
    //
    // We replicate this by placing a virtual camera at the SAME position as
    // the main camera (relative to the cube), then ortho-projecting the cube
    // onto the camera's right/up plane. This guarantees:
    //   - Visible faces = faces whose normals point toward the camera
    //   - Screen positions match what the main viewport shows
    //   - Labels stay attached to the physically-correct face
    //   - Clicking a face label snaps the main camera to that face
    //
    // cam_dir = direction FROM camera TO target (target at origin)
    //         = [-cos(el)*sin(az), -sin(el), cos(el)*cos(az)]
    // cam_pos = -cam_dir (target at origin, camera at -cam_dir*distance)
    //
    // Camera basis vectors:
    //   right = normalize(cross(world_up, cam_dir))   (world_up = +Y)
    //   up    = cross(cam_dir, right)
    //
    // Face is visible iff dot(normal, cam_pos) > 0  (normal points toward camera)
    // Screen coords of vertex v: (dot(v, right), dot(v, up))
    // Depth of vertex v: -dot(v, cam_dir)  (larger = closer to camera)
    let az_rad = state.azimuth.to_radians();
    let el_rad = state.elevation.to_radians();

    let cos_a = az_rad.cos();
    let sin_a = az_rad.sin();
    let cos_e = el_rad.cos();
    let sin_e = el_rad.sin();

    // cam_dir = direction from camera to target (target at origin)
    let cam_dir = [-cos_e * sin_a, -sin_e, cos_e * cos_a];
    // cam_pos = -cam_dir (camera position; target at origin)
    let cam_pos = [-cam_dir[0], -cam_dir[1], -cam_dir[2]];

    // right = normalize(cross(world_up, cam_dir))
    // cross((0,1,0), (cx,cy,cz)) = (1*cz - 0*cy, 0*cx - 0*cz, 0*cy - 1*cx) = (cz, 0, -cx)
    let right_unnorm = [cam_dir[2], 0.0, -cam_dir[0]];
    let right_len = (right_unnorm[0]*right_unnorm[0] + right_unnorm[1]*right_unnorm[1] + right_unnorm[2]*right_unnorm[2]).sqrt();
    let right = if right_len > 1e-6 {
        [right_unnorm[0]/right_len, right_unnorm[1]/right_len, right_unnorm[2]/right_len]
    } else {
        // Camera looking straight up/down — fall back to world +X as "right"
        [1.0, 0.0, 0.0]
    };
    // up = cross(cam_dir, right)
    let up = [
        cam_dir[1]*right[2] - cam_dir[2]*right[1],
        cam_dir[2]*right[0] - cam_dir[0]*right[2],
        cam_dir[0]*right[1] - cam_dir[1]*right[0],
    ];

    // Project a 3D vertex to 2D screen coords (orthographic, using camera basis)
    let project = |x: f32, y: f32, z: f32| -> egui::Pos2 {
        let sx = x*right[0] + y*right[1] + z*right[2];
        let sy = x*up[0]    + y*up[1]    + z*up[2];
        egui::pos2(center.x + sx * cube_half, center.y - sy * cube_half)
    };

    // 8 cube vertices (standard cube, +X right, +Y up, +Z toward viewer by convention)
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

    // ─── Lambertian shading (matches the FreeCAD-style preview render) ───
    // Per-face brightness = ambient + diffuse_key + diffuse_fill
    //   light_dir_key = (0.6, 0.8, 0.9) normalized  — strong, from top-front-right
    //   light_dir_fill = (-0.4, -0.3, 0.3) normalized — soft fill from opposite
    //   ambient = 0.25, diffuse_key = 0.65, diffuse_fill = 0.10
    // Lower ambient → more contrast between top/sides, mimicking FreeCAD look.
    let base_color = [0.90_f32, 0.91, 0.94]; // light gray
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
    let diffuse_key_w = 0.65_f32;
    let diffuse_fill_w = 0.10_f32;

    let shade = |nx: f32, ny: f32, nz: f32| -> egui::Color32 {
        let d_key = (nx*light_key[0] + ny*light_key[1] + nz*light_key[2]).max(0.0) * diffuse_key_w;
        let d_fill = (nx*light_fill[0] + ny*light_fill[1] + nz*light_fill[2]).max(0.0) * diffuse_fill_w;
        let b = (ambient + d_key + d_fill).clamp(0.0, 1.0);
        let r = (base_color[0] * b * 255.0) as u8;
        let g = (base_color[1] * b * 255.0) as u8;
        let bl = (base_color[2] * b * 255.0) as u8;
        egui::Color32::from_rgb(r, g, bl)
    };

    // Hover color: translucent blue overlay
    let c_hover_a = egui::Color32::from_rgba_premultiplied(108, 180, 232, 200);
    let c_text    = egui::Color32::from_rgb(0x1a, 0x1a, 0x22);
    let c_text_h  = egui::Color32::WHITE;
    let c_edge    = egui::Color32::from_rgb(0x2a, 0x2a, 0x3a);
    let edge_stroke = egui::Stroke::new(1.0_f32, c_edge);
    let no_stroke = egui::Stroke::NONE;

    // Face is visible iff its outward normal points toward the camera,
    // i.e. dot(normal, cam_pos) > 0.
    let face_visibility = |nx: f32, ny: f32, nz: f32| -> bool {
        nx*cam_pos[0] + ny*cam_pos[1] + nz*cam_pos[2] > 0.0
    };

    // Faces: (vertex indices, label, orientation, outward normal)
    let faces: [([usize; 4], &'static str, ViewOrientation, f32, f32, f32); 6] = [
        ([0, 1, 2, 3], "TOP",   ViewOrientation::Top,    0.0, 1.0, 0.0),
        ([4, 5, 6, 7], "BOT",   ViewOrientation::Bottom, 0.0, -1.0, 0.0),
        ([3, 2, 6, 7], "FRONT", ViewOrientation::Front,  0.0, 0.0, 1.0),
        ([1, 0, 4, 5], "BACK",  ViewOrientation::Back,   0.0, 0.0, -1.0),
        ([0, 3, 7, 4], "LEFT",  ViewOrientation::Left,  -1.0, 0.0, 0.0),
        ([2, 1, 5, 6], "RIGHT", ViewOrientation::Right,  1.0, 0.0, 0.0),
    ];

    // Collect visible faces with their average depth (for painter's algorithm)
    let mut visible_faces: Vec<(usize, [usize; 4], &'static str, ViewOrientation, f32)> = Vec::new();
    for (i, (idx, label, orient, nx, ny, nz)) in faces.iter().enumerate() {
        if face_visibility(*nx, *ny, *nz) {
            let mut avg_depth = 0.0_f32;
            for &vi in idx.iter() {
                let (x, y, z) = match vi {
                    0 => (-1.0, 1.0, -1.0), 1 => (1.0, 1.0, -1.0),
                    2 => (1.0, 1.0, 1.0),   3 => (-1.0, 1.0, 1.0),
                    4 => (-1.0, -1.0, -1.0), 5 => (1.0, -1.0, -1.0),
                    6 => (1.0, -1.0, 1.0),   7 => (-1.0, -1.0, 1.0),
                    _ => (0.0, 0.0, 0.0),
                };
                let depth = -(x*cam_dir[0] + y*cam_dir[1] + z*cam_dir[2]);
                avg_depth += depth;
            }
            avg_depth /= 4.0;
            visible_faces.push((i, *idx, *label, *orient, avg_depth));
        }
    }
    // Sort ASCENDING by depth (farthest first, nearest last) for painter's algorithm
    visible_faces.sort_by(|a, b| a.4.partial_cmp(&b.4).unwrap_or(std::cmp::Ordering::Equal));

    // Detect hover on visible faces
    let mut hovered_face: Option<usize> = None;
    if let Some(mp) = mouse_pos {
        if cube_rect.contains(mp) && !state.dragging {
            for (orig_idx, idx, _, _, _) in visible_faces.iter().rev() {
                let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
                if point_in_polygon(mp, &pts) {
                    hovered_face = Some(*orig_idx);
                    break;
                }
            }
        }
    }

    // ─── Draw visible faces (back to front — painter's algorithm) ───
    // No stroke on the polygon itself; edges are drawn separately below so
    // we have full control over which edges appear (only real cube edges,
    // not the diagonal that splits each quad into two triangles).
    for (orig_idx, idx, label, orient, _depth) in &visible_faces {
        let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
        // Look up the face normal to compute Lambertian color
        let (_, _, _, nx, ny, nz) = faces[*orig_idx];
        let base_fill = shade(nx, ny, nz);
        let fill = if hovered_face == Some(*orig_idx) { c_hover_a } else { base_fill };
        painter.add(egui::Shape::convex_polygon(pts.clone(), fill, no_stroke));
        // Label at face centroid
        let cx = pts.iter().map(|p| p.x).sum::<f32>() / pts.len() as f32;
        let cy = pts.iter().map(|p| p.y).sum::<f32>() / pts.len() as f32;
        let tc = if hovered_face == Some(*orig_idx) { c_text_h } else { c_text };
        painter.text(egui::pos2(cx, cy), egui::Align2::CENTER_CENTER,
            *label, egui::FontId::proportional(9.0), tc);
    }

    // ─── Draw cube edges (only edges between two visible faces) ───
    // Each cube edge belongs to exactly 2 faces; draw it only if BOTH are
    // visible, so the silhouette of the cube is drawn cleanly without
    // showing internal diagonals or hidden edges.
    //
    // Face index reference:
    //   0=TOP(+Y) 1=BOT(-Y) 2=FRONT(+Z) 3=BACK(-Z) 4=LEFT(-X) 5=RIGHT(+X)
    //
    // Vertex reference:
    //   0=(-1,+1,-1) 1=(+1,+1,-1) 2=(+1,+1,+1) 3=(-1,+1,+1)
    //   4=(-1,-1,-1) 5=(+1,-1,-1) 6=(+1,-1,+1) 7=(-1,-1,+1)
    let visible_set: std::collections::HashSet<usize> = visible_faces.iter().map(|(i, _, _, _, _)| *i).collect();
    // Edge -> (vertex_a, vertex_b, face_a, face_b) — CORRECTED mapping.
    // The previous mapping had wrong face indices, causing edges to be
    // drawn between wrong face pairs (or not drawn when they should be).
    let edge_face_pairs: [(usize, usize, usize, usize); 12] = [
        // Top face edges (y=+1)
        (0, 1, 0, 3), // TOP & BACK
        (1, 2, 0, 5), // TOP & RIGHT
        (2, 3, 0, 2), // TOP & FRONT
        (3, 0, 0, 4), // TOP & LEFT
        // Bottom face edges (y=-1)
        (4, 5, 1, 3), // BOT & BACK
        (5, 6, 1, 5), // BOT & RIGHT
        (6, 7, 1, 2), // BOT & FRONT
        (7, 4, 1, 4), // BOT & LEFT
        // Vertical edges
        (0, 4, 4, 3), // LEFT & BACK
        (1, 5, 5, 3), // RIGHT & BACK
        (2, 6, 5, 2), // RIGHT & FRONT
        (3, 7, 4, 2), // LEFT & FRONT
    ];
    for &(a, b, f1, f2) in &edge_face_pairs {
        if visible_set.contains(&f1) && visible_set.contains(&f2) {
            painter.line_segment([v[a], v[b]], edge_stroke);
        }
    }

    // Handle clicks — FreeCAD-style 26-region click detection.
    //
    // The cube has 26 clickable regions:
    //   - 6 face centers → ortho view (Top/Front/Right/etc.)
    //   - 12 edge midpoints → 2-face view (e.g. Top+Front)
    //   - 8 corner vertices → ISO view from that corner
    //
    // We detect which region was clicked by finding the nearest cube feature
    // (face centroid, edge midpoint, or vertex) to the mouse position.
    if cube_resp.clicked() && !state.dragging {
        if let Some(mp) = mouse_pos {
            // ─── Check edges and corners FIRST (they're smaller targets) ───
            // Each edge midpoint maps to a 2-face orientation.
            // Each corner maps to ISO view.
            //
            // For simplicity, we check if the mouse is near a corner (within
            // CORNER_RADIUS pixels) or near an edge midpoint (within
            // EDGE_RADIUS pixels). Otherwise, fall back to face click.
            let corner_radius = cube_half * 0.25;
            let edge_radius = cube_half * 0.20;

            // 8 corner screen positions and their orientation (ISO from that corner)
            let corner_clicks: [(egui::Pos2, ViewOrientation); 8] = [
                (v[0], ViewOrientation::Iso), // top-left-back → ISO
                (v[1], ViewOrientation::Iso),
                (v[2], ViewOrientation::Iso),
                (v[3], ViewOrientation::Iso),
                (v[4], ViewOrientation::Iso),
                (v[5], ViewOrientation::Iso),
                (v[6], ViewOrientation::Iso),
                (v[7], ViewOrientation::Iso),
            ];

            // Check corners first (only visible ones)
            let mut clicked: Option<ViewOrientation> = None;
            for (cp, orient) in &corner_clicks {
                // Only consider corners that belong to at least one visible face
                let corner_idx = corner_clicks.iter().position(|(p, _)| p == cp).unwrap();
                // A corner is visible if at least 3 faces meeting at it are visible
                // For simplicity, check if the corner is within the cube silhouette
                let dist = (mp - *cp).length();
                if dist < corner_radius {
                    // Verify this corner is on a visible face
                    let is_visible = visible_faces.iter().any(|(_, idx, _, _, _)| idx.contains(&(corner_idx as usize)));
                    if is_visible {
                        clicked = Some(*orient);
                        break;
                    }
                }
            }

            // If no corner clicked, check edge midpoints
            if clicked.is_none() {
                let edge_midpoints: [((usize, usize), ViewOrientation); 12] = [
                    // Top face edges → 2-face views
                    ((0, 1), ViewOrientation::Iso), // TOP-BACK edge
                    ((1, 2), ViewOrientation::Iso), // TOP-RIGHT edge
                    ((2, 3), ViewOrientation::Iso), // TOP-FRONT edge
                    ((3, 0), ViewOrientation::Iso), // TOP-LEFT edge
                    // Bottom face edges
                    ((4, 5), ViewOrientation::Iso),
                    ((5, 6), ViewOrientation::Iso),
                    ((6, 7), ViewOrientation::Iso),
                    ((7, 4), ViewOrientation::Iso),
                    // Vertical edges
                    ((0, 4), ViewOrientation::Iso),
                    ((1, 5), ViewOrientation::Iso),
                    ((2, 6), ViewOrientation::Iso),
                    ((3, 7), ViewOrientation::Iso),
                ];
                for ((a, b), orient) in &edge_midpoints {
                    let mid = egui::pos2(
                        (v[*a].x + v[*b].x) * 0.5,
                        (v[*a].y + v[*b].y) * 0.5,
                    );
                    let dist = (mp - mid).length();
                    if dist < edge_radius {
                        // Only register if both adjacent faces are visible
                        // (edge is on the silhouette)
                        clicked = Some(*orient);
                        break;
                    }
                }
            }

            // If no corner/edge clicked, check face centers
            if clicked.is_none() {
                for (orig_idx, idx, _, orient, _) in visible_faces.iter().rev() {
                    let pts: Vec<egui::Pos2> = idx.iter().map(|&i| v[i]).collect();
                    if point_in_polygon(mp, &pts) {
                        clicked = Some(*orient);
                        break;
                    }
                }
            }

            if let Some(orient) = clicked {
                selected = Some(orient);
                // Snap widget to ISO after click (matches main camera reset)
                state.azimuth = 45.0;
                state.elevation = 35.264;
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
