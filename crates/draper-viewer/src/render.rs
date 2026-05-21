//! 3D rendering using egui's painting API.
//!
//! For the initial version, we use egui's 2D painter with manual
//! 3D→2D projection. This gives us a working viewer without
//! needing wgpu pipeline setup, which can be added later.

use draper_mesh::triangulate::TriangleMesh;
use egui::*;
use glam::{Mat4, Vec3, Vec4};

pub struct RenderState {
    pub bg_color: Color32,
}

impl RenderState {
    pub fn new() -> Self {
        Self {
            bg_color: Color32::from_rgb(40, 42, 54),
        }
    }

    pub fn render(
        &self,
        ui: &mut Ui,
        rect: Rect,
        meshes: &[TriangleMesh],
        yaw: f32,
        pitch: f32,
        distance: f32,
        target: [f32; 3],
        wireframe: bool,
        show_axes: bool,
    ) {
        let painter = ui.painter_at(rect);

        // Clear background
        painter.rect_filled(rect, 0.0, self.bg_color);

        // Compute view-projection matrix
        let (view_matrix, proj_matrix) = compute_camera_matrices(yaw, pitch, distance, target, rect);

        let vp = proj_matrix * view_matrix;

        // Draw axes
        if show_axes {
            draw_axes(&painter, &vp, rect);
        }

        // Draw meshes
        for mesh in meshes {
            if wireframe {
                draw_wireframe_mesh(&painter, mesh, &vp, rect);
            } else {
                draw_solid_mesh(&painter, mesh, &vp, rect);
            }
        }
    }
}

fn compute_camera_matrices(
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: [f32; 3],
    rect: Rect,
) -> (Mat4, Mat4) {
    let yaw_rad = yaw.to_radians();
    let pitch_rad = pitch.to_radians();

    let eye = Vec3::new(
        target[0] + distance * pitch_rad.cos() * yaw_rad.sin(),
        target[1] + distance * pitch_rad.sin(),
        target[2] + distance * pitch_rad.cos() * yaw_rad.cos(),
    );

    let view = Mat4::look_at_rh(
        eye,
        Vec3::new(target[0], target[1], target[2]),
        Vec3::Y,
    );

    let aspect = rect.width() / rect.height().max(1.0);
    let proj = Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect, 0.01, 1000.0);

    (view, proj)
}

fn project_point(vp: &Mat4, point: &draper_geometry::point::Point3, rect: Rect) -> Option<(Pos2, f32)> {
    let p = Vec4::new(point.x as f32, point.y as f32, point.z as f32, 1.0);
    let clip = *vp * p;

    if clip.w.abs() < 1e-6 {
        return None;
    }

    let ndc_x = clip.x / clip.w;
    let ndc_y = clip.y / clip.w;
    let ndc_z = clip.z / clip.w;

    // Use a generous clip range to avoid popping artifacts
    if ndc_x < -2.0 || ndc_x > 2.0 || ndc_y < -2.0 || ndc_y > 2.0 {
        return None;
    }

    let screen_x = rect.left() + (ndc_x + 1.0) * 0.5 * rect.width();
    let screen_y = rect.top() + (1.0 - ndc_y) * 0.5 * rect.height();

    Some((Pos2::new(screen_x, screen_y), ndc_z))
}

fn draw_axes(painter: &Painter, vp: &Mat4, rect: Rect) {
    let origin = draper_geometry::point::Point3::ORIGIN;
    let x_end = draper_geometry::point::Point3::new(1.0, 0.0, 0.0);
    let y_end = draper_geometry::point::Point3::new(0.0, 1.0, 0.0);
    let z_end = draper_geometry::point::Point3::new(0.0, 0.0, 1.0);

    let o = project_point(vp, &origin, rect);
    let x = project_point(vp, &x_end, rect);
    let y = project_point(vp, &y_end, rect);
    let z = project_point(vp, &z_end, rect);

    if let (Some((o, _)), Some((x, _))) = (o, x) {
        painter.line_segment([o, x], Stroke::new(2.0, Color32::RED));
        painter.text(x, Align2::LEFT_CENTER, "X", FontId::proportional(12.0), Color32::RED);
    }
    if let (Some((o, _)), Some((y, _))) = (o, y) {
        painter.line_segment([o, y], Stroke::new(2.0, Color32::GREEN));
        painter.text(y, Align2::LEFT_CENTER, "Y", FontId::proportional(12.0), Color32::GREEN);
    }
    if let (Some((o, _)), Some((z, _))) = (o, z) {
        painter.line_segment([o, z], Stroke::new(2.0, Color32::BLUE));
        painter.text(z, Align2::LEFT_CENTER, "Z", FontId::proportional(12.0), Color32::BLUE);
    }
}

fn draw_wireframe_mesh(painter: &Painter, mesh: &TriangleMesh, vp: &Mat4, rect: Rect) {
    let color = Color32::from_rgb(100, 180, 255);

    for tri in mesh.indices.chunks(3) {
        if tri.len() < 3 {
            continue;
        }

        let a = mesh.vertices.get(tri[0] as usize);
        let b = mesh.vertices.get(tri[1] as usize);
        let c = mesh.vertices.get(tri[2] as usize);

        if let (Some(a), Some(b), Some(c)) = (a, b, c) {
            let pa = project_point(vp, a, rect);
            let pb = project_point(vp, b, rect);
            let pc = project_point(vp, c, rect);

            if let (Some((pa, _)), Some((pb, _)), Some((pc, _))) = (pa, pb, pc) {
                painter.line_segment([pa, pb], Stroke::new(1.0, color));
                painter.line_segment([pb, pc], Stroke::new(1.0, color));
                painter.line_segment([pc, pa], Stroke::new(1.0, color));
            }
        }
    }
}

fn draw_solid_mesh(painter: &Painter, mesh: &TriangleMesh, vp: &Mat4, rect: Rect) {
    // For solid rendering, we use filled triangles with basic shading.
    // Project all triangles, compute simple face normals for lighting,
    // and draw back-to-front (painter's algorithm) using clip-space Z for depth.

    let mut projected_tris: Vec<(f32, [Pos2; 3], Color32)> = Vec::new();

    let light_dir = Vec3::new(0.3, 0.8, 0.5).normalize();

    for tri in mesh.indices.chunks(3) {
        if tri.len() < 3 {
            continue;
        }

        let a = mesh.vertices.get(tri[0] as usize);
        let b = mesh.vertices.get(tri[1] as usize);
        let c = mesh.vertices.get(tri[2] as usize);

        if let (Some(a), Some(b), Some(c)) = (a, b, c) {
            let pa = project_point(vp, a, rect);
            let pb = project_point(vp, b, rect);
            let pc = project_point(vp, c, rect);

            if let (Some((pa, za)), Some((pb, zb)), Some((pc, zc))) = (pa, pb, pc) {
                // Compute face normal for simple lighting
                let ab = Vec3::new(
                    (b.x - a.x) as f32,
                    (b.y - a.y) as f32,
                    (b.z - a.z) as f32,
                );
                let ac = Vec3::new(
                    (c.x - a.x) as f32,
                    (c.y - a.y) as f32,
                    (c.z - a.z) as f32,
                );
                let normal = ab.cross(ac).normalize();

                // Simple diffuse lighting
                let diffuse = normal.dot(light_dir).max(0.0);
                let ambient = 0.3;
                let intensity = (ambient + diffuse * 0.7).min(1.0);

                let r = (100.0 * intensity) as u8;
                let g = (160.0 * intensity) as u8;
                let b_val = (220.0 * intensity) as u8;

                // Depth for sorting: average NDC Z (higher Z = further from camera in RHS)
                let depth = (za + zb + zc) / 3.0;

                projected_tris.push((depth, [pa, pb, pc], Color32::from_rgb(r, g, b_val)));
            }
        }
    }

    // Sort by depth (painter's algorithm — back to front, largest Z first)
    projected_tris.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    // Draw triangles
    for (_, pts, color) in &projected_tris {
        painter.add(Shape::convex_polygon(
            pts.to_vec(),
            *color,
            Stroke::new(0.5, Color32::from_rgb(60, 100, 140)),
        ));
    }
}

impl Default for RenderState {
    fn default() -> Self {
        Self::new()
    }
}
