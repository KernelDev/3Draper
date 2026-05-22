//! Main application state and UI.

use crate::render::MeshRenderer;
use draper_core::engine::{EngineConfig, build_engine};
use draper_core::Document;
use draper_topology::ShapeBuilder;
use draper_mesh::{triangulate_compound, triangulate_solid, TriangleMesh, TriangulationParams};
use eframe::egui;

/// The viewer application.
pub struct ViewerApp {
    /// Current mesh to display.
    mesh: TriangleMesh,
    /// Mesh renderer.
    renderer: Option<MeshRenderer>,
    /// Camera rotation angles.
    rot_x: f32,
    rot_y: f32,
    /// Camera distance.
    zoom: f32,
    /// Pan offset.
    pan_x: f32,
    pan_y: f32,
    /// Show wireframe.
    wireframe: bool,
    /// Status message.
    status: String,
    /// Model info.
    vertex_count: usize,
    triangle_count: usize,
}

impl ViewerApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        // Start with a default engine model
        let doc = build_engine(&EngineConfig::default());
        let mesh = doc.triangulate();

        let vertex_count = mesh.vertex_count();
        let triangle_count = mesh.triangle_count();

        Self {
            mesh,
            renderer: None,
            rot_x: 30.0_f32.to_radians(),
            rot_y: -45.0_f32.to_radians(),
            zoom: 400.0,
            pan_x: 0.0,
            pan_y: 0.0,
            wireframe: false,
            status: "ICE Engine loaded".to_string(),
            vertex_count,
            triangle_count,
        }
    }

    fn load_box(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let params = TriangulationParams::default();
        self.mesh = triangulate_solid(&solid, &params);
        self.update_info("Box loaded");
    }

    fn load_cylinder(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let params = TriangulationParams::default();
        self.mesh = triangulate_solid(&solid, &params);
        self.update_info("Cylinder loaded");
    }

    fn load_sphere(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let params = TriangulationParams::default();
        self.mesh = triangulate_solid(&solid, &params);
        self.update_info("Sphere loaded");
    }

    fn load_engine(&mut self) {
        let doc = build_engine(&EngineConfig::default());
        self.mesh = doc.triangulate();
        self.update_info("ICE Engine loaded");
    }

    fn update_info(&mut self, msg: &str) {
        self.vertex_count = self.mesh.vertex_count();
        self.triangle_count = self.mesh.triangle_count();
        self.status = msg.to_string();
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // === Side panel (controls) ===
        egui::SidePanel::left("controls").show(ctx, |ui| {
            ui.heading("3Draper Viewer");
            ui.separator();

            ui.heading("Primitives");
            if ui.button("Box 100x80x60").clicked() {
                self.load_box();
            }
            if ui.button("Cylinder R=40 H=100").clicked() {
                self.load_cylinder();
            }
            if ui.button("Sphere R=50").clicked() {
                self.load_sphere();
            }

            ui.separator();
            ui.heading("Models");
            if ui.button("ICE Engine (I4)").clicked() {
                self.load_engine();
            }

            ui.separator();
            ui.heading("Display");
            ui.checkbox(&mut self.wireframe, "Wireframe");

            ui.separator();
            ui.heading("Info");
            ui.label(format!("Vertices: {}", self.vertex_count));
            ui.label(format!("Triangles: {}", self.triangle_count));
            ui.label(&self.status);
        });

        // === Central 3D viewport ===
        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, response) = ui.allocate_exact_size(
                ui.available_size(),
                egui::Sense::click_and_drag(),
            );

            // Handle mouse input
            if response.dragged_by(egui::PointerButton::Primary) {
                let delta = response.drag_delta();
                self.rot_y += delta.x * 0.01;
                self.rot_x += delta.y * 0.01;
            }
            if response.dragged_by(egui::PointerButton::Secondary) {
                let delta = response.drag_delta();
                self.pan_x += delta.x;
                self.pan_y += delta.y;
            }
            // Zoom with scroll
            let scroll = ui.input(|i| i.smooth_scroll_delta);
            if scroll.y != 0.0 {
                self.zoom -= scroll.y * 0.5;
                self.zoom = self.zoom.max(10.0).min(5000.0);
            }

            // Render the mesh
            self.render_mesh_painter(ui, rect);
        });
    }
}

impl ViewerApp {
    /// Simple software renderer using egui painting.
    fn render_mesh_painter(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let mesh = &self.mesh;
        if mesh.vertices.is_empty() {
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(30, 30, 40));
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No geometry loaded",
                egui::FontId::proportional(20.0),
                egui::Color32::GRAY,
            );
            return;
        }

        // Compute mesh center for auto-centering
        let (bbox_min, bbox_max) = mesh.bounding_box();
        let cx_model = (bbox_min.x + bbox_max.x) / 2.0;
        let cy_model = (bbox_min.y + bbox_max.y) / 2.0;
        let cz_model = (bbox_min.z + bbox_max.z) / 2.0;

        let scale = self.zoom;
        let cx = rect.center().x + self.pan_x;
        let cy = rect.center().y + self.pan_y;

        // Precompute projected vertices
        let cos_rx = self.rot_x.cos();
        let sin_rx = self.rot_x.sin();
        let cos_ry = self.rot_y.cos();
        let sin_ry = self.rot_y.sin();

        let projected: Vec<(f32, f32, f32)> = mesh.vertices.iter().map(|v| {
            let x = (v.x - cx_model) as f32;
            let y = (v.y - cy_model) as f32;
            let z = (v.z - cz_model) as f32;

            // Rotate around Y axis
            let x1 = x * cos_ry + z * sin_ry;
            let z1 = -x * sin_ry + z * cos_ry;

            // Rotate around X axis
            let y1 = y * cos_rx - z1 * sin_rx;
            let z2 = y * sin_rx + z1 * cos_rx;

            // Project to screen
            let sx = cx + x1 * scale / 100.0;
            let sy = cy - y1 * scale / 100.0;

            (sx, sy, z2)
        }).collect();

        // Background
        ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 30));

        // Draw triangles with simple z-sorting
        let mut triangles_with_depth: Vec<(f32, [u32; 3])> = mesh.triangles.iter().map(|tri| {
            let avg_z = (projected[tri[0] as usize].2
                + projected[tri[1] as usize].2
                + projected[tri[2] as usize].2) / 3.0;
            (avg_z, *tri)
        }).collect();

        // Sort back-to-front (painter's algorithm)
        triangles_with_depth.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        for (_depth, tri) in &triangles_with_depth {
            let p0 = projected[tri[0] as usize];
            let p1 = projected[tri[1] as usize];
            let p2 = projected[tri[2] as usize];

            // Simple directional lighting
            let normal = compute_triangle_normal(p0, p1, p2);
            let light_dir = (0.3_f32, 0.5_f32, 0.8_f32);
            let light_len = (light_dir.0*light_dir.0 + light_dir.1*light_dir.1 + light_dir.2*light_dir.2).sqrt();
            let dot = (normal.0 * light_dir.0 + normal.1 * light_dir.1 + normal.2 * light_dir.2) / light_len;
            let intensity = (dot * 0.5 + 0.5).clamp(0.15, 1.0);

            let base_color = (70.0 * intensity, 130.0 * intensity, 200.0 * intensity);
            let color = egui::Color32::from_rgb(
                base_color.0 as u8,
                base_color.1 as u8,
                base_color.2 as u8,
            );

            let points = vec![
                egui::Pos2::new(p0.0, p0.1),
                egui::Pos2::new(p1.0, p1.1),
                egui::Pos2::new(p2.0, p2.1),
            ];

            ui.painter().add(egui::Shape::convex_polygon(
                points,
                color,
                if self.wireframe {
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(200, 200, 255))
                } else {
                    egui::Stroke::NONE
                },
            ));
        }

        // Draw axes
        let axis_len = 50.0;
        let axes: [(f32, f32, f32, egui::Color32, &str); 3] = [
            (axis_len, 0.0, 0.0, egui::Color32::RED, "X"),
            (0.0, axis_len, 0.0, egui::Color32::GREEN, "Y"),
            (0.0, 0.0, axis_len, egui::Color32::BLUE, "Z"),
        ];
        let origin_x = rect.left() + 50.0;
        let origin_y = rect.bottom() - 50.0;

        for (ax, ay, az, color, label) in axes {
            let x1 = ax * cos_ry + az * sin_ry;
            let z1 = -ax * sin_ry + az * cos_ry;
            let y1 = ay * cos_rx - z1 * sin_rx;

            let end_x = origin_x + x1;
            let end_y = origin_y - y1;

            ui.painter().line_segment(
                [egui::Pos2::new(origin_x, origin_y), egui::Pos2::new(end_x, end_y)],
                egui::Stroke::new(2.0, color),
            );
            ui.painter().text(
                egui::Pos2::new(end_x + 5.0, end_y - 5.0),
                egui::Align2::LEFT_BOTTOM,
                label,
                egui::FontId::proportional(14.0),
                color,
            );
        }
    }
}

fn compute_triangle_normal(p0: (f32, f32, f32), p1: (f32, f32, f32), p2: (f32, f32, f32)) -> (f32, f32, f32) {
    let e1 = (p1.0 - p0.0, p1.1 - p0.1, p1.2 - p0.2);
    let e2 = (p2.0 - p0.0, p2.1 - p0.1, p2.2 - p0.2);
    let nx = e1.1 * e2.2 - e1.2 * e2.1;
    let ny = e1.2 * e2.0 - e1.0 * e2.2;
    let nz = e1.0 * e2.1 - e1.1 * e2.0;
    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    if len > 1e-10 {
        (nx / len, ny / len, nz / len)
    } else {
        (0.0, 0.0, 1.0)
    }
}
