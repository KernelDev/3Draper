//! Main application state and UI.

use std::sync::Arc;
use std::sync::Mutex;

use crate::camera::OrbitCamera;
use crate::renderer::{
    MeshVertex, SceneCallback, SceneResources, SceneUniforms,
    create_scene_resources, update_mesh_buffers, update_uniforms,
};
use draper_core::engine::{EngineConfig, build_engine};
use draper_topology::ShapeBuilder;
use draper_mesh::{triangulate_solid, TriangleMesh, TriangulationParams};
use egui_wgpu::RenderState;
use eframe::egui;

/// Convert TriangleMesh to GPU vertex/index data.
fn mesh_to_gpu_data(mesh: &TriangleMesh) -> (Vec<MeshVertex>, Vec<u32>) {
    // Compute face normals if not present
    let mut mesh = mesh.clone();
    if mesh.face_normals.is_none() {
        mesh.compute_face_normals();
    }

    let normals = mesh.face_normals.as_ref();

    // For proper per-vertex normals, we duplicate vertices per triangle
    // so each vertex has the correct face normal.
    let mut gpu_vertices = Vec::with_capacity(mesh.triangles.len() * 3);
    let mut gpu_indices = Vec::with_capacity(mesh.triangles.len() * 3);

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let normal = normals
            .and_then(|n| n.get(i))
            .map(|n| [n[0] as f32, n[1] as f32, n[2] as f32])
            .unwrap_or([0.0, 0.0, 1.0]);

        let base_idx = gpu_vertices.len() as u32;
        for &idx in tri {
            let v = &mesh.vertices[idx as usize];
            gpu_vertices.push(MeshVertex {
                position: [v.x as f32, v.y as f32, v.z as f32],
                normal,
            });
        }
        gpu_indices.push(base_idx);
        gpu_indices.push(base_idx + 1);
        gpu_indices.push(base_idx + 2);
    }

    (gpu_vertices, gpu_indices)
}

/// Model entry for the scene.
#[derive(Clone, Debug)]
pub struct ModelEntry {
    pub name: String,
    pub vertex_count: usize,
    pub triangle_count: usize,
}

/// Log entry with timestamp.
#[derive(Clone, Debug)]
struct LogEntry {
    time: String,
    message: String,
}

/// The viewer application.
pub struct ViewerApp {
    /// Current mesh to display.
    mesh: TriangleMesh,
    /// GPU resources.
    gpu_resources: Arc<Mutex<Option<SceneResources>>>,
    /// Render state (device, queue, etc).
    render_state: Option<RenderState>,
    /// Orbit camera.
    camera: OrbitCamera,
    /// Show wireframe.
    wireframe: bool,
    /// Model info.
    current_model: ModelEntry,
    /// Whether mesh needs GPU upload.
    mesh_dirty: bool,
    /// Show grid.
    show_grid: bool,
    /// Show axes.
    show_axes: bool,
    /// Log entries.
    log: Vec<LogEntry>,
    /// Auto-scroll log.
    log_auto_scroll: bool,
}

impl ViewerApp {
    fn log(&mut self, msg: &str) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let secs = (now % 3600) / 60;
        let mins = (now % 86400) / 3600;
        let time = format!("{:02}:{:02}:{:02}", (now / 3600) % 24, mins, secs);
        self.log.push(LogEntry {
            time,
            message: msg.to_string(),
        });
        // Keep last 500 entries
        if self.log.len() > 500 {
            self.log.drain(0..self.log.len() - 500);
        }
        self.log_auto_scroll = true;
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let render_state = cc.wgpu_render_state.clone();

        // Start with a default box
        let solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let params = TriangulationParams::default();
        let mesh = triangulate_solid(&solid, &params);

        let current_model = ModelEntry {
            name: "Box 100x100x100".to_string(),
            vertex_count: mesh.vertex_count(),
            triangle_count: mesh.triangle_count(),
        };

        let mut camera = OrbitCamera::new();
        let (bbox_min, bbox_max) = mesh.bounding_box();
        camera.fit_to_bounding_box(
            [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
            [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
        );

        let gpu_resources = Arc::new(Mutex::new(None));

        // Initialize GPU resources if wgpu is available
        if let Some(ref rs) = render_state {
            let (vertices, indices) = mesh_to_gpu_data(&mesh);
            let resources = create_scene_resources(rs, &vertices, &indices);
            *gpu_resources.lock().unwrap() = Some(resources);
        }

        let mut app = Self {
            mesh,
            gpu_resources,
            render_state,
            camera,
            wireframe: false,
            current_model,
            mesh_dirty: false,
            show_grid: true,
            show_axes: true,
            log: Vec::new(),
            log_auto_scroll: true,
        };
        app.log("3Draper Viewer started");
        app.log(&format!("Default model: Box 100x100x100 ({} vertices, {} triangles)",
            app.current_model.vertex_count, app.current_model.triangle_count));
        app
    }

    fn load_mesh(&mut self, mesh: TriangleMesh, name: &str) {
        // Auto-fit camera to new model center
        let (bbox_min, bbox_max) = mesh.bounding_box();
        self.camera.fit_to_bounding_box(
            [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
            [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
        );

        self.current_model = ModelEntry {
            name: name.to_string(),
            vertex_count: mesh.vertex_count(),
            triangle_count: mesh.triangle_count(),
        };
        self.mesh = mesh;
        self.mesh_dirty = true;
        self.log(&format!("Loaded: {} ({} vertices, {} triangles)",
            name, self.current_model.vertex_count, self.current_model.triangle_count));
    }

    fn load_box(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.load_mesh(mesh, "Box 100x80x60");
    }

    fn load_cylinder(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.load_mesh(mesh, "Cylinder R=40 H=100");
    }

    fn load_sphere(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.load_mesh(mesh, "Sphere R=50");
    }

    fn load_cone(&mut self) {
        let radius: f64 = 40.0;
        let height: f64 = 80.0;
        let half_angle = (radius / height).atan();
        let solid = ShapeBuilder::make_cone(radius, height, half_angle);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.load_mesh(mesh, "Cone R=40 H=80");
    }

    fn load_torus(&mut self) {
        let solid = ShapeBuilder::make_torus(40.0, 12.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.load_mesh(mesh, "Torus R=40 r=12");
    }

    fn load_engine(&mut self) {
        let doc = build_engine(&EngineConfig::default());
        let mesh = doc.triangulate();
        self.load_mesh(mesh, "ICE Engine (I4)");
    }

    fn import_stl_file(&mut self, path: &str) {
        match draper_mesh::import_stl_binary(path) {
            Ok(mesh) => {
                let name = std::path::Path::new(path)
                    .file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "STL file".to_string());
                self.load_mesh(mesh, &format!("STL: {}", name));
            }
            Err(e) => {
                self.log(&format!("STL import error: {}", e));
            }
        }
    }

    fn import_step_file(&mut self, path: &str) {
        match draper_step::parse_step_file(path) {
            Ok(step_file) => {
                let name = std::path::Path::new(path)
                    .file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "STEP file".to_string());

                // Count relevant geometry entities
                let mut point_count = 0;
                let mut face_count = 0;
                let mut shell_count = 0;
                let mut brep_count = 0;
                let mut surface_types: std::collections::HashMap<String, usize> = std::collections::HashMap::new();

                for entity in &step_file.entities {
                    match entity.type_name.as_str() {
                        "CARTESIAN_POINT" => point_count += 1,
                        "ADVANCED_FACE" | "FACE_OUTER_BOUND" | "FACE_BOUND" => face_count += 1,
                        "CLOSED_SHELL" | "OPEN_SHELL" => shell_count += 1,
                        "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => brep_count += 1,
                        _ => {
                            if entity.type_name.contains("SURFACE") || entity.type_name.contains("PLANE") {
                                *surface_types.entry(entity.type_name.clone()).or_insert(0) += 1;
                            }
                        }
                    }
                }

                let surface_summary: Vec<String> = surface_types.iter()
                    .map(|(k, v)| format!("{}({})", k, v))
                    .collect();

                self.log(&format!(
                    "STEP parsed: {} — {} entities, {} points, {} faces, {} shells, {} breps",
                    name, step_file.entities.len(), point_count, face_count, shell_count, brep_count
                ));
                if !surface_summary.is_empty() {
                    self.log(&format!("STEP surfaces: {}", surface_summary.join(", ")));
                }

                // Convert STEP to mesh
                match draper_step::step_to_mesh(&step_file) {
                    Ok(mesh) => {
                        let vcount = mesh.vertex_count();
                        let tcount = mesh.triangle_count();
                        self.log(&format!(
                            "STEP converted: {} vertices, {} triangles", vcount, tcount
                        ));
                        self.load_mesh(mesh, &format!("STEP: {}", name));
                    }
                    Err(e) => {
                        self.log(&format!("STEP conversion: {}", e));
                    }
                }
            }
            Err(e) => {
                self.log(&format!("STEP import error: {}", e));
            }
        }
    }

    fn export_stl_binary(&mut self, path: &str) {
        match draper_mesh::stl::write_stl_file(&self.mesh, path, true) {
            Ok(()) => self.log(&format!("Exported STL (binary): {}", path)),
            Err(e) => self.log(&format!("STL export error: {}", e)),
        }
    }

    fn export_stl_ascii(&mut self, path: &str) {
        match draper_mesh::stl::write_stl_file(&self.mesh, path, false) {
            Ok(()) => self.log(&format!("Exported STL (ASCII): {}", path)),
            Err(e) => self.log(&format!("STL export error: {}", e)),
        }
    }

    fn export_step(&mut self, path: &str) {
        // Re-create the solid from the current mesh description
        // For now, export based on the model name — we'll need to store the solid
        // We need to reconstruct a solid from the current mesh for export
        let solid = self.rebuild_current_solid();
        let name = std::path::Path::new(path)
            .file_stem()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| "model".to_string());
        let content = draper_step::export_step(&solid, &name);
        match draper_step::write_step_file(&content, path) {
            Ok(()) => self.log(&format!("Exported STEP: {}", path)),
            Err(e) => self.log(&format!("STEP export error: {}", e)),
        }
    }

    /// Rebuild a Solid from the current model for export purposes.
    /// This is a temporary solution — ideally we'd store the original solid.
    fn rebuild_current_solid(&self) -> draper_topology::Solid {
        use draper_topology::ShapeBuilder;
        match self.current_model.name.as_str() {
            n if n.starts_with("Box") => {
                ShapeBuilder::make_box(100.0, 80.0, 60.0)
            }
            n if n.starts_with("Cylinder") => {
                ShapeBuilder::make_cylinder(40.0, 100.0)
            }
            n if n.starts_with("Sphere") => {
                ShapeBuilder::make_sphere(50.0)
            }
            n if n.starts_with("Cone") => {
                let radius = 40.0_f64;
                let height = 80.0_f64;
                let half_angle = (radius / height).atan();
                ShapeBuilder::make_cone(radius, height, half_angle)
            }
            n if n.starts_with("Torus") => {
                ShapeBuilder::make_torus(40.0, 12.0)
            }
            _ => {
                // Default: create a box
                ShapeBuilder::make_box(100.0, 100.0, 100.0)
            }
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Request repaint for continuous rendering
        ctx.request_repaint();

        // === Top menu bar ===
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Import STL...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STL", &["stl"])
                            .pick_file()
                        {
                            self.import_stl_file(&path.to_string_lossy());
                        }
                        ui.close_menu();
                    }
                    if ui.button("Import STEP...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STEP", &["stp", "step"])
                            .pick_file()
                        {
                            self.import_step_file(&path.to_string_lossy());
                        }
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Export STL (Binary)...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STL", &["stl"])
                            .save_file()
                        {
                            self.export_stl_binary(&path.to_string_lossy());
                        }
                        ui.close_menu();
                    }
                    if ui.button("Export STL (ASCII)...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STL", &["stl"])
                            .save_file()
                        {
                            self.export_stl_ascii(&path.to_string_lossy());
                        }
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Export STEP...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STEP", &["stp", "step"])
                            .save_file()
                        {
                            self.export_step(&path.to_string_lossy());
                        }
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Quit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.wireframe, "Wireframe");
                    ui.checkbox(&mut self.show_axes, "Show axes");
                    ui.checkbox(&mut self.show_grid, "Show grid");
                    ui.separator();
                    if ui.button("Reset Camera").clicked() {
                        let (bbox_min, bbox_max) = self.mesh.bounding_box();
                        self.camera.fit_to_bounding_box(
                            [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                            [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                        );
                    }
                    if ui.button("Top View").clicked() {
                        self.camera.azimuth = 0.0;
                        self.camera.elevation = 89.0_f32.to_radians();
                    }
                    if ui.button("Front View").clicked() {
                        self.camera.azimuth = 0.0;
                        self.camera.elevation = 0.0;
                    }
                    if ui.button("Right View").clicked() {
                        self.camera.azimuth = 90.0_f32.to_radians();
                        self.camera.elevation = 0.0;
                    }
                    if ui.button("Isometric View").clicked() {
                        self.camera.azimuth = -45.0_f32.to_radians();
                        self.camera.elevation = 30.0_f32.to_radians();
                    }
                });
            });
        });

        // === Bottom panel: log ===
        egui::TopBottomPanel::bottom("log_panel")
            .min_height(80.0)
            .default_height(120.0)
            .resizable(true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading(egui::RichText::new("Log").size(12.0));
                    ui.separator();
                    if ui.button("Clear").clicked() {
                        self.log.clear();
                    }
                    if ui.button("Copy All").clicked() {
                        let all_text: String = self.log.iter()
                            .map(|e| format!("[{}] {}", e.time, e.message))
                            .collect::<Vec<_>>()
                            .join("\n");
                        ui.ctx().copy_text(all_text);
                    }
                    ui.separator();
                    ui.checkbox(&mut self.log_auto_scroll, "Auto-scroll");
                });
                egui::ScrollArea::vertical()
                    .stick_to_bottom(self.log_auto_scroll)
                    .show(ui, |ui| {
                        for entry in &self.log {
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new(format!("[{}]", entry.time))
                                    .size(10.0)
                                    .color(egui::Color32::from_rgb(120, 120, 140)));
                                // Make log messages selectable/copyable
                                ui.add(egui::Label::new(
                                    egui::RichText::new(&entry.message).size(10.0)
                                ).wrap());
                            });
                        }
                    });
            });

        // === Side panel (controls) ===
        egui::SidePanel::left("controls")
            .min_width(220.0)
            .default_width(240.0)
            .show(ctx, |ui| {
                ui.add_space(8.0);
                ui.heading("3Draper Viewer");
                ui.label(
                    egui::RichText::new("3D Geometric Kernel")
                        .size(11.0)
                        .color(egui::Color32::GRAY)
                );
                ui.separator();
                ui.add_space(4.0);

                // --- Primitives ---
                ui.heading(egui::RichText::new("Primitives").size(14.0));
                ui.horizontal(|ui| {
                    if ui.button("Box").clicked() { self.load_box(); }
                    if ui.button("Cylinder").clicked() { self.load_cylinder(); }
                    if ui.button("Sphere").clicked() { self.load_sphere(); }
                });
                ui.horizontal(|ui| {
                    if ui.button("Cone").clicked() { self.load_cone(); }
                    if ui.button("Torus").clicked() { self.load_torus(); }
                });
                ui.add_space(4.0);

                // --- Models ---
                ui.separator();
                ui.heading(egui::RichText::new("Models").size(14.0));
                if ui.button("ICE Engine (Inline-4)").clicked() {
                    self.load_engine();
                }
                ui.add_space(4.0);

                // --- Import ---
                ui.separator();
                ui.heading(egui::RichText::new("Import").size(14.0));
                ui.horizontal(|ui| {
                    ui.label("STL:");
                    if ui.button("Open...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STL", &["stl"])
                            .pick_file()
                        {
                            self.import_stl_file(&path.to_string_lossy());
                        }
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("STEP:");
                    if ui.button("Open...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STEP", &["stp", "step"])
                            .pick_file()
                        {
                            self.import_step_file(&path.to_string_lossy());
                        }
                    }
                });
                ui.add_space(4.0);

                // --- Export ---
                ui.separator();
                ui.heading(egui::RichText::new("Export").size(14.0));
                ui.horizontal(|ui| {
                    if ui.button("STL Binary").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STL", &["stl"])
                            .save_file()
                        {
                            self.export_stl_binary(&path.to_string_lossy());
                        }
                    }
                    if ui.button("STL ASCII").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STL", &["stl"])
                            .save_file()
                        {
                            self.export_stl_ascii(&path.to_string_lossy());
                        }
                    }
                });
                ui.horizontal(|ui| {
                    if ui.button("Export STEP").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STEP", &["stp", "step"])
                            .save_file()
                        {
                            self.export_step(&path.to_string_lossy());
                        }
                    }
                });
                ui.add_space(4.0);

                // --- Display ---
                ui.separator();
                ui.heading(egui::RichText::new("Display").size(14.0));
                ui.checkbox(&mut self.wireframe, "Wireframe");
                ui.checkbox(&mut self.show_axes, "Show axes");
                ui.checkbox(&mut self.show_grid, "Show grid");

                if ui.button("Reset Camera").clicked() {
                    let (bbox_min, bbox_max) = self.mesh.bounding_box();
                    self.camera.fit_to_bounding_box(
                        [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                        [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                    );
                }
                ui.add_space(4.0);

                // --- Info ---
                ui.separator();
                ui.heading(egui::RichText::new("Info").size(14.0));
                ui.label(egui::RichText::new(format!("Model: {}", self.current_model.name)).size(12.0));
                ui.label(egui::RichText::new(format!("Vertices: {}", self.current_model.vertex_count)).size(12.0));
                ui.label(egui::RichText::new(format!("Triangles: {}", self.current_model.triangle_count)).size(12.0));
                let cam_pos = self.camera.position();
                ui.label(egui::RichText::new(format!("Camera: ({:.0}, {:.0}, {:.0})", cam_pos[0], cam_pos[1], cam_pos[2]))
                    .size(11.0).color(egui::Color32::GRAY));
                ui.label(egui::RichText::new(format!("Distance: {:.1}", self.camera.distance))
                    .size(11.0).color(egui::Color32::GRAY));

                ui.add_space(8.0);
                ui.separator();
                ui.label(
                    egui::RichText::new("LMB: Rotate | Scroll: Zoom | MMB: Pan")
                        .size(10.0)
                        .color(egui::Color32::from_rgb(160, 160, 160))
                );
            });

        // === Central 3D viewport ===
        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(egui::Color32::from_rgb(30, 30, 40)))
            .show(ctx, |ui| {
                let (rect, response) = ui.allocate_exact_size(
                    ui.available_size(),
                    egui::Sense::click_and_drag(),
                );

                // Handle mouse input for camera
                let is_hovering = response.hovered();

                // Rotation: left mouse drag — orbits around model center
                if response.dragged_by(egui::PointerButton::Primary) {
                    let delta = response.drag_delta();
                    self.camera.rotate(delta.x, delta.y);
                }

                // Pan: middle mouse drag
                if response.dragged_by(egui::PointerButton::Middle) {
                    let delta = response.drag_delta();
                    self.camera.pan(delta.x, delta.y, rect.width(), rect.height());
                }

                // Zoom: scroll wheel — zoom toward mouse cursor position
                if is_hovering {
                    let scroll = ui.input(|i| i.smooth_scroll_delta);
                    if scroll.y != 0.0 {
                        // Compute normalized mouse position (-1 to 1) relative to viewport center
                        let mouse_pos_opt = ui.input(|i| i.pointer.latest_pos());
                        let mouse_norm = mouse_pos_opt.map(|pos| {
                            let nx = ((pos.x - rect.center().x) / (rect.width() * 0.5)).clamp(-1.0, 1.0);
                            let ny = -((pos.y - rect.center().y) / (rect.height() * 0.5)).clamp(-1.0, 1.0);
                            [nx, ny]
                        });
                        self.camera.zoom(scroll.y, mouse_norm);
                    }
                }

                // Get viewport dimensions
                let width = rect.width() as u32;
                let height = rect.height() as u32;

                if width == 0 || height == 0 {
                    return;
                }

                // Upload mesh data if dirty
                if self.mesh_dirty {
                    if let Some(ref rs) = self.render_state {
                        let (vertices, indices) = mesh_to_gpu_data(&self.mesh);
                        let mut guard = self.gpu_resources.lock().unwrap();
                        if let Some(ref mut resources) = *guard {
                            update_mesh_buffers(resources, &rs.device, &vertices, &indices);
                        } else {
                            let resources = create_scene_resources(rs, &vertices, &indices);
                            *guard = Some(resources);
                        }
                    }
                    self.mesh_dirty = false;
                }

                // Update uniforms — compute headlight from camera direction
                if let Some(ref rs) = self.render_state {
                    let aspect = rect.width() / rect.height();
                    let view = self.camera.view_matrix();
                    let proj = self.camera.projection_matrix(aspect);

                    // MVP = proj * view
                    let mvp = mat4_mul(&proj, &view);

                    // Model matrix is identity (mesh is already in world space)
                    let model: [[f32; 4]; 4] = [
                        [1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ];

                    let cam_pos = self.camera.position();
                    let cam_fwd = self.camera.forward();

                    // Headlight: light comes FROM the camera direction toward the model
                    // This ensures that whatever we look at is well-lit from the front
                    let uniforms = SceneUniforms {
                        mvp,
                        model,
                        light_dir: [cam_fwd[0], cam_fwd[1], cam_fwd[2], 0.25], // xyz = direction from camera, w = ambient
                        camera_pos: [cam_pos[0], cam_pos[1], cam_pos[2], 0.0],
                    };

                    let guard = self.gpu_resources.lock().unwrap();
                    if let Some(ref resources) = *guard {
                        update_uniforms(resources, &rs.queue, &uniforms);
                    }
                }

                // Create the wgpu paint callback
                let callback = SceneCallback {
                    resources: self.gpu_resources.clone(),
                    wireframe: self.wireframe,
                    viewport_width: width,
                    viewport_height: height,
                };

                let paint_callback = egui_wgpu::Callback::new_paint_callback(
                    rect,
                    callback,
                );
                ui.painter().add(paint_callback);

                // Draw axes overlay (using egui painter on top)
                if self.show_axes {
                    self.draw_axes_overlay(ui, rect);
                }
            });
    }
}

impl ViewerApp {
    fn draw_axes_overlay(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        let cos_rx = self.camera.elevation.cos();
        let sin_rx = self.camera.elevation.sin();
        let cos_ry = self.camera.azimuth.cos();
        let sin_ry = self.camera.azimuth.sin();

        let axis_len = 50.0;
        let axes: [(f32, f32, f32, egui::Color32, &str); 3] = [
            (axis_len, 0.0, 0.0, egui::Color32::RED, "X"),
            (0.0, axis_len, 0.0, egui::Color32::GREEN, "Y"),
            (0.0, 0.0, axis_len, egui::Color32::BLUE, "Z"),
        ];

        let origin_x = rect.left() + 60.0;
        let origin_y = rect.bottom() - 60.0;

        for (ax, ay, az, color, label) in axes {
            // Rotate like the camera
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

/// Multiply two 4x4 matrices (column-major).
fn mat4_mul(a: &[[f32; 4]; 4], b: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut result = [[0.0f32; 4]; 4];
    for col in 0..4 {
        for row in 0..4 {
            result[col][row] =
                a[0][row] * b[col][0] +
                a[1][row] * b[col][1] +
                a[2][row] * b[col][2] +
                a[3][row] * b[col][3];
        }
    }
    result
}
