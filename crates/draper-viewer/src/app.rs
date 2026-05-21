//! Main application state and UI.

use crate::render::RenderState;
use crate::structure_tree;
use draper_core::document::Document;
use draper_core::engine::EngineModel;
use draper_step::ast::StructureNode;
use egui::*;
use std::path::PathBuf;

pub struct DraperViewer {
    document: Option<Document>,
    structure_tree: Option<StructureNode>,
    render_state: RenderState,
    status_message: String,
    show_wireframe: bool,
    show_axes: bool,
    camera_distance: f32,
    camera_yaw: f32,
    camera_pitch: f32,
    camera_target: [f32; 3],
    _is_dragging: bool,
    _last_mouse_pos: Option<Pos2>,
    file_path: Option<PathBuf>,
}

impl DraperViewer {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            document: None,
            structure_tree: None,
            render_state: RenderState::new(),
            status_message: "No file loaded. Use File → Open to load a STEP file, or Models → ICE Engine.".to_string(),
            show_wireframe: false,
            show_axes: true,
            camera_distance: 5.0,
            camera_yaw: 45.0,
            camera_pitch: 30.0,
            camera_target: [0.0, 0.0, 0.0],
            _is_dragging: false,
            _last_mouse_pos: None,
            file_path: None,
        }
    }

    fn load_engine_model(&mut self) {
        self.status_message = "Building ICE engine model...".to_string();
        log::info!("Building inline-4 ICE engine model...");

        let engine = EngineModel::build();
        let doc = Document::from_engine(engine);

        let stats = doc.statistics();
        self.status_message = format!(
            "ICE Engine — {} vertices, {} edges, {} faces, {} solids, {} triangles",
            stats.total_vertices,
            stats.total_edges,
            stats.total_faces,
            stats.total_solids,
            stats.total_triangles,
        );

        log::info!(
            "ICE Engine built: {} vertices, {} edges, {} faces, {} solids, {} triangles",
            stats.total_vertices,
            stats.total_edges,
            stats.total_faces,
            stats.total_solids,
            stats.total_triangles
        );

        // Set camera to fit the model
        if !doc.meshes.is_empty() {
            let bb = doc.meshes[0].bounding_box();
            let center = bb.center();
            self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
            self.camera_distance = (bb.diagonal() * 1.5) as f32;
            if self.camera_distance < 1.0 {
                self.camera_distance = 500.0;
            }
        }

        self.document = Some(doc);
    }

    fn open_file(&mut self, path: PathBuf) {
        self.status_message = format!("Loading {}...", path.display());
        log::info!("Opening STEP file: {}", path.display());

        match Document::open_step(&path) {
            Ok(doc) => {
                let stats = doc.statistics();
                self.structure_tree = doc.structure_tree();
                self.status_message = format!(
                    "Loaded: {} — {} vertices, {} edges, {} faces, {} solids, {} triangles",
                    path.file_name().unwrap_or_default().to_string_lossy(),
                    stats.total_vertices,
                    stats.total_edges,
                    stats.total_faces,
                    stats.total_solids,
                    stats.total_triangles,
                );

                log::info!(
                    "STEP file loaded: {} vertices, {} edges, {} faces, {} solids, {} triangles",
                    stats.total_vertices,
                    stats.total_edges,
                    stats.total_faces,
                    stats.total_solids,
                    stats.total_triangles
                );

                // Set camera to fit the model
                if !doc.meshes.is_empty() {
                    let bb = doc.meshes[0].bounding_box();
                    let center = bb.center();
                    self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                    self.camera_distance = (bb.diagonal() * 1.5) as f32;
                    if self.camera_distance < 1.0 {
                        self.camera_distance = 50.0;
                    }
                    log::info!(
                        "Camera: target=({:.1}, {:.1}, {:.1}), distance={:.1}",
                        center.x, center.y, center.z, self.camera_distance
                    );
                } else {
                    let bb = doc.shape.bounding_box();
                    if !bb.is_empty() {
                        let center = bb.center();
                        self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                        self.camera_distance = (bb.diagonal() * 1.5) as f32;
                        if self.camera_distance < 1.0 {
                            self.camera_distance = 50.0;
                        }
                    }
                    log::warn!("No mesh generated from B-rep — using shape bounding box for camera");
                }

                self.file_path = Some(path);
                self.document = Some(doc);
            }
            Err(e) => {
                log::error!("Failed to load STEP file: {}", e);
                self.status_message = format!("Error loading file: {}", e);
            }
        }
    }

    fn save_file(&mut self) {
        if let Some(ref doc) = self.document {
            if let Some(ref path) = self.file_path {
                match doc.save_step(path) {
                    Ok(()) => {
                        self.status_message = format!("Saved: {}", path.display());
                    }
                    Err(e) => {
                        self.status_message = format!("Error saving: {}", e);
                    }
                }
            } else {
                self.status_message = "No file path set.".to_string();
            }
        }
    }
}

impl eframe::App for DraperViewer {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Menu bar
        TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Open STEP...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STEP files", &["stp", "step", "STP", "STEP"])
                            .pick_file()
                        {
                            self.open_file(path);
                        }
                        ui.close_menu();
                    }
                    if ui.button("Save").clicked() {
                        self.save_file();
                        ui.close_menu();
                    }
                    if ui.button("Save As...").clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STEP files", &["stp", "step"])
                            .save_file()
                        {
                            if let Some(ref doc) = self.document {
                                match doc.save_step(&path) {
                                    Ok(()) => {
                                        self.status_message = format!("Saved: {}", path.display());
                                        self.file_path = Some(path);
                                    }
                                    Err(e) => {
                                        self.status_message = format!("Error saving: {}", e);
                                    }
                                }
                            }
                        }
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Exit").clicked() {
                        ctx.send_viewport_cmd(ViewportCommand::Close);
                    }
                });

                ui.menu_button("Models", |ui| {
                    if ui.button("ICE Engine (Inline-4)").clicked() {
                        self.load_engine_model();
                        ui.close_menu();
                    }
                    if ui.button("Primitive: Box").clicked() {
                        let mut shape = draper_topology::shape::Shape::new();
                        draper_topology::builder::ShapeBuilder::make_box(&mut shape, 100.0, 100.0, 100.0);
                        let doc = Document::from_shape(shape);
                        let stats = doc.statistics();
                        self.status_message = format!("Box — {} triangles", stats.total_triangles);
                        if !doc.meshes.is_empty() {
                            let bb = doc.meshes[0].bounding_box();
                            let center = bb.center();
                            self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                            self.camera_distance = (bb.diagonal() * 1.5) as f32;
                        }
                        self.document = Some(doc);
                        ui.close_menu();
                    }
                    if ui.button("Primitive: Cylinder").clicked() {
                        let mut shape = draper_topology::shape::Shape::new();
                        draper_topology::builder::ShapeBuilder::make_cylinder(&mut shape, 50.0, 100.0);
                        let doc = Document::from_shape(shape);
                        let stats = doc.statistics();
                        self.status_message = format!("Cylinder — {} triangles", stats.total_triangles);
                        if !doc.meshes.is_empty() {
                            let bb = doc.meshes[0].bounding_box();
                            let center = bb.center();
                            self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                            self.camera_distance = (bb.diagonal() * 1.5) as f32;
                        }
                        self.document = Some(doc);
                        ui.close_menu();
                    }
                    if ui.button("Primitive: Sphere").clicked() {
                        let mut shape = draper_topology::shape::Shape::new();
                        draper_topology::builder::ShapeBuilder::make_sphere(&mut shape, 50.0);
                        let doc = Document::from_shape(shape);
                        let stats = doc.statistics();
                        self.status_message = format!("Sphere — {} triangles", stats.total_triangles);
                        if !doc.meshes.is_empty() {
                            let bb = doc.meshes[0].bounding_box();
                            let center = bb.center();
                            self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                            self.camera_distance = (bb.diagonal() * 1.5) as f32;
                        }
                        self.document = Some(doc);
                        ui.close_menu();
                    }
                    if ui.button("Primitive: Cone").clicked() {
                        let mut shape = draper_topology::shape::Shape::new();
                        draper_topology::builder::ShapeBuilder::make_cone(&mut shape, 50.0, 25.0, 100.0);
                        let doc = Document::from_shape(shape);
                        let stats = doc.statistics();
                        self.status_message = format!("Cone — {} triangles", stats.total_triangles);
                        if !doc.meshes.is_empty() {
                            let bb = doc.meshes[0].bounding_box();
                            let center = bb.center();
                            self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                            self.camera_distance = (bb.diagonal() * 1.5) as f32;
                        }
                        self.document = Some(doc);
                        ui.close_menu();
                    }
                    if ui.button("Primitive: Torus").clicked() {
                        let mut shape = draper_topology::shape::Shape::new();
                        draper_topology::builder::ShapeBuilder::make_torus(&mut shape, 60.0, 20.0);
                        let doc = Document::from_shape(shape);
                        let stats = doc.statistics();
                        self.status_message = format!("Torus — {} triangles", stats.total_triangles);
                        if !doc.meshes.is_empty() {
                            let bb = doc.meshes[0].bounding_box();
                            let center = bb.center();
                            self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                            self.camera_distance = (bb.diagonal() * 1.5) as f32;
                        }
                        self.document = Some(doc);
                        ui.close_menu();
                    }
                });

                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.show_wireframe, "Wireframe");
                    ui.checkbox(&mut self.show_axes, "Axes");
                    if ui.button("Fit All").clicked() {
                        if let Some(ref doc) = self.document {
                            if !doc.meshes.is_empty() {
                                let bb = doc.meshes[0].bounding_box();
                                let center = bb.center();
                                self.camera_target = [center.x as f32, center.y as f32, center.z as f32];
                                self.camera_distance = (bb.diagonal() * 1.5) as f32;
                            }
                        }
                    }
                });

                ui.menu_button("Help", |ui| {
                    ui.label("3Draper STEP Viewer v0.1.0");
                    ui.label("Custom 3D kernel — Rust");
                });
            });
        });

        // Status bar
        TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(&self.status_message);
            });
        });

        // Left panel — structure tree
        SidePanel::left("structure_panel")
            .min_width(200.0)
            .default_width(280.0)
            .show(ctx, |ui| {
                ui.heading("Structure");
                ui.separator();

                if let Some(ref tree) = self.structure_tree {
                    structure_tree::show_structure_tree(ui, tree);
                } else if let Some(ref doc) = self.document {
                    ui.label("No structure tree available.");
                    let stats = doc.statistics();
                    ui.add_space(8.0);
                    ui.heading("Statistics");
                    ui.label(format!("Vertices: {}", stats.total_vertices));
                    ui.label(format!("Edges: {}", stats.total_edges));
                    ui.label(format!("Faces: {}", stats.total_faces));
                    ui.label(format!("Solids: {}", stats.total_solids));
                    ui.label(format!("Triangles: {}", stats.total_triangles));

                    // Show part names if available
                    if !doc.part_names.is_empty() {
                        ui.add_space(8.0);
                        ui.heading("Parts");
                        let mut parts: Vec<_> = doc.part_names.iter().collect();
                        parts.sort_by_key(|(_, name)| name.clone());
                        for (_, name) in &parts {
                            ui.label(format!("• {}", name));
                        }
                    }
                } else {
                    ui.vertical_centered(|ui| {
                        ui.add_space(40.0);
                        ui.label("No file loaded");
                        ui.add_space(10.0);
                        ui.label("File → Open STEP...");
                        ui.add_space(5.0);
                        ui.label("Models → ICE Engine");
                    });
                }
            });

        // Central panel — 3D viewport
        CentralPanel::default().show(ctx, |ui| {
            let (rect, response) = ui.allocate_at_least(ui.available_size(), Sense::click_and_drag());

            // Handle mouse input for camera control
            if response.dragged() {
                let delta = response.drag_delta();
                if response.dragged_by(PointerButton::Primary) {
                    self.camera_yaw += delta.x * 0.5;
                    self.camera_pitch -= delta.y * 0.5;
                    self.camera_pitch = self.camera_pitch.clamp(-89.0, 89.0);
                } else if response.dragged_by(PointerButton::Secondary) {
                    let pan_speed = self.camera_distance * 0.002;
                    self.camera_target[0] -= delta.x * pan_speed;
                    self.camera_target[1] += delta.y * pan_speed;
                }
            }

            let scroll = ui.input(|i| i.raw_scroll_delta.y);
            if scroll != 0.0 {
                self.camera_distance *= 1.0 - scroll * 0.001;
                self.camera_distance = self.camera_distance.max(0.01);
            }

            if let Some(ref doc) = self.document {
                self.render_state.render(
                    ui,
                    rect,
                    &doc.meshes,
                    self.camera_yaw,
                    self.camera_pitch,
                    self.camera_distance,
                    self.camera_target,
                    self.show_wireframe,
                    self.show_axes,
                );
            } else {
                ui.painter().rect_filled(rect, 0.0, Color32::from_rgb(30, 30, 40));
                ui.painter().text(
                    rect.center(),
                    Align2::CENTER_CENTER,
                    "Open a STEP file or load a model",
                    FontId::proportional(20.0),
                    Color32::from_rgb(120, 120, 140),
                );
            }
        });

        ctx.request_repaint();
    }
}
