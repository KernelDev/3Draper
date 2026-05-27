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
use draper_step::{AssemblyNode, DetailedMeshInstance, FaceInfo};
use draper_geometry::{Surface, Point2d};
use egui_wgpu::RenderState;
use eframe::egui;

/// Convert TriangleMesh to GPU vertex/index data.
/// Uses flat shading with face normals to properly support per-triangle colors from STEP files.
fn mesh_to_gpu_data(mesh: &TriangleMesh, highlighted_face_id: Option<u64>) -> (Vec<MeshVertex>, Vec<u32>) {
    let mut mesh = mesh.clone();
    if mesh.face_normals.is_none() {
        mesh.compute_face_normals();
    }
    mesh.ensure_colors([0.48, 0.52, 0.58, 1.0]);

    let normals = mesh.face_normals.as_ref();
    let colors = mesh.triangle_colors.as_ref();
    let face_ids = mesh.triangle_face_ids.as_ref();

    // Check if we have meaningful per-triangle colors (not all default grey)
    let has_real_colors = colors.map_or(false, |c| {
        c.iter().any(|col| (col[0] - 0.48).abs() > 0.01 || (col[1] - 0.52).abs() > 0.01 || (col[2] - 0.58).abs() > 0.01)
    });

    // If we have vertex normals and no special per-triangle colors, use smooth shading
    // But if we need face ID highlighting, we must use flat shading
    if let Some(ref vertex_normals) = mesh.normals {
        if vertex_normals.len() == mesh.vertices.len() && !has_real_colors && highlighted_face_id.is_none() {
            let mut gpu_vertices = Vec::with_capacity(mesh.vertices.len());
            let mut gpu_indices = Vec::with_capacity(mesh.triangles.len() * 3);

            for (i, v) in mesh.vertices.iter().enumerate() {
                let n = vertex_normals.get(i).map(|nn| [nn[0] as f32, nn[1] as f32, nn[2] as f32]).unwrap_or([0.0, 0.0, 1.0]);
                gpu_vertices.push(MeshVertex {
                    position: [v.x as f32, v.y as f32, v.z as f32],
                    normal: n,
                    color: [0.48, 0.52, 0.58],
                });
            }
            for tri in &mesh.triangles {
                gpu_indices.push(tri[0]);
                gpu_indices.push(tri[1]);
                gpu_indices.push(tri[2]);
            }
            return (gpu_vertices, gpu_indices);
        }
    }

    // Flat shading: duplicate vertices per triangle with face normals and colors
    let mut gpu_vertices = Vec::with_capacity(mesh.triangles.len() * 3);
    let mut gpu_indices = Vec::with_capacity(mesh.triangles.len() * 3);

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let normal = normals
            .and_then(|n| n.get(i))
            .map(|n| [n[0] as f32, n[1] as f32, n[2] as f32])
            .unwrap_or([0.0, 0.0, 1.0]);

        let mut color = colors
            .and_then(|c| c.get(i))
            .map(|c| [c[0], c[1], c[2]])
            .unwrap_or([0.48, 0.52, 0.58]);

        // Highlight the selected face with a bright yellow-orange color
        if let Some(hid) = highlighted_face_id {
            if let Some(ids) = face_ids {
                if ids.get(i).map_or(false, |id| *id == hid) {
                    color = [1.0, 0.85, 0.1]; // bright yellow for selected face
                }
            }
        }

        let base_idx = gpu_vertices.len() as u32;
        for &idx in tri {
            let v = &mesh.vertices[idx as usize];
            gpu_vertices.push(MeshVertex {
                position: [v.x as f32, v.y as f32, v.z as f32],
                normal,
                color,
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

/// Result of an async file load (used on wasm).
#[derive(Debug)]
enum FileLoadResult {
    Step { name: String, content: String },
    Stl { name: String, data: Vec<u8> },
}

/// Shared state for async file loading on wasm.
#[cfg(target_arch = "wasm32")]
type SharedFileResult = Arc<Mutex<Option<FileLoadResult>>>;

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
    /// Shared file result for async web file loading.
    #[cfg(target_arch = "wasm32")]
    file_result: SharedFileResult,

    // ─── Structure panel state ─────────────────────────────────────
    /// Detailed mesh instances from STEP (with per-face info).
    detailed_instances: Vec<DetailedMeshInstance>,
    /// Assembly tree for structure panel.
    assembly_tree: Option<AssemblyNode>,
    /// Currently selected instance index.
    selected_instance: Option<usize>,
    /// Currently selected face ID (within the instance).
    selected_face_id: Option<u64>,
    /// Whether to show the structure panel.
    show_structure: bool,

    // ─── UV grid state ─────────────────────────────────────────────
    /// Whether to show UV grid for the selected face.
    show_uv_grid: bool,
    /// UV grid U subdivisions.
    uv_grid_u: usize,
    /// UV grid V subdivisions.
    uv_grid_v: usize,
    /// Cached UV grid SVG string for the selected face.
    uv_svg_cache: Option<(u64, String)>, // (face_id, svg_content)

    // ─── Face highlight state ──────────────────────────────────────
    /// Currently highlighted face ID (for 3D view highlighting).
    highlighted_face_id: Option<u64>,
    /// Whether the GPU data needs update due to highlight change.
    highlight_dirty: bool,
}

impl ViewerApp {
    fn log(&mut self, msg: &str) {
        #[cfg(not(target_arch = "wasm32"))]
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);

        #[cfg(target_arch = "wasm32")]
        let now = {
            let millis = js_sys::Date::now() as u64;
            millis / 1000
        };

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
            let (vertices, indices) = mesh_to_gpu_data(&mesh, None);
            let resources = create_scene_resources(rs, &vertices, &indices);
            *gpu_resources.lock().unwrap() = Some(resources);
        }

        #[cfg(target_arch = "wasm32")]
        let file_result = Arc::new(Mutex::new(None));

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
            #[cfg(target_arch = "wasm32")]
            file_result,
            detailed_instances: Vec::new(),
            assembly_tree: None,
            selected_instance: None,
            selected_face_id: None,
            show_structure: true,
            show_uv_grid: false,
            uv_grid_u: 10,
            uv_grid_v: 10,
            uv_svg_cache: None,
            highlighted_face_id: None,
            highlight_dirty: false,
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
        // Reset selection when loading new model
        self.selected_instance = None;
        self.selected_face_id = None;
        self.highlighted_face_id = None;
        self.highlight_dirty = true;
        self.uv_svg_cache = None;
        self.log(&format!("Loaded: {} ({} vertices, {} triangles)",
            name, self.current_model.vertex_count, self.current_model.triangle_count));
    }

    fn load_box(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Box 100x80x60");
    }

    fn load_cylinder(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cylinder R=40 H=100");
    }

    fn load_sphere(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Sphere R=50");
    }

    fn load_cone(&mut self) {
        let radius: f64 = 40.0;
        let height: f64 = 80.0;
        let half_angle = (radius / height).atan();
        let solid = ShapeBuilder::make_cone(radius, height, half_angle);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cone R=40 H=80");
    }

    fn load_torus(&mut self) {
        let solid = ShapeBuilder::make_torus(40.0, 12.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Torus R=40 r=12");
    }

    fn load_engine(&mut self) {
        let doc = build_engine(&EngineConfig::default());
        let mesh = doc.triangulate();
        self.detailed_instances.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "ICE Engine (I4)");
    }

    // ─── Native file I/O (uses rfd + filesystem) ─────────────────────────

    #[cfg(not(target_arch = "wasm32"))]
    fn import_stl_file(&mut self, path: &str) {
        match draper_mesh::import_stl_binary(path) {
            Ok(mesh) => {
                let name = std::path::Path::new(path)
                    .file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "STL file".to_string());
                self.detailed_instances.clear();
                self.assembly_tree = None;
                self.load_mesh(mesh, &format!("STL: {}", name));
            }
            Err(e) => {
                self.log(&format!("STL import error: {}", e));
            }
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn import_step_file(&mut self, path: &str) {
        match draper_step::parse_step_file(path) {
            Ok(step_file) => {
                let name = std::path::Path::new(path)
                    .file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "STEP file".to_string());
                self.process_step_file(&step_file, &name);
            }
            Err(e) => {
                self.log(&format!("STEP import error: {}", e));
            }
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn export_stl_binary(&mut self, path: &str) {
        match draper_mesh::stl::write_stl_file(&self.mesh, path, true) {
            Ok(()) => self.log(&format!("Exported STL (binary): {}", path)),
            Err(e) => self.log(&format!("STL export error: {}", e)),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn export_stl_ascii(&mut self, path: &str) {
        match draper_mesh::stl::write_stl_file(&self.mesh, path, false) {
            Ok(()) => self.log(&format!("Exported STL (ASCII): {}", path)),
            Err(e) => self.log(&format!("STL export error: {}", e)),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn export_step(&mut self, path: &str) {
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

    // ─── Shared file processing (used by both native and web) ─────────────

    /// Process a parsed STEP file (common logic for native and web).
    fn process_step_file(&mut self, step_file: &draper_step::StepFile, name: &str) {
        // Count relevant geometry entities
        let mut point_count = 0;
        let mut face_count = 0;
        let mut shell_count = 0;
        let mut brep_count = 0;
        let mut nauo_count = 0;
        let mut styled_item_count = 0;
        let mut idt_count = 0;
        let mut cdsr_count = 0;
        let mut srr_count = 0;
        let mut surface_types: std::collections::HashMap<String, usize> = std::collections::HashMap::new();

        for entity in &step_file.entities {
            match entity.type_name.as_str() {
                "CARTESIAN_POINT" => point_count += 1,
                "ADVANCED_FACE" | "FACE_OUTER_BOUND" | "FACE_BOUND" => face_count += 1,
                "CLOSED_SHELL" | "OPEN_SHELL" => shell_count += 1,
                "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => brep_count += 1,
                "NEXT_ASSEMBLY_USAGE_OCCURRENCE" => nauo_count += 1,
                "STYLED_ITEM" => styled_item_count += 1,
                "ITEM_DEFINED_TRANSFORMATION" => idt_count += 1,
                "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION" => cdsr_count += 1,
                _ => {
                    if entity.type_name.contains("SHAPE_REPRESENTATION_RELATIONSHIP") {
                        srr_count += 1;
                    }
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
            "STEP parsed: {} — {} entities, {} pts, {} faces, {} shells, {} breps, {} NAUOs, {} styled, {} IDT, {} CDSR, {} SRR",
            name, step_file.entities.len(), point_count, face_count, shell_count, brep_count, nauo_count, styled_item_count, idt_count, cdsr_count, srr_count
        ));
        if !surface_summary.is_empty() {
            self.log(&format!("  Surfaces: {}", surface_summary.join(", ")));
        }

        // Build assembly tree for structure panel
        self.assembly_tree = Some(draper_step::step_structure(step_file));

        // ── Convert STEP to detailed mesh instances ──
        match draper_step::step_to_detailed_instances(step_file) {
            Ok(instances) => {
                self.log(&format!("── Mesh Rendering Tree ({} instances) ──", instances.len()));

                // Store detailed instances for structure/selection
                self.detailed_instances = instances.clone();

                // Merge all instances into a single mesh for rendering
                let mut mesh = TriangleMesh::new();
                for inst in &instances {
                    if let Some(color) = inst.color {
                        mesh.merge_with_color(&inst.mesh, color);
                    } else {
                        mesh.merge_with_color(&inst.mesh, [0.48, 0.52, 0.58, 1.0]);
                    }
                }
                let vcount = mesh.vertex_count();
                let tcount = mesh.triangle_count();
                self.log(&format!(
                    "Total merged: {} vertices, {} triangles", vcount, tcount
                ));
                self.load_mesh(mesh, &format!("STEP: {}", name));
            }
            Err(e) => {
                self.log(&format!("STEP detailed conversion error: {}, trying simple conversion", e));
                // Fallback to simple conversion
                match draper_step::step_to_mesh_instances(step_file) {
                    Ok(instances) => {
                        self.log(&format!("── Simple Mesh Instances: {} ──", instances.len()));
                        let mut mesh = TriangleMesh::new();
                        for inst in &instances {
                            if let Some(color) = inst.color {
                                mesh.merge_with_color(&inst.mesh, color);
                            } else {
                                mesh.merge_with_color(&inst.mesh, [0.48, 0.52, 0.58, 1.0]);
                            }
                        }
                        self.detailed_instances.clear();
                        self.load_mesh(mesh, &format!("STEP: {}", name));
                    }
                    Err(e2) => {
                        self.log(&format!("STEP conversion error: {}", e2));
                    }
                }
            }
        }
    }

    /// Import STL from bytes (used by web file loading).
    fn import_stl_from_bytes(&mut self, data: &[u8], name: &str) {
        match draper_mesh::import_stl_from_bytes(data) {
            Ok(mesh) => {
                self.load_mesh(mesh, &format!("STL: {}", name));
            }
            Err(e) => {
                self.log(&format!("STL import error: {}", e));
            }
        }
    }

    /// Import STEP from string (used by web file loading).
    fn import_step_from_str(&mut self, content: &str, name: &str) {
        match draper_step::parse_step(content) {
            Ok(step_file) => {
                self.process_step_file(&step_file, name);
            }
            Err(e) => {
                self.log(&format!("STEP import error: {}", e));
            }
        }
    }

    // ─── Web file loading (uses web-sys for file input) ───────────────────

    /// Trigger a file input dialog on the web for STL files.
    #[cfg(target_arch = "wasm32")]
    fn trigger_stl_file_input(&mut self) {
        use wasm_bindgen::prelude::*;

        let window = web_sys::window().unwrap();
        let document = window.document().unwrap();
        let input = document.create_element("input").unwrap();
        input.set_attribute("type", "file").unwrap();
        input.set_attribute("accept", ".stl").unwrap();
        input.set_attribute("style", "display:none").unwrap();

        let input_elem: web_sys::HtmlInputElement = input.clone().unchecked_into();
        let html_elem: web_sys::HtmlElement = input.clone().unchecked_into();
        let shared_result = self.file_result.clone();

        let input_elem_for_closure = input_elem.clone();

        let onchange = Closure::wrap(Box::new(move |_: web_sys::Event| {
            if let Some(files) = input_elem_for_closure.files() {
                if let Some(file) = files.get(0) {
                    let file_name = file.name();
                    let reader = web_sys::FileReader::new().unwrap();
                    let reader_clone = reader.clone();
                    let shared = shared_result.clone();

                    let onload = Closure::wrap(Box::new(move |_: web_sys::Event| {
                        if let Ok(result) = reader_clone.result() {
                            let array_buffer: js_sys::ArrayBuffer = result.into();
                            let uint8_array = js_sys::Uint8Array::new(&array_buffer);
                            let data = uint8_array.to_vec();
                            *shared.lock().unwrap() = Some(FileLoadResult::Stl {
                                name: file_name.clone(),
                                data,
                            });
                        }
                    }) as Box<dyn FnMut(_)>);

                    reader.set_onload(Some(onload.as_ref().unchecked_ref()));
                    onload.forget();
                    let _ = reader.read_as_array_buffer(&file);
                }
            }
        }) as Box<dyn FnMut(_)>);

        input_elem.set_onchange(Some(onchange.as_ref().unchecked_ref()));
        onchange.forget();

        let body = document.body().unwrap();
        let _ = body.append_child(&input);
        html_elem.click();
    }

    /// Trigger a file input dialog on the web for STEP files.
    #[cfg(target_arch = "wasm32")]
    fn trigger_step_file_input(&mut self) {
        use wasm_bindgen::prelude::*;

        let window = web_sys::window().unwrap();
        let document = window.document().unwrap();
        let input = document.create_element("input").unwrap();
        input.set_attribute("type", "file").unwrap();
        input.set_attribute("accept", ".stp,.step").unwrap();
        input.set_attribute("style", "display:none").unwrap();

        let input_elem: web_sys::HtmlInputElement = input.clone().unchecked_into();
        let html_elem: web_sys::HtmlElement = input.clone().unchecked_into();
        let shared_result = self.file_result.clone();

        let input_elem_for_closure = input_elem.clone();

        let onchange = Closure::wrap(Box::new(move |_: web_sys::Event| {
            if let Some(files) = input_elem_for_closure.files() {
                if let Some(file) = files.get(0) {
                    let file_name = file.name();
                    let reader = web_sys::FileReader::new().unwrap();
                    let reader_clone = reader.clone();
                    let shared = shared_result.clone();

                    let onload = Closure::wrap(Box::new(move |_: web_sys::Event| {
                        if let Ok(result) = reader_clone.result() {
                            if let Some(text) = result.as_string() {
                                *shared.lock().unwrap() = Some(FileLoadResult::Step {
                                    name: file_name.clone(),
                                    content: text,
                                });
                            }
                        }
                    }) as Box<dyn FnMut(_)>);

                    reader.set_onload(Some(onload.as_ref().unchecked_ref()));
                    onload.forget();
                    let _ = reader.read_as_text(&file);
                }
            }
        }) as Box<dyn FnMut(_)>);

        input_elem.set_onchange(Some(onchange.as_ref().unchecked_ref()));
        onchange.forget();

        let body = document.body().unwrap();
        let _ = body.append_child(&input);
        html_elem.click();
    }

    /// Check for loaded web files and process them.
    #[cfg(target_arch = "wasm32")]
    fn process_web_file_loads(&mut self) {
        let result = self.file_result.lock().unwrap().take();
        if let Some(file_result) = result {
            match file_result {
                FileLoadResult::Step { name, content } => {
                    self.import_step_from_str(&content, &name);
                }
                FileLoadResult::Stl { name, data } => {
                    self.import_stl_from_bytes(&data, &name);
                }
            }
        }
    }

    /// Rebuild a Solid from the current model for export purposes.
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
                ShapeBuilder::make_box(100.0, 100.0, 100.0)
            }
        }
    }

    /// Generate UV grid SVG for the currently selected face.
    fn generate_uv_svg(&self, face: &FaceInfo) -> String {
        let u_divs = self.uv_grid_u;
        let v_divs = self.uv_grid_v;
        let svg_width = 600.0;
        let svg_height = 600.0;
        let margin = 40.0;
        let draw_w = svg_width - 2.0 * margin;
        let draw_h = svg_height - 2.0 * margin;

        // Compute UV bounding box from boundary polylines
        let mut u_min = f64::MAX;
        let mut u_max = f64::MIN;
        let mut v_min = f64::MAX;
        let mut v_max = f64::MIN;

        for polyline in &face.outer_uv_boundary {
            for pt in polyline {
                u_min = u_min.min(pt.u);
                u_max = u_max.max(pt.u);
                v_min = v_min.min(pt.v);
                v_max = v_max.max(pt.v);
            }
        }
        for boundary in &face.inner_uv_boundaries {
            for polyline in boundary {
                for pt in polyline {
                    u_min = u_min.min(pt.u);
                    u_max = u_max.max(pt.u);
                    v_min = v_min.min(pt.v);
                    v_max = v_max.max(pt.v);
                }
            }
        }

        if u_min >= u_max || v_min >= v_max {
            // Try to compute from surface sampling
            match &face.surface {
                Surface::Nurbs(n) => {
                    let (ur0, ur1) = n.u_range();
                    let (vr0, vr1) = n.v_range();
                    u_min = ur0; u_max = ur1; v_min = vr0; v_max = vr1;
                }
                _ => {
                    u_min = 0.0; u_max = 1.0; v_min = 0.0; v_max = 1.0;
                }
            }
        }

        // Add padding
        let u_range = (u_max - u_min).max(1e-6);
        let v_range = (v_max - v_min).max(1e-6);
        u_min -= u_range * 0.05;
        u_max += u_range * 0.05;
        v_min -= v_range * 0.05;
        v_max += v_range * 0.05;

        let map_u = |u: f64| -> f64 { margin + (u - u_min) / (u_max - u_min) * draw_w };
        let map_v = |v: f64| -> f64 { margin + (1.0 - (v - v_min) / (v_max - v_min)) * draw_h };

        let mut svg = String::new();
        svg.push_str(&format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
            <svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" height=\"{}\" viewBox=\"0 0 {} {}\">\n",
            svg_width as i32, svg_height as i32, svg_width as i32, svg_height as i32
        ));
        svg.push_str(&format!(
            "  <rect width=\"{}\" height=\"{}\" fill=\"#1a1a2e\"/>\n",
            svg_width as i32, svg_height as i32
        ));

        // Draw UV grid lines
        for i in 0..=u_divs {
            let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
            let x = map_u(u);
            svg.push_str(&format!(
                "  <line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#334\" stroke-width=\"0.5\"/>\n",
                x, margin, x, margin + draw_h
            ));
        }
        for j in 0..=v_divs {
            let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
            let y = map_v(v);
            svg.push_str(&format!(
                "  <line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#334\" stroke-width=\"0.5\"/>\n",
                margin, y, margin + draw_w, y
            ));
        }

        // Draw outer boundary polylines
        for polyline in &face.outer_uv_boundary {
            if polyline.len() < 2 { continue; }
            let mut d = format!("M {:.2} {:.2}", map_u(polyline[0].u), map_v(polyline[0].v));
            for pt in &polyline[1..] {
                d.push_str(&format!(" L {:.2} {:.2}", map_u(pt.u), map_v(pt.v)));
            }
            d.push_str(" Z");
            svg.push_str(&format!(
                "  <path d=\"{}\" fill=\"none\" stroke=\"#00ff88\" stroke-width=\"1.5\"/>\n", d
            ));
        }

        // Draw inner boundary polylines (holes)
        for boundary in &face.inner_uv_boundaries {
            for polyline in boundary {
                if polyline.len() < 2 { continue; }
                let mut d = format!("M {:.2} {:.2}", map_u(polyline[0].u), map_v(polyline[0].v));
                for pt in &polyline[1..] {
                    d.push_str(&format!(" L {:.2} {:.2}", map_u(pt.u), map_v(pt.v)));
                }
                d.push_str(" Z");
                svg.push_str(&format!(
                    "  <path d=\"{}\" fill=\"none\" stroke=\"#ff4444\" stroke-width=\"1.5\" stroke-dasharray=\"4,2\"/>\n", d
                ));
            }
        }

        // Draw UV grid intersection points
        for i in 0..=u_divs {
            for j in 0..=v_divs {
                let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                let pt3d = face.surface.point_at(u, v);
                // Only draw if the point is finite
                if pt3d.x.is_finite() && pt3d.y.is_finite() && pt3d.z.is_finite() {
                    let x = map_u(u);
                    let y = map_v(v);
                    svg.push_str(&format!(
                        "  <circle cx=\"{:.2}\" cy=\"{:.2}\" r=\"2\" fill=\"#6688ff\" opacity=\"0.7\"/>\n", x, y
                    ));
                }
            }
        }

        // Draw axis labels
        svg.push_str(&format!(
            "  <text x=\"{}\" y=\"{}\" fill=\"#aaa\" font-size=\"12\" text-anchor=\"middle\">U ({:.2} .. {:.2})</text>\n",
            margin + draw_w / 2.0, svg_height - 5.0, u_min, u_max
        ));
        svg.push_str(&format!(
            "  <text x=\"10\" y=\"{}\" fill=\"#aaa\" font-size=\"12\" text-anchor=\"middle\" transform=\"rotate(-90, 10, {})\">V ({:.2} .. {:.2})</text>\n",
            margin + draw_h / 2.0, margin + draw_h / 2.0, v_min, v_max
        ));

        // Title with face info
        svg.push_str(&format!(
            "  <text x=\"{}\" y=\"20\" fill=\"#fff\" font-size=\"13\" text-anchor=\"middle\">Face #{} (STEP #{}) {} forward={}</text>\n",
            svg_width / 2.0, face.face_id, face.step_face_id, face.surface_type, face.forward
        ));

        svg.push_str("</svg>\n");
        svg
    }

    /// Handle picking: find which face is at the given screen position.
    fn pick_face_at(&self, _screen_x: f32, _screen_y: f32) -> Option<(usize, u64)> {
        // Simplified picking: iterate all instances and faces, check if any triangle 
        // is under the cursor. For now, return None since true raycasting is complex.
        // The primary selection mechanism is through the structure panel.
        None
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Request repaint for continuous rendering
        ctx.request_repaint();

        // Process any pending web file loads
        #[cfg(target_arch = "wasm32")]
        self.process_web_file_loads();

        // === Top menu bar ===
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    #[cfg(not(target_arch = "wasm32"))]
                    {
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
                    }

                    #[cfg(target_arch = "wasm32")]
                    {
                        if ui.button("Import STL...").clicked() {
                            self.trigger_stl_file_input();
                            ui.close_menu();
                        }
                        if ui.button("Import STEP...").clicked() {
                            self.trigger_step_file_input();
                            ui.close_menu();
                        }
                    }

                    if ui.button("Quit").clicked() {
                        #[cfg(not(target_arch = "wasm32"))]
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.wireframe, "Wireframe");
                    ui.checkbox(&mut self.show_axes, "Show axes");
                    ui.checkbox(&mut self.show_grid, "Show grid");
                    ui.checkbox(&mut self.show_structure, "Structure Panel");
                    ui.separator();
                    if ui.button("Reset Camera").clicked() {
                        let (bbox_min, bbox_max) = self.mesh.bounding_box();
                        self.camera.fit_to_bounding_box(
                            [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                            [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                        );
                    }
                    if ui.button("Top View").clicked() {
                        self.camera.look_from_direction([0.0, -1.0, 0.0]);
                    }
                    if ui.button("Front View").clicked() {
                        self.camera.look_from_direction([0.0, 0.0, 1.0]);
                    }
                    if ui.button("Right View").clicked() {
                        self.camera.look_from_direction([-1.0, 0.0, 0.0]);
                    }
                    if ui.button("Isometric View").clicked() {
                        let d = 45.0_f32.to_radians();
                        let e = 30.0_f32.to_radians();
                        self.camera.look_from_direction([
                            -e.cos() * d.sin(),
                            -e.sin(),
                            e.cos() * d.cos(),
                        ]);
                    }
                });
            });
        });

        // === Bottom panel: log ===
        egui::TopBottomPanel::bottom("log_panel")
            .min_height(60.0)
            .default_height(100.0)
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
                                ui.add(egui::Label::new(
                                    egui::RichText::new(&entry.message).size(10.0)
                                ).wrap());
                            });
                        }
                    });
            });

        // === Right panel: Structure / Faces / UV ===
        if self.show_structure {
            egui::SidePanel::right("structure_panel")
                .min_width(280.0)
                .default_width(320.0)
                .resizable(true)
                .show(ctx, |ui| {
                    ui.add_space(4.0);
                    ui.heading(egui::RichText::new("Structure").size(14.0));
                    ui.separator();

                    // ─── Assembly Tree ──────────────────────────────────
                    egui::ScrollArea::vertical()
                        .max_height(250.0)
                        .show(ui, |ui| {
                            if let Some(ref tree) = self.assembly_tree {
                                self.draw_assembly_node(ui, tree);
                            } else if !self.detailed_instances.is_empty() {
                                for (i, inst) in self.detailed_instances.iter().enumerate() {
                                    let is_selected = self.selected_instance == Some(i);
                                    let label = format!("{} (BREP#{})", inst.name, inst.brep_id);
                                    if ui.selectable_label(is_selected, &label).clicked() {
                                        self.selected_instance = Some(i);
                                        self.selected_face_id = None;
                                        self.highlighted_face_id = None;
                                        self.highlight_dirty = true;
                                        self.uv_svg_cache = None;
                                    }
                                }
                            } else {
                                ui.label(egui::RichText::new("No STEP file loaded").size(11.0).color(egui::Color32::GRAY));
                            }
                        });

                    ui.separator();

                    // ─── Face List for Selected Instance ─────────────────
                    if let Some(inst_idx) = self.selected_instance {
                        if let Some(inst) = self.detailed_instances.get(inst_idx) {
                            ui.heading(egui::RichText::new(format!("Faces: {}", inst.name)).size(12.0));
                            ui.label(egui::RichText::new(format!("BREP #{} — {} faces", inst.brep_id, inst.faces.len()))
                                .size(11.0).color(egui::Color32::GRAY));

                            egui::ScrollArea::vertical()
                                .max_height(200.0)
                                .show(ui, |ui| {
                                    for face in &inst.faces {
                                        let is_selected = self.selected_face_id == Some(face.face_id);
                                        let label = format!("F#{} STEP#{} {} [{}..{}]",
                                            face.face_id, face.step_face_id, face.surface_type,
                                            face.triangle_range.0, face.triangle_range.1);
                                        let response = ui.selectable_label(is_selected, &label);
                                        if response.clicked() {
                                            self.selected_face_id = Some(face.face_id);
                                            self.highlighted_face_id = Some(face.face_id);
                                            self.highlight_dirty = true;
                                            self.uv_svg_cache = None;
                                            self.log(&format!("Selected face #{} (STEP #{}) {} tris=[{},{}]",
                                                face.face_id, face.step_face_id, face.surface_type,
                                                face.triangle_range.0, face.triangle_range.1));
                                        }
                                        // Show tooltip on hover
                                        response.on_hover_text(format!(
                                            "Face ID: {}\nSTEP ID: {}\nSurface: {}\nTriangles: [{}, {})\nOuter edges: {}\nInner edges: {}\nForward: {}",
                                            face.face_id, face.step_face_id, face.surface_type,
                                            face.triangle_range.0, face.triangle_range.1,
                                            face.outer_boundary.len(), face.inner_boundaries.len(),
                                            face.forward
                                        ));
                                    }
                                });
                        }
                    } else {
                        ui.label(egui::RichText::new("Select an instance to see faces").size(11.0).color(egui::Color32::GRAY));
                    }

                    ui.separator();

                    // ─── UV Grid Controls ────────────────────────────────
                    ui.heading(egui::RichText::new("UV Grid").size(14.0));
                    ui.checkbox(&mut self.show_uv_grid, "Show UV grid");
                    ui.horizontal(|ui| {
                        ui.label("U divs:");
                        ui.add(egui::DragValue::new(&mut self.uv_grid_u).range(2..=50));
                    });
                    ui.horizontal(|ui| {
                        ui.label("V divs:");
                        ui.add(egui::DragValue::new(&mut self.uv_grid_v).range(2..=50));
                    });

                    // ─── UV Grid Display ─────────────────────────────────
                    if self.show_uv_grid {
                        if let Some(inst_idx) = self.selected_instance {
                            if let Some(face_id) = self.selected_face_id {
                                if let Some(inst) = self.detailed_instances.get(inst_idx) {
                                    if let Some(face) = inst.faces.iter().find(|f| f.face_id == face_id) {
                                        // Check cache
                                        let needs_regen = self.uv_svg_cache.as_ref().map_or(true, |(id, _)| *id != face_id);
                                        if needs_regen {
                                            let svg = self.generate_uv_svg(face);
                                            self.uv_svg_cache = Some((face_id, svg));
                                        }

                                        // Draw UV grid in the panel using custom painting
                                        let available = ui.available_size();
                                        let size = available.x.min(available.y - 30.0).min(400.0);
                                        if size > 50.0 {
                                            let (rect, _response) = ui.allocate_exact_size(
                                                egui::vec2(size, size),
                                                egui::Sense::hover(),
                                            );
                                            // Draw UV grid background
                                            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(26, 26, 46));
                                            
                                            let margin = size * 0.067; // 40/600 ratio
                                            let draw_size = size - 2.0 * margin;

                                            // Compute UV bounds from face
                                            let mut u_min = f64::MAX;
                                            let mut u_max = f64::MIN;
                                            let mut v_min = f64::MAX;
                                            let mut v_max = f64::MIN;
                                            for polyline in &face.outer_uv_boundary {
                                                for pt in polyline {
                                                    u_min = u_min.min(pt.u); u_max = u_max.max(pt.u);
                                                    v_min = v_min.min(pt.v); v_max = v_max.max(pt.v);
                                                }
                                            }
                                            if u_min >= u_max || v_min >= v_max {
                                                match &face.surface {
                                                    Surface::Nurbs(n) => {
                                                        let (ur0, ur1) = n.u_range();
                                                        let (vr0, vr1) = n.v_range();
                                                        u_min = ur0; u_max = ur1; v_min = vr0; v_max = vr1;
                                                    }
                                                    _ => { u_min = 0.0; u_max = 1.0; v_min = 0.0; v_max = 1.0; }
                                                }
                                            }
                                            let u_range = (u_max - u_min).max(1e-6);
                                            let v_range = (v_max - v_min).max(1e-6);
                                            u_min -= u_range * 0.05; u_max += u_range * 0.05;
                                            v_min -= v_range * 0.05; v_max += v_range * 0.05;

                                            let margin_f64 = margin as f64;
                                            let draw_size_f64 = draw_size as f64;
                                            let map_u = |u: f64| -> f32 { (margin_f64 + (u - u_min) / (u_max - u_min) * draw_size_f64) as f32 };
                                            let map_v = |v: f64| -> f32 { (margin_f64 + (1.0 - (v - v_min) / (v_max - v_min)) * draw_size_f64) as f32 };

                                            // Draw grid lines
                                            let u_divs = self.uv_grid_u.min(50);
                                            let v_divs = self.uv_grid_v.min(50);
                                            for i in 0..=u_divs {
                                                let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                                                let x = map_u(u);
                                                ui.painter().line_segment(
                                                    [egui::pos2(x, rect.top() + margin), egui::pos2(x, rect.bottom() - margin)],
                                                    egui::Stroke::new(0.5, egui::Color32::from_rgb(51, 51, 68)),
                                                );
                                            }
                                            for j in 0..=v_divs {
                                                let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                                                let y = map_v(v);
                                                ui.painter().line_segment(
                                                    [egui::pos2(rect.left() + margin, y), egui::pos2(rect.right() - margin, y)],
                                                    egui::Stroke::new(0.5, egui::Color32::from_rgb(51, 51, 68)),
                                                );
                                            }

                                            // Draw outer boundary
                                            for polyline in &face.outer_uv_boundary {
                                                if polyline.len() < 2 { continue; }
                                                let points: Vec<egui::Pos2> = polyline.iter()
                                                    .map(|pt| egui::pos2(map_u(pt.u), map_v(pt.v)))
                                                    .collect();
                                                ui.painter().line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(0, 255, 136)));
                                            }

                                            // Draw inner boundaries (holes)
                                            for boundary in &face.inner_uv_boundaries {
                                                for polyline in boundary {
                                                    if polyline.len() < 2 { continue; }
                                                    let points: Vec<egui::Pos2> = polyline.iter()
                                                        .map(|pt| egui::pos2(map_u(pt.u), map_v(pt.v)))
                                                        .collect();
                                                    ui.painter().line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 68, 68)));
                                                }
                                            }

                                            // Draw grid intersection points
                                            for i in 0..=u_divs {
                                                for j in 0..=v_divs {
                                                    let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                                                    let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                                                    let pt3d = face.surface.point_at(u, v);
                                                    if pt3d.x.is_finite() && pt3d.y.is_finite() && pt3d.z.is_finite() {
                                                        let x = map_u(u);
                                                        let y = map_v(v);
                                                        ui.painter().circle_filled(
                                                            egui::pos2(x, y), 2.0,
                                                            egui::Color32::from_rgba_premultiplied(102, 136, 255, 180),
                                                        );
                                                    }
                                                }
                                            }

                                            // Labels
                                            ui.painter().text(
                                                egui::pos2(rect.center().x, rect.bottom() - 5.0),
                                                egui::Align2::CENTER_BOTTOM,
                                                format!("U ({:.2}..{:.2})", u_min, u_max),
                                                egui::FontId::proportional(10.0),
                                                egui::Color32::from_rgb(170, 170, 170),
                                            );
                                        }

                                        // SVG export button
                                        ui.add_space(4.0);
                                        #[cfg(not(target_arch = "wasm32"))]
                                        {
                                            ui.horizontal(|ui| {
                                                if ui.button("Save UV as SVG...").clicked() {
                                                    if let Some((_, ref svg_content)) = self.uv_svg_cache {
                                                        if let Some(path) = rfd::FileDialog::new()
                                                            .add_filter("SVG", &["svg"])
                                                            .save_file()
                                                        {
                                                            match std::fs::write(&path, svg_content) {
                                                                Ok(()) => self.log(&format!("Exported UV SVG: {}", path.to_string_lossy())),
                                                                Err(e) => self.log(&format!("SVG export error: {}", e)),
                                                            }
                                                        }
                                                    }
                                                }
                                            });
                                        }
                                    }
                                }
                            } else {
                                ui.label(egui::RichText::new("Select a face to see UV grid").size(11.0).color(egui::Color32::GRAY));
                            }
                        } else {
                            ui.label(egui::RichText::new("Select an instance first").size(11.0).color(egui::Color32::GRAY));
                        }
                    }

                    ui.separator();

                    // ─── Selected Face Info ───────────────────────────────
                    if let Some(inst_idx) = self.selected_instance {
                        if let Some(face_id) = self.selected_face_id {
                            if let Some(inst) = self.detailed_instances.get(inst_idx) {
                                if let Some(face) = inst.faces.iter().find(|f| f.face_id == face_id) {
                                    ui.heading(egui::RichText::new("Face Info").size(13.0));
                                    ui.label(egui::RichText::new(format!("ID: {}", face.face_id)).size(11.0));
                                    ui.label(egui::RichText::new(format!("STEP ID: #{}", face.step_face_id)).size(11.0));
                                    ui.label(egui::RichText::new(format!("Surface: {}", face.surface_type)).size(11.0));
                                    ui.label(egui::RichText::new(format!("Triangles: [{}, {})", face.triangle_range.0, face.triangle_range.1)).size(11.0));
                                    ui.label(egui::RichText::new(format!("Boundary pts: {}", face.outer_boundary.len())).size(11.0));
                                    ui.label(egui::RichText::new(format!("Holes: {}", face.inner_boundaries.len())).size(11.0));
                                    ui.label(egui::RichText::new(format!("Forward: {}", face.forward)).size(11.0));

                                    // Copy face ID to clipboard
                                    if ui.button("Copy Face ID").clicked() {
                                        ui.ctx().copy_text(format!("{}", face.face_id));
                                        self.log(&format!("Copied face ID: {}", face.face_id));
                                    }
                                }
                            }
                        }
                    }
                });
        }

        // === Left side panel (controls) ===
        egui::SidePanel::left("controls")
            .min_width(180.0)
            .default_width(200.0)
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

                #[cfg(not(target_arch = "wasm32"))]
                {
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
                }

                #[cfg(target_arch = "wasm32")]
                {
                    ui.horizontal(|ui| {
                        ui.label("STL:");
                        if ui.button("Open...").clicked() {
                            self.trigger_stl_file_input();
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("STEP:");
                        if ui.button("Open...").clicked() {
                            self.trigger_step_file_input();
                        }
                    });
                }

                ui.add_space(4.0);

                // --- Export ---
                ui.separator();
                ui.heading(egui::RichText::new("Export").size(14.0));

                #[cfg(not(target_arch = "wasm32"))]
                {
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
                }

                #[cfg(target_arch = "wasm32")]
                {
                    ui.label(egui::RichText::new("(Export not available on web)")
                        .size(11.0)
                        .color(egui::Color32::GRAY));
                }

                ui.add_space(4.0);

                // --- Display ---
                ui.separator();
                ui.heading(egui::RichText::new("Display").size(14.0));
                ui.checkbox(&mut self.wireframe, "Wireframe");
                ui.checkbox(&mut self.show_axes, "Show axes");
                ui.checkbox(&mut self.show_grid, "Show grid");
                ui.checkbox(&mut self.show_structure, "Structure Panel");

                if ui.button("Reset Camera").clicked() {
                    let (bbox_min, bbox_max) = self.mesh.bounding_box();
                    self.camera.fit_to_bounding_box(
                        [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                        [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                    );
                }

                // Clear selection button
                if ui.button("Clear Selection").clicked() {
                    self.selected_instance = None;
                    self.selected_face_id = None;
                    self.highlighted_face_id = None;
                    self.highlight_dirty = true;
                    self.uv_svg_cache = None;
                }

                ui.add_space(4.0);

                // --- Info ---
                ui.separator();
                ui.heading(egui::RichText::new("Info").size(14.0));
                ui.label(egui::RichText::new(format!("Model: {}", self.current_model.name)).size(12.0));
                ui.label(egui::RichText::new(format!("Vertices: {}", self.current_model.vertex_count)).size(12.0));
                ui.label(egui::RichText::new(format!("Triangles: {}", self.current_model.triangle_count)).size(12.0));
                ui.label(egui::RichText::new(format!("Instances: {}", self.detailed_instances.len())).size(12.0));

                if let Some(fid) = self.highlighted_face_id {
                    ui.label(egui::RichText::new(format!("Selected face: #{}", fid))
                        .size(12.0)
                        .color(egui::Color32::from_rgb(255, 220, 50)));
                }

                let cam_pos = self.camera.position();
                ui.label(egui::RichText::new(format!("Camera: ({:.0}, {:.0}, {:.0})", cam_pos[0], cam_pos[1], cam_pos[2]))
                    .size(11.0).color(egui::Color32::GRAY));

                ui.add_space(8.0);
                ui.separator();
                ui.label(
                    egui::RichText::new("Mouse: LMB Rotate | Scroll Zoom | MMB Pan")
                        .size(10.0)
                        .color(egui::Color32::from_rgb(160, 160, 160))
                );
            });

        // === Central 3D viewport ===
        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(egui::Color32::from_rgb(230, 230, 230)))
            .show(ctx, |ui| {
                let (rect, response) = ui.allocate_exact_size(
                    ui.available_size(),
                    egui::Sense::click_and_drag(),
                );

                // Handle multi-touch gestures
                let multi_touch = ui.input(|i| i.multi_touch());
                if let Some(touch) = multi_touch {
                    if touch.zoom_delta != 1.0 {
                        let zoom_delta = (touch.zoom_delta - 1.0) * 500.0;
                        self.camera.zoom(zoom_delta, None);
                    }
                    if touch.translation_delta.length() > 0.0 {
                        self.camera.pan(
                            touch.translation_delta.x,
                            touch.translation_delta.y,
                            rect.width(),
                            rect.height(),
                        );
                    }
                    if touch.rotation_delta.abs() > 0.001 {
                        self.camera.rotate(touch.rotation_delta * 50.0, 0.0);
                    }
                } else {
                    let is_hovering = response.hovered();

                    if response.dragged_by(egui::PointerButton::Primary) {
                        let delta = response.drag_delta();
                        self.camera.rotate(delta.x, delta.y);
                    }

                    if response.dragged_by(egui::PointerButton::Middle) {
                        let delta = response.drag_delta();
                        self.camera.pan(delta.x, delta.y, rect.width(), rect.height());
                    }

                    if is_hovering {
                        let scroll = ui.input(|i| i.smooth_scroll_delta);
                        if scroll.y != 0.0 {
                            let mouse_pos_opt = ui.input(|i| i.pointer.latest_pos());
                            let mouse_norm = mouse_pos_opt.map(|pos| {
                                let nx = ((pos.x - rect.center().x) / (rect.width() * 0.5)).clamp(-1.0, 1.0);
                                let ny = -((pos.y - rect.center().y) / (rect.height() * 0.5)).clamp(-1.0, 1.0);
                                [nx, ny]
                            });
                            self.camera.zoom(scroll.y, mouse_norm);
                        }
                    }
                }

                // Get viewport dimensions
                let width = rect.width() as u32;
                let height = rect.height() as u32;

                if width == 0 || height == 0 {
                    return;
                }

                // Upload mesh data if dirty or highlight changed
                if self.mesh_dirty || self.highlight_dirty {
                    if let Some(ref rs) = self.render_state {
                        let (vertices, indices) = mesh_to_gpu_data(&self.mesh, self.highlighted_face_id);
                        let mut guard = self.gpu_resources.lock().unwrap();
                        if let Some(ref mut resources) = *guard {
                            update_mesh_buffers(resources, &rs.device, &vertices, &indices);
                        } else {
                            let resources = create_scene_resources(rs, &vertices, &indices);
                            *guard = Some(resources);
                        }
                    }
                    self.mesh_dirty = false;
                    self.highlight_dirty = false;
                }

                // Update uniforms
                if let Some(ref rs) = self.render_state {
                    let aspect = rect.width() / rect.height();
                    let view = self.camera.view_matrix();
                    let proj = self.camera.projection_matrix(aspect);
                    let mvp = mat4_mul(&proj, &view);
                    let model: [[f32; 4]; 4] = [
                        [1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ];
                    let cam_pos = self.camera.position();
                    let cam_fwd = self.camera.forward();
                    let uniforms = SceneUniforms {
                        mvp,
                        model,
                        light_dir: [cam_fwd[0], cam_fwd[1], cam_fwd[2], 0.45],
                        camera_pos: [cam_pos[0], cam_pos[1], cam_pos[2], 0.0],
                    };
                    let guard = self.gpu_resources.lock().unwrap();
                    if let Some(ref resources) = *guard {
                        update_uniforms(resources, &rs.queue, &uniforms);
                    }
                }

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

                if self.show_axes {
                    self.draw_axes_overlay(ui, rect);
                }
            });
    }
}

impl ViewerApp {
    /// Draw the assembly tree node recursively in the structure panel.
    fn draw_assembly_node(&mut self, ui: &mut egui::Ui, node: &AssemblyNode) {
        let has_children = !node.children.is_empty();
        let brep_str = match node.brep_id {
            Some(id) => format!(" BREP#{}", id),
            None => String::new(),
        };
        let label = format!("{}{}", node.name, brep_str);

        // Check if this node matches a detailed instance
        let is_selected = node.brep_id.map_or(false, |bid| {
            self.detailed_instances.iter()
                .position(|inst| inst.brep_id == bid)
                .map_or(false, |idx| self.selected_instance == Some(idx))
        });

        if has_children {
            egui::CollapsingHeader::new(egui::RichText::new(&label).size(11.0))
                .default_open(false)
                .show(ui, |ui| {
                    for child in &node.children {
                        self.draw_assembly_node(ui, child);
                    }
                });
        } else {
            let response = ui.selectable_label(is_selected, egui::RichText::new(&label).size(11.0));
            if response.clicked() {
                // Find the corresponding detailed instance
                if let Some(bid) = node.brep_id {
                    if let Some(idx) = self.detailed_instances.iter().position(|inst| inst.brep_id == bid) {
                        self.selected_instance = Some(idx);
                        self.selected_face_id = None;
                        self.highlighted_face_id = None;
                        self.highlight_dirty = true;
                        self.uv_svg_cache = None;
                    }
                }
            }
        }
    }

    fn draw_axes_overlay(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        let cam_right = self.camera.right();
        let cam_up = self.camera.up();

        let axis_len = 50.0;
        let axes: [([f32; 3], egui::Color32, &str); 3] = [
            ([1.0, 0.0, 0.0], egui::Color32::RED, "X"),
            ([0.0, 1.0, 0.0], egui::Color32::GREEN, "Y"),
            ([0.0, 0.0, 1.0], egui::Color32::BLUE, "Z"),
        ];

        let origin_x = rect.left() + 60.0;
        let origin_y = rect.bottom() - 60.0;

        for (dir, color, label) in axes {
            let sx = (dir[0] * cam_right[0] + dir[1] * cam_right[1] + dir[2] * cam_right[2]) * axis_len;
            let sy = (dir[0] * cam_up[0] + dir[1] * cam_up[1] + dir[2] * cam_up[2]) * axis_len;

            let end_x = origin_x + sx;
            let end_y = origin_y - sy;

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
