// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
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
use draper_mesh::{triangulate_solid, TriangleMesh, TriangulationParams, check_manifold, ManifoldReport};
use draper_step::{AssemblyNode, DetailedMeshInstance, FaceInfo};
use draper_geometry::{Surface, Point2d};
use egui_wgpu::RenderState;
use eframe::egui;

/// Convert TriangleMesh to GPU vertex/index data.
/// Uses flat shading with face normals to properly support per-triangle colors from STEP files.
/// Selection state is encoded as per-vertex attributes and applied AFTER lighting in the shader:
///   selection: 0 = normal, 1 = selected instance, 2 = dimmed instance
///   highlight: 0 = normal face, 1 = highlighted face
fn mesh_to_gpu_data(
    mesh: &TriangleMesh,
    highlighted_face: Option<(usize, u64)>,
    selected_instance: Option<usize>,
    instance_triangle_ranges: &[(usize, usize)],
) -> (Vec<MeshVertex>, Vec<u32>) {
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

    // Determine if we need per-triangle processing (face highlight, instance selection, or colors)
    let needs_per_tri = highlighted_face.is_some() || selected_instance.is_some() || has_real_colors;

    // If we have vertex normals and no special per-triangle processing, use smooth shading
    if let Some(ref vertex_normals) = mesh.normals {
        if vertex_normals.len() == mesh.vertices.len() && !needs_per_tri {
            let mut gpu_vertices = Vec::with_capacity(mesh.vertices.len());
            let mut gpu_indices = Vec::with_capacity(mesh.triangles.len() * 3);

            for (i, v) in mesh.vertices.iter().enumerate() {
                let n = vertex_normals.get(i).map(|nn| [nn[0] as f32, nn[1] as f32, nn[2] as f32]).unwrap_or([0.0, 0.0, 1.0]);
                gpu_vertices.push(MeshVertex {
                    position: [v.x as f32, v.y as f32, v.z as f32],
                    normal: n,
                    color: [0.48, 0.52, 0.58],
                    selection: 0.0,
                    highlight: 0.0,
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

        // Use original base color — never modify it for selection state
        let color = colors
            .and_then(|c| c.get(i))
            .map(|c| [c[0], c[1], c[2]])
            .unwrap_or([0.48, 0.52, 0.58]);

        // Compute per-triangle selection state (applied AFTER lighting in shader)
        // selection: 0 = normal, 1 = selected instance, 2 = dimmed instance
        let mut selection = 0.0_f32;
        let mut is_highlighted = false;

        // Determine instance selection state
        if let Some(sel_idx) = selected_instance {
            if let Some(&(start, end)) = instance_triangle_ranges.get(sel_idx) {
                if i >= start && i < end {
                    // This triangle belongs to the selected instance
                    selection = 1.0;
                } else {
                    // This triangle belongs to a non-selected instance
                    selection = 2.0;
                }
            }
            // If instance_triangle_ranges doesn't have this index, selection stays 0.0 (normal)
        }

        // Check face-level highlighting (instance-aware)
        // highlighted_face = (instance_index, face_id) — only highlight within the correct instance
        if let Some((hl_inst, hl_fid)) = highlighted_face {
            if let Some(&(start, end)) = instance_triangle_ranges.get(hl_inst) {
                if i >= start && i < end {
                    if let Some(ids) = face_ids {
                        if ids.get(i).map_or(false, |id| *id == hl_fid) {
                            is_highlighted = true;
                        }
                    }
                }
            }
        }

        let highlight = if is_highlighted { 1.0 } else { 0.0 };

        let base_idx = gpu_vertices.len() as u32;
        for &idx in tri {
            let v = &mesh.vertices[idx as usize];
            gpu_vertices.push(MeshVertex {
                position: [v.x as f32, v.y as f32, v.z as f32],
                normal,
                color,
                selection,
                highlight,
            });
        }
        gpu_indices.push(base_idx);
        gpu_indices.push(base_idx + 1);
        gpu_indices.push(base_idx + 2);
    }

    (gpu_vertices, gpu_indices)
}

/// Result of a mouse pick operation.
#[derive(Clone, Debug)]
struct PickResult {
    /// Index of the instance that was hit (matches instance_triangle_ranges index).
    instance_idx: usize,
    /// Face ID (TopoId) of the triangle that was hit, if available.
    face_id: Option<u64>,
    /// Distance along the ray to the hit point (for depth sorting).
    distance: f32,
}

/// Möller–Trumbore ray-triangle intersection.
/// Returns the distance `t` along the ray if hit, or None.
fn ray_triangle_intersect(
    ray_origin: [f32; 3],
    ray_dir: [f32; 3],
    v0: [f32; 3],
    v1: [f32; 3],
    v2: [f32; 3],
) -> Option<f32> {
    const EPSILON: f32 = 1e-7;
    let edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
    let edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

    let h = [
        ray_dir[1] * edge2[2] - ray_dir[2] * edge2[1],
        ray_dir[2] * edge2[0] - ray_dir[0] * edge2[2],
        ray_dir[0] * edge2[1] - ray_dir[1] * edge2[0],
    ];
    let a = edge1[0] * h[0] + edge1[1] * h[1] + edge1[2] * h[2];
    if a.abs() < EPSILON {
        return None; // ray parallel to triangle
    }
    let f = 1.0 / a;
    let s = [ray_origin[0] - v0[0], ray_origin[1] - v0[1], ray_origin[2] - v0[2]];
    let u = f * (s[0] * h[0] + s[1] * h[1] + s[2] * h[2]);
    if u < 0.0 || u > 1.0 {
        return None;
    }
    let q = [
        s[1] * edge1[2] - s[2] * edge1[1],
        s[2] * edge1[0] - s[0] * edge1[2],
        s[0] * edge1[1] - s[1] * edge1[0],
    ];
    let v = f * (ray_dir[0] * q[0] + ray_dir[1] * q[1] + ray_dir[2] * q[2]);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * (edge2[0] * q[0] + edge2[1] * q[1] + edge2[2] * q[2]);
    if t > EPSILON {
        Some(t)
    } else {
        None
    }
}

/// Pick the closest triangle under the given screen position.
/// Returns the instance index and face ID of the hit triangle, or None if nothing was hit.
fn pick_at(
    mesh: &TriangleMesh,
    instance_triangle_ranges: &[(usize, usize)],
    camera: &OrbitCamera,
    screen_pos: [f32; 2],
    viewport: (f32, f32, f32, f32),
) -> Option<PickResult> {
    let (ray_origin, ray_dir) = camera.screen_to_ray(screen_pos, viewport);

    let face_ids = mesh.triangle_face_ids.as_ref();
    let mut best: Option<PickResult> = None;

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices.get(tri[0] as usize);
        let v1 = mesh.vertices.get(tri[1] as usize);
        let v2 = mesh.vertices.get(tri[2] as usize);
        let (v0, v1, v2) = match (v0, v1, v2) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            _ => continue,
        };

        if let Some(t) = ray_triangle_intersect(
            ray_origin,
            ray_dir,
            [v0.x as f32, v0.y as f32, v0.z as f32],
            [v1.x as f32, v1.y as f32, v1.z as f32],
            [v2.x as f32, v2.y as f32, v2.z as f32],
        ) {
            let dist = best.as_ref().map_or(f32::MAX, |b| b.distance);
            if t < dist {
                // Determine which instance this triangle belongs to
                let instance_idx = instance_triangle_ranges
                    .iter()
                    .position(|&(start, end)| i >= start && i < end)
                    .unwrap_or(0);

                let face_id = face_ids.and_then(|ids| ids.get(i)).copied();

                best = Some(PickResult {
                    instance_idx,
                    face_id,
                    distance: t,
                });
            }
        }
    }

    best
}

/// Model entry for the scene.
#[derive(Clone, Debug)]
pub struct ModelEntry {
    pub name: String,
    pub vertex_count: usize,
    pub triangle_count: usize,
}

/// Severity level for log entries.
#[derive(Clone, Debug, PartialEq, Eq)]
enum LogSeverity {
    Info,
    Warning,
    Error,
}

/// Log entry with timestamp and severity.
#[derive(Clone, Debug)]
struct LogEntry {
    time: String,
    message: String,
    severity: LogSeverity,
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
    /// Currently selected face: (instance_index, face_id_within_instance).
    selected_face: Option<(usize, u64)>,
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
    uv_svg_cache: Option<((usize, u64), String)>, // ((instance_idx, face_id), svg_content)

    // ─── Face highlight state ──────────────────────────────────────
    /// Currently highlighted face: (instance_index, face_id_within_instance).
    highlighted_face: Option<(usize, u64)>,
    /// Whether the GPU data needs update due to highlight change.
    highlight_dirty: bool,

    // ─── Instance-level highlight state ────────────────────────────
    /// Per-instance triangle ranges in the merged mesh: Vec<(start_tri, end_tri)>
    /// When an instance is selected, triangles outside its range are dimmed.
    instance_triangle_ranges: Vec<(usize, usize)>,
    // ─── Tree navigation state ────────────────────────────────────────
    /// Node keys ("name_pd_id") that should be forced open in the assembly tree.
    open_tree_nodes: std::collections::HashSet<String>,
    /// Node key to scroll to in the assembly tree (set when selecting from 3D view).
    scroll_to_tree_node: Option<String>,
    /// Face ID to scroll to in the face list (set when selecting a face from 3D view).
    scroll_to_face_id: Option<u64>,

    // ─── Progressive loading state ────────────────────────────────────
    /// Pending instances to triangulate (populated during parse phase, consumed during render phase).
    pending_instances: Vec<DetailedMeshInstance>,
    /// Total number of instances being loaded (for progress display).
    total_instance_count: usize,
    /// Number of instances already triangulated.
    triangulated_count: usize,
    /// Whether we are currently in the progressive triangulation phase.
    is_loading: bool,
    /// Name of the file being loaded.
    loading_name: String,
    /// How many instances to triangulate per frame (adaptive).
    instances_per_frame: usize,

    // ─── Manifold statistics ──────────────────────────────────────────
    /// Manifold report for the current mesh (computed on load).
    manifold_report: Option<ManifoldReport>,

    // ─── Panel collapse state (for mobile optimization) ──────────────
    /// Whether the left controls panel is open.
    controls_panel_open: bool,
    /// Whether the structure tree section is expanded.
    structure_tree_open: bool,
    /// Whether the face list section is expanded.
    face_list_open: bool,
    /// Whether the UV grid section is expanded.
    uv_grid_open: bool,
    /// Whether the face info section is expanded.
    face_info_open: bool,

    // ─── Error tracking ──────────────────────────────────────────────────
    /// Last error message (for dedicated "Errors" UI section).
    last_error: Option<String>,
    /// Number of warnings encountered during the current session.
    warning_count: usize,
    /// Number of errors encountered during the current session.
    error_count: usize,
    /// Number of faces that failed triangulation (for graceful degradation).
    failed_face_count: usize,
}

impl ViewerApp {
    /// Get the current timestamp as a formatted string.
    fn timestamp() -> String {
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
        format!("{:02}:{:02}:{:02}", (now / 3600) % 24, mins, secs)
    }

    /// Log an informational message.
    fn log(&mut self, msg: &str) {
        let time = Self::timestamp();
        self.log.push(LogEntry {
            time,
            message: msg.to_string(),
            severity: LogSeverity::Info,
        });
        // Keep last 500 entries
        if self.log.len() > 500 {
            self.log.drain(0..self.log.len() - 500);
        }
        self.log_auto_scroll = true;
    }

    /// Log a warning message (yellow in log panel).
    fn log_warning(&mut self, msg: &str) {
        let time = Self::timestamp();
        self.log.push(LogEntry {
            time,
            message: msg.to_string(),
            severity: LogSeverity::Warning,
        });
        self.warning_count += 1;
        if self.log.len() > 500 {
            self.log.drain(0..self.log.len() - 500);
        }
        self.log_auto_scroll = true;
    }

    /// Log an error message (red in log panel) and update the error section.
    fn log_error(&mut self, msg: &str) {
        let time = Self::timestamp();
        self.log.push(LogEntry {
            time,
            message: msg.to_string(),
            severity: LogSeverity::Error,
        });
        self.last_error = Some(msg.to_string());
        self.error_count += 1;
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
            let (vertices, indices) = mesh_to_gpu_data(&mesh, None, None, &[]);
            let resources = create_scene_resources(rs, &vertices, &indices);
            *gpu_resources.lock().unwrap() = Some(resources);
        }

        #[cfg(target_arch = "wasm32")]
        let file_result = Arc::new(Mutex::new(None));

        // Compute manifold report for initial mesh before moving it
        let manifold_report = Some(check_manifold(&mesh));

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
            selected_face: None,
            show_structure: true,
            show_uv_grid: false,
            uv_grid_u: 10,
            uv_grid_v: 10,
            uv_svg_cache: None,
            highlighted_face: None,
            highlight_dirty: false,
            instance_triangle_ranges: Vec::new(),
            open_tree_nodes: std::collections::HashSet::new(),
            scroll_to_tree_node: None,
            scroll_to_face_id: None,
            pending_instances: Vec::new(),
            total_instance_count: 0,
            triangulated_count: 0,
            is_loading: false,
            loading_name: String::new(),
            instances_per_frame: 1,
            manifold_report,
            controls_panel_open: true,
            structure_tree_open: true,
            face_list_open: true,
            uv_grid_open: false,
            face_info_open: false,
            last_error: None,
            warning_count: 0,
            error_count: 0,
            failed_face_count: 0,
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
        // Compute manifold statistics for the new mesh
        let report = check_manifold(&mesh);
        let is_watertight = report.is_watertight();
        self.manifold_report = Some(report);

        self.mesh = mesh;
        self.mesh_dirty = true;
        // Reset selection when loading new model
        self.selected_instance = None;
        self.selected_face = None;
        self.highlighted_face = None;
        self.highlight_dirty = true;
        self.uv_svg_cache = None;
        self.open_tree_nodes.clear();
        self.scroll_to_tree_node = None;
        self.scroll_to_face_id = None;
        self.log(&format!("Loaded: {} ({} vertices, {} triangles) — {}",
            name, self.current_model.vertex_count, self.current_model.triangle_count,
            if is_watertight { "watertight" } else { "not watertight" }));
    }

    fn load_box(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Box 100x80x60");
    }

    fn load_cylinder(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cylinder R=40 H=100");
    }

    fn load_sphere(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
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
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cone R=40 H=80");
    }

    fn load_torus(&mut self) {
        let solid = ShapeBuilder::make_torus(40.0, 12.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Torus R=40 r=12");
    }

    fn load_engine(&mut self) {
        let doc = build_engine(&EngineConfig::default());
        let mesh = doc.triangulate();
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
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
        self.instance_triangle_ranges.clear();
                self.assembly_tree = None;
                self.load_mesh(mesh, &format!("STL: {}", name));
            }
            Err(e) => {
                self.log_error(&format!("STL import error: {}", e));
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
                self.log_error(&format!("STEP import error: {}", e));
            }
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn export_stl_binary(&mut self, path: &str) {
        match draper_mesh::stl::write_stl_file(&self.mesh, path, true) {
            Ok(()) => self.log(&format!("Exported STL (binary): {}", path)),
            Err(e) => self.log_error(&format!("STL export error: {}", e)),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn export_stl_ascii(&mut self, path: &str) {
        match draper_mesh::stl::write_stl_file(&self.mesh, path, false) {
            Ok(()) => self.log(&format!("Exported STL (ASCII): {}", path)),
            Err(e) => self.log_error(&format!("STL export error: {}", e)),
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
            Err(e) => self.log_error(&format!("STEP export error: {}", e)),
        }
    }

    // ─── Shared file processing (used by both native and web) ─────────────

    /// Process a parsed STEP file — Phase 1: Parse + Build tree (fast).
    /// The tree is shown immediately. Triangulation happens progressively in update().
    fn process_step_file(&mut self, step_file: &draper_step::StepFile, name: &str) {
        // Cancel any previous loading
        self.cancel_loading();

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

        // Build assembly tree AND detailed instances together so that
        // instance_index is properly populated in the assembly tree
        let (tree, instances) = draper_step::step_structure_with_instances(step_file);
        self.assembly_tree = Some(tree);
        self.show_structure = true;

        // ── Set up progressive triangulation ──
        if !instances.is_empty() {
            self.log(&format!("Triangulating {} instances...", instances.len()));
            self.total_instance_count = instances.len();
            self.triangulated_count = 0;
            self.pending_instances = instances;
            self.is_loading = true;
            self.loading_name = name.to_string();
            self.instances_per_frame = 1;

            // Clear existing rendering data
            self.detailed_instances.clear();
            self.instance_triangle_ranges.clear();
            self.selected_instance = None;
            self.selected_face = None;
            self.highlighted_face = None;

            // Start with empty mesh — will be built progressively
            self.mesh = TriangleMesh::new();
            self.mesh_dirty = true;

            // Auto-fit camera once tree is ready (even before mesh)
            self.log("Structure tree ready — triangulation in progress...");
        } else {
            // Fallback: try separate conversion (no progressive loading for fallback)
            self.log("step_structure_with_instances returned no instances, trying separate conversion");
            self.assembly_tree = Some(draper_step::step_structure(step_file));
            match draper_step::step_to_detailed_instances(step_file) {
                Ok(instances) => {
                    self.log(&format!("Mesh Rendering Tree ({} instances)", instances.len()));
                    self.detailed_instances = instances.clone();
                    let mut mesh = TriangleMesh::new();
                    let mut instance_ranges = Vec::new();
                    for inst in &instances {
                        let tri_start = mesh.triangle_count();
                        if let Some(color) = inst.color {
                            mesh.merge_with_color(&inst.mesh, color);
                        } else {
                            mesh.merge_with_color(&inst.mesh, [0.48, 0.52, 0.58, 1.0]);
                        }
                        let tri_end = mesh.triangle_count();
                        instance_ranges.push((tri_start, tri_end));
                    }
                    self.instance_triangle_ranges = instance_ranges;
                    let vcount = mesh.vertex_count();
                    let tcount = mesh.triangle_count();
                    self.log(&format!("Total merged: {} vertices, {} triangles", vcount, tcount));
                    self.load_mesh(mesh, &format!("STEP: {}", name));
                }
                Err(e) => {
                    self.log_warning(&format!("STEP detailed conversion error: {}, trying simple conversion", e));
                    match draper_step::step_to_mesh_instances(step_file) {
                        Ok(instances) => {
                            self.log(&format!("Simple Mesh Instances: {}", instances.len()));
                            let mut mesh = TriangleMesh::new();
                            for inst in &instances {
                                if let Some(color) = inst.color {
                                    mesh.merge_with_color(&inst.mesh, color);
                                } else {
                                    mesh.merge_with_color(&inst.mesh, [0.48, 0.52, 0.58, 1.0]);
                                }
                            }
                            self.detailed_instances.clear();
                            self.instance_triangle_ranges.clear();
                            self.load_mesh(mesh, &format!("STEP: {}", name));
                        }
                        Err(e2) => {
                            self.log_error(&format!("STEP conversion error: {}", e2));
                        }
                    }
                }
            }
        }
    }

    /// Cancel any in-progress loading.
    fn cancel_loading(&mut self) {
        self.is_loading = false;
        self.pending_instances.clear();
        self.triangulated_count = 0;
        self.total_instance_count = 0;
    }

    /// Process one batch of pending instances (called each frame from update()).
    /// Returns true if there are still more instances to process.
    /// Gracefully handles triangulation failures: logs a warning and skips
    /// instances with empty meshes instead of crashing.
    fn process_pending_instances(&mut self) -> bool {
        if !self.is_loading || self.pending_instances.is_empty() {
            return false;
        }

        let batch_size = self.instances_per_frame.min(self.pending_instances.len());
        for _ in 0..batch_size {
            if let Some(inst) = self.pending_instances.pop() {
                // Check if this instance has a valid mesh (non-empty triangulation)
                if inst.mesh.triangle_count() == 0 && inst.mesh.vertex_count() == 0 {
                    self.log_warning(&format!(
                        "Instance '{}' (BREP #{}) produced empty mesh — skipping",
                        inst.name, inst.brep_id
                    ));
                    self.failed_face_count += 1;
                    self.triangulated_count += 1;
                    continue;
                }

                let tri_start = self.mesh.triangle_count();
                if let Some(color) = inst.color {
                    self.mesh.merge_with_color(&inst.mesh, color);
                } else {
                    self.mesh.merge_with_color(&inst.mesh, [0.48, 0.52, 0.58, 1.0]);
                }
                let tri_end = self.mesh.triangle_count();
                self.instance_triangle_ranges.push((tri_start, tri_end));

                let inst_idx = self.triangulated_count;
                self.detailed_instances.push(inst);
                self.triangulated_count += 1;

                // Update the assembly tree: link this instance to its tree node
                if let Some(ref mut tree) = self.assembly_tree {
                    assign_instance_to_tree(tree, inst_idx);
                }
            }
        }

        self.mesh_dirty = true;

        // Adaptive: process more per frame if instances are small
        if self.triangulated_count > 0 && self.triangulated_count % 5 == 0 {
            let avg_triangles = self.mesh.triangle_count() / self.triangulated_count;
            if avg_triangles < 1000 {
                self.instances_per_frame = (self.instances_per_frame + 1).min(10);
            }
        }

        if self.pending_instances.is_empty() {
            // Loading complete
            self.is_loading = false;
            let vcount = self.mesh.vertex_count();
            let tcount = self.mesh.triangle_count();
            self.log(&format!(
                "Triangulation complete: {} instances, {} vertices, {} triangles",
                self.triangulated_count, vcount, tcount
            ));
            self.load_mesh(self.mesh.clone(), &format!("STEP: {}", self.loading_name));
            self.loading_name.clear();
            return false;
        }
        true
    }

    /// Import STL from bytes (used by web file loading).
    fn import_stl_from_bytes(&mut self, data: &[u8], name: &str) {
        match draper_mesh::import_stl_from_bytes(data) {
            Ok(mesh) => {
                self.load_mesh(mesh, &format!("STL: {}", name));
            }
            Err(e) => {
                self.log_error(&format!("STL import error: {}", e));
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
                self.log_error(&format!("STEP import error: {}", e));
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
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Request repaint for continuous rendering
        ctx.request_repaint();

        // Process any pending web file loads
        #[cfg(target_arch = "wasm32")]
        self.process_web_file_loads();

        // Process progressive triangulation (one batch per frame)
        if self.is_loading {
            self.process_pending_instances();
            ctx.request_repaint(); // Keep repainting during loading
        }

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
                    // Show warning/error counts as badges
                    if self.warning_count > 0 {
                        ui.label(egui::RichText::new(format!("⚠ {}", self.warning_count))
                            .size(11.0)
                            .color(egui::Color32::from_rgb(255, 200, 50)));
                    }
                    if self.error_count > 0 {
                        ui.label(egui::RichText::new(format!("✖ {}", self.error_count))
                            .size(11.0)
                            .color(egui::Color32::from_rgb(255, 80, 80)));
                    }
                    ui.separator();
                    if ui.button("Clear").clicked() {
                        self.log.clear();
                    }
                    if ui.button("Copy All").clicked() {
                        let all_text: String = self.log.iter()
                            .map(|e| {
                                let prefix = match e.severity {
                                    LogSeverity::Info => "INFO",
                                    LogSeverity::Warning => "WARN",
                                    LogSeverity::Error => "ERR",
                                };
                                format!("[{}] [{}] {}", e.time, prefix, e.message)
                            })
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
                            // Color coding based on severity
                            let (icon, msg_color) = match entry.severity {
                                LogSeverity::Info => ("ℹ", egui::Color32::from_rgb(200, 200, 210)),
                                LogSeverity::Warning => ("⚠", egui::Color32::from_rgb(255, 200, 50)),
                                LogSeverity::Error => ("✖", egui::Color32::from_rgb(255, 80, 80)),
                            };
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new(format!("[{}]", entry.time))
                                    .size(10.0)
                                    .color(egui::Color32::from_rgb(120, 120, 140)));
                                ui.label(egui::RichText::new(icon)
                                    .size(10.0)
                                    .color(msg_color));
                                ui.add(egui::Label::new(
                                    egui::RichText::new(&entry.message).size(10.0).color(msg_color)
                                ).wrap());
                            });
                        }
                    });
            });

        // === Right panel: Structure / Faces / UV ===
        // Collect pending UI actions to avoid borrow checker conflicts
        let mut pending_instance_select: Option<usize> = None;
        let mut pending_face_select: Option<(usize, u64)> = None;
        let mut pending_svg_export = false;
        let mut pending_copy_face_id: Option<u64> = None;

        if self.show_structure {
            // Clone data needed for drawing to avoid borrow conflicts
            let assembly_tree_clone = self.assembly_tree.clone();
            let detailed_instances_clone = self.detailed_instances.clone();
            let selected_instance = self.selected_instance;
            let selected_face = self.selected_face;
            let uv_grid_u = self.uv_grid_u;
            let uv_grid_v = self.uv_grid_v;
            let show_uv_grid = self.show_uv_grid;
            let uv_svg_cache_key = self.uv_svg_cache.as_ref().map(|(key, _)| *key);
            let open_tree_nodes = self.open_tree_nodes.clone();
            let scroll_to_tree_node = self.scroll_to_tree_node.clone();
            let scroll_to_face_id = self.scroll_to_face_id;

            egui::SidePanel::right("structure_panel")
                .min_width(220.0)
                .default_width(280.0)
                .resizable(true)
                .show(ctx, |ui| {
                    ui.add_space(2.0);

                    // ─── Loading progress in panel header ───
                    if self.is_loading && self.total_instance_count > 0 {
                        let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                        ui.horizontal(|ui| {
                            ui.heading(egui::RichText::new("Structure").size(14.0));
                            ui.label(egui::RichText::new(format!("({:.0}%)", progress * 100.0))
                                .size(11.0).color(egui::Color32::from_rgb(80, 180, 80)));
                        });
                        let avail = ui.available_width();
                        ui.add(egui::ProgressBar::new(progress)
                            .desired_width(avail)
                            .show_percentage());
                    } else {
                        ui.heading(egui::RichText::new("Structure").size(14.0));
                    }
                    ui.separator();

                    // ─── Assembly Tree (collapsible) ──────────────────────────────────
                    let tree_id = ui.make_persistent_id("structure_tree_section");
                    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), tree_id, self.structure_tree_open)
                        .show_header(ui, |ui| {
                            ui.heading(egui::RichText::new("▼ Tree").size(12.0));
                        }).body(|ui| {
                            egui::ScrollArea::vertical()
                                .max_height(300.0)
                                .show(ui, |ui| {
                                    if let Some(ref tree) = assembly_tree_clone {
                                        draw_assembly_node_static(ui, tree, selected_instance, &mut pending_instance_select, &open_tree_nodes, &scroll_to_tree_node);
                                    } else if !detailed_instances_clone.is_empty() {
                                        for (i, inst) in detailed_instances_clone.iter().enumerate() {
                                            let is_selected = selected_instance == Some(i);
                                            let label = format!("{} (BREP#{})", inst.name, inst.brep_id);
                                            if ui.selectable_label(is_selected, &label).clicked() {
                                                pending_instance_select = Some(i);
                                            }
                                        }
                                    } else {
                                        ui.label(egui::RichText::new("No STEP file loaded").size(11.0).color(egui::Color32::GRAY));
                                    }
                                });
                        });

                    ui.separator();

                    // ─── Face List (collapsible) ─────────────────────────────────────
                    let face_id = ui.make_persistent_id("face_list_section");
                    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), face_id, self.face_list_open)
                        .show_header(ui, |ui| {
                            let label = if let Some(inst_idx) = selected_instance {
                                if let Some(inst) = detailed_instances_clone.get(inst_idx) {
                                    format!("▼ Faces: {} (BREP #{})", inst.name, inst.brep_id)
                                } else {
                                    "▼ Faces".to_string()
                                }
                            } else {
                                "▼ Faces (select instance)".to_string()
                            };
                            ui.heading(egui::RichText::new(label).size(12.0));
                        }).body(|ui| {
                            if let Some(inst_idx) = selected_instance {
                                if let Some(inst) = detailed_instances_clone.get(inst_idx) {
                                    ui.label(egui::RichText::new(format!("BREP #{} — {} faces", inst.brep_id, inst.faces.len()))
                                        .size(11.0).color(egui::Color32::GRAY));

                                    egui::ScrollArea::vertical()
                                        .id_salt("face_list_scroll")
                                        .max_height(250.0)
                                        .show(ui, |ui| {
                                            for face in &inst.faces {
                                                let is_selected = selected_face == Some((inst_idx, face.face_id));
                                                let label = format!("F#{} STEP#{} {} [{}..{}]",
                                                    face.face_id, face.step_face_id, face.surface_type,
                                                    face.triangle_range.0, face.triangle_range.1);
                                                let response = ui.selectable_label(is_selected, &label);
                                                if scroll_to_face_id == Some(face.face_id) {
                                                    response.scroll_to_me(Some(egui::Align::Center));
                                                }
                                                if response.clicked() {
                                                    pending_face_select = Some((inst_idx, face.face_id));
                                                }
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
                        });

                    ui.separator();

                    // ─── UV Grid Controls (collapsible) ────────────────────────────────
                    let uv_id = ui.make_persistent_id("uv_grid_section");
                    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), uv_id, self.uv_grid_open)
                        .show_header(ui, |ui| {
                            ui.heading(egui::RichText::new("▼ UV Grid").size(12.0));
                        }).body(|ui| {
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
                    if show_uv_grid {
                        if let Some(inst_idx) = selected_instance {
                            if let Some((_, face_id)) = selected_face {
                                if let Some(inst) = detailed_instances_clone.get(inst_idx) {
                                    if let Some(face) = inst.faces.iter().find(|f| f.face_id == face_id) {
                                        // Check cache
                                        let cache_key = (inst_idx, face_id);
                                        let needs_regen = uv_svg_cache_key != Some(cache_key);
                                        if needs_regen {
                                            let svg = generate_uv_svg(face, uv_grid_u, uv_grid_v);
                                            self.uv_svg_cache = Some((cache_key, svg));
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
                                            let u_divs = uv_grid_u.min(50);
                                            let v_divs = uv_grid_v.min(50);
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

                                            // Build a combined outer boundary polygon for point-in-polygon test
                                            let outer_uv_poly: Vec<(f64, f64)> = face.outer_uv_boundary.iter()
                                                .flat_map(|pl| pl.iter().map(|pt| (pt.u, pt.v)))
                                                .collect();

                                            // Draw grid intersection points (only inside boundary)
                                            for i in 0..=u_divs {
                                                for j in 0..=v_divs {
                                                    let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                                                    let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                                                    let pt3d = face.surface.point_at(u, v);
                                                    if pt3d.x.is_finite() && pt3d.y.is_finite() && pt3d.z.is_finite() {
                                                        // Only draw dot if inside the outer boundary polygon
                                                        let inside = !outer_uv_poly.is_empty() && point_in_polygon(u, v, &outer_uv_poly);
                                                        if inside {
                                                            let x = map_u(u);
                                                            let y = map_v(v);
                                                            ui.painter().circle_filled(
                                                                egui::pos2(x, y), 2.0,
                                                                egui::Color32::from_rgba_premultiplied(102, 136, 255, 180),
                                                            );
                                                        }
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
                                                    pending_svg_export = true;
                                                }
                                            });
                                        }

                                        // SVG export on web (download)
                                        #[cfg(target_arch = "wasm32")]
                                        {
                                            ui.add_space(4.0);
                                            if ui.button("Download UV as SVG").clicked() {
                                                pending_svg_export = true;
                                            }
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
                        }); // close UV Grid .body()

                    ui.separator();

                    // ─── Selected Face Info (collapsible) ───────────────────────────────
                    let info_id = ui.make_persistent_id("face_info_section");
                    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), info_id, self.face_info_open)
                        .show_header(ui, |ui| {
                            ui.heading(egui::RichText::new("▼ Face Info").size(12.0));
                        }).body(|ui| {
                            if let Some(inst_idx) = selected_instance {
                                if let Some((_, fid)) = selected_face {
                                    if let Some(inst) = detailed_instances_clone.get(inst_idx) {
                                        if let Some(face) = inst.faces.iter().find(|f| f.face_id == fid) {
                                            ui.label(egui::RichText::new(format!("ID: {}", face.face_id)).size(11.0));
                                            ui.label(egui::RichText::new(format!("STEP ID: #{}", face.step_face_id)).size(11.0));
                                            ui.label(egui::RichText::new(format!("Surface: {}", face.surface_type)).size(11.0));
                                            ui.label(egui::RichText::new(format!("Triangles: [{}, {})", face.triangle_range.0, face.triangle_range.1)).size(11.0));
                                            ui.label(egui::RichText::new(format!("Boundary pts: {}", face.outer_boundary.len())).size(11.0));
                                            ui.label(egui::RichText::new(format!("Holes: {}", face.inner_boundaries.len())).size(11.0));
                                            ui.label(egui::RichText::new(format!("Forward: {}", face.forward)).size(11.0));

                                            if ui.button("Copy Face ID").clicked() {
                                                pending_copy_face_id = Some(face.face_id);
                                            }
                                        }
                                    }
                                } else {
                                    ui.label(egui::RichText::new("Select a face").size(11.0).color(egui::Color32::GRAY));
                                }
                            } else {
                                ui.label(egui::RichText::new("Select an instance").size(11.0).color(egui::Color32::GRAY));
                            }
                        });
                });
            // Clear scroll/open targets after the tree has been rendered.
            // The CollapsingState::set_open calls above already persisted the open state
            // in egui's memory, so users can freely toggle nodes afterwards.
            self.scroll_to_tree_node = None;
            self.scroll_to_face_id = None;
            self.open_tree_nodes.clear();
        }

        // Apply pending UI actions (after all borrows are released)
        if let Some(idx) = pending_instance_select {
            self.selected_instance = Some(idx);
            self.selected_face = None;
            self.highlighted_face = None;
            self.highlight_dirty = true;
            self.uv_svg_cache = None;
            // Find the path to this instance in the assembly tree and open it
            if let Some(ref tree) = self.assembly_tree {
                let (path, target) = find_instance_path(tree, idx);
                self.open_tree_nodes = path.into_iter().collect();
                self.scroll_to_tree_node = target;
            }
        }
        if let Some((inst_idx, fid)) = pending_face_select {
            self.selected_instance = Some(inst_idx);
            self.selected_face = Some((inst_idx, fid));
            self.highlighted_face = Some((inst_idx, fid));
            self.highlight_dirty = true;
            self.uv_svg_cache = None;
            self.scroll_to_face_id = Some(fid);
            self.log(&format!("Selected face #{} in instance #{}", fid, inst_idx));
            // Find the path to this instance in the assembly tree and open it
            if let Some(ref tree) = self.assembly_tree {
                let (path, target) = find_instance_path(tree, inst_idx);
                self.open_tree_nodes = path.into_iter().collect();
                self.scroll_to_tree_node = target;
            }
        }
        if pending_svg_export {
            if let Some((_, ref svg_content)) = self.uv_svg_cache {
                #[cfg(not(target_arch = "wasm32"))]
                {
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
                #[cfg(target_arch = "wasm32")]
                {
                    // Download SVG via browser
                    use wasm_bindgen::prelude::*;
                    if let Some(window) = web_sys::window() {
                        if let Some(document) = window.document() {
                            let blob = web_sys::Blob::new_with_str_sequence(
                                &js_sys::Array::of1(&JsValue::from_str(svg_content)),
                            ).ok();
                            if let Some(blob) = blob {
                                let url = web_sys::Url::create_object_url_with_blob(&blob).ok();
                                if let Some(url) = url {
                                    let a = document.create_element("a").ok();
                                    if let Some(a) = a {
                                        let _ = a.set_attribute("href", &url);
                                        let _ = a.set_attribute("download", "uv_grid.svg");
                                        let _ = a.set_attribute("style", "display:none");
                                        if let Some(body) = document.body() {
                                            let _ = body.append_child(&a);
                                            let html_elem: web_sys::HtmlElement = a.unchecked_into();
                                            html_elem.click();
                                        }
                                    }
                                    web_sys::Url::revoke_object_url(&url).ok();
                                }
                            }
                        }
                    }
                    self.log("Exported UV SVG (download)");
                }
            }
        }
        if let Some(fid) = pending_copy_face_id {
            ctx.copy_text(format!("{}", fid));
            self.log(&format!("Copied face ID: {}", fid));
        }

        // === Left side panel (controls) ===
        egui::SidePanel::left("controls")
            .min_width(150.0)
            .default_width(180.0)
            .show(ctx, |ui| {
                ui.add_space(4.0);
                ui.heading(egui::RichText::new("3Draper").size(14.0));
                ui.label(
                    egui::RichText::new("3D Geometric Kernel")
                        .size(10.0)
                        .color(egui::Color32::GRAY)
                );
                ui.separator();

                // --- Primitives ---
                ui.heading(egui::RichText::new("Primitives").size(12.0));
                ui.horizontal(|ui| {
                    if ui.button("Box").clicked() { self.load_box(); }
                    if ui.button("Cylinder").clicked() { self.load_cylinder(); }
                    if ui.button("Sphere").clicked() { self.load_sphere(); }
                });
                ui.horizontal(|ui| {
                    if ui.button("Cone").clicked() { self.load_cone(); }
                    if ui.button("Torus").clicked() { self.load_torus(); }
                });
                // --- Models ---
                ui.separator();
                ui.heading(egui::RichText::new("Models").size(12.0));
                if ui.button("ICE Engine (Inline-4)").clicked() {
                    self.load_engine();
                }
                // --- Import ---
                ui.separator();
                ui.heading(egui::RichText::new("Import").size(12.0));

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

                // --- Export ---
                ui.separator();
                ui.heading(egui::RichText::new("Export").size(12.0));

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

                // --- Display ---
                ui.separator();
                ui.heading(egui::RichText::new("Display").size(12.0));
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
                    self.selected_face = None;
                    self.highlighted_face = None;
                    self.highlight_dirty = true;
                    self.uv_svg_cache = None;
                    self.open_tree_nodes.clear();
                    self.scroll_to_tree_node = None;
                    self.scroll_to_face_id = None;
                }

                // --- Info ---
                ui.separator();
                ui.heading(egui::RichText::new("Info").size(12.0));
                ui.label(egui::RichText::new(format!("Model: {}", self.current_model.name)).size(12.0));
                ui.label(egui::RichText::new(format!("Vertices: {}", self.current_model.vertex_count)).size(12.0));
                ui.label(egui::RichText::new(format!("Triangles: {}", self.current_model.triangle_count)).size(12.0));
                ui.label(egui::RichText::new(format!("Instances: {}", self.detailed_instances.len())).size(12.0));

                // Loading progress in info section
                if self.is_loading && self.total_instance_count > 0 {
                    let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                    ui.label(egui::RichText::new(format!("Loading: {}/{} ({:.0}%)", self.triangulated_count, self.total_instance_count, progress * 100.0))
                        .size(11.0).color(egui::Color32::from_rgb(80, 180, 80)));
                }

                if let Some((inst_idx, fid)) = self.highlighted_face {
                    ui.label(egui::RichText::new(format!("Selected face: #{} (inst #{})", fid, inst_idx))
                        .size(12.0)
                        .color(egui::Color32::from_rgb(255, 220, 50)));
                }

                // --- Manifold Statistics ---
                ui.separator();
                ui.heading(egui::RichText::new("Manifold").size(12.0));
                if let Some(ref report) = self.manifold_report {
                    let watertight = report.is_watertight();
                    let wt_color = if watertight {
                        egui::Color32::from_rgb(80, 200, 80)
                    } else {
                        egui::Color32::from_rgb(255, 100, 80)
                    };
                    let wt_label = if watertight { "Yes" } else { "No" };
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Watertight:").size(12.0));
                        ui.label(egui::RichText::new(wt_label).size(12.0).color(wt_color));
                    });
                    ui.label(egui::RichText::new(format!("Euler characteristic: {}", report.euler_characteristic)).size(12.0));
                    ui.label(egui::RichText::new(format!("Boundary edges: {}", report.boundary_edge_count)).size(12.0));
                    ui.label(egui::RichText::new(format!("Non-manifold edges: {}", report.non_manifold_edge_count)).size(12.0));
                    ui.label(egui::RichText::new(format!("Degenerate triangles: {}", report.degenerate_triangle_count)).size(12.0));
                    ui.label(egui::RichText::new(format!("T-junctions: {}", report.t_junction_count)).size(12.0));
                } else {
                    ui.label(egui::RichText::new("No mesh loaded").size(11.0).color(egui::Color32::GRAY));
                }

                // --- Errors section ---
                if self.last_error.is_some() || self.failed_face_count > 0 {
                    ui.separator();
                    ui.heading(egui::RichText::new("Errors").size(12.0));
                    if let Some(ref err) = self.last_error {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("✖")
                                .size(12.0)
                                .color(egui::Color32::from_rgb(255, 80, 80)));
                            ui.add(egui::Label::new(
                                egui::RichText::new(err)
                                    .size(11.0)
                                    .color(egui::Color32::from_rgb(255, 120, 120))
                            ).wrap());
                        });
                        if ui.button("Dismiss").clicked() {
                            self.last_error = None;
                        }
                    }
                    if self.failed_face_count > 0 {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("⚠")
                                .size(12.0)
                                .color(egui::Color32::from_rgb(255, 200, 50)));
                            ui.label(egui::RichText::new(format!(
                                "{} face(s) failed triangulation (skipped)",
                                self.failed_face_count
                            )).size(11.0).color(egui::Color32::from_rgb(255, 220, 100)));
                        });
                    }
                }

                let cam_pos = self.camera.position();
                ui.label(egui::RichText::new(format!("Camera: ({:.0}, {:.0}, {:.0})", cam_pos[0], cam_pos[1], cam_pos[2]))
                    .size(11.0).color(egui::Color32::GRAY));

                ui.add_space(4.0);
                ui.separator();
                ui.label(
                    egui::RichText::new("Click: Select | Ctrl+Click: Face | Drag: Rotate | Scroll: Zoom")
                        .size(9.0)
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

                    // ─── Mouse picking: click = select solid, Ctrl+click = select face ───
                    if response.clicked_by(egui::PointerButton::Primary) {
                        let ctrl_held = ui.input(|i| i.modifiers.ctrl || i.modifiers.command);
                        let mouse_pos = ui.input(|i| i.pointer.latest_pos());
                        if let Some(pos) = mouse_pos {
                            // Convert screen position to viewport-local coordinates
                            let local_x = pos.x - rect.min.x;
                            let local_y = pos.y - rect.min.y;
                            let viewport = (0.0, 0.0, rect.width(), rect.height());

                            if let Some(pick) = pick_at(
                                &self.mesh,
                                &self.instance_triangle_ranges,
                                &self.camera,
                                [local_x, local_y],
                                viewport,
                            ) {
                                if ctrl_held {
                                    // Ctrl+click: select face
                                    if let Some(fid) = pick.face_id {
                                        self.selected_instance = Some(pick.instance_idx);
                                        self.selected_face = Some((pick.instance_idx, fid));
                                        self.highlighted_face = Some((pick.instance_idx, fid));
                                        self.highlight_dirty = true;
                                        self.uv_svg_cache = None;
                                        self.scroll_to_face_id = Some(fid);
                                        self.log(&format!("Picked face #{} (instance #{})", fid, pick.instance_idx));
                                        // Navigate structure tree
                                        if let Some(ref tree) = self.assembly_tree {
                                            let (path, target) = find_instance_path(tree, pick.instance_idx);
                                            self.open_tree_nodes = path.into_iter().collect();
                                            self.scroll_to_tree_node = target;
                                        }
                                    }
                                } else {
                                    // Simple click: select solid/instance
                                    self.selected_instance = Some(pick.instance_idx);
                                    self.selected_face = None;
                                    self.highlighted_face = None;
                                    self.highlight_dirty = true;
                                    self.uv_svg_cache = None;
                                    self.log(&format!("Picked instance #{}", pick.instance_idx));
                                    // Navigate structure tree
                                    if let Some(ref tree) = self.assembly_tree {
                                        let (path, target) = find_instance_path(tree, pick.instance_idx);
                                        self.open_tree_nodes = path.into_iter().collect();
                                        self.scroll_to_tree_node = target;
                                    }
                                }
                            } else {
                                // Clicked on empty space — deselect
                                self.selected_instance = None;
                                self.selected_face = None;
                                self.highlighted_face = None;
                                self.highlight_dirty = true;
                                self.uv_svg_cache = None;
                                self.open_tree_nodes.clear();
                                self.scroll_to_tree_node = None;
                                self.scroll_to_face_id = None;
                            }
                        }
                    }

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
                        let (vertices, indices) = mesh_to_gpu_data(&self.mesh, self.highlighted_face, self.selected_instance, &self.instance_triangle_ranges);
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

                // ─── Loading progress overlay ───
                if self.is_loading && self.total_instance_count > 0 {
                    let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                    let bar_w = rect.width() * 0.6;
                    let bar_h = 20.0;
                    let bar_x = rect.center().x - bar_w / 2.0;
                    let bar_y = rect.bottom() - 60.0;
                    let bar_rect = egui::Rect::from_min_max(
                        egui::pos2(bar_x, bar_y),
                        egui::pos2(bar_x + bar_w, bar_y + bar_h),
                    );

                    // Background
                    ui.painter().rect_filled(
                        egui::Rect::from_min_max(
                            egui::pos2(bar_x - 10.0, bar_y - 25.0),
                            egui::pos2(bar_x + bar_w + 10.0, bar_y + bar_h + 10.0),
                        ),
                        4.0,
                        egui::Color32::from_rgba_premultiplied(0, 0, 0, 160),
                    );

                    // Label
                    ui.painter().text(
                        egui::pos2(bar_x + bar_w / 2.0, bar_y - 8.0),
                        egui::Align2::CENTER_CENTER,
                        format!("Triangulating: {}/{} ({:.0}%)", self.triangulated_count, self.total_instance_count, progress * 100.0),
                        egui::FontId::proportional(12.0),
                        egui::Color32::WHITE,
                    );

                    // Progress bar background
                    ui.painter().rect_filled(
                        bar_rect,
                        3.0,
                        egui::Color32::from_rgb(60, 60, 60),
                    );

                    // Progress bar fill
                    let fill_w = bar_w * progress;
                    if fill_w > 0.0 {
                        ui.painter().rect_filled(
                            egui::Rect::from_min_max(
                                egui::pos2(bar_x, bar_y),
                                egui::pos2(bar_x + fill_w, bar_y + bar_h),
                            ),
                            3.0,
                            egui::Color32::from_rgb(80, 180, 80),
                        );
                    }
                }
            });
    }
}

/// Generate UV grid SVG for a face (standalone function to avoid borrow conflicts).
fn generate_uv_svg(face: &FaceInfo, u_divs: usize, v_divs: usize) -> String {
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

    // Build outer boundary polygon for point-in-polygon clipping
    let outer_uv_poly: Vec<(f64, f64)> = face.outer_uv_boundary.iter()
        .flat_map(|pl| pl.iter().map(|pt| (pt.u, pt.v)))
        .collect();

    for i in 0..=u_divs {
        for j in 0..=v_divs {
            let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
            let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
            let pt3d = face.surface.point_at(u, v);
            if pt3d.x.is_finite() && pt3d.y.is_finite() && pt3d.z.is_finite() {
                // Only draw dot if inside the outer boundary polygon
                let inside = !outer_uv_poly.is_empty() && point_in_polygon(u, v, &outer_uv_poly);
                if inside {
                    let x = map_u(u);
                    let y = map_v(v);
                    svg.push_str(&format!(
                        "  <circle cx=\"{:.2}\" cy=\"{:.2}\" r=\"2\" fill=\"#6688ff\" opacity=\"0.7\"/>\n", x, y
                    ));
                }
            }
        }
    }

    svg.push_str(&format!(
        "  <text x=\"{}\" y=\"{}\" fill=\"#aaa\" font-size=\"12\" text-anchor=\"middle\">U ({:.2} .. {:.2})</text>\n",
        margin + draw_w / 2.0, svg_height - 5.0, u_min, u_max
    ));
    svg.push_str(&format!(
        "  <text x=\"10\" y=\"{}\" fill=\"#aaa\" font-size=\"12\" text-anchor=\"middle\" transform=\"rotate(-90, 10, {})\">V ({:.2} .. {:.2})</text>\n",
        margin + draw_h / 2.0, margin + draw_h / 2.0, v_min, v_max
    ));
    svg.push_str(&format!(
        "  <text x=\"{}\" y=\"20\" fill=\"#fff\" font-size=\"13\" text-anchor=\"middle\">Face #{} (STEP #{}) {} forward={}</text>\n",
        svg_width / 2.0, face.face_id, face.step_face_id, face.surface_type, face.forward
    ));

    svg.push_str("</svg>\n");
    svg
}

/// Draw an assembly tree node recursively (static function to avoid borrow conflicts).
fn draw_assembly_node_static(
    ui: &mut egui::Ui,
    node: &AssemblyNode,
    selected_instance: Option<usize>,
    pending_instance_select: &mut Option<usize>,
    open_tree_nodes: &std::collections::HashSet<String>,
    scroll_to_tree_node: &Option<String>,
) {
    let key = node_key(node);
    let has_children = !node.children.is_empty();
    let brep_str = match node.brep_id {
        Some(id) => format!(" BREP#{}", id),
        None => String::new(),
    };
    let inst_str = match node.instance_index {
        Some(idx) => format!(" [{}]", idx),
        None => String::new(),
    };
    let label = format!("{}{}{}", node.name, brep_str, inst_str);

    // Use instance_index for selection (exact mapping to instance)
    let is_selected = node.instance_index.map_or(false, |idx| selected_instance == Some(idx));

    if has_children {
        let should_be_open = open_tree_nodes.contains(&key);
        // Use CollapsingState to programmatically force open/close
        let id = egui::Id::new(format!("tree_{}_{}", node.name, node.pd_id));
        let mut state = egui::collapsing_header::CollapsingState::load_with_default_open(
            ui.ctx(), id, false,
        );
        if should_be_open {
            state.set_open(true);
        }
        // If the scroll target is this node's child, also ensure this node is open
        if let Some(ref scroll_key) = scroll_to_tree_node {
            if !state.is_open() {
                // Check if any descendant matches the scroll target
                if has_descendant_with_key(node, scroll_key) {
                    state.set_open(true);
                }
            }
        }
        state.show_header(ui, |ui| {
            ui.label(egui::RichText::new(&label).size(11.0));
        }).body(|ui| {
            for child in &node.children {
                draw_assembly_node_static(ui, child, selected_instance, pending_instance_select, open_tree_nodes, scroll_to_tree_node);
            }
        });
    } else {
        let response = ui.selectable_label(is_selected, egui::RichText::new(&label).size(11.0));
        // Scroll to this node if it's the scroll target
        if scroll_to_tree_node.as_ref() == Some(&key) {
            response.scroll_to_me(Some(egui::Align::Center));
        }
        if response.clicked() {
            // Use instance_index for precise selection
            if let Some(idx) = node.instance_index {
                *pending_instance_select = Some(idx);
            }
        }
    }
}

/// Check if any descendant of the given node has the specified key.
fn has_descendant_with_key(node: &AssemblyNode, target_key: &str) -> bool {
    for child in &node.children {
        if node_key(child) == target_key {
            return true;
        }
        if has_descendant_with_key(child, target_key) {
            return true;
        }
    }
    false
}

impl ViewerApp {


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

/// Point-in-polygon test using ray casting algorithm.
/// Returns true if the point (x, y) is inside the polygon defined by vertices.
/// Uses the even-odd rule: cast a horizontal ray from the point and count
/// how many polygon edges it crosses. If odd, the point is inside.
fn point_in_polygon(x: f64, y: f64, polygon: &[(f64, f64)]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    let n = polygon.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = polygon[i];
        let (xj, yj) = polygon[j];
        // Check if the ray from (x, y) going right crosses the edge (j, i)
        if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ─── Assembly tree path finder ─────────────────────────────────────────────

/// Generate a unique key for an assembly node (used for open/scroll tracking).
/// Uses name + pd_id to avoid collisions when multiple nodes share the same name.
fn node_key(node: &AssemblyNode) -> String {
    format!("{}_{}", node.name, node.pd_id)
}

/// Find the path of node keys from root to the leaf node with the given instance_index.
/// Returns (path_keys, target_leaf_key) — path_keys for opening ancestors,
/// target_leaf_key for scrolling to the selected item.
fn find_instance_path(node: &AssemblyNode, target_instance: usize) -> (Vec<String>, Option<String>) {
    let key = node_key(node);
    if node.instance_index == Some(target_instance) {
        return (vec![key.clone()], Some(key));
    }
    for child in &node.children {
        let (mut path, target) = find_instance_path(child, target_instance);
        if !path.is_empty() {
            path.insert(0, key.clone());
            return (path, target);
        }
    }
    (Vec::new(), None)
}

/// Assign instance_index to the next unassigned leaf node in the assembly tree (DFS order).
/// Called during progressive loading to link tree nodes to their graphical instances
/// as each instance gets triangulated.
fn assign_instance_to_tree(node: &mut AssemblyNode, instance_idx: usize) {
    // Use an explicit stack to avoid stack overflow on deeply nested trees
    let mut stack: Vec<&mut AssemblyNode> = vec![node];
    while let Some(n) = stack.pop() {
        if n.children.is_empty() && n.brep_id.is_some() && n.instance_index.is_none() {
            n.instance_index = Some(instance_idx);
            return;
        }
        // Push children in reverse order so leftmost is processed first
        for child in n.children.iter_mut().rev() {
            stack.push(child);
        }
    }
}
