// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Main application state and UI.

use std::sync::Arc;
use std::sync::Mutex;

use crate::camera::OrbitCamera;
use crate::renderer::{
    MeshVertex, LineVertex, SceneCallback, SceneResources, SceneUniforms,
    create_scene_resources, update_mesh_buffers, update_uniforms, update_edge_buffers,
    update_wireframe_overlay_buffers,
};
use draper_core::engine::{EngineConfig, build_engine};
use draper_topology::ShapeBuilder;
use draper_mesh::{triangulate_solid, TriangleMesh, TriangulationParams, check_manifold, ManifoldReport, generate_3draper_text, cut_text_holes_in_mesh, TextSurface};
use draper_step::{AssemblyNode, DetailedMeshInstance, FaceInfo, PendingBrepInstance, OwnedStepConversionContext, StepFile, step_structure_lazy};
use draper_geometry::{Surface, Point2d};
use egui_wgpu::RenderState;
use eframe::egui;

/// Unified triangulation parameters for all platforms.
///
/// Use same quality parameters on all platforms for consistent results.
/// Previously WASM used reduced quality to avoid browser freezes,
/// but this caused significant visual differences from the native version.
fn wasm_tri_params() -> TriangulationParams {
    TriangulationParams::default()
}

/// Convert TriangleMesh to GPU vertex/index data.
/// Uses flat shading with face normals to properly support per-triangle colors from STEP files.
/// Selection state is encoded as per-vertex attributes and applied AFTER lighting in the shader:
///   selection: 0 = normal, 1 = selected instance, 2 = unused (was dimmed)
///   highlight: 0 = normal face, 1 = highlighted face
///
/// Hidden instances are skipped entirely — their triangles are not included in the output.
/// The returned Vec<(usize, usize)> contains per-instance triangle ranges in the GPU OUTPUT
/// buffer (visible triangles only, indices shift when instances are hidden). These ranges
/// must NOT be stored back into instance_triangle_ranges because that field must always map
/// to the ORIGINAL mesh's triangle indices (self.mesh.triangles), which are used by
/// build_wireframe_overlay_vertices(), pick_at(), and subsequent mesh_to_gpu_data() calls.
fn mesh_to_gpu_data(
    mesh: &TriangleMesh,
    highlighted_face: Option<(usize, u64)>,
    selected_instance: Option<usize>,
    instance_triangle_ranges: &[(usize, usize)],
    hidden_instances: &std::collections::HashSet<usize>,
) -> (Vec<MeshVertex>, Vec<u32>, Vec<(usize, usize)>) {
    // NOTE: compute_face_normals() and ensure_colors() must be called on the mesh
    // BEFORE calling this function, to avoid cloning the entire mesh here.
    let normals = mesh.face_normals.as_ref();
    let colors = mesh.triangle_colors.as_ref();
    let face_ids = mesh.triangle_face_ids.as_ref();

    // Check if we have meaningful per-triangle colors (not all default grey)
    let has_real_colors = colors.map_or(false, |c| {
        c.iter().any(|col| (col[0] - 0.62).abs() > 0.01 || (col[1] - 0.65).abs() > 0.01 || (col[2] - 0.70).abs() > 0.01)
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
                    color: [0.62, 0.65, 0.70],
                    selection: 0.0,
                    highlight: 0.0,
                });
            }
            for tri in &mesh.triangles {
                gpu_indices.push(tri[0]);
                gpu_indices.push(tri[1]);
                gpu_indices.push(tri[2]);
            }
            // No instance tracking for smooth shading path (single-instance primitives)
            let new_ranges: Vec<(usize, usize)> = instance_triangle_ranges.to_vec();
            return (gpu_vertices, gpu_indices, new_ranges);
        }
    }

    // Flat shading: duplicate vertices per triangle with face normals and colors
    let mut gpu_vertices = Vec::with_capacity(mesh.triangles.len() * 3);
    let mut gpu_indices = Vec::with_capacity(mesh.triangles.len() * 3);

    // Build updated instance triangle ranges for visible instances.
    // When an instance is hidden, its triangles are skipped, so ranges shift.
    let mut new_instance_ranges: Vec<(usize, usize)> = vec![(0, 0); instance_triangle_ranges.len()];
    let mut visible_tri_count: usize = 0;

    for (i, tri) in mesh.triangles.iter().enumerate() {
        // Determine which instance this triangle belongs to
        let mut inst_idx: Option<usize> = None;
        for (idx, &(start, end)) in instance_triangle_ranges.iter().enumerate() {
            if i >= start && i < end {
                inst_idx = Some(idx);
                break;
            }
        }

        // Skip triangles belonging to hidden instances
        if let Some(idx) = inst_idx {
            if hidden_instances.contains(&idx) {
                continue;
            }
        }

        let normal = normals
            .and_then(|n| n.get(i))
            .map(|n| [n[0] as f32, n[1] as f32, n[2] as f32])
            .unwrap_or([0.0, 0.0, 1.0]);

        // Use original base color — never modify it for selection state
        let color = colors
            .and_then(|c| c.get(i))
            .map(|c| [c[0], c[1], c[2]])
            .unwrap_or([0.62, 0.65, 0.70]);

        // Compute per-triangle selection state (applied AFTER lighting in shader)
        // selection: 0 = normal, 1 = selected instance, 2 = unused
        let mut selection = 0.0_f32;
        let mut is_highlighted = false;

        // Determine instance selection state
        if let Some(sel_idx) = selected_instance {
            if Some(sel_idx) == inst_idx {
                // This triangle belongs to the selected instance
                selection = 1.0;
            }
            // Non-selected instances keep selection = 0.0 (normal, no dimming)
        }

        // Check face-level highlighting (instance-aware)
        // highlighted_face = (instance_index, face_id) — only highlight within the correct instance
        if let Some((hl_inst, hl_fid)) = highlighted_face {
            if Some(hl_inst) == inst_idx {
                if let Some(ids) = face_ids {
                    if ids.get(i).map_or(false, |id| *id == hl_fid) {
                        is_highlighted = true;
                    }
                }
            }
        }

        let highlight = if is_highlighted { 1.0 } else { 0.0 };

        // Track instance ranges in the output mesh (visible triangles only)
        if let Some(idx) = inst_idx {
            if new_instance_ranges[idx].0 == 0 && new_instance_ranges[idx].1 == 0 && visible_tri_count == 0 {
                // First triangle of this instance
                new_instance_ranges[idx] = (visible_tri_count, visible_tri_count + 1);
            } else if new_instance_ranges[idx].1 == visible_tri_count {
                // Extending existing range
                new_instance_ranges[idx].1 = visible_tri_count + 1;
            } else {
                // Gap (shouldn't happen with sequential instances) — extend anyway
                new_instance_ranges[idx].1 = visible_tri_count + 1;
            }
        }
        visible_tri_count += 1;

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

    (gpu_vertices, gpu_indices, new_instance_ranges)
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
/// Hidden instances are excluded from picking.
fn pick_at(
    mesh: &TriangleMesh,
    instance_triangle_ranges: &[(usize, usize)],
    hidden_instances: &std::collections::HashSet<usize>,
    camera: &OrbitCamera,
    screen_pos: [f32; 2],
    viewport: (f32, f32, f32, f32),
) -> Option<PickResult> {
    let (ray_origin, ray_dir) = camera.screen_to_ray(screen_pos, viewport);

    let face_ids = mesh.triangle_face_ids.as_ref();
    let mut best: Option<PickResult> = None;

    for (i, tri) in mesh.triangles.iter().enumerate() {
        // Determine which instance this triangle belongs to
        let instance_idx = match instance_triangle_ranges
            .iter()
            .position(|&(start, end)| i >= start && i < end)
        {
            Some(idx) => idx,
            None => continue, // Triangle not in any instance range — skip
        };

        // Skip hidden instances — they can't be picked
        if hidden_instances.contains(&instance_idx) {
            continue;
        }

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
    /// Show B-Rep boundary edges.
    show_edges: bool,
    /// Show wireframe overlay (triangle mesh edges on top of filled surfaces).
    show_wireframe_overlay: bool,
    /// Wireframe overlay line vertices need GPU upload.
    wireframe_overlay_dirty: bool,
    /// Edge line vertices need GPU upload.
    edge_dirty: bool,
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
    /// Set of instance indices that are currently hidden (not rendered).
    /// Users toggle visibility via checkboxes in the assembly tree.
    hidden_instances: std::collections::HashSet<usize>,
    // ─── Tree navigation state ────────────────────────────────────────
    /// Node keys ("name_pd_id") that should be forced open in the assembly tree.
    open_tree_nodes: std::collections::HashSet<String>,
    /// Node key to scroll to in the assembly tree (set when selecting from 3D view).
    scroll_to_tree_node: Option<String>,
    /// Face ID to scroll to in the face list (set when selecting a face from 3D view).
    scroll_to_face_id: Option<u64>,

    // ─── Progressive loading state ────────────────────────────────────
    /// Pending BREP instances to triangulate (populated during parse phase, consumed during render phase).
    /// Unlike the old approach which stored already-triangulated DetailedMeshInstance,
    /// this stores PendingBrepInstance descriptors that are triangulated ONE AT A TIME
    /// in the frame loop, keeping the browser responsive.
    pending_breps: Vec<PendingBrepInstance>,
    /// Cached conversion context — owns the StepFile and is reused across frames.
    /// This avoids rebuilding entity maps, cloning HashMaps, and recomputing bounding boxes
    /// on every animation frame (which was the main cause of performance degradation).
    /// Created LAZILY on the first `process_pending_breps()` call to avoid blocking
    /// the main thread during `process_step_file()`.
    conversion_ctx: Option<OwnedStepConversionContext>,
    /// Stored StepFile for lazy context creation. When a STEP file is loaded, we store it
    /// here instead of immediately creating OwnedStepConversionContext (which is expensive).
    /// The context is built on the first frame of progressive triangulation.
    pending_step_file: Option<StepFile>,
    /// Total number of instances being loaded (for progress display).
    total_instance_count: usize,
    /// Number of instances already triangulated.
    triangulated_count: usize,
    /// Whether we are currently in the progressive triangulation phase.
    is_loading: bool,
    /// Name of the file being loaded.
    loading_name: String,
    /// Start time of loading (for timeout detection).
    /// Uses web_time::Instant on WASM (std::time::Instant panics on wasm32).
    #[cfg(not(target_arch = "wasm32"))]
    loading_start: Option<std::time::Instant>,
    #[cfg(target_arch = "wasm32")]
    loading_start: Option<web_time::Instant>,

    // ─── Manifold statistics ──────────────────────────────────────────
    /// Manifold report for the current mesh (computed on load).
    manifold_report: Option<ManifoldReport>,

    // ─── Panel collapse state (for mobile optimization) ──────────────
    /// Whether the left controls panel is open.
    controls_panel_open: bool,
    /// Whether the log panel is expanded (collapsed = thin bar with toggle).
    log_panel_open: bool,
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
    /// When enabled, validates edge consistency after loading and logs details.
    /// This is the `--validate-consistency` diagnostic mode — it runs
    /// `validate_edge_consistency` on the final mesh to check that shared edges
    /// have bit-identical vertex positions. Useful for debugging mesh quality issues.
    validate_consistency: bool,
    /// Last edge consistency report (for display in the UI).
    last_consistency_report: Option<String>,

    // ─── JSON API state ────────────────────────────────────────────────────
    /// JSON API engine.
    json_api: draper_json::JsonApi,
    /// JSON API command input text.
    json_api_input: String,
    /// JSON API response text.
    json_api_output: String,
    /// Whether to show the JSON API panel.
    show_json_api: bool,

    // ─── Mobile UI state ────────────────────────────────────────────────────
    /// Whether we are in mobile (narrow screen) mode — updated each frame.
    is_mobile: bool,
    /// Which mobile overlay panel is currently shown (None = none).
    mobile_panel: Option<MobilePanel>,
    /// Whether the mobile log panel is visible.
    mobile_log_open: bool,

    // ─── Chunked triangulation ──────────────────────────────────────────────
    /// Time-budgeted BREP triangulation processor.
    chunked_triangulator: ChunkedBrepTriangulator,
}

/// Mobile overlay panel type.
#[derive(Clone, Debug, PartialEq)]
enum MobilePanel {
    /// Left controls panel (primitives, import, display, info)
    Controls,
    /// Right structure panel (tree, faces, UV, face info)
    Structure,
}

/// Result of processing a single chunk of BREP triangulation.
#[derive(Clone, Debug, PartialEq)]
enum ChunkResult {
    /// A BREP was fully triangulated and merged into the mesh.
    BrepCompleted {
        /// Name of the completed instance.
        name: String,
        /// Number of triangles added.
        triangles_added: usize,
        /// Time taken for this BREP.
        elapsed_ms: f64,
    },
    /// No more BREPs to process — loading is complete.
    AllDone,
    /// Time budget exceeded — call again next frame.
    TimeBudgetExceeded,
    /// Triangulation failed for this instance.
    Failed {
        name: String,
        reason: String,
    },
}

/// Time-budgeted BREP triangulation processor.
///
/// Processes pending BREP instances one at a time, respecting a frame time budget.
/// This keeps the browser/UI responsive during loading of large STEP assemblies.
///
/// # Time Budget Strategy
/// - **WASM**: 8ms per frame (targets 120fps, leaves headroom for rendering)
/// - **Native**: 16ms per frame (targets 60fps, but allows batch processing)
///
/// Individual BREPs are processed atomically — if a single BREP takes longer than
/// the budget, we accept the frame drop but log a warning. Future work: intra-BREP
/// chunked triangulation would require refactoring the converter API.
struct ChunkedBrepTriangulator {
    /// Time budget per frame for triangulation work.
    time_budget: std::time::Duration,
}

impl ChunkedBrepTriangulator {
    /// Create a new chunked processor with platform-appropriate time budget.
    fn new() -> Self {
        let time_budget = if cfg!(target_arch = "wasm32") {
            // 8ms for 120fps target on WASM
            std::time::Duration::from_millis(8)
        } else {
            // 16ms for 60fps target on native
            std::time::Duration::from_millis(16)
        };
        Self { time_budget }
    }

    /// Create with a custom time budget (for testing).
    #[allow(dead_code)]
    fn with_budget(time_budget: std::time::Duration) -> Self {
        Self { time_budget }
    }
}

impl ViewerApp {
    /// CAD-style color palette for instances without STEP-defined colors.
    /// Designed for a light background — richer, more saturated colors that
    /// stand out well against a light gray viewport.
    fn instance_color(index: usize) -> [f32; 4] {
        const PALETTE: [[f32; 4]; 12] = [
            [0.62, 0.65, 0.70, 1.0], // Steel (default)
            [0.72, 0.58, 0.38, 1.0], // Gold
            [0.35, 0.62, 0.48, 1.0], // Emerald
            [0.65, 0.42, 0.50, 1.0], // Rose
            [0.55, 0.58, 0.35, 1.0], // Olive
            [0.38, 0.55, 0.72, 1.0], // Sky blue
            [0.72, 0.50, 0.35, 1.0], // Copper
            [0.32, 0.58, 0.60, 1.0], // Teal
            [0.55, 0.40, 0.60, 1.0], // Lavender
            [0.42, 0.62, 0.40, 1.0], // Mint
            [0.65, 0.52, 0.40, 1.0], // Bronze
            [0.42, 0.45, 0.68, 1.0], // Periwinkle
        ];
        PALETTE[index % PALETTE.len()]
    }

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
        let mut mesh = triangulate_solid(&solid, &params);

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
            // Ensure mesh has face normals and colors before GPU upload
            if mesh.face_normals.is_none() {
                mesh.compute_face_normals();
            }
            mesh.ensure_colors([0.62, 0.65, 0.70, 1.0]);
            let (vertices, indices, _new_ranges) = mesh_to_gpu_data(&mesh, None, None, &[], &std::collections::HashSet::new());
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
            show_edges: true,
            show_wireframe_overlay: false,
            wireframe_overlay_dirty: false,
            edge_dirty: false,
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
            hidden_instances: std::collections::HashSet::new(),
            open_tree_nodes: std::collections::HashSet::new(),
            scroll_to_tree_node: None,
            scroll_to_face_id: None,
            pending_breps: Vec::new(),
            conversion_ctx: None,
            pending_step_file: None,
            total_instance_count: 0,
            triangulated_count: 0,
            is_loading: false,
            loading_name: String::new(),
            loading_start: None,
            manifold_report,
            controls_panel_open: true,
            log_panel_open: true,
            structure_tree_open: true,
            face_list_open: true,
            uv_grid_open: false,
            face_info_open: false,
            last_error: None,
            warning_count: 0,
            error_count: 0,
            failed_face_count: 0,
            validate_consistency: false,
            last_consistency_report: None,
            json_api: draper_json::JsonApi::new(),
            json_api_input: String::new(),
            json_api_output: String::new(),
            show_json_api: false,
            is_mobile: false,
            mobile_panel: None,
            mobile_log_open: false,
            chunked_triangulator: ChunkedBrepTriangulator::new(),
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
        self.edge_dirty = true;
        self.wireframe_overlay_dirty = true;
        // Reset selection when loading new model
        self.selected_instance = None;
        self.selected_face = None;
        self.highlighted_face = None;
        self.highlight_dirty = true;
        self.uv_svg_cache = None;
        self.open_tree_nodes.clear();
        self.scroll_to_tree_node = None;
        self.scroll_to_face_id = None;
        self.hidden_instances.clear();
        self.log(&format!("Loaded: {} ({} vertices, {} triangles) — {}",
            name, self.current_model.vertex_count, self.current_model.triangle_count,
            if is_watertight { "watertight" } else { "not watertight" }));

        // Run edge consistency validation if enabled
        if self.validate_consistency {
            let report = draper_mesh::watertight::validate_edge_consistency(&self.mesh, 0.0);
            let msg = format!(
                "Edge consistency: {}/{} consistent, {} inconsistent ({:.2}%), max_dist={:.2e}",
                report.consistent_edges, report.shared_edges_checked,
                report.inconsistent_edges, report.inconsistency_rate(),
                report.max_vertex_distance
            );
            if report.is_consistent() {
                self.log(&msg);
            } else {
                self.log_warning(&msg);
                for inc in &report.worst_inconsistencies {
                    self.log_warning(&format!(
                        "  Edge: vertices ({}, {}) dist={:.2e}",
                        inc.vertex_indices.0, inc.vertex_indices.1, inc.distance
                    ));
                }
            }
            self.last_consistency_report = Some(msg);
        } else {
            self.last_consistency_report = None;
        }
    }

    fn load_box(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let mesh = triangulate_solid(&solid, &wasm_tri_params());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Box 100x80x60");
    }

    fn load_cylinder(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let mesh = triangulate_solid(&solid, &wasm_tri_params());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cylinder R=40 H=100");
    }

    fn load_sphere(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let mesh = triangulate_solid(&solid, &wasm_tri_params());
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
        let mesh = triangulate_solid(&solid, &wasm_tri_params());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cone R=40 H=80");
    }

    fn load_torus(&mut self) {
        let solid = ShapeBuilder::make_torus(40.0, 12.0);
        let mesh = triangulate_solid(&solid, &wasm_tri_params());
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

    /// Load a revolution solid (vase-like shape) — demonstrates RevolutionSurface.
    fn load_revolution(&mut self) {
        use draper_geometry::{Curve3d, NurbsCurve, Point3d as P3};
        // Profile: a wavy curve that creates an interesting vase shape when revolved around Z
        // Using a NURBS curve for smooth profile
        let profile = Curve3d::Nurbs(NurbsCurve {
            degree: 2,
            control_points: vec![
                P3::new(20.0, 0.0, 0.0),
                P3::new(40.0, 0.0, 30.0),
                P3::new(30.0, 0.0, 60.0),
                P3::new(15.0, 0.0, 80.0),
                P3::new(35.0, 0.0, 100.0),
            ],
            weights: vec![1.0, 1.0, 1.0, 1.0, 1.0],
            knots: vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        });
        let solid = ShapeBuilder::make_revolution(profile, std::f64::consts::PI * 2.0);
        let mesh = triangulate_solid(&solid, &wasm_tri_params());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Revolution (Vase)");
    }

    /// Load an extrusion solid — demonstrates ExtrusionSurface.
    fn load_extrusion(&mut self) {
        use draper_geometry::{Curve3d, Circle, Point3d as P3};
        // Profile: a circle extruded along Y axis → creates a tube
        let profile = Curve3d::Circle(Circle::new_xy(
            P3::new(0.0, 0.0, 50.0),
            30.0,
        ));
        let solid = ShapeBuilder::make_extrusion(
            profile,
            draper_geometry::Direction3d::Y,
            80.0,
        );
        let mesh = triangulate_solid(&solid, &wasm_tri_params());
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Extrusion (Circle→Y)");
    }

    /// Load a NURBS surface — demonstrates NurbsSurface.
    fn load_nurbs(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3, Point2d};
        // Create a bicubic NURBS surface (a wavy sheet)
        let control_points = vec![
            vec![P3::new(-50.0, -50.0,  0.0), P3::new(-50.0, -15.0, 10.0), P3::new(-50.0,  15.0, 10.0), P3::new(-50.0,  50.0,  0.0)],
            vec![P3::new(-15.0, -50.0, 10.0), P3::new(-15.0, -15.0, 30.0), P3::new(-15.0,  15.0, 25.0), P3::new(-15.0,  50.0,  5.0)],
            vec![P3::new( 15.0, -50.0, 10.0), P3::new( 15.0, -15.0, 25.0), P3::new( 15.0,  15.0, 30.0), P3::new( 15.0,  50.0, 10.0)],
            vec![P3::new( 50.0, -50.0,  0.0), P3::new( 50.0, -15.0,  5.0), P3::new( 50.0,  15.0, 10.0), P3::new( 50.0,  50.0,  0.0)],
        ];
        let weights = vec![vec![1.0; 4]; 4];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];

        let nurbs_surface = NurbsSurface {
            u_degree: 3, v_degree: 3,
            control_points, weights,
            u_knots, v_knots,
        };

        // Sample boundary points from the NURBS surface for triangulation
        let (u_min, u_max) = nurbs_surface.u_range();
        let (v_min, v_max) = nurbs_surface.v_range();
        let surface = Surface::Nurbs(nurbs_surface);
        let mut boundary = Vec::new();
        let mut boundary_uvs = Vec::new();
        let steps = 20;
        // Bottom edge (v = v_min)
        for i in 0..=steps {
            let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
            boundary.push(surface.point_at(u, v_min));
            boundary_uvs.push(Point2d::new(u, v_min));
        }
        // Right edge (u = u_max)
        for i in 1..=steps {
            let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
            boundary.push(surface.point_at(u_max, v));
            boundary_uvs.push(Point2d::new(u_max, v));
        }
        // Top edge (v = v_max), reversed
        for i in (0..steps).rev() {
            let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
            boundary.push(surface.point_at(u, v_max));
            boundary_uvs.push(Point2d::new(u, v_max));
        }
        // Left edge (u = u_min), reversed
        for i in (1..steps).rev() {
            let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
            boundary.push(surface.point_at(u_min, v));
            boundary_uvs.push(Point2d::new(u_min, v));
        }

        let params = TriangulationParams::default();
        // Use the UV-aware path for fast and correct NURBS triangulation.
        // This avoids the slow and inaccurate project_point() calls by providing
        // the exact UV coordinates directly.
        let mesh = draper_mesh::triangulate_face_with_boundary_and_holes_uv(
            &surface, &boundary, &boundary_uvs, &[], &[], true, &params,
        );

        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS (Wavy Sheet)");
    }

    /// Load Box with "3" hole CUT OUT on the top face.
    fn load_box_text(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let base_mesh = triangulate_solid(&solid, &wasm_tri_params());
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Plane { z: 30.0 },
            3.0,   // text scale — large enough to be visible on 100×80 face
            5.0,   // hole depth
            [0.15, 0.15, 0.2, 1.0], // dark hole color
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Box + hole(3)");
    }

    /// Load Cylinder with "3" hole CUT OUT on the lateral surface.
    fn load_cylinder_text(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let base_mesh = triangulate_solid(&solid, &wasm_tri_params());
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Cylinder { radius: 40.0, height: 100.0 },
            2.5,   // text scale — visible on r=40 cylinder
            5.0,   // hole depth
            [0.1, 0.15, 0.1, 1.0], // dark green hole
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cylinder + hole(3)");
    }

    /// Load Sphere with "3" hole CUT OUT on the surface.
    fn load_sphere_text(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let base_mesh = triangulate_solid(&solid, &wasm_tri_params());
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Sphere { center: [0.0, 0.0, 0.0], radius: 50.0 },
            3.0,   // text scale — visible on r=50 sphere
            5.0,   // hole depth
            [0.1, 0.1, 0.2, 1.0], // dark blue hole
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Sphere + hole(3)");
    }

    /// Load Cone with "3" hole CUT OUT on the lateral surface.
    fn load_cone_text(&mut self) {
        let radius: f64 = 40.0;
        let height: f64 = 80.0;
        let half_angle = (radius / height).atan();
        let solid = ShapeBuilder::make_cone(radius, height, half_angle);
        let base_mesh = triangulate_solid(&solid, &wasm_tri_params());
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Cone { radius: 40.0, height: 80.0 },
            2.5,   // text scale — visible on r=40 cone
            5.0,   // hole depth
            [0.2, 0.15, 0.05, 1.0], // dark amber hole
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cone + hole(3)");
    }

    /// Load Torus with "3" hole CUT OUT on the outer surface.
    fn load_torus_text(&mut self) {
        let solid = ShapeBuilder::make_torus(40.0, 12.0);
        let base_mesh = triangulate_solid(&solid, &wasm_tri_params());
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Torus { major_radius: 40.0, minor_radius: 12.0 },
            2.0,   // text scale — visible on torus
            3.0,   // hole depth
            [0.15, 0.05, 0.2, 1.0], // dark purple hole
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Torus + hole(3)");
    }

    /// Load Revolution with "3" hole CUT OUT (approximated as cylinder for projection).
    fn load_revolution_text(&mut self) {
        use draper_geometry::{Curve3d, NurbsCurve, Point3d as P3};
        let profile = Curve3d::Nurbs(NurbsCurve {
            degree: 2,
            control_points: vec![
                P3::new(20.0, 0.0, 0.0),
                P3::new(40.0, 0.0, 30.0),
                P3::new(30.0, 0.0, 60.0),
                P3::new(15.0, 0.0, 80.0),
                P3::new(35.0, 0.0, 100.0),
            ],
            weights: vec![1.0, 1.0, 1.0, 1.0, 1.0],
            knots: vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        });
        let solid = ShapeBuilder::make_revolution(profile, std::f64::consts::PI * 2.0);
        let base_mesh = triangulate_solid(&solid, &wasm_tri_params());
        // Approximate revolution as cylinder for text projection
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Cylinder { radius: 30.0, height: 100.0 },
            2.0,   // text scale — visible on revolution
            5.0,   // hole depth
            [0.05, 0.15, 0.15, 1.0], // dark cyan hole
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Revolution + hole(3)");
    }

    /// Load NURBS with "3" hole CUT OUT (projected as flat plane).
    fn load_nurbs_text(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3, Point2d};
        let control_points = vec![
            vec![P3::new(-50.0, -50.0,  0.0), P3::new(-50.0, -15.0, 10.0), P3::new(-50.0,  15.0, 10.0), P3::new(-50.0,  50.0,  0.0)],
            vec![P3::new(-15.0, -50.0, 10.0), P3::new(-15.0, -15.0, 30.0), P3::new(-15.0,  15.0, 25.0), P3::new(-15.0,  50.0,  5.0)],
            vec![P3::new( 15.0, -50.0, 10.0), P3::new( 15.0, -15.0, 25.0), P3::new( 15.0,  15.0, 30.0), P3::new( 15.0,  50.0, 10.0)],
            vec![P3::new( 50.0, -50.0,  0.0), P3::new( 50.0, -15.0,  5.0), P3::new( 50.0,  15.0, 10.0), P3::new( 50.0,  50.0,  0.0)],
        ];
        let weights = vec![vec![1.0; 4]; 4];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface {
            u_degree: 3, v_degree: 3,
            control_points, weights,
            u_knots, v_knots,
        };
        let (u_min, u_max) = nurbs_surface.u_range();
        let (v_min, v_max) = nurbs_surface.v_range();
        let surface = Surface::Nurbs(nurbs_surface);
        let mut boundary = Vec::new();
        let mut boundary_uvs = Vec::new();
        let steps = 20;
        for i in 0..=steps {
            let u = u_min + (u_max-u_min)*i as f64/steps as f64;
            boundary.push(surface.point_at(u, v_min));
            boundary_uvs.push(Point2d::new(u, v_min));
        }
        for i in 1..=steps {
            let v = v_min + (v_max-v_min)*i as f64/steps as f64;
            boundary.push(surface.point_at(u_max, v));
            boundary_uvs.push(Point2d::new(u_max, v));
        }
        for i in (0..steps).rev() {
            let u = u_min + (u_max-u_min)*i as f64/steps as f64;
            boundary.push(surface.point_at(u, v_max));
            boundary_uvs.push(Point2d::new(u, v_max));
        }
        for i in (1..steps).rev() {
            let v = v_min + (v_max-v_min)*i as f64/steps as f64;
            boundary.push(surface.point_at(u_min, v));
            boundary_uvs.push(Point2d::new(u_min, v));
        }
        let params = TriangulationParams::default();
        let base_mesh = draper_mesh::triangulate_face_with_boundary_and_holes_uv(
            &surface, &boundary, &boundary_uvs, &[], &[], true, &params,
        );
        // NURBS sheet is roughly flat at z~0 to z~30, use plane projection at average z
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Plane { z: 10.0 },
            2.5,   // text scale — visible on NURBS sheet
            5.0,   // hole depth
            [0.2, 0.1, 0.15, 1.0], // dark pink hole
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS + hole(3)");
    }

    /// Load Extrusion with "3" hole CUT OUT (approximated as cylinder).
    fn load_extrusion_text(&mut self) {
        use draper_geometry::{Curve3d, Circle, Point3d as P3};
        let profile = Curve3d::Circle(Circle::new_xy(
            P3::new(0.0, 0.0, 50.0),
            30.0,
        ));
        let solid = ShapeBuilder::make_extrusion(
            profile,
            draper_geometry::Direction3d::Y,
            80.0,
        );
        let base_mesh = triangulate_solid(&solid, &wasm_tri_params());
        let mesh = cut_text_holes_in_mesh(
            &base_mesh,
            "3",
            &TextSurface::Cylinder { radius: 30.0, height: 80.0 },
            2.0,   // text scale — visible on extrusion
            5.0,   // hole depth
            [0.1, 0.08, 0.05, 1.0], // dark brown hole
        );
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Extrusion + hole(3)");
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

    /// Export the current model to JSON file.
    #[cfg(not(target_arch = "wasm32"))]
    fn export_json(&mut self, path: &str) {
        use draper_json::JsonModel;
        let model = if !self.detailed_instances.is_empty() {
            let assembly = self.assembly_tree.clone().unwrap_or_else(|| AssemblyNode {
                name: self.current_model.name.clone(),
                pd_id: 0,
                brep_id: None,
                instance_index: None,
                transform: None,
                color: None,
                children: Vec::new(),
            });
            JsonModel::from_instances(self.detailed_instances.clone(), assembly, &self.current_model.name)
        } else {
            // Create a minimal JsonModel from the current mesh
            let mut instances = Vec::new();
            if !self.mesh.vertices.is_empty() {
                let vertices: Vec<f64> = self.mesh.vertices.iter()
                    .flat_map(|p| [p.x, p.y, p.z]).collect();
                let triangles: Vec<u32> = self.mesh.triangles.iter()
                    .flat_map(|t| [t[0], t[1], t[2]]).collect();
                let normals = self.mesh.normals.as_ref().map(|n| {
                    n.iter().flat_map(|v| [v[0], v[1], v[2]]).collect()
                });
                let triangle_colors = self.mesh.triangle_colors.as_ref().map(|c| {
                    c.iter().flat_map(|v| [v[0], v[1], v[2], v[3]]).collect()
                });
                use draper_json::JsonMeshInstance;
                instances.push(JsonMeshInstance {
                    name: self.current_model.name.clone(),
                    brep_id: 0,
                    vertices,
                    triangles,
                    normals,
                    triangle_colors,
                    triangle_face_ids: self.mesh.triangle_face_ids.clone(),
                    transform: None,
                    color: None,
                    faces: Vec::new(),
                });
            }
            let assembly = AssemblyNode {
                name: self.current_model.name.clone(),
                pd_id: 0,
                brep_id: None,
                instance_index: Some(0),
                transform: None,
                color: None,
                children: Vec::new(),
            };
            JsonModel::from_instances(self.detailed_instances.clone(), assembly, &self.current_model.name)
        };
        match model.to_json_pretty() {
            Ok(json) => {
                match std::fs::write(path, &json) {
                    Ok(()) => self.log(&format!("Exported JSON: {} ({} bytes)", path, json.len())),
                    Err(e) => self.log_error(&format!("JSON write error: {}", e)),
                }
            }
            Err(e) => self.log_error(&format!("JSON export error: {}", e)),
        }
    }

    /// Import a model from JSON file.
    #[cfg(not(target_arch = "wasm32"))]
    fn import_json(&mut self, path: &str) {
        use draper_json::JsonModel;
        match std::fs::read_to_string(path) {
            Ok(json) => {
                match JsonModel::from_json(&json) {
                    Ok(model) => {
                        let mesh = model.to_triangle_mesh();
                        let name = model.metadata.name.clone();
                        self.detailed_instances = model.to_detailed_instances();
                        self.assembly_tree = Some(model.assembly);
                        self.load_mesh(mesh, &name);
                        self.log(&format!("Imported JSON: {} ({} instances)", path, model.metadata.instance_count));
                    }
                    Err(e) => self.log_error(&format!("JSON parse error: {}", e)),
                }
            }
            Err(e) => self.log_error(&format!("JSON read error: {}", e)),
        }
    }

    // ─── Shared file processing (used by both native and web) ─────────────

    /// Process a parsed STEP file — Phase 1: Parse + Build tree (fast).
    /// The tree is shown immediately. Triangulation happens progressively in update(),
    /// one BREP per frame, so the browser stays responsive.
    fn process_step_file(&mut self, step_file: &draper_step::StepFile, name: &str) {
        // Cancel any previous loading
        self.cancel_loading();

        // Count relevant geometry entities (fast O(n) pass)
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

        // ── Use LAZY conversion: build tree + collect pending BREP descriptors (FAST) ──
        // No triangulation happens here — that's done one-BREP-per-frame in process_pending_breps()
        let (tree, pending) = step_structure_lazy(step_file);
        self.assembly_tree = Some(tree);
        self.show_structure = true;

        if !pending.is_empty() {
            self.log(&format!("Queued {} BREP instances for progressive triangulation...", pending.len()));
            self.total_instance_count = pending.len();
            self.triangulated_count = 0;
            self.pending_breps = pending;
            // LAZY: Store the StepFile for later context creation.
            // OwnedStepConversionContext::new() is expensive (builds entity maps,
            // computes bounding box, clones HashMaps). Doing it synchronously here
            // blocks the WASM main thread and prevents the browser from ever
            // reaching process_pending_breps(). Instead, we create the context
            // lazily on the first frame of progressive triangulation.
            self.pending_step_file = Some(step_file.clone());
            self.conversion_ctx = None;
            self.is_loading = true;
            self.loading_name = name.to_string();
            self.loading_start = Some({
                #[cfg(not(target_arch = "wasm32"))]
                { std::time::Instant::now() }
                #[cfg(target_arch = "wasm32")]
                { web_time::Instant::now() }
            });

            // Clear existing rendering data — free WASM memory from previous load.
            // Without this, repeated file loads accumulate data in the WASM heap
            // (which has a 4GB limit in Chrome) and eventually crash the tab.
            self.detailed_instances.clear();
            self.instance_triangle_ranges.clear();
            self.selected_instance = None;
            self.selected_face = None;
            self.highlighted_face = None;
            self.failed_face_count = 0;

            // Drop old mesh and GPU resources explicitly to free WASM memory
            self.mesh = TriangleMesh::new();
            {
                let mut gpu = self.gpu_resources.lock().unwrap();
                *gpu = None; // Drop GPU buffers — frees GPU memory
            }
            self.mesh_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;

            // Auto-fit camera once tree is ready (even before mesh)
            self.log("Structure tree ready — triangulation will begin...");
        } else {
            // No BREPs found in the file
            self.log_warning("No BREP instances found in STEP file");
        }
    }

    /// Build GPU edge line vertices from B-Rep boundary data in detailed_instances.
    ///
    /// For each face, outer_boundary and inner_boundaries provide 3D polylines
    /// that represent the B-Rep edges. We convert these into LineVertex pairs
    /// (two vertices per line segment) with a dark edge color.
    ///
    /// IMPORTANT: Face boundary points are stored in LOCAL BREP coordinates,
    /// but the mesh vertices are already transformed to world space. So we must
    /// apply the instance transform to boundary points to match the rendered mesh.
    fn build_edge_line_vertices(&self) -> Vec<LineVertex> {
        let mut edge_vertices: Vec<LineVertex> = Vec::new();

        // Edge color: dark charcoal visible on light background
        let edge_color: [f32; 3] = [0.20, 0.20, 0.25];

        /// Transform a Point3d by a 4×4 matrix (homogeneous coordinates).
        fn transform_point(p: &draper_geometry::Point3d, m: &[[f64; 4]; 4]) -> [f32; 3] {
            let x = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
            let y = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
            let z = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];
            [x as f32, y as f32, z as f32]
        }

        for (inst_idx, inst) in self.detailed_instances.iter().enumerate() {
            // Skip hidden instances — don't draw their edges
            if self.hidden_instances.contains(&inst_idx) {
                continue;
            }

            // Apply instance transform to boundary points to match the mesh
            // (mesh vertices are already in world space, but boundary points are local)
            let tf = inst.transform.as_ref();

            for face in &inst.faces {
                // Outer boundary polylines
                for polyline in &face.outer_boundary {
                    if polyline.len() < 2 {
                        continue;
                    }
                    // Downsample: limit to ~500 points per polyline to avoid
                    // excessive edge vertex count on highly tessellated boundaries
                    let step = if polyline.len() > 500 {
                        (polyline.len() as f64 / 500.0).ceil() as usize
                    } else {
                        1
                    };
                    let mut prev_pos: Option<[f32; 3]> = None;
                    for i in (0..polyline.len()).step_by(step) {
                        let p = &polyline[i];
                        let pos = if let Some(m) = tf {
                            transform_point(p, m)
                        } else {
                            [p.x as f32, p.y as f32, p.z as f32]
                        };
                        if let Some(pp) = prev_pos {
                            edge_vertices.push(LineVertex {
                                position: pp,
                                color: edge_color,
                            });
                            edge_vertices.push(LineVertex {
                                position: pos,
                                color: edge_color,
                            });
                        }
                        prev_pos = Some(pos);
                    }
                    // Close the loop: connect last drawn point back to first
                    if let Some(first) = polyline.first() {
                        if let Some(last_pos) = prev_pos {
                            let first_pos = if let Some(m) = tf {
                                transform_point(first, m)
                            } else {
                                [first.x as f32, first.y as f32, first.z as f32]
                            };
                            let dx = first_pos[0] - last_pos[0];
                            let dy = first_pos[1] - last_pos[1];
                            let dz = first_pos[2] - last_pos[2];
                            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                            if dist > 1e-6 {
                                edge_vertices.push(LineVertex {
                                    position: last_pos,
                                    color: edge_color,
                                });
                                edge_vertices.push(LineVertex {
                                    position: first_pos,
                                    color: edge_color,
                                });
                            }
                        }
                    }
                }

                // Inner boundary polylines (holes)
                for polyline in &face.inner_boundaries {
                    if polyline.len() < 2 {
                        continue;
                    }
                    let step = if polyline.len() > 500 {
                        (polyline.len() as f64 / 500.0).ceil() as usize
                    } else {
                        1
                    };
                    let mut prev_pos: Option<[f32; 3]> = None;
                    for i in (0..polyline.len()).step_by(step) {
                        let p = &polyline[i];
                        let pos = if let Some(m) = tf {
                            transform_point(p, m)
                        } else {
                            [p.x as f32, p.y as f32, p.z as f32]
                        };
                        if let Some(pp) = prev_pos {
                            edge_vertices.push(LineVertex {
                                position: pp,
                                color: edge_color,
                            });
                            edge_vertices.push(LineVertex {
                                position: pos,
                                color: edge_color,
                            });
                        }
                        prev_pos = Some(pos);
                    }
                    if let Some(first) = polyline.first() {
                        if let Some(last_pos) = prev_pos {
                            let first_pos = if let Some(m) = tf {
                                transform_point(first, m)
                            } else {
                                [first.x as f32, first.y as f32, first.z as f32]
                            };
                            let dx = first_pos[0] - last_pos[0];
                            let dy = first_pos[1] - last_pos[1];
                            let dz = first_pos[2] - last_pos[2];
                            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                            if dist > 1e-6 {
                                edge_vertices.push(LineVertex {
                                    position: last_pos,
                                    color: edge_color,
                                });
                                edge_vertices.push(LineVertex {
                                    position: first_pos,
                                    color: edge_color,
                                });
                            }
                        }
                    }
                }
            }
        }

        edge_vertices
    }

    /// Build wireframe overlay line vertices from the mesh's triangle data.
    ///
    /// This generates LineVertex pairs for each triangle edge, creating a wireframe
    /// overlay that shows the mesh triangulation structure. Uses LineList topology
    /// which works on ALL platforms including WebGPU/WASM (unlike PolygonMode::Line).
    ///
    /// The vertices are in the same coordinate space as the mesh (world space),
    /// so depth testing against the solid mesh will properly occlude hidden edges.
    ///
    /// For large meshes (>100K triangles), the wireframe overlay is skipped entirely
    /// because it would consume too much GPU memory and rendering it would be slow.
    /// The B-Rep edge lines already show the model structure without the overhead.
    fn build_wireframe_overlay_vertices(&self) -> Vec<LineVertex> {
        // Skip wireframe overlay for extremely large meshes — it would consume
        // too much GPU memory. With edge deduplication, each manifold edge
        // requires only 2 LineVertex × 24 bytes = 48 bytes. For a 1M-triangle
        // mesh, there are ~1.5M edges, needing ~72 MB — still manageable.
        const MAX_TRIANGLES_FOR_WIREFRAME_OVERLAY: usize = 1_000_000;
        if self.mesh.triangles.len() > MAX_TRIANGLES_FOR_WIREFRAME_OVERLAY {
            log::info!(
                "Skipping wireframe overlay: {} triangles > {} limit. B-Rep edges still shown.",
                self.mesh.triangles.len(),
                MAX_TRIANGLES_FOR_WIREFRAME_OVERLAY,
            );
            return Vec::new();
        }

        let mut vertices: Vec<LineVertex> = Vec::new();

        // Wireframe overlay color: subtle dark gray
        let overlay_color: [f32; 3] = [0.15, 0.15, 0.20];

        let mesh = &self.mesh;
        let hidden = &self.hidden_instances;
        let ranges = &self.instance_triangle_ranges;

        // Deduplicate edges: each shared edge between adjacent triangles is
        // drawn only once instead of twice. This cuts the vertex count by ~50%
        // for manifold meshes and makes the overlay work for much larger models.
        let mut edge_set: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();

        for (i, tri) in mesh.triangles.iter().enumerate() {
            // Check if this triangle belongs to a hidden instance
            let mut is_hidden = false;
            for (idx, &(start, end)) in ranges.iter().enumerate() {
                if i >= start && i < end && hidden.contains(&idx) {
                    is_hidden = true;
                    break;
                }
            }
            if is_hidden {
                continue;
            }

            let v0_idx = tri[0] as usize;
            let v1_idx = tri[1] as usize;
            let v2_idx = tri[2] as usize;
            // Safety: skip triangles with out-of-bounds vertex indices
            if v0_idx >= mesh.vertices.len() || v1_idx >= mesh.vertices.len() || v2_idx >= mesh.vertices.len() {
                log::warn!("Wireframe overlay: skipping triangle with OOB indices [{}, {}, {}] (max={})", v0_idx, v1_idx, v2_idx, mesh.vertices.len());
                continue;
            }

            // Generate edges with deduplication
            for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let edge = if a < b { (a, b) } else { (b, a) };
                if edge_set.insert(edge) {
                    let va = &mesh.vertices[a as usize];
                    let vb = &mesh.vertices[b as usize];
                    vertices.push(LineVertex {
                        position: [va.x as f32, va.y as f32, va.z as f32],
                        color: overlay_color,
                    });
                    vertices.push(LineVertex {
                        position: [vb.x as f32, vb.y as f32, vb.z as f32],
                        color: overlay_color,
                    });
                }
            }
        }

        vertices
    }

    /// Cancel any in-progress loading.
    fn cancel_loading(&mut self) {
        self.is_loading = false;
        self.loading_start = None;
        self.pending_breps.clear();
        // Drop the conversion context and step file to free WASM memory.
        // Without this, repeated file loads accumulate data in the WASM heap
        // (which has a 4GB limit in Chrome) and eventually crash the tab.
        self.conversion_ctx = None;
        self.pending_step_file = None;
        self.triangulated_count = 0;
        self.total_instance_count = 0;
    }

    /// Process pending BREPs with time budget (called from update()).
    ///
    /// Uses `ChunkedBrepTriangulator` to respect a per-frame time budget,
    /// keeping the browser/UI responsive during loading of large assemblies.
    /// On WASM, the budget is 8ms (for 120fps); on native, 16ms (for 60fps).
    ///
    /// Individual BREPs are processed atomically — if a single BREP takes longer
    /// than the budget, we accept the frame drop but log a warning. This is
    /// acceptable because most BREPs have 6-50 faces and complete in <5ms.
    ///
    /// Returns true if there are still more instances to process.
    fn process_pending_breps(&mut self) -> bool {
        if !self.is_loading || self.pending_breps.is_empty() {
            return false;
        }

        // LAZY: Create the conversion context on the first frame.
        if self.conversion_ctx.is_none() {
            if let Some(step_file) = self.pending_step_file.take() {
                self.log("Building conversion context (entity maps, bounding box)...");
                let ctx_result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    OwnedStepConversionContext::new(step_file)
                }));
                match ctx_result {
                    Ok(ctx) => {
                        self.conversion_ctx = Some(ctx);
                        #[cfg(not(target_arch = "wasm32"))]
                        self.log("Conversion context ready — starting triangulation (parallel mode)...");
                        #[cfg(target_arch = "wasm32")]
                        self.log("Conversion context ready — starting triangulation...");
                    }
                    Err(_) => {
                        self.log_error("Panic during conversion context creation — STEP loading aborted");
                        self.is_loading = false;
                        self.conversion_ctx = None;
                        self.pending_step_file = None;
                        return false;
                    }
                }
            } else {
                self.is_loading = false;
                return false;
            }
        }

        // Check for global loading timeout (5 minutes)
        if let Some(start) = self.loading_start {
            let timeout = std::time::Duration::from_secs(300);
            if start.elapsed() > timeout {
                let elapsed = start.elapsed().as_secs();
                let remaining = self.pending_breps.len();
                self.log_warning(&format!(
                    "Loading timed out after {}s — {} instances remaining, showing partial result",
                    elapsed, remaining
                ));
                self.is_loading = false;
                self.conversion_ctx = None;
                self.pending_step_file = None;
                self.loading_start = None;
                if self.mesh.vertex_count() > 0 {
                    self.load_mesh(self.mesh.clone(), &format!("STEP (partial): {}", self.loading_name));
                }
                self.loading_name.clear();
                return false;
            }
        }

        // Time-budgeted processing: process BREPs until the frame budget is exceeded.
        // Use web_time::Instant on WASM (std::time::Instant panics on wasm32).
        #[cfg(not(target_arch = "wasm32"))]
        let frame_start = std::time::Instant::now();
        #[cfg(target_arch = "wasm32")]
        let frame_start = web_time::Instant::now();
        let frame_budget = self.chunked_triangulator.time_budget;

        let mut processed = 0;
        // On native, process up to 8 BREPs per frame if time allows.
        // On WASM, process just 1 BREP per frame.
        #[cfg(not(target_arch = "wasm32"))]
        let max_batch = std::cmp::min(self.pending_breps.len(), 8);
        #[cfg(target_arch = "wasm32")]
        let max_batch = 1;

        while processed < max_batch && !self.pending_breps.is_empty() {
            // Check time budget (skip check for first BREP — we always process at least one)
            if processed > 0 && frame_start.elapsed() > frame_budget {
                log::debug!("Frame time budget exceeded after {} BREPs ({:.1}ms), yielding",
                    processed, frame_start.elapsed().as_secs_f64() * 1000.0);
                break;
            }

            // Take the next pending BREP
            let pending = self.pending_breps.remove(0);

            // Log transform info for debugging positioning issues
            if let Some(ref tf) = pending.transform {
                let is_identity = (tf[0][0] - 1.0).abs() < 1e-10 && (tf[1][1] - 1.0).abs() < 1e-10 && (tf[2][2] - 1.0).abs() < 1e-10 && tf[0][3].abs() < 1e-10 && tf[1][3].abs() < 1e-10 && tf[2][3].abs() < 1e-10;
                if !is_identity {
                    let has_rotation = (tf[0][0] - 1.0).abs() > 1e-6 || (tf[1][1] - 1.0).abs() > 1e-6 || (tf[2][2] - 1.0).abs() > 1e-6
                        || tf[0][1].abs() > 1e-6 || tf[0][2].abs() > 1e-6
                        || tf[1][0].abs() > 1e-6 || tf[1][2].abs() > 1e-6
                        || tf[2][0].abs() > 1e-6 || tf[2][1].abs() > 1e-6;
                    if has_rotation {
                        self.log(&format!(
                            "Instance '{}' (BREP #{}): non-identity transform with ROTATION — translation=({:.3}, {:.3}, {:.3}), matrix=[[{:.4},{:.4},{:.4}],[{:.4},{:.4},{:.4}],[{:.4},{:.4},{:.4}]]",
                            pending.name, pending.brep_id,
                            tf[0][3], tf[1][3], tf[2][3],
                            tf[0][0], tf[0][1], tf[0][2],
                            tf[1][0], tf[1][1], tf[1][2],
                            tf[2][0], tf[2][1], tf[2][2]
                        ));
                    } else {
                        self.log(&format!(
                            "Instance '{}' (BREP #{}): non-identity transform — translation=({:.3}, {:.3}, {:.3})",
                            pending.name, pending.brep_id, tf[0][3], tf[1][3], tf[2][3]
                        ));
                    }
                }
            }

            // Time this BREP's triangulation
            #[cfg(not(target_arch = "wasm32"))]
            let brep_start = std::time::Instant::now();
            #[cfg(target_arch = "wasm32")]
            let brep_start = web_time::Instant::now();

            // Triangulate this single BREP using the CACHED conversion context.
            let instance = if let Some(ref mut ctx) = self.conversion_ctx {
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    ctx.triangulate_pending(&pending)
                })).unwrap_or_else(|_| {
                    log::error!("Panic during triangulation of '{}' (BREP #{}), skipping", pending.name, pending.brep_id);
                    None
                })
            } else {
                None
            };

            let brep_elapsed = brep_start.elapsed();
            let brep_elapsed_ms = brep_elapsed.as_secs_f64() * 1000.0;

            // Warn if this BREP exceeded the time budget (indicates a complex BREP
            // that would benefit from intra-BREP chunked triangulation in the future)
            if brep_elapsed > frame_budget && processed == 0 {
                log::warn!("BREP '{}' took {:.1}ms (exceeded {:.0}ms budget) — consider intra-BREP chunking",
                    pending.name, brep_elapsed_ms, frame_budget.as_secs_f64() * 1000.0);
            }

            match instance {
                Some(inst) => {
                    if inst.mesh.triangle_count() == 0 && inst.mesh.vertex_count() == 0 {
                        self.log_warning(&format!(
                            "Instance '{}' (BREP #{}) produced empty mesh — skipping ({:.1}ms)",
                            inst.name, inst.brep_id, brep_elapsed_ms
                        ));
                        self.failed_face_count += 1;
                    } else {
                        let tri_start = self.mesh.triangle_count();
                        let color = inst.color.unwrap_or_else(|| {
                            Self::instance_color(self.triangulated_count)
                        });
                        self.mesh.merge_with_color(&inst.mesh, color);
                        let tri_end = self.mesh.triangle_count();
                        self.instance_triangle_ranges.push((tri_start, tri_end));

                        let inst_idx = self.instance_triangle_ranges.len() - 1;
                        if let Some(ref mut tree) = self.assembly_tree {
                            assign_instance_to_tree(tree, inst_idx);
                        }

                        self.detailed_instances.push(inst);
                    }
                }
                None => {
                    self.log_warning(&format!(
                        "Instance '{}' (BREP #{}) failed triangulation — skipping ({:.1}ms)",
                        pending.name, pending.brep_id, brep_elapsed_ms
                    ));
                    self.failed_face_count += 1;
                }
            }

            self.triangulated_count += 1;
            processed += 1;
        }

        self.mesh_dirty = true;
        self.edge_dirty = true;
        self.wireframe_overlay_dirty = true;

        if self.pending_breps.is_empty() {
            // Loading complete — free the conversion context (and its StepFile)
            self.is_loading = false;
            self.conversion_ctx = None;
            self.loading_start = None;
            let vcount = self.mesh.vertex_count();
            let tcount = self.mesh.triangle_count();
            self.log(&format!(
                "Triangulation complete: {} instances, {} vertices, {} triangles",
                self.triangulated_count, vcount, tcount
            ));
            if self.failed_face_count > 0 {
                self.log_warning(&format!("{} instances failed triangulation", self.failed_face_count));
            }
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
        self.log(&format!("Parsing STEP file: '{}' ({} chars)...", name, content.len()));
        // Wrap parse_step in catch_unwind to prevent WASM panics from crashing the app.
        // If STEP parsing panics (e.g., malformed input), we log the error and keep running.
        let parse_result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            draper_step::parse_step(content)
        }));
        match parse_result {
            Ok(Ok(step_file)) => {
                let entity_count = step_file.entities.len();
                self.log(&format!("STEP parsed: {} entities found in '{}'", entity_count, name));
                self.process_step_file(&step_file, name);
            }
            Ok(Err(e)) => {
                self.log_error(&format!("STEP import error for '{}': {}", name, e));
            }
            Err(_) => {
                self.log_error(&format!("STEP parser panicked on '{}' — file may be malformed", name));
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
        let input_for_cleanup: web_sys::HtmlElement = input.clone().unchecked_into();

        let onchange = Closure::wrap(Box::new(move |_: web_sys::Event| {
            if let Some(files) = input_elem_for_closure.files() {
                if let Some(file) = files.get(0) {
                    let file_name = file.name();
                    log::info!("STL file selected: '{}'", file_name);

                    let reader = web_sys::FileReader::new().unwrap();
                    let reader_clone = reader.clone();
                    let shared = shared_result.clone();

                    let onload = Closure::wrap(Box::new(move |_: web_sys::Event| {
                        if let Ok(result) = reader_clone.result() {
                            let array_buffer: js_sys::ArrayBuffer = result.into();
                            let uint8_array = js_sys::Uint8Array::new(&array_buffer);
                            let data = uint8_array.to_vec();
                            log::info!("STL file loaded: {} bytes", data.len());
                            *shared.lock().unwrap() = Some(FileLoadResult::Stl {
                                name: file_name.clone(),
                                data,
                            });
                        } else {
                            log::error!("STL file read_as_array_buffer() returned error");
                        }
                    }) as Box<dyn FnMut(_)>);

                    let onerror = Closure::wrap(Box::new(move |_evt: web_sys::Event| {
                        log::error!("FileReader error while reading STL file");
                    }) as Box<dyn FnMut(_)>);

                    reader.set_onload(Some(onload.as_ref().unchecked_ref()));
                    onload.forget();
                    reader.set_onerror(Some(onerror.as_ref().unchecked_ref()));
                    onerror.forget();
                    let _ = reader.read_as_array_buffer(&file);
                }
            }
            // Remove the file input element from DOM after use
            if let Some(parent) = input_for_cleanup.parent_node() {
                let _ = parent.remove_child(&input_for_cleanup);
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
        // Clone the input element for cleanup after file selection
        let input_for_cleanup: web_sys::HtmlElement = input.clone().unchecked_into();

        let onchange = Closure::wrap(Box::new(move |_: web_sys::Event| {
            if let Some(files) = input_elem_for_closure.files() {
                if let Some(file) = files.get(0) {
                    let file_name = file.name();
                    let file_size = file.size();
                    log::info!("STEP file selected: '{}' ({} bytes)", file_name, file_size);

                    // Check file size limit (50MB max for WASM)
                    if file_size > 50.0 * 1024.0 * 1024.0 {
                        log::error!("STEP file too large: {} bytes (max 50MB)", file_size);
                        *shared_result.lock().unwrap() = None;
                        return;
                    }

                    let reader = web_sys::FileReader::new().unwrap();
                    let reader_clone = reader.clone();
                    let shared = shared_result.clone();
                    let name_for_log = file_name.clone();

                    let onload = Closure::wrap(Box::new(move |_: web_sys::Event| {
                        log::info!("STEP file loaded into memory: '{}'", name_for_log);
                        if let Ok(result) = reader_clone.result() {
                            if let Some(text) = result.as_string() {
                                log::info!("STEP file text extracted: {} chars", text.len());
                                *shared.lock().unwrap() = Some(FileLoadResult::Step {
                                    name: name_for_log.clone(),
                                    content: text,
                                });
                            } else {
                                log::error!("STEP file read result is not a string — file may be binary or have encoding issues");
                            }
                        } else {
                            log::error!("STEP file read_as_text() returned error");
                        }
                    }) as Box<dyn FnMut(_)>);

                    // Add onerror handler
                    let onerror = Closure::wrap(Box::new(move |_evt: web_sys::Event| {
                        log::error!("FileReader error while reading STEP file");
                    }) as Box<dyn FnMut(_)>);

                    reader.set_onload(Some(onload.as_ref().unchecked_ref()));
                    onload.forget();
                    reader.set_onerror(Some(onerror.as_ref().unchecked_ref()));
                    onerror.forget();

                    match reader.read_as_text(&file) {
                        Ok(()) => {
                            log::info!("Started reading STEP file: '{}'", file_name);
                        }
                        Err(_) => {
                            log::error!("Failed to start reading STEP file: '{}'", file_name);
                        }
                    }
                }
            }
            // Remove the file input element from DOM after use
            if let Some(parent) = input_for_cleanup.parent_node() {
                let _ = parent.remove_child(&input_for_cleanup);
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
        let result = self.file_result.lock().unwrap_or_else(|e| e.into_inner()).take();
        if let Some(file_result) = result {
            match file_result {
                FileLoadResult::Step { name, content } => {
                    log::info!("Processing loaded STEP file: '{}' ({} chars)", name, content.len());
                    self.import_step_from_str(&content, &name);
                }
                FileLoadResult::Stl { name, data } => {
                    log::info!("Processing loaded STL file: '{}' ({} bytes)", name, data.len());
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

        // Process progressive triangulation (one BREP per frame)
        if self.is_loading {
            self.process_pending_breps();
            ctx.request_repaint(); // Keep repainting during loading
        }

        // === Detect mobile mode (narrow screen) ===
        let screen_width = ctx.screen_rect().width();
        self.is_mobile = screen_width < 768.0;

        // === Top menu bar (desktop only — mobile uses overlay buttons) ===
        if !self.is_mobile {
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
                        if ui.button("Export JSON...").clicked() {
                            if let Some(path) = rfd::FileDialog::new()
                                .add_filter("JSON", &["json"])
                                .save_file()
                            {
                                self.export_json(&path.to_string_lossy());
                            }
                            ui.close_menu();
                        }
                        ui.separator();
                        if ui.button("Import JSON...").clicked() {
                            if let Some(path) = rfd::FileDialog::new()
                                .add_filter("JSON", &["json"])
                                .pick_file()
                            {
                                self.import_json(&path.to_string_lossy());
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
                    ui.checkbox(&mut self.show_edges, "Show Edges");
                    ui.checkbox(&mut self.show_wireframe_overlay, "Mesh Overlay");
                    ui.checkbox(&mut self.show_axes, "Show axes");
                    ui.checkbox(&mut self.show_grid, "Show grid");
                    ui.checkbox(&mut self.show_structure, "Structure Panel");
                    ui.separator();
                    ui.checkbox(&mut self.validate_consistency, "Validate Edge Consistency");
                    if let Some(ref report) = self.last_consistency_report {
                        ui.label(egui::RichText::new(report).small().color(egui::Color32::YELLOW));
                    }
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
        } // end desktop top menu bar

        // === Bottom panel: log (collapsible — especially important on mobile) ===
        // On mobile, the log panel can cover the entire viewport when expanded,
        // so it MUST have a collapse/expand toggle. On desktop the toggle is also
        // useful for users who want more 3D viewport space.
        if self.log_panel_open {
            egui::TopBottomPanel::bottom("log_panel")
                .min_height(60.0)
                .default_height(100.0)
                .resizable(true)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        // Collapse button — always visible, easy to tap on mobile
                        if ui.button("▼").clicked() {
                            self.log_panel_open = false;
                        }
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
        } else {
            // Collapsed: thin bar with only expand button and last log line
            egui::TopBottomPanel::bottom("log_panel_collapsed")
                .exact_height(24.0)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        // Expand button — easy to tap on mobile
                        if ui.button("▲").clicked() {
                            self.log_panel_open = true;
                        }
                        ui.heading(egui::RichText::new("Log").size(11.0));
                        // Show warning/error counts as badges
                        if self.warning_count > 0 {
                            ui.label(egui::RichText::new(format!("⚠ {}", self.warning_count))
                                .size(10.0)
                                .color(egui::Color32::from_rgb(255, 200, 50)));
                        }
                        if self.error_count > 0 {
                            ui.label(egui::RichText::new(format!("✖ {}", self.error_count))
                                .size(10.0)
                                .color(egui::Color32::from_rgb(255, 80, 80)));
                        }
                        ui.separator();
                        // Show last log entry as preview
                        if let Some(last) = self.log.last() {
                            let color = match last.severity {
                                LogSeverity::Info => egui::Color32::from_rgb(200, 200, 210),
                                LogSeverity::Warning => egui::Color32::from_rgb(255, 200, 50),
                                LogSeverity::Error => egui::Color32::from_rgb(255, 80, 80),
                            };
                            ui.add(egui::Label::new(
                                egui::RichText::new(&last.message).size(9.0).color(color)
                            ).truncate());
                        }
                    });
                });
        }

        // === Right panel: Structure / Faces / UV (desktop only) ===
        // Collect pending UI actions to avoid borrow checker conflicts
        let mut pending_instance_select: Option<usize> = None;
        let mut pending_face_select: Option<(usize, u64)> = None;
        let mut pending_svg_export = false;
        let mut pending_copy_face_id: Option<u64> = None;
        let mut pending_visibility_toggle: Option<usize> = None;

        if self.show_structure && !self.is_mobile {
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
            let hidden_instances = self.hidden_instances.clone();

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
                                        draw_assembly_node_static(ui, tree, selected_instance, &hidden_instances, &mut pending_instance_select, &mut pending_visibility_toggle, &open_tree_nodes, &scroll_to_tree_node);
                                    } else if !detailed_instances_clone.is_empty() {
                                        for (i, inst) in detailed_instances_clone.iter().enumerate() {
                                            let is_selected = selected_instance == Some(i);
                                            let is_visible = !hidden_instances.contains(&i);
                                            ui.horizontal(|ui| {
                                                // Visibility eye icon
                                                let eye_color = if is_visible {
                                                    egui::Color32::from_rgb(80, 180, 80)
                                                } else {
                                                    egui::Color32::from_rgb(180, 80, 80)
                                                };
                                                let eye_text = if is_visible { "👁" } else { "  " };
                                                if ui.add(egui::Label::new(egui::RichText::new(eye_text).size(11.0).color(eye_color)).sense(egui::Sense::click())).clicked() {
                                                    pending_visibility_toggle = Some(i);
                                                }
                                                // Selectable label
                                                let label = format!("{} (BREP#{})", inst.name, inst.brep_id);
                                                if ui.selectable_label(is_selected, &label).clicked() {
                                                    pending_instance_select = Some(i);
                                                }
                                            });
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
                                                    "Face ID: {}\nSTEP ID: {}\nSurface: {}\nTriangles: [{}, {})\nBoundary loops: {} ({} pts)\nHoles: {} ({} pts)\nForward: {}",
                                                    face.face_id, face.step_face_id, face.surface_type,
                                                    face.triangle_range.0, face.triangle_range.1,
                                                    face.outer_boundary.len(), face.outer_boundary.iter().map(|p| p.len()).sum::<usize>(),
                                                    face.inner_boundaries.len(), face.inner_boundaries.iter().map(|p| p.len()).sum::<usize>(),
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

                                            // Draw UV triangles (the actual triangulation)
                                            if !face.uv_triangles.is_empty() {
                                                // Build hole polygons for classification
                                                let hole_polys: Vec<Vec<(f64, f64)>> = face.inner_uv_boundaries.iter()
                                                    .flat_map(|boundaries| boundaries.iter().map(|poly| {
                                                        poly.iter().map(|pt| (pt.u, pt.v)).collect()
                                                    }))
                                                    .collect();

                                                let tri_limit = 2000.min(face.uv_triangles.len());
                                                for (ti, tri) in face.uv_triangles.iter().enumerate() {
                                                    let cu = (tri[0].u + tri[1].u + tri[2].u) / 3.0;
                                                    let cv = (tri[0].v + tri[1].v + tri[2].v) / 3.0;
                                                    let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h));
                                                    let in_outer = !outer_uv_poly.is_empty() && point_in_polygon(cu, cv, &outer_uv_poly);

                                                    let p0 = egui::pos2(map_u(tri[0].u), map_v(tri[0].v));
                                                    let p1 = egui::pos2(map_u(tri[1].u), map_v(tri[1].v));
                                                    let p2 = egui::pos2(map_u(tri[2].u), map_v(tri[2].v));

                                                    if in_hole || !in_outer {
                                                        // Triangle inside hole — red fill + outline
                                                        let points = vec![p0, p1, p2];
                                                        ui.painter().add(egui::Shape::convex_polygon(
                                                            points,
                                                            egui::Color32::from_rgba_premultiplied(255, 34, 34, 50),
                                                            egui::Stroke::new(0.5, egui::Color32::from_rgba_premultiplied(255, 68, 68, 120)),
                                                        ));
                                                    } else {
                                                        // Valid triangle — blue fill + outline
                                                        let points = vec![p0, p1, p2];
                                                        let fill = if ti % 2 == 0 {
                                                            egui::Color32::from_rgba_premultiplied(68, 136, 255, 20)
                                                        } else {
                                                            egui::Color32::from_rgba_premultiplied(85, 170, 255, 20)
                                                        };
                                                        let stroke = if ti % 2 == 0 {
                                                            egui::Stroke::new(0.4, egui::Color32::from_rgba_premultiplied(68, 136, 255, 160))
                                                        } else {
                                                            egui::Stroke::new(0.4, egui::Color32::from_rgba_premultiplied(85, 170, 255, 160))
                                                        };
                                                        ui.painter().add(egui::Shape::convex_polygon(points, fill, stroke));
                                                    }
                                                    if ti >= tri_limit { break; }
                                                }
                                            }

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
                                            let outer_pt_count: usize = face.outer_boundary.iter().map(|p| p.len()).sum();
                                            ui.label(egui::RichText::new(format!("Boundary loops: {} ({} pts)", face.outer_boundary.len(), outer_pt_count)).size(11.0));
                                            let inner_pt_count: usize = face.inner_boundaries.iter().map(|p| p.len()).sum();
                                            ui.label(egui::RichText::new(format!("Holes: {} ({} pts)", face.inner_boundaries.len(), inner_pt_count)).size(11.0));
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
        if let Some(idx) = pending_visibility_toggle {
            if self.hidden_instances.contains(&idx) {
                self.hidden_instances.remove(&idx);
                self.log(&format!("Instance #{} shown", idx));
            } else {
                self.hidden_instances.insert(idx);
                self.log(&format!("Instance #{} hidden", idx));
                // If the hidden instance was selected, deselect it
                if self.selected_instance == Some(idx) {
                    self.selected_instance = None;
                    self.selected_face = None;
                    self.highlighted_face = None;
                }
            }
            self.highlight_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;
        }
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

        // === Left side panel (controls) — desktop only ===
        if !self.is_mobile {
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
                    if ui.button("Revolution").clicked() { self.load_revolution(); }
                });
                ui.horizontal(|ui| {
                    if ui.button("Extrusion").clicked() { self.load_extrusion(); }
                    if ui.button("NURBS").clicked() { self.load_nurbs(); }
                });
                // --- Hole: 3 ---
                ui.separator();
                ui.heading(egui::RichText::new("Hole: 3").size(12.0));
                ui.label(egui::RichText::new("Cut-out \"3\" on surfaces").size(9.0).color(egui::Color32::GRAY));
                ui.horizontal(|ui| {
                    if ui.button("Box~").clicked() { self.load_box_text(); }
                    if ui.button("Cyl~").clicked() { self.load_cylinder_text(); }
                    if ui.button("Sph~").clicked() { self.load_sphere_text(); }
                });
                ui.horizontal(|ui| {
                    if ui.button("Cone~").clicked() { self.load_cone_text(); }
                    if ui.button("Torus~").clicked() { self.load_torus_text(); }
                    if ui.button("Rev~").clicked() { self.load_revolution_text(); }
                });
                ui.horizontal(|ui| {
                    if ui.button("Ext~").clicked() { self.load_extrusion_text(); }
                    if ui.button("NURBS~").clicked() { self.load_nurbs_text(); }
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
                        if ui.button("Export JSON").clicked() {
                            if let Some(path) = rfd::FileDialog::new()
                                .add_filter("JSON", &["json"])
                                .save_file()
                            {
                                self.export_json(&path.to_string_lossy());
                            }
                        }
                    });
                    ui.horizontal(|ui| {
                        if ui.button("Import JSON").clicked() {
                            if let Some(path) = rfd::FileDialog::new()
                                .add_filter("JSON", &["json"])
                                .pick_file()
                            {
                                self.import_json(&path.to_string_lossy());
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
                ui.checkbox(&mut self.show_edges, "Show Edges");
                ui.checkbox(&mut self.show_wireframe_overlay, "Mesh Overlay");
                ui.checkbox(&mut self.show_axes, "Show axes");
                ui.checkbox(&mut self.show_grid, "Show grid");
                ui.checkbox(&mut self.show_structure, "Structure Panel");
                ui.checkbox(&mut self.show_json_api, "JSON API");

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

                // --- JSON API Panel ---
                if self.show_json_api {
                    ui.separator();
                    ui.heading(egui::RichText::new("JSON API").size(12.0));
                    ui.label(egui::RichText::new("Enter command (JSON):").size(10.0));
                    let response = ui.add(
                        egui::TextEdit::multiline(&mut self.json_api_input)
                            .desired_width(f32::INFINITY)
                            .desired_rows(3)
                            .font(egui::TextStyle::Monospace)
                    );
                    let mut execute = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    ui.horizontal(|ui| {
                        if ui.button("Execute").clicked() {
                            execute = true;
                        }
                        if ui.button("Help").clicked() {
                            let help_response = self.json_api.execute(draper_json::ApiRequest::Help);
                            self.json_api_output = serde_json::to_string_pretty(&help_response)
                                .unwrap_or_else(|_| "Error formatting response".to_string());
                        }
                        if ui.button("Stats").clicked() {
                            let resp = self.json_api.execute(draper_json::ApiRequest::GetStats);
                            self.json_api_output = serde_json::to_string_pretty(&resp)
                                .unwrap_or_else(|_| "Error".to_string());
                        }
                        if ui.button("Clear").clicked() {
                            self.json_api_input.clear();
                            self.json_api_output.clear();
                        }
                    });
                    if execute && !self.json_api_input.is_empty() {
                        let result = self.json_api.execute_json(&self.json_api_input);
                        self.json_api_output = result;
                        // If the command loaded a model, sync the viewer
                        let should_sync = self.json_api.model().map_or(false, |m| {
                            let vc = m.metadata.total_vertices;
                            vc > 0 && vc != self.current_model.vertex_count
                        });
                        if should_sync {
                            let (mesh, name, instances, assembly) = {
                                let model = self.json_api.model().unwrap();
                                (model.to_triangle_mesh(), model.metadata.name.clone(), model.to_detailed_instances(), model.assembly.clone())
                            };
                            self.detailed_instances = instances;
                            self.assembly_tree = Some(assembly);
                            self.load_mesh(mesh, &name);
                        }
                    }
                    if !self.json_api_output.is_empty() {
                        ui.label(egui::RichText::new("Response:").size(10.0));
                        ui.add(
                            egui::TextEdit::multiline(&mut self.json_api_output)
                                .desired_width(f32::INFINITY)
                                .desired_rows(6)
                                .font(egui::TextStyle::Monospace)
                                .interactive(false)
                        );
                    }
                }

                ui.add_space(4.0);
                ui.separator();
                ui.label(
                    egui::RichText::new("Click: Select | Ctrl+Click: Face | Drag: Rotate | Scroll: Zoom")
                        .size(9.0)
                        .color(egui::Color32::from_rgb(160, 160, 160))
                );
            });
        } // end desktop left controls panel

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

                    // ─── Mouse/touch picking: click = select solid, Ctrl+click = select face ───
                    // On mobile, tap always selects face (since there's no Ctrl key)
                    if response.clicked_by(egui::PointerButton::Primary) {
                        let ctrl_held = ui.input(|i| i.modifiers.ctrl || i.modifiers.command) || self.is_mobile;
                        let mouse_pos = ui.input(|i| i.pointer.latest_pos());
                        if let Some(pos) = mouse_pos {
                            // Convert screen position to viewport-local coordinates
                            let local_x = pos.x - rect.min.x;
                            let local_y = pos.y - rect.min.y;
                            let viewport = (0.0, 0.0, rect.width(), rect.height());

                            if let Some(pick) = pick_at(
                                &self.mesh,
                                &self.instance_triangle_ranges,
                                &self.hidden_instances,
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
                if self.mesh_dirty || self.highlight_dirty || self.edge_dirty || self.wireframe_overlay_dirty {
                    // Ensure mesh has face normals and colors before GPU upload
                    // (moved from mesh_to_gpu_data to avoid cloning the entire mesh)
                    if self.mesh.face_normals.is_none() {
                        self.mesh.compute_face_normals();
                    }
                    self.mesh.ensure_colors([0.62, 0.65, 0.70, 1.0]);

                    if let Some(ref rs) = self.render_state {
                        let (vertices, indices, _new_ranges) = mesh_to_gpu_data(&self.mesh, self.highlighted_face, self.selected_instance, &self.instance_triangle_ranges, &self.hidden_instances);
                        // NOTE: We intentionally do NOT overwrite instance_triangle_ranges with new_ranges.
                        // The new_ranges map instance indices to triangle ranges in the GPU output buffer
                        // (with hidden instances removed, so indices shift). But self.mesh.triangles still
                        // contains ALL triangles including hidden ones. All subsequent operations —
                        // build_wireframe_overlay_vertices(), pick_at(), and future mesh_to_gpu_data() calls —
                        // iterate over self.mesh.triangles and need the ORIGINAL ranges to correctly determine
                        // which instance each triangle belongs to. Overwriting with GPU output ranges corrupted
                        // the mapping after the first visibility toggle, causing mesh/edges to hide incorrectly.

                        // Build edge line vertices from B-Rep boundary data
                        let edge_vertices = self.build_edge_line_vertices();

                        // Build wireframe overlay line vertices from mesh triangles
                        let wf_overlay_vertices = self.build_wireframe_overlay_vertices();

                        let mut guard = self.gpu_resources.lock().unwrap();
                        if let Some(ref mut resources) = *guard {
                            update_mesh_buffers(resources, &rs.device, &vertices, &indices);
                            update_edge_buffers(resources, &rs.device, &edge_vertices);
                            update_wireframe_overlay_buffers(resources, &rs.device, &wf_overlay_vertices);
                        } else {
                            let resources = create_scene_resources(rs, &vertices, &indices);
                            *guard = Some(resources);
                            // After creating new resources, we need to upload edges and overlay too
                            if let Some(ref mut resources) = *guard {
                                update_edge_buffers(resources, &rs.device, &edge_vertices);
                                update_wireframe_overlay_buffers(resources, &rs.device, &wf_overlay_vertices);
                            }
                        }
                    }
                    self.mesh_dirty = false;
                    self.highlight_dirty = false;
                    self.edge_dirty = false;
                    self.wireframe_overlay_dirty = false;
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
                        light_dir: [cam_fwd[0], cam_fwd[1], cam_fwd[2], 0.35],
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
                    show_edges: self.show_edges,
                    show_wireframe_overlay: self.show_wireframe_overlay,
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

        // ═══════════════════════════════════════════════════════════════════════
        // === MOBILE UI — floating buttons + overlay panels ===
        // ═══════════════════════════════════════════════════════════════════════
        if self.is_mobile {
            self.draw_mobile_ui(ctx);
        }
    }
}

impl ViewerApp {
    /// Draw mobile-specific UI: floating action buttons and overlay panels.
    fn draw_mobile_ui(&mut self, ctx: &egui::Context) {
        let screen = ctx.screen_rect();
        let btn_size = 44.0;
        let margin = 8.0;

        // ─── Top bar: compact menu ─────────────────────────────────────
        egui::TopBottomPanel::top("mobile_top_bar")
            .exact_height(40.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading(egui::RichText::new("3Draper").size(14.0));
                    ui.separator();
                    // File menu
                    ui.menu_button("File", |ui| {
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
                    });
                    // View presets
                    ui.menu_button("View", |ui| {
                        if ui.button("Reset Camera").clicked() {
                            let (bbox_min, bbox_max) = self.mesh.bounding_box();
                            self.camera.fit_to_bounding_box(
                                [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                                [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                            );
                            ui.close_menu();
                        }
                        if ui.button("Top").clicked() {
                            self.camera.look_from_direction([0.0, -1.0, 0.0]);
                            ui.close_menu();
                        }
                        if ui.button("Front").clicked() {
                            self.camera.look_from_direction([0.0, 0.0, 1.0]);
                            ui.close_menu();
                        }
                        if ui.button("Isometric").clicked() {
                            let d = 45.0_f32.to_radians();
                            let e = 30.0_f32.to_radians();
                            self.camera.look_from_direction([
                                -e.cos() * d.sin(), -e.sin(), e.cos() * d.cos(),
                            ]);
                            ui.close_menu();
                        }
                        ui.separator();
                        ui.checkbox(&mut self.wireframe, "Wireframe");
                        ui.checkbox(&mut self.show_edges, "Edges");
                        ui.checkbox(&mut self.show_wireframe_overlay, "Mesh Overlay");
                        ui.checkbox(&mut self.show_axes, "Axes");
                    });
                    // Quick primitives
                    ui.menu_button("Models", |ui| {
                        if ui.button("Box").clicked() { self.load_box(); ui.close_menu(); }
                        if ui.button("Cylinder").clicked() { self.load_cylinder(); ui.close_menu(); }
                        if ui.button("Sphere").clicked() { self.load_sphere(); ui.close_menu(); }
                        if ui.button("Cone").clicked() { self.load_cone(); ui.close_menu(); }
                        if ui.button("Torus").clicked() { self.load_torus(); ui.close_menu(); }
                        if ui.button("Engine").clicked() { self.load_engine(); ui.close_menu(); }
                    });
                    // Spacer
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // Info summary
                        let info_text = format!("V:{} T:{}", self.current_model.vertex_count, self.current_model.triangle_count);
                        ui.label(egui::RichText::new(info_text).size(10.0).color(egui::Color32::GRAY));
                    });
                });
            });

        // ─── Floating buttons: bottom-left (Controls) + bottom-right (Structure) ─
        // When mobile log is open, push floating buttons up so they aren't covered by the log window.
        let log_window_height = if self.mobile_log_open { (screen.height() * 0.4).min(250.0) + 55.0 } else { 0.0 };
        let controls_btn_pos = egui::Pos2::new(screen.min.x + margin, screen.bottom() - margin - btn_size - 50.0 - log_window_height);
        let structure_btn_pos = egui::Pos2::new(screen.right() - margin - btn_size, screen.bottom() - margin - btn_size - 50.0 - log_window_height);
        let log_btn_pos = egui::Pos2::new(screen.center().x - btn_size * 0.5, screen.bottom() - margin - btn_size - 50.0 - log_window_height);

        // Controls panel button (bottom-left)
        let controls_active = self.mobile_panel == Some(MobilePanel::Controls);
        egui::Area::new(egui::Id::new("mobile_controls_btn"))
            .fixed_pos(controls_btn_pos)
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                let (rect, _) = ui.allocate_exact_size(
                    egui::vec2(btn_size, btn_size),
                    egui::Sense::click(),
                );
                let fill = if controls_active {
                    egui::Color32::from_rgba_premultiplied(60, 120, 200, 220)
                } else {
                    egui::Color32::from_rgba_premultiplied(40, 40, 50, 200)
                };
                ui.painter().rect_filled(rect, 8.0, fill);
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "⚙",
                    egui::FontId::proportional(22.0),
                    egui::Color32::WHITE,
                );
                if ui.interact(rect, egui::Id::new("mobile_controls_click"), egui::Sense::click()).clicked() {
                    self.mobile_panel = if controls_active { None } else { Some(MobilePanel::Controls) };
                }
            });

        // Structure panel button (bottom-right)
        let structure_active = self.mobile_panel == Some(MobilePanel::Structure);
        egui::Area::new(egui::Id::new("mobile_structure_btn"))
            .fixed_pos(structure_btn_pos)
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                let (rect, _) = ui.allocate_exact_size(
                    egui::vec2(btn_size, btn_size),
                    egui::Sense::click(),
                );
                let fill = if structure_active {
                    egui::Color32::from_rgba_premultiplied(60, 120, 200, 220)
                } else {
                    egui::Color32::from_rgba_premultiplied(40, 40, 50, 200)
                };
                ui.painter().rect_filled(rect, 8.0, fill);
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "📋",
                    egui::FontId::proportional(20.0),
                    egui::Color32::WHITE,
                );
                if ui.interact(rect, egui::Id::new("mobile_structure_click"), egui::Sense::click()).clicked() {
                    self.mobile_panel = if structure_active { None } else { Some(MobilePanel::Structure) };
                }
            });

        // Log button (bottom-center)
        let log_active = self.mobile_log_open;
        egui::Area::new(egui::Id::new("mobile_log_btn"))
            .fixed_pos(log_btn_pos)
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                let (rect, _) = ui.allocate_exact_size(
                    egui::vec2(btn_size, btn_size),
                    egui::Sense::click(),
                );
                let fill = if log_active {
                    egui::Color32::from_rgba_premultiplied(60, 120, 200, 220)
                } else if self.error_count > 0 {
                    egui::Color32::from_rgba_premultiplied(200, 60, 60, 200)
                } else if self.warning_count > 0 {
                    egui::Color32::from_rgba_premultiplied(200, 160, 40, 200)
                } else {
                    egui::Color32::from_rgba_premultiplied(40, 40, 50, 200)
                };
                ui.painter().rect_filled(rect, 8.0, fill);
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "📝",
                    egui::FontId::proportional(20.0),
                    egui::Color32::WHITE,
                );
                if ui.interact(rect, egui::Id::new("mobile_log_click"), egui::Sense::click()).clicked() {
                    self.mobile_log_open = !self.mobile_log_open;
                }
            });

        // ─── Touch gesture help (bottom center, small) ─────────────────
        egui::Area::new(egui::Id::new("mobile_touch_help"))
            .fixed_pos(egui::Pos2::new(screen.center().x - 100.0, screen.bottom() - margin - 44.0))
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new("1drag:Rotate  2drag:Pan  Pinch:Zoom  Tap:Select")
                        .size(9.0)
                        .color(egui::Color32::from_rgba_premultiplied(180, 180, 190, 180))
                );
            });

        // ─── Mobile overlay: Controls panel ─────────────────────────────
        if self.mobile_panel == Some(MobilePanel::Controls) {
            let panel_width = (screen.width() * 0.85).min(320.0);
            egui::Window::new("Controls")
                .id(egui::Id::new("mobile_controls_window"))
                .fixed_pos(egui::Pos2::new(screen.min.x, screen.min.y + 42.0))
                .fixed_size(egui::vec2(panel_width, screen.height() - 50.0))
                .resizable(false)
                .collapsible(false)
                .default_open(true)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        // Primitives
                        ui.heading(egui::RichText::new("Primitives").size(13.0));
                        ui.horizontal(|ui| {
                            if ui.button("Box").clicked() { self.load_box(); }
                            if ui.button("Cylinder").clicked() { self.load_cylinder(); }
                            if ui.button("Sphere").clicked() { self.load_sphere(); }
                        });
                        ui.horizontal(|ui| {
                            if ui.button("Cone").clicked() { self.load_cone(); }
                            if ui.button("Torus").clicked() { self.load_torus(); }
                            if ui.button("Revolution").clicked() { self.load_revolution(); }
                        });
                        ui.horizontal(|ui| {
                            if ui.button("Extrusion").clicked() { self.load_extrusion(); }
                            if ui.button("NURBS").clicked() { self.load_nurbs(); }
                        });
                        ui.separator();

                        // Display
                        ui.heading(egui::RichText::new("Display").size(13.0));
                        ui.checkbox(&mut self.wireframe, "Wireframe");
                        ui.checkbox(&mut self.show_edges, "Show Edges");
                        ui.checkbox(&mut self.show_wireframe_overlay, "Mesh Overlay");
                        ui.checkbox(&mut self.show_axes, "Show axes");
                        ui.separator();

                        // Camera
                        ui.heading(egui::RichText::new("Camera").size(13.0));
                        if ui.button("Reset Camera").clicked() {
                            let (bbox_min, bbox_max) = self.mesh.bounding_box();
                            self.camera.fit_to_bounding_box(
                                [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                                [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                            );
                        }
                        ui.horizontal(|ui| {
                            if ui.button("Top").clicked() { self.camera.look_from_direction([0.0, -1.0, 0.0]); }
                            if ui.button("Front").clicked() { self.camera.look_from_direction([0.0, 0.0, 1.0]); }
                            if ui.button("Right").clicked() { self.camera.look_from_direction([-1.0, 0.0, 0.0]); }
                            if ui.button("Iso").clicked() {
                                let d = 45.0_f32.to_radians();
                                let e = 30.0_f32.to_radians();
                                self.camera.look_from_direction([
                                    -e.cos() * d.sin(), -e.sin(), e.cos() * d.cos(),
                                ]);
                            }
                        });
                        ui.separator();

                        // Selection
                        if ui.button("Clear Selection").clicked() {
                            self.selected_instance = None;
                            self.selected_face = None;
                            self.highlighted_face = None;
                            self.highlight_dirty = true;
                            self.uv_svg_cache = None;
                        }
                        ui.separator();

                        // Info
                        ui.heading(egui::RichText::new("Info").size(13.0));
                        ui.label(egui::RichText::new(format!("Model: {}", self.current_model.name)).size(12.0));
                        ui.label(egui::RichText::new(format!("Vertices: {}", self.current_model.vertex_count)).size(12.0));
                        ui.label(egui::RichText::new(format!("Triangles: {}", self.current_model.triangle_count)).size(12.0));
                        ui.label(egui::RichText::new(format!("Instances: {}", self.detailed_instances.len())).size(12.0));
                        if self.is_loading && self.total_instance_count > 0 {
                            let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                            ui.label(egui::RichText::new(format!("Loading: {}/{} ({:.0}%)", self.triangulated_count, self.total_instance_count, progress * 100.0))
                                .size(11.0).color(egui::Color32::from_rgb(80, 180, 80)));
                        }
                        if let Some((inst_idx, fid)) = self.highlighted_face {
                            ui.label(egui::RichText::new(format!("Face: #{} (inst #{})", fid, inst_idx))
                                .size(12.0).color(egui::Color32::from_rgb(255, 220, 50)));
                        }

                        // Manifold
                        ui.separator();
                        ui.heading(egui::RichText::new("Manifold").size(13.0));
                        if let Some(ref report) = self.manifold_report {
                            let watertight = report.is_watertight();
                            let wt_color = if watertight { egui::Color32::from_rgb(80, 200, 80) } else { egui::Color32::from_rgb(255, 100, 80) };
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new("Watertight:").size(12.0));
                                ui.label(egui::RichText::new(if watertight { "Yes" } else { "No" }).size(12.0).color(wt_color));
                            });
                            ui.label(egui::RichText::new(format!("Boundary edges: {}", report.boundary_edge_count)).size(11.0));
                            ui.label(egui::RichText::new(format!("Degenerate tris: {}", report.degenerate_triangle_count)).size(11.0));
                        }

                        // Errors
                        if self.last_error.is_some() || self.failed_face_count > 0 {
                            ui.separator();
                            ui.heading(egui::RichText::new("Errors").size(13.0));
                            if let Some(ref err) = self.last_error {
                                ui.label(egui::RichText::new(err).size(11.0).color(egui::Color32::from_rgb(255, 120, 120)));
                                if ui.button("Dismiss").clicked() { self.last_error = None; }
                            }
                            if self.failed_face_count > 0 {
                                ui.label(egui::RichText::new(format!("{} face(s) failed triangulation", self.failed_face_count))
                                    .size(11.0).color(egui::Color32::from_rgb(255, 220, 100)));
                            }
                        }

                        // JSON API
                        if self.show_json_api {
                            ui.separator();
                            ui.heading(egui::RichText::new("JSON API").size(13.0));
                            ui.add(egui::TextEdit::multiline(&mut self.json_api_input)
                                .desired_width(f32::INFINITY).desired_rows(2).font(egui::TextStyle::Monospace));
                            ui.horizontal(|ui| {
                                if ui.button("Execute").clicked() && !self.json_api_input.is_empty() {
                                    self.json_api_output = self.json_api.execute_json(&self.json_api_input);
                                }
                                if ui.button("Help").clicked() {
                                    let r = self.json_api.execute(draper_json::ApiRequest::Help);
                                    self.json_api_output = serde_json::to_string_pretty(&r).unwrap_or_default();
                                }
                                if ui.button("Clear").clicked() { self.json_api_input.clear(); self.json_api_output.clear(); }
                            });
                            if !self.json_api_output.is_empty() {
                                ui.add(egui::TextEdit::multiline(&mut self.json_api_output)
                                    .desired_width(f32::INFINITY).desired_rows(4).font(egui::TextStyle::Monospace).interactive(false));
                            }
                        }
                    });
                });
        }

        // ─── Mobile overlay: Structure panel ────────────────────────────
        if self.mobile_panel == Some(MobilePanel::Structure) {
            let panel_width = (screen.width() * 0.85).min(360.0);
            let panel_x = screen.right() - panel_width;

            // Collect pending UI actions
            let mut pending_instance_select: Option<usize> = None;
            let mut pending_face_select: Option<(usize, u64)> = None;
            let mut pending_svg_export = false;
            let mut pending_copy_face_id: Option<u64> = None;
            let mut pending_visibility_toggle: Option<usize> = None;

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
            let hidden_instances = self.hidden_instances.clone();

            egui::Window::new("Structure")
                .id(egui::Id::new("mobile_structure_window"))
                .fixed_pos(egui::Pos2::new(panel_x, screen.min.y + 44.0))
                .fixed_size(egui::vec2(panel_width, screen.height() - 50.0))
                .resizable(false)
                .collapsible(false)
                .default_open(true)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        // Loading progress
                        if self.is_loading && self.total_instance_count > 0 {
                            let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                            ui.add(egui::ProgressBar::new(progress).show_percentage());
                        }

                        // Tree
                        ui.heading(egui::RichText::new("Tree").size(13.0));
                        egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                            if let Some(ref tree) = assembly_tree_clone {
                                draw_assembly_node_static(ui, tree, selected_instance, &hidden_instances, &mut pending_instance_select, &mut pending_visibility_toggle, &open_tree_nodes, &scroll_to_tree_node);
                            } else if !detailed_instances_clone.is_empty() {
                                for (i, inst) in detailed_instances_clone.iter().enumerate() {
                                    let is_selected = selected_instance == Some(i);
                                    let is_visible = !hidden_instances.contains(&i);
                                    ui.horizontal(|ui| {
                                        let eye_text = if is_visible { "👁" } else { "  " };
                                        let eye_color = if is_visible { egui::Color32::from_rgb(80, 180, 80) } else { egui::Color32::from_rgb(180, 80, 80) };
                                        if ui.add(egui::Label::new(egui::RichText::new(eye_text).size(12.0).color(eye_color)).sense(egui::Sense::click())).clicked() {
                                            pending_visibility_toggle = Some(i);
                                        }
                                        let label = format!("{} (BREP#{})", inst.name, inst.brep_id);
                                        if ui.selectable_label(is_selected, &label).clicked() {
                                            pending_instance_select = Some(i);
                                        }
                                    });
                                }
                            } else {
                                ui.label(egui::RichText::new("No STEP file loaded").size(11.0).color(egui::Color32::GRAY));
                            }
                        });
                        ui.separator();

                        // Faces
                        ui.heading(egui::RichText::new("Faces").size(13.0));
                        if let Some(inst_idx) = selected_instance {
                            if let Some(inst) = detailed_instances_clone.get(inst_idx) {
                                ui.label(egui::RichText::new(format!("BREP #{} — {} faces", inst.brep_id, inst.faces.len()))
                                    .size(11.0).color(egui::Color32::GRAY));
                                egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                                    for face in &inst.faces {
                                        let is_selected = selected_face == Some((inst_idx, face.face_id));
                                        let label = format!("F#{} STEP#{} {}", face.face_id, face.step_face_id, face.surface_type);
                                        let response = ui.selectable_label(is_selected, &label);
                                        if scroll_to_face_id == Some(face.face_id) {
                                            response.scroll_to_me(Some(egui::Align::Center));
                                        }
                                        if response.clicked() {
                                            pending_face_select = Some((inst_idx, face.face_id));
                                        }
                                    }
                                });
                            }
                        } else {
                            ui.label(egui::RichText::new("Select an instance").size(11.0).color(egui::Color32::GRAY));
                        }
                        ui.separator();

                        // UV Grid
                        ui.heading(egui::RichText::new("UV Grid").size(13.0));
                        ui.checkbox(&mut self.show_uv_grid, "Show UV grid");
                        ui.horizontal(|ui| {
                            ui.label("U:");
                            ui.add(egui::DragValue::new(&mut self.uv_grid_u).range(2..=50));
                            ui.label("V:");
                            ui.add(egui::DragValue::new(&mut self.uv_grid_v).range(2..=50));
                        });

                        if show_uv_grid {
                            if let Some(inst_idx) = selected_instance {
                                if let Some((_, face_id)) = selected_face {
                                    if let Some(inst) = detailed_instances_clone.get(inst_idx) {
                                        if let Some(face) = inst.faces.iter().find(|f| f.face_id == face_id) {
                                            let cache_key = (inst_idx, face_id);
                                            let needs_regen = uv_svg_cache_key != Some(cache_key);
                                            if needs_regen {
                                                let svg = generate_uv_svg(face, uv_grid_u, uv_grid_v);
                                                self.uv_svg_cache = Some((cache_key, svg));
                                            }
                                            let available = ui.available_size();
                                            let size = available.x.min(available.y - 20.0).min(350.0);
                                            if size > 50.0 {
                                                let (rect, _response) = ui.allocate_exact_size(
                                                    egui::vec2(size, size),
                                                    egui::Sense::hover(),
                                                );
                                                ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(26, 26, 46));

                                                let margin_f = size * 0.067;
                                                let draw_size = size - 2.0 * margin_f;

                                                // Compute UV bounds
                                                let mut u_min = f64::MAX; let mut u_max = f64::MIN;
                                                let mut v_min = f64::MAX; let mut v_max = f64::MIN;
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

                                                let mf = margin_f as f64;
                                                let ds = draw_size as f64;
                                                let map_u = |u: f64| -> f32 { (mf + (u - u_min) / (u_max - u_min) * ds) as f32 };
                                                let map_v = |v: f64| -> f32 { (mf + (1.0 - (v - v_min) / (v_max - v_min)) * ds) as f32 };

                                                // Grid lines
                                                for i in 0..=uv_grid_u.min(50) {
                                                    let u = u_min + (u_max - u_min) * i as f64 / uv_grid_u.min(50) as f64;
                                                    let x = map_u(u);
                                                    ui.painter().line_segment(
                                                        [egui::pos2(x, rect.top() + margin_f), egui::pos2(x, rect.bottom() - margin_f)],
                                                        egui::Stroke::new(0.5, egui::Color32::from_rgb(51, 51, 68)),
                                                    );
                                                }
                                                for j in 0..=uv_grid_v.min(50) {
                                                    let v = v_min + (v_max - v_min) * j as f64 / uv_grid_v.min(50) as f64;
                                                    let y = map_v(v);
                                                    ui.painter().line_segment(
                                                        [egui::pos2(rect.left() + margin_f, y), egui::pos2(rect.right() - margin_f, y)],
                                                        egui::Stroke::new(0.5, egui::Color32::from_rgb(51, 51, 68)),
                                                    );
                                                }

                                                // Outer boundary
                                                for polyline in &face.outer_uv_boundary {
                                                    if polyline.len() < 2 { continue; }
                                                    let points: Vec<egui::Pos2> = polyline.iter()
                                                        .map(|pt| egui::pos2(map_u(pt.u), map_v(pt.v)))
                                                        .collect();
                                                    ui.painter().line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(0, 255, 136)));
                                                }
                                                // Inner boundaries (holes)
                                                for boundary in &face.inner_uv_boundaries {
                                                    for polyline in boundary {
                                                        if polyline.len() < 2 { continue; }
                                                        let points: Vec<egui::Pos2> = polyline.iter()
                                                            .map(|pt| egui::pos2(map_u(pt.u), map_v(pt.v)))
                                                            .collect();
                                                        ui.painter().line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 68, 68)));
                                                    }
                                                }

                                                // UV triangles
                                                let outer_uv_poly: Vec<(f64, f64)> = face.outer_uv_boundary.iter()
                                                    .flat_map(|pl| pl.iter().map(|pt| (pt.u, pt.v))).collect();
                                                if !face.uv_triangles.is_empty() {
                                                    let hole_polys: Vec<Vec<(f64, f64)>> = face.inner_uv_boundaries.iter()
                                                        .flat_map(|boundaries| boundaries.iter().map(|poly| {
                                                            poly.iter().map(|pt| (pt.u, pt.v)).collect()
                                                        })).collect();
                                                    let tri_limit = 1500.min(face.uv_triangles.len());
                                                    for (ti, tri) in face.uv_triangles.iter().enumerate() {
                                                        let cu = (tri[0].u + tri[1].u + tri[2].u) / 3.0;
                                                        let cv = (tri[0].v + tri[1].v + tri[2].v) / 3.0;
                                                        let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h));
                                                        let in_outer = !outer_uv_poly.is_empty() && point_in_polygon(cu, cv, &outer_uv_poly);
                                                        let p0 = egui::pos2(map_u(tri[0].u), map_v(tri[0].v));
                                                        let p1 = egui::pos2(map_u(tri[1].u), map_v(tri[1].v));
                                                        let p2 = egui::pos2(map_u(tri[2].u), map_v(tri[2].v));
                                                        if in_hole || !in_outer {
                                                            let points = vec![p0, p1, p2];
                                                            ui.painter().add(egui::Shape::convex_polygon(points,
                                                                egui::Color32::from_rgba_premultiplied(255, 34, 34, 50),
                                                                egui::Stroke::new(0.5, egui::Color32::from_rgba_premultiplied(255, 68, 68, 120)),
                                                            ));
                                                        } else {
                                                            let points = vec![p0, p1, p2];
                                                            let fill = if ti % 2 == 0 {
                                                                egui::Color32::from_rgba_premultiplied(68, 136, 255, 20)
                                                            } else {
                                                                egui::Color32::from_rgba_premultiplied(85, 170, 255, 20)
                                                            };
                                                            let stroke = if ti % 2 == 0 {
                                                                egui::Stroke::new(0.4, egui::Color32::from_rgba_premultiplied(68, 136, 255, 160))
                                                            } else {
                                                                egui::Stroke::new(0.4, egui::Color32::from_rgba_premultiplied(85, 170, 255, 160))
                                                            };
                                                            ui.painter().add(egui::Shape::convex_polygon(points, fill, stroke));
                                                        }
                                                        if ti >= tri_limit { break; }
                                                    }
                                                }
                                            }

                                            // SVG export
                                            #[cfg(target_arch = "wasm32")]
                                            {
                                                if ui.button("Download UV as SVG").clicked() {
                                                    pending_svg_export = true;
                                                }
                                            }
                                            #[cfg(not(target_arch = "wasm32"))]
                                            {
                                                if ui.button("Save UV as SVG...").clicked() {
                                                    pending_svg_export = true;
                                                }
                                            }
                                        }
                                    }
                                } else {
                                    ui.label(egui::RichText::new("Select a face").size(11.0).color(egui::Color32::GRAY));
                                }
                            } else {
                                ui.label(egui::RichText::new("Select an instance first").size(11.0).color(egui::Color32::GRAY));
                            }
                        }
                        ui.separator();

                        // Face Info
                        ui.heading(egui::RichText::new("Face Info").size(13.0));
                        if let Some(inst_idx) = selected_instance {
                            if let Some((_, fid)) = selected_face {
                                if let Some(inst) = detailed_instances_clone.get(inst_idx) {
                                    if let Some(face) = inst.faces.iter().find(|f| f.face_id == fid) {
                                        ui.label(egui::RichText::new(format!("ID: {} | STEP: #{}", face.face_id, face.step_face_id)).size(11.0));
                                        ui.label(egui::RichText::new(format!("Surface: {}", face.surface_type)).size(11.0));
                                        ui.label(egui::RichText::new(format!("Triangles: [{}, {})", face.triangle_range.0, face.triangle_range.1)).size(11.0));
                                        ui.label(egui::RichText::new(format!("Boundary: {} loops, Holes: {}", face.outer_boundary.len(), face.inner_boundaries.len())).size(11.0));
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

            // Apply pending actions from mobile structure panel
            if let Some(idx) = pending_visibility_toggle {
                if self.hidden_instances.contains(&idx) {
                    self.hidden_instances.remove(&idx);
                } else {
                    self.hidden_instances.insert(idx);
                    if self.selected_instance == Some(idx) {
                        self.selected_instance = None;
                        self.selected_face = None;
                        self.highlighted_face = None;
                    }
                }
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
            if let Some(idx) = pending_instance_select {
                self.selected_instance = Some(idx);
                self.selected_face = None;
                self.highlighted_face = None;
                self.highlight_dirty = true;
                self.uv_svg_cache = None;
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
                if let Some(ref tree) = self.assembly_tree {
                    let (path, target) = find_instance_path(tree, inst_idx);
                    self.open_tree_nodes = path.into_iter().collect();
                    self.scroll_to_tree_node = target;
                }
            }
            if pending_svg_export {
                if let Some((_, ref svg_content)) = self.uv_svg_cache {
                    #[cfg(target_arch = "wasm32")]
                    {
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
                    }
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
                }
            }
            if let Some(fid) = pending_copy_face_id {
                ctx.copy_text(format!("{}", fid));
            }
            self.scroll_to_tree_node = None;
            self.scroll_to_face_id = None;
            self.open_tree_nodes.clear();
        }

        // ─── Mobile overlay: Log panel ─────────────────────────────────
        if self.mobile_log_open {
            let panel_height = (screen.height() * 0.4).min(250.0);
            egui::Window::new("Log")
                .id(egui::Id::new("mobile_log_window"))
                .fixed_pos(egui::Pos2::new(screen.min.x, screen.bottom() - panel_height - 50.0))
                .fixed_size(egui::vec2(screen.width(), panel_height))
                .resizable(false)
                .collapsible(false)
                .default_open(true)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        // Close button — large and easy to tap on mobile
                        if ui.button("✕").clicked() {
                            self.mobile_log_open = false;
                        }
                        ui.heading(egui::RichText::new("Log").size(12.0));
                        if self.warning_count > 0 {
                            ui.label(egui::RichText::new(format!("W:{}", self.warning_count)).size(10.0).color(egui::Color32::from_rgb(255, 200, 50)));
                        }
                        if self.error_count > 0 {
                            ui.label(egui::RichText::new(format!("E:{}", self.error_count)).size(10.0).color(egui::Color32::from_rgb(255, 80, 80)));
                        }
                        if ui.button("Clear").clicked() { self.log.clear(); }
                        ui.checkbox(&mut self.log_auto_scroll, "Auto");
                    });
                    egui::ScrollArea::vertical()
                        .stick_to_bottom(self.log_auto_scroll)
                        .show(ui, |ui| {
                            for entry in &self.log {
                                let msg_color = match entry.severity {
                                    LogSeverity::Info => egui::Color32::from_rgb(200, 200, 210),
                                    LogSeverity::Warning => egui::Color32::from_rgb(255, 200, 50),
                                    LogSeverity::Error => egui::Color32::from_rgb(255, 80, 80),
                                };
                                ui.horizontal(|ui| {
                                    ui.label(egui::RichText::new(format!("[{}]", entry.time)).size(9.0).color(egui::Color32::from_rgb(120, 120, 140)));
                                    ui.add(egui::Label::new(
                                        egui::RichText::new(&entry.message).size(9.0).color(msg_color)
                                    ).wrap());
                                });
                            }
                        });
                });
        }
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

    // Draw UV triangles (the actual triangulation result)
    if !face.uv_triangles.is_empty() {
        // Build hole polygons for classification
        let hole_polys: Vec<Vec<(f64, f64)>> = face.inner_uv_boundaries.iter()
            .flat_map(|boundaries| boundaries.iter().map(|poly| {
                poly.iter().map(|pt| (pt.u, pt.v)).collect()
            }))
            .collect();

        let n_triangles = face.uv_triangles.len();
        let svg_limit = 3000.min(n_triangles);

        for (ti, tri) in face.uv_triangles.iter().enumerate() {
            let x0 = map_u(tri[0].u);
            let y0 = map_v(tri[0].v);
            let x1 = map_u(tri[1].u);
            let y1 = map_v(tri[1].v);
            let x2 = map_u(tri[2].u);
            let y2 = map_v(tri[2].v);

            // Classify: check if centroid is inside any hole
            let cu = (tri[0].u + tri[1].u + tri[2].u) / 3.0;
            let cv = (tri[0].v + tri[1].v + tri[2].v) / 3.0;
            let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h));
            let in_outer = !outer_uv_poly.is_empty() && point_in_polygon(cu, cv, &outer_uv_poly);

            if in_hole || !in_outer {
                // Triangle inside a hole or outside boundary — red fill
                svg.push_str(&format!(
                    "  <polygon points=\"{:.2},{:.2} {:.2},{:.2} {:.2},{:.2}\" fill=\"#ff222244\" stroke=\"#ff4444\" stroke-width=\"0.5\"/>\n",
                    x0, y0, x1, y1, x2, y2
                ));
            } else {
                // Valid triangle — use alternating colors for visibility
                let fill_color = if ti % 2 == 0 { "#4488ff22" } else { "#55aaff22" };
                let stroke_color = if ti % 2 == 0 { "#4488ff" } else { "#55aaff" };
                svg.push_str(&format!(
                    "  <polygon points=\"{:.2},{:.2} {:.2},{:.2} {:.2},{:.2}\" fill=\"{}\" stroke=\"{}\" stroke-width=\"0.5\"/>\n",
                    x0, y0, x1, y1, x2, y2, fill_color, stroke_color
                ));
            }
            if ti >= svg_limit { break; }
        }

        // Add triangle count info
        svg.push_str(&format!(
            "  <text x=\"{}\" y=\"{}\" fill=\"#888\" font-size=\"11\" text-anchor=\"end\">Triangles: {}/{} (valid/in UV)</text>\n",
            margin + draw_w, svg_height - 20.0,
            face.uv_triangles.iter().take(svg_limit).filter(|tri| {
                let cu = (tri[0].u + tri[1].u + tri[2].u) / 3.0;
                let cv = (tri[0].v + tri[1].v + tri[2].v) / 3.0;
                let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h));
                let in_outer = !outer_uv_poly.is_empty() && point_in_polygon(cu, cv, &outer_uv_poly);
                !in_hole && in_outer
            }).count(),
            n_triangles
        ));
    }

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
    hidden_instances: &std::collections::HashSet<usize>,
    pending_instance_select: &mut Option<usize>,
    pending_visibility_toggle: &mut Option<usize>,
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
                draw_assembly_node_static(ui, child, selected_instance, hidden_instances, pending_instance_select, pending_visibility_toggle, open_tree_nodes, scroll_to_tree_node);
            }
        });
    } else {
        // Leaf node: draw visibility checkbox + selectable label
        ui.horizontal(|ui| {
            // Visibility checkbox (eye icon equivalent)
            if let Some(idx) = node.instance_index {
                let is_visible = !hidden_instances.contains(&idx);
                let checkbox_color = if is_visible {
                    egui::Color32::from_rgb(80, 180, 80)
                } else {
                    egui::Color32::from_rgb(180, 80, 80)
                };
                let eye_text = if is_visible { "👁" } else { "  " };
                if ui.add(egui::Label::new(egui::RichText::new(eye_text).size(11.0).color(checkbox_color)).sense(egui::Sense::click())).clicked() {
                    *pending_visibility_toggle = Some(idx);
                }
            }
            // Selectable label for the instance
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
        });
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
