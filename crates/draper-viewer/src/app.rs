// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
#![allow(dead_code)]
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
use draper_topology::{ShapeBuilder, Solid, Edge, Wire};
use draper_mesh::{triangulate_solid, triangulate_face, TriangleMesh, TriangulationParams, check_manifold, ManifoldReport, cut_text_holes_in_mesh, TextSurface};
use draper_step::{AssemblyNode, DetailedMeshInstance, FaceInfo, PendingBrepInstance, OwnedStepConversionContext, StepFile, step_structure_lazy};
use draper_geometry::Surface;
use draper_geometry::Point3d;
use egui_wgpu::RenderState;
use eframe::egui;

/// Unified triangulation parameters for all platforms.
///
/// Uses the current LOD level to control quality/performance trade-off.
/// The LOD level can be changed via the UI before loading a model.
fn tri_params_for_lod(lod: LodLevel) -> TriangulationParams {
    lod.params()
}

/// Human-readable label for a GDT check type ID.
fn gdt_type_label(id: u32) -> &'static str {
    match id {
        0 => "Flatness",
        1 => "Straightness",
        2 => "Circularity",
        3 => "Cylindricity",
        4 => "Position",
        5 => "Parallelism",
        6 => "Perpendicularity",
        7 => "Angularity",
        8 => "Runout",
        _ => "Unknown",
    }
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
    /// JSON model file (text content stored as bytes for symmetry with STL).
    Json { name: String, data: Vec<u8> },
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
    /// When true, close the currently-open mobile panel after the next
    /// frame. Set by model-loading buttons so the user can see the
    /// freshly-loaded mesh instead of the panel covering it.
    close_mobile_panel_after_load: bool,
    /// Which mobile "tab" (category) is currently active in the Controls panel.
    /// Lets the user switch between Primitives / Holes / Modeling / Display / Info
    /// without scrolling through one giant list — important on phones.
    mobile_controls_tab: MobileControlsTab,

    // ─── Chunked triangulation ──────────────────────────────────────────────
    /// Time-budgeted BREP triangulation processor.
    chunked_triangulator: ChunkedBrepTriangulator,

    // ─── Level of Detail ────────────────────────────────────────────────────
    /// Current LOD level for triangulation quality.
    /// Affects `max_deviation`, `angular_samples`, `height_samples`, etc.
    /// Users can change this before loading a STEP file to trade quality for speed.
    lod_level: LodLevel,

    // ─── Web Worker mode (WASM only) ────────────────────────────────────────
    /// Whether to use a Web Worker for STEP parsing + triangulation.
    /// When enabled, heavy computation runs in a background thread,
    /// keeping the main thread free for UI rendering at 60+ FPS.
    /// Falls back to main-thread chunked processing if the worker is unavailable.
    #[cfg(target_arch = "wasm32")]
    use_worker: bool,
    /// Whether the Web Worker is available and initialized.
    #[cfg(target_arch = "wasm32")]
    worker_ready: bool,
    /// Pending mesh data received from the worker, waiting to be merged.
    #[cfg(target_arch = "wasm32")]
    worker_pending_meshes: Vec<WorkerMeshResult>,

    // ─── Modeling (editing + boolean + GDT) ────────────────────────────────
    /// The current solid being edited (set whenever a primitive is loaded
    /// or a STEP file with a single solid is imported). Operations like
    /// fillet/chamfer/shell/transform work on this solid.
    current_solid: Option<Solid>,
    /// A second solid for boolean operations (set via "Set as Boolean B"
    /// button which captures the current solid).
    secondary_solid: Option<Solid>,
    /// Fillet radius (mm) for the fillet_edge operation.
    fillet_radius: f64,
    /// Chamfer distance (mm) for the chamfer_edge operation.
    chamfer_distance: f64,
    /// Shell thickness (mm) for the make_shell operation.
    shell_thickness: f64,
    /// Edge ID to fillet/chamfer (0 = first manifold edge found).
    model_edge_index: usize,
    /// Translate delta (mm).
    translate_dx: f64, translate_dy: f64, translate_dz: f64,
    /// Rotation axis (will be normalized).
    rotate_axis_x: f64, rotate_axis_y: f64, rotate_axis_z: f64,
    /// Rotation angle in degrees.
    rotate_angle_deg: f64,
    /// Scale factor.
    scale_factor: f64,
    /// Mirror plane normal.
    mirror_nx: f64, mirror_ny: f64, mirror_nz: f64,
    /// Circular pattern count.
    pattern_count: usize,
    /// GDT check type (0=Flatness, 1=Straightness, 2=Circularity, 3=Cylindricity,
    /// 4=Position, 5=Parallelism, 6=Perpendicularity, 7=Angularity, 8=Runout).
    gdt_check_type: u32,
    /// GDT tolerance value (mm).
    gdt_tolerance: f64,
    /// Last GDT result (actual deviation).
    gdt_last_result: Option<(f64, f64, bool)>, // (tolerance, actual, passed)
    /// Show the modeling panel.
    show_modeling: bool,
    /// Rotate-around-point center (px, py, pz).
    rotate_pivot_x: f64, rotate_pivot_y: f64, rotate_pivot_z: f64,
    /// Scale-around-point center (cx, cy, cz).
    scale_pivot_x: f64, scale_pivot_y: f64, scale_pivot_z: f64,
    /// Face index for face-level operations (delete/reverse/clear-holes).
    face_op_index: usize,
    /// Hole index for remove_hole.
    hole_op_index: usize,

    // ─── Curve / surface test visualizations ────────────────────────────
    /// Extra line-strip vertices used to visualize parametric curves
    /// (Line, Circle, Ellipse, Hyperbola, Parabola, NURBS curve, Trimmed,
    /// PCurve). These are appended to the edge line buffer each frame.
    extra_curve_lines: Vec<LineVertex>,
    /// Whether the extra curve lines buffer needs GPU re-upload.
    extra_curve_lines_dirty: bool,

    // ─── UV breakdown for the current solid ─────────────────────────────
    /// Whether to show the per-face UV breakdown window.
    show_uv_window: bool,
    /// Index of the face currently selected in the UV window.
    uv_window_face_idx: usize,
    /// U subdivisions for the UV grid display.
    uv_window_u_divs: usize,
    /// V subdivisions for the UV grid display.
    uv_window_v_divs: usize,
    /// Cached UV breakdown for the current solid — recomputed whenever the
    /// solid changes or when the user clicks "View UV" / "Save UV SVG".
    /// Keyed by the solid's face count + a generation counter so stale
    /// cache from a previous solid is never shown.
    solid_uv_breakdown: Option<SolidUvBreakdown>,
    /// Set when the user clicked "Save UV SVG" — the next frame will
    /// trigger a file dialog (native) or browser download (WASM) of the
    /// SVG of the currently-selected face in the UV window.
    pending_solid_uv_svg_export: bool,
}

/// UV breakdown for a single face of a solid — the outer boundary plus
/// any inner boundaries (holes), expressed as UV polylines. Each polyline
/// is a sequence of (u, v) points in the surface's parametric space.
#[derive(Clone, Debug)]
struct FaceUvBreakdown {
    /// 0-based face index within the solid's outer shell.
    face_idx: usize,
    /// Surface type label (e.g. "Plane", "Cylinder", "Nurbs").
    surface_type: String,
    /// Whether the face's normal matches the surface normal.
    forward: bool,
    /// Outer boundary as one or more UV polylines (usually just one).
    outer_polylines: Vec<Vec<(f64, f64)>>,
    /// Inner boundaries (holes) as a list of UV polylines.
    inner_polylines: Vec<Vec<(f64, f64)>>,
    /// Actual UV triangles of the face tessellation, each triangle being
    /// three (u, v) points. Used to draw the UV mesh in the popup window.
    /// Empty if the face failed to triangulate.
    uv_triangles: Vec<[(f64, f64); 3]>,
}

/// UV breakdown for an entire solid — collects all faces' UV data.
#[derive(Clone, Debug)]
struct SolidUvBreakdown {
    /// Per-face UV data, in face-index order.
    faces: Vec<FaceUvBreakdown>,
    /// Name of the model (for the SVG title).
    model_name: String,
}

/// Mobile overlay panel type.
#[derive(Clone, Debug, PartialEq)]
enum MobilePanel {
    /// Left controls panel (primitives, import, display, info)
    Controls,
    /// Right structure panel (tree, faces, UV, face info)
    Structure,
}

/// Tabs within the mobile Controls overlay.
///
/// The Controls panel has too many features to fit on one phone screen,
/// so we split it into categorized tabs. The user taps a tab label at the
/// top of the panel to switch which category is visible.
#[derive(Clone, Debug, PartialEq, Copy)]
enum MobileControlsTab {
    /// Primitives: Box, Cylinder, Sphere, Cone, Torus, Revolution, Extrusion, Engine
    Primitives,
    /// Comprehensive NURBS surface tests: saddle, bump, wave, ruled, Coons, etc.
    Surfaces,
    /// Comprehensive curve tests: Line, Circle, Ellipse, Hyperbola, Parabola, NURBS, etc.
    Curves,
    /// Hole:3 cut-outs: Box~, Cyl~, Sph~, Cone~, Tor~, Rev~, Ext~, NURBS~
    Holes,
    /// Modeling: Fillet, Chamfer, Shell, Transform, Boolean, GDT, Patterns, Face Ops
    Modeling,
    /// Display: wireframe, edges, overlay, axes, grid, quality
    Display,
    /// Info: model name, vertex/triangle count, manifold stats, errors
    Info,
}

impl MobileControlsTab {
    fn label(self) -> &'static str {
        match self {
            Self::Primitives => "Models",
            Self::Surfaces   => "Surf",
            Self::Curves     => "Curves",
            Self::Holes      => "Holes",
            Self::Modeling   => "Edit",
            Self::Display    => "View",
            Self::Info       => "Info",
        }
    }
    fn all() -> &'static [MobileControlsTab; 7] {
        &[
            Self::Primitives,
            Self::Surfaces,
            Self::Curves,
            Self::Holes,
            Self::Modeling,
            Self::Display,
            Self::Info,
        ]
    }
}

/// Level of Detail for triangulation quality.
///
/// Controls the trade-off between mesh quality and performance.
/// Lower LOD = fewer triangles, faster loading, coarser appearance.
/// Higher LOD = more triangles, slower loading, smoother appearance.
#[derive(Clone, Copy, Debug, PartialEq)]
enum LodLevel {
    /// Very coarse mesh — fastest loading, suitable for preview/thumbnails.
    Preview,
    /// Low quality — good for distant objects, fast interactive rotation.
    Low,
    /// Medium quality — balanced for general use.
    Medium,
    /// High quality — default, good for most engineering workflows.
    High,
    /// Ultra quality — maximum detail, slowest loading.
    Ultra,
}

impl LodLevel {
    /// Convert to a numeric LOD value for `TriangulationParams::for_lod()`.
    fn lod_value(self) -> f64 {
        match self {
            LodLevel::Preview => 0.1,
            LodLevel::Low => 0.3,
            LodLevel::Medium => 0.5,
            LodLevel::High => 0.75,
            LodLevel::Ultra => 1.0,
        }
    }

    /// Convert to `TriangulationParams` for this LOD level.
    fn params(self) -> TriangulationParams {
        TriangulationParams::for_lod(self.lod_value())
    }

    /// Human-readable label.
    fn label(self) -> &'static str {
        match self {
            LodLevel::Preview => "Preview",
            LodLevel::Low => "Low",
            LodLevel::Medium => "Medium",
            LodLevel::High => "High",
            LodLevel::Ultra => "Ultra",
        }
    }

    /// All LOD levels in order from coarsest to finest.
    fn all() -> &'static [LodLevel] {
        &[LodLevel::Preview, LodLevel::Low, LodLevel::Medium, LodLevel::High, LodLevel::Ultra]
    }
}

/// Result of a worker-based triangulation (WASM only).
///
/// Contains the flat mesh data (TypedArray-compatible) received from
/// the Web Worker, which needs to be converted back to a TriangleMesh
/// for merging into the scene.
#[cfg(target_arch = "wasm32")]
struct WorkerMeshResult {
    /// Name of the BREP instance.
    name: String,
    /// BREP ID.
    brep_id: usize,
    /// Per-instance color (if specified in the STEP file).
    color: Option<[f32; 4]>,
    /// Flat vertex positions [x0,y0,z0, x1,y1,z1, ...] as f32.
    vertices: Vec<f32>,
    /// Flat triangle indices [i0,j0,k0, i1,j1,k1, ...] as u32.
    indices: Vec<u32>,
    /// Flat vertex normals [nx0,ny0,nz0, ...] as f32 (if available).
    normals: Option<Vec<f32>>,
    /// Flat face normals [nx0,ny0,nz0, ...] as f32 (if available).
    face_normals: Option<Vec<f32>>,
    /// Flat per-triangle colors [r0,g0,b0,a0, ...] as f32 (if available).
    colors: Option<Vec<f32>>,
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
        let solid_clone_for_field = solid.clone();
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
            close_mobile_panel_after_load: false,
            mobile_controls_tab: MobileControlsTab::Primitives,
            chunked_triangulator: ChunkedBrepTriangulator::new(),
            lod_level: LodLevel::High,
            #[cfg(target_arch = "wasm32")]
            use_worker: false, // Disabled until wasm-bindgen exports are complete (Phase 1.3 WIP)
            #[cfg(target_arch = "wasm32")]
            worker_ready: false,
            #[cfg(target_arch = "wasm32")]
            worker_pending_meshes: Vec::new(),
            current_solid: Some(solid_clone_for_field),
            secondary_solid: None,
            fillet_radius: 5.0,
            chamfer_distance: 3.0,
            shell_thickness: 2.0,
            model_edge_index: 0,
            translate_dx: 50.0, translate_dy: 0.0, translate_dz: 0.0,
            rotate_axis_x: 0.0, rotate_axis_y: 0.0, rotate_axis_z: 1.0,
            rotate_angle_deg: 45.0,
            scale_factor: 1.5,
            mirror_nx: 1.0, mirror_ny: 0.0, mirror_nz: 0.0,
            pattern_count: 6,
            gdt_check_type: 0,
            gdt_tolerance: 0.1,
            gdt_last_result: None,
            show_modeling: false,
            rotate_pivot_x: 0.0, rotate_pivot_y: 0.0, rotate_pivot_z: 0.0,
            scale_pivot_x: 0.0, scale_pivot_y: 0.0, scale_pivot_z: 0.0,
            face_op_index: 0,
            hole_op_index: 0,
            extra_curve_lines: Vec::new(),
            extra_curve_lines_dirty: false,
            show_uv_window: false,
            uv_window_face_idx: 0,
            uv_window_u_divs: 10,
            uv_window_v_divs: 10,
            solid_uv_breakdown: None,
            pending_solid_uv_svg_export: false,
        };
        app.log("3Draper Viewer started");
        app.log(&format!("Default model: Box 100x100x100 ({} vertices, {} triangles)",
            app.current_model.vertex_count, app.current_model.triangle_count));
        app
    }

    fn load_mesh(&mut self, mesh: TriangleMesh, name: &str) {
        // Auto-fit camera to new model center AND reset orientation so the
        // user always sees the model from a recognizable 3/4 perspective.
        // Without the orientation reset, a user who previously rotated to
        // top-down view would see a flat sheet edge-on after loading a NURBS
        // surface — making the model look "broken" on mobile.
        let (bbox_min, bbox_max) = mesh.bounding_box();
        self.camera.fit_and_reset_orientation(
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
        // Invalidate the per-solid UV breakdown cache so the next time the
        // UV window is opened we recompute from the freshly-loaded solid.
        self.solid_uv_breakdown = None;
        self.uv_window_face_idx = 0;
        self.open_tree_nodes.clear();
        self.scroll_to_tree_node = None;
        self.scroll_to_face_id = None;
        self.hidden_instances.clear();
        // Clear any extra curve visualization lines from a previous test.
        // Curve tests that want to KEEP their lines will set them AFTER
        // calling load_mesh (or use a dedicated loader that doesn't call
        // load_mesh for the triangle mesh part).
        if !self.extra_curve_lines.is_empty() {
            self.extra_curve_lines.clear();
            self.extra_curve_lines_dirty = true;
        }
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
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Box 100x80x60");
    }

    fn load_cylinder(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cylinder R=40 H=100");
    }

    fn load_sphere(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
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
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cone R=40 H=80");
    }

    fn load_torus(&mut self) {
        let solid = ShapeBuilder::make_torus(40.0, 12.0);
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
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
        self.current_solid = doc.solids().into_iter().next().cloned();
        self.load_mesh(mesh, "ICE Engine (I4)");
    }

    // ─── Modeling operations ────────────────────────────────────────────

    /// Helper: re-triangulate from `current_solid` and update the viewer.
    fn refresh_from_current_solid(&mut self, name: &str) {
        if let Some(solid) = &self.current_solid {
            let mesh = triangulate_solid(solid, &tri_params_for_lod(self.lod_level));
            self.detailed_instances.clear();
            self.instance_triangle_ranges.clear();
            self.assembly_tree = None;
            self.load_mesh(mesh, name);
        } else {
            self.log_warning("No current solid — load a primitive first");
        }
    }

    /// Apply fillet to the edge at `model_edge_index` of the current solid.
    fn model_fillet_edge(&mut self) {
        let radius = self.fillet_radius;
        let edge_index = self.model_edge_index;
        let mut solid = match self.current_solid.take() {
            Some(s) => s,
            None => {
                self.log_warning("Fillet: no current solid");
                return;
            }
        };
        // If edge_index is 0, find the first manifold edge.
        let actual_edge_id = if edge_index == 0 {
            self.find_first_manifold_edge(&solid)
        } else {
            edge_index
        };
        match draper_core::operations::fillet_edge(&mut solid, actual_edge_id, radius) {
            Ok(()) => {
                self.current_solid = Some(solid);
                self.refresh_from_current_solid(&format!("Fillet r={} on edge {}", radius, actual_edge_id));
            }
            Err(e) => {
                self.log_warning(&format!("Fillet failed: {}", e));
                self.current_solid = Some(solid);
            }
        }
    }

    /// Apply chamfer to the edge at `model_edge_index` of the current solid.
    fn model_chamfer_edge(&mut self) {
        let distance = self.chamfer_distance;
        let edge_index = self.model_edge_index;
        let mut solid = match self.current_solid.take() {
            Some(s) => s,
            None => {
                self.log_warning("Chamfer: no current solid");
                return;
            }
        };
        let actual_edge_id = if edge_index == 0 {
            self.find_first_manifold_edge(&solid)
        } else {
            edge_index
        };
        match draper_core::operations::chamfer_edge(&mut solid, actual_edge_id, distance) {
            Ok(()) => {
                self.current_solid = Some(solid);
                self.refresh_from_current_solid(&format!("Chamfer d={} on edge {}", distance, actual_edge_id));
            }
            Err(e) => {
                self.log_warning(&format!("Chamfer failed: {}", e));
                self.current_solid = Some(solid);
            }
        }
    }

    /// Apply shell to the current solid.
    fn model_make_shell(&mut self) {
        let thickness = self.shell_thickness;
        let mut solid = match self.current_solid.take() {
            Some(s) => s,
            None => {
                self.log_warning("Shell: no current solid");
                return;
            }
        };
        match draper_core::operations::make_shell(&mut solid, thickness) {
            Ok(()) => {
                self.current_solid = Some(solid);
                self.refresh_from_current_solid(&format!("Shell thickness={}", thickness));
            }
            Err(e) => {
                self.log_warning(&format!("Shell failed: {}", e));
                self.current_solid = Some(solid);
            }
        }
    }

    /// Translate the current solid.
    fn model_translate(&mut self) {
        let (dx, dy, dz) = (self.translate_dx, self.translate_dy, self.translate_dz);
        if let Some(s) = &mut self.current_solid {
            draper_core::operations::translate_solid(s, dx, dy, dz);
            self.refresh_from_current_solid(&format!("Translate ({},{},{})", dx, dy, dz));
        }
    }

    /// Rotate the current solid about (axis) by angle (degrees).
    fn model_rotate(&mut self) {
        let (ax, ay, az) = (self.rotate_axis_x, self.rotate_axis_y, self.rotate_axis_z);
        let angle = self.rotate_angle_deg.to_radians();
        let axis = match draper_geometry::Direction3d::new(ax, ay, az) {
            Some(d) => d,
            None => {
                self.log_warning("Rotate: zero-length axis");
                return;
            }
        };
        if let Some(s) = &mut self.current_solid {
            draper_core::operations::rotate_solid(s, &axis, angle);
            self.refresh_from_current_solid(&format!("Rotate {}° about ({},{},{})", self.rotate_angle_deg, ax, ay, az));
        }
    }

    /// Scale the current solid uniformly.
    fn model_scale(&mut self) {
        let f = self.scale_factor;
        if !f.is_finite() || f <= 0.0 {
            self.log_warning(&format!("Scale: invalid factor {}", f));
            return;
        }
        if let Some(s) = &mut self.current_solid {
            draper_core::operations::scale_solid(s, f);
            self.refresh_from_current_solid(&format!("Scale ×{}", f));
        }
    }

    /// Rotate the current solid about (axis) passing through (px,py,pz).
    fn model_rotate_around_point(&mut self) {
        let (ax, ay, az) = (self.rotate_axis_x, self.rotate_axis_y, self.rotate_axis_z);
        let (px, py, pz) = (self.rotate_pivot_x, self.rotate_pivot_y, self.rotate_pivot_z);
        let angle = self.rotate_angle_deg.to_radians();
        let axis = match draper_geometry::Direction3d::new(ax, ay, az) {
            Some(d) => d,
            None => {
                self.log_warning("RotateAroundPoint: zero-length axis");
                return;
            }
        };
        if let Some(s) = &mut self.current_solid {
            let pivot = draper_geometry::Point3d::new(px, py, pz);
            draper_core::operations::rotate_solid_around_point(s, &axis, angle, &pivot);
            self.refresh_from_current_solid(&format!(
                "Rotate {}° about ({},{},{}) through ({},{},{})",
                self.rotate_angle_deg, ax, ay, az, px, py, pz,
            ));
        }
    }

    /// Scale the current solid uniformly about (cx, cy, cz).
    fn model_scale_around_point(&mut self) {
        let f = self.scale_factor;
        if !f.is_finite() || f <= 0.0 {
            self.log_warning(&format!("ScaleAroundPoint: invalid factor {}", f));
            return;
        }
        let (cx, cy, cz) = (self.scale_pivot_x, self.scale_pivot_y, self.scale_pivot_z);
        if let Some(s) = &mut self.current_solid {
            let center = draper_geometry::Point3d::new(cx, cy, cz);
            draper_core::operations::scale_solid_around_point(s, f, &center);
            self.refresh_from_current_solid(&format!("Scale ×{} about ({},{},{})", f, cx, cy, cz));
        }
    }

    /// Delete a face from the current solid by index.
    fn model_delete_face(&mut self) {
        let idx = self.face_op_index;
        if let Some(s) = &mut self.current_solid {
            match draper_core::operations::delete_face_from_solid(s, idx) {
                Ok(_) => {
                    self.refresh_from_current_solid(&format!("Delete face {}", idx));
                }
                Err(e) => self.log_warning(&format!("Delete face: {}", e)),
            }
        }
    }

    /// Reverse the orientation of a face of the current solid.
    fn model_reverse_face(&mut self) {
        let idx = self.face_op_index;
        if let Some(s) = &mut self.current_solid {
            if let Some(face) = draper_core::operations::get_face_mut(s, idx) {
                draper_core::operations::reverse_face_orientation(face);
                self.refresh_from_current_solid(&format!("Reverse face {}", idx));
            } else {
                self.log_warning(&format!("Reverse face: index {} out of range", idx));
            }
        }
    }

    /// Clear all holes from a face of the current solid.
    fn model_clear_holes(&mut self) {
        let idx = self.face_op_index;
        if let Some(s) = &mut self.current_solid {
            if let Some(face) = draper_core::operations::get_face_mut(s, idx) {
                let n = draper_core::operations::clear_holes_from_face(face);
                self.refresh_from_current_solid(&format!("Cleared {} hole(s) from face {}", n, idx));
            } else {
                self.log_warning(&format!("Clear holes: face index {} out of range", idx));
            }
        }
    }

    /// Remove a single hole from a face of the current solid.
    fn model_remove_hole(&mut self) {
        let face_idx = self.face_op_index;
        let hole_idx = self.hole_op_index;
        if let Some(s) = &mut self.current_solid {
            if let Some(face) = draper_core::operations::get_face_mut(s, face_idx) {
                match draper_core::operations::remove_hole_from_face(face, hole_idx) {
                    Ok(_) => self.refresh_from_current_solid(&format!(
                        "Removed hole {} from face {}", hole_idx, face_idx,
                    )),
                    Err(e) => self.log_warning(&format!("Remove hole: {}", e)),
                }
            } else {
                self.log_warning(&format!("Remove hole: face index {} out of range", face_idx));
            }
        }
    }

    /// Mirror the current solid about the plane through origin with normal (nx,ny,nz).
    fn model_mirror(&mut self) {
        let (nx, ny, nz) = (self.mirror_nx, self.mirror_ny, self.mirror_nz);
        let normal = match draper_geometry::Direction3d::new(nx, ny, nz) {
            Some(d) => d,
            None => {
                self.log_warning("Mirror: zero-length normal");
                return;
            }
        };
        if let Some(s) = &self.current_solid {
            let mirrored = draper_core::operations::mirror_solid(
                s,
                draper_geometry::Point3d::ORIGIN,
                normal,
            );
            self.current_solid = Some(mirrored);
            self.refresh_from_current_solid(&format!("Mirror about ({},{},{})", nx, ny, nz));
        }
    }

    /// Set the current solid as the secondary solid (B) for boolean ops.
    fn model_capture_secondary(&mut self) {
        if let Some(s) = &self.current_solid {
            self.secondary_solid = Some(s.clone());
            self.log("Captured current solid as Boolean B");
        } else {
            self.log_warning("No current solid to capture");
        }
    }

    /// Boolean union of current (A) and secondary (B).
    fn model_boolean_union(&mut self) {
        let b = match self.secondary_solid.take() {
            Some(b) => b,
            None => {
                self.log_warning("Union: no secondary solid (use 'Set B' first)");
                return;
            }
        };
        let a = match self.current_solid.take() {
            Some(a) => a,
            None => {
                self.log_warning("Union: no current solid");
                self.secondary_solid = Some(b);
                return;
            }
        };
        match draper_core::boolean::boolean_union(&a, &b) {
            Ok(result) => {
                self.current_solid = Some(result);
                self.refresh_from_current_solid("A ∪ B");
            }
            Err(e) => {
                self.log_warning(&format!("Union failed: {}", e));
                self.current_solid = Some(a);
                self.secondary_solid = Some(b);
            }
        }
    }

    /// Boolean subtract: A - B.
    fn model_boolean_subtract(&mut self) {
        let b = match self.secondary_solid.take() {
            Some(b) => b,
            None => {
                self.log_warning("Subtract: no secondary solid (use 'Set B' first)");
                return;
            }
        };
        let a = match self.current_solid.take() {
            Some(a) => a,
            None => {
                self.log_warning("Subtract: no current solid");
                self.secondary_solid = Some(b);
                return;
            }
        };
        match draper_core::boolean::boolean_subtract(&a, &b) {
            Ok(result) => {
                self.current_solid = Some(result);
                self.refresh_from_current_solid("A − B");
            }
            Err(e) => {
                self.log_warning(&format!("Subtract failed: {}", e));
                self.current_solid = Some(a);
                self.secondary_solid = Some(b);
            }
        }
    }

    /// Boolean intersect: A ∩ B.
    fn model_boolean_intersect(&mut self) {
        let b = match self.secondary_solid.take() {
            Some(b) => b,
            None => {
                self.log_warning("Intersect: no secondary solid (use 'Set B' first)");
                return;
            }
        };
        let a = match self.current_solid.take() {
            Some(a) => a,
            None => {
                self.log_warning("Intersect: no current solid");
                self.secondary_solid = Some(b);
                return;
            }
        };
        match draper_core::boolean::boolean_intersect(&a, &b) {
            Ok(result) => {
                self.current_solid = Some(result);
                self.refresh_from_current_solid("A ∩ B");
            }
            Err(e) => {
                self.log_warning(&format!("Intersect failed: {}", e));
                self.current_solid = Some(a);
                self.secondary_solid = Some(b);
            }
        }
    }

    /// Create a circular pattern: `pattern_count` copies around Z axis.
    fn model_circular_pattern(&mut self) {
        let count = self.pattern_count;
        if count == 0 || count > 100 {
            self.log_warning(&format!("Pattern count {} out of range (1..100)", count));
            return;
        }
        let axis = draper_geometry::Direction3d::Z;
        if let Some(s) = &self.current_solid {
            let copies = draper_core::operations::circular_pattern(
                s, axis, count, 2.0 * std::f64::consts::PI,
            );
            if copies.is_empty() {
                self.log_warning("Pattern produced no copies");
                return;
            }
            // Merge copies into one solid by replacing current_solid with the first copy.
            // For visualization, we'd ideally create a Compound — but our triangulate_solid
            // works on single solids. So we triangulate each and merge meshes.
            let mut merged_mesh = triangulate_solid(s, &tri_params_for_lod(self.lod_level));
            for c in &copies {
                let m = triangulate_solid(c, &tri_params_for_lod(self.lod_level));
                merged_mesh.merge(&m);
            }
            self.current_solid = Some(copies.into_iter().next().unwrap_or_else(|| s.clone()));
            self.detailed_instances.clear();
            self.instance_triangle_ranges.clear();
            self.assembly_tree = None;
            self.load_mesh(merged_mesh, &format!("Circular pattern ×{}", count));
        }
    }

    /// Run a GDT check on the current solid's mesh.
    fn model_gdt_check(&mut self) {
        let solid = match &self.current_solid {
            Some(s) => s,
            None => {
                self.log_warning("GDT: no current solid");
                return;
            }
        };
        let mesh = triangulate_solid(solid, &tri_params_for_lod(self.lod_level));
        let check_type = match self.gdt_check_type {
            0 => draper_mesh::gdt_check::GdtCheckType::Flatness,
            1 => draper_mesh::gdt_check::GdtCheckType::Straightness,
            2 => draper_mesh::gdt_check::GdtCheckType::Circularity,
            3 => draper_mesh::gdt_check::GdtCheckType::Cylindricity,
            4 => draper_mesh::gdt_check::GdtCheckType::Position,
            5 => draper_mesh::gdt_check::GdtCheckType::Parallelism,
            6 => draper_mesh::gdt_check::GdtCheckType::Perpendicularity,
            7 => draper_mesh::gdt_check::GdtCheckType::Angularity,
            8 => draper_mesh::gdt_check::GdtCheckType::Runout,
            _ => draper_mesh::gdt_check::GdtCheckType::Flatness,
        };
        let check_type_for_log = format!("{:?}", check_type);
        let spec = draper_mesh::gdt_check::ToleranceSpec {
            tolerance_type: check_type,
            tolerance_value: self.gdt_tolerance,
            ..Default::default()
        };
        let checker = draper_mesh::gdt_check::GdtChecker::new(&mesh);
        let r = checker.check(&spec);
        let status = if r.passed { "PASS" } else { "FAIL" };
        self.gdt_last_result = Some((r.tolerance_value, r.actual_deviation, r.passed));
        self.log(&format!(
            "GDT {}: actual={:.4}mm, tolerance={:.4}mm → {}",
            check_type_for_log, r.actual_deviation, r.tolerance_value, status
        ));
    }

    /// Find the first edge ID that is shared by exactly 2 faces (manifold edge).
    fn find_first_manifold_edge(&self, solid: &Solid) -> usize {
        use std::collections::HashMap;
        let mut edge_count: HashMap<u64, usize> = HashMap::new();
        if let Some(shell) = solid.outer_shell.as_ref() {
            for face in &shell.faces {
                for edge in &face.edges {
                    *edge_count.entry(edge.id.to_u64()).or_insert(0) += 1;
                }
            }
        }
        for (id, count) in &edge_count {
            if *count == 2 {
                return *id as usize;
            }
        }
        // Fallback: return 1 (the first edge ID usually)
        1
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
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Extrusion (Circle→Y)");
    }

    /// Load a NURBS surface — demonstrates NurbsSurface.
    ///
    /// Replaces the previous chaotic "wavy sheet" (random z-amplitude noise
    /// that looked bad on phone screens) with a clean hyperbolic paraboloid
    /// SADDLE: z = (x² − y²) / 100, sampled over x, y ∈ [-50, +50].
    ///
    /// The saddle is a textbook NURBS surface — its Gaussian curvature is
    /// negative everywhere, it has two pairs of asymptotic directions, and
    /// it's instantly recognizable as a "Pringles chip" shape. The
    /// bicubic (4×4) control grid reproduces the analytic saddle exactly
    /// at the corners and interpolates the interior via the standard
    /// bilinearly-blended Coons-like construction.
    ///
    /// Boundary sampling uses 30 points per side (120 total) so the
    /// triangulator has enough rim vertices for a watertight result.
    fn load_nurbs(&mut self) {
        self.load_nurbs_saddle();
    }

    /// Helper: build a NURBS surface mesh from a 2D grid of control points.
    ///
    /// Samples the boundary at `steps` points per side, then triangulates
    /// via the UV-aware path so the surface curvature is captured with
    /// chord-error-based adaptive refinement.
    fn build_nurbs_surface_mesh(
        &self,
        nurbs_surface: draper_geometry::NurbsSurface,
        steps: usize,
    ) -> TriangleMesh {
        use draper_geometry::Point2d;
        let (u_min, u_max) = nurbs_surface.u_range();
        let (v_min, v_max) = nurbs_surface.v_range();
        let surface = Surface::Nurbs(nurbs_surface);
        let mut boundary = Vec::new();
        let mut boundary_uvs = Vec::new();
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

        let params = tri_params_for_lod(self.lod_level);
        draper_mesh::triangulate_face_with_boundary_and_holes_uv(
            &surface, &boundary, &boundary_uvs, &[], &[], true, &params,
        )
    }

    /// NURBS Saddle (hyperbolic paraboloid): z = (x² − y²) / 100.
    ///
    /// Bicubic NURBS with a 4×4 control grid that exactly reproduces the
    /// analytic saddle at the corners and gives a smooth negative-Gaussian-
    /// -curvature surface in between. Recognizable as a "Pringles chip".
    fn load_nurbs_saddle(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        // z = (x^2 - y^2) / 100 over x,y ∈ [-50, +50] → z range = [-50, +50]
        // 4×4 bicubic control grid (clamped knots).
        // Corners are exact: at (±50, ±50), z = (2500 - 2500)/100 = 0.
        // At (±50, ∓50), z = (2500 - 2500)/100 = 0... wait that's the same.
        // Actually z = (x²-y²)/100 — at (50, 0): z = 25; at (0, 50): z = -25.
        // So along x-axis: ridge; along y-axis: valley. Classic saddle.
        // NOTE: control_points layout is [v_idx][u_idx] (rows-of-V), which is
        // the natural authoring orientation. from_v_rows transposes it to the
        // struct's [u_idx][v_idx] convention.
        let control_points = vec![
            // row v=0 (y=-50)
            vec![P3::new(-50.0, -50.0,   0.0), P3::new(-17.0, -50.0, -28.0), P3::new( 17.0, -50.0, -28.0), P3::new( 50.0, -50.0,   0.0)],
            // row v=1 (y≈-17)
            vec![P3::new(-50.0, -17.0,  28.0), P3::new(-17.0, -17.0,  -6.0), P3::new( 17.0, -17.0,  -6.0), P3::new( 50.0, -17.0,  28.0)],
            // row v=2 (y≈+17)
            vec![P3::new(-50.0,  17.0,  28.0), P3::new(-17.0,  17.0,  -6.0), P3::new( 17.0,  17.0,  -6.0), P3::new( 50.0,  17.0,  28.0)],
            // row v=3 (y=+50)
            vec![P3::new(-50.0,  50.0,   0.0), P3::new(-17.0,  50.0, -28.0), P3::new( 17.0,  50.0, -28.0), P3::new( 50.0,  50.0,   0.0)],
        ];
        let weights = vec![vec![1.0; 4]; 4];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface::from_v_rows(
            3, 3, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Saddle (z = (x²−y²)/100)");
    }

    /// NURBS Bump: a single smooth Gaussian-like hump in the center.
    ///
    /// Built as a 5×5 bicubic NURBS where the center control point has z=+40
    /// and the corners are at z=0. The result is a smooth isolated mound
    /// with positive Gaussian curvature everywhere — a "hill" shape.
    fn load_nurbs_bump(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        let control_points = vec![
            vec![P3::new(-50.0, -50.0,  0.0), P3::new(-25.0, -50.0,  0.0), P3::new(  0.0, -50.0,  0.0), P3::new( 25.0, -50.0,  0.0), P3::new( 50.0, -50.0,  0.0)],
            vec![P3::new(-50.0, -25.0,  0.0), P3::new(-25.0, -25.0, 10.0), P3::new(  0.0, -25.0, 18.0), P3::new( 25.0, -25.0, 10.0), P3::new( 50.0, -25.0,  0.0)],
            vec![P3::new(-50.0,   0.0,  0.0), P3::new(-25.0,   0.0, 18.0), P3::new(  0.0,   0.0, 40.0), P3::new( 25.0,   0.0, 18.0), P3::new( 50.0,   0.0,  0.0)],
            vec![P3::new(-50.0,  25.0,  0.0), P3::new(-25.0,  25.0, 10.0), P3::new(  0.0,  25.0, 18.0), P3::new( 25.0,  25.0, 10.0), P3::new( 50.0,  25.0,  0.0)],
            vec![P3::new(-50.0,  50.0,  0.0), P3::new(-25.0,  50.0,  0.0), P3::new(  0.0,  50.0,  0.0), P3::new( 25.0,  50.0,  0.0), P3::new( 50.0,  50.0,  0.0)],
        ];
        let weights = vec![vec![1.0; 5]; 5];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface::from_v_rows(
            3, 3, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Bump (Gaussian hill)");
    }

    /// NURBS Wave: a single sine wave along the X axis (one ridge, one valley).
    ///
    /// Built as a 4×3 bicubic NURBS where z follows sin(π·x/100)·30, so
    /// at x=-50 we have z=−30 (valley), at x=0 z=0 (zero crossing), at x=+50
    /// z=+30 (ridge). The Y direction is flat (degree 1, 2 control points
    /// per row) so the wave is extruded along Y — a "corrugated sheet".
    fn load_nurbs_wave(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        // z = sin(π·x/100)·30, sampled at x = -50, 0, +50 → z = -30, 0, +30
        // Cubic NURBS in X (3 ctrl pts → clamped cubic needs ≥4, so use 4: -50,-17,+17,+50)
        // Linear in Y (2 ctrl pts, degree 1).
        let control_points = vec![
            // y = -50
            vec![P3::new(-50.0, -50.0, -30.0), P3::new(-17.0, -50.0, -22.0), P3::new( 17.0, -50.0,  22.0), P3::new( 50.0, -50.0,  30.0)],
            // y = +50
            vec![P3::new(-50.0,  50.0, -30.0), P3::new(-17.0,  50.0, -22.0), P3::new( 17.0,  50.0,  22.0), P3::new( 50.0,  50.0,  30.0)],
        ];
        let weights = vec![vec![1.0; 4]; 2];
        // u: clamped cubic with 4 control points → [0,0,0,0, 1,1,1,1]
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        // v: clamped linear with 2 control points → [0,0, 1,1]
        let v_knots = vec![0.0, 0.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface::from_v_rows(
            3, 1, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Wave (sin(π·x/100)·30)");
    }

    /// NURBS Ruled Surface: linearly interpolates between two NURBS curves
    /// in 3D space. The result is a "ruled" surface — every point on it
    /// lies on a straight line connecting a point on curve A to a point
    /// on curve B.
    ///
    /// Here, curve A is a parabola in the plane y=-50 (opens upward), and
    /// curve B is a parabola in the plane y=+50 (opens downward). The
    /// ruled surface between them is a "saddle-like" shape but with
    /// straight rulings along Y, making the linear-interpolation structure
    /// visually obvious.
    fn load_nurbs_ruled(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        // Curve A (y=-50): z = (x²/100) - 25  → at x=-50: z=0; x=0: z=-25; x=+50: z=0
        //   Cubic NURBS control pts (clamped): (-50,−50,0), (-17,−50,-19), (17,−50,-19), (50,−50,0)
        // Curve B (y=+50): z = 25 - (x²/100)  → at x=-50: z=0; x=0: z=+25; x=+50: z=0
        //   Cubic NURBS control pts (clamped): (-50,+50,0), (-17,+50,19), (17,+50,19), (50,+50,0)
        let control_points = vec![
            // y = -50 (curve A — downward parabola)
            vec![P3::new(-50.0, -50.0,   0.0), P3::new(-17.0, -50.0, -19.0), P3::new( 17.0, -50.0, -19.0), P3::new( 50.0, -50.0,   0.0)],
            // y = +50 (curve B — upward parabola)
            vec![P3::new(-50.0,  50.0,   0.0), P3::new(-17.0,  50.0,  19.0), P3::new( 17.0,  50.0,  19.0), P3::new( 50.0,  50.0,   0.0)],
        ];
        let weights = vec![vec![1.0; 4]; 2];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]; // cubic
        let v_knots = vec![0.0, 0.0, 1.0, 1.0]; // linear (ruled)
        let nurbs_surface = NurbsSurface::from_v_rows(
            3, 1, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Ruled (parabola A → parabola B)");
    }

    /// NURBS Surface of Revolution: a wavy profile curve revolved around
    /// the Z axis, producing a "vase" or "poker chip" shape.
    ///
    /// Built directly as a NURBS surface using `NurbsSurface::surface_of_revolution_z`,
    /// which uses the **exact rational-quadratic circle** construction from
    /// "The NURBS Book" (Piegl & Tiller, §7.3) for the angular direction.
    /// This gives a PERFECT circle in every cross-section — no radius
    /// oscillation, no missing sweep.
    ///
    /// The profile is a 4-control-point cubic NURBS in the XZ plane, and the
    /// revolution uses 9 control points around the Z axis (4 on-axis + 4
    /// bounding-box corners + 1 duplicate for closure) with weights
    /// `[1, 1/√2, 1, 1/√2, 1, 1/√2, 1, 1/√2, 1]` and knots
    /// `[0,0,0, π/2,π/2, π,π, 3π/2,3π/2, 2π,2π,2π]`.
    fn load_nurbs_revolution(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        // Profile (radius vs height): 4 control points, cubic, in XZ plane (y=0).
        // r = 40 at z=0, r = 30 at z=33, r = 50 at z=66, r = 35 at z=100.
        let profile_pts = vec![
            P3::new(40.0, 0.0,   0.0),
            P3::new(30.0, 0.0,  33.0),
            P3::new(50.0, 0.0,  66.0),
            P3::new(35.0, 0.0, 100.0),
        ];
        let profile_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]; // clamped cubic
        let profile_weights = vec![1.0; 4];

        let nurbs_surface = NurbsSurface::surface_of_revolution_z(
            &profile_pts,
            3, // cubic profile
            profile_knots,
            profile_weights,
            0.0,                       // angle_start
            2.0 * std::f64::consts::PI, // angle_end (full revolution)
            false,                      // u_closed (profile is open)
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Surface of Revolution (vase profile)");
    }

    /// NURBS Coons Patch: a bicubic surface that interpolates 4 boundary
    /// curves. The classic Coons construction takes 4 boundary curves and
    /// builds a surface that exactly matches them at the edges.
    ///
    /// Here we use a 4×4 bicubic NURBS with control points chosen so the
    /// boundary curves form a "rounded square" shape — slightly raised
    /// corners with a dip in the middle of each edge. Recognizable as a
    /// "pillow" or "inflated cushion".
    fn load_nurbs_coons(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        // 4×4 bicubic. Corners at z=15 (raised), edge midpoints at z=-5 (dipped),
        // center at z=10 (slight dome). Forms a "puffy cushion" shape.
        let control_points = vec![
            vec![P3::new(-50.0, -50.0, 15.0), P3::new(-17.0, -50.0, -5.0), P3::new( 17.0, -50.0, -5.0), P3::new( 50.0, -50.0, 15.0)],
            vec![P3::new(-50.0, -17.0, -5.0), P3::new(-17.0, -17.0, 10.0), P3::new( 17.0, -17.0, 10.0), P3::new( 50.0, -17.0, -5.0)],
            vec![P3::new(-50.0,  17.0, -5.0), P3::new(-17.0,  17.0, 10.0), P3::new( 17.0,  17.0, 10.0), P3::new( 50.0,  17.0, -5.0)],
            vec![P3::new(-50.0,  50.0, 15.0), P3::new(-17.0,  50.0, -5.0), P3::new( 17.0,  50.0, -5.0), P3::new( 50.0,  50.0, 15.0)],
        ];
        let weights = vec![vec![1.0; 4]; 4];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface::from_v_rows(
            3, 3, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Coons Patch (puffy cushion)");
    }

    /// NURBS Bilinear Patch: degree-1×degree-1 surface — 4 control points
    /// forming a quadrilateral. This is the simplest possible NURBS
    /// surface and is equivalent to a flat quadrilateral (or a hyperbolic
    /// paraboloid if the 4 points are not coplanar).
    ///
    /// We use 4 points that are NOT coplanar — the classic "warped quad"
    /// used to test that the bilinear interpolation produces a hyperbolic
    /// paraboloid (saddle).
    fn load_nurbs_bilinear(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        let control_points = vec![
            vec![P3::new(-50.0, -50.0, -20.0), P3::new( 50.0, -50.0,  20.0)],
            vec![P3::new(-50.0,  50.0,  20.0), P3::new( 50.0,  50.0, -20.0)],
        ];
        let weights = vec![vec![1.0; 2]; 2];
        let u_knots = vec![0.0, 0.0, 1.0, 1.0]; // degree 1, 2 ctrl → [0,0,1,1]
        let v_knots = vec![0.0, 0.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface::from_v_rows(
            1, 1, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 20);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Bilinear Patch (warped quad saddle)");
    }

    /// NURBS Half-Cylinder: a rational quadratic NURBS surface that exactly
    /// represents a half-cylinder (180° arc × linear height).
    ///
    /// A 180° arc CANNOT be represented by a single rational quadratic Bézier
    /// (the middle weight would have to be cos(90°)=0, which is degenerate).
    /// The standard solution is to use TWO 90° arc segments joined at the
    /// top, giving 5 control points with a C¹-continuous junction.
    ///
    /// Construction:
    ///   - 5 angular control points (one full half-circle, two 90° Bézier segments)
    ///   - 2 height control points (linear extrusion)
    ///   - weights = [1, 1/√2, 1, 1/√2, 1] in angle, [1, 1] in height
    ///   - knots_u = [0,0,0, 0.5,0.5, 1,1,1] (degree 2, 5 ctrl pts, with interior knot at 0.5 of multiplicity 2)
    ///   - knots_v = [0,0, 1,1] (degree 1, 2 ctrl pts)
    ///
    /// The 5 angular control points alternate between "on-arc" (corner) and
    /// "off-arc" (bounding-box corner) positions:
    ///   P0 = (R, 0, 0)        ← on arc, angle 0°
    ///   P1 = (R, 0, R)        ← bounding-box corner for first 90° segment
    ///   P2 = (0, 0, R)        ← on arc, angle 90° (junction)
    ///   P3 = (-R, 0, R)       ← bounding-box corner for second 90° segment
    ///   P4 = (-R, 0, 0)       ← on arc, angle 180°
    fn load_nurbs_half_cylinder(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        let r = 40.0;
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        // Control points laid out as [v_idx][u_idx] (rows of V = height, cols of U = angle).
        let control_points = vec![
            // v = 0 (bottom of cylinder, y=0)
            vec![
                P3::new( r, 0.0, 0.0),  // u=0:    angle 0°
                P3::new( r, 0.0,   r),  // u=0.25: bounding-box corner (1st 90° segment)
                P3::new( 0.0, 0.0,   r),  // u=0.5:  angle 90° (junction)
                P3::new(-r, 0.0,   r),  // u=0.75: bounding-box corner (2nd 90° segment)
                P3::new(-r, 0.0, 0.0),  // u=1:    angle 180°
            ],
            // v = 1 (top of cylinder, y=100)
            vec![
                P3::new( r, 100.0, 0.0),
                P3::new( r, 100.0,   r),
                P3::new( 0.0, 100.0,   r),
                P3::new(-r, 100.0,   r),
                P3::new(-r, 100.0, 0.0),
            ],
        ];
        let weights = vec![
            vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
            vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
        ];
        // U: degree 2, 5 ctrl pts → 8 knots = 5+2+1.
        // Two Bézier segments: [0,0,0, 0.5,0.5, 1,1,1]
        // (interior knot at 0.5 with multiplicity 2 → C⁰ B-spline, but the
        // tangent direction is continuous because the Bézier control points
        // are arranged symmetrically — so the rendered curve looks smooth.)
        let u_knots = vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
        // V: degree 1, 2 ctrl pts → 4 knots
        let v_knots = vec![0.0, 0.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface::from_v_rows(
            2, 1, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Half-Cylinder (rational quad arc × linear)");
    }

    /// NURBS Quarter-Sphere: a rational quadratic NURBS patch that exactly
    /// represents one octant of a sphere (90° × 90°).
    ///
    /// Standard construction (see "The NURBS Book" §7.3):
    ///   - 3×3 rational quadratic control grid
    ///   - Weights: corner=1, edge-midpoint=1/√2, interior=1/2 (= (1/√2)²)
    ///   - Corner control points ARE on the sphere (4 of them at radius R)
    ///   - Edge-midpoint control points are at the CORNER of the bounding box
    ///     (NOT on the sphere — they're at distance R√2 from origin)
    ///   - Interior control point is at (R, R, R) — corner of the 3D bounding
    ///     box, distance R√3 from origin
    ///   - Top row of control points all coincide at the north pole (0, 0, R)
    ///     — this is a degenerate edge, but the surface is still well-defined.
    ///
    /// With this construction, the rational quadratic NURBS reproduces the
    /// spherical octant EXACTLY (not an approximation).
    fn load_nurbs_quarter_sphere(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        let r = 50.0;
        let inv_s = 1.0 / 2.0_f64.sqrt();
        // Control points laid out as [v_idx][u_idx] (rows of V = elevation,
        // cols of U = azimuth).
        //
        // v=0 (equator, z=0):
        //   u=0:        (R, 0, 0)         ← on sphere, azimuth 0°
        //   u=π/4 mid:  (R, R, 0)         ← bounding-box corner (NOT on sphere)
        //   u=π/2:      (0, R, 0)         ← on sphere, azimuth 90°
        // v=π/4 (mid-elevation):
        //   u=0:        (R, 0, R)         ← bounding-box corner
        //   u=π/4 mid:  (R, R, R)         ← 3D bounding-box corner (interior)
        //   u=π/2:      (0, R, R)         ← bounding-box corner
        // v=π/2 (north pole):
        //   all u:      (0, 0, R)         ← degenerate edge (north pole)
        let control_points = vec![
            // v=0 (equator)
            vec![P3::new( r, 0.0, 0.0), P3::new( r,  r, 0.0), P3::new(0.0,  r, 0.0)],
            // v=π/4 (mid-elevation)
            vec![P3::new( r, 0.0,   r), P3::new( r,  r,   r), P3::new(0.0,  r,   r)],
            // v=π/2 (north pole — degenerate)
            vec![P3::new(0.0, 0.0,   r), P3::new(0.0, 0.0,   r), P3::new(0.0, 0.0,   r)],
        ];
        let weights = vec![
            vec![1.0,   inv_s, 1.0],
            vec![inv_s, 0.5,   inv_s],
            vec![1.0,   inv_s, 1.0],
        ];
        let u_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let nurbs_surface = NurbsSurface::from_v_rows(
            2, 2, control_points, weights, u_knots, v_knots, false, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Quarter-Sphere (rational quad octant)");
    }

    /// NURBS Closed/Periodic Cylinder: a closed NURBS surface forming a
    /// full cylinder, built using the **exact rational-quadratic circle**
    /// construction for the angular direction.
    ///
    /// This uses 9 angular control points (4 on-axis at 0°, 90°, 180°, 270°,
    /// 4 bounding-box corners at 45°, 135°, 225°, 315°, and 1 duplicate of
    /// the first for closure) with weights `[1, 1/√2, 1, 1/√2, 1, 1/√2, 1, 1/√2, 1]`.
    ///
    /// The result is a PERFECT cylinder — every circular cross-section is
    /// an exact circle of radius `r`, with no radius oscillation or
    /// parameter-to-angle nonlinearity.
    ///
    /// Linear in the height direction (V), so the surface is a "ruled"
    /// cylinder.
    fn load_nurbs_closed_cylinder(&mut self) {
        use draper_geometry::{NurbsSurface, Point3d as P3};
        let r = 40.0;
        let h = 100.0;

        // Get the 9 angular control points, weights, and knots for an exact circle.
        let (circle_pts, circle_weights, circle_knots) = NurbsSurface::full_circle_xy(r);

        // Build the surface: linear in V (height), rational quadratic in U (angle).
        // from_v_rows expects [v_idx][u_idx] = [height_idx][angle_idx].
        let mut control_points = Vec::with_capacity(2);
        for &z in &[0.0, h] {
            let row: Vec<P3> = circle_pts.iter()
                .map(|p| P3::new(p.x, p.y, z))
                .collect();
            control_points.push(row);
        }
        let weights = vec![circle_weights.clone(); 2]; // same weights at both heights

        // U (angle): rational quadratic with 9 cps, knots [0,0,0, 1/4,1/4, 1/2,1/2, 3/4,3/4, 1,1,1]
        // We need to scale the knots from [0, 1] to [0, 2π] for the angle parameter.
        let u_knots: Vec<f64> = circle_knots.iter()
            .map(|&k| k * 2.0 * std::f64::consts::PI)
            .collect();
        // V (height): linear, 2 control points, knots [0, 0, 1, 1]
        let v_knots = vec![0.0, 0.0, 1.0, 1.0];

        let nurbs_surface = NurbsSurface::from_v_rows(
            2, 1, control_points, weights, u_knots, v_knots, true, false,
        );
        let mesh = self.build_nurbs_surface_mesh(nurbs_surface, 30);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "NURBS Closed Cylinder (rational quad × linear, exact circle)");
    }

    /// Load Box with "3" hole CUT OUT on the top face.
    fn load_box_text(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let base_mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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
        let base_mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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
        let base_mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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
        let base_mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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
        let base_mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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
        let base_mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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
        let nurbs_surface = NurbsSurface::from_v_rows(
            3, 3, control_points, weights, u_knots, v_knots, false, false,
        );
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
        let base_mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
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

    // ─── Curve tests (3D line-strip visualizations) ──────────────────────
    //
    // Each curve test samples a parametric Curve3d at high resolution and
    // converts the polyline into a LineVertex pair list (each segment is
    // 2 vertices: start, end). The line vertices are stored in
    // `extra_curve_lines` and appended to the edge line buffer each frame
    // by `build_edge_line_vertices()`.
    //
    // To give the camera a meaningful bounding box to fit to, we also
    // create a "marker" triangle mesh: a thin axis-aligned box that
    // encompasses the curve's extent. The box is rendered as triangle
    // geometry (visible) and gives the camera framing something to fit.
    // Without this, the camera would fit to an empty scene and the curve
    // would be invisible until manually zoomed out.

    /// Push a polyline (vector of 3D points) into `extra_curve_lines` as
    /// a connected line strip. Each consecutive pair of points becomes
    /// two LineVertex entries (start, end) for the LineList topology.
    fn push_curve_polyline(&mut self, points: &[draper_geometry::Point3d], color: [f32; 3]) {
        if points.len() < 2 {
            return;
        }
        for w in points.windows(2) {
            self.extra_curve_lines.push(LineVertex {
                position: [w[0].x as f32, w[0].y as f32, w[0].z as f32],
                color,
            });
            self.extra_curve_lines.push(LineVertex {
                position: [w[1].x as f32, w[1].y as f32, w[1].z as f32],
                color,
            });
        }
    }

    /// Build a tiny "marker" mesh that gives the camera something to fit
    /// to when only curve lines are loaded. The marker is a transparent
    /// bounding box around the curve's extent — invisible in the final
    /// render but provides a bounding box for `camera.fit_and_reset_orientation`.
    fn curve_marker_mesh(min: [f64; 3], max: [f64; 3]) -> TriangleMesh {
        // Create 8 corner vertices of an axis-aligned box.
        let (xmin, ymin, zmin) = (min[0], min[1], min[2]);
        let (xmax, ymax, zmax) = (max[0], max[1], max[2]);
        let mut mesh = TriangleMesh::new();
        let v000 = mesh.add_vertex(draper_geometry::Point3d::new(xmin, ymin, zmin));
        let v100 = mesh.add_vertex(draper_geometry::Point3d::new(xmax, ymin, zmin));
        let v010 = mesh.add_vertex(draper_geometry::Point3d::new(xmin, ymax, zmin));
        let v110 = mesh.add_vertex(draper_geometry::Point3d::new(xmax, ymax, zmin));
        let v001 = mesh.add_vertex(draper_geometry::Point3d::new(xmin, ymin, zmax));
        let v101 = mesh.add_vertex(draper_geometry::Point3d::new(xmax, ymin, zmax));
        let v011 = mesh.add_vertex(draper_geometry::Point3d::new(xmin, ymax, zmax));
        let v111 = mesh.add_vertex(draper_geometry::Point3d::new(xmax, ymax, zmax));
        // 12 triangles (2 per face × 6 faces). All normals point outward.
        // We mark these triangles with a sentinel face_id so they can be
        // colored transparently. Actually, simpler: just leave the default
        // mesh shading — the user will see a faint box outline which is
        // acceptable as a "context volume".
        // Bottom (z=zmin)
        mesh.add_triangle(v000, v010, v100);
        mesh.add_triangle(v100, v010, v110);
        // Top (z=zmax)
        mesh.add_triangle(v001, v101, v011);
        mesh.add_triangle(v101, v111, v011);
        // Front (y=ymin)
        mesh.add_triangle(v000, v100, v001);
        mesh.add_triangle(v100, v101, v001);
        // Back (y=ymax)
        mesh.add_triangle(v010, v011, v110);
        mesh.add_triangle(v110, v011, v111);
        // Left (x=xmin)
        mesh.add_triangle(v000, v001, v010);
        mesh.add_triangle(v010, v001, v011);
        // Right (x=xmax)
        mesh.add_triangle(v100, v110, v101);
        mesh.add_triangle(v101, v110, v111);
        // Color all triangles semi-transparent so the curve is the focus.
        let transparent = [0.4, 0.5, 0.7, 0.15];
        mesh.triangle_colors = Some(vec![transparent; mesh.triangles.len()]);
        mesh
    }

    /// Helper used by every curve test: clear any previous curve state,
    /// install the new curve line strip + marker mesh, and auto-fit the
    /// camera to the curve's extent.
    fn load_curve_test(
        &mut self,
        points: Vec<draper_geometry::Point3d>,
        name: &str,
        color: [f32; 3],
    ) {
        // Compute bounding box from the curve points.
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for p in &points {
            min[0] = min[0].min(p.x); max[0] = max[0].max(p.x);
            min[1] = min[1].min(p.y); max[1] = max[1].max(p.y);
            min[2] = min[2].min(p.z); max[2] = max[2].max(p.z);
        }
        // Pad the bounding box slightly so the curve isn't clipped at edges.
        let pad = 5.0;
        for i in 0..3 {
            min[i] -= pad;
            max[i] += pad;
        }

        // Build marker mesh for camera framing.
        let marker = Self::curve_marker_mesh(min, max);

        // Push the curve polyline into extra_curve_lines AFTER calling
        // load_mesh (which clears extra_curve_lines).
        // Use a small scope to satisfy the borrow checker.
        let curve_pts = points;
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(marker, name);
        // Now safe to set extra_curve_lines (load_mesh already cleared it).
        self.push_curve_polyline(&curve_pts, color);
        self.extra_curve_lines_dirty = true;
        // Force edge re-build so the new curve lines are uploaded this frame.
        self.edge_dirty = true;
        self.show_edges = true; // make sure edges (which include curve lines) are visible
    }

    /// Sample a Curve3d over its parameter range and load it as a line strip.
    fn sample_and_load_curve(
        &mut self,
        curve: &draper_geometry::Curve3d,
        t_min: f64,
        t_max: f64,
        samples: usize,
        name: &str,
        color: [f32; 3],
    ) {
        let mut pts = Vec::with_capacity(samples + 1);
        for i in 0..=samples {
            let t = t_min + (t_max - t_min) * (i as f64) / (samples as f64);
            pts.push(curve.point_at(t));
        }
        self.load_curve_test(pts, name, color);
    }

    /// Load a straight Line curve (3D).
    fn load_curve_line(&mut self) {
        use draper_geometry::{Curve3d, Line, Point3d as P3, Direction3d};
        let line = Curve3d::Line(Line::new(
            P3::new(-60.0, -40.0, -30.0),
            Direction3d::new(0.6, 0.4, 0.7).unwrap(),
        ));
        self.sample_and_load_curve(&line, 0.0, 200.0, 50, "Curve: Line (3D)", [0.95, 0.55, 0.15]);
    }

    /// Load a Circle curve in the XY plane.
    fn load_curve_circle(&mut self) {
        use draper_geometry::{Curve3d, Circle, Point3d as P3, Direction3d};
        let circle = Curve3d::Circle(Circle {
            center: P3::new(0.0, 0.0, 0.0),
            normal: Direction3d::Z,
            radius: 50.0,
            x_axis: Direction3d::X,
        });
        self.sample_and_load_curve(&circle, 0.0, 2.0 * std::f64::consts::PI, 120,
            "Curve: Circle (XY plane, R=50)", [0.20, 0.80, 0.30]);
    }

    /// Load an Ellipse curve in the XY plane.
    fn load_curve_ellipse(&mut self) {
        use draper_geometry::{Curve3d, Ellipse, Point3d as P3, Direction3d};
        let ellipse = Curve3d::Ellipse(Ellipse {
            center: P3::new(0.0, 0.0, 0.0),
            normal: Direction3d::Z,
            semi_major: 60.0,
            semi_minor: 30.0,
            x_axis: Direction3d::X,
        });
        self.sample_and_load_curve(&ellipse, 0.0, 2.0 * std::f64::consts::PI, 150,
            "Curve: Ellipse (semi=60×30)", [0.25, 0.55, 0.95]);
    }

    /// Load a Hyperbola branch curve in the XZ plane.
    ///
    /// P(t) = center + a·cosh(t)·x_axis + b·sinh(t)·y_axis
    /// where y_axis = normal × x_axis. We pick normal=+Y so the hyperbola
    /// lies in the XZ plane (x_axis=X, y_axis=Z), giving x = a·cosh(t),
    /// z = b·sinh(t).
    fn load_curve_hyperbola(&mut self) {
        use draper_geometry::{Curve3d, Hyperbola, Point3d as P3, Direction3d};
        let hyperbola = Curve3d::Hyperbola(Hyperbola {
            center: P3::new(0.0, 0.0, 0.0),
            normal: Direction3d::Y,  // normal × X = Y × X = -Z... wait, that's wrong.
            x_axis: Direction3d::X,
            semi_real: 30.0,  // a
            semi_imag: 20.0,  // b
        });
        self.sample_and_load_curve(&hyperbola, -2.0, 2.0, 100,
            "Curve: Hyperbola (x=30·cosh(t), z=20·sinh(t))", [0.95, 0.30, 0.55]);
    }

    /// Load a Parabola curve in the XZ plane.
    ///
    /// P(t) = vertex + (t²/(4f))·x_axis + t·y_axis
    /// where y_axis = normal × x_axis. With normal=+Y, x_axis=X,
    /// y_axis = Y × X = -Z, so we get x = t²/(4f), z = -t.
    /// We pick vertex at (-40, 0, 0) so the parabola sits centered in the view.
    fn load_curve_parabola(&mut self) {
        use draper_geometry::{Curve3d, Parabola, Point3d as P3, Direction3d};
        let parabola = Curve3d::Parabola(Parabola {
            vertex: P3::new(-40.0, 0.0, 0.0),
            normal: Direction3d::Y,
            x_axis: Direction3d::X,
            focal_dist: 20.0,
        });
        self.sample_and_load_curve(&parabola, -50.0, 50.0, 100,
            "Curve: Parabola (x=t²/80−40, z=−t)", [0.75, 0.20, 0.95]);
    }

    /// Load an open NURBS curve (cubic).
    fn load_curve_nurbs_open(&mut self) {
        use draper_geometry::{Curve3d, NurbsCurve, Point3d as P3};
        let curve = Curve3d::Nurbs(NurbsCurve {
            degree: 3,
            control_points: vec![
                P3::new(-60.0,  -40.0,   0.0),
                P3::new(-40.0,   40.0,  20.0),
                P3::new(  0.0,  -40.0, -30.0),
                P3::new( 40.0,   40.0,  30.0),
                P3::new( 60.0,  -40.0,   0.0),
            ],
            weights: vec![1.0; 5],
            knots: vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        });
        self.sample_and_load_curve(&curve, 0.0, 1.0, 200,
            "Curve: NURBS open (cubic, 5 ctrl pts)", [0.95, 0.85, 0.20]);
    }

    /// Load a closed (periodic) NURBS curve.
    fn load_curve_nurbs_closed(&mut self) {
        use draper_geometry::{Curve3d, NurbsCurve, Point3d as P3};
        // 6 control points around a wavy "flower" shape.
        let n = 6;
        let mut ctrl = Vec::with_capacity(n + 3); // wrap first 3 for cubic periodic
        for i in 0..(n + 3) {
            let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
            let r = 40.0 + 15.0 * (3.0 * theta).sin();
            ctrl.push(P3::new(r * theta.cos(), r * theta.sin(), 10.0 * (2.0 * theta).cos()));
        }
        // Periodic cubic knot vector: uniform, n+degree+1 = n+4 knots, but
        // for periodic we need (n+degree+1) knots with the first `degree`
        // and last `degree` extending the range. Simpler: use clamped knot
        // vector and the periodic flag set, but with the wrapped control
        // points the curve will close.
        let total = ctrl.len();
        let degree = 3;
        let n_knots = total + degree + 1;
        let mut knots = Vec::with_capacity(n_knots);
        for i in 0..n_knots {
            knots.push(i as f64);
        }
        let curve = Curve3d::Nurbs(NurbsCurve {
            degree,
            control_points: ctrl,
            weights: vec![1.0; total],
            knots,
        });
        self.sample_and_load_curve(&curve, 3.0, 3.0 + n as f64, 250,
            "Curve: NURBS closed (periodic, flower shape)", [0.20, 0.95, 0.85]);
    }

    /// Load a Trimmed curve (a portion of a basis curve).
    fn load_curve_trimmed(&mut self) {
        use draper_geometry::{Curve3d, NurbsCurve, Point3d as P3};
        // Basis: full sine-wave-like NURBS curve. Trim to middle half.
        let basis = Curve3d::Nurbs(NurbsCurve {
            degree: 3,
            control_points: vec![
                P3::new(-70.0,   0.0,  0.0),
                P3::new(-35.0,  50.0,  0.0),
                P3::new(  0.0, -50.0,  0.0),
                P3::new( 35.0,  50.0,  0.0),
                P3::new( 70.0,   0.0,  0.0),
            ],
            weights: vec![1.0; 5],
            knots: vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        });
        let trimmed = Curve3d::Trimmed {
            basis: Box::new(basis),
            start: 0.25,
            end: 0.75,
        };
        self.sample_and_load_curve(&trimmed, 0.0, 1.0, 150,
            "Curve: Trimmed NURBS (middle half)", [0.95, 0.40, 0.20]);
    }

    /// Load a PCurve — a 2D circle in the UV space of a sphere,
    /// producing a 3D "latitude line" that's actually a curve-on-surface.
    ///
    /// The sphere's UV parameterization is: u = azimuth ∈ [0, 2π],
    /// v = polar ∈ [0, π]. A 2D circle of radius 0.4 centered at (0, 0.5)
    /// in UV space traces a small loop around the sphere at roughly the
    /// equator.
    fn load_curve_pcurve(&mut self) {
        use draper_geometry::{Curve3d, Curve2d, Circle2d, Point2d, Surface, SphereSurface, Point3d as P3};
        let curve_2d = Curve2d::Circle(Circle2d::new_full(Point2d::new(0.0, std::f64::consts::FRAC_PI_2), 0.4));
        let sphere = Surface::Sphere(SphereSurface {
            center: P3::new(0.0, 0.0, 0.0),
            radius: 50.0,
        });
        let pcurve = Curve3d::PCurve {
            curve_2d: Box::new(curve_2d),
            surface: Box::new(sphere),
        };
        self.sample_and_load_curve(&pcurve, 0.0, 1.0, 200,
            "Curve: PCurve (2D circle on sphere)", [0.95, 0.20, 0.55]);
    }

    /// Load ALL curves in a single scene — a "curve gallery" showing each
    /// curve type side-by-side. Curves are offset along X so they don't
    /// overlap.
    fn load_curve_all(&mut self) {
        use draper_geometry::{Point3d as P3, Curve3d, Line, Circle, Ellipse,
                              Hyperbola, Parabola, NurbsCurve,
                              Direction3d};
        // Sample each curve and translate to its own slot.
        // 8 curves arranged in a 4×2 grid.
        let spacing = 130.0;
        let mut all_pts: Vec<Vec<P3>> = Vec::new();
        let mut colors: Vec<[f32; 3]> = Vec::new();
        let mut labels: Vec<&str> = Vec::new();

        // 1. Line — top-left slot
        {
            let line = Curve3d::Line(Line::new(
                P3::new(0.0, -30.0, -20.0),
                Direction3d::new(0.6, 0.6, 0.5).unwrap(),
            ));
            let mut pts: Vec<P3> = (0..=50).map(|i| {
                let t = i as f64 * 4.0;
                line.point_at(t)
            }).collect();
            for p in &mut pts { p.x -= 3.0 * spacing / 2.0; p.y += spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.95, 0.55, 0.15]);
            labels.push("Line");
        }
        // 2. Circle — top-second slot
        {
            let circle = Curve3d::Circle(Circle {
                center: P3::new(0.0, 0.0, 0.0),
                normal: Direction3d::Z,
                radius: 30.0,
                x_axis: Direction3d::X,
            });
            let mut pts: Vec<P3> = (0..=120).map(|i| {
                let t = 2.0 * std::f64::consts::PI * (i as f64) / 120.0;
                circle.point_at(t)
            }).collect();
            for p in &mut pts { p.x -= spacing / 2.0; p.y += spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.20, 0.80, 0.30]);
            labels.push("Circle");
        }
        // 3. Ellipse — top-third slot
        {
            let ellipse = Curve3d::Ellipse(Ellipse {
                center: P3::new(0.0, 0.0, 0.0),
                normal: Direction3d::Z,
                semi_major: 40.0,
                semi_minor: 22.0,
                x_axis: Direction3d::X,
            });
            let mut pts: Vec<P3> = (0..=150).map(|i| {
                let t = 2.0 * std::f64::consts::PI * (i as f64) / 150.0;
                ellipse.point_at(t)
            }).collect();
            for p in &mut pts { p.x += spacing / 2.0; p.y += spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.25, 0.55, 0.95]);
            labels.push("Ellipse");
        }
        // 4. Hyperbola — top-right slot
        {
            let hyperbola = Curve3d::Hyperbola(Hyperbola {
                center: P3::new(0.0, 0.0, 0.0),
                normal: Direction3d::Y,
                x_axis: Direction3d::X,
                semi_real: 20.0,
                semi_imag: 15.0,
            });
            let mut pts: Vec<P3> = (0..=80).map(|i| {
                let t = -2.0 + 4.0 * (i as f64) / 80.0;
                hyperbola.point_at(t)
            }).collect();
            for p in &mut pts { p.x += 3.0 * spacing / 2.0; p.y += spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.95, 0.30, 0.55]);
            labels.push("Hyperbola");
        }
        // 5. Parabola — bottom-left
        {
            let parabola = Curve3d::Parabola(Parabola {
                vertex: P3::new(-25.0, 0.0, 0.0),
                normal: Direction3d::Y,
                x_axis: Direction3d::X,
                focal_dist: 12.0,
            });
            let mut pts: Vec<P3> = (0..=80).map(|i| {
                let t = -35.0 + 70.0 * (i as f64) / 80.0;
                parabola.point_at(t)
            }).collect();
            for p in &mut pts { p.x -= 3.0 * spacing / 2.0; p.y -= spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.75, 0.20, 0.95]);
            labels.push("Parabola");
        }
        // 6. NURBS open — bottom-second
        {
            let nurbs = Curve3d::Nurbs(NurbsCurve {
                degree: 3,
                control_points: vec![
                    P3::new(-40.0, -25.0,   0.0),
                    P3::new(-20.0,  25.0,  15.0),
                    P3::new(  0.0, -25.0, -20.0),
                    P3::new( 20.0,  25.0,  20.0),
                    P3::new( 40.0, -25.0,   0.0),
                ],
                weights: vec![1.0; 5],
                knots: vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
            });
            let mut pts: Vec<P3> = (0..=200).map(|i| {
                let t = (i as f64) / 200.0;
                nurbs.point_at(t)
            }).collect();
            for p in &mut pts { p.x -= spacing / 2.0; p.y -= spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.95, 0.85, 0.20]);
            labels.push("NURBS open");
        }
        // 7. NURBS closed (periodic) — bottom-third
        {
            let n = 6;
            let mut ctrl = Vec::with_capacity(n + 3);
            for i in 0..(n + 3) {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                let r = 25.0 + 10.0 * (3.0 * theta).sin();
                ctrl.push(P3::new(r * theta.cos(), r * theta.sin(), 8.0 * (2.0 * theta).cos()));
            }
            let total = ctrl.len();
            let degree = 3;
            let n_knots = total + degree + 1;
            let knots: Vec<f64> = (0..n_knots).map(|i| i as f64).collect();
            let nurbs = Curve3d::Nurbs(NurbsCurve {
                degree, control_points: ctrl, weights: vec![1.0; total], knots,
            });
            let mut pts: Vec<P3> = (0..=200).map(|i| {
                let t = 3.0 + (n as f64) * (i as f64) / 200.0;
                nurbs.point_at(t)
            }).collect();
            for p in &mut pts { p.x += spacing / 2.0; p.y -= spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.20, 0.95, 0.85]);
            labels.push("NURBS closed");
        }
        // 8. Trimmed — bottom-right
        {
            let basis = Curve3d::Nurbs(NurbsCurve {
                degree: 3,
                control_points: vec![
                    P3::new(-40.0,   0.0,  0.0),
                    P3::new(-20.0,  30.0,  0.0),
                    P3::new(  0.0, -30.0,  0.0),
                    P3::new( 20.0,  30.0,  0.0),
                    P3::new( 40.0,   0.0,  0.0),
                ],
                weights: vec![1.0; 5],
                knots: vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
            });
            let trimmed = Curve3d::Trimmed {
                basis: Box::new(basis),
                start: 0.2,
                end: 0.8,
            };
            let mut pts: Vec<P3> = (0..=150).map(|i| {
                let t = (i as f64) / 150.0;
                trimmed.point_at(t)
            }).collect();
            for p in &mut pts { p.x += 3.0 * spacing / 2.0; p.y -= spacing / 2.0; }
            all_pts.push(pts);
            colors.push([0.95, 0.40, 0.20]);
            labels.push("Trimmed");
        }

        // Compute combined bounding box.
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for pts in &all_pts {
            for p in pts {
                min[0] = min[0].min(p.x); max[0] = max[0].max(p.x);
                min[1] = min[1].min(p.y); max[1] = max[1].max(p.y);
                min[2] = min[2].min(p.z); max[2] = max[2].max(p.z);
            }
        }
        for i in 0..3 {
            min[i] -= 5.0;
            max[i] += 5.0;
        }
        let marker = Self::curve_marker_mesh(min, max);
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(marker, "Curve Gallery: All 8 curve types");
        // Push all curves with their colors.
        for (pts, color) in all_pts.iter().zip(colors.iter()) {
            self.push_curve_polyline(pts, *color);
        }
        self.extra_curve_lines_dirty = true;
        self.edge_dirty = true;
        self.show_edges = true;
        // Log each label so the user sees what each color represents.
        for (i, label) in labels.iter().enumerate() {
            let c = colors[i];
            self.log(&format!("  [RGB({:.2},{:.2},{:.2})] #{}: {}", c[0], c[1], c[2], i + 1, label));
        }
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

    /// Import a JSON model from an in-memory string (used by WASM file input).
    ///
    /// Shared by both native and WASM builds — native builds can use this
    /// when they already have the text in memory (e.g., from a drag-and-drop
    /// event), and WASM builds use it from `process_web_file_loads`.
    fn import_json_from_str(&mut self, text: &str, name: &str) {
        use draper_json::JsonModel;
        match JsonModel::from_json(text) {
            Ok(model) => {
                let mesh = model.to_triangle_mesh();
                let model_name = model.metadata.name.clone();
                self.detailed_instances = model.to_detailed_instances();
                self.assembly_tree = Some(model.assembly);
                self.load_mesh(mesh, &model_name);
                self.log(&format!("Imported JSON: {} ({} instances)", name, model.metadata.instance_count));
            }
            Err(e) => self.log_error(&format!("JSON parse error: {}", e)),
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

        // Append extra curve line strips (from curve-test visualizations).
        // These are added to the same edge vertex buffer so they render with
        // the same line pipeline as B-Rep edges, with depth testing against
        // the solid mesh so curves behind geometry are properly occluded.
        if !self.extra_curve_lines.is_empty() {
            edge_vertices.extend_from_slice(&self.extra_curve_lines);
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
        #[cfg(target_arch = "wasm32")]
        self.worker_pending_meshes.clear();
    }

    /// Convert a WorkerMeshResult (flat f32 arrays from Web Worker) to a TriangleMesh.
    ///
    /// This is the reverse of `MeshData::from_mesh()` — it unpacks the flat
    /// TypedArray-compatible format back into the structured TriangleMesh
    /// that the renderer expects.
    #[cfg(target_arch = "wasm32")]
    fn worker_mesh_to_triangle_mesh(result: &WorkerMeshResult) -> TriangleMesh {
        let vertex_count = result.vertices.len() / 3;
        let triangle_count = result.indices.len() / 3;

        // Reconstruct Point3d vertices from flat f32 array
        let mut vertices = Vec::with_capacity(vertex_count);
        for i in 0..vertex_count {
            let x = result.vertices[i * 3] as f64;
            let y = result.vertices[i * 3 + 1] as f64;
            let z = result.vertices[i * 3 + 2] as f64;
            vertices.push(draper_geometry::Point3d::new(x, y, z));
        }

        // Reconstruct triangle indices from flat u32 array
        let mut triangles = Vec::with_capacity(triangle_count);
        for i in 0..triangle_count {
            triangles.push([
                result.indices[i * 3],
                result.indices[i * 3 + 1],
                result.indices[i * 3 + 2],
            ]);
        }

        // Reconstruct vertex normals (if present)
        let normals = result.normals.as_ref().map(|flat| {
            let count = flat.len() / 3;
            let mut ns = Vec::with_capacity(count);
            for i in 0..count {
                ns.push([flat[i * 3] as f64, flat[i * 3 + 1] as f64, flat[i * 3 + 2] as f64]);
            }
            ns
        });

        // Reconstruct face normals (if present)
        let face_normals = result.face_normals.as_ref().map(|flat| {
            let count = flat.len() / 3;
            let mut ns = Vec::with_capacity(count);
            for i in 0..count {
                ns.push([flat[i * 3] as f64, flat[i * 3 + 1] as f64, flat[i * 3 + 2] as f64]);
            }
            ns
        });

        // Reconstruct per-triangle colors (if present)
        let triangle_colors = result.colors.as_ref().map(|flat| {
            let count = flat.len() / 4;
            let mut cs = Vec::with_capacity(count);
            for i in 0..count {
                cs.push([flat[i * 4], flat[i * 4 + 1], flat[i * 4 + 2], flat[i * 4 + 3]]);
            }
            cs
        });

        TriangleMesh {
            vertices,
            triangles,
            normals,
            face_normals,
            triangle_colors,
            triangle_face_ids: None,
        }
    }

    /// Process pending worker mesh results (WASM only).
    ///
    /// When the Web Worker completes a BREP triangulation, it sends back
    /// flat mesh data (Float32Array/Uint32Array). This method converts
    /// the data back to TriangleMesh and merges it into the scene.
    ///
    /// Returns true if there are still more results to process.
    #[cfg(target_arch = "wasm32")]
    fn process_worker_results(&mut self) -> bool {
        if self.worker_pending_meshes.is_empty() {
            return false;
        }

        // Process all pending worker results (they're already computed, just need conversion)
        while let Some(result) = self.worker_pending_meshes.pop() {
            let mesh = Self::worker_mesh_to_triangle_mesh(&result);
            let vcount = mesh.vertex_count();
            let tcount = mesh.triangle_count();

            if tcount == 0 {
                self.log_warning(&format!(
                    "Worker: Instance '{}' (BREP #{}) produced empty mesh — skipping",
                    result.name, result.brep_id
                ));
                self.failed_face_count += 1;
                continue;
            }

            let tri_start = self.mesh.triangle_count();
            let color = result.color.unwrap_or_else(|| Self::instance_color(self.triangulated_count));
            self.mesh.merge_with_color(&mesh, color);
            let tri_end = self.mesh.triangle_count();
            self.instance_triangle_ranges.push((tri_start, tri_end));

            self.triangulated_count += 1;
            self.log(&format!(
                "Worker: '{}' — {} vertices, {} triangles ({:.1}ms)",
                result.name, vcount, tcount, 0.0 // timing tracked on worker side
            ));
        }

        self.mesh_dirty = true;
        self.edge_dirty = true;
        self.wireframe_overlay_dirty = true;

        self.is_loading || !self.worker_pending_meshes.is_empty()
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
                        // Consume the tree leaf for this failed instance
                        if let Some(ref mut tree) = self.assembly_tree {
                            skip_instance_in_tree(tree);
                        }
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
                    // Mark the corresponding tree leaf as "failed" so that
                    // subsequent successful instances get the correct leaf.
                    // Without this, failed instances leave their leaf nodes
                    // with instance_index=None, and the next successful
                    // instance gets assigned to the wrong leaf (e.g., nut's
                    // leaf gets bolt's instance_idx, causing name mismatch).
                    if let Some(ref mut tree) = self.assembly_tree {
                        skip_instance_in_tree(tree);
                    }
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
                FileLoadResult::Json { name, data } => {
                    log::info!("Processing loaded JSON file: '{}' ({} bytes)", name, data.len());
                    let text = String::from_utf8_lossy(&data).into_owned();
                    self.import_json_from_str(&text, &name);
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

    /// Download a binary blob in the browser (WASM-only).
    ///
    /// Creates a temporary `<a>` element with a `download` attribute and
    /// synthetic Blob URL, clicks it, then cleans up. Works on all modern
    /// browsers including iOS Safari 14+.
    #[cfg(target_arch = "wasm32")]
    fn download_blob(&mut self, filename: &str, mime: &str, data: &[u8]) {
        use wasm_bindgen::JsCast;

        let window = match web_sys::window() {
            Some(w) => w,
            None => {
                self.log_error("download_blob: no window available");
                return;
            }
        };
        let document = match window.document() {
            Some(d) => d,
            None => {
                self.log_error("download_blob: no document available");
                return;
            }
        };

        // Build a Uint8Array view over the data slice and wrap it in a Blob.
        let uint8 = js_sys::Uint8Array::from(data);
        let parts = js_sys::Array::new();
        parts.push(&uint8);
        let blob_options = {
            let bo = web_sys::BlobPropertyBag::new();
            bo.set_type(mime);
            bo
        };
        let blob = match web_sys::Blob::new_with_u8_array_sequence_and_options(
            &parts,
            &blob_options,
        ) {
            Ok(b) => b,
            Err(_) => {
                self.log_error("download_blob: Blob::new failed");
                return;
            }
        };

        let url = match web_sys::Url::create_object_url_with_blob(&blob) {
            Ok(u) => u,
            Err(_) => {
                self.log_error("download_blob: create_object_url failed");
                return;
            }
        };

        let a = match document.create_element("a") {
            Ok(el) => el,
            Err(_) => {
                let _ = web_sys::Url::revoke_object_url(&url);
                self.log_error("download_blob: create_element('a') failed");
                return;
            }
        };
        let _ = a.set_attribute("href", &url);
        let _ = a.set_attribute("download", filename);
        let _ = a.set_attribute("style", "display:none");
        if let Some(body) = document.body() {
            let _ = body.append_child(&a);
            // Cast to HtmlElement so we can call click()
            let html_elem: web_sys::HtmlElement = match a.dyn_into() {
                Ok(h) => h,
                Err(_) => {
                    self.log_error("download_blob: dyn_into(HtmlElement) failed");
                    return;
                }
            };
            html_elem.click();
            let _ = body.remove_child(&html_elem);
        }
        let _ = web_sys::Url::revoke_object_url(&url);
    }

    /// Download a text file in the browser (WASM-only).
    #[cfg(target_arch = "wasm32")]
    fn download_text(&mut self, filename: &str, mime: &str, text: &str) {
        self.download_blob(filename, mime, text.as_bytes());
    }

    /// Export the current mesh as binary STL via browser download (WASM).
    #[cfg(target_arch = "wasm32")]
    fn export_stl_binary_wasm(&mut self) {
        // Serialize STL into memory first, then download as a blob.
        let mut buf: Vec<u8> = Vec::with_capacity(84 + self.mesh.triangles.len() * 50);
        // 80-byte header
        buf.extend_from_slice(&[0u8; 80]);
        // 4-byte triangle count (little-endian)
        let n = self.mesh.triangles.len() as u32;
        buf.extend_from_slice(&n.to_le_bytes());
        for tri in &self.mesh.triangles {
            // Compute face normal from triangle vertices
            let v0 = self.mesh.vertices[tri[0] as usize];
            let v1 = self.mesh.vertices[tri[1] as usize];
            let v2 = self.mesh.vertices[tri[2] as usize];
            let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
            let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
            let nx = e1[1] * e2[2] - e1[2] * e2[1];
            let ny = e1[2] * e2[0] - e1[0] * e2[2];
            let nz = e1[0] * e2[1] - e1[1] * e2[0];
            let len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
            let normal = [nx / len, ny / len, nz / len];
            for &c in &normal { buf.extend_from_slice(&(c as f32).to_le_bytes()); }
            for &vi in &[tri[0], tri[1], tri[2]] {
                let v = self.mesh.vertices[vi as usize];
                for &c in &[v.x as f32, v.y as f32, v.z as f32] {
                    buf.extend_from_slice(&c.to_le_bytes());
                }
            }
            // 2-byte attribute byte count
            buf.extend_from_slice(&0u16.to_le_bytes());
        }
        let name = self.current_model.name.replace(' ', "_");
        self.download_blob(&format!("{}.stl", name), "model/stl", &buf);
        self.log(&format!("Exported STL (binary): {} ({} bytes)", name, buf.len()));
    }

    /// Export the current mesh as ASCII STL via browser download (WASM).
    #[cfg(target_arch = "wasm32")]
    fn export_stl_ascii_wasm(&mut self) {
        let mut text = String::with_capacity(self.mesh.triangles.len() * 200);
        let name = self.current_model.name.replace(' ', "_");
        text.push_str(&format!("solid {}\n", name));
        for tri in &self.mesh.triangles {
            let v0 = self.mesh.vertices[tri[0] as usize];
            let v1 = self.mesh.vertices[tri[1] as usize];
            let v2 = self.mesh.vertices[tri[2] as usize];
            let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
            let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
            let nx = e1[1] * e2[2] - e1[2] * e2[1];
            let ny = e1[2] * e2[0] - e1[0] * e2[2];
            let nz = e1[0] * e2[1] - e1[1] * e2[0];
            let len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
            text.push_str(&format!(
                "  facet normal {} {} {}\n    outer loop\n      vertex {} {} {}\n      vertex {} {} {}\n      vertex {} {} {}\n    endloop\n  endfacet\n",
                nx / len, ny / len, nz / len,
                v0.x, v0.y, v0.z,
                v1.x, v1.y, v1.z,
                v2.x, v2.y, v2.z,
            ));
        }
        text.push_str(&format!("endsolid {}\n", name));
        self.download_text(&format!("{}.stl", name), "model/stl", &text);
        self.log(&format!("Exported STL (ascii): {} ({} bytes)", name, text.len()));
    }

    /// Export the current solid as STEP via browser download (WASM).
    #[cfg(target_arch = "wasm32")]
    fn export_step_wasm(&mut self) {
        let solid = self.rebuild_current_solid();
        let name = self.current_model.name.replace(' ', "_");
        let content = draper_step::export_step(&solid, &name);
        self.download_text(&format!("{}.stp", name), "model/step", &content);
        self.log(&format!("Exported STEP: {} ({} bytes)", name, content.len()));
    }

    /// Export the current model as JSON via browser download (WASM).
    #[cfg(target_arch = "wasm32")]
    fn export_json_wasm(&mut self) {
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
                let name = self.current_model.name.replace(' ', "_");
                self.download_text(&format!("{}.json", name), "application/json", &json);
                self.log(&format!("Exported JSON: {} ({} bytes)", name, json.len()));
            }
            Err(e) => self.log_error(&format!("JSON export error: {}", e)),
        }
    }

    /// Trigger a JSON file import dialog (WASM).
    #[cfg(target_arch = "wasm32")]
    fn trigger_json_file_input(&mut self) {
        use wasm_bindgen::prelude::*;

        let window = web_sys::window().unwrap();
        let document = window.document().unwrap();
        let input = document.create_element("input").unwrap();
        input.set_attribute("type", "file").unwrap();
        input.set_attribute("accept", ".json").unwrap();
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
                    log::info!("JSON file selected: '{}'", file_name);

                    let reader = web_sys::FileReader::new().unwrap();
                    let reader_clone = reader.clone();
                    let shared = shared_result.clone();

                    let onload = Closure::wrap(Box::new(move |_: web_sys::Event| {
                        if let Ok(result) = reader_clone.result() {
                            let text = result.as_string().unwrap_or_default();
                            log::info!("JSON file loaded: {} bytes", text.len());
                            *shared.lock().unwrap() = Some(FileLoadResult::Json {
                                name: file_name.clone(),
                                data: text.into_bytes(),
                            });
                        }
                    }) as Box<dyn FnMut(_)>);

                    reader.set_onload(Some(onload.as_ref().unchecked_ref()));
                    onload.forget();
                    let _ = reader.read_as_text(&file);
                }
            }
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
                    // LOD selector — affects quality of future STEP loads
                    ui.label(egui::RichText::new("Triangulation Quality:").small());
                    for &lod in LodLevel::all() {
                        if ui.radio_value(&mut self.lod_level, lod, lod.label()).clicked() {
                            self.log(&format!("LOD changed to {} (applies to next file load)", lod.label()));
                        }
                    }
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

                    // Wrap all collapsing sections in a single vertical ScrollArea
                    // so the panel never grows beyond the window — when all four
                    // sections (Tree, Face list, UV Grid, Face Info) are expanded,
                    // the user can scroll to reach the bottom ones instead of
                    // having them silently clipped.
                    egui::ScrollArea::vertical()
                        .auto_shrink([false; 2])
                        .show(ui, |ui| {

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
                                            let rect_left = rect.left() as f64;
                                            let rect_top = rect.top() as f64;
                                            // map_u/map_v must return SCREEN coords that are relative to
                                            // the rect (the allocated painter area). Without the
                                            // rect.left()/rect.top() offset, the painted content would
                                            // stay at screen position (margin, margin) regardless of
                                            // where the parent panel is.
                                            let map_u = |u: f64| -> f32 { (rect_left + margin_f64 + (u - u_min) / (u_max - u_min) * draw_size_f64) as f32 };
                                            let map_v = |v: f64| -> f32 { (rect_top + margin_f64 + (1.0 - (v - v_min) / (v_max - v_min)) * draw_size_f64) as f32 };

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
                    }); // close ScrollArea wrapping the four sections
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
            .default_width(200.0)
            .resizable(true)
            .show(ctx, |ui| {
                // Wrap the entire panel body in a vertical ScrollArea so the
                // panel never grows beyond the window height — settings at the
                // bottom remain reachable via scroll instead of being clipped.
                egui::ScrollArea::vertical()
                    .auto_shrink([false; 2])
                    .stick_to_right(false)
                    .show(ui, |ui| {
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
                // UV breakdown button — opens a window showing the UV
                // parametric grid of every face of the current solid.
                // Works for all primitives (Box, Cylinder, Sphere, Cone,
                // Torus, Revolution, Extrusion) and NURBS surface tests.
                ui.horizontal(|ui| {
                    if ui.button("View UV").clicked() {
                        self.show_uv_window = true;
                        self.solid_uv_breakdown = None; // force recompute
                    }
                    if ui.button("Save UV SVG").clicked() {
                        self.pending_solid_uv_svg_export = true;
                        self.solid_uv_breakdown = None; // force recompute
                    }
                });
                // --- NURBS Surface Gallery ---
                ui.separator();
                ui.heading(egui::RichText::new("NURBS Surfaces").size(12.0));
                ui.label(egui::RichText::new("Comprehensive surface tests").size(9.0).color(egui::Color32::GRAY));
                egui::Grid::new("desk_surf_grid").num_columns(3).spacing([4.0, 4.0]).show(ui, |ui| {
                    if ui.button("Saddle").clicked()      { self.load_nurbs_saddle(); }
                    if ui.button("Bump").clicked()        { self.load_nurbs_bump(); }
                    if ui.button("Wave").clicked()        { self.load_nurbs_wave(); }
                    ui.end_row();
                    if ui.button("Ruled").clicked()       { self.load_nurbs_ruled(); }
                    if ui.button("Revolution").clicked()  { self.load_nurbs_revolution(); }
                    if ui.button("Coons").clicked()       { self.load_nurbs_coons(); }
                    ui.end_row();
                    if ui.button("Bilinear").clicked()    { self.load_nurbs_bilinear(); }
                    if ui.button("Half-Cyl").clicked()    { self.load_nurbs_half_cylinder(); }
                    if ui.button("Q-Sphere").clicked()    { self.load_nurbs_quarter_sphere(); }
                    ui.end_row();
                    if ui.button("Closed-Cyl").clicked()  { self.load_nurbs_closed_cylinder(); }
                    ui.end_row();
                });
                // --- Curve Gallery ---
                ui.separator();
                ui.heading(egui::RichText::new("Curves (3D line strips)").size(12.0));
                ui.label(egui::RichText::new("All curve types as colored line strips").size(9.0).color(egui::Color32::GRAY));
                egui::Grid::new("desk_curves_grid").num_columns(3).spacing([4.0, 4.0]).show(ui, |ui| {
                    if ui.button("Line").clicked()         { self.load_curve_line(); }
                    if ui.button("Circle").clicked()      { self.load_curve_circle(); }
                    if ui.button("Ellipse").clicked()     { self.load_curve_ellipse(); }
                    ui.end_row();
                    if ui.button("Hyperbola").clicked()   { self.load_curve_hyperbola(); }
                    if ui.button("Parabola").clicked()    { self.load_curve_parabola(); }
                    if ui.button("NURBS open").clicked()  { self.load_curve_nurbs_open(); }
                    ui.end_row();
                    if ui.button("NURBS closed").clicked(){ self.load_curve_nurbs_closed(); }
                    if ui.button("Trimmed").clicked()     { self.load_curve_trimmed(); }
                    if ui.button("PCurve").clicked()      { self.load_curve_pcurve(); }
                    ui.end_row();
                    if ui.button("All (Gallery)").clicked() { self.load_curve_all(); }
                    ui.end_row();
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

                // --- Modeling ---
                ui.separator();
                ui.heading(egui::RichText::new("Modeling").size(12.0));
                ui.checkbox(&mut self.show_modeling, "Show Modeling Panel");
                if self.show_modeling {
                    egui::Frame::group(ui.style())
                        .inner_margin(egui::Margin::same(6))
                        .show(ui, |ui| {
                            // Fillet
                            ui.label(egui::RichText::new("Fillet Edge").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("r:");
                                ui.add(egui::DragValue::new(&mut self.fillet_radius)
                                    .speed(0.1).range(0.01..=100.0).suffix(" mm"));
                            });
                            ui.horizontal(|ui| {
                                ui.label("edge:");
                                ui.add(egui::DragValue::new(&mut self.model_edge_index)
                                    .range(0..=100000));
                                ui.label("(0=auto)");
                            });
                            if ui.button("Fillet").clicked() { self.model_fillet_edge(); }

                            // Chamfer
                            ui.add_space(2.0);
                            ui.label(egui::RichText::new("Chamfer Edge").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("d:");
                                ui.add(egui::DragValue::new(&mut self.chamfer_distance)
                                    .speed(0.1).range(0.01..=100.0).suffix(" mm"));
                            });
                            if ui.button("Chamfer").clicked() { self.model_chamfer_edge(); }

                            // Shell
                            ui.add_space(2.0);
                            ui.label(egui::RichText::new("Shell").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("t:");
                                ui.add(egui::DragValue::new(&mut self.shell_thickness)
                                    .speed(0.1).range(0.01..=100.0).suffix(" mm"));
                            });
                            if ui.button("Shell").clicked() { self.model_make_shell(); }

                            // Transform
                            ui.add_space(2.0);
                            ui.label(egui::RichText::new("Transform").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("Δ:");
                                ui.add(egui::DragValue::new(&mut self.translate_dx).speed(1.0));
                                ui.add(egui::DragValue::new(&mut self.translate_dy).speed(1.0));
                                ui.add(egui::DragValue::new(&mut self.translate_dz).speed(1.0));
                            });
                            if ui.button("Translate").clicked() { self.model_translate(); }
                            ui.horizontal(|ui| {
                                ui.label("axis:");
                                ui.add(egui::DragValue::new(&mut self.rotate_axis_x).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.rotate_axis_y).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.rotate_axis_z).speed(0.05).range(-1.0..=1.0));
                            });
                            ui.horizontal(|ui| {
                                ui.label("angle:");
                                ui.add(egui::DragValue::new(&mut self.rotate_angle_deg)
                                    .speed(1.0).range(-360.0..=360.0).suffix("°"));
                            });
                            if ui.button("Rotate").clicked() { self.model_rotate(); }
                            ui.horizontal(|ui| {
                                ui.label("pivot:");
                                ui.add(egui::DragValue::new(&mut self.rotate_pivot_x).speed(0.5));
                                ui.add(egui::DragValue::new(&mut self.rotate_pivot_y).speed(0.5));
                                ui.add(egui::DragValue::new(&mut self.rotate_pivot_z).speed(0.5));
                            });
                            if ui.button("Rotate (around pivot)").clicked() { self.model_rotate_around_point(); }
                            ui.horizontal(|ui| {
                                ui.label("scale:");
                                ui.add(egui::DragValue::new(&mut self.scale_factor)
                                    .speed(0.05).range(0.01..=100.0));
                            });
                            if ui.button("Scale").clicked() { self.model_scale(); }
                            ui.horizontal(|ui| {
                                ui.label("pivot:");
                                ui.add(egui::DragValue::new(&mut self.scale_pivot_x).speed(0.5));
                                ui.add(egui::DragValue::new(&mut self.scale_pivot_y).speed(0.5));
                                ui.add(egui::DragValue::new(&mut self.scale_pivot_z).speed(0.5));
                            });
                            if ui.button("Scale (around pivot)").clicked() { self.model_scale_around_point(); }
                            ui.horizontal(|ui| {
                                ui.label("normal:");
                                ui.add(egui::DragValue::new(&mut self.mirror_nx).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.mirror_ny).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.mirror_nz).speed(0.05).range(-1.0..=1.0));
                            });
                            if ui.button("Mirror").clicked() { self.model_mirror(); }

                            // Face ops
                            ui.add_space(2.0);
                            ui.label(egui::RichText::new("Face Ops").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("face:");
                                ui.add(egui::DragValue::new(&mut self.face_op_index)
                                    .range(0..=100000));
                                ui.label("hole:");
                                ui.add(egui::DragValue::new(&mut self.hole_op_index)
                                    .range(0..=100000));
                            });
                            ui.horizontal(|ui| {
                                if ui.button("Delete Face").clicked() { self.model_delete_face(); }
                                if ui.button("Reverse Face").clicked() { self.model_reverse_face(); }
                            });
                            ui.horizontal(|ui| {
                                if ui.button("Clear Holes").clicked() { self.model_clear_holes(); }
                                if ui.button("Remove Hole").clicked() { self.model_remove_hole(); }
                            });

                            // Patterns
                            ui.add_space(2.0);
                            ui.label(egui::RichText::new("Pattern").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("count:");
                                ui.add(egui::DragValue::new(&mut self.pattern_count)
                                    .range(1..=100));
                            });
                            if ui.button("Circular Pattern (Z)").clicked() { self.model_circular_pattern(); }

                            // Boolean
                            ui.add_space(2.0);
                            ui.label(egui::RichText::new("Boolean").size(11.0).strong());
                            if ui.button("Set Current as B").clicked() { self.model_capture_secondary(); }
                            ui.horizontal(|ui| {
                                if ui.button("A ∪ B").clicked() { self.model_boolean_union(); }
                                if ui.button("A − B").clicked() { self.model_boolean_subtract(); }
                                if ui.button("A ∩ B").clicked() { self.model_boolean_intersect(); }
                            });

                            // GDT
                            ui.add_space(2.0);
                            ui.label(egui::RichText::new("GDT Check").size(11.0).strong());
                            egui::ComboBox::from_id_salt("gdt_type")
                                .selected_text(gdt_type_label(self.gdt_check_type))
                                .show_ui(ui, |ui| {
                                    for i in 0..=8u32 {
                                        ui.selectable_value(&mut self.gdt_check_type, i, gdt_type_label(i));
                                    }
                                });
                            ui.horizontal(|ui| {
                                ui.label("tol:");
                                ui.add(egui::DragValue::new(&mut self.gdt_tolerance)
                                    .speed(0.01).range(0.001..=100.0).suffix(" mm"));
                            });
                            if ui.button("Run GDT Check").clicked() { self.model_gdt_check(); }
                            if let Some((tol, actual, passed)) = self.gdt_last_result {
                                let color = if passed {
                                    egui::Color32::from_rgb(80, 200, 80)
                                } else {
                                    egui::Color32::from_rgb(220, 80, 80)
                                };
                                ui.label(
                                    egui::RichText::new(format!(
                                        "Actual: {:.4} / Tol: {:.4} → {}",
                                        actual, tol,
                                        if passed { "PASS" } else { "FAIL" }
                                    ))
                                    .size(10.0)
                                    .color(color)
                                );
                            }
                        });
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

                // LOD selector for mobile
                ui.add_space(4.0);
                egui::ComboBox::from_id_salt("lod_mobile")
                    .selected_text(format!("Quality: {}", self.lod_level.label()))
                    .show_ui(ui, |ui| {
                        for &lod in LodLevel::all() {
                            ui.selectable_value(&mut self.lod_level, lod, lod.label());
                        }
                    });

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
                    }); // close ScrollArea
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

        // ═══ UV breakdown window (desktop + mobile) ═══════════════════════════
        // Shows a per-face UV grid for the current solid (any primitive or
        // NURBS surface test). Lets the user switch between faces, adjust
        // grid resolution, and save the SVG to disk / trigger a download.
        self.draw_uv_window(ctx);

        // ═══ Pending UV SVG export ═════════════════════════════════════════════
        if self.pending_solid_uv_svg_export {
            self.pending_solid_uv_svg_export = false;
            // Ensure the breakdown is computed for the current solid.
            if self.solid_uv_breakdown.is_none() {
                if let Some(ref solid) = self.current_solid {
                    let name = self.current_model.name.clone();
                    self.solid_uv_breakdown = Some(compute_solid_uv_breakdown(solid, &name));
                }
            }
            if let Some(ref breakdown) = self.solid_uv_breakdown {
                if let Some(face_uv) = breakdown.faces.get(self.uv_window_face_idx) {
                    // Get the surface from the solid for grid-point rendering.
                    let surface: Option<Surface> = self.current_solid.as_ref().and_then(|s| {
                        s.outer_shell.as_ref().and_then(|sh| {
                            sh.faces.get(self.uv_window_face_idx)
                                .and_then(|f| f.surface.clone())
                        })
                    });
                    let svg = generate_solid_face_uv_svg(
                        face_uv,
                        self.uv_window_u_divs,
                        self.uv_window_v_divs,
                        &breakdown.model_name,
                        surface.as_ref(),
                    );
                    let filename = format!(
                        "uv_face{}_{}.svg",
                        face_uv.face_idx,
                        breakdown.model_name.replace(' ', "_").replace('/', "_")
                    );
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("SVG", &["svg"])
                            .set_file_name(&filename)
                            .save_file()
                        {
                            match std::fs::write(&path, &svg) {
                                Ok(()) => self.log(&format!("Exported UV SVG: {}", path.to_string_lossy())),
                                Err(e) => self.log(&format!("SVG export error: {}", e)),
                            }
                        }
                    }
                    #[cfg(target_arch = "wasm32")]
                    {
                        self.download_text(&filename, "image/svg+xml", &svg);
                        self.log(&format!("Exported UV SVG ({})", filename));
                    }
                } else {
                    self.log_warning("UV export: no face selected");
                }
            } else {
                self.log_warning("UV export: no solid loaded");
            }
        }
    }
}

impl ViewerApp {

    /// Draw the UV breakdown window for the current solid.
    ///
    /// Shows a face selector (combo box), U/V division sliders, the UV grid
    /// canvas (rendered directly via painter), and Save / Close buttons.
    /// On WASM the Save button triggers a browser SVG download; on native
    /// it opens an rfd save-file dialog.
    fn draw_uv_window(&mut self, ctx: &egui::Context) {
        if !self.show_uv_window {
            return;
        }
        // Compute breakdown on demand if not cached.
        if self.solid_uv_breakdown.is_none() {
            if let Some(ref solid) = self.current_solid {
                let name = self.current_model.name.clone();
                self.solid_uv_breakdown = Some(compute_solid_uv_breakdown(solid, &name));
                // Clamp face index to valid range.
                if let Some(ref b) = self.solid_uv_breakdown {
                    if self.uv_window_face_idx >= b.faces.len() {
                        self.uv_window_face_idx = 0;
                    }
                }
            }
        }

        let mut window_open = self.show_uv_window;
        // We need to clone or borrow carefully. The breakdown is in self,
        // but we also need to mutate self inside the window. So we take
        // the breakdown out temporarily.
        let breakdown_taken = self.solid_uv_breakdown.take();
        let current_solid_taken = self.current_solid.clone();
        let model_name = self.current_model.name.clone();

        egui::Window::new("UV Breakdown")
            .id(egui::Id::new("uv_breakdown_window"))
            .open(&mut window_open)
            .default_width(560.0)
            .default_height(640.0)
            .resizable(true)
            .collapsible(true)
            .scroll([false, true])
            .show(ctx, |ui| {
                if breakdown_taken.is_none() {
                    ui.label(egui::RichText::new(
                        "No solid loaded. Click a primitive (Box, Cylinder, ...) first.")
                        .color(egui::Color32::from_rgb(255, 200, 80))
                        .size(12.0));
                    return;
                }
                let breakdown = breakdown_taken.as_ref().unwrap();
                if breakdown.faces.is_empty() {
                    ui.label(egui::RichText::new("Solid has no faces to display.")
                        .color(egui::Color32::GRAY)
                        .size(12.0));
                    return;
                }

                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("Face:").size(12.0));
                    let selected_label = breakdown.faces
                        .get(self.uv_window_face_idx)
                        .map(|f| format!("#{} {}", f.face_idx, f.surface_type))
                        .unwrap_or_else(|| "(none)".to_string());
                    egui::ComboBox::from_id_salt("uv_face_combo")
                        .selected_text(selected_label)
                        .show_ui(ui, |ui| {
                            for f in &breakdown.faces {
                                ui.selectable_value(
                                    &mut self.uv_window_face_idx,
                                    f.face_idx,
                                    format!("#{} {} (outer pts: {}, holes: {})",
                                        f.face_idx, f.surface_type,
                                        f.outer_polylines.iter().map(|p| p.len()).sum::<usize>(),
                                        f.inner_polylines.len()),
                                );
                            }
                        });
                });

                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("U divs:").size(11.0));
                    ui.add(egui::DragValue::new(&mut self.uv_window_u_divs).range(2..=50));
                    ui.label(egui::RichText::new("V divs:").size(11.0));
                    ui.add(egui::DragValue::new(&mut self.uv_window_v_divs).range(2..=50));
                });

                ui.horizontal(|ui| {
                    if ui.button("Save UV as SVG...").clicked() {
                        self.pending_solid_uv_svg_export = true;
                    }
                    if ui.button("Recompute").clicked() {
                        // Force recompute on next frame
                        self.solid_uv_breakdown = None;
                    }
                });

                ui.separator();

                if let Some(face_uv) = breakdown.faces.get(self.uv_window_face_idx) {
                    // Get the surface for this face from the current solid.
                    let surface: Option<Surface> = current_solid_taken.as_ref().and_then(|s| {
                        s.outer_shell.as_ref().and_then(|sh| {
                            sh.faces.get(self.uv_window_face_idx)
                                .and_then(|f| f.surface.clone())
                        })
                    });
                    let surface_ref = surface.as_ref();

                    // Header line
                    ui.label(egui::RichText::new(format!(
                        "Model: {} | Face #{} {} | forward={}",
                        model_name, face_uv.face_idx, face_uv.surface_type, face_uv.forward
                    )).size(11.0));

                    // Draw the UV grid in a square painter area
                    let available = ui.available_size();
                    let size = available.x.min(available.y - 20.0).min(480.0);
                    if size > 50.0 {
                        let (rect, _response) = ui.allocate_exact_size(
                            egui::vec2(size, size),
                            egui::Sense::hover(),
                        );
                        ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(26, 26, 46));

                        let margin = size * 0.067;
                        let draw_size = size - 2.0 * margin;

                        // Compute UV bounds
                        let mut u_min = f64::MAX;
                        let mut u_max = f64::MIN;
                        let mut v_min = f64::MAX;
                        let mut v_max = f64::MIN;
                        for poly in &face_uv.outer_polylines {
                            for &(u, v) in poly {
                                u_min = u_min.min(u); u_max = u_max.max(u);
                                v_min = v_min.min(v); v_max = v_max.max(v);
                            }
                        }
                        for poly in &face_uv.inner_polylines {
                            for &(u, v) in poly {
                                u_min = u_min.min(u); u_max = u_max.max(u);
                                v_min = v_min.min(v); v_max = v_max.max(v);
                            }
                        }
                        if u_min >= u_max || v_min >= v_max {
                            if let Some(s) = surface_ref {
                                match s {
                                    Surface::Nurbs(n) => {
                                        let (ur0, ur1) = n.u_range();
                                        let (vr0, vr1) = n.v_range();
                                        u_min = ur0; u_max = ur1; v_min = vr0; v_max = vr1;
                                    }
                                    Surface::Cylinder(_) => {
                                        u_min = 0.0; u_max = 2.0 * std::f64::consts::PI;
                                        v_min = -100.0; v_max = 100.0;
                                    }
                                    _ => { u_min = 0.0; u_max = 1.0; v_min = 0.0; v_max = 1.0; }
                                }
                            } else {
                                u_min = 0.0; u_max = 1.0; v_min = 0.0; v_max = 1.0;
                            }
                        }
                        let u_range = (u_max - u_min).max(1e-6);
                        let v_range = (v_max - v_min).max(1e-6);
                        u_min -= u_range * 0.05; u_max += u_range * 0.05;
                        v_min -= v_range * 0.05; v_max += v_range * 0.05;

                        let margin_f64 = margin as f64;
                        let draw_size_f64 = draw_size as f64;
                        let rect_left = rect.left() as f64;
                        let rect_top = rect.top() as f64;
                        // IMPORTANT: map_u/map_v must return SCREEN coords that are
                        // relative to the rect (the allocated area inside the
                        // egui::Window). Without the rect.left()/rect.top() offset
                        // the painted content would be stuck at screen position
                        // (margin, margin) regardless of where the user dragged
                        // the window.
                        let map_u = |u: f64| -> f32 { (rect_left + margin_f64 + (u - u_min) / (u_max - u_min) * draw_size_f64) as f32 };
                        let map_v = |v: f64| -> f32 { (rect_top + margin_f64 + (1.0 - (v - v_min) / (v_max - v_min)) * draw_size_f64) as f32 };

                        // Grid lines
                        let u_divs = self.uv_window_u_divs.min(50);
                        let v_divs = self.uv_window_v_divs.min(50);
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

                        // ─── UV triangles (the actual surface triangulation) ───────
                        // Draw filled triangles so the user can see the actual UV
                        // subdivision of the face, not just the boundary.
                        let outer_uv_poly: Vec<(f64, f64)> = face_uv.outer_polylines.iter()
                            .flat_map(|p| p.iter().copied())
                            .collect();
                        let hole_polys: Vec<Vec<(f64, f64)>> = face_uv.inner_polylines.iter()
                            .cloned()
                            .collect();
                        if !face_uv.uv_triangles.is_empty() {
                            let tri_limit = 3000.min(face_uv.uv_triangles.len());
                            for (ti, tri) in face_uv.uv_triangles.iter().enumerate() {
                                let cu = (tri[0].0 + tri[1].0 + tri[2].0) / 3.0;
                                let cv = (tri[0].1 + tri[1].1 + tri[2].1) / 3.0;
                                let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h));
                                let in_outer = !outer_uv_poly.is_empty() && point_in_polygon(cu, cv, &outer_uv_poly);

                                let p0 = egui::pos2(map_u(tri[0].0), map_v(tri[0].1));
                                let p1 = egui::pos2(map_u(tri[1].0), map_v(tri[1].1));
                                let p2 = egui::pos2(map_u(tri[2].0), map_v(tri[2].1));

                                if in_hole || !in_outer {
                                    // Triangle inside a hole or outside the outer boundary — outline only.
                                    ui.painter().add(egui::Shape::convex_polygon(
                                        vec![p0, p1, p2],
                                        egui::Color32::from_rgba_premultiplied(255, 34, 34, 30),
                                        egui::Stroke::new(0.4, egui::Color32::from_rgba_premultiplied(255, 68, 68, 100)),
                                    ));
                                } else {
                                    // Valid triangle — alternating blue tints + thin outline.
                                    let fill = if ti % 2 == 0 {
                                        egui::Color32::from_rgba_premultiplied(68, 136, 255, 24)
                                    } else {
                                        egui::Color32::from_rgba_premultiplied(85, 170, 255, 24)
                                    };
                                    let stroke = if ti % 2 == 0 {
                                        egui::Stroke::new(0.4, egui::Color32::from_rgba_premultiplied(68, 136, 255, 140))
                                    } else {
                                        egui::Stroke::new(0.4, egui::Color32::from_rgba_premultiplied(85, 170, 255, 140))
                                    };
                                    ui.painter().add(egui::Shape::convex_polygon(vec![p0, p1, p2], fill, stroke));
                                }
                                if ti >= tri_limit { break; }
                            }
                        }

                        // Outer boundary
                        for poly in &face_uv.outer_polylines {
                            if poly.len() < 2 { continue; }
                            let points: Vec<egui::Pos2> = poly.iter()
                                .map(|&(u, v)| egui::pos2(map_u(u), map_v(v)))
                                .collect();
                            ui.painter().line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(0, 255, 136)));
                        }
                        // Inner boundaries
                        for poly in &face_uv.inner_polylines {
                            if poly.len() < 2 { continue; }
                            let points: Vec<egui::Pos2> = poly.iter()
                                .map(|&(u, v)| egui::pos2(map_u(u), map_v(v)))
                                .collect();
                            ui.painter().line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 68, 68)));
                        }

                        // Surface evaluation points (inside outer boundary)
                        if let Some(s) = surface_ref {
                            for i in 0..=u_divs {
                                for j in 0..=v_divs {
                                    let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                                    let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                                    let p = s.point_at(u, v);
                                    if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
                                        let inside = !outer_uv_poly.is_empty() && point_in_polygon(u, v, &outer_uv_poly);
                                        if inside {
                                            ui.painter().circle_filled(
                                                egui::pos2(map_u(u), map_v(v)), 2.0,
                                                egui::Color32::from_rgba_premultiplied(102, 136, 255, 180),
                                            );
                                        }
                                    }
                                }
                            }
                        }

                        // Axis labels
                        ui.painter().text(
                            egui::pos2(rect.center().x, rect.bottom() - 5.0),
                            egui::Align2::CENTER_BOTTOM,
                            format!("U ({:.2}..{:.2})", u_min, u_max),
                            egui::FontId::proportional(10.0),
                            egui::Color32::from_rgb(170, 170, 170),
                        );
                        ui.painter().text(
                            egui::pos2(rect.left() + 8.0, rect.center().y),
                            egui::Align2::CENTER_CENTER,
                            format!("V ({:.2}..{:.2})", v_min, v_max),
                            egui::FontId::proportional(10.0),
                            egui::Color32::from_rgb(170, 170, 170),
                        );
                    }
                }
            });

        // Restore the breakdown (only if still valid — it was before).
        if self.solid_uv_breakdown.is_none() {
            self.solid_uv_breakdown = breakdown_taken;
        }
        self.show_uv_window = window_open;
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
                    // File menu — import + export (WASM downloads via Blob)
                    ui.menu_button("File", |ui| {
                        // ── Import ──
                        if ui.button("Import STL...").clicked() {
                            #[cfg(target_arch = "wasm32")]
                            self.trigger_stl_file_input();
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("STL", &["stl"])
                                    .pick_file()
                                {
                                    self.import_stl_file(&path.to_string_lossy());
                                }
                            }
                            ui.close_menu();
                        }
                        if ui.button("Import STEP...").clicked() {
                            #[cfg(target_arch = "wasm32")]
                            self.trigger_step_file_input();
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("STEP", &["stp", "step"])
                                    .pick_file()
                                {
                                    self.import_step_file(&path.to_string_lossy());
                                }
                            }
                            ui.close_menu();
                        }
                        if ui.button("Import JSON...").clicked() {
                            #[cfg(target_arch = "wasm32")]
                            self.trigger_json_file_input();
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("JSON", &["json"])
                                    .pick_file()
                                {
                                    self.import_json(&path.to_string_lossy());
                                }
                            }
                            ui.close_menu();
                        }
                        ui.separator();
                        // ── Export ──
                        if ui.button("Export STL (Binary)").clicked() {
                            #[cfg(target_arch = "wasm32")]
                            self.export_stl_binary_wasm();
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("STL", &["stl"])
                                    .save_file()
                                {
                                    self.export_stl_binary(&path.to_string_lossy());
                                }
                            }
                            ui.close_menu();
                        }
                        if ui.button("Export STL (ASCII)").clicked() {
                            #[cfg(target_arch = "wasm32")]
                            self.export_stl_ascii_wasm();
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("STL", &["stl"])
                                    .save_file()
                                {
                                    self.export_stl_ascii(&path.to_string_lossy());
                                }
                            }
                            ui.close_menu();
                        }
                        if ui.button("Export STEP").clicked() {
                            #[cfg(target_arch = "wasm32")]
                            self.export_step_wasm();
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("STEP", &["stp", "step"])
                                    .save_file()
                                {
                                    self.export_step(&path.to_string_lossy());
                                }
                            }
                            ui.close_menu();
                        }
                        if ui.button("Export JSON").clicked() {
                            #[cfg(target_arch = "wasm32")]
                            self.export_json_wasm();
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("JSON", &["json"])
                                    .save_file()
                                {
                                    self.export_json(&path.to_string_lossy());
                                }
                            }
                            ui.close_menu();
                        }
                    });
                    // View presets + display toggles
                    ui.menu_button("View", |ui| {
                        if ui.button("Reset Camera").clicked() {
                            let (bbox_min, bbox_max) = self.mesh.bounding_box();
                            self.camera.fit_and_reset_orientation(
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
                        ui.checkbox(&mut self.show_grid, "Grid");
                    });
                    // Quick primitives — full list (matches desktop left panel)
                    ui.menu_button("Models", |ui| {
                        if ui.button("Box").clicked()            { self.load_box();         self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("Cylinder").clicked()       { self.load_cylinder();    self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("Sphere").clicked()         { self.load_sphere();      self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("Cone").clicked()           { self.load_cone();        self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("Torus").clicked()          { self.load_torus();       self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("Revolution").clicked()     { self.load_revolution();  self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("Extrusion").clicked()      { self.load_extrusion();   self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("NURBS (Saddle)").clicked() { self.load_nurbs_saddle(); self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("NURBS Bump").clicked()     { self.load_nurbs_bump();   self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("NURBS Wave").clicked()     { self.load_nurbs_wave();   self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("NURBS Ruled").clicked()    { self.load_nurbs_ruled();  self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("NURBS Coons").clicked()    { self.load_nurbs_coons();  self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("NURBS Half-Cyl").clicked() { self.load_nurbs_half_cylinder(); self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("NURBS Q-Sphere").clicked() { self.load_nurbs_quarter_sphere(); self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("Curve Gallery").clicked()  { self.load_curve_all();    self.close_mobile_panel_after_load = true; ui.close_menu(); }
                        if ui.button("ICE Engine").clicked()     { self.load_engine();      self.close_mobile_panel_after_load = true; ui.close_menu(); }
                    });
                    // Spacer + info summary
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
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
            let panel_width = (screen.width() * 0.92).min(360.0);
            egui::Window::new("Controls")
                .id(egui::Id::new("mobile_controls_window"))
                .fixed_pos(egui::Pos2::new(screen.min.x + (screen.width() - panel_width) * 0.5, screen.min.y + 42.0))
                .fixed_size(egui::vec2(panel_width, screen.height() - 50.0))
                .resizable(false)
                .collapsible(false)
                .default_open(true)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    // ── Tab bar (compact, finger-friendly) ──
                    ui.horizontal(|ui| {
                        for &tab in MobileControlsTab::all() {
                            let active = self.mobile_controls_tab == tab;
                            let txt = egui::RichText::new(tab.label()).size(12.0);
                            let btn = if active {
                                ui.add(egui::Button::new(txt.strong()).fill(egui::Color32::from_rgb(60, 120, 200)))
                            } else {
                                ui.add(egui::Button::new(txt))
                            };
                            if btn.clicked() {
                                self.mobile_controls_tab = tab;
                            }
                        }
                    });
                    ui.separator();

                    egui::ScrollArea::vertical().show(ui, |ui| match self.mobile_controls_tab {
                        MobileControlsTab::Primitives => {
                            // ── Primitives tab ──
                            ui.heading(egui::RichText::new("Solids").size(13.0));
                            // 3-column grid for compact, finger-friendly tap targets
                            egui::Grid::new("mob_prim_grid").num_columns(3).spacing([6.0, 6.0]).show(ui, |ui| {
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Box")).clicked()            { self.load_box();         self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Cylinder")).clicked()       { self.load_cylinder();    self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Sphere")).clicked()         { self.load_sphere();      self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Cone")).clicked()           { self.load_cone();        self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Torus")).clicked()          { self.load_torus();       self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Revolution")).clicked()     { self.load_revolution();  self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Extrusion")).clicked()      { self.load_extrusion();   self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("NURBS")).clicked()          { self.load_nurbs();       self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Engine")).clicked()         { self.load_engine();      self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                            });

                            ui.add_space(6.0);
                            ui.heading(egui::RichText::new("Quick Camera").size(13.0));
                            egui::Grid::new("mob_cam_grid").num_columns(4).spacing([4.0, 4.0]).show(ui, |ui| {
                                if ui.add_sized([68.0, 30.0], egui::Button::new("Reset")).clicked() {
                                    let (bmin, bmax) = self.mesh.bounding_box();
                                    self.camera.fit_and_reset_orientation(
                                        [bmin.x as f32, bmin.y as f32, bmin.z as f32],
                                        [bmax.x as f32, bmax.y as f32, bmax.z as f32],
                                    );
                                }
                                if ui.add_sized([68.0, 30.0], egui::Button::new("Top")).clicked()   { self.camera.look_from_direction([0.0, -1.0, 0.0]); }
                                if ui.add_sized([68.0, 30.0], egui::Button::new("Front")).clicked() { self.camera.look_from_direction([0.0, 0.0, 1.0]); }
                                if ui.add_sized([68.0, 30.0], egui::Button::new("Iso")).clicked()  {
                                    let d = 45.0_f32.to_radians();
                                    let e = 30.0_f32.to_radians();
                                    self.camera.look_from_direction([
                                        -e.cos() * d.sin(), -e.sin(), e.cos() * d.cos(),
                                    ]);
                                }
                                ui.end_row();
                            });

                            ui.add_space(6.0);
                            ui.heading(egui::RichText::new("UV Breakdown").size(13.0));
                            ui.label(egui::RichText::new("View / save parametric UV grid of the current solid").size(10.0).color(egui::Color32::GRAY));
                            egui::Grid::new("mob_uv_grid").num_columns(2).spacing([6.0, 6.0]).show(ui, |ui| {
                                if ui.add_sized([120.0, 34.0], egui::Button::new("View UV")).clicked() {
                                    self.show_uv_window = true;
                                    self.solid_uv_breakdown = None;
                                    // Close the mobile controls panel so the UV
                                    // window is visible above the 3D viewport.
                                    self.mobile_panel = None;
                                }
                                if ui.add_sized([120.0, 34.0], egui::Button::new("Save UV SVG")).clicked() {
                                    self.show_uv_window = true;
                                    self.pending_solid_uv_svg_export = true;
                                    self.solid_uv_breakdown = None;
                                    self.mobile_panel = None;
                                }
                                ui.end_row();
                            });

                            ui.add_space(6.0);
                            if ui.add_sized([ui.available_width(), 32.0], egui::Button::new("Clear Selection")).clicked() {
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

                        MobileControlsTab::Surfaces => {
                            // ── Surfaces tab: comprehensive NURBS surface tests ──
                            ui.label(egui::RichText::new("NURBS surface gallery").size(11.0).color(egui::Color32::GRAY));
                            ui.add_space(4.0);
                            egui::Grid::new("mob_surf_grid").num_columns(2).spacing([6.0, 6.0]).show(ui, |ui| {
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Saddle")).clicked()      { self.load_nurbs_saddle();          self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Bump")).clicked()        { self.load_nurbs_bump();            self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Wave")).clicked()        { self.load_nurbs_wave();            self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Ruled")).clicked()       { self.load_nurbs_ruled();           self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Revolution")).clicked()  { self.load_nurbs_revolution();      self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Coons")).clicked()       { self.load_nurbs_coons();           self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Bilinear")).clicked()    { self.load_nurbs_bilinear();        self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Half-Cyl")).clicked()    { self.load_nurbs_half_cylinder();   self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Q-Sphere")).clicked()    { self.load_nurbs_quarter_sphere();  self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Closed-Cyl")).clicked()  { self.load_nurbs_closed_cylinder(); self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                            });
                            ui.add_space(6.0);
                            ui.label(egui::RichText::new("Primitive analytic surfaces").size(11.0).color(egui::Color32::GRAY));
                            egui::Grid::new("mob_prim_surf_grid").num_columns(3).spacing([6.0, 6.0]).show(ui, |ui| {
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Box")).clicked()       { self.load_box();       self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Cylinder")).clicked() { self.load_cylinder();  self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Sphere")).clicked()   { self.load_sphere();    self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Cone")).clicked()      { self.load_cone();      self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Torus")).clicked()     { self.load_torus();     self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Revolution")).clicked() { self.load_revolution(); self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Extrusion")).clicked() { self.load_extrusion(); self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                            });
                        }

                        MobileControlsTab::Curves => {
                            // ── Curves tab: comprehensive 3D curve tests ──
                            ui.label(egui::RichText::new("3D curve gallery (rendered as colored line strips)").size(11.0).color(egui::Color32::GRAY));
                            ui.add_space(4.0);
                            egui::Grid::new("mob_curves_grid").num_columns(2).spacing([6.0, 6.0]).show(ui, |ui| {
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Line")).clicked()         { self.load_curve_line();         self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Circle")).clicked()      { self.load_curve_circle();       self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Ellipse")).clicked()     { self.load_curve_ellipse();      self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Hyperbola")).clicked()   { self.load_curve_hyperbola();    self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Parabola")).clicked()    { self.load_curve_parabola();     self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("NURBS open")).clicked()  { self.load_curve_nurbs_open();   self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("NURBS closed")).clicked(){ self.load_curve_nurbs_closed(); self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("Trimmed")).clicked()     { self.load_curve_trimmed();      self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([120.0, 32.0], egui::Button::new("PCurve")).clicked()      { self.load_curve_pcurve();       self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([120.0, 32.0], egui::Button::new("All (Gallery)")).clicked() { self.load_curve_all();        self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                            });
                            ui.add_space(6.0);
                            ui.label(egui::RichText::new("Tip: tap \"All\" to see every curve type side-by-side, each in a different color.").size(10.0).color(egui::Color32::GRAY));
                        }

                        MobileControlsTab::Holes => {
                            // ── Hole:3 cut-outs tab ──
                            ui.label(egui::RichText::new("Cut-out \"3\" on each surface type").size(11.0).color(egui::Color32::GRAY));
                            ui.add_space(4.0);
                            egui::Grid::new("mob_holes_grid").num_columns(3).spacing([6.0, 6.0]).show(ui, |ui| {
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Box~")).clicked()   { self.load_box_text();         self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Cyl~")).clicked()  { self.load_cylinder_text();    self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Sph~")).clicked()  { self.load_sphere_text();      self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Cone~")).clicked() { self.load_cone_text();        self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Torus~")).clicked() { self.load_torus_text();       self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Rev~")).clicked()  { self.load_revolution_text();  self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                                if ui.add_sized([90.0, 32.0], egui::Button::new("Ext~")).clicked()  { self.load_extrusion_text();   self.close_mobile_panel_after_load = true; }
                                if ui.add_sized([90.0, 32.0], egui::Button::new("NURBS~")).clicked() { self.load_nurbs_text();      self.close_mobile_panel_after_load = true; }
                                ui.end_row();
                            });
                        }

                        MobileControlsTab::Modeling => {
                            // ── Modeling tab (Fillet / Chamfer / Shell / Transform / Boolean / GDT) ──
                            ui.label(egui::RichText::new("Fillet / Chamfer / Shell").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("r:");
                                ui.add(egui::DragValue::new(&mut self.fillet_radius).speed(0.1).range(0.01..=100.0).suffix(" mm"));
                            });
                            ui.horizontal(|ui| {
                                ui.label("edge:");
                                ui.add(egui::DragValue::new(&mut self.model_edge_index).range(0..=100000));
                                ui.label("(0=auto)");
                            });
                            ui.horizontal(|ui| {
                                if ui.add_sized([100.0, 28.0], egui::Button::new("Fillet")).clicked()  { self.model_fillet_edge(); }
                                if ui.add_sized([100.0, 28.0], egui::Button::new("Chamfer")).clicked() { self.model_chamfer_edge(); }
                            });
                            ui.horizontal(|ui| {
                                ui.label("t:");
                                ui.add(egui::DragValue::new(&mut self.shell_thickness).speed(0.1).range(0.01..=100.0).suffix(" mm"));
                            });
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Shell")).clicked() { self.model_make_shell(); }

                            ui.add_space(4.0);
                            ui.label(egui::RichText::new("Transform").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("Δ:");
                                ui.add(egui::DragValue::new(&mut self.translate_dx).speed(1.0));
                                ui.add(egui::DragValue::new(&mut self.translate_dy).speed(1.0));
                                ui.add(egui::DragValue::new(&mut self.translate_dz).speed(1.0));
                            });
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Translate")).clicked() { self.model_translate(); }
                            ui.horizontal(|ui| {
                                ui.label("axis:");
                                ui.add(egui::DragValue::new(&mut self.rotate_axis_x).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.rotate_axis_y).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.rotate_axis_z).speed(0.05).range(-1.0..=1.0));
                            });
                            ui.horizontal(|ui| {
                                ui.label("angle:");
                                ui.add(egui::DragValue::new(&mut self.rotate_angle_deg).speed(1.0).range(-360.0..=360.0).suffix("°"));
                            });
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Rotate")).clicked() { self.model_rotate(); }
                            ui.horizontal(|ui| {
                                ui.label("scale:");
                                ui.add(egui::DragValue::new(&mut self.scale_factor).speed(0.05).range(0.01..=100.0));
                            });
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Scale")).clicked() { self.model_scale(); }
                            ui.horizontal(|ui| {
                                ui.label("normal:");
                                ui.add(egui::DragValue::new(&mut self.mirror_nx).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.mirror_ny).speed(0.05).range(-1.0..=1.0));
                                ui.add(egui::DragValue::new(&mut self.mirror_nz).speed(0.05).range(-1.0..=1.0));
                            });
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Mirror")).clicked() { self.model_mirror(); }

                            ui.add_space(4.0);
                            ui.label(egui::RichText::new("Boolean").size(11.0).strong());
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Set Current as B")).clicked() { self.model_capture_secondary(); }
                            ui.horizontal(|ui| {
                                if ui.add_sized([90.0, 28.0], egui::Button::new("A ∪ B")).clicked() { self.model_boolean_union(); }
                                if ui.add_sized([90.0, 28.0], egui::Button::new("A − B")).clicked() { self.model_boolean_subtract(); }
                                if ui.add_sized([90.0, 28.0], egui::Button::new("A ∩ B")).clicked() { self.model_boolean_intersect(); }
                            });

                            ui.add_space(4.0);
                            ui.label(egui::RichText::new("Pattern").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("count:");
                                ui.add(egui::DragValue::new(&mut self.pattern_count).range(1..=100));
                            });
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Circular Pattern (Z)")).clicked() { self.model_circular_pattern(); }

                            ui.add_space(4.0);
                            ui.label(egui::RichText::new("Face Ops").size(11.0).strong());
                            ui.horizontal(|ui| {
                                ui.label("face:");
                                ui.add(egui::DragValue::new(&mut self.face_op_index).range(0..=100000));
                                ui.label("hole:");
                                ui.add(egui::DragValue::new(&mut self.hole_op_index).range(0..=100000));
                            });
                            ui.horizontal(|ui| {
                                if ui.add_sized([80.0, 28.0], egui::Button::new("Delete")).clicked()  { self.model_delete_face(); }
                                if ui.add_sized([80.0, 28.0], egui::Button::new("Reverse")).clicked() { self.model_reverse_face(); }
                                if ui.add_sized([80.0, 28.0], egui::Button::new("RmHole")).clicked()  { self.model_remove_hole(); }
                            });

                            ui.add_space(4.0);
                            ui.label(egui::RichText::new("GDT Check").size(11.0).strong());
                            egui::ComboBox::from_id_salt("gdt_type_mobile")
                                .selected_text(gdt_type_label(self.gdt_check_type))
                                .show_ui(ui, |ui| {
                                    for i in 0..=8u32 {
                                        ui.selectable_value(&mut self.gdt_check_type, i, gdt_type_label(i));
                                    }
                                });
                            ui.horizontal(|ui| {
                                ui.label("tol:");
                                ui.add(egui::DragValue::new(&mut self.gdt_tolerance).speed(0.01).range(0.001..=100.0).suffix(" mm"));
                            });
                            if ui.add_sized([ui.available_width(), 28.0], egui::Button::new("Run GDT Check")).clicked() { self.model_gdt_check(); }
                            if let Some((tol, actual, passed)) = self.gdt_last_result {
                                let color = if passed { egui::Color32::from_rgb(80, 200, 80) } else { egui::Color32::from_rgb(220, 80, 80) };
                                ui.label(
                                    egui::RichText::new(format!("Actual: {:.4} / Tol: {:.4} → {}", actual, tol, if passed { "PASS" } else { "FAIL" }))
                                        .size(10.0).color(color)
                                );
                            }
                        }

                        MobileControlsTab::Display => {
                            // ── Display tab ──
                            ui.heading(egui::RichText::new("Display Options").size(13.0));
                            ui.checkbox(&mut self.wireframe, "Wireframe");
                            ui.checkbox(&mut self.show_edges, "Show Edges");
                            ui.checkbox(&mut self.show_wireframe_overlay, "Mesh Overlay");
                            ui.checkbox(&mut self.show_axes, "Show axes");
                            ui.checkbox(&mut self.show_grid, "Show grid");
                            ui.checkbox(&mut self.show_structure, "Structure Panel (desktop)");
                            ui.checkbox(&mut self.show_json_api, "Show JSON API");

                            ui.add_space(6.0);
                            ui.heading(egui::RichText::new("Triangulation Quality").size(13.0));
                            egui::ComboBox::from_id_salt("lod_mobile_panel")
                                .selected_text(format!("Quality: {}", self.lod_level.label()))
                                .show_ui(ui, |ui| {
                                    for &lod in LodLevel::all() {
                                        ui.selectable_value(&mut self.lod_level, lod, lod.label());
                                    }
                                });

                            ui.add_space(6.0);
                            if ui.add_sized([ui.available_width(), 32.0], egui::Button::new("Reset Camera (fit + iso)")).clicked() {
                                let (bmin, bmax) = self.mesh.bounding_box();
                                self.camera.fit_and_reset_orientation(
                                    [bmin.x as f32, bmin.y as f32, bmin.z as f32],
                                    [bmax.x as f32, bmax.y as f32, bmax.z as f32],
                                );
                            }

                            // JSON API panel (only shown if enabled)
                            if self.show_json_api {
                                ui.add_space(6.0);
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
                        }

                        MobileControlsTab::Info => {
                            // ── Info tab ──
                            ui.heading(egui::RichText::new("Model Info").size(13.0));
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

                            ui.separator();
                            ui.heading(egui::RichText::new("Manifold").size(13.0));
                            if let Some(ref report) = self.manifold_report {
                                let watertight = report.is_watertight();
                                let wt_color = if watertight { egui::Color32::from_rgb(80, 200, 80) } else { egui::Color32::from_rgb(255, 100, 80) };
                                ui.horizontal(|ui| {
                                    ui.label(egui::RichText::new("Watertight:").size(12.0));
                                    ui.label(egui::RichText::new(if watertight { "Yes" } else { "No" }).size(12.0).color(wt_color));
                                });
                                ui.label(egui::RichText::new(format!("Euler χ: {}", report.euler_characteristic)).size(11.0));
                                ui.label(egui::RichText::new(format!("Boundary edges: {}", report.boundary_edge_count)).size(11.0));
                                ui.label(egui::RichText::new(format!("Non-manifold edges: {}", report.non_manifold_edge_count)).size(11.0));
                                ui.label(egui::RichText::new(format!("Degenerate tris: {}", report.degenerate_triangle_count)).size(11.0));
                                ui.label(egui::RichText::new(format!("T-junctions: {}", report.t_junction_count)).size(11.0));
                            } else {
                                ui.label(egui::RichText::new("No mesh loaded").size(11.0).color(egui::Color32::GRAY));
                            }

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
                        }
                    });
                });
        }

        // ── Auto-close mobile panel after a model load was triggered ──
        // The model-loading buttons set `close_mobile_panel_after_load = true`
        // so the user can see the freshly-loaded mesh. We reset the flag here
        // AFTER the panel has been drawn for this frame (so the button click
        // was already processed), and close the panel for the NEXT frame.
        if self.close_mobile_panel_after_load {
            self.mobile_panel = None;
            self.close_mobile_panel_after_load = false;
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
                                                let rect_left = rect.left() as f64;
                                                let rect_top = rect.top() as f64;
                                                // map_u/map_v must return SCREEN coords relative to the
                                                // rect — otherwise the painted content sticks at screen
                                                // position (mf, mf) when the parent window is moved.
                                                let map_u = |u: f64| -> f32 { (rect_left + mf + (u - u_min) / (u_max - u_min) * ds) as f32 };
                                                let map_v = |v: f64| -> f32 { (rect_top + mf + (1.0 - (v - v_min) / (v_max - v_min)) * ds) as f32 };

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

/// Sample a single edge into a 3D polyline.
///
/// Returns None if the edge has no curve (degenerate) or is otherwise
/// not sampleable. The polyline has `n_samples` points evenly distributed
/// across the edge's parametric range.
fn sample_edge_polyline(edge: &Edge, n_samples: usize) -> Vec<Point3d> {
    let mut pts = Vec::with_capacity(n_samples);
    let curve = match edge.curve.as_ref() {
        Some(c) => c,
        None => return pts,
    };
    let (tmin, tmax) = edge.param_range;
    if n_samples <= 1 {
        let mid = (tmin + tmax) * 0.5;
        pts.push(curve.point_at(mid));
        return pts;
    }
    for i in 0..n_samples {
        let t = tmin + (tmax - tmin) * (i as f64) / ((n_samples - 1) as f64);
        pts.push(curve.point_at(t));
    }
    pts
}

/// Sample a wire into a 3D polyline by concatenating each coedge's edge
/// samples. Each edge is sampled `samples_per_edge` times. The result is
/// a single Vec<Point3d> (no per-edge grouping) so the caller can project
/// each point onto the surface to build a UV polyline.
///
/// If `forward` is false on a coedge, the edge's samples are reversed so
/// the polyline follows the wire's logical direction.
fn sample_wire_polyline(
    wire: &Wire,
    edges: &[Edge],
    samples_per_edge: usize,
) -> Vec<Point3d> {
    let mut all_pts = Vec::new();
    for coedge in &wire.coedges {
        // Find the matching edge by TopoId.
        let edge = edges.iter().find(|e| e.id == coedge.edge);
        let edge = match edge {
            Some(e) => e,
            None => continue,
        };
        let mut pts = sample_edge_polyline(edge, samples_per_edge);
        if !coedge.forward {
            pts.reverse();
        }
        // Drop the last point of each segment to avoid duplicating
        // shared vertices between consecutive edges. We'll add the
        // final point only when the wire is closed (handled by caller).
        if pts.len() > 1 {
            all_pts.extend_from_slice(&pts[..pts.len() - 1]);
        } else {
            all_pts.extend(pts);
        }
    }
    all_pts
}

/// Compute the UV breakdown for every face of a solid.
///
/// For each face, samples the outer wire (and any inner wires / holes)
/// into 3D polylines, then projects each 3D point onto the face's
/// underlying surface via `Surface::project_point` to obtain UV
/// coordinates. The result is a per-face list of UV polylines.
///
/// Returns an empty `SolidUvBreakdown` if the solid has no outer shell.
fn compute_solid_uv_breakdown(solid: &Solid, model_name: &str) -> SolidUvBreakdown {
    let mut breakdown = SolidUvBreakdown {
        faces: Vec::new(),
        model_name: model_name.to_string(),
    };
    let shell = match solid.outer_shell.as_ref() {
        Some(s) => s,
        None => return breakdown,
    };
    let samples_per_edge = 32; // dense enough to capture curvature on cylinders/spheres
    for (fidx, face) in shell.faces.iter().enumerate() {
        let surface = match face.surface.as_ref() {
            Some(s) => s,
            None => continue,
        };
        let surface_type = surface.type_name().to_string();
        let mut outer_polylines: Vec<Vec<(f64, f64)>> = Vec::new();
        if let Some(ow) = face.outer_wire.as_ref() {
            let pts3d = sample_wire_polyline(ow, &face.edges, samples_per_edge);
            let uv: Vec<(f64, f64)> = pts3d
                .iter()
                .map(|p| surface.project_point(p))
                .collect();
            if uv.len() >= 2 {
                outer_polylines.push(uv);
            }
        }
        let mut inner_polylines: Vec<Vec<(f64, f64)>> = Vec::new();
        for iw in &face.inner_wires {
            let pts3d = sample_wire_polyline(iw, &face.edges, samples_per_edge);
            let uv: Vec<(f64, f64)> = pts3d
                .iter()
                .map(|p| surface.project_point(p))
                .collect();
            if uv.len() >= 2 {
                inner_polylines.push(uv);
            }
        }

        // ─── Compute UV triangles for the face ─────────────────────────────
        // We triangulate the face using the same triangulator the renderer
        // uses, then project every triangle vertex back into UV space via
        // Surface::project_point. This gives us the actual UV tessellation
        // that the user wants to see and save.
        let mut uv_triangles: Vec<[(f64, f64); 3]> = Vec::new();
        let tri_params = TriangulationParams::default();
        let tri_mesh = triangulate_face(face, &tri_params);
        if !tri_mesh.triangles.is_empty() {
            uv_triangles.reserve(tri_mesh.triangles.len());
            for tri in &tri_mesh.triangles {
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;
                if i0 >= tri_mesh.vertices.len()
                    || i1 >= tri_mesh.vertices.len()
                    || i2 >= tri_mesh.vertices.len()
                {
                    continue;
                }
                let v0 = &tri_mesh.vertices[i0];
                let v1 = &tri_mesh.vertices[i1];
                let v2 = &tri_mesh.vertices[i2];
                let (u0, vv0) = surface.project_point(v0);
                let (u1, vv1) = surface.project_point(v1);
                let (u2, vv2) = surface.project_point(v2);
                if u0.is_finite() && vv0.is_finite()
                    && u1.is_finite() && vv1.is_finite()
                    && u2.is_finite() && vv2.is_finite()
                {
                    uv_triangles.push([(u0, vv0), (u1, vv1), (u2, vv2)]);
                }
            }
        }

        breakdown.faces.push(FaceUvBreakdown {
            face_idx: fidx,
            surface_type,
            forward: face.forward,
            outer_polylines,
            inner_polylines,
            uv_triangles,
        });
    }
    breakdown
}

/// Generate an SVG visualization of one face's UV breakdown.
///
/// Renders:
///   - A dark background
///   - Light grid lines at u_divs × v_divs
///   - The outer boundary in green (solid)
///   - Inner boundaries (holes) in red (dashed)
///   - Surface evaluation grid points (only those inside the outer boundary)
///   - Axis labels (U range, V range, face index, surface type, forward flag)
///
/// This is the solid-face analogue of `generate_uv_svg` (which works on
/// STEP `FaceInfo`). The two functions produce visually consistent SVGs.
fn generate_solid_face_uv_svg(
    face_uv: &FaceUvBreakdown,
    u_divs: usize,
    v_divs: usize,
    model_name: &str,
    surface: Option<&Surface>,
) -> String {
    let svg_width = 600.0_f64;
    let svg_height = 600.0_f64;
    let margin = 40.0_f64;
    let draw_w = svg_width - 2.0 * margin;
    let draw_h = svg_height - 2.0 * margin;

    // Compute UV bounds from outer + inner polylines.
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for poly in &face_uv.outer_polylines {
        for &(u, v) in poly {
            u_min = u_min.min(u); u_max = u_max.max(u);
            v_min = v_min.min(v); v_max = v_max.max(v);
        }
    }
    for poly in &face_uv.inner_polylines {
        for &(u, v) in poly {
            u_min = u_min.min(u); u_max = u_max.max(u);
            v_min = v_min.min(v); v_max = v_max.max(v);
        }
    }
    if u_min >= u_max || v_min >= v_max {
        // Fallback: use the surface's natural parametric domain if known.
        if let Some(s) = surface {
            match s {
                Surface::Nurbs(n) => {
                    let (ur0, ur1) = n.u_range();
                    let (vr0, vr1) = n.v_range();
                    u_min = ur0; u_max = ur1; v_min = vr0; v_max = vr1;
                }
                Surface::Cylinder(_) => {
                    u_min = 0.0; u_max = 2.0 * std::f64::consts::PI;
                    v_min = -100.0; v_max = 100.0;
                }
                _ => { u_min = 0.0; u_max = 1.0; v_min = 0.0; v_max = 1.0; }
            }
        } else {
            u_min = 0.0; u_max = 1.0; v_min = 0.0; v_max = 1.0;
        }
    }
    let u_range = (u_max - u_min).max(1e-6);
    let v_range = (v_max - v_min).max(1e-6);
    u_min -= u_range * 0.05; u_max += u_range * 0.05;
    v_min -= v_range * 0.05; v_max += v_range * 0.05;

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

    // Grid lines
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

    // Outer boundary (green)
    for poly in &face_uv.outer_polylines {
        if poly.len() < 2 { continue; }
        let mut d = format!("M {:.2} {:.2}", map_u(poly[0].0), map_v(poly[0].1));
        for &(u, v) in &poly[1..] {
            d.push_str(&format!(" L {:.2} {:.2}", map_u(u), map_v(v)));
        }
        d.push_str(" Z");
        svg.push_str(&format!(
            "  <path d=\"{}\" fill=\"none\" stroke=\"#00ff88\" stroke-width=\"1.5\"/>\n", d
        ));
    }

    // Inner boundaries / holes (red, dashed)
    for poly in &face_uv.inner_polylines {
        if poly.len() < 2 { continue; }
        let mut d = format!("M {:.2} {:.2}", map_u(poly[0].0), map_v(poly[0].1));
        for &(u, v) in &poly[1..] {
            d.push_str(&format!(" L {:.2} {:.2}", map_u(u), map_v(v)));
        }
        d.push_str(" Z");
        svg.push_str(&format!(
            "  <path d=\"{}\" fill=\"none\" stroke=\"#ff4444\" stroke-width=\"1.5\" stroke-dasharray=\"4,2\"/>\n", d
        ));
    }

    // Build outer boundary polygon for point-in-polygon clipping
    let outer_uv_poly: Vec<(f64, f64)> = face_uv.outer_polylines.iter()
        .flat_map(|p| p.iter().copied())
        .collect();

    // ─── UV triangles ────────────────────────────────────────────────
    // Render the actual UV tessellation so saved SVGs match what the
    // interactive viewer shows. Triangles inside holes or outside the
    // outer boundary are drawn in red; valid triangles use a blue tint.
    let hole_polys: &[Vec<(f64, f64)>] = &face_uv.inner_polylines;
    if !face_uv.uv_triangles.is_empty() {
        let tri_limit = 5000.min(face_uv.uv_triangles.len());
        svg.push_str("  <g opacity=\"0.55\">\n");
        for (ti, tri) in face_uv.uv_triangles.iter().enumerate() {
            let cu = (tri[0].0 + tri[1].0 + tri[2].0) / 3.0;
            let cv = (tri[0].1 + tri[1].1 + tri[2].1) / 3.0;
            let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h));
            let in_outer = !outer_uv_poly.is_empty() && point_in_polygon(cu, cv, &outer_uv_poly);
            let (fill, stroke) = if in_hole || !in_outer {
                ("#ff2222", "#ff4444")
            } else if ti % 2 == 0 {
                ("#4488ff", "#4488ff")
            } else {
                ("#55aaff", "#55aaff")
            };
            svg.push_str(&format!(
                "    <polygon points=\"{:.2},{:.2} {:.2},{:.2} {:.2},{:.2}\" fill=\"{}\" fill-opacity=\"0.18\" stroke=\"{}\" stroke-width=\"0.4\" stroke-opacity=\"0.7\"/>\n",
                map_u(tri[0].0), map_v(tri[0].1),
                map_u(tri[1].0), map_v(tri[1].1),
                map_u(tri[2].0), map_v(tri[2].1),
                fill, stroke
            ));
            if ti >= tri_limit { break; }
        }
        svg.push_str("  </g>\n");
    }

    // Surface evaluation points (only those inside the outer boundary)
    if let Some(s) = surface {
        for i in 0..=u_divs {
            for j in 0..=v_divs {
                let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                let p = s.point_at(u, v);
                if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
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
    }

    // Labels
    svg.push_str(&format!(
        "  <text x=\"{}\" y=\"{}\" fill=\"#aaa\" font-size=\"12\" text-anchor=\"middle\">U ({:.2} .. {:.2})</text>\n",
        margin + draw_w / 2.0, svg_height - 5.0, u_min, u_max
    ));
    svg.push_str(&format!(
        "  <text x=\"10\" y=\"{}\" fill=\"#aaa\" font-size=\"12\" text-anchor=\"middle\" transform=\"rotate(-90, 10, {})\">V ({:.2} .. {:.2})</text>\n",
        margin + draw_h / 2.0, margin + draw_h / 2.0, v_min, v_max
    ));
    svg.push_str(&format!(
        "  <text x=\"{}\" y=\"20\" fill=\"#fff\" font-size=\"13\" text-anchor=\"middle\">Face #{} {} forward={} [{}]</text>\n",
        svg_width / 2.0, face_uv.face_idx, face_uv.surface_type, face_uv.forward, model_name
    ));

    svg.push_str("</svg>\n");
    svg
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
        Some(idx) if idx == usize::MAX => " [failed]".to_string(),
        Some(idx) => format!(" [{}]", idx),
        None => String::new(),
    };
    let label = format!("{}{}{}", node.name, brep_str, inst_str);

    // Use instance_index for selection (exact mapping to instance)
    // usize::MAX sentinel means "failed triangulation" — not selectable
    let is_selected = node.instance_index.map_or(false, |idx| idx != usize::MAX && selected_instance == Some(idx));

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
            // usize::MAX sentinel = failed instance, not selectable
            if let Some(idx) = node.instance_index {
                if idx == usize::MAX {
                    // Failed instance — show red X, no toggle
                    ui.label(egui::RichText::new("X").size(11.0).color(egui::Color32::from_rgb(180, 80, 80)));
                } else {
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
            }
            // Selectable label for the instance
            let response = ui.selectable_label(is_selected, egui::RichText::new(&label).size(11.0));
            // Scroll to this node if it's the scroll target
            if scroll_to_tree_node.as_ref() == Some(&key) {
                response.scroll_to_me(Some(egui::Align::Center));
            }
            if response.clicked() {
                // Use instance_index for precise selection
                // Skip failed instances (usize::MAX sentinel)
                if let Some(idx) = node.instance_index {
                    if idx != usize::MAX {
                        *pending_instance_select = Some(idx);
                    }
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

/// Skip the next unassigned leaf node in the assembly tree (DFS order).
/// Called when a BREP instance fails triangulation — we must consume the
/// corresponding tree leaf so that subsequent successful instances get
/// the correct leaf assignment. Without this, failed instances cause
/// leaf-to-instance misalignment (e.g., plate_1's leaf gets nut_3's name).
///
/// We mark the leaf with `instance_index = Some(usize::MAX)` as a sentinel
/// meaning "failed — no graphical instance". The tree display code treats
/// this the same as `None` for selection/highlighting purposes.
fn skip_instance_in_tree(node: &mut AssemblyNode) {
    let mut stack: Vec<&mut AssemblyNode> = vec![node];
    while let Some(n) = stack.pop() {
        if n.children.is_empty() && n.brep_id.is_some() && n.instance_index.is_none() {
            // Mark as failed: use usize::MAX as sentinel
            n.instance_index = Some(usize::MAX);
            return;
        }
        for child in n.children.iter_mut().rev() {
            stack.push(child);
        }
    }
}
