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
use draper_topology::{ShapeBuilder, Solid, Edge, Wire, Face, Shell};
use draper_mesh::{triangulate_solid, triangulate_face, TriangleMesh, TriangulationParams, check_manifold, ManifoldReport, cut_text_holes_in_mesh, TextSurface};
use draper_step::{AssemblyNode, DetailedMeshInstance, PendingBrepInstance, OwnedStepConversionContext, StepFile, step_structure_lazy};
use draper_geometry::Surface;
use draper_geometry::Point3d;
use draper_geometry::NurbsSurface;
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
    hidden_faces: &std::collections::HashSet<(usize, u64)>,
    section_plane: Option<(usize, f32)>,
) -> (Vec<MeshVertex>, Vec<u32>, Vec<(usize, usize)>) {
    // NOTE: compute_face_normals() and ensure_colors() must be called on the mesh
    // BEFORE calling this function, to avoid cloning the entire mesh here.
    let normals = mesh.face_normals.as_ref();
    let colors = mesh.triangle_colors.as_ref();
    let face_ids = mesh.triangle_face_ids.as_ref();

    // Check if we have meaningful per-triangle colors (not all default grey)
    let _has_real_colors = colors.map_or(false, |c| {
        c.iter().any(|col| (col[0] - 0.62).abs() > 0.01 || (col[1] - 0.65).abs() > 0.01 || (col[2] - 0.70).abs() > 0.01)
    });

    // ALWAYS use flat shading with face normals for CAD models.
    // The previous smooth shading path (using vertex normals) produced
    // incorrect results because shared vertices at face boundaries received
    // normals from the wrong face during merge_deduplicating. This caused
    // planar faces to appear curved and lighting to be inconsistent.
    //
    // Flat shading with analytical face_normals gives each triangle a
    // uniform, correct normal that matches the surface geometry exactly.
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
            // Skip triangles belonging to hidden individual faces.
            if let Some(fid) = face_ids.and_then(|ids| ids.get(i)).copied() {
                if hidden_faces.contains(&(idx, fid)) {
                    continue;
                }
            }
        }

        // ─── Section cut: skip triangles behind the plane ───
        // section_plane = (axis, position): keep triangles where ALL vertices
        // are on the positive side (>= position) of the plane.
        if let Some((axis, pos)) = section_plane {
            let pos = pos as f64;
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            let c0 = match axis { 0 => v0.x, 1 => v0.y, _ => v0.z };
            let c1 = match axis { 0 => v1.x, 1 => v1.y, _ => v1.z };
            let c2 = match axis { 0 => v2.x, 1 => v2.y, _ => v2.z };
            // Skip if all 3 vertices are behind the plane (< pos)
            if c0 < pos && c1 < pos && c2 < pos {
                continue;
            }
        }

        // Skip degenerate triangles (zero-area or near-zero-area).
        // These produce NaN/zero-length cross-product normals and render
        // as visual artifacts (flickering, random-color pixels).
        {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
            let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
            let cross_len_sq = (e1.1 * e2.2 - e1.2 * e2.1).powi(2)
                             + (e1.2 * e2.0 - e1.0 * e2.2).powi(2)
                             + (e1.0 * e2.1 - e1.1 * e2.0).powi(2);
            if cross_len_sq < 1e-20 {
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
    /// 3D intersection point in world space.
    point: [f32; 3],
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
/// Hidden instances and hidden individual faces are excluded from picking.
fn pick_at(
    mesh: &TriangleMesh,
    instance_triangle_ranges: &[(usize, usize)],
    hidden_instances: &std::collections::HashSet<usize>,
    hidden_faces: &std::collections::HashSet<(usize, u64)>,
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
        // Skip triangles belonging to hidden individual faces
        if let Some(fid) = face_ids.and_then(|ids| ids.get(i)).copied() {
            if hidden_faces.contains(&(instance_idx, fid)) {
                continue;
            }
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
                // Compute 3D intersection point: origin + t * direction
                let point = [
                    ray_origin[0] + ray_dir[0] * t,
                    ray_origin[1] + ray_dir[1] * t,
                    ray_origin[2] + ray_dir[2] * t,
                ];

                best = Some(PickResult {
                    instance_idx,
                    face_id,
                    distance: t,
                    point,
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
    /// Set of (instance_idx, face_id) pairs for individual faces that are
    /// currently hidden (not rendered). Toggled via per-face eye icon in
    /// the Face List section. Independent from hidden_instances — an
    /// instance can be visible while some of its faces are hidden.
    hidden_faces: std::collections::HashSet<(usize, u64)>,
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
    /// Last successfully-parsed STEP file, kept so the user can re-triangulate
    /// at a different LOD without re-opening the file. When the user changes
    /// the Quality dropdown, `retriangulate_for_lod()` re-runs
    /// `process_step_file` on this saved copy. Cleared whenever a non-STEP
    /// model is loaded (primitive, NURBS gallery, JSON, STL) so we don't
    /// accidentally re-load a stale STEP file.
    last_step_file: Option<StepFile>,
    /// Display name of `last_step_file` (used in log messages and as the
    /// `name` argument to `process_step_file` when re-triangulating).
    last_step_name: String,
    /// Total number of instances being loaded (for progress display).
    total_instance_count: usize,
    /// Number of instances already triangulated.
    triangulated_count: usize,
    /// Faces triangulated in the CURRENT chunked BREP session (for progress display).
    /// Reset to 0 when a BREP completes and the next one starts.
    chunked_brep_faces_done: usize,
    /// Total faces in the current chunked BREP session (for progress display).
    chunked_brep_faces_total: usize,
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
    /// If the LOD was auto-downgraded for mobile, this records the original
    /// level before downgrade (for UI display: "Medium (auto-downgraded from Ultra)").
    /// Cleared when the user manually changes LOD or when a new file is loaded.
    lod_downgraded_from: Option<LodLevel>,

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

    // ─── IndexedDB cache (WASM only) ────────────────────────────────────
    /// Cache manager for triangulation results stored in IndexedDB.
    /// On cache hit, the viewer skips parsing + triangulation entirely
    /// and loads the mesh data directly from the cache.
    #[cfg(target_arch = "wasm32")]
    cache_manager: crate::cache::CacheManager,
    /// Current state of an in-flight cache lookup.
    #[cfg(target_arch = "wasm32")]
    cache_state: crate::cache::CacheState,
    /// SHA-256 hash of the current STEP file content (for cache storage after triangulation).
    #[cfg(target_arch = "wasm32")]
    cache_step_hash: Option<String>,
    /// Whether the current load came from the cache (for UI display).
    #[cfg(target_arch = "wasm32")]
    loaded_from_cache: bool,
    /// Pending STEP file content (held during async cache lookup).
    /// If cache misses, this content is passed to the Worker or main-thread parser.
    #[cfg(target_arch = "wasm32")]
    _pending_step_content: Option<String>,
    /// Pending STEP file name (held during async cache lookup).
    #[cfg(target_arch = "wasm32")]
    _pending_step_name: Option<String>,

    /// Information about a partial result after loading timeout or cancel.
    /// Displayed in the UI Info section to inform the user that the model
    /// is incomplete but intentionally so. Set to Some("...") when timeout
    /// fires or when cancel salvages partial results; cleared on new load.
    partial_result_info: Option<String>,

    // ─── Modeling (editing + boolean + GDT) ────────────────────────────────
    /// The current solid being edited (set whenever a primitive is loaded
    /// or a STEP file with a single solid is imported). Operations like
    /// fillet/chamfer/shell/transform work on this solid.
    current_solid: Option<Solid>,
    /// The current NURBS surface, if a NURBS gallery model is loaded.
    /// Required so that `retriangulate_for_lod()` can rebuild the grid mesh
    /// with LOD-aware step count. `triangulate_solid` does NOT work on a
    /// Solid whose only face is `Face::new_surface_only(...)` (no outer wire),
    /// which is how NURBS gallery surfaces are stored — so we MUST keep the
    /// original surface here and re-run `build_nurbs_surface_mesh` on LOD change.
    /// Cleared to `None` whenever a non-NURBS model is loaded.
    current_nurbs_surface: Option<NurbsSurface>,
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
    /// Whether to show the GD&T (Geometric Dimensioning and Tolerancing) panel.
    show_gdt_window: bool,
    /// Whether to show GD&T annotations overlaid on the 3D viewport.
    show_gdt_annotations: bool,
    /// Index of the face currently selected in the UV window.
    ///
    /// `None` means no face is currently active in the UV window — in that
    /// state the canvas is hidden and a "Select a face" prompt is shown
    /// instead. This keeps the UV Breakdown window consistent with the
    /// structure panel: when the user switches solids or clears the
    /// selection, the previously-shown UV grid becomes stale and would
    /// be misleading, so we hide it until the user picks a face again.
    ///
    /// The face index is positional within the active solid's outer shell
    /// faces array (for primitives/NURBS) or within the selected STEP
    /// instance's `faces` array (for STEP files — the order matches
    /// `solid_from_detailed_instance`).
    uv_window_face_idx: Option<usize>,
    /// U subdivisions for the UV grid display.
    uv_window_u_divs: usize,
    /// V subdivisions for the UV grid display.
    uv_window_v_divs: usize,
    /// Zoom level for the UV breakdown window canvas.
    /// 1.0 = auto-fit (whole UV domain visible). Higher = zoomed in.
    /// Range: 0.1 .. 50.0. User-adjustable via slider, scroll wheel, or
    /// the "Reset View" button (which restores 1.0).
    uv_window_zoom: f32,
    /// Pan offset in UV space for the UV breakdown window canvas.
    /// [0] = u-pan, [1] = v-pan (in UV units, not screen pixels).
    /// The center of the visible region is shifted by this offset
    /// before zoom is applied. Reset to [0.0, 0.0] on face change.
    uv_window_pan: [f64; 2],
    /// Aspect-ratio override for the UV breakdown window canvas.
    /// 1.0 = preserve the UV box's natural aspect ratio (default).
    /// <1.0 = squeeze U (e.g. 0.5 makes a 2π×1 cylinder UV appear square).
    /// >1.0 = stretch U.
    /// Useful when the U range is much larger than V (e.g. cylinder: U=2π,
    /// V=height) — without an override the canvas becomes wide and short,
    /// making individual triangles hard to see. User-adjustable via slider
    /// in the UV window controls.
    uv_window_aspect_override: f32,
    /// Whether to display UV in metric-correct proportions (U scaled by
    /// radius for cylinders/cones, etc.). When false, shows raw parameter
    /// space (U in radians for rotational surfaces).
    uv_window_metric_uv: bool,
    /// Last face index shown in the UV window — used to detect face
    /// switches and reset zoom/pan automatically.
    uv_window_prev_face_idx: Option<usize>,
    /// Cached UV breakdown for the current solid — recomputed whenever the
    /// solid changes or when the user clicks "View UV" / "Save UV SVG".
    /// Keyed by the solid's face count + a generation counter so stale
    /// cache from a previous solid is never shown.
    solid_uv_breakdown: Option<SolidUvBreakdown>,
    /// Set when the user clicked "Save UV SVG" — the next frame will
    /// trigger a file dialog (native) or browser download (WASM) of the
    /// SVG of the currently-selected face in the UV window.
    pending_solid_uv_svg_export: bool,

    // ─── GD&T (Geometric Dimensioning and Tolerancing) ────────────────
    /// Cached GD&T data extracted from the current STEP file.
    /// Populated when the user opens the GD&T window or when a new file
    /// is loaded.
    gdt_data: Option<draper_step::GdtData>,

    // ─── BRepCAD UI (extended menu/ribbon) ────────────────────────────
    /// When true, the BRepCAD 21-menu + 15-tab ribbon is rendered at the top
    /// instead of the simple "File / View" menu. Set by the brepcad-shell binary.
    pub enable_brepcad_ui: bool,
    /// Active ribbon tab for BRepCAD UI.
    pub brepcad_ribbon_tab: crate::ui::ribbon::RibbonTab,
    /// Active dialog for BRepCAD UI.
    pub brepcad_dialog: crate::ui::dialogs::DialogType,
    /// Command palette state for BRepCAD UI.
    pub brepcad_command_palette: crate::ui::command_palette::CommandPalette,
    /// Marking menu visibility for BRepCAD UI.
    pub brepcad_marking_menu_visible: bool,
    /// Last status message displayed in toast.
    pub brepcad_status_msg: String,
    /// Left panel active tab (Tree/Layers/Selection).
    pub brepcad_left_tab: BrepcadLeftTab,
    /// Right panel active tab (Properties/Constraints/Dimensions/Material).
    pub brepcad_right_tab: BrepcadRightTab,
    /// Tree filter text.
    pub brepcad_tree_filter: String,
    /// Active tool name (shown in status bar).
    pub brepcad_active_tool: String,
    /// Current view orientation label (shown in status bar).
    pub brepcad_view_orientation: String,
    /// Undo stack: snapshots of (solid, model_name) before each mutation.
    pub brepcad_undo_stack: Vec<(Option<draper_topology::Solid>, String)>,
    /// Redo stack: snapshots for redo.
    pub brepcad_redo_stack: Vec<(Option<draper_topology::Solid>, String)>,
    /// Max undo history depth.
    pub brepcad_max_history: usize,
    /// Measure mode: None / Distance / Angle
    pub brepcad_measure_mode: BrepcadMeasureMode,
    /// First picked point for measurement (3D world space).
    pub brepcad_measure_point1: Option<[f32; 3]>,
    /// Second picked point for measurement.
    pub brepcad_measure_point2: Option<[f32; 3]>,
    /// Third picked point (for angle measurement: vertex + 2 points).
    pub brepcad_measure_point3: Option<[f32; 3]>,
    /// Last measurement result string.
    pub brepcad_measure_result: String,
    /// Section cut: enabled?
    pub brepcad_section_enabled: bool,
    /// Section cut: plane axis (0=X, 1=Y, 2=Z).
    pub brepcad_section_axis: u8,
    /// Section cut: plane position along axis.
    pub brepcad_section_position: f32,
    /// Parameter table: HashMap<name, (value, formula, unit)>.
    pub brepcad_parameters: std::collections::HashMap<String, (f64, Option<String>, String)>,
    /// Parameter dialog visible?
    pub brepcad_param_dialog_open: bool,
    /// New parameter name input.
    pub brepcad_new_param_name: String,
    /// New parameter formula input.
    pub brepcad_new_param_formula: String,
    /// Feature timeline: list of (operation_name, solid_snapshot).
    pub brepcad_timeline: Vec<(String, Option<draper_topology::Solid>)>,
    /// Timeline panel visible?
    pub brepcad_timeline_open: bool,
    /// Current rollback position (index into timeline). None = latest.
    pub brepcad_timeline_rollback: Option<usize>,
}

/// Measure mode for the BRepCAD viewport.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BrepcadMeasureMode {
    #[default]
    None,
    Distance,
    Angle,
    Length,
}

/// Left panel tab for BRepCAD Browser.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrepcadLeftTab {
    Tree,
    Layers,
    Selection,
}

impl Default for BrepcadLeftTab {
    fn default() -> Self { BrepcadLeftTab::Tree }
}

/// Right panel tab for BRepCAD Properties.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrepcadRightTab {
    Properties,
    Constraints,
    Dimensions,
    Material,
}

impl Default for BrepcadRightTab {
    fn default() -> Self { BrepcadRightTab::Properties }
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
    /// Whether the surface is periodic in U (cone, cylinder, sphere, torus,
    /// revolution, or NURBS with u_closed). When true, the U=u_min edge of
    /// the UV domain is the same physical line as U=u_max — the "seam".
    u_periodic: bool,
    /// Whether the surface is periodic in V (sphere, torus, or NURBS with
    /// v_closed). When true, the V=v_min edge is the same as V=v_max.
    v_periodic: bool,
    /// U period (only meaningful when u_periodic). Typically 2π.
    /// Used to compute the seam location and to unwrap seam-crossing
    /// triangles. Zero when not u_periodic.
    u_period: f64,
    /// V period (only meaningful when v_periodic). π for sphere, 2π for
    /// torus. Zero when not v_periodic.
    v_period: f64,
    /// Metric scale factors for displaying the UV domain in correct surface
    /// proportions. For a cylinder with radius R, u_metric_scale = R so that
    /// the U axis (normally in radians) is displayed in arc-length units (mm),
    /// making the UV rectangle show the true surface shape.
    u_metric_scale: f64,
    v_metric_scale: f64,
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

    /// Parse a LOD level from its label string (used for localStorage restore).
    fn from_label(label: &str) -> Option<LodLevel> {
        match label {
            "Preview" => Some(LodLevel::Preview),
            "Low" => Some(LodLevel::Low),
            "Medium" => Some(LodLevel::Medium),
            "High" => Some(LodLevel::High),
            "Ultra" => Some(LodLevel::Ultra),
            _ => None,
        }
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
            let (vertices, indices, _new_ranges) = mesh_to_gpu_data(&mesh, None, None, &[], &std::collections::HashSet::new(), &std::collections::HashSet::new(), None);
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
            highlighted_face: None,
            highlight_dirty: false,
            instance_triangle_ranges: Vec::new(),
            hidden_instances: std::collections::HashSet::new(),
            hidden_faces: std::collections::HashSet::new(),
            open_tree_nodes: std::collections::HashSet::new(),
            scroll_to_tree_node: None,
            scroll_to_face_id: None,
            pending_breps: Vec::new(),
            conversion_ctx: None,
            pending_step_file: None,
            last_step_file: None,
            last_step_name: String::new(),
            total_instance_count: 0,
            triangulated_count: 0,
            chunked_brep_faces_done: 0,
            chunked_brep_faces_total: 0,
            is_loading: false,
            loading_name: String::new(),
            loading_start: None,
            manifold_report,
            controls_panel_open: true,
            log_panel_open: true,
            structure_tree_open: true,
            face_list_open: true,
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
            lod_level: {
                // Restore LOD from localStorage if available (WASM only).
                // This prevents "quality jumping" between sessions on mobile.
                #[cfg(target_arch = "wasm32")]
                {
                    Self::load_lod_from_local_storage().unwrap_or(LodLevel::High)
                }
                #[cfg(not(target_arch = "wasm32"))]
                LodLevel::High
            },
            lod_downgraded_from: None,
            #[cfg(target_arch = "wasm32")]
            use_worker: {
                // Try to initialize the Worker on startup. If it fails
                // (CSP restrictions, old browser, etc.), fall back to
                // main-thread chunked processing.
                let worker_ok = Self::try_init_worker();
                worker_ok
            },
            #[cfg(target_arch = "wasm32")]
            worker_ready: false, // Will be set to true when Worker posts 'ready'
            #[cfg(target_arch = "wasm32")]
            worker_pending_meshes: Vec::new(),
            #[cfg(target_arch = "wasm32")]
            cache_manager: crate::cache::CacheManager::new(),
            #[cfg(target_arch = "wasm32")]
            cache_state: crate::cache::CacheState::Idle,
            #[cfg(target_arch = "wasm32")]
            cache_step_hash: None,
            #[cfg(target_arch = "wasm32")]
            loaded_from_cache: false,
            #[cfg(target_arch = "wasm32")]
            _pending_step_content: None,
            #[cfg(target_arch = "wasm32")]
            _pending_step_name: None,
            partial_result_info: None,
            current_solid: Some(solid_clone_for_field),
            current_nurbs_surface: None,
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
            show_gdt_window: false,
            show_gdt_annotations: false,
            uv_window_face_idx: None,
            uv_window_u_divs: 10,
            uv_window_v_divs: 10,
            uv_window_zoom: 1.0,
            uv_window_pan: [0.0, 0.0],
            uv_window_aspect_override: 1.0,
            uv_window_metric_uv: true, // default: show metric-correct UV
            uv_window_prev_face_idx: None,
            solid_uv_breakdown: None,
            pending_solid_uv_svg_export: false,
            gdt_data: None,
            enable_brepcad_ui: false,
            brepcad_ribbon_tab: crate::ui::ribbon::RibbonTab::Home,
            brepcad_dialog: crate::ui::dialogs::DialogType::None,
            brepcad_command_palette: Default::default(),
            brepcad_marking_menu_visible: false,
            brepcad_status_msg: String::new(),
            brepcad_left_tab: BrepcadLeftTab::Tree,
            brepcad_right_tab: BrepcadRightTab::Properties,
            brepcad_tree_filter: String::new(),
            brepcad_active_tool: "Select".to_string(),
            brepcad_view_orientation: "ISO".to_string(),
            brepcad_undo_stack: Vec::new(),
            brepcad_redo_stack: Vec::new(),
            brepcad_max_history: 50,
            brepcad_measure_mode: BrepcadMeasureMode::None,
            brepcad_measure_point1: None,
            brepcad_measure_point2: None,
            brepcad_measure_point3: None,
            brepcad_measure_result: String::new(),
            brepcad_section_enabled: false,
            brepcad_section_axis: 2, // Z axis by default
            brepcad_section_position: 0.0,
            brepcad_parameters: std::collections::HashMap::new(),
            brepcad_param_dialog_open: false,
            brepcad_new_param_name: String::new(),
            brepcad_new_param_formula: String::new(),
            brepcad_timeline: Vec::new(),
            brepcad_timeline_open: false,
            brepcad_timeline_rollback: None,
        };
        app.log(&format!("3Draper Viewer started [build: {}]", env!("DRAPER_GIT_HASH")));
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
        // Invalidate the per-solid UV breakdown cache so the next time the
        // UV window is opened we recompute from the freshly-loaded solid.
        self.solid_uv_breakdown = None;
        // Reset the active face in the UV window — the previous solid's
        // face index is meaningless for the new solid, so we hide the
        // canvas until the user picks a face again (either from the
        // structure panel or from the UV window's own face list).
        self.uv_window_face_idx = None;
        self.uv_window_prev_face_idx = None;
        self.open_tree_nodes.clear();
        self.scroll_to_tree_node = None;
        self.scroll_to_face_id = None;
        self.hidden_instances.clear();
        self.hidden_faces.clear();
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

    /// Store the current triangulation result in the IndexedDB cache.
    /// Called after triangulation completes (WASM only, fire-and-forget).
    /// Only stores if we have STEP content and the data didn't come from
    /// the cache already (no need to re-store what we just loaded).
    #[cfg(target_arch = "wasm32")]
    fn cache_step_result(&self) {
        if self.loaded_from_cache {
            return; // Already from cache, no need to re-store
        }
        let content = match &self._pending_step_content {
            Some(c) => c,
            None => return, // No content available
        };
        let name = match &self._pending_step_name {
            Some(n) => n,
            None => return,
        };
        let assembly_tree = match &self.assembly_tree {
            Some(tree) => tree,
            None => return,
        };
        if self.detailed_instances.is_empty() {
            return; // Nothing to cache
        }

        let lod = self.lod_level.lod_value();
        self.cache_manager.store_result(
            content,
            name,
            lod,
            &self.mesh,
            &self.detailed_instances,
            assembly_tree,
        );
    }

    fn load_box(&mut self) {
        let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Box 100x80x60");
    }

    fn load_cylinder(&mut self) {
        let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cylinder R=40 H=100");
    }

    fn load_sphere(&mut self) {
        let solid = ShapeBuilder::make_sphere(50.0);
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
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
        self.current_nurbs_surface = None;
        self.detailed_instances.clear();
        self.instance_triangle_ranges.clear();
        self.assembly_tree = None;
        self.load_mesh(mesh, "Cone R=40 H=80");
    }

    fn load_torus(&mut self) {
        let solid = ShapeBuilder::make_torus(40.0, 12.0);
        let mesh = triangulate_solid(&solid, &tri_params_for_lod(self.lod_level));
        self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
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

    /// Re-triangulate the currently-loaded model at the current LOD level.
    ///
    /// Called when the user changes the Quality dropdown. Three cases:
    ///
    /// 1. **NURBS gallery** (`current_nurbs_surface` is set): re-runs
    ///    `build_nurbs_surface_mesh` with `steps_for_lod()`. This is needed
    ///    because NURBS gallery surfaces are stored as a `Solid` with a single
    ///    `Face::new_surface_only(...)` (no outer wire), and `triangulate_solid`
    ///    returns an EMPTY mesh for such faces (no boundary for earcutr).
    ///    Without this branch, switching LOD on a NURBS surface would log
    ///    "0 vertices, 0 triangles" and show the old mesh.
    ///
    /// 2. **Primitive** (`current_solid` is set, no NURBS): re-runs
    ///    `triangulate_solid` with the new `TriangulationParams::for_lod(...)`.
    ///    Fast (single solid, no I/O), structure is just "1 solid" so nothing
    ///    to preserve. Selection is reset by `load_mesh` (primitives have no
    ///    STEP face IDs anyway, so this is fine).
    ///
    /// 3. **STEP file** (`last_step_file` is set): re-runs `process_step_file`
    ///    on the saved StepFile with the new LOD. This rebuilds the assembly
    ///    tree (identical, since same file) and re-queues progressive
    ///    triangulation. Selection state (`selected_instance`,
    ///    `selected_face`, `highlighted_face`, `open_tree_nodes`) is saved
    ///    before and restored after — the instance_idx and face_id are stable
    ///    across re-triangulation because they come from the STEP file, not
    ///    from the triangulation. The highlight won't render until each
    ///    instance is re-triangulated, but it auto-fixes as loading
    ///    progresses.
    ///
    /// If neither `current_solid` nor `last_step_file` is set, logs a warning.
    fn retriangulate_for_lod(&mut self) {
        if let Some(nurbs_surface) = self.current_nurbs_surface.clone() {
            // NURBS gallery surface — re-build grid mesh with LOD-aware steps.
            let name = self.current_model.name.clone();
            let steps = self.steps_for_lod();
            let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
            self.current_solid = Some(nurbs_solid);
            self.detailed_instances.clear();
            self.instance_triangle_ranges.clear();
            self.assembly_tree = None;
            self.load_mesh(mesh, &name);
            self.log(&format!(
                "Re-triangulated NURBS surface at LOD={} ({:.2}) — {} grid steps → {} vertices, {} triangles",
                self.lod_level.label(),
                self.lod_level.lod_value(),
                steps,
                self.current_model.vertex_count,
                self.current_model.triangle_count,
            ));
        } else if self.current_solid.is_some() {
            // Primitive — fast path, re-triangulate in-place.
            let name = self.current_model.name.clone();
            self.refresh_from_current_solid(&name);
            self.log(&format!(
                "Re-triangulated primitive at LOD={} ({:.2}) — {} vertices, {} triangles",
                self.lod_level.label(),
                self.lod_level.lod_value(),
                self.current_model.vertex_count,
                self.current_model.triangle_count,
            ));
        } else if let Some(step_file) = self.last_step_file.clone() {
            // STEP file — save selection, re-run process_step_file, restore.
            let saved_selected_instance = self.selected_instance;
            let saved_selected_face = self.selected_face;
            let saved_highlighted_face = self.highlighted_face;
            let saved_open_tree_nodes = self.open_tree_nodes.clone();
            let name = self.last_step_name.clone();
            self.log(&format!(
                "Re-triangulating STEP file '{}' at LOD={} ({:.2}) — {} instances queued...",
                name,
                self.lod_level.label(),
                self.lod_level.lod_value(),
                self.total_instance_count
            ));
            self.process_step_file(&step_file, &name);
            // Restore selection. The instance_idx and face_id are stable
            // across re-triangulation (they come from the STEP file, not
            // from the triangulation). `detailed_instances` is rebuilt
            // progressively, so the highlight won't render until each
            // instance is re-triangulated — but it auto-fixes as loading
            // progresses.
            self.selected_instance = saved_selected_instance;
            self.selected_face = saved_selected_face;
            self.highlighted_face = saved_highlighted_face;
            self.open_tree_nodes = saved_open_tree_nodes;
            self.highlight_dirty = true;
        } else {
            self.log_warning(
                "No solid loaded — load a primitive or STEP file before changing LOD",
            );
        }
    }

    // ─── localStorage helpers for LOD persistence (WASM only) ────────────

    /// Save the current LOD level to localStorage so it persists across sessions.
    /// Key: `3draper_mobile_lod`. Value: LOD label string ("Low", "Medium", etc.)
    #[cfg(target_arch = "wasm32")]
    fn save_lod_to_local_storage(&self) {
        let label = self.lod_level.label();
        let js = format!(
            "try {{ localStorage.setItem('3draper_mobile_lod', '{}'); }} catch(e) {{}}"
            , label
        );
        let _ = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.default_view())
            .and_then(|w| js_sys::Function::new_no_args(&js).call0(&w.into()).ok());
    }

    /// Load the previously saved LOD level from localStorage.
    /// Returns None if localStorage is unavailable or no saved value exists.
    #[cfg(target_arch = "wasm32")]
    fn load_lod_from_local_storage() -> Option<LodLevel> {
        let js = "try { return localStorage.getItem('3draper_mobile_lod') || ''; } catch(e) { return ''; }";
        let result = web_sys::window()
            .and_then(|w| js_sys::Function::new_no_args(js).call0(&w.into()).ok())
            .and_then(|v| v.as_string());
        result.as_deref().and_then(LodLevel::from_label)
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
        self.current_nurbs_surface = None;
                self.refresh_from_current_solid(&format!("Fillet r={} on edge {}", radius, actual_edge_id));
            }
            Err(e) => {
                self.log_warning(&format!("Fillet failed: {}", e));
                self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
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
        self.current_nurbs_surface = None;
                self.refresh_from_current_solid(&format!("Chamfer d={} on edge {}", distance, actual_edge_id));
            }
            Err(e) => {
                self.log_warning(&format!("Chamfer failed: {}", e));
                self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
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
        self.current_nurbs_surface = None;
                self.refresh_from_current_solid(&format!("Shell thickness={}", thickness));
            }
            Err(e) => {
                self.log_warning(&format!("Shell failed: {}", e));
                self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
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
        self.current_nurbs_surface = None;
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
        // IMPORTANT: assign current_solid BEFORE load_mesh so the UV
        // breakdown window operates on THIS revolution surface, not the
        // previously loaded solid. Without this, "View UV" after loading
        // a revolution would show stale UV data from the prior solid.
        self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
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
        // IMPORTANT: assign current_solid BEFORE load_mesh so the UV
        // breakdown window operates on THIS surface, not the previously
        // loaded one. Without this assignment, opening "View UV" after
        // loading an extrusion would show stale UV data from whatever
        // solid was loaded before (e.g. Box or Cylinder).
        self.current_solid = Some(solid);
        self.current_nurbs_surface = None;
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

    /// Map a LOD level to a NURBS grid subdivision count.
    ///
    /// NURBS gallery surfaces are rendered as a regular (n+1)×(n+1) UV grid
    /// with 2 triangles per cell. The step count scales with LOD so that:
    /// - Preview (LOD 0.1): 6 steps → 49 vertices, 72 triangles
    /// - Low     (LOD 0.3): 12 steps → 169 vertices, 288 triangles
    /// - Medium  (LOD 0.5): 20 steps → 441 vertices, 800 triangles
    /// - High    (LOD 0.75): 32 steps → 1089 vertices, 2048 triangles
    /// - Ultra   (LOD 1.0): 48 steps → 2401 vertices, 4608 triangles
    ///
    /// This gives a ~64× triangle ratio between Preview and Ultra — clearly
    /// visible to the user as a quality difference.
    fn steps_for_lod(&self) -> usize {
        let v = self.lod_level.lod_value();
        if v >= 0.9 {
            48
        } else if v >= 0.7 {
            32
        } else if v >= 0.45 {
            20
        } else if v >= 0.2 {
            12
        } else {
            6
        }
    }

    /// Helper: build a NURBS surface mesh from a 2D grid of control points.
    ///
    /// Samples the surface on a regular (steps+1)×(steps+1) UV grid and
    /// creates two triangles per grid cell. This direct grid-sampling
    /// approach is used INSTEAD of the complex
    /// `triangulate_face_with_boundary_and_holes_uv` path because the
    /// latter produces broken meshes for NURBS surfaces (162 boundary
    /// edges for 160 triangles — i.e. the triangles are barely connected
    /// to each other, producing the "torn, chaotic" appearance the user
    /// reported). A regular grid mesh has only the perimeter as boundary
    /// edges (4*steps edges), which is correct for an open surface.
    ///
    /// Returns BOTH the triangle mesh AND a `Solid` containing the NURBS
    /// surface as a single face. The solid is needed so that the UV
    /// breakdown window (which reads from `self.current_solid`) shows the
    /// correct surface — without it, the UV window would display stale
    /// data from whatever solid was previously loaded. The face is built
    /// with `Face::new_surface_only` (no outer wire), which causes
    /// `compute_solid_uv_breakdown` to fall back to the surface's
    /// `natural_uv_domain()` and render the full UV grid.
    fn build_nurbs_surface_mesh(
        &self,
        nurbs_surface: NurbsSurface,
        steps: usize,
    ) -> (TriangleMesh, Solid) {
        let (u_min, u_max) = nurbs_surface.u_range();
        let (v_min, v_max) = nurbs_surface.v_range();
        // Clone the surface for the Solid we'll return at the end. The
        // original `nurbs_surface` is consumed by `Surface::Nurbs(...)` below.
        let surface_for_solid = Surface::Nurbs(nurbs_surface.clone());
        let surface = Surface::Nurbs(nurbs_surface);

        let n = steps.max(2); // at least 2 subdivisions → 2×2 cells → 8 triangles
        let du = (u_max - u_min) / n as f64;
        let dv = (v_max - v_min) / n as f64;

        let mut mesh = TriangleMesh::new();
        // Sample (n+1)×(n+1) vertices on the regular UV grid.
        // Vertex index = j*(n+1) + i, where i ∈ [0..=n] is the U index
        // and j ∈ [0..=n] is the V index.
        mesh.vertices.reserve((n + 1) * (n + 1));
        for j in 0..=n {
            for i in 0..=n {
                let u = u_min + i as f64 * du;
                let v = v_min + j as f64 * dv;
                mesh.vertices.push(surface.point_at(u, v));
            }
        }
        // Create two triangles per grid cell. The winding (v00, v10, v11)
        // and (v00, v11, v01) produces counter-clockwise triangles when
        // viewed from the +normal side of a standard UV→3D mapping
        // (U increases to the right, V increases upward), which matches
        // the `forward = true` convention used for face normals.
        let row_stride = (n + 1) as u32;
        mesh.triangles.reserve(n * n * 2);
        for j in 0..n as u32 {
            for i in 0..n as u32 {
                let v00 = j * row_stride + i;
                let v10 = v00 + 1;
                let v01 = v00 + row_stride;
                let v11 = v01 + 1;
                mesh.triangles.push([v00, v10, v11]);
                mesh.triangles.push([v00, v11, v01]);
            }
        }

        // Build a Solid containing the NURBS surface as a single face.
        // The face has no outer wire (Face::new_surface_only), so
        // compute_solid_uv_breakdown will use the surface's natural UV
        // domain as the visible bounds — perfect for showing the full
        // UV grid of the NURBS patch.
        let face = Face::new_surface_only(surface_for_solid);
        let shell = Shell::new(vec![face]);
        let solid = Solid::new(shell);
        (mesh, solid)
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
        // Save the surface so `retriangulate_for_lod()` can rebuild
        // the grid mesh with a different LOD step count.
        self.current_nurbs_surface = Some(nurbs_surface.clone());
        let steps = self.steps_for_lod();
        let (mesh, nurbs_solid) = self.build_nurbs_surface_mesh(nurbs_surface, steps);
        self.current_solid = Some(nurbs_solid);
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
                layers: Vec::new(),
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
                layers: Vec::new(),
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
                        // JSON-loaded models have no `Solid` representation —
                        // only `detailed_instances`. Clear `current_solid`
                        // so the UV Breakdown window uses the JSON instances
                        // instead of leaking a stale primitive/NURBS solid.
                        self.current_solid = None;
                        self.current_nurbs_surface = None;
                        self.solid_uv_breakdown = None;
                        self.uv_window_face_idx = None;
                        self.uv_window_prev_face_idx = None;
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
                // JSON-loaded models have no `Solid` representation —
                // only `detailed_instances`. Clear `current_solid`
                // so the UV Breakdown window uses the JSON instances
                // instead of leaking a stale primitive/NURBS solid.
                self.current_solid = None;
                self.current_nurbs_surface = None;
                self.solid_uv_breakdown = None;
                self.uv_window_face_idx = None;
                self.uv_window_prev_face_idx = None;
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
        // Cancel any previous loading. Pass `false` because the user is
        // opening a NEW file — they don't want to see the OLD partial result.
        self.cancel_loading(false);

        // ─── Clear `current_solid` so the UV Breakdown window falls through
        // to the `solid_from_detailed_instance` path. Without this, a stale
        // `current_solid` from a previously-loaded primitive/NURBS gallery
        // shape would leak into the UV Breakdown window: the dropdown would
        // show the old solid's faces (e.g. "#0 Revolution") instead of the
        // STEP file's faces, and structure-panel face clicks would set
        // `uv_window_face_idx` to a positional index that's out of range for
        // the stale solid, leaving the canvas blank. STEP files have NO
        // `Solid` representation in the viewer — only `detailed_instances`
        // — so `current_solid` MUST be `None` for them.
        self.current_solid = None;
        self.current_nurbs_surface = None; // STEP files are not NURBS gallery surfaces.
        // Also invalidate any cached UV breakdown + active face index —
        // they belong to the previous solid and are meaningless for the
        // new STEP file. (load_mesh does this too, but load_mesh is only
        // called at the END of triangulation, so without this line the UV
        // window would show stale content during the triangulation window.)
        self.solid_uv_breakdown = None;
        self.uv_window_face_idx = None;
        self.uv_window_prev_face_idx = None;

        // Keep a clone of the StepFile + name so the user can re-triangulate
        // at a different LOD via the Quality dropdown without re-opening the
        // file. This is consumed by `retriangulate_for_lod()`.
        self.last_step_file = Some(step_file.clone());
        self.last_step_name = name.to_string();

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
            // ─── Mobile: auto-downgrade LOD for faster loading ───────────────
            // Mobile CPUs are 2-4× slower than desktop, and users have less
            // patience for long loads. If we're on mobile and the user hasn't
            // manually lowered the quality, drop by ONE level (Ultra→High,
            // High→Medium) to speed up loading by ~2× without making the mesh
            // look like a "low-poly preview".
            //
            // Previous strategy (Ultra→Medium, High→Low) was too aggressive —
            // user reported "сильно хуже чем было раньше" (much worse than before).
            // With the new SteinerBudgetProfile system (Mobile profile caps grid
            // at 32×16 instead of 96×64), the Steiner grid is already much
            // coarser on mobile, so we don't need to also aggressively drop LOD.
            //
            // For very large files (>2000 faces estimated) we still drop by TWO
            // levels because the per-face work compounds. We approximate the
            // face count from the number of BREPs: each BREP typically has
            // 100-500 faces, so 5 BREPs ≈ 2500 faces.
            let total_faces_estimate: usize = pending.len() * 500;
            if self.is_mobile {
                let new_lod = if total_faces_estimate > 2000 {
                    // Very large file — drop by 2 levels
                    match self.lod_level {
                        LodLevel::Ultra => Some(LodLevel::Medium),
                        LodLevel::High => Some(LodLevel::Low),
                        LodLevel::Medium => Some(LodLevel::Low),
                        _ => None,
                    }
                } else {
                    // Normal file — drop by 1 level only
                    match self.lod_level {
                        LodLevel::Ultra => Some(LodLevel::High),
                        LodLevel::High => Some(LodLevel::Medium),
                        _ => None,
                    }
                };
                if let Some(new) = new_lod {
                    self.log(&format!(
                        "Mobile detected (faces≈{}) — auto-lowering quality from {} to {} for faster loading (raise manually after load)",
                        total_faces_estimate,
                        self.lod_level.label(),
                        new.label()
                    ));
                    self.lod_downgraded_from = Some(self.lod_level);
                    self.lod_level = new;
                    // Persist the downgraded LOD to localStorage
                    #[cfg(target_arch = "wasm32")]
                    self.save_lod_to_local_storage();
                }
            }

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
                // Skip edges of individually-hidden faces
                if self.hidden_faces.contains(&(inst_idx, face.face_id)) {
                    continue;
                }
                // Outer boundary polylines
                for polyline in &face.outer_boundary {
                    if polyline.len() < 2 {
                        continue;
                    }
                    // Draw ALL boundary points — no downsampling.
                    // The boundary points come from the edge cache with the
                    // same LOD-dependent density as the mesh, so edges and
                    // mesh triangles are always consistent.
                    let mut prev_pos: Option<[f32; 3]> = None;
                    for p in polyline.iter() {
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
                    // Draw ALL boundary points — no downsampling.
                    let mut prev_pos: Option<[f32; 3]> = None;
                    for p in polyline.iter() {
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
        let hidden_faces = &self.hidden_faces;
        let ranges = &self.instance_triangle_ranges;
        let face_ids = mesh.triangle_face_ids.as_ref();

        // Deduplicate edges: each shared edge between adjacent triangles is
        // drawn only once instead of twice. This cuts the vertex count by ~50%
        // for manifold meshes and makes the overlay work for much larger models.
        let mut edge_set: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();

        for (i, tri) in mesh.triangles.iter().enumerate() {
            // Check if this triangle belongs to a hidden instance or hidden face
            let mut is_hidden = false;
            let mut tri_inst_idx: Option<usize> = None;
            for (idx, &(start, end)) in ranges.iter().enumerate() {
                if i >= start && i < end {
                    tri_inst_idx = Some(idx);
                    if hidden.contains(&idx) {
                        is_hidden = true;
                    }
                    break;
                }
            }
            if is_hidden {
                continue;
            }
            // Skip triangles belonging to individually-hidden faces
            if let Some(idx) = tri_inst_idx {
                if let Some(fid) = face_ids.and_then(|ids| ids.get(i)).copied() {
                    if hidden_faces.contains(&(idx, fid)) {
                        continue;
                    }
                }
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
    ///
    /// If `show_partial` is true and we already have some triangulated
    /// instances, commit them to the viewer as a partial result so the
    /// user can see what was loaded (instead of getting a blank screen).
    /// This is critical on mobile where users often want to bail out of
    /// a long load and still see something useful.
    fn cancel_loading(&mut self, show_partial: bool) {
        let had_mesh = self.mesh.vertex_count() > 0;
        let instance_count = self.detailed_instances.len();
        self.is_loading = false;
        self.loading_start = None;

        // Cancel Worker if active
        #[cfg(target_arch = "wasm32")]
        self.worker_cancel();

        // CRITICAL: Before clearing pending_breps, salvage any partial
        // triangulation from the active chunked session. This is symmetric
        // with the timeout handler — if the user clicked Cancel during a
        // long-running BREP, the partial work should still appear.
        let mut salvaged_count = 0usize;
        if show_partial && !self.pending_breps.is_empty() {
            let pending_front = self.pending_breps[0].clone();
            if let Some(ref mut ctx) = self.conversion_ctx {
                if let Some((partial_mesh, partial_faces, faces_done, faces_total)) =
                    ctx.take_partial_active_session(&pending_front)
                {
                    if partial_mesh.triangle_count() > 0 {
                        let tri_start = self.mesh.triangle_count();
                        let color = pending_front.color.unwrap_or_else(|| {
                            Self::instance_color(self.triangulated_count)
                        });
                        self.mesh.merge_with_color(&partial_mesh, color);
                        let tri_end = self.mesh.triangle_count();
                        self.instance_triangle_ranges.push((tri_start, tri_end));
                        let inst_idx = self.instance_triangle_ranges.len() - 1;
                        if let Some(ref mut tree) = self.assembly_tree {
                            assign_instance_to_tree(tree, inst_idx);
                        }
                        self.detailed_instances.push(DetailedMeshInstance {
                            name: pending_front.name.clone(),
                            mesh: partial_mesh,
                            color: pending_front.color,
                            transform: pending_front.transform,
                            brep_id: pending_front.brep_id,
                            faces: partial_faces,
                        });
                        self.triangulated_count += 1;
                        salvaged_count = 1;
                        self.log_warning(&format!(
                            "BREP #{} '{}' — salvaged PARTIAL triangulation on cancel: {}/{} faces ({}%), {} triangles",
                            pending_front.brep_id, pending_front.name,
                            faces_done, faces_total,
                            if faces_total > 0 { (faces_done * 100) / faces_total } else { 0 },
                            tri_end - tri_start
                        ));
                    }
                }
            }
        }

        self.pending_breps.clear();
        // Abort any remaining chunked BREP session (already salvaged above if show_partial).
        if let Some(ref mut ctx) = self.conversion_ctx {
            ctx.abort_active_session();
        }
        // Drop the conversion context and step file to free WASM memory.
        // Without this, repeated file loads accumulate data in the WASM heap
        // (which has a 4GB limit in Chrome) and eventually crash the tab.
        self.conversion_ctx = None;
        self.pending_step_file = None;
        self.chunked_brep_faces_done = 0;
        self.chunked_brep_faces_total = 0;
        #[cfg(target_arch = "wasm32")]
        self.worker_pending_meshes.clear();
        // Note: do NOT reset triangulated_count / total_instance_count here —
        // they are used by the partial-result message below. They get reset
        // on the next load_step_file call.

        if show_partial && (had_mesh || salvaged_count > 0) {
            // Commit partial result so the user sees what was loaded.
            let shown_instances = instance_count + salvaged_count;
            let face_count = self.mesh.triangle_count() / 3;
            self.log_warning(&format!(
                "Loading canceled — showing partial result ({} instance(s), {} triangles)",
                shown_instances,
                self.mesh.triangle_count()
            ));
            self.partial_result_info = Some(format!(
                "Canceled — {}/{} instances, {} faces triangulated",
                shown_instances,
                self.total_instance_count.max(shown_instances),
                face_count
            ));
            self.load_mesh(self.mesh.clone(), &format!("STEP (partial): {}", self.loading_name));
        } else {
            self.log("Loading canceled");
            self.triangulated_count = 0;
            self.total_instance_count = 0;
        }
        self.loading_name.clear();
    }

    // ─── Worker initialization and JS interop (WASM only) ────────────────

    /// Try to initialize the Web Worker via the JS bridge.
    ///
    /// Returns `true` if the Worker was created successfully, `false` if
    /// the browser doesn't support Workers or the JS bridge is unavailable
    /// (e.g., CSP restrictions, old browser). On failure, the viewer falls
    /// back to main-thread chunked processing.
    #[cfg(target_arch = "wasm32")]
    fn try_init_worker() -> bool {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        let window = match web_sys::window() {
            Some(w) => w,
            None => return false,
        };

        // Call window.workerInit() — returns true if Worker was created
        let result = window.get("workerInit").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });

        match result {
            Some(val) => {
                let ok = val.as_bool().unwrap_or(false);
                if ok {
                    log::info!("[Viewer] Web Worker initialized successfully");
                } else {
                    log::warn!("[Viewer] Web Worker init returned false — falling back to main thread");
                }
                ok
            }
            None => {
                log::warn!("[Viewer] window.workerInit not found — falling back to main thread");
                false
            }
        }
    }

    /// Check if the Worker has posted 'ready' (WASM initialized).
    #[cfg(target_arch = "wasm32")]
    fn check_worker_ready(&mut self) {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker || self.worker_ready {
            return;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return,
        };
        let result = window.get("workerIsReady").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });
        if let Some(val) = result {
            self.worker_ready = val.as_bool().unwrap_or(false);
        }
    }

    /// Check for Worker errors and log them.
    #[cfg(target_arch = "wasm32")]
    fn check_worker_error(&mut self) -> Option<String> {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker {
            return None;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return None,
        };
        let result = window.get("workerGetError").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });
        result.and_then(|v| v.as_string()).filter(|s| !s.is_empty())
    }

    /// Send STEP content to the Worker for parsing.
    ///
    /// Returns `true` if the message was sent successfully.
    /// The result is polled via `check_worker_parse_result()`.
    #[cfg(target_arch = "wasm32")]
    fn worker_parse_step(&mut self, content: &str, name: &str, lod: f64, profile: &str) -> bool {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker {
            return false;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return false,
        };
        let result = window.get("workerParseStep").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call4(
                &JsValue::NULL,
                &JsValue::from_str(content),
                &JsValue::from_str(name),
                &JsValue::from_f64(lod),
                &JsValue::from_str(profile),
            ).ok()
        });
        result.map(|v| v.as_bool().unwrap_or(false)).unwrap_or(false)
    }

    /// Check if the Worker has finished parsing the STEP file.
    ///
    /// Returns `Some((pending_breps_json, assembly_tree_json))` if ready,
    /// `None` if still pending.
    #[cfg(target_arch = "wasm32")]
    fn check_worker_parse_result(&mut self) -> Option<(String, String)> {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker {
            return None;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return None,
        };
        let result = window.get("workerGetParseResult").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });
        result.and_then(|v| {
            if v.is_null() || v.is_undefined() {
                return None;
            }
            let obj: js_sys::Object = v.dyn_into().ok()?;
            let pbj = js_sys::Reflect::get(&obj, &JsValue::from_str("pending_breps_json")).ok()?;
            let atj = js_sys::Reflect::get(&obj, &JsValue::from_str("assembly_tree_json")).ok()?;
            Some((pbj.as_string()?, atj.as_string()?))
        })
    }

    /// Request the Worker to triangulate the next BREP.
    #[cfg(target_arch = "wasm32")]
    fn worker_triangulate_next(&mut self) -> bool {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker {
            return false;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return false,
        };
        let result = window.get("workerTriangulateNext").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });
        result.map(|v| v.as_bool().unwrap_or(false)).unwrap_or(false)
    }

    /// Collect pending mesh results from the Worker.
    ///
    /// This reads the JS `window._draperWorkerPendingMeshes` array and
    /// converts each entry to a `WorkerMeshResult`.
    #[cfg(target_arch = "wasm32")]
    fn collect_worker_meshes(&mut self) {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker {
            return;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return,
        };
        let result = window.get("workerGetPendingMeshes").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });

        if let Some(val) = result {
            let arr_result: Result<js_sys::Array, JsValue> = val.dyn_into();
            if let Ok(arr) = arr_result {
                for i in 0..arr.length() {
                    let item = arr.get(i);
                    if item.is_null() || item.is_undefined() {
                        continue;
                    }
                    let obj: js_sys::Object = match item.dyn_into() {
                        Ok(o) => o,
                        Err(_) => continue,
                    };

                    let name = js_sys::Reflect::get(&obj, &JsValue::from_str("name"))
                        .ok()
                        .and_then(|v| v.as_string())
                        .unwrap_or_default();

                    let brep_id = js_sys::Reflect::get(&obj, &JsValue::from_str("brep_id"))
                        .ok()
                        .and_then(|v| v.as_f64())
                        .unwrap_or(-1.0) as usize;

                    let color = js_sys::Reflect::get(&obj, &JsValue::from_str("color"))
                        .ok()
                        .and_then(|v| {
                            if v.is_null() || v.is_undefined() {
                                return None;
                            }
                            let arr: js_sys::Array = v.dyn_into().ok()?;
                            if arr.length() < 4 {
                                return None;
                            }
                            Some([
                                arr.get(0).as_f64()? as f32,
                                arr.get(1).as_f64()? as f32,
                                arr.get(2).as_f64()? as f32,
                                arr.get(3).as_f64()? as f32,
                            ])
                        });

                    let vertices = js_sys::Reflect::get(&obj, &JsValue::from_str("vertices"))
                        .ok()
                        .and_then(|v| {
                            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
                            Some(arr.to_vec())
                        })
                        .unwrap_or_default();

                    let indices = js_sys::Reflect::get(&obj, &JsValue::from_str("indices"))
                        .ok()
                        .and_then(|v| {
                            let arr: js_sys::Uint32Array = v.dyn_into().ok()?;
                            Some(arr.to_vec())
                        })
                        .unwrap_or_default();

                    let normals = js_sys::Reflect::get(&obj, &JsValue::from_str("normals"))
                        .ok()
                        .and_then(|v| {
                            if v.is_null() || v.is_undefined() {
                                return None;
                            }
                            let len = js_sys::Reflect::get(&v, &JsValue::from_str("length"))
                                .ok()
                                .and_then(|l| l.as_f64())
                                .unwrap_or(0.0) as usize;
                            if len == 0 {
                                return None;
                            }
                            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
                            Some(arr.to_vec())
                        });

                    let face_normals = js_sys::Reflect::get(&obj, &JsValue::from_str("face_normals"))
                        .ok()
                        .and_then(|v| {
                            if v.is_null() || v.is_undefined() {
                                return None;
                            }
                            let len = js_sys::Reflect::get(&v, &JsValue::from_str("length"))
                                .ok()
                                .and_then(|l| l.as_f64())
                                .unwrap_or(0.0) as usize;
                            if len == 0 {
                                return None;
                            }
                            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
                            Some(arr.to_vec())
                        });

                    let colors = js_sys::Reflect::get(&obj, &JsValue::from_str("colors"))
                        .ok()
                        .and_then(|v| {
                            if v.is_null() || v.is_undefined() {
                                return None;
                            }
                            let len = js_sys::Reflect::get(&v, &JsValue::from_str("length"))
                                .ok()
                                .and_then(|l| l.as_f64())
                                .unwrap_or(0.0) as usize;
                            if len == 0 {
                                return None;
                            }
                            let arr: js_sys::Float32Array = v.dyn_into().ok()?;
                            Some(arr.to_vec())
                        });

                    self.worker_pending_meshes.push(WorkerMeshResult {
                        name,
                        brep_id,
                        color,
                        vertices,
                        indices,
                        normals,
                        face_normals,
                        colors,
                    });
                }
            }
        }
    }

    /// Check if the Worker has completed all BREPs.
    #[cfg(target_arch = "wasm32")]
    fn worker_all_complete(&mut self) -> bool {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker {
            return false;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return false,
        };
        let result = window.get("workerAllComplete").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });
        result.map(|v| v.as_bool().unwrap_or(false)).unwrap_or(false)
    }

    /// Cancel Worker triangulation.
    #[cfg(target_arch = "wasm32")]
    fn worker_cancel(&mut self) {
        use wasm_bindgen::JsCast;
        use wasm_bindgen::JsValue;
        if !self.use_worker {
            return;
        }
        let window = match web_sys::window() {
            Some(w) => w,
            None => return,
        };
        let _ = window.get("workerCancel").and_then(|v| {
            let func: js_sys::Function = v.dyn_into().ok()?;
            func.call0(&JsValue::NULL).ok()
        });
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
                let lod_label = self.lod_level.label();
                let lod_value = self.lod_level.lod_value();
                self.log(&format!(
                    "Building conversion context (entity maps, bounding box) — LOD={} ({:.2})...",
                    lod_label, lod_value
                ));
                // Pass the user-selected LOD so the Quality dropdown actually
                // affects STEP triangulation. Without this, the context would
                // use TriangulationParams::default() (LOD 1.0) regardless of
                // the dropdown, and switching High→Low would have no effect.
                //
                // Profile: select based on `is_mobile` (which is computed from
                // `screen_width < 768.0` in `update()`). Desktop/Tablet users
                // get the Desktop profile (96×64 grid cap) for visual quality;
                // Mobile users get the Mobile profile (32×16 grid cap) for
                // responsiveness on slow CPUs.
                let steiner_profile = if self.is_mobile {
                    draper_mesh::triangulate::SteinerBudgetProfile::Mobile
                } else {
                    draper_mesh::triangulate::SteinerBudgetProfile::Desktop
                };
                let ctx_result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    OwnedStepConversionContext::new_with_lod_and_profile(step_file, lod_value, steiner_profile)
                }));
                match ctx_result {
                    Ok(ctx) => {
                        self.conversion_ctx = Some(ctx);
                        self.log(&format!(
                            "Conversion context ready (quality={}) — starting triangulation...",
                            lod_label
                        ));
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

        // Check for global loading timeout.
        // Desktop: 5 minutes (users have more patience, more CPU).
        // Mobile: 2 minutes (slower CPU, less patience — user can cancel manually too).
        if let Some(start) = self.loading_start {
            let timeout = if self.is_mobile {
                std::time::Duration::from_secs(120)
            } else {
                std::time::Duration::from_secs(300)
            };
            if start.elapsed() > timeout {
                let elapsed = start.elapsed().as_secs();
                let remaining = self.pending_breps.len();

                // CRITICAL: Before declaring timeout, salvage any partial
                // triangulation from the active chunked session. Without this,
                // the active BREP would simply vanish — the user would see
                // "4 of 5 instances" but the 5th would be missing. This was
                // the user-reported "исчез один элемент сборки" regression.
                if !self.pending_breps.is_empty() {
                    let pending_front = self.pending_breps[0].clone();
                    if let Some(ref mut ctx) = self.conversion_ctx {
                        match ctx.take_partial_active_session(&pending_front) {
                            Some((partial_mesh, partial_faces, faces_done, faces_total)) => {
                                if partial_mesh.triangle_count() > 0 {
                                    let tri_start = self.mesh.triangle_count();
                                    let color = pending_front.color.unwrap_or_else(|| {
                                        Self::instance_color(self.triangulated_count)
                                    });
                                    self.mesh.merge_with_color(&partial_mesh, color);
                                    let tri_end = self.mesh.triangle_count();
                                    self.instance_triangle_ranges.push((tri_start, tri_end));

                                    // Build a partial DetailedMeshInstance for
                                    // the structure tree / detailed_instances.
                                    let inst_idx = self.instance_triangle_ranges.len() - 1;
                                    if let Some(ref mut tree) = self.assembly_tree {
                                        assign_instance_to_tree(tree, inst_idx);
                                    }
                                    self.detailed_instances.push(DetailedMeshInstance {
                                        name: pending_front.name.clone(),
                                        mesh: partial_mesh,
                                        color: pending_front.color,
                                        transform: pending_front.transform,
                                        brep_id: pending_front.brep_id,
                                        faces: partial_faces,
                                    });
                                    self.triangulated_count += 1;
                                    self.log_warning(&format!(
                                        "BREP #{} '{}' — salvaged PARTIAL triangulation: {}/{} faces ({}%), {} triangles",
                                        pending_front.brep_id, pending_front.name,
                                        faces_done, faces_total,
                                        if faces_total > 0 { (faces_done * 100) / faces_total } else { 0 },
                                        tri_end - tri_start
                                    ));
                                } else {
                                    self.log_warning(&format!(
                                        "BREP #{} '{}' — partial session produced 0 triangles, instance skipped",
                                        pending_front.brep_id, pending_front.name
                                    ));
                                    self.failed_face_count += 1;
                                    if let Some(ref mut tree) = self.assembly_tree {
                                        skip_instance_in_tree(tree);
                                    }
                                }
                            }
                            None => {
                                self.log_warning(&format!(
                                    "BREP #{} '{}' — no active session to salvage, instance skipped",
                                    pending_front.brep_id, pending_front.name
                                ));
                                self.failed_face_count += 1;
                                if let Some(ref mut tree) = self.assembly_tree {
                                    skip_instance_in_tree(tree);
                                }
                            }
                        }
                    }
                    // Remove the front BREP regardless — we've either salvaged
                    // it or skipped it.
                    self.pending_breps.remove(0);
                }

                self.chunked_brep_faces_done = 0;
                self.chunked_brep_faces_total = 0;
                let loaded_count = self.triangulated_count;
                let total_count = loaded_count + remaining - 1; // -1 for the one we just salvaged/skipped
                self.log_warning(&format!(
                    "Loading timed out after {}s — salvaged partial result: {}/{} instances loaded",
                    elapsed, loaded_count, total_count.max(loaded_count)
                ));
                // Store partial result info for UI display
                let total_instances = total_count.max(loaded_count);
                let face_count = self.current_model.triangle_count / 3; // approximate faces from triangles
                self.partial_result_info = Some(format!(
                    "Timed out after {}s — {}/{} instances, {} faces triangulated",
                    elapsed, loaded_count, total_instances, face_count
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

        // ─── WASM path: Worker-based triangulation (if available) ───────────
        // When the Web Worker is active, we use a polling approach:
        //   1. Collect any pending mesh results from the Worker
        //   2. Process them (merge into scene)
        //   3. Request the next BREP to be triangulated
        //   4. Check if all BREPs are done
        #[cfg(target_arch = "wasm32")]
        if self.use_worker && self.worker_ready {
            // If Worker is still parsing (no pending_breps yet), check for result
            if self.is_loading && self.pending_breps.is_empty() {
                if let Some((pbreps_json, _tree_json)) = self.check_worker_parse_result() {
                    // Worker finished parsing — set up the pending BREP list
                    let pbreps: Vec<serde_json::Value> = serde_json::from_str(&pbreps_json)
                        .unwrap_or_default();
                    self.total_instance_count = pbreps.len();

                    // Populate pending_breps from JSON
                    for p in &pbreps {
                        let name = p.get("name").and_then(|v| v.as_str()).unwrap_or("unknown").to_string();
                        let brep_id = p.get("brep_id").and_then(|v| v.as_i64()).unwrap_or(-1);
                        let transform = p.get("transform").and_then(|v| {
                            if v.is_null() { return None; }
                            let arr = v.as_array()?;
                            if arr.len() < 4 { return None; }
                            let mut tf = [[0.0f64; 4]; 4];
                            for i in 0..4 {
                                let row = arr.get(i)?.as_array()?;
                                if row.len() < 4 { return None; }
                                for j in 0..4 {
                                    tf[i][j] = row.get(j)?.as_f64()?;
                                }
                            }
                            Some(tf)
                        });
                        let color = p.get("color").and_then(|v| {
                            if v.is_null() { return None; }
                            let arr = v.as_array()?;
                            if arr.len() < 4 { return None; }
                            Some([
                                arr.get(0)?.as_f64()? as f32,
                                arr.get(1)?.as_f64()? as f32,
                                arr.get(2)?.as_f64()? as f32,
                                arr.get(3)?.as_f64()? as f32,
                            ])
                        });
                        self.pending_breps.push(draper_step::PendingBrepInstance {
                            name,
                            brep_id,
                            transform,
                            color,
                            face_count_estimate: None,
                        });
                    }

                    // Set up assembly tree from JSON (basic)
                    self.show_structure = true;

                    self.log(&format!(
                        "Worker: STEP parsed — {} BREP instances queued for triangulation",
                        self.pending_breps.len()
                    ));

                    // Start triangulating the first BREP
                    if !self.pending_breps.is_empty() {
                        self.worker_triangulate_next();
                    }
                } else {
                    // Still waiting for Worker to parse — check for errors
                    if let Some(err) = self.check_worker_error() {
                        self.log_warning(&format!("Worker parse error: {} — falling back to main thread", err));
                        self.use_worker = false;
                        self.is_loading = false;
                        // Fall through — user can retry
                    }
                    return true; // Still loading
                }
            }

            // Check for Worker errors
            if let Some(err) = self.check_worker_error() {
                self.log_warning(&format!("Worker error: {} — falling back to main thread", err));
                self.use_worker = false;
                // Fall through to the chunked path below
            } else {
                // Collect any pending mesh results from the Worker
                self.collect_worker_meshes();
                if !self.worker_pending_meshes.is_empty() {
                    self.process_worker_results();
                }

                // Request the next BREP if Worker is idle
                // (Worker processes one BREP at a time; we request the next
                // after each result comes back)
                if self.is_loading && !self.pending_breps.is_empty() {
                    self.worker_triangulate_next();
                }

                // Check if all BREPs are done
                if self.worker_all_complete() || self.pending_breps.is_empty() {
                    // Final collection of any remaining results
                    self.collect_worker_meshes();
                    if !self.worker_pending_meshes.is_empty() {
                        self.process_worker_results();
                    }

                    self.is_loading = false;
                    self.conversion_ctx = None;
                    self.loading_start = None;
                    let vcount = self.mesh.vertex_count();
                    let tcount = self.mesh.triangle_count();
                    self.log(&format!(
                        "Worker: Triangulation complete — {} instances, {} vertices, {} triangles",
                        self.triangulated_count, vcount, tcount
                    ));
                    if self.failed_face_count > 0 {
                        self.log_warning(&format!("{} instances failed triangulation", self.failed_face_count));
                    }
                    self.load_mesh(self.mesh.clone(), &format!("STEP: {}", self.loading_name));
                    self.cache_step_result();
                    self.loading_name.clear();
                    self._pending_step_content = None;
                    self._pending_step_name = None;
                    return false;
                }

                return self.is_loading;
            }
        }

        // ─── WASM path: intra-BREP chunked triangulation (fallback) ─────────
        // On WASM, a single BREP can take 10-60s on mobile (e.g., drill_top.stp's
        // GEAR BREP is 12s desktop = 60s mobile). Blocking the main thread for
        // that long freezes the browser UI. Instead, we process each BREP in
        // time-bounded chunks (default 500ms per chunk), yielding to the browser
        // between chunks so it can repaint and process input (including the
        // Cancel button).
        #[cfg(target_arch = "wasm32")]
        {
            if !self.pending_breps.is_empty() {
                // 500ms per chunk — long enough to make progress, short enough
                // to keep the UI responsive (browser can repaint between chunks).
                let max_chunk_time = std::time::Duration::from_millis(500);

                // Peek at the first pending BREP (don't remove until Done).
                let pending = self.pending_breps[0].clone();

                // Log transform info on first chunk of this BREP.
                if let Some(ref tf) = pending.transform {
                    let is_identity = (tf[0][0] - 1.0).abs() < 1e-10 && (tf[1][1] - 1.0).abs() < 1e-10 && (tf[2][2] - 1.0).abs() < 1e-10 && tf[0][3].abs() < 1e-10 && tf[1][3].abs() < 1e-10 && tf[2][3].abs() < 1e-10;
                    if !is_identity && self.triangulated_count == 0 {
                        // Only log once per BREP (when it's the first chunk).
                        // We detect "first chunk" by checking if the context has
                        // an active session for this brep_id — but that's internal.
                        // Simpler: log always, it's not that expensive.
                        let has_rotation = (tf[0][0] - 1.0).abs() > 1e-6 || (tf[1][1] - 1.0).abs() > 1e-6 || (tf[2][2] - 1.0).abs() > 1e-6
                            || tf[0][1].abs() > 1e-6 || tf[0][2].abs() > 1e-6
                            || tf[1][0].abs() > 1e-6 || tf[1][2].abs() > 1e-6
                            || tf[2][0].abs() > 1e-6 || tf[2][1].abs() > 1e-6;
                        if has_rotation {
                            self.log(&format!(
                                "Instance '{}' (BREP #{}): non-identity transform with ROTATION — translation=({:.3}, {:.3}, {:.3})",
                                pending.name, pending.brep_id,
                                tf[0][3], tf[1][3], tf[2][3]
                            ));
                        }
                    }
                }

                let brep_start = web_time::Instant::now();
                let result = if let Some(ref mut ctx) = self.conversion_ctx {
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        ctx.triangulate_pending_chunked(&pending, max_chunk_time)
                    })).unwrap_or_else(|_| {
                        log::error!("Panic during chunked triangulation of '{}' (BREP #{}), aborting", pending.name, pending.brep_id);
                        ctx.abort_active_session();
                        draper_step::TriangulatePendingResult::Done(None)
                    })
                } else {
                    draper_step::TriangulatePendingResult::Done(None)
                };

                match result {
                    draper_step::TriangulatePendingResult::Done(instance) => {
                        // BREP fully triangulated — remove from pending and handle.
                        self.pending_breps.remove(0);
                        self.chunked_brep_faces_done = 0;
                        self.chunked_brep_faces_total = 0;
                        let brep_elapsed_ms = brep_start.elapsed().as_secs_f64() * 1000.0;

                        match instance {
                            Some(inst) => {
                                if inst.mesh.triangle_count() == 0 && inst.mesh.vertex_count() == 0 {
                                    self.log_warning(&format!(
                                        "Instance '{}' (BREP #{}) produced empty mesh — skipping ({:.1}ms)",
                                        inst.name, inst.brep_id, brep_elapsed_ms
                                    ));
                                    self.failed_face_count += 1;
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
                                if let Some(ref mut tree) = self.assembly_tree {
                                    skip_instance_in_tree(tree);
                                }
                            }
                        }
                        self.triangulated_count += 1;
                    }
                    draper_step::TriangulatePendingResult::InProgress { faces_done, faces_total } => {
                        // BREP still has more faces to process — yield to next frame.
                        self.chunked_brep_faces_done = faces_done;
                        self.chunked_brep_faces_total = faces_total;
                        log::debug!(
                            "BREP '{}' chunk: {}/{} faces done in {:.1}ms, yielding",
                            pending.name, faces_done, faces_total,
                            brep_start.elapsed().as_secs_f64() * 1000.0
                        );
                        // Don't increment triangulated_count — this BREP isn't done yet.
                        // Don't remove from pending_breps — we'll continue next frame.
                    }
                }

                self.mesh_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;

                if self.pending_breps.is_empty() {
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
                    self.cache_step_result();
                    self.loading_name.clear();
                    self._pending_step_content = None;
                    self._pending_step_name = None;
                    return false;
                }
                return true;
            }
            return false;
        }

        // ─── Native path: process full BREPs per frame (fast on desktop) ───
        #[cfg(not(target_arch = "wasm32"))]
        {
            // When there are many BREPs (> 4), use rayon-based parallel
            // triangulation to utilize all CPU cores. This gives ~2.5× speedup
            // on 4-core CPUs for assemblies like as1-oc-214 (12 BREPs).
            // For small files (≤ 4 BREPs), sequential processing avoids the
            // overhead of thread pool coordination.
            let use_parallel = self.pending_breps.len() > 4;

            if use_parallel {
                // ─── Parallel path: dispatch all BREPs to rayon at once ───
                let brep_count = self.pending_breps.len();
                self.log(&format!(
                    "Triangulating {} BREPs in parallel ({} threads)...",
                    brep_count,
                    rayon::current_num_threads().min(brep_count)
                ));

                let pending_snapshot: Vec<_> = self.pending_breps.drain(..).collect();
                let total = pending_snapshot.len();

                let results = if let Some(ref mut ctx) = self.conversion_ctx {
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        ctx.triangulate_breps_parallel(
                            &pending_snapshot,
                            || false, // no cancellation in parallel path (Cancel handled by timeout)
                            |done, total| {
                                log::debug!("Parallel progress: {}/{} BREPs done", done, total);
                            },
                        )
                    })).unwrap_or_else(|_| {
                        log::error!("Panic during parallel triangulation — falling back to empty results");
                        vec![None; total]
                    })
                } else {
                    vec![None; total]
                };

                let mut success_count = 0usize;
                let mut fail_count = 0usize;

                for (i, result) in results.into_iter().enumerate() {
                    match result {
                        Some(inst) => {
                            if inst.mesh.triangle_count() == 0 && inst.mesh.vertex_count() == 0 {
                                self.log_warning(&format!(
                                    "Instance '{}' (BREP #{}) produced empty mesh — skipping (parallel)",
                                    inst.name, inst.brep_id
                                ));
                                self.failed_face_count += 1;
                                if let Some(ref mut tree) = self.assembly_tree {
                                    skip_instance_in_tree(tree);
                                }
                            } else {
                                let tri_start = self.mesh.triangle_count();
                                let color = inst.color.unwrap_or_else(|| {
                                    Self::instance_color(self.triangulated_count + success_count)
                                });
                                self.mesh.merge_with_color(&inst.mesh, color);
                                let tri_end = self.mesh.triangle_count();
                                self.instance_triangle_ranges.push((tri_start, tri_end));

                                let inst_idx = self.instance_triangle_ranges.len() - 1;
                                if let Some(ref mut tree) = self.assembly_tree {
                                    assign_instance_to_tree(tree, inst_idx);
                                }

                                self.detailed_instances.push(inst);
                                success_count += 1;
                            }
                        }
                        None => {
                            if i < pending_snapshot.len() {
                                self.log_warning(&format!(
                                    "Instance '{}' (BREP #{}) failed triangulation — skipping (parallel)",
                                    pending_snapshot[i].name, pending_snapshot[i].brep_id
                                ));
                            }
                            self.failed_face_count += 1;
                            if let Some(ref mut tree) = self.assembly_tree {
                                skip_instance_in_tree(tree);
                            }
                            fail_count += 1;
                        }
                    }
                }

                self.triangulated_count += success_count;
                self.mesh_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;

                // Loading complete
                self.is_loading = false;
                self.conversion_ctx = None;
                self.loading_start = None;
                let vcount = self.mesh.vertex_count();
                let tcount = self.mesh.triangle_count();
                self.log(&format!(
                    "Parallel triangulation complete: {} instances ({} ok, {} failed), {} vertices, {} triangles",
                    self.triangulated_count, success_count, fail_count, vcount, tcount
                ));
                if self.failed_face_count > 0 {
                    self.log_warning(&format!("{} instances failed triangulation", self.failed_face_count));
                }
                self.load_mesh(self.mesh.clone(), &format!("STEP: {}", self.loading_name));
                self.loading_name.clear();
                return false;
            }

            // ─── Sequential path: process up to 8 BREPs per frame ───
            let mut processed = 0;
            let max_batch = std::cmp::min(self.pending_breps.len(), 8);

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
                let brep_start = std::time::Instant::now();

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

                // Warn if this BREP exceeded the time budget
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
            return true;
        }

        #[allow(unreachable_code)]
        false
    }

    /// Import STL from bytes (used by web file loading).
    fn import_stl_from_bytes(&mut self, data: &[u8], name: &str) {
        match draper_mesh::import_stl_from_bytes(data) {
            Ok(mesh) => {
                // STL has no surfaces / faces — clear `current_solid`
                // (and the UV window state) so the UV Breakdown window
                // correctly shows "No solid loaded" instead of leaking
                // a stale primitive/NURBS solid from a previous session.
                self.current_solid = None;
                self.current_nurbs_surface = None;
                self.solid_uv_breakdown = None;
                self.uv_window_face_idx = None;
                self.uv_window_prev_face_idx = None;
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

        // Clear partial result info from previous load
        self.partial_result_info = None;
        // Clear LOD downgrade info — new file starts fresh
        self.lod_downgraded_from = None;

        // ─── Cache lookup (WASM only) ──────────────────────────────────
        // Before doing any parsing or triangulation, check if we already
        // have a cached result for this file content + LOD. If so, load
        // it instantly without any computation.
        #[cfg(target_arch = "wasm32")]
        {
            self.loaded_from_cache = false;
            self.cache_step_hash = None;
            let lod_value = self.lod_level.lod_value();
            self.cache_state = crate::cache::CacheState::Idle;
            self.log(&format!("Checking cache for '{}' (LOD={:.2})...", name, lod_value));
            self.cache_manager.start_lookup(content, name, lod_value);
            // The lookup is async — we'll check the result in check_cache_lookup()
            // which is called each frame. For now, set loading state and return.
            // If the cache hits, we'll load the data instantly. If it misses,
            // we'll fall through to the Worker or main-thread path.
            self.cancel_loading(false);
            self.current_solid = None;
            self.current_nurbs_surface = None;
            self.solid_uv_breakdown = None;
            self.uv_window_face_idx = None;
            self.uv_window_prev_face_idx = None;
            self.last_step_name = name.to_string();
            self.is_loading = true;
            self.loading_name = name.to_string();
            self.loading_start = Some(web_time::Instant::now());
            self.total_instance_count = 0;
            self.triangulated_count = 0;
            self.detailed_instances.clear();
            self.instance_triangle_ranges.clear();
            self.selected_instance = None;
            self.selected_face = None;
            self.highlighted_face = None;
            self.failed_face_count = 0;
            self.mesh = TriangleMesh::new();
            self.pending_breps.clear();
            self.conversion_ctx = None;
            self.pending_step_file = None;
            // Store content for later (cache miss → need to parse)
            self._pending_step_content = Some(content.to_string());
            self._pending_step_name = Some(name.to_string());
            return;
        }

        // ─── Native path: no cache, parse directly ─────────────────────
        #[cfg(not(target_arch = "wasm32"))]
        {
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
    }

    /// Check if an in-flight cache lookup has completed.
    /// Called each frame from the render loop. On cache hit, loads the
    /// cached data directly. On cache miss, falls through to the Worker
    /// or main-thread parsing path.
    #[cfg(target_arch = "wasm32")]
    fn check_cache_lookup(&mut self) {
        use crate::cache::CacheState;

        let state = self.cache_manager.take_state();
        match state {
            CacheState::Idle | CacheState::Hashing | CacheState::LookingUp { .. } => {
                // Still in progress — restore state and wait
                self.cache_state = state;
            }
            CacheState::Hit(result) => {
                // Cache hit! Load the data instantly.
                self.loaded_from_cache = true;
                self.log(&format!(
                    "Loaded from cache: '{}' — {} vertices, {} triangles (LOD={:.2})",
                    result.file_name,
                    result.mesh.vertex_count(),
                    result.mesh.triangle_count(),
                    result.lod
                ));

                // Merge all instance meshes into one TriangleMesh
                // (cached data has the merged mesh + instance metadata)
                let mut merged_mesh = result.mesh;
                merged_mesh.fill_missing_face_normals();
                merged_mesh.ensure_colors([0.62, 0.65, 0.70, 1.0]);

                let cached_instances = result.instances;

                // ─── Reconstruct instance triangle ranges from face_ids ───
                // The merged mesh has triangle_face_ids, and each cached instance
                // has faces with their face_ids. We use these to determine which
                // triangles belong to which instance.
                let total_tris = merged_mesh.triangle_count();
                let mut final_ranges: Vec<(usize, usize)> = Vec::with_capacity(cached_instances.len());

                if let Some(ref face_ids) = merged_mesh.triangle_face_ids {
                    // Collect face_ids belonging to each instance
                    for inst in &cached_instances {
                        let inst_face_ids: std::collections::HashSet<u64> = inst.faces.iter()
                            .map(|f| f.face_id)
                            .collect();
                        let mut tri_start = total_tris;
                        let mut tri_end = 0usize;
                        for (ti, &fid) in face_ids.iter().enumerate() {
                            if inst_face_ids.contains(&fid) {
                                tri_start = tri_start.min(ti);
                                tri_end = tri_end.max(ti + 1);
                            }
                        }
                        if tri_start >= tri_end {
                            tri_start = 0;
                            tri_end = 0;
                        }
                        final_ranges.push((tri_start, tri_end));
                    }

                    // ─── Recompute FaceInfo.triangle_range from merged mesh ───
                    // The cached triangle_range may be stale. Use triangle_face_ids
                    // to find the actual contiguous range for each face.
                    let mut face_ranges: std::collections::HashMap<u64, (usize, usize)> = std::collections::HashMap::new();
                    for (ti, &fid) in face_ids.iter().enumerate() {
                        let entry = face_ranges.entry(fid).or_insert((ti, ti));
                        entry.0 = entry.0.min(ti);
                        entry.1 = entry.1.max(ti + 1);
                    }
                    let mut updated_instances = cached_instances;
                    for inst in &mut updated_instances {
                        for fi in &mut inst.faces {
                            if let Some(&(start, end)) = face_ranges.get(&fi.face_id) {
                                fi.triangle_range = (start, end);
                            }
                        }
                    }
                    self.detailed_instances = updated_instances;
                } else {
                    // No face_ids — fallback: single range covering all triangles
                    let total = merged_mesh.triangle_count();
                    for _ in &cached_instances {
                        final_ranges.push((0, total));
                    }
                    self.detailed_instances = cached_instances;
                }

                // Set the assembly tree
                self.assembly_tree = Some(result.assembly_tree);
                self.show_structure = true;

                // Load the merged mesh
                self.instance_triangle_ranges = final_ranges;
                self.load_mesh(merged_mesh, &result.file_name);

                // Mark loading complete
                self.is_loading = false;
                self.total_instance_count = self.detailed_instances.len();
                self.triangulated_count = self.detailed_instances.len();

                self.cache_state = CacheState::Idle;
            }
            CacheState::Miss { hash } => {
                // Cache miss — proceed with normal parsing path
                self.cache_step_hash = Some(hash);
                self.log("Cache: miss — proceeding with triangulation...");
                self.cache_state = CacheState::Idle;

                // Fall through to Worker or main-thread path
                // Clone content — we need to keep it for cache storage after triangulation
                let content = self._pending_step_content.clone();
                let name = self._pending_step_name.clone();

                if let (Some(content), Some(name)) = (content, name) {
                    self.import_step_from_str_no_cache(&content, &name);
                }
            }
            CacheState::Error(e) => {
                // Cache error — log and fall through to normal path
                self.log_warning(&format!("Cache lookup error: {} — proceeding with triangulation", e));
                self.cache_state = CacheState::Idle;

                let content = self._pending_step_content.clone();
                let name = self._pending_step_name.clone();

                if let (Some(content), Some(name)) = (content, name) {
                    self.import_step_from_str_no_cache(&content, &name);
                }
            }
        }
    }

    /// Import STEP from string WITHOUT cache lookup (called after cache miss).
    /// This is the original import logic: Worker path or main-thread path.
    #[cfg(target_arch = "wasm32")]
    fn import_step_from_str_no_cache(&mut self, content: &str, name: &str) {
        // ─── Worker path: send STEP content to Web Worker ────────────────
        if self.use_worker && self.worker_ready {
            let lod_value = self.lod_level.lod_value();
            let profile = if self.is_mobile { "Mobile" } else { "Desktop" };
            let sent = self.worker_parse_step(content, name, lod_value, profile);
            if sent {
                self.log(&format!("Sent '{}' to Worker for background parsing (LOD={:.2}, profile={})", name, lod_value, profile));
                return;
            } else {
                self.log_warning("Worker send failed — falling back to main-thread parsing");
            }
        }

        // ─── Main-thread path ────────────────────────────────────────────
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
                            // Read as ArrayBuffer to support ANSI/Windows-1251/1252
                            // encoded files (not just UTF-8). Convert to String
                            // using from_utf8_lossy which replaces invalid bytes
                            // with U+FFFD — safe for STEP parsing.
                            if let Some(ab) = result.dyn_ref::<js_sys::ArrayBuffer>() {
                                let array = js_sys::Uint8Array::new(ab);
                                let mut bytes = vec![0u8; array.length() as usize];
                                array.copy_to(&mut bytes);
                                let text = String::from_utf8_lossy(&bytes).into_owned();
                                log::info!("STEP file text extracted: {} chars (from {} bytes)", text.len(), bytes.len());
                                *shared.lock().unwrap() = Some(FileLoadResult::Step {
                                    name: name_for_log.clone(),
                                    content: text,
                                });
                            } else if let Some(text) = result.as_string() {
                                // Fallback: some browsers return string directly
                                log::info!("STEP file text extracted (string): {} chars", text.len());
                                *shared.lock().unwrap() = Some(FileLoadResult::Step {
                                    name: name_for_log.clone(),
                                    content: text,
                                });
                            } else {
                                log::error!("STEP file read result is not ArrayBuffer or string");
                            }
                        } else {
                            log::error!("STEP file read_as_array_buffer() returned error");
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

                    match reader.read_as_array_buffer(&file) {
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
                layers: Vec::new(),
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
                layers: Vec::new(),
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
        // Apply Catppuccin Mocha dark theme when BRepCAD UI is enabled
        if self.enable_brepcad_ui {
            apply_brepcad_theme(ctx);
        }

        // Request repaint for continuous rendering
        ctx.request_repaint();

        // Process any pending web file loads
        #[cfg(target_arch = "wasm32")]
        self.process_web_file_loads();

        // Check if cache lookup has completed (WASM only)
        #[cfg(target_arch = "wasm32")]
        self.check_cache_lookup();

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
            if self.enable_brepcad_ui {
                // === BRepCAD extended UI: 21-menu bar + 15-tab ribbon ===
                if let Some(action) = crate::ui::menubar::render_menu_bar(ctx) {
                    let msg = self.handle_brepcad_action(&action);
                    if !msg.is_empty() {
                        self.brepcad_status_msg = msg;
                    }
                }
                if let Some(action) = crate::ui::ribbon::render_ribbon(ctx, &mut self.brepcad_ribbon_tab) {
                    let msg = self.handle_brepcad_action(&action);
                    if !msg.is_empty() {
                        self.brepcad_status_msg = msg;
                    }
                }
            } else {
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
                    // LOD selector — affects quality of future STEP loads AND
                    // re-triangulates the current model immediately so the user
                    // sees the change without re-opening the file.
                    ui.label(egui::RichText::new("Triangulation Quality:").small());
                    let prev_lod = self.lod_level;
                    for &lod in LodLevel::all() {
                        if ui.radio_value(&mut self.lod_level, lod, lod.label()).clicked() {
                            if lod != prev_lod {
                                self.lod_downgraded_from = None; // user manually changed
                                self.log(&format!(
                                    "LOD changed {} → {} — re-triangulating current model...",
                                    prev_lod.label(),
                                    lod.label()
                                ));
                                self.retriangulate_for_lod();
                                #[cfg(target_arch = "wasm32")]
                                self.save_lod_to_local_storage();
                            }
                        }
                    }
                    ui.separator();
                    ui.checkbox(&mut self.validate_consistency, "Validate Edge Consistency");
                    if let Some(ref report) = self.last_consistency_report {
                        ui.label(egui::RichText::new(report).small().color(egui::Color32::YELLOW));
                    }
                    // Cache controls (WASM only)
                    #[cfg(target_arch = "wasm32")]
                    {
                        ui.separator();
                        ui.label(egui::RichText::new("Cache:").small());
                        if self.loaded_from_cache {
                            ui.label(egui::RichText::new("Last load: from cache").small().color(egui::Color32::from_rgb(80, 200, 80)));
                        }
                        if ui.button("Clear Cache").clicked() {
                            self.cache_manager.clear_cache();
                            self.log("Cache: cleared all entries");
                        }
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
            } // end else (original menu)
        } // end desktop top menu bar

        // === Bottom panel: log (collapsible — especially important on mobile) ===
        // On mobile, the log panel can cover the entire viewport when expanded,
        // so it MUST have a collapse/expand toggle. On desktop the toggle is also
        // useful for users who want more 3D viewport space.
        // When BRepCAD UI is enabled, skip the log panel — a status bar is rendered instead.
        if !self.enable_brepcad_ui {
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
        } // end if !enable_brepcad_ui (skip log panel)

        // === Right panel: Structure / Faces / UV (desktop only) ===
        // Collect pending UI actions to avoid borrow checker conflicts
        let mut pending_instance_select: Option<usize> = None;
        let mut pending_face_select: Option<(usize, u64)> = None;
        let mut pending_copy_face_id: Option<u64> = None;
        let mut pending_visibility_toggle: Option<usize> = None;
        let mut pending_instance_isolate: Option<usize> = None;
        let mut pending_face_visibility_toggle: Option<(usize, u64)> = None;
        let mut pending_face_isolate: Option<(usize, u64)> = None;
        // Subtree-level actions: hide/show/isolate all descendant instances of a subassembly node
        let mut pending_subtree_hide: Option<Vec<usize>> = None;
        let mut pending_subtree_show: Option<Vec<usize>> = None;
        let mut pending_subtree_isolate: Option<Vec<usize>> = None;

        if self.show_structure && !self.is_mobile && !self.enable_brepcad_ui {
            // Clone data needed for drawing to avoid borrow conflicts
            let assembly_tree_clone = self.assembly_tree.clone();
            let detailed_instances_clone = self.detailed_instances.clone();
            let selected_instance = self.selected_instance;
            let selected_face = self.selected_face;
            let open_tree_nodes = self.open_tree_nodes.clone();
            let scroll_to_tree_node = self.scroll_to_tree_node.clone();
            let scroll_to_face_id = self.scroll_to_face_id;
            let hidden_instances = self.hidden_instances.clone();
            let hidden_faces_clone = self.hidden_faces.clone();

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

                    // ─── "Show All" button ───────────────────────────────────────────
                    // Restores visibility of all hidden instances and hidden faces.
                    // Disabled (greyed out) when nothing is hidden so the user gets
                    // immediate visual feedback that there's nothing to restore.
                    let has_hidden = !self.hidden_instances.is_empty() || !self.hidden_faces.is_empty();
                    let show_all_btn = egui::Button::new(
                        egui::RichText::new("Show All").size(11.0)
                    );
                    let show_all_resp = ui.add_enabled(has_hidden, show_all_btn);
                    if show_all_resp.clicked() {
                        self.hidden_instances.clear();
                        self.hidden_faces.clear();
                        self.highlight_dirty = true;
                        self.edge_dirty = true;
                        self.wireframe_overlay_dirty = true;
                        self.log("Show All: restored visibility of all instances and faces");
                    }
                    if show_all_resp.hovered() {
                        let hidden_inst_count = self.hidden_instances.len();
                        let hidden_face_count = self.hidden_faces.len();
                        let tooltip = if has_hidden {
                            format!("Restore visibility of {} hidden instance(s) and {} hidden face(s)",
                                hidden_inst_count, hidden_face_count)
                        } else {
                            "Nothing is hidden — all instances and faces are visible".to_string()
                        };
                        show_all_resp.on_hover_text(tooltip);
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
                                        draw_assembly_node_static(ui, tree, selected_instance, &hidden_instances, &mut pending_instance_select, &mut pending_visibility_toggle, &mut pending_instance_isolate, &mut pending_subtree_hide, &mut pending_subtree_show, &mut pending_subtree_isolate, &open_tree_nodes, &scroll_to_tree_node);
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
                                                // Isolate button (desktop fallback list — no assembly tree)
                                                let isolate_btn = egui::Button::new(
                                                    egui::RichText::new("◎").size(11.0)
                                                ).frame(false);
                                                if ui.add(isolate_btn).on_hover_text(
                                                    "Isolate: hide all other instances (click again to restore)"
                                                ).clicked() {
                                                    pending_instance_isolate = Some(i);
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
                                                let is_face_visible = !hidden_faces_clone.contains(&(inst_idx, face.face_id));
                                                ui.horizontal(|ui| {
                                                    // Per-face visibility eye icon
                                                    let eye_color = if is_face_visible {
                                                        egui::Color32::from_rgb(80, 180, 80)
                                                    } else {
                                                        egui::Color32::from_rgb(180, 80, 80)
                                                    };
                                                    let eye_text = if is_face_visible { "👁" } else { "  " };
                                                    if ui.add(egui::Label::new(egui::RichText::new(eye_text).size(11.0).color(eye_color)).sense(egui::Sense::click())).clicked() {
                                                        pending_face_visibility_toggle = Some((inst_idx, face.face_id));
                                                    }
                                                    // Per-face isolate button — hides all OTHER faces of this instance
                                                    let isolate_btn = egui::Button::new(
                                                        egui::RichText::new("◎").size(11.0)
                                                    ).frame(false);
                                                    if ui.add(isolate_btn).on_hover_text(
                                                        "Isolate: hide all other faces of this instance (click again to restore)"
                                                    ).clicked() {
                                                        pending_face_isolate = Some((inst_idx, face.face_id));
                                                    }
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
                                                });
                                            }
                                        });
                                }
                            } else {
                                ui.label(egui::RichText::new("Select an instance to see faces").size(11.0).color(egui::Color32::GRAY));
                            }
                        });

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
                    // The active solid for the UV window may have been
                    // derived from this instance — clear the UV window's
                    // face index AND invalidate the cached breakdown so
                    // we don't keep showing UV for a hidden instance.
                    self.uv_window_face_idx = None;
                    self.uv_window_prev_face_idx = None;
                    self.solid_uv_breakdown = None;
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
            // ─── Instance switch → UV window must reset ──────────────────
            // The active solid for the UV window is derived from the
            // selected instance (for STEP files), so when the user picks
            // a different instance the previously-shown face index is
            // stale. Clear it AND invalidate the cached breakdown so the
            // next frame recomputes from the new instance. The UV window
            // will show "Select a face" until the user picks one.
            if self.uv_window_face_idx.is_some() {
                self.uv_window_face_idx = None;
                self.uv_window_prev_face_idx = None;
                self.solid_uv_breakdown = None;
            }
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
            self.scroll_to_face_id = Some(fid);
            self.log(&format!("Selected face #{} in instance #{}", fid, inst_idx));
            // ─── Sync structure-panel face pick → UV Breakdown window ──────
            // The UV window shows the UV breakdown for the active solid's
            // faces. When the user clicks a face in the structure panel,
            // we want the UV window to jump to that face immediately —
            // no matter whether the active solid is a primitive/NURBS
            // (current_solid) or a STEP instance (detailed_instances).
            //
            // For STEP files, the positional index in the instance's
            // `faces` array matches the order used by
            // `solid_from_detailed_instance` (which builds the Solid that
            // `compute_solid_uv_breakdown` iterates), so a direct lookup
            // is correct. For primitives/NURBS, `selected_face` carries
            // no usable index (face IDs come from STEP, not from the
            // primitive's shell), so we leave the UV window's face
            // selection untouched in that case.
            if let Some(inst) = self.detailed_instances.get(inst_idx) {
                if let Some(pos) = inst.faces.iter().position(|f| f.face_id == fid) {
                    if self.uv_window_face_idx != Some(pos) {
                        self.uv_window_face_idx = Some(pos);
                        // Reset zoom/pan so the new face's UV domain is
                        // auto-fitted, not shown with the previous face's
                        // pan/zoom (which would be misleading).
                        self.uv_window_zoom = 1.0;
                        self.uv_window_pan = [0.0, 0.0];
                        self.uv_window_prev_face_idx = Some(pos);
                    }
                }
            }
            // Find the path to this instance in the assembly tree and open it
            if let Some(ref tree) = self.assembly_tree {
                let (path, target) = find_instance_path(tree, inst_idx);
                self.open_tree_nodes = path.into_iter().collect();
                self.scroll_to_tree_node = target;
            }
        }
        if let Some(fid) = pending_copy_face_id {
            ctx.copy_text(format!("{}", fid));
            self.log(&format!("Copied face ID: {}", fid));
        }
        // ─── Isolate instance: hide all OTHER instances ───────────────────
        // If the clicked instance is already the only visible one, restore
        // all instances (toggle behavior). Otherwise, hide every instance
        // except the clicked one.
        if let Some(idx) = pending_instance_isolate {
            let total_instances = self.instance_triangle_ranges.len();
            let visible_count = total_instances.saturating_sub(self.hidden_instances.len());
            let is_only_visible = visible_count == 1 && !self.hidden_instances.contains(&idx);
            if is_only_visible {
                // Restore all
                self.hidden_instances.clear();
                self.log(&format!("Restored all instances (was isolated to #{})", idx));
            } else {
                // Hide all except idx
                self.hidden_instances.clear();
                for i in 0..total_instances {
                    if i != idx {
                        self.hidden_instances.insert(i);
                    }
                }
                self.log(&format!("Isolated instance #{}", idx));
                // Auto-select the isolated instance so the Face List shows it
                if self.selected_instance != Some(idx) {
                    self.selected_instance = Some(idx);
                    self.selected_face = None;
                    self.highlighted_face = None;
                    if self.uv_window_face_idx.is_some() {
                        self.uv_window_face_idx = None;
                        self.uv_window_prev_face_idx = None;
                        self.solid_uv_breakdown = None;
                    }
                }
            }
            self.highlight_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;
        }
        // ─── Subtree-level hide/show/isolate (desktop) ─────────────────
        if let Some(indices) = pending_subtree_hide {
            let count = indices.len();
            for idx in &indices {
                self.hidden_instances.insert(*idx);
            }
            self.log(&format!("Subtree hidden: {} instance(s)", count));
            self.highlight_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;
        }
        if let Some(indices) = pending_subtree_show {
            let count = indices.len();
            for idx in &indices {
                self.hidden_instances.remove(idx);
            }
            self.log(&format!("Subtree shown: {} instance(s)", count));
            self.highlight_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;
        }
        if let Some(indices) = pending_subtree_isolate {
            let total_instances = self.instance_triangle_ranges.len();
            let visible_count = total_instances.saturating_sub(self.hidden_instances.len());
            // Check if the subtree is already the only visible one
            let all_others_hidden = indices.iter().all(|idx| !self.hidden_instances.contains(idx))
                && self.hidden_instances.len() == total_instances - indices.len();
            if all_others_hidden && visible_count == indices.len() {
                // Restore all
                self.hidden_instances.clear();
                self.log(&format!("Restored all instances (was isolated to subtree of {})", indices.len()));
            } else {
                // Hide all except subtree instances
                self.hidden_instances.clear();
                for i in 0..total_instances {
                    if !indices.contains(&i) {
                        self.hidden_instances.insert(i);
                    }
                }
                self.log(&format!("Isolated subtree: {} instance(s) visible", indices.len()));
                // Auto-select the first instance in the subtree
                if let Some(&first_idx) = indices.first() {
                    if self.selected_instance != Some(first_idx) {
                        self.selected_instance = Some(first_idx);
                        self.selected_face = None;
                        self.highlighted_face = None;
                        if self.uv_window_face_idx.is_some() {
                            self.uv_window_face_idx = None;
                            self.uv_window_prev_face_idx = None;
                            self.solid_uv_breakdown = None;
                        }
                    }
                }
            }
            self.highlight_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;
        }
        // ─── Toggle individual face visibility ───────────────────────────
        if let Some((inst_idx, fid)) = pending_face_visibility_toggle {
            if self.hidden_faces.contains(&(inst_idx, fid)) {
                self.hidden_faces.remove(&(inst_idx, fid));
                self.log(&format!("Face #{} in instance #{} shown", fid, inst_idx));
            } else {
                self.hidden_faces.insert((inst_idx, fid));
                self.log(&format!("Face #{} in instance #{} hidden", fid, inst_idx));
                // If the hidden face was selected, deselect it
                if self.selected_face == Some((inst_idx, fid)) {
                    self.selected_face = None;
                    self.highlighted_face = None;
                    // Also clear UV window if it was showing this face
                    if let Some(inst) = self.detailed_instances.get(inst_idx) {
                        if let Some(pos) = inst.faces.iter().position(|f| f.face_id == fid) {
                            if self.uv_window_face_idx == Some(pos) {
                                self.uv_window_face_idx = None;
                                self.uv_window_prev_face_idx = None;
                                self.solid_uv_breakdown = None;
                            }
                        }
                    }
                }
            }
            self.highlight_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;
        }
        // ─── Isolate face: hide all OTHER faces of the same instance ────
        if let Some((inst_idx, fid)) = pending_face_isolate {
            if let Some(inst) = self.detailed_instances.get(inst_idx) {
                let total_faces = inst.faces.len();
                let hidden_in_inst = inst.faces.iter()
                    .filter(|f| self.hidden_faces.contains(&(inst_idx, f.face_id)))
                    .count();
                let visible_in_inst = total_faces.saturating_sub(hidden_in_inst);
                let is_only_visible_face = visible_in_inst == 1
                    && !self.hidden_faces.contains(&(inst_idx, fid));
                if is_only_visible_face {
                    // Restore all faces of this instance
                    let face_keys: Vec<_> = inst.faces.iter()
                        .map(|f| (inst_idx, f.face_id))
                        .collect();
                    for k in face_keys {
                        self.hidden_faces.remove(&k);
                    }
                    self.log(&format!("Restored all faces in instance #{}", inst_idx));
                } else {
                    // Hide all other faces of this instance
                    for f in &inst.faces {
                        if f.face_id != fid {
                            self.hidden_faces.insert((inst_idx, f.face_id));
                        } else {
                            self.hidden_faces.remove(&(inst_idx, f.face_id));
                        }
                    }
                    self.log(&format!("Isolated face #{} in instance #{}", fid, inst_idx));
                }
            }
            self.highlight_dirty = true;
            self.edge_dirty = true;
            self.wireframe_overlay_dirty = true;
        }

        // === Left side panel (controls) — desktop only ===
        // When BRepCAD UI is enabled, skip this — a Browser panel is rendered instead.
        if !self.is_mobile && !self.enable_brepcad_ui {
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
                ui.label(
                    egui::RichText::new(format!("build: {}", env!("DRAPER_GIT_HASH")))
                        .size(9.0)
                        .color(egui::Color32::from_rgb(140, 160, 180))
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
                    // GD&T button — opens a panel showing all geometric
                    // tolerances extracted from the STEP file.
                    if self.last_step_file.is_some() {
                        if ui.button("GD&T").clicked() {
                            self.show_gdt_window = !self.show_gdt_window;
                            if self.gdt_data.is_none() {
                                self.gdt_data = self.last_step_file.as_ref()
                                    .map(|sf| draper_step::pmi::extract_gdt(sf));
                            }
                        }
                        // Toggle 3D annotation overlay for GD&T tolerances
                        ui.checkbox(&mut self.show_gdt_annotations, "3D Annotations");
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
                let prev_lod_mobile = self.lod_level;
                let quality_label = if let Some(from) = self.lod_downgraded_from {
                    format!("Quality: {} (auto from {})", self.lod_level.label(), from.label())
                } else {
                    format!("Quality: {}", self.lod_level.label())
                };
                egui::ComboBox::from_id_salt("lod_mobile")
                    .selected_text(&quality_label)
                    .show_ui(ui, |ui| {
                        for &lod in LodLevel::all() {
                            ui.selectable_value(&mut self.lod_level, lod, lod.label());
                        }
                    });
                if self.lod_level != prev_lod_mobile {
                    self.lod_downgraded_from = None; // user manually changed
                    self.log(&format!(
                        "LOD changed {} → {} — re-triangulating current model...",
                        prev_lod_mobile.label(),
                        self.lod_level.label()
                    ));
                    self.retriangulate_for_lod();
                    #[cfg(target_arch = "wasm32")]
                    self.save_lod_to_local_storage();
                }

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
                    self.open_tree_nodes.clear();
                    self.scroll_to_tree_node = None;
                    self.scroll_to_face_id = None;
                    // Also clear the UV window's active face — the user
                    // explicitly cleared the selection, so the previously
                    // shown UV grid no longer corresponds to anything
                    // highlighted in 3D / structure panel.
                    self.uv_window_face_idx = None;
                    self.uv_window_prev_face_idx = None;
                }

                // --- Info ---
                ui.separator();
                ui.heading(egui::RichText::new("Info").size(12.0));
                ui.label(egui::RichText::new(format!("Model: {}", self.current_model.name)).size(12.0));
                ui.label(egui::RichText::new(format!("Vertices: {}", self.current_model.vertex_count)).size(12.0));
                ui.label(egui::RichText::new(format!("Triangles: {}", self.current_model.triangle_count)).size(12.0));
                ui.label(egui::RichText::new(format!("Instances: {}", self.detailed_instances.len())).size(12.0));
                #[cfg(target_arch = "wasm32")]
                {
                    if self.loaded_from_cache {
                        ui.label(egui::RichText::new("Loaded from cache").size(11.0).color(egui::Color32::from_rgb(80, 200, 80)));
                    }
                }

                // Loading progress in info section
                if self.is_loading && self.total_instance_count > 0 {
                    let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                    ui.label(egui::RichText::new(format!("Loading: {}/{} ({:.0}%)", self.triangulated_count, self.total_instance_count, progress * 100.0))
                        .size(11.0).color(egui::Color32::from_rgb(80, 180, 80)));
                }

                // Partial result info (after timeout or cancel)
                if let Some(ref info) = self.partial_result_info {
                    ui.label(egui::RichText::new(info.clone())
                        .size(11.0)
                        .color(egui::Color32::from_rgb(255, 180, 50)));
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
                            // JSON API-loaded models have no `Solid`
                            // representation — only `detailed_instances`.
                            // Clear `current_solid` so the UV Breakdown
                            // window doesn't leak a stale primitive/NURBS
                            // solid from a previous session.
                            self.current_solid = None;
                            self.current_nurbs_surface = None;
                            self.solid_uv_breakdown = None;
                            self.uv_window_face_idx = None;
                            self.uv_window_prev_face_idx = None;
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

        // ═══ BRepCAD layout: Browser (left) + Properties (right) + Status bar (bottom) ═══
        // When enable_brepcad_ui=true, render the mockup-compliant layout.
        // Uses the same pending-actions pattern as the existing structure panel
        // to avoid borrow checker conflicts (clone data before closure, apply
        // mutations after).
        if self.enable_brepcad_ui && !self.is_mobile {
            // ── Pending actions (collected during panel rendering, applied after) ──
            let mut pending_instance_select: Option<usize> = None;
            let mut pending_visibility_toggle: Option<usize> = None;
            let mut pending_instance_isolate: Option<usize> = None;
            let mut pending_face_select: Option<(usize, u64)> = None;
            let mut pending_face_visibility_toggle: Option<(usize, u64)> = None;

            // ── Clone data needed for Browser panel ──
            let assembly_tree_clone = self.assembly_tree.clone();
            let detailed_instances_clone = self.detailed_instances.clone();
            let selected_instance = self.selected_instance;
            let selected_face_clone = self.selected_face;
            let hidden_instances_clone = self.hidden_instances.clone();
            let hidden_faces_clone = self.hidden_faces.clone();
            let model_name = self.current_model.name.clone();
            let vert_count = self.mesh.vertex_count();
            let tri_count = self.mesh.triangle_count();
            let tree_filter = self.brepcad_tree_filter.clone();

            // ── Left: Browser (Model Tree) ──
            egui::SidePanel::left("brepcad_browser")
                .default_width(280.0)
                .min_width(200.0)
                .resizable(true)
                .show(ctx, |ui| {
                    // Tab strip
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.brepcad_left_tab, BrepcadLeftTab::Tree, "Tree");
                        ui.selectable_value(&mut self.brepcad_left_tab, BrepcadLeftTab::Layers, "Layers");
                        ui.selectable_value(&mut self.brepcad_left_tab, BrepcadLeftTab::Selection, "Selection");
                    });
                    ui.separator();

                    // Filter box
                    ui.horizontal(|ui| {
                        ui.text_edit_singleline(&mut self.brepcad_tree_filter);
                        ui.button("Filter");
                    });
                    ui.separator();

                    match self.brepcad_left_tab {
                        BrepcadLeftTab::Tree => {
                            let filter_lower = tree_filter.to_lowercase();
                            let filter_active = !filter_lower.is_empty();

                            egui::ScrollArea::vertical()
                                .auto_shrink([false; 2])
                                .show(ui, |ui| {
                                    // Model name header
                                    ui.label(egui::RichText::new(format!("📁 {}", model_name))
                                        .size(12.0).color(egui::Color32::from_rgb(0xcd, 0xd6, 0xf4)));
                                    ui.add_space(2.0);

                                    if let Some(ref tree) = assembly_tree_clone {
                                        // Render assembly tree recursively
                                        brepcad_render_assembly_node(
                                            ui, tree, 0,
                                            &filter_lower, filter_active,
                                            selected_instance,
                                            selected_face_clone,
                                            &hidden_instances_clone,
                                            &hidden_faces_clone,
                                            &detailed_instances_clone,
                                            &mut pending_instance_select,
                                            &mut pending_visibility_toggle,
                                            &mut pending_instance_isolate,
                                            &mut pending_face_select,
                                            &mut pending_face_visibility_toggle,
                                        );
                                    } else if !detailed_instances_clone.is_empty() {
                                        // Fallback: flat list of instances
                                        for (i, inst) in detailed_instances_clone.iter().enumerate() {
                                            if filter_active && !inst.name.to_lowercase().contains(&filter_lower) {
                                                continue;
                                            }
                                            let is_selected = selected_instance == Some(i);
                                            let is_visible = !hidden_instances_clone.contains(&i);

                                            ui.horizontal(|ui| {
                                                // Visibility eye icon
                                                let eye_color = if is_visible {
                                                    egui::Color32::from_rgb(0xa6, 0xe3, 0xa1)
                                                } else {
                                                    egui::Color32::from_rgb(0xf3, 0x8b, 0xa8)
                                                };
                                                let eye_text = if is_visible { "👁" } else { "🚫" };
                                                let eye_resp = ui.add(egui::Label::new(
                                                    egui::RichText::new(eye_text).size(11.0).color(eye_color)
                                                ).sense(egui::Sense::click()));
                                                if eye_resp.clicked() {
                                                    pending_visibility_toggle = Some(i);
                                                }

                                                // Isolate button
                                                let iso_resp = ui.add(egui::Button::new(
                                                    egui::RichText::new("◎").size(10.0)
                                                ).frame(false));
                                                if iso_resp.clicked() {
                                                    pending_instance_isolate = Some(i);
                                                }
                                                iso_resp.on_hover_text("Isolate: hide all others");

                                                // Selectable label
                                                let bg = if is_selected {
                                                    egui::Color32::from_rgb(0x09, 0x47, 0x71)
                                                } else {
                                                    egui::Color32::TRANSPARENT
                                                };
                                                let frame = egui::Frame::new().fill(bg)
                                                    .inner_margin(egui::Margin::symmetric(4, 2));
                                                frame.show(ui, |ui| {
                                                    let resp = ui.selectable_label(is_selected,
                                                        egui::RichText::new(format!("📦 {}", inst.name)).size(11.0));
                                                    if resp.clicked() {
                                                        pending_instance_select = Some(i);
                                                    }
                                                });
                                            });
                                        }
                                    } else {
                                        ui.label(egui::RichText::new("No model loaded")
                                            .size(11.0).color(egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
                                        ui.label(egui::RichText::new("Use File → Open to load a STEP file")
                                            .size(10.0).color(egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
                                    }

                                    // Mesh stats at bottom
                                    ui.add_space(8.0);
                                    ui.separator();
                                    ui.label(egui::RichText::new(format!("Vertices: {}", vert_count))
                                        .size(10.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
                                    ui.label(egui::RichText::new(format!("Triangles: {}", tri_count))
                                        .size(10.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
                                });
                        }
                        BrepcadLeftTab::Layers => {
                            self.render_brepcad_layers(ui);
                        }
                        BrepcadLeftTab::Selection => {
                            if selected_instance.is_some() {
                                ui.label(format!("Selected: Instance #{}", selected_instance.unwrap()));
                                if let Some(inst) = detailed_instances_clone.get(selected_instance.unwrap()) {
                                    ui.label(format!("Name: {}", inst.name));
                                    ui.label(format!("BREP ID: {}", inst.brep_id));
                                    ui.label(format!("Faces: {}", inst.faces.len()));
                                }
                            } else {
                                ui.label("No selection");
                                ui.label("Click entities in viewport or tree");
                            }
                        }
                    }
                });

            // ── Clone data for Properties panel ──
            let selected_face = self.selected_face;
            let detailed_instances_clone2 = self.detailed_instances.clone();
            let model_name2 = self.current_model.name.clone();
            let model_vc = self.current_model.vertex_count;
            let model_tc = self.current_model.triangle_count;

            // ── Right: Properties ──
            egui::SidePanel::right("brepcad_properties")
                .default_width(280.0)
                .min_width(200.0)
                .resizable(true)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.brepcad_right_tab, BrepcadRightTab::Properties, "Props");
                        ui.selectable_value(&mut self.brepcad_right_tab, BrepcadRightTab::Constraints, "Constraints");
                        ui.selectable_value(&mut self.brepcad_right_tab, BrepcadRightTab::Dimensions, "Dimensions");
                        ui.selectable_value(&mut self.brepcad_right_tab, BrepcadRightTab::Material, "Material");
                    });
                    ui.separator();

                    match self.brepcad_right_tab {
                        BrepcadRightTab::Properties => {
                            // Face properties
                            if let Some((inst_idx, face_id)) = selected_face {
                                ui.label(egui::RichText::new("Selected: Face")
                                    .size(11.0).color(egui::Color32::from_rgb(0x89, 0xb4, 0xfa)));
                                ui.separator();
                                ui.collapsing("General", |ui| {
                                    ui.label(format!("Instance: #{}", inst_idx));
                                    ui.label(format!("Face ID: {}", face_id));
                                });
                                if let Some(inst) = detailed_instances_clone2.get(inst_idx) {
                                    if let Some(face_info) = inst.faces.iter().find(|f| f.face_id == face_id) {
                                        let tri_count_face = face_info.triangle_range.1.saturating_sub(face_info.triangle_range.0);
                                        ui.collapsing("Geometry", |ui| {
                                            ui.label(format!("Surface: {}", face_info.surface_type));
                                            ui.label(format!("Triangles: {}", tri_count_face));
                                            ui.label(format!("Forward: {}", face_info.forward));
                                            if face_info.is_void {
                                                ui.label(egui::RichText::new("Void face (internal cavity)")
                                                    .size(10.0).color(egui::Color32::from_rgb(0xf9, 0xe2, 0xaf)));
                                            }
                                        });
                                        ui.collapsing("Appearance", |ui| {
                                            ui.color_edit_button_srgb(&mut [100, 150, 200]);
                                            ui.label("Color");
                                        });
                                    }
                                }
                            } else if let Some(inst_idx) = selected_instance {
                                ui.label(egui::RichText::new(format!("Selected: Instance #{}", inst_idx))
                                    .size(11.0).color(egui::Color32::from_rgb(0x89, 0xb4, 0xfa)));
                                ui.separator();
                                if let Some(inst) = detailed_instances_clone2.get(inst_idx) {
                                    ui.collapsing("General", |ui| {
                                        ui.label(format!("Name: {}", inst.name));
                                        ui.label(format!("BREP ID: {}", inst.brep_id));
                                        ui.label(format!("Faces: {}", inst.faces.len()));
                                    });
                                }
                            } else {
                                ui.label(egui::RichText::new("No entity selected")
                                    .size(11.0).color(egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
                                ui.add_space(4.0);
                                ui.label(egui::RichText::new("Click a face or instance in the")
                                    .size(10.0).color(egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
                                ui.label(egui::RichText::new("viewport to see its properties.")
                                    .size(10.0).color(egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
                            }
                            ui.add_space(8.0);
                            ui.separator();
                            ui.collapsing("Model Info", |ui| {
                                ui.label(format!("Name: {}", model_name2));
                                ui.label(format!("Vertices: {}", model_vc));
                                ui.label(format!("Triangles: {}", model_tc));
                            });
                        }
                        BrepcadRightTab::Constraints => {
                            ui.label("Constraints (sketch mode)");
                        }
                        BrepcadRightTab::Dimensions => {
                            ui.label("Dimensions (sketch mode)");
                        }
                        BrepcadRightTab::Material => {
                            ui.label("No material assigned");
                            if ui.button("Assign Material…").clicked() {}
                            ui.separator();
                            ui.collapsing("Library", |ui| {
                                for cat in &["Metals", "Plastics", "Ceramics", "Composites", "Wood", "Glass", "Custom"] {
                                    ui.label(*cat);
                                }
                            });
                        }
                    }
                });

            // ── Bottom: Status bar ──
            egui::TopBottomPanel::bottom("brepcad_status_bar")
                .exact_height(24.0)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        let cam_pos = self.camera.position();
                        ui.label(format!("X {:.1}  Y {:.1}  Z {:.1}", cam_pos[0], cam_pos[1], cam_pos[2]));
                        ui.separator();
                        ui.label(format!("D {:.0}", self.camera.distance));
                        ui.separator();
                        ui.label(format!("Tool: {}", self.brepcad_active_tool));
                        ui.separator();
                        ui.label(format!("FPS: {:.0}", 1.0 / ui.input(|i| i.stable_dt).max(0.001)));
                        ui.separator();
                        ui.label("mm");
                        ui.separator();
                        ui.label(match (self.wireframe, self.show_edges) {
                            (true, _) => "Wireframe",
                            (false, false) => "Shaded",
                            (false, true) => "Shaded+Edges",
                        });
                        ui.separator();
                        ui.label(&self.brepcad_view_orientation);
                        ui.separator();
                        if self.selected_instance.is_some() || self.selected_face.is_some() {
                            ui.label(format!("Sel: {} item(s)",
                                self.selected_instance.map(|_| 1).unwrap_or(0) +
                                self.selected_face.map(|_| 1).unwrap_or(0)));
                        } else {
                            ui.label("Ready");
                        }
                    });
                });

            // ── Apply pending actions (after panels are rendered) ──
            if let Some(idx) = pending_instance_select {
                self.selected_instance = Some(idx);
                self.selected_face = None;
                self.highlighted_face = None;
                self.highlight_dirty = true;
                self.brepcad_status_msg = format!("Selected instance #{} ({})",
                    idx, self.detailed_instances.get(idx).map(|i| i.name.as_str()).unwrap_or("?"));
            }
            if let Some(idx) = pending_visibility_toggle {
                if self.hidden_instances.contains(&idx) {
                    self.hidden_instances.remove(&idx);
                    self.brepcad_status_msg = format!("Instance #{} shown", idx);
                } else {
                    self.hidden_instances.insert(idx);
                    self.brepcad_status_msg = format!("Instance #{} hidden", idx);
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
            if let Some(idx) = pending_instance_isolate {
                // Isolate: hide all other instances
                if self.hidden_instances.len() == self.detailed_instances.len() - 1
                    && !self.hidden_instances.contains(&idx)
                {
                    // Already isolated — restore all
                    self.hidden_instances.clear();
                    self.brepcad_status_msg = "All instances shown".to_string();
                } else {
                    self.hidden_instances.clear();
                    for i in 0..self.detailed_instances.len() {
                        if i != idx {
                            self.hidden_instances.insert(i);
                        }
                    }
                    self.selected_instance = Some(idx);
                    self.brepcad_status_msg = format!("Isolated instance #{}", idx);
                }
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
            // ── Apply face selection ──
            if let Some((inst_idx, fid)) = pending_face_select {
                self.selected_instance = Some(inst_idx);
                self.selected_face = Some((inst_idx, fid));
                self.highlighted_face = Some((inst_idx, fid));
                self.highlight_dirty = true;
                self.brepcad_status_msg = format!("Selected face #{} in instance #{}", fid, inst_idx);
            }
            // ── Apply face visibility toggle ──
            if let Some((inst_idx, fid)) = pending_face_visibility_toggle {
                if self.hidden_faces.contains(&(inst_idx, fid)) {
                    self.hidden_faces.remove(&(inst_idx, fid));
                    self.brepcad_status_msg = format!("Face #{} shown", fid);
                } else {
                    self.hidden_faces.insert((inst_idx, fid));
                    self.brepcad_status_msg = format!("Face #{} hidden", fid);
                    if self.selected_face == Some((inst_idx, fid)) {
                        self.selected_face = None;
                        self.highlighted_face = None;
                    }
                }
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
        }

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
                    // On mobile, skip click handling when a panel (structure/UV) is open,
                    // otherwise the CentralPanel steals the tap and clears selected_instance.
                    let mobile_panel_open = self.is_mobile && self.mobile_panel.is_some();
                    if response.clicked_by(egui::PointerButton::Primary) && !mobile_panel_open {
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
                                &self.hidden_faces,
                                &self.camera,
                                [local_x, local_y],
                                viewport,
                            ) {
                                // ─── BRepCAD Measure mode ───
                                if self.enable_brepcad_ui && self.brepcad_measure_mode != BrepcadMeasureMode::None {
                                    let pt = pick.point;
                                    match self.brepcad_measure_mode {
                                        BrepcadMeasureMode::Distance | BrepcadMeasureMode::Length => {
                                            if self.brepcad_measure_point1.is_none() {
                                                self.brepcad_measure_point1 = Some(pt);
                                                self.brepcad_status_msg = "Point 1 set. Click for point 2.".to_string();
                                            } else if self.brepcad_measure_point2.is_none() {
                                                self.brepcad_measure_point2 = Some(pt);
                                                let p1 = self.brepcad_measure_point1.unwrap();
                                                let dx = pt[0] - p1[0];
                                                let dy = pt[1] - p1[1];
                                                let dz = pt[2] - p1[2];
                                                let dist = (dx*dx + dy*dy + dz*dz).sqrt();
                                                self.brepcad_measure_result = format!("Distance: {:.3} mm", dist);
                                                self.brepcad_status_msg = self.brepcad_measure_result.clone();
                                                self.brepcad_measure_mode = BrepcadMeasureMode::None;
                                                self.brepcad_active_tool = "Select".to_string();
                                            }
                                        }
                                        BrepcadMeasureMode::Angle => {
                                            if self.brepcad_measure_point1.is_none() {
                                                self.brepcad_measure_point1 = Some(pt);
                                                self.brepcad_status_msg = "Vertex set. Click point 2.".to_string();
                                            } else if self.brepcad_measure_point2.is_none() {
                                                self.brepcad_measure_point2 = Some(pt);
                                                self.brepcad_status_msg = "Point 2 set. Click point 3.".to_string();
                                            } else if self.brepcad_measure_point3.is_none() {
                                                self.brepcad_measure_point3 = Some(pt);
                                                let vertex = self.brepcad_measure_point1.unwrap();
                                                let p2 = self.brepcad_measure_point2.unwrap();
                                                let p3 = self.brepcad_measure_point3.unwrap();
                                                // Vectors from vertex to p2 and p3
                                                let v1 = [p2[0] - vertex[0], p2[1] - vertex[1], p2[2] - vertex[2]];
                                                let v2 = [p3[0] - vertex[0], p3[1] - vertex[1], p3[2] - vertex[2]];
                                                let dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
                                                let len1 = (v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]).sqrt();
                                                let len2 = (v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]).sqrt();
                                                let angle = if len1 > 1e-9 && len2 > 1e-9 {
                                                    (dot / (len1 * len2)).clamp(-1.0, 1.0).acos().to_degrees()
                                                } else { 0.0 };
                                                self.brepcad_measure_result = format!("Angle: {:.2}°", angle);
                                                self.brepcad_status_msg = self.brepcad_measure_result.clone();
                                                self.brepcad_measure_mode = BrepcadMeasureMode::None;
                                                self.brepcad_active_tool = "Select".to_string();
                                            }
                                        }
                                        BrepcadMeasureMode::None => {}
                                    }
                                } else if ctrl_held {
                                    // Ctrl+click: select face
                                    if let Some(fid) = pick.face_id {
                                        self.selected_instance = Some(pick.instance_idx);
                                        self.selected_face = Some((pick.instance_idx, fid));
                                        self.highlighted_face = Some((pick.instance_idx, fid));
                                        self.highlight_dirty = true;
                                        self.scroll_to_face_id = Some(fid);
                                        self.log(&format!("Picked face #{} (instance #{})", fid, pick.instance_idx));
                                        // ─── Sync 3D viewport face pick → UV Breakdown ──
                                        // Same logic as the structure-panel face
                                        // pick: resolve face_id to positional
                                        // index in the instance's faces array,
                                        // then set the UV window's face index.
                                        if let Some(inst) = self.detailed_instances.get(pick.instance_idx) {
                                            if let Some(pos) = inst.faces.iter().position(|f| f.face_id == fid) {
                                                if self.uv_window_face_idx != Some(pos) {
                                                    self.uv_window_face_idx = Some(pos);
                                                    self.uv_window_zoom = 1.0;
                                                    self.uv_window_pan = [0.0, 0.0];
                                                    self.uv_window_prev_face_idx = Some(pos);
                                                }
                                            }
                                        }
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
                                    // ─── Switching instance → UV window must reset ──
                                    // The active solid for the UV window changes
                                    // when the user picks a different instance,
                                    // so the previously-shown face index is no
                                    // longer valid. Clear it AND invalidate the
                                    // cached breakdown so the next frame
                                    // recomputes from the new instance.
                                    if self.uv_window_face_idx.is_some() {
                                        self.uv_window_face_idx = None;
                                        self.uv_window_prev_face_idx = None;
                                        self.solid_uv_breakdown = None;
                                    }
                                    self.log(&format!("Picked instance #{}", pick.instance_idx));
                                    // Navigate structure tree
                                    if let Some(ref tree) = self.assembly_tree {
                                        let (path, target) = find_instance_path(tree, pick.instance_idx);
                                        self.open_tree_nodes = path.into_iter().collect();
                                        self.scroll_to_tree_node = target;
                                    }
                                }
                            } else {
                                // Clicked on empty space.
                                //
                                // Behavior:
                                // 1. If any instances or faces are hidden (e.g. user
                                //    isolated a single face), restore the full model
                                //    visibility — clicking empty space "exits
                                //    isolation mode".
                                // 2. Otherwise (nothing hidden), just deselect the
                                //    current selection.
                                let has_hidden = !self.hidden_instances.is_empty()
                                    || !self.hidden_faces.is_empty();
                                if has_hidden {
                                    self.hidden_instances.clear();
                                    self.hidden_faces.clear();
                                    self.highlight_dirty = true;
                                    self.edge_dirty = true;
                                    self.wireframe_overlay_dirty = true;
                                    self.log("Empty click: restored visibility of all instances and faces");
                                }
                                // Always deselect on empty click
                                self.selected_instance = None;
                                self.selected_face = None;
                                self.highlighted_face = None;
                                self.highlight_dirty = true;
                                self.open_tree_nodes.clear();
                                self.scroll_to_tree_node = None;
                                self.scroll_to_face_id = None;
                            }
                        }
                    }

                    if response.dragged_by(egui::PointerButton::Primary) && !mobile_panel_open {
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
                    } else {
                        // Fill any remaining [0,0,1] placeholder normals from merge
                        // (curved surfaces may not have set face_normals, leaving
                        // placeholders that cause incorrect flat shading)
                        self.mesh.fill_missing_face_normals();
                    }
                    self.mesh.ensure_colors([0.62, 0.65, 0.70, 1.0]);

                    if let Some(ref rs) = self.render_state {
                        let section_plane = if self.enable_brepcad_ui && self.brepcad_section_enabled {
                            Some((self.brepcad_section_axis as usize, self.brepcad_section_position))
                        } else {
                            None
                        };
                        let (vertices, indices, _new_ranges) = mesh_to_gpu_data(&self.mesh, self.highlighted_face, self.selected_instance, &self.instance_triangle_ranges, &self.hidden_instances, &self.hidden_faces, section_plane);
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

                // ─── BRepCAD measure overlay ───
                if self.enable_brepcad_ui && !self.brepcad_measure_result.is_empty() {
                    let pos = egui::pos2(rect.left() + 10.0, rect.top() + 60.0);
                    ui.painter().text(
                        pos,
                        egui::Align2::LEFT_TOP,
                        &self.brepcad_measure_result,
                        egui::FontId::proportional(14.0),
                        egui::Color32::from_rgb(0xa6, 0xe3, 0xa1),
                    );
                }

                // ─── BRepCAD measure mode indicator ───
                if self.enable_brepcad_ui && self.brepcad_measure_mode != BrepcadMeasureMode::None {
                    let mode_text = match self.brepcad_measure_mode {
                        BrepcadMeasureMode::Distance => "📏 Measure Distance — click points in viewport (ESC to cancel)",
                        BrepcadMeasureMode::Angle => "📐 Measure Angle — click vertex + 2 points (ESC to cancel)",
                        BrepcadMeasureMode::Length => "📏 Measure Length — click 2 points (ESC to cancel)",
                        BrepcadMeasureMode::None => "",
                    };
                    let pos = egui::pos2(rect.center().x, rect.top() + 40.0);
                    ui.painter().text(
                        pos,
                        egui::Align2::CENTER_TOP,
                        mode_text,
                        egui::FontId::proportional(14.0),
                        egui::Color32::from_rgb(0xf9, 0xe2, 0xaf),
                    );
                }

                // ─── GD&T 3D annotations overlay ───
                if self.show_gdt_annotations {
                    // Lazy-load GD&T data if not yet extracted
                    if self.gdt_data.is_none() {
                        self.gdt_data = self.last_step_file.as_ref()
                            .map(|sf| draper_step::pmi::extract_gdt(sf));
                    }
                    self.draw_gdt_annotations(ui, rect);
                }

                // ─── Loading progress overlay ───
                if self.is_loading && self.total_instance_count > 0 {
                    // Progress includes partial BREP progress (chunked triangulation).
                    // Each BREP contributes 1/total_instance_count to the total progress.
                    // If we're mid-BREP, add the fraction of that BREP's faces done.
                    let base_progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                    let partial_brep_progress = if self.chunked_brep_faces_total > 0 {
                        (self.chunked_brep_faces_done as f32 / self.chunked_brep_faces_total as f32)
                            / self.total_instance_count as f32
                    } else {
                        0.0
                    };
                    let progress = (base_progress + partial_brep_progress).min(1.0);
                    let bar_w = rect.width() * 0.6;
                    let bar_h = 20.0;
                    let bar_x = rect.center().x - bar_w / 2.0;
                    let bar_y = rect.bottom() - 60.0;
                    let bar_rect = egui::Rect::from_min_max(
                        egui::pos2(bar_x, bar_y),
                        egui::pos2(bar_x + bar_w, bar_y + bar_h),
                    );

                    // Compute elapsed time + ETA for better mobile UX
                    let elapsed_secs = self.loading_start.map(|t| t.elapsed().as_secs_f64()).unwrap_or(0.0);
                    let eta_text = if self.triangulated_count > 0 && elapsed_secs > 0.5 {
                        let per_inst = elapsed_secs / self.triangulated_count as f64;
                        let remaining = (self.total_instance_count - self.triangulated_count) as f64 * per_inst;
                        format!(" · ETA: {:.0}s", remaining.max(0.0))
                    } else {
                        String::new()
                    };

                    // Background
                    ui.painter().rect_filled(
                        egui::Rect::from_min_max(
                            egui::pos2(bar_x - 10.0, bar_y - 25.0),
                            egui::pos2(bar_x + bar_w + 10.0, bar_y + bar_h + 10.0),
                        ),
                        4.0,
                        egui::Color32::from_rgba_premultiplied(0, 0, 0, 160),
                    );

                    // Label — includes intra-BREP face progress when chunking
                    let face_info = if self.chunked_brep_faces_total > 0 {
                        format!(" [{} faces]", self.chunked_brep_faces_done)
                    } else {
                        String::new()
                    };
                    ui.painter().text(
                        egui::pos2(bar_x + bar_w / 2.0, bar_y - 8.0),
                        egui::Align2::CENTER_CENTER,
                        format!("Triangulating: {}/{} ({:.0}%) · {:.0}s{}{}",
                            self.triangulated_count, self.total_instance_count, progress * 100.0,
                            elapsed_secs, eta_text, face_info),
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

                    // ─── Cancel button (always visible during loading) ───
                    // Critical for mobile users — if loading takes too long,
                    // they can bail out and see what was loaded so far.
                    let btn_w = 90.0;
                    let btn_h = 24.0;
                    let btn_x = bar_x + bar_w + 14.0;
                    let btn_y = bar_y - 2.0;
                    let btn_rect = egui::Rect::from_min_size(
                        egui::pos2(btn_x, btn_y),
                        egui::vec2(btn_w, btn_h),
                    );
                    // Only show the cancel button if there's room (desktop wide layout)
                    if btn_x + btn_w < rect.right() - 10.0 {
                        let cancel_btn = egui::Button::new(
                            egui::RichText::new("Cancel").size(11.0).color(egui::Color32::WHITE)
                        ).fill(egui::Color32::from_rgb(180, 60, 60));
                        let cancel_resp = ui.put(btn_rect, cancel_btn);
                        if cancel_resp.clicked() {
                            self.cancel_loading(true);
                        }
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

        // ═══ GD&T panel ══════════════════════════════════════════════════════
        self.draw_gdt_window(ctx);

        // ═══ Pending UV SVG export ═════════════════════════════════════════════
        if self.pending_solid_uv_svg_export {
            self.pending_solid_uv_svg_export = false;
            // Resolve the active solid: prefer `detailed_instances` (STEP/JSON)
            // when populated — `current_solid` may be stale from a previous
            // primitive/NURBS session. Fall back to `current_solid` for
            // primitive/NURBS gallery loaders (where detailed_instances is
            // always empty). This matches `draw_uv_window`'s logic below.
            let active_detailed: Option<&DetailedMeshInstance> = if !self.detailed_instances.is_empty() {
                let chosen_idx = self
                    .selected_instance
                    .or_else(|| Some(0));
                chosen_idx
                    .and_then(|i| self.detailed_instances.get(i))
            } else {
                None
            };
            let active_solid: Option<Solid> = if let Some(di) = active_detailed {
                Some(solid_from_detailed_instance(di))
            } else {
                self.current_solid.clone()
            };
            // Ensure the breakdown is computed for the active solid.
            if self.solid_uv_breakdown.is_none() {
                if let Some(ref solid) = active_solid {
                    let name = self.current_model.name.clone();
                    // Pass the DetailedMeshInstance so we can use the actual
                    // FaceInfo UV triangles + boundary polylines (the real
                    // triangulation with holes/arcs) instead of falling back
                    // to a synthetic square grid.
                    self.solid_uv_breakdown = Some(compute_solid_uv_breakdown_with_detailed(
                        solid,
                        &name,
                        active_detailed,
                    ));
                }
            }
            if let Some(ref breakdown) = self.solid_uv_breakdown {
                if let Some(face_idx) = self.uv_window_face_idx {
                    if let Some(face_uv) = breakdown.faces.get(face_idx) {
                    // Get the surface from the active solid for grid-point rendering.
                    let surface: Option<Surface> = active_solid.as_ref().and_then(|s| {
                        s.outer_shell.as_ref().and_then(|sh| {
                            sh.faces.get(face_idx)
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
                        self.log_warning("UV export: face index out of range");
                    }
                } else {
                    self.log_warning("UV export: no face selected");
                }
            } else {
                self.log_warning("UV export: no solid loaded");
            }
        }

        // ═══ BRepCAD UI: command palette, dialogs, view cube, status toast ═══════════
        if self.enable_brepcad_ui {
            // View Cube (top-right corner of viewport)
            if let Some(orient) = crate::ui::view_modes::render_view_cube(ctx) {
                self.camera.look_from_direction(orient.direction());
                self.brepcad_view_orientation = orient.label().to_string();
                let (bbox_min, bbox_max) = self.mesh.bounding_box();
                self.camera.fit_to_bounding_box(
                    [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                    [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                );
                self.brepcad_status_msg = format!("View: {}", orient.label());
            }

            // Display style switcher (bottom-right corner)
            {
                let mut style = if self.wireframe { crate::ui::DisplayStyle::Wireframe }
                    else if self.show_edges { crate::ui::DisplayStyle::ShadedWithEdges }
                    else { crate::ui::DisplayStyle::Shaded };
                crate::ui::view_modes::render_display_style_switcher(ctx, &mut style);
                let new_wf = matches!(style, crate::ui::DisplayStyle::Wireframe);
                let new_edges = matches!(style, crate::ui::DisplayStyle::ShadedWithEdges);
                if new_wf != self.wireframe || new_edges != self.show_edges {
                    self.wireframe = new_wf;
                    self.show_edges = new_edges;
                }
            }

            // Section cut panel (floating, top-left of viewport when enabled)
            if self.brepcad_section_enabled {
                egui::Area::new(egui::Id::new("brepcad_section_panel"))
                    .anchor(egui::Align2::LEFT_TOP, egui::vec2(10.0, 60.0))
                    .order(egui::Order::Foreground)
                    .show(ctx, |ui| {
                        egui::Frame::new()
                            .fill(egui::Color32::from_black_alpha(200))
                            .corner_radius(6.0)
                            .inner_margin(egui::Margin::symmetric(10, 8))
                            .show(ui, |ui| {
                                ui.label(egui::RichText::new("📐 Section Cut").size(12.0).color(egui::Color32::from_rgb(0xf9, 0xe2, 0xaf)));
                                ui.separator();
                                ui.horizontal(|ui| {
                                    ui.label("Axis:");
                                    if ui.radio_value(&mut self.brepcad_section_axis, 0, "X").clicked() { self.mesh_dirty = true; }
                                    if ui.radio_value(&mut self.brepcad_section_axis, 1, "Y").clicked() { self.mesh_dirty = true; }
                                    if ui.radio_value(&mut self.brepcad_section_axis, 2, "Z").clicked() { self.mesh_dirty = true; }
                                });
                                let (bbox_min, bbox_max) = self.mesh.bounding_box();
                                let min_val = match self.brepcad_section_axis { 0 => bbox_min.x, 1 => bbox_min.y, _ => bbox_min.z } as f32;
                                let max_val = match self.brepcad_section_axis { 0 => bbox_max.x, 1 => bbox_max.y, _ => bbox_max.z } as f32;
                                ui.horizontal(|ui| {
                                    ui.label("Position:");
                                    let slider = egui::Slider::new(&mut self.brepcad_section_position, min_val..=max_val)
                                        .suffix(" mm");
                                    if ui.add(slider).changed() {
                                        self.mesh_dirty = true;
                                    }
                                });
                                ui.horizontal(|ui| {
                                    if ui.button("Fit").clicked() {
                                        self.brepcad_section_position = (min_val + max_val) * 0.5;
                                        self.mesh_dirty = true;
                                    }
                                    if ui.button("Close").clicked() {
                                        self.brepcad_section_enabled = false;
                                        self.mesh_dirty = true;
                                    }
                                });
                            });
                    });
            }

            // Command palette (Ctrl+Shift+P)
            if let Some(cmd) = crate::ui::command_palette::render_command_palette(ctx, &mut self.brepcad_command_palette) {
                if let Some(action) = Self::command_name_to_action(&cmd) {
                    let msg = self.handle_brepcad_action(&action);
                    if !msg.is_empty() {
                        self.brepcad_status_msg = msg;
                    }
                }
            }

            // Keyboard shortcut: Ctrl+, for Options
            if ctx.input(|i| i.key_pressed(egui::Key::Comma) && (i.modifiers.ctrl || i.modifiers.command)) {
                self.brepcad_dialog = crate::ui::dialogs::DialogType::Options;
            }

            // Dialogs
            if let Some(action) = crate::ui::dialogs::render_dialog(ctx, &mut self.brepcad_dialog) {
                let msg = self.handle_brepcad_dialog_action(&action);
                if !msg.is_empty() {
                    self.brepcad_status_msg = msg;
                }
            }

            // Marking menu (Space key)
            if ctx.input(|i| i.key_pressed(egui::Key::Space)) {
                self.brepcad_marking_menu_visible = !self.brepcad_marking_menu_visible;
            }
            if let Some(_action) = crate::ui::context_menus::marking_menu(ctx, &mut self.brepcad_marking_menu_visible) {
                // Marking menu actions could be wired here
            }

            // Status toast
            if !self.brepcad_status_msg.is_empty() {
                egui::Area::new(egui::Id::new("brepcad_status_toast"))
                    .anchor(egui::Align2::CENTER_BOTTOM, [0.0, -30.0])
                    .order(egui::Order::Foreground)
                    .show(ctx, |ui| {
                        egui::Frame::new()
                            .fill(egui::Color32::from_black_alpha(200))
                            .corner_radius(6.0)
                            .inner_margin(egui::Margin::symmetric(12, 6))
                            .show(ui, |ui| {
                                ui.label(&self.brepcad_status_msg);
                            });
                    });
            }

            // Ctrl+Z = Undo, Ctrl+Shift+Z = Redo (BRepCAD mode)
            if ctx.input(|i| i.key_pressed(egui::Key::Z) && (i.modifiers.ctrl || i.modifiers.command) && !i.modifiers.shift) {
                if self.brepcad_undo() {
                    // status updated by brepcad_undo
                } else {
                    self.brepcad_status_msg = "Nothing to undo".to_string();
                }
            }
            if ctx.input(|i| i.key_pressed(egui::Key::Z) && (i.modifiers.ctrl || i.modifiers.command) && i.modifiers.shift) {
                if self.brepcad_redo() {
                    // status updated by brepcad_redo
                } else {
                    self.brepcad_status_msg = "Nothing to redo".to_string();
                }
            }

            // F = Fit view
            if ctx.input(|i| i.key_pressed(egui::Key::F) && !i.modifiers.ctrl && !i.modifiers.command) {
                let (bbox_min, bbox_max) = self.mesh.bounding_box();
                self.camera.fit_to_bounding_box(
                    [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                    [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                );
                self.brepcad_status_msg = "Fit to view".to_string();
            }

            // ESC = cancel measure mode
            if ctx.input(|i| i.key_pressed(egui::Key::Escape)) && self.brepcad_measure_mode != BrepcadMeasureMode::None {
                self.brepcad_measure_mode = BrepcadMeasureMode::None;
                self.brepcad_measure_point1 = None;
                self.brepcad_measure_point2 = None;
                self.brepcad_measure_point3 = None;
                self.brepcad_active_tool = "Select".to_string();
                self.brepcad_status_msg = "Measure cancelled".to_string();
            }

            // ─── Parameter dialog ───
            if self.brepcad_param_dialog_open {
                let mut open = self.brepcad_param_dialog_open;
                egui::Window::new("Parameters")
                    .open(&mut open)
                    .resizable(true)
                    .default_width(500.0)
                    .default_height(400.0)
                    .show(ctx, |ui| {
                        // Add new parameter
                        ui.horizontal(|ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(&mut self.brepcad_new_param_name);
                            ui.label("Formula:");
                            ui.text_edit_singleline(&mut self.brepcad_new_param_formula);
                            if ui.button("Add").clicked() && !self.brepcad_new_param_name.is_empty() {
                                let name = self.brepcad_new_param_name.clone();
                                let formula = self.brepcad_new_param_formula.clone();
                                if formula.is_empty() {
                                    self.brepcad_set_param(&name, 0.0);
                                } else {
                                    self.brepcad_set_param_formula(&name, &formula, "mm");
                                }
                                self.brepcad_new_param_name.clear();
                                self.brepcad_new_param_formula.clear();
                                self.brepcad_eval_params();
                            }
                        });
                        ui.separator();

                        // Parameter table
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            let mut params_to_remove = Vec::new();
                            let mut params_to_update: Vec<(String, f64, Option<String>)> = Vec::new();

                            // Collect names first to avoid borrow issues
                            let names: Vec<String> = self.brepcad_parameters.keys().cloned().collect();

                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new("Name").size(11.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
                                ui.add_space(80.0);
                                ui.label(egui::RichText::new("Value").size(11.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
                                ui.add_space(60.0);
                                ui.label(egui::RichText::new("Formula").size(11.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
                                ui.add_space(80.0);
                                ui.label(egui::RichText::new("Unit").size(11.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
                            });
                            ui.separator();

                            for name in &names {
                                let entry = self.brepcad_parameters.get(name).cloned();
                                if let Some((value, formula, unit)) = entry {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new(name).size(11.0));
                                        ui.add_space(20.0);

                                        let mut val_str = format!("{:.3}", value);
                                        ui.add(egui::TextEdit::singleline(&mut val_str).desired_width(60.0));
                                        if let Ok(new_val) = val_str.parse::<f64>() {
                                            if (new_val - value).abs() > 1e-9 {
                                                params_to_update.push((name.clone(), new_val, None));
                                            }
                                        }

                                        ui.add_space(10.0);
                                        let mut formula_str = formula.clone().unwrap_or_default();
                                        ui.add(egui::TextEdit::singleline(&mut formula_str).desired_width(80.0));
                                        if !formula_str.is_empty() && Some(formula_str.clone()) != formula {
                                            params_to_update.push((name.clone(), 0.0, Some(formula_str)));
                                        }

                                        ui.add_space(10.0);
                                        ui.label(egui::RichText::new(&unit).size(10.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));

                                        ui.add_space(10.0);
                                        if ui.small_button("✕").clicked() {
                                            params_to_remove.push(name.clone());
                                        }
                                    });
                                }
                            }

                            // Apply updates
                            for (name, new_val, new_formula) in params_to_update {
                                if let Some(entry) = self.brepcad_parameters.get_mut(&name) {
                                    if let Some(f) = new_formula {
                                        entry.1 = Some(f);
                                    } else {
                                        entry.1 = None;
                                        entry.0 = new_val;
                                    }
                                }
                            }
                            self.brepcad_eval_params();

                            // Apply removals
                            for name in params_to_remove {
                                self.brepcad_parameters.remove(&name);
                            }
                        });

                        ui.separator();
                        ui.horizontal(|ui| {
                            if ui.button("Add Defaults (W, H, D, R)").clicked() {
                                self.brepcad_set_param("Width", 100.0);
                                self.brepcad_set_param("Height", 80.0);
                                self.brepcad_set_param("Depth", 60.0);
                                self.brepcad_set_param("Radius", 50.0);
                                self.brepcad_set_param_formula("Diameter", "Radius * 2", "mm");
                                self.brepcad_set_param_formula("Volume", "Width * Height * Depth", "mm³");
                            }
                            if ui.button("Clear All").clicked() {
                                self.brepcad_parameters.clear();
                            }
                            ui.label(format!("({} params)", self.brepcad_parameters.len()));
                        });
                    });
                self.brepcad_param_dialog_open = open;
            }

            // ─── Feature Timeline panel ───
            if self.brepcad_timeline_open {
                let mut timeline_open = self.brepcad_timeline_open;
                egui::Window::new("Feature Timeline")
                    .open(&mut timeline_open)
                    .resizable(true)
                    .default_width(350.0)
                    .default_height(300.0)
                    .anchor(egui::Align2::CENTER_BOTTOM, [0.0, -30.0])
                    .show(ctx, |ui| {
                        if self.brepcad_timeline.is_empty() {
                            ui.label(egui::RichText::new("No operations yet. Insert a primitive or modify the model to build the timeline.")
                                .size(11.0).color(egui::Color32::from_rgb(0x6c, 0x70, 0x86)));
                        } else {
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new(format!("{} operations", self.brepcad_timeline.len()))
                                    .size(11.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
                                if ui.small_button("Clear").clicked() {
                                    self.brepcad_timeline.clear();
                                    self.brepcad_timeline_rollback = None;
                                }
                            });
                            ui.separator();

                            egui::ScrollArea::vertical().show(ui, |ui| {
                                let rollback = self.brepcad_timeline_rollback;
                                let total = self.brepcad_timeline.len();
                                // Collect pending rollback target
                                let mut rollback_target: Option<usize> = None;
                                let entries: Vec<(usize, String)> = self.brepcad_timeline
                                    .iter().enumerate().rev()
                                    .map(|(i, (name, _))| (i, name.clone()))
                                    .collect();
                                for (i, name) in &entries {
                                    let is_current = rollback == Some(*i);
                                    let is_rolled_back = rollback.is_some() && *i > rollback.unwrap();
                                    let bg = if is_current {
                                        egui::Color32::from_rgb(0x09, 0x47, 0x71)
                                    } else if is_rolled_back {
                                        egui::Color32::from_rgb(0x1e, 0x1e, 0x2e)
                                    } else {
                                        egui::Color32::TRANSPARENT
                                    };
                                    let frame = egui::Frame::new().fill(bg).inner_margin(egui::Margin::symmetric(6, 3));
                                    frame.show(ui, |ui| {
                                        ui.horizontal(|ui| {
                                            let num = total - i;
                                            let icon = if is_rolled_back { "○" } else { "●" };
                                            let color = if is_rolled_back {
                                                egui::Color32::from_rgb(0x6c, 0x70, 0x86)
                                            } else {
                                                egui::Color32::from_rgb(0xa6, 0xe3, 0xa1)
                                            };
                                            ui.label(egui::RichText::new(format!("{} #{}", icon, num))
                                                .size(10.0).color(color));
                                            ui.label(egui::RichText::new(name).size(11.0));
                                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                if ui.small_button("↩").clicked() {
                                                    rollback_target = Some(*i);
                                                }
                                            });
                                        });
                                    });
                                }
                                if let Some(idx) = rollback_target {
                                    self.brepcad_timeline_rollback_to(idx);
                                    let num = total - idx;
                                    self.brepcad_status_msg = format!("Rolled back to #{}", num);
                                }
                            });
                        }
                    });
                self.brepcad_timeline_open = timeline_open;
            }
        }
    }
}

impl ViewerApp {

    /// Draw the UV breakdown window for the current solid.
    ///
    /// Shows a face list (selectable, scrollable) of the active solid,
    /// U/V division sliders, the UV grid canvas (rendered directly via
    /// painter), and Save / Close buttons. On WASM the Save button
    /// triggers a browser SVG download; on native it opens an rfd
    /// save-file dialog.
    ///
    /// Works for BOTH primitives/NURBS (uses `self.current_solid`) AND
    /// STEP files (builds a Solid on-the-fly from the selected
    /// `detailed_instances` entry, falling back to the first instance
    /// when nothing is selected).
    ///
    /// # Active face
    ///
    /// The window's face selection (`uv_window_face_idx`) is an
    /// `Option<usize>`:
    ///   - `None` → no face is active. The canvas is hidden and a
    ///     "Select a face" prompt is shown instead. This happens after
    ///     loading a new solid, switching instances, or clicking "Clear
    ///     Selection".
    ///   - `Some(idx)` → the face at position `idx` in the active
    ///     solid's outer shell faces array is shown.
    ///
    /// The face selection is synced bi-directionally with the structure
    /// panel / 3D viewport:
    ///   - When the user clicks a face in the structure panel or
    ///     Ctrl+clicks a face in the 3D viewport, `selected_face` is
    ///     set AND `uv_window_face_idx` is set to the corresponding
    ///     positional index (for STEP files; primitives/NURBS don't
    ///     have STEP face IDs so this sync is skipped there).
    ///   - When the user clicks a face in the UV window's own face
    ///     list, `uv_window_face_idx` is set AND, for STEP files,
    ///     `selected_face`/`highlighted_face` are updated so the 3D
    ///     viewport highlights the same face.
    fn draw_uv_window(&mut self, ctx: &egui::Context) {
        if !self.show_uv_window {
            return;
        }

        // ─── Determine the "active solid" for the UV window ───────────────
        // Two sources, in priority order:
        //   1. `detailed_instances` (STEP/JSON files) — always preferred
        //      when non-empty, because `uv_window_face_idx` is keyed to the
        //      positional order inside `solid_from_detailed_instance(...)`
        //      and the structure-panel / 3D-viewport face-pick sync only
        //      fires when `detailed_instances` is populated. If we fell
        //      back to `current_solid` here, the indices would be off and
        //      the canvas would silently render nothing.
        //   2. `current_solid` (primitive/NURBS gallery loaders) — used
        //      when `detailed_instances` is empty. Primitive loaders
        //      explicitly clear `detailed_instances` before `load_mesh`,
        //      so this case is unambiguous.
        //
        // Defensive belt-and-suspenders: even if a future regression
        // forgets to clear `current_solid` when a STEP/JSON file is
        // loaded, the preference for `detailed_instances` here ensures
        // the UV Breakdown window still shows the right solid.
        let active_detailed: Option<&DetailedMeshInstance> = if !self.detailed_instances.is_empty() {
            // STEP/JSON file path. Pick the selected instance, or fall
            // back to the first instance so the window always shows
            // SOMETHING.
            let chosen_idx = self
                .selected_instance
                .or_else(|| Some(0));
            chosen_idx
                .and_then(|i| self.detailed_instances.get(i))
        } else {
            None
        };
        let active_solid: Option<Solid> = if let Some(di) = active_detailed {
            Some(solid_from_detailed_instance(di))
        } else {
            // Primitive / NURBS gallery path.
            self.current_solid.clone()
        };

        // Reset zoom/pan when the user switches faces — the new face has a
        // different UV domain, so the previous pan/zoom would be meaningless.
        if self.uv_window_prev_face_idx != self.uv_window_face_idx {
            self.uv_window_zoom = 1.0;
            self.uv_window_pan = [0.0, 0.0];
            self.uv_window_prev_face_idx = self.uv_window_face_idx;
        }
        // Compute breakdown on demand if not cached.
        if self.solid_uv_breakdown.is_none() {
            if let Some(ref solid) = active_solid {
                let name = self.current_model.name.clone();
                self.solid_uv_breakdown = Some(compute_solid_uv_breakdown_with_detailed(
                    solid,
                    &name,
                    active_detailed,
                ));
                // If the active face index is now out of range (e.g. the
                // new solid has fewer faces than the previous one), clear
                // it — the user must pick a new face from the list.
                if let Some(ref b) = self.solid_uv_breakdown {
                    if let Some(idx) = self.uv_window_face_idx {
                        if idx >= b.faces.len() {
                            self.uv_window_face_idx = None;
                        }
                    }
                }
            }
        }

        let mut window_open = self.show_uv_window;
        // We need to clone or borrow carefully. The breakdown is in self,
        // but we also need to mutate self inside the window. So we take
        // the breakdown out temporarily.
        let breakdown_taken = self.solid_uv_breakdown.take();
        // For surface lookup inside the window, use the active solid
        // (which may be a synthesized STEP solid, not self.current_solid).
        let current_solid_taken = active_solid.clone();
        let model_name = self.current_model.name.clone();
        // Track whether the user clicked a face in the UV window's own
        // face list — used after the egui closure to sync
        // `selected_face` / `highlighted_face` for STEP files (so the 3D
        // viewport highlights the same face the user picked in the UV
        // window). Captured as a local var because we can't safely borrow
        // `self.detailed_instances` from inside the egui closure that
        // already borrows `self` mutably.
        let mut uv_window_face_clicked: Option<usize> = None;

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

                // ─── Face list (scrollable, selectable) ──────────────────
                // Mirrors the structure panel's Faces section: each face
                // is a `selectable_label` showing its index, surface type,
                // and boundary/hole summary. Clicking a face sets
                // `uv_window_face_idx` AND (captured via
                // `uv_window_face_clicked`) triggers a reverse sync into
                // `selected_face` for STEP files after the closure.
                ui.label(egui::RichText::new(
                    format!("Faces of active solid ({}):", breakdown.faces.len())
                ).size(11.0).color(egui::Color32::from_rgb(170, 170, 190)));
                egui::ScrollArea::vertical()
                    .id_salt("uv_window_face_list")
                    .max_height(160.0)
                    .show(ui, |ui| {
                        for f in &breakdown.faces {
                            let is_selected = self.uv_window_face_idx == Some(f.face_idx);
                            let label = format!("#{} {} (outer pts: {}, holes: {})",
                                f.face_idx, f.surface_type,
                                f.outer_polylines.iter().map(|p| p.len()).sum::<usize>(),
                                f.inner_polylines.len());
                            let response = ui.selectable_label(is_selected, &label);
                            if response.clicked() {
                                self.uv_window_face_idx = Some(f.face_idx);
                                // Reset zoom/pan on face change.
                                self.uv_window_zoom = 1.0;
                                self.uv_window_pan = [0.0, 0.0];
                                self.uv_window_prev_face_idx = Some(f.face_idx);
                                uv_window_face_clicked = Some(f.face_idx);
                            }
                            response.on_hover_text(format!(
                                "Face #{}\nSurface: {}\nOuter pts: {}\nHoles: {}\nforward: {}",
                                f.face_idx, f.surface_type,
                                f.outer_polylines.iter().map(|p| p.len()).sum::<usize>(),
                                f.inner_polylines.len(),
                                f.forward,
                            ));
                        }
                    });

                ui.separator();

                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("U divs:").size(11.0));
                    ui.add(egui::DragValue::new(&mut self.uv_window_u_divs).range(2..=50));
                    ui.label(egui::RichText::new("V divs:").size(11.0));
                    ui.add(egui::DragValue::new(&mut self.uv_window_v_divs).range(2..=50));
                });

                // ─── Zoom / Pan controls ───────────────────────────────────
                // The canvas is a fixed-size square — to let users inspect
                // dense UV triangulations in detail, we expose a zoom slider
                // (with + / − buttons for click-to-zoom) and a "Reset View"
                // button. Pan is performed by left-dragging the canvas.
                // These controls are only useful when a face is actually
                // selected, but we keep them visible (just dimmed) when no
                // face is selected so the user can see what's available.
                let controls_enabled = self.uv_window_face_idx.is_some();
                ui.horizontal(|ui| {
                    ui.add_enabled_ui(controls_enabled, |ui| {
                        ui.label(egui::RichText::new("Zoom:").size(11.0));
                        if ui.button(egui::RichText::new("−").size(13.0)).clicked() {
                            self.uv_window_zoom = (self.uv_window_zoom / 1.25_f32).max(0.25);
                        }
                        ui.add(
                            egui::Slider::new(&mut self.uv_window_zoom, 0.25..=20.0)
                                .step_by(0.05)
                                .clamping(egui::SliderClamping::Always)
                                .fixed_decimals(2)
                                .text("x"),
                        );
                        if ui.button(egui::RichText::new("+").size(13.0)).clicked() {
                            self.uv_window_zoom = (self.uv_window_zoom * 1.25_f32).min(20.0);
                        }
                        if ui.button("Reset View").clicked() {
                            self.uv_window_zoom = 1.0;
                            self.uv_window_pan = [0.0, 0.0];
                            self.uv_window_aspect_override = 1.0;
                        }
                    });
                });
                // ─── Aspect-ratio override ─────────────────────────────────
                // Lets the user squeeze/stretch the U axis relative to V so
                // that surfaces with U-range ≫ V-range (e.g. cylinder:
                // U=2π ≈ 6.28, V=height) become easier to inspect — without
                // this, the canvas becomes wide and short and individual
                // triangles are hard to distinguish. The default 1.0 keeps
                // the natural aspect.
                ui.horizontal(|ui| {
                    ui.add_enabled_ui(controls_enabled, |ui| {
                        ui.label(egui::RichText::new("U-squeeze:").size(11.0));
                        ui.add(
                            egui::Slider::new(&mut self.uv_window_aspect_override, 0.1..=2.0)
                                .step_by(0.05)
                                .clamping(egui::SliderClamping::Always)
                                .fixed_decimals(2)
                                .text("(1.0=natural)"),
                        );
                    });
                });
                // ─── Metric UV toggle ──────────────────────────────────────
                // When enabled (default), the UV canvas scales the U and V axes
                // by the surface's metric scale factors so that the UV rectangle
                // shows the true surface proportions (e.g., for a cylinder the U
                // axis is scaled by the radius to show arc-length instead of angle).
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.uv_window_metric_uv, "Metric UV");
                    if self.uv_window_metric_uv {
                        ui.label(egui::RichText::new("(axes scaled to surface units)")
                            .size(9.0).color(egui::Color32::from_rgb(120, 180, 120)));
                    } else {
                        ui.label(egui::RichText::new("(raw parameter space)")
                            .size(9.0).color(egui::Color32::from_rgb(180, 120, 120)));
                    }
                });
                ui.label(egui::RichText::new(
                    "Tip: drag the canvas to pan, use + / − or the slider to zoom. Metric UV shows true surface proportions."
                ).size(10.0).color(egui::Color32::from_rgb(140, 140, 160)));

                ui.horizontal(|ui| {
                    if ui.add_enabled(controls_enabled, egui::Button::new("Save UV as SVG...")).clicked() {
                        self.pending_solid_uv_svg_export = true;
                    }
                    if ui.button("Recompute").clicked() {
                        // Force recompute on next frame
                        self.solid_uv_breakdown = None;
                    }
                });

                ui.separator();

                // ─── Canvas — only shown when a face is selected ──────────
                // If `uv_window_face_idx` is `None`, show a "Select a face"
                // prompt instead of the canvas. This is the core of the
                // "no active face → no grid" requirement: when the user
                // switches solids, clears the selection, or hasn't yet
                // picked a face, the canvas is hidden so no stale UV grid
                // is displayed.
                match self.uv_window_face_idx {
                    None => {
                        ui.add_space(40.0);
                        ui.vertical_centered(|ui| {
                            ui.label(egui::RichText::new(
                                "Select a face from the list above to view its UV breakdown."
                            ).size(13.0).color(egui::Color32::from_rgb(180, 180, 200)));
                            ui.add_space(6.0);
                            ui.label(egui::RichText::new(
                                "Tip: clicking a face in the structure panel or 3D viewport (Ctrl+click) also selects it here."
                            ).size(10.0).color(egui::Color32::from_rgb(120, 120, 140)));
                        });
                    }
                    Some(face_idx) => {
                        if let Some(face_uv) = breakdown.faces.get(face_idx) {
                    // Get the surface for this face from the current solid.
                    let surface: Option<Surface> = current_solid_taken.as_ref().and_then(|s| {
                        s.outer_shell.as_ref().and_then(|sh| {
                            sh.faces.get(face_idx)
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
                        // Use click_and_drag so the user can pan by left-dragging
                        // the canvas. Wheel-zoom is handled separately below.
                        let (rect, response) = ui.allocate_exact_size(
                            egui::vec2(size, size),
                            egui::Sense::click_and_drag(),
                        );
                        ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(26, 26, 46));

                        let margin = size * 0.067;
                        let draw_size = size - 2.0 * margin;
                        let size_f64 = size as f64;
                        let draw_size_f64 = draw_size as f64;

                        // ─── Compute base (auto-fit) UV bounds ────────────────
                        // The "base" bounds are what we'd show at zoom = 1.0:
                        // the full UV extent of the face, padded by 5% on each
                        // side. The visible bounds (u_min..u_max / v_min..v_max)
                        // are then derived from these by applying pan + zoom.
                        let mut u_min_base = f64::MAX;
                        let mut u_max_base = f64::MIN;
                        let mut v_min_base = f64::MAX;
                        let mut v_max_base = f64::MIN;
                        for poly in &face_uv.outer_polylines {
                            for &(u, v) in poly {
                                u_min_base = u_min_base.min(u); u_max_base = u_max_base.max(u);
                                v_min_base = v_min_base.min(v); v_max_base = v_max_base.max(v);
                            }
                        }
                        for poly in &face_uv.inner_polylines {
                            for &(u, v) in poly {
                                u_min_base = u_min_base.min(u); u_max_base = u_max_base.max(u);
                                v_min_base = v_min_base.min(v); v_max_base = v_max_base.max(v);
                            }
                        }
                        // Also include UV triangle extents in the bounds,
                        // because some triangles may have been unwrapped
                        // across the periodic seam and now extend outside
                        // the boundary polyline's nominal UV range.
                        for tri in &face_uv.uv_triangles {
                            for &(u, v) in tri {
                                u_min_base = u_min_base.min(u); u_max_base = u_max_base.max(u);
                                v_min_base = v_min_base.min(v); v_max_base = v_max_base.max(v);
                            }
                        }
                        if u_min_base >= u_max_base || v_min_base >= v_max_base {
                            if let Some(s) = surface_ref {
                                let (u0, u1, v0, v1) = s.natural_uv_domain();
                                u_min_base = u0; u_max_base = u1; v_min_base = v0; v_max_base = v1;
                            } else {
                                u_min_base = 0.0; u_max_base = 1.0; v_min_base = 0.0; v_max_base = 1.0;
                            }
                        }
                        let u_range_base = (u_max_base - u_min_base).max(1e-6);
                        let v_range_base = (v_max_base - v_min_base).max(1e-6);
                        u_min_base -= u_range_base * 0.05; u_max_base += u_range_base * 0.05;
                        v_min_base -= v_range_base * 0.05; v_max_base += v_range_base * 0.05;

                        // ─── Apply pan + zoom to derive visible bounds ────────
                        // center = base_center + pan;  half_extent = base_half / zoom
                        // Zoom pivots around the center of the REAL UV box
                        // (base_center_u, base_center_v), not the canvas center.
                        // Because we preserve aspect ratio (see below), the UV
                        // box is displayed with its real shape and the zoom
                        // faithfully enlarges the geometry around its own
                        // center — not around an arbitrary square-canvas point.
                        let base_center_u = (u_min_base + u_max_base) * 0.5;
                        let base_center_v = (v_min_base + v_max_base) * 0.5;
                        let base_half_u = (u_max_base - u_min_base) * 0.5;
                        let base_half_v = (v_max_base - v_min_base) * 0.5;
                        let zoom_f64 = (self.uv_window_zoom as f64).max(0.01);
                        let center_u = base_center_u + self.uv_window_pan[0];
                        let center_v = base_center_v + self.uv_window_pan[1];
                        let half_u = (base_half_u / zoom_f64).max(1e-9);
                        let half_v = (base_half_v / zoom_f64).max(1e-9);
                        let u_min = center_u - half_u;
                        let u_max = center_u + half_u;
                        let v_min = center_v - half_v;
                        let v_max = center_v + half_v;

                        // ─── Aspect-ratio-preserving screen mapping ──────────
                        // The UV box has its own aspect ratio (u_range / v_range).
                        // Previously we stretched it to fill the square canvas,
                        // which distorted the geometry (e.g. a 2π×1 cone UV
                        // became a square). Now we fit the UV box into the
                        // square draw_size area while preserving aspect ratio,
                        // centering it inside the canvas. This means zooming
                        // truly pivots around the center of the real UV box.
                        //
                        // The user can override the aspect ratio via the
                        // `uv_window_aspect_override` slider (1.0 = natural).
                        // This is useful when the U range is much larger than
                        // V (e.g. cylinder: U=2π, V=height) — without an
                        // override the canvas becomes wide and short, making
                        // individual triangles hard to see. Setting the
                        // override to 0.5 effectively "squeezes" U by 2× so
                        // a 2π×π cylinder UV appears as a square.
                        let u_range_vis = (u_max - u_min).max(1e-12);
                        let v_range_vis = (v_max - v_min).max(1e-12);
                        let ar_uv_natural = u_range_vis / v_range_vis;

                        // ─── Metric UV scaling (Bug 8.3.1 fix) ──────────────
                        // When metric UV mode is enabled, the aspect ratio is
                        // computed in metric (arc-length) space rather than raw
                        // parameter space. For a cylinder, this scales U by the
                        // radius so the UV rectangle shows the true surface
                        // proportions (circumference × height instead of
                        // 2π × height). This makes the UV layout look correct
                        // relative to the 3D shape.
                        let metric_ar = if self.uv_window_metric_uv {
                            let (us, vs) = (face_uv.u_metric_scale, face_uv.v_metric_scale);
                            (u_range_vis * us) / (v_range_vis * vs).max(1e-12)
                        } else {
                            ar_uv_natural
                        };

                        // Effective aspect: metric_ar * override.
                        // override=1.0 keeps the natural/metric aspect;
                        // override<1.0 squeezes U (useful when metric UV is
                        // off and U range >> V range).
                        let aspect_override = (self.uv_window_aspect_override as f64).max(0.05);
                        let ar_uv = metric_ar * aspect_override;
                        let (width_f64, height_f64) = if ar_uv >= 1.0 {
                            // Wider than tall — fit width to draw_size.
                            (draw_size_f64, draw_size_f64 / ar_uv)
                        } else {
                            // Taller than wide — fit height to draw_size.
                            (draw_size_f64 * ar_uv, draw_size_f64)
                        };
                        let x_offset_f64 = (size_f64 - width_f64) * 0.5;
                        let y_offset_f64 = (size_f64 - height_f64) * 0.5;

                        let rect_left = rect.left() as f64;
                        let rect_top = rect.top() as f64;
                        // IMPORTANT: map_u/map_v must return SCREEN coords that are
                        // relative to the rect (the allocated area inside the
                        // egui::Window). Without the rect.left()/rect.top() offset
                        // the painted content would be stuck at screen position
                        // (margin, margin) regardless of where the user dragged
                        // the window.
                        //
                        // ─── Bug 8.3.2 fix: V-axis flip for forward=false faces ──
                        // When a face has forward=false, the face normal is
                        // opposite to the surface normal. The user sees the face
                        // from the face-normal direction (i.e., from the "back"
                        // of the surface). In this view, the V axis appears
                        // inverted compared to looking from the surface normal
                        // side. To make the UV polygon match the 3D view, we
                        // flip the V mapping: V grows downward for forward=false
                        // faces (matching the "back-side" view) instead of upward.
                        let v_flip = face_uv.forward; // true = V up (surface normal side), false = V down (back side)
                        let map_u = |u: f64| -> f32 {
                            (rect_left + x_offset_f64 + (u - u_min) / u_range_vis * width_f64) as f32
                        };
                        let map_v = |v: f64| -> f32 {
                            if v_flip {
                                // Standard: V grows upward (v_min at bottom, v_max at top)
                                (rect_top + y_offset_f64 + (1.0 - (v - v_min) / v_range_vis) * height_f64) as f32
                            } else {
                                // Inverted: V grows downward (v_min at top, v_max at bottom)
                                // This matches viewing the face from the back side.
                                (rect_top + y_offset_f64 + ((v - v_min) / v_range_vis) * height_f64) as f32
                            }
                        };
                        // Screen coordinates of the UV box corners (for grid
                        // line endpoints and seam line endpoints).
                        let box_left_x = rect_left + x_offset_f64;
                        let box_right_x = rect_left + x_offset_f64 + width_f64;
                        let box_top_y = rect_top + y_offset_f64;
                        let box_bottom_y = rect_top + y_offset_f64 + height_f64;

                        // ─── Pan via left-drag ────────────────────────────────
                        // drag_delta() is in screen pixels since last frame.
                        // Convert to UV units using the CURRENT visible range
                        // and the aspect-ratio-correct screen dimensions.
                        // Y is flipped because screen Y grows downward while
                        // UV V grows upward.
                        if response.dragged_by(egui::PointerButton::Primary) {
                            let delta = response.drag_delta();
                            if delta.length_sq() > 0.25 {
                                let du = delta.x as f64 / width_f64 * u_range_vis;
                                let dv = delta.y as f64 / height_f64 * v_range_vis;
                                self.uv_window_pan[0] -= du;
                                self.uv_window_pan[1] += dv;
                            }
                        }

                        // ─── Zoom via scroll wheel (zoom-to-cursor) ──────────
                        // When the user scrolls the wheel over the canvas, we
                        // zoom around the cursor's UV position rather than
                        // around the UV box center. This is the standard
                        // "zoom-to-cursor" behavior: the UV point under the
                        // cursor stays at the same screen position before
                        // and after the zoom.
                        //
                        // Math derivation:
                        //   Before: u_mouse = base_center_u + pan_old[0] +
                        //            (2*tx - 1) * base_half_u / zoom_old
                        //   After:  u_mouse = base_center_u + pan_new[0] +
                        //            (2*tx - 1) * base_half_u / zoom_new
                        //   Solving for pan_new:
                        //     pan_new[0] = pan_old[0] +
                        //       (2*tx - 1) * base_half_u * (1/zoom_old - 1/zoom_new)
                        //   where tx ∈ [0,1] is the cursor's normalized X
                        //   position within the UV box (0 = left edge,
                        //   1 = right edge). For the Y axis, ty is flipped
                        //   because screen Y grows downward while UV V grows
                        //   upward.
                        //
                        // Because the aspect-ratio-preserving screen layout
                        // (width_f64, height_f64, x_offset_f64, y_offset_f64)
                        // is INDEPENDENT of zoom (it depends only on the UV
                        // box's intrinsic aspect ratio, which doesn't change
                        // with zoom), tx and ty are the same before and after
                        // the zoom change — so a single computation suffices.
                        if response.hovered() {
                            let scroll_y = ui.input(|i| i.smooth_scroll_delta.y);
                            if scroll_y.abs() > 0.01 {
                                let factor = if scroll_y > 0.0 { 1.12 } else { 1.0 / 1.12 };
                                let old_zoom_f32 = self.uv_window_zoom;
                                let new_zoom_f32 = (old_zoom_f32 * factor).clamp(0.25, 20.0);
                                if (new_zoom_f32 - old_zoom_f32).abs() > 1e-9 {
                                    let old_z = old_zoom_f32 as f64;
                                    let new_z = new_zoom_f32 as f64;
                                    // Cursor's normalized position within the
                                    // UV box. If the cursor is outside the UV
                                    // box (in the gray margin), tx/ty can be
                                    // < 0 or > 1 — the math still works, the
                                    // pivot is just outside the visible area.
                                    if let Some(hp) = response.hover_pos() {
                                        let mx = (hp.x as f64) - rect_left - x_offset_f64;
                                        let my = (hp.y as f64) - rect_top - y_offset_f64;
                                        let tx = mx / width_f64;
                                        let ty = 1.0 - my / height_f64;
                                        let inv_diff = 1.0 / old_z - 1.0 / new_z;
                                        self.uv_window_pan[0] += (2.0 * tx - 1.0) * base_half_u * inv_diff;
                                        self.uv_window_pan[1] += (2.0 * ty - 1.0) * base_half_v * inv_diff;
                                    }
                                    self.uv_window_zoom = new_zoom_f32;
                                }
                                // Consume the scroll so the parent ScrollArea
                                // does not also scroll the window content.
                                ui.input_mut(|i| i.smooth_scroll_delta = egui::Vec2::ZERO);
                            }
                        }

                        // ─── Clipped painter ──────────────────────────────────
                        // All subsequent draws are clipped to the canvas rect so
                        // that zoomed-in content (which extends past the visible
                        // bounds) does not paint over the controls above/below.
                        let painter = ui.painter().with_clip_rect(rect);
                        let painter = &painter;

                        // ─── UV box background ────────────────────────────────
                        // Subtle border around the actual UV box (the area
                        // inside the canvas where UV content is drawn), so the
                        // user can see the real aspect ratio of the UV domain.
                        painter.rect_stroke(
                            egui::Rect::from_min_max(
                                egui::pos2(box_left_x as f32, box_top_y as f32),
                                egui::pos2(box_right_x as f32, box_bottom_y as f32),
                            ),
                            0.0,
                            egui::Stroke::new(0.5, egui::Color32::from_rgb(60, 60, 90)),
                            egui::StrokeKind::Middle,
                        );

                        // Grid lines — span the UV box (not the full canvas),
                        // so they respect the aspect-ratio-preserving layout.
                        let u_divs = self.uv_window_u_divs.min(50);
                        let v_divs = self.uv_window_v_divs.min(50);
                        for i in 0..=u_divs {
                            let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                            let x = map_u(u);
                            painter.line_segment(
                                [egui::pos2(x, box_top_y as f32), egui::pos2(x, box_bottom_y as f32)],
                                egui::Stroke::new(0.5, egui::Color32::from_rgb(51, 51, 68)),
                            );
                        }
                        for j in 0..=v_divs {
                            let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                            let y = map_v(v);
                            painter.line_segment(
                                [egui::pos2(box_left_x as f32, y), egui::pos2(box_right_x as f32, y)],
                                egui::Stroke::new(0.5, egui::Color32::from_rgb(51, 51, 68)),
                            );
                        }

                        // ─── UV triangles (the actual surface triangulation) ───────
                        // Draw filled triangles so the user can see the actual UV
                        // subdivision of the face, not just the boundary.
                        //
                        // Each triangle is rendered as a filled polygon with a thin
                        // edge stroke so the user can see both the triangle shape
                        // AND its edges at the same time.
                        //
                        // The in-hole / outside-boundary check is done using
                        // per-polyline `point_in_polygon` against EACH outer
                        // polyline separately (not a flat_map). This is necessary
                        // because `split_at_seam_jumps` breaks the outer boundary
                        // into multiple disconnected segments, and a flat_map
                        // polygon would have self-intersections. Triangles that
                        // were seam-unwrapped may have centroids outside the
                        // original [0,2π] range, so we also check against a
                        // period-shifted version of each polyline.
                        let hole_polys: Vec<Vec<(f64, f64)>> = face_uv.inner_polylines.iter()
                            .cloned()
                            .collect();
                        // Build per-polyline outer polygons for point-in-polygon
                        // testing. Also build period-shifted copies for seam-
                        // unwrapped triangles whose centroids may lie outside
                        // the original UV range.
                        let u_per = if face_uv.u_periodic { face_uv.u_period } else { 0.0 };
                        let _v_per = if face_uv.v_periodic { face_uv.v_period } else { 0.0 };
                        let outer_polys: Vec<Vec<(f64, f64)>> = face_uv.outer_polylines.clone();
                        // Period-shifted copies: shift by -u_period and +u_period
                        let outer_polys_shifted: Vec<Vec<(f64, f64)>> = if u_per > 0.0 {
                            let mut shifted = Vec::new();
                            for poly in &outer_polys {
                                let p_neg: Vec<(f64, f64)> = poly.iter().map(|&(u, v)| (u - u_per, v)).collect();
                                let p_pos: Vec<(f64, f64)> = poly.iter().map(|&(u, v)| (u + u_per, v)).collect();
                                shifted.push(p_neg);
                                shifted.push(p_pos);
                            }
                            shifted
                        } else {
                            Vec::new()
                        };
                        let hole_polys_shifted: Vec<Vec<(f64, f64)>> = if u_per > 0.0 {
                            let mut shifted = Vec::new();
                            for poly in &hole_polys {
                                let p_neg: Vec<(f64, f64)> = poly.iter().map(|&(u, v)| (u - u_per, v)).collect();
                                let p_pos: Vec<(f64, f64)> = poly.iter().map(|&(u, v)| (u + u_per, v)).collect();
                                shifted.push(p_neg);
                                shifted.push(p_pos);
                            }
                            shifted
                        } else {
                            Vec::new()
                        };

                        if !face_uv.uv_triangles.is_empty() {
                            let tri_limit = 3000.min(face_uv.uv_triangles.len());
                            for (ti, tri) in face_uv.uv_triangles.iter().enumerate() {
                                let cu = (tri[0].0 + tri[1].0 + tri[2].0) / 3.0;
                                let cv = (tri[0].1 + tri[1].1 + tri[2].1) / 3.0;
                                // Check centroid against each outer polyline separately
                                // (including period-shifted copies) to handle seam-unwrapped
                                // triangles correctly.
                                let in_outer = outer_polys.iter().any(|p| point_in_polygon(cu, cv, p))
                                    || outer_polys_shifted.iter().any(|p| point_in_polygon(cu, cv, p));
                                let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h))
                                    || hole_polys_shifted.iter().any(|h| point_in_polygon(cu, cv, h));

                                let p0 = egui::pos2(map_u(tri[0].0), map_v(tri[0].1));
                                let p1 = egui::pos2(map_u(tri[1].0), map_v(tri[1].1));
                                let p2 = egui::pos2(map_u(tri[2].0), map_v(tri[2].1));

                                if in_hole || !in_outer {
                                    // Triangle inside a hole or outside the outer boundary —
                                    // skip drawing entirely to avoid the "aura" effect.
                                } else {
                                    // Valid triangle — alternating blue tints with stronger contrast.
                                    let fill = if ti % 2 == 0 {
                                        egui::Color32::from_rgba_premultiplied(68, 136, 255, 50)
                                    } else {
                                        egui::Color32::from_rgba_premultiplied(100, 180, 255, 50)
                                    };
                                    // Draw filled triangle (no stroke on convex_polygon — we draw
                                    // edges separately below for guaranteed visibility).
                                    painter.add(egui::Shape::convex_polygon(vec![p0, p1, p2], fill, egui::Stroke::NONE));
                                    // Draw each edge of the triangle as an explicit line segment
                                    // so the diagonal of each quad-pair is always visible.
                                    let edge_color = egui::Color32::from_rgba_premultiplied(100, 170, 255, 140);
                                    let edge_stroke = egui::Stroke::new(1.0, edge_color);
                                    painter.line_segment([p0, p1], edge_stroke);
                                    painter.line_segment([p1, p2], edge_stroke);
                                    painter.line_segment([p2, p0], edge_stroke);
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
                            painter.line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(0, 255, 136)));
                        }
                        // Inner boundaries
                        for poly in &face_uv.inner_polylines {
                            if poly.len() < 2 { continue; }
                            let points: Vec<egui::Pos2> = poly.iter()
                                .map(|&(u, v)| egui::pos2(map_u(u), map_v(v)))
                                .collect();
                            painter.line(points, egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 68, 68)));
                        }

                        // ─── Seam lines (for periodic surfaces) ───────────────
                        // For U-periodic surfaces (cone, cylinder, sphere,
                        // torus, revolution), the U=u_min edge of the natural
                        // UV domain is the same physical line as U=u_max — the
                        // "seam" where the surface wraps around. We draw it as
                        // a bright yellow line so the user can see where the
                        // surface closes on itself.
                        //
                        // For V-periodic surfaces (sphere, torus), the same
                        // applies in the V direction.
                        //
                        // We use the surface's natural_uv_domain() to find the
                        // seam location (typically u=0 and u=2π). The seam is
                        // only drawn if it falls within the visible UV range.
                        let seam_stroke = egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 200, 0));
                        if face_uv.u_periodic && face_uv.u_period > 0.0 {
                            if let Some(s) = surface_ref {
                                let (nat_u0, nat_u1, _nat_v0, _nat_v1) = s.natural_uv_domain();
                                // The seam is at u = nat_u0 (equivalently nat_u1).
                                // Draw a vertical line at this U value, spanning
                                // the full visible V range of the UV box.
                                for &seam_u in &[nat_u0, nat_u1] {
                                    if seam_u.is_finite() && seam_u >= u_min && seam_u <= u_max {
                                        let x = map_u(seam_u);
                                        painter.line_segment(
                                            [egui::pos2(x, box_top_y as f32),
                                             egui::pos2(x, box_bottom_y as f32)],
                                            seam_stroke,
                                        );
                                    }
                                }
                            }
                        }
                        if face_uv.v_periodic && face_uv.v_period > 0.0 {
                            if let Some(s) = surface_ref {
                                let (_nat_u0, _nat_u1, nat_v0, nat_v1) = s.natural_uv_domain();
                                for &seam_v in &[nat_v0, nat_v1] {
                                    if seam_v.is_finite() && seam_v >= v_min && seam_v <= v_max {
                                        let y = map_v(seam_v);
                                        painter.line_segment(
                                            [egui::pos2(box_left_x as f32, y),
                                             egui::pos2(box_right_x as f32, y)],
                                            seam_stroke,
                                        );
                                    }
                                }
                            }
                        }

                        // Surface evaluation points (inside outer boundary)
                        // Use per-polyline point_in_polygon (not flat_map) for
                        // consistency with the triangle rendering above.
                        if let Some(s) = surface_ref {
                            for i in 0..=u_divs {
                                for j in 0..=v_divs {
                                    let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
                                    let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
                                    let p = s.point_at(u, v);
                                    if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
                                        let inside = outer_polys.iter().any(|poly| point_in_polygon(u, v, poly))
                                            || outer_polys_shifted.iter().any(|poly| point_in_polygon(u, v, poly));
                                        if inside {
                                            painter.circle_filled(
                                                egui::pos2(map_u(u), map_v(v)), 2.0,
                                                egui::Color32::from_rgba_premultiplied(102, 136, 255, 120),
                                            );
                                        }
                                    }
                                }
                            }
                        }

                        // Axis labels — drawn on the unclipped parent painter so
                        // they are always visible at the canvas edges even when
                        // the user has zoomed in past the boundary.
                        //
                        // When metric UV is enabled, show metric values
                        // (e.g., arc length for U on cylinders) alongside
                        // the raw parameter values.
                        let u_label = if self.uv_window_metric_uv && face_uv.u_metric_scale != 1.0 {
                            format!("U ({:.2}..{:.2}) [{:.1}..{:.1}mm]",
                                u_min, u_max,
                                u_min * face_uv.u_metric_scale,
                                u_max * face_uv.u_metric_scale)
                        } else {
                            format!("U ({:.2}..{:.2})", u_min, u_max)
                        };
                        let v_dir_label = if !face_uv.forward { "V↓" } else { "V" };
                        let v_label = if self.uv_window_metric_uv && face_uv.v_metric_scale != 1.0 {
                            format!("{} ({:.2}..{:.2}) [{:.1}..{:.1}mm]",
                                v_dir_label, v_min, v_max,
                                v_min * face_uv.v_metric_scale,
                                v_max * face_uv.v_metric_scale)
                        } else {
                            format!("{} ({:.2}..{:.2})", v_dir_label, v_min, v_max)
                        };
                        ui.painter().text(
                            egui::pos2(rect.center().x, rect.bottom() - 5.0),
                            egui::Align2::CENTER_BOTTOM,
                            u_label,
                            egui::FontId::proportional(10.0),
                            egui::Color32::from_rgb(170, 170, 170),
                        );
                        ui.painter().text(
                            egui::pos2(rect.left() + 8.0, rect.center().y),
                            egui::Align2::CENTER_CENTER,
                            v_label,
                            egui::FontId::proportional(10.0),
                            egui::Color32::from_rgb(170, 170, 170),
                        );
                    }
                } // close `if let Some(face_uv) = breakdown.faces.get(face_idx)`
                } // close `Some(face_idx) => {`
                } // close `match self.uv_window_face_idx`
            });

        // ─── Reverse sync: UV window face list click → selected_face ───────
        // If the user just clicked a face in the UV window's own face list,
        // propagate the selection back to `selected_face` / `highlighted_face`
        // so the 3D viewport highlights the same face. This is the reverse
        // direction of the sync that happens in `pending_face_select`.
        //
        // Only applies to STEP files (where `detailed_instances` is
        // populated and `current_solid` is None) — for primitives/NURBS
        // the 3D viewport doesn't support per-face highlighting anyway
        // (the primitive's mesh is merged into one buffer without
        // per-face IDs), so there's nothing to sync.
        if let Some(face_pos_idx) = uv_window_face_clicked {
            if self.current_solid.is_none() {
                // Use the same instance the UV window derived its solid from.
                let chosen_idx = self
                    .selected_instance
                    .or_else(|| if self.detailed_instances.is_empty() { None } else { Some(0) });
                if let Some(inst_idx) = chosen_idx {
                    if let Some(inst) = self.detailed_instances.get(inst_idx) {
                        if let Some(face_info) = inst.faces.get(face_pos_idx) {
                            let fid = face_info.face_id;
                            self.selected_instance = Some(inst_idx);
                            self.selected_face = Some((inst_idx, fid));
                            self.highlighted_face = Some((inst_idx, fid));
                            self.highlight_dirty = true;
                            self.scroll_to_face_id = Some(fid);
                        }
                    }
                }
            }
        }

        // Restore the breakdown (only if still valid — it was before).
        if self.solid_uv_breakdown.is_none() {
            self.solid_uv_breakdown = breakdown_taken;
        }
        self.show_uv_window = window_open;
    }

    /// Draw the GD&T (Geometric Dimensioning and Tolerancing) panel.
    ///
    /// Shows all geometric tolerances, datum features, and datum references
    /// extracted from the STEP file. The panel lists tolerances with their
    /// type, value, datum references, and applied-to faces. Clicking a
    /// tolerance highlights the corresponding face in the 3D viewport.
    fn draw_gdt_window(&mut self, ctx: &egui::Context) {
        if !self.show_gdt_window {
            return;
        }

        // Lazy-extract GD&T data if not already cached
        if self.gdt_data.is_none() {
            self.gdt_data = self.last_step_file.as_ref()
                .map(|sf| draper_step::pmi::extract_gdt(sf));
        }

        let mut window_open = self.show_gdt_window;
        let gdt_data_taken = self.gdt_data.take();

        egui::Window::new("GD&T — Geometric Tolerances")
            .id(egui::Id::new("gdt_window"))
            .open(&mut window_open)
            .resizable(true)
            .default_size([420.0, 500.0])
            .show(ctx, |ui| {
                let gdt = match &gdt_data_taken {
                    Some(d) => d,
                    None => {
                        ui.label(egui::RichText::new("No GD&T data available. Load a STEP file with GD&T annotations.")
                            .color(egui::Color32::from_rgb(180, 120, 120)));
                        return;
                    }
                };

                if gdt.tolerances.is_empty() && gdt.datum_features.is_empty() {
                    ui.label(egui::RichText::new("No GD&T entities found in this STEP file.")
                        .color(egui::Color32::from_rgb(160, 160, 180)));
                    ui.add_space(8.0);
                    ui.label(egui::RichText::new(
                        "GD&T data appears when the STEP file contains GEOMETRIC_TOLERANCE, DATUM_FEATURE, or SHAPE_ASPECT entities (AP242)."
                    ).size(10.0).color(egui::Color32::from_rgb(120, 120, 140)));
                    return;
                }

                // Summary
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(format!("Tolerances: {}", gdt.tolerances.len()))
                        .size(11.0).color(egui::Color32::from_rgb(100, 180, 255)));
                    ui.separator();
                    ui.label(egui::RichText::new(format!("Datums: {}", gdt.datum_features.len()))
                        .size(11.0).color(egui::Color32::from_rgb(255, 180, 100)));
                    ui.separator();
                    ui.label(egui::RichText::new(format!("Shape Aspects: {}", gdt.shape_aspects.len()))
                        .size(11.0).color(egui::Color32::from_rgb(100, 255, 180)));
                });
                ui.separator();

                // ─── Tolerances table ────────────────────────────────────
                if !gdt.tolerances.is_empty() {
                    ui.collapsing(egui::RichText::new("Tolerances").size(12.0), |ui| {
                        egui::ScrollArea::vertical().max_height(250.0).show(ui, |ui| {
                            for (i, tol) in gdt.tolerances.iter().enumerate() {
                                let type_label = match &tol.tolerance_type {
                                    draper_step::pmi::GdtToleranceType::Position => "⊕ Position",
                                    draper_step::pmi::GdtToleranceType::Flatness => "⏥ Flatness",
                                    draper_step::pmi::GdtToleranceType::Straightness => "━ Straightness",
                                    draper_step::pmi::GdtToleranceType::Circularity => "○ Circularity",
                                    draper_step::pmi::GdtToleranceType::Cylindricity => "⌭ Cylindricity",
                                    draper_step::pmi::GdtToleranceType::Perpendicularity => "⊥ Perpendicularity",
                                    draper_step::pmi::GdtToleranceType::Parallelism => "∥ Parallelism",
                                    draper_step::pmi::GdtToleranceType::Angularity => "∠ Angularity",
                                    draper_step::pmi::GdtToleranceType::Concentricity => "◎ Concentricity",
                                    draper_step::pmi::GdtToleranceType::Symmetry => "⌯ Symmetry",
                                    draper_step::pmi::GdtToleranceType::Runout => "↗ Runout",
                                    draper_step::pmi::GdtToleranceType::ProfileOfLine => "⌒ Profile of Line",
                                    draper_step::pmi::GdtToleranceType::ProfileOfSurface => "⌓ Profile of Surface",
                                    draper_step::pmi::GdtToleranceType::Other(s) => {
                                        // Use a temporary to avoid lifetime issues
                                        ui.label(egui::RichText::new(format!("? {}", s)).size(10.0));
                                        continue;
                                    }
                                };

                                ui.group(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new(format!("#{}", tol.step_id))
                                            .size(10.0).color(egui::Color32::from_rgb(100, 100, 120)));
                                        ui.label(egui::RichText::new(type_label).size(11.0));
                                    });
                                    ui.horizontal(|ui| {
                                        if let Some(val) = tol.tolerance_value {
                                            ui.label(egui::RichText::new(format!("Tol: {:.4}", val))
                                                .size(10.0).color(egui::Color32::from_rgb(100, 200, 100)));
                                        }
                                        if !tol.name.is_empty() {
                                            ui.label(egui::RichText::new(&tol.name)
                                                .size(10.0).color(egui::Color32::from_rgb(180, 180, 200)));
                                        }
                                    });
                                    if !tol.datum_references.is_empty() {
                                        let datums: Vec<String> = tol.datum_references.iter()
                                            .map(|r| format!("#{}", r))
                                            .collect();
                                        ui.label(egui::RichText::new(format!("Datums: {}", datums.join(", ")))
                                            .size(9.0).color(egui::Color32::from_rgb(200, 160, 100)));
                                    }
                                    if let Some(applied) = tol.applied_to {
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new(format!("Applied to: #{}", applied))
                                                .size(9.0).color(egui::Color32::from_rgb(120, 160, 200)));
                                            // Click to highlight the face in 3D
                                            if ui.small_button("Show").clicked() {
                                                self.log(&format!("GD&T: tolerance #{} applies to entity #{}", tol.step_id, applied));
                                            }
                                        });
                                    }
                                });
                                if i < gdt.tolerances.len() - 1 {
                                    ui.add_space(2.0);
                                }
                            }
                        });
                    });
                }

                // ─── Datum Features table ────────────────────────────────
                if !gdt.datum_features.is_empty() {
                    ui.collapsing(egui::RichText::new("Datum Features").size(12.0), |ui| {
                        for df in &gdt.datum_features {
                            ui.group(|ui| {
                                ui.horizontal(|ui| {
                                    ui.label(egui::RichText::new(format!("#{}", df.step_id))
                                        .size(10.0).color(egui::Color32::from_rgb(100, 100, 120)));
                                    ui.label(egui::RichText::new(if df.name.is_empty() { "Datum" } else { &df.name })
                                        .size(11.0));
                                });
                                if let Some(applied) = df.applied_to {
                                    ui.label(egui::RichText::new(format!("Applied to: #{}", applied))
                                        .size(9.0).color(egui::Color32::from_rgb(120, 160, 200)));
                                }
                            });
                        }
                    });
                }

                // ─── Shape Aspects table ─────────────────────────────────
                if !gdt.shape_aspects.is_empty() {
                    ui.collapsing(egui::RichText::new("Shape Aspects").size(12.0), |ui| {
                        egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                            for sa in &gdt.shape_aspects {
                                ui.group(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.label(egui::RichText::new(format!("#{}", sa.step_id))
                                            .size(10.0).color(egui::Color32::from_rgb(100, 100, 120)));
                                        ui.label(egui::RichText::new(if sa.name.is_empty() { "Shape Aspect" } else { &sa.name })
                                            .size(11.0));
                                    });
                                    if let Some(rs) = sa.relating_shape {
                                        ui.label(egui::RichText::new(format!("→ face/surface #{}", rs))
                                            .size(9.0).color(egui::Color32::from_rgb(120, 160, 200)));
                                    }
                                });
                            }
                        });
                    });
                }
            });

        // Restore GD&T data
        if self.gdt_data.is_none() {
            self.gdt_data = gdt_data_taken;
        }
        self.show_gdt_window = window_open;
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
                    ui.label(egui::RichText::new(env!("DRAPER_GIT_HASH")).size(9.0).color(egui::Color32::from_rgb(140, 160, 180)));
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
                        let info_text = format!("V:{} T:{} [{}]", self.current_model.vertex_count, self.current_model.triangle_count, env!("DRAPER_GIT_HASH"));
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

        // ─── Mobile loading overlay: progress bar + Cancel button ────────
        // On mobile, the user can't see the desktop-style progress overlay
        // drawn in the viewport (it's hidden by the touch-gesture help at the
        // bottom). We draw a dedicated mobile loading overlay on top of
        // everything, with a large Cancel button that's easy to tap.
        if self.is_loading && self.total_instance_count > 0 {
            let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
            let elapsed_secs = self.loading_start.map(|t| t.elapsed().as_secs_f64()).unwrap_or(0.0);
            let eta_text = if self.triangulated_count > 0 && elapsed_secs > 0.5 {
                let per_inst = elapsed_secs / self.triangulated_count as f64;
                let remaining = (self.total_instance_count - self.triangulated_count) as f64 * per_inst;
                format!(" · ETA: {:.0}s", remaining.max(0.0))
            } else {
                String::new()
            };

            let overlay_w = (screen.width() - 2.0 * margin).min(380.0);
            let overlay_h = 84.0;
            let overlay_x = screen.center().x - overlay_w / 2.0;
            let overlay_y = screen.min.y + 50.0;
            let overlay_pos = egui::Pos2::new(overlay_x, overlay_y);

            egui::Area::new(egui::Id::new("mobile_loading_overlay"))
                .fixed_pos(overlay_pos)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    let (rect, _) = ui.allocate_exact_size(
                        egui::vec2(overlay_w, overlay_h),
                        egui::Sense::hover(),
                    );
                    // Background panel
                    ui.painter().rect_filled(
                        rect,
                        10.0,
                        egui::Color32::from_rgba_premultiplied(0, 0, 0, 200),
                    );

                    // Progress text (top line)
                    ui.painter().text(
                        egui::pos2(rect.center().x, rect.min.y + 14.0),
                        egui::Align2::CENTER_CENTER,
                        format!("Loading: {}/{} ({:.0}%) · {:.0}s{}",
                            self.triangulated_count, self.total_instance_count,
                            progress * 100.0, elapsed_secs, eta_text),
                        egui::FontId::proportional(12.0),
                        egui::Color32::WHITE,
                    );

                    // Progress bar
                    let bar_x = rect.min.x + 14.0;
                    let bar_y = rect.min.y + 30.0;
                    let bar_w = rect.width() - 28.0;
                    let bar_h = 12.0;
                    ui.painter().rect_filled(
                        egui::Rect::from_min_size(egui::pos2(bar_x, bar_y), egui::vec2(bar_w, bar_h)),
                        4.0,
                        egui::Color32::from_rgb(60, 60, 60),
                    );
                    let fill_w = bar_w * progress;
                    if fill_w > 0.0 {
                        ui.painter().rect_filled(
                            egui::Rect::from_min_size(egui::pos2(bar_x, bar_y), egui::vec2(fill_w, bar_h)),
                            4.0,
                            egui::Color32::from_rgb(80, 180, 80),
                        );
                    }

                    // Cancel button (large, easy to tap)
                    let cancel_w = 120.0;
                    let cancel_h = 32.0;
                    let cancel_x = rect.center().x - cancel_w / 2.0;
                    let cancel_y = rect.min.y + 48.0;
                    let cancel_rect = egui::Rect::from_min_size(
                        egui::pos2(cancel_x, cancel_y),
                        egui::vec2(cancel_w, cancel_h),
                    );
                    let cancel_btn = egui::Button::new(
                        egui::RichText::new("✕ Cancel").size(13.0).color(egui::Color32::WHITE)
                    ).fill(egui::Color32::from_rgb(180, 60, 60));
                    let cancel_resp = ui.put(cancel_rect, cancel_btn);
                    if cancel_resp.clicked() {
                        self.cancel_loading(true);
                    }
                });
        }

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
                                self.open_tree_nodes.clear();
                                self.scroll_to_tree_node = None;
                                self.scroll_to_face_id = None;
                                // Also clear the UV window's active face
                                // — see desktop "Clear Selection" handler
                                // for rationale.
                                self.uv_window_face_idx = None;
                                self.uv_window_prev_face_idx = None;
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
                            let prev_lod_panel = self.lod_level;
                            let quality_label_panel = if let Some(from) = self.lod_downgraded_from {
                                format!("Quality: {} (auto from {})", self.lod_level.label(), from.label())
                            } else {
                                format!("Quality: {}", self.lod_level.label())
                            };
                            egui::ComboBox::from_id_salt("lod_mobile_panel")
                                .selected_text(&quality_label_panel)
                                .show_ui(ui, |ui| {
                                    for &lod in LodLevel::all() {
                                        ui.selectable_value(&mut self.lod_level, lod, lod.label());
                                    }
                                });
                            if self.lod_level != prev_lod_panel {
                                self.lod_downgraded_from = None; // user manually changed
                                self.log(&format!(
                                    "LOD changed {} → {} — re-triangulating current model...",
                                    prev_lod_panel.label(),
                                    self.lod_level.label()
                                ));
                                self.retriangulate_for_lod();
                                #[cfg(target_arch = "wasm32")]
                                self.save_lod_to_local_storage();
                            }

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
                            // Partial result info (after timeout or cancel)
                            if let Some(ref info) = self.partial_result_info {
                                ui.label(egui::RichText::new(info.clone())
                                    .size(11.0)
                                    .color(egui::Color32::from_rgb(255, 180, 50)));
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
            let mut pending_copy_face_id: Option<u64> = None;
            let mut pending_visibility_toggle: Option<usize> = None;
            let mut pending_instance_isolate: Option<usize> = None;
            let mut pending_face_visibility_toggle: Option<(usize, u64)> = None;
            let mut pending_face_isolate: Option<(usize, u64)> = None;
            // Subtree-level actions for mobile
            let mut pending_subtree_hide: Option<Vec<usize>> = None;
            let mut pending_subtree_show: Option<Vec<usize>> = None;
            let mut pending_subtree_isolate: Option<Vec<usize>> = None;

            let assembly_tree_clone = self.assembly_tree.clone();
            let detailed_instances_clone = self.detailed_instances.clone();
            let selected_instance = self.selected_instance;
            let selected_face = self.selected_face;
            let open_tree_nodes = self.open_tree_nodes.clone();
            let scroll_to_tree_node = self.scroll_to_tree_node.clone();
            let scroll_to_face_id = self.scroll_to_face_id;
            let hidden_instances = self.hidden_instances.clone();
            let hidden_faces_clone = self.hidden_faces.clone();

            egui::Window::new("Structure")
                .id(egui::Id::new("mobile_structure_window"))
                .fixed_pos(egui::Pos2::new(panel_x, screen.min.y + 44.0))
                .fixed_size(egui::vec2(panel_width, screen.height() - 50.0))
                .resizable(false)
                .collapsible(false)
                .default_open(true)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical().id_salt("mobile_structure_outer").auto_shrink([false, false]).show(ui, |ui| {
                        // Loading progress
                        if self.is_loading && self.total_instance_count > 0 {
                            let progress = self.triangulated_count as f32 / self.total_instance_count as f32;
                            ui.add(egui::ProgressBar::new(progress).show_percentage());
                        }

                        // ─── "Show All" button (mobile) ─────────────────────────────
                        // Restores visibility of all hidden instances and hidden faces.
                        // Big enough to tap on mobile.
                        let has_hidden = !self.hidden_instances.is_empty() || !self.hidden_faces.is_empty();
                        let show_all_btn = egui::Button::new(
                            egui::RichText::new("Show All").size(13.0)
                        ).min_size(egui::vec2(ui.available_width(), 28.0));
                        let show_all_resp = ui.add_enabled(has_hidden, show_all_btn);
                        if show_all_resp.clicked() {
                            self.hidden_instances.clear();
                            self.hidden_faces.clear();
                            self.highlight_dirty = true;
                            self.edge_dirty = true;
                            self.wireframe_overlay_dirty = true;
                            self.log("Show All: restored visibility of all instances and faces");
                        }

                        // Tree
                        ui.heading(egui::RichText::new("Tree").size(13.0));
                        egui::ScrollArea::vertical().id_salt("mobile_tree_scroll").max_height(200.0).show(ui, |ui| {
                            if let Some(ref tree) = assembly_tree_clone {
                                draw_assembly_node_static(ui, tree, selected_instance, &hidden_instances, &mut pending_instance_select, &mut pending_visibility_toggle, &mut pending_instance_isolate, &mut pending_subtree_hide, &mut pending_subtree_show, &mut pending_subtree_isolate, &open_tree_nodes, &scroll_to_tree_node);
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
                                        // Isolate button (mobile fallback list)
                                        let isolate_btn = egui::Button::new(
                                            egui::RichText::new("◎").size(12.0)
                                        ).frame(false);
                                        if ui.add(isolate_btn).on_hover_text(
                                            "Isolate: hide all other instances (click again to restore)"
                                        ).clicked() {
                                            pending_instance_isolate = Some(i);
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
                                egui::ScrollArea::vertical().id_salt("mobile_face_list_scroll").max_height(200.0).show(ui, |ui| {
                                    for face in &inst.faces {
                                        let is_selected = selected_face == Some((inst_idx, face.face_id));
                                        let is_face_visible = !hidden_faces_clone.contains(&(inst_idx, face.face_id));
                                        ui.horizontal(|ui| {
                                            // Per-face visibility eye icon
                                            let eye_color = if is_face_visible {
                                                egui::Color32::from_rgb(80, 180, 80)
                                            } else {
                                                egui::Color32::from_rgb(180, 80, 80)
                                            };
                                            let eye_text = if is_face_visible { "👁" } else { "  " };
                                            if ui.add(egui::Label::new(egui::RichText::new(eye_text).size(12.0).color(eye_color)).sense(egui::Sense::click())).clicked() {
                                                pending_face_visibility_toggle = Some((inst_idx, face.face_id));
                                            }
                                            // Per-face isolate button
                                            let isolate_btn = egui::Button::new(
                                                egui::RichText::new("◎").size(12.0)
                                            ).frame(false);
                                            if ui.add(isolate_btn).on_hover_text(
                                                "Isolate: hide all other faces of this instance"
                                            ).clicked() {
                                                pending_face_isolate = Some((inst_idx, face.face_id));
                                            }
                                            let label = format!("F#{} STEP#{} {}", face.face_id, face.step_face_id, face.surface_type);
                                            let response = ui.selectable_label(is_selected, &label);
                                            if scroll_to_face_id == Some(face.face_id) {
                                                response.scroll_to_me(Some(egui::Align::Center));
                                            }
                                            if response.clicked() {
                                                pending_face_select = Some((inst_idx, face.face_id));
                                            }
                                        });
                                    }
                                });
                            }
                        } else {
                            ui.label(egui::RichText::new("Select an instance").size(11.0).color(egui::Color32::GRAY));
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
                        // Same UV window reset as desktop visibility toggle.
                        self.uv_window_face_idx = None;
                        self.uv_window_prev_face_idx = None;
                        self.solid_uv_breakdown = None;
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
                // ─── Instance switch → UV window must reset ──
                // Same logic as desktop: clear the UV window's face
                // index and invalidate the cached breakdown so the next
                // frame recomputes from the new instance.
                if self.uv_window_face_idx.is_some() {
                    self.uv_window_face_idx = None;
                    self.uv_window_prev_face_idx = None;
                    self.solid_uv_breakdown = None;
                }
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
                self.scroll_to_face_id = Some(fid);
                // ─── Sync structure-panel face pick → UV Breakdown window ──
                // Same logic as desktop: resolve the face_id to a positional
                // index in the instance's faces array, then set the UV
                // window's face index. See the desktop handler above for
                // the full rationale.
                if let Some(inst) = self.detailed_instances.get(inst_idx) {
                    if let Some(pos) = inst.faces.iter().position(|f| f.face_id == fid) {
                        if self.uv_window_face_idx != Some(pos) {
                            self.uv_window_face_idx = Some(pos);
                            self.uv_window_zoom = 1.0;
                            self.uv_window_pan = [0.0, 0.0];
                            self.uv_window_prev_face_idx = Some(pos);
                        }
                    }
                }
                if let Some(ref tree) = self.assembly_tree {
                    let (path, target) = find_instance_path(tree, inst_idx);
                    self.open_tree_nodes = path.into_iter().collect();
                    self.scroll_to_tree_node = target;
                }
            }
            if let Some(fid) = pending_copy_face_id {
                ctx.copy_text(format!("{}", fid));
            }
            // ─── Mobile: Isolate instance ───────────────────────────────
            if let Some(idx) = pending_instance_isolate {
                let total_instances = self.instance_triangle_ranges.len();
                let visible_count = total_instances.saturating_sub(self.hidden_instances.len());
                let is_only_visible = visible_count == 1 && !self.hidden_instances.contains(&idx);
                if is_only_visible {
                    self.hidden_instances.clear();
                } else {
                    self.hidden_instances.clear();
                    for i in 0..total_instances {
                        if i != idx {
                            self.hidden_instances.insert(i);
                        }
                    }
                    if self.selected_instance != Some(idx) {
                        self.selected_instance = Some(idx);
                        self.selected_face = None;
                        self.highlighted_face = None;
                        if self.uv_window_face_idx.is_some() {
                            self.uv_window_face_idx = None;
                            self.uv_window_prev_face_idx = None;
                            self.solid_uv_breakdown = None;
                        }
                    }
                }
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
            // ─── Mobile: Toggle individual face visibility ──────────────
            if let Some((inst_idx, fid)) = pending_face_visibility_toggle {
                if self.hidden_faces.contains(&(inst_idx, fid)) {
                    self.hidden_faces.remove(&(inst_idx, fid));
                } else {
                    self.hidden_faces.insert((inst_idx, fid));
                    if self.selected_face == Some((inst_idx, fid)) {
                        self.selected_face = None;
                        self.highlighted_face = None;
                        if let Some(inst) = self.detailed_instances.get(inst_idx) {
                            if let Some(pos) = inst.faces.iter().position(|f| f.face_id == fid) {
                                if self.uv_window_face_idx == Some(pos) {
                                    self.uv_window_face_idx = None;
                                    self.uv_window_prev_face_idx = None;
                                    self.solid_uv_breakdown = None;
                                }
                            }
                        }
                    }
                }
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
            // ─── Mobile: Isolate face ───────────────────────────────────
            if let Some((inst_idx, fid)) = pending_face_isolate {
                if let Some(inst) = self.detailed_instances.get(inst_idx) {
                    let total_faces = inst.faces.len();
                    let hidden_in_inst = inst.faces.iter()
                        .filter(|f| self.hidden_faces.contains(&(inst_idx, f.face_id)))
                        .count();
                    let visible_in_inst = total_faces.saturating_sub(hidden_in_inst);
                    let is_only_visible_face = visible_in_inst == 1
                        && !self.hidden_faces.contains(&(inst_idx, fid));
                    if is_only_visible_face {
                        let face_keys: Vec<_> = inst.faces.iter()
                            .map(|f| (inst_idx, f.face_id))
                            .collect();
                        for k in face_keys {
                            self.hidden_faces.remove(&k);
                        }
                    } else {
                        for f in &inst.faces {
                            if f.face_id != fid {
                                self.hidden_faces.insert((inst_idx, f.face_id));
                            } else {
                                self.hidden_faces.remove(&(inst_idx, f.face_id));
                            }
                        }
                    }
                }
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
            // ─── Mobile: Subtree-level hide/show/isolate ──────────────
            if let Some(indices) = pending_subtree_hide {
                for idx in &indices {
                    self.hidden_instances.insert(*idx);
                }
                self.log(&format!("Subtree hidden: {} instance(s)", indices.len()));
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
            if let Some(indices) = pending_subtree_show {
                for idx in &indices {
                    self.hidden_instances.remove(idx);
                }
                self.log(&format!("Subtree shown: {} instance(s)", indices.len()));
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
            }
            if let Some(indices) = pending_subtree_isolate {
                let total_instances = self.instance_triangle_ranges.len();
                let visible_count = total_instances.saturating_sub(self.hidden_instances.len());
                let all_others_hidden = indices.iter().all(|idx| !self.hidden_instances.contains(idx))
                    && self.hidden_instances.len() == total_instances - indices.len();
                if all_others_hidden && visible_count == indices.len() {
                    self.hidden_instances.clear();
                    self.log(&format!("Restored all instances (was isolated to subtree of {})", indices.len()));
                } else {
                    self.hidden_instances.clear();
                    for i in 0..total_instances {
                        if !indices.contains(&i) {
                            self.hidden_instances.insert(i);
                        }
                    }
                    self.log(&format!("Isolated subtree: {} instance(s) visible", indices.len()));
                    if let Some(&first_idx) = indices.first() {
                        if self.selected_instance != Some(first_idx) {
                            self.selected_instance = Some(first_idx);
                            self.selected_face = None;
                            self.highlighted_face = None;
                            if self.uv_window_face_idx.is_some() {
                                self.uv_window_face_idx = None;
                                self.uv_window_prev_face_idx = None;
                                self.solid_uv_breakdown = None;
                            }
                        }
                    }
                }
                self.highlight_dirty = true;
                self.edge_dirty = true;
                self.wireframe_overlay_dirty = true;
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
                        // Copy All — same as desktop. On mobile this is the
                        // ONLY way to get the log text out (no right-click →
                        // Copy works reliably on touch devices).
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

/// Split a UV polyline at large parameter jumps caused by periodic seams.
///
/// For a periodic surface (e.g., cylinder where u ∈ [0, 2π]), the boundary
/// polyline will have a sudden jump from u ≈ 2π back to u ≈ 0 when the
/// boundary wraps around the seam. If we draw a single polyline through
/// these points, the renderer connects the two sides with a horizontal
/// line across the entire UV plane, producing a wrong "X" shape.
///
/// This function detects such jumps (delta > π for U-periodic, delta > π/2
/// for V-periodic — sphere's v range is only π so we use a smaller threshold)
/// and splits the polyline into multiple sub-polylines, each contiguous in
/// UV space. The caller can then render each sub-polyline independently.
///
/// Points whose U coordinate is near the seam (within a small epsilon of
/// either 0 or 2π) are kept on the side they were sampled from, so the
/// split correctly produces two halves: one with u ∈ [0, ~π] and another
/// with u ∈ [~π, 2π].
fn split_at_seam_jumps(
    uv: Vec<(f64, f64)>,
    u_periodic: bool,
    v_periodic: bool,
) -> Vec<Vec<(f64, f64)>> {
    if !u_periodic && !v_periodic {
        return vec![uv];
    }
    if uv.len() < 2 {
        return vec![uv];
    }
    // Threshold for detecting a seam jump. For a periodic parameter with
    // period 2π, a jump larger than π almost certainly means the polyline
    // crossed the seam (the largest legitimate step in a dense sample is
    // 2π / samples_per_edge, which for samples_per_edge=32 is ~0.2 rad).
    const U_JUMP: f64 = std::f64::consts::PI;
    const V_JUMP: f64 = std::f64::consts::PI / 2.0;

    let mut result: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut current: Vec<(f64, f64)> = Vec::with_capacity(uv.len());
    current.push(uv[0]);
    for window in uv.windows(2) {
        let (u0, v0) = window[0];
        let (u1, v1) = window[1];
        let du = (u1 - u0).abs();
        let dv = (v1 - v0).abs();
        let jumped = (u_periodic && du > U_JUMP) || (v_periodic && dv > V_JUMP);
        if jumped {
            if current.len() >= 2 {
                result.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
            current.push((u1, v1));
        } else {
            current.push((u1, v1));
        }
    }
    if current.len() >= 2 {
        result.push(current);
    }
    if result.is_empty() {
        result.push(uv);
    }
    result
}

/// Unwrap a UV triangle across a periodic seam if necessary.
///
/// If two of the triangle's U coordinates are on opposite sides of the
/// seam (one near 0, the other near 2π), this function shifts the side
/// near 2π down by 2π so the triangle becomes contiguous. The shifted
/// triangle may extend outside [0, 2π], but the renderer's `point_in_polygon`
/// test still works correctly because the polygon is now non-wrapping.
///
/// Returns the (possibly shifted) triangle. If no shift is needed, returns
/// the triangle unchanged.
fn unwrap_triangle_seam(
    tri: [(f64, f64); 3],
    u_periodic: bool,
    v_periodic: bool,
    u_period: f64,
    v_period: f64,
) -> [(f64, f64); 3] {
    let mut result = tri;
    if u_periodic && u_period > 0.0 {
        // Check if any pair has a jump > u_period / 2
        let half = u_period / 2.0;
        let us = [tri[0].0, tri[1].0, tri[2].0];
        let max_u = us.iter().cloned().fold(f64::MIN, f64::max);
        let min_u = us.iter().cloned().fold(f64::MAX, f64::min);
        if max_u - min_u > half {
            // Shift the points with u > half down by u_period.
            // After shifting, all points will be in [-u_period/2, u_period/2]
            // roughly, which is contiguous.
            for pt in &mut result {
                if pt.0 > half {
                    pt.0 -= u_period;
                }
            }
        }
    }
    if v_periodic && v_period > 0.0 {
        let half = v_period / 2.0;
        let vs = [tri[0].1, tri[1].1, tri[2].1];
        let max_v = vs.iter().cloned().fold(f64::MIN, f64::max);
        let min_v = vs.iter().cloned().fold(f64::MAX, f64::min);
        if max_v - min_v > half {
            for pt in &mut result {
                if pt.1 > half {
                    pt.1 -= v_period;
                }
            }
        }
    }
    result
}

/// Build a `Solid` from a STEP `DetailedMeshInstance`'s face list.
///
/// STEP files store per-face surface geometry in `FaceInfo.surface` (a
/// `Surface` enum), but the viewer's UV breakdown pipeline works on
/// `Solid`/`Shell`/`Face` from `draper-topology`. This helper bridges the
/// two representations: each `FaceInfo.surface` is wrapped in
/// `Face::new_surface_only` (no outer wire, so the breakdown falls back
/// to the surface's natural UV domain — exactly the same path used by
/// the NURBS gallery loaders via `build_nurbs_surface_mesh`).
///
/// The resulting Solid is what powers the UV Breakdown window when a
/// STEP file is loaded (no `current_solid` is set for STEP imports).
fn solid_from_detailed_instance(inst: &DetailedMeshInstance) -> Solid {
    let faces: Vec<Face> = inst
        .faces
        .iter()
        .map(|fi| Face::new_surface_only(fi.surface.clone()))
        .collect();
    Solid::new(Shell::new(faces))
}

/// Compute the UV breakdown for every face of a solid.
///
/// For each face, samples the outer wire (and any inner wires / holes)
/// into 3D polylines, then projects each 3D point onto the face's
/// underlying surface via `Surface::project_point` to obtain UV
/// coordinates. The result is a per-face list of UV polylines.
///
/// Returns an empty `SolidUvBreakdown` if the solid has no outer shell.
///
/// When `detailed_instance` is provided (STEP-loaded meshes), the UV
/// triangles and boundary polylines are taken DIRECTLY from the
/// `FaceInfo` (which contains the actual triangulation produced by the
/// STEP converter). Without this, the UV breakdown window would fall
/// back to a synthetic square grid because `Face::new_surface_only`
/// (used by `solid_from_detailed_instance`) has no wires for
/// `triangulate_face` to use — producing an empty mesh and triggering
/// the square-grid fallback. This was the root cause of the "Plane
/// face shows square grid instead of circle with hole" bug on
/// test/3.05.078.stp face #87.
fn compute_solid_uv_breakdown(solid: &Solid, model_name: &str) -> SolidUvBreakdown {
    compute_solid_uv_breakdown_with_detailed(solid, model_name, None)
}

fn compute_solid_uv_breakdown_with_detailed(
    solid: &Solid,
    model_name: &str,
    detailed_instance: Option<&DetailedMeshInstance>,
) -> SolidUvBreakdown {
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
        let u_periodic = surface.is_u_periodic();
        let v_periodic = surface.is_v_periodic();
        let mut outer_polylines: Vec<Vec<(f64, f64)>> = Vec::new();

        // ─── Prefer FaceInfo from DetailedMeshInstance when available ────
        // The FaceInfo has the ACTUAL outer/inner boundary polylines and
        // uv_triangles produced by the STEP converter — these reflect the
        // real triangulation with holes, arcs, and shared-edge consistency.
        // Without this, we'd only have `Face::new_surface_only` (no wires),
        // and triangulate_face would return an empty mesh → square grid
        // fallback (the user-visible "Plane shows square" bug).
        let face_info = detailed_instance.and_then(|inst| inst.faces.get(fidx));
        if let Some(fi) = face_info {
            // Use FaceInfo's outer_uv_boundary (2D UV polylines) directly
            // instead of re-projecting 3D boundary points to UV.
            // Re-projecting via surface.project_point() can produce different
            // u values than the original triangulation used (e.g., for periodic
            // surfaces where project_point returns u in [0, 2π) while the
            // triangulation used unwrapped u values). Using the stored UV
            // boundary ensures the boundary and UV triangles are in the
            // same coordinate system.
            for poly2d in &fi.outer_uv_boundary {
                let uv_raw: Vec<(f64, f64)> = poly2d
                    .iter()
                    .map(|p| (p.u, p.v))
                    .collect();
                let uv = split_at_seam_jumps(uv_raw, u_periodic, v_periodic);
                for poly in uv {
                    if poly.len() >= 2 {
                        outer_polylines.push(poly);
                    }
                }
            }
        }

        if let Some(ow) = face.outer_wire.as_ref() {
            if !ow.coedges.is_empty() {
                let pts3d = sample_wire_polyline(ow, &face.edges, samples_per_edge);
                let uv_raw: Vec<(f64, f64)> = pts3d
                    .iter()
                    .map(|p| surface.project_point(p))
                    .collect();
                // For periodic surfaces, the boundary polyline may wrap
                // across the seam (u jumps from 2π-ε to 0+ε, or v similarly).
                // The renderer would otherwise draw a horizontal line across
                // the entire UV plane. Split the polyline at large jumps.
                let uv = split_at_seam_jumps(uv_raw, u_periodic, v_periodic);
                for poly in uv {
                    if poly.len() >= 2 {
                        outer_polylines.push(poly);
                    }
                }
            }
        }
        // If the outer wire is empty (common for the lateral face of cones
        // and cylinders where the only edge is the bottom circle, stored
        // in face.edges but not in the wire), generate a synthetic UV
        // boundary rectangle using the surface's natural parametric domain.
        // This ensures the UV breakdown window shows the actual UV space
        // instead of falling back to the [0,1]x[0,1] default.
        if outer_polylines.is_empty() {
            let (u0, u1, v0, v1) = surface.natural_uv_domain();
            if u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()
                && u1 > u0 && v1 > v0
            {
                // Build a rectangle going counter-clockwise in UV space.
                // For U-periodic surfaces (cone, cylinder, sphere, torus),
                // the rectangle naturally represents the full periodic
                // domain [0, 2π] x [v0, v1].
                let n = 64; // sample density along periodic edges
                let mut rect = Vec::new();
                // Bottom edge: v = v0, u from u0 to u1
                for i in 0..n {
                    let t = i as f64 / n as f64;
                    let u = u0 + t * (u1 - u0);
                    rect.push((u, v0));
                }
                // Right edge: u = u1, v from v0 to v1
                for i in 0..n {
                    let t = i as f64 / n as f64;
                    let v = v0 + t * (v1 - v0);
                    rect.push((u1, v));
                }
                // Top edge: v = v1, u from u1 to u0
                for i in 0..n {
                    let t = i as f64 / n as f64;
                    let u = u1 - t * (u1 - u0);
                    rect.push((u, v1));
                }
                // Left edge: u = u0, v from v1 to v0
                for i in 0..n {
                    let t = i as f64 / n as f64;
                    let v = v1 - t * (v1 - v0);
                    rect.push((u0, v));
                }
                // Close the loop
                rect.push((u0, v0));
                outer_polylines.push(rect);
            }
        }

        let mut inner_polylines: Vec<Vec<(f64, f64)>> = Vec::new();
        // Prefer FaceInfo's inner_boundaries
        if let Some(fi) = face_info {
            // Use FaceInfo's inner_uv_boundaries (2D UV polylines) directly
            // for the same reason as outer_uv_boundary above.
            for hole_group in &fi.inner_uv_boundaries {
                for poly2d in hole_group {
                    let uv_raw: Vec<(f64, f64)> = poly2d
                        .iter()
                        .map(|p| (p.u, p.v))
                        .collect();
                    let uv = split_at_seam_jumps(uv_raw, u_periodic, v_periodic);
                    for poly in uv {
                        if poly.len() >= 2 {
                            inner_polylines.push(poly);
                        }
                    }
                }
            }
        }
        for iw in &face.inner_wires {
            let pts3d = sample_wire_polyline(iw, &face.edges, samples_per_edge);
            let uv_raw: Vec<(f64, f64)> = pts3d
                .iter()
                .map(|p| surface.project_point(p))
                .collect();
            let uv = split_at_seam_jumps(uv_raw, u_periodic, v_periodic);
            for poly in uv {
                if poly.len() >= 2 {
                    inner_polylines.push(poly);
                }
            }
        }

        // ─── Compute UV triangles for the face ─────────────────────────────
        // We triangulate the face using the same triangulator the renderer
        // uses, then project every triangle vertex back into UV space via
        // Surface::project_point. This gives us the actual UV tessellation
        // that the user wants to see and save.
        //
        // For periodic surfaces, we unwrap triangles that cross the seam
        // (e.g., one vertex at u=0.1 and another at u=2π-0.1) by shifting
        // the high-u side down by 2π. Without this, the renderer would
        // draw stretched triangles spanning the entire UV plane.
        //
        // PREFER FaceInfo.uv_triangles when available — these are the
        // ACTUAL UV triangles produced by the STEP converter during the
        // real mesh build, including holes, arcs, and shared-edge
        // consistency. The `triangulate_face` call below only works when
        // the Face has wires (i.e. primitives), but for STEP files the
        // Face is created with `Face::new_surface_only` (no wires), so
        // triangulate_face would return an empty mesh and we'd fall back
        // to a synthetic square grid (the "Plane shows square grid
        // instead of circle with hole" bug).
        let mut uv_triangles: Vec<[(f64, f64); 3]> = Vec::new();
        let u_period = if u_periodic { 2.0 * std::f64::consts::PI } else { 0.0 };
        let v_period = if v_periodic {
            // Sphere's v range is [0, π], so v_period = π; torus is [0, 2π].
            match surface {
                Surface::Sphere(_) => std::f64::consts::PI,
                Surface::Torus(_) => 2.0 * std::f64::consts::PI,
                _ => 2.0 * std::f64::consts::PI,
            }
        } else { 0.0 };

        if let Some(fi) = face_info {
            if !fi.uv_triangles.is_empty() {
                uv_triangles.reserve(fi.uv_triangles.len());
                for tri in &fi.uv_triangles {
                    let raw = [(tri[0].u, tri[0].v), (tri[1].u, tri[1].v), (tri[2].u, tri[2].v)];
                    let unwrapped = unwrap_triangle_seam(raw, u_periodic, v_periodic, u_period, v_period);
                    uv_triangles.push(unwrapped);
                }
            }
        }

        if uv_triangles.is_empty() {
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
                        let raw = [(u0, vv0), (u1, vv1), (u2, vv2)];
                        let unwrapped = unwrap_triangle_seam(raw, u_periodic, v_periodic, u_period, v_period);
                        uv_triangles.push(unwrapped);
                    }
                }
            }
        }

        // ─── Fallback: synthetic UV grid triangles ─────────────────────
        // When `triangulate_face` returns no triangles — which happens for
        // NURBS surfaces loaded via the gallery (the face is constructed
        // with `Face::new_surface_only`, i.e. no outer wire, so the
        // triangulator has no boundary to triangulate) — generate UV
        // triangles by sampling the surface's natural UV domain on a
        // regular grid. This ensures the UV breakdown window ALWAYS shows
        // triangles, even for wire-less NURBS faces.
        //
        // The grid resolution (20×20 cells → 800 triangles) is dense
        // enough to show the UV domain structure clearly, but not so
        // dense that it clutters the view. Each grid cell produces 2
        // triangles (a/b split along the diagonal).
        if uv_triangles.is_empty() {
            let (u0, u1, v0, v1) = surface.natural_uv_domain();
            if u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()
                && u1 > u0 && v1 > v0
            {
                const GRID_N: usize = 20;
                let du = (u1 - u0) / GRID_N as f64;
                let dv = (v1 - v0) / GRID_N as f64;
                uv_triangles.reserve(GRID_N * GRID_N * 2);
                for j in 0..GRID_N {
                    for i in 0..GRID_N {
                        let ua = u0 + i as f64 * du;
                        let ub = ua + du;
                        let va = v0 + j as f64 * dv;
                        let vb = va + dv;
                        // Two triangles per cell (a/b split along the
                        // diagonal from (ua,va) to (ub,vb)).
                        uv_triangles.push([(ua, va), (ub, va), (ub, vb)]);
                        uv_triangles.push([(ua, va), (ub, vb), (ua, vb)]);
                    }
                }
            }
        }

        let (u_metric_scale, v_metric_scale) = surface.uv_metric_scale();

        // ─── Normalize UV coordinates for periodic surfaces ──────────────
        // For U-periodic surfaces, the boundary UVs and triangle UVs may be
        // in different u-ranges. For example, a half-cone face covering u from
        // π to 2π might have boundary points at u=[π, 2π] while the UV triangles
        // (projected from 3D via project_point) have u in [0, π] (because
        // project_point maps the 2π seam to u=0). This creates a visual
        // mismatch where the green boundary appears far from the blue triangles.
        //
        // Fix: anchor the u-range to the BOUNDARY polylines (which are the
        // ground truth), then shift any triangle vertices that are in the
        // wrong "period cell" by ±u_period to bring them into the same range.
        // We also need to handle the reverse: boundary might need shifting
        // if triangles are the anchor (e.g., for synthetic grids).
        if u_periodic && u_period > 0.0 && !uv_triangles.is_empty() && !outer_polylines.is_empty() {
            // Find the median u of boundary polylines as the anchor
            let mut boundary_us: Vec<f64> = Vec::new();
            for poly in &outer_polylines {
                boundary_us.extend(poly.iter().map(|p| p.0));
            }
            boundary_us.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let boundary_median = if boundary_us.is_empty() { 0.0 } else { boundary_us[boundary_us.len() / 2] };

            // Find median u of triangle vertices
            let mut tri_us: Vec<f64> = Vec::with_capacity(uv_triangles.len() * 3);
            for tri in &uv_triangles {
                tri_us.extend_from_slice(&[tri[0].0, tri[1].0, tri[2].0]);
            }
            tri_us.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let tri_median = tri_us[tri_us.len() / 2];

            // If the medians are more than half a period apart, shift the
            // outlier by ±u_period to bring them into the same range.
            // Use >= to handle the exact π offset case (boundary at 3π/2,
            // triangles at π/2, difference = π = half period).
            let du = boundary_median - tri_median;
            let tri_shift = if du >= u_period / 2.0 {
                // Triangles are in a lower period cell — shift up
                u_period
            } else if du <= -u_period / 2.0 {
                // Triangles are in a higher period cell — shift down
                -u_period
            } else {
                0.0
            };

            if tri_shift.abs() > 0.0 {
                for tri in &mut uv_triangles {
                    for pt in tri.iter_mut() {
                        pt.0 += tri_shift;
                    }
                }
            }

            // After shifting triangles, also check inner polylines against
            // the outer boundary range (they might also need shifting).
            let outer_median = boundary_median; // anchor
            for poly in &mut inner_polylines {
                if poly.is_empty() { continue; }
                let mut poly_us: Vec<f64> = poly.iter().map(|p| p.0).collect();
                poly_us.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let poly_median = poly_us[poly_us.len() / 2];
                let dv = poly_median - outer_median;
                let hole_shift = if dv >= u_period / 2.0 {
                    -u_period
                } else if dv <= -u_period / 2.0 {
                    u_period
                } else {
                    0.0
                };
                if hole_shift.abs() > 0.0 {
                    for p in poly.iter_mut() { p.0 += hole_shift; }
                }
            }
        }

        // Same normalization for V-periodic surfaces
        if v_periodic && v_period > 0.0 && !uv_triangles.is_empty() && !outer_polylines.is_empty() {
            let mut boundary_vs: Vec<f64> = Vec::new();
            for poly in &outer_polylines {
                boundary_vs.extend(poly.iter().map(|p| p.1));
            }
            boundary_vs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let boundary_median = if boundary_vs.is_empty() { 0.0 } else { boundary_vs[boundary_vs.len() / 2] };

            let mut tri_vs: Vec<f64> = Vec::with_capacity(uv_triangles.len() * 3);
            for tri in &uv_triangles {
                tri_vs.extend_from_slice(&[tri[0].1, tri[1].1, tri[2].1]);
            }
            tri_vs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let tri_median = tri_vs[tri_vs.len() / 2];

            let dv = boundary_median - tri_median;
            let tri_shift = if dv >= v_period / 2.0 {
                v_period
            } else if dv <= -v_period / 2.0 {
                -v_period
            } else {
                0.0
            };

            if tri_shift.abs() > 0.0 {
                for tri in &mut uv_triangles {
                    for pt in tri.iter_mut() {
                        pt.1 += tri_shift;
                    }
                }
            }

            for poly in &mut inner_polylines {
                if poly.is_empty() { continue; }
                let mut poly_vs: Vec<f64> = poly.iter().map(|p| p.1).collect();
                poly_vs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let poly_median = poly_vs[poly_vs.len() / 2];
                let dv = poly_median - boundary_median;
                let hole_shift = if dv >= v_period / 2.0 {
                    -v_period
                } else if dv <= -v_period / 2.0 {
                    v_period
                } else {
                    0.0
                };
                if hole_shift.abs() > 0.0 {
                    for p in poly.iter_mut() { p.1 += hole_shift; }
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
            u_periodic,
            v_periodic,
            u_period,
            v_period,
            u_metric_scale,
            v_metric_scale,
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
    let draw_w_max = svg_width - 2.0 * margin;
    let draw_h_max = svg_height - 2.0 * margin;

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
    // Also include UV triangle extents in the bounds, since unwrapped
    // triangles on periodic surfaces may extend outside the boundary's
    // nominal UV range.
    for tri in &face_uv.uv_triangles {
        for &(u, v) in tri {
            u_min = u_min.min(u); u_max = u_max.max(u);
            v_min = v_min.min(v); v_max = v_max.max(v);
        }
    }
    if u_min >= u_max || v_min >= v_max {
        // Fallback: use the surface's natural parametric domain if known.
        if let Some(s) = surface {
            let (u0, u1, v0, v1) = s.natural_uv_domain();
            u_min = u0; u_max = u1; v_min = v0; v_max = v1;
        } else {
            u_min = 0.0; u_max = 1.0; v_min = 0.0; v_max = 1.0;
        }
    }
    let u_range = (u_max - u_min).max(1e-6);
    let v_range = (v_max - v_min).max(1e-6);
    u_min -= u_range * 0.05; u_max += u_range * 0.05;
    v_min -= v_range * 0.05; v_max += v_range * 0.05;

    // ─── Aspect-ratio-preserving layout ──────────────────────────────
    // Fit the UV box into the square draw area while preserving its real
    // aspect ratio. This matches the interactive viewer's behavior: a
    // 2π×1 cone UV is rendered as a wide rectangle, not a square.
    let u_range_vis = (u_max - u_min).max(1e-12);
    let v_range_vis = (v_max - v_min).max(1e-12);
    let ar_uv = u_range_vis / v_range_vis;
    let (draw_w, draw_h) = if ar_uv >= 1.0 {
        (draw_w_max, draw_w_max / ar_uv)
    } else {
        (draw_h_max * ar_uv, draw_h_max)
    };
    let x_offset = (svg_width - draw_w) * 0.5;
    let y_offset = (svg_height - draw_h) * 0.5;

    let map_u = |u: f64| -> f64 { x_offset + (u - u_min) / u_range_vis * draw_w };
    let map_v = |v: f64| -> f64 { y_offset + (1.0 - (v - v_min) / v_range_vis) * draw_h };

    // Screen coordinates of the UV box corners.
    let box_left_x = x_offset;
    let box_right_x = x_offset + draw_w;
    let box_top_y = y_offset;
    let box_bottom_y = y_offset + draw_h;

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

    // UV box border (subtle, to show the real aspect ratio).
    svg.push_str(&format!(
        "  <rect x=\"{:.2}\" y=\"{:.2}\" width=\"{:.2}\" height=\"{:.2}\" fill=\"none\" stroke=\"#3c3c5a\" stroke-width=\"0.5\"/>\n",
        box_left_x, box_top_y, draw_w, draw_h
    ));

    // Grid lines — span the UV box (not the full draw area).
    for i in 0..=u_divs {
        let u = u_min + (u_max - u_min) * i as f64 / u_divs as f64;
        let x = map_u(u);
        svg.push_str(&format!(
            "  <line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#334\" stroke-width=\"0.5\"/>\n",
            x, box_top_y, x, box_bottom_y
        ));
    }
    for j in 0..=v_divs {
        let v = v_min + (v_max - v_min) * j as f64 / v_divs as f64;
        let y = map_v(v);
        svg.push_str(&format!(
            "  <line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#334\" stroke-width=\"0.5\"/>\n",
            box_left_x, y, box_right_x, y
        ));
    }

    // ─── Seam lines (for periodic surfaces) ──────────────────────────
    // Draw the seam as a bright yellow line so the user can see where
    // the surface wraps around. For U-periodic surfaces, the seam is at
    // u = natural_u_min (equivalently natural_u_max). For V-periodic
    // surfaces, the seam is at v = natural_v_min (equivalently v_max).
    if face_uv.u_periodic && face_uv.u_period > 0.0 {
        if let Some(s) = surface {
            let (nat_u0, nat_u1, _nv0, _nv1) = s.natural_uv_domain();
            for &seam_u in &[nat_u0, nat_u1] {
                if seam_u.is_finite() && seam_u >= u_min && seam_u <= u_max {
                    let x = map_u(seam_u);
                    svg.push_str(&format!(
                        "  <line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#ffc800\" stroke-width=\"2.0\"/>\n",
                        x, box_top_y, x, box_bottom_y
                    ));
                }
            }
        }
    }
    if face_uv.v_periodic && face_uv.v_period > 0.0 {
        if let Some(s) = surface {
            let (_nu0, _nu1, nat_v0, nat_v1) = s.natural_uv_domain();
            for &seam_v in &[nat_v0, nat_v1] {
                if seam_v.is_finite() && seam_v >= v_min && seam_v <= v_max {
                    let y = map_v(seam_v);
                    svg.push_str(&format!(
                        "  <line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#ffc800\" stroke-width=\"2.0\"/>\n",
                        box_left_x, y, box_right_x, y
                    ));
                }
            }
        }
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
    // Each triangle has a clearly visible edge stroke so the user can
    // see the triangulation structure, not just colored regions.
    let hole_polys: &[Vec<(f64, f64)>] = &face_uv.inner_polylines;
    if !face_uv.uv_triangles.is_empty() {
        let tri_limit = 5000.min(face_uv.uv_triangles.len());
        svg.push_str("  <g>\n");
        for (ti, tri) in face_uv.uv_triangles.iter().enumerate() {
            let cu = (tri[0].0 + tri[1].0 + tri[2].0) / 3.0;
            let cv = (tri[0].1 + tri[1].1 + tri[2].1) / 3.0;
            let in_hole = hole_polys.iter().any(|h| point_in_polygon(cu, cv, h));
            let in_outer = !outer_uv_poly.is_empty() && point_in_polygon(cu, cv, &outer_uv_poly);
            let (fill, fill_op, stroke, stroke_op, stroke_w) = if in_hole || !in_outer {
                ("#ff2222", "0.18", "#ff4444", "0.9", "0.8")
            } else if ti % 2 == 0 {
                ("#4488ff", "0.22", "#b4dcff", "0.95", "0.8")
            } else {
                ("#55aaff", "0.22", "#b4dcff", "0.95", "0.8")
            };
            svg.push_str(&format!(
                "    <polygon points=\"{:.2},{:.2} {:.2},{:.2} {:.2},{:.2}\" fill=\"{}\" fill-opacity=\"{}\" stroke=\"{}\" stroke-width=\"{}\" stroke-opacity=\"{}\" stroke-linejoin=\"round\"/>\n",
                map_u(tri[0].0), map_v(tri[0].1),
                map_u(tri[1].0), map_v(tri[1].1),
                map_u(tri[2].0), map_v(tri[2].1),
                fill, fill_op, stroke, stroke_w, stroke_op
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
        box_left_x + draw_w / 2.0, svg_height - 5.0, u_min, u_max
    ));
    svg.push_str(&format!(
        "  <text x=\"10\" y=\"{}\" fill=\"#aaa\" font-size=\"12\" text-anchor=\"middle\" transform=\"rotate(-90, 10, {})\">V ({:.2} .. {:.2})</text>\n",
        box_top_y + draw_h / 2.0, box_top_y + draw_h / 2.0, v_min, v_max
    ));
    svg.push_str(&format!(
        "  <text x=\"{}\" y=\"20\" fill=\"#fff\" font-size=\"13\" text-anchor=\"middle\">Face #{} {} forward={} [{}]</text>\n",
        svg_width / 2.0, face_uv.face_idx, face_uv.surface_type, face_uv.forward, model_name
    ));
    // Seam legend (only if any seam was drawn)
    if (face_uv.u_periodic || face_uv.v_periodic) && (face_uv.u_period > 0.0 || face_uv.v_period > 0.0) {
        svg.push_str(&format!(
            "  <text x=\"{}\" y=\"{}\" fill=\"#ffc800\" font-size=\"11\" text-anchor=\"end\">━━ seam (periodic wrap)</text>\n",
            svg_width - 10.0, svg_height - 5.0
        ));
    }

    svg.push_str("</svg>\n");
    svg
}

/// Draw an assembly tree node recursively (static function to avoid borrow conflicts).
///
/// Internal nodes (subassemblies) now show:
/// - Folder icon + child count in label
/// - Eye icon to show/hide the entire subtree
/// - Isolate button to show only this subtree's instances
fn draw_assembly_node_static(
    ui: &mut egui::Ui,
    node: &AssemblyNode,
    selected_instance: Option<usize>,
    hidden_instances: &std::collections::HashSet<usize>,
    pending_instance_select: &mut Option<usize>,
    pending_visibility_toggle: &mut Option<usize>,
    pending_instance_isolate: &mut Option<usize>,
    pending_subtree_hide: &mut Option<Vec<usize>>,
    pending_subtree_show: &mut Option<Vec<usize>>,
    pending_subtree_isolate: &mut Option<Vec<usize>>,
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

    // Use instance_index for selection (exact mapping to instance)
    // usize::MAX sentinel means "failed triangulation" — not selectable
    let is_selected = node.instance_index.map_or(false, |idx| idx != usize::MAX && selected_instance == Some(idx));

    if has_children {
        // ─── Internal node (subassembly) ─────────────────────────────────
        // Collect all descendant instance indices for subtree operations
        let subtree_indices = collect_subtree_instance_indices(node);
        let subtree_count = subtree_indices.len();
        // Count how many subtree instances are currently visible
        let visible_subtree_count = subtree_indices.iter().filter(|idx| !hidden_instances.contains(idx)).count();
        let all_subtree_hidden = !subtree_indices.is_empty() && visible_subtree_count == 0;

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
            ui.horizontal(|ui| {
                // ─── Subtree visibility toggle (eye icon) ─────────────
                if !subtree_indices.is_empty() {
                    let eye_color = if all_subtree_hidden {
                        egui::Color32::from_rgb(180, 80, 80)
                    } else if visible_subtree_count < subtree_count {
                        // Partially visible — amber color
                        egui::Color32::from_rgb(200, 160, 40)
                    } else {
                        egui::Color32::from_rgb(80, 180, 80)
                    };
                    let eye_text = if all_subtree_hidden { "  " } else { "👁" };
                    if ui.add(egui::Label::new(egui::RichText::new(eye_text).size(11.0).color(eye_color)).sense(egui::Sense::click())).clicked() {
                        if all_subtree_hidden {
                            *pending_subtree_show = Some(subtree_indices.clone());
                        } else {
                            *pending_subtree_hide = Some(subtree_indices.clone());
                        }
                    }
                }
                // ─── Subtree isolate button ───────────────────────────
                if subtree_count > 0 {
                    let isolate_btn = egui::Button::new(
                        egui::RichText::new("◎").size(11.0)
                    ).frame(false);
                    if ui.add(isolate_btn).on_hover_text(
                        "Isolate subtree: hide all other instances (click again to restore)"
                    ).clicked() {
                        *pending_subtree_isolate = Some(subtree_indices.clone());
                    }
                }
                // ─── Label with folder icon and child count ───────────
                let count_str = if subtree_count > 0 {
                    format!(" ({} part{})", subtree_count, if subtree_count != 1 { "s" } else { "" })
                } else {
                    String::new()
                };
                let label = format!("[+] {}{}{}{}", node.name, brep_str, inst_str, count_str);
                ui.label(egui::RichText::new(&label).size(11.0));
            });
        }).body(|ui| {
            for child in &node.children {
                draw_assembly_node_static(ui, child, selected_instance, hidden_instances, pending_instance_select, pending_visibility_toggle, pending_instance_isolate, pending_subtree_hide, pending_subtree_show, pending_subtree_isolate, open_tree_nodes, scroll_to_tree_node);
            }
        });
    } else {
        // ─── Leaf node: draw visibility checkbox + isolate button + selectable label ───
        let label = format!("{}{}{}", node.name, brep_str, inst_str);
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
                    // Isolate button — "solo" icon (◎). When clicked, hides all
                    // OTHER instances so only this one remains visible. If this
                    // instance is already the only visible one, clicking again
                    // restores visibility of all instances (toggle behavior).
                    let isolate_btn = egui::Button::new(
                        egui::RichText::new("◎").size(11.0)
                    ).frame(false);
                    if ui.add(isolate_btn).on_hover_text(
                        "Isolate: hide all other instances (click again to restore)"
                    ).clicked() {
                        *pending_instance_isolate = Some(idx);
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

/// Collect all valid instance indices from the subtree rooted at `node`.
/// Only leaf nodes with a valid (non-MAX) instance_index are included.
/// Used for subtree-level visibility/isolate operations on subassembly nodes.
fn collect_subtree_instance_indices(node: &AssemblyNode) -> Vec<usize> {
    let mut result = Vec::new();
    collect_subtree_instance_indices_recursive(node, &mut result);
    result
}

fn collect_subtree_instance_indices_recursive(node: &AssemblyNode, out: &mut Vec<usize>) {
    if let Some(idx) = node.instance_index {
        if idx != usize::MAX {
            out.push(idx);
        }
    }
    for child in &node.children {
        collect_subtree_instance_indices_recursive(child, out);
    }
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

    /// Draw GD&T annotations as 2D overlays on the 3D viewport.
    ///
    /// For each geometric tolerance extracted from the STEP file, this method:
    /// 1. Resolves the `applied_to` entity → ADVANCED_FACE → FaceInfo
    /// 2. Computes the centroid of the face boundary in 3D
    /// 3. Projects the centroid to screen coordinates
    /// 4. Draws a leader line + tolerance frame label at an offset from the centroid
    fn draw_gdt_annotations(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        let gdt_data = match &self.gdt_data {
            Some(d) => d,
            None => return,
        };

        if gdt_data.tolerances.is_empty() {
            return;
        }

        // Compute MVP for world-to-screen projection
        let view = self.camera.view_matrix();
        let aspect = rect.width() / rect.height();
        let proj = self.camera.projection_matrix(aspect);
        let mvp = mat4_mul(&proj, &view);

        // Build a lookup: shape_aspect.relating_shape → step_face_id
        // The chain is: tolerance.applied_to → shape_aspect.step_id → relating_shape (ADVANCED_FACE ID)
        let mut sa_to_face_id: std::collections::HashMap<i64, i64> = std::collections::HashMap::new();
        for sa in &gdt_data.shape_aspects {
            if let Some(rs) = sa.relating_shape {
                sa_to_face_id.insert(sa.step_id, rs);
            }
        }

        // Build a lookup: step_face_id → centroid (average of outer boundary points)
        // Search all detailed_instances for matching faces
        let mut face_centroids: std::collections::HashMap<i64, [f32; 3]> = std::collections::HashMap::new();
        for inst in &self.detailed_instances {
            for face in &inst.faces {
                // Compute centroid from outer_boundary
                let mut cx = 0.0f64;
                let mut cy = 0.0f64;
                let mut cz = 0.0f64;
                let mut count = 0u32;
                for poly in &face.outer_boundary {
                    for pt in poly {
                        cx += pt.x;
                        cy += pt.y;
                        cz += pt.z;
                        count += 1;
                    }
                }
                if count > 0 {
                    let inv = 1.0 / count as f64;
                    face_centroids.insert(
                        face.step_face_id,
                        [(cx * inv) as f32, (cy * inv) as f32, (cz * inv) as f32],
                    );
                }
            }
        }

        // Project 3D → 2D helper
        let world_to_screen = |p: [f32; 3]| -> Option<egui::Pos2> {
            let x = mvp[0][0] * p[0] + mvp[1][0] * p[1] + mvp[2][0] * p[2] + mvp[3][0];
            let y = mvp[0][1] * p[0] + mvp[1][1] * p[1] + mvp[2][1] * p[2] + mvp[3][1];
            let w = mvp[0][3] * p[0] + mvp[1][3] * p[1] + mvp[2][3] * p[2] + mvp[3][3];
            if w.abs() < 1e-6 {
                return None; // Behind camera or degenerate
            }
            let ndc_x = x / w;
            let ndc_y = y / w;
            // Check if behind camera
            if w < 0.0 {
                return None;
            }
            // NDC [-1,1] → screen
            let sx = rect.left() + (ndc_x * 0.5 + 0.5) * rect.width();
            let sy = rect.top() + (1.0 - (ndc_y * 0.5 + 0.5)) * rect.height();
            // Clip to viewport with some margin
            if sx < rect.left() - 50.0 || sx > rect.right() + 50.0 ||
               sy < rect.top() - 50.0 || sy > rect.bottom() + 50.0 {
                return None;
            }
            Some(egui::pos2(sx, sy))
        };

        let painter = ui.painter();

        for tol in &gdt_data.tolerances {
            // Resolve tolerance.applied_to → face centroid
            let face_id = tol.applied_to
                .and_then(|aid| sa_to_face_id.get(&aid).copied())
                .or(tol.applied_to); // Fallback: maybe applied_to IS the face ID directly

            let centroid_3d = match face_id.and_then(|fid| face_centroids.get(&fid)) {
                Some(c) => *c,
                None => continue, // No matching face found
            };

            let anchor_2d = match world_to_screen(centroid_3d) {
                Some(p) => p,
                None => continue,
            };

            // Compute a label offset: shift right and up from the anchor point
            // The offset direction follows the camera's right/up but stays in screen space
            let offset_x = 60.0;
            let offset_y = -40.0;
            let label_pos = egui::pos2(anchor_2d.x + offset_x, anchor_2d.y + offset_y);

            // ─── Leader line (dashed look via two segments with a small gap) ───
            let mid_x = anchor_2d.x + offset_x * 0.3;
            let mid_y = anchor_2d.y + offset_y * 0.3;
            painter.line_segment(
                [anchor_2d, egui::pos2(mid_x, mid_y)],
                egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 200, 60)),
            );
            painter.line_segment(
                [egui::pos2(mid_x, mid_y), label_pos],
                egui::Stroke::new(1.0, egui::Color32::from_rgb(255, 200, 60)),
            );

            // ─── Small dot at the anchor (attachment point on the face) ───
            painter.circle_filled(anchor_2d, 3.0, egui::Color32::from_rgb(255, 200, 60));

            // ─── Tolerance frame (feature control frame) ───
            // Build the text content
            let type_symbol = match &tol.tolerance_type {
                draper_step::pmi::GdtToleranceType::Position => "\u{2295}",
                draper_step::pmi::GdtToleranceType::Flatness => "\u{2365}",
                draper_step::pmi::GdtToleranceType::Straightness => "\u{2500}",
                draper_step::pmi::GdtToleranceType::Circularity => "\u{25CB}",
                draper_step::pmi::GdtToleranceType::Cylindricity => "\u{232D}",
                draper_step::pmi::GdtToleranceType::Perpendicularity => "\u{22A5}",
                draper_step::pmi::GdtToleranceType::Parallelism => "\u{2225}",
                draper_step::pmi::GdtToleranceType::Angularity => "\u{2220}",
                draper_step::pmi::GdtToleranceType::Concentricity => "\u{25CE}",
                draper_step::pmi::GdtToleranceType::Symmetry => "\u{232F}",
                draper_step::pmi::GdtToleranceType::Runout => "\u{2197}",
                draper_step::pmi::GdtToleranceType::ProfileOfLine => "\u{2312}",
                draper_step::pmi::GdtToleranceType::ProfileOfSurface => "\u{2313}",
                draper_step::pmi::GdtToleranceType::Other(s) => {
                    // Use first characters of the name as fallback
                    if s.is_empty() { "?" } else { s.as_str() }
                }
            };

            let tol_text = match tol.tolerance_value {
                Some(v) => format!("{} {:.4}", type_symbol, v),
                None => format!("{}", type_symbol),
            };

            // Datum references
            let datum_text = if !tol.datum_references.is_empty() {
                let datums: Vec<String> = gdt_data.datum_features.iter()
                    .filter(|df| tol.datum_references.contains(&df.step_id))
                    .map(|df| if df.name.is_empty() { format!("#{}", df.step_id) } else { df.name.clone() })
                    .collect();
                if datums.is_empty() {
                    // Fallback: show raw entity IDs
                    let refs: Vec<String> = tol.datum_references.iter().map(|r| format!("#{}", r)).collect();
                    format!(" | {}", refs.join(" "))
                } else {
                    format!(" | {}", datums.join(" "))
                }
            } else {
                String::new()
            };

            let full_text = format!("{}{}", tol_text, datum_text);

            // Measure text for frame sizing
            let font_id = egui::FontId::proportional(11.0);
            let text_galley = painter.layout_no_wrap(full_text.clone(), font_id.clone(), egui::Color32::WHITE);
            let text_width = text_galley.size().x;
            let text_height = text_galley.size().y;

            // Frame padding
            let pad = 4.0;
            let frame_rect = egui::Rect::from_min_size(
                label_pos - egui::vec2(pad, pad),
                egui::vec2(text_width + pad * 2.0, text_height + pad * 2.0),
            );

            // Draw frame background
            painter.rect_filled(
                frame_rect,
                2.0,
                egui::Color32::from_rgba_premultiplied(30, 30, 50, 220),
            );
            // Draw frame border
            painter.rect_stroke(
                frame_rect,
                2.0,
                egui::Stroke::new(1.0, egui::Color32::from_rgb(255, 200, 60)),
                egui::StrokeKind::Outside,
            );

            // Draw separator lines for multi-cell FCF (Feature Control Frame)
            // Cell 1: symbol, Cell 2: tolerance value, Cell 3+: datum references
            let symbol_width = painter.layout_no_wrap(
                format!("{} ", type_symbol), font_id.clone(), egui::Color32::WHITE
            ).size().x;
            let tol_width = painter.layout_no_wrap(
                tol_text.clone(), font_id.clone(), egui::Color32::WHITE
            ).size().x;

            // Vertical separator after symbol
            let sep1_x = label_pos.x + symbol_width + pad * 0.5;
            painter.line_segment(
                [egui::pos2(sep1_x, frame_rect.top()), egui::pos2(sep1_x, frame_rect.bottom())],
                egui::Stroke::new(0.5, egui::Color32::from_rgb(200, 160, 40)),
            );

            // Vertical separator after tolerance value (if there are datums)
            if !tol.datum_references.is_empty() {
                let sep2_x = label_pos.x + tol_width + pad * 0.5;
                painter.line_segment(
                    [egui::pos2(sep2_x, frame_rect.top()), egui::pos2(sep2_x, frame_rect.bottom())],
                    egui::Stroke::new(0.5, egui::Color32::from_rgb(200, 160, 40)),
                );
            }

            // Draw text
            painter.galley(label_pos, text_galley, egui::Color32::WHITE);
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

// ============================================================
// BRepCAD UI integration
// ============================================================

impl ViewerApp {
    /// Handle a BRepCAD MenuAction by delegating to the existing ViewerApp methods.
    /// Returns a status message.
    pub fn handle_brepcad_action(&mut self, action: &crate::ui::menubar::MenuAction) -> String {
        use crate::ui::menubar::MenuAction;
        let msg = self.handle_brepcad_action_inner(action);
        // Update tool name and view orientation based on action
        match action {
            MenuAction::ViewIso => self.brepcad_view_orientation = "ISO".to_string(),
            MenuAction::ViewFront => self.brepcad_view_orientation = "Front".to_string(),
            MenuAction::ViewBack => self.brepcad_view_orientation = "Back".to_string(),
            MenuAction::ViewTop => self.brepcad_view_orientation = "Top".to_string(),
            MenuAction::ViewBottom => self.brepcad_view_orientation = "Bottom".to_string(),
            MenuAction::ViewLeft => self.brepcad_view_orientation = "Left".to_string(),
            MenuAction::ViewRight => self.brepcad_view_orientation = "Right".to_string(),
            MenuAction::ViewDimetric => self.brepcad_view_orientation = "Dimetric".to_string(),
            MenuAction::InsertBox => self.brepcad_active_tool = "Insert Box".to_string(),
            MenuAction::InsertSphere => self.brepcad_active_tool = "Insert Sphere".to_string(),
            MenuAction::InsertCylinder => self.brepcad_active_tool = "Insert Cylinder".to_string(),
            MenuAction::InsertCone => self.brepcad_active_tool = "Insert Cone".to_string(),
            MenuAction::InsertTorus => self.brepcad_active_tool = "Insert Torus".to_string(),
            MenuAction::ModifyUnion => self.brepcad_active_tool = "Boolean Union".to_string(),
            MenuAction::ModifySubtract => self.brepcad_active_tool = "Boolean Subtract".to_string(),
            MenuAction::ModifyIntersect => self.brepcad_active_tool = "Boolean Intersect".to_string(),
            MenuAction::ModifyFillet => self.brepcad_active_tool = "Fillet".to_string(),
            MenuAction::ModifyChamfer => self.brepcad_active_tool = "Chamfer".to_string(),
            MenuAction::ModifyMove => self.brepcad_active_tool = "Move".to_string(),
            MenuAction::ModifyRotate => self.brepcad_active_tool = "Rotate".to_string(),
            MenuAction::ModifyScale => self.brepcad_active_tool = "Scale".to_string(),
            MenuAction::SketchEnter => self.brepcad_active_tool = "Sketch".to_string(),
            MenuAction::SketchLine => self.brepcad_active_tool = "Line".to_string(),
            MenuAction::SketchCircle => self.brepcad_active_tool = "Circle".to_string(),
            MenuAction::SketchRectangle => self.brepcad_active_tool = "Rectangle".to_string(),
            MenuAction::ViewWireframe => { self.brepcad_active_tool = "Select".to_string(); }
            MenuAction::ViewShaded => { self.brepcad_active_tool = "Select".to_string(); }
            MenuAction::ViewShadedEdges => { self.brepcad_active_tool = "Select".to_string(); }
            _ => {}
        }
        msg
    }

    fn handle_brepcad_action_inner(&mut self, action: &crate::ui::menubar::MenuAction) -> String {
        use crate::ui::menubar::MenuAction;
        match action {
            MenuAction::None => String::new(),

            // ── File actions ──
            MenuAction::FileNew => {
                self.load_box();
                "New document created (default Box 100×100×100)".to_string()
            }
            MenuAction::FileOpen => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STEP", &["stp", "step"])
                        .add_filter("STL", &["stl"])
                        .add_filter("OBJ", &["obj"])
                        .add_filter("JSON", &["json"])
                        .pick_file()
                    {
                        let p = path.to_string_lossy().to_string();
                        let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_lowercase();
                        match ext.as_str() {
                            "stp" | "step" => self.import_step_file(&p),
                            "stl" => self.import_stl_file(&p),
                            "json" => self.import_json(&p),
                            _ => self.import_step_file(&p),
                        }
                        return format!("Opened: {}", p);
                    }
                }
                "Open cancelled".to_string()
            }
            MenuAction::FileSave | MenuAction::FileSaveAs => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STEP", &["stp"])
                        .set_file_name(&self.current_model.name)
                        .save_file()
                    {
                        self.export_step(&path.to_string_lossy());
                        return format!("Saved: {}", path.to_string_lossy());
                    }
                }
                "Save cancelled".to_string()
            }
            MenuAction::FileExportStep => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STEP", &["stp", "step"])
                        .save_file()
                    {
                        self.export_step(&path.to_string_lossy());
                        return format!("Exported STEP: {}", path.to_string_lossy());
                    }
                }
                "Export cancelled".to_string()
            }
            MenuAction::FileExportStl => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STL", &["stl"])
                        .save_file()
                    {
                        self.export_stl_binary(&path.to_string_lossy());
                        return format!("Exported STL: {}", path.to_string_lossy());
                    }
                }
                "Export cancelled".to_string()
            }
            MenuAction::FileImportStep => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STEP", &["stp", "step"])
                        .pick_file()
                    {
                        let p = path.to_string_lossy().to_string();
                        self.import_step_file(&p);
                        return format!("Imported STEP: {}", p);
                    }
                }
                "Import cancelled".to_string()
            }
            MenuAction::FileImportStl => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STL", &["stl"])
                        .pick_file()
                    {
                        let p = path.to_string_lossy().to_string();
                        self.import_stl_file(&p);
                        return format!("Imported STL: {}", p);
                    }
                }
                "Import cancelled".to_string()
            }
            MenuAction::FileQuit => {
                "Quit requested — close window to exit".to_string()
            }
            MenuAction::FilePrint => "Print dialog not yet implemented".to_string(),
            MenuAction::FileExportObj => "OBJ export not yet implemented (use STL)".to_string(),
            MenuAction::FileExportGltf => "GLTF export not yet implemented".to_string(),
            MenuAction::FileExportPdf => "PDF export not yet implemented".to_string(),
            MenuAction::FileExportDxf => "DXF export not yet implemented".to_string(),
            MenuAction::FileImportObj => "OBJ import not yet implemented (use STL)".to_string(),
            MenuAction::FileImportPly => "PLY import not yet implemented".to_string(),
            MenuAction::FileImportDxf => "DXF import not yet implemented".to_string(),
            MenuAction::FileImportPointCloud => "Point cloud import not yet implemented".to_string(),

            // ── Edit actions ──
            MenuAction::EditUndo => {
                if self.brepcad_undo() { "Undo applied".to_string() } else { "Nothing to undo".to_string() }
            }
            MenuAction::EditRedo => {
                if self.brepcad_redo() { "Redo applied".to_string() } else { "Nothing to redo".to_string() }
            }
            MenuAction::EditDuplicate => {
                // Re-load current solid as a duplicate (offset by ShapeBuilder translation)
                if let Some(solid) = self.current_solid.clone() {
                    self.brepcad_push_undo_named("Duplicate Solid");
                    let mut new_solid = solid;
                    use draper_geometry::Transform;
                    draper_topology::ShapeBuilder::transform_solid(&mut new_solid, &Transform::translation(20.0, 0.0, 0.0));
                    self.current_solid = Some(new_solid);
                    self.refresh_from_current_solid("Duplicate solid");
                    "Solid duplicated (+20mm X offset)".to_string()
                } else {
                    "No solid to duplicate".to_string()
                }
            }
            MenuAction::EditCut | MenuAction::EditCopy | MenuAction::EditPaste => {
                "Clipboard operations not yet implemented".to_string()
            }
            MenuAction::EditFind => "Find (parameters search) not yet implemented".to_string(),

            // ── View actions ──
            MenuAction::ViewIso => {
                let d = 45.0_f32.to_radians();
                let e = 30.0_f32.to_radians();
                self.camera.look_from_direction([
                    -e.cos() * d.sin(),
                    -e.sin(),
                    e.cos() * d.cos(),
                ]);
                "ISO view".to_string()
            }
            MenuAction::ViewFront => {
                self.camera.look_from_direction([0.0, 0.0, 1.0]);
                "Front view".to_string()
            }
            MenuAction::ViewBack => {
                self.camera.look_from_direction([0.0, 0.0, -1.0]);
                "Back view".to_string()
            }
            MenuAction::ViewTop => {
                self.camera.look_from_direction([0.0, -1.0, 0.0]);
                "Top view".to_string()
            }
            MenuAction::ViewBottom => {
                self.camera.look_from_direction([0.0, 1.0, 0.0]);
                "Bottom view".to_string()
            }
            MenuAction::ViewLeft => {
                self.camera.look_from_direction([1.0, 0.0, 0.0]);
                "Left view".to_string()
            }
            MenuAction::ViewRight => {
                self.camera.look_from_direction([-1.0, 0.0, 0.0]);
                "Right view".to_string()
            }
            MenuAction::ViewDimetric => {
                let d = 20.0_f32.to_radians();
                let e = 15.0_f32.to_radians();
                self.camera.look_from_direction([
                    -e.cos() * d.sin(),
                    -e.sin(),
                    e.cos() * d.cos(),
                ]);
                "Dimetric view".to_string()
            }
            MenuAction::ViewFit => {
                let (bbox_min, bbox_max) = self.mesh.bounding_box();
                self.camera.fit_to_bounding_box(
                    [bbox_min.x as f32, bbox_min.y as f32, bbox_min.z as f32],
                    [bbox_max.x as f32, bbox_max.y as f32, bbox_max.z as f32],
                );
                "Fit to view".to_string()
            }
            MenuAction::ViewZoomIn => { self.camera.zoom(-50.0, None); "Zoom in".to_string() }
            MenuAction::ViewZoomOut => { self.camera.zoom(50.0, None); "Zoom out".to_string() }
            MenuAction::ViewZoomWindow => "Zoom window: drag in viewport".to_string(),
            MenuAction::ViewZoomSelection => "Zoom to selection".to_string(),
            MenuAction::ViewWireframe => { self.wireframe = true; self.show_edges = false; "Wireframe display".to_string() }
            MenuAction::ViewShaded => { self.wireframe = false; self.show_edges = false; "Shaded display".to_string() }
            MenuAction::ViewShadedEdges => { self.wireframe = false; self.show_edges = true; "Shaded + Edges display".to_string() }
            MenuAction::ViewToggleGrid => { self.show_grid = !self.show_grid; format!("Grid: {}", if self.show_grid {"on"} else {"off"}) }
            MenuAction::ViewToggleAxis => { self.show_axes = !self.show_axes; format!("Axes: {}", if self.show_axes {"on"} else {"off"}) }
            MenuAction::ViewToggleTriad => { format!("Triad: {}", if self.show_axes {"on"} else {"off"}) }
            MenuAction::ViewToggleViewCube => "View Cube toggle (always shown)".to_string(),
            MenuAction::ViewToggleShadows => "Shadows not supported in current renderer".to_string(),
            MenuAction::ViewToggleAo => "Ambient occlusion not supported in current renderer".to_string(),
            MenuAction::ViewToggleAa => "Anti-aliasing is always on".to_string(),
            MenuAction::ViewToggleEdges => { self.show_edges = !self.show_edges; format!("Edges: {}", if self.show_edges {"on"} else {"off"}) }
            MenuAction::ViewToggleNormals => "Normals display not yet implemented".to_string(),
            MenuAction::ViewToggleSilhouette => "Silhouette not yet implemented".to_string(),
            MenuAction::ViewSectionCut => {
                self.brepcad_section_enabled = !self.brepcad_section_enabled;
                if self.brepcad_section_enabled {
                    let (bbox_min, bbox_max) = self.mesh.bounding_box();
                    self.brepcad_section_position = match self.brepcad_section_axis {
                        0 => ((bbox_min.x + bbox_max.x) * 0.5) as f32,
                        1 => ((bbox_min.y + bbox_max.y) * 0.5) as f32,
                        _ => ((bbox_min.z + bbox_max.z) * 0.5) as f32,
                    };
                    self.mesh_dirty = true;
                    "Section cut enabled".to_string()
                } else {
                    self.mesh_dirty = true;
                    "Section cut disabled".to_string()
                }
            }
            MenuAction::ViewTimeline => {
                self.brepcad_timeline_open = !self.brepcad_timeline_open;
                if self.brepcad_timeline_open { "Timeline opened".to_string() } else { String::new() }
            }
            MenuAction::ViewPerspective | MenuAction::ViewOrthographic => "Camera mode toggle not yet implemented".to_string(),
            MenuAction::ViewSaveLayout | MenuAction::ViewLoadLayout => "Layout save/load not yet implemented".to_string(),

            // ── Insert actions ──
            MenuAction::InsertBox => { self.brepcad_dialog = crate::ui::dialogs::DialogType::InsertPrimitive(crate::ui::dialogs::PrimitiveType::Box); String::new() }
            MenuAction::InsertSphere => { self.brepcad_dialog = crate::ui::dialogs::DialogType::InsertPrimitive(crate::ui::dialogs::PrimitiveType::Sphere); String::new() }
            MenuAction::InsertCylinder => { self.brepcad_dialog = crate::ui::dialogs::DialogType::InsertPrimitive(crate::ui::dialogs::PrimitiveType::Cylinder); String::new() }
            MenuAction::InsertCone => { self.brepcad_dialog = crate::ui::dialogs::DialogType::InsertPrimitive(crate::ui::dialogs::PrimitiveType::Cone); String::new() }
            MenuAction::InsertTorus => { self.brepcad_dialog = crate::ui::dialogs::DialogType::InsertPrimitive(crate::ui::dialogs::PrimitiveType::Torus); String::new() }
            MenuAction::InsertPlane | MenuAction::InsertAxis | MenuAction::InsertPoint
            | MenuAction::InsertCs => "Reference geometry not yet implemented".to_string(),
            MenuAction::InsertSketch => "Press S to enter sketch mode (coming soon)".to_string(),
            MenuAction::InsertMesh | MenuAction::InsertMeshFromSolid | MenuAction::InsertRemesh => {
                "Mesh operations not yet implemented".to_string()
            }
            MenuAction::InsertComponent => "Component insertion not yet implemented".to_string(),
            MenuAction::InsertLinearPattern => "Linear pattern not yet implemented".to_string(),
            MenuAction::InsertCircularPattern => { self.model_circular_pattern(); "Circular pattern applied".to_string() }
            MenuAction::InsertMirror => "Mirror not yet implemented".to_string(),

            // ── Modify: Boolean operations ──
            MenuAction::ModifyUnion => { self.brepcad_push_undo_named("Boolean Union"); self.model_boolean_union(); "Boolean union applied".to_string() }
            MenuAction::ModifySubtract => { self.brepcad_push_undo_named("Boolean Subtract"); self.model_boolean_subtract(); "Boolean subtract applied".to_string() }
            MenuAction::ModifyIntersect => { self.brepcad_push_undo_named("Boolean Intersect"); self.model_boolean_intersect(); "Boolean intersect applied".to_string() }
            MenuAction::ModifyFillet => { self.brepcad_push_undo_named("Fillet Edge"); self.model_fillet_edge(); "Fillet applied to first manifold edge".to_string() }
            MenuAction::ModifyChamfer => { self.brepcad_push_undo_named("Chamfer Edge"); self.model_chamfer_edge(); "Chamfer applied to first manifold edge".to_string() }
            MenuAction::ModifyLoft | MenuAction::ModifySweep => {
                "Loft/Sweep: requires sketch profiles (not yet implemented)".to_string()
            }
            MenuAction::ModifyMove => { self.brepcad_push_undo_named("Move +20 X"); self.model_translate(); "Moved +20mm in X".to_string() }
            MenuAction::ModifyRotate => { self.brepcad_push_undo_named("Rotate 15° Z"); self.model_rotate(); "Rotated 15° about Z".to_string() }
            MenuAction::ModifyScale => { self.brepcad_push_undo_named("Scale ×1.1"); self.model_scale(); "Scaled ×1.1".to_string() }
            MenuAction::ModifyLinearPattern => "Linear pattern not yet implemented".to_string(),
            MenuAction::ModifyCircularPattern => { self.model_circular_pattern(); "Circular pattern applied".to_string() }
            MenuAction::ModifyMirror => "Mirror not yet implemented".to_string(),
            MenuAction::ModifyMoveFace => { self.model_delete_face(); "Move/Delete face applied".to_string() }
            MenuAction::ModifyOffsetFace => "Offset face not yet implemented".to_string(),
            MenuAction::ModifyDeleteFace => { self.model_delete_face(); "Face deleted".to_string() }
            MenuAction::ModifyReplaceFace | MenuAction::ModifySplitFace
            | MenuAction::ModifyMergeFaces | MenuAction::ModifySimplify
            | MenuAction::ModifyThicken => "Direct modeling not yet implemented".to_string(),
            MenuAction::ModifyBend | MenuAction::ModifyTwist | MenuAction::ModifyTaper
            | MenuAction::ModifyStretch => "Deform operations not yet implemented".to_string(),

            // ── Sketch actions ──
            MenuAction::SketchEnter | MenuAction::SketchLine | MenuAction::SketchCircle
            | MenuAction::SketchArc3 | MenuAction::SketchArcTangent | MenuAction::SketchRectangle
            | MenuAction::SketchSpline | MenuAction::SketchPolygon | MenuAction::SketchPoint
            | MenuAction::SketchExit | MenuAction::SketchConstraintCoincident
            | MenuAction::SketchConstraintCollinear | MenuAction::SketchConstraintConcentric
            | MenuAction::SketchConstraintParallel | MenuAction::SketchConstraintPerpendicular
            | MenuAction::SketchConstraintTangent | MenuAction::SketchConstraintHorizontal
            | MenuAction::SketchConstraintVertical | MenuAction::SketchConstraintEqual
            | MenuAction::SketchDimLinear | MenuAction::SketchDimAngular
            | MenuAction::SketchDimRadial | MenuAction::SketchDimDiameter
            | MenuAction::SketchTrim | MenuAction::SketchExtend | MenuAction::SketchSplit
            | MenuAction::SketchOffset | MenuAction::SketchMirror | MenuAction::SketchPattern
            | MenuAction::SketchFillet => "Sketch mode coming soon (use existing NURBS/Curve tests)".to_string(),

            // ── Measure actions ──
            MenuAction::MeasureDistance => {
                self.brepcad_measure_mode = BrepcadMeasureMode::Distance;
                self.brepcad_measure_point1 = None;
                self.brepcad_measure_point2 = None;
                self.brepcad_measure_result = String::new();
                self.brepcad_active_tool = "Measure Distance".to_string();
                "Measure Distance: click 2 points in viewport".to_string()
            }
            MenuAction::MeasureAngle => {
                self.brepcad_measure_mode = BrepcadMeasureMode::Angle;
                self.brepcad_measure_point1 = None;
                self.brepcad_measure_point2 = None;
                self.brepcad_measure_point3 = None;
                self.brepcad_measure_result = String::new();
                self.brepcad_active_tool = "Measure Angle".to_string();
                "Measure Angle: click 3 points (vertex, p1, p2)".to_string()
            }
            MenuAction::MeasureLength => {
                self.brepcad_measure_mode = BrepcadMeasureMode::Length;
                self.brepcad_measure_point1 = None;
                self.brepcad_measure_point2 = None;
                self.brepcad_measure_result = String::new();
                self.brepcad_active_tool = "Measure Length".to_string();
                "Measure Length: click 2 points".to_string()
            }
            MenuAction::MeasureArea => {
                let mut area = 0.0_f64;
                for tri in &self.mesh.triangles {
                    let v0 = self.mesh.vertices[tri[0] as usize];
                    let v1 = self.mesh.vertices[tri[1] as usize];
                    let v2 = self.mesh.vertices[tri[2] as usize];
                    let ax = v1.x - v0.x; let ay = v1.y - v0.y; let az = v1.z - v0.z;
                    let bx = v2.x - v0.x; let by = v2.y - v0.y; let bz = v2.z - v0.z;
                    let cx = ay * bz - az * by;
                    let cy = az * bx - ax * bz;
                    let cz = ax * by - ay * bx;
                    area += 0.5 * (cx * cx + cy * cy + cz * cz).sqrt();
                }
                format!("Surface area: {:.3} mm²", area)
            }
            MenuAction::MeasureVolume => {
                let mut vol = 0.0_f64;
                for tri in &self.mesh.triangles {
                    let v0 = self.mesh.vertices[tri[0] as usize];
                    let v1 = self.mesh.vertices[tri[1] as usize];
                    let v2 = self.mesh.vertices[tri[2] as usize];
                    vol += (v0.x * (v1.y * v2.z - v1.z * v2.y)
                          + v0.y * (v1.z * v2.x - v1.x * v2.z)
                          + v0.z * (v1.x * v2.y - v1.y * v2.x)) / 6.0;
                }
                format!("Volume: {:.3} mm³", vol.abs())
            }

            // ── Parametric actions ──
            MenuAction::ParamParameters => {
                self.brepcad_param_dialog_open = !self.brepcad_param_dialog_open;
                if self.brepcad_param_dialog_open { "Parameters dialog opened".to_string() } else { String::new() }
            }
            MenuAction::ParamEquations => "Use Parameters dialog to edit formulas".to_string(),
            MenuAction::ParamDesignTable | MenuAction::ParamDependencyGraph
            | MenuAction::ParamVariants => "Parametric feature not yet implemented".to_string(),

            // ── Sheet Metal, Assembly, CAM, Drawing, Simulation, etc. ──
            _ => format!("{:?} — not yet implemented", action),
        }
    }

    /// Handle a BRepCAD dialog action.
    pub fn handle_brepcad_dialog_action(&mut self, action: &crate::ui::dialogs::DialogAction) -> String {
        use crate::ui::dialogs::{DialogAction, PrimitiveType};
        match action {
            DialogAction::InsertPrimitive(pt, values) => {
                self.brepcad_push_undo_named(&format!("Insert {}", pt.label()));
                let params = pt.params();
                let v: Vec<f64> = (0..params.len())
                    .map(|i| values.get(i).copied().unwrap_or(params[i].1))
                    .collect();

                let solid = match pt {
                    PrimitiveType::Box => {
                        let (w, h, d) = (v[0], v[1], v[2]);
                        let s = ShapeBuilder::make_box(w, h, d);
                        let m = triangulate_solid(&s, &tri_params_for_lod(self.lod_level));
                        self.load_mesh(m, &format!("Box {:.0}x{:.0}x{:.0}", w, h, d));
                        s
                    }
                    PrimitiveType::Sphere => {
                        let r = v[0];
                        let s = ShapeBuilder::make_sphere(r);
                        let m = triangulate_solid(&s, &tri_params_for_lod(self.lod_level));
                        self.load_mesh(m, &format!("Sphere R={:.0}", r));
                        s
                    }
                    PrimitiveType::Cylinder => {
                        let (r, h) = (v[0], v[1]);
                        let s = ShapeBuilder::make_cylinder(r, h);
                        let m = triangulate_solid(&s, &tri_params_for_lod(self.lod_level));
                        self.load_mesh(m, &format!("Cylinder R={:.0} H={:.0}", r, h));
                        s
                    }
                    PrimitiveType::Cone => {
                        let (br, _tr, h) = (v[0], v[1], v[2]);
                        let half_angle = (br / h).atan();
                        let s = ShapeBuilder::make_cone(br, h, half_angle);
                        let m = triangulate_solid(&s, &tri_params_for_lod(self.lod_level));
                        self.load_mesh(m, &format!("Cone R={:.0} H={:.0}", br, h));
                        s
                    }
                    PrimitiveType::Torus => {
                        let (mr, nr) = (v[0], v[1]);
                        let s = ShapeBuilder::make_torus(mr, nr);
                        let m = triangulate_solid(&s, &tri_params_for_lod(self.lod_level));
                        self.load_mesh(m, &format!("Torus R={:.0} r={:.0}", mr, nr));
                        s
                    }
                };
                self.current_solid = Some(solid);
                self.current_nurbs_surface = None;
                self.detailed_instances.clear();
                self.instance_triangle_ranges.clear();
                self.assembly_tree = None;
                format!("{} inserted", pt.label())
            }
            DialogAction::Close => "Dialog closed".to_string(),
        }
    }

    /// Map a command palette name to a MenuAction.
    pub fn command_name_to_action(name: &str) -> Option<crate::ui::menubar::MenuAction> {
        use crate::ui::menubar::MenuAction;
        match name {
            "New" => Some(MenuAction::FileNew),
            "Open…" => Some(MenuAction::FileOpen),
            "Save" => Some(MenuAction::FileSave),
            "Export STEP" => Some(MenuAction::FileExportStep),
            "Export STL" => Some(MenuAction::FileExportStl),
            "Import STEP" => Some(MenuAction::FileImportStep),
            "Undo" => Some(MenuAction::EditUndo),
            "Redo" => Some(MenuAction::EditRedo),
            "Cut" => Some(MenuAction::EditCut),
            "Copy" => Some(MenuAction::EditCopy),
            "Paste" => Some(MenuAction::EditPaste),
            "Duplicate" => Some(MenuAction::EditDuplicate),
            "Fit to View" => Some(MenuAction::ViewFit),
            "ISO View" => Some(MenuAction::ViewIso),
            "Front View" => Some(MenuAction::ViewFront),
            "Top View" => Some(MenuAction::ViewTop),
            "Right View" => Some(MenuAction::ViewRight),
            "Wireframe" => Some(MenuAction::ViewWireframe),
            "Shaded" => Some(MenuAction::ViewShaded),
            "Shaded + Edges" => Some(MenuAction::ViewShadedEdges),
            "Insert Box" => Some(MenuAction::InsertBox),
            "Insert Sphere" => Some(MenuAction::InsertSphere),
            "Insert Cylinder" => Some(MenuAction::InsertCylinder),
            "Insert Cone" => Some(MenuAction::InsertCone),
            "Insert Torus" => Some(MenuAction::InsertTorus),
            "Insert Sketch" => Some(MenuAction::InsertSketch),
            "Boolean Union" => Some(MenuAction::ModifyUnion),
            "Boolean Subtract" => Some(MenuAction::ModifySubtract),
            "Boolean Intersect" => Some(MenuAction::ModifyIntersect),
            "Fillet" => Some(MenuAction::ModifyFillet),
            "Chamfer" => Some(MenuAction::ModifyChamfer),
            "Move" => Some(MenuAction::ModifyMove),
            "Rotate" => Some(MenuAction::ModifyRotate),
            "Scale" => Some(MenuAction::ModifyScale),
            "Sketch Mode" => Some(MenuAction::SketchEnter),
            "Options…" => Some(MenuAction::ToolsOptions),
            "Customize…" => Some(MenuAction::ToolsCustomize),
            "Plugins Manager…" => Some(MenuAction::ToolsPlugins),
            "Scripting Console" => Some(MenuAction::ToolsScriptingConsole),
            "Performance Monitor" => Some(MenuAction::ToolsPerformance),
            "Measure Distance" => Some(MenuAction::MeasureDistance),
            "Measure Angle" => Some(MenuAction::MeasureAngle),
            "Measure Area" => Some(MenuAction::MeasureArea),
            "Measure Volume" => Some(MenuAction::MeasureVolume),
            "Heal: Stitch" => Some(MenuAction::HealStitch),
            "Heal: Gap Fill" => Some(MenuAction::HealGapFill),
            "Heal: Fix Orientation" => Some(MenuAction::HealFixOrientation),
            "Watertight Check" => Some(MenuAction::AnalysisWatertight),
            "Manifold Check" => Some(MenuAction::AnalysisManifold),
            _ => None,
        }
    }

    /// Push current state to undo stack (call BEFORE a mutation).
    /// Truncates redo stack — branching point.
    /// Also records a timeline entry with the operation name.
    pub fn brepcad_push_undo(&mut self) {
        self.brepcad_push_undo_named("Operation");
    }

    /// Push current state to undo stack with a named operation.
    pub fn brepcad_push_undo_named(&mut self, op_name: &str) {
        self.brepcad_redo_stack.clear();
        let snapshot = (self.current_solid.clone(), self.current_model.name.clone());
        self.brepcad_undo_stack.push(snapshot);
        if self.brepcad_undo_stack.len() > self.brepcad_max_history {
            self.brepcad_undo_stack.remove(0);
        }
        // Add to timeline
        self.brepcad_timeline.push((op_name.to_string(), self.current_solid.clone()));
        if self.brepcad_timeline.len() > self.brepcad_max_history {
            self.brepcad_timeline.remove(0);
        }
        self.brepcad_timeline_rollback = None; // latest
    }

    /// Rollback to a specific point in the timeline.
    pub fn brepcad_timeline_rollback_to(&mut self, idx: usize) -> bool {
        if idx >= self.brepcad_timeline.len() {
            return false;
        }
        let (_, solid) = &self.brepcad_timeline[idx];
        self.current_solid = solid.clone();
        if let Some(ref s) = self.current_solid {
            self.refresh_from_current_solid("Rollback");
        }
        self.selected_instance = None;
        self.selected_face = None;
        self.highlighted_face = None;
        self.highlight_dirty = true;
        self.mesh_dirty = true;
        self.brepcad_timeline_rollback = Some(idx);
        true
    }

    // ─── Parameter system ──────────────────────────────────────────────

    /// Add or update a parameter with a numeric value.
    pub fn brepcad_set_param(&mut self, name: &str, value: f64) {
        self.brepcad_parameters.insert(name.to_string(), (value, None, "mm".to_string()));
    }

    /// Add or update a parameter with a formula.
    pub fn brepcad_set_param_formula(&mut self, name: &str, formula: &str, unit: &str) {
        let value = Self::brepcad_eval_formula(formula);
        self.brepcad_parameters.insert(name.to_string(), (value, Some(formula.to_string()), unit.to_string()));
    }

    /// Get a parameter value by name.
    pub fn brepcad_get_param(&self, name: &str) -> Option<f64> {
        self.brepcad_parameters.get(name).map(|(v, _, _)| *v)
    }

    /// Remove a parameter.
    pub fn brepcad_remove_param(&mut self, name: &str) -> bool {
        self.brepcad_parameters.remove(name).is_some()
    }

    /// Re-evaluate all formula-based parameters.
    pub fn brepcad_eval_params(&mut self) {
        let params = self.brepcad_parameters.clone();
        let mut updates: Vec<(String, f64)> = Vec::new();
        for (name, (_, formula, _)) in &params {
            if let Some(f) = formula {
                let value = Self::brepcad_eval_formula_with_params(f, &params);
                updates.push((name.clone(), value));
            }
        }
        for (name, value) in updates {
            if let Some(entry) = self.brepcad_parameters.get_mut(&name) {
                entry.0 = value;
            }
        }
    }

    /// Simple formula evaluator (supports: number, name, a+b, a-b, a*b, a/b, a*b+c, parentheses).
    pub fn brepcad_eval_formula(formula: &str) -> f64 {
        Self::brepcad_eval_formula_with_params(formula, &std::collections::HashMap::new())
    }

    /// Evaluate a formula with access to the parameter table.
    /// Supports: numbers, parameter names, +, -, *, /, parentheses.
    fn brepcad_eval_formula_with_params(
        formula: &str,
        params: &std::collections::HashMap<String, (f64, Option<String>, String)>,
    ) -> f64 {
        let formula = formula.trim();
        // Try to parse as a plain number first
        if let Ok(val) = formula.parse::<f64>() {
            return val;
        }

        // Tokenize: split by operators while keeping them
        let tokens = Self::tokenize_formula(formula);
        if tokens.is_empty() {
            return 0.0;
        }

        // Simple recursive descent parser: expr = term (('+'|'-') term)*
        // term = factor (('*'|'/') factor)*
        // factor = number | name | '(' expr ')'
        let mut pos = 0;
        Self::parse_expr(&tokens, &mut pos, params).unwrap_or(0.0)
    }

    /// Tokenize a formula into numbers, identifiers, and operators.
    fn tokenize_formula(formula: &str) -> Vec<String> {
        let mut tokens = Vec::new();
        let mut current = String::new();
        for ch in formula.chars() {
            if ch.is_whitespace() {
                if !current.is_empty() {
                    tokens.push(current.clone());
                    current.clear();
                }
                continue;
            }
            if "+-*/()".contains(ch) {
                if !current.is_empty() {
                    tokens.push(current.clone());
                    current.clear();
                }
                tokens.push(ch.to_string());
            } else {
                current.push(ch);
            }
        }
        if !current.is_empty() {
            tokens.push(current);
        }
        tokens
    }

    /// Parse expression: term (('+'|'-') term)*
    fn parse_expr(
        tokens: &[String],
        pos: &mut usize,
        params: &std::collections::HashMap<String, (f64, Option<String>, String)>,
    ) -> Option<f64> {
        let mut left = Self::parse_term(tokens, pos, params)?;
        while *pos < tokens.len() {
            let op = &tokens[*pos];
            if op == "+" || op == "-" {
                *pos += 1;
                let right = Self::parse_term(tokens, pos, params)?;
                left = if op == "+" { left + right } else { left - right };
            } else {
                break;
            }
        }
        Some(left)
    }

    /// Parse term: factor (('*'|'/') factor)*
    fn parse_term(
        tokens: &[String],
        pos: &mut usize,
        params: &std::collections::HashMap<String, (f64, Option<String>, String)>,
    ) -> Option<f64> {
        let mut left = Self::parse_factor(tokens, pos, params)?;
        while *pos < tokens.len() {
            let op = &tokens[*pos];
            if op == "*" || op == "/" {
                *pos += 1;
                let right = Self::parse_factor(tokens, pos, params)?;
                left = if op == "*" {
                    left * right
                } else {
                    if right.abs() < 1e-12 { 0.0 } else { left / right }
                };
            } else {
                break;
            }
        }
        Some(left)
    }

    /// Parse factor: number | name | '(' expr ')'
    fn parse_factor(
        tokens: &[String],
        pos: &mut usize,
        params: &std::collections::HashMap<String, (f64, Option<String>, String)>,
    ) -> Option<f64> {
        if *pos >= tokens.len() {
            return None;
        }
        let token = &tokens[*pos];
        *pos += 1;

        if token == "(" {
            let val = Self::parse_expr(tokens, pos, params)?;
            // Expect closing paren
            if *pos < tokens.len() && tokens[*pos] == ")" {
                *pos += 1;
            }
            return Some(val);
        }
        if token == ")" {
            return None;
        }

        // Try to parse as number
        if let Ok(val) = token.parse::<f64>() {
            return Some(val);
        }

        // Try to look up as parameter name
        if let Some((val, _, _)) = params.get(token) {
            return Some(*val);
        }

        // Unknown token — return 0
        Some(0.0)
    }

    /// BRepCAD undo: restore previous solid state.
    /// Returns true if undone, false if stack empty.
    pub fn brepcad_undo(&mut self) -> bool {
        if let Some((solid, name)) = self.brepcad_undo_stack.pop() {
            // Save current state to redo stack
            let current = (self.current_solid.clone(), self.current_model.name.clone());
            self.brepcad_redo_stack.push(current);
            // Restore from snapshot
            self.current_solid = solid;
            self.current_model.name = name;
            // Re-triangulate
            if let Some(ref solid) = self.current_solid {
                self.refresh_from_current_solid("Undo");
            }
            self.selected_instance = None;
            self.selected_face = None;
            self.highlighted_face = None;
            self.highlight_dirty = true;
            self.mesh_dirty = true;
            true
        } else {
            false
        }
    }

    /// BRepCAD redo: re-apply undone state.
    pub fn brepcad_redo(&mut self) -> bool {
        if let Some((solid, name)) = self.brepcad_redo_stack.pop() {
            let current = (self.current_solid.clone(), self.current_model.name.clone());
            self.brepcad_undo_stack.push(current);
            self.current_solid = solid;
            self.current_model.name = name;
            if let Some(ref solid) = self.current_solid {
                self.refresh_from_current_solid("Redo");
            }
            self.selected_instance = None;
            self.selected_face = None;
            self.highlighted_face = None;
            self.highlight_dirty = true;
            self.mesh_dirty = true;
            true
        } else {
            false
        }
    }

    // ─── BRepCAD panel rendering methods ────────────────────────────────────

    /// Render the Layers tab.
    fn render_brepcad_layers(&self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Layers").size(12.0).color(egui::Color32::from_rgb(0xcd, 0xd6, 0xf4)));
        ui.separator();

        let layers = [
            ("0", "Default", true, [0xff, 0xff, 0xff]),
            ("1", "Construction", true, [0xa6, 0xe3, 0xa1]),
            ("2", "Dimensions", true, [0x89, 0xb4, 0xfa]),
            ("3", "Annotations", true, [0xf9, 0xe2, 0xaf]),
            ("4", "Hidden Lines", false, [0x6c, 0x70, 0x86]),
        ];

        for (id, name, visible, color) in &layers {
            ui.horizontal(|ui| {
                let mut vis = *visible;
                ui.checkbox(&mut vis, "");
                ui.color_edit_button_srgb(&mut color.clone());
                ui.label(egui::RichText::new(format!("{}: {}", id, name)).size(11.0));
            });
        }

        ui.add_space(8.0);
        ui.separator();
        if ui.button("+ New Layer").clicked() {}
    }
}

// ============================================================
// BRepCAD Catppuccin Mocha dark theme
// ============================================================

/// Apply Catppuccin Mocha dark theme to match the UI mockups in docs/ui_mockups/.
/// Colors from docs/ui_mockups/01_main_window.svg:
///   #1e1e2e base, #181825 mantle, #11111b crust, #313244 surface0,
///   #45475a surface1, #cdd6f4 text, #a6adc8 subtext, #89b4fa blue accent
pub fn apply_brepcad_theme(ctx: &egui::Context) {
    use egui::{Color32, Vec2, Rounding, Stroke};

    let mut style: egui::Style = (*ctx.style()).clone();
    let mut visuals = egui::Visuals::dark();

    visuals.panel_fill       = Color32::from_rgb(0x1e, 0x1e, 0x2e);
    visuals.faint_bg_color   = Color32::from_rgb(0x18, 0x18, 0x25);
    visuals.extreme_bg_color = Color32::from_rgb(0x11, 0x11, 0x1b);
    visuals.window_fill      = Color32::from_rgb(0x1e, 0x1e, 0x2e);
    visuals.window_stroke    = Stroke::new(1.0, Color32::from_rgb(0x45, 0x47, 0x5a));
    visuals.override_text_color = Some(Color32::from_rgb(0xcd, 0xd6, 0xf4));

    let blue = Color32::from_rgb(0x89, 0xb4, 0xfa);
    visuals.widgets.noninteractive.bg_fill   = Color32::from_rgb(0x18, 0x18, 0x25);
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x45, 0x47, 0x5a));
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0, Color32::from_rgb(0xa6, 0xad, 0xc8));
    visuals.widgets.inactive.bg_fill   = Color32::from_rgb(0x31, 0x32, 0x44);
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x45, 0x47, 0x5a));
    visuals.widgets.inactive.fg_stroke = Stroke::new(1.0, Color32::from_rgb(0xcd, 0xd6, 0xf4));
    visuals.widgets.hovered.bg_fill   = Color32::from_rgb(0x45, 0x47, 0x5a);
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, blue);
    visuals.widgets.hovered.fg_stroke = Stroke::new(1.0, Color32::from_rgb(0xcd, 0xd6, 0xf4));
    visuals.widgets.active.bg_fill   = Color32::from_rgb(0x58, 0x5b, 0x70);
    visuals.widgets.active.bg_stroke = Stroke::new(1.0, blue);
    visuals.widgets.active.fg_stroke = Stroke::new(1.5, Color32::from_rgb(0xff, 0xff, 0xff));
    visuals.widgets.open.bg_fill = Color32::from_rgb(0x45, 0x47, 0x5a);
    visuals.selection.bg_fill = Color32::from_rgb(0x09, 0x47, 0x71);
    visuals.selection.stroke  = Stroke::new(1.0, blue);
    visuals.hyperlink_color   = blue;

    style.visuals = visuals;
    style.spacing.item_spacing = Vec2::new(6.0, 4.0);
    style.spacing.button_padding = Vec2::new(8.0, 4.0);
    style.visuals.widgets.noninteractive.corner_radius = Rounding::same(4);
    style.visuals.widgets.inactive.corner_radius = Rounding::same(4);
    style.visuals.widgets.hovered.corner_radius = Rounding::same(4);
    style.visuals.widgets.active.corner_radius = Rounding::same(4);
    style.visuals.window_shadow = egui::epaint::Shadow::NONE;

    ctx.set_style(style);
}

// ============================================================
// BRepCAD free functions (for tree rendering with pending actions)
// ============================================================

/// Recursively render an assembly tree node for the BRepCAD Browser panel.
/// Sets pending actions on click — applied by the caller after the panel.
fn brepcad_render_assembly_node(
    ui: &mut egui::Ui,
    node: &AssemblyNode,
    depth: usize,
    filter: &str,
    filter_active: bool,
    selected_instance: Option<usize>,
    selected_face: Option<(usize, u64)>,
    hidden_instances: &std::collections::HashSet<usize>,
    hidden_faces: &std::collections::HashSet<(usize, u64)>,
    detailed_instances: &[DetailedMeshInstance],
    pending_instance_select: &mut Option<usize>,
    pending_visibility_toggle: &mut Option<usize>,
    pending_instance_isolate: &mut Option<usize>,
    pending_face_select: &mut Option<(usize, u64)>,
    pending_face_visibility_toggle: &mut Option<(usize, u64)>,
) {
    // Filter: skip if node doesn't match and no children match
    if filter_active && !node.name.to_lowercase().contains(filter) {
        let has_matching_child = node.children.iter().any(|c| brepcad_node_matches_filter(c, filter));
        if !has_matching_child {
            return;
        }
    }

    let is_leaf = node.instance_index.is_some();
    let is_selected = node.instance_index.is_some() && selected_instance == node.instance_index;
    let is_visible = node.instance_index.map(|i| !hidden_instances.contains(&i)).unwrap_or(true);

    ui.horizontal(|ui| {
        ui.add_space((depth * 14) as f32);

        // Expand/collapse arrow (non-leaf nodes)
        if !node.children.is_empty() {
            ui.label(egui::RichText::new("▼").size(9.0).color(egui::Color32::from_rgb(0xa6, 0xad, 0xc8)));
        } else {
            ui.add_space(9.0);
        }

        // For leaf nodes with instance_index: eye icon + isolate button
        if is_leaf {
            let idx = node.instance_index.unwrap();
            let eye_color = if is_visible {
                egui::Color32::from_rgb(0xa6, 0xe3, 0xa1) // green
            } else {
                egui::Color32::from_rgb(0xf3, 0x8b, 0xa8) // red
            };
            let eye_text = if is_visible { "👁" } else { "🚫" };
            let eye_resp = ui.add(egui::Label::new(
                egui::RichText::new(eye_text).size(11.0).color(eye_color)
            ).sense(egui::Sense::click()));
            if eye_resp.clicked() {
                *pending_visibility_toggle = Some(idx);
            }
            eye_resp.on_hover_text(if is_visible { "Click to hide instance" } else { "Click to show instance" });

            // Isolate button
            let iso_resp = ui.add(egui::Button::new(
                egui::RichText::new("◎").size(10.0)
            ).frame(false));
            if iso_resp.clicked() {
                *pending_instance_isolate = Some(idx);
            }
            iso_resp.on_hover_text("Isolate: hide all other instances");
        } else {
            ui.add_space(28.0); // align with leaf nodes
        }

        // Node icon + name
        let icon = if is_leaf { "📦" } else if node.brep_id.is_some() { "🔧" } else { "📁" };
        let bg = if is_selected {
            egui::Color32::from_rgb(0x09, 0x47, 0x71)
        } else {
            egui::Color32::TRANSPARENT
        };
        let frame = egui::Frame::new().fill(bg).inner_margin(egui::Margin::symmetric(2, 1));
        frame.show(ui, |ui| {
            let label_text = format!("{} {}", icon, node.name);
            let resp = ui.selectable_label(is_selected,
                egui::RichText::new(label_text).size(11.0));
            if resp.clicked() && is_leaf {
                *pending_instance_select = node.instance_index;
            }
        });
    });

    // Render children
    for child in &node.children {
        brepcad_render_assembly_node(
            ui, child, depth + 1, filter, filter_active,
            selected_instance, selected_face, hidden_instances, hidden_faces, detailed_instances,
            pending_instance_select, pending_visibility_toggle, pending_instance_isolate,
            pending_face_select, pending_face_visibility_toggle,
        );
    }

    // ─── For leaf nodes: render face list under the instance ───
    if is_leaf && is_visible {
        let inst_idx = node.instance_index.unwrap();
        if let Some(inst) = detailed_instances.get(inst_idx) {
            // Only show faces if the instance is selected (to avoid huge trees)
            if is_selected && !inst.faces.is_empty() {
                for face_info in &inst.faces {
                    let fid = face_info.face_id;
                    let face_is_selected = selected_face == Some((inst_idx, fid));
                    let face_is_visible = !hidden_faces.contains(&(inst_idx, fid));
                    let tri_count_face = face_info.triangle_range.1.saturating_sub(face_info.triangle_range.0);

                    ui.horizontal(|ui| {
                        ui.add_space(((depth + 1) * 14) as f32);
                        ui.add_space(9.0); // align with arrow

                        // Face eye icon
                        let face_eye_color = if face_is_visible {
                            egui::Color32::from_rgb(0xa6, 0xe3, 0xa1) // green
                        } else {
                            egui::Color32::from_rgb(0xf3, 0x8b, 0xa8) // red
                        };
                        let face_eye_text = if face_is_visible { "👁" } else { "🚫" };
                        let face_eye_resp = ui.add(egui::Label::new(
                            egui::RichText::new(face_eye_text).size(10.0).color(face_eye_color)
                        ).sense(egui::Sense::click()));
                        if face_eye_resp.clicked() {
                            *pending_face_visibility_toggle = Some((inst_idx, fid));
                        }
                        face_eye_resp.on_hover_text(if face_is_visible { "Click to hide face" } else { "Click to show face" });

                        ui.add_space(18.0); // align with isolate button

                        // Face icon + label
                        let face_bg = if face_is_selected {
                            egui::Color32::from_rgb(0x09, 0x47, 0x71)
                        } else {
                            egui::Color32::TRANSPARENT
                        };
                        let face_frame = egui::Frame::new().fill(face_bg)
                            .inner_margin(egui::Margin::symmetric(2, 1));
                        face_frame.show(ui, |ui| {
                            let label = format!("🔷 Face #{} ({}, {} tris)",
                                fid, face_info.surface_type, tri_count_face);
                            let resp = ui.selectable_label(face_is_selected,
                                egui::RichText::new(label).size(10.0));
                            if resp.clicked() {
                                *pending_face_select = Some((inst_idx, fid));
                            }
                        });
                    });
                }
            }
        }
    }
}

/// Check if a node or any descendant matches the filter.
fn brepcad_node_matches_filter(node: &AssemblyNode, filter: &str) -> bool {
    if node.name.to_lowercase().contains(filter) {
        return true;
    }
    node.children.iter().any(|c| brepcad_node_matches_filter(c, filter))
}
