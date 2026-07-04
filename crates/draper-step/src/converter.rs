// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP-to-mesh converter.
//!
//! Converts parsed STEP entities into a triangle mesh by:
//! 1. Resolving entity references to build geometry
//! 2. Creating B-Rep faces from STEP surface entities with boundary edges
//! 3. Triangulating using the existing mesh pipeline (ear-clipping for planar faces)
//!
//! Supported surface types:
//! - PLANE
//! - CYLINDRICAL_SURFACE
//! - SPHERICAL_SURFACE
//! - CONICAL_SURFACE
//! - TOROIDAL_SURFACE
//! - SURFACE_OF_REVOLUTION
//! - SURFACE_OF_LINEAR_EXTRUSION
//! - B_SPLINE_SURFACE_WITH_KNOTS / B_SPLINE_SURFACE / BEZIER_SURFACE
//!
//! Boundary extraction:
//! - ADVANCED_FACE → FACE_BOUND → EDGE_LOOP → ORIENTED_EDGE → EDGE_CURVE
//! - EDGE_CURVE → SURFACE_CURVE → 3D curve + vertex endpoints
//! - Boundary edges enable proper ear-clipping triangulation of planar faces

#![allow(dead_code)]
use crate::schema::{StepFile, StepValue};
use draper_geometry::{
    Point3d, Point2d, Direction3d, Vec3d, Surface, Plane, CylinderSurface, SphereSurface,
    ConeSurface, TorusSurface, RevolutionSurface, ExtrusionSurface,
    NurbsSurface, Curve3d, Curve2d, Line, Circle,  Arc, NurbsCurve,
    Line2d, Circle2d, Ellipse2d, Hyperbola2d, Parabola2d, Nurbs2d,
};
use draper_mesh::{TriangleMesh, TriangulationParams, triangulate_face, triangulate_face_with_boundary_and_holes_uv, ear_clip, validate_watertight, validate_edge_consistency, filter_degenerate_triangles, weld_boundary_edge_vertices};
use draper_topology::{Face, Wire, CoEdge, Edge as TopoEdge, Shell, Solid};
use draper_topology::healing::{heal_solid, HealingParams, HealingReport};
use draper_topology::validator::validate_brep;
use draper_topology::validation::TopologyValidationConfig;
use draper_geometry::tolerance::ToleranceContext;
use draper_mesh::edge_cache::{EdgeDiscretizationCache, deterministic_round_point};
use std::collections::HashMap;

// WASM-compatible Instant: on native uses std::time::Instant,
// on wasm32 uses web_time::Instant (backed by performance.now()).
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant as StdInstant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant as StdInstant;
use log::{info, warn};

// ============================================================
// STEP Conversion Configuration
// ============================================================

/// Configuration for STEP-to-mesh conversion.
///
/// Controls optional pipeline stages like healing.
#[derive(Clone, Debug)]
pub struct StepConversionConfig {
    /// Whether to apply the healing pipeline between BRep extraction and
    /// triangulation. Healing fixes common B-Rep defects such as gaps,
    /// holes, flipped normals, degenerate edges, and small features.
    ///
    /// Default: `false` — healing is DISABLED because the heal_solid pipeline
    /// was dropping valid NURBS faces (e.g., cylindrical walls of bolt holes
    /// represented as NURBS surfaces), producing non-watertight meshes with
    /// missing faces. The edge cache + bit-exact dedup already produces
    /// watertight meshes without healing.
    pub heal: bool,
}

impl Default for StepConversionConfig {
    fn default() -> Self {
        Self { heal: false }
    }
}

impl StepConversionConfig {
    /// Create a config with healing disabled.
    pub fn no_healing() -> Self {
        Self { heal: false }
    }
}

// ============================================================
// FaceData — extracted face data with surface and boundary edges
// ============================================================

/// Extracted face data with surface and boundary edges.
#[derive(Clone)]
struct FaceData {
    surface: Surface,
    /// Edges from the outer boundary loop (FACE_OUTER_BOUND)
    outer_edges: Vec<TopoEdge>,
    /// Edges from inner boundary loops (FACE_BOUND = holes)
    inner_edges: Vec<Vec<TopoEdge>>,
    /// All edges combined (for backward compat with surface_to_mesh)
    edges: Vec<TopoEdge>,
    forward: bool,
    /// STEP entity ID of the ADVANCED_FACE this data was extracted from.
    step_face_id: i64,
    /// STEP entity ID of the face's surface (for PCURVE matching).
    surface_step_id: Option<i64>,
    /// Analytical PCURVEs in UV space for each edge in `edges`.
    /// Same length as `edges`. When present, used instead of surface.project_point().
    edge_curves_2d: Vec<Option<Curve2d>>,
    /// STEP entity IDs of EDGE_CURVE entities for each edge in `edges`.
    /// Same length as `edges`. Used for edge discretization caching —
    /// when two faces share the same STEP EDGE_CURVE, they must produce
    /// identical 3D boundary points to ensure mesh watertightness.
    edge_step_ids: Vec<i64>,
    /// STEP entity IDs of EDGE_CURVE entities for outer edges.
    /// Same length as `outer_edges`.
    outer_edge_step_ids: Vec<i64>,
    /// STEP entity IDs of EDGE_CURVE entities for inner edges.
    /// Same structure as `inner_edges`.
    inner_edge_step_ids: Vec<Vec<i64>>,
    /// Whether this face belongs to a void shell (internal cavity).
    /// Void faces have normals pointing INTO the solid material (away from
    /// the void cavity). This flag is used by the healing pipeline to
    /// avoid incorrectly flipping void face normals.
    is_void: bool,
}

// ============================================================
// FaceData ↔ Solid conversion (for healing pipeline)
// ============================================================

/// Build a `Solid` from a list of `FaceData` so that the healing
/// pipeline can operate on the topology.
///
/// Returns the Solid and a mapping from each Face's `TopoId` to its
/// index in the original `face_data_list`. This mapping allows us to
/// recover STEP-specific metadata after healing.
///
/// Faces with `is_void = true` are placed into separate inner shells
/// (one per contiguous group, or all in one inner shell for simplicity).
/// This ensures that `fix_normal_orientation` in the healing pipeline
/// processes void shells independently and does not corrupt their normals.
fn face_data_list_to_solid(face_data_list: &[FaceData]) -> (Solid, HashMap<draper_topology::TopoId, usize>) {
    let mut outer_topo_faces = Vec::new();
    let mut void_topo_faces = Vec::new();
    let mut face_id_to_index = HashMap::new();

    for (fd_idx, fd) in face_data_list.iter().enumerate() {
        // Build outer wire from outer edges — attach curve_2d (PCURVE) data
        // so that triangulate_face can use it for accurate UV coordinates.
        let outer_wire = if fd.outer_edges.is_empty() {
            None
        } else {
            let coedges: Vec<CoEdge> = fd.outer_edges.iter()
                .map(|edge| {
                    // Try to find the curve_2d for this edge
                    let curve_2d = fd.edges.iter().position(|e| e.id == edge.id)
                        .and_then(|idx| fd.edge_curves_2d.get(idx).and_then(|c| c.clone()));
                    let mut coedge = CoEdge::new(edge.id, true);
                    coedge.curve_2d = curve_2d;
                    coedge
                })
                .collect();
            Some(Wire::new(coedges))
        };

        // Build inner wires from inner edges (holes) — also with curve_2d
        let inner_wires: Vec<Wire> = fd.inner_edges.iter()
            .map(|inner_edge_list| {
                let coedges: Vec<CoEdge> = inner_edge_list.iter()
                    .map(|edge| {
                        let curve_2d = fd.edges.iter().position(|e| e.id == edge.id)
                            .and_then(|idx| fd.edge_curves_2d.get(idx).and_then(|c| c.clone()));
                        let mut coedge = CoEdge::new(edge.id, true);
                        coedge.curve_2d = curve_2d;
                        coedge
                    })
                    .collect();
                Wire::new(coedges)
            })
            .collect();

        let mut face = Face::new(fd.surface.clone(), outer_wire.unwrap_or_else(|| Wire::new(vec![])));
        face.forward = fd.forward;
        face.edges = fd.edges.clone();

        // Record the mapping before we add holes (which doesn't change the ID)
        face_id_to_index.insert(face.id, fd_idx);

        // Add inner wires as holes
        for wire in inner_wires {
            face.add_hole(wire);
        }

        if fd.is_void {
            void_topo_faces.push(face);
        } else {
            outer_topo_faces.push(face);
        }
    }

    let outer_shell = Shell::new_closed(outer_topo_faces);
    let mut solid = Solid::new(outer_shell);

    // Add void faces as a single inner shell (all voids together).
    // In practice, BREP_WITH_VOIDS typically has one void shell, but
    // we merge all void faces into one inner shell for simplicity.
    // The healing pipeline will process inner shells independently,
    // so their normals won't be corrupted by the centroid-based
    // orientation fix that works for outer shells.
    if !void_topo_faces.is_empty() {
        let void_shell = Shell::new_closed(void_topo_faces);
        solid.add_void(void_shell);
    }

    (solid, face_id_to_index)
}

/// Apply healing results back to the original `FaceData` list.
///
/// This function takes the original `FaceData` list, the healed `Solid`,
/// and a mapping from face TopoId → original FaceData index. It produces
/// a new `FaceData` list that:
///
/// - **Preserves** STEP-specific metadata (step_face_id, edge_step_ids,
///   edge_curves_2d, etc.) for faces that originated from the STEP file.
///   The original edge data and STEP IDs are kept intact because the
///   EdgeDiscretizationCache already handles watertightness at the mesh level.
/// - **Updates** the `forward` flag if the healing pipeline flipped a face
///   normal.
/// - **Removes** faces that were removed by the healing pipeline (small
///   feature removal).
/// - **Adds** new faces that were created by the healing pipeline (hole
///   filling), with default (0) STEP IDs.
fn apply_healing_to_face_data(
    original_face_data: &[FaceData],
    healed_solid: &Solid,
    face_id_to_index: &HashMap<draper_topology::TopoId, usize>,
) -> Vec<FaceData> {
    let mut result = Vec::new();
    let mut used_original_indices: std::collections::HashSet<usize> = std::collections::HashSet::new();

    let faces = healed_solid.faces();
    for face in faces {
        if let Some(&orig_idx) = face_id_to_index.get(&face.id) {
            // This face existed in the original FaceData — reuse it with updates.
            // We preserve the original FaceData's STEP-specific metadata (step IDs,
            // 2D curves, etc.) and only apply structural changes from healing.
            used_original_indices.insert(orig_idx);
            let mut fd = original_face_data[orig_idx].clone();

            // Apply forward flag change (normal repair)
            fd.forward = face.forward;

            result.push(fd);
        } else {
            // This is a new face added by the healing pipeline (e.g., hole fill).
            // Create a FaceData with default STEP IDs.
            let outer_edges: Vec<TopoEdge> = if let Some(ref wire) = face.outer_wire {
                wire.coedges.iter()
                    .filter_map(|coedge| {
                        face.edges.iter().find(|e| e.id == coedge.edge).cloned()
                    })
                    .collect()
            } else {
                vec![]
            };

            let inner_edges: Vec<Vec<TopoEdge>> = face.inner_wires.iter()
                .map(|wire| {
                    wire.coedges.iter()
                        .filter_map(|coedge| {
                            face.edges.iter().find(|e| e.id == coedge.edge).cloned()
                        })
                        .collect()
                })
                .collect();

            let n_outer = outer_edges.len();
            let n_inner: Vec<usize> = inner_edges.iter().map(|ie| ie.len()).collect();

            let mut all_edges = outer_edges.clone();
            for inner in &inner_edges {
                all_edges.extend(inner.clone());
            }

            let n_edges = all_edges.len();

            result.push(FaceData {
                surface: face.surface.clone().unwrap_or(Surface::Plane(Plane::from_origin_and_normal(
                    Point3d::ORIGIN, Direction3d::Z))),
                outer_edges,
                inner_edges,
                edges: all_edges,
                forward: face.forward,
                step_face_id: 0, // Synthetic face — no STEP ID
                surface_step_id: None,
                edge_curves_2d: vec![None; n_edges],
                edge_step_ids: vec![0; n_edges],
                outer_edge_step_ids: vec![0; n_outer],
                inner_edge_step_ids: n_inner.iter().map(|&n| vec![0; n]).collect(),
                is_void: false, // Synthetic faces from healing are not void faces
            });
        }
    }

    result
}

/// Log a summary of the healing report.
fn log_healing_report(brep_id: i64, report: &HealingReport) {
    if report.total_fixes() == 0 {
        info!("BREP #{}: healing complete — no fixes needed", brep_id);
        return;
    }

    info!("BREP #{}: healing report — {} total fixes applied:", brep_id, report.total_fixes());
    if report.degenerate_edges_marked > 0 {
        info!("  • {} degenerate edges marked", report.degenerate_edges_marked);
    }
    if report.gaps_closed > 0 {
        info!("  • {} gaps closed", report.gaps_closed);
    }
    if report.holes_filled > 0 {
        info!("  • {} holes filled", report.holes_filled);
    }
    if report.edges_stitched > 0 {
        info!("  • {} edge pairs stitched", report.edges_stitched);
    }
    if report.normals_fixed > 0 {
        info!("  • {} face normals fixed", report.normals_fixed);
    }
    if report.small_faces_removed > 0 {
        info!("  • {} small-feature faces removed", report.small_faces_removed);
    }
    if report.sliver_triangles_detected > 0 {
        info!("  • {} sliver triangles detected", report.sliver_triangles_detected);
    }
    // Log individual messages at debug level
    for msg in &report.messages {
        log::debug!("  → {}", msg);
    }
}

/// Information about a single face within a BREP, for structure display and UV visualization.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceInfo {
    /// Unique face identifier (for selection and tracking).
    pub face_id: u64,
    /// STEP entity ID of the ADVANCED_FACE.
    pub step_face_id: i64,
    /// Human-readable surface type name (e.g., "Plane", "Cylinder", "Nurbs").
    pub surface_type: String,
    /// The surface geometry (for UV grid generation).
    pub surface: Surface,
    /// 3D boundary edge polylines (outer boundary).
    pub outer_boundary: Vec<Vec<Point3d>>,
    /// 3D boundary edge polylines (inner boundaries = holes).
    pub inner_boundaries: Vec<Vec<Point3d>>,
    /// UV-space boundary polylines (outer boundary).
    pub outer_uv_boundary: Vec<Vec<Point2d>>,
    /// UV-space boundary polylines (inner boundaries = holes).
    pub inner_uv_boundaries: Vec<Vec<Vec<Point2d>>>,
    /// Triangle index range [start, end) in the merged mesh for this face.
    pub triangle_range: (usize, usize),
    /// Whether the face normal matches the surface normal.
    pub forward: bool,
    /// UV-space triangles for visualization: each triangle is 3 UV points.
    /// Used to display the actual triangulation in the UV grid SVG.
    pub uv_triangles: Vec<[Point2d; 3]>,
    /// Whether this face belongs to a void shell (internal cavity).
    /// Void faces have normals pointing INTO the solid material.
    pub is_void: bool,
}
/// A mesh instance to be rendered — the mesh geometry is transformed by the given matrix
/// and painted with the given color. Multiple instances can reference the same BREP geometry
/// but with different transforms (e.g., a bolt inserted 6 times at different positions).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MeshInstance {
    /// Human-readable name (from STEP PRODUCT or NAUO).
    pub name: String,
    /// The triangulated mesh (already transformed to world space).
    pub mesh: TriangleMesh,
    /// Optional RGBA color (0..1 range).
    pub color: Option<[f32; 4]>,
    /// The 4×4 transform that was applied to get this instance into world space.
    pub transform: Option<[[f64; 4]; 4]>,
    /// The STEP entity ID of the source MANIFOLD_SOLID_BREP.
    pub brep_id: i64,
}

/// A detailed mesh instance with per-face information for structure display,
/// selection, UV grid visualization, and debugging.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DetailedMeshInstance {
    /// Human-readable name (from STEP PRODUCT or NAUO).
    pub name: String,
    /// The triangulated mesh (already transformed to world space) with per-triangle face IDs.
    pub mesh: TriangleMesh,
    /// Optional RGBA color (0..1 range).
    pub color: Option<[f32; 4]>,
    /// The 4×4 transform that was applied to get this instance into world space.
    pub transform: Option<[[f64; 4]; 4]>,
    /// The STEP entity ID of the source MANIFOLD_SOLID_BREP.
    pub brep_id: i64,
    /// Per-face information for structure display and UV visualization.
    pub faces: Vec<FaceInfo>,
}

/// A node in the STEP assembly tree (for structure display).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AssemblyNode {
    /// Name of this assembly node.
    pub name: String,
    /// STEP entity ID of the PRODUCT_DEFINITION.
    pub pd_id: i64,
    /// STEP entity ID of the MANIFOLD_SOLID_BREP (if leaf).
    pub brep_id: Option<i64>,
    /// Index into the detailed_instances Vec that this leaf node corresponds to.
    /// Set only for leaf nodes. Multiple leaf nodes may share the same brep_id
    /// but each will have a unique instance_index (e.g., bolt at different positions).
    pub instance_index: Option<usize>,
    /// Transform from parent to this node.
    pub transform: Option<[[f64; 4]; 4]>,
    /// Color for this node.
    pub color: Option<[f32; 4]>,
    /// Child nodes.
    pub children: Vec<AssemblyNode>,
}

/// Convert a parsed STEP file to a single merged triangle mesh.
pub fn step_to_mesh(step_file: &StepFile) -> Result<TriangleMesh, String> {
    step_to_mesh_with_config(step_file, &StepConversionConfig::default())
}

/// Extract all BREP solids from a parsed STEP file WITHOUT triangulating.
///
/// This is the foundation for editing workflows: read STEP → extract solids →
/// edit (transform faces, add/remove holes, modify NURBS) → export back to STEP.
///
/// Each MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS in the STEP file becomes one `Solid`.
/// Void shells are attached as `Solid::inner_shells`.
///
/// Returns `(solids, brep_ids)` where `brep_ids[i]` is the STEP entity ID of
/// the BREP that produced `solids[i]`.
pub fn extract_solids(step_file: &StepFile) -> (Vec<Solid>, Vec<i64>) {
    let config = StepConversionConfig::default();
    let converter = StepConverter::with_config(step_file, config);
    converter.extract_all_solids()
}

/// Extract solids from a STEP file using a custom conversion config
/// (e.g., disabling healing for raw extraction).
pub fn extract_solids_with_config(
    step_file: &StepFile,
    config: &StepConversionConfig,
) -> (Vec<Solid>, Vec<i64>) {
    let converter = StepConverter::with_config(step_file, config.clone());
    converter.extract_all_solids()
}

/// Convert a parsed STEP file to a single merged triangle mesh with custom configuration.
pub fn step_to_mesh_with_config(step_file: &StepFile, config: &StepConversionConfig) -> Result<TriangleMesh, String> {
    let converter = StepConverter::with_config(step_file, config.clone());
    converter.convert()
}

/// Convert a parsed STEP file to mesh instances (one per assembly leaf occurrence).
/// Each instance has its own transform and color — the same BREP can appear
/// multiple times with different transforms (e.g., bolt inserted 6 times).
pub fn step_to_mesh_instances(step_file: &StepFile) -> Result<Vec<MeshInstance>, String> {
    step_to_mesh_instances_with_config(step_file, &StepConversionConfig::default())
}

/// Convert a parsed STEP file to mesh instances with custom configuration.
pub fn step_to_mesh_instances_with_config(step_file: &StepFile, config: &StepConversionConfig) -> Result<Vec<MeshInstance>, String> {
    let converter = StepConverter::with_config(step_file, config.clone());
    converter.convert_instances()
}

/// Convert a parsed STEP file to detailed mesh instances with per-face information.
/// Includes face IDs, surface types, boundary polylines, and UV-space data
/// for structure display, selection, UV grid visualization, and debugging.
pub fn step_to_detailed_instances(step_file: &StepFile) -> Result<Vec<DetailedMeshInstance>, String> {
    step_to_detailed_instances_with_config(step_file, &StepConversionConfig::default())
}

/// Convert a parsed STEP file to detailed mesh instances with custom configuration.
pub fn step_to_detailed_instances_with_config(step_file: &StepFile, config: &StepConversionConfig) -> Result<Vec<DetailedMeshInstance>, String> {
    let converter = StepConverter::with_config(step_file, config.clone());
    converter.convert_detailed_instances()
}

/// Get the assembly tree structure of a STEP file for display/debugging.
pub fn step_structure(step_file: &StepFile) -> AssemblyNode {
    let converter = StepConverter::new(step_file);
    converter.build_assembly_tree()
}

/// Build the assembly tree AND detailed instances together, so that each leaf
/// AssemblyNode gets its `instance_index` populated (mapping it to the correct
/// entry in the returned instances Vec). This solves the problem of multiple
/// assembly nodes sharing the same brep_id (e.g., same bolt at different positions).
pub fn step_structure_with_instances(step_file: &StepFile) -> (AssemblyNode, Vec<DetailedMeshInstance>) {
    let converter = StepConverter::new(step_file);
    let mut tree = converter.build_assembly_tree();
    let instances = converter.convert_detailed_instances().unwrap_or_default();

    // Walk the assembly tree leaf nodes in the same order as the NAUO tree walk
    // that generated the instances, and assign instance_index to each leaf.
    // The instances are created by walk_assembly_tree_detailed which visits
    // leaf nodes in the same DFS order as build_assembly_tree.
    let mut next_index: usize = 0;
    assign_instance_indices(&mut tree, &instances, &mut next_index);

    (tree, instances)
}

// ============================================================
// Lazy (progressive) conversion — no triangulation in the initial call
// ============================================================

/// A pending BREP instance that has NOT been triangulated yet.
///
/// Contains all the metadata needed to produce a `DetailedMeshInstance`
/// (name, transform, color, brep_id), but the expensive triangulation
/// is deferred to a separate call to `triangulate_pending_instance()`.
///
/// This is used by `step_structure_lazy()` to return the assembly tree
/// and instance descriptors quickly, then triangulate them one-by-one
/// in a progressive frame loop (e.g., the wasm web viewer).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PendingBrepInstance {
    /// Human-readable name (from STEP PRODUCT or NAUO).
    pub name: String,
    /// The STEP entity ID of the source MANIFOLD_SOLID_BREP.
    pub brep_id: i64,
    /// The 4×4 transform to apply to get this instance into world space.
    pub transform: Option<[[f64; 4]; 4]>,
    /// Optional RGBA color (0..1 range).
    pub color: Option<[f32; 4]>,
    /// Optional estimate of the face count for this BREP.
    /// Used by the viewer to decide how aggressively to downgrade LOD on
    /// mobile (>2000 faces → 2-level downgrade, else 1-level). `None` if
    /// the estimate is not available (viewer treats it as 0, i.e., no
    /// downgrade beyond the default 1-level).
    #[cfg_attr(feature = "serde", serde(default))]
    pub face_count_estimate: Option<usize>,
}

/// Result of `triangulate_pending_chunked` — allows progressive, non-blocking
/// BREP triangulation across multiple frames on WASM.
#[derive(Debug)]
pub enum TriangulatePendingResult {
    /// The BREP is fully triangulated. `Option<DetailedMeshInstance>` is `Some`
    /// if the mesh is non-empty, `None` if triangulation failed (zero faces/triangles).
    Done(Option<DetailedMeshInstance>),
    /// The BREP is partially triangulated. More faces remain — call again next frame.
    /// `faces_done` / `faces_total` can be used for progress display.
    InProgress { faces_done: usize, faces_total: usize },
}

/// Active intra-BREP triangulation session for chunked progressive loading.
///
/// Holds the setup state (healed face_data_list, edge_cache, dedup_map) and
/// accumulated output (mesh, face_infos) between chunks. Created by
/// `start_brep_session`, advanced by `process_chunk`, consumed by `finalize`.
pub struct BrepSession {
    /// BREP being triangulated.
    brep_id: i64,
    /// Healed face_data_list (from setup phase).
    face_data_list: Vec<FaceData>,
    /// Edge discretization cache (mutated as faces are processed).
    edge_cache: EdgeDiscretizationCache,
    /// Tolerance context (from bounding box).
    tol_ctx: ToleranceContext,
    /// Vertex dedup map (mutated as faces are merged).
    dedup_map: draper_mesh::mesh::VertexDedupMap,
    /// Accumulated mesh across all processed faces.
    mesh: TriangleMesh,
    /// Accumulated face info across all processed faces.
    face_infos: Vec<FaceInfo>,
    /// Next face ID to assign.
    next_face_id: u64,
    /// Index of the next face to process in face_data_list.
    next_face_idx: usize,
    /// Total face vertices processed (for dedup stats logging).
    total_face_vertices: usize,
    /// Number of faces skipped due to time limits.
    skipped_faces: usize,
    /// Start time of the BREP (for time limit enforcement).
    brep_start: StdInstant,
    /// Per-BREP time limit.
    brep_time_limit: std::time::Duration,
    /// Per-FACE time limit.
    face_time_limit: std::time::Duration,
    /// Triangulation params (cloned from context).
    params: TriangulationParams,
    /// Bounding box (cloned from context).
    bbox: Option<(Point3d, Point3d)>,
}

/// Build the assembly tree and collect BREP instance descriptors — **without**
/// triangulating any geometry.
///
/// Returns the assembly tree (for immediate structure display) and a list of
/// `PendingBrepInstance` descriptors. Each descriptor can later be triangulated
/// individually via `triangulate_pending_instance()`.
///
/// This function is fast (O(n) in STEP entity count) because it only resolves
/// references and collects metadata — no mesh computation is performed.
pub fn step_structure_lazy(step_file: &StepFile) -> (AssemblyNode, Vec<PendingBrepInstance>) {
    let converter = StepConverter::new(step_file);
    let mut tree = converter.build_assembly_tree();
    let pending = converter.collect_pending_instances();

    // Assign instance_index to tree leaves based on pending list
    let mut next_index: usize = 0;
    assign_instance_indices_pending(&mut tree, &pending, &mut next_index);

    (tree, pending)
}

/// Triangulate a single pending BREP instance on demand.
///
/// This is the progressive counterpart to `step_structure_lazy()`: instead of
/// triangulating all BREPs at once (which blocks the main thread on wasm),
/// call this function once per frame for each pending instance.
///
/// Returns `None` if the BREP cannot be triangulated (missing geometry,
/// unsupported surface types, etc.).
///
/// # Timeout
/// On wasm32, a per-BREP time limit of 3 seconds is enforced via
/// `TriangulationGuard`. Faces that exceed the limit are skipped,
/// producing a partial mesh instead of hanging the browser.
/// A reusable conversion context for progressive BREP triangulation.
///
/// Instead of creating a new `StepConverter` for every BREP instance (which
/// rebuilds entity maps and recomputes bounding boxes), this context keeps
/// a single converter alive across all triangulation calls.
///
/// Usage:
/// ```ignore
/// let ctx = StepConversionContext::new(&step_file);
/// for pending in &pending_instances {
///     if let Some(instance) = ctx.triangulate_pending(pending) {
///         // use instance
///     }
/// }
/// ```
pub struct StepConversionContext<'a> {
    converter: StepConverter<'a>,
    bbox: Option<(Point3d, Point3d)>,
    params: TriangulationParams,
    /// BREP triangulation cache: brep_id → (mesh_in_brep_local_space, face_info).
    /// Uses RefCell for interior mutability since triangulate_pending takes &self.
    brep_detail_cache: std::cell::RefCell<HashMap<i64, (TriangleMesh, Vec<FaceInfo>)>>,
}

impl<'a> StepConversionContext<'a> {
    /// Create a new conversion context for the given STEP file reference.
    /// Computes bounding box once and prepares adaptive triangulation parameters.
    /// The StepFile's internal type index (lazy) will be reused across calls.
    pub fn new(step_file: &'a StepFile) -> Self {
        // Enable healing with aggressive preset — it fixes gaps, holes,
        // flipped normals, self-intersections, and sliver faces.
        // The aggressive preset fixes ~85% of defects in "dirty" STEP files
        // from SolidWorks/CATIA, compared to ~30% for conservative.
        let config = StepConversionConfig::default();

        let converter = StepConverter::with_config(step_file, config);
        let bbox = converter.compute_bounding_box();

        let mut params = TriangulationParams::default();
        if let Some((bmin, bmax)) = &bbox {
            let dx = bmax.x - bmin.x;
            let dy = bmax.y - bmin.y;
            let dz = bmax.z - bmin.z;
            let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
            if diagonal > 1.0 {
                params.max_deviation = params.max_deviation.max(diagonal * 0.0002);
            }
        }

        Self { converter, bbox, params, brep_detail_cache: std::cell::RefCell::new(HashMap::new()) }
    }

    /// Triangulate a single pending BREP instance.
    ///
    /// Returns `None` if the BREP cannot be triangulated (missing geometry,
    /// unsupported surface types, etc.).
    ///
    /// # Timeout
    /// On wasm32, a per-BREP time limit of 10 seconds is enforced.
    /// Faces that exceed the limit are skipped, producing a partial mesh
    /// instead of hanging the browser.
    pub fn triangulate_pending(&self, pending: &PendingBrepInstance) -> Option<DetailedMeshInstance> {
        // Check BREP cache first — avoid re-triangulating the same BREP
        let (mesh, faces) = {
            let cache = self.brep_detail_cache.borrow();
            if let Some(cached) = cache.get(&pending.brep_id) {
                log::info!(
                    "BREP #{} — using cached triangulation (instance of previously computed BREP)",
                    pending.brep_id
                );
                cached.clone()
            } else {
                drop(cache); // Release borrow before mutating
                let result = self.converter.triangulate_brep_detailed(pending.brep_id, &self.params, &self.bbox)?;
                self.brep_detail_cache.borrow_mut().insert(pending.brep_id, result.clone());
                result
            }
        };

        // Apply the instance transform
        let mut instance_mesh = mesh;
        if let Some(ref tf) = pending.transform {
            instance_mesh.transform(tf);
        }

        Some(DetailedMeshInstance {
            name: pending.name.clone(),
            mesh: instance_mesh,
            color: pending.color,
            transform: pending.transform,
            brep_id: pending.brep_id,
            faces,
        })
    }
}

// ============================================================
// Owned conversion context (for caching across frames)
// ============================================================

/// An **owning** conversion context that takes ownership of the `StepFile`.
///
/// This is designed for progressive triangulation where the context must be
/// cached across multiple animation frames (e.g., in a WASM web viewer).
/// Unlike `StepConversionContext<'a>` which borrows the `StepFile`, this
/// struct owns it — so it can be stored as a field in a viewer struct without
/// lifetime issues.
///
/// Usage:
/// ```ignore
/// let ctx = OwnedStepConversionContext::new(step_file);
/// for pending in &pending_instances {
///     if let Some(instance) = ctx.triangulate_pending(pending) { ... }
/// }
/// ```
pub struct OwnedStepConversionContext {
    step_file: StepFile,
    bbox: Option<(Point3d, Point3d)>,
    params: TriangulationParams,
    /// Pre-built index: pd_id → brep_id. Cached here to avoid re-cloning
    /// from StepFile's RefCell on every triangulate_pending() call.
    pd_brep_map: HashMap<i64, Option<i64>>,
    /// Pre-built index: nauo_id → transform. Cached here for the same reason.
    nauo_transform_map: HashMap<i64, Option<[[f64; 4]; 4]>>,
    /// Pre-built entity map: step_entity_id → index in entities vec.
    entity_map: HashMap<i64, usize>,
    /// Conversion configuration (healing on/off).
    config: StepConversionConfig,
    /// Whether the bounding box has been computed (for lazy init on WASM).
    bbox_computed: bool,
    /// BREP triangulation cache: brep_id → (mesh_in_brep_local_space, face_info).
    /// When the same BREP appears via STEP mapped_item/instance, the cached
    /// triangulation is reused with just a transform applied, eliminating
    /// redundant re-triangulation and guaranteeing identical meshes.
    brep_detail_cache: HashMap<i64, (TriangleMesh, Vec<FaceInfo>)>,
    /// Active intra-BREP chunked triangulation session.
    ///
    /// When `Some`, a BREP is being triangulated across multiple frames.
    /// The session holds the healed face_data_list, edge_cache, dedup_map,
    /// and accumulated mesh/face_infos between chunks. This allows the viewer
    /// to process N faces per frame (yielding to the browser between chunks)
    /// instead of blocking the UI for the entire BREP.
    ///
    /// On native, this is never used (triangulate_pending does the whole BREP
    /// in one call). On WASM, triangulate_pending_chunked uses this.
    active_session: Option<Box<BrepSession>>,
}

impl OwnedStepConversionContext {
    /// Create a new owning conversion context.
    /// Pre-builds and caches all index maps to avoid per-BREP recomputation.
    ///
    /// On WASM, the bounding box computation is deferred to the first
    /// `triangulate_pending()` call to avoid blocking the main thread
    /// during construction. This makes `new()` return quickly, keeping
    /// the browser responsive.
    pub fn new(step_file: StepFile) -> Self {
        // Delegate to new_with_lod using the default LOD (1.0 = full quality).
        // This preserves backward compatibility for callers that don't care
        // about LOD. The viewer's `process_pending_breps` calls `new_with_lod`
        // directly with the user-selected LOD value so the Quality dropdown
        // actually affects STEP triangulation.
        Self::new_with_lod(step_file, 1.0)
    }

    /// Create a conversion context that uses `TriangulationParams::for_lod(lod)`
    /// instead of `TriangulationParams::default()`. This is what makes the
    /// viewer's Quality dropdown (Preview / Low / Medium / High / Ultra)
    /// actually affect STEP file triangulation — lower LOD values produce
    /// coarser meshes (fewer triangles, larger deviation), higher LOD values
    /// produce finer meshes.
    ///
    /// `lod` is a value in `[0.0, 1.0]` where `0.0` = coarsest, `1.0` = full
    /// quality. The bbox-based `max_deviation` floor (`diagonal * 0.0002`)
    /// is still applied on top of the LOD-scaled deviation, so very small
    /// models don't get absurdly tight tolerances at high LOD.
    pub fn new_with_lod(step_file: StepFile, lod: f64) -> Self {
        // Start from LOD-scaled params (NOT default()) so the Quality
        // dropdown actually changes triangle count. for_lod(lod) scales
        // max_deviation, angular_samples, height_samples, max_face_triangles,
        // and detail_level proportionally to `lod`.
        let mut params = TriangulationParams::for_lod(lod.clamp(0.0, 1.0));
        // Enable adaptive LOD — each face gets a fair share of the total
        // triangle budget instead of triangulating all faces at full quality
        // and then decimating.
        params.adaptive_lod_enabled = true;
        Self::new_with_params(step_file, params)
    }

    /// Create a new owning conversion context with an LOD value and a Steiner
    /// budget profile.
    ///
    /// This is the entry point used by the WASM viewer, which detects the
    /// platform (mobile / tablet / desktop) from `screen_width` and selects
    /// the appropriate profile. The profile controls the maximum grid density
    /// per face — mobile uses lower caps to avoid UI freezes on slow CPUs,
    /// desktop uses higher caps for visual quality.
    ///
    /// See `draper_mesh::triangulate::SteinerBudgetProfile` for the full
    /// matrix of cap values per profile.
    pub fn new_with_lod_and_profile(
        step_file: StepFile,
        lod: f64,
        profile: draper_mesh::triangulate::SteinerBudgetProfile,
    ) -> Self {
        let mut params = TriangulationParams::for_lod(lod.clamp(0.0, 1.0));
        params.steiner_profile = profile;
        // Enable adaptive LOD — each face gets a fair share of the total
        // triangle budget instead of triangulating all faces at full quality
        // and then decimating. The per-face budget is computed later in
        // triangulate_brep_detailed when the face count is known.
        params.adaptive_lod_enabled = true;
        Self::new_with_params(step_file, params)
    }

    /// Create a new owning conversion context with explicit triangulation params.
    ///
    /// This is the most flexible entry point — it accepts a fully-constructed
    /// `TriangulationParams` rather than just an LOD value, so callers that
    /// need fine-grained control (e.g., custom max_deviation for high-precision
    /// manufacturing exports) can use it directly. The viewer's LOD path
    /// typically goes through `new_with_lod` instead, which builds params from
    /// an LOD value via `TriangulationParams::for_lod(lod)`.
    ///
    /// The bounding-box-aware `max_deviation` adjustment is applied on top of
    /// whatever `params` the caller supplies, so it is safe to pass LOD-scaled
    /// params here — they will only be made more conservative (never coarser)
    /// if the model's diagonal is large enough to warrant it.
    pub fn new_with_params(step_file: StepFile, mut params: TriangulationParams) -> Self {
        // Enable healing with default preset — it fixes gaps, holes,
        // flipped normals, degenerate edges, and small features.
        // The aggressive preset (fix_self_intersections) can be enabled
        // separately via StepConversionConfig for "dirty" files.
        let config = StepConversionConfig::default();

        // Cache the index maps — after this, creating a lightweight StepConverter
        // is nearly free (just clones of already-built maps).
        let pd_brep_map = step_file.pd_brep_index().clone();
        let nauo_transform_map = step_file.nauo_transform_index().clone();
        // Reuse the StepFile's entity_index instead of rebuilding it from scratch
        let entity_map = step_file.entity_index_ref().clone();

        // Compute bounding box eagerly on all platforms for consistent results.
        // The cost is negligible compared to the triangulation that follows.
        let bbox = {
            let converter = StepConverter::with_config(&step_file, config.clone());
            converter.compute_bounding_box()
        };

        // Apply the bbox-based max_deviation floor.
        //
        // This floor protects very small models from getting an absurdly tight
        // tolerance at high LOD (max_deviation = 0.01 / 1.0² = 0.01, but if
        // the model diagonal is 0.5mm, 0.01 is 2% of the model — way too tight).
        //
        // LOD-AWARENESS: For low LODs (where max_deviation is large, ≥ 1.0),
        // the floor is NOT applied — otherwise LOD 0.3, 0.5, 0.75, and 1.0
        // would all get the SAME clamped max_deviation (= diagonal * 0.0002)
        // on large models, making the Quality selector have no visible effect
        // on curved surfaces.
        //
        // For high LODs (max_deviation < 1.0, i.e. lod > 0.1), the floor is
        // applied as before.
        if let Some((bmin, bmax)) = &bbox {
            let dx = bmax.x - bmin.x;
            let dy = bmax.y - bmin.y;
            let dz = bmax.z - bmin.z;
            let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
            if diagonal > 1.0 {
                let floor = diagonal * 0.0002;
                if params.max_deviation < floor && params.max_deviation < 1.0 {
                    params.max_deviation = floor;
                }
            }
            // Compute bounding box surface area for adaptive per-face-area
            // budget scaling (task 1.1.4). The surface area of a box is
            // 2*(dx*dy + dy*dz + dz*dx). This is used by
            // `face_area_budget_multiplier` in `triangulate_surface_consistent`
            // to give larger faces more Steiner points and smaller faces fewer.
            let bbox_area = 2.0 * (dx * dy + dy * dz + dz * dx);
            if bbox_area > 1e-10 {
                params.bbox_surface_area = Some(bbox_area);
            }
        }

        // Enable parallel face triangulation on native (uses rayon internally).
        // On WASM, threads are not available, so keep sequential.
        #[cfg(not(target_arch = "wasm32"))]
        {
            params.parallel = true;
        }

        // Use consistent max_face_triangles on all platforms
        // (no WASM-specific cap — quality should match native)

        Self { step_file, bbox, params, pd_brep_map, nauo_transform_map, entity_map, config, bbox_computed: false, brep_detail_cache: HashMap::new(), active_session: None }
    }

    /// Replace the triangulation parameters used by subsequent `triangulate_pending()` calls.
    ///
    /// **Important:** this also clears the BREP triangulation cache, because
    /// any previously-cached meshes were generated with the old params and must
    /// not be reused at a different LOD.
    ///
    /// This is used by the viewer when the user changes the LOD selector after
    /// a STEP file has been loaded — re-triangulating with the new params gives
    /// visibly different vertex/triangle counts.
    pub fn set_params(&mut self, params: TriangulationParams) {
        // Apply the same LOD-aware bbox max_deviation floor used in new_with_params().
        let mut params = params;
        if let Some((bmin, bmax)) = &self.bbox {
            let dx = bmax.x - bmin.x;
            let dy = bmax.y - bmin.y;
            let dz = bmax.z - bmin.z;
            let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
            if diagonal > 1.0 {
                let floor = diagonal * 0.0002;
                if params.max_deviation < floor && params.max_deviation < 1.0 {
                    params.max_deviation = floor;
                }
            }
            // Also propagate bbox_surface_area for adaptive per-face budget
            let bbox_area = 2.0 * (dx * dy + dy * dz + dz * dx);
            if bbox_area > 1e-10 {
                params.bbox_surface_area = Some(bbox_area);
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            params.parallel = true;
        }
        self.params = params;
        // Invalidate any cached meshes — they were triangulated at the old LOD.
        self.brep_detail_cache.clear();
    }

    /// Triangulate a single pending BREP instance.
    ///
    /// Returns `None` if the BREP cannot be triangulated.
    ///
    /// On WASM, the bounding box is computed lazily on the first call
    /// to avoid blocking the main thread during `new()`.
    pub fn triangulate_pending(&mut self, pending: &PendingBrepInstance) -> Option<DetailedMeshInstance> {
        // Lazy bounding box computation on WASM (deferred from new())
        if !self.bbox_computed && self.bbox.is_none() {
            let converter = StepConverter::with_config(&self.step_file, self.config.clone());
            self.bbox = converter.compute_bounding_box();
            self.bbox_computed = true;

            // Update params based on actual bounding box.
            //
            // IMPORTANT: This bbox-floor is LOD-AWARE — it only applies at
            // higher LODs (≥ 0.5, where the user expects maximum quality).
            // For low LODs the raw max_deviation from for_lod() is allowed to
            // stand, otherwise all LODs from 0.3 to 1.0 would get the SAME
            // clamped max_deviation (= diagonal * 0.0002) and LOD changes
            // would have no visible effect on curved surfaces.
            //
            // We reconstruct the LOD value from max_deviation: for_lod(lod)
            // sets max_deviation = 0.01 / lod² (for lod ≥ 0.01). So
            //   lod ≈ sqrt(0.01 / max_deviation).
            // If max_deviation is very large (≥ 1.0, i.e. lod ≤ 0.1), skip
            // the floor entirely.
            if let Some((bmin, bmax)) = &self.bbox {
                let dx = bmax.x - bmin.x;
                let dy = bmax.y - bmin.y;
                let dz = bmax.z - bmin.z;
                let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
                if diagonal > 1.0 {
                    let floor = diagonal * 0.0002;
                    // Skip the floor for low LODs (where max_deviation is large).
                    // For high LODs, apply the floor as before (protects small models).
                    if self.params.max_deviation < floor && self.params.max_deviation < 1.0 {
                        // Only clamp UP if the raw max_deviation is "tight"
                        // (below the floor) AND we're at a high LOD (raw < 1.0).
                        // For low LODs (raw ≥ 1.0), leave it alone.
                        self.params.max_deviation = floor;
                    }
                }
            }
        }

        // Check the BREP triangulation cache first — if the same BREP has already
        // been triangulated (e.g., a bolt used 6 times in an assembly), reuse the
        // cached result instead of re-triangulating from scratch. This eliminates
        // both redundant computation and the possibility of different triangulations
        // for the same shape (guaranteeing watertight instances).
        //
        // IMPORTANT: The cached mesh is the FULL-resolution mesh (before decimation).
        // Decimation is applied PER-INSTANCE below, because the keep_ratio is the
        // same for all instances of the same BREP (it's a property of self.params,
        // which doesn't change between calls to triangulate_pending() — only
        // set_params() changes it, and that clears the cache).
        //
        // Optimization: if keep_ratio is constant across calls, we could cache
        // the decimated result too. But the per-instance decimation is fast
        // (~1ms per BREP for Zentralstaender-sized meshes), so the overhead
        // is negligible compared to the initial triangulation.
        let (mesh, faces) = if let Some(cached) = self.brep_detail_cache.get(&pending.brep_id) {
            log::info!(
                "BREP #{} — using cached triangulation (instance of previously computed BREP)",
                pending.brep_id
            );
            cached.clone()
        } else {
            // Build a lightweight StepConverter using cached maps.
            let converter = StepConverter::from_cached_maps(
                &self.step_file,
                self.config.clone(),
                self.entity_map.clone(),
                self.pd_brep_map.clone(),
                self.nauo_transform_map.clone(),
            );
            let result = converter.triangulate_brep_detailed(pending.brep_id, &self.params, &self.bbox)?;
            self.brep_detail_cache.insert(pending.brep_id, result.clone());
            result
        };

        // Apply the instance transform
        let mut instance_mesh = mesh;
        if let Some(ref tf) = pending.transform {
            instance_mesh.transform(tf);
        }

        // Apply post-triangulation decimation for LOD support.
        //
        // This is what makes "Preview" (LOD 0.1, keep_ratio ~0.25) visibly
        // different from "Ultra" (LOD 1.0, keep_ratio 1.0) on STEP files
        // whose faces are predominantly planar with linear edges — there,
        // per-face triangulation alone gives the same N-2 triangles regardless
        // of LOD, so without decimation the Quality dropdown has no visible
        // effect.
        //
        // The decimation algorithm:
        // 1. Welds coincident vertices (so adjacent triangles share edges).
        // 2. Finds the shortest internal edge (shared by exactly 2 triangles).
        // 3. Collapses it (moves both endpoints to midpoint, or to the
        //    boundary vertex if one end is a boundary vertex).
        // 4. Removes degenerate triangles.
        // 5. Repeats until target_count = original * keep_ratio is reached.
        //
        // Decimation preserves the silhouette (boundary edges are never
        // collapsed, boundary vertices are never moved).
        //
        // Skip decimation when adaptive LOD is enabled — each face already
        // respects its per-face budget, so global decimation is unnecessary.
        if !self.params.adaptive_lod_enabled && self.params.keep_ratio < 1.0 && instance_mesh.triangle_count() >= 4 {
            let (orig, final_) = draper_mesh::decimate_mesh(&mut instance_mesh, self.params.keep_ratio);
            if final_ < orig {
                log::info!(
                    "BREP #{} — decimated {}→{} triangles (keep_ratio={:.2}, LOD target reached)",
                    pending.brep_id, orig, final_, self.params.keep_ratio
                );
            }
        }

        Some(DetailedMeshInstance {
            name: pending.name.clone(),
            mesh: instance_mesh,
            color: pending.color,
            transform: pending.transform,
            brep_id: pending.brep_id,
            faces,
        })
    }

    /// Triangulate multiple BREP instances in **parallel** using rayon.
    ///
    /// This is the native-only counterpart to `triangulate_pending()` —
    /// instead of processing BREPs one-at-a-time sequentially, it dispatches
    /// all uncached BREPs to rayon's thread pool. Each thread creates its own
    /// `StepConverter`, `EdgeDiscretizationCache`, and dedup map, so there is
    /// **no shared mutable state** between threads. The `StepFile` reference
    /// is shared read-only (it's `Sync`).
    ///
    /// BREPs that are already in `brep_detail_cache` are resolved on the
    /// calling thread (just a clone) — only uncached BREPs go through
    /// rayon. After completion, all new results are inserted into the cache.
    ///
    /// Returns results in the same order as the input `pending` list.
    ///
    /// # Cancellation
    /// If `cancel_flag` returns `true` at any point, remaining BREPs are
    /// skipped and partial results are returned. This enables the viewer's
    /// Cancel button to work with parallel triangulation.
    ///
    /// # Progress
    /// `progress_callback` is called after each BREP completes (from any
    /// thread) with `(completed_count, total_count)`. The callback must be
    /// thread-safe.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn triangulate_breps_parallel<F, C>(
        &mut self,
        pending: &[PendingBrepInstance],
        cancel_flag: C,
        progress_callback: F,
    ) -> Vec<Option<DetailedMeshInstance>>
    where
        F: Fn(usize, usize) + Sync + Send,
        C: Fn() -> bool + Sync + Send,
    {
        if pending.is_empty() {
            return Vec::new();
        }

        // Lazy bounding box computation (same as triangulate_pending).
        if !self.bbox_computed && self.bbox.is_none() {
            let converter = StepConverter::with_config(&self.step_file, self.config.clone());
            self.bbox = converter.compute_bounding_box();
            self.bbox_computed = true;
            if let Some((bmin, bmax)) = &self.bbox {
                let dx = bmax.x - bmin.x;
                let dy = bmax.y - bmin.y;
                let dz = bmax.z - bmin.z;
                let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
                if diagonal > 1.0 {
                    let floor = diagonal * 0.0002;
                    if self.params.max_deviation < floor && self.params.max_deviation < 1.0 {
                        self.params.max_deviation = floor;
                    }
                }
            }
        }

        // Ensure StepFile indexes are populated BEFORE entering parallel scope.
        // The RefCell-based lazy init in StepFile is NOT thread-safe — we must
        // trigger it on the main thread so that rayon threads only read.
        {
            let _ = self.step_file.pd_brep_index();
            let _ = self.step_file.nauo_transform_index();
        }

        // Separate cached vs uncached BREPs.
        // Cached BREPs are resolved immediately (just a clone); uncached go to rayon.
        let mut results: Vec<Option<DetailedMeshInstance>> = Vec::with_capacity(pending.len());
        let mut uncached_indices: Vec<usize> = Vec::new();
        let mut uncached_brep_ids: Vec<i64> = Vec::new();
        let mut uncached_pending: Vec<PendingBrepInstance> = Vec::new();

        for (i, p) in pending.iter().enumerate() {
            if let Some(cached) = self.brep_detail_cache.get(&p.brep_id) {
                // Cache hit — resolve immediately
                let (mesh, faces) = cached.clone();
                let mut instance_mesh = mesh;
                if let Some(ref tf) = p.transform {
                    instance_mesh.transform(tf);
                }
                // Apply decimation for non-adaptive LOD
                if !self.params.adaptive_lod_enabled && self.params.keep_ratio < 1.0 && instance_mesh.triangle_count() >= 4 {
                    draper_mesh::decimate_mesh(&mut instance_mesh, self.params.keep_ratio);
                }
                results.push(Some(DetailedMeshInstance {
                    name: p.name.clone(),
                    mesh: instance_mesh,
                    color: p.color,
                    transform: p.transform,
                    brep_id: p.brep_id,
                    faces,
                }));
            } else {
                results.push(None); // placeholder — will be filled by rayon
                uncached_indices.push(i);
                uncached_brep_ids.push(p.brep_id);
                uncached_pending.push(p.clone());
            }
        }

        if uncached_indices.is_empty() {
            return results; // all cached
        }

        // Clone the data needed by each rayon thread. Each thread will create
        // its own StepConverter from these pre-built maps — no RefCell access
        // inside the parallel scope.
        //
        // We extract the step_file reference separately from &self to avoid
        // borrowing conflicts with the mutable cache update later.
        let entity_map = self.entity_map.clone();
        let pd_brep_map = self.pd_brep_map.clone();
        let nauo_transform_map = self.nauo_transform_map.clone();
        let config = self.config.clone();
        let params = self.params.clone();
        let bbox = self.bbox;

        let total_count = uncached_indices.len();
        let completed = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));

        // Wrap callbacks in Arc so they can be shared across rayon threads.
        let cancel_flag = std::sync::Arc::new(cancel_flag);
        let progress_callback = std::sync::Arc::new(progress_callback);

        // Dispatch uncached BREPs to rayon.
        // We use a scoped thread pool so we can borrow step_file by reference.
        let parallel_results: Vec<(usize, DetailedMeshInstance, (TriangleMesh, Vec<FaceInfo>))> =
            rayon::scope(|s| {
                let step_file = &self.step_file;
                let (tx, rx) = std::sync::mpsc::channel();

                for (local_idx, (&result_idx, &brep_id)) in uncached_indices.iter().zip(uncached_brep_ids.iter()).enumerate() {
                    let tx = tx.clone();
                    let entity_map = entity_map.clone();
                    let pd_brep_map = pd_brep_map.clone();
                    let nauo_transform_map = nauo_transform_map.clone();
                    let config = config.clone();
                    let params = params.clone();
                    let pending_inst = uncached_pending[local_idx].clone();
                    let cancel_flag = cancel_flag.clone();
                    let progress_callback = progress_callback.clone();
                    let completed = completed.clone();

                    s.spawn(move |_| {
                        // Check cancellation
                        if cancel_flag() {
                            return;
                        }

                        // Each thread creates its own StepConverter — no shared mutable state.
                        let converter = StepConverter::from_cached_maps(
                            step_file,
                            config,
                            entity_map,
                            pd_brep_map,
                            nauo_transform_map,
                        );

                        let (mesh, faces) = match converter.triangulate_brep_detailed(brep_id, &params, &bbox) {
                            Some(result) => result,
                            None => return, // triangulation failed
                        };

                        // Cache the mesh in BREP-local space (before transform/decimation)
                        let cache_entry = (mesh.clone(), faces.clone());

                        // Apply instance transform
                        let mut instance_mesh = mesh;
                        if let Some(ref tf) = pending_inst.transform {
                            instance_mesh.transform(tf);
                        }

                        // Apply decimation for non-adaptive LOD
                        if !params.adaptive_lod_enabled && params.keep_ratio < 1.0 && instance_mesh.triangle_count() >= 4 {
                            draper_mesh::decimate_mesh(&mut instance_mesh, params.keep_ratio);
                        }

                        let instance = DetailedMeshInstance {
                            name: pending_inst.name.clone(),
                            mesh: instance_mesh,
                            color: pending_inst.color,
                            transform: pending_inst.transform,
                            brep_id,
                            faces,
                        };

                        // Report progress
                        let done = completed.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
                        progress_callback(done, total_count);

                        let _ = tx.send((result_idx, instance, cache_entry));
                    });
                }

                drop(tx); // drop the extra sender
                rx.iter().collect()
            });

        // Merge parallel results back into the results vector + cache.
        for (result_idx, instance, cache_entry) in parallel_results {
            let brep_id = instance.brep_id;
            results[result_idx] = Some(instance);
            self.brep_detail_cache.insert(brep_id, cache_entry);
        }

        results
    }

    /// Get a reference to the owned StepFile.
    pub fn step_file(&self) -> &StepFile {
        &self.step_file
    }

    /// Progressive (chunked) BREP triangulation — processes a time-bounded
    /// chunk of faces per call, yielding `InProgress` between chunks so the
    /// WASM main thread can repaint and process input.
    ///
    /// On the first call for a given BREP, sets up the session (extract faces,
    /// heal, build edge cache). On subsequent calls, continues processing
    /// faces from where the previous chunk left off. When all faces are done,
    /// runs post-processing (filter, weld, validate) and returns `Done`.
    ///
    /// `max_chunk_time` controls how long each chunk runs before yielding.
    /// Recommended: 200-500ms on WASM (keeps UI responsive).
    pub fn triangulate_pending_chunked(
        &mut self,
        pending: &PendingBrepInstance,
        max_chunk_time: std::time::Duration,
    ) -> TriangulatePendingResult {
        // Lazy bounding box computation (same as triangulate_pending).
        if !self.bbox_computed && self.bbox.is_none() {
            let converter = StepConverter::with_config(&self.step_file, self.config.clone());
            self.bbox = converter.compute_bounding_box();
            self.bbox_computed = true;
            if let Some((bmin, bmax)) = &self.bbox {
                let dx = bmax.x - bmin.x;
                let dy = bmax.y - bmin.y;
                let dz = bmax.z - bmin.z;
                let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
                if diagonal > 1.0 {
                    let floor = diagonal * 0.0002;
                    if self.params.max_deviation < floor && self.params.max_deviation < 1.0 {
                        self.params.max_deviation = floor;
                    }
                }
            }
        }

        // Check BREP cache first — if this BREP was already fully triangulated,
        // reuse the cached result (same logic as triangulate_pending).
        if let Some(cached) = self.brep_detail_cache.get(&pending.brep_id) {
            log::info!(
                "BREP #{} — using cached triangulation (chunked path)",
                pending.brep_id
            );
            let (mesh, faces) = cached.clone();
            return TriangulatePendingResult::Done(self.build_instance(pending, mesh, faces));
        }

        // If no active session, or session is for a different BREP, start a new one.
        let need_new_session = self.active_session.as_ref().map_or(true, |s| s.brep_id != pending.brep_id);
        if need_new_session {
            // Create a fresh converter for setup.
            let converter = StepConverter::from_cached_maps(
                &self.step_file,
                self.config.clone(),
                self.entity_map.clone(),
                self.pd_brep_map.clone(),
                self.nauo_transform_map.clone(),
            );
            let session = match converter.prepare_brep_session(
                pending.brep_id,
                &self.params,
                &self.bbox,
            ) {
                Some(s) => s,
                None => return TriangulatePendingResult::Done(None),
            };
            log::info!(
                "BREP #{}: started chunked session — {} faces, max_chunk_time={:?}",
                pending.brep_id, session.face_data_list.len(), max_chunk_time
            );
            self.active_session = Some(Box::new(session));
        }

        // Process a chunk of faces.
        let converter = StepConverter::from_cached_maps(
            &self.step_file,
            self.config.clone(),
            self.entity_map.clone(),
            self.pd_brep_map.clone(),
            self.nauo_transform_map.clone(),
        );
        let total_faces = self.active_session.as_ref().unwrap().face_data_list.len();
        let chunk_start = StdInstant::now();
        let mut _faces_this_chunk = 0usize;

        loop {
            let done = {
                let session = self.active_session.as_mut().unwrap();
                if session.next_face_idx >= session.face_data_list.len() {
                    true
                } else {
                    // Check BREP-level time limit.
                    if session.brep_start.elapsed() > session.brep_time_limit {
                        let remaining = session.face_data_list.len() - session.next_face_idx;
                        session.skipped_faces += remaining;
                        log::warn!(
                            "BREP #{}: time limit reached after {} faces, skipping {} remaining (chunked)",
                            session.brep_id, session.next_face_idx, remaining
                        );
                        true
                    } else {
                        // Check per-face time budget before starting an expensive face.
                        let elapsed = session.brep_start.elapsed();
                        if elapsed + session.face_time_limit > session.brep_time_limit {
                            let remaining = session.face_data_list.len() - session.next_face_idx;
                            session.skipped_faces += remaining;
                            log::warn!(
                                "BREP #{}: insufficient time budget for face {} (chunked), skipping {} remaining",
                                session.brep_id, session.next_face_idx, remaining
                            );
                            true
                        } else {
                            false
                        }
                    }
                }
            };

            if done {
                break;
            }

            // Process one face.
            let session = self.active_session.as_mut().unwrap();
            session.process_one_face(&converter);

            _faces_this_chunk += 1;

            // Check chunk time budget.
            if chunk_start.elapsed() >= max_chunk_time {
                break;
            }
        }

        let faces_done = self.active_session.as_ref().unwrap().next_face_idx;
        let is_complete = faces_done >= total_faces
            || self.active_session.as_ref().unwrap().brep_start.elapsed() > self.active_session.as_ref().unwrap().brep_time_limit;

        if is_complete {
            // Finalize: run post-processing and build the instance.
            let session = *self.active_session.take().unwrap();
            let brep_id = session.brep_id;
            let (mesh, faces) = session.finalize(&converter);

            // Cache the result (same as triangulate_pending).
            if mesh.triangle_count() > 0 {
                self.brep_detail_cache.insert(brep_id, (mesh.clone(), faces.clone()));
            }

            log::info!(
                "BREP #{}: chunked triangulation complete — {} verts, {} tris, {} faces",
                brep_id, mesh.vertex_count(), mesh.triangle_count(), faces.len()
            );

            TriangulatePendingResult::Done(self.build_instance(pending, mesh, faces))
        } else {
            TriangulatePendingResult::InProgress {
                faces_done,
                faces_total: total_faces,
            }
        }
    }

    /// Build a `DetailedMeshInstance` from a triangulated mesh + face infos,
    /// applying the instance transform and decimation. Shared between
    /// `triangulate_pending` and `triangulate_pending_chunked`.
    fn build_instance(
        &self,
        pending: &PendingBrepInstance,
        mut mesh: TriangleMesh,
        faces: Vec<FaceInfo>,
    ) -> Option<DetailedMeshInstance> {
        // Apply the instance transform
        if let Some(ref tf) = pending.transform {
            mesh.transform(tf);
        }

        // Apply post-triangulation decimation for LOD support.
        // Skip when adaptive LOD is enabled — each face already respects
        // its per-face budget, so global decimation is unnecessary.
        if !self.params.adaptive_lod_enabled && self.params.keep_ratio < 1.0 && mesh.triangle_count() >= 4 {
            let (orig, final_) = draper_mesh::decimate_mesh(&mut mesh, self.params.keep_ratio);
            if final_ < orig {
                log::info!(
                    "BREP #{} — decimated {}→{} triangles (keep_ratio={:.2})",
                    pending.brep_id, orig, final_, self.params.keep_ratio
                );
            }
        }

        Some(DetailedMeshInstance {
            name: pending.name.clone(),
            mesh,
            color: pending.color,
            transform: pending.transform,
            brep_id: pending.brep_id,
            faces,
        })
    }

    /// Abort any active chunked session (e.g., when the user cancels loading).
    pub fn abort_active_session(&mut self) {
        if self.active_session.take().is_some() {
            log::info!("Aborted active BREP chunked session");
        }
    }

    /// Take the currently-active chunked session, finalize it with whatever
    /// faces have been processed so far, and return the partial mesh + face
    /// infos.
    ///
    /// This is called by the viewer when the global loading timeout fires
    /// (120s on mobile, 300s on desktop). Without this method, the active
    /// BREP session would be silently dropped on timeout — the user would
    /// see "4 of 5 instances" loaded but the 5th instance would simply
    /// vanish ("исчез один элемент сборки" — user-reported regression).
    ///
    /// Returns `None` if there is no active session, or if the session
    /// produced 0 triangles (e.g., it was aborted before the first face
    /// completed). In either case the caller should log a warning.
    ///
    /// Returns `Some((mesh, faces, faces_done, faces_total))` if the session
    /// had any progress. The caller is responsible for building a
    /// `DetailedMeshInstance` from the mesh via `build_instance`-equivalent
    /// logic (apply transform + decimation).
    pub fn take_partial_active_session(
        &mut self,
        pending: &PendingBrepInstance,
    ) -> Option<(TriangleMesh, Vec<FaceInfo>, usize, usize)> {
        let session_box = self.active_session.take()?;
        let session = *session_box;
        let brep_id = session.brep_id;
        let faces_done = session.next_face_idx;
        let faces_total = session.face_data_list.len();

        log::warn!(
            "BREP #{}: taking PARTIAL session — {}/{} faces processed ({}% complete)",
            brep_id, faces_done, faces_total,
            if faces_total > 0 { (faces_done * 100) / faces_total } else { 0 }
        );

        if faces_done == 0 {
            log::warn!(
                "BREP #{}: partial session had 0 faces completed — nothing to return",
                brep_id
            );
            return None;
        }

        // Build a lightweight converter for finalize (which uses it only for
        // _converter param — currently unused inside finalize).
        let converter = StepConverter::from_cached_maps(
            &self.step_file,
            self.config.clone(),
            self.entity_map.clone(),
            self.pd_brep_map.clone(),
            self.nauo_transform_map.clone(),
        );

        let (mut mesh, faces) = session.finalize(&converter);

        // Apply the instance transform (same as build_instance).
        if let Some(ref tf) = pending.transform {
            mesh.transform(tf);
        }

        // Apply post-triangulation decimation (same as build_instance).
        // Skip when adaptive LOD is enabled — each face already respects
        // its per-face budget, so global decimation is unnecessary.
        if !self.params.adaptive_lod_enabled && self.params.keep_ratio < 1.0 && mesh.triangle_count() >= 4 {
            let (orig, final_) = draper_mesh::decimate_mesh(&mut mesh, self.params.keep_ratio);
            if final_ < orig {
                log::info!(
                    "BREP #{} (partial) — decimated {}→{} triangles (keep_ratio={:.2})",
                    brep_id, orig, final_, self.params.keep_ratio
                );
            }
        }

        if mesh.triangle_count() == 0 {
            log::warn!(
                "BREP #{}: partial session produced 0 triangles after finalize — skipping",
                brep_id
            );
            return None;
        }

        log::info!(
            "BREP #{}: partial session finalized — {} verts, {} tris, {} faces",
            brep_id, mesh.vertex_count(), mesh.triangle_count(), faces.len()
        );

        Some((mesh, faces, faces_done, faces_total))
    }
}

impl BrepSession {
    /// Process exactly one face. The caller is responsible for time-budget
    /// checks between faces. This advances `next_face_idx` by 1.
    ///
    /// Mirrors the body of the face loop in `triangulate_brep_detailed`.
    fn process_one_face(&mut self, converter: &StepConverter) {
        let fi = self.next_face_idx;
        if fi >= self.face_data_list.len() {
            return;
        }
        self.next_face_idx += 1;

        let face_data = &self.face_data_list[fi];
        let brep_id = self.brep_id;
        let face_id = self.next_face_id;
        self.next_face_id += 1;
        let step_face_id = face_data.step_face_id;

        let surface_type = match &face_data.surface {
            Surface::Plane(_) => "Plane".to_string(),
            Surface::Cylinder(_) => "Cylinder".to_string(),
            Surface::Cone(_) => "Cone".to_string(),
            Surface::Sphere(_) => "Sphere".to_string(),
            Surface::Torus(_) => "Torus".to_string(),
            Surface::Revolution(_) => "Revolution".to_string(),
            Surface::Extrusion(_) => "Extrusion".to_string(),
            Surface::Nurbs(n) => {
                format!("Nurbs(deg={}/{}, cps={}x{})",
                    n.u_degree, n.v_degree, n.control_points.len(),
                    n.control_points.first().map(|r| r.len()).unwrap_or(0))
            }
        };

        let tri_start = self.mesh.triangle_count();
        let face_mesh = converter.surface_to_mesh_cached(face_data, &self.params, &self.bbox, &mut self.edge_cache);

        let face_tri_count = face_mesh.triangle_count();
        if face_tri_count == 0 {
            log::warn!(
                "BREP #{} face #{} (STEP #{}, type={}): produced 0 triangles — triangulation may have failed",
                brep_id, face_id, step_face_id, surface_type
            );
        }

        let mut face_mesh_with_ids = face_mesh.clone();
        face_mesh_with_ids.triangle_face_ids = Some(vec![face_id; face_tri_count]);

        self.mesh.merge_deduplicating(&face_mesh_with_ids, &mut self.dedup_map);
        self.total_face_vertices += face_mesh_with_ids.vertices.len();
        let tri_end = self.mesh.triangle_count();

        // Sample boundary edges into polylines (3D and UV)
        let outer_boundary: Vec<Vec<Point3d>> = if face_data.outer_edges.is_empty() {
            vec![]
        } else {
            vec![converter.sample_edges_to_polylines(&face_data.outer_edges)]
        };
        let inner_boundaries: Vec<Vec<Point3d>> = face_data.inner_edges.iter()
            .map(|edges| converter.sample_edges_to_polylines(edges))
            .collect();

        let outer_uv_boundary = converter.sample_edges_to_uv_polylines(&face_data.outer_edges, &face_data.surface);
        let inner_uv_boundaries: Vec<Vec<Vec<Point2d>>> = face_data.inner_edges.iter()
            .map(|edges| converter.sample_edges_to_uv_polylines(edges, &face_data.surface))
            .collect();

        // Project each triangle's vertices to UV space for visualization
        let surface_ref = &face_data.surface;
        let uv_triangles: Vec<[Point2d; 3]> = face_mesh_with_ids.triangles.iter()
            .map(|tri| {
                let v0 = face_mesh_with_ids.vertices[tri[0] as usize];
                let v1 = face_mesh_with_ids.vertices[tri[1] as usize];
                let v2 = face_mesh_with_ids.vertices[tri[2] as usize];
                let (u0, v0v) = surface_ref.project_point(&v0);
                let (u1, v1v) = surface_ref.project_point(&v1);
                let (u2, v2v) = surface_ref.project_point(&v2);
                [
                    Point2d::new(u0, v0v),
                    Point2d::new(u1, v1v),
                    Point2d::new(u2, v2v),
                ]
            })
            .collect();

        self.face_infos.push(FaceInfo {
            face_id,
            step_face_id,
            surface_type,
            surface: face_data.surface.clone(),
            outer_boundary,
            inner_boundaries,
            outer_uv_boundary,
            inner_uv_boundaries,
            triangle_range: (tri_start, tri_end),
            forward: face_data.forward,
            uv_triangles,
            is_void: face_data.is_void,
        });
    }

    /// Finalize the session: run post-processing (filter, weld, validate,
    /// smooth normals). Mirrors the post-face-loop code in `triangulate_brep_detailed`.
    fn finalize(mut self, _converter: &StepConverter) -> (TriangleMesh, Vec<FaceInfo>) {
        let brep_id = self.brep_id;

        // Filter degenerate triangles
        filter_degenerate_triangles(&mut self.mesh, 1e-10);

        // ─── Recompute face_infos.triangle_range from triangle_face_ids ───
        // After filter_degenerate_triangles removes triangles, the original
        // triangle_range values are stale. Recompute from surviving ids.
        if let Some(ref fids) = self.mesh.triangle_face_ids {
            let mut fid_ranges: std::collections::HashMap<u64, (usize, usize)> = std::collections::HashMap::new();
            for (ti, &fid) in fids.iter().enumerate() {
                let entry = fid_ranges.entry(fid).or_insert((ti, ti));
                entry.1 = ti + 1;
            }
            for fi in &mut self.face_infos {
                if let Some(&(start, end)) = fid_ranges.get(&fi.face_id) {
                    fi.triangle_range = (start, end);
                } else {
                    fi.triangle_range = (0, 0);
                }
            }
        }

        // Weld boundary edge vertices
        {
            // Check watertightness BEFORE welding — if the mesh is already
            // watertight via edge cache + merge_deduplicating, skip welding
            // to avoid creating degenerate triangles (Step#87/#78 lose 101/50
            // triangles when welding collapses boundary vertices).
            let report_before = validate_watertight(&self.mesh, false);
            if report_before.is_watertight() {
                eprintln!("WELD_SKIP: BREP #{} already watertight ({} interior edges) — skipping weld to preserve triangles",
                    brep_id, report_before.interior_edge_count);
            } else {
                let weld_tol = (self.tol_ctx.model_scale * 3e-2).min(10.0).max(1e-4);
                weld_boundary_edge_vertices(&mut self.mesh, weld_tol);
            }
        }

        // Remove duplicate triangles
        let dup_removed = self.mesh.remove_duplicate_triangles();
        if dup_removed > 0 {
            log::info!(
                "BREP #{} detailed (chunked): removed {} duplicate/degenerate triangles ({} → {})",
                brep_id, dup_removed, self.mesh.triangle_count() + dup_removed, self.mesh.triangle_count(),
            );
        }

        // Recompute triangle_range after remove_duplicate_triangles (same fix as
        // the non-chunked path — see comment there for rationale).
        if dup_removed > 0 {
            if let Some(ref fids) = self.mesh.triangle_face_ids {
                let mut fid_ranges: std::collections::HashMap<u64, (usize, usize)> = std::collections::HashMap::new();
                for (ti, &fid) in fids.iter().enumerate() {
                    let entry = fid_ranges.entry(fid).or_insert((ti, ti));
                    entry.1 = ti + 1;
                }
                for fi in &mut self.face_infos {
                    if let Some(&(start, end)) = fid_ranges.get(&fi.face_id) {
                        fi.triangle_range = (start, end);
                    } else {
                        fi.triangle_range = (0, 0);
                    }
                }
            }
        }

        // Validate watertightness (logging only)
        let adaptive_tol = self.edge_cache.adaptive_tolerance().merge_tolerance();
        let report_before = validate_watertight(&self.mesh, false);
        if !report_before.is_watertight() {
            let boundary_pct = if report_before.edge_count > 0 {
                report_before.boundary_edge_count as f64 / report_before.edge_count as f64 * 100.0
            } else {
                0.0
            };
            log::error!(
                "BUG: BREP #{} detailed (chunked) not watertight: {} boundary edges ({:.2}%), {} non-manifold (tol={:.2e})",
                brep_id, report_before.boundary_edge_count, boundary_pct,
                report_before.non_manifold_edge_count, adaptive_tol
            );
            let deduped = self.total_face_vertices - self.mesh.vertices.len();
            let (exact_hits, tolerance_hits, misses) = self.dedup_map.stats();
            log::error!(
                "  Dedup stats: {} face vertices → {} unique ({} shared), dedup_rate={:.1}%, exact_hits={}, tolerance_hits={}, misses={}",
                self.total_face_vertices, self.mesh.vertices.len(), deduped,
                if self.total_face_vertices > 0 { deduped as f64 / self.total_face_vertices as f64 * 100.0 } else { 0.0 },
                exact_hits, tolerance_hits, misses,
            );
        }

        // Smooth vertex normals
        draper_mesh::smooth_normals(&mut self.mesh, 0.785);

        if self.skipped_faces > 0 {
            log::warn!("BREP #{} detailed (chunked): {} faces skipped due to time limit", brep_id, self.skipped_faces);
        }

        // Final watertightness check
        let wt_report = validate_watertight(&self.mesh, false);
        if wt_report.is_watertight() {
            log::info!("BREP #{} detailed (chunked): watertight ✓ ({} interior edges, Euler χ={})",
                brep_id, wt_report.interior_edge_count, wt_report.euler_characteristic);
        } else {
            log::warn!("BREP #{} detailed (chunked): NOT watertight — {} boundary edges, {} non-manifold edges, χ={}",
                brep_id, wt_report.boundary_edge_count, wt_report.non_manifold_edge_count,
                wt_report.euler_characteristic);
        }

        log::info!("BREP #{} detailed (chunked): edge_cache={} entries, mesh v={} t={} skipped={}",
            brep_id, self.edge_cache.len(), self.mesh.vertex_count(), self.mesh.triangle_count(), self.skipped_faces);

        (self.mesh, self.face_infos)
    }
}

pub fn triangulate_pending_instance(
    step_file: &StepFile,
    pending: &PendingBrepInstance,
) -> Option<DetailedMeshInstance> {
    let ctx = StepConversionContext::new(step_file);
    ctx.triangulate_pending(pending)
}

/// Assign instance_index to leaf AssemblyNodes for pending instances.
/// Uses simple sequential matching since both tree leaves and instances
/// are produced in the same DFS order.
fn assign_instance_indices_pending(root: &mut AssemblyNode, instances: &[PendingBrepInstance], _next_index: &mut usize) {
    let mut leaves: Vec<*mut AssemblyNode> = Vec::new();
    {
        let mut stack: Vec<*mut AssemblyNode> = vec![root as *mut AssemblyNode];
        while let Some(node_ptr) = stack.pop() {
            let node = unsafe { &mut *node_ptr };
            if node.children.is_empty() {
                leaves.push(node_ptr);
            } else {
                for child in node.children.iter_mut().rev() {
                    stack.push(child as *mut AssemblyNode);
                }
            }
        }
    }

    // Simple 1:1 sequential assignment — both leaves and instances are in DFS order
    for (i, leaf_ptr) in leaves.iter().enumerate() {
        let leaf = unsafe { &mut **leaf_ptr };
        if leaf.brep_id.is_some() && i < instances.len() {
            leaf.instance_index = Some(i);
        }
    }
}

/// Assign instance_index to leaf AssemblyNodes by sequential DFS position.
///
/// Both tree leaves (from `build_assembly_node_iterative`) and instances
/// (from `walk_assembly_tree_detailed`) are produced in the same DFS order,
/// so we can use simple 1:1 sequential matching instead of brep_id matching.
fn assign_instance_indices(root: &mut AssemblyNode, instances: &[DetailedMeshInstance], _next_index: &mut usize) {
    // Collect tree leaves in DFS order (leftmost child first)
    let mut leaves: Vec<*mut AssemblyNode> = Vec::new();
    {
        let mut stack: Vec<*mut AssemblyNode> = vec![root as *mut AssemblyNode];
        while let Some(node_ptr) = stack.pop() {
            let node = unsafe { &mut *node_ptr };
            if node.children.is_empty() {
                leaves.push(node_ptr);
            } else {
                // Push children in reverse order so leftmost is processed first
                for child in node.children.iter_mut().rev() {
                    stack.push(child as *mut AssemblyNode);
                }
            }
        }
    }

    // Simple 1:1 sequential assignment — both leaves and instances are in DFS order
    for (i, leaf_ptr) in leaves.iter().enumerate() {
        let leaf = unsafe { &mut **leaf_ptr };
        if leaf.brep_id.is_some() && i < instances.len() {
            leaf.instance_index = Some(i);
        }
    }
}

/// Get a detailed text dump of the STEP file structure, including:
/// - All NAUO (assembly) relationships with transforms
/// - All PD -> BREP mappings
/// - The full assembly tree
/// - The mesh rendering tree (which BREPs are drawn how many times with which transforms)
pub fn step_structure_detailed(step_file: &StepFile) -> String {
    let converter = StepConverter::new(step_file);
    converter.build_detailed_structure()
}

/// Extract tolerance values from STEP file entities.
///
/// STEP files can contain explicit tolerance information via:
/// - `UNCERTAINTY_MEASURE_WITH_UNIT` — overall model uncertainty
/// - `GEOMETRIC_TOLERANCE` / `GEOMETRIC_TOLERANCE_WITH_DATUM_REFERENCE` — GD&T tolerances
/// - `SHAPE_TOLERANCE` / `SHAPE_TOLERANCE_WITH_DATUM_REFERENCE` — shape-level tolerances
///
/// Returns an `Option<f64>` with the best available tolerance value.
/// If multiple tolerance values are found, the smallest (tightest) is returned.
/// If no tolerance information is found, returns `None`.
pub fn extract_step_tolerance(step_file: &StepFile) -> Option<f64> {
    let mut best_tolerance: Option<f64> = None;

    for entity in &step_file.entities {
        let type_name = entity.type_name.to_uppercase();

        if type_name == "UNCERTAINTY_MEASURE_WITH_UNIT" {
            // Format: UNCERTAINTY_MEASURE_WITH_UNIT(name, measure_with_unit)
            // The first parameter is typically the uncertainty value
            if let Some(StepValue::List(params)) = entity.params.get(0) {
                if let Some(StepValue::Float(val)) = params.first() {
                    let tol = val.abs();
                    best_tolerance = Some(match best_tolerance {
                        Some(existing) => existing.min(tol),
                        None => tol,
                    });
                }
            }
            // Also try the second parameter pattern
            if let Some(StepValue::Float(val)) = entity.params.get(0) {
                let tol = val.abs();
                best_tolerance = Some(match best_tolerance {
                    Some(existing) => existing.min(tol),
                    None => tol,
                });
            }
        }

        if type_name.starts_with("GEOMETRIC_TOLERANCE") || type_name.starts_with("SHAPE_TOLERANCE") {
            // Geometric tolerances have the tolerance value as one of the parameters
            // The exact position depends on the specific subtype, but typically
            // the first numeric parameter is the tolerance value
            for param in &entity.params {
                if let StepValue::Float(val) = param {
                    let tol = val.abs();
                    if tol > 1e-15 && tol < 1000.0 {
                        // Sanity check: tolerance should be small positive number
                        best_tolerance = Some(match best_tolerance {
                            Some(existing) => existing.min(tol),
                            None => tol,
                        });
                    }
                    break; // Only use the first numeric parameter
                }
            }
        }
    }

    best_tolerance
}

/// Get a text representation of the STEP file structure.
pub fn step_structure_text(step_file: &StepFile) -> String {
    let converter = StepConverter::new(step_file);
    let tree = converter.build_assembly_tree();
    let mut text = String::new();
    format_assembly_node(&tree, 0, &mut text);
    text
}

fn format_assembly_node(node: &AssemblyNode, depth: usize, out: &mut String) {
    let indent = "  ".repeat(depth);
    let brep_str = match node.brep_id {
        Some(id) => format!(" BREP=#{}", id),
        None => String::new(),
    };
    let color_str = match node.color {
        Some(c) => format!(" color=({:.2},{:.2},{:.2})", c[0], c[1], c[2]),
        None => String::new(),
    };
    let tf_str = match node.transform {
        Some(_) => " [T]".to_string(),
        None => String::new(),
    };
    out.push_str(&format!("{}{} (PD=#{}){}{}{}\n", indent, node.name, node.pd_id, brep_str, color_str, tf_str));
    for child in &node.children {
        format_assembly_node(child, depth + 1, out);
    }
}

/// Format assembly node with detailed transform information.
fn format_assembly_node_detailed(node: &AssemblyNode, depth: usize, out: &mut String) {
    let indent = "  ".repeat(depth);
    let node_type = if node.brep_id.is_some() { "part" } else if !node.children.is_empty() { "assembly" } else { "empty" };
    let brep_str = match node.brep_id {
        Some(id) => format!(" BREP=#{}", id),
        None => String::new(),
    };
    let color_str = match node.color {
        Some(c) => format!(" color=({:.2},{:.2},{:.2})", c[0], c[1], c[2]),
        None => String::new(),
    };
    let tf_str = match node.transform {
        Some(tf) => {
            let tx = tf[0][3]; let ty = tf[1][3]; let tz = tf[2][3];
            if tx.abs() < 1e-10 && ty.abs() < 1e-10 && tz.abs() < 1e-10 {
                " [rotation]".to_string()
            } else {
                format!(" [T:({:.1},{:.1},{:.1})]", tx, ty, tz)
            }
        }
        None => String::new(),
    };
    out.push_str(&format!(
        "{}{} [{}] (PD=#{}){}{}{}\n",
        indent, node.name, node_type, node.pd_id, brep_str, color_str, tf_str
    ));
    for child in &node.children {
        format_assembly_node_detailed(child, depth + 1, out);
    }
}

struct StepConverter<'a> {
    step: &'a StepFile,
    _entity_map: HashMap<i64, usize>,
    config: StepConversionConfig,
    /// Whether the STEP file uses degrees for plane angle measures.
    /// Determined from HEADER units via `extract_units()`. Affects
    /// CONICAL_SURFACE semi_angle interpretation: if degrees, the raw
    /// value must be converted to radians; if radians (SI default),
    /// use as-is.
    angle_in_degrees: bool,
    /// Pre-built index: pd_id → brep_id (resolved eagerly in new() — O(n) instead of O(n³) per-call)
    pd_brep_map: HashMap<i64, Option<i64>>,
    /// Pre-built index: nauo_id → transform (resolved eagerly in new() — O(n) instead of O(n²) per-call)
    nauo_transform_map: HashMap<i64, Option<[[f64; 4]; 4]>>,
    /// Cache: bounding box (computed once, reused across all BREP triangulations)
    bbox_cache: std::cell::RefCell<Option<Option<(Point3d, Point3d)>>>,
}

impl<'a> StepConverter<'a> {
    fn new(step: &'a StepFile) -> Self {
        Self::with_config(step, StepConversionConfig::default())
    }

    fn with_config(step: &'a StepFile, config: StepConversionConfig) -> Self {
        // Reuse the StepFile's entity_index instead of rebuilding from scratch
        let entity_map = step.entity_index_ref().clone();

        // ─── Determine angle unit from HEADER ───
        // STEP files may use RADIAN (SI default) or DEGREE (via CONVERSION_BASED_UNIT).
        // This affects CONICAL_SURFACE semi_angle interpretation: degrees must be
        // converted to radians; radians are used as-is.
        let unit_data = crate::pmi::extract_units(step);
        let angle_in_degrees = unit_data.units.uses_degrees();

        // ─── Use or build reverse-index maps ───
        // These are cached on the StepFile so that multiple StepConverter instances
        // (created per-BREP in progressive triangulation) don't rebuild them.
        // First call builds and caches; subsequent calls reuse the cache.

        // Build pd_brep_map if not cached
        if step.pd_brep_index().is_empty() || (!step.pd_brep_index().is_empty() && step.pd_brep_index().values().all(|v| v.is_none())) {
            let pd_brep_map = Self::build_pd_brep_index(step);
            step.set_pd_brep_index(pd_brep_map);
        }
        let pd_brep_map = step.pd_brep_index().clone();

        // Build nauo_transform_map if not cached
        if step.nauo_transform_index().is_empty() {
            let nauo_transform_map = Self::build_nauo_transform_index(step);
            step.set_nauo_transform_index(nauo_transform_map);
        }
        let nauo_transform_map = step.nauo_transform_index().clone();

        Self {
            step,
            _entity_map: entity_map,
            config,
            angle_in_degrees,
            pd_brep_map,
            nauo_transform_map,
            bbox_cache: std::cell::RefCell::new(None),
        }
    }

    /// Extract all BREP solids from the STEP file without triangulating.
    ///
    /// Each MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS produces one `Solid`.
    /// For BREP_WITH_VOIDS, the void shells are attached as `Solid::inner_shells`.
    ///
    /// Returns `(solids, brep_ids)`. The order corresponds to the order in
    /// which BREP entities appear in the STEP file.
    fn extract_all_solids(&self) -> (Vec<Solid>, Vec<i64>) {
        let mut solids = Vec::new();
        let mut brep_ids = Vec::new();

        // Collect all BREP entity IDs from the STEP file (both MANIFOLD_SOLID_BREP
        // and BREP_WITH_VOIDS).
        let mut all_brep_ids: Vec<i64> = Vec::new();
        for brep in self.step.find_entities_by_type("MANIFOLD_SOLID_BREP") {
            all_brep_ids.push(brep.id);
        }
        for brep in self.step.find_entities_by_type("BREP_WITH_VOIDS") {
            all_brep_ids.push(brep.id);
        }
        all_brep_ids.sort_unstable();

        for brep_id in all_brep_ids {
            match self.extract_solid_from_brep(brep_id) {
                Some(solid) => {
                    solids.push(solid);
                    brep_ids.push(brep_id);
                }
                None => {
                    log::warn!(
                        "extract_all_solids: failed to extract solid from BREP #{} — skipping",
                        brep_id
                    );
                }
            }
        }

        (solids, brep_ids)
    }

    /// Extract a single Solid from a BREP entity (MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS).
    ///
    /// Extracts outer shell + void shells, converts each to FaceData list,
    /// and assembles a `Solid` with outer_shell + inner_shells.
    fn extract_solid_from_brep(&self, brep_id: i64) -> Option<Solid> {
        let (outer_shell_id, void_shell_ids) = self.find_all_shell_refs_by_brep_id(brep_id);
        let outer_shell_id = outer_shell_id?;

        // Extract outer shell faces
        let outer_face_data = self.extract_shell_faces(outer_shell_id, false)?;
        if outer_face_data.is_empty() {
            log::warn!(
                "extract_solid_from_brep: outer shell #{} has no faces — skipping BREP #{}",
                outer_shell_id, brep_id
            );
            return None;
        }

        // Convert FaceData list → Solid (outer shell only, no healing applied)
        let (mut solid, _) = face_data_list_to_solid(&outer_face_data);

        // Extract void shells (if any) and add as inner shells
        for void_shell_id in &void_shell_ids {
            match self.extract_shell_faces(*void_shell_id, true) {
                Some(void_face_data) => {
                    if void_face_data.is_empty() {
                        continue;
                    }
                    let (void_solid, _) = face_data_list_to_solid(&void_face_data);
                    if let Some(void_shell) = void_solid.outer_shell {
                        solid.add_void(void_shell);
                    }
                }
                None => {
                    log::warn!(
                        "extract_solid_from_brep: failed to extract void shell #{} — skipping",
                        void_shell_id
                    );
                }
            }
        }

        Some(solid)
    }

    /// Create a StepConverter from pre-built index maps.
    ///
    /// This is used by `OwnedStepConversionContext::triangulate_pending()` to
    /// avoid rebuilding the entity_map, pd_brep_map, and nauo_transform_map
    /// on every BREP instance. The maps are cached once in the context and
    /// cloned here (which is much cheaper than rebuilding from scratch).
    fn from_cached_maps(
        step: &'a StepFile,
        config: StepConversionConfig,
        entity_map: HashMap<i64, usize>,
        pd_brep_map: HashMap<i64, Option<i64>>,
        nauo_transform_map: HashMap<i64, Option<[[f64; 4]; 4]>>,
    ) -> Self {
        // Determine angle unit from HEADER (same logic as with_config)
        let unit_data = crate::pmi::extract_units(step);
        let angle_in_degrees = unit_data.units.uses_degrees();

        Self {
            step,
            _entity_map: entity_map,
            config,
            angle_in_degrees,
            pd_brep_map,
            nauo_transform_map,
            bbox_cache: std::cell::RefCell::new(None),
        }
    }

    /// Build a complete pd_id → brep_id index in a single O(n) pass.
    ///
    /// This replaces the per-call `find_pd_brep_uncached()` which was O(n³)
    /// (triple-nested loop: PDS × SDR × params). By pre-building the
    /// reverse indices (pds_id → pd_id, pds_id → sdr_ids, sdr_id → sr_ids)
    /// and resolving all BREP lookups upfront, we turn O(n³ × m) into O(n).
    fn build_pd_brep_index(step: &StepFile) -> HashMap<i64, Option<i64>> {
        // Phase 1: Build reverse index: pd_id → pds_ids
        let mut pd_to_pds: HashMap<i64, Vec<i64>> = HashMap::new();
        for pds in step.find_entities_by_type("PRODUCT_DEFINITION_SHAPE") {
            for param in &pds.params {
                if let Some(pd_id) = get_ref_standalone(param) {
                    pd_to_pds.entry(pd_id).or_default().push(pds.id);
                }
            }
        }

        // Phase 2: Build reverse index: pds_id → sdr_ids
        let mut pds_to_sdrs: HashMap<i64, Vec<i64>> = HashMap::new();
        for sdr in step.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION") {
            if let Some(pds_id) = sdr.params.first().and_then(|p| get_ref_standalone(p)) {
                pds_to_sdrs.entry(pds_id).or_default().push(sdr.id);
            }
        }

        // Phase 3: For each SDR, extract SR IDs and resolve BREP
        let mut sdr_to_brep: HashMap<i64, Option<i64>> = HashMap::new();
        for sdr in step.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION") {
            let mut brep_id: Option<i64> = None;
            for param in &sdr.params {
                if let Some(sr_id) = get_ref_standalone(param) {
                    // Direct: check if SR contains a BREP
                    if let Some(bid) = Self::find_brep_in_representation_static(step, sr_id) {
                        brep_id = Some(bid);
                        break;
                    }
                    // Indirect: follow SRR chain
                    if let Some(bid) = Self::find_brep_via_srr_static(step, sr_id, 0) {
                        brep_id = Some(bid);
                        break;
                    }
                }
            }
            sdr_to_brep.insert(sdr.id, brep_id);
        }

        // Phase 4: Combine: for each PD, find its PDS → SDR → BREP chain
        let mut result: HashMap<i64, Option<i64>> = HashMap::new();
        for (pd_id, pds_ids) in &pd_to_pds {
            let mut found_brep: Option<i64> = None;
            for pds_id in pds_ids {
                if let Some(sdr_ids) = pds_to_sdrs.get(pds_id) {
                    for sdr_id in sdr_ids {
                        if let Some(bid) = sdr_to_brep.get(sdr_id).copied().flatten() {
                            found_brep = Some(bid);
                            break;
                        }
                    }
                }
                if found_brep.is_some() { break; }
            }
            result.insert(*pd_id, found_brep);
        }

        result
    }

    /// Static version of find_brep_in_representation (doesn't need &self).
    fn find_brep_in_representation_static(step: &StepFile, sr_id: i64) -> Option<i64> {
        let sr = step.find_entity(sr_id)?;
        if sr.type_name.contains("ADVANCED_BREP_SHAPE_REPRESENTATION") {
            for sp in &sr.params {
                if let Some(brep_id) = get_ref_standalone(sp) {
                    if let Some(brep) = step.find_entity(brep_id) {
                        if brep.type_name == "MANIFOLD_SOLID_BREP" {
                            return Some(brep_id);
                        }
                    }
                }
                if let StepValue::List(items) = sp {
                    for item in items {
                        if let Some(brep_id) = get_ref_standalone(item) {
                            if let Some(brep) = step.find_entity(brep_id) {
                                if brep.type_name == "MANIFOLD_SOLID_BREP" {
                                    return Some(brep_id);
                                }
                            }
                        }
                    }
                }
            }
        }
        // Also check FACETED_BREP
        if sr.type_name.contains("FACETED_BREP_SHAPE_REPRESENTATION") {
            for sp in &sr.params {
                if let Some(brep_id) = get_ref_standalone(sp) {
                    if let Some(brep) = step.find_entity(brep_id) {
                        if brep.type_name == "FACETED_BREP" {
                            return Some(brep_id);
                        }
                    }
                }
            }
        }
        None
    }

    /// Static version of find_brep_via_srr (doesn't need &self).
    /// Uses the same priority-based strategy as the instance method:
    /// 1. SRRs whose other end is an ADVANCED_BREP_SHAPE_REPRESENTATION (direct link to BREP)
    /// 2. SRRs whose other end is a plain SHAPE_REPRESENTATION (indirect, recurse)
    ///
    /// This avoids the bug where assembly-placement SRRs (complex entities with transforms)
    /// are followed instead of the direct SR→ABSR link, causing all parts to map to the same BREP.
    fn find_brep_via_srr_static(step: &StepFile, sr_id: i64, _depth: usize) -> Option<i64> {
        let mut visited: std::collections::HashSet<i64> = std::collections::HashSet::new();
        let mut stack = vec![(sr_id, 0usize)]; // (sr_id, depth)

        while let Some((current_sr_id, depth)) = stack.pop() {
            if depth > 20 { continue; }
            if !visited.insert(current_sr_id) { continue; }

            let sr = match step.find_entity(current_sr_id) {
                Some(s) => s,
                None => continue,
            };
            if !sr.type_name.contains("SHAPE_REPRESENTATION") { continue; }

            // Collect SRR relationships into priority buckets
            let mut direct_absr_links: Vec<i64> = Vec::new();  // other SR is an ABSR/FBSR
            let mut indirect_sr_links: Vec<i64> = Vec::new();   // other SR is a plain SR

            // Helper closure to classify SRRs
            let mut classify_srr = |srr: &crate::schema::StepEntity| {
                let mut refs_our_sr = false;
                let mut other_sr_id: Option<i64> = None;
                for (i, param) in srr.params.iter().enumerate() {
                    if let Some(ref_id) = get_ref_standalone(param) {
                        if ref_id == current_sr_id {
                            refs_our_sr = true;
                        } else if i >= 2 {
                            if let Some(entity) = step.find_entity(ref_id) {
                                if entity.type_name.contains("SHAPE_REPRESENTATION") {
                                    other_sr_id = Some(ref_id);
                                }
                            }
                        }
                    }
                }
                if !refs_our_sr { return; }
                if let Some(other_id) = other_sr_id {
                    if let Some(other_entity) = step.find_entity(other_id) {
                        if other_entity.type_name.contains("ADVANCED_BREP_SHAPE_REPRESENTATION")
                            || other_entity.type_name.contains("FACETED_BREP_SHAPE_REPRESENTATION") {
                            direct_absr_links.push(other_id);
                        } else {
                            indirect_sr_links.push(other_id);
                        }
                    }
                }
            };

            // Check SHAPE_REPRESENTATION_RELATIONSHIP entities
            for srr in step.find_entities_by_type("SHAPE_REPRESENTATION_RELATIONSHIP") {
                classify_srr(srr);
            }

            // Also check REPRESENTATION_RELATIONSHIP entities (may catch additional complex entities)
            for srr in step.find_entities_by_type("REPRESENTATION_RELATIONSHIP") {
                if srr.type_name.contains("SHAPE_REPRESENTATION_RELATIONSHIP") { continue; }
                classify_srr(srr);
            }

            // Priority 1: try direct ABSR links
            for absr_id in &direct_absr_links {
                if let Some(brep_id) = Self::find_brep_in_representation_static(step, *absr_id) {
                    return Some(brep_id);
                }
            }

            // Priority 2: add indirect SR links to stack (process in next iterations)
            for other_id in indirect_sr_links {
                stack.push((other_id, depth + 1));
            }
        }
        None
    }

    /// Build a complete nauo_id → transform index in a single pass.
    ///
    /// This replaces `find_nauo_transform_uncached()` which was O(n²) per NAUO.
    fn build_nauo_transform_index(step: &StepFile) -> HashMap<i64, Option<[[f64; 4]; 4]>> {
        let mut result: HashMap<i64, Option<[[f64; 4]; 4]>> = HashMap::new();

        // Build a reverse index: pds_id → nauo_id (which PDS belongs to which NAUO)
        let nauos = step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if nauos.is_empty() { return result; }

        // Build: nauo_id → the NAUO entity itself is already accessible via find_entity
        // Build: pds_id → set of nauo_ids that this PDS references
        //
        // Two strategies:
        //   1) Some NAUOs explicitly reference a PDS in their 6th parameter
        //      (NAUO params: id, name, description, #relating_pd, #related_pd, #pds_or_$).
        //   2) More commonly, the PDS's definition (3rd param) references the NAUO.
        //      E.g. PRODUCT_DEFINITION_SHAPE('Placement','Placement of an item',#751)
        //      where #751 is a NEXT_ASSEMBLY_USAGE_OCCURRENCE.
        // We must use strategy 2 because many STEP files (including the CAx-IF
        // as1-oc-214 reference file) use $ for the NAUO's 6th param.
        let nauo_id_set: std::collections::HashSet<i64> = nauos.iter().map(|n| n.id).collect();
        let mut pds_to_nauos: HashMap<i64, Vec<i64>> = HashMap::new();

        // Strategy 1: NAUO → PDS (works when NAUO's 6th param is not $)
        for nauo in &nauos {
            for param in nauo.params.iter().skip(5) {
                if let Some(pds_id) = get_ref_standalone(param) {
                    pds_to_nauos.entry(pds_id).or_default().push(nauo.id);
                }
            }
        }

        // Strategy 2: PDS → NAUO (works when PDS's definition references a NAUO)
        for pds in step.find_entities_by_type("PRODUCT_DEFINITION_SHAPE") {
            for param in &pds.params {
                if let Some(ref_id) = get_ref_standalone(param) {
                    if nauo_id_set.contains(&ref_id) {
                        pds_to_nauos.entry(pds.id).or_default().push(ref_id);
                    }
                }
            }
        }

        // Build index: PD → representation_id via SHAPE_DEFINITION_REPRESENTATION
        // This is needed to determine which representation in the SRR belongs to
        // the parent (relating) PD vs the child (related) PD.
        let pd_to_repr = Self::build_pd_to_representation_index(step);

        // Build index: nauo_id → (relating_pd, related_pd)
        let mut nauo_to_pd_refs: HashMap<i64, (i64, i64)> = HashMap::new();
        for nauo in &nauos {
            let (relating_pd, related_pd) = extract_nauo_pd_refs_static(step, nauo);
            if let (Some(rp), Some(rd)) = (relating_pd, related_pd) {
                nauo_to_pd_refs.insert(nauo.id, (rp, rd));
            }
        }

        // Now walk CDSR entities and match them to NAUOs
        for cdsr in step.find_entities_by_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") {
            // CDSR params: #srr_or_repr_rel, #pds
            // The PDS param links to the NAUO
            let mut srr_id: Option<i64> = None;
            let mut pds_id: Option<i64> = None;

            for (i, param) in cdsr.params.iter().enumerate() {
                if let Some(ref_id) = get_ref_standalone(param) {
                    if i == 0 {
                        srr_id = Some(ref_id);
                    } else {
                        pds_id = Some(ref_id);
                    }
                }
            }

            let srr_id = match srr_id {
                Some(id) => id,
                None => continue,
            };
            let pds_id = match pds_id {
                Some(id) => id,
                None => continue,
            };

            // Which NAUO(s) does this CDSR's PDS belong to?
            let matched_nauo_ids = match pds_to_nauos.get(&pds_id) {
                Some(ids) => ids,
                None => continue,
            };

            // Extract transform from SRR
            let srr_entity = match step.find_entity(srr_id) {
                Some(e) => e,
                None => continue,
            };

            // Determine the transform direction by checking which SRR representation
            // belongs to the parent (relating) PD.
            //
            // The ITEM_DEFINED_TRANSFORMATION defines:
            //   transform_item_1 (o) is in rep_1's coordinate space
            //   transform_item_2 (t) is in rep_2's coordinate space
            //
            // The formula M = o * t⁻¹ maps from rep_2 → rep_1.
            //
            // For assembly placement, we need the transform that maps from
            // the CHILD's coordinate space to the PARENT's coordinate space.
            //
            // CAx-IF convention (most STEP files):
            //   rep_1 = child component, rep_2 = parent assembly
            //   M = o * t⁻¹ maps parent → child (WRONG direction)
            //   We need M⁻¹ = t * o⁻¹ (child → parent)
            //
            // Opposite convention (some STEP exporters):
            //   rep_1 = parent assembly, rep_2 = child component
            //   M = o * t⁻¹ maps child → parent (CORRECT direction)
            //
            // We detect which convention is used by checking which representation
            // in the SRR belongs to the parent PD.
            let needs_inversion = Self::srr_needs_transform_inversion(
                step, &srr_entity, matched_nauo_ids, &nauo_to_pd_refs, &pd_to_repr,
            );

            let raw_transform = Self::extract_transform_from_srr_static(step, &srr_entity);

            let transform = match (raw_transform, needs_inversion) {
                (Some(tf), true) => mat4_inverse(&tf),
                (Some(tf), false) => Some(tf),
                (None, _) => None,
            };

            // Assign to all NAUOs that reference this PDS
            for nauo_id in matched_nauo_ids {
                result.insert(*nauo_id, transform);
            }
        }

        // Ensure all NAUOs have an entry (even if no CDSR found → None transform)
        for nauo in &nauos {
            result.entry(nauo.id).or_insert(None);
        }

        // Debug logging
        let with_transform = result.values().filter(|v| v.is_some()).count();
        let total = result.len();
        log::info!("NAUO transform index: {}/{} NAUOs have transforms, pds_to_nauos has {} entries",
            with_transform, total, pds_to_nauos.len());

        result
    }

    /// Build an index: PD_id → representation_id via SHAPE_DEFINITION_REPRESENTATION.
    ///
    /// Each PRODUCT_DEFINITION links to a PRODUCT_DEFINITION_SHAPE, which links to
    /// a SHAPE_DEFINITION_REPRESENTATION, which references a SHAPE_REPRESENTATION.
    /// This index allows us to determine which representation belongs to which PD.
    fn build_pd_to_representation_index(step: &StepFile) -> HashMap<i64, i64> {
        let mut pd_to_pds: HashMap<i64, Vec<i64>> = HashMap::new();
        for pds in step.find_entities_by_type("PRODUCT_DEFINITION_SHAPE") {
            for param in &pds.params {
                if let Some(pd_id) = get_ref_standalone(param) {
                    if let Some(entity) = step.find_entity(pd_id) {
                        if entity.type_name == "PRODUCT_DEFINITION" {
                            pd_to_pds.entry(pd_id).or_default().push(pds.id);
                        }
                    }
                }
            }
        }

        let mut pds_to_repr: HashMap<i64, i64> = HashMap::new();
        for sdr in step.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION") {
            if let Some(pds_id) = sdr.params.first().and_then(|p| get_ref_standalone(p)) {
                // The second parameter of SDR is the representation
                if let Some(repr_id) = sdr.params.get(1).and_then(|p| get_ref_standalone(p)) {
                    pds_to_repr.insert(pds_id, repr_id);
                }
            }
        }

        let mut result: HashMap<i64, i64> = HashMap::new();
        for (pd_id, pds_ids) in &pd_to_pds {
            for pds_id in pds_ids {
                if let Some(repr_id) = pds_to_repr.get(pds_id) {
                    result.insert(*pd_id, *repr_id);
                    break;
                }
            }
        }
        result
    }

    /// Determine whether the computed transform from the SRR needs to be inverted.
    ///
    /// Returns `true` if rep_2 of the SRR belongs to the parent PD (CAx-IF convention),
    /// meaning M = o * t⁻¹ maps in the wrong direction and must be inverted.
    /// Returns `false` if rep_1 belongs to the parent PD, meaning M = o * t⁻¹
    /// already maps child → parent.
    ///
    /// If the direction cannot be determined, defaults to `true` (CAx-IF convention),
    /// which is the most common case.
    fn srr_needs_transform_inversion(
        step: &StepFile,
        srr: &crate::schema::StepEntity,
        nauo_ids: &[i64],
        nauo_to_pd_refs: &HashMap<i64, (i64, i64)>,
        pd_to_repr: &HashMap<i64, i64>,
    ) -> bool {
        // Extract rep_1 and rep_2 from the SRR
        // For a complex entity, the REPRESENTATION_RELATIONSHIP sub-entity has the rep refs.
        // For a simple entity, params are: name, description, rep_1, rep_2
        let rr_sub = if srr.is_complex() {
            srr.find_sub_entity("REPRESENTATION_RELATIONSHIP")
        } else {
            Some(srr)
        };

        let rr = match rr_sub {
            Some(s) => s,
            None => return true, // Default: assume CAx-IF convention
        };

        // Extract representation references (params 2 and 3 of RR: name, desc, rep_1, rep_2)
        let mut repr_refs: Vec<i64> = Vec::new();
        for (i, param) in rr.params.iter().enumerate() {
            if i >= 2 { // Skip name (0) and description (1)
                if let Some(ref_id) = get_ref_standalone(param) {
                    if step.find_entity(ref_id).is_some() {
                        repr_refs.push(ref_id);
                    }
                }
            }
        }

        let rep1_id = match repr_refs.get(0) {
            Some(&id) => id,
            None => return true, // Can't determine direction, assume CAx-IF
        };
        let rep2_id = match repr_refs.get(1) {
            Some(&id) => id,
            None => return true,
        };

        // Get the parent (relating) PD from any of the NAUOs
        let relating_pd = nauo_ids.iter()
            .filter_map(|nid| nauo_to_pd_refs.get(nid).map(|(rp, _)| *rp))
            .next();

        let relating_pd = match relating_pd {
            Some(pd) => pd,
            None => return true, // Can't determine, assume CAx-IF
        };

        // Check which representation belongs to the relating (parent) PD
        if let Some(&parent_repr_id) = pd_to_repr.get(&relating_pd) {
            if parent_repr_id == rep1_id {
                // rep_1 = parent → M = o * t⁻¹ maps child → parent (correct)
                log::debug!("SRR #{}: rep_1 is parent → no inversion needed", srr.id);
                return false;
            } else if parent_repr_id == rep2_id {
                // rep_2 = parent → M = o * t⁻¹ maps parent → child (wrong, needs inversion)
                log::debug!("SRR #{}: rep_2 is parent → inversion needed (CAx-IF convention)", srr.id);
                return true;
            }
        }

        // Default: assume CAx-IF convention (rep_1 = child, rep_2 = parent)
        log::debug!("SRR #{}: cannot determine parent repr, assuming CAx-IF (inversion needed)", srr.id);
        true
    }

    /// Static version of extract_transform_from_srr.
    fn extract_transform_from_srr_static(step: &StepFile, srr: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        // Check complex entity with RRWT sub-entity
        if srr.is_complex() {
            if let Some(rrwt_sub) = srr.find_sub_entity("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
                for param in &rrwt_sub.params {
                    if let Some(ref_id) = get_ref_standalone(param) {
                        if let Some(entity) = step.find_entity(ref_id) {
                            if entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                                return Self::compute_item_defined_transform_static(step, &entity);
                            }
                            // Support CARTESIAN_TRANSFORMATION_OPERATOR_3D
                            if entity.type_name == "CARTESIAN_TRANSFORMATION_OPERATOR_3D" {
                                return Self::compute_cartesian_transform_static(step, &entity);
                            }
                        }
                    }
                }
            }
        }

        // Fallback: search params for IDT or CTO3D reference
        for param in &srr.params {
            if let Some(ref_id) = get_ref_standalone(param) {
                if let Some(entity) = step.find_entity(ref_id) {
                    if entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                        return Self::compute_item_defined_transform_static(step, &entity);
                    }
                    if entity.type_name == "CARTESIAN_TRANSFORMATION_OPERATOR_3D" {
                        return Self::compute_cartesian_transform_static(step, &entity);
                    }
                    for inner_param in &entity.params {
                        if let Some(inner_id) = get_ref_standalone(inner_param) {
                            if let Some(inner_entity) = step.find_entity(inner_id) {
                                if inner_entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                                    return Self::compute_item_defined_transform_static(step, &inner_entity);
                                }
                                if inner_entity.type_name == "CARTESIAN_TRANSFORMATION_OPERATOR_3D" {
                                    return Self::compute_cartesian_transform_static(step, &inner_entity);
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Static version of compute_item_defined_transform.
    ///
    /// Computes the "raw" transform from an ITEM_DEFINED_TRANSFORMATION entity.
    /// Returns M = o * t⁻¹, which maps from rep_2 → rep_1 (i.e., from the
    /// "related" representation to the "relating" representation).
    ///
    /// IMPORTANT: The caller must determine whether this raw transform maps
    /// from child → parent or from parent → child, depending on which
    /// representation in the SRR corresponds to the parent PD. The
    /// `build_nauo_transform_index` function handles this by inverting
    /// the result when rep_2 is the parent (CAx-IF convention).
    fn compute_item_defined_transform_static(step: &StepFile, idt: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        let mut axis2_ids: Vec<i64> = Vec::new();
        for (i, param) in idt.params.iter().enumerate() {
            if i < 2 { continue; }
            if let Some(ref_id) = get_ref_standalone(param) {
                if let Some(entity) = step.find_entity(ref_id) {
                    if entity.type_name == "AXIS2_PLACEMENT_3D" {
                        axis2_ids.push(ref_id);
                    }
                }
            }
        }

        if axis2_ids.len() < 2 {
            return None;
        }

        let (origin_pt, origin_z, origin_x) = resolve_axis2_standalone(step, axis2_ids[0])?;
        let (target_pt, target_z, target_x) = resolve_axis2_standalone(step, axis2_ids[1])?;

        let origin_y = origin_z.cross(&origin_x);
        let target_y = target_z.cross(&target_x);

        let o = [
            [origin_x.x, origin_y.x, origin_z.x, origin_pt.x],
            [origin_x.y, origin_y.y, origin_z.y, origin_pt.y],
            [origin_x.z, origin_y.z, origin_z.z, origin_pt.z],
            [0.0, 0.0, 0.0, 1.0],
        ];

        let t = [
            [target_x.x, target_y.x, target_z.x, target_pt.x],
            [target_x.y, target_y.y, target_z.y, target_pt.y],
            [target_x.z, target_y.z, target_z.z, target_pt.z],
            [0.0, 0.0, 0.0, 1.0],
        ];

        // The IDT defines: the coordinate system described by transform_item_1 (o)
        // in rep_1's space is IDENTICAL to the coordinate system described by
        // transform_item_2 (t) in rep_2's space.
        //
        // This formula computes M = o * t⁻¹, which maps from rep_2 → rep_1.
        // The caller (build_nauo_transform_index) determines whether this is
        // child → parent or parent → child based on the SRR/NAUO relationship,
        // and inverts if necessary.
        let t_inv = mat4_inverse(&t)?;
        Some(mat4_mul(&o, &t_inv))
    }

    /// Static version: compute a 4×4 transform from CARTESIAN_TRANSFORMATION_OPERATOR_3D.
    ///
    /// CTO3D directly specifies a coordinate system (origin + local axes) with an
    /// optional scale factor. This is an alternative to ITEM_DEFINED_TRANSFORMATION
    /// used by some STEP exporters.
    ///
    /// STEP definition:
    ///   CARTESIAN_TRANSFORMATION_OPERATOR_3D(
    ///     name,                           -- optional
    ///     origin,                         -- CARTESIAN_POINT
    ///     axis1 : OPTIONAL DIRECTION,     -- local X (u axis)
    ///     axis2 : OPTIONAL DIRECTION,     -- local Y (v axis, in 3D variant)
    ///     scale,                          -- scale factor (default 1.0)
    ///     axis3 : OPTIONAL DIRECTION      -- local Z
    ///   )
    fn compute_cartesian_transform_static(step: &StepFile, cto: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        let mut origin = [0.0_f64; 3];
        let mut axis1: Option<[f64; 3]> = None;
        let mut axis2: Option<[f64; 3]> = None;
        let mut axis3: Option<[f64; 3]> = None;
        let mut scale = 1.0_f64;

        for (i, param) in cto.params.iter().enumerate() {
            match i {
                0 => { /* name — skip */ }
                1 => {
                    // origin — CARTESIAN_POINT reference
                    if let Some(ref_id) = get_ref_standalone(param) {
                        if let Some(cp) = step.find_entity(ref_id) {
                            if let Some(pt) = get_cartesian_point_coords_standalone(&cp) {
                                origin = pt;
                            }
                        }
                    }
                }
                2 => { /* axis1 (u direction) */ axis1 = get_direction_from_param_standalone(step, param); }
                3 => { /* axis2 (v direction) */ axis2 = get_direction_from_param_standalone(step, param); }
                4 => {
                    // scale
                    if let Some(s) = get_float_standalone(param) {
                        scale = s;
                    }
                }
                5 => { /* axis3 (w direction / Z) */ axis3 = get_direction_from_param_standalone(step, param); }
                _ => {}
            }
        }

        // Build orthogonal coordinate frame
        // Z = axis3 (if provided), else [0,0,1]
        let z = axis3.unwrap_or([0.0, 0.0, 1.0]);
        // X = axis1 (if provided), else default perpendicular to Z
        let x_raw = axis1.unwrap_or_else(|| {
            if z[2].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] }
        });
        // Y = axis2 (if provided), else Z × X
        let y_raw = axis2.unwrap_or_else(|| {
            let y = [
                z[1] * x_raw[2] - z[2] * x_raw[1],
                z[2] * x_raw[0] - z[0] * x_raw[2],
                z[0] * x_raw[1] - z[1] * x_raw[0],
            ];
            let len = (y[0]*y[0] + y[1]*y[1] + y[2]*y[2]).sqrt();
            if len > 1e-10 { [y[0]/len, y[1]/len, y[2]/len] } else { [0.0, 1.0, 0.0] }
        });

        // Project X onto plane perpendicular to Z (STEP spec requirement)
        let dot_xz = x_raw[0]*z[0] + x_raw[1]*z[1] + x_raw[2]*z[2];
        let x_proj = [x_raw[0] - dot_xz*z[0], x_raw[1] - dot_xz*z[1], x_raw[2] - dot_xz*z[2]];
        let x_len = (x_proj[0]*x_proj[0] + x_proj[1]*x_proj[1] + x_proj[2]*x_proj[2]).sqrt();
        let x = if x_len > 1e-10 { [x_proj[0]/x_len, x_proj[1]/x_len, x_proj[2]/x_len] } else { x_raw };

        // Recompute Y = Z × X to ensure orthogonality
        let y = [
            z[1]*x[2] - z[2]*x[1],
            z[2]*x[0] - z[0]*x[2],
            z[0]*x[1] - z[1]*x[0],
        ];
        let y_len = (y[0]*y[0] + y[1]*y[1] + y[2]*y[2]).sqrt();
        let y = if y_len > 1e-10 { [y[0]/y_len, y[1]/y_len, y[2]/y_len] } else { y_raw };

        // Normalize Z
        let z_len = (z[0]*z[0] + z[1]*z[1] + z[2]*z[2]).sqrt();
        let z = if z_len > 1e-10 { [z[0]/z_len, z[1]/z_len, z[2]/z_len] } else { z };

        // Build 4×4 matrix with scale
        Some([
            [x[0] * scale, y[0] * scale, z[0] * scale, origin[0]],
            [x[1] * scale, y[1] * scale, z[1] * scale, origin[1]],
            [x[2] * scale, y[2] * scale, z[2] * scale, origin[2]],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    fn convert(&self) -> Result<TriangleMesh, String> {
        let instances = self.convert_instances()?;
        let mut mesh = TriangleMesh::new();
        for inst in &instances {
            if let Some(color) = inst.color {
                mesh.merge_with_color(&inst.mesh, color);
            } else {
                mesh.merge_with_color(&inst.mesh, [0.48, 0.52, 0.58, 1.0]);
            }
        }
        Ok(mesh)
    }

    /// Convert STEP to mesh instances, walking the assembly tree from root.
    /// Each leaf BREP produces one mesh instance per assembly occurrence,
    /// with the composed transform from root → leaf applied.
    fn convert_instances(&self) -> Result<Vec<MeshInstance>, String> {
        let bbox = self.compute_bounding_box();
        let mut params = TriangulationParams::default();
        // Scale max_deviation based on bounding box
        if let Some((bmin, bmax)) = &bbox {
            let dx = bmax.x - bmin.x;
            let dy = bmax.y - bmin.y;
            let dz = bmax.z - bmin.z;
            let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
            if diagonal > 1.0 {
                params.max_deviation = params.max_deviation.max(diagonal * 0.0002);
            }
        }
        let color_map = self.extract_color_map();
        let mut brep_mesh_cache: HashMap<i64, TriangleMesh> = HashMap::new();
        let mut results: Vec<MeshInstance> = Vec::new();

        // ─── Phase 1: Assembly-based conversion via NAUO tree walk ────────
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if !nauos.is_empty() {
            info!("Found {} NAUO assembly instances — walking assembly tree", nauos.len());

            // Build: parent_pd_id → Vec<(nauo_id, child_pd_id, nauo_name)>
            let mut parent_pd_to_children: HashMap<i64, Vec<(i64, i64, String)>> = HashMap::new();
            for nauo in &nauos {
                let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
                if let (Some(parent_pd), Some(child_pd)) = (relating_pd, related_pd) {
                    let name = self.extract_nauo_name(nauo);
                    parent_pd_to_children.entry(parent_pd).or_default().push((nauo.id, child_pd, name));
                }
            }

            // Find root PD(s): PDs that are parents but are never children
            let parent_pds: std::collections::HashSet<i64> = parent_pd_to_children.keys().copied().collect();
            let child_pds: std::collections::HashSet<i64> = nauos.iter()
                .filter_map(|n| self.extract_nauo_pd_refs(n).1)
                .collect();
            let roots: Vec<i64> = parent_pds.difference(&child_pds).copied().collect();

            if roots.is_empty() {
                info!("No root assembly found, falling back to direct BREP conversion");
            } else {
                for root_pd in &roots {
                    let root_name = self.get_product_name(*root_pd);
                    info!("Root assembly: PD=#{} name='{}'", root_pd, root_name);
                    self.walk_assembly_tree(
                        *root_pd,
                        &root_name,
                        &None,
                        &color_map,
                        &mut brep_mesh_cache,
                        &params,
                        &bbox,
                        &parent_pd_to_children,
                        &mut results,
                        &mut std::collections::HashSet::new(),
                    );
                }
            }

            if !results.is_empty() {
                info!("Assembly conversion: {} mesh instances", results.len());
                return Ok(results);
            }
        }

        // ─── Phase 2: No assembly structure — try direct BREP conversion ───
        let breps = self.step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        if !breps.is_empty() {
            for brep in &breps {
                let name = self.get_brep_name(brep.id);
                if let Some(mesh) = self.triangulate_brep_cached(brep.id, &mut brep_mesh_cache, &params, &bbox) {
                    let color = color_map.get(&brep.id).copied();
                    results.push(MeshInstance {
                        name,
                        mesh,
                        color,
                        transform: None,
                        brep_id: brep.id,
                    });
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // FACETED_BREP
        let faceted = self.step.find_entities_by_type("FACETED_BREP");
        if !faceted.is_empty() {
            for fb in &faceted {
                let name = self.get_brep_name(fb.id);
                if let Some(mesh) = self.triangulate_brep_cached(fb.id, &mut brep_mesh_cache, &params, &bbox) {
                    let color = color_map.get(&fb.id).copied();
                    results.push(MeshInstance {
                        name,
                        mesh,
                        color,
                        transform: None,
                        brep_id: fb.id,
                    });
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // ADVANCED_BREP_SHAPE_REPRESENTATION
        let abrep = self.step.find_entities_by_type("ADVANCED_BREP_SHAPE_REPRESENTATION");
        for ab in &abrep {
            for param in &ab.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(entity) = self.step.find_entity(ref_id) {
                        if entity.type_name == "MANIFOLD_SOLID_BREP" {
                            let name = self.get_brep_name(entity.id);
                            if let Some(mesh) = self.triangulate_brep_cached(entity.id, &mut brep_mesh_cache, &params, &bbox) {
                                let color = color_map.get(&entity.id).copied();
                                results.push(MeshInstance {
                                    name,
                                    mesh,
                                    color,
                                    transform: None,
                                    brep_id: entity.id,
                                });
                            }
                        }
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name == "MANIFOLD_SOLID_BREP" {
                                    let name = self.get_brep_name(entity.id);
                                    if let Some(mesh) = self.triangulate_brep_cached(entity.id, &mut brep_mesh_cache, &params, &bbox) {
                                        let color = color_map.get(&entity.id).copied();
                                        results.push(MeshInstance {
                                            name,
                                            mesh,
                                            color,
                                            transform: None,
                                            brep_id: entity.id,
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // ─── Phase 3: SHELL_BASED_SURFACE_MODEL / MANIFOLD_SURFACE_SHAPE_REPRESENTATION ──
        // Some STEP files use surface models instead of solid BREP models
        let shell_models = self.step.find_entities_by_type("SHELL_BASED_SURFACE_MODEL");
        for sm in &shell_models {
            for param in &sm.params {
                // Look for shell references (OPEN_SHELL, CLOSED_SHELL)
                if let Some(shell_id) = self.get_ref(param) {
                    if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                        results.push(MeshInstance {
                            name: format!("ShellModel#{}", sm.id),
                            mesh,
                            color: None,
                            transform: None,
                            brep_id: sm.id,
                        });
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(shell_id) = self.get_ref(item) {
                            if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                                results.push(MeshInstance {
                                    name: format!("ShellModel#{}", sm.id),
                                    mesh,
                                    color: None,
                                    transform: None,
                                    brep_id: sm.id,
                                });
                            }
                        }
                    }
                }
            }
        }

        // Also try MANIFOLD_SURFACE_SHAPE_REPRESENTATION
        let msr = self.step.find_entities_by_type("MANIFOLD_SURFACE_SHAPE_REPRESENTATION");
        for ms in &msr {
            for param in &ms.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(entity) = self.step.find_entity(ref_id) {
                        if entity.type_name.contains("SHELL") {
                            if let Some(mesh) = self.triangulate_shell_by_id(ref_id, &params, &bbox) {
                                results.push(MeshInstance {
                                    name: format!("SurfaceModel#{}", ms.id),
                                    mesh,
                                    color: None,
                                    transform: None,
                                    brep_id: ms.id,
                                });
                            }
                        }
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name.contains("SHELL") {
                                    if let Some(mesh) = self.triangulate_shell_by_id(ref_id, &params, &bbox) {
                                        results.push(MeshInstance {
                                            name: format!("SurfaceModel#{}", ms.id),
                                            mesh,
                                            color: None,
                                            transform: None,
                                            brep_id: ms.id,
                                        });
                                    }
                                } else if entity.type_name.contains("SHELL_BASED_SURFACE_MODEL") {
                                    // Follow the reference chain
                                    for sp in &entity.params {
                                        if let Some(shell_id) = self.get_ref(sp) {
                                            if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                                                results.push(MeshInstance {
                                                    name: format!("SurfaceModel#{}", ms.id),
                                                    mesh,
                                                    color: None,
                                                    transform: None,
                                                    brep_id: ms.id,
                                                });
                                            }
                                        }
                                        if let StepValue::List(inner) = sp {
                                            for inner_item in inner {
                                                if let Some(shell_id) = self.get_ref(inner_item) {
                                                    if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                                                        results.push(MeshInstance {
                                                            name: format!("SurfaceModel#{}", ms.id),
                                                            mesh,
                                                            color: None,
                                                            transform: None,
                                                            brep_id: ms.id,
                                                        });
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // Direct surface extraction fallback
        let surface_types = [
            "PLANE", "CYLINDRICAL_SURFACE", "SPHERICAL_SURFACE",
            "CONICAL_SURFACE", "TOROIDAL_SURFACE", "SURFACE_OF_REVOLUTION",
            "SURFACE_OF_LINEAR_EXTRUSION", "B_SPLINE_SURFACE_WITH_KNOTS",
            "B_SPLINE_SURFACE", "BEZIER_SURFACE",
        ];
        for type_name in &surface_types {
            for entity in self.step.find_entities_by_type(type_name) {
                if let Some(surface) = self.extract_surface(entity.id, 0) {
                    let face_data = FaceData { surface, outer_edges: vec![], inner_edges: vec![], edges: vec![], forward: true, step_face_id: entity.id, surface_step_id: None, edge_curves_2d: vec![], edge_step_ids: vec![], outer_edge_step_ids: vec![], inner_edge_step_ids: vec![], is_void: false };
                    let mesh = self.surface_to_mesh(&face_data, &params, &bbox);
                    results.push(MeshInstance {
                        name: entity.type_name.clone(),
                        mesh,
                        color: None,
                        transform: None,
                        brep_id: entity.id,
                    });
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // Point cloud fallback
        let points: Vec<Point3d> = self.step.find_entities_by_type("CARTESIAN_POINT")
            .iter()
            .filter_map(|e| self.resolve_cartesian_point(e.id))
            .collect();
        if points.len() >= 3 {
            let mut mesh = TriangleMesh::new();
            for p in &points { mesh.add_vertex(*p); }
            for i in 1..points.len().saturating_sub(1) {
                mesh.add_triangle(0, i as u32, (i + 1) as u32);
            }
            if mesh.triangle_count() > 0 {
                results.push(MeshInstance {
                    name: "Point Cloud".to_string(),
                    mesh,
                    color: None,
                    transform: None,
                    brep_id: 0,
                });
                return Ok(results);
            }
        }

        Err("No convertible surface geometry found in STEP file".to_string())
    }

    /// Convert STEP to detailed mesh instances with per-face information.
    fn convert_detailed_instances(&self) -> Result<Vec<DetailedMeshInstance>, String> {
        let bbox = self.compute_bounding_box();
        let mut params = TriangulationParams::default();
        // Scale max_deviation based on bounding box
        if let Some((bmin, bmax)) = &bbox {
            let dx = bmax.x - bmin.x;
            let dy = bmax.y - bmin.y;
            let dz = bmax.z - bmin.z;
            let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();
            if diagonal > 1.0 {
                params.max_deviation = params.max_deviation.max(diagonal * 0.0002);
            }
        }
        let color_map = self.extract_color_map();
        let mut brep_detail_cache: HashMap<i64, (TriangleMesh, Vec<FaceInfo>)> = HashMap::new();
        let mut results: Vec<DetailedMeshInstance> = Vec::new();

        // ─── Phase 1: Assembly-based conversion via NAUO tree walk ────────
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if !nauos.is_empty() {
            let mut parent_pd_to_children: HashMap<i64, Vec<(i64, i64, String)>> = HashMap::new();
            for nauo in &nauos {
                let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
                if let (Some(parent_pd), Some(child_pd)) = (relating_pd, related_pd) {
                    let name = self.extract_nauo_name(nauo);
                    parent_pd_to_children.entry(parent_pd).or_default().push((nauo.id, child_pd, name));
                }
            }

            let parent_pds: std::collections::HashSet<i64> = parent_pd_to_children.keys().copied().collect();
            let child_pds: std::collections::HashSet<i64> = nauos.iter()
                .filter_map(|n| self.extract_nauo_pd_refs(n).1)
                .collect();
            let roots: Vec<i64> = parent_pds.difference(&child_pds).copied().collect();

            if !roots.is_empty() {
                for root_pd in &roots {
                    let root_name = self.get_product_name(*root_pd);
                    self.walk_assembly_tree_detailed(
                        *root_pd,
                        &root_name,
                        &None,
                        &color_map,
                        &mut brep_detail_cache,
                        &params,
                        &bbox,
                        &parent_pd_to_children,
                        &mut results,
                        &mut std::collections::HashSet::new(),
                    );
                }
            }

            if !results.is_empty() {
                return Ok(results);
            }
        }

        // ─── Phase 2: No assembly — direct BREP conversion ───
        let breps = self.step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        for brep in &breps {
            let name = self.get_brep_name(brep.id);
            if let Some((mesh, faces)) = self.triangulate_brep_detailed_cached(brep.id, &mut brep_detail_cache, &params, &bbox) {
                let color = color_map.get(&brep.id).copied();
                results.push(DetailedMeshInstance {
                    name,
                    mesh,
                    color,
                    transform: None,
                    brep_id: brep.id,
                    faces,
                });
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // FACETED_BREP
        let faceted = self.step.find_entities_by_type("FACETED_BREP");
        for fb in &faceted {
            let name = self.get_brep_name(fb.id);
            if let Some((mesh, faces)) = self.triangulate_brep_detailed_cached(fb.id, &mut brep_detail_cache, &params, &bbox) {
                let color = color_map.get(&fb.id).copied();
                results.push(DetailedMeshInstance {
                    name,
                    mesh,
                    color,
                    transform: None,
                    brep_id: fb.id,
                    faces,
                });
            }
        }

        Ok(results)
    }

    fn triangulate_brep_detailed_cached(
        &self,
        brep_id: i64,
        cache: &mut HashMap<i64, (TriangleMesh, Vec<FaceInfo>)>,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<(TriangleMesh, Vec<FaceInfo>)> {
        if let Some(cached) = cache.get(&brep_id) {
            return Some(cached.clone());
        }
        let result = self.triangulate_brep_detailed(brep_id, params, bbox)?;
        cache.insert(brep_id, result.clone());
        Some(result)
    }

    /// Walk assembly tree producing DetailedMeshInstance results.
    /// Uses an explicit stack to avoid stack overflow on deeply nested assemblies.
    ///
    /// IMPORTANT: Instances are produced in DFS order (first child first) to match
    /// the order of tree leaves in `build_assembly_node_iterative`. This ensures
    /// that `assign_instance_indices` can correctly map tree leaves to instances.
    fn walk_assembly_tree_detailed(
        &self,
        root_pd_id: i64,
        _root_name: &str,
        root_transform: &Option<[[f64; 4]; 4]>,
        color_map: &HashMap<i64, [f32; 4]>,
        brep_detail_cache: &mut HashMap<i64, (TriangleMesh, Vec<FaceInfo>)>,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
        parent_pd_to_children: &HashMap<i64, Vec<(i64, i64, String)>>,
        results: &mut Vec<DetailedMeshInstance>,
        _visited: &mut std::collections::HashSet<(i64, i64)>,
    ) {
        // Work stack: (pd_id, composed_transform, depth, ancestor_set, is_leaf, nauo_name, brep_id)
        // We push ALL nodes (both sub-assemblies and leaves) onto the stack.
        // Children are pushed in REVERSE order so the first child is on top (LIFO → DFS).
        //
        // For leaf nodes (is_leaf=true), we create the instance immediately when popped.
        // For sub-assembly nodes (is_leaf=false), we push their children when popped.
        struct WorkItem {
            pd_id: i64,
            composed: Option<[[f64; 4]; 4]>,
            depth: usize,
            ancestors: std::collections::HashSet<i64>,
            is_leaf: bool,
            nauo_name: String,
            brep_id: Option<i64>,
        }

        let mut stack: Vec<WorkItem> = Vec::new();

        // Push root's children as initial work items (root itself is never a leaf)
        if let Some(children) = parent_pd_to_children.get(&root_pd_id) {
            let root_ancestors = std::collections::HashSet::new();
            // Push in reverse order so first child is processed first
            for (nauo_id, child_pd_id, nauo_name) in children.iter().rev() {
                let nauo_transform = self.find_nauo_transform(*nauo_id, *child_pd_id);
                let composed = match (&root_transform, &nauo_transform) {
                    (Some(pt), Some(nt)) => Some(mat4_mul(pt, nt)),
                    (Some(pt), None) => Some(*pt),
                    (None, Some(nt)) => Some(*nt),
                    (None, None) => None,
                };
                let has_nauo_children = parent_pd_to_children.contains_key(child_pd_id);
                let brep_id = if has_nauo_children { None } else { self.find_pd_brep(*child_pd_id) };

                stack.push(WorkItem {
                    pd_id: *child_pd_id,
                    composed,
                    depth: 1,
                    ancestors: root_ancestors.clone(),
                    is_leaf: !has_nauo_children,
                    nauo_name: nauo_name.clone(),
                    brep_id,
                });
            }
        }

        const MAX_DEPTH: usize = 50;

        while let Some(item) = stack.pop() {
            // Depth limit to prevent infinite loops
            if item.depth > MAX_DEPTH {
                log::warn!("Max depth {} reached at PD #{}, skipping", MAX_DEPTH, item.pd_id);
                continue;
            }
            // Cycle detection: only skip if this PD is already in our ancestor chain
            if item.ancestors.contains(&item.pd_id) {
                log::warn!("Cycle detected in NAUO tree at PD #{}, skipping", item.pd_id);
                continue;
            }

            if item.is_leaf {
                // Leaf node — create instance
                if let Some(brep_id) = item.brep_id {
                    if let Some((mesh, faces)) = self.triangulate_brep_detailed_cached(brep_id, brep_detail_cache, params, bbox) {
                        let mut instance_mesh = mesh.clone();
                        if let Some(ref tf) = item.composed {
                            instance_mesh.transform(tf);
                        }
                        let color = color_map.get(&brep_id).copied();
                        let name = format!("{} (BREP#{})", item.nauo_name, brep_id);
                        results.push(DetailedMeshInstance {
                            name,
                            mesh: instance_mesh,
                            color,
                            transform: item.composed,
                            brep_id,
                            faces,
                        });
                    }
                }
            } else {
                // Sub-assembly — push children in reverse order (first child on top)
                if let Some(children) = parent_pd_to_children.get(&item.pd_id) {
                    for (nauo_id, child_pd_id, nauo_name) in children.iter().rev() {
                        let nauo_transform = self.find_nauo_transform(*nauo_id, *child_pd_id);
                        let composed = match (&item.composed, &nauo_transform) {
                            (Some(pt), Some(nt)) => Some(mat4_mul(pt, nt)),
                            (Some(pt), None) => Some(*pt),
                            (None, Some(nt)) => Some(*nt),
                            (None, None) => None,
                        };
                        let has_nauo_children = parent_pd_to_children.contains_key(child_pd_id);
                        let brep_id = if has_nauo_children { None } else { self.find_pd_brep(*child_pd_id) };

                        let mut new_ancestors = item.ancestors.clone();
                        new_ancestors.insert(item.pd_id);

                        stack.push(WorkItem {
                            pd_id: *child_pd_id,
                            composed,
                            depth: item.depth + 1,
                            ancestors: new_ancestors,
                            is_leaf: !has_nauo_children,
                            nauo_name: nauo_name.clone(),
                            brep_id,
                        });
                    }
                }
            }
        }
    }

    /// Walk the assembly tree from a root PD, creating mesh instances
    /// for each leaf BREP occurrence with the correct composed transform.
    /// Uses an explicit stack to avoid stack overflow on deeply nested assemblies.
    ///
    /// IMPORTANT: Instances are produced in DFS order (first child first) to match
    /// the order of tree leaves in `build_assembly_node_iterative`.
    fn walk_assembly_tree(
        &self,
        root_pd_id: i64,
        _root_name: &str,
        root_transform: &Option<[[f64; 4]; 4]>,
        color_map: &HashMap<i64, [f32; 4]>,
        brep_mesh_cache: &mut HashMap<i64, TriangleMesh>,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
        parent_pd_to_children: &HashMap<i64, Vec<(i64, i64, String)>>,
        results: &mut Vec<MeshInstance>,
        _visited: &mut std::collections::HashSet<(i64, i64)>,
    ) {
        // Work stack: ALL nodes (both sub-assemblies and leaves) are pushed.
        // Children pushed in REVERSE order so first child is on top (LIFO → DFS).
        struct WorkItem {
            pd_id: i64,
            composed: Option<[[f64; 4]; 4]>,
            depth: usize,
            ancestors: std::collections::HashSet<i64>,
            is_leaf: bool,
            nauo_name: String,
            brep_id: Option<i64>,
        }

        let mut stack: Vec<WorkItem> = Vec::new();

        if let Some(children) = parent_pd_to_children.get(&root_pd_id) {
            let root_ancestors = std::collections::HashSet::new();
            for (nauo_id, child_pd_id, nauo_name) in children.iter().rev() {
                let nauo_transform = self.find_nauo_transform(*nauo_id, *child_pd_id);
                let composed = match (&root_transform, &nauo_transform) {
                    (Some(pt), Some(nt)) => Some(mat4_mul(pt, nt)),
                    (Some(pt), None) => Some(*pt),
                    (None, Some(nt)) => Some(*nt),
                    (None, None) => None,
                };
                let has_nauo_children = parent_pd_to_children.contains_key(child_pd_id);
                let brep_id = if has_nauo_children { None } else { self.find_pd_brep(*child_pd_id) };

                stack.push(WorkItem {
                    pd_id: *child_pd_id,
                    composed,
                    depth: 1,
                    ancestors: root_ancestors.clone(),
                    is_leaf: !has_nauo_children,
                    nauo_name: nauo_name.clone(),
                    brep_id,
                });
            }
        }

        const MAX_DEPTH: usize = 50;

        while let Some(item) = stack.pop() {
            if item.depth > MAX_DEPTH {
                log::warn!("Max depth {} reached at PD #{}, skipping", MAX_DEPTH, item.pd_id);
                continue;
            }
            if item.ancestors.contains(&item.pd_id) {
                log::warn!("Cycle detected in NAUO tree at PD #{}, skipping", item.pd_id);
                continue;
            }

            if item.is_leaf {
                if let Some(brep_id) = item.brep_id {
                    if let Some(mesh) = self.triangulate_brep_cached(brep_id, brep_mesh_cache, params, bbox) {
                        let mut instance_mesh = mesh.clone();
                        if let Some(ref tf) = item.composed {
                            instance_mesh.transform(tf);
                        }
                        let color = color_map.get(&brep_id).copied();
                        let name = format!("{} (BREP#{})", item.nauo_name, brep_id);
                        results.push(MeshInstance {
                            name,
                            mesh: instance_mesh,
                            color,
                            transform: item.composed,
                            brep_id,
                        });
                        info!("Instance: {} PD=#{} BREP=#{} color={:?} transform={}",
                            item.nauo_name, item.pd_id, brep_id, color, item.composed.is_some());
                    }
                } else {
                    warn!("PD=#{} has no NAUO children and no BREP — skipped", item.pd_id);
                }
            } else {
                if let Some(children) = parent_pd_to_children.get(&item.pd_id) {
                    for (nauo_id, child_pd_id, nauo_name) in children.iter().rev() {
                        let nauo_transform = self.find_nauo_transform(*nauo_id, *child_pd_id);
                        let composed = match (&item.composed, &nauo_transform) {
                            (Some(pt), Some(nt)) => Some(mat4_mul(pt, nt)),
                            (Some(pt), None) => Some(*pt),
                            (None, Some(nt)) => Some(*nt),
                            (None, None) => None,
                        };
                        let has_nauo_children = parent_pd_to_children.contains_key(child_pd_id);
                        let brep_id = if has_nauo_children { None } else { self.find_pd_brep(*child_pd_id) };

                        let mut new_ancestors = item.ancestors.clone();
                        new_ancestors.insert(item.pd_id);

                        stack.push(WorkItem {
                            pd_id: *child_pd_id,
                            composed,
                            depth: item.depth + 1,
                            ancestors: new_ancestors,
                            is_leaf: !has_nauo_children,
                            nauo_name: nauo_name.clone(),
                            brep_id,
                        });
                    }
                }
            }
        }
    }


    /// Build the assembly tree for display/debugging purposes.
    fn build_assembly_tree(&self) -> AssemblyNode {
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        let color_map = self.extract_color_map();

        // Build: parent_pd_id → Vec<(nauo_id, child_pd_id, nauo_name)>
        let mut parent_pd_to_children: HashMap<i64, Vec<(i64, i64, String)>> = HashMap::new();
        for nauo in &nauos {
            let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
            if let (Some(parent_pd), Some(child_pd)) = (relating_pd, related_pd) {
                let name = self.extract_nauo_name(nauo);
                parent_pd_to_children.entry(parent_pd).or_default().push((nauo.id, child_pd, name));
            }
        }

        // Find root(s)
        let parent_pds: std::collections::HashSet<i64> = parent_pd_to_children.keys().copied().collect();
        let child_pds: std::collections::HashSet<i64> = nauos.iter()
            .filter_map(|n| self.extract_nauo_pd_refs(n).1)
            .collect();
        let roots: Vec<i64> = parent_pds.difference(&child_pds).copied().collect();

        if let Some(&root_pd) = roots.first() {
            let root_name = self.get_product_name(root_pd);
            self.build_assembly_node_iterative(
                root_pd, &root_name, &color_map, &parent_pd_to_children,
            )
        } else {
            // No assembly — build flat tree from BREPs
            let mut node = AssemblyNode {
                name: "No Assembly".to_string(),
                pd_id: 0,
                brep_id: None,
                instance_index: None,
                transform: None,
                color: None,
                children: Vec::new(),
            };
            for brep in self.step.find_entities_by_type("MANIFOLD_SOLID_BREP") {
                let name = self.get_brep_name(brep.id);
                let color = color_map.get(&brep.id).copied();
                node.children.push(AssemblyNode {
                    name,
                    pd_id: 0,
                    brep_id: Some(brep.id),
                    instance_index: None,
                    transform: None,
                    color,
                    children: Vec::new(),
                });
            }
            node
        }
    }

    /// Build the assembly node tree using an explicit stack to avoid stack overflow.
    fn build_assembly_node_iterative(
        &self,
        root_pd_id: i64,
        root_name: &str,
        color_map: &HashMap<i64, [f32; 4]>,
        parent_pd_to_children: &HashMap<i64, Vec<(i64, i64, String)>>,
    ) -> AssemblyNode {
        // Stack entries: (pd_id, name, transform, color_override, ancestor_pd_set)
        // We build the tree bottom-up. Each occurrence is keyed by NAUO ID
        // (unique per usage), not PD ID, so the same part can appear at
        // multiple positions in the tree (e.g., 3 identical nuts at different
        // positions). The root node is keyed by its PD ID (negated to avoid
        // collision with NAUO IDs, which are always positive).
        const ROOT_KEY: i64 = -1; // sentinel key for the root node

        // First pass: DFS to determine processing order
        // Each entry: (node_key, pd_id, name, transform, color, ancestors)
        let mut order: Vec<(i64, i64, String, Option<[[f64; 4]; 4]>, Option<[f32; 4]>)> = Vec::new();
        let mut dfs_stack: Vec<(i64, i64, String, Option<[[f64; 4]; 4]>, Option<[f32; 4]>, std::collections::HashSet<i64>)> =
            vec![(ROOT_KEY, root_pd_id, root_name.to_string(), None, None, std::collections::HashSet::new())];

        while let Some((node_key, pd_id, name, transform, color_override, ancestors)) = dfs_stack.pop() {
            // Cycle detection: only skip if this PD is already in our ancestor chain
            if ancestors.contains(&pd_id) {
                log::warn!("Cycle detected in assembly tree at PD #{}, skipping", pd_id);
                continue;
            }

            let has_nauo_children = parent_pd_to_children.contains_key(&pd_id);
            let brep_id = if has_nauo_children { None } else { self.find_pd_brep(pd_id) };
            let color = color_override.or_else(|| brep_id.and_then(|id| color_map.get(&id).copied()));

            order.push((node_key, pd_id, name, transform, color));

            if let Some(children) = parent_pd_to_children.get(&pd_id) {
                for &(nauo_id, child_pd_id, ref nauo_name) in children.iter().rev() {
                    let nauo_transform = self.find_nauo_transform(nauo_id, child_pd_id);
                    let child_has_nauo_children = parent_pd_to_children.contains_key(&child_pd_id);
                    let child_brep_id = if child_has_nauo_children { None } else { self.find_pd_brep(child_pd_id) };
                    let child_color = child_brep_id.and_then(|id| color_map.get(&id).copied());
                    let child_name = self.get_product_name(child_pd_id);
                    let display_name = format!("{} ({})", nauo_name, child_name);
                    // Key each child by its NAUO ID (unique per occurrence)
                    let mut new_ancestors = ancestors.clone();
                    new_ancestors.insert(pd_id);
                    dfs_stack.push((nauo_id, child_pd_id, display_name, nauo_transform, child_color, new_ancestors));
                }
            }
        }

        // Build nodes bottom-up (children are processed before parents).
        // Key by node_key (NAUO ID for children, ROOT_KEY for root).
        let mut node_map: HashMap<i64, AssemblyNode> = HashMap::new();

        for (node_key, pd_id, name, transform, color) in order.into_iter().rev() {
            let has_nauo_children = parent_pd_to_children.contains_key(&pd_id);
            let brep_id = if has_nauo_children { None } else { self.find_pd_brep(pd_id) };

            let mut node = AssemblyNode {
                name,
                pd_id,
                brep_id,
                instance_index: None,
                transform,
                color,
                children: Vec::new(),
            };

            // Attach already-built children (keyed by NAUO ID)
            if let Some(children) = parent_pd_to_children.get(&pd_id) {
                for &(nauo_id, _child_pd_id, _) in children {
                    if let Some(child_node) = node_map.remove(&nauo_id) {
                        node.children.push(child_node);
                    }
                }
            }

            node_map.insert(node_key, node);
        }

        node_map.remove(&ROOT_KEY).unwrap_or_else(|| AssemblyNode {
            name: root_name.to_string(),
            pd_id: root_pd_id,
            brep_id: None,
            instance_index: None,
            transform: None,
            color: None,
            children: Vec::new(),
        })
    }

    /// Build a detailed text representation of the STEP file structure.
    fn build_detailed_structure(&self) -> String {
        let mut out = String::new();

        // ── Section 1: Raw NAUO relationships ──
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        out.push_str(&format!("== NAUO Relationships ({} total) ==\n", nauos.len()));
        for nauo in &nauos {
            let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
            let nauo_name = self.extract_nauo_name(nauo);
            let parent_name = relating_pd.map(|id| self.get_product_name(id)).unwrap_or_else(|| "?".to_string());
            let child_name = related_pd.map(|id| self.get_product_name(id)).unwrap_or_else(|| "?".to_string());
            let transform = relating_pd.and_then(|_| {
                related_pd.and_then(|cpid| self.find_nauo_transform(nauo.id, cpid))
            });
            let tf_str = match transform {
                Some(tf) => {
                    let tx = tf[0][3]; let ty = tf[1][3]; let tz = tf[2][3];
                    if tx.abs() < 1e-10 && ty.abs() < 1e-10 && tz.abs() < 1e-10 {
                        "rotation only".to_string()
                    } else {
                        format!("translate({:.1},{:.1},{:.1})", tx, ty, tz)
                    }
                }
                None => "NO TRANSFORM".to_string(),
            };
            out.push_str(&format!(
                "  NAUO#{} '{}' : {}(PD#{}) → {}(PD#{}) [{}]\n",
                nauo.id, nauo_name,
                parent_name, relating_pd.unwrap_or(0),
                child_name, related_pd.unwrap_or(0),
                tf_str
            ));
        }
        out.push('\n');

        // ── Section 2: PD → BREP mappings ──
        let pds = self.step.find_entities_by_type("PRODUCT_DEFINITION");
        out.push_str(&format!("== PD → BREP Mappings ({} PDs) ==\n", pds.len()));
        for pd in &pds {
            let name = self.get_product_name(pd.id);
            let brep_id = self.find_pd_brep(pd.id);
            match brep_id {
                Some(bid) => out.push_str(&format!("  PD#{} ({}) → BREP#{}\n", pd.id, name, bid)),
                None => out.push_str(&format!("  PD#{} ({}) → no BREP (assembly)\n", pd.id, name)),
            }
        }
        out.push('\n');

        // ── Section 3: Assembly Tree ──
        let tree = self.build_assembly_tree();
        out.push_str("== STEP Assembly Tree ==\n");
        format_assembly_node_detailed(&tree, 0, &mut out);
        out.push('\n');

        out
    }

    /// Triangulate a BREP, using cache to avoid re-triangulating the same BREP multiple times.
    fn triangulate_brep_cached(
        &self,
        brep_id: i64,
        cache: &mut HashMap<i64, TriangleMesh>,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<TriangleMesh> {
        if let Some(mesh) = cache.get(&brep_id) {
            return Some(mesh.clone());
        }
        let mesh = self.triangulate_brep(brep_id, params, bbox)?;
        cache.insert(brep_id, mesh.clone());
        Some(mesh)
    }

    /// Triangulate a single BREP entity.
    fn triangulate_brep(
        &self,
        brep_id: i64,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<TriangleMesh> {
        // P7: Use find_all_shell_refs to support BREP_WITH_VOIDS.
        let (outer_shell_id, void_shell_ids) = self.find_all_shell_refs_by_brep_id(brep_id);
        let shell_id = outer_shell_id?;
        let mut face_data_list = self.extract_shell_faces(shell_id, false)?;

        // Append void shell faces (already correctly oriented per STEP convention).
        for void_shell_id in &void_shell_ids {
            if let Some(void_faces) = self.extract_shell_faces(*void_shell_id, true) {
                face_data_list.extend(void_faces);
            }
        }

        // Create tolerance context for this BREP
        let tol_ctx = match bbox {
            Some((bmin, bmax)) => ToleranceContext::from_bounding_box(bmin, bmax),
            None => ToleranceContext::new(),
        };

        // ─── Healing pipeline: heal the solid before triangulation ────────
        let face_data_list = if self.config.heal {
            let (solid, face_id_map) = face_data_list_to_solid(&face_data_list);
            // Use aggressive healing: fix normals, stitch edges, propagate
            // tolerances, merge faces, fix self-intersections, and remove slivers.
            let healing_params = HealingParams::aggressive_with_context(&tol_ctx);
            let (healed, report) = heal_solid(&solid, &healing_params);
            log_healing_report(brep_id, &report);
            apply_healing_to_face_data(&face_data_list, &healed, &face_id_map)
        } else {
            face_data_list
        };

        // Create edge discretization cache for this BREP to ensure
        // shared edges produce identical 3D points (watertightness).
        let mut edge_cache = EdgeDiscretizationCache::with_tolerance(tol_ctx.clone(), 64);

        // ─── Build vertex-pair → canonical step_id aliases ──────────────
        // In STEP B-Rep, two faces sharing a geometric boundary may use
        // DIFFERENT EDGE_CURVE entities (e.g., a Plane face uses a LINE
        // while a NURBS face uses a NURBS curve). They share the same
        // VERTEX_POINT endpoints. By aliasing all step_ids with the same
        // vertex pair to a single canonical step_id, we ensure the edge
        // cache returns identical 3D points for all of them — guaranteeing
        // watertightness by construction.
        //
        // CRITICAL: The canonical step_id must be the one with the DENSEST
        // sampling. If a Plane face uses a LINE (2 pts) and a NURBS face
        // uses a NURBS curve (55 pts) on the same boundary, we MUST use
        // the NURBS step_id as canonical. Otherwise the NURBS face gets
        // only 2 boundary points, producing a degenerate triangulation.
        {
            // Phase 1: STEP entity ID-based aliasing (existing approach)
            // Uses VERTEX_POINT entity IDs to match edges sharing the same
            // geometric boundary.
            //
            // CRITICAL FIX: We also check the curve MIDPOINT to avoid aliasing
            // two DIFFERENT curves that share the same endpoints (e.g., two
            // half-circles forming a full circle). Without this check, the bolt's
            // transition plane gets its outer boundary collapsed to a single
            // half-circle, breaking watertightness.
            let mut vertex_pair_to_step_ids: HashMap<(i64, i64), Vec<i64>> = HashMap::new();
            for face_data in &face_data_list {
                for &step_id in &face_data.edge_step_ids {
                    if step_id == 0 { continue; }
                    if let Some(vp) = self.get_edge_curve_vertex_pair(step_id) {
                        vertex_pair_to_step_ids.entry(vp).or_default().push(step_id);
                    }
                }
            }
            let mut alias_count = 0usize;
            let mut skipped_different_curves = 0usize;
            for (_vp, step_ids) in &vertex_pair_to_step_ids {
                if step_ids.len() < 2 { continue; }

                // P2: Group by curve SHAPE using 5-point sampling (not just midpoint).
                // Two edges with the same vertex pair but different shapes
                // (e.g., two semicircles forming a full circle, or a LINE
                // vs a B-spline approximating a line) must NOT be aliased.
                let shape_tol = (tol_ctx.model_scale * 1e-3).max(1e-6); // 1000 PPM of model scale
                let shape_groups = self.group_step_ids_by_curve_shape(step_ids, shape_tol);

                // Alias within each shape group
                for (_samples, group_sids) in &shape_groups {
                    if group_sids.len() < 2 { continue; }
                    let canonical = *group_sids.iter().max_by_key(|&&sid| {
                        self.edge_curve_complexity_score(sid)
                    }).unwrap();
                    for &sid in group_sids {
                        if sid != canonical {
                            edge_cache.register_step_id_alias(sid, canonical);
                            alias_count += 1;
                        }
                    }
                }

                // Count groups with multiple step_ids that were NOT aliased
                // (different curves sharing the same vertex pair)
                if shape_groups.len() > 1 {
                    skipped_different_curves += step_ids.len() - shape_groups.iter().map(|(_, g)| g.len().min(1)).sum::<usize>();
                }
            }
            if alias_count > 0 || skipped_different_curves > 0 {
                log::info!(
                    "BREP #{}: registered {} step_id aliases from vertex-pair matching (skipped {} edges with same endpoints but different curves)",
                    brep_id, alias_count, skipped_different_curves,
                );
            }

            // Phase 2: 3D coordinate-based aliasing (supplementary approach)
            // When STEP files use DIFFERENT VERTEX_POINT entities for the
            // same geometric endpoint, the entity-ID approach misses them.
            // This phase matches edges by their 3D endpoint coordinates,
            // catching edges that share the same geometric boundary but
            // have different STEP entity IDs.
            //
            // Algorithm:
            // 1. For each edge's step_id, compute its start and end 3D points
            // 2. Create a spatial key from the rounded 3D coordinates
            // 3. Group step_ids by their 3D endpoint pair
            // 4. Alias non-canonical step_ids to the canonical one
            let coord_tol = (tol_ctx.model_scale * 2e-3).max(tol_ctx.absolute * 10.0); // 2000 PPM of model scale — matches snap tolerance // 0.1% of model scale for coordinate matching
            let mut coord_pair_to_step_ids: HashMap<(i64, i64, i64, i64, i64, i64), Vec<i64>> = HashMap::new();
            let mut step_id_endpoints: HashMap<i64, (Point3d, Point3d)> = HashMap::new();
            let mut unaliased_count = 0usize;
            for face_data in &face_data_list {
                for (edge_idx, &step_id) in face_data.edge_step_ids.iter().enumerate() {
                    if step_id == 0 { continue; }
                    // Skip if already aliased (from Phase 1)
                    if edge_cache.resolve_canonical_step_id(step_id) != step_id {
                        continue;
                    }
                    unaliased_count += 1;
                    // Get edge endpoints from the Edge object
                    let edge = &face_data.edges[edge_idx];
                    let start = match edge.start_point() {
                        Some(p) => p,
                        None => continue,
                    };
                    let end = match edge.end_point() {
                        Some(p) => p,
                        None => continue,
                    };
                    step_id_endpoints.insert(step_id, (start, end));
                    // Quantize 3D coordinates to grid cells for spatial matching
                    let sk = (
                        (start.x / coord_tol).round() as i64,
                        (start.y / coord_tol).round() as i64,
                        (start.z / coord_tol).round() as i64,
                        (end.x / coord_tol).round() as i64,
                        (end.y / coord_tol).round() as i64,
                        (end.z / coord_tol).round() as i64,
                    );
                    // Also try reversed endpoint order (same geometric edge, opposite direction)
                    let sk_rev = (
                        (end.x / coord_tol).round() as i64,
                        (end.y / coord_tol).round() as i64,
                        (end.z / coord_tol).round() as i64,
                        (start.x / coord_tol).round() as i64,
                        (start.y / coord_tol).round() as i64,
                        (start.z / coord_tol).round() as i64,
                    );
                    // Use canonical ordering (smaller key first) for consistent grouping
                    let canonical_key = if sk <= sk_rev { sk } else { sk_rev };
                    coord_pair_to_step_ids.entry(canonical_key).or_default().push(step_id);
                }
            }
            log::info!(
                "BREP #{}: Phase 2 alias: {} unaliased step_ids, {} coordinate groups, tol={:.2e}",
                brep_id, unaliased_count, coord_pair_to_step_ids.len(), coord_tol
            );
            let mut coord_alias_count = 0usize;
            for (_key, step_ids) in &coord_pair_to_step_ids {
                if step_ids.len() < 2 { continue; }
                // Choose canonical: highest complexity score (densest sampling)
                let canonical = *step_ids.iter().max_by_key(|&&sid| {
                    self.edge_curve_complexity_score(sid)
                }).unwrap();
                for &sid in step_ids {
                    if sid != canonical {
                        edge_cache.register_step_id_alias(sid, canonical);
                        coord_alias_count += 1;
                    }
                }
            }
            if coord_alias_count > 0 {
                log::info!(
                    "BREP #{}: registered {} additional step_id aliases from 3D coordinate matching (tol={:.2e})",
                    brep_id, coord_alias_count, coord_tol
                );
            }
        }

        let mut mesh = TriangleMesh::new();
        // Tolerance-based dedup: catches near-identical vertices from different
        // STEP EDGE_CURVE entities on the same geometric boundary (FP drift
        // typically 1e-13). Merge tolerance = 1 PPM of model scale.
        let merge_tol = (tol_ctx.model_scale * 1e-4).max(tol_ctx.absolute * 10.0);
        let mut dedup_map = draper_mesh::mesh::VertexDedupMap::with_tolerance(merge_tol);
        let mut total_face_vertices = 0usize;
        for (fi, face_data) in face_data_list.iter().enumerate() {
            let surface_type = match &face_data.surface {
                Surface::Plane(_) => "Plane",
                Surface::Cylinder(_) => "Cylinder",
                Surface::Cone(_) => "Cone",
                Surface::Sphere(_) => "Sphere",
                Surface::Torus(_) => "Torus",
                Surface::Revolution(_) => "Revolution",
                Surface::Extrusion(_) => "Extrusion",
                Surface::Nurbs(n) => {
                    let (u0, u1) = n.u_range();
                    let (v0, v1) = n.v_range();
                    &format!("Nurbs(deg={}/{}, cps={}x{}, knots_u={}({:.2}..{:.2}), knots_v={}({:.2}..{:.2}))",
                        n.u_degree, n.v_degree, n.control_points.len(), 
                        n.control_points.first().map(|r| r.len()).unwrap_or(0),
                        n.u_knots.len(), u0, u1, n.v_knots.len(), v0, v1)
                }
            };
            let n_outer = face_data.outer_edges.len();
            let n_inner = face_data.inner_edges.len();

            // Sample the surface at a few points to check if evaluation works
            if let Surface::Nurbs(ref n) = face_data.surface {
                let (u0, u1) = n.u_range();
                let (v0, v1) = n.v_range();
                let p00 = face_data.surface.point_at(u0, v0);
                let p_mid = face_data.surface.point_at((u0+u1)/2.0, (v0+v1)/2.0);
                let p11 = face_data.surface.point_at(u1, v1);
                log::debug!("BREP #{} face[{}]: {} outer={} inner={} ur={:.4}..{:.4} vr={:.4}..{:.4}", 
                    brep_id, fi, surface_type, n_outer, n_inner, u0, u1, v0, v1);
                log::debug!("  sample(0,0)=({:.4},{:.4},{:.4}) mid=({:.4},{:.4},{:.4}) end=({:.4},{:.4},{:.4})",
                    p00.x, p00.y, p00.z, p_mid.x, p_mid.y, p_mid.z, p11.x, p11.y, p11.z);
                // Print first 3 control points
                for (ri, row) in n.control_points.iter().enumerate().take(3) {
                    for (ci, cp) in row.iter().enumerate().take(3) {
                        log::debug!("  cp[{}][{}]=({:.4},{:.4},{:.4})", ri, ci, cp.x, cp.y, cp.z);
                    }
                }
                log::debug!("  u_knots={:?}", &n.u_knots);
                log::debug!("  v_knots={:?}", &n.v_knots);
            } else {
                log::debug!("BREP #{} face[{}]: {} outer={} inner={}", brep_id, fi, surface_type, n_outer, n_inner);
            }

            let face_mesh = self.surface_to_mesh_cached(face_data, params, bbox, &mut edge_cache);
            let (fbmin, fbmax) = face_mesh.bounding_box();
            log::debug!("  -> v={} t={} bbox=({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2})",
                face_mesh.vertex_count(), face_mesh.triangle_count(),
                fbmin.x, fbmin.y, fbmin.z, fbmax.x, fbmax.y, fbmax.z);
            total_face_vertices += face_mesh.vertices.len();
            let pre_merge_tri_count = mesh.triangle_count();
            mesh.merge_deduplicating(&face_mesh, &mut dedup_map);
            let post_merge_tri_count = mesh.triangle_count();
            let face_tri_count = face_mesh.triangle_count();
            let added_tris = post_merge_tri_count - pre_merge_tri_count;
            let lost_tris = face_tri_count as isize - added_tris as isize;
            if lost_tris > 3 {
                eprintln!(
                    "MERGE_LOSS: BREP #{} STEP #{} ({}): {} of {} triangles lost during merge",
                    brep_id, face_data.step_face_id, surface_type, lost_tris, face_tri_count
                );
            }
        }
        let deduped_vertices = total_face_vertices - mesh.vertices.len();
        if deduped_vertices > 0 {
            log::info!(
                "BREP #{} vertex deduplication: {} face vertices → {} unique ({} shared)",
                brep_id, total_face_vertices, mesh.vertices.len(), deduped_vertices,
            );
        }

        // ─── Post-merge: filter degenerate triangles ────────────────
        // Degenerate triangles (zero area, NaN/Inf vertices, or collapsed
        // indices) create non-manifold edges and false boundary edges.
        // Remove them before validation and snapping.
        let pre_filter_tris = mesh.triangle_count();
        filter_degenerate_triangles(&mut mesh, 1e-10);
        let post_filter_tris = mesh.triangle_count();
        let filter_removed = pre_filter_tris - post_filter_tris;
        if filter_removed > 0 {
            eprintln!("POST_FILTER: BREP #{}: {} degenerate triangles removed ({}→{})", brep_id, filter_removed, pre_filter_tris, post_filter_tris);
        }

        // ─── Post-merge: weld boundary edge vertices ────────────────
        // When two adjacent faces share a geometric edge but use different
        // STEP EDGE_CURVE entities (e.g., a Plane face uses a full circle
        // while a NURBS face uses a half-arc of the same circle), their
        // discretizations produce slightly different vertex positions at
        // the shared corners. This creates short boundary edges (holes) in
        // the merged mesh.
        //
        // This step welds (merges) vertices connected by short boundary
        // edges, fixing the seam mismatch without affecting interior
        // vertices.
        //
        // Tolerance: 3% of model scale, capped at 10mm. This catches the
        // typical seam mismatch (0.1-0.5mm) AND the larger mismatches from
        // different EDGE_CURVE entities on the same geometric boundary
        // (up to ~5mm observed in some files), while being tight enough
        // to not collapse distinct features.
        //
        // PASS 2 (long-edge vertex welding) uses a MUCH TIGHTER tolerance
        // internally (1% of this value, capped at 1e-3) to avoid welding
        // unrelated boundary vertices from different faces. See
        // weld_boundary_edge_vertices's PASS 2 comment for details.
        {
            // Skip welding if mesh is already watertight — prevents
            // degenerate triangle creation (Step#87/#78).
            let report_before = validate_watertight(&mesh, false);
            if !report_before.is_watertight() {
                let weld_tol = (tol_ctx.model_scale * 3e-2).min(10.0).max(1e-4);
                weld_boundary_edge_vertices(&mut mesh, weld_tol);
            }
        }
        // When the STEP file uses different VERTEX_POINT entities for the
        // same geometric boundary (e.g., Plane face uses LINE, NURBS face
        // uses NURBS curve), bit-exact dedup can't merge the boundary
        // vertices. This post-processing step snaps boundary vertices to
        // nearby shared/boundary vertices, welding the mesh at geometrically
        // coincident boundaries.
        //
        // RE-ENABLED with boundary-only snap logic: the snap function now
        // only snaps boundary vertices to other boundary or shared vertices,
        // NEVER to interior vertices. This prevents the corruption that
        // previously disabled this code path.
        //
        // Tolerance: 2000 PPM of model scale. This catches typical FP drift
        // between independent edge curve discretizations of the same geometric
        // boundary (observed up to ~1400 PPM in brick_thin.stp), while being
        // tight enough to not collapse distinct features (which are typically
        // >10,000 PPM apart).
        //
        // DISABLED AGAIN: The snap_tolerance is computed from the FULL file bbox
        // (which includes world placements like (47.5, 87.99, 33) for the bolt),
        // inflating model_scale by ~3x. This made snap_tol = 0.245mm for a 7.5mm
        // radius bolt, creating 301 duplicate triangles and corrupting the mesh.
        // The edge cache produces 100% bit-identical shared vertices, so snapping
        // is not needed for watertightness.
        /*
        {
            let snap_tol = (tol_ctx.model_scale * 2e-3).max(tol_ctx.absolute * 10.0);
            let snapped = mesh.snap_boundary_vertices(snap_tol);
            if snapped > 0 {
                log::info!(
                    "BREP #{}: snapped {} boundary vertices (tol={:.2e})",
                    brep_id, snapped, snap_tol
                );
                // After snapping, filter degenerate triangles and remove duplicates
                filter_degenerate_triangles(&mut mesh, 1e-10);
                let dup_after_snap = mesh.remove_duplicate_triangles();
                if dup_after_snap > 0 {
                    log::info!(
                        "BREP #{}: removed {} duplicate triangles after snapping",
                        brep_id, dup_after_snap,
                    );
                }
            }
        }
        */
        // Validation — do NOT apply repair_mesh. If the mesh is not watertight,
        // that indicates a bug in the edge cache or surface discretization.
        // repair_mask/stitch_boundary_edges mask the real problem by moving
        // vertices by up to 100× base_tol. Instead, log the issue and run
        // edge consistency validation to diagnose the root cause.
        let adaptive_tol = edge_cache.adaptive_tolerance().merge_tolerance();
        let report_before = validate_watertight(&mesh, false);
        if !report_before.is_watertight() {
            let boundary_pct = if report_before.edge_count > 0 {
                report_before.boundary_edge_count as f64 / report_before.edge_count as f64 * 100.0
            } else {
                0.0
            };
            log::error!(
                "BUG: BREP #{} not watertight: {} boundary edges ({:.2}%), {} non-manifold (tol={:.2e})",
                brep_id, report_before.boundary_edge_count, boundary_pct,
                report_before.non_manifold_edge_count, adaptive_tol
            );
            if boundary_pct > 1.0 {
                log::error!("More than 1% boundary edges — edge cache is NOT working correctly!");
            }
            // Run edge consistency validation to diagnose
            let consistency = validate_edge_consistency(&mesh, adaptive_tol);
            log::error!(
                "Edge consistency: {}/{} consistent, {} inconsistent ({:.2}%), max_dist={:.2e}",
                consistency.consistent_edges, consistency.shared_edges_checked,
                consistency.inconsistent_edges, consistency.inconsistency_rate(),
                consistency.max_vertex_distance
            );
            for inc in &consistency.worst_inconsistencies {
                log::error!(
                    "  Inconsistent edge: vertices ({}, {}), dist={:.2e}, faces={:?}",
                    inc.vertex_indices.0, inc.vertex_indices.1, inc.distance, inc.face_ids
                );
            }
            // Log edge cache stats
            let stats = edge_cache.stats();
            log::info!("Edge cache stats: {} entries, {} hits, {} misses, {} shared, hit_rate={:.1}%",
                stats.total_edges, stats.cache_hits, stats.cache_misses, stats.shared_edges,
                if stats.cache_hits + stats.cache_misses > 0 {
                    stats.cache_hits as f64 / (stats.cache_hits + stats.cache_misses) as f64 * 100.0
                } else { 0.0 }
            );
        }

        // Phase 2: Validate watertightness of the merged mesh
        let wt_report = validate_watertight(&mesh, false);
        if wt_report.is_watertight() {
            log::info!("BREP #{}: watertight ✓ ({} interior edges, Euler χ={})",
                brep_id, wt_report.interior_edge_count, wt_report.euler_characteristic);
        } else {
            log::warn!("BREP #{}: NOT watertight — {} boundary edges, {} non-manifold edges, χ={}",
                brep_id, wt_report.boundary_edge_count, wt_report.non_manifold_edge_count,
                wt_report.euler_characteristic);
            if wt_report.degenerate_triangle_count > 0 {
                log::warn!("BREP #{}: {} degenerate triangles", brep_id, wt_report.degenerate_triangle_count);
            }
        }

        log::info!("BREP #{}: edge_cache={} entries, mesh v={} t={}",
            brep_id, edge_cache.len(), mesh.vertex_count(), mesh.triangle_count());
        Some(mesh)
    }

    /// Triangulate a BREP with per-face ID tracking and FaceInfo generation.
    fn triangulate_brep_detailed(
        &self,
        brep_id: i64,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<(TriangleMesh, Vec<FaceInfo>)> {
        eprintln!("ENTER triangulate_brep_detailed: BREP #{}", brep_id);
        // P7: Use find_all_shell_refs to support BREP_WITH_VOIDS.
        // The outer shell provides the main solid; each void shell provides
        // an internal cavity (face normals already point into the solid material).
        let (outer_shell_id, void_shell_ids) = self.find_all_shell_refs_by_brep_id(brep_id);
        let shell_id = outer_shell_id?;
        let mut face_data_list = self.extract_shell_faces(shell_id, false)?;

        // Extract faces from each void shell and append them to the list.
        // The void shells have INVERTED orientation in STEP — their face normals
        // point INTO the solid material. By including them as additional faces,
        // the resulting mesh represents a solid with internal cavities.
        // (Algorithm adapted from truck-step v0.4.0, ricosjp/truck, Apache-2.0 OR MIT.)
        for void_shell_id in &void_shell_ids {
            match self.extract_shell_faces(*void_shell_id, true) {
                Some(void_faces) => {
                    log::info!(
                        "BREP #{}: appending {} faces from void shell #{}",
                        brep_id, void_faces.len(), void_shell_id
                    );
                    face_data_list.extend(void_faces);
                }
                None => {
                    log::warn!(
                        "BREP #{}: failed to extract faces from void shell #{} — skipping",
                        brep_id, void_shell_id
                    );
                }
            }
        }

        // Log face_data_list composition
        {
            let mut surface_counts: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
            for fd in &face_data_list {
                let key = match &fd.surface {
                    Surface::Plane(_) => "Plane",
                    Surface::Cylinder(_) => "Cylinder",
                    Surface::Cone(_) => "Cone",
                    Surface::Sphere(_) => "Sphere",
                    Surface::Torus(_) => "Torus",
                    Surface::Revolution(_) => "Revolution",
                    Surface::Extrusion(_) => "Extrusion",
                    Surface::Nurbs(_) => "Nurbs",
                };
                *surface_counts.entry(key).or_insert(0) += 1;
            }
            let summary: Vec<String> = surface_counts.iter()
                .map(|(k, v)| format!("{}={}", k, v))
                .collect();
            log::info!(
                "BREP #{} face_data_list: {} faces [{}]",
                brep_id, face_data_list.len(), summary.join(", "),
            );
        }

        // Create tolerance context for this BREP
        let tol_ctx = match bbox {
            Some((bmin, bmax)) => ToleranceContext::from_bounding_box(bmin, bmax),
            None => ToleranceContext::new(),
        };

        // ─── Healing pipeline: heal the solid before triangulation ────────
        let face_data_list = if self.config.heal {
            let pre_heal_count = face_data_list.len();
            let (solid, face_id_map) = face_data_list_to_solid(&face_data_list);
            // Use aggressive healing: fix normals, stitch edges, propagate
            // tolerances, merge faces, fix self-intersections, and remove slivers.
            let healing_params = HealingParams::aggressive_with_context(&tol_ctx);
            let (healed, report) = heal_solid(&solid, &healing_params);
            log_healing_report(brep_id, &report);
            let healed_list = apply_healing_to_face_data(&face_data_list, &healed, &face_id_map);
            if healed_list.len() != pre_heal_count {
                log::warn!(
                    "BREP #{} healing changed face count: {} → {}",
                    brep_id, pre_heal_count, healed_list.len(),
                );
            }
            healed_list
        } else {
            face_data_list
        };

        // ─── Adaptive LOD: compute per-face triangle budget ─────────────
        // When adaptive_lod_enabled is set in the viewer, compute a per-face
        // budget from total_budget / face_count so that each face gets a fair
        // share. This replaces the old approach of triangulating every face at
        // full quality and then decimating the combined mesh.
        let mut params = params.clone();
        if params.adaptive_lod_enabled {
            params.with_adaptive_lod(face_data_list.len());
        }

        // Create edge discretization cache for this BREP
        let mut edge_cache = EdgeDiscretizationCache::with_tolerance(tol_ctx.clone(), 64);

        // ─── Build vertex-pair → canonical step_id aliases ──────────────
        {
            let mut vertex_pair_to_step_ids: HashMap<(i64, i64), Vec<i64>> = HashMap::new();
            for face_data in &face_data_list {
                for &step_id in &face_data.edge_step_ids {
                    if step_id == 0 { continue; }
                    if let Some(vp) = self.get_edge_curve_vertex_pair(step_id) {
                        vertex_pair_to_step_ids.entry(vp).or_default().push(step_id);
                    }
                }
            }
            let mut alias_count = 0usize;
            let mut skipped_different_curves = 0usize;
            for (vp, step_ids) in &vertex_pair_to_step_ids {
                if step_ids.len() < 2 { continue; }

                // P2: Group by curve SHAPE using 5-point sampling.
                let shape_tol = (tol_ctx.model_scale * 1e-3).max(1e-6);
                let shape_groups = self.group_step_ids_by_curve_shape(step_ids, shape_tol);

                for (_samples, group_sids) in &shape_groups {
                    if group_sids.len() < 2 { continue; }
                    let canonical = *group_sids.iter().max_by_key(|&&sid| {
                        self.edge_curve_complexity_score(sid)
                    }).unwrap();
                    for &sid in group_sids {
                        if sid != canonical {
                            edge_cache.register_step_id_alias(sid, canonical);
                            alias_count += 1;
                        }
                    }
                    log::debug!(
                        "BREP #{}: vertex pair {:?} → canonical step_id={}, aliases={:?} (detailed path)",
                        brep_id, vp, canonical, group_sids.iter().filter(|&&s| s != canonical).collect::<Vec<_>>()
                    );
                }

                if shape_groups.len() > 1 {
                    skipped_different_curves += step_ids.len() - shape_groups.iter().map(|(_, g)| g.len().min(1)).sum::<usize>();
                    // Log details about skipped curves for diagnosis
                    log::warn!(
                        "BREP #{}: skipped {} step_ids at vertex_pair {:?} — {} shape groups (shape_tol={:.4})",
                        brep_id,
                        step_ids.len() - shape_groups.iter().map(|(_, g)| g.len().min(1)).sum::<usize>(),
                        vp,
                        shape_groups.len(),
                        shape_tol,
                    );
                    for (i, (samples, group_sids)) in shape_groups.iter().enumerate() {
                        let mid = samples.get(samples.len() / 2).copied().unwrap_or(Point3d::new(f64::NAN, f64::NAN, f64::NAN));
                        log::warn!(
                            "  group {}: mid=({:.4},{:.4},{:.4}) step_ids={:?}",
                            i, mid.x, mid.y, mid.z, group_sids,
                        );
                    }
                }
            }
            if alias_count > 0 || skipped_different_curves > 0 {
                log::info!(
                    "BREP #{}: registered {} step_id aliases from vertex-pair matching (detailed path, skipped {} edges with same endpoints but different curves)",
                    brep_id, alias_count, skipped_different_curves,
                );
            }

            // Phase 2: 3D coordinate-based aliasing (supplementary)
            // Same logic as in triangulate_brep() — see comments there.
            // Also applies midpoint check to avoid aliasing different curves.
            let coord_tol = (tol_ctx.model_scale * 2e-3).max(tol_ctx.absolute * 10.0); // 2000 PPM of model scale
            let mut coord_pair_to_step_ids: HashMap<(i64, i64, i64, i64, i64, i64), Vec<i64>> = HashMap::new();
            for face_data in &face_data_list {
                for (edge_idx, &step_id) in face_data.edge_step_ids.iter().enumerate() {
                    if step_id == 0 { continue; }
                    if edge_cache.resolve_canonical_step_id(step_id) != step_id {
                        continue;
                    }
                    let edge = &face_data.edges[edge_idx];
                    let start = match edge.start_point() { Some(p) => p, None => continue };
                    let end = match edge.end_point() { Some(p) => p, None => continue };
                    let sk = (
                        (start.x / coord_tol).round() as i64,
                        (start.y / coord_tol).round() as i64,
                        (start.z / coord_tol).round() as i64,
                        (end.x / coord_tol).round() as i64,
                        (end.y / coord_tol).round() as i64,
                        (end.z / coord_tol).round() as i64,
                    );
                    let sk_rev = (
                        (end.x / coord_tol).round() as i64,
                        (end.y / coord_tol).round() as i64,
                        (end.z / coord_tol).round() as i64,
                        (start.x / coord_tol).round() as i64,
                        (start.y / coord_tol).round() as i64,
                        (start.z / coord_tol).round() as i64,
                    );
                    let canonical_key = if sk <= sk_rev { sk } else { sk_rev };
                    coord_pair_to_step_ids.entry(canonical_key).or_default().push(step_id);
                }
            }
            let mut coord_alias_count = 0usize;
            let mut coord_groups_with_multiple = 0usize;
            let mut coord_skipped_different_curves = 0usize;
            for (_key, step_ids) in &coord_pair_to_step_ids {
                if step_ids.len() < 2 { continue; }
                coord_groups_with_multiple += 1;

                // P2: Apply shape-based grouping (5-point sampling) — same as Phase 1
                let shape_tol = (tol_ctx.model_scale * 1e-3).max(1e-6);
                let shape_groups = self.group_step_ids_by_curve_shape(step_ids, shape_tol);

                for (_samples, group_sids) in &shape_groups {
                    if group_sids.len() < 2 { continue; }
                    let canonical = *group_sids.iter().max_by_key(|&&sid| {
                        self.edge_curve_complexity_score(sid)
                    }).unwrap();
                    for &sid in group_sids {
                        if sid != canonical {
                            edge_cache.register_step_id_alias(sid, canonical);
                            coord_alias_count += 1;
                        }
                    }
                }

                if shape_groups.len() > 1 {
                    coord_skipped_different_curves += step_ids.len() - shape_groups.iter().map(|(_, g)| g.len().min(1)).sum::<usize>();
                }
            }
            log::info!(
                "BREP #{}: Phase 2 alias: {} coord groups, {} with multiple step_ids, {} aliases registered, {} skipped different curves (tol={:.2e})",
                brep_id, coord_pair_to_step_ids.len(), coord_groups_with_multiple, coord_alias_count, coord_skipped_different_curves, coord_tol,
            );
        }

        // Time guard: limit per-BREP triangulation time.
        // WASM uses a moderate limit to avoid browser freezes;
        // native uses a generous limit for complex assemblies.
        // Use override from TriangulationParams if provided.
        #[cfg(target_arch = "wasm32")]
        let default_brep_time_limit = std::time::Duration::from_secs(30);
        #[cfg(not(target_arch = "wasm32"))]
        let default_brep_time_limit = std::time::Duration::from_secs(600);
        let brep_time_limit = params.brep_time_limit_override.unwrap_or(default_brep_time_limit);

        // Per-FACE time limit: if a single face takes longer than this,
        // we skip it (returning an empty mesh for that face) rather than blocking.
        // Use override from TriangulationParams if provided.
        #[cfg(target_arch = "wasm32")]
        let default_face_time_limit = std::time::Duration::from_secs(3);
        #[cfg(not(target_arch = "wasm32"))]
        let default_face_time_limit = std::time::Duration::from_secs(120);
        let face_time_limit = params.face_time_limit_override.unwrap_or(default_face_time_limit);

        let brep_start = StdInstant::now();

        let mut mesh = TriangleMesh::new();
        // Tolerance-based dedup: edge cache with deterministic rounding (48-bit
        // mantissa) produces bit-identical coordinates for shared STEP EDGE_CURVEs.
        // However, STEP files often have DIFFERENT EDGE_CURVE entities on the same
        // geometric boundary (e.g., a seam edge of a cylinder stored as two separate
        // EDGE_CURVEs for the two adjacent faces). In those cases the edge cache
        // produces near-identical but not bit-identical vertices (typically 1e-13
        // apart). Without a tolerance fallback these vertices end up with different
        // indices, producing boundary edges and non-watertight meshes.
        //
        // The merge tolerance is set to 1 PPM of the model scale — small enough
        // to never collapse genuinely distinct features, but large enough to catch
        // FP drift between different EDGE_CURVE entities on the same boundary.
        let merge_tol = (tol_ctx.model_scale * 1e-4).max(tol_ctx.absolute * 10.0);
        let mut dedup_map = draper_mesh::mesh::VertexDedupMap::with_tolerance(merge_tol);
        let mut total_face_vertices_detailed = 0usize;
        let mut face_infos = Vec::new();
        let mut next_face_id: u64 = 1;
        let mut skipped_faces = 0;
        let _timed_out_faces = 0;

        for (fi, face_data) in face_data_list.iter().enumerate() {
            // Check BREP-level time budget — skip remaining faces if we're over limit
            if brep_start.elapsed() > brep_time_limit {
                skipped_faces += face_data_list.len() - fi;
                log::warn!(
                    "BREP #{}: time limit reached after {} faces, skipping {} remaining",
                    brep_id, fi, skipped_faces
                );
                break;
            }

            // Check per-FACE time budget before starting an expensive face.
            // If we've already used most of the BREP time budget, skip remaining faces.
            let elapsed = brep_start.elapsed();
            if elapsed + face_time_limit > brep_time_limit {
                skipped_faces += face_data_list.len() - fi;
                log::warn!(
                    "BREP #{}: insufficient time budget for face {} (elapsed {:?}), skipping {} remaining",
                    brep_id, fi, elapsed, face_data_list.len() - fi
                );
                break;
            }

            let _face_start = StdInstant::now();

            let face_id = next_face_id;
            next_face_id += 1;
            let step_face_id = face_data.step_face_id;

            let surface_type = match &face_data.surface {
                Surface::Plane(_) => "Plane".to_string(),
                Surface::Cylinder(_) => "Cylinder".to_string(),
                Surface::Cone(_) => "Cone".to_string(),
                Surface::Sphere(_) => "Sphere".to_string(),
                Surface::Torus(_) => "Torus".to_string(),
                Surface::Revolution(_) => "Revolution".to_string(),
                Surface::Extrusion(_) => "Extrusion".to_string(),
                Surface::Nurbs(n) => {
                    format!("Nurbs(deg={}/{}, cps={}x{})",
                        n.u_degree, n.v_degree, n.control_points.len(), 
                        n.control_points.first().map(|r| r.len()).unwrap_or(0))
                }
            };

            let tri_start = mesh.triangle_count();
            let face_mesh = self.surface_to_mesh_cached(face_data, &params, bbox, &mut edge_cache);
            
            // Set face ID for all triangles in this face mesh
            let face_tri_count = face_mesh.triangle_count();
            
            // Log faces that produced zero triangles (likely triangulation failures)
            if face_tri_count == 0 {
                log::warn!(
                    "BREP #{} face #{} (STEP #{}, type={}): produced 0 triangles — triangulation may have failed",
                    brep_id, face_id, step_face_id, surface_type
                );
            }
            
            let mut face_mesh_with_ids = face_mesh.clone();
            face_mesh_with_ids.triangle_face_ids = Some(vec![face_id; face_tri_count]);

            let pre_merge_tri_count = mesh.triangle_count();
            mesh.merge_deduplicating(&face_mesh_with_ids, &mut dedup_map);
            let post_merge_tri_count = mesh.triangle_count();
            let added_tris = post_merge_tri_count - pre_merge_tri_count;
            let lost_tris = face_tri_count as isize - added_tris as isize;
            eprintln!("MERGE: BREP #{} STEP #{} ({}): face_tris={} added={} lost={}",
                brep_id, step_face_id, surface_type, face_tri_count, added_tris, lost_tris);
            if lost_tris > 0 {
                log::warn!(
                    "BREP #{} face #{} (STEP #{}, {}): {} of {} triangles lost during merge (degenerate/duplicate)",
                    brep_id, face_id, step_face_id, surface_type, lost_tris, face_tri_count
                );
                eprintln!(
                    "MERGE_LOSS: BREP #{} face #{} (STEP #{}, {}): {} of {} triangles lost during merge",
                    brep_id, face_id, step_face_id, surface_type, lost_tris, face_tri_count
                );
            }
            total_face_vertices_detailed += face_mesh_with_ids.vertices.len();
            let tri_end = mesh.triangle_count();

            // Sample boundary edges into polylines (3D and UV)
            let outer_boundary: Vec<Vec<Point3d>> = if face_data.outer_edges.is_empty() {
                vec![]
            } else {
                vec![self.sample_edges_to_polylines(&face_data.outer_edges)]
            };
            let inner_boundaries: Vec<Vec<Point3d>> = face_data.inner_edges.iter()
                .map(|edges| self.sample_edges_to_polylines(edges))
                .collect();

            // Project boundary to UV space on all platforms
            let outer_uv_boundary = self.sample_edges_to_uv_polylines(&face_data.outer_edges, &face_data.surface);
            let inner_uv_boundaries: Vec<Vec<Vec<Point2d>>> = face_data.inner_edges.iter()
                .map(|edges| self.sample_edges_to_uv_polylines(edges, &face_data.surface))
                .collect();

            // Project each triangle's vertices to UV space for visualization
            let surface_ref = &face_data.surface;
            let uv_triangles: Vec<[Point2d; 3]> = face_mesh_with_ids.triangles.iter()
                .map(|tri| {
                    let v0 = face_mesh_with_ids.vertices[tri[0] as usize];
                    let v1 = face_mesh_with_ids.vertices[tri[1] as usize];
                    let v2 = face_mesh_with_ids.vertices[tri[2] as usize];
                    let (u0, v0v) = surface_ref.project_point(&v0);
                    let (u1, v1v) = surface_ref.project_point(&v1);
                    let (u2, v2v) = surface_ref.project_point(&v2);
                    [
                        Point2d::new(u0, v0v),
                        Point2d::new(u1, v1v),
                        Point2d::new(u2, v2v),
                    ]
                })
                .collect();

            face_infos.push(FaceInfo {
                face_id,
                step_face_id,
                surface_type,
                surface: face_data.surface.clone(),
                outer_boundary,
                inner_boundaries,
                outer_uv_boundary,
                inner_uv_boundaries,
                triangle_range: (tri_start, tri_end),
                forward: face_data.forward,
                uv_triangles,
                is_void: face_data.is_void,
            });
        }
        // Validation — do NOT apply repair_mesh (see comment in triangulate_brep).
        // Post-merge boundary vertex snapping for STEP files with disjoint VERTEX_POINT entities
        //
        // DISABLED: snap_boundary_vertices was corrupting valid triangulations by
        // snapping boundary vertices to interior vertices with an overly-aggressive
        // tolerance (1e-3 of bbox diagonal ≈ 0.4 units for typical parts). This
        // created massive numbers of degenerate triangles (e.g., 422 of 496 tris on
        // l-bracket_1) while only reducing boundary edges from 464 to 22.
        //
        // The correct approach is for the edge cache to produce bit-identical shared
        // vertices in the first place (which it does), so post-hoc snapping is not
        // needed. Remaining boundary edges indicate topology issues (missing faces,
        // STEP representation quirks) that should be fixed at the source, not papered
        // over with snapping.
        /*
        {
            let snap_tol = tol_ctx.model_scale * 1e-3;
            let pre_snap_degen = count_degenerate_triangles(&mesh);
            let pre_snap_boundary = count_boundary_edges(&mesh);
            let snapped = mesh.snap_boundary_vertices(snap_tol);
            let post_snap_degen = count_degenerate_triangles(&mesh);
            let post_snap_boundary = count_boundary_edges(&mesh);
            if snapped > 0 {
                log::warn!(
                    "BREP #{}: snapped {} boundary vertices (tol={:.2e}) — degen {}→{}, boundary {}→{}",
                    brep_id, snapped, snap_tol,
                    pre_snap_degen, post_snap_degen,
                    pre_snap_boundary, post_snap_boundary,
                );
            }
        }
        */
        // Filter degenerate triangles (zero area, NaN/Inf, or collapsed indices).
        // These create non-manifold edges and false boundary edges. The detailed
        // path was missing this call (only the non-detailed path had it), which
        // is why degenerate triangles appeared in the final mesh.
        let pre_filter_tris = mesh.triangle_count();
        filter_degenerate_triangles(&mut mesh, 1e-10);
        let post_filter_tris = mesh.triangle_count();
        let filter_removed = pre_filter_tris - post_filter_tris;
        eprintln!("POST_FILTER_DETAILED: BREP #{}: {} degenerate removed ({}→{})", brep_id, filter_removed, pre_filter_tris, post_filter_tris);

        // ─── Recompute face_infos.triangle_range from triangle_face_ids ───
        // After filter_degenerate_triangles removes triangles, the original
        // triangle_range values in FaceInfo are stale — they refer to the
        // pre-filter mesh indices. This causes the UV viewer and diagnostic
        // tools to display triangles from the WRONG face. Recompute ranges
        // from the surviving triangle_face_ids.
        if let Some(ref fids) = mesh.triangle_face_ids {
            // Build face_id → [start, end) mapping from the filtered mesh
            let mut fid_ranges: std::collections::HashMap<u64, (usize, usize)> = std::collections::HashMap::new();
            for (ti, &fid) in fids.iter().enumerate() {
                let entry = fid_ranges.entry(fid).or_insert((ti, ti));
                entry.1 = ti + 1;
            }
            for fi in &mut face_infos {
                if let Some(&(start, end)) = fid_ranges.get(&fi.face_id) {
                    if (fi.triangle_range.0, fi.triangle_range.1) != (start, end) {
                        log::debug!(
                            "FaceInfo #{} (STEP #{}): triangle_range updated [{},{}) → [{},{}) ({}→{} tris)",
                            fi.face_id, fi.step_face_id,
                            fi.triangle_range.0, fi.triangle_range.1,
                            start, end,
                            fi.triangle_range.1 - fi.triangle_range.0,
                            end - start,
                        );
                        fi.triangle_range = (start, end);
                    }
                } else {
                    // Face has no surviving triangles after filtering
                    fi.triangle_range = (0, 0);
                }
            }
        }

        // Weld boundary edge vertices to fix seam mismatches between
        // adjacent faces using different EDGE_CURVE entities.
        //
        // PASS 1 (short-edge welding) uses this tolerance — it must be large
        // enough to catch seam mismatches from different EDGE_CURVE entities
        // (up to ~5mm observed in some STEP files). 3% of model scale, capped
        // at 10mm, is the historical value that works for most STEP files.
        //
        // PASS 2 (long-edge vertex welding) uses a MUCH TIGHTER tolerance
        // internally (1% of this value, capped at 1e-3) to avoid welding
        // unrelated boundary vertices from different faces. See
        // weld_boundary_edge_vertices's PASS 2 comment for details.
        {
            // Check watertightness BEFORE welding — if the mesh is already
            // watertight via edge cache + merge_deduplicating, skip welding
            // to avoid creating degenerate triangles (Step#87/#78 lose 101/50
            // triangles when welding collapses boundary vertices).
            let report_before = validate_watertight(&mesh, false);
            if report_before.is_watertight() {
                eprintln!("WELD_SKIP: BREP #{} detailed already watertight ({} interior edges) — skipping weld to preserve triangles",
                    brep_id, report_before.interior_edge_count);
            } else {
                let weld_tol = (tol_ctx.model_scale * 3e-2).min(10.0).max(1e-4);
                let pre_weld_tris = mesh.triangle_count();
                weld_boundary_edge_vertices(&mut mesh, weld_tol);
                let post_weld_tris = mesh.triangle_count();
                eprintln!("POST_WELD_DETAILED: BREP #{}: tris after weld ({}→{})", brep_id, pre_weld_tris, post_weld_tris);
                // After welding, some triangles may have become degenerate (duplicate
                // vertices merged). Filter again before dedup.
                let pre_filter2 = mesh.triangle_count();
                filter_degenerate_triangles(&mut mesh, 1e-10);
                let post_filter2 = mesh.triangle_count();
                let filter2_removed = pre_filter2 - post_filter2;
                if filter2_removed > 0 {
                    eprintln!("POST_WELD_FILTER: BREP #{}: {} degenerate removed after welding ({}→{})", brep_id, filter2_removed, pre_filter2, post_filter2);
                }
            }
        }

        // Remove duplicate triangles (same 3 vertex indices). These arise when
        // two STEP faces overlap geometrically and share the same edges — common
        // in parts with threaded surfaces (e.g., nuts, bolts) where the thread
        // is represented as multiple NURBS faces covering the same region.
        // Duplicates create non-manifold edges (3+ triangles per edge).
        let dup_removed = mesh.remove_duplicate_triangles();
        eprintln!("POST_DEDUP_DETAILED: BREP #{}: {} duplicates removed ({}→{})", brep_id, dup_removed, mesh.triangle_count() + dup_removed, mesh.triangle_count());
        if dup_removed > 0 {
            log::info!(
                "BREP #{} detailed: removed {} duplicate/degenerate triangles ({} → {})",
                brep_id, dup_removed, mesh.triangle_count() + dup_removed, mesh.triangle_count(),
            );
            eprintln!("POST_DEDUP_DETAILED: BREP #{}: {} duplicate triangles removed", brep_id, dup_removed);
        }

        // ─── Recompute face_infos.triangle_range after remove_duplicate_triangles ───
        // remove_duplicate_triangles can remove triangles (degenerate or duplicate),
        // which shifts indices and makes the previously-recomputed triangle_range stale.
        // Without this, FaceInfo.triangle_range can reference out-of-bounds indices,
        // causing the UV viewer and face diagnostics to display wrong triangles.
        if dup_removed > 0 {
            if let Some(ref fids) = mesh.triangle_face_ids {
                let mut fid_ranges: std::collections::HashMap<u64, (usize, usize)> = std::collections::HashMap::new();
                for (ti, &fid) in fids.iter().enumerate() {
                    let entry = fid_ranges.entry(fid).or_insert((ti, ti));
                    entry.1 = ti + 1;
                }
                for fi in &mut face_infos {
                    if let Some(&(start, end)) = fid_ranges.get(&fi.face_id) {
                        fi.triangle_range = (start, end);
                    } else {
                        fi.triangle_range = (0, 0);
                    }
                }
            }
        }

        // ─── Post-merge boundary edge filling ──────────────────────────
        // DISABLED: The fill function adds triangles using vertices from other
        // faces, which creates overlapping triangles and non-manifold edges.
        // The proper fix is to ensure the edge cache shares vertices between
        // faces (via aliasing), not to fill gaps post-merge.
        /*
        {
            let max_fill = mesh.triangle_count();
            let filled = mesh.fill_boundary_edges(max_fill);
            if filled > 0 {
                log::info!(
                    "BREP #{} detailed: filled {} boundary edges ({} → {} tris)",
                    brep_id, filled, mesh.triangle_count() - filled, mesh.triangle_count(),
                );
                let dup_after_fill = mesh.remove_duplicate_triangles();
                if dup_after_fill > 0 {
                    log::info!(
                        "BREP #{} detailed: removed {} duplicate triangles after filling",
                        brep_id, dup_after_fill,
                    );
                }
            }
        }
        */

        // ─── Post-merge boundary vertex snapping (detailed path) ────────
        // DISABLED: snap_boundary_vertices with the world-placement-inflated
        // bbox creates massive numbers of duplicate triangles (301 on the bolt!)
        // and corrupts valid triangulations. The edge cache already produces
        // bit-identical shared vertices (100% edge consistency), so post-hoc
        // snapping is unnecessary. Remaining boundary edges come from STEP
        // topology issues (different EDGE_CURVE entities on the same geometric
        // boundary) that should be fixed by Phase 1/2 aliasing, not snapping.
        //
        // The snap tolerance formula `(model_scale * 2e-3)` uses the FULL file
        // bbox (including world placements), which inflates model_scale by 3x
        // for the bolt (122 vs 42 local), making snap_tol = 0.245mm — way too
        // aggressive for a 7.5mm radius part.
        /*
        {
            let snap_tol = (tol_ctx.model_scale * 2e-3).max(tol_ctx.absolute * 10.0);
            let snapped = mesh.snap_boundary_vertices(snap_tol);
            if snapped > 0 {
                log::info!(
                    "BREP #{} detailed: snapped {} boundary vertices (tol={:.2e})",
                    brep_id, snapped, snap_tol
                );
                filter_degenerate_triangles(&mut mesh, 1e-10);
                let dup_after_snap = mesh.remove_duplicate_triangles();
                if dup_after_snap > 0 {
                    log::info!(
                        "BREP #{} detailed: removed {} duplicate triangles after snapping",
                        brep_id, dup_after_snap,
                    );
                }
            }
        }
        */

        let adaptive_tol = edge_cache.adaptive_tolerance().merge_tolerance();
        let report_before = validate_watertight(&mesh, false);
        if !report_before.is_watertight() {
            let boundary_pct = if report_before.edge_count > 0 {
                report_before.boundary_edge_count as f64 / report_before.edge_count as f64 * 100.0
            } else {
                0.0
            };
            log::error!(
                "BUG: BREP #{} detailed not watertight: {} boundary edges ({:.2}%), {} non-manifold (tol={:.2e})",
                brep_id, report_before.boundary_edge_count, boundary_pct,
                report_before.non_manifold_edge_count, adaptive_tol
            );
            let deduped = total_face_vertices_detailed - mesh.vertices.len();
            let (exact_hits, tolerance_hits, misses) = dedup_map.stats();
            log::error!(
                "  Dedup stats: {} face vertices → {} unique ({} shared), dedup_rate={:.1}%, exact_hits={}, tolerance_hits={}, misses={}",
                total_face_vertices_detailed, mesh.vertices.len(), deduped,
                if total_face_vertices_detailed > 0 { deduped as f64 / total_face_vertices_detailed as f64 * 100.0 } else { 0.0 },
                exact_hits, tolerance_hits, misses,
            );
            if boundary_pct > 1.0 {
                log::error!("More than 1% boundary edges — edge cache is NOT working correctly!");
            }
            let consistency = validate_edge_consistency(&mesh, adaptive_tol);
            log::error!(
                "Edge consistency: {}/{} consistent, {} inconsistent ({:.2}%), max_dist={:.2e}",
                consistency.consistent_edges, consistency.shared_edges_checked,
                consistency.inconsistent_edges, consistency.inconsistency_rate(),
                consistency.max_vertex_distance
            );
            for inc in &consistency.worst_inconsistencies {
                log::error!(
                    "  Inconsistent edge: vertices ({}, {}), dist={:.2e}, faces={:?}",
                    inc.vertex_indices.0, inc.vertex_indices.1, inc.distance, inc.face_ids
                );
            }
            let stats = edge_cache.stats();
            log::info!("Edge cache stats: {} entries, {} hits, {} misses, {} shared, hit_rate={:.1}%",
                stats.total_edges, stats.cache_hits, stats.cache_misses, stats.shared_edges,
                if stats.cache_hits + stats.cache_misses > 0 {
                    stats.cache_hits as f64 / (stats.cache_hits + stats.cache_misses) as f64 * 100.0
                } else { 0.0 }
            );
        }

        // Smooth vertex normals across shared edges for Gouraud-like shading.
        // Use a crease angle of 45° (0.785 rad) — edges sharper than this
        // maintain their sharp appearance (e.g., box edges), while smoother
        // transitions (cylinders, spheres) get averaged normals.
        // TODO: Use smooth_normals_adaptive with a Solid reference when available
        // in this context, for surface-type-specific crease angles.
        draper_mesh::smooth_normals(&mut mesh, 0.785);

        // Recompute face normals from the FINAL vertex positions.
        // This is necessary because post-processing steps (weld_boundary_edge_vertices,
        // merge_coincident_vertices, filter_degenerate_triangles) may have moved vertices
        // or removed/added triangles, making the original face_normals (computed from
        // pre-merge geometry or padded with (0,0,1) defaults) inconsistent with the
        // current vertex positions. Without this recomputation, lighting artifacts
        // appear on faces like cones and cylinders that lacked face_normals in their
        // per-face meshes (the structured grid triangulation functions don't set them).
        mesh.compute_face_normals();

        if skipped_faces > 0 {
            log::warn!("BREP #{} detailed: {} faces skipped due to time limit", brep_id, skipped_faces);
        }

        // Phase 2: Validate watertightness of the merged mesh
        let wt_report = validate_watertight(&mesh, false);
        if wt_report.is_watertight() {
            log::info!("BREP #{} detailed: watertight ✓ ({} interior edges, Euler χ={})",
                brep_id, wt_report.interior_edge_count, wt_report.euler_characteristic);
        } else {
            log::warn!("BREP #{} detailed: NOT watertight — {} boundary edges, {} non-manifold edges, χ={}",
                brep_id, wt_report.boundary_edge_count, wt_report.non_manifold_edge_count,
                wt_report.euler_characteristic);
            if wt_report.degenerate_triangle_count > 0 {
                log::warn!("BREP #{} detailed: {} degenerate triangles", brep_id, wt_report.degenerate_triangle_count);
            }
        }

        log::info!("BREP #{} detailed: edge_cache={} entries, mesh v={} t={} skipped={}",
            brep_id, edge_cache.len(), mesh.vertex_count(), mesh.triangle_count(), skipped_faces);

        // ─── FINAL recomputation of face_infos.triangle_range ───
        // After ALL post-processing (filter, weld, dedup, smooth), the triangle
        // indices may have shifted. Recompute from the final triangle_face_ids
        // to guarantee FaceInfo.triangle_range matches the actual mesh layout.
        // This is the DEFINITIVE recomputation — any earlier recomputations
        // may have been invalidated by subsequent processing steps.
        if let Some(ref fids) = mesh.triangle_face_ids {
            let mut fid_ranges: std::collections::HashMap<u64, (usize, usize)> = std::collections::HashMap::new();
            for (ti, &fid) in fids.iter().enumerate() {
                let entry = fid_ranges.entry(fid).or_insert((ti, ti));
                entry.1 = ti + 1;
            }
            for fi in &mut face_infos {
                if let Some(&(start, end)) = fid_ranges.get(&fi.face_id) {
                    fi.triangle_range = (start, end);
                } else {
                    fi.triangle_range = (0, 0);
                }
            }
        }

        Some((mesh, face_infos))
    }

    /// Prepare a BREP triangulation session — extracts faces, heals, builds
    /// edge cache + alias map. This is the setup phase of `triangulate_brep_detailed`,
    /// extracted so it can run ONCE per BREP, with face processing chunked across
    /// multiple frames via `BrepSession::process_one_face`.
    ///
    /// Returns `None` if the BREP has no shell or no faces.
    fn prepare_brep_session(
        &self,
        brep_id: i64,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<BrepSession> {
        // P7: Use find_all_shell_refs to support BREP_WITH_VOIDS.
        let (outer_shell_id, void_shell_ids) = self.find_all_shell_refs_by_brep_id(brep_id);
        let shell_id = outer_shell_id?;
        let mut face_data_list = self.extract_shell_faces(shell_id, false)?;

        // Extract faces from each void shell and append them to the list.
        for void_shell_id in &void_shell_ids {
            match self.extract_shell_faces(*void_shell_id, true) {
                Some(void_faces) => {
                    log::info!(
                        "BREP #{}: appending {} faces from void shell #{}",
                        brep_id, void_faces.len(), void_shell_id
                    );
                    face_data_list.extend(void_faces);
                }
                None => {
                    log::warn!(
                        "BREP #{}: failed to extract faces from void shell #{} — skipping",
                        brep_id, void_shell_id
                    );
                }
            }
        }

        // Log face_data_list composition
        {
            let mut surface_counts: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
            for fd in &face_data_list {
                let key = match &fd.surface {
                    Surface::Plane(_) => "Plane",
                    Surface::Cylinder(_) => "Cylinder",
                    Surface::Cone(_) => "Cone",
                    Surface::Sphere(_) => "Sphere",
                    Surface::Torus(_) => "Torus",
                    Surface::Revolution(_) => "Revolution",
                    Surface::Extrusion(_) => "Extrusion",
                    Surface::Nurbs(_) => "Nurbs",
                };
                *surface_counts.entry(key).or_insert(0) += 1;
            }
            let summary: Vec<String> = surface_counts.iter()
                .map(|(k, v)| format!("{}={}", k, v))
                .collect();
            log::info!(
                "BREP #{} face_data_list: {} faces [{}]",
                brep_id, face_data_list.len(), summary.join(", "),
            );
        }

        // Create tolerance context for this BREP
        let tol_ctx = match bbox {
            Some((bmin, bmax)) => ToleranceContext::from_bounding_box(bmin, bmax),
            None => ToleranceContext::new(),
        };

        // ─── Healing pipeline: heal the solid before triangulation ────────
        let face_data_list = if self.config.heal {
            let pre_heal_count = face_data_list.len();
            let (solid, face_id_map) = face_data_list_to_solid(&face_data_list);
            let healing_params = HealingParams::aggressive_with_context(&tol_ctx);
            let (healed, report) = heal_solid(&solid, &healing_params);
            log_healing_report(brep_id, &report);
            let healed_list = apply_healing_to_face_data(&face_data_list, &healed, &face_id_map);
            if healed_list.len() != pre_heal_count {
                log::warn!(
                    "BREP #{} healing changed face count: {} → {}",
                    brep_id, pre_heal_count, healed_list.len(),
                );
            }
            healed_list
        } else {
            face_data_list
        };

        // ─── Validate BREP topology (5.2.3) ─────────────────────────────
        // Run topology validation and log warnings. This is diagnostic only —
        // it does NOT block triangulation. Issues are logged for developer
        // awareness and debugging.
        {
            let (solid, _) = face_data_list_to_solid(&face_data_list);
            let report = validate_brep(&solid, &TopologyValidationConfig::critical_only());
            if !report.is_clean() {
                log::warn!(
                    "BREP #{} topology validation: {}",
                    brep_id, report.summary()
                );
                // Log individual error-level issues
                for issue in report.detailed.issues.iter()
                    .filter(|i| i.severity == draper_topology::validation::Severity::Error)
                    .take(10) // Cap at 10 to avoid log spam
                {
                    log::warn!("  [{}] {}{}: {}",
                        issue.severity, issue.check,
                        issue.entity_id.map(|id| format!(" {}", id)).unwrap_or_default(),
                        issue.message
                    );
                }
            } else {
                log::info!(
                    "BREP #{} topology validation: clean ({} faces, {} edges, Euler={})",
                    brep_id, report.face_count, report.edge_count, report.euler_characteristic
                );
            }
        }

        // Create edge discretization cache for this BREP
        let mut edge_cache = EdgeDiscretizationCache::with_tolerance(tol_ctx.clone(), 64);

        // ─── Build vertex-pair → canonical step_id aliases ──────────────
        // (Same logic as triangulate_brep_detailed — see comments there.)
        {
            let mut vertex_pair_to_step_ids: HashMap<(i64, i64), Vec<i64>> = HashMap::new();
            for face_data in &face_data_list {
                for &step_id in &face_data.edge_step_ids {
                    if step_id == 0 { continue; }
                    if let Some(vp) = self.get_edge_curve_vertex_pair(step_id) {
                        vertex_pair_to_step_ids.entry(vp).or_default().push(step_id);
                    }
                }
            }
            let mut alias_count = 0usize;
            let mut skipped_different_curves = 0usize;
            for (_vp, step_ids) in &vertex_pair_to_step_ids {
                if step_ids.len() < 2 { continue; }
                let shape_tol = (tol_ctx.model_scale * 1e-3).max(1e-6);
                let shape_groups = self.group_step_ids_by_curve_shape(step_ids, shape_tol);
                for (_samples, group_sids) in &shape_groups {
                    if group_sids.len() < 2 { continue; }
                    let canonical = *group_sids.iter().max_by_key(|&&sid| {
                        self.edge_curve_complexity_score(sid)
                    }).unwrap();
                    for &sid in group_sids {
                        if sid != canonical {
                            edge_cache.register_step_id_alias(sid, canonical);
                            alias_count += 1;
                        }
                    }
                }
                if shape_groups.len() > 1 {
                    skipped_different_curves += step_ids.len() - shape_groups.iter().map(|(_, g)| g.len().min(1)).sum::<usize>();
                }
            }
            if alias_count > 0 || skipped_different_curves > 0 {
                log::info!(
                    "BREP #{}: registered {} step_id aliases from vertex-pair matching (chunked, skipped {} edges with same endpoints but different curves)",
                    brep_id, alias_count, skipped_different_curves,
                );
            }

            // Phase 2: 3D coordinate-based aliasing (supplementary)
            let coord_tol = (tol_ctx.model_scale * 2e-3).max(tol_ctx.absolute * 10.0);
            let mut coord_pair_to_step_ids: HashMap<(i64, i64, i64, i64, i64, i64), Vec<i64>> = HashMap::new();
            for face_data in &face_data_list {
                for (edge_idx, &step_id) in face_data.edge_step_ids.iter().enumerate() {
                    if step_id == 0 { continue; }
                    if edge_cache.resolve_canonical_step_id(step_id) != step_id {
                        continue;
                    }
                    let edge = &face_data.edges[edge_idx];
                    let start = match edge.start_point() { Some(p) => p, None => continue };
                    let end = match edge.end_point() { Some(p) => p, None => continue };
                    let sk = (
                        (start.x / coord_tol).round() as i64,
                        (start.y / coord_tol).round() as i64,
                        (start.z / coord_tol).round() as i64,
                        (end.x / coord_tol).round() as i64,
                        (end.y / coord_tol).round() as i64,
                        (end.z / coord_tol).round() as i64,
                    );
                    let sk_rev = (
                        (end.x / coord_tol).round() as i64,
                        (end.y / coord_tol).round() as i64,
                        (end.z / coord_tol).round() as i64,
                        (start.x / coord_tol).round() as i64,
                        (start.y / coord_tol).round() as i64,
                        (start.z / coord_tol).round() as i64,
                    );
                    let canonical_key = if sk <= sk_rev { sk } else { sk_rev };
                    coord_pair_to_step_ids.entry(canonical_key).or_default().push(step_id);
                }
            }
            let mut coord_alias_count = 0usize;
            let mut coord_groups_with_multiple = 0usize;
            let mut coord_skipped_different_curves = 0usize;
            for (_key, step_ids) in &coord_pair_to_step_ids {
                if step_ids.len() < 2 { continue; }
                coord_groups_with_multiple += 1;
                let shape_tol = (tol_ctx.model_scale * 1e-3).max(1e-6);
                let shape_groups = self.group_step_ids_by_curve_shape(step_ids, shape_tol);
                for (_samples, group_sids) in &shape_groups {
                    if group_sids.len() < 2 { continue; }
                    let canonical = *group_sids.iter().max_by_key(|&&sid| {
                        self.edge_curve_complexity_score(sid)
                    }).unwrap();
                    for &sid in group_sids {
                        if sid != canonical {
                            edge_cache.register_step_id_alias(sid, canonical);
                            coord_alias_count += 1;
                        }
                    }
                }
                if shape_groups.len() > 1 {
                    coord_skipped_different_curves += step_ids.len() - shape_groups.iter().map(|(_, g)| g.len().min(1)).sum::<usize>();
                }
            }
            log::info!(
                "BREP #{}: Phase 2 alias (chunked): {} coord groups, {} with multiple step_ids, {} aliases registered, {} skipped different curves (tol={:.2e})",
                brep_id, coord_pair_to_step_ids.len(), coord_groups_with_multiple, coord_alias_count, coord_skipped_different_curves, coord_tol,
            );
        }

        // Time guard: limit per-BREP triangulation time.
        // Use override from TriangulationParams if provided, otherwise use
        // platform-specific defaults (30s WASM / 600s native).
        #[cfg(target_arch = "wasm32")]
        let default_brep_time_limit = std::time::Duration::from_secs(30);
        #[cfg(not(target_arch = "wasm32"))]
        let default_brep_time_limit = std::time::Duration::from_secs(600);
        let brep_time_limit = params.brep_time_limit_override.unwrap_or(default_brep_time_limit);

        #[cfg(target_arch = "wasm32")]
        let default_face_time_limit = std::time::Duration::from_secs(3);
        #[cfg(not(target_arch = "wasm32"))]
        let default_face_time_limit = std::time::Duration::from_secs(120);
        let face_time_limit = params.face_time_limit_override.unwrap_or(default_face_time_limit);

        // Tolerance-based dedup
        let merge_tol = (tol_ctx.model_scale * 1e-4).max(tol_ctx.absolute * 10.0);
        let dedup_map = draper_mesh::mesh::VertexDedupMap::with_tolerance(merge_tol);

        Some(BrepSession {
            brep_id,
            face_data_list,
            edge_cache,
            tol_ctx,
            dedup_map,
            mesh: TriangleMesh::new(),
            face_infos: Vec::new(),
            next_face_id: 1,
            next_face_idx: 0,
            total_face_vertices: 0,
            skipped_faces: 0,
            brep_start: StdInstant::now(),
            brep_time_limit,
            face_time_limit,
            params: params.clone(),
            bbox: bbox.clone(),
        })
    }

    /// Sample edges into 3D polylines for boundary visualization.
    fn sample_edges_to_polylines(&self, edges: &[TopoEdge]) -> Vec<Point3d> {
        let mut points = Vec::new();
        for edge in edges {
            if let Some(ref _curve) = edge.curve {
                let steps = 20;
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    if let Some(p) = edge.point_at(t) {
                        let p = deterministic_round_point(p);
                        if points.last().map_or(true, |last: &Point3d| last.distance_to(&p) > 1e-8) {
                            points.push(p);
                        }
                    }
                }
            }
        }
        points
    }

    /// Sample edges into UV-space polylines for UV grid visualization.
    /// All edges in a single loop are concatenated into one polyline (matching
    /// the behavior of `sample_edges_to_polylines`), with deduplication at
    /// junction points to avoid duplicate vertices where edges meet.
    fn sample_edges_to_uv_polylines(&self, edges: &[TopoEdge], surface: &Surface) -> Vec<Vec<Point2d>> {
        let mut polyline = Vec::new();
        for edge in edges {
            if let Some(ref _curve) = edge.curve {
                let steps = 20;
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    if let Some(p) = edge.point_at(t) {
                        let p = deterministic_round_point(p);
                        let (u, v) = surface.project_point(&p);
                        let pt = Point2d::new(u, v);
                        // Deduplicate: skip if same as last point (at edge junctions)
                        if polyline.last().map_or(true, |last: &Point2d| {
                            (last.u - pt.u).abs() > 1e-8 || (last.v - pt.v).abs() > 1e-8
                        }) {
                            polyline.push(pt);
                        }
                    }
                }
            }
        }
        if polyline.is_empty() { vec![] } else { vec![polyline] }
    }

    /// Triangulate a shell entity (CLOSED_SHELL, OPEN_SHELL) directly by its ID.
    fn triangulate_shell_by_id(
        &self,
        shell_id: i64,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<TriangleMesh> {
        let face_data_list = self.extract_shell_faces(shell_id, false)?;

        // Create tolerance context for healing
        let tol_ctx = match bbox {
            Some((bmin, bmax)) => ToleranceContext::from_bounding_box(bmin, bmax),
            None => ToleranceContext::new(),
        };

        // ─── Healing pipeline ────────
        let face_data_list = if self.config.heal {
            let (solid, face_id_map) = face_data_list_to_solid(&face_data_list);
            let healing_params = HealingParams::aggressive_with_context(&tol_ctx);
            let (healed, report) = heal_solid(&solid, &healing_params);
            log_healing_report(shell_id, &report);
            apply_healing_to_face_data(&face_data_list, &healed, &face_id_map)
        } else {
            face_data_list
        };

        let mut mesh = TriangleMesh::new();
        // Tolerance-based dedup: catches near-identical vertices from different
        // STEP EDGE_CURVE entities on the same geometric boundary.
        let merge_tol = (tol_ctx.model_scale * 1e-4).max(tol_ctx.absolute * 10.0);
        let mut dedup_map = draper_mesh::mesh::VertexDedupMap::with_tolerance(merge_tol);
        for face_data in &face_data_list {
            let face_mesh = self.surface_to_mesh(face_data, params, bbox);
            mesh.merge_deduplicating(&face_mesh, &mut dedup_map);
        }
        if mesh.vertex_count() == 0 { None } else { Some(mesh) }
    }

    /// Extract the NAUO instance name (e.g., "nut_1", "bolt_2").
    fn extract_nauo_name(&self, nauo: &crate::schema::StepEntity) -> String {
        // NEXT_ASSEMBLY_USAGE_OCCURRENCE('id','name','description',#relating,#related,$)
        //
        // Many real-world STEP exporters (e.g., SIEMENS NX) leave the 'name'
        // field (2nd param) as a single space ' ' and put the actual part
        // identifier in the 'description' field (3rd param). Naively returning
        // the first non-empty string returns the whitespace, which produces
        // blank-looking node names like "  (BREP#1068)" in the structure tree.
        //
        // Strategy: collect every non-blank trimmed string (skipping the ID at
        // index 0) and prefer the LAST one. The description field is the last
        // string-typed parameter before the ref list, so it wins ties. If no
        // non-blank string is present, fall back to a synthetic name.
        let mut candidates: Vec<String> = Vec::new();
        for (i, param) in nauo.params.iter().enumerate() {
            if i == 0 { continue; } // Skip ID
            if let StepValue::String(s) = param {
                let trimmed = s.trim();
                if !trimmed.is_empty() {
                    candidates.push(trimmed.to_string());
                }
            }
        }
        if let Some(name) = candidates.last() {
            return name.clone();
        }
        format!("NAUO_{}", nauo.id)
    }

    /// Get a human-readable product name from a PRODUCT_DEFINITION.
    fn get_product_name(&self, pd_id: i64) -> String {
        // PRODUCT_DEFINITION('design','',#product_formation,#context)
        // → #product_formation is PRODUCT_DEFINITION_FORMATION('','',#product)
        // → #product is PRODUCT('id', 'name', ...)
        // The chain is: PD → PDF → PRODUCT → name
        let pd = match self.step.find_entity(pd_id) {
            Some(e) => e,
            None => return format!("PD#{}", pd_id),
        };

        // Search for PRODUCT_DEFINITION_FORMATION reference in PD params
        for param in &pd.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(pdf) = self.step.find_entity(ref_id) {
                    // Direct PRODUCT reference (some exporters skip the PDF layer).
                    if pdf.type_name == "PRODUCT" {
                        for p in &pdf.params {
                            if let StepValue::String(s) = p {
                                let t = s.trim();
                                if !t.is_empty() {
                                    return t.to_string();
                                }
                            }
                        }
                    }
                    // Follow PD → PDF → PRODUCT chain.
                    // STEP defines both `PRODUCT_DEFINITION_FORMATION` and
                    // `PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE`
                    // (the latter is what SIEMENS NX and most modern exporters
                    // emit). Match by prefix so both forms work, plus any
                    // future `..._WITH_*` variants the standard may add.
                    if pdf.type_name == "PRODUCT_DEFINITION_FORMATION"
                        || pdf.type_name.starts_with("PRODUCT_DEFINITION_FORMATION")
                    {
                        for p in &pdf.params {
                            if let Some(product_id) = self.get_ref(p) {
                                if let Some(product) = self.step.find_entity(product_id) {
                                    if product.type_name == "PRODUCT" {
                                        // The PRODUCT entity has the form
                                        //   PRODUCT('id', 'name', 'description', ...)
                                        // Prefer the first non-blank trimmed string,
                                        // which is typically the part identifier.
                                        let mut picked: Option<String> = None;
                                        for pp in &product.params {
                                            if let StepValue::String(s) = pp {
                                                let t = s.trim();
                                                if !t.is_empty() {
                                                    picked = Some(t.to_string());
                                                    break;
                                                }
                                            }
                                        }
                                        if let Some(name) = picked {
                                            return name;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        format!("PD#{}", pd_id)
    }

    /// Get a name for a BREP from its first parameter.
    fn get_brep_name(&self, brep_id: i64) -> String {
        if let Some(brep) = self.step.find_entity(brep_id) {
            for param in &brep.params {
                if let StepValue::String(s) = param {
                    if !s.is_empty() {
                        return format!("{} (#{})", s, brep_id);
                    }
                }
            }
        }
        format!("BREP#{}", brep_id)
    }

    // ─── Assembly tree traversal ────────────────────────────────────────────

    /// Extract relating and related PRODUCT_DEFINITION IDs from a NAUO entity.
    fn extract_nauo_pd_refs(&self, nauo: &crate::schema::StepEntity) -> (Option<i64>, Option<i64>) {
        let mut pd_refs: Vec<i64> = Vec::new();
        for param in &nauo.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "PRODUCT_DEFINITION" {
                        pd_refs.push(ref_id);
                    }
                }
            }
        }
        (pd_refs.get(0).copied(), pd_refs.get(1).copied())
    }

    /// Find the transform for a NAUO instance by walking CDSR → SRR → ITEM_DEFINED_TRANSFORMATION.
    fn find_nauo_transform(&self, nauo_id: i64, _related_pd_id: i64) -> Option<[[f64; 4]; 4]> {
        // Use pre-built index for O(1) lookup
        self.nauo_transform_map.get(&nauo_id).copied().flatten()
    }

    /// Actual find_nauo_transform computation (uncached).
    fn find_nauo_transform_uncached(&self, nauo_id: i64, _related_pd_id: i64) -> Option<[[f64; 4]; 4]> {
        let cdsrs = self.step.find_entities_by_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION");
        for cdsr in &cdsrs {
            let linked = self.cdsr_links_to_nauo(cdsr, nauo_id);
            if !linked { continue; }

            if let Some(srr_id) = self.get_ref(cdsr.params.first()?) {
                if let Some(srr_entity) = self.step.find_entity(srr_id) {
                    return self.extract_transform_from_srr(&srr_entity);
                }
            }
        }
        None
    }

    /// Check if a CDSR links to a specific NAUO through PRODUCT_DEFINITION_SHAPE.
    fn cdsr_links_to_nauo(&self, cdsr: &crate::schema::StepEntity, nauo_id: i64) -> bool {
        for (i, param) in cdsr.params.iter().enumerate() {
            if i == 0 { continue; }
            if let Some(pds_id) = self.get_ref(param) {
                if let Some(pds) = self.step.find_entity(pds_id) {
                    for p in &pds.params {
                        if let Some(nid) = self.get_ref(p) {
                            if nid == nauo_id { return true; }
                            if let Some(inner) = self.step.find_entity(nid) {
                                for ip in &inner.params {
                                    if let Some(ref_id) = self.get_ref(ip) {
                                        if ref_id == nauo_id { return true; }
                                    }
                                    if let StepValue::List(items) = ip {
                                        for item in items {
                                            if let Some(ref_id) = self.get_ref(item) {
                                                if ref_id == nauo_id { return true; }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        false
    }

    /// Extract the 4x4 transform from a SHAPE_REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION.
    /// Handles both simple SRR entities and complex/composite entities
    /// (e.g., REPRESENTATION_RELATIONSHIP+REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION+SHAPE_REPRESENTATION_RELATIONSHIP).
    ///
    /// NOTE: Returns the "raw" transform M = o * t⁻¹ which maps from rep_2 → rep_1.
    /// The caller must determine the direction (child → parent or parent → child)
    /// based on the SRR/NAUO relationship. For the cached path used by the viewer,
    /// `build_nauo_transform_index` handles this automatically by detecting the
    /// convention and inverting when needed.
    fn extract_transform_from_srr(&self, srr: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        // First: check if this is a complex entity with a REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION sub-entity
        if srr.is_complex() {
            if let Some(rrwt_sub) = srr.find_sub_entity("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
                // The RRWT sub-entity has a single parameter: the reference to ITEM_DEFINED_TRANSFORMATION
                for param in &rrwt_sub.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                                return self.compute_item_defined_transform(&entity);
                            }
                            // Support CARTESIAN_TRANSFORMATION_OPERATOR_3D
                            if entity.type_name == "CARTESIAN_TRANSFORMATION_OPERATOR_3D" {
                                return self.compute_cartesian_transform(&entity);
                            }
                        }
                    }
                }
            }
        }

        // Fallback: search all params for direct reference to ITEM_DEFINED_TRANSFORMATION or CTO3D
        for param in &srr.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                        return self.compute_item_defined_transform(&entity);
                    }
                    if entity.type_name == "CARTESIAN_TRANSFORMATION_OPERATOR_3D" {
                        return self.compute_cartesian_transform(&entity);
                    }
                    // Also search inside nested entities
                    for inner_param in &entity.params {
                        if let Some(inner_id) = self.get_ref(inner_param) {
                            if let Some(inner_entity) = self.step.find_entity(inner_id) {
                                if inner_entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                                    return self.compute_item_defined_transform(&inner_entity);
                                }
                                if inner_entity.type_name == "CARTESIAN_TRANSFORMATION_OPERATOR_3D" {
                                    return self.compute_cartesian_transform(&inner_entity);
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Compute a 4x4 transform from ITEM_DEFINED_TRANSFORMATION(origin_axis2, target_axis2).
    ///
    /// NOTE: This returns the "raw" transform M = o * t⁻¹ which maps from rep_2 → rep_1.
    /// The caller must determine the direction (child → parent or parent → child)
    /// based on which representation in the SRR is the parent. For the cached path
    /// used by the viewer, `build_nauo_transform_index` handles this automatically.
    fn compute_item_defined_transform(&self, idt: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        let mut axis2_ids: Vec<i64> = Vec::new();
        for (i, param) in idt.params.iter().enumerate() {
            if i < 2 { continue; }
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "AXIS2_PLACEMENT_3D" {
                        axis2_ids.push(ref_id);
                    }
                }
            }
        }

        if axis2_ids.len() < 2 {
            warn!("ITEM_DEFINED_TRANSFORMATION has {} axis2 refs (need 2)", axis2_ids.len());
            return None;
        }

        let (origin_pt, origin_z, origin_x) = self.resolve_axis2(axis2_ids[0])?;
        let (target_pt, target_z, target_x) = self.resolve_axis2(axis2_ids[1])?;

        let origin_y = origin_z.cross(&origin_x);
        let target_y = target_z.cross(&target_x);

        let o = [
            [origin_x.x, origin_y.x, origin_z.x, origin_pt.x],
            [origin_x.y, origin_y.y, origin_z.y, origin_pt.y],
            [origin_x.z, origin_y.z, origin_z.z, origin_pt.z],
            [0.0, 0.0, 0.0, 1.0],
        ];

        let t = [
            [target_x.x, target_y.x, target_z.x, target_pt.x],
            [target_x.y, target_y.y, target_z.y, target_pt.y],
            [target_x.z, target_y.z, target_z.z, target_pt.z],
            [0.0, 0.0, 0.0, 1.0],
        ];

        // The IDT defines: the coordinate system described by transform_item_1 (o)
        // in rep_1's space is IDENTICAL to the coordinate system described by
        // transform_item_2 (t) in rep_2's space.
        //
        // M = o * t⁻¹ maps from rep_2 → rep_1.
        // The caller must determine whether this maps child → parent or vice versa.
        let t_inv = mat4_inverse(&t)?;
        let result = mat4_mul(&o, &t_inv);
        Some(result)
    }

    /// Compute a 4×4 transform from CARTESIAN_TRANSFORMATION_OPERATOR_3D.
    ///
    /// CTO3D directly specifies a coordinate system (origin + local axes) with an
    /// optional scale factor. This is an alternative to ITEM_DEFINED_TRANSFORMATION
    /// used by some STEP exporters.
    fn compute_cartesian_transform(&self, cto: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        let mut origin = [0.0_f64; 3];
        let mut axis1: Option<[f64; 3]> = None;
        let mut axis2: Option<[f64; 3]> = None;
        let mut axis3: Option<[f64; 3]> = None;
        let mut scale = 1.0_f64;

        for (i, param) in cto.params.iter().enumerate() {
            match i {
                0 => { /* name — skip */ }
                1 => {
                    // origin — CARTESIAN_POINT reference
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(cp) = self.step.find_entity(ref_id) {
                            for cp_param in &cp.params {
                                if let StepValue::List(coords) = cp_param {
                                    let x = coords.get(0).and_then(|v| self.get_float(v))?;
                                    let y = coords.get(1).and_then(|v| self.get_float(v))?;
                                    let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                                    origin = [x, y, z];
                                    break;
                                }
                            }
                        }
                    }
                }
                2 => { /* axis1 (u direction) */ axis1 = self.get_direction_from_param(param); }
                3 => { /* axis2 (v direction) */ axis2 = self.get_direction_from_param(param); }
                4 => {
                    // scale
                    if let Some(s) = self.get_float(param) {
                        scale = s;
                    }
                }
                5 => { /* axis3 (w direction / Z) */ axis3 = self.get_direction_from_param(param); }
                _ => {}
            }
        }

        // Build orthogonal coordinate frame
        let z = axis3.unwrap_or([0.0, 0.0, 1.0]);
        let x_raw = axis1.unwrap_or_else(|| {
            if z[2].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] }
        });
        let y_raw = axis2.unwrap_or_else(|| {
            let y = [
                z[1] * x_raw[2] - z[2] * x_raw[1],
                z[2] * x_raw[0] - z[0] * x_raw[2],
                z[0] * x_raw[1] - z[1] * x_raw[0],
            ];
            let len = (y[0]*y[0] + y[1]*y[1] + y[2]*y[2]).sqrt();
            if len > 1e-10 { [y[0]/len, y[1]/len, y[2]/len] } else { [0.0, 1.0, 0.0] }
        });

        // Project X onto plane perpendicular to Z (STEP spec requirement)
        let dot_xz = x_raw[0]*z[0] + x_raw[1]*z[1] + x_raw[2]*z[2];
        let x_proj = [x_raw[0] - dot_xz*z[0], x_raw[1] - dot_xz*z[1], x_raw[2] - dot_xz*z[2]];
        let x_len = (x_proj[0]*x_proj[0] + x_proj[1]*x_proj[1] + x_proj[2]*x_proj[2]).sqrt();
        let x = if x_len > 1e-10 { [x_proj[0]/x_len, x_proj[1]/x_len, x_proj[2]/x_len] } else { x_raw };

        // Recompute Y = Z × X to ensure orthogonality
        let y = [
            z[1]*x[2] - z[2]*x[1],
            z[2]*x[0] - z[0]*x[2],
            z[0]*x[1] - z[1]*x[0],
        ];
        let y_len = (y[0]*y[0] + y[1]*y[1] + y[2]*y[2]).sqrt();
        let y = if y_len > 1e-10 { [y[0]/y_len, y[1]/y_len, y[2]/y_len] } else { y_raw };

        // Normalize Z
        let z_len = (z[0]*z[0] + z[1]*z[1] + z[2]*z[2]).sqrt();
        let z = if z_len > 1e-10 { [z[0]/z_len, z[1]/z_len, z[2]/z_len] } else { z };

        // Build 4×4 matrix with scale
        Some([
            [x[0] * scale, y[0] * scale, z[0] * scale, origin[0]],
            [x[1] * scale, y[1] * scale, z[1] * scale, origin[1]],
            [x[2] * scale, y[2] * scale, z[2] * scale, origin[2]],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    /// Helper: extract a [f64;3] direction from a DIRECTION entity referenced by a StepValue param.
    fn get_direction_from_param(&self, param: &StepValue) -> Option<[f64; 3]> {
        if let Some(ref_id) = self.get_ref(param) {
            if let Some(dir_entity) = self.step.find_entity(ref_id) {
                if dir_entity.type_name == "DIRECTION" {
                    for dir_param in &dir_entity.params {
                        if let StepValue::List(coords) = dir_param {
                            let x = coords.get(0).and_then(|v| self.get_float(v))?;
                            let y = coords.get(1).and_then(|v| self.get_float(v))?;
                            let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                            let len = (x*x + y*y + z*z).sqrt();
                            if len > 1e-10 {
                                return Some([x/len, y/len, z/len]);
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Find the MANIFOLD_SOLID_BREP associated with a PRODUCT_DEFINITION.
    /// Uses the pre-built pd_brep_map for O(1) lookup.
    fn find_pd_brep(&self, pd_id: i64) -> Option<i64> {
        self.pd_brep_map.get(&pd_id).copied().flatten()
    }

    /// Actual find_pd_brep computation (uncached).
    fn find_pd_brep_uncached(&self, pd_id: i64) -> Option<i64> {
        let _pd = self.step.find_entity(pd_id)?;

        for pds in self.step.find_entities_by_type("PRODUCT_DEFINITION_SHAPE") {
            let mut refs_our_pd = false;
            for param in &pds.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if ref_id == pd_id { refs_our_pd = true; break; }
                }
            }
            if !refs_our_pd { continue; }

            for sdr in self.step.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION") {
                let mut refs_our_pds = false;
                for param in &sdr.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if ref_id == pds.id { refs_our_pds = true; break; }
                    }
                }
                if !refs_our_pds { continue; }

                for param in &sdr.params {
                    if let Some(sr_id) = self.get_ref(param) {
                        // Direct: SR is an ADVANCED_BREP_SHAPE_REPRESENTATION
                        if let Some(brep_id) = self.find_brep_in_representation(sr_id) {
                            return Some(brep_id);
                        }
                        // Indirect: SR is a SHAPE_REPRESENTATION linked to ABSR via SRR
                        if let Some(brep_id) = self.find_brep_via_srr(sr_id, 0) {
                            return Some(brep_id);
                        }
                    }
                }
            }
        }
        None
    }

    /// Find a MANIFOLD_SOLID_BREP inside a SHAPE_REPRESENTATION or ADVANCED_BREP_SHAPE_REPRESENTATION.
    fn find_brep_in_representation(&self, sr_id: i64) -> Option<i64> {
        let sr = self.step.find_entity(sr_id)?;
        if sr.type_name.contains("ADVANCED_BREP_SHAPE_REPRESENTATION") {
            for sp in &sr.params {
                if let Some(brep_id) = self.get_ref(sp) {
                    if let Some(brep) = self.step.find_entity(brep_id) {
                        if brep.type_name == "MANIFOLD_SOLID_BREP" {
                            return Some(brep_id);
                        }
                    }
                }
                if let StepValue::List(items) = sp {
                    for item in items {
                        if let Some(brep_id) = self.get_ref(item) {
                            if let Some(brep) = self.step.find_entity(brep_id) {
                                if brep.type_name == "MANIFOLD_SOLID_BREP" {
                                    return Some(brep_id);
                                }
                            }
                        }
                    }
                }
            }
        }
        // Also check FACETED_BREP
        if sr.type_name.contains("FACETED_BREP_SHAPE_REPRESENTATION") {
            for sp in &sr.params {
                if let Some(brep_id) = self.get_ref(sp) {
                    if let Some(brep) = self.step.find_entity(brep_id) {
                        if brep.type_name == "FACETED_BREP" || brep.type_name == "MANIFOLD_SOLID_BREP" {
                            return Some(brep_id);
                        }
                    }
                }
                if let StepValue::List(items) = sp {
                    for item in items {
                        if let Some(brep_id) = self.get_ref(item) {
                            if let Some(brep) = self.step.find_entity(brep_id) {
                                if brep.type_name == "FACETED_BREP" || brep.type_name == "MANIFOLD_SOLID_BREP" {
                                    return Some(brep_id);
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Find a BREP by following SHAPE_REPRESENTATION_RELATIONSHIP links from a SHAPE_REPRESENTATION.
    /// Many STEP files use: SR → SRR → ABSR → BREP
    ///
    /// Strategy: collect ALL SRRs that reference this SR, then try them in priority order:
    /// 1. SRRs whose other end is an ADVANCED_BREP_SHAPE_REPRESENTATION (direct link to BREP)
    /// 2. SRRs whose other end is a plain SHAPE_REPRESENTATION (indirect, recurse)
    ///
    /// This avoids the bug where assembly-placement SRRs (complex entities with transforms)
    /// are followed instead of the direct SR→ABSR link, causing all parts to map to the same BREP.
    fn find_brep_via_srr(&self, sr_id: i64, depth: usize) -> Option<i64> {
        if depth > 20 {
            return None;
        }
        let sr = self.step.find_entity(sr_id)?;
        if !sr.type_name.contains("SHAPE_REPRESENTATION") { return None; }

        // Collect all SRR relationships that reference this SR
        // Priority: direct ABSR links first, then plain SR links
        let mut direct_absr_links: Vec<i64> = Vec::new();  // other SR is an ABSR
        let mut indirect_sr_links: Vec<i64> = Vec::new();   // other SR is a plain SR

        // Helper: extract the two SR references from an SRR's params
        let extract_sr_refs = |srr: &crate::schema::StepEntity| -> (bool, Option<i64>) {
            let mut refs_our_sr = false;
            let mut other_sr_id: Option<i64> = None;
            for (i, param) in srr.params.iter().enumerate() {
                if let Some(ref_id) = self.get_ref(param) {
                    if ref_id == sr_id {
                        refs_our_sr = true;
                    } else if i >= 2 {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name.contains("SHAPE_REPRESENTATION") {
                                other_sr_id = Some(ref_id);
                            }
                        }
                    }
                }
            }
            (refs_our_sr, other_sr_id)
        };

        // Check simple SHAPE_REPRESENTATION_RELATIONSHIP entities (these are typically SR→ABSR links)
        for srr in self.step.find_entities_by_type("SHAPE_REPRESENTATION_RELATIONSHIP") {
            let (refs_our_sr, other_sr_id) = extract_sr_refs(srr);
            if !refs_our_sr { continue; }
            if let Some(other_id) = other_sr_id {
                if let Some(other_entity) = self.step.find_entity(other_id) {
                    if other_entity.type_name.contains("ADVANCED_BREP_SHAPE_REPRESENTATION")
                        || other_entity.type_name.contains("FACETED_BREP_SHAPE_REPRESENTATION") {
                        direct_absr_links.push(other_id);
                    } else {
                        indirect_sr_links.push(other_id);
                    }
                }
            }
        }

        // Also check REPRESENTATION_RELATIONSHIP entities (may catch additional complex entities)
        for srr in self.step.find_entities_by_type("REPRESENTATION_RELATIONSHIP") {
            // Skip if this is already caught as SHAPE_REPRESENTATION_RELATIONSHIP
            if srr.type_name.contains("SHAPE_REPRESENTATION_RELATIONSHIP") { continue; }
            let (refs_our_sr, other_sr_id) = extract_sr_refs(srr);
            if !refs_our_sr { continue; }
            if let Some(other_id) = other_sr_id {
                if let Some(other_entity) = self.step.find_entity(other_id) {
                    if other_entity.type_name.contains("ADVANCED_BREP_SHAPE_REPRESENTATION")
                        || other_entity.type_name.contains("FACETED_BREP_SHAPE_REPRESENTATION") {
                        direct_absr_links.push(other_id);
                    } else {
                        indirect_sr_links.push(other_id);
                    }
                }
            }
        }

        // Priority 1: try direct ABSR links
        for absr_id in &direct_absr_links {
            if let Some(brep_id) = self.find_brep_in_representation(*absr_id) {
                return Some(brep_id);
            }
        }

        // Priority 2: try indirect SR links (recurse)
        for other_id in &indirect_sr_links {
            if let Some(brep_id) = self.find_brep_via_srr(*other_id, depth + 1) {
                return Some(brep_id);
            }
        }

        None
    }

    /// Check if an ADVANCED_BREP_SHAPE_REPRESENTATION belongs to a PRODUCT_DEFINITION.
    fn absr_belongs_to_pd(&self, absr: &crate::schema::StepEntity, pd_id: i64) -> bool {
        for sdr in self.step.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION") {
            let mut refs_absr = false;
            let mut refs_pds_id: Option<i64> = None;
            for (i, param) in sdr.params.iter().enumerate() {
                if let Some(ref_id) = self.get_ref(param) {
                    if ref_id == absr.id { refs_absr = true; }
                    if i == 0 { refs_pds_id = Some(ref_id); }
                }
            }
            if !refs_absr { continue; }
            if let Some(pds_id) = refs_pds_id {
                if let Some(pds) = self.step.find_entity(pds_id) {
                    for param in &pds.params {
                        if let Some(ref_id) = self.get_ref(param) {
                            if ref_id == pd_id { return true; }
                        }
                    }
                }
            }
        }
        false
    }

    /// Find the shell ref given a BREP entity ID.
    fn find_shell_ref_by_brep_id(&self, brep_id: i64) -> Option<i64> {
        let brep = self.step.find_entity(brep_id)?;
        self.find_shell_ref(&brep)
    }

    // ─── Color extraction ───────────────────────────────────────────────────

    /// Build a map from BREP entity ID → RGBA color from STYLED_ITEM chain.
    fn extract_color_map(&self) -> HashMap<i64, [f32; 4]> {
        let mut color_map: HashMap<i64, [f32; 4]> = HashMap::new();

        let styled_items = self.step.find_entities_by_type("STYLED_ITEM");
        for styled in &styled_items {
            let mut item_id: Option<i64> = None;
            let mut style_ids: Vec<i64> = Vec::new();

            for (i, param) in styled.params.iter().enumerate() {
                if i == 0 { continue; }
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(entity) = self.step.find_entity(ref_id) {
                        if entity.type_name == "MANIFOLD_SOLID_BREP" {
                            item_id = Some(ref_id);
                        } else if entity.type_name == "ADVANCED_BREP_SHAPE_REPRESENTATION" {
                            for p in &entity.params {
                                if let Some(brep_id) = self.get_ref(p) {
                                    if let Some(brep) = self.step.find_entity(brep_id) {
                                        if brep.type_name == "MANIFOLD_SOLID_BREP" {
                                            item_id = Some(brep_id);
                                        }
                                    }
                                }
                            }
                        } else if entity.type_name == "PRESENTATION_STYLE_ASSIGNMENT" {
                            style_ids.push(ref_id);
                        }
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name == "PRESENTATION_STYLE_ASSIGNMENT" {
                                    style_ids.push(ref_id);
                                }
                            }
                        }
                    }
                }
            }

            if item_id.is_none() {
                for param in styled.params.iter().rev() {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name == "MANIFOLD_SOLID_BREP" {
                                item_id = Some(ref_id);
                                break;
                            }
                        }
                    }
                }
            }

            let color = self.resolve_color_from_styles(&style_ids);
            if let (Some(brep_id), Some(col)) = (item_id, color) {
                color_map.insert(brep_id, col);
            }
        }

        if !color_map.is_empty() {
            info!("Extracted {} colors from STYLED_ITEMs", color_map.len());
        }
        color_map
    }

    /// Resolve color from PRESENTATION_STYLE_ASSIGNMENT chain.
    fn resolve_color_from_styles(&self, style_ids: &[i64]) -> Option<[f32; 4]> {
        let mut visited = std::collections::HashSet::new();
        for style_id in style_ids {
            if let Some(psa) = self.step.find_entity(*style_id) {
                if psa.type_name != "PRESENTATION_STYLE_ASSIGNMENT" { continue; }
                for param in &psa.params {
                    if let StepValue::List(items) = param {
                        for item in items {
                            if let Some(ref_id) = self.get_ref(item) {
                                if let Some(color) = self.walk_style_chain(ref_id, &mut visited) {
                                    return Some(color);
                                }
                            }
                        }
                    }
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(color) = self.walk_style_chain(ref_id, &mut visited) {
                            return Some(color);
                        }
                    }
                }
            }
        }
        None
    }

    /// Walk the style chain from SURFACE_STYLE_USAGE down to COLOUR_RGB.
    fn walk_style_chain(&self, entity_id: i64, visited: &mut std::collections::HashSet<i64>) -> Option<[f32; 4]> {
        if visited.contains(&entity_id) {
            return None;
        }
        visited.insert(entity_id);
        let result = self.walk_style_chain_inner(entity_id, visited);
        visited.remove(&entity_id);
        result
    }

    /// Inner implementation of walk_style_chain (called after visited guard).
    fn walk_style_chain_inner(&self, entity_id: i64, visited: &mut std::collections::HashSet<i64>) -> Option<[f32; 4]> {
        let entity = self.step.find_entity(entity_id)?;

        match entity.type_name.as_str() {
            "SURFACE_STYLE_USAGE" | "SURFACE_SIDE_STYLE" | "SURFACE_STYLE_FILL_AREA" | "FILL_AREA_STYLE" | "FILL_AREA_STYLE_COLOUR" => {
                for param in &entity.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(color) = self.walk_style_chain(ref_id, visited) {
                            return Some(color);
                        }
                    }
                    if let StepValue::List(items) = param {
                        for item in items {
                            if let Some(ref_id) = self.get_ref(item) {
                                if let Some(color) = self.walk_style_chain(ref_id, visited) {
                                    return Some(color);
                                }
                            }
                        }
                    }
                }
                None
            }
            "COLOUR_RGB" => {
                let mut rgb = [0.5f32, 0.5, 0.5];
                let mut idx = 0;
                for param in &entity.params {
                    if let Some(f) = self.get_float(param) {
                        if idx < 3 {
                            rgb[idx] = f as f32;
                            idx += 1;
                        }
                    }
                }
                Some([rgb[0], rgb[1], rgb[2], 1.0])
            }
            "DRAUGHTING_PRE_DEFINED_COLOUR" => {
                // Named colors like 'red', 'green', 'blue', etc.
                for param in &entity.params {
                    if let StepValue::String(name) = param {
                        return Some(resolve_predefined_colour(&name));
                    }
                    if let StepValue::Enum(name) = param {
                        return Some(resolve_predefined_colour(&name));
                    }
                }
                None
            }
            _ => {
                // For unknown types, try to walk deeper
                for param in &entity.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(color) = self.walk_style_chain(ref_id, visited) {
                            return Some(color);
                        }
                    }
                    if let StepValue::List(items) = param {
                        for item in items {
                            if let Some(ref_id) = self.get_ref(item) {
                                if let Some(color) = self.walk_style_chain(ref_id, visited) {
                                    return Some(color);
                                }
                            }
                        }
                    }
                }
                None
            }
        }
    }

    fn find_shell_ref(&self, brep: &crate::schema::StepEntity) -> Option<i64> {
        // MANIFOLD_SOLID_BREP('name', #shell_ref) — shell ref is usually 2nd param
        // But some files have it as the first Ref parameter
        for param in &brep.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name.contains("SHELL") {
                        return Some(ref_id);
                    }
                }
            }
        }
        // If not found by type name, try the second parameter
        if let Some(param) = brep.params.get(1) {
            if let Some(ref_id) = self.get_ref(param) {
                return Some(ref_id);
            }
        }
        None
    }

    /// Find ALL shell references in a BREP entity.
    ///
    /// For `MANIFOLD_SOLID_BREP('name', #outer_shell)`:
    ///   returns `(Some(#outer_shell), [])`
    ///
    /// For `BREP_WITH_VOIDS('name', #outer_shell, (#void_shell1, #void_shell2, ...))`:
    ///   returns `(Some(#outer_shell), [#void_shell1, #void_shell2, ...])`
    ///
    /// The void shells have INVERTED orientation — their face normals point INTO
    /// the solid material (which is OUT of the void). This is the STEP convention
    /// for representing boolean subtraction: the void shells are subtracted from
    /// the outer shell. By including all shell faces in the face_data_list,
    /// the resulting mesh will be a watertight solid with internal cavities.
    ///
    /// Algorithm adapted from truck-step v0.4.0 (ricosjp/truck, Apache-2.0 OR MIT).
    fn find_all_shell_refs(&self, brep: &crate::schema::StepEntity) -> (Option<i64>, Vec<i64>) {
        let mut outer: Option<i64> = None;
        let mut voids: Vec<i64> = Vec::new();

        // Determine if this is a BREP_WITH_VOIDS by checking the type_name.
        let is_brep_with_voids = brep.type_name == "BREP_WITH_VOIDS";

        // Strategy:
        // - The first SHELL reference is the outer shell.
        // - For BREP_WITH_VOIDS, the third parameter is a LIST of void shell refs.
        // - For MANIFOLD_SOLID_BREP, there is only the outer shell.
        for (i, param) in brep.params.iter().enumerate() {
            // Skip name (index 0)
            if i == 0 { continue; }

            // Check if this parameter is a LIST of shell references
            // (this is the BREP_WITH_VOIDS voids list)
            if let StepValue::List(items) = param {
                if is_brep_with_voids && outer.is_some() {
                    // This is the voids list — collect all shell refs
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name.contains("SHELL") {
                                    voids.push(ref_id);
                                }
                            }
                        }
                    }
                }
                continue;
            }

            // Single shell reference
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name.contains("SHELL") {
                        if outer.is_none() {
                            outer = Some(ref_id);
                        } else if is_brep_with_voids {
                            // Additional shell refs in BREP_WITH_VOIDS (rare, but handle it)
                            voids.push(ref_id);
                        }
                    }
                }
            }
        }

        // Fallback: if outer not found, use the second parameter
        if outer.is_none() {
            if let Some(param) = brep.params.get(1) {
                if let Some(ref_id) = self.get_ref(param) {
                    outer = Some(ref_id);
                }
            }
        }

        if !voids.is_empty() {
            log::info!(
                "BREP #{} (BREP_WITH_VOIDS): outer shell #{}, {} void shell(s) {:?}",
                brep.id,
                outer.unwrap_or(0),
                voids.len(),
                voids
            );
        }

        (outer, voids)
    }

    /// Find all shell references by BREP ID — convenience wrapper.
    fn find_all_shell_refs_by_brep_id(&self, brep_id: i64) -> (Option<i64>, Vec<i64>) {
        match self.step.find_entity(brep_id) {
            Some(brep) => self.find_all_shell_refs(&brep),
            None => (None, vec![]),
        }
    }

    /// Collect pending BREP instances without triangulating them.
    ///
    /// This mirrors `convert_detailed_instances()` but skips the expensive
    /// `triangulate_brep_detailed_cached()` call. Instead, it returns
    /// `PendingBrepInstance` descriptors that can be triangulated later
    /// via `triangulate_pending_instance()`.
    fn collect_pending_instances(&self) -> Vec<PendingBrepInstance> {
        let color_map = self.extract_color_map();
        let mut results: Vec<PendingBrepInstance> = Vec::new();

        // ─── Phase 1: Assembly-based collection via NAUO tree walk ────────
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if !nauos.is_empty() {
            let mut parent_pd_to_children: HashMap<i64, Vec<(i64, i64, String)>> = HashMap::new();
            for nauo in &nauos {
                let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
                if let (Some(parent_pd), Some(child_pd)) = (relating_pd, related_pd) {
                    let name = self.extract_nauo_name(nauo);
                    parent_pd_to_children.entry(parent_pd).or_default().push((nauo.id, child_pd, name));
                }
            }

            let parent_pds: std::collections::HashSet<i64> = parent_pd_to_children.keys().copied().collect();
            let child_pds: std::collections::HashSet<i64> = nauos.iter()
                .filter_map(|n| self.extract_nauo_pd_refs(n).1)
                .collect();
            let roots: Vec<i64> = parent_pds.difference(&child_pds).copied().collect();

            if !roots.is_empty() {
                // Track which BREPs we've already added (to avoid duplicates)
                let mut seen_breps: std::collections::HashSet<i64> = std::collections::HashSet::new();

                for root_pd in &roots {
                    self.collect_pending_from_assembly_tree(
                        *root_pd,
                        &None,
                        &color_map,
                        &parent_pd_to_children,
                        &mut results,
                        &mut seen_breps,
                    );
                }
            }

            if !results.is_empty() {
                return results;
            }
        }

        // ─── Phase 2: No assembly — direct BREP descriptors ───
        let breps = self.step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        for brep in &breps {
            let name = self.get_brep_name(brep.id);
            let color = color_map.get(&brep.id).copied();
            results.push(PendingBrepInstance {
                name,
                brep_id: brep.id,
                transform: None,
                color,
                face_count_estimate: None,
            });
        }

        if !results.is_empty() {
            return results;
        }

        // FACETED_BREP
        let faceted = self.step.find_entities_by_type("FACETED_BREP");
        for fb in &faceted {
            let name = self.get_brep_name(fb.id);
            let color = color_map.get(&fb.id).copied();
            results.push(PendingBrepInstance {
                name,
                brep_id: fb.id,
                transform: None,
                color,
                face_count_estimate: None,
            });
        }

        results
    }

    /// Walk the assembly tree collecting PendingBrepInstance descriptors
    /// without triangulating any geometry.
    ///
    /// IMPORTANT: Instances are produced in DFS order (first child first) to match
    /// the order of tree leaves in `build_assembly_node_iterative`.
    fn collect_pending_from_assembly_tree(
        &self,
        root_pd_id: i64,
        root_transform: &Option<[[f64; 4]; 4]>,
        color_map: &HashMap<i64, [f32; 4]>,
        parent_pd_to_children: &HashMap<i64, Vec<(i64, i64, String)>>,
        results: &mut Vec<PendingBrepInstance>,
        _seen_breps: &mut std::collections::HashSet<i64>,
    ) {
        // Work stack: ALL nodes (both sub-assemblies and leaves) are pushed.
        // Children pushed in REVERSE order so first child is on top (LIFO → DFS).
        struct WorkItem {
            pd_id: i64,
            composed: Option<[[f64; 4]; 4]>,
            depth: usize,
            ancestors: std::collections::HashSet<i64>,
            is_leaf: bool,
            nauo_name: String,
            brep_id: Option<i64>,
        }

        let mut stack: Vec<WorkItem> = Vec::new();

        if let Some(children) = parent_pd_to_children.get(&root_pd_id) {
            let root_ancestors = std::collections::HashSet::new();
            for (nauo_id, child_pd_id, nauo_name) in children.iter().rev() {
                let nauo_transform = self.find_nauo_transform(*nauo_id, *child_pd_id);
                let composed = match (&root_transform, &nauo_transform) {
                    (Some(pt), Some(nt)) => Some(mat4_mul(pt, nt)),
                    (Some(pt), None) => Some(*pt),
                    (None, Some(nt)) => Some(nt.clone()),
                    (None, None) => None,
                };
                let has_nauo_children = parent_pd_to_children.contains_key(child_pd_id);
                let brep_id = if has_nauo_children { None } else { self.find_pd_brep(*child_pd_id) };

                stack.push(WorkItem {
                    pd_id: *child_pd_id,
                    composed,
                    depth: 1,
                    ancestors: root_ancestors.clone(),
                    is_leaf: !has_nauo_children,
                    nauo_name: nauo_name.clone(),
                    brep_id,
                });
            }
        }

        const MAX_DEPTH: usize = 50;

        while let Some(item) = stack.pop() {
            if item.depth > MAX_DEPTH {
                log::warn!("Max depth {} reached at PD #{}, skipping", MAX_DEPTH, item.pd_id);
                continue;
            }
            if item.ancestors.contains(&item.pd_id) {
                log::warn!("Cycle detected in NAUO tree at PD #{}, skipping", item.pd_id);
                continue;
            }

            if item.is_leaf {
                if let Some(brep_id) = item.brep_id {
                    let color = color_map.get(&brep_id).copied();
                    let name = format!("{} (BREP#{})", item.nauo_name, brep_id);
                    results.push(PendingBrepInstance {
                        name,
                        brep_id,
                        transform: item.composed,
                        color,
                        face_count_estimate: None,
                    });
                }
            } else {
                if let Some(children) = parent_pd_to_children.get(&item.pd_id) {
                    for (nauo_id, child_pd_id, nauo_name) in children.iter().rev() {
                        let nauo_transform = self.find_nauo_transform(*nauo_id, *child_pd_id);
                        let composed = match (&item.composed, &nauo_transform) {
                            (Some(pt), Some(nt)) => Some(mat4_mul(pt, nt)),
                            (Some(pt), None) => Some(*pt),
                            (None, Some(nt)) => Some(nt.clone()),
                            (None, None) => None,
                        };
                        let has_nauo_children = parent_pd_to_children.contains_key(child_pd_id);
                        let brep_id = if has_nauo_children { None } else { self.find_pd_brep(*child_pd_id) };

                        let mut new_ancestors = item.ancestors.clone();
                        new_ancestors.insert(item.pd_id);

                        stack.push(WorkItem {
                            pd_id: *child_pd_id,
                            composed,
                            depth: item.depth + 1,
                            ancestors: new_ancestors,
                            is_leaf: !has_nauo_children,
                            nauo_name: nauo_name.clone(),
                            brep_id,
                        });
                    }
                }
            }
        }
    }

    /// Compute a bounding box from all CARTESIAN_POINT entities.
    /// Result is cached so repeated calls don't recompute.
    fn compute_bounding_box(&self) -> Option<(Point3d, Point3d)> {
        // Check cache first
        if let Some(ref cached) = *self.bbox_cache.borrow() {
            return cached.clone();
        }
        let result = self.compute_bounding_box_uncached();
        *self.bbox_cache.borrow_mut() = Some(result.clone());
        result
    }

    /// Actual bounding box computation (uncached).
    fn compute_bounding_box_uncached(&self) -> Option<(Point3d, Point3d)> {
        // Stream min/max without allocating a Vec — for large STEP files with
        // 100K+ CARTESIAN_POINT entities, the Vec allocation was a significant
        // blocking cost on the WASM main thread.
        //
        // CRITICAL FIX: We EXCLUDE cartesian points that are referenced by
        // AXIS2_PLACEMENT_3D entities used for PRODUCT_DEFINITION placement
        // (i.e., world placements like (47.5, 87.99, 33) for the bolt).
        // These world placements inflate the bbox by 3x and make tolerance
        // calculations (snap_tol, merge_tol) way too aggressive, corrupting
        // the mesh with false-positive vertex snapping.
        //
        // We only include points that are part of the actual BREP geometry:
        //   - CARTESIAN_POINT used as edge curve control points
        //   - CARTESIAN_POINT used as surface control points
        //   - CARTESIAN_POINT used as VERTEX_POINT entities
        //
        // The simplest heuristic: skip points referenced by AXIS2_PLACEMENT_3D
        // entities that appear in ITEM_DEFINED_TRANSFORMATION (these are the
        // world placements).

        // Step 1: Collect cartesian point IDs referenced by ITEM_DEFINED_TRANSFORMATION
        let mut world_placement_point_ids: std::collections::HashSet<i64> = std::collections::HashSet::new();
        let item_transforms = self.step.find_entities_by_type("ITEM_DEFINED_TRANSFORMATION");
        for t in item_transforms.iter() {
            // ITEM_DEFINED_TRANSFORMATION('', '', #axis1, #axis2)
            // The 4th param (index 3) is the "transform" axis2 placement —
            // this is the WORLD placement we want to exclude.
            if t.params.len() >= 4 {
                if let Some(axis2_id) = self.get_ref(&t.params[3]) {
                    // Get the AXIS2_PLACEMENT_3D entity, then its first param (cartesian point ref)
                    if let Some(axis2_entity) = self.step.find_entity(axis2_id) {
                        if let Some(cp_id) = self.get_ref(&axis2_entity.params[0]) {
                            world_placement_point_ids.insert(cp_id);
                        }
                    }
                }
            }
        }

        let point_entities = self.step.find_entities_by_type("CARTESIAN_POINT");
        let mut first = true;
        let mut min = Point3d::ORIGIN;
        let mut max = Point3d::ORIGIN;
        let mut skipped_world_points = 0usize;
        let mut included_points = 0usize;

        for e in point_entities.iter() {
            // Skip world placement points
            if world_placement_point_ids.contains(&e.id) {
                skipped_world_points += 1;
                continue;
            }
            if let Some(p) = self.resolve_cartesian_point(e.id) {
                included_points += 1;
                if first {
                    min = p;
                    max = p;
                    first = false;
                } else {
                    min.x = min.x.min(p.x);
                    min.y = min.y.min(p.y);
                    min.z = min.z.min(p.z);
                    max.x = max.x.max(p.x);
                    max.y = max.y.max(p.y);
                    max.z = max.z.max(p.z);
                }
            }
        }

        if first {
            return None;
        }

        if skipped_world_points > 0 {
            log::info!(
                "compute_bounding_box: excluded {} world-placement cartesian points, included {} geometry points, bbox=[{:.2},{:.2},{:.2}]×[{:.2},{:.2},{:.2}]",
                skipped_world_points, included_points,
                min.x, min.y, min.z, max.x, max.y, max.z,
            );
        }

        // Expand the box slightly
        let margin = 0.001;
        min.x -= margin; min.y -= margin; min.z -= margin;
        max.x += margin; max.y += margin; max.z += margin;

        Some((min, max))
    }

    /// Extract FaceData (surface + boundary edges) from a CLOSED_SHELL, OPEN_SHELL,
    /// or ORIENTED_CLOSED_SHELL entity.
    ///
    /// For `ORIENTED_CLOSED_SHELL('', #basis_shell, .F.)`, the shell-level orientation
    /// is applied by flipping the `forward` flag on all extracted faces. This is critical
    /// for `BREP_WITH_VOIDS` where void shells may use `ORIENTED_CLOSED_SHELL` with
    /// `.F.` orientation to indicate that face normals point INTO the solid material.
    fn extract_shell_faces(&self, shell_id: i64, is_void: bool) -> Option<Vec<FaceData>> {
        let shell = self.step.find_entity(shell_id)?;

        // Check if this is an ORIENTED_CLOSED_SHELL wrapping a basis shell.
        // ORIENTED_CLOSED_SHELL('', #basis_shell_ref, .T./.F.)
        // When orientation is .F., face normals must be flipped.
        let shell_orientation_forward = if shell.type_name == "ORIENTED_CLOSED_SHELL" {
            // Last parameter is the orientation (.T. or .F.)
            let orient = shell.params.last().and_then(|p| {
                if let StepValue::Enum(e) = p { Some(e.as_str()) } else { None }
            }).unwrap_or("T");
            orient == "T"
        } else {
            true // Default forward orientation for CLOSED_SHELL, OPEN_SHELL
        };

        // If ORIENTED_CLOSED_SHELL, we need to extract faces from the basis shell reference
        let effective_shell_id = if shell.type_name == "ORIENTED_CLOSED_SHELL" {
            // Find the basis shell reference (typically 2nd param, index 1)
            let mut basis_id = None;
            for (i, param) in shell.params.iter().enumerate() {
                if i == 0 { continue; } // Skip name
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(entity) = self.step.find_entity(ref_id) {
                        if entity.type_name.contains("SHELL") {
                            basis_id = Some(ref_id);
                            break;
                        }
                    }
                }
            }
            match basis_id {
                Some(id) => {
                    log::info!(
                        "ORIENTED_CLOSED_SHELL #{}: orientation={}, basis shell #{}",
                        shell_id, shell_orientation_forward, id
                    );
                    id
                }
                None => {
                    log::warn!(
                        "ORIENTED_CLOSED_SHELL #{}: no basis shell reference found — using shell directly",
                        shell_id
                    );
                    shell_id
                }
            }
        } else {
            shell_id
        };

        let actual_shell = if effective_shell_id != shell_id {
            self.step.find_entity(effective_shell_id)?
        } else {
            shell
        };

        let mut face_data_list = Vec::new();
        let mut total_face_refs = 0usize;
        let mut failed_count = 0usize;

        // CLOSED_SHELL('', (#face1, #face2, ...))
        for param in &actual_shell.params {
            match param {
                StepValue::List(items) => {
                    for item in items {
                        if let Some(face_id) = self.get_ref(item) {
                            total_face_refs += 1;
                            if let Some(mut face_data) = self.extract_face_data(face_id) {
                                // Apply shell-level orientation flip if needed
                                if !shell_orientation_forward {
                                    face_data.forward = !face_data.forward;
                                }
                                face_data.is_void = is_void;
                                face_data_list.push(face_data);
                            } else {
                                failed_count += 1;
                                log::warn!(
                                    "SHELL_FACE_FAIL: shell #{} face ref #{} — extract_face_data returned None",
                                    shell_id, face_id,
                                );
                            }
                        }
                    }
                }
                StepValue::Ref(face_id) => {
                    total_face_refs += 1;
                    if let Some(mut face_data) = self.extract_face_data(*face_id) {
                        // Apply shell-level orientation flip if needed
                        if !shell_orientation_forward {
                            face_data.forward = !face_data.forward;
                        }
                        face_data.is_void = is_void;
                        face_data_list.push(face_data);
                    } else {
                        failed_count += 1;
                        log::warn!(
                            "SHELL_FACE_FAIL: shell #{} face ref #{} — extract_face_data returned None",
                            shell_id, face_id,
                        );
                    }
                }
                _ => {}
            }
        }

        if failed_count > 0 {
            log::warn!(
                "SHELL_FACE_SUMMARY: shell #{} — {} face refs, {} extracted, {} failed",
                shell_id, total_face_refs, face_data_list.len(), failed_count,
            );
        }

        if face_data_list.is_empty() { None } else { Some(face_data_list) }
    }

    /// Extract both surface geometry and boundary edges from an ADVANCED_FACE or FACE_SURFACE entity.
    fn extract_face_data(&self, face_id: i64) -> Option<FaceData> {
        let face_entity = self.step.find_entity(face_id)?;

        match face_entity.type_name.as_str() {
            "ADVANCED_FACE" | "FACE_SURFACE" => {
                // Format: #N = ADVANCED_FACE('', (bounds), #surface_ref, .T.);
                // params: [name, bounds_list, surface_ref, orientation]

                // Extract surface
                let surface = self.extract_face_surface_from_entity(face_entity)?;
                
                // Extract boundary edges with inner/outer distinction AND STEP IDs
                let (outer_edges, outer_step_ids, inner_edges, inner_step_ids) =
                    self.extract_face_bounds_separated_with_step_ids(face_entity);

                // All edges combined for backward compat
                let mut all_edges = outer_edges.clone();
                let mut all_step_ids = outer_step_ids.clone();
                for (i, inner) in inner_edges.iter().enumerate() {
                    all_edges.extend(inner.clone());
                    if let Some(ids) = inner_step_ids.get(i) {
                        all_step_ids.extend(ids.clone());
                    }
                }

                // Extract face orientation (last param, typically .T. or .F.)
                let forward = self.extract_face_orientation(face_entity);

                // Extract the STEP surface entity ID for PCURVE matching
                let surface_step_id = self.extract_face_surface_step_id(face_entity);

                // Extract analytical PCURVEs (Curve2d) for each edge
                let edge_curves_2d = self.extract_edge_curves_2d(
                    face_entity, &all_edges, surface_step_id,
                );

                Some(FaceData {
                    surface,
                    outer_edges,
                    inner_edges,
                    edges: all_edges,
                    forward,
                    step_face_id: face_id,
                    surface_step_id,
                    edge_curves_2d,
                    edge_step_ids: all_step_ids,
                    outer_edge_step_ids: outer_step_ids,
                    inner_edge_step_ids: inner_step_ids,
                    is_void: false, // Will be set by extract_shell_faces caller
                })
            }
            _ => {
                // Try to extract directly as a surface (no boundary info)
                if let Some(surface) = self.extract_surface(face_id, 0) {
                    Some(FaceData {
                        surface,
                        outer_edges: vec![],
                        inner_edges: vec![],
                        edges: vec![],
                        forward: true,
                        step_face_id: face_id,
                        surface_step_id: None,
                        edge_curves_2d: vec![],
                        edge_step_ids: vec![],
                        outer_edge_step_ids: vec![],
                        inner_edge_step_ids: vec![],
                        is_void: false, // Will be set by extract_shell_faces caller
                    })
                } else {
                    None
                }
            }
        }
    }

    /// Extract the surface geometry from an ADVANCED_FACE or FACE_SURFACE entity.
    /// This is the surface-only extraction logic (previously extract_face_surface).
    fn extract_face_surface_from_entity(&self, face: &crate::schema::StepEntity) -> Option<Surface> {
        // Format: #N = ADVANCED_FACE('', (bounds), #surface_ref, .T.);
        // The surface reference is typically the 3rd parameter (index 2).
        // But bounds can be complex (lists of lists), so we need to be smart.

        // Try parameter index 2 first (the typical position for surface ref)
        if let Some(param) = face.params.get(2) {
            if let Some(surface_id) = self.get_ref(param) {
                if let Some(surface) = self.extract_surface(surface_id, 0) {
                    return Some(surface);
                } else {
                    // Log when surface extraction fails for an ADVANCED_FACE
                    if let Some(surface_entity) = self.step.find_entity(surface_id) {
                        log::warn!(
                            "FACE_SURFACE_FAIL: ADVANCED_FACE #{} → surface ref #{} type='{}' — extract_surface returned None",
                            face.id, surface_id, surface_entity.type_name,
                        );
                    }
                }
            }
        }

        // If index 2 didn't work, scan all params for the surface ref
        // Skip the first param (usually a string name)
        for (i, param) in face.params.iter().enumerate() {
            if i == 0 { continue; } // Skip name
            if let Some(surface_id) = self.get_ref(param) {
                // Check if this ref points to a surface entity (not a bound)
                if let Some(entity) = self.step.find_entity(surface_id) {
                    let tn = entity.type_name.as_str();
                    let is_surface = matches!(
                        tn,
                        "PLANE" | "CYLINDRICAL_SURFACE" | "SPHERICAL_SURFACE" |
                        "CONICAL_SURFACE" | "TOROIDAL_SURFACE" |
                        "SURFACE_OF_REVOLUTION" | "SURFACE_OF_LINEAR_EXTRUSION" |
                        "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" |
                        "BEZIER_SURFACE" | "RECTANGULAR_TRIMMED_SURFACE" |
                        "OFFSET_SURFACE" | "SWEPT_SURFACE"
                    ) || tn.contains("B_SPLINE_SURFACE") // Handle complex entities like "BOUNDED_SURFACE+B_SPLINE_SURFACE+..."
                    || tn.contains("SURFACE") && !tn.contains("CURVE"); // Handle other complex surface entities
                    if is_surface {
                        if let Some(surface) = self.extract_surface(surface_id, 0) {
                            return Some(surface);
                        }
                    }
                }
            }
        }

        None
    }

    /// Extract the face orientation from an ADVANCED_FACE entity.
    /// The orientation is the last parameter, typically .T. or .F.
    fn extract_face_orientation(&self, face: &crate::schema::StepEntity) -> bool {
        if let Some(last_param) = face.params.last() {
            match last_param {
                StepValue::Enum(e) => return e == "T",
                StepValue::Float(f) => return *f != 0.0,
                StepValue::Integer(i) => return *i != 0,
                _ => {}
            }
        }
        // Default to true if not found
        true
    }

    /// Extract boundary edges from an ADVANCED_FACE entity.
    /// Traverses: ADVANCED_FACE → bounds_list → FACE_BOUND/FACE_OUTER_BOUND →
    ///            EDGE_LOOP → ORIENTED_EDGE → EDGE_CURVE → curve + vertices
    fn extract_face_bounds(&self, face: &crate::schema::StepEntity) -> Vec<TopoEdge> {
        let (outer, inner) = self.extract_face_bounds_separated(face);
        let mut all_edges = outer;
        for loop_edges in inner {
            all_edges.extend(loop_edges);
        }
        all_edges
    }

    /// Extract boundary edges from an ADVANCED_FACE entity, separating outer and inner loops.
    /// FACE_OUTER_BOUND → outer loop (the main boundary)
    /// FACE_BOUND → inner loop (a hole)
    /// Returns (outer_edges, inner_loops) where inner_loops is a Vec of edge loops.
    fn extract_face_bounds_separated(&self, face: &crate::schema::StepEntity) -> (Vec<TopoEdge>, Vec<Vec<TopoEdge>>) {
        let (outer_edges, _, inner_loops, _) = self.extract_face_bounds_separated_with_step_ids(face);
        (outer_edges, inner_loops)
    }

    /// Extract boundary edges with STEP EDGE_CURVE IDs for cache-key tracking.
    /// Returns (outer_edges, outer_step_ids, inner_loops, inner_step_ids).
    fn extract_face_bounds_separated_with_step_ids(
        &self,
        face: &crate::schema::StepEntity,
    ) -> (Vec<TopoEdge>, Vec<i64>, Vec<Vec<TopoEdge>>, Vec<Vec<i64>>) {
        let mut outer_edges: Vec<TopoEdge> = Vec::new();
        let mut outer_step_ids: Vec<i64> = Vec::new();
        let mut inner_loops: Vec<Vec<TopoEdge>> = Vec::new();
        let mut inner_step_ids: Vec<Vec<i64>> = Vec::new();

        // ADVANCED_FACE params: [name, (bounds_list), surface_ref, orientation]
        // The bounds are in params[1], which is a List of references to FACE_BOUND/FACE_OUTER_BOUND
        for param in &face.params {
            // Look for the bounds list — it's a StepValue::List containing references
            if let StepValue::List(items) = param {
                let mut found_bound = false;
                for item in items {
                    if let Some(bound_id) = self.get_ref(item) {
                        if let Some(bound_entity) = self.step.find_entity(bound_id) {
                            if bound_entity.type_name == "FACE_OUTER_BOUND" {
                                found_bound = true;
                                if let Some((loop_edges, loop_step_ids)) = self.resolve_face_bound_with_step_ids(bound_entity) {
                                    outer_edges = loop_edges;
                                    outer_step_ids = loop_step_ids;
                                }
                            } else if bound_entity.type_name == "FACE_BOUND" {
                                found_bound = true;
                                if let Some((loop_edges, loop_step_ids)) = self.resolve_face_bound_with_step_ids(bound_entity) {
                                    inner_loops.push(loop_edges);
                                    inner_step_ids.push(loop_step_ids);
                                }
                            }
                        }
                    }
                }
                // If we found bounds in this list, don't process it again
                if found_bound {
                    // CRITICAL: If no FACE_OUTER_BOUND was found but FACE_BOUND entries exist,
                    // the first FACE_BOUND is the outer boundary (not a hole).
                    // Many STEP files use only FACE_BOUND (no FACE_OUTER_BOUND at all).
                    if outer_edges.is_empty() && !inner_loops.is_empty() {
                        outer_edges = inner_loops.remove(0);
                        outer_step_ids = inner_step_ids.remove(0);
                    }
                    return (outer_edges, outer_step_ids, inner_loops, inner_step_ids);
                }
            }
        }

        // Fallback: if no FACE_OUTER_BOUND found but FACE_BOUND exists,
        // treat the first FACE_BOUND as the outer boundary
        // (some STEP files use FACE_BOUND for both outer and inner)
        if outer_edges.is_empty() && !inner_loops.is_empty() {
            outer_edges = inner_loops.remove(0);
            outer_step_ids = inner_step_ids.remove(0);
        }

        (outer_edges, outer_step_ids, inner_loops, inner_step_ids)
    }

    /// Resolve a FACE_BOUND or FACE_OUTER_BOUND entity to a list of Edge objects with STEP IDs.
    /// FACE_BOUND params: [name, loop_ref, orientation]
    fn resolve_face_bound(&self, bound_entity: &crate::schema::StepEntity) -> Option<Vec<TopoEdge>> {
        let (edges, _step_ids) = self.resolve_face_bound_with_step_ids(bound_entity)?;
        Some(edges)
    }

    /// Resolve a FACE_BOUND or FACE_OUTER_BOUND entity, returning both edges and their STEP EDGE_CURVE IDs.
    /// FACE_BOUND params: [name, loop_ref, orientation]
    /// When orientation is .F., the entire loop should be reversed (winding order flipped).
    ///
    /// P7: Now handles VERTEX_LOOP — a degenerate loop consisting of a single vertex
    /// (used for the apex of a cone or pyramid). Returns an empty edge list, which
    /// the triangulator will treat as a degenerate face (zero area, no triangles).
    fn resolve_face_bound_with_step_ids(&self, bound_entity: &crate::schema::StepEntity) -> Option<(Vec<TopoEdge>, Vec<i64>)> {
        // FACE_BOUND('', #loop_ref, .T.)
        // The loop reference is typically the 2nd parameter (index 1)
        // The orientation is typically the 3rd parameter (index 2)
        let mut orientation = true;
        for (i, param) in bound_entity.params.iter().enumerate() {
            if i == 0 { continue; } // Skip name
            // Check for orientation enum (.T. or .F.)
            if let StepValue::Enum(e) = param {
                orientation = e == "T";
                continue;
            }
            if let Some(loop_id) = self.get_ref(param) {
                if let Some(loop_entity) = self.step.find_entity(loop_id) {
                    if loop_entity.type_name == "EDGE_LOOP" {
                        let (mut edges, step_ids) = self.resolve_edge_loop_with_step_ids(loop_id);
                        if !orientation {
                            // FACE_BOUND orientation=.F. means the entire loop is reversed.
                            // Reverse the edge order AND flip each individual edge.
                            edges.reverse();
                            for edge in &mut edges {
                                *edge = edge.reversed();
                            }
                        }
                        return Some((edges, step_ids));
                    }
                    if loop_entity.type_name == "VERTEX_LOOP" {
                        // P7: VERTEX_LOOP is a degenerate loop consisting of a single vertex.
                        // It is used for the apex of a cone or pyramid, where the face
                        // degenerates to a single point. We return an empty edge list,
                        // which causes the triangulator to produce zero triangles for
                        // this face (correct behavior — the apex contributes no area).
                        //
                        // Algorithm adapted from truck-step v0.4.0 (ricosjp/truck, Apache-2.0 OR MIT).
                        log::debug!(
                            "VERTEX_LOOP #{} resolved as degenerate (empty edge list) for FACE_BOUND #{}",
                            loop_id, bound_entity.id
                        );
                        return Some((Vec::new(), Vec::new()));
                    }
                }
            }
        }
        None
    }

    /// Resolve an EDGE_LOOP entity to a list of Edge objects.
    /// EDGE_LOOP params: [name, (oriented_edge_refs)]
    fn resolve_edge_loop(&self, loop_id: i64) -> Vec<TopoEdge> {
        let (edges, _) = self.resolve_edge_loop_with_step_ids(loop_id);
        edges
    }

    /// Resolve an EDGE_LOOP entity, returning both edges and their STEP EDGE_CURVE IDs.
    fn resolve_edge_loop_with_step_ids(&self, loop_id: i64) -> (Vec<TopoEdge>, Vec<i64>) {
        let loop_entity = match self.step.find_entity(loop_id) {
            Some(e) => e,
            None => return (vec![], vec![]),
        };

        let mut edges = Vec::new();
        let mut step_ids = Vec::new();

        // EDGE_LOOP('', (#oe1, #oe2, ...))
        for param in &loop_entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(oe_id) = self.get_ref(item) {
                        if let Some((edge, step_id)) = self.resolve_oriented_edge_with_step_id(oe_id) {
                            edges.push(edge);
                            step_ids.push(step_id);
                        }
                    }
                }
            }
        }

        // Reorder edges to form a connected loop (end of one = start of next).
        // Some STEP files (e.g., SampleCube.step) list edges in arbitrary order
        // rather than topologically connected order. This causes the boundary
        // point list to be scrambled, breaking dedup and triangulation.
        let (edges, step_ids) = reorder_edge_loop(edges, step_ids);

        (edges, step_ids)
    }

    /// Resolve an ORIENTED_EDGE entity to an Edge object.
    /// ORIENTED_EDGE params: [name, *, *, edge_curve_ref, orientation]
    fn resolve_oriented_edge(&self, oe_id: i64) -> Option<TopoEdge> {
        let (edge, _) = self.resolve_oriented_edge_with_step_id(oe_id)?;
        Some(edge)
    }

    /// Resolve an ORIENTED_EDGE entity, returning both the edge and its STEP EDGE_CURVE ID.
    fn resolve_oriented_edge_with_step_id(&self, oe_id: i64) -> Option<(TopoEdge, i64)> {
        let oe_entity = self.step.find_entity(oe_id)?;

        // ORIENTED_EDGE('', *, *, #edge_curve_ref, .T./.F.)
        // The edge_curve_ref is typically the 4th parameter (index 3)
        // The orientation is typically the 5th parameter (index 4)
        let mut edge_curve_id: Option<i64> = None;
        let mut orientation = true;

        // Find the edge curve reference and orientation
        for (_i, param) in oe_entity.params.iter().enumerate() {
            if let Some(ref_id) = self.get_ref(param) {
                // Check if this reference points to an EDGE_CURVE
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "EDGE_CURVE" {
                        edge_curve_id = Some(ref_id);
                    }
                }
            }
            // Check for orientation enum
            if let StepValue::Enum(e) = param {
                orientation = e == "T";
            }
        }

        let edge_curve_id_val = edge_curve_id?;
        let mut edge = self.resolve_edge_curve(edge_curve_id_val)?;

        // If the oriented edge is reversed relative to the edge curve, reverse it
        if !orientation {
            edge = edge.reversed();
        }

        Some((edge, edge_curve_id_val))
    }

    /// Resolve an EDGE_CURVE entity to an Edge object.
    /// EDGE_CURVE params: [name, vertex1_ref, vertex2_ref, curve_ref, orientation]
    /// Some files omit the name: EDGE_CURVE(#v1, #v2, #curve, .T.)
    /// We handle both cases by scanning parameters for their entity types.
    fn resolve_edge_curve(&self, edge_curve_id: i64) -> Option<TopoEdge> {
        let ec_entity = self.step.find_entity(edge_curve_id)?;

        let mut vertex_ids: Vec<i64> = Vec::new();
        let mut curve_ref_id: Option<i64> = None;

        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "VERTEX_POINT" {
                        vertex_ids.push(ref_id);
                    } else if entity.type_name == "EDGE_CURVE" {
                        // Nested edge curve (shouldn't happen, but handle gracefully)
                    } else if self.is_curve_type(&entity.type_name)
                        || entity.type_name == "SURFACE_CURVE"
                    {
                        curve_ref_id = Some(ref_id);
                    }
                }
            }
        }

        // Resolve vertex points
        let p1 = vertex_ids.get(0).and_then(|id| self.resolve_vertex_point(*id));
        let p2 = vertex_ids.get(1).and_then(|id| self.resolve_vertex_point(*id));

        // If we have both vertex points but no curve ref, create a line edge
        let curve_ref_id = match curve_ref_id {
            Some(id) => id,
            None => {
                if let (Some(p1), Some(p2)) = (&p1, &p2) {
                    return Some(TopoEdge::new_line(*p1, *p2));
                }
                return None;
            }
        };

        // Resolve the 3D curve (possibly through SURFACE_CURVE)
        let resolved_curve_id = self.resolve_3d_curve_ref(curve_ref_id);
        let curve = match resolved_curve_id {
            Some(id) => self.resolve_curve(id, 0),
            None => self.resolve_curve(curve_ref_id, 0),
        };

        match (curve, &p1, &p2) {
            (Some(curve), Some(p1), Some(p2)) => {
                // We have both curve and vertex points — create edge with vertex info
                // Use vertex points to determine param_range for the curve
                let curve_type_name = match &curve {
                    Curve3d::Line(_) => "Line",
                    Curve3d::Circle(_) => "Circle",
                    Curve3d::Ellipse(_) => "Ellipse",
                    Curve3d::Arc(_) => "Arc",
                    Curve3d::Hyperbola(_) => "Hyperbola",
                    Curve3d::Parabola(_) => "Parabola",
                    Curve3d::Nurbs(_) => "Nurbs",
                    Curve3d::PCurve { .. } => "PCurve",
                    Curve3d::Trimmed { .. } => "Trimmed",
                    Curve3d::Composite { .. } => "Composite",
                };
                let edge = if let Curve3d::Line(ref line) = curve {
                    // For lines, compute param range from vertex projections.
                    //
                    // ROBUSTNESS: First check whether both vertex points actually
                    // lie on the STEP LINE (within a small tolerance). If they do,
                    // we use the LINE's own geometry (origin, direction) and just
                    // compute the param_range via projection — this is the standard
                    // case for well-formed STEP files.
                    //
                    // If a vertex point does NOT lie on the LINE — which happens
                    // in some hand-crafted or buggy STEP files where the LINE's
                    // direction vector was set incorrectly (e.g., the test file
                    // nist_chamfer_block.stp has chamfer edges with direction
                    // (0,0,-1) when they should be (0,-0.7071,-0.7071)) — we
                    // OVERRIDE the line geometry with a new line through the two
                    // vertex points. The vertex positions are authoritative for
                    // topology; the curve geometry is just a hint.
                    //
                    // DISAMBIGUATION: Some STEP files (e.g., nist_cylinder.stp)
                    // intentionally have a vertex at the center of a circle rather
                    // than on the circle itself — a "topologically degenerate"
                    // vertex. For edges connecting to such a vertex, the LINE
                    // geometry is correct and the vertex position is wrong. We
                    // distinguish these cases by checking the ANGLE between the
                    // line's direction and the vertex-to-vertex direction:
                    //   - If the angle is small (< 30°), the line is mostly
                    //     aligned with the vertices — the line is correct and
                    //     the vertex is "off" for some other reason (degenerate).
                    //   - If the angle is large (>= 30°), the line's direction
                    //     is inconsistent with the vertices — the line is wrong
                    //     and we should override.
                    //
                    // NOTE: This override is only for LINES. CIRCLES and other
                    // curves retain the STEP curve's geometry (see comment above
                    // about nist_cylinder.stp having a vertex at the circle center).
                    let t1_proj = project_point_on_line(line, p1);
                    let t2_proj = project_point_on_line(line, p2);
                    let p1_on_line = line.point_at(t1_proj);
                    let p2_on_line = line.point_at(t2_proj);
                    let d1_sq = (p1_on_line.x - p1.x).powi(2)
                        + (p1_on_line.y - p1.y).powi(2)
                        + (p1_on_line.z - p1.z).powi(2);
                    let d2_sq = (p2_on_line.x - p2.x).powi(2)
                        + (p2_on_line.y - p2.y).powi(2)
                        + (p2_on_line.z - p2.z).powi(2);
                    // Tolerance: 1e-6 of the larger coordinate magnitude, with a
                    // 1e-9 floor for tiny geometries. This is loose enough to
                    // tolerate FP drift in CAD-exported files but tight enough
                    // to catch genuinely wrong line directions.
                    let coord_scale = p1.x.abs().max(p1.y.abs()).max(p1.z.abs())
                        .max(p2.x.abs()).max(p2.y.abs()).max(p2.z.abs())
                        .max(1.0);
                    let tol_sq = (coord_scale * 1e-6).powi(2);

                    // Decide whether to override the line.
                    // - both_on_line: use line as-is (standard case)
                    // - both_off_line: override (line is definitely wrong)
                    // - one_off_line: check angle to decide
                    let both_on_line = d1_sq <= tol_sq && d2_sq <= tol_sq;
                    let both_off_line = d1_sq > tol_sq && d2_sq > tol_sq;
                    let should_override = if both_on_line {
                        false
                    } else if both_off_line {
                        true
                    } else {
                        // Exactly one vertex is off the line. Check the angle
                        // between line direction and vertex-to-vertex direction.
                        let v2v_x = p2.x - p1.x;
                        let v2v_y = p2.y - p1.y;
                        let v2v_z = p2.z - p1.z;
                        let v2v_len = (v2v_x * v2v_x + v2v_y * v2v_y + v2v_z * v2v_z).sqrt();
                        if v2v_len < 1e-12 {
                            // Vertices coincide — can't compute angle, don't override
                            false
                        } else {
                            // Dot product of unit vectors (line.direction is already unit)
                            let dot = (line.direction.x * v2v_x
                                + line.direction.y * v2v_y
                                + line.direction.z * v2v_z) / v2v_len;
                            // |dot| = cos(angle). If |dot| < cos(30°) ≈ 0.866,
                            // the angle is > 30° → line direction is inconsistent.
                            // Use abs because the line might be parametrized in
                            // the opposite direction (which is fine).
                            dot.abs() < 0.866
                        }
                    };

                    if !should_override {
                        // Use line geometry as-is
                        log::debug!("    EDGE_CURVE #{}: {} p1=({:.4},{:.4},{:.4}) p2=({:.4},{:.4},{:.4}) param=({:.6},{:.6})",
                            edge_curve_id, curve_type_name, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, t1_proj, t2_proj);
                        let mut edge = TopoEdge::new(curve, (t1_proj, t2_proj));
                        edge.vertex_start = Some(draper_topology::TopoId::new());
                        edge.vertex_end = Some(draper_topology::TopoId::new());
                        edge
                    } else {
                        // Override with line through vertices
                        log::warn!(
                            "    EDGE_CURVE #{}: LINE direction inconsistent with vertices (d1={:.2e}, d2={:.2e}, tol={:.2e}) — overriding with line through p1->p2",
                            edge_curve_id, d1_sq.sqrt(), d2_sq.sqrt(), tol_sq.sqrt());
                        if let Some(new_line) = draper_geometry::Line::through_points(*p1, *p2) {
                            // Line::through_points normalizes the direction, so we
                            // must use param_range = (0, |p2-p1|) to cover the full
                            // edge. Using (0, 1) would only cover a unit-length
                            // segment, which is shorter than the actual edge for
                            // most geometries.
                            let edge_len = ((p2.x - p1.x).powi(2)
                                + (p2.y - p1.y).powi(2)
                                + (p2.z - p1.z).powi(2)).sqrt();
                            let mut edge = TopoEdge::new(
                                Curve3d::Line(new_line),
                                (0.0, edge_len),
                            );
                            edge.vertex_start = Some(draper_topology::TopoId::new());
                            edge.vertex_end = Some(draper_topology::TopoId::new());
                            edge
                        } else {
                            // Degenerate (vertices coincide) — fallback to original line
                            log::warn!(
                                "    EDGE_CURVE #{}: p1 and p2 coincide — falling back to original line",
                                edge_curve_id);
                            let mut edge = TopoEdge::new(curve, (t1_proj, t2_proj));
                            edge.vertex_start = Some(draper_topology::TopoId::new());
                            edge.vertex_end = Some(draper_topology::TopoId::new());
                            edge
                        }
                    }
                } else if let Curve3d::Circle(ref circle) = curve {
                    // For circles, compute angular range from vertex projections
                    let (t1, t2) = project_points_on_circle(circle, p1, p2);
                    log::debug!("    EDGE_CURVE #{}: {} p1=({:.4},{:.4},{:.4}) p2=({:.4},{:.4},{:.4}) param=({:.6},{:.6}) center=({:.4},{:.4},{:.4}) r={:.4} normal=({:.4},{:.4},{:.4}) x_axis=({:.4},{:.4},{:.4})",
                        edge_curve_id, curve_type_name, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, t1, t2,
                        circle.center.x, circle.center.y, circle.center.z, circle.radius,
                        circle.normal.x, circle.normal.y, circle.normal.z,
                        circle.x_axis.x, circle.x_axis.y, circle.x_axis.z);
                    let mut edge = TopoEdge::new(curve, (t1, t2));
                    edge.vertex_start = Some(draper_topology::TopoId::new());
                    edge.vertex_end = Some(draper_topology::TopoId::new());
                    edge
                } else {
                    // For other curves, use the default param range
                    let param_range = curve.param_range();
                    log::debug!("    EDGE_CURVE #{}: {} p1=({:.4},{:.4},{:.4}) p2=({:.4},{:.4},{:.4}) param=({:.6},{:.6})",
                        edge_curve_id, curve_type_name, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, param_range.0, param_range.1);
                    let mut edge = TopoEdge::new(curve, param_range);
                    edge.vertex_start = Some(draper_topology::TopoId::new());
                    edge.vertex_end = Some(draper_topology::TopoId::new());
                    edge
                };
                Some(edge)
            }
            (Some(curve), _, _) => {
                // Curve but missing vertex points — use default param range
                let param_range = curve.param_range();
                Some(TopoEdge::new(curve, param_range))
            }
            (None, Some(p1), Some(p2)) => {
                // No curve but have vertex points — create a line edge
                log::debug!("    EDGE_CURVE #{}: NO CURVE, falling back to LINE p1=({:.4},{:.4},{:.4}) p2=({:.4},{:.4},{:.4})",
                    edge_curve_id, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
                Some(TopoEdge::new_line(*p1, *p2))
            }
            _ => {
                log::debug!("    EDGE_CURVE #{}: RESOLUTION FAILED", edge_curve_id);
                None
            }
        }
    }

    /// Get the canonical VERTEX_POINT entity ID pair for an EDGE_CURVE.
    ///
    /// Each EDGE_CURVE references exactly two VERTEX_POINT entities (start and end).
    /// When two different EDGE_CURVEs share the same pair of VERTEX_POINT entities,
    /// they represent the same geometric boundary (different curve representations
    /// of the same edge). Returns the vertex IDs in canonical order (min, max)
    /// for consistent hashing.
    fn get_edge_curve_vertex_pair(&self, edge_curve_id: i64) -> Option<(i64, i64)> {
        let ec_entity = self.step.find_entity(edge_curve_id)?;
        let mut vertex_ids: Vec<i64> = Vec::new();
        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "VERTEX_POINT" {
                        vertex_ids.push(ref_id);
                    }
                }
            }
        }
        if vertex_ids.len() >= 2 {
            let (v1, v2) = (vertex_ids[0], vertex_ids[1]);
            Some(if v1 < v2 { (v1, v2) } else { (v2, v1) })
        } else {
            None
        }
    }

    /// Estimate the "complexity" of an EDGE_CURVE's underlying curve geometry.
    ///
    /// Higher scores indicate curves that require more sample points for
    /// accurate representation. This is used to choose the canonical step_id
    /// when multiple EDGE_CURVEs share the same vertex pair: the most complex
    /// curve should be canonical so its denser sampling is used for all
    /// aliased edges.
    ///
    /// Score assignment:
    /// - NURBS curves: 1000 (highest — need many control points)
    /// - Circle/Arc/Ellipse: 100 (moderate — need angular samples)
    /// - Line: 1 (lowest — only needs 2 points)
    /// - Unknown/none: 0
    fn edge_curve_complexity_score(&self, edge_curve_id: i64) -> i32 {
        let ec_entity = match self.step.find_entity(edge_curve_id) {
            Some(e) => e,
            None => return 0,
        };

        // Find the curve reference in the EDGE_CURVE params
        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    match entity.type_name.as_str() {
                        "NURBS_CURVE" | "BSPLINE_CURVE_WITH_KNOTS" | "BSPLINE_CURVE" |
                        "RATIONAL_BSPLINE_CURVE" | "SURFACE_CURVE" => return 1000,
                        "CIRCLE" | "ARC" | "ELLIPSE" | "TRIMMED_CURVE" => return 100,
                        "LINE" => return 1,
                        _ => {}
                    }
                    // SURFACE_CURVE wraps a 3D curve — recurse to find its type
                    if entity.type_name == "SURFACE_CURVE" {
                        if let Some(inner_id) = self.resolve_3d_curve_ref(ref_id) {
                            if let Some(inner_entity) = self.step.find_entity(inner_id) {
                                match inner_entity.type_name.as_str() {
                                    "NURBS_CURVE" | "BSPLINE_CURVE_WITH_KNOTS" | "BSPLINE_CURVE" |
                                    "RATIONAL_BSPLINE_CURVE" => return 1000,
                                    "CIRCLE" | "ARC" | "ELLIPSE" => return 100,
                                    "LINE" => return 1,
                                    _ => {}
                                }
                            }
                        }
                    }
                }
            }
        }
        0
    }

    /// Group step_ids by curve shape using 5-point sampling along each curve.
    ///
    /// P2 improvement over the previous midpoint-only grouping: instead of
    /// comparing just the parametric midpoint (1 point), we sample 5 interior
    /// points at t = 0.1, 0.3, 0.5, 0.7, 0.9 and require ALL 5 (or as many
    /// as we could compute) to match within tolerance.
    ///
    /// This catches:
    /// - Two semicircles that share endpoints and midpoint (a single midpoint
    ///   at t=0.5 lies on the diameter, which is the same for both arcs).
    ///   The samples at t=0.1 and t=0.9 differ.
    /// - Two B-spline curves that share endpoints and centroid but have
    ///   different control polygons (different shape).
    ///
    /// Returns a Vec of (representative_sample_set, step_ids_in_group) pairs.
    /// step_ids that couldn't be sampled (curve resolution failed) each get
    /// their own group with an empty sample set — they will never be aliased.
    fn group_step_ids_by_curve_shape(
        &self,
        step_ids: &[i64],
        tol: f64,
    ) -> Vec<(Vec<Point3d>, Vec<i64>)> {
        let mut groups: Vec<(Vec<Point3d>, Vec<i64>)> = Vec::new();
        for &sid in step_ids {
            let samples = self.compute_edge_curve_sample_points(sid).unwrap_or_default();
            // Find an existing group with matching samples.
            // A "match" means: same number of samples AND every sample
            // within `tol` of the corresponding group sample.
            let mut found_group = false;
            for (group_samples, group_sids) in groups.iter_mut() {
                if group_samples.len() != samples.len() {
                    continue;
                }
                if samples.is_empty() {
                    // Both empty — treat as same group only if we're the
                    // first one. Otherwise, isolated step_ids stay isolated.
                    if group_sids.is_empty() {
                        group_sids.push(sid);
                        found_group = true;
                        break;
                    }
                    continue;
                }
                let mut all_match = true;
                for (a, b) in samples.iter().zip(group_samples.iter()) {
                    let dx = a.x - b.x;
                    let dy = a.y - b.y;
                    let dz = a.z - b.z;
                    if (dx * dx + dy * dy + dz * dz).sqrt() > tol {
                        all_match = false;
                        break;
                    }
                }
                if all_match {
                    group_sids.push(sid);
                    found_group = true;
                    break;
                }
            }
            if !found_group {
                groups.push((samples, vec![sid]));
            }
        }
        groups
    }

    /// Compute the 3D midpoint of an EDGE_CURVE's underlying curve.
    ///
    /// This is used to distinguish between two DIFFERENT curves that share
    /// the same VERTEX_POINT endpoints (e.g., two half-circles forming a
    /// full circle). Such curves must NOT be aliased together, because they
    /// represent different geometric boundaries.
    ///
    /// The midpoint is computed by evaluating the curve at t=0.5 (the middle
    /// of the parameter range). For curves where we can't determine the
    /// midpoint, returns None.
    fn compute_edge_curve_midpoint(&self, edge_curve_id: i64) -> Option<Point3d> {
        // Use resolve_edge_curve to get a proper TopoEdge with the correct
        // curve geometry and parameter range. This handles CIRCLEs, LINEs,
        // and other curves correctly.
        if let Some(edge) = self.resolve_edge_curve(edge_curve_id) {
            // Evaluate at the parametric midpoint
            let (t1, t2) = edge.param_range;
            let mid_t = (t1 + t2) * 0.5;
            if let Some(p) = edge.point_at(mid_t) {
                return Some(p);
            }
        }

        // Fallback: try to find the curve entity and evaluate B_SPLINE midpoint
        let ec_entity = self.step.find_entity(edge_curve_id)?;
        let mut curve_id: Option<i64> = None;
        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    match entity.type_name.as_str() {
                        "B_SPLINE_CURVE_WITH_KNOTS" | "BSPLINE_CURVE_WITH_KNOTS" |
                        "B_SPLINE_CURVE" | "BSPLINE_CURVE" |
                        "RATIONAL_B_SPLINE_CURVE" | "RATIONAL_BSPLINE_CURVE" |
                        "CIRCLE" | "ARC" | "ELLIPSE" | "HYPERBOLA" | "PARABOLA" |
                        "TRIMMED_CURVE" | "LINE" |
                        "SURFACE_CURVE" => {
                            curve_id = Some(ref_id);
                            break;
                        }
                        _ => {}
                    }
                }
            }
        }

        if let Some(curve_id) = curve_id {
            let curve_id = self.resolve_3d_curve_ref(curve_id).unwrap_or(curve_id);
            if let Some(curve_entity) = self.step.find_entity(curve_id) {
                if curve_entity.type_name.contains("B_SPLINE_CURVE") || curve_entity.type_name.contains("BSPLINE_CURVE") {
                    if let Some(mid) = self.evaluate_bspline_curve_at_midpoint(curve_id, &curve_entity) {
                        return Some(mid);
                    }
                }
            }
        }

        // UNIVERSAL FALLBACK: For LINEs (where chord midpoint == arc midpoint),
        // use the average of the edge's two VERTEX_POINT endpoints.
        //
        // For CIRCLEs and other curves, the chord midpoint is NOT the arc
        // midpoint — two different arcs with the same endpoints (e.g., two
        // semicircles forming a full circle) would have the SAME chord
        // midpoint and be incorrectly aliased. To avoid this, we return None
        // for non-LINE curves when we can't compute the true arc midpoint.
        //
        // This means some valid aliases (two STEP entities representing the
        // same CIRCLE arc) will be missed, but that's safer than incorrectly
        // aliasing different arcs.
        if let Some(curve_id) = curve_id {
            let curve_id = self.resolve_3d_curve_ref(curve_id).unwrap_or(curve_id);
            if let Some(curve_entity) = self.step.find_entity(curve_id) {
                if curve_entity.type_name == "LINE" {
                    if let Some((v1, v2)) = self.get_edge_curve_vertex_pair_3d(edge_curve_id) {
                        return Some(Point3d::new(
                            (v1.x + v2.x) * 0.5,
                            (v1.y + v2.y) * 0.5,
                            (v1.z + v2.z) * 0.5,
                        ));
                    }
                }
            }
        }

        None
    }

    /// Sample 5 interior points along an edge curve at t = 0.1, 0.3, 0.5, 0.7, 0.9.
    ///
    /// Used by Phase 1 aliasing to compare edges by their full shape, not just
    /// the midpoint. Two CIRCLE arcs that share endpoints but go in opposite
    /// directions (e.g., two semicircles forming a full circle) have the same
    /// midpoint but different sample points at t=0.1, 0.3, 0.7, 0.9.
    ///
    /// Returns `None` if no samples can be computed (e.g., the curve entity
    /// cannot be resolved or `point_at` fails for all 5 parameters). Returns
    /// fewer than 5 points if some evaluations fail (callers should treat
    /// `None` as "unknown" and `Some(vec)` of any length as a usable signature).
    ///
    /// For LINEs (where samples are determined by endpoints), this still works
    /// correctly — two LINEs with the same endpoints produce identical samples.
    fn compute_edge_curve_sample_points(&self, edge_curve_id: i64) -> Option<Vec<Point3d>> {
        // Primary path: use resolve_edge_curve to get a proper TopoEdge.
        if let Some(edge) = self.resolve_edge_curve(edge_curve_id) {
            let (t1, t2) = edge.param_range;
            // Avoid degenerate param_range (zero-length edge).
            if (t2 - t1).abs() < 1e-15 {
                return None;
            }
            let mut samples: Vec<Point3d> = Vec::with_capacity(5);
            for &frac in &[0.1, 0.3, 0.5, 0.7, 0.9] {
                let t = t1 + (t2 - t1) * frac;
                if let Some(p) = edge.point_at(t) {
                    samples.push(p);
                }
            }
            if !samples.is_empty() {
                return Some(samples);
            }
        }

        // Fallback: B_SPLINE_CURVE — compute 5 sample points from control
        // point polygon by interpolating at the same fractions. This is a
        // coarse approximation but sufficient for aliasing (we just need
        // a stable signature that distinguishes different curves).
        let ec_entity = self.step.find_entity(edge_curve_id)?;
        let mut curve_id: Option<i64> = None;
        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    match entity.type_name.as_str() {
                        "B_SPLINE_CURVE_WITH_KNOTS" | "BSPLINE_CURVE_WITH_KNOTS" |
                        "B_SPLINE_CURVE" | "BSPLINE_CURVE" |
                        "RATIONAL_B_SPLINE_CURVE" | "RATIONAL_BSPLINE_CURVE" |
                        "CIRCLE" | "ARC" | "ELLIPSE" | "HYPERBOLA" | "PARABOLA" |
                        "TRIMMED_CURVE" | "LINE" |
                        "SURFACE_CURVE" => {
                            curve_id = Some(ref_id);
                            break;
                        }
                        _ => {}
                    }
                }
            }
        }

        if let Some(curve_id) = curve_id {
            let curve_id = self.resolve_3d_curve_ref(curve_id).unwrap_or(curve_id);
            if let Some(curve_entity) = self.step.find_entity(curve_id) {
                if curve_entity.type_name.contains("B_SPLINE_CURVE")
                    || curve_entity.type_name.contains("BSPLINE_CURVE")
                {
                    // For B-splines, sample the control polygon at the same
                    // fractions. Different B-splines with the same endpoints
                    // almost always have different control polygons, so this
                    // is a usable signature.
                    if let Some(cps) = self.extract_bspline_control_points(curve_id, &curve_entity) {
                        if cps.len() >= 2 {
                            let mut samples: Vec<Point3d> = Vec::with_capacity(5);
                            for &frac in &[0.1, 0.3, 0.5, 0.7, 0.9] {
                                // Linear interp along control polygon indices
                                let idx_f = frac * (cps.len() - 1) as f64;
                                let idx0 = idx_f.floor() as usize;
                                let idx1 = (idx0 + 1).min(cps.len() - 1);
                                let t = idx_f - idx0 as f64;
                                samples.push(Point3d::new(
                                    cps[idx0].x * (1.0 - t) + cps[idx1].x * t,
                                    cps[idx0].y * (1.0 - t) + cps[idx1].y * t,
                                    cps[idx0].z * (1.0 - t) + cps[idx1].z * t,
                                ));
                            }
                            return Some(samples);
                        }
                    }
                }
                // For LINEs, samples are determined by endpoints (degenerate
                // case where samples match the linear interpolation between
                // the two VERTEX_POINTs). Two LINEs with the same endpoints
                // produce identical samples, which is correct.
                if curve_entity.type_name == "LINE" {
                    if let Some((v1, v2)) = self.get_edge_curve_vertex_pair_3d(edge_curve_id) {
                        let mut samples: Vec<Point3d> = Vec::with_capacity(5);
                        for &frac in &[0.1, 0.3, 0.5, 0.7, 0.9] {
                            samples.push(Point3d::new(
                                v1.x * (1.0 - frac) + v2.x * frac,
                                v1.y * (1.0 - frac) + v2.y * frac,
                                v1.z * (1.0 - frac) + v2.z * frac,
                            ));
                        }
                        return Some(samples);
                    }
                }
            }
        }

        None
    }

    /// Extract control points from a B_SPLINE_CURVE entity.
    /// Helper for `compute_edge_curve_sample_points`.
    fn extract_bspline_control_points(
        &self,
        _curve_id: i64,
        curve_entity: &crate::schema::StepEntity,
    ) -> Option<Vec<Point3d>> {
        let mut control_points: Vec<Point3d> = Vec::new();
        for param in &curve_entity.params {
            if let crate::schema::StepValue::List(items) = param {
                for item in items {
                    if let Some(cp_ref) = self.get_ref(item) {
                        if let Some(p) = self.resolve_cartesian_point(cp_ref) {
                            control_points.push(p);
                        }
                    }
                }
            }
        }
        if control_points.is_empty() {
            None
        } else {
            Some(control_points)
        }
    }

    /// Get the 3D coordinates of an EDGE_CURVE's two VERTEX_POINT endpoints.
    fn get_edge_curve_vertex_pair_3d(&self, edge_curve_id: i64) -> Option<(Point3d, Point3d)> {
        let ec_entity = self.step.find_entity(edge_curve_id)?;
        let mut points: Vec<Point3d> = Vec::new();
        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "VERTEX_POINT" {
                        // VERTEX_POINT('', #cartesian_point_ref)
                        // The cartesian_point_ref is the FIRST Ref param (not
                        // necessarily params[0], which might be the name string).
                        // Iterate through params to find the first Ref.
                        for vp_param in &entity.params {
                            if let Some(cp_ref) = self.get_ref(vp_param) {
                                if let Some(p) = self.resolve_cartesian_point(cp_ref) {
                                    points.push(p);
                                    break; // Only need the first cartesian point ref
                                }
                            }
                        }
                    }
                }
            }
        }
        if points.len() >= 2 {
            Some((points[0], points[1]))
        } else {
            None
        }
    }

    /// Evaluate a B_SPLINE_CURVE at its parameter midpoint.
    fn evaluate_bspline_curve_at_midpoint(&self, _curve_id: i64, curve_entity: &crate::schema::StepEntity) -> Option<Point3d> {
        // B_SPLINE_CURVE_WITH_KNOTS format:
        // (degree, (control_points...), .UNSPECIFIED., .F., .F., knots, weights)
        // We extract control points and compute their centroid as an approximation.
        let mut control_points: Vec<Point3d> = Vec::new();
        for param in &curve_entity.params {
            if let crate::schema::StepValue::List(items) = param {
                for item in items {
                    if let Some(cp_ref) = self.get_ref(item) {
                        if let Some(p) = self.resolve_cartesian_point(cp_ref) {
                            control_points.push(p);
                        }
                    }
                }
            }
        }
        if control_points.is_empty() {
            return None;
        }
        // Use the middle control point as the midpoint approximation
        let mid_idx = control_points.len() / 2;
        Some(control_points[mid_idx])
    }

    /// Build a TopoEdge from a curve entity (for midpoint evaluation).
    fn extract_edge_from_curve_entity(&self, curve_id: i64, _curve_entity: &crate::schema::StepEntity) -> Option<TopoEdge> {
        // This is a simplified extraction — we just need point_at(0.5) to work.
        // The full edge extraction is complex, so we delegate to the existing
        // extract_edge_curve_data function if available.
        // For now, return None to fall back to the B_SPLINE midpoint approximation.
        let _ = (curve_id,);
        None
    }

    /// Resolve a SURFACE_CURVE entity to get the 3D curve reference.
    /// SURFACE_CURVE params: [name, curve3d_ref, (pcurve_refs), .PCURVE_S1.]
    fn resolve_3d_curve_ref(&self, surface_curve_id: i64) -> Option<i64> {
        let sc_entity = self.step.find_entity(surface_curve_id)?;
        
        if sc_entity.type_name != "SURFACE_CURVE" {
            return Some(surface_curve_id); // Not a surface curve, return as-is
        }

        // SURFACE_CURVE('', #curve3d_ref, (#pcurve1, #pcurve2), .PCURVE_S1.)
        // The 3D curve is the 2nd parameter (index 1)
        if let Some(param) = sc_entity.params.get(1) {
            if let Some(curve3d_id) = self.get_ref(param) {
                if let Some(curve_entity) = self.step.find_entity(curve3d_id) {
                    if self.is_curve_type(&curve_entity.type_name) {
                        return Some(curve3d_id);
                    }
                }
            }
        }

        // Fallback: search all params for a curve reference
        for param in &sc_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&entity.type_name) {
                        return Some(ref_id);
                    }
                }
            }
            // Also check inside lists (pcurve refs might be in a list)
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if self.is_curve_type(&entity.type_name) {
                                return Some(ref_id);
                            }
                        }
                    }
                }
            }
        }

        None
    }

    /// Extract the STEP surface entity ID from an ADVANCED_FACE entity.
    /// This is used for PCURVE matching — the PCURVE that references the same
    /// surface as the face is the one we want.
    fn extract_face_surface_step_id(&self, face: &crate::schema::StepEntity) -> Option<i64> {
        // Format: #N = ADVANCED_FACE('', (bounds), #surface_ref, .T.);
        // The surface reference is typically the 3rd parameter (index 2).
        if let Some(param) = face.params.get(2) {
            if let Some(surface_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(surface_id) {
                    let tn = entity.type_name.as_str();
                    if tn.contains("SURFACE") || tn == "PLANE" || tn == "CYLINDRICAL_SURFACE"
                        || tn == "SPHERICAL_SURFACE" || tn == "CONICAL_SURFACE"
                        || tn == "TOROIDAL_SURFACE"
                    {
                        return Some(surface_id);
                    }
                }
            }
        }
        // Fallback: scan all params for a surface ref
        for param in &face.params {
            if let Some(surface_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(surface_id) {
                    let tn = entity.type_name.as_str();
                    if tn.contains("SURFACE") || tn == "PLANE" || tn == "CYLINDRICAL_SURFACE"
                        || tn == "SPHERICAL_SURFACE" || tn == "CONICAL_SURFACE"
                        || tn == "TOROIDAL_SURFACE"
                    {
                        return Some(surface_id);
                    }
                }
            }
        }
        None
    }

    /// Extract analytical PCURVEs (Curve2d) for edges in a face.
    /// Traverses: ADVANCED_FACE → FACE_BOUND → EDGE_LOOP → ORIENTED_EDGE →
    ///            EDGE_CURVE → SURFACE_CURVE → PCURVE → DEFINITIONAL_REPRESENTATION → 2D curve
    fn extract_edge_curves_2d(
        &self,
        face_entity: &crate::schema::StepEntity,
        edges: &[TopoEdge],
        surface_step_id: Option<i64>,
    ) -> Vec<Option<Curve2d>> {
        let surface_id = match surface_step_id {
            Some(id) => id,
            None => return vec![None; edges.len()],
        };

        // Collect all ORIENTED_EDGE entity IDs from the face bounds
        let mut oriented_edge_ids: Vec<i64> = Vec::new();
        for param in &face_entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(bound_id) = self.get_ref(item) {
                        if let Some(bound_entity) = self.step.find_entity(bound_id) {
                            if bound_entity.type_name == "FACE_OUTER_BOUND"
                                || bound_entity.type_name == "FACE_BOUND"
                            {
                                // FACE_BOUND('', #loop_ref, .T.)
                                for bp in &bound_entity.params {
                                    if let Some(loop_id) = self.get_ref(bp) {
                                        if let Some(loop_entity) = self.step.find_entity(loop_id) {
                                            if loop_entity.type_name == "EDGE_LOOP" {
                                                self.collect_oriented_edge_ids(loop_entity, &mut oriented_edge_ids);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // For each ORIENTED_EDGE, trace to SURFACE_CURVE and extract PCURVE
        let mut edge_curve_2d_map: HashMap<draper_topology::TopoId, Curve2d> = HashMap::new();

        for oe_id in oriented_edge_ids {
            if let Some(oe_entity) = self.step.find_entity(oe_id) {
                let mut edge_curve_id: Option<i64> = None;
                let mut orientation = true;

                for param in &oe_entity.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name == "EDGE_CURVE" {
                                edge_curve_id = Some(ref_id);
                            }
                        }
                    }
                    if let StepValue::Enum(e) = param {
                        orientation = e == "T";
                    }
                }

                if let Some(ec_id) = edge_curve_id {
                    // Get the SURFACE_CURVE ID from the EDGE_CURVE
                    if let Some(sc_id) = self.find_surface_curve_from_edge_curve(ec_id) {
                        // Find the PCURVE matching our surface
                        if let Some(curve_2d) = self.extract_pcurve_for_surface(sc_id, surface_id) {
                            // Find the edge TopoId that this oriented edge resolves to
                            if let Some(edge) = self.resolve_edge_curve(ec_id) {
                                let mut final_edge = edge;
                                if !orientation {
                                    final_edge = final_edge.reversed();
                                }
                                // Store by edge ID
                                edge_curve_2d_map.insert(final_edge.id, curve_2d);
                            }
                        }
                    }
                }
            }
        }

        // Map edges to their Curve2d
        edges.iter().map(|e| edge_curve_2d_map.get(&e.id).cloned()).collect()
    }

    /// Collect ORIENTED_EDGE entity IDs from an EDGE_LOOP entity.
    fn collect_oriented_edge_ids(&self, loop_entity: &crate::schema::StepEntity, ids: &mut Vec<i64>) {
        for param in &loop_entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(oe_id) = self.get_ref(item) {
                        ids.push(oe_id);
                    }
                }
            }
        }
    }

    /// Find the SURFACE_CURVE entity ID from an EDGE_CURVE.
    /// The EDGE_CURVE's curve parameter might point directly to a SURFACE_CURVE
    /// or to a regular curve.
    fn find_surface_curve_from_edge_curve(&self, edge_curve_id: i64) -> Option<i64> {
        let ec_entity = self.step.find_entity(edge_curve_id)?;
        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "SURFACE_CURVE" {
                        return Some(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Extract the PCURVE Curve2d from a SURFACE_CURVE that matches a given surface.
    /// SURFACE_CURVE('', #3d_curve, (#pcurve1, #pcurve2), .PCURVE_S1.)
    fn extract_pcurve_for_surface(&self, surface_curve_id: i64, target_surface_id: i64) -> Option<Curve2d> {
        let sc_entity = self.step.find_entity(surface_curve_id)?;
        if sc_entity.type_name != "SURFACE_CURVE" {
            return None;
        }

        // The PCURVE references are in the 3rd parameter (index 2), as a list
        // SURFACE_CURVE('', #curve3d_ref, (#pcurve1, #pcurve2), .PCURVE_S1.)
        for param in &sc_entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(pcurve_id) = self.get_ref(item) {
                        if let Some(pcurve_entity) = self.step.find_entity(pcurve_id) {
                            if pcurve_entity.type_name == "PCURVE" {
                                // PCURVE('', #surface_ref, #definitional_rep)
                                // Check if this PCURVE references our target surface
                                if self.pcurve_references_surface(pcurve_entity, target_surface_id) {
                                    if let Some(curve_2d) = self.resolve_pcurve_to_curve2d(pcurve_entity) {
                                        return Some(curve_2d);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Check if a PCURVE entity references a given surface.
    /// PCURVE('', #surface_ref, #definitional_rep)
    fn pcurve_references_surface(&self, pcurve_entity: &crate::schema::StepEntity, target_surface_id: i64) -> bool {
        // The surface reference is typically the 2nd parameter (index 1)
        for param in &pcurve_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if ref_id == target_surface_id {
                    return true;
                }
                // Also check if the referenced entity is the same surface
                // (some STEP files reference through intermediate entities)
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name.contains("SURFACE") || entity.type_name == "PLANE" {
                        // Direct match
                        if ref_id == target_surface_id {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Resolve a PCURVE entity to a Curve2d.
    /// PCURVE('', #surface_ref, #definitional_rep)
    /// The DEFINITIONAL_REPRESENTATION contains the 2D curve in UV space.
    fn resolve_pcurve_to_curve2d(&self, pcurve_entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        // Find the DEFINITIONAL_REPRESENTATION reference
        // PCURVE('', #surface, #definitional_rep)
        // The definitional_rep is typically the last reference param
        let mut def_rep_id: Option<i64> = None;

        for param in &pcurve_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "DEFINITIONAL_REPRESENTATION" {
                        def_rep_id = Some(ref_id);
                    }
                }
            }
        }

        let def_rep_id = def_rep_id?;

        // DEFINITIONAL_REPRESENTATION('', (#2d_curve), #context)
        // The 2D curve is the first reference in the list parameter
        let def_rep_entity = self.step.find_entity(def_rep_id)?;
        for param in &def_rep_entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(curve_2d_id) = self.get_ref(item) {
                        // First, try resolving as a native 2D curve
                        if let Some(curve_2d) = self.resolve_curve_2d(curve_2d_id) {
                            return Some(curve_2d);
                        }

                        // Fallback: if the entity is OFFSET_CURVE_3D referenced from
                        // a PCURVE, we convert the 3D offset to a 2D approximation by
                        // sampling the 3D offset curve and projecting each point to UV
                        // space using the surface's parameterization.
                        if let Some(entity) = self.step.find_entity(curve_2d_id) {
                            if entity.type_name == "OFFSET_CURVE_3D" {
                                if let Some(curve_3d) = self.resolve_offset_curve_3d(entity, 0) {
                                    return self.project_curve_3d_to_2d(&curve_3d, pcurve_entity);
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Project a 3D curve to 2D UV space by finding the surface referenced
    /// in the PCURVE entity and using its `project_point` method to convert
    /// sampled 3D points to UV coordinates, then fitting a Nurbs2d.
    fn project_curve_3d_to_2d(&self, curve_3d: &Curve3d, pcurve_entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        // Extract the surface from the PCURVE entity
        // PCURVE('', #surface, #definitional_rep)
        let mut surface_id: Option<i64> = None;
        for param in &pcurve_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name != "DEFINITIONAL_REPRESENTATION" && surface_id.is_none() {
                        surface_id = Some(ref_id);
                    }
                }
            }
        }

        let surface_id = surface_id?;
        let surface = self.extract_surface(surface_id, 0)?;

        // Sample the 3D curve and project to UV
        let n_samples = 64;
        let (t_min, t_max) = curve_3d.param_range();
        let mut uv_points: Vec<Point2d> = Vec::with_capacity(n_samples);

        for i in 0..n_samples {
            let t = t_min + (t_max - t_min) * i as f64 / (n_samples - 1) as f64;
            let p3d = curve_3d.point_at(t);
            let (u, v) = surface.project_point(&p3d);
            if u.is_finite() && v.is_finite() {
                uv_points.push(Point2d::new(u, v));
            }
        }

        if uv_points.len() < 2 {
            return None;
        }

        // Deduplicate and fit a Nurbs2d
        uv_points = deduplicate_points_2d(&uv_points, 1e-6);

        fit_nurbs_curve_through_points_2d(&uv_points, 3)
            .map(Curve2d::Nurbs)
            .or_else(|| {
                // Fallback to degree-1 polyline
                let n = uv_points.len();
                if n < 2 { return None; }
                let degree = 1;
                let weights = vec![1.0; n];
                let mut knots = Vec::with_capacity(n + degree + 1);
                for _ in 0..=degree { knots.push(0.0); }
                for i in 1..n-1 { knots.push(i as f64); }
                for _ in 0..=degree { knots.push((n - 1) as f64); }
                Some(Curve2d::Nurbs(Nurbs2d {
                    degree,
                    control_points: uv_points,
                    weights,
                    knots,
                }))
            })
    }

    /// Resolve a 2D curve entity (in UV space) to a Curve2d.
    fn resolve_curve_2d(&self, curve_id: i64) -> Option<Curve2d> {
        let entity = self.step.find_entity(curve_id)?;
        let type_name = entity.type_name.as_str();

        // Handle complex entity types containing B_SPLINE_CURVE
        if type_name.contains("B_SPLINE_CURVE") {
            return self.resolve_bspline_curve_2d(entity);
        }

        match type_name {
            "LINE" => self.resolve_line_curve_2d(entity),
            "CIRCLE" => self.resolve_circle_curve_2d(entity),
            "ELLIPSE" => self.resolve_ellipse_curve_2d(entity),
            "HYPERBOLA" => self.resolve_hyperbola_curve_2d(entity),
            "PARABOLA" => self.resolve_parabola_curve_2d(entity),
            "B_SPLINE_CURVE_WITH_KNOTS" | "B_SPLINE_CURVE" | "BEZIER_CURVE" |
            "RATIONAL_B_SPLINE_CURVE" => self.resolve_bspline_curve_2d(entity),
            "TRIMMED_CURVE" => self.resolve_trimmed_curve_2d(entity),
            "POLYLINE" => self.resolve_polyline_curve_2d(entity),
            "OFFSET_CURVE_2D" => self.resolve_offset_curve_2d(entity, 0),
            _ => {
                log::debug!("resolve_curve_2d #{}: unsupported 2D curve type '{}'", curve_id, type_name);
                None
            }
        }
    }

    /// Resolve a 2D LINE curve to a Line2d.
    /// STEP format: LINE('', #point, #vector) in 2D
    fn resolve_line_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        // LINE('', #cartesian_point_2d, #vector_2d)
        let mut point_ref: Option<i64> = None;
        let mut dir_ref: Option<i64> = None;
        let mut magnitude: f64 = 1.0;

        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "CARTESIAN_POINT" && point_ref.is_none() {
                        point_ref = Some(ref_id);
                    } else if referenced.type_name == "DIRECTION" && dir_ref.is_none() {
                        dir_ref = Some(ref_id);
                    } else if referenced.type_name == "VECTOR" && dir_ref.is_none() {
                        // VECTOR('', #direction, magnitude)
                        dir_ref = self.find_direction_from_vector_2d(referenced);
                        magnitude = self.find_float_param(referenced, 0).unwrap_or(1.0);
                    }
                }
            }
        }

        let start = point_ref.and_then(|id| self.resolve_cartesian_point_2d(id))?;
        let direction = dir_ref.and_then(|id| self.resolve_direction_2d(id))?;

        // End point = start + direction * magnitude
        let end = Point2d::new(
            start.u + direction.0 * magnitude,
            start.v + direction.1 * magnitude,
        );

        Some(Curve2d::Line(Line2d::new(start, end)))
    }

    /// Resolve a 2D CIRCLE curve to a Circle2d.
    fn resolve_circle_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        // CIRCLE('', #axis2_2d, radius)
        let axis2_id = self.find_axis2_2d_ref(entity)?;
        let center = self.resolve_axis2_2d(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;

        Some(Curve2d::Circle(Circle2d::new_full(center, radius)))
    }

    /// Resolve a 2D ELLIPSE curve to an Ellipse2d.
    fn resolve_ellipse_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        let axis2_id = self.find_axis2_2d_ref(entity)?;
        let (center, rotation) = self.resolve_axis2_2d_with_rotation(axis2_id)?;
        let semi_major = self.find_float_param(entity, 0)?;
        let semi_minor = self.find_float_param(entity, 1)?;

        Some(Curve2d::Ellipse(Ellipse2d::new_full(center, semi_major, semi_minor, rotation)))
    }

    /// Resolve a 2D HYPERBOLA curve to a Hyperbola2d.
    /// STEP format: `#N = HYPERBOLA('', #axis2_2d, semi_real, semi_imag);`
    ///
    /// The axis2 placement provides:
    ///   - location = center of the hyperbola
    ///   - ref_direction = transverse axis direction (along which the branches open)
    ///
    /// In 2D (PCURVE context), the hyperbola is defined in UV parameter space.
    /// Since STEP does not carry explicit parameter bounds on the HYPERBOLA entity
    /// itself (those come from TRIMMED_CURVE wrapping), we use a default parameter
    /// range of [-5, 5] which is sufficient for most PCURVE hyperbolas that appear
    /// on conic surfaces. When wrapped in TRIMMED_CURVE, `resolve_trimmed_curve_2d`
    /// will override this range.
    fn resolve_hyperbola_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        let axis2_id = self.find_axis2_2d_ref(entity)?;
        let (center, rotation) = self.resolve_axis2_2d_with_rotation(axis2_id)?;
        let semi_real = self.find_float_param(entity, 0)?;
        let semi_imag = self.find_float_param(entity, 1)?;

        // Transverse axis direction from rotation angle
        let axis_u = rotation.cos();
        let axis_v = rotation.sin();

        // Default parameter range; will be overridden by TRIMMED_CURVE
        let t_start = -5.0;
        let t_end = 5.0;

        Some(Curve2d::Hyperbola(Hyperbola2d::new(
            center,
            semi_real.abs(),
            semi_imag.abs(),
            axis_u,
            axis_v,
            t_start,
            t_end,
        )))
    }

    /// Resolve a 2D PARABOLA curve to a Parabola2d.
    /// STEP format: `#N = PARABOLA('', #axis2_2d, focal_dist);`
    ///
    /// The axis2 placement provides:
    ///   - location = vertex of the parabola
    ///   - ref_direction = direction the parabola opens (toward the focus)
    ///
    /// In 2D (PCURVE context), the parabola is defined in UV parameter space.
    /// Since STEP does not carry explicit parameter bounds on the PARABOLA entity
    /// itself (those come from TRIMMED_CURVE wrapping), we use a default parameter
    /// range of [-5, 5]. When wrapped in TRIMMED_CURVE, `resolve_trimmed_curve_2d`
    /// will override this range.
    fn resolve_parabola_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        let axis2_id = self.find_axis2_2d_ref(entity)?;
        let (vertex, rotation) = self.resolve_axis2_2d_with_rotation(axis2_id)?;
        let focal_dist = self.find_float_param(entity, 0)?;

        // Axis direction from rotation angle
        let axis_u = rotation.cos();
        let axis_v = rotation.sin();

        // Default parameter range; will be overridden by TRIMMED_CURVE
        let t_start = -5.0;
        let t_end = 5.0;

        Some(Curve2d::Parabola(Parabola2d::new(
            vertex,
            focal_dist.abs(),
            axis_u,
            axis_v,
            t_start,
            t_end,
        )))
    }

    /// Resolve a 2D B_SPLINE_CURVE to a Nurbs2d.
    fn resolve_bspline_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        let bspline_sub = entity.find_sub_entity("B_SPLINE_CURVE");
        let knots_sub = entity.find_sub_entity("B_SPLINE_CURVE_WITH_KNOTS");
        let rational_sub = entity.find_sub_entity("RATIONAL_B_SPLINE_CURVE");

        let cp_entity = bspline_sub.unwrap_or(entity);
        let knot_entity = knots_sub.unwrap_or(entity);

        // Find degree
        let mut degree = None;
        let mut cp_param_idx = None;
        for (i, param) in cp_entity.params.iter().enumerate() {
            if degree.is_none() {
                if let Some(d) = self.get_float(param) {
                    degree = Some(d as usize);
                }
            } else if cp_param_idx.is_none() {
                if let StepValue::List(_) = param {
                    cp_param_idx = Some(i);
                }
            }
        }

        let degree = degree?;

        // Extract 2D control points
        let mut control_points = Vec::new();
        if let Some(cp_idx) = cp_param_idx {
            if let Some(StepValue::List(items)) = cp_entity.params.get(cp_idx) {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(pt) = self.resolve_cartesian_point_2d(ref_id) {
                            control_points.push(pt);
                        }
                    } else if let StepValue::List(coords) = item {
                        let u = coords.get(0).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                        let v = coords.get(1).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                        control_points.push(Point2d::new(u, v));
                    }
                }
            }
        }

        if control_points.is_empty() {
            return None;
        }

        // Extract weights
        let weights = if let Some(rational_ent) = rational_sub {
            self.extract_curve_weights(rational_ent, control_points.len())
        } else {
            vec![1.0; control_points.len()]
        };

        // Extract knots
        let n = control_points.len();
        let knots = self.extract_curve_knots(knot_entity, n, degree);

        Some(Curve2d::Nurbs(Nurbs2d {
            degree,
            control_points,
            weights,
            knots,
        }))
    }

    /// Resolve a 2D TRIMMED_CURVE.
    fn resolve_trimmed_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        // TRIMMED_CURVE(#basis_curve, #trim1, #trim2, .T., .T., .CARTESIAN., .CARTESIAN.)
        let basis_id = self.get_ref(entity.params.first()?)?;
        let basis = self.resolve_curve_2d(basis_id)?;

        // Extract trim values
        let mut trim1: Option<f64> = None;
        let mut trim2: Option<f64> = None;

        if entity.params.len() >= 3 {
            if let Some(param) = entity.params.get(1) {
                trim1 = self.get_float(param);
                if trim1.is_none() {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(pt) = self.resolve_cartesian_point_2d(ref_id) {
                            // Could use the point, but for now just note we have it
                            let _ = pt;
                        }
                    }
                }
            }
            if let Some(param) = entity.params.get(2) {
                trim2 = self.get_float(param);
            }
        }

        // For LINE in UV, trims define start/end points directly
        if let Curve2d::Line(ref line) = basis {
            if let (Some(t1), Some(t2)) = (trim1, trim2) {
                let start = Point2d::new(
                    line.start.u + t1 * (line.end.u - line.start.u),
                    line.start.v + t1 * (line.end.v - line.start.v),
                );
                let end = Point2d::new(
                    line.start.u + t2 * (line.end.u - line.start.u),
                    line.start.v + t2 * (line.end.v - line.start.v),
                );
                return Some(Curve2d::Line(Line2d::new(start, end)));
            }
        }

        // For CIRCLE in UV, trims define angle range
        if let Curve2d::Circle(ref circle) = basis {
            if let (Some(t1), Some(t2)) = (trim1, trim2) {
                return Some(Curve2d::Circle(Circle2d::new_arc(
                    circle.center, circle.radius, t1, t2,
                )));
            }
        }

        // For HYPERBOLA in UV, trims define parameter range [t_start, t_end]
        if let Curve2d::Hyperbola(ref hyp) = basis {
            if let (Some(t1), Some(t2)) = (trim1, trim2) {
                return Some(Curve2d::Hyperbola(Hyperbola2d::new(
                    hyp.center,
                    hyp.semi_real,
                    hyp.semi_imag,
                    hyp.axis_u,
                    hyp.axis_v,
                    t1,
                    t2,
                )));
            }
        }

        // For PARABOLA in UV, trims define parameter range [t_start, t_end]
        if let Curve2d::Parabola(ref par) = basis {
            if let (Some(t1), Some(t2)) = (trim1, trim2) {
                return Some(Curve2d::Parabola(Parabola2d::new(
                    par.vertex,
                    par.focal_dist,
                    par.axis_u,
                    par.axis_v,
                    t1,
                    t2,
                )));
            }
        }

        Some(basis)
    }

    /// Resolve a 2D POLYLINE as a series of Line2d segments (returning the first as representative).
    /// For a polyline in UV, we approximate by sampling and returning a Nurbs2d.
    fn resolve_polyline_curve_2d(&self, entity: &crate::schema::StepEntity) -> Option<Curve2d> {
        let mut points = Vec::new();
        for param in &entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(pt) = self.resolve_cartesian_point_2d(ref_id) {
                            points.push(pt);
                        }
                    }
                }
            }
        }

        if points.len() >= 2 {
            // Create a degree-1 NURBS through the polyline points
            let n = points.len();
            let degree = 1;
            let weights = vec![1.0; n];
            let mut knots = Vec::with_capacity(n + degree + 1);
            for _ in 0..=degree {
                knots.push(0.0);
            }
            for i in 1..n-1 {
                knots.push(i as f64);
            }
            for _ in 0..=degree {
                knots.push((n - 1) as f64);
            }

            Some(Curve2d::Nurbs(Nurbs2d {
                degree,
                control_points: points,
                weights,
                knots,
            }))
        } else {
            None
        }
    }

    /// Resolve an OFFSET_CURVE_2D entity with NURBS approximation.
    /// STEP format: OFFSET_CURVE_2D('', #basis_curve, distance, #ref_direction, self_intersect)
    /// The basis curve is a 2D curve in UV space; offset is applied perpendicular
    /// to the tangent direction in 2D.
    fn resolve_offset_curve_2d(&self, entity: &crate::schema::StepEntity, _depth: usize) -> Option<Curve2d> {
        let mut basis_curve_id: Option<i64> = None;
        let mut offset_dist: f64 = 0.0;
        let mut found_floats = 0;

        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    // The first curve-type reference is the basis curve
                    if self.is_2d_curve_type(&referenced.type_name) && basis_curve_id.is_none() {
                        basis_curve_id = Some(ref_id);
                    }
                }
            }
            if let Some(val) = self.get_float(param) {
                if found_floats == 0 {
                    offset_dist = val;
                }
                found_floats += 1;
            }
        }

        let basis_curve_id = basis_curve_id?;
        let basis_curve = self.resolve_curve_2d(basis_curve_id)?;

        if offset_dist.abs() < 1e-10 {
            // Zero offset — just return the basis curve
            return Some(basis_curve);
        }

        // Approximate the offset curve in 2D using Nurbs2d
        Some(approximate_offset_curve_2d(&basis_curve, offset_dist))
    }

    /// Check if a type name is a known 2D curve type (used for OFFSET_CURVE_2D basis curve detection).
    fn is_2d_curve_type(&self, type_name: &str) -> bool {
        matches!(
            type_name,
            "LINE" | "CIRCLE" | "ELLIPSE" | "HYPERBOLA" | "PARABOLA" |
            "B_SPLINE_CURVE_WITH_KNOTS" | "B_SPLINE_CURVE" | "BEZIER_CURVE" |
            "RATIONAL_B_SPLINE_CURVE" | "TRIMMED_CURVE" | "POLYLINE" |
            "OFFSET_CURVE_2D"
        ) || type_name.contains("B_SPLINE_CURVE")
    }

    /// Resolve a 2D CARTESIAN_POINT entity.
    /// CARTESIAN_POINT('', (u, v)) — 2 coordinates for UV space
    fn resolve_cartesian_point_2d(&self, point_id: i64) -> Option<Point2d> {
        let point_entity = self.step.find_entity(point_id)?;

        if point_entity.type_name != "CARTESIAN_POINT" {
            return None;
        }

        // Look for a list of coordinates (2 or 3 values — if 3, take first 2)
        for param in &point_entity.params {
            if let StepValue::List(coords) = param {
                let values: Vec<f64> = coords.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if values.len() >= 2 {
                    return Some(Point2d::new(values[0], values[1]));
                }
            }
        }
        None
    }

    /// Resolve a 2D DIRECTION entity.
    /// DIRECTION('', (du, dv)) — 2 components for UV space
    fn resolve_direction_2d(&self, dir_id: i64) -> Option<(f64, f64)> {
        let dir_entity = self.step.find_entity(dir_id)?;

        if dir_entity.type_name != "DIRECTION" {
            return None;
        }

        for param in &dir_entity.params {
            if let StepValue::List(coords) = param {
                let values: Vec<f64> = coords.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if values.len() >= 2 {
                    let len = (values[0] * values[0] + values[1] * values[1]).sqrt();
                    if len > 1e-15 {
                        return Some((values[0] / len, values[1] / len));
                    }
                }
            }
        }
        None
    }

    /// Find a DIRECTION reference from a 2D VECTOR entity.
    fn find_direction_from_vector_2d(&self, vector_entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &vector_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "DIRECTION" {
                        return Some(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Find a reference to AXIS2_PLACEMENT_2D in an entity's parameters.
    fn find_axis2_2d_ref(&self, entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "AXIS2_PLACEMENT_2D" {
                        return Some(ref_id);
                    }
                }
            }
        }
        // Fallback: AXIS2_PLACEMENT_3D can also be used in 2D context
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "AXIS2_PLACEMENT_3D" {
                        return Some(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Resolve an AXIS2_PLACEMENT_2D to a 2D center point.
    fn resolve_axis2_2d(&self, axis2_id: i64) -> Option<Point2d> {
        let axis2_entity = self.step.find_entity(axis2_id)?;

        if axis2_entity.type_name == "AXIS2_PLACEMENT_2D" {
            // AXIS2_PLACEMENT_2D('', #point_ref)
            for param in &axis2_entity.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(pt) = self.resolve_cartesian_point_2d(ref_id) {
                        return Some(pt);
                    }
                }
            }
        } else if axis2_entity.type_name == "AXIS2_PLACEMENT_3D" {
            // AXIS2_PLACEMENT_3D('', #point_ref, #axis_ref, #ref_dir_ref)
            // Extract the 2D projection of the 3D point
            for param in &axis2_entity.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                        return Some(Point2d::new(pt.x, pt.y));
                    }
                }
            }
        }
        None
    }

    /// Resolve an AXIS2_PLACEMENT_2D to a center point and rotation angle.
    fn resolve_axis2_2d_with_rotation(&self, axis2_id: i64) -> Option<(Point2d, f64)> {
        let axis2_entity = self.step.find_entity(axis2_id)?;
        let center = self.resolve_axis2_2d(axis2_id)?;

        // Try to extract the ref direction for rotation
        let mut rotation = 0.0;
        if axis2_entity.type_name == "AXIS2_PLACEMENT_2D" {
            // AXIS2_PLACEMENT_2D('', #point_ref, #direction_ref)
            let mut refs: Vec<i64> = Vec::new();
            for param in &axis2_entity.params {
                if let Some(ref_id) = self.get_ref(param) {
                    refs.push(ref_id);
                }
            }
            // The second reference (if present) is the direction
            if refs.len() >= 2 {
                if let Some((du, dv)) = self.resolve_direction_2d(refs[1]) {
                    rotation = dv.atan2(du);
                }
            }
        }

        Some((center, rotation))
    }

    /// Resolve a VERTEX_POINT entity to a 3D point.
    /// VERTEX_POINT params: [name, point_ref]
    fn resolve_vertex_point(&self, vertex_id: i64) -> Option<Point3d> {
        let vertex_entity = self.step.find_entity(vertex_id)?;
        
        if vertex_entity.type_name != "VERTEX_POINT" {
            return None;
        }

        // VERTEX_POINT('', #point_ref)
        for param in &vertex_entity.params {
            if let Some(point_id) = self.get_ref(param) {
                if let Some(point) = self.resolve_cartesian_point(point_id) {
                    return Some(point);
                }
            }
        }

        None
    }

    /// Extract a Surface from a STEP surface entity.
    fn extract_surface(&self, surface_id: i64, depth: usize) -> Option<Surface> {
        if depth > 20 {
            warn!("extract_surface depth limit reached at surface_id=#{} — returning None", surface_id);
            return None;
        }
        let entity = self.step.find_entity(surface_id)?;
        let type_name = entity.type_name.as_str();

        // Handle complex entity types (e.g., "BOUNDED_SURFACE+B_SPLINE_SURFACE+B_SPLINE_SURFACE_WITH_KNOTS+RATIONAL_B_SPLINE_SURFACE+GEOMETRIC_REPRESENTATION_ITEM+REPRESENTATION_ITEM+SURFACE")
        // by checking if the type_name contains known surface type keywords
        if type_name.contains("B_SPLINE_SURFACE") {
            return self.extract_bspline_surface(entity);
        }

        match type_name {
            "PLANE" => self.extract_plane(entity),
            "CYLINDRICAL_SURFACE" => self.extract_cylinder(entity),
            "SPHERICAL_SURFACE" => self.extract_sphere(entity),
            "CONICAL_SURFACE" => self.extract_cone(entity),
            "TOROIDAL_SURFACE" => self.extract_torus(entity),
            "SURFACE_OF_REVOLUTION" => self.extract_revolution(entity),
            "SURFACE_OF_LINEAR_EXTRUSION" => self.extract_extrusion(entity),
            "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" | "BEZIER_SURFACE" => {
                self.extract_bspline_surface(entity)
            }
            "RECTANGULAR_TRIMMED_SURFACE" => self.extract_trimmed_surface(entity, depth + 1),
            "SWEPT_SURFACE" => self.extract_swept_surface(entity),
            "OFFSET_SURFACE" => self.extract_offset_surface(entity, depth + 1),
            _ => {
                log::warn!("extract_surface: unknown surface type '{}' (id=#{}), skipping", type_name, surface_id);
                None
            }
        }
    }

    /// Extract a PLANE surface.
    fn extract_plane(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (origin, normal, u_dir) = self.resolve_axis2(axis2_id)?;
        let v_dir = normal.cross(&u_dir);
        Some(Surface::Plane(Plane { origin, u_dir, v_dir, normal }))
    }

    /// Extract a CYLINDRICAL_SURFACE.
    fn extract_cylinder(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (origin, axis, u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        Some(Surface::Cylinder(CylinderSurface::new_with_frame(origin, axis, radius, u_dir)))
    }

    /// Extract a SPHERICAL_SURFACE.
    fn extract_sphere(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, _axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        Some(Surface::Sphere(SphereSurface::new(center, radius)))
    }

    /// Extract a CONICAL_SURFACE.
    /// STEP stores the semi-angle in DEGREES; our ConeSurface expects RADIANS.
    /// A negative semi-angle means the apex is in the OPPOSITE direction of the axis.
    /// We handle this by flipping the axis and using the absolute semi-angle.
    ///
    /// Special case: radius=0. In STEP, a CONICAL_SURFACE with radius=0 means
    /// the apex is at the origin point. The cone expands outward from there.
    /// For our ConeSurface, we need to compute the effective base radius and
    /// height from the boundary edges, since the STEP definition gives us a
    /// cone that starts at a point.
    fn extract_cone(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (origin, axis, u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        // STEP ISO-10303-42 specifies CONICAL_SURFACE semi_angle as a
        // plane_angle_measure, whose unit is determined by the HEADER's
        // GLOBAL_UNIT_ASSIGNED_CONTEXT. Most industrial STEP files use
        // DEGREE (via CONVERSION_BASED_UNIT), while some use RADIAN
        // (SI_UNIT $ .RADIAN.). We detect the unit from HEADER and
        // convert accordingly.
        //
        // However, many real-world STEP files are inconsistent: they
        // declare RADIAN in the HEADER but provide semi_angle in degrees
        // (e.g. 45.0 instead of 0.7854). To handle this, we use a
        // heuristic: if the raw value exceeds π/2 (the maximum valid
        // half-angle for a cone in radians), it must be in degrees.
        let half_angle_raw = self.find_float_param(entity, 1)?;
        let use_degrees = self.angle_in_degrees || half_angle_raw.abs() > std::f64::consts::FRAC_PI_2;
        let half_angle_rad = if use_degrees {
            half_angle_raw.abs().to_radians()
        } else {
            // Already in radians — use as-is
            half_angle_raw.abs()
        };
        // Negative semi-angle: apex is opposite to axis direction → flip axis
        let (axis, u_dir) = if half_angle_raw < 0.0 {
            let flipped_axis = Direction3d::new(-axis.x, -axis.y, -axis.z).unwrap_or(axis);
            let flipped_u_dir = Direction3d::new(-u_dir.x, -u_dir.y, -u_dir.z).unwrap_or(u_dir);
            (flipped_axis, flipped_u_dir)
        } else {
            (axis, u_dir)
        };

        if radius.abs() < 1e-10 && half_angle_rad > 1e-10 {
            // Radius=0 cone: apex is at origin. The cone expands outward.
            // Use the new_expanding constructor which models radius = v * tan(half_angle).
            Some(Surface::Cone(ConeSurface::new_expanding(
                origin, axis, half_angle_rad, u_dir,
            )))
        } else {
            Some(Surface::Cone(ConeSurface::new_with_frame(origin, axis, radius, half_angle_rad, u_dir)))
        }
    }

    /// Extract a TOROIDAL_SURFACE.
    fn extract_torus(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, axis, u_dir) = self.resolve_axis2(axis2_id)?;
        let major_radius = self.find_float_param(entity, 0)?;
        let minor_radius = self.find_float_param(entity, 1)?;
        Some(Surface::Torus(TorusSurface::new_with_frame(center, axis, major_radius, minor_radius, u_dir)))
    }

    /// Extract a SURFACE_OF_REVOLUTION.
    /// Format: #N = SURFACE_OF_REVOLUTION('', #profile_curve, #axis2_placement);
    fn extract_revolution(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Find the profile curve (2nd param, index 1)
        let profile_id = self.find_curve_ref(entity, 1)?;
        let profile = self.resolve_curve(profile_id, 0)?;

        // Find the axis placement (3rd param, index 2)
        let axis2_id = self.find_param_ref(entity, 2)?;
        let (origin, axis, _u_dir) = self.resolve_axis2(axis2_id)?;

        Some(Surface::Revolution(RevolutionSurface {
            profile,
            axis,
            origin,
        }))
    }

    /// Extract a SURFACE_OF_LINEAR_EXTRUSION.
    /// Format: #N = SURFACE_OF_LINEAR_EXTRUSION('', #profile_curve, #direction_or_vector);
    /// The 3rd param can be a DIRECTION or a VECTOR(#direction, magnitude).
    fn extract_extrusion(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Find the profile curve
        let profile_id = self.find_curve_ref(entity, 1)?;
        let profile = self.resolve_curve(profile_id, 0)?;

        // Find the extrusion direction (3rd param, index 2)
        // Can be a DIRECTION or a VECTOR(#direction, magnitude)
        let dir_id = self.find_param_ref(entity, 2)?;
        let direction = if let Some(dir_entity) = self.step.find_entity(dir_id) {
            if dir_entity.type_name == "DIRECTION" {
                self.resolve_direction(dir_id)?
            } else if dir_entity.type_name == "VECTOR" {
                // VECTOR('', #direction, magnitude) — extract direction from it
                let inner_dir_id = self.find_direction_from_vector(dir_entity)?;
                self.resolve_direction(inner_dir_id)?
            } else {
                // Try to find a direction within the referenced entity
                self.resolve_direction(dir_id)?
            }
        } else {
            self.resolve_direction(dir_id)?
        };

        Some(Surface::Extrusion(ExtrusionSurface {
            profile,
            direction,
        }))
    }

    /// Extract a B_SPLINE_SURFACE_WITH_KNOTS.
    /// Format: #N = B_SPLINE_SURFACE_WITH_KNOTS(degree_u, degree_v,
    ///   ((cp_list_row1), (cp_list_row2), ...), .UNSPECIFIED., .F., .F., .F.,
    ///   knot_count_u, knot_count_v, (knots_u), (knots_v), .UNSPECIFIED.);
    fn extract_bspline_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // For complex entities, find the B_SPLINE_SURFACE sub-entity for control points
        // and B_SPLINE_SURFACE_WITH_KNOTS sub-entity for knot vectors
        // and RATIONAL_B_SPLINE_SURFACE sub-entity for weights
        let bspline_sub = entity.find_sub_entity("B_SPLINE_SURFACE");
        let knots_sub = entity.find_sub_entity("B_SPLINE_SURFACE_WITH_KNOTS");
        let rational_sub = entity.find_sub_entity("RATIONAL_B_SPLINE_SURFACE");

        // Use the B_SPLINE_SURFACE sub-entity if available, otherwise use the entity itself
        let cp_entity = bspline_sub.unwrap_or(entity);
        let knot_entity = knots_sub.unwrap_or(entity);

        // Degree u and v — use find_float_param to handle name prefix
        let u_degree = self.find_float_param(cp_entity, 0).unwrap_or(1.0) as usize;
        let v_degree = self.find_float_param(cp_entity, 1).unwrap_or(1.0) as usize;

        // Control points: search params for a list-of-lists that contains the control points
        let mut control_points: Vec<Vec<Point3d>> = Vec::new();
        for param in &cp_entity.params {
            if let StepValue::List(rows) = param {
                // Check if this is a list of lists (i.e., a control point grid)
                let mut is_cp_grid = false;
                for row in rows {
                    if let StepValue::List(_) = row {
                        is_cp_grid = true;
                        break;
                    }
                }
                if !is_cp_grid {
                    continue;
                }

                for row in rows {
                    if let StepValue::List(cols) = row {
                        let mut row_pts = Vec::new();
                        for col in cols {
                            if let StepValue::List(coords) = col {
                                // Inline [x, y, z]
                                let x = coords.get(0).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                                let y = coords.get(1).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                                row_pts.push(Point3d::new(x, y, z));
                            } else if let Some(ref_id) = self.get_ref(col) {
                                // Reference to CARTESIAN_POINT
                                if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                                    row_pts.push(pt);
                                }
                            }
                        }
                        if !row_pts.is_empty() {
                            control_points.push(row_pts);
                        }
                    }
                }
                // Only process the first list-of-lists found (the control point grid)
                break;
            }
        }

        if control_points.is_empty() {
            return None;
        }

        let n_u = control_points.len();
        let n_v = control_points[0].len();

        // Extract weights from RATIONAL_B_SPLINE_SURFACE sub-entity if present
        let weights = if let Some(rational_ent) = rational_sub {
            self.extract_rational_weights(rational_ent, n_u, n_v)
        } else {
            vec![vec![1.0; n_v]; n_u]
        };

        // Find knot vectors — use B_SPLINE_SURFACE_WITH_KNOTS sub-entity if available
        let (u_knots, v_knots) = self.extract_bspline_knots(knot_entity, n_u, n_v, u_degree, v_degree);

        // Detect if the NURBS surface is closed in u and/or v direction.
        // A surface is closed when the first and last rows (or columns) of
        // control points coincide within tolerance. This is critical for
        // correct UV polygon normalization during triangulation — a closed
        // NURBS surface needs its UV boundary wrapped around the seam.
        let closure_tol = 1e-6;
        let u_closed = n_u > 2 && control_points.first().zip(control_points.last())
            .map(|(first_row, last_row)| {
                first_row.iter().zip(last_row.iter())
                    .all(|(p_first, p_last)| {
                        (p_first.x - p_last.x).abs() < closure_tol
                            && (p_first.y - p_last.y).abs() < closure_tol
                            && (p_first.z - p_last.z).abs() < closure_tol
                    })
            })
            .unwrap_or(false);
        let v_closed = n_v > 2 && control_points.iter()
            .all(|row| {
                row.first().zip(row.last())
                    .map(|(p_first, p_last)| {
                        (p_first.x - p_last.x).abs() < closure_tol
                            && (p_first.y - p_last.y).abs() < closure_tol
                            && (p_first.z - p_last.z).abs() < closure_tol
                    })
                    .unwrap_or(false)
            });

        Some(Surface::Nurbs(NurbsSurface {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
            u_closed,
            v_closed,
        }))
    }

    /// Extract weight matrix from a RATIONAL_B_SPLINE_SURFACE sub-entity.
    /// Format: RATIONAL_B_SPLINE_SURFACE(((w11,w12,...),(w21,w22,...),...))
    fn extract_rational_weights(&self, entity: &crate::schema::StepEntity, n_u: usize, n_v: usize) -> Vec<Vec<f64>> {
        for param in &entity.params {
            if let StepValue::List(rows) = param {
                // Check if this is a list of lists of floats (weight matrix)
                let mut is_weight_matrix = false;
                for row in rows {
                    if let StepValue::List(inner) = row {
                        // Check if the inner list contains floats/numbers
                        if !inner.is_empty() && inner.iter().any(|v| matches!(v, StepValue::Float(_) | StepValue::Integer(_))) {
                            is_weight_matrix = true;
                            break;
                        }
                    }
                }

                if is_weight_matrix {
                    let mut weights: Vec<Vec<f64>> = Vec::new();
                    for row in rows {
                        if let StepValue::List(inner) = row {
                            let row_weights: Vec<f64> = inner.iter()
                                .filter_map(|v| self.get_float(v))
                                .collect();
                            if !row_weights.is_empty() {
                                weights.push(row_weights);
                            }
                        }
                    }

                    // Validate dimensions match control points
                    if weights.len() == n_u && weights.iter().all(|r| r.len() == n_v) {
                        return weights;
                    }
                    // If dimensions don't match but we have weights, try to use what we have
                    if !weights.is_empty() {
                        // Resize to match control point dimensions
                        let mut result = vec![vec![1.0; n_v]; n_u];
                        for (i, row) in weights.iter().enumerate() {
                            if i >= n_u { break; }
                            for (j, &w) in row.iter().enumerate() {
                                if j >= n_v { break; }
                                result[i][j] = w;
                            }
                        }
                        return result;
                    }
                }
            }
        }

        // Default to unit weights
        vec![vec![1.0; n_v]; n_u]
    }

    /// Extract knot vectors from a B_SPLINE_SURFACE_WITH_KNOTS entity.
    /// Format: B_SPLINE_SURFACE_WITH_KNOTS(u_mults, v_mults, u_knots, v_knots, knot_type)
    /// The knot values are distinct knots and the multiplicities tell how many times each is repeated.
    /// The full knot vector is: for each distinct knot value, repeat it by its multiplicity.
    fn extract_bspline_knots(
        &self,
        entity: &crate::schema::StepEntity,
        n_u: usize,
        n_v: usize,
        u_degree: usize,
        v_degree: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let expected_u_knots = n_u + u_degree + 1;
        let expected_v_knots = n_v + v_degree + 1;

        // Strategy 1: Scan for consecutive (multiplicity, knot_values) pairs.
        //
        // In B_SPLINE_SURFACE_WITH_KNOTS, the parameter layout is:
        //   (name, u_deg, v_deg, ((control_points),...), surface_form, u_closed, v_closed, self_intersect,
        //    (u_multiplicities), (v_multiplicities), (u_knot_values), (v_knot_values), knot_type)
        //
        // The control point list (a list-of-lists) comes before the multiplicity/knot lists.
        // We scan consecutive pairs of top-level params where:
        //   - The first is a list of positive integers (multiplicities)
        //   - The second is a list of non-decreasing floats (knot values)
        //   - Both have the same length (number of distinct knot values)
        //
        // This approach mirrors extract_curve_knots() and is robust against
        // the control-point list offsetting indices.
        {
            let params = &entity.params;
            // Collect all consecutive (list, list) pairs
            let mut mult_knot_pairs: Vec<(Vec<usize>, Vec<f64>)> = Vec::new();
            for i in 0..params.len().saturating_sub(1) {
                if let (StepValue::List(mult_items), StepValue::List(val_items)) = (&params[i], &params[i + 1]) {
                    // Check if the first list looks like integer multiplicities
                    let multiplicities: Vec<usize> = mult_items.iter()
                        .filter_map(|v| self.get_float(v).map(|f| f as usize))
                        .collect();
                    // Check if the second list looks like knot values (non-decreasing floats)
                    let knot_values: Vec<f64> = val_items.iter()
                        .filter_map(|v| self.get_float(v))
                        .collect();

                    // A valid multiplicity list has all positive integers
                    let mult_valid = !multiplicities.is_empty()
                        && multiplicities.len() == mult_items.len()
                        && multiplicities.iter().all(|&m| m > 0);
                    // A valid knot value list is non-decreasing
                    let knot_valid = !knot_values.is_empty()
                        && knot_values.len() == val_items.len()
                        && knot_values.windows(2).all(|w| w[0] <= w[1] + 1e-10);
                    // Both must have the same length (one multiplicity per distinct knot value)
                    let matching_len = multiplicities.len() == knot_values.len();

                    if mult_valid && knot_valid && matching_len {
                        // Reject if "knot values" look like multiplicities:
                        // if all values are positive integers, this is likely
                        // a pair of two multiplicity lists, not (mults, knots).
                        // A valid knot vector must have at least 2 distinct values
                        // (a surface with range [3,3] is degenerate).
                        let all_int_like = knot_values.iter().all(|v| v > &0.0 && (v - v.round()).abs() < 1e-10);
                        let has_distinct_values = knot_values.first().map_or(false, |first| {
                            knot_values.last().map_or(false, |last| (last - first).abs() > 1e-10)
                        });
                        if all_int_like && !has_distinct_values {
                            continue; // This pair is two multiplicity lists, not (mults, knots)
                        }
                        mult_knot_pairs.push((multiplicities, knot_values));
                    }
                }
            }

            // We expect exactly 2 such pairs: (u_mults, u_knots) and (v_mults, v_knots)
            if mult_knot_pairs.len() >= 2 {
                // Try all pair combinations — the first pair should be U, second should be V
                for i in 0..mult_knot_pairs.len() {
                    let (ref u_mults, ref u_vals) = mult_knot_pairs[i];
                    let u_expanded = expand_knot_vector(u_mults, u_vals);
                    if u_expanded.len() != expected_u_knots { continue; }

                    for j in 0..mult_knot_pairs.len() {
                        if j == i { continue; }
                        let (ref v_mults, ref v_vals) = mult_knot_pairs[j];
                        let v_expanded = expand_knot_vector(v_mults, v_vals);
                        if v_expanded.len() != expected_v_knots { continue; }

                        log::debug!(
                            "extract_bspline_knots: found valid pair i={} j={}: u_mults={:?} u_vals={:?} v_mults={:?} v_vals={:?}",
                            i, j, u_mults, u_vals, v_mults, v_vals
                        );
                        return (u_expanded, v_expanded);
                    }
                }
            }

            // If we found exactly 1 pair, maybe U and V share the same knots
            if mult_knot_pairs.len() == 1 {
                let (ref mults, ref vals) = mult_knot_pairs[0];
                let expanded = expand_knot_vector(mults, vals);
                if expanded.len() == expected_u_knots && expected_u_knots == expected_v_knots {
                    return (expanded.clone(), expanded);
                }
            }
        }

        // Strategy 2: Heuristic approach — collect all numeric lists and try to distinguish
        // multiplicities from knot values based on their properties.
        let mut numeric_lists: Vec<Vec<f64>> = Vec::new();

        for param in entity.params.iter() {
            if let StepValue::List(items) = param {
                // Skip lists that contain sub-lists (control point grids)
                if items.iter().any(|v| matches!(v, StepValue::List(_))) {
                    continue;
                }
                // Skip lists that contain entity references (control point references)
                if items.iter().any(|v| matches!(v, StepValue::Ref(_))) {
                    continue;
                }

                let floats: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();

                if floats.len() >= 2 && floats.iter().all(|f| f.is_finite()) {
                    numeric_lists.push(floats);
                }
            }
        }

        // Separate into multiplicities and knot values based on value properties.
        // CRITICAL: A list that looks like integer multiplicities (positive, sum matches
        // expected knot count) should NOT be added to knot_value_lists, even if it
        // happens to be non-decreasing. The multiplicity values (3,3) were previously
        // incorrectly matched as knot values, producing [3,3,3,3,3,3] instead of [0,0,0,1,1,1].
        let max_mult = (n_u + n_v + u_degree + v_degree + 10).max(20);
        let mut mult_lists: Vec<Vec<usize>> = Vec::new();
        let mut knot_value_lists: Vec<Vec<f64>> = Vec::new();

        // Identify multiplicity lists: all positive integers with correct sum
        for floats in &numeric_lists {
            let ints: Vec<usize> = floats.iter().map(|f| *f as usize).collect();
            // Check if all values are positive integers (no fractional parts)
            let all_positive_ints = floats.iter().all(|f| *f > 0.0 && (*f - f.round()).abs() < 1e-10 && *f as usize <= max_mult);
            if all_positive_ints {
                let sum: usize = ints.iter().sum();
                if sum == expected_u_knots || sum == expected_v_knots {
                    mult_lists.push(ints);
                    continue; // Don't add this to knot_value_lists
                }
            }
        }

        // Identify knot value lists: non-decreasing, and NOT already identified as multiplicities
        let mult_floats: Vec<Vec<f64>> = mult_lists.iter().map(|ints| ints.iter().map(|&i| i as f64).collect()).collect();
        for floats in &numeric_lists {
            if floats.windows(2).all(|w| w[0] <= w[1] + 1e-10) {
                // Check this isn't already a multiplicity list
                let is_mult = mult_floats.iter().any(|ml| {
                    ml.len() == floats.len() && ml.iter().zip(floats.iter()).all(|(a, b)| (a - b).abs() < 1e-10)
                });
                if !is_mult {
                    knot_value_lists.push(floats.clone());
                }
            }
        }

        // Try to pair multiplicities with knot values
        if mult_lists.len() >= 2 && knot_value_lists.len() >= 2 {
            // Try all pairings of mult_lists with knot_value_lists
            for (mi, m) in mult_lists.iter().enumerate() {
                for (ki, k) in knot_value_lists.iter().enumerate() {
                    if m.len() == k.len() {
                        let expanded = expand_knot_vector(m, k);
                        if expanded.len() == expected_u_knots {
                            for (mi2, m2) in mult_lists.iter().enumerate() {
                                if mi2 == mi { continue; }
                                for (ki2, k2) in knot_value_lists.iter().enumerate() {
                                    if ki2 == ki { continue; }
                                    if m2.len() == k2.len() {
                                        let expanded2 = expand_knot_vector(m2, k2);
                                        if expanded2.len() == expected_v_knots {
                                            log::debug!(
                                                "extract_bspline_knots: Strategy 2 found valid pair: u_mults={:?} u_vals={:?} v_mults={:?} v_vals={:?}",
                                                m, k, m2, k2
                                            );
                                            return (expanded, expanded2);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Strategy 3: Try using numeric lists directly (without multiplicity expansion)
        if numeric_lists.len() >= 2 {
            if let Some(k) = numeric_lists.iter().find(|l| l.len() == expected_u_knots) {
                if let Some(k2) = numeric_lists.iter().find(|l| l.len() == expected_v_knots && l.as_ptr() != k.as_ptr()) {
                    return (k.clone(), k2.clone());
                }
            }
        }

        // Strategy 4: Generate uniform knot vectors as fallback
        log::warn!(
            "extract_bspline_knots: all strategies failed for entity #{} (n_u={}, n_v={}, u_deg={}, v_deg={}), using uniform knot vectors",
            entity.id, n_u, n_v, u_degree, v_degree
        );
        let u_n = expected_u_knots;
        let v_n = expected_v_knots;
        let u_knots = (0..u_n).map(|i| i as f64 / (u_n - 1).max(1) as f64).collect();
        let v_knots = (0..v_n).map(|i| i as f64 / (v_n - 1).max(1) as f64).collect();
        (u_knots, v_knots)
    }

    /// Extract a RECTANGULAR_TRIMMED_SURFACE (wrapper around another surface with explicit parameter bounds).
    /// Format: RECTANGULAR_TRIMMED_SURFACE(#basis_surface, u1, u2, v1, v2, usense, vsense)
    ///
    /// The trim bounds define the valid parameter domain [u1,u2] × [v1,v2].
    /// For triangulation purposes, the basis surface geometry is unchanged —
    /// the actual face shape is defined by its boundary edges. However, the
    /// trim parameters are important for UV projection and PCURVE matching.
    ///
    /// We extract the basis surface and log the trim bounds. Future work may
    /// convert trimmed surfaces to NURBS approximations that respect the bounds.
    fn extract_trimmed_surface(&self, entity: &crate::schema::StepEntity, depth: usize) -> Option<Surface> {
        if depth > 20 {
            return None;
        }
        // RECTANGULAR_TRIMMED_SURFACE(#basis_surface, u1, u2, v1, v2, .T., .T.)
        // The first param may be a name string; find the first entity reference
        let basis_id = self.find_param_ref(entity, 0)
            .or_else(|| self.find_param_ref(entity, 1))?;
        let surface = self.extract_surface(basis_id, depth + 1)?;

        // Parse trim parameters for logging and future use
        let mut u1: Option<f64> = None;
        let mut u2: Option<f64> = None;
        let mut v1: Option<f64> = None;
        let mut v2: Option<f64> = None;
        let mut found_floats = 0;
        for param in entity.params.iter().skip(1) {
            if let Some(val) = self.get_float(param) {
                match found_floats {
                    0 => u1 = Some(val),
                    1 => u2 = Some(val),
                    2 => v1 = Some(val),
                    3 => v2 = Some(val),
                    _ => {}
                }
                found_floats += 1;
            }
        }

        // Parse sense flags
        let mut usense = true;
        let mut vsense = true;
        for param in &entity.params {
            if let StepValue::Enum(val) = param {
                if val == "T" || val == "F" {
                    if !usense == false && val == "F" {
                        // First F we encounter flips usense
                        usense = false;
                    } else if usense == false && vsense == true && val == "F" {
                        vsense = false;
                    }
                }
            }
        }

        if let (Some(u1v), Some(u2v), Some(v1v), Some(v2v)) = (u1, u2, v1, v2) {
            info!(
                "RECTANGULAR_TRIMMED_SURFACE #{}: basis=#{} u=[{:.4},{:.4}] v=[{:.4},{:.4}] usense={} vsense={}",
                entity.id, basis_id, u1v, u2v, v1v, v2v, usense, vsense
            );

            // If the basis is a NURBS surface, we could adjust its knot vector
            // to respect the trim bounds. For now, we just return the basis surface
            // since boundary edges define the actual face geometry during triangulation.
        }

        Some(surface)
    }

    /// Extract a SWEPT_SURFACE (surface created by sweeping a curve).
    fn extract_swept_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // SWEPT_SURFACE('', #profile_curve, #swept_curve_or_direction)
        // This is a base type for SURFACE_OF_REVOLUTION and SURFACE_OF_LINEAR_EXTRUSION
        // Try to extract the profile curve and create a revolution or extrusion
        let profile_id = self.find_curve_ref(entity, 1)?;

        // Look for the 3rd param — could be a direction or axis placement
        if let Some(param) = entity.params.get(2) {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(dir_entity) = self.step.find_entity(ref_id) {
                    if dir_entity.type_name == "DIRECTION" {
                        // It's a linear extrusion
                        let profile = self.resolve_curve(profile_id, 0)?;
                        let direction = self.resolve_direction(ref_id)?;
                        return Some(Surface::Extrusion(ExtrusionSurface {
                            profile,
                            direction,
                        }));
                    } else if dir_entity.type_name.contains("AXIS2_PLACEMENT") {
                        // It's a revolution
                        let profile = self.resolve_curve(profile_id, 0)?;
                        let (origin, axis, _u_dir) = self.resolve_axis2(ref_id)?;
                        return Some(Surface::Revolution(RevolutionSurface {
                            profile,
                            axis,
                            origin,
                        }));
                    }
                }
            }
        }
        None
    }

    /// Extract an OFFSET_SURFACE with NURBS approximation.
    /// OFFSET_SURFACE('', #basis_surface, offset_distance, .T./.F.)
    ///
    /// Approximation approach:
    /// 1. Sample the basis surface on a grid
    /// 2. At each grid point, compute the surface normal
    /// 3. Offset each point along the normal by the given distance
    /// 4. Create a NURBS surface from the offset grid
    fn extract_offset_surface(&self, entity: &crate::schema::StepEntity, depth: usize) -> Option<Surface> {
        // Find the basis surface reference (2nd param, index 1)
        let basis_id = self.find_param_ref(entity, 1)?;
        let surface = self.extract_surface(basis_id, depth)?;

        // Extract offset distance
        let offset_dist = self.find_float_param(entity, 0).unwrap_or(0.0);

        if offset_dist.abs() < 1e-10 {
            // Zero offset — just return the basis surface
            return Some(surface);
        }

        // Approximate the offset surface using NURBS
        info!("OFFSET_SURFACE #{}: approximating offset={} as NURBS surface", entity.id, offset_dist);
        Some(approximate_offset_surface(&surface, offset_dist))
    }

    /// Find a curve reference from an entity's parameters.
    /// Handles both direct references and indirect references through
    /// entities like DEFINITIONAL_REPRESENTATION, GEOMETRIC_REPRESENTATION_ITEM, etc.
    fn find_curve_ref(&self, entity: &crate::schema::StepEntity, param_index: usize) -> Option<i64> {
        if let Some(param) = entity.params.get(param_index) {
            if let Some(ref_id) = self.get_ref(param) {
                // Direct reference — check if it's a curve entity
                if let Some(curve_entity) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&curve_entity.type_name) {
                        return Some(ref_id);
                    }
                    // Check if it's a SURFACE_CURVE wrapping a curve
                    if curve_entity.type_name == "SURFACE_CURVE" {
                        return self.resolve_3d_curve_ref(ref_id);
                    }
                    // Indirect reference — try to find a curve within this entity
                    return self.find_nested_curve(curve_entity, 0);
                }
            }
        }
        None
    }

    /// Find a curve reference nested inside an entity (e.g., through
    /// DEFINITIONAL_REPRESENTATION, GEOMETRIC_REPRESENTATION_ITEM, etc.)
    fn find_nested_curve(&self, entity: &crate::schema::StepEntity, depth: usize) -> Option<i64> {
        if depth > 20 {
            return None;
        }
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(nested) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&nested.type_name) {
                        return Some(ref_id);
                    }
                    // Check SURFACE_CURVE
                    if nested.type_name == "SURFACE_CURVE" {
                        return self.resolve_3d_curve_ref(ref_id);
                    }
                    // Go one level deeper
                    let deeper = self.find_nested_curve(nested, depth + 1);
                    if deeper.is_some() {
                        return deeper;
                    }
                }
            }
            // Also check lists
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(nested) = self.step.find_entity(ref_id) {
                            if self.is_curve_type(&nested.type_name) {
                                return Some(ref_id);
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Check if a type name is a known curve type.
    fn is_curve_type(&self, type_name: &str) -> bool {
        matches!(
            type_name,
            "LINE" | "CIRCLE" | "ELLIPSE" | "B_SPLINE_CURVE_WITH_KNOTS" |
            "B_SPLINE_CURVE" | "BEZIER_CURVE" | "POLYLINE" | "TRIMMED_CURVE" |
            "COMPOSITE_CURVE" | "COMPOSITE_CURVE_SEGMENT" | "OFFSET_CURVE_3D" |
            "HYPERBOLA" | "PARABOLA" | "RATIONAL_B_SPLINE_CURVE" |
            "SURFACE_CURVE" | "COMPOSITE_CURVE_ON_SURFACE" |
            "BOUNDED_CURVE" | "CURVE_ON_SURFACE" | "PCURVE"
        ) || type_name.contains("B_SPLINE_CURVE") // Handle complex entity types like "BOUNDED_CURVE+B_SPLINE_CURVE_WITH_KNOTS+..."
    }

    /// Find a reference in the entity's parameters at a specific index.
    fn find_param_ref(&self, entity: &crate::schema::StepEntity, index: usize) -> Option<i64> {
        if let Some(param) = entity.params.get(index) {
            self.get_ref(param)
        } else {
            None
        }
    }

    /// Find a reference to an AXIS2_PLACEMENT_3D (or 2D/1D variant) entity in the params list.
    /// Handles the case where the name parameter may or may not be present.
    fn find_axis2_ref(&self, entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "AXIS2_PLACEMENT_3D"
                        || referenced.type_name == "AXIS2_PLACEMENT_2D"
                        || referenced.type_name == "AXIS1_PLACEMENT"
                    {
                        return Some(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Find a float parameter by searching all params, skipping the first `skip` potential matches.
    /// This handles cases where the name parameter may or may not be present.
    fn find_float_param(&self, entity: &crate::schema::StepEntity, skip: usize) -> Option<f64> {
        let mut found = 0;
        for param in &entity.params {
            if let Some(val) = self.get_float(param) {
                if found >= skip {
                    return Some(val);
                }
                found += 1;
            }
        }
        None
    }

    /// Find a DIRECTION reference nested inside a VECTOR entity.
    /// VECTOR(#direction, magnitude) — extract the direction reference.
    fn find_direction_from_vector(&self, vector_entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &vector_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "DIRECTION" {
                        return Some(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Resolve a STEP curve entity to a Curve3d.
    fn resolve_curve(&self, curve_id: i64, depth: usize) -> Option<Curve3d> {
        if depth > 30 {
            warn!("resolve_curve depth limit reached at curve_id=#{} — returning None", curve_id);
            return None;
        }
        let entity = self.step.find_entity(curve_id)?;
        let type_name = entity.type_name.as_str();

        // Handle complex entity types (e.g., "BOUNDED_CURVE+B_SPLINE_CURVE_WITH_KNOTS+RATIONAL_B_SPLINE_CURVE+...")
        if type_name.contains("B_SPLINE_CURVE") {
            return self.resolve_bspline_curve(entity);
        }

        match type_name {
            "LINE" => self.resolve_line_curve(entity),
            "CIRCLE" => self.resolve_circle_curve(entity),
            "ELLIPSE" => self.resolve_ellipse_curve(entity),
            "HYPERBOLA" => self.resolve_hyperbola_curve(entity),
            "PARABOLA" => self.resolve_parabola_curve(entity),
            "B_SPLINE_CURVE_WITH_KNOTS" | "B_SPLINE_CURVE" | "BEZIER_CURVE" |
            "RATIONAL_B_SPLINE_CURVE" => self.resolve_bspline_curve(entity),
            "POLYLINE" => self.resolve_polyline_curve(entity),
            "TRIMMED_CURVE" => self.resolve_trimmed_curve(entity, depth + 1),
            "COMPOSITE_CURVE" | "COMPOSITE_CURVE_ON_SURFACE" => {
                self.resolve_composite_curve(entity, depth + 1)
            }
            "OFFSET_CURVE_3D" => self.resolve_offset_curve_3d(entity, depth + 1),
            "SURFACE_CURVE" => {
                // Unwrap SURFACE_CURVE to get the 3D curve
                if let Some(curve3d_id) = self.resolve_3d_curve_ref(curve_id) {
                    self.resolve_curve(curve3d_id, depth + 1)
                } else {
                    None
                }
            }
            "BOUNDED_CURVE" | "CURVE_ON_SURFACE" | "PCURVE" => {
                // These are abstract/wrapper types — try to find the underlying curve
                self.resolve_nested_curve_entity(entity, depth + 1)
            }
            "COMPOSITE_CURVE_SEGMENT" => {
                // A standalone segment — extract the parent_curve
                self.resolve_composite_curve_segment_curve(entity, depth + 1)
            }
            _ => None,
        }
    }

    /// Resolve a LINE curve entity.
    /// STEP format: `#N = LINE(#point, #direction);` or `#N = LINE('', #point, #direction);`
    /// Also handles VECTOR references: `#N = LINE(#point, #vector);`
    fn resolve_line_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let mut point_ref: Option<i64> = None;
        let mut dir_ref: Option<i64> = None;

        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "CARTESIAN_POINT" && point_ref.is_none() {
                        point_ref = Some(ref_id);
                    } else if referenced.type_name == "DIRECTION" && dir_ref.is_none() {
                        dir_ref = Some(ref_id);
                    } else if referenced.type_name == "VECTOR" && dir_ref.is_none() {
                        // VECTOR(#direction, magnitude) — extract direction from it
                        dir_ref = self.find_direction_from_vector(referenced);
                    }
                }
            }
        }

        let origin = point_ref.and_then(|id| self.resolve_cartesian_point(id))?;
        let direction = dir_ref.and_then(|id| self.resolve_direction(id))?;
        Some(Curve3d::Line(Line::new(origin, direction)))
    }

    /// Resolve a CIRCLE curve entity.
    /// STEP format: `#N = CIRCLE('', #axis2, radius);` or `#N = CIRCLE(#axis2, radius);`
    fn resolve_circle_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, normal, x_axis) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        Some(Curve3d::Circle(Circle {
            center,
            normal,
            radius,
            x_axis,
        }))
    }

    /// Resolve an ELLIPSE curve entity.
    /// STEP format: `#N = ELLIPSE('', #axis2, semi_major, semi_minor);`
    fn resolve_ellipse_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, normal, x_axis) = self.resolve_axis2(axis2_id)?;
        let semi_major = self.find_float_param(entity, 0)?;
        let semi_minor = self.find_float_param(entity, 1)?;
        Some(Curve3d::Ellipse(draper_geometry::Ellipse {
            center,
            normal,
            semi_major,
            semi_minor,
            x_axis,
        }))
    }

    /// Resolve a HYPERBOLA curve entity.
    /// STEP format: `#N = HYPERBOLA('', #axis2, semi_real, semi_imag);`
    ///
    /// The axis2 placement provides:
    ///   - location = center of the hyperbola
    ///   - axis (z) = normal to the plane of the hyperbola
    ///   - ref_direction (x) = transverse axis direction
    ///
    /// semi_real is the semi-axis length along x (a in x²/a² - y²/b² = 1).
    /// semi_imag is the semi-imaginary axis length along y = z × x (b).
    ///
    /// Parametric form:
    ///   P(t) = center + a·cosh(t)·x + b·sinh(t)·(z × x)
    fn resolve_hyperbola_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, normal, x_axis) = self.resolve_axis2(axis2_id)?;
        let semi_real = self.find_float_param(entity, 0)?;
        let semi_imag = self.find_float_param(entity, 1)?;
        Some(Curve3d::Hyperbola(draper_geometry::Hyperbola {
            center,
            normal,
            x_axis,
            semi_real: semi_real.abs(),
            semi_imag: semi_imag.abs(),
        }))
    }

    /// Resolve a PARABOLA curve entity.
    /// STEP format: `#N = PARABOLA('', #axis2, focal_dist);`
    ///
    /// The axis2 placement provides:
    ///   - location = vertex of the parabola
    ///   - axis (z) = normal to the plane of the parabola
    ///   - ref_direction (x) = direction the parabola opens (toward the focus)
    ///
    /// focal_dist is f > 0. Focus is at vertex + f·x.
    ///
    /// Parametric form (parameter t = y-coordinate):
    ///   P(t) = vertex + (t²/(4f))·x + t·(z × x)
    fn resolve_parabola_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (vertex, normal, x_axis) = self.resolve_axis2(axis2_id)?;
        let focal_dist = self.find_float_param(entity, 0)?;
        Some(Curve3d::Parabola(draper_geometry::Parabola {
            vertex,
            normal,
            x_axis,
            focal_dist: focal_dist.abs(),
        }))
    }

    /// Resolve a B_SPLINE_CURVE_WITH_KNOTS entity (or complex entity containing B_SPLINE_CURVE).
    fn resolve_bspline_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // For complex entities, find the B_SPLINE_CURVE sub-entity for control points
        // and B_SPLINE_CURVE_WITH_KNOTS sub-entity for knot vectors
        // and RATIONAL_B_SPLINE_CURVE sub-entity for weights
        let bspline_sub = entity.find_sub_entity("B_SPLINE_CURVE");
        let knots_sub = entity.find_sub_entity("B_SPLINE_CURVE_WITH_KNOTS");
        let rational_sub = entity.find_sub_entity("RATIONAL_B_SPLINE_CURVE");

        // Use the B_SPLINE_CURVE sub-entity if available, otherwise use the entity itself
        let cp_entity = bspline_sub.unwrap_or(entity);
        let knot_entity = knots_sub.unwrap_or(entity);

        // STEP format: B_SPLINE_CURVE_WITH_KNOTS(name, degree, (control_points), form, closed, self_intersect, (multiplicities), (knot_values), knot_type)
        // Or without name: B_SPLINE_CURVE_WITH_KNOTS(degree, (control_points), ...)
        // The degree is the first numeric parameter; the name (if present) is a string.

        // Find the degree: scan params for the first float value (skip string name if present)
        let mut degree = None;
        let mut cp_param_idx = None;
        for (i, param) in cp_entity.params.iter().enumerate() {
            if degree.is_none() {
                if let Some(d) = self.get_float(param) {
                    degree = Some(d as usize);
                }
            } else if cp_param_idx.is_none() {
                // The control points list should be the next parameter after degree
                if let StepValue::List(_) = param {
                    cp_param_idx = Some(i);
                }
            }
        }

        let degree = match degree {
            Some(d) => d,
            None => {
                log::debug!("    resolve_bspline_curve #{}: no degree param found in {} params", entity.id, cp_entity.params.len());
                return None;
            }
        };

        // Control points: find the list parameter after degree
        let mut control_points = Vec::new();
        if let Some(cp_idx) = cp_param_idx {
            if let Some(StepValue::List(items)) = cp_entity.params.get(cp_idx) {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                            control_points.push(pt);
                        }
                    } else if let StepValue::List(coords) = item {
                        let x = coords.get(0).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                        let y = coords.get(1).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                        let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                        control_points.push(Point3d::new(x, y, z));
                    }
                }
            }
        }

        if control_points.is_empty() {
            log::debug!("    resolve_bspline_curve #{}: no control points (degree={}, cp_param_idx={:?}, params count={})",
                entity.id, degree, cp_param_idx, cp_entity.params.len());
            return None;
        }

        // Extract weights from RATIONAL_B_SPLINE_CURVE sub-entity if present
        let weights = if let Some(rational_ent) = rational_sub {
            self.extract_curve_weights(rational_ent, control_points.len())
        } else {
            vec![1.0; control_points.len()]
        };

        // Extract knots from the B_SPLINE_CURVE_WITH_KNOTS sub-entity if available
        let n = control_points.len();
        let knots = self.extract_curve_knots(knot_entity, n, degree);

        Some(Curve3d::Nurbs(NurbsCurve {
            degree,
            control_points,
            weights,
            knots,
        }))
    }

    /// Extract weight list from a RATIONAL_B_SPLINE_CURVE sub-entity.
    /// Format: RATIONAL_B_SPLINE_CURVE(weights_list) or as part of a complex entity.
    fn extract_curve_weights(&self, entity: &crate::schema::StepEntity, n_cp: usize) -> Vec<f64> {
        // Search for a list of floats in the entity params
        for param in &entity.params {
            if let StepValue::List(items) = param {
                let weights: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if weights.len() == n_cp {
                    return weights;
                }
                // If the list has floats but wrong length, try to use what we have
                if !weights.is_empty() && weights.len() >= 2 {
                    let mut result = vec![1.0; n_cp];
                    let len = weights.len().min(n_cp);
                    result[..len].copy_from_slice(&weights[..len]);
                    return result;
                }
            }
        }
        vec![1.0; n_cp]
    }

    /// Extract knot vector from a B_SPLINE_CURVE entity.
    ///
    /// STEP B_SPLINE_CURVE_WITH_KNOTS stores knots in compressed form:
    ///   B_SPLINE_CURVE_WITH_KNOTS((mult1, mult2, ...), (val1, val2, ...), knot_type)
    /// where each knot value `val_i` is repeated `mult_i` times in the actual knot vector.
    ///
    /// For example: ((4,4), (0.0, 45.0), .PIECEWISE_BEZIER_KNOTS.)
    /// expands to: [0, 0, 0, 0, 45, 45, 45, 45]
    fn extract_curve_knots(&self, entity: &crate::schema::StepEntity, n_cp: usize, degree: usize) -> Vec<f64> {
        let expected_knot_count = n_cp + degree + 1;

        // --- Strategy 1: B_SPLINE_CURVE_WITH_KNOTS compressed format ---
        // The entity params are: (multiplicities, distinct_knot_values, knot_type_enum)
        // Try to find two consecutive lists: first = integer multiplicities, second = float knot values
        let params = &entity.params;
        for i in 0..params.len().saturating_sub(1) {
            if let (StepValue::List(mult_items), StepValue::List(val_items)) = (&params[i], &params[i + 1]) {
                // Check if first list looks like integer multiplicities and second like knot values
                let multiplicities: Vec<usize> = mult_items.iter()
                    .filter_map(|v| self.get_float(v).map(|f| f as usize))
                    .collect();
                let knot_values: Vec<f64> = val_items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();

                if !multiplicities.is_empty() && multiplicities.len() == knot_values.len() {
                    // Expand: repeat each knot value by its multiplicity
                    let mut expanded: Vec<f64> = Vec::new();
                    for (&val, &mult) in knot_values.iter().zip(multiplicities.iter()) {
                        for _ in 0..mult {
                            expanded.push(val);
                        }
                    }
                    if expanded.len() == expected_knot_count {
                        return expanded;
                    }
                    // If the expanded length is close but not exact, try it anyway
                    // (some STEP files may have slightly different conventions)
                    if !expanded.is_empty() && expanded.len() >= degree + 2 {
                        return expanded;
                    }
                }
            }
        }

        // --- Strategy 2: Search for a flat knot list among all params ---
        // Some STEP files may store the full expanded knot vector directly
        let mut knot_lists: Vec<Vec<f64>> = Vec::new();
        for param in params {
            if let StepValue::List(items) = param {
                let floats: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if floats.len() >= 2 && floats.iter().all(|f| f.is_finite()) {
                    knot_lists.push(floats);
                }
            }
        }

        // Try to find a knot list matching the expected length
        if let Some(k) = knot_lists.iter().find(|l| l.len() == expected_knot_count) {
            return k.clone();
        }

        // If we found some lists, the longest one might be the knot vector
        if !knot_lists.is_empty() {
            knot_lists.sort_by(|a, b| b.len().cmp(&a.len()));
            let candidate = &knot_lists[0];
            let is_monotonic = candidate.windows(2).all(|w| w[0] <= w[1] + 1e-10);
            if is_monotonic && candidate.len() >= degree + 2 {
                return candidate.clone();
            }
        }

        // Fallback: generate uniform clamped knot vector
        // Clamped: first (degree+1) knots = 0.0, last (degree+1) knots = 1.0
        let mut knots = Vec::with_capacity(expected_knot_count);
        for i in 0..expected_knot_count {
            if i <= degree {
                knots.push(0.0);
            } else if i >= expected_knot_count - degree - 1 {
                knots.push(1.0);
            } else {
                knots.push((i - degree) as f64 / (expected_knot_count - 2 * degree - 1) as f64);
            }
        }
        knots
    }

    /// Resolve a POLYLINE entity — return as a degree-1 NURBS curve
    /// that interpolates all the polyline vertices in order.
    fn resolve_polyline_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // POLYLINE('', (#pt1, #pt2, ...))
        let mut points = Vec::new();
        for param in &entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                            points.push(pt);
                        }
                    }
                }
            }
        }

        if points.len() >= 2 {
            // Create a degree-1 (piecewise linear) NURBS curve through all points.
            // For N points, we need N control points, N weights (=1), and N+2 knots.
            // Knot vector: clamped — first 2 knots = 0.0, last 2 = (N-1), interior knots = 1,2,...,N-2
            let n = points.len();
            let degree = 1;
            let weights = vec![1.0; n];
            let mut knots = Vec::with_capacity(n + degree + 1);
            // Clamped knot vector for degree 1
            for _ in 0..=degree {
                knots.push(0.0);
            }
            for i in 1..n-1 {
                knots.push(i as f64);
            }
            for _ in 0..=degree {
                knots.push((n - 1) as f64);
            }

            Some(Curve3d::Nurbs(NurbsCurve {
                degree,
                control_points: points,
                weights,
                knots,
            }))
        } else {
            None
        }
    }

    /// Resolve a TRIMMED_CURVE entity by extracting the basis curve and
    /// applying trim parameters to set the correct param_range.
    fn resolve_trimmed_curve(&self, entity: &crate::schema::StepEntity, depth: usize) -> Option<Curve3d> {
        // TRIMMED_CURVE(#basis_curve, #trim1, #trim2, .T., .T., .CARTESIAN., .CARTESIAN.)
        // trim1/trim2 can be either parameter values or point references
        
        let basis_id = self.get_ref(entity.params.first()?)?;
        let curve = self.resolve_curve(basis_id, depth + 1)?;
        
        // Try to extract trim parameter values or points
        // The 2nd and 3rd params are the trim specifications
        let mut trim1: Option<f64> = None;
        let mut trim2: Option<f64> = None;
        let mut _trim_point1: Option<Point3d> = None;
        let mut _trim_point2: Option<Point3d> = None;
        
        if entity.params.len() >= 3 {
            // Trim 1
            if let Some(param) = entity.params.get(1) {
                if let Some(val) = self.get_float(param) {
                    trim1 = Some(val);
                } else if let Some(ref_id) = self.get_ref(param) {
                    _trim_point1 = self.resolve_cartesian_point(ref_id);
                }
            }
            // Trim 2
            if let Some(param) = entity.params.get(2) {
                if let Some(val) = self.get_float(param) {
                    trim2 = Some(val);
                } else if let Some(ref_id) = self.get_ref(param) {
                    _trim_point2 = self.resolve_cartesian_point(ref_id);
                }
            }
        }
        
        // If we have parameter values, create a new curve with adjusted param_range
        // For circles/ellipses with angle trims, convert to Arc
        match (&trim1, &trim2, &curve) {
            (Some(t1), Some(t2), Curve3d::Circle(circle)) => {
                // Trim a circle by angles — create an Arc
                return Some(Curve3d::Arc(Arc::new(circle.clone(), *t1, *t2)));
            }
            (Some(t1), Some(t2), _) => {
                // P14: For other curves (Line, Ellipse, Hyperbola, Parabola, NURBS, PCurve),
                // wrap in a Trimmed curve that maps t ∈ [0, 1] to [t1, t2] on the basis curve.
                // This preserves the trim information instead of returning the untrimmed basis.
                return Some(Curve3d::Trimmed {
                    basis: Box::new(curve),
                    start: *t1,
                    end: *t2,
                });
            }
            _ => {}
        }
        
        // If we have trim points but no param values, project them onto the curve
        // This is handled at the Edge level by resolve_edge_curve which uses vertex points.
        
        Some(curve)
    }

    /// Resolve a COMPOSITE_CURVE or COMPOSITE_CURVE_ON_SURFACE entity by
    /// concatenating all segments into a single degree-1 NURBS curve
    /// (polyline approximation of the composite).
    ///
    /// Handles all COMPOSITE_CURVE_SEGMENT types including:
    /// - BOUNDED_CURVE segments (any bounded curve type)
    /// - Transition codes: CONTINUOUS, DISCONTINUOUS, etc.
    /// - same_sense flag (reverses segment direction if false)
    ///
    /// COMPOSITE_CURVE_ON_SURFACE is handled identically — the 3D curve
    /// of each segment is extracted and sampled.
    fn resolve_composite_curve(&self, entity: &crate::schema::StepEntity, depth: usize) -> Option<Curve3d> {
        // COMPOSITE_CURVE('', (#segment1, #segment2, ...), .U.)
        // COMPOSITE_CURVE_ON_SURFACE('', (#segment1, #segment2, ...), .U., #basis_surface)
        //
        // Primary path: return Curve3d::Composite which preserves the analytical
        // structure of each segment. Falls back to degree-1 NURBS polyline if
        // segment resolution fails or if the composite would be degenerate.
        let mut segments: Vec<Curve3d> = Vec::new();
        let mut seg_lengths: Vec<f64> = Vec::new();

        for param in &entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(seg_entity) = self.step.find_entity(ref_id) {
                            // Handle both COMPOSITE_CURVE_SEGMENT and direct curve references
                            let curve = if seg_entity.type_name == "COMPOSITE_CURVE_SEGMENT" {
                                self.resolve_composite_curve_segment_curve(seg_entity, depth + 1)
                            } else if self.is_curve_type(&seg_entity.type_name) {
                                self.resolve_curve(ref_id, depth + 1)
                            } else {
                                None
                            };

                            if let Some(curve) = curve {
                                // Check same_sense flag (3rd param of COMPOSITE_CURVE_SEGMENT)
                                let same_sense = if seg_entity.type_name == "COMPOSITE_CURVE_SEGMENT" {
                                    self.extract_same_sense(seg_entity)
                                } else {
                                    true
                                };

                                // If same_sense is false, wrap in Trimmed with reversed direction
                                let curve = if same_sense {
                                    curve
                                } else {
                                    // Reverse: map t ∈ [0,1] → [param_max, param_min]
                                    let (t_min, t_max) = curve.param_range();
                                    Curve3d::Trimmed {
                                        basis: Box::new(curve),
                                        start: t_max,
                                        end: t_min,
                                    }
                                };

                                // Estimate segment arc length for proportional parameter mapping
                                let (t_min, t_max) = curve.param_range();
                                let seg_len = estimate_curve_length(&curve, t_min, t_max, 16);

                                segments.push(curve);
                                seg_lengths.push(seg_len);
                            }
                        }
                    }
                }
            }
        }

        if segments.is_empty() {
            return None;
        }

        // Single segment — just return it directly
        if segments.len() == 1 {
            return Some(segments.into_iter().next().unwrap());
        }

        // Compute cumulative arc-length fractions for parameter mapping
        let total_len: f64 = seg_lengths.iter().sum();
        if total_len < 1e-15 {
            // Degenerate — all segments have zero length
            return Some(segments.into_iter().next().unwrap());
        }

        let mut cum_lengths: Vec<f64> = Vec::with_capacity(seg_lengths.len());
        let mut cum = 0.0;
        for len in &seg_lengths {
            cum += len / total_len;
            cum_lengths.push(cum.min(1.0)); // Clamp to avoid floating-point drift
        }
        // Ensure last entry is exactly 1.0
        if let Some(last) = cum_lengths.last_mut() {
            *last = 1.0;
        }

        Some(Curve3d::Composite {
            segments,
            cum_lengths,
        })
    }

    /// Extract the parent_curve from a COMPOSITE_CURVE_SEGMENT.
    /// Format: COMPOSITE_CURVE_SEGMENT(transition, #parent_curve, same_sense)
    fn resolve_composite_curve_segment_curve(&self, seg_entity: &crate::schema::StepEntity, depth: usize) -> Option<Curve3d> {
        // The parent_curve is typically the 2nd parameter (index 1)
        // But also try other parameter positions
        for param in &seg_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(curve_entity) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&curve_entity.type_name) {
                        return self.resolve_curve(ref_id, depth + 1);
                    }
                    // Check for BOUNDED_CURVE or other wrappers
                    if curve_entity.type_name == "BOUNDED_CURVE" {
                        return self.resolve_nested_curve_entity(curve_entity, depth + 1);
                    }
                }
            }
        }
        None
    }

    /// Extract the same_sense flag from a COMPOSITE_CURVE_SEGMENT.
    /// Format: COMPOSITE_CURVE_SEGMENT(transition, #parent_curve, same_sense)
    /// same_sense is typically the 3rd parameter — .T. or .F.
    fn extract_same_sense(&self, seg_entity: &crate::schema::StepEntity) -> bool {
        for param in &seg_entity.params {
            if let StepValue::Enum(val) = param {
                if val == "T" { return true; }
                if val == "F" { return false; }
            }
        }
        true // Default to true
    }

    /// Resolve a nested curve entity (BOUNDED_CURVE, CURVE_ON_SURFACE, PCURVE).
    /// These are wrapper types that contain an underlying curve entity.
    fn resolve_nested_curve_entity(&self, entity: &crate::schema::StepEntity, depth: usize) -> Option<Curve3d> {
        if depth > 30 { return None; }
        // Search all params for a curve reference
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(nested) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&nested.type_name) {
                        return self.resolve_curve(ref_id, depth + 1);
                    }
                    // Go deeper
                    let result = self.resolve_nested_curve_entity(nested, depth + 1);
                    if result.is_some() { return result; }
                }
            }
            // Also check lists
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(nested) = self.step.find_entity(ref_id) {
                            if self.is_curve_type(&nested.type_name) {
                                return self.resolve_curve(ref_id, depth + 1);
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Resolve an OFFSET_CURVE_3D entity with NURBS approximation.
    /// Format: OFFSET_CURVE_3D('', #basis_curve, distance, #ref_direction, .F.)
    ///
    /// Approximation approach:
    /// 1. Sample the basis curve at many points
    /// 2. At each point, compute the tangent and the Frenet normal
    /// 3. Offset each point along the normal direction by the given distance
    /// 4. Fit a NURBS curve through the offset points
    fn resolve_offset_curve_3d(&self, entity: &crate::schema::StepEntity, depth: usize) -> Option<Curve3d> {
        // Parse parameters: name (optional), basis_curve_ref, distance, ref_direction, self_intersect
        let mut basis_curve_id: Option<i64> = None;
        let mut offset_dist: f64 = 0.0;
        let mut ref_dir_id: Option<i64> = None;

        let mut found_floats = 0;
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&referenced.type_name) && basis_curve_id.is_none() {
                        basis_curve_id = Some(ref_id);
                    } else if referenced.type_name == "DIRECTION" && ref_dir_id.is_none() {
                        ref_dir_id = Some(ref_id);
                    } else if referenced.type_name == "VECTOR" && ref_dir_id.is_none() {
                        // Extract direction from VECTOR
                        ref_dir_id = self.find_direction_from_vector(referenced);
                    }
                }
            }
            if let Some(val) = self.get_float(param) {
                if found_floats == 0 {
                    offset_dist = val;
                }
                found_floats += 1;
            }
        }

        let basis_curve_id = basis_curve_id?;
        let basis_curve = self.resolve_curve(basis_curve_id, depth + 1)?;

        if offset_dist.abs() < 1e-10 {
            // Zero offset — just return the basis curve
            return Some(basis_curve);
        }

        // Resolve reference direction (if provided)
        let ref_dir = ref_dir_id.and_then(|id| self.resolve_direction(id));

        // Approximate the offset curve using NURBS
        Some(approximate_offset_curve(&basis_curve, offset_dist, ref_dir.as_ref()))
    }

    /// Convert a FaceData (surface + boundary edges) to a mesh by creating a Face
    /// with proper wire/edges and triangulating.
    /// Triangulate a face using the STEP edge discretization cache.
    ///
    /// This is the cache-aware version of `surface_to_mesh`. When multiple faces
    /// share the same STEP EDGE_CURVE entity, the cache ensures they produce
    /// identical 3D boundary points, guaranteeing watertight meshes.
    fn surface_to_mesh_cached(
        &self,
        face_data: &FaceData,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
        edge_cache: &mut EdgeDiscretizationCache,
    ) -> TriangleMesh {
        let surface_type = match &face_data.surface {
            Surface::Plane(_) => "Plane",
            Surface::Cylinder(_) => "Cylinder",
            Surface::Cone(_) => "Cone",
            Surface::Sphere(_) => "Sphere",
            Surface::Torus(_) => "Torus",
            Surface::Revolution(_) => "Revolution",
            Surface::Extrusion(_) => "Extrusion",
            Surface::Nurbs(n) => {
                let n_u = n.control_points.len();
                let n_v = n.control_points.first().map(|r| r.len()).unwrap_or(0);
                &*format!("Nurbs({}x{},deg={}/{})", n_u, n_v, n.u_degree, n.v_degree)
            }
        };

        // Log NURBS face processing for debugging
        if matches!(&face_data.surface, Surface::Nurbs(_)) {
            log::info!(
                "NURBS_FACE_PROC: STEP face #{}, surface_type={}, outer_edges={}, inner_edges={}, forward={}",
                face_data.step_face_id, surface_type,
                face_data.outer_edges.len(), face_data.inner_edges.len(), face_data.forward,
            );
        }

        if face_data.edges.is_empty() {
            // No boundary edges — fall back to bounding-box-based triangulation for planes,
            // or standard triangulation for curved surfaces
            log::warn!("FACE_DIAG: surface={} edges=EMPTY → fallback path", surface_type);
            if let Surface::Plane(ref plane) = face_data.surface {
                return self.triangulate_unbounded_plane(plane, params, bbox);
            }
            
            let wire = Wire::new(vec![]);
            let mut face = Face::new(face_data.surface.clone(), wire);
            face.forward = face_data.forward;
            face.edges = vec![];
            return triangulate_face(&face, params);
        }

        // For planar faces with inner loops (holes), use the dedicated hole-aware path
        if let Surface::Plane(ref plane) = face_data.surface {
            if !face_data.inner_edges.is_empty() {
                return self.triangulate_planar_face_with_holes_cached(
                    plane, &face_data.outer_edges, &face_data.outer_edge_step_ids,
                    &face_data.inner_edges, &face_data.inner_edge_step_ids,
                    face_data.forward, edge_cache,
                );
            }
        }

        // Collect 3D boundary points from edge curves using the cache.
        // When an edge is already in the cache (shared with another face),
        // we reuse the identical 3D points to guarantee watertightness.
        let mut boundary_points = Vec::new();
        let mut inner_boundary_points: Vec<Vec<Point3d>> = Vec::new();

        // Collect UV coordinates for boundary points, computed from PCURVE
        // (if available) or surface.project_point() (as fallback).
        let mut boundary_uvs: Vec<Point2d> = Vec::new();
        let mut inner_boundary_uvs: Vec<Vec<Point2d>> = Vec::new();

        // Whether we have any analytical PCURVE data for this face
        let _has_pcurves = face_data.edge_curves_2d.iter().any(|c| c.is_some());

        // Sample outer edges using the cache
        let mut edges_without_step_id = 0usize;
        let mut edge_debug_info: Vec<String> = Vec::new();
        // Track the last UV from the previous edge to use as initial guess for
        // NURBS chain projection. This ensures UV continuity across edges —
        // without it, project_point() can return different UVs for the same
        // 3D point (e.g., u=0 vs u=2π on periodic surfaces), causing the UV
        // polygon to jump and self-intersect.
        let mut prev_edge_last_uv: Option<Point2d> = None;
        for (edge_idx, edge) in face_data.outer_edges.iter().enumerate() {
            let step_id = face_data.outer_edge_step_ids.get(edge_idx).copied().unwrap_or(0);
            let n_samples = self.edge_sample_count(edge);
            let curve_2d = face_data.edge_curves_2d.get(edge_idx).and_then(|c| c.as_ref());

            if step_id != 0 {
                let (pts, params) = edge_cache.discretize_step_edge(step_id, edge, n_samples);
                // Compute UV for each boundary point using actual parameter values
                let uvs = self.compute_edge_uvs_with_points(&params, &pts, &face_data.surface, curve_2d, prev_edge_last_uv);
                // Track last UV for next edge's initial guess
                if let Some(last_uv) = uvs.last() {
                    prev_edge_last_uv = Some(*last_uv);
                }
                // Diagnostic: log per-edge UV range for NURBS faces
                if matches!(&face_data.surface, Surface::Nurbs(_)) && !uvs.is_empty() {
                    let eu_min = uvs.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                    let eu_max = uvs.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                    let ev_min = uvs.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                    let ev_max = uvs.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                    let has_c2d = curve_2d.is_some();
                    let p0 = pts.first();
                    let p_n = pts.last();
                    log::info!(
                        "EDGE_UV_DIAG: edge_idx={} step_id={} n_pts={} pcurve={} u=[{:.4},{:.4}] v=[{:.4},{:.4}] 3d_first=({:.2},{:.2},{:.2}) 3d_last=({:.2},{:.2},{:.2})",
                        edge_idx, step_id, uvs.len(), has_c2d, eu_min, eu_max, ev_min, ev_max,
                        p0.map(|p| p.x).unwrap_or(0.0), p0.map(|p| p.y).unwrap_or(0.0), p0.map(|p| p.z).unwrap_or(0.0),
                        p_n.map(|p| p.x).unwrap_or(0.0), p_n.map(|p| p.y).unwrap_or(0.0), p_n.map(|p| p.z).unwrap_or(0.0)
                    );
                }
                edge_debug_info.push(format!("step_id={}→{}pts", step_id, pts.len()));
                boundary_points.extend(pts);
                boundary_uvs.extend(uvs);
            } else {
                log::warn!("BREP face: edge has step_id=0 (no cache), surface={:?}, edge_id={}", 
                    std::mem::discriminant(&face_data.surface), edge.id);
                // No STEP ID (e.g., synthetic edge) — sample independently
                let (pts_3d, pts_uv) = self.sample_edge_points_with_uv(edge, &face_data.surface, curve_2d);
                boundary_points.extend(pts_3d);
                boundary_uvs.extend(pts_uv);
                edges_without_step_id += 1;
            }
        }

        // For curved surfaces, also sample inner edges (holes) using the cache
        match &face_data.surface {
            Surface::Plane(_) => {}, // Planes use the dedicated hole-aware path above
            _ => {
                for (loop_idx, inner_edges) in face_data.inner_edges.iter().enumerate() {
                    let mut hole_pts = Vec::new();
                    let mut hole_uvs = Vec::new();
                    let step_ids = face_data.inner_edge_step_ids.get(loop_idx);
                    for (edge_idx, edge) in inner_edges.iter().enumerate() {
                        let step_id = step_ids.and_then(|ids| ids.get(edge_idx).copied()).unwrap_or(0);
                        let n_samples = self.edge_sample_count(edge);
                        // Try to find the curve_2d for this inner edge
                        let curve_2d = self.find_curve_2d_for_edge(edge, face_data);

                        if step_id != 0 {
                            let (pts, params) = edge_cache.discretize_step_edge(step_id, edge, n_samples);
                            let uvs = self.compute_edge_uvs_with_points(&params, &pts, &face_data.surface, curve_2d, None);
                            hole_pts.extend(pts);
                            hole_uvs.extend(uvs);
                        } else {
                            log::warn!("BREP face: inner edge has step_id=0 (no cache), surface={:?}, edge_id={}", 
                                std::mem::discriminant(&face_data.surface), edge.id);
                            let (pts_3d, pts_uv) = self.sample_edge_points_with_uv(edge, &face_data.surface, curve_2d);
                            hole_pts.extend(pts_3d);
                            hole_uvs.extend(pts_uv);
                            edges_without_step_id += 1;
                        }
                    }
                    if !hole_pts.is_empty() {
                        // Deduplicate 3D and UV points together (keep them in sync)
                        let (deduped_pts, deduped_uvs) = deduplicate_points_3d_with_uv(&hole_pts, &hole_uvs, 1e-6);
                        inner_boundary_points.push(deduped_pts);
                        inner_boundary_uvs.push(deduped_uvs);
                    }
                }
            }
        }

        // If outer boundary is empty, try all edges with cache
        if boundary_points.is_empty() {
            for (edge_idx, edge) in face_data.edges.iter().enumerate() {
                let step_id = face_data.edge_step_ids.get(edge_idx).copied().unwrap_or(0);
                let n_samples = self.edge_sample_count(edge);
                let curve_2d = face_data.edge_curves_2d.get(edge_idx).and_then(|c| c.as_ref());

                if step_id != 0 {
                    let (pts, params) = edge_cache.discretize_step_edge(step_id, edge, n_samples);
                    let uvs = self.compute_edge_uvs_with_points(&params, &pts, &face_data.surface, curve_2d, None);
                    boundary_points.extend(pts);
                    boundary_uvs.extend(uvs);
                } else {
                    log::warn!("BREP face: fallback edge has step_id=0 (no cache), surface={:?}, edge_id={}", 
                        std::mem::discriminant(&face_data.surface), edge.id);
                    let (pts_3d, pts_uv) = self.sample_edge_points_with_uv(edge, &face_data.surface, curve_2d);
                    boundary_points.extend(pts_3d);
                    boundary_uvs.extend(pts_uv);
                    edges_without_step_id += 1;
                }
            }
        }

        if edges_without_step_id > 0 {
            log::warn!("surface_to_mesh_cached: {} edges without step_id (bypassing edge cache)", edges_without_step_id);
        }

        // Deduplicate boundary points and their corresponding UVs together
        let before_dedup = boundary_points.len();
        let (boundary_points, boundary_uvs) = if matches!(&face_data.surface, Surface::Nurbs(_)) {
            // For NURBS: skip dedup entirely. UV-aware dedup still removes
            // too many points when edges share 3D curves but have different
            // UV parameterizations. earcutr can handle duplicate vertices.
            (boundary_points, boundary_uvs)
        } else {
            deduplicate_points_3d_with_uv(&boundary_points, &boundary_uvs, 1e-6)
        };
        if before_dedup != boundary_points.len() {
            log::info!(
                "DEDUP: {}→{} points (removed {})",
                before_dedup, boundary_points.len(), before_dedup - boundary_points.len()
            );
        }

        // If we have boundary points, use boundary-aware triangulation
        // Use the UV-aware API when we have UV coordinates (from PCURVE or projection)
        if !boundary_points.is_empty() {
            if !boundary_uvs.is_empty() && boundary_uvs.len() == boundary_points.len() {
                log::info!(
                    "FACE_DIAG: surface={} n_bnd={} n_holes={} → boundary_uv path, edges=[{}]",
                    surface_type, boundary_points.len(), inner_boundary_points.len(),
                    edge_debug_info.join(", ")
                );
                return triangulate_face_with_boundary_and_holes_uv(
                    &face_data.surface,
                    &boundary_points,
                    &boundary_uvs,
                    &inner_boundary_points,
                    &inner_boundary_uvs,
                    face_data.forward,
                    params,
                );
            }
            // Fallback: use the non-UV API when UVs are not available or mismatched
            log::warn!("FACE_DIAG: surface={} n_bnd={} → boundary_no_uv fallback (UVs missing/mismatched)", surface_type, boundary_points.len());
            return draper_mesh::triangulate_face_with_boundary_and_holes(
                &face_data.surface,
                &boundary_points,
                &inner_boundary_points,
                face_data.forward,
                params,
            );
        }

        // Fallback: use the old Face-based path
        log::warn!("FACE_DIAG_CACHED: surface={} → OLD Face-based path (no boundary points!)", surface_type);
        // IMPORTANT: Attach curve_2d data to CoEdges so that triangulate_face
        // can use PCURVE data for accurate UV coordinates on NURBS surfaces.
        let coedges: Vec<CoEdge> = face_data.edges.iter().enumerate().map(|(i, e)| {
            let mut coedge = CoEdge::new(e.id, true);
            coedge.curve_2d = face_data.edge_curves_2d.get(i).and_then(|c| c.clone());
            coedge
        }).collect();
        let wire = Wire::new(coedges);

        let mut face = Face::new(face_data.surface.clone(), wire);
        face.forward = face_data.forward;
        face.edges = face_data.edges.clone();

        triangulate_face(&face, params)
    }

    fn surface_to_mesh(
        &self,
        face_data: &FaceData,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> TriangleMesh {
        if face_data.edges.is_empty() {
            // No boundary edges — fall back to bounding-box-based triangulation for planes,
            // or standard triangulation for curved surfaces
            if let Surface::Plane(ref plane) = face_data.surface {
                return self.triangulate_unbounded_plane(plane, params, bbox);
            }
            
            let wire = Wire::new(vec![]);
            let mut face = Face::new(face_data.surface.clone(), wire);
            face.forward = face_data.forward;
            face.edges = vec![];
            return triangulate_face(&face, params);
        }

        // For planar faces with inner loops (holes), use the dedicated hole-aware path
        if let Surface::Plane(ref plane) = face_data.surface {
            if !face_data.inner_edges.is_empty() {
                return self.triangulate_planar_face_with_holes(
                    plane, &face_data.outer_edges, &face_data.inner_edges, face_data.forward,
                );
            }
        }

        // Collect 3D boundary points from edge curves by sampling each edge.
        // Use adaptive sampling: fewer samples for line edges, more for curved edges.
        //
        // IMPORTANT: For curved surfaces (cylinder, cone, torus, etc.), we must include
        // BOTH outer and inner edge samples. If we only include outer edges, projecting
        // a single circular boundary onto a cylinder gives v_range ≈ 0, which causes
        // the UV trimming algorithm to produce zero triangles. Including inner edges
        // (holes) provides the missing v-range information.
        let mut boundary_points = Vec::new();
        let mut inner_boundary_points: Vec<Vec<Point3d>> = Vec::new();

        for edge in &face_data.outer_edges {
            let n_samples = self.edge_sample_count(edge);
            for i in 0..n_samples {
                let t = i as f64 / (n_samples - 1).max(1) as f64;
                if let Some(p) = edge.point_at(t) {
                    boundary_points.push(p);
                }
            }
        }

        // For curved surfaces, also sample inner edges (holes)
        // These are passed as separate hole polylines for proper trimming
        match &face_data.surface {
            Surface::Plane(_) => {}, // Planes use the dedicated hole-aware path above
            _ => {
                for inner_edges in &face_data.inner_edges {
                    let mut hole_pts = Vec::new();
                    for edge in inner_edges {
                        let n_samples = self.edge_sample_count(edge);
                        for i in 0..n_samples {
                            let t = i as f64 / (n_samples - 1).max(1) as f64;
                            if let Some(p) = edge.point_at(t) {
                                hole_pts.push(p);
                            }
                        }
                    }
                    if !hole_pts.is_empty() {
                        hole_pts = deduplicate_points_3d(&hole_pts, 1e-6);
                        inner_boundary_points.push(hole_pts);
                    }
                }
            }
        }

        // If outer boundary is empty, try all edges
        if boundary_points.is_empty() {
            for edge in &face_data.edges {
                let n_samples = self.edge_sample_count(edge);
                for i in 0..n_samples {
                    let t = i as f64 / (n_samples - 1).max(1) as f64;
                    if let Some(p) = edge.point_at(t) {
                        boundary_points.push(p);
                    }
                }
            }
        }

        // Deduplicate boundary points — critical for ear clipping to work correctly.
        // Without deduplication, shared vertices between edges create zero-area triangles
        // and self-intersecting polygons that break the triangulation.
        boundary_points = deduplicate_points_3d(&boundary_points, 1e-6);

        // If we have boundary points, use boundary-aware triangulation
        if !boundary_points.is_empty() {
            // For curved surfaces with inner edges, pass them as hole polylines
            return draper_mesh::triangulate_face_with_boundary_and_holes(
                &face_data.surface,
                &boundary_points,
                &inner_boundary_points,
                face_data.forward,
                params,
            );
        }

        // Fallback: use the old Face-based path
        // IMPORTANT: Attach curve_2d data to CoEdges so that triangulate_face
        // can use PCURVE data for accurate UV coordinates on NURBS surfaces.
        let coedges: Vec<CoEdge> = face_data.edges.iter().enumerate().map(|(i, e)| {
            let mut coedge = CoEdge::new(e.id, true);
            coedge.curve_2d = face_data.edge_curves_2d.get(i).and_then(|c| c.clone());
            coedge
        }).collect();
        let wire = Wire::new(coedges);

        let mut face = Face::new(face_data.surface.clone(), wire);
        face.forward = face_data.forward;
        face.edges = face_data.edges.clone();

        triangulate_face(&face, params)
    }

    /// Triangulate a planar face with holes using the bridge-edge technique.
    /// This connects each hole to the outer boundary with a pair of coincident edges,
    /// creating a single polygon that can be ear-clipped.
    /// Triangulate a planar face with holes using the edge cache for consistent boundary points.
    fn triangulate_planar_face_with_holes_cached(
        &self,
        plane: &Plane,
        outer_edges: &[TopoEdge],
        outer_step_ids: &[i64],
        inner_loops: &[Vec<TopoEdge>],
        inner_step_ids: &[Vec<i64>],
        forward: bool,
        edge_cache: &mut EdgeDiscretizationCache,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        // Sample outer boundary points using the cache
        let mut outer_points_3d = Vec::new();
        for (edge_idx, edge) in outer_edges.iter().enumerate() {
            let step_id = outer_step_ids.get(edge_idx).copied().unwrap_or(0);
            let n_samples = self.edge_sample_count(edge);
            if step_id != 0 {
                let (pts, _params) = edge_cache.discretize_step_edge(step_id, edge, n_samples);
                outer_points_3d.extend(pts);
            } else {
                outer_points_3d.extend(self.sample_edge_points(edge));
            }
        }
        outer_points_3d = deduplicate_points_3d(&outer_points_3d, 1e-6);
        if outer_points_3d.is_empty() {
            log::warn!("planar_face_with_holes: outer boundary is empty after dedup — {} outer edges", outer_edges.len());
            return mesh;
        }

        // Log inner loop step_ids for diagnostic
        if !inner_loops.is_empty() {
            let mut inner_summary: Vec<String> = Vec::new();
            for (loop_idx, inner_edges) in inner_loops.iter().enumerate() {
                let step_ids = inner_step_ids.get(loop_idx);
                let mut s = format!("loop{}:", loop_idx);
                for (ei, edge) in inner_edges.iter().enumerate() {
                    let sid = step_ids.and_then(|ids| ids.get(ei).copied()).unwrap_or(0);
                    let curve_type = match &edge.curve {
                        Some(Curve3d::Line(_)) => "Line",
                        Some(Curve3d::Circle(_)) => "Circle",
                        Some(Curve3d::Ellipse(_)) => "Ellipse",
                        Some(Curve3d::Arc(_)) => "Arc",
                        Some(Curve3d::Hyperbola(_)) => "Hyperbola",
                        Some(Curve3d::Parabola(_)) => "Parabola",
                        Some(Curve3d::Nurbs(_)) => "Nurbs",
                        Some(Curve3d::PCurve { .. }) => "PCurve",
                        Some(Curve3d::Trimmed { .. }) => "Trimmed",
                        Some(Curve3d::Composite { .. }) => "Composite",
                        None => "None",
                    };
                    s.push_str(&format!(" #{}={}({})", sid, curve_type, self.edge_sample_count(edge)));
                }
                inner_summary.push(s);
            }
            log::info!("PLANAR_HOLES_DIAG: {} inner loops: {}", inner_loops.len(), inner_summary.join(", "));
        }

        // NOTE: We intentionally do NOT snap boundary points onto the plane.
        // Snapping was previously done to eliminate numerical drift from edge
        // curve sampling, but it causes boundary vertices on shared edges to
        // have DIFFERENT 3D positions between adjacent faces (e.g., a planar
        // cap face and a cylinder side face share a circular edge — the cap
        // face would snap circle points to the cap plane, the cylinder face
        // wouldn't — creating gaps). The boundary points come from the
        // EdgeDiscretizationCache which ensures shared edges produce identical 3D points.
        // Snapping breaks this watertightness guarantee.

        // Log diagnostic info for complex planar faces
        if !inner_loops.is_empty() {
            log::info!("planar_face_with_holes: {} outer pts, {} holes, {} outer edges",
                outer_points_3d.len(), inner_loops.len(), outer_edges.len());
        }

        // Project all points onto the plane's 2D coordinate system
        let project = |p: &Point3d| -> Point2d {
            let dx = p.x - plane.origin.x;
            let dy = p.y - plane.origin.y;
            let dz = p.z - plane.origin.z;
            Point2d::new(
                dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
                dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
            )
        };

        let outer_2d: Vec<Point2d> = outer_points_3d.iter().map(|p| project(p)).collect();

        // Sample inner loop (hole) points using the cache
        let mut hole_points_3d: Vec<Vec<Point3d>> = Vec::new();
        let mut hole_points_2d: Vec<Vec<Point2d>> = Vec::new();
        for (loop_idx, inner_edges) in inner_loops.iter().enumerate() {
            let mut hp3d = Vec::new();
            let step_ids = inner_step_ids.get(loop_idx);
            for (edge_idx, edge) in inner_edges.iter().enumerate() {
                let step_id = step_ids.and_then(|ids| ids.get(edge_idx).copied()).unwrap_or(0);
                let n_samples = self.edge_sample_count(edge);
                if step_id != 0 {
                    let (pts, _params) = edge_cache.discretize_step_edge(step_id, edge, n_samples);
                    hp3d.extend(pts);
                } else {
                    hp3d.extend(self.sample_edge_points(edge));
                }
            }
            if !hp3d.is_empty() {
                hp3d = deduplicate_points_3d(&hp3d, 1e-6);
                // No snap_to_plane — see comment above about watertightness
                let hp2d: Vec<Point2d> = hp3d.iter().map(|p| project(p)).collect();
                hole_points_3d.push(hp3d);
                hole_points_2d.push(hp2d);
            }
        }

        // Same triangulation logic as the non-cached version
        if hole_points_2d.is_empty() {
            let is_convex = is_convex_polygon_2d(&outer_2d);
            if is_convex && outer_points_3d.len() >= 3 {
                for p in &outer_points_3d { mesh.add_vertex(*p); }
                let n = outer_points_3d.len() as u32;
                for i in 1..n - 1 {
                    if forward { mesh.add_triangle(0, i, i + 1); }
                    else { mesh.add_triangle(0, i + 1, i); }
                }
            } else {
                let triangles = ear_clip(&outer_2d);
                for p in &outer_points_3d { mesh.add_vertex(*p); }
                for tri in &triangles {
                    if forward { mesh.add_triangle(tri[0], tri[1], tri[2]); }
                    else { mesh.add_triangle(tri[0], tri[2], tri[1]); }
                }
            }
        } else {
            // Use earcutr (mapbox/earcut algorithm) which natively handles holes.
            // The previous bridge-edge + ear-clip approach failed for circular bolt holes
            // because ear-clipping could produce triangles that span across the thin
            // bridge-edge passage. earcutr uses a different strategy that connects
            // holes to the outer boundary using optimal Z-order curves.
            if let Some(m) = earcutr_triangulate_planar_converter(
                &outer_2d, &outer_points_3d, &hole_points_2d, &hole_points_3d, forward, plane,
            ) {
                return m;
            }

            // Fallback to bridge-edge + ear-clip if earcutr fails
            log::warn!("earcutr failed for planar face with holes (cached), falling back to bridge-edge ear-clip");
            let (merged_2d, merged_3d) = merge_holes_into_polygon(
                &outer_2d, &outer_points_3d, &hole_points_2d, &hole_points_3d,
            );
            let triangles = ear_clip(&merged_2d);

            let filtered_triangles: Vec<[u32; 3]> = triangles.iter()
                .filter(|tri| {
                    let a = merged_2d[tri[0] as usize];
                    let b = merged_2d[tri[1] as usize];
                    let c = merged_2d[tri[2] as usize];
                    let centroid_u = (a.u + b.u + c.u) / 3.0;
                    let centroid_v = (a.v + b.v + c.v) / 3.0;
                    let centroid = Point2d::new(centroid_u, centroid_v);
                    for hole in &hole_points_2d {
                        if point_in_polygon_2d_converter(&centroid, hole) {
                            return false;
                        }
                    }
                    point_in_polygon_2d_converter(&centroid, &outer_2d)
                })
                .cloned()
                .collect();

            for p in &merged_3d { mesh.add_vertex(*p); }
            for tri in &filtered_triangles {
                if forward { mesh.add_triangle(tri[0], tri[1], tri[2]); }
                else { mesh.add_triangle(tri[0], tri[2], tri[1]); }
            }
        }

        let normal = if forward { plane.normal } else {
            Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
        };
        mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);
        mesh
    }

    fn triangulate_planar_face_with_holes(
        &self,
        plane: &Plane,
        outer_edges: &[TopoEdge],
        inner_loops: &[Vec<TopoEdge>],
        forward: bool,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        // Sample outer boundary points
        let outer_points_3d = self.sample_edges(outer_edges);
        if outer_points_3d.is_empty() {
            return mesh;
        }

        // Project all points onto the plane's 2D coordinate system
        let project = |p: &Point3d| -> Point2d {
            let dx = p.x - plane.origin.x;
            let dy = p.y - plane.origin.y;
            let dz = p.z - plane.origin.z;
            Point2d::new(
                dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
                dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
            )
        };

        let outer_2d: Vec<Point2d> = outer_points_3d.iter().map(|p| project(p)).collect();

        // Sample inner loop (hole) points
        let mut hole_points_3d: Vec<Vec<Point3d>> = Vec::new();
        let mut hole_points_2d: Vec<Vec<Point2d>> = Vec::new();
        for inner_edges in inner_loops {
            let pts_3d = self.sample_edges(inner_edges);
            if pts_3d.is_empty() { continue; }
            let pts_2d: Vec<Point2d> = pts_3d.iter().map(|p| project(p)).collect();
            hole_points_3d.push(pts_3d);
            hole_points_2d.push(pts_2d);
        }

        // Use earcutr (mapbox/earcut algorithm) which natively handles holes.
        // The bridge-edge + ear-clip approach fails for circular bolt holes.
        if let Some(m) = earcutr_triangulate_planar_converter(
            &outer_2d, &outer_points_3d, &hole_points_2d, &hole_points_3d, forward, plane,
        ) {
            return m;
        }

        // Fallback to bridge-edge + ear-clip if earcutr fails
        log::warn!("earcutr failed for planar face with holes (non-cached), falling back to bridge-edge ear-clip");
        let (merged_2d, merged_3d) = merge_holes_into_polygon(&outer_2d, &outer_points_3d, &hole_points_2d, &hole_points_3d);
        let triangles = ear_clip(&merged_2d);
        let filtered_triangles: Vec<[u32; 3]> = triangles.iter()
            .filter(|tri| {
                let a = merged_2d[tri[0] as usize];
                let b = merged_2d[tri[1] as usize];
                let c = merged_2d[tri[2] as usize];
                let centroid_u = (a.u + b.u + c.u) / 3.0;
                let centroid_v = (a.v + b.v + c.v) / 3.0;
                let centroid = Point2d::new(centroid_u, centroid_v);
                for hole in &hole_points_2d {
                    if point_in_polygon_2d_converter(&centroid, hole) {
                        return false;
                    }
                }
                point_in_polygon_2d_converter(&centroid, &outer_2d)
            })
            .cloned()
            .collect();
        for p in &merged_3d {
            mesh.add_vertex(*p);
        }
        for tri in &filtered_triangles {
            if forward {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            } else {
                mesh.add_triangle(tri[0], tri[2], tri[1]);
            }
        }

        mesh
    }

    /// Sample points from a list of edges at uniform parameter intervals.
    /// Determine the number of samples to take from an edge based on its curve type.
    /// Lines need only 2 samples (start and end), while circles/NURBS need more for curvature.
    ///
    /// On WASM, we use fewer samples to reduce the number of expensive project_point
    /// calls during UV-space triangulation of curved surfaces.
    fn edge_sample_count(&self, edge: &TopoEdge) -> usize {
        // Use same sample counts on all platforms for consistent results.
        // Keeping these moderate to avoid excessive boundary points that
        // slow down triangulation without improving quality.
        match &edge.curve {
            Some(Curve3d::Line(_)) => 2,
            Some(Curve3d::Circle(_)) => 24,
            Some(Curve3d::Ellipse(_)) => 24,
            Some(Curve3d::Arc(_)) => 16,
            Some(Curve3d::Hyperbola(_)) => 24,
            Some(Curve3d::Parabola(_)) => 24,
            Some(Curve3d::Nurbs(_)) => 32,
            Some(Curve3d::PCurve { .. }) => 32,
            Some(Curve3d::Trimmed { .. }) => 32,
            Some(Curve3d::Composite { .. }) => 32,
            None => 2,
        }
    }

    fn sample_edges(&self, edges: &[TopoEdge]) -> Vec<Point3d> {
        let mut points = Vec::new();
        for edge in edges {
            points.extend(self.sample_edge_points(edge));
        }

        // Remove near-duplicate consecutive points
        points = deduplicate_points_3d(&points, 1e-6);

        points
    }

    /// Sample points from a single edge curve.
    fn sample_edge_points(&self, edge: &TopoEdge) -> Vec<Point3d> {
        let n_samples = self.edge_sample_count(edge);
        let mut pts = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = i as f64 / (n_samples - 1).max(1) as f64;
            if let Some(p) = edge.point_at(t) {
                pts.push(p);
            }
        }
        pts
    }

    /// Sample 3D points and their UV coordinates from an edge curve.
    ///
    /// Uses the analytical PCURVE (`curve_2d`) if available for accurate UV,
    /// otherwise falls back to `surface.project_point()`.
    fn sample_edge_points_with_uv(
        &self,
        edge: &TopoEdge,
        surface: &Surface,
        curve_2d: Option<&Curve2d>,
    ) -> (Vec<Point3d>, Vec<Point2d>) {
        let n_samples = self.edge_sample_count(edge);
        let mut pts_3d = Vec::with_capacity(n_samples);
        let mut pts_uv = Vec::with_capacity(n_samples);

        if let Some(c2d) = curve_2d {
            let (t_min, t_max) = c2d.param_range();
            let (e_min, e_max) = edge.param_range;
            let (pmin, pmax) = if e_min <= e_max { (e_min, e_max) } else { (e_max, e_min) };
            let e_range = pmax - pmin;

            for i in 0..n_samples {
                let t_frac = i as f64 / (n_samples - 1).max(1) as f64;
                // Map fraction to edge parameter, then to curve_2d parameter
                let _edge_t = pmin + t_frac * e_range;
                if let Some(p) = edge.point_at(t_frac) {
                    let curve_t = t_min + t_frac * (t_max - t_min);
                    let uv = c2d.point_at(curve_t);
                    pts_3d.push(p);
                    pts_uv.push(uv);
                }
            }
        } else {
            // No PCURVE — sample 3D points and project to surface
            // For NURBS, use adaptive strategy: chain Newton for small UV ranges,
            // independent project_point for large UV ranges, brute-force fallback.
            if let Surface::Nurbs(ref nurbs) = surface {
                let (u_min, u_max) = nurbs.u_range();
                let (v_min, v_max) = nurbs.v_range();
                let u_range = u_max - u_min;
                let v_range = v_max - v_min;
                let use_chain_newton = u_range < 10.0 && v_range < 10.0;
                let mut prev_u = (u_min + u_max) * 0.5;
                let mut prev_v = (v_min + v_max) * 0.5;
                for i in 0..n_samples {
                    let t = i as f64 / (n_samples - 1).max(1) as f64;
                    if let Some(p) = edge.point_at(t) {
                        let (u, v) = if use_chain_newton && i > 0 {
                            draper_mesh::reproject_nurbs_point(nurbs, &p, prev_u, prev_v)
                        } else {
                            surface.project_point(&p)
                        };
                        // Verify convergence
                        let pp = surface.point_at(u, v);
                        let err = p.distance_to(&pp);
                        let (final_u, final_v) = if err > 1e-4 {
                            let grid_size = draper_mesh::adaptive_grid_size(u_range, v_range);
                            let (ub, vb) = draper_mesh::brute_force_project_point(nurbs, &p, grid_size);
                            let bf_p = surface.point_at(ub, vb);
                            let bf_err = p.distance_to(&bf_p);
                            if bf_err < err { (ub, vb) } else { (u, v) }
                        } else {
                            (u, v)
                        };
                        pts_3d.push(p);
                        pts_uv.push(Point2d::new(final_u, final_v));
                        prev_u = final_u;
                        prev_v = final_v;
                    }
                }
            } else {
                // Non-NURBS: fast analytic projection
                for i in 0..n_samples {
                    let t = i as f64 / (n_samples - 1).max(1) as f64;
                    if let Some(p) = edge.point_at(t) {
                        let (u, v) = surface.project_point(&p);
                        pts_3d.push(p);
                        pts_uv.push(Point2d::new(u, v));
                    }
                }
            }
        }

        // Apply deterministic rounding to 3D points for consistent deduplication
        for p in &mut pts_3d {
            *p = deterministic_round_point(*p);
        }

        (pts_3d, pts_uv)
    }

    /// Compute UV coordinates for a set of already-discretized edge points.
    ///
    /// Uses the analytical PCURVE (`curve_2d`) if available for accurate UV,
    /// otherwise falls back to `surface.project_point()`.
    ///
    /// # Arguments
    /// * `params` — Normalized parameter values [0, 1] for each point, as returned
    ///   by `EdgeDiscretizationCache::discretize_step_edge()`. These are the ACTUAL parameter positions
    ///   on the edge curve — NOT uniform index fractions — and must be used for
    ///   correct PCURVE evaluation.
    /// * `points_3d` — The 3D points corresponding to the params. Used for
    ///   the no-PCURVE fallback (surface.project_point).
    /// * `surface` — The surface of the face (for fallback projection).
    /// * `curve_2d` — Optional analytical PCURVE in UV space.
    fn compute_edge_uvs_with_points(
        &self,
        params: &[f64],
        points_3d: &[Point3d],
        surface: &Surface,
        curve_2d: Option<&Curve2d>,
        initial_uv: Option<Point2d>,
    ) -> Vec<Point2d> {
        if let Some(c2d) = curve_2d {
            let (c2d_t_min, c2d_t_max) = c2d.param_range();

            // Map each normalized parameter [0,1] to the curve_2d parameter range
            // and evaluate the PCURVE to get UV coordinates.
            //
            // IMPORTANT: The params are the ACTUAL parameter positions from
            // adaptive discretization (not uniform index fractions). Using
            // uniform fractions after adaptive refinement was the root cause
            // of incorrect NURBS face triangulation — the midpoints inserted
            // by adaptive refinement would shift all subsequent point indices,
            // causing wrong UV coordinates for the entire edge.
            params.iter().map(|&t_norm| {
                let curve_t = c2d_t_min + t_norm * (c2d_t_max - c2d_t_min);
                c2d.point_at(curve_t)
            }).collect()
        } else {
            // No PCURVE — project 3D points to surface.
            // For NURBS surfaces, we use the same adaptive strategy as
            // EdgeDiscretizationCache::compute_uvs():
            // - Small UV ranges (< 10 units): chain Newton-Raphson (fast, reliable)
            // - Large UV ranges (>= 10 units): independent project_point() per point
            //   (deterministic — same 3D point → same UV regardless of traversal order)
            // - Brute-force fallback if projection error is too large
            if let Surface::Nurbs(ref nurbs) = surface {
                let (u_min, u_max) = nurbs.u_range();
                let (v_min, v_max) = nurbs.v_range();
                let u_range = u_max - u_min;
                let v_range = v_max - v_min;
                let use_chain_newton = u_range < 10.0 && v_range < 10.0;

                let mut uvs = Vec::with_capacity(points_3d.len());
                // Use the previous edge's last UV as the initial guess for the
                // first point. This ensures UV continuity across edges — without
                // it, project_point() can return a different UV for the same 3D
                // point (e.g., u=0 vs u=2π on periodic surfaces), causing the UV
                // polygon to jump and self-intersect.
                let (mut prev_u, mut prev_v) = if let Some(iu) = initial_uv {
                    (iu.u, iu.v)
                } else {
                    ((u_min + u_max) * 0.5, (v_min + v_max) * 0.5)
                };
                let mut newton_failures = 0usize;

                for (i, p) in points_3d.iter().enumerate() {
                    let (u, v) = if use_chain_newton && (i > 0 || initial_uv.is_some()) {
                        // Chain Newton for small UV ranges — fast and reliable.
                        // Also use chain Newton for the first point if we have
                        // an initial UV guess from the previous edge — this
                        // ensures UV continuity across edges.
                        draper_mesh::reproject_nurbs_point(nurbs, p, prev_u, prev_v)
                    } else {
                        // Independent project_point() for large UV ranges or
                        // first point without initial guess
                        surface.project_point(p)
                    };

                    // Validate: check that the projected UV maps back close to the target
                    let proj_p = surface.point_at(u, v);
                    let err = p.distance_to(&proj_p);

                    if err > 1e-4 {
                        // Projection failed — try brute-force grid search
                        let grid_size = draper_mesh::adaptive_grid_size(u_range, v_range);
                        let (ub, vb) = draper_mesh::brute_force_project_point(nurbs, p, grid_size);
                        let bf_p = surface.point_at(ub, vb);
                        let bf_err = p.distance_to(&bf_p);

                        if bf_err < err {
                            uvs.push(Point2d::new(ub, vb));
                            prev_u = ub;
                            prev_v = vb;
                        } else {
                            uvs.push(Point2d::new(u, v));
                            prev_u = u;
                            prev_v = v;
                            log::error!(
                                "NURBS projection failed in converter: point={:?}, error={:.2e}, brute_force_error={:.2e}",
                                p, err, bf_err
                            );
                        }
                        newton_failures += 1;
                    } else {
                        uvs.push(Point2d::new(u, v));
                        prev_u = u;
                        prev_v = v;
                    }
                }

                if newton_failures > 0 {
                    log::warn!(
                        "NURBS UV (converter): {}/{} projections failed (u_range={:.1}, v_range={:.1})",
                        newton_failures, points_3d.len(), u_range, v_range,
                    );
                }
                uvs
            } else {
                // Non-NURBS surfaces: project_point() is fast (analytic formulas)
                points_3d.iter().map(|p| {
                    let (u, v) = surface.project_point(p);
                    Point2d::new(u, v)
                }).collect()
            }
        }
    }

    /// Find the Curve2d for an edge within the FaceData.
    ///
    /// Searches through the `edges` list and returns the corresponding
    /// entry from `edge_curves_2d` if found.
    fn find_curve_2d_for_edge<'b>(
        &self,
        edge: &TopoEdge,
        face_data: &'b FaceData,
    ) -> Option<&'b Curve2d> {
        for (i, e) in face_data.edges.iter().enumerate() {
            if e.id == edge.id {
                return face_data.edge_curves_2d.get(i).and_then(|c| c.as_ref());
            }
        }
        None
    }

    /// Triangulate a PLANE surface with no boundary edges, using the bounding box
    /// to determine a finite extent.
    fn triangulate_unbounded_plane(
        &self,
        plane: &Plane,
        _params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        // Determine the plane's extent from the bounding box
        let (size, center) = if let Some((bmin, bmax)) = bbox {
            let cx = (bmin.x + bmax.x) / 2.0;
            let cy = (bmin.y + bmax.y) / 2.0;
            let cz = (bmin.z + bmax.z) / 2.0;
            let sx = (bmax.x - bmin.x).max(1.0);
            let sy = (bmax.y - bmin.y).max(1.0);
            let sz = (bmax.z - bmin.z).max(1.0);
            let max_dim = sx.max(sy).max(sz);
            (max_dim, Point3d::new(cx, cy, cz))
        } else {
            (100.0, plane.origin)
        };

        // Create a 4x4 grid of points on the plane for better surface representation
        let n = 4;
        let half = size * 0.5;

        for j in 0..=n {
            for i in 0..=n {
                let u = -half + size * i as f64 / n as f64;
                let v = -half + size * j as f64 / n as f64;
                let p = Point3d::new(
                    center.x + u * plane.u_dir.x + v * plane.v_dir.x,
                    center.y + u * plane.u_dir.y + v * plane.v_dir.y,
                    center.z + u * plane.u_dir.z + v * plane.v_dir.z,
                );
                mesh.add_vertex(p);
            }
        }

        let cols = n + 1;
        for j in 0..n {
            for i in 0..n {
                let v0 = (j * cols + i) as u32;
                let v1 = (j * cols + i + 1) as u32;
                let v2 = ((j + 1) * cols + i + 1) as u32;
                let v3 = ((j + 1) * cols + i) as u32;
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            }
        }

        mesh
    }

    /// Resolve an AXIS2_PLACEMENT_3D entity to (origin, z_direction, x_direction).
    ///
    /// STEP AP203/AP214 format:
    ///   `#N = AXIS2_PLACEMENT_3D('', #location, #axis, #ref_direction);`
    /// Some files omit the name parameter:
    ///   `#N = AXIS2_PLACEMENT_3D(#location, #axis, #ref_direction);`
    ///
    /// We handle both cases by scanning parameters for their entity types
    /// instead of assuming fixed positional indices.
    fn resolve_axis2(&self, axis2_id: i64) -> Option<(Point3d, Direction3d, Direction3d)> {
        let entity = self.step.find_entity(axis2_id)?;

        let mut origin: Option<Point3d> = None;
        let mut directions: Vec<Direction3d> = Vec::new();

        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "CARTESIAN_POINT" {
                        if origin.is_none() {
                            origin = self.resolve_cartesian_point(ref_id);
                        }
                    } else if referenced.type_name == "DIRECTION" {
                        if let Some(dir) = self.resolve_direction(ref_id) {
                            directions.push(dir);
                        }
                    }
                }
            }
        }

        let origin = origin?;
        let z_dir = directions.get(0).copied().unwrap_or(Direction3d::Z);
        let x_dir_raw = directions.get(1).copied().unwrap_or_else(|| Self::default_x_dir(&z_dir));

        // STEP spec: ref_direction is approximate — must be projected onto the plane
        // perpendicular to axis (z_dir) before use. This ensures the resulting
        // coordinate frame is orthogonal.
        let dot_xz = x_dir_raw.x * z_dir.x + x_dir_raw.y * z_dir.y + x_dir_raw.z * z_dir.z;
        let x_proj = Direction3d::new(
            x_dir_raw.x - dot_xz * z_dir.x,
            x_dir_raw.y - dot_xz * z_dir.y,
            x_dir_raw.z - dot_xz * z_dir.z,
        );
        let x_dir = x_proj.unwrap_or(x_dir_raw);

        Some((origin, z_dir, x_dir))
    }

    /// Compute a default x direction given a z direction.
    fn default_x_dir(z_dir: &Direction3d) -> Direction3d {
        if z_dir.is_parallel_to(&Direction3d::Y) {
            Direction3d::X
        } else {
            z_dir.cross(&Direction3d::Y)
        }
    }

    /// Resolve a CARTESIAN_POINT entity.
    fn resolve_cartesian_point(&self, point_id: i64) -> Option<Point3d> {
        let entity = self.step.find_entity(point_id)?;
        for param in &entity.params {
            if let StepValue::List(coords) = param {
                let x = self.get_float(coords.get(0)?)?;
                let y = self.get_float(coords.get(1)?)?;
                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                return Some(Point3d::new(x, y, z));
            }
        }
        None
    }

    /// Resolve a DIRECTION entity.
    fn resolve_direction(&self, dir_id: i64) -> Option<Direction3d> {
        let entity = self.step.find_entity(dir_id)?;
        for param in &entity.params {
            if let StepValue::List(coords) = param {
                let x = self.get_float(coords.get(0)?)?;
                let y = self.get_float(coords.get(1)?)?;
                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                return Direction3d::new(x, y, z);
            }
        }
        None
    }

    // Helper methods

    fn get_ref(&self, value: &StepValue) -> Option<i64> {
        match value {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        }
    }

    fn get_float(&self, value: &StepValue) -> Option<f64> {
        match value {
            StepValue::Float(f) => Some(*f),
            StepValue::Integer(i) => Some(*i as f64),
            _ => None,
        }
    }
}

// ============================================================
// Standalone helper functions (used by pre-built index construction)
// ============================================================

/// Standalone version of get_ref for StepValue.
fn get_ref_standalone(value: &StepValue) -> Option<i64> {
    match value {
        StepValue::Ref(id) => Some(*id),
        _ => None,
    }
}

/// Standalone version of get_float for StepValue.
fn get_float_standalone(value: &StepValue) -> Option<f64> {
    match value {
        StepValue::Float(f) => Some(*f),
        StepValue::Integer(i) => Some(*i as f64),
        _ => None,
    }
}

/// Static version of extract_nauo_pd_refs: extract relating and related PD IDs from a NAUO entity.
/// Returns (relating_pd_id, related_pd_id).
fn extract_nauo_pd_refs_static(step: &StepFile, nauo: &crate::schema::StepEntity) -> (Option<i64>, Option<i64>) {
    let mut pd_refs: Vec<i64> = Vec::new();
    for param in &nauo.params {
        if let Some(ref_id) = get_ref_standalone(param) {
            if let Some(entity) = step.find_entity(ref_id) {
                if entity.type_name == "PRODUCT_DEFINITION" {
                    pd_refs.push(ref_id);
                }
            }
        }
    }
    (pd_refs.get(0).copied(), pd_refs.get(1).copied())
}

/// Standalone: extract a [f64;3] direction from a DIRECTION entity referenced by a StepValue param.
fn get_direction_from_param_standalone(step: &StepFile, param: &StepValue) -> Option<[f64; 3]> {
    if let Some(ref_id) = get_ref_standalone(param) {
        if let Some(entity) = step.find_entity(ref_id) {
            if entity.type_name == "DIRECTION" {
                return resolve_direction_coords_standalone(&entity);
            }
        }
    }
    None
}

/// Standalone: extract [x,y,z] coordinates from a DIRECTION entity's param list.
fn resolve_direction_coords_standalone(entity: &crate::schema::StepEntity) -> Option<[f64; 3]> {
    for param in &entity.params {
        if let StepValue::List(coords) = param {
            let x = get_float_standalone(coords.get(0)?)?;
            let y = get_float_standalone(coords.get(1)?)?;
            let z = coords.get(2).and_then(|v| get_float_standalone(v)).unwrap_or(0.0);
            let len = (x*x + y*y + z*z).sqrt();
            if len > 1e-10 {
                return Some([x/len, y/len, z/len]);
            }
        }
    }
    None
}

/// Standalone: extract [x,y,z] coordinates from a CARTESIAN_POINT entity.
fn get_cartesian_point_coords_standalone(entity: &crate::schema::StepEntity) -> Option<[f64; 3]> {
    for param in &entity.params {
        if let StepValue::List(coords) = param {
            let x = get_float_standalone(coords.get(0)?)?;
            let y = get_float_standalone(coords.get(1)?)?;
            let z = coords.get(2).and_then(|v| get_float_standalone(v)).unwrap_or(0.0);
            return Some([x, y, z]);
        }
    }
    None
}

/// Standalone version of resolve_cartesian_point.
fn resolve_cartesian_point_standalone(step: &StepFile, point_id: i64) -> Option<Point3d> {
    let entity = step.find_entity(point_id)?;
    for param in &entity.params {
        if let StepValue::List(coords) = param {
            let x = get_float_standalone(coords.get(0)?)?;
            let y = get_float_standalone(coords.get(1)?)?;
            let z = coords.get(2).and_then(|v| get_float_standalone(v)).unwrap_or(0.0);
            return Some(Point3d::new(x, y, z));
        }
    }
    None
}

/// Standalone version of resolve_direction.
fn resolve_direction_standalone(step: &StepFile, dir_id: i64) -> Option<Direction3d> {
    let entity = step.find_entity(dir_id)?;
    for param in &entity.params {
        if let StepValue::List(coords) = param {
            let x = get_float_standalone(coords.get(0)?)?;
            let y = get_float_standalone(coords.get(1)?)?;
            let z = coords.get(2).and_then(|v| get_float_standalone(v)).unwrap_or(0.0);
            return Direction3d::new(x, y, z);
        }
    }
    None
}

/// Default x direction given a z direction.
fn default_x_dir_standalone(z_dir: &Direction3d) -> Direction3d {
    if z_dir.is_parallel_to(&Direction3d::Y) {
        Direction3d::X
    } else {
        z_dir.cross(&Direction3d::Y)
    }
}

/// Standalone version of resolve_axis2 (used by pre-built index construction).
fn resolve_axis2_standalone(step: &StepFile, axis2_id: i64) -> Option<(Point3d, Direction3d, Direction3d)> {
    let entity = step.find_entity(axis2_id)?;

    let mut origin: Option<Point3d> = None;
    let mut directions: Vec<Direction3d> = Vec::new();

    for param in &entity.params {
        if let Some(ref_id) = get_ref_standalone(param) {
            if let Some(referenced) = step.find_entity(ref_id) {
                if referenced.type_name == "CARTESIAN_POINT" {
                    if origin.is_none() {
                        origin = resolve_cartesian_point_standalone(step, ref_id);
                    }
                } else if referenced.type_name == "DIRECTION" {
                    if let Some(dir) = resolve_direction_standalone(step, ref_id) {
                        directions.push(dir);
                    }
                }
            }
        }
    }

    let origin = origin?;
    let z_dir = directions.get(0).copied().unwrap_or(Direction3d::Z);
    let x_dir_raw = directions.get(1).copied().unwrap_or_else(|| default_x_dir_standalone(&z_dir));

    // STEP spec: ref_direction is approximate — must be projected onto the plane
    // perpendicular to axis (z_dir) before use. This ensures the resulting
    // coordinate frame is orthogonal.
    // x_proj = x_raw - dot(x_raw, z) * z
    let dot_xz = x_dir_raw.x * z_dir.x + x_dir_raw.y * z_dir.y + x_dir_raw.z * z_dir.z;
    let x_proj = Direction3d::new(
        x_dir_raw.x - dot_xz * z_dir.x,
        x_dir_raw.y - dot_xz * z_dir.y,
        x_dir_raw.z - dot_xz * z_dir.z,
    );
    let x_dir = x_proj.unwrap_or(x_dir_raw);

    Some((origin, z_dir, x_dir))
}

/// Project a 3D point onto a line and return the parameter t.
/// For Line: P(t) = origin + t * direction
/// t = dot(point - origin, direction)
fn project_point_on_line(line: &Line, point: &Point3d) -> f64 {
    let dx = point.x - line.origin.x;
    let dy = point.y - line.origin.y;
    let dz = point.z - line.origin.z;
    dx * line.direction.x + dy * line.direction.y + dz * line.direction.z
}

/// Deduplicate a list of 3D points by removing consecutive points that are within
/// the given tolerance. Also removes the last point if it coincides with the first
/// (closing a loop).

/// Triangulate a planar face with holes using the earcutr (mapbox/earcut) algorithm.
///
/// Unlike the bridge-edge + ear-clip approach, earcutr natively handles holes
/// by connecting them to the outer boundary using an optimal Z-order curve strategy.
/// This produces correct triangulations for circular bolt holes and other complex
/// hole shapes where bridge edges fail.
///
/// Returns None if the input is degenerate, in which case the caller should
/// fall back to bridge-edge ear-clip.
fn earcutr_triangulate_planar_converter(
    outer_2d: &[Point2d],
    outer_3d: &[Point3d],
    holes_2d: &[Vec<Point2d>],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    plane: &Plane,
) -> Option<TriangleMesh> {
    if outer_2d.len() < 3 {
        return None;
    }

    let mut mesh = TriangleMesh::new();

    // Build the flat coordinate array for earcutr.
    // Layout: [outer_pts...][hole0_pts...][hole1_pts...]
    // earcutr expects coordinates as [x0,y0, x1,y1, ...] (2D flat)
    let mut coords: Vec<f64> = Vec::with_capacity((outer_2d.len() + holes_2d.iter().map(|h| h.len()).sum::<usize>()) * 2);
    let mut hole_indices: Vec<usize> = Vec::with_capacity(holes_2d.len());

    // Outer boundary points
    for p in outer_2d {
        coords.push(p.u);
        coords.push(p.v);
    }

    // Hole points — each hole starts at the current vertex count.
    // Track which holes are valid (>=3 2D points) so the 3D vertex array
    // stays in sync with the 2D coordinate array.  Skipping a hole in 2D
    // but including it in 3D would shift all subsequent indices and produce
    // wrong triangles (the root cause of Step#87 plane-face bug).
    let mut valid_hole_indices: Vec<usize> = Vec::with_capacity(holes_2d.len());
    for (hi, hole) in holes_2d.iter().enumerate() {
        if hole.len() < 3 {
            continue;
        }
        valid_hole_indices.push(hi);
        hole_indices.push(coords.len() / 2);
        for p in hole {
            coords.push(p.u);
            coords.push(p.v);
        }
    }

    // Run triangulation via draper-mesh adapter (tries earcut int, i_triangle, earcutr)
    let triangle_indices = draper_mesh::earcut_adapter::triangulate_polygon_with_holes(&coords, &hole_indices);

    // Diagnostic: count degenerate triangles produced by adapter
    {
        let n_outer = outer_2d.len();
        let mut degen_count = 0usize;
        let mut total = 0usize;
        for chunk in triangle_indices.chunks(3) {
            if chunk.len() < 3 { break; }
            total += 1;
            let a = chunk[0];
            let b = chunk[1];
            let c = chunk[2];
            if a == b || b == c || a == c {
                degen_count += 1;
                continue;
            }
            // Check zero area in 2D
            let pa = if a < n_outer { &outer_2d[a] } else { &holes_2d.iter().flatten().nth(a - n_outer).unwrap() };
            let pb = if b < n_outer { &outer_2d[b] } else { &holes_2d.iter().flatten().nth(b - n_outer).unwrap() };
            let pc = if c < n_outer { &outer_2d[c] } else { &holes_2d.iter().flatten().nth(c - n_outer).unwrap() };
            let area2 = (pb.u - pa.u) * (pc.v - pa.v) - (pb.v - pa.v) * (pc.u - pa.u);
            if area2.abs() < 1e-15 {
                degen_count += 1;
            }
        }
        if degen_count > 0 || total > 100 {
            log::warn!(
                "EARCUTR_DIAG: outer={}pts, holes={} ({} total hole pts), total_tris={}, degenerate={}, forward={}",
                n_outer, holes_2d.len(), holes_2d.iter().map(|h| h.len()).sum::<usize>(),
                total, degen_count, forward,
            );
            // Print outer bbox
            if !outer_2d.is_empty() {
                let ou_min = outer_2d.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                let ou_max = outer_2d.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                let ov_min = outer_2d.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                let ov_max = outer_2d.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                log::warn!("  outer uv bbox: [{:.4},{:.4}] x [{:.4},{:.4}]",
                    ou_min, ou_max, ov_min, ov_max);
            }
            for (hi, h) in holes_2d.iter().enumerate() {
                if h.is_empty() { continue; }
                let hu_min = h.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                let hu_max = h.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                let hv_min = h.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                let hv_max = h.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                log::warn!("  hole {} uv bbox: [{:.4},{:.4}] x [{:.4},{:.4}] ({} pts)",
                    hi, hu_min, hu_max, hv_min, hv_max, h.len());
            }
        }
    }

    if triangle_indices.is_empty() {
        return None;
    }

    // Build combined 3D vertex array: outer vertices first, then hole vertices.
    // Only include 3D vertices for holes that were included in the 2D coords
    // array (valid_hole_indices).  This keeps the 3D array in sync with the
    // 2D array so that earcutr's index output maps correctly to 3D points.
    let mut all_3d: Vec<Point3d> = outer_3d.to_vec();
    for &hi in &valid_hole_indices {
        all_3d.extend_from_slice(&holes_3d[hi]);
    }

    // Verify that all triangle indices are within bounds
    let n_verts = coords.len() / 2;
    for &idx in &triangle_indices {
        if idx as usize >= n_verts {
            log::warn!("earcutr produced out-of-bounds index {} (max {})", idx, n_verts - 1);
            return None;
        }
    }

    // Add vertices and triangles to the mesh
    for p in &all_3d {
        mesh.add_vertex(*p);
    }

    // DIAGNOSTIC: Check for duplicate 3D positions in the local mesh
    {
        use std::collections::HashMap;
        let mut pos_count: HashMap<[u64; 3], usize> = HashMap::new();
        for p in &all_3d {
            let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
            *pos_count.entry(key).or_insert(0) += 1;
        }
        let dup_positions = pos_count.values().filter(|&&c| c > 1).count();
        if dup_positions > 0 {
            log::warn!(
                "LOCAL_MESH_DUP: all_3d has {} vertices, {} duplicate positions (forward={})",
                all_3d.len(), dup_positions, forward,
            );
            // Show first 3 duplicate groups
            let dups: Vec<_> = pos_count.iter().filter(|(_, &c)| c > 1).take(3).collect();
            for (key, count) in dups {
                let x = f64::from_bits(key[0]);
                let y = f64::from_bits(key[1]);
                let z = f64::from_bits(key[2]);
                log::warn!("  pos=({:.4},{:.4},{:.4}): {} occurrences", x, y, z, count);
            }
        }
    }

    // earcutr produces triangles as [i0, i1, i2, i0, i1, i2, ...]
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 {
            break;
        }
        let a = chunk[0] as u32;
        let b = chunk[1] as u32;
        let c = chunk[2] as u32;

        // Skip degenerate triangles
        if a == b || b == c || a == c {
            continue;
        }

        // Skip position-degenerate triangles (different indices but same 3D
        // position). These occur when the boundary has duplicate positions
        // (e.g., seam points). Adding such triangles creates phantom edges
        // that break watertightness after merge.
        let pa = all_3d[a as usize];
        let pb = all_3d[b as usize];
        let pc = all_3d[c as usize];
        let ab = (pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2) + (pa.z - pb.z).powi(2);
        let bc = (pb.x - pc.x).powi(2) + (pb.y - pc.y).powi(2) + (pb.z - pc.z).powi(2);
        let ac = (pa.x - pc.x).powi(2) + (pa.y - pc.y).powi(2) + (pa.z - pc.z).powi(2);
        if ab < 1e-20 || bc < 1e-20 || ac < 1e-20 {
            continue;
        }

        // Verify vertices are valid
        if (a as usize) < all_3d.len() && (b as usize) < all_3d.len() && (c as usize) < all_3d.len() {
            if forward {
                mesh.add_triangle(a, b, c);
            } else {
                mesh.add_triangle(a, c, b);
            }
        }
    }

    if mesh.triangles.is_empty() {
        return None;
    }

    let normal = if forward { plane.normal } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };
    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);

    Some(mesh)
}

/// Find the best bridge edge between an outer polygon and a hole in 2D.
/// Uses the rightmost-hole-point / closest-visible-outer-point technique.
///
/// For non-convex polygons (like L-shapes), verifies that the bridge edge
/// doesn't cross any polygon edges before accepting it.
fn find_bridge_edge_2d(outer_2d: &[Point2d], hole_2d: &[Point2d]) -> (usize, usize) {
    if hole_2d.is_empty() || outer_2d.is_empty() {
        return (0, 0);
    }
    let mut hole_idx = 0;
    let mut max_u = hole_2d[0].u;
    for (i, p) in hole_2d.iter().enumerate() {
        if p.u > max_u {
            max_u = p.u;
            hole_idx = i;
        }
    }
    let hole_pt = hole_2d[hole_idx];

    // Sort outer polygon vertices by distance to the rightmost hole point (closest first)
    let mut candidates: Vec<(usize, f64)> = outer_2d.iter().enumerate()
        .map(|(i, p)| {
            let dx = p.u - hole_pt.u;
            let dy = p.v - hole_pt.v;
            (i, dx * dx + dy * dy)
        })
        .collect();
    candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    if candidates.is_empty() {
        return (0, hole_idx);
    }

    // Try each candidate — accept the first visible one
    let fallback_idx = candidates[0].0;
    let bridge = candidates.into_iter().find(|(outer_idx, _)| {
        let outer_pt = outer_2d[*outer_idx];
        is_bridge_visible_converter(outer_2d, hole_pt, outer_pt, *outer_idx)
    });

    let outer_idx = bridge.map(|(idx, _)| idx).unwrap_or(fallback_idx);
    (outer_idx, hole_idx)
}

/// Check if a bridge edge from `hole_pt` to `outer_pt` is visible (doesn't cross polygon edges).
fn is_bridge_visible_converter(outer_2d: &[Point2d], hole_pt: Point2d, outer_pt: Point2d, outer_idx: usize) -> bool {
    let n = outer_2d.len();
    for i in 0..n {
        let j = (i + 1) % n;
        if i == outer_idx || j == outer_idx {
            continue;
        }
        if segments_intersect_converter(hole_pt, outer_pt, outer_2d[i], outer_2d[j]) {
            return false;
        }
    }
    // Verify midpoint is inside the polygon
    let mid = Point2d::new((hole_pt.u + outer_pt.u) / 2.0, (hole_pt.v + outer_pt.v) / 2.0);
    if !point_in_polygon_2d_converter(&mid, outer_2d) {
        return false;
    }
    true
}

/// Check if two line segments properly intersect.
fn segments_intersect_converter(p1: Point2d, p2: Point2d, p3: Point2d, p4: Point2d) -> bool {
    let d1 = cross_2d_converter(p3, p4, p1);
    let d2 = cross_2d_converter(p3, p4, p2);
    let d3 = cross_2d_converter(p1, p2, p3);
    let d4 = cross_2d_converter(p1, p2, p4);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    false
}

/// Cross product for 2D segment intersection test.
fn cross_2d_converter(p1: Point2d, p2: Point2d, p3: Point2d) -> f64 {
    (p2.u - p1.u) * (p3.v - p1.v) - (p2.v - p1.v) * (p3.u - p1.u)
}

/// Point-in-polygon test for 2D (ray casting).
fn point_in_polygon_2d_converter(point: &Point2d, polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 3 { return false; }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        if ((polygon[i].v > point.v) != (polygon[j].v > point.v))
            && (point.u < (polygon[j].u - polygon[i].u) * (point.v - polygon[i].v) / (polygon[j].v - polygon[i].v) + polygon[i].u)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Helper: count degenerate triangles in a mesh (zero area or repeated indices)
fn count_degenerate_triangles(mesh: &TriangleMesh) -> usize {
    let mut count = 0;
    for tri in &mesh.triangles {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
            count += 1;
            continue;
        }
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let ex = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let fx = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let cx = ex.1 * fx.2 - ex.2 * fx.1;
        let cy = ex.2 * fx.0 - ex.0 * fx.2;
        let cz = ex.0 * fx.1 - ex.1 * fx.0;
        let area_sq = cx * cx + cy * cy + cz * cz;
        if area_sq < 1e-20 {
            count += 1;
        }
    }
    count
}

/// Helper: count boundary edges (edges appearing in only 1 triangle)
fn count_boundary_edges(mesh: &TriangleMesh) -> usize {
    use std::collections::HashMap;
    let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in &mesh.triangles {
        for k in 0..3 {
            let a = tri[k].min(tri[(k + 1) % 3]);
            let b = tri[k].max(tri[(k + 1) % 3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    edge_count.values().filter(|&&c| c == 1).count()
}

/// Check if a 2D polygon is convex.
fn is_convex_polygon_2d(points: &[Point2d]) -> bool {
    let n = points.len();
    if n < 3 { return false; }
    let mut sign = 0i32;
    for i in 0..n {
        let a = &points[i];
        let b = &points[(i + 1) % n];
        let c = &points[(i + 2) % n];
        let cross = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
        if cross.abs() > 1e-10 {
            let s = if cross > 0.0 { 1 } else { -1 };
            if sign == 0 { sign = s; } else if sign != s { return false; }
        }
    }
    sign != 0
}

/// Reorder edges in a loop so that each edge's start point matches the previous
/// edge's end point.
///
/// Some STEP files list edges in arbitrary order rather than topologically
/// connected order. This causes the boundary point list to be scrambled,
/// breaking dedup (shared endpoints not detected) and triangulation (boundary
/// polygon is self-crossing).
///
/// Algorithm: greedy chain — start with edge[0], then repeatedly find an unused
/// edge whose start_point matches the current edge's end_point (within tolerance).
/// If no match is found, the loop has a gap; remaining edges are appended as-is.
fn reorder_edge_loop(edges: Vec<TopoEdge>, step_ids: Vec<i64>) -> (Vec<TopoEdge>, Vec<i64>) {
    if edges.len() <= 2 {
        return (edges, step_ids);
    }

    // Tolerance for matching vertices: 1e-6 mm (typical STEP precision)
    let tol = 1e-6;
    let tol_sq = tol * tol;

    // Extract start/end points for each edge
    let points: Vec<(Option<Point3d>, Option<Point3d>)> = edges.iter()
        .map(|e| (e.start_point(), e.end_point()))
        .collect();

    // If any edge lacks start/end points, can't reorder
    if points.iter().any(|(s, e)| s.is_none() || e.is_none()) {
        return (edges, step_ids);
    }

    let points: Vec<(Point3d, Point3d)> = points.into_iter()
        .map(|(s, e)| (s.unwrap(), e.unwrap()))
        .collect();

    // Check if the original order is already a connected loop.
    // If so, don't reorder — the greedy chain below might pick a different
    // edge when multiple edges share an endpoint (e.g., cylinder seam),
    // breaking the correct order.
    let n = edges.len();
    let mut already_connected = true;
    for i in 0..n {
        let next_i = (i + 1) % n;
        let end_cur = points[i].1;
        let start_next = points[next_i].0;
        let dx = end_cur.x - start_next.x;
        let dy = end_cur.y - start_next.y;
        let dz = end_cur.z - start_next.z;
        if dx * dx + dy * dy + dz * dz > tol_sq {
            already_connected = false;
            break;
        }
    }
    if already_connected {
        return (edges, step_ids);
    }

    let n = edges.len();
    let mut used = vec![false; n];
    let mut order: Vec<usize> = Vec::with_capacity(n);
    // Track whether each edge in the reordered loop was reversed
    let mut reversed_flags: Vec<bool> = Vec::with_capacity(n);

    // Start with edge 0 (not reversed)
    order.push(0);
    reversed_flags.push(false);
    used[0] = true;

    for _ in 1..n {
        let last_idx = *order.last().unwrap();
        let last_end = if *reversed_flags.last().unwrap() {
            // If the last edge was reversed, its effective end is its original start
            points[last_idx].0
        } else {
            points[last_idx].1
        };

        // Find an unused edge whose start matches last_end (forward match)
        let mut found: Option<(usize, bool)> = None;
        for i in 0..n {
            if used[i] { continue; }
            let start = points[i].0;
            let dx = start.x - last_end.x;
            let dy = start.y - last_end.y;
            let dz = start.z - last_end.z;
            if dx*dx + dy*dy + dz*dz <= tol_sq {
                found = Some((i, false));
                break;
            }
        }

        // If no forward match, try an edge whose END matches last_end
        // (this edge needs to be REVERSED to connect)
        if found.is_none() {
            for i in 0..n {
                if used[i] { continue; }
                let end = points[i].1;
                let dx = end.x - last_end.x;
                let dy = end.y - last_end.y;
                let dz = end.z - last_end.z;
                if dx*dx + dy*dy + dz*dz <= tol_sq {
                    found = Some((i, true));
                    break;
                }
            }
        }

        match found {
            Some((i, reversed)) => {
                order.push(i);
                reversed_flags.push(reversed);
                used[i] = true;
            }
            None => {
                // No connecting edge found — append remaining edges as-is
                for i in 0..n {
                    if !used[i] {
                        order.push(i);
                        reversed_flags.push(false);
                        used[i] = true;
                    }
                }
                break;
            }
        }
    }

    // Build reordered vectors, reversing edges as needed
    let mut reordered_edges = Vec::with_capacity(n);
    let mut reordered_ids = Vec::with_capacity(n);
    let mut reversed_count = 0usize;
    for (pos, &i) in order.iter().enumerate() {
        let reversed = reversed_flags[pos];
        if reversed { reversed_count += 1; }
        let edge = if reversed {
            edges[i].reversed()
        } else {
            edges[i].clone()
        };
        reordered_edges.push(edge);
        if i < step_ids.len() {
            reordered_ids.push(step_ids[i]);
        }
    }
    if reversed_count > 0 {
        log::debug!(
            "REORDER: {} edges, {} reversed to form connected loop",
            n, reversed_count,
        );
    }

    (reordered_edges, reordered_ids)
}

/// (closing a loop). This is essential for ear clipping algorithms which produce
/// degenerate triangles on duplicate vertices.
fn deduplicate_points_3d(points: &[Point3d], _tolerance: f64) -> Vec<Point3d> {
    if points.is_empty() {
        return Vec::new();
    }

    // Use bit-exact VertexKey comparison for deterministic deduplication.
    // After deterministic rounding, shared-edge vertices produce bit-identical
    // f64 values, so bit-exact dedup is correct and avoids the problem where
    // tolerance-based dedup removes different points from different faces
    // (because the preceding edge is different, changing which point gets
    // removed at edge junctions).
    use draper_mesh::mesh::VertexKey;
    let mut unique = vec![points[0]];
    let mut unique_keys = vec![VertexKey::from_point(&points[0])];
    for p in &points[1..] {
        let key = VertexKey::from_point(p);
        if let Some(last_key) = unique_keys.last() {
            if key != *last_key {
                unique.push(*p);
                unique_keys.push(key);
            }
        }
    }
    // Also check last vs first (closed loop) — bit-exact
    if unique.len() > 1 {
        let first_key = unique_keys[0];
        if let Some(last_key) = unique_keys.last() {
            if first_key == *last_key {
                unique.pop();
                unique_keys.pop();
            }
        }
    }
    // NOTE: Bowtie detection (non-adjacent duplicate truncation) removed.
    // It incorrectly truncates legitimate boundaries where the same 3D vertex
    // appears multiple times (e.g., cylinder face seam endpoints). See
    // deduplicate_points_3d_with_uv for full explanation.
    unique
}

/// Deduplicate 3D points while keeping UV coordinates in sync.
///
/// When two consecutive 3D points are within `tolerance` of each other,
/// only the first is kept, and its UV coordinate is preserved.
/// This is essential for the consistent triangulation path where
/// UV coordinates must correspond 1:1 with 3D boundary points.
fn deduplicate_points_3d_with_uv(points: &[Point3d], uvs: &[Point2d], _tolerance: f64) -> (Vec<Point3d>, Vec<Point2d>) {
    if points.is_empty() {
        return (Vec::new(), Vec::new());
    }
    // If UVs don't match points length, just deduplicate 3D points
    if uvs.len() != points.len() {
        log::debug!(
            "DEDUP_FALLBACK_3D: points={} uvs={} (length mismatch)",
            points.len(), uvs.len()
        );
        return (deduplicate_points_3d(points, _tolerance), uvs.to_vec());
    }

    // UV-aware deduplication for NURBS and other parametric surfaces.
    //
    // For analytic surfaces (plane, cylinder), every distinct 3D point maps
    // to a unique UV, so pure 3D dedup is correct. But for NURBS surfaces
    // with degenerate boundaries, multiple distinct UV points can map to the
    // SAME 3D point (e.g., an edge that collapses to a single point on the
    // surface but spans a UV range). Pure 3D dedup would remove these,
    // collapsing the UV polygon and producing non-watertight meshes.
    //
    // Solution: Only consider two consecutive points duplicates if BOTH
    // their 3D positions AND UV coordinates match. Points at the same 3D
    // location but different UVs (degenerate boundary) are preserved.
    //
    // 3D comparison uses tolerance (1e-6) instead of bit-exact, because
    // different EDGE_CURVE entities that share the same geometric vertex
    // may produce slightly different 3D coordinates (FP drift ~1e-13).

    // Compute UV range for relative tolerance
    let u_min = uvs.iter().map(|p| p.u).fold(f64::MAX, f64::min);
    let u_max = uvs.iter().map(|p| p.u).fold(f64::MIN, f64::max);
    let v_min = uvs.iter().map(|p| p.v).fold(f64::MAX, f64::min);
    let v_max = uvs.iter().map(|p| p.v).fold(f64::MIN, f64::max);
    let u_span = (u_max - u_min).max(1e-10);
    let v_span = (v_max - v_min).max(1e-10);
    // Relative UV tolerance: two UVs are "the same" if their relative
    // difference is < 1e-10 of the total UV span. This is effectively
    // bit-exact for normal cases but preserves degenerate-boundary points
    // that span a significant UV range.
    let uv_rel_tol = 1e-10;
    // 3D tolerance for consecutive point dedup (catches FP drift between
    // different EDGE_CURVE entities sharing the same geometric vertex).
    let dist_tol = 1e-6;
    let dist_tol_sq = dist_tol * dist_tol;

    let mut unique_pts = vec![points[0]];
    let mut unique_uvs = vec![uvs[0]];

    for i in 1..points.len() {
        let last_pt = unique_pts.last().unwrap();
        let dx = points[i].x - last_pt.x;
        let dy = points[i].y - last_pt.y;
        let dz = points[i].z - last_pt.z;
        let dist_sq = dx * dx + dy * dy + dz * dz;

        if dist_sq > dist_tol_sq {
            // 3D points differ — always keep
            unique_pts.push(points[i]);
            unique_uvs.push(uvs[i]);
        } else {
            // 3D points are close — check UV too.
            // For degenerate NURBS boundaries, multiple boundary points
            // at the same 3D location have DIFFERENT UVs and must be
            // preserved to maintain a valid UV polygon for triangulation.
            let last_uv = unique_uvs.last().unwrap();
            let du = (uvs[i].u - last_uv.u).abs() / u_span;
            let dv = (uvs[i].v - last_uv.v).abs() / v_span;
            if du > uv_rel_tol || dv > uv_rel_tol {
                // UV differs — this is a degenerate-boundary point, KEEP it
                unique_pts.push(points[i]);
                unique_uvs.push(uvs[i]);
            }
            // else: Both 3D and UV match — true consecutive duplicate, skip
        }
    }
    // Also check last vs first (closed loop) — UV-aware + tolerance 3D
    if unique_pts.len() > 1 {
        let first_pt = unique_pts[0];
        let last_pt = *unique_pts.last().unwrap();
        let dx = last_pt.x - first_pt.x;
        let dy = last_pt.y - first_pt.y;
        let dz = last_pt.z - first_pt.z;
        if dx * dx + dy * dy + dz * dz <= dist_tol_sq {
            let first_uv = unique_uvs[0];
            let last_uv = *unique_uvs.last().unwrap();
            let du = (last_uv.u - first_uv.u).abs() / u_span;
            let dv = (last_uv.v - first_uv.v).abs() / v_span;
            if du <= uv_rel_tol && dv <= uv_rel_tol {
                unique_pts.pop();
                unique_uvs.pop();
            }
        }
    }

    // NOTE: Bowtie detection (non-adjacent duplicate truncation) was REMOVED.
    //
    // The bowtie detection was intended to handle self-intersecting UV polygons
    // where the boundary crosses itself. However, it incorrectly truncates
    // legitimate boundaries where the same 3D vertex appears multiple times
    // (e.g., a cylinder face whose boundary visits the seam endpoint at the
    // top circle start AND the seam edge end — both are the same 3D point
    // with the same UV, but they are NOT a bowtie).
    //
    // For cylinder/cone/torus faces, the boundary legitimately visits shared
    // vertices (seam endpoints) multiple times. Truncating at the first match
    // destroys the boundary topology and produces non-watertight meshes.
    //
    // The consecutive dedup above is sufficient for normal cases. True bowtie
    // detection should be handled at the UV polygon level (in parametric_domain)
    // using proper geometric self-intersection tests, not vertex key matching.

    (unique_pts, unique_uvs)
}

/// Project two 3D points onto a circle and return the angular parameter range (t1, t2).
/// The angles are computed in the circle's local coordinate system.
/// t1 and t2 are in radians and the arc goes from t1 to t2 in the positive direction.
/// For full circles (p1 ≈ p2), returns a full 2π range.
fn project_points_on_circle(circle: &Circle, p1: &Point3d, p2: &Point3d) -> (f64, f64) {
    let y_axis = circle.normal.cross(&circle.x_axis);

    let d1x = p1.x - circle.center.x;
    let d1y = p1.y - circle.center.y;
    let d1z = p1.z - circle.center.z;
    let local1_x = d1x * circle.x_axis.x + d1y * circle.x_axis.y + d1z * circle.x_axis.z;
    let local1_y = d1x * y_axis.x + d1y * y_axis.y + d1z * y_axis.z;
    let t1 = local1_y.atan2(local1_x);

    // Check if p1 and p2 are approximately the same point (full circle)
    let dist_sq = (p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2) + (p2.z - p1.z).powi(2);
    if dist_sq < 1e-10 {
        // Full circle — use the full 2π range starting from t1
        return (t1, t1 + 2.0 * std::f64::consts::PI);
    }

    let d2x = p2.x - circle.center.x;
    let d2y = p2.y - circle.center.y;
    let d2z = p2.z - circle.center.z;
    let local2_x = d2x * circle.x_axis.x + d2y * circle.x_axis.y + d2z * circle.x_axis.z;
    let local2_y = d2x * y_axis.x + d2y * y_axis.y + d2z * y_axis.z;
    let t2 = local2_y.atan2(local2_x);

    // Ensure t2 > t1 (positive direction arc from t1 to t2)
    let mut t2 = t2;
    let mut guard = 0;
    while t2 <= t1 {
        guard += 1;
        if guard > 1000 { break; } // Safety: prevent infinite loop with NaN/Inf
        t2 += 2.0 * std::f64::consts::PI;
    }

    // Use the positive direction arc from t1 to t2 without assuming shorter arc.
    // The STEP file's orientation flag determines direction; we preserve the
    // natural order from p1 to p2 in the positive (counterclockwise) sense.
    (t1, t2)
}

/// Multiply two 4x4 matrices (row-major storage).
fn mat4_mul(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut r = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

/// Compute the inverse of a 4x4 matrix using cofactor expansion.
fn mat4_inverse(m: &[[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let s0 = m[0][0] * m[1][1] - m[1][0] * m[0][1];
    let s1 = m[0][0] * m[1][2] - m[1][0] * m[0][2];
    let s2 = m[0][0] * m[1][3] - m[1][0] * m[0][3];
    let s3 = m[0][1] * m[1][2] - m[1][1] * m[0][2];
    let s4 = m[0][1] * m[1][3] - m[1][1] * m[0][3];
    let s5 = m[0][2] * m[1][3] - m[1][2] * m[0][3];

    let c5 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
    let c4 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
    let c3 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
    let c2 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
    let c1 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
    let c0 = m[2][0] * m[3][1] - m[3][0] * m[2][1];

    let det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    if det.abs() < 1e-12 { return None; }
    let inv_det = 1.0 / det;

    Some([
        [( m[1][1]*c5 - m[1][2]*c4 + m[1][3]*c3) * inv_det,
         (-m[0][1]*c5 + m[0][2]*c4 - m[0][3]*c3) * inv_det,
         ( m[3][1]*s5 - m[3][2]*s4 + m[3][3]*s3) * inv_det,
         (-m[2][1]*s5 + m[2][2]*s4 - m[2][3]*s3) * inv_det],
        [(-m[1][0]*c5 + m[1][2]*c2 - m[1][3]*c1) * inv_det,
         ( m[0][0]*c5 - m[0][2]*c2 + m[0][3]*c1) * inv_det,
         (-m[3][0]*s5 + m[3][2]*s2 - m[3][3]*s1) * inv_det,
         ( m[2][0]*s5 - m[2][2]*s2 + m[2][3]*s1) * inv_det],
        [( m[1][0]*c4 - m[1][1]*c2 + m[1][3]*c0) * inv_det,
         (-m[0][0]*c4 + m[0][1]*c2 - m[0][3]*c0) * inv_det,
         ( m[3][0]*s4 - m[3][1]*s2 + m[3][3]*s0) * inv_det,
         (-m[2][0]*s4 + m[2][1]*s2 - m[2][3]*s0) * inv_det],
        [(-m[1][0]*c3 + m[1][1]*c1 - m[1][2]*c0) * inv_det,
         ( m[0][0]*c3 - m[0][1]*c1 + m[0][2]*c0) * inv_det,
         (-m[3][0]*s3 + m[3][1]*s1 - m[3][2]*s0) * inv_det,
         ( m[2][0]*s3 - m[2][1]*s1 + m[2][2]*s0) * inv_det],
    ])
}

/// Resolve STEP predefined colour names to RGBA values.
/// These follow ISO 10209-2:1993 / STEP draughting predefined colours.
fn resolve_predefined_colour(name: &str) -> [f32; 4] {
    match name.to_lowercase().as_str() {
        "red" => [1.0, 0.0, 0.0, 1.0],
        "green" => [0.0, 1.0, 0.0, 1.0],
        "blue" => [0.0, 0.0, 1.0, 1.0],
        "yellow" => [1.0, 1.0, 0.0, 1.0],
        "magenta" => [1.0, 0.0, 1.0, 1.0],
        "cyan" => [0.0, 1.0, 1.0, 1.0],
        "black" => [0.0, 0.0, 0.0, 1.0],
        "white" => [1.0, 1.0, 1.0, 1.0],
        "brown" => [0.6, 0.3, 0.1, 1.0],
        "orange" => [1.0, 0.65, 0.0, 1.0],
        "pink" => [1.0, 0.75, 0.8, 1.0],
        "purple" => [0.5, 0.0, 0.5, 1.0],
        "grey" | "gray" => [0.5, 0.5, 0.5, 1.0],
        "light grey" | "light gray" => [0.75, 0.75, 0.75, 1.0],
        "dark grey" | "dark gray" => [0.25, 0.25, 0.25, 1.0],
        _ => {
            warn!("Unknown predefined colour: {}", name);
            [0.5, 0.5, 0.5, 1.0] // Default grey
        }
    }
}

/// Expand a knot vector from distinct knot values and multiplicities.
/// For example: mults=[2,2], values=[0.0, 1.0] → [0.0, 0.0, 1.0, 1.0]
fn expand_knot_vector(multiplicities: &[usize], knot_values: &[f64]) -> Vec<f64> {
    if multiplicities.len() != knot_values.len() {
        // Mismatch — fall back to just repeating the values
        let mut result = Vec::new();
        for &v in knot_values {
            result.push(v);
            result.push(v);
        }
        return result;
    }
    let mut result = Vec::new();
    for (i, &mult) in multiplicities.iter().enumerate() {
        for _ in 0..mult {
            result.push(knot_values[i]);
        }
    }
    result
}

// ============================================================
// NURBS Approximation Helpers for Offset Curves/Surfaces
// ============================================================

/// Approximate an offset curve as a NURBS curve.
///
/// Given a basis curve and an offset distance, this samples the basis curve,
/// offsets each sample point along the normal direction, and creates a
/// degree-3 NURBS curve through the offset points.
///
/// # Arguments
/// * `basis_curve` - The curve to offset
/// * `distance` - The offset distance (positive = left of curve, negative = right)
/// * `ref_direction` - Optional reference direction for the offset plane.
///   When provided, the offset normal is computed in the plane containing
///   the tangent and this direction. When None, the Frenet normal is used.
fn approximate_offset_curve(basis_curve: &Curve3d, distance: f64, ref_direction: Option<&Direction3d>) -> Curve3d {
    let n_samples = 64;
    let (t_min, t_max) = basis_curve.param_range();
    let eps = (t_max - t_min) * 1e-7;

    let mut offset_points: Vec<Point3d> = Vec::with_capacity(n_samples);

    for i in 0..n_samples {
        let t = t_min + (t_max - t_min) * i as f64 / (n_samples - 1) as f64;
        let p = basis_curve.point_at(t);

        // Compute tangent using central differences
        let t_lo = (t - eps).max(t_min);
        let t_hi = (t + eps).min(t_max);
        let p_lo = basis_curve.point_at(t_lo);
        let p_hi = basis_curve.point_at(t_hi);

        let tx = p_hi.x - p_lo.x;
        let ty = p_hi.y - p_lo.y;
        let tz = p_hi.z - p_lo.z;
        let t_len = (tx * tx + ty * ty + tz * tz).sqrt();

        if t_len < 1e-15 {
            offset_points.push(p);
            continue;
        }

        let tangent = Vec3d::new(tx / t_len, ty / t_len, tz / t_len);

        // Compute the normal direction for the offset
        let normal = if let Some(ref_dir) = ref_direction {
            // Use the reference direction to define the offset plane
            // Normal = tangent × ref_dir, then normalize
            let n = tangent.cross(&Vec3d::new(ref_dir.x, ref_dir.y, ref_dir.z));
            let n_len = n.length();
            if n_len < 1e-15 {
                // Tangent is parallel to ref_dir — fall back to Frenet
                compute_frenet_normal(&tangent, basis_curve, t, eps)
            } else {
                Direction3d::new(n.x / n_len, n.y / n_len, n.z / n_len).unwrap_or(Direction3d::Z)
            }
        } else {
            // Use Frenet normal (tangent × second_derivative direction)
            compute_frenet_normal(&tangent, basis_curve, t, eps)
        };

        // Offset the point along the normal
        offset_points.push(Point3d::new(
            p.x + distance * normal.x,
            p.y + distance * normal.y,
            p.z + distance * normal.z,
        ));
    }

    // Remove near-duplicate consecutive points
    offset_points = deduplicate_points_3d(&offset_points, 1e-6);

    if offset_points.len() < 2 {
        // Degenerate offset — return basis curve as-is
        return basis_curve.clone();
    }

    // Create a degree-3 NURBS curve through the offset points
    // Using global interpolation with chord-length parameterization
    fit_nurbs_curve_through_points(&offset_points, 3)
        .map(Curve3d::Nurbs)
        .unwrap_or_else(|| {
            // Fallback to degree-1 polyline
            let n = offset_points.len();
            let degree = 1;
            let weights = vec![1.0; n];
            let mut knots = Vec::with_capacity(n + degree + 1);
            for _ in 0..=degree { knots.push(0.0); }
            for i in 1..n-1 { knots.push(i as f64); }
            for _ in 0..=degree { knots.push((n - 1) as f64); }
            Curve3d::Nurbs(NurbsCurve {
                degree,
                control_points: offset_points,
                weights,
                knots,
            })
        })
}

/// Approximate an offset of a 2D curve by sampling, offsetting each point
/// perpendicular to the 2D tangent, and fitting a degree-3 Nurbs2d through
/// the result. Falls back to degree-1 polyline if the fit fails.
///
/// In 2D, the offset normal is simply the tangent rotated 90° CCW
/// (for positive distance) or CW (for negative distance).
fn approximate_offset_curve_2d(basis_curve: &Curve2d, distance: f64) -> Curve2d {
    let n_samples = 64;
    let (t_min, t_max) = basis_curve.param_range();
    let eps = (t_max - t_min) * 1e-7;

    let mut offset_points: Vec<Point2d> = Vec::with_capacity(n_samples);

    for i in 0..n_samples {
        let t = t_min + (t_max - t_min) * i as f64 / (n_samples - 1) as f64;
        let p = basis_curve.point_at(t);

        // Compute tangent using central differences
        let t_lo = (t - eps).max(t_min);
        let t_hi = (t + eps).min(t_max);
        let p_lo = basis_curve.point_at(t_lo);
        let p_hi = basis_curve.point_at(t_hi);

        let du = p_hi.u - p_lo.u;
        let dv = p_hi.v - p_lo.v;
        let t_len = (du * du + dv * dv).sqrt();

        if t_len < 1e-15 {
            offset_points.push(p);
            continue;
        }

        // 2D normal: tangent rotated 90° CCW → (-dv, du) / length
        // For positive distance, offset to the left of the tangent direction.
        let nu = -dv / t_len;
        let nv = du / t_len;

        offset_points.push(Point2d::new(
            p.u + distance * nu,
            p.v + distance * nv,
        ));
    }

    // Remove near-duplicate consecutive points
    offset_points = deduplicate_points_2d(&offset_points, 1e-6);

    if offset_points.len() < 2 {
        // Degenerate offset — return basis curve as-is
        return basis_curve.clone();
    }

    // Fit a degree-3 Nurbs2d through the offset points
    fit_nurbs_curve_through_points_2d(&offset_points, 3)
        .map(Curve2d::Nurbs)
        .unwrap_or_else(|| {
            // Fallback to degree-1 polyline
            let n = offset_points.len();
            let degree = 1;
            let weights = vec![1.0; n];
            let mut knots = Vec::with_capacity(n + degree + 1);
            for _ in 0..=degree { knots.push(0.0); }
            for i in 1..n-1 { knots.push(i as f64); }
            for _ in 0..=degree { knots.push((n - 1) as f64); }
            Curve2d::Nurbs(Nurbs2d {
                degree,
                control_points: offset_points,
                weights,
                knots,
            })
        })
}

/// Estimate the arc length of a curve by sampling n points and summing chord distances.
fn estimate_curve_length(curve: &Curve3d, t_min: f64, t_max: f64, n: usize) -> f64 {
    if n < 2 {
        return 0.0;
    }
    let mut total = 0.0;
    let mut prev = curve.point_at(t_min);
    for i in 1..n {
        let t = t_min + (t_max - t_min) * i as f64 / (n - 1) as f64;
        let p = curve.point_at(t);
        let dx = p.x - prev.x;
        let dy = p.y - prev.y;
        let dz = p.z - prev.z;
        total += (dx * dx + dy * dy + dz * dz).sqrt();
        prev = p;
    }
    total
}

/// Deduplicate consecutive 2D points that are very close together.
fn deduplicate_points_2d(points: &[Point2d], _tolerance: f64) -> Vec<Point2d> {
    if points.is_empty() {
        return Vec::new();
    }
    let mut unique = vec![points[0]];
    for p in &points[1..] {
        let last = unique.last().unwrap();
        let du = p.u - last.u;
        let dv = p.v - last.v;
        if du * du + dv * dv > 1e-12 {
            unique.push(*p);
        }
    }
    // Also check last vs first (closed loop)
    if unique.len() > 1 {
        let first = unique[0];
        let last = *unique.last().unwrap();
        let du = first.u - last.u;
        let dv = first.v - last.v;
        if du * du + dv * dv < 1e-12 {
            unique.pop();
        }
    }
    unique
}

/// Fit a degree-p NURBS curve through 2D points using global interpolation
/// with chord-length parameterization. This is the 2D analog of
/// `fit_nurbs_curve_through_points`.
fn fit_nurbs_curve_through_points_2d(points: &[Point2d], degree: usize) -> Option<Nurbs2d> {
    let n = points.len();
    if n < degree + 1 {
        return None;
    }

    // 1. Chord-length parameterization
    let mut chords = vec![0.0f64; n];
    let mut total_chord = 0.0f64;
    for i in 1..n {
        let du = points[i].u - points[i-1].u;
        let dv = points[i].v - points[i-1].v;
        total_chord += (du * du + dv * dv).sqrt();
        chords[i] = total_chord;
    }

    if total_chord < 1e-15 {
        return None;
    }

    // Normalize to [0, 1]
    let t: Vec<f64> = chords.iter().map(|c| c / total_chord).collect();

    // 2. Compute knot vector using averaging
    let m = n + degree + 1;
    let mut knots = Vec::with_capacity(m);
    for _ in 0..=degree { knots.push(0.0); }
    for j in 1..=(n - degree - 1) {
        let sum: f64 = (1..=degree).map(|k| t[j + k - 1]).sum();
        knots.push(sum / degree as f64);
    }
    for _ in 0..=degree { knots.push(1.0); }

    // 3. Build and solve the linear system for control points
    let mut matrix = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        let basis = compute_bspline_basis_values(&knots, degree, t[i], n);
        for j in 0..n {
            matrix[i][j] = basis[j];
        }
    }

    // Solve for each coordinate (u, v) using Gaussian elimination
    let mut control_points = Vec::with_capacity(n);
    for coord in 0..2 {
        let rhs: Vec<f64> = points.iter().map(|p| if coord == 0 { p.u } else { p.v }).collect();

        if let Some(solution) = solve_linear_system_gauss(&matrix, &rhs) {
            for (i, &val) in solution.iter().enumerate() {
                if coord == 0 {
                    control_points.push(Point2d::new(val, 0.0));
                } else {
                    control_points[i].v = val;
                }
            }
        } else {
            return None;
        }
    }

    Some(Nurbs2d {
        degree,
        control_points,
        weights: vec![1.0; n],
        knots,
    })
}

/// Compute the principal normal (Frenet normal, in the osculating plane,
/// perpendicular to the tangent and pointing toward the center of curvature)
/// using numerical differentiation.
///
/// For a circle, this should point radially inward (toward the center).
fn compute_frenet_normal(tangent: &Vec3d, curve: &Curve3d, t: f64, eps: f64) -> Direction3d {
    let (t_min, t_max) = curve.param_range();
    let eps2 = eps.max((t_max - t_min) * 1e-6);

    // Second derivative approximation
    let p0 = curve.point_at(t - eps2);
    let p1 = curve.point_at(t);
    let p2 = curve.point_at(t + eps2);

    let ddx = (p2.x - 2.0 * p1.x + p0.x) / (eps2 * eps2);
    let ddy = (p2.y - 2.0 * p1.y + p0.y) / (eps2 * eps2);
    let ddz = (p2.z - 2.0 * p1.z + p0.z) / (eps2 * eps2);

    // The principal normal is the component of the second derivative
    // that is perpendicular to the tangent:
    // N = P'' - (P'' · T) * T, then normalize
    let dot = ddx * tangent.x + ddy * tangent.y + ddz * tangent.z;
    let nx = ddx - dot * tangent.x;
    let ny = ddy - dot * tangent.y;
    let nz = ddz - dot * tangent.z;
    let n_len = (nx * nx + ny * ny + nz * nz).sqrt();

    if n_len > 1e-15 {
        Direction3d::new(nx / n_len, ny / n_len, nz / n_len).unwrap_or(Direction3d::Z)
    } else {
        // Principal normal is degenerate (straight line or inflection point)
        // Use an arbitrary normal perpendicular to the tangent
        let arbitrary = if tangent.x.abs() < 0.9 {
            Vec3d::new(1.0, 0.0, 0.0)
        } else {
            Vec3d::new(0.0, 1.0, 0.0)
        };
        let n = tangent.cross(&arbitrary);
        Direction3d::new(n.x, n.y, n.z).unwrap_or(Direction3d::Z)
    }
}

/// Fit a degree-p NURBS curve through a set of 3D points using global
/// curve interpolation with chord-length parameterization.
///
/// This implements the standard algorithm from "The NURBS Book" (Piegl & Tiller):
/// 1. Compute chord-length parameter values for each data point
/// 2. Compute knot vector using the averaging technique
/// 3. Solve the linear system N^T * P = Q for control points
fn fit_nurbs_curve_through_points(points: &[Point3d], degree: usize) -> Option<NurbsCurve> {
    let n = points.len();
    if n < degree + 1 {
        return None;
    }

    // 1. Chord-length parameterization
    let mut chords = vec![0.0f64; n];
    let mut total_chord = 0.0f64;
    for i in 1..n {
        let dx = points[i].x - points[i-1].x;
        let dy = points[i].y - points[i-1].y;
        let dz = points[i].z - points[i-1].z;
        total_chord += (dx * dx + dy * dy + dz * dz).sqrt();
        chords[i] = total_chord;
    }

    if total_chord < 1e-15 {
        return None;
    }

    // Normalize to [0, 1]
    let t: Vec<f64> = chords.iter().map(|c| c / total_chord).collect();

    // 2. Compute knot vector using averaging
    let m = n + degree + 1;
    let mut knots = Vec::with_capacity(m);
    // First degree+1 knots = 0
    for _ in 0..=degree {
        knots.push(0.0);
    }
    // Interior knots: average of degree parameter values
    for j in 1..=(n - degree - 1) {
        let sum: f64 = (1..=degree).map(|k| t[j + k - 1]).sum();
        knots.push(sum / degree as f64);
    }
    // Last degree+1 knots = 1
    for _ in 0..=degree {
        knots.push(1.0);
    }

    // 3. Build and solve the linear system for control points
    // N[i][j] = B-spline basis function N_{j,degree}(t_i)
    let mut matrix = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        let basis = compute_bspline_basis_values(&knots, degree, t[i], n);
        for j in 0..n {
            matrix[i][j] = basis[j];
        }
    }

    // Solve for each coordinate using Gaussian elimination
    let mut control_points = Vec::with_capacity(n);
    for coord in 0..3 {
        let rhs: Vec<f64> = points.iter().map(|p| match coord {
            0 => p.x, 1 => p.y, _ => p.z
        }).collect();

        if let Some(solution) = solve_linear_system_gauss(&matrix, &rhs) {
            for (i, &val) in solution.iter().enumerate() {
                if coord == 0 {
                    control_points.push(Point3d::new(val, 0.0, 0.0));
                } else if coord == 1 {
                    control_points[i].y = val;
                } else {
                    control_points[i].z = val;
                }
            }
        } else {
            return None;
        }
    }

    Some(NurbsCurve {
        degree,
        control_points,
        weights: vec![1.0; n],
        knots,
    })
}

/// Compute all B-spline basis function values N_{i,p}(t) for i=0..n_cp.
/// Uses the Cox-de Boor recursion formula.
fn compute_bspline_basis_values(knots: &[f64], degree: usize, t: f64, n_cp: usize) -> Vec<f64> {
    let n = n_cp;
    let p = degree;

    // Clamp t to valid range
    let t_min = knots[p];
    let t_max = knots[knots.len() - p - 1];
    let t = t.clamp(t_min, t_max);

    // Initialize degree-0 basis functions
    let mut basis = vec![0.0f64; n];
    for i in 0..n {
        if i + 1 < knots.len() {
            if (t >= knots[i] || (i == 0 && t >= knots[0])) &&
               (t < knots[i + 1] || (i == n - 1 && t <= knots[knots.len() - 1])) {
                // Special case for the last span
                if i == n - 1 && t >= knots[i] && t <= knots[i + 1].max(knots[knots.len() - 1]) {
                    basis[i] = 1.0;
                } else if t >= knots[i] && t < knots[i + 1] {
                    basis[i] = 1.0;
                }
            }
        }
    }

    // Build up higher-degree basis functions
    for d in 1..=p {
        let mut new_basis = vec![0.0f64; n];
        for i in 0..n {
            // Left term
            let left = if i + d < knots.len() && i < knots.len() {
                let denom = knots[i + d] - knots[i];
                if denom.abs() < 1e-15 { 0.0 } else { (t - knots[i]) / denom * basis[i] }
            } else { 0.0 };

            // Right term
            let right = if i + 1 < n && i + d + 1 < knots.len() {
                let denom = knots[i + d + 1] - knots[i + 1];
                if denom.abs() < 1e-15 { 0.0 } else { (knots[i + d + 1] - t) / denom * basis[i + 1] }
            } else { 0.0 };

            new_basis[i] = left + right;
        }
        basis = new_basis;
    }

    basis
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
fn solve_linear_system_gauss(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    if a.len() != n || a.iter().any(|row| row.len() != n) {
        return None;
    }

    // Create augmented matrix
    let mut aug: Vec<Vec<f64>> = a.iter().zip(b.iter())
        .map(|(row, &bi)| {
            let mut r = row.clone();
            r.push(bi);
            r
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-15 {
            // Singular or near-singular matrix
            continue;
        }

        // Swap rows
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Eliminate below
        let pivot = aug[col][col];
        for row in (col + 1)..n {
            if aug[row][col].abs() < 1e-30 { continue; }
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        if aug[i][i].abs() < 1e-15 {
            x[i] = 0.0; // Free variable
            continue;
        }
        let mut sum = aug[i][n]; // RHS
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        x[i] = sum / aug[i][i];
    }

    Some(x)
}

/// Approximate an offset surface as a NURBS surface.
///
/// Given a basis surface and an offset distance, this samples the basis surface
/// on a grid, offsets each grid point along the surface normal, and creates
/// a NURBS surface from the offset grid.
fn approximate_offset_surface(basis_surface: &Surface, distance: f64) -> Surface {
    let n_u = 16;
    let n_v = 16;

    // Determine parameter ranges
    let (u_min, u_max) = surface_param_range_u(basis_surface);
    let (v_min, v_max) = surface_param_range_v(basis_surface);

    let mut offset_grid: Vec<Vec<Point3d>> = Vec::with_capacity(n_u);

    for i in 0..n_u {
        let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1) as f64;
        let mut row = Vec::with_capacity(n_v);

        for j in 0..n_v {
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
            let p = basis_surface.point_at(u, v);
            let normal = basis_surface.normal_at(u, v);

            // Offset along the normal
            row.push(Point3d::new(
                p.x + distance * normal.x,
                p.y + distance * normal.y,
                p.z + distance * normal.z,
            ));
        }
        offset_grid.push(row);
    }

    // Create a degree-3 NURBS surface with the offset grid as control points
    let u_degree = 3.min(n_u - 1);
    let v_degree = 3.min(n_v - 1);

    // Generate clamped knot vectors
    let u_knots = generate_clamped_knots(n_u, u_degree);
    let v_knots = generate_clamped_knots(n_v, v_degree);

    // Unit weights
    let weights = vec![vec![1.0; n_v]; n_u];

    Surface::Nurbs(NurbsSurface {
        u_degree,
        v_degree,
        control_points: offset_grid,
        weights,
        u_knots,
        v_knots,
        u_closed: false,
        v_closed: false,
    })
}

/// Get the parameter range for u direction of a surface.
fn surface_param_range_u(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Nurbs(n) => n.u_range(),
        Surface::Plane(_) => (0.0, 1.0),
        Surface::Cylinder(c) => c.u_range(),
        Surface::Cone(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Sphere(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Torus(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Revolution(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Extrusion(_) => {
            if let Surface::Extrusion(e) = surface {
                e.profile.param_range()
            } else {
                (0.0, 1.0)
            }
        }
    }
}

/// Get the parameter range for v direction of a surface.
fn surface_param_range_v(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Nurbs(n) => n.v_range(),
        Surface::Plane(_) => (0.0, 1.0),
        Surface::Cylinder(_) => (-10.0, 10.0), // Infinite in v
        Surface::Cone(_) => (-10.0, 10.0),
        Surface::Sphere(_) => (0.0, std::f64::consts::PI),
        Surface::Torus(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Revolution(_) => {
            if let Surface::Revolution(r) = surface {
                r.profile.param_range()
            } else {
                (0.0, 1.0)
            }
        }
        Surface::Extrusion(_) => (-10.0, 10.0), // Infinite in v (extrusion direction)
    }
}

/// Generate a clamped uniform knot vector for n control points and given degree.
fn generate_clamped_knots(n: usize, degree: usize) -> Vec<f64> {
    let m = n + degree + 1;
    let mut knots = Vec::with_capacity(m);

    // First degree+1 knots = 0
    for _ in 0..=degree {
        knots.push(0.0);
    }

    // Interior knots: uniformly spaced
    let n_interior = n - degree - 1;
    for i in 1..=n_interior {
        knots.push(i as f64 / (n_interior + 1) as f64);
    }

    // Last degree+1 knots = 1
    for _ in 0..=degree {
        knots.push(1.0);
    }

    knots
}

/// Merge holes into an outer polygon using the bridge-edge technique.
/// For each hole, find the rightmost point of the hole, then find the
/// closest visible point on the outer polygon (or previously merged holes),
/// and insert the hole at that point with a bridge edge.
///
/// The resulting polygon has all holes connected via zero-width bridges,
/// forming a single simple polygon that can be triangulated with ear-clipping.
fn merge_holes_into_polygon(
    outer_2d: &[Point2d],
    outer_3d: &[Point3d],
    holes_2d: &[Vec<Point2d>],
    holes_3d: &[Vec<Point3d>],
) -> (Vec<Point2d>, Vec<Point3d>) {
    if outer_2d.is_empty() {
        return (Vec::new(), Vec::new());
    }
    if holes_2d.is_empty() {
        return (outer_2d.to_vec(), outer_3d.to_vec());
    }

    let mut poly_2d: Vec<Point2d> = outer_2d.to_vec();
    let mut poly_3d: Vec<Point3d> = outer_3d.to_vec();

    // Sort holes by rightmost point (u-coordinate) descending,
    // so we process rightmost holes first for more stable bridge construction
    let mut hole_indices: Vec<usize> = (0..holes_2d.len()).collect();
    hole_indices.sort_by(|&a, &b| {
        let max_u_a = holes_2d[a].iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
        let max_u_b = holes_2d[b].iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
        max_u_b.partial_cmp(&max_u_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    for hole_idx in hole_indices {
        let hole_2d = &holes_2d[hole_idx];
        let hole_3d = &holes_3d[hole_idx];
        if hole_2d.is_empty() { continue; }

        // Find the rightmost point of the hole
        let (rightmost_idx, _) = hole_2d.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.u.partial_cmp(&b.u).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or_else(|| (0, &hole_2d[0]));

        // Find the closest VISIBLE point on the polygon to the rightmost hole point.
        // For non-convex polygons, we must verify the bridge edge doesn't cross
        // any polygon edge or go outside the polygon.
        let hole_pt = hole_2d[rightmost_idx];

        // Sort polygon vertices by distance to the hole point (closest first)
        let mut candidates: Vec<(usize, f64)> = poly_2d.iter().enumerate()
            .map(|(i, pt)| {
                let dx = pt.u - hole_pt.u;
                let dy = pt.v - hole_pt.v;
                (i, dx * dx + dy * dy)
            })
            .collect();
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        let best_poly_idx = candidates.iter()
            .find(|(idx, _)| {
                let outer_pt = poly_2d[*idx];
                is_bridge_visible_converter(&poly_2d, hole_pt, outer_pt, *idx)
            })
            .map(|(idx, _)| *idx)
            .unwrap_or_else(|| candidates.first().map(|c| c.0).unwrap_or(0));

        // Insert the hole into the polygon at the bridge point
        // The bridge creates: ...poly[best] -> hole[rightmost] -> ...hole -> hole[rightmost] -> poly[best]...
        // This is done by inserting the hole (rotated to start at rightmost_idx)
        // twice at the bridge point, with the rightmost point duplicated.

        // Rotate hole to start at rightmost_idx
        let n_hole = hole_2d.len();
        let mut rotated_hole_2d = Vec::with_capacity(n_hole + 1);
        let mut rotated_hole_3d = Vec::with_capacity(n_hole + 1);
        for i in 0..=n_hole {
            let idx = (rightmost_idx + i) % n_hole;
            rotated_hole_2d.push(hole_2d[idx]);
            rotated_hole_3d.push(hole_3d[idx]);
        }

        // Insert: poly[..best+1] + bridge_point + hole + bridge_point + poly[best..]
        let mut new_poly_2d = Vec::new();
        let mut new_poly_3d = Vec::new();

        // Part 1: outer polygon up to and including the bridge point
        for i in 0..=best_poly_idx {
            new_poly_2d.push(poly_2d[i]);
            new_poly_3d.push(poly_3d[i]);
        }

        // Part 2: bridge to hole (rightmost point)
        new_poly_2d.push(hole_2d[rightmost_idx]);
        new_poly_3d.push(hole_3d[rightmost_idx]);

        // Part 3: hole vertices starting from rightmost+1 going around back to rightmost
        for i in 1..rotated_hole_2d.len() {
            new_poly_2d.push(rotated_hole_2d[i]);
            new_poly_3d.push(rotated_hole_3d[i]);
        }

        // Part 4: bridge back to the same outer polygon point
        new_poly_2d.push(poly_2d[best_poly_idx]);
        new_poly_3d.push(poly_3d[best_poly_idx]);

        // Part 5: rest of outer polygon after bridge point
        for i in (best_poly_idx + 1)..poly_2d.len() {
            new_poly_2d.push(poly_2d[i]);
            new_poly_3d.push(poly_3d[i]);
        }

        poly_2d = new_poly_2d;
        poly_3d = new_poly_3d;
    }

    (poly_2d, poly_3d)
}

#[cfg(test)]
mod diag_tests {
    use super::*;
    use crate::parse_step;

    fn diagnose_file(path: &str) {
        eprintln!("\n========================================");
        eprintln!("FILE: {}", path);
        eprintln!("========================================");
        
        let content = match std::fs::read_to_string(path) {
            Ok(c) => c,
            Err(e) => { eprintln!("ERROR reading: {}", e); return; }
        };
        
        let step = match parse_step(&content) {
            Ok(s) => s,
            Err(e) => { eprintln!("PARSE ERROR: {:?}", e); return; }
        };
        
        // Count surface types found in faces
        let faces = step.find_entities_by_type("ADVANCED_FACE");
        let shells = step.find_entities_by_type("CLOSED_SHELL");
        let open_shells = step.find_entities_by_type("OPEN_SHELL");
        let breps = step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        eprintln!("  ADVANCED_FACE: {}, CLOSED_SHELL: {}, OPEN_SHELL: {}, MANIFOLD_SOLID_BREP: {}",
            faces.len(), shells.len(), open_shells.len(), breps.len());

        // For each face, find its surface type
        let mut surface_types: HashMap<String, usize> = HashMap::new();
        let mut faces_with_no_surface = 0;
        for face in &faces {
            if let Some(surface) = StepConverter::new(&step).extract_face_surface_from_entity(face) {
                let tn = match surface {
                    Surface::Plane(_) => "PLANE",
                    Surface::Cylinder(_) => "CYLINDER",
                    Surface::Cone(_) => "CONE",
                    Surface::Sphere(_) => "SPHERE",
                    Surface::Torus(_) => "TORUS",
                    Surface::Revolution(_) => "REVOLUTION",
                    Surface::Extrusion(_) => "EXTRUSION",
                    Surface::Nurbs(_) => "NURBS",
                };
                *surface_types.entry(tn.to_string()).or_insert(0) += 1;
            } else {
                faces_with_no_surface += 1;
                // Print what entity the surface ref points to
                let converter = StepConverter::new(&step);
                for (i, param) in face.params.iter().enumerate() {
                    if i == 0 { continue; }
                    if let Some(surface_id) = converter.get_ref(param) {
                        if let Some(entity) = step.find_entity(surface_id) {
                            eprintln!("    FACE #{}: ref #{} type='{}'", face.id, surface_id, entity.type_name);
                        }
                    }
                }
            }
        }
        eprintln!("  Surface types extracted: {:?}", surface_types);
        eprintln!("  Faces with NO surface: {}", faces_with_no_surface);

        // Count FACE_BOUND vs FACE_OUTER_BOUND usage
        let outer_bounds = step.find_entities_by_type("FACE_OUTER_BOUND").len();
        let inner_bounds = step.find_entities_by_type("FACE_BOUND").len();
        eprintln!("  FACE_OUTER_BOUND: {}, FACE_BOUND (holes): {}", outer_bounds, inner_bounds);

        // Try full conversion
        let converter = StepConverter::new(&step);
        match converter.convert_instances() {
            Ok(instances) => {
                let total_verts: usize = instances.iter().map(|i| i.mesh.vertex_count()).sum();
                let total_tris: usize = instances.iter().map(|i| i.mesh.triangle_count()).sum();
                eprintln!("  MESH: {} instances, {} verts, {} tris", instances.len(), total_verts, total_tris);
                for inst in &instances {
                    eprintln!("    {} : {}v {}t color={:?}", inst.name, inst.mesh.vertex_count(), inst.mesh.triangle_count(), inst.color);
                }
            }
            Err(e) => eprintln!("  CONVERSION ERROR: {}", e),
        }
    }

    #[test]
    fn test_brick_thin() { diagnose_file("/home/z/my-project/3Draper/test/brick_thin.stp"); }
    #[test]
    fn test_brick_thin_hole() { diagnose_file("/home/z/my-project/3Draper/test/brick_thin_hole.stp"); }
    #[test]
    fn test_brick_thin_round() { diagnose_file("/home/z/my-project/3Draper/test/brick_thin_round.stp"); }
    #[test]
    fn test_compressor() { diagnose_file("/home/z/my-project/3Draper/test/compressor-13920_top.stp"); }
    #[test]
    fn test_drill() { diagnose_file("/home/z/my-project/3Draper/test/drill_top.stp"); }
    #[test]
    fn test_transmission() { diagnose_file("/home/z/my-project/3Draper/test/transmission_top.stp"); }
    #[test]
    fn test_3_05_078() { diagnose_file("/home/z/my-project/3Draper/test/3.05.078.stp"); }
    #[test]
    fn test_zentralstaender() { diagnose_file("/home/z/my-project/test/Zentralstaender.stp"); }

    /// Comprehensive surface triangulation diagnostic across ALL test STEP files.
    /// Checks each face for: surface type, boundary edges, triangulation success,
    /// finite vertices, reasonable area, hole handling, and special surface issues.
    #[test]
    fn test_surface_diagnostic() {
        let test_dir = "/home/z/my-project/3Draper_repo/test/";
        let step_files = [
            "SampleCube.step",
            "3.05.078.stp",
            "brick_thin_hole.stp",
            "brick_thin_round.stp",
            "brick_thin.stp",
            "compressor-13920_top.stp",
            "Zentralstaender.stp",
            "as1-oc-214.stp",
            "drill_top.stp",
            "transmission_top.stp",
        ];

        let mut grand_total_faces = 0usize;
        let mut grand_total_empty = 0usize;
        let mut grand_total_nan = 0usize;
        let mut grand_total_zero_area = 0usize;
        let mut grand_total_inf_area = 0usize;
        let mut grand_total_tris = 0usize;
        let mut grand_total_verts = 0usize;
        let mut grand_surface_counts: HashMap<String, usize> = HashMap::new();
        let mut grand_fail_by_type: HashMap<String, usize> = HashMap::new();

        for fname in &step_files {
            let path = format!("{}{}", test_dir, fname);
            eprintln!("\n{}", "=".repeat(70));
            eprintln!("FILE: {}", fname);
            eprintln!("{}", "=".repeat(70));

            let content = match std::fs::read_to_string(&path) {
                Ok(c) => c,
                Err(e) => { eprintln!("  ERROR reading: {}", e); continue; }
            };

            let step = match parse_step(&content) {
                Ok(s) => s,
                Err(e) => { eprintln!("  PARSE ERROR: {:?}", e); continue; }
            };

            let converter = StepConverter::new(&step);
            let params = TriangulationParams::default();
            let bbox = converter.compute_bounding_box();

            // ─── Per-face diagnostics using FaceData ────────────────────────
            let breps = step.find_entities_by_type("MANIFOLD_SOLID_BREP");
            let faceted_breps = step.find_entities_by_type("FACETED_BREP");

            let mut file_total_faces = 0usize;
            let mut file_surface_counts: HashMap<String, usize> = HashMap::new();
            let mut file_faces_with_holes = 0usize;
            let mut file_empty_meshes = 0usize;
            let mut file_nan_vertices = 0usize;
            let mut file_zero_area_tris = 0usize;
            let mut file_inf_area = 0usize;
            let mut file_fail_by_type: HashMap<String, usize> = HashMap::new();
            let mut file_total_tris = 0usize;
            let mut file_total_verts = 0usize;
            let mut file_cone_issues = Vec::new();
            let mut file_sphere_issues = Vec::new();
            let mut file_cylinder_issues = Vec::new();
            let mut file_hole_issues = Vec::new();

            let all_brep_ids: Vec<i64> = breps.iter().chain(faceted_breps.iter())
                .map(|e| e.id).collect();

            for brep_id in &all_brep_ids {
                let shell_id = match converter.find_shell_ref_by_brep_id(*brep_id) {
                    Some(id) => id,
                    None => continue,
                };
                let face_data_list = match converter.extract_shell_faces(shell_id, false) {
                    Some(list) => list,
                    None => continue,
                };

                for (fi, face_data) in face_data_list.iter().enumerate() {
                    file_total_faces += 1;

                    let surface_type = match &face_data.surface {
                        Surface::Plane(_) => "Plane",
                        Surface::Cylinder(_) => "Cylinder",
                        Surface::Cone(_) => "Cone",
                        Surface::Sphere(_) => "Sphere",
                        Surface::Torus(_) => "Torus",
                        Surface::Revolution(_) => "Revolution",
                        Surface::Extrusion(_) => "Extrusion",
                        Surface::Nurbs(_) => "Nurbs",
                    }.to_string();
                    *file_surface_counts.entry(surface_type.clone()).or_insert(0) += 1;

                    // Count outer/inner edges and their curve types
                    let n_outer = face_data.outer_edges.len();
                    let n_inner_loops = face_data.inner_edges.len();
                    let n_inner_edges: usize = face_data.inner_edges.iter().map(|l| l.len()).sum();

                    let mut edge_type_counts: HashMap<String, usize> = HashMap::new();
                    for edge in &face_data.outer_edges {
                        if let Some(ref curve) = edge.curve {
                            let tn = match curve {
                                Curve3d::Line(_) => "Line",
                                Curve3d::Circle(_) => "Circle",
                                Curve3d::Ellipse(_) => "Ellipse",
                                Curve3d::Arc(_) => "Arc",
                                Curve3d::Hyperbola(_) => "Hyperbola",
                                Curve3d::Parabola(_) => "Parabola",
                                Curve3d::Nurbs(_) => "Nurbs",
                                Curve3d::PCurve { .. } => "PCurve",
                                Curve3d::Trimmed { .. } => "Trimmed",
                                Curve3d::Composite { .. } => "Composite",
                            };
                            *edge_type_counts.entry(tn.to_string()).or_insert(0) += 1;
                        }
                    }
                    for inner_loop in &face_data.inner_edges {
                        for edge in inner_loop {
                            if let Some(ref curve) = edge.curve {
                                let tn = match curve {
                                    Curve3d::Line(_) => "Line",
                                    Curve3d::Circle(_) => "Circle",
                                    Curve3d::Ellipse(_) => "Ellipse",
                                    Curve3d::Arc(_) => "Arc",
                                    Curve3d::Hyperbola(_) => "Hyperbola",
                                    Curve3d::Parabola(_) => "Parabola",
                                    Curve3d::Nurbs(_) => "Nurbs",
                                    Curve3d::PCurve { .. } => "PCurve",
                                    Curve3d::Trimmed { .. } => "Trimmed",
                                    Curve3d::Composite { .. } => "Composite",
                                };
                                *edge_type_counts.entry(tn.to_string()).or_insert(0) += 1;
                            }
                        }
                    }

                    if n_inner_loops > 0 {
                        file_faces_with_holes += 1;
                    }

                    // ─── Triangulate ─────────────────────────────────────────
                    let face_mesh = converter.surface_to_mesh(face_data, &params, &bbox);
                    let tri_count = face_mesh.triangle_count();
                    let vert_count = face_mesh.vertex_count();
                    file_total_tris += tri_count;
                    file_total_verts += vert_count;

                    if tri_count == 0 {
                        file_empty_meshes += 1;
                        *file_fail_by_type.entry(surface_type.clone()).or_insert(0) += 1;
                        eprintln!("  EMPTY MESH: BREP#{} face[{}] {} outer_edges={} inner_loops={} inner_edges={} edges={:?} forward={}",
                            brep_id, fi, surface_type, n_outer, n_inner_loops, n_inner_edges,
                            edge_type_counts, face_data.forward);

                        // For cone/sphere/cylinder faces that failed, show UV diagnostics
                        if matches!(face_data.surface, Surface::Cone(_) | Surface::Sphere(_) | Surface::Cylinder(_)) {
                            let mut uv_samples = Vec::new();
                            for edge in &face_data.outer_edges {
                                for ti in 0..4 {
                                    if let Some(p) = edge.point_at(ti as f64 / 3.0) {
                                        let (u, v) = face_data.surface.project_point(&p);
                                        uv_samples.push(format!("({:.3},{:.3})", u, v));
                                    }
                                }
                            }
                            eprintln!("    UV boundary samples: {}", uv_samples.iter().take(8).cloned().collect::<Vec<_>>().join(", "));
                        }
                        continue;
                    }

                    // ─── Check for NaN / Inf vertices ────────────────────────
                    let mut has_nan = false;
                    for v in &face_mesh.vertices {
                        if v.x.is_nan() || v.y.is_nan() || v.z.is_nan() ||
                           v.x.is_infinite() || v.y.is_infinite() || v.z.is_infinite() {
                            has_nan = true;
                            break;
                        }
                    }
                    if has_nan {
                        file_nan_vertices += 1;
                        let nan_count = face_mesh.vertices.iter()
                            .filter(|v| v.x.is_nan() || v.y.is_nan() || v.z.is_nan() ||
                                        v.x.is_infinite() || v.y.is_infinite() || v.z.is_infinite())
                            .count();
                        eprintln!("  NaN/Inf VERTICES: BREP#{} face[{}] {} => {} of {} vertices are non-finite",
                            brep_id, fi, surface_type, nan_count, vert_count);
                    }

                    // ─── Check triangle areas ─────────────────────────────────
                    let mesh_area = face_mesh.surface_area();
                    if mesh_area == 0.0 {
                        file_zero_area_tris += 1;
                        eprintln!("  ZERO AREA: BREP#{} face[{}] {} => {} tris but total area=0",
                            brep_id, fi, surface_type, tri_count);
                    } else if mesh_area.is_infinite() {
                        file_inf_area += 1;
                        eprintln!("  INF AREA: BREP#{} face[{}] {} => area is infinite",
                            brep_id, fi, surface_type);
                    }

                    // Count individual zero-area triangles (cap at 10k tris to avoid O(n²) on huge meshes)
                    if tri_count <= 10000 {
                        let mut zero_tri_count = 0usize;
                        for tri_idx in 0..face_mesh.triangles.len() {
                            let tri = &face_mesh.triangles[tri_idx];
                            let v0 = face_mesh.vertices[tri[0] as usize];
                            let v1 = face_mesh.vertices[tri[1] as usize];
                            let v2 = face_mesh.vertices[tri[2] as usize];
                            let e1x = v1.x - v0.x; let e1y = v1.y - v0.y; let e1z = v1.z - v0.z;
                            let e2x = v2.x - v0.x; let e2y = v2.y - v0.y; let e2z = v2.z - v0.z;
                            let cx = e1y * e2z - e1z * e2y;
                            let cy = e1z * e2x - e1x * e2z;
                            let cz = e1x * e2y - e1y * e2x;
                            let area2 = (cx*cx + cy*cy + cz*cz).sqrt();
                            if area2 < 1e-20 {
                                zero_tri_count += 1;
                            }
                        }
                        if zero_tri_count > 0 {
                            eprintln!("  ZERO-AREA TRIS: BREP#{} face[{}] {} => {}/{} degenerate tris",
                                brep_id, fi, surface_type, zero_tri_count, tri_count);
                        }
                    }

                    // ─── Special surface checks ───────────────────────────────

                    // Cone: check v range and apex degeneracy
                    if let Surface::Cone(cone) = &face_data.surface {
                        let mut v_min = f64::MAX;
                        let mut v_max = f64::MIN;
                        for edge in &face_data.outer_edges {
                            for ti in 0..4 {
                                if let Some(p) = edge.point_at(ti as f64 / 3.0) {
                                    let (_u, v) = cone.project_point(&p);
                                    v_min = v_min.min(v);
                                    v_max = v_max.max(v);
                                }
                            }
                        }
                        let apex_height = cone.height();
                        let touches_apex = v_max >= apex_height * 0.99;
                        if touches_apex {
                            // Apex degeneracy: all u values should map to the same point at v=apex_height
                            // Check if triangulation handles it correctly
                            let apex_points: Vec<Point3d> = face_mesh.vertices.iter()
                                .filter(|v| {
                                    let (_u, vv) = cone.project_point(v);
                                    vv >= apex_height * 0.95
                                })
                                .cloned()
                                .collect();
                            if apex_points.len() > 2 {
                                // Check how spread they are (should be very close to each other)
                                let first = apex_points[0];
                                let max_spread = apex_points.iter()
                                    .map(|p| (p.x-first.x).abs().max((p.y-first.y).abs()).max((p.z-first.z).abs()))
                                    .fold(0.0f64, f64::max);
                                if max_spread > apex_height * 0.1 {
                                    file_cone_issues.push(format!(
                                        "BREP#{} face[{}]: apex degeneracy spread={:.4} (height={:.4}) v_range=[{:.4},{:.4}]",
                                        brep_id, fi, max_spread, apex_height, v_min, v_max));
                                }
                            }
                        }
                        if v_min == f64::MAX {
                            file_cone_issues.push(format!(
                                "BREP#{} face[{}]: could not compute v_range from boundary edges",
                                brep_id, fi));
                        }
                    }

                    // Sphere: check pole handling
                    if let Surface::Sphere(sphere) = &face_data.surface {
                        let mut v_values = Vec::new();
                        for edge in &face_data.outer_edges {
                            for ti in 0..4 {
                                if let Some(p) = edge.point_at(ti as f64 / 3.0) {
                                    let (_u, v) = sphere.project_point(&p);
                                    v_values.push(v);
                                }
                            }
                        }
                        // v=0 is north pole, v=pi is south pole
                        let touches_north = v_values.iter().any(|v| *v < 0.05);
                        let touches_south = v_values.iter().any(|v| *v > std::f64::consts::PI - 0.05);
                        if touches_north || touches_south {
                            // Check if mesh has vertices near the poles
                            let pole = if touches_north { "north" } else { "south" };
                            let pole_v = if touches_north { 0.0 } else { std::f64::consts::PI };
                            let near_pole: Vec<&Point3d> = face_mesh.vertices.iter()
                                .filter(|v| {
                                    let (_u, vv) = sphere.project_point(v);
                                    (vv - pole_v).abs() < 0.1
                                })
                                .collect();
                            if near_pole.is_empty() && tri_count > 0 {
                                file_sphere_issues.push(format!(
                                    "BREP#{} face[{}]: touches {} pole but no mesh vertices near pole",
                                    brep_id, fi, pole));
                            }
                        }
                    }

                    // Cylinder: check v range from boundary edges
                    if let Surface::Cylinder(cyl) = &face_data.surface {
                        let mut v_min = f64::MAX;
                        let mut v_max = f64::MIN;
                        for edge in &face_data.outer_edges {
                            for ti in 0..4 {
                                if let Some(p) = edge.point_at(ti as f64 / 3.0) {
                                    let (_u, v) = cyl.project_point(&p);
                                    v_min = v_min.min(v);
                                    v_max = v_max.max(v);
                                }
                            }
                        }
                        if v_max - v_min < 1e-10 {
                            file_cylinder_issues.push(format!(
                                "BREP#{} face[{}]: v_range degenerate [{:.6},{:.6}] delta={:.2e}",
                                brep_id, fi, v_min, v_max, v_max - v_min));
                        }
                    }

                    // Holes: check if inner boundaries produce valid triangulation
                    if n_inner_loops > 0 {
                        // Check that inner loop edges are properly oriented
                        for (li, inner_loop) in face_data.inner_edges.iter().enumerate() {
                            if inner_loop.is_empty() {
                                file_hole_issues.push(format!(
                                    "BREP#{} face[{}]: inner loop {} is EMPTY",
                                    brep_id, fi, li));
                            }
                        }
                        // Check that the triangulated mesh has fewer triangles than
                        // a version without holes would (rough check)
                    }
                }
            }

            // ─── Skip step_to_mesh_instances cross-check (redundant with per-face analysis) ──
            let n_instances = 0usize;
            let n_detailed_tris = 0usize;

            // ─── Print summary table ────────────────────────────────────────
            eprintln!("\n  ┌─────────────────────────────────────────────────────┐");
            eprintln!("  │  SUMMARY: {} ", fname);
            eprintln!("  ├─────────────────────────────────────────────────────┤");
            eprintln!("  │  Total faces (FaceData):  {:>6}", file_total_faces);
            eprintln!("  │  Instances:               {:>6}", n_instances);
            eprintln!("  │  Instance mesh tris:      {:>6}", n_detailed_tris);
            eprintln!("  │  Faces with holes:        {:>6}", file_faces_with_holes);

            eprintln!("  │  ─── Surface Types ───────────────────────────────");
            let mut sorted_types: Vec<_> = file_surface_counts.iter().collect();
            sorted_types.sort_by(|a, b| b.1.cmp(a.1));
            for (st, count) in &sorted_types {
                eprintln!("  │    {:<20} {:>6}", format!("{}:", st), count);
            }

            eprintln!("  │  ─── Triangulation Results ────────────────────────");
            eprintln!("  │    Total triangles:       {:>6}", file_total_tris);
            eprintln!("  │    Total vertices:        {:>6}", file_total_verts);
            eprintln!("  │    Empty meshes (FAIL):   {:>6}", file_empty_meshes);
            eprintln!("  │    NaN/Inf vertices:      {:>6}", file_nan_vertices);
            eprintln!("  │    Zero-area mesh:        {:>6}", file_zero_area_tris);
            eprintln!("  │    Infinite-area mesh:    {:>6}", file_inf_area);

            if !file_fail_by_type.is_empty() {
                eprintln!("  │  ─── Failures by Surface Type ─────────────────────");
                for (st, count) in &file_fail_by_type {
                    eprintln!("  │    {:<20} {:>6} EMPTY", format!("{}:", st), count);
                }
            }

            if !file_cone_issues.is_empty() {
                eprintln!("  │  ─── Cone Issues ─────────────────────────────────");
                for issue in &file_cone_issues {
                    eprintln!("  │    {}", issue);
                }
            }
            if !file_sphere_issues.is_empty() {
                eprintln!("  │  ─── Sphere Issues ───────────────────────────────");
                for issue in &file_sphere_issues {
                    eprintln!("  │    {}", issue);
                }
            }
            if !file_cylinder_issues.is_empty() {
                eprintln!("  │  ─── Cylinder Issues ─────────────────────────────");
                for issue in &file_cylinder_issues {
                    eprintln!("  │    {}", issue);
                }
            }
            if !file_hole_issues.is_empty() {
                eprintln!("  │  ─── Hole Issues ─────────────────────────────────");
                for issue in &file_hole_issues {
                    eprintln!("  │    {}", issue);
                }
            }
            eprintln!("  └─────────────────────────────────────────────────────┘");

            // Accumulate grand totals
            grand_total_faces += file_total_faces;
            grand_total_empty += file_empty_meshes;
            grand_total_nan += file_nan_vertices;
            grand_total_zero_area += file_zero_area_tris;
            grand_total_inf_area += file_inf_area;
            grand_total_tris += file_total_tris;
            grand_total_verts += file_total_verts;
            for (st, count) in &file_surface_counts {
                *grand_surface_counts.entry(st.clone()).or_insert(0) += count;
            }
            for (st, count) in &file_fail_by_type {
                *grand_fail_by_type.entry(st.clone()).or_insert(0) += count;
            }
        }

        // ─── Grand summary ─────────────────────────────────────────────────
        eprintln!("\n{}", "═".repeat(72));
        eprintln!("GRAND SUMMARY — ALL TEST STEP FILES");
        eprintln!("{}", "═".repeat(72));
        eprintln!("  Total faces across all files:    {}", grand_total_faces);
        eprintln!("  Total triangles:                 {}", grand_total_tris);
        eprintln!("  Total vertices:                  {}", grand_total_verts);
        eprintln!("  Empty mesh failures:             {}", grand_total_empty);
        eprintln!("  NaN/Inf vertex failures:         {}", grand_total_nan);
        eprintln!("  Zero-area mesh failures:         {}", grand_total_zero_area);
        eprintln!("  Infinite-area mesh failures:     {}", grand_total_inf_area);
        eprintln!();
        eprintln!("  Surface type distribution:");
        let mut sorted: Vec<_> = grand_surface_counts.iter().collect();
        sorted.sort_by(|a, b| b.1.cmp(a.1));
        for (st, count) in &sorted {
            let fails = grand_fail_by_type.get(*st).copied().unwrap_or(0);
            if fails > 0 {
                eprintln!("    {:<20} {:>6}  ({} empty)", st, count, fails);
            } else {
                eprintln!("    {:<20} {:>6}  ✓", st, count);
            }
        }
        eprintln!();

        let total_issues = grand_total_empty + grand_total_nan + grand_total_zero_area + grand_total_inf_area;
        if total_issues == 0 {
            eprintln!("  ✓ ALL FACES TRIANGULATED SUCCESSFULLY — NO ISSUES FOUND");
        } else {
            eprintln!("  ✗ TOTAL ISSUES: {} (empty={}, NaN={}, zero_area={}, inf_area={})",
                total_issues, grand_total_empty, grand_total_nan, grand_total_zero_area, grand_total_inf_area);
        }
    }

    #[test]
    fn test_zentralstaender_face_detail() {
        let path = "/home/z/my-project/test/Zentralstaender.stp";
        let content = std::fs::read_to_string(path).unwrap();
        let step = parse_step(&content).unwrap();
        let converter = StepConverter::new(&step);
        let params = TriangulationParams::default();
        let bbox = converter.compute_bounding_box();

        let breps = step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        eprintln!("\n=== Zentralstaender Face Detail ===");
        eprintln!("BREPs: {}", breps.len());

        let mut total_faces = 0usize;
        let mut empty_faces = 0usize;
        let mut surface_type_counts: HashMap<String, usize> = HashMap::new();
        let mut empty_by_type: HashMap<String, usize> = HashMap::new();

        for brep in &breps {
            if let Some(shell_id) = converter.find_shell_ref_by_brep_id(brep.id) {
                if let Some(face_data_list) = converter.extract_shell_faces(shell_id, false) {
                    for (fi, face_data) in face_data_list.iter().enumerate() {
                        total_faces += 1;
                        let surface_type = match &face_data.surface {
                            Surface::Plane(_) => "Plane",
                            Surface::Cylinder(_) => "Cylinder",
                            Surface::Cone(_) => "Cone",
                            Surface::Sphere(_) => "Sphere",
                            Surface::Torus(_) => "Torus",
                            Surface::Revolution(_) => "Revolution",
                            Surface::Extrusion(_) => "Extrusion",
                            Surface::Nurbs(_) => "Nurbs",
                        }.to_string();
                        *surface_type_counts.entry(surface_type.clone()).or_insert(0) += 1;

                        let face_mesh = converter.surface_to_mesh(face_data, &params, &bbox);
                        let tri_count = face_mesh.triangle_count();
                        if tri_count == 0 {
                            empty_faces += 1;
                            *empty_by_type.entry(surface_type.clone()).or_insert(0) += 1;
                            // Sample boundary points to understand what we have
                            let mut bp_count = 0;
                            let mut bp_sample_pts = Vec::new();
                            for edge in &face_data.outer_edges {
                                for i in 0..4 {
                                    let t = i as f64 / 3.0;
                                    if let Some(p) = edge.point_at(t) {
                                        bp_count += 1;
                                        bp_sample_pts.push(p);
                                    }
                                }
                            }
                            // Project boundary points to UV
                            let uv_samples: Vec<_> = bp_sample_pts.iter().map(|p| face_data.surface.project_point(p)).collect();
                            eprintln!("  BREP #{} face[{}]: {} edges={}/{} forward={} => 0 TRIANGLES! bp_sample={}",
                                brep.id, fi, surface_type, face_data.outer_edges.len(), face_data.inner_edges.len(),
                                face_data.forward, bp_count);
                            eprintln!("    UV samples: {:?}", uv_samples.iter().take(6).collect::<Vec<_>>());
                            // Also show inner edge UV samples
                            for (li, inner_loop) in face_data.inner_edges.iter().enumerate() {
                                for edge in inner_loop {
                                    if let Some(p0) = edge.point_at(0.0) {
                                        let (u, v) = face_data.surface.project_point(&p0);
                                        eprintln!("    inner[{}] uv=({:.4},{:.4}) 3d=({:.2},{:.2},{:.2})", li, u, v, p0.x, p0.y, p0.z);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        eprintln!("\nTotal faces: {}, Empty faces: {}", total_faces, empty_faces);
        eprintln!("Surface types: {:?}", surface_type_counts);
        eprintln!("Empty by type: {:?}", empty_by_type);
    }

    /// Detailed diagnostic for cone faces in Zentralstaender.stp that produce
    /// degenerate triangulation (720/720 degenerate triangles in BREP#1086 and BREP#1088).
    /// Examines surface parameters, boundary edges, UV ranges, and apex detection.
    #[test]
    fn test_zentralstaender_cone_detail() {
        let path = "/home/z/my-project/3Draper_repo/test/Zentralstaender.stp";
        let content = match std::fs::read_to_string(path) {
            Ok(c) => c,
            Err(e) => { eprintln!("ERROR reading {}: {}", path, e); return; }
        };
        let step = match parse_step(&content) {
            Ok(s) => s,
            Err(e) => { eprintln!("PARSE ERROR: {:?}", e); return; }
        };
        let converter = StepConverter::new(&step);
        let params = TriangulationParams::default();
        let bbox = converter.compute_bounding_box();

        let target_brep_ids: Vec<i64> = vec![1086, 1088];

        eprintln!("\n{}", "=".repeat(80));
        eprintln!("ZENTRALSTAENDER CONE FACE DIAGNOSTIC — BREP#1086 & BREP#1088");
        eprintln!("{}", "=".repeat(80));

        let breps = step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        eprintln!("Total MANIFOLD_SOLID_BREP entities: {}", breps.len());
        eprintln!("Target BREP IDs: {:?}", target_brep_ids);

        for brep in &breps {
            if !target_brep_ids.contains(&brep.id) {
                continue;
            }

            let shell_id = match converter.find_shell_ref_by_brep_id(brep.id) {
                Some(id) => id,
                None => {
                    eprintln!("\nBREP#{} — could not find shell ref, skipping", brep.id);
                    continue;
                }
            };

            let face_data_list = match converter.extract_shell_faces(shell_id, false) {
                Some(list) => list,
                None => {
                    eprintln!("\nBREP#{} — could not extract shell faces, skipping", brep.id);
                    continue;
                }
            };

            eprintln!("\n{}", "-".repeat(80));
            eprintln!("BREP#{} — {} faces total", brep.id, face_data_list.len());
            eprintln!("{}", "-".repeat(80));

            for (fi, face_data) in face_data_list.iter().enumerate() {
                // Only examine cone faces
                let cone = match &face_data.surface {
                    Surface::Cone(c) => c,
                    _ => continue,
                };

                eprintln!("\n  ┌─────────────────────────────────────────────────────────────");
                eprintln!("  │ BREP#{} face[{}] (STEP face #{})", brep.id, fi, face_data.step_face_id);
                eprintln!("  ├─────────────────────────────────────────────────────────────");

                // (a) Surface parameters
                let half_angle_deg = cone.half_angle.to_degrees();
                let height = cone.height();
                eprintln!("  │ Surface: CONE");
                eprintln!("  │   half_angle = {:.6} rad = {:.4} deg", cone.half_angle, half_angle_deg);
                eprintln!("  │   radius     = {:.6}", cone.radius);
                eprintln!("  │   height     = {:.6}", height);
                eprintln!("  │   origin     = ({:.6}, {:.6}, {:.6})", cone.origin.x, cone.origin.y, cone.origin.z);
                eprintln!("  │   axis       = ({:.6}, {:.6}, {:.6})", cone.axis.x, cone.axis.y, cone.axis.z);
                eprintln!("  │   x_dir      = ({:.6}, {:.6}, {:.6})", cone.x_dir.x, cone.x_dir.y, cone.x_dir.z);
                eprintln!("  │   forward    = {}", face_data.forward);

                // (b) Number of boundary edges and their curve types
                let n_outer = face_data.outer_edges.len();
                let n_inner_loops = face_data.inner_edges.len();
                let n_inner_edges: usize = face_data.inner_edges.iter().map(|l| l.len()).sum();

                let mut outer_edge_types: Vec<String> = Vec::new();
                for edge in &face_data.outer_edges {
                    let tn = match &edge.curve {
                        Some(Curve3d::Line(_)) => "Line",
                        Some(Curve3d::Circle(_)) => "Circle",
                        Some(Curve3d::Ellipse(_)) => "Ellipse",
                        Some(Curve3d::Arc(_)) => "Arc",
                        Some(Curve3d::Hyperbola(_)) => "Hyperbola",
                        Some(Curve3d::Parabola(_)) => "Parabola",
                        Some(Curve3d::Nurbs(_)) => "Nurbs",
                        Some(Curve3d::PCurve { .. }) => "PCurve",
                        Some(Curve3d::Trimmed { .. }) => "Trimmed",
                        Some(Curve3d::Composite { .. }) => "Composite",
                        None => "None",
                    };
                    outer_edge_types.push(tn.to_string());
                }

                let mut inner_edge_types: Vec<String> = Vec::new();
                for inner_loop in &face_data.inner_edges {
                    for edge in inner_loop {
                        let tn = match &edge.curve {
                            Some(Curve3d::Line(_)) => "Line",
                            Some(Curve3d::Circle(_)) => "Circle",
                            Some(Curve3d::Ellipse(_)) => "Ellipse",
                            Some(Curve3d::Arc(_)) => "Arc",
                            Some(Curve3d::Hyperbola(_)) => "Hyperbola",
                            Some(Curve3d::Parabola(_)) => "Parabola",
                            Some(Curve3d::Nurbs(_)) => "Nurbs",
                            Some(Curve3d::PCurve { .. }) => "PCurve",
                            Some(Curve3d::Trimmed { .. }) => "Trimmed",
                            Some(Curve3d::Composite { .. }) => "Composite",
                            None => "None",
                        };
                        inner_edge_types.push(tn.to_string());
                    }
                }

                eprintln!("  │ Boundary: {} outer edges {:?}", n_outer, outer_edge_types);
                eprintln!("  │           {} inner loops, {} inner edges {:?}", n_inner_loops, n_inner_edges, inner_edge_types);

                // (c) Projected UV range of boundary points
                let mut u_min = f64::MAX; let mut u_max = f64::MIN;
                let mut v_min = f64::MAX; let mut v_max = f64::MIN;
                let mut boundary_pts_3d: Vec<Point3d> = Vec::new();

                // Sample outer edges densely
                for edge in &face_data.outer_edges {
                    let n_samples = 20; // denser sampling for accurate UV range
                    for i in 0..=n_samples {
                        let t = i as f64 / n_samples as f64;
                        if let Some(p) = edge.point_at(t) {
                            let (u, v) = cone.project_point(&p);
                            u_min = u_min.min(u);
                            u_max = u_max.max(u);
                            v_min = v_min.min(v);
                            v_max = v_max.max(v);
                            boundary_pts_3d.push(p);
                        }
                    }
                }

                // Also sample inner edges
                for inner_loop in &face_data.inner_edges {
                    for edge in inner_loop {
                        let n_samples = 20;
                        for i in 0..=n_samples {
                            let t = i as f64 / n_samples as f64;
                            if let Some(p) = edge.point_at(t) {
                                let (u, v) = cone.project_point(&p);
                                u_min = u_min.min(u);
                                u_max = u_max.max(u);
                                v_min = v_min.min(v);
                                v_max = v_max.max(v);
                                boundary_pts_3d.push(p);
                            }
                        }
                    }
                }

                let u_range = u_max - u_min;
                let v_range = v_max - v_min;
                eprintln!("  │ UV range: u=[{:.6}, {:.6}] range={:.6}", u_min, u_max, u_range);
                eprintln!("  │           v=[{:.6}, {:.6}] range={:.6}", v_min, v_max, v_range);

                // (d) Whether top_at_apex detection triggers
                // Replicate the logic from triangulate_cone_face:
                //   let apex_v = cone.height();
                //   let top_at_apex = (v_max - apex_v).abs() < apex_v * 0.05 + 1e-6;
                let apex_v = height;
                let v_max_clamped = v_max.min(apex_v);
                let top_at_apex = (v_max_clamped - apex_v).abs() < apex_v * 0.05 + 1e-6;
                eprintln!("  │ top_at_apex: {} (v_max_clamped={:.6}, apex_v={:.6}, threshold={:.6})",
                    top_at_apex, v_max_clamped, apex_v, apex_v * 0.05 + 1e-6);

                // Also check the other conditions from triangulate_cone_face:
                let v_range_degenerate = v_range < apex_v * 0.001 + 1e-6;
                let full_circle = u_range < 0.5 * std::f64::consts::PI || u_range > 1.9 * std::f64::consts::PI;
                eprintln!("  │ v_range_degenerate (cap face): {}", v_range_degenerate);
                eprintln!("  │ full_circle: {}", full_circle);

                // (e) The v_min, v_max, apex_v values
                eprintln!("  │ v_min={:.6}, v_max={:.6}, apex_v={:.6}", v_min, v_max, apex_v);
                eprintln!("  │ v_max - v_min = {:.6}", v_max - v_min);
                eprintln!("  │ apex_v - v_max = {:.6}", apex_v - v_max);

                // (f) First 5 boundary points (3D coordinates)
                eprintln!("  │ First 5 boundary points:");
                for (i, p) in boundary_pts_3d.iter().take(5).enumerate() {
                    let (u, v) = cone.project_point(p);
                    eprintln!("  │   [{}] ({:.4}, {:.4}, {:.4}) → uv=({:.4}, {:.4})", i, p.x, p.y, p.z, u, v);
                }

                // (g) Whether the face has inner edges
                eprintln!("  │ Has inner edges: {} ({} loops, {} total inner edges)",
                    n_inner_loops > 0, n_inner_loops, n_inner_edges);

                // Now triangulate and report results
                let face_mesh = converter.surface_to_mesh(face_data, &params, &bbox);
                let tri_count = face_mesh.triangle_count();
                let vert_count = face_mesh.vertex_count();

                // Count degenerate triangles
                let mut degenerate_count = 0usize;
                for tri_idx in 0..face_mesh.triangles.len() {
                    let tri = &face_mesh.triangles[tri_idx];
                    let v0 = face_mesh.vertices[tri[0] as usize];
                    let v1 = face_mesh.vertices[tri[1] as usize];
                    let v2 = face_mesh.vertices[tri[2] as usize];
                    let e1x = v1.x - v0.x; let e1y = v1.y - v0.y; let e1z = v1.z - v0.z;
                    let e2x = v2.x - v0.x; let e2y = v2.y - v0.y; let e2z = v2.z - v0.z;
                    let cx = e1y * e2z - e1z * e2y;
                    let cy = e1z * e2x - e1x * e2z;
                    let cz = e1x * e2y - e1y * e2x;
                    let area2 = (cx*cx + cy*cy + cz*cz).sqrt();
                    if area2 < 1e-10 {
                        degenerate_count += 1;
                    }
                }

                let mesh_area = face_mesh.surface_area();
                eprintln!("  │ Triangulation: {} tris, {} verts, {} degenerate, area={:.6}",
                    tri_count, vert_count, degenerate_count, mesh_area);

                if tri_count > 0 && degenerate_count == tri_count {
                    eprintln!("  │ ★ ALL {} TRIANGLES ARE DEGENERATE ★", tri_count);
                } else if degenerate_count > 0 {
                    eprintln!("  │ ⚠ {}/{} triangles are degenerate", degenerate_count, tri_count);
                }

                eprintln!("  └─────────────────────────────────────────────────────────────");
            }
        }

        eprintln!("\n{}", "=".repeat(80));
        eprintln!("END ZENTRALSTAENDER CONE DIAGNOSTIC");
        eprintln!("{}", "=".repeat(80));
    }

    /// Test that convert_instances and convert_detailed_instances produce
    /// non-empty results for all test STEP files.
    #[test]
    fn test_all_files_instance_conversion() {
        let test_dir = "/home/z/my-project/3Draper_repo/test/";
        let step_files = [
            "brick_thin.stp",
            "brick_thin_hole.stp",
            "brick_thin_round.stp",
            "3.05.078.stp",
            "compressor-13920_top.stp",
            "drill_top.stp",
            "transmission_top.stp",
            "Zentralstaender.stp",
        ];

        for fname in &step_files {
            let path = format!("{}{}", test_dir, fname);
            eprintln!("\n=== Testing instance conversion: {} ===", fname);

            let content = match std::fs::read_to_string(&path) {
                Ok(c) => c,
                Err(e) => { eprintln!("  ERROR reading: {}", e); continue; }
            };

            let step = match parse_step(&content) {
                Ok(s) => s,
                Err(e) => { eprintln!("  PARSE ERROR: {:?}", e); continue; }
            };

            // Test convert_instances
            let converter = StepConverter::new(&step);
            match converter.convert_instances() {
                Ok(instances) => {
                    let total_tris: usize = instances.iter().map(|i| i.mesh.triangle_count()).sum();
                    eprintln!("  convert_instances: {} instances, {} total tris", instances.len(), total_tris);
                    if total_tris == 0 {
                        eprintln!("  ⚠ NO TRIANGLES GENERATED — file may not convert properly!");
                    }
                }
                Err(e) => { eprintln!("  convert_instances ERROR: {}", e); }
            }

            // Test convert_detailed_instances
            let converter2 = StepConverter::new(&step);
            match converter2.convert_detailed_instances() {
                Ok(instances) => {
                    let total_tris: usize = instances.iter().map(|i| i.mesh.triangle_count()).sum();
                    eprintln!("  convert_detailed_instances: {} instances, {} total tris", instances.len(), total_tris);
                    if total_tris == 0 {
                        eprintln!("  ⚠ NO TRIANGLES GENERATED — file may not convert properly!");
                    }
                }
                Err(e) => { eprintln!("  convert_detailed_instances ERROR: {}", e); }
            }
        }
    }
}

// ============================================================
// Unit Tests for STEP Parser Extension (3.3.x)
// ============================================================

#[cfg(test)]
mod step_parser_extension_tests {
    use super::*;
    use crate::parse_step;

    /// Helper: create a minimal STEP file with the given DATA section content
    fn make_step(data: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('test'),'2;1');\n\
             FILE_NAME('test.stp','2024-01-01',(''),(''),'3Draper','','');\n\
             FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n{}\nENDSEC;\nEND-ISO-10303-21;",
            data
        )
    }

    // ─── 3.3.1 B_SPLINE_CURVE_WITH_KNOTS (any degree) ─────────

    #[test]
    fn test_bspline_curve_degree1_linear() {
        let step = make_step(
            "#1 = B_SPLINE_CURVE_WITH_KNOTS('',1,(#10,#11),.UNSPECIFIED.,.F.,.U.,\
             (2,2),(0.0,1.0),.UNSPECIFIED.);\n\
             #10 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #11 = CARTESIAN_POINT('',(10.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0).unwrap();
        let p0 = curve.point_at(0.0);
        let p1 = curve.point_at(1.0);
        assert!((p0.x - 0.0).abs() < 1e-6, "p0.x should be 0");
        assert!((p1.x - 10.0).abs() < 1e-6, "p1.x should be 10");
    }

    #[test]
    fn test_bspline_curve_degree2_quadratic() {
        let step = make_step(
            "#1 = B_SPLINE_CURVE_WITH_KNOTS('',2,(#10,#11,#12),.UNSPECIFIED.,.F.,.U.,\
             (3,3),(0.0,1.0),.UNSPECIFIED.);\n\
             #10 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #11 = CARTESIAN_POINT('',(5.0,10.0,0.0));\n\
             #12 = CARTESIAN_POINT('',(10.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0).unwrap();
        let p_mid = curve.point_at(0.5);
        assert!((p_mid.x - 5.0).abs() < 0.1, "midpoint x should be ~5, got {}", p_mid.x);
        assert!(p_mid.y > 0.0, "midpoint y should be positive");
    }

    #[test]
    fn test_bspline_curve_degree3_cubic() {
        let step = make_step(
            "#1 = B_SPLINE_CURVE_WITH_KNOTS('',3,(#10,#11,#12,#13),.UNSPECIFIED.,.F.,.U.,\
             (4,4),(0.0,1.0),.UNSPECIFIED.);\n\
             #10 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #11 = CARTESIAN_POINT('',(1.0,3.0,0.0));\n\
             #12 = CARTESIAN_POINT('',(4.0,3.0,0.0));\n\
             #13 = CARTESIAN_POINT('',(5.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0).unwrap();
        let p0 = curve.point_at(0.0);
        let p1 = curve.point_at(1.0);
        assert!((p0.x).abs() < 1e-6, "start x should be 0");
        assert!((p1.x - 5.0).abs() < 1e-6, "end x should be 5");
    }

    #[test]
    fn test_rational_bspline_curve() {
        let step = make_step(
            "#1 = (BOUNDED_CURVE()B_SPLINE_CURVE(2,(#10,#11,#12),.UNSPECIFIED.,.F.,.U.)\
             B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.0,1.0),.UNSPECIFIED.)\
             RATIONAL_B_SPLINE_CURVE((1.0,0.7071067811865476,1.0))REPRESENTATION_ITEM('')GEOMETRIC_REPRESENTATION_ITEM()CURVE());\n\
             #10 = CARTESIAN_POINT('',(1.0,0.0,0.0));\n\
             #11 = CARTESIAN_POINT('',(1.0,1.0,0.0));\n\
             #12 = CARTESIAN_POINT('',(0.0,1.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0).unwrap();
        if let Curve3d::Nurbs(nurbs) = &curve {
            assert_eq!(nurbs.degree, 2);
            assert_eq!(nurbs.control_points.len(), 3);
            assert_eq!(nurbs.weights.len(), 3);
            assert!((nurbs.weights[1] - 0.707).abs() < 0.01, "middle weight should be ~0.707, got {}", nurbs.weights[1]);
        } else {
            panic!("Expected NURBS curve");
        }
    }

    // ─── 3.3.3 COMPOSITE_CURVE_SEGMENT ─────────

    #[test]
    fn test_composite_curve_with_segments() {
        // Two ARC segments, each with a bounded param range
        let step = make_step(
            "#1 = COMPOSITE_CURVE('',(#2,#3),.U.);\n\
             #2 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#10,.T.);\n\
             #3 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#20,.T.);\n\
             #10 = CIRCLE('',#100,1.0);\n\
             #20 = CIRCLE('',#101,2.0);\n\
             #100 = AXIS2_PLACEMENT_3D('',#200,#300,#400);\n\
             #101 = AXIS2_PLACEMENT_3D('',#201,#301,#401);\n\
             #200 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #201 = CARTESIAN_POINT('',(3.0,0.0,0.0));\n\
             #300 = DIRECTION('',(0.0,0.0,1.0));\n\
             #301 = DIRECTION('',(0.0,0.0,1.0));\n\
             #400 = DIRECTION('',(1.0,0.0,0.0));\n\
             #401 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0);

        // The result might be Composite if 2 segments are resolved,
        // or it could be a single curve if one segment fails.
        // At minimum, it should resolve.
        assert!(curve.is_some(), "Should resolve composite curve");

        if let Some(Curve3d::Composite { segments, cum_lengths }) = &curve {
            assert_eq!(segments.len(), 2, "Should have 2 segments");
            assert_eq!(cum_lengths.len(), 2, "Should have 2 cumulative lengths");
            assert!((cum_lengths[1] - 1.0).abs() < 1e-10, "Last cum_length should be 1.0");

            // Verify point_at works
            let p0 = curve.as_ref().unwrap().point_at(0.0);
            assert!(p0.x.is_finite(), "Start point should be finite");
        }
    }

    #[test]
    fn test_composite_curve_param_range() {
        let step = make_step(
            "#1 = COMPOSITE_CURVE('',(#2,#3),.U.);\n\
             #2 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#10,.T.);\n\
             #3 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#20,.T.);\n\
             #10 = CIRCLE('',#100,1.0);\n\
             #20 = CIRCLE('',#101,2.0);\n\
             #100 = AXIS2_PLACEMENT_3D('',#200,#300,#400);\n\
             #101 = AXIS2_PLACEMENT_3D('',#201,#301,#401);\n\
             #200 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #201 = CARTESIAN_POINT('',(3.0,0.0,0.0));\n\
             #300 = DIRECTION('',(0.0,0.0,1.0));\n\
             #301 = DIRECTION('',(0.0,0.0,1.0));\n\
             #400 = DIRECTION('',(1.0,0.0,0.0));\n\
             #401 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0);

        if let Some(curve) = &curve {
            if let Curve3d::Composite { .. } = curve {
                let (t_min, t_max) = curve.param_range();
                assert!((t_min - 0.0).abs() < 1e-10, "t_min should be 0");
                assert!((t_max - 1.0).abs() < 1e-10, "t_max should be 1");
            }
        }
    }

    #[test]
    fn test_composite_curve_not_degenerate() {
        let step = make_step(
            "#1 = COMPOSITE_CURVE('',(#2,#3),.U.);\n\
             #2 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#10,.T.);\n\
             #3 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#20,.T.);\n\
             #10 = CIRCLE('',#100,1.0);\n\
             #20 = CIRCLE('',#101,2.0);\n\
             #100 = AXIS2_PLACEMENT_3D('',#200,#300,#400);\n\
             #101 = AXIS2_PLACEMENT_3D('',#201,#301,#401);\n\
             #200 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #201 = CARTESIAN_POINT('',(3.0,0.0,0.0));\n\
             #300 = DIRECTION('',(0.0,0.0,1.0));\n\
             #301 = DIRECTION('',(0.0,0.0,1.0));\n\
             #400 = DIRECTION('',(1.0,0.0,0.0));\n\
             #401 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0);

        if let Some(curve) = &curve {
            if let Curve3d::Composite { .. } = curve {
                assert!(!curve.is_degenerate(1e-6), "Composite of circles should not be degenerate");
            }
        }
    }

    #[test]
    fn test_composite_curve_segment_same_sense() {
        let step = make_step(
            "#1 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#10,.F.);\n\
             #10 = LINE('',#100,#200);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #200 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let entity = file.find_entity(1).unwrap();
        let same_sense = converter.extract_same_sense(entity);
        assert!(!same_sense, "same_sense should be false for .F.");
    }

    // ─── 3.3.4 OFFSET_CURVE_3D ─────────

    #[test]
    fn test_offset_curve_3d_parsing() {
        let step = make_step(
            "#1 = OFFSET_CURVE_3D('',#10,2.0,#200,.F.);\n\
             #10 = LINE('',#100,#201);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #201 = DIRECTION('',(1.0,0.0,0.0));\n\
             #200 = DIRECTION('',(0.0,0.0,1.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0);
        assert!(curve.is_some(), "Should resolve offset curve");
    }

    #[test]
    fn test_offset_curve_zero_distance() {
        let step = make_step(
            "#1 = OFFSET_CURVE_3D('',#10,0.0,#200,.F.);\n\
             #10 = LINE('',#100,#201);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #201 = DIRECTION('',(1.0,0.0,0.0));\n\
             #200 = DIRECTION('',(0.0,0.0,1.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0).unwrap();
        assert!(matches!(curve, Curve3d::Line(_)), "Zero offset should return the line");
    }

    // ─── 3.3.5 OFFSET_SURFACE ─────────

    // ─── 3.3 OFFSET_CURVE_2D ─────────

    #[test]
    fn test_offset_curve_2d_line_basis() {
        // OFFSET_CURVE_2D with a LINE basis curve — offset of a straight line
        // in UV space is another straight line (or Nurbs2d approximating it).
        let step = make_step(
            "#1 = OFFSET_CURVE_2D('',#10,0.5,.F.);\n\
             #10 = LINE('',#100,#201);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0));\n\
             #201 = DIRECTION('',(1.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve_2d(1);
        assert!(curve.is_some(), "Should resolve 2D offset curve from line basis");

        // The result should be a Nurbs2d (offset approximation)
        let c = curve.unwrap();
        assert!(matches!(c, Curve2d::Nurbs(_)), "Offset of line should be approximated as Nurbs2d");

        // Verify the offset is approximately 0.5 perpendicular to the line
        let p_start = c.point_at(0.0);
        let p_end = c.point_at(1.0);
        // Original line goes from (0,0) to (1,0) direction, offset 0.5 in +v
        assert!((p_start.v - 0.5).abs() < 0.1, "Start point v should be near 0.5, got {}", p_start.v);
        assert!((p_end.v - 0.5).abs() < 0.1, "End point v should be near 0.5, got {}", p_end.v);
    }

    #[test]
    fn test_offset_curve_2d_circle_basis() {
        // OFFSET_CURVE_2D with a CIRCLE basis — offset of a circle in UV.
        // For a CCW-oriented circle, the 2D offset normal points inward
        // (toward center), so a positive offset reduces the radius.
        let step = make_step(
            "#1 = OFFSET_CURVE_2D('',#10,0.2,.F.);\n\
             #10 = CIRCLE('',#100,1.0);\n\
             #100 = AXIS2_PLACEMENT_2D('',#101,#102);\n\
             #101 = CARTESIAN_POINT('',(2.0,3.0));\n\
             #102 = DIRECTION('',(1.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve_2d(1);
        assert!(curve.is_some(), "Should resolve 2D offset curve from circle basis");

        let c = curve.unwrap();
        // Positive offset on a CCW circle → radius decreases: 1.0 - 0.2 = 0.8
        for t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let p = c.point_at(*t);
            let dx = p.u - 2.0;
            let dy = p.v - 3.0;
            let r = (dx * dx + dy * dy).sqrt();
            assert!((r - 0.8).abs() < 0.15, "Point at t={} should be at radius ~0.8, got {:.4}", t, r);
        }
    }

    #[test]
    fn test_offset_curve_2d_zero_distance() {
        // Zero offset should return the basis curve as-is
        let step = make_step(
            "#1 = OFFSET_CURVE_2D('',#10,0.0,.F.);\n\
             #10 = LINE('',#100,#201);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0));\n\
             #201 = DIRECTION('',(1.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve_2d(1).unwrap();
        assert!(matches!(curve, Curve2d::Line(_)), "Zero offset should return the line as-is");
    }

    #[test]
    fn test_offset_curve_2d_negative_distance() {
        // Negative offset should offset in the opposite direction
        let step = make_step(
            "#1 = OFFSET_CURVE_2D('',#10,-0.5,.F.);\n\
             #10 = LINE('',#100,#201);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0));\n\
             #201 = DIRECTION('',(1.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve_2d(1);
        assert!(curve.is_some(), "Should resolve 2D offset curve with negative distance");

        let c = curve.unwrap();
        let p = c.point_at(0.5);
        // Negative offset: line goes +u direction, normal is +v for CCW,
        // so negative offset should go to -v
        assert!(p.v < 0.0, "Negative offset should be at negative v, got v={}", p.v);
    }

    // ─── 3.3.5 OFFSET_SURFACE ─────────

    #[test]
    fn test_offset_surface_parsing() {
        let step = make_step(
            "#1 = OFFSET_SURFACE('',#10,0.5,.T.);\n\
             #10 = PLANE('',#100);\n\
             #100 = AXIS2_PLACEMENT_3D('',#101,#102,#103);\n\
             #101 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #102 = DIRECTION('',(0.0,0.0,1.0));\n\
             #103 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let surface = converter.extract_surface(1, 0);
        assert!(surface.is_some(), "Should resolve offset surface");
    }

    // ─── 3.3.6 RECTANGULAR_TRIMMED_SURFACE ─────────

    #[test]
    fn test_rectangular_trimmed_surface_parsing() {
        let step = make_step(
            "#1 = RECTANGULAR_TRIMMED_SURFACE('',#10,0.0,6.28,0.0,1.0,.T.,.T.);\n\
             #10 = CYLINDRICAL_SURFACE('',#100,5.0);\n\
             #100 = AXIS2_PLACEMENT_3D('',#101,#102,#103);\n\
             #101 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #102 = DIRECTION('',(0.0,0.0,1.0));\n\
             #103 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let surface = converter.extract_surface(1, 0);
        assert!(surface.is_some(), "Should resolve trimmed surface");
        assert!(matches!(surface.unwrap(), Surface::Cylinder(_)), "Should return basis cylinder");
    }

    // ─── 3.3.7 SURFACE_OF_REVOLUTION ─────────

    #[test]
    fn test_revolution_surface_arbitrary_axis() {
        let step = make_step(
            "#1 = SURFACE_OF_REVOLUTION('',#10,#100);\n\
             #10 = LINE('',#200,#201);\n\
             #200 = CARTESIAN_POINT('',(5.0,0.0,0.0));\n\
             #201 = DIRECTION('',(0.0,0.0,1.0));\n\
             #100 = AXIS1_PLACEMENT('',#101,#102);\n\
             #101 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #102 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let surface = converter.extract_surface(1, 0);
        assert!(surface.is_some(), "Should resolve revolution surface with arbitrary axis");
    }

    // ─── 3.3.8 SURFACE_OF_LINEAR_EXTRUSION ─────────

    #[test]
    fn test_extrusion_with_vector() {
        let step = make_step(
            "#1 = SURFACE_OF_LINEAR_EXTRUSION('',#10,#200);\n\
             #10 = LINE('',#100,#201);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #201 = DIRECTION('',(1.0,0.0,0.0));\n\
             #200 = VECTOR('',#202,10.0);\n\
             #202 = DIRECTION('',(0.0,0.0,1.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let surface = converter.extract_surface(1, 0);
        assert!(surface.is_some(), "Should resolve extrusion with VECTOR");
    }

    // ─── 3.3.9 COMPOSITE_CURVE_ON_SURFACE ─────────

    #[test]
    fn test_composite_curve_on_surface() {
        let step = make_step(
            "#1 = COMPOSITE_CURVE_ON_SURFACE('',(#2,#3),.U.,#50);\n\
             #2 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#10,.T.);\n\
             #3 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,#20,.T.);\n\
             #10 = LINE('',#100,#200);\n\
             #20 = LINE('',#101,#201);\n\
             #50 = PLANE('',#500);\n\
             #100 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #101 = CARTESIAN_POINT('',(5.0,0.0,0.0));\n\
             #200 = DIRECTION('',(1.0,0.0,0.0));\n\
             #201 = DIRECTION('',(0.0,1.0,0.0));\n\
             #500 = AXIS2_PLACEMENT_3D('',#501,#502,#503);\n\
             #501 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #502 = DIRECTION('',(0.0,0.0,1.0));\n\
             #503 = DIRECTION('',(1.0,0.0,0.0));"
        );
        let file = parse_step(&step).unwrap();
        let converter = StepConverter::new(&file);
        let curve = converter.resolve_curve(1, 0);
        assert!(curve.is_some(), "Should resolve COMPOSITE_CURVE_ON_SURFACE");
    }

    // ─── NURBS approximation helpers ─────────

    #[test]
    fn test_nurbs_curve_interpolation() {
        let points = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 2.0, 0.0),
            Point3d::new(2.0, 2.0, 0.0),
            Point3d::new(3.0, 0.0, 0.0),
        ];
        let nurbs = fit_nurbs_curve_through_points(&points, 3);
        assert!(nurbs.is_some(), "Should fit a degree-3 curve through 4 points");
        let nurbs = nurbs.unwrap();
        assert_eq!(nurbs.degree, 3);
        assert_eq!(nurbs.control_points.len(), 4);

        let p0 = Curve3d::Nurbs(nurbs.clone()).point_at(0.0);
        let p3 = Curve3d::Nurbs(nurbs.clone()).point_at(1.0);
        assert!((p0.x - 0.0).abs() < 1e-4, "First point x should be 0");
        assert!((p3.x - 3.0).abs() < 1e-4, "Last point x should be 3");
    }

    #[test]
    fn test_offset_circle_curve() {
        // Offset a circle by -1.0 (outward) — should produce a circle with radius+1
        // Note: the Frenet principal normal points inward (toward center of curvature),
        // so a negative offset distance offsets outward.
        let circle = Circle::new_xy(Point3d::ORIGIN, 5.0);
        let basis = Curve3d::Circle(circle);
        let offset = approximate_offset_curve(&basis, -1.0, None);
        // The offset curve should be approximately a circle of radius 6
        // Check the point at t=0 (which should be at (6, 0, 0) for outward offset)
        let p_start = offset.point_at(0.0);
        assert!(p_start.x > 5.0, "Offset circle start x should be > 5, got {}", p_start.x);
    }

    #[test]
    fn test_offset_surface_plane() {
        let plane = Plane::xy();
        let offset = approximate_offset_surface(&Surface::Plane(plane), 1.0);
        if let Surface::Nurbs(nurbs) = &offset {
            assert!(nurbs.u_degree >= 1);
            assert!(nurbs.v_degree >= 1);
            let (u_min, u_max) = nurbs.u_range();
            let (v_min, v_max) = nurbs.v_range();
            let p_mid = offset.point_at((u_min + u_max) / 2.0, (v_min + v_max) / 2.0);
            assert!((p_mid.z - 1.0).abs() < 0.1, "Offset plane z should be ~1.0, got {}", p_mid.z);
        } else {
            panic!("Expected NURBS surface for offset plane");
        }
    }

    #[test]
    fn test_generate_clamped_knots() {
        let knots = generate_clamped_knots(6, 3);
        assert_eq!(knots.len(), 10);
        for i in 0..=3 {
            assert!((knots[i] - 0.0).abs() < 1e-10, "First 4 knots should be 0");
        }
        for i in 6..=9 {
            assert!((knots[i] - 1.0).abs() < 1e-10, "Last 4 knots should be 1");
        }
        for i in 1..knots.len() {
            assert!(knots[i] >= knots[i-1] - 1e-10, "Knots should be non-decreasing");
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Regression tests for the three Zentralstaender.stp bugs:
    //   1. NAUO name extraction returned whitespace " " instead of the
    //      descriptive name in the 3rd parameter (description field).
    //   2. get_product_name() failed to follow PD → PDF → PRODUCT chain
    //      when the PDF entity was PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE
    //      (only the shorter PRODUCT_DEFINITION_FORMATION was recognized).
    //   3. OwnedStepConversionContext::new() always used TriangulationParams::default(),
    //      ignoring the user-selected LOD — so the "Quality" selector in the
    //      viewer had no visible effect on vertex/triangle counts.
    // ─────────────────────────────────────────────────────────────────────

    fn load_zentralstaender() -> StepFile {
        let path = "/home/z/my-project/test/Zentralstaender.stp";
        let content = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("read {}: {}", path, e));
        parse_step(&content).expect("parse step file")
    }

    /// Root assembly name should be the PRODUCT name (`_hebevorrichtung_wwk-017846-kon-a`),
    /// not the placeholder `PD#306`.
    #[test]
    fn test_zentralstaender_root_name() {
        let step = load_zentralstaender();
        let (tree, _pending) = step_structure_lazy(&step);
        assert_eq!(
            tree.name, "_hebevorrichtung_wwk-017846-kon-a",
            "Root node name should be the PRODUCT name, not PD#{{id}}"
        );
    }

    /// Every pending instance should have a non-blank name (the descriptive
    /// NAUO description field, not the whitespace-only name field).
    #[test]
    fn test_zentralstaender_instance_names_nonempty() {
        let step = load_zentralstaender();
        let (_tree, pending) = step_structure_lazy(&step);
        assert!(pending.len() >= 30, "expected ~34 instances, got {}", pending.len());
        for (i, p) in pending.iter().enumerate() {
            let trimmed = p.name.trim();
            assert!(
                !trimmed.is_empty() && !trimmed.starts_with("BREP#") && !trimmed.starts_with("NAUO#"),
                "instance[{}] name is blank/placeholder: {:?}",
                i, p.name
            );
        }
    }

    /// Specific instance names that should appear in the tree (sampled from
    /// the file's NAUO description fields).
    #[test]
    fn test_zentralstaender_known_instance_names_present() {
        let step = load_zentralstaender();
        let (_tree, pending) = step_structure_lazy(&step);
        let names: Vec<String> = pending.iter().map(|p| p.name.clone()).collect();
        let joined = names.join("\n");
        // These are descriptive part names from the STEP file's NAUO entries.
        for expected in [
            "BEVORRICHTUNG_WWK-017847-KON_A",
            "B_PROFILROHR_WWK-017849-KON-A",
            "B_EINHAENGUNG_WWK-017851-KON-A",
            "B_LASCHE_WWK-017852-KON-A",
            "B_HEBEL_WWK-017854-KON-A",
            "CHWEISSPLATTE_WWK-017855-KON-A",
            "DAPPTERPLATTE_WWK-017856-KON-A",
            "B_WELLE_WWK-017857-KON-A",
            "ENTRIERSTUECK_WWK-017858-KON-A",
            "TEIFUNGSBLECH_WWK-017860-KON-A",
            "DISTANZBUCHSE_WWK-017864-KON-A",
            "SPEZIALMUTTER_WWK-017873-KON-A",
            "TRANSPORTROLLE",
            "WINKEL",
        ] {
            assert!(
                joined.contains(expected),
                "expected instance name {:?} not found in pending list",
                expected
            );
        }
    }

    /// LOD should actually affect triangulation: lower LOD → fewer triangles.
    /// This is a regression test for the bug where `OwnedStepConversionContext::new()`
    /// ignored the LOD selector and always used `TriangulationParams::default()`.
    #[test]
    fn test_lod_actually_changes_triangle_count() {
        let step = load_zentralstaender();
        let (_tree, pending) = step_structure_lazy(&step);
        assert!(pending.len() >= 30);

        let coarse = TriangulationParams::for_lod(0.1);
        let fine = TriangulationParams::for_lod(1.0);
        // Sanity: the two param sets must actually differ, otherwise the
        // test would silently pass even if the bug regressed.
        assert!(
            fine.max_deviation < coarse.max_deviation,
            "fine LOD should have smaller max_deviation (fine={}, coarse={})",
            fine.max_deviation, coarse.max_deviation
        );

        let count_tris = |params: TriangulationParams| -> usize {
            let mut ctx = OwnedStepConversionContext::new_with_params(step.clone(), params);
            let mut total = 0usize;
            for p in &pending {
                if let Some(inst) = ctx.triangulate_pending(p) {
                    total += inst.mesh.triangle_count();
                }
            }
            total
        };

        let coarse_tris = count_tris(coarse);
        let fine_tris = count_tris(fine);

        assert!(
            fine_tris > coarse_tris,
            "fine LOD should produce MORE triangles than coarse (fine={}, coarse={}) \
             — if this fails, LOD is being ignored again",
            fine_tris, coarse_tris
        );
        // With post-triangulation decimation enabled, the difference is
        // substantial: ~12500 fine vs ~2500 coarse (5× ratio) on Zentralstaender.stp.
        // Require at least 3000 triangle difference — well below the empirical
        // ~10000 difference, but high enough to catch regressions where
        // decimation is accidentally disabled.
        let diff = fine_tris.saturating_sub(coarse_tris);
        assert!(
            diff >= 3000,
            "expected at least 3000 triangle difference between coarse and fine LOD, got {} \
             (fine={}, coarse={}) — decimation may be disabled or keep_ratio curve is wrong",
            diff, fine_tris, coarse_tris
        );
        // Also require at least a 2× ratio —Preview should be drastically coarser than Ultra.
        let ratio = fine_tris as f64 / coarse_tris.max(1) as f64;
        assert!(
            ratio >= 2.0,
            "expected fine/coarse triangle ratio ≥ 2.0, got {:.2} (fine={}, coarse={})",
            ratio, fine_tris, coarse_tris
        );
    }

    /// `set_params()` should clear the BREP cache so re-triangulating with
    /// different params gives different results. We scan the pending list for
    /// a BREP whose triangle count actually differs between LODs (some BREPs
    /// in this file are simple boxes that produce identical counts at any LOD).
    #[test]
    fn test_set_params_clears_cache() {
        let step = load_zentralstaender();
        let (_tree, pending) = step_structure_lazy(&step);

        // Find a BREP that is non-trivial enough to show LOD-dependent counts.
        // We use a unique set of brep_ids because the cache is keyed by brep_id
        // and a repeated instance wouldn't exercise the re-triangulation path.
        let mut seen: std::collections::HashSet<i64> = std::collections::HashSet::new();
        let mut target: Option<&PendingBrepInstance> = None;
        for p in &pending {
            if seen.insert(p.brep_id) {
                target = Some(p);
                // Use the first unique BREP — they all have ≥32 tris, but
                // we verify below that the count actually differs.
                break;
            }
        }
        let target = target.expect("at least one pending instance");

        let mut ctx = OwnedStepConversionContext::new_with_params(
            step.clone(),
            TriangulationParams::for_lod(1.0),
        );
        let fine_mesh = ctx.triangulate_pending(target).expect("triangulate at fine LOD");
        let fine_tris = fine_mesh.mesh.triangle_count();

        // Switch to coarse LOD and triangulate the same BREP again.
        ctx.set_params(TriangulationParams::for_lod(0.1));
        let coarse_mesh = ctx.triangulate_pending(target).expect("triangulate at coarse LOD");
        let coarse_tris = coarse_mesh.mesh.triangle_count();

        // The total triangle difference on this file is ~1000 across 27 unique
        // BREPs, so on average each BREP differs by ~37 tris. Some individual
        // BREPs (especially cylinders/cones) differ much more. If the picked
        // BREP happens to be one of the few that doesn't differ (pure planar
        // box), iterate to find one that does.
        if fine_tris == coarse_tris {
            // Find any unique BREP whose count actually differs.
            let mut found_diff = false;
            for p in &pending {
                if !seen.insert(p.brep_id) {
                    continue;
                }
                let mut ctx2 = OwnedStepConversionContext::new_with_params(
                    step.clone(),
                    TriangulationParams::for_lod(1.0),
                );
                let f = ctx2.triangulate_pending(p).expect("triangulate").mesh.triangle_count();
                ctx2.set_params(TriangulationParams::for_lod(0.1));
                let c = ctx2.triangulate_pending(p).expect("triangulate").mesh.triangle_count();
                if f != c {
                    found_diff = true;
                    break;
                }
            }
            assert!(
                found_diff,
                "set_params() did not invalidate the cache OR no BREP in Zentralstaender.stp \
                 shows LOD-dependent triangle counts (highly unlikely — the aggregate difference \
                 is ~1000 tris). fine={} coarse={} on BREP#{}",
                fine_tris, coarse_tris, target.brep_id
            );
        }
    }
}

// ============================================================
// Parallel BREP triangulation tests (native only)
// ============================================================
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod parallel_brep_tests {
    use super::*;
    use crate::parse_step;

    /// Create a minimal STEP file with a single box (MANIFOLD_SOLID_BREP).
    fn make_box_step(offset: i64) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('test'),'2;1');\n\
             FILE_NAME('test.stp','2024-01-01',(''),(''),'3Draper','','');\n\
             FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n\
             #{o}1 = MANIFOLD_SOLID_BREP('',#{o}2);\n\
             #{o}2 = CLOSED_SHELL('',(#{o}3,#{o}4,#{o}5,#{o}6,#{o}7,#{o}8));\n\
             #{o}3 = ADVANCED_FACE('',(#{o}9),#{o}10,.T.);\n\
             #{o}4 = ADVANCED_FACE('',(#{o}11),#{o}12,.T.);\n\
             #{o}5 = ADVANCED_FACE('',(#{o}13),#{o}14,.T.);\n\
             #{o}6 = ADVANCED_FACE('',(#{o}15),#{o}16,.T.);\n\
             #{o}7 = ADVANCED_FACE('',(#{o}17),#{o}18,.T.);\n\
             #{o}8 = ADVANCED_FACE('',(#{o}19),#{o}20,.T.);\n\
             #{o}9 = EDGE_LOOP('',(#{o}21));\n\
             #{o}10 = PLANE('',#{o}22);\n\
             #{o}11 = EDGE_LOOP('',(#{o}23));\n\
             #{o}12 = PLANE('',#{o}24);\n\
             #{o}13 = EDGE_LOOP('',(#{o}25));\n\
             #{o}14 = PLANE('',#{o}26);\n\
             #{o}15 = EDGE_LOOP('',(#{o}27));\n\
             #{o}16 = PLANE('',#{o}28);\n\
             #{o}17 = EDGE_LOOP('',(#{o}29));\n\
             #{o}18 = PLANE('',#{o}30);\n\
             #{o}19 = EDGE_LOOP('',(#{o}31));\n\
             #{o}20 = PLANE('',#{o}32);\n\
             #{o}21 = ORIENTED_EDGE('',*,*,#{o}33,.T.);\n\
             #{o}22 = AXIS2_PLACEMENT_3D('',#{o}34,$,$);\n\
             #{o}33 = EDGE_CURVE('',#{o}35,#{o}35,#{o}36,.T.);\n\
             #{o}34 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #{o}35 = VERTEX_POINT('',#{o}37);\n\
             #{o}36 = LINE('',#{o}38,#{o}39);\n\
             #{o}37 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #{o}38 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #{o}39 = VECTOR('',#{o}40,1.0);\n\
             #{o}40 = DIRECTION('',(1.0,0.0,0.0));\n\
             #{o}23 = EDGE_LOOP('',(#{o}41));\n\
             #{o}24 = AXIS2_PLACEMENT_3D('',#{o}42,$,$);\n\
             #{o}41 = ORIENTED_EDGE('',*,*,#{o}43,.T.);\n\
             #{o}42 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #{o}43 = EDGE_CURVE('',#{o}35,#{o}35,#{o}44,.T.);\n\
             #{o}44 = LINE('',#{o}45,#{o}46);\n\
             #{o}45 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #{o}46 = VECTOR('',#{o}47,1.0);\n\
             #{o}47 = DIRECTION('',(0.0,1.0,0.0));\n\
             #{o}25 = EDGE_LOOP('',(#{o}48));\n\
             #{o}26 = AXIS2_PLACEMENT_3D('',#{o}49,$,$);\n\
             #{o}48 = ORIENTED_EDGE('',*,*,#{o}50,.T.);\n\
             #{o}49 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #{o}50 = EDGE_CURVE('',#{o}35,#{o}35,#{o}51,.T.);\n\
             #{o}51 = LINE('',#{o}52,#{o}53);\n\
             #{o}52 = CARTESIAN_POINT('',(0.0,0.0,0.0));\n\
             #{o}53 = VECTOR('',#{o}54,1.0);\n\
             #{o}54 = DIRECTION('',(0.0,0.0,1.0));\n\
             #{o}27 = EDGE_LOOP('',(#{o}55));\n\
             #{o}28 = AXIS2_PLACEMENT_3D('',#{o}56,$,$);\n\
             #{o}55 = ORIENTED_EDGE('',*,*,#{o}57,.T.);\n\
             #{o}56 = CARTESIAN_POINT('',(1.0,0.0,0.0));\n\
             #{o}57 = EDGE_CURVE('',#{o}35,#{o}35,#{o}58,.T.);\n\
             #{o}58 = LINE('',#{o}59,#{o}60);\n\
             #{o}59 = CARTESIAN_POINT('',(1.0,0.0,0.0));\n\
             #{o}60 = VECTOR('',#{o}61,1.0);\n\
             #{o}61 = DIRECTION('',(0.0,1.0,0.0));\n\
             #{o}29 = EDGE_LOOP('',(#{o}62));\n\
             #{o}30 = AXIS2_PLACEMENT_3D('',#{o}63,$,$);\n\
             #{o}62 = ORIENTED_EDGE('',*,*,#{o}64,.T.);\n\
             #{o}63 = CARTESIAN_POINT('',(0.0,1.0,0.0));\n\
             #{o}64 = EDGE_CURVE('',#{o}35,#{o}35,#{o}65,.T.);\n\
             #{o}65 = LINE('',#{o}66,#{o}67);\n\
             #{o}66 = CARTESIAN_POINT('',(0.0,1.0,0.0));\n\
             #{o}67 = VECTOR('',#{o}68,1.0);\n\
             #{o}68 = DIRECTION('',(1.0,0.0,0.0));\n\
             #{o}31 = EDGE_LOOP('',(#{o}69));\n\
             #{o}32 = AXIS2_PLACEMENT_3D('',#{o}70,$,$);\n\
             #{o}69 = ORIENTED_EDGE('',*,*,#{o}71,.T.);\n\
             #{o}70 = CARTESIAN_POINT('',(0.0,0.0,1.0));\n\
             #{o}71 = EDGE_CURVE('',#{o}35,#{o}35,#{o}72,.T.);\n\
             #{o}72 = LINE('',#{o}73,#{o}74);\n\
             #{o}73 = CARTESIAN_POINT('',(0.0,0.0,1.0));\n\
             #{o}74 = VECTOR('',#{o}75,1.0);\n\
             #{o}75 = DIRECTION('',(1.0,0.0,0.0));\n\
             ENDSEC;\nEND-ISO-10303-21;",
            o = offset
        )
    }

    #[test]
    fn test_parallel_empty_input() {
        // Empty input should return empty results immediately.
        let step_content = make_box_step(0);
        let step_file = parse_step(&step_content).unwrap();
        let mut ctx = OwnedStepConversionContext::new(step_file);
        let results = ctx.triangulate_breps_parallel(
            &[],
            || false,
            |_, _| {},
        );
        assert!(results.is_empty());
    }

    #[test]
    fn test_parallel_cancel_flag_respected() {
        // When cancel_flag returns true immediately, no BREPs should be processed.
        let step_content = make_box_step(0);
        let step_file = parse_step(&step_content).unwrap();
        let mut ctx = OwnedStepConversionContext::new(step_file);
        let pending = vec![PendingBrepInstance {
            name: "test".to_string(),
            brep_id: 1,
            transform: None,
            color: None,
            face_count_estimate: None,
        }];
        let results = ctx.triangulate_breps_parallel(
            &pending,
            || true, // cancel immediately
            |_, _| {},
        );
        // All results should be None (cancelled)
        assert!(results.iter().all(|r| r.is_none()));
    }

    #[test]
    fn test_parallel_progress_callback_no_panic() {
        // The progress callback should not cause panics.
        let step_content = make_box_step(0);
        let step_file = parse_step(&step_content).unwrap();
        let mut ctx = OwnedStepConversionContext::new(step_file);

        let pending = vec![PendingBrepInstance {
            name: "test".to_string(),
            brep_id: 1,
            transform: None,
            color: None,
            face_count_estimate: None,
        }];
        let _results = ctx.triangulate_breps_parallel(
            &pending,
            || false,
            |_done, _total| {
                // Just ensure no panic
            },
        );
    }

    #[test]
    fn test_parallel_results_order_preserved() {
        // Results should be returned in the same order as input.
        // Even if BREPs complete in different order, the result indices
        // should match the input indices.
        let step_content = make_box_step(0);
        let step_file = parse_step(&step_content).unwrap();
        let mut ctx = OwnedStepConversionContext::new(step_file);

        let pending = vec![
            PendingBrepInstance {
                name: "first".to_string(),
                brep_id: 1,
                transform: None,
                color: None,
                face_count_estimate: None,
            },
            PendingBrepInstance {
                name: "second".to_string(),
                brep_id: 1, // same brep_id — should hit cache on second
                transform: None,
                color: None,
                face_count_estimate: None,
            },
        ];
        let results = ctx.triangulate_breps_parallel(
            &pending,
            || false,
            |_, _| {},
        );

        // Both should succeed or both should fail, but they should have
        // matching names in the correct order.
        assert_eq!(results.len(), 2);
        if let Some(ref inst) = results[0] {
            assert_eq!(inst.name, "first");
        }
        if let Some(ref inst) = results[1] {
            assert_eq!(inst.name, "second");
        }
    }
}

/// Unit tests for BREP_WITH_VOIDS support (6.3).
#[cfg(test)]
mod brep_with_voids_tests {
    use super::*;

    /// Parse a minimal STEP content string into a StepFile.
    fn parse_step_content(content: &str) -> Option<StepFile> {
        crate::parser::parse_step(content).ok()
    }

    /// Test that find_all_shell_refs correctly identifies the outer shell
    /// and void shells in a BREP_WITH_VOIDS entity.
    #[test]
    fn test_find_all_shell_refs_brep_with_voids() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test BREP_WITH_VOIDS'),'2;1');
FILE_NAME('test.stp','2026-07-02',('3Draper'),(''),'Test','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.0,0.0,0.0));
#2 = DIRECTION('',(1.0,0.0,0.0));
#3 = DIRECTION('',(0.0,1.0,0.0));
#4 = DIRECTION('',(0.0,0.0,1.0));
#10 = AXIS2_PLACEMENT_3D('',#1,$,$);
#20 = PLANE('',#10);
#30 = ADVANCED_FACE('',(#40),#20,.T.);
#40 = FACE_OUTER_BOUND('',#50,.T.);
#50 = EDGE_LOOP('',(#60));
#60 = ORIENTED_EDGE('',*,*,#70,.T.);
#70 = EDGE_CURVE('',#80,#80,#90,.T.);
#80 = VERTEX_POINT('',#1);
#90 = CIRCLE('',#10,5.0);
#100 = CLOSED_SHELL('',(#30));
#110 = CLOSED_SHELL('',(#30));
#200 = BREP_WITH_VOIDS('test',#100,(#110));
ENDSEC;
END-ISO-10303-21;
"#;
        let step_file = parse_step_content(content).expect("parse STEP");
        let converter = StepConverter::new(&step_file);

        // Find the BREP_WITH_VOIDS entity
        let brep_entity = step_file.find_entity(200).expect("BREP entity #200");
        assert_eq!(brep_entity.type_name, "BREP_WITH_VOIDS");

        let (outer, voids) = converter.find_all_shell_refs(&brep_entity);
        assert!(outer.is_some(), "Should find outer shell reference");
        assert_eq!(voids.len(), 1, "Should find 1 void shell reference");
    }

    /// Test that find_all_shell_refs returns empty voids for MANIFOLD_SOLID_BREP.
    #[test]
    fn test_find_all_shell_refs_manifold_solid_brep() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test'),'2;1');
FILE_NAME('test.stp','2026-07-02',('3Draper'),(''),'Test','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.0,0.0,0.0));
#10 = AXIS2_PLACEMENT_3D('',#1,$,$);
#20 = PLANE('',#10);
#30 = ADVANCED_FACE('',(#40),#20,.T.);
#40 = FACE_OUTER_BOUND('',#50,.T.);
#50 = EDGE_LOOP('',(#60));
#60 = ORIENTED_EDGE('',*,*,#70,.T.);
#70 = EDGE_CURVE('',#80,#80,#90,.T.);
#80 = VERTEX_POINT('',#1);
#90 = CIRCLE('',#10,5.0);
#100 = CLOSED_SHELL('',(#30));
#200 = MANIFOLD_SOLID_BREP('test',#100);
ENDSEC;
END-ISO-10303-21;
"#;
        let step_file = parse_step_content(content).expect("parse STEP");
        let converter = StepConverter::new(&step_file);

        let brep_entity = step_file.find_entity(200).expect("BREP entity #200");
        assert_eq!(brep_entity.type_name, "MANIFOLD_SOLID_BREP");

        let (outer, voids) = converter.find_all_shell_refs(&brep_entity);
        assert!(outer.is_some(), "Should find outer shell reference");
        assert!(voids.is_empty(), "MANIFOLD_SOLID_BREP should have no void shells");
    }

    /// Test that ORIENTED_CLOSED_SHELL with .F. orientation is correctly handled.
    /// When a void shell is wrapped in ORIENTED_CLOSED_SHELL with orientation .F.,
    /// the face forward flags should be flipped.
    #[test]
    fn test_oriented_closed_shell_with_false_orientation() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test ORIENTED_CLOSED_SHELL'),'2;1');
FILE_NAME('test.stp','2026-07-02',('3Draper'),(''),'Test','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.0,0.0,0.0));
#10 = AXIS2_PLACEMENT_3D('',#1,$,$);
#20 = PLANE('',#10);
#30 = ADVANCED_FACE('',(#40),#20,.T.);
#40 = FACE_OUTER_BOUND('',#50,.T.);
#50 = EDGE_LOOP('',(#60));
#60 = ORIENTED_EDGE('',*,*,#70,.T.);
#70 = EDGE_CURVE('',#80,#80,#90,.T.);
#80 = VERTEX_POINT('',#1);
#90 = CIRCLE('',#10,5.0);
#100 = CLOSED_SHELL('',(#30));
/* ORIENTED_CLOSED_SHELL wrapping #100 with .F. orientation */
#110 = ORIENTED_CLOSED_SHELL('',#100,.F.);
#200 = BREP_WITH_VOIDS('test',#100,(#110));
ENDSEC;
END-ISO-10303-21;
"#;
        let step_file = parse_step_content(content).expect("parse STEP");
        let converter = StepConverter::new(&step_file);

        // Find the BREP_WITH_VOIDS entity
        let brep_entity = step_file.find_entity(200).expect("BREP entity #200");
        let (outer, voids) = converter.find_all_shell_refs(&brep_entity);
        assert!(outer.is_some(), "Should find outer shell reference");
        assert_eq!(voids.len(), 1, "Should find 1 void shell reference (ORIENTED_CLOSED_SHELL)");

        // The void shell is #110 (ORIENTED_CLOSED_SHELL)
        let void_shell_id = voids[0];
        assert_eq!(void_shell_id, 110);

        // Extract faces from void shell with is_void=true
        let void_faces = converter.extract_shell_faces(void_shell_id, true);
        assert!(void_faces.is_some(), "Should extract faces from ORIENTED_CLOSED_SHELL");

        let void_faces = void_faces.unwrap();
        // The face inside CLOSED_SHELL #100 had forward=true from ADVANCED_FACE.
        // ORIENTED_CLOSED_SHELL with .F. should flip it to forward=false.
        for face in &void_faces {
            assert!(!face.forward, "Void face should have flipped forward flag due to ORIENTED_CLOSED_SHELL .F.");
            assert!(face.is_void, "Void face should have is_void=true");
        }
    }

    /// Test that face_data_list_to_solid correctly separates void faces into inner_shells.
    #[test]
    fn test_face_data_list_to_solid_separates_voids() {
        // Create a minimal FaceData list with both outer and void faces
        let outer_face = FaceData {
            surface: Surface::Plane(Plane::from_origin_and_normal(
                Point3d::ORIGIN, Direction3d::Z)),
            outer_edges: vec![],
            inner_edges: vec![],
            edges: vec![],
            forward: true,
            step_face_id: 1,
            surface_step_id: None,
            edge_curves_2d: vec![],
            edge_step_ids: vec![],
            outer_edge_step_ids: vec![],
            inner_edge_step_ids: vec![],
            is_void: false,
        };

        let void_face = FaceData {
            surface: Surface::Plane(Plane::from_origin_and_normal(
                Point3d::new(5.0, 5.0, 5.0), Direction3d::Z)),
            outer_edges: vec![],
            inner_edges: vec![],
            edges: vec![],
            forward: false, // Void faces have reversed orientation
            step_face_id: 2,
            surface_step_id: None,
            edge_curves_2d: vec![],
            edge_step_ids: vec![],
            outer_edge_step_ids: vec![],
            inner_edge_step_ids: vec![],
            is_void: true,
        };

        let face_data_list = vec![outer_face, void_face];
        let (solid, face_id_map) = face_data_list_to_solid(&face_data_list);

        // Outer shell should have 1 face
        assert!(solid.outer_shell.is_some());
        assert_eq!(solid.outer_shell.as_ref().unwrap().faces.len(), 1,
            "Outer shell should have 1 face (non-void)");

        // Inner shells should have 1 face
        assert_eq!(solid.inner_shells.len(), 1, "Should have 1 inner shell (void)");
        assert_eq!(solid.inner_shells[0].faces.len(), 1,
            "Inner shell should have 1 face (void)");

        // Face ID map should have 2 entries
        assert_eq!(face_id_map.len(), 2, "Face ID map should have 2 entries");
    }

    /// Test that FaceData.is_void field defaults to false and survives cloning.
    #[test]
    fn test_face_data_is_void_field() {
        let fd = FaceData {
            surface: Surface::Plane(Plane::from_origin_and_normal(
                Point3d::ORIGIN, Direction3d::Z)),
            outer_edges: vec![],
            inner_edges: vec![],
            edges: vec![],
            forward: true,
            step_face_id: 1,
            surface_step_id: None,
            edge_curves_2d: vec![],
            edge_step_ids: vec![],
            outer_edge_step_ids: vec![],
            inner_edge_step_ids: vec![],
            is_void: true,
        };

        assert!(fd.is_void);
        let cloned = fd.clone();
        assert!(cloned.is_void, "is_void should survive cloning");
    }
}
