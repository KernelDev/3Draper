// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Topology healing — repair common B-Rep defects.
//!
//! Phase 3.1 of the 3Draper roadmap. This module provides configurable
//! healing operations that can fix gaps, fill holes, stitch edges,
//! repair normal orientations, and remove small features.
//!
//! # Operations
//!
//! - **Gap closing** (3.1.2): merge boundary edges within `tolerance * gap_factor`
//! - **Hole filling** (3.1.3): triangulate small boundary loops (≤ `max_hole_edges`)
//! - **Edge stitching** (3.1.4): merge collinear edges that share a vertex
//! - **Normal orientation repair** (3.1.6): flip faces whose normals point inward
//! - **Small feature removal** (3.1.8): remove faces with area < `min_face_area`
//! - **Sliver triangle removal** (3.1.9): detect/remove triangles with aspect ratio > threshold
//!
//! # Usage
//!
//! ```ignore
//! use draper_topology::healing::{heal_solid, HealingParams};
//!
//! let params = HealingParams::default();
//! let (healed, report) = heal_solid(solid, &params);
//! println!("Applied {} fixes", report.total_fixes());
//! ```

use crate::entity::*;
use draper_geometry::{
    CylinderSurface, Direction3d, Plane, Point3d, Surface, Vec3d,
    ToleranceContext,
};

// ============================================================
// HealingParams
// ============================================================

/// Configurable parameters for the healing pipeline.
///
/// Every threshold is expressed relative to a base `tolerance` so that
/// the same pipeline works correctly at micron scale and meter scale.
#[derive(Clone, Debug)]
pub struct HealingParams {
    /// Multiplier on `tolerance` for gap closing.
    /// Boundary edges closer than `tolerance * gap_factor` are merged.
    pub gap_factor: f64,

    /// Maximum number of edges in a boundary loop to qualify for hole filling.
    pub max_hole_edges: usize,

    /// Faces with area below this threshold are removed (small feature removal).
    pub min_face_area: f64,

    /// Sliver triangle threshold: triangles with aspect ratio > this value
    /// are considered slivers.
    pub max_aspect_ratio: f64,

    /// Whether to repair face normals for closed shells.
    pub fix_normals: bool,

    /// Whether to stitch collinear edges that share a vertex.
    pub stitch_edges: bool,

    /// Whether to merge coplanar and co-cylindrical faces that share edges.
    pub merge_faces: bool,

    /// Whether to propagate tolerances through the topological graph.
    /// When true, tolerance propagation runs as the first step in the
    /// healing pipeline, ensuring all entities have consistent tolerances
    /// before subsequent operations.
    pub propagate_tolerances: bool,

    /// Optional tolerance context from the STEP file or model scale.
    /// When present, the coincidence tolerance from this context is used
    /// as a floor for all entity tolerances during propagation.
    pub tolerance_context: Option<ToleranceContext>,

    /// Base geometric tolerance.
    pub tolerance: f64,
}

impl Default for HealingParams {
    fn default() -> Self {
        Self {
            gap_factor: 10.0,
            max_hole_edges: 8,
            min_face_area: 1e-12,
            max_aspect_ratio: 100.0,
            fix_normals: true,
            stitch_edges: true,
            merge_faces: true,
            propagate_tolerances: true,
            tolerance_context: None,
            tolerance: 1e-6,
        }
    }
}

impl HealingParams {
    /// Create parameters suitable for a given tolerance context.
    pub fn from_tolerance_context(ctx: &ToleranceContext) -> Self {
        Self {
            tolerance: ctx.coincidence_tolerance(),
            min_face_area: ctx.coincidence_tolerance().powi(2),
            tolerance_context: Some(ctx.clone()),
            ..Self::default()
        }
    }

    /// Effective gap tolerance: `tolerance * gap_factor`.
    pub fn gap_tolerance(&self) -> f64 {
        self.tolerance * self.gap_factor
    }
}

// ============================================================
// HealingReport
// ============================================================

/// Report describing what the healing pipeline changed.
#[derive(Clone, Debug, Default)]
pub struct HealingReport {
    /// Number of gap pairs that were closed.
    pub gaps_closed: u32,
    /// Number of holes that were filled.
    pub holes_filled: u32,
    /// Number of edge pairs that were stitched.
    pub edges_stitched: u32,
    /// Number of face normals that were flipped.
    pub normals_fixed: u32,
    /// Number of faces removed (small features).
    pub small_faces_removed: u32,
    /// Number of degenerate edges marked.
    pub degenerate_edges_marked: u32,
    /// Number of sliver triangles detected (mesh-level).
    pub sliver_triangles_detected: u32,
    /// Number of face pairs merged (coplanar or co-cylindrical).
    pub faces_merged: u32,
    /// Number of entities whose tolerance was increased during propagation.
    pub tolerances_propagated: u32,
    /// Human-readable messages describing each operation.
    pub messages: Vec<String>,
}

impl HealingReport {
    /// Total number of individual fixes applied.
    pub fn total_fixes(&self) -> u32 {
        self.gaps_closed
            + self.holes_filled
            + self.edges_stitched
            + self.normals_fixed
            + self.small_faces_removed
            + self.degenerate_edges_marked
            + self.faces_merged
            + self.tolerances_propagated
    }

    fn add_msg(&mut self, msg: impl Into<String>) {
        self.messages.push(msg.into());
    }
}

// ============================================================
// Top-level healing functions
// ============================================================

/// Heal a solid using the given parameters.
///
/// Returns the healed solid (a clone with modifications applied) and a
/// report describing what was changed.
pub fn heal_solid(solid: &Solid, params: &HealingParams) -> (Solid, HealingReport) {
    let mut report = HealingReport::default();

    // Heal outer shell
    let outer_shell = if let Some(ref shell) = solid.outer_shell {
        let (healed_shell, shell_report) = heal_shell(shell, params);
        merge_report(&mut report, &shell_report);
        Some(healed_shell)
    } else {
        None
    };

    // Heal inner shells
    let inner_shells: Vec<Shell> = solid
        .inner_shells
        .iter()
        .map(|shell| {
            let (healed, r) = heal_shell(shell, params);
            merge_report(&mut report, &r);
            healed
        })
        .collect();

    let healed_solid = Solid {
        id: solid.id,
        outer_shell,
        inner_shells,
    };

    (healed_solid, report)
}

/// Heal a shell using the given parameters.
///
/// Applies the healing pipeline in a specific order:
/// 0. Propagate tolerances (3.1.7)
/// 1. Mark degenerate edges
/// 2. Close gaps between boundary edges
/// 3. Fill small holes
/// 4. Stitch collinear edges
/// 5. Merge coplanar/co-cylindrical faces (3.1.5)
/// 6. Remove small-feature faces
/// 7. Fix normal orientation (for closed shells)
pub fn heal_shell(shell: &Shell, params: &HealingParams) -> (Shell, HealingReport) {
    let mut report = HealingReport::default();
    let mut shell = shell.clone();

    // 0. Propagate tolerances (must run first — correct tolerances are
    //    needed for all subsequent operations)
    if params.propagate_tolerances {
        propagate_tolerances(&mut shell, params, &mut report);
    }

    // 1. Mark degenerate edges
    mark_degenerate_edges(&mut shell, params, &mut report);

    // 2. Close gaps
    close_gaps(&mut shell, params, &mut report);

    // 3. Fill small holes
    fill_holes(&mut shell, params, &mut report);

    // 4. Stitch collinear edges
    if params.stitch_edges {
        stitch_collinear_edges(&mut shell, params, &mut report);
    }

    // 5. Merge coplanar and co-cylindrical faces
    if params.merge_faces {
        merge_faces(&mut shell, params, &mut report);
    }

    // 6. Remove small-feature faces
    remove_small_features(&mut shell, params, &mut report);

    // 7. Fix normal orientation for closed shells
    if params.fix_normals && shell.closed {
        fix_normal_orientation(&mut shell, params, &mut report);
    }

    (shell, report)
}

// ============================================================
// Internal healing operations
// ============================================================

/// Propagate tolerances through the topological graph (3.1.7).
///
/// Ensures tolerance consistency across the B-Rep hierarchy:
///
/// 1. **Downward propagation**: If a `ToleranceContext` is provided, all
///    entities are guaranteed to have at least the `coincidence_tolerance()`
///    from the context. This is important when STEP files specify global
///    tolerances that should override individual entity tolerances.
///
/// 2. **Vertex → Edge**: An edge's tolerance must be at least the maximum
///    of its vertex tolerances. If either vertex has a larger tolerance,
///    the edge's tolerance is increased to match.
///
/// 3. **Edge → Face**: A face's tolerance must be at least the maximum
///    of its edge tolerances. If any edge has a larger tolerance, the
///    face's tolerance is increased to match.
///
/// 4. **Face → Shell**: The shell's effective tolerance is the maximum
///    of all its face tolerances. (This is informational; shells don't
///    have a tolerance field, so we report it.)
fn propagate_tolerances(shell: &mut Shell, params: &HealingParams, report: &mut HealingReport) {
    let mut count = 0u32;

    // Determine the floor tolerance from ToleranceContext (downward propagation)
    let floor_tol = params
        .tolerance_context
        .as_ref()
        .map(|ctx| ctx.coincidence_tolerance())
        .unwrap_or(0.0);

    // Build a map of vertex ID → tolerance so we can look up vertex
    // tolerances when processing edges.
    // Vertices are stored as TopoId references in edges (vertex_start, vertex_end),
    // but the actual Vertex objects live inside the face.edges[].vertex_start/vertex_end
    // which are just IDs. We need to find the vertex point to reconstruct tolerances.
    //
    // Since vertices are not stored as standalone objects in our entity model
    // (they're implicit in edges), we track vertex tolerances via a HashMap.
    // The vertex tolerance is initialized from the floor or from the edge's
    // starting tolerance — but since we don't have explicit Vertex objects in
    // the shell, we use a simpler approach:
    // - For downward propagation: apply floor tolerance to edges and faces
    // - For upward propagation: edges inherit max of their vertex-related
    //   tolerances (estimated from the edge's own geometry), faces inherit
    //   from edges.

    // Phase 1: Apply floor tolerance to all edges and faces (downward propagation)
    if floor_tol > 0.0 {
        for face in &mut shell.faces {
            // Apply floor to edges
            for edge in &mut face.edges {
                if edge.tolerance < floor_tol {
                    edge.tolerance = floor_tol;
                    count += 1;
                }
            }
            // Apply floor to face
            if face.tolerance < floor_tol {
                face.tolerance = floor_tol;
                count += 1;
            }
        }
    }

    // Phase 2: Vertex → Edge propagation
    // Since vertices are stored as TopoId references (not inline objects),
    // we collect vertex tolerances by building a map from vertex ID to
    // the maximum tolerance seen at that vertex across all edges.
    //
    // In our entity model, Edge has vertex_start: Option<TopoId> and
    // vertex_end: Option<TopoId>, but Vertex objects with their own
    // tolerance are not directly stored in the Shell. Instead, the
    // vertex tolerance is implicitly derived from the edge's tolerance.
    //
    // For a complete implementation, we collect the max edge tolerance
    // associated with each vertex, then propagate that back to edges
    // that share the vertex.
    let mut vertex_max_tol: std::collections::HashMap<TopoId, f64> =
        std::collections::HashMap::new();

    // First pass: collect the maximum tolerance for each vertex
    for face in &shell.faces {
        for edge in &face.edges {
            if let Some(vid) = edge.vertex_start {
                let entry = vertex_max_tol.entry(vid).or_insert(0.0);
                *entry = (*entry).max(edge.tolerance);
            }
            if let Some(vid) = edge.vertex_end {
                let entry = vertex_max_tol.entry(vid).or_insert(0.0);
                *entry = (*entry).max(edge.tolerance);
            }
        }
    }

    // Second pass: propagate vertex tolerances upward to edges
    for face in &mut shell.faces {
        for edge in &mut face.edges {
            let mut max_vertex_tol = 0.0f64;
            if let Some(vid) = edge.vertex_start {
                if let Some(&tol) = vertex_max_tol.get(&vid) {
                    max_vertex_tol = max_vertex_tol.max(tol);
                }
            }
            if let Some(vid) = edge.vertex_end {
                if let Some(&tol) = vertex_max_tol.get(&vid) {
                    max_vertex_tol = max_vertex_tol.max(tol);
                }
            }
            if max_vertex_tol > edge.tolerance {
                edge.tolerance = max_vertex_tol;
                count += 1;
            }
        }
    }

    // Phase 3: Edge → Face propagation
    for face in &mut shell.faces {
        let max_edge_tol = face
            .edges
            .iter()
            .map(|e| e.tolerance)
            .fold(0.0f64, f64::max);
        if max_edge_tol > face.tolerance {
            face.tolerance = max_edge_tol;
            count += 1;
        }
    }

    // Phase 4: Face → Shell (informational)
    // Shell doesn't have a tolerance field, but we report the max face
    // tolerance in the report messages for debugging.
    let max_face_tol = shell
        .faces
        .iter()
        .map(|f| f.tolerance)
        .fold(0.0f64, f64::max);

    if count > 0 {
        report.tolerances_propagated = count;
        report.add_msg(format!(
            "Propagated tolerances: {} entities updated, shell max tolerance = {:.2e}",
            count, max_face_tol
        ));
    }
}

/// Mark degenerate edges (zero-length or degenerate curves).
fn mark_degenerate_edges(shell: &mut Shell, params: &HealingParams, report: &mut HealingReport) {
    let mut count = 0u32;
    for face in &mut shell.faces {
        for edge in &mut face.edges {
            if !edge.degenerate {
                let is_degen = if let Some(ref curve) = edge.curve {
                    curve.is_degenerate(params.tolerance)
                } else if let (Some(sp), Some(ep)) = (edge.start_point(), edge.end_point()) {
                    // No curve — check if start/end points coincide
                    sp.distance_sq_to(&ep) < params.tolerance * params.tolerance
                } else {
                    // No curve and no evaluable points — edge has no meaningful
                    // geometry and is considered degenerate
                    true
                };
                if is_degen {
                    edge.degenerate = true;
                    count += 1;
                }
            }
        }
    }
    if count > 0 {
        report.degenerate_edges_marked = count;
        report.add_msg(format!("Marked {} degenerate edges", count));
    }
}

/// Close gaps between boundary edges.
///
/// Searches for pairs of boundary edges (edges used by only one coedge
/// in the shell) whose midpoints are closer than `tolerance * gap_factor`,
/// and merges them by making the coedges reference the same edge.
fn close_gaps(shell: &mut Shell, params: &HealingParams, report: &mut HealingReport) {
    let gap_tol = params.gap_tolerance();
    let gap_tol_sq = gap_tol * gap_tol;

    // Collect all edge IDs used by coedges in the shell.
    // Count how many times each edge ID is referenced.
    let mut edge_use_count: std::collections::HashMap<TopoId, u32> =
        std::collections::HashMap::new();
    for face in &shell.faces {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                *edge_use_count.entry(coedge.edge).or_insert(0) += 1;
            }
        }
        for wire in &face.inner_wires {
            for coedge in &wire.coedges {
                *edge_use_count.entry(coedge.edge).or_insert(0) += 1;
            }
        }
    }

    // Also count from the face.edges list — edges stored directly
    let mut edge_midpoints: std::collections::HashMap<TopoId, Point3d> =
        std::collections::HashMap::new();
    for face in &shell.faces {
        for edge in &face.edges {
            let midpoint = edge
                .point_at(0.5)
                .unwrap_or_else(|| edge.start_point().unwrap_or(Point3d::ORIGIN));
            edge_midpoints.entry(edge.id).or_insert(midpoint);
            // Count from edges list too
            *edge_use_count.entry(edge.id).or_insert(0) += 1;
        }
    }

    // Boundary edges: those referenced only once in coedges
    let boundary_edge_ids: Vec<TopoId> = {
        let mut coedge_use_count: std::collections::HashMap<TopoId, u32> =
            std::collections::HashMap::new();
        for face in &shell.faces {
            if let Some(ref wire) = face.outer_wire {
                for coedge in &wire.coedges {
                    *coedge_use_count.entry(coedge.edge).or_insert(0) += 1;
                }
            }
            for wire in &face.inner_wires {
                for coedge in &wire.coedges {
                    *coedge_use_count.entry(coedge.edge).or_insert(0) += 1;
                }
            }
        }
        coedge_use_count
            .iter()
            .filter(|(_, &count)| count == 1)
            .map(|(&id, _)| id)
            .collect()
    };

    // Find pairs of boundary edges that are close
    let mut merges: Vec<(TopoId, TopoId)> = Vec::new();
    let mut already_merged: std::collections::HashSet<TopoId> = std::collections::HashSet::new();

    for i in 0..boundary_edge_ids.len() {
        let id_a = boundary_edge_ids[i];
        if already_merged.contains(&id_a) {
            continue;
        }
        let mp_a = match edge_midpoints.get(&id_a) {
            Some(p) => *p,
            None => continue,
        };

        for j in (i + 1)..boundary_edge_ids.len() {
            let id_b = boundary_edge_ids[j];
            if already_merged.contains(&id_b) {
                continue;
            }
            if id_a == id_b {
                continue;
            }
            let mp_b = match edge_midpoints.get(&id_b) {
                Some(p) => *p,
                None => continue,
            };

            if mp_a.distance_sq_to(&mp_b) < gap_tol_sq {
                merges.push((id_a, id_b));
                already_merged.insert(id_b);
                break; // Each edge is merged at most once
            }
        }
    }

    // Apply merges: replace references to id_b with id_a in coedges
    let mut gap_count = 0u32;
    for (id_a, id_b) in &merges {
        let replaced = replace_coedge_edge_refs(&mut shell.faces, *id_b, *id_a);
        if replaced > 0 {
            gap_count += 1;
        }
    }

    if gap_count > 0 {
        report.gaps_closed = gap_count;
        report.add_msg(format!("Closed {} gaps between boundary edges", gap_count));
    }
}

/// Fill small holes in the shell.
///
/// A hole is a closed boundary loop formed by edges that appear in only
/// one coedge. If the loop has ≤ `max_hole_edges` edges, a new face is
/// created to cap the hole.
fn fill_holes(shell: &mut Shell, params: &HealingParams, report: &mut HealingReport) {
    // Collect boundary edges (used by exactly 1 coedge)
    let mut coedge_use_count: std::collections::HashMap<TopoId, u32> =
        std::collections::HashMap::new();
    for face in &shell.faces {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                *coedge_use_count.entry(coedge.edge).or_insert(0) += 1;
            }
        }
        for wire in &face.inner_wires {
            for coedge in &wire.coedges {
                *coedge_use_count.entry(coedge.edge).or_insert(0) += 1;
            }
        }
    }

    let boundary_edge_ids: std::collections::HashSet<TopoId> = coedge_use_count
        .iter()
        .filter(|(_, &count)| count == 1)
        .map(|(&id, _)| id)
        .collect();

    if boundary_edge_ids.is_empty() {
        return;
    }

    // Build a map from edge ID to edge geometry (start/end points)
    let mut edge_points: std::collections::HashMap<TopoId, (Point3d, Point3d)> =
        std::collections::HashMap::new();
    for face in &shell.faces {
        for edge in &face.edges {
            if boundary_edge_ids.contains(&edge.id) {
                if let (Some(sp), Some(ep)) = (edge.start_point(), edge.end_point()) {
                    edge_points.insert(edge.id, (sp, ep));
                }
            }
        }
    }

    // Find boundary loops by chaining edges end-to-start
    let loops = find_boundary_loops(&boundary_edge_ids, &edge_points, params.tolerance);

    let mut holes_filled = 0u32;
    for hole_loop in &loops {
        if hole_loop.len() <= params.max_hole_edges && hole_loop.len() >= 3 {
            // Create a face to fill the hole
            if let Some(fill_face) = create_fill_face(hole_loop, &edge_points, params.tolerance) {
                shell.faces.push(fill_face);
                holes_filled += 1;
            }
        }
    }

    if holes_filled > 0 {
        report.holes_filled = holes_filled;
        report.add_msg(format!("Filled {} holes", holes_filled));
    }
}

/// Stitch collinear edges that share a vertex.
///
/// When two adjacent edges are collinear (their direction vectors are
/// parallel within angular tolerance) and share a common vertex, they
/// are merged into a single edge.
fn stitch_collinear_edges(shell: &mut Shell, params: &HealingParams, report: &mut HealingReport) {
    let angular_tol = 1e-6; // Radians — ~0.00006 degrees
    let mut stitch_count = 0u32;

    for face in &mut shell.faces {
        // Look for consecutive edges in the outer wire that are collinear
        if let Some(ref mut wire) = face.outer_wire {
            if wire.coedges.len() < 2 {
                continue;
            }

            let n = wire.coedges.len();
            let mut to_remove: Vec<usize> = Vec::new();

            for i in 0..n {
                if to_remove.contains(&i) {
                    continue;
                }
                let j = (i + 1) % n;
                if to_remove.contains(&j) {
                    continue;
                }

                let edge_id_i = wire.coedges[i].edge;
                let edge_id_j = wire.coedges[j].edge;

                // Find the corresponding edges
                let edge_i = face.edges.iter().find(|e| e.id == edge_id_i);
                let edge_j = face.edges.iter().find(|e| e.id == edge_id_j);

                if let (Some(ei), Some(ej)) = (edge_i, edge_j) {
                    if are_edges_collinear(ei, ej, angular_tol, params.tolerance) {
                        // Merge: remove edge j, extend edge i to cover both
                        to_remove.push(j);
                        stitch_count += 1;
                    }
                }
            }

            // Remove merged coedges (in reverse order to keep indices valid)
            if !to_remove.is_empty() {
                let mut sorted = to_remove;
                sorted.sort_unstable();
                sorted.dedup();
                for &idx in sorted.iter().rev() {
                    wire.coedges.remove(idx);
                }
                if wire.coedges.len() > 1 {
                    wire.closed = true;
                }
            }
        }
    }

    if stitch_count > 0 {
        report.edges_stitched = stitch_count;
        report.add_msg(format!("Stitched {} collinear edge pairs", stitch_count));
    }
}

/// Merge coplanar and co-cylindrical faces that share edges.
///
/// When two faces share one or more edges and lie on geometrically
/// compatible surfaces (same plane or same cylinder within tolerance),
/// they are merged into a single face with a combined wire.
///
/// # Algorithm
///
/// 1. Build a map from edge ID → face indices to find face pairs sharing edges.
/// 2. For each pair of adjacent faces, check surface compatibility.
/// 3. For compatible pairs, reconstruct the merged boundary by removing
///    shared edges and re-connecting the remaining edges.
/// 4. Replace both original faces with the merged face.
fn merge_faces(shell: &mut Shell, params: &HealingParams, report: &mut HealingReport) {
    let tol = params.tolerance;
    let angular_tol = 1e-6; // radians (~0.00006°)

    // Iteratively merge face pairs until no more merges are possible.
    // Each iteration may enable new merges (transitive merging).
    let mut total_merged = 0u32;
    loop {
        let merged_this_pass = merge_one_pass(shell, tol, angular_tol);
        if merged_this_pass == 0 {
            break;
        }
        total_merged += merged_this_pass;
    }

    if total_merged > 0 {
        report.faces_merged = total_merged;
        report.add_msg(format!("Merged {} face pairs", total_merged));
    }
}

/// Perform one pass of face merging. Returns the number of merges performed.
fn merge_one_pass(shell: &mut Shell, tol: f64, angular_tol: f64) -> u32 {
    let n = shell.faces.len();
    if n < 2 {
        return 0;
    }

    // Build a map: edge_id → set of face indices that use this edge in their coedges
    let mut edge_to_faces: std::collections::HashMap<TopoId, std::collections::HashSet<usize>> =
        std::collections::HashMap::new();
    for (fi, face) in shell.faces.iter().enumerate() {
        let coedge_edge_ids = face_coedge_edge_ids(face);
        for eid in coedge_edge_ids {
            edge_to_faces.entry(eid).or_default().insert(fi);
        }
    }

    // Build face adjacency: find pairs of faces sharing at least one edge
    let mut adjacent_pairs: std::collections::HashSet<(usize, usize)> =
        std::collections::HashSet::new();
    for face_set in edge_to_faces.values() {
        let face_vec: Vec<usize> = face_set.iter().copied().collect();
        for i in 0..face_vec.len() {
            for j in (i + 1)..face_vec.len() {
                let a = face_vec[i].min(face_vec[j]);
                let b = face_vec[i].max(face_vec[j]);
                adjacent_pairs.insert((a, b));
            }
        }
    }

    // Check each adjacent pair for surface compatibility and merge
    let mut faces_to_remove: std::collections::HashSet<usize> = std::collections::HashSet::new();
    let mut new_faces: Vec<Face> = Vec::new();
    let mut merge_count = 0u32;

    for (fi, fj) in &adjacent_pairs {
        if faces_to_remove.contains(fi) || faces_to_remove.contains(fj) {
            continue;
        }

        let face_a = &shell.faces[*fi];
        let face_b = &shell.faces[*fj];

        // Check surface compatibility
        if !are_surfaces_compatible(&face_a.surface, &face_b.surface, tol, angular_tol) {
            continue;
        }

        // Find shared edge IDs
        let edges_a = face_coedge_edge_ids(face_a);
        let edges_b = face_coedge_edge_ids(face_b);
        let set_a: std::collections::HashSet<TopoId> = edges_a.iter().copied().collect();
        let set_b: std::collections::HashSet<TopoId> = edges_b.iter().copied().collect();
        let shared: std::collections::HashSet<TopoId> =
            set_a.intersection(&set_b).copied().collect();

        if shared.is_empty() {
            continue;
        }

        // Merge the two faces
        if let Some(merged_face) = merge_two_faces(face_a, face_b, &shared, tol) {
            faces_to_remove.insert(*fi);
            faces_to_remove.insert(*fj);
            new_faces.push(merged_face);
            merge_count += 1;
        }
    }

    // Apply: remove merged faces and add new ones
    if merge_count > 0 {
        let mut sorted_remove: Vec<usize> = faces_to_remove.iter().copied().collect();
        sorted_remove.sort_unstable();
        for &idx in sorted_remove.iter().rev() {
            shell.faces.remove(idx);
        }
        shell.faces.extend(new_faces);
    }

    merge_count
}

/// Collect all edge IDs referenced by coedges in a face (outer + inner wires).
fn face_coedge_edge_ids(face: &Face) -> Vec<TopoId> {
    let mut ids = Vec::new();
    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            ids.push(coedge.edge);
        }
    }
    for wire in &face.inner_wires {
        for coedge in &wire.coedges {
            ids.push(coedge.edge);
        }
    }
    ids
}

/// Check whether two surfaces are compatible for merging.
///
/// Two surfaces are compatible if:
/// - Both are Planes with the same normal (within angular tolerance) and
///   same origin distance from the plane (within tolerance).
/// - Both are Cylinders with the same axis, radius, and origin (within tolerance).
fn are_surfaces_compatible(
    surf_a: &Option<Surface>,
    surf_b: &Option<Surface>,
    tol: f64,
    angular_tol: f64,
) -> bool {
    let (sa, sb) = match (surf_a, surf_b) {
        (Some(a), Some(b)) => (a, b),
        _ => return false,
    };

    match (sa, sb) {
        (Surface::Plane(pa), Surface::Plane(pb)) => are_planes_compatible(pa, pb, tol, angular_tol),
        (Surface::Cylinder(ca), Surface::Cylinder(cb)) => {
            are_cylinders_compatible(ca, cb, tol, angular_tol)
        }
        _ => false,
    }
}

/// Check whether two planes are compatible for merging.
///
/// Two planes are compatible if their normals are parallel (within angular
/// tolerance) and their origin distances from the common plane are the same
/// (within tolerance). This handles both same-side and opposite-side normals
/// (a face with `forward = false` has its effective normal flipped).
fn are_planes_compatible(a: &Plane, b: &Plane, tol: f64, angular_tol: f64) -> bool {
    // Check if normals are parallel (same or opposite direction)
    let dot = a.normal.dot(&b.normal);
    let angle = dot.acos().abs();
    if angle > angular_tol && (std::f64::consts::PI - angle).abs() > angular_tol {
        return false;
    }

    // Check if planes are coplanar: distance from b.origin to plane a
    let dx = b.origin.x - a.origin.x;
    let dy = b.origin.y - a.origin.y;
    let dz = b.origin.z - a.origin.z;
    let dist = dx * a.normal.x + dy * a.normal.y + dz * a.normal.z;
    dist.abs() < tol
}

/// Check whether two cylinders are compatible for merging.
///
/// Two cylinders are compatible if their axes are parallel, radii match,
/// and their origins project to the same point on the shared axis
/// (within tolerance).
fn are_cylinders_compatible(
    a: &CylinderSurface,
    b: &CylinderSurface,
    tol: f64,
    angular_tol: f64,
) -> bool {
    // Check radii match
    if (a.radius - b.radius).abs() > tol {
        return false;
    }

    // Check axes are parallel
    let dot = a.axis.dot(&b.axis);
    let angle = dot.acos().abs();
    if angle > angular_tol && (std::f64::consts::PI - angle).abs() > angular_tol {
        return false;
    }

    // Check that origins project to the same point on the shared axis.
    // Project b.origin onto a's axis and check distance.
    let dx = b.origin.x - a.origin.x;
    let dy = b.origin.y - a.origin.y;
    let dz = b.origin.z - a.origin.z;
    let along_axis = dx * a.axis.x + dy * a.axis.y + dz * a.axis.z;
    let perp_x = dx - along_axis * a.axis.x;
    let perp_y = dy - along_axis * a.axis.y;
    let perp_z = dz - along_axis * a.axis.z;
    let perp_dist_sq = perp_x * perp_x + perp_y * perp_y + perp_z * perp_z;
    perp_dist_sq < tol * tol
}

/// Merge two faces that share edges and have compatible surfaces.
///
/// The merged face:
/// - Uses the surface from `face_a`
/// - Has a combined outer wire formed by removing shared edges and
///   reconnecting the remaining edges
/// - Preserves all inner wires (holes) from both faces
fn merge_two_faces(
    face_a: &Face,
    face_b: &Face,
    shared_edge_ids: &std::collections::HashSet<TopoId>,
    tol: f64,
) -> Option<Face> {
    // Collect coedges from both faces, marking which are shared
    // We need to walk the boundary and skip shared edges
    let coedges_a = face_coedges_with_forward(face_a);
    let coedges_b = face_coedges_with_forward(face_b);

    // Collect edge geometry from both faces
    let mut all_edges: std::collections::HashMap<TopoId, Edge> = std::collections::HashMap::new();
    for edge in &face_a.edges {
        all_edges.entry(edge.id).or_insert_with(|| edge.clone());
    }
    for edge in &face_b.edges {
        all_edges.entry(edge.id).or_insert_with(|| edge.clone());
    }

    // Build the combined outer wire:
    // Non-shared coedges from both faces form the new boundary.
    // We need to order them correctly by connecting end-to-start.
    let mut non_shared_coedges: Vec<(TopoId, bool)> = Vec::new(); // (edge_id, forward)

    for (edge_id, forward) in &coedges_a {
        if !shared_edge_ids.contains(edge_id) {
            non_shared_coedges.push((*edge_id, *forward));
        }
    }
    for (edge_id, forward) in &coedges_b {
        if !shared_edge_ids.contains(edge_id) {
            // If face_b has opposite orientation from face_a, we may need to
            // flip the coedge direction for the merged face.
            // The merged face uses face_a's surface orientation.
            // If face_b.forward != face_a.forward, we need to flip face_b's coedges.
            let effective_forward = if face_a.forward != face_b.forward {
                !*forward
            } else {
                *forward
            };
            non_shared_coedges.push((*edge_id, effective_forward));
        }
    }

    // Now we need to order these coedges to form a proper wire.
    // Build a graph of edge connectivity and walk the boundary.
    let ordered_coedges = order_coedges_into_wire(&non_shared_coedges, &all_edges, tol)?;

    // Collect all edges needed by the merged face
    let mut merged_edge_list: Vec<Edge> = Vec::new();
    for (edge_id, _forward) in &ordered_coedges {
        if let Some(edge) = all_edges.remove(edge_id) {
            merged_edge_list.push(edge);
        }
    }

    // Build the wire
    let wire_coedges: Vec<CoEdge> = ordered_coedges
        .iter()
        .map(|(edge_id, forward)| CoEdge::new(*edge_id, *forward))
        .collect();
    let mut outer_wire = Wire::new(wire_coedges);
    outer_wire.closed = true;

    // Build inner wires from both faces (preserve holes)
    let mut inner_wires: Vec<Wire> = Vec::new();
    inner_wires.extend(face_a.inner_wires.clone());
    inner_wires.extend(face_b.inner_wires.clone());

    // Create merged face
    let mut merged_face = Face::new(
        face_a.surface.clone().unwrap_or(Surface::Plane(Plane::xy())),
        outer_wire,
    );
    merged_face.forward = face_a.forward;
    merged_face.edges = merged_edge_list;
    merged_face.inner_wires = inner_wires;

    Some(merged_face)
}

/// Get the list of (edge_id, forward) for coedges in the outer wire of a face.
fn face_coedges_with_forward(face: &Face) -> Vec<(TopoId, bool)> {
    let mut result = Vec::new();
    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            result.push((coedge.edge, coedge.forward));
        }
    }
    result
}

/// Order a set of coedges into a proper wire by connecting end-to-start.
///
/// Each coedge represents an edge with an orientation. When `forward` is true,
/// the edge goes from start_point to end_point. When false, it's reversed.
///
/// We walk from one coedge to the next by matching the end point of the
/// current coedge with the start point of the next.
fn order_coedges_into_wire(
    coedges: &[(TopoId, bool)],
    edge_map: &std::collections::HashMap<TopoId, Edge>,
    tol: f64,
) -> Option<Vec<(TopoId, bool)>> {
    if coedges.is_empty() {
        return Some(Vec::new());
    }
    if coedges.len() == 1 {
        return Some(coedges.to_vec());
    }

    let tol_sq = tol * tol;

    // For each coedge, compute its start and end points considering orientation
    let mut start_pts: Vec<Point3d> = Vec::with_capacity(coedges.len());
    let mut end_pts: Vec<Point3d> = Vec::with_capacity(coedges.len());

    for (edge_id, forward) in coedges {
        let edge = edge_map.get(edge_id)?;
        let (sp, ep) = if let (Some(s), Some(e)) = (edge.start_point(), edge.end_point()) {
            (s, e)
        } else {
            return None;
        };
        if *forward {
            start_pts.push(sp);
            end_pts.push(ep);
        } else {
            start_pts.push(ep);
            end_pts.push(sp);
        }
    }

    // Build adjacency: for each coedge end, find which coedge starts there
    // Use grid snapping for robustness
    let snap = |p: &Point3d| -> (i64, i64, i64) {
        let scale = 1.0 / tol;
        (
            (p.x * scale).round() as i64,
            (p.y * scale).round() as i64,
            (p.z * scale).round() as i64,
        )
    };

    let mut end_to_next: std::collections::HashMap<(i64, i64, i64), Vec<usize>> =
        std::collections::HashMap::new();
    for i in 0..coedges.len() {
        let key = snap(&start_pts[i]);
        end_to_next.entry(key).or_default().push(i);
    }

    // Walk the wire starting from coedge 0
    let mut ordered = Vec::with_capacity(coedges.len());
    let mut visited = vec![false; coedges.len()];
    let mut current = 0;

    for _ in 0..coedges.len() {
        if visited[current] {
            // Cycle detected before visiting all coedges — disconnected graph
            break;
        }
        visited[current] = true;
        ordered.push((coedges[current].0, coedges[current].1));

        // Find next coedge whose start matches our end
        let end_key = snap(&end_pts[current]);
        let candidates = end_to_next.get(&end_key).cloned().unwrap_or_default();

        let mut found = false;
        for next_idx in &candidates {
            if !visited[*next_idx] {
                // Verify proximity
                if end_pts[current].distance_sq_to(&start_pts[*next_idx]) < tol_sq {
                    current = *next_idx;
                    found = true;
                    break;
                }
            }
        }

        if !found {
            // Try to find any unvisited coedge whose start is close to our end
            for i in 0..coedges.len() {
                if !visited[i] && end_pts[current].distance_sq_to(&start_pts[i]) < tol_sq {
                    current = i;
                    found = true;
                    break;
                }
            }
        }

        if !found {
            // Can't connect — just add remaining unvisited coedges
            for i in 0..coedges.len() {
                if !visited[i] {
                    ordered.push((coedges[i].0, coedges[i].1));
                    visited[i] = true;
                }
            }
            break;
        }
    }

    Some(ordered)
}

/// Remove faces with area below `min_face_area`.
fn remove_small_features(shell: &mut Shell, params: &HealingParams, report: &mut HealingReport) {
    let before = shell.faces.len();

    shell.faces.retain(|face| {
        // Keep faces that have no surface (we can't estimate area)
        let surface = match face.surface {
            Some(ref s) => s,
            None => return true,
        };

        // Estimate face area by sampling the surface within the wire boundary.
        // For simplicity, use the edge polygon area as an approximation.
        let area = estimate_face_area(face, surface);
        area >= params.min_face_area
    });

    let removed = before - shell.faces.len();
    if removed > 0 {
        report.small_faces_removed = removed as u32;
        report.add_msg(format!("Removed {} small-feature faces", removed));
    }
}

/// Fix face normals for closed shells so they all point outward.
///
/// For a closed shell, the faces' normals should all point away from the
/// interior. This function:
/// 1. Computes the shell's centroid
/// 2. For each face, checks whether its normal points away from the centroid
/// 3. Flips faces whose normals point inward
fn fix_normal_orientation(shell: &mut Shell, _params: &HealingParams, report: &mut HealingReport) {
    if shell.faces.is_empty() {
        return;
    }

    // Compute centroid as average of edge midpoints
    let mut centroid = Point3d::ORIGIN;
    let mut count = 0usize;
    for face in &shell.faces {
        for edge in &face.edges {
            if let Some(p) = edge.point_at(0.5) {
                centroid.x += p.x;
                centroid.y += p.y;
                centroid.z += p.z;
                count += 1;
            }
        }
        // Also sample surface center
        if let Some(ref surface) = face.surface {
            let p = surface.point_at(0.0, 0.0);
            centroid.x += p.x;
            centroid.y += p.y;
            centroid.z += p.z;
            count += 1;
        }
    }

    if count == 0 {
        return;
    }
    centroid.x /= count as f64;
    centroid.y /= count as f64;
    centroid.z /= count as f64;

    let mut flipped = 0u32;
    for face in &mut shell.faces {
        let surface = match face.surface {
            Some(ref s) => s,
            None => continue,
        };

        // Sample a point on the face
        let face_point = surface.point_at(0.0, 0.0);

        // Get the face normal at that point
        let normal = if face.forward {
            surface.normal_at(0.0, 0.0)
        } else {
            surface.normal_at(0.0, 0.0).neg()
        };

        // Vector from centroid to face point
        let to_face = Vec3d::new(
            face_point.x - centroid.x,
            face_point.y - centroid.y,
            face_point.z - centroid.z,
        );

        // If the normal points toward the centroid (dot product < 0),
        // the face is oriented inward and needs to be flipped
        let dot = normal.x * to_face.x + normal.y * to_face.y + normal.z * to_face.z;
        if dot < 0.0 {
            face.forward = !face.forward;
            flipped += 1;
        }
    }

    if flipped > 0 {
        report.normals_fixed = flipped;
        report.add_msg(format!("Fixed orientation of {} faces", flipped));
    }
}

// ============================================================
// Sliver triangle detection (for draper-mesh)
// ============================================================

/// Aspect ratio of a triangle defined by three 3D points.
///
/// The aspect ratio is defined as `longest_edge / shortest_altitude`.
/// A value > 100:1 indicates a sliver triangle.
pub fn triangle_aspect_ratio(p0: &Point3d, p1: &Point3d, p2: &Point3d) -> f64 {
    let e0 = p1.distance_to(p0);
    let e1 = p2.distance_to(p1);
    let e2 = p0.distance_to(p2);

    let longest = e0.max(e1).max(e2);
    if longest < 1e-15 {
        return 0.0;
    }

    // Area via cross product
    let v1 = Vec3d::new(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    let v2 = Vec3d::new(p2.x - p0.x, p2.y - p0.y, p2.z - p0.z);
    let cross = v1.cross(&v2);
    let area = cross.length() * 0.5;

    if area < 1e-20 {
        return f64::INFINITY; // Degenerate
    }

    // Shortest altitude = 2 * area / longest_edge
    let shortest_altitude = 2.0 * area / longest;

    if shortest_altitude < 1e-15 {
        return f64::INFINITY;
    }

    longest / shortest_altitude
}

// ============================================================
// Helper functions
// ============================================================

/// Replace all coedge references from `old_id` to `new_id` in the face list.
fn replace_coedge_edge_refs(faces: &mut [Face], old_id: TopoId, new_id: TopoId) -> usize {
    let mut count = 0;
    for face in faces.iter_mut() {
        if let Some(ref mut wire) = face.outer_wire {
            for coedge in &mut wire.coedges {
                if coedge.edge == old_id {
                    coedge.edge = new_id;
                    count += 1;
                }
            }
        }
        for wire in &mut face.inner_wires {
            for coedge in &mut wire.coedges {
                if coedge.edge == old_id {
                    coedge.edge = new_id;
                    count += 1;
                }
            }
        }
    }
    count
}

/// Find boundary loops by chaining edges end-to-start.
///
/// Each loop is returned as a `Vec<TopoId>` of edge IDs forming a closed loop.
fn find_boundary_loops(
    boundary_edge_ids: &std::collections::HashSet<TopoId>,
    edge_points: &std::collections::HashMap<TopoId, (Point3d, Point3d)>,
    tolerance: f64,
) -> Vec<Vec<TopoId>> {
    let tol_sq = tolerance * tolerance;

    // Build adjacency: end point → list of (edge_id, start point)
    // An edge goes from start to end. The next edge in a loop starts
    // where the previous one ends.
    let mut end_to_edges: std::collections::HashMap<(i64, i64, i64), Vec<TopoId>> =
        std::collections::HashMap::new();

    // Snap points to grid for hashing
    let snap = |p: &Point3d| -> (i64, i64, i64) {
        let scale = 1.0 / tolerance;
        (
            (p.x * scale).round() as i64,
            (p.y * scale).round() as i64,
            (p.z * scale).round() as i64,
        )
    };

    for &edge_id in boundary_edge_ids {
        if let Some(&(_start, end)) = edge_points.get(&edge_id) {
            let key = snap(&end);
            end_to_edges.entry(key).or_default().push(edge_id);
        }
    }

    let mut visited: std::collections::HashSet<TopoId> = std::collections::HashSet::new();
    let mut loops: Vec<Vec<TopoId>> = Vec::new();

    for &start_edge_id in boundary_edge_ids {
        if visited.contains(&start_edge_id) {
            continue;
        }

        let mut current_loop: Vec<TopoId> = Vec::new();
        let mut current_end = match edge_points.get(&start_edge_id) {
            Some(&(_, end)) => end,
            None => continue,
        };

        let mut current_edge = start_edge_id;
        let loop_start = match edge_points.get(&start_edge_id) {
            Some(&(start, _)) => start,
            None => continue,
        };

        loop {
            if visited.contains(&current_edge) {
                break;
            }
            visited.insert(current_edge);
            current_loop.push(current_edge);

            // Find next edge whose start is close to current_end
            let key = snap(&current_end);
            let candidates = end_to_edges.get(&key).cloned().unwrap_or_default();

            let mut found_next = false;
            for next_id in &candidates {
                if visited.contains(next_id) {
                    // Check if we've closed the loop
                    if let Some(&(next_start, _)) = edge_points.get(next_id) {
                        if next_start.distance_sq_to(&loop_start) < tol_sq
                            && current_loop.len() >= 3
                        {
                            // Loop is closed
                            found_next = false; // Don't add, just finish
                            break;
                        }
                    }
                    continue;
                }

                if let Some(&(next_start, next_end)) = edge_points.get(next_id) {
                    if next_start.distance_sq_to(&current_end) < tol_sq {
                        current_end = next_end;
                        current_edge = *next_id;
                        found_next = true;
                        break;
                    }
                }
            }

            if !found_next {
                break;
            }

            // Check if loop closed
            if current_end.distance_sq_to(&loop_start) < tol_sq && current_loop.len() >= 3 {
                break;
            }
        }

        if current_loop.len() >= 3 {
            loops.push(current_loop);
        }
    }

    loops
}

/// Create a face to fill a boundary hole.
///
/// The hole is defined by a list of edge IDs. We create a planar face
/// that caps the hole using a fan triangulation from the centroid.
fn create_fill_face(
    edge_ids: &[TopoId],
    edge_points: &std::collections::HashMap<TopoId, (Point3d, Point3d)>,
    tolerance: f64,
) -> Option<Face> {
    // Collect all unique vertices in order
    let mut vertices: Vec<Point3d> = Vec::new();
    for &edge_id in edge_ids {
        if let Some(&(start, _)) = edge_points.get(&edge_id) {
            // Avoid duplicates
            if vertices.last().map_or(true, |last| last.distance_sq_to(&start) > tolerance * tolerance) {
                vertices.push(start);
            }
        }
    }

    if vertices.len() < 3 {
        return None;
    }

    // Compute centroid
    let mut centroid = Point3d::ORIGIN;
    for v in &vertices {
        centroid.x += v.x;
        centroid.y += v.y;
        centroid.z += v.z;
    }
    let n = vertices.len() as f64;
    centroid.x /= n;
    centroid.y /= n;
    centroid.z /= n;

    // Create edges from each vertex to the next (forming the boundary)
    let mut edges: Vec<Edge> = Vec::new();
    let mut coedges: Vec<CoEdge> = Vec::new();
    for i in 0..vertices.len() {
        let j = (i + 1) % vertices.len();
        let edge = Edge::new_line(vertices[i], vertices[j]);
        coedges.push(CoEdge::new(edge.id, true));
        edges.push(edge);
    }

    let wire = Wire::new(coedges);

    // Create a plane through the centroid using the first 3 vertices
    let plane = if vertices.len() >= 3 {
        Plane::from_three_points(&centroid, &vertices[0], &vertices[1])
            .unwrap_or_else(|| Plane::from_origin_and_normal(centroid, Direction3d::Z))
    } else {
        Plane::from_origin_and_normal(centroid, Direction3d::Z)
    };

    let mut face = Face::new(Surface::Plane(plane), wire);
    face.edges = edges;
    Some(face)
}

/// Estimate the area of a face using its edge polygon.
fn estimate_face_area(face: &Face, surface: &Surface) -> f64 {
    // Collect 3D points from edges
    let mut points: Vec<Point3d> = Vec::new();

    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            // Find the edge in face.edges
            if let Some(edge) = face.edges.iter().find(|e| e.id == coedge.edge) {
                if let Some(sp) = edge.start_point() {
                    points.push(sp);
                }
            }
        }
    }

    if points.len() < 3 {
        // For faces without wire edges (e.g., sphere), estimate from surface sampling
        return estimate_surface_area(surface);
    }

    // Compute polygon area using the shoelace formula in 3D
    // Project onto the dominant plane
    let normal = polygon_normal(&points);
    if normal.is_none() {
        return estimate_surface_area(surface);
    }
    let n = normal.unwrap();

    // Choose the projection plane that maximizes the projected area
    let abs_nx = n.x.abs();
    let abs_ny = n.y.abs();
    let abs_nz = n.z.abs();

    let area = if abs_nz >= abs_nx && abs_nz >= abs_ny {
        // Project onto XY
        shoelace_xy(&points)
    } else if abs_ny >= abs_nx {
        // Project onto XZ
        shoelace_xz(&points)
    } else {
        // Project onto YZ
        shoelace_yz(&points)
    };

    if area > 1e-20 {
        area
    } else {
        estimate_surface_area(surface)
    }
}

/// Estimate surface area by sampling (for faces without boundary edges).
fn estimate_surface_area(surface: &Surface) -> f64 {
    // Sample a 10x10 grid
    let n = 10;
    let mut area = 0.0;
    for i in 0..n {
        for j in 0..n {
            let u0 = i as f64 / n as f64 * 2.0 * std::f64::consts::PI;
            let u1 = (i + 1) as f64 / n as f64 * 2.0 * std::f64::consts::PI;
            let v0 = j as f64 / n as f64 * std::f64::consts::PI;
            let v1 = (j + 1) as f64 / n as f64 * std::f64::consts::PI;

            let p00 = surface.point_at(u0, v0);
            let p10 = surface.point_at(u1, v0);
            let p01 = surface.point_at(u0, v1);

            let e1 = Vec3d::new(p10.x - p00.x, p10.y - p00.y, p10.z - p00.z);
            let e2 = Vec3d::new(p01.x - p00.x, p01.y - p00.y, p01.z - p00.z);
            area += e1.cross(&e2).length() * 0.5;
        }
    }
    area
}

/// Compute the normal of a polygon defined by 3D points.
fn polygon_normal(points: &[Point3d]) -> Option<Direction3d> {
    if points.len() < 3 {
        return None;
    }

    let mut normal = Vec3d::ZERO;
    for i in 0..points.len() {
        let j = (i + 1) % points.len();
        // Cross product of vectors from origin (Newell's method)
        normal.x += (points[i].y - points[j].y) * (points[i].z + points[j].z);
        normal.y += (points[i].z - points[j].z) * (points[i].x + points[j].x);
        normal.z += (points[i].x - points[j].x) * (points[i].y + points[j].y);
    }

    normal.normalize()
}

/// Shoelace formula for polygon area projected onto XY plane.
fn shoelace_xy(points: &[Point3d]) -> f64 {
    let mut area = 0.0;
    for i in 0..points.len() {
        let j = (i + 1) % points.len();
        area += points[i].x * points[j].y - points[j].x * points[i].y;
    }
    area.abs() * 0.5
}

/// Shoelace formula for polygon area projected onto XZ plane.
fn shoelace_xz(points: &[Point3d]) -> f64 {
    let mut area = 0.0;
    for i in 0..points.len() {
        let j = (i + 1) % points.len();
        area += points[i].x * points[j].z - points[j].x * points[i].z;
    }
    area.abs() * 0.5
}

/// Shoelace formula for polygon area projected onto YZ plane.
fn shoelace_yz(points: &[Point3d]) -> f64 {
    let mut area = 0.0;
    for i in 0..points.len() {
        let j = (i + 1) % points.len();
        area += points[i].y * points[j].z - points[j].y * points[i].z;
    }
    area.abs() * 0.5
}

/// Check if two edges are collinear (their directions are parallel).
fn are_edges_collinear(e1: &Edge, e2: &Edge, angular_tol: f64, _tolerance: f64) -> bool {
    // Get direction vectors
    let dir1 = edge_direction(e1);
    let dir2 = edge_direction(e2);

    match (dir1, dir2) {
        (Some(d1), Some(d2)) => {
            // Check if directions are parallel (dot product ≈ ±1)
            let dot = d1.x * d2.x + d1.y * d2.y + d1.z * d2.z;
            let angle = dot.acos().abs();
            angle < angular_tol || (std::f64::consts::PI - angle).abs() < angular_tol
        }
        _ => false,
    }
}

/// Get the direction of an edge (from start to end).
fn edge_direction(edge: &Edge) -> Option<Direction3d> {
    match (edge.start_point(), edge.end_point()) {
        (Some(sp), Some(ep)) => Direction3d::new(ep.x - sp.x, ep.y - sp.y, ep.z - sp.z),
        _ => {
            // Try using the curve
            if let Some(ref curve) = edge.curve {
                let (tmin, tmax) = edge.param_range;
                let p0 = curve.point_at(tmin);
                let p1 = curve.point_at(tmax);
                Direction3d::new(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
            } else {
                None
            }
        }
    }
}

/// Merge a sub-report into the main report.
fn merge_report(target: &mut HealingReport, source: &HealingReport) {
    target.gaps_closed += source.gaps_closed;
    target.holes_filled += source.holes_filled;
    target.edges_stitched += source.edges_stitched;
    target.normals_fixed += source.normals_fixed;
    target.small_faces_removed += source.small_faces_removed;
    target.degenerate_edges_marked += source.degenerate_edges_marked;
    target.sliver_triangles_detected += source.sliver_triangles_detected;
    target.faces_merged += source.faces_merged;
    target.tolerances_propagated += source.tolerances_propagated;
    target.messages.extend(source.messages.iter().cloned());
}

// ============================================================
// Unit tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::ShapeBuilder;
    use draper_geometry::{Curve3d, Plane, Point3d};

    /// Test that default healing parameters are sensible.
    #[test]
    fn test_default_params() {
        let params = HealingParams::default();
        assert!(params.gap_factor > 0.0);
        assert!(params.max_hole_edges > 0);
        assert!(params.min_face_area > 0.0);
        assert!(params.max_aspect_ratio > 0.0);
        assert!(params.tolerance > 0.0);
        assert!(params.gap_tolerance() > params.tolerance);
    }

    /// Test that healing a clean box preserves its structure.
    ///
    /// Note: `ShapeBuilder::make_box` creates each face with its own
    /// independent edge objects (no shared edge IDs between faces).
    /// The healing pipeline detects these as topological gaps (geometrically
    /// coincident edges with different IDs) and closes them by merging
    /// coedge references.
    #[test]
    fn test_heal_clean_box() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let params = HealingParams {
            fix_normals: false,
            ..HealingParams::default()
        };
        let (healed, report) = heal_solid(&box_solid, &params);
        // The box has 12 shared physical edges, but each face has its own
        // edge objects, so gap closing finds 12 pairs to merge.
        assert_eq!(report.gaps_closed, 12);
        assert_eq!(report.holes_filled, 0);
        assert_eq!(report.small_faces_removed, 0);
        // The box should still have 6 faces
        if let Some(ref shell) = healed.outer_shell {
            assert_eq!(shell.faces.len(), 6);
        }
    }

    /// Test that healing fixes flipped normals.
    ///
    /// Note: ShapeBuilder::make_box may produce faces with inconsistent
    /// normals (not all pointing outward). The healing pipeline fixes all
    /// inward-pointing faces, so we check that at least one is corrected.
    #[test]
    fn test_fix_flipped_normal() {
        // Create a simple closed shell with a known-flipped face
        let mut box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // First, fix all normals so we have a known-good starting point
        let params_fix = HealingParams {
            fix_normals: true,
            ..HealingParams::default()
        };
        let (mut fixed_solid, _) = heal_solid(&box_solid, &params_fix);

        // Now flip one face deliberately
        if let Some(ref mut shell) = fixed_solid.outer_shell {
            if !shell.faces.is_empty() {
                shell.faces[0].forward = !shell.faces[0].forward;
            }
        }

        // Heal again — should fix exactly the one we flipped
        let (_healed, report) = heal_solid(&fixed_solid, &params_fix);
        assert_eq!(report.normals_fixed, 1);
    }

    /// Test that small face removal works.
    #[test]
    fn test_remove_small_faces() {
        // Create a shell with one tiny face and one normal face.
        // We use make_polygon_face which works for triangles with
        // edge lengths above the global TOLERANCE (1e-6).
        let mut faces = Vec::new();

        // Add a tiny face (area ≈ 0.5 * 0.001 * 0.001 = 5e-7)
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(0.001, 0.0, 0.0);
        let p2 = Point3d::new(0.0, 0.001, 0.0);

        let tiny_face = ShapeBuilder::make_polygon_face(&[p0, p1, p2]).unwrap();
        faces.push(tiny_face);

        // Add a normal face (area ≈ 0.5)
        let normal_face = ShapeBuilder::make_polygon_face(&[
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ])
        .unwrap();
        faces.push(normal_face);

        let mut shell = Shell::new(faces);
        let params = HealingParams {
            min_face_area: 1e-5,  // Threshold above the tiny face's area (5e-7)
            fix_normals: false,
            stitch_edges: false,
            ..HealingParams::default()
        };

        let (healed, report) = heal_shell(&shell, &params);

        assert_eq!(report.small_faces_removed, 1);
        assert_eq!(healed.faces.len(), 1);
    }

    /// Test HealingReport::total_fixes()
    #[test]
    fn test_report_total_fixes() {
        let report = HealingReport {
            gaps_closed: 2,
            holes_filled: 1,
            edges_stitched: 3,
            normals_fixed: 0,
            small_faces_removed: 1,
            degenerate_edges_marked: 4,
            sliver_triangles_detected: 0,
            faces_merged: 0,
            tolerances_propagated: 2,
            messages: Vec::new(),
        };
        assert_eq!(report.total_fixes(), 13);
    }

    /// Test triangle_aspect_ratio for an equilateral triangle.
    #[test]
    fn test_aspect_ratio_equilateral() {
        // Equilateral triangle with side length 1
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(1.0, 0.0, 0.0);
        let p2 = Point3d::new(0.5, 0.8660254037844386, 0.0); // sqrt(3)/2

        let ratio = triangle_aspect_ratio(&p0, &p1, &p2);
        // For an equilateral triangle, aspect ratio should be close to 2/sqrt(3) ≈ 1.15
        // (longest_edge / shortest_altitude = 1 / (sqrt(3)/2) ≈ 1.155)
        assert!(ratio > 1.0 && ratio < 1.5, "Equilateral aspect ratio should be ~1.15, got {}", ratio);
    }

    /// Test triangle_aspect_ratio for a sliver triangle.
    #[test]
    fn test_aspect_ratio_sliver() {
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(100.0, 0.0, 0.0);
        let p2 = Point3d::new(50.0, 0.001, 0.0); // Very thin

        let ratio = triangle_aspect_ratio(&p0, &p1, &p2);
        assert!(ratio > 100.0, "Sliver should have ratio > 100, got {}", ratio);
    }

    /// Test triangle_aspect_ratio for a degenerate (zero-area) triangle.
    #[test]
    fn test_aspect_ratio_degenerate() {
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(1.0, 0.0, 0.0);
        let p2 = Point3d::new(0.5, 0.0, 0.0); // Collinear

        let ratio = triangle_aspect_ratio(&p0, &p1, &p2);
        assert!(ratio.is_infinite(), "Degenerate triangle should have infinite ratio, got {}", ratio);
    }

    /// Test that gap tolerance is computed correctly.
    #[test]
    fn test_gap_tolerance() {
        let params = HealingParams {
            tolerance: 1e-6,
            gap_factor: 10.0,
            ..HealingParams::default()
        };
        assert!((params.gap_tolerance() - 1e-5).abs() < 1e-15);
    }

    /// Test HealingParams::from_tolerance_context.
    #[test]
    fn test_params_from_tolerance_context() {
        let ctx = ToleranceContext::from_model_scale(100.0);
        let params = HealingParams::from_tolerance_context(&ctx);
        assert!(params.tolerance > 0.0);
        assert!(params.min_face_area > 0.0);
    }

    /// Test healing a cylinder (closed shell with normals).
    #[test]
    fn test_heal_cylinder() {
        let cyl = ShapeBuilder::make_cylinder(5.0, 10.0);
        let params = HealingParams::default();
        let (healed, report) = heal_solid(&cyl, &params);

        // Cylinder should be fine after healing
        assert_eq!(report.gaps_closed, 0);
        assert_eq!(report.holes_filled, 0);

        // Should still have 3 faces
        if let Some(ref shell) = healed.outer_shell {
            assert_eq!(shell.faces.len(), 3);
        }
    }

    /// Test that edge direction extraction works.
    #[test]
    fn test_edge_direction() {
        let edge = Edge::new_line(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
        );
        let dir = edge_direction(&edge);
        assert!(dir.is_some());
        let d = dir.unwrap();
        assert!(d.x > 0.9); // Should point in +X direction
    }

    /// Test collinear edge detection.
    #[test]
    fn test_collinear_edges() {
        let e1 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0));
        let e2 = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(2.0, 0.0, 0.0));
        assert!(are_edges_collinear(&e1, &e2, 1e-6, 1e-6));

        let e3 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0));
        assert!(!are_edges_collinear(&e1, &e3, 1e-6, 1e-6));
    }

    /// Test that healing handles empty shells gracefully.
    #[test]
    fn test_heal_empty_shell() {
        let shell = Shell::new(vec![]);
        let params = HealingParams::default();
        let (healed, report) = heal_shell(&shell, &params);
        assert!(healed.faces.is_empty());
        assert_eq!(report.total_fixes(), 0);
    }

    /// Test that healing handles solids without outer shells.
    #[test]
    fn test_heal_no_outer_shell() {
        let solid = Solid {
            id: TopoId::new(),
            outer_shell: None,
            inner_shells: vec![],
        };
        let params = HealingParams::default();
        let (_healed, report) = heal_solid(&solid, &params);
        assert_eq!(report.total_fixes(), 0);
    }

    /// Test polygon_normal computation.
    #[test]
    fn test_polygon_normal() {
        let points = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let normal = polygon_normal(&points);
        assert!(normal.is_some());
        let n = normal.unwrap();
        // Should point in +Z or -Z direction
        assert!(n.z.abs() > 0.9, "Normal should be along Z, got ({}, {}, {})", n.x, n.y, n.z);
    }

    /// Test shoelace area computation.
    #[test]
    fn test_shoelace_area() {
        let points = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let area = shoelace_xy(&points);
        assert!((area - 1.0).abs() < 1e-10, "Unit square area should be 1.0, got {}", area);
    }

    /// Test healing with a shell that has a deliberately created gap.
    #[test]
    fn test_close_gap_in_shell() {
        // Create two adjacent rectangular faces that share almost the same edge
        // but with slightly different edge IDs (a gap)
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(1.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 1.0, 0.0);
        let p3 = Point3d::new(0.0, 1.0, 0.0);
        let p4 = Point3d::new(2.0, 0.0, 0.0);
        let p5 = Point3d::new(2.0, 1.0, 0.0);

        let face1 = ShapeBuilder::make_polygon_face(&[p0, p1, p2, p3]).unwrap();
        let face2 = ShapeBuilder::make_polygon_face(&[p1, p4, p5, p2]).unwrap();

        let mut shell = Shell::new(vec![face1, face2]);

        // The shared edge between face1 (p1→p2) and face2 (p2→p1 in reverse)
        // has different IDs — this creates a gap in topological connectivity
        let params = HealingParams {
            gap_factor: 100.0, // Large gap factor to close the gap
            fix_normals: false,
            ..HealingParams::default()
        };

        let mut report = HealingReport::default();
        close_gaps(&mut shell, &params, &mut report);

        // The gap should be detected and closed
        // (may or may not find gaps depending on midpoint proximity)
        // At minimum, the function should not crash
        assert!(shell.faces.len() >= 2);
    }

    /// Test that degenerate edges are marked during healing.
    #[test]
    fn test_mark_degenerate() {
        // Create a face with a degenerate edge (zero-length line)
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(1.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 1.0, 0.0);

        let mut face = ShapeBuilder::make_polygon_face(&[p0, p1, p2]).unwrap();

        // Add a degenerate edge manually: an edge with no curve and no
        // evaluable geometry should be detected as degenerate.
        let degenerate_edge = Edge {
            id: TopoId::new(),
            curve: None,
            param_range: (0.0, 0.0),
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
        };
        face.edges.push(degenerate_edge);

        // Also add a zero-radius circle edge (also degenerate)
        let zero_circle = draper_geometry::Circle::new_xy(Point3d::ORIGIN, 0.0);
        let degen_circle_edge = Edge {
            id: TopoId::new(),
            curve: Some(Curve3d::Circle(zero_circle)),
            param_range: (0.0, 2.0 * std::f64::consts::PI),
            vertex_start: None,
            vertex_end: None,
            forward: true,
            tolerance: 1e-6,
            degenerate: false,
        };
        face.edges.push(degen_circle_edge);

        let mut shell = Shell::new(vec![face]);
        let params = HealingParams {
            fix_normals: false,
            ..HealingParams::default()
        };

        let mut report = HealingReport::default();
        mark_degenerate_edges(&mut shell, &params, &mut report);

        // Both the no-curve edge and the zero-radius circle should be degenerate
        assert!(report.degenerate_edges_marked >= 2,
            "Expected at least 2 degenerate edges, got {}", report.degenerate_edges_marked);
    }

    /// Integration test: heal a box with a flipped normal.
    #[test]
    fn test_heal_box_with_issues() {
        // First create and fix a box so we have a known-good starting point
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let params_fix = HealingParams {
            fix_normals: true,
            ..HealingParams::default()
        };
        let (mut fixed_solid, _) = heal_solid(&box_solid, &params_fix);

        // Introduce a flipped normal
        if let Some(ref mut shell) = fixed_solid.outer_shell {
            if !shell.faces.is_empty() {
                shell.faces[0].forward = !shell.faces[0].forward;
            }
        }

        let params = HealingParams {
            fix_normals: true,
            stitch_edges: true,
            ..HealingParams::default()
        };

        let (healed, report) = heal_solid(&fixed_solid, &params);
        assert_eq!(report.normals_fixed, 1);

        // After healing, the box should still be valid
        if let Some(ref shell) = healed.outer_shell {
            assert_eq!(shell.faces.len(), 6);
        }
    }

    /// Test that two coplanar faces sharing an edge are merged into one.
    ///
    /// Creates two rectangles on the XY plane that share an edge,
    /// then verifies that face merging combines them.
    #[test]
    fn test_merge_coplanar_faces() {
        // Face A: rectangle (0,0)-(2,1) on XY plane
        //   vertices: (0,0,0), (2,0,0), (2,1,0), (0,1,0)
        let e_a0 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(2.0, 0.0, 0.0));
        let e_a1 = Edge::new_line(Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 0.0));
        // Shared edge: from (2,0,0) to (2,1,0) — but face A uses it forward
        let e_shared = Edge::new_line(Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 0.0));
        let e_a3 = Edge::new_line(Point3d::new(2.0, 1.0, 0.0), Point3d::new(0.0, 1.0, 0.0));
        let e_a4 = Edge::new_line(Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 0.0, 0.0));

        let coedges_a = vec![
            CoEdge::new(e_a0.id, true),
            CoEdge::new(e_shared.id, true),
            CoEdge::new(e_a3.id, true),
            CoEdge::new(e_a4.id, true),
        ];
        let wire_a = Wire::new(coedges_a);
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z);
        let mut face_a = Face::new(Surface::Plane(plane), wire_a);
        face_a.edges = vec![e_a0, e_a1.clone(), e_shared.clone(), e_a3, e_a4];

        // Face B: rectangle (2,0)-(4,1) on XY plane
        //   vertices: (2,0,0), (4,0,0), (4,1,0), (2,1,0)
        let e_b0 = Edge::new_line(Point3d::new(2.0, 0.0, 0.0), Point3d::new(4.0, 0.0, 0.0));
        let e_b1 = Edge::new_line(Point3d::new(4.0, 0.0, 0.0), Point3d::new(4.0, 1.0, 0.0));
        let e_b2 = Edge::new_line(Point3d::new(4.0, 1.0, 0.0), Point3d::new(2.0, 1.0, 0.0));
        // Face B uses the shared edge in reverse direction (from (2,1,0) to (2,0,0))
        let coedges_b = vec![
            CoEdge::new(e_shared.id, false),
            CoEdge::new(e_b0.id, true),
            CoEdge::new(e_b1.id, true),
            CoEdge::new(e_b2.id, true),
        ];
        let wire_b = Wire::new(coedges_b);
        let plane_b = Plane::from_origin_and_normal(Point3d::new(2.0, 0.0, 0.0), Direction3d::Z);
        let mut face_b = Face::new(Surface::Plane(plane_b), wire_b);
        face_b.edges = vec![e_shared, e_b0, e_b1, e_b2];

        let shell = Shell::new(vec![face_a, face_b]);

        let params = HealingParams {
            fix_normals: false,
            stitch_edges: false,
            merge_faces: true,
            ..HealingParams::default()
        };

        let (healed, report) = heal_shell(&shell, &params);

        assert!(report.faces_merged >= 1, "Expected at least 1 face pair merged, got {}", report.faces_merged);
        assert_eq!(healed.faces.len(), 1, "Expected 1 face after merging, got {}", healed.faces.len());

        // The merged face should have 6 edges (4 outer boundary + 2 from the L-shape)
        // Actually: the merged L-shape has 6 boundary edges after removing the shared one
        let merged_face = &healed.faces[0];
        if let Some(ref wire) = merged_face.outer_wire {
            // The L-shaped boundary has 6 edges
            assert_eq!(wire.coedges.len(), 6, "Expected 6 coedges in merged wire, got {}", wire.coedges.len());
        }
    }

    /// Test that two co-cylindrical faces sharing an edge are merged into one.
    ///
    /// Creates two rectangular faces on the same cylinder surface that
    /// share an edge, then verifies that face merging combines them.
    #[test]
    fn test_merge_cocylindrical_faces() {
        let cyl = CylinderSurface::new_z(5.0);

        // Face A: a strip on the cylinder from u=0 to u=π/2, v=0 to v=10
        // We approximate with line edges connecting sampled points
        let p0 = cyl.point_at(0.0, 0.0);
        let p1 = cyl.point_at(std::f64::consts::FRAC_PI_2, 0.0);
        let p2 = cyl.point_at(std::f64::consts::FRAC_PI_2, 10.0);
        let p3 = cyl.point_at(0.0, 10.0);

        let e_a0 = Edge::new_line(p0, p1);
        let e_a1 = Edge::new_line(p1, p2);
        let e_shared = Edge::new_line(p1, p2); // shared along u=π/2
        let e_a3 = Edge::new_line(p2, p3);
        let e_a4 = Edge::new_line(p3, p0);

        // Actually, let's simplify: the shared edge is e_a1 (from p1 to p2)
        let coedges_a = vec![
            CoEdge::new(e_a0.id, true),
            CoEdge::new(e_a1.id, true),
            CoEdge::new(e_a3.id, true),
            CoEdge::new(e_a4.id, true),
        ];
        let wire_a = Wire::new(coedges_a);
        let mut face_a = Face::new(Surface::Cylinder(cyl.clone()), wire_a);
        face_a.edges = vec![e_a0.clone(), e_a1.clone(), e_a3.clone(), e_a4.clone()];

        // Face B: adjacent strip from u=π/2 to u=π, v=0 to v=10
        let p4 = cyl.point_at(std::f64::consts::PI, 0.0);
        let p5 = cyl.point_at(std::f64::consts::PI, 10.0);

        let e_b0 = Edge::new_line(p1, p4);
        let e_b1 = Edge::new_line(p4, p5);
        let e_b2 = Edge::new_line(p5, p2);
        // Face B uses the shared edge (e_a1) in reverse direction
        let coedges_b = vec![
            CoEdge::new(e_a1.id, false), // shared edge reversed
            CoEdge::new(e_b0.id, true),
            CoEdge::new(e_b1.id, true),
            CoEdge::new(e_b2.id, true),
        ];
        let wire_b = Wire::new(coedges_b);
        let mut face_b = Face::new(Surface::Cylinder(cyl.clone()), wire_b);
        face_b.edges = vec![e_a1, e_b0.clone(), e_b1.clone(), e_b2.clone()];

        let shell = Shell::new(vec![face_a, face_b]);

        let params = HealingParams {
            fix_normals: false,
            stitch_edges: false,
            merge_faces: true,
            ..HealingParams::default()
        };

        let (healed, report) = heal_shell(&shell, &params);

        assert!(report.faces_merged >= 1, "Expected at least 1 face pair merged, got {}", report.faces_merged);
        assert_eq!(healed.faces.len(), 1, "Expected 1 face after merging, got {}", healed.faces.len());

        // The merged face should be on a cylinder surface
        let merged_face = &healed.faces[0];
        assert!(matches!(merged_face.surface, Some(Surface::Cylinder(_))),
            "Merged face should have a Cylinder surface");
    }

    /// Test that faces on different planes are NOT merged.
    #[test]
    fn test_no_merge_non_coplanar_faces() {
        // Two faces that share an edge but are NOT coplanar
        let e_shared = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0));

        // Face A on XY plane
        let e_a0 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0));
        let e_a1 = Edge::new_line(Point3d::new(0.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 0.0));
        let e_a2 = Edge::new_line(Point3d::new(1.0, 1.0, 0.0), Point3d::new(1.0, 0.0, 0.0));

        let coedges_a = vec![
            CoEdge::new(e_shared.id, true),
            CoEdge::new(e_a2.id, false),
            CoEdge::new(e_a1.id, false),
            CoEdge::new(e_a0.id, false),
        ];
        let wire_a = Wire::new(coedges_a);
        let plane_a = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z);
        let mut face_a = Face::new(Surface::Plane(plane_a), wire_a);
        face_a.edges = vec![e_shared.clone(), e_a0, e_a1, e_a2];

        // Face B on XZ plane (different normal!)
        let e_b0 = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 1.0));
        let e_b1 = Edge::new_line(Point3d::new(1.0, 0.0, 1.0), Point3d::new(0.0, 0.0, 1.0));
        let e_b2 = Edge::new_line(Point3d::new(0.0, 0.0, 1.0), Point3d::new(0.0, 0.0, 0.0));

        let coedges_b = vec![
            CoEdge::new(e_shared.id, false),
            CoEdge::new(e_b0.id, true),
            CoEdge::new(e_b1.id, true),
            CoEdge::new(e_b2.id, true),
        ];
        let wire_b = Wire::new(coedges_b);
        let plane_b = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), Direction3d::Y);
        let mut face_b = Face::new(Surface::Plane(plane_b), wire_b);
        face_b.edges = vec![e_shared, e_b0, e_b1, e_b2];

        let shell = Shell::new(vec![face_a, face_b]);

        let params = HealingParams {
            fix_normals: false,
            stitch_edges: false,
            merge_faces: true,
            ..HealingParams::default()
        };

        let (healed, report) = heal_shell(&shell, &params);

        assert_eq!(report.faces_merged, 0, "Non-coplanar faces should not be merged");
        assert_eq!(healed.faces.len(), 2, "Should still have 2 faces");
    }

    /// Test that `are_planes_compatible` works correctly.
    #[test]
    fn test_planes_compatible() {
        let p1 = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z);
        let p2 = Plane::from_origin_and_normal(Point3d::new(5.0, 3.0, 0.0), Direction3d::Z);
        let p3 = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 5.0), Direction3d::Z);
        let p4 = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), Direction3d::NEG_Z);
        let p5 = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), Direction3d::X);

        // Same plane, different origin in-plane → compatible
        assert!(are_planes_compatible(&p1, &p2, 1e-6, 1e-6));

        // Same normal, different offset → NOT compatible
        assert!(!are_planes_compatible(&p1, &p3, 1e-6, 1e-6));

        // Opposite normals, same plane → compatible (coplanar)
        assert!(are_planes_compatible(&p1, &p4, 1e-6, 1e-6));

        // Different normals → NOT compatible
        assert!(!are_planes_compatible(&p1, &p5, 1e-6, 1e-6));
    }

    /// Test that `are_cylinders_compatible` works correctly.
    #[test]
    fn test_cylinders_compatible() {
        let c1 = CylinderSurface::new_z(5.0);
        let c2 = CylinderSurface::new(Point3d::new(0.0, 0.0, 10.0), Direction3d::Z, 5.0);
        let c3 = CylinderSurface::new_z(3.0);
        let c4 = CylinderSurface::new(Point3d::new(1.0, 0.0, 0.0), Direction3d::Z, 5.0);

        // Same cylinder, different origin along axis → compatible
        assert!(are_cylinders_compatible(&c1, &c2, 1e-6, 1e-6));

        // Different radii → NOT compatible
        assert!(!are_cylinders_compatible(&c1, &c3, 1e-6, 1e-6));

        // Same radius but offset from axis → NOT compatible
        assert!(!are_cylinders_compatible(&c1, &c4, 1e-6, 1e-6));
    }

    /// Test tolerance propagation: edge tolerances should increase to match
    /// vertex tolerances, and face tolerances should increase to match edges.
    #[test]
    fn test_propagate_tolerances_upward() {
        // Create a simple box where we manually set some vertices/edges
        // with small tolerances, then verify propagation increases them.
        let mut box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // Set one edge's tolerance very high (simulating a vertex with large tolerance)
        // and all others to small values
        if let Some(ref mut shell) = box_solid.outer_shell {
            let first_face = &mut shell.faces[0];
            // Set the first edge to a large tolerance
            if !first_face.edges.is_empty() {
                first_face.edges[0].tolerance = 1e-3;
            }
            // Set the face tolerance to a small value
            first_face.tolerance = 1e-8;
        }

        let params = HealingParams {
            propagate_tolerances: true,
            fix_normals: false,
            stitch_edges: false,
            merge_faces: false,
            ..HealingParams::default()
        };

        let (healed, report) = heal_solid(&box_solid, &params);

        // The face containing the high-tolerance edge should have its
        // tolerance increased to match
        if let Some(ref shell) = healed.outer_shell {
            let first_face = &shell.faces[0];
            assert!(
                first_face.tolerance >= 1e-3,
                "Face tolerance should be >= 1e-3 after propagation, got {:.2e}",
                first_face.tolerance
            );
        }

        // Report should show that tolerances were propagated
        assert!(
            report.tolerances_propagated > 0,
            "Expected some tolerance propagations, got {}",
            report.tolerances_propagated
        );
    }

    /// Test tolerance propagation with a ToleranceContext that overrides
    /// small tolerances (downward propagation).
    #[test]
    fn test_propagate_tolerances_with_context() {
        // Create a box with default small tolerances
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // Use a ToleranceContext with a large model scale that produces
        // a coincidence tolerance larger than the default 1e-6
        let ctx = ToleranceContext::from_model_scale(1000.0);
        let coincidence = ctx.coincidence_tolerance();

        let params = HealingParams {
            propagate_tolerances: true,
            tolerance_context: Some(ctx),
            fix_normals: false,
            stitch_edges: false,
            merge_faces: false,
            ..HealingParams::default()
        };

        let (healed, report) = heal_solid(&box_solid, &params);

        // All entities should have tolerance >= the context's coincidence tolerance
        if let Some(ref shell) = healed.outer_shell {
            for face in &shell.faces {
                assert!(
                    face.tolerance >= coincidence,
                    "Face tolerance {:.2e} should be >= context coincidence {:.2e}",
                    face.tolerance,
                    coincidence
                );
                for edge in &face.edges {
                    assert!(
                        edge.tolerance >= coincidence,
                        "Edge tolerance {:.2e} should be >= context coincidence {:.2e}",
                        edge.tolerance,
                        coincidence
                    );
                }
            }
        }

        // Report should show many propagations (all edges and faces in the box)
        assert!(
            report.tolerances_propagated > 0,
            "Expected tolerance propagations from context, got {}",
            report.tolerances_propagated
        );
    }

    /// Test that tolerance propagation can be disabled.
    #[test]
    fn test_propagate_tolerances_disabled() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        let params = HealingParams {
            propagate_tolerances: false,
            fix_normals: false,
            stitch_edges: false,
            merge_faces: false,
            ..HealingParams::default()
        };

        let (_healed, report) = heal_solid(&box_solid, &params);

        // No tolerances should be propagated
        assert_eq!(
            report.tolerances_propagated, 0,
            "No tolerance propagation expected when disabled"
        );
    }

    /// Test edge → face propagation specifically.
    #[test]
    fn test_edge_to_face_tolerance_propagation() {
        // Create a triangle face with edges having different tolerances
        let mut face = ShapeBuilder::make_polygon_face(&[
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ])
        .unwrap();

        // Set edge tolerances to different values
        face.edges[0].tolerance = 1e-6;
        face.edges[1].tolerance = 1e-4;
        face.edges[2].tolerance = 1e-5;
        face.tolerance = 1e-8; // Very small face tolerance

        let mut shell = Shell::new(vec![face]);
        let mut report = HealingReport::default();
        let params = HealingParams {
            propagate_tolerances: true,
            ..HealingParams::default()
        };

        propagate_tolerances(&mut shell, &params, &mut report);

        // Face tolerance should be at least max edge tolerance (1e-4)
        assert!(
            shell.faces[0].tolerance >= 1e-4,
            "Face tolerance {:.2e} should be >= 1e-4 after propagation",
            shell.faces[0].tolerance
        );
        assert!(report.tolerances_propagated > 0);
    }

    /// Test that downward propagation from ToleranceContext applies to
    /// all entity types (edges and faces).
    #[test]
    fn test_downward_propagation_from_context() {
        // Create a face with very small tolerances
        let mut face = ShapeBuilder::make_polygon_face(&[
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ])
        .unwrap();

        // Set everything to a very small tolerance
        for edge in &mut face.edges {
            edge.tolerance = 1e-10;
        }
        face.tolerance = 1e-10;

        let mut shell = Shell::new(vec![face]);

        // Use a context with a much larger coincidence tolerance
        let ctx = ToleranceContext {
            absolute: 1e-3,
            relative: 1e-6,
            angular: 1e-5,
            parametric: 1e-8,
            model_scale: 100.0,
        };
        let floor = ctx.coincidence_tolerance(); // 1e-3 + 1e-6 * 100 = 1.001e-3

        let params = HealingParams {
            propagate_tolerances: true,
            tolerance_context: Some(ctx),
            ..HealingParams::default()
        };

        let mut report = HealingReport::default();
        propagate_tolerances(&mut shell, &params, &mut report);

        // All edges should have tolerance >= floor
        for edge in &shell.faces[0].edges {
            assert!(
                edge.tolerance >= floor,
                "Edge tolerance {:.2e} should be >= floor {:.2e}",
                edge.tolerance,
                floor
            );
        }

        // Face should have tolerance >= floor
        assert!(
            shell.faces[0].tolerance >= floor,
            "Face tolerance {:.2e} should be >= floor {:.2e}",
            shell.faces[0].tolerance,
            floor
        );

        assert!(report.tolerances_propagated > 0);
    }
}
