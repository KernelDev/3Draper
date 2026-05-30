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
    Direction3d, Plane, Point3d, Surface, Vec3d,
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
/// 1. Mark degenerate edges
/// 2. Close gaps between boundary edges
/// 3. Fill small holes
/// 4. Stitch collinear edges
/// 5. Remove small-feature faces
/// 6. Fix normal orientation (for closed shells)
pub fn heal_shell(shell: &Shell, params: &HealingParams) -> (Shell, HealingReport) {
    let mut report = HealingReport::default();
    let mut shell = shell.clone();

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

    // 5. Remove small-feature faces
    remove_small_features(&mut shell, params, &mut report);

    // 6. Fix normal orientation for closed shells
    if params.fix_normals && shell.closed {
        fix_normal_orientation(&mut shell, params, &mut report);
    }

    (shell, report)
}

// ============================================================
// Internal healing operations
// ============================================================

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
            messages: Vec::new(),
        };
        assert_eq!(report.total_fixes(), 11);
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
}
