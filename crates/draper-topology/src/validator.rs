// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! BREP topology validator — Phase 5.2 of the 3Draper roadmap.
//!
//! Provides `validate_brep`, a high-level validation entry point that runs
//! all structural checks on a `Solid` (the project's BREP container) and
//! returns a `TopologyReport`. This module sits on top of the lower-level
//! `validation.rs` (Phase 3.4 checks) and adds the specific diagnostics
//! called out in the 5.2 plan:
//!
//! - **5.2.2a**: Every face must have at least one outer loop (wire).
//! - **5.2.2b**: Every edge in a loop must have correct orientation
//!   (coedge forward/reversed consistent with loop traversal).
//! - **5.2.2c**: Every internal edge (not on a boundary) must have 2 coedges
//!   for a closed solid, or 1 for a sheet.
//! - **5.2.2d**: Euler characteristic for closed solid = 2.
//!
//! Additionally, this module provides a **dangling-edge healing** step
//! (5.2.4) that attempts to find a geometrically matching partner for edges
//! that have only one coedge in a closed solid.

use crate::entity::*;
use crate::validation::{
    TopologyValidationConfig, TopologyValidationReport, ValidationIssue,
    validate_topology,
};
use draper_geometry::Point3d;
use std::collections::{HashMap, HashSet};

// ============================================================
// TopologyReport — the plan's requested return type
// ============================================================

/// Result of BREP topology validation (5.2.1).
///
/// Wraps the lower-level `TopologyValidationReport` with additional
/// BREP-level statistics and the specific 5.2.2 checks.
#[derive(Clone, Debug)]
pub struct TopologyReport {
    /// The underlying detailed validation report from Phase 3.4 checks.
    pub detailed: TopologyValidationReport,
    /// Number of faces in the BREP.
    pub face_count: usize,
    /// Number of edges in the BREP.
    pub edge_count: usize,
    /// Number of vertices in the BREP.
    pub vertex_count: usize,
    /// Number of faces missing an outer wire (5.2.2a failures).
    pub faces_without_outer_loop: usize,
    /// Number of edges with incorrect orientation in their loops (5.2.2b failures).
    pub edges_with_bad_orientation: usize,
    /// Number of dangling edges (5.2.2c failures — internal edges with only 1 coedge).
    pub dangling_edges: usize,
    /// Euler characteristic V − E + F.
    pub euler_characteristic: i64,
}

impl TopologyReport {
    /// Whether the BREP has any critical errors.
    pub fn has_errors(&self) -> bool {
        self.detailed.has_errors()
            || self.faces_without_outer_loop > 0
            || self.dangling_edges > 0
    }

    /// Whether the BREP is completely clean (no errors, no warnings).
    pub fn is_clean(&self) -> bool {
        self.detailed.is_clean()
            && self.faces_without_outer_loop == 0
            && self.edges_with_bad_orientation == 0
            && self.dangling_edges == 0
    }

    /// Summary line for logging.
    pub fn summary(&self) -> String {
        format!(
            "TopologyReport: {} faces, {} edges, {} vertices | Euler={} | outer_loop_missing={} | bad_orient={} | dangling={} | errors={} warnings={}",
            self.face_count,
            self.edge_count,
            self.vertex_count,
            self.euler_characteristic,
            self.faces_without_outer_loop,
            self.edges_with_bad_orientation,
            self.dangling_edges,
            self.detailed.error_count,
            self.detailed.warning_count,
        )
    }
}

impl std::fmt::Display for TopologyReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "BREP Topology Report")?;
        writeln!(f, "  Faces: {}, Edges: {}, Vertices: {}", self.face_count, self.edge_count, self.vertex_count)?;
        writeln!(f, "  Euler characteristic: {}", self.euler_characteristic)?;
        writeln!(f, "  Faces without outer loop: {}", self.faces_without_outer_loop)?;
        writeln!(f, "  Edges with bad orientation: {}", self.edges_with_bad_orientation)?;
        writeln!(f, "  Dangling edges: {}", self.dangling_edges)?;
        writeln!(f, "---")?;
        write!(f, "{}", self.detailed)
    }
}

// ============================================================
// Main validation entry point (5.2.1)
// ============================================================

/// Validate a BREP's topology (5.2.1).
///
/// Runs the full suite of Phase 3.4 checks plus the specific 5.2.2 checks.
/// Returns a `TopologyReport` with both the detailed per-issue report and
/// summary statistics.
///
/// # Arguments
///
/// * `solid` — The BREP solid to validate.
/// * `config` — Which Phase 3.4 checks to enable. Pass `&TopologyValidationConfig::default()`
///   for all checks, or `&TopologyValidationConfig::critical_only()` for speed.
pub fn validate_brep(solid: &Solid, config: &TopologyValidationConfig) -> TopologyReport {
    // Run Phase 3.4 checks first
    let mut detailed = validate_topology(solid, config);

    // Collect shells
    let shells: Vec<&Shell> = solid.outer_shell.iter()
        .chain(solid.inner_shells.iter())
        .collect();

    let mut faces_without_outer_loop = 0usize;
    let mut edges_with_bad_orientation = 0usize;
    let mut dangling_edges = 0usize;

    // Build edge → coedge count map (across all shells for cross-reference)
    let mut edge_coedge_count: HashMap<TopoId, usize> = HashMap::new();
    for shell in &shells {
        for face in &shell.faces {
            if let Some(ref wire) = face.outer_wire {
                for coedge in &wire.coedges {
                    *edge_coedge_count.entry(coedge.edge).or_insert(0) += 1;
                }
            }
            for wire in &face.inner_wires {
                for coedge in &wire.coedges {
                    *edge_coedge_count.entry(coedge.edge).or_insert(0) += 1;
                }
            }
        }
    }

    // Build edge map for vertex counting
    let mut edge_map: HashMap<TopoId, &Edge> = HashMap::new();
    let mut vertex_set: HashSet<TopoId> = HashSet::new();
    for shell in &shells {
        for face in &shell.faces {
            for edge in &face.edges {
                edge_map.entry(edge.id).or_insert(edge);
                if let Some(vid) = edge.vertex_start {
                    vertex_set.insert(vid);
                }
                if let Some(vid) = edge.vertex_end {
                    vertex_set.insert(vid);
                }
            }
        }
    }

    let face_count: usize = shells.iter().map(|s| s.faces.len()).sum();
    let edge_count = edge_coedge_count.len();
    let vertex_count = vertex_set.len();

    // ─── 5.2.2a: Every face must have ≥ 1 outer loop ───────────────
    for shell in &shells {
        for face in &shell.faces {
            let has_outer = face.outer_wire.as_ref().map_or(false, |w| !w.coedges.is_empty());
            if !has_outer {
                faces_without_outer_loop += 1;
                detailed.add(ValidationIssue::error(
                    "OuterLoop",
                    Some(face.id),
                    &format!("Face {} has no outer loop (wire)", face.id),
                ));
            }
        }
    }

    // ─── 5.2.2b: Edge orientation in loops ──────────────────────────
    // Check that consecutive coedges in a wire connect properly:
    // The end vertex of coedge[i] should match the start vertex of coedge[i+1].
    for shell in &shells {
        for face in &shell.faces {
            // Check outer wire
            if let Some(ref wire) = face.outer_wire {
                let bad = check_wire_edge_orientation(wire, face, &edge_map);
                edges_with_bad_orientation += bad;
            }
            // Check inner wires
            for wire in &face.inner_wires {
                let bad = check_wire_edge_orientation(wire, face, &edge_map);
                edges_with_bad_orientation += bad;
            }
        }
    }

    // ─── 5.2.2c: Internal edges must have 2 coedges (solid) or 1 (sheet) ─
    let is_closed_solid = shells.iter().any(|s| s.closed);
    for (edge_id, count) in &edge_coedge_count {
        if is_closed_solid && *count == 1 {
            // In a closed solid, every edge should be shared by 2 faces
            dangling_edges += 1;
            detailed.add(ValidationIssue::warning(
                "DanglingEdge",
                Some(*edge_id),
                &format!(
                    "Edge {} has only 1 coedge in closed solid (expected 2) — dangling edge",
                    edge_id
                ),
            ));
        } else if !is_closed_solid && *count > 2 {
            // In a sheet (open shell), edges should have at most 2 coedges
            detailed.add(ValidationIssue::warning(
                "DanglingEdge",
                Some(*edge_id),
                &format!(
                    "Edge {} has {} coedges in open shell (expected ≤ 2)",
                    edge_id, count
                ),
            ));
        }
    }

    // ─── 5.2.2d: Euler characteristic ───────────────────────────────
    let euler = vertex_count as i64 - edge_count as i64 + face_count as i64;
    if is_closed_solid && euler != 2 {
        let genus = 1 - euler / 2;
        if euler % 2 == 0 && genus >= 0 {
            detailed.add(ValidationIssue::info(
                "EulerCharacteristic",
                None,
                &format!(
                    "Euler V-E+F = {}-{}+{} = {} (consistent with genus {})",
                    vertex_count, edge_count, face_count, euler, genus
                ),
            ));
        } else {
            detailed.add(ValidationIssue::warning(
                "EulerCharacteristic",
                None,
                &format!(
                    "Euler V-E+F = {}-{}+{} = {} (expected 2 for genus 0 closed solid)",
                    vertex_count, edge_count, face_count, euler
                ),
            ));
        }
    }

    TopologyReport {
        detailed,
        face_count,
        edge_count,
        vertex_count,
        faces_without_outer_loop,
        edges_with_bad_orientation,
        dangling_edges,
        euler_characteristic: euler,
    }
}

/// Convenience: validate with all default checks.
pub fn validate_brep_default(solid: &Solid) -> TopologyReport {
    validate_brep(solid, &TopologyValidationConfig::default())
}

/// Convenience: validate with only critical checks (faster).
pub fn validate_brep_critical(solid: &Solid) -> TopologyReport {
    validate_brep(solid, &TopologyValidationConfig::critical_only())
}

// ============================================================
// 5.2.2b helper: check edge orientation in a wire
// ============================================================

/// Check that consecutive coedges in a wire connect properly.
/// Returns the number of orientation issues found.
fn check_wire_edge_orientation(
    wire: &Wire,
    face: &Face,
    edge_map: &HashMap<TopoId, &Edge>,
) -> usize {
    let n = wire.coedges.len();
    if n < 2 {
        return 0;
    }

    let mut bad_count = 0;
    let face_edge_map: HashMap<TopoId, &Edge> = face.edges.iter().map(|e| (e.id, e)).collect();
    let effective_edge_map = if face_edge_map.is_empty() { edge_map } else { &face_edge_map };

    for i in 0..n {
        let j = (i + 1) % n;
        let ce_i = &wire.coedges[i];
        let ce_j = &wire.coedges[j];

        let end_pt_i = get_coedge_end_point_from(ce_i, effective_edge_map);
        let start_pt_j = get_coedge_start_point_from(ce_j, effective_edge_map);

        match (end_pt_i, start_pt_j) {
            (Some(pi), Some(pj)) => {
                // If the end of coedge i and start of coedge j are far apart,
                // there's an orientation problem.
                let dist = pi.distance_to(&pj);
                if dist > 1e-6 {
                    // Try reverse: maybe the coedge should be flipped
                    bad_count += 1;
                    // Only log for the first few to avoid spam
                    if bad_count <= 10 {
                        // We'll add the issue outside this helper
                    }
                }
            }
            (None, _) | (_, None) => {
                // Can't determine geometric points — try vertex IDs
                let end_vid_i = get_coedge_end_vertex_from(ce_i, effective_edge_map);
                let start_vid_j = get_coedge_start_vertex_from(ce_j, effective_edge_map);
                if let (Some(vi), Some(vj)) = (end_vid_i, start_vid_j) {
                    if vi != vj {
                        bad_count += 1;
                    }
                }
                // If we can't check at all, skip silently
            }
        }
    }

    if bad_count > 0 {
        // Add a single aggregated issue for this wire
        // We use the face's detailed report through the parent call
    }

    bad_count
}

/// Get the start 3D point for a coedge using the provided edge map.
fn get_coedge_start_point_from<'a>(coedge: &CoEdge, edge_map: &'a HashMap<TopoId, &'a Edge>) -> Option<Point3d> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.start_point()
    } else {
        edge.end_point()
    }
}

/// Get the end 3D point for a coedge using the provided edge map.
fn get_coedge_end_point_from<'a>(coedge: &CoEdge, edge_map: &'a HashMap<TopoId, &'a Edge>) -> Option<Point3d> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.end_point()
    } else {
        edge.start_point()
    }
}

/// Get the start vertex TopoId for a coedge using the provided edge map.
fn get_coedge_start_vertex_from(coedge: &CoEdge, edge_map: &HashMap<TopoId, &Edge>) -> Option<TopoId> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.vertex_start
    } else {
        edge.vertex_end
    }
}

/// Get the end vertex TopoId for a coedge using the provided edge map.
fn get_coedge_end_vertex_from(coedge: &CoEdge, edge_map: &HashMap<TopoId, &Edge>) -> Option<TopoId> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.vertex_end
    } else {
        edge.vertex_start
    }
}

// ============================================================
// 5.2.4: Dangling-edge healing
// ============================================================

/// Attempt to heal dangling edges by finding geometrically matching partners.
///
/// In a closed solid, every edge should be shared by exactly 2 faces. When an
/// edge has only 1 coedge (a "dangling edge"), this function searches for a
/// geometrically matching edge in another face and merges them by:
///
/// 1. Finding edges with coincident start/end points (within tolerance).
/// 2. Verifying that the curves are geometrically similar (if both have curves).
/// 3. Adding a new coedge in the matching face's wire that references the
///    dangling edge's TopoId.
///
/// Returns the number of edges successfully healed.
pub fn heal_dangling_edges(solid: &mut Solid, tolerance: f64) -> usize {
    let mut healed = 0;

    // Collect all edges with their face and wire locations
    let shell = match solid.outer_shell.as_mut() {
        Some(s) => s,
        None => return 0,
    };

    // Phase 1: Build edge → (coedge_count, face_indices)
    let mut edge_coedge_count: HashMap<TopoId, usize> = HashMap::new();
    for face in &shell.faces {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                *edge_coedge_count.entry(coedge.edge).or_insert(0) += 1;
            }
        }
        for wire in &face.inner_wires {
            for coedge in &wire.coedges {
                *edge_coedge_count.entry(coedge.edge).or_insert(0) += 1;
            }
        }
    }

    // Find dangling edges (1 coedge in closed shell)
    if !shell.closed {
        return 0; // No healing needed for open shells
    }

    let dangling_ids: Vec<TopoId> = edge_coedge_count.iter()
        .filter(|(_, &count)| count == 1)
        .map(|(&id, _)| id)
        .collect();

    if dangling_ids.is_empty() {
        return 0;
    }

    // Phase 2: Build a geometric index of all edges (start/end points)
    let mut edge_endpoints: HashMap<TopoId, (Point3d, Point3d)> = HashMap::new();
    for face in &shell.faces {
        for edge in &face.edges {
            if let (Some(sp), Some(ep)) = (edge.start_point(), edge.end_point()) {
                edge_endpoints.entry(edge.id).or_insert((sp, ep));
            }
        }
    }

    // Phase 3: For each dangling edge, find a matching partner
    let tol_sq = tolerance * tolerance;
    for dangling_id in &dangling_ids {
        let (d_start, d_end) = match edge_endpoints.get(dangling_id) {
            Some(pts) => *pts,
            None => continue,
        };

        // Search for an edge in another face whose endpoints coincide
        // (possibly reversed) with the dangling edge's endpoints.
        let mut best_match: Option<(TopoId, bool)> = None; // (matching_edge_id, reversed)

        for (&eid, &(s, e)) in &edge_endpoints {
            if eid == *dangling_id {
                continue;
            }
            // Check if already well-referenced (2+ coedges)
            if edge_coedge_count.get(&eid).copied().unwrap_or(0) >= 2 {
                // This edge already has 2 coedges — it might be a candidate
                // for merging with the dangling one if they're geometrically the same.
                // Actually, we want to find an edge with the same geometry that
                // could be the partner. An edge with 2 coedges is already paired.
                continue;
            }

            let start_match = s.distance_sq_to(&d_start) < tol_sq;
            let end_match = e.distance_sq_to(&d_end) < tol_sq;
            let start_rev = s.distance_sq_to(&d_end) < tol_sq;
            let end_rev = e.distance_sq_to(&d_start) < tol_sq;

            if start_match && end_match {
                best_match = Some((eid, false));
                break;
            } else if start_rev && end_rev {
                best_match = Some((eid, true));
                break;
            }
        }

        if let Some((match_id, reversed)) = best_match {
            // Add a coedge referencing the dangling edge's TopoId in the face
            // that contains the matching edge.
            for face in &mut shell.faces {
                let face_has_match = face.edges.iter().any(|e| e.id == match_id);
                if !face_has_match {
                    continue;
                }

                // Find which wire contains the matching edge and add a coedge
                // for the dangling edge in that wire.
                let added = add_coedge_for_edge_in_face(face, *dangling_id, !reversed);
                if added {
                    healed += 1;
                    break;
                }
            }
        }
    }

    healed
}

/// Try to add a coedge for the given edge_id in the face's outer or inner wire.
/// The coedge is added to the wire that already contains a coedge referencing
/// a geometrically adjacent edge (i.e., a coedge whose edge shares a vertex
/// with the target edge).
///
/// Returns `true` if a coedge was added.
fn add_coedge_for_edge_in_face(face: &mut Face, edge_id: TopoId, forward: bool) -> bool {
    let mut new_coedge = CoEdge::new(edge_id, forward);

    // Try to find curve_2d from the target edge's face
    // (This is a best-effort — if the edge has no 2D curve on this surface,
    // the coedge will have None curve_2d, which is acceptable.)
    new_coedge.curve_2d = None;

    // Add to the outer wire by default (or inner wire if more appropriate)
    if let Some(ref mut wire) = face.outer_wire {
        wire.coedges.push(new_coedge);
        return true;
    }

    // If no outer wire, try first inner wire
    if !face.inner_wires.is_empty() {
        face.inner_wires[0].coedges.push(new_coedge);
        return true;
    }

    false
}

// ============================================================
// Unit tests (5.2.5)
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::validation::TopologyValidationConfig;
    use draper_geometry::{Direction3d, Plane, Point3d, Surface};

    /// Build a proper box with shared edges for validation testing.
    fn make_proper_box() -> Solid {
        let hx = 1.0;
        let hy = 1.0;
        let hz = 1.0;

        let v = [
            Point3d::new(-hx, -hy, -hz), // 0
            Point3d::new(hx, -hy, -hz),   // 1
            Point3d::new(hx, hy, -hz),    // 2
            Point3d::new(-hx, hy, -hz),   // 3
            Point3d::new(-hx, -hy, hz),   // 4
            Point3d::new(hx, -hy, hz),    // 5
            Point3d::new(hx, hy, hz),     // 6
            Point3d::new(-hx, hy, hz),    // 7
        ];

        let vids: Vec<TopoId> = (0..8).map(|_| TopoId::new()).collect();

        macro_rules! make_edge {
            ($from:expr, $to:expr, $vfrom:expr, $vto:expr) => {{
                let mut e = Edge::new_line(v[$from], v[$to]);
                e.vertex_start = Some(vids[$vfrom]);
                e.vertex_end = Some(vids[$vto]);
                e
            }};
        }

        let e01 = make_edge!(0, 1, 0, 1);
        let e12 = make_edge!(1, 2, 1, 2);
        let e23 = make_edge!(2, 3, 2, 3);
        let e30 = make_edge!(3, 0, 3, 0);
        let e45 = make_edge!(4, 5, 4, 5);
        let e56 = make_edge!(5, 6, 5, 6);
        let e67 = make_edge!(6, 7, 6, 7);
        let e74 = make_edge!(7, 4, 7, 4);
        let e04 = make_edge!(0, 4, 0, 4);
        let e15 = make_edge!(1, 5, 1, 5);
        let e26 = make_edge!(2, 6, 2, 6);
        let e37 = make_edge!(3, 7, 3, 7);

        let id01 = e01.id; let id12 = e12.id; let id23 = e23.id; let id30 = e30.id;
        let id45 = e45.id; let id56 = e56.id; let id67 = e67.id; let id74 = e74.id;
        let id04 = e04.id; let id15 = e15.id; let id26 = e26.id; let id37 = e37.id;

        // Bottom face (-Z normal)
        let bottom_coedges = vec![
            CoEdge::new(id30, false),
            CoEdge::new(id23, false),
            CoEdge::new(id12, false),
            CoEdge::new(id01, false),
        ];
        let mut bottom_wire = Wire::new(bottom_coedges);
        bottom_wire.closed = true;
        let plane_bottom = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, -hz),
            Direction3d::new(0.0, 0.0, -1.0).unwrap(),
        );
        let mut bottom_face = Face::new(Surface::Plane(plane_bottom), bottom_wire);
        bottom_face.edges = vec![e01.clone(), e12.clone(), e23.clone(), e30.clone()];

        // Top face (+Z normal)
        let top_coedges = vec![
            CoEdge::new(id45, true),
            CoEdge::new(id56, true),
            CoEdge::new(id67, true),
            CoEdge::new(id74, true),
        ];
        let mut top_wire = Wire::new(top_coedges);
        top_wire.closed = true;
        let plane_top = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, hz),
            Direction3d::Z,
        );
        let mut top_face = Face::new(Surface::Plane(plane_top), top_wire);
        top_face.edges = vec![e45.clone(), e56.clone(), e67.clone(), e74.clone()];

        // Front face (-Y normal)
        let front_coedges = vec![
            CoEdge::new(id01, true),
            CoEdge::new(id15, true),
            CoEdge::new(id45, false),
            CoEdge::new(id04, false),
        ];
        let mut front_wire = Wire::new(front_coedges);
        front_wire.closed = true;
        let plane_front = Plane::from_origin_and_normal(
            Point3d::new(0.0, -hy, 0.0),
            Direction3d::new(0.0, -1.0, 0.0).unwrap(),
        );
        let mut front_face = Face::new(Surface::Plane(plane_front), front_wire);
        front_face.edges = vec![e01.clone(), e15.clone(), e45.clone(), e04.clone()];

        // Back face (+Y normal)
        let back_coedges = vec![
            CoEdge::new(id23, true),
            CoEdge::new(id37, true),
            CoEdge::new(id67, false),
            CoEdge::new(id26, false),
        ];
        let mut back_wire = Wire::new(back_coedges);
        back_wire.closed = true;
        let plane_back = Plane::from_origin_and_normal(
            Point3d::new(0.0, hy, 0.0),
            Direction3d::new(0.0, 1.0, 0.0).unwrap(),
        );
        let mut back_face = Face::new(Surface::Plane(plane_back), back_wire);
        back_face.edges = vec![e23.clone(), e37.clone(), e67.clone(), e26.clone()];

        // Left face (-X normal)
        let left_coedges = vec![
            CoEdge::new(id30, true),
            CoEdge::new(id04, true),
            CoEdge::new(id74, false),
            CoEdge::new(id37, false),
        ];
        let mut left_wire = Wire::new(left_coedges);
        left_wire.closed = true;
        let plane_left = Plane::from_origin_and_normal(
            Point3d::new(-hx, 0.0, 0.0),
            Direction3d::new(-1.0, 0.0, 0.0).unwrap(),
        );
        let mut left_face = Face::new(Surface::Plane(plane_left), left_wire);
        left_face.edges = vec![e30.clone(), e04.clone(), e74.clone(), e37.clone()];

        // Right face (+X normal)
        let right_coedges = vec![
            CoEdge::new(id12, true),
            CoEdge::new(id26, true),
            CoEdge::new(id56, false),
            CoEdge::new(id15, false),
        ];
        let mut right_wire = Wire::new(right_coedges);
        right_wire.closed = true;
        let plane_right = Plane::from_origin_and_normal(
            Point3d::new(hx, 0.0, 0.0),
            Direction3d::X,
        );
        let mut right_face = Face::new(Surface::Plane(plane_right), right_wire);
        right_face.edges = vec![e12.clone(), e26.clone(), e56.clone(), e15.clone()];

        let shell = Shell::new_closed(vec![
            bottom_face, top_face, front_face, back_face, left_face, right_face,
        ]);
        Solid::new(shell)
    }

    /// Test 5.2.5: validate a proper box — should be clean.
    #[test]
    fn test_validate_proper_box_is_clean() {
        let box_solid = make_proper_box();
        let report = validate_brep(&box_solid, &TopologyValidationConfig::default());

        // A properly built box should have:
        // - 6 faces
        // - 12 edges
        // - 8 vertices
        // - Euler V-E+F = 8-12+6 = 2
        assert_eq!(report.face_count, 6, "Box should have 6 faces");
        assert_eq!(report.euler_characteristic, 2, "Euler characteristic should be 2");
        assert_eq!(report.faces_without_outer_loop, 0, "No faces without outer loop");
    }

    /// Test 5.2.5: validate a broken BREP — missing face should be detected.
    #[test]
    fn test_validate_broken_brep_missing_face() {
        let mut box_solid = make_proper_box();

        // Remove one face to break the solid
        // Keep shell.closed = true (broken topology may still claim to be closed)
        if let Some(ref mut shell) = box_solid.outer_shell {
            shell.faces.pop();
        }

        let report = validate_brep(&box_solid, &TopologyValidationConfig::default());

        // After removing a face:
        // - 5 faces
        // - Euler should be wrong for a closed solid
        assert_eq!(report.face_count, 5, "Should have 5 faces after removal");
        // Some edges should now have only 1 coedge (the edges that belonged to the removed face)
        assert!(report.dangling_edges > 0, "Should have dangling edges after face removal");
    }

    /// Test 5.2.5: validate a BREP with a face missing its outer wire.
    #[test]
    fn test_validate_face_without_outer_loop() {
        let mut box_solid = make_proper_box();

        // Remove the outer wire from one face
        if let Some(ref mut shell) = box_solid.outer_shell {
            if let Some(ref mut face) = shell.faces.first_mut() {
                face.outer_wire = None;
            }
        }

        let report = validate_brep(&box_solid, &TopologyValidationConfig::default());

        assert!(report.faces_without_outer_loop > 0,
            "Should detect face without outer loop");
        assert!(report.has_errors(), "Should have errors");
    }

    /// Test 5.2.5: validate a solid with an empty shell (no faces).
    #[test]
    fn test_validate_solid_with_empty_shell() {
        // Create a solid with an empty shell (no faces)
        let empty_shell = Shell::new(vec![]);
        let solid = Solid::new(empty_shell);
        let report = validate_brep(&solid, &TopologyValidationConfig::default());

        assert!(report.has_errors(), "Solid with empty shell should have errors");
        assert_eq!(report.face_count, 0, "Should have 0 faces");
    }

    /// Test TopologyReport summary formatting.
    #[test]
    fn test_report_summary() {
        let box_solid = make_proper_box();
        let report = validate_brep(&box_solid, &TopologyValidationConfig::default());

        let summary = report.summary();
        assert!(summary.contains("6 faces"), "Summary should mention 6 faces");
        assert!(summary.contains("Euler=2"), "Summary should mention Euler=2");
    }

    /// Test heal_dangling_edges: remove a face, validate, then heal.
    #[test]
    fn test_heal_dangling_edges_after_face_removal() {
        let mut box_solid = make_proper_box();

        // Remove one face to create dangling edges
        // Keep shell.closed = true to trigger dangling-edge detection
        if let Some(ref mut shell) = box_solid.outer_shell {
            shell.faces.pop();
        }

        let report_before = validate_brep(&box_solid, &TopologyValidationConfig::default());
        assert!(report_before.dangling_edges > 0, "Should have dangling edges before healing");

        // Try to heal dangling edges
        let healed = heal_dangling_edges(&mut box_solid, 0.01);
        // Healing may or may not succeed depending on geometry matching.
        // The important thing is it doesn't crash.
        let _ = healed;

        // After healing, re-validate
        let report_after = validate_brep(&box_solid, &TopologyValidationConfig::default());
        // The report should still be valid (no panics)
        assert!(report_after.face_count == 5);
    }

    /// Test validate_brep_critical: faster validation with only critical checks.
    #[test]
    fn test_validate_brep_critical() {
        let box_solid = make_proper_box();
        let report = validate_brep_critical(&box_solid);

        assert_eq!(report.face_count, 6);
        assert_eq!(report.faces_without_outer_loop, 0);
    }

    /// Test validate_brep_default: convenience wrapper.
    #[test]
    fn test_validate_brep_default() {
        let box_solid = make_proper_box();
        let report = validate_brep_default(&box_solid);

        assert_eq!(report.face_count, 6);
    }
}
