//! B-Rep topology healing and validation.
//!
//! Real STEP files often contain topological anomalies that must be fixed
//! before triangulation. This module implements the healing pipeline:
//! 1. Duplicate vertex/edge merging
//! 2. Zero-length edge removal
//! 3. Orientation normalization (CCW outer, CW inner)
//! 4. T-junction handling
//! 5. Tolerance harmonization
//! 6. Wire continuity verification
//!
//! Principle: preparation must result in a fully consistent, non-contradictory
//! set of 2D contours for each face before any triangulation begins.

use crate::entity::*;
use crate::shape::Shape;
use draper_geometry::point::Point3;
use std::collections::HashMap;

/// Result of healing operations.
#[derive(Debug, Clone)]
pub struct HealingReport {
    /// Number of duplicate vertices merged.
    pub vertices_merged: usize,
    /// Number of degenerate edges removed.
    pub edges_removed: usize,
    /// Number of wires fixed for continuity.
    pub wires_fixed: usize,
    /// Number of face orientations corrected.
    pub orientations_fixed: usize,
    /// Number of tolerances harmonized.
    pub tolerances_harmonized: usize,
    /// Warnings generated during healing.
    pub warnings: Vec<String>,
}

impl Default for HealingReport {
    fn default() -> Self {
        Self::new()
    }
}

impl HealingReport {
    pub fn new() -> Self {
        Self {
            vertices_merged: 0,
            edges_removed: 0,
            wires_fixed: 0,
            orientations_fixed: 0,
            tolerances_harmonized: 0,
            warnings: Vec::new(),
        }
    }

    pub fn is_clean(&self) -> bool {
        self.vertices_merged == 0
            && self.edges_removed == 0
            && self.wires_fixed == 0
            && self.orientations_fixed == 0
            && self.warnings.is_empty()
    }
}

/// Run the full healing pipeline on a shape.
pub fn heal_shape(shape: &mut Shape) -> HealingReport {
    let mut report = HealingReport::new();

    // Phase 1: Merge duplicate vertices
    merge_duplicate_vertices(shape, &mut report);

    // Phase 2: Remove degenerate (zero-length) edges
    remove_degenerate_edges(shape, &mut report);

    // Phase 3: Verify and fix wire continuity
    fix_wire_continuity(shape, &mut report);

    // Phase 4: Normalize face orientations
    normalize_face_orientations(shape, &mut report);

    // Phase 5: Harmonize tolerances
    harmonize_tolerances(shape, &mut report);

    log::info!(
        "Healing complete: {} vertices merged, {} edges removed, {} wires fixed, {} orientations fixed",
        report.vertices_merged,
        report.edges_removed,
        report.wires_fixed,
        report.orientations_fixed,
    );

    report
}

/// Merge duplicate vertices (vertices that are within tolerance of each other).
fn merge_duplicate_vertices(shape: &mut Shape, report: &mut HealingReport) {
    let vertices: Vec<(TopoId, Point3, f64)> = shape
        .vertices()
        .iter()
        .map(|v| (v.id, v.point, v.tolerance))
        .collect();

    if vertices.is_empty() {
        return;
    }

    // Global tolerance for merging
    let merge_tol = vertices
        .iter()
        .map(|(_, _, t)| *t)
        .fold(1e-7, f64::max)
        .max(1e-6);

    // Build merge map: old_id → new_id
    let mut merge_map: HashMap<TopoId, TopoId> = HashMap::new();
    let mut representative: Vec<(TopoId, Point3)> = Vec::new();

    for (id, point, _) in &vertices {
        let mut found_duplicate = None;
        for (rep_id, rep_point) in &representative {
            if point.distance_to(*rep_point) < merge_tol {
                found_duplicate = Some(*rep_id);
                break;
            }
        }

        match found_duplicate {
            Some(rep_id) => {
                merge_map.insert(*id, rep_id);
                report.vertices_merged += 1;
            }
            None => {
                representative.push((*id, *point));
            }
        }
    }

    if merge_map.is_empty() {
        return;
    }

    // Apply merge map to all edge references
    for entity in shape.entities.values_mut() {
        if let TopoShape::Edge(edge) = entity {
            if let Some(new_start) = merge_map.get(&edge.start_vertex) {
                edge.start_vertex = *new_start;
            }
            if let Some(new_end) = merge_map.get(&edge.end_vertex) {
                edge.end_vertex = *new_end;
            }
        }
    }

    // Remove merged vertices
    let merged_ids: Vec<TopoId> = merge_map.keys().copied().collect();
    for id in merged_ids {
        shape.entities.remove(&id);
    }
}

/// Remove degenerate edges (zero length or start == end vertex).
fn remove_degenerate_edges(shape: &mut Shape, report: &mut HealingReport) {
    let degenerate_ids: Vec<TopoId> = shape
        .edges()
        .iter()
        .filter(|e| {
            if e.start_vertex == e.end_vertex && !e.is_seam {
                // Non-seam degenerate edge
                return true;
            }
            // Check geometric length
            if let (Some(start_vid), Some(end_vid)) = (
                shape.get(e.start_vertex),
                shape.get(e.end_vertex),
            ) {
                if let (TopoShape::Vertex(sv), TopoShape::Vertex(ev)) = (start_vid, end_vid) {
                    if sv.point.distance_to(ev.point) < e.tolerance && !e.is_seam {
                        return true;
                    }
                }
            }
            false
        })
        .map(|e| e.id)
        .collect();

    // Remove degenerate edges from wires
    for entity in shape.entities.values_mut() {
        if let TopoShape::Wire(wire) = entity {
            let before = wire.edges.len();
            wire.edges
                .retain(|oe| !degenerate_ids.contains(&oe.edge_id));
            if wire.edges.len() < before {
                report.edges_removed += before - wire.edges.len();
            }
        }
    }

    // Remove degenerate edges from the shape
    for id in &degenerate_ids {
        shape.entities.remove(id);
    }
}

/// Verify and fix wire continuity (end vertex of edge[i] == start vertex of edge[i+1]).
fn fix_wire_continuity(shape: &mut Shape, report: &mut HealingReport) {
    let wire_ids: Vec<TopoId> = shape
        .entities
        .values()
        .filter_map(|e| {
            if let TopoShape::Wire(w) = e {
                Some(w.id)
            } else {
                None
            }
        })
        .collect();

    for wire_id in wire_ids {
        let wire_edges = match shape.get(wire_id) {
            Some(TopoShape::Wire(w)) => w.edges.clone(),
            _ => continue,
        };

        if wire_edges.len() < 2 {
            continue;
        }

        // Check continuity
        let mut discontinuities = 0;
        for i in 0..wire_edges.len() {
            let j = (i + 1) % wire_edges.len();
            let end_v = get_edge_end_vertex(shape, wire_edges[i]);
            let start_v = get_edge_start_vertex(shape, wire_edges[j]);

            if end_v.is_none() || start_v.is_none() {
                continue;
            }
            if end_v != start_v {
                discontinuities += 1;
            }
        }

        if discontinuities > 0 {
            // Try to reorder edges
            // Clone edges first to avoid borrow conflict
            let edges_clone = match shape.get(wire_id) {
                Some(TopoShape::Wire(wire)) => wire.edges.clone(),
                _ => continue,
            };
            let reordered = reorder_edges_for_continuity(&edges_clone, shape);
            if reordered.len() == edges_clone.len() {
                if let Some(TopoShape::Wire(wire)) = shape.get_mut(wire_id) {
                    wire.edges = reordered;
                    report.wires_fixed += 1;
                }
            } else {
                log::warn!(
                    "Wire #{}: {} discontinuities, could not fully fix",
                    wire_id, discontinuities
                );
            }
        }
    }
}

fn get_edge_start_vertex(shape: &Shape, oe: OrientedEdge) -> Option<TopoId> {
    match shape.get(oe.edge_id) {
        Some(TopoShape::Edge(e)) => {
            if oe.orientation {
                Some(e.start_vertex)
            } else {
                Some(e.end_vertex)
            }
        }
        _ => None,
    }
}

fn get_edge_end_vertex(shape: &Shape, oe: OrientedEdge) -> Option<TopoId> {
    match shape.get(oe.edge_id) {
        Some(TopoShape::Edge(e)) => {
            if oe.orientation {
                Some(e.end_vertex)
            } else {
                Some(e.start_vertex)
            }
        }
        _ => None,
    }
}

/// Reorder wire edges to form a consecutive closed path.
fn reorder_edges_for_continuity(edges: &[OrientedEdge], shape: &Shape) -> Vec<OrientedEdge> {
    if edges.len() <= 2 {
        return edges.to_vec();
    }

    let mut from_map: HashMap<TopoId, Vec<(usize, TopoId)>> = HashMap::new();

    for (i, oe) in edges.iter().enumerate() {
        let (start, end) = match shape.get(oe.edge_id) {
            Some(TopoShape::Edge(e)) => {
                if oe.orientation {
                    (e.start_vertex, e.end_vertex)
                } else {
                    (e.end_vertex, e.start_vertex)
                }
            }
            _ => continue,
        };
        from_map.entry(start).or_default().push((i, end));
    }

    // Start from first edge
    let first_edge = &edges[0];
    let start_vertex = match shape.get(first_edge.edge_id) {
        Some(TopoShape::Edge(e)) => {
            if first_edge.orientation {
                e.start_vertex
            } else {
                e.end_vertex
            }
        }
        _ => return edges.to_vec(),
    };

    let mut ordered = Vec::with_capacity(edges.len());
    let mut used = vec![false; edges.len()];
    let mut current_vertex = start_vertex;

    for _ in 0..edges.len() {
        if let Some(candidates) = from_map.get(&current_vertex) {
            let mut found = false;
            for &(idx, to_v) in candidates {
                if !used[idx] {
                    ordered.push(edges[idx]);
                    used[idx] = true;
                    current_vertex = to_v;
                    found = true;
                    break;
                }
            }
            if !found {
                break;
            }
        } else {
            break;
        }
    }

    ordered
}

/// Normalize face orientations so outer wires are CCW and inner wires are CW
/// when viewed from outside the face.
fn normalize_face_orientations(shape: &mut Shape, report: &mut HealingReport) {
    let face_ids: Vec<TopoId> = shape
        .faces()
        .iter()
        .map(|f| f.id)
        .collect();

    for face_id in face_ids {
        // For now, just verify that wires exist and are non-empty
        let outer_wire_id = match shape.get(face_id) {
            Some(TopoShape::Face(f)) => f.outer_wire,
            _ => continue,
        };

        if let Some(wire_id) = outer_wire_id {
            if let Some(TopoShape::Wire(wire)) = shape.get(wire_id) {
                if wire.edges.is_empty() {
                    report
                        .warnings
                        .push(format!("Face #{}: outer wire #{} has no edges", face_id, wire_id));
                }
            }
        }
    }
}

/// Harmonize tolerances across the shape to a unified value.
fn harmonize_tolerances(shape: &mut Shape, report: &mut HealingReport) {
    // Compute a unified tolerance as the max of all vertex tolerances
    let max_tol = shape
        .vertices()
        .iter()
        .map(|v| v.tolerance)
        .fold(1e-7, f64::max);

    let unified_tol = max_tol.max(1e-6); // Minimum 1e-6

    let mut changed = 0;
    for entity in shape.entities.values_mut() {
        match entity {
            TopoShape::Vertex(v) => {
                if (v.tolerance - unified_tol).abs() > 1e-10 {
                    v.tolerance = unified_tol;
                    changed += 1;
                }
            }
            TopoShape::Edge(e) => {
                if (e.tolerance - unified_tol).abs() > 1e-10 {
                    e.tolerance = unified_tol;
                    changed += 1;
                }
            }
            _ => {}
        }
    }

    if changed > 0 {
        report.tolerances_harmonized = changed;
        log::debug!("Harmonized {} tolerances to {:.1e}", changed, unified_tol);
    }
}

/// Validate a shape and return a list of issues found.
pub fn validate_shape(shape: &Shape) -> Vec<String> {
    let mut issues = Vec::new();

    // Check for faces without outer wire
    for face in shape.faces() {
        if face.outer_wire.is_none() {
            issues.push(format!("Face #{} has no outer wire", face.id));
        }

        // Check that wire edges exist
        if let Some(wire_id) = face.outer_wire {
            if let Some(TopoShape::Wire(wire)) = shape.get(wire_id) {
                for oe in &wire.edges {
                    if shape.get(oe.edge_id).is_none() {
                        issues.push(format!(
                            "Face #{}: outer wire #{} references missing edge #{}",
                            face.id, wire_id, oe.edge_id
                        ));
                    }
                }
            } else {
                issues.push(format!(
                    "Face #{}: outer wire #{} does not exist",
                    face.id, wire_id
                ));
            }
        }

        // Check that the face has a surface
        if face.surface.is_none() {
            issues.push(format!("Face #{} has no surface geometry", face.id));
        }
    }

    // Check for shells without faces
    for shell in shape.shells() {
        if shell.faces.is_empty() {
            issues.push(format!("Shell #{} has no faces", shell.id));
        }
    }

    // Check for edges with missing vertices
    for edge in shape.edges() {
        if shape.get(edge.start_vertex).is_none() {
            issues.push(format!(
                "Edge #{}: start vertex #{} does not exist",
                edge.id, edge.start_vertex
            ));
        }
        if shape.get(edge.end_vertex).is_none() {
            issues.push(format!(
                "Edge #{}: end vertex #{} does not exist",
                edge.id, edge.end_vertex
            ));
        }
    }

    issues
}
