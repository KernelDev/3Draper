// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Multi-level topology validation (Task 3.4).
//!
//! Provides configurable, severity-annotated validation checks for B-Rep topology:
//!
//! - **ShellClosure** (3.4.1): Every edge in a closed shell is shared by exactly 2 faces.
//! - **FaceOrientation** (3.4.2): Adjacent faces have consistent outward normals.
//! - **EdgeManifoldness** (3.4.3): No edge belongs to more than 2 faces.
//! - **VertexConnectivity** (3.4.4): Edges around each vertex form a closed cycle.
//! - **WireClosure** (3.4.5): Each outer wire is closed.
//! - **LoopOrientation** (3.4.6): Outer loops CCW, inner loops CW.
//! - **GeometricConsistency** (3.4.7): Edge 3D points lie on the face surface.
//! - **EulerCharacteristic** (3.4.8): V − E + F = 2(1 − genus) for closed shells.
//!
//! # Severity levels (3.4.10)
//!
//! | Severity | Meaning |
//! |----------|---------|
//! | **Error** | Critical — prevents correct operation (non-manifold, unclosed wire in closed shell) |
//! | **Warning** | May cause problems but can be worked around (orientation mismatch, Euler mismatch) |
//! | **Info** | Informational (tolerance adjustments, degenerate edges) |

use crate::entity::*;
use draper_geometry::{Point2d, Point3d, Surface};
use std::collections::{HashMap, HashSet};
use draper_geometry::tolerance::TOLERANCE;

// ============================================================
// Legacy types and functions (backward compatibility)
// ============================================================

/// Validation error (legacy).
#[derive(Debug, Clone)]
pub enum ValidationError {
    EmptyShell,
    UnclosedWire,
    DisconnectedEdges,
    MissingGeometry,
    DegenerateFace,
    OrientationConflict,
    VertexCoincidence,
    /// An edge with a degenerate curve (zero length, zero radius, etc.)
    DegenerateEdge(TopoId),
}

/// Validate a solid's topology (legacy, mutable — sets degenerate flags).
///
/// This checks for structural and geometric issues:
/// - Empty shells
/// - Missing geometry
/// - Degenerate faces
/// - Degenerate edges (zero-length, zero-radius curves)
///
/// After validation, degenerate edges will have their `degenerate` flag set.
pub fn validate_solid(solid: &mut Solid) -> Vec<ValidationError> {
    let mut errors = Vec::new();

    if solid.outer_shell.is_none() {
        errors.push(ValidationError::EmptyShell);
        return errors;
    }

    if let Some(ref mut shell) = solid.outer_shell {
        if shell.faces.is_empty() {
            errors.push(ValidationError::EmptyShell);
        }

        for face in &mut shell.faces {
            if face.surface.is_none() {
                errors.push(ValidationError::MissingGeometry);
            }

            if let Some(ref wire) = face.outer_wire {
                if wire.coedges.is_empty() {
                    errors.push(ValidationError::DegenerateFace);
                }
            }

            // Check edges for degeneracy
            for edge in &mut face.edges {
                if !edge.degenerate {
                    if let Some(ref curve) = edge.curve {
                        if curve.is_degenerate(edge.tolerance) {
                            edge.degenerate = true;
                            errors.push(ValidationError::DegenerateEdge(edge.id));
                        }
                    } else {
                        // No curve geometry — check if start and end are coincident
                        if let (Some(sp), Some(ep)) = (edge.start_point(), edge.end_point()) {
                            let dx = sp.x - ep.x;
                            let dy = sp.y - ep.y;
                            let dz = sp.z - ep.z;
                            if (dx * dx + dy * dy + dz * dz) < edge.tolerance * edge.tolerance {
                                edge.degenerate = true;
                                errors.push(ValidationError::DegenerateEdge(edge.id));
                            }
                        }
                    }
                }
            }
        }
    }

    errors
}

/// Validate a solid's topology (read-only version — does not set degenerate flags).
pub fn validate_solid_readonly(solid: &Solid) -> Vec<ValidationError> {
    let mut errors = Vec::new();

    if solid.outer_shell.is_none() {
        errors.push(ValidationError::EmptyShell);
        return errors;
    }

    if let Some(ref shell) = solid.outer_shell {
        if shell.faces.is_empty() {
            errors.push(ValidationError::EmptyShell);
        }

        for face in &shell.faces {
            if face.surface.is_none() {
                errors.push(ValidationError::MissingGeometry);
            }

            if let Some(ref wire) = face.outer_wire {
                if wire.coedges.is_empty() {
                    errors.push(ValidationError::DegenerateFace);
                }
            }

            for edge in &face.edges {
                if edge.degenerate {
                    errors.push(ValidationError::DegenerateEdge(edge.id));
                } else if let Some(ref curve) = edge.curve {
                    if curve.is_degenerate(edge.tolerance) {
                        errors.push(ValidationError::DegenerateEdge(edge.id));
                    }
                }
            }
        }
    }

    errors
}

/// Validate a shell (legacy).
pub fn validate_shell(shell: &Shell) -> Vec<ValidationError> {
    let mut errors = Vec::new();

    if shell.faces.is_empty() {
        errors.push(ValidationError::EmptyShell);
    }

    for face in &shell.faces {
        if face.surface.is_none() {
            errors.push(ValidationError::MissingGeometry);
        }
    }

    errors
}

/// Heal a solid: fix common topological issues (legacy).
///
/// This performs the following healing operations:
/// 1. Remove degenerate faces (no surface or empty wire)
/// 2. Mark degenerate edges (zero-length or degenerate curves)
///
/// Degenerate edges are NOT removed — they may still carry topological
/// meaning (e.g., a seam edge at a cone apex). Instead, they are flagged
/// so that downstream code (triangulation) can handle them specially.
pub fn heal_solid(solid: &mut Solid) -> Vec<String> {
    let mut fixes = Vec::new();

    if let Some(ref mut shell) = solid.outer_shell {
        // Mark degenerate edges
        let mut degenerate_count = 0;
        for face in &mut shell.faces {
            for edge in &mut face.edges {
                if !edge.degenerate {
                    let is_degen = if let Some(ref curve) = edge.curve {
                        curve.is_degenerate(edge.tolerance)
                    } else {
                        // No curve — check if start/end points coincide
                        if let (Some(sp), Some(ep)) = (edge.start_point(), edge.end_point()) {
                            let dx = sp.x - ep.x;
                            let dy = sp.y - ep.y;
                            let dz = sp.z - ep.z;
                            (dx * dx + dy * dy + dz * dz) < edge.tolerance * edge.tolerance
                        } else {
                            false
                        }
                    };
                    if is_degen {
                        edge.degenerate = true;
                        degenerate_count += 1;
                    }
                }
            }
        }
        if degenerate_count > 0 {
            fixes.push(format!("Marked {} degenerate edges", degenerate_count));
        }

        // Remove degenerate faces
        let before = shell.faces.len();
        shell.faces.retain(|f| {
            f.surface.is_some() &&
            f.outer_wire.as_ref().map_or(true, |w| !w.coedges.is_empty())
        });
        if shell.faces.len() < before {
            fixes.push(format!("Removed {} degenerate faces", before - shell.faces.len()));
        }
    }

    fixes
}

// ============================================================
// New validation types (3.4.9, 3.4.10)
// ============================================================

/// Severity level for validation issues (3.4.10).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Severity {
    /// Critical — prevents correct operation (non-manifold, unclosed wire in closed shell).
    Error,
    /// May cause problems but can be worked around (orientation mismatch, Euler mismatch).
    Warning,
    /// Informational (tolerance adjustments, degenerate edges).
    Info,
}

impl std::fmt::Display for Severity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Severity::Error => write!(f, "Error"),
            Severity::Warning => write!(f, "Warning"),
            Severity::Info => write!(f, "Info"),
        }
    }
}

/// A single validation issue.
#[derive(Clone, Debug)]
pub struct ValidationIssue {
    /// Severity of the issue.
    pub severity: Severity,
    /// Which check found this issue (e.g., "ShellClosure", "EdgeManifoldness").
    pub check: String,
    /// Which entity has the issue (if identifiable).
    pub entity_id: Option<TopoId>,
    /// Human-readable description.
    pub message: String,
}

impl ValidationIssue {
    /// Create a new validation issue.
    pub fn new(severity: Severity, check: &str, entity_id: Option<TopoId>, message: &str) -> Self {
        Self {
            severity,
            check: check.to_string(),
            entity_id,
            message: message.to_string(),
        }
    }

    /// Create an Error-level issue.
    pub fn error(check: &str, entity_id: Option<TopoId>, message: &str) -> Self {
        Self::new(Severity::Error, check, entity_id, message)
    }

    /// Create a Warning-level issue.
    pub fn warning(check: &str, entity_id: Option<TopoId>, message: &str) -> Self {
        Self::new(Severity::Warning, check, entity_id, message)
    }

    /// Create an Info-level issue.
    pub fn info(check: &str, entity_id: Option<TopoId>, message: &str) -> Self {
        Self::new(Severity::Info, check, entity_id, message)
    }
}

/// Configuration for which validation checks to run (3.4.9).
#[derive(Clone, Debug)]
pub struct TopologyValidationConfig {
    /// Check that every edge in a closed shell is shared by exactly 2 faces.
    pub check_shell_closure: bool,
    /// Check that face normals point outward for closed shells.
    pub check_face_orientation: bool,
    /// Check that no edge belongs to more than 2 faces.
    pub check_edge_manifoldness: bool,
    /// Check that edges around each vertex form a closed cycle.
    pub check_vertex_connectivity: bool,
    /// Check that each outer wire is closed.
    pub check_wire_closure: bool,
    /// Check that outer loops wind CCW and inner loops wind CW.
    pub check_loop_orientation: bool,
    /// Check that edge 3D points lie on the face surface.
    pub check_geometric_consistency: bool,
    /// Check Euler characteristic for closed shells.
    pub check_euler_characteristic: bool,
}

impl Default for TopologyValidationConfig {
    fn default() -> Self {
        Self {
            check_shell_closure: true,
            check_face_orientation: true,
            check_edge_manifoldness: true,
            check_vertex_connectivity: true,
            check_wire_closure: true,
            check_loop_orientation: true,
            check_geometric_consistency: true,
            check_euler_characteristic: true,
        }
    }
}

impl TopologyValidationConfig {
    /// Run only critical checks (shell closure, edge manifoldness, wire closure).
    pub fn critical_only() -> Self {
        Self {
            check_shell_closure: true,
            check_face_orientation: false,
            check_edge_manifoldness: true,
            check_vertex_connectivity: false,
            check_wire_closure: true,
            check_loop_orientation: false,
            check_geometric_consistency: false,
            check_euler_characteristic: false,
        }
    }

    /// Run all checks.
    pub fn all() -> Self {
        Self::default()
    }

    /// Run no checks (produces empty report).
    pub fn none() -> Self {
        Self {
            check_shell_closure: false,
            check_face_orientation: false,
            check_edge_manifoldness: false,
            check_vertex_connectivity: false,
            check_wire_closure: false,
            check_loop_orientation: false,
            check_geometric_consistency: false,
            check_euler_characteristic: false,
        }
    }
}

/// Result of topology validation.
#[derive(Clone, Debug, Default)]
pub struct TopologyValidationReport {
    /// All issues found during validation.
    pub issues: Vec<ValidationIssue>,
    /// Number of Error-level issues.
    pub error_count: usize,
    /// Number of Warning-level issues.
    pub warning_count: usize,
    /// Number of Info-level issues.
    pub info_count: usize,
}

impl TopologyValidationReport {
    /// Create an empty report.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add an issue to the report and update counters.
    pub fn add(&mut self, issue: ValidationIssue) {
        match issue.severity {
            Severity::Error => self.error_count += 1,
            Severity::Warning => self.warning_count += 1,
            Severity::Info => self.info_count += 1,
        }
        self.issues.push(issue);
    }

    /// Whether the report contains any errors.
    pub fn has_errors(&self) -> bool {
        self.error_count > 0
    }

    /// Whether the report is completely clean.
    pub fn is_clean(&self) -> bool {
        self.issues.is_empty()
    }

    /// Filter issues by severity.
    pub fn issues_with_severity(&self, severity: &Severity) -> Vec<&ValidationIssue> {
        self.issues.iter().filter(|i| &i.severity == severity).collect()
    }

    /// Filter issues by check name.
    pub fn issues_for_check(&self, check: &str) -> Vec<&ValidationIssue> {
        self.issues.iter().filter(|i| i.check == check).collect()
    }
}

impl std::fmt::Display for TopologyValidationReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Topology Validation Report")?;
        writeln!(f, "  Errors: {}, Warnings: {}, Info: {}", self.error_count, self.warning_count, self.info_count)?;
        for issue in &self.issues {
            let eid = issue.entity_id.map(|id| format!(" {}", id)).unwrap_or_default();
            writeln!(f, "  [{}] {}{}: {}", issue.severity, issue.check, eid, issue.message)?;
        }
        Ok(())
    }
}

// ============================================================
// Main validation function
// ============================================================

/// Validate a solid's topology with configurable checks.
///
/// Runs the enabled checks from `config` on all shells in the solid
/// and returns a report with severity-annotated issues.
pub fn validate_topology(solid: &Solid, config: &TopologyValidationConfig) -> TopologyValidationReport {
    let mut report = TopologyValidationReport::new();

    // Also run basic legacy checks
    if solid.outer_shell.is_none() {
        report.add(ValidationIssue::error(
            "BasicStructure",
            None,
            "Solid has no outer shell",
        ));
        return report;
    }

    // Validate all shells
    let shells: Vec<&Shell> = solid.outer_shell.iter()
        .chain(solid.inner_shells.iter())
        .collect();

    for shell in &shells {
        validate_shell_internal(shell, config, &mut report);
    }

    report
}

/// Validate a single shell with the given config.
fn validate_shell_internal(
    shell: &Shell,
    config: &TopologyValidationConfig,
    report: &mut TopologyValidationReport,
) {
    if shell.faces.is_empty() {
        report.add(ValidationIssue::error(
            "BasicStructure",
            Some(shell.id),
            "Shell has no faces",
        ));
        return;
    }

    // Check for missing geometry and degenerate faces
    for face in &shell.faces {
        if face.surface.is_none() {
            report.add(ValidationIssue::error(
                "BasicStructure",
                Some(face.id),
                "Face has no surface geometry",
            ));
        }
        if let Some(ref wire) = face.outer_wire {
            if wire.coedges.is_empty() {
                report.add(ValidationIssue::warning(
                    "BasicStructure",
                    Some(face.id),
                    "Face has an empty outer wire",
                ));
            }
        }
        // Check for degenerate edges
        for edge in &face.edges {
            if edge.degenerate {
                report.add(ValidationIssue::info(
                    "DegenerateEdge",
                    Some(edge.id),
                    &format!("Edge {} is degenerate (zero-length or degenerate curve)", edge.id),
                ));
            }
        }
    }

    // Build the edge-to-coedge map for topology checks
    let edge_coedge_map = build_edge_coedge_map(shell);
    let edge_map = build_edge_map(shell);

    // 3.4.1 Shell Closure
    if config.check_shell_closure {
        check_shell_closure(shell, &edge_coedge_map, report);
    }

    // 3.4.2 Face Orientation
    if config.check_face_orientation {
        check_face_orientation(shell, &edge_coedge_map, report);
    }

    // 3.4.3 Edge Manifoldness
    if config.check_edge_manifoldness {
        check_edge_manifoldness(shell, &edge_coedge_map, report);
    }

    // 3.4.4 Vertex Connectivity
    if config.check_vertex_connectivity {
        check_vertex_connectivity(shell, &edge_map, report);
    }

    // 3.4.5 Wire Closure
    if config.check_wire_closure {
        check_wire_closure(shell, &edge_map, report);
    }

    // 3.4.6 Loop Orientation
    if config.check_loop_orientation {
        check_loop_orientation(shell, report);
    }

    // 3.4.7 Geometric Consistency
    if config.check_geometric_consistency {
        check_geometric_consistency(shell, &edge_map, report);
    }

    // 3.4.8 Euler Characteristic
    if config.check_euler_characteristic {
        check_euler_characteristic(shell, &edge_coedge_map, &edge_map, report);
    }
}

// ============================================================
// Helper: build topology maps
// ============================================================

/// Information about a coedge reference: which face and which wire it belongs to,
/// plus the coedge's orientation.
#[derive(Clone, Debug)]
struct CoedgeInfo {
    face_id: TopoId,
    #[allow(dead_code)]
    face_index: usize,
    #[allow(dead_code)]
    edge_id: TopoId,
    forward: bool,
}

/// Build a map from edge TopoId → list of CoedgeInfo for all coedges in the shell
/// that reference that edge.
fn build_edge_coedge_map(shell: &Shell) -> HashMap<TopoId, Vec<CoedgeInfo>> {
    let mut map: HashMap<TopoId, Vec<CoedgeInfo>> = HashMap::new();

    for (fi, face) in shell.faces.iter().enumerate() {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                map.entry(coedge.edge).or_default().push(CoedgeInfo {
                    face_id: face.id,
                    face_index: fi,
                    edge_id: coedge.edge,
                    forward: coedge.forward,
                });
            }
        }
        for wire in &face.inner_wires {
            for coedge in &wire.coedges {
                map.entry(coedge.edge).or_default().push(CoedgeInfo {
                    face_id: face.id,
                    face_index: fi,
                    edge_id: coedge.edge,
                    forward: coedge.forward,
                });
            }
        }
    }

    map
}

/// Build a map from edge TopoId → Edge object (first occurrence in the shell).
fn build_edge_map(shell: &Shell) -> HashMap<TopoId, Edge> {
    let mut map: HashMap<TopoId, Edge> = HashMap::new();
    for face in &shell.faces {
        for edge in &face.edges {
            map.entry(edge.id).or_insert_with(|| edge.clone());
        }
    }
    map
}

// ============================================================
// 3.4.1 Shell Closure
// ============================================================

/// Check that for a closed shell, every edge (by TopoId in coedges) is referenced
/// by exactly 2 coedges from different faces.
fn check_shell_closure(
    shell: &Shell,
    edge_coedge_map: &HashMap<TopoId, Vec<CoedgeInfo>>,
    report: &mut TopologyValidationReport,
) {
    if !shell.closed {
        // Shell is not closed — skip closure check (open shells have boundary edges)
        return;
    }

    for (edge_id, coedges) in edge_coedge_map {
        let face_count = coedges.iter().map(|c| c.face_id).collect::<HashSet<_>>().len();

        if face_count == 0 {
            continue; // Shouldn't happen
        } else if face_count == 1 {
            // Edge is used by only one face — boundary edge in a closed shell
            report.add(ValidationIssue::error(
                "ShellClosure",
                Some(*edge_id),
                &format!(
                    "Edge {} is referenced by only 1 face in closed shell (expected 2)",
                    edge_id
                ),
            ));
        } else if face_count == 2 {
            // Correct for closed shell
        } else {
            // Referenced by more than 2 faces — this is also a closure problem
            // (but also an edge manifoldness problem)
            report.add(ValidationIssue::warning(
                "ShellClosure",
                Some(*edge_id),
                &format!(
                    "Edge {} is referenced by {} faces in closed shell (expected 2)",
                    edge_id, face_count
                ),
            ));
        }
    }
}

// ============================================================
// 3.4.2 Face Orientation
// ============================================================

/// For a closed shell, verify that adjacent faces have consistent normal orientation
/// at shared edges. When two faces share an edge, their coedges should traverse
/// the edge in opposite directions (one forward, one reversed). This ensures
/// that both face normals point outward.
fn check_face_orientation(
    shell: &Shell,
    edge_coedge_map: &HashMap<TopoId, Vec<CoedgeInfo>>,
    report: &mut TopologyValidationReport,
) {
    if !shell.closed {
        return;
    }

    for (edge_id, coedges) in edge_coedge_map {
        if coedges.len() != 2 {
            continue; // Only check pairs
        }

        let face_ids: HashSet<TopoId> = coedges.iter().map(|c| c.face_id).collect();
        if face_ids.len() != 2 {
            continue; // Same face on both sides — skip
        }

        // Check that the two coedges have opposite orientations
        let forward_count = coedges.iter().filter(|c| c.forward).count();
        if forward_count == 2 || forward_count == 0 {
            // Both coedges traverse the edge in the same direction — inconsistent orientation
            report.add(ValidationIssue::warning(
                "FaceOrientation",
                Some(*edge_id),
                &format!(
                    "Edge {} is traversed in the same direction by both adjacent faces \
                     (both {}), suggesting inconsistent face orientation",
                    edge_id,
                    if forward_count == 2 { "forward" } else { "reversed" }
                ),
            ));
        }
        // If one is forward and one is reversed, orientation is consistent ✓
    }
}

// ============================================================
// 3.4.3 Edge Manifoldness
// ============================================================

/// Check that no edge is shared by more than 2 faces (non-manifold condition).
fn check_edge_manifoldness(
    _shell: &Shell,
    edge_coedge_map: &HashMap<TopoId, Vec<CoedgeInfo>>,
    report: &mut TopologyValidationReport,
) {
    for (edge_id, coedges) in edge_coedge_map {
        let face_count = coedges.iter().map(|c| c.face_id).collect::<HashSet<_>>().len();

        if face_count > 2 {
            report.add(ValidationIssue::error(
                "EdgeManifoldness",
                Some(*edge_id),
                &format!(
                    "Edge {} is shared by {} faces (non-manifold, max 2 allowed)",
                    edge_id, face_count
                ),
            ));
        }
    }
}

// ============================================================
// 3.4.4 Vertex Connectivity
// ============================================================

/// For each vertex, the edges incident to it should form a closed cycle
/// (or a single loop for boundary vertices).
fn check_vertex_connectivity(
    shell: &Shell,
    edge_map: &HashMap<TopoId, Edge>,
    report: &mut TopologyValidationReport,
) {
    // Build vertex → incident edges map
    let mut vertex_edges: HashMap<TopoId, Vec<TopoId>> = HashMap::new();

    for (edge_id, edge) in edge_map {
        if let Some(vid) = edge.vertex_start {
            vertex_edges.entry(vid).or_default().push(*edge_id);
        }
        if let Some(vid) = edge.vertex_end {
            vertex_edges.entry(vid).or_default().push(*edge_id);
        }
    }

    // For closed shells, each vertex should have at least 3 incident edges
    // and the edges should form a connected ring.
    for (vertex_id, edge_ids) in &vertex_edges {
        let num_edges = edge_ids.len();

        if num_edges == 0 {
            report.add(ValidationIssue::warning(
                "VertexConnectivity",
                Some(*vertex_id),
                &format!("Vertex {} has no incident edges", vertex_id),
            ));
        } else if num_edges == 1 {
            // A vertex with only one incident edge is a dangling vertex
            if shell.closed {
                report.add(ValidationIssue::error(
                    "VertexConnectivity",
                    Some(*vertex_id),
                    &format!(
                        "Vertex {} has only 1 incident edge in closed shell (dangling vertex)",
                        vertex_id
                    ),
                ));
            } else {
                report.add(ValidationIssue::warning(
                    "VertexConnectivity",
                    Some(*vertex_id),
                    &format!(
                        "Vertex {} has only 1 incident edge (dangling vertex)",
                        vertex_id
                    ),
                ));
            }
        } else if num_edges == 2 {
            // In a closed shell, a vertex with exactly 2 incident edges means
            // only 2 faces meet at that vertex, which is unusual but can happen
            // at the seam of a cylinder or sphere.
            // This is informational, not an error.
            if shell.closed {
                report.add(ValidationIssue::info(
                    "VertexConnectivity",
                    Some(*vertex_id),
                    &format!(
                        "Vertex {} has exactly 2 incident edges in closed shell",
                        vertex_id
                    ),
                ));
            }
        }

        // Check that the edges form a connected ring by verifying that
        // each pair of consecutive edges in the ring shares a common face.
        // Simplified check: verify that each edge at this vertex can be
        // reached from any other edge through face adjacency.
        if num_edges >= 3 && shell.closed {
            // Build adjacency: two edges are adjacent if they share a face
            // For a proper closed vertex, all edges should be connected
            let edge_coedge_map = build_edge_coedge_map(shell);
            let connected = check_vertex_ring_connectivity(edge_ids, &edge_coedge_map);
            if !connected {
                report.add(ValidationIssue::warning(
                    "VertexConnectivity",
                    Some(*vertex_id),
                    &format!(
                        "Edges around vertex {} do not form a connected ring",
                        vertex_id
                    ),
                ));
            }
        }
    }
}

/// Check that the edges incident to a vertex form a connected ring
/// (each edge shares a face with at least one other edge at the vertex).
fn check_vertex_ring_connectivity(
    edge_ids: &[TopoId],
    edge_coedge_map: &HashMap<TopoId, Vec<CoedgeInfo>>,
) -> bool {
    if edge_ids.len() <= 2 {
        return true;
    }

    // Build a graph: edges are nodes, edges are connected if they share a face
    let mut adjacency: HashMap<TopoId, HashSet<TopoId>> = HashMap::new();
    for &eid in edge_ids {
        adjacency.entry(eid).or_default();
    }

    for &eid_a in edge_ids {
        if let Some(coedges_a) = edge_coedge_map.get(&eid_a) {
            let faces_a: HashSet<TopoId> = coedges_a.iter().map(|c| c.face_id).collect();
            for &eid_b in edge_ids {
                if eid_a == eid_b {
                    continue;
                }
                if let Some(coedges_b) = edge_coedge_map.get(&eid_b) {
                    let faces_b: HashSet<TopoId> = coedges_b.iter().map(|c| c.face_id).collect();
                    if !faces_a.is_disjoint(&faces_b) {
                        adjacency.get_mut(&eid_a).unwrap().insert(eid_b);
                        adjacency.get_mut(&eid_b).unwrap().insert(eid_a);
                    }
                }
            }
        }
    }

    // BFS to check connectivity
    let start = edge_ids[0];
    let mut visited: HashSet<TopoId> = HashSet::new();
    let mut queue = vec![start];
    visited.insert(start);

    while let Some(current) = queue.pop() {
        if let Some(neighbors) = adjacency.get(&current) {
            for &next in neighbors {
                if !visited.contains(&next) {
                    visited.insert(next);
                    queue.push(next);
                }
            }
        }
    }

    visited.len() == edge_ids.len()
}

// ============================================================
// 3.4.5 Wire Closure
// ============================================================

/// Verify that the last coedge's end vertex connects to the first coedge's start vertex,
/// and consecutive coedges are connected.
fn check_wire_closure(
    shell: &Shell,
    edge_map: &HashMap<TopoId, Edge>,
    report: &mut TopologyValidationReport,
) {
    for face in &shell.faces {
        // Check outer wire
        if let Some(ref wire) = face.outer_wire {
            check_wire_closed(wire, face.id, true, shell.closed, edge_map, report);
        }
        // Check inner wires
        for wire in &face.inner_wires {
            check_wire_closed(wire, face.id, false, shell.closed, edge_map, report);
        }
    }
}

/// Check that a single wire is properly closed.
fn check_wire_closed(
    wire: &Wire,
    face_id: TopoId,
    is_outer: bool,
    shell_closed: bool,
    edge_map: &HashMap<TopoId, Edge>,
    report: &mut TopologyValidationReport,
) {
    if wire.coedges.is_empty() {
        return;
    }

    // Single-coedge wire: it's a closed loop (e.g., a circle) if the edge's
    // start and end are the same vertex, or if it has no explicit vertices.
    // We skip the closure check for single-coedge wires.
    if wire.coedges.len() == 1 {
        return;
    }

    // For multi-coedge wires, check connectivity:
    // The end vertex of coedge[i] should match the start vertex of coedge[i+1],
    // and the end vertex of the last should match the start vertex of the first.
    let n = wire.coedges.len();
    let mut has_gap = false;

    for i in 0..n {
        let j = (i + 1) % n;
        let ce_i = &wire.coedges[i];
        let ce_j = &wire.coedges[j];

        let end_vid_i = get_coedge_end_vertex(ce_i, edge_map);
        let start_vid_j = get_coedge_start_vertex(ce_j, edge_map);

        match (end_vid_i, start_vid_j) {
            (Some(vid_i), Some(vid_j)) => {
                if vid_i != vid_j {
                    // Also check geometric proximity as a fallback
                    let end_pt_i = get_coedge_end_point(ce_i, edge_map);
                    let start_pt_j = get_coedge_start_point(ce_j, edge_map);
                    if let (Some(pi), Some(pj)) = (end_pt_i, start_pt_j) {
                        if pi.distance_to(&pj) > TOLERANCE * 10.0 {
                            has_gap = true;
                            report.add(ValidationIssue::warning(
                                "WireClosure",
                                Some(wire.id),
                                &format!(
                                    "Gap in {} wire of face {} between coedge {} end and coedge {} start",
                                    if is_outer { "outer" } else { "inner" },
                                    face_id, ce_i.id, ce_j.id
                                ),
                            ));
                        }
                    } else {
                        has_gap = true;
                    }
                }
            }
            (None, _) | (_, None) => {
                // Can't determine vertex IDs — try geometric check
                let end_pt_i = get_coedge_end_point(ce_i, edge_map);
                let start_pt_j = get_coedge_start_point(ce_j, edge_map);
                if let (Some(pi), Some(pj)) = (end_pt_i, start_pt_j) {
                    if pi.distance_to(&pj) > TOLERANCE * 10.0 {
                        has_gap = true;
                        report.add(ValidationIssue::warning(
                            "WireClosure",
                            Some(wire.id),
                            &format!(
                                "Gap in {} wire of face {} between coedge {} end and coedge {} start (geometric check)",
                                if is_outer { "outer" } else { "inner" },
                                face_id, ce_i.id, ce_j.id
                            ),
                        ));
                    }
                }
                // If we can't check at all, skip silently
            }
        }
    }

    // Check that wire.closed flag is consistent
    if !has_gap && !wire.closed && n > 1 {
        report.add(ValidationIssue::info(
            "WireClosure",
            Some(wire.id),
            &format!(
                "Wire {} appears closed but is not marked as closed",
                wire.id
            ),
        ));
    }

    if has_gap && wire.closed {
        report.add(ValidationIssue::warning(
            "WireClosure",
            Some(wire.id),
            &format!(
                "Wire {} is marked closed but has gaps",
                wire.id
            ),
        ));
    }

    // In a closed shell, an unclosed outer wire is an error
    if shell_closed && is_outer && has_gap {
        // Upgrade to error: find existing warning and note it should be error
        // Actually, let's just add an additional error
        report.add(ValidationIssue::error(
            "WireClosure",
            Some(wire.id),
            &format!(
                "Outer wire of face {} is not closed in a closed shell",
                face_id
            ),
        ));
    }
}

/// Get the start vertex TopoId for a coedge.
fn get_coedge_start_vertex(coedge: &CoEdge, edge_map: &HashMap<TopoId, Edge>) -> Option<TopoId> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.vertex_start
    } else {
        edge.vertex_end
    }
}

/// Get the end vertex TopoId for a coedge.
fn get_coedge_end_vertex(coedge: &CoEdge, edge_map: &HashMap<TopoId, Edge>) -> Option<TopoId> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.vertex_end
    } else {
        edge.vertex_start
    }
}

/// Get the start 3D point for a coedge.
fn get_coedge_start_point(coedge: &CoEdge, edge_map: &HashMap<TopoId, Edge>) -> Option<Point3d> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.start_point()
    } else {
        edge.end_point()
    }
}

/// Get the end 3D point for a coedge.
fn get_coedge_end_point(coedge: &CoEdge, edge_map: &HashMap<TopoId, Edge>) -> Option<Point3d> {
    let edge = edge_map.get(&coedge.edge)?;
    if coedge.forward {
        edge.end_point()
    } else {
        edge.start_point()
    }
}

// ============================================================
// 3.4.6 Loop Orientation
// ============================================================

/// Check that the outer wire winds counter-clockwise and inner wires wind clockwise
/// (when viewed from outside the surface).
fn check_loop_orientation(shell: &Shell, report: &mut TopologyValidationReport) {
    for face in &shell.faces {
        let surface = match face.surface {
            Some(ref s) => s,
            None => continue,
        };

        // Check outer wire
        if let Some(ref wire) = face.outer_wire {
            if wire.coedges.len() >= 3 {
                let winding = compute_wire_winding_3d(wire, surface, face);
                match winding {
                    WindingDirection::Clockwise => {
                        report.add(ValidationIssue::warning(
                            "LoopOrientation",
                            Some(face.id),
                            &format!(
                                "Outer wire of face {} winds clockwise (expected CCW)",
                                face.id
                            ),
                        ));
                    }
                    WindingDirection::CounterClockwise => {
                        // Correct for outer wire
                    }
                    WindingDirection::Indeterminate => {
                        // Can't determine — skip
                    }
                }
            }
        }

        // Check inner wires
        for wire in &face.inner_wires {
            if wire.coedges.len() >= 3 {
                let winding = compute_wire_winding_3d(wire, surface, face);
                match winding {
                    WindingDirection::CounterClockwise => {
                        report.add(ValidationIssue::warning(
                            "LoopOrientation",
                            Some(face.id),
                            &format!(
                                "Inner wire of face {} winds counter-clockwise (expected CW)",
                                face.id
                            ),
                        ));
                    }
                    WindingDirection::Clockwise => {
                        // Correct for inner wire
                    }
                    WindingDirection::Indeterminate => {
                        // Can't determine — skip
                    }
                }
            }
        }
    }
}

/// Winding direction of a wire.
#[derive(Clone, Copy, Debug, PartialEq)]
enum WindingDirection {
    Clockwise,
    CounterClockwise,
    Indeterminate,
}

/// Compute the winding direction of a wire by projecting onto the face's surface
/// and computing the signed area in UV space.
fn compute_wire_winding_3d(wire: &Wire, surface: &Surface, face: &Face) -> WindingDirection {
    // Collect 3D points from the wire's coedges using the face's edge map
    let edge_map: HashMap<TopoId, Edge> = face.edges.iter().map(|e| (e.id, e.clone())).collect();

    let mut points_3d: Vec<Point3d> = Vec::new();
    for coedge in &wire.coedges {
        if let Some(pt) = get_coedge_start_point(coedge, &edge_map) {
            points_3d.push(pt);
        }
    }

    if points_3d.len() < 3 {
        return WindingDirection::Indeterminate;
    }

    // Project 3D points onto the surface's parametric space
    let points_uv: Vec<Point2d> = points_3d.iter().map(|p| {
        let (u, v) = surface.project_point(p);
        Point2d::new(u, v)
    }).collect();

    // Compute signed area using the shoelace formula
    let signed_area = compute_signed_area_2d(&points_uv);

    if signed_area.abs() < 1e-10 {
        return WindingDirection::Indeterminate;
    }

    // In standard UV space, positive signed area = CCW, negative = CW
    // But we need to account for the face's forward flag
    let effective_area = if face.forward { signed_area } else { -signed_area };

    if effective_area > 0.0 {
        WindingDirection::CounterClockwise
    } else {
        WindingDirection::Clockwise
    }
}

/// Compute the signed area of a 2D polygon using the shoelace formula.
fn compute_signed_area_2d(points: &[Point2d]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += points[i].u * points[j].v;
        area -= points[j].u * points[i].v;
    }
    area / 2.0
}

// ============================================================
// 3.4.7 Geometric Consistency
// ============================================================

/// For each edge in a face, verify that the edge's 3D points lie on the face's
/// surface within tolerance.
fn check_geometric_consistency(
    shell: &Shell,
    edge_map: &HashMap<TopoId, Edge>,
    report: &mut TopologyValidationReport,
) {
    let tol = TOLERANCE * 100.0; // Use a generous tolerance for geometric consistency

    for face in &shell.faces {
        let surface = match face.surface {
            Some(ref s) => s,
            None => continue,
        };

        // Get edge IDs from the face's coedges
        let coedge_edge_ids: Vec<TopoId> = {
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
        };

        for edge_id in &coedge_edge_ids {
            if let Some(edge) = edge_map.get(edge_id) {
                // Sample several points along the edge and check distance to surface
                let num_samples = 5;
                let mut max_deviation = 0.0f64;

                for k in 0..=num_samples {
                    let t = k as f64 / num_samples as f64;
                    if let Some(pt) = edge.point_at(t) {
                        let (u, v) = surface.project_point(&pt);
                        let surface_pt = surface.point_at(u, v);
                        let deviation = pt.distance_to(&surface_pt);
                        max_deviation = max_deviation.max(deviation);
                    }
                }

                if max_deviation > tol {
                    report.add(ValidationIssue::warning(
                        "GeometricConsistency",
                        Some(*edge_id),
                        &format!(
                            "Edge {} deviates up to {:.6} from face surface (tolerance: {:.6})",
                            edge_id, max_deviation, tol
                        ),
                    ));
                }
            }
        }
    }
}

// ============================================================
// 3.4.8 Euler Characteristic
// ============================================================

/// For a closed shell, compute and verify Euler characteristic: V − E + F = 2(1 − genus).
fn check_euler_characteristic(
    shell: &Shell,
    edge_coedge_map: &HashMap<TopoId, Vec<CoedgeInfo>>,
    edge_map: &HashMap<TopoId, Edge>,
    report: &mut TopologyValidationReport,
) {
    if !shell.closed {
        return;
    }

    let f = shell.faces.len();
    if f == 0 {
        return;
    }

    // Count unique edges referenced by coedges
    let e = edge_coedge_map.len();

    // Count unique vertices from edges that are referenced by coedges
    let mut vertex_set: HashSet<TopoId> = HashSet::new();
    for edge_id in edge_coedge_map.keys() {
        if let Some(edge) = edge_map.get(edge_id) {
            if let Some(vid) = edge.vertex_start {
                vertex_set.insert(vid);
            }
            if let Some(vid) = edge.vertex_end {
                vertex_set.insert(vid);
            }
        }
    }
    let v = vertex_set.len();

    // For edges without explicit vertices, try geometric vertex counting
    if v == 0 && e > 0 {
        // No vertex IDs available — try geometric approach
        let geom_result = count_geometric_vertices(shell, edge_map);
        let (gv, ge) = geom_result;
        if gv > 0 && ge > 0 {
            let chi = gv as i64 - ge as i64 + f as i64;
            let expected = 2; // genus 0
            if chi != expected {
                report.add(ValidationIssue::warning(
                    "EulerCharacteristic",
                    Some(shell.id),
                    &format!(
                        "Euler characteristic V-E+F = {}-{}+{} = {} (expected {} for genus 0)",
                        gv, ge, f, chi, expected
                    ),
                ));
            }
            return;
        }
        // Can't compute — skip
        return;
    }

    let chi = v as i64 - e as i64 + f as i64;
    let expected = 2; // For genus 0 (sphere topology)

    if chi != expected {
        // Check for higher genus
        let genus = 1 - chi / 2;
        if chi % 2 == 0 && genus >= 0 {
            report.add(ValidationIssue::info(
                "EulerCharacteristic",
                Some(shell.id),
                &format!(
                    "Euler characteristic V-E+F = {}-{}+{} = {} (consistent with genus {})",
                    v, e, f, chi, genus
                ),
            ));
        } else {
            report.add(ValidationIssue::warning(
                "EulerCharacteristic",
                Some(shell.id),
                &format!(
                    "Euler characteristic V-E+F = {}-{}+{} = {} (expected {} for genus 0)",
                    v, e, f, chi, expected
                ),
            ));
        }
    }
}

/// Count geometric vertices and edges for shells where TopoId-based vertices
/// are not available. Uses point coincidence to identify unique vertices.
fn count_geometric_vertices(
    shell: &Shell,
    edge_map: &HashMap<TopoId, Edge>,
) -> (usize, usize) {
    let coedge_edge_ids: HashSet<TopoId> = {
        let mut ids = HashSet::new();
        for face in &shell.faces {
            if let Some(ref wire) = face.outer_wire {
                for coedge in &wire.coedges {
                    ids.insert(coedge.edge);
                }
            }
            for wire in &face.inner_wires {
                for coedge in &wire.coedges {
                    ids.insert(coedge.edge);
                }
            }
        }
        ids
    };

    let e = coedge_edge_ids.len();
    let tol = TOLERANCE * 10.0;

    // Collect all edge endpoints
    let mut points: Vec<Point3d> = Vec::new();
    for eid in &coedge_edge_ids {
        if let Some(edge) = edge_map.get(eid) {
            if let Some(sp) = edge.start_point() {
                points.push(sp);
            }
            if let Some(ep) = edge.end_point() {
                points.push(ep);
            }
        }
    }

    // Count unique points via coincidence
    let v = count_unique_points(&points, tol);

    (v, e)
}

/// Count unique points by merging coincident points.
fn count_unique_points(points: &[Point3d], tolerance: f64) -> usize {
    if points.is_empty() {
        return 0;
    }
    let tol_sq = tolerance * tolerance;
    let mut unique: Vec<Point3d> = Vec::new();

    for p in points {
        let is_dup = unique.iter().any(|u| p.distance_sq_to(u) < tol_sq);
        if !is_dup {
            unique.push(*p);
        }
    }

    unique.len()
}

// ============================================================
// Unit tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::ShapeBuilder;
    use draper_geometry::{Direction3d, Plane, Point3d, Surface};

    /// Build a proper box with shared edges for validation testing.
    /// Unlike ShapeBuilder::make_box, this creates 12 shared edges and 8 shared vertices,
    /// so that each edge is referenced by exactly 2 faces.
    fn make_proper_box() -> Solid {
        let hx = 1.0;
        let hy = 1.0;
        let hz = 1.0;

        // 8 vertices of the box
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

        // Create shared vertex IDs
        let vids: Vec<TopoId> = (0..8).map(|_| TopoId::new()).collect();

        // Create 12 shared edges with explicit vertex IDs
        macro_rules! make_edge {
            ($from:expr, $to:expr, $vfrom:expr, $vto:expr) => {{
                let mut e = Edge::new_line(v[$from], v[$to]);
                e.vertex_start = Some(vids[$vfrom]);
                e.vertex_end = Some(vids[$vto]);
                e
            }};
        }

        // Bottom ring
        let e01 = make_edge!(0, 1, 0, 1);
        let e12 = make_edge!(1, 2, 1, 2);
        let e23 = make_edge!(2, 3, 2, 3);
        let e30 = make_edge!(3, 0, 3, 0);
        // Top ring
        let e45 = make_edge!(4, 5, 4, 5);
        let e56 = make_edge!(5, 6, 5, 6);
        let e67 = make_edge!(6, 7, 6, 7);
        let e74 = make_edge!(7, 4, 7, 4);
        // Verticals
        let e04 = make_edge!(0, 4, 0, 4);
        let e15 = make_edge!(1, 5, 1, 5);
        let e26 = make_edge!(2, 6, 2, 6);
        let e37 = make_edge!(3, 7, 3, 7);

        // Save edge IDs before moving
        let id01 = e01.id; let id12 = e12.id; let id23 = e23.id; let id30 = e30.id;
        let id45 = e45.id; let id56 = e56.id; let id67 = e67.id; let id74 = e74.id;
        let id04 = e04.id; let id15 = e15.id; let id26 = e26.id; let id37 = e37.id;

        // Build faces with properly oriented coedges
        // Convention: right-hand rule with outward normal

        // Bottom face (-Z normal): traversal v0→v3→v2→v1→v0
        //   e30(rev: v0→v3), e23(rev: v3→v2), e12(rev: v2→v1), e01(rev: v1→v0)
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

        // Top face (+Z normal): traversal v4→v5→v6→v7→v4
        //   e45(fwd: v4→v5), e56(fwd: v5→v6), e67(fwd: v6→v7), e74(fwd: v7→v4)
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

        // Front face (-Y normal): traversal v0→v1→v5→v4→v0
        //   e01(fwd: v0→v1), e15(fwd: v1→v5), e45(rev: v5→v4), e04(rev: v4→v0)
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

        // Back face (+Y normal): traversal v2→v3→v7→v6→v2
        //   e23(fwd: v2→v3), e37(fwd: v3→v7), e67(rev: v7→v6), e26(rev: v6→v2)
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
            Direction3d::Y,
        );
        let mut back_face = Face::new(Surface::Plane(plane_back), back_wire);
        back_face.edges = vec![e23.clone(), e37.clone(), e67.clone(), e26.clone()];

        // Left face (-X normal): traversal v0→v4→v7→v3→v0
        //   e04(fwd: v0→v4), e74(rev: v4→v7... wait, e74 forward is v7→v4, reversed is v4→v7)
        //   Hmm, e74 = Edge::new_line(v[7], v[4]), forward direction is v7→v4
        //   So for traversal v4→v7, we need e74 reversed? No, reversed of v7→v4 is v4→v7. ✓
        //   Wait: CoEdge forward=true means traverse edge in its natural direction.
        //   e74 natural: v7→v4. forward=false: v4→v7. ✓
        //   e37 natural: v3→v7. forward=false: v7→v3. ✓
        //   e30 natural: v3→v0. forward=true: v3→v0. ✓
        let left_coedges = vec![
            CoEdge::new(id04, true),   // v0→v4
            CoEdge::new(id74, false),  // v4→v7
            CoEdge::new(id37, false),  // v7→v3
            CoEdge::new(id30, true),   // v3→v0
        ];
        let mut left_wire = Wire::new(left_coedges);
        left_wire.closed = true;
        let plane_left = Plane::from_origin_and_normal(
            Point3d::new(-hx, 0.0, 0.0),
            Direction3d::new(-1.0, 0.0, 0.0).unwrap(),
        );
        let mut left_face = Face::new(Surface::Plane(plane_left), left_wire);
        left_face.edges = vec![e04.clone(), e74.clone(), e37.clone(), e30.clone()];

        // Right face (+X normal): traversal v1→v2→v6→v5→v1
        //   e12(fwd: v1→v2), e26(fwd: v2→v6), e56(rev: v6→v5), e15(rev: v5→v1)
        let right_coedges = vec![
            CoEdge::new(id12, true),   // v1→v2
            CoEdge::new(id26, true),   // v2→v6
            CoEdge::new(id56, false),  // v6→v5
            CoEdge::new(id15, false),  // v5→v1
        ];
        let mut right_wire = Wire::new(right_coedges);
        right_wire.closed = true;
        let plane_right = Plane::from_origin_and_normal(
            Point3d::new(hx, 0.0, 0.0),
            Direction3d::X,
        );
        let mut right_face = Face::new(Surface::Plane(plane_right), right_wire);
        right_face.edges = vec![e12.clone(), e26.clone(), e56.clone(), e15.clone()];

        let shell = Shell::new_closed(vec![bottom_face, top_face, front_face, back_face, left_face, right_face]);
        Solid::new(shell)
    }

    // ---- Test 1: Valid box should have no errors ----
    #[test]
    fn test_valid_box_no_errors() {
        let box_solid = make_proper_box();
        let config = TopologyValidationConfig::all();
        let report = validate_topology(&box_solid, &config);

        // Print report for debugging
        if report.has_errors() {
            for issue in &report.issues {
                eprintln!("  [{}] {} : {}", issue.severity, issue.check, issue.message);
            }
        }

        assert!(!report.has_errors(), "Valid box should have no errors, but found {}", report.error_count);

        // Also check specific important validations
        let shell = box_solid.outer_shell.as_ref().unwrap();
        let edge_coedge_map = build_edge_coedge_map(shell);

        // All edges should be shared by exactly 2 faces
        for (edge_id, coedges) in &edge_coedge_map {
            let face_count = coedges.iter().map(|c| c.face_id).collect::<HashSet<_>>().len();
            assert_eq!(
                face_count, 2,
                "Edge {} should be shared by 2 faces, found {}",
                edge_id, face_count
            );
        }
    }

    // ---- Test 2: Shell with missing face should report shell closure error ----
    #[test]
    fn test_missing_face_shell_closure_error() {
        let box_solid = make_proper_box();
        let shell = box_solid.outer_shell.as_ref().unwrap();

        // Create a shell with one face removed
        let mut faces = shell.faces.clone();
        faces.pop(); // Remove one face
        let open_shell = Shell::new_closed(faces); // Still marked closed

        let solid = Solid::new(open_shell);
        let config = TopologyValidationConfig::all();
        let report = validate_topology(&solid, &config);

        // Should have shell closure errors (edges that were shared with the removed face
        // are now referenced by only 1 face)
        let closure_errors = report.issues_for_check("ShellClosure");
        assert!(
            !closure_errors.is_empty(),
            "Shell with missing face should have ShellClosure issues, but found none"
        );

        // Should be at least one error-level issue
        let has_error = closure_errors.iter().any(|i| i.severity == Severity::Error);
        assert!(
            has_error,
            "ShellClosure should report at least one Error for missing face"
        );
    }

    // ---- Test 3: Unclosed wire should report wire closure warning ----
    #[test]
    fn test_unclosed_wire_warning() {
        // Create a face with an intentionally unclosed wire.
        // The wire has 3 edges: p0→p1, p1→p2, then p2 does NOT connect back to p0.
        // There is a large gap between p2 and p0.
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(1.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 1.0, 0.0);

        let e0 = Edge::new_line(p0, p1);
        let e1 = Edge::new_line(p1, p2);

        // To create a truly unclosed wire, we need a gap in geometry.
        // The third edge goes far away instead of back to p0.
        let e_gap = Edge::new_line(p2, Point3d::new(10.0, 10.0, 10.0));

        let coedges = vec![
            CoEdge::new(e0.id, true),
            CoEdge::new(e1.id, true),
            CoEdge::new(e_gap.id, true),
        ];
        let mut wire = Wire::new(coedges);
        wire.closed = true;

        let plane = Plane::from_three_points(&p0, &p1, &p2)
            .unwrap_or_else(|| Plane::from_origin_and_normal(p0, Direction3d::Z));
        let mut face = Face::new(Surface::Plane(plane), wire);
        face.edges = vec![e0, e1, e_gap];

        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);

        let config = TopologyValidationConfig {
            check_wire_closure: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&solid, &config);

        let wire_issues = report.issues_for_check("WireClosure");
        assert!(
            !wire_issues.is_empty(),
            "Unclosed wire should produce WireClosure issues, found none"
        );
    }

    // ---- Test 4: Euler characteristic check for a valid closed shell ----
    #[test]
    fn test_euler_characteristic_valid_shell() {
        let box_solid = make_proper_box();
        let config = TopologyValidationConfig {
            check_euler_characteristic: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&box_solid, &config);

        // For a proper box: V=8, E=12, F=6 → V-E+F = 2 ✓
        let euler_issues = report.issues_for_check("EulerCharacteristic");
        let euler_warnings: Vec<_> = euler_issues
            .iter()
            .filter(|i| i.severity == Severity::Warning)
            .collect();

        assert!(
            euler_warnings.is_empty(),
            "Valid box should have no Euler characteristic warnings, but found {}",
            euler_warnings.len()
        );
    }

    // ---- Test 5: Edge manifoldness — edge shared by 3+ faces ----
    #[test]
    fn test_non_manifold_edge() {
        let box_solid = make_proper_box();
        let shell = box_solid.outer_shell.as_ref().unwrap();

        // Duplicate one face to create a non-manifold condition.
        // We need to give the cloned face a new ID so it's counted as a separate face.
        let mut faces = shell.faces.clone();
        if !faces.is_empty() {
            let mut cloned = faces[0].clone();
            cloned.id = TopoId::new(); // Give it a unique face ID
            faces.push(cloned);
        }

        let open_shell = Shell {
            id: TopoId::new(),
            faces,
            closed: false, // Not closed since we have extra face
        };
        let solid = Solid::new(open_shell);

        let config = TopologyValidationConfig {
            check_edge_manifoldness: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&solid, &config);

        let manifold_errors = report.issues_for_check("EdgeManifoldness");
        assert!(
            !manifold_errors.is_empty(),
            "Non-manifold edge (3+ faces) should produce EdgeManifoldness issues"
        );

        let has_error = manifold_errors.iter().any(|i| i.severity == Severity::Error);
        assert!(
            has_error,
            "Non-manifold edge should be reported as Error severity"
        );
    }

    // ---- Test 6: Config can disable checks ----
    #[test]
    fn test_config_disables_checks() {
        let box_solid = make_proper_box();

        // Create a shell with a missing face
        let shell = box_solid.outer_shell.as_ref().unwrap();
        let mut faces = shell.faces.clone();
        faces.pop();
        let broken_shell = Shell::new_closed(faces);
        let broken_solid = Solid::new(broken_shell);

        // With all checks disabled, should have no issues
        let config = TopologyValidationConfig::none();
        let report = validate_topology(&broken_solid, &config);

        // Only the basic structure check for no outer shell would fire
        // But we have an outer shell, so no issues
        assert!(
            report.issues.is_empty() || report.issues.iter().all(|i| i.check != "ShellClosure"),
            "With all checks disabled, ShellClosure should not run"
        );

        // With shell closure enabled, should find issues
        let config = TopologyValidationConfig {
            check_shell_closure: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&broken_solid, &config);
        assert!(
            !report.issues_for_check("ShellClosure").is_empty(),
            "With ShellClosure enabled, should find issues"
        );
    }

    // ---- Test 7: Validation report formatting ----
    #[test]
    fn test_validation_report_display() {
        let mut report = TopologyValidationReport::new();
        report.add(ValidationIssue::error("TestCheck", None, "test error"));
        report.add(ValidationIssue::warning("TestCheck", None, "test warning"));
        report.add(ValidationIssue::info("TestCheck", None, "test info"));

        assert_eq!(report.error_count, 1);
        assert_eq!(report.warning_count, 1);
        assert_eq!(report.info_count, 1);
        assert!(report.has_errors());
        assert!(!report.is_clean());

        let display = format!("{}", report);
        assert!(display.contains("Errors: 1"));
        assert!(display.contains("Warnings: 1"));
        assert!(display.contains("Info: 1"));
    }

    // ---- Test 8: Severity display ----
    #[test]
    fn test_severity_display() {
        assert_eq!(format!("{}", Severity::Error), "Error");
        assert_eq!(format!("{}", Severity::Warning), "Warning");
        assert_eq!(format!("{}", Severity::Info), "Info");
    }

    // ---- Test 9: Geometric consistency for a valid box ----
    #[test]
    fn test_geometric_consistency_valid_box() {
        let box_solid = make_proper_box();
        let config = TopologyValidationConfig {
            check_geometric_consistency: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&box_solid, &config);

        let geo_issues = report.issues_for_check("GeometricConsistency");
        let geo_warnings: Vec<_> = geo_issues
            .iter()
            .filter(|i| i.severity == Severity::Warning)
            .collect();

        assert!(
            geo_warnings.is_empty(),
            "Valid box edges should lie on their face surfaces, found {} geometric consistency warnings",
            geo_warnings.len()
        );
    }

    // ---- Test 10: Face orientation consistency for proper box ----
    #[test]
    fn test_face_orientation_proper_box() {
        let box_solid = make_proper_box();
        let config = TopologyValidationConfig {
            check_face_orientation: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&box_solid, &config);

        let orientation_issues = report.issues_for_check("FaceOrientation");
        let orientation_warnings: Vec<_> = orientation_issues
            .iter()
            .filter(|i| i.severity == Severity::Warning)
            .collect();

        assert!(
            orientation_warnings.is_empty(),
            "Proper box should have consistent face orientations, found {} warnings",
            orientation_warnings.len()
        );
    }

    // ---- Test 11: Backward compatibility — legacy validate_solid ----
    #[test]
    fn test_legacy_validate_solid() {
        let box_solid = ShapeBuilder::make_box(2.0, 2.0, 2.0);
        let mut mutable_solid = box_solid;
        let errors = validate_solid(&mut mutable_solid);
        // Legacy function should still work
        // ShapeBuilder's box has no degenerate edges or missing geometry
        assert!(
            errors.iter().all(|e| !matches!(e, ValidationError::EmptyShell)),
            "Box should not have EmptyShell error"
        );
    }

    // ---- Test 12: Wire closure detects unclosed wire in closed shell ----
    #[test]
    fn test_unclosed_wire_in_closed_shell_is_error() {
        // Create a face with an unclosed wire (geometric gap)
        let p0 = Point3d::new(0.0, 0.0, 0.0);
        let p1 = Point3d::new(1.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 1.0, 0.0);

        let e0 = Edge::new_line(p0, p1);
        let e1 = Edge::new_line(p1, p2);
        let e2 = Edge::new_line(p2, Point3d::new(0.0, 1.0, 1.0)); // Gap: p3 is not at p0

        // Edges don't share vertex IDs, so the wire will appear unclosed
        // The geometric gap is detected by the validation

        let coedges = vec![
            CoEdge::new(e0.id, true),
            CoEdge::new(e1.id, true),
            CoEdge::new(e2.id, true),
        ];
        let wire = Wire::new(coedges);

        let plane = Plane::from_three_points(&p0, &p1, &p2)
            .unwrap_or_else(|| Plane::from_origin_and_normal(p0, Direction3d::Z));
        let mut face = Face::new(Surface::Plane(plane), wire);
        face.edges = vec![e0, e1, e2];

        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);

        let config = TopologyValidationConfig {
            check_wire_closure: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&solid, &config);

        // Should detect wire issues in closed shell
        let wire_issues: Vec<_> = report
            .issues
            .iter()
            .filter(|i| i.check == "WireClosure")
            .collect();
        assert!(
            !wire_issues.is_empty(),
            "Unclosed wire in closed shell should be detected"
        );
    }

    // ---- Test 13: TopologyValidationConfig defaults ----
    #[test]
    fn test_config_defaults() {
        let config = TopologyValidationConfig::default();
        assert!(config.check_shell_closure);
        assert!(config.check_face_orientation);
        assert!(config.check_edge_manifoldness);
        assert!(config.check_vertex_connectivity);
        assert!(config.check_wire_closure);
        assert!(config.check_loop_orientation);
        assert!(config.check_geometric_consistency);
        assert!(config.check_euler_characteristic);
    }

    // ---- Test 14: Critical-only config ----
    #[test]
    fn test_critical_only_config() {
        let config = TopologyValidationConfig::critical_only();
        assert!(config.check_shell_closure);
        assert!(!config.check_face_orientation);
        assert!(config.check_edge_manifoldness);
        assert!(!config.check_vertex_connectivity);
        assert!(config.check_wire_closure);
        assert!(!config.check_loop_orientation);
        assert!(!config.check_geometric_consistency);
        assert!(!config.check_euler_characteristic);
    }

    // ---- Test 15: Shell with 5 faces (missing one) has wrong Euler characteristic ----
    #[test]
    fn test_euler_characteristic_broken_shell() {
        let box_solid = make_proper_box();
        let shell = box_solid.outer_shell.as_ref().unwrap();

        // Remove one face
        let mut faces = shell.faces.clone();
        faces.pop();
        let broken_shell = Shell::new_closed(faces);
        let solid = Solid::new(broken_shell);

        let config = TopologyValidationConfig {
            check_euler_characteristic: true,
            ..TopologyValidationConfig::none()
        };
        let report = validate_topology(&solid, &config);

        // V=8, E=12, F=5 → V-E+F = 1, expected 2 → warning
        let euler_issues = report.issues_for_check("EulerCharacteristic");
        assert!(
            !euler_issues.is_empty(),
            "Shell with missing face should have Euler characteristic issue"
        );
    }
}
