//! Topology validation and healing.

use crate::entity::*;
use draper_geometry::tolerance::TOLERANCE;

/// Validation error.
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

/// Validate a solid's topology.
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

/// Validate a shell.
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

/// Heal a solid: fix common topological issues.
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
