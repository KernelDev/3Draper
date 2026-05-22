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
}

/// Validate a solid's topology.
pub fn validate_solid(solid: &Solid) -> Vec<ValidationError> {
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
pub fn heal_solid(solid: &mut Solid) -> Vec<String> {
    let mut fixes = Vec::new();

    if let Some(ref mut shell) = solid.outer_shell {
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
