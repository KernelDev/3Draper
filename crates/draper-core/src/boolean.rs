//! Boolean operations — union, subtract, intersection.
//!
//! These are simplified implementations. Full boolean operations require:
//! 1. Surface-surface intersection
//! 2. Edge splitting and face trimming
//! 3. Topology reconstruction
//!
//! Current implementation uses approximation for simple cases.

use draper_geometry::{
    Point3d, Direction3d, Vec3d,
    Surface, Plane, CylinderSurface,
    Transform,
};
use draper_topology::{
    Solid, Shell, Face, Wire, CoEdge, Edge, Vertex,
    ShapeBuilder,
};
use crate::operations;

/// Result of a boolean operation.
pub type BooleanResult = Result<Solid, String>;

/// Boolean union: combine two solids into one.
pub fn boolean_union(a: &Solid, b: &Solid) -> BooleanResult {
    // Simplified: just merge faces from both shells
    // A real implementation would need to:
    // 1. Find intersection curves between all face pairs
    // 2. Split faces along intersection curves
    // 3. Classify which parts to keep
    // 4. Rebuild topology

    let mut faces = Vec::new();

    if let Some(ref shell_a) = a.outer_shell {
        faces.extend(shell_a.faces.iter().cloned());
    }
    if let Some(ref shell_b) = b.outer_shell {
        faces.extend(shell_b.faces.iter().cloned());
    }

    // TODO: Remove internal faces (faces that are inside the other solid)

    let shell = Shell::new_closed(faces);
    Ok(Solid::new(shell))
}

/// Boolean subtraction: subtract solid B from solid A.
pub fn boolean_subtract(a: &Solid, b: &Solid) -> BooleanResult {
    // Simplified: add reversed faces of B to A
    // A real implementation would need intersection and classification

    let mut faces = Vec::new();

    if let Some(ref shell_a) = a.outer_shell {
        faces.extend(shell_a.faces.iter().cloned());
    }
    if let Some(ref shell_b) = b.outer_shell {
        for face in &shell_b.faces {
            faces.push(face.reversed());
        }
    }

    // TODO: Remove faces of A that are inside B
    // TODO: Remove faces of B that are outside A
    // TODO: Find intersection edges and split faces

    let shell = Shell::new_closed(faces);
    Ok(Solid::new(shell))
}

/// Boolean intersection: keep only the overlap of A and B.
pub fn boolean_intersect(a: &Solid, b: &Solid) -> BooleanResult {
    // Simplified: intersection is even harder than union/subtract
    // Would need full classification

    log::warn!("Boolean intersection is not fully implemented");

    let mut faces = Vec::new();

    if let Some(ref shell_a) = a.outer_shell {
        faces.extend(shell_a.faces.iter().cloned());
    }

    let shell = Shell::new_closed(faces);
    Ok(Solid::new(shell))
}

/// Check if a point is inside a solid (ray casting).
pub fn point_in_solid(point: &Point3d, solid: &Solid) -> bool {
    // Ray casting: shoot a ray and count intersections
    // Odd number = inside, even = outside

    if solid.outer_shell.is_none() {
        return false;
    }

    let ray_dir = Direction3d::X; // Shoot along +X
    let ray = draper_geometry::Line::new(*point, ray_dir);

    let mut intersections = 0;

    if let Some(ref shell) = solid.outer_shell {
        for face in &shell.faces {
            if let Some(ref surface) = face.surface {
                match surface {
                    Surface::Plane(plane) => {
                        if let Some(hit) = draper_geometry::intersection::intersect_line_plane(&ray, plane) {
                            // Check if hit point is within the face boundary
                            // Simplified: just count intersections
                            intersections += 1;
                        }
                    }
                    _ => {
                        // For curved surfaces, use numerical intersection
                        // Simplified: skip
                    }
                }
            }
        }
    }

    intersections % 2 == 1
}
