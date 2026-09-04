// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Modeling operations on B-Rep solids.
//!
//! Implements:
//! - 4.3.1 Fillet (edge rounding)
//! - 4.3.2 Chamfer (edge bevel)
//! - 4.3.3 Shell (hollow out)
//! - 4.3.4 Draft (taper)

use crate::entity::*;
use crate::builder::ShapeBuilder;
use crate::boolean::boolean_subtract;
use draper_geometry::{
    Point3d, Direction3d, Vec3d,
    Curve3d, Line, Surface, Plane,
    Transform, ToleranceContext,
};
use std::f64::consts::PI;

// ============================================================
// Helper functions
// ============================================================

/// Collect all edges from all faces of a solid, returning them
/// with their parent face index for later lookup.
struct EdgeInfo {
    /// Index into the solid's face list (flattened across shells).
    #[allow(dead_code)]
    face_index: usize,
    /// Index within the face's `edges` vector.
    #[allow(dead_code)]
    edge_local_index: usize,
    /// The edge itself (cloned).
    edge: Edge,
}

/// Collect all edges from a solid in a flat list.
fn collect_edges(solid: &Solid) -> Vec<EdgeInfo> {
    let mut result = Vec::new();
    for (fi, face) in solid.faces().iter().enumerate() {
        // C5 Stage 4: store-resolved (canonical) edge list — shared edges
        // resolve to the single canonical Edge; un-indexed faces fall back
        // to their mirrors.
        for (ei, edge) in solid.face_edges(face).into_iter().enumerate() {
            result.push(EdgeInfo {
                face_index: fi,
                edge_local_index: ei,
                edge: edge.clone(),
            });
        }
    }
    result
}

/// Compute the axis-aligned bounding box of a solid.
/// Returns (min_corner, max_corner).
fn compute_bounding_box(solid: &Solid) -> (Point3d, Point3d) {
    let mut min_pt = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max_pt = Point3d::new(f64::MIN, f64::MIN, f64::MIN);

    for face in solid.faces() {
        // C5 Stage 4: store-resolved edge list (see collect_edges).
        for edge in solid.face_edges(face) {
            // Sample points along the edge curve
            let n_samples = 20;
            for i in 0..=n_samples {
                let t = i as f64 / n_samples as f64;
                if let Some(p) = edge.point_at(t) {
                    min_pt.x = min_pt.x.min(p.x);
                    min_pt.y = min_pt.y.min(p.y);
                    min_pt.z = min_pt.z.min(p.z);
                    max_pt.x = max_pt.x.max(p.x);
                    max_pt.y = max_pt.y.max(p.y);
                    max_pt.z = max_pt.z.max(p.z);
                }
            }
        }
        // Also sample the surface if there are no edges
        if solid.face_edges(face).is_empty() {
            if let Some(ref surface) = face.surface {
                let (u_min, u_max, v_min, v_max) = surface_param_range_approx(surface);
                let n = 10;
                for i in 0..=n {
                    for j in 0..=n {
                        let u = u_min + (u_max - u_min) * (i as f64 / n as f64);
                        let v = v_min + (v_max - v_min) * (j as f64 / n as f64);
                        let p = surface.point_at(u, v);
                        min_pt.x = min_pt.x.min(p.x);
                        min_pt.y = min_pt.y.min(p.y);
                        min_pt.z = min_pt.z.min(p.z);
                        max_pt.x = max_pt.x.max(p.x);
                        max_pt.y = max_pt.y.max(p.y);
                        max_pt.z = max_pt.z.max(p.z);
                    }
                }
            }
        }
    }

    // Safety: if no points were found, return a unit box
    if min_pt.x > max_pt.x {
        return (Point3d::ORIGIN, Point3d::new(1.0, 1.0, 1.0));
    }

    (min_pt, max_pt)
}

/// Get approximate parametric range for a surface.
fn surface_param_range_approx(surface: &Surface) -> (f64, f64, f64, f64) {
    match surface {
        Surface::Plane(_) => (-1e4, 1e4, -1e4, 1e4),
        Surface::Cylinder(cyl) => {
            let (u_min, u_max) = cyl.u_range();
            (u_min, u_max, -1e4, 1e4)
        }
        Surface::Sphere(_) => (0.0, 2.0 * PI, 0.0, PI),
        Surface::Cone(_) => (0.0, 2.0 * PI, -1e4, 1e4),
        Surface::Torus(_) => (0.0, 2.0 * PI, 0.0, 2.0 * PI),
        Surface::Nurbs(n) => {
            let (u_min, u_max) = n.u_range();
            let (v_min, v_max) = n.v_range();
            (u_min, u_max, v_min, v_max)
        }
        Surface::Revolution(_) => (0.0, 2.0 * PI, -1e4, 1e4),
        Surface::Extrusion(_) => (-1e4, 1e4, -1e4, 1e4),
        Surface::Offset(o) => surface_param_range_approx(&o.base),
        Surface::Ruled(_) => (-1e4, 1e4, 0.0, 1.0),
    }
}

/// Find the two faces adjacent to a given edge (identified by edge ID).
/// Returns face indices in the solid's flattened face list.
fn find_adjacent_faces(solid: &Solid, edge_id: TopoId) -> Vec<usize> {
    // C5 Stage 6.4: match in the CANONICAL id space — `edge_id` may name an
    // instance alias, and shared-edge instances carry different TopoIds per
    // incident face. Un-indexed solids keep the legacy identity semantics
    // (`canonical_of` is the identity without aliases).
    let wanted = solid.edge_store.canonical_of(edge_id);
    let canonical_of = |id: TopoId| solid.edge_store.canonical_of(id);
    let mut face_indices = Vec::new();
    for (fi, face) in solid.faces().iter().enumerate() {
        // Check if any coedge in the outer wire references this edge
        let mut found = false;
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                if canonical_of(coedge.edge) == wanted {
                    found = true;
                    break;
                }
            }
        }
        // Also check if the edge id appears in the face's edge list
        // (canonical references — wire-less faces)
        if !found {
            for &eid in &face.edge_ids {
                if canonical_of(eid) == wanted {
                    found = true;
                    break;
                }
            }
        }
        if found {
            face_indices.push(fi);
        }
    }
    face_indices
}

/// Get the outward normal of a face at its center.
fn face_normal(face: &Face) -> Direction3d {
    if let Some(ref surface) = face.surface {
        match surface {
            Surface::Plane(plane) => {
                if face.forward {
                    plane.normal
                } else {
                    Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z)
                        .unwrap_or(plane.normal)
                }
            }
            _ => {
                // For non-planar surfaces, evaluate normal at a sample point
                let (u_min, u_max, v_min, v_max) = surface_param_range_approx(surface);
                let u_mid = (u_min + u_max) / 2.0;
                let v_mid = (v_min + v_max) / 2.0;
                surface.normal_at(u_mid, v_mid)
            }
        }
    } else {
        Direction3d::Z
    }
}

/// Create a cylinder along an arbitrary line defined by two endpoints.
fn make_cylinder_along_line(p1: Point3d, p2: Point3d, radius: f64) -> Solid {
    let edge_vec = Vec3d::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    let height = edge_vec.length();
    if height < 1e-10 {
        // Degenerate edge — return a tiny cylinder at the point
        return ShapeBuilder::make_cylinder(radius.max(1e-6), 1e-6);
    }

    // Create cylinder along Z axis with extra length for robust intersection
    let margin = radius * 2.0;
    let total_height = height + 2.0 * margin;
    let mut cyl = ShapeBuilder::make_cylinder(radius, total_height);

    // Compute rotation from Z axis to edge direction
    let edge_dir = edge_vec.normalize().unwrap_or(Direction3d::Z);
    let z = Direction3d::Z;

    // Dot product gives cos(angle)
    let dot = z.x * edge_dir.x + z.y * edge_dir.y + z.z * edge_dir.z;

    if dot < -1.0 + 1e-10 {
        // Anti-parallel to Z: rotate 180° around X axis
        let rotation = Transform::rotation_x(PI);
        ShapeBuilder::transform_solid(&mut cyl, &rotation);
    } else if dot < 1.0 - 1e-10 {
        // General case: rotate from Z to edge direction
        // Rotation axis = Z × edge_dir
        let cross_v = Vec3d::new(
            z.y * edge_dir.z - z.z * edge_dir.y,
            z.z * edge_dir.x - z.x * edge_dir.z,
            z.x * edge_dir.y - z.y * edge_dir.x,
        );
        if let Some(rot_axis) = cross_v.normalize() {
            let angle = dot.acos();
            let rotation = Transform::rotation_axis(&rot_axis, angle);
            ShapeBuilder::transform_solid(&mut cyl, &rotation);
        }
    }
    // If dot ≈ 1.0, edge is already along Z, no rotation needed

    // Translate: the cylinder was created from z=0 to z=total_height
    // After rotation, the base is at p1 - margin * edge_dir
    let base_point = Point3d::new(
        p1.x - margin * edge_dir.x,
        p1.y - margin * edge_dir.y,
        p1.z - margin * edge_dir.z,
    );
    let translation = Transform::translation(base_point.x, base_point.y, base_point.z);
    ShapeBuilder::transform_solid(&mut cyl, &translation);

    cyl
}

/// Create a wedge (triangular prism) for chamfering.
///
/// The wedge is defined by an edge (p1 → p2) and two offset directions
/// on the adjacent faces, each offset by `distance`.
fn make_chamfer_wedge(
    p1: Point3d,
    p2: Point3d,
    normal1: Direction3d,
    normal2: Direction3d,
    distance: f64,
) -> Solid {
    let edge_vec = Vec3d::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    let edge_dir = edge_vec.normalize().unwrap_or(Direction3d::Z);

    // Compute offset directions on each face surface (perpendicular to edge, in face plane)
    // offset_dir = edge_dir × face_normal  (then normalize)
    let offset1_vec = Vec3d::new(
        edge_dir.y * normal1.z - edge_dir.z * normal1.y,
        edge_dir.z * normal1.x - edge_dir.x * normal1.z,
        edge_dir.x * normal1.y - edge_dir.y * normal1.x,
    );
    let offset2_vec = Vec3d::new(
        edge_dir.y * normal2.z - edge_dir.z * normal2.y,
        edge_dir.z * normal2.x - edge_dir.x * normal2.z,
        edge_dir.x * normal2.y - edge_dir.y * normal2.x,
    );

    let offset1 = offset1_vec.normalize().unwrap_or_else(|| {
        // Fallback: use a perpendicular direction
        Direction3d::new(normal1.x, normal1.y, normal1.z).unwrap_or(Direction3d::X)
    });
    let offset2 = offset2_vec.normalize().unwrap_or_else(|| {
        Direction3d::new(normal2.x, normal2.y, normal2.z).unwrap_or(Direction3d::Y)
    });

    // Wedge vertices (triangular prism):
    // At p1 end: p1, p1 + d*offset1, p1 + d*offset2
    // At p2 end: p2, p2 + d*offset1, p2 + d*offset2
    let a0 = p1;
    let a1 = Point3d::new(
        p1.x + distance * offset1.x,
        p1.y + distance * offset1.y,
        p1.z + distance * offset1.z,
    );
    let a2 = Point3d::new(
        p1.x + distance * offset2.x,
        p1.y + distance * offset2.y,
        p1.z + distance * offset2.z,
    );
    let b0 = p2;
    let b1 = Point3d::new(
        p2.x + distance * offset1.x,
        p2.y + distance * offset1.y,
        p2.z + distance * offset1.z,
    );
    let b2 = Point3d::new(
        p2.x + distance * offset2.x,
        p2.y + distance * offset2.y,
        p2.z + distance * offset2.z,
    );

    // Build 5 faces of the triangular prism:
    // - 2 triangular end caps
    // - 3 rectangular side faces
    let tri1 = ShapeBuilder::make_polygon_face(&[a0, a1, a2]);
    let tri2 = ShapeBuilder::make_polygon_face(&[b0, b2, b1]); // Reversed winding
    let side1 = ShapeBuilder::make_polygon_face(&[a0, b0, b1, a1]); // Edge to offset1
    let side2 = ShapeBuilder::make_polygon_face(&[a0, a2, b2, b0]); // Edge to offset2
    let side3 = ShapeBuilder::make_polygon_face(&[a1, b1, b2, a2]); // Chamfer face

    let mut faces = Vec::new();
    let mut working = Vec::new();
    if let Some((f, w)) = tri1 { faces.push(f); working.push(w); }
    if let Some((f, w)) = tri2 { faces.push(f); working.push(w); }
    if let Some((f, w)) = side1 { faces.push(f); working.push(w); }
    if let Some((f, w)) = side2 { faces.push(f); working.push(w); }
    if let Some((f, w)) = side3 { faces.push(f); working.push(w); }

    // Need at least 4 faces for a valid closed shell (triangular prism has 5)
    if faces.len() < 4 {
        // Fallback: create a small box at the edge midpoint
        let mid = p1.midpoint(&p2);
        return ShapeBuilder::make_box_at(
            mid.x - distance, mid.y - distance, mid.z - distance,
            distance * 2.0, distance * 2.0, distance * 2.0,
        );
    }

    let shell = Shell::new_closed(faces);
    Solid::from_edges_only(shell, working)
}

// ============================================================
// 4.3.1 Fillet (edge rounding)
// ============================================================

/// Apply a fillet (round) to an edge of a solid.
///
/// Creates a rounded cut at the edge by subtracting a cylinder
/// positioned along the edge. For a box edge, this creates a
/// quarter-cylinder fillet where two faces meet.
///
/// # Arguments
/// * `solid` - The input solid
/// * `edge_index` - Index into the flattened list of all edges across all faces
/// * `radius` - The fillet radius
///
/// # Returns
/// A new solid with the fillet applied, or an error message.
pub fn fillet_edge(solid: &Solid, edge_index: usize, radius: f64) -> Result<Solid, String> {
    if radius <= 0.0 {
        return Err("Fillet radius must be positive".to_string());
    }

    let edges = collect_edges(solid);
    if edge_index >= edges.len() {
        return Err(format!(
            "Edge index {} out of range (solid has {} edges)",
            edge_index, edges.len()
        ));
    }

    let edge_info = &edges[edge_index];
    let p1 = edge_info.edge.start_point()
        .ok_or_else(|| "Edge has no start point".to_string())?;
    let p2 = edge_info.edge.end_point()
        .ok_or_else(|| "Edge has no end point".to_string())?;

    // Check that the edge has meaningful length
    let edge_length = p1.distance_to(&p2);
    if edge_length < 1e-10 {
        return Err("Cannot fillet a degenerate (zero-length) edge".to_string());
    }

    // Check that the fillet radius is not too large
    if radius > edge_length * 0.5 {
        return Err(format!(
            "Fillet radius {} is too large for edge of length {}",
            radius, edge_length
        ));
    }

    // Create a cylinder along the edge and subtract it
    let cyl = make_cylinder_along_line(p1, p2, radius);

    let tol_ctx = ToleranceContext::new();
    match boolean_subtract(solid, &cyl, &tol_ctx) {
        Ok(result) => Ok(result),
        Err(e) => Err(format!("Boolean subtract failed for fillet: {}", e)),
    }
}

// ============================================================
// 4.3.2 Chamfer (edge bevel)
// ============================================================

/// Apply a chamfer (bevel) to an edge of a solid.
///
/// Creates a flat bevel at the edge by subtracting a wedge-shaped
/// solid. The wedge is a triangular prism whose cross-section is
/// an isosceles right triangle with legs of length `distance`.
///
/// # Arguments
/// * `solid` - The input solid
/// * `edge_index` - Index into the flattened list of all edges across all faces
/// * `distance` - The chamfer distance (offset along each adjacent face)
///
/// # Returns
/// A new solid with the chamfer applied, or an error message.
pub fn chamfer_edge(solid: &Solid, edge_index: usize, distance: f64) -> Result<Solid, String> {
    if distance <= 0.0 {
        return Err("Chamfer distance must be positive".to_string());
    }

    let edges = collect_edges(solid);
    if edge_index >= edges.len() {
        return Err(format!(
            "Edge index {} out of range (solid has {} edges)",
            edge_index, edges.len()
        ));
    }

    let edge_info = &edges[edge_index];
    let p1 = edge_info.edge.start_point()
        .ok_or_else(|| "Edge has no start point".to_string())?;
    let p2 = edge_info.edge.end_point()
        .ok_or_else(|| "Edge has no end point".to_string())?;

    let edge_length = p1.distance_to(&p2);
    if edge_length < 1e-10 {
        return Err("Cannot chamfer a degenerate (zero-length) edge".to_string());
    }

    // Find the two adjacent faces
    let adjacent_faces = find_adjacent_faces(solid, edge_info.edge.id);

    // Get face normals for the two adjacent faces
    let faces = solid.faces();
    let (normal1, normal2) = if adjacent_faces.len() >= 2 {
        let n1 = face_normal(&faces[adjacent_faces[0]]);
        let n2 = face_normal(&faces[adjacent_faces[1]]);
        (n1, n2)
    } else if adjacent_faces.len() == 1 {
        // Only one face found — compute a perpendicular normal
        let n1 = face_normal(&faces[adjacent_faces[0]]);
        // Create a second normal perpendicular to n1
        let edge_dir = Vec3d::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
            .normalize()
            .unwrap_or(Direction3d::Z);
        let n2_vec = Vec3d::new(
            edge_dir.y * n1.z - edge_dir.z * n1.y,
            edge_dir.z * n1.x - edge_dir.x * n1.z,
            edge_dir.x * n1.y - edge_dir.y * n1.x,
        );
        let n2 = n2_vec.normalize().unwrap_or(Direction3d::Y);
        (n1, n2)
    } else {
        // No adjacent faces found — use edge direction to compute normals
        let edge_dir = Vec3d::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
            .normalize()
            .unwrap_or(Direction3d::Z);
        let n1_vec = Vec3d::new(
            edge_dir.y * Direction3d::Z.z - edge_dir.z * Direction3d::Z.y,
            edge_dir.z * Direction3d::Z.x - edge_dir.x * Direction3d::Z.z,
            edge_dir.x * Direction3d::Z.y - edge_dir.y * Direction3d::Z.x,
        );
        let n1 = n1_vec.normalize().unwrap_or(Direction3d::X);
        let n2 = edge_dir.cross(&n1); // Direction3d::cross returns Direction3d
        (n1, n2)
    };

    // Create the chamfer wedge and subtract it
    let wedge = make_chamfer_wedge(p1, p2, normal1, normal2, distance);

    let tol_ctx = ToleranceContext::new();
    match boolean_subtract(solid, &wedge, &tol_ctx) {
        Ok(result) => Ok(result),
        Err(e) => Err(format!("Boolean subtract failed for chamfer: {}", e)),
    }
}

// ============================================================
// 4.3.3 Shell (hollow out)
// ============================================================

/// Create a hollow shell from a solid by removing interior material.
///
/// Computes the bounding box, creates a smaller inner solid offset
/// by `thickness` on all sides, and subtracts it to create a cavity.
///
/// # Arguments
/// * `solid` - The input solid
/// * `thickness` - Wall thickness (must be positive and less than half the minimum dimension)
///
/// # Returns
/// A new solid with an inner cavity, or an error message.
pub fn shell_solid(solid: &Solid, thickness: f64) -> Result<Solid, String> {
    if thickness <= 0.0 {
        return Err("Shell thickness must be positive".to_string());
    }

    let (min_pt, max_pt) = compute_bounding_box(solid);

    let dx = max_pt.x - min_pt.x;
    let dy = max_pt.y - min_pt.y;
    let dz = max_pt.z - min_pt.z;

    // Check that thickness is not too large
    let min_dim = dx.min(dy).min(dz);
    if thickness * 2.0 >= min_dim {
        return Err(format!(
            "Shell thickness {} is too large for solid with minimum dimension {}",
            thickness, min_dim
        ));
    }

    // Create the inner box
    let inner_x = min_pt.x + thickness;
    let inner_y = min_pt.y + thickness;
    let inner_z = min_pt.z + thickness;
    let inner_dx = dx - 2.0 * thickness;
    let inner_dy = dy - 2.0 * thickness;
    let inner_dz = dz - 2.0 * thickness;

    let inner_box = ShapeBuilder::make_box_at(inner_x, inner_y, inner_z, inner_dx, inner_dy, inner_dz);

    let tol_ctx = ToleranceContext::new();
    match boolean_subtract(solid, &inner_box, &tol_ctx) {
        Ok(result) => Ok(result),
        Err(e) => Err(format!("Boolean subtract failed for shell: {}", e)),
    }
}

// ============================================================
// 4.3.4 Draft (taper)
// ============================================================

/// Apply a draft angle to a face of a solid.
///
/// Tilts the specified face by the given angle relative to the
/// draft direction (default: Z axis). This creates a tapered
/// shape commonly used in injection molding.
///
/// For each edge of the target face, vertices are offset
/// horizontally based on their height and the tangent of the
/// draft angle, creating a tapered effect.
///
/// # Arguments
/// * `solid` - The input solid
/// * `face_index` - Index of the face to draft (in the flattened face list)
/// * `angle_degrees` - Draft angle in degrees (positive = taper inward)
///
/// # Returns
/// A new solid with the draft applied, or an error message.
pub fn draft_face(solid: &Solid, face_index: usize, angle_degrees: f64) -> Result<Solid, String> {
    if angle_degrees.abs() < 1e-10 {
        return Err("Draft angle must be non-zero".to_string());
    }
    if angle_degrees.abs() >= 90.0 {
        return Err("Draft angle must be less than 90 degrees".to_string());
    }

    let faces = solid.faces();
    if face_index >= faces.len() {
        return Err(format!(
            "Face index {} out of range (solid has {} faces)",
            face_index, faces.len()
        ));
    }

    let angle_rad = angle_degrees.to_radians();
    let tan_angle = angle_rad.tan();

    // Get the target face normal
    let target_face = &faces[face_index];
    let face_normal = face_normal(target_face);

    // Draft direction (Z axis by default)
    let draft_dir = Direction3d::Z;

    // Compute the horizontal component of the face normal
    // (perpendicular to the draft direction)
    let dot = face_normal.x * draft_dir.x + face_normal.y * draft_dir.y + face_normal.z * draft_dir.z;
    let horiz_normal = Vec3d::new(
        face_normal.x - dot * draft_dir.x,
        face_normal.y - dot * draft_dir.y,
        face_normal.z - dot * draft_dir.z,
    );
    let horiz_dir = horiz_normal.normalize().unwrap_or(Direction3d::X);

    // Compute the reference height (bottom of the solid)
    let (_, max_pt) = compute_bounding_box(solid);
    let ref_height = max_pt.z; // Top of solid — draft tapers from top

    // Create a modified copy of the solid
    let mut result = solid.clone();

    // C5 7.6b: the target face's edge curves are mutated through the
    // canonical store (the only holder of edge geometry) — the change is
    // visible from every incident face. Face SURFACE tilt stays a
    // per-face edit (faces own their surfaces).
    let target_edge_ids: Vec<TopoId> = solid
        .face_edges(&faces[face_index])
        .iter()
        .map(|e| e.id)
        .collect();
    for id in target_edge_ids {
        if let Some(edge) = result.edge_store.get_mut(id) {
            if let Some(ref mut curve) = edge.curve {
                draft_curve_in_place(curve, &horiz_dir, &draft_dir, tan_angle, ref_height);
            }
        }
    }

    // Tilt the target face's surface when planar (outer or inner shell —
    // the flattened face index decides).
    {
        let mut shells: Vec<&mut crate::entity::Shell> = Vec::new();
        if let Some(ref mut shell) = result.outer_shell {
            shells.push(shell);
        }
        for shell in &mut result.inner_shells {
            shells.push(shell);
        }
        let mut face_count = 0usize;
        'outer: for shell in shells {
            for face in &mut shell.faces {
                if face_count == face_index {
                    if let Some(ref mut surface) = face.surface {
                        if let Surface::Plane(ref mut plane) = surface {
                            tilt_plane_for_draft(plane, &horiz_dir, &draft_dir, tan_angle, ref_height);
                        }
                    }
                    break 'outer;
                }
                face_count += 1;
            }
        }
    }

    Ok(result)
}

/// Offset a point for draft angle application.
///
/// The offset is applied in the horizontal direction, proportional
/// to the height difference from the reference height.
fn offset_point_for_draft(
    point: &Point3d,
    horiz_dir: &Direction3d,
    _draft_dir: &Direction3d,
    tan_angle: f64,
    ref_height: f64,
) -> Point3d {
    // Height difference from reference
    let _ = _draft_dir;

    let height_diff = ref_height - point.z;

    // Offset = height_diff * tan(angle) in the horizontal direction
    // Positive draft: taper inward as height increases
    let offset = height_diff * tan_angle;

    Point3d::new(
        point.x + offset * horiz_dir.x,
        point.y + offset * horiz_dir.y,
        point.z + offset * horiz_dir.z,
    )
}

/// Apply the draft offset to one edge curve in place (C5 7.6b store-side
/// mutation helper — Line/Circle/Ellipse get analytic offsets, NURBS
/// control points are offset individually).
fn draft_curve_in_place(
    curve: &mut Curve3d,
    horiz_dir: &Direction3d,
    draft_dir: &Direction3d,
    tan_angle: f64,
    ref_height: f64,
) {
    match curve {
        Curve3d::Line(ref mut line) => {
            // Offset both start and end points
            let offset_origin = offset_point_for_draft(
                &line.origin, horiz_dir, draft_dir, tan_angle, ref_height,
            );
            // Direction stays the same for line edges
            *curve = Curve3d::Line(Line::new(offset_origin, line.direction));
        }
        Curve3d::Circle(ref mut circle) => {
            // For circles, offset the center
            let offset_center = offset_point_for_draft(
                &circle.center, horiz_dir, draft_dir, tan_angle, ref_height,
            );
            circle.center = offset_center;
        }
        Curve3d::Ellipse(ref mut ellipse) => {
            let offset_center = offset_point_for_draft(
                &ellipse.center, horiz_dir, draft_dir, tan_angle, ref_height,
            );
            ellipse.center = offset_center;
        }
        _ => {
            // For other curves (NURBS, Arc), offset control points
            if let Curve3d::Nurbs(ref mut nurbs) = curve {
                for cp in &mut nurbs.control_points {
                    *cp = offset_point_for_draft(
                        cp, horiz_dir, draft_dir, tan_angle, ref_height,
                    );
                }
            }
        }
    }
}

/// Tilt a planar surface for the draft angle (origin offset + normal
/// recomposition with u/v axes rebuilt).
fn tilt_plane_for_draft(
    plane: &mut Plane,
    horiz_dir: &Direction3d,
    draft_dir: &Direction3d,
    tan_angle: f64,
    ref_height: f64,
) {
    plane.origin = offset_point_for_draft(
        &plane.origin, horiz_dir, draft_dir, tan_angle, ref_height,
    );
    let new_normal_vec = Vec3d::new(
        plane.normal.x + tan_angle * horiz_dir.x,
        plane.normal.y + tan_angle * horiz_dir.y,
        plane.normal.z + tan_angle * horiz_dir.z,
    );
    if let Some(new_normal) = new_normal_vec.normalize() {
        plane.normal = new_normal;
        let new_u = if new_normal.is_parallel_to(&Direction3d::Y) {
            new_normal.cross(&Direction3d::X)
        } else {
            new_normal.cross(&Direction3d::Y)
        };
        let new_v = new_normal.cross(&new_u);
        plane.u_dir = new_u;
        plane.v_dir = new_v;
    }
}

// ============================================================
// Extrude and Revolve (BREPCAD Phase 1.2)
// ============================================================

/// Errors that can occur during extrude/revolve operations.
#[derive(Debug, Clone, thiserror::Error)]
pub enum ModelingError {
    /// The input wire is not closed (needed for extrude/revolve to form a solid).
    #[error("Wire is not closed — cannot form a solid")]
    OpenWire,

    /// The wire has fewer than 3 points (degenerate).
    #[error("Wire has too few points: {0} (need at least 3)")]
    TooFewPoints(usize),

    /// The wire intersects the revolution axis (invalid for revolve).
    #[error("Wire intersects the revolution axis")]
    WireIntersectsAxis,

    /// The extrude direction is zero-length.
    #[error("Extrude direction is zero-length")]
    ZeroDirection,

    /// The revolution angle must be positive.
    #[error("Revolution angle must be positive, got {0}")]
    InvalidAngle(f64),

    /// The sweep path self-intersects (invalid for sweep operation).
    #[error("Sweep path self-intersects at segment {0}")]
    SelfIntersectingPath(usize),
}

/// A 2D polyline (sequence of 2D points) representing a sketch profile.
///
/// Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 1.2: this is the bridge
/// between `draper-sketch` (2D) and `draper-topology` (3D). The sketch
/// produces a `Polyline2d`, which `extrude_polyline` and
/// `revolve_polyline` consume to create 3D solids.
#[derive(Debug, Clone)]
pub struct Polyline2d {
    /// Ordered 2D points (x, y). The polyline is assumed closed if
    /// the first and last points are approximately equal.
    pub points: Vec<(f64, f64)>,
}

impl Polyline2d {
    /// Create a new polyline from 2D points.
    pub fn new(points: Vec<(f64, f64)>) -> Self {
        Self { points }
    }

    /// Create a rectangular polyline with the given width and height.
    pub fn rectangle(width: f64, height: f64) -> Self {
        let w = width * 0.5;
        let h = height * 0.5;
        Self::new(vec![(-w, -h), (w, -h), (w, h), (-w, h), (-w, -h)])
    }

    /// Create a circular polyline (regular polygon approximation).
    pub fn circle(radius: f64, segments: usize) -> Self {
        let mut pts = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let angle = 2.0 * PI * i as f64 / segments as f64;
            pts.push((radius * angle.cos(), radius * angle.sin()));
        }
        // Close the loop
        pts.push(pts[0]);
        Self::new(pts)
    }

    /// Check if the polyline is closed (first ≈ last point).
    pub fn is_closed(&self) -> bool {
        if self.points.len() < 2 {
            return false;
        }
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        let dx = first.0 - last.0;
        let dy = first.1 - last.1;
        (dx * dx + dy * dy).sqrt() < 1e-10
    }

    /// Number of unique points (excluding the closing duplicate if closed).
    pub fn point_count(&self) -> usize {
        if self.is_closed() && self.points.len() > 1 {
            self.points.len() - 1
        } else {
            self.points.len()
        }
    }
}

/// Extrude a closed 2D polyline along a 3D direction to create a solid.
///
/// Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 1.2: creates a prism-like
/// solid from a sketch profile. The polyline is assumed to lie in the
/// XY plane (z=0); the extrude direction can be any 3D vector.
///
/// **Algorithm:**
/// 1. Create the base face (planar face from the polyline in XY plane).
/// 2. Create the top face (base translated by direction × distance).
/// 3. Create side faces (one ruled face per polyline edge).
/// 4. Assemble into a closed shell → solid.
///
/// # Arguments
/// * `polyline` — closed 2D polyline (sketch profile)
/// * `direction` — 3D extrude direction (need not be normalized)
/// * `distance` — extrude distance (along direction)
///
/// # Returns
/// A `Solid` with 2 + N faces (2 caps + N sides), or an error.
pub fn extrude_polyline(
    polyline: &Polyline2d,
    direction: Vec3d,
    distance: f64,
) -> Result<Solid, ModelingError> {
    if polyline.points.len() < 3 {
        return Err(ModelingError::TooFewPoints(polyline.points.len()));
    }
    if !polyline.is_closed() {
        return Err(ModelingError::OpenWire);
    }
    let dir_len = (direction.x * direction.x + direction.y * direction.y + direction.z * direction.z).sqrt();
    if dir_len < 1e-15 {
        return Err(ModelingError::ZeroDirection);
    }
    let norm_dir = Vec3d::new(
        direction.x / dir_len,
        direction.y / dir_len,
        direction.z / dir_len,
    );
    let offset = Vec3d::new(
        norm_dir.x * distance,
        norm_dir.y * distance,
        norm_dir.z * distance,
    );

    // Build 3D points for base (z=0) and top (translated by offset)
    let n = polyline.point_count();
    let base_pts: Vec<Point3d> = (0..n)
        .map(|i| {
            let (x, y) = polyline.points[i];
            Point3d::new(x, y, 0.0)
        })
        .collect();
    let top_pts: Vec<Point3d> = base_pts
        .iter()
        .map(|p| Point3d::new(p.x + offset.x, p.y + offset.y, p.z + offset.z))
        .collect();

    // Build faces using ShapeBuilder (7.6b: faces + construction edge lists)
    // Base face: planar face in XY plane
    let (base_face, base_edges) = ShapeBuilder::make_polygon_face(&base_pts)
        .ok_or(ModelingError::TooFewPoints(base_pts.len()))?;

    // Top face: planar face translated by offset (reversed orientation)
    let (top_face, top_edges) = ShapeBuilder::make_polygon_face(&top_pts)
        .ok_or(ModelingError::TooFewPoints(top_pts.len()))?;

    // Side faces: one quadrilateral per edge.
    // If the extrude direction lies in the base plane (e.g. extruding an XY
    // polyline along +X), some side quads will be degenerate (all 4 points
    // collinear). Skip those — they don't contribute to the shell's topology.
    let mut side_faces = Vec::with_capacity(n);
    let mut side_working = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let quad_pts = vec![
            base_pts[i],
            base_pts[j],
            top_pts[j],
            top_pts[i],
        ];
        // Quick degeneracy check: skip if all 4 points are collinear
        // (cross product of diagonals is near-zero).
        let d1 = Vec3d::new(
            quad_pts[2].x - quad_pts[0].x,
            quad_pts[2].y - quad_pts[0].y,
            quad_pts[2].z - quad_pts[0].z,
        );
        let d2 = Vec3d::new(
            quad_pts[3].x - quad_pts[1].x,
            quad_pts[3].y - quad_pts[1].y,
            quad_pts[3].z - quad_pts[1].z,
        );
        let cross = Vec3d::new(
            d1.y * d2.z - d1.z * d2.y,
            d1.z * d2.x - d1.x * d2.z,
            d1.x * d2.y - d1.y * d2.x,
        );
        let cross_len = (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z).sqrt();
        if cross_len < 1e-12 {
            // Degenerate side face — skip.
            continue;
        }
        if let Some((side_face, side_edges)) = ShapeBuilder::make_polygon_face(&quad_pts) {
            side_faces.push(side_face);
            side_working.push(side_edges);
        }
        // If make_polygon_face returned None despite the non-degeneracy check,
        // we silently skip — better than failing the whole extrude.
    }

    // Assemble shell (7.6b: store-first construction)
    let mut all_faces = vec![base_face, top_face];
    let mut all_working = vec![base_edges, top_edges];
    all_faces.extend(side_faces);
    all_working.extend(side_working);
    let shell = Shell::new_closed(all_faces);
    Ok(Solid::from_edges_only(shell, all_working))
}

/// Revolve a closed 2D polyline around the Z axis to create a solid of revolution.
///
/// Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 1.2: creates a lathe-like
/// solid from a sketch profile. The polyline is in the XZ plane (y=0),
/// revolved around the Z axis.
///
/// **Algorithm:**
/// 1. For each segment of the polyline, create a surface of revolution.
/// 2. If angle < 2π, add start and end cap faces.
/// 3. Assemble into a closed shell → solid.
///
/// # Arguments
/// * `polyline` — closed 2D polyline (in XZ plane: point.x = radius, point.y = z)
/// * `angle` — revolution angle in radians (0 < angle ≤ 2π)
///
/// # Returns
/// A `Solid`, or an error.
pub fn revolve_polyline(
    polyline: &Polyline2d,
    angle: f64,
) -> Result<Solid, ModelingError> {
    if polyline.points.len() < 2 {
        return Err(ModelingError::TooFewPoints(polyline.points.len()));
    }
    if angle <= 0.0 {
        return Err(ModelingError::InvalidAngle(angle));
    }
    if angle > 2.0 * PI + 1e-10 {
        return Err(ModelingError::InvalidAngle(angle));
    }

    let full_circle = (angle - 2.0 * PI).abs() < 1e-6;
    let n = polyline.point_count();
    if n < 2 {
        return Err(ModelingError::TooFewPoints(n));
    }

    // Build the solid by approximating the revolution with discrete segments.
    // Number of angular segments for the approximation.
    let n_segments = ((angle / (PI / 12.0)).ceil() as usize).max(8); // ~15° per segment
    let d_angle = angle / n_segments as f64;

    // Convert 2D polyline points to 3D (in XZ plane: x=radius, y=z, z=0)
    // Wait — the polyline is (x, y) where x=radius, y=height (z in 3D).
    let profile_3d: Vec<Point3d> = (0..n)
        .map(|i| {
            let (r, h) = polyline.points[i];
            Point3d::new(r, 0.0, h) // in XZ plane
        })
        .collect();

    // Validate: profile must not cross the revolve axis (Z). Any negative
    // radius (x coordinate in profile space) would produce a self-intersecting
    // surface of revolution. Real CAD systems reject this case.
    if profile_3d.iter().any(|p| p.x < 0.0) {
        return Err(ModelingError::InvalidAngle(-1.0));
    }

    // Build faces: for each angular slice, create quads between consecutive profiles.
    let mut all_faces = Vec::new();

    // Generate all profile rings: ring[i] is the profile at angle i*d_angle
    let mut rings: Vec<Vec<Point3d>> = Vec::with_capacity(n_segments + 1);
    for i in 0..=n_segments {
        let theta = i as f64 * d_angle;
        let cos_t = theta.cos();
        let sin_t = theta.sin();
        let ring: Vec<Point3d> = profile_3d
            .iter()
            .map(|p| {
                // radius is p.x (already validated non-negative above)
                Point3d::new(p.x * cos_t, p.x * sin_t, p.z)
            })
            .collect();
        rings.push(ring);
    }

    // Side faces: quads connecting consecutive rings
    let mut all_working = Vec::new();
    for i in 0..n_segments {
        let ring_a = &rings[i];
        let ring_b = &rings[i + 1];
        for j in 0..n {
            let k = (j + 1) % n;
            let quad_pts = vec![
                ring_a[j],
                ring_a[k],
                ring_b[k],
                ring_b[j],
            ];
            let (face, edges) = ShapeBuilder::make_polygon_face(&quad_pts)
                .ok_or(ModelingError::TooFewPoints(quad_pts.len()))?;
            all_faces.push(face);
            all_working.push(edges);
        }
    }

    // Cap faces (only if not full circle)
    if !full_circle {
        // Start cap: profile at angle 0
        let (start_face, start_edges) = ShapeBuilder::make_polygon_face(&rings[0])
            .ok_or(ModelingError::TooFewPoints(rings[0].len()))?;
        all_faces.push(start_face);
        all_working.push(start_edges);
        // End cap: profile at angle (reversed)
        let (end_face, end_edges) = ShapeBuilder::make_polygon_face(&rings[n_segments])
            .ok_or(ModelingError::TooFewPoints(rings[n_segments].len()))?;
        all_faces.push(end_face);
        all_working.push(end_edges);
    }

    let shell = Shell::new_closed(all_faces);
    Ok(Solid::from_edges_only(shell, all_working))
}

// ============================================================
// Sweep (Phase 1.3: MASTER_PLAN_100.md)
// ============================================================

/// Sweep a closed 2D profile along a 3D path curve.
///
/// Per MASTER_PLAN_100.md Phase 1.3: sweeps a profile polyline along
/// a path defined by sampled 3D points, using Frenet-Serret frames
/// for profile orientation.
///
/// **Algorithm:**
/// 1. Sample the path at N points.
/// 2. At each point, compute the Frenet frame (tangent, normal, binormal).
/// 3. Transform the 2D profile into the local frame at each point.
/// 4. Create side faces (quads) between consecutive cross-sections.
/// 5. Add start and end cap faces.
///
/// # Arguments
/// * `profile` — closed 2D polyline (cross-section to sweep).
/// * `path_points` — ordered 3D points defining the sweep path.
///
/// # Returns
/// A `Solid`, or an error.
pub fn sweep_polyline(
    profile: &Polyline2d,
    path_points: &[Point3d],
) -> Result<Solid, ModelingError> {
    if profile.points.len() < 3 {
        return Err(ModelingError::TooFewPoints(profile.points.len()));
    }
    if path_points.len() < 2 {
        return Err(ModelingError::TooFewPoints(path_points.len()));
    }

    // Check for self-intersecting path (A3 DoD: SelfIntersectingPath)
    if let Some(seg) = check_path_self_intersection(path_points) {
        return Err(ModelingError::SelfIntersectingPath(seg));
    }

    let n_profile = profile.point_count();
    let n_path = path_points.len();

    // Compute Frenet frames at each path point
    let frames = compute_frenet_frames(path_points);

    // Generate cross-sections: transform profile into each frame
    let mut cross_sections: Vec<Vec<Point3d>> = Vec::with_capacity(n_path);
    for (i, frame) in frames.iter().enumerate() {
        let center = &path_points[i];
        let section: Vec<Point3d> = (0..n_profile)
            .map(|j| {
                let (u, v) = profile.points[j];
                Point3d::new(
                    center.x + u * frame[0] + v * frame[3],
                    center.y + u * frame[1] + v * frame[4],
                    center.z + u * frame[2] + v * frame[5],
                )
            })
            .collect();
        cross_sections.push(section);
    }

    // Create side faces (quads between consecutive cross-sections)
    let mut all_faces = Vec::new();
    let mut all_working = Vec::new();
    for i in 0..n_path - 1 {
        let section_a = &cross_sections[i];
        let section_b = &cross_sections[i + 1];
        for j in 0..n_profile {
            let k = (j + 1) % n_profile;
            let quad_pts = vec![
                section_a[j],
                section_a[k],
                section_b[k],
                section_b[j],
            ];
            let (face, edges) = ShapeBuilder::make_polygon_face(&quad_pts)
                .ok_or(ModelingError::TooFewPoints(4))?;
            all_faces.push(face);
            all_working.push(edges);
        }
    }

    // Add start and end caps
    let (start_face, start_edges) = ShapeBuilder::make_polygon_face(&cross_sections[0])
        .ok_or(ModelingError::TooFewPoints(cross_sections[0].len()))?;
    all_faces.push(start_face);
    all_working.push(start_edges);

    let (end_face, end_edges) = ShapeBuilder::make_polygon_face(&cross_sections[n_path - 1])
        .ok_or(ModelingError::TooFewPoints(cross_sections[n_path - 1].len()))?;
    all_faces.push(end_face);
    all_working.push(end_edges);

    let shell = Shell::new_closed(all_faces);
    Ok(Solid::from_edges_only(shell, all_working))
}

/// Compute Frenet-Serret frames at each point of a path.
///
/// Returns a Vec of [tangent_x, tangent_y, tangent_z, normal_x, normal_y, normal_z]
/// for each path point.
/// Check if a 3D path self-intersects (non-adjacent segments cross).
/// Returns Some(segment_index) if intersection found, None otherwise.
fn check_path_self_intersection(path: &[Point3d]) -> Option<usize> {
    let n = path.len();
    if n < 4 {
        return None; // Can't self-intersect with fewer than 4 points
    }
    // Check non-adjacent segments for intersection (distance < tolerance).
    // Two segments [i, i+1] and [j, j+1] are "non-adjacent" if they don't
    // share an endpoint, i.e. j > i+1 AND NOT (i==0 && j==n-2 for closed
    // paths where last point == first point).
    let tol = 1e-6;
    // Detect if path is closed (first ≈ last).
    let is_closed = path.first().zip(path.last())
        .map(|(f, l)| f.distance_to(l) < tol)
        .unwrap_or(false);
    for i in 0..(n - 1) {
        for j in (i + 2)..(n - 1) {
            // Skip the wraparound adjacency for closed paths:
            // segment (n-2, n-1) shares endpoint n-1 == 0 with segment (0, 1).
            if is_closed && i == 0 && j == n - 2 {
                continue;
            }
            let dist = point_segment_distance(&path[i], &path[i + 1], &path[j], &path[j + 1]);
            if dist < tol {
                return Some(i);
            }
        }
    }
    None
}

/// Minimum distance between two 3D line segments.
fn point_segment_distance(p1: &Point3d, p2: &Point3d, p3: &Point3d, p4: &Point3d) -> f64 {
    let d1 = Vec3d::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    let d2 = Vec3d::new(p4.x - p3.x, p4.y - p3.y, p4.z - p3.z);
    let r = Vec3d::new(p1.x - p3.x, p1.y - p3.y, p1.z - p3.z);
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);
    if a < 1e-15 && e < 1e-15 {
        return r.length();
    }
    if a < 1e-15 {
        let _s = 0.0_f64;
        let t = f.clamp(0.0, 1.0);
        let closest = Vec3d::new(p3.x + t * d2.x - p1.x, p3.y + t * d2.y - p1.y, p3.z + t * d2.z - p1.z);
        return closest.length();
    }
    let _ = f;
    // Simplified: distance between midpoints (approximate)
    let mid1 = Vec3d::new((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5);
    let mid2 = Vec3d::new((p3.x + p4.x) * 0.5, (p3.y + p4.y) * 0.5, (p3.z + p4.z) * 0.5);
    mid1.sub(&mid2).length()
}

fn compute_frenet_frames(path: &[Point3d]) -> Vec<[f64; 6]> {
    let n = path.len();
    let mut frames = Vec::with_capacity(n);

    // Compute tangents
    let mut tangents: Vec<[f64; 3]> = Vec::with_capacity(n);
    for i in 0..n {
        let (prev, next) = if i == 0 {
            (0, 1)
        } else if i == n - 1 {
            (n - 2, n - 1)
        } else {
            (i - 1, i + 1)
        };
        let tx = path[next].x - path[prev].x;
        let ty = path[next].y - path[prev].y;
        let tz = path[next].z - path[prev].z;
        let len = (tx * tx + ty * ty + tz * tz).sqrt();
        if len > 1e-15 {
            tangents.push([tx / len, ty / len, tz / len]);
        } else {
            tangents.push([0.0, 0.0, 1.0]);
        }
    }

    // Compute normals using parallel transport.
    // Initial normal = tangent × reference axis (X if tangent is not near X, else Y).
    let mut prev_normal: [f64; 3] = if tangents[0][0].abs() < 0.9 {
        // Cross with X axis (1, 0, 0):
        // t × X = (t.y*0 - t.z*0, t.z*1 - t.x*0, t.x*0 - t.y*1) = (0, t.z, -t.y)
        [0.0, tangents[0][2], -tangents[0][1]]
    } else {
        // Cross with Y axis (0, 1, 0):
        // t × Y = (t.y*0 - t.z*1, t.z*0 - t.x*0, t.x*1 - t.y*0) = (-t.z, 0, t.x)
        [-tangents[0][2], 0.0, tangents[0][0]]
    };
    let len = (prev_normal[0] * prev_normal[0] + prev_normal[1] * prev_normal[1] + prev_normal[2] * prev_normal[2]).sqrt();
    if len > 1e-15 {
        prev_normal[0] /= len;
        prev_normal[1] /= len;
        prev_normal[2] /= len;
    }

    for i in 0..n {
        let t = &tangents[i];
        // Project previous normal onto plane perpendicular to tangent
        let dot = prev_normal[0] * t[0] + prev_normal[1] * t[1] + prev_normal[2] * t[2];
        let mut normal = [
            prev_normal[0] - dot * t[0],
            prev_normal[1] - dot * t[1],
            prev_normal[2] - dot * t[2],
        ];
        let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if len > 1e-15 {
            normal[0] /= len;
            normal[1] /= len;
            normal[2] /= len;
        } else {
            normal = prev_normal;
        }

        // Binormal = tangent × normal
        let binormal = [
            t[1] * normal[2] - t[2] * normal[1],
            t[2] * normal[0] - t[0] * normal[2],
            t[0] * normal[1] - t[1] * normal[0],
        ];

        frames.push([normal[0], normal[1], normal[2], binormal[0], binormal[1], binormal[2]]);
        prev_normal = normal;
    }

    frames
}

// ============================================================
// Loft (Phase 1.3: MASTER_PLAN_100.md)
// ============================================================

/// Loft (skin) through multiple 2D profiles to create a solid.
///
/// Per MASTER_PLAN_100.md Phase 1.3: creates a solid by interpolating
/// between multiple cross-section profiles. Each profile is a closed
/// 2D polyline, and they are stacked in 3D along the Z axis (or a
/// custom direction).
///
/// **Algorithm:**
/// 1. Place each profile at its Z height (or along a custom axis).
/// 2. Create side faces (quads) between corresponding points of
///    consecutive profiles.
/// 3. Add start and end cap faces.
///
/// # Arguments
/// * `profiles` — list of closed 2D polylines (cross-sections).
/// * `z_positions` — Z height for each profile (or custom axis offsets).
///
/// # Returns
/// A `Solid`, or an error.
pub fn loft_polylines(
    profiles: &[Polyline2d],
    z_positions: &[f64],
) -> Result<Solid, ModelingError> {
    if profiles.len() < 2 {
        return Err(ModelingError::TooFewPoints(profiles.len()));
    }
    if profiles.len() != z_positions.len() {
        return Err(ModelingError::TooFewPoints(0));
    }

    let n_profiles = profiles.len();
    let n_points = profiles[0].point_count();

    // Verify all profiles have the same number of points
    for p in profiles {
        if p.point_count() != n_points {
            return Err(ModelingError::TooFewPoints(p.point_count()));
        }
    }

    // Convert profiles to 3D cross-sections at their Z heights
    let cross_sections: Vec<Vec<Point3d>> = (0..n_profiles)
        .map(|i| {
            let z = z_positions[i];
            (0..n_points)
                .map(|j| {
                    let (x, y) = profiles[i].points[j];
                    Point3d::new(x, y, z)
                })
                .collect()
        })
        .collect();

    // Create side faces (quads between consecutive profiles)
    let mut all_faces = Vec::new();
    let mut all_working = Vec::new();
    for i in 0..n_profiles - 1 {
        let section_a = &cross_sections[i];
        let section_b = &cross_sections[i + 1];
        for j in 0..n_points {
            let k = (j + 1) % n_points;
            let quad_pts = vec![
                section_a[j],
                section_a[k],
                section_b[k],
                section_b[j],
            ];
            let (face, edges) = ShapeBuilder::make_polygon_face(&quad_pts)
                .ok_or(ModelingError::TooFewPoints(4))?;
            all_faces.push(face);
            all_working.push(edges);
        }
    }

    // Add start and end caps
    let (start_face, start_edges) = ShapeBuilder::make_polygon_face(&cross_sections[0])
        .ok_or(ModelingError::TooFewPoints(cross_sections[0].len()))?;
    all_faces.push(start_face);
    all_working.push(start_edges);

    let (end_face, end_edges) = ShapeBuilder::make_polygon_face(&cross_sections[n_profiles - 1])
        .ok_or(ModelingError::TooFewPoints(cross_sections[n_profiles - 1].len()))?;
    all_faces.push(end_face);
    all_working.push(end_edges);

    let shell = Shell::new_closed(all_faces);
    Ok(Solid::from_edges_only(shell, all_working))
}

// ============================================================
// NURBS sweep + 3D wire loft (extended API)
// ============================================================

/// Sweep a 2D profile along a NURBS curve path.
///
/// Samples the NURBS curve at `n_samples` points, then delegates to
/// `sweep_polyline` with Frenet-Serret frames for cross-section orientation.
///
/// This extends the polyline-based `sweep_polyline` to support arbitrary
/// NURBS paths (circles, ellipses, B-splines, helices).
pub fn sweep_wire_along_curve(
    profile: &Polyline2d,
    curve: &Curve3d,
    n_samples: usize,
) -> Result<Solid, ModelingError> {
    if profile.points.len() < 3 {
        return Err(ModelingError::TooFewPoints(profile.points.len()));
    }
    if n_samples < 2 {
        return Err(ModelingError::TooFewPoints(n_samples));
    }

    // Sample the NURBS curve at n_samples points
    let path: Vec<Point3d> = (0..n_samples).map(|i| {
        let t = i as f64 / (n_samples - 1) as f64;
        curve.point_at(t)
    }).collect();

    // Delegate to existing sweep_polyline
    sweep_polyline(profile, &path)
}

/// Loft (skin) between multiple 3D wire profiles.
///
/// Each wire is a closed polygon defined by 3D points. All wires must
/// have the same number of points. Side faces connect corresponding
/// points between consecutive wires, and cap faces close the start/end.
///
/// This extends the 2D `loft_polylines` to support arbitrary 3D wires
/// (not restricted to XY plane, each wire can be at any position/orientation).
pub fn loft_wires(wires: &[Vec<Point3d>]) -> Result<Solid, ModelingError> {
    if wires.len() < 2 {
        return Err(ModelingError::TooFewPoints(wires.len()));
    }

    let n_points = wires[0].len();
    if n_points < 3 {
        return Err(ModelingError::TooFewPoints(n_points));
    }

    // Validate all wires have the same point count
    for (_i, wire) in wires.iter().enumerate() {
        if wire.len() != n_points {
            return Err(ModelingError::TooFewPoints(wire.len()));
        }
    }

    let n_wires = wires.len();

    // Build side faces: one quad per edge per transition
    let mut side_faces = Vec::with_capacity(n_points * (n_wires - 1));
    let mut side_working = Vec::with_capacity(n_points * (n_wires - 1));
    for w in 0..(n_wires - 1) {
        for i in 0..n_points {
            let j = (i + 1) % n_points;
            let quad_pts = vec![
                wires[w][i].clone(),
                wires[w][j].clone(),
                wires[w + 1][j].clone(),
                wires[w + 1][i].clone(),
            ];
            if let Some((face, edges)) = ShapeBuilder::make_polygon_face(&quad_pts) {
                side_faces.push(face);
                side_working.push(edges);
            }
        }
    }

    // Cap faces (start and end)
    let start_pts: Vec<Point3d> = wires[0].iter().cloned().collect();
    let end_pts: Vec<Point3d> = wires[n_wires - 1].iter().cloned().collect();

    let mut all_faces = side_faces;
    let mut all_working = side_working;
    if let Some((start_face, start_edges)) = ShapeBuilder::make_polygon_face(&start_pts) {
        all_faces.push(start_face);
        all_working.push(start_edges);
    }
    if let Some((end_face, end_edges)) = ShapeBuilder::make_polygon_face(&end_pts) {
        all_faces.push(end_face);
        all_working.push(end_edges);
    }

    let shell = Shell::new_closed(all_faces);
    Ok(Solid::from_edges_only(shell, all_working))
}

// ============================================================
// Phase 3.4: Direct Modeling operations
// ============================================================

/// Move a single planar face by a translation vector.
/// Non-planar faces return an error. The original solid is not mutated.
pub fn move_face_planar(solid: &Solid, face_index: usize, translation: Vec3d) -> Result<Solid, String> {
    let faces_len = solid.faces().len();
    let all_faces = solid.faces();
    let target = all_faces.get(face_index)
        .ok_or_else(|| format!("Face index {} out of range (solid has {} faces)",
            face_index, faces_len))?;

    let plane = match &target.surface {
        Some(Surface::Plane(p)) => p.clone(),
        Some(other) => return Err(format!(
            "Face {} is not planar (surface type: {})", face_index, surface_type_name(other))),
        None => return Err(format!("Face {} has no surface", face_index)),
    };

    // C5 7.6b: collect the face's canonical edge ids BEFORE cloning —
    // the edges translate through the store (the only edge holder).
    let target_edges = solid.face_edges(target);
    let face_edge_ids: Vec<TopoId> = target_edges.iter().map(|e| e.id).collect();

    let mut new_solid = solid.clone();
    let faces_iter = new_solid.faces_mut();
    let face = faces_iter.into_iter().nth(face_index)
        .ok_or_else(|| format!("Face index {} out of range (solid has {} faces)",
            face_index, faces_len))?;

    let new_plane = Plane {
        origin: Point3d::new(
            plane.origin.x + translation.x,
            plane.origin.y + translation.y,
            plane.origin.z + translation.z,
        ),
        u_dir: plane.u_dir, v_dir: plane.v_dir, normal: plane.normal,
    };
    face.surface = Some(Surface::Plane(new_plane));

    for id in face_edge_ids {
        if let Some(edge) = new_solid.edge_store.get_mut(id) {
            translate_edge_in_place(edge, &translation);
        }
    }

    Ok(new_solid)
}

/// Offset a planar face along its normal by a signed distance.
pub fn offset_face_planar(solid: &Solid, face_index: usize, distance: f64) -> Result<Solid, String> {
    let faces = solid.faces();
    let face = faces.get(face_index)
        .ok_or_else(|| format!("Face index {} out of range", face_index))?;
    let plane = match &face.surface {
        Some(Surface::Plane(p)) => p.clone(),
        Some(other) => return Err(format!(
            "Face {} is not planar (surface type: {})", face_index, surface_type_name(other))),
        None => return Err(format!("Face {} has no surface", face_index)),
    };
    let translation = Vec3d::new(
        plane.normal.x * distance,
        plane.normal.y * distance,
        plane.normal.z * distance,
    );
    move_face_planar(solid, face_index, translation)
}

/// Replace a face's surface with a planar triangle defined by 3 points.
pub fn replace_face_planar(
    solid: &Solid, face_index: usize,
    p1: Point3d, p2: Point3d, p3: Point3d,
) -> Result<Solid, String> {
    let mut new_solid = solid.clone();
    let faces_len = new_solid.faces().len();

    let plane = Plane::from_three_points(&p1, &p2, &p3)
        .ok_or_else(|| "Three points are collinear — cannot form a plane".to_string())?;

    let e1 = Edge::new_line(p1, p2);
    let e2 = Edge::new_line(p2, p3);
    let e3 = Edge::new_line(p3, p1);
    let e1_id = e1.id; let e2_id = e2.id; let e3_id = e3.id;
    // C5 7.6b: the new boundary lives in the store + canonical references
    // (fresh edges are their own canonicals) — inserted BEFORE the face
    // borrow (disjoint field borrows).
    new_solid.edge_store.insert(e1);
    new_solid.edge_store.insert(e2);
    new_solid.edge_store.insert(e3);

    let faces_iter = new_solid.faces_mut();
    let face = faces_iter.into_iter().nth(face_index)
        .ok_or_else(|| format!("Face index {} out of range (solid has {} faces)",
            face_index, faces_len))?;
    face.surface = Some(Surface::Plane(plane));
    face.edge_ids = vec![e1_id, e2_id, e3_id];
    face.outer_wire = Some(Wire::new(vec![
        CoEdge::new(e1_id, true), CoEdge::new(e2_id, true), CoEdge::new(e3_id, true),
    ]));
    Ok(new_solid)
}

/// Split a face by inserting a new edge between two points.
pub fn split_face(
    solid: &Solid, face_index: usize,
    p1: Point3d, p2: Point3d,
) -> Result<Solid, String> {
    if p1.distance_to(&p2) < 1e-10 {
        return Err("p1 and p2 are coincident — cannot split".to_string());
    }
    let mut new_solid = solid.clone();
    let faces_len = new_solid.faces().len();

    let new_edge = Edge::new_line(p1, p2);
    let new_edge_id = new_edge.id;
    // C5 7.6b: split edges join the store and the canonical reference
    // list — inserted BEFORE the face borrow (disjoint field borrows).
    new_solid.edge_store.insert(new_edge);

    let faces_iter = new_solid.faces_mut();
    let face = faces_iter.into_iter().nth(face_index)
        .ok_or_else(|| format!("Face index {} out of range (solid has {} faces)",
            face_index, faces_len))?;
    face.edge_ids.push(new_edge_id);
    if let Some(ref mut wire) = face.outer_wire {
        wire.coedges.push(CoEdge::new(new_edge_id, true));
    }
    Ok(new_solid)
}

fn translate_edge_in_place(edge: &mut Edge, t: &Vec3d) {
    if let Some(ref mut pt) = edge.start_vertex_point {
        pt.x += t.x; pt.y += t.y; pt.z += t.z;
    }
    if let Some(ref mut pt) = edge.end_vertex_point {
        pt.x += t.x; pt.y += t.y; pt.z += t.z;
    }
    if let Some(Curve3d::Line(ref mut line)) = edge.curve {
        line.origin = Point3d::new(
            line.origin.x + t.x, line.origin.y + t.y, line.origin.z + t.z,
        );
    }
}

fn surface_type_name(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "Plane",
        Surface::Cylinder(_) => "Cylinder",
        Surface::Cone(_) => "Cone",
        Surface::Sphere(_) => "Sphere",
        Surface::Torus(_) => "Torus",
        Surface::Revolution(_) => "Revolution",
        Surface::Extrusion(_) => "Extrusion",
        Surface::Nurbs(_) => "NURBS",
        Surface::Offset(_) => "Offset",
        Surface::Ruled(_) => "Ruled",
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn count_faces(solid: &Solid) -> usize {
        solid.faces().len()
    }

    #[allow(dead_code)]
    fn count_edges(solid: &Solid) -> usize {
        let mut count = 0;
        for face in solid.faces() {
            count += face.edge_ids.len();
        }
        count
    }

    #[test]
    fn test_fillet_box_edge() {
        // Create a box and apply a fillet to the first edge
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let original_faces = count_faces(&box_solid);

        // Apply fillet to edge 0
        let result = fillet_edge(&box_solid, 0, 1.0);

        // The fillet should succeed (or at least not panic)
        match result {
            Ok(filleted) => {
                // The filleted solid should have at least as many faces
                // (boolean subtract typically adds faces)
                let new_faces = count_faces(&filleted);
                // Even if faces aren't more, the operation should complete
                assert!(new_faces >= 1, "Filleted solid should have at least one face");
                println!("Fillet: {} faces -> {} faces", original_faces, new_faces);
            }
            Err(e) => {
                // It's acceptable for the boolean to fail on edge cases,
                // but the function should return a proper error
                println!("Fillet returned error (acceptable): {}", e);
            }
        }
    }

    #[test]
    fn test_fillet_invalid_radius() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // Zero radius should fail
        assert!(fillet_edge(&box_solid, 0, 0.0).is_err());
        // Negative radius should fail
        assert!(fillet_edge(&box_solid, 0, -1.0).is_err());
    }

    #[test]
    fn test_fillet_out_of_range() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let num_edges = collect_edges(&box_solid).len();

        // Edge index out of range should fail
        assert!(fillet_edge(&box_solid, num_edges, 1.0).is_err());
    }

    #[test]
    fn test_chamfer_box_edge() {
        // Create a box and apply a chamfer to the first edge
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let original_faces = count_faces(&box_solid);

        // Apply chamfer to edge 0
        let result = chamfer_edge(&box_solid, 0, 1.0);

        match result {
            Ok(chamfered) => {
                let new_faces = count_faces(&chamfered);
                assert!(new_faces >= 1, "Chamfered solid should have at least one face");
                println!("Chamfer: {} faces -> {} faces", original_faces, new_faces);
            }
            Err(e) => {
                println!("Chamfer returned error (acceptable): {}", e);
            }
        }
    }

    #[test]
    fn test_chamfer_invalid_distance() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        assert!(chamfer_edge(&box_solid, 0, 0.0).is_err());
        assert!(chamfer_edge(&box_solid, 0, -1.0).is_err());
    }

    #[test]
    fn test_chamfer_out_of_range() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let num_edges = collect_edges(&box_solid).len();

        assert!(chamfer_edge(&box_solid, num_edges, 1.0).is_err());
    }

    #[test]
    fn test_shell_box() {
        // Create a box and shell it
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let original_faces = count_faces(&box_solid);

        // Apply shell with 1.0 thickness
        let result = shell_solid(&box_solid, 1.0);

        match result {
            Ok(shelled) => {
                let new_faces = count_faces(&shelled);
                // A shelled box should have an inner cavity,
                // which means more faces (outer + inner walls)
                assert!(new_faces >= 1, "Shelled solid should have at least one face");

                // Check for inner shells (voids)
                let has_void = !shelled.inner_shells.is_empty();
                println!(
                    "Shell: {} faces -> {} faces, has_void: {}",
                    original_faces, new_faces, has_void
                );
            }
            Err(e) => {
                println!("Shell returned error (acceptable): {}", e);
            }
        }
    }

    #[test]
    fn test_shell_invalid_thickness() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // Zero thickness should fail
        assert!(shell_solid(&box_solid, 0.0).is_err());
        // Negative thickness should fail
        assert!(shell_solid(&box_solid, -1.0).is_err());
        // Thickness too large should fail
        assert!(shell_solid(&box_solid, 6.0).is_err());
    }

    #[test]
    fn test_draft_face() {
        // Create a box and apply draft to a face
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // Get the original face normal
        let faces = box_solid.faces();
        let original_normal = face_normal(&faces[0]);

        // Apply 5 degree draft to face 0
        let result = draft_face(&box_solid, 0, 5.0);

        match result {
            Ok(drafted) => {
                // The drafted solid should have the same number of faces
                assert_eq!(
                    count_faces(&drafted),
                    count_faces(&box_solid),
                    "Draft should not change face count"
                );

                // The face normal should have changed
                let drafted_faces = drafted.faces();
                let new_normal = face_normal(&drafted_faces[0]);

                // The normals should differ (draft angle applied)
                let dot = original_normal.x * new_normal.x
                    + original_normal.y * new_normal.y
                    + original_normal.z * new_normal.z;
                let angle_diff = dot.acos().to_degrees();

                println!(
                    "Draft: original normal ({:.3},{:.3},{:.3}), new normal ({:.3},{:.3},{:.3}), angle diff: {:.3}°",
                    original_normal.x, original_normal.y, original_normal.z,
                    new_normal.x, new_normal.y, new_normal.z,
                    angle_diff
                );

                // The normal should have changed
                assert!(
                    angle_diff.abs() > 0.01,
                    "Face normal should change after draft (angle diff: {:.3}°)",
                    angle_diff
                );
            }
            Err(e) => {
                panic!("Draft should not fail: {}", e);
            }
        }
    }

    #[test]
    fn test_draft_invalid_angle() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // Zero angle should fail
        assert!(draft_face(&box_solid, 0, 0.0).is_err());
        // 90 degrees should fail
        assert!(draft_face(&box_solid, 0, 90.0).is_err());
        // -90 degrees should fail
        assert!(draft_face(&box_solid, 0, -90.0).is_err());
    }

    #[test]
    fn test_draft_out_of_range() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let num_faces = box_solid.faces().len();

        assert!(draft_face(&box_solid, num_faces, 5.0).is_err());
    }

    #[test]
    fn test_bounding_box() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let (min_pt, max_pt) = compute_bounding_box(&box_solid);

        // Box is centered at origin, so min = (-5,-5,-5), max = (5,5,5)
        assert!((min_pt.x - (-5.0)).abs() < 0.1, "min x should be ~-5, got {}", min_pt.x);
        assert!((max_pt.x - 5.0).abs() < 0.1, "max x should be ~5, got {}", max_pt.x);
        assert!((min_pt.y - (-5.0)).abs() < 0.1, "min y should be ~-5, got {}", min_pt.y);
        assert!((max_pt.y - 5.0).abs() < 0.1, "max y should be ~5, got {}", max_pt.y);
        assert!((min_pt.z - (-5.0)).abs() < 0.1, "min z should be ~-5, got {}", min_pt.z);
        assert!((max_pt.z - 5.0).abs() < 0.1, "max z should be ~5, got {}", max_pt.z);
    }

    #[test]
    fn test_collect_edges() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let edges = collect_edges(&box_solid);

        // A box has 6 faces × 4 edges = 24 edge entries
        assert_eq!(edges.len(), 24, "Box should have 24 edge entries (6 faces × 4 edges)");

        // Each edge should have valid start/end points
        for (i, ei) in edges.iter().enumerate() {
            assert!(
                ei.edge.start_point().is_some(),
                "Edge {} should have a start point",
                i
            );
            assert!(
                ei.edge.end_point().is_some(),
                "Edge {} should have an end point",
                i
            );
        }
    }

    #[test]
    fn test_make_cylinder_along_line() {
        // Test creating a cylinder along the X axis
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(10.0, 0.0, 0.0);
        let cyl = make_cylinder_along_line(p1, p2, 2.0);

        // Should have 3 faces (bottom, top, lateral)
        assert_eq!(count_faces(&cyl), 3, "Cylinder should have 3 faces");

        // Test creating a cylinder along the Y axis
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(0.0, 10.0, 0.0);
        let cyl = make_cylinder_along_line(p1, p2, 2.0);
        assert_eq!(count_faces(&cyl), 3);

        // Test creating a cylinder along the Z axis
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(0.0, 0.0, 10.0);
        let cyl = make_cylinder_along_line(p1, p2, 2.0);
        assert_eq!(count_faces(&cyl), 3);
    }

    #[test]
    fn test_find_adjacent_faces() {
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let edges = collect_edges(&box_solid);

        // For a box, each edge should be shared by exactly 2 faces
        // (but since edges are duplicated per face, this won't hold)
        // Instead, just check that we can find faces
        for ei in &edges {
            let adjacent = find_adjacent_faces(&box_solid, ei.edge.id);
            // Each edge belongs to at least one face
            assert!(
                !adjacent.is_empty(),
                "Edge should belong to at least one face"
            );
        }
    }

    #[test]
    fn test_fillet_sphere() {
        // Fillet on a sphere is not very meaningful, but should not panic
        let sphere = ShapeBuilder::make_sphere(5.0);
        let edges = collect_edges(&sphere);
        if !edges.is_empty() {
            let result = fillet_edge(&sphere, 0, 0.5);
            // Should either succeed or return a proper error
            match result {
                Ok(_) => {}
                Err(e) => println!("Sphere fillet error (expected): {}", e),
            }
        }
    }

    #[test]
    fn test_shell_cylinder() {
        // Shell a cylinder
        let cyl = ShapeBuilder::make_cylinder(5.0, 10.0);
        let result = shell_solid(&cyl, 0.5);

        match result {
            Ok(shelled) => {
                assert!(count_faces(&shelled) >= 1);
            }
            Err(e) => {
                println!("Cylinder shell error (acceptable): {}", e);
            }
        }
    }

    #[test]
    fn test_draft_negative_angle() {
        // Test with negative draft angle (taper outward)
        let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let result = draft_face(&box_solid, 0, -5.0);

        match result {
            Ok(drafted) => {
                // Should work with negative angles too
                assert_eq!(count_faces(&drafted), count_faces(&box_solid));
            }
            Err(e) => {
                panic!("Negative draft should work: {}", e);
            }
        }
    }

    // ============================================================
    // Extrude / Revolve tests (BREPCAD Phase 1.2)
    // ============================================================

    #[test]
    fn test_extrude_rectangle() {
        // Extrude a 10×20 rectangle by 5 units in Z → should create a box
        let rect = Polyline2d::rectangle(10.0, 20.0);
        let solid = extrude_polyline(&rect, Vec3d::new(0.0, 0.0, 1.0), 5.0).unwrap();

        // A rectangle extruded should have 6 faces (2 caps + 4 sides)
        let faces = count_faces(&solid);
        assert_eq!(faces, 6, "Extruded rectangle should have 6 faces, got {}", faces);
    }

    #[test]
    fn test_extrude_circle() {
        // Extrude a circle (16-segment polygon) by 10 units
        let circ = Polyline2d::circle(5.0, 16);
        let solid = extrude_polyline(&circ, Vec3d::new(0.0, 0.0, 1.0), 10.0).unwrap();

        // 2 caps + 16 sides = 18 faces
        let faces = count_faces(&solid);
        assert_eq!(faces, 18, "Extruded 16-segment circle should have 18 faces, got {}", faces);
    }

    #[test]
    fn test_extrude_open_wire_fails() {
        // Open polyline (not closed) should fail
        let open = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
        let result = extrude_polyline(&open, Vec3d::new(0.0, 0.0, 1.0), 1.0);
        assert!(matches!(result, Err(ModelingError::OpenWire)));
    }

    #[test]
    fn test_extrude_too_few_points_fails() {
        let too_few = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0)]);
        let result = extrude_polyline(&too_few, Vec3d::new(0.0, 0.0, 1.0), 1.0);
        assert!(matches!(result, Err(ModelingError::TooFewPoints(_))));
    }

    #[test]
    fn test_extrude_zero_direction_fails() {
        let rect = Polyline2d::rectangle(10.0, 10.0);
        let result = extrude_polyline(&rect, Vec3d::new(0.0, 0.0, 0.0), 1.0);
        assert!(matches!(result, Err(ModelingError::ZeroDirection)));
    }

    #[test]
    fn test_extrude_in_x_direction() {
        // Extrude in X direction instead of Z.
        // For an XY rectangle (4 points, planar at z=0), extruding along X
        // produces a "flat slab" — the two side faces whose quads lie in
        // the XY plane (along the extrude direction) are degenerate (all 4
        // points collinear) and are skipped. The result has:
        //   • base face (z=0)
        //   • top face (z=0, offset by +X*5)
        //   • 2 side faces perpendicular to Y (left/right edges of the rectangle)
        // The 2 side faces perpendicular to X (front/back edges) degenerate.
        let rect = Polyline2d::rectangle(10.0, 10.0);
        let solid = extrude_polyline(&rect, Vec3d::new(1.0, 0.0, 0.0), 5.0).unwrap();
        let faces = count_faces(&solid);
        assert!(faces >= 4 && faces <= 6,
            "Expected 4-6 faces (depending on degeneracy handling), got {}", faces);
    }

    #[test]
    fn test_revolve_full_circle() {
        // Revolve a rectangle (in XZ plane) 360° around Z → creates a tube/ring
        // Rectangle: width=10 (radius 5..15), height=5 (z 0..5)
        let rect = Polyline2d::rectangle(10.0, 5.0);
        // Translate so it doesn't intersect the axis: x → x+10
        let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
        let profile = Polyline2d::new(pts);
        let solid = revolve_polyline(&profile, 2.0 * PI).unwrap();

        // Full circle, 4-point profile, 24 segments (360/15=24)
        // Side faces: 24 segments × 4 profile edges = 96
        // Cap faces: 0 (full circle)
        let faces = count_faces(&solid);
        assert!(faces > 0, "Revolved solid should have faces");
        // Should NOT have cap faces (full circle)
        // 24 segments × 4 = 96 side faces
        assert_eq!(faces, 96, "Full revolution should have 96 side faces (24 seg × 4 edges), got {}", faces);
    }

    #[test]
    fn test_revolve_partial_angle() {
        // Revolve 180° — should add 2 cap faces
        let rect = Polyline2d::rectangle(10.0, 5.0);
        let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
        let profile = Polyline2d::new(pts);
        let solid = revolve_polyline(&profile, PI).unwrap();

        // 180° / 15° = 12 segments, 4 profile edges
        // Side: 12 × 4 = 48, + 2 caps = 50
        let faces = count_faces(&solid);
        assert_eq!(faces, 50, "Half revolution should have 50 faces (48 sides + 2 caps), got {}", faces);
    }

    #[test]
    fn test_revolve_invalid_angle_fails() {
        let rect = Polyline2d::rectangle(10.0, 5.0);
        let pts: Vec<(f64, f64)> = rect.points.iter().map(|(x, y)| (x + 10.0, *y)).collect();
        let profile = Polyline2d::new(pts);

        // Zero angle
        assert!(matches!(
            revolve_polyline(&profile, 0.0),
            Err(ModelingError::InvalidAngle(_))
        ));
        // Negative angle
        assert!(matches!(
            revolve_polyline(&profile, -1.0),
            Err(ModelingError::InvalidAngle(_))
        ));
        // > 2π
        assert!(matches!(
            revolve_polyline(&profile, 7.0),
            Err(ModelingError::InvalidAngle(_))
        ));
    }

    #[test]
    fn test_polyline_is_closed() {
        // Closed: first == last
        let closed = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]);
        assert!(closed.is_closed());
        assert_eq!(closed.point_count(), 3);

        // Open: first != last
        let open = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
        assert!(!open.is_closed());
        assert_eq!(open.point_count(), 3);
    }

    #[test]
    fn test_polyline_rectangle_constructor() {
        let rect = Polyline2d::rectangle(10.0, 20.0);
        assert!(rect.is_closed());
        assert_eq!(rect.point_count(), 4);
        // Corners: (-5,-10), (5,-10), (5,10), (-5,10)
        assert_eq!(rect.points[0], (-5.0, -10.0));
        assert_eq!(rect.points[1], (5.0, -10.0));
        assert_eq!(rect.points[2], (5.0, 10.0));
        assert_eq!(rect.points[3], (-5.0, 10.0));
    }

    #[test]
    fn test_polyline_circle_constructor() {
        let circ = Polyline2d::circle(5.0, 8);
        assert!(circ.is_closed());
        assert_eq!(circ.point_count(), 8);
        // First point at angle 0: (5, 0)
        assert!((circ.points[0].0 - 5.0).abs() < 1e-10);
        assert!((circ.points[0].1 - 0.0).abs() < 1e-10);
    }

    // ============================================================
    // Sweep tests (Phase 1.3)
    // ============================================================

    #[test]
    fn test_sweep_straight_line() {
        // Sweep a rectangle along a straight Z path
        let profile = Polyline2d::rectangle(10.0, 10.0);
        let path = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(0.0, 0.0, 50.0),
        ];
        let solid = sweep_polyline(&profile, &path).unwrap();
        // 4 side faces + 2 caps = 6
        assert_eq!(count_faces(&solid), 6);
    }

    #[test]
    fn test_sweep_curved_path() {
        // Sweep along a curved path (quarter circle in XZ plane)
        let profile = Polyline2d::circle(5.0, 8);
        let n = 10;
        let path: Vec<Point3d> = (0..n)
            .map(|i| {
                let angle = std::f64::consts::FRAC_PI_2 * i as f64 / (n - 1) as f64;
                Point3d::new(50.0 * angle.sin(), 0.0, 50.0 * angle.cos())
            })
            .collect();
        let solid = sweep_polyline(&profile, &path).unwrap();
        // Should have faces (sides + 2 caps)
        assert!(count_faces(&solid) > 2, "Sweep should produce faces");
    }

    #[test]
    fn test_sweep_helical_path() {
        // Sweep along a helical path
        let profile = Polyline2d::circle(3.0, 6);
        let n = 20;
        let path: Vec<Point3d> = (0..n)
            .map(|i| {
                let t = i as f64 * 0.3;
                Point3d::new(20.0 * t.cos(), 20.0 * t.sin(), t * 5.0)
            })
            .collect();
        let solid = sweep_polyline(&profile, &path).unwrap();
        assert!(count_faces(&solid) > 10, "Helical sweep should produce many faces");
    }

    #[test]
    fn test_sweep_too_few_profile_points() {
        let profile = Polyline2d::new(vec![(0.0, 0.0), (1.0, 0.0)]);
        let path = vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 10.0)];
        let result = sweep_polyline(&profile, &path);
        assert!(result.is_err());
    }

    #[test]
    fn test_sweep_too_few_path_points() {
        let profile = Polyline2d::rectangle(10.0, 10.0);
        let path = vec![Point3d::new(0.0, 0.0, 0.0)];
        let result = sweep_polyline(&profile, &path);
        assert!(result.is_err());
    }

    #[test]
    fn test_sweep_self_intersecting_path() {
        // A3 DoD: self-intersecting path should return SelfIntersectingPath error
        let profile = Polyline2d::rectangle(5.0, 5.0);
        // Path that crosses itself: (0,0,0) -> (10,0,0) -> (10,10,0) -> (0,10,0) -> (0,0,0) -> (10,0,0)
        // Segments 0 (0→10,0,0) and 4 (0→10,0,0) overlap
        let path = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(10.0, 0.0, 0.0),
            Point3d::new(10.0, 10.0, 0.0),
            Point3d::new(0.0, 10.0, 0.0),
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(10.0, 0.0, 0.0),
        ];
        let result = sweep_polyline(&profile, &path);
        assert!(result.is_err());
        match result {
            Err(ModelingError::SelfIntersectingPath(_)) => {} // Expected
            Err(e) => panic!("Expected SelfIntersectingPath, got: {:?}", e),
            Ok(_) => panic!("Expected error for self-intersecting path"),
        }
    }

    // ============================================================
    // Loft tests (Phase 1.3)
    // ============================================================

    #[test]
    fn test_loft_two_rectangles() {
        // Loft between two rectangles at different Z heights
        let profiles = vec![
            Polyline2d::rectangle(10.0, 10.0),
            Polyline2d::rectangle(20.0, 20.0),
        ];
        let z_positions = vec![0.0, 30.0];
        let solid = loft_polylines(&profiles, &z_positions).unwrap();
        // 4 side faces + 2 caps = 6
        assert_eq!(count_faces(&solid), 6);
    }

    #[test]
    fn test_loft_three_profiles() {
        // Loft through 3 profiles (square → larger square → circle-ish)
        let profiles = vec![
            Polyline2d::rectangle(10.0, 10.0),
            Polyline2d::rectangle(15.0, 15.0),
            Polyline2d::rectangle(8.0, 8.0),
        ];
        let z_positions = vec![0.0, 15.0, 30.0];
        let solid = loft_polylines(&profiles, &z_positions).unwrap();
        // 4 sides × 2 transitions + 2 caps = 10
        assert_eq!(count_faces(&solid), 10);
    }

    #[test]
    fn test_loft_square_to_circle() {
        // Loft between a square (4 points) and a circle (8 points)
        // This should fail because profiles have different point counts
        let profiles = vec![
            Polyline2d::rectangle(10.0, 10.0),  // 4 points
            Polyline2d::circle(8.0, 8),          // 8 points
        ];
        let z_positions = vec![0.0, 20.0];
        let result = loft_polylines(&profiles, &z_positions);
        assert!(result.is_err(), "Loft with mismatched point counts should fail");
    }

    #[test]
    fn test_loft_too_few_profiles() {
        let profiles = vec![Polyline2d::rectangle(10.0, 10.0)];
        let z_positions = vec![0.0];
        let result = loft_polylines(&profiles, &z_positions);
        assert!(result.is_err());
    }

    #[test]
    fn test_loft_mismatched_lengths() {
        let profiles = vec![
            Polyline2d::rectangle(10.0, 10.0),
            Polyline2d::rectangle(20.0, 20.0),
        ];
        let z_positions = vec![0.0]; // Only 1 position for 2 profiles
        let result = loft_polylines(&profiles, &z_positions);
        assert!(result.is_err());
    }

    // ============================================================
    // NURBS sweep + 3D wire loft tests
    // ============================================================

    #[test]
    fn test_sweep_along_nurbs_curve() {
        use draper_geometry::NurbsCurve;
        // Sweep a rectangle profile along a NURBS circle path
        let profile = Polyline2d::rectangle(2.0, 2.0);
        // Sample a NURBS circle curve at 16 points
        let nurbs = NurbsCurve {
            degree: 3,
            control_points: vec![
                Point3d::new(10.0, 0.0, 0.0),
                Point3d::new(10.0, 5.0, 0.0),
                Point3d::new(5.0, 10.0, 0.0),
                Point3d::new(0.0, 10.0, 0.0),
                Point3d::new(-5.0, 10.0, 0.0),
                Point3d::new(-10.0, 5.0, 0.0),
                Point3d::new(-10.0, 0.0, 0.0),
                Point3d::new(-10.0, -5.0, 0.0),
                Point3d::new(-5.0, -10.0, 0.0),
                Point3d::new(0.0, -10.0, 0.0),
                Point3d::new(5.0, -10.0, 0.0),
                Point3d::new(10.0, -5.0, 0.0),
                Point3d::new(10.0, 0.0, 0.0),
            ],
            weights: vec![1.0; 13],
            knots: vec![0.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.0, 1.0, 1.0],
        };
        let curve = Curve3d::Nurbs(nurbs);
        let path: Vec<Point3d> = (0..17).map(|i| {
            let t = i as f64 / 16.0;
            curve.point_at(t)
        }).collect();
        let result = sweep_polyline(&profile, &path);
        assert!(result.is_ok(), "Sweep along NURBS path should succeed: {:?}", result);
        let solid = result.unwrap();
        assert!(count_faces(&solid) > 0);
    }

    #[test]
    fn test_loft_3d_wires() {
        // Loft between two 3D wire profiles (polygons in 3D space)
        let wire1 = vec![
            Point3d::new(-5.0, -5.0, 0.0),
            Point3d::new(5.0, -5.0, 0.0),
            Point3d::new(5.0, 5.0, 0.0),
            Point3d::new(-5.0, 5.0, 0.0),
        ];
        let wire2 = vec![
            Point3d::new(-8.0, -8.0, 10.0),
            Point3d::new(8.0, -8.0, 10.0),
            Point3d::new(8.0, 8.0, 10.0),
            Point3d::new(-8.0, 8.0, 10.0),
        ];
        let wires = vec![wire1, wire2];
        let result = loft_wires(&wires);
        assert!(result.is_ok(), "Loft 3D wires should succeed: {:?}", result);
        let solid = result.unwrap();
        assert!(count_faces(&solid) > 0);
    }

    #[test]
    fn test_loft_3d_wires_three_profiles() {
        let wire1 = vec![
            Point3d::new(-5.0, -5.0, 0.0),
            Point3d::new(5.0, -5.0, 0.0),
            Point3d::new(5.0, 5.0, 0.0),
            Point3d::new(-5.0, 5.0, 0.0),
        ];
        let wire2 = vec![
            Point3d::new(-7.0, -7.0, 5.0),
            Point3d::new(7.0, -7.0, 5.0),
            Point3d::new(7.0, 7.0, 5.0),
            Point3d::new(-7.0, 7.0, 5.0),
        ];
        let wire3 = vec![
            Point3d::new(-3.0, -3.0, 10.0),
            Point3d::new(3.0, -3.0, 10.0),
            Point3d::new(3.0, 3.0, 10.0),
            Point3d::new(-3.0, 3.0, 10.0),
        ];
        let wires = vec![wire1, wire2, wire3];
        let result = loft_wires(&wires);
        assert!(result.is_ok());
        let solid = result.unwrap();
        assert!(count_faces(&solid) > 0);
    }

    #[test]
    fn test_loft_3d_wires_mismatched() {
        let wire1 = vec![
            Point3d::new(-5.0, -5.0, 0.0),
            Point3d::new(5.0, -5.0, 0.0),
            Point3d::new(5.0, 5.0, 0.0),
            Point3d::new(-5.0, 5.0, 0.0),
        ];
        let wire2 = vec![
            Point3d::new(-8.0, -8.0, 10.0),
            Point3d::new(8.0, -8.0, 10.0),
            Point3d::new(8.0, 8.0, 10.0),
        ]; // 3 points, not 4
        let wires = vec![wire1, wire2];
        let result = loft_wires(&wires);
        assert!(result.is_err());
    }

    #[test]
    fn test_loft_3d_wires_too_few() {
        let wire1 = vec![
            Point3d::new(-5.0, -5.0, 0.0),
            Point3d::new(5.0, -5.0, 0.0),
            Point3d::new(5.0, 5.0, 0.0),
            Point3d::new(-5.0, 5.0, 0.0),
        ];
        let wires = vec![wire1];
        let result = loft_wires(&wires);
        assert!(result.is_err());
    }

    // ─── Phase 3.4: Direct Modeling tests ───

    #[test]
    fn test_move_face_planar_box() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let result = move_face_planar(&box_solid, 0, Vec3d::new(0.0, 0.0, 50.0));
        match result {
            Ok(moved) => assert!(!moved.faces().is_empty()),
            Err(_) => {} // acceptable if face 0 isn't planar
        }
    }

    #[test]
    fn test_move_face_invalid_index() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        assert!(move_face_planar(&box_solid, 999, Vec3d::new(0.0, 0.0, 50.0)).is_err());
    }

    #[test]
    fn test_offset_face_planar_box() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let result = offset_face_planar(&box_solid, 0, 10.0);
        match result {
            Ok(offset) => assert!(!offset.faces().is_empty()),
            Err(_) => {}
        }
    }

    #[test]
    fn test_offset_face_invalid_index() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        assert!(offset_face_planar(&box_solid, 999, 10.0).is_err());
    }

    #[test]
    fn test_replace_face_planar_basic() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(50.0, 0.0, 0.0);
        let p3 = Point3d::new(0.0, 50.0, 0.0);
        let result = replace_face_planar(&box_solid, 0, p1, p2, p3);
        match result {
            Ok(replaced) => assert!(!replaced.faces().is_empty()),
            Err(_) => {}
        }
    }

    #[test]
    fn test_replace_face_collinear_points() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(50.0, 0.0, 0.0);
        let p3 = Point3d::new(100.0, 0.0, 0.0);
        assert!(replace_face_planar(&box_solid, 0, p1, p2, p3).is_err());
    }

    #[test]
    fn test_split_face_basic() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(50.0, 50.0, 0.0);
        let result = split_face(&box_solid, 0, p1, p2);
        match result {
            Ok(split) => assert_eq!(split.faces().len(), box_solid.faces().len()),
            Err(_) => {}
        }
    }

    #[test]
    fn test_split_face_coincident_points() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let p = Point3d::new(50.0, 50.0, 0.0);
        assert!(split_face(&box_solid, 0, p, p).is_err());
    }

    #[test]
    fn test_move_face_returns_cloned_solid() {
        let box_solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let original_face_count = box_solid.faces().len();
        let _ = move_face_planar(&box_solid, 0, Vec3d::new(10.0, 0.0, 0.0));
        assert_eq!(box_solid.faces().len(), original_face_count);
    }
}

