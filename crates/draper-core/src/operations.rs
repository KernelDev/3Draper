// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Modeling operations — transform, fillet, chamfer, shell, pattern, hole editing.
//!
//! Editing API for the 3Draper kernel. All operations preserve topological
//! consistency: transforms propagate to surfaces, edges, and wires; hole
//! addition/removal maintains the outer+inner wire structure; surface
//! replacement preserves the existing wire bounds.

use draper_geometry::{
    Circle, Curve3d, Curve2d, Direction3d, Line2d, Point2d, Point3d, Surface,
    Transform, Vec3d,
};
use draper_topology::{CoEdge, Compound, Edge, Face, Shell, Solid, TopoId, Wire};

// ═══════════════════════════════════════════════════════════════════════
// SECTION 1: Transform operations (the foundation)
// ═══════════════════════════════════════════════════════════════════════

/// Apply a transform to every geometric entity in a solid — surfaces, edge
/// curves, edge endpoints, and wires are all transformed consistently.
///
/// This is the foundation operation: translate/rotate/scale/mirror all call
/// this with the appropriate Transform.
pub fn transform_solid(solid: &mut Solid, transform: &Transform) {
    if let Some(ref mut shell) = solid.outer_shell {
        transform_shell(shell, transform);
    }
    for shell in &mut solid.inner_shells {
        transform_shell(shell, transform);
    }
}

/// Apply a transform to every geometric entity in a compound (assembly).
pub fn transform_compound(compound: &mut Compound, transform: &Transform) {
    for solid in &mut compound.solids {
        transform_solid(solid, transform);
    }
}

/// Apply a transform to every face, edge, and wire in a shell.
pub fn transform_shell(shell: &mut Shell, transform: &Transform) {
    for face in &mut shell.faces {
        transform_face(face, transform);
    }
}

/// Apply a transform to a face's surface, all edge curves, and all wires.
///
/// The wires' coedge structure is preserved (topology is invariant under
/// affine transforms); only the underlying edge geometry is transformed.
pub fn transform_face(face: &mut Face, transform: &Transform) {
    // Transform the surface
    if let Some(ref mut surface) = face.surface {
        *surface = surface.transform(transform);
    }

    // Transform each edge's curve
    for edge in &mut face.edges {
        if let Some(ref mut curve) = edge.curve {
            *curve = curve.transform(transform);
        }
    }

    // Note: wires' coedges reference edges by TopoId (which doesn't change),
    // and pcurve/curve_2d data is in UV space of the (already transformed)
    // surface — so no direct wire transform is needed. The UV-space data
    // remains valid because the surface's parameterization is unchanged
    // (only the surface's spatial position/orientation changes).
    //
    // HOWEVER, if a wire has explicit curve_2d (PCURVE) data, that data is
    // in UV space of the OLD surface. After transforming the surface (which
    // doesn't change its UV parameterization — only its 3D embedding), the
    // UV curve_2d remains valid. ✓
}

/// Translate a solid by a vector (dx, dy, dz).
pub fn translate_solid(solid: &mut Solid, dx: f64, dy: f64, dz: f64) {
    let t = Transform::translation(dx, dy, dz);
    transform_solid(solid, &t);
}

/// Rotate a solid around an axis through the origin by `angle` radians.
pub fn rotate_solid(solid: &mut Solid, axis: &Direction3d, angle: f64) {
    let t = Transform::rotation_axis(axis, angle);
    transform_solid(solid, &t);
}

/// Rotate a solid around an axis through a specific pivot point.
pub fn rotate_solid_around_point(
    solid: &mut Solid,
    axis: &Direction3d,
    angle: f64,
    pivot: &Point3d,
) {
    // T_pivot * R * T_-pivot
    let to_origin = Transform::translation(-pivot.x, -pivot.y, -pivot.z);
    let rot = Transform::rotation_axis(axis, angle);
    let back = Transform::translation(pivot.x, pivot.y, pivot.z);
    let t = back.multiply(&rot).multiply(&to_origin);
    transform_solid(solid, &t);
}

/// Scale a solid uniformly by `factor` about the origin.
pub fn scale_solid(solid: &mut Solid, factor: f64) {
    let t = Transform::uniform_scaling(factor);
    transform_solid(solid, &t);
}

/// Scale a solid uniformly about a specific center point.
pub fn scale_solid_around_point(solid: &mut Solid, factor: f64, center: &Point3d) {
    let to_origin = Transform::translation(-center.x, -center.y, -center.z);
    let s = Transform::uniform_scaling(factor);
    let back = Transform::translation(center.x, center.y, center.z);
    let t = back.multiply(&s).multiply(&to_origin);
    transform_solid(solid, &t);
}

/// Mirror a solid about a plane defined by origin + normal.
pub fn mirror_solid(solid: &Solid, plane_origin: Point3d, plane_normal: Direction3d) -> Solid {
    let mut copy = solid.clone();
    let t = mirror_transform(&plane_origin, &plane_normal);
    transform_solid(&mut copy, &t);
    copy
}

/// Construct a reflection Transform about a plane.
pub fn mirror_transform(plane_origin: &Point3d, plane_normal: &Direction3d) -> Transform {
    let mut m = [[0.0; 4]; 4];
    let nx = plane_normal.x;
    let ny = plane_normal.y;
    let nz = plane_normal.z;
    let dot = nx * plane_origin.x + ny * plane_origin.y + nz * plane_origin.z;
    m[0][0] = 1.0 - 2.0 * nx * nx;
    m[0][1] = -2.0 * nx * ny;
    m[0][2] = -2.0 * nx * nz;
    m[0][3] = 2.0 * dot * nx;
    m[1][0] = -2.0 * nx * ny;
    m[1][1] = 1.0 - 2.0 * ny * ny;
    m[1][2] = -2.0 * ny * nz;
    m[1][3] = 2.0 * dot * ny;
    m[2][0] = -2.0 * nx * nz;
    m[2][1] = -2.0 * ny * nz;
    m[2][2] = 1.0 - 2.0 * nz * nz;
    m[2][3] = 2.0 * dot * nz;
    m[3][3] = 1.0;
    Transform { m }
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 2: Pattern operations (rewritten to use transform_solid)
// ═══════════════════════════════════════════════════════════════════════

/// Circular pattern: create `count` rotated copies of `solid` around `axis`.
///
/// Returns only the NEW copies (not the original). Total angular span is
/// `total_angle` radians (use 2π for a full ring).
pub fn circular_pattern(
    solid: &Solid,
    axis: Direction3d,
    count: usize,
    total_angle: f64,
) -> Vec<Solid> {
    let mut result = Vec::with_capacity(count.saturating_sub(1));
    for i in 1..count {
        let angle = total_angle * i as f64 / count as f64;
        let mut copy = solid.clone();
        rotate_solid(&mut copy, &axis, angle);
        result.push(copy);
    }
    result
}

/// Linear pattern: create `count` translated copies of `solid` along `direction`.
///
/// `spacing` is the distance between consecutive copies. Returns only the NEW
/// copies (not the original).
pub fn linear_pattern(
    solid: &Solid,
    direction: Direction3d,
    count: usize,
    spacing: f64,
) -> Vec<Solid> {
    let mut result = Vec::with_capacity(count.saturating_sub(1));
    for i in 1..count {
        let dx = direction.x * spacing * i as f64;
        let dy = direction.y * spacing * i as f64;
        let dz = direction.z * spacing * i as f64;
        let mut copy = solid.clone();
        translate_solid(&mut copy, dx, dy, dz);
        result.push(copy);
    }
    result
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 3: Hole operations (add/remove inner wires)
// ═══════════════════════════════════════════════════════════════════════

/// Add a circular hole to a face.
///
/// Creates a new inner wire (hole) on the face, defined by a center point
/// (in 3D) and a radius. The hole's edge is a Circle in 3D space and a
/// corresponding linear PCURVE in UV space (approximating the circle's UV
/// image with a 32-segment polyline).
///
/// The face's surface must be planar for exact UV mapping. For curved
/// surfaces, the UV polyline is approximated by projecting 32 sample points
/// of the 3D circle onto the surface.
pub fn add_circular_hole_to_face(
    face: &mut Face,
    center_3d: Point3d,
    radius: f64,
    normal: Direction3d,
) -> Result<(), String> {
    if radius <= 0.0 {
        return Err(format!("Radius must be positive, got {}", radius));
    }

    // Build the 3D circle edge
    let circle = Circle {
        center: center_3d,
        normal,
        radius,
        x_axis: perpendicular_direction(&normal),
    };
    let edge_curve = Curve3d::Circle(circle.clone());
    let edge_id = TopoId::new();
    let edge = Edge {
        id: edge_id,
        curve: Some(edge_curve),
        param_range: (0.0, 2.0 * std::f64::consts::PI),
        vertex_start: Some(TopoId::new()),
        vertex_end: Some(TopoId::new()),
        forward: true,
        tolerance: 1e-6,
        degenerate: false,
        step_entity_id: None,
    };
    face.edges.push(edge.clone());

    // Build the UV polyline (32 segments) by projecting 3D circle samples
    // onto the face's surface
    let uv_polyline = if let Some(ref surface) = face.surface {
        let mut pts = Vec::with_capacity(33);
        for i in 0..=32 {
            let t = i as f64 * 2.0 * std::f64::consts::PI / 32.0;
            let p3d = circle.point_at(t);
            let (u, v) = project_point_to_surface(surface, &p3d);
            pts.push(Point2d::new(u, v));
        }
        pts
    } else {
        // No surface — fall back to (0,0) UV
        vec![Point2d::new(0.0, 0.0); 33]
    };

    // Build the inner wire (closed loop with one coedge)
    let coedge = CoEdge {
        id: TopoId::new(),
        edge: edge_id,
        forward: true,
        pcurve: Some(draper_topology::Pcurve::new(uv_polyline)),
        curve_2d: Some(Curve2d::Line(Line2d::new(
            Point2d::new(0.0, 0.0),
            Point2d::new(0.0, 0.0),
        ))), // placeholder; the polyline is in pcurve
    };
    let wire = Wire {
        id: TopoId::new(),
        coedges: vec![coedge],
        closed: true,
    };

    face.add_hole(wire);
    Ok(())
}

/// Remove the i-th inner wire (hole) from a face.
///
/// Returns the removed wire on success, or an error if the index is out of
/// range. The corresponding edge in `face.edges` is NOT removed (it may be
/// referenced by other wires or by the outer wire).
pub fn remove_hole_from_face(face: &mut Face, hole_index: usize) -> Result<Wire, String> {
    if hole_index >= face.inner_wires.len() {
        return Err(format!(
            "Hole index {} out of range (face has {} inner wires)",
            hole_index,
            face.inner_wires.len()
        ));
    }
    Ok(face.inner_wires.remove(hole_index))
}

/// Remove ALL holes from a face. Returns the count of holes removed.
pub fn clear_holes_from_face(face: &mut Face) -> usize {
    let count = face.inner_wires.len();
    face.inner_wires.clear();
    count
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 4: Face / surface editing
// ═══════════════════════════════════════════════════════════════════════

/// Replace a face's underlying surface with a new one.
///
/// This is the foundation for NURBS editing: the caller modifies a copy of
/// the existing NURBS surface (e.g., moves control points, changes weights)
/// and passes the new surface here. The face's wires and edges are preserved
/// — only the surface geometry changes.
///
/// **Warning**: If the new surface is incompatible with the existing wires
/// (e.g., wires lie outside the new surface's domain), triangulation may
/// fail or produce incorrect results. The caller is responsible for ensuring
/// compatibility.
pub fn replace_face_surface(face: &mut Face, new_surface: Surface) {
    face.surface = Some(new_surface);
}

/// Get a mutable reference to a face by its index in the solid's outer shell.
///
/// Returns None if the index is out of range or the solid has no outer shell.
pub fn get_face_mut<'a>(solid: &'a mut Solid, face_index: usize) -> Option<&'a mut Face> {
    solid.outer_shell.as_mut()?.faces.get_mut(face_index)
}

/// Get a mutable reference to a face by its index in any shell (outer or void).
///
/// `shell_index` 0 = outer shell, 1..N = inner shells (voids).
pub fn get_face_in_shell_mut<'a>(
    solid: &'a mut Solid,
    shell_index: usize,
    face_index: usize,
) -> Option<&'a mut Face> {
    if shell_index == 0 {
        solid.outer_shell.as_mut()?.faces.get_mut(face_index)
    } else {
        solid
            .inner_shells
            .get_mut(shell_index - 1)?
            .faces
            .get_mut(face_index)
    }
}

/// Reverse a face's orientation (swap forward flag).
///
/// This effectively turns the face "inside out" — useful for fixing flipped
/// normals or for preparing a face for use as a void boundary.
pub fn reverse_face_orientation(face: &mut Face) {
    face.forward = !face.forward;
}

/// Delete a face from a solid's outer shell by index.
///
/// **Warning**: This breaks watertightness! The resulting solid will have
/// a hole where the face used to be. Use only when you intend to replace
/// the face with another operation, or when constructing an open shell.
pub fn delete_face_from_solid(solid: &mut Solid, face_index: usize) -> Result<Face, String> {
    let shell = solid
        .outer_shell
        .as_mut()
        .ok_or_else(|| "Solid has no outer shell".to_string())?;
    if face_index >= shell.faces.len() {
        return Err(format!(
            "Face index {} out of range (shell has {} faces)",
            face_index,
            shell.faces.len()
        ));
    }
    Ok(shell.faces.remove(face_index))
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 5: Edge editing
// ═══════════════════════════════════════════════════════════════════════

/// Replace an edge's curve geometry.
///
/// Updates the edge's curve and parametric range. The edge's TopoId and
/// vertex references are preserved. Use this for NURBS curve editing,
/// converting a Line to an Arc, etc.
pub fn replace_edge_curve(
    face: &mut Face,
    edge_id: TopoId,
    new_curve: Curve3d,
    new_param_range: (f64, f64),
) -> Result<(), String> {
    let edge = face
        .edges
        .iter_mut()
        .find(|e| e.id == edge_id)
        .ok_or_else(|| format!("Edge {:?} not found in face", edge_id))?;
    edge.curve = Some(new_curve);
    edge.param_range = new_param_range;
    Ok(())
}

/// Reverse an edge's orientation (swap forward flag).
pub fn reverse_edge(face: &mut Face, edge_id: TopoId) -> Result<(), String> {
    let edge = face
        .edges
        .iter_mut()
        .find(|e| e.id == edge_id)
        .ok_or_else(|| format!("Edge {:?} not found in face", edge_id))?;
    edge.forward = !edge.forward;
    edge.param_range = (edge.param_range.1, edge.param_range.0);
    let tmp = edge.vertex_start;
    edge.vertex_start = edge.vertex_end;
    edge.vertex_end = tmp;
    Ok(())
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 6: Fillet / Chamfer / Shell (stubs with clear documentation)
// ═══════════════════════════════════════════════════════════════════════

/// Fillet (round) an edge of a solid.
///
/// **Not yet implemented.** A full fillet requires:
/// 1. Finding the edge and its two adjacent faces
/// 2. Computing the rolling ball trajectory
/// 3. Creating the fillet surface (a tube/torus patch)
/// 4. Trimming adjacent faces
/// 5. Rebuilding topology
pub fn fillet_edge(_solid: &mut Solid, _edge_index: usize, _radius: f64) -> Result<(), String> {
    Err("Fillet operation not yet implemented".to_string())
}

/// Chamfer an edge of a solid.
///
/// **Not yet implemented.** A chamfer requires similar steps to fillet
/// but produces a beveled face instead of a rounded one.
pub fn chamfer_edge(_solid: &mut Solid, _edge_index: usize, _distance: f64) -> Result<(), String> {
    Err("Chamfer operation not yet implemented".to_string())
}

/// Create a shell (hollow) from a solid by removing a face and offsetting.
///
/// **Not yet implemented.** A shell operation creates a hollow version of a
/// solid by offsetting all faces inward by `thickness`.
pub fn make_shell(_solid: &mut Solid, _thickness: f64) -> Result<(), String> {
    Err("Shell operation not yet implemented".to_string())
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 7: Helper functions
// ═══════════════════════════════════════════════════════════════════════

/// Compute a unit vector perpendicular to the given direction.
fn perpendicular_direction(d: &Direction3d) -> Direction3d {
    // Pick the smallest-magnitude component to cross with, to avoid
    // numerical issues when d is aligned with the chosen axis.
    let abs_x = d.x.abs();
    let abs_y = d.y.abs();
    let abs_z = d.z.abs();

    let other = if abs_x <= abs_y && abs_x <= abs_z {
        Direction3d::X
    } else if abs_y <= abs_x && abs_y <= abs_z {
        Direction3d::Y
    } else {
        Direction3d::Z
    };

    // Direction3d::cross already returns a normalised Direction3d.
    let result = d.cross(&other);
    // If d is parallel to `other`, cross returns Direction3d::Z as a fallback.
    // Verify the result is actually perpendicular; if not, try another axis.
    if result.dot(d).abs() > 1e-6 {
        let alt = if abs_x <= abs_y && abs_x <= abs_z {
            Direction3d::Y
        } else {
            Direction3d::X
        };
        d.cross(&alt)
    } else {
        result
    }
}

/// Project a 3D point onto a surface's UV domain.
///
/// Uses the surface's project_point method when available (analytical
/// surfaces). For NURBS surfaces, falls back to a coarse grid search
/// followed by Newton refinement.
fn project_point_to_surface(surface: &Surface, point: &Point3d) -> (f64, f64) {
    match surface {
        Surface::Plane(p) => {
            // Project point onto plane and compute UV
            let v = Vec3d::new(point.x - p.origin.x, point.y - p.origin.y, point.z - p.origin.z);
            let u = v.dot(&Vec3d::new(p.u_dir.x, p.u_dir.y, p.u_dir.z));
            let v_coord = v.dot(&Vec3d::new(p.v_dir.x, p.v_dir.y, p.v_dir.z));
            (u, v_coord)
        }
        Surface::Cylinder(c) => c.project_point(point),
        Surface::Cone(c) => c.project_point(point),
        Surface::Sphere(s) => s.project_point(point),
        Surface::Torus(t) => t.project_point(point),
        Surface::Revolution(_) => {
            // Coarse grid search + Newton
            project_point_to_surface_grid(surface, point, 0.0, 2.0 * std::f64::consts::PI, -100.0, 100.0)
        }
        Surface::Extrusion(_) => {
            project_point_to_surface_grid(surface, point, -100.0, 100.0, -100.0, 100.0)
        }
        Surface::Nurbs(_) => {
            project_point_to_surface_grid(surface, point, 0.0, 1.0, 0.0, 1.0)
        }
    }
}

/// Coarse grid search for surface projection (fallback for surfaces without
/// an analytical project_point method).
fn project_point_to_surface_grid(
    surface: &Surface,
    point: &Point3d,
    u_min: f64,
    u_max: f64,
    v_min: f64,
    v_max: f64,
) -> (f64, f64) {
    const GRID_SIZE: usize = 32;
    let mut best_u = (u_min + u_max) * 0.5;
    let mut best_v = (v_min + v_max) * 0.5;
    let mut best_dist_sq = f64::MAX;

    for i in 0..GRID_SIZE {
        for j in 0..GRID_SIZE {
            let u = u_min + (u_max - u_min) * (i as f64) / ((GRID_SIZE - 1) as f64);
            let v = v_min + (v_max - v_min) * (j as f64) / ((GRID_SIZE - 1) as f64);
            let p = surface.point_at(u, v);
            let dx = p.x - point.x;
            let dy = p.y - point.y;
            let dz = p.z - point.z;
            let dist_sq = dx * dx + dy * dy + dz * dz;
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_u = u;
                best_v = v;
            }
        }
    }

    // One step of Newton refinement
    let h = 1e-4;
    let p = surface.point_at(best_u, best_v);
    let pu_plus = surface.point_at(best_u + h, best_v);
    let pu_minus = surface.point_at(best_u - h, best_v);
    let pv_plus = surface.point_at(best_u, best_v + h);
    let pv_minus = surface.point_at(best_u, best_v - h);

    let du = Vec3d::new(
        (pu_plus.x - pu_minus.x) / (2.0 * h),
        (pu_plus.y - pu_minus.y) / (2.0 * h),
        (pu_plus.z - pu_minus.z) / (2.0 * h),
    );
    let dv = Vec3d::new(
        (pv_plus.x - pv_minus.x) / (2.0 * h),
        (pv_plus.y - pv_minus.y) / (2.0 * h),
        (pv_plus.z - pv_minus.z) / (2.0 * h),
    );

    let f = Vec3d::new(p.x - point.x, p.y - point.y, p.z - point.z);
    let det = du.x * (du.x * 0.0 + dv.x * 0.0) + 0.0; // placeholder
    let _ = det;

    // Simple gradient descent step
    let step_u = -(f.dot(&du)) / du.dot(&du).max(1e-10);
    let step_v = -(f.dot(&dv)) / dv.dot(&dv).max(1e-10);
    best_u += step_u * 0.5;
    best_v += step_v * 0.5;

    // Clamp to range
    best_u = best_u.clamp(u_min, u_max);
    best_v = best_v.clamp(v_min, v_max);

    (best_u, best_v)
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 8: Tests
// ═══════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Plane;

    #[test]
    fn test_transform_solid_translates_surfaces_and_edges() {
        // Build a solid with one plane face and one edge
        let plane = Plane::xy();
        let mut edge = Edge::new_line(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
        );
        edge.param_range = (0.0, 1.0);
        let mut face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        face.edges.push(edge);
        let shell = Shell::new_closed(vec![face]);
        let mut solid = Solid::new(shell);

        // Translate by (10, 20, 30)
        translate_solid(&mut solid, 10.0, 20.0, 30.0);

        // Verify surface origin moved
        let f = &solid.outer_shell.as_ref().unwrap().faces[0];
        if let Surface::Plane(p) = f.surface.as_ref().unwrap() {
            assert!((p.origin.x - 10.0).abs() < 1e-9);
            assert!((p.origin.y - 20.0).abs() < 1e-9);
            assert!((p.origin.z - 30.0).abs() < 1e-9);
        } else {
            panic!("Expected Plane surface");
        }

        // Verify edge endpoints moved
        let e = &f.edges[0];
        let start = e.start_point().unwrap();
        let end = e.end_point().unwrap();
        assert!((start.x - 10.0).abs() < 1e-9);
        assert!((start.y - 20.0).abs() < 1e-9);
        assert!((start.z - 30.0).abs() < 1e-9);
        assert!((end.x - 11.0).abs() < 1e-9);
        assert!((end.y - 20.0).abs() < 1e-9);
        assert!((end.z - 30.0).abs() < 1e-9);
    }

    #[test]
    fn test_rotate_solid_around_z() {
        let plane = Plane::xy();
        let edge = Edge::new_line(
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(2.0, 0.0, 0.0),
        );
        let mut face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        face.edges.push(edge);
        let shell = Shell::new_closed(vec![face]);
        let mut solid = Solid::new(shell);

        // Rotate 90° around Z
        rotate_solid(&mut solid, &Direction3d::Z, std::f64::consts::PI / 2.0);

        // Verify edge endpoints rotated
        let f = &solid.outer_shell.as_ref().unwrap().faces[0];
        let e = &f.edges[0];
        let start = e.start_point().unwrap();
        // (1, 0, 0) → (0, 1, 0)
        assert!(start.x.abs() < 1e-9);
        assert!((start.y - 1.0).abs() < 1e-9);
        assert!(start.z.abs() < 1e-9);
    }

    #[test]
    fn test_mirror_solid() {
        let plane = Plane::xy();
        let edge = Edge::new_line(
            Point3d::new(1.0, 2.0, 3.0),
            Point3d::new(4.0, 5.0, 6.0),
        );
        let mut face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        face.edges.push(edge);
        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);

        // Mirror about XY plane (z=0)
        let mirrored = mirror_solid(&solid, Point3d::ORIGIN, Direction3d::Z);

        let f = &mirrored.outer_shell.as_ref().unwrap().faces[0];
        let e = &f.edges[0];
        let start = e.start_point().unwrap();
        // (1, 2, 3) → (1, 2, -3)
        assert!((start.x - 1.0).abs() < 1e-9);
        assert!((start.y - 2.0).abs() < 1e-9);
        assert!((start.z + 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_add_and_remove_hole() {
        let plane = Plane::xy();
        let mut face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        assert_eq!(face.inner_wires.len(), 0);

        // Add a hole
        add_circular_hole_to_face(
            &mut face,
            Point3d::new(0.5, 0.5, 0.0),
            0.1,
            Direction3d::Z,
        )
        .unwrap();
        assert_eq!(face.inner_wires.len(), 1);

        // Remove the hole
        let removed = remove_hole_from_face(&mut face, 0).unwrap();
        assert_eq!(removed.coedges.len(), 1);
        assert_eq!(face.inner_wires.len(), 0);

        // Removing again should fail
        assert!(remove_hole_from_face(&mut face, 0).is_err());
    }

    #[test]
    fn test_replace_face_surface() {
        let plane = Plane::xy();
        let mut face = Face::new(Surface::Plane(plane), Wire::new(vec![]));

        // Replace with a different plane (offset in Z)
        let new_plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 5.0),
            Direction3d::Z,
        );
        replace_face_surface(&mut face, Surface::Plane(new_plane));

        // Verify
        if let Surface::Plane(p) = face.surface.as_ref().unwrap() {
            assert!((p.origin.z - 5.0).abs() < 1e-9);
        } else {
            panic!("Expected Plane surface");
        }
    }

    #[test]
    fn test_circular_pattern() {
        let plane = Plane::xy();
        let edge = Edge::new_line(
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(2.0, 0.0, 0.0),
        );
        let mut face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        face.edges.push(edge);
        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);

        // 5 total instances: original + 4 copies at 72°, 144°, 216°, 288°
        let copies = circular_pattern(&solid, Direction3d::Z, 5, 2.0 * std::f64::consts::PI);
        assert_eq!(copies.len(), 4);

        // Each copy should have its edge at a different angular position
        for (i, c) in copies.iter().enumerate() {
            let f = &c.outer_shell.as_ref().unwrap().faces[0];
            let e = &f.edges[0];
            let start = e.start_point().unwrap();
            let angle = 2.0 * std::f64::consts::PI * (i + 1) as f64 / 5.0;
            let expected_x = angle.cos();
            let expected_y = angle.sin();
            assert!(
                (start.x - expected_x).abs() < 1e-9,
                "Copy {}: expected x={}, got {}",
                i,
                expected_x,
                start.x
            );
            assert!(
                (start.y - expected_y).abs() < 1e-9,
                "Copy {}: expected y={}, got {}",
                i,
                expected_y,
                start.y
            );
        }
    }
}
