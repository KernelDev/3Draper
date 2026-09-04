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
///
/// C5 Stage 7.1: the solid's canonical `EdgeStore` curves are transformed
/// too (they are the ONLY payload of mirror-free/compacted faces), and the
/// store is re-indexed afterwards — born-indexed solids carry pre-transform
/// canonicals, and without the rebuild every store-first reader (mesh,
/// queries, exporters) would sample stale curves after a transform.
pub fn transform_solid(solid: &mut Solid, transform: &Transform) {
    if let Some(ref mut shell) = solid.outer_shell {
        transform_shell(shell, transform);
    }
    for shell in &mut solid.inner_shells {
        transform_shell(shell, transform);
    }
    // Transform the canonical store curves — the store is the ONLY holder
    // of edge geometry (C5 7.6b), so this IS the whole edge transform:
    // identity (ids, aliases, orientations) is unaffected, no re-index.
    solid.edge_store.transform_curves(transform);
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

    // C5 7.6b: edge curves live in the owning solid's `EdgeStore` —
    // transform them via `solid.edge_store.transform_curves` (see
    // `transform_solid`); a standalone face carries no edge geometry.

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
    solid: &mut Solid,
    face_index: usize,
    center_3d: Point3d,
    radius: f64,
    normal: Direction3d,
) -> Result<(), String> {
    if radius <= 0.0 {
        return Err(format!("Radius must be positive, got {}", radius));
    }
    let faces_len = solid.faces().len();
    if face_index >= faces_len {
        return Err(format!(
            "Face index {} out of range (solid has {} faces)",
            face_index, faces_len
        ));
    }

    // Build the 3D circle edge
    let circle = Circle {
        center: center_3d,
        normal,
        radius,
        x_axis: perpendicular_direction(&normal),
    };
    let edge_curve = Curve3d::Circle(circle.clone());
    let edge = Edge {
        id: TopoId::new(),
        curve: Some(edge_curve),
        param_range: (0.0, 2.0 * std::f64::consts::PI),
        vertex_start: Some(TopoId::new()),
        vertex_end: Some(TopoId::new()),
        start_vertex_point: None,
        end_vertex_point: None,
        forward: true,
        tolerance: 1e-6,
        degenerate: false,
        step_entity_id: None,
    };
    // C5 7.6b: the hole's boundary edge joins the store + the face's
    // canonical reference list.
    let hole_edge_id = edge.id;
    solid.edge_store.insert(edge);

    // Build the UV polyline (32 segments) by projecting 3D circle samples
    // onto the face's surface
    let face_surface = solid
        .faces()
        .get(face_index)
        .and_then(|f| f.surface.clone());
    let uv_polyline = if let Some(ref surface) = face_surface {
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
        edge: hole_edge_id,
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

    let faces_iter = solid.faces_mut();
    let face = faces_iter.into_iter().nth(face_index)
        .ok_or_else(|| format!("Face index {} vanished", face_index))?;
    face.add_hole(wire);
    face.edge_ids.push(hole_edge_id);
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
    solid: &mut Solid,
    edge_id: TopoId,
    new_curve: Curve3d,
    new_param_range: (f64, f64),
) -> Result<(), String> {
    // C5 7.6b: edges live in the solid's canonical store.
    let edge = solid
        .edge_store
        .get_mut(edge_id)
        .ok_or_else(|| format!("Edge {:?} not found in store", edge_id))?;
    edge.curve = Some(new_curve);
    edge.param_range = new_param_range;
    Ok(())
}

/// Reverse an edge's orientation (swap forward flag).
pub fn reverse_edge(solid: &mut Solid, edge_id: TopoId) -> Result<(), String> {
    // C5 7.6b: edges live in the solid's canonical store.
    let edge = solid
        .edge_store
        .get_mut(edge_id)
        .ok_or_else(|| format!("Edge {:?} not found in store", edge_id))?;
    edge.forward = !edge.forward;
    edge.param_range = (edge.param_range.1, edge.param_range.0);
    let tmp = edge.vertex_start;
    edge.vertex_start = edge.vertex_end;
    edge.vertex_end = tmp;
    Ok(())
}

// ═══════════════════════════════════════════════════════════════════════
// SECTION 6: Fillet / Chamfer / Shell — full implementations
// ═══════════════════════════════════════════════════════════════════════

/// Fillet (round) an edge of a solid by replacing the edge with a cylindrical
/// "tube" surface that tangentially connects the two adjacent faces.
///
/// # Algorithm
///
/// 1. Find the edge by `edge_index` in `solid.outer_shell.faces[*].edges`
///    (the first face containing that edge ID becomes face_a, the next
///    becomes face_b).
/// 2. Read the edge's curve, start/end points.
/// 3. Construct a Cylinder surface whose axis is the edge curve and whose
///    radius is `radius`. This is the fillet surface.
/// 4. Replace the edge in face_a with a new edge offset by `radius` along
///    face_a's surface normal, and similarly in face_b. Both new edges
///    are stored on the new fillet face.
/// 5. Insert the new fillet face into the shell.
///
/// # Limitations
///
/// - Currently supports **linear** edge curves (LINE). For curved edges
///   (CIRCLE, B_SPLINE_CURVE) the fillet surface would need to be a Torus
///   or sweep, which is more complex.
/// - Both adjacent faces must be **planes** so we can compute the offset
///   direction analytically. Cylindrical/conical adjacent faces are
///   detected and an error is returned.
/// - The radius must be small enough that the offset edges do not
///   cross each other or other edges of the face.
///
/// # Errors
///
/// Returns an error string when:
/// - `edge_index` does not match any edge ID in the solid.
/// - The edge curve is not a Line.
/// - Either adjacent face has no surface or a non-planar surface.
/// - The radius is ≤ 0.
pub fn fillet_edge(solid: &mut Solid, edge_index: usize, radius: f64) -> Result<(), String> {
    if radius <= 0.0 {
        return Err(format!("fillet radius must be > 0, got {}", radius));
    }
    if radius > 1e6 {
        return Err(format!("fillet radius is unreasonably large: {}", radius));
    }
    if solid.outer_shell.is_none() {
        return Err("solid has no outer shell".to_string());
    }

    let shell = solid.outer_shell.as_mut().unwrap();

    // C5 Stage 4 (fillet identity): snapshot the canonical alias map BEFORE
    // the mutable shell borrow. Both instances of a shared STEP edge carry
    // DIFFERENT instance TopoIds — matching `edge_index` against instance
    // ids alone found only one owner face and fillet rejected the edge
    // ("only 1 adjacent face"). Resolving through the store finds both.
    // Un-indexed / builder solids are unaffected (empty alias map).
    let aliases: std::collections::HashMap<TopoId, TopoId> =
        solid.edge_store.iter_aliases().collect();
    let resolve = |id: TopoId| aliases.get(&id).copied().unwrap_or(id);
    let wanted = resolve(TopoId::from_u64(edge_index as u64));

    // Find the edge by index across all faces. We need:
    // - the edge itself (curve, endpoints)
    // - the two adjacent faces (face_a, face_b)
    let mut edge_owner_faces: Vec<(usize, usize)> = Vec::new(); // (face_idx, edge_pos_in_edge_ids)
    for (fi, face) in shell.faces.iter().enumerate() {
        for (ei, eid) in face.edge_ids.iter().enumerate() {
            if eid.to_u64() as usize == edge_index
                || resolve(*eid) == wanted
            {
                edge_owner_faces.push((fi, ei));
            }
        }
    }

    if edge_owner_faces.is_empty() {
        return Err(format!("edge {} not found in any face", edge_index));
    }
    if edge_owner_faces.len() < 2 {
        return Err(format!(
            "edge {} has only {} adjacent face(s); fillet requires exactly 2 (no boundary edges)",
            edge_index, edge_owner_faces.len()
        ));
    }
    if edge_owner_faces.len() > 2 {
        return Err(format!(
            "edge {} has {} adjacent faces; non-manifold edge — fillet not supported",
            edge_index, edge_owner_faces.len()
        ));
    }

    let (face_a_idx, edge_a_pos) = edge_owner_faces[0];
    let (face_b_idx, edge_b_pos) = edge_owner_faces[1];

    // C5 7.6b: store-first edge geometry — the store's instance view
    // (alias-following, orientation-correct) is the source of truth.
    let eid_a = shell.faces[face_a_idx].edge_ids[edge_a_pos];
    let edge_a = solid
        .edge_store
        .instance_edge(eid_a)
        .ok_or_else(|| format!("edge {} not found in store", edge_index))?;
    let edge_curve = edge_a.curve.clone();
    let edge_curve = edge_curve.ok_or("edge has no curve")?;

    // Only LINE edges are supported in this implementation.
    let (edge_origin, edge_dir) = match edge_curve {
        Curve3d::Line(ref line) => (line.origin, line.direction),
        _ => {
            return Err(format!(
                "fillet_edge currently supports only LINE edges; got {:?} \
                 (curved-edge fillet would require a Torus/sweep surface)",
                edge_curve
            ));
        }
    };

    // Both adjacent faces must be planes.
    let plane_a = match shell.faces[face_a_idx].surface.as_ref() {
        Some(Surface::Plane(p)) => p.clone(),
        _ => return Err(format!(
            "fillet_edge currently supports planar adjacent faces; face {} is not a plane",
            face_a_idx
        )),
    };
    let plane_b = match shell.faces[face_b_idx].surface.as_ref() {
        Some(Surface::Plane(p)) => p.clone(),
        _ => return Err(format!(
            "fillet_edge currently supports planar adjacent faces; face {} is not a plane",
            face_b_idx
        )),
    };

    // Compute the edge length from param_range.
    let (t_min, t_max) = edge_a.param_range;
    let edge_length = (t_max - t_min).abs();
    if edge_length < 1e-12 {
        return Err("edge has zero length — cannot fillet".to_string());
    }

    // Build the fillet cylinder surface: axis = edge direction, origin = edge_origin,
    // radius = the fillet radius.
    let fillet_surface = Surface::Cylinder(draper_geometry::CylinderSurface::new_with_frame(
        edge_origin,
        edge_dir,
        radius,
        // x_dir = plane_a's normal (so u=0 is on plane_a, u=π is on plane_b)
        plane_a.normal,
    ));

    // Compute the offset edges on plane_a and plane_b.
    // The offset direction on plane_a is perpendicular to the edge, in the plane of plane_a.
    // That direction = plane_a.normal × edge_dir (then normalised).
    let offset_a_dir = cross_unit(&plane_a.normal, &edge_dir)
        .ok_or_else(|| "edge is parallel to plane_a normal — cannot compute offset".to_string())?;
    let offset_b_dir = cross_unit(&plane_b.normal, &edge_dir)
        .ok_or_else(|| "edge is parallel to plane_b normal — cannot compute offset".to_string())?;

    // The offset edges are parallel to the original edge, shifted by ±radius
    // along offset_a_dir / offset_b_dir. We pick the direction that points
    // "into" the fillet cylinder (toward the other face).
    //
    // Heuristic: the offset should move each face's edge toward the bisector
    // of the two face normals. The bisector direction in the plane perpendicular
    // to edge_dir is (offset_a_dir + offset_b_dir) / 2 if both normals point
    // "outward", or (offset_a_dir - offset_b_dir) / 2 if one is flipped.
    //
    // We compute both offsets and pick the pair whose midpoint is closest
    // to the original edge.
    let p_start = Point3d::new(
        edge_origin.x + t_min * edge_dir.x,
        edge_origin.y + t_min * edge_dir.y,
        edge_origin.z + t_min * edge_dir.z,
    );
    let p_end = Point3d::new(
        edge_origin.x + t_max * edge_dir.x,
        edge_origin.y + t_max * edge_dir.y,
        edge_origin.z + t_max * edge_dir.z,
    );

    let a_offset_start_plus = Point3d::new(
        p_start.x + radius * offset_a_dir.x,
        p_start.y + radius * offset_a_dir.y,
        p_start.z + radius * offset_a_dir.z,
    );
    let a_offset_start_minus = Point3d::new(
        p_start.x - radius * offset_a_dir.x,
        p_start.y - radius * offset_a_dir.y,
        p_start.z - radius * offset_a_dir.z,
    );
    let b_offset_start_plus = Point3d::new(
        p_start.x + radius * offset_b_dir.x,
        p_start.y + radius * offset_b_dir.y,
        p_start.z + radius * offset_b_dir.z,
    );
    let b_offset_start_minus = Point3d::new(
        p_start.x - radius * offset_b_dir.x,
        p_start.y - radius * offset_b_dir.y,
        p_start.z - radius * offset_b_dir.z,
    );

    // Choose the offset pair that minimises |a_offset - b_offset| (they
    // should meet at the same point on the fillet cylinder's seam).
    let d_pp = (a_offset_start_plus.x - b_offset_start_plus.x).powi(2)
        + (a_offset_start_plus.y - b_offset_start_plus.y).powi(2)
        + (a_offset_start_plus.z - b_offset_start_plus.z).powi(2);
    let d_pm = (a_offset_start_plus.x - b_offset_start_minus.x).powi(2)
        + (a_offset_start_plus.y - b_offset_start_minus.y).powi(2)
        + (a_offset_start_plus.z - b_offset_start_minus.z).powi(2);
    let d_mp = (a_offset_start_minus.x - b_offset_start_plus.x).powi(2)
        + (a_offset_start_minus.y - b_offset_start_plus.y).powi(2)
        + (a_offset_start_minus.z - b_offset_start_plus.z).powi(2);
    let d_mm = (a_offset_start_minus.x - b_offset_start_minus.x).powi(2)
        + (a_offset_start_minus.y - b_offset_start_minus.y).powi(2)
        + (a_offset_start_minus.z - b_offset_start_minus.z).powi(2);

    let (a_sign, b_sign) = if d_pp <= d_pm && d_pp <= d_mp && d_pp <= d_mm {
        (1.0_f64, 1.0_f64)
    } else if d_pm <= d_pp && d_pm <= d_mp && d_pm <= d_mm {
        (1.0, -1.0)
    } else if d_mp <= d_pp && d_mp <= d_pm && d_mp <= d_mm {
        (-1.0, 1.0)
    } else {
        (-1.0, -1.0)
    };

    let a_offset_start = Point3d::new(
        p_start.x + a_sign * radius * offset_a_dir.x,
        p_start.y + a_sign * radius * offset_a_dir.y,
        p_start.z + a_sign * radius * offset_a_dir.z,
    );
    let a_offset_end = Point3d::new(
        p_end.x + a_sign * radius * offset_a_dir.x,
        p_end.y + a_sign * radius * offset_a_dir.y,
        p_end.z + a_sign * radius * offset_a_dir.z,
    );
    let b_offset_start = Point3d::new(
        p_start.x + b_sign * radius * offset_b_dir.x,
        p_start.y + b_sign * radius * offset_b_dir.y,
        p_start.z + b_sign * radius * offset_b_dir.z,
    );
    let b_offset_end = Point3d::new(
        p_end.x + b_sign * radius * offset_b_dir.x,
        p_end.y + b_sign * radius * offset_b_dir.y,
        p_end.z + b_sign * radius * offset_b_dir.z,
    );

    // Build the new offset edges on each adjacent face.
    let new_edge_a = Edge::new_line(a_offset_start, a_offset_end);
    let new_edge_b = Edge::new_line(b_offset_start, b_offset_end);

    // C5 7.6b: replace the old edge reference on each face — the new
    // offset edges join the store, the wire coedges follow the new ids,
    // and the old canonical leaves the store.
    let old_id_a = shell.faces[face_a_idx].edge_ids[edge_a_pos];
    let old_id_b = shell.faces[face_b_idx].edge_ids[edge_b_pos];
    // Capture the offset-edge ids BEFORE the store takes ownership — they
    // occupy the fillet face's edge_ids slots below.
    let fillet_slot_a = new_edge_a.id;
    let fillet_slot_b = new_edge_b.id;
    let _ = (old_id_a, old_id_b);
    replace_face_edge(shell, face_a_idx, edge_a_pos, &new_edge_a);
    replace_face_edge(shell, face_b_idx, edge_b_pos, &new_edge_b);
    solid.edge_store.insert(new_edge_a);
    solid.edge_store.insert(new_edge_b);
    solid.edge_store.remove(old_id_a);
    solid.edge_store.remove(old_id_b);

    // Build the fillet face: a cylindrical face bounded by the two offset
    // edges (wire-less — the boundary lives in the store + edge_ids).
    let mut fillet_face = Face::new_surface_only(fillet_surface);
    // Edges run along the cylinder axis (constant u). The two offset edges
    // correspond to u=0 (on plane_a) and u=π (on plane_b), each running
    // from start to end along the axis.
    // Add two cap edges (degenerate points at the start and end) so the
    // face is topologically closed. These caps have zero length.
    // C5 7.6b fix: ALL FOUR boundary edges must occupy edge_ids slots
    // (the offsets ARE the fillet cylinder's boundary) — dropping them
    // starved the wire-less boundary readers (v-range detection).
    let cap_start = Edge::new_line(a_offset_start, b_offset_start);
    let cap_end = Edge::new_line(a_offset_end, b_offset_end);
    fillet_face.edge_ids = vec![fillet_slot_a, fillet_slot_b, cap_start.id, cap_end.id];
    fillet_face.forward = true;
    solid.edge_store.insert(cap_start);
    solid.edge_store.insert(cap_end);

    // Add the fillet face to the shell.
    shell.faces.push(fillet_face);

    // C5 7.6b: the store was mutated in place (replaced canonicals, new
    // fillet edges) — no re-index pass exists or is needed.

    Ok(())
}

/// Chamfer an edge of a solid by replacing the edge with a beveled planar
/// face that connects the two adjacent faces at a fixed distance.
///
/// # Algorithm
///
/// 1. Find the edge by `edge_index` (same as fillet_edge).
/// 2. Read the edge curve and endpoints.
/// 3. Compute the offset edges on both adjacent faces at distance
///    `distance` from the original edge.
/// 4. Build a planar face through the four offset points (a rectangle
///    if the edge is straight and the two faces are perpendicular).
/// 5. Replace the original edge in each adjacent face with the offset edge.
/// 6. Insert the chamfer face into the shell.
///
/// # Limitations
///
/// Same as `fillet_edge`: only LINE edges and planar adjacent faces are
/// supported.
pub fn chamfer_edge(solid: &mut Solid, edge_index: usize, distance: f64) -> Result<(), String> {
    if distance <= 0.0 {
        return Err(format!("chamfer distance must be > 0, got {}", distance));
    }
    if distance > 1e6 {
        return Err(format!("chamfer distance is unreasonably large: {}", distance));
    }
    if solid.outer_shell.is_none() {
        return Err("solid has no outer shell".to_string());
    }

    let shell = solid.outer_shell.as_mut().unwrap();

    // C5 Stage 4 (chamfer identity): see fillet_edge — resolve the numeric
    // edge id through the store's alias map so both instances of a shared
    // STEP edge are found.
    let aliases: std::collections::HashMap<TopoId, TopoId> =
        solid.edge_store.iter_aliases().collect();
    let resolve = |id: TopoId| aliases.get(&id).copied().unwrap_or(id);
    let wanted = resolve(TopoId::from_u64(edge_index as u64));

    // Find the edge by index across all faces (canonical references).
    let mut edge_owner_faces: Vec<(usize, usize)> = Vec::new();
    for (fi, face) in shell.faces.iter().enumerate() {
        for (ei, eid) in face.edge_ids.iter().enumerate() {
            if eid.to_u64() as usize == edge_index
                || resolve(*eid) == wanted
            {
                edge_owner_faces.push((fi, ei));
            }
        }
    }

    if edge_owner_faces.is_empty() {
        return Err(format!("edge {} not found in any face", edge_index));
    }
    if edge_owner_faces.len() < 2 {
        return Err(format!(
            "edge {} has only {} adjacent face(s); chamfer requires exactly 2",
            edge_index, edge_owner_faces.len()
        ));
    }
    if edge_owner_faces.len() > 2 {
        return Err(format!(
            "edge {} has {} adjacent faces; non-manifold — chamfer not supported",
            edge_index, edge_owner_faces.len()
        ));
    }

    let (face_a_idx, edge_a_pos) = edge_owner_faces[0];
    let (face_b_idx, edge_b_pos) = edge_owner_faces[1];

    // C5 7.6b: store-first edge geometry (see fillet_edge).
    let eid_a = shell.faces[face_a_idx].edge_ids[edge_a_pos];
    let edge_a = solid
        .edge_store
        .instance_edge(eid_a)
        .ok_or_else(|| format!("edge {} not found in store", edge_index))?;
    let edge_curve = edge_a.curve.clone();
    let edge_curve = edge_curve.ok_or("edge has no curve")?;

    let (edge_origin, edge_dir) = match edge_curve {
        Curve3d::Line(ref line) => (line.origin, line.direction),
        _ => {
            return Err(format!(
                "chamfer_edge currently supports only LINE edges; got {:?}",
                edge_curve
            ));
        }
    };

    let plane_a = match shell.faces[face_a_idx].surface.as_ref() {
        Some(Surface::Plane(p)) => p.clone(),
        _ => return Err(format!(
            "chamfer_edge requires planar adjacent faces; face {} is not a plane",
            face_a_idx
        )),
    };
    let plane_b = match shell.faces[face_b_idx].surface.as_ref() {
        Some(Surface::Plane(p)) => p.clone(),
        _ => return Err(format!(
            "chamfer_edge requires planar adjacent faces; face {} is not a plane",
            face_b_idx
        )),
    };

    let (t_min, t_max) = edge_a.param_range;
    let edge_length = (t_max - t_min).abs();
    if edge_length < 1e-12 {
        return Err("edge has zero length — cannot chamfer".to_string());
    }

    // Offset directions on each face.
    let offset_a_dir = cross_unit(&plane_a.normal, &edge_dir)
        .ok_or_else(|| "edge is parallel to plane_a normal".to_string())?;
    let offset_b_dir = cross_unit(&plane_b.normal, &edge_dir)
        .ok_or_else(|| "edge is parallel to plane_b normal".to_string())?;

    let p_start = Point3d::new(
        edge_origin.x + t_min * edge_dir.x,
        edge_origin.y + t_min * edge_dir.y,
        edge_origin.z + t_min * edge_dir.z,
    );
    let p_end = Point3d::new(
        edge_origin.x + t_max * edge_dir.x,
        edge_origin.y + t_max * edge_dir.y,
        edge_origin.z + t_max * edge_dir.z,
    );

    // Choose the offset direction that points "into" the chamfer (toward
    // the other face). Same heuristic as fillet_edge.
    let a_plus_start = Point3d::new(
        p_start.x + distance * offset_a_dir.x,
        p_start.y + distance * offset_a_dir.y,
        p_start.z + distance * offset_a_dir.z,
    );
    let a_minus_start = Point3d::new(
        p_start.x - distance * offset_a_dir.x,
        p_start.y - distance * offset_a_dir.y,
        p_start.z - distance * offset_a_dir.z,
    );
    let b_plus_start = Point3d::new(
        p_start.x + distance * offset_b_dir.x,
        p_start.y + distance * offset_b_dir.y,
        p_start.z + distance * offset_b_dir.z,
    );
    let b_minus_start = Point3d::new(
        p_start.x - distance * offset_b_dir.x,
        p_start.y - distance * offset_b_dir.y,
        p_start.z - distance * offset_b_dir.z,
    );

    let d_pp = (a_plus_start.x - b_plus_start.x).powi(2)
        + (a_plus_start.y - b_plus_start.y).powi(2)
        + (a_plus_start.z - b_plus_start.z).powi(2);
    let d_pm = (a_plus_start.x - b_minus_start.x).powi(2)
        + (a_plus_start.y - b_minus_start.y).powi(2)
        + (a_plus_start.z - b_minus_start.z).powi(2);
    let d_mp = (a_minus_start.x - b_plus_start.x).powi(2)
        + (a_minus_start.y - b_plus_start.y).powi(2)
        + (a_minus_start.z - b_plus_start.z).powi(2);
    let d_mm = (a_minus_start.x - b_minus_start.x).powi(2)
        + (a_minus_start.y - b_minus_start.y).powi(2)
        + (a_minus_start.z - b_minus_start.z).powi(2);

    let (a_sign, b_sign) = if d_pp <= d_pm && d_pp <= d_mp && d_pp <= d_mm {
        (1.0_f64, 1.0_f64)
    } else if d_pm <= d_pp && d_pm <= d_mp && d_pm <= d_mm {
        (1.0, -1.0)
    } else if d_mp <= d_pp && d_mp <= d_pm && d_mp <= d_mm {
        (-1.0, 1.0)
    } else {
        (-1.0, -1.0)
    };

    let a_offset_start = Point3d::new(
        p_start.x + a_sign * distance * offset_a_dir.x,
        p_start.y + a_sign * distance * offset_a_dir.y,
        p_start.z + a_sign * distance * offset_a_dir.z,
    );
    let a_offset_end = Point3d::new(
        p_end.x + a_sign * distance * offset_a_dir.x,
        p_end.y + a_sign * distance * offset_a_dir.y,
        p_end.z + a_sign * distance * offset_a_dir.z,
    );
    let b_offset_start = Point3d::new(
        p_start.x + b_sign * distance * offset_b_dir.x,
        p_start.y + b_sign * distance * offset_b_dir.y,
        p_start.z + b_sign * distance * offset_b_dir.z,
    );
    let b_offset_end = Point3d::new(
        p_end.x + b_sign * distance * offset_b_dir.x,
        p_end.y + b_sign * distance * offset_b_dir.y,
        p_end.z + b_sign * distance * offset_b_dir.z,
    );

    let new_edge_a = Edge::new_line(a_offset_start, a_offset_end);
    let new_edge_b = Edge::new_line(b_offset_start, b_offset_end);

    // C5 7.6b: store-first replacement (see fillet_edge).
    let old_id_a = shell.faces[face_a_idx].edge_ids[edge_a_pos];
    let old_id_b = shell.faces[face_b_idx].edge_ids[edge_b_pos];
    // Capture the offset-edge ids BEFORE the store takes ownership.
    let chamfer_slot_a = new_edge_a.id;
    let chamfer_slot_b = new_edge_b.id;
    let _ = (old_id_a, old_id_b);
    replace_face_edge(shell, face_a_idx, edge_a_pos, &new_edge_a);
    replace_face_edge(shell, face_b_idx, edge_b_pos, &new_edge_b);
    solid.edge_store.insert(new_edge_a);
    solid.edge_store.insert(new_edge_b);
    solid.edge_store.remove(old_id_a);
    solid.edge_store.remove(old_id_b);

    // The chamfer face is a plane through the four offset points.
    // Compute the plane normal as (edge_dir × (a_offset_start - b_offset_start)).
    let diag = Vec3d::new(
        a_offset_start.x - b_offset_start.x,
        a_offset_start.y - b_offset_start.y,
        a_offset_start.z - b_offset_start.z,
    );
    let normal_vec = Vec3d::new(
        edge_dir.y * diag.z - edge_dir.z * diag.y,
        edge_dir.z * diag.x - edge_dir.x * diag.z,
        edge_dir.x * diag.y - edge_dir.y * diag.x,
    );
    let normal_len = (normal_vec.x * normal_vec.x
        + normal_vec.y * normal_vec.y
        + normal_vec.z * normal_vec.z)
        .sqrt();
    if normal_len < 1e-12 {
        return Err("chamfer face is degenerate (edge_dir parallel to offset_diag)".to_string());
    }
    let normal = Direction3d::new(
        normal_vec.x / normal_len,
        normal_vec.y / normal_len,
        normal_vec.z / normal_len,
    )
    .ok_or("chamfer face normal is zero")?;

    let chamfer_plane = draper_geometry::Plane {
        origin: a_offset_start,
        u_dir: edge_dir,
        v_dir: cross_unit(&edge_dir, &normal).unwrap_or(Direction3d::Y),
        normal,
    };
    let chamfer_surface = Surface::Plane(chamfer_plane);

    let mut chamfer_face = Face::new_surface_only(chamfer_surface);
    // C5 7.6b fix: ALL FOUR boundary edges occupy edge_ids slots (the
    // offsets ARE the chamfer plane's boundary), matching the legacy
    // mirror list [new_edge_a, new_edge_b, cap_start, cap_end].
    let cap_start = Edge::new_line(a_offset_start, b_offset_start);
    let cap_end = Edge::new_line(a_offset_end, b_offset_end);
    chamfer_face.edge_ids = vec![
        chamfer_slot_a,
        chamfer_slot_b,
        cap_start.id,
        cap_end.id,
    ];
    chamfer_face.forward = true;
    solid.edge_store.insert(cap_start);
    solid.edge_store.insert(cap_end);

    shell.faces.push(chamfer_face);

    // C5 7.6b: the store was mutated in place — no re-index pass.

    Ok(())
}

/// Create a shell (hollow solid) by offsetting all faces inward by `thickness`.
///
/// # Algorithm
///
/// 1. For each face in `solid.outer_shell`, translate the surface geometry
///    along its outward normal by `-thickness` (inward).
/// 2. The edges are shared between faces; we do not move them in this
///    simplified implementation (a full offset would also rebuild edges
///    and add "side" faces connecting the original and offset boundaries).
/// 3. The original outer faces become the OUTER shell of the hollow solid.
///    A new INNER shell is created with the offset (inward-shifted) faces.
///
/// # Limitations
///
/// This is a simplified shell: it does not handle concave edges, varying
/// thickness, or self-intersections that can arise when the offset
/// distance exceeds the local curvature radius. For a full B-rep offset
/// algorithm, a thorough surface-surface intersection and edge-rebuild
/// pass would be needed (planned for a future P21 task).
///
/// # Errors
///
/// Returns an error string when:
/// - `thickness` ≤ 0.
/// - The solid has no outer shell.
/// - Any face has no surface (cannot compute offset direction).
pub fn make_shell(solid: &mut Solid, thickness: f64) -> Result<(), String> {
    if thickness <= 0.0 {
        return Err(format!("shell thickness must be > 0, got {}", thickness));
    }
    if thickness > 1e6 {
        return Err(format!("shell thickness is unreasoningly large: {}", thickness));
    }
    if solid.outer_shell.is_none() {
        return Err("solid has no outer shell".to_string());
    }

    // Clone the outer shell, then offset each face inward.
    let mut inner_shell = solid.outer_shell.as_ref().unwrap().clone();
    inner_shell.id = TopoId::new();
    inner_shell.closed = true;

    for face in &mut inner_shell.faces {
        let surface = face.surface.take().ok_or("face has no surface")?;
        let offset_surface = offset_surface_inward(&surface, thickness)?;
        face.surface = Some(offset_surface);
        face.id = TopoId::new();

        // C5 7.6b: offset the face's boundary edges through the store —
        // each resolved instance is offset, re-keyed to a fresh id, inserted
        // into the store, and referenced by the face's `edge_ids` (the
        // original outer-shell references stay untouched).
        let orig_ids = std::mem::take(&mut face.edge_ids);
        for oid in orig_ids {
            if let Some(mut edge) = solid.edge_store.instance_edge(oid) {
                if let Some(curve) = edge.curve.take() {
                    let offset_curve = offset_curve_along_normal(
                        &curve,
                        &surface,
                        thickness,
                    );
                    edge.curve = Some(offset_curve);
                }
                edge.id = TopoId::new();
                face.edge_ids.push(edge.id);
                solid.edge_store.insert(edge);
            }
        }
    }

    // The inner shell's faces have been moved inward. The original outer
    // shell remains as-is. Together they form a hollow solid.
    solid.inner_shells.push(inner_shell);

    Ok(())
}

/// Offset a surface inward (along its outward normal) by `distance`.
/// Returns the offset surface, or an error if the surface type is not
/// supported.
fn offset_surface_inward(
    surface: &Surface,
    distance: f64,
) -> Result<Surface, String> {
    match surface {
        Surface::Plane(p) => {
            // Plane offset = same plane shifted along normal by -distance.
            let new_origin = Point3d::new(
                p.origin.x - distance * p.normal.x,
                p.origin.y - distance * p.normal.y,
                p.origin.z - distance * p.normal.z,
            );
            Ok(Surface::Plane(draper_geometry::Plane {
                origin: new_origin,
                u_dir: p.u_dir,
                v_dir: p.v_dir,
                normal: p.normal,
            }))
        }
        Surface::Cylinder(c) => {
            // Cylinder offset = change radius. Inward = smaller radius.
            let new_radius = (c.radius - distance).max(0.001 * c.radius);
            Ok(Surface::Cylinder(draper_geometry::CylinderSurface::new_with_frame(
                c.origin,
                c.axis,
                new_radius,
                c.x_dir,
            )))
        }
        Surface::Sphere(s) => {
            // Sphere offset = change radius. Inward = smaller radius.
            let new_radius = (s.radius - distance).max(0.001 * s.radius);
            Ok(Surface::Sphere(draper_geometry::SphereSurface {
                center: s.center,
                radius: new_radius,
            }))
        }
        Surface::Cone(c) => {
            // Cone offset = change radius (at origin). half_angle stays the same.
            let new_radius = (c.radius - distance).max(0.001 * c.radius);
            Ok(Surface::Cone(draper_geometry::ConeSurface {
                origin: c.origin,
                axis: c.axis,
                half_angle: c.half_angle,
                radius: new_radius,
                x_dir: c.x_dir,
                expanding: c.expanding,
            }))
        }
        Surface::Torus(t) => {
            // Torus offset = change minor radius (inward = smaller minor).
            let new_minor = (t.minor_radius - distance).max(0.001 * t.minor_radius);
            Ok(Surface::Torus(draper_geometry::TorusSurface {
                center: t.center,
                axis: t.axis,
                major_radius: t.major_radius,
                minor_radius: new_minor,
                x_dir: t.x_dir,
            }))
        }
        _ => Err(format!(
            "offset_surface_inward does not yet support this surface type"
        )),
    }
}

/// Offset a curve along the normal of a surface by `distance`. Used by
/// `make_shell` to offset edge curves consistently with their face.
fn offset_curve_along_normal(
    curve: &Curve3d,
    surface: &Surface,
    distance: f64,
) -> Curve3d {
    // Get the surface normal direction (approximate — use a representative
    // point on the curve projected to the surface).
    let t_mid = {
        let (t0, t1) = curve.param_range();
        0.5 * (t0 + t1)
    };
    let p_mid = curve.point_at(t_mid);

    let normal = match surface {
        Surface::Plane(p) => Some(p.normal),
        Surface::Cylinder(c) => {
            // Radial direction from axis to point.
            let dx = p_mid.x - c.origin.x;
            let dy = p_mid.y - c.origin.y;
            let dz = p_mid.z - c.origin.z;
            let proj = dx * c.axis.x + dy * c.axis.y + dz * c.axis.z;
            let perp_x = dx - proj * c.axis.x;
            let perp_y = dy - proj * c.axis.y;
            let perp_z = dz - proj * c.axis.z;
            let len = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
            if len > 1e-12 {
                Direction3d::new(perp_x / len, perp_y / len, perp_z / len)
            } else {
                None
            }
        }
        Surface::Sphere(s) => {
            let dx = p_mid.x - s.center.x;
            let dy = p_mid.y - s.center.y;
            let dz = p_mid.z - s.center.z;
            let len = (dx * dx + dy * dy + dz * dz).sqrt();
            if len > 1e-12 {
                Direction3d::new(dx / len, dy / len, dz / len)
            } else {
                None
            }
        }
        _ => None,
    };

    let normal = match normal {
        Some(n) => n,
        None => {
            // Cannot compute offset direction; return the curve unchanged.
            return curve.clone();
        }
    };

    // Apply the offset to the curve geometry.
    match curve {
        Curve3d::Line(line) => {
            let new_origin = Point3d::new(
                line.origin.x - distance * normal.x,
                line.origin.y - distance * normal.y,
                line.origin.z - distance * normal.z,
            );
            Curve3d::Line(draper_geometry::Line::new(new_origin, line.direction))
        }
        Curve3d::Circle(c) => {
            let new_center = Point3d::new(
                c.center.x - distance * normal.x,
                c.center.y - distance * normal.y,
                c.center.z - distance * normal.z,
            );
            let new_radius = (c.radius - distance).max(0.001 * c.radius);
            Curve3d::Circle(Circle {
                center: new_center,
                normal: c.normal,
                radius: new_radius,
                x_axis: c.x_axis,
            })
        }
        // For other curve types, return as-is (offset approximation would
        // require curve re-parameterisation).
        _ => curve.clone(),
    }
}

/// Cross product of two Direction3d, returning a normalised Direction3d.
/// Returns None if the inputs are parallel.
fn cross_unit(a: &Direction3d, b: &Direction3d) -> Option<Direction3d> {
    let cx = a.y * b.z - a.z * b.y;
    let cy = a.z * b.x - a.x * b.z;
    let cz = a.x * b.y - a.y * b.x;
    let len = (cx * cx + cy * cy + cz * cz).sqrt();
    if len < 1e-12 {
        return None;
    }
    Direction3d::new(cx / len, cy / len, cz / len)
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
        Surface::Offset(_) | Surface::Ruled(_) => {
            project_point_to_surface_grid(surface, point, -100.0, 100.0, -100.0, 100.0)
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
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        // (7.6b: the boundary edge rides the solid's store.)
        let mut solid =
            Solid::from_edges_only(Shell::new_closed(vec![face]), vec![vec![edge]]);

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

        // Verify edge endpoints moved (store-resolved instance).
        let resolved = solid.resolve_face_edges(f);
        let e = &resolved[0];
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
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        // (7.6b: the boundary edge rides the solid's store.)
        let mut solid =
            Solid::from_edges_only(Shell::new_closed(vec![face]), vec![vec![edge]]);

        // Rotate 90° around Z
        rotate_solid(&mut solid, &Direction3d::Z, std::f64::consts::PI / 2.0);

        // Verify edge endpoints rotated
        let f = &solid.outer_shell.as_ref().unwrap().faces[0];
        let resolved = solid.resolve_face_edges(f);
        let e = &resolved[0];
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
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        // (7.6b: the boundary edge rides the solid's store.)
        let solid =
            Solid::from_edges_only(Shell::new_closed(vec![face]), vec![vec![edge]]);

        // Mirror about XY plane (z=0)
        let mirrored = mirror_solid(&solid, Point3d::ORIGIN, Direction3d::Z);

        let f = &mirrored.outer_shell.as_ref().unwrap().faces[0];
        let resolved = mirrored.resolve_face_edges(f);
        let e = &resolved[0];
        let start = e.start_point().unwrap();
        // (1, 2, 3) → (1, 2, -3)
        assert!((start.x - 1.0).abs() < 1e-9);
        assert!((start.y - 2.0).abs() < 1e-9);
        assert!((start.z + 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_add_and_remove_hole() {
        let plane = Plane::xy();
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        // (7.6b: hole surgery takes the SOLID + face index — the hole edge
        // joins the store; the face keeps topology only.)
        let mut solid =
            Solid::from_edges_only(Shell::new_closed(vec![face]), vec![Vec::new()]);
        assert_eq!(solid.faces()[0].inner_wires.len(), 0);

        // Add a hole
        add_circular_hole_to_face(
            &mut solid,
            0,
            Point3d::new(0.5, 0.5, 0.0),
            0.1,
            Direction3d::Z,
        )
        .unwrap();
        assert_eq!(solid.faces()[0].inner_wires.len(), 1);

        // Remove the hole
        let mut faces = solid.faces_mut();
        let removed = remove_hole_from_face(faces[0], 0).unwrap();
        drop(faces);
        assert_eq!(removed.coedges.len(), 1);
        assert_eq!(solid.faces()[0].inner_wires.len(), 0);

        // Removing again should fail
        let mut faces = solid.faces_mut();
        assert!(remove_hole_from_face(faces[0], 0).is_err());
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
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        // (7.6b: the boundary edge rides the solid's store.)
        let solid =
            Solid::from_edges_only(Shell::new_closed(vec![face]), vec![vec![edge]]);

        // 5 total instances: original + 4 copies at 72°, 144°, 216°, 288°
        let copies = circular_pattern(&solid, Direction3d::Z, 5, 2.0 * std::f64::consts::PI);
        assert_eq!(copies.len(), 4);

        // Each copy should have its edge at a different angular position
        for (i, c) in copies.iter().enumerate() {
            let f = &c.outer_shell.as_ref().unwrap().faces[0];
            let resolved = c.resolve_face_edges(f);
            let e = &resolved[0];
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

    /// Build a unit cube (1×1×1) with 6 planar faces and 12 linear edges.
    /// Edges are SHARED between adjacent faces (same TopoId), so fillet
    /// and chamfer operations can find the two adjacent faces for each
    /// manifold edge.
    fn unit_cube() -> Solid {
        // First, create all 12 unique edges with stable TopoIds. We'll
        // clone them into each face's edges list so that the TopoIds
        // match across faces.
        let e_bottom_01 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0));
        let e_bottom_12 = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 0.0));
        let e_bottom_23 = Edge::new_line(Point3d::new(1.0, 1.0, 0.0), Point3d::new(0.0, 1.0, 0.0));
        let e_bottom_30 = Edge::new_line(Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 0.0, 0.0));
        let e_top_01 = Edge::new_line(Point3d::new(0.0, 0.0, 1.0), Point3d::new(1.0, 0.0, 1.0));
        let e_top_12 = Edge::new_line(Point3d::new(1.0, 0.0, 1.0), Point3d::new(1.0, 1.0, 1.0));
        let e_top_23 = Edge::new_line(Point3d::new(1.0, 1.0, 1.0), Point3d::new(0.0, 1.0, 1.0));
        let e_top_30 = Edge::new_line(Point3d::new(0.0, 1.0, 1.0), Point3d::new(0.0, 0.0, 1.0));
        let e_vert_0 = Edge::new_line(Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 1.0));
        let e_vert_1 = Edge::new_line(Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 1.0));
        let e_vert_2 = Edge::new_line(Point3d::new(1.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 1.0));
        let e_vert_3 = Edge::new_line(Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 1.0, 1.0));

        // Bottom face (z=0): edges 0,1,2,3 of bottom
        let bottom = Face::new_surface_only(Surface::Plane(Plane {
            origin: Point3d::new(0.0, 0.0, 0.0),
            u_dir: Direction3d::X,
            v_dir: Direction3d::Y,
            normal: Direction3d::new(0.0, 0.0, -1.0).unwrap(),
        }));
        let bottom_w: Vec<Edge> = vec![
            e_bottom_01.clone(),
            e_bottom_12.clone(),
            e_bottom_23.clone(),
            e_bottom_30.clone(),
        ];

        // Top face (z=1): edges 0,1,2,3 of top
        let top = Face::new_surface_only(Surface::Plane(Plane {
            origin: Point3d::new(0.0, 0.0, 1.0),
            u_dir: Direction3d::X,
            v_dir: Direction3d::Y,
            normal: Direction3d::Z,
        }));
        let top_w: Vec<Edge> = vec![
            e_top_01.clone(),
            e_top_12.clone(),
            e_top_23.clone(),
            e_top_30.clone(),
        ];

        // Front face (y=0): bottom_01 (shared), vert_1, top_01 (shared), vert_0
        let front = Face::new_surface_only(Surface::Plane(Plane {
            origin: Point3d::new(0.0, 0.0, 0.0),
            u_dir: Direction3d::X,
            v_dir: Direction3d::Z,
            normal: Direction3d::new(0.0, -1.0, 0.0).unwrap(),
        }));
        let front_w: Vec<Edge> = vec![
            e_bottom_01.clone(),
            e_vert_1.clone(),
            e_top_01.clone(),
            e_vert_0.clone(),
        ];

        // Back face (y=1): bottom_23, vert_3, top_23, vert_2
        let back = Face::new_surface_only(Surface::Plane(Plane {
            origin: Point3d::new(0.0, 1.0, 0.0),
            u_dir: Direction3d::X,
            v_dir: Direction3d::Z,
            normal: Direction3d::Y,
        }));
        let back_w: Vec<Edge> = vec![
            e_bottom_23.clone(),
            e_vert_3.clone(),
            e_top_23.clone(),
            e_vert_2.clone(),
        ];

        // Left face (x=0): bottom_30, vert_3, top_30, vert_0
        let left = Face::new_surface_only(Surface::Plane(Plane {
            origin: Point3d::new(0.0, 0.0, 0.0),
            u_dir: Direction3d::Y,
            v_dir: Direction3d::Z,
            normal: Direction3d::new(-1.0, 0.0, 0.0).unwrap(),
        }));
        let left_w: Vec<Edge> = vec![
            e_bottom_30.clone(),
            e_vert_3.clone(),
            e_top_30.clone(),
            e_vert_0.clone(),
        ];

        // Right face (x=1): bottom_12, vert_1, top_12, vert_2
        let right = Face::new_surface_only(Surface::Plane(Plane {
            origin: Point3d::new(1.0, 0.0, 0.0),
            u_dir: Direction3d::Y,
            v_dir: Direction3d::Z,
            normal: Direction3d::X,
        }));
        let right_w: Vec<Edge> = vec![
            e_bottom_12.clone(),
            e_vert_1.clone(),
            e_top_12.clone(),
            e_vert_2.clone(),
        ];

        let shell = Shell::new_closed(vec![bottom, top, front, back, left, right]);
        // 7.6b: born-indexed — the shared edge instances (same TopoIds
        // across faces) ride the working lists into the store.
        Solid::from_edges_only(
            shell,
            vec![bottom_w, top_w, front_w, back_w, left_w, right_w],
        )
    }

    #[test]
    fn test_fillet_edge_on_unit_cube() {
        let mut cube = unit_cube();
        // Pick the first edge of the first face (store-resolved instance).
        let f0 = &cube.outer_shell.as_ref().unwrap().faces[0];
        let edge_id = cube.resolve_face_edges(f0)[0].id.to_u64() as usize;

        // Apply fillet with radius 0.1
        let result = fillet_edge(&mut cube, edge_id, 0.1);
        assert!(result.is_ok(), "fillet_edge failed: {:?}", result);

        // After fillet: 6 original faces + 1 new fillet face = 7 faces.
        assert_eq!(
            cube.outer_shell.as_ref().unwrap().faces.len(),
            7,
            "expected 7 faces after fillet (6 original + 1 fillet)"
        );

        // The fillet face should be a Cylinder surface.
        let fillet_face = &cube.outer_shell.as_ref().unwrap().faces[6];
        match fillet_face.surface.as_ref().unwrap() {
            Surface::Cylinder(_) => {}
            other => panic!("expected Cylinder fillet surface, got {:?}", other),
        }
        // The fillet face should have 4 boundary slots (2 offset + 2
        // caps). 7.6b: `edge_ids` keeps the per-slot canonical references
        // (cap slots may share canonicals with the adjacent faces' split
        // edges — geometric dedup), so the SLOT count is the contract.
        assert_eq!(fillet_face.edge_ids.len(), 4);
    }

    #[test]
    fn test_fillet_edge_on_step_style_shared_edge() {
        // C5 Stage 4 → 7.6b: STEP-imported solids unify a shared edge under
        // one canonical store entry (same step_entity_id in both incident
        // faces' working lists). Fillet used to fail with "only 1 adjacent
        // face(s)" when the scan matched per-face instance ids only; the
        // store-first scan finds both incident faces through the canonical.
        let mut cube = unit_cube();
        // The vertical edge e_vert_0 is shared between front (face 2,
        // slot 3) and left (face 4, slot 3).
        let f_left = &cube.outer_shell.as_ref().unwrap().faces[4];
        let shared_id = cube.resolve_face_edges(f_left)[3].id;

        // STEP-ify: stamp the shared canonical with a STEP entity id —
        // both incident faces see it through the store.
        if let Some(e) = cube.edge_store.get_mut(shared_id) {
            e.step_entity_id = Some(700);
        }

        // Fillet by the shared (canonical) id — both incident faces must
        // resolve through the store.
        let result = fillet_edge(&mut cube, shared_id.to_u64() as usize, 0.1);
        assert!(
            result.is_ok(),
            "fillet on STEP-style shared edge failed: {:?}",
            result
        );
        // 6 original + 1 fillet face.
        assert_eq!(cube.outer_shell.as_ref().unwrap().faces.len(), 7);
    }

    #[test]
    fn test_chamfer_edge_on_unit_cube() {
        let mut cube = unit_cube();
        let f0 = &cube.outer_shell.as_ref().unwrap().faces[0];
        let edge_id = cube.resolve_face_edges(f0)[0].id.to_u64() as usize;

        let result = chamfer_edge(&mut cube, edge_id, 0.1);
        assert!(result.is_ok(), "chamfer_edge failed: {:?}", result);

        // 6 original + 1 chamfer face = 7
        assert_eq!(
            cube.outer_shell.as_ref().unwrap().faces.len(),
            7,
            "expected 7 faces after chamfer"
        );

        // The chamfer face should be a Plane.
        let chamfer_face = &cube.outer_shell.as_ref().unwrap().faces[6];
        match chamfer_face.surface.as_ref().unwrap() {
            Surface::Plane(_) => {}
            other => panic!("expected Plane chamfer surface, got {:?}", other),
        }
        assert_eq!(chamfer_face.edge_ids.len(), 4);
    }

    #[test]
    fn test_fillet_edge_invalid_radius() {
        let mut cube = unit_cube();
        let result = fillet_edge(&mut cube, 0, 0.0);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("radius"));

        let result = fillet_edge(&mut cube, 0, -1.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_chamfer_edge_invalid_distance() {
        let mut cube = unit_cube();
        let result = chamfer_edge(&mut cube, 0, 0.0);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("distance"));
    }

    #[test]
    fn test_fillet_edge_not_found() {
        let mut cube = unit_cube();
        // Edge ID 99999 does not exist.
        let result = fillet_edge(&mut cube, 99999, 0.1);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("not found"));
    }

    #[test]
    fn test_make_shell_on_unit_cube() {
        let mut cube = unit_cube();
        let result = make_shell(&mut cube, 0.1);
        assert!(result.is_ok(), "make_shell failed: {:?}", result);

        // The cube should now have an outer shell + 1 inner shell.
        assert_eq!(cube.inner_shells.len(), 1, "expected 1 inner shell");

        // The inner shell should have the same number of faces as the outer.
        let inner = &cube.inner_shells[0];
        let outer = cube.outer_shell.as_ref().unwrap();
        assert_eq!(inner.faces.len(), outer.faces.len());

        // The inner shell's first face should be a Plane offset by -0.1
        // along its normal.
        let inner_face = &inner.faces[0];
        if let Surface::Plane(p) = inner_face.surface.as_ref().unwrap() {
            // The offset distance along the normal should be 0.1.
            let outer_face = &outer.faces[0];
            if let Surface::Plane(op) = outer_face.surface.as_ref().unwrap() {
                let dx = p.origin.x - op.origin.x;
                let dy = p.origin.y - op.origin.y;
                let dz = p.origin.z - op.origin.z;
                let offset = (dx * dx + dy * dy + dz * dz).sqrt();
                assert!(
                    (offset - 0.1).abs() < 1e-9,
                    "expected offset 0.1, got {}",
                    offset
                );
            }
        } else {
            panic!("expected Plane surface on inner face");
        }
    }

    #[test]
    fn test_make_shell_invalid_thickness() {
        let mut cube = unit_cube();
        let result = make_shell(&mut cube, 0.0);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("thickness"));

        let result = make_shell(&mut cube, -1.0);
        assert!(result.is_err());
    }
}

/// C5 7.6b helper: replace the `edge_ids[pos]` entry of face `fi` with the
/// new edge's id, updating the face's wire coedges that referenced the old
/// id (the store insert/removal is the caller's job — disjoint borrows).
fn replace_face_edge(shell: &mut Shell, fi: usize, pos: usize, new_edge: &Edge) {
    let old_id = shell.faces[fi].edge_ids[pos];
    let new_id = new_edge.id;
    shell.faces[fi].edge_ids[pos] = new_id;
    for wire in shell.faces[fi].inner_wires.iter_mut() {
        for coedge in wire.coedges.iter_mut() {
            if coedge.edge == old_id {
                coedge.edge = new_id;
            }
        }
    }
    if let Some(ref mut wire) = shell.faces[fi].outer_wire {
        for coedge in wire.coedges.iter_mut() {
            if coedge.edge == old_id {
                coedge.edge = new_id;
            }
        }
    }
}
