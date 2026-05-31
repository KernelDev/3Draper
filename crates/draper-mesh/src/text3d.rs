// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 3D text mesh generation for "3Draper" branding.
//!
//! Generates 3D extruded text as a `TriangleMesh` using vector glyph outlines.
//! Each glyph is defined as a set of closed polygon contours (stroke paths).
//! The resulting mesh has front faces, back faces, and side walls.
//!
//! Also provides mesh-level text hole cutting: actual holes in the shape of
//! "3Draper" text are cut through primitive surfaces.

use crate::mesh::TriangleMesh;
use draper_geometry::Point3d;

/// A single glyph defined by its closed polygon contours.
struct GlyphDef {
    /// Closed polygon contours. Each contour is a list of (x, y) points
    /// forming a closed loop (last point connects to first).
    contours: Vec<Vec<(f64, f64)>>,
    /// Width of the glyph in font units.
    width: f64,
}

/// Vector glyph definitions for "3Draper" characters.
/// Each glyph is defined at roughly 10x14 unit size, centered horizontally.
fn get_glyph(ch: char) -> Option<GlyphDef> {
    match ch {
        '3' => Some(GlyphDef {
            width: 10.0,
            contours: vec![
                // Outer contour of '3'
                vec![
                    (1.0, 13.0), (7.0, 13.0), (9.0, 12.0), (9.5, 10.5),
                    (9.0, 9.0), (7.5, 8.0), (5.5, 7.5),
                    (7.5, 7.0), (9.5, 5.5), (10.0, 3.5),
                    (9.5, 1.5), (7.5, 0.0), (4.5, -0.5),
                    (1.5, 0.0), (0.0, 1.5), (0.0, 3.0),
                    (1.0, 4.0), (2.5, 3.5), (2.0, 2.0),
                    (3.5, 1.0), (5.5, 1.5), (6.5, 3.0),
                    (6.0, 5.0), (4.5, 6.0), (3.0, 6.0),
                    (3.0, 7.5), (4.5, 7.5), (6.0, 8.0),
                    (6.5, 9.5), (5.5, 11.0), (3.5, 11.5),
                    (2.0, 10.5), (2.5, 9.0), (1.0, 8.5),
                    (0.0, 9.5), (0.5, 12.0), (2.5, 13.5),
                ],
            ],
        }),
        'D' => Some(GlyphDef {
            width: 11.0,
            contours: vec![
                // Outer contour of 'D'
                vec![
                    (1.0, 13.0), (1.0, 0.0), (5.0, 0.0),
                    (8.0, 0.5), (10.0, 2.5), (10.5, 5.0),
                    (10.0, 9.0), (8.0, 11.5), (5.0, 13.0),
                ],
                // Inner hole of 'D'
                vec![
                    (3.0, 11.0), (5.0, 11.0), (7.0, 9.5),
                    (7.5, 7.0), (7.5, 5.5), (7.0, 3.5),
                    (5.0, 2.0), (3.0, 2.0),
                ],
            ],
        }),
        'r' => Some(GlyphDef {
            width: 7.0,
            contours: vec![
                vec![
                    (1.0, 5.5), (1.0, 0.0), (3.0, 0.0), (3.0, 5.0),
                    (4.0, 6.0), (5.5, 5.5), (6.0, 4.0), (6.5, 5.0),
                    (5.5, 7.0), (3.5, 7.0), (2.5, 6.0), (2.5, 9.0),
                    (1.0, 9.0),
                ],
            ],
        }),
        'a' => Some(GlyphDef {
            width: 9.0,
            contours: vec![
                // Outer contour of 'a'
                vec![
                    (7.5, 0.0), (7.5, 1.0), (6.0, 0.0), (4.0, -0.5),
                    (2.0, 0.0), (0.5, 1.5), (0.5, 3.5),
                    (2.0, 5.0), (5.0, 5.5), (7.0, 5.5), (7.0, 7.0),
                    (5.5, 8.0), (3.5, 7.5), (2.5, 6.0), (1.0, 6.5),
                    (2.5, 8.5), (5.0, 9.5), (7.5, 9.0), (9.0, 7.0),
                    (9.0, 0.0),
                ],
                // Inner hole
                vec![
                    (2.5, 3.0), (3.0, 1.5), (5.0, 1.0),
                    (7.0, 2.0), (7.0, 4.0), (4.5, 4.0),
                ],
            ],
        }),
        'p' => Some(GlyphDef {
            width: 9.0,
            contours: vec![
                // Outer contour
                vec![
                    (1.0, 9.0), (1.0, -4.0), (3.0, -4.0), (3.0, 1.0),
                    (4.5, 0.0), (6.5, 0.0), (8.0, 1.5), (8.5, 4.0),
                    (8.0, 7.0), (6.0, 9.0), (4.0, 9.0), (2.5, 8.0),
                    (2.5, 9.0),
                ],
                // Inner hole
                vec![
                    (3.0, 2.0), (4.5, 1.5), (6.0, 2.5),
                    (6.5, 4.5), (6.0, 6.5), (4.5, 7.5),
                    (3.0, 6.5),
                ],
            ],
        }),
        'e' => Some(GlyphDef {
            width: 9.0,
            contours: vec![
                vec![
                    (8.0, 2.5), (6.5, 0.5), (4.5, -0.5),
                    (2.0, 0.0), (0.5, 2.0), (0.0, 4.5),
                    (0.5, 7.0), (2.5, 9.0), (5.0, 9.5),
                    (7.5, 8.5), (8.5, 6.0), (7.0, 5.5),
                    (6.0, 7.5), (4.5, 8.0), (2.5, 7.0),
                    (2.0, 5.0), (8.5, 5.0),
                ],
                // Counter (hole) is the upper part
                vec![
                    (2.0, 4.0), (2.5, 2.0), (4.0, 1.0),
                    (6.0, 1.5), (7.0, 3.0), (7.0, 4.0),
                ],
            ],
        }),
        _ => None,
    }
}

/// Generate 2D text contour polygons for the given text string.
///
/// Returns a list of closed polygons, each being a Vec of (x, y) points.
/// The first polygon of each glyph is the outer contour; subsequent polygons
/// are holes (counter-clockwise). For hole cutting, we treat all contours
/// as regions to cut away.
///
/// # Arguments
/// * `text` - Text string (typically "3Draper")
/// * `scale` - Scale factor for glyph outlines
/// * `spacing` - Extra spacing between characters
pub fn generate_text_contours(text: &str, scale: f64, spacing: f64) -> Vec<Vec<(f64, f64)>> {
    let mut contours = Vec::new();
    let mut cursor_x = 0.0_f64;

    for ch in text.chars() {
        if let Some(glyph) = get_glyph(ch) {
            let offset_x = cursor_x;
            for contour in &glyph.contours {
                let scaled: Vec<(f64, f64)> = contour
                    .iter()
                    .map(|&(x, y)| (offset_x + x * scale, y * scale))
                    .collect();
                if scaled.len() >= 3 {
                    contours.push(scaled);
                }
            }
            cursor_x += glyph.width * scale + spacing;
        } else if ch == ' ' {
            cursor_x += 6.0 * scale + spacing;
        }
    }

    // Center all contours at origin
    if !contours.is_empty() {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        for contour in &contours {
            for &(x, y) in contour {
                min_x = min_x.min(x);
                min_y = min_y.min(y);
                max_y = max_y.max(y);
                max_x = max_x.max(x);
            }
        }
        let cx = (min_x + max_x) / 2.0;
        let cy = (min_y + max_y) / 2.0;
        for contour in &mut contours {
            for pt in contour.iter_mut() {
                pt.0 -= cx;
                pt.1 -= cy;
            }
        }
    }

    contours
}

/// Ray-casting point-in-polygon test for a 2D polygon.
///
/// Returns true if the point (px, py) is inside the polygon defined by `vertices`.
fn point_in_polygon_2d(px: f64, py: f64, vertices: &[(f64, f64)]) -> bool {
    let n = vertices.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = vertices[i];
        let (xj, yj) = vertices[j];
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Check if a 2D point is inside any of the text contour polygons.
fn point_in_any_contour(px: f64, py: f64, contours: &[(Vec<(f64, f64)>, f64)]) -> bool {
    for (contour, _winding) in contours {
        if point_in_polygon_2d(px, py, contour) {
            return true;
        }
    }
    false
}

/// Compute signed area (shoelace formula) to determine winding order.
/// Positive = counter-clockwise, negative = clockwise.
fn signed_area_2d(polygon: &[(f64, f64)]) -> f64 {
    let n = polygon.len();
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += polygon[i].0 * polygon[j].1;
        area -= polygon[j].0 * polygon[i].1;
    }
    area / 2.0
}

/// Surface type for projecting text contours onto.
#[derive(Clone, Debug)]
pub enum TextSurface {
    /// Flat plane: text is placed on the XY plane at given z height.
    /// Text coordinates: (x, y) in 2D → (x, y, z) in 3D
    Plane { z: f64 },
    /// Sphere: text is mapped onto the sphere surface.
    /// Text coordinates: (x, y) → spherical angles → 3D point on sphere
    Sphere { center: [f64; 3], radius: f64 },
    /// Cylinder along Z axis: text is unrolled onto the lateral surface.
    /// Text x → angle around axis, text y → height along axis
    Cylinder { radius: f64, height: f64 },
    /// Cone along Z axis: text is mapped onto the lateral surface.
    Cone { radius: f64, height: f64 },
    /// Torus: text is mapped onto the outer surface.
    Torus { major_radius: f64, minor_radius: f64 },
}

/// Result of projecting a 2D text contour point onto a 3D surface.
#[derive(Clone, Debug)]
struct SurfacePoint {
    /// 3D position on the surface.
    pos: Point3d,
    /// Surface normal at this point.
    normal: [f64; 3],
}

/// Project a 2D point from text space onto a 3D surface.
fn project_2d_to_surface(x: f64, y: f64, surface: &TextSurface) -> SurfacePoint {
    match surface {
        TextSurface::Plane { z } => SurfacePoint {
            pos: Point3d::new(x, y, *z),
            normal: [0.0, 0.0, 1.0],
        },
        TextSurface::Sphere { center, radius } => {
            // Map text (x, y) to spherical coordinates
            // x maps to azimuthal angle (longitude), y maps to polar angle (latitude)
            let r = *radius;
            // Scale text to cover a reasonable portion of the sphere
            let scale_angle = 1.0 / r; // radians per unit
            let theta = y * scale_angle; // polar angle from equator
            let phi = x * scale_angle;   // azimuthal angle

            let cos_t = theta.cos();
            let sin_t = theta.sin();
            let cos_p = phi.cos();
            let sin_p = phi.sin();

            let px = center[0] + r * cos_t * cos_p;
            let py = center[1] + r * cos_t * sin_p;
            let pz = center[2] + r * sin_t;

            // Normal points outward from center
            let nx = cos_t * cos_p;
            let ny = cos_t * sin_p;
            let nz = sin_t;

            SurfacePoint {
                pos: Point3d::new(px, py, pz),
                normal: [nx, ny, nz],
            }
        }
        TextSurface::Cylinder { radius, height } => {
            // Map text x to angle around cylinder, y to height
            let r = *radius;
            let h = *height;
            let scale_angle = 1.0 / r;
            let angle = x * scale_angle;

            let px = r * angle.cos();
            let py = r * angle.sin();
            let pz = y + h / 2.0; // center vertically

            // Normal points radially outward
            let nx = angle.cos();
            let ny = angle.sin();
            let nz = 0.0;

            SurfacePoint {
                pos: Point3d::new(px, py, pz),
                normal: [nx, ny, nz],
            }
        }
        TextSurface::Cone { radius, height } => {
            // Map text x to angle around cone, y to height
            let r = *radius;
            let h = *height;
            let scale_angle = 1.2 / r; // slightly wider for cone
            let angle = x * scale_angle;
            let z = y + h / 2.0;
            let local_r = r * (1.0 - z / h); // radius decreases with height

            let px = local_r * angle.cos();
            let py = local_r * angle.sin();
            let pz = z;

            // Cone normal (slightly tilted outward)
            let half_angle = (r / h).atan();
            let sin_ha = half_angle.sin();
            let cos_ha = half_angle.cos();
            let nx = angle.cos() * cos_ha;
            let ny = angle.sin() * cos_ha;
            let nz = sin_ha;

            SurfacePoint {
                pos: Point3d::new(px, py, pz),
                normal: [nx, ny, nz],
            }
        }
        TextSurface::Torus { major_radius, minor_radius } => {
            // Map text x to major angle (around the ring), y to minor angle (around the tube)
            let R = *major_radius;
            let r = *minor_radius;
            let scale_major = 1.0 / R;
            let scale_minor = 1.0 / r;
            let u = x * scale_major; // major angle
            let v = y * scale_minor; // minor angle

            let cos_u = u.cos();
            let sin_u = u.sin();
            let cos_v = v.cos();
            let sin_v = v.sin();

            let px = (R + r * cos_v) * cos_u;
            let py = (R + r * cos_v) * sin_u;
            let pz = r * sin_v;

            // Normal points outward from the tube center
            let nx = cos_v * cos_u;
            let ny = cos_v * sin_u;
            let nz = sin_v;

            SurfacePoint {
                pos: Point3d::new(px, py, pz),
                normal: [nx, ny, nz],
            }
        }
    }
}

/// Project a 3D point back to 2D text space (inverse of project_2d_to_surface).
/// Used to test if a mesh vertex is inside a text contour.
fn project_3d_to_2d(point: Point3d, surface: &TextSurface) -> (f64, f64) {
    match surface {
        TextSurface::Plane { z: _ } => {
            (point.x, point.y)
        }
        TextSurface::Sphere { center, radius } => {
            let dx = point.x - center[0];
            let dy = point.y - center[1];
            let dz = point.z - center[2];
            let r = *radius;
            // Inverse of: theta = y/r, phi = x/r
            // theta (polar from equator) = asin(dz/r)
            // phi (azimuthal) = atan2(dy, dx)
            let theta = (dz / r).asin();
            let phi = dy.atan2(dx);
            (phi * r, theta * r)
        }
        TextSurface::Cylinder { radius, height } => {
            let r = *radius;
            let h = *height;
            // Inverse of: angle = x/r, pz = y + h/2
            let angle = point.y.atan2(point.x);
            let pz = point.z;
            (angle * r, pz - h / 2.0)
        }
        TextSurface::Cone { radius, height } => {
            let r = *radius;
            let h = *height;
            let scale_angle = 1.2 / r;
            let angle = point.y.atan2(point.x);
            let z = point.z;
            (angle / scale_angle, z - h / 2.0)
        }
        TextSurface::Torus { major_radius, minor_radius } => {
            let major_r = *major_radius;
            let minor_r = *minor_radius;
            // Inverse of: u = x * scale_major, v = y * scale_minor
            let u = point.y.atan2(point.x); // major angle
            let cos_u = u.cos();
            let sin_u = u.sin();
            // Project point onto ring center to find minor angle
            let ring_x = point.x - major_r * cos_u;
            let ring_y = point.y - major_r * sin_u;
            let ring_z = point.z;
            let radial_dist = ring_x * cos_u + ring_y * sin_u;
            let v = ring_z.atan2(radial_dist.max(0.0));
            (u * major_r, v * minor_r)
        }
    }
}

/// Cut "3Draper" text-shaped holes in a mesh, projected onto a surface.
///
/// This removes triangles whose centroids fall inside the text contour polygons,
/// and adds new geometry along the contour boundaries to create clean holes
/// with depth (inset surfaces).
///
/// # Arguments
/// * `base_mesh` - The base primitive mesh to cut holes into
/// * `text` - Text string to cut (typically "3Draper")
/// * `surface` - Surface type for projection
/// * `scale` - Scale factor for text size
/// * `depth` - Depth of the cut holes (inset distance along surface normals)
/// * `hole_color` - RGBA color for the inset (hole bottom) surfaces
pub fn cut_text_holes_in_mesh(
    base_mesh: &TriangleMesh,
    text: &str,
    surface: &TextSurface,
    scale: f64,
    depth: f64,
    hole_color: [f32; 4],
) -> TriangleMesh {
    let spacing = 1.5 * scale;
    let contours_2d = generate_text_contours(text, scale, spacing);

    if contours_2d.is_empty() || base_mesh.triangle_count() == 0 {
        let mut result = base_mesh.clone();
        result.ensure_colors([0.48, 0.52, 0.58, 1.0]);
        return result;
    }

    // Identify triangles to remove: those whose centroid projects inside text contours.
    // A point is "inside text" if it's inside an odd number of contours
    // (ray-casting parity test). This naturally handles letter holes (D, a, p, e)
    // because a point in a hole is inside 2 contours (outer + hole) → even → not cut.
    let mut triangles_to_remove = vec![false; base_mesh.triangles.len()];

    for (i, tri) in base_mesh.triangles.iter().enumerate() {
        let v0 = base_mesh.vertices[tri[0] as usize];
        let v1 = base_mesh.vertices[tri[1] as usize];
        let v2 = base_mesh.vertices[tri[2] as usize];

        // Centroid
        let cx = (v0.x + v1.x + v2.x) / 3.0;
        let cy = (v0.y + v1.y + v2.y) / 3.0;
        let cz = (v0.z + v1.z + v2.z) / 3.0;

        // Project centroid to 2D text space
        let (px, py) = project_3d_to_2d(Point3d::new(cx, cy, cz), surface);

        // Count how many contours the point is inside (parity test)
        let mut inside_count = 0;
        for contour in &contours_2d {
            if point_in_polygon_2d(px, py, contour) {
                inside_count += 1;
            }
        }

        // Odd count → inside the text region → remove this triangle
        if inside_count % 2 == 1 {
            triangles_to_remove[i] = true;
        }
    }

    // Build result mesh
    let mut result = TriangleMesh::new();
    let base_color = [0.48, 0.52, 0.58, 1.0];

    // Keep triangles that are NOT inside the text
    let mut num_base_tris = 0usize;
    for (i, tri) in base_mesh.triangles.iter().enumerate() {
        if !triangles_to_remove[i] {
            let base = result.vertices.len() as u32;
            for &idx in tri {
                result.vertices.push(base_mesh.vertices[idx as usize].clone());
            }
            result.triangles.push([base, base + 1, base + 2]);
            num_base_tris += 1;
        }
    }

    // For each contour, project its 2D points onto the 3D surface and create:
    // 1. Surface ring vertices on the surface at the contour boundary
    // 2. Inset ring vertices pushed inward along surface normal by `depth`
    // 3. Side walls connecting surface ring to inset ring
    // 4. Inset face (fan triangulation) for the bottom of the hole
    for contour_2d in &contours_2d {
        if contour_2d.len() < 3 {
            continue;
        }

        let n = contour_2d.len();

        // Project contour points onto surface → surface ring
        let surface_ring_start = result.vertices.len() as u32;
        for &(x, y) in contour_2d {
            let sp = project_2d_to_surface(x, y, surface);
            result.vertices.push(sp.pos);
        }

        // Create inset ring (pushed along normal by depth)
        let inset_ring_start = result.vertices.len() as u32;
        for &(x, y) in contour_2d {
            let sp = project_2d_to_surface(x, y, surface);
            let inset_pos = Point3d::new(
                sp.pos.x - sp.normal[0] * depth,
                sp.pos.y - sp.normal[1] * depth,
                sp.pos.z - sp.normal[2] * depth,
            );
            result.vertices.push(inset_pos);
        }

        // Side walls: connect surface ring to inset ring
        for i in 0..n as u32 {
            let j = (i + 1) % n as u32;
            let si = surface_ring_start + i;
            let sj = surface_ring_start + j;
            let ii = inset_ring_start + i;
            let ij = inset_ring_start + j;

            // Quad: si → sj → ij → ii
            // Triangle 1: si, ii, sj
            result.triangles.push([si, ii, sj]);
            // Triangle 2: sj, ii, ij
            result.triangles.push([sj, ii, ij]);
        }

        // Inset face (bottom of hole) — fan triangulation from first vertex
        for i in 1..n as u32 - 1 {
            result.triangles.push([
                inset_ring_start,
                inset_ring_start + i + 1,
                inset_ring_start + i,
            ]);
        }
    }

    // Set up per-triangle colors
    // Base triangles get the default color, hole triangles get the hole color
    let total_tris = result.triangles.len();
    let hole_tris = total_tris - num_base_tris;
    let mut colors = Vec::with_capacity(total_tris);
    for _ in 0..num_base_tris {
        colors.push(base_color);
    }
    for _ in 0..hole_tris {
        colors.push(hole_color);
    }
    result.triangle_colors = Some(colors);

    if result.vertex_count() > 0 {
        result.compute_face_normals();
    }

    result
}

/// Generate a 3D extruded text mesh for the string "3Draper".
///
/// The text is placed in the XY plane and extruded along the Z axis.
/// Returns a `TriangleMesh` with the extruded text geometry.
///
/// # Arguments
/// * `text` - The text string to render (typically "3Draper")
/// * `height` - Height of the extrusion along Z
/// * `scale` - Scale factor for the glyph outlines
/// * `spacing` - Extra spacing between characters
pub fn generate_text_mesh(text: &str, height: f64, scale: f64, spacing: f64) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let mut cursor_x = 0.0_f64;

    for ch in text.chars() {
        if let Some(glyph) = get_glyph(ch) {
            let offset_x = cursor_x;
            for contour in &glyph.contours {
                extrude_contour(&mut mesh, contour, offset_x, height, scale);
            }
            cursor_x += glyph.width * scale + spacing;
        } else if ch == ' ' {
            cursor_x += 6.0 * scale + spacing;
        }
    }

    if mesh.vertex_count() > 0 {
        mesh.compute_face_normals();
    }
    mesh
}

/// Extrude a single closed contour into 3D.
///
/// Creates:
/// - Front face (z = 0)
/// - Back face (z = height)
/// - Side walls connecting front and back
fn extrude_contour(
    mesh: &mut TriangleMesh,
    contour: &[(f64, f64)],
    offset_x: f64,
    height: f64,
    scale: f64,
) {
    if contour.len() < 3 {
        return;
    }

    // Create front face vertices (z = 0)
    let front_start = mesh.vertices.len() as u32;
    for &(x, y) in contour {
        mesh.vertices.push(Point3d::new(
            offset_x + x * scale,
            y * scale,
            0.0,
        ));
    }

    // Create back face vertices (z = height)
    let back_start = mesh.vertices.len() as u32;
    for &(x, y) in contour {
        mesh.vertices.push(Point3d::new(
            offset_x + x * scale,
            y * scale,
            height,
        ));
    }

    let n = contour.len() as u32;

    // Triangulate front face using fan triangulation
    // For convex contours, this works perfectly.
    // For concave contours, this is approximate but visually acceptable.
    for i in 1..n - 1 {
        mesh.triangles.push([front_start, front_start + i, front_start + i + 1]);
    }

    // Triangulate back face (reversed winding)
    for i in 1..n - 1 {
        mesh.triangles.push([back_start + i + 1, back_start + i, back_start]);
    }

    // Create side walls
    for i in 0..n {
        let j = (i + 1) % n;
        let fi = front_start + i;
        let fj = front_start + j;
        let bi = back_start + i;
        let bj = back_start + j;

        // Two triangles per side quad
        mesh.triangles.push([fi, bi, fj]);
        mesh.triangles.push([fj, bi, bj]);
    }
}

/// Create a "3Draper" text mesh centered at origin, placed on the XY plane.
///
/// The text is scaled and positioned so that it can be easily placed on surfaces.
pub fn generate_3draper_text(depth: f64, scale: f64) -> TriangleMesh {
    let text = "3Draper";
    let spacing = 1.5 * scale;
    let mut mesh = generate_text_mesh(text, depth, scale, spacing);

    // Center the mesh at origin
    if mesh.vertex_count() > 0 {
        let (bmin, bmax) = mesh.bounding_box();
        let cx = (bmin.x + bmax.x) / 2.0;
        let cy = (bmin.y + bmax.y) / 2.0;
        let cz = (bmin.z + bmax.z) / 2.0;
        for v in &mut mesh.vertices {
            v.x -= cx;
            v.y -= cy;
            v.z -= cz;
        }
    }

    mesh
}

/// Carve text into a surface by creating the text as a colored overlay mesh.
///
/// This creates a text mesh positioned on a surface with a slight offset
/// and a different color (darker/recessed appearance).
pub fn carve_text_on_surface(
    text_mesh: &TriangleMesh,
    surface_center: Point3d,
    surface_normal: [f64; 3],
    text_scale: f64,
) -> TriangleMesh {
    let mut result = text_mesh.clone();

    // Scale the text
    for v in &mut result.vertices {
        v.x *= text_scale;
        v.y *= text_scale;
        v.z *= text_scale;
    }

    // Simple placement: offset text to surface position along normal
    let offset = 0.1; // Small offset above surface
    for v in &mut result.vertices {
        v.x += surface_center.x + surface_normal[0] * offset;
        v.y += surface_center.y + surface_normal[1] * offset;
        v.z += surface_center.z + surface_normal[2] * offset;
    }

    result
}
