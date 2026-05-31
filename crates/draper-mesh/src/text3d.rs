// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 3D text mesh generation for "3Draper" branding.
//!
//! Generates 3D extruded text as a `TriangleMesh` using vector glyph outlines.
//! Each glyph is defined as a set of closed polygon contours (stroke paths).
//! The resulting mesh has front faces, back faces, and side walls.
//!
//! Also provides text hole cutting: actual holes in the shape of
//! "3Draper" text are cut through primitive surfaces using ear-clipping
//! with bridge-edge hole insertion (guaranteed to terminate, unlike CDT).

use crate::mesh::TriangleMesh;
// Note: ParametricDomain/CDT imports removed — we use ear-clipping instead
// which is guaranteed to terminate (unlike spade CDT which can hang).
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

/// Simplify a polygon contour by removing points that are nearly collinear.
///
/// Uses the Ramer-Douglas-Peucker algorithm to reduce the number of points
/// while preserving the overall shape. This is essential for CDT performance —
/// fewer constraint edges means faster triangulation and fewer opportunities
/// for spade's CDT to encounter degenerate configurations.
///
/// # Arguments
/// * `points` - Input polygon points
/// * `epsilon` - Maximum allowed deviation from the original contour
fn simplify_contour(points: &[(f64, f64)], epsilon: f64) -> Vec<(f64, f64)> {
    if points.len() <= 3 {
        return points.to_vec();
    }

    // Find the point with the maximum distance from the line between first and last
    let first = points[0];
    let last = points[points.len() - 1];
    let mut max_dist = 0.0_f64;
    let mut max_idx = 0;

    for (i, p) in points.iter().enumerate().skip(1).take(points.len() - 2) {
        let dist = point_to_line_dist(*p, first, last);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    if max_dist > epsilon {
        // Recursive simplification
        let left = simplify_contour(&points[..=max_idx], epsilon);
        let right = simplify_contour(&points[max_idx..], epsilon);
        // Merge, avoiding duplicate at junction
        let mut result = left;
        result.extend_from_slice(&right[1..]);
        result
    } else {
        // All intermediate points are close to the line
        vec![first, last]
    }
}

/// Compute the perpendicular distance from a point to a line segment.
fn point_to_line_dist(point: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-20 {
        let px = point.0 - a.0;
        let py = point.1 - a.1;
        return (px * px + py * py).sqrt();
    }
    let t = ((point.0 - a.0) * dx + (point.1 - a.1) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let proj_x = a.0 + t * dx;
    let proj_y = a.1 + t * dy;
    let px = point.0 - proj_x;
    let py = point.1 - proj_y;
    (px * px + py * py).sqrt()
}

/// Generate 2D text contour polygons for the given text string.
///
/// Returns a list of closed polygons, each being a Vec of (x, y) points.
/// The first polygon of each glyph is the outer contour; subsequent polygons
/// are holes (counter-clockwise). For hole cutting, we treat all contours
/// as regions to cut away.
///
/// The contours are simplified using Ramer-Douglas-Peucker to reduce
/// the number of constraint edges in the CDT, which prevents spade's
/// constrained Delaunay triangulation from hanging on complex inputs.
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
                    // Simplify the contour to reduce ear-clipping work.
                    // Use a very conservative epsilon — too aggressive simplification
                    // can create self-intersecting polygons or lose shape features.
                    // For a single character like '3', we want ~12-18 points per contour
                    // to preserve the shape's curves without being too coarse.
                    let epsilon = scale * 0.2;
                    let simplified = simplify_contour(&scaled, epsilon);
                    // Ensure we still have at least 3 points after simplification
                    if simplified.len() >= 3 {
                        contours.push(simplified);
                    } else {
                        contours.push(scaled);
                    }
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

/// Compute signed area (shoelace formula) to determine winding order.
/// Positive = counter-clockwise, negative = clockwise.
#[allow(dead_code)]
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
            let r = *radius;
            let scale_angle = 1.0 / r;
            let theta = y * scale_angle;
            let phi = x * scale_angle;

            let cos_t = theta.cos();
            let sin_t = theta.sin();
            let cos_p = phi.cos();
            let sin_p = phi.sin();

            let px = center[0] + r * cos_t * cos_p;
            let py = center[1] + r * cos_t * sin_p;
            let pz = center[2] + r * sin_t;

            let nx = cos_t * cos_p;
            let ny = cos_t * sin_p;
            let nz = sin_t;

            SurfacePoint {
                pos: Point3d::new(px, py, pz),
                normal: [nx, ny, nz],
            }
        }
        TextSurface::Cylinder { radius, height } => {
            let r = *radius;
            let h = *height;
            let scale_angle = 1.0 / r;
            let angle = x * scale_angle;

            let px = r * angle.cos();
            let py = r * angle.sin();
            let pz = y + h / 2.0;

            let nx = angle.cos();
            let ny = angle.sin();
            let nz = 0.0;

            SurfacePoint {
                pos: Point3d::new(px, py, pz),
                normal: [nx, ny, nz],
            }
        }
        TextSurface::Cone { radius, height } => {
            let r = *radius;
            let h = *height;
            let scale_angle = 1.2 / r;
            let angle = x * scale_angle;
            let z = y + h / 2.0;
            let local_r = r * (1.0 - z / h);

            let px = local_r * angle.cos();
            let py = local_r * angle.sin();
            let pz = z;

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
            let R = *major_radius;
            let r = *minor_radius;
            let scale_major = 1.0 / R;
            let scale_minor = 1.0 / r;
            let u = x * scale_major;
            let v = y * scale_minor;

            let cos_u = u.cos();
            let sin_u = u.sin();
            let cos_v = v.cos();
            let sin_v = v.sin();

            let px = (R + r * cos_v) * cos_u;
            let py = (R + r * cos_v) * sin_u;
            let pz = r * sin_v;

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

/// Cut text-shaped holes in a mesh using a simple, reliable approach.
///
/// This function creates holes in the shape of text characters through a surface:
/// 1. Projects each triangle centroid from 3D back to 2D text coordinates
/// 2. Removes triangles whose centroids fall inside any hole contour
/// 3. Creates inset surfaces for each hole with proper depth
/// 4. Creates side walls connecting the surface boundary to the inset
///
/// This approach is simple and always terminates — it doesn't use CDT or
/// bridge-edge triangulation, avoiding all the failure modes of those algorithms.
/// The hole boundaries follow the mesh triangulation (slightly jagged), but
/// for simple shapes like the digit "3" this looks perfectly acceptable.
///
/// # Arguments
/// * `base_mesh` - The base primitive mesh to cut holes into
/// * `text` - Text string to cut (e.g., "3")
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

    // Determine which contours are "holes to cut" vs "solid islands to keep".
    // For the digit "3" there is only one contour (the outer shape), so it
    // becomes a hole. For characters with inner holes (D, a, p, e), the
    // inner contours become solid islands.
    //
    // Parity-based classification: a contour whose centroid is inside an
    // EVEN number of other contours → outer letter shape → becomes a hole.
    // Inside an ODD number → inner island (like D's hole) → stays solid.
    let hole_indices: Vec<usize> = contours_2d.iter()
        .enumerate()
        .filter(|(_, c)| {
            let mid_x: f64 = c.iter().map(|p| p.0).sum::<f64>() / c.len() as f64;
            let mid_y: f64 = c.iter().map(|p| p.1).sum::<f64>() / c.len() as f64;

            let mut inside_count = 0;
            for other in contours_2d.iter() {
                if point_in_polygon_2d(mid_x, mid_y, other) {
                    inside_count += 1;
                }
            }
            // Even = outer letter contour → hole; Odd = inner island → keep
            inside_count % 2 == 0
        })
        .map(|(i, _)| i)
        .collect();

    // === Simple subtractive approach: remove triangles inside hole contours ===
    //
    // For each triangle in the base mesh, project its centroid to 2D text
    // coordinates using the inverse of project_2d_to_surface, then check if
    // the 2D point is inside any hole contour.
    let mut result = TriangleMesh::new();
    let base_color = [0.48, 0.52, 0.58, 1.0];
    let mut num_base_tris = 0usize;

    for tri in &base_mesh.triangles {
        let v0 = base_mesh.vertices[tri[0] as usize];
        let v1 = base_mesh.vertices[tri[1] as usize];
        let v2 = base_mesh.vertices[tri[2] as usize];

        let cx = (v0.x + v1.x + v2.x) / 3.0;
        let cy = (v0.y + v1.y + v2.y) / 3.0;
        let cz = (v0.z + v1.z + v2.z) / 3.0;

        // Project 3D centroid to 2D text coordinates
        let (tx, ty) = project_3d_to_text_2d(cx, cy, cz, surface);

        // Check if the centroid is inside any hole contour
        let mut inside_hole = false;
        for &idx in &hole_indices {
            if point_in_polygon_2d(tx, ty, &contours_2d[idx]) {
                inside_hole = true;
                break;
            }
        }

        if !inside_hole {
            let base = result.vertices.len() as u32;
            result.vertices.push(v0);
            result.vertices.push(v1);
            result.vertices.push(v2);
            result.triangles.push([base, base + 1, base + 2]);
            num_base_tris += 1;
        }
    }

    // Add inset surfaces for each hole contour
    let mut hole_tri_count = 0usize;
    for &idx in &hole_indices {
        let contour_2d = &contours_2d[idx];
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

            result.triangles.push([si, ii, sj]);
            result.triangles.push([sj, ii, ij]);
            hole_tri_count += 2;
        }

        // Inset face (bottom of hole) — fan triangulation from first vertex
        for i in 1..n as u32 - 1 {
            result.triangles.push([
                inset_ring_start,
                inset_ring_start + i + 1,
                inset_ring_start + i,
            ]);
            hole_tri_count += 1;
        }
    }

    // Set up per-triangle colors
    let total_tris = result.triangles.len();
    let mut colors = Vec::with_capacity(total_tris);
    for _ in 0..num_base_tris {
        colors.push(base_color);
    }
    for _ in 0..hole_tri_count {
        colors.push(hole_color);
    }
    result.triangle_colors = Some(colors);

    if result.vertex_count() > 0 {
        result.compute_face_normals();
        filter_degenerate_tris(&mut result, 1e-8);
    }

    result
}

/// Filter out degenerate triangles from a mesh.
fn filter_degenerate_tris(mesh: &mut TriangleMesh, min_area_sq: f64) {
    let old_tris = std::mem::take(&mut mesh.triangles);
    let old_colors = mesh.triangle_colors.take();
    let old_face_normals = mesh.face_normals.take();
    
    for (i, tri) in old_tris.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        
        // Skip degenerate triangles (zero or near-zero area)
        let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
        let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
        let cross = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let area_sq = cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2];
        
        if area_sq >= min_area_sq && tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
            mesh.triangles.push(*tri);
            if let Some(ref colors) = old_colors {
                if let Some(c) = colors.get(i) {
                    mesh.triangle_colors.get_or_insert_with(Vec::new).push(*c);
                }
            }
            if let Some(ref normals) = old_face_normals {
                if let Some(n) = normals.get(i) {
                    mesh.face_normals.get_or_insert_with(Vec::new).push(*n);
                }
            }
        }
    }
}



/// Project a 3D point back to 2D text coordinates.
///
/// This is the inverse of `project_2d_to_surface`: given a 3D point on or near
/// the surface, compute the (x, y) text coordinates that would project to this point.
/// Used to determine which base mesh triangles overlap with text hole contours.
///
/// Returns (f64::MAX, f64::MAX) if the point is not on the surface.
fn project_3d_to_text_2d(px: f64, py: f64, pz: f64, surface: &TextSurface) -> (f64, f64) {
    match surface {
        TextSurface::Plane { z } => {
            // Inverse of: pos = (x, y, z), UV = (x, y)
            // For a plane, text x = 3D x, text y = 3D y
            (px, py)
        }
        TextSurface::Sphere { center, radius } => {
            // Inverse of project_2d_to_surface for sphere:
            //   scale_angle = 1/r, theta = y * scale_angle, phi = x * scale_angle
            //   pos = center + r * (cos(theta)*cos(phi), cos(theta)*sin(phi), sin(theta))
            // Inverse:
            //   dx = px - center[0], dy = py - center[1], dz = pz - center[2]
            //   theta = asin(dz / r), phi = atan2(dy, dx)
            //   text_x = phi * r, text_y = theta * r
            let r = *radius;
            let dx = px - center[0];
            let dy = py - center[1];
            let dz = pz - center[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            // Only consider points approximately on the sphere surface
            if (dist - r).abs() > r * 0.3 {
                return (f64::MAX, f64::MAX);
            }
            let phi = dy.atan2(dx);        // azimuthal angle
            let theta = (dz / r).clamp(-1.0, 1.0).asin(); // elevation angle
            let text_x = phi * r;
            let text_y = theta * r;
            (text_x, text_y)
        }
        TextSurface::Cylinder { radius, height } => {
            // Inverse of project_2d_to_surface for cylinder:
            //   scale_angle = 1/r, angle = x * scale_angle
            //   pos = (r*cos(angle), r*sin(angle), y + h/2)
            // Inverse:
            //   angle = atan2(py, px), text_x = angle * r
            //   text_y = pz - h/2
            let r = *radius;
            let h = *height;
            let dist_from_axis = (px * px + py * py).sqrt();
            if (dist_from_axis - r).abs() > r * 0.3 {
                return (f64::MAX, f64::MAX);
            }
            let angle = py.atan2(px);
            let text_x = angle * r;
            let text_y = pz - h / 2.0;
            (text_x, text_y)
        }
        TextSurface::Cone { radius, height } => {
            // Inverse of project_2d_to_surface for cone:
            //   scale_angle = 1.2/r, angle = x * scale_angle
            //   local_r = r * (1 - z/h), pos = (local_r*cos(angle), local_r*sin(angle), z)
            // Inverse:
            //   angle = atan2(py, px), text_x = angle / scale_angle = angle * r / 1.2
            //   text_y = pz - h/2
            let r = *radius;
            let h = *height;
            let dist_from_axis = (px * px + py * py).sqrt();
            let expected_r = r * (1.0 - pz / h).max(0.01);
            if (dist_from_axis - expected_r).abs() > expected_r.max(1.0) * 0.4 {
                return (f64::MAX, f64::MAX);
            }
            let angle = py.atan2(px);
            let text_x = angle * r / 1.2;
            let text_y = pz - h / 2.0;
            (text_x, text_y)
        }
        TextSurface::Torus { major_radius, minor_radius } => {
            // Inverse of project_2d_to_surface for torus:
            //   scale_major = 1/R, scale_minor = 1/r
            //   u = x * scale_major, v = y * scale_minor
            //   pos = ((R + r*cos(v))*cos(u), (R + r*cos(v))*sin(u), r*sin(v))
            // Inverse:
            //   u = atan2(py, px), v = atan2(pz, dist_from_axis - R)
            //   text_x = u * R, text_y = v * r
            let R = *major_radius;
            let r = *minor_radius;
            let dist_from_axis = (px * px + py * py).sqrt();
            let dist_from_ring = ((dist_from_axis - R) * (dist_from_axis - R) + pz * pz).sqrt();
            if (dist_from_ring - r).abs() > r * 0.5 {
                return (f64::MAX, f64::MAX);
            }
            let u = py.atan2(px);
            let v = pz.atan2(dist_from_axis - R);
            let text_x = u * R;
            let text_y = v * r;
            (text_x, text_y)
        }
    }
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
    let offset = 0.1;
    for v in &mut result.vertices {
        v.x += surface_center.x + surface_normal[0] * offset;
        v.y += surface_center.y + surface_normal[1] * offset;
        v.z += surface_center.z + surface_normal[2] * offset;
    }

    result
}

/// Simple fallback for text hole cutting when CDT fails.
///
/// Removes triangles whose centroids fall inside text contours and
/// replaces them with inset hole surfaces. Less accurate than CDT
/// (jagged edges), but always terminates and never hangs.
fn cut_text_holes_simple(
    base_mesh: &TriangleMesh,
    surface: &TextSurface,
    contours_2d: &[Vec<(f64, f64)>],
    hole_indices: &[usize],
) -> TriangleMesh {
    let mut result = TriangleMesh::new();

    for tri in &base_mesh.triangles {
        let v0 = base_mesh.vertices[tri[0] as usize];
        let v1 = base_mesh.vertices[tri[1] as usize];
        let v2 = base_mesh.vertices[tri[2] as usize];

        // Centroid
        let cx = (v0.x + v1.x + v2.x) / 3.0;
        let cy = (v0.y + v1.y + v2.y) / 3.0;

        // Project centroid onto surface to get 2D position
        let sp = project_2d_to_surface(cx, cy, surface);

        // Check if the projected centroid is inside any hole contour
        let mut inside_hole = false;
        for &idx in hole_indices {
            if idx < contours_2d.len() {
                if point_in_polygon_2d(sp.pos.x, sp.pos.y, &contours_2d[idx]) {
                    inside_hole = true;
                    break;
                }
            }
        }

        if !inside_hole {
            let base = result.vertices.len() as u32;
            result.vertices.push(v0);
            result.vertices.push(v1);
            result.vertices.push(v2);
            result.triangles.push([base, base + 1, base + 2]);
        }
    }

    result
}
