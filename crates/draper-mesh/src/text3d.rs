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
use draper_geometry::{Point3d, Point2d, Surface, Plane, SphereSurface, CylinderSurface, ConeSurface, TorusSurface};
use std::f64::consts::PI;

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
                    // Simplify the contour to reduce CDT constraint edge count.
                    // Epsilon is relative to scale — larger text = more simplification tolerance.
                    // Target: reduce each contour to ~6-10 points max.
                    let epsilon = scale * 1.5;
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

/// Get the Surface geometry object and UV domain for a TextSurface.
fn get_surface_and_uv_domain(surface: &TextSurface, text_scale: f64) -> (Surface, (f64, f64), (f64, f64)) {
    match surface {
        TextSurface::Plane { z } => {
            // XY plane at the given z height — must use correct origin for CDT vertex mapping
            let half_w = 50.0 * text_scale;
            let half_h = 20.0 * text_scale;
            let plane = Plane::from_origin_and_normal(
                Point3d::new(0.0, 0.0, *z),
                draper_geometry::Direction3d::Z,
            );
            (Surface::Plane(plane), (-half_w, half_w), (-half_h, half_h))
        }
        TextSurface::Sphere { center, radius } => {
            let r = *radius;
            let scale_angle = 1.0 / r;
            let half_w = 30.0 * text_scale * scale_angle;
            let half_h = 10.0 * text_scale * scale_angle;
            (Surface::Sphere(SphereSurface::new(Point3d::new(center[0], center[1], center[2]), r)),
             (-half_w, half_w), (-half_h, half_h))
        }
        TextSurface::Cylinder { radius, height } => {
            let r = *radius;
            let h = *height;
            let scale_angle = 1.0 / r;
            let half_u = 30.0 * text_scale * scale_angle;
            let half_v = 10.0 * text_scale;
            (Surface::Cylinder(CylinderSurface::new_z(r)),
             (-half_u, half_u), (-half_v + h / 2.0, half_v + h / 2.0))
        }
        TextSurface::Cone { radius, height } => {
            let r = *radius;
            let h = *height;
            let scale_angle = 1.2 / r;
            let half_u = 30.0 * text_scale * scale_angle;
            let half_v = 10.0 * text_scale;
            let half_angle = (r / h).atan();
            (Surface::Cone(ConeSurface::new_z(r, half_angle)),
             (-half_u, half_u), (-half_v + h / 2.0, half_v + h / 2.0))
        }
        TextSurface::Torus { major_radius, minor_radius } => {
            let R = *major_radius;
            let r = *minor_radius;
            let scale_major = 1.0 / R;
            let scale_minor = 1.0 / r;
            let half_u = 20.0 * text_scale * scale_major;
            let half_v = 10.0 * text_scale * scale_minor;
            (Surface::Torus(TorusSurface::new_z(Point3d::ORIGIN, R, r)),
             (-half_u, half_u), (-half_v, half_v))
        }
    }
}

/// Convert 2D text contour points to UV coordinates on the surface.
fn text_2d_to_uv(pts: &[(f64, f64)], surface: &TextSurface) -> Vec<Point2d> {
    match surface {
        TextSurface::Plane { z: _ } => {
            // UV = (x, y) directly
            pts.iter().map(|&(x, y)| Point2d::new(x, y)).collect()
        }
        TextSurface::Sphere { radius, .. } => {
            // x → azimuthal angle (phi), y → polar angle (theta)
            let r = *radius;
            let scale = 1.0 / r;
            pts.iter().map(|&(x, y)| Point2d::new(x * scale, y * scale + PI / 2.0)).collect()
        }
        TextSurface::Cylinder { radius, height } => {
            // x → angle, y → height
            let r = *radius;
            let h = *height;
            let scale = 1.0 / r;
            pts.iter().map(|&(x, y)| Point2d::new(x * scale, y + h / 2.0)).collect()
        }
        TextSurface::Cone { radius, height } => {
            // x → angle, y → z position
            let r = *radius;
            let h = *height;
            let scale = 1.2 / r;
            pts.iter().map(|&(x, y)| Point2d::new(x * scale, y + h / 2.0)).collect()
        }
        TextSurface::Torus { major_radius, minor_radius } => {
            // x → major angle (u), y → minor angle (v)
            let R = *major_radius;
            let r = *minor_radius;
            pts.iter().map(|&(x, y)| Point2d::new(x / R, y / r)).collect()
        }
    }
}

/// Cut "3Draper" text-shaped holes in a mesh.
///
/// This function creates holes in the shape of text characters through a surface:
/// 1. Removes triangles whose centroids fall inside text contours (simple, always terminates)
/// 2. Replaces the text region with a new surface mesh that has proper holes
///    using ear-clipping with bridge-edge hole insertion (guaranteed to terminate,
///    unlike CDT which can hang on complex constraint edge configurations)
/// 3. Creates inset surfaces for the holes with proper depth
/// 4. Creates side walls connecting the surface boundary to the inset
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

    // Determine which contours are outer contours and which are inner holes
    // A contour is a "hole" (text cut-out) if it's inside an odd number of contours
    // (parity test). The D/a/p/e letters have their own internal holes which should
    // NOT be cut — we need parity-based classification.
    let hole_indices: Vec<usize> = contours_2d.iter()
        .enumerate()
        .filter(|(_, c)| {
            let mid_x: f64 = c.iter().map(|p| p.0).sum::<f64>() / c.len() as f64;
            let mid_y: f64 = c.iter().map(|p| p.1).sum::<f64>() / c.len() as f64;

            let mut inside_count = 0;
            for other in &contours_2d {
                if point_in_polygon_2d(mid_x, mid_y, other) {
                    inside_count += 1;
                }
            }
            // Odd count = this is a solid text region (outer contour) → becomes a hole
            inside_count % 2 == 1
        })
        .map(|(i, _)| i)
        .collect();

    // Get the analytical surface and UV domain
    let (geo_surface, u_range, v_range) = get_surface_and_uv_domain(surface, scale);
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;

    // Expand the UV domain a bit to ensure text fits well
    let margin_u = (u_max - u_min) * 0.3;
    let margin_v = (v_max - v_min) * 0.3;
    let outer_u_min = u_min - margin_u;
    let outer_u_max = u_max + margin_u;
    let outer_v_min = v_min - margin_v;
    let outer_v_max = v_max + margin_v;

    // Build the text face mesh with holes using ear-clipping with bridge edges.
    // This approach is guaranteed to terminate (unlike CDT which can hang
    // when constraint edges intersect or are nearly coincident).
    //
    // Algorithm:
    // 1. Create a 2D rectangle in UV space for the text region
    // 2. Map text hole contours into the rectangle
    // 3. Use bridge-edge technique to merge holes into the outer polygon
    // 4. Ear-clip the merged polygon (always terminates in O(n²))
    // 5. Map UV vertices back to 3D using the analytical surface
    let text_face_mesh = build_text_face_earclip(
        &geo_surface,
        surface,
        &contours_2d,
        &hole_indices,
        outer_u_min, outer_u_max,
        outer_v_min, outer_v_max,
    );

    // Now build the complete mesh: base mesh faces + text-cut face + hole insets
    let mut result = TriangleMesh::new();
    let base_color = [0.48, 0.52, 0.58, 1.0];

    // Add base mesh triangles that are NOT on the text face
    let mut num_base_tris = 0usize;

    for (_i, tri) in base_mesh.triangles.iter().enumerate() {
        let v0 = base_mesh.vertices[tri[0] as usize];
        let v1 = base_mesh.vertices[tri[1] as usize];
        let v2 = base_mesh.vertices[tri[2] as usize];

        let cx = (v0.x + v1.x + v2.x) / 3.0;
        let cy = (v0.y + v1.y + v2.y) / 3.0;
        let cz = (v0.z + v1.z + v2.z) / 3.0;

        let in_text_region = is_in_text_uv_region(
            cx, cy, cz, surface,
            outer_u_min, outer_u_max, outer_v_min, outer_v_max,
        );

        if !in_text_region {
            let base = result.vertices.len() as u32;
            result.vertices.push(v0);
            result.vertices.push(v1);
            result.vertices.push(v2);
            result.triangles.push([base, base + 1, base + 2]);
            num_base_tris += 1;
        }
    }

    // Add the ear-clipped text face (with holes)
    let text_face_offset = result.vertices.len() as u32;
    let text_face_tri_count = text_face_mesh.triangles.len();
    for v in &text_face_mesh.vertices {
        result.vertices.push(*v);
    }
    for tri in &text_face_mesh.triangles {
        result.triangles.push([tri[0] + text_face_offset, tri[1] + text_face_offset, tri[2] + text_face_offset]);
    }

    // Add inset surfaces for each hole contour
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
    let total_tris = result.triangles.len();
    let hole_tris = total_tris - num_base_tris - text_face_tri_count;
    let mut colors = Vec::with_capacity(total_tris);
    for _ in 0..num_base_tris {
        colors.push(base_color);
    }
    for _ in 0..text_face_tri_count {
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

/// Build the text face mesh with holes using ear-clipping with bridge edges.
///
/// This is a guaranteed-to-terminate alternative to CDT that:
/// 1. Creates a UV-space rectangle for the text region
/// 2. Maps text hole contours into the rectangle
/// 3. Uses bridge-edge technique to merge each hole into the outer polygon
/// 4. Ear-clips the merged polygon (O(n²) worst case, always terminates)
/// 5. Maps UV vertices to 3D using the analytical surface
///
/// For curved surfaces, we add interior grid points to capture curvature,
/// but these are added as simple vertex insertions (not CDT constraints).
fn build_text_face_earclip(
    geo_surface: &Surface,
    surface: &TextSurface,
    contours_2d: &[Vec<(f64, f64)>],
    hole_indices: &[usize],
    u_min: f64, u_max: f64,
    v_min: f64, v_max: f64,
) -> TriangleMesh {
    // Convert hole contours from 2D text space to UV space
    let hole_contours_uv: Vec<Vec<Point2d>> = hole_indices.iter()
        .map(|&i| text_2d_to_uv(&contours_2d[i], surface))
        .collect();

    // Build the outer boundary in UV space (rectangle covering the text area)
    let outer_uv = vec![
        Point2d::new(u_min, v_min),
        Point2d::new(u_max, v_min),
        Point2d::new(u_max, v_max),
        Point2d::new(u_min, v_max),
    ];

    // Collect all 2D points: outer boundary first, then holes
    let mut all_uv_points: Vec<Point2d> = outer_uv.clone();

    // Insert each hole into the polygon using bridge-edge technique
    // This is the same approach used in triangulate_planar_face for holes
    let mut polygon_indices: Vec<u32> = (0..outer_uv.len() as u32).collect();

    for hole_uv in &hole_contours_uv {
        let hole_start_idx = all_uv_points.len();

        // Find the bridge: rightmost point of the hole, closest point on outer polygon
        let bridge_result = find_bridge_edge_text(&all_uv_points, &polygon_indices, hole_uv);

        // Add hole points to the combined point list
        for p in hole_uv {
            all_uv_points.push(*p);
        }

        // Insert hole into polygon via bridge edge
        let mut new_polygon = Vec::with_capacity(polygon_indices.len() + hole_uv.len() + 2);
        let bridge_outer = bridge_result.outer_idx;
        let bridge_hole = hole_start_idx + bridge_result.hole_idx;

        for &idx in &polygon_indices[..=bridge_outer] {
            new_polygon.push(idx);
        }
        // Bridge: outer → hole → ... hole loop ... → hole → outer
        new_polygon.push(bridge_hole as u32);
        for i in 0..hole_uv.len() {
            let idx = hole_start_idx + (bridge_result.hole_idx + i) % hole_uv.len();
            new_polygon.push(idx as u32);
        }
        new_polygon.push(bridge_hole as u32);
        new_polygon.push(polygon_indices[bridge_outer]);
        for &idx in &polygon_indices[bridge_outer + 1..] {
            new_polygon.push(idx);
        }

        polygon_indices = new_polygon;
    }

    // Add interior grid points for curved surfaces to capture curvature
    let n_interior_u = match surface {
        TextSurface::Plane { .. } => 0, // No interior points needed for flat plane
        _ => {
            #[cfg(target_arch = "wasm32")]
            { 4 }
            #[cfg(not(target_arch = "wasm32"))]
            { 8 }
        }
    };
    let n_interior_v = match surface {
        TextSurface::Plane { .. } => 0,
        _ => {
            #[cfg(target_arch = "wasm32")]
            { 3 }
            #[cfg(not(target_arch = "wasm32"))]
            { 5 }
        }
    };

    let mut interior_point_indices: Vec<u32> = Vec::new();
    if n_interior_u > 0 && n_interior_v > 0 {
        for j in 1..n_interior_v {
            for i in 1..n_interior_u {
                let u = u_min + (u_max - u_min) * i as f64 / n_interior_u as f64;
                let v = v_min + (v_max - v_min) * j as f64 / n_interior_v as f64;

                // Check if this interior point is NOT inside any hole
                let pt = Point2d::new(u, v);
                let mut inside_hole = false;
                for hole_uv in &hole_contours_uv {
                    if point_in_polygon_2d_uv(&pt, hole_uv) {
                        inside_hole = true;
                        break;
                    }
                }
                if !inside_hole {
                    let idx = all_uv_points.len() as u32;
                    all_uv_points.push(pt);
                    interior_point_indices.push(idx);
                }
            }
        }
    }

    // Now ear-clip the merged polygon (with holes inserted via bridge edges)
    let merged_2d: Vec<Point2d> = polygon_indices.iter()
        .map(|&idx| all_uv_points[idx as usize])
        .collect();

    let triangles = crate::ear_clip(&merged_2d);

    // Build 3D mesh: map UV vertices to 3D using the analytical surface
    let mut mesh = TriangleMesh::new();

    // Add all polygon vertices as 3D points
    for &idx in &polygon_indices {
        let uv = all_uv_points[idx as usize];
        let p3d = geo_surface.point_at(uv.u, uv.v);
        mesh.add_vertex(p3d);
    }

    // Add interior vertices
    let interior_offset = polygon_indices.len() as u32;
    for &idx in &interior_point_indices {
        let uv = all_uv_points[idx as usize];
        let p3d = geo_surface.point_at(uv.u, uv.v);
        mesh.add_vertex(p3d);
    }

    // Map ear-clip triangle indices back to vertex indices
    for tri in &triangles {
        let i0 = polygon_indices[tri[0] as usize];
        let i1 = polygon_indices[tri[1] as usize];
        let i2 = polygon_indices[tri[2] as usize];
        // These are indices into the polygon_indices array, which are direct
        // vertex indices in our mesh (we added polygon vertices first)
        mesh.add_triangle(i0, i1, i2);
    }

    // For interior points, we need to insert them into the mesh.
    // Simple approach: for each interior point, find the triangle that contains it
    // and subdivide that triangle into 3 sub-triangles.
    for (k, &_uv_idx) in interior_point_indices.iter().enumerate() {
        let new_vert_idx = interior_offset + k as u32;
        let uv = all_uv_points[interior_point_indices[k] as usize];

        // Find a triangle whose 2D bounding contains this point
        let mut best_tri: Option<usize> = None;
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];

            // Project to UV for containment test
            let (u0, v0v) = geo_surface.project_point(&v0);
            let (u1, v1v) = geo_surface.project_point(&v1);
            let (u2, v2v) = geo_surface.project_point(&v2);

            // Simple bounding box test
            let min_u = u0.min(u1).min(u2);
            let max_u = u0.max(u1).max(u2);
            let min_v = v0v.min(v1v).min(v2v);
            let max_v = v0v.max(v1v).max(v2v);

            if uv.u >= min_u && uv.u <= max_u && uv.v >= min_v && uv.v <= max_v {
                best_tri = Some(ti);
                break;
            }
        }

        if let Some(ti) = best_tri {
            let [a, b, c] = mesh.triangles[ti];
            // Replace triangle with 3 sub-triangles
            mesh.triangles[ti] = [a, b, new_vert_idx];
            mesh.triangles.push([b, c, new_vert_idx]);
            mesh.triangles.push([c, a, new_vert_idx]);
        }
    }

    // Compute normals for the text face
    mesh.compute_face_normals();

    mesh
}

/// Find bridge edge between outer polygon and a hole for ear-clipping.
/// Returns indices into the outer polygon indices and hole points.
struct BridgeResultText {
    outer_idx: usize,
    hole_idx: usize,
}

fn find_bridge_edge_text(
    all_points: &[Point2d],
    polygon_indices: &[u32],
    hole_2d: &[Point2d],
) -> BridgeResultText {
    // Find rightmost point of the hole
    let mut hole_idx = 0;
    let mut max_u = hole_2d[0].u;
    for (i, p) in hole_2d.iter().enumerate() {
        if p.u > max_u {
            max_u = p.u;
            hole_idx = i;
        }
    }

    // Find closest point on outer polygon to the rightmost hole point
    let hole_pt = &hole_2d[hole_idx];
    let mut outer_idx = 0;
    let mut min_dist = f64::MAX;
    for (i, &idx) in polygon_indices.iter().enumerate() {
        let p = &all_points[idx as usize];
        let dx = p.u - hole_pt.u;
        let dy = p.v - hole_pt.v;
        let dist = dx * dx + dy * dy;
        if dist < min_dist {
            min_dist = dist;
            outer_idx = i;
        }
    }

    BridgeResultText { outer_idx, hole_idx }
}

/// Point-in-polygon test for UV-space Point2d.
fn point_in_polygon_2d_uv(point: &Point2d, polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let px = point.u;
    let py = point.v;
    let mut j = n - 1;
    for i in 0..n {
        let xi = polygon[i].u;
        let yi = polygon[i].v;
        let xj = polygon[j].u;
        let yj = polygon[j].v;
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Check if a 3D point is within the UV region where text is being cut.
///
/// Uses the analytical surface's `project_point` to map 3D → UV, then checks
/// if the UV coordinates fall within the text region's UV bounding box.
/// This works correctly for all surface types because `project_point` is the
/// inverse of `point_at`, which is used by `triangulate_cdt` to generate 3D
/// vertices from UV coordinates.
fn is_in_text_uv_region(
    px: f64, py: f64, pz: f64,
    surface: &TextSurface,
    u_min: f64, u_max: f64,
    v_min: f64, v_max: f64,
) -> bool {
    match surface {
        TextSurface::Plane { z } => {
            // For plane, UV = (x, y), check if point is at the right z level
            let z_val = *z;
            if (pz - z_val).abs() > 5.0 {
                return false; // Not on this face
            }
            px >= u_min && px <= u_max && py >= v_min && py <= v_max
        }
        TextSurface::Sphere { center, radius } => {
            // Project 3D point to sphere UV (phi, theta)
            let r = *radius;
            let dx = px - center[0];
            let dy = py - center[1];
            let dz = pz - center[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            // Check if point is approximately on the sphere surface
            if (dist - r).abs() > r * 0.2 {
                return false;
            }
            // UV mapping matches text_2d_to_uv: phi = x * (1/r), theta = y * (1/r) + PI/2
            let phi = dy.atan2(dx); // azimuthal angle
            let theta = dz.asin().clamp(-1.0, 1.0); // elevation angle
            // The text UV uses: u = x * (1/r), v = y * (1/r) + PI/2
            // where x,y are the 2D text coordinates centered at origin.
            // So for a point at the front of the sphere (phi≈0, theta≈0),
            // the corresponding 2D x = phi * r, y = (theta + PI/2 - PI/2) * r = theta * r
            // But wait — the actual mapping is u = x/r → x = u * r.
            // And the text is centered at origin in 2D, which maps to u=0, v=PI/2 in UV.
            // So u_text = phi, v_text = theta + PI/2
            // But phi is in [-PI, PI] and we need it relative to the text center (u=0).
            // For text centered at the front of sphere, phi=0 → u=0.
            let u = phi;  // matches text_2d_to_uv mapping
            let v = theta + PI / 2.0;
            u >= u_min && u <= u_max && v >= v_min && v <= v_max
        }
        TextSurface::Cylinder { radius, height: _ } => {
            // UV mapping: u = x * (1/r) → angle, v = y + h/2
            let r = *radius;
            let dist_from_axis = (px * px + py * py).sqrt();
            if (dist_from_axis - r).abs() > r * 0.2 {
                return false; // Not on cylinder surface
            }
            let angle = py.atan2(px);
            let u = angle; // matches text_2d_to_uv mapping: x * (1/r)
            let v = pz;    // matches text_2d_to_uv mapping: y + h/2
            u >= u_min && u <= u_max && v >= v_min && v <= v_max
        }
        TextSurface::Cone { radius, height } => {
            // UV mapping: u = x * (1.2/r) → angle, v = y + h/2
            let r = *radius;
            let h = *height;
            let dist_from_axis = (px * px + py * py).sqrt();
            let expected_r = r * (1.0 - pz / h);
            if (dist_from_axis - expected_r).abs() > expected_r.max(1.0) * 0.3 {
                return false; // Not on cone surface
            }
            let angle = py.atan2(px);
            let u = angle;
            let v = pz;
            u >= u_min && u <= u_max && v >= v_min && v <= v_max
        }
        TextSurface::Torus { major_radius, minor_radius } => {
            // UV mapping: u = x / R, v = y / r
            let R = *major_radius;
            let r = *minor_radius;
            let dist_from_axis = (px * px + py * py).sqrt();
            // Approximate check: is this point on the torus surface?
            let dist_from_ring = ((dist_from_axis - R) * (dist_from_axis - R) + pz * pz).sqrt();
            if (dist_from_ring - r).abs() > r * 0.3 {
                return false; // Not on torus surface
            }
            let u = py.atan2(px); // major angle
            // For v (minor angle), compute the angle in the tube cross-section
            let v_approx = pz.atan2(dist_from_axis - R);
            u >= u_min && u <= u_max && v_approx >= v_min && v_approx <= v_max
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
