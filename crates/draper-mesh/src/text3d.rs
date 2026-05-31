// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 3D text mesh generation for "3Draper" branding.
//!
//! Generates 3D extruded text as a `TriangleMesh` using vector glyph outlines.
//! Each glyph is defined as a set of closed polygon contours (stroke paths).
//! The resulting mesh has front faces, back faces, and side walls.
//!
//! Also provides CDT-based text hole cutting: actual holes in the shape of
//! "3Draper" text are cut through primitive surfaces using Constrained Delaunay
//! Triangulation for clean, accurate boundaries.

use crate::mesh::TriangleMesh;
use crate::parametric_domain::{ParametricDomain, triangulate_cdt, generate_interior_points};
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
        TextSurface::Plane { z: _ } => {
            // We'll handle plane specially — just use XY extents
            let half_w = 50.0 * text_scale;
            let half_h = 20.0 * text_scale;
            (Surface::Plane(Plane::xy()), (-half_w, half_w), (-half_h, half_h))
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

/// Cut "3Draper" text-shaped holes in a mesh using CDT-based approach.
///
/// Instead of removing whole triangles whose centroids fall inside text contours
/// (which creates jagged edges), this function:
/// 1. Generates a new surface mesh using Constrained Delaunay Triangulation
///    with the text contours as holes in UV space
/// 2. Creates clean inset surfaces for the holes with proper depth
/// 3. Creates side walls connecting the surface boundary to the inset
///
/// # Arguments
/// * `base_mesh` - The base primitive mesh to cut holes into (used for non-text faces)
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

    // Get the analytical surface and UV domain
    let (geo_surface, u_range, v_range) = get_surface_and_uv_domain(surface, scale);

    // Convert text contours from 2D text space to UV space
    let contours_uv: Vec<Vec<Point2d>> = contours_2d.iter()
        .map(|c| text_2d_to_uv(c, surface))
        .collect();

    // Build the outer boundary in UV space (rectangle covering the text area)
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;

    // Expand the UV domain a bit to ensure text fits well
    let margin_u = (u_max - u_min) * 0.3;
    let margin_v = (v_max - v_min) * 0.3;
    let outer_u_min = u_min - margin_u;
    let outer_u_max = u_max + margin_u;
    let outer_v_min = v_min - margin_v;
    let outer_v_max = v_max + margin_v;

    // Outer boundary rectangle in UV space
    let outer_boundary = vec![
        Point2d::new(outer_u_min, outer_v_min),
        Point2d::new(outer_u_max, outer_v_min),
        Point2d::new(outer_u_max, outer_v_max),
        Point2d::new(outer_u_min, outer_v_max),
    ];

    // Determine which contours are outer contours and which are inner holes
    // A contour is a "hole" (text cut-out) if it's inside an odd number of contours
    // (parity test). In practice, we simplify: all contours become holes.
    // The D/a/p/e letters have their own internal holes which should NOT be
    // cut — we need parity-based classification.
    let hole_contours: Vec<Vec<Point2d>> = contours_uv.iter()
        .filter(|c| {
            // Test midpoint of the contour — if it's inside an odd number of
            // OTHER contours, it's a hole; otherwise it's an outer boundary.
            let mid_x: f64 = c.iter().map(|p| p.u).sum::<f64>() / c.len() as f64;
            let mid_y: f64 = c.iter().map(|p| p.v).sum::<f64>() / c.len() as f64;

            let mut inside_count = 0;
            for other in &contours_2d {
                if point_in_polygon_2d(mid_x, mid_y, other) {
                    inside_count += 1;
                }
            }
            // Odd count = this is a solid text region (outer contour) → becomes a hole
            inside_count % 2 == 1
        })
        .cloned()
        .collect();

    // Build parametric domain with holes
    let mut domain = ParametricDomain::new(
        outer_boundary,
        (outer_u_min, outer_u_max),
        (outer_v_min, outer_v_max),
    );
    for hole in &hole_contours {
        domain = domain.with_hole(hole.clone());
    }

    // Generate interior points for CDT
    let n_u = 16;
    let n_v = 8;
    let boundary_margin = (outer_u_max - outer_u_min) / n_u as f64 * 0.2;
    let interior_points = generate_interior_points(&domain, n_u, n_v, boundary_margin);

    // Triangulate using CDT
    let mut text_face_mesh = triangulate_cdt(&domain, &geo_surface, true, &interior_points);

    // Now build the complete mesh: base mesh faces + text-cut face + hole insets
    let mut result = TriangleMesh::new();
    let base_color = [0.48, 0.52, 0.58, 1.0];

    // Add base mesh triangles that are NOT on the text face
    // (i.e., keep all faces from the primitive that don't overlap with the text region)
    let mut num_base_tris = 0usize;

    for (i, tri) in base_mesh.triangles.iter().enumerate() {
        let v0 = base_mesh.vertices[tri[0] as usize];
        let v1 = base_mesh.vertices[tri[1] as usize];
        let v2 = base_mesh.vertices[tri[2] as usize];

        // Centroid
        let cx = (v0.x + v1.x + v2.x) / 3.0;
        let cy = (v0.y + v1.y + v2.y) / 3.0;
        let cz = (v0.z + v1.z + v2.z) / 3.0;

        // Check if centroid is inside the outer UV domain of the text face
        let sp = project_2d_to_surface(cx, cy, surface); // approximate
        // Project centroid to UV to check if it's in the text region
        let in_text_region = is_in_text_uv_region(cx, cy, cz, surface, outer_u_min, outer_u_max, outer_v_min, outer_v_max);

        if !in_text_region {
            // Keep this triangle — it's outside the text region
            let base = result.vertices.len() as u32;
            result.vertices.push(v0);
            result.vertices.push(v1);
            result.vertices.push(v2);
            result.triangles.push([base, base + 1, base + 2]);
            num_base_tris += 1;
        }
    }

    // Add the CDT-triangulated text face (with holes)
    let text_face_offset = result.vertices.len() as u32;
    let text_face_tri_count = text_face_mesh.triangles.len();
    for v in &text_face_mesh.vertices {
        result.vertices.push(*v);
    }
    for tri in &text_face_mesh.triangles {
        result.triangles.push([tri[0] + text_face_offset, tri[1] + text_face_offset, tri[2] + text_face_offset]);
    }

    // Add inset surfaces for each hole contour
    for contour_2d in &contours_2d {
        let contour_uv = text_2d_to_uv(contour_2d, surface);
        if contour_uv.len() < 3 {
            continue;
        }

        // Check if this is an outer contour (odd parity) — only create insets for those
        let mid_x: f64 = contour_2d.iter().map(|p| p.0).sum::<f64>() / contour_2d.len() as f64;
        let mid_y: f64 = contour_2d.iter().map(|p| p.1).sum::<f64>() / contour_2d.len() as f64;
        let mut inside_count = 0;
        for other in &contours_2d {
            if point_in_polygon_2d(mid_x, mid_y, other) {
                inside_count += 1;
            }
        }
        if inside_count % 2 != 1 {
            // This is an inner hole (like the hole in 'D' or 'a') — skip inset
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

            // Two triangles per quad
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

/// Check if a 3D point is within the UV region where text is being cut.
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
        TextSurface::Sphere { center, radius: _ } => {
            // Check if point is on the sphere and within the UV region
            let dx = px - center[0];
            let dy = py - center[1];
            let dz = pz - center[2];
            let phi = dy.atan2(dx);
            let theta = dz.asin().max(-1.0).min(1.0);
            let r = (dx * dx + dy * dy + dz * dz).sqrt();
            if r < 1.0 {
                return false; // Too close to center, not on surface
            }
            // Normalize angles to compare
            let phi_norm = if phi < 0.0 { phi + 2.0 * PI } else { phi };
            let theta_norm = theta + PI / 2.0;
            phi_norm >= u_min && phi_norm <= u_max && theta_norm >= v_min && theta_norm <= v_max
        }
        TextSurface::Cylinder { radius: _, height: _ } => {
            // For cylinder, UV = (angle, z)
            let angle = py.atan2(px);
            let angle_norm = if angle < 0.0 { angle + 2.0 * PI } else { angle };
            angle_norm >= u_min && angle_norm <= u_max && pz >= v_min && pz <= v_max
        }
        TextSurface::Cone { radius: _, height: _ } => {
            // For cone, UV = (angle, z)
            let angle = py.atan2(px);
            let angle_norm = if angle < 0.0 { angle + 2.0 * PI } else { angle };
            angle_norm >= u_min && angle_norm <= u_max && pz >= v_min && pz <= v_max
        }
        TextSurface::Torus { major_radius, minor_radius: _ } => {
            // For torus, UV = (major angle, minor angle)
            let R = *major_radius;
            let u = py.atan2(px);
            let u_norm = if u < 0.0 { u + 2.0 * PI } else { u };
            // Approximate: check if near the outer equator of the torus
            let dist_from_axis = (px * px + py * py).sqrt();
            let v_approx = (pz / (dist_from_axis - R).max(0.01)).atan();
            u_norm >= u_min && u_norm <= u_max && v_approx >= v_min && v_approx <= v_max
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
