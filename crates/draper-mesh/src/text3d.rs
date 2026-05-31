// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 3D text mesh generation for "3Draper" branding.
//!
//! Generates 3D extruded text as a `TriangleMesh` using vector glyph outlines.
//! Each glyph is defined as a set of closed polygon contours (stroke paths).
//! The resulting mesh has front faces, back faces, and side walls.

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
