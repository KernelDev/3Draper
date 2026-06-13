// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! PMI (Product Manufacturing Information) 3D display generation.
//!
//! Generates 3D mesh elements for visualizing PMI annotations from STEP files:
//!
//! - **Leader lines** — 3D line segments from the annotated geometry to the
//!   text label, rendered as thin rectangular strips
//! - **Text labels** — flat 3D text meshes positioned in world space using
//!   the existing `text3d::generate_text_contours` infrastructure
//! - **Dimension lines** — linear or angular dimension indicators with
//!   arrows and extension lines
//!
//! # Coordinate system
//!
//! PMI elements are positioned in 3D world coordinates. The caller provides
//! the attachment point on the geometry and the label position. The module
//! generates a `PmiDisplayMesh` containing all the 3D mesh elements.
//!
//! # Usage
//! ```ignore
//! use draper_mesh::pmi_display::{PmiDisplayBuilder, PmiAnnotationRef};
//!
//! let builder = PmiDisplayBuilder::new();
//! let display = builder
//!     .add_dimension(
//!         Point3d::new(0.0, 0.0, 0.0),   // start point
//!         Point3d::new(10.0, 0.0, 0.0),  // end point
//!         Point3d::new(5.0, 5.0, 0.0),   // label position
//!         "10.0 mm",                       // text
//!     )
//!     .add_leader_line(
//!         Point3d::new(5.0, 0.0, 0.0),   // attachment point
//!         Point3d::new(5.0, 5.0, 0.0),   // label position
//!         "R5",                            // text
//!     )
//!     .build();
//! ```

use crate::mesh::TriangleMesh;
use crate::text3d;
use draper_geometry::Point3d;

/// A collection of 3D mesh elements representing PMI annotations.
///
/// Contains separate meshes for different PMI element types so that
/// the viewer can style them differently (e.g., different colors for
/// dimensions vs. tolerances, different line widths for leader lines).
#[derive(Clone, Debug)]
pub struct PmiDisplayMesh {
    /// Mesh for leader lines and dimension lines (thin strips).
    pub lines: TriangleMesh,
    /// Mesh for text labels (flat 3D text).
    pub labels: TriangleMesh,
    /// Mesh for arrow heads.
    pub arrows: TriangleMesh,
    /// Number of individual PMI annotations in this display.
    pub annotation_count: usize,
}

impl PmiDisplayMesh {
    /// Create an empty PMI display mesh.
    pub fn new() -> Self {
        Self {
            lines: TriangleMesh::new(),
            labels: TriangleMesh::new(),
            arrows: TriangleMesh::new(),
            annotation_count: 0,
        }
    }

    /// Merge another PMI display mesh into this one.
    pub fn merge(&mut self, other: &PmiDisplayMesh) {
        self.lines.merge(&other.lines);
        self.labels.merge(&other.labels);
        self.arrows.merge(&other.arrows);
        self.annotation_count += other.annotation_count;
    }

    /// Get a combined mesh with all elements (lines + labels + arrows).
    pub fn combined(&self) -> TriangleMesh {
        let mut combined = TriangleMesh::new();
        combined.merge(&self.lines);
        combined.merge(&self.labels);
        combined.merge(&self.arrows);
        combined
    }
}

impl Default for PmiDisplayMesh {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for constructing PMI display meshes.
///
/// Accumulates individual PMI annotations (dimensions, tolerances, notes)
/// and their associated 3D mesh elements, then produces a single
/// `PmiDisplayMesh` on `build()`.
pub struct PmiDisplayBuilder {
    display: PmiDisplayMesh,
    /// Scale factor for text labels (default: 1.0).
    text_scale: f64,
    /// Width of leader/dimension lines in 3D units (default: 0.1).
    line_width: f64,
    /// Size of arrow heads (default: 0.5).
    arrow_size: f64,
}

impl PmiDisplayBuilder {
    /// Create a new PMI display builder with default settings.
    pub fn new() -> Self {
        Self {
            display: PmiDisplayMesh::new(),
            text_scale: 1.0,
            line_width: 0.1,
            arrow_size: 0.5,
        }
    }

    /// Set the text scale factor (default: 1.0).
    ///
    /// Larger values produce bigger text labels. The scale should be
    /// proportional to the model size — use ~1% of the model's bounding
    /// box diagonal for readable labels.
    pub fn text_scale(mut self, scale: f64) -> Self {
        self.text_scale = scale;
        self
    }

    /// Set the line width for leader/dimension lines (default: 0.1).
    pub fn line_width(mut self, width: f64) -> Self {
        self.line_width = width.max(0.01);
        self
    }

    /// Set the arrow head size (default: 0.5).
    pub fn arrow_size(mut self, size: f64) -> Self {
        self.arrow_size = size.max(0.01);
        self
    }

    /// Add a linear dimension annotation.
    ///
    /// Generates:
    /// - A dimension line from `start` to `end` (possibly offset to `label_pos`)
    /// - Extension lines from start/end perpendicular to the dimension line
    /// - Arrow heads at both ends
    /// - A text label at `label_pos`
    pub fn add_dimension(
        mut self,
        start: Point3d,
        end: Point3d,
        label_pos: Point3d,
        text: &str,
    ) -> Self {
        // Dimension line: from start to end via label_pos
        let line_mesh = generate_line_strip(&start, &end, self.line_width);
        self.display.lines.merge(&line_mesh);

        // Extension lines: from start/end to label_pos plane
        let ext1 = generate_line_strip(&start, &label_pos, self.line_width * 0.5);
        let ext2 = generate_line_strip(&end, &label_pos, self.line_width * 0.5);
        self.display.lines.merge(&ext1);
        self.display.lines.merge(&ext2);

        // Arrow heads at start and end
        let arrow1 = generate_arrow_head(&start, &end, self.arrow_size);
        let arrow2 = generate_arrow_head(&end, &start, self.arrow_size);
        self.display.arrows.merge(&arrow1);
        self.display.arrows.merge(&arrow2);

        // Text label at label_pos
        let label = generate_flat_text_label(text, &label_pos, self.text_scale);
        self.display.labels.merge(&label);

        self.display.annotation_count += 1;
        self
    }

    /// Add a leader line annotation (e.g., for notes, surface finish).
    ///
    /// Generates:
    /// - A leader line from `attachment` to `label_pos`
    /// - A small dot at the attachment point
    /// - A text label at `label_pos`
    pub fn add_leader_line(
        mut self,
        attachment: Point3d,
        label_pos: Point3d,
        text: &str,
    ) -> Self {
        // Leader line
        let line_mesh = generate_line_strip(&attachment, &label_pos, self.line_width);
        self.display.lines.merge(&line_mesh);

        // Small dot at attachment point
        let dot = generate_dot(&attachment, self.arrow_size * 0.3);
        self.display.arrows.merge(&dot);

        // Text label
        let label = generate_flat_text_label(text, &label_pos, self.text_scale);
        self.display.labels.merge(&label);

        self.display.annotation_count += 1;
        self
    }

    /// Add a diameter/radius annotation.
    ///
    /// Generates:
    /// - A dimension line across the diameter (or from center to edge for radius)
    /// - Arrow heads
    /// - A text label at `label_pos`
    pub fn add_diameter(
        mut self,
        center: Point3d,
        edge_point: Point3d,
        label_pos: Point3d,
        text: &str,
        is_radius: bool,
    ) -> Self {
        if is_radius {
            // Radius: center → edge
            let line_mesh = generate_line_strip(&center, &edge_point, self.line_width);
            self.display.lines.merge(&line_mesh);
            let arrow = generate_arrow_head(&edge_point, &center, self.arrow_size);
            self.display.arrows.merge(&arrow);
        } else {
            // Diameter: opposite_edge → center → edge
            let opposite = Point3d::new(
                2.0 * center.x - edge_point.x,
                2.0 * center.y - edge_point.y,
                2.0 * center.z - edge_point.z,
            );
            let line_mesh = generate_line_strip(&opposite, &edge_point, self.line_width);
            self.display.lines.merge(&line_mesh);
            let arrow1 = generate_arrow_head(&edge_point, &center, self.arrow_size);
            let arrow2 = generate_arrow_head(&opposite, &center, self.arrow_size);
            self.display.arrows.merge(&arrow1);
            self.display.arrows.merge(&arrow2);
        }

        // Leader to label position
        let ext = generate_line_strip(&edge_point, &label_pos, self.line_width * 0.5);
        self.display.lines.merge(&ext);

        // Text label
        let label = generate_flat_text_label(text, &label_pos, self.text_scale);
        self.display.labels.merge(&label);

        self.display.annotation_count += 1;
        self
    }

    /// Add a tolerance annotation (e.g., GD&T feature control frame).
    ///
    /// Generates:
    /// - A leader line from the feature to the tolerance frame
    /// - A rectangular frame containing the tolerance text
    /// - The tolerance text
    pub fn add_tolerance(
        mut self,
        attachment: Point3d,
        label_pos: Point3d,
        tolerance_text: &str,
    ) -> Self {
        // Leader line
        let line_mesh = generate_line_strip(&attachment, &label_pos, self.line_width);
        self.display.lines.merge(&line_mesh);

        // Tolerance frame (rectangular border)
        let frame = generate_tolerance_frame(&label_pos, tolerance_text, self.text_scale, self.line_width);
        self.display.lines.merge(&frame);

        // Text label
        let label = generate_flat_text_label(tolerance_text, &label_pos, self.text_scale);
        self.display.labels.merge(&label);

        self.display.annotation_count += 1;
        self
    }

    /// Build the final PMI display mesh.
    pub fn build(self) -> PmiDisplayMesh {
        self.display
    }
}

impl Default for PmiDisplayBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Internal mesh generation helpers
// ============================================================

/// Generate a thin rectangular strip (line) between two 3D points.
///
/// The strip is perpendicular to the line direction, with the given width.
/// This produces 2 triangles (a quad) for each line segment.
fn generate_line_strip(start: &Point3d, end: &Point3d, width: f64) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let dz = end.z - start.z;
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-10 {
        return mesh;
    }

    // Direction along the line
    let dir_x = dx / len;
    let dir_y = dy / len;
    let dir_z = dz / len;

    // Find a perpendicular direction for the strip width.
    // Use cross product with a non-parallel axis.
    let (perp_x, perp_y, perp_z) = if dir_y.abs() < 0.9 {
        // Cross with Y-axis
        let px = dir_z;
        let py = 0.0;
        let pz = -dir_x;
        let pl = (px * px + py * py + pz * pz).sqrt();
        (px / pl, py / pl, pz / pl)
    } else {
        // Cross with X-axis
        let px = 0.0;
        let py = -dir_z;
        let pz = dir_y;
        let pl = (px * px + py * py + pz * pz).sqrt();
        (px / pl, py / pl, pz / pl)
    };

    let hw = width * 0.5;

    // Four corners of the strip
    let v0 = mesh.add_vertex(Point3d::new(
        start.x + perp_x * hw,
        start.y + perp_y * hw,
        start.z + perp_z * hw,
    ));
    let v1 = mesh.add_vertex(Point3d::new(
        start.x - perp_x * hw,
        start.y - perp_y * hw,
        start.z - perp_z * hw,
    ));
    let v2 = mesh.add_vertex(Point3d::new(
        end.x - perp_x * hw,
        end.y - perp_y * hw,
        end.z - perp_z * hw,
    ));
    let v3 = mesh.add_vertex(Point3d::new(
        end.x + perp_x * hw,
        end.y + perp_y * hw,
        end.z + perp_z * hw,
    ));

    mesh.add_triangle(v0, v1, v2);
    mesh.add_triangle(v0, v2, v3);

    mesh
}

/// Generate an arrow head at `tip` pointing towards `from` (base direction).
///
/// The arrow is a flat triangle in the plane defined by the tip-to-from
/// direction and a perpendicular axis.
fn generate_arrow_head(tip: &Point3d, from: &Point3d, size: f64) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let dx = from.x - tip.x;
    let dy = from.y - tip.y;
    let dz = from.z - tip.z;
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-10 {
        return mesh;
    }

    // Direction from tip towards base
    let dir_x = dx / len;
    let dir_y = dy / len;
    let dir_z = dz / len;

    // Perpendicular direction
    let (perp_x, perp_y, perp_z) = if dir_y.abs() < 0.9 {
        let px = dir_z;
        let py = 0.0;
        let pz = -dir_x;
        let pl = (px * px + py * py + pz * pz).sqrt();
        (px / pl, py / pl, pz / pl)
    } else {
        let px = 0.0;
        let py = -dir_z;
        let pz = dir_y;
        let pl = (px * px + py * py + pz * pz).sqrt();
        (px / pl, py / pl, pz / pl)
    };

    // Arrow head: tip point + two base points
    let v0 = mesh.add_vertex(*tip);
    let v1 = mesh.add_vertex(Point3d::new(
        tip.x + dir_x * size + perp_x * size * 0.4,
        tip.y + dir_y * size + perp_y * size * 0.4,
        tip.z + dir_z * size + perp_z * size * 0.4,
    ));
    let v2 = mesh.add_vertex(Point3d::new(
        tip.x + dir_x * size - perp_x * size * 0.4,
        tip.y + dir_y * size - perp_y * size * 0.4,
        tip.z + dir_z * size - perp_z * size * 0.4,
    ));

    mesh.add_triangle(v0, v1, v2);

    mesh
}

/// Generate a small dot (octahedron) at the given position.
fn generate_dot(center: &Point3d, radius: f64) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let r = radius;

    // 6 vertices: +X, -X, +Y, -Y, +Z, -Z
    let vpx = mesh.add_vertex(Point3d::new(center.x + r, center.y, center.z));
    let vmx = mesh.add_vertex(Point3d::new(center.x - r, center.y, center.z));
    let vpy = mesh.add_vertex(Point3d::new(center.x, center.y + r, center.z));
    let vmy = mesh.add_vertex(Point3d::new(center.x, center.y - r, center.z));
    let vpz = mesh.add_vertex(Point3d::new(center.x, center.y, center.z + r));
    let vmz = mesh.add_vertex(Point3d::new(center.x, center.y, center.z - r));

    // 8 triangles (2 per quadrant)
    // +X face
    mesh.add_triangle(vpx, vpy, vpz);
    mesh.add_triangle(vpx, vpz, vmy);
    mesh.add_triangle(vpx, vmy, vmz);
    mesh.add_triangle(vpx, vmz, vpy);
    // -X face
    mesh.add_triangle(vmx, vpz, vpy);
    mesh.add_triangle(vmx, vmy, vpz);
    mesh.add_triangle(vmx, vmz, vmy);
    mesh.add_triangle(vmx, vpy, vmz);

    mesh
}

/// Generate a flat text label at a 3D position.
///
/// Uses the `text3d` module to create 3D text geometry, then
/// translates it to the given position. The text is flat (zero depth)
/// for PMI labels, making them lightweight and always-facing.
fn generate_flat_text_label(text: &str, position: &Point3d, scale: f64) -> TriangleMesh {
    // Generate text contours (2D polygon outlines)
    let contours = text3d::generate_text_contours(text, scale * 0.5, scale * 0.15);

    let mut mesh = TriangleMesh::new();

    for contour in &contours {
        if contour.len() < 3 {
            continue;
        }

        // Add contour points as vertices, offset to the 3D position
        let base_idx = mesh.vertices.len() as u32;
        for &(x, y) in contour {
            mesh.add_vertex(Point3d::new(
                position.x + x,
                position.y + y,
                position.z,
            ));
        }

        // Simple ear-clipping for the contour
        let n = contour.len() as u32;
        if n == 3 {
            mesh.add_triangle(base_idx, base_idx + 1, base_idx + 2);
        } else {
            // Fan triangulation (works for convex contours)
            for i in 1..n - 1 {
                mesh.add_triangle(base_idx, base_idx + i, base_idx + i + 1);
            }
        }
    }

    mesh
}

/// Generate a rectangular tolerance frame around the label position.
///
/// The frame is a rectangle that bounds the text, drawn as four line strips.
fn generate_tolerance_frame(
    label_pos: &Point3d,
    text: &str,
    scale: f64,
    line_width: f64,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Estimate text bounding box from character count
    let char_width = scale * 0.5;
    let char_height = scale * 0.5 * 1.4;
    let text_width = text.len() as f64 * char_width;
    let padding = char_height * 0.3;

    let x0 = label_pos.x - padding;
    let x1 = label_pos.x + text_width + padding;
    let y0 = label_pos.y - padding;
    let y1 = label_pos.y + char_height + padding;
    let z = label_pos.z;

    // Four sides of the rectangle
    let p0 = Point3d::new(x0, y0, z);
    let p1 = Point3d::new(x1, y0, z);
    let p2 = Point3d::new(x1, y1, z);
    let p3 = Point3d::new(x0, y1, z);

    mesh.merge(&generate_line_strip(&p0, &p1, line_width));
    mesh.merge(&generate_line_strip(&p1, &p2, line_width));
    mesh.merge(&generate_line_strip(&p2, &p3, line_width));
    mesh.merge(&generate_line_strip(&p3, &p0, line_width));

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pmi_builder_empty() {
        let display = PmiDisplayBuilder::new().build();
        assert_eq!(display.annotation_count, 0);
        assert!(display.lines.vertices.is_empty());
        assert!(display.labels.vertices.is_empty());
        assert!(display.arrows.vertices.is_empty());
    }

    #[test]
    fn test_pmi_builder_dimension() {
        let display = PmiDisplayBuilder::new()
            .text_scale(1.0)
            .add_dimension(
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(10.0, 0.0, 0.0),
                Point3d::new(5.0, 3.0, 0.0),
                "10",
            )
            .build();

        assert_eq!(display.annotation_count, 1);
        assert!(!display.lines.vertices.is_empty(), "Should have dimension lines");
        assert!(!display.arrows.vertices.is_empty(), "Should have arrow heads");
    }

    #[test]
    fn test_pmi_builder_leader() {
        let display = PmiDisplayBuilder::new()
            .add_leader_line(
                Point3d::new(5.0, 0.0, 0.0),
                Point3d::new(5.0, 5.0, 0.0),
                "R5",
            )
            .build();

        assert_eq!(display.annotation_count, 1);
        assert!(!display.lines.vertices.is_empty());
        assert!(!display.arrows.vertices.is_empty(), "Should have dot at attachment");
    }

    #[test]
    fn test_pmi_builder_diameter() {
        let display = PmiDisplayBuilder::new()
            .add_diameter(
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(5.0, 0.0, 0.0),
                Point3d::new(3.0, 3.0, 0.0),
                "D10",
                false,
            )
            .build();

        assert_eq!(display.annotation_count, 1);
        assert!(!display.lines.vertices.is_empty());
    }

    #[test]
    fn test_pmi_builder_radius() {
        let display = PmiDisplayBuilder::new()
            .add_diameter(
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(5.0, 0.0, 0.0),
                Point3d::new(3.0, 3.0, 0.0),
                "R5",
                true,
            )
            .build();

        assert_eq!(display.annotation_count, 1);
        assert!(!display.lines.vertices.is_empty());
    }

    #[test]
    fn test_pmi_builder_tolerance() {
        let display = PmiDisplayBuilder::new()
            .add_tolerance(
                Point3d::new(5.0, 0.0, 0.0),
                Point3d::new(5.0, 5.0, 0.0),
                "0.05",
            )
            .build();

        assert_eq!(display.annotation_count, 1);
        assert!(!display.lines.vertices.is_empty(), "Should have frame and leader");
    }

    #[test]
    fn test_pmi_combined_mesh() {
        let display = PmiDisplayBuilder::new()
            .add_dimension(
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(10.0, 0.0, 0.0),
                Point3d::new(5.0, 3.0, 0.0),
                "10",
            )
            .add_leader_line(
                Point3d::new(5.0, 0.0, 0.0),
                Point3d::new(5.0, 5.0, 0.0),
                "R5",
            )
            .build();

        let combined = display.combined();
        assert!(!combined.vertices.is_empty());
        assert!(!combined.triangles.is_empty());
    }

    #[test]
    fn test_line_strip_degenerate() {
        // Zero-length line should produce empty mesh
        let mesh = generate_line_strip(
            &Point3d::new(1.0, 2.0, 3.0),
            &Point3d::new(1.0, 2.0, 3.0),
            0.1,
        );
        assert!(mesh.vertices.is_empty());
    }

    #[test]
    fn test_dot() {
        let mesh = generate_dot(&Point3d::new(0.0, 0.0, 0.0), 0.5);
        assert_eq!(mesh.vertices.len(), 6); // Octahedron has 6 vertices
        assert_eq!(mesh.triangles.len(), 8); // 8 faces
    }
}
