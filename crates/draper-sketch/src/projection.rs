// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 3D-to-2D Projection for Sketch Integration.
//!
//! Per MASTER_PLAN_100.md Phase 1.2: Project 3D edges from adjacent
//! B-Rep faces onto a 2D sketch plane, creating SketchEntity::Line/Arc.
//!
//! This enables "Use Geometry" / "Convert Geometry" functionality
//! where edges of existing 3D solids are projected onto the sketch
//! plane and become reference entities.

use crate::{Sketch2d, SketchEntity, Constraint};
use draper_geometry::{Point3d, Vec3d, Direction3d, Curve3d, Line as GeoLine, Circle as GeoCircle, Arc as GeoArc};

// ============================================================
// Error types
// ============================================================

#[derive(Debug, Clone, thiserror::Error)]
pub enum ProjectionError {
    #[error("Edge curve is degenerate or empty")]
    DegenerateEdge,

    #[error("Edge is parallel to sketch plane (no projection)")]
    ParallelToPlane,

    #[error("Invalid plane normal: zero vector")]
    InvalidNormal,
}

// ============================================================
// Sketch Plane
// ============================================================

/// A 2D sketch plane in 3D space, defined by an origin and normal.
#[derive(Debug, Clone)]
pub struct SketchPlane {
    /// Origin point of the plane.
    pub origin: Point3d,
    /// Normal direction of the plane.
    pub normal: Direction3d,
    /// U-axis (in-plane, perpendicular to normal).
    pub u_axis: Direction3d,
    /// V-axis (in-plane, perpendicular to normal and U).
    pub v_axis: Direction3d,
}

impl SketchPlane {
    /// Create a sketch plane from origin and normal.
    pub fn new(origin: Point3d, normal: Direction3d) -> Result<Self, ProjectionError> {
        let n = Vec3d::new(normal.x, normal.y, normal.z);
        let len = (n.x * n.x + n.y * n.y + n.z * n.z).sqrt();
        if len < 1e-15 {
            return Err(ProjectionError::InvalidNormal);
        }

        // Build orthonormal basis: pick seed not parallel to normal
        let seed = if n.x.abs() < 0.9 {
            Vec3d::new(1.0, 0.0, 0.0)
        } else {
            Vec3d::new(0.0, 1.0, 0.0)
        };

        let u = Vec3d::new(
            n.y * seed.z - n.z * seed.y,
            n.z * seed.x - n.x * seed.z,
            n.x * seed.y - n.y * seed.x,
        );
        let u_len = (u.x * u.x + u.y * u.y + u.z * u.z).sqrt();
        let u_dir = Direction3d::new(u.x / u_len, u.y / u_len, u.z / u_len)
            .ok_or(ProjectionError::InvalidNormal)?;

        let v = Vec3d::new(
            n.y * u_dir.z - n.z * u_dir.y,
            n.z * u_dir.x - n.x * u_dir.z,
            n.x * u_dir.y - n.y * u_dir.x,
        );
        let v_dir = Direction3d::new(v.x, v.y, v.z)
            .ok_or(ProjectionError::InvalidNormal)?;

        Ok(Self {
            origin,
            normal,
            u_axis: u_dir,
            v_axis: v_dir,
        })
    }

    /// Create XY plane (Z=0).
    pub fn xy() -> Self {
        Self::new(Point3d::ORIGIN, Direction3d::new(0.0, 0.0, 1.0).unwrap())
            .expect("XY plane should always be valid")
    }

    /// Create XZ plane (Y=0).
    pub fn xz() -> Self {
        Self::new(Point3d::ORIGIN, Direction3d::new(0.0, 1.0, 0.0).unwrap())
            .expect("XZ plane should always be valid")
    }

    /// Create YZ plane (X=0).
    pub fn yz() -> Self {
        Self::new(Point3d::ORIGIN, Direction3d::new(1.0, 0.0, 0.0).unwrap())
            .expect("YZ plane should always be valid")
    }

    /// Project a 3D point onto this plane, returning 2D (u, v) coordinates.
    pub fn project_point(&self, p: &Point3d) -> (f64, f64) {
        let dx = p.x - self.origin.x;
        let dy = p.y - self.origin.y;
        let dz = p.z - self.origin.z;
        let u = dx * self.u_axis.x + dy * self.u_axis.y + dz * self.u_axis.z;
        let v = dx * self.v_axis.x + dy * self.v_axis.y + dz * self.v_axis.z;
        (u, v)
    }

    /// Unproject 2D (u, v) coordinates back to 3D point.
    pub fn unproject_point(&self, u: f64, v: f64) -> Point3d {
        Point3d::new(
            self.origin.x + u * self.u_axis.x + v * self.v_axis.x,
            self.origin.y + u * self.u_axis.y + v * self.v_axis.y,
            self.origin.z + u * self.u_axis.z + v * self.v_axis.z,
        )
    }
}

// ============================================================
// Edge Projection
// ============================================================

/// Result of projecting a 3D edge onto a sketch plane.
#[derive(Debug, Clone)]
pub enum ProjectedEdge {
    /// A straight line in 2D.
    Line { start: (f64, f64), end: (f64, f64) },
    /// An arc in 2D (center, radius, start_angle, end_angle).
    Arc { center: (f64, f64), radius: f64, start_angle: f64, end_angle: f64 },
}

/// Project a 3D curve (edge) onto a sketch plane.
///
/// Per MASTER_PLAN_100.md Phase 1.2: converts 3D edges from B-Rep
/// faces into 2D sketch entities.
pub fn project_curve(
    curve: &Curve3d,
    plane: &SketchPlane,
    samples: usize,
) -> Result<Vec<ProjectedEdge>, ProjectionError> {
    if samples < 2 {
        return Err(ProjectionError::DegenerateEdge);
    }

    match curve {
        Curve3d::Line(line) => project_line(line, plane),
        Curve3d::Circle(circle) => project_circle(circle, plane, samples),
        Curve3d::Arc(arc) => project_arc(arc, plane, samples),
        _ => {
            // For NURBS and other curves, sample points and fit line segments
            project_generic(curve, plane, samples)
        }
    }
}

/// Project a 3D line onto the sketch plane.
fn project_line(line: &GeoLine, plane: &SketchPlane) -> Result<Vec<ProjectedEdge>, ProjectionError> {
    let p1 = line.point_at(0.0);
    let p2 = line.point_at(1.0);
    let (u1, v1) = plane.project_point(&p1);
    let (u2, v2) = plane.project_point(&p2);

    let du = u2 - u1;
    let dv = v2 - v1;
    if (du * du + dv * dv).sqrt() < 1e-10 {
        return Err(ProjectionError::DegenerateEdge);
    }

    Ok(vec![ProjectedEdge::Line {
        start: (u1, v1),
        end: (u2, v2),
    }])
}

/// Project a 3D circle onto the sketch plane.
fn project_circle(circle: &GeoCircle, plane: &SketchPlane, samples: usize) -> Result<Vec<ProjectedEdge>, ProjectionError> {
    // Check if circle is parallel to plane (projects as circle) or
    // perpendicular (projects as line segment).
    let circle_normal = Vec3d::new(circle.normal.x, circle.normal.y, circle.normal.z);
    let plane_normal = Vec3d::new(plane.normal.x, plane.normal.y, plane.normal.z);
    let dot = circle_normal.x * plane_normal.x
        + circle_normal.y * plane_normal.y
        + circle_normal.z * plane_normal.z;

    if dot.abs() > 0.9999 {
        // Circle is parallel to plane → projects as circle
        let center_2d = plane.project_point(&circle.center);
        Ok(vec![ProjectedEdge::Arc {
            center: center_2d,
            radius: circle.radius,
            start_angle: 0.0,
            end_angle: 2.0 * std::f64::consts::PI,
        }])
    } else {
        // Circle is tilted → sample and project as line segments
        project_generic(&Curve3d::Circle(circle.clone()), plane, samples)
    }
}

/// Project a 3D arc onto the sketch plane.
fn project_arc(arc: &GeoArc, plane: &SketchPlane, samples: usize) -> Result<Vec<ProjectedEdge>, ProjectionError> {
    // For simplicity, sample the arc and create line segments
    project_generic(&Curve3d::Arc(arc.clone()), plane, samples)
}

/// Project any 3D curve by sampling points and creating line segments.
fn project_generic(
    curve: &Curve3d,
    plane: &SketchPlane,
    samples: usize,
) -> Result<Vec<ProjectedEdge>, ProjectionError> {
    let mut points_2d: Vec<(f64, f64)> = Vec::with_capacity(samples);
    for i in 0..samples {
        let t = i as f64 / (samples - 1) as f64;
        let p3d = curve.point_at(t);
        points_2d.push(plane.project_point(&p3d));
    }

    let mut edges = Vec::new();
    for i in 0..points_2d.len() - 1 {
        let (u1, v1) = points_2d[i];
        let (u2, v2) = points_2d[i + 1];
        let du = u2 - u1;
        let dv = v2 - v1;
        if (du * du + dv * dv).sqrt() > 1e-10 {
            edges.push(ProjectedEdge::Line {
                start: (u1, v1),
                end: (u2, v2),
            });
        }
    }

    if edges.is_empty() {
        Err(ProjectionError::DegenerateEdge)
    } else {
        Ok(edges)
    }
}

// ============================================================
// Sketch Integration
// ============================================================

/// Project a 3D edge and add it to a Sketch2d as entities.
///
/// Per MASTER_PLAN_100.md Phase 1.2: "Use Geometry" / "Convert Geometry".
/// Projects 3D edges from B-Rep faces onto the sketch plane and creates
/// corresponding 2D sketch entities (points, lines, arcs).
///
/// Returns the IDs of created entities.
pub fn project_edge_to_sketch(
    curve: &Curve3d,
    plane: &SketchPlane,
    sketch: &mut Sketch2d,
    samples: usize,
) -> Result<Vec<u64>, ProjectionError> {
    let edges = project_curve(curve, plane, samples)?;

    let mut ids = Vec::new();
    for edge in edges {
        match edge {
            ProjectedEdge::Line { start, end } => {
                let p1 = sketch.add_point(start.0, start.1);
                let p2 = sketch.add_point(end.0, end.1);
                let line_id = sketch.add_line(p1, p2);
                ids.push(line_id);
            }
            ProjectedEdge::Arc { center, radius, start_angle, end_angle } => {
                let c = sketch.add_point(center.0, center.1);
                // For arc, we create start and end points and a line
                // (full arc support will be added later)
                let sx = center.0 + radius * start_angle.cos();
                let sy = center.1 + radius * start_angle.sin();
                let ex = center.0 + radius * end_angle.cos();
                let ey = center.1 + radius * end_angle.sin();
                let p1 = sketch.add_point(sx, sy);
                let p2 = sketch.add_point(ex, ey);
                let line_id = sketch.add_line(p1, p2);
                ids.push(line_id);
            }
        }
    }

    Ok(ids)
}

/// Project multiple 3D edges (e.g., all edges of a face) onto the sketch.
pub fn project_edges_to_sketch(
    curves: &[Curve3d],
    plane: &SketchPlane,
    sketch: &mut Sketch2d,
    samples: usize,
) -> Vec<u64> {
    let mut all_ids = Vec::new();
    for curve in curves {
        match project_edge_to_sketch(curve, plane, sketch, samples) {
            Ok(ids) => all_ids.extend(ids),
            Err(e) => {
                log::debug!("Edge projection failed: {}", e);
            }
        }
    }
    all_ids
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ---------- SketchPlane tests ----------

    #[test]
    fn test_plane_xy_creation() {
        let plane = SketchPlane::xy();
        assert_relative_eq!(plane.normal.z, 1.0);
    }

    #[test]
    fn test_plane_invalid_normal() {
        let result = SketchPlane::new(Point3d::ORIGIN, Direction3d::new(0.0, 0.0, 0.0));
        assert!(result.is_err());
    }

    #[test]
    fn test_plane_project_point() {
        let plane = SketchPlane::xy();
        let p = Point3d::new(5.0, 3.0, 10.0); // Z is dropped
        let (u, v) = plane.project_point(&p);
        assert_relative_eq!(u, 5.0);
        assert_relative_eq!(v, 3.0);
    }

    #[test]
    fn test_plane_unproject_point() {
        let plane = SketchPlane::xy();
        let p = plane.unproject_point(5.0, 3.0);
        assert_relative_eq!(p.x, 5.0);
        assert_relative_eq!(p.y, 3.0);
        assert_relative_eq!(p.z, 0.0);
    }

    #[test]
    fn test_plane_xz() {
        let plane = SketchPlane::xz();
        let p = Point3d::new(5.0, 10.0, 3.0); // Y is dropped
        let (u, v) = plane.project_point(&p);
        assert_relative_eq!(u, 5.0);
        assert_relative_eq!(v, 3.0);
    }

    #[test]
    fn test_plane_yz() {
        let plane = SketchPlane::yz();
        let p = Point3d::new(10.0, 5.0, 3.0); // X is dropped
        let (u, v) = plane.project_point(&p);
        assert_relative_eq!(u, 5.0);
        assert_relative_eq!(v, 3.0);
    }

    #[test]
    fn test_plane_arbitrary_normal() {
        let plane = SketchPlane::new(
            Point3d::ORIGIN,
            Direction3d::new(1.0, 1.0, 1.0).unwrap(),
        ).unwrap();
        // Project origin → (0, 0)
        let (u, v) = plane.project_point(&Point3d::ORIGIN);
        assert_relative_eq!(u, 0.0);
        assert_relative_eq!(v, 0.0);
    }

    // ---------- Line Projection tests ----------

    #[test]
    fn test_project_line_xy() {
        let plane = SketchPlane::xy();
        let line = GeoLine::through_points(
            Point3d::new(0.0, 0.0, 5.0),
            Point3d::new(10.0, 5.0, 5.0),
        ).unwrap();
        let edges = project_curve(&Curve3d::Line(line), &plane, 10).unwrap();
        assert_eq!(edges.len(), 1);
        match &edges[0] {
            ProjectedEdge::Line { start, end } => {
                assert_relative_eq!(start.0, 0.0);
                assert_relative_eq!(start.1, 0.0);
                assert_relative_eq!(end.0, 10.0);
                assert_relative_eq!(end.1, 5.0);
            }
            _ => panic!("Expected Line"),
        }
    }

    #[test]
    fn test_project_line_degenerate() {
        let plane = SketchPlane::xy();
        // Line along Z axis — projects to a single point
        let line = GeoLine::through_points(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(0.0, 0.0, 10.0),
        ).unwrap();
        let result = project_curve(&Curve3d::Line(line), &plane, 10);
        assert!(result.is_err());
    }

    #[test]
    fn test_project_line_on_xz_plane() {
        let plane = SketchPlane::xz();
        let line = GeoLine::through_points(
            Point3d::new(0.0, 5.0, 0.0),
            Point3d::new(10.0, 5.0, 20.0),
        ).unwrap();
        let edges = project_curve(&Curve3d::Line(line), &plane, 10).unwrap();
        assert_eq!(edges.len(), 1);
        match &edges[0] {
            ProjectedEdge::Line { start, end } => {
                assert_relative_eq!(start.0, 0.0);
                assert_relative_eq!(end.0, 10.0);
            }
            _ => panic!("Expected Line"),
        }
    }

    // ---------- Circle Projection tests ----------

    #[test]
    fn test_project_circle_parallel() {
        let plane = SketchPlane::xy();
        let circle = GeoCircle::new_xy(Point3d::ORIGIN, 5.0);
        let edges = project_curve(&Curve3d::Circle(circle), &plane, 20).unwrap();
        assert_eq!(edges.len(), 1);
        match &edges[0] {
            ProjectedEdge::Arc { radius, .. } => {
                assert_relative_eq!(*radius, 5.0);
            }
            _ => panic!("Expected Arc"),
        }
    }

    #[test]
    fn test_project_circle_tilted() {
        let plane = SketchPlane::xy();
        // Circle in XZ plane (perpendicular to XY)
        let circle = GeoCircle::new(
            Point3d::ORIGIN,
            Direction3d::new(0.0, 1.0, 0.0).unwrap(),
            5.0,
        );
        let edges = project_curve(&Curve3d::Circle(circle), &plane, 20).unwrap();
        // Tilted circle projects as line segments
        assert!(edges.len() > 1);
    }

    // ---------- Generic Projection tests ----------

    #[test]
    fn test_project_generic_curve() {
        let plane = SketchPlane::xy();
        // Use a circle as generic curve (tilted → line segments)
        let circle = GeoCircle::new(
            Point3d::ORIGIN,
            Direction3d::new(1.0, 1.0, 0.0).unwrap(),
            5.0,
        );
        let edges = project_curve(&Curve3d::Circle(circle), &plane, 30).unwrap();
        assert!(edges.len() > 1);
    }

    #[test]
    fn test_project_too_few_samples() {
        let plane = SketchPlane::xy();
        let line = GeoLine::through_points(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(10.0, 0.0, 0.0),
        ).unwrap();
        let result = project_curve(&Curve3d::Line(line), &plane, 1);
        assert!(result.is_err());
    }

    // ---------- Sketch Integration tests ----------

    #[test]
    fn test_project_edge_to_sketch_line() {
        let plane = SketchPlane::xy();
        let mut sketch = Sketch2d::new();
        let line = GeoLine::through_points(
            Point3d::new(0.0, 0.0, 5.0),
            Point3d::new(20.0, 10.0, 5.0),
        ).unwrap();
        let ids = project_edge_to_sketch(&Curve3d::Line(line), &plane, &mut sketch, 10).unwrap();
        assert_eq!(ids.len(), 1);
        assert!(sketch.entities.len() >= 3); // 2 points + 1 line
    }

    #[test]
    fn test_project_edge_to_sketch_circle() {
        let plane = SketchPlane::xy();
        let mut sketch = Sketch2d::new();
        let circle = GeoCircle::new_xy(Point3d::ORIGIN, 10.0);
        let ids = project_edge_to_sketch(&Curve3d::Circle(circle), &plane, &mut sketch, 20).unwrap();
        assert_eq!(ids.len(), 1); // Arc creates one entity
    }

    #[test]
    fn test_project_edges_multiple() {
        let plane = SketchPlane::xy();
        let mut sketch = Sketch2d::new();
        let curves = vec![
            Curve3d::Line(GeoLine::through_points(
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(10.0, 0.0, 0.0),
            ).unwrap()),
            Curve3d::Line(GeoLine::through_points(
                Point3d::new(10.0, 0.0, 0.0),
                Point3d::new(10.0, 10.0, 0.0),
            ).unwrap()),
            Curve3d::Line(GeoLine::through_points(
                Point3d::new(10.0, 10.0, 0.0),
                Point3d::new(0.0, 10.0, 0.0),
            ).unwrap()),
            Curve3d::Line(GeoLine::through_points(
                Point3d::new(0.0, 10.0, 0.0),
                Point3d::new(0.0, 0.0, 0.0),
            ).unwrap()),
        ];
        let ids = project_edges_to_sketch(&curves, &plane, &mut sketch, 10);
        assert_eq!(ids.len(), 4);
    }

    #[test]
    fn test_project_edge_degenerate_skipped() {
        let plane = SketchPlane::xy();
        let mut sketch = Sketch2d::new();
        // Line along Z → degenerate, should be skipped
        let line = GeoLine::through_points(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(0.0, 0.0, 10.0),
        ).unwrap();
        let result = project_edge_to_sketch(&Curve3d::Line(line), &plane, &mut sketch, 10);
        assert!(result.is_err());
    }
}
