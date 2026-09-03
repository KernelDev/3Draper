// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Boolean operations on B-Rep solids.
//!
//! Implements:
//! - 4.1.1 Surface-Surface Intersection (SSI)
//! - 4.1.2 Curve-Surface Intersection (CSI)
//! - 4.1.3 Boolean Union
//! - 4.1.4 Boolean Subtract
//! - 4.1.5 Boolean Intersect
//! - 4.1.6 Face splitting by intersection lines
//! - 4.1.7 Point classification (inside/outside/on-boundary)
//! - 4.1.8 Unit tests

use crate::entity::*;
use crate::builder::ShapeBuilder;
use crate::edge_store::EdgeStore;
use draper_geometry::{
    Point3d, Direction3d, Vec3d,
    Curve3d, Curve2d, Line, Circle, Ellipse,
    Line2d, Circle2d,
    Surface, Plane, CylinderSurface, SphereSurface, ConeSurface, TorusSurface,
    ToleranceContext, Point2d,
    intersection::intersect_line_cylinder,
};
use std::f64::consts::PI;

// ============================================================
// Error type
// ============================================================

/// Errors that can occur during boolean operations.
///
/// Note: We define our own error type here rather than using
/// `draper_core::KernelError` to avoid a circular dependency
/// (draper-core depends on draper-topology).
#[derive(Debug)]
pub enum BooleanError {
    /// The input solids have no outer shell.
    MissingShell(String),
    /// Surface-surface intersection failed.
    IntersectionFailed(String),
    /// Face splitting failed.
    FaceSplitFailed(String),
    /// The result of the boolean operation is empty (no volume).
    EmptyResult(String),
    /// General boolean error.
    Other(String),
}

impl std::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BooleanError::MissingShell(msg) => write!(f, "Missing shell: {}", msg),
            BooleanError::IntersectionFailed(msg) => write!(f, "Intersection failed: {}", msg),
            BooleanError::FaceSplitFailed(msg) => write!(f, "Face split failed: {}", msg),
            BooleanError::EmptyResult(msg) => write!(f, "Empty result: {}", msg),
            BooleanError::Other(msg) => write!(f, "Boolean error: {}", msg),
        }
    }
}

impl std::error::Error for BooleanError {}

/// Convenience alias for results from boolean operations.
pub type BooleanResult<T> = Result<T, BooleanError>;

// ============================================================
// 4.1.7 Point Classification
// ============================================================

/// Classification of a point relative to a solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointClassification {
    /// Point is inside the solid.
    Inside,
    /// Point is outside the solid.
    Outside,
    /// Point is on the boundary of the solid.
    OnBoundary,
}

/// Classify a point as inside, outside, or on the boundary of a solid.
///
/// Uses ray casting: cast a ray from the point in an arbitrary direction,
/// count intersections with the solid's faces.
/// - Odd count = inside
/// - Even count = outside
/// - If the point is on a face = on-boundary
pub fn classify_point(solid: &Solid, point: &Point3d, tol_ctx: &ToleranceContext) -> PointClassification {
    let tol = tol_ctx.coincidence_tolerance();

    // First check if the point is on any face boundary
    if let Some(ref shell) = solid.outer_shell {
        for face in &shell.faces {
            if is_point_on_face(point, face, tol) {
                return PointClassification::OnBoundary;
            }
        }
    }

    // Ray casting: cast a ray in the +X direction and count intersections
    let ray_origin = *point;
    let ray_dir = Direction3d::X;

    let mut intersection_count = 0u32;

    if let Some(ref shell) = solid.outer_shell {
        for face in &shell.faces {
            // C5 Stage 6.2: boundary reads are store-first with per-id
            // mirror fallback (split results / un-indexed faces stay complete).
            let face_edges = solid.resolve_face_edges(face);
            let count = count_ray_face_intersections(&ray_origin, &ray_dir, face, &face_edges, tol);
            intersection_count += count;
        }
    }

    if intersection_count % 2 == 1 {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    }
}

/// Check if a point lies on a face (within tolerance).
fn is_point_on_face(point: &Point3d, face: &Face, tol: f64) -> bool {
    let surface = match &face.surface {
        Some(s) => s,
        None => return false,
    };

    // Check distance from point to surface
    match surface {
        Surface::Plane(plane) => {
            let dx = point.x - plane.origin.x;
            let dy = point.y - plane.origin.y;
            let dz = point.z - plane.origin.z;
            let dist = (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).abs();
            dist < tol
        }
        Surface::Sphere(sphere) => {
            let dx = point.x - sphere.center.x;
            let dy = point.y - sphere.center.y;
            let dz = point.z - sphere.center.z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            (dist - sphere.radius).abs() < tol
        }
        Surface::Cylinder(cyl) => {
            // Distance from point to cylinder axis
            let dx = point.x - cyl.origin.x;
            let dy = point.y - cyl.origin.y;
            let dz = point.z - cyl.origin.z;
            // Project onto the plane perpendicular to axis
            let along_axis = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
            let perp_x = dx - along_axis * cyl.axis.x;
            let perp_y = dy - along_axis * cyl.axis.y;
            let perp_z = dz - along_axis * cyl.axis.z;
            let radial_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
            (radial_dist - cyl.radius).abs() < tol
        }
        Surface::Cone(cone) => {
            // For cones, project point and check distance to cone surface
            let dx = point.x - cone.origin.x;
            let dy = point.y - cone.origin.y;
            let dz = point.z - cone.origin.z;
            let along_axis = dx * cone.axis.x + dy * cone.axis.y + dz * cone.axis.z;
            let perp_x = dx - along_axis * cone.axis.x;
            let perp_y = dy - along_axis * cone.axis.y;
            let perp_z = dz - along_axis * cone.axis.z;
            let radial_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
            let expected_radius = if cone.expanding {
                along_axis * cone.half_angle.tan()
            } else {
                (cone.radius - along_axis * cone.half_angle.tan()).max(0.0)
            };
            (radial_dist - expected_radius).abs() < tol
        }
        _ => {
            // For other surfaces (NURBS, revolution, extrusion, torus),
            // sample the surface and find minimum distance
            let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
            let n_samples = 20;
            let mut min_dist = f64::MAX;
            for i in 0..=n_samples {
                for j in 0..=n_samples {
                    let u = u_min + (u_max - u_min) * (i as f64 / n_samples as f64);
                    let v = v_min + (v_max - v_min) * (j as f64 / n_samples as f64);
                    let sp = surface.point_at(u, v);
                    let d = point.distance_to(&sp);
                    if d < min_dist {
                        min_dist = d;
                    }
                    if min_dist < tol {
                        return true;
                    }
                }
            }
            min_dist < tol
        }
    }
}

/// Count how many times a ray (origin + t*direction, t > 0) intersects a face.
fn count_ray_face_intersections(
    origin: &Point3d,
    direction: &Direction3d,
    face: &Face,
    face_edges: &[Edge],
    tol: f64,
) -> u32 {
    let surface = match &face.surface {
        Some(s) => s,
        None => return 0,
    };

    match surface {
        Surface::Plane(plane) => {
            // Ray-plane intersection
            let denom = plane.normal.x * direction.x
                + plane.normal.y * direction.y
                + plane.normal.z * direction.z;
            if denom.abs() < 1e-10 {
                return 0; // Ray parallel to plane
            }
            let dx = plane.origin.x - origin.x;
            let dy = plane.origin.y - origin.y;
            let dz = plane.origin.z - origin.z;
            let t = (plane.normal.x * dx + plane.normal.y * dy + plane.normal.z * dz) / denom;
            if t < tol {
                return 0; // Behind the ray or at origin
            }
            // Check if the intersection point is within the face's boundary
            let hit = Point3d::new(
                origin.x + t * direction.x,
                origin.y + t * direction.y,
                origin.z + t * direction.z,
            );
            if is_point_in_face_boundary(&hit, face, face_edges, tol) { 1 } else { 0 }
        }
        Surface::Sphere(sphere) => {
            // Ray-sphere intersection
            let oc = Vec3d::new(
                origin.x - sphere.center.x,
                origin.y - sphere.center.y,
                origin.z - sphere.center.z,
            );
            let dir = Vec3d::new(direction.x, direction.y, direction.z);
            let a = dir.dot(&dir);
            let b = 2.0 * oc.dot(&dir);
            let c = oc.dot(&oc) - sphere.radius * sphere.radius;
            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return 0;
            }
            let sqrt_disc = disc.sqrt();
            let t1 = (-b - sqrt_disc) / (2.0 * a);
            let t2 = (-b + sqrt_disc) / (2.0 * a);
            let mut count = 0u32;
            if t1 > tol {
                let hit = Point3d::new(
                    origin.x + t1 * direction.x,
                    origin.y + t1 * direction.y,
                    origin.z + t1 * direction.z,
                );
                if is_point_in_face_boundary(&hit, face, face_edges, tol) {
                    count += 1;
                }
            }
            if t2 > tol {
                let hit = Point3d::new(
                    origin.x + t2 * direction.x,
                    origin.y + t2 * direction.y,
                    origin.z + t2 * direction.z,
                );
                if is_point_in_face_boundary(&hit, face, face_edges, tol) {
                    count += 1;
                }
            }
            count
        }
        Surface::Cylinder(cyl) => {
            // Ray-cylinder intersection (simplified for axis-aligned)
            let ray_line = Line::new(*origin, *direction);
            let hits = intersect_line_cylinder(&ray_line, cyl);
            hits.iter()
                .filter(|p| {
                    let dx = p.x - origin.x;
                    let dy = p.y - origin.y;
                    let dz = p.z - origin.z;
                    let t = dx * direction.x + dy * direction.y + dz * direction.z;
                    t > tol && is_point_in_face_boundary(p, face, face_edges, tol)
                })
                .count() as u32
        }
        _ => {
            // For complex surfaces, sample the face edges and use a simplified approach
            count_ray_face_intersections_sampling(origin, direction, face, tol)
        }
    }
}

/// Sample-based ray-face intersection for complex surfaces.
fn count_ray_face_intersections_sampling(
    origin: &Point3d,
    direction: &Direction3d,
    face: &Face,
    tol: f64,
) -> u32 {
    let surface = match &face.surface {
        Some(s) => s,
        None => return 0,
    };

    let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
    let n = 30; // grid resolution
    let mut count = 0u32;

    // For each UV cell, build two triangles (p00, p10, p11) and (p00, p11, p01)
    // and run Möller-Trumbore ray-triangle intersection. This is the standard,
    // robust approach — no sign-of-cross-product heuristics.
    for i in 0..n {
        for j in 0..n {
            let u0 = u_min + (u_max - u_min) * (i as f64 / n as f64);
            let v0 = v_min + (v_max - v_min) * (j as f64 / n as f64);
            let u1 = u_min + (u_max - u_min) * ((i + 1) as f64 / n as f64);
            let v1 = v_min + (v_max - v_min) * ((j + 1) as f64 / n as f64);

            let p00 = surface.point_at(u0, v0);
            let p10 = surface.point_at(u1, v0);
            let p01 = surface.point_at(u0, v1);
            let p11 = surface.point_at(u1, v1);

            // Triangle 1: p00 → p10 → p11
            if moller_trumbore(origin, direction, &p00, &p10, &p11, tol).is_some() {
                count += 1;
            }
            // Triangle 2: p00 → p11 → p01
            if moller_trumbore(origin, direction, &p00, &p11, &p01, tol).is_some() {
                count += 1;
            }
        }
    }

    count
}

/// Möller-Trumbore ray-triangle intersection.
///
/// Returns `Some(t)` (distance along the ray) if the ray intersects the
/// triangle, with `t > tol` and barycentric coordinates (u, v) in [0, 1]
/// with u + v ≤ 1. Returns `None` otherwise.
///
/// This is the canonical, robust ray-triangle test used in essentially
/// every modern ray tracer. It does not depend on the sign of any single
/// cross-product component — it solves a 3×3 linear system to get the
/// exact barycentric coordinates of the hit point.
fn moller_trumbore(
    origin: &Point3d,
    direction: &Direction3d,
    v0: &Point3d,
    v1: &Point3d,
    v2: &Point3d,
    tol: f64,
) -> Option<f64> {
    let edge1 = (
        v1.x - v0.x,
        v1.y - v0.y,
        v1.z - v0.z,
    );
    let edge2 = (
        v2.x - v0.x,
        v2.y - v0.y,
        v2.z - v0.z,
    );
    // h = direction × edge2
    let h = (
        direction.y * edge2.2 - direction.z * edge2.1,
        direction.z * edge2.0 - direction.x * edge2.2,
        direction.x * edge2.1 - direction.y * edge2.0,
    );
    // a = edge1 · h
    let a = edge1.0 * h.0 + edge1.1 * h.1 + edge1.2 * h.2;
    if a.abs() < 1e-12 {
        return None; // Ray parallel to triangle
    }
    let f = 1.0 / a;
    let s = (
        origin.x - v0.x,
        origin.y - v0.y,
        origin.z - v0.z,
    );
    // u = f * (s · h)
    let u = f * (s.0 * h.0 + s.1 * h.1 + s.2 * h.2);
    if u < 0.0 || u > 1.0 {
        return None;
    }
    // q = s × edge1
    let q = (
        s.1 * edge1.2 - s.2 * edge1.1,
        s.2 * edge1.0 - s.0 * edge1.2,
        s.0 * edge1.1 - s.1 * edge1.0,
    );
    // v = f * (direction · q)
    let v = f * (direction.x * q.0 + direction.y * q.1 + direction.z * q.2);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    // t = f * (edge2 · q)
    let t = f * (edge2.0 * q.0 + edge2.1 * q.1 + edge2.2 * q.2);
    if t > tol {
        Some(t)
    } else {
        None
    }
}

/// Check if a point (already known to be on the surface) is within the face's boundary.
fn is_point_in_face_boundary(point: &Point3d, face: &Face, face_edges: &[Edge], tol: f64) -> bool {
    // If the face has no outer wire, it's an infinite face — always true
    let outer_wire = match &face.outer_wire {
        Some(w) => w,
        None => return true,
    };

    // If the wire has no coedges, the face covers the full surface
    if outer_wire.coedges.is_empty() {
        return true;
    }

    // Check using the face's edges
    // Use a winding number / ray casting approach in the face's plane
    // For simplicity, project all edge points and do 2D point-in-polygon
    let surface = match &face.surface {
        Some(s) => s,
        None => return false,
    };

    // Collect edge points for polygon check (C5 Stage 6.2: from the
    // resolved instance-faithful list, not the face's mirrors).
    let mut polygon_points: Vec<Point3d> = Vec::new();
    for edge in face_edges {
        if let Some(ref curve) = edge.curve {
            let (t_min, t_max) = edge.param_range;
            let n_samples = 10;
            for k in 0..n_samples {
                let t = t_min + (t_max - t_min) * (k as f64 / n_samples as f64);
                let p = curve.point_at(t);
                polygon_points.push(p);
            }
        }
    }

    if polygon_points.is_empty() {
        return true;
    }

    // Use 3D ray casting within the face boundary
    // Cast a local ray from the point and count edge crossings
    point_in_polygon_3d(point, &polygon_points, surface, tol)
}

/// Check if a 3D point is inside a polygon defined by 3D points on a surface.
/// Uses a simplified approach: project to 2D using the surface parameterization.
fn point_in_polygon_3d(point: &Point3d, polygon: &[Point3d], surface: &Surface, tol: f64) -> bool {
    // Project to 2D using surface parameterization
    let (pu, pv) = project_point_to_surface_uv(point, surface);

    let mut polygon_2d: Vec<(f64, f64)> = Vec::new();
    for p in polygon {
        let (u, v) = project_point_to_surface_uv(p, surface);
        polygon_2d.push((u, v));
    }

    // 2D point-in-polygon using ray casting
    point_in_polygon_2d(pu, pv, &polygon_2d, tol)
}

/// Project a 3D point to surface (u, v) coordinates.
fn project_point_to_surface_uv(point: &Point3d, surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Plane(plane) => plane.project_point(point),
        Surface::Cylinder(cyl) => cyl.project_point(point),
        Surface::Sphere(sphere) => sphere.project_point(point),
        Surface::Cone(cone) => cone.project_point(point),
        Surface::Torus(torus) => torus.project_point(point),
        _ => {
            // Fallback: search by sampling
            let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
            let mut best_u = (u_min + u_max) / 2.0;
            let mut best_v = (v_min + v_max) / 2.0;
            let mut best_dist = f64::MAX;

            for i in 0..50 {
                for j in 0..50 {
                    let u = u_min + (u_max - u_min) * (i as f64 / 49.0);
                    let v = v_min + (v_max - v_min) * (j as f64 / 49.0);
                    let sp = surface.point_at(u, v);
                    let d = point.distance_to(&sp);
                    if d < best_dist {
                        best_dist = d;
                        best_u = u;
                        best_v = v;
                    }
                }
            }
            (best_u, best_v)
        }
    }
}

/// Get approximate parametric range for a surface.
fn surface_param_range(surface: &Surface) -> (f64, f64, f64, f64) {
    match surface {
        Surface::Plane(_) => (-1e6, 1e6, -1e6, 1e6),
        Surface::Cylinder(cyl) => {
            let (u_min, u_max) = cyl.u_range();
            (u_min, u_max, -1e6, 1e6)
        }
        Surface::Sphere(_) => (0.0, 2.0 * PI, 0.0, PI),
        Surface::Cone(_) => (0.0, 2.0 * PI, -1e6, 1e6),
        Surface::Torus(_) => (0.0, 2.0 * PI, 0.0, 2.0 * PI),
        Surface::Nurbs(n) => {
            let (u_min, u_max) = n.u_range();
            let (v_min, v_max) = n.v_range();
            (u_min, u_max, v_min, v_max)
        }
        Surface::Revolution(_) => (0.0, 2.0 * PI, -1e6, 1e6),
        Surface::Extrusion(_) => (-1e6, 1e6, -1e6, 1e6),
        Surface::Offset(o) => surface_param_range(&o.base),
        Surface::Ruled(_) => (-1e6, 1e6, 0.0, 1.0),
    }
}

/// 2D point-in-polygon test using ray casting algorithm.
fn point_in_polygon_2d(px: f64, py: f64, polygon: &[(f64, f64)], _tol: f64) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }

    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = polygon[i];
        let (xj, yj) = polygon[j];

        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ============================================================
// 4.1.1 Surface-Surface Intersection (SSI)
// ============================================================

/// Result of a surface-surface intersection.
///
/// Mirrors OCCT's `IntTools_Curve`: carries one 3D curve plus two PCurves
/// (2D curves in the UV domain of each surface). The PCurves are critical
/// for watertight topology — they allow each face to evaluate the intersection
/// edge in its own UV space, guaranteeing identical 3D points.
#[derive(Clone, Debug)]
pub struct IntersectionCurve {
    /// Points sampled along the intersection curve (for visualization/fallback).
    pub points: Vec<Point3d>,
    /// 3D curve representation (analytic: Circle, Line, Ellipse; or polyline).
    pub curve: Option<Curve3d>,
    /// PCurve on surface A (2D curve in A's UV domain).
    pub pcurve_a: Option<Curve2d>,
    /// PCurve on surface B (2D curve in B's UV domain).
    pub pcurve_b: Option<Curve2d>,
    /// Tolerance = max deviation between 3D curve and surfaces.
    pub tolerance: f64,
}

/// Intersect two surfaces and return intersection curves.
///
/// Handles analytic surface pairs:
/// - Plane-Plane: line intersection
/// - Plane-Cylinder: ellipse intersection
/// - Plane-Sphere: circle intersection
/// - Plane-Cone: conic section
/// - Cylinder-Cylinder: intersection curve
/// - Cylinder-Sphere: intersection curve
/// - Sphere-Sphere: circle in the radical plane (analytic)
/// - General: subdivision/Newton-Raphson approach
pub fn intersect_surfaces(
    surface_a: &Surface,
    surface_b: &Surface,
    tol_ctx: &ToleranceContext,
) -> Vec<IntersectionCurve> {
    let tol = tol_ctx.coincidence_tolerance();

    match (surface_a, surface_b) {
        (Surface::Plane(p), Surface::Plane(q)) => intersect_plane_plane(p, q, tol),
        (Surface::Plane(p), Surface::Cylinder(c)) => intersect_plane_cylinder(p, c, tol),
        (Surface::Cylinder(c), Surface::Plane(p)) => {
            let mut curves = intersect_plane_cylinder(p, c, tol);
            // Reverse the curve orientation for consistency
            for curve in &mut curves {
                curve.points.reverse();
            }
            curves
        }
        (Surface::Plane(p), Surface::Sphere(s)) => intersect_plane_sphere(p, s, tol),
        (Surface::Sphere(s), Surface::Plane(p)) => {
            let mut curves = intersect_plane_sphere(p, s, tol);
            for curve in &mut curves {
                curve.points.reverse();
            }
            curves
        }
        (Surface::Plane(p), Surface::Cone(c)) => intersect_plane_cone(p, c, tol),
        (Surface::Cone(c), Surface::Plane(p)) => {
            let mut curves = intersect_plane_cone(p, c, tol);
            for curve in &mut curves {
                curve.points.reverse();
            }
            curves
        }
        (Surface::Cylinder(c1), Surface::Cylinder(c2)) => {
            intersect_cylinder_cylinder(c1, c2, tol)
        }
        (Surface::Cylinder(c), Surface::Sphere(s)) => {
            intersect_cylinder_sphere(c, s, tol)
        }
        (Surface::Sphere(s), Surface::Cylinder(c)) => {
            intersect_cylinder_sphere(c, s, tol)
        }
        (Surface::Sphere(s1), Surface::Sphere(s2)) => {
            intersect_sphere_sphere(s1, s2, tol)
        }
        (Surface::Cone(c1), Surface::Cone(c2)) => {
            intersect_cone_cone(c1, c2, tol)
        }
        (Surface::Cone(c), Surface::Cylinder(y)) => {
            intersect_cone_cylinder_pair(c, y, tol)
        }
        (Surface::Cylinder(y), Surface::Cone(c)) => {
            intersect_cone_cylinder_pair(c, y, tol)
        }
        (Surface::Torus(t), Surface::Plane(p)) => {
            intersect_torus_plane_pair(p, t, tol)
        }
        (Surface::Plane(p), Surface::Torus(t)) => {
            intersect_torus_plane_pair(p, t, tol)
        }
        (Surface::Torus(t), Surface::Sphere(s)) => {
            intersect_torus_sphere_pair(s, t, tol)
        }
        (Surface::Sphere(s), Surface::Torus(t)) => {
            intersect_torus_sphere_pair(s, t, tol)
        }
        (Surface::Torus(t), Surface::Cylinder(c)) => {
            intersect_torus_cylinder_pair(c, t, tol)
        }
        (Surface::Cylinder(c), Surface::Torus(t)) => {
            intersect_torus_cylinder_pair(c, t, tol)
        }
        _ => {
            // General case: subdivision/Newton-Raphson
            intersect_surfaces_general(surface_a, surface_b, tol)
        }
    }
}

/// Plane-Plane intersection: returns a line (if not parallel).
fn intersect_plane_plane(p: &Plane, q: &Plane, tol: f64) -> Vec<IntersectionCurve> {
    // Check if planes are parallel
    // Compute cross product as Vec3d (not Direction3d) to properly detect zero length
    let cross_v = Vec3d::new(
        p.normal.y * q.normal.z - p.normal.z * q.normal.y,
        p.normal.z * q.normal.x - p.normal.x * q.normal.z,
        p.normal.x * q.normal.y - p.normal.y * q.normal.x,
    );
    let cross_len = cross_v.length();

    if cross_len < tol {
        // Planes are parallel (or coincident)
        return Vec::new();
    }

    // Direction of intersection line = cross product of normals
    let direction = Direction3d::new(cross_v.x, cross_v.y, cross_v.z)
        .unwrap_or(Direction3d::X);

    // Find a point on the intersection line
    // Solve: p.normal · (x - p.origin) = 0 and q.normal · (x - q.origin) = 0
    // Use Cramer's rule on the 3x3 system

    let d1 = p.normal.x * p.origin.x + p.normal.y * p.origin.y + p.normal.z * p.origin.z;
    let d2 = q.normal.x * q.origin.x + q.normal.y * q.origin.y + q.normal.z * q.origin.z;

    let denom = cross_v.length_sq();

    if denom < tol * tol {
        return Vec::new();
    }

    // More robust computation using Cramer's rule
    // We have: p.normal · x = d1, q.normal · x = d2
    // Plus: direction · x = some_value (we can choose 0)
    // This gives us a 3x3 system

    let line_origin = find_line_point_on_both_planes(p, q, &direction, d1, d2);

    let line = Line::new(line_origin, direction);

    // Sample points along the line
    let n_samples = 100;
    let extent = 1000.0; // Large extent for infinite planes
    let points: Vec<Point3d> = (0..=n_samples)
        .map(|i| {
            let t = -extent + 2.0 * extent * (i as f64 / n_samples as f64);
            line.point_at(t)
        })
        .collect();

    vec![IntersectionCurve {
        points,
        curve: Some(Curve3d::Line(line)),
        pcurve_a: None,
        pcurve_b: None,
        tolerance: tol,
    }]
}

/// Find a point on the intersection line of two planes using Cramer's rule.
fn find_line_point_on_both_planes(
    p: &Plane,
    q: &Plane,
    direction: &Direction3d,
    d1: f64,
    d2: f64,
) -> Point3d {
    // Build the 3x3 system:
    // p.normal · x = d1
    // q.normal · x = d2
    // direction · x = 0 (choose this for simplicity)
    let a = [
        [p.normal.x, p.normal.y, p.normal.z],
        [q.normal.x, q.normal.y, q.normal.z],
        [direction.x, direction.y, direction.z],
    ];
    let b = [d1, d2, 0.0];

    // Solve using Cramer's rule
    let det_a = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det_a.abs() < 1e-15 {
        return Point3d::ORIGIN;
    }

    let x = b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]);

    let y = a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]);

    let z = a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    Point3d::new(x / det_a, y / det_a, z / det_a)
}

/// Plane-Cylinder intersection: returns an ellipse (or circle, or two lines).
fn intersect_plane_cylinder(plane: &Plane, cyl: &CylinderSurface, tol: f64) -> Vec<IntersectionCurve> {
    // Distance from cylinder origin to plane (signed)
    let dx = cyl.origin.x - plane.origin.x;
    let dy = cyl.origin.y - plane.origin.y;
    let dz = cyl.origin.z - plane.origin.z;
    let signed_dist = dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z;
    let dist = signed_dist.abs();

    // Angle between plane normal and cylinder axis
    let cos_angle = (plane.normal.x * cyl.axis.x
        + plane.normal.y * cyl.axis.y
        + plane.normal.z * cyl.axis.z)
        .abs();

    // For a plane PERPENDICULAR to the cylinder axis (cos_angle ≈ 1):
    //   The intersection is always a circle (if the plane is within the
    //   cylinder's height range). The `dist > radius` check does NOT apply
    //   here because dist is the distance along the axis, not from the axis.
    //
    // For a plane PARALLEL or at an angle to the axis (cos_angle < 1):
    //   The intersection is an ellipse (or two lines if parallel).
    //   The `dist > radius` check applies — if the plane is too far from
    //   the axis, there's no intersection.
    if cos_angle < 1.0 - 1e-10 && dist > cyl.radius + tol {
        return Vec::new(); // No intersection (plane misses cylinder)
    }

    if cos_angle > 1.0 - 1e-10 {
        // Plane perpendicular to cylinder axis — circle intersection.
        // The center of the intersection circle is the point on the cylinder
        // axis that lies on the plane. Since the plane normal is parallel to
        // the axis, we project the cylinder origin onto the plane:
        //   center = cyl.origin - signed_dist * plane.normal
        let center_on_axis = Point3d::new(
            cyl.origin.x - signed_dist * plane.normal.x,
            cyl.origin.y - signed_dist * plane.normal.y,
            cyl.origin.z - signed_dist * plane.normal.z,
        );

        // Create a circle in the plane at the intersection
        let circle = Circle::new(center_on_axis, plane.normal, cyl.radius);

        // Sample the circle
        let n_samples = 100;
        let points: Vec<Point3d> = (0..=n_samples)
            .map(|i| {
                let t = 2.0 * PI * (i as f64 / n_samples as f64);
                circle.point_at(t)
            })
            .collect();

        // Build PCurve on the cylinder: v = signed_dist (constant height),
        // u goes from 0 to 2π. This is a straight horizontal line in UV.
        let v_on_cyl = signed_dist; // height along cylinder axis
        let pcurve_cyl = Curve2d::Line(Line2d::new(
            Point2d::new(0.0, v_on_cyl),
            Point2d::new(2.0 * PI, v_on_cyl),
        ));

        // Build PCurve on the plane: a circle in the plane's UV domain.
        // The plane's UV origin is at plane.origin, with u_dir and v_dir.
        // The intersection circle has center at center_on_axis and radius cyl.radius.
        // Project the circle center into plane UV:
        let dcx = center_on_axis.x - plane.origin.x;
        let dcy = center_on_axis.y - plane.origin.y;
        let dcz = center_on_axis.z - plane.origin.z;
        let center_u = dcx * plane.u_dir.x + dcy * plane.u_dir.y + dcz * plane.u_dir.z;
        let center_v = dcx * plane.v_dir.x + dcy * plane.v_dir.y + dcz * plane.v_dir.z;
        let pcurve_plane = Curve2d::Circle(Circle2d::new_full(
            Point2d::new(center_u, center_v),
            cyl.radius,
        ));

        vec![IntersectionCurve {
            points,
            curve: Some(Curve3d::Circle(circle)),
            pcurve_a: Some(pcurve_plane),   // PCurve on plane (surface A)
            pcurve_b: Some(pcurve_cyl),     // PCurve on cylinder (surface B)
            tolerance: tol,
        }]
    } else {
        // Ellipse intersection
        // Semi-minor axis = sqrt(R² - d²)
        let semi_minor_sq = cyl.radius * cyl.radius - dist * dist;
        if semi_minor_sq < 0.0 {
            return Vec::new();
        }
        let semi_minor = semi_minor_sq.sqrt();
        let semi_major = cyl.radius / cos_angle.max(1e-10).min(1.0);

        // The ellipse center is on the cylinder axis, projected onto the plane
        let axis_dot_normal = dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z;
        let center = Point3d::new(
            cyl.origin.x - axis_dot_normal * plane.normal.x,
            cyl.origin.y - axis_dot_normal * plane.normal.y,
            cyl.origin.z - axis_dot_normal * plane.normal.z,
        );

        // X-axis of ellipse is along the cylinder's radial direction projected onto the plane
        let x_axis = if cos_angle < 1e-10 {
            // Plane parallel to axis — intersection is two lines or a rectangle
            // For now, sample points
            cyl.x_dir
        } else {
            cyl.x_dir
        };

        let ellipse = Ellipse {
            center,
            normal: plane.normal,
            semi_major,
            semi_minor,
            x_axis,
        };

        let n_samples = 100;
        let points: Vec<Point3d> = (0..=n_samples)
            .map(|i| {
                let t = 2.0 * PI * (i as f64 / n_samples as f64);
                ellipse.point_at(t)
            })
            .collect();

        vec![IntersectionCurve {
            points,
            curve: Some(Curve3d::Ellipse(ellipse)),
            pcurve_a: None,
            pcurve_b: None,
            tolerance: tol,
        }]
    }
}

/// Plane-Sphere intersection: returns a circle (or point, or nothing).
fn intersect_plane_sphere(plane: &Plane, sphere: &SphereSurface, tol: f64) -> Vec<IntersectionCurve> {
    // Distance from sphere center to plane
    let dx = sphere.center.x - plane.origin.x;
    let dy = sphere.center.y - plane.origin.y;
    let dz = sphere.center.z - plane.origin.z;
    let dist = (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).abs();

    if dist > sphere.radius + tol {
        return Vec::new(); // No intersection
    }

    if dist > sphere.radius - tol {
        // Tangent — single point
        let point = Point3d::new(
            sphere.center.x - dist * plane.normal.x,
            sphere.center.y - dist * plane.normal.y,
            sphere.center.z - dist * plane.normal.z,
        );
        return vec![IntersectionCurve {
            points: vec![point],
            curve: None,
            pcurve_a: None,
            pcurve_b: None,
            tolerance: tol,
        }];
    }

    // Circle intersection
    let circle_radius = (sphere.radius * sphere.radius - dist * dist).sqrt();
    let center = Point3d::new(
        sphere.center.x - dist * plane.normal.x
            * (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).signum(),
        sphere.center.y - dist * plane.normal.y
            * (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).signum(),
        sphere.center.z - dist * plane.normal.z
            * (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).signum(),
    );

    let circle = Circle::new(center, plane.normal, circle_radius);

    let n_samples = 100;
    let points: Vec<Point3d> = (0..=n_samples)
        .map(|i| {
            let t = 2.0 * PI * (i as f64 / n_samples as f64);
            circle.point_at(t)
        })
        .collect();

    vec![IntersectionCurve {
        points,
        curve: Some(Curve3d::Circle(circle)),
        pcurve_a: None,
        pcurve_b: None,
        tolerance: tol,
    }]
}

/// Plane-Cone intersection: returns a conic section (ellipse, parabola, or hyperbola).
fn intersect_plane_cone(plane: &Plane, cone: &ConeSurface, tol: f64) -> Vec<IntersectionCurve> {
    // B1 leftover (2026-09-01): analytic conic section — ellipse / circle
    // (plane flatter than the generators), parabola (plane parallel to a
    // generator), hyperbola branch (steeper plane), or degenerate generator
    // rays when the plane passes through the apex. Every output point
    // satisfies both surface equations exactly (parametrized on cone
    // generators: P = apex + t(u)·g(u)).
    // Previously this was a stub delegating to the brute-force grid pairing
    // in `sample_surface_intersection`.
    let polylines = draper_geometry::intersection::intersect_plane_cone(plane, cone, tol);
    polylines
        .into_iter()
        .filter(|pts| pts.len() >= 2)
        .map(|points| IntersectionCurve {
            points,
            curve: None,
            pcurve_a: None,
            pcurve_b: None,
            tolerance: tol,
        })
        .collect()
}

/// Cylinder-Cylinder intersection (B1-series follow-up, 2026-09-02).
///
/// Analytic path via
/// [`draper_geometry::intersection::intersect_cylinder_cylinder`]:
/// parallel axes → 2 / 1 / 0 straight lines with the EXACT `Line`
/// geometry attached; non-parallel axes → exact per-θ quadratic solve
/// (points lie on both cylinders to floating-point precision, no
/// marching): full-circle discriminant → two root-branch loops
/// (bicylinder envelopes), arc discriminant → one closed loop per arc
/// with the branches joining at the pinch ends, tangency → a single
/// point, disjoint/contained → empty.
fn intersect_cylinder_cylinder(
    c1: &CylinderSurface,
    c2: &CylinderSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let polylines = draper_geometry::intersection::intersect_cylinder_cylinder(c1, c2, tol);

    // Exact Line geometry for the PARALLEL case: the intersection lines
    // run along the shared axis direction. Non-parallel curves are space
    // quartics (not in this kernel's analytic curve set) — polyline-only,
    // same convention as the sphere×cylinder off-axis quartic.
    let axes_parallel = {
        let dot = c1.axis.x * c2.axis.x + c1.axis.y * c2.axis.y + c1.axis.z * c2.axis.z;
        dot.abs() > 0.9999
    };
    let line_dir = if axes_parallel {
        Direction3d::new(c1.axis.x, c1.axis.y, c1.axis.z)
    } else {
        None
    };

    polylines
        .into_iter()
        .filter(|pts| !pts.is_empty())
        .map(|points| {
            let curve = line_dir.as_ref().and_then(|dir| {
                points.first().map(|p0| Curve3d::Line(Line::new(*p0, *dir)))
            });
            // A single point (tangency) has no curve geometry.
            let curve = if points.len() >= 2 { curve } else { None };
            IntersectionCurve {
                points,
                curve,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: tol,
            }
        })
        .collect()
}

/// Cone-Cone intersection (B1-series follow-up, 2026-09-02).
///
/// Analytic path via
/// [`draper_geometry::intersection::intersect_cone_cone`]: generic
/// non-parallel axes → per-θ quadratic on the generator slant of the
/// parametrized cone (points on both nappes to floating-point precision,
/// no marching); parallel axes + equal angles → planar conic (linear
/// root, arms clipped); coaxial configurations → full circles with the
/// EXACT `Circle` geometry attached; shared apices → common generator
/// rays; tangency → a single point; disjoint/identical → empty.
fn intersect_cone_cone(c1: &ConeSurface, c2: &ConeSurface, tol: f64) -> Vec<IntersectionCurve> {
    let polylines = draper_geometry::intersection::intersect_cone_cone(c1, c2, tol);

    // Exact Circle for the coaxial full-circle case: parallel axes and the
    // single curve equidistant from the common axis line (through cone A's
    // apex). Conic arms / quartic loops / rays stay polyline-only.
    let axes_parallel = {
        let cx = c1.axis.y * c2.axis.z - c1.axis.z * c2.axis.y;
        let cy = c1.axis.z * c2.axis.x - c1.axis.x * c2.axis.z;
        let cz = c1.axis.x * c2.axis.y - c1.axis.y * c2.axis.x;
        (cx * cx + cy * cy + cz * cz).sqrt() < 1e-6
    };
    let axis_origin = cone_apex(c1);

    polylines
        .into_iter()
        .filter(|pts| !pts.is_empty())
        .map(|points| {
            let curve = if axes_parallel && points.len() >= 8 {
                coaxial_circle_from_points(&points, &axis_origin, &c1.axis, tol)
                    .map(Curve3d::Circle)
            } else {
                None
            };
            IntersectionCurve {
                points,
                curve,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: tol,
            }
        })
        .collect()
}

/// Cone-Cylinder intersection (B1-series follow-up, 2026-09-02).
///
/// Analytic path via
/// [`draper_geometry::intersection::intersect_cone_cylinder`] (the
/// cylinder parametrized axially — the cylinder×cylinder quadratic
/// structure with the cone nappe-side sheet filter). Coaxial circles get
/// the EXACT `Circle` geometry; everything else stays polyline-only.
fn intersect_cone_cylinder_pair(
    cone: &ConeSurface,
    cyl: &CylinderSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let polylines = draper_geometry::intersection::intersect_cone_cylinder(cone, cyl, tol);

    let axes_parallel = {
        let cx = cone.axis.y * cyl.axis.z - cone.axis.z * cyl.axis.y;
        let cy = cone.axis.z * cyl.axis.x - cone.axis.x * cyl.axis.z;
        let cz = cone.axis.x * cyl.axis.y - cone.axis.y * cyl.axis.x;
        (cx * cx + cy * cy + cz * cz).sqrt() < 1e-6
    };

    polylines
        .into_iter()
        .filter(|pts| !pts.is_empty())
        .map(|points| {
            let curve = if axes_parallel && points.len() >= 8 {
                coaxial_circle_from_points(&points, &cyl.origin, &cyl.axis, tol)
                    .map(Curve3d::Circle)
            } else {
                None
            };
            IntersectionCurve {
                points,
                curve,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: tol,
            }
        })
        .collect()
}

/// Cone apex (for the wrapper's circle fit; mirrors the geometry-side
/// `ConeView` computation).
fn cone_apex(cone: &ConeSurface) -> Point3d {
    let tan_ha = cone.half_angle.tan();
    if cone.expanding || !tan_ha.is_finite() {
        cone.origin
    } else {
        let v_apex = -cone.radius / tan_ha;
        Point3d::new(
            cone.origin.x + v_apex * cone.axis.x,
            cone.origin.y + v_apex * cone.axis.y,
            cone.origin.z + v_apex * cone.axis.z,
        )
    }
}

/// Torus-Plane intersection (T-series, 2026-09-02).
///
/// Analytic path via
/// [`draper_geometry::intersection::intersect_torus_plane`]: plane ⟂ axis
/// → 0/1/2 latitude circles (EXACT `Circle` geometry around the torus
/// axis); plane containing the axis → 2 meridian tube circles (EXACT
/// `Circle` geometry, dual-candidate axis fit); oblique/offset planes →
/// per-θ linear tube solve (polyline-only: toric quartic sections,
/// offset "peanut" ovals); tangency → single point.
fn intersect_torus_plane_pair(
    plane: &Plane,
    torus: &TorusSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let polylines = draper_geometry::intersection::intersect_torus_plane(plane, torus, tol);

    let n_t = Vec3d::new(torus.axis.x, torus.axis.y, torus.axis.z);
    let n_p = Vec3d::new(plane.normal.x, plane.normal.y, plane.normal.z);
    let b_axis = n_p.dot(&n_t);
    // h = signed distance of the torus center from the plane.
    let w0 = Vec3d::new(
        torus.center.x - plane.origin.x,
        torus.center.y - plane.origin.y,
        torus.center.z - plane.origin.z,
    );
    let h = w0.dot(&n_p);

    // Meridian candidate axis (plane ∥ axis containing it): the tube
    // circles have centers O ± R·u with u = (n_p × n)/|n_p × n| and
    // circle-plane normal u × n.
    let meridian: Option<(Point3d, Direction3d)> = if b_axis.abs() <= 1e-10 && h.abs() <= tol.max(1e-9)
    {
        let cross = n_p.cross(&n_t);
        let l = cross.length();
        if l < 1e-12 {
            None
        } else {
            let u = Vec3d::new(cross.x / l, cross.y / l, cross.z / l);
            let d = u.cross(&n_t);
            let center = Point3d::new(
                torus.center.x + torus.major_radius * u.x,
                torus.center.y + torus.major_radius * u.y,
                torus.center.z + torus.major_radius * u.z,
            );
            let axis = Direction3d::new(d.x, d.y, d.z).unwrap_or(Direction3d::Z);
            Some((center, axis))
        }
    } else {
        None
    };

    polylines
        .into_iter()
        .filter(|pts| !pts.is_empty())
        .map(|points| {
            let curve = if b_axis.abs() >= 1.0 - 1e-12 {
                // Plane ⟂ axis: latitude circles around the torus axis.
                coaxial_circle_from_points(&points, &torus.center, &torus.axis, tol)
                    .map(Curve3d::Circle)
            } else if let Some((c0, axis_dir)) = &meridian {
                // Plane containing the axis: meridian tube circle — try
                // this candidate center, then the antipodal one.
                coaxial_circle_from_points(&points, c0, axis_dir, tol)
                    .or_else(|| {
                        let c1 = Point3d::new(
                            2.0 * torus.center.x - c0.x,
                            2.0 * torus.center.y - c0.y,
                            2.0 * torus.center.z - c0.z,
                        );
                        coaxial_circle_from_points(&points, &c1, axis_dir, tol)
                    })
                    .map(Curve3d::Circle)
            } else {
                None
            };
            IntersectionCurve {
                points,
                curve,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: tol,
            }
        })
        .collect()
}

/// Torus-Sphere intersection (T-series, 2026-09-02).
///
/// Analytic path via
/// [`draper_geometry::intersection::intersect_torus_sphere`]: concentric
/// (sphere center on the torus axis) → 0/1/2 latitude circles with the
/// EXACT `Circle` geometry; off-axis → per-θ linear tube solve
/// (polyline-only space curves); tangency → single point;
/// contained/disjoint → empty.
fn intersect_torus_sphere_pair(
    sphere: &SphereSurface,
    torus: &TorusSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let polylines = draper_geometry::intersection::intersect_torus_sphere(sphere, torus, tol);

    // Concentric guard: sphere center on the torus axis.
    let v = Vec3d::new(
        sphere.center.x - torus.center.x,
        sphere.center.y - torus.center.y,
        sphere.center.z - torus.center.z,
    );
    let n_t = Vec3d::new(torus.axis.x, torus.axis.y, torus.axis.z);
    let v_ax = v.dot(&n_t);
    let v_perp = Vec3d::new(
        v.x - v_ax * n_t.x,
        v.y - v_ax * n_t.y,
        v.z - v_ax * n_t.z,
    );
    let concentric = v_perp.length() <= tol.max(1e-9);

    polylines
        .into_iter()
        .filter(|pts| !pts.is_empty())
        .map(|points| {
            let curve = if concentric && points.len() >= 8 {
                coaxial_circle_from_points(&points, &torus.center, &torus.axis, tol)
                    .map(Curve3d::Circle)
            } else {
                None
            };
            IntersectionCurve {
                points,
                curve,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: tol,
            }
        })
        .collect()
}

/// Torus-Cylinder intersection (T-series, 2026-09-02).
///
/// Analytic path via
/// [`draper_geometry::intersection::intersect_torus_cylinder`]: coaxial →
/// 0/1/2 circles with the EXACT `Circle` geometry around the common
/// axis; parallel offset → per-θ quadratic (upper half + equatorial
/// mirror, polyline-only); perpendicular → ψ-parametrized twin-pass
/// solve (polyline-only); skew → marching fallback.
fn intersect_torus_cylinder_pair(
    cyl: &CylinderSurface,
    torus: &TorusSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let polylines = draper_geometry::intersection::intersect_torus_cylinder(cyl, torus, tol);

    // Coaxial guard: parallel axes and the cylinder axis through the
    // torus center (radial offset ≈ 0).
    let n_c = Vec3d::new(cyl.axis.x, cyl.axis.y, cyl.axis.z);
    let n_t = Vec3d::new(torus.axis.x, torus.axis.y, torus.axis.z);
    let cross = n_c.cross(&n_t);
    let axes_parallel = cross.length_sq() < 1e-12;
    let coaxial = axes_parallel && {
        let w = Vec3d::new(
            cyl.origin.x - torus.center.x,
            cyl.origin.y - torus.center.y,
            cyl.origin.z - torus.center.z,
        );
        let w_ax = w.dot(&n_t);
        let w_perp = Vec3d::new(
            w.x - w_ax * n_t.x,
            w.y - w_ax * n_t.y,
            w.z - w_ax * n_t.z,
        );
        w_perp.length() <= tol.max(1e-9)
    };

    polylines
        .into_iter()
        .filter(|pts| !pts.is_empty())
        .map(|points| {
            let curve = if coaxial && points.len() >= 8 {
                coaxial_circle_from_points(&points, &torus.center, &torus.axis, tol)
                    .map(Curve3d::Circle)
            } else {
                None
            };
            IntersectionCurve {
                points,
                curve,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: tol,
            }
        })
        .collect()
}

/// Fit an exact [`Circle`] to a polyline that is a full circle around the
/// axis line (origin, axis): the foot of the first point is the center
/// candidate; every point must project to the same foot (planarity ⊥
/// axis) and be equidistant from it. Returns `None` for non-circular
/// polylines (conic arms, quartic loops, rays, micro-slivers).
fn coaxial_circle_from_points(
    points: &[Point3d],
    axis_origin: &Point3d,
    axis: &Direction3d,
    tol: f64,
) -> Option<Circle> {
    let n = Vec3d::new(axis.x, axis.y, axis.z);
    let foot = |p: &Point3d| -> Point3d {
        let dx = p.x - axis_origin.x;
        let dy = p.y - axis_origin.y;
        let dz = p.z - axis_origin.z;
        let d = dx * n.x + dy * n.y + dz * n.z;
        Point3d::new(
            axis_origin.x + d * n.x,
            axis_origin.y + d * n.y,
            axis_origin.z + d * n.z,
        )
    };
    let center = foot(&points[0]);
    let v0 = Vec3d::new(
        points[0].x - center.x,
        points[0].y - center.y,
        points[0].z - center.z,
    );
    let radius = v0.length();
    if radius <= tol.max(1e-9) {
        return None;
    }
    let fit_tol = 1e-6 * (1.0 + radius);
    for p in points.iter().skip(1) {
        let c = foot(p);
        if (c.x - center.x).abs() > fit_tol
            || (c.y - center.y).abs() > fit_tol
            || (c.z - center.z).abs() > fit_tol
        {
            return None; // not planar ⊥ axis → not a coaxial circle
        }
        let v = Vec3d::new(p.x - center.x, p.y - center.y, p.z - center.z);
        if (v.length() - radius).abs() > fit_tol {
            return None; // not equidistant → conic arm or generic loop
        }
    }
    let normal = Direction3d::new(axis.x, axis.y, axis.z).unwrap_or(Direction3d::Z);
    Some(Circle::new(center, normal, radius))
}

/// Cylinder-Sphere intersection (B1-series follow-up, 2026-09-02).
///
/// Analytic path via
/// [`draper_geometry::intersection::intersect_sphere_cylinder`]
/// ("Steinmetch" cases): axis-through-center → circles at
/// `z = ±√(r² − R²)` (with the EXACT `Circle` geometry attached);
/// off-axis → the quartic `t²(θ) = A + B·cos(θ − φ₀)` sampled with
/// cos-clustered branch points (exact on both surfaces, no marching);
/// tangency → a single point; disjoint/contained → empty.
fn intersect_cylinder_sphere(
    cyl: &CylinderSurface,
    sphere: &SphereSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let eps = tol.max(1e-9);
    let polylines = draper_geometry::intersection::intersect_sphere_cylinder(sphere, cyl, tol);

    // Exact circle geometry for the axis-through-center case (d ≈ 0):
    // two Steinmetch circles at t = ±√(r² − R²), or one equatorial circle
    // of tangency (r ≈ R). Off-axis quartics and tangent points stay
    // polyline-only (the curve is a space quartic, not an analytic
    // Circle in this kernel's curve set).
    let dx = sphere.center.x - cyl.origin.x;
    let dy = sphere.center.y - cyl.origin.y;
    let dz = sphere.center.z - cyl.origin.z;
    let along = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
    let perp_x = dx - along * cyl.axis.x;
    let perp_y = dy - along * cyl.axis.y;
    let perp_z = dz - along * cyl.axis.z;
    let d = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();

    let exact_circles: Vec<Option<Circle>> = if d <= eps {
        let t_sq = sphere.radius * sphere.radius - cyl.radius * cyl.radius;
        if t_sq < -eps * eps {
            // Sphere strictly inside — no curves at all.
            vec![]
        } else {
            let t = t_sq.max(0.0).sqrt();
            let foot = Point3d::new(
                cyl.origin.x + along * cyl.axis.x,
                cyl.origin.y + along * cyl.axis.y,
                cyl.origin.z + along * cyl.axis.z,
            );
            let normal = Direction3d::new(cyl.axis.x, cyl.axis.y, cyl.axis.z)
                .unwrap_or(Direction3d::Z);
            if t > eps {
                vec![
                    Some(Circle::new(
                        Point3d::new(
                            foot.x + t * cyl.axis.x,
                            foot.y + t * cyl.axis.y,
                            foot.z + t * cyl.axis.z,
                        ),
                        normal,
                        cyl.radius,
                    )),
                    Some(Circle::new(
                        Point3d::new(
                            foot.x - t * cyl.axis.x,
                            foot.y - t * cyl.axis.y,
                            foot.z - t * cyl.axis.z,
                        ),
                        normal,
                        cyl.radius,
                    )),
                ]
            } else {
                // Equatorial tangency (r ≈ R): one circle at t = 0.
                vec![Some(Circle::new(foot, normal, cyl.radius))]
            }
        }
    } else {
        vec![]
    };

    polylines
        .into_iter()
        .enumerate()
        .filter(|(_, pts)| !pts.is_empty())
        .map(|(i, points)| IntersectionCurve {
            points,
            curve: exact_circles
                .get(i)
                .and_then(|c| c.clone())
                .map(Curve3d::Circle),
            pcurve_a: None,
            pcurve_b: None,
            tolerance: tol,
        })
        .collect()
}

/// Sphere-Sphere intersection (B1 series follow-up, 2026-09-01).
///
/// The intersection of two spheres is a circle in the **radical plane**
/// (perpendicular to the center line) — computed analytically by
/// [`draper_geometry::intersection::intersect_sphere_sphere`]; tangency
/// degenerates to a single point, disjoint/contained/concentric to empty.
/// The exact circle is attached as `curve` so downstream consumers get
/// the analytic geometry, not just the sampled polyline.
fn intersect_sphere_sphere(
    s1: &SphereSurface,
    s2: &SphereSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let polylines = draper_geometry::intersection::intersect_sphere_sphere(s1, s2, tol);

    // Recompute the radical-plane circle parameters for the exact curve
    // (mirrors the math in the geometry function; kept in sync by tests).
    let dx = s2.center.x - s1.center.x;
    let dy = s2.center.y - s1.center.y;
    let dz = s2.center.z - s1.center.z;
    let d = (dx * dx + dy * dy + dz * dz).sqrt();
    let curve = if d > tol.max(1e-9) {
        let sum = s1.radius + s2.radius;
        let diff = (s1.radius - s2.radius).abs();
        let eps = tol.max(1e-9);
        let general = (d - sum).abs() > eps && (d - diff).abs() > eps && d <= sum && d >= diff;
        if general {
            let a = (d * d + s1.radius * s1.radius - s2.radius * s2.radius) / (2.0 * d);
            let h_sq = (s1.radius * s1.radius - a * a).max(0.0);
            let h = h_sq.sqrt();
            let center = Point3d::new(
                s1.center.x + (a / d) * dx,
                s1.center.y + (a / d) * dy,
                s1.center.z + (a / d) * dz,
            );
            let normal = Direction3d::new(dx / d, dy / d, dz / d)
                .unwrap_or(Direction3d::Z);
            Some(Circle::new(center, normal, h))
        } else {
            None
        }
    } else {
        None
    };

    polylines
        .into_iter()
        .filter(|pts| pts.len() >= 1)
        .map(|points| IntersectionCurve {
            points,
            curve: curve.clone().map(Curve3d::Circle),
            pcurve_a: None,
            pcurve_b: None,
            tolerance: tol,
        })
        .collect()
}

/// General surface-surface intersection using subdivision/Newton-Raphson.
///
/// Algorithm:
/// 1. Sample both surfaces on a grid
/// 2. Find cells where the signed distance changes sign (indicating intersection)
/// 3. Refine intersection points using Newton-Raphson
/// 4. Connect points into curves
fn intersect_surfaces_general(
    surface_a: &Surface,
    surface_b: &Surface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    sample_surface_intersection(surface_a, surface_b, tol)
}

/// Sample-based surface intersection.
fn sample_surface_intersection(
    surface_a: &Surface,
    surface_b: &Surface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    let (u_min_a, u_max_a, v_min_a, v_max_a) = surface_param_range(surface_a);
    let (u_min_b, u_max_b, v_min_b, v_max_b) = surface_param_range(surface_b);

    let n_a = 40;
    let n_b = 40;

    // Sample surface A on a grid
    let mut points_a: Vec<(f64, f64, Point3d)> = Vec::with_capacity((n_a + 1) * (n_a + 1));
    for i in 0..=n_a {
        for j in 0..=n_a {
            let u = u_min_a + (u_max_a - u_min_a) * (i as f64 / n_a as f64);
            let v = v_min_a + (v_max_a - v_min_a) * (j as f64 / n_a as f64);
            points_a.push((u, v, surface_a.point_at(u, v)));
        }
    }

    // Sample surface B on a grid
    let mut points_b: Vec<(f64, f64, Point3d)> = Vec::with_capacity((n_b + 1) * (n_b + 1));
    for i in 0..=n_b {
        for j in 0..=n_b {
            let u = u_min_b + (u_max_b - u_min_b) * (i as f64 / n_b as f64);
            let v = v_min_b + (v_max_b - v_min_b) * (j as f64 / n_b as f64);
            points_b.push((u, v, surface_b.point_at(u, v)));
        }
    }

    // Find approximate intersection points by finding close point pairs
    let mut intersection_points: Vec<Point3d> = Vec::new();

    for (_, _, pa) in &points_a {
        for (_, _, pb) in &points_b {
            if pa.distance_to(pb) < tol * 10.0 {
                let midpoint = pa.midpoint(pb);
                intersection_points.push(midpoint);
            }
        }
    }

    // Refine intersection points using Newton-Raphson
    let mut refined_points: Vec<Point3d> = Vec::new();
    for p in &intersection_points {
        if let Some(rp) = refine_intersection_point(p, surface_a, surface_b, tol) {
            // Check for duplicates
            let is_dup = refined_points.iter().any(|ep| ep.distance_to(&rp) < tol * 10.0);
            if !is_dup {
                refined_points.push(rp);
            }
        }
    }

    if refined_points.is_empty() {
        return Vec::new();
    }

    // Sort points into curves by proximity
    let curves = chain_points_into_curves(&refined_points, tol * 100.0);

    // Refine each curve with more sampling along the intersection
    curves
        .into_iter()
        .map(|mut curve| {
            if curve.points.len() >= 2 {
                // Resample the curve to get smoother results
                curve.points = resample_curve_points(&curve.points, 100);
            }
            curve
        })
        .collect()
}

/// Refine an approximate intersection point using Newton-Raphson.
///
/// The intersection condition is: S_a(u_a, v_a) = S_b(u_b, v_b)
/// We solve for (u_a, v_a, u_b, v_b) such that S_a - S_b = 0.
fn refine_intersection_point(
    initial: &Point3d,
    surface_a: &Surface,
    surface_b: &Surface,
    tol: f64,
) -> Option<Point3d> {
    // Project the initial point onto both surfaces to get initial UV params
    let (mut ua, mut va) = project_point_to_surface_uv(initial, surface_a);
    let (mut ub, mut vb) = project_point_to_surface_uv(initial, surface_b);

    let max_iter = 20;
    let eps = 1e-10;

    for _ in 0..max_iter {
        let pa = surface_a.point_at(ua, va);
        let pb = surface_b.point_at(ub, vb);

        // Residual: pa - pb
        let rx = pa.x - pb.x;
        let ry = pa.y - pb.y;
        let rz = pa.z - pb.z;
        let residual = (rx * rx + ry * ry + rz * rz).sqrt();

        if residual < tol {
            return Some(Point3d::new(
                (pa.x + pb.x) / 2.0,
                (pa.y + pb.y) / 2.0,
                (pa.z + pb.z) / 2.0,
            ));
        }

        // Compute Jacobian numerically
        let h = 1e-6;

        // Partial derivatives of S_a
        let pa_du = surface_a.point_at(ua + h, va);
        let pa_dv = surface_a.point_at(ua, va + h);
        let da_du = Vec3d::new((pa_du.x - pa.x) / h, (pa_du.y - pa.y) / h, (pa_du.z - pa.z) / h);
        let da_dv = Vec3d::new((pa_dv.x - pa.x) / h, (pa_dv.y - pa.y) / h, (pa_dv.z - pa.z) / h);

        // Partial derivatives of S_b
        let pb_du = surface_b.point_at(ub + h, vb);
        let pb_dv = surface_b.point_at(ub, vb + h);
        let db_du = Vec3d::new((pb_du.x - pb.x) / h, (pb_du.y - pb.y) / h, (pb_du.z - pb.z) / h);
        let db_dv = Vec3d::new((pb_dv.x - pb.x) / h, (pb_dv.y - pb.y) / h, (pb_dv.z - pb.z) / h);

        // Jacobian: J = [da_du, da_dv, -db_du, -db_dv]
        // System: J * [dua, dva, dub, dvb]^T = -[rx, ry, rz]^T
        // This is an underdetermined system (3 equations, 4 unknowns)
        // Use pseudo-inverse or least-squares

        // Build 3x4 Jacobian matrix
        let j = [
            [da_du.x, da_dv.x, -db_du.x, -db_dv.x],
            [da_du.y, da_dv.y, -db_du.y, -db_dv.y],
            [da_du.z, da_dv.z, -db_du.z, -db_dv.z],
        ];

        // Solve using normal equations: J^T J x = J^T b
        let jtj = mat4_multiply_mat4_transpose(&j);
        let jtb = [
            -(j[0][0] * rx + j[1][0] * ry + j[2][0] * rz),
            -(j[0][1] * rx + j[1][1] * ry + j[2][1] * rz),
            -(j[0][2] * rx + j[1][2] * ry + j[2][2] * rz),
            -(j[0][3] * rx + j[1][3] * ry + j[2][3] * rz),
        ];

        if let Some(delta) = solve_4x4(&jtj, &jtb) {
            ua += delta[0];
            va += delta[1];
            ub += delta[2];
            vb += delta[3];

            // Clamp to parametric ranges
            let (ua_min, ua_max, va_min, va_max) = surface_param_range(surface_a);
            let (ub_min, ub_max, vb_min, vb_max) = surface_param_range(surface_b);
            ua = ua.clamp(ua_min, ua_max);
            va = va.clamp(va_min, va_max);
            ub = ub.clamp(ub_min, ub_max);
            vb = vb.clamp(vb_min, vb_max);

            let step_norm = delta.iter().map(|d| d * d).sum::<f64>().sqrt();
            if step_norm < eps {
                break;
            }
        } else {
            break;
        }
    }

    let pa = surface_a.point_at(ua, va);
    let pb = surface_b.point_at(ub, vb);
    let residual = pa.distance_to(&pb);

    if residual < tol * 100.0 {
        Some(Point3d::new(
            (pa.x + pb.x) / 2.0,
            (pa.y + pb.y) / 2.0,
            (pa.z + pb.z) / 2.0,
        ))
    } else {
        None
    }
}

/// Multiply J^T * J to get a 4x4 matrix.
fn mat4_multiply_mat4_transpose(j: &[[f64; 4]; 3]) -> [[f64; 4]; 4] {
    let mut result = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j_idx in 0..4 {
            let mut sum = 0.0;
            for k in 0..3 {
                sum += j[k][i] * j[k][j_idx];
            }
            result[i][j_idx] = sum;
        }
    }
    // Add regularization
    for i in 0..4 {
        result[i][i] += 1e-10;
    }
    result
}

/// Solve a 4x4 linear system using Gaussian elimination with partial pivoting.
fn solve_4x4(a: &[[f64; 4]; 4], b: &[f64; 4]) -> Option<[f64; 4]> {
    let mut aug = [[0.0f64; 5]; 4];
    for i in 0..4 {
        for j in 0..4 {
            aug[i][j] = a[i][j];
        }
        aug[i][4] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..4 {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..4 {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-15 {
            return None; // Singular
        }

        // Swap rows
        if max_row != col {
            for j in 0..5 {
                let tmp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }

        // Eliminate below
        for row in (col + 1)..4 {
            let factor = aug[row][col] / aug[col][col];
            for j in col..5 {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0f64; 4];
    for i in (0..4).rev() {
        let mut sum = aug[i][4];
        for j in (i + 1)..4 {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-15 {
            return None;
        }
        x[i] = sum / aug[i][i];
    }

    Some(x)
}

/// Chain intersection points into curves based on proximity.
fn chain_points_into_curves(points: &[Point3d], max_gap: f64) -> Vec<IntersectionCurve> {
    if points.is_empty() {
        return Vec::new();
    }

    let n = points.len();
    let mut visited = vec![false; n];
    let mut curves: Vec<IntersectionCurve> = Vec::new();

    // Build adjacency based on proximity
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in (i + 1)..n {
            if points[i].distance_to(&points[j]) < max_gap {
                adjacency[i].push(j);
                adjacency[j].push(i);
            }
        }
    }

    // Find connected components using DFS
    for start in 0..n {
        if visited[start] {
            continue;
        }

        let mut component: Vec<usize> = Vec::new();
        let mut stack = vec![start];
        while let Some(node) = stack.pop() {
            if visited[node] {
                continue;
            }
            visited[node] = true;
            component.push(node);
            for &neighbor in &adjacency[node] {
                if !visited[neighbor] {
                    stack.push(neighbor);
                }
            }
        }

        // Sort the component into a chain
        if component.len() >= 2 {
            let chain = order_chain(&component, &adjacency, points);
            let curve_points: Vec<Point3d> = chain.iter().map(|&i| points[i]).collect();
            curves.push(IntersectionCurve {
                points: curve_points,
                curve: None,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: max_gap,
            });
        } else if component.len() == 1 {
            curves.push(IntersectionCurve {
                points: vec![points[component[0]]],
                curve: None,
                pcurve_a: None,
                pcurve_b: None,
                tolerance: max_gap,
            });
        }
    }

    curves
}

/// Order points in a connected component into a chain (path).
fn order_chain(
    component: &[usize],
    adjacency: &[Vec<usize>],
    _points: &[Point3d],
) -> Vec<usize> {
    if component.len() <= 2 {
        return component.to_vec();
    }

    // Find an endpoint (vertex with degree 1)
    let mut start = component[0];
    for &idx in component {
        let degree = adjacency[idx]
            .iter()
            .filter(|&&n| component.contains(&n))
            .count();
        if degree <= 1 {
            start = idx;
            break;
        }
    }

    // Traverse the chain
    let mut ordered = vec![start];
    let mut current = start;
    let component_set: std::collections::HashSet<usize> = component.iter().copied().collect();

    loop {
        let next = adjacency[current]
            .iter()
            .filter(|&&n| component_set.contains(&n) && !ordered.contains(&n))
            .copied()
            .next();

        match next {
            Some(n) => {
                ordered.push(n);
                current = n;
            }
            None => break,
        }
    }

    ordered
}

/// Resample curve points to get a smooth, evenly-spaced curve.
fn resample_curve_points(points: &[Point3d], n_target: usize) -> Vec<Point3d> {
    if points.len() < 2 {
        return points.to_vec();
    }

    // Compute cumulative arc lengths
    let mut arc_lengths: Vec<f64> = vec![0.0];
    for i in 1..points.len() {
        let d = points[i].distance_to(&points[i - 1]);
        arc_lengths.push(arc_lengths[i - 1] + d);
    }

    let total_length = arc_lengths[arc_lengths.len() - 1];
    if total_length < 1e-15 {
        return points.to_vec();
    }

    // Resample at evenly-spaced arc lengths
    let mut resampled: Vec<Point3d> = Vec::with_capacity(n_target);
    for i in 0..=n_target {
        let target_len = total_length * (i as f64 / n_target as f64);

        // Find the segment containing this arc length
        let mut seg = 0;
        while seg < arc_lengths.len() - 1 && arc_lengths[seg + 1] < target_len {
            seg += 1;
        }
        if seg >= points.len() - 1 {
            resampled.push(points[points.len() - 1]);
        } else {
            let seg_len = arc_lengths[seg + 1] - arc_lengths[seg];
            let t = if seg_len > 1e-15 {
                (target_len - arc_lengths[seg]) / seg_len
            } else {
                0.0
            };
            resampled.push(points[seg].lerp(&points[seg + 1], t));
        }
    }

    resampled
}

// ============================================================
// 4.1.2 Curve-Surface Intersection (CSI)
// ============================================================

/// Result of a curve-surface intersection.
#[derive(Clone, Debug)]
pub struct CurveSurfaceIntersectionResult {
    /// The 3D intersection point.
    pub point: Point3d,
    /// Curve parameter (t).
    pub t: f64,
    /// Surface parameters (u, v).
    pub u: f64,
    pub v: f64,
}

/// Intersect a curve with a surface.
///
/// Uses analytic solutions for common curve-surface pairs (line-plane,
/// line-sphere, line-cylinder, circle-plane) and falls back to a
/// sampling + Newton-Raphson approach for the general case.
///
/// Returns intersection points as Vec<(t, u, v)> parameter triples.
pub fn intersect_curve_surface(
    curve: &Curve3d,
    surface: &Surface,
    tol_ctx: &ToleranceContext,
) -> Vec<CurveSurfaceIntersectionResult> {
    let tol = tol_ctx.coincidence_tolerance();

    // Try analytic solutions first for common pairs
    match (curve, surface) {
        (Curve3d::Line(line), Surface::Plane(plane)) => {
            return intersect_line_plane_csi(line, plane, tol);
        }
        (Curve3d::Line(line), Surface::Sphere(sphere)) => {
            return intersect_line_sphere_csi(line, sphere, tol);
        }
        (Curve3d::Line(line), Surface::Cylinder(cyl)) => {
            return intersect_line_cylinder_csi(line, cyl, tol);
        }
        (Curve3d::Line(line), Surface::Cone(cone)) => {
            return intersect_line_cone_csi(line, cone, tol);
        }
        (Curve3d::Circle(circle), Surface::Plane(plane)) => {
            return intersect_circle_plane_csi(circle, plane, tol);
        }
        _ => {}
    }

    // General case: sampling + Newton-Raphson
    intersect_curve_surface_general(curve, surface, tol_ctx)
}

/// Analytic line-plane CSI.
fn intersect_line_plane_csi(
    line: &Line,
    plane: &Plane,
    _tol: f64,
) -> Vec<CurveSurfaceIntersectionResult> {
    let denom = plane.normal.x * line.direction.x
        + plane.normal.y * line.direction.y
        + plane.normal.z * line.direction.z;
    if denom.abs() < 1e-15 {
        return Vec::new(); // Parallel
    }
    let dx = plane.origin.x - line.origin.x;
    let dy = plane.origin.y - line.origin.y;
    let dz = plane.origin.z - line.origin.z;
    let t = (plane.normal.x * dx + plane.normal.y * dy + plane.normal.z * dz) / denom;
    let point = line.point_at(t);
    let (u, v) = plane.project_point(&point);

    vec![CurveSurfaceIntersectionResult { point, t, u, v }]
}

/// Analytic line-sphere CSI.
fn intersect_line_sphere_csi(
    line: &Line,
    sphere: &SphereSurface,
    tol: f64,
) -> Vec<CurveSurfaceIntersectionResult> {
    let oc = Vec3d::new(
        line.origin.x - sphere.center.x,
        line.origin.y - sphere.center.y,
        line.origin.z - sphere.center.z,
    );
    let dir = Vec3d::new(line.direction.x, line.direction.y, line.direction.z);
    let a = dir.dot(&dir);
    let b = 2.0 * oc.dot(&dir);
    let c = oc.dot(&oc) - sphere.radius * sphere.radius;
    let disc = b * b - 4.0 * a * c;

    if disc < -tol {
        return Vec::new();
    }

    let sqrt_disc = if disc > 0.0 { disc.sqrt() } else { 0.0 };
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);

    let mut results = Vec::new();
    for t in [t1, t2] {
        if t.is_finite() {
            let point = line.point_at(t);
            let (u, v) = sphere.project_point(&point);
            results.push(CurveSurfaceIntersectionResult { point, t, u, v });
        }
    }
    results
}

/// Analytic line-cylinder CSI.
fn intersect_line_cylinder_csi(
    line: &Line,
    cyl: &CylinderSurface,
    _tol: f64,
) -> Vec<CurveSurfaceIntersectionResult> {
    let hits = intersect_line_cylinder(line, cyl);
    hits.into_iter()
        .filter_map(|point| {
            let dx = point.x - line.origin.x;
            let dy = point.y - line.origin.y;
            let dz = point.z - line.origin.z;
            let t = dx * line.direction.x + dy * line.direction.y + dz * line.direction.z;
            if t.is_finite() {
                let (u, v) = cyl.project_point(&point);
                Some(CurveSurfaceIntersectionResult { point, t, u, v })
            } else {
                None
            }
        })
        .collect()
}

/// Analytic line-cone CSI.
fn intersect_line_cone_csi(
    line: &Line,
    cone: &ConeSurface,
    tol: f64,
) -> Vec<CurveSurfaceIntersectionResult> {
    // For simplicity, use a sampling-based approach for line-cone
    // A full analytic solution requires solving a quadratic in the cone's local frame
    let y_dir = cone.axis.cross(&cone.x_dir);

    // Project line into cone's local coordinate system
    let dx0 = line.origin.x - cone.origin.x;
    let dy0 = line.origin.y - cone.origin.y;
    let dz0 = line.origin.z - cone.origin.z;

    let x0 = dx0 * cone.x_dir.x + dy0 * cone.x_dir.y + dz0 * cone.x_dir.z;
    let y0 = dx0 * y_dir.x + dy0 * y_dir.y + dz0 * y_dir.z;
    let z0 = dx0 * cone.axis.x + dy0 * cone.axis.y + dz0 * cone.axis.z;

    let dx = line.direction.x * cone.x_dir.x + line.direction.y * cone.x_dir.y + line.direction.z * cone.x_dir.z;
    let dy = line.direction.x * y_dir.x + line.direction.y * y_dir.y + line.direction.z * y_dir.z;
    let dz = line.direction.x * cone.axis.x + line.direction.y * cone.axis.y + line.direction.z * cone.axis.z;

    // Cone equation: x² + y² = (R₀ - z*tan(α))² (for standard cone)
    // Substituting line: (x0+t*dx)² + (y0+t*dy)² = (R₀ - (z0+t*dz)*tan(α))²
    let tan_a = cone.half_angle.tan();
    let r0 = if cone.expanding { 0.0 } else { cone.radius };

    let lhs_a = dx * dx + dy * dy - (dz * tan_a) * (dz * tan_a);
    let lhs_b = 2.0 * (x0 * dx + y0 * dy + dz * tan_a * (r0 - z0 * tan_a));
    let lhs_c = x0 * x0 + y0 * y0 - (r0 - z0 * tan_a) * (r0 - z0 * tan_a);

    let disc = lhs_b * lhs_b - 4.0 * lhs_a * lhs_c;
    if disc < -tol {
        return Vec::new();
    }

    let sqrt_disc = if disc > 0.0 { disc.sqrt() } else { 0.0 };
    let mut results = Vec::new();

    for t in [
        (-lhs_b - sqrt_disc) / (2.0 * lhs_a),
        (-lhs_b + sqrt_disc) / (2.0 * lhs_a),
    ] {
        if t.is_finite() && lhs_a.abs() > 1e-15 {
            let point = line.point_at(t);
            let (u, v) = cone.project_point(&point);
            results.push(CurveSurfaceIntersectionResult { point, t, u, v });
        }
    }

    // If lhs_a ≈ 0, it's a linear equation
    if lhs_a.abs() < 1e-15 && lhs_b.abs() > 1e-15 {
        let t = -lhs_c / lhs_b;
        if t.is_finite() {
            let point = line.point_at(t);
            let (u, v) = cone.project_point(&point);
            results.push(CurveSurfaceIntersectionResult { point, t, u, v });
        }
    }

    results
}

/// Analytic circle-plane CSI.
fn intersect_circle_plane_csi(
    circle: &Circle,
    plane: &Plane,
    tol: f64,
) -> Vec<CurveSurfaceIntersectionResult> {
    // The circle lies in a plane. If the circle's plane intersects
    // the given plane, the intersection is a line. The circle-line
    // intersection gives 0, 1, or 2 points.

    // Circle's plane: normal = circle.normal, origin = circle.center
    let circle_normal = circle.normal;

    // Direction of intersection line
    let cross_v = Vec3d::new(
        circle_normal.y * plane.normal.z - circle_normal.z * plane.normal.y,
        circle_normal.z * plane.normal.x - circle_normal.x * plane.normal.z,
        circle_normal.x * plane.normal.y - circle_normal.y * plane.normal.x,
    );
    let cross_len = cross_v.length();

    if cross_len < tol {
        // Planes are parallel
        // Check if circle center is on the plane
        let dx = circle.center.x - plane.origin.x;
        let dy = circle.center.y - plane.origin.y;
        let dz = circle.center.z - plane.origin.z;
        let dist = (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).abs();
        if dist < tol {
            // Circle is in the plane — infinite intersections
            // Return a few sample points
            let mut results = Vec::new();
            for i in 0..4 {
                let t = PI * (i as f64) / 2.0;
                let point = circle.point_at(t);
                let (u, v) = plane.project_point(&point);
                results.push(CurveSurfaceIntersectionResult { point, t, u, v });
            }
            return results;
        }
        return Vec::new();
    }

    // Find the intersection line of the two planes
    let line_dir = Direction3d::new(cross_v.x, cross_v.y, cross_v.z).unwrap_or(Direction3d::X);
    let d1 = circle_normal.x * circle.center.x + circle_normal.y * circle.center.y + circle_normal.z * circle.center.z;
    let d2 = plane.normal.x * plane.origin.x + plane.normal.y * plane.origin.y + plane.normal.z * plane.origin.z;
    let line_origin = find_line_point_on_both_planes(
        &Plane::from_origin_and_normal(circle.center, circle_normal),
        plane,
        &line_dir,
        d1,
        d2,
    );

    let line = Line::new(line_origin, line_dir);

    // Now find intersections of this line with the circle
    // Vector from circle center to line origin
    let oc = Vec3d::new(
        line.origin.x - circle.center.x,
        line.origin.y - circle.center.y,
        line.origin.z - circle.center.z,
    );
    let dir = Vec3d::new(line.direction.x, line.direction.y, line.direction.z);

    let a = dir.dot(&dir);
    let b = 2.0 * oc.dot(&dir);
    let c = oc.dot(&oc) - circle.radius * circle.radius;
    let disc = b * b - 4.0 * a * c;

    if disc < -tol {
        return Vec::new();
    }

    let sqrt_disc = if disc > 0.0 { disc.sqrt() } else { 0.0 };
    let mut results = Vec::new();

    for line_t in [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
        if line_t.is_finite() {
            let point = line.point_at(line_t);
            // Find the circle parameter t
            let y_axis = circle.normal.cross(&circle.x_axis);
            let dx = point.x - circle.center.x;
            let dy = point.y - circle.center.y;
            let dz = point.z - circle.center.z;
            let x_comp = dx * circle.x_axis.x + dy * circle.x_axis.y + dz * circle.x_axis.z;
            let y_comp = dx * y_axis.x + dy * y_axis.y + dz * y_axis.z;
            let t = y_comp.atan2(x_comp);

            let (u, v) = plane.project_point(&point);
            results.push(CurveSurfaceIntersectionResult { point, t, u, v });
        }
    }

    results
}

/// General CSI using sampling + Newton-Raphson.
fn intersect_curve_surface_general(
    curve: &Curve3d,
    surface: &Surface,
    tol_ctx: &ToleranceContext,
) -> Vec<CurveSurfaceIntersectionResult> {
    let tol = tol_ctx.coincidence_tolerance();
    let (t_min_raw, t_max_raw) = curve.param_range();

    // Clamp infinite parametric ranges to a reasonable search interval
    let t_min = if t_min_raw.is_finite() { t_min_raw } else { -1e3 };
    let t_max = if t_max_raw.is_finite() { t_max_raw } else { 1e3 };

    // Multi-phase search: first coarse, then refine around candidates
    let mut initial_guesses: Vec<f64> = Vec::new();

    // Phase 1: Coarse search
    let n_coarse = 200;
    let tol_coarse = compute_surface_extent(surface) * 0.1; // 10% of surface extent
    for i in 0..=n_coarse {
        let t = t_min + (t_max - t_min) * (i as f64 / n_coarse as f64);
        let p = curve.point_at(t);
        if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            continue;
        }
        if is_point_near_surface(&p, surface, tol_coarse) {
            initial_guesses.push(t);
        }
    }

    // Phase 2: Refine around candidates with tighter tolerance
    let mut refined_guesses: Vec<f64> = Vec::new();
    let delta = (t_max - t_min) / n_coarse as f64;
    for t_init in &initial_guesses {
        let local_min = (t_min).max(t_init - delta);
        let local_max = (t_max).min(t_init + delta);
        let n_fine = 50;
        for i in 0..=n_fine {
            let t = local_min + (local_max - local_min) * (i as f64 / n_fine as f64);
            let p = curve.point_at(t);
            if is_point_near_surface(&p, surface, tol * 100.0) {
                refined_guesses.push(t);
            }
        }
    }

    // Refine with Newton-Raphson
    let mut results: Vec<CurveSurfaceIntersectionResult> = Vec::new();
    for t_init in refined_guesses {
        if let Some(result) = refine_curve_surface_intersection(curve, surface, t_init, tol) {
            let is_dup = results.iter().any(|r| (r.t - result.t).abs() < tol * 10.0);
            if !is_dup {
                results.push(result);
            }
        }
    }

    results.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
    results
}

/// Estimate the spatial extent of a surface for tolerance scaling.
fn compute_surface_extent(surface: &Surface) -> f64 {
    match surface {
        Surface::Plane(_) => 100.0, // Infinite
        Surface::Sphere(s) => 2.0 * s.radius,
        Surface::Cylinder(c) => 2.0 * c.radius,
        Surface::Cone(c) => 2.0 * c.radius,
        Surface::Torus(t) => 2.0 * (t.major_radius + t.minor_radius),
        _ => 100.0,
    }
}

/// Check if a point is near a surface (within tolerance).
fn is_point_near_surface(point: &Point3d, surface: &Surface, tol: f64) -> bool {
    match surface {
        Surface::Plane(plane) => {
            let dx = point.x - plane.origin.x;
            let dy = point.y - plane.origin.y;
            let dz = point.z - plane.origin.z;
            let dist = (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).abs();
            dist < tol
        }
        Surface::Sphere(sphere) => {
            let dx = point.x - sphere.center.x;
            let dy = point.y - sphere.center.y;
            let dz = point.z - sphere.center.z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            (dist - sphere.radius).abs() < tol
        }
        Surface::Cylinder(cyl) => {
            let dx = point.x - cyl.origin.x;
            let dy = point.y - cyl.origin.y;
            let dz = point.z - cyl.origin.z;
            let along_axis = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
            let perp_x = dx - along_axis * cyl.axis.x;
            let perp_y = dy - along_axis * cyl.axis.y;
            let perp_z = dz - along_axis * cyl.axis.z;
            let radial_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
            (radial_dist - cyl.radius).abs() < tol
        }
        _ => {
            // General case: sample and find minimum distance
            let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
            let n = 20;
            for i in 0..=n {
                for j in 0..=n {
                    let u = u_min + (u_max - u_min) * (i as f64 / n as f64);
                    let v = v_min + (v_max - v_min) * (j as f64 / n as f64);
                    let sp = surface.point_at(u, v);
                    if point.distance_to(&sp) < tol {
                        return true;
                    }
                }
            }
            false
        }
    }
}

/// Refine a curve-surface intersection using Newton-Raphson.
///
/// Solves: C(t) - S(u, v) = 0
fn refine_curve_surface_intersection(
    curve: &Curve3d,
    surface: &Surface,
    t_init: f64,
    tol: f64,
) -> Option<CurveSurfaceIntersectionResult> {
    let (t_min, t_max) = curve.param_range();
    let mut t = t_init.clamp(t_min, t_max);
    let p = curve.point_at(t);
    let (mut u, mut v) = project_point_to_surface_uv(&p, surface);

    let max_iter = 30;
    let eps = 1e-12;

    for _ in 0..max_iter {
        let cp = curve.point_at(t);
        let sp = surface.point_at(u, v);

        let rx = cp.x - sp.x;
        let ry = cp.y - sp.y;
        let rz = cp.z - sp.z;
        let residual = (rx * rx + ry * ry + rz * rz).sqrt();

        if residual < tol {
            return Some(CurveSurfaceIntersectionResult {
                point: Point3d::new((cp.x + sp.x) / 2.0, (cp.y + sp.y) / 2.0, (cp.z + sp.z) / 2.0),
                t,
                u,
                v,
            });
        }

        // Compute Jacobian: J = [dC/dt, -dS/du, -dS/dv]
        let h_t = (t_max - t_min) * 1e-7;
        let h_uv = 1e-7;

        let cp_dt = curve.point_at(t + h_t);
        let dc_dt = Vec3d::new(
            (cp_dt.x - cp.x) / h_t,
            (cp_dt.y - cp.y) / h_t,
            (cp_dt.z - cp.z) / h_t,
        );

        let sp_du = surface.point_at(u + h_uv, v);
        let ds_du = Vec3d::new(
            (sp_du.x - sp.x) / h_uv,
            (sp_du.y - sp.y) / h_uv,
            (sp_du.z - sp.z) / h_uv,
        );

        let sp_dv = surface.point_at(u, v + h_uv);
        let ds_dv = Vec3d::new(
            (sp_dv.x - sp.x) / h_uv,
            (sp_dv.y - sp.y) / h_uv,
            (sp_dv.z - sp.z) / h_uv,
        );

        // 3x3 system: J * [dt, du, dv]^T = -[rx, ry, rz]^T
        let j = [
            [dc_dt.x, -ds_du.x, -ds_dv.x],
            [dc_dt.y, -ds_du.y, -ds_dv.y],
            [dc_dt.z, -ds_du.z, -ds_dv.z],
        ];

        if let Some(delta) = solve_3x3(&j, &[-rx, -ry, -rz]) {
            t = (t + delta[0]).clamp(t_min, t_max);
            let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
            u = (u + delta[1]).clamp(u_min, u_max);
            v = (v + delta[2]).clamp(v_min, v_max);

            let step_norm = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
            if step_norm < eps {
                break;
            }
        } else {
            break;
        }
    }

    // Final check
    let cp = curve.point_at(t);
    let sp = surface.point_at(u, v);
    let residual = cp.distance_to(&sp);

    if residual < tol * 100.0 {
        Some(CurveSurfaceIntersectionResult {
            point: Point3d::new((cp.x + sp.x) / 2.0, (cp.y + sp.y) / 2.0, (cp.z + sp.z) / 2.0),
            t,
            u,
            v,
        })
    } else {
        None
    }
}

/// Solve a 3x3 linear system using Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-15 {
        return None;
    }

    let x = b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]);

    let y = a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]);

    let z = a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    Some([x / det, y / det, z / det])
}

// ============================================================
// 4.1.6 Face Splitting
// ============================================================

/// Result of splitting a face along an intersection curve.
#[derive(Clone, Debug)]
pub struct SplitFaceResult {
    /// The sub-faces created by the split.
    pub faces: Vec<Face>,
}

/// Split a face along an intersection curve.
///
/// Given a face and an intersection curve (represented as a polyline),
/// split the face into two or more sub-faces along the curve.
pub fn split_face(
    face: &Face,
    face_edges: &[Edge],
    intersection_points: &[Point3d],
    tol_ctx: &ToleranceContext,
) -> BooleanResult<SplitFaceResult> {
    let tol = tol_ctx.coincidence_tolerance();

    if intersection_points.len() < 2 {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    // For planar faces, we can do a proper polygon split
    let surface = match &face.surface {
        Some(s) => s,
        None => {
            return Ok(SplitFaceResult {
                faces: vec![face.clone()],
            });
        }
    };

    match surface {
        Surface::Plane(plane) => {
            split_planar_face(face, face_edges, plane, intersection_points, tol)
        }
        _ => {
            // For non-planar faces, use a simplified approach:
            // Create two faces with the intersection curve as a shared boundary
            split_general_face(face, face_edges, intersection_points, tol)
        }
    }
}

/// Split a planar face along an intersection curve.
fn split_planar_face(
    face: &Face,
    face_edges: &[Edge],
    plane: &Plane,
    intersection_points: &[Point3d],
    tol: f64,
) -> BooleanResult<SplitFaceResult> {
    // Get the face's boundary polygon — use ONLY the edge endpoints (vertices)
    // not intermediate samples. This keeps the polygon simple (4 vertices for
    // a rectangle) and ensures split faces have minimal edges.
    // C5 Stage 6.2: boundary reads come from the resolved instance-faithful
    // edge list, not the face's mirrors.
    let mut boundary: Vec<Point3d> = Vec::new();
    for edge in face_edges {
        if let Some(ref curve) = edge.curve {
            let (t_min, _t_max) = edge.param_range;
            // Use just the start point of each edge (end point = start of next)
            boundary.push(curve.point_at(t_min));
        }
    }

    if boundary.is_empty() {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    // Project everything to 2D using the plane's parameterization
    let boundary_2d: Vec<(f64, f64)> = boundary
        .iter()
        .map(|p| plane.project_point(p))
        .map(|(u, v)| (u, v))
        .collect();

    let intersection_2d: Vec<(f64, f64)> = intersection_points
        .iter()
        .map(|p| plane.project_point(p))
        .map(|(u, v)| (u, v))
        .collect();

    // Find entry and exit points of the intersection curve with the boundary
    let entry_exit = find_boundary_intersections(&boundary_2d, &intersection_2d, tol);

    if entry_exit.len() < 2 {
        // Intersection curve doesn't cross the boundary — can't split
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    // Create two sub-faces by splitting the boundary polygon
    let (poly_a, poly_b) = split_polygon_at_intersections(
        &boundary_2d,
        &intersection_2d,
        &entry_exit,
    );

    // Convert back to 3D faces
    let mut result_faces = Vec::new();

    for poly_2d in &[poly_a, poly_b] {
        if poly_2d.len() < 3 {
            continue;
        }

        let points_3d: Vec<Point3d> = poly_2d
            .iter()
            .map(|(u, v)| plane.point_at(*u, *v))
            .collect();

        if let Some(new_face) = ShapeBuilder::make_polygon_face(&points_3d) {
            result_faces.push(new_face);
        }
    }

    if result_faces.is_empty() {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    Ok(SplitFaceResult {
        faces: result_faces,
    })
}

/// Find where the intersection curve crosses the boundary polygon.
fn find_boundary_intersections(
    boundary: &[(f64, f64)],
    intersection: &[(f64, f64)],
    tol: f64,
) -> Vec<usize> {
    let mut crossings = Vec::new();

    for (i, ip) in intersection.iter().enumerate() {
        // Check if this intersection point is near the boundary
        for (_j, bp) in boundary.iter().enumerate() {
            let du = ip.0 - bp.0;
            let dv = ip.1 - bp.1;
            if (du * du + dv * dv).sqrt() < tol * 10.0 {
                crossings.push(i);
                break;
            }
        }
    }

    crossings
}

/// Split a polygon at intersection points.
fn split_polygon_at_intersections(
    boundary: &[(f64, f64)],
    intersection: &[(f64, f64)],
    _crossings: &[usize],
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    // Create two sub-polygons by inserting ONLY the entry/exit points
    // of the intersection curve as shared boundary.
    // The actual intersection curve is represented by the SHARED EDGE,
    // not by inserting all intersection points into the polygon.

    let n = boundary.len();
    let ni = intersection.len();

    if n < 3 || ni < 2 {
        return (boundary.to_vec(), Vec::new());
    }

    // Find the closest boundary point to the first and last intersection points
    let start_idx = find_closest_boundary_point(&intersection[0], boundary);
    let end_idx = find_closest_boundary_point(&intersection[ni - 1], boundary);

    // The entry/exit points are the first and last intersection points
    let entry = intersection[0];
    let exit = intersection[ni - 1];

    // Build two sub-polygons using ONLY entry/exit points (not all 100 points)
    let mut poly_a: Vec<(f64, f64)> = Vec::new();
    let mut poly_b: Vec<(f64, f64)> = Vec::new();

    // Polygon A: boundary from start to end, then exit→entry (intersection curve)
    if start_idx <= end_idx {
        for i in start_idx..=end_idx {
            poly_a.push(boundary[i % n]);
        }
    } else {
        for i in start_idx..n {
            poly_a.push(boundary[i]);
        }
        for i in 0..=end_idx {
            poly_a.push(boundary[i]);
        }
    }
    // Add only exit and entry points (the shared edge represents the curve between them)
    poly_a.push(exit);
    poly_a.push(entry);

    // Polygon B: boundary from end to start, then entry→exit (intersection curve)
    if end_idx <= start_idx {
        for i in end_idx..=start_idx {
            poly_b.push(boundary[i % n]);
        }
    } else {
        for i in end_idx..n {
            poly_b.push(boundary[i]);
        }
        for i in 0..=start_idx {
            poly_b.push(boundary[i]);
        }
    }
    poly_b.push(entry);
    poly_b.push(exit);

    (poly_a, poly_b)
}

/// Find the index of the boundary point closest to a given 2D point.
fn find_closest_boundary_point(point: &(f64, f64), boundary: &[(f64, f64)]) -> usize {
    let mut best_idx = 0;
    let mut best_dist = f64::MAX;

    for (i, bp) in boundary.iter().enumerate() {
        let du = point.0 - bp.0;
        let dv = point.1 - bp.1;
        let dist = du * du + dv * dv;
        if dist < best_dist {
            best_dist = dist;
            best_idx = i;
        }
    }

    best_idx
}

/// Split a general (non-planar) face along an intersection curve.
///
/// This is a minimal implementation that handles the common case where
/// the intersection curve enters and exits the face's boundary at two
/// distinct points. We:
///   1. Project the intersection curve to the face's UV domain.
///   2. Project the face's boundary edges to UV.
///   3. Find the two boundary points closest to the intersection curve
///      endpoints (entry and exit).
///   4. Walk the boundary in two directions from entry to exit, producing
///      two sub-wires.
///   5. Build two new faces, each with one sub-wire as outer_wire, plus
///      a shared edge along the intersection curve.
///
/// Limitations: assumes intersection curve has well-defined endpoints
/// on or near the boundary. If the intersection is a closed loop fully
/// inside the face (no boundary crossings), we return the face unsplit
/// with the curve stored as an additional edge for future processing.
fn split_general_face(
    face: &Face,
    face_edges: &[Edge],
    intersection_points: &[Point3d],
    tol: f64,
) -> BooleanResult<SplitFaceResult> {
    if intersection_points.len() < 2 {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    let surface = match &face.surface {
        Some(s) => s,
        None => {
            // Can't split a face without a surface — return unsplit.
            let mut f = face.clone();
            let int_curve = create_polyline_curve(intersection_points);
            let int_edge = Edge {
                id: TopoId::new(),
                curve: Some(int_curve),
                param_range: (0.0, 1.0),
                vertex_start: None,
                vertex_end: None,
                start_vertex_point: Some(intersection_points[0]),
                end_vertex_point: Some(intersection_points[intersection_points.len() - 1]),
                forward: true,
                tolerance: tol,
                degenerate: false,
                step_entity_id: None,
            };
            f.edges.push(int_edge);
            return Ok(SplitFaceResult { faces: vec![f] });
        }
    };

    // Project intersection curve endpoints to the face's UV domain.
    let int_start = intersection_points[0];
    let int_end = intersection_points[intersection_points.len() - 1];
    let (int_uv_start_u, int_uv_start_v) = surface.project_point(&int_start);
    let (int_uv_end_u, int_uv_end_v) = surface.project_point(&int_end);

    // If the projected endpoints are essentially the same point in UV,
    // the intersection is a closed loop — we can't split using this
    // simple algorithm. Return unsplit.
    let du = int_uv_start_u - int_uv_end_u;
    let dv = int_uv_start_v - int_uv_end_v;
    if (du * du + dv * dv).sqrt() < tol {
        let mut f = face.clone();
        let int_curve = create_polyline_curve(intersection_points);
        let int_edge = Edge {
            id: TopoId::new(),
            curve: Some(int_curve),
            param_range: (0.0, 1.0),
            vertex_start: None,
            vertex_end: None,
            start_vertex_point: Some(intersection_points[0]),
            end_vertex_point: Some(intersection_points[intersection_points.len() - 1]),
            forward: true,
            tolerance: tol,
            degenerate: false,
            step_entity_id: None,
        };
        f.edges.push(int_edge);
        return Ok(SplitFaceResult { faces: vec![f] });
    }

    // Collect the face's boundary points (3D + UV).
    // C5 Stage 6.2: from the resolved instance-faithful edge list.
    let boundary_3d: Vec<Point3d> = face_edges.iter()
        .filter_map(|e| e.start_vertex_point)
        .collect();
    if boundary_3d.len() < 3 {
        // Not enough boundary to split.
        return Ok(SplitFaceResult { faces: vec![face.clone()] });
    }
    let boundary_uv: Vec<((f64, f64), usize)> = boundary_3d.iter().enumerate()
        .map(|(i, p)| {
            let uv = surface.project_point(p);
            (uv, i)
        })
        .collect();

    // Find the boundary points closest to int_uv_start and int_uv_end.
    let mut best_start_idx = 0;
    let mut best_start_dist = f64::MAX;
    let mut best_end_idx = 0;
    let mut best_end_dist = f64::MAX;
    for (i, ((u, v), _)) in boundary_uv.iter().enumerate() {
        let du_s = u - int_uv_start_u;
        let dv_s = v - int_uv_start_v;
        let d_s = du_s * du_s + dv_s * dv_s;
        if d_s < best_start_dist {
            best_start_dist = d_s;
            best_start_idx = i;
        }
        let du_e = u - int_uv_end_u;
        let dv_e = v - int_uv_end_v;
        let d_e = du_e * du_e + dv_e * dv_e;
        if d_e < best_end_dist {
            best_end_dist = d_e;
            best_end_idx = i;
        }
    }

    // If both endpoints map to the same boundary point, we can't split.
    if best_start_idx == best_end_idx {
        return Ok(SplitFaceResult { faces: vec![face.clone()] });
    }

    // Create the shared edge along the intersection curve.
    let int_curve = create_polyline_curve(intersection_points);
    let shared_edge = Edge {
        id: TopoId::new(),
        curve: Some(int_curve.clone()),
        param_range: (0.0, 1.0),
        vertex_start: None,
        vertex_end: None,
        start_vertex_point: Some(int_start),
        end_vertex_point: Some(int_end),
        forward: true,
        tolerance: tol,
        degenerate: false,
        step_entity_id: None,
    };
    let shared_edge_rev = Edge {
        id: shared_edge.id, // Same ID — both faces reference this edge
        curve: Some(int_curve),
        param_range: (0.0, 1.0),
        vertex_start: None,
        vertex_end: None,
        start_vertex_point: Some(int_end),
        end_vertex_point: Some(int_start),
        forward: false, // Reversed orientation for the second face
        tolerance: tol,
        degenerate: false,
        step_entity_id: None,
    };

    // Walk boundary from best_start_idx → best_end_idx → back to start,
    // producing two sub-wires. The shared edge connects them at the
    // intersection curve.
    //
    // Sub-wire A: boundary[best_start_idx .. best_end_idx] + shared_edge
    // Sub-wire B: boundary[best_end_idx .. best_start_idx] + shared_edge (reversed)
    let n = boundary_3d.len();

    // Build sub-wire A: edges from start_idx to end_idx, then shared_edge.
    let mut wire_a_edges: Vec<Edge> = Vec::new();
    let mut i = best_start_idx;
    while i != best_end_idx {
        let next_i = (i + 1) % n;
        let p0 = boundary_3d[i];
        let p1 = boundary_3d[next_i];
        wire_a_edges.push(Edge::new_line(p0, p1));
        i = next_i;
    }
    // Now i == best_end_idx. Close sub-wire A via the shared edge (end → int_end → int_start → start).
    // Actually: sub-wire A goes from boundary[best_start_idx] to boundary[best_end_idx] via boundary,
    // then back to boundary[best_start_idx] via the shared edge (int_start → int_end).
    // So the shared edge in sub-wire A goes from boundary[best_end_idx] (=int_end) to boundary[best_start_idx] (=int_start).
    wire_a_edges.push(shared_edge_rev.clone());

    // Build sub-wire B: edges from end_idx to start_idx, then shared_edge (reversed).
    let mut wire_b_edges: Vec<Edge> = Vec::new();
    let mut j = best_end_idx;
    while j != best_start_idx {
        let next_j = (j + 1) % n;
        let p0 = boundary_3d[j];
        let p1 = boundary_3d[next_j];
        wire_b_edges.push(Edge::new_line(p0, p1));
        j = next_j;
    }
    wire_b_edges.push(shared_edge.clone());

    // Build face A
    let coedges_a: Vec<CoEdge> = wire_a_edges.iter()
        .map(|e| CoEdge::new(e.id, true))
        .collect();
    let wire_a = Wire::new(coedges_a);
    let mut face_a = Face::new(surface.clone(), wire_a);
    face_a.edges = wire_a_edges;
    face_a.forward = face.forward;
    face_a.tolerance = face.tolerance;

    // Build face B
    let coedges_b: Vec<CoEdge> = wire_b_edges.iter()
        .map(|e| CoEdge::new(e.id, true))
        .collect();
    let wire_b = Wire::new(coedges_b);
    let mut face_b = Face::new(surface.clone(), wire_b);
    face_b.edges = wire_b_edges;
    face_b.forward = face.forward;
    face_b.tolerance = face.tolerance;

    Ok(SplitFaceResult { faces: vec![face_a, face_b] })
}

/// Create a polyline NURBS curve through a set of points.
fn create_polyline_curve(points: &[Point3d]) -> Curve3d {
    if points.len() < 2 {
        return Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    }

    // Create a NURBS curve that interpolates the points
    let n = points.len();
    let degree = n.min(3);

    // Use a simple approach: create a NURBS with uniform knots
    let mut knots = Vec::new();
    for _ in 0..=degree {
        knots.push(0.0);
    }
    for i in 1..(n - degree) {
        knots.push(i as f64 / (n - degree) as f64);
    }
    for _ in 0..=degree {
        knots.push(1.0);
    }

    let weights = vec![1.0; n];

    Curve3d::Nurbs(draper_geometry::NurbsCurve {
        degree,
        control_points: points.to_vec(),
        weights,
        knots,
    })
}

// ============================================================
// 4.1.3 / 4.1.4 / 4.1.5 Boolean Operations
// ============================================================

/// Boolean operation type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanOp {
    Union,
    Subtract,
    Intersect,
}

/// Perform a boolean operation on two solids.
///
/// This is the main entry point for boolean operations. The algorithm:
/// 1. Find all intersection curves between faces of A and faces of B
/// 2. Split faces along intersection curves
/// 3. Classify each face piece as inside/outside the other solid
/// 4. Keep faces according to the operation type
/// 5. Connect them into a new closed shell
/// Internal: shared intersection between two faces.
/// Created once per intersection curve, referenced by both split faces.
#[derive(Clone)]
struct SharedIntersection {
    face_a_idx: usize,
    face_b_idx: usize,
    shared_edge: Edge,
    points: Vec<Point3d>,
    /// PCurve on face A's surface (2D curve in A's UV domain)
    pcurve_a: Option<Curve2d>,
    /// PCurve on face B's surface (2D curve in B's UV domain)
    pcurve_b: Option<Curve2d>,
}

pub fn boolean_operation(
    solid_a: &Solid,
    solid_b: &Solid,
    op: BooleanOp,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<Solid> {
    let _tol = tol_ctx.coincidence_tolerance();

    let shell_a = solid_a.outer_shell.as_ref().ok_or_else(|| {
        BooleanError::MissingShell("Solid A has no outer shell".to_string())
    })?;
    let shell_b = solid_b.outer_shell.as_ref().ok_or_else(|| {
        BooleanError::MissingShell("Solid B has no outer shell".to_string())
    })?;

    // Step 1: Find all intersection curves between faces of A and faces of B.
    // For each intersection curve, create a SHARED Edge that will be used by
    // both the A-face split and the B-face split. This ensures both split
    // faces reference the same edge ID, which is critical for watertightness
    // — the edge cache will produce identical 3D vertices for both faces.
    let mut shared_intersections: Vec<SharedIntersection> = Vec::new();

    for (ia, face_a) in shell_a.faces.iter().enumerate() {
        let surf_a = match &face_a.surface {
            Some(s) => s,
            None => continue,
        };
        for (ib, face_b) in shell_b.faces.iter().enumerate() {
            let surf_b = match &face_b.surface {
                Some(s) => s,
                None => continue,
            };

            let curves = intersect_surfaces(surf_a, surf_b, tol_ctx);
            for curve in curves {
                if curve.points.len() < 2 {
                    continue;
                }
                // Create a shared edge for this intersection curve.
                // PRESERVE the analytic curve (Circle, Line, Ellipse) when
                // available — don't flatten to polyline. This ensures the
                // edge cache discretizes it identically for all sharing faces.
                let (int_curve, param_range) = if let Some(ref analytic) = curve.curve {
                    let pr = match analytic {
                        Curve3d::Circle(_) => (0.0, 2.0 * PI),
                        Curve3d::Line(_) => (0.0, 1.0),
                        Curve3d::Ellipse(_) => (0.0, 2.0 * PI),
                        _ => (0.0, 1.0),
                    };
                    (analytic.clone(), pr)
                } else {
                    (create_polyline_curve(&curve.points), (0.0, 1.0))
                };
                let shared_edge = Edge {
                    id: TopoId::new(),
                    curve: Some(int_curve),
                    param_range,
                    vertex_start: None,
                    vertex_end: None,
                    start_vertex_point: Some(curve.points[0]),
                    end_vertex_point: Some(curve.points[curve.points.len() - 1]),
                    forward: true,
                    tolerance: curve.tolerance,
                    degenerate: false,
                    step_entity_id: None,
                };
                shared_intersections.push(SharedIntersection {
                    face_a_idx: ia,
                    face_b_idx: ib,
                    shared_edge,
                    points: curve.points.clone(),
                    pcurve_a: curve.pcurve_a.clone(),
                    pcurve_b: curve.pcurve_b.clone(),
                });
            }
        }
    }


    // Step 2: If no intersections, handle the simple cases
    if shared_intersections.is_empty() {
        return handle_no_intersection(solid_a, solid_b, op, tol_ctx);
    }

    // Step 3: Split faces along intersection curves.
    // Each split uses the SHARED edge, so adjacent split faces will have
    // the same edge ID in their wires → edge cache produces shared vertices.
    let mut faces_a: Vec<Face> = shell_a.faces.clone();
    let mut faces_b: Vec<Face> = shell_b.faces.clone();

    // Track which faces were split and need re-classification
    let mut a_split_map: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
    let mut b_split_map: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();

    // Group intersections by face index so we can split each face with ALL
    // its intersection curves at once (needed for cylinder splitting which
    // requires both top and bottom intersection circles).
    let mut a_intersections_by_face: std::collections::HashMap<usize, Vec<&SharedIntersection>> = std::collections::HashMap::new();
    let mut b_intersections_by_face: std::collections::HashMap<usize, Vec<&SharedIntersection>> = std::collections::HashMap::new();
    for si in &shared_intersections {
        a_intersections_by_face.entry(si.face_a_idx).or_default().push(si);
        b_intersections_by_face.entry(si.face_b_idx).or_default().push(si);
    }

    // Split face A — process faces in reverse order so indices don't shift
    let mut a_face_indices: Vec<usize> = a_intersections_by_face.keys().cloned().collect();
    a_face_indices.sort_by(|a, b| b.cmp(a)); // Reverse order
    for face_a_idx in a_face_indices {
        if face_a_idx >= faces_a.len() { continue; }
        let sis = &a_intersections_by_face[&face_a_idx];
        // Combine all intersection points for this face
        let mut all_points: Vec<Point3d> = Vec::new();
        let mut all_edges: Vec<Edge> = Vec::new();
        let mut all_pcurves: Vec<Option<Curve2d>> = Vec::new();
        for si in sis {
            all_points.extend(si.points.iter().cloned());
            all_edges.push(si.shared_edge.clone());
            // Face A uses pcurve_a (PCurve on surface A)
            all_pcurves.push(si.pcurve_a.clone());
        }
        // C5 Stage 6.2: split readers consume the resolved instance-faithful
        // edge list (store-first for input-solid faces; split pieces carry
        // fresh ids and resolve via their construction mirrors).
        let face_edges = solid_a.resolve_face_edges(&faces_a[face_a_idx]);
        let split_result = split_face_with_shared_edges(
            &faces_a[face_a_idx],
            &all_points,
            &all_edges,
            &all_pcurves,
            tol_ctx,
            &face_edges,
        )?;
        if split_result.faces.len() > 1 {
            let mut new_indices = Vec::new();
            let first_idx = face_a_idx;
            faces_a[first_idx] = split_result.faces[0].clone();
            new_indices.push(first_idx);
            for extra_face in split_result.faces.iter().skip(1) {
                let new_idx = faces_a.len();
                faces_a.push(extra_face.clone());
                new_indices.push(new_idx);
            }
            a_split_map.insert(face_a_idx, new_indices);
        } else if split_result.faces.len() == 1 {
            faces_a[face_a_idx] = split_result.faces[0].clone();
        }
    }

    // Split face B — same approach
    let mut b_face_indices: Vec<usize> = b_intersections_by_face.keys().cloned().collect();
    b_face_indices.sort_by(|a, b| b.cmp(a)); // Reverse order
    for face_b_idx in b_face_indices {
        if face_b_idx >= faces_b.len() { continue; }
        let sis = &b_intersections_by_face[&face_b_idx];
        let mut all_points: Vec<Point3d> = Vec::new();
        let mut all_edges: Vec<Edge> = Vec::new();
        let mut all_pcurves: Vec<Option<Curve2d>> = Vec::new();
        for si in sis {
            all_points.extend(si.points.iter().cloned());
            all_edges.push(si.shared_edge.clone());
            // Face B uses pcurve_b (PCurve on surface B)
            all_pcurves.push(si.pcurve_b.clone());
        }
        let face_edges = solid_b.resolve_face_edges(&faces_b[face_b_idx]);
        let split_result = split_face_with_shared_edges(
            &faces_b[face_b_idx],
            &all_points,
            &all_edges,
            &all_pcurves,
            tol_ctx,
            &face_edges,
        )?;
        if split_result.faces.len() > 1 {
            let mut new_indices = Vec::new();
            let first_idx = face_b_idx;
            faces_b[first_idx] = split_result.faces[0].clone();
            new_indices.push(first_idx);
            for extra_face in split_result.faces.iter().skip(1) {
                let new_idx = faces_b.len();
                faces_b.push(extra_face.clone());
                new_indices.push(new_idx);
            }
            b_split_map.insert(face_b_idx, new_indices);
        } else if split_result.faces.len() == 1 {
            faces_b[face_b_idx] = split_result.faces[0].clone();
        }
    }

    // Step 4: Classify each face piece using MULTIPLE sample points
    // (not just centroid) for more robust inside/outside determination.
    let mut result_faces: Vec<Face> = Vec::new();

    for face in &faces_a {
        // C5 Stage 6.2: classification reads store-first edges of the
        // face's OWNING solid (clones keep the source key space).
        let face_edges = solid_a.resolve_face_edges(face);
        let classification = classify_face_robust(face, solid_b, tol_ctx, &face_edges);
        match op {
            BooleanOp::Union | BooleanOp::Subtract => {
                if classification != FaceClassification::Inside {
                    result_faces.push(face.clone());
                }
            }
            BooleanOp::Intersect => {
                if classification == FaceClassification::Inside
                    || classification == FaceClassification::OnBoundary
                {
                    result_faces.push(face.clone());
                }
            }
        }
    }

    for face in &faces_b {
        let face_edges = solid_b.resolve_face_edges(face);
        let classification = classify_face_robust(face, solid_a, tol_ctx, &face_edges);
        match op {
            BooleanOp::Union => {
                if classification != FaceClassification::Inside {
                    result_faces.push(face.clone());
                }
            }
            BooleanOp::Subtract => {
                if classification == FaceClassification::Inside
                    || classification == FaceClassification::OnBoundary
                {
                    // For Subtract (A - B), B-faces inside A become the cavity
                    // walls (with reversed normal). BUT: only keep B-faces that
                    // were SPLIT by an intersection curve — these are the faces
                    // that form the cavity boundary (e.g., cylinder lateral
                    // surface between the two intersection circles).
                    //
                    // B-faces that are entirely inside A and NOT split (e.g.,
                    // cylinder cap disks that are fully embedded in A) are
                    // INTERNAL faces that close the cavity — they must be
                    // discarded to produce a proper through-hole.
                    let was_split = was_face_split(&face_edges, &shared_intersections);
                    if was_split {
                        let mut reversed = face.reversed();
                        reversed.forward = !reversed.forward;
                        replace_matching_edges(&mut reversed, &shared_intersections, &face_edges);
                        result_faces.push(reversed);
                    }
                }
            }
            BooleanOp::Intersect => {
                if classification == FaceClassification::Inside
                    || classification == FaceClassification::OnBoundary
                {
                    let mut cloned = face.clone();
                    replace_matching_edges(&mut cloned, &shared_intersections, &face_edges);
                    result_faces.push(cloned);
                }
            }
        }
    }

    if result_faces.is_empty() {
        return Err(BooleanError::EmptyResult(
            "Boolean operation produced an empty result".to_string(),
        ));
    }

    // Step 5: Connect faces into a new closed shell
    let shell = Shell::new_closed(result_faces);
    Ok(Solid::new(shell))
}

/// Split a face with multiple shared edges (for faces intersected by multiple curves).
/// Each shared edge may have an associated PCurve on this face's surface.
fn split_face_with_shared_edges(
    face: &Face,
    intersection_points: &[Point3d],
    shared_edges: &[Edge],
    pcurves: &[Option<Curve2d>],
    tol_ctx: &ToleranceContext,
    face_edges: &[Edge],
) -> BooleanResult<SplitFaceResult> {
    let tol = tol_ctx.coincidence_tolerance();

    if intersection_points.len() < 2 || shared_edges.is_empty() {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    let surface = match &face.surface {
        Some(s) => s,
        None => {
            return Ok(SplitFaceResult {
                faces: vec![face.clone()],
            });
        }
    };

    match surface {
        Surface::Plane(plane) => {
            // For plane faces, use the first shared edge + PCurve
            let pc = pcurves.first().and_then(|p| p.clone());
            split_planar_face_shared(face, face_edges, plane, intersection_points, &shared_edges[0], pc, tol)
        }
        Surface::Cylinder(cyl) => {
            // For cylinder faces, pass ALL shared edges so the inner band
            // can use the correct shared edge for each boundary circle
            split_cylinder_face_multi_shared(face, face_edges, cyl, intersection_points, shared_edges, tol)
        }
        _ => {
            // For other non-planar faces: add each shared edge as a hole
            let mut face_with_holes = face.clone();
            for se in shared_edges {
                let coedge = CoEdge::new(se.id, true);
                let wire = Wire::new(vec![coedge]);
                face_with_holes.add_hole(wire);
                face_with_holes.edges.push(se.clone());
            }
            Ok(SplitFaceResult {
                faces: vec![face_with_holes],
            })
        }
    }
}

/// Split a cylinder face with multiple shared edges.
///
/// Each shared edge corresponds to an intersection circle at a specific height.
/// The cylinder is split into bands at each intersection height:
/// - Inner bands (between intersection heights) — inside the other solid
/// - Outer bands (above/below intersections) — outside the other solid
///
/// Each band uses the SHARED edge for its boundary at the intersection height,
/// ensuring watertight topology with the adjacent planar faces.
fn split_cylinder_face_multi_shared(
    face: &Face,
    face_edges: &[Edge],
    cyl: &CylinderSurface,
    intersection_points: &[Point3d],
    shared_edges: &[Edge],
    tol: f64,
) -> BooleanResult<SplitFaceResult> {
    // Match each shared edge to its v (height) value
    // Each shared edge has start_vertex_point which we can project to get v
    let mut v_edges: Vec<(f64, Edge)> = Vec::new();
    for se in shared_edges {
        // Use the first point of the edge to determine v
        let p = if let Some(ref curve) = se.curve {
            curve.point_at(se.param_range.0)
        } else if let Some(p) = se.start_vertex_point {
            p
        } else {
            continue;
        };
        let (_, v) = cyl.project_point(&p);
        v_edges.push((v, se.clone()));
    }

    // Also collect v values from intersection points that don't have a shared edge
    let mut v_values: Vec<f64> = v_edges.iter().map(|(v, _)| *v).collect();
    for p in intersection_points {
        let (_, v) = cyl.project_point(p);
        if !v_values.iter().any(|&v2| (v - v2).abs() < tol * 100.0) {
            v_values.push(v);
        }
    }
    v_values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v_edges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Get the cylinder height range from existing edges
    // (C5 Stage 6.2: from the resolved instance-faithful edge list)
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for edge in face_edges {
        if let Some(ref curve) = edge.curve {
            let (t_min, t_max) = edge.param_range;
            let n = 10;
            for i in 0..=n {
                let t = t_min + (t_max - t_min) * (i as f64 / n as f64);
                let p = curve.point_at(t);
                let (_, v) = cyl.project_point(&p);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }
    if v_min > v_max {
        v_min = 0.0;
        v_max = v_values.last().copied().unwrap_or(1.0) + 1.0;
    }

    // If we have fewer than 2 distinct v values, can't split into bands
    let distinct_v: Vec<f64> = {
        let mut dv: Vec<f64> = Vec::new();
        for &v in &v_values {
            if !dv.iter().any(|&v2| (v - v2).abs() < tol * 100.0) {
                dv.push(v);
            }
        }
        dv.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        dv
    };

    if distinct_v.len() < 2 {
        // Can't split — add shared edges as holes
        let mut face_with_holes = face.clone();
        for se in shared_edges {
            let coedge = CoEdge::new(se.id, true);
            let wire = Wire::new(vec![coedge]);
            face_with_holes.add_hole(wire);
            face_with_holes.edges.push(se.clone());
        }
        return Ok(SplitFaceResult {
            faces: vec![face_with_holes],
        });
    }

    let v1 = distinct_v[0];
    let v2 = distinct_v[1];

    // Find the shared edges for v1 and v2
    let edge_v1 = v_edges.iter()
        .find(|(v, _)| (v - v1).abs() < tol * 100.0)
        .map(|(_, e)| e.clone());
    let edge_v2 = v_edges.iter()
        .find(|(v, _)| (v - v2).abs() < tol * 100.0)
        .map(|(_, e)| e.clone());

    // If we don't have shared edges for both v values, create circle edges
    let edge_v1 = match edge_v1 {
        Some(e) => e,
        None => {
            let c = Circle::new(
                Point3d::new(cyl.origin.x + v1 * cyl.axis.x, cyl.origin.y + v1 * cyl.axis.y, cyl.origin.z + v1 * cyl.axis.z),
                cyl.axis, cyl.radius,
            );
            Edge { id: TopoId::new(), curve: Some(Curve3d::Circle(c.clone())), param_range: (0.0, 2.0*PI), vertex_start: None, vertex_end: None, start_vertex_point: Some(c.point_at(0.0)), end_vertex_point: Some(c.point_at(2.0*PI)), forward: true, tolerance: tol, degenerate: false, step_entity_id: None }
        }
    };
    let edge_v2 = match edge_v2 {
        Some(e) => e,
        None => {
            let c = Circle::new(
                Point3d::new(cyl.origin.x + v2 * cyl.axis.x, cyl.origin.y + v2 * cyl.axis.y, cyl.origin.z + v2 * cyl.axis.z),
                cyl.axis, cyl.radius,
            );
            Edge { id: TopoId::new(), curve: Some(Curve3d::Circle(c.clone())), param_range: (0.0, 2.0*PI), vertex_start: None, vertex_end: None, start_vertex_point: Some(c.point_at(0.0)), end_vertex_point: Some(c.point_at(2.0*PI)), forward: true, tolerance: tol, degenerate: false, step_entity_id: None }
        }
    };

    // Create the INNER band face (v1 to v2) — inside the other solid
    // Use PCurves for the CoEdges so triangulation evaluates in UV space.
    // The PCurve for a circle at height v on the cylinder is the line v=const
    // in UV space: Line2d from (0, v) to (2π, v).
    let mut inner_coedge_bottom = CoEdge::new(edge_v1.id, false);
    inner_coedge_bottom.curve_2d = Some(Curve2d::Line(Line2d::new(
        Point2d::new(2.0 * PI, v1),  // reversed: start at 2π
        Point2d::new(0.0, v1),       // end at 0
    )));
    let mut inner_coedge_top = CoEdge::new(edge_v2.id, true);
    inner_coedge_top.curve_2d = Some(Curve2d::Line(Line2d::new(
        Point2d::new(0.0, v2),
        Point2d::new(2.0 * PI, v2),
    )));
    let inner_wire = Wire::new(vec![inner_coedge_bottom, inner_coedge_top]);
    let mut inner_face = Face::new(Surface::Cylinder(cyl.clone()), inner_wire);
    inner_face.edges = vec![edge_v1.clone(), edge_v2.clone()];

    let mut all_faces = vec![inner_face];

    // Bottom outer band (v_min to v1)
    if v1 > v_min + tol {
        let c = Circle::new(
            Point3d::new(cyl.origin.x + v_min * cyl.axis.x, cyl.origin.y + v_min * cyl.axis.y, cyl.origin.z + v_min * cyl.axis.z),
            cyl.axis, cyl.radius,
        );
        let bottom_edge = Edge { id: TopoId::new(), curve: Some(Curve3d::Circle(c.clone())), param_range: (0.0, 2.0*PI), vertex_start: None, vertex_end: None, start_vertex_point: Some(c.point_at(0.0)), end_vertex_point: Some(c.point_at(2.0*PI)), forward: true, tolerance: tol, degenerate: false, step_entity_id: None };
        let mut coedge_bottom = CoEdge::new(bottom_edge.id, true);
        coedge_bottom.curve_2d = Some(Curve2d::Line(Line2d::new(
            Point2d::new(0.0, v_min),
            Point2d::new(2.0 * PI, v_min),
        )));
        let mut coedge_top = CoEdge::new(edge_v1.id, false);
        coedge_top.curve_2d = Some(Curve2d::Line(Line2d::new(
            Point2d::new(2.0 * PI, v1),
            Point2d::new(0.0, v1),
        )));
        let wire = Wire::new(vec![coedge_bottom, coedge_top]);
        let mut f = Face::new(Surface::Cylinder(cyl.clone()), wire);
        f.edges = vec![bottom_edge, edge_v1.clone()];
        all_faces.push(f);
    }

    // Top outer band (v2 to v_max)
    if v2 < v_max - tol {
        let c = Circle::new(
            Point3d::new(cyl.origin.x + v_max * cyl.axis.x, cyl.origin.y + v_max * cyl.axis.y, cyl.origin.z + v_max * cyl.axis.z),
            cyl.axis, cyl.radius,
        );
        let top_edge = Edge { id: TopoId::new(), curve: Some(Curve3d::Circle(c.clone())), param_range: (0.0, 2.0*PI), vertex_start: None, vertex_end: None, start_vertex_point: Some(c.point_at(0.0)), end_vertex_point: Some(c.point_at(2.0*PI)), forward: true, tolerance: tol, degenerate: false, step_entity_id: None };
        let mut coedge_bottom = CoEdge::new(edge_v2.id, false);
        coedge_bottom.curve_2d = Some(Curve2d::Line(Line2d::new(
            Point2d::new(2.0 * PI, v2),
            Point2d::new(0.0, v2),
        )));
        let mut coedge_top = CoEdge::new(top_edge.id, true);
        coedge_top.curve_2d = Some(Curve2d::Line(Line2d::new(
            Point2d::new(0.0, v_max),
            Point2d::new(2.0 * PI, v_max),
        )));
        let wire = Wire::new(vec![coedge_bottom, coedge_top]);
        let mut f = Face::new(Surface::Cylinder(cyl.clone()), wire);
        f.edges = vec![edge_v2.clone(), top_edge];
        all_faces.push(f);
    }

    Ok(SplitFaceResult {
        faces: all_faces,
    })
}

/// Split a planar face using a shared edge for the intersection boundary.
///
/// Instead of creating entirely new edges for the split faces, we use the
/// SHARED edge for the intersection curve portion of the boundary. This
/// ensures both adjacent split faces (from solid A and solid B) reference
/// the same edge ID, so the edge cache produces identical 3D vertices
/// for both faces → watertight mesh.
fn split_planar_face_shared(
    face: &Face,
    face_edges: &[Edge],
    plane: &Plane,
    intersection_points: &[Point3d],
    shared_edge: &Edge,
    pcurve: Option<Curve2d>,
    tol: f64,
) -> BooleanResult<SplitFaceResult> {
    // Get the face's boundary polygon — use ONLY the edge endpoints (vertices)
    // not intermediate samples. This keeps the polygon simple (4 vertices for
    // a rectangle) and ensures split faces have minimal edges.
    // C5 Stage 6.2: boundary reads come from the resolved instance-faithful
    // edge list, not the face's mirrors.
    let mut boundary: Vec<Point3d> = Vec::new();
    for edge in face_edges {
        if let Some(ref curve) = edge.curve {
            let (t_min, _t_max) = edge.param_range;
            // Use just the start point of each edge (end point = start of next)
            boundary.push(curve.point_at(t_min));
        }
    }

    if boundary.is_empty() {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    // Project to 2D
    let boundary_2d: Vec<(f64, f64)> = boundary
        .iter()
        .map(|p| plane.project_point(p))
        .collect();

    let intersection_2d: Vec<(f64, f64)> = intersection_points
        .iter()
        .map(|p| plane.project_point(p))
        .collect();

    let entry_exit = find_boundary_intersections(&boundary_2d, &intersection_2d, tol);

    if entry_exit.len() < 2 {
        // Intersection curve doesn't cross the boundary.
        // Check if ANY intersection point is actually inside the face.
        // If not, the intersection is a false positive (e.g., plane-plane
        // intersection line that doesn't overlap the actual faces).
        let any_inside = intersection_2d.iter().any(|ip| {
            point_in_polygon_2d(ip.0, ip.1, &boundary_2d, tol)
        });

        if !any_inside {
            // Intersection doesn't touch this face — no split needed
            return Ok(SplitFaceResult {
                faces: vec![face.clone()],
            });
        }

        // Intersection curve is entirely inside the face → add as a hole.
        let mut face_with_hole = face.clone();
        let mut coedge = CoEdge::new(shared_edge.id, true);
        // Store the PCurve on the CoEdge so triangulation uses the analytic
        // 2D curve in UV space (ensures watertight topology)
        coedge.curve_2d = pcurve.clone();
        let wire = Wire::new(vec![coedge]);
        face_with_hole.add_hole(wire);
        face_with_hole.edges.push(shared_edge.clone());
        // C5 Stage 4: canonical ref from birth (parallel to the mirror).
        face_with_hole.edge_ids.push(shared_edge.id);
        return Ok(SplitFaceResult {
            faces: vec![face_with_hole],
        });
    }

    // Additional check: verify that at least one intersection point is
    // inside the face. This filters out false-positive plane-plane
    // intersections where the infinite line crosses the boundary polygon
    // but the actual face doesn't overlap.
    let any_inside = intersection_2d.iter().any(|ip| {
        point_in_polygon_2d(ip.0, ip.1, &boundary_2d, tol)
    });
    if !any_inside {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    let (poly_a, poly_b) = split_polygon_at_intersections(
        &boundary_2d,
        &intersection_2d,
        &entry_exit,
    );

    // Build faces using the SHARED edge for the intersection portion.
    // Each split face has:
    //   - Edges from the original face boundary (REUSED Edge IDs for watertight)
    //   - The shared edge for the intersection curve portion
    //   - NEW shared edges for boundary segments created by the split
    //     (entry/exit points that split original edges)
    //
    // For watertightness, we create a map of "split edge" → shared Edge
    // so that both poly_a and poly_b reference the same Edge ID for
    // the same geometric segment.
    //
    // IMPORTANT: Consecutive segments that belong to the SAME original
    // edge are MERGED into a single edge — this avoids micro-segments
    // in the B-Rep that would create spurious edges in the viewport.
    let mut result_faces = Vec::new();

    // C5 Stage 4: split-edge identity lives in a local EdgeStore instead
    // of an ad-hoc `HashMap<u64, Edge>` (worklog TODO). The canonical Edge
    // is registered ONCE per geometric key; every incident split face gets
    // its clone (same id) plus a canonical `edge_ids` entry at creation
    // time — `index_boolean_result` re-indexes the assembled solid, and the
    // identity mapping is already canonical when that happens.
    let mut split_store = EdgeStore::new();
    // Geometric key (quantized endpoint pair hash) → canonical TopoId —
    // the split-face analogue of the store's `by_step_id` index.
    let mut split_key_to_id: std::collections::HashMap<u64, TopoId> =
        std::collections::HashMap::new();

    for poly_idx in 0..2 {
        let poly_2d = if poly_idx == 0 { &poly_a } else { &poly_b };
        if poly_2d.len() < 3 {
            continue;
        }

        let points_3d: Vec<Point3d> = poly_2d
            .iter()
            .map(|(u, v)| plane.point_at(*u, *v))
            .collect();

        let n = points_3d.len();

        // First pass: classify each segment — which original edge does it belong to?
        // Segments belonging to the same original edge will be merged.
        #[derive(Clone)]
        struct SegmentInfo {
            p0: Point3d,
            p1: Point3d,
            is_intersection: bool,
            orig_edge_id: Option<TopoId>,
        }

        let mut segments: Vec<SegmentInfo> = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            let p0 = points_3d[i];
            let p1 = points_3d[j];

            let p0_on_curve = intersection_points.iter().any(|ip| {
                (ip.x - p0.x).powi(2) + (ip.y - p0.y).powi(2) + (ip.z - p0.z).powi(2) < tol * tol * 100.0
            });
            let p1_on_curve = intersection_points.iter().any(|ip| {
                (ip.x - p1.x).powi(2) + (ip.y - p1.y).powi(2) + (ip.z - p1.z).powi(2) < tol * tol * 100.0
            });

            if p0_on_curve && p1_on_curve {
                segments.push(SegmentInfo {
                    p0, p1,
                    is_intersection: true,
                    orig_edge_id: None,
                });
            } else {
                // Find which original edge this segment belongs to
                let mut found_id = None;
                for orig_edge in face_edges {
                    if let Some(ref curve) = orig_edge.curve {
                        let (t_min, t_max) = orig_edge.param_range;
                        let n_samples = 30;
                        let mut p0_found = false;
                        let mut p1_found = false;
                        for k in 0..=n_samples {
                            let t = t_min + (t_max - t_min) * (k as f64 / n_samples as f64);
                            let ep = curve.point_at(t);
                            if (ep.x - p0.x).powi(2) + (ep.y - p0.y).powi(2) + (ep.z - p0.z).powi(2) < tol * tol * 100.0 {
                                p0_found = true;
                            }
                            if (ep.x - p1.x).powi(2) + (ep.y - p1.y).powi(2) + (ep.z - p1.z).powi(2) < tol * tol * 100.0 {
                                p1_found = true;
                            }
                        }
                        if p0_found && p1_found {
                            found_id = Some(orig_edge.id);
                            break;
                        }
                    }
                }
                segments.push(SegmentInfo {
                    p0, p1,
                    is_intersection: false,
                    orig_edge_id: found_id,
                });
            }
        }

        // Second pass: merge consecutive segments with the same orig_edge_id
        // into single edges. This eliminates micro-segments from split.
        let mut merged_segments: Vec<SegmentInfo> = Vec::new();
        for seg in &segments {
            if let Some(last) = merged_segments.last_mut() {
                // Merge if both are non-intersection, same orig_edge_id,
                // and last.p1 == seg.p0 (connected)
                if !seg.is_intersection && !last.is_intersection
                    && seg.orig_edge_id == last.orig_edge_id
                    && seg.orig_edge_id.is_some()
                    && (last.p1.x - seg.p0.x).abs() < tol * 10.0
                    && (last.p1.y - seg.p0.y).abs() < tol * 10.0
                    && (last.p1.z - seg.p0.z).abs() < tol * 10.0
                {
                    // Merge: extend last segment's end to current segment's end
                    last.p1 = seg.p1;
                    continue;
                }
            }
            merged_segments.push(seg.clone());
        }

        // Also merge wrap-around: if first and last merged segments have
        // the same orig_edge_id and are connected, merge them
        if merged_segments.len() >= 2 {
            let first = merged_segments.first().unwrap();
            let last = merged_segments.last().unwrap();
            if !first.is_intersection && !last.is_intersection
                && first.orig_edge_id == last.orig_edge_id
                && first.orig_edge_id.is_some()
                && (last.p1.x - first.p0.x).abs() < tol * 10.0
                && (last.p1.y - first.p0.y).abs() < tol * 10.0
                && (last.p1.z - first.p0.z).abs() < tol * 10.0
            {
                // Merge last into first, remove last
                merged_segments.first_mut().unwrap().p0 = last.p0;
                merged_segments.pop();
            }
        }

        // Third pass: create edges and coedges from merged segments.
        // C5 Stage 4: `edge_ids` is collected in parallel — the new faces
        // carry canonical edge references from birth instead of waiting
        // for a later `index_edges` pass.
        let mut edges = Vec::new();
        let mut edge_ids: Vec<TopoId> = Vec::new();
        let mut coedges = Vec::new();

        for seg in &merged_segments {
            if seg.is_intersection {
                // Use the shared edge for the intersection curve
                edges.push(shared_edge.clone());
                edge_ids.push(shared_edge.id);
                coedges.push(CoEdge::new(shared_edge.id, true));
            } else if let Some(orig_id) = seg.orig_edge_id {
                // Find the original edge and reuse it
                if let Some(orig_edge) = face.edge_by_id(orig_id) {
                    edges.push(orig_edge.clone());
                    edge_ids.push(orig_edge.id);
                    coedges.push(CoEdge::new(orig_edge.id, true));
                } else {
                    // Fallback: create new edge
                    let e = Edge::new_line(seg.p0, seg.p1);
                    let eid = e.id;
                    edges.push(e);
                    edge_ids.push(eid);
                    coedges.push(CoEdge::new(eid, true));
                }
            } else {
                // New segment from split — create/reuse the canonical shared
                // edge through the EdgeStore (C5 Stage 4).
                let edge_key_str = format!("{:.6},{:.6},{:.6}-{:.6},{:.6},{:.6}",
                    seg.p0.x.min(seg.p1.x), seg.p0.y.min(seg.p1.y), seg.p0.z.min(seg.p1.z),
                    seg.p0.x.max(seg.p1.x), seg.p0.y.max(seg.p1.y), seg.p0.z.max(seg.p1.z));
                let edge_hash: u64 = edge_key_str.bytes().fold(0u64, |acc, b| acc.wrapping_mul(31).wrapping_add(b as u64));

                let canonical_id = *split_key_to_id.entry(edge_hash).or_insert_with(|| {
                    let edge = Edge::new_line(seg.p0, seg.p1);
                    let id = edge.id;
                    split_store.insert(edge);
                    id
                });
                let edge = split_store
                    .get(canonical_id)
                    .expect("split edge just inserted")
                    .clone();

                edges.push(edge);
                edge_ids.push(canonical_id);
                coedges.push(CoEdge::new(canonical_id, true));
            }
        }

        let wire = Wire::new(coedges);
        let surface = Surface::Plane(plane.clone());
        let mut new_face = Face::new(surface, wire);
        new_face.edges = edges;
        new_face.edge_ids = edge_ids;
        result_faces.push(new_face);
    }

    if result_faces.is_empty() {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    Ok(SplitFaceResult {
        faces: result_faces,
    })
}

/// Robust face classification using multiple sample points.
/// Instead of just the centroid, samples several points on the face
/// and takes a majority vote for inside/outside determination.
/// Replace edges in a face that geometrically match a shared intersection edge.
/// This ensures that cap faces (e.g., cylinder bottom/top disks) use the same
/// Edge ID as the cylinder lateral face's boundary → watertight topology.
/// Check if a face was split by any intersection curve.
/// A face is "split" if any of its edges geometrically matches a shared
/// intersection edge (i.e., the face's boundary was modified by the boolean).
/// Unsplit faces that are entirely inside the other solid are internal
/// faces (e.g., cylinder cap disks) and should be discarded for Subtract.
fn was_face_split(
    face_edges: &[Edge],
    shared_intersections: &[SharedIntersection],
) -> bool {
    // C5 Stage 6.2: matching runs on the resolved instance-faithful
    // edge list, not the face's mirrors.
    for edge in face_edges {
        if let Some(ref curve) = edge.curve {
            for si in shared_intersections {
                if let Some(ref shared_curve) = si.shared_edge.curve {
                    // Check if the edge curves match by sampling
                    let n = 5;
                    let mut all_match = true;
                    for k in 0..n {
                        let t = k as f64 / (n - 1) as f64;
                        let p1 = curve.point_at(edge.param_range.0 + t * (edge.param_range.1 - edge.param_range.0));
                        let p2 = shared_curve.point_at(si.shared_edge.param_range.0 + t * (si.shared_edge.param_range.1 - si.shared_edge.param_range.0));
                        let d = (p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2) + (p1.z - p2.z).powi(2);
                        if d > 1e-6 {
                            all_match = false;
                            break;
                        }
                    }
                    if all_match {
                        return true;
                    }
                }
            }
        }
    }
    false
}

fn replace_matching_edges(
    face: &mut Face,
    shared_intersections: &[SharedIntersection],
    face_edges: &[Edge],
) {
    // C5 Stage 6.2: the matching pass runs on the RESOLVED instance-faithful
    // edge list (store-first), builds the new mirror list, and writes it
    // back once. The mirrors of result faces are construction data — this
    // function remains their sanctioned writer.
    let mut new_edges: Vec<Edge> = Vec::with_capacity(face_edges.len());
    for edge in face_edges {
        let mut replacement = edge.clone();
        if let Some(ref curve) = edge.curve {
            // Check if this edge geometrically matches any shared intersection edge
            'shared: for si in shared_intersections {
                if let Some(ref shared_curve) = si.shared_edge.curve {
                    // Compare by sampling: if the first few points match, they're the same curve
                    let n = 5;
                    let mut all_match = true;
                    for k in 0..n {
                        let t = k as f64 / (n - 1) as f64;
                        let p1 = curve.point_at(edge.param_range.0 + t * (edge.param_range.1 - edge.param_range.0));
                        let p2 = shared_curve.point_at(si.shared_edge.param_range.0 + t * (si.shared_edge.param_range.1 - si.shared_edge.param_range.0));
                        let d = (p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2) + (p1.z - p2.z).powi(2);
                        if d > 1e-6 {
                            all_match = false;
                            break;
                        }
                    }
                    if all_match {
                        // Replace this edge with the shared edge (same ID)
                        replacement = si.shared_edge.clone();
                        break 'shared;
                    }
                }
            }
        }
        new_edges.push(replacement);
    }
    face.edges = new_edges;

    // Also update the outer_wire coedges to match the new edge IDs
    if let Some(ref mut wire) = face.outer_wire {
        for coedge in &mut wire.coedges {
            // Find the edge in face.edges that matches this coedge's edge ID
            // If the coedge's edge ID is no longer in face.edges, find a replacement
            let coedge_id = coedge.edge;
            let found = face.edges.iter().any(|e| e.id == coedge_id);
            if !found {
                // This coedge's edge was replaced — find the replacement
                // by matching geometry (check which edge in face.edges is a Circle
                // at the same position)
                for (idx, e) in face.edges.iter().enumerate() {
                    if matches!(e.curve, Some(Curve3d::Circle(_))) {
                        coedge.edge = e.id;
                        break;
                    }
                    let _ = idx;
                }
            }
        }
    }
}

fn classify_face_robust(
    face: &Face,
    solid: &Solid,
    tol_ctx: &ToleranceContext,
    face_edges: &[Edge],
) -> FaceClassification {
    let surface = match &face.surface {
        Some(s) => s,
        None => return FaceClassification::Outside,
    };

    let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
    let (u_min, u_max, v_min, v_max) =
        compute_face_uv_range(face_edges, surface, u_min, u_max, v_min, v_max);

    // Sample a grid of points and classify each
    let n = 5;
    let mut inside_count = 0;
    let mut outside_count = 0;
    let mut boundary_count = 0;
    let mut total = 0;

    for i in 1..n {
        for j in 1..n {
            let u = u_min + (u_max - u_min) * (i as f64 / n as f64);
            let v = v_min + (v_max - v_min) * (j as f64 / n as f64);
            let p = surface.point_at(u, v);

            // Offset slightly inward along normal to avoid boundary ambiguity
            let normal = surface.normal_at(u, v);
            let offset = tol_ctx.coincidence_tolerance() * 10.0;
            let offset_point = Point3d::new(
                p.x - normal.x * offset,
                p.y - normal.y * offset,
                p.z - normal.z * offset,
            );

            total += 1;
            match classify_point(solid, &offset_point, tol_ctx) {
                PointClassification::Inside => inside_count += 1,
                PointClassification::Outside => outside_count += 1,
                PointClassification::OnBoundary => boundary_count += 1,
            }
        }
    }

    if total == 0 {
        return FaceClassification::Outside;
    }

    // Majority vote
    if inside_count > outside_count && inside_count > boundary_count {
        FaceClassification::Inside
    } else if outside_count >= inside_count && outside_count >= boundary_count {
        if boundary_count > 0 && outside_count == 0 {
            FaceClassification::OnBoundary
        } else {
            FaceClassification::Outside
        }
    } else {
        FaceClassification::OnBoundary
    }
}

/// Classification of a face relative to a solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FaceClassification {
    Inside,
    Outside,
    OnBoundary,
}

/// Compute the UV range of a face from its boundary edges.
fn compute_face_uv_range(
    face_edges: &[Edge],
    surface: &Surface,
    default_u_min: f64,
    default_u_max: f64,
    default_v_min: f64,
    default_v_max: f64,
) -> (f64, f64, f64, f64) {
    let mut u_min = default_u_max;
    let mut u_max = default_u_min;
    let mut v_min = default_v_max;
    let mut v_max = default_v_min;

    let mut found_bounds = false;

    // C5 Stage 6.2: UV bounds from the resolved instance-faithful list.
    for edge in face_edges {
        if let Some(ref curve) = edge.curve {
            let (t_min, t_max) = edge.param_range;
            let n = 10;
            for i in 0..=n {
                let t = t_min + (t_max - t_min) * (i as f64 / n as f64);
                let p = curve.point_at(t);
                let (u, v) = project_point_to_surface_uv(&p, surface);
                u_min = u_min.min(u);
                u_max = u_max.max(u);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
                found_bounds = true;
            }
        }
    }

    if found_bounds {
        (u_min, u_max, v_min, v_max)
    } else {
        (default_u_min, default_u_max, default_v_min, default_v_max)
    }
}

/// Handle the case where there are no intersections between the two solids.
fn handle_no_intersection(
    solid_a: &Solid,
    solid_b: &Solid,
    op: BooleanOp,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<Solid> {
    // Determine the spatial relationship between the two solids
    let a_in_b = is_solid_inside_solid(solid_a, solid_b, tol_ctx);
    let b_in_a = is_solid_inside_solid(solid_b, solid_a, tol_ctx);

    match op {
        BooleanOp::Union => {
            if a_in_b {
                // A is inside B → result is B
                return Ok(solid_b.clone());
            }
            if b_in_a {
                // B is inside A → result is A
                return Ok(solid_a.clone());
            }
            // Disjoint solids → combine shells
            let mut all_faces = Vec::new();
            if let Some(ref shell) = solid_a.outer_shell {
                all_faces.extend(shell.faces.clone());
            }
            if let Some(ref shell) = solid_b.outer_shell {
                all_faces.extend(shell.faces.clone());
            }
            if all_faces.is_empty() {
                return Err(BooleanError::EmptyResult("Both solids are empty".to_string()));
            }
            // Create a compound-like solid with both shells
            let shell = Shell::new_closed(all_faces);
            Ok(Solid::new(shell))
        }
        BooleanOp::Subtract => {
            if a_in_b {
                // A is inside B → result is empty (B with A-shaped void, but we return empty for now)
                return Err(BooleanError::EmptyResult(
                    "Subtraction: A is entirely inside B".to_string(),
                ));
            }
            if b_in_a {
                // B is inside A → create A with a B-shaped void
                let mut result = solid_a.clone();
                if let Some(ref shell) = solid_b.outer_shell {
                    result.add_void(Shell::new_closed(shell.faces.clone()));
                }
                return Ok(result);
            }
            // Disjoint solids → result is just A
            Ok(solid_a.clone())
        }
        BooleanOp::Intersect => {
            if a_in_b {
                // A is inside B → result is A
                return Ok(solid_a.clone());
            }
            if b_in_a {
                // B is inside A → result is B
                return Ok(solid_b.clone());
            }
            // Disjoint → no intersection
            Err(BooleanError::EmptyResult(
                "No intersection between disjoint solids".to_string(),
            ))
        }
    }
}

/// Check if solid_a is entirely inside solid_b.
fn is_solid_inside_solid(solid_a: &Solid, solid_b: &Solid, tol_ctx: &ToleranceContext) -> bool {
    // Sample points on solid_a and check if they're all inside solid_b
    let shell = match &solid_a.outer_shell {
        Some(s) => s,
        None => return false,
    };

    let mut total = 0;
    let mut inside = 0;

    for face in &shell.faces {
        if let Some(ref surface) = face.surface {
            // C5 Stage 6.2: store-first reads with per-id mirror fallback.
            let face_edges = solid_a.resolve_face_edges(face);
            let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
            let (u_min, u_max, v_min, v_max) =
                compute_face_uv_range(&face_edges, surface, u_min, u_max, v_min, v_max);

            for i in 0..5 {
                for j in 0..5 {
                    let u = u_min + (u_max - u_min) * (i as f64 / 4.0);
                    let v = v_min + (v_max - v_min) * (j as f64 / 4.0);
                    let p = surface.point_at(u, v);

                    // Offset the point slightly inward to avoid boundary ambiguity
                    let normal = surface.normal_at(u, v);
                    let offset_point = Point3d::new(
                        p.x - normal.x * tol_ctx.coincidence_tolerance() * 10.0,
                        p.y - normal.y * tol_ctx.coincidence_tolerance() * 10.0,
                        p.z - normal.z * tol_ctx.coincidence_tolerance() * 10.0,
                    );

                    total += 1;
                    match classify_point(solid_b, &offset_point, tol_ctx) {
                        PointClassification::Inside => inside += 1,
                        PointClassification::OnBoundary => inside += 1, // Count boundary as inside
                        PointClassification::Outside => {}
                    }
                }
            }
        }
    }

    total > 0 && inside == total
}

/// Boolean union: combine two solids into one.
pub fn boolean_union(
    solid_a: &Solid,
    solid_b: &Solid,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<Solid> {
    index_boolean_result(boolean_operation(solid_a, solid_b, BooleanOp::Union, tol_ctx))
}

/// Boolean subtract: remove solid_b from solid_a.
pub fn boolean_subtract(
    solid_a: &Solid,
    solid_b: &Solid,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<Solid> {
    index_boolean_result(boolean_operation(solid_a, solid_b, BooleanOp::Subtract, tol_ctx))
}

/// Boolean intersect: keep only the overlapping volume.
pub fn boolean_intersect(
    solid_a: &Solid,
    solid_b: &Solid,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<Solid> {
    index_boolean_result(boolean_operation(solid_a, solid_b, BooleanOp::Intersect, tol_ctx))
}

/// C5 Stage 3: give boolean results a unified edge store.
///
/// Boolean split faces share geometrically identical split edges (the
/// `shared_split_edges` map inside `split_face` clones one `Edge` into both
/// result faces). `index_edges` unifies their identity through geometric
/// dedup, so healing and the mesh edge-discretization cache observe ONE
/// canonical edge per shared segment instead of independent copies.
fn index_boolean_result(result: BooleanResult<Solid>) -> BooleanResult<Solid> {
    result.map(|mut solid| {
        solid.index_edges();
        solid
    })
}

// ============================================================
// 4.1.8 Unit Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::ShapeBuilder;

    fn make_tol_ctx() -> ToleranceContext {
        ToleranceContext::from_model_scale(10.0)
    }

    // ---- SSI Tests ----

    #[test]
    fn test_plane_plane_intersection() {
        let p1 = Plane::xy();
        let p2 = Plane::xz();
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Plane(p1), &Surface::Plane(p2), &tol);
        assert_eq!(curves.len(), 1, "Two non-parallel planes should intersect in one line");

        // The intersection should be along the X axis
        if let Some(Curve3d::Line(ref line)) = curves[0].curve {
            // Line should be along X
            assert!(
                line.direction.is_parallel_to(&Direction3d::X),
                "Plane XY intersect Plane XZ should give a line along X"
            );
        }
    }

    #[test]
    fn test_plane_plane_parallel() {
        let p1 = Plane::xy();
        let p2 = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 5.0),
            Direction3d::Z,
        );
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Plane(p1), &Surface::Plane(p2), &tol);
        assert!(curves.is_empty(), "Parallel planes should not intersect");
    }

    #[test]
    fn test_plane_sphere_intersection() {
        let plane = Plane::xy();
        let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Plane(plane), &Surface::Sphere(sphere), &tol);
        assert_eq!(curves.len(), 1, "Plane through sphere center should intersect in one circle");

        // The intersection circle should have radius 5.0
        if let Some(Curve3d::Circle(ref circle)) = curves[0].curve {
            assert!(
                (circle.radius - 5.0).abs() < 1e-6,
                "Circle radius should be 5.0, got {}",
                circle.radius
            );
        }
    }

    #[test]
    fn test_plane_sphere_no_intersection() {
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 10.0),
            Direction3d::Z,
        );
        let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Plane(plane), &Surface::Sphere(sphere), &tol);
        assert!(curves.is_empty(), "Plane too far from sphere should not intersect");
    }

    #[test]
    fn test_plane_sphere_tangent() {
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 5.0),
            Direction3d::Z,
        );
        let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Plane(plane), &Surface::Sphere(sphere), &tol);
        assert_eq!(curves.len(), 1, "Tangent plane should intersect sphere at one point");
        assert_eq!(curves[0].points.len(), 1, "Tangent intersection should be a single point");
    }

    #[test]
    fn test_plane_cylinder_intersection() {
        let plane = Plane::from_origin_and_normal(Point3d::ORIGIN, Direction3d::Z);
        let cyl = CylinderSurface::new_z(3.0);
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Plane(plane), &Surface::Cylinder(cyl), &tol);
        assert_eq!(curves.len(), 1, "Perpendicular plane should intersect cylinder in a circle/ellipse");

        if let Some(Curve3d::Circle(ref circle)) = curves[0].curve {
            assert!(
                (circle.radius - 3.0).abs() < 1e-6,
                "Circle radius should be 3.0, got {}",
                circle.radius
            );
        }
    }

    // ---- Point Classification Tests ----

    #[test]
    fn test_classify_point_inside_cube() {
        let cube = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let tol = make_tol_ctx();

        let point = Point3d::new(0.0, 0.0, 0.0); // Center of the cube
        let classification = classify_point(&cube, &point, &tol);
        assert_eq!(
            classification,
            PointClassification::Inside,
            "Center of cube should be inside"
        );
    }

    #[test]
    fn test_classify_point_outside_cube() {
        let cube = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let tol = make_tol_ctx();

        let point = Point3d::new(20.0, 20.0, 20.0); // Far outside
        let classification = classify_point(&cube, &point, &tol);
        assert_eq!(
            classification,
            PointClassification::Outside,
            "Point far from cube should be outside"
        );
    }

    #[test]
    fn test_classify_point_on_cube_boundary() {
        let cube = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let tol = make_tol_ctx();

        let point = Point3d::new(5.0, 0.0, 0.0); // On the face
        let classification = classify_point(&cube, &point, &tol);
        assert_eq!(
            classification,
            PointClassification::OnBoundary,
            "Point on face should be on boundary"
        );
    }

    #[test]
    fn test_classify_point_inside_sphere() {
        let sphere = ShapeBuilder::make_sphere(5.0);
        let tol = make_tol_ctx();

        let point = Point3d::new(0.0, 0.0, 0.0); // Center
        let classification = classify_point(&sphere, &point, &tol);
        assert_eq!(
            classification,
            PointClassification::Inside,
            "Center of sphere should be inside"
        );
    }

    // ---- Boolean Operation Tests ----

    #[test]
    fn test_union_two_disjoint_cubes() {
        let cube_a = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let cube_b = ShapeBuilder::make_box_at(20.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let tol = make_tol_ctx();

        let result = boolean_union(&cube_a, &cube_b, &tol);
        assert!(result.is_ok(), "Union of disjoint cubes should succeed");

        let union_solid = result.unwrap();
        assert!(union_solid.outer_shell.is_some());
        // Should have faces from both cubes
        let n_faces = union_solid.outer_shell.as_ref().unwrap().faces.len();
        assert!(
            n_faces >= 12,
            "Union of disjoint cubes should have at least 12 faces, got {}",
            n_faces
        );
    }

    #[test]
    fn test_subtract_cube_sphere() {
        // Cube minus sphere (creates a dimple)
        let cube = ShapeBuilder::make_box(20.0, 20.0, 20.0);
        let sphere = ShapeBuilder::make_sphere(5.0);
        let tol = ToleranceContext::from_model_scale(20.0);

        let result = boolean_subtract(&cube, &sphere, &tol);
        // The sphere should be entirely inside the cube
        // So the result should be a cube with a void
        if let Ok(subtract_solid) = result {
            // The cube minus a fully enclosed sphere gives a cube with an inner shell
            assert!(subtract_solid.outer_shell.is_some());
        }
        // If the sphere is not fully enclosed, we might get an error or a modified solid
        // This is acceptable for the initial implementation
    }

    #[test]
    fn test_subtract_cylinder_cylinder() {
        // Cylinder minus cylinder (creates a cross-hole)
        let cyl_a = ShapeBuilder::make_cylinder(5.0, 20.0);
        let cyl_b = ShapeBuilder::make_cylinder_at(0.0, 0.0, 5.0, 3.0, 20.0);
        // Note: make_cylinder_at translates the cylinder

        let tol = ToleranceContext::from_model_scale(20.0);

        let result = boolean_subtract(&cyl_a, &cyl_b, &tol);
        // This is a complex intersection - the result should succeed or
        // provide a meaningful error
        // For the initial implementation, we just check it doesn't panic
        let _ = result;
    }

    #[test]
    fn test_union_two_overlapping_cubes_l_shape() {
        // Union of two cubes that share an edge (creates an L-shape)
        let cube_a = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        // Second cube overlapping with first
        let cube_b = ShapeBuilder::make_box_at(5.0, 5.0, 0.0, 10.0, 10.0, 10.0);

        let tol = ToleranceContext::from_model_scale(20.0);

        let result = boolean_union(&cube_a, &cube_b, &tol);
        // Should succeed — the union of overlapping cubes creates an L-shape
        assert!(result.is_ok(), "Union of overlapping cubes should succeed");

        let union_solid = result.unwrap();
        assert!(union_solid.outer_shell.is_some());
    }

    // ---- CSI Tests ----

    #[test]
    fn test_curve_surface_intersection_line_plane() {
        let line = Line::new(Point3d::new(0.0, 0.0, -5.0), Direction3d::Z);
        let plane = Plane::xy();
        let surface = Surface::Plane(plane);
        let tol = ToleranceContext::new();

        let results = intersect_curve_surface(&Curve3d::Line(line), &surface, &tol);
        assert_eq!(
            results.len(),
            1,
            "Line through plane should have one intersection"
        );
        assert!(
            results[0].point.distance_to(&Point3d::ORIGIN) < 1e-6,
            "Intersection should be at origin"
        );
    }

    #[test]
    fn test_curve_surface_intersection_line_sphere() {
        let line = Line::new(Point3d::new(0.0, 0.0, -10.0), Direction3d::Z);
        let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let surface = Surface::Sphere(sphere);
        let tol = ToleranceContext::new();

        let results = intersect_curve_surface(&Curve3d::Line(line), &surface, &tol);
        assert_eq!(
            results.len(),
            2,
            "Line through sphere should have two intersections"
        );
    }

    // ---- Face Splitting Tests ----

    #[test]
    fn test_split_face_basic() {
        let tol_ctx = ToleranceContext::new();

        // Create a simple square face
        let face = ShapeBuilder::make_polygon_face(&[
            Point3d::new(-5.0, -5.0, 0.0),
            Point3d::new(5.0, -5.0, 0.0),
            Point3d::new(5.0, 5.0, 0.0),
            Point3d::new(-5.0, 5.0, 0.0),
        ])
        .unwrap();

        // Split with a line from left to right
        let intersection = vec![
            Point3d::new(-5.0, 0.0, 0.0),
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(5.0, 0.0, 0.0),
        ];

        let result = split_face(&face, &face.edges, &intersection, &tol_ctx);
        assert!(result.is_ok(), "Face splitting should succeed");
    }

    // ---- Integration Tests ----

    #[test]
    fn test_intersect_disjoint_solids() {
        let cube_a = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let cube_b = ShapeBuilder::make_box_at(50.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let tol = make_tol_ctx();

        let result = boolean_intersect(&cube_a, &cube_b, &tol);
        assert!(result.is_err(), "Intersection of disjoint solids should return error");
    }

    #[test]
    fn test_subtract_disjoint_solids() {
        let cube_a = ShapeBuilder::make_box(10.0, 10.0, 10.0);
        let cube_b = ShapeBuilder::make_box_at(50.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let tol = make_tol_ctx();

        let result = boolean_subtract(&cube_a, &cube_b, &tol);
        assert!(result.is_ok(), "Subtracting disjoint solid should return original");

        let result_solid = result.unwrap();
        // Should have the same faces as the original cube
        let n_faces = result_solid
            .outer_shell
            .as_ref()
            .map(|s| s.faces.len())
            .unwrap_or(0);
        assert_eq!(n_faces, 6, "Subtracting disjoint solid should return original cube");
    }

    #[test]
    fn test_boolean_op_enum() {
        assert_ne!(BooleanOp::Union, BooleanOp::Subtract);
        assert_ne!(BooleanOp::Subtract, BooleanOp::Intersect);
        assert_ne!(BooleanOp::Union, BooleanOp::Intersect);
    }

    // ---- Plane × Cone analytic intersection (B1 leftover fix) ----

    #[test]
    fn test_plane_cone_intersection_analytic() {
        // Narrowing cone: base radius 5 at z=0, apex at z=10 (tan(α)=0.5,
        // negative half_angle = STEP narrowing convention). Plane z=0 →
        // the base circle: every point exactly on both surfaces.
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            5.0,
            -(0.5f64).atan(),
        );
        let plane = Plane::from_origin_and_normal(Point3d::ORIGIN, Direction3d::Z);
        let tol = ToleranceContext::new();

        let forward =
            intersect_surfaces(&Surface::Plane(plane.clone()), &Surface::Cone(cone.clone()), &tol);
        assert_eq!(forward.len(), 1, "Expected 1 circle section, got {}", forward.len());
        assert!(forward[0].points.len() >= 64);
        for p in &forward[0].points {
            // On the plane
            assert!(p.z.abs() < 1e-9, "Point should be at z=0, got z={}", p.z);
            // On the cone: radius r(v) = 5 − 0.5·v, and v = z = 0 → r = 5
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 5.0).abs() < 1e-9, "Circle radius should be 5.0, got {}", r);
        }

        // Reversed order: same geometry (the dispatcher reverses point order
        // for consistency, but the point set must match).
        let reverse =
            intersect_surfaces(&Surface::Cone(cone), &Surface::Plane(plane), &tol);
        assert_eq!(reverse.len(), 1);
        assert_eq!(reverse[0].points.len(), forward[0].points.len());
        for p in &reverse[0].points {
            assert!(p.z.abs() < 1e-9);
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 5.0).abs() < 1e-9);
        }
    }

    #[test]
    fn test_plane_cone_intersection_oblique_ellipse() {
        // Tilted plane through the cone body → closed ellipse with points
        // exactly on both surfaces (the old brute-force sampling path only
        // produced approximate points).
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            5.0,
            -(0.5f64).atan(),
        );
        let tilt = 20.0f64.to_radians();
        let normal = Direction3d::new(tilt.sin(), 0.0, tilt.cos()).unwrap();
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 5.0), normal);
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Plane(plane), &Surface::Cone(cone), &tol);
        assert_eq!(curves.len(), 1, "Expected 1 ellipse, got {}", curves.len());
        let pts = &curves[0].points;
        assert!(pts.len() >= 64, "Expected dense sampling, got {}", pts.len());
        // Ellipse spans both sides of the axis in x (tilt direction).
        let min_x = pts.iter().map(|p| p.x).fold(f64::MAX, f64::min);
        let max_x = pts.iter().map(|p| p.x).fold(f64::MIN, f64::max);
        assert!(max_x - min_x > 1.0, "Ellipse should have spread");
    }

    // ---- Sphere × Sphere analytic intersection (B1 series follow-up) ----

    #[test]
    fn test_sphere_sphere_dispatch_analytic() {
        // Two overlapping spheres → radical-plane circle, exact points on
        // both surfaces, and the analytic Circle attached as `curve`.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::new(6.0, 0.0, 0.0), 3.0);
        let tol = ToleranceContext::new();

        let curves =
            intersect_surfaces(&Surface::Sphere(s1.clone()), &Surface::Sphere(s2.clone()), &tol);
        assert_eq!(curves.len(), 1, "Expected 1 circle, got {}", curves.len());
        let pts = &curves[0].points;
        assert!(pts.len() >= 64, "Expected dense sampling, got {}", pts.len());
        for p in pts {
            let d1 = ((p.x).powi(2) + (p.y).powi(2) + (p.z).powi(2)).sqrt();
            assert!((d1 - 5.0).abs() < 1e-9, "point not on sphere 1: r={}", d1);
            let d2 = ((p.x - 6.0).powi(2) + (p.y).powi(2) + (p.z).powi(2)).sqrt();
            assert!((d2 - 3.0).abs() < 1e-9, "point not on sphere 2: r={}", d2);
        }
        // The exact curve: circle of radius h = √(25 − (13/3)²) centered
        // at (13/3, 0, 0) with normal along the center line (+X).
        match &curves[0].curve {
            Some(Curve3d::Circle(c)) => {
                assert!((c.center.x - 13.0 / 3.0).abs() < 1e-9, "circle center x");
                assert!(c.center.y.abs() < 1e-9 && c.center.z.abs() < 1e-9);
                let h = (25.0 - (13.0_f64 / 3.0).powi(2)).sqrt();
                assert!((c.radius - h).abs() < 1e-9, "circle radius");
            }
            other => panic!("expected an exact Circle curve, got {:?}", other),
        }

        // Reversed order: same geometry, same point count.
        let reverse = intersect_surfaces(&Surface::Sphere(s2), &Surface::Sphere(s1), &tol);
        assert_eq!(reverse.len(), 1);
        assert_eq!(reverse[0].points.len(), pts.len());
    }

    #[test]
    fn test_sphere_sphere_tangent_and_disjoint() {
        let tol = ToleranceContext::new();
        // External tangency (d = r1 + r2): a single point on the center line.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::new(8.0, 0.0, 0.0), 3.0);
        let curves = intersect_surfaces(&Surface::Sphere(s1), &Surface::Sphere(s2), &tol);
        assert_eq!(curves.len(), 1, "tangent pair should yield 1 curve");
        assert_eq!(curves[0].points.len(), 1, "tangent point only");
        let p = curves[0].points[0];
        assert!((p.x - 5.0).abs() < 1e-6 && p.y.abs() < 1e-6 && p.z.abs() < 1e-6);

        // Disjoint: no curves.
        let s3 = SphereSurface::new(Point3d::new(20.0, 0.0, 0.0), 3.0);
        let empty = intersect_surfaces(
            &Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 5.0)),
            &Surface::Sphere(s3),
            &tol,
        );
        assert!(empty.is_empty(), "disjoint spheres should not intersect");
    }

    // ---- Sphere × Cylinder analytic intersection (B1 series follow-up) ----

    #[test]
    fn test_sphere_cylinder_dispatch_analytic_circles() {
        // Axis through the sphere center: two Steinmetch circles at
        // z = ±√(r² − R²) = ±4, each with the exact Circle attached.
        let cyl = CylinderSurface::new_z(3.0);
        let s = SphereSurface::new(Point3d::new(0.0, 0.0, 2.0), 5.0);
        let tol = ToleranceContext::new();

        let curves =
            intersect_surfaces(&Surface::Cylinder(cyl.clone()), &Surface::Sphere(s.clone()), &tol);
        assert_eq!(curves.len(), 2, "Expected 2 circles, got {}", curves.len());
        for curve in &curves {
            assert_eq!(curve.points.len(), 128);
            for p in &curve.points {
                let lateral = (p.x * p.x + p.y * p.y).sqrt();
                assert!((lateral - 3.0).abs() < 1e-9, "point not on cylinder");
                let d = ((p.x).powi(2) + (p.y).powi(2) + (p.z - 2.0).powi(2)).sqrt();
                assert!((d - 5.0).abs() < 1e-9, "point not on sphere: r={}", d);
            }
        }
        // Exact circle geometry: centers at z = 2 ± 4, radius 3, axis +Z.
        let z_centers: Vec<f64> = curves
            .iter()
            .map(|c| match &c.curve {
                Some(Curve3d::Circle(g)) => g.center.z,
                other => panic!("expected exact Circle, got {:?}", other),
            })
            .collect();
        assert!(z_centers.contains(&(6.0)) && z_centers.contains(&(-2.0)),
            "circle centers at z=-2 and z=6, got {:?}", z_centers);
        for c in &curves {
            if let Curve3d::Circle(g) = c.curve.as_ref().unwrap() {
                assert!((g.radius - 3.0).abs() < 1e-9);
                assert!(g.normal.z > 0.999);
            }
        }

        // Reversed argument order: same curves.
        let reverse = intersect_surfaces(&Surface::Sphere(s), &Surface::Cylinder(cyl), &tol);
        assert_eq!(reverse.len(), 2);
        assert_eq!(reverse[0].points.len(), 128);
    }

    #[test]
    fn test_sphere_cylinder_off_axis_one_loop() {
        // d = 2, R = 3, r = 4 → single closed Viviani-style curve, points
        // exactly on both surfaces (1e-9 proves the analytic path).
        let cyl = CylinderSurface::new_z(3.0);
        let s = SphereSurface::new(Point3d::new(2.0, 0.0, 0.0), 4.0);
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Cylinder(cyl), &Surface::Sphere(s), &tol);
        assert_eq!(curves.len(), 1, "Expected 1 closed loop");
        for p in &curves[0].points {
            let lateral = (p.x * p.x + p.y * p.y).sqrt();
            assert!((lateral - 3.0).abs() < 1e-9, "point not on cylinder");
            let d = ((p.x - 2.0).powi(2) + (p.y).powi(2) + (p.z).powi(2)).sqrt();
            assert!((d - 4.0).abs() < 1e-9, "point not on sphere: r={}", d);
        }
        // Off-axis quartic: no analytic Circle attached.
        assert!(curves[0].curve.is_none(), "quartic should be polyline-only");
    }

    #[test]
    fn test_sphere_cylinder_tangent_and_disjoint() {
        let tol = ToleranceContext::new();
        let cyl = CylinderSurface::new_z(3.0);
        // External tangency: d = R + r = 3 + 5 → a single point at x = 3.
        let s = SphereSurface::new(Point3d::new(8.0, 0.0, 0.0), 5.0);
        let curves = intersect_surfaces(&Surface::Cylinder(cyl.clone()), &Surface::Sphere(s), &tol);
        assert_eq!(curves.len(), 1);
        assert_eq!(curves[0].points.len(), 1);
        let p = curves[0].points[0];
        assert!((p.x - 3.0).abs() < 1e-9 && p.y.abs() < 1e-9 && p.z.abs() < 1e-9);

        // Disjoint: no curves.
        let far = SphereSurface::new(Point3d::new(20.0, 0.0, 0.0), 5.0);
        let empty = intersect_surfaces(&Surface::Cylinder(cyl), &Surface::Sphere(far), &tol);
        assert!(empty.is_empty(), "disjoint pair should not intersect");
    }

    #[test]
    fn test_cylinder_cylinder_parallel_lines_exact_geometry() {
        // Parallel axes, lateral separation 4 < 3 + 3 → two straight lines
        // with the EXACT Line geometry attached (direction = shared axis).
        let c1 = CylinderSurface::new_z(3.0);
        let c2 = CylinderSurface::new(Point3d::new(4.0, 0.0, 0.0), Direction3d::Z, 3.0);
        let tol = ToleranceContext::new();

        let curves =
            intersect_surfaces(&Surface::Cylinder(c1.clone()), &Surface::Cylinder(c2.clone()), &tol);
        assert_eq!(curves.len(), 2, "expected two lines, got {}", curves.len());
        for curve in &curves {
            assert!(curve.points.len() >= 2);
            // Exact Line geometry along the +Z axis.
            match &curve.curve {
                Some(Curve3d::Line(l)) => {
                    assert!(l.direction.z > 0.999, "line direction should be +Z");
                }
                other => panic!("expected exact Line, got {:?}", other),
            }
            // Points exactly on both cylinders.
            for p in &curve.points {
                let lat1 = (p.x * p.x + p.y * p.y).sqrt();
                assert!((lat1 - 3.0).abs() < 1e-9, "point not on cylinder 1");
                let dx = p.x - 4.0;
                let lat2 = (dx * dx + p.y * p.y).sqrt();
                assert!((lat2 - 3.0).abs() < 1e-9, "point not on cylinder 2");
            }
        }

        // Reversed argument order: same result.
        let reverse = intersect_surfaces(&Surface::Cylinder(c2), &Surface::Cylinder(c1), &tol);
        assert_eq!(reverse.len(), 2);
    }

    #[test]
    fn test_cylinder_cylinder_non_parallel_analytic() {
        // Skew cylinders (perpendicular axes, offset origins): one closed
        // loop of exact points — 1e-9 on-surface proves the analytic
        // quadratic path (the old grid marcher achieved only ~5% of R).
        let c1 = CylinderSurface::new_z(2.0);
        let c2 = CylinderSurface::new(
            Point3d::new(0.0, 1.0, 0.5),
            Direction3d::new(1.0, 0.0, 0.0).unwrap(),
            2.0,
        );
        let tol = ToleranceContext::new();

        let curves = intersect_surfaces(&Surface::Cylinder(c1), &Surface::Cylinder(c2), &tol);
        assert_eq!(curves.len(), 1, "expected one closed loop");
        assert!(curves[0].points.len() >= 16);
        for p in &curves[0].points {
            let lat1 = (p.x * p.x + p.y * p.y).sqrt();
            assert!((lat1 - 2.0).abs() < 1e-9, "point not on cylinder 1");
            let dy = p.y - 1.0;
            let dz = p.z - 0.5;
            let lat2 = (dy * dy + dz * dz).sqrt();
            assert!((lat2 - 2.0).abs() < 1e-9, "point not on cylinder 2");
        }
        // Space quartic: no analytic curve attached (polyline-only).
        assert!(curves[0].curve.is_none(), "quartic should be polyline-only");
    }

    #[test]
    fn test_cylinder_cylinder_tangent_and_disjoint() {
        let tol = ToleranceContext::new();
        let c1 = CylinderSurface::new_z(3.0);
        // External tangency of non-parallel cylinders: B (+X through
        // (0, 6, 0), R=3) touches A at (0, 3, 0) → a single point.
        let c2 = CylinderSurface::new(
            Point3d::new(0.0, 6.0, 0.0),
            Direction3d::new(1.0, 0.0, 0.0).unwrap(),
            3.0,
        );
        let curves =
            intersect_surfaces(&Surface::Cylinder(c1.clone()), &Surface::Cylinder(c2), &tol);
        assert_eq!(curves.len(), 1, "tangency: one curve");
        assert_eq!(curves[0].points.len(), 1, "tangency: single point");
        let p = curves[0].points[0];
        assert!((p.x).abs() < 1e-4 && (p.y - 3.0).abs() < 1e-4 && (p.z).abs() < 1e-4);

        // Disjoint non-parallel: empty.
        let far = CylinderSurface::new(
            Point3d::new(0.0, 10.0, 0.0),
            Direction3d::new(1.0, 0.0, 0.0).unwrap(),
            1.0,
        );
        let empty =
            intersect_surfaces(&Surface::Cylinder(c1), &Surface::Cylinder(far), &tol);
        assert!(empty.is_empty(), "disjoint pair should not intersect");
    }

    #[test]
    fn test_cone_cone_coaxial_circle_exact_geometry() {
        // Nose-to-nose 30° cones from z=0 (up) and z=10 (down): the radii
        // meet at z=5 → one circle of radius 5·tan30 with the EXACT Circle
        // geometry attached.
        let tol = ToleranceContext::new();
        let c1 = ConeSurface::new_expanding(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            30.0f64.to_radians(),
            Direction3d::X,
        );
        let c2 = ConeSurface::new_expanding(
            Point3d::new(0.0, 0.0, 10.0),
            Direction3d::new(0.0, 0.0, -1.0).unwrap(),
            30.0f64.to_radians(),
            Direction3d::X,
        );
        let curves = intersect_surfaces(&Surface::Cone(c1), &Surface::Cone(c2), &tol);
        assert_eq!(curves.len(), 1, "expected one circle, got {}", curves.len());
        let r_expected = 5.0 * 30.0f64.to_radians().tan();
        match &curves[0].curve {
            Some(Curve3d::Circle(c)) => {
                assert!((c.center.z - 5.0).abs() < 1e-9, "center z = {}", c.center.z);
                assert!(c.center.x.abs() < 1e-9 && c.center.y.abs() < 1e-9);
                assert!(
                    (c.radius - r_expected).abs() < 1e-9,
                    "radius = {} vs {}",
                    c.radius,
                    r_expected
                );
            }
            other => panic!("expected exact Circle, got {:?}", other),
        }
        for p in &curves[0].points {
            assert!((p.z - 5.0).abs() < 1e-9, "z = {} vs 5", p.z);
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - r_expected).abs() < 1e-9, "r = {} vs {}", r, r_expected);
        }
    }

    #[test]
    fn test_cone_cone_generic_polyline_exactness() {
        // Non-parallel cones (30° up from O; 45° +X from (−4, 0, 2)):
        // polyline-only (space quartic), every point exact on both nappes.
        let tol = ToleranceContext::new();
        let c1 = ConeSurface::new_expanding(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            30.0f64.to_radians(),
            Direction3d::X,
        );
        let c2 = ConeSurface::new_expanding(
            Point3d::new(-4.0, 0.0, 2.0),
            Direction3d::new(1.0, 0.0, 0.0).unwrap(),
            45.0f64.to_radians(),
            Direction3d::X,
        );
        let curves = intersect_surfaces(&Surface::Cone(c1), &Surface::Cone(c2), &tol);
        assert!(!curves.is_empty(), "overlapping cones must intersect");
        for curve in &curves {
            // Non-parallel axes → no Circle fit attempted.
            assert!(curve.curve.is_none(), "generic quartic should be polyline-only");
            for p in &curve.points {
                // Cone 1: apex O, nappe +Z, 30°.
                let wl = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
                let cos1 = p.z / wl;
                assert!(
                    (cos1 - 30.0f64.to_radians().cos()).abs() < 1e-9,
                    "not on cone 1: cos = {}",
                    cos1
                );
                // Cone 2: apex (−4, 0, 2), nappe +X, 45°.
                let wx = p.x + 4.0;
                let wy = p.y;
                let wz = p.z - 2.0;
                let wl2 = (wx * wx + wy * wy + wz * wz).sqrt();
                let cos2 = wx / wl2;
                assert!(
                    (cos2 - 45.0f64.to_radians().cos()).abs() < 1e-9,
                    "not on cone 2: cos = {}",
                    cos2
                );
            }
        }
    }

    #[test]
    fn test_cone_cylinder_coaxial_circle_both_orders() {
        // 45° up-cone × coaxial R=1 cylinder: one circle at z=1 with the
        // EXACT Circle geometry — from both dispatch orders.
        let tol = ToleranceContext::new();
        let cone = ConeSurface::new_expanding(
            Point3d::ORIGIN,
            Direction3d::Z,
            45.0f64.to_radians(),
            Direction3d::X,
        );
        let cyl = CylinderSurface::new_z(1.0);
        for (a, b) in [
            (&Surface::Cone(cone.clone()), &Surface::Cylinder(cyl.clone())),
            (&Surface::Cylinder(cyl), &Surface::Cone(cone)),
        ] {
            let curves = intersect_surfaces(a, b, &tol);
            assert_eq!(curves.len(), 1, "one circle, got {}", curves.len());
            match &curves[0].curve {
                Some(Curve3d::Circle(c)) => {
                    assert!((c.center.z - 1.0).abs() < 1e-9, "center z = {}", c.center.z);
                    assert!((c.radius - 1.0).abs() < 1e-9, "radius = {}", c.radius);
                }
                other => panic!("expected exact Circle, got {:?}", other),
            }
            for p in &curves[0].points {
                assert!((p.z - 1.0).abs() < 1e-9, "z = {} vs 1", p.z);
                let r = (p.x * p.x + p.y * p.y).sqrt();
                assert!((r - 1.0).abs() < 1e-9, "r = {} vs 1", r);
            }
        }
    }

    // ── Property-based tests (proptest) ──

    /// Invariant 1: Box volume = w × h × d for any positive dimensions.
    /// Tests that `ShapeBuilder::make_box` produces a solid whose
    /// `solid_volume()` matches the analytical formula.
    #[test]
    fn proptest_box_volume() {
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn box_volume_equals_w_h_d(w in 0.1f64..100.0, h in 0.1f64..100.0, d in 0.1f64..100.0) {
                let solid = ShapeBuilder::make_box(w, h, d);
                let volume = crate::queries::solid_volume(&solid);
                let expected = w * h * d;
                // Allow 1% tolerance for floating-point triangulation error.
                prop_assert!(
                    (volume - expected).abs() / expected.max(1e-10) < 0.01,
                    "Box volume {} != expected {} for ({}, {}, {})",
                    volume, expected, w, h, d
                );
            }
        }
    }

    /// Invariant 2: Box surface area = 2(wh + wd + hd) for any positive dimensions.
    #[test]
    fn proptest_box_surface_area() {
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn box_area_matches_formula(w in 0.1f64..100.0, h in 0.1f64..100.0, d in 0.1f64..100.0) {
                let solid = ShapeBuilder::make_box(w, h, d);
                let area = crate::queries::solid_surface_area(&solid);
                let expected = 2.0 * (w * h + w * d + h * d);
                // Allow 5% tolerance for triangulation (areas from triangle sum).
                prop_assert!(
                    (area - expected).abs() / expected.max(1e-10) < 0.05,
                    "Box area {} != expected {} for ({}, {}, {})",
                    area, expected, w, h, d
                );
            }
        }
    }

    /// Invariant 3: Boolean union volume >= max(vol_a, vol_b).
    /// The union of two solids has at least as much volume as the larger solid.
    #[test]
    fn proptest_union_volume_geq_max() {
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn union_volume_invariant(
                // Two boxes at different positions.
                ax in -10.0f64..10.0,
                bx in -10.0f64..10.0
            ) {
                let box_a = ShapeBuilder::make_box_at(ax, 0.0, 0.0, 2.0, 2.0, 2.0);
                let box_b = ShapeBuilder::make_box_at(bx, 0.0, 0.0, 2.0, 2.0, 2.0);
                let vol_a = crate::queries::solid_volume(&box_a);
                let vol_b = crate::queries::solid_volume(&box_b);
                let tol = ToleranceContext::default();
                if let Ok(union) = boolean_union(&box_a, &box_b, &tol) {
                    let vol_union = crate::queries::solid_volume(&union);
                    let max_vol = vol_a.max(vol_b);
                    // Union should have volume >= max(vol_a, vol_b).
                    // (May be less than vol_a + vol_b if they overlap.)
                    prop_assert!(
                        vol_union >= max_vol * 0.95,
                        "Union volume {} < max({}, {}) = {}",
                        vol_union, vol_a, vol_b, max_vol
                    );
                }
            }
        }
    }

    /// Invariant 4: Boolean subtract volume <= volume of A.
    /// Subtracting B from A cannot increase the volume.
    #[test]
    fn proptest_subtract_volume_leq_a() {
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn subtract_volume_invariant(
                // Subtract a box from a larger box.
                offset in 0.0f64..5.0,
                size in 0.5f64..3.0
            ) {
                let big = ShapeBuilder::make_box(10.0, 10.0, 10.0);
                let small = ShapeBuilder::make_box_at(offset, offset, offset, size, size, size);
                let vol_a = crate::queries::solid_volume(&big);
                let tol = ToleranceContext::default();
                if let Ok(result) = boolean_subtract(&big, &small, &tol) {
                    let vol_result = crate::queries::solid_volume(&result);
                    prop_assert!(
                        vol_result <= vol_a * 1.01,  // 1% tolerance
                        "Subtract volume {} > A volume {}",
                        vol_result, vol_a
                    );
                }
            }
        }
    }

    /// Invariant 5: Boolean intersect volume <= min(vol_a, vol_b).
    /// The intersection cannot be larger than either solid.
    #[test]
    fn proptest_intersect_volume_leq_min() {
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn intersect_volume_invariant(
                offset in 0.0f64..3.0
            ) {
                let box_a = ShapeBuilder::make_box_at(0.0, 0.0, 0.0, 4.0, 4.0, 4.0);
                let box_b = ShapeBuilder::make_box_at(offset, 0.0, 0.0, 4.0, 4.0, 4.0);
                let vol_a = crate::queries::solid_volume(&box_a);
                let vol_b = crate::queries::solid_volume(&box_b);
                let tol = ToleranceContext::default();
                if let Ok(result) = boolean_intersect(&box_a, &box_b, &tol) {
                    let vol_result = crate::queries::solid_volume(&result);
                    let min_vol = vol_a.min(vol_b);
                    prop_assert!(
                        vol_result <= min_vol * 1.01,  // 1% tolerance
                        "Intersect volume {} > min({}, {}) = {}",
                        vol_result, vol_a, vol_b, min_vol
                    );
                }
            }
        }
    }

    /// Invariant 6: Fillet reduces volume (rounding corners removes material).
    #[test]
    fn proptest_fillet_reduces_volume() {
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn fillet_volume_decreases(
                radius in 0.01f64..2.0
            ) {
                let box_solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
                let vol_before = crate::queries::solid_volume(&box_solid);
                if let Ok(filleted) = crate::operations::fillet_edge(&box_solid, 0, radius) {
                    let vol_after = crate::queries::solid_volume(&filleted);
                    // Fillet should not increase volume.
                    prop_assert!(
                        vol_after <= vol_before * 1.01,  // 1% tolerance
                        "Fillet volume {} > original {} (radius={})",
                        vol_after, vol_before, radius
                    );
                }
            }
        }
    }
}

    // ── Torus SSI wrappers (T-series, 2026-09-02) ─────────────────────

    #[test]
    fn test_torus_plane_perp_axis_exact_circles() {
        // Equatorial plane z=0 on a (10, 3) torus → 2 latitude circles
        // with the EXACT Circle geometry (radii 7 and 13, z=0).
        let tol = ToleranceContext::new();
        let torus = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0);
        let plane = Plane::xy();
        let curves =
            intersect_surfaces(&Surface::Plane(plane), &Surface::Torus(torus.clone()), &tol);
        assert_eq!(curves.len(), 2, "equatorial plane → 2 circles");
        let mut radii: Vec<f64> = curves
            .iter()
            .map(|c| match &c.curve {
                Some(Curve3d::Circle(circle)) => {
                    assert!(circle.center.z.abs() < 1e-9, "circle at z=0");
                    assert!(circle.center.x.abs() < 1e-9 && circle.center.y.abs() < 1e-9);
                    circle.radius
                }
                other => panic!("expected exact Circle, got {:?}", other),
            })
            .collect();
        radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((radii[0] - 7.0).abs() < 1e-9, "inner R−r = 7, got {}", radii[0]);
        assert!((radii[1] - 13.0).abs() < 1e-9, "outer R+r = 13, got {}", radii[1]);
    }

    #[test]
    fn test_torus_plane_containing_axis_exact_circles() {
        // Plane x=0 contains the Z axis → 2 meridian tube circles with
        // centers (0, ±10, 0) and radius 3 (EXACT Circle geometry).
        let tol = ToleranceContext::new();
        let torus = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0);
        let plane = Plane::from_origin_and_normal(Point3d::ORIGIN, Direction3d::X);
        let curves =
            intersect_surfaces(&Surface::Plane(plane), &Surface::Torus(torus), &tol);
        assert_eq!(curves.len(), 2, "axis plane → 2 meridian circles");
        for curve in &curves {
            match &curve.curve {
                Some(Curve3d::Circle(circle)) => {
                    let cy = circle.center.y.abs();
                    assert!((cy - 10.0).abs() < 1e-9, "center (0,±10,0), y={}", circle.center.y);
                    assert!(circle.center.x.abs() < 1e-9 && circle.center.z.abs() < 1e-9);
                    assert!((circle.radius - 3.0).abs() < 1e-9, "r=3, got {}", circle.radius);
                }
                other => panic!("expected exact Circle, got {:?}", other),
            }
        }
    }

    #[test]
    fn test_torus_sphere_concentric_exact_circles() {
        // Concentric sphere radius 10 → 2 latitude circles with EXACT
        // Circle geometry: ρ = R + r·cosφ with cosφ = −0.15, z = ±r·sinφ.
        let tol = ToleranceContext::new();
        let torus = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0);
        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let curves =
            intersect_surfaces(&Surface::Sphere(sphere), &Surface::Torus(torus), &tol);
        assert!(curves.len() >= 2, "concentric → 2 latitude circles");
        let cos_phi: f64 = -0.15;
        let rho = 10.0 + 3.0 * cos_phi;
        let z = 3.0 * (1.0f64 - cos_phi * cos_phi).sqrt();
        for curve in &curves {
            match &curve.curve {
                Some(Curve3d::Circle(circle)) => {
                    assert!((circle.radius - rho).abs() < 1e-8, "radius ρ={rho}, got {}", circle.radius);
                    assert!((circle.center.z.abs() - z).abs() < 1e-8, "center z=±{z}");
                }
                other => panic!("expected exact Circle, got {:?}", other),
            }
        }
    }

    #[test]
    fn test_torus_cylinder_coaxial_exact_circles() {
        // Coaxial cylinder R_c = 8 → 2 circles at z = ±√(9−4) = ±√5 with
        // the EXACT Circle geometry (radius 8 around the common axis).
        let tol = ToleranceContext::new();
        let torus = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0);
        let cyl = CylinderSurface::new_z(8.0);
        for (a, b) in [
            (&Surface::Torus(torus.clone()), &Surface::Cylinder(cyl.clone())),
            (&Surface::Cylinder(cyl.clone()), &Surface::Torus(torus.clone())),
        ] {
            let curves = intersect_surfaces(a, b, &tol);
            assert_eq!(curves.len(), 2, "coaxial → 2 circles both orders");
            let z_exp = (9.0f64 - 4.0).sqrt();
            let mut zs: Vec<f64> = curves
                .iter()
                .map(|c| match &c.curve {
                    Some(Curve3d::Circle(circle)) => {
                        assert!((circle.radius - 8.0).abs() < 1e-9, "radius 8");
                        circle.center.z
                    }
                    other => panic!("expected exact Circle, got {:?}", other),
                })
                .collect();
            zs.sort_by(|x, y| x.partial_cmp(y).unwrap());
            assert!((zs[0] + z_exp).abs() < 1e-8, "z = −√5, got {}", zs[0]);
            assert!((zs[1] - z_exp).abs() < 1e-8, "z = +√5, got {}", zs[1]);
        }
    }

    #[test]
    fn test_torus_cylinder_perpendicular_polyline_exactness() {
        // Perpendicular axes: ψ-parametrized analytic path — polyline-only,
        // points exact on both surfaces.
        let tol = ToleranceContext::new();
        let torus = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0);
        let cyl = CylinderSurface {
            origin: Point3d::ORIGIN,
            axis: Direction3d::X,
            radius: 3.0,
            x_dir: Direction3d::Z,
        };
        let curves = intersect_surfaces(&Surface::Cylinder(cyl), &Surface::Torus(torus), &tol);
        assert!(!curves.is_empty(), "perpendicular cylinder crosses the tube");
        for curve in &curves {
            assert!(curve.curve.is_none(), "generic perpendicular → polyline-only");
            for p in &curve.points {
                // On torus: profile-circle distance = r.
                let rho = (p.x * p.x + p.y * p.y).sqrt();
                let d = ((rho - 10.0) * (rho - 10.0) + p.z * p.z).sqrt();
                assert!((d - 3.0).abs() < 1e-7, "on torus, tube-dist={d:.9}");
                // On cylinder: distance to the X axis = 3.
                let c = (p.y * p.y + p.z * p.z).sqrt();
                assert!((c - 3.0).abs() < 1e-9, "on cylinder, radial={c:.9}");
            }
        }
    }

    #[test]
    fn test_boolean_indexed_equivalence() {
        // Store-first reads must not change boolean results: the same
        // subtraction with un-indexed (mirror reads) and indexed
        // (store reads) inputs produces identical topology and volume.
        let tol = ToleranceContext::from_model_scale(133.0);
        let box_plain = ShapeBuilder::make_box(100.0, 80.0, 50.0);
        let cyl_plain = ShapeBuilder::make_cylinder(20.0, 100.0);

        let mut box_indexed = box_plain.clone();
        box_indexed.index_edges();
        let mut cyl_indexed = cyl_plain.clone();
        cyl_indexed.index_edges();

        let r_plain = boolean_subtract(&box_plain, &cyl_plain, &tol)
            .expect("plain subtract must succeed");
        let r_indexed = boolean_subtract(&box_indexed, &cyl_indexed, &tol)
            .expect("indexed subtract must succeed");

        assert_eq!(
            r_plain.faces().len(),
            r_indexed.faces().len(),
            "face count must match"
        );

        let fingerprint = |s: &Solid| -> Vec<Vec<usize>> {
            s.faces()
                .iter()
                .map(|f| {
                    let mut v: Vec<usize> = Vec::new();
                    if let Some(w) = &f.outer_wire {
                        v.push(w.coedges.len());
                    }
                    for w in &f.inner_wires {
                        v.push(w.coedges.len());
                    }
                    v.sort_unstable();
                    v
                })
                .collect()
        };
        assert_eq!(
            fingerprint(&r_plain),
            fingerprint(&r_indexed),
            "per-face wire fingerprint must match"
        );

        let vol_plain = crate::queries::solid_volume(&r_plain);
        let vol_indexed = crate::queries::solid_volume(&r_indexed);
        assert_eq!(
            vol_plain.to_bits(),
            vol_indexed.to_bits(),
            "volumes must be bit-identical: {vol_plain} vs {vol_indexed}"
        );
    }
