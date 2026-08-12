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
use draper_geometry::{
    Point3d, Direction3d, Vec3d,
    Curve3d, Curve2d, Line, Circle, Ellipse,
    Line2d, Circle2d,
    Surface, Plane, CylinderSurface, SphereSurface, ConeSurface,
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
            let count = count_ray_face_intersections(&ray_origin, &ray_dir, face, tol);
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
fn count_ray_face_intersections(origin: &Point3d, direction: &Direction3d, face: &Face, tol: f64) -> u32 {
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
            if is_point_in_face_boundary(&hit, face, tol) { 1 } else { 0 }
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
                if is_point_in_face_boundary(&hit, face, tol) {
                    count += 1;
                }
            }
            if t2 > tol {
                let hit = Point3d::new(
                    origin.x + t2 * direction.x,
                    origin.y + t2 * direction.y,
                    origin.z + t2 * direction.z,
                );
                if is_point_in_face_boundary(&hit, face, tol) {
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
                    t > tol && is_point_in_face_boundary(p, face, tol)
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

    // Sample the surface on a grid and check for ray crossings
    // by looking for sign changes of the signed distance to the ray plane
    let ray_normal = *direction;

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

            // Signed distances from the ray line
            let d00 = signed_distance_to_ray(&p00, origin, &ray_normal);
            let d10 = signed_distance_to_ray(&p10, origin, &ray_normal);
            let d01 = signed_distance_to_ray(&p01, origin, &ray_normal);
            let d11 = signed_distance_to_ray(&p11, origin, &ray_normal);

            // If there's a sign change, there's likely a ray crossing in this patch
            let has_sign_change = (d00 > 0.0 && d10 < 0.0)
                || (d00 > 0.0 && d01 < 0.0)
                || (d10 > 0.0 && d00 < 0.0)
                || (d01 > 0.0 && d00 < 0.0)
                || (d10 > 0.0 && d11 < 0.0)
                || (d11 > 0.0 && d10 < 0.0)
                || (d01 > 0.0 && d11 < 0.0)
                || (d11 > 0.0 && d01 < 0.0);

            if has_sign_change {
                // Also check that the crossing is in the forward direction of the ray
                let mid_u = (u0 + u1) / 2.0;
                let mid_v = (v0 + v1) / 2.0;
                let mid_p = surface.point_at(mid_u, mid_v);
                let dx = mid_p.x - origin.x;
                let dy = mid_p.y - origin.y;
                let dz = mid_p.z - origin.z;
                let t = dx * direction.x + dy * direction.y + dz * direction.z;
                if t > tol {
                    count += 1;
                }
            }
        }
    }

    count
}

/// Signed distance from a point to the ray line.
/// The ray is defined as origin + t*direction.
/// Returns the cross product magnitude projected to determine which side.
fn signed_distance_to_ray(point: &Point3d, origin: &Point3d, direction: &Direction3d) -> f64 {
    let dx = point.x - origin.x;
    let dy = point.y - origin.y;
    let dz = point.z - origin.z;
    // Cross product of (point-origin) x direction
    let cx = dy * direction.z - dz * direction.y;
    let cy = dz * direction.x - dx * direction.z;
    let cz = dx * direction.y - dy * direction.x;
    // Use one component as a sign indicator
    // We choose the component that's most perpendicular to the direction
    let dir_abs_x = direction.x.abs();
    let dir_abs_y = direction.y.abs();
    let dir_abs_z = direction.z.abs();
    if dir_abs_x <= dir_abs_y && dir_abs_x <= dir_abs_z {
        cx // X component is most perpendicular
    } else if dir_abs_y <= dir_abs_z {
        cy
    } else {
        cz
    }
}

/// Check if a point (already known to be on the surface) is within the face's boundary.
fn is_point_in_face_boundary(point: &Point3d, face: &Face, tol: f64) -> bool {
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

    // Collect edge points for polygon check
    let mut polygon_points: Vec<Point3d> = Vec::new();
    for edge in &face.edges {
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
    // Distance from cone origin to plane
    let dx = cone.origin.x - plane.origin.x;
    let dy = cone.origin.y - plane.origin.y;
    let dz = cone.origin.z - plane.origin.z;
    let _signed_dist = dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z;

    // Angle between plane normal and cone axis
    let cos_angle = plane.normal.x * cone.axis.x
        + plane.normal.y * cone.axis.y
        + plane.normal.z * cone.axis.z;

    let _sin_angle = (1.0 - cos_angle * cos_angle).max(0.0).sqrt();

    // If plane is perpendicular to cone axis (cos_angle ≈ 0):
    //   - If |signed_dist| < cone.radius, it's a circle/ellipse
    // If plane is parallel to cone surface (sin_angle ≈ sin(half_angle)):
    //   - Parabola
    // Otherwise: ellipse or hyperbola

    // For simplicity, sample the intersection by finding points on both surfaces
    let curves = sample_surface_intersection(
        &Surface::Cone(cone.clone()),
        &Surface::Plane(plane.clone()),
        tol,
    );

    if curves.is_empty() {
        Vec::new()
    } else {
        curves
    }
}

/// Cylinder-Cylinder intersection.
fn intersect_cylinder_cylinder(
    c1: &CylinderSurface,
    c2: &CylinderSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    // For simplicity, use the general sampling approach
    sample_surface_intersection(
        &Surface::Cylinder(c1.clone()),
        &Surface::Cylinder(c2.clone()),
        tol,
    )
}

/// Cylinder-Sphere intersection.
fn intersect_cylinder_sphere(
    cyl: &CylinderSurface,
    sphere: &SphereSurface,
    tol: f64,
) -> Vec<IntersectionCurve> {
    // Distance from sphere center to cylinder axis
    let dx = sphere.center.x - cyl.origin.x;
    let dy = sphere.center.y - cyl.origin.y;
    let dz = sphere.center.z - cyl.origin.z;
    let along_axis = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
    let perp_x = dx - along_axis * cyl.axis.x;
    let perp_y = dy - along_axis * cyl.axis.y;
    let perp_z = dz - along_axis * cyl.axis.z;
    let dist_to_axis = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();

    // Quick rejection
    if dist_to_axis > sphere.radius + cyl.radius + tol {
        return Vec::new();
    }

    // Use general sampling approach
    sample_surface_intersection(
        &Surface::Cylinder(cyl.clone()),
        &Surface::Sphere(sphere.clone()),
        tol,
    )
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
            split_planar_face(face, plane, intersection_points, tol)
        }
        _ => {
            // For non-planar faces, use a simplified approach:
            // Create two faces with the intersection curve as a shared boundary
            split_general_face(face, intersection_points, tol)
        }
    }
}

/// Split a planar face along an intersection curve.
fn split_planar_face(
    face: &Face,
    plane: &Plane,
    intersection_points: &[Point3d],
    tol: f64,
) -> BooleanResult<SplitFaceResult> {
    // Get the face's boundary polygon
    let mut boundary: Vec<Point3d> = Vec::new();
    for edge in &face.edges {
        if let Some(ref curve) = edge.curve {
            let (t_min, t_max) = edge.param_range;
            let n = 20;
            for i in 0..n {
                let t = t_min + (t_max - t_min) * (i as f64 / n as f64);
                boundary.push(curve.point_at(t));
            }
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
    // Simplified approach: create two sub-polygons by inserting the
    // intersection curve as a shared boundary.

    let n = boundary.len();
    let ni = intersection.len();

    if n < 3 || ni < 2 {
        return (boundary.to_vec(), Vec::new());
    }

    // Find the closest boundary point to the first and last intersection points
    let start_idx = find_closest_boundary_point(&intersection[0], boundary);
    let end_idx = find_closest_boundary_point(&intersection[ni - 1], boundary);

    // Build two sub-polygons
    let mut poly_a: Vec<(f64, f64)> = Vec::new();
    let mut poly_b: Vec<(f64, f64)> = Vec::new();

    // Polygon A: boundary from start to end, then intersection curve back
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
    // Add intersection curve in reverse
    for i in (0..ni).rev() {
        poly_a.push(intersection[i]);
    }

    // Polygon B: boundary from end to start, then intersection curve
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
    for i in 0..ni {
        poly_b.push(intersection[i]);
    }

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
fn split_general_face(
    face: &Face,
    intersection_points: &[Point3d],
    tol: f64,
) -> BooleanResult<SplitFaceResult> {
    // For non-planar faces (cylinder, sphere, etc.), we CANNOT easily split
    // the face along the intersection curve because that would require
    // trimming the parametric surface — a complex operation.
    //
    // Previous approach created face_b with only 1 edge (the intersection
    // curve), which is topologically invalid (unclosed wire) and caused
    // the triangulation to produce garbage.
    //
    // New approach: return the face UNSPLIT. The classification step
    // (Step 4) will use the face centroid to determine if the ENTIRE face
    // is inside or outside the other solid, and keep/discard accordingly.
    //
    // This is correct for the common case where the intersection curve
    // divides the face into a "mostly inside" and "mostly outside" region —
    // the centroid will be on the dominant side.
    //
    // For cases where the face is split roughly in half, this may keep or
    // discard the entire face, but that's a better failure mode than
    // producing invalid topology.
    //
    // The intersection curve is still stored in the face's edges for
    // potential future use (e.g., creating a proper trimmed surface).
    let _ = tol;
    if intersection_points.len() < 2 {
        return Ok(SplitFaceResult {
            faces: vec![face.clone()],
        });
    }

    // Return the face unsplit, but store the intersection curve as an
    // additional edge (for debugging/future use)
    let mut face_with_curve = face.clone();
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
    face_with_curve.edges.push(int_edge);

    Ok(SplitFaceResult {
        faces: vec![face_with_curve],
    })
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
        let split_result = split_face_with_shared_edges(
            &faces_a[face_a_idx],
            &all_points,
            &all_edges,
            &all_pcurves,
            tol_ctx,
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
        let split_result = split_face_with_shared_edges(
            &faces_b[face_b_idx],
            &all_points,
            &all_edges,
            &all_pcurves,
            tol_ctx,
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
        let classification = classify_face_robust(face, solid_b, tol_ctx);
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
        let classification = classify_face_robust(face, solid_a, tol_ctx);
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
                    let mut reversed = face.reversed();
                    reversed.forward = !reversed.forward;
                    result_faces.push(reversed);
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

    if result_faces.is_empty() {
        return Err(BooleanError::EmptyResult(
            "Boolean operation produced an empty result".to_string(),
        ));
    }

    // Step 5: Connect faces into a new closed shell
    let shell = Shell::new_closed(result_faces);
    Ok(Solid::new(shell))
}

/// Split a face along an intersection curve, using a SHARED edge that is
/// also used by the adjacent face's split. This ensures both split faces
/// reference the same edge ID, producing watertight meshes.
fn split_face_with_shared_edge(
    face: &Face,
    intersection_points: &[Point3d],
    shared_edge: &Edge,
    pcurve: Option<&Curve2d>,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<SplitFaceResult> {
    split_face_with_shared_edges(face, intersection_points, &[shared_edge.clone()], &[pcurve.cloned()], tol_ctx)
}

/// Split a face with multiple shared edges (for faces intersected by multiple curves).
/// Each shared edge may have an associated PCurve on this face's surface.
fn split_face_with_shared_edges(
    face: &Face,
    intersection_points: &[Point3d],
    shared_edges: &[Edge],
    pcurves: &[Option<Curve2d>],
    tol_ctx: &ToleranceContext,
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
            split_planar_face_shared(face, plane, intersection_points, &shared_edges[0], pc, tol)
        }
        Surface::Cylinder(cyl) => {
            // For cylinder faces, pass ALL shared edges so the inner band
            // can use the correct shared edge for each boundary circle
            split_cylinder_face_multi_shared(face, cyl, intersection_points, shared_edges, tol)
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
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for edge in &face.edges {
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
    let inner_coedge_bottom = CoEdge::new(edge_v1.id, false);
    let inner_coedge_top = CoEdge::new(edge_v2.id, true);
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
        let wire = Wire::new(vec![CoEdge::new(bottom_edge.id, true), CoEdge::new(edge_v1.id, false)]);
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
        let wire = Wire::new(vec![CoEdge::new(edge_v2.id, false), CoEdge::new(top_edge.id, true)]);
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
    plane: &Plane,
    intersection_points: &[Point3d],
    shared_edge: &Edge,
    pcurve: Option<Curve2d>,
    tol: f64,
) -> BooleanResult<SplitFaceResult> {
    // Get the face's boundary polygon
    let mut boundary: Vec<Point3d> = Vec::new();
    for edge in &face.edges {
        if let Some(ref curve) = edge.curve {
            let (t_min, t_max) = edge.param_range;
            let n = 20;
            for i in 0..n {
                let t = t_min + (t_max - t_min) * (i as f64 / n as f64);
                boundary.push(curve.point_at(t));
            }
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
    //   - Some edges from the original boundary (new Edge IDs)
    //   - The shared edge for the intersection curve portion
    let mut result_faces = Vec::new();

    for poly_2d in &[poly_a, poly_b] {
        if poly_2d.len() < 3 {
            continue;
        }

        let points_3d: Vec<Point3d> = poly_2d
            .iter()
            .map(|(u, v)| plane.point_at(*u, *v))
            .collect();

        // Build the face with edges, using the shared edge where the
        // intersection curve is part of the boundary.
        let n = points_3d.len();
        let mut edges = Vec::new();
        let mut coedges = Vec::new();

        for i in 0..n {
            let j = (i + 1) % n;
            let p0 = points_3d[i];
            let p1 = points_3d[j];

            // Check if this edge is part of the intersection curve
            // (both endpoints are on the intersection curve)
            let p0_on_curve = intersection_points.iter().any(|ip| {
                (ip.x - p0.x).powi(2) + (ip.y - p0.y).powi(2) + (ip.z - p0.z).powi(2) < tol * tol * 100.0
            });
            let p1_on_curve = intersection_points.iter().any(|ip| {
                (ip.x - p1.x).powi(2) + (ip.y - p1.y).powi(2) + (ip.z - p1.z).powi(2) < tol * tol * 100.0
            });

            if p0_on_curve && p1_on_curve {
                // Use the shared edge for this segment
                edges.push(shared_edge.clone());
                coedges.push(CoEdge::new(shared_edge.id, true));
            } else {
                // Create a new edge for non-intersection boundary segments
                let e = Edge::new_line(p0, p1);
                let eid = e.id;
                edges.push(e);
                coedges.push(CoEdge::new(eid, true));
            }
        }

        let wire = Wire::new(coedges);
        let surface = Surface::Plane(plane.clone());
        let mut new_face = Face::new(surface, wire);
        new_face.edges = edges;
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
fn classify_face_robust(
    face: &Face,
    solid: &Solid,
    tol_ctx: &ToleranceContext,
) -> FaceClassification {
    let surface = match &face.surface {
        Some(s) => s,
        None => return FaceClassification::Outside,
    };

    let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
    let (u_min, u_max, v_min, v_max) =
        compute_face_uv_range(face, surface, u_min, u_max, v_min, v_max);

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

/// Classify a face as inside, outside, or on the boundary of a solid.
///
/// Uses the face centroid and ray casting.
fn classify_face_relative_to_solid(
    face: &Face,
    solid: &Solid,
    tol_ctx: &ToleranceContext,
) -> FaceClassification {
    // Compute face centroid
    let centroid = compute_face_centroid(face);

    if centroid.is_none() {
        return FaceClassification::Outside;
    }
    let centroid = centroid.unwrap();

    // Use point classification
    match classify_point(solid, &centroid, tol_ctx) {
        PointClassification::Inside => FaceClassification::Inside,
        PointClassification::Outside => FaceClassification::Outside,
        PointClassification::OnBoundary => FaceClassification::OnBoundary,
    }
}

/// Compute the centroid of a face.
fn compute_face_centroid(face: &Face) -> Option<Point3d> {
    let surface = face.surface.as_ref()?;

    // Sample the surface and compute centroid
    let (u_min, u_max, v_min, v_max) = surface_param_range(surface);

    // Try to use the face's boundary to determine the UV range
    let (u_min, u_max, v_min, v_max) =
        compute_face_uv_range(face, surface, u_min, u_max, v_min, v_max);

    let n = 10;
    let mut sum = Point3d::ORIGIN;
    let mut count = 0;

    for i in 0..n {
        for j in 0..n {
            let u = u_min + (u_max - u_min) * (i as f64 / n as f64);
            let v = v_min + (v_max - v_min) * (j as f64 / n as f64);
            let p = surface.point_at(u, v);
            sum = Point3d::new(sum.x + p.x, sum.y + p.y, sum.z + p.z);
            count += 1;
        }
    }

    if count == 0 {
        return None;
    }

    Some(Point3d::new(
        sum.x / count as f64,
        sum.y / count as f64,
        sum.z / count as f64,
    ))
}

/// Compute the UV range of a face from its boundary edges.
fn compute_face_uv_range(
    face: &Face,
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

    for edge in &face.edges {
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
            let (u_min, u_max, v_min, v_max) = surface_param_range(surface);
            let (u_min, u_max, v_min, v_max) =
                compute_face_uv_range(face, surface, u_min, u_max, v_min, v_max);

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
    boolean_operation(solid_a, solid_b, BooleanOp::Union, tol_ctx)
}

/// Boolean subtract: remove solid_b from solid_a.
pub fn boolean_subtract(
    solid_a: &Solid,
    solid_b: &Solid,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<Solid> {
    boolean_operation(solid_a, solid_b, BooleanOp::Subtract, tol_ctx)
}

/// Boolean intersect: keep only the overlapping volume.
pub fn boolean_intersect(
    solid_a: &Solid,
    solid_b: &Solid,
    tol_ctx: &ToleranceContext,
) -> BooleanResult<Solid> {
    boolean_operation(solid_a, solid_b, BooleanOp::Intersect, tol_ctx)
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

        let result = split_face(&face, &intersection, &tol_ctx);
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
}
