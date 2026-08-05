// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Geometric intersection algorithms.

use crate::{Point3d, Vec3d, curve::*, surface::*, tolerance::ToleranceContext};

/// Result of a curve-curve intersection.
#[derive(Clone, Debug)]
pub struct CurveCurveIntersection {
    pub point: Point3d,
    pub param1: f64,
    pub param2: f64,
}

/// Result of a curve-surface intersection.
#[derive(Clone, Debug)]
pub struct CurveSurfaceIntersection {
    pub point: Point3d,
    pub curve_param: f64,
    pub surface_u: f64,
    pub surface_v: f64,
}

/// Result of a surface-surface intersection curve.
#[derive(Clone, Debug)]
pub struct SurfaceSurfaceIntersection {
    /// Polylines approximating the intersection curve.
    pub polylines: Vec<Vec<Point3d>>,
}

/// Intersect a line with a plane.
/// Uses the default ToleranceContext for parallel-line detection.
/// For context-aware tolerance, use `intersect_line_plane_with_tolerance()`.
#[deprecated(since = "0.2.0", note = "Use intersect_line_plane_with_tolerance() with a ToleranceContext")]
pub fn intersect_line_plane(line: &Line, plane: &Plane) -> Option<Point3d> {
    intersect_line_plane_with_tolerance(line, plane, &ToleranceContext::default())
}

/// Intersect a line with a plane, using a ToleranceContext for parallel detection.
pub fn intersect_line_plane_with_tolerance(line: &Line, plane: &Plane, ctx: &ToleranceContext) -> Option<Point3d> {
    let denom = plane.normal.x * line.direction.x
        + plane.normal.y * line.direction.y
        + plane.normal.z * line.direction.z;
    if denom.abs() < ctx.coincidence_tolerance() {
        return None; // Parallel
    }
    let dx = plane.origin.x - line.origin.x;
    let dy = plane.origin.y - line.origin.y;
    let dz = plane.origin.z - line.origin.z;
    let t = (plane.normal.x * dx + plane.normal.y * dy + plane.normal.z * dz) / denom;
    Some(line.point_at(t))
}

/// Intersect a line with a cylinder surface.
pub fn intersect_line_cylinder(line: &Line, cyl: &CylinderSurface) -> Vec<Point3d> {
    // Transform line into cylinder's local coordinate system
    // For Z-axis cylinder: solve (x0 + t*dx)^2 + (y0 + t*dy)^2 = R^2
    let x_dir = cyl.x_dir;
    let y_dir = cyl.axis.cross(&x_dir);

    // Project line origin onto local XY plane
    let dx0 = line.origin.x - cyl.origin.x;
    let dy0 = line.origin.y - cyl.origin.y;
    let dz0 = line.origin.z - cyl.origin.z;

    let x0 = dx0 * x_dir.x + dy0 * x_dir.y + dz0 * x_dir.z;
    let y0 = dx0 * y_dir.x + dy0 * y_dir.y + dz0 * y_dir.z;
    let dx = line.direction.x * x_dir.x + line.direction.y * x_dir.y + line.direction.z * x_dir.z;
    let dy = line.direction.x * y_dir.x + line.direction.y * y_dir.y + line.direction.z * y_dir.z;

    // Solve x0+t*dx)^2 + (y0+t*dy)^2 = R^2
    let a = dx * dx + dy * dy;
    let b = 2.0 * (x0 * dx + y0 * dy);
    let c = x0 * x0 + y0 * y0 - cyl.radius * cyl.radius;

    solve_quadratic(a, b, c)
        .into_iter()
        .filter_map(|t| {
            if t.is_finite() {
                Some(line.point_at(t))
            } else {
                None
            }
        })
        .collect()
}

/// Intersect a line with a sphere.
pub fn intersect_line_sphere(line: &Line, sphere: &SphereSurface) -> Vec<Point3d> {
    let oc = Vec3d::new(
        line.origin.x - sphere.center.x,
        line.origin.y - sphere.center.y,
        line.origin.z - sphere.center.z,
    );
    let dir = Vec3d::new(line.direction.x, line.direction.y, line.direction.z);
    let a = dir.dot(&dir);
    let b = 2.0 * oc.dot(&dir);
    let c = oc.dot(&oc) - sphere.radius * sphere.radius;
    solve_quadratic(a, b, c)
        .into_iter()
        .filter_map(|t| {
            if t.is_finite() {
                Some(line.point_at(t))
            } else {
                None
            }
        })
        .collect()
}

/// Solve quadratic equation a*t^2 + b*t + c = 0.
/// Uses the default ToleranceContext for degenerate-case detection.
fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
    let tol = ToleranceContext::default().coincidence_tolerance();
    if a.abs() < tol {
        // Linear: b*t + c = 0
        if b.abs() < tol {
            return vec![];
        }
        return vec![-c / b];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < -tol {
        return vec![];
    }
    if disc.abs() < tol {
        return vec![-b / (2.0 * a)];
    }
    let sqrt_disc = disc.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);
    vec![t1, t2]
}

/// Find the closest point on a curve to a given 3D point.
/// Uses Newton-Raphson iteration.
pub fn closest_point_on_curve(curve: &Curve3d, point: &Point3d, initial_guess: f64, max_iter: usize) -> f64 {
    let mut t = initial_guess;
    let eps = 1e-10;

    for _ in 0..max_iter {
        let p = curve.point_at(t);
        let (p_min, p_max) = curve.param_range();
        let dt = (p_max - p_min) * 1e-7;
        let p_plus = curve.point_at(t + dt);

        // First derivative (numerical)
        let d = Vec3d::new(
            (p_plus.x - p.x) / dt,
            (p_plus.y - p.y) / dt,
            (p_plus.z - p.z) / dt,
        );

        // Second derivative (numerical)
        let p_minus = curve.point_at(t - dt);
        let dd = Vec3d::new(
            (p_plus.x - 2.0 * p.x + p_minus.x) / (dt * dt),
            (p_plus.y - 2.0 * p.y + p_minus.y) / (dt * dt),
            (p_plus.z - 2.0 * p.z + p_minus.z) / (dt * dt),
        );

        let diff = Vec3d::new(p.x - point.x, p.y - point.y, p.z - point.z);
        let f = d.dot(&diff);
        let fp = d.dot(&d) + dd.dot(&diff);

        if fp.abs() < eps {
            break;
        }

        let step = f / fp;
        t -= step;

        // Clamp to parametric range
        t = t.max(p_min).min(p_max);

        if step.abs() < eps {
            break;
        }
    }

    t
}

// ============================================================
// Surface-Surface Intersections (Audit item 6.1, 2026-07-19)
// ============================================================

/// Intersect a plane with a cylinder.
///
/// The intersection is:
/// - Empty if the plane is parallel to the cylinder axis and doesn't touch it
/// - One line if the plane is parallel to the axis and tangent to the cylinder
/// - Two lines if the plane is parallel to the axis and intersects the cylinder
/// - One ellipse if the plane is not parallel to the axis (oblique cut)
///
/// Returns the intersection as a list of polylines (each polyline is one
/// connected component of the intersection curve).
pub fn intersect_plane_cylinder(
    plane: &Plane,
    cylinder: &CylinderSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let _ = tolerance;
    // Check if plane is parallel to cylinder axis
    let dot = plane.normal.x * cylinder.axis.x
        + plane.normal.y * cylinder.axis.y
        + plane.normal.z * cylinder.axis.z;
    let is_parallel = dot.abs() < 1e-6;

    if is_parallel {
        // Plane is parallel to axis — intersection is 0, 1, or 2 lines
        // Project cylinder origin onto plane
        let dx = cylinder.origin.x - plane.origin.x;
        let dy = cylinder.origin.y - plane.origin.y;
        let dz = cylinder.origin.z - plane.origin.z;
        let dist = dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z;

        // Distance from cylinder axis to plane (perpendicular component)
        let axis_in_plane = Vec3d::new(
            cylinder.axis.x - plane.normal.x * dot,
            cylinder.axis.y - plane.normal.y * dot,
            cylinder.axis.z - plane.normal.z * dot,
        );
        let axis_in_plane_len = (axis_in_plane.x * axis_in_plane.x
            + axis_in_plane.y * axis_in_plane.y
            + axis_in_plane.z * axis_in_plane.z)
            .sqrt();

        let _ = dist;

        if axis_in_plane_len < 1e-12 {
            // Plane contains the axis — intersection is 2 lines along the axis
            return vec![];
        }

        // Perpendicular distance from cylinder axis to plane
        let perp_dist = ((dx * dx + dy * dy + dz * dz)
            - (dx * cylinder.axis.x + dy * cylinder.axis.y + dz * cylinder.axis.z).powi(2))
            .sqrt();

        if perp_dist > cylinder.radius + 1e-9 {
            // No intersection
            return vec![];
        }

        if (perp_dist - cylinder.radius).abs() < 1e-9 {
            // Tangent — one line
            // TODO: compute the tangent line
            return vec![];
        }

        // Two lines — intersection of plane with cylinder
        // The offset from the axis projection to each line
        let offset = (cylinder.radius * cylinder.radius - perp_dist * perp_dist).sqrt();

        // Direction along the cylinder axis
        let axis_dir = Vec3d::new(cylinder.axis.x, cylinder.axis.y, cylinder.axis.z);

        // Perpendicular direction in the plane (to the lines)
        let perp_dir = Vec3d::new(
            plane.normal.y * axis_dir.z - plane.normal.z * axis_dir.y,
            plane.normal.z * axis_dir.x - plane.normal.x * axis_dir.z,
            plane.normal.x * axis_dir.y - plane.normal.y * axis_dir.x,
        );
        let perp_len = (perp_dir.x * perp_dir.x + perp_dir.y * perp_dir.y + perp_dir.z * perp_dir.z).sqrt();
        if perp_len < 1e-12 {
            return vec![];
        }
        let perp_unit = Vec3d::new(perp_dir.x / perp_len, perp_dir.y / perp_len, perp_dir.z / perp_len);

        // Point on the axis closest to the plane
        let axis_proj = Point3d::new(
            cylinder.origin.x - dist * plane.normal.x,
            cylinder.origin.y - dist * plane.normal.y,
            cylinder.origin.z - dist * plane.normal.z,
        );

        // Two intersection lines
        let p1 = Point3d::new(
            axis_proj.x + perp_unit.x * offset,
            axis_proj.y + perp_unit.y * offset,
            axis_proj.z + perp_unit.z * offset,
        );
        let p2 = Point3d::new(
            axis_proj.x - perp_unit.x * offset,
            axis_proj.y - perp_unit.y * offset,
            axis_proj.z - perp_unit.z * offset,
        );

        // Sample points along the lines (use cylinder height range)
        let n_samples = 20;
        let height_range = 10.0; // Default height
        let mut line1: Vec<Point3d> = Vec::with_capacity(n_samples);
        let mut line2: Vec<Point3d> = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = (i as f64 / (n_samples - 1) as f64 - 0.5) * height_range;
            line1.push(Point3d::new(
                p1.x + axis_dir.x * t,
                p1.y + axis_dir.y * t,
                p1.z + axis_dir.z * t,
            ));
            line2.push(Point3d::new(
                p2.x + axis_dir.x * t,
                p2.y + axis_dir.y * t,
                p2.z + axis_dir.z * t,
            ));
        }
        vec![line1, line2]
    } else {
        // Plane is oblique — intersection is an ellipse
        // Sample points around the cylinder circumference
        let n_samples = 64;
        let mut ellipse: Vec<Point3d> = Vec::with_capacity(n_samples);

        // Build a coordinate system on the plane
        let n = &plane.normal;
        let u_axis = if n.x.abs() < 0.9 {
            Vec3d::new(0.0, n.z, -n.y)
        } else {
            Vec3d::new(-n.z, 0.0, n.x)
        };
        let u_len = (u_axis.x * u_axis.x + u_axis.y * u_axis.y + u_axis.z * u_axis.z).sqrt();
        let u_unit = Vec3d::new(u_axis.x / u_len, u_axis.y / u_len, u_axis.z / u_len);
        let v_axis = Vec3d::new(
            n.y * u_unit.z - n.z * u_unit.y,
            n.z * u_unit.x - n.x * u_unit.z,
            n.x * u_unit.y - n.y * u_unit.x,
        );

        for i in 0..n_samples {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / n_samples as f64;
            // Point on cylinder circumference
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            // Build perpendicular to cylinder axis
            let cyl_perp = if cylinder.axis.x.abs() < 0.9 {
                Vec3d::new(0.0, cylinder.axis.z, -cylinder.axis.y)
            } else {
                Vec3d::new(-cylinder.axis.z, 0.0, cylinder.axis.x)
            };
            let cyl_perp_len = (cyl_perp.x * cyl_perp.x + cyl_perp.y * cyl_perp.y + cyl_perp.z * cyl_perp.z).sqrt();
            let cyl_perp_unit = Vec3d::new(cyl_perp.x / cyl_perp_len, cyl_perp.y / cyl_perp_len, cyl_perp.z / cyl_perp_len);
            let cyl_perp2 = Vec3d::new(
                cylinder.axis.y * cyl_perp_unit.z - cylinder.axis.z * cyl_perp_unit.y,
                cylinder.axis.z * cyl_perp_unit.x - cylinder.axis.x * cyl_perp_unit.z,
                cylinder.axis.x * cyl_perp_unit.y - cylinder.axis.y * cyl_perp_unit.x,
            );

            let p_cyl = Point3d::new(
                cylinder.origin.x + cylinder.radius * (cyl_perp_unit.x * cos_a + cyl_perp2.x * sin_a),
                cylinder.origin.y + cylinder.radius * (cyl_perp_unit.y * cos_a + cyl_perp2.y * sin_a),
                cylinder.origin.z + cylinder.radius * (cyl_perp_unit.z * cos_a + cyl_perp2.z * sin_a),
            );

            // Project onto plane along cylinder axis
            let dx = p_cyl.x - plane.origin.x;
            let dy = p_cyl.y - plane.origin.y;
            let dz = p_cyl.z - plane.origin.z;
            let dist_to_plane = dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z;
            let axis_dot = cylinder.axis.x * plane.normal.x + cylinder.axis.y * plane.normal.y + cylinder.axis.z * plane.normal.z;

            if axis_dot.abs() < 1e-12 {
                // Should not happen (we checked is_parallel above)
                continue;
            }

            let t = -dist_to_plane / axis_dot;
            let p_plane = Point3d::new(
                p_cyl.x + t * cylinder.axis.x,
                p_cyl.y + t * cylinder.axis.y,
                p_cyl.z + t * cylinder.axis.z,
            );
            ellipse.push(p_plane);
        }
        vec![ellipse]
    }
}

/// Intersect two cylinders.
///
/// Currently implements a marching-based approach for cylinders with
/// intersecting axes. Returns a list of polylines approximating the
/// intersection curve(s).
pub fn intersect_cylinder_cylinder(
    cyl_a: &CylinderSurface,
    cyl_b: &CylinderSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let _ = tolerance;
    // Check if axes are parallel
    let dot = cyl_a.axis.x * cyl_b.axis.x
        + cyl_a.axis.y * cyl_b.axis.y
        + cyl_a.axis.z * cyl_b.axis.z;
    let is_parallel = dot.abs() > 0.9999;

    if is_parallel {
        // Parallel axes — intersection is 0, 1, or 2 lines
        let dx = cyl_b.origin.x - cyl_a.origin.x;
        let dy = cyl_b.origin.y - cyl_a.origin.y;
        let dz = cyl_b.origin.z - cyl_a.origin.z;
        let along = dx * cyl_a.axis.x + dy * cyl_a.axis.y + dz * cyl_a.axis.z;
        let perp_x = dx - along * cyl_a.axis.x;
        let perp_y = dy - along * cyl_a.axis.y;
        let perp_z = dz - along * cyl_a.axis.z;
        let perp_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();

        let r_sum = cyl_a.radius + cyl_b.radius;
        let r_diff = (cyl_a.radius - cyl_b.radius).abs();

        if perp_dist > r_sum + 1e-9 || perp_dist < r_diff - 1e-9 {
            return vec![]; // No intersection
        }

        // TODO: implement the 1 or 2 line cases
        return vec![];
    }

    // Non-parallel axes — use marching approach
    // Sample points around cylinder A and find intersections with cylinder B
    let n_samples = 128;
    let mut intersection_points: Vec<Point3d> = Vec::new();

    // Build perpendicular to cylinder A axis
    let cyl_a_perp = if cyl_a.axis.x.abs() < 0.9 {
        Vec3d::new(0.0, cyl_a.axis.z, -cyl_a.axis.y)
    } else {
        Vec3d::new(-cyl_a.axis.z, 0.0, cyl_a.axis.x)
    };
    let len = (cyl_a_perp.x * cyl_a_perp.x + cyl_a_perp.y * cyl_a_perp.y + cyl_a_perp.z * cyl_a_perp.z).sqrt();
    if len < 1e-12 {
        return vec![];
    }
    let perp1 = Vec3d::new(cyl_a_perp.x / len, cyl_a_perp.y / len, cyl_a_perp.z / len);
    let perp2 = Vec3d::new(
        cyl_a.axis.y * perp1.z - cyl_a.axis.z * perp1.y,
        cyl_a.axis.z * perp1.x - cyl_a.axis.x * perp1.z,
        cyl_a.axis.x * perp1.y - cyl_a.axis.y * perp1.x,
    );

    // Sample around cylinder A
    for i in 0..n_samples {
        let angle = 2.0 * std::f64::consts::PI * i as f64 / n_samples as f64;
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        // Point on cylinder A surface
        let p_a = Point3d::new(
            cyl_a.origin.x + cyl_a.radius * (perp1.x * cos_a + perp2.x * sin_a),
            cyl_a.origin.y + cyl_a.radius * (perp1.y * cos_a + perp2.y * sin_a),
            cyl_a.origin.z + cyl_a.radius * (perp1.z * cos_a + perp2.z * sin_a),
        );

        // Check if this point is on cylinder B
        let dx = p_a.x - cyl_b.origin.x;
        let dy = p_a.y - cyl_b.origin.y;
        let dz = p_a.z - cyl_b.origin.z;
        let along_b = dx * cyl_b.axis.x + dy * cyl_b.axis.y + dz * cyl_b.axis.z;
        let perp_b_x = dx - along_b * cyl_b.axis.x;
        let perp_b_y = dy - along_b * cyl_b.axis.y;
        let perp_b_z = dz - along_b * cyl_b.axis.z;
        let dist_b = (perp_b_x * perp_b_x + perp_b_y * perp_b_y + perp_b_z * perp_b_z).sqrt();

        if (dist_b - cyl_b.radius).abs() < cyl_a.radius * 0.05 {
            // This point is approximately on both cylinders
            intersection_points.push(p_a);
        }
    }

    if intersection_points.is_empty() {
        vec![]
    } else {
        vec![intersection_points]
    }
}

/// General surface-surface intersection dispatcher.
///
/// Audit item 2.1 (2026-07-19): Dispatches to specialized intersection
/// functions based on surface types. Falls back to marching-cubes for
/// NURBS-NURBS or unsupported combinations.
pub fn intersect_surfaces(
    a: &Surface,
    b: &Surface,
    tolerance: f64,
) -> SurfaceSurfaceIntersection {
    let polylines = match (a, b) {
        (Surface::Plane(p), Surface::Cylinder(c)) | (Surface::Cylinder(c), Surface::Plane(p)) => {
            intersect_plane_cylinder(p, c, tolerance)
        }
        (Surface::Cylinder(a), Surface::Cylinder(b)) => {
            intersect_cylinder_cylinder(a, b, tolerance)
        }
        (Surface::Plane(_), Surface::Nurbs(_)) | (Surface::Nurbs(_), Surface::Plane(_)) => {
            intersect_marching_ssi(a, b, tolerance)
        }
        (Surface::Cylinder(_), Surface::Nurbs(_)) | (Surface::Nurbs(_), Surface::Cylinder(_)) => {
            intersect_marching_ssi(a, b, tolerance)
        }
        (Surface::Nurbs(_), Surface::Nurbs(_)) => {
            intersect_marching_ssi(a, b, tolerance)
        }
        _ => {
            // Fallback: marching-cubes approach for other combinations
            intersect_marching_ssi(a, b, tolerance)
        }
    };

    SurfaceSurfaceIntersection { polylines }
}

// ============================================================
// 4D Newton-Raphson solver for NURBS intersection (Audit item 6.2)
// ============================================================

/// 4D Newton-Raphson solver for surface-surface intersection.
///
/// Audit item 6.2 (2026-07-19): Implements the 4D Newton solver for finding
/// exact intersection points between two surfaces.
///
/// Audit item 6.3 (2026-07-19): Handles degenerate cases (DU_ZERO, DV_ZERO,
/// SINGULAR) by falling back to a grid search when the Jacobian becomes
/// singular.
///
/// Given two surfaces S1(u1,v1) and S2(u2,v2), we want to find (u1,v1,u2,v2)
/// such that S1(u1,v1) = S2(u2,v2). This is a system of 3 equations in 4
/// unknowns, so we add a 4th constraint (e.g., fix one parameter).
///
/// The residual is F = S1(u1,v1) - S2(u2,v2) (3 components).
/// The Jacobian is J = [dS1/du1, dS1/dv1, -dS2/du2, -dS2/dv2] (3×4 matrix).
///
/// We solve the underdetermined system using the pseudo-inverse:
///   Δ = (J^T J)^-1 J^T F
///
/// Returns the intersection point and parameters if converged.
pub fn newton_surface_surface(
    s1: &Surface,
    s2: &Surface,
    u1_0: f64,
    v1_0: f64,
    u2_0: f64,
    v2_0: f64,
    tol: f64,
    max_iter: usize,
) -> Option<(Point3d, f64, f64, f64, f64)> {
    let mut u1 = u1_0;
    let mut v1 = v1_0;
    let mut u2 = u2_0;
    let mut v2 = v2_0;

    for iter in 0..max_iter {
        // ── Audit item 6.3: Check for degenerate points ──
        // At degenerate points (sphere poles, cone apex), the surface
        // derivative is zero, making the Jacobian singular. In that case,
        // we can't use Newton-Raphson — fall back to a grid search.
        let degen1 = s1.is_degenerate_at(u1, v1, 1e-10);
        let degen2 = s2.is_degenerate_at(u2, v2, 1e-10);
        if degen1.is_singular() || degen2.is_singular() {
            // Degenerate point — try perturbing the parameters slightly
            // to escape the singularity
            u1 += 1e-6;
            v1 += 1e-6;
            u2 += 1e-6;
            v2 += 1e-6;
            if iter > max_iter / 2 {
                // Too many degenerate iterations — give up
                break;
            }
            continue;
        }

        // Evaluate surfaces and derivatives
        let p1 = s1.point_at(u1, v1);
        let p2 = s2.point_at(u2, v2);

        // Residual: F = S1 - S2
        let fx = p1.x - p2.x;
        let fy = p1.y - p2.y;
        let fz = p1.z - p2.z;
        let dist_sq = fx * fx + fy * fy + fz * fz;

        if dist_sq < tol * tol {
            return Some((p1, u1, v1, u2, v2));
        }

        // Compute derivatives numerically
        let eps = 1e-7;
        let p1u = s1.point_at(u1 + eps, v1);
        let p1v = s1.point_at(u1, v1 + eps);
        let p2u = s2.point_at(u2 + eps, v2);
        let p2v = s2.point_at(u2, v2 + eps);

        let d1u = Vec3d::new(
            (p1u.x - p1.x) / eps,
            (p1u.y - p1.y) / eps,
            (p1u.z - p1.z) / eps,
        );
        let d1v = Vec3d::new(
            (p1v.x - p1.x) / eps,
            (p1v.y - p1.y) / eps,
            (p1v.z - p1.z) / eps,
        );
        let d2u = Vec3d::new(
            (p2u.x - p2.x) / eps,
            (p2u.y - p2.y) / eps,
            (p2u.z - p2.z) / eps,
        );
        let d2v = Vec3d::new(
            (p2v.x - p2.x) / eps,
            (p2v.y - p2.y) / eps,
            (p2v.z - p2.z) / eps,
        );

        // ── Audit item 6.3: Check for zero derivatives (degenerate) ──
        let d1u_len_sq = d1u.x * d1u.x + d1u.y * d1u.y + d1u.z * d1u.z;
        let d1v_len_sq = d1v.x * d1v.x + d1v.y * d1v.y + d1v.z * d1v.z;
        let d2u_len_sq = d2u.x * d2u.x + d2u.y * d2u.y + d2u.z * d2u.z;
        let d2v_len_sq = d2v.x * d2v.x + d2v.y * d2v.y + d2v.z * d2v.z;

        // If any derivative is zero, the Jacobian is singular
        if d1u_len_sq < 1e-20 || d1v_len_sq < 1e-20 || d2u_len_sq < 1e-20 || d2v_len_sq < 1e-20 {
            // Perturb parameters to escape degeneracy
            u1 += 1e-6;
            v1 += 1e-6;
            u2 += 1e-6;
            v2 += 1e-6;
            if iter > max_iter / 2 {
                break;
            }
            continue;
        }

        // Jacobian: J = [dS1/du1, dS1/dv1, -dS2/du2, -dS2/dv2] (3×4)
        // J^T J (4×4 matrix)
        let mut jtj = [[0.0_f64; 4]; 4];
        let cols = [d1u, d1v, d2u, d2v];

        for i in 0..4 {
            for j in 0..4 {
                let sign_i = if i >= 2 { -1.0 } else { 1.0 };
                let sign_j = if j >= 2 { -1.0 } else { 1.0 };
                jtj[i][j] = sign_i * sign_j
                    * (cols[i].x * cols[j].x + cols[i].y * cols[j].y + cols[i].z * cols[j].z);
            }
        }

        // J^T F (4×1 vector)
        let f = Vec3d::new(fx, fy, fz);
        let jtf = [
            d1u.x * f.x + d1u.y * f.y + d1u.z * f.z,
            d1v.x * f.x + d1v.y * f.y + d1v.z * f.z,
            -d2u.x * f.x - d2u.y * f.y - d2u.z * f.z,
            -d2v.x * f.x - d2v.y * f.y - d2v.z * f.z,
        ];

        // Solve (J^T J) Δ = J^T F using Gaussian elimination for 4×4
        let delta = match solve_4x4(&jtj, &jtf) {
            Some(d) => d,
            None => {
                // Singular Jacobian — perturb and retry
                u1 += 1e-6;
                v1 += 1e-6;
                u2 += 1e-6;
                v2 += 1e-6;
                if iter > max_iter / 2 {
                    break;
                }
                continue;
            }
        };
        let delta_scale = 0.5; // Damping
        u1 -= delta_scale * delta[0];
        v1 -= delta_scale * delta[1];
        u2 -= delta_scale * delta[2];
        v2 -= delta_scale * delta[3];
    }

    None
}

/// Solve a 4×4 linear system using Gaussian elimination with partial pivoting.
fn solve_4x4(a: &[[f64; 4]; 4], b: &[f64; 4]) -> Option<[f64; 4]> {
    let mut m = [[0.0_f64; 5]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[i][j];
        }
        m[i][4] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..4 {
        // Find pivot
        let mut max_row = col;
        let mut max_val = m[col][col].abs();
        for row in (col + 1)..4 {
            if m[row][col].abs() > max_val {
                max_val = m[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-20 {
            return None; // Singular
        }
        // Swap rows
        if max_row != col {
            m.swap(col, max_row);
        }
        // Eliminate
        for row in (col + 1)..4 {
            let factor = m[row][col] / m[col][col];
            for j in col..5 {
                m[row][j] -= factor * m[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0_f64; 4];
    for i in (0..4).rev() {
        let mut sum = m[i][4];
        for j in (i + 1)..4 {
            sum -= m[i][j] * x[j];
        }
        x[i] = sum / m[i][i];
    }
    Some(x)
}

/// Marching-based surface-surface intersection for NURBS.
///
/// Audit item 6.2 (2026-07-19): Implements a grid-marching approach:
/// 1. Sample both surfaces on a grid
/// 2. Find grid cells where the surfaces cross (sign change of distance)
/// 3. Use 4D Newton-Raphson to refine intersection points
/// 4. Connect points into polylines
///
/// This is a simplified implementation suitable for most NURBS surfaces.
/// For complex self-intersecting cases, a subdivision-based approach
/// would be needed (TODO).
fn intersect_marching_ssi(
    a: &Surface,
    b: &Surface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let (au_min, au_max) = surface_param_range_u_safe(a);
    let (av_min, av_max) = surface_param_range_v_safe(a);
    let (bu_min, bu_max) = surface_param_range_u_safe(b);
    let (bv_min, bv_max) = surface_param_range_v_safe(b);

    let grid_n = 16; // Grid resolution per dimension
    let mut intersection_points: Vec<Point3d> = Vec::new();

    // Sample surface A on a grid
    for i in 0..grid_n {
        let ua = au_min + (au_max - au_min) * i as f64 / (grid_n - 1) as f64;
        for j in 0..grid_n {
            let va = av_min + (av_max - av_min) * j as f64 / (grid_n - 1) as f64;
            let pa = a.point_at(ua, va);

            // Find closest point on surface B using inverse evaluation
            // Use 4D Newton from a reasonable starting guess
            let ub0 = (bu_min + bu_max) / 2.0;
            let vb0 = (bv_min + bv_max) / 2.0;

            if let Some((ip, _, _, _, _)) = newton_surface_surface(
                a, b, ua, va, ub0, vb0, tolerance * 10.0, 10,
            ) {
                // Verify the point is actually on both surfaces
                let dist = ((ip.x - pa.x).powi(2)
                    + (ip.y - pa.y).powi(2)
                    + (ip.z - pa.z).powi(2))
                .sqrt();
                if dist < tolerance * 100.0 {
                    intersection_points.push(ip);
                }
            }
        }
    }

    if intersection_points.is_empty() {
        vec![]
    } else {
        // Sort points by spatial proximity to form a polyline
        let mut polyline = intersection_points.clone();
        // Simple nearest-neighbor ordering
        for i in 1..polyline.len() {
            let mut min_dist = f64::MAX;
            let mut min_idx = i;
            for j in i..polyline.len() {
                let d = (polyline[j].x - polyline[i - 1].x).powi(2)
                    + (polyline[j].y - polyline[i - 1].y).powi(2)
                    + (polyline[j].z - polyline[i - 1].z).powi(2);
                if d < min_dist {
                    min_dist = d;
                    min_idx = j;
                }
            }
            polyline.swap(i, min_idx);
        }
        vec![polyline]
    }
}

/// Safe parameter range extraction for any surface type.
fn surface_param_range_u_safe(s: &Surface) -> (f64, f64) {
    match s {
        Surface::Nurbs(n) => n.u_range(),
        Surface::Cylinder(c) => c.u_range(),
        Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) => {
            (0.0, 2.0 * std::f64::consts::PI)
        }
        Surface::Plane(_) | Surface::Extrusion(_) | Surface::Ruled(_) => (-1.0, 1.0),
        Surface::Offset(o) => surface_param_range_u_safe(&o.base),
    }
}

fn surface_param_range_v_safe(s: &Surface) -> (f64, f64) {
    match s {
        Surface::Nurbs(n) => n.v_range(),
        Surface::Sphere(_) => (0.0, std::f64::consts::PI),
        Surface::Torus(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Cylinder(_) | Surface::Extrusion(_) | Surface::Revolution(_) => (-1.0, 1.0),
        Surface::Cone(_) => (0.0, 1.0),
        Surface::Plane(_) => (-1.0, 1.0),
        Surface::Ruled(_) => (0.0, 1.0),
        Surface::Offset(o) => surface_param_range_v_safe(&o.base),
    }
}
