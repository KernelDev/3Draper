//! Geometric intersection algorithms.

use crate::{Point3d, Direction3d, Vec3d, curve::*, surface::*, tolerance::TOLERANCE};

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
pub fn intersect_line_plane(line: &Line, plane: &Plane) -> Option<Point3d> {
    let denom = plane.normal.x * line.direction.x
        + plane.normal.y * line.direction.y
        + plane.normal.z * line.direction.z;
    if denom.abs() < TOLERANCE {
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
    let x_dir = if cyl.axis.is_parallel_to(&Direction3d::Z) {
        Direction3d::X
    } else {
        cyl.axis.cross(&Direction3d::Z)
    };
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
fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
    if a.abs() < TOLERANCE {
        // Linear: b*t + c = 0
        if b.abs() < TOLERANCE {
            return vec![];
        }
        return vec![-c / b];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < -TOLERANCE {
        return vec![];
    }
    if disc.abs() < TOLERANCE {
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
