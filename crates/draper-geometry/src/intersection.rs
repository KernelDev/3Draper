//! Intersection and projection utilities.
//!
//! Provides algorithms for computing intersections between geometric primitives
//! and projecting points onto curves and surfaces.

use crate::curve::Curve;
use crate::direction::Direction3;
use crate::point::Point3;
use crate::surface::Surface;

/// Result of an intersection computation.
#[derive(Debug, Clone)]
pub struct IntersectionPoint {
    pub point: Point3,
    pub u1: f64,
    pub v1: Option<f64>,
    pub u2: f64,
    pub v2: Option<f64>,
}

/// Find the intersection of a ray with a surface.
/// Returns a list of (parameter on surface u, v, t along ray) tuples.
pub fn ray_surface_intersection(
    ray_origin: Point3,
    ray_direction: Direction3,
    surface: &Surface,
    tolerance: f64,
) -> Vec<(f64, f64, f64)> {
    // Use Newton-Raphson iteration on the surface
    // For now, use a grid-based search followed by refinement
    let mut results = Vec::new();

    let grid_size = 20;
    for i in 0..grid_size {
        for j in 0..grid_size {
            let u = i as f64 / (grid_size - 1) as f64;
            let v = j as f64 / (grid_size - 1) as f64;

            let surf_pt = surface.point_at(u, v);
            let to_point = surf_pt - ray_origin;
            let t = to_point.dot(ray_direction.to_dvec3());

            if t < 0.0 {
                continue;
            }

            let closest_on_ray = ray_origin + ray_direction.to_dvec3() * t;
            let dist = closest_on_ray.distance_to(surf_pt);

            if dist < tolerance {
                results.push((u, v, t));
            }
        }
    }

    results
}

/// Project a point onto a surface, returning the closest (u, v) parameters.
pub fn project_point_to_surface(
    point: Point3,
    surface: &Surface,
    tolerance: f64,
    max_iterations: usize,
) -> Option<(f64, f64)> {
    // Initial guess: sample the surface and find the closest point
    let mut best_u = 0.5;
    let mut best_v = 0.5;
    let mut best_dist = f64::MAX;

    let grid_size = 20;
    for i in 0..=grid_size {
        for j in 0..=grid_size {
            let u = i as f64 / grid_size as f64;
            let v = j as f64 / grid_size as f64;
            let surf_pt = surface.point_at(u, v);
            let dist = point.distance_to(surf_pt);
            if dist < best_dist {
                best_dist = dist;
                best_u = u;
                best_v = v;
            }
        }
    }

    // Newton-Raphson refinement
    let eps = 1e-7;
    for _ in 0..max_iterations {
        let s = surface.point_at(best_u, best_v);
        let su = surface.point_at(best_u + eps, best_v);
        let sv = surface.point_at(best_u, best_v + eps);

        let ds_du = (su - s) / eps;
        let ds_dv = (sv - s) / eps;

        let diff = point - s;

        // Solve: [ds_du . ds_du, ds_du . ds_dv] [du]   [diff . ds_du]
        //        [ds_dv . ds_du, ds_dv . ds_dv] [dv] = [diff . ds_dv]
        let a11 = ds_du.dot(ds_du);
        let a12 = ds_du.dot(ds_dv);
        let a22 = ds_dv.dot(ds_dv);
        let b1 = diff.dot(ds_du);
        let b2 = diff.dot(ds_dv);

        let det = a11 * a22 - a12 * a12;
        if det.abs() < 1e-20 {
            break;
        }

        let du = (a22 * b1 - a12 * b2) / det;
        let dv = (a11 * b2 - a12 * b1) / det;

        best_u += du;
        best_v += dv;

        if du.abs() < tolerance && dv.abs() < tolerance {
            break;
        }
    }

    Some((best_u, best_v))
}

/// Find the closest point on a curve to a given point.
pub fn project_point_to_curve(
    point: Point3,
    curve: &Curve,
    tolerance: f64,
    max_iterations: usize,
) -> Option<f64> {
    // Sample the curve and find the closest parameter
    let mut best_t = 0.5;
    let mut best_dist = f64::MAX;

    let samples = 100;
    for i in 0..=samples {
        let t = i as f64 / samples as f64;
        let curve_pt = curve.point_at(t);
        let dist = point.distance_to(curve_pt);
        if dist < best_dist {
            best_dist = dist;
            best_t = t;
        }
    }

    // Newton-Raphson refinement
    let eps = 1e-7;
    for _ in 0..max_iterations {
        let p = curve.point_at(best_t);
        let pu = curve.point_at(best_t + eps);

        let dp_dt = (pu - p) / eps;
        let diff = point - p;

        let numerator = diff.dot(dp_dt);
        let denominator = dp_dt.dot(dp_dt);

        if denominator.abs() < 1e-20 {
            break;
        }

        let dt = numerator / denominator;
        best_t += dt;

        if dt.abs() < tolerance {
            break;
        }
    }

    Some(best_t)
}
