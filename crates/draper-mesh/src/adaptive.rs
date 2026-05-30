//! Adaptive triangulation — compute required samples from surface curvature and max_deviation.
//!
//! Instead of using fixed `angular_samples` and `height_samples` for all surfaces,
//! this module computes the required number of samples based on:
//! - The maximum curvature of the surface
//! - The user-specified `max_deviation` (maximum allowed distance between
//!   the triangulated mesh and the true surface)
//! - The user-specified `max_angular_deviation` (maximum allowed angle
//!   between adjacent face normals)
//!
//! The formula for required samples on a circular arc is:
//!   n = ceil(angle / (2 * acos(1 - max_deviation / radius)))
//!
//! For planes, the minimum number of triangles is always used.

use draper_geometry::Surface;

/// Minimum number of angular samples for any curved surface.
const MIN_ANGULAR_SAMPLES: usize = 6;

/// Maximum number of angular samples (to prevent excessive tessellation).
const MAX_ANGULAR_SAMPLES: usize = 256;

/// Minimum number of height/v samples for any curved surface.
const MIN_HEIGHT_SAMPLES: usize = 2;

/// Maximum number of height/v samples.
const MAX_HEIGHT_SAMPLES: usize = 256;

/// Compute the required number of angular (u-direction) samples for a surface
/// given a maximum deviation tolerance.
///
/// For a circular arc of angle θ and radius r, the chord deviation is:
///   d = r * (1 - cos(θ/(2n)))
/// Solving for n:
///   n = ceil(θ / (2 * acos(1 - d/r)))
///
/// For non-circular surfaces, we use the maximum curvature to estimate
/// the equivalent radius: r_eq = 1/k_max.
pub fn required_angular_samples(
    surface: &Surface,
    u_start: f64,
    u_end: f64,
    v_start: f64,
    v_end: f64,
    max_deviation: f64,
    detail_level: f64,
) -> usize {
    match surface {
        Surface::Plane(_) => {
            // Planes need minimum samples — they're always flat
            MIN_ANGULAR_SAMPLES
        }
        Surface::Cylinder(cyl) => {
            samples_for_arc_radius(u_end - u_start, cyl.radius, max_deviation, detail_level)
        }
        Surface::Cone(cone) => {
            // Cone radius varies along v — use average radius
            let r_start = if cone.expanding {
                v_start * cone.half_angle.tan()
            } else {
                (cone.radius - v_start * cone.half_angle.tan()).max(0.0)
            };
            let r_end = if cone.expanding {
                v_end * cone.half_angle.tan()
            } else {
                (cone.radius - v_end * cone.half_angle.tan()).max(0.0)
            };
            let avg_radius = (r_start + r_end) / 2.0;
            if avg_radius < 1e-10 {
                MIN_ANGULAR_SAMPLES
            } else {
                samples_for_arc_radius(u_end - u_start, avg_radius, max_deviation, detail_level)
            }
        }
        Surface::Sphere(sphere) => {
            // Sphere curvature is 1/radius everywhere
            samples_for_arc_radius(u_end - u_start, sphere.radius, max_deviation, detail_level)
        }
        Surface::Torus(torus) => {
            // Torus: the effective radius in u-direction varies with v
            // Use the minimum radius for conservative estimate
            let r_inner = torus.major_radius - torus.minor_radius;
            let r_eff = if r_inner > 1e-10 { r_inner } else { torus.major_radius };
            samples_for_arc_radius(u_end - u_start, r_eff, max_deviation, detail_level)
        }
        _ => {
            // For NURBS, Revolution, Extrusion: sample curvature at several points
            // and use the maximum curvature
            let max_k = max_curvature_over_domain(surface, u_start, u_end, v_start, v_end);
            if max_k < 1e-10 {
                return MIN_ANGULAR_SAMPLES;
            }
            let r_eq = 1.0 / max_k;
            samples_for_arc_radius(u_end - u_start, r_eq, max_deviation, detail_level)
        }
    }
}

/// Compute the required number of height/v-direction samples for a surface
/// given a maximum deviation tolerance.
pub fn required_height_samples(
    surface: &Surface,
    u_start: f64,
    u_end: f64,
    v_start: f64,
    v_end: f64,
    max_deviation: f64,
    detail_level: f64,
) -> usize {
    let v_range = v_end - v_start;
    if v_range < 1e-10 {
        return MIN_HEIGHT_SAMPLES;
    }

    match surface {
        Surface::Plane(_) => MIN_HEIGHT_SAMPLES,
        Surface::Cylinder(_) => {
            // Cylinder has zero curvature in v-direction (along axis)
            MIN_HEIGHT_SAMPLES.max(2)
        }
        Surface::Cone(cone) => {
            // Cone has zero meridional curvature (along generator)
            // But the radius changes, so we need at least a few samples
            let r_start = if cone.expanding {
                v_start * cone.half_angle.tan()
            } else {
                (cone.radius - v_start * cone.half_angle.tan()).max(0.0)
            };
            let r_end = if cone.expanding {
                v_end * cone.half_angle.tan()
            } else {
                (cone.radius - v_end * cone.half_angle.tan()).max(0.0)
            };
            // If radius changes significantly, need more samples
            let radius_ratio = (r_start / r_end.max(1e-10)).max(r_end / r_start.max(1e-10));
            let n = if radius_ratio > 2.0 { 8 } else if radius_ratio > 1.5 { 4 } else { 2 };
            let n_scaled = (n as f64 * detail_level).ceil() as usize;
            n_scaled.max(MIN_HEIGHT_SAMPLES).min(MAX_HEIGHT_SAMPLES)
        }
        Surface::Sphere(sphere) => {
            // v goes from polar angle v_start to v_end
            // The effective "radius" in v-direction is also sphere.radius
            samples_for_arc_radius(v_range, sphere.radius, max_deviation, detail_level)
        }
        Surface::Torus(torus) => {
            // v-direction is around the tube with radius minor_radius
            samples_for_arc_radius(v_range, torus.minor_radius, max_deviation, detail_level)
        }
        _ => {
            // For NURBS, Revolution, Extrusion: sample curvature
            let max_k = max_curvature_over_domain(surface, u_start, u_end, v_start, v_end);
            if max_k < 1e-10 {
                return MIN_HEIGHT_SAMPLES;
            }
            let r_eq = 1.0 / max_k;
            samples_for_arc_radius(v_range, r_eq, max_deviation, detail_level)
        }
    }
}

/// Compute required samples for a circular arc of given angle and radius.
///
/// Uses the formula: n = ceil(angle / (2 * acos(1 - max_deviation / radius)))
///
/// The `detail_level` parameter scales the result:
/// - detail_level = 1.0 → normal quality
/// - detail_level = 0.5 → half the samples (coarser)
/// - detail_level = 2.0 → double the samples (finer)
fn samples_for_arc_radius(angle: f64, radius: f64, max_deviation: f64, detail_level: f64) -> usize {
    if radius < 1e-10 || angle < 1e-10 {
        return MIN_ANGULAR_SAMPLES;
    }

    let d_over_r = (max_deviation / radius).min(1.0 - 1e-10);
    if d_over_r <= 0.0 {
        return MAX_ANGULAR_SAMPLES;
    }

    let half_angle = (1.0 - d_over_r).acos();
    if half_angle < 1e-10 {
        return MAX_ANGULAR_SAMPLES;
    }

    let n = (angle / (2.0 * half_angle)).ceil() as usize;
    let n = (n as f64 * detail_level).ceil() as usize;
    n.max(MIN_ANGULAR_SAMPLES).min(MAX_ANGULAR_SAMPLES)
}

/// Sample curvature at several points and return the maximum absolute curvature.
fn max_curvature_over_domain(
    surface: &Surface,
    u_start: f64,
    u_end: f64,
    v_start: f64,
    v_end: f64,
) -> f64 {
    let n_sample = 5; // Sample on a 5x5 grid
    let mut max_k = 0.0_f64;

    for i in 0..=n_sample {
        for j in 0..=n_sample {
            let u = u_start + (u_end - u_start) * i as f64 / n_sample as f64;
            let v = v_start + (v_end - v_start) * j as f64 / n_sample as f64;
            let curv = surface.curvature_at(u, v);
            if curv.max_abs.is_finite() {
                max_k = max_k.max(curv.max_abs);
            }
        }
    }

    max_k
}

/// Compute adaptive samples for both u and v directions simultaneously.
///
/// Returns (n_u, n_v) — the number of samples in each parametric direction.
pub fn required_samples(
    surface: &Surface,
    u_start: f64,
    u_end: f64,
    v_start: f64,
    v_end: f64,
    max_deviation: f64,
    detail_level: f64,
) -> (usize, usize) {
    let n_u = required_angular_samples(surface, u_start, u_end, v_start, v_end, max_deviation, detail_level);
    let n_v = required_height_samples(surface, u_start, u_end, v_start, v_end, max_deviation, detail_level);
    (n_u, n_v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::{SphereSurface, CylinderSurface, Plane, Point3d};
    use std::f64::consts::PI;

    #[test]
    fn test_plane_requires_minimum_samples() {
        let plane = Surface::Plane(Plane::xy());
        let (n_u, n_v) = required_samples(&plane, 0.0, 100.0, 0.0, 100.0, 0.01, 1.0);
        assert_eq!(n_u, MIN_ANGULAR_SAMPLES);
        assert_eq!(n_v, MIN_HEIGHT_SAMPLES);
    }

    #[test]
    fn test_sphere_adaptive_samples() {
        let sphere = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 10.0));
        let (n_u, _n_v) = required_samples(&sphere, 0.0, 2.0 * PI, 0.0, PI, 0.1, 1.0);
        // With radius=10 and max_deviation=0.1, we expect ~10 angular samples
        assert!(n_u >= 6, "Sphere should have at least 6 angular samples, got {}", n_u);
        assert!(n_u <= 256, "Sphere should have at most 256 angular samples, got {}", n_u);
    }

    #[test]
    fn test_cylinder_adaptive_samples() {
        let cyl = Surface::Cylinder(CylinderSurface::new_z(5.0));
        let (n_u, n_v) = required_samples(&cyl, 0.0, 2.0 * PI, 0.0, 10.0, 0.1, 1.0);
        // Cylinder has curvature in u-direction (1/r), zero in v-direction
        assert!(n_u >= 6, "Cylinder should have at least 6 angular samples, got {}", n_u);
        assert_eq!(n_v, MIN_HEIGHT_SAMPLES, "Cylinder v-direction should use minimum samples");
    }

    #[test]
    fn test_detail_level_scales_samples() {
        let sphere = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 10.0));
        let (n_u_normal, _) = required_samples(&sphere, 0.0, 2.0 * PI, 0.0, PI, 0.1, 1.0);
        let (n_u_coarse, _) = required_samples(&sphere, 0.0, 2.0 * PI, 0.0, PI, 0.1, 0.5);
        let (n_u_fine, _) = required_samples(&sphere, 0.0, 2.0 * PI, 0.0, PI, 0.1, 2.0);
        assert!(n_u_coarse <= n_u_normal, "Coarse detail should have fewer samples");
        assert!(n_u_fine >= n_u_normal, "Fine detail should have more samples");
    }

    #[test]
    fn test_sphere_area_accuracy() {
        // Triangulate a sphere with adaptive samples and check area accuracy
        // Analytical sphere area = 4 * π * r²
        let radius = 10.0;
        let sphere = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, radius));
        let analytical_area = 4.0 * PI * radius * radius;

        // Use max_deviation = 0.1 for a 10mm radius sphere
        let (n_u, n_v) = required_samples(&sphere, 0.0, 2.0 * PI, 0.0, PI, 0.1, 1.0);

        // Compute triangulated area
        let mut total_area = 0.0;
        for j in 0..n_v {
            let v0 = PI * j as f64 / n_v as f64;
            let v1 = PI * (j + 1) as f64 / n_v as f64;
            for i in 0..n_u {
                let u0 = 2.0 * PI * i as f64 / n_u as f64;
                let u1 = 2.0 * PI * (i + 1) as f64 / n_u as f64;

                let p00 = sphere.point_at(u0, v0);
                let p10 = sphere.point_at(u1, v0);
                let p01 = sphere.point_at(u0, v1);
                let p11 = sphere.point_at(u1, v1);

                // Two triangles per quad
                let tri1_area = triangle_area(&p00, &p10, &p11);
                let tri2_area = triangle_area(&p00, &p11, &p01);
                total_area += tri1_area + tri2_area;
            }
        }

        let error_pct = ((total_area - analytical_area) / analytical_area).abs() * 100.0;
        assert!(error_pct < 5.0,
            "Sphere area error should be < 5%, got {}% (area={}, analytical={})",
            error_pct, total_area, analytical_area);
    }

    fn triangle_area(a: &Point3d, b: &Point3d, c: &Point3d) -> f64 {
        let ab = draper_geometry::Vec3d::new(b.x - a.x, b.y - a.y, b.z - a.z);
        let ac = draper_geometry::Vec3d::new(c.x - a.x, c.y - a.y, c.z - a.z);
        let cross = ab.cross(&ac);
        (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z).sqrt() / 2.0
    }
}
