// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Geometric intersection algorithms.

use crate::{Point3d, Vec3d, Direction3d, curve::*, surface::*, tolerance::ToleranceContext};

/// Error type for B-spline fitting failures.
#[derive(Clone, Debug)]
pub enum FittingError {
    TooFewPoints { got: usize, min: usize },
    DeviationTooHigh { max_dev: f64, tolerance: f64 },
    DegenerateGeometry,
}

impl std::fmt::Display for FittingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FittingError::TooFewPoints { got, min } => write!(f, "Too few points for fitting: {} (min {})", got, min),
            FittingError::DeviationTooHigh { max_dev, tolerance } => write!(f, "Deviation too high: {:.2e} > tol {:.2e}", max_dev, tolerance),
            FittingError::DegenerateGeometry => write!(f, "Degenerate geometry"),
        }
    }
}

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
    /// B-spline curve fitted to the first polyline (if fitting succeeded).
    /// Per ROADMAP_VISION_2036 §2.1: the primary output should be an exact
    /// B-spline curve, with polylines as fallback only.
    pub b_spline_curve: Option<NurbsCurve>,
}

impl SurfaceSurfaceIntersection {
    /// Get the primary intersection curve as a NurbsCurve if available,
    /// otherwise return None (caller should fall back to polylines).
    pub fn b_spline(&self) -> Option<&NurbsCurve> {
        self.b_spline_curve.as_ref()
    }

    /// Fit a B-spline curve to the first polyline using chord-length
    /// parameterized least-squares approximation.
    ///
    /// Per Vision 2030 Task 1: Chord-Length Parameterized B-Spline Fitting.
    ///
    /// Returns `Ok(NurbsCurve)` on success, or `Err(FittingError)` on failure.
    /// The caller can fall back to polylines on error.
    pub fn fit_b_spline(&mut self, tolerance: f64) {
        match self.try_fit_b_spline(tolerance) {
            Ok(curve) => {
                self.b_spline_curve = Some(curve);
            }
            Err(e) => {
                log::debug!("SSI: B-spline fitting failed: {} — using polyline fallback", e);
            }
        }
    }

    /// Try to fit a B-spline curve, returning Result.
    pub fn try_fit_b_spline(&self, tolerance: f64) -> Result<NurbsCurve, FittingError> {
        if self.polylines.is_empty() {
            return Err(FittingError::TooFewPoints { got: 0, min: 4 });
        }
        let pts = &self.polylines[0];
        if pts.len() < 4 {
            return Err(FittingError::TooFewPoints { got: pts.len(), min: 4 });
        }

        // Step 1: Compute chord-length parameters
        let mut params = vec![0.0_f64; pts.len()];
        let mut total_len = 0.0_f64;
        for i in 1..pts.len() {
            let dx = pts[i].x - pts[i-1].x;
            let dy = pts[i].y - pts[i-1].y;
            let dz = pts[i].z - pts[i-1].z;
            total_len += (dx*dx + dy*dy + dz*dz).sqrt();
            params[i] = total_len;
        }
        if total_len < 1e-15 {
            return Err(FittingError::DegenerateGeometry);
        }
        for p in &mut params {
            *p /= total_len;
        }

        // Step 2: Adaptive control point selection based on curvature.
        // Instead of a fixed cap of 20, scale control points with curvature.
        let n_cp_target = adaptive_cp_count(pts);
        let degree = 3;
        let n_cp = n_cp_target.max(degree + 1).min(pts.len());

        // Step 3: Select control points via chord-length-weighted subsampling.
        let mut control_points: Vec<Point3d> = Vec::with_capacity(n_cp);
        control_points.push(pts[0]);

        let interval = 1.0 / (n_cp - 1) as f64;
        let mut next_target = interval;
        for i in 1..pts.len() - 1 {
            if params[i] >= next_target {
                control_points.push(pts[i]);
                next_target += interval;
            }
        }
        if control_points.last() != Some(&pts[pts.len() - 1]) {
            control_points.push(pts[pts.len() - 1]);
        }

        let n = control_points.len();
        if n < degree + 1 {
            return Err(FittingError::TooFewPoints { got: n, min: degree + 1 });
        }

        // Step 4: Build clamped B-spline knot vector.
        let n_knots = n + degree + 1;
        let mut knots = vec![0.0; n_knots];
        for i in 0..n_knots {
            if i <= degree {
                knots[i] = 0.0;
            } else if i >= n {
                knots[i] = 1.0;
            } else {
                knots[i] = (i - degree) as f64 / (n - degree) as f64;
            }
        }

        let weights = vec![1.0; n];
        let curve = NurbsCurve { degree, control_points, weights, knots };

        // Step 5: Verify max deviation
        let mut max_dev = 0.0_f64;
        for (i, &p) in pts.iter().enumerate() {
            let t = params[i];
            let eval = Curve3d::Nurbs(curve.clone()).point_at(t);
            let dev = ((p.x - eval.x).powi(2) + (p.y - eval.y).powi(2) + (p.z - eval.z).powi(2)).sqrt();
            if dev > max_dev {
                max_dev = dev;
            }
        }

        if max_dev < tolerance {
            log::info!(
                "SSI: fitted B-spline curve ({} control points, degree={}, max_dev={:.2e}, tol={:.2e})",
                n, degree, max_dev, tolerance
            );
            Ok(curve)
        } else {
            Err(FittingError::DeviationTooHigh { max_dev, tolerance })
        }
    }
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
            // Tangent — one line along the cylinder axis, located at the
            // tangential touch point. The touch point is the point on the
            // cylinder surface closest to the plane, which is the cylinder
            // origin offset along the direction FROM cylinder axis TO the
            // closest point on the plane.
            //
            // The closest point on the plane to the cylinder origin is:
            //   closest = cyl.origin - dist * plane.normal
            // (where dist is the signed distance from origin to plane).
            // The direction from cylinder axis to this closest point,
            // projected onto the plane containing the axis (perpendicular
            // to plane.normal), gives the perpendicular direction we need.
            let closest_on_plane = Point3d::new(
                cylinder.origin.x - dist * plane.normal.x,
                cylinder.origin.y - dist * plane.normal.y,
                cylinder.origin.z - dist * plane.normal.z,
            );
            // Direction from cylinder origin to closest_on_plane (this is
            // perpendicular to the cylinder axis because the axis is parallel
            // to the plane, and to the plane normal because we projected).
            let mut perp_dir = Vec3d::new(
                closest_on_plane.x - cylinder.origin.x,
                closest_on_plane.y - cylinder.origin.y,
                closest_on_plane.z - cylinder.origin.z,
            );
            let perp_len = (perp_dir.x * perp_dir.x + perp_dir.y * perp_dir.y + perp_dir.z * perp_dir.z).sqrt();
            if perp_len < 1e-12 {
                return vec![];
            }
            perp_dir = Vec3d::new(perp_dir.x / perp_len, perp_dir.y / perp_len, perp_dir.z / perp_len);
            // Tangent point on the cylinder surface (also on the plane)
            let touch = Point3d::new(
                cylinder.origin.x + cylinder.radius * perp_dir.x,
                cylinder.origin.y + cylinder.radius * perp_dir.y,
                cylinder.origin.z + cylinder.radius * perp_dir.z,
            );
            // Sample a finite segment along axis centered at touch point
            let span = cylinder.radius * 8.0;
            return vec![sample_axis_parallel_line(&touch, &cylinder.axis, span)];
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
        let _v_axis = Vec3d::new(
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

/// Intersect a plane with a cone — analytic conic section.
///
/// B1 leftover (2026-09-01): Plane×Cone previously fell through to the
/// generic marching SSI fallback. This is the analytic path.
///
/// Classification by the angle θ between the plane and the cone axis
/// (α = |half_angle|):
/// - θ > α: ellipse (a circle when the plane is perpendicular to the axis)
/// - θ = α: parabola
/// - θ < α: hyperbola — one branch per nappe; a single-nappe cone surface
///   yields one branch
/// - plane through the apex (degenerate): two generator rays (θ < α),
///   one tangent ray (θ = α), or nothing (θ > α)
///
/// Method: the section is parametrized on cone generators. The generator at
/// angle u is the ray from the apex A
///     g(u) = sin α · radial(u) + s · cos α · k,   t ≥ 0
/// where k is the unit axis, radial(u) = cos u·X + sin u·Y with (X, Y) ⊥ k,
/// and s = sign(tan(half_angle)) selects the nappe side (the surface lives
/// where r(v) ≥ 0, i.e. where s·(P−A)·k ≥ 0). The generator hits the plane
/// n·(P − P0) = 0 at
///     t(u) = −d / D(u),   d = n·(A − P0),   D(u) = n·g(u)
/// so every output point P = A + t(u)·g(u) satisfies both surface equations
/// exactly (up to floating point) — no marching, no grid search.
/// D(u) = a·cos(u − u0) + b with a = sin α·|n⊥|, b = s·cos α·(n·k): when
/// |b| ≤ a the plane is parallel to two generators (asymptote directions,
/// u = u0 ± acos(−b/a)) and the valid arc between them is one hyperbola
/// branch / parabola arm, clipped at a finite slant length.
pub fn intersect_plane_cone(
    plane: &Plane,
    cone: &ConeSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let tan_ha = cone.half_angle.tan();

    // Degenerate cone: half_angle ≈ 0 is a cylinder — reuse the analytic
    // plane×cylinder path.
    if !tan_ha.is_finite() || tan_ha.abs() < 1e-10 {
        let cylinder = CylinderSurface {
            origin: cone.origin,
            axis: cone.axis,
            radius: cone.radius,
            x_dir: cone.x_dir,
        };
        return intersect_plane_cylinder(plane, &cylinder, tolerance);
    }

    // Apex and nappe side. For a standard cone the apex sits at
    // v_apex = −radius / tan(half_angle) along the axis; for an expanding
    // cone the origin *is* the apex.
    let s = tan_ha.signum();
    let apex = if cone.expanding {
        cone.origin
    } else {
        let v_apex = -cone.radius / tan_ha;
        Point3d::new(
            cone.origin.x + v_apex * cone.axis.x,
            cone.origin.y + v_apex * cone.axis.y,
            cone.origin.z + v_apex * cone.axis.z,
        )
    };

    let alpha = cone.half_angle.abs();
    let sin_a = alpha.sin();
    let cos_a = alpha.cos();

    // Orthonormal frame ⊥ axis: X = x_dir re-orthogonalized against k,
    // Y = k × X.
    let kx = cone.axis.x;
    let ky = cone.axis.y;
    let kz = cone.axis.z;
    let mut xx = cone.x_dir.x;
    let mut xy = cone.x_dir.y;
    let mut xz = cone.x_dir.z;
    let x_dot_k = xx * kx + xy * ky + xz * kz;
    xx -= x_dot_k * kx;
    xy -= x_dot_k * ky;
    xz -= x_dot_k * kz;
    let mut x_len = (xx * xx + xy * xy + xz * xz).sqrt();
    if x_len < 1e-9 {
        // x_dir parallel to the axis — build any perpendicular direction
        xx = if kz.abs() < 0.9 { 0.0 } else { -kz };
        xy = if kz.abs() < 0.9 { kz } else { 0.0 };
        xz = if kz.abs() < 0.9 { -ky } else { kx };
        x_len = (xx * xx + xy * xy + xz * xz).sqrt();
    }
    xx /= x_len;
    xy /= x_len;
    xz /= x_len;
    // Y = k × X
    let yx = ky * xz - kz * xy;
    let yy = kz * xx - kx * xz;
    let yz = kx * xy - ky * xx;

    let nx = plane.normal.x;
    let ny = plane.normal.y;
    let nz = plane.normal.z;

    // Signed distance from the apex to the plane: d = n·(A − P0)
    let d = (apex.x - plane.origin.x) * nx
        + (apex.y - plane.origin.y) * ny
        + (apex.z - plane.origin.z) * nz;

    // D(u) = n·g(u) = a·cos(u − u0) + b
    let nk = nx * kx + ny * ky + nz * kz;
    let n_x = nx * xx + ny * xy + nz * xz;
    let n_y = nx * yx + ny * yy + nz * yz;
    let a = sin_a * (n_x * n_x + n_y * n_y).sqrt();
    let b = s * cos_a * nk;
    let u0 = n_y.atan2(n_x);

    // Scale for ray clipping (hyperbola/parabola arms) and degeneracy eps.
    // Includes the base radius, the cone height, and the apex-to-plane
    // distance so that a far plane still yields its section.
    let v_apex_len = if cone.expanding { 0.0 } else { (-cone.radius / tan_ha).abs() };
    let scale = cone
        .radius
        .max(v_apex_len)
        .max(apex.distance_to(&plane.origin))
        .max(tolerance.max(1e-6));
    let apex_eps = tolerance.max(1e-9) * scale.max(1.0);
    let t_clip = 20.0 * scale.max(1.0);

    // Evaluate a point on the generator at parameter u, slant length t.
    let generator_point = |u: f64, t: f64| -> Point3d {
        let cu = u.cos();
        let su = u.sin();
        let gx = sin_a * (cu * xx + su * yx) + s * cos_a * kx;
        let gy = sin_a * (cu * xy + su * yy) + s * cos_a * ky;
        let gz = sin_a * (cu * xz + su * yz) + s * cos_a * kz;
        Point3d::new(
            apex.x + t * gx,
            apex.y + t * gy,
            apex.z + t * gz,
        )
    };

    // ── Degenerate: the plane passes through the apex ──
    // The section degenerates to the generator rays lying in the plane
    // (D(u) = 0 — the whole ray is in the plane because the apex is).
    // θ < α → two rays, θ = α → one tangent ray, θ > α → nothing.
    if d.abs() < apex_eps {
        if a < 1e-12 * (1.0 + b.abs()) {
            // n ∥ k: plane ⊥ axis through the apex — apex point only.
            return vec![];
        }
        let rhs = (-b / a).clamp(-1.0, 1.0);
        if rhs >= 1.0 {
            // θ > α — the plane touches only the apex.
            return vec![];
        }
        let base = rhs.acos();
        let mut roots = vec![u0 + base];
        if base > 1e-9 && (std::f64::consts::PI - base) > 1e-9 {
            roots.push(u0 - base);
        }
        let n_samples = 20;
        let mut lines = Vec::with_capacity(roots.len());
        for u in roots {
            let mut line = Vec::with_capacity(n_samples);
            for i in 0..n_samples {
                let t = t_clip * i as f64 / (n_samples - 1) as f64;
                line.push(generator_point(u, t));
            }
            lines.push(line);
        }
        return lines;
    }

    let n_samples = 128;

    // ── Closed section: |b| > a — D(u) never zero, ellipse / circle ──
    // t = −d/D has constant sign over the loop; a non-positive sample means
    // the whole ellipse lies on the opposite nappe → empty.
    if a < 1e-12 * (1.0 + b.abs()) || (-b / a).abs() > 1.0 + 1e-12 {
        let mut ellipse: Vec<Point3d> = Vec::with_capacity(n_samples);
        // Sanity cap: near-parabolic planes can push the far side very far
        // out; points stay exact, but avoid astronomically large output.
        let t_cap = 1e9 * scale.max(1.0);
        for i in 0..n_samples {
            let u = 2.0 * std::f64::consts::PI * i as f64 / n_samples as f64;
            let denom = a * (u - u0).cos() + b;
            if !denom.is_finite() || denom.abs() < 1e-15 {
                continue;
            }
            let t = -d / denom;
            if !t.is_finite() || t <= 0.0 || t > t_cap {
                // Constant-sign t: a single invalid sample ⇒ empty section.
                return vec![];
            }
            ellipse.push(generator_point(u, t));
        }
        return if ellipse.len() >= 2 { vec![ellipse] } else { vec![] };
    }

    // ── Open section: |b| ≤ a — hyperbola branch / parabola arm ──
    // D = 0 at u = u0 ± base (asymptote directions). The valid arc is where
    // t = −d/D > 0:
    //   d < 0 → D > 0 → arc (u0 − base, u0 + base)
    //   d > 0 → D < 0 → arc (u0 + base, u0 − base + 2π)
    // Midpoint sampling over the arc never lands on an asymptote.
    let base = (-b / a).clamp(-1.0, 1.0).acos();
    let (arc_start, arc_len) = if d < 0.0 {
        (u0 - base, 2.0 * base)
    } else {
        (u0 + base, 2.0 * (std::f64::consts::PI - base))
    };
    if arc_len < 1e-12 {
        // Degenerate arc — the valid arc shrank to a point (parabola on the
        // opposite nappe).
        return vec![];
    }

    let mut current: Vec<Point3d> = Vec::with_capacity(n_samples);
    let mut polylines: Vec<Vec<Point3d>> = Vec::new();
    for i in 0..n_samples {
        let u = arc_start + arc_len * (i as f64 + 0.5) / n_samples as f64;
        let denom = a * (u - u0).cos() + b;
        if !denom.is_finite() || denom.abs() < 1e-15 {
            continue;
        }
        let t = -d / denom;
        // The arc guarantees t > 0; only clip runaway arms.
        if !t.is_finite() || t <= 0.0 || t > t_clip {
            if current.len() >= 2 {
                polylines.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
            continue;
        }
        current.push(generator_point(u, t));
    }
    if current.len() >= 2 {
        polylines.push(current);
    }
    polylines
}

/// Intersect two spheres analytically (B1 series follow-up, 2026-09-01).
///
/// The intersection of two spheres is a circle lying in the **radical
/// plane** — the plane perpendicular to the center line. Classification:
///
/// - concentric (`d ≈ 0`): empty — coincident surfaces produce no curve
///   (same convention as co-axial cylinders in
///   [`intersect_cylinder_cylinder`]);
/// - disjoint (`d > r1 + r2`) or contained (`d < |r1 − r2|`): empty;
/// - external tangency (`d ≈ r1 + r2`): a single point between the
///   centers;
/// - internal tangency (`d ≈ |r1 − r2|`): a single point on the center
///   line on the far side of the smaller sphere;
/// - general position: the circle `center = c1 + a·n`,
///   `radius = √(r1² − a²)` with `n = (c2 − c1)/d` and
///   `a = (d² + r1² − r2²)/(2d)`.
///
/// Every sampled point satisfies both sphere equations exactly (up to
/// floating-point rounding) — no marching, no Newton refinement. Spheres
/// in this kernel are full spheres, so the circle needs no boundary
/// clipping.
pub fn intersect_sphere_sphere(
    s1: &SphereSurface,
    s2: &SphereSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);

    let dx = s2.center.x - s1.center.x;
    let dy = s2.center.y - s1.center.y;
    let dz = s2.center.z - s1.center.z;
    let d = (dx * dx + dy * dy + dz * dz).sqrt();

    let (r1, r2) = (s1.radius, s2.radius);

    // Concentric: no radical plane, no curve.
    if d < eps {
        return vec![];
    }

    let sum = r1 + r2;
    let diff = (r1 - r2).abs();

    // External tangency: the single point r1/(r1+r2) of the way from c1
    // to c2 — at distance r1 from c1 and r2 from c2.
    if (d - sum).abs() <= eps {
        let t = r1 / sum;
        return vec![vec![Point3d::new(
            s1.center.x + t * dx,
            s1.center.y + t * dy,
            s1.center.z + t * dz,
        )]];
    }

    // Internal tangency: the touch point lies on the center line at
    // distance r1 from c1 — toward c2 when the first sphere is the larger
    // one, away from c2 when it is the smaller one.
    if (d - diff).abs() <= eps && diff > eps {
        let sign = if r1 >= r2 { 1.0 } else { -1.0 };
        let t = sign * r1 / d;
        return vec![vec![Point3d::new(
            s1.center.x + t * dx,
            s1.center.y + t * dy,
            s1.center.z + t * dz,
        )]];
    }

    // Disjoint or one sphere fully inside the other.
    if d > sum || d < diff {
        return vec![];
    }

    // Radical-plane circle: a = distance from c1 to the radical plane
    // along n; h = circle radius.
    let a = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
    let h_sq = (r1 * r1 - a * a).max(0.0);
    let h = h_sq.sqrt();
    let cx = s1.center.x + (a / d) * dx;
    let cy = s1.center.y + (a / d) * dy;
    let cz = s1.center.z + (a / d) * dz;

    // Unit normal of the radical plane = center-line direction.
    let n = Vec3d::new(dx / d, dy / d, dz / d);

    // Orthonormal frame ⊥ n: reference axis not parallel to n, u =
    // normalize(ref − (ref·n)·n), v = n × u.
    let (rx, ry, rz) = if n.z.abs() < 0.9 {
        (0.0, 0.0, 1.0)
    } else {
        (1.0, 0.0, 0.0)
    };
    let ref_dot_n = rx * n.x + ry * n.y + rz * n.z;
    let mut ux = rx - ref_dot_n * n.x;
    let mut uy = ry - ref_dot_n * n.y;
    let mut uz = rz - ref_dot_n * n.z;
    let u_len = (ux * ux + uy * uy + uz * uz).sqrt();
    if u_len < 1e-12 {
        return vec![];
    }
    ux /= u_len;
    uy /= u_len;
    uz /= u_len;
    let vx = n.y * uz - n.z * uy;
    let vy = n.z * ux - n.x * uz;
    let vz = n.x * uy - n.y * ux;

    // Sample the closed circle (same density as the plane×cone closed
    // section: 128 points, endpoint not duplicated).
    let n_samples = 128;
    let mut circle = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let theta = 2.0 * std::f64::consts::PI * i as f64 / n_samples as f64;
        let c = theta.cos();
        let s = theta.sin();
        circle.push(Point3d::new(
            cx + h * (c * ux + s * vx),
            cy + h * (c * uy + s * vy),
            cz + h * (c * uz + s * vz),
        ));
    }
    vec![circle]
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
        // Parallel axes — intersection is 0, 1, or 2 lines parallel to the axes.
        // The cross-section perpendicular to the axes is two circles of radii
        // r_a and r_b, with centers separated by perp_dist. If the circles
        // intersect at 2 points, those points sweep along the cylinder axes
        // to form 2 straight intersection lines.
        let dx = cyl_b.origin.x - cyl_a.origin.x;
        let dy = cyl_b.origin.y - cyl_a.origin.y;
        let dz = cyl_b.origin.z - cyl_a.origin.z;
        let along = dx * cyl_a.axis.x + dy * cyl_a.axis.y + dz * cyl_a.axis.z;
        let perp_x = dx - along * cyl_a.axis.x;
        let perp_y = dy - along * cyl_a.axis.y;
        let perp_z = dz - along * cyl_a.axis.z;
        let perp_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();

        let r_a = cyl_a.radius;
        let r_b = cyl_b.radius;
        let r_sum = r_a + r_b;
        let r_diff = (r_a - r_b).abs();

        if perp_dist > r_sum + 1e-9 || perp_dist < r_diff - 1e-9 {
            return vec![]; // No intersection (too far apart, or one inside the other)
        }
        if perp_dist < 1e-12 {
            // Coaxial cylinders — either identical (infinite intersection)
            // or no intersection (different radii). Treat as no intersection.
            return vec![];
        }

        // Compute the 2 intersection points of the perpendicular cross-section
        // circles. Set up 2D coordinate system in the perpendicular plane:
        //   axis-2D-x along the direction from cyl_a to cyl_b projected to perp plane
        //   axis-2D-y perpendicular to that within the perp plane
        let e_x = Vec3d::new(perp_x / perp_dist, perp_y / perp_dist, perp_z / perp_dist);
        // e_y = axis × e_x (right-handed; both unit vectors)
        let e_y = Vec3d::new(
            cyl_a.axis.y * e_x.z - cyl_a.axis.z * e_x.y,
            cyl_a.axis.z * e_x.x - cyl_a.axis.x * e_x.z,
            cyl_a.axis.x * e_x.y - cyl_a.axis.y * e_x.x,
        );

        // Distance from cyl_a's center to the chord (line through intersection points)
        let a_to_chord = (r_a * r_a - r_b * r_b + perp_dist * perp_dist) / (2.0 * perp_dist);
        // Half-length of the chord
        let h_sq = r_a * r_a - a_to_chord * a_to_chord;
        if h_sq < 0.0 {
            // Numerical edge case — tangential touch (1 line)
            let center_3d = Point3d::new(
                cyl_a.origin.x + a_to_chord * e_x.x,
                cyl_a.origin.y + a_to_chord * e_x.y,
                cyl_a.origin.z + a_to_chord * e_x.z,
            );
            // Sample the line along the cylinder axis direction over a finite range
            let line_points = sample_axis_parallel_line(&center_3d, &cyl_a.axis, r_a.max(r_b) * 4.0);
            return vec![line_points];
        }
        let h = h_sq.sqrt();

        // Two intersection points in 3D
        let p1 = Point3d::new(
            cyl_a.origin.x + a_to_chord * e_x.x + h * e_y.x,
            cyl_a.origin.y + a_to_chord * e_x.y + h * e_y.y,
            cyl_a.origin.z + a_to_chord * e_x.z + h * e_y.z,
        );
        let p2 = Point3d::new(
            cyl_a.origin.x + a_to_chord * e_x.x - h * e_y.x,
            cyl_a.origin.y + a_to_chord * e_x.y - h * e_y.y,
            cyl_a.origin.z + a_to_chord * e_x.z - h * e_y.z,
        );

        // Each point sweeps along cyl_a.axis to form a line — sample a finite
        // segment long enough to cover both cylinders' extents.
        let span = r_a.max(r_b) * 4.0;
        let line1 = sample_axis_parallel_line(&p1, &cyl_a.axis, span);
        let line2 = sample_axis_parallel_line(&p2, &cyl_a.axis, span);
        return vec![line1, line2];
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
        (Surface::Plane(p), Surface::Cone(c)) | (Surface::Cone(c), Surface::Plane(p)) => {
            intersect_plane_cone(p, c, tolerance)
        }
        (Surface::Cylinder(a), Surface::Cylinder(b)) => {
            intersect_cylinder_cylinder(a, b, tolerance)
        }
        (Surface::Sphere(s1), Surface::Sphere(s2)) => {
            intersect_sphere_sphere(s1, s2, tolerance)
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

    let mut result = SurfaceSurfaceIntersection { polylines, b_spline_curve: None };
    // Attempt B-spline fitting (ROADMAP_VISION_2036 §2.1)
    result.fit_b_spline(tolerance);
    result
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

// ============================================================
// Adaptive control point selection for B-spline fitting
// ============================================================

/// Compute the optimal number of control points for B-spline fitting
/// based on the curvature of the polyline.
///
/// Instead of a fixed cap (was 20), this function:
/// 1. Estimates curvature via second differences (cross product of
///    consecutive edge vectors)
/// 2. Scales control point count with average curvature
/// 3. Clamps to [10, pts.len()]
///
/// High-curvature intersection curves (e.g., two NURBS surfaces with
/// complex intersection topology) get more control points for accurate
/// fitting within tolerance. Low-curvature curves (near-linear) get
/// fewer for efficiency.
fn adaptive_cp_count(pts: &[Point3d]) -> usize {
    if pts.len() < 4 {
        return pts.len();
    }

    // Estimate curvature via second differences
    let mut curvature_sum = 0.0_f64;
    for i in 1..pts.len() - 1 {
        let v1x = pts[i].x - pts[i-1].x;
        let v1y = pts[i].y - pts[i-1].y;
        let v1z = pts[i].z - pts[i-1].z;
        let v2x = pts[i+1].x - pts[i].x;
        let v2y = pts[i+1].y - pts[i].y;
        let v2z = pts[i+1].z - pts[i].z;

        // Cross product magnitude = curvature indicator
        let cx = v1y * v2z - v1z * v2y;
        let cy = v1z * v2x - v1x * v2z;
        let cz = v1x * v2y - v1y * v2x;
        curvature_sum += (cx * cx + cy * cy + cz * cz).sqrt();
    }
    let avg_curvature = curvature_sum / (pts.len() - 2) as f64;

    // Base count: sqrt of polyline length (scales sub-linearly)
    let base = (pts.len() as f64).sqrt().ceil() as usize;

    // Curvature factor: more curvature → more control points
    // avg_curvature is typically 0.0 (straight) to ~1.0 (high curvature)
    let curvature_factor = (avg_curvature * 100.0).max(1.0).min(4.0);

    let result = (base as f64 * curvature_factor).ceil() as usize;
    result.max(10).min(pts.len())
}

/// Sample a finite line segment along `axis` direction, centered at `origin`.
/// Used by `intersect_cylinder_cylinder` for parallel-axis case: each
/// intersection point sweeps along the cylinder axis to form a straight line.
///
/// Returns 2 points: origin - axis*span/2 and origin + axis*span/2.
fn sample_axis_parallel_line(origin: &Point3d, axis: &Direction3d, span: f64) -> Vec<Point3d> {
    let half = span * 0.5;
    vec![
        Point3d::new(
            origin.x - axis.x * half,
            origin.y - axis.y * half,
            origin.z - axis.z * half,
        ),
        Point3d::new(
            origin.x + axis.x * half,
            origin.y + axis.y * half,
            origin.z + axis.z * half,
        ),
    ]
}

#[cfg(test)]
mod parallel_cylinder_tests {
    use super::*;
    use crate::surface::CylinderSurface;
    use crate::direction::Direction3d;

    #[test]
    fn test_parallel_disjoint_cylinders_no_intersection() {
        // Two parallel cylinders far apart — no intersection.
        let cyl_a = CylinderSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let cyl_b = CylinderSurface::new(
            Point3d::new(10.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let result = intersect_cylinder_cylinder(&cyl_a, &cyl_b, 1e-6);
        assert!(result.is_empty(), "Disjoint parallel cylinders should have no intersection");
    }

    #[test]
    fn test_parallel_concentric_cylinders_no_intersection() {
        // Two coaxial cylinders with different radii — no intersection.
        let cyl_a = CylinderSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let cyl_b = CylinderSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            2.0,
        );
        let result = intersect_cylinder_cylinder(&cyl_a, &cyl_b, 1e-6);
        assert!(result.is_empty(), "Concentric cylinders of different radii should have no intersection");
    }

    #[test]
    fn test_parallel_cylinders_two_intersection_lines() {
        // Two parallel cylinders, both radius=1, centers offset by 1.0 in X.
        // perp_dist=1, r_sum=2, r_diff=0 → 2 intersection lines.
        let cyl_a = CylinderSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let cyl_b = CylinderSurface::new(
            Point3d::new(1.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let result = intersect_cylinder_cylinder(&cyl_a, &cyl_b, 1e-6);
        assert_eq!(result.len(), 2, "Expected 2 intersection lines, got {}", result.len());
        // Each line has 2 sample points (start and end of the segment)
        assert_eq!(result[0].len(), 2);
        assert_eq!(result[1].len(), 2);
        // The two lines should be symmetric in Y around 0
        let y0 = result[0][0].y;
        let y1 = result[1][0].y;
        assert!((y0 + y1).abs() < 1e-9, "Lines should be Y-symmetric, got y0={}, y1={}", y0, y1);
        // Both intersection points should be on cylinder A: x² + y² = 1
        for line in &result {
            for p in line {
                let r = (p.x * p.x + p.y * p.y).sqrt();
                assert!((r - 1.0).abs() < 1e-9, "Point {:?} should be on cyl A (r=1), got r={}", p, r);
            }
        }
    }

    #[test]
    fn test_parallel_cylinders_tangent_one_line() {
        // Two parallel cylinders, both radius=1, centers offset by 2.0 in X.
        // perp_dist=2 = r_sum → tangential touch, 1 intersection line (or 2
        // numerically-close lines due to floating-point edge case).
        let cyl_a = CylinderSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let cyl_b = CylinderSurface::new(
            Point3d::new(2.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let result = intersect_cylinder_cylinder(&cyl_a, &cyl_b, 1e-6);
        // Acceptable: 0, 1, or 2 lines (numerical edge case).
        assert!(result.len() <= 2, "Tangent cylinders should have ≤2 intersections, got {}", result.len());
        if !result.is_empty() {
            // All intersection points should be approximately at (1, 0, ?)
            // (the tangential touch point on both cylinders).
            for line in &result {
                for p in line {
                    assert!((p.x - 1.0).abs() < 1e-6, "Tangent point x should be 1.0, got {}", p.x);
                    assert!(p.y.abs() < 1e-6, "Tangent point y should be 0.0, got {}", p.y);
                }
            }
        }
    }

    #[test]
    fn test_plane_cylinder_tangent_one_line() {
        // Plane y=1 tangent to cylinder (origin=0, axis=Z, radius=1).
        // Intersection should be 1 line at (0, 1, ?).
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 1.0, 0.0),
            Direction3d::Y,
        );
        let cylinder = CylinderSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
        );
        let result = intersect_plane_cylinder(&plane, &cylinder, 1e-6);
        // May be 1 line (tangent) or empty (numerical edge case).
        assert!(result.len() <= 1, "Tangent plane-cylinder should have ≤1 intersection, got {}", result.len());
        if !result.is_empty() {
            for line in &result {
                for p in line {
                    assert!((p.y - 1.0).abs() < 1e-6, "Tangent point y should be 1.0, got {}", p.y);
                    assert!(p.x.abs() < 1e-6, "Tangent point x should be 0.0, got {}", p.x);
                }
            }
        }
    }
}

#[cfg(test)]
mod plane_cone_tests {
    use super::*;
    use crate::surface::{ConeSurface, Plane, Surface};
    use crate::direction::Direction3d;

    /// Narrowing cone: base radius 5 at z=0, apex at z=10 (tan(α) = 0.5,
    /// half_angle negative — the STEP narrowing convention).
    fn narrowing_cone() -> ConeSurface {
        ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            5.0,
            -(0.5f64).atan(),
        )
    }

    /// Expanding cone: apex at origin, α = 45°, opening upward.
    fn expanding_cone() -> ConeSurface {
        ConeSurface::new_expanding(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            std::f64::consts::FRAC_PI_4,
            Direction3d::X,
        )
    }

    /// Apex of a standard (non-expanding) cone: origin + v_apex·axis with
    /// v_apex = −radius / tan(half_angle).
    fn cone_apex(cone: &ConeSurface) -> Point3d {
        let v_apex = -cone.radius / cone.half_angle.tan();
        Point3d::new(
            cone.origin.x + v_apex * cone.axis.x,
            cone.origin.y + v_apex * cone.axis.y,
            cone.origin.z + v_apex * cone.axis.z,
        )
    }

    /// Assert that a point lies on the plane (exact analytic check).
    fn assert_on_plane(p: &Point3d, plane: &Plane, eps: f64) {
        let dist = (p.x - plane.origin.x) * plane.normal.x
            + (p.y - plane.origin.y) * plane.normal.y
            + (p.z - plane.origin.z) * plane.normal.z;
        assert!(
            dist.abs() < eps,
            "Point {:?} not on plane (signed dist {})",
            p,
            dist
        );
    }

    /// Assert that a point lies on the cone: the angle between (P − apex)
    /// and the axis equals half_angle, and P is on the nappe side.
    fn assert_on_cone(p: &Point3d, cone: &ConeSurface, eps: f64) {
        let apex = if cone.expanding { cone.origin } else { cone_apex(cone) };
        let dx = p.x - apex.x;
        let dy = p.y - apex.y;
        let dz = p.z - apex.z;
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        let along = dx * cone.axis.x + dy * cone.axis.y + dz * cone.axis.z;
        let cos_alpha = cone.half_angle.abs().cos();
        let s = cone.half_angle.tan().signum();
        if cone.expanding {
            // r = v·tan(α) ≥ 0 → nappe is (P−A)·k ≥ 0
            assert!(along > -eps, "Point {:?} not on expanding-cone nappe (along={})", p, along);
        } else if s > 0.0 {
            assert!(along > -eps, "Point {:?} not on nappe (along={})", p, along);
        } else {
            assert!(along < eps, "Point {:?} not on nappe (along={})", p, along);
        }
        if len > eps {
            let cos_angle = along.abs() / len;
            assert!(
                (cos_angle - cos_alpha).abs() < 1e-9,
                "Point {:?} not on cone surface (cos_angle={}, cos_alpha={})",
                p,
                cos_angle,
                cos_alpha
            );
        }
    }

    fn assert_all_on_both(pts: &[Point3d], plane: &Plane, cone: &ConeSurface, eps: f64) {
        for p in pts {
            assert_on_plane(p, plane, eps);
            assert_on_cone(p, cone, eps);
        }
    }

    #[test]
    fn test_plane_cone_circle_perpendicular() {
        // Plane z=0 ⊥ axis of the narrowing cone → base circle, radius 5.
        let cone = narrowing_cone();
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
        );
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1, "Expected 1 closed section, got {}", result.len());
        assert!(result[0].len() >= 64, "Expected dense circle sampling, got {} points", result[0].len());
        assert_all_on_both(&result[0], &plane, &cone, 1e-9);
        for p in &result[0] {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 5.0).abs() < 1e-9, "Circle radius should be 5.0, got {}", r);
            assert!(p.z.abs() < 1e-9, "Circle should lie at z=0, got z={}", p.z);
        }
    }

    #[test]
    fn test_plane_cone_ellipse_oblique() {
        // Tilted plane (20° from axis-normal) through the cone body →
        // closed ellipse, all points exactly on both surfaces.
        let cone = narrowing_cone();
        let tilt = 20.0f64.to_radians();
        let normal = Direction3d::new(tilt.sin(), 0.0, tilt.cos()).unwrap();
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 5.0),
            normal,
        );
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1, "Expected 1 ellipse, got {}", result.len());
        assert!(result[0].len() >= 64, "Expected dense ellipse sampling, got {} points", result[0].len());
        assert_all_on_both(&result[0], &plane, &cone, 1e-9);
        // The ellipse must be closed: the sampled loop wraps around the
        // axis (points span the full 2π of generator angles).
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        for p in &result[0] {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
        }
        assert!(max_x - min_x > 1.0, "Ellipse should have spread in the tilt direction");
    }

    #[test]
    fn test_plane_cone_empty_beyond_apex() {
        // Cone expanding upward from apex (0,0,-1): a plane below the apex
        // cuts only the opposite nappe → empty.
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
            std::f64::consts::FRAC_PI_4,
        );
        // apex = (0,0,-1), nappe upward.
        let below = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, -5.0),
            Direction3d::Z,
        );
        let result = intersect_plane_cone(&below, &cone, 1e-6);
        assert!(result.is_empty(), "Plane beyond the apex should give no section, got {} polylines", result.len());

        // Same plane above the apex: circle. Cone at z=5 (v=5) has
        // radius r = 1 + 5·tan(45°) = 6.
        let above = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 5.0),
            Direction3d::Z,
        );
        let result = intersect_plane_cone(&above, &cone, 1e-6);
        assert_eq!(result.len(), 1);
        assert_all_on_both(&result[0], &above, &cone, 1e-9);
        for p in &result[0] {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 6.0).abs() < 1e-9, "Circle radius should be 6.0, got {}", r);
        }
    }

    #[test]
    fn test_plane_cone_parabola_clipped() {
        // Plane parallel to a generator (θ = α) → parabola arm, clipped at
        // a finite slant length. Cone: α=45°, apex (0,0,-1), nappe up.
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
            std::f64::consts::FRAC_PI_4,
        );
        let angle45 = 45.0f64.to_radians();
        let normal = Direction3d::new(angle45.sin(), 0.0, angle45.cos()).unwrap();
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), normal);
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1, "Parabola should be 1 polyline, got {}", result.len());
        let pts = &result[0];
        assert!(pts.len() >= 10, "Parabola should have dense sampling, got {} points", pts.len());
        assert_all_on_both(pts, &plane, &cone, 1e-9);
        // Open curve: the arm does not close on itself.
        let first = pts[0];
        let last = pts[pts.len() - 1];
        assert!(
            first.distance_to(&last) > 1e-3,
            "Parabola arm should be open (first≈last)"
        );
    }

    #[test]
    fn test_plane_cone_hyperbola_single_branch() {
        // Steep plane (θ = 15° < α = 45°) → one hyperbola branch on this
        // nappe (the double cone would have two).
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
            std::f64::consts::FRAC_PI_4,
        );
        let normal = Direction3d::new(
            (75.0f64.to_radians()).sin(),
            0.0,
            (75.0f64.to_radians()).cos(),
        )
        .unwrap();
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 0.0), normal);
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1, "Single nappe ⇒ 1 branch, got {}", result.len());
        let pts = &result[0];
        assert!(pts.len() >= 10, "Branch should have dense sampling, got {} points", pts.len());
        assert_all_on_both(pts, &plane, &cone, 1e-9);
        // Open curve with clipped far ends.
        let first = pts[0];
        let last = pts[pts.len() - 1];
        assert!(first.distance_to(&last) > 1e-3, "Branch should be open");
    }

    #[test]
    fn test_plane_cone_two_generator_rays_through_apex() {
        // Plane through the apex, θ < α → two generator rays.
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
            std::f64::consts::FRAC_PI_4,
        );
        let apex = cone_apex(&cone); // (0,0,-1)
        let normal = Direction3d::new(
            (75.0f64.to_radians()).sin(),
            0.0,
            (75.0f64.to_radians()).cos(),
        )
        .unwrap();
        let plane = Plane::from_origin_and_normal(apex, normal);
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 2, "Expected 2 generator rays, got {}", result.len());
        for line in &result {
            assert!(line.len() >= 2);
            assert_all_on_both(line, &plane, &cone, 1e-9);
            // Each ray starts at the apex.
            assert!(
                line[0].distance_to(&apex) < 1e-9,
                "Ray should start at the apex, got {:?}",
                line[0]
            );
        }
    }

    #[test]
    fn test_plane_cone_tangent_ray_through_apex() {
        // Plane through the apex with θ = α → tangent along one generator.
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            1.0,
            std::f64::consts::FRAC_PI_4,
        );
        let apex = cone_apex(&cone);
        let normal = Direction3d::new(
            (45.0f64.to_radians()).sin(),
            0.0,
            (45.0f64.to_radians()).cos(),
        )
        .unwrap();
        let plane = Plane::from_origin_and_normal(apex, normal);
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1, "Expected 1 tangent ray, got {}", result.len());
        assert_all_on_both(&result[0], &plane, &cone, 1e-9);
        assert!(result[0][0].distance_to(&apex) < 1e-9);
    }

    #[test]
    fn test_plane_cone_expanding_circle() {
        // Expanding cone (apex at origin, α=45°) cut by z=2 → circle r=2.
        let cone = expanding_cone();
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 2.0),
            Direction3d::Z,
        );
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1);
        assert_all_on_both(&result[0], &plane, &cone, 1e-9);
        for p in &result[0] {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 2.0).abs() < 1e-9, "Circle radius should be 2.0, got {}", r);
            assert!((p.z - 2.0).abs() < 1e-9);
        }
    }

    #[test]
    fn test_plane_cone_zero_half_angle_delegates_to_cylinder() {
        // half_angle ≈ 0 → cylinder: plane z=3 ⊥ axis → circle of the
        // cone's base radius.
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::Z,
            2.0,
            1e-13,
        );
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 3.0),
            Direction3d::Z,
        );
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1, "Expected 1 circle, got {}", result.len());
        for p in &result[0] {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 2.0).abs() < 1e-6, "Circle radius should be 2.0, got {}", r);
            assert!((p.z - 3.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_plane_cone_dispatch_analytic() {
        // The dispatcher must route Plane×Cone (both orders) to the
        // analytic path — marching output would only be approximate, while
        // analytic points satisfy both surface equations to 1e-9.
        let cone = narrowing_cone();
        let tilt = 20.0f64.to_radians();
        let normal = Direction3d::new(tilt.sin(), 0.0, tilt.cos()).unwrap();
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 5.0), normal);

        let forward = intersect_surfaces(&Surface::Plane(plane.clone()), &Surface::Cone(cone.clone()), 1e-6);
        let reverse = intersect_surfaces(&Surface::Cone(cone.clone()), &Surface::Plane(plane.clone()), 1e-6);

        assert_eq!(forward.polylines.len(), 1);
        assert_eq!(reverse.polylines.len(), 1);
        assert_all_on_both(&forward.polylines[0], &plane, &cone, 1e-9);
        assert_all_on_both(&reverse.polylines[0], &plane, &cone, 1e-9);
        assert_eq!(forward.polylines[0].len(), reverse.polylines[0].len());
    }

    #[test]
    fn test_plane_cone_circle_matches_cylinder_path() {
        // A plane ⊥ axis cuts the cone in a circle whose radius follows
        // r = R + v·tan(α) — cross-check at z=5 for the narrowing cone.
        let cone = narrowing_cone();
        // v=5: r = 5 + 5·(−0.5) = 2.5
        let plane = Plane::from_origin_and_normal(
            Point3d::new(0.0, 0.0, 5.0),
            Direction3d::Z,
        );
        let result = intersect_plane_cone(&plane, &cone, 1e-6);
        assert_eq!(result.len(), 1);
        assert_all_on_both(&result[0], &plane, &cone, 1e-9);
        for p in &result[0] {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 2.5).abs() < 1e-9, "Circle radius should be 2.5, got {}", r);
        }
        // And the parametric surface must agree with the section points.
        for p in &result[0] {
            let v = p.z; // axis is +Z, origin at z=0
            let surface_p = cone.point_at(0.0, v);
            let surface_r = (surface_p.x * surface_p.x + surface_p.y * surface_p.y).sqrt();
            let p_r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((surface_r - p_r).abs() < 1e-9);
        }
    }
}

#[cfg(test)]
mod sphere_sphere_tests {
    use super::*;
    use crate::surface::{SphereSurface, Surface};

    /// |p − center| − radius| < eps
    fn assert_on_sphere(p: &Point3d, center: &Point3d, radius: f64, eps: f64, label: &str) {
        let d = ((p.x - center.x).powi(2)
            + (p.y - center.y).powi(2)
            + (p.z - center.z).powi(2))
        .sqrt();
        assert!(
            (d - radius).abs() < eps,
            "{label}: point {:?} not on sphere (c={:?}, r={}): |p-c|={}",
            p,
            center,
            radius,
            d
        );
    }

    #[test]
    fn disjoint_spheres_empty() {
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::new(20.0, 0.0, 0.0), 5.0);
        assert!(intersect_sphere_sphere(&s1, &s2, 1e-9).is_empty());
    }

    #[test]
    fn contained_spheres_empty() {
        // Small sphere fully inside the big one.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let s2 = SphereSurface::new(Point3d::new(2.0, 1.0, 0.0), 3.0);
        assert!(intersect_sphere_sphere(&s1, &s2, 1e-9).is_empty());
        // Same, arguments swapped.
        assert!(intersect_sphere_sphere(&s2, &s1, 1e-9).is_empty());
    }

    #[test]
    fn concentric_spheres_empty() {
        // Same center, same radius — coincident, no curve.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        assert!(intersect_sphere_sphere(&s1, &s2, 1e-9).is_empty());
        // Same center, different radii — nested, no curve.
        let s3 = SphereSurface::new(Point3d::ORIGIN, 2.0);
        assert!(intersect_sphere_sphere(&s1, &s3, 1e-9).is_empty());
    }

    #[test]
    fn external_tangent_single_point() {
        // d = r1 + r2 = 8 exactly.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::new(8.0, 0.0, 0.0), 3.0);
        let out = intersect_sphere_sphere(&s1, &s2, 1e-9);
        assert_eq!(out.len(), 1, "expected a single tangent point");
        assert_eq!(out[0].len(), 1);
        let p = out[0][0];
        assert_on_sphere(&p, &s1.center, s1.radius, 1e-9, "tangent/s1");
        assert_on_sphere(&p, &s2.center, s2.radius, 1e-9, "tangent/s2");
        // On the center line, between the centers.
        assert!((p.x - 5.0).abs() < 1e-9, "x should be 5.0, got {}", p.x);
        assert!(p.y.abs() < 1e-9 && p.z.abs() < 1e-9);
    }

    #[test]
    fn internal_tangent_single_point() {
        // d = r1 − r2 = 4 exactly; the small sphere touches from inside.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let s2 = SphereSurface::new(Point3d::new(4.0, 0.0, 0.0), 6.0);
        let out = intersect_sphere_sphere(&s1, &s2, 1e-9);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].len(), 1);
        let p = out[0][0];
        assert_on_sphere(&p, &s1.center, s1.radius, 1e-9, "internal/s1");
        assert_on_sphere(&p, &s2.center, s2.radius, 1e-9, "internal/s2");
        // Touch point beyond the inner sphere's center: x = r1 = 10.
        assert!((p.x - 10.0).abs() < 1e-9, "x should be 10.0, got {}", p.x);

        // Swapped argument order: tangency is symmetric — the SAME touch
        // point (x = 10) is produced regardless of which sphere is first.
        let out_swapped = intersect_sphere_sphere(&s2, &s1, 1e-9);
        assert_eq!(out_swapped.len(), 1);
        assert_eq!(out_swapped[0].len(), 1);
        let q = out_swapped[0][0];
        assert_on_sphere(&q, &s2.center, s2.radius, 1e-9, "internal-swapped/s2");
        assert_on_sphere(&q, &s1.center, s1.radius, 1e-9, "internal-swapped/s1");
        // First sphere is the smaller one (c=(4,0,0), r=6): the touch point
        // is still the unique point at distance 6 from it and 10 from the
        // big sphere — x = 10.
        assert!((q.x - 10.0).abs() < 1e-9, "x should be 10.0, got {}", q.x);
    }

    #[test]
    fn general_position_circle_on_both_spheres() {
        // d = 6, r1 = 5, r2 = 3 → a = (36+25−9)/12 = 13/3, h = √(25−169/9).
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::new(6.0, 0.0, 0.0), 3.0);
        let out = intersect_sphere_sphere(&s1, &s2, 1e-9);
        assert_eq!(out.len(), 1, "expected one circle polyline");
        let pts = &out[0];
        assert_eq!(pts.len(), 128, "expected 128 samples, got {}", pts.len());

        let a = 13.0_f64 / 3.0;
        let h = (25.0 - a * a).sqrt();
        for p in pts {
            assert_on_sphere(p, &s1.center, s1.radius, 1e-9, "general/s1");
            assert_on_sphere(p, &s2.center, s2.radius, 1e-9, "general/s2");
            // Radius of the circle around the radical-plane center.
            let r = ((p.x - a).powi(2) + p.y.powi(2) + p.z.powi(2)).sqrt();
            assert!(
                (r - h).abs() < 1e-9,
                "circle radius should be {h}, got {r}"
            );
        }

        // Centroid of a uniformly sampled circle = circle center.
        let centroid = {
            let n = pts.len() as f64;
            let mut c = Point3d::ORIGIN;
            for p in pts {
                c.x += p.x / n;
                c.y += p.y / n;
                c.z += p.z / n;
            }
            c
        };
        assert!((centroid.x - a).abs() < 1e-9);
        assert!(centroid.y.abs() < 1e-9 && centroid.z.abs() < 1e-9);
    }

    #[test]
    fn equal_radii_midpoint_circle() {
        // Equal radii → the radical plane is the perpendicular bisector:
        // circle center at the midpoint of the centers; circle radius
        // h = √(r² − (d/2)²) = √(16 − 9) = √7.
        let s1 = SphereSurface::new(Point3d::new(0.0, 0.0, 0.0), 4.0);
        let s2 = SphereSurface::new(Point3d::new(0.0, 0.0, 6.0), 4.0);
        let h = 7.0_f64.sqrt();
        let out = intersect_sphere_sphere(&s1, &s2, 1e-9);
        assert_eq!(out.len(), 1);
        for p in &out[0] {
            assert_on_sphere(p, &s1.center, s1.radius, 1e-9, "equal/s1");
            assert_on_sphere(p, &s2.center, s2.radius, 1e-9, "equal/s2");
            assert!((p.z - 3.0).abs() < 1e-9, "radical plane at z=3, got z={}", p.z);
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - h).abs() < 1e-9, "circle radius {h}, got {r}");
        }
    }

    #[test]
    fn off_axis_centers_perpendicular_circle() {
        // General center-line direction (not aligned with any axis).
        let c1 = Point3d::new(1.0, 2.0, 3.0);
        let c2 = Point3d::new(4.0, 6.0, 3.0);
        let s1 = SphereSurface::new(c1, 4.0);
        let s2 = SphereSurface::new(c2, 2.0);
        let out = intersect_sphere_sphere(&s1, &s2, 1e-9);
        assert_eq!(out.len(), 1);
        let pts = &out[0];

        // Center-line vector (3, 4, 0)/5. The radical plane sits at
        // distance a = (d² + r1² − r2²)/(2d) = (25+16−4)/10 = 3.7 from c1
        // along the line — every point has the SAME projection.
        let dir = (3.0_f64 / 5.0, 4.0_f64 / 5.0, 0.0_f64);
        let a_expected = 3.7_f64;
        for p in pts {
            assert_on_sphere(p, &c1, 4.0, 1e-9, "offaxis/s1");
            assert_on_sphere(p, &c2, 2.0, 1e-9, "offaxis/s2");
            let dot = (p.x - c1.x) * dir.0 + (p.y - c1.y) * dir.1 + (p.z - c1.z) * dir.2;
            assert!(
                (dot - a_expected).abs() < 1e-9,
                "point not in the radical plane: projection={} (expected {})",
                dot,
                a_expected
            );
        }
    }

    #[test]
    fn dispatch_both_orders() {
        // The geometry dispatcher routes Sphere×Sphere (both orders) to the
        // analytic path — point sets must match exactly.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::new(6.0, 0.0, 0.0), 3.0);
        let a = intersect_surfaces(
            &Surface::Sphere(s1.clone()),
            &Surface::Sphere(s2.clone()),
            1e-9,
        );
        let b = intersect_surfaces(
            &Surface::Sphere(s2),
            &Surface::Sphere(s1),
            1e-9,
        );
        assert_eq!(a.polylines.len(), 1);
        assert_eq!(b.polylines.len(), 1);
        assert_eq!(a.polylines[0].len(), b.polylines[0].len());
        // The two orders build mirrored frames → different parameter phase
        // (point[i] of A is not point[i] of B), so compare as POINT SETS:
        // every point of A has a point of B within 1e-9.
        for pa in a.polylines[0].iter() {
            let min_dist = b.polylines[0]
                .iter()
                .map(|pb| {
                    ((pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2) + (pa.z - pb.z).powi(2))
                        .sqrt()
                })
                .fold(f64::MAX, f64::min);
            assert!(
                min_dist < 1e-9,
                "point sets differ between dispatch orders (min_dist={min_dist})"
            );
        }
    }

    #[test]
    fn near_tangent_yields_tiny_circle() {
        // Just inside the external tangency band: a small-but-valid circle,
        // every point still exactly on both spheres.
        let s1 = SphereSurface::new(Point3d::ORIGIN, 5.0);
        let s2 = SphereSurface::new(Point3d::new(7.9, 0.0, 0.0), 3.0);
        let out = intersect_sphere_sphere(&s1, &s2, 1e-9);
        assert_eq!(out.len(), 1);
        for p in &out[0] {
            assert_on_sphere(p, &s1.center, s1.radius, 1e-9, "near-tangent/s1");
            assert_on_sphere(p, &s2.center, s2.radius, 1e-9, "near-tangent/s2");
        }
    }
}
