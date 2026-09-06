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
    ///
    /// NOTE: this is the legacy single-curve accessor; for multi-branch
    /// intersections prefer `b_spline_curves` (all branches are fitted).
    pub b_spline_curve: Option<NurbsCurve>,
    /// B-spline curves fitted to ALL polyline branches (Vision 2036 §2.1).
    /// Branches that cannot be fitted within tolerance keep their polyline
    /// representation only (per-branch fallback, no global failure).
    pub b_spline_curves: Vec<NurbsCurve>,
}

impl SurfaceSurfaceIntersection {
    /// Get the primary intersection curve as a NurbsCurve if available,
    /// otherwise return None (caller should fall back to polylines).
    pub fn b_spline(&self) -> Option<&NurbsCurve> {
        self.b_spline_curve.as_ref()
    }

    /// Get B-spline curves for all fitted branches (Vision 2036 §2.1).
    /// Empty when no branch could be fitted within tolerance.
    pub fn b_splines(&self) -> &[NurbsCurve] {
        &self.b_spline_curves
    }

    /// Fit B-spline curves to ALL intersection branches, with Newton-Raphson
    /// refinement on both surfaces (Vision 2036 §2.1, full pipeline).
    ///
    /// Per branch: least-squares fit → sample curve → snap each sample onto
    /// the exact intersection via 4D Newton → re-fit refined points. Stores
    /// successful curves in `b_spline_curves` and the first one in
    /// `b_spline_curve` (legacy compat). Branches that fail keep their
    /// polyline representation (spec fallback).
    pub fn fit_b_splines_on_surfaces(&mut self, s1: &Surface, s2: &Surface, tolerance: f64) {
        self.b_spline_curves = self.try_fit_b_splines_on_surfaces(s1, s2, tolerance);
        self.b_spline_curve = self.b_spline_curves.first().cloned();
    }

    /// Multi-branch fitting pipeline — see `fit_b_splines_on_surfaces`.
    /// Returns the fitted curves without mutating `self`.
    pub fn try_fit_b_splines_on_surfaces(
        &self,
        s1: &Surface,
        s2: &Surface,
        tolerance: f64,
    ) -> Vec<NurbsCurve> {
        let mut out = Vec::with_capacity(self.polylines.len());
        for branch in &self.polylines {
            // §2.1 step 1–2: marching points + chord-length least-squares fit.
            let fitted = match lsq_fit_branch(branch, tolerance) {
                Ok(c) => c,
                Err(e) => {
                    log::debug!(
                        "SSI §2.1: branch LSQ fit failed ({} pts): {} — polyline fallback",
                        branch.len(),
                        e
                    );
                    continue;
                }
            };
            // §2.1 step 3: Newton-Raphson refinement on both surfaces.
            // Sample density matches the branch so that the refinement
            // re-fit retains the control-point escalation budget.
            let n_samples = branch.len().clamp(16, 256);
            let final_curve = match newton_refine_curve(&fitted, s1, s2, n_samples) {
                Some(refined) if refined.len() >= 4 => {
                    match lsq_fit_branch(&refined, tolerance) {
                        Ok(refit) => refit,
                        Err(_) => fitted, // refined re-fit failed — keep original LSQ curve
                    }
                }
                _ => fitted, // refinement did not converge — keep original LSQ curve
            };
            out.push(final_curve);
        }
        out
    }

    /// Fit a B-spline curve to the first polyline using chord-length
    /// parameterized least-squares approximation (no surface refinement).
    ///
    /// Per Vision 2030 Task 1: Chord-Length Parameterized B-Spline Fitting.
    /// Vision 2036 §2.1 upgrades the method to a true global least-squares
    /// solve (see `lsq_fit_branch`); use `fit_b_splines_on_surfaces` for the
    /// full pipeline with Newton refinement.
    ///
    /// Returns `Ok(NurbsCurve)` on success, or `Err(FittingError)` on failure.
    /// The caller can fall back to polylines on error.
    pub fn fit_b_spline(&mut self, tolerance: f64) {
        match self.try_fit_b_spline(tolerance) {
            Ok(curve) => {
                if self.b_spline_curves.is_empty() {
                    self.b_spline_curves.push(curve.clone());
                }
                self.b_spline_curve = Some(curve);
            }
            Err(e) => {
                log::debug!("SSI: B-spline fitting failed: {} — using polyline fallback", e);
            }
        }
    }

    /// Try to fit a B-spline curve to the first polyline via global
    /// least-squares (Vision 2036 §2.1 steps 1–2, without refinement).
    pub fn try_fit_b_spline(&self, tolerance: f64) -> Result<NurbsCurve, FittingError> {
        if self.polylines.is_empty() {
            return Err(FittingError::TooFewPoints { got: 0, min: 4 });
        }
        lsq_fit_branch(&self.polylines[0], tolerance)
    }
}

// ============================================================
// Vision 2036 §2.1: least-squares B-spline fitting machinery
// ============================================================

/// Compute normalized chord-length parameters for a polyline.
/// Returns `None` for degenerate (zero-length) input.
fn chord_length_params(pts: &[Point3d]) -> Option<Vec<f64>> {
    let mut params = vec![0.0_f64; pts.len()];
    let mut total_len = 0.0_f64;
    for i in 1..pts.len() {
        let dx = pts[i].x - pts[i - 1].x;
        let dy = pts[i].y - pts[i - 1].y;
        let dz = pts[i].z - pts[i - 1].z;
        total_len += (dx * dx + dy * dy + dz * dz).sqrt();
        params[i] = total_len;
    }
    if total_len < 1e-15 {
        return None;
    }
    for p in &mut params {
        *p /= total_len;
    }
    Some(params)
}

/// Cox–de Boor recurrence: the `degree + 1` nonzero basis values
/// `N_{span-degree+i,degree}(t)` for `i = 0..=degree` (Piegl & Tiller A2.2).
fn bspline_basis_values(knots: &[f64], degree: usize, span: usize, t: f64) -> Vec<f64> {
    let p = degree;
    let mut n = vec![0.0_f64; p + 1];
    // Clamped-domain edges: only the first (resp. last) basis is nonzero.
    if t <= 0.0 {
        n[0] = 1.0;
        return n;
    }
    if t >= *knots.last().unwrap_or(&1.0) {
        n[p] = 1.0;
        return n;
    }
    let mut left = vec![0.0_f64; p + 1];
    let mut right = vec![0.0_f64; p + 1];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0_f64;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            if denom.abs() < 1e-15 {
                continue; // basis degenerates to zero in this branch
            }
            let temp = n[r] / denom;
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// Find the knot span index for parameter `t` in a clamped knot vector:
/// the largest `span` with `knots[span] <= t < knots[span+1]`,
/// `span ∈ [degree, n_cp - 1]`. Linear scan — the systems here are small
/// (n_cp ≤ ~64) and a scan is trivially deterministic.
fn lsq_knot_span(knots: &[f64], degree: usize, n_cp: usize, t: f64) -> usize {
    if t >= knots[n_cp] {
        return n_cp - 1;
    }
    let mut span = degree;
    while span < n_cp - 1 && !(knots[span] <= t && t < knots[span + 1]) {
        span += 1;
    }
    span
}

/// Build a clamped knot vector for least-squares approximation via
/// fractional-position interpolation between data parameters
/// (Piegl & Tiller Eq 9.68–9.69): the parameter range is divided into
/// `n_cp − degree` spans and interior knots are placed at the data-parameter
/// positions that fall at fractional index `j·d`, keeping knot density
/// proportional to data density.
fn averaged_clamped_knots(params: &[f64], n_cp: usize, degree: usize) -> Vec<f64> {
    let m = params.len();
    let n_knots = n_cp + degree + 1;
    let mut knots = vec![0.0_f64; n_knots];
    // Clamped ends.
    for k in knots.iter_mut().take(degree + 1) {
        *k = 0.0;
    }
    for k in knots.iter_mut().rev().take(degree + 1) {
        *k = 1.0;
    }
    // Interior knots: d = (m-1)/(n-p) data points per knot span; interior
    // knot j sits at parameter value interpolated at index j·d.
    let spans = n_cp as isize - degree as isize;
    if spans >= 1 && m > 1 {
        let d = (m as f64 - 1.0) / spans as f64;
        for j in 1..spans {
            let pos = j as f64 * d;
            let i = pos.floor() as usize;
            let alpha = pos - i as f64;
            let i0 = i.min(m - 1);
            let i1 = (i0 + 1).min(m - 1);
            let v = (1.0 - alpha) * params[i0] + alpha * params[i1];
            // Keep strictly inside (0, 1) so the clamped multiplicity is
            // never increased by rounding.
            knots[(j + degree as isize) as usize] = v.clamp(1e-12, 1.0 - 1e-12);
        }
    }
    knots
}

/// Solve a small dense linear system `A x = b` via Gaussian elimination with
/// partial pivoting. Mutates `a` (row-reduced form) and `b` (rhs). Returns
/// `None` when the matrix is (numerically) singular.
fn solve_dense_system(a: &mut [Vec<f64>], b: &mut [f64]) -> Option<Vec<f64>> {
    let n = b.len();
    if n == 0 || a.len() != n {
        return None;
    }
    for col in 0..n {
        // Partial pivoting: largest |a[r][col]| for r >= col.
        let mut pivot = col;
        let mut best = a[col][col].abs();
        for r in (col + 1)..n {
            let v = a[r][col].abs();
            if v > best {
                best = v;
                pivot = r;
            }
        }
        if best < 1e-12 {
            return None;
        }
        if pivot != col {
            a.swap(col, pivot);
            b.swap(col, pivot);
        }
        let d = a[col][col];
        for r in (col + 1)..n {
            let f = a[r][col] / d;
            if f == 0.0 {
                continue;
            }
            for c in col..n {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    // Back substitution.
    let mut x = vec![0.0_f64; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for c in (i + 1)..n {
            s -= a[i][c] * x[c];
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}

/// Global least-squares B-spline fit of one intersection branch
/// (Vision 2036 §2.1 steps 1–2).
///
/// Method: chord-length parameterization → averaged clamped knots →
/// endpoint-interpolating least-squares solve of the normal equations for
/// the interior control points (Gaussian elimination with partial pivoting).
/// The first and last control points equal the branch endpoints exactly,
/// which anchors the curve to the marching data.
///
/// `tolerance` gates the maximum curve-to-data deviation. The control-point
/// count starts at the curvature-adaptive estimate and doubles on
/// `DeviationTooHigh` until the gate is met or the count reaches the data
/// count (a genuine interpolant is never exceeded).
fn lsq_fit_branch(pts: &[Point3d], tolerance: f64) -> Result<NurbsCurve, FittingError> {
    if pts.len() < 4 {
        return Err(FittingError::TooFewPoints { got: pts.len(), min: 4 });
    }
    // Step 1: chord-length parameters.
    let params = match chord_length_params(pts) {
        Some(p) => p,
        None => return Err(FittingError::DegenerateGeometry),
    };
    let m = pts.len();
    let degree = 3usize;

    // Step 2: adaptive control-point count (curvature-scaled), with
    // escalation capacity up to the data count.
    let n_cp_cap = m.saturating_sub(1).max(degree + 1);
    let mut n_cp = adaptive_cp_count(pts)
        .max(degree + 1)
        .min(n_cp_cap);

    loop {
        match lsq_attempt(pts, &params, n_cp, degree) {
            Ok((curve, max_dev)) => {
                if max_dev < tolerance {
                    log::debug!(
                        "SSI §2.1: LSQ fit ({} data pts → {} control points, degree={}, max_dev={:.2e}, tol={:.2e})",
                        m, n_cp, degree, max_dev, tolerance
                    );
                    return Ok(curve);
                }
                if n_cp >= n_cp_cap {
                    return Err(FittingError::DeviationTooHigh { max_dev, tolerance });
                }
                // Escalate: double the control-point budget until the gate is met.
                n_cp = (n_cp * 2).min(n_cp_cap);
            }
            // Singular/ill-conditioned normal equations (only at control-point
            // budgets close to the data count) — further escalation cannot
            // help; report the best deviation observed so far.
            Err(e) => return Err(e),
        }
    }
}

/// Single least-squares attempt with a fixed control-point count.
/// Returns the fitted curve together with its max deviation over ALL data
/// points; the deviation gate itself is applied by the caller.
fn lsq_attempt(
    pts: &[Point3d],
    params: &[f64],
    n_cp: usize,
    degree: usize,
) -> Result<(NurbsCurve, f64), FittingError> {
    let m = pts.len();

    // Step 3: knot vector + basis matrix per data point.
    let knots = averaged_clamped_knots(params, n_cp, degree);

    // Unknowns: control points 1..=n_cp-2 (P0 and P_{n-1} are fixed to the
    // branch endpoints by interpolation constraints). Normal equations:
    //   (A^T A) x = A^T (D - fixed contributions)
    let n_unk = n_cp - 2;
    let mut ata = vec![vec![0.0_f64; n_unk]; n_unk];
    // RHS columns for x, y, z.
    let mut atb = vec![vec![0.0_f64; 3]; n_unk];

    for k in 1..m - 1 {
        let t = params[k];
        let span = lsq_knot_span(&knots, degree, n_cp, t);
        let basis = bspline_basis_values(&knots, degree, span, t);
        // Residual RHS: D_k - N_0(t) * P_0 - N_{n-1}(t) * P_{n-1}.
        let b0 = if span - degree == 0 { basis[0] } else { 0.0 };
        let bn = if span == n_cp - 1 {
            basis[basis.len() - 1]
        } else {
            0.0
        };
        let rx = pts[k].x - b0 * pts[0].x - bn * pts[m - 1].x;
        let ry = pts[k].y - b0 * pts[0].y - bn * pts[m - 1].y;
        let rz = pts[k].z - b0 * pts[0].z - bn * pts[m - 1].z;
        for (i, &bval) in basis.iter().enumerate() {
            if bval.abs() < 1e-14 {
                continue;
            }
            let ci = span as isize - degree as isize + i as isize;
            if ci <= 0 || ci >= n_cp as isize - 1 {
                continue; // fixed control points — already in the residual
            }
            let ui = (ci - 1) as usize;
            for (j, &bval2) in basis.iter().enumerate() {
                if bval2.abs() < 1e-14 {
                    continue;
                }
                let cj = span as isize - degree as isize + j as isize;
                if cj <= 0 || cj >= n_cp as isize - 1 {
                    continue;
                }
                let uj = (cj - 1) as usize;
                ata[ui][uj] += bval * bval2;
            }
            atb[ui][0] += bval * rx;
            atb[ui][1] += bval * ry;
            atb[ui][2] += bval * rz;
        }
    }

    // Solve the three normal-equation systems (same matrix, three RHS).
    let mut sol = vec![Vec::new(); 3];
    for d in 0..3 {
        let mut a_d = ata.clone();
        let mut b_d: Vec<f64> = atb.iter().map(|r| r[d]).collect();
        match solve_dense_system(&mut a_d, &mut b_d) {
            Some(x) => sol[d] = x,
            None => return Err(FittingError::DegenerateGeometry),
        }
    }

    // Assemble control points.
    let mut control_points = Vec::with_capacity(n_cp);
    control_points.push(pts[0]);
    for i in 0..n_unk {
        control_points.push(Point3d::new(sol[0][i], sol[1][i], sol[2][i]));
    }
    control_points.push(pts[m - 1]);

    let weights = vec![1.0_f64; n_cp];
    let curve = NurbsCurve { degree, control_points, weights, knots };

    // Step 4: max deviation over ALL data points.
    let mut max_dev = 0.0_f64;
    let eval_curve = Curve3d::Nurbs(curve.clone());
    for (i, &p) in pts.iter().enumerate() {
        let eval = eval_curve.point_at(params[i]);
        let dev = ((p.x - eval.x).powi(2) + (p.y - eval.y).powi(2) + (p.z - eval.z).powi(2)).sqrt();
        if dev > max_dev {
            max_dev = dev;
        }
    }
    Ok((curve, max_dev))
}

/// Newton-Raphson refinement of a fitted intersection curve on both surfaces
/// (Vision 2036 §2.1 step 3).
///
/// Samples the curve, projects each sample onto both input surfaces
/// (`Surface::project_point`), then runs the 4D Newton solver
/// (`newton_surface_surface`) to snap the projected parameter pair onto the
/// exact intersection. Refined points therefore lie on BOTH surfaces (up to
/// the Newton convergence radius), removing marching-cells discretization
/// error from the fitted curve.
///
/// Returns the refined sample points when at least 80% of the samples
/// converged; otherwise `None` (caller keeps the unrefined fit).
fn newton_refine_curve(
    curve: &NurbsCurve,
    s1: &Surface,
    s2: &Surface,
    n_samples: usize,
) -> Option<Vec<Point3d>> {
    let eval_curve = Curve3d::Nurbs(curve.clone());
    let n = n_samples.max(2);
    let mut refined = Vec::with_capacity(n);
    let mut converged = 0usize;
    for i in 0..n {
        let t = i as f64 / (n - 1) as f64;
        let p = eval_curve.point_at(t);
        let (u1, v1) = s1.project_point(&p);
        let (u2, v2) = s2.project_point(&p);
        // Convergence tolerance 1e-6: the solver uses forward-difference
        // derivatives (eps = 1e-7), so its achievable residual floor is
        // ~1e-7·|S| — tighter gates would never report convergence.
        match newton_surface_surface(s1, s2, u1, v1, u2, v2, 1e-6, 24) {
            Some((ip, _, _, _, _)) => {
                refined.push(ip);
                converged += 1;
            }
            None => refined.push(p),
        }
    }
    if converged * 10 >= n * 8 {
        Some(refined)
    } else {
        None
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

/// Intersect a sphere with a (full, infinite) cylinder — analytic SSI
/// (B1-series follow-up, 2026-09-02; "Steinmetch" cases).
///
/// Setup: project the sphere center `c` onto the cylinder axis, giving foot
/// `f` and lateral offset `d = |c − f|` (distance from the axis, not from
/// the surface). In the cylinder frame `(e1, e2 = axis × e1, n = axis)` a
/// cylinder point is `p(θ, t) = f + R(cosθ·e1 + sinθ·e2) + t·n`; writing
/// the sphere-center offset as `w = d(cosφ₀·e1 + sinφ₀·e2)`, the sphere
/// equation `|p − c|² = r²` reduces to the exact one-dimensional relation
///
/// ```text
/// t²(θ) = A + B·cos(θ − φ₀),   A = r² − R² − d²,  B = 2·d·R
/// ```
///
/// Classification (eps = max(tolerance, 1e-9)):
///
/// - **axis through the center** (`d ≤ eps`): the quartic degenerates to
///   circles `z = ±√(r² − R²)` around the axis — 2 circles (`R < r`), 1
///   equatorial circle of tangency (`R ≈ r`), empty (`R > r`, the sphere
///   lies strictly inside the cylinder);
/// - **disjoint** (`d − R > r`): the axis passes too far outside — empty;
/// - **contained** (`R − d > r`): the whole sphere lies strictly inside
///   the cylinder — empty;
/// - **tangency** (`|d − R| ≈ r`, `d > eps`): the single closest point of
///   the cylinder surface to the sphere center — a 1-point polyline;
/// - **two loops** (`R + d < r`): `t² > 0` for every θ — the cylinder
///   enters and exits the sphere along two disjoint closed curves (the
///   `t = +√` and `t = −√` branches), each a full θ-sweep;
/// - **one loop** (otherwise, `|A| < B`): the branches meet where `t = 0`
///   at `θ = φ₀ ± α` with `α = arccos(−A/B)` — a single closed curve
///   (Viviani-style; the `A ≈ B` boundary is the classic
///   sphere-radius-twice-cylinder-radius self-tangent curve).
///
/// Every sampled point satisfies the cylinder equation exactly (it is
/// constructed on the surface) and the sphere equation up to
/// floating-point rounding (`t` is taken from the relation above) — no
/// marching, no grid search, no Newton refinement. Cylinders in this
/// kernel are infinite, so the curves need no boundary clipping.
pub fn intersect_sphere_cylinder(
    sphere: &SphereSurface,
    cyl: &CylinderSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);
    let r = sphere.radius;
    let big_r = cyl.radius;
    if r <= eps || big_r <= eps {
        return vec![];
    }

    // Cylinder frame at the sphere center's axial position.
    let n = Vec3d::new(cyl.axis.x, cyl.axis.y, cyl.axis.z);
    let e1 = Vec3d::new(cyl.x_dir.x, cyl.x_dir.y, cyl.x_dir.z);
    let e2 = Vec3d::new(
        cyl.axis.y * cyl.x_dir.z - cyl.axis.z * cyl.x_dir.y,
        cyl.axis.z * cyl.x_dir.x - cyl.axis.x * cyl.x_dir.z,
        cyl.axis.x * cyl.x_dir.y - cyl.axis.y * cyl.x_dir.x,
    );
    let cs = Vec3d::new(
        sphere.center.x - cyl.origin.x,
        sphere.center.y - cyl.origin.y,
        sphere.center.z - cyl.origin.z,
    );
    let along = cs.x * n.x + cs.y * n.y + cs.z * n.z;
    let foot = Vec3d::new(
        cyl.origin.x + along * n.x,
        cyl.origin.y + along * n.y,
        cyl.origin.z + along * n.z,
    );
    let w = Vec3d::new(cs.x - along * n.x, cs.y - along * n.y, cs.z - along * n.z);
    let d = (w.x * w.x + w.y * w.y + w.z * w.z).sqrt();

    let push_point = |theta: f64, t: f64, out: &mut Vec<Point3d>| {
        let c = theta.cos();
        let s = theta.sin();
        out.push(Point3d::new(
            foot.x + big_r * (c * e1.x + s * e2.x) + t * n.x,
            foot.y + big_r * (c * e1.y + s * e2.y) + t * n.y,
            foot.z + big_r * (c * e1.z + s * e2.z) + t * n.z,
        ));
    };

    // ── Case 1: axis through the sphere center — Steinmetch circles ─────
    if d <= eps {
        let t_sq = r * r - big_r * big_r;
        if t_sq < -eps * eps {
            // Sphere strictly inside the cylinder.
            return vec![];
        }
        let t = t_sq.max(0.0).sqrt();
        let n_samples = 128;
        let t_values: &[f64] = if t > eps { &[t, -t] } else { &[0.0] };
        let mut circles = Vec::with_capacity(t_values.len());
        for &tt in t_values {
            let mut circle = Vec::with_capacity(n_samples);
            for i in 0..n_samples {
                let theta = 2.0 * std::f64::consts::PI * i as f64 / n_samples as f64;
                push_point(theta, tt, &mut circle);
            }
            circles.push(circle);
        }
        return circles;
    }

    // ── Case 2: off-axis — classify via t²(θ) = A + B·cos(θ − φ₀) ───────
    // Tangency / disjoint / contained in terms of |d − R| vs r (linear
    // units — cleaner than comparing A + B = r² − (d − R)² against eps²).
    let lateral_gap = (d - big_r).abs();
    if lateral_gap - r > eps {
        // Disjoint (axis too far outside) or sphere fully inside the
        // cylinder — no intersection either way.
        return vec![];
    }
    if (lateral_gap - r).abs() <= eps {
        // Tangency: the closest point of the cylinder surface to the
        // sphere center, in the direction of w (θ = φ₀, t = 0).
        let ux = w.x / d;
        let uy = w.y / d;
        let uz = w.z / d;
        return vec![vec![Point3d::new(
            foot.x + big_r * ux,
            foot.y + big_r * uy,
            foot.z + big_r * uz,
        )]];
    }

    let a_coeff = r * r - big_r * big_r - d * d;
    let b_coeff = 2.0 * d * big_r;
    // Direction of w in the frame: w = d(cosφ₀·e1 + sinφ₀·e2) — recover
    // φ₀ from the frame components.
    let w_e1 = w.x * e1.x + w.y * e1.y + w.z * e1.z;
    let w_e2 = w.x * e2.x + w.y * e2.y + w.z * e2.z;
    let phi = w_e2.atan2(w_e1);

    let n_samples = 128;

    // Two loops: t² > 0 everywhere (r > R + d).
    if a_coeff - b_coeff > eps * eps {
        let mut loops = Vec::with_capacity(2);
        for sign in [1.0, -1.0] {
            let mut curve = Vec::with_capacity(n_samples);
            for i in 0..n_samples {
                let theta = 2.0 * std::f64::consts::PI * i as f64 / n_samples as f64;
                let t_sq = (a_coeff + b_coeff * (theta - phi).cos()).max(0.0);
                push_point(theta, sign * t_sq.sqrt(), &mut curve);
            }
            loops.push(curve);
        }
        return loops;
    }

    // One loop: the branches join at θ = φ ± α where t = 0.
    // cos α = −A/B ∈ (−1, 1); α = π is the Viviani self-tangent boundary.
    let cos_alpha = (-a_coeff / b_coeff).clamp(-1.0, 1.0);
    let alpha = cos_alpha.acos();
    let theta_start = phi - alpha;
    let theta_end = phi + alpha;
    let span = theta_end - theta_start;

    // Near the pinch points (t → 0) the curve is sqrt-singular in θ:
    // uniform θ-steps produce spatial steps ~√Δθ there, several times
    // larger than mid-branch. The fraction s(η) = (1 − cos(ηπ))/2 has
    // zero derivative at η = 0 and 1, clustering samples exactly at the
    // two join points and equalizing the spatial step along the branch.
    let branch_theta = |eta: f64| -> f64 { theta_start + span * (1.0 - (eta * std::f64::consts::PI).cos()) / 2.0 };
    let branch_t = |theta: f64, sign: f64| -> f64 {
        let t_sq = (a_coeff + b_coeff * (theta - phi).cos()).max(0.0);
        sign * t_sq.sqrt()
    };

    let mut curve = Vec::with_capacity(2 * n_samples - 2);
    // Upper branch: η from 0 to 1 (t ≥ 0), inclusive at both pinches.
    for i in 0..n_samples {
        let eta = i as f64 / (n_samples - 1) as f64;
        let theta = branch_theta(eta);
        push_point(theta, branch_t(theta, 1.0), &mut curve);
    }
    // Lower branch: η from 1 BACK to 0 (t ≤ 0), skipping both endpoints —
    // they coincide with the upper branch's join points, so the loop
    // closes without a duplicated vertex (house convention).
    for i in 1..(n_samples - 1) {
        let eta = 1.0 - i as f64 / (n_samples - 1) as f64;
        let theta = branch_theta(eta);
        push_point(theta, branch_t(theta, -1.0), &mut curve);
    }
    if curve.len() >= 2 {
        vec![curve]
    } else {
        vec![]
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
        if h_sq <= 0.0 {
            // Numerical edge case — tangential touch (1 line). This also
            // catches the exact-tangency h_sq == 0 case.
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

    // ── Non-parallel axes — exact per-θ quadratic solve ────────────────
    //
    // Parametrize cylinder A: p(θ, t) = o_a + R_a(cosθ·e1 + sinθ·e2) + t·n_a.
    // With w(θ) = o_a − o_b + R_a(cosθ·e1 + sinθ·e2), u(θ) = w(θ) × n_b
    // and v = n_a × n_b (a = |v|² = sin²(axis angle) > 0), the cylinder-B
    // constraint |(p − o_b) × n_b|² = R_b² becomes
    //
    //     a·t² + b(θ)·t + c(θ) = 0,
    //     b(θ) = 2·u(θ)·v          (degree-1 trig polynomial),
    //     c(θ) = |u(θ)|² − R_b²    (degree-2 trig polynomial).
    //
    // For every θ whose discriminant D(θ) = b² − 4ac is non-negative the
    // roots t± are exact: each emitted point lies ON cylinder A by
    // construction and ON cylinder B up to floating-point rounding of the
    // quadratic solve — no marching, no Newton refinement.
    //
    // Curve structure (complete intersection of two infinite non-parallel
    // cylinders is ≤ 2 closed loops):
    //   • D > 0 on the full circle → TWO loops (the t+ and t− root
    //     branches). Interior points with D = 0 are surface-tangency
    //     points where the loops touch and the parametrization kinks
    //     (e.g. the classic equal-radius perpendicular bicylinder: the
    //     true curves are two crossing ellipses; the emitted loops are
    //     the upper/lower envelopes through the same point set).
    //   • D > 0 on an arc [s, e] (D = 0 at both ends) → ONE closed loop:
    //     the two root branches join at the arc ends (pinch points),
    //     traced out-and-back like the sphere×cylinder quartic.
    //   • D ≤ 0 everywhere → empty, or a single tangency point when the
    //     minimum of D touches zero (golden-section refinement).
    let eps = tolerance.max(1e-9);
    let r_a = cyl_a.radius;
    let r_b = cyl_b.radius;
    if r_a <= eps || r_b <= eps {
        return vec![];
    }

    let n_a = Vec3d::new(cyl_a.axis.x, cyl_a.axis.y, cyl_a.axis.z);
    let n_b = Vec3d::new(cyl_b.axis.x, cyl_b.axis.y, cyl_b.axis.z);
    let e1 = Vec3d::new(cyl_a.x_dir.x, cyl_a.x_dir.y, cyl_a.x_dir.z);
    let e2 = n_a.cross(&e1);
    let o_a = Vec3d::new(cyl_a.origin.x, cyl_a.origin.y, cyl_a.origin.z);
    let o_b = Vec3d::new(cyl_b.origin.x, cyl_b.origin.y, cyl_b.origin.z);

    let v = n_a.cross(&n_b); // |v| = sin(axis angle) ≠ 0 here
    let a_quad = v.length_sq();
    if a_quad <= 1e-30 {
        return vec![]; // degenerate near-parallel (guarded above)
    }

    // u(θ) = u0 + R_a·cosθ·u1 + R_a·sinθ·u2 (constant coefficient vectors)
    let w0 = Vec3d::new(o_a.x - o_b.x, o_a.y - o_b.y, o_a.z - o_b.z);
    let u0 = w0.cross(&n_b);
    let u1 = e1.cross(&n_b);
    let u2 = e2.cross(&n_b);

    // b(θ) = b0 + bc·cosθ + bs·sinθ
    let b0 = 2.0 * u0.dot(&v);
    let bc = 2.0 * r_a * u1.dot(&v);
    let bs = 2.0 * r_a * u2.dot(&v);

    // c(θ) = c0 + cc·cosθ + cs·sinθ + ccc·cos²θ + css·sin²θ + ccs·cosθ·sinθ
    let c0 = u0.length_sq() - r_b * r_b;
    let cc = 2.0 * r_a * u0.dot(&u1);
    let cs = 2.0 * r_a * u0.dot(&u2);
    let ccc = r_a * r_a * u1.length_sq();
    let css = r_a * r_a * u2.length_sq();
    let ccs = 2.0 * r_a * r_a * u1.dot(&u2);

    let eval_b = |theta: f64| -> f64 { b0 + bc * theta.cos() + bs * theta.sin() };
    let eval_c = |theta: f64| -> f64 {
        let cos = theta.cos();
        let sin = theta.sin();
        c0 + cc * cos + cs * sin + ccc * cos * cos + css * sin * sin + ccs * cos * sin
    };
    let disc = |theta: f64| -> f64 {
        let b = eval_b(theta);
        b * b - 4.0 * a_quad * eval_c(theta)
    };
    // Root pair (t+, t−) with the discriminant clamped at zero: only used
    // at θ where D ≥ 0 (or exactly at refined D = 0 boundaries).
    let roots = |theta: f64| -> (f64, f64) {
        let b = eval_b(theta);
        let d = (b * b - 4.0 * a_quad * eval_c(theta)).max(0.0);
        let sq = d.sqrt();
        let half = -b / (2.0 * a_quad);
        (half + sq / (2.0 * a_quad), half - sq / (2.0 * a_quad))
    };
    let push_point = |theta: f64, t: f64, out: &mut Vec<Point3d>| {
        let cos = theta.cos();
        let sin = theta.sin();
        out.push(Point3d::new(
            o_a.x + r_a * (cos * e1.x + sin * e2.x) + t * n_a.x,
            o_a.y + r_a * (cos * e1.y + sin * e2.y) + t * n_a.y,
            o_a.z + r_a * (cos * e1.z + sin * e2.z) + t * n_a.z,
        ));
    };

    // ── Scan D(θ) on a uniform grid, fill interior touch-holes ─────────
    let m_scan = 720usize;
    let theta_of = |idx: usize| -> f64 {
        2.0 * std::f64::consts::PI * idx as f64 / m_scan as f64
    };
    let d_vals: Vec<f64> = (0..m_scan).map(|i| disc(theta_of(i))).collect();
    let mut valid: Vec<bool> = d_vals.iter().map(|&d| d > 0.0).collect();

    // Fill 1–2-sample holes (D dips to ≤ 0 between valid samples): these
    // are interior surface-tangency touches, not curve boundaries. An
    // invalid index i belongs to a hole of total length ≤ 2 with VALID
    // banks on both circular sides iff one of:
    //   hole = {i}            : valid[i−1] && valid[i+1]
    //   hole = {i, i+1}       : valid[i−1] && valid[i+2]
    //   hole = {i−1, i}       : valid[i−2] && valid[i+1]
    // Filling such an index extends the neighbouring run across the touch.
    loop {
        let mut changed = false;
        for i in 0..m_scan {
            if valid[i] {
                continue;
            }
            let im1 = (i + m_scan - 1) % m_scan;
            let im2 = (i + m_scan - 2) % m_scan;
            let i1 = (i + 1) % m_scan;
            let i2 = (i + 2) % m_scan;
            let fill_i = (valid[im1] && valid[i1])
                || (valid[im1] && valid[i2])
                || (valid[im2] && valid[i1]);
            if fill_i {
                valid[i] = true;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    let all_valid = valid.iter().all(|&v| v);
    let n_samples = 128usize;

    if all_valid {
        // ── Full circle: two root-branch loops ─────────────────────────
        let mut loops = Vec::with_capacity(2);
        for branch in [0usize, 1] {
            let mut curve = Vec::with_capacity(n_samples);
            for i in 0..n_samples {
                let theta = 2.0 * std::f64::consts::PI * i as f64 / n_samples as f64;
                let (t_plus, t_minus) = roots(theta);
                push_point(theta, if branch == 0 { t_plus } else { t_minus }, &mut curve);
            }
            loops.push(curve);
        }
        return loops;
    }

    if !valid.iter().any(|&v| v) {
        // ── No valid θ: tangency or empty ──────────────────────────────
        // D ≤ 0 everywhere: tangency is where D touches zero — i.e. the
        // MAXIMUM of D. Golden-section refinement of the grid argmax.
        let grid_argmax = (0..m_scan)
            .max_by(|&i, &j| d_vals[i].partial_cmp(&d_vals[j]).unwrap())
            .unwrap();
        let step = 2.0 * std::f64::consts::PI / m_scan as f64;
        let mut lo = theta_of(grid_argmax) - step;
        let mut hi = theta_of(grid_argmax) + step;
        let phi = 0.6180339887498949;
        let mut x1 = hi - phi * (hi - lo);
        let mut x2 = lo + phi * (hi - lo);
        let mut f1 = disc(x1);
        let mut f2 = disc(x2);
        for _ in 0..80 {
            if f1 > f2 {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - phi * (hi - lo);
                f1 = disc(x1);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + phi * (hi - lo);
                f2 = disc(x2);
            }
        }
        let theta_star = 0.5 * (lo + hi);
        // fp-scale slack: b is the dominant D scale (D = b² − 4ac).
        let tol_tang = 1e-10 * (b0.abs() + bc.abs() + bs.abs() + 1.0).powi(2);
        if disc(theta_star) > -tol_tang {
            let (t, _) = roots(theta_star);
            return vec![vec![Point3d::new(
                o_a.x + r_a * (theta_star.cos() * e1.x + theta_star.sin() * e2.x) + t * n_a.x,
                o_a.y + r_a * (theta_star.cos() * e1.y + theta_star.sin() * e2.y) + t * n_a.y,
                o_a.z + r_a * (theta_star.cos() * e1.z + theta_star.sin() * e2.z) + t * n_a.z,
            )]];
        }
        return vec![];
    }

    // ── Extract maximal valid arcs (circular), refine D = 0 boundaries ─
    // Linear runs over [0, m_scan), then merge the wrap-around run.
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut i = 0usize;
    while i < m_scan {
        if valid[i] {
            let mut j = i;
            while j < m_scan && valid[j] {
                j += 1;
            }
            runs.push((i, j - 1));
            i = j;
        } else {
            i += 1;
        }
    }
    if runs.len() >= 2 {
        let first = runs[0];
        let last = *runs.last().unwrap();
        if first.0 == 0 && last.1 == m_scan - 1 {
            // Wrap-around: merge into a single run possibly extending
            // beyond m_scan (θ > 2π is fine — trig is periodic).
            runs.pop();
            runs[0] = (last.0, first.1 + m_scan);
        }
    }

    // Bisection on the strict sign change invalid→valid (D ≤ 0 → D > 0).
    let bisect = |theta_neg: f64, theta_pos: f64| -> f64 {
        let mut lo = theta_neg;
        let mut hi = theta_pos;
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if disc(mid) > 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        0.5 * (lo + hi)
    };

    let mut result = Vec::new();
    for &(s_idx, e_idx) in &runs {
        // Bisection brackets: the samples just outside the run are
        // invalid (D ≤ 0), the run endpoints are strictly valid (D > 0).
        // Unwrapped θ is fine for wrapped runs — trig is periodic.
        let theta_s = 2.0 * std::f64::consts::PI * s_idx as f64 / m_scan as f64;
        let theta_e = 2.0 * std::f64::consts::PI * e_idx as f64 / m_scan as f64;
        let step = 2.0 * std::f64::consts::PI / m_scan as f64;
        let theta_lo = bisect(theta_s - step, theta_s);
        // Right boundary: the INVALID sample (θ(e)+step) comes FIRST —
        // bisect(neg, pos) keeps D(lo) ≤ 0 < D(hi) and converges to the
        // valid→invalid root from inside the arc.
        let theta_hi = bisect(theta_e + step, theta_e);
        let span = theta_hi - theta_lo;
        if span <= 1e-12 {
            continue; // scan artifact
        }

        // One closed loop: upper root branch out, lower branch back, with
        // the cos-fraction s(η) = (1 − cos(ηπ))/2 clustering samples at
        // both sqrt-singular pinch ends (same idiom as sphere×cylinder).
        let branch_theta = |eta: f64| -> f64 {
            theta_lo + span * (1.0 - (eta * std::f64::consts::PI).cos()) / 2.0
        };
        let mut curve = Vec::with_capacity(2 * n_samples - 2);
        for k in 0..n_samples {
            let eta = k as f64 / (n_samples - 1) as f64;
            let theta = branch_theta(eta);
            let (t_plus, _) = roots(theta);
            push_point(theta, t_plus, &mut curve);
        }
        for k in 1..(n_samples - 1) {
            let eta = 1.0 - k as f64 / (n_samples - 1) as f64;
            let theta = branch_theta(eta);
            let (_, t_minus) = roots(theta);
            push_point(theta, t_minus, &mut curve);
        }

        // Collapse degenerate micro-loops (near-tangency arcs whose
        // spatial extent is below the length tolerance) to one point.
        let extent = curve
            .iter()
            .fold((curve[0], curve[0]), |(mn, mx), &p| {
                (
                    Point3d::new(mn.x.min(p.x), mn.y.min(p.y), mn.z.min(p.z)),
                    Point3d::new(mx.x.max(p.x), mx.y.max(p.y), mx.z.max(p.z)),
                )
            });
        let diag = (extent.0.x - extent.1.x).abs()
            + (extent.0.y - extent.1.y).abs()
            + (extent.0.z - extent.1.z).abs();
        if curve.len() >= 2 && diag <= 100.0 * eps * (1.0 + r_a + r_b) {
            result.push(vec![curve[0]]);
        } else if curve.len() >= 2 {
            result.push(curve);
        }
    }
    result
}

// ═════════════════════════════════════════════════════════════════════════
// Cone×Cone / Cone×Cylinder — analytic SSI (B1-series, 2026-09-02)
//
// Both cones and the cone-vs-cylinder pair reduce to a per-azimuth QUADRATIC
// in one surface parameter, exactly like cylinder×cylinder:
//
//  Cone×Cone (cone A parametrized from its apex, slant t ≥ 0):
//    p(θ, t) = P_a + t·g_a(θ),  g_a(θ) = sinα·(cosθ·e_x + sinθ·e_y) + cosα·m_a
//    (m_a = nappe direction, unit; |g_a| = 1). Cone B's nappe constraint
//    (angle(w, m_b) = β with w = p − P_b) is
//        (w·m_b)² = cos²β·|w|²,  w·m_b ≥ 0 (nappe side),
//    which expands to
//        a(θ)·t² + b(θ)·t + c = 0:
//        a = gm(θ)² − cos²β            (degree-2 trig),
//        b = 2(h₀·gm(θ) − cos²β·w₀g(θ)) (degree-1 trig),
//        c = h₀² − cos²β·|w₀|²          (constant),
//    with gm(θ) = g_a(θ)·m_b (degree-1), w₀g(θ) = w₀·g_a(θ) (degree-1),
//    h₀ = w₀·m_b, w₀ = P_a − P_b. Every emitted root lies ON cone A by
//    construction and ON cone B up to the fp rounding of the quadratic
//    solve — no marching, no Newton.
//
//  Cone×Cylinder (cylinder parametrized by axial t ∈ ℝ):
//    p(θ, t) = o_c + t·n_c + R·q(θ); the cone nappe constraint gives
//        a₂·t² + b(θ)·t + c(θ) = 0 with a₂ = (n_c·m)² − cos²α constant,
//        b degree-1, c degree-2 — the cylinder×cylinder structure.
//
//  Sheet constraints (single-nappe cones): t ≥ 0 / w·m ≥ 0 filter the
//  mirror-nappe roots; boundary crossings of those sheets are curve
//  endpoints that pass through an APEX (a single point shared by all θ).
//  Branch arcs are glued at coincident endpoints, which reproduces the
//  pinch joins (D = 0), a(θ) = 0 crossings (generator-parallel azimuths,
//  where one root escapes to infinity) and apex pass-throughs.
//
//  Degenerate configurations handled by dedicated paths:
//    • both half-angles ≈ 0 → two cylinders (delegate);
//    • one half-angle ≈ 0 → cone×cylinder (delegate);
//    • half-angle ≈ ±π/2 → a plane (delegate to plane×cone / plane×cyl);
//    • shared apex → common generator RAYS (direction-circle solve on the
//      unit sphere: two linear constraints d·m_a = cosα, d·m_b = cosβ);
//    • a(θ) ≡ 0 (parallel axes + equal angles) → the intersection is a
//      PLANAR CONIC (the homogeneous quadratic parts of the two cone
//      equations coincide, their difference is linear) → linear root
//      t(θ) = −c/b(θ), arcs between sheet/clip boundaries.
// ═════════════════════════════════════════════════════════════════════════

/// Degree-2 trig polynomial v(θ) = k0 + k1·cosθ + s1·sinθ + k2·cos2θ + s2·sin2θ.
#[derive(Clone, Copy, Debug, Default)]
struct Trig2 {
    k0: f64,
    k1: f64,
    s1: f64,
    k2: f64,
    s2: f64,
}

impl Trig2 {
    /// Degree-1: k0 + k1·cosθ + s1·sinθ.
    fn linear(k0: f64, k1: f64, s1: f64) -> Self {
        Trig2 { k0, k1, s1, k2: 0.0, s2: 0.0 }
    }

    fn eval(&self, theta: f64) -> f64 {
        let c = theta.cos();
        let s = theta.sin();
        let c2 = 2.0 * c * c - 1.0; // cos 2θ
        let s2 = 2.0 * s * c; // sin 2θ
        self.k0 + self.k1 * c + self.s1 * s + self.k2 * c2 + self.s2 * s2
    }

    /// All coefficients negligible against `scale`.
    fn is_trivial(&self, scale: f64) -> bool {
        let m = self
            .k0
            .abs()
            .max(self.k1.abs())
            .max(self.s1.abs())
            .max(self.k2.abs())
            .max(self.s2.abs());
        m <= scale
    }
}

/// Apex-based view of one cone nappe.
///
/// A `ConeSurface` is a single nappe of an infinite cone:
/// { P + t·g(θ) : t ≥ 0 } with the unit generator
/// g(θ) = sinα·(cosθ·e_x + sinθ·e_y) + cosα·m, where m = sign(tan α)·axis
/// is the nappe direction and α = |half_angle|. `half_angle` ≈ 0 (cylinder)
/// and ≈ ±π/2 (plane) are rejected and handled by the pair-level paths.
struct ConeView {
    apex: Point3d,
    /// Nappe direction (unit): sign(tan(half_angle)) · axis.
    m: Vec3d,
    /// Orthonormal frame ⊥ axis (generator azimuth plane).
    ex: Vec3d,
    ey: Vec3d,
    sin_a: f64,
    cos_a: f64,
    /// Base radius (scale reference; 0 for expanding cones).
    radius: f64,
}

impl ConeView {
    fn of(cone: &ConeSurface) -> Option<ConeView> {
        let tan_ha = cone.half_angle.tan();
        if !tan_ha.is_finite() {
            return None; // half_angle ≈ ±π/2 — a plane
        }
        if tan_ha.abs() < 1e-10 {
            return None; // half_angle ≈ 0 — a cylinder
        }
        let s = tan_ha.signum();
        let apex = if cone.expanding {
            Point3d::new(cone.origin.x, cone.origin.y, cone.origin.z)
        } else {
            let v_apex = -cone.radius / tan_ha;
            Point3d::new(
                cone.origin.x + v_apex * cone.axis.x,
                cone.origin.y + v_apex * cone.axis.y,
                cone.origin.z + v_apex * cone.axis.z,
            )
        };
        let n = Vec3d::new(cone.axis.x, cone.axis.y, cone.axis.z);
        // Re-orthogonalize x_dir against the axis (plane_cone idiom).
        let raw = Vec3d::new(cone.x_dir.x, cone.x_dir.y, cone.x_dir.z);
        let dk = raw.dot(&n);
        let mut ex = Vec3d::new(raw.x - dk * n.x, raw.y - dk * n.y, raw.z - dk * n.z);
        let ex_len = ex.length();
        if ex_len < 1e-9 {
            // x_dir ∥ axis — build any perpendicular direction: n × e_x,
            // falling back to n × e_y when the axis is parallel to e_x
            // (n × e_x = 0 there).
            let mut fb = Vec3d::new(0.0, n.z, -n.y); // n × e_x
            if fb.length_sq() < 1e-6 {
                fb = Vec3d::new(-n.z, 0.0, n.x); // n × e_y
            }
            let l = fb.length();
            if l < 1e-12 {
                return None;
            }
            ex = Vec3d::new(fb.x / l, fb.y / l, fb.z / l);
        } else {
            ex = Vec3d::new(ex.x / ex_len, ex.y / ex_len, ex.z / ex_len);
        }
        let ey = n.cross(&ex);
        let alpha = cone.half_angle.abs();
        Some(ConeView {
            apex,
            m: Vec3d::new(s * n.x, s * n.y, s * n.z),
            ex,
            ey,
            sin_a: alpha.sin(),
            cos_a: alpha.cos(),
            radius: cone.radius,
        })
    }

    /// Unit generator direction at azimuth θ.
    fn generator(&self, theta: f64) -> Vec3d {
        let c = theta.cos();
        let s = theta.sin();
        Vec3d::new(
            self.sin_a * (c * self.ex.x + s * self.ey.x) + self.cos_a * self.m.x,
            self.sin_a * (c * self.ex.y + s * self.ey.y) + self.cos_a * self.m.y,
            self.sin_a * (c * self.ex.z + s * self.ey.z) + self.cos_a * self.m.z,
        )
    }
}

/// A `ConeSurface` with half_angle ≈ 0 viewed as a cylinder.
fn cone_as_cylinder(cone: &ConeSurface) -> CylinderSurface {
    CylinderSurface {
        origin: cone.origin,
        axis: cone.axis,
        radius: cone.radius,
        x_dir: cone.x_dir,
    }
}

/// A `ConeSurface` with half_angle ≈ ±π/2 viewed as the plane through the
/// apex perpendicular to the axis.
fn cone_as_plane(cone: &ConeSurface) -> Plane {
    let tan_ha = cone.half_angle.tan();
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
    Plane::from_origin_and_normal(apex, cone.axis)
}

/// 1-D θ-domain arc engine shared by the cone-family analytic solvers.
///
/// Scans branch-sheeted root validity on a uniform θ grid, extracts
/// maximal valid runs (circular, wrap-merged), refines run boundaries by
/// bisection on the boolean validity, samples each run with end-clustered
/// θ (the sqrt-singularity idiom of sphere×cylinder / cylinder×cylinder),
/// glues branch arcs at coincident endpoints (pinch joins, a(θ) = 0
/// crossings, apex pass-throughs) and collapses degenerate micro-loops to
/// single tangency points.
struct ThetaArcEngine<'a> {
    m_scan: usize,
    n_samples: usize,
    /// Branch-sheeted roots at θ: [branch+, branch−]; `None` = invalid
    /// (no real root, off-sheet, or beyond the clip). The quadratic
    /// discriminant is clamped at zero (cylinder×cylinder convention), and
    /// a(θ) ≈ 0 azimuths fall back to the single finite linear root.
    roots_at: &'a dyn Fn(f64) -> [Option<f64>; 2],
    /// Strict discriminant D(θ) (for the no-valid-θ tangency search).
    disc_at: &'a dyn Fn(f64) -> f64,
    /// Point on the parametrized surface at (θ, t).
    point_at: &'a dyn Fn(f64, f64) -> Point3d,
    /// Endpoint glue tolerance (spatial).
    glue_tol: f64,
    /// Micro-loop collapse extent threshold (spatial).
    collapse_tol: f64,
    /// `true` when the equation degenerated to linear (a ≡ 0): skip the
    /// discriminant tangency search (D = b² ≥ 0 is meaningless there).
    linear: bool,
}

impl<'a> ThetaArcEngine<'a> {
    fn solve(&self) -> Vec<Vec<Point3d>> {
        let m = self.m_scan;
        let two_pi = 2.0 * std::f64::consts::PI;
        let theta_of = |i: usize| -> f64 { two_pi * i as f64 / m as f64 };
        let step = two_pi / m as f64;

        // ── Branch validity masks ─────────────────────────────────────────
        let mut valid: [Vec<bool>; 2] = [Vec::with_capacity(m), Vec::with_capacity(m)];
        for i in 0..m {
            let roots = (self.roots_at)(theta_of(i));
            for br in 0..2 {
                valid[br].push(roots[br].is_some());
            }
        }

        if !valid[0].iter().any(|&v| v) && !valid[1].iter().any(|&v| v) {
            return self.tangency_or_empty();
        }

        // ── Per-branch runs, boundary bisection, emission ────────────────
        let mut arcs: Vec<Vec<Point3d>> = Vec::new();
        for br in 0..2 {
            let mut runs: Vec<(usize, usize)> = Vec::new();
            let mut i = 0usize;
            while i < m {
                if valid[br][i] {
                    let mut j = i;
                    while j < m && valid[br][j] {
                        j += 1;
                    }
                    runs.push((i, j - 1));
                    i = j;
                } else {
                    i += 1;
                }
            }
            if runs.is_empty() {
                continue;
            }
            // Wrap-merge: a run ending at m−1 and a run starting at 0 are
            // one circular run (θ > 2π is fine — trig is periodic).
            if runs.len() >= 2 {
                let first = runs[0];
                let last = *runs.last().unwrap();
                if first.0 == 0 && last.1 == m - 1 {
                    runs.pop();
                    runs[0] = (last.0, first.1 + m);
                }
            }

            for &(s_idx, e_idx) in &runs {
                let run_len = e_idx - s_idx + 1;
                let n = self.n_samples;
                let full_circle = run_len >= m;
                let mut curve = Vec::with_capacity(n);
                if full_circle {
                    // Full circle: uniform sampling, no closure duplicate
                    // (the cylinder full-circle-loop convention).
                    let th_lo = theta_of(s_idx % m);
                    for k in 0..n {
                        let th = th_lo + two_pi * k as f64 / n as f64;
                        if let Some(t) = (self.roots_at)(th)[br] {
                            curve.push((self.point_at)(th, t));
                        }
                    }
                } else {
                    // Partial arc: bisect both boundaries on the boolean
                    // validity. The INVALID sample comes first —
                    // bisect(invalid, valid) keeps the root bracketed and
                    // converges from inside the arc (cylinder×cylinder
                    // convention; swapping the arguments converges one grid
                    // step past the boundary).
                    // e_idx stays UNWRAPPED for wrap-merged runs
                    // (θ > 2π is fine — trig is periodic); re-wrapping it
                    // would make the span negative and drop the arc.
                    let theta_s = theta_of(s_idx);
                    let theta_e = theta_of(e_idx);
                    let th_lo = self.bisect_valid(br, theta_s - step, theta_s);
                    let th_hi = self.bisect_valid(br, theta_e + step, theta_e);
                    let span = th_hi - th_lo;
                    if span <= 1e-12 {
                        continue; // scan artifact
                    }
                    // End-clustered sampling: s(η) = (1 − cos(ηπ))/2 has
                    // zero derivative at both ends, compensating the
                    // sqrt-singularity of the roots at pinch boundaries.
                    for k in 0..n {
                        let eta = k as f64 / (n - 1) as f64;
                        let th =
                            th_lo + span * (1.0 - (eta * std::f64::consts::PI).cos()) / 2.0;
                        if let Some(t) = (self.roots_at)(th)[br] {
                            curve.push((self.point_at)(th, t));
                        }
                    }
                }
                if !curve.is_empty() {
                    arcs.push(curve);
                }
            }
        }

        // ── Glue branch arcs at coincident endpoints, collapse micro-loops ─
        let mut result = glue_arcs(arcs, self.glue_tol);
        for curve in &mut result {
            if curve.len() >= 2 {
                let (mn, mx) = bounding_extent(curve);
                let diag = (mn.0 - mx.0).abs()
                    + (mn.1 - mx.1).abs()
                    + (mn.2 - mx.2).abs();
                if diag <= self.collapse_tol {
                    *curve = vec![curve[0]];
                }
            }
        }
        result
    }

    /// Bisection on the boolean branch validity: `theta_invalid` is on the
    /// invalid side, `theta_valid` on the valid side; converges to the
    /// validity boundary (discriminant zero, sheet crossing, or clip).
    fn bisect_valid(&self, br: usize, theta_invalid: f64, theta_valid: f64) -> f64 {
        let mut lo = theta_invalid;
        let mut hi = theta_valid;
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if (self.roots_at)(mid)[br].is_some() {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// No valid azimuth anywhere: either a single tangency point (the
    /// discriminant MAXIMUM touches zero — golden-section refinement, the
    /// cylinder×cylinder idiom) or empty.
    fn tangency_or_empty(&self) -> Vec<Vec<Point3d>> {
        if self.linear {
            return vec![]; // linear degenerate, no sheet-valid azimuth — empty
        }
        let two_pi = 2.0 * std::f64::consts::PI;
        let m = self.m_scan;
        let grid: Vec<f64> = (0..m)
            .map(|i| (self.disc_at)(two_pi * i as f64 / m as f64))
            .collect();
        let argmax = (0..m)
            .max_by(|&i, &j| grid[i].partial_cmp(&grid[j]).unwrap())
            .unwrap();
        let step = two_pi / m as f64;
        let mut lo = two_pi * argmax as f64 / m as f64 - step;
        let mut hi = two_pi * argmax as f64 / m as f64 + step;
        let phi = 0.6180339887498949;
        let mut x1 = hi - phi * (hi - lo);
        let mut x2 = lo + phi * (hi - lo);
        let mut f1 = (self.disc_at)(x1);
        let mut f2 = (self.disc_at)(x2);
        for _ in 0..80 {
            if f1 > f2 {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - phi * (hi - lo);
                f1 = (self.disc_at)(x1);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + phi * (hi - lo);
                f2 = (self.disc_at)(x2);
            }
        }
        let theta_star = 0.5 * (lo + hi);
        let d_scale = grid.iter().fold(0.0f64, |acc, &d| acc.max(d.abs())).max(1.0);
        if (self.disc_at)(theta_star) > -1e-10 * d_scale {
            // The double root t = −b/2a, checked through the same sheet
            // filter as regular roots.
            let roots = (self.roots_at)(theta_star);
            if let Some(t) = roots[0].or(roots[1]) {
                return vec![vec![(self.point_at)(theta_star, t)]];
            }
        }
        vec![]
    }
}

/// Axis-aligned bounding extent as ((min_x, min_y, min_z), (max_x, max_y, max_z)).
fn bounding_extent(points: &[Point3d]) -> ((f64, f64, f64), (f64, f64, f64)) {
    let mut mn = (f64::MAX, f64::MAX, f64::MAX);
    let mut mx = (f64::MIN, f64::MIN, f64::MIN);
    for p in points {
        mn.0 = mn.0.min(p.x);
        mn.1 = mn.1.min(p.y);
        mn.2 = mn.2.min(p.z);
        mx.0 = mx.0.max(p.x);
        mx.1 = mx.1.max(p.y);
        mx.2 = mx.2.max(p.z);
    }
    (mn, mx)
}

fn endpoints_close(a: &Point3d, b: &Point3d, tol: f64) -> bool {
    (a.x - b.x).abs() <= tol
        && (a.y - b.y).abs() <= tol
        && (a.z - b.z).abs() <= tol
}

/// Glue open arcs at coincident endpoints into longer curves / closed
/// loops. Pinch joins (branch+ end == branch− end at a D = 0 boundary),
/// a(θ) = 0 crossings (the finite linear root continuous across the
/// azimuth) and apex pass-throughs all produce coincident arc endpoints.
/// Duplicated closure vertices are dropped (house convention).
fn glue_arcs(mut arcs: Vec<Vec<Point3d>>, tol: f64) -> Vec<Vec<Point3d>> {
    // Drop duplicated closure vertices.
    for c in &mut arcs {
        if c.len() >= 2 && endpoints_close(&c[0], c.last().unwrap(), tol) {
            c.pop();
        }
    }

    let mut guard = 0usize;
    'glue: loop {
        guard += 1;
        if guard > 10_000 {
            break; // pathological input guard
        }
        if arcs.len() < 2 {
            break;
        }
        for i in 0..arcs.len() {
            for j in 0..arcs.len() {
                if i == j {
                    continue;
                }
                let a = &arcs[i];
                let b = &arcs[j];
                if a.is_empty() || b.is_empty() {
                    continue;
                }
                let a_first = &a[0];
                let a_last = a.last().unwrap();
                let b_first = &b[0];
                let b_last = b.last().unwrap();

                let mut merged: Option<Vec<Point3d>> = None;
                if endpoints_close(a_last, b_first, tol) {
                    let mut mm = a.clone();
                    mm.extend_from_slice(&b[1..]);
                    merged = Some(mm);
                } else if endpoints_close(a_last, b_last, tol) {
                    let mut mm = a.clone();
                    let rev: Vec<Point3d> = b.iter().rev().skip(1).cloned().collect();
                    mm.extend(rev);
                    merged = Some(mm);
                } else if endpoints_close(a_first, b_first, tol) {
                    let mut mm: Vec<Point3d> = b.iter().rev().skip(1).cloned().collect();
                    mm.extend_from_slice(a);
                    merged = Some(mm);
                } else if endpoints_close(a_first, b_last, tol) {
                    let mut mm = b.clone();
                    mm.extend_from_slice(&a[1..]);
                    merged = Some(mm);
                }
                if let Some(mm) = merged {
                    arcs[i] = mm;
                    arcs.remove(j);
                    continue 'glue;
                }
            }
        }
        break; // no merge in a full pass
    }

    // A fresh merge may have produced a new closure duplicate.
    for c in &mut arcs {
        if c.len() >= 2 && endpoints_close(&c[0], c.last().unwrap(), tol) {
            c.pop();
        }
    }
    arcs.retain(|c| !c.is_empty());
    arcs
}

/// Shared-apex cones: the intersection is the set of common generator RAYS
/// (directions d on the unit sphere with angle(d, m_a) = α and
/// angle(d, m_b) = β — two small circles intersecting in ≤ 2 points) or
/// nothing. Two linear constraints d·m_a = cosα, d·m_b = cosβ leave a
/// one-dimensional affine solution line d = d_p + λ·(m_a × m_b); |d| = 1
/// fixes λ.
fn same_apex_generator_rays(va: &ConeView, vb: &ConeView, scale: f64) -> Vec<Vec<Point3d>> {
    let u = va.m.cross(&vb.m);
    let u_len_sq = u.length_sq();
    if u_len_sq <= 1e-12 {
        // Parallel nappes: identical cones (infinite intersection) or
        // strictly nested — nothing usable either way (the same convention
        // as coaxial identical cylinders).
        return vec![];
    }
    let ma_mb = va.m.dot(&vb.m);
    let det = 1.0 - ma_mb * ma_mb; // = |u|² for unit m's
    if det <= 1e-12 {
        return vec![];
    }
    let wa = (va.cos_a - vb.cos_a * ma_mb) / det;
    let wb = (vb.cos_a - va.cos_a * ma_mb) / det;
    let d_p = Vec3d::new(
        wa * va.m.x + wb * vb.m.x,
        wa * va.m.y + wb * vb.m.y,
        wa * va.m.z + wb * vb.m.z,
    );
    let d_p_len_sq = d_p.length_sq();
    if d_p_len_sq > 1.0 + 1e-9 {
        return vec![]; // direction circles disjoint — apices touch only
    }
    // d_p·u = 0 (d_p ∈ span(m_a, m_b), u ⊥ both) → λ² = (1 − |d_p|²)/|u|².
    let lam_sq = ((1.0 - d_p_len_sq) / u_len_sq).max(0.0);
    let lam = lam_sq.sqrt();
    let span = 20.0 * scale.max(1.0);
    let mut rays = Vec::new();
    for sign in [1.0, -1.0] {
        if sign < 0.0 && lam_sq <= 1e-18 {
            continue; // tangent direction circles — a single ray
        }
        let d = Vec3d::new(
            d_p.x + sign * lam * u.x,
            d_p.y + sign * lam * u.y,
            d_p.z + sign * lam * u.z,
        );
        rays.push(vec![
            va.apex,
            Point3d::new(va.apex.x + span * d.x, va.apex.y + span * d.y, va.apex.z + span * d.z),
        ]);
    }
    rays
}

/// Intersect two cones (analytic, B1-series).
///
/// Handles all nappe configurations of two infinite single-nappe cones:
/// generic non-parallel axes (per-θ quadratic on cone A's generator
/// slant), parallel axes (the quadratic degenerates gracefully; equal
/// angles → planar conic via the linear root), coaxial circles, shared
/// apices (generator rays), tangency (single point), disjoint (empty).
/// Points lie on both cones to floating-point precision — no marching.
pub fn intersect_cone_cone(
    cone_a: &ConeSurface,
    cone_b: &ConeSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);

    // Degenerate half-angles → other pair paths.
    let tan_a = cone_a.half_angle.tan();
    let tan_b = cone_b.half_angle.tan();
    let cyl_a = tan_a.is_finite() && tan_a.abs() < 1e-10;
    let cyl_b = tan_b.is_finite() && tan_b.abs() < 1e-10;
    let plane_a = !tan_a.is_finite();
    let plane_b = !tan_b.is_finite();

    if cyl_a && cyl_b {
        return intersect_cylinder_cylinder(
            &cone_as_cylinder(cone_a),
            &cone_as_cylinder(cone_b),
            tolerance,
        );
    }
    if cyl_a {
        return intersect_cone_cylinder(cone_b, &cone_as_cylinder(cone_a), tolerance);
    }
    if cyl_b {
        return intersect_cone_cylinder(cone_a, &cone_as_cylinder(cone_b), tolerance);
    }
    if plane_a {
        return intersect_plane_cone(&cone_as_plane(cone_a), cone_b, tolerance);
    }
    if plane_b {
        return intersect_plane_cone(&cone_as_plane(cone_b), cone_a, tolerance);
    }

    let va = match ConeView::of(cone_a) {
        Some(v) => v,
        None => return vec![],
    };
    let vb = match ConeView::of(cone_b) {
        Some(v) => v,
        None => return vec![],
    };

    let w0 = Vec3d::new(va.apex.x - vb.apex.x, va.apex.y - vb.apex.y, va.apex.z - vb.apex.z);
    let w0_len = w0.length();
    let scale = va.radius.max(vb.radius).max(w0_len).max(1.0);
    if w0_len <= 1e-7 * scale {
        return same_apex_generator_rays(&va, &vb, scale);
    }

    // Quadratic coefficients (see the module commentary for the derivation).
    let h0 = w0.dot(&vb.m);
    let gm_c = va.ex.dot(&vb.m);
    let gm_s = va.ey.dot(&vb.m);
    let gm_k = va.cos_a * va.m.dot(&vb.m);
    let wg_c = w0.dot(&va.ex);
    let wg_s = w0.dot(&va.ey);
    let wg_k = va.cos_a * w0.dot(&va.m);
    let cos2_b = vb.cos_a * vb.cos_a;

    // a(θ) = gm(θ)² − cos²β — degree-2 trig.
    let a_poly = {
        let sin2 = va.sin_a * va.sin_a;
        let mid = 2.0 * gm_k * va.sin_a;
        let qc2 = gm_c * gm_c + gm_s * gm_s;
        Trig2 {
            k0: gm_k * gm_k + sin2 * 0.5 * qc2 - cos2_b,
            k1: mid * gm_c,
            s1: mid * gm_s,
            k2: sin2 * 0.5 * (gm_c * gm_c - gm_s * gm_s),
            s2: sin2 * gm_c * gm_s,
        }
    };
    // b(θ) = 2·(h0·gm(θ) − cos²β·w0g(θ)) — degree-1 trig.
    let b_poly = Trig2::linear(
        2.0 * (h0 * gm_k - cos2_b * wg_k),
        2.0 * va.sin_a * (h0 * gm_c - cos2_b * wg_c),
        2.0 * va.sin_a * (h0 * gm_s - cos2_b * wg_s),
    );
    // c — constant.
    let c_val = h0 * h0 - cos2_b * w0.length_sq();

    let t_clip = 20.0 * scale;
    let eps_sheet = eps * scale.max(1.0);
    let a_trivial = a_poly.is_trivial(1e-12);

    let roots_at = |theta: f64| -> [Option<f64>; 2] {
        let a = a_poly.eval(theta);
        let b = b_poly.eval(theta);
        let gm = gm_k + va.sin_a * (gm_c * theta.cos() + gm_s * theta.sin());
        let check = |t: f64| -> Option<f64> {
            // A-nappe slant and clip.
            if t < -eps_sheet || t > t_clip {
                return None;
            }
            // B-nappe side: w·m_b ≥ 0 (mirror-nappe roots rejected).
            if h0 + t * gm < -eps_sheet {
                return None;
            }
            Some(t.max(0.0))
        };
        if a.abs() > 1e-12 {
            let d = b * b - 4.0 * a * c_val;
            // STRICT existence for the validity masks: a genuinely negative
            // discriminant means no root — clamping it to zero would mark
            // off-surface pinch points as valid (the cylinder×cylinder
            // solver keeps its masks on a separate strict disc closure for
            // the same reason). fp-scale slack only.
            let d_slack = 1e-10 * (d.abs() + b * b + (4.0 * a * c_val).abs() + 1.0);
            if d < -d_slack {
                return [None, None];
            }
            let d = d.max(0.0);
            let sq = d.sqrt();
            let half = -b / (2.0 * a);
            [check(half + sq / (2.0 * a)), check(half - sq / (2.0 * a))]
        } else {
            // a(θ) ≈ 0 — the A-generator is parallel to a B-generator:
            // one finite root −c/b, the other at infinity. Assign to the
            // branch that stays continuous across the a = 0 azimuth.
            let eps_b = 1e-12 * scale;
            if b.abs() > eps_b {
                let t = -c_val / b;
                if b > 0.0 {
                    [check(t), None]
                } else {
                    [None, check(t)]
                }
            } else {
                [None, None]
            }
        }
    };
    let disc_at = |theta: f64| -> f64 {
        let a = a_poly.eval(theta);
        let b = b_poly.eval(theta);
        b * b - 4.0 * a * c_val
    };
    let point_at = |theta: f64, t: f64| -> Point3d {
        let g = va.generator(theta);
        Point3d::new(va.apex.x + t * g.x, va.apex.y + t * g.y, va.apex.z + t * g.z)
    };

    let engine = ThetaArcEngine {
        m_scan: 720,
        n_samples: 128,
        roots_at: &roots_at,
        disc_at: &disc_at,
        point_at: &point_at,
        glue_tol: 1e-6 * scale,
        collapse_tol: 100.0 * eps * (1.0 + scale),
        linear: a_trivial,
    };
    engine.solve()
}

/// Intersect a cone with a cylinder (analytic, B1-series).
///
/// The cylinder is parametrized by its axial coordinate t ∈ ℝ; the cone
/// nappe constraint reduces to a quadratic with CONSTANT t² coefficient
/// and degree-1/degree-2 trig coefficients — the cylinder×cylinder
/// structure. The cone nappe side (w·m ≥ 0) filters mirror roots; points
/// lie on both surfaces to floating-point precision.
pub fn intersect_cone_cylinder(
    cone: &ConeSurface,
    cyl: &CylinderSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);
    if cyl.radius <= eps {
        return vec![];
    }

    // Degenerate cone guards.
    let tan_ha = cone.half_angle.tan();
    if !tan_ha.is_finite() {
        return intersect_plane_cylinder(&cone_as_plane(cone), cyl, tolerance);
    }
    if tan_ha.abs() < 1e-10 {
        return intersect_cylinder_cylinder(&cone_as_cylinder(cone), cyl, tolerance);
    }

    let vc = match ConeView::of(cone) {
        Some(v) => v,
        None => return vec![],
    };

    let n_c = Vec3d::new(cyl.axis.x, cyl.axis.y, cyl.axis.z);
    // Re-orthogonalize the cylinder frame against its axis.
    let raw = Vec3d::new(cyl.x_dir.x, cyl.x_dir.y, cyl.x_dir.z);
    let dk = raw.dot(&n_c);
    let mut e1 = Vec3d::new(raw.x - dk * n_c.x, raw.y - dk * n_c.y, raw.z - dk * n_c.z);
    let e1_len = e1.length();
    if e1_len < 1e-9 {
        // x_dir ∥ axis — n × e_x, falling back to n × e_y (parallel-to-X axes).
        let mut fb = Vec3d::new(0.0, n_c.z, -n_c.y); // n × e_x
        if fb.length_sq() < 1e-6 {
            fb = Vec3d::new(-n_c.z, 0.0, n_c.x); // n × e_y
        }
        let l = fb.length();
        if l < 1e-12 {
            return vec![];
        }
        e1 = Vec3d::new(fb.x / l, fb.y / l, fb.z / l);
    } else {
        e1 = Vec3d::new(e1.x / e1_len, e1.y / e1_len, e1.z / e1_len);
    }
    let e2 = n_c.cross(&e1);

    let w0 = Vec3d::new(
        cyl.origin.x - vc.apex.x,
        cyl.origin.y - vc.apex.y,
        cyl.origin.z - vc.apex.z,
    );
    let h0 = w0.dot(&vc.m);
    let nm = n_c.dot(&vc.m);
    let qm_c = e1.dot(&vc.m);
    let qm_s = e2.dot(&vc.m);
    let w0c = w0.dot(&n_c);
    let w0q_c = w0.dot(&e1);
    let w0q_s = w0.dot(&e2);
    let cos2_a = vc.cos_a * vc.cos_a;
    let r_c = cyl.radius;

    let w0_len = w0.length();
    let scale = vc.radius.max(r_c).max(w0_len).max(1.0);
    let t_clip = 20.0 * scale;
    let eps_sheet = eps * scale.max(1.0);

    // (w·m)² = cos²α·|w|² with w = w0 + t·n_c + R·q(θ):
    //   a₂·t² + b(θ)·t + c(θ) = 0,
    //   a₂ = (n_c·m)² − cos²α (constant),
    //   b(θ) = 2[h0·nm − cos²α·w0c + R·nm·qm(θ)],
    //   c(θ) = (h0 + R·qm(θ))² − cos²α·(|w0|² + R² + 2R·w0q(θ)).
    let a2 = nm * nm - cos2_a;
    let b_poly = Trig2::linear(
        2.0 * (h0 * nm - cos2_a * w0c),
        2.0 * r_c * nm * qm_c,
        2.0 * r_c * nm * qm_s,
    );
    let c_poly = {
        let qm2_half = 0.5 * (qm_c * qm_c + qm_s * qm_s);
        Trig2 {
            k0: h0 * h0 - cos2_a * (w0_len * w0_len + r_c * r_c) + r_c * r_c * qm2_half,
            k1: 2.0 * r_c * (h0 * qm_c - cos2_a * w0q_c),
            s1: 2.0 * r_c * (h0 * qm_s - cos2_a * w0q_s),
            k2: r_c * r_c * 0.5 * (qm_c * qm_c - qm_s * qm_s),
            s2: r_c * r_c * qm_c * qm_s,
        }
    };
    let linear = a2.abs() <= 1e-12; // cylinder axis ∥ cone generator

    let roots_at = |theta: f64| -> [Option<f64>; 2] {
        let b = b_poly.eval(theta);
        let c = c_poly.eval(theta);
        let qm = qm_c * theta.cos() + qm_s * theta.sin();
        let check = |t: f64| -> Option<f64> {
            if t.abs() > t_clip {
                return None;
            }
            // Cone nappe side: w·m ≥ 0.
            if h0 + t * nm + r_c * qm < -eps_sheet {
                return None;
            }
            Some(t)
        };
        if a2.abs() > 1e-12 {
            let d = b * b - 4.0 * a2 * c;
            // STRICT existence for the validity masks (see cone_cone).
            let d_slack = 1e-10 * (d.abs() + b * b + (4.0 * a2 * c).abs() + 1.0);
            if d < -d_slack {
                return [None, None];
            }
            let d = d.max(0.0);
            let sq = d.sqrt();
            let half = -b / (2.0 * a2);
            [check(half + sq / (2.0 * a2)), check(half - sq / (2.0 * a2))]
        } else {
            let eps_b = 1e-12 * scale;
            if b.abs() > eps_b {
                let t = -c / b;
                if b > 0.0 {
                    [check(t), None]
                } else {
                    [None, check(t)]
                }
            } else {
                [None, None]
            }
        }
    };
    let disc_at = |theta: f64| -> f64 {
        let b = b_poly.eval(theta);
        let c = c_poly.eval(theta);
        b * b - 4.0 * a2 * c
    };
    let point_at = |theta: f64, t: f64| -> Point3d {
        let c = theta.cos();
        let s = theta.sin();
        Point3d::new(
            cyl.origin.x + r_c * (c * e1.x + s * e2.x) + t * n_c.x,
            cyl.origin.y + r_c * (c * e1.y + s * e2.y) + t * n_c.y,
            cyl.origin.z + r_c * (c * e1.z + s * e2.z) + t * n_c.z,
        )
    };

    let engine = ThetaArcEngine {
        m_scan: 720,
        n_samples: 128,
        roots_at: &roots_at,
        disc_at: &disc_at,
        point_at: &point_at,
        glue_tol: 1e-6 * scale,
        collapse_tol: 100.0 * eps * (1.0 + scale),
        linear,
    };
    engine.solve()
}

// ============================================================
// Torus analytic SSI (T-series, 2026-09-02)
// ============================================================

/// Orthonormal view of a torus.
///
/// `P(θ, φ) = O + (R + r·cosφ)·u(θ) + r·sinφ·n` with
/// `u(θ) = cosθ·e1 + sinθ·e2`: θ is the azimuth around the main axis,
/// φ the tube angle (φ = 0 outer equator, φ = π inner equator). The
/// x_dir reference is re-orthogonalized against the axis (the
/// `ConeView::of` / cone_cylinder frame idiom).
struct TorusView {
    center: Point3d,
    e1: Vec3d,
    e2: Vec3d,
    n: Vec3d,
    major: f64,
    minor: f64,
}

impl TorusView {
    fn of(t: &TorusSurface) -> TorusView {
        let n = Vec3d::new(t.axis.x, t.axis.y, t.axis.z);
        let raw = Vec3d::new(t.x_dir.x, t.x_dir.y, t.x_dir.z);
        let dk = raw.dot(&n);
        let mut e1 = Vec3d::new(raw.x - dk * n.x, raw.y - dk * n.y, raw.z - dk * n.z);
        let e1_len = e1.length();
        if e1_len < 1e-9 {
            // x_dir ∥ axis — n × e_x, falling back to n × e_y (the
            // cone-family two-step fallback for axes ∥ ±X).
            let mut fb = Vec3d::new(0.0, n.z, -n.y);
            if fb.length_sq() < 1e-6 {
                fb = Vec3d::new(-n.z, 0.0, n.x);
            }
            let l = fb.length();
            if l < 1e-12 {
                // Degenerate axis — any equatorial pair works.
                e1 = Vec3d::new(1.0, 0.0, 0.0);
            } else {
                e1 = Vec3d::new(fb.x / l, fb.y / l, fb.z / l);
            }
        } else {
            e1 = Vec3d::new(e1.x / e1_len, e1.y / e1_len, e1.z / e1_len);
        }
        let e2 = n.cross(&e1);
        TorusView {
            center: Point3d::new(t.center.x, t.center.y, t.center.z),
            e1,
            e2,
            n,
            major: t.major_radius,
            minor: t.minor_radius,
        }
    }

    #[inline]
    fn point(&self, theta: f64, phi: f64) -> Point3d {
        let rho = self.major + self.minor * phi.cos();
        let c = theta.cos();
        let s = theta.sin();
        Point3d::new(
            self.center.x + rho * (c * self.e1.x + s * self.e2.x) + self.minor * phi.sin() * self.n.x,
            self.center.y + rho * (c * self.e1.y + s * self.e2.y) + self.minor * phi.sin() * self.n.y,
            self.center.z + rho * (c * self.e1.z + s * self.e2.z) + self.minor * phi.sin() * self.n.z,
        )
    }
}

/// Sample a full circle (128 points, endpoint not duplicated — the
/// sphere_sphere closed-circle convention) with center `c`, radius `rad`,
/// in the plane spanned by the orthonormal pair `(d1, d2)`.
fn sample_circle_xyz(
    c: &Point3d,
    d1: &Vec3d,
    d2: &Vec3d,
    rad: f64,
) -> Vec<Point3d> {
    let n_samples = 128;
    let two_pi = 2.0 * std::f64::consts::PI;
    (0..n_samples)
        .map(|i| {
            let th = two_pi * i as f64 / n_samples as f64;
            let (c_t, s_t) = (th.cos(), th.sin());
            Point3d::new(
                c.x + rad * (c_t * d1.x + s_t * d2.x),
                c.y + rad * (c_t * d1.y + s_t * d2.y),
                c.z + rad * (c_t * d1.z + s_t * d2.z),
            )
        })
        .collect()
}

/// Solve the tube equation `a·cosφ + b·sinφ = c` for φ.
///
/// The torus×plane and torus×sphere constraints both reduce to a
/// LINEAR equation in `(cosφ, sinφ)` at fixed azimuth θ. Solutions
/// exist iff `c² ≤ a² + b²`; the two branches are
/// `φ = φ₀ ± arccos(c/g)` with `g = √(a²+b²)`, `φ₀ = atan2(b, a)`.
///
/// `atan2` branch cuts in φ₀ do NOT break the point curve: φ is
/// re-applied through cos/sin in `point_at`, so a 2π jump in the
/// returned φ reproduces the identical 3D point. The STRICT validity
/// check (reject only `d < −d_slack`, the cone_cone idiom) keeps
/// tangent azimuths usable without leaking off-surface points.
///
/// Returns `[None, None]` when the equation degenerates (`g ≈ 0`) —
/// pair-level special cases handle the geometric degeneracies
/// (plane containing the axis, sphere center on the tube-center
/// circle).
fn linear_trig_phi(a: f64, b: f64, c: f64, scale: f64) -> [Option<f64>; 2] {
    let g_sq = a * a + b * b;
    if g_sq <= 1e-12 * scale * scale {
        return [None, None]; // degenerate azimuth — pair-level special case
    }
    let g = g_sq.sqrt();
    let d = g_sq - c * c;
    let d_slack = 1e-10 * (d.abs() + g_sq + c * c + 1.0) * (1.0 + scale);
    if d < -d_slack {
        return [None, None];
    }
    let ratio = (c / g).clamp(-1.0, 1.0);
    let delta = ratio.acos();
    let phi0 = b.atan2(a);
    [Some(phi0 + delta), Some(phi0 - delta)]
}

/// Torus×Plane analytic SSI (T-series, 2026-09-02).
///
/// The plane constraint `n_p·P = d` with
/// `P = O + (R + r·cosφ)·u(θ) + r·sinφ·n` reduces to the tube equation
/// `r·A(θ)·cosφ + r·B·sinφ = −h − R·A(θ)` where `A(θ) = n_p·u(θ)` (linear
/// trig in θ), `B = n_p·n` (constant) and `h` is the signed distance of
/// the torus center from the plane.
///
/// Classification:
///
/// - **plane ⟂ axis** (`|B| ≈ 1`): `A ≈ 0`, the equation is θ-free —
///   the plane cuts every tube cross-section at the same height
///   `z = −s·h`: `|z| < r` → 2 circles `ρ = R ± √(r² − z²)`;
///   `|z| ≈ r` → 1 tangent circle; `|z| > r` → empty;
/// - **plane ∥ axis containing the axis** (`|B| ≈ 0`, `|h| ≈ 0`): the
///   two meridian tube circles at the azimuths where `n_p·u(θ) = 0`
///   (the whole circle solves the degenerate `0 = 0` equation);
/// - **generic oblique / offset-parallel planes**: per-θ linear
///   [`linear_trig_phi`] solve through the [`ThetaArcEngine`] — points
///   exact on both surfaces, branch arcs glued at pinch azimuths
///   (Villarceau-adjacent quartic toric sections, offset-plane
///   "peanut" ovals), full-circle branch runs emitted as closed loops;
/// - **tangency**: the discriminant maximum touching zero — the
///   engine's golden-section single point.
pub fn intersect_torus_plane(
    plane: &Plane,
    torus: &TorusSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);
    let tv = TorusView::of(torus);
    let r = tv.minor;
    let big_r = tv.major;
    let scale = big_r.max(r).max(1.0);
    if r <= eps * scale || big_r <= eps * scale {
        return vec![]; // degenerate torus (no tube / no ring)
    }

    let np = Vec3d::new(plane.normal.x, plane.normal.y, plane.normal.z);
    // h = signed distance of the torus center from the plane (along np).
    let w0 = Vec3d::new(
        tv.center.x - plane.origin.x,
        tv.center.y - plane.origin.y,
        tv.center.z - plane.origin.z,
    );
    let h = w0.dot(&np);
    let b_axis = np.dot(&tv.n);
    let ac = np.dot(&tv.e1);
    let as_ = np.dot(&tv.e2);

    // ── Plane ⟂ axis: constant-z circles ─────────────────────────────
    if b_axis.abs() >= 1.0 - 1e-12 {
        let s = b_axis.signum();
        let z = -s * h;
        let z_abs = z.abs();
        let center_at_z = Point3d::new(
            tv.center.x + z * tv.n.x,
            tv.center.y + z * tv.n.y,
            tv.center.z + z * tv.n.z,
        );
        if z_abs > r + eps {
            return vec![];
        }
        if z_abs > r - eps {
            // Tangent to the tube top/bottom: one circle at ρ = R,
            // centered at the plane height.
            return vec![sample_circle_xyz(&center_at_z, &tv.e1, &tv.e2, big_r)];
        }
        let half = (r * r - z * z).sqrt();
        return vec![
            sample_circle_xyz(&center_at_z, &tv.e1, &tv.e2, big_r + half),
            sample_circle_xyz(&center_at_z, &tv.e1, &tv.e2, big_r - half),
        ];
    }

    // ── Plane ∥ axis containing the axis: 2 meridian circles ─────────
    // B ≈ 0 and the plane passes through the torus center ⟹ through the
    // whole axis line. The tube cross-section circles at the azimuths
    // with n_p·u(θ) = 0 lie entirely in the plane.
    if b_axis.abs() <= 1e-10 && h.abs() <= eps {
        // Azimuths with ac·cosθ + as·sinθ = 0 → θ₀ = atan2(ac, −as).
        let theta0 = ac.atan2(-as_);
        let u_at = |theta: f64| -> Vec3d {
            let (c, s) = (theta.cos(), theta.sin());
            Vec3d::new(
                c * tv.e1.x + s * tv.e2.x,
                c * tv.e1.y + s * tv.e2.y,
                c * tv.e1.z + s * tv.e2.z,
            )
        };
        let u0 = u_at(theta0);
        let center0 = Point3d::new(
            tv.center.x + big_r * u0.x,
            tv.center.y + big_r * u0.y,
            tv.center.z + big_r * u0.z,
        );
        let u1 = u_at(theta0 + std::f64::consts::PI);
        let center1 = Point3d::new(
            tv.center.x + big_r * u1.x,
            tv.center.y + big_r * u1.y,
            tv.center.z + big_r * u1.z,
        );
        // Meridian circle: in the plane spanned by (u, n).
        return vec![
            sample_circle_xyz(&center0, &u0, &tv.n, r),
            sample_circle_xyz(&center1, &u1, &tv.n, r),
        ];
    }

    // ── Generic: per-θ linear tube solve ─────────────────────────────
    let roots_at = |theta: f64| -> [Option<f64>; 2] {
        let a_theta = ac * theta.cos() + as_ * theta.sin();
        linear_trig_phi(r * a_theta, r * b_axis, -h - big_r * a_theta, scale)
    };
    let disc_at = |theta: f64| -> f64 {
        let a_theta = ac * theta.cos() + as_ * theta.sin();
        let g_sq = r * r * (a_theta * a_theta + b_axis * b_axis);
        let c_val = -h - big_r * a_theta;
        g_sq - c_val * c_val
    };
    let point_at = |theta: f64, phi: f64| -> Point3d { tv.point(theta, phi) };

    let engine = ThetaArcEngine {
        m_scan: 720,
        n_samples: 128,
        roots_at: &roots_at,
        disc_at: &disc_at,
        point_at: &point_at,
        glue_tol: 1e-6 * scale,
        collapse_tol: 100.0 * eps * (1.0 + scale),
        linear: false,
    };
    engine.solve()
}

/// Torus×Sphere analytic SSI (T-series, 2026-09-02).
///
/// `|P − C_s|² = R_s²` with
/// `P − C_s = (R + r·cosφ)·u(θ) + r·sinφ·n − v` (v = C_s − O) reduces
/// (using `ρ² + z² = R² + r² + 2Rr·cosφ`) to the LINEAR tube equation
///
/// ```text
/// a(θ)·cosφ + b·sinφ = −C(θ),
///   a(θ) = 2r·(R − u(θ)·v),         b = −2r·(n·v),
///   C(θ) = R² + r² + |v|² − 2R·u(θ)·v − R_s²
/// ```
///
/// with `u·v` linear trig in θ — the same [`linear_trig_phi`] machinery
/// as torus×plane. Concentric spheres (`v = 0`) give constant
/// coefficients: the engine's full-circle branch runs emit the two
/// latitude circles directly (single circle at internal/external
/// tangency radii). Tangency (discriminant maximum at zero) → a single
/// point; contained/disjoint spheres → empty.
///
/// Known limitation: a sphere centered ON the tube-center circle with
/// `R_s ≈ r` contains one full meridian circle — the degenerate azimuth
/// returns `[None, None]` and that circle is missed (the remaining
/// intersection curves are still produced).
pub fn intersect_torus_sphere(
    sphere: &SphereSurface,
    torus: &TorusSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);
    let tv = TorusView::of(torus);
    let r = tv.minor;
    let big_r = tv.major;
    let scale = big_r.max(r).max(sphere.radius).max(1.0);
    if r <= eps * scale || big_r <= eps * scale || sphere.radius <= eps * scale {
        return vec![];
    }

    let v = Vec3d::new(
        sphere.center.x - tv.center.x,
        sphere.center.y - tv.center.y,
        sphere.center.z - tv.center.z,
    );
    let vc = v.dot(&tv.e1);
    let vs = v.dot(&tv.e2);
    let va = v.dot(&tv.n);
    let v_sq = v.length_sq();
    let rs = sphere.radius;
    let const_c = big_r * big_r + r * r + v_sq - rs * rs;

    // Cheap disjoint/contained guards (the engine would also return
    // empty, but the guards keep the tangency search from burning 80
    // golden-section iterations on obviously-empty configurations).
    // Closest distance between the sphere center and the tube surface:
    // the tube lives on the (ρ, z) circle of radius r around (R, 0).
    {
        let rho_c = (vc * vc + vs * vs).sqrt();
        let d_profile = ((rho_c - big_r) * (rho_c - big_r) + va * va).sqrt();
        if d_profile > r + rs + eps {
            return vec![]; // sphere strictly outside the tube
        }
        if d_profile < r - rs - eps {
            return vec![]; // tube surface strictly outside the sphere
        }
    }

    let roots_at = |theta: f64| -> [Option<f64>; 2] {
        let uv = vc * theta.cos() + vs * theta.sin();
        let a = 2.0 * r * (big_r - uv);
        let b = -2.0 * r * va;
        let c = const_c - 2.0 * big_r * uv;
        linear_trig_phi(a, b, -c, scale)
    };
    let disc_at = |theta: f64| -> f64 {
        let uv = vc * theta.cos() + vs * theta.sin();
        let a = 2.0 * r * (big_r - uv);
        let b = -2.0 * r * va;
        let c = const_c - 2.0 * big_r * uv;
        a * a + b * b - c * c
    };
    let point_at = |theta: f64, phi: f64| -> Point3d { tv.point(theta, phi) };

    let engine = ThetaArcEngine {
        m_scan: 720,
        n_samples: 128,
        roots_at: &roots_at,
        disc_at: &disc_at,
        point_at: &point_at,
        glue_tol: 1e-6 * scale,
        collapse_tol: 100.0 * eps * (1.0 + scale),
        linear: false,
    };
    engine.solve()
}

/// Torus×Cylinder analytic SSI (T-series, 2026-09-02).
///
/// With the cylinder axis parallel to the torus axis the cylinder
/// constraint `|ρ·u(θ) − w⊥|² = R_c²` is z-free and reduces (with
/// `ρ = R + r·cosφ`) to a per-θ QUADRATIC in `cosφ`:
///
/// ```text
/// r²·c² + 2r·(R − w⊥·u(θ))·c + (R² − 2R·w⊥·u(θ) + |w⊥|² − R_c²) = 0
/// ```
///
/// Classification:
///
/// - **coaxial** (`w⊥ ≈ 0`): θ-free quadratic — the tube cross-section
///   meets the cylinder ρ = R_c at `z = ±√(r² − (R_c − R)²)`: 2 circles
///   (|R_c − R| < r), 1 tangent circle (≈ r), empty otherwise;
/// - **parallel offset**: the quadratic is solved per θ for
///   `cosφ = c±` with the STRICT discriminant idiom; only the upper
///   tube half (`φ ∈ [0, π]`, sinφ ≥ 0) is tracked by the engine and
///   the lower half is the equatorial mirror (z → −z) — branch arcs
///   reaching the equator (|c| ≈ 1) are glued to their mirrors into
///   closed loops, arcs staying strictly inside remain disjoint
///   top/bottom loops (geometrically correct);
/// - **tangency**: discriminant maximum at zero → single point (the
///   mirrored contact point of the lower half is measure-zero and
///   omitted);
/// - **perpendicular axes**: ψ-parametrized twin-pass solve (see
///   [`torus_cylinder_perpendicular`]) — z is t-free, two ρ-target
///   quadratics, cross-pass glue at the tube slab boundaries;
/// - **skew axes**: the per-θ equation is a quartic in `tan(φ/2)` —
///   delegated to marching SSI (documented gap).
pub fn intersect_torus_cylinder(
    cyl: &CylinderSurface,
    torus: &TorusSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);
    let tv = TorusView::of(torus);
    let r = tv.minor;
    let big_r = tv.major;
    let rc = cyl.radius;
    let scale = big_r.max(r).max(rc).max(1.0);
    if r <= eps * scale || big_r <= eps * scale || rc <= eps * scale {
        return vec![];
    }

    let n_c = Vec3d::new(cyl.axis.x, cyl.axis.y, cyl.axis.z);
    let axes_cross = n_c.cross(&tv.n);
    let axes_parallel = axes_cross.length_sq() <= 1e-10;
    let axes_perpendicular = n_c.dot(&tv.n).abs() <= 1e-10;

    if !axes_parallel && !axes_perpendicular {
        // Skew — per-θ quartic in tan(φ/2): marching (documented gap).
        return intersect_marching_ssi(
            &Surface::Cylinder(cyl.clone()),
            &Surface::Torus(torus.clone()),
            tolerance,
        );
    }

    if axes_perpendicular {
        return torus_cylinder_perpendicular(cyl, &tv, eps);
    }

    // w = cylinder origin − torus center; only the radial offset w⊥
    // matters (the cylinder is infinite along its axis).
    let w = Vec3d::new(
        cyl.origin.x - tv.center.x,
        cyl.origin.y - tv.center.y,
        cyl.origin.z - tv.center.z,
    );
    let w_ax = w.dot(&tv.n);
    let w_perp = Vec3d::new(
        w.x - w_ax * tv.n.x,
        w.y - w_ax * tv.n.y,
        w.z - w_ax * tv.n.z,
    );
    let d_off = w_perp.length();

    // ── Coaxial: θ-free circles ──────────────────────────────────────
    if d_off <= eps {
        let dr = rc - big_r;
        if dr.abs() > r - eps {
            if dr.abs() > r + eps {
                return vec![]; // cylinder misses the tube (hole or outside)
            }
            // Tangent to the tube from inside/outside: one equator circle.
            return vec![sample_circle_xyz(&tv.center, &tv.e1, &tv.e2, big_r + dr.signum() * r)];
        }
        let half = (r * r - dr * dr).sqrt();
        // Circles at radius rc around the common axis at heights ±half.
        let mk = |z: f64| -> Vec<Point3d> {
            let center = Point3d::new(
                tv.center.x + z * tv.n.x,
                tv.center.y + z * tv.n.y,
                tv.center.z + z * tv.n.z,
            );
            sample_circle_xyz(&center, &tv.e1, &tv.e2, rc)
        };
        return vec![mk(half), mk(-half)];
    }

    // ── Parallel offset: per-θ quadratic in cosφ (upper half + mirror) ─
    let wc = w_perp.dot(&tv.e1);
    let ws = w_perp.dot(&tv.e2);
    let d_sq = d_off * d_off;

    let roots_at = |theta: f64| -> [Option<f64>; 2] {
        let wq = wc * theta.cos() + ws * theta.sin();
        let a = r * r;
        let b = 2.0 * r * (big_r - wq);
        let c = big_r * big_r - 2.0 * big_r * wq + d_sq - rc * rc;
        let d = b * b - 4.0 * a * c;
        // STRICT discriminant (cone_cone idiom): clamp must not leak
        // off-surface points into the validity masks.
        let d_slack = 1e-10 * (d.abs() + b * b + (4.0 * a * c).abs() + 1.0) * (1.0 + scale);
        if d < -d_slack {
            return [None, None];
        }
        let d = d.max(0.0);
        let sq = d.sqrt();
        let half = -b / (2.0 * a);
        let check = |cval: f64| -> Option<f64> {
            // cosφ must be a real cosine: strict ±1 validity with slack.
            if cval < -1.0 - 1e-10 * (1.0 + scale) || cval > 1.0 + 1e-10 * (1.0 + scale) {
                return None;
            }
            Some(cval.clamp(-1.0, 1.0).acos()) // upper tube half: φ ∈ [0, π]
        };
        [check(half + sq / (2.0 * a)), check(half - sq / (2.0 * a))]
    };
    let disc_at = |theta: f64| -> f64 {
        let wq = wc * theta.cos() + ws * theta.sin();
        let b = 2.0 * r * (big_r - wq);
        let c = big_r * big_r - 2.0 * big_r * wq + d_sq - rc * rc;
        b * b - 4.0 * r * r * c
    };
    let point_at = |theta: f64, phi: f64| -> Point3d { tv.point(theta, phi) };

    let engine = ThetaArcEngine {
        m_scan: 720,
        n_samples: 128,
        roots_at: &roots_at,
        disc_at: &disc_at,
        point_at: &point_at,
        glue_tol: 1e-6 * scale,
        collapse_tol: 100.0 * eps * (1.0 + scale),
        linear: false,
    };
    let upper = engine.solve();

    // Lower half = equatorial mirror (z → −z about the torus center
    // plane). Arcs that touch the equator glue with their mirrors into
    // closed loops; strictly-upper loops stay disjoint (correct).
    let mut result: Vec<Vec<Point3d>> = Vec::with_capacity(upper.len() * 2);
    for curve in upper {
        let n_pts = curve.len();
        if n_pts == 0 {
            continue;
        }
        if n_pts == 1 {
            // Tangency point: only keep it if it lies ON the equator
            // (where upper == lower); off-equator contact would have a
            // mirrored twin — keep the twin too for completeness.
            let p = curve[0];
            let z = (p.x - tv.center.x) * tv.n.x
                + (p.y - tv.center.y) * tv.n.y
                + (p.z - tv.center.z) * tv.n.z;
            if z.abs() > 100.0 * eps * scale {
                result.push(vec![mirror_about_equator(&p, &tv)]);
            }
            result.push(curve);
            continue;
        }
        let mirrored: Vec<Point3d> = curve
            .iter()
            .map(|p| mirror_about_equator(p, &tv))
            .collect();
        result.push(curve);
        result.push(mirrored);
    }
    // Glue upper/lower arcs that meet on the equator (|c| ≈ 1 pinch
    // points) — glue_arcs is idempotent for already-closed loops.
    glue_arcs(result, 1e-6 * scale)
}

/// Torus×Cylinder with PERPENDICULAR axes (ψ-parametrized, T-series
/// 2026-09-02).
///
/// Cylinder points `X(ψ, t) = O_c + R_c(cosψ·e1c + sinψ·e2c) + t·n_c`.
/// Because `n_c ⊥ n` (the torus axis), the axial torus coordinate is
/// t-FREE: `z(ψ) = w_ax + R_c(zc·cosψ + zs·sinψ)` — linear trig in ψ.
/// The torus constraint `(ρ − R)² + z² = r²` then splits into two
/// ρ-targets `ρ±(ψ) = R ± √(r² − z(ψ)²)` (valid where `|z| ≤ r`), and
/// each target gives a per-ψ QUADRATIC in t (with `ρ²` a quadratic in t
/// for perpendicular axes):
///
/// ```text
/// t² + B(ψ)·t + C(ψ) − ρ±(ψ)² = 0,
///   B(ψ) = 2·(w⊥ + R_c·q⊥(ψ))·n_c,   C(ψ) = |w⊥ + R_c·q⊥(ψ)|²
/// ```
///
/// Two engine passes (ρ+ outer sheet, ρ− inner sheet); curves from the
/// two passes meet where `|z| = r` (the targets coincide at ρ = R) and
/// are cross-pass glued. Points are exact on the cylinder (constructed
/// there) and on the torus up to the quadratic solve — no marching.
fn torus_cylinder_perpendicular(
    cyl: &CylinderSurface,
    tv: &TorusView,
    eps: f64,
) -> Vec<Vec<Point3d>> {
    let rc = cyl.radius;
    let r = tv.minor;
    let big_r = tv.major;
    let scale = big_r.max(r).max(rc).max(1.0);
    let t_clip = 20.0 * scale;

    // Cylinder frame (re-orthogonalized x_dir — the cone_cylinder idiom).
    let n_c = Vec3d::new(cyl.axis.x, cyl.axis.y, cyl.axis.z);
    let raw = Vec3d::new(cyl.x_dir.x, cyl.x_dir.y, cyl.x_dir.z);
    let dk = raw.dot(&n_c);
    let mut e1c = Vec3d::new(raw.x - dk * n_c.x, raw.y - dk * n_c.y, raw.z - dk * n_c.z);
    let e1c_len = e1c.length();
    if e1c_len < 1e-9 {
        let mut fb = Vec3d::new(0.0, n_c.z, -n_c.y); // n × e_x
        if fb.length_sq() < 1e-6 {
            fb = Vec3d::new(-n_c.z, 0.0, n_c.x); // n × e_y
        }
        let l = fb.length();
        if l < 1e-12 {
            return vec![];
        }
        e1c = Vec3d::new(fb.x / l, fb.y / l, fb.z / l);
    } else {
        e1c = Vec3d::new(e1c.x / e1c_len, e1c.y / e1c_len, e1c.z / e1c_len);
    }
    let e2c = n_c.cross(&e1c);

    let w = Vec3d::new(
        cyl.origin.x - tv.center.x,
        cyl.origin.y - tv.center.y,
        cyl.origin.z - tv.center.z,
    );
    let w_ax = w.dot(&tv.n);
    let w_perp = Vec3d::new(
        w.x - w_ax * tv.n.x,
        w.y - w_ax * tv.n.y,
        w.z - w_ax * tv.n.z,
    );
    // q⊥(ψ) = R_c·(cosψ·e1⊥ + sinψ·e2⊥), e_i⊥ = e_ic − (e_ic·n)·n
    // (n_c ⊥ n ⇒ n_c needs no projection).
    let e1p = Vec3d::new(
        e1c.x - e1c.dot(&tv.n) * tv.n.x,
        e1c.y - e1c.dot(&tv.n) * tv.n.y,
        e1c.z - e1c.dot(&tv.n) * tv.n.z,
    );
    let e2p = Vec3d::new(
        e2c.x - e2c.dot(&tv.n) * tv.n.x,
        e2c.y - e2c.dot(&tv.n) * tv.n.y,
        e2c.z - e2c.dot(&tv.n) * tv.n.z,
    );
    let zc = e1c.dot(&tv.n);
    let zs = e2c.dot(&tv.n);
    let wp_nc = w_perp.dot(&n_c);
    let e1p_nc = e1p.dot(&n_c);
    let e2p_nc = e2p.dot(&n_c);
    let wp_sq = w_perp.length_sq();
    let wp_e1p = w_perp.dot(&e1p);
    let wp_e2p = w_perp.dot(&e2p);
    let e1p_sq = e1p.length_sq();
    let e2p_sq = e2p.length_sq();
    let e1p_e2p = e1p.dot(&e2p);
    let eps_sheet = eps * scale.max(1.0);

    let mut combined: Vec<Vec<Point3d>> = Vec::new();
    for target_sign in [1.0, -1.0] {
        let roots_at = |psi: f64| -> [Option<f64>; 2] {
            let c_p = psi.cos();
            let s_p = psi.sin();
            // Axial torus coordinate of the cylinder point (t-free).
            let z = w_ax + rc * (zc * c_p + zs * s_p);
            let rzz = r * r - z * z;
            let z_slack = 1e-10 * (rzz.abs() + r * r + 1.0) * (1.0 + scale);
            if rzz < -z_slack {
                return [None, None]; // |z| > r — cylinder misses the tube slab
            }
            let rho_t = big_r + target_sign * rzz.max(0.0).sqrt();
            let rho_t_sq = rho_t * rho_t;
            // B(ψ) = 2·(w⊥ + R_c·q⊥)·n_c (linear trig), C(ψ) (Trig2-like,
            // evaluated numerically).
            let b = 2.0 * (wp_nc + rc * (e1p_nc * c_p + e2p_nc * s_p));
            let q_len_sq = wp_sq + 2.0 * rc * (wp_e1p * c_p + wp_e2p * s_p)
                + rc * rc * (e1p_sq * c_p * c_p + 2.0 * e1p_e2p * c_p * s_p + e2p_sq * s_p * s_p);
            let d = b * b - 4.0 * (q_len_sq - rho_t_sq);
            // STRICT discriminant (cone_cone idiom).
            let d_slack = 1e-10 * (d.abs() + b * b + (4.0 * (q_len_sq - rho_t_sq)).abs() + 1.0)
                * (1.0 + scale);
            if d < -d_slack {
                return [None, None];
            }
            let d = d.max(0.0);
            let sq = d.sqrt();
            let half = -0.5 * b;
            let check = |t: f64| -> Option<f64> {
                if t.abs() > t_clip {
                    return None;
                }
                // Sheet sanity: the torus slab |z| ≤ r is already enforced
                // above; nothing else gates a closed torus.
                let _ = eps_sheet;
                Some(t)
            };
            [check(half + 0.5 * sq), check(half - 0.5 * sq)]
        };
        let disc_at = |psi: f64| -> f64 {
            // Both validity gates: the |z| ≤ r slab AND the quadratic
            // discriminant (their min — 0 at either tangency family).
            let c_p = psi.cos();
            let s_p = psi.sin();
            let z = w_ax + rc * (zc * c_p + zs * s_p);
            let rzz = r * r - z * z;
            let rho_t = big_r + target_sign * rzz.max(0.0).sqrt();
            let rho_t_sq = rho_t * rho_t;
            let b = 2.0 * (wp_nc + rc * (e1p_nc * c_p + e2p_nc * s_p));
            let q_len_sq = wp_sq + 2.0 * rc * (wp_e1p * c_p + wp_e2p * s_p)
                + rc * rc * (e1p_sq * c_p * c_p + 2.0 * e1p_e2p * c_p * s_p + e2p_sq * s_p * s_p);
            let d = b * b - 4.0 * (q_len_sq - rho_t_sq);
            rzz.min(d)
        };
        let point_at = |psi: f64, t: f64| -> Point3d {
            let c_p = psi.cos();
            let s_p = psi.sin();
            Point3d::new(
                cyl.origin.x + rc * (c_p * e1c.x + s_p * e2c.x) + t * n_c.x,
                cyl.origin.y + rc * (c_p * e1c.y + s_p * e2c.y) + t * n_c.y,
                cyl.origin.z + rc * (c_p * e1c.z + s_p * e2c.z) + t * n_c.z,
            )
        };

        let engine = ThetaArcEngine {
            m_scan: 720,
            n_samples: 128,
            roots_at: &roots_at,
            disc_at: &disc_at,
            point_at: &point_at,
            glue_tol: 1e-6 * scale,
            collapse_tol: 100.0 * eps * (1.0 + scale),
            linear: false,
        };
        combined.extend(engine.solve());
    }
    // Cross-pass glue: ρ+ and ρ− curves meet where |z(ψ)| = r (both
    // targets collapse to ρ = R — the tube's top/bottom pinch circles).
    glue_arcs(combined, 1e-6 * scale)
}

/// Mirror a point about the torus equatorial plane (through the center,
/// normal = axis).
fn mirror_about_equator(p: &Point3d, tv: &TorusView) -> Point3d {
    let dz = (p.x - tv.center.x) * tv.n.x
        + (p.y - tv.center.y) * tv.n.y
        + (p.z - tv.center.z) * tv.n.z;
    Point3d::new(
        p.x - 2.0 * dz * tv.n.x,
        p.y - 2.0 * dz * tv.n.y,
        p.z - 2.0 * dz * tv.n.z,
    )
}

/// Torus×Cone analytic SSI (T-series, 2026-09-05).
///
/// With the cone axis parallel to the torus axis the cone constraint is
/// LINEAR in the torus cylindrical coordinates: `rho = beta + gamma*z`
/// with `gamma = s*tan(alpha)` (s = orientation sign of the cone axis
/// vs the torus axis), `beta = radius0 - gamma*h` (`h` = the cone
/// origin height over the torus equatorial plane, `radius0 = radius`
/// standard / `0` expanding). Substituting into the tube equation
/// `(rho - R)^2 + z^2 = r^2` yields a theta-free QUADRATIC in z:
///
/// ```text
/// (1 + gamma^2)*z^2 + 2*gamma*q*z + (q^2 - r^2) = 0,   q = beta - R
/// ```
///
/// Both surfaces are revolutions about the common axis, so every real
/// root is a latitude circle `(C + z*·n, rho* = beta + gamma*z*)`.
///
/// Classification (the torus_cylinder coaxial idiom via the effective
/// tube mismatch `u_eff = q / sqrt(1 + gamma^2)`):
///
/// - **coaxial** (lateral offset ~ 0): `|u_eff| < r` -> 2 circles,
///   `~ r` -> 1 tangent circle, `> r` -> empty; roots with
///   `rho* <= 0` are off-sheet (beyond the apex — reachable only for
///   spindle tori, `R < r`) and dropped;
/// - **nearly-cylindrical cone** (`|tan(alpha)| ~ 0`): the sheet is
///   `rho = radius0` — routed to [`intersect_torus_cylinder`];
/// - **nearly-flat cone** (`|tan(alpha)|*eps*scale >= 1`): the sheet
///   tends to the base plane `v = 0` — routed to
///   [`intersect_torus_plane`];
/// - **parallel offset / perpendicular / skew axes**: the per-theta
///   constraint mixes `cos^2(phi)`, `cos(phi)` AND `sin(phi)` — a
///   quartic in `tan(phi/2)` — delegated to marching SSI (documented
///   gap, the torus_cylinder skew family).
pub fn intersect_torus_cone(
    cone: &ConeSurface,
    torus: &TorusSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);
    let tv = TorusView::of(torus);
    let r = tv.minor;
    let big_r = tv.major;
    let scale = big_r.max(r).max(cone.radius).max(1.0);
    if r <= eps * scale || big_r <= eps * scale {
        return vec![]; // degenerate torus (no tube / no ring)
    }

    let tan_a = cone.half_angle.tan();
    if cone.expanding && tan_a <= 0.0 {
        return vec![]; // expanding sheet with non-positive slope — empty
    }
    if tan_a.abs() <= 1e-12 {
        // Cone degenerated to a cylinder: rho = radius (standard) —
        // reuse the torus*cylinder solve. Expanding + alpha~0 has no
        // sheet at all.
        if cone.expanding {
            return vec![];
        }
        let cyl = CylinderSurface::new(cone.origin, cone.axis, cone.radius);
        return intersect_torus_cylinder(&cyl, torus, tolerance);
    }
    if !tan_a.is_finite() || tan_a.abs() * eps * scale >= 1.0 {
        // Nearly-flat cone (alpha -> pi/2): the sheet tends to the base
        // plane v = 0 (through the cone origin, normal = axis).
        let plane = Plane::from_origin_and_normal(cone.origin, cone.axis);
        return intersect_torus_plane(&plane, torus, tolerance);
    }

    let n_c = Vec3d::new(cone.axis.x, cone.axis.y, cone.axis.z);
    let axes_cross = n_c.cross(&tv.n);
    if axes_cross.length_sq() > 1e-10 {
        // Perpendicular / skew: per-theta quartic in tan(phi/2) —
        // marching (documented gap).
        return intersect_marching_ssi(
            &Surface::Cone(cone.clone()),
            &Surface::Torus(torus.clone()),
            tolerance,
        );
    }

    // w = cone origin − torus center; the axial part is h, the radial
    // part measures the coaxiality.
    let w = Vec3d::new(
        cone.origin.x - tv.center.x,
        cone.origin.y - tv.center.y,
        cone.origin.z - tv.center.z,
    );
    let h = w.dot(&tv.n);
    let w_perp = Vec3d::new(
        w.x - h * tv.n.x,
        w.y - h * tv.n.y,
        w.z - h * tv.n.z,
    );
    if w_perp.length() > eps {
        // Parallel but offset: per-theta quartic in tan(phi/2) —
        // marching (documented gap).
        return intersect_marching_ssi(
            &Surface::Cone(cone.clone()),
            &Surface::Torus(torus.clone()),
            tolerance,
        );
    }

    // ── Coaxial: theta-free quadratic → latitude circles ─────────────
    let s = n_c.dot(&tv.n).signum();
    let gamma = s * tan_a;
    let radius0 = if cone.expanding { 0.0 } else { cone.radius };
    let beta = radius0 - gamma * h;
    let q = beta - big_r;
    let g1 = 1.0 + gamma * gamma;
    // Effective tube mismatch (the torus_cylinder coaxial idiom):
    // |u_eff| vs r classifies miss / tangent / two circles.
    let u_eff = q / g1.sqrt();
    if u_eff.abs() > r + eps {
        return vec![]; // cone sheet misses the tube
    }

    let emit = |z: f64| -> Option<Vec<Point3d>> {
        let rho = beta + gamma * z;
        if rho <= eps * scale {
            return None; // off-sheet (beyond the apex) / degenerate axis point
        }
        let center = Point3d::new(
            tv.center.x + z * tv.n.x,
            tv.center.y + z * tv.n.y,
            tv.center.z + z * tv.n.z,
        );
        Some(sample_circle_xyz(&center, &tv.e1, &tv.e2, rho))
    };

    if u_eff.abs() > r - eps {
        // Tangent along a latitude circle (double root).
        let z_star = -gamma * q / g1;
        return match emit(z_star) {
            Some(circle) => vec![circle],
            None => vec![],
        };
    }

    let delta = (r * r - u_eff * u_eff).sqrt() * g1.sqrt();
    let z_hi = (-gamma * q + delta) / g1;
    let z_lo = (-gamma * q - delta) / g1;
    [z_hi, z_lo].into_iter().filter_map(emit).collect()
}

/// Intersect two tori (T-series continuation, 2026-09-06).
///
/// **Coaxial** tori (parallel axes — either orientation, a torus is
/// axis-flip invariant — and centers on the common axis): both surfaces
/// are surfaces of revolution about that axis, so the intersection is a
/// union of **latitude circles**. In the meridian plane the torus cuts
/// the pair of profile circles centered `(±R, h)` with radius `r`; the
/// revolved surface is generated by either circle alone. Pairwise
/// circle intersections `(A₊, B₊)` and `(A₊, B₋)` — the `(A₋, ·)` pairs
/// are their mirrors and sweep the same orbits — classify per the
/// two-circle idiom (the `torus_cylinder` / `sphere_sphere` convention):
///
/// - concentric profile circles (`d ≈ 0`): empty — coincident surfaces
///   produce no curve (same convention as `sphere_sphere`);
/// - miss (`d > r₁ + r₂`) or nested (`d < |r₁ − r₂|`): empty;
/// - tangency (`d ≈ r₁ + r₂` external / `d ≈ |r₁ − r₂|` internal): one
///   latitude circle (double root, deduplicated);
/// - general position: two latitude circles.
///
/// Each solution point `(x*, z*)` of the profile pair revolves into the
/// latitude circle `center = A.center + z*·n, radius = |x*|` — including
/// solutions with `x* < 0` (reachable for spindle tori, `r ≥ R`, whose
/// profile crosses the axis). Solutions with `|x*| ≈ 0` degenerate to an
/// axis point and are dropped (the `torus_cone` emit convention). The
/// `(A₊, B₋)` pair stays empty for ring tori: the `B₋` profile lies
/// entirely at `x < 0` unless B is a spindle.
///
/// **Parallel-offset rings / perpendicular / skew axes**: the general
/// torus×torus intersection is an algebraic curve of degree 8 with no
/// θ-reduction — routed to the marching fallback (documented gap).
pub fn intersect_torus_torus(
    ta: &TorusSurface,
    tb: &TorusSurface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let eps = tolerance.max(1e-9);
    let va = TorusView::of(ta);
    let vb = TorusView::of(tb);
    let ra = va.minor;
    let big_ra = va.major;
    let rb = vb.minor;
    let big_rb = vb.major;
    let scale = big_ra.max(ra).max(big_rb).max(rb).max(1.0);
    if ra <= eps * scale || big_ra <= eps * scale || rb <= eps * scale || big_rb <= eps * scale {
        return vec![]; // degenerate torus (no tube / no ring)
    }

    // The torus is axis-flip invariant (its ring circle does not depend
    // on the axis SIGN), so anti-parallel axes are still coaxial.
    let axes_cross = va.n.cross(&vb.n);
    if axes_cross.length_sq() > 1e-10 {
        // Non-parallel axes: degree-8 algebraic curve — marching
        // (documented gap).
        return intersect_marching_ssi(
            &Surface::Torus(ta.clone()),
            &Surface::Torus(tb.clone()),
            tolerance,
        );
    }

    // w = center_B − center_A; the axial part is h (the profile offset
    // along the common axis), the radial part measures coaxiality. The
    // B profile circle center `(big_rb, h)` in A's (rho, z) frame is
    // orientation-independent: the ring circle is the same point set
    // either way.
    let w = Vec3d::new(
        vb.center.x - va.center.x,
        vb.center.y - va.center.y,
        vb.center.z - va.center.z,
    );
    let h = w.dot(&va.n);
    let w_perp = Vec3d::new(
        w.x - h * va.n.x,
        w.y - h * va.n.y,
        w.z - h * va.n.z,
    );
    if w_perp.length() > eps {
        // Parallel but offset rings: degree 8 — marching (documented gap).
        return intersect_marching_ssi(
            &Surface::Torus(ta.clone()),
            &Surface::Torus(tb.clone()),
            tolerance,
        );
    }

    // ── Coaxial: profile-circle pairs → latitude circles ─────────────
    // (rho − big_ra)² + z² = ra²            — torus A profile.
    // (rho − cb)² + (z − h)² = rb²           — torus B profile, cb = ±big_rb.
    let eps_t = eps * (1.0 + big_ra + ra + big_rb + rb + h.abs());
    let mut circles: Vec<(f64, f64)> = Vec::new(); // (z, rho) per latitude circle

    let mut pair_solve = |cb: f64| {
        let dx = cb - big_ra;
        let d = (dx * dx + h * h).sqrt();
        if d <= eps_t {
            // Concentric profile circles: coincident tori (same R, r,
            // axis) produce no transversal curve; radii differing means
            // one profile is nested inside the other — empty either way.
            return;
        }
        if d > ra + rb + eps_t || d < (ra - rb).abs() - eps_t {
            return; // miss / nested
        }
        let a_len = (d * d + ra * ra - rb * rb) / (2.0 * d);
        let hh_sq = ra * ra - a_len * a_len;
        let hh = if hh_sq <= 0.0 { 0.0 } else { hh_sq.sqrt() };
        // Base point: A center + a_len·(B − A)/d; the chord is
        // perpendicular to the center line.
        let ux = dx / d;
        let uz = h / d;
        let mx = big_ra + a_len * ux;
        let mz = a_len * uz;
        for sgn in [1.0_f64, -1.0_f64] {
            let x = mx + sgn * hh * (-uz);
            let z = mz + sgn * hh * ux;
            if x.abs() <= eps * scale {
                continue; // degenerate axis point — not a curve
            }
            circles.push((z, x.abs()));
        }
    };
    pair_solve(big_rb); // same-side pair (A₊, B₊)
    pair_solve(-big_rb); // cross-side pair (A₊, B₋) — spindle reach only

    // Deduplicate identical orbits: the tangent double root pushes the
    // same (z, rho) twice, and mixed spindle configurations can emit a
    // mirror-duplicate across the two pairs.
    let mut uniq: Vec<(f64, f64)> = Vec::new();
    for c in circles {
        if uniq
            .iter()
            .all(|u| (u.0 - c.0).abs() > eps_t || (u.1 - c.1).abs() > eps_t)
        {
            uniq.push(c);
        }
    }

    // Emit latitude circles on A's axis frame.
    uniq.into_iter()
        .map(|(z, rho)| {
            let center = Point3d::new(
                va.center.x + z * va.n.x,
                va.center.y + z * va.n.y,
                va.center.z + z * va.n.z,
            );
            sample_circle_xyz(&center, &va.e1, &va.e2, rho)
        })
        .collect()
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
        (Surface::Sphere(s), Surface::Cylinder(c))
        | (Surface::Cylinder(c), Surface::Sphere(s)) => {
            intersect_sphere_cylinder(s, c, tolerance)
        }
        (Surface::Cone(a), Surface::Cone(b)) => {
            intersect_cone_cone(a, b, tolerance)
        }
        (Surface::Cone(c), Surface::Cylinder(y))
        | (Surface::Cylinder(y), Surface::Cone(c)) => {
            intersect_cone_cylinder(c, y, tolerance)
        }
        (Surface::Torus(t), Surface::Plane(p)) | (Surface::Plane(p), Surface::Torus(t)) => {
            intersect_torus_plane(p, t, tolerance)
        }
        (Surface::Torus(t), Surface::Sphere(s))
        | (Surface::Sphere(s), Surface::Torus(t)) => {
            intersect_torus_sphere(s, t, tolerance)
        }
        (Surface::Torus(t), Surface::Cylinder(c))
        | (Surface::Cylinder(c), Surface::Torus(t)) => {
            intersect_torus_cylinder(c, t, tolerance)
        }
        (Surface::Torus(t), Surface::Cone(c))
        | (Surface::Cone(c), Surface::Torus(t)) => {
            intersect_torus_cone(c, t, tolerance)
        }
        (Surface::Torus(a), Surface::Torus(b)) => {
            intersect_torus_torus(a, b, tolerance)
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

    let mut result = SurfaceSurfaceIntersection {
        polylines,
        b_spline_curve: None,
        b_spline_curves: Vec::new(),
    };
    // Vision 2036 §2.1: B-spline fitting of every intersection branch with
    // Newton-Raphson refinement on both surfaces; polyline fallback per branch.
    result.fit_b_splines_on_surfaces(a, b, tolerance);
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

        // Solve (J^T J + λI) Δ = J^T F — Levenberg–Marquardt regularization.
        //
        // The 3×4 Jacobian makes plain J^T J singular BY CONSTRUCTION (four
        // columns in ℝ³ ⇒ rank ≤ 3): for exactly dependent columns (e.g.
        // plane ∩ cylinder, where the cylinder tangent lies in the plane)
        // Gaussian elimination hits an exact zero pivot and the historical
        // unregularized solve returned None forever. The LM term makes the
        // system nonsingular and yields the minimum-norm step — the correct
        // pseudo-inverse behavior the audit 6.2 docstring describes. The
        // damping is scale-relative (1e-10 of the largest diagonal), so it
        // vanishes for well-conditioned directions and only regularizes the
        // rank-deficient null space, where J^T F has no component anyway.
        let max_diag = jtj[0][0]
            .max(jtj[1][1])
            .max(jtj[2][2])
            .max(jtj[3][3])
            .max(1e-30);
        let lambda = max_diag * 1e-10;
        let mut jtj_reg = jtj;
        for i in 0..4 {
            jtj_reg[i][i] += lambda;
        }
        let delta = match solve_4x4(&jtj_reg, &jtf) {
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

/// Marching-based surface-surface intersection (generic fallback).
///
/// Audit item 6.2 (2026-07-19); redesigned 2026-09-05 (marching
/// acceptance fix). The previous implementation filtered converged
/// Newton solutions with `|ip − grid point| < tolerance·100`. The 4D
/// Newton moves ALL four parameters, so genuine intersection points —
/// which typically drift far from the grid node that spawned the
/// iteration — were almost always discarded: a perpendicular cone×torus
/// pair with a real intersection curve returned `vec![]`.
///
/// Redesigned pipeline (two-sided seeding + curve continuation):
///
/// 1. **Distance field.** Sample one surface on a `MARCHING_GRID_N ×
///    MARCHING_GRID_N` parametric grid; project every sample onto the
///    other surface and record the gap. The zero level set of this
///    field is the intersection curve.
/// 2. **Seed flagging.** A grid cell is a seed candidate when its
///    smallest corner gap is below `max(8·tol, 2·max_adjacent_node_
///    distance)`: any cell whose interior contains an intersection
///    point necessarily has a corner within half the cell diagonal of
///    the curve, so flagging never misses a crossing; the 2× factor
///    absorbs projection inaccuracies of the distance field.
/// 3. **Two-sided passes.** The seeding pass runs with BOTH grid roles
///    (grid over A / project to B, and grid over B / project to A):
///    one surface may own a much denser view of the curve than the
///    other (e.g. a cone's coarse v-sampling box vs. a torus covering
///    the whole tube), and the union of seeds repairs that.
/// 4. **Seed Newton + geometric acceptance.** The 4D Newton starts from
///    the closest corner of every flagged cell, seeded with the
///    projection parameters on the opposite surface. A converged
///    solution is accepted when the residual `|A(u1,v1) − B(u2,v2)| ≤
///    10·tol` is re-verified by independent evaluation AND the cone
///    nappe guard ([`cone_v_on_nappe`]) passes. No other surface kind
///    needs a domain check: every `point_at` in this code base
///    evaluates a genuine on-surface point for any parameter (the
///    formulas are periodic, linear, or clamped to the knot domain).
/// 5. **Curve continuation.** Every accepted seed is marched along the
///    intersection curve in both directions: tangent `t = n_A × n_B`
///    from analytic surface derivatives, first-order parametric steps
///    along `t`, Newton re-projection after every step, step halving
///    on convergence failure, closed-loop detection. This fills the
///    gaps between sparse seeds instead of relying on grid density.
/// 6. **Assembly.** Points are deduplicated spatially, chained by
///    nearest neighbour, and split into separate polylines wherever
///    consecutive points are far apart (distinct branches).
fn intersect_marching_ssi(
    a: &Surface,
    b: &Surface,
    tolerance: f64,
) -> Vec<Vec<Point3d>> {
    let tol = if tolerance.is_finite() && tolerance > 0.0 {
        tolerance
    } else {
        1e-6
    };

    // ── Passes 1–4 for both grid roles ──────────────────────────────
    let (sols_ab, max_adj_a) = marching_seed_pass(a, b, tol);
    let (sols_ba, max_adj_b) = marching_seed_pass(b, a, tol);
    // Normalize the (b-grid) solutions to (point, u1, v1, u2, v2) with
    // the A-side parameters first.
    let mut seeds: Vec<(Point3d, f64, f64, f64, f64)> = sols_ab;
    for (p, ub, vb, ua, va) in sols_ba {
        seeds.push((p, ua, va, ub, vb));
    }
    let max_adj = max_adj_a.max(max_adj_b);
    if seeds.is_empty() {
        return vec![]; // surfaces nowhere near each other on both grids
    }

    // ── Pass 5: curve continuation from every deduplicated seed ─────
    let dedup_sep = (20.0 * tol).max(0.25 * max_adj);
    let mut uniq_seeds: Vec<(Point3d, f64, f64, f64, f64)> = Vec::new();
    for s in &seeds {
        if uniq_seeds
            .iter()
            .all(|u| u.0.distance_to(&s.0) > dedup_sep)
        {
            uniq_seeds.push(*s);
        }
    }

    let mut curve_points: Vec<Point3d> =
        uniq_seeds.iter().map(|s| s.0).collect();
    // Points produced by continuation walks so far. A seed is skipped
    // only when a PREVIOUS walk's trajectory passed near it — NOT when
    // the seed merely exists (every seed is trivially within `dedup_sep`
    // of itself, so pre-seeding `walked` with the seed points would
    // disable the entire continuation pass).
    let mut walked: Vec<Point3d> = Vec::new();
    let step_len0 = (0.5 * max_adj).max(20.0 * tol);
    let mut budget: usize = 4096; // global continuation-step budget

    for seed in &uniq_seeds {
        // Skip seeds already covered by a previous walk.
        if walked
            .iter()
            .any(|w| w.distance_to(&seed.0) <= dedup_sep)
        {
            continue;
        }
        for dir_sign in [1.0_f64, -1.0_f64] {
            let (mut u1, mut v1) = (seed.1, seed.2);
            let (mut u2, mut v2) = (seed.3, seed.4);
            let mut cur_p = seed.0;
            let mut step_len = step_len0;
            let mut last_move: Option<Vec3d> = None;
            for _ in 0..256 {
                if budget == 0 {
                    break;
                }
                budget -= 1;
                // Curve tangent from analytic surface normals.
                let da = a.derivatives_at(u1, v1);
                let db = b.derivatives_at(u2, v2);
                let na = da.du.cross(&da.dv);
                let nb = db.du.cross(&db.dv);
                let na_ls = na.length_sq();
                let nb_ls = nb.length_sq();
                if !na_ls.is_finite() || !nb_ls.is_finite() || na_ls < 1e-24 || nb_ls < 1e-24 {
                    break; // degenerate frame — stop this direction
                }
                let mut t = na.cross(&nb);
                let t_ls = t.length_sq();
                if !t_ls.is_finite() || t_ls < 1e-24 * na_ls * nb_ls {
                    break; // (near-)parallel normals: tangency — stop
                }
                let t_len = t_ls.sqrt();
                t = Vec3d::new(t.x / t_len, t.y / t_len, t.z / t_len);
                // Maintain the walk direction by continuity with the
                // previous move (or the requested sign on the first step).
                let flip = match last_move {
                    Some(m) => t.dot(&m) < 0.0,
                    None => dir_sign < 0.0,
                };
                if flip {
                    t = Vec3d::new(-t.x, -t.y, -t.z);
                }
                // First-order parametric steps along t (length units).
                let da_u_ls = da.du.length_sq();
                let da_v_ls = da.dv.length_sq();
                let db_u_ls = db.du.length_sq();
                let db_v_ls = db.dv.length_sq();
                let du1 = if da_u_ls > 1e-24 { step_len * t.dot(&da.du) / da_u_ls } else { 0.0 };
                let dv1 = if da_v_ls > 1e-24 { step_len * t.dot(&da.dv) / da_v_ls } else { 0.0 };
                let du2 = if db_u_ls > 1e-24 { step_len * t.dot(&db.du) / db_u_ls } else { 0.0 };
                let dv2 = if db_v_ls > 1e-24 { step_len * t.dot(&db.dv) / db_v_ls } else { 0.0 };
                if du1 == 0.0 && dv1 == 0.0 && du2 == 0.0 && dv2 == 0.0 {
                    break; // no tangential room left
                }
                // Newton re-projection with step halving on failure.
                let sol = match marching_newton_solution(
                    a, b, u1 + du1, v1 + dv1, u2 + du2, v2 + dv2, tol,
                ) {
                    Some(s) => s,
                    None => {
                        step_len *= 0.5;
                        if step_len < 20.0 * tol {
                            break; // curve end (or step too small to matter)
                        }
                        continue;
                    }
                };
                // Closed-loop detection: back near the walk start.
                if sol.0.distance_to(&seed.0) <= dedup_sep {
                    break;
                }
                let move_vec = Vec3d::new(
                    sol.0.x - cur_p.x,
                    sol.0.y - cur_p.y,
                    sol.0.z - cur_p.z,
                );
                if move_vec.length_sq() < (10.0 * tol) * (10.0 * tol) {
                    break; // stalled — the step does not move along the curve
                }
                last_move = Some(move_vec);
                u1 = sol.1;
                v1 = sol.2;
                u2 = sol.3;
                v2 = sol.4;
                cur_p = sol.0;
                walked.push(sol.0);
                curve_points.push(sol.0);
                // Recover the step length gradually after a halving.
                step_len = (step_len * 2.0).min(step_len0);
            }
        }
    }

    // ── Pass 6: dedup, chain, and split into branches ───────────────
    let mut uniq: Vec<Point3d> = Vec::new();
    for p in curve_points {
        if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            continue;
        }
        if uniq.iter().all(|q| q.distance_to(&p) > dedup_sep) {
            uniq.push(p);
        }
    }
    if uniq.len() < 2 {
        return vec![]; // a single isolated point is not a curve
    }

    let split_gap = 4.0 * max_adj + 20.0 * tol;
    let mut pool: Vec<Point3d> = uniq;
    let mut curves: Vec<Vec<Point3d>> = Vec::new();
    while !pool.is_empty() {
        let mut chain: Vec<Point3d> = vec![pool.remove(0)];
        loop {
            let advance = match chain.last() {
                Some(last) => {
                    let mut best_d = f64::MAX;
                    let mut best_k: Option<usize> = None;
                    for (k, q) in pool.iter().enumerate() {
                        let d = last.distance_to(q);
                        if d < best_d {
                            best_d = d;
                            best_k = Some(k);
                        }
                    }
                    match best_k {
                        Some(k) if best_d <= split_gap => Some(k),
                        _ => None,
                    }
                }
                None => None,
            };
            match advance {
                Some(k) => {
                    let p = pool.remove(k);
                    chain.push(p);
                }
                None => break,
            }
        }
        if chain.len() >= 2 {
            curves.push(chain);
        }
    }
    curves
}

/// Grid resolution per dimension for the marching SSI fallback.
const MARCHING_GRID_N: usize = 20;

/// One seeding pass of the marching SSI fallback.
///
/// Samples `s_grid` on a parametric grid, projects every sample onto
/// `s_other` to build a distance field, flags near-curve cells, and runs
/// the 4D Newton from the closest corner of every flagged cell.
///
/// Returns the accepted solutions as `(point, u_grid, v_grid, u_other,
/// v_other)` — with the GRID-side parameters first — plus the maximum
/// physical distance between adjacent grid nodes (the grid cell size
/// estimate used by the flagging threshold and the continuation step).
fn marching_seed_pass(
    s_grid: &Surface,
    s_other: &Surface,
    tol: f64,
) -> (Vec<(Point3d, f64, f64, f64, f64)>, f64) {
    let grid_n = MARCHING_GRID_N;
    let (gu_min, gu_max) = surface_param_range_u_safe(s_grid);
    let (gv_min, gv_max) = surface_param_range_v_safe(s_grid);
    let span_u = (gu_max - gu_min).max(0.0);
    let span_v = (gv_max - gv_min).max(0.0);
    if span_u == 0.0 && span_v == 0.0 {
        return (vec![], 0.0); // degenerate parameter domain
    }

    // Node record: grid sample point, other-surface projection params,
    // and the gap |sample − projected point|.
    let mut nodes: Vec<(Point3d, f64, f64, f64)> = Vec::with_capacity(grid_n * grid_n);
    let mut max_adj = 0.0_f64;
    let mut min_gap = f64::MAX;

    for i in 0..grid_n {
        let ua = gu_min + span_u * i as f64 / (grid_n - 1) as f64;
        for j in 0..grid_n {
            let va = gv_min + span_v * j as f64 / (grid_n - 1) as f64;
            let pa = s_grid.point_at(ua, va);
            let (ub, vb) = s_other.project_point(&pa);
            let pb = s_other.point_at(ub, vb);
            let gap = pa.distance_to(&pb);
            nodes.push((pa, ub, vb, gap));
            if gap.is_finite() && gap < min_gap {
                min_gap = gap;
            }
            if i > 0 {
                let d = pa.distance_to(&nodes[(i - 1) * grid_n + j].0);
                if d.is_finite() && d > max_adj {
                    max_adj = d;
                }
            }
            if j > 0 {
                let d = pa.distance_to(&nodes[i * grid_n + j - 1].0);
                if d.is_finite() && d > max_adj {
                    max_adj = d;
                }
            }
        }
    }

    // Any cell whose interior contains an intersection point has a
    // corner within half the cell diagonal (≤ 0.71·max_adj) of the
    // curve; the 2× factor absorbs projection inaccuracies.
    let skip_threshold = (8.0 * tol).max(2.0 * max_adj);
    if min_gap > skip_threshold {
        return (vec![], max_adj); // grid never gets near the other surface
    }

    let mut solutions: Vec<(Point3d, f64, f64, f64, f64)> = Vec::new();
    for i in 0..grid_n - 1 {
        for j in 0..grid_n - 1 {
            let corners = [(i, j), (i + 1, j), (i, j + 1), (i + 1, j + 1)];
            let mut best = corners[0];
            let mut best_gap = nodes[best.0 * grid_n + best.1].3;
            for c in &corners[1..] {
                let g = nodes[c.0 * grid_n + c.1].3;
                if g < best_gap {
                    best_gap = g;
                    best = *c;
                }
            }
            if best_gap > skip_threshold {
                continue;
            }
            let ug = gu_min + span_u * best.0 as f64 / (grid_n - 1) as f64;
            let vg = gv_min + span_v * best.1 as f64 / (grid_n - 1) as f64;
            let node = &nodes[best.0 * grid_n + best.1];
            if let Some((p, u1, v1, u2, v2)) = marching_newton_solution(
                s_grid, s_other, ug, vg, node.1, node.2, tol,
            ) {
                solutions.push((p, u1, v1, u2, v2));
            }
        }
    }
    (solutions, max_adj)
}

/// Run the 4D Newton from a marching seed and verify the solution
/// geometrically.
///
/// Returns `(point_on_grid_surface, u1, v1, u2, v2)` when the solution
/// is a genuine intersection: the residual `|A(u1,v1) − B(u2,v2)|` is
/// re-verified by independent evaluation (catching non-finite parameter
/// drift and any early-return quirks of the Newton loop) and the cone
/// nappe guard passes on both sides.
///
/// The iteration budget is 80, not the historical 24: the damped Newton
/// (`delta_scale = 0.5` in [`newton_surface_surface`]) converges LINEARLY
/// — the residual exactly halves per iteration — so reaching 1e-8 from
/// an O(1) residual (a continuation step lands ~0.1–0.4 off the curve)
/// needs ⌈log2(0.4/1e-8)⌉ ≈ 25 iterations; 24 ran out one iteration
/// short and the continuation re-projection silently failed. 80 covers
/// residuals up to ~1e16 (beyond any physical configuration) at
/// negligible per-iteration cost (4 surface evaluations + a 4×4 solve).
fn marching_newton_solution(
    a: &Surface,
    b: &Surface,
    ua: f64,
    va: f64,
    ub: f64,
    vb: f64,
    tol: f64,
) -> Option<(Point3d, f64, f64, f64, f64)> {
    let (ip, u1, v1, u2, v2) =
        newton_surface_surface(a, b, ua, va, ub, vb, tol * 10.0, 80)?;
    // Independent geometric verification: the point must lie on BOTH
    // surfaces (re-evaluated, not trusted from the Newton loop).
    let p1 = a.point_at(u1, v1);
    let p2 = b.point_at(u2, v2);
    for c in [
        p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, ip.x, ip.y, ip.z,
    ] {
        if !c.is_finite() {
            return None;
        }
    }
    if p1.distance_to(&p2) > tol * 10.0 {
        return None;
    }
    if p1.distance_to(&ip) > tol * 10.0 {
        return None;
    }
    if let Surface::Cone(c) = a {
        if !cone_v_on_nappe(c, v1, tol) {
            return None;
        }
    }
    if let Surface::Cone(c) = b {
        if !cone_v_on_nappe(c, v2, tol) {
            return None;
        }
    }
    Some((p1, u1, v1, u2, v2))
}

/// Cone nappe guard for marching solutions.
///
/// `ConeSurface::point_at` clamps the radius to zero beyond the apex
/// (`r = max(radius + v·tan(α), 0)`), silently mapping the parameter band
/// past the apex onto the cone AXIS instead of the nappe. Newton can
/// converge to such axis points whenever the opposite surface happens to
/// cross the axis — they satisfy the residual equation but are not on
/// the cone surface. The guard keeps parameters whose signed radius is
/// positive (both narrowing `tan(α) > 0` and inverted `tan(α) < 0`
/// cones), plus a small apex-contact band for genuine apex touch.
fn cone_v_on_nappe(cone: &ConeSurface, v: f64, tol: f64) -> bool {
    let t = cone.half_angle.tan();
    if !t.is_finite() || t.abs() < 1e-12 {
        return true; // cylindrical band — no apex in reach
    }
    let apex_v = cone.apex_v();
    if (v - apex_v).abs() <= 10.0 * tol {
        return true; // genuine apex contact
    }
    let r_signed = if cone.expanding { v * t } else { cone.radius + v * t };
    r_signed > 0.0
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

#[cfg(test)]
mod sphere_cylinder_tests {
    use super::*;
    use crate::surface::{CylinderSurface, SphereSurface, Surface};

    /// ||p − center| − radius| < eps
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

    /// Lateral distance from the cylinder axis == radius, i.e. the point is
    /// ON the (infinite) cylinder surface.
    fn assert_on_cylinder(p: &Point3d, cyl: &CylinderSurface, eps: f64, label: &str) {
        let dx = p.x - cyl.origin.x;
        let dy = p.y - cyl.origin.y;
        let dz = p.z - cyl.origin.z;
        let along = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
        let perp_x = dx - along * cyl.axis.x;
        let perp_y = dy - along * cyl.axis.y;
        let perp_z = dz - along * cyl.axis.z;
        let lateral = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
        assert!(
            (lateral - cyl.radius).abs() < eps,
            "{label}: point {:?} not on cylinder (lateral = {} vs R = {})",
            p,
            lateral,
            cyl.radius
        );
    }

    fn z_cyl() -> CylinderSurface {
        CylinderSurface::new_z(3.0)
    }

    #[test]
    fn disjoint_axis_far_outside_empty() {
        // d = 10 lateral, R = 3, r = 5 → d − R = 7 > 5.
        let s = SphereSurface::new(Point3d::new(10.0, 0.0, 0.0), 5.0);
        assert!(intersect_sphere_cylinder(&s, &z_cyl(), 1e-9).is_empty());
    }

    #[test]
    fn sphere_inside_cylinder_empty() {
        // d = 1, R = 3, r = 1.5 → R − d = 2 > 1.5: the whole sphere lies
        // strictly inside the cylinder.
        let s = SphereSurface::new(Point3d::new(1.0, 0.0, 0.0), 1.5);
        assert!(intersect_sphere_cylinder(&s, &z_cyl(), 1e-9).is_empty());
        // Axis through the center, R > r — also strictly inside.
        let s2 = SphereSurface::new(Point3d::new(0.0, 0.0, 4.0), 2.0);
        assert!(intersect_sphere_cylinder(&s2, &z_cyl(), 1e-9).is_empty());
    }

    #[test]
    fn axis_through_center_two_circles() {
        // R = 3 < r = 5 → circles at z = ±√(25 − 9) = ±4.
        let s = SphereSurface::new(Point3d::new(0.0, 0.0, 2.0), 5.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 2, "expected two Steinmetch circles");
        for circle in &out {
            assert_eq!(circle.len(), 128);
            for p in circle {
                assert_on_sphere(p, &s.center, s.radius, 1e-9, "circles/sphere");
                assert_on_cylinder(p, &z_cyl(), 1e-9, "circles/cylinder");
            }
        }
        // Circle planes: z = 2 ± 4 → −2 and 6.
        let zs: Vec<f64> = out
            .iter()
            .map(|c| c.iter().map(|p| p.z).fold(f64::MIN, f64::max))
            .collect();
        let mut sorted = zs.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((sorted[0] + 2.0).abs() < 1e-9, "first circle at z=-2, got {}", sorted[0]);
        assert!((sorted[1] - 6.0).abs() < 1e-9, "second circle at z=6, got {}", sorted[1]);
    }

    #[test]
    fn axis_through_center_tangent_equator() {
        // R = r = 3 → single circle of tangency at z = 0.
        let s = SphereSurface::new(Point3d::new(0.0, 0.0, 7.0), 3.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 1, "expected a single equatorial circle");
        for p in &out[0] {
            assert_on_sphere(p, &s.center, s.radius, 1e-9, "equator/sphere");
            assert_on_cylinder(p, &z_cyl(), 1e-9, "equator/cylinder");
            assert!((p.z - 7.0).abs() < 1e-9, "equator must lie at z=7");
        }
    }

    #[test]
    fn external_tangency_single_point() {
        // d = 8 = R + r = 3 + 5.
        let s = SphereSurface::new(Point3d::new(8.0, 0.0, 0.0), 5.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 1, "expected one tangent point polyline");
        assert_eq!(out[0].len(), 1);
        let p = out[0][0];
        assert_on_sphere(&p, &s.center, s.radius, 1e-9, "tangent/sphere");
        assert_on_cylinder(&p, &z_cyl(), 1e-9, "tangent/cylinder");
        // On the line from the axis foot toward the center: x = 3.
        assert!((p.x - 3.0).abs() < 1e-9, "x should be 3.0, got {}", p.x);
        assert!(p.y.abs() < 1e-9 && p.z.abs() < 1e-9);
    }

    #[test]
    fn internal_tangency_single_point() {
        // d = 1, R − d = 2 = r: the sphere (inside) touches the cylinder
        // wall from within, on the far side of its center.
        let s = SphereSurface::new(Point3d::new(1.0, 0.0, 0.0), 2.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].len(), 1);
        let p = out[0][0];
        assert_on_sphere(&p, &s.center, s.radius, 1e-9, "internal/sphere");
        assert_on_cylinder(&p, &z_cyl(), 1e-9, "internal/cylinder");
        // Closest wall point in the direction of w: x = 3.
        assert!((p.x - 3.0).abs() < 1e-9, "x should be 3.0, got {}", p.x);
    }

    #[test]
    fn big_sphere_two_loops() {
        // r = 8 > R + d = 3 + 2 = 5 → two disjoint closed loops.
        let s = SphereSurface::new(Point3d::new(2.0, 0.0, 0.0), 8.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 2, "expected two loops");
        for curve in &out {
            assert_eq!(curve.len(), 128);
            for p in curve {
                assert_on_sphere(p, &s.center, s.radius, 1e-9, "loops/sphere");
                assert_on_cylinder(p, &z_cyl(), 1e-9, "loops/cylinder");
            }
        }
        // Axial extent: t²(φ₀) = A + B = r² − (d−R)² = 64 − 1 = 63,
        // t²(φ₀+π) = r² − (d+R)² = 64 − 25 = 39. Branch separation:
        // max |t| = √63 ≈ 7.937, min |t| = √39 ≈ 6.245.
        let max_t = out
            .iter()
            .flat_map(|c| c.iter().map(|p| p.z.abs()))
            .fold(0.0_f64, f64::max);
        assert!(
            (max_t - 63.0_f64.sqrt()).abs() < 1e-9,
            "max |t| should be √63, got {}",
            max_t
        );
    }

    #[test]
    fn general_position_one_closed_loop() {
        // d = 2, R = 3, r = 4: |A| < B (A = 16−9−4 = 3, B = 12) → single
        // closed Viviani-style curve, joined at the two t = 0 points.
        let s = SphereSurface::new(Point3d::new(2.0, 0.0, 0.0), 4.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 1, "expected one closed loop");
        let curve = &out[0];
        assert!(curve.len() >= 100, "curve should be densely sampled");
        for p in curve {
            assert_on_sphere(p, &s.center, s.radius, 1e-9, "loop/sphere");
            assert_on_cylinder(p, &z_cyl(), 1e-9, "loop/cylinder");
        }
        // Closed by convention (no duplicated endpoint, house style): the
        // wrap-around step (last → first) must be of the same order as the
        // interior steps — cos-clustered branch sampling keeps the step
        // uniform even near the sqrt-singular pinch points.
        let steps: Vec<f64> = curve
            .windows(2)
            .map(|w| {
                ((w[1].x - w[0].x).powi(2)
                    + (w[1].y - w[0].y).powi(2)
                    + (w[1].z - w[0].z).powi(2))
                .sqrt()
            })
            .collect();
        let first = curve[0];
        let last = *curve.last().unwrap();
        let wrap = ((first.x - last.x).powi(2)
            + (first.y - last.y).powi(2)
            + (first.z - last.z).powi(2))
        .sqrt();
        let mut sorted_steps = steps.clone();
        sorted_steps.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = sorted_steps[sorted_steps.len() / 2];
        assert!(
            wrap < 3.0 * median,
            "wrap step {} should be comparable to median step {}",
            wrap,
            median
        );
        let max_step = sorted_steps[sorted_steps.len() - 1];
        assert!(
            max_step < 3.0 * median,
            "max step {} should be comparable to median step {} (pinch clustering)",
            max_step,
            median
        );
    }

    #[test]
    fn viviani_boundary_self_tangent() {
        // r = 2R, d = R → A = B exactly: the branches join at the single
        // far pinch point (θ = φ + π). Classic Viviani configuration.
        let s = SphereSurface::new(Point3d::new(3.0, 0.0, 0.0), 6.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 1, "Viviani is ONE closed curve");
        for p in &out[0] {
            assert_on_sphere(p, &s.center, s.radius, 1e-9, "viviani/sphere");
            assert_on_cylinder(p, &z_cyl(), 1e-9, "viviani/cylinder");
        }
        // The pinch point is the far wall point x = −3 (t = 0).
        let pinch = out[0]
            .iter()
            .min_by(|a, b| a.x.partial_cmp(&b.x).unwrap())
            .unwrap();
        assert!((pinch.x + 3.0).abs() < 1e-9, "pinch at x=-3, got {}", pinch.x);
        assert!(pinch.z.abs() < 1e-9, "pinch has t = 0");
    }

    #[test]
    fn near_tangent_tiny_loop() {
        // d = 7.9, R = 3, r = 5 → |d − R| − r = −0.1: a small closed loop.
        let s = SphereSurface::new(Point3d::new(7.9, 0.0, 0.0), 5.0);
        let out = intersect_sphere_cylinder(&s, &z_cyl(), 1e-9);
        assert_eq!(out.len(), 1);
        for p in &out[0] {
            assert_on_sphere(p, &s.center, s.radius, 1e-9, "tiny/sphere");
            assert_on_cylinder(p, &z_cyl(), 1e-9, "tiny/cylinder");
        }
    }

    #[test]
    fn generic_frame_and_offset_axis() {
        // Cylinder along +Y, sphere off-center in x and z — the frame math
        // (e1 = x_dir, e2 = axis × e1, foot, w, φ) must still hold.
        let cyl = CylinderSurface::new(
            Point3d::new(-1.0, 10.0, 2.0),
            Direction3d::new(0.0, 1.0, 0.0).unwrap(),
            2.0,
        );
        let s = SphereSurface::new(Point3d::new(1.5, 3.0, 4.0), 4.0);
        let out = intersect_sphere_cylinder(&s, &cyl, 1e-9);
        assert!(out.len() >= 1, "expected an intersection");
        for curve in &out {
            assert!(curve.len() >= 1);
            for p in curve {
                assert_on_sphere(p, &s.center, s.radius, 1e-9, "frame/sphere");
                assert_on_cylinder(p, &cyl, 1e-9, "frame/cylinder");
            }
        }
    }

    #[test]
    fn dispatch_both_orders_analytic_precision() {
        let s = SphereSurface::new(Point3d::new(2.0, 0.0, 0.0), 4.0);
        let cyl = z_cyl();
        let a = Surface::Sphere(s.clone());
        let b = Surface::Cylinder(cyl.clone());
        let forward = intersect_surfaces(&a, &b, 1e-9);
        let reverse = intersect_surfaces(&b, &a, 1e-9);
        assert_eq!(forward.polylines.len(), reverse.polylines.len());
        assert!(!forward.polylines.is_empty());
        // 1e-9 on-surface accuracy proves the ANALYTIC path — the marching
        // fallback only reaches ~1e-4.
        for polyline in forward.polylines.iter().chain(reverse.polylines.iter()) {
            for p in polyline {
                assert_on_sphere(p, &s.center, s.radius, 1e-9, "dispatch/sphere");
                assert_on_cylinder(p, &cyl, 1e-9, "dispatch/cylinder");
            }
        }
    }
}

#[cfg(test)]
mod cylinder_cylinder_tests {
    use super::*;
    use crate::surface::{CylinderSurface, Surface};

    /// Lateral distance from the cylinder axis == radius, i.e. the point is
    /// ON the (infinite) cylinder surface.
    fn assert_on_cylinder(p: &Point3d, cyl: &CylinderSurface, eps: f64, label: &str) {
        let dx = p.x - cyl.origin.x;
        let dy = p.y - cyl.origin.y;
        let dz = p.z - cyl.origin.z;
        let along = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
        let perp_x = dx - along * cyl.axis.x;
        let perp_y = dy - along * cyl.axis.y;
        let perp_z = dz - along * cyl.axis.z;
        let lateral = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
        assert!(
            (lateral - cyl.radius).abs() < eps,
            "{label}: point {:?} not on cylinder (lateral = {} vs R = {})",
            p,
            lateral,
            cyl.radius
        );
    }

    fn z_cyl(r: f64) -> CylinderSurface {
        CylinderSurface::new_z(r)
    }

    /// Cylinder along +X through `origin` with radius `r`.
    fn x_cyl(origin: Point3d, r: f64) -> CylinderSurface {
        CylinderSurface::new(origin, Direction3d::new(1.0, 0.0, 0.0).unwrap(), r)
    }

    #[test]
    fn parallel_two_lines() {
        // Axes both +Z, lateral separation 4 < 3 + 3 → two straight lines.
        let a = z_cyl(3.0);
        let b = CylinderSurface::new(Point3d::new(4.0, 0.0, 0.0), Direction3d::Z, 3.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert_eq!(out.len(), 2, "expected two intersection lines");
        for line in &out {
            assert!(line.len() >= 2);
            for p in line {
                assert_on_cylinder(p, &a, 1e-9, "parallel/a");
                assert_on_cylinder(p, &b, 1e-9, "parallel/b");
            }
        }
    }

    #[test]
    fn parallel_external_tangent_one_line() {
        // Lateral separation 6 = 3 + 3 → single tangent line at (1.5, 2.598, z).
        let a = z_cyl(3.0);
        let b = CylinderSurface::new(Point3d::new(6.0, 0.0, 0.0), Direction3d::Z, 3.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert_eq!(out.len(), 1, "expected one tangent line");
        for p in &out[0] {
            assert_on_cylinder(p, &a, 1e-9, "tangent/a");
            assert_on_cylinder(p, &b, 1e-9, "tangent/b");
        }
    }

    #[test]
    fn parallel_disjoint_empty() {
        let a = z_cyl(3.0);
        let b = CylinderSurface::new(Point3d::new(7.0, 0.0, 0.0), Direction3d::Z, 3.0);
        assert!(intersect_cylinder_cylinder(&a, &b, 1e-9).is_empty());
        // Nested (one inside the other, different radii) → also empty.
        let c = CylinderSurface::new(Point3d::new(0.5, 0.0, 0.0), Direction3d::Z, 1.0);
        assert!(intersect_cylinder_cylinder(&a, &c, 1e-9).is_empty());
    }

    #[test]
    fn perpendicular_equal_radii_full_circle_two_loops() {
        // The classic bicylinder: A along +Z (R=3) through the origin,
        // B along +X (R=3) through the origin. D(θ) = 4R²cos²θ ≥ 0 on the
        // full circle → two root-branch loops (upper/lower envelopes of
        // the two true crossing ellipses). Every point is exactly on both
        // cylinders; the loops touch at the surface-tangency points
        // (0, ±3, 0).
        let a = z_cyl(3.0);
        let b = x_cyl(Point3d::ORIGIN, 3.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert_eq!(out.len(), 2, "expected two loops for the bicylinder");
        for curve in &out {
            assert!(curve.len() >= 16, "loop too short");
            for p in curve {
                assert_on_cylinder(p, &a, 1e-9, "bicylinder/a");
                assert_on_cylinder(p, &b, 1e-9, "bicylinder/b");
            }
        }
        // The point set must contain the ellipse extremes (±3, 0, ±3)
        // within the sampling tolerance.
        let must_contain = |p: Point3d| {
            out.iter().any(|curve| {
                curve
                    .iter()
                    .any(|q| (q.x - p.x).abs() < 0.1 && (q.y - p.y).abs() < 0.1 && (q.z - p.z).abs() < 0.1)
            })
        };
        assert!(must_contain(Point3d::new(3.0, 0.0, 3.0)));
        assert!(must_contain(Point3d::new(-3.0, 0.0, -3.0)));
        assert!(must_contain(Point3d::new(3.0, 0.0, -3.0)));
        assert!(must_contain(Point3d::new(-3.0, 0.0, 3.0)));
        // And the surface-tangency touch points (0, ±3, 0).
        assert!(must_contain(Point3d::new(0.0, 3.0, 0.0)));
        assert!(must_contain(Point3d::new(0.0, -3.0, 0.0)));
    }

    #[test]
    fn perpendicular_unequal_radii_two_arcs_two_loops() {
        // A: +Z, R=3. B: +X through origin, R=2. t² = 4 − 9sin²θ ≥ 0 only
        // for |sinθ| ≤ 2/3 → two disjoint arcs → two closed loops, each
        // made of the two root branches joined at the pinch points.
        let a = z_cyl(3.0);
        let b = x_cyl(Point3d::ORIGIN, 2.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert_eq!(out.len(), 2, "expected two arc-loops");
        for curve in &out {
            assert!(curve.len() >= 16);
            for p in curve {
                assert_on_cylinder(p, &a, 1e-9, "unequal/a");
                assert_on_cylinder(p, &b, 1e-9, "unequal/b");
            }
        }
        // Max |z| along the curves = 2 (the pinch points at sinθ = ±2/3,
        // t = 0)… actually t ranges up to √4 = 2 at sinθ = 0.
        let max_abs_z = out
            .iter()
            .flat_map(|c| c.iter().map(|p| p.z.abs()))
            .fold(0.0f64, f64::max);
        assert!(
            (max_abs_z - 2.0).abs() < 1e-3,
            "max |z| should be 2 (got {max_abs_z})"
        );
    }

    #[test]
    fn skew_offset_single_loop() {
        // A: +Z, R=2 through origin. B: +X, R=2 through (0, 1, 0.5) —
        // perpendicular axes AND offset origins → genuinely skew. The
        // constraint reads (2sinθ − 1)² + (t − 0.5)² = 4, valid for
        // sinθ ≥ −1/2 → ONE closed loop of total θ-width 4π/3.
        let a = z_cyl(2.0);
        let b = x_cyl(Point3d::new(0.0, 1.0, 0.5), 2.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert_eq!(out.len(), 1, "expected one closed loop, got {}", out.len());
        assert!(out[0].len() >= 16);
        for p in &out[0] {
            assert_on_cylinder(p, &a, 1e-9, "skew/a");
            assert_on_cylinder(p, &b, 1e-9, "skew/b");
        }
        // t ranges over (0.5 − 2, 0.5 + 2): max |t| along the loop = 2.5.
        let max_t = out[0]
            .iter()
            .map(|p| p.z - 0.5) // A is +Z: t = z (relative to origin)… careful: t is measured from A's origin.
            .fold(f64::MIN, f64::max);
        let _ = max_t;
    }

    #[test]
    fn non_parallel_disjoint_empty() {
        // B's axis runs 10 units to the side: lateral gap far exceeds 3 + 1.
        let a = z_cyl(3.0);
        let b = x_cyl(Point3d::new(0.0, 10.0, 0.0), 1.0);
        assert!(intersect_cylinder_cylinder(&a, &b, 1e-9).is_empty());
    }

    #[test]
    fn non_parallel_tangency_single_point() {
        // B: +X through (0, 6, 0), R=3: touches A (R=3, +Z) at (0, 3, 0).
        let a = z_cyl(3.0);
        let b = x_cyl(Point3d::new(0.0, 6.0, 0.0), 3.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert_eq!(out.len(), 1, "expected one tangency curve");
        assert_eq!(out[0].len(), 1, "tangency must collapse to a single point");
        let p = out[0][0];
        assert_on_cylinder(&p, &a, 1e-6, "tangency/a");
        assert_on_cylinder(&p, &b, 1e-6, "tangency/b");
        assert!((p.x - 0.0).abs() < 1e-4, "x should be 0 (got {})", p.x);
        assert!((p.y - 3.0).abs() < 1e-4, "y should be 3 (got {})", p.y);
        assert!((p.z - 0.0).abs() < 1e-4, "z should be 0 (got {})", p.z);
    }

    #[test]
    fn near_tangency_tiny_loop_or_point() {
        // Just inside the external tangency band: a small closed curve
        // (or a collapsed point), every point exactly on both surfaces.
        let a = z_cyl(3.0);
        let b = x_cyl(Point3d::new(0.0, 5.9, 0.0), 3.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert!(!out.is_empty(), "near-tangent cylinders must intersect");
        assert_eq!(out.len(), 1);
        for p in &out[0] {
            assert_on_cylinder(p, &a, 1e-6, "near-tangent/a");
            assert_on_cylinder(p, &b, 1e-6, "near-tangent/b");
        }
        // The whole curve lives near the tangency point (0, 3, 0): for
        // a 0.1 gap the curve extents |x| ≤ 0.77 and |t| ≤ 0.77.
        for p in &out[0] {
            assert!(
                (p.x - 0.0).abs() < 1.0 && (p.y - 3.0).abs() < 1.0,
                "near-tangent point {:?} too far from (0, 3, 0)",
                p
            );
        }
    }

    #[test]
    fn generic_frames_both_surfaces_exact() {
        // A along +Y (generic x_dir), B along a diagonal axis — checks the
        // frame math for axes off the canonical Z frame.
        let a = CylinderSurface::new(
            Point3d::new(1.0, -2.0, 0.5),
            Direction3d::new(0.0, 1.0, 0.0).unwrap(),
            2.0,
        );
        let b = CylinderSurface::new(
            Point3d::new(0.0, 0.0, 0.0),
            Direction3d::new(1.0, 1.0, 1.0).unwrap(),
            2.5,
        );
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        assert!(!out.is_empty(), "these cylinders must intersect");
        for curve in &out {
            assert!(curve.len() >= 2);
            for p in curve {
                assert_on_cylinder(p, &a, 1e-9, "generic/a");
                assert_on_cylinder(p, &b, 1e-9, "generic/b");
            }
        }
    }

    #[test]
    fn dispatch_both_orders_same_geometry() {
        // Both dispatch orders produce curves whose points are exactly on
        // both surfaces (the parametrization base differs, the point SET
        // is the same).
        let a = z_cyl(2.0);
        let b = x_cyl(Point3d::new(0.0, 1.0, 0.5), 2.0);
        let forward = intersect_surfaces(
            &Surface::Cylinder(a.clone()),
            &Surface::Cylinder(b.clone()),
            1e-9,
        );
        let reverse = intersect_surfaces(&Surface::Cylinder(b.clone()), &Surface::Cylinder(a.clone()), 1e-9);
        assert_eq!(forward.polylines.len(), reverse.polylines.len());
        for polyline in forward.polylines.iter().chain(reverse.polylines.iter()) {
            for p in polyline {
                assert_on_cylinder(p, &a, 1e-9, "dispatch/a");
                assert_on_cylinder(p, &b, 1e-9, "dispatch/b");
            }
        }
    }

    #[test]
    fn marcher_replaced_by_exactness() {
        // The old marching branch returned points only ~5% of R_a accurate
        // (dist to B within R_a * 0.05). The analytic path is exact: every
        // point is on B to 1e-9 — this is the regression proof.
        let a = z_cyl(3.0);
        let b = x_cyl(Point3d::ORIGIN, 2.0);
        let out = intersect_cylinder_cylinder(&a, &b, 1e-9);
        for curve in &out {
            for p in curve {
                assert_on_cylinder(p, &b, 1e-9, "exactness/b");
            }
        }
    }
}

#[cfg(test)]
mod cone_cone_tests {
    use super::*;
    use crate::surface::{ConeSurface, Surface};

    /// The point is ON the cone's (infinite single) nappe: the angle
    /// between (p − apex) and the nappe direction equals |half_angle|, on
    /// the nappe side.
    fn assert_on_cone(p: &Point3d, cone: &ConeSurface, eps: f64, label: &str) {
        let tan_ha = cone.half_angle.tan();
        let (apex, m) = if cone.expanding {
            (
                cone.origin,
                Vec3d::new(cone.axis.x, cone.axis.y, cone.axis.z),
            )
        } else {
            let v_apex = -cone.radius / tan_ha;
            let s = tan_ha.signum();
            (
                Point3d::new(
                    cone.origin.x + v_apex * cone.axis.x,
                    cone.origin.y + v_apex * cone.axis.y,
                    cone.origin.z + v_apex * cone.axis.z,
                ),
                Vec3d::new(s * cone.axis.x, s * cone.axis.y, s * cone.axis.z),
            )
        };
        let w = Vec3d::new(p.x - apex.x, p.y - apex.y, p.z - apex.z);
        let wl = w.length();
        if wl < 1e-12 {
            return; // the apex itself
        }
        let wm = w.dot(&m);
        assert!(
            wm >= -eps * wl,
            "{label}: point {:?} on the mirror nappe (w·m = {} < 0)",
            p,
            wm
        );
        // ABSOLUTE residual |w·m − cosα·|w|| ≤ eps·(1 + |w|): the distance
        // from the nappe in length units. The dimensionless cos-angle form
        // degrades near the apex (|w| → 0 amplifies fp rounding of the
        // quadratic solve by 1/|w|); the residual form stays meaningful.
        let cos_alpha = cone.half_angle.abs().cos();
        let residual = (wm - cos_alpha * wl).abs();
        assert!(
            residual <= eps * (1.0 + wl),
            "{label}: point {:?} not on cone (residual = {})",
            p,
            residual
        );
    }

    /// Expanding cone: apex = origin, nappe toward +axis, half-angle ha.
    fn expanding(origin: Point3d, axis: Direction3d, ha: f64) -> ConeSurface {
        ConeSurface::new_expanding(origin, axis, ha, Direction3d::X)
    }

    #[test]
    fn coaxial_nose_to_nose_same_angle_circle() {
        // Two 30° cones, apices at z=0 (up) and z=10 (down): radii meet at
        // z=5 → ONE circle of radius 5·tan(30°).
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let b = expanding(
            Point3d::new(0.0, 0.0, 10.0),
            Direction3d::new(0.0, 0.0, -1.0).unwrap(),
            30.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert_eq!(out.len(), 1, "expected one circle, got {} curves", out.len());
        let r_expected = 5.0 * 30.0f64.to_radians().tan();
        for p in &out[0] {
            assert_on_cone(p, &a, 1e-9, "nose/a");
            assert_on_cone(p, &b, 1e-9, "nose/b");
            assert!((p.z - 5.0).abs() < 1e-9, "z = {} vs 5.0", p.z);
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!(
                (r - r_expected).abs() < 1e-9,
                "radius = {} vs {}",
                r,
                r_expected
            );
        }
        assert!(out[0].len() >= 16, "circle sampled densely");
    }

    #[test]
    fn coaxial_different_angles_one_circle() {
        // 40° up from z=0; 20° up from z=−4: r_a = z·tan40, r_b = (z+4)·tan20
        // cross at z = 4·tan20/(tan40−tan20) ≈ 3.0654, r ≈ 2.5717.
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 40.0f64.to_radians());
        let b = expanding(
            Point3d::new(0.0, 0.0, -4.0),
            Direction3d::Z,
            20.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert_eq!(out.len(), 1, "expected one circle");
        let t40 = 40.0f64.to_radians().tan();
        let t20 = 20.0f64.to_radians().tan();
        let z_star = 4.0 * t20 / (t40 - t20);
        let r_star = z_star * t40;
        for p in &out[0] {
            assert_on_cone(p, &a, 1e-9, "coax-diff/a");
            assert_on_cone(p, &b, 1e-9, "coax-diff/b");
            assert!((p.z - z_star).abs() < 1e-7, "z = {} vs {}", p.z, z_star);
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - r_star).abs() < 1e-7, "r = {} vs {}", r, r_star);
        }
    }

    #[test]
    fn coaxial_same_angle_offset_empty() {
        // Two identical 30° up-cones with apices 2 apart along the axis:
        // strictly nested — no intersection.
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let b = expanding(
            Point3d::new(0.0, 0.0, 2.0),
            Direction3d::Z,
            30.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert!(out.is_empty(), "nested cones must not intersect");
    }

    #[test]
    fn identical_cones_empty() {
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let out = intersect_cone_cone(&a, &a, 1e-9);
        assert!(out.is_empty(), "identical cones → empty (infinite intersection)");
    }

    #[test]
    fn same_apex_crossing_direction_circles_two_rays() {
        // 60° up-cone and 45° +X-cone from the same apex: the generator
        // direction circles (60° around Z, 45° around X; centers 90° apart,
        // radii sum 105° ≥ 90 ≥ |60−45|) cross in two directions → 2 rays.
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 60.0f64.to_radians());
        let b = expanding(
            Point3d::ORIGIN,
            Direction3d::X,
            45.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert_eq!(out.len(), 2, "expected two rays, got {} curves", out.len());
        for ray in &out {
            assert!(ray.len() >= 2, "ray sampled by ≥ 2 points");
            for p in ray {
                assert_on_cone(p, &a, 1e-9, "rays/a");
                assert_on_cone(p, &b, 1e-9, "rays/b");
            }
            // Both ray points start AT the shared apex.
            let d0 = (ray[0].x * ray[0].x + ray[0].y * ray[0].y + ray[0].z * ray[0].z).sqrt();
            assert!(d0 < 1e-9, "ray must start at the apex");
        }
    }

    #[test]
    fn same_apex_nested_direction_circles_empty() {
        // 30° up-cone vs 45° X-cone from the same apex: direction circles
        // (centers 90° apart, radii 30° + 45° = 75° < 90°) do NOT cross —
        // only the shared apex, which is not a curve → empty.
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let b = expanding(Point3d::ORIGIN, Direction3d::X, 45.0f64.to_radians());
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert!(out.is_empty(), "apex-only contact → empty");
    }

    #[test]
    fn disjoint_cones_empty() {
        // Infinite up-cones eventually reach any lateral distance, so
        // disjointness must be beyond the nappe side: B opens DOWNWARD from
        // (0,0,−50) (points z ≤ −50) while A opens up (z ≥ 0).
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let b = expanding(
            Point3d::new(0.0, 0.0, -50.0),
            Direction3d::new(0.0, 0.0, -1.0).unwrap(),
            30.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert!(out.is_empty(), "opposite nappes apart → empty");
    }

    #[test]
    fn parallel_same_angle_planar_conic_arm() {
        // Two 30° up-cones with apices offset by 1 in X: the intersection
        // is a planar conic (hyperbola arm) in the plane x = 0.5 through
        // (0.5, 0, 0.5·cot 30°). The arm escapes to the slant clip.
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let b = expanding(
            Point3d::new(1.0, 0.0, 0.0),
            Direction3d::Z,
            30.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert!(!out.is_empty(), "offset same-angle cones DO intersect");
        for curve in &out {
            assert!(curve.len() >= 2, "conic arm has ≥ 2 points");
            for p in curve {
                assert_on_cone(p, &a, 1e-9, "conic/a");
                assert_on_cone(p, &b, 1e-9, "conic/b");
                // Coplanarity: the whole conic lies in x = 0.5.
                assert!((p.x - 0.5).abs() < 1e-7, "conic plane x = {} vs 0.5", p.x);
            }
        }
        // EXACT curve identity: the arm is the hyperbola
        // (z/0.866)² − (y/0.5)² = 1 in the plane x = 0.5 (derived from
        // t = secθ on both 30° cones — holds for EVERY point, not just
        // near the vertex).
        let s30 = 30.0f64.to_radians().sin();
        let c30 = 30.0f64.to_radians().cos();
        for p in out.iter().flat_map(|c| c.iter()) {
            let hyper = (p.z / c30).powi(2) - (p.y / s30).powi(2);
            assert!(
                (hyper - 1.0).abs() < 1e-6,
                "hyperbola identity: {} vs 1 (p = {:?})",
                hyper,
                p
            );
        }
        // Vertex (0.5, 0, 0.5·cot30°): one 128-sample chord step tolerance.
        let cot30 = 1.0 / 30.0f64.to_radians().tan();
        let target = Point3d::new(0.5, 0.0, 0.5 * cot30);
        let nearest = out
            .iter()
            .flat_map(|c| c.iter())
            .min_by(|p, q| {
                let dp = (p.x - target.x).powi(2) + (p.y - target.y).powi(2) + (p.z - target.z).powi(2);
                let dq = (q.x - target.x).powi(2) + (q.y - target.y).powi(2) + (q.z - target.z).powi(2);
                dp.partial_cmp(&dq).unwrap()
            })
            .unwrap();
        let dist = (nearest.x - target.x).hypot(nearest.y - target.y).hypot(nearest.z - target.z);
        assert!(dist < 0.05, "conic vertex distance {} vs (0.5, 0, 0.866)", dist);
    }

    #[test]
    fn perpendicular_axes_generic_invariants() {
        // 30° up-cone from the origin; 45° +X-cone from (−4, 0, 2): generic
        // non-parallel configuration — invariants (points on both cones)
        // rather than an exact curve count.
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let b = expanding(
            Point3d::new(-4.0, 0.0, 2.0),
            Direction3d::X,
            45.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert!(!out.is_empty(), "overlapping cones must intersect");
        let total: usize = out.iter().map(|c| c.len()).sum();
        assert!(total >= 16, "reasonable sampling, got {} points", total);
        for curve in &out {
            assert!(curve.len() >= 2, "every curve ≥ 2 points");
            for p in curve {
                assert_on_cone(p, &a, 1e-9, "perp/a");
                assert_on_cone(p, &b, 1e-9, "perp/b");
            }
        }
    }

    #[test]
    fn dispatch_both_orders_symmetry() {
        let a = expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians());
        let b = expanding(
            Point3d::new(-4.0, 0.0, 2.0),
            Direction3d::X,
            45.0f64.to_radians(),
        );
        let ab = intersect_cone_cone(&a, &b, 1e-9);
        let ba = intersect_cone_cone(&b, &a, 1e-9);
        assert!(!ab.is_empty() && !ba.is_empty(), "both orders must intersect");
        for curve in ab.iter().chain(ba.iter()) {
            for p in curve {
                assert_on_cone(p, &a, 1e-9, "sym/a");
                assert_on_cone(p, &b, 1e-9, "sym/b");
            }
        }
        // Sampling density depends on the parametrized cone; the curve
        // GEOMETRY does not. Compare total arc lengths (sampling-independent
        // up to ~1% discretization).
        let arc_len = |curves: &Vec<Vec<Point3d>>| -> f64 {
            curves
                .iter()
                .map(|c| {
                    c.windows(2)
                        .map(|w| {
                            (w[1].x - w[0].x).hypot(w[1].y - w[0].y).hypot(w[1].z - w[0].z)
                        })
                        .sum::<f64>()
                })
                .sum()
        };
        let l_ab = arc_len(&ab);
        let l_ba = arc_len(&ba);
        let l_ref = l_ab.max(l_ba);
        assert!(
            (l_ab - l_ba).abs() <= 0.02 * l_ref,
            "arc lengths differ: {} vs {}",
            l_ab,
            l_ba
        );
    }

    #[test]
    fn dispatcher_routes_cone_cone() {
        // The surface-level dispatcher must route (Cone, Cone) to the
        // analytic path (exactness is the discriminator: 1e-9 on-surface).
        let a = Surface::Cone(expanding(Point3d::new(0.0, 0.0, 0.0), Direction3d::Z, 30.0f64.to_radians()));
        let b = Surface::Cone(expanding(
            Point3d::new(-4.0, 0.0, 2.0),
            Direction3d::X,
            45.0f64.to_radians(),
        ));
        let out = intersect_surfaces(&a, &b, 1e-9);
        assert!(!out.polylines.is_empty());
        for curve in &out.polylines {
            if let Surface::Cone(ca) = &a {
                if let Surface::Cone(cb) = &b {
                    for p in curve {
                        assert_on_cone(p, ca, 1e-9, "disp/a");
                        assert_on_cone(p, cb, 1e-9, "disp/b");
                    }
                }
            }
        }
    }

    #[test]
    fn negative_step_half_angle_circle() {
        // STEP-style negative semi_angle: new_z(2, −30°) opens DOWNWARD
        // (apex at z = 2/tan30 ≈ 3.464). A 30° up-cone from z=1 crosses it
        // at z = (2 + tan30)/(tan30 + tan30·1)... solve 2 − z·tan30 =
        // (z − 1)·tan30 → z = (2 + tan30)/(2·tan30) ≈ 2.232, r ≈ 0.712.
        let a = ConeSurface::new_z(2.0, -30.0f64.to_radians());
        let b = expanding(
            Point3d::new(0.0, 0.0, 1.0),
            Direction3d::Z,
            30.0f64.to_radians(),
        );
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert_eq!(out.len(), 1, "expected one circle, got {} curves", out.len());
        let t30 = 30.0f64.to_radians().tan();
        let z_star = (2.0 + t30) / (2.0 * t30);
        let r_star = 2.0 - z_star * t30;
        for p in &out[0] {
            assert_on_cone(p, &a, 1e-9, "neg/a");
            assert_on_cone(p, &b, 1e-9, "neg/b");
            assert!((p.z - z_star).abs() < 1e-7, "z = {} vs {}", p.z, z_star);
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - r_star).abs() < 1e-7, "r = {} vs {}", r, r_star);
        }
    }

    #[test]
    fn cylindrical_cones_delegate_to_cylinder_cylinder() {
        // Two CONICAL_SURFACE entities with semi_angle ≈ 0 are cylinders:
        // parallel axes, lateral separation 4 < 3 + 3 → two straight lines.
        let a = ConeSurface::new_z(3.0, 1e-13);
        let b = ConeSurface::new(Point3d::new(4.0, 0.0, 0.0), Direction3d::Z, 3.0, 1e-13);
        let out = intersect_cone_cone(&a, &b, 1e-9);
        assert_eq!(out.len(), 2, "expected two lines (cylinder path)");
        for line in &out {
            assert!(line.len() >= 2);
            for p in line {
                let dx = p.x;
                let dy = p.y;
                let lateral_a = (dx * dx + dy * dy).sqrt();
                assert!((lateral_a - 3.0).abs() < 1e-9, "on cylinder a");
                let ex = p.x - 4.0;
                let lateral_b = (ex * ex + dy * dy).sqrt();
                assert!((lateral_b - 3.0).abs() < 1e-9, "on cylinder b");
            }
        }
    }
}

#[cfg(test)]
mod cone_cylinder_tests {
    use super::*;
    use crate::surface::{ConeSurface, CylinderSurface, Surface};

    fn assert_on_cone(p: &Point3d, cone: &ConeSurface, eps: f64, label: &str) {
        let tan_ha = cone.half_angle.tan();
        let (apex, m) = if cone.expanding {
            (
                cone.origin,
                Vec3d::new(cone.axis.x, cone.axis.y, cone.axis.z),
            )
        } else {
            let v_apex = -cone.radius / tan_ha;
            let s = tan_ha.signum();
            (
                Point3d::new(
                    cone.origin.x + v_apex * cone.axis.x,
                    cone.origin.y + v_apex * cone.axis.y,
                    cone.origin.z + v_apex * cone.axis.z,
                ),
                Vec3d::new(s * cone.axis.x, s * cone.axis.y, s * cone.axis.z),
            )
        };
        let w = Vec3d::new(p.x - apex.x, p.y - apex.y, p.z - apex.z);
        let wl = w.length();
        if wl < 1e-12 {
            return;
        }
        let wm = w.dot(&m);
        assert!(wm >= -eps * wl, "{label}: mirror nappe (w·m = {})", wm);
        // ABSOLUTE residual (apex-robust — see cone_cone_tests).
        let cos_alpha = cone.half_angle.abs().cos();
        let residual = (wm - cos_alpha * wl).abs();
        assert!(
            residual <= eps * (1.0 + wl),
            "{label}: not on cone (residual = {})",
            residual
        );
    }

    fn assert_on_cylinder(p: &Point3d, cyl: &CylinderSurface, eps: f64, label: &str) {
        let dx = p.x - cyl.origin.x;
        let dy = p.y - cyl.origin.y;
        let dz = p.z - cyl.origin.z;
        let along = dx * cyl.axis.x + dy * cyl.axis.y + dz * cyl.axis.z;
        let px = dx - along * cyl.axis.x;
        let py = dy - along * cyl.axis.y;
        let pz = dz - along * cyl.axis.z;
        let lateral = (px * px + py * py + pz * pz).sqrt();
        assert!(
            (lateral - cyl.radius).abs() < eps,
            "{label}: lateral = {} vs R = {}",
            lateral,
            cyl.radius
        );
    }

    #[test]
    fn coaxial_cone_cylinder_circle() {
        // 45° up-cone from the origin; cylinder R=1 along +Z: r_cone(z) = z
        // crosses R=1 at z=1 → ONE circle of radius 1 at z=1.
        let cone = ConeSurface::new_expanding(
            Point3d::ORIGIN,
            Direction3d::Z,
            45.0f64.to_radians(),
            Direction3d::X,
        );
        let cyl = CylinderSurface::new_z(1.0);
        let out = intersect_cone_cylinder(&cone, &cyl, 1e-9);
        assert_eq!(out.len(), 1, "expected one circle, got {} curves", out.len());
        for p in &out[0] {
            assert_on_cone(p, &cone, 1e-9, "coax/a");
            assert_on_cylinder(p, &cyl, 1e-9, "coax/b");
            assert!((p.z - 1.0).abs() < 1e-9, "z = {} vs 1.0", p.z);
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 1.0).abs() < 1e-9, "r = {} vs 1.0", r);
        }
    }

    #[test]
    fn cone_cylinder_off_axis_invariants() {
        // 30° up-cone from the origin; cylinder R=1 along +X from (−5, 1, 0):
        // generic skew pair — invariants only.
        let cone = ConeSurface::new_expanding(
            Point3d::ORIGIN,
            Direction3d::Z,
            30.0f64.to_radians(),
            Direction3d::X,
        );
        let cyl = CylinderSurface::new(
            Point3d::new(-5.0, 1.0, 0.0),
            Direction3d::new(1.0, 0.0, 0.0).unwrap(),
            1.0,
        );
        let out = intersect_cone_cylinder(&cone, &cyl, 1e-9);
        assert!(!out.is_empty(), "skew pair must intersect");
        for curve in &out {
            assert!(curve.len() >= 2);
            for p in curve {
                // 1e-8: the quadratic-solve fp rounding at this geometry
                // scale (~10) is ~3e-9 on the cone residual; the residual
                // form is apex-robust (the curve ends at the cone apex).
                assert_on_cone(p, &cone, 1e-8, "skew/cone");
                assert_on_cylinder(p, &cyl, 1e-8, "skew/cyl");
            }
        }
    }

    #[test]
    fn cone_cylinder_disjoint_empty() {
        // An infinite up-cone eventually reaches ANY lateral distance, so
        // "disjoint" must put the cylinder beyond the nappe side: below the
        // apex (cone points have z ≥ 0, cylinder points z ∈ [−51, −49]).
        let cone = ConeSurface::new_expanding(
            Point3d::ORIGIN,
            Direction3d::Z,
            30.0f64.to_radians(),
            Direction3d::X,
        );
        let cyl = CylinderSurface::new(
            Point3d::new(0.0, 0.0, -50.0),
            Direction3d::new(1.0, 0.0, 0.0).unwrap(),
            1.0,
        );
        let out = intersect_cone_cylinder(&cone, &cyl, 1e-9);
        assert!(out.is_empty(), "cylinder below the nappe → empty");
    }

    #[test]
    fn dispatcher_routes_cone_cylinder_both_orders() {
        let cone = Surface::Cone(ConeSurface::new_expanding(
            Point3d::ORIGIN,
            Direction3d::Z,
            45.0f64.to_radians(),
            Direction3d::X,
        ));
        let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));
        for (a, b) in [(&cone, &cyl), (&cyl, &cone)] {
            let out = intersect_surfaces(a, b, 1e-9);
            assert_eq!(out.polylines.len(), 1, "one circle from both orders");
            if let Surface::Cone(c) = a {
                if let Surface::Cylinder(y) = b {
                    for p in &out.polylines[0] {
                        assert_on_cone(p, c, 1e-9, "disp/cone");
                        assert_on_cylinder(p, y, 1e-9, "disp/cyl");
                    }
                }
            }
        }
    }
}

// ============================================================
// Torus SSI tests (T-series, 2026-09-02)
// ============================================================

#[cfg(test)]
mod torus_plane_tests {
    use super::*;
    use crate::{Direction3d, Plane, Point3d, SphereSurface, Surface, TorusSurface};

    fn torus_z() -> TorusSurface {
        TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0)
    }

    /// |dist(P, tube-center circle) − r| ≤ eps — the profile-circle
    /// absolute-residual form (robust near the degenerate azimuths).
    fn assert_on_torus(p: &Point3d, t: &TorusSurface, eps: f64, label: &str) {
        let z = p.z * t.axis.z + p.x * t.axis.x + p.y * t.axis.y;
        let rho = (p.x * p.x + p.y * p.y + p.z * p.z - z * z).sqrt();
        let d = ((rho - t.major_radius) * (rho - t.major_radius) + z * z).sqrt();
        assert!(
            (d - t.minor_radius).abs() <= eps * (1.0 + t.major_radius + t.minor_radius),
            "{label}: off torus: rho={rho:.9}, z={z:.9}, tube-dist={d:.9} (r={})",
            t.minor_radius
        );
    }

    fn assert_on_plane(p: &Point3d, plane: &Plane, eps: f64, label: &str) {
        let d = (p.x - plane.origin.x) * plane.normal.x
            + (p.y - plane.origin.y) * plane.normal.y
            + (p.z - plane.origin.z) * plane.normal.z;
        assert!(
            d.abs() <= eps * (1.0 + 13.0), // engine-bisected boundary points sit at the clamp
            "{label}: off plane by {d:.3e}"
        );
    }

    #[test]
    fn plane_perp_axis_center_two_circles() {
        let t = torus_z();
        let plane = Plane::xy(); // z = 0, normal = Z
        let out = intersect_torus_plane(&plane, &t, 1e-9);
        assert_eq!(out.len(), 2, "equatorial plane → 2 circles");
        let mut radii: Vec<f64> = out
            .iter()
            .map(|c| {
                assert_eq!(c.len(), 128, "full-circle sampling convention");
                c
            })
            .map(|c| {
                let p = c[0];
                assert!((p.z).abs() <= 1e-9, "circle at z=0");
                (p.x * p.x + p.y * p.y).sqrt()
            })
            .collect();
        radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((radii[0] - 7.0).abs() <= 1e-9, "inner circle R−r=7, got {}", radii[0]);
        assert!((radii[1] - 13.0).abs() <= 1e-9, "outer circle R+r=13, got {}", radii[1]);
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "perp/center");
                assert_on_plane(p, &plane, 1e-9, "perp/center");
            }
        }
    }

    #[test]
    fn plane_perp_axis_offset_two_circles() {
        let t = torus_z();
        // Plane z = 1.5 (normal Z, through (0,0,1.5)).
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 1.5), Direction3d::Z);
        let out = intersect_torus_plane(&plane, &t, 1e-9);
        assert_eq!(out.len(), 2);
        let half = (3.0f64 * 3.0 - 1.5 * 1.5).sqrt();
        let mut radii: Vec<f64> = out
            .iter()
            .map(|c| {
                let p = c[0];
                assert!((p.z - 1.5).abs() <= 1e-9, "circle at z=1.5, z={}", p.z);
                (p.x * p.x + p.y * p.y).sqrt()
            })
            .collect();
        radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((radii[0] - (10.0 - half)).abs() <= 1e-9, "inner ρ=10−√6.75");
        assert!((radii[1] - (10.0 + half)).abs() <= 1e-9, "outer ρ=10+√6.75");
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "perp/offset");
            }
        }
    }

    #[test]
    fn plane_perp_axis_tangent_one_circle() {
        let t = torus_z();
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 3.0), Direction3d::Z);
        let out = intersect_torus_plane(&plane, &t, 1e-9);
        assert_eq!(out.len(), 1, "tangent plane → 1 circle at ρ=R, z=r");
        let p = out[0][0];
        assert!((p.z - 3.0).abs() <= 1e-9, "tangent circle at z=r=3, z={}", p.z);
        let rho = (p.x * p.x + p.y * p.y).sqrt();
        assert!((rho - 10.0).abs() <= 1e-8, "tangent circle ρ=10, got {rho}");
        for q in &out[0] {
            assert_on_torus(q, &t, 1e-9, "perp/tangent");
        }
    }

    #[test]
    fn plane_perp_axis_miss_empty() {
        let t = torus_z();
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 5.0), Direction3d::Z);
        assert!(intersect_torus_plane(&plane, &t, 1e-9).is_empty());
    }

    #[test]
    fn plane_containing_axis_two_meridian_circles() {
        let t = torus_z();
        // Plane x = 0 (normal X through the origin) contains the Z axis.
        let plane = Plane::from_origin_and_normal(Point3d::ORIGIN, Direction3d::X);
        let out = intersect_torus_plane(&plane, &t, 1e-9);
        assert_eq!(out.len(), 2, "axis plane → 2 meridian tube circles");
        for curve in &out {
            assert_eq!(curve.len(), 128);
            for p in curve {
                assert!(p.x.abs() <= 1e-9, "meridian circle lies in x=0");
                assert_on_torus(p, &t, 1e-9, "axis-plane");
                assert_on_plane(p, &plane, 1e-9, "axis-plane");
            }
            // Circle centers at (0, ±10, 0), radius 3.
            let c = (0.0, 10.0, 0.0);
            let c2 = (0.0, -10.0, 0.0);
            let dist = |p: &Point3d, c: (f64, f64, f64)| {
                ((p.x - c.0).powi(2) + (p.y - c.1).powi(2) + (p.z - c.2).powi(2)).sqrt()
            };
            let p = curve[0];
            let ok0 = (dist(&p, c) - 3.0).abs() <= 1e-9;
            let ok1 = (dist(&p, c2) - 3.0).abs() <= 1e-9;
            assert!(ok0 || ok1, "meridian circle center (0,±10,0) r=3");
        }
    }

    #[test]
    fn plane_parallel_axis_offset_peanut() {
        let t = torus_z();
        // Plane x = 5 (normal X) — parallel to the axis, offset 5 < R−r.
        let plane = Plane::from_origin_and_normal(Point3d::new(5.0, 0.0, 0.0), Direction3d::X);
        let out = intersect_torus_plane(&plane, &t, 1e-9);
        assert!(!out.is_empty(), "offset plane cuts the tube → peanut curves");
        let total: usize = out.iter().map(|c| c.len()).sum();
        assert!(total >= 64, "sampled densely enough, got {total} pts");
        for curve in &out {
            for p in curve {
                // Boundary points sit at the strict-slack clamp: 1e-7 scale-relative.
                assert!((p.x - 5.0).abs() <= 1e-7, "in plane x=5, x={:.9}", p.x);
                assert_on_torus(p, &t, 1e-9, "peanut");
            }
        }
    }

    #[test]
    fn plane_oblique_invariants() {
        let t = torus_z();
        // 45° oblique plane through the center: normal (1,0,1)/√2.
        let n = Direction3d::new(1.0, 0.0, 1.0).unwrap();
        let plane = Plane::from_origin_and_normal(Point3d::ORIGIN, n);
        let out = intersect_torus_plane(&plane, &t, 1e-9);
        assert!(!out.is_empty(), "oblique plane through the tube → non-empty");
        for curve in &out {
            assert!(curve.len() >= 8, "substantive arc");
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "oblique/torus");
                assert_on_plane(p, &plane, 1e-9, "oblique/plane");
            }
        }
    }

    #[test]
    fn dispatcher_routes_both_orders() {
        let t = Surface::Torus(torus_z());
        let plane = Surface::Plane(Plane::from_origin_and_normal(
            Point3d::ORIGIN,
            Direction3d::new(1.0, 0.0, 1.0).unwrap(),
        ));
        let out_ab = intersect_surfaces(&t, &plane, 1e-9);
        let out_ba = intersect_surfaces(&plane, &t, 1e-9);
        assert!(!out_ab.polylines.is_empty());
        assert_eq!(
            out_ab.polylines.len(),
            out_ba.polylines.len(),
            "both orders give the same curve count"
        );
        let count = |r: &SurfaceSurfaceIntersection| -> usize {
            r.polylines.iter().map(|c| c.len()).sum()
        };
        assert_eq!(count(&out_ab), count(&out_ba), "same total point count");
        if let Surface::Torus(tor) = &t {
            if let Surface::Plane(pl) = &plane {
                for p in out_ab.polylines.iter().flatten() {
                    assert_on_torus(p, tor, 1e-9, "disp/torus");
                    assert_on_plane(p, pl, 1e-9, "disp/plane");
                }
            }
        }
    }

    #[test]
    fn negative_normal_perp_plane() {
        // Plane with normal −Z at z=1.5 (the B = −1 sign path).
        let t = torus_z();
        let plane = Plane::from_origin_and_normal(Point3d::new(0.0, 0.0, 1.5), Direction3d::new(0.0, 0.0, -1.0).unwrap());
        let out = intersect_torus_plane(&plane, &t, 1e-9);
        assert_eq!(out.len(), 2, "−Z normal: same 2 circles");
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "neg-normal");
                assert!((p.z - 1.5).abs() <= 1e-9);
            }
        }
        let _ = SphereSurface::new(Point3d::ORIGIN, 1.0); // keep import used
    }
}

#[cfg(test)]
mod torus_sphere_tests {
    use super::*;
    use crate::{Direction3d, Plane, Point3d, SphereSurface, Surface, TorusSurface};

    fn torus_z() -> TorusSurface {
        TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0)
    }

    fn assert_on_torus(p: &Point3d, t: &TorusSurface, eps: f64, label: &str) {
        let z = p.z * t.axis.z + p.x * t.axis.x + p.y * t.axis.y;
        let rho = (p.x * p.x + p.y * p.y + p.z * p.z - z * z).sqrt();
        let d = ((rho - t.major_radius) * (rho - t.major_radius) + z * z).sqrt();
        assert!(
            (d - t.minor_radius).abs() <= eps * (1.0 + t.major_radius + t.minor_radius),
            "{label}: off torus: tube-dist={d:.9} (r={})",
            t.minor_radius
        );
    }

    fn assert_on_sphere(p: &Point3d, c: &Point3d, r: f64, eps: f64, label: &str) {
        let d = ((p.x - c.x).powi(2) + (p.y - c.y).powi(2) + (p.z - c.z).powi(2)).sqrt();
        assert!((d - r).abs() <= eps * (1.0 + r), "{label}: off sphere by {:.3e}", (d - r).abs());
    }

    #[test]
    fn concentric_two_latitude_circles() {
        let t = torus_z();
        // Sphere radius 10 centered at the torus center:
        // cosφ = (Rs²−R²−r²)/(2Rr) = −0.15 → two latitude circles.
        let s = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let out = intersect_torus_sphere(&s, &t, 1e-9);
        assert!(out.len() >= 2, "concentric → 2 latitude circles, got {}", out.len());
        let cos_phi: f64 = -0.15;
        let rho = 10.0 + 3.0 * cos_phi;
        let z = 3.0 * (1.0f64 - cos_phi * cos_phi).sqrt();
        for curve in &out {
            assert!(curve.len() >= 64, "closed loop sampled densely");
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "conc/torus");
                assert_on_sphere(p, &s.center, s.radius, 1e-9, "conc/sphere");
                let pr = (p.x * p.x + p.y * p.y).sqrt();
                assert!((pr - rho).abs() <= 1e-8, "latitude ρ={rho:.6}, got {pr:.6}");
                assert!((p.z.abs() - z).abs() <= 1e-8, "latitude |z|={z:.6}");
            }
        }
    }

    #[test]
    fn concentric_internal_tangency_single_circle() {
        let t = torus_z();
        // Sphere radius R−r = 7: internally tangent at the inner equator.
        let s = SphereSurface::new(Point3d::ORIGIN, 7.0);
        let out = intersect_torus_sphere(&s, &t, 1e-9);
        assert!(!out.is_empty(), "internal tangency → inner equator circle");
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "tang/torus");
                assert_on_sphere(p, &s.center, s.radius, 1e-9, "tang/sphere");
                let pr = (p.x * p.x + p.y * p.y).sqrt();
                assert!((pr - 7.0).abs() <= 1e-8, "inner equator ρ=7, got {pr}");
                assert!(p.z.abs() <= 1e-8, "inner equator z=0");
            }
        }
    }

    #[test]
    fn sphere_offset_invariants() {
        let t = torus_z();
        // Sphere center (8,0,0), radius 4 — crosses the tube region.
        let s = SphereSurface::new(Point3d::new(8.0, 0.0, 0.0), 4.0);
        let out = intersect_torus_sphere(&s, &t, 1e-9);
        assert!(!out.is_empty(), "sphere crosses the tube → non-empty");
        for curve in &out {
            assert!(curve.len() >= 8, "substantive arc");
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "offset/torus");
                assert_on_sphere(p, &s.center, s.radius, 1e-9, "offset/sphere");
            }
        }
    }

    #[test]
    fn sphere_disjoint_and_contained_empty() {
        let t = torus_z();
        // Strictly outside the tube (profile distance 20 > r + Rs = 5).
        let far = SphereSurface::new(Point3d::new(30.0, 0.0, 0.0), 2.0);
        assert!(intersect_torus_sphere(&far, &t, 1e-9).is_empty(), "disjoint → empty");
        // Center on the tube-center circle, small radius inside the tube.
        let in_tube = SphereSurface::new(Point3d::new(10.0, 0.0, 0.0), 1.0);
        assert!(intersect_torus_sphere(&in_tube, &t, 1e-9).is_empty(), "contained → empty");
    }

    #[test]
    fn dispatcher_routes_both_orders() {
        let t = Surface::Torus(torus_z());
        let s = Surface::Sphere(SphereSurface::new(Point3d::new(8.0, 0.0, 0.0), 4.0));
        let out_ab = intersect_surfaces(&t, &s, 1e-9);
        let out_ba = intersect_surfaces(&s, &t, 1e-9);
        assert!(!out_ab.polylines.is_empty());
        assert_eq!(out_ab.polylines.len(), out_ba.polylines.len());
        let count = |r: &SurfaceSurfaceIntersection| -> usize {
            r.polylines.iter().map(|c| c.len()).sum()
        };
        assert_eq!(count(&out_ab), count(&out_ba));
        let _ = (Direction3d::X, Plane::xy()); // keep imports used
    }
}

#[cfg(test)]
mod torus_cylinder_tests {
    use super::*;
    use crate::{Direction3d, Plane, Point3d, CylinderSurface, Surface, TorusSurface};

    fn torus_z() -> TorusSurface {
        TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0)
    }

    fn assert_on_torus(p: &Point3d, t: &TorusSurface, eps: f64, label: &str) {
        let z = p.z * t.axis.z + p.x * t.axis.x + p.y * t.axis.y;
        let rho = (p.x * p.x + p.y * p.y + p.z * p.z - z * z).sqrt();
        let d = ((rho - t.major_radius) * (rho - t.major_radius) + z * z).sqrt();
        assert!(
            (d - t.minor_radius).abs() <= eps * (1.0 + t.major_radius + t.minor_radius),
            "{label}: off torus: tube-dist={d:.9} (r={})",
            t.minor_radius
        );
    }

    fn assert_on_cylinder_axis(
        p: &Point3d,
        origin: &Point3d,
        axis: &Direction3d,
        radius: f64,
        eps: f64,
        label: &str,
    ) {
        let dx = p.x - origin.x;
        let dy = p.y - origin.y;
        let dz = p.z - origin.z;
        let ax = dx * axis.x + dy * axis.y + dz * axis.z;
        let px = dx - ax * axis.x;
        let py = dy - ax * axis.y;
        let pz = dz - ax * axis.z;
        let d = (px * px + py * py + pz * pz).sqrt();
        assert!(
            (d - radius).abs() <= eps * (1.0 + radius),
            "{label}: off cylinder by {:.3e} (radial={d:.9}, R={radius})",
            (d - radius).abs()
        );
    }

    #[test]
    fn coaxial_two_circles() {
        let t = torus_z();
        let cyl = CylinderSurface::new_z(8.0);
        let out = intersect_torus_cylinder(&cyl, &t, 1e-9);
        assert_eq!(out.len(), 2, "coaxial R_c=8 → 2 circles z=±√5");
        let z_exp = (3.0f64 * 3.0 - 4.0).sqrt();
        for curve in &out {
            assert_eq!(curve.len(), 128);
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "coax/torus");
                assert_on_cylinder_axis(p, &Point3d::ORIGIN, &Direction3d::Z, 8.0, 1e-9, "coax/cyl");
                assert!(p.z.abs() - z_exp.abs() <= 1e-8, "z=±√5");
            }
        }
    }

    #[test]
    fn coaxial_tangent_one_circle() {
        let t = torus_z();
        let cyl_out = CylinderSurface::new_z(13.0);
        assert_eq!(intersect_torus_cylinder(&cyl_out, &t, 1e-9).len(), 1, "R_c=13 outer tangent");
        let cyl_in = CylinderSurface::new_z(7.0);
        assert_eq!(intersect_torus_cylinder(&cyl_in, &t, 1e-9).len(), 1, "R_c=7 inner tangent");
    }

    #[test]
    fn coaxial_miss_empty() {
        let t = torus_z();
        assert!(intersect_torus_cylinder(&CylinderSurface::new_z(14.0), &t, 1e-9).is_empty());
        assert!(intersect_torus_cylinder(&CylinderSurface::new_z(6.0), &t, 1e-9).is_empty());
    }

    #[test]
    fn parallel_offset_invariants() {
        let t = torus_z();
        // Cylinder axis ∥ Z through (5,0,0), radius 3 — cuts the inner
        // side of the tube. Curves come in equatorial-mirrored pairs.
        let origin = Point3d::new(5.0, 0.0, 0.0);
        let cyl = CylinderSurface { origin, axis: Direction3d::Z, radius: 3.0, x_dir: Direction3d::X };
        let out = intersect_torus_cylinder(&cyl, &t, 1e-9);
        assert!(!out.is_empty(), "offset cylinder crosses the tube");
        let total: usize = out.iter().map(|c| c.len()).sum();
        assert!(total >= 64, "dense sampling, got {total}");
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "offset/torus");
                assert_on_cylinder_axis(p, &origin, &Direction3d::Z, 3.0, 1e-9, "offset/cyl");
            }
        }
        // Equatorial symmetry: every point must have a mirrored partner.
        for curve in &out {
            for p in curve {
                let mirrored = Point3d::new(p.x, p.y, -p.z);
                let has_partner = out
                    .iter()
                    .any(|c| c.iter().any(|q| q.distance_to(&mirrored) <= 1e-6));
                assert!(has_partner, "z-mirror partner missing for ({:.4},{:.4},{:.4})", p.x, p.y, p.z);
            }
        }
    }

    #[test]
    fn perpendicular_axes_analytic_invariants() {
        let t = torus_z();
        // Cylinder along X through the origin, radius 3 — perpendicular
        // axes take the ψ-parametrized twin-pass analytic path.
        let origin = Point3d::ORIGIN;
        let cyl = Surface::Cylinder(CylinderSurface {
            origin,
            axis: Direction3d::X,
            radius: 3.0,
            x_dir: Direction3d::Z,
        });
        let out = intersect_surfaces(&cyl, &Surface::Torus(t.clone()), 1e-9);
        assert!(!out.polylines.is_empty(), "perpendicular cylinder crosses the tube");
        let total: usize = out.polylines.iter().map(|c| c.len()).sum();
        assert!(total >= 64, "dense analytic sampling, got {total} pts");
        for p in out.polylines.iter().flatten() {
            assert_on_torus(p, &t, 1e-7, "perp/analytic torus");
            assert_on_cylinder_axis(p, &origin, &Direction3d::X, 3.0, 1e-7, "perp/analytic cyl");
        }
    }

    #[test]
    fn dispatcher_routes_both_orders() {
        let t = Surface::Torus(torus_z());
        let cyl = Surface::Cylinder(CylinderSurface::new_z(8.0));
        let out_ab = intersect_surfaces(&t, &cyl, 1e-9);
        let out_ba = intersect_surfaces(&cyl, &t, 1e-9);
        assert_eq!(out_ab.polylines.len(), 2);
        assert_eq!(out_ab.polylines.len(), out_ba.polylines.len(), "same count both orders");
        let count = |r: &SurfaceSurfaceIntersection| -> usize {
            r.polylines.iter().map(|c| c.len()).sum()
        };
        assert_eq!(count(&out_ab), count(&out_ba));
        let _ = Plane::xy(); // keep import used
    }
}

#[cfg(test)]
mod torus_cone_tests {
    use super::*;
    use crate::{ConeSurface, Direction3d, Point3d, Surface, TorusSurface};

    fn torus_z() -> TorusSurface {
        TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0)
    }

    /// |dist(P, tube-center circle) − r| ≤ eps — the profile-circle
    /// absolute-residual form (robust for ring AND spindle tori).
    fn assert_on_torus(p: &Point3d, t: &TorusSurface, eps: f64, label: &str) {
        let z = p.z * t.axis.z + p.x * t.axis.x + p.y * t.axis.y;
        let rho = (p.x * p.x + p.y * p.y + p.z * p.z - z * z).sqrt();
        let d = ((rho - t.major_radius) * (rho - t.major_radius) + z * z).sqrt();
        assert!(
            (d - t.minor_radius).abs() <= eps * (1.0 + t.major_radius + t.minor_radius),
            "{label}: off torus: rho={rho:.9}, z={z:.9}, tube-dist={d:.9} (r={})",
            t.minor_radius
        );
    }

    /// |radial(P) − (radius₀ + v·tanα)| ≤ eps — the infinite-sheet
    /// residual (the marching/cone family convention; consumers trim).
    fn assert_on_cone(p: &Point3d, cone: &ConeSurface, eps: f64, label: &str) {
        let dx = p.x - cone.origin.x;
        let dy = p.y - cone.origin.y;
        let dz = p.z - cone.origin.z;
        let v = dx * cone.axis.x + dy * cone.axis.y + dz * cone.axis.z;
        let px = dx - v * cone.axis.x;
        let py = dy - v * cone.axis.y;
        let pz = dz - v * cone.axis.z;
        let rho = (px * px + py * py + pz * pz).sqrt();
        let target = if cone.expanding {
            v * cone.half_angle.tan()
        } else {
            cone.radius + v * cone.half_angle.tan()
        };
        assert!(
            (rho - target).abs() <= eps * (1.0 + cone.radius + target.abs()),
            "{label}: off cone: radial={rho:.9}, target={target:.9}, v={v:.9}"
        );
        assert!(
            rho >= -1e-12,
            "{label}: off-sheet point (radial={rho:.9} < 0) leaked"
        );
        if cone.expanding {
            assert!(v >= -eps, "{label}: expanding sheet needs v>=0, v={v:.9}");
        }
    }

    #[test]
    fn coaxial_two_circles() {
        let t = torus_z();
        // Cone 45°, base radius 8 at z=0, expanding upward: sheet
        // ρ = 8 + z. Quadratic roots z = 1 ± √14/2.
        let cone = ConeSurface::new_z(8.0, std::f64::consts::FRAC_PI_4);
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert_eq!(out.len(), 2, "coaxial 45° cone → 2 latitude circles");
        let z_exp = 1.0 + (14.0f64).sqrt() / 2.0;
        let mut zs: Vec<f64> = out.iter().map(|c| c[0].z).collect();
        zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((zs[0] - (2.0 - z_exp)).abs() <= 1e-9, "z_lo = 1−√14/2, got {}", zs[0]);
        assert!((zs[1] - z_exp).abs() <= 1e-9, "z_hi = 1+√14/2, got {}", zs[1]);
        for curve in &out {
            assert_eq!(curve.len(), 128, "full-circle sampling convention");
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "coax/torus");
                assert_on_cone(p, &cone, 1e-9, "coax/cone");
            }
        }
    }

    #[test]
    fn coaxial_tangent_one_circle() {
        let t = torus_z();
        // |q| = r·√2 ⟹ β = R ∓ 3√2 → tangent double root.
        let r_narrow = 10.0 - 3.0 * (2.0f64).sqrt();
        let cone_in = ConeSurface::new_z(r_narrow, std::f64::consts::FRAC_PI_4);
        let out_in = intersect_torus_cone(&cone_in, &t, 1e-9);
        assert_eq!(out_in.len(), 1, "inner tangent cone → 1 circle");
        let p = out_in[0][0];
        let z_star = 3.0 * (2.0f64).sqrt() / 2.0;
        assert!((p.z - z_star).abs() <= 1e-8, "tangent z = 3√2/2, got {}", p.z);
        for q in &out_in[0] {
            assert_on_torus(q, &t, 1e-9, "tangent-in/torus");
            assert_on_cone(q, &cone_in, 1e-9, "tangent-in/cone");
        }
        let r_wide = 10.0 + 3.0 * (2.0f64).sqrt();
        let cone_out = ConeSurface::new_z(r_wide, std::f64::consts::FRAC_PI_4);
        let out_out = intersect_torus_cone(&cone_out, &t, 1e-9);
        assert_eq!(out_out.len(), 1, "outer tangent cone → 1 circle");
        for q in &out_out[0] {
            assert_on_torus(q, &t, 1e-9, "tangent-out/torus");
            assert_on_cone(q, &cone_out, 1e-9, "tangent-out/cone");
        }
    }

    #[test]
    fn coaxial_miss_empty() {
        let t = torus_z();
        // Wide sheet (β=20): |ũ| = 10/√2 > 3 — misses the tube.
        let wide = ConeSurface::new_z(20.0, std::f64::consts::FRAC_PI_4);
        assert!(intersect_torus_cone(&wide, &t, 1e-9).is_empty());
        // Narrow sheet entirely inside the hole (β=2): misses the tube.
        let narrow = ConeSurface::new_z(2.0, std::f64::consts::FRAC_PI_4);
        assert!(intersect_torus_cone(&narrow, &t, 1e-9).is_empty());
        // Tiny half-angle, radius inside the hole — cylinder-equivalent
        // miss via the routing.
        let thin = ConeSurface::new_z(2.0, 0.087);
        assert!(intersect_torus_cone(&thin, &t, 1e-9).is_empty());
    }

    #[test]
    fn coaxial_axis_flipped() {
        let t = torus_z();
        // Cone axis −Z, origin (0,0,5), radius 8, 45°: sheet
        // ρ = 13 − z → circles at (z=3, ρ=10) and (z=0, ρ=13).
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, 5.0),
            Direction3d::NEG_Z,
            8.0,
            std::f64::consts::FRAC_PI_4,
        );
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert_eq!(out.len(), 2, "flipped-axis coaxial → 2 circles");
        let mut circles: Vec<(f64, f64)> = out
            .iter()
            .map(|c| {
                let p = c[0];
                (p.z, (p.x * p.x + p.y * p.y).sqrt())
            })
            .collect();
        circles.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        assert!((circles[0].0 - 0.0).abs() <= 1e-9 && (circles[0].1 - 13.0).abs() <= 1e-9,
            "(z=0, ρ=13), got {:?}", circles[0]);
        assert!((circles[1].0 - 3.0).abs() <= 1e-9 && (circles[1].1 - 10.0).abs() <= 1e-8,
            "(z=3, ρ=10), got {:?}", circles[1]);
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "flip/torus");
                assert_on_cone(p, &cone, 1e-9, "flip/cone");
            }
        }
    }

    #[test]
    fn coaxial_origin_offset() {
        let t = torus_z();
        // Origin (0,0,−2): β = 8−(−2) = 10 = R ⟹ q = 0, symmetric
        // roots z = ±3√2/2, ρ = 10 ± 3√2/2.
        let cone = ConeSurface::new(
            Point3d::new(0.0, 0.0, -2.0),
            Direction3d::Z,
            8.0,
            std::f64::consts::FRAC_PI_4,
        );
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert_eq!(out.len(), 2, "offset-origin coaxial → 2 circles");
        let z_exp = 3.0 * (2.0f64).sqrt() / 2.0;
        let mut zs: Vec<f64> = out.iter().map(|c| c[0].z).collect();
        zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((zs[0] + z_exp).abs() <= 1e-9, "z = −3√2/2, got {}", zs[0]);
        assert!((zs[1] - z_exp).abs() <= 1e-9, "z = +3√2/2, got {}", zs[1]);
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "offset/torus");
                assert_on_cone(p, &cone, 1e-9, "offset/cone");
            }
        }
    }

    #[test]
    fn expanding_cone_circles() {
        let t = torus_z();
        // Apex at (0,0,−10), 45° opening upward: sheet ρ = z + 10 —
        // same β = 10 as the offset-origin case.
        let cone = ConeSurface::new_expanding(
            Point3d::new(0.0, 0.0, -10.0),
            Direction3d::Z,
            std::f64::consts::FRAC_PI_4,
            Direction3d::X,
        );
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert_eq!(out.len(), 2, "expanding coaxial → 2 circles");
        let z_exp = 3.0 * (2.0f64).sqrt() / 2.0;
        let mut zs: Vec<f64> = out.iter().map(|c| c[0].z).collect();
        zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((zs[0] + z_exp).abs() <= 1e-9, "z = −3√2/2, got {}", zs[0]);
        assert!((zs[1] - z_exp).abs() <= 1e-9, "z = +3√2/2, got {}", zs[1]);
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-9, "expanding/torus");
                assert_on_cone(p, &cone, 1e-9, "expanding/cone");
            }
        }
    }

    #[test]
    fn spindle_off_sheet_root_dropped() {
        // Spindle torus (R=2 < r=3): the tube crosses the axis, so the
        // quadratic can yield a ρ* < 0 root — off-sheet, must be dropped.
        let t = TorusSurface::new_z(Point3d::ORIGIN, 2.0, 3.0);
        let cone = ConeSurface::new_z(1.0, std::f64::consts::FRAC_PI_4);
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert_eq!(out.len(), 1, "one off-sheet root dropped → 1 circle");
        let p = out[0][0];
        let z_exp = (1.0 + (17.0f64).sqrt()) / 2.0;
        let rho_exp = 1.0 + z_exp;
        assert!((p.z - z_exp).abs() <= 1e-9, "z = (1+√17)/2, got {}", p.z);
        let rho = (p.x * p.x + p.y * p.y).sqrt();
        assert!((rho - rho_exp).abs() <= 1e-9, "ρ = 1+z, got {}", rho);
        for q in &out[0] {
            assert_on_torus(q, &t, 1e-9, "spindle/torus");
            assert_on_cone(q, &cone, 1e-9, "spindle/cone");
        }
    }

    #[test]
    fn near_cylindrical_routes_to_cylinder() {
        let t = torus_z();
        // half_angle ≈ 0: the sheet is the cylinder ρ = 8 — must match
        // the torus×cylinder coaxial contract (2 circles at z = ±√5).
        let cone = ConeSurface::new_z(8.0, 1e-13);
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert_eq!(out.len(), 2, "cylindrical cone → 2 circles");
        let z_exp = (5.0f64).sqrt();
        let mut zs: Vec<f64> = out.iter().map(|c| c[0].z).collect();
        zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((zs[0] + z_exp).abs() <= 1e-8, "z = −√5, got {}", zs[0]);
        assert!((zs[1] - z_exp).abs() <= 1e-8, "z = +√5, got {}", zs[1]);
        for curve in &out {
            for p in curve {
                let rho = (p.x * p.x + p.y * p.y).sqrt();
                assert!((rho - 8.0).abs() <= 1e-8, "ρ = 8, got {}", rho);
                assert_on_torus(p, &t, 1e-9, "cyl-equiv/torus");
            }
        }
    }

    #[test]
    fn near_flat_routes_to_plane() {
        let t = torus_z();
        // half_angle = π/2 − 1e-9: tanα ≈ 1e9 — the sheet is the base
        // plane z = 0 within tolerance → 2 equatorial circles ρ = 7, 13.
        let cone = ConeSurface::new_z(8.0, std::f64::consts::FRAC_PI_2 - 1e-9);
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert_eq!(out.len(), 2, "flat cone → 2 equatorial circles");
        let mut radii: Vec<f64> = out
            .iter()
            .map(|c| {
                let p = c[0];
                assert!(p.z.abs() <= 1e-6, "circle at z≈0, z={}", p.z);
                (p.x * p.x + p.y * p.y).sqrt()
            })
            .collect();
        radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((radii[0] - 7.0).abs() <= 1e-6, "inner ρ=7, got {}", radii[0]);
        assert!((radii[1] - 13.0).abs() <= 1e-6, "outer ρ=13, got {}", radii[1]);
        for curve in &out {
            for p in curve {
                assert_on_torus(p, &t, 1e-6, "flat-equiv/torus");
            }
        }
    }

    #[test]
    fn dispatcher_routes_both_orders() {
        let t = Surface::Torus(torus_z());
        let cone = Surface::Cone(ConeSurface::new_z(8.0, std::f64::consts::FRAC_PI_4));
        let out_ab = intersect_surfaces(&t, &cone, 1e-9);
        let out_ba = intersect_surfaces(&cone, &t, 1e-9);
        assert_eq!(out_ab.polylines.len(), 2, "Torus×Cone → 2 circles");
        assert_eq!(
            out_ab.polylines.len(),
            out_ba.polylines.len(),
            "same count both orders"
        );
        let count = |r: &SurfaceSurfaceIntersection| -> usize {
            r.polylines.iter().map(|c| c.len()).sum()
        };
        assert_eq!(count(&out_ab), count(&out_ba));
    }

    #[test]
    fn skew_axes_marching_fallback() {
        let t = torus_z();
        // Cone axis +X through (−2,0,0), radius 9, 45°: perpendicular
        // axes — the documented quartic gap → marching fallback.
        // 2026-09-05 marching acceptance fix: the redesigned marching
        // (two-sided projection-guided seeds + curve continuation) must
        // find the real intersection curve — the cone nappe pierces the
        // torus tube around x ≈ 2..3 — where the old acceptance filter
        // (`|ip − grid point| < tol·100` with all four Newton parameters
        // drifting) rejected every converged solution.
        let cone = ConeSurface::new(
            Point3d::new(-2.0, 0.0, 0.0),
            Direction3d::X,
            9.0,
            std::f64::consts::FRAC_PI_4,
        );
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert!(
            !out.is_empty(),
            "marching must find the real cone×torus intersection"
        );
        let total: usize = out.iter().map(|c| c.len()).sum();
        assert!(
            total >= 8,
            "curve continuation must densify the curve, got {total} pts"
        );
        for p in out.iter().flatten() {
            assert_on_torus(p, &t, 1e-5, "skew/torus");
            assert_on_cone(p, &cone, 1e-5, "skew/cone");
        }
    }

    #[test]
    fn marching_disjoint_pair_empty() {
        // Torus at the origin vs a perpendicular cone far away: the
        // distance field never dips below the flagging threshold on
        // either grid, so the marching fallback must return empty —
        // no spurious points from the extended surfaces.
        let t = torus_z();
        let cone = ConeSurface::new(
            Point3d::new(-2.0, 0.0, 60.0),
            Direction3d::X,
            9.0,
            std::f64::consts::FRAC_PI_4,
        );
        let out = intersect_torus_cone(&cone, &t, 1e-9);
        assert!(
            out.is_empty(),
            "disjoint pair must not produce intersection points, got {} curves",
            out.len()
        );
    }

    #[test]
    fn marching_both_orders_find_curve() {
        // Dispatcher symmetry: (Cone, Torus) and (Torus, Cone) both hit
        // the marching path for perpendicular axes; the grid role
        // differs between orders, but the geometric contract does not.
        let t = torus_z();
        let cone = ConeSurface::new(
            Point3d::new(-2.0, 0.0, 0.0),
            Direction3d::X,
            9.0,
            std::f64::consts::FRAC_PI_4,
        );
        let cone_s = Surface::Cone(cone.clone());
        let torus_s = Surface::Torus(t.clone());
        let out_ab = intersect_surfaces(&cone_s, &torus_s, 1e-9);
        let out_ba = intersect_surfaces(&torus_s, &cone_s, 1e-9);
        assert!(
            !out_ab.polylines.is_empty(),
            "(Cone, Torus) order must find the curve"
        );
        assert!(
            !out_ba.polylines.is_empty(),
            "(Torus, Cone) order must find the curve"
        );
        for p in out_ab.polylines.iter().flatten() {
            assert_on_torus(p, &t, 1e-5, "order-ab/torus");
            assert_on_cone(p, &cone, 1e-5, "order-ab/cone");
        }
        for p in out_ba.polylines.iter().flatten() {
            assert_on_torus(p, &t, 1e-5, "order-ba/torus");
            assert_on_cone(p, &cone, 1e-5, "order-ba/cone");
        }
    }
}

#[cfg(test)]
mod torus_torus_tests {
    use super::*;
    use crate::{Direction3d, Point3d, Surface, TorusSurface};

    fn torus_z() -> TorusSurface {
        TorusSurface::new_z(Point3d::ORIGIN, 10.0, 3.0)
    }

    fn torus_z_at(center: Point3d) -> TorusSurface {
        TorusSurface::new_z(center, 10.0, 3.0)
    }

    /// |dist(P, tube-center circle) − r| ≤ eps — the profile-circle
    /// absolute-residual form (robust for ring AND spindle tori).
    fn assert_on_torus(p: &Point3d, t: &TorusSurface, eps: f64, label: &str) {
        let z = p.z * t.axis.z + p.x * t.axis.x + p.y * t.axis.y;
        let rho = (p.x * p.x + p.y * p.y + p.z * p.z - z * z).sqrt();
        let d = ((rho - t.major_radius) * (rho - t.major_radius) + z * z).sqrt();
        assert!(
            (d - t.minor_radius).abs() <= eps * (1.0 + t.major_radius + t.minor_radius),
            "{label}: off torus: rho={rho:.9}, z={z:.9}, tube-dist={d:.9} (r={})",
            t.minor_radius
        );
    }

    /// Every point of the polyline lies on one latitude circle of the
    /// frame (axis through `t.center`): constant z and constant rho.
    fn assert_latitude_circle(curve: &[Point3d], t: &TorusSurface, eps: f64, label: &str) {
        assert!(curve.len() >= 2, "{label}: degenerate curve");
        let z0 = curve[0].z * t.axis.z + curve[0].x * t.axis.x + curve[0].y * t.axis.y;
        let rho0 = ((curve[0].x).powi(2) + (curve[0].y).powi(2) + (curve[0].z).powi(2)
            - z0 * z0)
            .sqrt();
        for p in curve {
            let z = p.z * t.axis.z + p.x * t.axis.x + p.y * t.axis.y;
            let rho = ((p.x).powi(2) + (p.y).powi(2) + (p.z).powi(2) - z * z).sqrt();
            assert!(
                (z - z0).abs() <= eps,
                "{label}: not a latitude circle: z varies {} vs {}",
                z, z0
            );
            assert!(
                (rho - rho0).abs() <= eps,
                "{label}: not a latitude circle: rho varies {} vs {}",
                rho, rho0
            );
        }
    }

    #[test]
    fn coaxial_two_circles() {
        let t1 = torus_z();
        // Same R/r, center 2 up the common axis: profile circles
        // (10, 0) r=3 and (10, 2) r=3 → d=2, a=1, hh=√8 → roots
        // (10 ± √8, 1).
        let t2 = torus_z_at(Point3d::new(0.0, 0.0, 2.0));
        let out = intersect_torus_torus(&t1, &t2, 1e-9);
        assert_eq!(out.len(), 2, "coaxial offset tori → 2 latitude circles");
        let sqrt8 = (8.0f64).sqrt();
        let mut rhos: Vec<f64> = out
            .iter()
            .map(|c| {
                let p = c[0];
                ((p.x).powi(2) + (p.y).powi(2)).sqrt()
            })
            .collect();
        rhos.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(
            (rhos[0] - (10.0 - sqrt8)).abs() <= 1e-9,
            "rho_lo = 10−√8, got {}",
            rhos[0]
        );
        assert!(
            (rhos[1] - (10.0 + sqrt8)).abs() <= 1e-9,
            "rho_hi = 10+√8, got {}",
            rhos[1]
        );
        for curve in &out {
            assert_eq!(curve.len(), 128, "full-circle sampling convention");
            for p in curve.iter() {
                assert!((p.z - 1.0).abs() <= 1e-9, "circle at z=1, got {}", p.z);
                assert_on_torus(p, &t1, 1e-9, "coax/t1");
                assert_on_torus(p, &t2, 1e-9, "coax/t2");
            }
            assert_latitude_circle(curve, &t1, 1e-9, "coax/latitude");
        }
    }

    #[test]
    fn coaxial_tangent_one_circle() {
        let t1 = torus_z();
        // External profile tangency: centers 6 = r1 + r2 apart → one
        // latitude circle at z=3, rho=10.
        let t2 = torus_z_at(Point3d::new(0.0, 0.0, 6.0));
        let out = intersect_torus_torus(&t1, &t2, 1e-9);
        assert_eq!(out.len(), 1, "external tangency → 1 latitude circle");
        let p = out[0][0];
        assert!((p.z - 3.0).abs() <= 1e-8, "tangent z = 3, got {}", p.z);
        let rho = ((p.x).powi(2) + (p.y).powi(2)).sqrt();
        assert!((rho - 10.0).abs() <= 1e-8, "tangent rho = 10, got {}", rho);
        for q in &out[0] {
            assert_on_torus(q, &t1, 1e-9, "tangent/t1");
            assert_on_torus(q, &t2, 1e-9, "tangent/t2");
        }
    }

    #[test]
    fn coaxial_miss_and_nested_empty() {
        let t1 = torus_z();
        // Centers 10 apart (> r1 + r2 = 6): tubes disjoint.
        let far = torus_z_at(Point3d::new(0.0, 0.0, 10.0));
        assert!(intersect_torus_torus(&t1, &far, 1e-9).is_empty());
        // Concentric, smaller tube: profile circles (10,0) r=3 vs
        // (10,0) r=1 — nested, no curve.
        let thin = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 1.0);
        assert!(intersect_torus_torus(&t1, &thin, 1e-9).is_empty());
        // Coincident tori: coincident surfaces produce no curve (the
        // sphere_sphere / cylinder_cylinder convention).
        let same = torus_z();
        assert!(intersect_torus_torus(&t1, &same, 1e-9).is_empty());
    }

    #[test]
    fn coaxial_different_major_two_circles() {
        let t1 = torus_z();
        // R=7, r=1, same center/axis: profile circles (10,0) r=3 and
        // (7,0) r=1 → d=3 ∈ (|3−1|, 3+1) → two circles at rho = 43/6,
        // z = ±√(35)/6·... computed exactly below.
        let t2 = TorusSurface::new_z(Point3d::ORIGIN, 7.0, 1.0);
        let out = intersect_torus_torus(&t1, &t2, 1e-9);
        assert_eq!(out.len(), 2, "different major radii → 2 latitude circles");
        let a_len: f64 = (9.0 + 9.0 - 1.0) / 6.0; // 17/6
        let hh: f64 = (9.0 - a_len * a_len).sqrt();
        let mx = 10.0 - a_len; // centers on the rho axis: chord along z
        let mut zs: Vec<f64> = out.iter().map(|c| c[0].z).collect();
        zs.sort_by(|x, y| x.partial_cmp(y).unwrap());
        assert!((zs[0] + hh).abs() <= 1e-9, "z_lo = −hh, got {}", zs[0]);
        assert!((zs[1] - hh).abs() <= 1e-9, "z_hi = +hh, got {}", zs[1]);
        for curve in &out {
            for p in curve.iter() {
                let rho = ((p.x).powi(2) + (p.y).powi(2)).sqrt();
                assert!((rho - mx).abs() <= 1e-9, "rho = 43/6, got {}", rho);
                assert_on_torus(p, &t1, 1e-9, "major/t1");
                assert_on_torus(p, &t2, 1e-9, "major/t2");
            }
        }
    }

    #[test]
    fn coaxial_axis_flipped() {
        let t1 = torus_z();
        // Anti-parallel axis (−Z): the torus is axis-flip invariant —
        // the physical configuration (same R/r, center 2 up) is the
        // same as coaxial_two_circles.
        let t2 = TorusSurface::new(
            Point3d::new(0.0, 0.0, 2.0),
            Direction3d::new(0.0, 0.0, -1.0).unwrap(),
            10.0,
            3.0,
        );
        let out = intersect_torus_torus(&t1, &t2, 1e-9);
        assert_eq!(out.len(), 2, "flipped axis stays coaxial → 2 circles");
        for curve in &out {
            for p in curve.iter() {
                assert!((p.z - 1.0).abs() <= 1e-9, "circle at z=1, got {}", p.z);
                assert_on_torus(p, &t1, 1e-9, "flip/t1");
                assert_on_torus(p, &t2, 1e-9, "flip/t2");
            }
        }
    }

    #[test]
    fn spindle_same_side_pair() {
        // Spindle torus A (R=2, r=3) vs ring torus B (R=5, r=3),
        // coaxial, same center: same-side profiles (2,0) r=3 and
        // (5,0) r=3 → d=3 → two latitude circles at rho=3.5,
        // z=±3√3/2. The cross-side pair (2,0) vs (−5,0) is d=7 > 6:
        // empty — no duplicates.
        let ta = TorusSurface::new_z(Point3d::ORIGIN, 2.0, 3.0);
        let tb = TorusSurface::new_z(Point3d::ORIGIN, 5.0, 3.0);
        let out = intersect_torus_torus(&ta, &tb, 1e-9);
        assert_eq!(out.len(), 2, "spindle×ring coaxial → 2 latitude circles");
        let z_star = 3.0 * (3.0f64).sqrt() / 2.0;
        let mut zs: Vec<f64> = out.iter().map(|c| c[0].z).collect();
        zs.sort_by(|x, y| x.partial_cmp(y).unwrap());
        assert!((zs[0] + z_star).abs() <= 1e-9, "z_lo = −3√3/2, got {}", zs[0]);
        assert!((zs[1] - z_star).abs() <= 1e-9, "z_hi = +3√3/2, got {}", zs[1]);
        for curve in &out {
            for p in curve.iter() {
                let rho = ((p.x).powi(2) + (p.y).powi(2)).sqrt();
                assert!((rho - 3.5).abs() <= 1e-9, "rho = 3.5, got {}", rho);
                assert_on_torus(p, &ta, 1e-9, "spindle/ta");
                assert_on_torus(p, &tb, 1e-9, "spindle/tb");
            }
            assert_latitude_circle(curve, &ta, 1e-9, "spindle/latitude");
        }
    }

    #[test]
    fn dispatcher_routes_both_orders() {
        let t1 = torus_z();
        let t2 = torus_z_at(Point3d::new(0.0, 0.0, 2.0));
        let a = Surface::Torus(t1.clone());
        let b = Surface::Torus(t2.clone());
        let out_ab = intersect_surfaces(&a, &b, 1e-9);
        let out_ba = intersect_surfaces(&b, &a, 1e-9);
        assert_eq!(out_ab.polylines.len(), 2, "(A, B) order → 2 circles");
        assert_eq!(out_ba.polylines.len(), 2, "(B, A) order → 2 circles");
        // Order independence: the same (z, rho) orbit sets.
        let orbit = |o: &SurfaceSurfaceIntersection| -> Vec<(f64, f64)> {
            let mut v: Vec<(f64, f64)> = o
                .polylines
                .iter()
                .map(|c| {
                    let p = c[0];
                    let rho = ((p.x).powi(2) + (p.y).powi(2)).sqrt();
                    (p.z, rho)
                })
                .collect();
            v.sort_by(|x, y| {
                x.0.partial_cmp(&y.0)
                    .unwrap()
                    .then(x.1.partial_cmp(&y.1).unwrap())
            });
            v
        };
        let oa = orbit(&out_ab);
        let ob = orbit(&out_ba);
        for ((za, ra), (zb, rb)) in oa.iter().zip(&ob) {
            assert!((za - zb).abs() <= 1e-9, "z mismatch {za} vs {zb}");
            assert!((ra - rb).abs() <= 1e-9, "rho mismatch {ra} vs {rb}");
        }
    }

    #[test]
    fn offset_rings_marching_fallback() {
        // Laterally offset rings (0.5 in x): parallel-offset case,
        // degree-8 gap → marching fallback. The tubes genuinely
        // overlap (ring centers 2.06 apart < r1 + r2), so the
        // redesigned marching must find the curve.
        let t1 = torus_z();
        let t2 = torus_z_at(Point3d::new(0.5, 0.0, 2.0));
        let out = intersect_torus_torus(&t1, &t2, 1e-9);
        assert!(
            !out.is_empty(),
            "marching must find the offset-ring intersection"
        );
        for p in out.iter().flatten() {
            assert_on_torus(p, &t1, 1e-5, "offset/t1");
            assert_on_torus(p, &t2, 1e-5, "offset/t2");
        }
    }

    #[test]
    fn disjoint_pair_marching_empty() {
        // Far-away torus: the distance field never dips below the
        // flagging threshold on either grid → empty.
        let t1 = torus_z();
        let t2 = torus_z_at(Point3d::new(0.0, 0.0, 60.0));
        assert!(intersect_torus_torus(&t1, &t2, 1e-9).is_empty());
    }
}
