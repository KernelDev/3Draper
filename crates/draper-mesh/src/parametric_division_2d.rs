// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # ParameterDivision2D — Adaptive Recursive UV Subdivision
//!
//! Inspired by truck's `ParameterDivision2D` trait
//! (see `truck-geotrait/src/algo/surface.rs:219-281`).
//!
//! ## Algorithm
//!
//! Given a parametric surface `S(u, v)` and a chord-error tolerance `tol`,
//! produce two sorted arrays `{u_0 < u_1 < ... < u_m}` and
//! `{v_0 < v_1 < ... < v_n}` such that the bilinear interpolation of
//! `S` evaluated at the corners of every sub-rectangle `[u_i, u_{i+1}] ×
//! [v_j, v_{j+1}]` is within `tol` (in 3D distance) of the true surface
//! at one interior sample point.
//!
//! Recursion:
//! 1. For the current rectangle `[u0, u1] × [v0, v1]`, evaluate the
//!    surface at the 4 corners and at a deterministic interior sample
//!    (golden-ratio jitter of the midpoint — bit-stable).
//! 2. Compute the bilinear estimate of the interior sample from the 4
//!    corners and compare with the true surface point.
//! 3. If `||true - bilinear|| <= tol`, accept the rectangle.
//! 4. Otherwise, decide which axis to split based on where the error
//!    concentrates: split u if the u-edge midpoint error exceeds the
//!    v-edge midpoint error, otherwise split v. Split both if both
//!    exceed `tol * 0.5`.
//! 5. Recurse until either tolerance is met or `max_depth` is reached.
//!
//! ## Why this is better than the previous approach
//!
//! Previously, `parametric_domain::generate_nurbs_interior_points` used
//! a uniform knot-span grid with `n_sub` subdivisions per span. This
//! either over-tessellated low-curvature regions (wasting triangles)
//! or under-tessellated high-curvature regions (chord error > tol).
//!
//! This implementation:
//! - Works uniformly for ALL surface types (Plane, Cylinder, NURBS, …)
//!   via a single `SurfacePointSampler` trait — no per-type formulas.
//! - Adapts to LOCAL curvature: dense subdivision where the surface
//!   bends, sparse subdivision where it's nearly flat.
//! - Produces a strictly-nested pair of sorted knot vectors, suitable
//!   for direct use as Steiner points in the existing CDT pipeline.
//!
//! ## Determinism
//!
//! The interior sample uses a golden-ratio offset from the midpoint,
//! which is bit-stable across platforms and FP evaluation orders
//! (no `rand()` call — truck uses `HashGen` for the same reason).

use draper_geometry::{Point2d, Point3d, Surface};

/// Golden ratio minus 1, i.e. `(sqrt(5) - 1) / 2 ≈ 0.618`.
/// Used as a deterministic, bit-stable jitter offset.
const FRAC_PHI_MINUS_1: f64 = 0.6180339887498949;

/// Maximum recursion depth for `parameter_division_2d`.
///
/// Each level halves the UV rectangle in at least one axis, so depth D
/// produces at most `2^D` rectangles along an axis. Depth 14 → up to
/// 16 384 samples along an axis, which is well beyond any reasonable
/// tessellation density.
const MAX_DEPTH: u32 = 14;

/// Minimum UV-span below which we stop subdividing regardless of error.
///
/// This prevents infinite recursion when the surface evaluator returns
/// identical points for slightly-different UVs (degenerate parameter
/// lines, e.g., at a cone apex or sphere pole).
const MIN_SPAN: f64 = 1e-9;

/// Golden-ratio jitter for the interior sample.
///
/// We sample the surface at `(lerp(u0, u1, 0.5 ± δ), lerp(v0, v1, 0.5 ± δ))`
/// where `δ = FRAC_PHI_MINUS_1 * 0.5 ≈ 0.309`. Using the golden ratio
/// ensures that, no matter how the recursion splits, the sample points
/// never coincide across levels — this matches truck's `HashGen` goal
/// of deterministic-but-uncorrelated sampling.
const SAMPLE_OFFSET: f64 = FRAC_PHI_MINUS_1 * 0.5; // ≈ 0.309

/// Public entry point: compute adaptive UV subdivision for any surface.
///
/// Returns `(u_knots, v_knots)` — sorted `Vec<f64>` including the
/// endpoints `u_min, u_max` and `v_min, v_max`. Always contains at
/// least 2 elements per axis.
///
/// # Arguments
/// * `surface` — the parametric surface to subdivide.
/// * `u_range` — `(u_min, u_max)` valid parameter range.
/// * `v_range` — `(v_min, v_max)` valid parameter range.
/// * `tol` — chord-error tolerance in 3D distance units. Must be > 0.
/// * `max_dim` — cap on the number of subdivisions per axis
///   (defensive against runaway recursion on pathological surfaces).
pub fn parameter_division_2d(
    surface: &Surface,
    u_range: (f64, f64),
    v_range: (f64, f64),
    tol: f64,
    max_dim: usize,
) -> (Vec<f64>, Vec<f64>) {
    let (u0, u1) = if u_range.0 < u_range.1 {
        u_range
    } else {
        (u_range.1, u_range.0)
    };
    let (v0, v1) = if v_range.0 < v_range.1 {
        v_range
    } else {
        (v_range.1, v_range.0)
    };

    // tol must be strictly positive; fall back to a small default.
    let tol = if tol.is_finite() && tol > 0.0 {
        tol
    } else {
        1e-3
    };

    let mut u_knots: Vec<f64> = Vec::new();
    let mut v_knots: Vec<f64> = Vec::new();

    subdivide(
        surface,
        u0,
        u1,
        v0,
        v1,
        tol,
        0,
        &mut u_knots,
        &mut v_knots,
        max_dim,
    );

    // Deduplicate + sort. Recursion collects in left-to-right order, but
    // interior splits may add the same value from sibling branches.
    u_knots.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v_knots.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    u_knots.dedup_by(|a, b| (*a - *b).abs() < MIN_SPAN);
    v_knots.dedup_by(|a, b| (*a - *b).abs() < MIN_SPAN);

    // Defensive: ensure endpoints are present.
    if u_knots.first().copied() != Some(u0) {
        u_knots.insert(0, u0);
    }
    if u_knots.last().copied() != Some(u1) {
        u_knots.push(u1);
    }
    if v_knots.first().copied() != Some(v0) {
        v_knots.insert(0, v0);
    }
    if v_knots.last().copied() != Some(v1) {
        v_knots.push(v1);
    }

    (u_knots, v_knots)
}

/// Recursive subdivision step.
///
/// Adds `u0`, `u1`, `v0`, `v1` to the corresponding knot vectors (the
/// caller deduplicates later). When splitting, only the *new* midpoint
/// is added by the splitting branch — the endpoints come from sibling
/// branches or the top-level call.
#[allow(clippy::too_many_arguments)]
fn subdivide(
    surface: &Surface,
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
    tol: f64,
    depth: u32,
    u_knots: &mut Vec<f64>,
    v_knots: &mut Vec<f64>,
    max_dim: usize,
) {
    // Hard stop on max depth or min span.
    let u_span = u1 - u0;
    let v_span = v1 - v0;

    if depth >= MAX_DEPTH
        || u_span < MIN_SPAN
        || v_span < MIN_SPAN
        || u_knots.len() >= max_dim
        || v_knots.len() >= max_dim
    {
        u_knots.push(u0);
        u_knots.push(u1);
        v_knots.push(v0);
        v_knots.push(v1);
        return;
    }

    // Evaluate surface at 4 corners.
    let p00 = surface.point_at(u0, v0);
    let p10 = surface.point_at(u1, v0);
    let p01 = surface.point_at(u0, v1);
    let p11 = surface.point_at(u1, v1);

    // Golden-ratio-jittered interior sample.
    let su = 0.5 + SAMPLE_OFFSET; // ≈ 0.809
    let sv = 0.5 - SAMPLE_OFFSET; // ≈ 0.191
    let u_mid = u0 + su * u_span;
    let v_mid = v0 + sv * v_span;
    let p_sample = surface.point_at(u_mid, v_mid);

    // Bilinear interpolation of the 4 corners at (su, sv).
    let bilerp = bilinear_estimate(&p00, &p10, &p01, &p11, su, sv);

    let err = p_sample.distance_to(&bilerp);

    if err <= tol {
        // Tolerance met — emit the endpoints and stop.
        u_knots.push(u0);
        u_knots.push(u1);
        v_knots.push(v0);
        v_knots.push(v1);
        return;
    }

    // Error too large. Decide split axis by computing edge-midpoint errors:
    //   - u-edge midpoints: ((u0+u1)/2, v0) and ((u0+u1)/2, v1)
    //   - v-edge midpoints: (u0, (v0+v1)/2) and (u1, (v0+v1)/2)
    // If the u-edge midpoints deviate strongly from the line (p00,p10) /
    // (p01,p11), the curvature is mostly in the u-direction → split u.
    let u_mid_axis = 0.5 * (u0 + u1);
    let v_mid_axis = 0.5 * (v0 + v1);

    let p_u_lo = surface.point_at(u_mid_axis, v0);
    let p_u_hi = surface.point_at(u_mid_axis, v1);
    let p_v_lo = surface.point_at(u0, v_mid_axis);
    let p_v_hi = surface.point_at(u1, v_mid_axis);

    let p_u_lo_lin = p00.lerp(&p10, 0.5);
    let p_u_hi_lin = p01.lerp(&p11, 0.5);
    let p_v_lo_lin = p00.lerp(&p01, 0.5);
    let p_v_hi_lin = p10.lerp(&p11, 0.5);

    let err_u = p_u_lo.distance_to(&p_u_lo_lin).max(p_u_hi.distance_to(&p_u_hi_lin));
    let err_v = p_v_lo.distance_to(&p_v_lo_lin).max(p_v_hi.distance_to(&p_v_hi_lin));

    let half_tol = 0.5 * tol;
    let split_u = err_u > half_tol;
    let split_v = err_v > half_tol;

    // If neither edge direction shows dominant error but the interior
    // sample still failed, split both — this catches saddle points
    // and other mixed-curvature cases.
    let (split_u, split_v) = if !split_u && !split_v {
        (true, true)
    } else {
        (split_u, split_v)
    };

    if split_u && split_v {
        // 4-way split.
        subdivide(surface, u0, u_mid_axis, v0, v_mid_axis, tol, depth + 1, u_knots, v_knots, max_dim);
        subdivide(surface, u_mid_axis, u1, v0, v_mid_axis, tol, depth + 1, u_knots, v_knots, max_dim);
        subdivide(surface, u0, u_mid_axis, v_mid_axis, v1, tol, depth + 1, u_knots, v_knots, max_dim);
        subdivide(surface, u_mid_axis, u1, v_mid_axis, v1, tol, depth + 1, u_knots, v_knots, max_dim);
    } else if split_u {
        // Split along u only.
        subdivide(surface, u0, u_mid_axis, v0, v1, tol, depth + 1, u_knots, v_knots, max_dim);
        subdivide(surface, u_mid_axis, u1, v0, v1, tol, depth + 1, u_knots, v_knots, max_dim);
    } else {
        // Split along v only.
        subdivide(surface, u0, u1, v0, v_mid_axis, tol, depth + 1, u_knots, v_knots, max_dim);
        subdivide(surface, u0, u1, v_mid_axis, v1, tol, depth + 1, u_knots, v_knots, max_dim);
    }
}

/// Bilinear interpolation of 4 corner points at parameter (su, sv) where
/// su, sv ∈ [0, 1].
#[inline]
fn bilinear_estimate(
    p00: &Point3d,
    p10: &Point3d,
    p01: &Point3d,
    p11: &Point3d,
    su: f64,
    sv: f64,
) -> Point3d {
    // Linear interp along u at v=v0 (sv=0) and v=v1 (sv=1), then along v.
    let p_u0 = p00.lerp(&p10, su); // (1-su)*p00 + su*p10
    let p_u1 = p01.lerp(&p11, su);
    p_u0.lerp(&p_u1, sv)
}

/// Filter the subdivided knot vectors to keep only STRICTLY INTERIOR
/// samples — i.e. remove the endpoints `u_min, u_max, v_min, v_max`
/// and any knots within `boundary_tol` of them.
///
/// This is the analog of `generate_nurbs_interior_points`'s strict-
/// interior filter: Steiner points on the boundary produce phantom
/// vertices on shared edges that aren't reproduced by adjacent faces,
/// which breaks watertightness.
pub fn interior_steiner_points(
    u_knots: &[f64],
    v_knots: &[f64],
    u_range: (f64, f64),
    v_range: (f64, f64),
    boundary_tol: f64,
) -> Vec<Point2d> {
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;

    let u_interior: Vec<f64> = u_knots
        .iter()
        .copied()
        .filter(|&u| u > u_min + boundary_tol && u < u_max - boundary_tol)
        .collect();
    let v_interior: Vec<f64> = v_knots
        .iter()
        .copied()
        .filter(|&v| v > v_min + boundary_tol && v < v_max - boundary_tol)
        .collect();

    let mut pts = Vec::with_capacity(u_interior.len() * v_interior.len());
    for &u in &u_interior {
        for &v in &v_interior {
            pts.push(Point2d::new(u, v));
        }
    }
    pts
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::{CylinderSurface, Plane, Point3d, SphereSurface, Surface};
    use std::f64::consts::PI;

    #[test]
    fn plane_returns_endpoints_only() {
        // A plane is exactly bilinear — bilinear interpolation of the 4
        // corners reproduces the surface everywhere. So subdivision should
        // terminate immediately and return only the endpoints.
        let plane = Surface::Plane(Plane::xy());
        let (us, vs) = parameter_division_2d(&plane, (0.0, 1.0), (0.0, 1.0), 0.01, 1024);
        assert_eq!(us, vec![0.0, 1.0]);
        assert_eq!(vs, vec![0.0, 1.0]);
    }

    #[test]
    fn cylinder_subdivides_in_u() {
        // A cylinder has zero curvature along its axis (v) and uniform
        // curvature 1/r along u (the angular direction). Subdivision
        // should be dense in u and minimal in v.
        let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));
        let (us, vs) = parameter_division_2d(&cyl, (0.0, 2.0 * PI), (0.0, 1.0), 0.01, 1024);
        // u should be subdivided into at least 4 segments (cylinder chord error
        // for radius 1 and 2π span at tol=0.01 → ~50 segments, but adaptive
        // recursion may stop earlier near the endpoints where it doesn't need
        // to be that fine).
        assert!(us.len() >= 4, "u_knots.len() = {}, expected >= 4", us.len());
        // v should NOT be subdivided (zero curvature along v).
        assert_eq!(vs.len(), 2, "v_knots should be just [v_min, v_max]");
        // Endpoints preserved.
        assert!((us[0] - 0.0).abs() < 1e-12);
        assert!((us[us.len() - 1] - 2.0 * PI).abs() < 1e-12);
    }

    #[test]
    fn sphere_subdivides_in_both() {
        // A sphere has curvature 1/r in both directions.
        let sph = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 1.0));
        let (us, vs) = parameter_division_2d(&sph, (0.0, 2.0 * PI), (0.0, PI), 0.01, 1024);
        assert!(us.len() >= 4, "u_knots.len() = {}", us.len());
        assert!(vs.len() >= 3, "v_knots.len() = {}", vs.len());
    }

    #[test]
    fn interior_filter_removes_endpoints() {
        let u_knots = vec![0.0, 0.25, 0.5, 0.75, 1.0];
        let v_knots = vec![0.0, 0.5, 1.0];
        let pts = interior_steiner_points(&u_knots, &v_knots, (0.0, 1.0), (0.0, 1.0), 1e-6);
        // Expected: 3 interior u × 1 interior v = 3 points
        assert_eq!(pts.len(), 3);
        // None of the points should be on the boundary
        for p in &pts {
            assert!(p.u > 1e-6 && p.u < 1.0 - 1e-6);
            assert!(p.v > 1e-6 && p.v < 1.0 - 1e-6);
        }
    }

    #[test]
    fn handles_degenerate_input() {
        // Zero-span range should not panic.
        let plane = Surface::Plane(Plane::xy());
        let (us, vs) = parameter_division_2d(&plane, (0.5, 0.5), (0.0, 1.0), 0.01, 1024);
        assert!(!us.is_empty());
        assert!(!vs.is_empty());
    }

    #[test]
    fn max_dim_caps_subdivision() {
        // Even with very tight tolerance, max_dim should prevent runaway.
        let plane = Surface::Plane(Plane::xy());
        let (us, _vs) = parameter_division_2d(&plane, (0.0, 1.0), (0.0, 1.0), 1e-12, 4);
        assert!(us.len() <= 4 + 2); // small slack for endpoint dedup
    }
}
