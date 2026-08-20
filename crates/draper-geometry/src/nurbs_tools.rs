// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! NURBS toolkit — knot insertion, knot removal, degree elevation, Bézier decomposition.
//!
//! Algorithms adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT)
//! and "The NURBS Book" (Piegl & Tiller, 1997).
//!
//! All operations work on the homogeneous representation (4D: x*w, y*w, z*w, w)
//! so they are valid for both rational and non-rational B-spline curves.

use crate::curve::NurbsCurve;
use crate::Point3d;

/// A point in 4D homogeneous coordinates: (x*w, y*w, z*w, w).
/// To convert back to 3D: divide first 3 components by the 4th.
#[derive(Clone, Copy, Debug)]
struct HomogeneousPoint {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

impl HomogeneousPoint {
    fn from_point_weight(p: &Point3d, w: f64) -> Self {
        Self { x: p.x * w, y: p.y * w, z: p.z * w, w }
    }

    fn to_point(&self) -> Point3d {
        if self.w.abs() < 1e-15 {
            Point3d::new(0.0, 0.0, 0.0)
        } else {
            Point3d::new(self.x / self.w, self.y / self.w, self.z / self.w)
        }
    }
}

/// Convert a NurbsCurve to its homogeneous representation.
fn to_homogeneous(curve: &NurbsCurve) -> Vec<HomogeneousPoint> {
    curve.control_points.iter()
        .zip(curve.weights.iter())
        .map(|(p, &w)| HomogeneousPoint::from_point_weight(p, w))
        .collect()
}

/// Convert a homogeneous representation back to a NurbsCurve.
fn from_homogeneous(hpts: &[HomogeneousPoint], degree: usize, knots: Vec<f64>) -> NurbsCurve {
    let control_points: Vec<Point3d> = hpts.iter().map(|h| h.to_point()).collect();
    let weights: Vec<f64> = hpts.iter().map(|h| h.w).collect();
    NurbsCurve { degree, control_points, weights, knots }
}

/// Find the knot span index `k` such that `knots[k] <= t < knots[k+1]`.
///
/// For `t == knots.last()`, returns `n - 1` where `n = control_points.len()`.
#[cfg(test)]
fn find_knot_span(knots: &[f64], degree: usize, t: f64, n: usize) -> usize {
    if t >= knots[n] {
        return n - 1;
    }
    let mut lo = degree;
    let mut hi = n + 1;
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// Insert a knot `u` into the curve `times` times (Boehm's algorithm).
///
/// Returns a new NurbsCurve with the knot inserted. The curve geometry is
/// unchanged — only the representation is refined.
///
/// Reference: "The NURBS Book" (Piegl & Tiller, 1997), Algorithm A5.1.
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn insert_knot(curve: &NurbsCurve, u: f64, times: usize) -> NurbsCurve {
    if times == 0 {
        return curve.clone();
    }

    let p = curve.degree;
    let n = curve.control_points.len();
    if n == 0 || p == 0 {
        return curve.clone();
    }

    // Clamp u to valid knot range
    let t_min = if curve.knots.len() > p { curve.knots[p] } else { 0.0 };
    let t_max = if curve.knots.len() > p { curve.knots[curve.knots.len() - p - 1] } else { 1.0 };
    let u_c = u.clamp(t_min, t_max);

    // Insert one knot at a time, `times` iterations total.
    // This is simpler and matches truck's implementation.
    let mut result = curve.clone();
    for _ in 0..times {
        result = insert_knot_once(&result, u_c);
    }
    result
}

/// Insert a single knot `u` into the curve (Boehm's algorithm, single insertion).
///
/// Adapts truck's `add_knot` implementation (ricosjp/truck, Apache-2.0 OR MIT).
fn insert_knot_once(curve: &NurbsCurve, u: f64) -> NurbsCurve {
    let p = curve.degree;
    let n = curve.control_points.len();
    if n == 0 || p == 0 {
        return curve.clone();
    }

    // Clamp u to valid knot range
    let t_min = if curve.knots.len() > p { curve.knots[p] } else { 0.0 };
    let t_max = if curve.knots.len() > p { curve.knots[curve.knots.len() - p - 1] } else { 1.0 };
    let u_c = u.clamp(t_min, t_max);

    // Find the floor index: largest i such that knots[i] <= u_c
    // (Returns None if u_c < knots[0])
    let floor_idx = {
        let mut fi: Option<usize> = None;
        for (i, &kn) in curve.knots.iter().enumerate() {
            if kn <= u_c + 1e-12 {
                fi = Some(i);
            } else {
                break;
            }
        }
        fi
    };

    // Work in homogeneous coordinates
    let mut hpts = to_homogeneous(curve);
    let mut knots = curve.knots.clone();

    // Insert the new knot and get its position (idx_truck) in the NEW knot vector
    let idx_truck = match floor_idx {
        Some(fi) => {
            knots.insert(fi + 1, u_c);
            fi + 1
        }
        None => {
            knots.insert(0, u_c);
            0
        }
    };

    // Don't insert if u_c already has multiplicity p+1 (max allowed)
    let s = knots.iter()
        .filter(|&&kn| (kn - u_c).abs() < 1e-12)
        .count();
    if s > p + 1 {
        // Should not happen, but just in case
        return curve.clone();
    }

    // Insert a placeholder control point at position idx_truck - 1
    // (copy of control_points[idx_truck - 1])
    let placeholder_pos = if idx_truck > 0 { idx_truck - 1 } else { 0 };
    let placeholder = hpts[placeholder_pos.min(hpts.len() - 1)];

    if idx_truck > n {
        // Append at the end
        hpts.push(HomogeneousPoint { x: 0.0, y: 0.0, z: 0.0, w: 0.0 });
    } else {
        // Insert placeholder at position idx_truck - 1
        hpts.insert(placeholder_pos, placeholder);
    }

    // Update the affected control points in REVERSE order.
    // Loop: for i0 from idx_truck-1 down to start (inclusive).
    let start = idx_truck.saturating_sub(p);
    let end = idx_truck; // exclusive upper bound for the forward loop

    // We iterate i0 = end-1, end-2, ..., start (reverse)
    for i0 in (start..end).rev() {
        if i0 + p + 1 >= knots.len() {
            continue;
        }
        let delta = knots[i0 + p + 1] - knots[i0];
        let a = if delta.abs() < 1e-15 {
            0.0
        } else {
            (u_c - knots[i0]) / delta
        };
        // Update: CP[i0] = a * CP[i0] + (1-a) * CP[i0-1]
        // (Matches truck's: CP[i0] -= (1-a) * (CP[i0] - CP[i0-1]))
        let curr = hpts[i0];
        let prev = if i0 > 0 { hpts[i0 - 1] } else { curr };
        hpts[i0] = HomogeneousPoint {
            x: a * curr.x + (1.0 - a) * prev.x,
            y: a * curr.y + (1.0 - a) * prev.y,
            z: a * curr.z + (1.0 - a) * prev.z,
            w: a * curr.w + (1.0 - a) * prev.w,
        };
    }

    from_homogeneous(&hpts, p, knots)
}

/// Try to remove the knot at index `idx` (Tiller-Theisen algorithm).
///
/// Returns `Some(NurbsCurve)` if the knot was successfully removed (the curve
/// geometry is preserved within tolerance), or `None` if removal is not possible
/// without changing the curve beyond the tolerance.
///
/// Reference: "The NURBS Book" (Piegl & Tiller, 1997), Algorithm A5.4.
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn try_remove_knot(curve: &NurbsCurve, idx: usize, tolerance: f64) -> Option<NurbsCurve> {
    let p = curve.degree;
    let n = curve.control_points.len();
    let knots = &curve.knots;

    // Cannot remove knots at the boundaries or out of range
    if idx < p + 1 || idx >= n {
        return None;
    }

    // Work in homogeneous coordinates
    let mut hpts = to_homogeneous(curve);

    // Build the new control points using Tiller's algorithm.
    // new_points[0] = P[idx - p - 1] (unchanged)
    // For i in [idx-p, idx): compute new_points via the formula
    //   a = (knots[idx] - knots[i]) / (knots[i+p+1] - knots[i])
    //   new_points[next] = new_points[last] + (P[i] - new_points[last]) / a
    let mut new_points: Vec<HomogeneousPoint> = Vec::with_capacity(p + 1);
    new_points.push(hpts[idx - p - 1]);

    for i in (idx - p)..idx {
        if i + p + 1 >= knots.len() {
            break;
        }
        let delta = knots[i + p + 1] - knots[i];
        if delta.abs() < 1e-15 {
            break;
        }
        let a = (knots[idx] - knots[i]) / delta;
        if a.abs() < 1e-15 {
            break;
        }
        // Safe: new_points is initialized with one element before the loop
        // and only grows. Using if-let to satisfy panic-free directive.
        let last = match new_points.last() {
            Some(&p) => p,
            None => break, // Should never happen, but panic-free
        };
        let curr = hpts[i];
        let new_pt = HomogeneousPoint {
            x: last.x + (curr.x - last.x) / a,
            y: last.y + (curr.y - last.y) / a,
            z: last.z + (curr.z - last.z) / a,
            w: last.w + (curr.w - last.w) / a,
        };
        new_points.push(new_pt);
    }

    // Check if the last computed point matches the existing control point at idx
    if new_points.is_empty() {
        return None;
    }
    // Safe: new_points is non-empty (checked above)
    let computed = match new_points.last() {
        Some(&p) => p,
        None => return None, // Should never happen after is_empty check
    };
    let target = hpts[idx];
    let dist_sq = (target.x - computed.x).powi(2)
        + (target.y - computed.y).powi(2)
        + (target.z - computed.z).powi(2)
        + (target.w - computed.w).powi(2);
    if dist_sq > tolerance * tolerance {
        return None;
    }

    // Update the control points: replace [idx-p+1 .. idx] with new_points[1..]
    for (i, pt) in new_points.into_iter().skip(1).enumerate() {
        let pos = idx - p + i;
        if pos < hpts.len() {
            hpts[pos] = pt;
        }
    }

    // Remove the knot at idx
    hpts.remove(idx);
    let mut new_knots = knots.clone();
    new_knots.remove(idx);

    Some(from_homogeneous(&hpts, p, new_knots))
}

/// Remove the knot at index `idx` (unconditional — falls back to no-op if
/// removal would change the geometry).
pub fn remove_knot(curve: &NurbsCurve, idx: usize, tolerance: f64) -> NurbsCurve {
    try_remove_knot(curve, idx, tolerance).unwrap_or_else(|| curve.clone())
}

/// Clamp the curve: ensure the first and last knots have multiplicity = degree.
///
/// For a clamped B-spline, the curve passes through the first and last control
/// points. If the curve is currently unclamped (periodic), this operation
/// inserts knots at the endpoints until they have multiplicity `degree`.
///
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn clamp(curve: &NurbsCurve) -> NurbsCurve {
    let p = curve.degree;
    if curve.knots.len() < 2 * (p + 1) {
        return curve.clone();
    }

    let t_min = curve.knots[p];
    let t_max = curve.knots[curve.knots.len() - p - 1];

    // Compute current multiplicity at t_min
    let s_min = curve.knots.iter()
        .filter(|&&kn| (kn - t_min).abs() < 1e-12)
        .count();
    let need_min = p.saturating_sub(s_min);

    // Compute current multiplicity at t_max
    let s_max = curve.knots.iter()
        .filter(|&&kn| (kn - t_max).abs() < 1e-12)
        .count();
    let need_max = p.saturating_sub(s_max);

    let mut result = curve.clone();
    for _ in 0..need_min {
        result = insert_knot(&result, t_min, 1);
    }
    for _ in 0..need_max {
        result = insert_knot(&result, t_max, 1);
    }

    result
}

/// Iteratively remove redundant knots from the interior of the knot vector.
///
/// A knot is "redundant" if removing it does not change the curve geometry
/// beyond `tolerance`. This is useful for cleaning up curves that have
/// accumulated many knots from previous operations.
///
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn optimize(curve: &NurbsCurve, tolerance: f64) -> NurbsCurve {
    let p = curve.degree;
    let n = curve.control_points.len();

    let mut result = curve.clone();
    let mut changed = true;

    while changed {
        changed = false;
        // Try to remove each interior knot
        let knots_len = result.knots.len();
        for idx in (p + 1)..n.min(knots_len) {
            if let Some(optimized) = try_remove_knot(&result, idx, tolerance) {
                result = optimized;
                changed = true;
                break; // Restart the loop after each successful removal
            }
        }
    }

    result
}

/// Split a curve at parameter `t`, returning two curves: [t_min, t] and [t, t_max].
///
/// Uses knot insertion to make `t` have multiplicity `degree + 1`, then splits.
/// The two segments are continuous at `t` (they share the same point value).
///
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn cut(curve: &NurbsCurve, t: f64) -> (NurbsCurve, NurbsCurve) {
    let p = curve.degree;
    let n = curve.control_points.len();

    if n == 0 {
        return (curve.clone(), curve.clone());
    }

    let t_min = if curve.knots.len() > p { curve.knots[p] } else { 0.0 };
    let t_max = if curve.knots.len() > p { curve.knots[curve.knots.len() - p - 1] } else { 1.0 };
    let t_c = t.clamp(t_min, t_max);

    // Compute existing multiplicity at t
    let s = curve.knots.iter()
        .filter(|&&kn| (kn - t_c).abs() < 1e-12)
        .count();

    // Insert t until it has multiplicity p+1 (needed for a clean cut)
    let need = (p + 1).saturating_sub(s);
    let refined = if need > 0 {
        insert_knot(curve, t_c, need)
    } else {
        curve.clone()
    };

    // Find k = largest index such that refined.knots[k] <= t_c
    // (This is the LAST copy of t_c in the knot vector)
    let mut k = 0;
    for (i, &kn) in refined.knots.iter().enumerate() {
        if kn <= t_c + 1e-12 {
            k = i;
        } else {
            break;
        }
    }

    let n_refined = refined.control_points.len();
    let split_idx = k.saturating_sub(p);

    // Left curve: knots [0..=k], control points [0..split_idx)
    let mut left_knots = Vec::with_capacity(k + 1);
    for i in 0..=k {
        left_knots.push(refined.knots[i]);
    }
    // Pad left knots with t_c at the end to make multiplicity p+1
    // (already the case since k is the last index of t_c)
    let mut left_cps = Vec::with_capacity(split_idx);
    let mut left_weights = Vec::with_capacity(split_idx);
    for i in 0..split_idx {
        left_cps.push(refined.control_points[i]);
        left_weights.push(refined.weights[i]);
    }

    let left = NurbsCurve {
        degree: p,
        control_points: left_cps,
        weights: left_weights,
        knots: left_knots,
    };

    // Right curve: knots [split_idx..end], control points [split_idx..end]
    let mut right_knots = Vec::with_capacity(refined.knots.len() - split_idx);
    for i in split_idx..refined.knots.len() {
        right_knots.push(refined.knots[i]);
    }
    let mut right_cps = Vec::with_capacity(n_refined - split_idx);
    let mut right_weights = Vec::with_capacity(n_refined - split_idx);
    for i in split_idx..n_refined {
        right_cps.push(refined.control_points[i]);
        right_weights.push(refined.weights[i]);
    }

    let right = NurbsCurve {
        degree: p,
        control_points: right_cps,
        weights: right_weights,
        knots: right_knots,
    };

    (left, right)
}

/// Decompose a B-spline curve into a sequence of Bézier segments.
///
/// Each segment has degree `curve.degree` and `curve.degree + 1` control points,
/// with knot vector [t, t, ..., t (p+1 times), t_next, t_next, ..., t_next (p+1 times)].
///
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn bezier_decomposition(curve: &NurbsCurve) -> Vec<NurbsCurve> {
    let p = curve.degree;
    if curve.control_points.is_empty() || curve.knots.len() < 2 * (p + 1) {
        return vec![curve.clone()];
    }

    // First clamp the curve so endpoints have multiplicity p
    let clamped = clamp(curve);

    // Extract unique interior knots (multiplicity 1)
    let mut interior_knots: Vec<f64> = Vec::new();
    let t_min = clamped.knots[p];
    let t_max = clamped.knots[clamped.knots.len() - p - 1];

    let mut prev = f64::NAN;
    for &kn in &clamped.knots {
        if kn > t_min && kn < t_max && (kn - prev).abs() > 1e-12 {
            interior_knots.push(kn);
            prev = kn;
        } else if kn <= t_min || kn >= t_max {
            prev = kn;
        }
    }

    if interior_knots.is_empty() {
        // Already a single Bézier segment
        return vec![clamped];
    }

    // Insert each interior knot until it has multiplicity p, then cut
    let mut current = clamped;
    let mut segments = Vec::with_capacity(interior_knots.len() + 1);

    for &knot in &interior_knots {
        // Ensure knot has multiplicity p in current
        let s = current.knots.iter()
            .filter(|&&kn| (kn - knot).abs() < 1e-12)
            .count();
        let need = p.saturating_sub(s);
        for _ in 0..need {
            current = insert_knot(&current, knot, 1);
        }

        // Cut at knot
        let (left, right) = cut(&current, knot);
        segments.push(left);
        current = right;
    }
    segments.push(current);

    segments
}

/// Elevate the degree of the curve by 1 (Prautzsch algorithm).
///
/// Decomposes the curve into Bézier segments, elevates each segment's degree,
/// then recombines using the standard Bézier-to-B-spline knot removal.
///
/// Reference: "The NURBS Book" (Piegl & Tiller, 1997), Algorithm A5.9.
/// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn elevate_degree(curve: &NurbsCurve) -> NurbsCurve {
    let p = curve.degree;
    if curve.control_points.is_empty() {
        return curve.clone();
    }

    // Step 1: Decompose into Bézier segments
    let beziers = bezier_decomposition(curve);

    // Step 2: Elevate each Bézier segment by 1
    let elevated_beziers: Vec<NurbsCurve> = beziers.into_iter()
        .map(|b| elevate_degree_bezier(&b))
        .collect();

    if elevated_beziers.is_empty() {
        return curve.clone();
    }
    if elevated_beziers.len() == 1 {
        // Safe: len() == 1 guarantees next() returns Some
        return elevated_beziers.into_iter().next()
            .unwrap_or_else(|| curve.clone());
    }

    // Step 3: Recombine using knot removal at the junctions
    // The junction knots have multiplicity p+1 (from the cut), but after degree
    // elevation they should have multiplicity p+2 (since each Bézier now has
    // degree p+1 with knot multiplicity p+2 at endpoints).
    // We need to remove one copy of each junction knot to get back to a proper
    // B-spline representation.

    let new_degree = p + 1;
    let mut combined = elevated_beziers[0].clone();

    for next_bezier in elevated_beziers.iter().skip(1) {
        combined = concat_bezier_segments(&combined, next_bezier, new_degree);
    }

    combined
}

/// Elevate the degree of a single Bézier segment by 1.
///
/// A Bézier segment of degree p has knot vector [t, t, ..., t (p+1 times), T, T, ..., T (p+1 times)].
/// After elevation, it has degree p+1 and p+2 control points.
///
/// New control points: Q_0 = P_0, Q_{p+1} = P_p,
/// Q_i = (i/(p+1)) * P_{i-1} + ((p+1-i)/(p+1)) * P_i for i in 1..=p.
fn elevate_degree_bezier(bezier: &NurbsCurve) -> NurbsCurve {
    let p = bezier.degree;
    let n = bezier.control_points.len();
    if n != p + 1 {
        // Not a proper Bézier segment — return as-is
        return bezier.clone();
    }

    let new_p = p + 1;
    let mut new_cps = Vec::with_capacity(new_p + 1);
    let mut new_weights = Vec::with_capacity(new_p + 1);

    new_cps.push(bezier.control_points[0]);
    new_weights.push(bezier.weights[0]);

    for i in 1..=p {
        let a = i as f64 / (new_p as f64);
        let prev_p = bezier.control_points[i - 1];
        let curr_p = bezier.control_points[i];
        let prev_w = bezier.weights[i - 1];
        let curr_w = bezier.weights[i];

        // For rational curves, operate in homogeneous coords
        let prev_h = HomogeneousPoint::from_point_weight(&prev_p, prev_w);
        let curr_h = HomogeneousPoint::from_point_weight(&curr_p, curr_w);
        let new_h = HomogeneousPoint {
            x: a * prev_h.x + (1.0 - a) * curr_h.x,
            y: a * prev_h.y + (1.0 - a) * curr_h.y,
            z: a * prev_h.z + (1.0 - a) * curr_h.z,
            w: a * prev_h.w + (1.0 - a) * curr_h.w,
        };
        new_cps.push(new_h.to_point());
        new_weights.push(new_h.w);
    }

    new_cps.push(bezier.control_points[p]);
    new_weights.push(bezier.weights[p]);

    // Build new knot vector: same domain, but with multiplicity p+2 at both ends
    let t_min = bezier.knots.first().copied().unwrap_or(0.0);
    let t_max = bezier.knots.last().copied().unwrap_or(1.0);
    let mut new_knots = Vec::with_capacity(2 * (new_p + 1));
    for _ in 0..=new_p {
        new_knots.push(t_min);
    }
    for _ in 0..=new_p {
        new_knots.push(t_max);
    }

    NurbsCurve {
        degree: new_p,
        control_points: new_cps,
        weights: new_weights,
        knots: new_knots,
    }
}

/// Concatenate two Bézier segments of the same degree into a single B-spline curve.
///
/// The junction knot will have multiplicity `degree` (C^0 continuity for a
/// B-spline of the given degree).
fn concat_bezier_segments(left: &NurbsCurve, right: &NurbsCurve, degree: usize) -> NurbsCurve {
    let t_junction = right.knots.first().copied().unwrap_or(0.0);

    // Combine control points and weights.
    // The last control point of `left` should equal the first of `right` — skip the duplicate.
    let mut combined_cps = left.control_points.clone();
    combined_cps.extend(right.control_points.iter().skip(1));

    let mut combined_weights = left.weights.clone();
    combined_weights.extend(right.weights.iter().skip(1));

    // Build the combined knot vector.
    // - Left part: all knots except the last (degree+1) copies of t_junction
    // - Junction: t_junction with multiplicity `degree` (C^0 continuity)
    // - Right part: all knots except the first (degree+1) copies of t_junction
    let left_keep = left.knots.len().saturating_sub(degree + 1);
    let right_start = degree + 1;
    let junction_count = degree; // C^0 continuity at the junction

    let total_knots = left_keep + junction_count + (right.knots.len() - right_start);
    let mut combined_knots = Vec::with_capacity(total_knots);

    // Left part
    for i in 0..left_keep {
        combined_knots.push(left.knots[i]);
    }
    // Junction
    for _ in 0..junction_count {
        combined_knots.push(t_junction);
    }
    // Right part
    for i in right_start..right.knots.len() {
        combined_knots.push(right.knots[i]);
    }

    NurbsCurve {
        degree,
        control_points: combined_cps,
        weights: combined_weights,
        knots: combined_knots,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::NurbsCurve;
    use crate::Point3d;
    use quickcheck_macros::quickcheck;

    fn make_quadratic_bezier() -> NurbsCurve {
        NurbsCurve {
            degree: 2,
            control_points: vec![
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(1.0, 2.0, 0.0),
                Point3d::new(2.0, 0.0, 0.0),
            ],
            weights: vec![1.0, 1.0, 1.0],
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        }
    }

    fn make_cubic_bspline() -> NurbsCurve {
        NurbsCurve {
            degree: 3,
            control_points: vec![
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(1.0, 2.0, 0.0),
                Point3d::new(2.0, 1.0, 0.0),
                Point3d::new(3.0, 3.0, 0.0),
                Point3d::new(4.0, 0.0, 0.0),
            ],
            weights: vec![1.0; 5],
            knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0],
        }
    }

    fn eval_curve(curve: &NurbsCurve, t: f64) -> Point3d {
        // Use the curve's point_at via the Curve3d wrapper
        use crate::curve::Curve3d;
        Curve3d::Nurbs(curve.clone()).point_at(t)
    }

    #[test]
    fn test_insert_knot_preserves_geometry() {
        let curve = make_quadratic_bezier();
        let original_at_mid = eval_curve(&curve, 0.5);

        let refined = insert_knot(&curve, 0.5, 1);
        let refined_at_mid = eval_curve(&refined, 0.5);

        let dist = ((original_at_mid.x - refined_at_mid.x).powi(2)
            + (original_at_mid.y - refined_at_mid.y).powi(2)
            + (original_at_mid.z - refined_at_mid.z).powi(2)).sqrt();
        assert!(dist < 1e-10,
            "insert_knot changed curve geometry: dist = {}", dist);

        // Refined curve should have one more control point
        assert_eq!(refined.control_points.len(), curve.control_points.len() + 1);
        // And one more knot
        assert_eq!(refined.knots.len(), curve.knots.len() + 1);
    }

    #[test]
    fn test_insert_knot_multiple_times() {
        let curve = make_quadratic_bezier();
        let original_at_third = eval_curve(&curve, 0.3);

        // Insert 0.3 twice — for a quadratic, max multiplicity is 2
        let refined = insert_knot(&curve, 0.3, 2);
        let refined_at_third = eval_curve(&refined, 0.3);

        let dist = ((original_at_third.x - refined_at_third.x).powi(2)
            + (original_at_third.y - refined_at_third.y).powi(2)
            + (original_at_third.z - refined_at_third.z).powi(2)).sqrt();
        assert!(dist < 1e-10,
            "insert_knot multiple changed geometry: dist = {}", dist);
    }

    #[test]
    fn test_remove_knot_after_insert() {
        let curve = make_quadratic_bezier();
        let original = curve.clone();

        // Insert a knot, then remove it — should get back the original (within tolerance)
        let refined = insert_knot(&curve, 0.5, 1);

        // Find the inserted knot index (it's at position 3 in the new knot vector)
        // [0,0,0,0.5,1,1,1] — index 3
        let removed = try_remove_knot(&refined, 3, 1e-10);
        assert!(removed.is_some(), "try_remove_knot should succeed for inserted knot");

        let restored = removed.unwrap();
        let original_at = eval_curve(&original, 0.5);
        let restored_at = eval_curve(&restored, 0.5);
        let dist = ((original_at.x - restored_at.x).powi(2)
            + (original_at.y - restored_at.y).powi(2)
            + (original_at.z - restored_at.z).powi(2)).sqrt();
        assert!(dist < 1e-9, "remove_knot changed geometry: dist = {}", dist);
    }

    #[test]
    fn test_clamp_already_clamped() {
        let curve = make_quadratic_bezier();
        let clamped = clamp(&curve);
        // The curve is already clamped (knots have multiplicity 3 at both ends)
        // Clamp should be a no-op
        assert_eq!(clamped.knots.len(), curve.knots.len());
        assert_eq!(clamped.control_points.len(), curve.control_points.len());
    }

    #[test]
    fn test_cut_preserves_geometry() {
        let curve = make_cubic_bspline();
        // Curve has knots [0,0,0,0,1,2,2,2,2], so parameter range is [0, 2]
        let original_at_quarter = eval_curve(&curve, 0.25);
        let original_at_three_quarter = eval_curve(&curve, 1.5);

        let (left, right) = cut(&curve, 0.5);
        // Left segment has knots [0,0,0,0,0.5,0.5,0.5,0.5], parameter range [0, 0.5]
        // Right segment has knots [0.5,0.5,0.5,0.5,1,2,2,2,2], parameter range [0.5, 2]

        // Left at t=0.25 should match original at t=0.25 (same parameter)
        let left_at_quarter = eval_curve(&left, 0.25);
        let dist_left = ((original_at_quarter.x - left_at_quarter.x).powi(2)
            + (original_at_quarter.y - left_at_quarter.y).powi(2)
            + (original_at_quarter.z - left_at_quarter.z).powi(2)).sqrt();
        assert!(dist_left < 1e-9,
            "cut left segment doesn't match: dist = {}", dist_left);

        // Right at t=1.5 should match original at t=1.5 (same parameter)
        let right_at_three_quarter = eval_curve(&right, 1.5);
        let dist_right = ((original_at_three_quarter.x - right_at_three_quarter.x).powi(2)
            + (original_at_three_quarter.y - right_at_three_quarter.y).powi(2)
            + (original_at_three_quarter.z - right_at_three_quarter.z).powi(2)).sqrt();
        assert!(dist_right < 1e-9,
            "cut right segment doesn't match: dist = {}", dist_right);
    }

    #[test]
    fn test_bezier_decomposition_single_segment() {
        let curve = make_quadratic_bezier();
        let segments = bezier_decomposition(&curve);
        assert_eq!(segments.len(), 1, "Single Bézier should decompose to 1 segment");
        assert_eq!(segments[0].control_points.len(), 3);
    }

    #[test]
    fn test_bezier_decomposition_multiple_segments() {
        let curve = make_cubic_bspline();
        let segments = bezier_decomposition(&curve);
        // The curve has one interior knot at t=1, so we expect 2 segments
        assert_eq!(segments.len(), 2,
            "Cubic B-spline with 1 interior knot should decompose to 2 segments, got {}",
            segments.len());

        // Each segment should be a Bézier with 4 control points
        for (i, seg) in segments.iter().enumerate() {
            assert_eq!(seg.control_points.len(), 4,
                "Segment {} should have 4 control points, got {}",
                i, seg.control_points.len());
        }
    }

    #[test]
    fn test_bezier_decomposition_preserves_geometry() {
        let curve = make_cubic_bspline();
        let original_at_third = eval_curve(&curve, 0.3);
        let original_at_two_thirds = eval_curve(&curve, 0.6);

        let segments = bezier_decomposition(&curve);

        // First segment covers [0, 1] in original parameter space
        let seg0_at_third = eval_curve(&segments[0], 0.3);
        let dist0 = ((original_at_third.x - seg0_at_third.x).powi(2)
            + (original_at_third.y - seg0_at_third.y).powi(2)
            + (original_at_third.z - seg0_at_third.z).powi(2)).sqrt();
        assert!(dist0 < 1e-9,
            "Bezier decomposition segment 0 doesn't match: dist = {}", dist0);

        // Second segment covers [1, 2] in original parameter space
        // original_t = 0.6 → second segment param 0.6 - 1.0 = ... wait, second segment is [1, 2]
        // So original_t = 0.6 doesn't fit in segment 1. Let me check the parameter ranges.
        // Actually, the original curve has parameter range [0, 2] (knots[3]=0, knots[4]=1, knots[5]=2... wait
        // looking at the make_cubic_bspline: knots = [0,0,0,0,1,2,2,2,2], so range is [0, 2]
        // Segment 0 is [0, 1], segment 1 is [1, 2]
        // So original_t = 0.3 is in segment 0 at parameter 0.3
        // And original_t = 1.5 is in segment 1 at parameter 1.5 (which is 0.5 in segment 1's normalized [0,1] range,
        // but the actual parameter is 1.5).
        let original_at_one_half = eval_curve(&curve, 1.5);
        let seg1_at_one_half = eval_curve(&segments[1], 1.5);
        let dist1 = ((original_at_one_half.x - seg1_at_one_half.x).powi(2)
            + (original_at_one_half.y - seg1_at_one_half.y).powi(2)
            + (original_at_one_half.z - seg1_at_one_half.z).powi(2)).sqrt();
        assert!(dist1 < 1e-9,
            "Bezier decomposition segment 1 doesn't match: dist = {}", dist1);
    }

    #[test]
    fn test_elevate_degree_quadratic_to_cubic() {
        let curve = make_quadratic_bezier();
        let original_at_mid = eval_curve(&curve, 0.5);

        let elevated = elevate_degree(&curve);
        assert_eq!(elevated.degree, 3, "Degree should be elevated from 2 to 3");
        assert_eq!(elevated.control_points.len(), 4,
            "Cubic Bézier should have 4 control points, got {}",
            elevated.control_points.len());

        let elevated_at_mid = eval_curve(&elevated, 0.5);
        let dist = ((original_at_mid.x - elevated_at_mid.x).powi(2)
            + (original_at_mid.y - elevated_at_mid.y).powi(2)
            + (original_at_mid.z - elevated_at_mid.z).powi(2)).sqrt();
        assert!(dist < 1e-10,
            "elevate_degree changed curve geometry: dist = {}", dist);
    }

    #[test]
    fn test_elevate_degree_cubic_bspline() {
        let curve = make_cubic_bspline();
        let original_at_quarter = eval_curve(&curve, 0.25);
        let original_at_three_quarter = eval_curve(&curve, 1.5);

        let elevated = elevate_degree(&curve);
        assert_eq!(elevated.degree, 4, "Degree should be elevated from 3 to 4");

        let elevated_at_quarter = eval_curve(&elevated, 0.25);
        let dist0 = ((original_at_quarter.x - elevated_at_quarter.x).powi(2)
            + (original_at_quarter.y - elevated_at_quarter.y).powi(2)
            + (original_at_quarter.z - elevated_at_quarter.z).powi(2)).sqrt();
        assert!(dist0 < 1e-9,
            "elevate_degree changed geometry at t=0.25: dist = {}", dist0);

        let elevated_at_three_quarter = eval_curve(&elevated, 1.5);
        let dist1 = ((original_at_three_quarter.x - elevated_at_three_quarter.x).powi(2)
            + (original_at_three_quarter.y - elevated_at_three_quarter.y).powi(2)
            + (original_at_three_quarter.z - elevated_at_three_quarter.z).powi(2)).sqrt();
        assert!(dist1 < 1e-9,
            "elevate_degree changed geometry at t=1.5: dist = {}", dist1);
    }

    #[test]
    fn test_optimize_removes_redundant_knots() {
        let curve = make_quadratic_bezier();
        let original_at_mid = eval_curve(&curve, 0.5);

        // Insert several knots, then optimize to remove them
        let refined = insert_knot(&curve, 0.5, 1);
        let refined = insert_knot(&refined, 0.25, 1);
        let refined = insert_knot(&refined, 0.75, 1);

        let optimized = optimize(&refined, 1e-10);

        // The optimized curve should have the same geometry
        let optimized_at_mid = eval_curve(&optimized, 0.5);
        let dist = ((original_at_mid.x - optimized_at_mid.x).powi(2)
            + (original_at_mid.y - optimized_at_mid.y).powi(2)
            + (original_at_mid.z - optimized_at_mid.z).powi(2)).sqrt();
        assert!(dist < 1e-9,
            "optimize changed geometry: dist = {}", dist);

        // The optimized curve should have fewer or equal knots than the refined
        assert!(optimized.knots.len() <= refined.knots.len(),
            "optimize should remove knots: {} vs {}",
            optimized.knots.len(), refined.knots.len());
    }

    #[test]
    fn test_rational_curve_operations() {
        // A rational quadratic Bézier representing a quarter circle
        let curve = NurbsCurve {
            degree: 2,
            control_points: vec![
                Point3d::new(1.0, 0.0, 0.0),
                Point3d::new(1.0, 1.0, 0.0),
                Point3d::new(0.0, 1.0, 0.0),
            ],
            weights: vec![1.0, 1.0 / 2.0_f64.sqrt(), 1.0],
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        };

        let original_at_mid = eval_curve(&curve, 0.5);
        // Midpoint of a quarter circle should be at (√2/2, √2/2) ≈ (0.707, 0.707)
        let expected = std::f64::consts::FRAC_1_SQRT_2;
        assert!((original_at_mid.x - expected).abs() < 1e-6,
            "Quarter circle midpoint x = {}, expected {}", original_at_mid.x, expected);
        assert!((original_at_mid.y - expected).abs() < 1e-6,
            "Quarter circle midpoint y = {}, expected {}", original_at_mid.y, expected);

        // Insert a knot and verify geometry is preserved
        let refined = insert_knot(&curve, 0.5, 1);
        let refined_at_mid = eval_curve(&refined, 0.5);
        let dist = ((original_at_mid.x - refined_at_mid.x).powi(2)
            + (original_at_mid.y - refined_at_mid.y).powi(2)
            + (original_at_mid.z - refined_at_mid.z).powi(2)).sqrt();
        assert!(dist < 1e-10,
            "insert_knot on rational curve changed geometry: dist = {}", dist);

        // Elevate degree and verify geometry is preserved
        let elevated = elevate_degree(&curve);
        let elevated_at_mid = eval_curve(&elevated, 0.5);
        let dist2 = ((original_at_mid.x - elevated_at_mid.x).powi(2)
            + (original_at_mid.y - elevated_at_mid.y).powi(2)
            + (original_at_mid.z - elevated_at_mid.z).powi(2)).sqrt();
        assert!(dist2 < 1e-9,
            "elevate_degree on rational curve changed geometry: dist = {}", dist2);
    }

    #[test]
    fn test_find_knot_span() {
        // knots = [0,0,0,1,2,3,3,3], degree = 2, n = 5 (control points)
        let knots = vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0];
        let degree = 2;
        let n = 5;

        assert_eq!(find_knot_span(&knots, degree, 0.5, n), 2);
        assert_eq!(find_knot_span(&knots, degree, 1.0, n), 3);
        assert_eq!(find_knot_span(&knots, degree, 1.5, n), 3);
        assert_eq!(find_knot_span(&knots, degree, 2.5, n), 4);
        assert_eq!(find_knot_span(&knots, degree, 3.0, n), 4); // At the end
    }

    // ── Fuzz tests (quickcheck) ──
    // Goal: NURBS evaluation must NEVER panic or produce NaN/Inf
    // for any combination of control points, weights, and knot vectors.

    /// Fuzz 1: find_knot_span must not panic on any valid knot vector and parameter.
    #[quickcheck]
    fn fuzz_find_knot_span(n_ctrl: u8, degree: u8, t_frac: u8) -> bool {
        let n = (n_ctrl as usize).max(2);
        let p = (degree as usize).min(n - 1).max(1);
        // Build clamped uniform knot vector.
        let n_knots = n + p + 1;
        let mut knots = vec![0.0; n_knots];
        for i in 0..(n - p) {
            knots[p + i] = i as f64;
        }
        // Normalize last to 1.0
        let max_knot = knots[n_knots - 1].max(1.0);
        for k in &mut knots {
            *k /= max_knot;
        }
        let t = (t_frac as f64) / 255.0;  // [0, 1]
        // Must not panic.
        let _ = find_knot_span(&knots, p, t, n);
        true
    }

    /// Fuzz 2: NURBS curve evaluation must not produce NaN or Inf.
    #[quickcheck]
    fn fuzz_nurbs_eval_no_nan(
        n_ctrl: u8,
        degree: u8,
        t_frac: u8,
        seed_x: u8,
        seed_y: u8,
        seed_z: u8,
    ) -> bool {
        let n = (n_ctrl as usize).max(2).min(20);  // Cap at 20 control points
        let p = (degree as usize).min(n - 1).max(1);
        // Build clamped uniform knot vector.
        let n_knots = n + p + 1;
        let mut knots = vec![0.0; n_knots];
        for i in 0..=(n - p) {
            knots[p + i] = i as f64;
        }
        for i in 0..p {
            knots[n_knots - 1 - i] = (n - p) as f64;
        }
        // Generate control points from seeds.
        let control_points: Vec<crate::Point3d> = (0..n).map(|i| {
            let s = (i as f64) + 1.0;
            crate::Point3d::new(
                ((seed_x.wrapping_add(i as u8)) as f64) / 10.0 * s,
                ((seed_y.wrapping_add(i as u8)) as f64) / 10.0 * s,
                ((seed_z.wrapping_add(i as u8)) as f64) / 10.0 * s,
            )
        }).collect();
        let weights = vec![1.0; n];
        let nurbs = crate::NurbsCurve { degree: p, control_points, weights, knots };
        let t = (t_frac as f64) / 255.0;  // [0, 1]
        let curve = crate::Curve3d::Nurbs(nurbs);
        let p = curve.point_at(t);
        // Result must be finite (no NaN or Inf).
        p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
    }

    /// Fuzz 3: NURBS derivative must not produce NaN or Inf.
    #[quickcheck]
    fn fuzz_nurbs_derivative_no_nan(
        n_ctrl: u8,
        degree: u8,
        t_frac: u8,
    ) -> bool {
        let n = (n_ctrl as usize).max(2).min(20);
        let p = (degree as usize).min(n - 1).max(1);
        let n_knots = n + p + 1;
        let mut knots = vec![0.0; n_knots];
        for i in 0..=(n - p) {
            knots[p + i] = i as f64;
        }
        for i in 0..p {
            knots[n_knots - 1 - i] = (n - p) as f64;
        }
        let control_points: Vec<crate::Point3d> = (0..n).map(|i| {
            crate::Point3d::new(i as f64, (i as f64) * 0.5, 0.0)
        }).collect();
        let weights = vec![1.0; n];
        let nurbs = crate::NurbsCurve { degree: p, control_points, weights, knots };
        let t = (t_frac as f64) / 255.0;
        let d = nurbs.derivative_at(t);
        d.x.is_finite() && d.y.is_finite() && d.z.is_finite()
    }

    /// Fuzz 4: Transform application must not produce NaN.
    #[quickcheck]
    fn fuzz_transform_no_nan(
        tx: f64, ty: f64, tz: f64,
        sx: f64, sy: f64, sz: f64,
        angle: f64,
    ) -> bool {
        // Skip extreme/non-finite values.
        if !tx.is_finite() || !ty.is_finite() || !tz.is_finite() { return true; }
        if !sx.is_finite() || !sy.is_finite() || !sz.is_finite() { return true; }
        if !angle.is_finite() { return true; }
        // Skip zero/negative/extreme scales.
        if sx.abs() < 1e-10 || sy.abs() < 1e-10 || sz.abs() < 1e-10 { return true; }
        if sx.abs() > 1e10 || sy.abs() > 1e10 || sz.abs() > 1e10 { return true; }

        let t = crate::Transform::translation(tx, ty, tz);
        let s = crate::Transform::scaling(sx, sy, sz);
        let r = crate::Transform::rotation_z(angle);
        let combined = crate::Transform::multiply(&t, &s);
        let combined = crate::Transform::multiply(&combined, &r);

        let p = crate::Point3d::new(1.0, 2.0, 3.0);
        let transformed = combined.m[0][0] * p.x + combined.m[0][1] * p.y + combined.m[0][2] * p.z + combined.m[0][3];
        transformed.is_finite()
    }
}
