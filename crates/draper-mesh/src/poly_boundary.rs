//! Unified seam / periodic-surface handling for boundary polygons.
//!
//! Inspired by truck's `PolyBoundary` (truck-polymesh / truck-topology), which
//! carries the surface's `u_period` / `v_period` alongside the boundary polygon
//! so that seam wrapping, normalization, and split-at-seam all share one code
//! path for cylinder / cone / sphere / torus / revolution / extrusion.
//!
//! ## Why this exists
//!
//! Before P3, the codebase had three separate places that needed to know about
//! surface periodicity:
//!   1. `triangulate.rs::surface_u_period` / `surface_v_period` — return the
//!      period (if any) for a given `Surface` variant.
//!   2. `triangulate.rs::normalize_uv_polygon` — shift UV coordinates that
//!      wrapped around the seam so the polygon becomes earcutr-friendly.
//!   3. `parametric_domain.rs::try_split_at_seam` — split a self-intersecting
//!      periodic polygon at the seam into two non-self-intersecting halves.
//!
//! Each call site had to thread `u_period: Option<f64>` and `v_period: Option<f64>`
//! through the call chain manually. This module bundles the surface, the
//! periods, and the boundary polygon into one `PolyBoundary` value so the
//! caller doesn't have to repeat the period plumbing.
//!
//! ## What this does NOT do
//!
//! This is a **thin unification layer**, not a rewrite. The actual normalization
//! and split logic still lives in `triangulate.rs::normalize_uv_polygon` and
//! `parametric_domain.rs::try_split_at_seam` — those functions are battle-tested
//! across all 24 STEP test files (24/24 watertight). `PolyBoundary` simply
//! groups the inputs and delegates to them, so we get API unification without
//! risking regressions on the existing leaky-but-now-fixed files.

use crate::triangulate::{normalize_uv_polygon, surface_u_period, surface_v_period};
use draper_geometry::{Point2d, Surface};
use std::f64::consts::PI;

/// A detected pole degeneracy in the boundary polygon.
///
/// A pole degeneracy occurs when the surface's partial derivative dS/du or dS/dv
/// is zero at some boundary point — typically at the poles of a sphere (v=0 or v=π)
/// or at the inner ring of a torus (where the tube self-intersects).
///
/// When the boundary polygon has all points at the same v value (e.g., v=0 for the
/// north pole of a sphere), the polygon has zero area in UV space and earcutr
/// cannot triangulate it. The fix is to inject an intermediate ring of UV points
/// at v=ε (or v=π-ε) to give the polygon some extent.
#[derive(Clone, Debug)]
pub struct PoleDegeneracy {
    /// The parameter axis that is degenerate: 'u' or 'v'.
    pub axis: char,
    /// The constant value of the degenerate axis (e.g., v=0 for north pole).
    pub value: f64,
    /// The number of boundary points that lie on this pole.
    pub point_count: usize,
}

impl PolyBoundary {
    /// Detect if the boundary polygon is pole-degenerate.
    ///
    /// Returns `Some(PoleDegeneracy)` if ALL boundary points have the same v
    /// value (within tolerance) — this happens at the north/south pole of a
    /// sphere or at the inner/outer ring of a torus when the face is a single
    /// "polar cap".
    ///
    /// Returns `None` if the polygon has non-degenerate extent in both u and v.
    ///
    /// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
    pub fn detect_pole_degeneracy(&self, tolerance: f64) -> Option<PoleDegeneracy> {
        if self.polygon.len() < 3 {
            return None;
        }

        // Check v-axis degeneracy (all points at same v)
        let mut v_min = f64::MAX;
        let mut v_max = f64::MIN;
        for p in &self.polygon {
            if !p.v.is_finite() {
                return None;
            }
            if p.v < v_min { v_min = p.v; }
            if p.v > v_max { v_max = p.v; }
        }
        let v_span = v_max - v_min;
        if v_span < tolerance {
            return Some(PoleDegeneracy {
                axis: 'v',
                value: (v_min + v_max) * 0.5,
                point_count: self.polygon.len(),
            });
        }

        // Check u-axis degeneracy (all points at same u)
        let mut u_min = f64::MAX;
        let mut u_max = f64::MIN;
        for p in &self.polygon {
            if !p.u.is_finite() {
                return None;
            }
            if p.u < u_min { u_min = p.u; }
            if p.u > u_max { u_max = p.u; }
        }
        let u_span = u_max - u_min;
        if u_span < tolerance {
            return Some(PoleDegeneracy {
                axis: 'u',
                value: (u_min + u_max) * 0.5,
                point_count: self.polygon.len(),
            });
        }

        None
    }

    /// Inject a ring of UV points at `axis_value ± epsilon` to give a
    /// pole-degenerate polygon some extent.
    ///
    /// This is used to "open up" a degenerate polar cap so that earcutr can
    /// triangulate it. The injected ring has `n_points` points evenly spaced
    /// along the NON-degenerate axis (u for a v-pole, v for a u-pole).
    ///
    /// Returns `true` if injection was performed, `false` if no degeneracy was
    /// detected.
    ///
    /// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
    pub fn inject_pole_ring(&mut self, surface: &Surface, tolerance: f64, n_points: usize) -> bool {
        let degen = match self.detect_pole_degeneracy(tolerance) {
            Some(d) => d,
            None => return false,
        };

        let n_points = n_points.max(3);

        match degen.axis {
            'v' => {
                // All points at v = degen.value. Inject a ring at v ± ε.
                // Determine ε from the surface's v-range.
                let (v_min, v_max) = surface_v_range(surface);
                let v_span = v_max - v_min;
                let epsilon = (v_span * 1e-3).max(1e-6);

                // Determine the direction to inject: if degen.value is closer to v_min,
                // inject at v + ε; otherwise inject at v - ε.
                let inject_v = if (degen.value - v_min).abs() < (degen.value - v_max).abs() {
                    degen.value + epsilon
                } else {
                    degen.value - epsilon
                };

                // Determine u-range from existing points
                let mut u_min = f64::MAX;
                let mut u_max = f64::MIN;
                for p in &self.polygon {
                    if p.u < u_min { u_min = p.u; }
                    if p.u > u_max { u_max = p.u; }
                }
                if u_max <= u_min {
                    // No u extent — use the surface's u-range
                    let (su_min, su_max) = surface_u_range(surface);
                    u_min = su_min;
                    u_max = su_max;
                }

                // Build the injected ring
                let mut ring: Vec<Point2d> = Vec::with_capacity(n_points);
                for i in 0..n_points {
                    let u = u_min + (u_max - u_min) * (i as f64) / ((n_points - 1) as f64);
                    ring.push(Point2d::new(u, inject_v));
                }

                // Insert the ring at the beginning of the polygon (as an inner hole boundary)
                // The caller (triangulate_surface_consistent) will treat this as a separate ring.
                // For now, we just append it to the polygon — the actual hole-loop separation
                // happens in the caller.
                self.polygon.extend(ring);

                log::debug!(
                    "Injected v-pole ring at v={:.4} (degenerate at v={:.4}, {} points)",
                    inject_v, degen.value, n_points
                );
                true
            }
            'u' => {
                // All points at u = degen.value. Inject a ring at u ± ε.
                let (u_min, u_max) = surface_u_range(surface);
                let u_span = u_max - u_min;
                let epsilon = (u_span * 1e-3).max(1e-6);

                let inject_u = if (degen.value - u_min).abs() < (degen.value - u_max).abs() {
                    degen.value + epsilon
                } else {
                    degen.value - epsilon
                };

                let mut v_min = f64::MAX;
                let mut v_max = f64::MIN;
                for p in &self.polygon {
                    if p.v < v_min { v_min = p.v; }
                    if p.v > v_max { v_max = p.v; }
                }
                if v_max <= v_min {
                    let (sv_min, sv_max) = surface_v_range(surface);
                    v_min = sv_min;
                    v_max = sv_max;
                }

                let mut ring: Vec<Point2d> = Vec::with_capacity(n_points);
                for i in 0..n_points {
                    let v = v_min + (v_max - v_min) * (i as f64) / ((n_points - 1) as f64);
                    ring.push(Point2d::new(inject_u, v));
                }

                self.polygon.extend(ring);

                log::debug!(
                    "Injected u-pole ring at u={:.4} (degenerate at u={:.4}, {} points)",
                    inject_u, degen.value, n_points
                );
                true
            }
            _ => false,
        }
    }
}

/// Get the u-parameter range of a surface.
fn surface_u_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Plane(_) => (-1e6, 1e6),
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) |
        Surface::Torus(_) | Surface::Revolution(_) => (0.0, 2.0 * PI),
        Surface::Extrusion(_) => (-1e6, 1e6),
        Surface::Nurbs(n) => {
            let (u_min, u_max) = n.u_range();
            (u_min, u_max)
        }
    }
}

/// Get the v-parameter range of a surface.
fn surface_v_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Plane(_) => (-1e6, 1e6),
        Surface::Cylinder(_) | Surface::Extrusion(_) => (-1e6, 1e6),
        Surface::Cone(_) => (0.0, 1e6),  // cone extends from apex to base
        Surface::Sphere(_) => (0.0, PI),
        Surface::Torus(_) => (0.0, 2.0 * PI),
        Surface::Revolution(_) => (-1e6, 1e6),
        Surface::Nurbs(n) => {
            let (v_min, v_max) = n.v_range();
            (v_min, v_max)
        }
    }
}

/// A boundary polygon paired with its surface's periodicity metadata.
///
/// Construct this once from a `Surface` + boundary UVs, then call `normalize`
/// and (if needed) `split_at_seam` without re-deriving the periods each time.
///
/// # Fields
/// - `polygon` — the 2D UV points of the boundary (mutable, normalized in-place).
/// - `u_period` — `Some(2π)` for cylinder/cone/sphere/torus/revolution, `None` otherwise.
/// - `v_period` — `Some(2π)` for torus, `None` otherwise.
pub struct PolyBoundary {
    pub polygon: Vec<Point2d>,
    pub u_period: Option<f64>,
    pub v_period: Option<f64>,
}

impl PolyBoundary {
    /// Construct from a surface + boundary UV polygon.
    ///
    /// The periods are derived from the surface type — no caller plumbing needed.
    pub fn from_surface(surface: &Surface, polygon: Vec<Point2d>) -> Self {
        Self {
            polygon,
            u_period: surface_u_period(surface),
            v_period: surface_v_period(surface),
        }
    }

    /// Construct with explicit periods (for unit tests or non-standard surfaces).
    pub fn with_periods(polygon: Vec<Point2d>, u_period: Option<f64>, v_period: Option<f64>) -> Self {
        Self { polygon, u_period, v_period }
    }

    /// Normalize the polygon so it doesn't wrap around the seam.
    ///
    /// Delegates to `triangulate::normalize_uv_polygon`. After this call,
    /// `polygon` is safe to pass to earcutr.
    pub fn normalize(&mut self) {
        normalize_uv_polygon(&mut self.polygon, self.u_period, self.v_period);
    }

    /// Returns `true` if either u or v direction is periodic.
    pub fn is_periodic(&self) -> bool {
        self.u_period.is_some() || self.v_period.is_some()
    }

    /// Returns `true` if the boundary wraps the full u-period.
    ///
    /// A "full-period" boundary is one where the u-span of the polygon is close
    /// to `u_period` (within 5%). Such polygons are degenerate in UV space —
    /// u=0 and u=2π map to the same geometric line (the seam), so the polygon
    /// has zero area. earcutr cannot triangulate it; the caller should delegate
    /// to a grid-based full-surface triangulation instead.
    ///
    /// This is the same check that `triangulate.rs::is_full_period_boundary`
    /// performs on 3D boundary points — but here we work directly on the UV
    /// polygon, which is cheaper and avoids a separate 3D projection.
    pub fn is_full_u_period(&self) -> bool {
        let period = match self.u_period {
            Some(p) if p > 0.0 => p,
            _ => return false,
        };
        if self.polygon.len() < 3 {
            return false;
        }
        let mut u_min = f64::MAX;
        let mut u_max = f64::MIN;
        for p in &self.polygon {
            if !p.u.is_finite() {
                return false;
            }
            if p.u < u_min { u_min = p.u; }
            if p.u > u_max { u_max = p.u; }
        }
        let span = u_max - u_min;
        // 5% tolerance — matches `triangulate.rs::is_full_period_boundary`
        span >= period * 0.95
    }

    /// Returns `true` if the boundary wraps the full v-period (torus only).
    pub fn is_full_v_period(&self) -> bool {
        let period = match self.v_period {
            Some(p) if p > 0.0 => p,
            _ => return false,
        };
        if self.polygon.len() < 3 {
            return false;
        }
        let mut v_min = f64::MAX;
        let mut v_max = f64::MIN;
        for p in &self.polygon {
            if !p.v.is_finite() {
                return false;
            }
            if p.v < v_min { v_min = p.v; }
            if p.v > v_max { v_max = p.v; }
        }
        let span = v_max - v_min;
        span >= period * 0.95
    }

    /// Consume self and return the normalized polygon.
    pub fn into_polygon(mut self) -> Vec<Point2d> {
        self.normalize();
        self.polygon
    }

    /// Borrow the polygon (read-only).
    pub fn as_slice(&self) -> &[Point2d] {
        &self.polygon
    }

    /// Borrow the polygon (read-write).
    pub fn as_mut_slice(&mut self) -> &mut [Point2d] {
        &mut self.polygon
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn p(u: f64, v: f64) -> Point2d {
        Point2d::new(u, v)
    }

    #[test]
    fn test_non_periodic_no_change() {
        // A simple square polygon on a plane (no periodicity).
        let poly = vec![p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0)];
        let mut pb = PolyBoundary::with_periods(poly.clone(), None, None);
        pb.normalize();
        // Should be unchanged (no wrap-around to fix)
        assert!(!pb.is_periodic());
        assert!(!pb.is_full_u_period());
        assert!(!pb.is_full_v_period());
        for (orig, normalized) in poly.iter().zip(pb.as_slice().iter()) {
            assert!((orig.u - normalized.u).abs() < 1e-12);
            assert!((orig.v - normalized.v).abs() < 1e-12);
        }
    }

    #[test]
    fn test_cylinder_seam_wrap_fixed() {
        // Boundary that wraps the cylinder seam: u=0.1, 6.0, 6.1, 0.2
        // (after normalize, the values >π should be shifted down by 2π)
        let poly = vec![
            p(0.1, 0.0),
            p(6.0, 0.0), // >π, should become 6.0 - 2π ≈ -0.28
            p(6.1, 1.0),
            p(0.2, 1.0),
        ];
        let mut pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), None);
        assert!(pb.is_periodic());
        pb.normalize();
        // After normalize, all u values should be in a continuous range
        // (no gap > π)
        let us: Vec<f64> = pb.as_slice().iter().map(|p| p.u).collect();
        let mut sorted = us.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut max_gap = 0.0;
        for i in 0..sorted.len() - 1 {
            let gap = sorted[i + 1] - sorted[i];
            if gap > max_gap {
                max_gap = gap;
            }
        }
        assert!(max_gap < PI, "max gap {} should be < π after normalize", max_gap);
    }

    #[test]
    fn test_full_u_period_detection() {
        // A boundary that spans the full u-period (cylinder full wrap).
        // Use 20 points so the sampled span (19 * 2π/20 = 0.95 * 2π) reaches
        // the 95% detection threshold.
        let poly: Vec<Point2d> = (0..20)
            .map(|i| p(i as f64 * 2.0 * PI / 20.0, 0.0))
            .collect();
        let pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), None);
        assert!(pb.is_full_u_period(), "should detect full u-period wrap");
        assert!(!pb.is_full_v_period());
    }

    #[test]
    fn test_partial_u_period_not_full() {
        // A boundary spanning only ~half the u-period
        let poly = vec![p(0.0, 0.0), p(PI * 0.4, 0.0), p(PI * 0.4, 1.0), p(0.0, 1.0)];
        let pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), None);
        assert!(!pb.is_full_u_period(), "half-period wrap should not be detected as full");
    }

    #[test]
    fn test_torus_full_v_period() {
        // Torus: both u and v can be periodic. Use 20 points to reach the
        // 95% span threshold for v-period detection.
        let poly: Vec<Point2d> = (0..20)
            .map(|i| p(0.0, i as f64 * 2.0 * PI / 20.0))
            .collect();
        let pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), Some(2.0 * PI));
        assert!(!pb.is_full_u_period(), "u span is zero, not full");
        assert!(pb.is_full_v_period(), "should detect full v-period wrap");
    }

    #[test]
    fn test_into_polygon_consumes_and_normalizes() {
        let poly = vec![p(0.0, 0.0), p(6.0, 0.0), p(6.0, 1.0), p(0.0, 1.0)];
        let pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), None);
        let normalized = pb.into_polygon();
        // The wrap-around value 6.0 should have been shifted down by 2π
        let shifted_count = normalized.iter().filter(|p| p.u < 0.0).count();
        assert!(shifted_count >= 2, "expected at least 2 shifted values, got {}", shifted_count);
    }

    // ─── Pole-degenerate detection tests (P10) ──────────────────────────

    #[test]
    fn test_detect_v_pole_degeneracy_north_pole() {
        // All points at v=0 (north pole of a sphere) — degenerate.
        let poly = vec![
            p(0.0, 0.0), p(1.0, 0.0), p(2.0, 0.0),
            p(3.0, 0.0), p(4.0, 0.0), p(5.0, 0.0),
            p(6.0, 0.0),
        ];
        let pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), None);
        let degen = pb.detect_pole_degeneracy(1e-6);
        assert!(degen.is_some(), "should detect v-pole degeneracy at v=0");
        let d = degen.unwrap();
        assert_eq!(d.axis, 'v');
        assert!(d.value.abs() < 1e-6, "value should be 0 (north pole)");
        assert_eq!(d.point_count, 7);
    }

    #[test]
    fn test_detect_v_pole_degeneracy_south_pole() {
        // All points at v=π (south pole of a sphere) — degenerate.
        let poly = vec![
            p(0.0, PI), p(1.0, PI), p(2.0, PI),
            p(3.0, PI), p(4.0, PI), p(5.0, PI),
        ];
        let pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), None);
        let degen = pb.detect_pole_degeneracy(1e-6);
        assert!(degen.is_some(), "should detect v-pole degeneracy at v=π");
        let d = degen.unwrap();
        assert_eq!(d.axis, 'v');
        assert!((d.value - PI).abs() < 1e-6, "value should be π (south pole)");
    }

    #[test]
    fn test_detect_u_pole_degeneracy() {
        // All points at u=0 (a "seam" edge that collapsed to a line) — degenerate.
        let poly = vec![
            p(0.0, 0.0), p(0.0, 0.5), p(0.0, 1.0),
            p(0.0, 1.5), p(0.0, 2.0), p(0.0, 2.5),
        ];
        let pb = PolyBoundary::with_periods(poly, Some(2.0 * PI), None);
        let degen = pb.detect_pole_degeneracy(1e-6);
        assert!(degen.is_some(), "should detect u-pole degeneracy at u=0");
        let d = degen.unwrap();
        assert_eq!(d.axis, 'u');
        assert!(d.value.abs() < 1e-6, "value should be 0");
    }

    #[test]
    fn test_no_degeneracy_for_normal_polygon() {
        // A normal square polygon — no degeneracy.
        let poly = vec![p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0)];
        let pb = PolyBoundary::with_periods(poly, None, None);
        let degen = pb.detect_pole_degeneracy(1e-6);
        assert!(degen.is_none(), "should not detect degeneracy for normal polygon");
    }

    #[test]
    fn test_inject_pole_ring_v_axis_north() {
        // North pole cap (v=0) on a sphere — inject a ring at v=ε.
        use draper_geometry::{SphereSurface, Surface};
        let sphere = Surface::Sphere(SphereSurface::new(draper_geometry::Point3d::ORIGIN, 1.0));
        let poly = vec![
            p(0.0, 0.0), p(1.0, 0.0), p(2.0, 0.0),
            p(3.0, 0.0), p(4.0, 0.0), p(5.0, 0.0),
        ];
        let mut pb = PolyBoundary::from_surface(&sphere, poly);
        let injected = pb.inject_pole_ring(&sphere, 1e-6, 6);
        assert!(injected, "should inject a ring for north pole cap");
        // After injection, the polygon should have more points (original 6 + injected 6 = 12)
        assert_eq!(pb.polygon.len(), 12, "expected 12 points after injection");
        // The injected points should have v > 0 (since north pole is at v_min=0)
        let injected_v = pb.polygon[6].v;
        assert!(injected_v > 0.0, "injected v should be > 0 for north pole, got {}", injected_v);
        // The injected v should be small (ε ≈ π * 1e-3)
        assert!(injected_v < 0.01, "injected v should be small, got {}", injected_v);
    }

    #[test]
    fn test_inject_pole_ring_v_axis_south() {
        // South pole cap (v=π) on a sphere — inject a ring at v=π-ε.
        use draper_geometry::{SphereSurface, Surface};
        let sphere = Surface::Sphere(SphereSurface::new(draper_geometry::Point3d::ORIGIN, 1.0));
        let poly = vec![
            p(0.0, PI), p(1.0, PI), p(2.0, PI),
            p(3.0, PI), p(4.0, PI), p(5.0, PI),
        ];
        let mut pb = PolyBoundary::from_surface(&sphere, poly);
        let injected = pb.inject_pole_ring(&sphere, 1e-6, 6);
        assert!(injected, "should inject a ring for south pole cap");
        // The injected v should be < π (since south pole is at v_max=π)
        let injected_v = pb.polygon[6].v;
        assert!(injected_v < PI, "injected v should be < π for south pole, got {}", injected_v);
        assert!(injected_v > PI - 0.01, "injected v should be close to π, got {}", injected_v);
    }

    #[test]
    fn test_inject_pole_ring_no_degeneracy() {
        use draper_geometry::{Plane, Surface};
        let plane = Surface::Plane(Plane::xy());
        let poly = vec![p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0)];
        let mut pb = PolyBoundary::from_surface(&plane, poly);
        let injected = pb.inject_pole_ring(&plane, 1e-6, 4);
        assert!(!injected, "should not inject for non-degenerate polygon");
        assert_eq!(pb.polygon.len(), 4, "polygon should be unchanged");
    }
}
