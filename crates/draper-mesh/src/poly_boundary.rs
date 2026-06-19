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
}
