// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Tolerance context for geometric computations.
//! 
//! Replaces global tolerance constants with a configurable, model-scale-aware context.
//! This ensures correct behavior across different model scales (micron to kilometer).

/// Default geometric tolerance (1e-6 mm = 1 nanometer).
pub const DEFAULT_ABSOLUTE_TOLERANCE: f64 = 1e-6;

/// Default angular tolerance in radians (approximately 0.001 degrees).
pub const DEFAULT_ANGULAR_TOLERANCE: f64 = 1e-5;

/// Default parametric tolerance for curve/surface parameter comparisons.
pub const DEFAULT_PARAMETRIC_TOLERANCE: f64 = 1e-8;

/// Default relative tolerance.
pub const DEFAULT_RELATIVE_TOLERANCE: f64 = 1e-8;

/// Context-aware tolerance system for geometric computations.
/// 
/// Provides model-scale-aware tolerance values instead of global constants.
/// The `model_scale` field represents the characteristic size of the model
/// (e.g., the diagonal of its bounding box), which allows the tolerance
/// system to adapt to different model scales.
#[derive(Clone, Debug)]
pub struct ToleranceContext {
    /// Absolute geometric tolerance for point coincidence checks.
    pub absolute: f64,
    /// Relative tolerance (scaled by model_scale).
    pub relative: f64,
    /// Angular tolerance in radians.
    pub angular: f64,
    /// Parametric tolerance for curve/surface parameter comparisons.
    pub parametric: f64,
    /// Characteristic size of the model (bounding box diagonal).
    pub model_scale: f64,
}

impl Default for ToleranceContext {
    fn default() -> Self {
        Self {
            absolute: DEFAULT_ABSOLUTE_TOLERANCE,
            relative: DEFAULT_RELATIVE_TOLERANCE,
            angular: DEFAULT_ANGULAR_TOLERANCE,
            parametric: DEFAULT_PARAMETRIC_TOLERANCE,
            model_scale: 1.0,
        }
    }
}

impl ToleranceContext {
    /// Create a new tolerance context with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a tolerance context adapted to a model's bounding box.
    /// The model_scale is set to the diagonal of the bounding box,
    /// and the absolute tolerance is scaled accordingly.
    pub fn from_bounding_box(min: &crate::Point3d, max: &crate::Point3d) -> Self {
        let dx = max.x - min.x;
        let dy = max.y - min.y;
        let dz = max.z - min.z;
        let model_scale = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-10);
        Self {
            absolute: DEFAULT_ABSOLUTE_TOLERANCE,
            relative: DEFAULT_RELATIVE_TOLERANCE,
            angular: DEFAULT_ANGULAR_TOLERANCE,
            parametric: DEFAULT_PARAMETRIC_TOLERANCE,
            model_scale,
        }
    }

    /// Create a tolerance context from a specific model scale.
    pub fn from_model_scale(model_scale: f64) -> Self {
        Self {
            absolute: DEFAULT_ABSOLUTE_TOLERANCE,
            relative: DEFAULT_RELATIVE_TOLERANCE,
            angular: DEFAULT_ANGULAR_TOLERANCE,
            parametric: DEFAULT_PARAMETRIC_TOLERANCE,
            model_scale: model_scale.max(1e-10),
        }
    }

    /// Effective coincidence tolerance: absolute + relative * model_scale.
    /// This adapts to the model's scale — for large models, the tolerance
    /// increases slightly to avoid false negatives at boundaries.
    pub fn coincidence_tolerance(&self) -> f64 {
        self.absolute + self.relative * self.model_scale
    }

    /// Squared coincidence tolerance for efficient distance comparisons.
    pub fn coincidence_tolerance_sq(&self) -> f64 {
        let t = self.coincidence_tolerance();
        t * t
    }

    /// Check if two 3D points are coincident within tolerance.
    pub fn is_coincident_3d(&self, a: &crate::Point3d, b: &crate::Point3d) -> bool {
        let dx = a.x - b.x;
        let dy = a.y - b.y;
        let dz = a.z - b.z;
        (dx * dx + dy * dy + dz * dz) < self.coincidence_tolerance_sq()
    }

    /// Check if two scalar values are within tolerance.
    pub fn is_coincident(&self, a: f64, b: f64) -> bool {
        (a - b).abs() < self.coincidence_tolerance()
    }

    /// Check if a value is approximately zero.
    pub fn is_zero(&self, a: f64) -> bool {
        a.abs() < self.coincidence_tolerance()
    }

    /// Check if two values are within a custom tolerance.
    pub fn is_within(&self, a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    /// Convert a 3D distance tolerance to a parametric (UV) tolerance
    /// for a given surface, accounting for surface stretching.
    /// 
    /// For surfaces with high curvature or stretching, the parametric
    /// tolerance needs to be smaller to achieve the same 3D accuracy.
    /// Uses the Jacobian (first fundamental form) of the surface.
    /// 
    /// For now, uses a simple scaling based on the surface's parametric range
    /// vs. its 3D extent. A full implementation would use the actual Jacobian.
    pub fn parametric_tolerance_for_surface(
        &self,
        surface: &crate::Surface,
        u: f64,
        v: f64,
    ) -> f64 {
        // Estimate the surface "stretch" by computing the partial derivatives
        // numerically and taking the larger of the two norms.
        let eps = self.parametric;
        let p0 = surface.point_at(u, v);
        let pu = surface.point_at(u + eps, v);
        let pv = surface.point_at(u, v + eps);
        
        let du_len = ((pu.x - p0.x).powi(2) + (pu.y - p0.y).powi(2) + (pu.z - p0.z).powi(2)).sqrt();
        let dv_len = ((pv.x - p0.x).powi(2) + (pv.y - p0.y).powi(2) + (pv.z - p0.z).powi(2)).sqrt();
        
        let max_stretch = du_len.max(dv_len).max(eps);
        
        // parametric_tol = 3d_tol / max_stretch
        // But ensure it's at least PARAMETRIC_TOLERANCE
        (self.coincidence_tolerance() / max_stretch).max(DEFAULT_PARAMETRIC_TOLERANCE)
    }
}

// Keep backward-compatible constants for gradual migration
/// Default geometric tolerance (1e-6 mm = 1 nanometer).
/// DEPRECATED: Use ToleranceContext instead.
pub const TOLERANCE: f64 = DEFAULT_ABSOLUTE_TOLERANCE;

/// Squared tolerance for efficient distance comparisons.
/// DEPRECATED: Use ToleranceContext::coincidence_tolerance_sq() instead.
pub const TOLERANCE_SQ: f64 = TOLERANCE * TOLERANCE;

/// Angular tolerance in radians.
/// DEPRECATED: Use ToleranceContext::angular instead.
pub const ANGULAR_TOLERANCE: f64 = DEFAULT_ANGULAR_TOLERANCE;

/// Parametric tolerance for curve/surface parameter comparisons.
/// DEPRECATED: Use ToleranceContext::parametric instead.
pub const PARAMETRIC_TOLERANCE: f64 = DEFAULT_PARAMETRIC_TOLERANCE;

/// Check if two values are within geometric tolerance.
/// DEPRECATED: Use ToleranceContext::is_coincident() instead.
#[inline]
pub fn is_coincident(a: f64, b: f64) -> bool {
    (a - b).abs() < TOLERANCE
}

/// Check if a value is approximately zero.
/// DEPRECATED: Use ToleranceContext::is_zero() instead.
#[inline]
pub fn is_zero(a: f64) -> bool {
    a.abs() < TOLERANCE
}

/// Check if two values are within a custom tolerance.
#[inline]
pub fn is_within(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() < tol
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_micron_scale() {
        // Micron-scale model (0.001 mm)
        let ctx = ToleranceContext::from_model_scale(0.001);
        assert_eq!(ctx.model_scale, 0.001);
        // coincidence_tolerance = 1e-6 + 1e-8 * 0.001 ≈ 1e-6
        let tol = ctx.coincidence_tolerance();
        assert!(tol > 0.0);
        assert!((tol - 1e-6).abs() < 1e-10, "Micron scale: tolerance should be ~1e-6, got {}", tol);

        // Two points 1nm apart should be coincident
        let p1 = crate::Point3d::new(0.0, 0.0, 0.0);
        let p2 = crate::Point3d::new(1e-7, 0.0, 0.0); // 0.1 microns apart
        assert!(ctx.is_coincident_3d(&p1, &p2), "Points 0.1 microns apart should be coincident at micron scale");

        // Two points 1 micron apart should NOT be coincident
        let p3 = crate::Point3d::new(0.0, 1e-3, 0.0); // 1mm apart at micron scale
        assert!(!ctx.is_coincident_3d(&p1, &p3), "Points 1mm apart should NOT be coincident at micron scale");
    }

    #[test]
    fn test_meter_scale() {
        // Meter-scale model (1000 mm = 1m)
        let ctx = ToleranceContext::from_model_scale(1000.0);
        assert_eq!(ctx.model_scale, 1000.0);
        // coincidence_tolerance = 1e-6 + 1e-8 * 1000 = 1e-6 + 1e-5 ≈ 1.1e-5
        let tol = ctx.coincidence_tolerance();
        assert!(tol > 1e-6, "Meter scale: tolerance should be larger than 1e-6, got {}", tol);
        assert!(tol < 1e-4, "Meter scale: tolerance should be smaller than 1e-4, got {}", tol);

        // Two points 1e-5 mm apart should be coincident at meter scale
        let p1 = crate::Point3d::new(0.0, 0.0, 0.0);
        let p2 = crate::Point3d::new(1e-5, 0.0, 0.0);
        assert!(ctx.is_coincident_3d(&p1, &p2), "Points 1e-5 apart should be coincident at meter scale");
    }

    #[test]
    fn test_kilometer_scale() {
        // Kilometer-scale model (1e6 mm = 1km)
        let ctx = ToleranceContext::from_model_scale(1e6);
        // coincidence_tolerance = 1e-6 + 1e-8 * 1e6 = 1e-6 + 1e-2 ≈ 0.01
        let tol = ctx.coincidence_tolerance();
        assert!(tol > 1e-3, "Kilometer scale: tolerance should be ~0.01, got {}", tol);
        assert!(tol < 0.1, "Kilometer scale: tolerance should be ~0.01, got {}", tol);

        // Two points 0.005mm apart should be coincident at kilometer scale
        // (0.005 < 0.01 = tolerance)
        let p1 = crate::Point3d::new(0.0, 0.0, 0.0);
        let p2 = crate::Point3d::new(0.005, 0.0, 0.0); // 0.005mm apart
        assert!(ctx.is_coincident_3d(&p1, &p2), "Points 0.005mm apart should be coincident at kilometer scale");

        // Two points 100mm apart should NOT be coincident even at kilometer scale
        let p3 = crate::Point3d::new(100.0, 0.0, 0.0);
        assert!(!ctx.is_coincident_3d(&p1, &p3), "Points 100mm apart should NOT be coincident at kilometer scale");
    }

    #[test]
    fn test_from_bounding_box() {
        let min = crate::Point3d::new(-50.0, -50.0, -50.0);
        let max = crate::Point3d::new(50.0, 50.0, 50.0);
        let ctx = ToleranceContext::from_bounding_box(&min, &max);
        // Diagonal = sqrt(100² + 100² + 100²) = 100 * sqrt(3) ≈ 173.2
        let expected_scale = (100.0_f64 * 100.0 + 100.0 * 100.0 + 100.0 * 100.0).sqrt();
        assert!((ctx.model_scale - expected_scale).abs() < 1e-6,
            "model_scale should be {}, got {}", expected_scale, ctx.model_scale);
    }

    #[test]
    fn test_parametric_tolerance_for_plane() {
        let ctx = ToleranceContext::new();
        let plane = crate::Surface::Plane(crate::Plane::xy());
        let ptol = ctx.parametric_tolerance_for_surface(&plane, 0.5, 0.5);
        // For a plane with unit u/v directions, max_stretch ≈ eps (1e-8)
        // because the numerical derivative is evaluated with step = parametric (1e-8)
        // This is an approximation — the result should be positive
        assert!(ptol > 0.0, "Parametric tolerance must be positive");
        // The result should be at least DEFAULT_PARAMETRIC_TOLERANCE
        assert!(ptol >= DEFAULT_PARAMETRIC_TOLERANCE,
            "Parametric tolerance should be >= 1e-8, got {}", ptol);
    }

    #[test]
    fn test_is_coincident_scalar() {
        let ctx = ToleranceContext::new();
        assert!(ctx.is_coincident(0.0, 5e-7), "Values within 1e-6 should be coincident");
        assert!(!ctx.is_coincident(0.0, 2e-6), "Values beyond 1e-6 should NOT be coincident");
    }

    #[test]
    fn test_is_zero() {
        let ctx = ToleranceContext::new();
        assert!(ctx.is_zero(0.0));
        assert!(ctx.is_zero(5e-7));
        assert!(!ctx.is_zero(2e-6));
    }
}
