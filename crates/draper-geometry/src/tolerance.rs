//! Tolerance and precision constants for geometric computations.

/// Default geometric tolerance for point coincidence checks (1e-6 mm = 1 nanometer).
pub const TOLERANCE: f64 = 1e-6;

/// Squared tolerance for efficient distance comparisons.
pub const TOLERANCE_SQ: f64 = TOLERANCE * TOLERANCE;

/// Angular tolerance in radians (approximately 0.001 degrees).
pub const ANGULAR_TOLERANCE: f64 = 1e-5;

/// Parametric tolerance for curve/surface parameter comparisons.
pub const PARAMETRIC_TOLERANCE: f64 = 1e-8;

/// Check if two values are within geometric tolerance.
#[inline]
pub fn is_coincident(a: f64, b: f64) -> bool {
    (a - b).abs() < TOLERANCE
}

/// Check if a value is approximately zero.
#[inline]
pub fn is_zero(a: f64) -> bool {
    a.abs() < TOLERANCE
}

/// Check if two values are within a custom tolerance.
#[inline]
pub fn is_within(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() < tol
}
