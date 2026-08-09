// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Error types for geometric operations.
//!
//! Per ROADMAP_VISION_2036.md §12 Directive 3: Panic-Free Guarantee.
//! All mathematical modules must return `Result<T, GeometryError>` instead
//! of using `unwrap()` or `panic!()`.

use std::fmt;

/// Error type for geometric operations.
#[derive(Clone, Debug)]
pub enum GeometryError {
    /// A degenerate geometric case was encountered (zero-area triangle,
    /// collapsed surface, singular point on a curve/surface).
    DegenerateCase(String),

    /// A numerical computation produced NaN or Inf.
    NumericalError(String),

    /// An unexpected geometric variant was encountered.
    /// (e.g., expected Nurbs variant but found a different Curve3d/Surface type)
    UnexpectedVariant(String),

    /// A required entity or data was not found.
    NotFound(String),

    /// A parameter was out of the valid range.
    ParameterOutOfRange { name: String, value: f64, min: f64, max: f64 },

    /// A linear algebra operation failed (e.g., singular matrix).
    LinearAlgebraError(String),

    /// An internal invariant was violated.
    InternalError(String),
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeometryError::DegenerateCase(msg) => write!(f, "Degenerate case: {}", msg),
            GeometryError::NumericalError(msg) => write!(f, "Numerical error: {}", msg),
            GeometryError::UnexpectedVariant(msg) => write!(f, "Unexpected variant: {}", msg),
            GeometryError::NotFound(msg) => write!(f, "Not found: {}", msg),
            GeometryError::ParameterOutOfRange { name, value, min, max } => {
                write!(f, "Parameter '{}' = {} is out of range [{}, {}]", name, value, min, max)
            }
            GeometryError::LinearAlgebraError(msg) => write!(f, "Linear algebra error: {}", msg),
            GeometryError::InternalError(msg) => write!(f, "Internal error: {}", msg),
        }
    }
}

impl std::error::Error for GeometryError {}

/// Result type for geometric operations.
pub type GeometryResult<T> = Result<T, GeometryError>;

/// Check if a float value is finite (not NaN or Inf).
/// Returns `GeometryError::NumericalError` if not.
#[inline]
pub fn check_finite(value: f64, context: &str) -> GeometryResult<f64> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(GeometryError::NumericalError(format!(
            "{}: value {} is not finite (NaN or Inf)", context, value
        )))
    }
}

/// Check if a 3D point has all finite coordinates.
#[inline]
pub fn check_point_finite(x: f64, y: f64, z: f64, context: &str) -> GeometryResult<()> {
    if x.is_finite() && y.is_finite() && z.is_finite() {
        Ok(())
    } else {
        Err(GeometryError::NumericalError(format!(
            "{}: point ({}, {}, {}) has non-finite coordinates", context, x, y, z
        )))
    }
}
