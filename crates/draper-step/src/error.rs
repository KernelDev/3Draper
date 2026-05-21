//! Error types for STEP parsing.

use thiserror::Error;

#[derive(Error, Debug)]
pub enum StepError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Parse error at line {line}: {message}")]
    Parse { line: usize, message: String },

    #[error("Invalid entity #{id}: {reason}")]
    InvalidEntity { id: u64, reason: String },

    #[error("Reference error: entity #{id} not found")]
    ReferenceNotFound { id: u64 },

    #[error("Type mismatch: expected {expected}, got {got}")]
    TypeMismatch { expected: String, got: String },

    #[error("Unsupported STEP feature: {0}")]
    Unsupported(String),

    #[error("Encoding error: {0}")]
    Encoding(String),
}

pub type StepResult<T> = Result<T, StepError>;
