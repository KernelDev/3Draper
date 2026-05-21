//! Error types for draper-core.

use thiserror::Error;

#[derive(Error, Debug)]
pub enum CoreError {
    #[error("STEP error: {0}")]
    Step(#[from] draper_step::error::StepError),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Mesh generation error: {0}")]
    Mesh(String),

    #[error("Invalid operation: {0}")]
    InvalidOperation(String),

    #[error("Entity not found: {0}")]
    NotFound(String),
}

pub type CoreResult<T> = Result<T, CoreError>;
