//! Error types for the 3Draper geometric kernel.
//!
//! Defines a comprehensive error hierarchy that replaces `unwrap()`/`expect()`
//! and `Result<_, String>` with typed, structured errors. Each variant carries
//! contextual information for debugging and error reporting.
//!
//! # Error Categories
//!
//! - **Geometry**: Surface/curve evaluation failures, degenerate geometry, NaN/Inf
//! - **Topology**: Invalid B-Rep structure, broken references, non-manifold topology
//! - **Mesh**: Triangulation failures, degenerate triangles, non-watertight meshes
//! - **STEP**: Parsing errors, unsupported entities, missing references
//! - **IO**: File read/write errors, invalid paths

use thiserror::Error;

/// Root error type for the 3Draper geometric kernel.
///
/// All kernel operations that can fail should return `Result<T, KernelError>`.
/// This replaces `unwrap()`, `expect()`, and `Result<_, String>` with
/// structured, typed errors that carry contextual information.
#[derive(Error, Debug)]
pub enum KernelError {
    /// Geometry evaluation error.
    #[error("Geometry error: {0}")]
    Geometry(#[from] GeometryError),

    /// Topology (B-Rep) error.
    #[error("Topology error: {0}")]
    Topology(#[from] TopologyError),

    /// Mesh/triangulation error.
    #[error("Mesh error: {0}")]
    Mesh(#[from] MeshError),

    /// STEP file parsing/conversion error.
    #[error("STEP error: {0}")]
    Step(#[from] StepError),

    /// I/O error.
    #[error("I/O error: {0}")]
    Io(#[from] IoError),

    /// General internal error with message.
    #[error("Internal error: {0}")]
    Internal(String),
}

/// Geometry-related errors: surface evaluation, curve evaluation, degenerate cases.
#[derive(Error, Debug)]
pub enum GeometryError {
    /// A surface evaluation produced NaN or Inf.
    #[error("NaN/Inf in surface evaluation at (u={u}, v={v})")]
    InvalidSurfacePoint {
        u: f64,
        v: f64,
    },

    /// A curve evaluation produced NaN or Inf.
    #[error("NaN/Inf in curve evaluation at t={t}")]
    InvalidCurvePoint {
        t: f64,
    },

    /// Surface normal is degenerate (zero length) at the given UV.
    #[error("Degenerate surface normal at (u={u}, v={v})")]
    DegenerateNormal {
        u: f64,
        v: f64,
    },

    /// A degenerate surface (e.g., zero radius cylinder/sphere).
    #[error("Degenerate surface: {message}")]
    DegenerateSurface {
        message: String,
    },

    /// A degenerate curve (e.g., zero length edge).
    #[error("Degenerate curve: {message}")]
    DegenerateCurve {
        message: String,
    },

    /// Point projection onto surface failed to converge.
    #[error("Surface projection failed to converge after {iterations} iterations")]
    ProjectionFailed {
        iterations: usize,
    },

    /// Parametric value out of range.
    #[error("Parametric value out of range: {parameter}={value}, range=[{min}, {max}]")]
    ParametricOutOfRange {
        parameter: String,
        value: f64,
        min: f64,
        max: f64,
    },

    /// General geometry error with context.
    #[error("{message}")]
    Other {
        message: String,
    },
}

/// Topology-related errors: invalid B-Rep structure, broken references.
#[derive(Error, Debug)]
pub enum TopologyError {
    /// An edge has no curve geometry.
    #[error("Edge {edge_id:?} has no curve geometry")]
    EdgeWithoutCurve {
        edge_id: u64,
    },

    /// A wire is not closed (loop not closed).
    #[error("Wire is not closed: {vertex_count} vertices, gap at index {gap_index}")]
    WireNotClosed {
        vertex_count: usize,
        gap_index: usize,
    },

    /// A face has no surface.
    #[error("Face has no surface geometry")]
    FaceWithoutSurface,

    /// Topological reference is broken (e.g., edge references non-existent vertex).
    #[error("Broken topological reference: {detail}")]
    BrokenReference {
        detail: String,
    },

    /// Non-manifold topology detected.
    #[error("Non-manifold topology: {detail}")]
    NonManifold {
        detail: String,
    },

    /// Invalid orientation.
    #[error("Invalid orientation: {detail}")]
    InvalidOrientation {
        detail: String,
    },

    /// General topology error with context.
    #[error("{message}")]
    Other {
        message: String,
    },
}

/// Mesh/triangulation-related errors.
#[derive(Error, Debug)]
pub enum MeshError {
    /// Triangulation timed out.
    #[error("Triangulation timed out after {seconds}s on face {face_id:?}")]
    TriangulationTimeout {
        face_id: Option<u64>,
        seconds: f64,
    },

    /// Degenerate triangle detected.
    #[error("Degenerate triangle at indices ({a}, {b}, {c}), area={area:.2e}")]
    DegenerateTriangle {
        a: u32,
        b: u32,
        c: u32,
        area: f64,
    },

    /// Merge coincident vertices failed.
    #[error("Vertex merge error: {detail}")]
    VertexMergeError {
        detail: String,
    },

    /// CDT triangulation failed.
    #[error("CDT triangulation failed: {detail}")]
    CdtFailed {
        detail: String,
    },

    /// Mesh is not watertight when expected to be.
    #[error("Mesh not watertight: {boundary_edges} boundary edges, Euler χ={euler_chi}")]
    NotWatertight {
        boundary_edges: usize,
        euler_chi: i64,
    },

    /// General mesh error with context.
    #[error("{message}")]
    Other {
        message: String,
    },
}

/// STEP file parsing/conversion errors.
#[derive(Error, Debug)]
pub enum StepError {
    /// STEP file parsing error.
    #[error("STEP parse error at line {line}: {message}")]
    ParseError {
        line: usize,
        message: String,
    },

    /// Unsupported STEP entity type.
    #[error("Unsupported STEP entity: {entity_type} (#{entity_id})")]
    UnsupportedEntity {
        entity_type: String,
        entity_id: i64,
    },

    /// Missing reference in STEP file.
    #[error("Missing STEP reference: #{entity_id} ({expected_type})")]
    MissingReference {
        entity_id: i64,
        expected_type: String,
    },

    /// Invalid geometry in STEP entity.
    #[error("Invalid geometry in STEP entity #{entity_id}: {detail}")]
    InvalidGeometry {
        entity_id: i64,
        detail: String,
    },

    /// Failed to extract surface from STEP entity.
    #[error("Failed to extract surface from #{entity_id} ({entity_type})")]
    SurfaceExtractionFailed {
        entity_id: i64,
        entity_type: String,
    },

    /// Failed to extract curve from STEP entity.
    #[error("Failed to extract curve from #{entity_id} ({entity_type})")]
    CurveExtractionFailed {
        entity_id: i64,
        entity_type: String,
    },

    /// Failed to extract boundary from STEP face.
    #[error("Failed to extract boundary from STEP face #{face_id}")]
    BoundaryExtractionFailed {
        face_id: i64,
    },

    /// No convertible geometry found in STEP file.
    #[error("No convertible geometry found in STEP file")]
    NoGeometry,

    /// General STEP error with context.
    #[error("{message}")]
    Other {
        message: String,
    },
}

/// I/O-related errors.
#[derive(Error, Debug)]
pub enum IoError {
    /// File not found.
    #[error("File not found: {path}")]
    FileNotFound {
        path: String,
    },

    /// File read error.
    #[error("Failed to read file '{path}': {source}")]
    FileReadError {
        path: String,
        #[source]
        source: std::io::Error,
    },

    /// File write error.
    #[error("Failed to write file '{path}': {source}")]
    FileWriteError {
        path: String,
        #[source]
        source: std::io::Error,
    },

    /// Invalid file format.
    #[error("Invalid file format: {detail}")]
    InvalidFormat {
        detail: String,
    },

    /// General I/O error with context.
    #[error("{message}")]
    Other {
        message: String,
    },
}

/// Convenience type alias for kernel results.
pub type KernelResult<T> = Result<T, KernelError>;

// ============================================================
// Conversion helpers from String-based errors
// ============================================================

impl From<String> for KernelError {
    fn from(s: String) -> Self {
        KernelError::Internal(s)
    }
}

impl From<&str> for KernelError {
    fn from(s: &str) -> Self {
        KernelError::Internal(s.to_string())
    }
}

impl From<std::io::Error> for KernelError {
    fn from(e: std::io::Error) -> Self {
        KernelError::Io(IoError::Other {
            message: e.to_string(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = GeometryError::InvalidSurfacePoint { u: 1.0, v: 2.0 };
        assert!(err.to_string().contains("u=1"));
        assert!(err.to_string().contains("v=2"));

        let err = StepError::UnsupportedEntity {
            entity_type: "BLOB".to_string(),
            entity_id: 42,
        };
        assert!(err.to_string().contains("BLOB"));
        assert!(err.to_string().contains("#42"));
    }

    #[test]
    fn test_error_conversion() {
        let geom_err = GeometryError::DegenerateNormal { u: 0.5, v: 0.5 };
        let kernel_err: KernelError = geom_err.into();
        assert!(matches!(kernel_err, KernelError::Geometry(_)));

        let step_err = StepError::NoGeometry;
        let kernel_err: KernelError = step_err.into();
        assert!(matches!(kernel_err, KernelError::Step(_)));
    }

    #[test]
    fn test_kernel_result() {
        fn fallible() -> KernelResult<i32> {
            Ok(42)
        }
        assert_eq!(fallible().unwrap(), 42);

        fn failing() -> KernelResult<i32> {
            Err(KernelError::Internal("test".to_string()))
        }
        assert!(failing().is_err());
    }
}
