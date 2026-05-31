// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-ffi
//! C FFI bindings for the 3Draper kernel.
//!
//! Provides a C-compatible API for creating, manipulating, and exporting 3D models.
//! Every function returns a [`DraperResult`] error code instead of panicking.
//! Use [`draper_get_last_error`] to retrieve the last detailed error message.
//!
//! This crate is only available on native targets (not wasm32).

// All FFI code is native-only — cdylib/staticlib/cbindgen don't work on wasm.
#![cfg(not(target_arch = "wasm32"))]

use draper_core::{
    Document, KernelError, engine::{EngineConfig, build_engine},
};
use draper_mesh::{
    TriangleMesh,
    stl::write_stl_file,
    export::{export_gltf, export_obj, export_3mf},
};
use draper_step::exporter::export_step;
use draper_topology::{Solid, ShapeBuilder, solid_volume, solid_surface_area, validate_topology, TopologyValidationConfig};
use std::cell::RefCell;
use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;

// ============================================================
// C-compatible error codes
// ============================================================

/// C-compatible result codes returned by every FFI function.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DraperResult {
    /// Operation succeeded.
    Success = 0,
    /// An invalid argument was passed (null pointer, out-of-range value, etc.).
    InvalidArgument = -1,
    /// The requested file was not found.
    FileNotFound = -2,
    /// Parsing of a file or data structure failed.
    ParseError = -3,
    /// A geometry evaluation failed (degenerate surface, NaN, etc.).
    GeometryError = -4,
    /// A topology error occurred (broken B-Rep, non-manifold, etc.).
    TopologyError = -5,
    /// Triangulation / mesh generation failed.
    TriangulationError = -6,
    /// Out of memory or resource limit exceeded.
    OutOfMemory = -7,
    /// An unclassified error occurred.
    UnknownError = -99,
}

impl From<&KernelError> for DraperResult {
    fn from(err: &KernelError) -> Self {
        match err {
            KernelError::Geometry(_) => DraperResult::GeometryError,
            KernelError::Topology(_) => DraperResult::TopologyError,
            KernelError::Mesh(_) => DraperResult::TriangulationError,
            KernelError::Step(draper_core::StepError::ParseError { .. }) => DraperResult::ParseError,
            KernelError::Step(_) => DraperResult::ParseError,
            KernelError::Io(draper_core::IoError::FileNotFound { .. }) => DraperResult::FileNotFound,
            KernelError::Io(_) => DraperResult::FileNotFound,
            KernelError::Internal(_) => DraperResult::UnknownError,
        }
    }
}

// ============================================================
// Thread-local last error storage
// ============================================================

thread_local! {
    /// Stores the last error message for the current thread.
    /// Accessed via `draper_get_last_error()`.
    static LAST_ERROR: RefCell<Option<CString>> = RefCell::new(None);
}

/// Store an error message in thread-local storage.
fn set_last_error(msg: &str) {
    let c_str = CString::new(msg).unwrap_or_else(|_| CString::new("<invalid error message>").unwrap());
    LAST_ERROR.with(|e| {
        *e.borrow_mut() = Some(c_str);
    });
}

/// Store a KernelError as the last error and return the corresponding DraperResult.
fn store_error(err: KernelError) -> DraperResult {
    let code: DraperResult = (&err).into();
    set_last_error(&err.to_string());
    code
}

/// Retrieve the last error message.
///
/// Returns a pointer to a C string describing the last error that occurred on
/// the current thread. The pointer is valid until the next FFI call on the
/// same thread. Returns NULL if no error has occurred.
#[no_mangle]
pub extern "C" fn draper_get_last_error() -> *const c_char {
    LAST_ERROR.with(|e| {
        match e.borrow().as_ref() {
            Some(c_str) => c_str.as_ptr(),
            None => ptr::null(),
        }
    })
}

// ============================================================
// Version & feature detection (5.3.1)
// ============================================================

/// Major version number of the 3Draper library.
///
/// Incremented on incompatible API changes.
#[no_mangle]
pub extern "C" fn draper_version_major() -> u32 {
    0
}

/// Minor version number of the 3Draper library.
///
/// Incremented on backwards-compatible additions.
#[no_mangle]
pub extern "C" fn draper_version_minor() -> u32 {
    1
}

/// Patch version number of the 3Draper library.
///
/// Incremented on backwards-compatible bug fixes.
#[no_mangle]
pub extern "C" fn draper_version_patch() -> u32 {
    0
}

/// Version string in "major.minor.patch" format.
///
/// The returned pointer is valid for the lifetime of the library and must NOT be freed.
#[no_mangle]
pub extern "C" fn draper_version_string() -> *const c_char {
    static VERSION: &[u8; 6] = b"0.1.0\0";
    VERSION.as_ptr() as *const c_char
}

/// Check whether the library supports a named feature.
///
/// Feature names are case-sensitive ASCII strings.  Currently recognised:
///
/// | Feature name        | Meaning                                      |
/// |---------------------|----------------------------------------------|
/// | `step_import`       | STEP AP242 file import                       |
/// | `step_export`       | STEP AP242 file export                       |
/// | `stl_export`        | STL (ASCII & binary) export                  |
/// | `gltf_export`       | glTF 2.0 / GLB export                        |
/// | `obj_export`        | Wavefront OBJ export                         |
/// | `3mf_export`        | 3MF (3D Manufacturing Format) export         |
/// | `boolean_ops`       | Boolean union / subtract / intersect         |
/// | `healing`           | Geometry healing pipeline                    |
/// | `validation`        | Topology & STEP validation                   |
/// | `analytical_queries`| Volume, surface area, center of mass, inertia|
/// | `bvh`               | BVH acceleration for ray / proximity queries |
/// | `wasm`              | WebAssembly target support                   |
#[no_mangle]
pub extern "C" fn draper_has_feature(feature: *const c_char) -> bool {
    if feature.is_null() {
        return false;
    }
    let feature_str = unsafe { CStr::from_ptr(feature) }.to_string_lossy();
    matches!(
        feature_str.as_ref(),
        "step_import"
        | "step_export"
        | "stl_export"
        | "gltf_export"
        | "obj_export"
        | "3mf_export"
        | "boolean_ops"
        | "healing"
        | "validation"
        | "analytical_queries"
        | "bvh"
    )
}

// ============================================================
// Opaque handles
// ============================================================

/// Opaque document handle.
pub struct DraperDocument {
    inner: Document,
}

/// Opaque solid handle.
pub struct DraperSolid {
    inner: Solid,
}

/// Opaque mesh handle.
pub struct DraperMesh {
    inner: TriangleMesh,
}

// ============================================================
// Document functions
// ============================================================

/// Create a new empty document.
///
/// Returns a pointer to the new document, or NULL on error.
#[no_mangle]
pub extern "C" fn draper_document_new(name: *const c_char) -> *mut DraperDocument {
    let name_str = if name.is_null() {
        "Untitled".to_string()
    } else {
        unsafe { CStr::from_ptr(name) }.to_string_lossy().into_owned()
    };

    Box::into_raw(Box::new(DraperDocument {
        inner: Document::new(&name_str),
    }))
}

/// Free a document.
#[no_mangle]
pub extern "C" fn draper_document_free(doc: *mut DraperDocument) {
    if !doc.is_null() {
        unsafe { drop(Box::from_raw(doc)); }
    }
}

/// Add a box to the document.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_document_add_box(
    doc: *mut DraperDocument,
    dx: f64, dy: f64, dz: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_add_box: doc is null");
        return DraperResult::InvalidArgument;
    }
    if dx <= 0.0 || dy <= 0.0 || dz <= 0.0 {
        set_last_error(&format!(
            "draper_document_add_box: dimensions must be positive, got ({}, {}, {})", dx, dy, dz
        ));
        return DraperResult::InvalidArgument;
    }
    let doc = unsafe { &mut *doc };
    let box_solid = ShapeBuilder::make_box(dx, dy, dz);
    doc.inner.add_solid(box_solid);
    DraperResult::Success
}

/// Add a cylinder to the document.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_document_add_cylinder(
    doc: *mut DraperDocument,
    radius: f64, height: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_add_cylinder: doc is null");
        return DraperResult::InvalidArgument;
    }
    if radius <= 0.0 || height <= 0.0 {
        set_last_error(&format!(
            "draper_document_add_cylinder: radius and height must be positive, got ({}, {})", radius, height
        ));
        return DraperResult::InvalidArgument;
    }
    let doc = unsafe { &mut *doc };
    let cyl = ShapeBuilder::make_cylinder(radius, height);
    doc.inner.add_solid(cyl);
    DraperResult::Success
}

/// Add a sphere to the document.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_document_add_sphere(
    doc: *mut DraperDocument,
    radius: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_add_sphere: doc is null");
        return DraperResult::InvalidArgument;
    }
    if radius <= 0.0 {
        set_last_error(&format!(
            "draper_document_add_sphere: radius must be positive, got {}", radius
        ));
        return DraperResult::InvalidArgument;
    }
    let doc = unsafe { &mut *doc };
    let sphere = ShapeBuilder::make_sphere(radius);
    doc.inner.add_solid(sphere);
    DraperResult::Success
}

/// Add a cone to the document.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_document_add_cone(
    doc: *mut DraperDocument,
    radius: f64, height: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_add_cone: doc is null");
        return DraperResult::InvalidArgument;
    }
    if radius <= 0.0 || height <= 0.0 {
        set_last_error(&format!(
            "draper_document_add_cone: radius and height must be positive, got ({}, {})", radius, height
        ));
        return DraperResult::InvalidArgument;
    }
    let doc = unsafe { &mut *doc };
    let cone = ShapeBuilder::make_cone(radius, height, (radius / height).atan());
    doc.inner.add_solid(cone);
    DraperResult::Success
}

/// Add a torus to the document.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_document_add_torus(
    doc: *mut DraperDocument,
    major_radius: f64, minor_radius: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_add_torus: doc is null");
        return DraperResult::InvalidArgument;
    }
    if major_radius <= 0.0 || minor_radius <= 0.0 {
        set_last_error(&format!(
            "draper_document_add_torus: major_radius and minor_radius must be positive, got ({}, {})",
            major_radius, minor_radius
        ));
        return DraperResult::InvalidArgument;
    }
    let doc = unsafe { &mut *doc };
    let torus = ShapeBuilder::make_torus(major_radius, minor_radius);
    doc.inner.add_solid(torus);
    DraperResult::Success
}

/// Build an ICE engine model.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_document_add_engine(doc: *mut DraperDocument) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_add_engine: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc = unsafe { &mut *doc };
    let config = EngineConfig::default();
    let engine_doc = build_engine(&config);

    // Merge engine solids into this document
    for solid in engine_doc.solids() {
        doc.inner.add_solid(solid.clone());
    }

    DraperResult::Success
}

// ============================================================
// Triangulation
// ============================================================

/// Triangulate the document and return a mesh.
///
/// Returns a pointer to the new mesh on success, or NULL on error.
/// Use `draper_get_last_error()` for details on failure.
#[no_mangle]
pub extern "C" fn draper_document_triangulate(doc: *mut DraperDocument) -> *mut DraperMesh {
    if doc.is_null() {
        set_last_error("draper_document_triangulate: doc is null");
        return ptr::null_mut();
    }
    let doc = unsafe { &mut *doc };
    let mesh = doc.inner.triangulate();
    Box::into_raw(Box::new(DraperMesh { inner: mesh }))
}

// ============================================================
// Mesh functions
// ============================================================

/// Free a mesh.
#[no_mangle]
pub extern "C" fn draper_mesh_free(mesh: *mut DraperMesh) {
    if !mesh.is_null() {
        unsafe { drop(Box::from_raw(mesh)); }
    }
}

/// Get mesh vertex count.
///
/// Returns 0 if mesh is null.
#[no_mangle]
pub extern "C" fn draper_mesh_vertex_count(mesh: *const DraperMesh) -> u32 {
    if mesh.is_null() { return 0; }
    unsafe { (*mesh).inner.vertex_count() as u32 }
}

/// Get mesh triangle count.
///
/// Returns 0 if mesh is null.
#[no_mangle]
pub extern "C" fn draper_mesh_triangle_count(mesh: *const DraperMesh) -> u32 {
    if mesh.is_null() { return 0; }
    unsafe { (*mesh).inner.triangle_count() as u32 }
}

/// Get mesh vertex data (x, y, z triplets).
/// Caller must allocate buffer of size vertex_count * 3.
///
/// Returns the number of vertices written, or 0 on error.
#[no_mangle]
pub extern "C" fn draper_mesh_get_vertices(
    mesh: *const DraperMesh,
    out: *mut f64,
    max_count: u32,
) -> u32 {
    if mesh.is_null() {
        set_last_error("draper_mesh_get_vertices: mesh is null");
        return 0;
    }
    if out.is_null() {
        set_last_error("draper_mesh_get_vertices: out buffer is null");
        return 0;
    }
    let mesh_ref = unsafe { &(*mesh).inner };
    let count = mesh_ref.vertex_count().min(max_count as usize);
    let out_slice = unsafe { std::slice::from_raw_parts_mut(out, count * 3) };
    for (i, v) in mesh_ref.vertices.iter().take(count).enumerate() {
        out_slice[i * 3] = v.x;
        out_slice[i * 3 + 1] = v.y;
        out_slice[i * 3 + 2] = v.z;
    }
    count as u32
}

/// Get mesh triangle indices (i, j, k triplets).
/// Caller must allocate buffer of size triangle_count * 3.
///
/// Returns the number of triangles written, or 0 on error.
#[no_mangle]
pub extern "C" fn draper_mesh_get_triangles(
    mesh: *const DraperMesh,
    out: *mut u32,
    max_count: u32,
) -> u32 {
    if mesh.is_null() {
        set_last_error("draper_mesh_get_triangles: mesh is null");
        return 0;
    }
    if out.is_null() {
        set_last_error("draper_mesh_get_triangles: out buffer is null");
        return 0;
    }
    let mesh_ref = unsafe { &(*mesh).inner };
    let count = mesh_ref.triangle_count().min(max_count as usize);
    let out_slice = unsafe { std::slice::from_raw_parts_mut(out, count * 3) };
    for (i, tri) in mesh_ref.triangles.iter().take(count).enumerate() {
        out_slice[i * 3] = tri[0];
        out_slice[i * 3 + 1] = tri[1];
        out_slice[i * 3 + 2] = tri[2];
    }
    count as u32
}

/// Export mesh to STL file.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_mesh_export_stl(
    mesh: *const DraperMesh,
    path: *const c_char,
    binary: i32,
) -> DraperResult {
    if mesh.is_null() {
        set_last_error("draper_mesh_export_stl: mesh is null");
        return DraperResult::InvalidArgument;
    }
    if path.is_null() {
        set_last_error("draper_mesh_export_stl: path is null");
        return DraperResult::InvalidArgument;
    }
    let mesh_ref = unsafe { &(*mesh).inner };
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();

    let mut mesh_copy = mesh_ref.clone();
    mesh_copy.compute_face_normals();

    match write_stl_file(&mesh_copy, &path_str, binary != 0) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            let err = KernelError::from(e);
            store_error(err)
        }
    }
}

/// Export document to STEP file.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_document_export_step(
    doc: *mut DraperDocument,
    path: *const c_char,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_export_step: doc is null");
        return DraperResult::InvalidArgument;
    }
    if path.is_null() {
        set_last_error("draper_document_export_step: path is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();

    // Export each solid
    if let Some(solid) = doc_ref.solids().first() {
        let step_content = export_step(solid, &doc_ref.name);
        match draper_step::exporter::write_step_file(&step_content, &path_str) {
            Ok(()) => DraperResult::Success,
            Err(e) => {
                let err = KernelError::from(e);
                store_error(err)
            }
        }
    } else {
        set_last_error("draper_document_export_step: document has no solids to export");
        DraperResult::InvalidArgument
    }
}

/// Get the number of solids in the document.
///
/// Returns 0 if doc is null.
#[no_mangle]
pub extern "C" fn draper_document_solid_count(doc: *const DraperDocument) -> u32 {
    if doc.is_null() { return 0; }
    unsafe { (*doc).inner.solid_count() as u32 }
}

// ============================================================
// Analytical queries (5.3.1)
// ============================================================

/// Compute the total volume of all solids in the document.
///
/// Uses the divergence theorem on a triangulated approximation of each solid.
/// Returns 0.0 if `doc` is null or the document has no solids.
#[no_mangle]
pub extern "C" fn draper_solid_volume(doc: *const DraperDocument) -> f64 {
    if doc.is_null() {
        set_last_error("draper_solid_volume: doc is null");
        return 0.0;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let solids = doc_ref.solids();
    if solids.is_empty() {
        set_last_error("draper_solid_volume: document has no solids");
        return 0.0;
    }
    solids.iter().map(|s| solid_volume(s)).sum()
}

/// Compute the total surface area of all solids in the document.
///
/// Returns 0.0 if `doc` is null or the document has no solids.
#[no_mangle]
pub extern "C" fn draper_solid_surface_area(doc: *const DraperDocument) -> f64 {
    if doc.is_null() {
        set_last_error("draper_solid_surface_area: doc is null");
        return 0.0;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let solids = doc_ref.solids();
    if solids.is_empty() {
        set_last_error("draper_solid_surface_area: document has no solids");
        return 0.0;
    }
    solids.iter().map(|s| solid_surface_area(s)).sum()
}

// ============================================================
// Validation (5.3.1)
// ============================================================

/// Run topology validation on all solids in the document.
///
/// Returns `DraperResult::Success` if no errors were found,
/// `DraperResult::TopologyError` if any error-level issues exist.
/// Detailed issues are stored in the thread-local error message
/// (accessible via `draper_get_last_error()`).
#[no_mangle]
pub extern "C" fn draper_validate_step(doc: *const DraperDocument) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_validate_step: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let solids = doc_ref.solids();
    if solids.is_empty() {
        set_last_error("draper_validate_step: document has no solids to validate");
        return DraperResult::InvalidArgument;
    }

    let config = TopologyValidationConfig::default();
    let mut combined_report = String::new();
    let mut has_errors = false;

    for (i, solid) in solids.iter().enumerate() {
        let report = validate_topology(solid, &config);
        if report.has_errors() {
            has_errors = true;
        }
        if !report.is_clean() {
            combined_report.push_str(&format!(
                "--- Solid {} ---\n{}\n",
                i, report
            ));
        }
    }

    if has_errors {
        set_last_error(&combined_report);
        DraperResult::TopologyError
    } else if !combined_report.is_empty() {
        // Only warnings/info — still success but record the details
        set_last_error(&combined_report);
        DraperResult::Success
    } else {
        DraperResult::Success
    }
}

// ============================================================
// Export convenience functions (5.3.1)
// ============================================================

/// Triangulate the document and export to glTF 2.0 (GLB binary).
///
/// Convenience function that triangulates the document and writes
/// the result to a glTF file in a single call.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_export_gltf(
    doc: *const DraperDocument,
    path: *const c_char,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_export_gltf: doc is null");
        return DraperResult::InvalidArgument;
    }
    if path.is_null() {
        set_last_error("draper_export_gltf: path is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();

    let mesh = doc_ref.triangulate();
    match export_gltf(&mesh, &path_str) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            let err = KernelError::Io(draper_core::IoError::FileWriteError {
                path: path_str,
                source: std::io::Error::new(std::io::ErrorKind::Other, e.to_string()),
            });
            store_error(err)
        }
    }
}

/// Triangulate the document and export to Wavefront OBJ.
///
/// Convenience function that triangulates the document and writes
/// the result to an OBJ file in a single call.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_export_obj(
    doc: *const DraperDocument,
    path: *const c_char,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_export_obj: doc is null");
        return DraperResult::InvalidArgument;
    }
    if path.is_null() {
        set_last_error("draper_export_obj: path is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();

    let mesh = doc_ref.triangulate();
    match export_obj(&mesh, &path_str) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            let err = KernelError::Io(draper_core::IoError::FileWriteError {
                path: path_str,
                source: std::io::Error::new(std::io::ErrorKind::Other, e.to_string()),
            });
            store_error(err)
        }
    }
}

/// Triangulate the document and export to 3MF (3D Manufacturing Format).
///
/// Convenience function that triangulates the document and writes
/// the result to a 3MF file in a single call.
///
/// Returns `DraperResult::Success` on success, or an error code on failure.
#[no_mangle]
pub extern "C" fn draper_export_3mf(
    doc: *const DraperDocument,
    path: *const c_char,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_export_3mf: doc is null");
        return DraperResult::InvalidArgument;
    }
    if path.is_null() {
        set_last_error("draper_export_3mf: path is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();

    let mesh = doc_ref.triangulate();
    match export_3mf(&mesh, &path_str) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            let err = KernelError::Io(draper_core::IoError::FileWriteError {
                path: path_str,
                source: std::io::Error::new(std::io::ErrorKind::Other, e.to_string()),
            });
            store_error(err)
        }
    }
}
