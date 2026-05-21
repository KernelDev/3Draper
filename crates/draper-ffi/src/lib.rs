//! # draper-ffi
//!
//! C FFI bindings for 3Draper.
//!
//! This crate exposes a C-compatible API so that the 3Draper kernel
//! can be used from any programming language (C, C++, Python, etc.).
//!
//! ## Usage from C:
//! ```c
//! #include "draper.h"
//!
//! DraperDocument* doc = draper_open_step("model.stp");
//! if (doc) {
//!     DraperMesh* mesh = draper_get_mesh(doc);
//!     // ... use mesh data ...
//!     draper_mesh_free(mesh);
//!     draper_document_free(doc);
//! }
//! ```

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;

use draper_core::document::Document;
use draper_core::Point3;
use draper_mesh::triangulate::TriangleMesh;

// Opaque handle types for FFI

/// Opaque handle to a Document.
pub struct DraperDocument {
    inner: Document,
}

/// Opaque handle to a TriangleMesh.
pub struct DraperMesh {
    inner: TriangleMesh,
}

/// 3D point for FFI.
#[repr(C)]
pub struct DraperVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<Point3> for DraperVec3 {
    fn from(p: Point3) -> Self {
        DraperVec3 { x: p.x, y: p.y, z: p.z }
    }
}

/// Statistics about a document.
#[repr(C)]
pub struct DraperStatistics {
    pub vertex_count: u64,
    pub edge_count: u64,
    pub face_count: u64,
    pub solid_count: u64,
    pub triangle_count: u64,
    pub mesh_vertex_count: u64,
}

// ---- Document functions ----

/// Create a new empty document.
#[no_mangle]
pub extern "C" fn draper_document_new() -> *mut DraperDocument {
    let doc = DraperDocument { inner: Document::new() };
    Box::into_raw(Box::new(doc))
}

/// Open a STEP file.
/// Returns null on failure.
#[no_mangle]
pub extern "C" fn draper_open_step(path: *const c_char) -> *mut DraperDocument {
    if path.is_null() {
        return ptr::null_mut();
    }

    let path_str = unsafe { CStr::from_ptr(path) };
    let path = match path_str.to_str() {
        Ok(s) => std::path::Path::new(s),
        Err(_) => return ptr::null_mut(),
    };

    match Document::open_step(path) {
        Ok(doc) => Box::into_raw(Box::new(DraperDocument { inner: doc })),
        Err(e) => {
            log::error!("Failed to open STEP file: {}", e);
            ptr::null_mut()
        }
    }
}

/// Save a document as a STEP file.
/// Returns 0 on success, -1 on failure.
#[no_mangle]
pub extern "C" fn draper_save_step(doc: *mut DraperDocument, path: *const c_char) -> i32 {
    if doc.is_null() || path.is_null() {
        return -1;
    }

    let doc = unsafe { &mut *doc };
    let path_str = unsafe { CStr::from_ptr(path) };

    match path_str.to_str() {
        Ok(s) => {
            let path = std::path::Path::new(s);
            match doc.inner.save_step(path) {
                Ok(()) => 0,
                Err(e) => {
                    log::error!("Failed to save STEP file: {}", e);
                    -1
                }
            }
        }
        Err(_) => -1,
    }
}

/// Get document statistics.
#[no_mangle]
pub extern "C" fn draper_get_statistics(doc: *const DraperDocument) -> DraperStatistics {
    if doc.is_null() {
        return DraperStatistics {
            vertex_count: 0,
            edge_count: 0,
            face_count: 0,
            solid_count: 0,
            triangle_count: 0,
            mesh_vertex_count: 0,
        };
    }

    let doc = unsafe { &*doc };
    let stats = doc.inner.statistics();

    DraperStatistics {
        vertex_count: stats.total_vertices as u64,
        edge_count: stats.total_edges as u64,
        face_count: stats.total_faces as u64,
        solid_count: stats.total_solids as u64,
        triangle_count: stats.total_triangles as u64,
        mesh_vertex_count: stats.total_mesh_vertices as u64,
    }
}

/// Free a document.
#[no_mangle]
pub extern "C" fn draper_document_free(doc: *mut DraperDocument) {
    if !doc.is_null() {
        unsafe { drop(Box::from_raw(doc)) };
    }
}

// ---- Mesh functions ----

/// Get the combined triangle mesh from the document.
/// The caller must free the mesh with draper_mesh_free().
#[no_mangle]
pub extern "C" fn draper_get_mesh(doc: *const DraperDocument) -> *mut DraperMesh {
    if doc.is_null() {
        return ptr::null_mut();
    }

    let doc = unsafe { &*doc };
    if let Some(mesh) = doc.inner.meshes.first() {
        let mesh_copy = DraperMesh { inner: mesh.clone() };
        Box::into_raw(Box::new(mesh_copy))
    } else {
        ptr::null_mut()
    }
}

/// Get the number of vertices in a mesh.
#[no_mangle]
pub extern "C" fn draper_mesh_vertex_count(mesh: *const DraperMesh) -> u64 {
    if mesh.is_null() {
        return 0;
    }
    let mesh = unsafe { &*mesh };
    mesh.inner.vertex_count() as u64
}

/// Get the number of triangles in a mesh.
#[no_mangle]
pub extern "C" fn draper_mesh_triangle_count(mesh: *const DraperMesh) -> u64 {
    if mesh.is_null() {
        return 0;
    }
    let mesh = unsafe { &*mesh };
    mesh.inner.triangle_count() as u64
}

/// Get a pointer to the vertex data.
/// Returns a pointer to an array of DraperVec3.
/// The pointer is valid as long as the mesh is alive.
#[no_mangle]
pub extern "C" fn draper_mesh_vertices(mesh: *const DraperMesh) -> *const DraperVec3 {
    if mesh.is_null() {
        return ptr::null();
    }
    let mesh = unsafe { &*mesh };
    // Safety: Point3 and DraperVec3 have the same layout
    mesh.inner.vertices.as_ptr() as *const DraperVec3
}

/// Get a pointer to the index data.
/// Returns a pointer to an array of u32 triangle indices.
/// The pointer is valid as long as the mesh is alive.
#[no_mangle]
pub extern "C" fn draper_mesh_indices(mesh: *const DraperMesh) -> *const u32 {
    if mesh.is_null() {
        return ptr::null();
    }
    let mesh = unsafe { &*mesh };
    mesh.inner.indices.as_ptr()
}

/// Get a pointer to the normal data (3 floats per vertex).
/// Returns null if normals are not computed.
#[no_mangle]
pub extern "C" fn draper_mesh_normals(mesh: *const DraperMesh) -> *const f32 {
    if mesh.is_null() {
        return ptr::null();
    }
    let mesh = unsafe { &*mesh };
    if mesh.inner.normals.is_empty() {
        ptr::null()
    } else {
        mesh.inner.normals.as_ptr() as *const f32
    }
}

/// Free a mesh.
#[no_mangle]
pub extern "C" fn draper_mesh_free(mesh: *mut DraperMesh) {
    if !mesh.is_null() {
        unsafe { drop(Box::from_raw(mesh)) };
    }
}

// ---- Version ----

/// Get the library version string.
/// The returned pointer is static and should not be freed.
#[no_mangle]
pub extern "C" fn draper_version() -> *const c_char {
    static VERSION: &[u8] = b"0.1.0\0";
    VERSION.as_ptr() as *const c_char
}
