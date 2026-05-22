//! # draper-ffi
//! C FFI bindings for the 3Draper kernel.
//!
//! Provides a C-compatible API for creating, manipulating, and exporting 3D models.

use draper_core::{
    Document, engine::{EngineConfig, build_engine},
};
use draper_mesh::{
    TriangleMesh, TriangulationParams,
    stl::{export_stl_ascii, export_stl_binary, write_stl_file},
};
use draper_step::exporter::export_step;
use draper_topology::{Solid, ShapeBuilder};
use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;

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

/// Create a new empty document.
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
#[no_mangle]
pub extern "C" fn draper_document_add_box(
    doc: *mut DraperDocument,
    dx: f64, dy: f64, dz: f64,
) -> i32 {
    let doc = unsafe { &mut *doc };
    let box_solid = ShapeBuilder::make_box(dx, dy, dz);
    doc.inner.add_solid(box_solid);
    0
}

/// Add a cylinder to the document.
#[no_mangle]
pub extern "C" fn draper_document_add_cylinder(
    doc: *mut DraperDocument,
    radius: f64, height: f64,
) -> i32 {
    let doc = unsafe { &mut *doc };
    let cyl = ShapeBuilder::make_cylinder(radius, height);
    doc.inner.add_solid(cyl);
    0
}

/// Add a sphere to the document.
#[no_mangle]
pub extern "C" fn draper_document_add_sphere(
    doc: *mut DraperDocument,
    radius: f64,
) -> i32 {
    let doc = unsafe { &mut *doc };
    let sphere = ShapeBuilder::make_sphere(radius);
    doc.inner.add_solid(sphere);
    0
}

/// Add a cone to the document.
#[no_mangle]
pub extern "C" fn draper_document_add_cone(
    doc: *mut DraperDocument,
    radius: f64, height: f64,
) -> i32 {
    let doc = unsafe { &mut *doc };
    let cone = ShapeBuilder::make_cone(radius, height, (radius / height).atan());
    doc.inner.add_solid(cone);
    0
}

/// Add a torus to the document.
#[no_mangle]
pub extern "C" fn draper_document_add_torus(
    doc: *mut DraperDocument,
    major_radius: f64, minor_radius: f64,
) -> i32 {
    let doc = unsafe { &mut *doc };
    let torus = ShapeBuilder::make_torus(major_radius, minor_radius);
    doc.inner.add_solid(torus);
    0
}

/// Build an ICE engine model.
#[no_mangle]
pub extern "C" fn draper_document_add_engine(doc: *mut DraperDocument) -> i32 {
    let doc = unsafe { &mut *doc };
    let config = EngineConfig::default();
    let engine_doc = build_engine(&config);

    // Merge engine solids into this document
    for solid in engine_doc.solids() {
        doc.inner.add_solid(solid.clone());
    }

    0
}

/// Triangulate the document and return a mesh.
#[no_mangle]
pub extern "C" fn draper_document_triangulate(doc: *mut DraperDocument) -> *mut DraperMesh {
    let doc = unsafe { &mut *doc };
    let mesh = doc.inner.triangulate();
    Box::into_raw(Box::new(DraperMesh { inner: mesh }))
}

/// Free a mesh.
#[no_mangle]
pub extern "C" fn draper_mesh_free(mesh: *mut DraperMesh) {
    if !mesh.is_null() {
        unsafe { drop(Box::from_raw(mesh)); }
    }
}

/// Get mesh vertex count.
#[no_mangle]
pub extern "C" fn draper_mesh_vertex_count(mesh: *const DraperMesh) -> u32 {
    if mesh.is_null() { return 0; }
    unsafe { (*mesh).inner.vertex_count() as u32 }
}

/// Get mesh triangle count.
#[no_mangle]
pub extern "C" fn draper_mesh_triangle_count(mesh: *const DraperMesh) -> u32 {
    if mesh.is_null() { return 0; }
    unsafe { (*mesh).inner.triangle_count() as u32 }
}

/// Get mesh vertex data (x, y, z triplets).
/// Caller must allocate buffer of size vertex_count * 3.
#[no_mangle]
pub extern "C" fn draper_mesh_get_vertices(
    mesh: *const DraperMesh,
    out: *mut f64,
    max_count: u32,
) -> u32 {
    if mesh.is_null() || out.is_null() { return 0; }
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
#[no_mangle]
pub extern "C" fn draper_mesh_get_triangles(
    mesh: *const DraperMesh,
    out: *mut u32,
    max_count: u32,
) -> u32 {
    if mesh.is_null() || out.is_null() { return 0; }
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
#[no_mangle]
pub extern "C" fn draper_mesh_export_stl(
    mesh: *const DraperMesh,
    path: *const c_char,
    binary: i32,
) -> i32 {
    if mesh.is_null() || path.is_null() { return -1; }
    let mesh_ref = unsafe { &(*mesh).inner };
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();

    let mut mesh_copy = mesh_ref.clone();
    mesh_copy.compute_face_normals();

    match write_stl_file(&mesh_copy, &path_str, binary != 0) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Export document to STEP file.
#[no_mangle]
pub extern "C" fn draper_document_export_step(
    doc: *mut DraperDocument,
    path: *const c_char,
) -> i32 {
    if doc.is_null() || path.is_null() { return -1; }
    let doc_ref = unsafe { &(*doc).inner };
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();

    // Export each solid
    if let Some(solid) = doc_ref.solids().first() {
        let step_content = export_step(solid, &doc_ref.name);
        match draper_step::exporter::write_step_file(&step_content, &path_str) {
            Ok(()) => 0,
            Err(_) => -1,
        }
    } else {
        -1
    }
}

/// Get the number of solids in the document.
#[no_mangle]
pub extern "C" fn draper_document_solid_count(doc: *const DraperDocument) -> u32 {
    if doc.is_null() { return 0; }
    unsafe { (*doc).inner.solid_count() as u32 }
}
