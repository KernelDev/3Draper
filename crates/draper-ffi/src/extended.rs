// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Extended FFI bindings — exposes the modeling, boolean, GDT and
//! STEP→USDA pipelines added after the initial FFI surface.
//!
//! All functions in this module follow the same conventions as
//! `lib.rs`: they take opaque `*mut DraperDocument` handles, return
//! `DraperResult` codes, and use `draper_get_last_error()` for
//! detailed error messages.

use super::{
    DraperDocument, DraperResult,
    set_last_error, store_error,
};
use draper_core::{
    KernelError, IoError,
    operations as ops,
    boolean::{boolean_union, boolean_subtract, boolean_intersect},
    step_to_usd::{export_step_to_usda, StepToUsdaParams},
};
use draper_geometry::{Point3d, Direction3d, Surface, Curve3d};
use draper_mesh::{
    gdt_check::{GdtChecker, GdtCheckType, ToleranceSpec, Plane},
    export_usd::UsdMaterial,
};
use draper_step::{parse_step_file, extract_solids};
use draper_topology::Solid;
use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::path::Path;

// ============================================================
// Transform operations
// ============================================================

/// Translate every solid in the document by (dx, dy, dz).
#[no_mangle]
pub extern "C" fn draper_document_translate(
    doc: *mut DraperDocument,
    dx: f64, dy: f64, dz: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_translate: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    for s in doc_ref.inner.root.solids.iter_mut() {
        ops::translate_solid(s, dx, dy, dz);
    }
    DraperResult::Success
}

/// Rotate every solid in the document about the given axis (unit vector)
/// through the origin by `angle` radians.
#[no_mangle]
pub extern "C" fn draper_document_rotate(
    doc: *mut DraperDocument,
    ax: f64, ay: f64, az: f64,
    angle: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_rotate: doc is null");
        return DraperResult::InvalidArgument;
    }
    let axis = match Direction3d::new(ax, ay, az) {
        Some(d) => d,
        None => {
            set_last_error("draper_document_rotate: axis is zero-length");
            return DraperResult::InvalidArgument;
        }
    };
    let doc_ref = unsafe { &mut *doc };
    for s in doc_ref.inner.root.solids.iter_mut() {
        ops::rotate_solid(s, &axis, angle);
    }
    DraperResult::Success
}

/// Rotate every solid in the document about an axis through `center`
/// (cx,cy,cz) with direction (ax,ay,az) by `angle` radians.
#[no_mangle]
pub extern "C" fn draper_document_rotate_around_point(
    doc: *mut DraperDocument,
    ax: f64, ay: f64, az: f64,
    cx: f64, cy: f64, cz: f64,
    angle: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_rotate_around_point: doc is null");
        return DraperResult::InvalidArgument;
    }
    let axis = match Direction3d::new(ax, ay, az) {
        Some(d) => d,
        None => {
            set_last_error("draper_document_rotate_around_point: axis is zero-length");
            return DraperResult::InvalidArgument;
        }
    };
    let center = Point3d::new(cx, cy, cz);
    let doc_ref = unsafe { &mut *doc };
    for s in doc_ref.inner.root.solids.iter_mut() {
        ops::rotate_solid_around_point(s, &axis, angle, &center);
    }
    DraperResult::Success
}

/// Uniformly scale every solid in the document by `factor` about the origin.
#[no_mangle]
pub extern "C" fn draper_document_scale(
    doc: *mut DraperDocument,
    factor: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_scale: doc is null");
        return DraperResult::InvalidArgument;
    }
    if !factor.is_finite() || factor <= 0.0 {
        set_last_error(&format!("draper_document_scale: invalid factor {}", factor));
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    for s in doc_ref.inner.root.solids.iter_mut() {
        ops::scale_solid(s, factor);
    }
    DraperResult::Success
}

/// Uniformly scale every solid in the document by `factor` about `center`.
#[no_mangle]
pub extern "C" fn draper_document_scale_around_point(
    doc: *mut DraperDocument,
    factor: f64,
    cx: f64, cy: f64, cz: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_scale_around_point: doc is null");
        return DraperResult::InvalidArgument;
    }
    if !factor.is_finite() || factor <= 0.0 {
        set_last_error(&format!("draper_document_scale_around_point: invalid factor {}", factor));
        return DraperResult::InvalidArgument;
    }
    let center = Point3d::new(cx, cy, cz);
    let doc_ref = unsafe { &mut *doc };
    for s in doc_ref.inner.root.solids.iter_mut() {
        ops::scale_solid_around_point(s, factor, &center);
    }
    DraperResult::Success
}

/// Mirror every solid in the document about the plane through
/// (ox,oy,oz) with normal (nx,ny,nz). The mirrored copies REPLACE the
/// originals.
#[no_mangle]
pub extern "C" fn draper_document_mirror(
    doc: *mut DraperDocument,
    ox: f64, oy: f64, oz: f64,
    nx: f64, ny: f64, nz: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_mirror: doc is null");
        return DraperResult::InvalidArgument;
    }
    let normal = match Direction3d::new(nx, ny, nz) {
        Some(d) => d,
        None => {
            set_last_error("draper_document_mirror: normal is zero-length");
            return DraperResult::InvalidArgument;
        }
    };
    let origin = Point3d::new(ox, oy, oz);
    let doc_ref = unsafe { &mut *doc };
    let mirrored: Vec<Solid> = doc_ref.inner.root.solids.iter()
        .map(|s| ops::mirror_solid(s, origin, normal))
        .collect();
    doc_ref.inner.root.solids = mirrored;
    DraperResult::Success
}

// ============================================================
// Fillet / Chamfer / Shell — solid-level editing ops
// ============================================================

/// Fillet (round) an edge of the solid at `solid_index`.
/// `edge_index` is the TopoId of the edge to fillet; `radius` is in mm.
#[no_mangle]
pub extern "C" fn draper_solid_fillet_edge(
    doc: *mut DraperDocument,
    solid_index: usize,
    edge_index: usize,
    radius: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_solid_fillet_edge: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error(&format!("solid_index {} out of range", solid_index));
        return DraperResult::InvalidArgument;
    }
    let s = &mut doc_ref.inner.root.solids[solid_index];
    match ops::fillet_edge(s, edge_index, radius) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            set_last_error(&e);
            DraperResult::TopologyError
        }
    }
}

/// Chamfer (bevel) an edge of the solid at `solid_index`.
/// `distance` is the linear distance from the original edge to the new face.
#[no_mangle]
pub extern "C" fn draper_solid_chamfer_edge(
    doc: *mut DraperDocument,
    solid_index: usize,
    edge_index: usize,
    distance: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_solid_chamfer_edge: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error(&format!("solid_index {} out of range", solid_index));
        return DraperResult::InvalidArgument;
    }
    let s = &mut doc_ref.inner.root.solids[solid_index];
    match ops::chamfer_edge(s, edge_index, distance) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            set_last_error(&e);
            DraperResult::TopologyError
        }
    }
}

/// Shell the solid at `solid_index` — creates an inward offset by
/// `thickness` mm.
#[no_mangle]
pub extern "C" fn draper_solid_make_shell(
    doc: *mut DraperDocument,
    solid_index: usize,
    thickness: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_solid_make_shell: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error(&format!("solid_index {} out of range", solid_index));
        return DraperResult::InvalidArgument;
    }
    let s = &mut doc_ref.inner.root.solids[solid_index];
    match ops::make_shell(s, thickness) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            set_last_error(&e);
            DraperResult::TopologyError
        }
    }
}

// ============================================================
// Boolean operations
// ============================================================

/// Boolean union of solids `a_index` and `b_index`. The result is APPENDED
/// to the document as a new solid; originals are NOT removed.
/// Returns the index of the new solid via `out_index`.
#[no_mangle]
pub extern "C" fn draper_document_boolean_union(
    doc: *mut DraperDocument,
    a_index: usize,
    b_index: usize,
    out_index: *mut u32,
) -> DraperResult {
    boolean_op(doc, a_index, b_index, out_index, "union", boolean_union)
}

#[no_mangle]
pub extern "C" fn draper_document_boolean_subtract(
    doc: *mut DraperDocument,
    a_index: usize,
    b_index: usize,
    out_index: *mut u32,
) -> DraperResult {
    boolean_op(doc, a_index, b_index, out_index, "subtract", boolean_subtract)
}

#[no_mangle]
pub extern "C" fn draper_document_boolean_intersect(
    doc: *mut DraperDocument,
    a_index: usize,
    b_index: usize,
    out_index: *mut u32,
) -> DraperResult {
    boolean_op(doc, a_index, b_index, out_index, "intersect", boolean_intersect)
}

fn boolean_op<F>(
    doc: *mut DraperDocument,
    a_index: usize,
    b_index: usize,
    out_index: *mut u32,
    name: &str,
    f: F,
) -> DraperResult
where
    F: FnOnce(&Solid, &Solid) -> Result<Solid, String>,
{
    if doc.is_null() {
        set_last_error(&format!("draper_document_boolean_{}: doc is null", name));
        return DraperResult::InvalidArgument;
    }
    if out_index.is_null() {
        set_last_error(&format!("draper_document_boolean_{}: out_index is null", name));
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if a_index >= doc_ref.inner.root.solids.len()
        || b_index >= doc_ref.inner.root.solids.len()
    {
        set_last_error(&format!(
            "boolean {}: index out of range (have {} solids)",
            name, doc_ref.inner.root.solids.len()
        ));
        return DraperResult::InvalidArgument;
    }
    let a = doc_ref.inner.root.solids[a_index].clone();
    let b = doc_ref.inner.root.solids[b_index].clone();
    match f(&a, &b) {
        Ok(result) => {
            doc_ref.inner.root.solids.push(result);
            let new_idx = (doc_ref.inner.root.solids.len() - 1) as u32;
            unsafe { *out_index = new_idx; }
            DraperResult::Success
        }
        Err(e) => {
            set_last_error(&e);
            unsafe { *out_index = u32::MAX; }
            DraperResult::TopologyError
        }
    }
}

/// Delete the solid at `index` from the document.
#[no_mangle]
pub extern "C" fn draper_document_delete_solid(
    doc: *mut DraperDocument,
    index: usize,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_delete_solid: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if index >= doc_ref.inner.root.solids.len() {
        set_last_error(&format!("delete_solid: index {} out of range", index));
        return DraperResult::InvalidArgument;
    }
    doc_ref.inner.root.solids.remove(index);
    DraperResult::Success
}

// ============================================================
// GDT checks
// ============================================================

/// GDT tolerance type codes (must match draper_mesh::gdt_check::GdtCheckType).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DraperGdtType {
    Flatness = 0,
    Straightness = 1,
    Circularity = 2,
    Cylindricity = 3,
    Position = 4,
    Parallelism = 5,
    Perpendicularity = 6,
    Angularity = 7,
    Runout = 8,
    ProfileOfLine = 9,
    ProfileOfSurface = 10,
    Unsupported = 99,
}

fn gdt_type_to_spec(t: DraperGdtType) -> GdtCheckType {
    match t {
        DraperGdtType::Flatness => GdtCheckType::Flatness,
        DraperGdtType::Straightness => GdtCheckType::Straightness,
        DraperGdtType::Circularity => GdtCheckType::Circularity,
        DraperGdtType::Cylindricity => GdtCheckType::Cylindricity,
        DraperGdtType::Position => GdtCheckType::Position,
        DraperGdtType::Parallelism => GdtCheckType::Parallelism,
        DraperGdtType::Perpendicularity => GdtCheckType::Perpendicularity,
        DraperGdtType::Angularity => GdtCheckType::Angularity,
        DraperGdtType::Runout => GdtCheckType::Runout,
        DraperGdtType::ProfileOfLine => GdtCheckType::ProfileOfLine,
        DraperGdtType::ProfileOfSurface => GdtCheckType::ProfileOfSurface,
        DraperGdtType::Unsupported => GdtCheckType::Unsupported("ffi".to_string()),
    }
}

/// Result of a GDT check (mirrors `GdtCheckResult`).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DraperGdtResult {
    pub tolerance_value: f64,
    pub actual_deviation: f64,
    pub passed: u8,
    pub status_code: u8,
}

/// Run a single GDT check on the triangulated mesh of `solid_index`.
#[no_mangle]
pub extern "C" fn draper_solid_gdt_check(
    doc: *const DraperDocument,
    solid_index: usize,
    check_type: DraperGdtType,
    tolerance_value: f64,
    datum_axis_x: f64, datum_axis_y: f64, datum_axis_z: f64,
    use_datum_axis: u8,
    nominal_pos_x: f64, nominal_pos_y: f64, nominal_pos_z: f64,
    use_nominal_pos: u8,
    nominal_angle_deg: f64,
    use_nominal_angle: u8,
) -> DraperGdtResult {
    if doc.is_null() {
        set_last_error("draper_solid_gdt_check: doc is null");
        return DraperGdtResult {
            tolerance_value, actual_deviation: f64::NAN,
            passed: 0, status_code: 2,
        };
    }
    let doc_ref = unsafe { &(*doc).inner };
    let solids: Vec<&Solid> = doc_ref.solids();
    if solid_index >= solids.len() {
        set_last_error(&format!("gdt_check: solid_index {} out of range", solid_index));
        return DraperGdtResult {
            tolerance_value, actual_deviation: f64::NAN,
            passed: 0, status_code: 2,
        };
    }
    let mesh = doc_ref.triangulate();
    let mut spec = ToleranceSpec::default();
    spec.tolerance_type = gdt_type_to_spec(check_type);
    spec.tolerance_value = tolerance_value;
    if use_datum_axis != 0 {
        if let Some(d) = Direction3d::new(datum_axis_x, datum_axis_y, datum_axis_z) {
            spec.datum_axis = Some(d);
        }
    }
    if use_nominal_pos != 0 {
        spec.nominal_position = Some(Point3d::new(nominal_pos_x, nominal_pos_y, nominal_pos_z));
    }
    if use_nominal_angle != 0 {
        spec.nominal_angle_deg = Some(nominal_angle_deg);
    }
    let checker = GdtChecker::new(&mesh);
    let result = checker.check(&spec);
    let status_code = if result.actual_deviation.is_nan() { 1 } else { 0 };
    DraperGdtResult {
        tolerance_value: result.tolerance_value,
        actual_deviation: result.actual_deviation,
        passed: if result.passed { 1 } else { 0 },
        status_code,
    }
}

/// Run all GDT checks passed as a JSON array. Returns results as JSON
/// string (caller MUST free with `draper_free_string`).
#[no_mangle]
pub extern "C" fn draper_solid_gdt_check_all(
    doc: *const DraperDocument,
    solid_index: usize,
    json_specs: *const c_char,
) -> *mut c_char {
    if doc.is_null() || json_specs.is_null() {
        set_last_error("draper_solid_gdt_check_all: null argument");
        return std::ptr::null_mut();
    }
    let doc_ref = unsafe { &(*doc).inner };
    let solids: Vec<&Solid> = doc_ref.solids();
    if solid_index >= solids.len() {
        set_last_error(&format!("gdt_check_all: solid_index {} out of range", solid_index));
        return std::ptr::null_mut();
    }
    let json_str = unsafe { CStr::from_ptr(json_specs) }.to_string_lossy();
    let parsed: Vec<serde_json::Value> = match serde_json::from_str(&json_str) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(&format!("gdt_check_all: invalid JSON: {}", e));
            return std::ptr::null_mut();
        }
    };
    let mesh = doc_ref.triangulate();
    let checker = GdtChecker::new(&mesh);
    let mut results = Vec::with_capacity(parsed.len());
    for v in &parsed {
        let spec = match json_value_to_spec(v) {
            Some(s) => s,
            None => continue,
        };
        let r = checker.check(&spec);
        results.push(serde_json::json!({
            "name": r.tolerance_name,
            "description": r.description,
            "type": format!("{:?}", r.tolerance_type),
            "tolerance_value": r.tolerance_value,
            "actual_deviation": r.actual_deviation,
            "passed": r.passed,
            "step_id": r.step_id,
        }));
    }
    let out = serde_json::to_string(&results).unwrap_or_else(|_| "[]".to_string());
    CString::new(out).map(|c| c.into_raw()).unwrap_or(std::ptr::null_mut())
}

/// Free a string previously returned by any draper_* function.
#[no_mangle]
pub extern "C" fn draper_free_string(s: *mut c_char) {
    if !s.is_null() {
        unsafe { drop(CString::from_raw(s)) };
    }
}

fn json_value_to_spec(v: &serde_json::Value) -> Option<ToleranceSpec> {
    let obj = v.as_object()?;
    let mut spec = ToleranceSpec::default();
    if let Some(t) = obj.get("type").and_then(|x| x.as_str()) {
        spec.tolerance_type = match t.to_lowercase().as_str() {
            "flatness" => GdtCheckType::Flatness,
            "straightness" => GdtCheckType::Straightness,
            "circularity" | "roundness" => GdtCheckType::Circularity,
            "cylindricity" => GdtCheckType::Cylindricity,
            "position" => GdtCheckType::Position,
            "parallelism" => GdtCheckType::Parallelism,
            "perpendicularity" => GdtCheckType::Perpendicularity,
            "angularity" => GdtCheckType::Angularity,
            "runout" => GdtCheckType::Runout,
            "profile_of_line" | "profileofline" => GdtCheckType::ProfileOfLine,
            "profile_of_surface" | "profileofsurface" => GdtCheckType::ProfileOfSurface,
            other => GdtCheckType::Unsupported(other.to_string()),
        };
    }
    if let Some(val) = obj.get("value").and_then(|x| x.as_f64()) {
        spec.tolerance_value = val;
    }
    if let Some(name) = obj.get("name").and_then(|x| x.as_str()) {
        spec.name = name.to_string();
    }
    if let Some(desc) = obj.get("description").and_then(|x| x.as_str()) {
        spec.description = desc.to_string();
    }
    if let Some(id) = obj.get("step_id").and_then(|x| x.as_i64()) {
        spec.step_id = id;
    }
    if let Some(arr) = obj.get("datum_axis").and_then(|x| x.as_array()) {
        if arr.len() == 3 {
            if let (Some(x), Some(y), Some(z)) = (
                arr[0].as_f64(), arr[1].as_f64(), arr[2].as_f64()
            ) {
                if let Some(d) = Direction3d::new(x, y, z) {
                    spec.datum_axis = Some(d);
                }
            }
        }
    }
    if let Some(arr) = obj.get("datum_plane_normal").and_then(|x| x.as_array()) {
        if arr.len() == 3 {
            if let (Some(x), Some(y), Some(z)) = (
                arr[0].as_f64(), arr[1].as_f64(), arr[2].as_f64()
            ) {
                if let Some(d) = Direction3d::new(x, y, z) {
                    spec.datum_plane_normal = Some(d);
                }
            }
        }
    }
    if let Some(arr) = obj.get("nominal_position").and_then(|x| x.as_array()) {
        if arr.len() == 3 {
            if let (Some(x), Some(y), Some(z)) = (
                arr[0].as_f64(), arr[1].as_f64(), arr[2].as_f64()
            ) {
                spec.nominal_position = Some(Point3d::new(x, y, z));
            }
        }
    }
    if let Some(a) = obj.get("nominal_angle_deg").and_then(|x| x.as_f64()) {
        spec.nominal_angle_deg = Some(a);
    }
    if let (Some(arr), Some(o)) = (
        obj.get("nominal_plane").and_then(|x| x.as_array()),
        obj.get("nominal_plane_origin").and_then(|x| x.as_array()),
    ) {
        if arr.len() == 3 && o.len() == 3 {
            if let (Some(nx), Some(ny), Some(nz), Some(ox), Some(oy), Some(oz)) = (
                arr[0].as_f64(), arr[1].as_f64(), arr[2].as_f64(),
                o[0].as_f64(), o[1].as_f64(), o[2].as_f64(),
            ) {
                if let Some(n) = Direction3d::new(nx, ny, nz) {
                    spec.nominal_surface = Some(Plane {
                        origin: Point3d::new(ox, oy, oz),
                        normal: n,
                    });
                }
            }
        }
    }
    Some(spec)
}

// ============================================================
// STEP → USDA export
// ============================================================

/// Convert a STEP file to a USDA (USD ASCII) file.
#[no_mangle]
pub extern "C" fn draper_export_step_to_usda(
    step_path: *const c_char,
    output_path: *const c_char,
    chord_tolerance: f64,
    smooth_normals: u8,
    include_camera: u8,
    include_light: u8,
) -> DraperResult {
    if step_path.is_null() || output_path.is_null() {
        set_last_error("draper_export_step_to_usda: null path argument");
        return DraperResult::InvalidArgument;
    }
    let step_str = unsafe { CStr::from_ptr(step_path) }.to_string_lossy().into_owned();
    let out_str = unsafe { CStr::from_ptr(output_path) }.to_string_lossy().into_owned();
    if chord_tolerance <= 0.0 || !chord_tolerance.is_finite() {
        set_last_error(&format!("draper_export_step_to_usda: invalid chord_tolerance {}", chord_tolerance));
        return DraperResult::InvalidArgument;
    }
    let params = StepToUsdaParams {
        chord_tolerance,
        parallel: true,
        smooth_normals: smooth_normals != 0,
        material: UsdMaterial::default_grey(),
        include_camera: include_camera != 0,
        include_light: include_light != 0,
    };
    match export_step_to_usda(
        Path::new(&step_str),
        Path::new(&out_str),
        &params,
    ) {
        Ok(_) => DraperResult::Success,
        Err(e) => {
            let err = KernelError::Io(IoError::FileWriteError {
                path: out_str,
                source: std::io::Error::new(std::io::ErrorKind::Other, e),
            });
            store_error(err)
        }
    }
}

// ============================================================
// STEP → Solid extraction (for programmatic access)
// ============================================================

/// Load a STEP file and append all its solids to the document.
#[no_mangle]
pub extern "C" fn draper_document_load_step(
    doc: *mut DraperDocument,
    path: *const c_char,
    _heal: u8,
) -> DraperResult {
    if doc.is_null() || path.is_null() {
        set_last_error("draper_document_load_step: null argument");
        return DraperResult::InvalidArgument;
    }
    let path_str = unsafe { CStr::from_ptr(path) }.to_string_lossy().into_owned();
    if !Path::new(&path_str).exists() {
        let err = KernelError::Io(IoError::FileNotFound { path: path_str.clone() });
        return store_error(err);
    }
    let step_file = match parse_step_file(&path_str) {
        Ok(sf) => sf,
        Err(e) => {
            set_last_error(&format!("STEP parse error: {}", e));
            return DraperResult::ParseError;
        }
    };
    let (solids, _brep_ids) = extract_solids(&step_file);
    if solids.is_empty() {
        set_last_error("STEP file contains no solids");
        return DraperResult::ParseError;
    }
    let doc_ref = unsafe { &mut *doc };
    for s in solids {
        doc_ref.inner.add_solid(s);
    }
    DraperResult::Success
}

// ============================================================
// Edge listing (for fillet/chamfer target discovery)
// ============================================================

/// Get a JSON array of all edges in the solid at `solid_index`.
/// Each entry: `{ "id": N, "curve_type": "Line"|"Circle"|..., "face_ids": [a, b] }`.
#[no_mangle]
pub extern "C" fn draper_solid_list_edges(
    doc: *const DraperDocument,
    solid_index: usize,
) -> *mut c_char {
    if doc.is_null() {
        set_last_error("draper_solid_list_edges: doc is null");
        return std::ptr::null_mut();
    }
    let doc_ref = unsafe { &(*doc).inner };
    let solids: Vec<&Solid> = doc_ref.solids();
    if solid_index >= solids.len() {
        set_last_error(&format!("list_edges: solid_index {} out of range", solid_index));
        return std::ptr::null_mut();
    }
    let solid = solids[solid_index];
    use std::collections::HashMap;
    // C5 Stage 5.3: store-first edge listing — canonical edges (a shared
    // edge appears once under one id) when the EdgeStore is populated;
    // per-face mirror walk otherwise (pre-C5 behavior).
    let mut edge_info: HashMap<u64, (String, Vec<usize>)> = HashMap::new();
    let store_ready = !solid.edge_store.is_empty();
    if store_ready {
        if let Some(shell) = solid.outer_shell.as_ref() {
            for (fi, face) in shell.faces.iter().enumerate() {
                for id in face.canonical_edge_ids() {
                    let curve_type = match solid
                        .edge_store
                        .get(id)
                        .and_then(|e| e.curve.as_ref())
                    {
                        None => "None".to_string(),
                        Some(Curve3d::Line(_)) => "Line".to_string(),
                        Some(Curve3d::Circle(_)) => "Circle".to_string(),
                        Some(Curve3d::Ellipse(_)) => "Ellipse".to_string(),
                        Some(Curve3d::Arc(_)) => "Arc".to_string(),
                        Some(Curve3d::Hyperbola(_)) => "Hyperbola".to_string(),
                        Some(Curve3d::Parabola(_)) => "Parabola".to_string(),
                        Some(Curve3d::Nurbs(_)) => "Nurbs".to_string(),
                        Some(Curve3d::PCurve { .. }) => "PCurve".to_string(),
                        Some(Curve3d::Trimmed { .. }) => "Trimmed".to_string(),
                        Some(Curve3d::Composite { .. }) => "Composite".to_string(),
                    };
                    edge_info
                        .entry(id.to_u64())
                        .and_modify(|(_, faces)| {
                            if !faces.contains(&fi) {
                                faces.push(fi);
                            }
                        })
                        .or_insert_with(|| (curve_type, vec![fi]));
                }
            }
        }
    } else if let Some(shell) = solid.outer_shell.as_ref() {
        for (fi, face) in shell.faces.iter().enumerate() {
            // C5 Stage 6.5: store-first boundary reads (per-id mirror
            // fallback keeps builder faces complete).
            for edge in solid.resolve_face_edges(face) {
                let id = edge.id.to_u64();
                let curve_type = match &edge.curve {
                    None => "None".to_string(),
                    Some(Curve3d::Line(_)) => "Line".to_string(),
                    Some(Curve3d::Circle(_)) => "Circle".to_string(),
                    Some(Curve3d::Ellipse(_)) => "Ellipse".to_string(),
                    Some(Curve3d::Arc(_)) => "Arc".to_string(),
                    Some(Curve3d::Hyperbola(_)) => "Hyperbola".to_string(),
                    Some(Curve3d::Parabola(_)) => "Parabola".to_string(),
                    Some(Curve3d::Nurbs(_)) => "Nurbs".to_string(),
                    Some(Curve3d::PCurve { .. }) => "PCurve".to_string(),
                    Some(Curve3d::Trimmed { .. }) => "Trimmed".to_string(),
                    Some(Curve3d::Composite { .. }) => "Composite".to_string(),
                };
                edge_info.entry(id)
                    .and_modify(|(_, faces)| faces.push(fi))
                    .or_insert((curve_type, vec![fi]));
            }
        }
    }
    let mut arr = Vec::with_capacity(edge_info.len());
    for (id, (curve_type, faces)) in edge_info {
        arr.push(serde_json::json!({
            "id": id,
            "curve_type": curve_type,
            "face_ids": faces,
        }));
    }
    let out = serde_json::to_string(&arr).unwrap_or_else(|_| "[]".to_string());
    CString::new(out).map(|c| c.into_raw()).unwrap_or(std::ptr::null_mut())
}

// ============================================================
// Pattern operations
// ============================================================

/// Create a circular pattern of the solid at `solid_index`.
/// `count` copies are placed evenly around `axis` through the origin,
/// spanning a full 360°. Original is kept; copies are appended.
#[no_mangle]
pub extern "C" fn draper_document_circular_pattern(
    doc: *mut DraperDocument,
    solid_index: usize,
    count: usize,
    ax: f64, ay: f64, az: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_circular_pattern: doc is null");
        return DraperResult::InvalidArgument;
    }
    if count == 0 || count > 10000 {
        set_last_error(&format!("circular_pattern: count {} out of range", count));
        return DraperResult::InvalidArgument;
    }
    let axis = match Direction3d::new(ax, ay, az) {
        Some(d) => d,
        None => {
            set_last_error("circular_pattern: axis is zero-length");
            return DraperResult::InvalidArgument;
        }
    };
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error("circular_pattern: solid_index out of range");
        return DraperResult::InvalidArgument;
    }
    let original = doc_ref.inner.root.solids[solid_index].clone();
    let copies = ops::circular_pattern(
        &original,
        axis,
        count,
        2.0 * std::f64::consts::PI,
    );
    for c in copies {
        doc_ref.inner.root.solids.push(c);
    }
    DraperResult::Success
}

/// Create a linear pattern: `count` copies of solid `solid_index`, each
/// translated by `step` along the unit vector (dx, dy, dz).
#[no_mangle]
pub extern "C" fn draper_document_linear_pattern(
    doc: *mut DraperDocument,
    solid_index: usize,
    count: usize,
    dx: f64, dy: f64, dz: f64,
    step: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_document_linear_pattern: doc is null");
        return DraperResult::InvalidArgument;
    }
    if count == 0 || count > 10000 {
        set_last_error(&format!("linear_pattern: count {} out of range", count));
        return DraperResult::InvalidArgument;
    }
    if !step.is_finite() || step <= 0.0 {
        set_last_error(&format!("linear_pattern: invalid step {}", step));
        return DraperResult::InvalidArgument;
    }
    let dir = match Direction3d::new(dx, dy, dz) {
        Some(d) => d,
        None => {
            set_last_error("linear_pattern: direction is zero-length");
            return DraperResult::InvalidArgument;
        }
    };
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error("linear_pattern: solid_index out of range");
        return DraperResult::InvalidArgument;
    }
    let original = doc_ref.inner.root.solids[solid_index].clone();
    let copies = ops::linear_pattern(&original, dir, count, step);
    for c in copies {
        doc_ref.inner.root.solids.push(c);
    }
    DraperResult::Success
}

// ============================================================
// Hole operations
// ============================================================

/// Add a circular hole of `radius` mm centered at (cx, cy, cz) on the
/// face at `face_index` of solid `solid_index`.
#[no_mangle]
pub extern "C" fn draper_solid_add_circular_hole(
    doc: *mut DraperDocument,
    solid_index: usize,
    face_index: usize,
    cx: f64, cy: f64, cz: f64,
    radius: f64,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_solid_add_circular_hole: doc is null");
        return DraperResult::InvalidArgument;
    }
    if radius <= 0.0 || !radius.is_finite() {
        set_last_error(&format!("add_circular_hole: invalid radius {}", radius));
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error("add_circular_hole: solid_index out of range");
        return DraperResult::InvalidArgument;
    }
    let center = Point3d::new(cx, cy, cz);
    let s = &mut doc_ref.inner.root.solids[solid_index];
    // Get the face, then determine its surface normal for the hole axis.
    let face_normal = {
        let face = match ops::get_face_mut(s, face_index) {
            Some(f) => f,
            None => {
                set_last_error(&format!("add_circular_hole: face_index {} out of range", face_index));
                return DraperResult::InvalidArgument;
            }
        };
        let surface = face.surface.clone();
        match &surface {
            Some(Surface::Plane(p)) => p.normal,
            Some(Surface::Cylinder(c)) => c.axis.clone(),
            Some(Surface::Cone(c)) => c.axis.clone(),
            Some(Surface::Sphere(_)) => {
                Direction3d::new(cx, cy, cz).unwrap_or(Direction3d::Z)
            }
            Some(Surface::Torus(t)) => t.axis.clone(),
            _ => Direction3d::Z,
        }
    };
    match ops::add_circular_hole_to_face(s, face_index, center, radius, face_normal) {
        Ok(()) => DraperResult::Success,
        Err(e) => {
            set_last_error(&e);
            DraperResult::TopologyError
        }
    }
}

/// Remove the i-th inner wire (hole) from a face. Returns Success even
/// if there were no holes (no-op).
#[no_mangle]
pub extern "C" fn draper_solid_remove_hole(
    doc: *mut DraperDocument,
    solid_index: usize,
    face_index: usize,
    hole_index: usize,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_solid_remove_hole: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error("remove_hole: solid_index out of range");
        return DraperResult::InvalidArgument;
    }
    let s = &mut doc_ref.inner.root.solids[solid_index];
    let face = match ops::get_face_mut(s, face_index) {
        Some(f) => f,
        None => {
            set_last_error(&format!("remove_hole: face_index {} out of range", face_index));
            return DraperResult::InvalidArgument;
        }
    };
    match ops::remove_hole_from_face(face, hole_index) {
        Ok(_) => DraperResult::Success,
        Err(e) => {
            set_last_error(&e);
            DraperResult::InvalidArgument
        }
    }
}

/// Clear all holes from a face. Returns the number of holes removed.
#[no_mangle]
pub extern "C" fn draper_solid_clear_holes(
    doc: *mut DraperDocument,
    solid_index: usize,
    face_index: usize,
) -> u32 {
    if doc.is_null() {
        return 0;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        return 0;
    }
    let s = &mut doc_ref.inner.root.solids[solid_index];
    let face = match ops::get_face_mut(s, face_index) {
        Some(f) => f,
        None => return 0,
    };
    ops::clear_holes_from_face(face) as u32
}

// ============================================================
// Face management
// ============================================================

/// Delete a face from a solid. **WARNING**: breaks watertightness.
/// Use only when replacing with another operation.
#[no_mangle]
pub extern "C" fn draper_solid_delete_face(
    doc: *mut DraperDocument,
    solid_index: usize,
    face_index: usize,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_solid_delete_face: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error("delete_face: solid_index out of range");
        return DraperResult::InvalidArgument;
    }
    let s = &mut doc_ref.inner.root.solids[solid_index];
    match ops::delete_face_from_solid(s, face_index) {
        Ok(_) => DraperResult::Success,
        Err(e) => {
            set_last_error(&e);
            DraperResult::TopologyError
        }
    }
}

/// Reverse the orientation of a face (swap forward flag).
#[no_mangle]
pub extern "C" fn draper_solid_reverse_face(
    doc: *mut DraperDocument,
    solid_index: usize,
    face_index: usize,
) -> DraperResult {
    if doc.is_null() {
        set_last_error("draper_solid_reverse_face: doc is null");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &mut *doc };
    if solid_index >= doc_ref.inner.root.solids.len() {
        set_last_error("reverse_face: solid_index out of range");
        return DraperResult::InvalidArgument;
    }
    let s = &mut doc_ref.inner.root.solids[solid_index];
    let face = match ops::get_face_mut(s, face_index) {
        Some(f) => f,
        None => {
            set_last_error(&format!("reverse_face: face_index {} out of range", face_index));
            return DraperResult::InvalidArgument;
        }
    };
    ops::reverse_face_orientation(face);
    DraperResult::Success
}

// ============================================================
// Mesh queries
// ============================================================

/// Compute the axis-aligned bounding box of the document's mesh.
/// Writes the 6 floats (min_x, min_y, min_z, max_x, max_y, max_z) to `out_bbox`
/// (must point to an array of at least 6 f64s).
#[no_mangle]
pub extern "C" fn draper_document_bbox(
    doc: *const DraperDocument,
    out_bbox: *mut f64,
) -> DraperResult {
    if doc.is_null() || out_bbox.is_null() {
        set_last_error("draper_document_bbox: null argument");
        return DraperResult::InvalidArgument;
    }
    let doc_ref = unsafe { &(*doc).inner };
    let mesh = doc_ref.triangulate();
    if mesh.vertices.is_empty() {
        set_last_error("bbox: document has no vertices");
        return DraperResult::GeometryError;
    }
    let (min_x, max_x, min_y, max_y, min_z, max_z) = mesh.vertices.iter().fold(
        (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY),
        |(mnx, mxx, mny, mxy, mnz, mxz), v| {
            (
                mnx.min(v.x), mxx.max(v.x),
                mny.min(v.y), mxy.max(v.y),
                mnz.min(v.z), mxz.max(v.z),
            )
        },
    );
    let out = unsafe { std::slice::from_raw_parts_mut(out_bbox, 6) };
    out[0] = min_x;
    out[1] = min_y;
    out[2] = min_z;
    out[3] = max_x;
    out[4] = max_y;
    out[5] = max_z;
    DraperResult::Success
}
