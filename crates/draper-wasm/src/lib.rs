// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-wasm
//! JavaScript / WASM bindings for the 3Draper kernel.
//!
//! This crate exposes the kernel's full surface area (modeling, boolean,
//! GDT, STEP I/O, mesh export) to JavaScript via `wasm-bindgen`.
//!
//! ## Quick start
//!
//! ```js
//! import init, { Document, GdtType } from "./draper_wasm.js";
//! await init();
//! const doc = Document.new("my doc");
//! doc.addBox(100, 80, 60);
//! doc.filletEdge(0, 0, 5.0);          // fillet first manifold edge of solid 0
//! const mesh = doc.triangulate();
//! console.log(mesh.vertexCount, mesh.triangleCount);
//! const stl = mesh.exportStlBinary(); // Uint8Array
//! ```

#![allow(clippy::unused_unit)]

#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod tests;

use draper_core::{
    operations as ops,
    boolean::{boolean_union, boolean_subtract, boolean_intersect},
    step_to_usd::StepToUsdaParams,
    Document,
};
use draper_geometry::{Point3d, Direction3d, Surface, Curve3d};
use draper_mesh::{
    TriangleMesh, gdt_check::{GdtChecker, GdtCheckType, ToleranceSpec},
    export_usd::UsdMaterial,
    stl::{export_stl_binary, export_stl_ascii},
    export::{build_glb, build_3mf_bytes, build_obj},
};
use draper_step::{parse_step, extract_solids, export_step};
use draper_topology::{Solid, ShapeBuilder};
use wasm_bindgen::prelude::*;

// ============================================================
// Initialization
// ============================================================

/// Initialize the WASM module — installs panic hook and logger.
/// Must be called once before any other function.
#[wasm_bindgen(start)]
pub fn wasm_init() {
    #[cfg(target_arch = "wasm32")]
    {
        console_error_panic_hook::set_once();
        let _ = console_log::init_with_level(log::Level::Info);
    }
}

/// Library version string (e.g. "0.1.0").
#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

/// Check if a named feature is supported. Always returns true for known
/// features in this build.
#[wasm_bindgen]
pub fn has_feature(feature: &str) -> bool {
    matches!(
        feature,
        "step_import"
        | "step_export"
        | "stl_export"
        | "gltf_export"
        | "obj_export"
        | "3mf_export"
        | "usda_export"
        | "boolean_ops"
        | "healing"
        | "validation"
        | "analytical_queries"
        | "bvh"
        | "editing"
        | "gdt_checks"
        | "modeling"
        | "patterns"
    )
}

// ============================================================
// GDT type enum (mirrors draper_mesh::gdt_check::GdtCheckType)
// ============================================================

/// GDT tolerance type for `Document::gdtCheck`.
#[wasm_bindgen]
#[derive(Clone, Copy)]
pub enum GdtType {
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
}

fn gdt_type_from_js(t: GdtType) -> GdtCheckType {
    match t {
        GdtType::Flatness => GdtCheckType::Flatness,
        GdtType::Straightness => GdtCheckType::Straightness,
        GdtType::Circularity => GdtCheckType::Circularity,
        GdtType::Cylindricity => GdtCheckType::Cylindricity,
        GdtType::Position => GdtCheckType::Position,
        GdtType::Parallelism => GdtCheckType::Parallelism,
        GdtType::Perpendicularity => GdtCheckType::Perpendicularity,
        GdtType::Angularity => GdtCheckType::Angularity,
        GdtType::Runout => GdtCheckType::Runout,
        GdtType::ProfileOfLine => GdtCheckType::ProfileOfLine,
        GdtType::ProfileOfSurface => GdtCheckType::ProfileOfSurface,
    }
}

// ============================================================
// Mesh wrapper
// ============================================================

/// Wrapper around `TriangleMesh` exposed to JavaScript.
#[wasm_bindgen]
pub struct Mesh {
    inner: TriangleMesh,
}

#[wasm_bindgen]
impl Mesh {
    /// Number of vertices.
    #[wasm_bindgen(getter)]
    pub fn vertex_count(&self) -> usize {
        self.inner.vertex_count()
    }

    /// Number of triangles.
    #[wasm_bindgen(getter)]
    pub fn triangle_count(&self) -> usize {
        self.inner.triangle_count()
    }

    /// Vertex positions as a flat Float32Array: [x0,y0,z0, x1,y1,z1, ...].
    pub fn vertices(&self) -> js_sys::Float32Array {
        let v: Vec<f32> = self.inner.vertices.iter()
            .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        js_sys::Float32Array::from(&v[..])
    }

    /// Triangle indices as a Uint32Array: [i0,j0,k0, i1,j1,k1, ...].
    pub fn triangles(&self) -> js_sys::Uint32Array {
        let t: Vec<u32> = self.inner.triangles.iter()
            .flat_map(|t| [t[0], t[1], t[2]])
            .collect();
        js_sys::Uint32Array::from(&t[..])
    }

    /// Vertex normals (3 per vertex) or empty array if not present.
    pub fn normals(&self) -> js_sys::Float32Array {
        match &self.inner.normals {
            Some(n) => {
                let v: Vec<f32> = n.iter().flat_map(|v| [v[0] as f32, v[1] as f32, v[2] as f32]).collect();
                js_sys::Float32Array::from(&v[..])
            }
            None => js_sys::Float32Array::new(&JsValue::from(0)),
        }
    }

    /// Per-triangle RGBA colors (4 per triangle) or empty array.
    pub fn colors(&self) -> js_sys::Float32Array {
        match &self.inner.triangle_colors {
            Some(c) => {
                let v: Vec<f32> = c.iter().flat_map(|c| [c[0], c[1], c[2], c[3]]).collect();
                js_sys::Float32Array::from(&v[..])
            }
            None => js_sys::Float32Array::new(&JsValue::from(0)),
        }
    }

    /// Export the mesh to binary STL. Returns a Uint8Array.
    pub fn export_stl_binary(&self) -> js_sys::Uint8Array {
        let bytes = export_stl_binary(&self.inner, "3Draper");
        js_sys::Uint8Array::from(&bytes[..])
    }

    /// Export the mesh to ASCII STL. Returns a UTF-8 string.
    pub fn export_stl_ascii(&self) -> String {
        export_stl_ascii(&self.inner, "3Draper")
    }

    /// Export the mesh to binary glTF (GLB). Returns a Uint8Array.
    pub fn export_gltf(&self) -> Result<js_sys::Uint8Array, JsValue> {
        let bytes = build_glb(&self.inner).map_err(|e| JsValue::from_str(&format!("{}", e)))?;
        Ok(js_sys::Uint8Array::from(&bytes[..]))
    }

    /// Export the mesh to Wavefront OBJ. Returns a UTF-8 string.
    pub fn export_obj(&self) -> String {
        build_obj(&self.inner)
    }

    /// Export the mesh to 3MF. Returns a Uint8Array.
    pub fn export_3mf(&self) -> Result<js_sys::Uint8Array, JsValue> {
        let bytes = build_3mf_bytes(&self.inner).map_err(|e| JsValue::from_str(&format!("{}", e)))?;
        Ok(js_sys::Uint8Array::from(&bytes[..]))
    }
}

// ============================================================
// GDT check result
// ============================================================

/// Result of a GDT check.
#[wasm_bindgen]
pub struct GdtResult {
    tolerance_value: f64,
    actual_deviation: f64,
    passed: bool,
}

#[wasm_bindgen]
impl GdtResult {
    #[wasm_bindgen(getter)]
    pub fn tolerance_value(&self) -> f64 { self.tolerance_value }
    #[wasm_bindgen(getter)]
    pub fn actual_deviation(&self) -> f64 { self.actual_deviation }
    #[wasm_bindgen(getter)]
    pub fn passed(&self) -> bool { self.passed }
    #[wasm_bindgen(getter)]
    pub fn status(&self) -> String {
        if self.passed { "PASS".to_string() } else { "FAIL".to_string() }
    }
}

// ============================================================
// Document
// ============================================================

/// Top-level CAD document. Holds a list of solids.
#[wasm_bindgen]
pub struct DraperDocument {
    inner: Document,
    /// We also keep the most recently loaded solid's ID for editing ops.
    current_solid_idx: Option<usize>,
}

#[wasm_bindgen]
impl DraperDocument {
    /// Create a new empty document.
    #[wasm_bindgen(constructor)]
    pub fn new(name: Option<String>) -> DraperDocument {
        let n = name.unwrap_or_else(|| "Untitled".to_string());
        DraperDocument {
            inner: Document::new(&n),
            current_solid_idx: None,
        }
    }

    /// Number of solids.
    #[wasm_bindgen(getter)]
    pub fn solid_count(&self) -> usize {
        self.inner.solid_count()
    }

    // ----- Primitive builders -----

    /// Add a box (dx × dy × dz) — returns the new solid's index.
    pub fn add_box(&mut self, dx: f64, dy: f64, dz: f64) -> usize {
        let s = ShapeBuilder::make_box(dx, dy, dz);
        self.inner.add_solid(s);
        let idx = self.inner.solid_count() - 1;
        self.current_solid_idx = Some(idx);
        idx
    }

    /// Add a cylinder (radius, height) — returns the new solid's index.
    pub fn add_cylinder(&mut self, radius: f64, height: f64) -> usize {
        let s = ShapeBuilder::make_cylinder(radius, height);
        self.inner.add_solid(s);
        let idx = self.inner.solid_count() - 1;
        self.current_solid_idx = Some(idx);
        idx
    }

    /// Add a sphere (radius).
    pub fn add_sphere(&mut self, radius: f64) -> usize {
        let s = ShapeBuilder::make_sphere(radius);
        self.inner.add_solid(s);
        let idx = self.inner.solid_count() - 1;
        self.current_solid_idx = Some(idx);
        idx
    }

    /// Add a cone (radius, height, half-angle in radians).
    pub fn add_cone(&mut self, radius: f64, height: f64, half_angle: f64) -> usize {
        let s = ShapeBuilder::make_cone(radius, height, half_angle);
        self.inner.add_solid(s);
        let idx = self.inner.solid_count() - 1;
        self.current_solid_idx = Some(idx);
        idx
    }

    /// Add a torus (major_radius, minor_radius).
    pub fn add_torus(&mut self, major_r: f64, minor_r: f64) -> usize {
        let s = ShapeBuilder::make_torus(major_r, minor_r);
        self.inner.add_solid(s);
        let idx = self.inner.solid_count() - 1;
        self.current_solid_idx = Some(idx);
        idx
    }

    /// Load a STEP file from its text content. Appends all solids to the
    /// document. Returns the number of solids added.
    pub fn load_step(&mut self, content: &str) -> Result<usize, JsValue> {
        let step_file = parse_step(content).map_err(|e| JsValue::from_str(&format!("STEP parse error: {}", e)))?;
        let (solids, _ids) = extract_solids(&step_file);
        let n = solids.len();
        for s in solids {
            self.inner.add_solid(s);
        }
        if n > 0 {
            self.current_solid_idx = Some(self.inner.solid_count() - 1);
        }
        Ok(n)
    }

    // ----- Editing ops -----

    /// Fillet (round) an edge of a solid.
    /// `solid_index` — index of the solid.
    /// `edge_index` — TopoId of the edge (use 0 to auto-pick the first manifold edge).
    /// `radius` — fillet radius in mm.
    pub fn fillet_edge(&mut self, solid_index: usize, edge_index: usize, radius: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let s = &mut self.inner.root.solids[solid_index];
        let actual = if edge_index == 0 {
            find_first_manifold_edge(s)
        } else {
            edge_index
        };
        ops::fillet_edge(s, actual, radius).map_err(|e| JsValue::from_str(&e))
    }

    /// Chamfer (bevel) an edge of a solid.
    pub fn chamfer_edge(&mut self, solid_index: usize, edge_index: usize, distance: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let s = &mut self.inner.root.solids[solid_index];
        let actual = if edge_index == 0 {
            find_first_manifold_edge(s)
        } else {
            edge_index
        };
        ops::chamfer_edge(s, actual, distance).map_err(|e| JsValue::from_str(&e))
    }

    /// Shell a solid (inward offset by `thickness`).
    pub fn make_shell(&mut self, solid_index: usize, thickness: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let s = &mut self.inner.root.solids[solid_index];
        ops::make_shell(s, thickness).map_err(|e| JsValue::from_str(&e))
    }

    // ----- Transform ops -----

    /// Translate a solid by (dx, dy, dz).
    pub fn translate(&mut self, solid_index: usize, dx: f64, dy: f64, dz: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        ops::translate_solid(&mut self.inner.root.solids[solid_index], dx, dy, dz);
        Ok(())
    }

    /// Rotate a solid about (axis_x, axis_y, axis_z) by `angle_radians`.
    pub fn rotate(&mut self, solid_index: usize, ax: f64, ay: f64, az: f64, angle_radians: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let axis = Direction3d::new(ax, ay, az)
            .ok_or_else(|| JsValue::from_str("zero-length axis"))?;
        ops::rotate_solid(&mut self.inner.root.solids[solid_index], &axis, angle_radians);
        Ok(())
    }

    /// Rotate a solid about (axis_x, axis_y, axis_z) passing through
    /// (px, py, pz) by `angle_radians`.
    pub fn rotate_around_point(
        &mut self,
        solid_index: usize,
        ax: f64, ay: f64, az: f64,
        px: f64, py: f64, pz: f64,
        angle_radians: f64,
    ) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let axis = Direction3d::new(ax, ay, az)
            .ok_or_else(|| JsValue::from_str("zero-length axis"))?;
        let center = Point3d::new(px, py, pz);
        ops::rotate_solid_around_point(
            &mut self.inner.root.solids[solid_index],
            &axis,
            angle_radians,
            &center,
        );
        Ok(())
    }

    /// Uniformly scale a solid by `factor`.
    pub fn scale(&mut self, solid_index: usize, factor: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        if !factor.is_finite() || factor <= 0.0 {
            return Err(JsValue::from_str(&format!("invalid scale factor {}", factor)));
        }
        ops::scale_solid(&mut self.inner.root.solids[solid_index], factor);
        Ok(())
    }

    /// Uniformly scale a solid by `factor` about the point (cx, cy, cz).
    pub fn scale_around_point(
        &mut self,
        solid_index: usize,
        factor: f64,
        cx: f64, cy: f64, cz: f64,
    ) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        if !factor.is_finite() || factor <= 0.0 {
            return Err(JsValue::from_str(&format!("invalid scale factor {}", factor)));
        }
        let center = Point3d::new(cx, cy, cz);
        ops::scale_solid_around_point(
            &mut self.inner.root.solids[solid_index],
            factor,
            &center,
        );
        Ok(())
    }

    /// Mirror a solid about the plane through (ox,oy,oz) with normal (nx,ny,nz).
    /// Replaces the solid in-place with its mirror image.
    pub fn mirror(&mut self, solid_index: usize, ox: f64, oy: f64, oz: f64, nx: f64, ny: f64, nz: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let normal = Direction3d::new(nx, ny, nz)
            .ok_or_else(|| JsValue::from_str("zero-length normal"))?;
        let origin = Point3d::new(ox, oy, oz);
        let mirrored = ops::mirror_solid(&self.inner.root.solids[solid_index], origin, normal);
        self.inner.root.solids[solid_index] = mirrored;
        Ok(())
    }

    // ----- Patterns -----

    /// Create a circular pattern: `count` copies of the solid, evenly
    /// spaced around `axis` through the origin. The copies are appended
    /// to the document. Returns the number of copies added.
    pub fn circular_pattern(&mut self, solid_index: usize, count: usize, ax: f64, ay: f64, az: f64) -> Result<usize, JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        if count == 0 || count > 10000 {
            return Err(JsValue::from_str(&format!("count {} out of range", count)));
        }
        let axis = Direction3d::new(ax, ay, az)
            .ok_or_else(|| JsValue::from_str("zero-length axis"))?;
        let original = self.inner.root.solids[solid_index].clone();
        let copies = ops::circular_pattern(&original, axis, count, 2.0 * std::f64::consts::PI);
        let n = copies.len();
        for c in copies {
            self.inner.add_solid(c);
        }
        Ok(n)
    }

    /// Create a linear pattern: `count` copies of the solid, each translated
    /// by `step` along (dx, dy, dz). Copies are appended.
    pub fn linear_pattern(&mut self, solid_index: usize, count: usize, dx: f64, dy: f64, dz: f64, step: f64) -> Result<usize, JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        if count == 0 || count > 10000 {
            return Err(JsValue::from_str(&format!("count {} out of range", count)));
        }
        if !step.is_finite() || step <= 0.0 {
            return Err(JsValue::from_str(&format!("invalid step {}", step)));
        }
        let dir = Direction3d::new(dx, dy, dz)
            .ok_or_else(|| JsValue::from_str("zero-length direction"))?;
        let original = self.inner.root.solids[solid_index].clone();
        let copies = ops::linear_pattern(&original, dir, count, step);
        let n = copies.len();
        for c in copies {
            self.inner.add_solid(c);
        }
        Ok(n)
    }

    // ----- Boolean ops -----

    /// Boolean union of two solids. Returns the index of the new solid.
    pub fn boolean_union(&mut self, a_index: usize, b_index: usize) -> Result<usize, JsValue> {
        let (a, b) = self.get_two_solids(a_index, b_index)?;
        let result = boolean_union(&a, &b).map_err(|e| JsValue::from_str(&e))?;
        self.inner.add_solid(result);
        Ok(self.inner.solid_count() - 1)
    }

    /// Boolean subtract (A - B). Returns the index of the new solid.
    pub fn boolean_subtract(&mut self, a_index: usize, b_index: usize) -> Result<usize, JsValue> {
        let (a, b) = self.get_two_solids(a_index, b_index)?;
        let result = boolean_subtract(&a, &b).map_err(|e| JsValue::from_str(&e))?;
        self.inner.add_solid(result);
        Ok(self.inner.solid_count() - 1)
    }

    /// Boolean intersect (A ∩ B). Returns the index of the new solid.
    pub fn boolean_intersect(&mut self, a_index: usize, b_index: usize) -> Result<usize, JsValue> {
        let (a, b) = self.get_two_solids(a_index, b_index)?;
        let result = boolean_intersect(&a, &b).map_err(|e| JsValue::from_str(&e))?;
        self.inner.add_solid(result);
        Ok(self.inner.solid_count() - 1)
    }

    /// Delete a solid by index.
    pub fn delete_solid(&mut self, index: usize) -> Result<(), JsValue> {
        if index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("index {} out of range", index)));
        }
        self.inner.root.solids.remove(index);
        Ok(())
    }

    // ----- Holes -----

    /// Add a circular hole of `radius` mm centered at (cx, cy, cz) on the
    /// face at `face_index` of solid `solid_index`.
    pub fn add_circular_hole(&mut self, solid_index: usize, face_index: usize, cx: f64, cy: f64, cz: f64, radius: f64) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        if radius <= 0.0 || !radius.is_finite() {
            return Err(JsValue::from_str(&format!("invalid radius {}", radius)));
        }
        let center = Point3d::new(cx, cy, cz);
        let s = &mut self.inner.root.solids[solid_index];
        let face_normal = {
            let face = ops::get_face_mut(s, face_index)
                .ok_or_else(|| JsValue::from_str(&format!("face_index {} out of range", face_index)))?;
            let surface = face.surface.clone();
            match &surface {
                Some(Surface::Plane(p)) => p.normal,
                Some(Surface::Cylinder(c)) => c.axis.clone(),
                Some(Surface::Cone(c)) => c.axis.clone(),
                Some(Surface::Sphere(_)) => Direction3d::new(cx, cy, cz).unwrap_or(Direction3d::Z),
                Some(Surface::Torus(t)) => t.axis.clone(),
                _ => Direction3d::Z,
            }
        };
        let face = ops::get_face_mut(s, face_index).unwrap();
        ops::add_circular_hole_to_face(face, center, radius, face_normal).map_err(|e| JsValue::from_str(&e))
    }

    /// Remove a hole by index from a face of a solid.
    pub fn remove_hole(&mut self, solid_index: usize, face_index: usize, hole_index: usize) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let s = &mut self.inner.root.solids[solid_index];
        let face = ops::get_face_mut(s, face_index)
            .ok_or_else(|| JsValue::from_str(&format!("face_index {} out of range", face_index)))?;
        ops::remove_hole_from_face(face, hole_index).map_err(|e| JsValue::from_str(&e))?;
        Ok(())
    }

    /// Remove all holes from a face of a solid. Returns the number removed.
    pub fn clear_holes(&mut self, solid_index: usize, face_index: usize) -> Result<usize, JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let s = &mut self.inner.root.solids[solid_index];
        let face = ops::get_face_mut(s, face_index)
            .ok_or_else(|| JsValue::from_str(&format!("face_index {} out of range", face_index)))?;
        Ok(ops::clear_holes_from_face(face))
    }

    /// Delete a face from a solid by index.
    pub fn delete_face(&mut self, solid_index: usize, face_index: usize) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        ops::delete_face_from_solid(&mut self.inner.root.solids[solid_index], face_index)
            .map_err(|e| JsValue::from_str(&e))?;
        Ok(())
    }

    /// Reverse the orientation of all edges of a face (flips face normal).
    pub fn reverse_face(&mut self, solid_index: usize, face_index: usize) -> Result<(), JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let s = &mut self.inner.root.solids[solid_index];
        let face = ops::get_face_mut(s, face_index)
            .ok_or_else(|| JsValue::from_str(&format!("face_index {} out of range", face_index)))?;
        ops::reverse_face_orientation(face);
        Ok(())
    }

    // ----- GDT checks -----

    /// Run a single GDT check on the mesh of solid `solid_index`.
    /// `datum_axis` is an optional `[x,y,z]` JS array (pass null/undefined to skip).
    /// `nominal_position` is an optional `[x,y,z]` JS array.
    pub fn gdt_check(
        &self,
        solid_index: usize,
        check_type: GdtType,
        tolerance_value: f64,
        datum_axis: Option<js_sys::Array>,
        nominal_position: Option<js_sys::Array>,
        nominal_angle_deg: Option<f64>,
    ) -> Result<GdtResult, JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let mesh = self.inner.triangulate();
        let mut spec = ToleranceSpec::default();
        spec.tolerance_type = gdt_type_from_js(check_type);
        spec.tolerance_value = tolerance_value;
        if let Some(arr) = datum_axis {
            if arr.length() == 3 {
                let x = jsval_to_f64(&arr.get(0))?;
                let y = jsval_to_f64(&arr.get(1))?;
                let z = jsval_to_f64(&arr.get(2))?;
                if let Some(d) = Direction3d::new(x, y, z) {
                    spec.datum_axis = Some(d);
                }
            }
        }
        if let Some(arr) = nominal_position {
            if arr.length() == 3 {
                let x = jsval_to_f64(&arr.get(0))?;
                let y = jsval_to_f64(&arr.get(1))?;
                let z = jsval_to_f64(&arr.get(2))?;
                spec.nominal_position = Some(Point3d::new(x, y, z));
            }
        }
        if let Some(a) = nominal_angle_deg {
            spec.nominal_angle_deg = Some(a);
        }
        let checker = GdtChecker::new(&mesh);
        let r = checker.check(&spec);
        Ok(GdtResult {
            tolerance_value: r.tolerance_value,
            actual_deviation: r.actual_deviation,
            passed: r.passed,
        })
    }

    /// Run all GDT checks specified as a JSON array. Each entry is an object:
    /// `{ "type": "flatness", "value": 0.05, "datum_axis": [x,y,z], ... }`.
    /// Returns a JSON string with the results.
    pub fn gdt_check_all(&self, solid_index: usize, json_specs: &str) -> Result<String, JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let parsed: Vec<serde_json::Value> = serde_json::from_str(json_specs)
            .map_err(|e| JsValue::from_str(&format!("invalid JSON: {}", e)))?;
        let mesh = self.inner.triangulate();
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
        serde_json::to_string(&results).map_err(|e| JsValue::from_str(&format!("serialization error: {}", e)))
    }

    // ----- Edge listing -----

    /// List all edges in a solid as a JSON array.
    /// Each entry: `{ "id": N, "curve_type": "Line"|"Circle"|..., "face_ids": [a, b] }`.
    pub fn list_edges(&self, solid_index: usize) -> Result<String, JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let solid = &self.inner.root.solids[solid_index];
        use std::collections::HashMap;
        let mut edge_info: HashMap<u64, (String, Vec<usize>)> = HashMap::new();
        if let Some(shell) = solid.outer_shell.as_ref() {
            for (fi, face) in shell.faces.iter().enumerate() {
                for edge in &face.edges {
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
        serde_json::to_string(&arr).map_err(|e| JsValue::from_str(&format!("serialization error: {}", e)))
    }

    // ----- Mesh export -----

    /// Triangulate the whole document into a single mesh.
    pub fn triangulate(&self) -> Mesh {
        Mesh { inner: self.inner.triangulate() }
    }

    /// Compute the axis-aligned bounding box of all solids.
    /// Returns a 6-element Float64Array: [min_x, min_y, min_z, max_x, max_y, max_z].
    pub fn bounding_box(&self) -> Result<js_sys::Float64Array, JsValue> {
        let mesh = self.inner.triangulate();
        if mesh.vertices.is_empty() {
            return Err(JsValue::from_str("document has no vertices"));
        }
        let (mnx, mxx, mny, mxy, mnz, mxz) = mesh.vertices.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY),
            |(mnx, mxx, mny, mxy, mnz, mxz), v| (
                mnx.min(v.x), mxx.max(v.x),
                mny.min(v.y), mxy.max(v.y),
                mnz.min(v.z), mxz.max(v.z),
            ),
        );
        let arr = [mnx, mny, mnz, mxx, mxy, mxz];
        Ok(js_sys::Float64Array::from(&arr[..]))
    }

    /// Compute total volume of all solids in mm³.
    pub fn volume(&self) -> f64 {
        use draper_topology::solid_volume;
        self.inner.solids().iter()
            .map(|s| solid_volume(s))
            .sum()
    }

    /// Compute total surface area of all solids in mm².
    pub fn surface_area(&self) -> f64 {
        use draper_topology::solid_surface_area;
        self.inner.solids().iter()
            .map(|s| solid_surface_area(s))
            .sum()
    }

    // ----- STEP → USDA -----

    /// Export a single solid to STEP (AP214) text. Round-trips with load_step.
    pub fn export_step(&self, solid_index: usize, name: Option<String>) -> Result<String, JsValue> {
        if solid_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!("solid_index {} out of range", solid_index)));
        }
        let n = name.unwrap_or_else(|| format!("solid_{}", solid_index));
        Ok(export_step(&self.inner.root.solids[solid_index], &n))
    }

    /// Export the entire document (all solids) to a single STEP text.
    pub fn export_step_all(&self, name: Option<String>) -> String {
        let n = name.unwrap_or_else(|| "document".to_string());
        let mut combined = String::new();
        for (i, s) in self.inner.root.solids.iter().enumerate() {
            let nm = format!("{}_{}", n, i);
            combined.push_str(&export_step(s, &nm));
            combined.push('\n');
        }
        combined
    }

    /// Convert STEP text content to USDA (USD ASCII) text.
    /// Writes the triangulated meshes with the given `chord_tolerance`.
    /// Returns the USDA text.
    pub fn step_to_usda(content: &str, chord_tolerance: f64, smooth_normals: bool) -> Result<String, JsValue> {
        // Write STEP to a temporary in-memory path — but we don't have FS in wasm.
        // Instead, parse the content and call export_step_to_usda's logic inline.
        if chord_tolerance <= 0.0 || !chord_tolerance.is_finite() {
            return Err(JsValue::from_str(&format!("invalid chord_tolerance {}", chord_tolerance)));
        }
        let step_file = parse_step(content).map_err(|e| JsValue::from_str(&format!("STEP parse error: {}", e)))?;
        let (solids, brep_ids) = extract_solids(&step_file);
        if solids.is_empty() {
            return Err(JsValue::from_str("STEP file contains no solids"));
        }
        let params = StepToUsdaParams {
            chord_tolerance,
            parallel: false,  // single-threaded on wasm
            smooth_normals,
            material: UsdMaterial::default_grey(),
            include_camera: true,
            include_light: true,
        };
        // Use a memory buffer via a custom path that we can intercept.
        // Since draper_core::export_step_to_usda takes a Path, we can't use it on wasm.
        // Instead, reimplement the pipeline here using in-memory data.
        let options = draper_mesh::export_usd::UsdExportOptions::default();
        let mut exporter = draper_mesh::export_usd::UsdExporter::with_options(options);
        let mut tri_params = draper_mesh::TriangulationParams::default();
        tri_params.max_deviation = params.chord_tolerance;
        tri_params.parallel = false;
        let mut meshes_added = 0;
        for (i, solid) in solids.iter().enumerate() {
            let mesh = draper_mesh::triangulate_solid(solid, &tri_params);
            if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
                continue;
            }
            let name = format!("solid_{}_brep_{}", i, brep_ids.get(i).copied().unwrap_or(-1));
            exporter.add_mesh_with_material(&name, &mesh, &params.material, None);
            meshes_added += 1;
        }
        if meshes_added == 0 {
            return Err(JsValue::from_str("all solids produced empty meshes"));
        }
        Ok(exporter.to_usda_string())
    }

    // ----- Internal helpers -----

    fn get_two_solids(&self, a_index: usize, b_index: usize) -> Result<(Solid, Solid), JsValue> {
        if a_index >= self.inner.root.solids.len() || b_index >= self.inner.root.solids.len() {
            return Err(JsValue::from_str(&format!(
                "solid index out of range (have {} solids)",
                self.inner.solid_count()
            )));
        }
        Ok((
            self.inner.root.solids[a_index].clone(),
            self.inner.root.solids[b_index].clone(),
        ))
    }
}

// ============================================================
// Helpers
// ============================================================

fn find_first_manifold_edge(solid: &Solid) -> usize {
    use std::collections::HashMap;
    let mut edge_count: HashMap<u64, usize> = HashMap::new();
    if let Some(shell) = solid.outer_shell.as_ref() {
        for face in &shell.faces {
            for edge in &face.edges {
                *edge_count.entry(edge.id.to_u64()).or_insert(0) += 1;
            }
        }
    }
    for (id, count) in &edge_count {
        if *count == 2 {
            return *id as usize;
        }
    }
    1
}

fn jsval_to_f64(v: &JsValue) -> Result<f64, JsValue> {
    v.as_f64()
        .ok_or_else(|| JsValue::from_str("expected number"))
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
    Some(spec)
}
