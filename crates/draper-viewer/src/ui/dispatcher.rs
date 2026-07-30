// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Action dispatcher — connects UI actions to backend.
//!
//! This module is the bridge between the modular UI (menubar, dialogs, ribbon)
//! and the 3Draper backend (draper-step, draper-mesh, draper-topology).

use draper_geometry::Point3d;
use draper_topology::{ShapeBuilder, Solid, Shell, Face, Edge, Wire};
use draper_mesh::{triangulate_solid, TriangleMesh, TriangulationParams};
use draper_step::{parse_step, step_structure_lazy, StepConversionContext, export_step};
use crate::ui::menubar::MenuAction;
use crate::ui::dialogs::{DialogAction, PrimitiveType};
use crate::ui::core_engine::{SelectionManager, UndoManager, TextCommand};

/// The application document — holds the current model state.
pub struct Document {
    pub solids: Vec<Solid>,
    pub mesh: TriangleMesh,
    pub name: String,
    pub dirty: bool,
}

impl Default for Document {
    fn default() -> Self {
        // Start with a default box
        let solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
        let mesh = triangulate_solid(&solid, &TriangulationParams::default());
        Self {
            solids: vec![solid],
            mesh,
            name: "Untitled".to_string(),
            dirty: false,
        }
    }
}

/// Dispatch a MenuAction to the appropriate backend operation.
/// Returns a human-readable status message.
pub fn dispatch_menu_action(
    action: &MenuAction,
    doc: &mut Document,
    selection: &mut SelectionManager,
    undo: &mut UndoManager,
) -> String {
    match action {
        // ── File actions ──
        MenuAction::FileNew => {
            *doc = Document::default();
            undo.clear();
            "New document created".to_string()
        }
        MenuAction::FileOpen => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STEP", &["stp", "step"])
                    .pick_file()
                {
                    return import_step_file(doc, &path.to_string_lossy());
                }
            }
            "Open cancelled".to_string()
        }
        MenuAction::FileSave | MenuAction::FileSaveAs => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STEP", &["stp"])
                    .set_file_name(&doc.name)
                    .save_file()
                {
                    return export_step_file(doc, &path.to_string_lossy());
                }
            }
            "Save cancelled".to_string()
        }
        MenuAction::FileExportStep => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STEP", &["stp", "step"])
                    .save_file()
                {
                    return export_step_file(doc, &path.to_string_lossy());
                }
            }
            "Export cancelled".to_string()
        }
        MenuAction::FileExportStl => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STL", &["stl"])
                    .save_file()
                {
                    return export_stl_file(doc, &path.to_string_lossy());
                }
            }
            "Export cancelled".to_string()
        }
        MenuAction::FileImportStep => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STEP", &["stp", "step"])
                    .pick_file()
                {
                    return import_step_file(doc, &path.to_string_lossy());
                }
            }
            "Import cancelled".to_string()
        }
        MenuAction::FileQuit => {
            "Quit requested".to_string()
        }

        // ── Edit actions ──
        MenuAction::EditUndo => {
            undo.undo().unwrap_or_else(|| "Nothing to undo".to_string())
        }
        MenuAction::EditRedo => {
            undo.redo().unwrap_or_else(|| "Nothing to redo".to_string())
        }
        MenuAction::EditDuplicate => {
            if let Some(solid) = doc.solids.last().cloned() {
                doc.solids.push(solid);
                retriangulate(doc);
                undo.execute(Box::new(TextCommand::new("Duplicate solid")));
                "Solid duplicated".to_string()
            } else {
                "No solid to duplicate".to_string()
            }
        }
        _ => format!("Action {:?} not yet wired", action),
    }
}

/// Dispatch a DialogAction (from Insert Primitive dialog).
pub fn dispatch_dialog_action(
    action: &DialogAction,
    doc: &mut Document,
    undo: &mut UndoManager,
) -> String {
    match action {
        DialogAction::InsertPrimitive(pt, values) => {
            let solid = create_primitive(*pt, values);
            doc.solids.push(solid);
            retriangulate(doc);
            undo.execute(Box::new(TextCommand::new(&format!("Insert {}", pt.label()))));
            format!("{} inserted", pt.label())
        }
        DialogAction::Close => "Dialog closed".to_string(),
    }
}

/// Create a primitive solid from parameters.
fn create_primitive(pt: PrimitiveType, values: &[f64]) -> Solid {
    match pt {
        PrimitiveType::Box => {
            let w = values.get(0).copied().unwrap_or(100.0);
            let h = values.get(1).copied().unwrap_or(100.0);
            let d = values.get(2).copied().unwrap_or(100.0);
            ShapeBuilder::make_box(w, h, d)
        }
        PrimitiveType::Sphere => {
            let r = values.get(0).copied().unwrap_or(50.0);
            ShapeBuilder::make_sphere(r)
        }
        PrimitiveType::Cylinder => {
            let r = values.get(0).copied().unwrap_or(50.0);
            let h = values.get(1).copied().unwrap_or(100.0);
            ShapeBuilder::make_cylinder(r, h)
        }
        PrimitiveType::Cone => {
            let br = values.get(0).copied().unwrap_or(50.0);
            let tr = values.get(1).copied().unwrap_or(0.0);
            let h = values.get(2).copied().unwrap_or(100.0);
            ShapeBuilder::make_cone(br, tr, h)
        }
        PrimitiveType::Torus => {
            let mr = values.get(0).copied().unwrap_or(50.0);
            let nr = values.get(1).copied().unwrap_or(10.0);
            ShapeBuilder::make_torus(mr, nr)
        }
    }
}

/// Import a STEP file into the document.
fn import_step_file(doc: &mut Document, path: &str) -> String {
    let data = match std::fs::read_to_string(path) {
        Ok(d) => d,
        Err(e) => return format!("Failed to read file: {}", e),
    };
    let step = match parse_step(&data) {
        Ok(s) => s,
        Err(e) => return format!("Failed to parse STEP: {}", e),
    };
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    let mut total_mesh = TriangleMesh::new();
    let mut solid_count = 0;
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            total_mesh.merge(&inst.mesh);
            solid_count += 1;
        }
    }

    doc.mesh = total_mesh;
    doc.name = std::path::Path::new(path)
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "Imported".to_string());
    doc.dirty = false;
    // Clear solids (we have the mesh, but not the topology)
    doc.solids.clear();

    format!("Imported {} ({} solids, {} verts, {} tris)",
        doc.name, solid_count, doc.mesh.vertex_count(), doc.mesh.triangle_count())
}

/// Export the document as STEP.
fn export_step_file(doc: &mut Document, path: &str) -> String {
    if doc.solids.is_empty() {
        return "No solids to export".to_string();
    }
    let solid = doc.solids.first().unwrap();
    let step_content = export_step(solid, &doc.name);
    match std::fs::write(path, step_content) {
        Ok(_) => {
            doc.dirty = false;
            format!("Exported STEP: {}", path)
        }
        Err(e) => format!("Failed to write: {}", e),
    }
}

/// Export the document as STL (binary).
fn export_stl_file(doc: &mut Document, path: &str) -> String {
    use draper_mesh::stl;
    let data = stl::export_stl_binary(&doc.mesh, path);
    match std::fs::write(path, &data) {
        Ok(_) => format!("Exported STL: {}", path),
        Err(e) => format!("Failed to write STL: {}", e),
    }
}

/// Re-triangulate all solids in the document.
fn retriangulate(doc: &mut Document) {
    if doc.solids.is_empty() {
        return;
    }
    let mut combined = TriangleMesh::new();
    for solid in &doc.solids {
        let mesh = triangulate_solid(solid, &TriangulationParams::default());
        combined.merge(&mesh);
    }
    doc.mesh = combined;
    doc.dirty = true;
}

/// Boolean operations on the last two solids.
pub fn boolean_union(doc: &mut Document, undo: &mut UndoManager) -> String {
    if doc.solids.len() < 2 {
        return "Need at least 2 solids for boolean".to_string();
    }
    let b = doc.solids.pop().unwrap();
    let a = doc.solids.pop().unwrap();
    use draper_geometry::tolerance::ToleranceContext;
    let tol_ctx = ToleranceContext::from_model_scale(200.0);
    match draper_topology::boolean_union(&a, &b, &tol_ctx) {
        Ok(result) => {
            doc.solids.push(result);
            retriangulate(doc);
            undo.execute(Box::new(TextCommand::new("Boolean Union")));
            "Boolean union completed".to_string()
        }
        Err(e) => {
            doc.solids.push(a);
            doc.solids.push(b);
            format!("Boolean union failed: {:?}", e)
        }
    }
}

pub fn boolean_subtract(doc: &mut Document, undo: &mut UndoManager) -> String {
    if doc.solids.len() < 2 {
        return "Need at least 2 solids for boolean".to_string();
    }
    let b = doc.solids.pop().unwrap();
    let a = doc.solids.pop().unwrap();
    use draper_geometry::tolerance::ToleranceContext;
    let tol_ctx = ToleranceContext::from_model_scale(200.0);
    match draper_topology::boolean_subtract(&a, &b, &tol_ctx) {
        Ok(result) => {
            doc.solids.push(result);
            retriangulate(doc);
            undo.execute(Box::new(TextCommand::new("Boolean Subtract")));
            "Boolean subtract completed".to_string()
        }
        Err(e) => {
            doc.solids.push(a);
            doc.solids.push(b);
            format!("Boolean subtract failed: {:?}", e)
        }
    }
}

pub fn boolean_intersect(doc: &mut Document, undo: &mut UndoManager) -> String {
    if doc.solids.len() < 2 {
        return "Need at least 2 solids for boolean".to_string();
    }
    let b = doc.solids.pop().unwrap();
    let a = doc.solids.pop().unwrap();
    use draper_geometry::tolerance::ToleranceContext;
    let tol_ctx = ToleranceContext::from_model_scale(200.0);
    match draper_topology::boolean_intersect(&a, &b, &tol_ctx) {
        Ok(result) => {
            doc.solids.push(result);
            retriangulate(doc);
            undo.execute(Box::new(TextCommand::new("Boolean Intersect")));
            "Boolean intersect completed".to_string()
        }
        Err(e) => {
            doc.solids.push(a);
            doc.solids.push(b);
            format!("Boolean intersect failed: {:?}", e)
        }
    }
}
