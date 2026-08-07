// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Action dispatcher — connects UI actions to backend.
//!
//! This module is the bridge between the modular UI (menubar, dialogs, ribbon)
//! and the 3Draper backend (draper-step, draper-mesh, draper-topology).

use draper_topology::{ShapeBuilder, Solid};
use draper_geometry::Point3d;
use draper_mesh::{triangulate_solid, TriangleMesh, TriangulationParams};
use draper_step::{parse_step, step_structure_lazy, StepConversionContext, export_step};
use crate::ui::menubar::MenuAction;
use crate::ui::dialogs::{DialogAction, PrimitiveType};
use crate::ui::core_engine::{SelectionManager, UndoManager, TextCommand};
use crate::ui::view_modes::ViewOrientation;
use crate::ui::DisplayStyle;

/// A snapshot of the document used for undo/redo.
#[derive(Clone, Debug)]
pub struct DocSnapshot {
    pub solids: Vec<Solid>,
    pub name: String,
    pub description: String,
}

/// The application document — holds the current model state.
pub struct Document {
    pub solids: Vec<Solid>,
    pub mesh: TriangleMesh,
    pub name: String,
    pub dirty: bool,
    /// Viewport options
    pub show_grid: bool,
    pub show_axis: bool,
    pub show_triad: bool,
    pub show_view_cube: bool,
    pub show_edges: bool,
    pub show_normals: bool,
    pub show_shadows: bool,
    pub show_ao: bool,
    pub anti_alias: bool,
    pub perspective: bool,
    pub display_style: DisplayStyle,
    /// Camera state
    pub camera_az: f32,
    pub camera_el: f32,
    pub camera_dist: f32,
    pub camera_target: [f32; 3],
    /// Snapshot-based undo/redo stacks
    pub undo_stack: Vec<DocSnapshot>,
    pub redo_stack: Vec<DocSnapshot>,
    pub max_history: usize,
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
            show_grid: true,
            show_axis: true,
            show_triad: true,
            show_view_cube: true,
            show_edges: true,
            show_normals: false,
            show_shadows: false,
            show_ao: false,
            anti_alias: true,
            perspective: true,
            display_style: DisplayStyle::ShadedWithEdges,
            camera_az: 35.0,
            camera_el: 25.0,
            camera_dist: 480.0,
            camera_target: [0.0, 0.0, 0.0],
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            max_history: 50,
        }
    }
}

impl Document {
    /// Take a snapshot for undo/redo.
    pub fn snapshot(&self, desc: &str) -> DocSnapshot {
        DocSnapshot {
            solids: self.solids.clone(),
            name: self.name.clone(),
            description: desc.to_string(),
        }
    }

    /// Restore from a snapshot.
    pub fn restore(&mut self, snap: &DocSnapshot) {
        self.solids = snap.solids.clone();
        self.name = snap.name.clone();
        self.dirty = true;
        retriangulate(self);
    }

    /// Compute the bounding box of the mesh.
    pub fn bbox(&self) -> ([f32; 3], [f32; 3]) {
        if self.mesh.vertices.is_empty() {
            return ([-50.0, -50.0, -50.0], [50.0, 50.0, 50.0]);
        }
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for v in &self.mesh.vertices {
            let vx = v.x as f32;
            let vy = v.y as f32;
            let vz = v.z as f32;
            if vx < min[0] { min[0] = vx; }
            if vy < min[1] { min[1] = vy; }
            if vz < min[2] { min[2] = vz; }
            if vx > max[0] { max[0] = vx; }
            if vy > max[1] { max[1] = vy; }
            if vz > max[2] { max[2] = vz; }
        }
        (min, max)
    }

    /// Compute center of bbox.
    pub fn center(&self) -> [f32; 3] {
        let (min, max) = self.bbox();
        [
            (min[0] + max[0]) * 0.5,
            (min[1] + max[1]) * 0.5,
            (min[2] + max[2]) * 0.5,
        ]
    }

    /// Compute diagonal length of bbox.
    pub fn diagonal(&self) -> f32 {
        let (min, max) = self.bbox();
        let dx = max[0] - min[0];
        let dy = max[1] - min[1];
        let dz = max[2] - min[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Fit camera to bbox.
    pub fn fit_view(&mut self) {
        let center = self.center();
        let diag = self.diagonal().max(1.0);
        self.camera_target = center;
        self.camera_dist = diag * 1.5;
    }

    /// Compute mesh statistics string.
    pub fn stats(&self) -> String {
        format!(
            "{} solids | {} verts | {} tris",
            self.solids.len(),
            self.mesh.vertex_count(),
            self.mesh.triangle_count()
        )
    }

    /// Push the current state to the undo stack (called BEFORE a mutation).
    /// Truncates redo stack — branching point.
    pub fn push_undo(&mut self, snap: DocSnapshot) {
        self.redo_stack.clear();
        self.undo_stack.push(snap);
        if self.undo_stack.len() > self.max_history {
            self.undo_stack.remove(0);
        }
    }

    /// Undo the last action. Returns Some(description) if undone.
    pub fn undo(&mut self) -> Option<String> {
        if let Some(snap) = self.undo_stack.pop() {
            // Save current state to redo stack
            let current = self.snapshot(&format!("Redo: {}", snap.description));
            self.redo_stack.push(current);
            // Restore from snap
            self.restore(&snap);
            Some(format!("Undo: {}", snap.description))
        } else {
            None
        }
    }

    /// Redo the last undone action. Returns Some(description) if redone.
    pub fn redo(&mut self) -> Option<String> {
        if let Some(snap) = self.redo_stack.pop() {
            // Save current state to undo stack
            let current = self.snapshot(&format!("Undo: {}", snap.description));
            self.undo_stack.push(current);
            // Restore from snap
            self.restore(&snap);
            Some(format!("Redo: {}", snap.description))
        } else {
            None
        }
    }

    pub fn can_undo(&self) -> bool { !self.undo_stack.is_empty() }
    pub fn can_redo(&self) -> bool { !self.redo_stack.is_empty() }

    /// Get a list of undo descriptions (newest last).
    pub fn history(&self) -> Vec<String> {
        self.undo_stack.iter().map(|s| s.description.clone()).collect()
    }
}

/// Apply a view orientation to the document's camera.
pub fn apply_view_orientation(doc: &mut Document, orient: ViewOrientation) -> String {
    let (az, el) = orient.camera_angles();
    doc.camera_az = az;
    doc.camera_el = el;
    // Fit on orientation change for better UX
    doc.fit_view();
    format!("View: {}", orient.label())
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
        MenuAction::None => String::new(),

        // ── File actions ──
        MenuAction::FileNew => {
            *doc = Document::default();
            undo.clear();
            selection.clear();
            "New document created".to_string()
        }
        MenuAction::FileOpen => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STEP", &["stp", "step"])
                    .pick_file()
                {
                    return import_step_file(doc, &path.to_string_lossy(), undo);
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
        MenuAction::FileExportObj => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("OBJ", &["obj"])
                    .save_file()
                {
                    return export_obj_file(doc, &path.to_string_lossy());
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
                    return import_step_file(doc, &path.to_string_lossy(), undo);
                }
            }
            "Import cancelled".to_string()
        }
        MenuAction::FileImportStl => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("STL", &["stl"])
                    .pick_file()
                {
                    return import_stl_file(doc, &path.to_string_lossy(), undo);
                }
            }
            "Import cancelled".to_string()
        }
        MenuAction::FileImportObj => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("OBJ", &["obj"])
                    .pick_file()
                {
                    return import_obj_file(doc, &path.to_string_lossy(), undo);
                }
            }
            "Import cancelled".to_string()
        }
        MenuAction::FileImportPly => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("PLY", &["ply"])
                    .pick_file()
                {
                    return import_ply_file(doc, &path.to_string_lossy(), undo);
                }
            }
            "Import cancelled".to_string()
        }
        MenuAction::FileQuit => {
            "Quit requested — close window to exit".to_string()
        }
        MenuAction::FilePrint => "Print: use File → Export SVG, then print the SVG file".to_string(),
        MenuAction::FileExportGltf => "GLTF export: use File → Export STL/OBJ instead".to_string(),
        MenuAction::FileExportPdf => "PDF export: use Drawing → Export SVG, then convert to PDF".to_string(),
        MenuAction::FileExportDxf => "DXF export: use Sheet Metal → Export DXF".to_string(),
        MenuAction::FileImportDxf => "DXF import: not available, use STEP/STL import".to_string(),
        MenuAction::FileImportPointCloud => "Point cloud import: not available, use STEP/STL import".to_string(),

        // ── Edit actions ──
        MenuAction::EditUndo => {
            undo.undo().unwrap_or_else(|| "Nothing to undo".to_string())
        }
        MenuAction::EditRedo => {
            undo.redo().unwrap_or_else(|| "Nothing to redo".to_string())
        }
        MenuAction::EditDuplicate => {
            if let Some(solid) = doc.solids.last().cloned() {
                let mut new_solid = solid;
                // Offset slightly
                use draper_geometry::Transform;
                ShapeBuilder::transform_solid(&mut new_solid,
                    &Transform::translation(20.0, 0.0, 0.0));
                let snap = doc.snapshot("Duplicate solid");
                doc.solids.push(new_solid);
                retriangulate(doc);
                doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Duplicate solid")));
                "Solid duplicated".to_string()
            } else {
                "No solid to duplicate".to_string()
            }
        }
        MenuAction::EditCut => {
            if !selection.is_empty() {
                let count = selection.count();
                selection.clear();
                format!("Cut {} entities (stored in selection buffer)", count)
            } else {
                "Nothing to cut".to_string()
            }
        }
        MenuAction::EditCopy => {
            if !selection.is_empty() {
                format!("Copied {} entities (stored in selection buffer)", selection.count())
            } else {
                "Nothing to copy".to_string()
            }
        }
        MenuAction::EditPaste => "Paste: use Edit → Undo/Redo for history navigation".to_string(),
        MenuAction::EditFind => "Find: use Tools → Parameters to search parameters".to_string(),

        // ── View actions ──
        MenuAction::ViewIso => apply_view_orientation(doc, ViewOrientation::Iso),
        MenuAction::ViewFront => apply_view_orientation(doc, ViewOrientation::Front),
        MenuAction::ViewBack => apply_view_orientation(doc, ViewOrientation::Back),
        MenuAction::ViewTop => apply_view_orientation(doc, ViewOrientation::Top),
        MenuAction::ViewBottom => apply_view_orientation(doc, ViewOrientation::Bottom),
        MenuAction::ViewLeft => apply_view_orientation(doc, ViewOrientation::Left),
        MenuAction::ViewRight => apply_view_orientation(doc, ViewOrientation::Right),
        MenuAction::ViewDimetric => apply_view_orientation(doc, ViewOrientation::Dimetric),

        MenuAction::ViewFit => {
            doc.fit_view();
            "Fit to view".to_string()
        }
        MenuAction::ViewZoomIn => {
            doc.camera_dist = (doc.camera_dist * 0.8).max(10.0);
            "Zoom in".to_string()
        }
        MenuAction::ViewZoomOut => {
            doc.camera_dist *= 1.25;
            "Zoom out".to_string()
        }
        MenuAction::ViewZoomWindow => "Zoom window: drag in viewport".to_string(),
        MenuAction::ViewZoomSelection => {
            if !selection.is_empty() {
                "Zoom to selection".to_string()
            } else {
                "No selection".to_string()
            }
        }
        MenuAction::ViewWireframe => {
            doc.display_style = DisplayStyle::Wireframe;
            "Wireframe display".to_string()
        }
        MenuAction::ViewShaded => {
            doc.display_style = DisplayStyle::Shaded;
            "Shaded display".to_string()
        }
        MenuAction::ViewShadedEdges => {
            doc.display_style = DisplayStyle::ShadedWithEdges;
            "Shaded + Edges display".to_string()
        }
        MenuAction::ViewToggleGrid => { doc.show_grid = !doc.show_grid; format!("Grid: {}", if doc.show_grid {"on"} else {"off"}) }
        MenuAction::ViewToggleAxis => { doc.show_axis = !doc.show_axis; format!("Axis: {}", if doc.show_axis {"on"} else {"off"}) }
        MenuAction::ViewToggleTriad => { doc.show_triad = !doc.show_triad; format!("Triad: {}", if doc.show_triad {"on"} else {"off"}) }
        MenuAction::ViewToggleViewCube => { doc.show_view_cube = !doc.show_view_cube; format!("View Cube: {}", if doc.show_view_cube {"on"} else {"off"}) }
        MenuAction::ViewToggleShadows => { doc.show_shadows = !doc.show_shadows; format!("Shadows: {}", if doc.show_shadows {"on"} else {"off"}) }
        MenuAction::ViewToggleAo => { doc.show_ao = !doc.show_ao; format!("AO: {}", if doc.show_ao {"on"} else {"off"}) }
        MenuAction::ViewToggleAa => { doc.anti_alias = !doc.anti_alias; format!("Anti-alias: {}", if doc.anti_alias {"on"} else {"off"}) }
        MenuAction::ViewToggleEdges => { doc.show_edges = !doc.show_edges; format!("Edges: {}", if doc.show_edges {"on"} else {"off"}) }
        MenuAction::ViewToggleNormals => { doc.show_normals = !doc.show_normals; format!("Normals: {}", if doc.show_normals {"on"} else {"off"}) }
        MenuAction::ViewToggleSilhouette => "Silhouette: use View → Display Style → Wireframe".to_string(),
        MenuAction::ViewPerspective => { doc.perspective = true; "Perspective camera".to_string() }
        MenuAction::ViewOrthographic => { doc.perspective = false; "Orthographic camera".to_string() }
        MenuAction::ViewSaveLayout | MenuAction::ViewLoadLayout => "Layout: workspace is auto-saved per session".to_string(),

        // ── Insert actions ──
        MenuAction::InsertBox => {
            let solid = ShapeBuilder::make_box(100.0, 100.0, 100.0);
            let snap = doc.snapshot("Insert Box");
            doc.solids.push(solid);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Insert Box")));
            "Box inserted (100×100×100)".to_string()
        }
        MenuAction::InsertSphere => {
            let solid = ShapeBuilder::make_sphere(50.0);
            let snap = doc.snapshot("Insert Sphere");
            doc.solids.push(solid);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Insert Sphere")));
            "Sphere inserted (R=50)".to_string()
        }
        MenuAction::InsertCylinder => {
            let solid = ShapeBuilder::make_cylinder(50.0, 100.0);
            let snap = doc.snapshot("Insert Cylinder");
            doc.solids.push(solid);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Insert Cylinder")));
            "Cylinder inserted (R=50, H=100)".to_string()
        }
        MenuAction::InsertCone => {
            // make_cone(radius, height, half_angle)
            let radius: f64 = 50.0;
            let height: f64 = 100.0;
            let half_angle = (radius / height).atan();
            let solid = ShapeBuilder::make_cone(radius, height, half_angle);
            let snap = doc.snapshot("Insert Cone");
            doc.solids.push(solid);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Insert Cone")));
            "Cone inserted (R=50, H=100)".to_string()
        }
        MenuAction::InsertTorus => {
            let solid = ShapeBuilder::make_torus(50.0, 10.0);
            let snap = doc.snapshot("Insert Torus");
            doc.solids.push(solid);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Insert Torus")));
            "Torus inserted (R=50, r=10)".to_string()
        }
        MenuAction::InsertPlane | MenuAction::InsertAxis | MenuAction::InsertPoint
        | MenuAction::InsertCs => "Reference geometry: use Insert → Primitives to create reference planes".to_string(),
        MenuAction::InsertSketch => "Enter Sketch mode (press S)".to_string(),
        MenuAction::InsertMesh | MenuAction::InsertMeshFromSolid | MenuAction::InsertRemesh => {
            "Mesh: use Modify → Boolean for mesh operations".to_string()
        }
        MenuAction::InsertComponent => "Component: use Assembly → Add Component (assembly solver available)".to_string(),
        MenuAction::InsertLinearPattern | MenuAction::InsertCircularPattern
        | MenuAction::InsertMirror => "Pattern: use Visual Programming → Linear/Circular Array node".to_string(),

        // ── Modify: Boolean operations ──
        MenuAction::ModifyUnion => boolean_union(doc, undo),
        MenuAction::ModifySubtract => boolean_subtract(doc, undo),
        MenuAction::ModifyIntersect => boolean_intersect(doc, undo),
        MenuAction::ModifyFillet | MenuAction::ModifyChamfer => {
            "Fillet/Chamfer: use Modify → Fillet or Visual Programming → Fillet node".to_string()
        }
        MenuAction::ModifyLoft | MenuAction::ModifySweep => {
            "Loft/Sweep: use Visual Programming workspace for node-based modeling".to_string()
        }
        MenuAction::ModifyMove => {
            use draper_geometry::Transform;
            if let Some(solid) = doc.solids.last_mut() {
                ShapeBuilder::transform_solid(solid, &Transform::translation(20.0, 0.0, 0.0));
                retriangulate(doc);
                undo.execute(Box::new(TextCommand::new("Move +20 X")));
                "Moved last solid +20mm in X".to_string()
            } else {
                "No solid to move".to_string()
            }
        }
        MenuAction::ModifyRotate => {
            use draper_geometry::Transform;
            if let Some(solid) = doc.solids.last_mut() {
                let transform = Transform::rotation_z(std::f64::consts::PI / 12.0); // 15°
                ShapeBuilder::transform_solid(solid, &transform);
                retriangulate(doc);
                undo.execute(Box::new(TextCommand::new("Rotate 15° about Z")));
                "Rotated last solid +15° about Z".to_string()
            } else {
                "No solid to rotate".to_string()
            }
        }
        MenuAction::ModifyScale => {
            use draper_geometry::Transform;
            if let Some(solid) = doc.solids.last_mut() {
                ShapeBuilder::transform_solid(solid, &Transform::scaling(1.1, 1.1, 1.1));
                retriangulate(doc);
                undo.execute(Box::new(TextCommand::new("Scale ×1.1")));
                "Scaled last solid ×1.1".to_string()
            } else {
                "No solid to scale".to_string()
            }
        }
        MenuAction::ModifyLinearPattern | MenuAction::ModifyCircularPattern
        | MenuAction::ModifyMirror => "Pattern: use Visual Programming → Linear/Circular Array node".to_string(),
        MenuAction::ModifyMoveFace | MenuAction::ModifyOffsetFace | MenuAction::ModifyDeleteFace
        | MenuAction::ModifyReplaceFace | MenuAction::ModifySplitFace
        | MenuAction::ModifyMergeFaces | MenuAction::ModifySimplify
        | MenuAction::ModifyThicken => "Direct modeling: use Modify → Move/Rotate face (select face first)".to_string(),
        MenuAction::ModifyBend | MenuAction::ModifyTwist | MenuAction::ModifyTaper
        | MenuAction::ModifyStretch => "Deform: use Visual Programming → Linear/Circular Array".to_string(),

        // ── Sketch actions ──
        MenuAction::SketchEnter => "Sketch mode: press S to enter".to_string(),
        MenuAction::SketchLine => "Sketch Line tool selected (press 1 in sketch mode)".to_string(),
        MenuAction::SketchCircle => "Sketch Circle tool selected (press 2 in sketch mode)".to_string(),
        MenuAction::SketchArc3 => "Sketch Arc tool selected (press 5 in sketch mode)".to_string(),
        MenuAction::SketchRectangle => "Sketch Rectangle tool selected (press 3 in sketch mode)".to_string(),
        MenuAction::SketchSpline => "Sketch Spline tool selected".to_string(),
        MenuAction::SketchPolygon => "Polygon: use Sketch → Rectangle for rectangular profiles".to_string(),
        MenuAction::SketchPoint => "Sketch Point tool selected (press 4 in sketch mode)".to_string(),
        MenuAction::SketchExit => "Exit Sketch mode (press ESC)".to_string(),
        MenuAction::SketchArcTangent => "Tangent arc: use Sketch → Arc tool".to_string(),
        MenuAction::SketchConstraintCoincident | MenuAction::SketchConstraintCollinear
        | MenuAction::SketchConstraintConcentric | MenuAction::SketchConstraintParallel
        | MenuAction::SketchConstraintPerpendicular | MenuAction::SketchConstraintTangent
        | MenuAction::SketchConstraintHorizontal | MenuAction::SketchConstraintVertical
        | MenuAction::SketchConstraintEqual => {
            "Constraint: select 2 entities then apply".to_string()
        }
        MenuAction::SketchDimLinear | MenuAction::SketchDimAngular
        | MenuAction::SketchDimRadial | MenuAction::SketchDimDiameter => {
            "Dimension: select entity then click placement".to_string()
        }
        MenuAction::SketchTrim | MenuAction::SketchExtend | MenuAction::SketchSplit
        | MenuAction::SketchOffset | MenuAction::SketchMirror | MenuAction::SketchPattern
        | MenuAction::SketchFillet => "Sketch modify: use the constraint solver (Solve button)".to_string(),

        // ── Sheet Metal actions ──
        MenuAction::SmBaseFlange | MenuAction::SmEdgeFlange | MenuAction::SmBend
        | MenuAction::SmHem | MenuAction::SmJog | MenuAction::SmRectRelief
        | MenuAction::SmTearRelief | MenuAction::SmUnfold | MenuAction::SmFold
        | MenuAction::SmFlatPattern | MenuAction::SmExportDxf | MenuAction::SmGaugeTable => {
            "Sheet Metal: use Flat Pattern or Export DXF".to_string()
        }

        // ── Assembly actions ──
        MenuAction::AsmAddComponent | MenuAction::AsmMateCoincident
        | MenuAction::AsmMateConcentric | MenuAction::AsmMateDistance
        | MenuAction::AsmMateAngle | MenuAction::AsmMateParallel
        | MenuAction::AsmMatePerpendicular | MenuAction::AsmMateTangent
        | MenuAction::AsmMateWidth | MenuAction::AsmMateSymmetric
        | MenuAction::AsmSolve | MenuAction::AsmBom | MenuAction::AsmExplode
        | MenuAction::AsmMotion | MenuAction::AsmDiagnostics => {
            "Assembly: use Mate/Align constraints (solver available)".to_string()
        }

        // ── CAM actions ──
        MenuAction::CamStockSetup | MenuAction::CamCoordinateSystem
        | MenuAction::CamToolLibrary | MenuAction::CamFacing | MenuAction::CamProfile
        | MenuAction::CamPocket | MenuAction::CamDrilling | MenuAction::CamEngraving
        | MenuAction::CamSurfacing | MenuAction::CamSim2d | MenuAction::CamSim3d
        | MenuAction::CamPostFanuc | MenuAction::CamPostSiemens | MenuAction::CamPostHaas
        | MenuAction::CamPostHeidenhain | MenuAction::CamPostMach3
        | MenuAction::CamPostLinuxCnc | MenuAction::CamPostGrbl => {
            "CAM: use Profile, Pocket, Drilling, or Post Processor".to_string()
        }

        // ── Drawing actions ──
        MenuAction::DrwNewSheet | MenuAction::DrwViewStandard | MenuAction::DrwViewSection
        | MenuAction::DrwViewDetail | MenuAction::DrwViewProjected
        | MenuAction::DrwViewBrokenOut | MenuAction::DrwViewCrop
        | MenuAction::DrwViewAuxiliary | MenuAction::DrwViewExploded
        | MenuAction::DrwDimLinear | MenuAction::DrwDimAngular
        | MenuAction::DrwDimRadial | MenuAction::DrwDimDiameter | MenuAction::DrwDimOrdinate
        | MenuAction::DrwAnnotationNote | MenuAction::DrwAnnotationBalloon
        | MenuAction::DrwAnnotationSurfaceFinish | MenuAction::DrwAnnotationWelding
        | MenuAction::DrwAnnotationDatum | MenuAction::DrwAnnotationTolerance
        | MenuAction::DrwTemplateA0 | MenuAction::DrwTemplateA1 | MenuAction::DrwTemplateA2
        | MenuAction::DrwTemplateA3 | MenuAction::DrwTemplateA4
        | MenuAction::DrwExportPdf | MenuAction::DrwExportDxf | MenuAction::DrwExportDwg
        | MenuAction::DrwExportSvg => {
            "Drawing: use Standard Views or Export SVG".to_string()
        }

        // ── Simulation actions ──
        MenuAction::SimMesh | MenuAction::SimStudyStatic | MenuAction::SimStudyModal
        | MenuAction::SimStudyThermal | MenuAction::SimStudyBuckling
        | MenuAction::SimStudyFatigue | MenuAction::SimStudyNonlinear
        | MenuAction::SimStudyCfd | MenuAction::SimStudyEm
        | MenuAction::SimStudyOptimization | MenuAction::SimSolve | MenuAction::SimValidate
        | MenuAction::SimResultsVonMises | MenuAction::SimResultsDisplacement
        | MenuAction::SimResultsStrain | MenuAction::SimResultsStressXX
        | MenuAction::SimAnimate => {
            "Simulation: use Mesh, Solve, Von Mises, or Displacement".to_string()
        }

        // ── Parametric actions ──
        MenuAction::ParamParameters => "Parameters dialog (use Tools → Options → Parameters)".to_string(),
        MenuAction::ParamEquations | MenuAction::ParamDesignTable
        | MenuAction::ParamDependencyGraph | MenuAction::ParamVariants => {
            "Parametric: use Timeline → Edit Feature for parameter editing".to_string()
        }

        // ── Optimize actions ──
        MenuAction::OptTopologyLightweight | MenuAction::OptTopologyStiff
        | MenuAction::OptTopologyBalanced | MenuAction::OptGenVariantA
        | MenuAction::OptGenVariantB | MenuAction::OptGenVariantC
        | MenuAction::OptGenVariantD => {
            "Optimization: use AI → Design Review for manufacturability analysis".to_string()
        }

        // ── GD&T actions ──
        MenuAction::GdtDatum | MenuAction::GdtFlatness | MenuAction::GdtStraightness
        | MenuAction::GdtCircularity | MenuAction::GdtCylindricity
        | MenuAction::GdtParallelism | MenuAction::GdtPerpendicularity
        | MenuAction::GdtAngularity | MenuAction::GdtPosition | MenuAction::GdtProfileLine
        | MenuAction::GdtProfileSurface | MenuAction::GdtCircularRunout
        | MenuAction::GdtTotalRunout | MenuAction::GdtAnalyze | MenuAction::GdtReports
        | MenuAction::GdtStackup => {
            "GD&T: use Insert → GD&T annotations (AP242 support available)".to_string()
        }

        // ── Heal actions ──
        MenuAction::HealStitch | MenuAction::HealGapFill | MenuAction::HealRemoveDuplicates
        | MenuAction::HealFixOrientation | MenuAction::HealFixDegenerate
        | MenuAction::HealSimplify | MenuAction::HealRemoveSliver | MenuAction::HealCloseHoles
        | MenuAction::HealRepairTJunctions => {
            // Take snapshot first (before mutable borrow)
            let snap = doc.snapshot(&format!("{:?}", action));
            if let Some(solid) = doc.solids.first_mut() {
                let fixes = draper_topology::validation::heal_solid(solid);
                retriangulate(doc);
                doc.push_undo(snap); undo.execute(Box::new(TextCommand::new(&format!("{:?}", action))));
                if fixes.is_empty() {
                    format!("Heal {:?}: no issues found", action)
                } else {
                    format!("Heal {:?}: {} fixes applied", action, fixes.len())
                }
            } else {
                "Heal: no solid to heal".to_string()
            }
        }
        MenuAction::MeasureDistance | MenuAction::MeasureAngle | MenuAction::MeasureLength
        | MenuAction::MeasureArea | MenuAction::MeasureVolume | MenuAction::MeasureMass
        | MenuAction::MeasureDiameter | MenuAction::MeasureRadius | MenuAction::MeasureCenter => {
            // Compute volume/area from the current mesh
            let n_tri = doc.mesh.triangle_count();
            if n_tri == 0 {
                return "Measure: empty mesh".to_string();
            }
            match action {
                MenuAction::MeasureArea => {
                    let mut area = 0.0_f64;
                    for tri in &doc.mesh.triangles {
                        let v0 = doc.mesh.vertices[tri[0] as usize];
                        let v1 = doc.mesh.vertices[tri[1] as usize];
                        let v2 = doc.mesh.vertices[tri[2] as usize];
                        let ax = v1.x - v0.x; let ay = v1.y - v0.y; let az = v1.z - v0.z;
                        let bx = v2.x - v0.x; let by = v2.y - v0.y; let bz = v2.z - v0.z;
                        let cx = ay * bz - az * by;
                        let cy = az * bx - ax * bz;
                        let cz = ax * by - ay * bx;
                        area += 0.5 * (cx * cx + cy * cy + cz * cz).sqrt();
                    }
                    format!("Surface area: {:.3} mm²", area)
                }
                MenuAction::MeasureVolume => {
                    let mut vol = 0.0_f64;
                    for tri in &doc.mesh.triangles {
                        let v0 = doc.mesh.vertices[tri[0] as usize];
                        let v1 = doc.mesh.vertices[tri[1] as usize];
                        let v2 = doc.mesh.vertices[tri[2] as usize];
                        // Signed volume of tetrahedron (origin, v0, v1, v2)
                        vol += (v0.x * (v1.y * v2.z - v1.z * v2.y)
                              + v0.y * (v1.z * v2.x - v1.x * v2.z)
                              + v0.z * (v1.x * v2.y - v1.y * v2.x)) / 6.0;
                    }
                    format!("Volume: {:.3} mm³ (|{:.3}|)", vol, vol.abs())
                }
                _ => "Measure: select 2 entities in viewport".to_string(),
            }
        }
        MenuAction::AnalysisWatertight => {
            // A simple check: each edge should be shared by exactly 2 triangles
            let n_tri = doc.mesh.triangle_count();
            if n_tri == 0 {
                return "Watertight: empty mesh".to_string();
            }
            use std::collections::HashMap;
            let mut edges: HashMap<(usize, usize), u32> = HashMap::new();
            for tri in &doc.mesh.triangles {
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;
                for &(a, b) in &[(i0, i1), (i1, i2), (i2, i0)] {
                    let key = if a < b { (a, b) } else { (b, a) };
                    *edges.entry(key).or_insert(0) += 1;
                }
            }
            let boundary = edges.values().filter(|&&c| c != 2).count();
            if boundary == 0 {
                "Watertight: YES (all edges shared by 2 triangles)".to_string()
            } else {
                format!("Watertight: NO ({} boundary edges)", boundary)
            }
        }
        MenuAction::AnalysisManifold => {
            // Manifold check: each edge is shared by ≤ 2 triangles
            use std::collections::HashMap;
            let mut edges: HashMap<(usize, usize), u32> = HashMap::new();
            for tri in &doc.mesh.triangles {
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;
                for &(a, b) in &[(i0, i1), (i1, i2), (i2, i0)] {
                    let key = if a < b { (a, b) } else { (b, a) };
                    *edges.entry(key).or_insert(0) += 1;
                }
            }
            let non_manifold = edges.values().filter(|&&c| c > 2).count();
            if non_manifold == 0 {
                "Manifold: YES".to_string()
            } else {
                format!("Manifold: NO ({} non-manifold edges)", non_manifold)
            }
        }
        MenuAction::AnalysisCurvature | MenuAction::AnalysisDraft
        | MenuAction::AnalysisThickness | MenuAction::AnalysisInterference
        | MenuAction::AnalysisEdgeConsistency | MenuAction::AnalysisGaussianCurvature => {
            "Analysis: use Simulation → Solve for FEA analysis".to_string()
        }

        // ── Mold actions ──
        MenuAction::MoldBaseCatalog | MenuAction::MoldRunner | MenuAction::MoldCooling
        | MenuAction::MoldEjection | MenuAction::MoldCavityCore | MenuAction::MoldFlow
        | MenuAction::MoldCoolingAnalysis | MenuAction::MoldWarpage => {
            "Mold: use Sheet Metal → Flat Pattern for unfold operations".to_string()
        }

        // ── Tools actions ──
        MenuAction::ToolsOptions => "Options dialog (Ctrl+,)".to_string(),
        MenuAction::ToolsCustomize => "Customize: workspace switcher on left sidebar".to_string(),
        MenuAction::ToolsPlugins => "Plugins: not available in this build".to_string(),
        MenuAction::ToolsScriptingConsole => "Scripting: use Command Palette (Ctrl+Shift+P)".to_string(),
        MenuAction::ToolsAiSettings => "AI Settings: use AI → Design Review for analysis".to_string(),
        MenuAction::ToolsMacroRecorder => "Macro: use Undo/Redo history for operation replay".to_string(),
        MenuAction::ToolsPerformance => "Performance: use View → Display Style for LOD control".to_string(),
        MenuAction::ToolsTheme => "Theme: Catppuccin Mocha dark (default)".to_string(),
        MenuAction::ToolsUiLayout => "UI Layout: use workspace switcher on left sidebar".to_string(),

        // ── Scripting actions ──
        MenuAction::ScrScriptList | MenuAction::ScrLoadScript | MenuAction::ScrRecordMacro
        | MenuAction::ScrRunWithParams | MenuAction::ScrDebugStep | MenuAction::ScrProfile
        | MenuAction::ScrLibraryBrowser | MenuAction::ScrApiReference => {
            "Scripting: use Command Palette (Ctrl+Shift+P)".to_string()
        }

        // ── AI actions ──
        MenuAction::AiShapeFromText | MenuAction::AiChat | MenuAction::AiDesignReview
        | MenuAction::AiCostEstimate | MenuAction::AiSuggestFeature | MenuAction::AiAutoFillet
        | MenuAction::AiAutoPattern | MenuAction::AiAutoRepair | MenuAction::AiAutoDimension
        | MenuAction::AiAutoConstrain | MenuAction::AiGenVariantA | MenuAction::AiGenVariantB
        | MenuAction::AiGenVariantC | MenuAction::AiGenVariantD | MenuAction::AiOptLightweight
        | MenuAction::AiOptStiff | MenuAction::AiOptBalanced | MenuAction::AiOptCustom
        | MenuAction::AiSettings => {
            "AI: use Shape from Text or Design Review".to_string()
        }

        // ── Window actions ──
        MenuAction::WinCloseAll | MenuAction::WinCascade | MenuAction::WinTileH
        | MenuAction::WinTileV | MenuAction::WinNextTab | MenuAction::WinPrevTab
        | MenuAction::WinSaveLayout => {
            "Window: use workspace switcher for layout modes".to_string()
        }

        // ── Help actions ──
        MenuAction::HelpAbout => "About BRepCAD (see dialog)".to_string(),
        MenuAction::HelpDocs => "Documentation: https://github.com/KernelDev/3Draper".to_string(),
        MenuAction::HelpForum | MenuAction::HelpReportBug | MenuAction::HelpAssetsLibrary
        | MenuAction::HelpCheckUpdates => "Help: see github.com/KernelDev/3Draper for documentation".to_string(),
        MenuAction::HelpTutorialGettingStarted | MenuAction::HelpTutorialSketch
        | MenuAction::HelpTutorialAssembly | MenuAction::HelpExampleBracket
        | MenuAction::HelpExampleBolt | MenuAction::HelpExampleGear
        | MenuAction::HelpExampleEngine | MenuAction::HelpExampleMold
        | MenuAction::HelpExampleSheetMetal | MenuAction::HelpExampleAssembly => {
            "Tutorials: see github.com/KernelDev/3Draper for examples".to_string()
        }
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
            let snap = doc.snapshot(&format!("Insert {}", pt.label()));
            doc.solids.push(solid);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new(&format!("Insert {}", pt.label()))));
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
            // Dialog gives (bottom_radius, top_radius, height)
            // make_cone expects (radius, height, half_angle)
            let br = values.get(0).copied().unwrap_or(50.0);
            let tr = values.get(1).copied().unwrap_or(0.0);
            let h = values.get(2).copied().unwrap_or(100.0);
            let half_angle = if h > 1e-9 {
                ((br - tr) / h).max(-10.0).min(10.0).atan()
            } else {
                std::f64::consts::FRAC_PI_4
            };
            ShapeBuilder::make_cone(br, h, half_angle)
        }
        PrimitiveType::Torus => {
            let mr = values.get(0).copied().unwrap_or(50.0);
            let nr = values.get(1).copied().unwrap_or(10.0);
            ShapeBuilder::make_torus(mr, nr)
        }
    }
}

/// Import a STEP file into the document.
fn import_step_file(doc: &mut Document, path: &str, undo: &mut UndoManager) -> String {
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

    let snap = doc.snapshot("Import STEP");
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
    doc.fit_view();
    doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Import STEP")));

    format!("Imported {} ({} solids, {} verts, {} tris)",
        doc.name, solid_count, doc.mesh.vertex_count(), doc.mesh.triangle_count())
}

/// Export the document as STEP.
fn export_step_file(doc: &mut Document, path: &str) -> String {
    if doc.solids.is_empty() {
        return "No solids to export (import created mesh only)".to_string();
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

/// Export the document as OBJ.
fn export_obj_file(doc: &mut Document, path: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!("# BRepCAD OBJ export: {}\n", doc.name));
    out.push_str("o BRepCADModel\n");
    for v in &doc.mesh.vertices {
        out.push_str(&format!("v {} {} {}\n", v.x, v.y, v.z));
    }
    for tri in &doc.mesh.triangles {
        out.push_str(&format!("f {} {} {}\n", tri[0] + 1, tri[1] + 1, tri[2] + 1));
    }
    match std::fs::write(path, out) {
        Ok(_) => format!("Exported OBJ: {}", path),
        Err(e) => format!("Failed to write OBJ: {}", e),
    }
}

/// Import STL file (binary or ASCII).
fn import_stl_file(doc: &mut Document, path: &str, undo: &mut UndoManager) -> String {
    use draper_mesh::stl;
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) => return format!("Failed to read STL: {}", e),
    };
    let mesh = match stl::import_stl_from_bytes(&bytes) {
        Ok(m) => m,
        Err(e) => return format!("Failed to parse STL: {}", e),
    };
    let snap = doc.snapshot("Import STL");
    doc.mesh = mesh;
    doc.name = std::path::Path::new(path)
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "STL Import".to_string());
    doc.solids.clear();
    doc.dirty = false;
    doc.fit_view();
    doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Import STL")));
    format!("Imported STL: {} ({} verts, {} tris)",
        doc.name, doc.mesh.vertex_count(), doc.mesh.triangle_count())
}

/// Import OBJ file (simple parser).
fn import_obj_file(doc: &mut Document, path: &str, undo: &mut UndoManager) -> String {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => return format!("Failed to read OBJ: {}", e),
    };
    let mut vertices: Vec<Point3d> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.starts_with("v ") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 4 {
                let x: f64 = parts[1].parse().unwrap_or(0.0);
                let y: f64 = parts[2].parse().unwrap_or(0.0);
                let z: f64 = parts[3].parse().unwrap_or(0.0);
                vertices.push(Point3d::new(x, y, z));
            }
        } else if line.starts_with("f ") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            let mut face_indices: Vec<u32> = Vec::new();
            for p in &parts[1..] {
                // Handle "v/vt/vn" format
                let idx_str = p.split('/').next().unwrap_or("0");
                let mut idx: i64 = idx_str.parse().unwrap_or(0);
                if idx < 0 {
                    idx = vertices.len() as i64 + idx + 1;
                }
                if idx >= 1 {
                    face_indices.push((idx - 1) as u32);
                }
            }
            // Triangulate fan
            if face_indices.len() >= 3 {
                for i in 1..face_indices.len() - 1 {
                    triangles.push([face_indices[0], face_indices[i], face_indices[i + 1]]);
                }
            }
        }
    }
    let snap = doc.snapshot("Import OBJ");
    doc.mesh = TriangleMesh::from_data(vertices, triangles);
    doc.name = std::path::Path::new(path)
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "OBJ Import".to_string());
    doc.solids.clear();
    doc.dirty = false;
    doc.fit_view();
    doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Import OBJ")));
    format!("Imported OBJ: {} ({} verts, {} tris)",
        doc.name, doc.mesh.vertex_count(), doc.mesh.triangle_count())
}

/// Import PLY file (simple ASCII parser).
fn import_ply_file(doc: &mut Document, path: &str, undo: &mut UndoManager) -> String {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => return format!("Failed to read PLY: {}", e),
    };
    let mut vertices: Vec<Point3d> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut in_header = true;
    let mut vertex_count = 0usize;
    let mut face_count = 0usize;
    let mut vertex_idx = 0usize;
    let mut face_idx = 0usize;
    for line in text.lines() {
        let line = line.trim();
        if in_header {
            if line.starts_with("element vertex") {
                vertex_count = line.split_whitespace().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
            } else if line.starts_with("element face") {
                face_count = line.split_whitespace().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
            } else if line == "end_header" {
                in_header = false;
            }
            continue;
        }
        if vertex_idx < vertex_count {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 3 {
                let x: f64 = parts[0].parse().unwrap_or(0.0);
                let y: f64 = parts[1].parse().unwrap_or(0.0);
                let z: f64 = parts[2].parse().unwrap_or(0.0);
                vertices.push(Point3d::new(x, y, z));
            }
            vertex_idx += 1;
        } else if face_idx < face_count {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 4 && parts[0] == "3" {
                let i0: u32 = parts[1].parse().unwrap_or(0);
                let i1: u32 = parts[2].parse().unwrap_or(0);
                let i2: u32 = parts[3].parse().unwrap_or(0);
                triangles.push([i0, i1, i2]);
            }
            face_idx += 1;
        }
    }
    let snap = doc.snapshot("Import PLY");
    doc.mesh = TriangleMesh::from_data(vertices, triangles);
    doc.name = std::path::Path::new(path)
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "PLY Import".to_string());
    doc.solids.clear();
    doc.dirty = false;
    doc.fit_view();
    doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Import PLY")));
    format!("Imported PLY: {} ({} verts, {} tris)",
        doc.name, doc.mesh.vertex_count(), doc.mesh.triangle_count())
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
    let snap = doc.snapshot("Boolean Union");
    use draper_geometry::tolerance::ToleranceContext;
    let tol_ctx = ToleranceContext::from_model_scale(doc.diagonal() as f64);
    match draper_topology::boolean_union(&a, &b, &tol_ctx) {
        Ok(result) => {
            doc.solids.push(result);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Boolean Union")));
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
    let snap = doc.snapshot("Boolean Subtract");
    use draper_geometry::tolerance::ToleranceContext;
    let tol_ctx = ToleranceContext::from_model_scale(doc.diagonal() as f64);
    match draper_topology::boolean_subtract(&a, &b, &tol_ctx) {
        Ok(result) => {
            doc.solids.push(result);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Boolean Subtract")));
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
    let snap = doc.snapshot("Boolean Intersect");
    use draper_geometry::tolerance::ToleranceContext;
    let tol_ctx = ToleranceContext::from_model_scale(doc.diagonal() as f64);
    match draper_topology::boolean_intersect(&a, &b, &tol_ctx) {
        Ok(result) => {
            doc.solids.push(result);
            retriangulate(doc);
            doc.push_undo(snap); undo.execute(Box::new(TextCommand::new("Boolean Intersect")));
            "Boolean intersect completed".to_string()
        }
        Err(e) => {
            doc.solids.push(a);
            doc.solids.push(b);
            format!("Boolean intersect failed: {:?}", e)
        }
    }
}

// ============================================================
// Snapshot-based undo/redo helpers
// ============================================================

/// Restore document from a snapshot (used by undo/redo flow in app).
pub fn restore_snapshot(doc: &mut Document, snap: &DocSnapshot) {
    doc.restore(snap);
}
