// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Ribbon tabs — 15 tabs per ROADMAP_UI Phase 2.
//!
//! Each tab contains grouped command buttons. The ribbon is rendered
//! as a horizontal bar below the menu bar.
//! Each button returns Option<MenuAction> that the dispatcher routes to the backend.

use eframe::egui;
use super::Workspace;
use super::menubar::MenuAction;

/// Which ribbon tab is active.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RibbonTab {
    File,
    Home,
    Sketch,
    Insert,
    Modify,
    SheetMetal,
    Assembly,
    Cam,
    Drawing,
    Simulation,
    Inspect,
    Ai,
    Tools,
    View,
    Surface,
}

impl RibbonTab {
    pub const ALL: &'static [RibbonTab] = &[
        RibbonTab::File,
        RibbonTab::Home,
        RibbonTab::Sketch,
        RibbonTab::Insert,
        RibbonTab::Modify,
        RibbonTab::SheetMetal,
        RibbonTab::Assembly,
        RibbonTab::Cam,
        RibbonTab::Drawing,
        RibbonTab::Simulation,
        RibbonTab::Inspect,
        RibbonTab::Ai,
        RibbonTab::Tools,
        RibbonTab::View,
        RibbonTab::Surface,
    ];

    fn label(&self) -> &'static str {
        match self {
            RibbonTab::File => "File",
            RibbonTab::Home => "Home",
            RibbonTab::Sketch => "Sketch",
            RibbonTab::Insert => "Insert",
            RibbonTab::Modify => "Modify",
            RibbonTab::SheetMetal => "Sheet Metal",
            RibbonTab::Assembly => "Assembly",
            RibbonTab::Cam => "CAM",
            RibbonTab::Drawing => "Drawing",
            RibbonTab::Simulation => "Simulation",
            RibbonTab::Inspect => "Inspect",
            RibbonTab::Ai => "AI",
            RibbonTab::Tools => "Tools",
            RibbonTab::View => "View",
            RibbonTab::Surface => "Surface",
        }
    }
}

/// Render the ribbon bar. Returns an action if any button was clicked.
pub fn render_ribbon(ctx: &egui::Context, active: &mut RibbonTab) -> Option<MenuAction> {
    let mut action = None;

    egui::TopBottomPanel::top("ribbon_bar")
        .exact_height(90.0)
        .show(ctx, |ui| {
            // Tab selector row with QAT (Quick Access Toolbar) on the right
            ui.horizontal(|ui| {
                // Tab selector (left side)
                ui.horizontal(|ui| {
                    for tab in RibbonTab::ALL {
                        if ui.selectable_label(*active == *tab, tab.label()).clicked() {
                            *active = *tab;
                        }
                    }
                });

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    // Quick Access Toolbar (QAT): Undo, Redo, Save
                    if ui.button("↶").on_hover_text("Undo (Ctrl+Z)").clicked() {
                        action = Some(MenuAction::EditUndo);
                    }
                    if ui.button("↷").on_hover_text("Redo (Ctrl+Shift+Z)").clicked() {
                        action = Some(MenuAction::EditRedo);
                    }
                    if ui.button("💾").on_hover_text("Save (Ctrl+S)").clicked() {
                        action = Some(MenuAction::FileSave);
                    }
                    ui.separator();
                    // Command search box (mockup 01: "Search commands... ⌘P")
                    let mut search_text = String::new();
                    ui.add(egui::TextEdit::singleline(&mut search_text)
                        .hint_text("Search... ⌘P")
                        .desired_width(120.0));
                });
            });

            ui.separator();

            // Content row — groups laid out horizontally, scrollable
            egui::ScrollArea::horizontal().show(ui, |ui| {
                ui.horizontal(|ui| {
                    if action.is_none() {
                        action = match active {
                            RibbonTab::File => render_file_ribbon(ui),
                            RibbonTab::Home => render_home_ribbon(ui),
                            RibbonTab::Sketch => render_sketch_ribbon(ui),
                            RibbonTab::Insert => render_insert_ribbon(ui),
                            RibbonTab::Modify => render_modify_ribbon(ui),
                            RibbonTab::SheetMetal => render_sheetmetal_ribbon(ui),
                            RibbonTab::Assembly => render_assembly_ribbon(ui),
                            RibbonTab::Cam => render_cam_ribbon(ui),
                            RibbonTab::Drawing => render_drawing_ribbon(ui),
                            RibbonTab::Simulation => render_simulation_ribbon(ui),
                            RibbonTab::Inspect => render_inspect_ribbon(ui),
                            RibbonTab::Ai => render_ai_ribbon(ui),
                            RibbonTab::Tools => render_tools_ribbon(ui),
                            RibbonTab::View => render_view_ribbon(ui),
                            RibbonTab::Surface => render_surface_ribbon(ui),
                        };
                    }
                });
            });
        });

    action
}

/// Helper: render a button group with a label at the bottom.
/// Groups are laid out horizontally (left to right) within the ribbon.
/// Buttons within a group are also horizontal.
fn group(ui: &mut egui::Ui, label: &str, content: impl FnOnce(&mut egui::Ui) -> Option<MenuAction>) -> Option<MenuAction> {
    let mut action = None;
    ui.vertical(|ui| {
        // Buttons row — horizontal
        ui.horizontal(|ui| {
            action = content(ui);
        });
        // Group label at the bottom
        ui.label(egui::RichText::new(label).small().color(egui::Color32::GRAY));
    });
    ui.separator();
    action
}

/// Helper: render a command button with drawn icon on top, label below.
/// Uses procedural icons from icons module instead of Unicode emoji.
fn cmd_button(ui: &mut egui::Ui, icon: &str, label: &str) -> bool {
    crate::ui::icons::icon_button(ui, icon, label)
}

/// Helper: emit an action if the button was clicked.
fn btn(ui: &mut egui::Ui, icon: &str, label: &str, action: MenuAction) -> Option<MenuAction> {
    if cmd_button(ui, icon, label) { Some(action) } else { None }
}

/// File ribbon tab.
fn render_file_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Document", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "new", "New") { a = Some(MenuAction::FileNew); }
        if a.is_none() && cmd_button(ui, "open", "Open") { a = Some(MenuAction::FileOpen); }
        if a.is_none() && cmd_button(ui, "save", "Save") { a = Some(MenuAction::FileSave); }
        a
    });}
    if action.is_none() { action = group(ui, "Import", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "open", "STEP") { a = Some(MenuAction::FileImportStep); }
        if a.is_none() && cmd_button(ui, "open", "STL") { a = Some(MenuAction::FileImportStl); }
        a
    });}
    if action.is_none() { action = group(ui, "Export", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "export", "STEP") { a = Some(MenuAction::FileExportStep); }
        if a.is_none() && cmd_button(ui, "export", "STL") { a = Some(MenuAction::FileExportStl); }
        if a.is_none() && cmd_button(ui, "export", "OBJ") { a = Some(MenuAction::FileExportObj); }
        a
    });}
    if action.is_none() { action = group(ui, "Render", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "print", "POV") { a = Some(MenuAction::FilePrint); }
        if a.is_none() && cmd_button(ui, "export", "Blender") { a = Some(MenuAction::FileExportObj); }
        a
    });}
    action
}

/// Home ribbon tab.
fn render_home_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Clipboard", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "cut", "Cut") { a = Some(MenuAction::EditCut); }
        if a.is_none() && cmd_button(ui, "copy", "Copy") { a = Some(MenuAction::EditCopy); }
        if a.is_none() && cmd_button(ui, "paste", "Paste") { a = Some(MenuAction::EditPaste); }
        a
    });}
    if action.is_none() { action = group(ui, "History", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "undo", "Undo") { a = Some(MenuAction::EditUndo); }
        if a.is_none() && cmd_button(ui, "redo", "Redo") { a = Some(MenuAction::EditRedo); }
        a
    });}
    if action.is_none() { action = group(ui, "View", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "fit", "Fit") { a = Some(MenuAction::ViewFit); }
        if a.is_none() && cmd_button(ui, "iso", "ISO") { a = Some(MenuAction::ViewIso); }
        a
    });}
    if action.is_none() { action = group(ui, "Display", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "box", "Wireframe") { a = Some(MenuAction::ViewWireframe); }
        if a.is_none() && cmd_button(ui, "sphere", "Shaded") { a = Some(MenuAction::ViewShaded); }
        if a.is_none() && cmd_button(ui, "box", "Shaded+") { a = Some(MenuAction::ViewShadedEdges); }
        a
    });}
    action
}

/// Sketch ribbon tab.
fn render_sketch_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Mode", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "line", "Sketch") { a = Some(MenuAction::SketchEnter); }
        if a.is_none() && cmd_button(ui, "fit", "Exit") { a = Some(MenuAction::SketchExit); }
        a
    });}
    if action.is_none() { action = group(ui, "Draw", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "line", "Line") { a = Some(MenuAction::SketchLine); }
        if a.is_none() && cmd_button(ui, "circle", "Circle") { a = Some(MenuAction::SketchCircle); }
        if a.is_none() && cmd_button(ui, "arc", "Arc") { a = Some(MenuAction::SketchArc3); }
        if a.is_none() && cmd_button(ui, "rectangle", "Rect") { a = Some(MenuAction::SketchRectangle); }
        if a.is_none() && cmd_button(ui, "spline", "Spline") { a = Some(MenuAction::SketchSpline); }
        a
    });}
    if action.is_none() { action = group(ui, "Constraint", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "point", "Coincident") { a = Some(MenuAction::SketchConstraintCoincident); }
        if a.is_none() && cmd_button(ui, "pattern_linear", "Parallel") { a = Some(MenuAction::SketchConstraintParallel); }
        if a.is_none() && cmd_button(ui, "pattern_circular", "Perpendicular") { a = Some(MenuAction::SketchConstraintPerpendicular); }
        if a.is_none() && cmd_button(ui, "arc", "Tangent") { a = Some(MenuAction::SketchConstraintTangent); }
        if a.is_none() && cmd_button(ui, "pattern_linear", "Horizontal") { a = Some(MenuAction::SketchConstraintHorizontal); }
        if a.is_none() && cmd_button(ui, "pattern_circular", "Vertical") { a = Some(MenuAction::SketchConstraintVertical); }
        a
    });}
    if action.is_none() { action = group(ui, "Dimension", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "dimension", "Linear") { a = Some(MenuAction::SketchDimLinear); }
        if a.is_none() && cmd_button(ui, "arc", "Angular") { a = Some(MenuAction::SketchDimAngular); }
        if a.is_none() && cmd_button(ui, "circle", "Radial") { a = Some(MenuAction::SketchDimRadial); }
        a
    });}
    action
}

/// Insert ribbon tab.
fn render_insert_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Primitives", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "box", "Box") { a = Some(MenuAction::InsertBox); }
        if a.is_none() && cmd_button(ui, "sphere", "Sphere") { a = Some(MenuAction::InsertSphere); }
        if a.is_none() && cmd_button(ui, "cylinder", "Cylinder") { a = Some(MenuAction::InsertCylinder); }
        if a.is_none() && cmd_button(ui, "cone", "Cone") { a = Some(MenuAction::InsertCone); }
        if a.is_none() && cmd_button(ui, "torus", "Torus") { a = Some(MenuAction::InsertTorus); }
        a
    });}
    if action.is_none() { action = group(ui, "Reference", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "rectangle", "Plane") { a = Some(MenuAction::InsertPlane); }
        if a.is_none() && cmd_button(ui, "line", "Axis") { a = Some(MenuAction::InsertAxis); }
        if a.is_none() && cmd_button(ui, "point", "Point") { a = Some(MenuAction::InsertPoint); }
        a
    });}
    if action.is_none() { action = group(ui, "Mesh", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "open", "Import") { a = Some(MenuAction::InsertMesh); }
        if a.is_none() && cmd_button(ui, "rotate", "Remesh") { a = Some(MenuAction::InsertRemesh); }
        a
    });}
    if action.is_none() { action = group(ui, "Pattern", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "pattern_linear", "Linear") { a = Some(MenuAction::InsertLinearPattern); }
        if a.is_none() && cmd_button(ui, "pattern_circular", "Circular") { a = Some(MenuAction::InsertCircularPattern); }
        if a.is_none() && cmd_button(ui, "mirror", "Mirror") { a = Some(MenuAction::InsertMirror); }
        a
    });}
    action
}

/// Modify ribbon tab.
fn render_modify_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Boolean", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "union", "Union") { a = Some(MenuAction::ModifyUnion); }
        if a.is_none() && cmd_button(ui, "subtract", "Subtract") { a = Some(MenuAction::ModifySubtract); }
        if a.is_none() && cmd_button(ui, "intersect", "Intersect") { a = Some(MenuAction::ModifyIntersect); }
        a
    });}
    if action.is_none() { action = group(ui, "Edge", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "fillet", "Fillet") { a = Some(MenuAction::ModifyFillet); }
        if a.is_none() && cmd_button(ui, "chamfer", "Chamfer") { a = Some(MenuAction::ModifyChamfer); }
        a
    });}
    if action.is_none() { action = group(ui, "Transform", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "move", "Move") { a = Some(MenuAction::ModifyMove); }
        if a.is_none() && cmd_button(ui, "rotate", "Rotate") { a = Some(MenuAction::ModifyRotate); }
        if a.is_none() && cmd_button(ui, "scale", "Scale") { a = Some(MenuAction::ModifyScale); }
        a
    });}
    if action.is_none() { action = group(ui, "Pattern", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "pattern_linear", "Linear") { a = Some(MenuAction::ModifyLinearPattern); }
        if a.is_none() && cmd_button(ui, "pattern_circular", "Circular") { a = Some(MenuAction::ModifyCircularPattern); }
        if a.is_none() && cmd_button(ui, "mirror", "Mirror") { a = Some(MenuAction::ModifyMirror); }
        a
    });}
    if action.is_none() { action = group(ui, "Direct", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "move", "Face") { a = Some(MenuAction::ModifyMoveFace); }
        if a.is_none() && cmd_button(ui, "scale", "Face") { a = Some(MenuAction::ModifyOffsetFace); }
        if a.is_none() && cmd_button(ui, "subtract", "Face") { a = Some(MenuAction::ModifyDeleteFace); }
        if a.is_none() && cmd_button(ui, "rectangle", "Face") { a = Some(MenuAction::ModifyReplaceFace); }
        a
    });}
    action
}

/// Sheet Metal ribbon tab.
fn render_sheetmetal_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Base", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "box", "Base Flange") { a = Some(MenuAction::SmBaseFlange); }
        if a.is_none() && cmd_button(ui, "rectangle", "Edge Flange") { a = Some(MenuAction::SmEdgeFlange); }
        a
    });}
    if action.is_none() { action = group(ui, "Bends", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "fillet", "Bend") { a = Some(MenuAction::SmBend); }
        if a.is_none() && cmd_button(ui, "chamfer", "Hem") { a = Some(MenuAction::SmHem); }
        if a.is_none() && cmd_button(ui, "line", "Jog") { a = Some(MenuAction::SmJog); }
        a
    });}
    if action.is_none() { action = group(ui, "Flatten", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "layers", "Unfold") { a = Some(MenuAction::SmUnfold); }
        if a.is_none() && cmd_button(ui, "folds", "Fold") { a = Some(MenuAction::SmFold); }
        if a.is_none() && cmd_button(ui, "rectangle", "Pattern") { a = Some(MenuAction::SmFlatPattern); }
        if a.is_none() && cmd_button(ui, "export", "DXF") { a = Some(MenuAction::SmExportDxf); }
        a
    });}
    if action.is_none() { action = group(ui, "Material", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "grid", "Table") { a = Some(MenuAction::SmGaugeTable); }
        a
    });}
    action
}

/// Assembly ribbon tab.
fn render_assembly_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Components", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "asm_add", "Insert") { a = Some(MenuAction::AsmAddComponent); }
        a
    });}
    if action.is_none() { action = group(ui, "Mate", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "point", "Coincident") { a = Some(MenuAction::AsmMateCoincident); }
        if a.is_none() && cmd_button(ui, "circle", "Concentric") { a = Some(MenuAction::AsmMateConcentric); }
        a
    });}
    if action.is_none() { action = group(ui, "Solve", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "sim_solve", "Solve") { a = Some(MenuAction::AsmSolve); }
        if a.is_none() && cmd_button(ui, "search", "Diagnose") { a = Some(MenuAction::AsmDiagnostics); }
        a
    });}
    if action.is_none() { action = group(ui, "Explode", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "asm_explode", "Explode") { a = Some(MenuAction::AsmExplode); }
        if a.is_none() && cmd_button(ui, "rotate", "Motion") { a = Some(MenuAction::AsmMotion); }
        a
    });}
    if action.is_none() { action = group(ui, "BOM", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "clipboard", "BOM") { a = Some(MenuAction::AsmBom); }
        a
    });}
    action
}

/// CAM ribbon tab.
fn render_cam_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Setup", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "box", "Stock") { a = Some(MenuAction::CamStockSetup); }
        if a.is_none() && cmd_button(ui, "point", "Origin") { a = Some(MenuAction::CamCoordinateSystem); }
        a
    });}
    if action.is_none() { action = group(ui, "Tools", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "settings", "Library") { a = Some(MenuAction::CamToolLibrary); }
        a
    });}
    if action.is_none() { action = group(ui, "2.5 Axis", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "rectangle", "Facing") { a = Some(MenuAction::CamFacing); }
        if a.is_none() && cmd_button(ui, "line", "Profile") { a = Some(MenuAction::CamProfile); }
        if a.is_none() && cmd_button(ui, "subtract", "Pocket") { a = Some(MenuAction::CamPocket); }
        if a.is_none() && cmd_button(ui, "cylinder", "Drilling") { a = Some(MenuAction::CamDrilling); }
        a
    });}
    if action.is_none() { action = group(ui, "Simulate", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "sim_solve", "Sim 2D") { a = Some(MenuAction::CamSim2d); }
        if a.is_none() && cmd_button(ui, "sim_solve", "Sim 3D") { a = Some(MenuAction::CamSim3d); }
        a
    });}
    if action.is_none() { action = group(ui, "Post", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "script", "GCode") { a = Some(MenuAction::CamPostFanuc); }
        a
    });}
    action
}

/// Drawing ribbon tab.
fn render_drawing_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Sheet", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "new", "New Sheet") { a = Some(MenuAction::DrwNewSheet); }
        a
    });}
    if action.is_none() { action = group(ui, "Views", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "drawing", "Standard") { a = Some(MenuAction::DrwViewStandard); }
        if a.is_none() && cmd_button(ui, "rectangle", "Section") { a = Some(MenuAction::DrwViewSection); }
        if a.is_none() && cmd_button(ui, "search", "Detail") { a = Some(MenuAction::DrwViewDetail); }
        if a.is_none() && cmd_button(ui, "asm_explode", "Exploded") { a = Some(MenuAction::DrwViewExploded); }
        a
    });}
    if action.is_none() { action = group(ui, "Dimensions", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "dimension", "Linear") { a = Some(MenuAction::DrwDimLinear); }
        if a.is_none() && cmd_button(ui, "arc", "Angular") { a = Some(MenuAction::DrwDimAngular); }
        if a.is_none() && cmd_button(ui, "circle", "Diameter") { a = Some(MenuAction::DrwDimDiameter); }
        a
    });}
    if action.is_none() { action = group(ui, "Annotations", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "clipboard", "Note") { a = Some(MenuAction::DrwAnnotationNote); }
        if a.is_none() && cmd_button(ui, "circle", "Balloon") { a = Some(MenuAction::DrwAnnotationBalloon); }
        a
    });}
    if action.is_none() { action = group(ui, "Export", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "print", "PDF") { a = Some(MenuAction::DrwExportPdf); }
        if a.is_none() && cmd_button(ui, "dimension", "DXF") { a = Some(MenuAction::DrwExportDxf); }
        a
    });}
    action
}

/// Simulation ribbon tab.
fn render_simulation_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Setup", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "sim_mesh", "Mesh") { a = Some(MenuAction::SimMesh); }
        a
    });}
    if action.is_none() { action = group(ui, "Study", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "box", "Static") { a = Some(MenuAction::SimStudyStatic); }
        if a.is_none() && cmd_button(ui, "rotate", "Modal") { a = Some(MenuAction::SimStudyModal); }
        if a.is_none() && cmd_button(ui, "sphere", "Thermal") { a = Some(MenuAction::SimStudyThermal); }
        if a.is_none() && cmd_button(ui, "pattern_circular", "Buckling") { a = Some(MenuAction::SimStudyBuckling); }
        a
    });}
    if action.is_none() { action = group(ui, "Run", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "sim_solve", "Solve") { a = Some(MenuAction::SimSolve); }
        if a.is_none() && cmd_button(ui, "ai_review", "Validate") { a = Some(MenuAction::SimValidate); }
        a
    });}
    if action.is_none() { action = group(ui, "Results", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "sim_stress", "Stress") { a = Some(MenuAction::SimResultsVonMises); }
        if a.is_none() && cmd_button(ui, "move", "Displacement") { a = Some(MenuAction::SimResultsDisplacement); }
        if a.is_none() && cmd_button(ui, "sim_solve", "Animate") { a = Some(MenuAction::SimAnimate); }
        a
    });}
    action
}

/// Inspect ribbon tab.
fn render_inspect_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Measure", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "dimension", "Distance") { a = Some(MenuAction::MeasureDistance); }
        if a.is_none() && cmd_button(ui, "arc", "Angle") { a = Some(MenuAction::MeasureAngle); }
        if a.is_none() && cmd_button(ui, "ruler", "Length") { a = Some(MenuAction::MeasureLength); }
        if a.is_none() && cmd_button(ui, "rectangle", "Area") { a = Some(MenuAction::MeasureArea); }
        if a.is_none() && cmd_button(ui, "box", "Volume") { a = Some(MenuAction::MeasureVolume); }
        a
    });}
    if action.is_none() { action = group(ui, "Analysis", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "ai_review", "Watertight") { a = Some(MenuAction::AnalysisWatertight); }
        if a.is_none() && cmd_button(ui, "grid", "Manifold") { a = Some(MenuAction::AnalysisManifold); }
        if a.is_none() && cmd_button(ui, "arc", "Curvature") { a = Some(MenuAction::AnalysisCurvature); }
        if a.is_none() && cmd_button(ui, "rotate", "Draft") { a = Some(MenuAction::AnalysisDraft); }
        if a.is_none() && cmd_button(ui, "scale", "Thickness") { a = Some(MenuAction::AnalysisThickness); }
        a
    });}
    if action.is_none() { action = group(ui, "Tools", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "heal", "Heal") { a = Some(MenuAction::HealStitch); }
        a
    });}
    action
}

/// AI ribbon tab.
fn render_ai_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Generate", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "ai_shape", "Shape") { a = Some(MenuAction::AiShapeFromText); }
        if a.is_none() && cmd_button(ui, "box", "Variant A") { a = Some(MenuAction::AiGenVariantA); }
        if a.is_none() && cmd_button(ui, "sphere", "Variant B") { a = Some(MenuAction::AiGenVariantB); }
        if a.is_none() && cmd_button(ui, "cylinder", "Variant C") { a = Some(MenuAction::AiGenVariantC); }
        if a.is_none() && cmd_button(ui, "cone", "Variant D") { a = Some(MenuAction::AiGenVariantD); }
        a
    });}
    if action.is_none() { action = group(ui, "Optimize", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "scale", "Lightweight") { a = Some(MenuAction::AiOptLightweight); }
        if a.is_none() && cmd_button(ui, "box", "Stiff") { a = Some(MenuAction::AiOptStiff); }
        if a.is_none() && cmd_button(ui, "fit", "Balanced") { a = Some(MenuAction::AiOptBalanced); }
        a
    });}
    if action.is_none() { action = group(ui, "Assistant", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "ai_chat", "Chat") { a = Some(MenuAction::AiChat); }
        if a.is_none() && cmd_button(ui, "ai_review", "Review") { a = Some(MenuAction::AiDesignReview); }
        if a.is_none() && cmd_button(ui, "dimension", "Cost") { a = Some(MenuAction::AiCostEstimate); }
        a
    });}
    if action.is_none() { action = group(ui, "Smart", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "fillet", "Auto-Fillet") { a = Some(MenuAction::AiAutoFillet); }
        if a.is_none() && cmd_button(ui, "heal", "Auto-Repair") { a = Some(MenuAction::AiAutoRepair); }
        if a.is_none() && cmd_button(ui, "dimension", "Auto-Dim") { a = Some(MenuAction::AiAutoDimension); }
        a
    });}
    action
}

/// Tools ribbon tab.
fn render_tools_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Application", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "settings", "Options") { a = Some(MenuAction::ToolsOptions); }
        if a.is_none() && cmd_button(ui, "settings", "Customize") { a = Some(MenuAction::ToolsCustomize); }
        if a.is_none() && cmd_button(ui, "asm_add", "Plugins") { a = Some(MenuAction::ToolsPlugins); }
        if a.is_none() && cmd_button(ui, "settings", "Theme") { a = Some(MenuAction::ToolsTheme); }
        a
    });}
    if action.is_none() { action = group(ui, "Scripting", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "script", "Console") { a = Some(MenuAction::ToolsScriptingConsole); }
        if a.is_none() && cmd_button(ui, "script", "Macro") { a = Some(MenuAction::ToolsMacroRecorder); }
        a
    });}
    if action.is_none() { action = group(ui, "Performance", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "grid", "Monitor") { a = Some(MenuAction::ToolsPerformance); }
        a
    });}
    if action.is_none() { action = group(ui, "UI", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "layers", "Layout") { a = Some(MenuAction::ToolsUiLayout); }
        a
    });}
    action
}

/// View ribbon tab.
fn render_view_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Orient", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "iso", "ISO") { a = Some(MenuAction::ViewIso); }
        if a.is_none() && cmd_button(ui, "cube", "Front") { a = Some(MenuAction::ViewFront); }
        if a.is_none() && cmd_button(ui, "cube", "Top") { a = Some(MenuAction::ViewTop); }
        if a.is_none() && cmd_button(ui, "cube", "Right") { a = Some(MenuAction::ViewRight); }
        a
    });}
    if action.is_none() { action = group(ui, "Zoom", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "fit", "Fit") { a = Some(MenuAction::ViewFit); }
        if a.is_none() && cmd_button(ui, "zoom_in", "In") { a = Some(MenuAction::ViewZoomIn); }
        if a.is_none() && cmd_button(ui, "zoom_out", "Out") { a = Some(MenuAction::ViewZoomOut); }
        a
    });}
    if action.is_none() { action = group(ui, "Style", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "box", "Wireframe") { a = Some(MenuAction::ViewWireframe); }
        if a.is_none() && cmd_button(ui, "sphere", "Shaded") { a = Some(MenuAction::ViewShaded); }
        if a.is_none() && cmd_button(ui, "box", "Edges") { a = Some(MenuAction::ViewShadedEdges); }
        a
    });}
    if action.is_none() { action = group(ui, "Camera", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "cube", "Perspective") { a = Some(MenuAction::ViewPerspective); }
        if a.is_none() && cmd_button(ui, "box", "Orthographic") { a = Some(MenuAction::ViewOrthographic); }
        a
    });}
    if action.is_none() { action = group(ui, "Layouts", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "save", "Save Layout") { a = Some(MenuAction::ViewSaveLayout); }
        a
    });}
    action
}

/// Surface ribbon tab.
fn render_surface_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Create", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "layers", "Loft") { a = Some(MenuAction::ModifyLoft); }
        if a.is_none() && cmd_button(ui, "move", "Sweep") { a = Some(MenuAction::ModifySweep); }
        a
    });}
    action
}
