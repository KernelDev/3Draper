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

/// Helper: render a command button with icon on top, label below.
fn cmd_button(ui: &mut egui::Ui, icon: &str, label: &str) -> bool {
    let button = egui::Button::new(
        egui::RichText::new(format!("{}\n{}", icon, label))
            .small()
    );
    ui.add_sized([55.0, 45.0], button).clicked()
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
        if a.is_none() && cmd_button(ui, "📄", "New") { a = Some(MenuAction::FileNew); }
        if a.is_none() && cmd_button(ui, "📂", "Open") { a = Some(MenuAction::FileOpen); }
        if a.is_none() && cmd_button(ui, "💾", "Save") { a = Some(MenuAction::FileSave); }
        a
    });}
    if action.is_none() { action = group(ui, "Import", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📥", "STEP") { a = Some(MenuAction::FileImportStep); }
        if a.is_none() && cmd_button(ui, "📥", "STL") { a = Some(MenuAction::FileImportStl); }
        a
    });}
    if action.is_none() { action = group(ui, "Export", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📤", "STEP") { a = Some(MenuAction::FileExportStep); }
        if a.is_none() && cmd_button(ui, "📤", "STL") { a = Some(MenuAction::FileExportStl); }
        if a.is_none() && cmd_button(ui, "📤", "OBJ") { a = Some(MenuAction::FileExportObj); }
        a
    });}
    if action.is_none() { action = group(ui, "Render", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "🎬", "POV") { a = Some(MenuAction::FilePrint); }
        if a.is_none() && cmd_button(ui, "🎬", "Blender") { a = Some(MenuAction::FileExportObj); }
        a
    });}
    action
}

/// Home ribbon tab.
fn render_home_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Clipboard", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "✂", "Cut") { a = Some(MenuAction::EditCut); }
        if a.is_none() && cmd_button(ui, "📋", "Copy") { a = Some(MenuAction::EditCopy); }
        if a.is_none() && cmd_button(ui, "📌", "Paste") { a = Some(MenuAction::EditPaste); }
        a
    });}
    if action.is_none() { action = group(ui, "History", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "↶", "Undo") { a = Some(MenuAction::EditUndo); }
        if a.is_none() && cmd_button(ui, "↷", "Redo") { a = Some(MenuAction::EditRedo); }
        a
    });}
    if action.is_none() { action = group(ui, "View", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "🔲", "Fit") { a = Some(MenuAction::ViewFit); }
        if a.is_none() && cmd_button(ui, "🏠", "ISO") { a = Some(MenuAction::ViewIso); }
        a
    });}
    if action.is_none() { action = group(ui, "Display", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, " Wire", "Wireframe") { a = Some(MenuAction::ViewWireframe); }
        if a.is_none() && cmd_button(ui, " Solid", "Shaded") { a = Some(MenuAction::ViewShaded); }
        if a.is_none() && cmd_button(ui, " Both", "Shaded+") { a = Some(MenuAction::ViewShadedEdges); }
        a
    });}
    action
}

/// Sketch ribbon tab.
fn render_sketch_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Mode", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "✏", "Sketch") { a = Some(MenuAction::SketchEnter); }
        if a.is_none() && cmd_button(ui, "✓", "Exit") { a = Some(MenuAction::SketchExit); }
        a
    });}
    if action.is_none() { action = group(ui, "Draw", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "／", "Line") { a = Some(MenuAction::SketchLine); }
        if a.is_none() && cmd_button(ui, "○", "Circle") { a = Some(MenuAction::SketchCircle); }
        if a.is_none() && cmd_button(ui, "◞", "Arc") { a = Some(MenuAction::SketchArc3); }
        if a.is_none() && cmd_button(ui, "▭", "Rect") { a = Some(MenuAction::SketchRectangle); }
        if a.is_none() && cmd_button(ui, "〰", "Spline") { a = Some(MenuAction::SketchSpline); }
        a
    });}
    if action.is_none() { action = group(ui, "Constraint", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "●", "Coincident") { a = Some(MenuAction::SketchConstraintCoincident); }
        if a.is_none() && cmd_button(ui, "∥", "Parallel") { a = Some(MenuAction::SketchConstraintParallel); }
        if a.is_none() && cmd_button(ui, "⊥", "Perpendicular") { a = Some(MenuAction::SketchConstraintPerpendicular); }
        if a.is_none() && cmd_button(ui, "⌒", "Tangent") { a = Some(MenuAction::SketchConstraintTangent); }
        if a.is_none() && cmd_button(ui, "↔", "Horizontal") { a = Some(MenuAction::SketchConstraintHorizontal); }
        if a.is_none() && cmd_button(ui, "↕", "Vertical") { a = Some(MenuAction::SketchConstraintVertical); }
        a
    });}
    if action.is_none() { action = group(ui, "Dimension", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "↔", "Linear") { a = Some(MenuAction::SketchDimLinear); }
        if a.is_none() && cmd_button(ui, "∠", "Angular") { a = Some(MenuAction::SketchDimAngular); }
        if a.is_none() && cmd_button(ui, "⌀", "Radial") { a = Some(MenuAction::SketchDimRadial); }
        a
    });}
    action
}

/// Insert ribbon tab.
fn render_insert_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Primitives", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "▭", "Box") { a = Some(MenuAction::InsertBox); }
        if a.is_none() && cmd_button(ui, "◯", "Sphere") { a = Some(MenuAction::InsertSphere); }
        if a.is_none() && cmd_button(ui, "⬭", "Cylinder") { a = Some(MenuAction::InsertCylinder); }
        if a.is_none() && cmd_button(ui, "△", "Cone") { a = Some(MenuAction::InsertCone); }
        if a.is_none() && cmd_button(ui, "◎", "Torus") { a = Some(MenuAction::InsertTorus); }
        a
    });}
    if action.is_none() { action = group(ui, "Reference", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "▱", "Plane") { a = Some(MenuAction::InsertPlane); }
        if a.is_none() && cmd_button(ui, "─", "Axis") { a = Some(MenuAction::InsertAxis); }
        if a.is_none() && cmd_button(ui, "•", "Point") { a = Some(MenuAction::InsertPoint); }
        a
    });}
    if action.is_none() { action = group(ui, "Mesh", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📥", "Import") { a = Some(MenuAction::InsertMesh); }
        if a.is_none() && cmd_button(ui, "🔄", "Remesh") { a = Some(MenuAction::InsertRemesh); }
        a
    });}
    if action.is_none() { action = group(ui, "Pattern", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "⫼", "Linear") { a = Some(MenuAction::InsertLinearPattern); }
        if a.is_none() && cmd_button(ui, "⊙", "Circular") { a = Some(MenuAction::InsertCircularPattern); }
        if a.is_none() && cmd_button(ui, "⇋", "Mirror") { a = Some(MenuAction::InsertMirror); }
        a
    });}
    action
}

/// Modify ribbon tab.
fn render_modify_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Boolean", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "∪", "Union") { a = Some(MenuAction::ModifyUnion); }
        if a.is_none() && cmd_button(ui, "∖", "Subtract") { a = Some(MenuAction::ModifySubtract); }
        if a.is_none() && cmd_button(ui, "∩", "Intersect") { a = Some(MenuAction::ModifyIntersect); }
        a
    });}
    if action.is_none() { action = group(ui, "Edge", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "◜", "Fillet") { a = Some(MenuAction::ModifyFillet); }
        if a.is_none() && cmd_button(ui, "◹", "Chamfer") { a = Some(MenuAction::ModifyChamfer); }
        a
    });}
    if action.is_none() { action = group(ui, "Transform", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "↗", "Move") { a = Some(MenuAction::ModifyMove); }
        if a.is_none() && cmd_button(ui, "↻", "Rotate") { a = Some(MenuAction::ModifyRotate); }
        if a.is_none() && cmd_button(ui, "⤢", "Scale") { a = Some(MenuAction::ModifyScale); }
        a
    });}
    if action.is_none() { action = group(ui, "Pattern", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "⫼", "Linear") { a = Some(MenuAction::ModifyLinearPattern); }
        if a.is_none() && cmd_button(ui, "⊙", "Circular") { a = Some(MenuAction::ModifyCircularPattern); }
        if a.is_none() && cmd_button(ui, "⇋", "Mirror") { a = Some(MenuAction::ModifyMirror); }
        a
    });}
    if action.is_none() { action = group(ui, "Direct", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Move", "Face") { a = Some(MenuAction::ModifyMoveFace); }
        if a.is_none() && cmd_button(ui, "Offset", "Face") { a = Some(MenuAction::ModifyOffsetFace); }
        if a.is_none() && cmd_button(ui, "Delete", "Face") { a = Some(MenuAction::ModifyDeleteFace); }
        if a.is_none() && cmd_button(ui, "Replace", "Face") { a = Some(MenuAction::ModifyReplaceFace); }
        a
    });}
    action
}

/// Sheet Metal ribbon tab.
fn render_sheetmetal_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Base", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "▭", "Base Flange") { a = Some(MenuAction::SmBaseFlange); }
        if a.is_none() && cmd_button(ui, "▭", "Edge Flange") { a = Some(MenuAction::SmEdgeFlange); }
        a
    });}
    if action.is_none() { action = group(ui, "Bends", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "◜", "Bend") { a = Some(MenuAction::SmBend); }
        if a.is_none() && cmd_button(ui, "◟", "Hem") { a = Some(MenuAction::SmHem); }
        if a.is_none() && cmd_button(ui, "∿", "Jog") { a = Some(MenuAction::SmJog); }
        a
    });}
    if action.is_none() { action = group(ui, "Flatten", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, " unfolds", "Unfold") { a = Some(MenuAction::SmUnfold); }
        if a.is_none() && cmd_button(ui, "folds", "Fold") { a = Some(MenuAction::SmFold); }
        if a.is_none() && cmd_button(ui, "Flat", "Pattern") { a = Some(MenuAction::SmFlatPattern); }
        if a.is_none() && cmd_button(ui, "📤", "DXF") { a = Some(MenuAction::SmExportDxf); }
        a
    });}
    if action.is_none() { action = group(ui, "Material", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Gauge", "Table") { a = Some(MenuAction::SmGaugeTable); }
        a
    });}
    action
}

/// Assembly ribbon tab.
fn render_assembly_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Components", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "➕", "Insert") { a = Some(MenuAction::AsmAddComponent); }
        a
    });}
    if action.is_none() { action = group(ui, "Mate", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "≡", "Coincident") { a = Some(MenuAction::AsmMateCoincident); }
        if a.is_none() && cmd_button(ui, "◎", "Concentric") { a = Some(MenuAction::AsmMateConcentric); }
        a
    });}
    if action.is_none() { action = group(ui, "Solve", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "▶", "Solve") { a = Some(MenuAction::AsmSolve); }
        if a.is_none() && cmd_button(ui, "🔍", "Diagnose") { a = Some(MenuAction::AsmDiagnostics); }
        a
    });}
    if action.is_none() { action = group(ui, "Explode", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "💥", "Explode") { a = Some(MenuAction::AsmExplode); }
        if a.is_none() && cmd_button(ui, "🎥", "Motion") { a = Some(MenuAction::AsmMotion); }
        a
    });}
    if action.is_none() { action = group(ui, "BOM", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📋", "BOM") { a = Some(MenuAction::AsmBom); }
        a
    });}
    action
}

/// CAM ribbon tab.
fn render_cam_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Setup", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📦", "Stock") { a = Some(MenuAction::CamStockSetup); }
        if a.is_none() && cmd_button(ui, "📍", "Origin") { a = Some(MenuAction::CamCoordinateSystem); }
        a
    });}
    if action.is_none() { action = group(ui, "Tools", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "🔧", "Library") { a = Some(MenuAction::CamToolLibrary); }
        a
    });}
    if action.is_none() { action = group(ui, "2.5 Axis", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Face", "Facing") { a = Some(MenuAction::CamFacing); }
        if a.is_none() && cmd_button(ui, "Profile", "Profile") { a = Some(MenuAction::CamProfile); }
        if a.is_none() && cmd_button(ui, "Pocket", "Pocket") { a = Some(MenuAction::CamPocket); }
        if a.is_none() && cmd_button(ui, "Drill", "Drilling") { a = Some(MenuAction::CamDrilling); }
        a
    });}
    if action.is_none() { action = group(ui, "Simulate", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "▶", "Sim 2D") { a = Some(MenuAction::CamSim2d); }
        if a.is_none() && cmd_button(ui, "▶", "Sim 3D") { a = Some(MenuAction::CamSim3d); }
        a
    });}
    if action.is_none() { action = group(ui, "Post", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "G", "GCode") { a = Some(MenuAction::CamPostFanuc); }
        a
    });}
    action
}

/// Drawing ribbon tab.
fn render_drawing_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Sheet", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📄", "New Sheet") { a = Some(MenuAction::DrwNewSheet); }
        a
    });}
    if action.is_none() { action = group(ui, "Views", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Std", "Standard") { a = Some(MenuAction::DrwViewStandard); }
        if a.is_none() && cmd_button(ui, "Section", "Section") { a = Some(MenuAction::DrwViewSection); }
        if a.is_none() && cmd_button(ui, "Detail", "Detail") { a = Some(MenuAction::DrwViewDetail); }
        if a.is_none() && cmd_button(ui, "Exploded", "Exploded") { a = Some(MenuAction::DrwViewExploded); }
        a
    });}
    if action.is_none() { action = group(ui, "Dimensions", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "↔", "Linear") { a = Some(MenuAction::DrwDimLinear); }
        if a.is_none() && cmd_button(ui, "∠", "Angular") { a = Some(MenuAction::DrwDimAngular); }
        if a.is_none() && cmd_button(ui, "⌀", "Diameter") { a = Some(MenuAction::DrwDimDiameter); }
        a
    });}
    if action.is_none() { action = group(ui, "Annotations", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📝", "Note") { a = Some(MenuAction::DrwAnnotationNote); }
        if a.is_none() && cmd_button(ui, "🎈", "Balloon") { a = Some(MenuAction::DrwAnnotationBalloon); }
        a
    });}
    if action.is_none() { action = group(ui, "Export", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📄", "PDF") { a = Some(MenuAction::DrwExportPdf); }
        if a.is_none() && cmd_button(ui, "📐", "DXF") { a = Some(MenuAction::DrwExportDxf); }
        a
    });}
    action
}

/// Simulation ribbon tab.
fn render_simulation_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Setup", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Mesh", "Mesh") { a = Some(MenuAction::SimMesh); }
        a
    });}
    if action.is_none() { action = group(ui, "Study", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Static", "Static") { a = Some(MenuAction::SimStudyStatic); }
        if a.is_none() && cmd_button(ui, "Modal", "Modal") { a = Some(MenuAction::SimStudyModal); }
        if a.is_none() && cmd_button(ui, "Therm", "Thermal") { a = Some(MenuAction::SimStudyThermal); }
        if a.is_none() && cmd_button(ui, "Buckle", "Buckling") { a = Some(MenuAction::SimStudyBuckling); }
        a
    });}
    if action.is_none() { action = group(ui, "Run", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "▶", "Solve") { a = Some(MenuAction::SimSolve); }
        if a.is_none() && cmd_button(ui, "✓", "Validate") { a = Some(MenuAction::SimValidate); }
        a
    });}
    if action.is_none() { action = group(ui, "Results", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "σ", "Stress") { a = Some(MenuAction::SimResultsVonMises); }
        if a.is_none() && cmd_button(ui, "δ", "Displacement") { a = Some(MenuAction::SimResultsDisplacement); }
        if a.is_none() && cmd_button(ui, "▶", "Animate") { a = Some(MenuAction::SimAnimate); }
        a
    });}
    action
}

/// Inspect ribbon tab.
fn render_inspect_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Measure", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "↔", "Distance") { a = Some(MenuAction::MeasureDistance); }
        if a.is_none() && cmd_button(ui, "∠", "Angle") { a = Some(MenuAction::MeasureAngle); }
        if a.is_none() && cmd_button(ui, "━", "Length") { a = Some(MenuAction::MeasureLength); }
        if a.is_none() && cmd_button(ui, "▢", "Area") { a = Some(MenuAction::MeasureArea); }
        if a.is_none() && cmd_button(ui, "Cube", "Volume") { a = Some(MenuAction::MeasureVolume); }
        a
    });}
    if action.is_none() { action = group(ui, "Analysis", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "WT", "Watertight") { a = Some(MenuAction::AnalysisWatertight); }
        if a.is_none() && cmd_button(ui, "M", "Manifold") { a = Some(MenuAction::AnalysisManifold); }
        if a.is_none() && cmd_button(ui, "κ", "Curvature") { a = Some(MenuAction::AnalysisCurvature); }
        if a.is_none() && cmd_button(ui, "Draft", "Draft") { a = Some(MenuAction::AnalysisDraft); }
        if a.is_none() && cmd_button(ui, "Thick", "Thickness") { a = Some(MenuAction::AnalysisThickness); }
        a
    });}
    if action.is_none() { action = group(ui, "Tools", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Heal", "Heal") { a = Some(MenuAction::HealStitch); }
        a
    });}
    action
}

/// AI ribbon tab.
fn render_ai_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Generate", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Text→3D", "Shape") { a = Some(MenuAction::AiShapeFromText); }
        if a.is_none() && cmd_button(ui, "A", "Variant A") { a = Some(MenuAction::AiGenVariantA); }
        if a.is_none() && cmd_button(ui, "B", "Variant B") { a = Some(MenuAction::AiGenVariantB); }
        if a.is_none() && cmd_button(ui, "C", "Variant C") { a = Some(MenuAction::AiGenVariantC); }
        if a.is_none() && cmd_button(ui, "D", "Variant D") { a = Some(MenuAction::AiGenVariantD); }
        a
    });}
    if action.is_none() { action = group(ui, "Optimize", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Light", "Lightweight") { a = Some(MenuAction::AiOptLightweight); }
        if a.is_none() && cmd_button(ui, "Stiff", "Stiff") { a = Some(MenuAction::AiOptStiff); }
        if a.is_none() && cmd_button(ui, "Balance", "Balanced") { a = Some(MenuAction::AiOptBalanced); }
        a
    });}
    if action.is_none() { action = group(ui, "Assistant", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "💬", "Chat") { a = Some(MenuAction::AiChat); }
        if a.is_none() && cmd_button(ui, "DRC", "Review") { a = Some(MenuAction::AiDesignReview); }
        if a.is_none() && cmd_button(ui, "€", "Cost") { a = Some(MenuAction::AiCostEstimate); }
        a
    });}
    if action.is_none() { action = group(ui, "Smart", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "AutoF", "Auto-Fillet") { a = Some(MenuAction::AiAutoFillet); }
        if a.is_none() && cmd_button(ui, "AutoR", "Auto-Repair") { a = Some(MenuAction::AiAutoRepair); }
        if a.is_none() && cmd_button(ui, "AutoD", "Auto-Dim") { a = Some(MenuAction::AiAutoDimension); }
        a
    });}
    action
}

/// Tools ribbon tab.
fn render_tools_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Application", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "⚙", "Options") { a = Some(MenuAction::ToolsOptions); }
        if a.is_none() && cmd_button(ui, "🎨", "Customize") { a = Some(MenuAction::ToolsCustomize); }
        if a.is_none() && cmd_button(ui, "🔌", "Plugins") { a = Some(MenuAction::ToolsPlugins); }
        if a.is_none() && cmd_button(ui, "🎨", "Theme") { a = Some(MenuAction::ToolsTheme); }
        a
    });}
    if action.is_none() { action = group(ui, "Scripting", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "🐍", "Console") { a = Some(MenuAction::ToolsScriptingConsole); }
        if a.is_none() && cmd_button(ui, "⏺", "Macro") { a = Some(MenuAction::ToolsMacroRecorder); }
        a
    });}
    if action.is_none() { action = group(ui, "Performance", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "📊", "Monitor") { a = Some(MenuAction::ToolsPerformance); }
        a
    });}
    if action.is_none() { action = group(ui, "UI", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Layout", "Layout") { a = Some(MenuAction::ToolsUiLayout); }
        a
    });}
    action
}

/// View ribbon tab.
fn render_view_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Orient", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "ISO", "ISO") { a = Some(MenuAction::ViewIso); }
        if a.is_none() && cmd_button(ui, "Front", "Front") { a = Some(MenuAction::ViewFront); }
        if a.is_none() && cmd_button(ui, "Top", "Top") { a = Some(MenuAction::ViewTop); }
        if a.is_none() && cmd_button(ui, "Right", "Right") { a = Some(MenuAction::ViewRight); }
        a
    });}
    if action.is_none() { action = group(ui, "Zoom", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Fit", "Fit") { a = Some(MenuAction::ViewFit); }
        if a.is_none() && cmd_button(ui, "In", "In") { a = Some(MenuAction::ViewZoomIn); }
        if a.is_none() && cmd_button(ui, "Out", "Out") { a = Some(MenuAction::ViewZoomOut); }
        a
    });}
    if action.is_none() { action = group(ui, "Style", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Wire", "Wireframe") { a = Some(MenuAction::ViewWireframe); }
        if a.is_none() && cmd_button(ui, "Solid", "Shaded") { a = Some(MenuAction::ViewShaded); }
        if a.is_none() && cmd_button(ui, "Both", "Edges") { a = Some(MenuAction::ViewShadedEdges); }
        a
    });}
    if action.is_none() { action = group(ui, "Camera", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Persp", "Perspective") { a = Some(MenuAction::ViewPerspective); }
        if a.is_none() && cmd_button(ui, "Ortho", "Orthographic") { a = Some(MenuAction::ViewOrthographic); }
        a
    });}
    if action.is_none() { action = group(ui, "Layouts", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Save", "Save Layout") { a = Some(MenuAction::ViewSaveLayout); }
        a
    });}
    action
}

/// Surface ribbon tab.
fn render_surface_ribbon(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    if action.is_none() { action = group(ui, "Create", |ui| {
        let mut a = None;
        if a.is_none() && cmd_button(ui, "Loft", "Loft") { a = Some(MenuAction::ModifyLoft); }
        if a.is_none() && cmd_button(ui, "Sweep", "Sweep") { a = Some(MenuAction::ModifySweep); }
        a
    });}
    action
}
