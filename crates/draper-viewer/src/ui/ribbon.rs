// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Ribbon tabs — 15 tabs per ROADMAP_UI Phase 2.
//!
//! Each tab contains grouped command buttons. The ribbon is rendered
//! as a horizontal bar below the menu bar.

use eframe::egui;
use super::Workspace;

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

/// Render the ribbon bar. Returns the active tab.
pub fn render_ribbon(ctx: &egui::Context, active: &mut RibbonTab) {
    egui::TopBottomPanel::top("ribbon_bar")
        .exact_height(90.0)
        .show(ctx, |ui| {
            // Tab selector row — horizontal, no wrapping
            ui.horizontal(|ui| {
                for tab in RibbonTab::ALL {
                    if ui.selectable_label(*active == *tab, tab.label()).clicked() {
                        *active = *tab;
                    }
                }
            });

            ui.separator();

            // Content row — groups laid out horizontally, scrollable
            egui::ScrollArea::horizontal().show(ui, |ui| {
                ui.horizontal(|ui| {
                    match active {
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
                    }
                });
            });
        });
}

/// Helper: render a button group with a label at the bottom.
/// Groups are laid out horizontally (left to right) within the ribbon.
/// Buttons within a group are also horizontal.
fn group(ui: &mut egui::Ui, label: &str, content: impl FnOnce(&mut egui::Ui)) {
    ui.vertical(|ui| {
        // Buttons row — horizontal
        ui.horizontal(|ui| {
            content(ui);
        });
        // Group label at the bottom
        ui.label(egui::RichText::new(label).small().color(egui::Color32::GRAY));
    });
    ui.separator();
}

/// Helper: render a command button with icon on top, label below.
fn cmd_button(ui: &mut egui::Ui, icon: &str, label: &str) -> bool {
    let button = egui::Button::new(
        egui::RichText::new(format!("{}\n{}", icon, label))
            .small()
    );
    ui.add_sized([55.0, 45.0], button).clicked()
}

/// File ribbon tab.
fn render_file_ribbon(ui: &mut egui::Ui) {
    group(ui, "Document", |ui| {
        if cmd_button(ui, "📄", "New") {}
        if cmd_button(ui, "📂", "Open") {}
        if cmd_button(ui, "💾", "Save") {}
    });
    group(ui, "Import", |ui| {
        if cmd_button(ui, "📥", "STEP") {}
        if cmd_button(ui, "📥", "STL") {}
    });
    group(ui, "Export", |ui| {
        if cmd_button(ui, "📤", "STEP") {}
        if cmd_button(ui, "📤", "STL") {}
        if cmd_button(ui, "📤", "OBJ") {}
    });
    group(ui, "Render", |ui| {
        if cmd_button(ui, "🎬", "POV") {}
        if cmd_button(ui, "🎬", "Blender") {}
    });
}

/// Home ribbon tab.
fn render_home_ribbon(ui: &mut egui::Ui) {
    group(ui, "Clipboard", |ui| {
        if cmd_button(ui, "✂️", "Cut") {}
        if cmd_button(ui, "📋", "Copy") {}
        if cmd_button(ui, "📌", "Paste") {}
    });
    group(ui, "History", |ui| {
        if cmd_button(ui, "↶", "Undo") {}
        if cmd_button(ui, "↷", "Redo") {}
    });
    group(ui, "View", |ui| {
        if cmd_button(ui, "🔲", "Fit") {}
        if cmd_button(ui, "🏠", "ISO") {}
    });
    group(ui, "Display", |ui| {
        if cmd_button(ui, " Wire", "Wireframe") {}
        if cmd_button(ui, " Solid", "Shaded") {}
        if cmd_button(ui, " Both", "Shaded+") {}
    });
}

/// Sketch ribbon tab.
fn render_sketch_ribbon(ui: &mut egui::Ui) {
    group(ui, "Mode", |ui| {
        if cmd_button(ui, "✏️", "Sketch") {}
        if cmd_button(ui, "✓", "Exit") {}
    });
    group(ui, "Draw", |ui| {
        if cmd_button(ui, "／", "Line") {}
        if cmd_button(ui, "○", "Circle") {}
        if cmd_button(ui, "◞", "Arc") {}
        if cmd_button(ui, "▭", "Rect") {}
        if cmd_button(ui, "〰", "Spline") {}
    });
    group(ui, "Constraint", |ui| {
        if cmd_button(ui, "●", "Coincident") {}
        if cmd_button(ui, "∥", "Parallel") {}
        if cmd_button(ui, "⊥", "Perpendicular") {}
        if cmd_button(ui, "⌒", "Tangent") {}
        if cmd_button(ui, "↔", "Horizontal") {}
        if cmd_button(ui, "↕", "Vertical") {}
    });
    group(ui, "Dimension", |ui| {
        if cmd_button(ui, "↔", "Linear") {}
        if cmd_button(ui, "∠", "Angular") {}
        if cmd_button(ui, "⌀", "Radial") {}
    });
}

/// Insert ribbon tab.
fn render_insert_ribbon(ui: &mut egui::Ui) {
    group(ui, "Primitives", |ui| {
        if cmd_button(ui, "▭", "Box") {}
        if cmd_button(ui, "◯", "Sphere") {}
        if cmd_button(ui, "⬭", "Cylinder") {}
        if cmd_button(ui, "△", "Cone") {}
        if cmd_button(ui, "◎", "Torus") {}
    });
    group(ui, "Reference", |ui| {
        if cmd_button(ui, "▱", "Plane") {}
        if cmd_button(ui, "─", "Axis") {}
        if cmd_button(ui, "•", "Point") {}
    });
    group(ui, "Mesh", |ui| {
        if cmd_button(ui, "📥", "Import") {}
        if cmd_button(ui, "🔄", "Remesh") {}
    });
    group(ui, "Pattern", |ui| {
        if cmd_button(ui, "⫼", "Linear") {}
        if cmd_button(ui, "⊙", "Circular") {}
        if cmd_button(ui, "⇋", "Mirror") {}
    });
}

/// Modify ribbon tab.
fn render_modify_ribbon(ui: &mut egui::Ui) {
    group(ui, "Boolean", |ui| {
        if cmd_button(ui, "∪", "Union") {}
        if cmd_button(ui, "∖", "Subtract") {}
        if cmd_button(ui, "∩", "Intersect") {}
    });
    group(ui, "Edge", |ui| {
        if cmd_button(ui, "◜", "Fillet") {}
        if cmd_button(ui, "◹", "Chamfer") {}
    });
    group(ui, "Transform", |ui| {
        if cmd_button(ui, "↗", "Move") {}
        if cmd_button(ui, "↻", "Rotate") {}
        if cmd_button(ui, "⤢", "Scale") {}
    });
    group(ui, "Pattern", |ui| {
        if cmd_button(ui, "⫼", "Linear") {}
        if cmd_button(ui, "⊙", "Circular") {}
        if cmd_button(ui, "⇋", "Mirror") {}
    });
    group(ui, "Direct", |ui| {
        if cmd_button(ui, "Move", "Face") {}
        if cmd_button(ui, "Offset", "Face") {}
        if cmd_button(ui, "Delete", "Face") {}
        if cmd_button(ui, "Replace", "Face") {}
    });
}

/// Sheet Metal ribbon tab.
fn render_sheetmetal_ribbon(ui: &mut egui::Ui) {
    group(ui, "Base", |ui| {
        if cmd_button(ui, "▭", "Base Flange") {}
        if cmd_button(ui, "▭", "Edge Flange") {}
        if cmd_button(ui, "▭", "Lofted") {}
    });
    group(ui, "Bends", |ui| {
        if cmd_button(ui, "◜", "Bend") {}
        if cmd_button(ui, "◟", "Hem") {}
        if cmd_button(ui, "∿", "Jog") {}
    });
    group(ui, "Flatten", |ui| {
        if cmd_button(ui, " unfolds", "Unfold") {}
        if cmd_button(ui, "folds", "Fold") {}
        if cmd_button(ui, "Flat", "Pattern") {}
        if cmd_button(ui, "📤", "DXF") {}
    });
    group(ui, "Material", |ui| {
        if cmd_button(ui, "Gauge", "Table") {}
        if cmd_button(ui, "K", "Factor") {}
    });
}

/// Assembly ribbon tab.
fn render_assembly_ribbon(ui: &mut egui::Ui) {
    group(ui, "Components", |ui| {
        if cmd_button(ui, "➕", "Insert") {}
        if cmd_button(ui, "🔄", "Replace") {}
    });
    group(ui, "Mate", |ui| {
        if cmd_button(ui, "≡", "Coincident") {}
        if cmd_button(ui, "◎", "Concentric") {}
    });
    group(ui, "Solve", |ui| {
        if cmd_button(ui, "▶", "Solve") {}
        if cmd_button(ui, "🔍", "Diagnose") {}
    });
    group(ui, "Explode", |ui| {
        if cmd_button(ui, "💥", "Explode") {}
        if cmd_button(ui, "🎥", "Motion") {}
    });
    group(ui, "BOM", |ui| {
        if cmd_button(ui, "📋", "BOM") {}
    });
}

/// CAM ribbon tab.
fn render_cam_ribbon(ui: &mut egui::Ui) {
    group(ui, "Setup", |ui| {
        if cmd_button(ui, "📦", "Stock") {}
        if cmd_button(ui, "📍", "Origin") {}
    });
    group(ui, "Tools", |ui| {
        if cmd_button(ui, "🔧", "Library") {}
        if cmd_button(ui, "Flat", "End Mill") {}
        if cmd_button(ui, "Ball", "End Mill") {}
    });
    group(ui, "2.5 Axis", |ui| {
        if cmd_button(ui, "Face", "Facing") {}
        if cmd_button(ui, "Profile", "Profile") {}
        if cmd_button(ui, "Pocket", "Pocket") {}
        if cmd_button(ui, "Drill", "Drilling") {}
    });
    group(ui, "Simulate", |ui| {
        if cmd_button(ui, "▶", "Sim 2D") {}
        if cmd_button(ui, "▶", "Sim 3D") {}
    });
    group(ui, "Post", |ui| {
        if cmd_button(ui, "G", "GCode") {}
        if cmd_button(ui, "📄", "NC View") {}
    });
}

/// Drawing ribbon tab.
fn render_drawing_ribbon(ui: &mut egui::Ui) {
    group(ui, "Sheet", |ui| {
        if cmd_button(ui, "📄", "New Sheet") {}
    });
    group(ui, "Views", |ui| {
        if cmd_button(ui, "Std", "Standard") {}
        if cmd_button(ui, "Section", "Section") {}
        if cmd_button(ui, "Detail", "Detail") {}
        if cmd_button(ui, "Exploded", "Exploded") {}
    });
    group(ui, "Dimensions", |ui| {
        if cmd_button(ui, "↔", "Linear") {}
        if cmd_button(ui, "∠", "Angular") {}
        if cmd_button(ui, "⌀", "Diameter") {}
    });
    group(ui, "Annotations", |ui| {
        if cmd_button(ui, "📝", "Note") {}
        if cmd_button(ui, "🎈", "Balloon") {}
    });
    group(ui, "Export", |ui| {
        if cmd_button(ui, "📄", "PDF") {}
        if cmd_button(ui, "📐", "DXF") {}
    });
}

/// Simulation ribbon tab.
fn render_simulation_ribbon(ui: &mut egui::Ui) {
    group(ui, "Setup", |ui| {
        if cmd_button(ui, "Mesh", "Mesh") {}
        if cmd_button(ui, "Material", "Material") {}
        if cmd_button(ui, "BC", "Boundary") {}
        if cmd_button(ui, "Load", "Load") {}
    });
    group(ui, "Study", |ui| {
        if cmd_button(ui, "Static", "Static") {}
        if cmd_button(ui, "Modal", "Modal") {}
        if cmd_button(ui, "Therm", "Thermal") {}
        if cmd_button(ui, "Buckle", "Buckling") {}
    });
    group(ui, "Run", |ui| {
        if cmd_button(ui, "▶", "Solve") {}
        if cmd_button(ui, "✓", "Validate") {}
    });
    group(ui, "Results", |ui| {
        if cmd_button(ui, "σ", "Stress") {}
        if cmd_button(ui, "δ", "Displacement") {}
        if cmd_button(ui, "▶", "Animate") {}
    });
}

/// Inspect ribbon tab.
fn render_inspect_ribbon(ui: &mut egui::Ui) {
    group(ui, "Measure", |ui| {
        if cmd_button(ui, "↔", "Distance") {}
        if cmd_button(ui, "∠", "Angle") {}
        if cmd_button(ui, "━", "Length") {}
        if cmd_button(ui, "▢", "Area") {}
        if cmd_button(ui, "Cube", "Volume") {}
    });
    group(ui, "Analysis", |ui| {
        if cmd_button(ui, "WT", "Watertight") {}
        if cmd_button(ui, "M", "Manifold") {}
        if cmd_button(ui, "κ", "Curvature") {}
        if cmd_button(ui, "Draft", "Draft") {}
        if cmd_button(ui, "Thick", "Thickness") {}
    });
    group(ui, "Tools", |ui| {
        if cmd_button(ui, "Section", "Section") {}
        if cmd_button(ui, "Compare", "Compare") {}
        if cmd_button(ui, "Heal", "Heal") {}
    });
}

/// AI ribbon tab.
fn render_ai_ribbon(ui: &mut egui::Ui) {
    group(ui, "Generate", |ui| {
        if cmd_button(ui, "Text→3D", "Shape") {}
        if cmd_button(ui, "A", "Variant A") {}
        if cmd_button(ui, "B", "Variant B") {}
        if cmd_button(ui, "C", "Variant C") {}
        if cmd_button(ui, "D", "Variant D") {}
    });
    group(ui, "Optimize", |ui| {
        if cmd_button(ui, "Light", "Lightweight") {}
        if cmd_button(ui, "Stiff", "Stiff") {}
        if cmd_button(ui, "Balance", "Balanced") {}
    });
    group(ui, "Assistant", |ui| {
        if cmd_button(ui, "💬", "Chat") {}
        if cmd_button(ui, "DRC", "Review") {}
        if cmd_button(ui, "€", "Cost") {}
    });
    group(ui, "Smart", |ui| {
        if cmd_button(ui, "AutoF", "Auto-Fillet") {}
        if cmd_button(ui, "AutoR", "Auto-Repair") {}
        if cmd_button(ui, "AutoD", "Auto-Dim") {}
    });
}

/// Tools ribbon tab.
fn render_tools_ribbon(ui: &mut egui::Ui) {
    group(ui, "Application", |ui| {
        if cmd_button(ui, "⚙", "Options") {}
        if cmd_button(ui, "🎨", "Customize") {}
        if cmd_button(ui, "🔌", "Plugins") {}
        if cmd_button(ui, "🎨", "Theme") {}
    });
    group(ui, "Scripting", |ui| {
        if cmd_button(ui, "🐍", "Console") {}
        if cmd_button(ui, "⏺", "Macro") {}
    });
    group(ui, "Performance", |ui| {
        if cmd_button(ui, "📊", "Monitor") {}
        if cmd_button(ui, "Profiler", "Profile") {}
    });
    group(ui, "UI", |ui| {
        if cmd_button(ui, "Layout", "Layout") {}
        if cmd_button(ui, "Reset", "Reset UI") {}
    });
}

/// View ribbon tab.
fn render_view_ribbon(ui: &mut egui::Ui) {
    group(ui, "Orient", |ui| {
        if cmd_button(ui, "ISO", "ISO") {}
        if cmd_button(ui, "Front", "Front") {}
        if cmd_button(ui, "Top", "Top") {}
        if cmd_button(ui, "Right", "Right") {}
    });
    group(ui, "Zoom", |ui| {
        if cmd_button(ui, "Fit", "Fit") {}
        if cmd_button(ui, "In", "In") {}
        if cmd_button(ui, "Out", "Out") {}
    });
    group(ui, "Style", |ui| {
        if cmd_button(ui, "Wire", "Wireframe") {}
        if cmd_button(ui, "Solid", "Shaded") {}
        if cmd_button(ui, "Both", "Edges") {}
    });
    group(ui, "Camera", |ui| {
        if cmd_button(ui, "Persp", "Perspective") {}
        if cmd_button(ui, "Ortho", "Orthographic") {}
    });
    group(ui, "Layouts", |ui| {
        if cmd_button(ui, "Save", "Save Layout") {}
    });
}

/// Surface ribbon tab.
fn render_surface_ribbon(ui: &mut egui::Ui) {
    group(ui, "Create", |ui| {
        if cmd_button(ui, "Loft", "Loft") {}
        if cmd_button(ui, "Sweep", "Sweep") {}
        if cmd_button(ui, "Boundary", "Boundary") {}
        if cmd_button(ui, "Fill", "Fill") {}
        if cmd_button(ui, "Network", "Network") {}
    });
    group(ui, "Continuity", |ui| {
        if cmd_button(ui, "G0", "G0") {}
        if cmd_button(ui, "G1", "G1") {}
        if cmd_button(ui, "G2", "G2") {}
    });
}
