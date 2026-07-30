// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Menu bar — 21 cascading menus per ROADMAP_UI Phase 1.
//!
//! Each menu is a function that returns `Option<MenuAction>` when an item is clicked.
//! Stub actions return `None` — they will be wired to backend logic in Phase 8.

use eframe::egui;

/// Action emitted by the menu bar.
#[derive(Clone, Debug)]
pub enum MenuAction {
    /// File actions.
    FileNew,
    FileOpen,
    FileSave,
    FileSaveAs,
    FileExportStep,
    FileExportStl,
    FileExportObj,
    FileImportStep,
    FileImportStl,
    FileQuit,
    /// Edit actions.
    EditUndo,
    EditRedo,
    EditCut,
    EditCopy,
    EditPaste,
    EditDuplicate,
    EditFind,
    /// View actions.
    ViewFit,
    ViewIso,
    ViewFront,
    ViewTop,
    ViewRight,
    ViewWireframe,
    ViewShaded,
    ViewShadedEdges,
    ViewToggleGrid,
    ViewToggleAxis,
    /// Insert actions.
    InsertBox,
    InsertSphere,
    InsertCylinder,
    InsertCone,
    InsertTorus,
    /// Modify actions.
    ModifyUnion,
    ModifySubtract,
    ModifyIntersect,
    ModifyFillet,
    ModifyChamfer,
    /// Sketch actions.
    SketchLine,
    SketchCircle,
    SketchArc,
    SketchRectangle,
    SketchSpline,
    SketchExit,
    /// Help actions.
    HelpAbout,
    HelpDocs,
    /// No action (stub).
    None,
}

/// Render the complete menu bar with 21 menus.
/// Returns the action if a menu item was clicked.
pub fn render_menu_bar(ctx: &egui::Context) -> Option<MenuAction> {
    let mut action: Option<MenuAction> = None;

    egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
        egui::menu::bar(ui, |ui| {
            // Each render_*_menu returns Option<MenuAction>.
            // We use take() to avoid move errors.
            if action.is_none() { action = render_file_menu(ui).take(); }
            if action.is_none() { action = render_edit_menu(ui).take(); }
            if action.is_none() { action = render_view_menu(ui).take(); }
            if action.is_none() { action = render_insert_menu(ui).take(); }
            if action.is_none() { action = render_sketch_menu(ui).take(); }
            if action.is_none() { action = render_modify_menu(ui).take(); }
            if action.is_none() { action = render_sheetmetal_menu(ui).take(); }
            if action.is_none() { action = render_assembly_menu(ui).take(); }
            if action.is_none() { action = render_cam_menu(ui).take(); }
            if action.is_none() { action = render_drawing_menu(ui).take(); }
            if action.is_none() { action = render_simulation_menu(ui).take(); }
            if action.is_none() { action = render_parametric_menu(ui).take(); }
            if action.is_none() { action = render_optimize_menu(ui).take(); }
            if action.is_none() { action = render_gdt_menu(ui).take(); }
            if action.is_none() { action = render_heal_menu(ui).take(); }
            if action.is_none() { action = render_mold_menu(ui).take(); }
            if action.is_none() { action = render_tools_menu(ui).take(); }
            if action.is_none() { action = render_scripting_menu(ui).take(); }
            if action.is_none() { action = render_ai_menu(ui).take(); }
            if action.is_none() { action = render_window_menu(ui).take(); }
            if action.is_none() { action = render_help_menu(ui).take(); }
        });
    });

    action
}

fn render_file_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("File", |ui| {
        if ui.button("New").clicked() { ui.close_menu(); return; }
        if ui.button("Open…").clicked() { ui.close_menu(); return; }
        ui.separator();
        if ui.button("Save").clicked() { ui.close_menu(); return; }
        if ui.button("Save As…").clicked() { ui.close_menu(); return; }
        ui.separator();
        ui.menu_button("Import", |ui| {
            if ui.button("STEP (*.stp, *.step)").clicked() { ui.close_menu(); return; }
            if ui.button("STL (*.stl)").clicked() { ui.close_menu(); return; }
            if ui.button("OBJ (*.obj)").clicked() { ui.close_menu(); return; }
            if ui.button("PLY (*.ply)").clicked() { ui.close_menu(); return; }
            if ui.button("DXF (*.dxf)").clicked() { ui.close_menu(); return; }
            if ui.button("Point Cloud (*.xyz, *.las)").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Export", |ui| {
            if ui.button("STEP (AP214)").clicked() { ui.close_menu(); return; }
            if ui.button("STEP (AP242)").clicked() { ui.close_menu(); return; }
            if ui.button("STL (*.stl)").clicked() { ui.close_menu(); return; }
            if ui.button("OBJ (*.obj)").clicked() { ui.close_menu(); return; }
            if ui.button("GLTF (*.gltf)").clicked() { ui.close_menu(); return; }
            if ui.button("PDF (*.pdf)").clicked() { ui.close_menu(); return; }
            if ui.button("DXF (*.dxf)").clicked() { ui.close_menu(); return; }
        });
        ui.separator();
        ui.menu_button("Recent", |ui| {
            ui.label("(no recent files)");
        });
        ui.separator();
        if ui.button("Print / Plot…").clicked() { ui.close_menu(); return; }
        ui.separator();
        if ui.button("Exit").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_edit_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Edit", |ui| {
        if ui.button("Undo  ⌘Z").clicked() { ui.close_menu(); return; }
        if ui.button("Redo  ⌘⇧Z").clicked() { ui.close_menu(); return; }
        ui.separator();
        ui.menu_button("History", |ui| {
            ui.label("Snapshot / Branch / Diff / Tree");
        });
        ui.separator();
        if ui.button("Cut  ⌘X").clicked() { ui.close_menu(); return; }
        if ui.button("Copy  ⌘C").clicked() { ui.close_menu(); return; }
        if ui.button("Paste  ⌘V").clicked() { ui.close_menu(); return; }
        if ui.button("Duplicate  ⌘D").clicked() { ui.close_menu(); return; }
        ui.separator();
        if ui.button("Find…").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_view_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("View", |ui| {
        ui.menu_button("Orient", |ui| {
            let orientations = ["ISO", "Front", "Back", "Top", "Bottom", "Left", "Right", "Dimetric"];
            for o in &orientations {
                if ui.button(*o).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Zoom", |ui| {
            if ui.button("Fit").clicked() { ui.close_menu(); return; }
            if ui.button("Window").clicked() { ui.close_menu(); return; }
            if ui.button("In").clicked() { ui.close_menu(); return; }
            if ui.button("Out").clicked() { ui.close_menu(); return; }
            if ui.button("Selection").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Display Style", |ui| {
            if ui.button("Wireframe").clicked() { ui.close_menu(); return; }
            if ui.button("Shaded").clicked() { ui.close_menu(); return; }
            if ui.button("Shaded + Edges").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Options", |ui| {
            for opt in &["Grid", "Axis", "Triad", "View Cube", "Shadows", "Ambient Occlusion", "Anti-alias", "Edges", "Normals", "Silhouette"] {
                ui.checkbox(&mut false, *opt);
            }
        });
        ui.menu_button("Camera", |ui| {
            ui.checkbox(&mut true, "Perspective");
            ui.checkbox(&mut false, "Orthographic");
            ui.add(egui::Slider::new(&mut 45.0_f32, 10.0..=120.0).text("FOV"));
        });
        ui.menu_button("Layouts", |ui| {
            if ui.button("Save Layout…").clicked() { ui.close_menu(); return; }
            if ui.button("Load Layout…").clicked() { ui.close_menu(); return; }
        });
    });
    None
}

fn render_insert_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Insert", |ui| {
        ui.menu_button("Primitives", |ui| {
            if ui.button("Box").clicked() { ui.close_menu(); return; }
            if ui.button("Sphere").clicked() { ui.close_menu(); return; }
            if ui.button("Cylinder").clicked() { ui.close_menu(); return; }
            if ui.button("Cone").clicked() { ui.close_menu(); return; }
            if ui.button("Torus").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Reference Geometry", |ui| {
            if ui.button("Plane").clicked() { ui.close_menu(); return; }
            if ui.button("Axis").clicked() { ui.close_menu(); return; }
            if ui.button("Point").clicked() { ui.close_menu(); return; }
            if ui.button("Coordinate System").clicked() { ui.close_menu(); return; }
        });
        if ui.button("Sketch").clicked() { ui.close_menu(); return; }
        ui.menu_button("Mesh", |ui| {
            if ui.button("Import Mesh").clicked() { ui.close_menu(); return; }
            if ui.button("Mesh from Solid").clicked() { ui.close_menu(); return; }
            if ui.button("Remesh").clicked() { ui.close_menu(); return; }
        });
        if ui.button("Component").clicked() { ui.close_menu(); return; }
        ui.menu_button("Pattern", |ui| {
            if ui.button("Linear Pattern").clicked() { ui.close_menu(); return; }
            if ui.button("Circular Pattern").clicked() { ui.close_menu(); return; }
            if ui.button("Mirror").clicked() { ui.close_menu(); return; }
        });
    });
    None
}

fn render_sketch_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Sketch", |ui| {
        ui.menu_button("Draw", |ui| {
            for t in &["Line", "Circle", "Arc (3-Point)", "Arc (Tangent)", "Rectangle", "Spline", "Polygon", "Point"] {
                if ui.button(*t).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Constrain", |ui| {
            for c in &["Coincident", "Collinear", "Concentric", "Parallel", "Perpendicular", "Tangent", "Horizontal", "Vertical", "Equal"] {
                if ui.button(*c).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Dimension", |ui| {
            for d in &["Linear", "Angular", "Radial", "Diameter"] {
                if ui.button(*d).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Modify", |ui| {
            for m in &["Trim", "Extend", "Split", "Offset", "Mirror", "Pattern", "Fillet"] {
                if ui.button(*m).clicked() { ui.close_menu(); return; }
            }
        });
        ui.separator();
        if ui.button("Exit Sketch").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_modify_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Modify", |ui| {
        ui.menu_button("Boolean", |ui| {
            if ui.button("Union").clicked() { ui.close_menu(); return; }
            if ui.button("Subtract").clicked() { ui.close_menu(); return; }
            if ui.button("Intersect").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Edge", |ui| {
            if ui.button("Fillet").clicked() { ui.close_menu(); return; }
            if ui.button("Chamfer").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Surface", |ui| {
            if ui.button("Loft").clicked() { ui.close_menu(); return; }
            if ui.button("Sweep").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Transform", |ui| {
            if ui.button("Move").clicked() { ui.close_menu(); return; }
            if ui.button("Rotate").clicked() { ui.close_menu(); return; }
            if ui.button("Scale").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Pattern", |ui| {
            if ui.button("Linear").clicked() { ui.close_menu(); return; }
            if ui.button("Circular").clicked() { ui.close_menu(); return; }
            if ui.button("Mirror").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Direct Modeling", |ui| {
            for d in &["Move Face", "Offset Face", "Delete Face", "Replace Face", "Split Face", "Merge Faces", "Simplify", "Thicken"] {
                if ui.button(*d).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Deform", |ui| {
            for d in &["Bend", "Twist", "Taper", "Stretch"] {
                if ui.button(*d).clicked() { ui.close_menu(); return; }
            }
        });
    });
    None
}

fn render_sheetmetal_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Sheet Metal", |ui| {
        ui.menu_button("Flange", |ui| {
            if ui.button("Base Flange").clicked() { ui.close_menu(); return; }
            if ui.button("Edge Flange").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Bend", |ui| {
            if ui.button("Bend").clicked() { ui.close_menu(); return; }
            if ui.button("Hem").clicked() { ui.close_menu(); return; }
            if ui.button("Jog").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Relief", |ui| {
            if ui.button("Rectangular Relief").clicked() { ui.close_menu(); return; }
            if ui.button("Tear Relief").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Flatten", |ui| {
            if ui.button("Unfold").clicked() { ui.close_menu(); return; }
            if ui.button("Fold").clicked() { ui.close_menu(); return; }
            if ui.button("Flat Pattern").clicked() { ui.close_menu(); return; }
            if ui.button("Export DXF").clicked() { ui.close_menu(); return; }
        });
        ui.separator();
        if ui.button("Gauge Table").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_assembly_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Assembly", |ui| {
        if ui.button("Add Component").clicked() { ui.close_menu(); return; }
        ui.menu_button("Mate", |ui| {
            for m in &["Coincident", "Concentric", "Distance", "Angle", "Parallel", "Perpendicular", "Tangent", "Width", "Symmetric"] {
                if ui.button(*m).clicked() { ui.close_menu(); return; }
            }
        });
        if ui.button("Solve").clicked() { ui.close_menu(); return; }
        if ui.button("BOM Editor").clicked() { ui.close_menu(); return; }
        if ui.button("Explode").clicked() { ui.close_menu(); return; }
        if ui.button("Motion Study").clicked() { ui.close_menu(); return; }
        if ui.button("Constraint Diagnostics").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_cam_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("CAM", |ui| {
        ui.menu_button("Setup", |ui| {
            if ui.button("Stock Setup").clicked() { ui.close_menu(); return; }
            if ui.button("Coordinate System").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Tools", |ui| {
            for t in &["Flat End Mill", "Ball End Mill", "Bull Nose", "Drill", "Face Mill", "Tap"] {
                if ui.button(*t).clicked() { ui.close_menu(); return; }
            }
            if ui.button("Tool Library").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Operations", |ui| {
            for op in &["Facing", "Profile", "Pocket", "Drilling", "Engraving", "3D Surfacing", "Thread Milling", "Boring", "Reaming"] {
                if ui.button(*op).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Simulate", |ui| {
            for s in &["2D Sim", "3D Sim", "Toolpath Only", "Material Removal", "Collision Check"] {
                if ui.button(*s).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Post Process", |ui| {
            for p in &["Fanuc", "Siemens", "Haas", "Heidenhain", "Mach3", "LinuxCNC", "GRBL"] {
                if ui.button(*p).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("5-Axis", |ui| {
            for a in &["SWARF", "Morph", "Flow Cut", "Multi-Axis Contour", "Blade", "Tube"] {
                if ui.button(*a).clicked() { ui.close_menu(); return; }
            }
        });
    });
    None
}

fn render_drawing_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Drawing", |ui| {
        if ui.button("New Sheet").clicked() { ui.close_menu(); return; }
        ui.menu_button("Views", |ui| {
            for v in &["Standard", "Section", "Detail", "Projected", "Broken-out", "Crop", "Auxiliary", "Exploded"] {
                if ui.button(*v).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Dimensions", |ui| {
            for d in &["Linear", "Angular", "Radial", "Diameter", "Ordinate"] {
                if ui.button(*d).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Annotations", |ui| {
            for a in &["Note", "Balloon", "Surface Finish", "Welding", "Datum", "Tolerance", "Leader"] {
                if ui.button(*a).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Templates", |ui| {
            for t in &["A0", "A1", "A2", "A3", "A4", "ANSI B"] {
                if ui.button(*t).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Export", |ui| {
            for e in &["PDF", "DXF", "DWG", "SVG"] {
                if ui.button(*e).clicked() { ui.close_menu(); return; }
            }
        });
    });
    None
}

fn render_simulation_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Simulation", |ui| {
        if ui.button("Mesh").clicked() { ui.close_menu(); return; }
        ui.menu_button("Study", |ui| {
            for s in &["Static", "Modal", "Thermal", "Buckling", "Fatigue", "Nonlinear", "CFD", "Electromagnetic", "Optimization"] {
                if ui.button(*s).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Run", |ui| {
            for r in &["Solve", "Validate", "Batch", "Parametric Sweep"] {
                if ui.button(*r).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Results", |ui| {
            for r in &["Von Mises", "Displacement", "Strain", "Stress XX", "Animate"] {
                if ui.button(*r).clicked() { ui.close_menu(); return; }
            }
        });
    });
    None
}

fn render_parametric_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Parametric", |ui| {
        if ui.button("Parameters").clicked() { ui.close_menu(); return; }
        if ui.button("Equations").clicked() { ui.close_menu(); return; }
        if ui.button("Design Table").clicked() { ui.close_menu(); return; }
        if ui.button("Dependency Graph").clicked() { ui.close_menu(); return; }
        if ui.button("Variants").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_optimize_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Optimize", |ui| {
        ui.menu_button("Topology Optimization", |ui| {
            for p in &["Lightweight", "Stiff", "Balanced"] {
                if ui.button(*p).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Generative Design", |ui| {
            for v in &["Variant A", "Variant B", "Variant C", "Variant D"] {
                if ui.button(*v).clicked() { ui.close_menu(); return; }
            }
        });
    });
    None
}

fn render_gdt_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("GD&T", |ui| {
        if ui.button("Datum").clicked() { ui.close_menu(); return; }
        ui.menu_button("Form", |ui| {
            for f in &["Flatness", "Straightness", "Circularity", "Cylindricity"] {
                if ui.button(*f).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Orientation", |ui| {
            for o in &["Parallelism", "Perpendicularity", "Angularity"] {
                if ui.button(*o).clicked() { ui.close_menu(); return; }
            }
        });
        if ui.button("Position").clicked() { ui.close_menu(); return; }
        ui.menu_button("Profile", |ui| {
            if ui.button("Profile of Line").clicked() { ui.close_menu(); return; }
            if ui.button("Profile of Surface").clicked() { ui.close_menu(); return; }
        });
        ui.menu_button("Runout", |ui| {
            if ui.button("Circular Runout").clicked() { ui.close_menu(); return; }
            if ui.button("Total Runout").clicked() { ui.close_menu(); return; }
        });
        ui.separator();
        if ui.button("Analyze").clicked() { ui.close_menu(); return; }
        if ui.button("Reports").clicked() { ui.close_menu(); return; }
        if ui.button("Stackup Analysis").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_heal_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Heal", |ui| {
        ui.menu_button("Heal", |ui| {
            for h in &["Stitch", "Gap Fill", "Remove Duplicates", "Fix Orientation", "Fix Degenerate", "Simplify", "Remove Sliver", "Close Holes", "Repair T-Junctions"] {
                if ui.button(*h).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Measure", |ui| {
            for m in &["Distance", "Angle", "Length", "Area", "Volume", "Mass", "Diameter", "Radius", "Center"] {
                if ui.button(*m).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Analysis", |ui| {
            for a in &["Watertight Check", "Manifold Check", "Curvature", "Draft Analysis", "Thickness", "Interference", "Edge Consistency", "Gaussian Curvature"] {
                if ui.button(*a).clicked() { ui.close_menu(); return; }
            }
        });
    });
    None
}

fn render_mold_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Mold", |ui| {
        if ui.button("Mold Base Catalog").clicked() { ui.close_menu(); return; }
        if ui.button("Runner System").clicked() { ui.close_menu(); return; }
        if ui.button("Cooling System").clicked() { ui.close_menu(); return; }
        if ui.button("Ejection System").clicked() { ui.close_menu(); return; }
        if ui.button("Cavity/Core").clicked() { ui.close_menu(); return; }
        if ui.button("Flow Analysis").clicked() { ui.close_menu(); return; }
        if ui.button("Cooling Analysis").clicked() { ui.close_menu(); return; }
        if ui.button("Warpage Analysis").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_tools_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Tools", |ui| {
        if ui.button("Options").clicked() { ui.close_menu(); return; }
        if ui.button("Customize").clicked() { ui.close_menu(); return; }
        if ui.button("Plugins Manager").clicked() { ui.close_menu(); return; }
        if ui.button("Scripting Console").clicked() { ui.close_menu(); return; }
        if ui.button("AI Settings").clicked() { ui.close_menu(); return; }
        if ui.button("Macro Recorder").clicked() { ui.close_menu(); return; }
        if ui.button("Performance Monitor").clicked() { ui.close_menu(); return; }
        if ui.button("Theme").clicked() { ui.close_menu(); return; }
        if ui.button("UI Layout").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_scripting_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Scripting", |ui| {
        if ui.button("Script List").clicked() { ui.close_menu(); return; }
        if ui.button("Load Script").clicked() { ui.close_menu(); return; }
        if ui.button("Record Macro").clicked() { ui.close_menu(); return; }
        if ui.button("Run with Parameters").clicked() { ui.close_menu(); return; }
        if ui.button("Debug Step").clicked() { ui.close_menu(); return; }
        if ui.button("Profile").clicked() { ui.close_menu(); return; }
        if ui.button("Library Browser").clicked() { ui.close_menu(); return; }
        if ui.button("API Reference").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_ai_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("AI", |ui| {
        if ui.button("Shape from Text").clicked() { ui.close_menu(); return; }
        ui.menu_button("Assistant", |ui| {
            for a in &["Chat", "Design Review", "Cost Estimate", "Suggest Feature"] {
                if ui.button(*a).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Smart", |ui| {
            for s in &["Auto-Fillet", "Auto-Pattern", "Auto-Repair", "Auto-Dimension", "Auto-Constrain"] {
                if ui.button(*s).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Generate", |ui| {
            for g in &["Variant A", "Variant B", "Variant C", "Variant D"] {
                if ui.button(*g).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Optimize", |ui| {
            for o in &["Lightweight", "Stiff", "Balanced", "Custom"] {
                if ui.button(*o).clicked() { ui.close_menu(); return; }
            }
        });
        if ui.button("AI Settings").clicked() { ui.close_menu(); return; }
    });
    None
}

fn render_window_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Window", |ui| {
        if ui.button("Close All").clicked() { ui.close_menu(); return; }
        if ui.button("Cascade").clicked() { ui.close_menu(); return; }
        if ui.button("Tile Horizontal").clicked() { ui.close_menu(); return; }
        if ui.button("Tile Vertical").clicked() { ui.close_menu(); return; }
        ui.separator();
        if ui.button("Next Tab").clicked() { ui.close_menu(); return; }
        if ui.button("Previous Tab").clicked() { ui.close_menu(); return; }
        ui.separator();
        if ui.button("Save Layout").clicked() { ui.close_menu(); return; }
        ui.separator();
        ui.label("Open Documents:");
        ui.label("  • model.step");
    });
    None
}

fn render_help_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    ui.menu_button("Help", |ui| {
        if ui.button("Check for Updates").clicked() { ui.close_menu(); return; }
        if ui.button("About BRepCAD").clicked() { ui.close_menu(); return; }
        ui.separator();
        if ui.button("Documentation").clicked() { ui.close_menu(); return; }
        if ui.button("Forum").clicked() { ui.close_menu(); return; }
        if ui.button("Report Bug").clicked() { ui.close_menu(); return; }
        if ui.button("Assets Library").clicked() { ui.close_menu(); return; }
        ui.separator();
        ui.menu_button("Tutorials", |ui| {
            for t in &["Getting Started", "Sketch Tutorial", "Assembly Tutorial"] {
                if ui.button(*t).clicked() { ui.close_menu(); return; }
            }
        });
        ui.menu_button("Examples", |ui| {
            for e in &["Bracket", "Bolt", "Gear", "Engine Block", "Mold Cavity", "Sheet Metal Part", "Assembly"] {
                if ui.button(*e).clicked() { ui.close_menu(); return; }
            }
        });
    });
    None
}
