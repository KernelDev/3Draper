// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Side panels for the non-VP workspaces (Sketch, SheetMetal, CAM, FEA,
//! Drawing, AI). Each panel exposes the real module API on the currently
//! loaded mesh, so clicking a workspace icon in the left sidebar actually
//! does something — instead of just printing "Workspace: X" in the status
//! bar.
//!
//! Layout: each workspace gets a right-side panel (280px wide) with:
//!   • a header describing what the workspace does
//!   • input fields for the most common parameters
//!   • a button that runs the operation on `self.mesh`
//!   • a result view (text / image / table)
//!
//! The 3D viewport (CentralPanel) remains visible on the left, so the
//! user can see the model they're operating on.

use eframe::egui;
use draper_mesh::TriangleMesh;

/// Show the right-side panel for the active workspace.
///
/// `mesh` is the current document mesh (read-only — these panels do not
/// modify the mesh; they only analyze it and display results).
/// `status_cb` is called with a short message to display in the bottom
/// status bar (e.g. "FEA: max stress = 250 MPa").
pub fn show_workspace_panel(
    ctx: &egui::Context,
    workspace: crate::ui::Workspace,
    mesh: &TriangleMesh,
    status_cb: &mut dyn FnMut(String),
) {
    egui::SidePanel::right("brepcad_workspace_tools")
        .default_width(320.0)
        .min_width(280.0)
        .resizable(true)
        .frame(egui::Frame::default()
            .fill(egui::Color32::from_rgb(0x16, 0x16, 0x22))
            .stroke(egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(0x31, 0x32, 0x44))))
        .show(ctx, |ui| {
            ui.add_space(6.0);
            ui.vertical(|ui| {
                match workspace {
                    crate::ui::Workspace::Modeling => {
                        modeling_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::Sketch => {
                        sketch_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::VisualProgramming => {
                        // VP has its own full layout — no right panel here
                    }
                    crate::ui::Workspace::Surface => {
                        surface_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::SheetMetal => {
                        sheetmetal_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::Assembly => {
                        assembly_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::Cam => {
                        cam_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::Drawing => {
                        drawing_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::Simulation => {
                        fea_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::Inspect => {
                        inspect_panel(ui, mesh, status_cb);
                    }
                    crate::ui::Workspace::Ai => {
                        ai_panel(ui, mesh, status_cb);
                    }
                }
            });
        });
}

// ─────────────────────────────────────────────────────────────────────
// Per-workspace panels
// ─────────────────────────────────────────────────────────────────────

fn modeling_panel(ui: &mut egui::Ui, mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("🏠 Modeling");
    ui.label(
        egui::RichText::new("Basic 3D modeling tools. Use the ribbon above to insert primitives.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    ui.label("Document stats:");
    ui.add_space(4.0);
    let (bb_min, bb_max) = mesh.bounding_box();
    let dx = (bb_max.x - bb_min.x).max(0.0);
    let dy = (bb_max.y - bb_min.y).max(0.0);
    let dz = (bb_max.z - bb_min.z).max(0.0);
    ui.label(format!("  Vertices: {}", mesh.vertices.len()));
    ui.label(format!("  Triangles: {}", mesh.triangles.len()));
    ui.label(format!("  Bounding box: {:.2} × {:.2} × {:.2}", dx, dy, dz));
    ui.label(format!("  Min: ({:.2}, {:.2}, {:.2})", bb_min.x, bb_min.y, bb_min.z));
    ui.label(format!("  Max: ({:.2}, {:.2}, {:.2})", bb_max.x, bb_max.y, bb_max.z));
    let _ = status_cb;
}

fn sketch_panel(ui: &mut egui::Ui, _mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("✏️ Sketch");
    ui.label(
        egui::RichText::new("2D sketcher with constraint solver. Build sketches that can be extruded into solids.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    // Live sketch state (stored in a private egui::Memory)
    let sketch_id = ui.id().with("sketch_state");
    let mut sketch: SketchState = ui.data_mut(|d| d.get_temp(sketch_id).unwrap_or_default());
    let mut changed = false;

    ui.label("Add a point:");
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut sketch.new_x).speed(0.1).range(-100.0..=100.0));
        ui.add(egui::DragValue::new(&mut sketch.new_y).speed(0.1).range(-100.0..=100.0));
        if ui.button("+ Point").clicked() {
            sketch.points.push((sketch.new_x, sketch.new_y));
            changed = true;
            status_cb(format!("Sketch: added point ({}, {})", sketch.new_x, sketch.new_y));
        }
    });
    ui.add_space(4.0);

    ui.label("Add a line (by point indices):");
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut sketch.line_a).range(0..=1000));
        ui.label("→");
        ui.add(egui::DragValue::new(&mut sketch.line_b).range(0..=1000));
        if ui.button("+ Line").clicked() {
            let n = sketch.points.len();
            if (sketch.line_a as usize) < n && (sketch.line_b as usize) < n && sketch.line_a != sketch.line_b {
                sketch.lines.push((sketch.line_a as usize, sketch.line_b as usize));
                changed = true;
                status_cb(format!("Sketch: added line {}→{}", sketch.line_a, sketch.line_b));
            } else {
                status_cb("Sketch: invalid point indices".to_string());
            }
        }
    });
    ui.add_space(4.0);

    ui.label("Add a circle (center index + radius):");
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut sketch.circ_center).range(0..=1000));
        ui.add(egui::DragValue::new(&mut sketch.circ_r).speed(0.1).range(0.01..=100.0));
        if ui.button("+ Circle").clicked() {
            if (sketch.circ_center as usize) < sketch.points.len() {
                sketch.circles.push((sketch.circ_center as usize, sketch.circ_r));
                changed = true;
                status_cb(format!("Sketch: added circle at point #{} (r={})", sketch.circ_center, sketch.circ_r));
            } else {
                status_cb("Sketch: invalid center index".to_string());
            }
        }
    });
    ui.add_space(8.0);

    ui.separator();
    ui.label(format!("Points: {}, Lines: {}, Circles: {}",
        sketch.points.len(), sketch.lines.len(), sketch.circles.len()));

    // Build a real Sketch2d and solve
    if ui.button("🧮 Solve Constraints").clicked() {
        let mut s2d = draper_sketch::Sketch2d::new();
        let mut pt_ids = Vec::new();
        for &(x, y) in &sketch.points {
            pt_ids.push(s2d.add_point(x, y));
        }
        for &(a, b) in &sketch.lines {
            s2d.add_line(pt_ids[a], pt_ids[b]);
        }
        for &(c, r) in &sketch.circles {
            s2d.add_circle(pt_ids[c], "r", r);
        }
        let mut solver = draper_sketch::ConstraintSolver::new();
        match solver.solve(&mut s2d, 200) {
            Ok(()) => status_cb(format!("Sketch: solved {} constraints", sketch.points.len())),
            Err(e) => status_cb(format!("Sketch: solve failed — {}", e)),
        }
    }

    if ui.button("🗑 Clear sketch").clicked() {
        sketch = SketchState::default();
        changed = true;
        status_cb("Sketch: cleared".to_string());
    }

    if changed {
        ui.data_mut(|d| d.insert_temp(sketch_id, sketch));
    }
}

#[derive(Clone, Default)]
struct SketchState {
    points: Vec<(f64, f64)>,
    lines: Vec<(usize, usize)>,
    circles: Vec<(usize, f64)>,
    new_x: f64,
    new_y: f64,
    line_a: i32,
    line_b: i32,
    circ_center: i32,
    circ_r: f64,
}

fn surface_panel(ui: &mut egui::Ui, mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("🌐 Surface");
    ui.label(
        egui::RichText::new("Surface analysis and manipulation on the current mesh.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    ui.label(format!("Mesh: {} triangles", mesh.triangles.len()));
    if mesh.triangles.is_empty() {
        ui.label(
            egui::RichText::new("Load a model first (File → Import STL/STEP).")
                .color(egui::Color32::from_rgb(0xff, 0x8b, 0x8b)),
        );
    }
    let _ = status_cb;
}

fn sheetmetal_panel(ui: &mut egui::Ui, _mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("📋 Sheet Metal");
    ui.label(
        egui::RichText::new("Calculate bend allowance, deduction, and unfold a sheet metal part.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    // Material picker
    let mat_id = ui.id().with("sm_state");
    let mut sm: SmState = ui.data_mut(|d| d.get_temp(mat_id).unwrap_or_default());
    let mut changed = false;

    ui.label("Material preset:");
    ui.horizontal(|ui| {
        let presets = ["Steel 1.5mm", "Aluminum 2mm", "Stainless 1mm"];
        let mat = draper_sheetmetal::SheetMaterial::steel_1_5mm(); // dummy for borrow
        let _ = mat;
        egui::ComboBox::from_id_salt("sm_mat_preset")
            .selected_text(presets[sm.mat_preset])
            .show_ui(ui, |ui| {
                for (i, p) in presets.iter().enumerate() {
                    if ui.selectable_label(sm.mat_preset == i, *p).clicked() {
                        sm.mat_preset = i;
                        changed = true;
                    }
                }
            });
    });
    ui.add_space(4.0);

    ui.label("Bend parameters:");
    ui.horizontal(|ui| {
        ui.label("Radius:");
        ui.add(egui::DragValue::new(&mut sm.bend_radius).speed(0.1).range(0.1..=50.0).suffix(" mm"));
    });
    ui.horizontal(|ui| {
        ui.label("Angle:");
        ui.add(egui::DragValue::new(&mut sm.bend_angle).speed(1.0).range(1.0..=180.0).suffix("°"));
    });
    ui.horizontal(|ui| {
        ui.label("Length:");
        ui.add(egui::DragValue::new(&mut sm.bend_length).speed(0.5).range(0.1..=500.0).suffix(" mm"));
    });
    ui.add_space(8.0);

    let material = match sm.mat_preset {
        0 => draper_sheetmetal::SheetMaterial::steel_1_5mm(),
        1 => draper_sheetmetal::SheetMaterial::aluminum_2mm(),
        2 => draper_sheetmetal::SheetMaterial::stainless_1mm(),
        _ => draper_sheetmetal::SheetMaterial::steel_1_5mm(),
    };
    ui.label(format!("Material: {}", material.name));
    ui.label(format!("  Thickness: {:.2} mm", material.thickness));
    ui.label(format!("  K-factor: {:.3}", material.k_factor));
    ui.add_space(8.0);

    if ui.button("🧮 Calculate Bend Allowance").clicked() {
        match draper_sheetmetal::Bend::new(sm.bend_radius, sm.bend_angle, sm.bend_length) {
            Ok(bend) => {
                let ba = bend.bend_allowance(&material);
                let bd = bend.bend_deduction(&material);
                let sb = bend.outside_setback(&material);
                sm.last_ba = Some(ba);
                sm.last_bd = Some(bd);
                sm.last_sb = Some(sb);
                changed = true;
                status_cb(format!("Sheet metal: BA={:.3} mm, BD={:.3} mm", ba, bd));
            }
            Err(e) => status_cb(format!("Sheet metal: error — {}", e)),
        }
    }
    ui.add_space(4.0);
    if let Some(ba) = sm.last_ba {
        ui.label(format!("Bend allowance: {:.3} mm", ba));
    }
    if let Some(bd) = sm.last_bd {
        ui.label(format!("Bend deduction: {:.3} mm", bd));
    }
    if let Some(sb) = sm.last_sb {
        ui.label(format!("Outside setback: {:.3} mm", sb));
    }

    if changed {
        ui.data_mut(|d| d.insert_temp(mat_id, sm));
    }
}

#[derive(Clone)]
struct SmState {
    mat_preset: usize,
    bend_radius: f64,
    bend_angle: f64,
    bend_length: f64,
    last_ba: Option<f64>,
    last_bd: Option<f64>,
    last_sb: Option<f64>,
}

impl Default for SmState {
    fn default() -> Self {
        Self {
            mat_preset: 0,
            bend_radius: 1.5,
            bend_angle: 90.0,
            bend_length: 50.0,
            last_ba: None,
            last_bd: None,
            last_sb: None,
        }
    }
}

fn assembly_panel(ui: &mut egui::Ui, _mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("🔗 Assembly");
    ui.label(
        egui::RichText::new("Multi-part assembly with 6-DOF constraint solver.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    let asm_id = ui.id().with("asm_state");
    let mut asm_state: AsmState = ui.data_mut(|d| d.get_temp(asm_id).unwrap_or_default());
    let mut changed = false;

    let mut assembly: draper_assembly::Assembly = ui.data(|d| d.get_temp::<draper_assembly::Assembly>(asm_id.with("tree")).unwrap_or_default());
    let n_components = assembly.components.len();

    ui.label(format!("Components: {}", n_components));
    if n_components == 0 {
        if ui.button("+ Add base component").clicked() {
            let c = draper_assembly::Component::new_fixed(0, "Base");
            assembly.add_component(c);
            changed = true;
            status_cb("Assembly: added base component".to_string());
        }
    } else {
        ui.horizontal(|ui| {
            ui.label("Name:");
            ui.text_edit_singleline(&mut asm_state.new_name);
        });
        ui.horizontal(|ui| {
            ui.label("Translation:");
            ui.add(egui::DragValue::new(&mut asm_state.new_x).speed(0.5).range(-500.0..=500.0));
            ui.add(egui::DragValue::new(&mut asm_state.new_y).speed(0.5).range(-500.0..=500.0));
            ui.add(egui::DragValue::new(&mut asm_state.new_z).speed(0.5).range(-500.0..=500.0));
        });
        if ui.button("+ Add component").clicked() {
            let id = assembly.components.len();
            let mut c = draper_assembly::Component::new(id, if asm_state.new_name.is_empty() { "Part" } else { &asm_state.new_name });
            c.set_translation(asm_state.new_x, asm_state.new_y, asm_state.new_z);
            assembly.add_component(c);
            changed = true;
            status_cb(format!("Assembly: added component #{}", id));
        }
    }
    ui.add_space(4.0);
    ui.separator();
    ui.add_space(4.0);

    ui.label("Components list:");
    for c in &assembly.components {
        let (x, y, z) = c.translation();
        let (rx, ry, rz) = c.rotation_vec();
        ui.label(format!("#{}: {}  pos=({:.1},{:.1},{:.1})  rot=({:.1},{:.1},{:.1})",
            c.id, c.name, x, y, z, rx, ry, rz));
    }

    ui.add_space(8.0);
    if ui.button("🧮 Solve constraints").clicked() && assembly.components.len() >= 2 {
        let mut solver = draper_assembly::AssemblySolver::new();
        match solver.solve(&mut assembly) {
            Ok(()) => status_cb(format!("Assembly: solved {} components", assembly.components.len())),
            Err(e) => status_cb(format!("Assembly: solve failed — {}", e)),
        }
    }

    if changed || ui.data(|d| d.get_temp::<draper_assembly::Assembly>(asm_id.with("tree")).is_none()) {
        ui.data_mut(|d| {
            d.insert_temp(asm_id, asm_state.clone());
            d.insert_temp(asm_id.with("tree"), assembly);
        });
    }
}

impl Default for AsmState { fn default() -> Self { Self { new_name: String::new(), new_x: 0.0, new_y: 0.0, new_z: 0.0 } } }

#[derive(Clone)]
struct AsmState {
    new_name: String,
    new_x: f64,
    new_y: f64,
    new_z: f64,
}

fn cam_panel(ui: &mut egui::Ui, _mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("⚙️ CAM");
    ui.label(
        egui::RichText::new("Generate CNC toolpaths and G-code for the current model.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    let cam_id = ui.id().with("cam_state");
    let mut cam: CamState = ui.data_mut(|d| d.get_temp(cam_id).unwrap_or_default());
    let mut changed = false;

    ui.label("Tool preset:");
    egui::ComboBox::from_id_salt("cam_tool")
        .selected_text(match cam.tool_preset {
            0 => "Endmill 3mm",
            1 => "Endmill 6mm",
            2 => "Facemill 10mm",
            3 => "Drill 5mm",
            _ => "?",
        })
        .show_ui(ui, |ui| {
            let tools = ["Endmill 3mm", "Endmill 6mm", "Facemill 10mm", "Drill 5mm"];
            for (i, t) in tools.iter().enumerate() {
                if ui.selectable_label(cam.tool_preset == i, *t).clicked() {
                    cam.tool_preset = i;
                    changed = true;
                }
            }
        });
    ui.add_space(4.0);

    ui.label("Operation parameters:");
    ui.horizontal(|ui| {
        ui.label("Spindle:");
        ui.add(egui::DragValue::new(&mut cam.spindle_rpm).speed(100.0).range(100..=60000).suffix(" RPM"));
    });
    ui.horizontal(|ui| {
        ui.label("Feed rate:");
        ui.add(egui::DragValue::new(&mut cam.feed_rate).speed(10.0).range(1..=5000).suffix(" mm/min"));
    });
    ui.horizontal(|ui| {
        ui.label("Step-down:");
        ui.add(egui::DragValue::new(&mut cam.step_down).speed(0.1).range(0.1..=50.0).suffix(" mm"));
    });
    ui.add_space(8.0);

    let tool = match cam.tool_preset {
        0 => draper_cam::Tool::endmill_3mm(),
        1 => draper_cam::Tool::endmill_6mm(),
        2 => draper_cam::Tool::facemill_10mm(),
        3 => draper_cam::Tool::drill_5mm(),
        _ => draper_cam::Tool::endmill_6mm(),
    };
    ui.label(format!("Tool: {} Ø{:.1}mm", tool.tool_type.name(), tool.diameter));

    if ui.button("🛠 Generate Toolpath").clicked() {
        // Build a simple rectangular pocket operation as a demo.
        let op = draper_cam::CamOperation::PocketRect {
            cx: 0.0, cy: 0.0,
            width: 40.0, height: 40.0,
            depth: 5.0,
            safe_z: 10.0,
            tool: tool.clone(),
            stepover: 0.5,
            step_down: cam.step_down,
        };
        match op.generate_toolpath() {
            Ok(path) => {
                cam.path_len = Some(path.len());
                changed = true;
                status_cb(format!("CAM: generated {} toolpath points", path.len()));
            }
            Err(e) => status_cb(format!("CAM: error — {}", e)),
        }
    }

    if ui.button("📄 Generate G-code").clicked() {
        let op = draper_cam::CamOperation::PocketRect {
            cx: 0.0, cy: 0.0,
            width: 40.0, height: 40.0,
            depth: 5.0,
            safe_z: 10.0,
            tool: tool.clone(),
            stepover: 0.5,
            step_down: cam.step_down,
        };
        let ops = vec![op];
        let gen = draper_cam::GcodeGenerator::new();
        match gen.generate(&ops) {
            Ok(gcode) => {
                let lines = gcode.lines().count();
                cam.gcode_len = Some(lines);
                cam.gcode_preview = Some(gcode.chars().take(800).collect());
                changed = true;
                status_cb(format!("CAM: generated {} lines of G-code", lines));
            }
            Err(e) => status_cb(format!("CAM: G-code error — {}", e)),
        }
    }
    ui.add_space(4.0);
    if let Some(n) = cam.path_len {
        ui.label(format!("Toolpath points: {}", n));
    }
    if let Some(n) = cam.gcode_len {
        ui.label(format!("G-code lines: {}", n));
        if ui.button("📋 Show preview").clicked() {
            // Toggle preview visibility by toggling state.
            cam.show_preview = !cam.show_preview;
            changed = true;
        }
        if cam.show_preview {
            if let Some(ref preview) = cam.gcode_preview {
                ui.add_space(4.0);
                ui.separator();
                ui.label("G-code preview:");
                egui::ScrollArea::vertical()
                    .max_height(180.0)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(preview)
                                .family(egui::FontFamily::Monospace)
                                .size(10.0)
                        );
                    });
            }
        }
    }

    if changed {
        ui.data_mut(|d| d.insert_temp(cam_id, cam));
    }
}

#[derive(Clone)]
struct CamState {
    tool_preset: usize,
    spindle_rpm: i32,
    feed_rate: i32,
    step_down: f64,
    path_len: Option<usize>,
    gcode_len: Option<usize>,
    gcode_preview: Option<String>,
    show_preview: bool,
}

impl Default for CamState {
    fn default() -> Self {
        Self {
            tool_preset: 1,
            spindle_rpm: 12000,
            feed_rate: 800,
            step_down: 0.5,
            path_len: None,
            gcode_len: None,
            gcode_preview: None,
            show_preview: false,
        }
    }
}

fn drawing_panel(ui: &mut egui::Ui, mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("📐 Drawing");
    ui.label(
        egui::RichText::new("Generate 2D drawings from the 3D model with hidden-line removal.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    if mesh.triangles.is_empty() {
        ui.label(
            egui::RichText::new("Load a model first (File → Import STL/STEP).")
                .color(egui::Color32::from_rgb(0xff, 0x8b, 0x8b)),
        );
        let _ = status_cb;
        return;
    }

    let dr_id = ui.id().with("dr_state");
    let mut dr: DrState = ui.data_mut(|d| d.get_temp(dr_id).unwrap_or_default());
    let mut changed = false;

    ui.label("View type:");
    egui::ComboBox::from_id_salt("dr_view_type")
        .selected_text(match dr.view_type {
            0 => "Front",
            1 => "Top",
            2 => "Right",
            _ => "Isometric",
        })
        .show_ui(ui, |ui| {
            let opts = [("Front", 0), ("Top", 1), ("Right", 2), ("Isometric", 3)];
            for (name, idx) in opts {
                if ui.selectable_label(dr.view_type == idx, name).clicked() {
                    dr.view_type = idx;
                    changed = true;
                }
            }
        });
    ui.add_space(4.0);

    ui.label("Drawing title:");
    ui.text_edit_singleline(&mut dr.title);
    ui.add_space(8.0);

    if ui.button("📐 Create Drawing").clicked() {
        let vt = match dr.view_type {
            0 => draper_drawing::ViewType::Front,
            1 => draper_drawing::ViewType::Top,
            2 => draper_drawing::ViewType::Right,
            _ => draper_drawing::ViewType::Isometric,
        };
        let title = if dr.title.is_empty() { "Drawing" } else { &dr.title };
        match draper_drawing::Drawing::single_view(mesh, vt, title) {
            Ok(drawing) => {
                let (vw, vh) = drawing.views.first()
                    .map(|v| (v.width(), v.height()))
                    .unwrap_or((0.0, 0.0));
                dr.view_width = Some(vw);
                dr.view_height = Some(vh);
                dr.created = true;
                changed = true;
                status_cb(format!("Drawing: created '{}' ({:.1}×{:.1} mm)", title, vw, vh));
            }
            Err(e) => status_cb(format!("Drawing: error — {}", e)),
        }
    }
    ui.add_space(4.0);
    if dr.created {
        if let Some(w) = dr.view_width {
            ui.label(format!("View width: {:.1} mm", w));
        }
        if let Some(h) = dr.view_height {
            ui.label(format!("View height: {:.1} mm", h));
        }
    }

    if changed {
        ui.data_mut(|d| d.insert_temp(dr_id, dr));
    }
}

#[derive(Clone)]
struct DrState {
    view_type: usize,
    title: String,
    created: bool,
    view_width: Option<f64>,
    view_height: Option<f64>,
}

impl Default for DrState {
    fn default() -> Self {
        Self {
            view_type: 3, // Isometric
            title: String::new(),
            created: false,
            view_width: None,
            view_height: None,
        }
    }
}

fn fea_panel(ui: &mut egui::Ui, mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("🔬 FEA Simulation");
    ui.label(
        egui::RichText::new("Linear static FEA on a tetrahedral mesh built from the surface mesh.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    if mesh.triangles.is_empty() {
        ui.label(
            egui::RichText::new("Load a model first (File → Import STL/STEP).")
                .color(egui::Color32::from_rgb(0xff, 0x8b, 0x8b)),
        );
        let _ = status_cb;
        return;
    }

    let fea_id = ui.id().with("fea_state");
    let mut fea: FeaState = ui.data_mut(|d| d.get_temp(fea_id).unwrap_or_default());
    let mut changed = false;

    ui.label("Material:");
    egui::ComboBox::from_id_salt("fea_material")
        .selected_text(match fea.material_preset {
            0 => "Steel (E=200 GPa, ν=0.29)",
            1 => "Aluminum (E=70 GPa, ν=0.33)",
            _ => "?",
        })
        .show_ui(ui, |ui| {
            if ui.selectable_label(fea.material_preset == 0, "Steel").clicked() { fea.material_preset = 0; changed = true; }
            if ui.selectable_label(fea.material_preset == 1, "Aluminum").clicked() { fea.material_preset = 1; changed = true; }
        });
    ui.add_space(4.0);

    ui.label("Boundary conditions:");
    ui.horizontal(|ui| {
        ui.label("Force (N):");
        ui.add(egui::DragValue::new(&mut fea.force_n).speed(1.0).range(-10000.0..=10000.0));
    });
    ui.horizontal(|ui| {
        ui.label("Fixed face id:");
        ui.add(egui::DragValue::new(&mut fea.fixed_face).range(0..=10000));
    });
    ui.add_space(4.0);

    ui.label("Mesh thickness (for tet generation):");
    ui.add(egui::DragValue::new(&mut fea.thickness).speed(0.1).range(0.1..=50.0).suffix(" mm"));
    ui.add_space(8.0);

    if ui.button("🧮 Run FEA").clicked() {
        let material = match fea.material_preset {
            0 => draper_fea::Material { youngs_modulus: 200_000.0, poissons_ratio: 0.29 },
            _ => draper_fea::Material { youngs_modulus: 70_000.0, poissons_ratio: 0.33 },
        };
        let tet_mesh = draper_fea::TetMesh::from_triangle_mesh(mesh, fea.thickness);
        let n_nodes = tet_mesh.num_nodes();
        let n_tets = tet_mesh.num_tets();
        let mut bcs = draper_fea::BoundaryConditions::new();
        bcs.add_fixed_face(fea.fixed_face as usize);
        bcs.add_force(0, 0.0, 0.0, fea.force_n); // Apply force on node 0
        let solver = draper_fea::FeaSolver::new(tet_mesh, material, bcs);
        match solver.solve() {
            Ok(result) => {
                fea.n_nodes = Some(n_nodes);
                fea.n_tets = Some(n_tets);
                fea.max_stress = Some(result.max_stress);
                fea.max_displacement = Some(result.max_displacement);
                changed = true;
                status_cb(format!("FEA: max stress = {:.2} MPa, max disp = {:.4} mm", result.max_stress, result.max_displacement));
            }
            Err(e) => status_cb(format!("FEA: error — {}", e)),
        }
    }
    ui.add_space(4.0);
    if let Some(n) = fea.n_nodes {
        ui.label(format!("Nodes: {}", n));
    }
    if let Some(n) = fea.n_tets {
        ui.label(format!("Tetrahedra: {}", n));
    }
    if let Some(s) = fea.max_stress {
        ui.label(
            egui::RichText::new(format!("Max stress: {:.2} MPa", s))
                .color(if s > 250.0 { egui::Color32::from_rgb(0xff, 0x6b, 0x6b) } else { egui::Color32::from_rgb(0xa6, 0xe3, 0xa1) })
                .strong(),
        );
    }
    if let Some(d) = fea.max_displacement {
        ui.label(format!("Max displacement: {:.4} mm", d));
    }

    if changed {
        ui.data_mut(|d| d.insert_temp(fea_id, fea));
    }
}

#[derive(Clone)]
struct FeaState {
    material_preset: usize,
    force_n: f64,
    fixed_face: i32,
    thickness: f64,
    n_nodes: Option<usize>,
    n_tets: Option<usize>,
    max_stress: Option<f64>,
    max_displacement: Option<f64>,
}

impl Default for FeaState {
    fn default() -> Self {
        Self {
            material_preset: 0,
            force_n: 100.0,
            fixed_face: 0,
            thickness: 1.0,
            n_nodes: None,
            n_tets: None,
            max_stress: None,
            max_displacement: None,
        }
    }
}

fn inspect_panel(ui: &mut egui::Ui, mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("🔍 Inspect");
    ui.label(
        egui::RichText::new("Mesh quality inspection — watertightness, Euler characteristic, manifold check.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    if mesh.triangles.is_empty() {
        ui.label(
            egui::RichText::new("Load a model first (File → Import STL/STEP).")
                .color(egui::Color32::from_rgb(0xff, 0x8b, 0x8b)),
        );
        let _ = status_cb;
        return;
    }

    if ui.button("🔍 Run Inspection").clicked() {
        let report = draper_mesh::watertight::validate_watertight(mesh, false);
        status_cb(format!(
            "Inspect: watertight={} boundary_edges={} non_manifold_edges={}",
            report.is_watertight(), report.boundary_edge_count, report.non_manifold_edge_count
        ));
    }

    ui.add_space(8.0);
    ui.label("Quick stats:");
    let report = draper_mesh::watertight::validate_watertight(mesh, false);
    ui.label(format!("Vertices: {}", mesh.vertices.len()));
    ui.label(format!("Triangles: {}", mesh.triangles.len()));
    ui.label(format!("Edges: {}", report.edge_count));
    ui.label(format!("Watertight: {}", if report.is_watertight() { "✓ yes" } else { "✗ no" }));
    ui.label(format!("Boundary edges: {}", report.boundary_edge_count));
    ui.label(format!("Non-manifold edges: {}", report.non_manifold_edge_count));
    ui.label(format!("Degenerate triangles: {}", report.degenerate_triangle_count));
    ui.label(format!("Duplicate triangles: {}", report.duplicate_triangle_count));
    ui.label(format!("Euler characteristic: {}", report.euler_characteristic));
    if report.euler_characteristic == 2 {
        ui.label(
            egui::RichText::new("→ Topology: sphere-like (genus 0) ✓")
                .color(egui::Color32::from_rgb(0xa6, 0xe3, 0xa1)),
        );
    } else if report.euler_characteristic < 2 {
        let genus = (2 - report.euler_characteristic) / 2;
        ui.label(format!("→ Topology: genus {} (handles)", genus));
    }
}

fn ai_panel(ui: &mut egui::Ui, mesh: &TriangleMesh, status_cb: &mut dyn FnMut(String)) {
    ui.heading("🤖 AI");
    ui.label(
        egui::RichText::new("Design review and defect detection powered by draper-ai.")
            .small()
            .color(egui::Color32::from_rgb(0x9a, 0x9a, 0xb0)),
    );
    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    if mesh.triangles.is_empty() {
        ui.label(
            egui::RichText::new("Load a model first (File → Import STL/STEP).")
                .color(egui::Color32::from_rgb(0xff, 0x8b, 0x8b)),
        );
        let _ = status_cb;
        return;
    }

    ui.label("Review profile:");
    let mut profile: i32 = ui.data(|d| d.get_temp(ui.id().with("ai_profile")).unwrap_or(0));
    egui::ComboBox::from_id_salt("ai_profile_cb")
        .selected_text(match profile {
            0 => "CNC milling",
            1 => "FDM 3D printing",
            _ => "Injection molding",
        })
        .show_ui(ui, |ui| {
            if ui.selectable_label(profile == 0, "CNC milling").clicked() { profile = 0; }
            if ui.selectable_label(profile == 1, "FDM 3D printing").clicked() { profile = 1; }
            if ui.selectable_label(profile == 2, "Injection molding").clicked() { profile = 2; }
        });
    ui.data_mut(|d| d.insert_temp(ui.id().with("ai_profile"), profile));
    ui.add_space(4.0);

    if ui.button("🔍 Run design review").clicked() {
        let config = match profile {
            0 => draper_ai::design_review::ReviewConfig::cnc_milling(),
            1 => draper_ai::design_review::ReviewConfig::fdm_printing(),
            _ => draper_ai::design_review::ReviewConfig::injection_molding(),
        };
        let reviewer = draper_ai::design_review::DesignReviewer::new(config);
        let report = reviewer.review(mesh);
        let n_issues = report.results.len();
        status_cb(format!("AI: review complete — {} issue(s) found", n_issues));
        ui.data_mut(|d| d.insert_temp(ui.id().with("ai_report"), report));
    }
    ui.add_space(4.0);

    // Show last report if present
    if let Some(report) = ui.data(|d| d.get_temp::<draper_ai::design_review::ReviewReport>(ui.id().with("ai_report"))) {
        ui.separator();
        ui.label(format!("Issues: {} (errors={}, warnings={})",
            report.results.len(), report.error_count, report.warning_count));
        ui.label(if report.passed {
            "Status: PASSED ✓"
        } else {
            "Status: FAILED ✗"
        });
        egui::ScrollArea::vertical()
            .max_height(280.0)
            .show(ui, |ui| {
                for issue in &report.results {
                    ui.horizontal(|ui| {
                        let color = match issue.severity {
                            draper_ai::design_review::Severity::Error => egui::Color32::from_rgb(0xff, 0x6b, 0x6b),
                            draper_ai::design_review::Severity::Warning => egui::Color32::from_rgb(0xf9, 0xe2, 0xaf),
                            draper_ai::design_review::Severity::Info => egui::Color32::from_rgb(0xa6, 0xe3, 0xa1),
                        };
                        ui.colored_label(color, format!("[{}]", issue.severity.name().to_uppercase()));
                        ui.label(format!("{}: {}", issue.check_name, issue.message));
                    });
                }
            });
    }
}
