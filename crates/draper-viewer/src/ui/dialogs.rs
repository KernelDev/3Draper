// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Dialog windows — Phase 6.
//!
//! Modal dialogs for Options, Customize, Primitives, About, etc.

use eframe::egui;

#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum DialogType {
    #[default]
    None,
    Options,
    About,
    InsertPrimitive(PrimitiveType),
    CommandSearch,
    Plugins,
    MaterialEditor,
    Performance,
    ShortcutEditor,
    /// Render Settings dialog (mockup 62).
    RenderSettings,
    /// Customize dialog (mockup 52).
    Customize,
    /// NC Code Viewer dialog (mockup 84).
    NcCodeViewer,
    /// Print/Plot dialog (mockup 74).
    PrintPlot,
    /// Constraint Diagnostics dialog (mockup 60).
    ConstraintDiagnostics,
    /// Macro Recorder dialog (mockup 70).
    MacroRecorder,
}

/// Primitive type for the Insert dialog.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrimitiveType {
    Box,
    Sphere,
    Cylinder,
    Cone,
    Torus,
}

impl PrimitiveType {
    pub fn label(&self) -> &'static str {
        match self {
            PrimitiveType::Box => "Box",
            PrimitiveType::Sphere => "Sphere",
            PrimitiveType::Cylinder => "Cylinder",
            PrimitiveType::Cone => "Cone",
            PrimitiveType::Torus => "Torus",
        }
    }

    pub fn params(&self) -> &'static [(&'static str, f64, f64, f64)] {
        // (name, default, min, max)
        match self {
            PrimitiveType::Box => &[("Width", 100.0, 0.1, 10000.0), ("Height", 100.0, 0.1, 10000.0), ("Depth", 100.0, 0.1, 10000.0)],
            PrimitiveType::Sphere => &[("Radius", 50.0, 0.1, 5000.0)],
            PrimitiveType::Cylinder => &[("Radius", 50.0, 0.1, 5000.0), ("Height", 100.0, 0.1, 10000.0)],
            PrimitiveType::Cone => &[("Bottom Radius", 50.0, 0.1, 5000.0), ("Top Radius", 0.0, 0.0, 5000.0), ("Height", 100.0, 0.1, 10000.0)],
            PrimitiveType::Torus => &[("Major Radius", 50.0, 0.1, 5000.0), ("Minor Radius", 10.0, 0.1, 1000.0)],
        }
    }
}

/// Render a modal dialog if one is open.
/// Returns Some(action) when the dialog produces a result.
pub fn render_dialog(ctx: &egui::Context, dialog: &mut DialogType) -> Option<DialogAction> {
    if *dialog == DialogType::None {
        return None;
    }

    let mut action = None;
    let mut close = false;

    egui::Window::new(dialog_title(dialog))
        .open(&mut {
            let v = *dialog != DialogType::None;
            v
        })
        .resizable(true)
        .collapsible(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            match dialog {
                DialogType::Options => action = render_options_dialog(ui, &mut close),
                DialogType::About => action = render_about_dialog(ui, &mut close),
                DialogType::InsertPrimitive(pt) => action = render_primitive_dialog(ui, pt, &mut close),
                DialogType::Plugins => action = render_plugins_dialog(ui, &mut close),
                DialogType::MaterialEditor => action = render_material_dialog(ui, &mut close),
                DialogType::Performance => action = render_performance_dialog(ui, &mut close),
                DialogType::ShortcutEditor => action = render_shortcut_dialog(ui, &mut close),
                DialogType::CommandSearch => action = render_command_search_dialog(ui, &mut close),
                DialogType::RenderSettings => action = render_render_settings_dialog(ui, &mut close),
                DialogType::Customize => action = render_customize_dialog(ui, &mut close),
                DialogType::NcCodeViewer => action = render_nc_code_viewer_dialog(ui, &mut close),
                DialogType::PrintPlot => action = render_print_plot_dialog(ui, &mut close),
                DialogType::ConstraintDiagnostics => action = render_constraint_diagnostics_dialog(ui, &mut close),
                DialogType::MacroRecorder => action = render_macro_recorder_dialog(ui, &mut close),
                DialogType::None => {}
            }
        });

    if close {
        *dialog = DialogType::None;
    }

    action
}

fn dialog_title(dialog: &DialogType) -> String {
    match dialog {
        DialogType::Options => "Options".to_string(),
        DialogType::About => "About BRepCAD".to_string(),
        DialogType::InsertPrimitive(pt) => format!("Insert {}", pt.label()),
        DialogType::Plugins => "Plugins Manager".to_string(),
        DialogType::MaterialEditor => "Material Editor".to_string(),
        DialogType::Performance => "Performance Monitor".to_string(),
        DialogType::ShortcutEditor => "Shortcut Editor".to_string(),
        DialogType::CommandSearch => "Command Search".to_string(),
        DialogType::RenderSettings => "Render Settings".to_string(),
        DialogType::Customize => "Customize".to_string(),
        DialogType::NcCodeViewer => "NC Code Viewer".to_string(),
        DialogType::PrintPlot => "Print / Plot".to_string(),
        DialogType::ConstraintDiagnostics => "Constraint Diagnostics".to_string(),
        DialogType::MacroRecorder => "Macro Recorder".to_string(),
        DialogType::None => String::new(),
    }
}

/// Actions emitted by dialogs.
#[derive(Clone, Debug)]
pub enum DialogAction {
    InsertPrimitive(PrimitiveType, Vec<f64>),
    Close,
}

fn render_options_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut selected_section = 0;
    let sections = ["General", "Display", "File Locations", "Hotkeys", "Theme", "Advanced", "Plugins", "AI", "Performance", "Cloud Sync"];

    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            for (i, s) in sections.iter().enumerate() {
                ui.selectable_value(&mut selected_section, i, *s);
            }
        });
        ui.separator();
        ui.vertical(|ui| {
            ui.label(format!("{} settings", sections[selected_section]));
            ui.separator();
            match selected_section {
                0 => { // General
                    ui.checkbox(&mut true, "Auto-save every 5 minutes");
                    ui.checkbox(&mut true, "Show splash screen");
                    ui.checkbox(&mut false, "Check for updates on startup");
                    ui.horizontal(|ui| { ui.label("Units:"); ui.label("mm (default)"); });
                }
                1 => { // Display
                    ui.checkbox(&mut true, "Anti-aliasing");
                    ui.checkbox(&mut true, "Ambient occlusion");
                    ui.checkbox(&mut true, "Show grid");
                    ui.checkbox(&mut true, "Show axis triad");
                    ui.checkbox(&mut true, "Show view cube");
                    ui.add(egui::Slider::new(&mut 60.0_f32, 30.0..=144.0).text("Max FPS"));
                }
                _ => { ui.label("Settings coming soon..."); }
            }
        });
    });

    ui.separator();
    ui.horizontal(|ui| {
        if ui.button("OK").clicked() { *close = true; }
        if ui.button("Cancel").clicked() { *close = true; }
        if ui.button("Apply").clicked() {}
    });
    None
}

fn render_about_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    ui.vertical_centered(|ui| {
        ui.add_space(10.0);
        ui.heading("BRepCAD");
        ui.label("3D CAD/CAE/CAM Application");
        ui.add_space(5.0);
        ui.label(format!("Version: {}", env!("CARGO_PKG_VERSION")));
        ui.label("Built on 3Draper engine");
        ui.add_space(10.0);
        ui.label("Powered by Rust + egui + OpenGL");
        ui.add_space(10.0);
        if ui.button("Close").clicked() { *close = true; }
    });
    None
}

fn render_primitive_dialog(ui: &mut egui::Ui, pt: &PrimitiveType, close: &mut bool) -> Option<DialogAction> {
    let params = pt.params();
    let mut result = None;

    // Use egui memory to persist slider values across frames
    let memory_id = ui.make_persistent_id(format!("primitive_dialog_{:?}", pt));
    let mut values: Vec<f64> = ui.memory(|m| {
        let v: Option<&Vec<f64>> = m.data.get_temp(memory_id);
        v.cloned().unwrap_or_else(|| {
            params.iter().map(|p| p.1).collect::<Vec<f64>>()
        })
    });

    // Ensure values has correct length (in case primitive type changed)
    if values.len() != params.len() {
        values = params.iter().map(|p| p.1).collect();
    }

    ui.vertical(|ui| {
        ui.label(format!("Insert {} — specify dimensions:", pt.label()));
        ui.separator();

        for (i, (name, _default, min, max)) in params.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!("{}:", name));
                ui.add(egui::Slider::new(&mut values[i], *min..=*max).suffix(" mm"));
            });
        }

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("OK").clicked() {
                *close = true;
                result = Some(DialogAction::InsertPrimitive(*pt, values.clone()));
            }
            if ui.button("Cancel").clicked() { *close = true; }
        });
    });

    // Store values back to memory for next frame
    ui.memory_mut(|m| m.data.insert_temp(memory_id, values));

    result
}

fn render_plugins_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let tabs = ["Installed", "Marketplace", "Settings"];
    let mut active_tab = 0;

    ui.horizontal(|ui| {
        for (i, t) in tabs.iter().enumerate() {
            ui.selectable_value(&mut active_tab, i, *t);
        }
    });
    ui.separator();

    match active_tab {
        0 => {
            ui.label("Installed Plugins:");
            let plugins = ["STEP Import/Export", "STL Import/Export", "OBJ Import", "NURBS Toolkit", "Mesh Repair", "GD&T Viewer", "CAM Postprocessor", "Theme: Dark"];
            for p in &plugins {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut true, "");
                    ui.label(*p);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.small_button("Settings");
                        ui.small_button("Disable");
                    });
                });
            }
        }
        1 => { ui.label("Marketplace coming soon..."); }
        _ => { ui.label("Plugin settings..."); }
    }

    ui.separator();
    if ui.button("Close").clicked() { *close = true; }
    None
}

fn render_material_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let categories = ["Metals", "Plastics", "Ceramics", "Composites", "Wood", "Glass", "Custom"];
    let mut selected_cat = 0;

    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            for (i, c) in categories.iter().enumerate() {
                ui.selectable_value(&mut selected_cat, i, *c);
            }
        });
        ui.separator();
        ui.vertical(|ui| {
            ui.label("Material Properties:");
            ui.horizontal(|ui| { ui.label("Name:"); ui.text_edit_singleline(&mut String::from("Steel AISI 1045")); });
            ui.horizontal(|ui| { ui.label("Density:"); ui.add(egui::DragValue::new(&mut 7850.0).suffix(" kg/m³")); });
            ui.horizontal(|ui| { ui.label("Young's Modulus:"); ui.add(egui::DragValue::new(&mut 200000.0).suffix(" MPa")); });
            ui.horizontal(|ui| { ui.label("Poisson's Ratio:"); ui.add(egui::DragValue::new(&mut 0.29).range(0.0..=0.5)); });
            ui.horizontal(|ui| { ui.label("Color:"); ui.color_edit_button_srgb(&mut [120, 120, 130]); });
        });
    });

    ui.separator();
    ui.horizontal(|ui| {
        if ui.button("Assign").clicked() { *close = true; }
        if ui.button("Cancel").clicked() { *close = true; }
    });
    None
}

fn render_performance_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    ui.vertical(|ui| {
        ui.label("Performance Monitor");
        ui.separator();
        ui.label(format!("FPS: 60.0"));
        ui.label(format!("Draw calls: 42"));
        ui.label(format!("Vertices: 18,428"));
        ui.label(format!("Triangles: 24,029"));
        ui.label(format!("Memory: 128 MB"));
        ui.separator();
        ui.label("Alerts: None");
        ui.separator();
        if ui.button("Close").clicked() { *close = true; }
    });
    None
}

fn render_shortcut_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    ui.vertical(|ui| {
        ui.label("Shortcut Editor — 245 commands");
        ui.separator();
        egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
            let commands = [
                ("New", "Ctrl+N"), ("Open", "Ctrl+O"), ("Save", "Ctrl+S"),
                ("Undo", "Ctrl+Z"), ("Redo", "Ctrl+Shift+Z"),
                ("Copy", "Ctrl+C"), ("Paste", "Ctrl+V"), ("Cut", "Ctrl+X"),
                ("Fit", "F"), ("ISO View", "0"),
                ("Sketch Mode", "S"), ("Line", "L"), ("Circle", "C"),
            ];
            for (cmd, shortcut) in &commands {
                ui.horizontal(|ui| {
                    ui.label(*cmd);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(*shortcut);
                        ui.small_button("Edit");
                    });
                });
            }
        });
        ui.separator();
        if ui.button("Close").clicked() { *close = true; }
    });
    None
}

fn render_command_search_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut query = String::new();
    ui.vertical(|ui| {
        ui.text_edit_singleline(&mut query);
        ui.separator();
        ui.label("Type to search commands...");
        ui.label("(Use Ctrl+Shift+P for the command palette)");
        ui.separator();
        if ui.button("Close").clicked() { *close = true; }
    });
    None
}

// ============================================================
// Phase 4: New dialogs (mockups 62, 52, 84, 74, 60, 70)
// ============================================================

/// Render Settings dialog (mockup 62).
///
/// Controls viewport rendering quality: anti-aliasing, shadows,
/// ambient occlusion, background color, edge rendering.
fn render_render_settings_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut quality = 1; // 0=Low, 1=Medium, 2=High
    let mut shadows = true;
    let mut ambient_occlusion = false;
    let mut anti_alias = true;
    let mut show_edges = true;
    let mut show_grid = true;
    let mut bg_color = [0.1f32, 0.1, 0.17, 1.0];
    let mut edge_color = [0.54, 0.71, 0.98, 1.0]; // Catppuccin blue

    ui.vertical(|ui| {
        ui.heading("Render Settings");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Quality:");
            ui.radio_value(&mut quality, 0, "Low");
            ui.radio_value(&mut quality, 1, "Medium");
            ui.radio_value(&mut quality, 2, "High");
        });

        ui.separator();
        ui.label("Effects:");
        ui.checkbox(&mut shadows, "Shadows");
        ui.checkbox(&mut ambient_occlusion, "Ambient Occlusion (SSAO)");
        ui.checkbox(&mut anti_alias, "Anti-aliasing (MSAA 4×)");

        ui.separator();
        ui.label("Display:");
        ui.checkbox(&mut show_edges, "Show edges");
        ui.checkbox(&mut show_grid, "Show grid");

        ui.separator();
        ui.label("Colors:");
        ui.horizontal(|ui| {
            ui.label("Background:");
            ui.color_edit_button_rgba_unmultiplied(&mut bg_color);
        });
        ui.horizontal(|ui| {
            ui.label("Edge color:");
            ui.color_edit_button_rgba_unmultiplied(&mut edge_color);
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Apply").clicked() {
                // In production, these would update ViewerApp fields
            }
            if ui.button("Reset to Defaults").clicked() {
                quality = 1;
                shadows = true;
                ambient_occlusion = false;
                anti_alias = true;
                show_edges = true;
                show_grid = true;
                bg_color = [0.1, 0.1, 0.17, 1.0];
                edge_color = [0.54, 0.71, 0.98, 1.0];
            }
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Customize dialog (mockup 52).
///
/// Tabs: Ribbon customization, Shortcut editor, Toolbar layout.
fn render_customize_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut selected_tab = 0;
    let tabs = ["Ribbon", "Shortcuts", "Toolbars", "Quick Access"];

    ui.vertical(|ui| {
        ui.heading("Customize");
        ui.separator();

        ui.horizontal(|ui| {
            for (i, tab) in tabs.iter().enumerate() {
                if ui.selectable_label(selected_tab == i, *tab).clicked() {
                    selected_tab = i;
                }
            }
        });
        ui.separator();

        match selected_tab {
            0 => {
                // Ribbon customization
                ui.label("Ribbon Tabs (drag to reorder):");
                let ribbon_tabs = ["File", "Home", "Sketch", "Insert", "Modify",
                    "Sheet Metal", "Assembly", "CAM", "Drawing", "Simulation",
                    "Inspect", "AI", "Tools", "View", "Surface"];
                for tab in &ribbon_tabs {
                    ui.horizontal(|ui| {
                        ui.label("≡");
                        ui.label(*tab);
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.checkbox(&mut true, "Visible");
                        });
                    });
                }
            }
            1 => {
                // Shortcuts
                ui.label("Keyboard Shortcuts:");
                egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                    let shortcuts = [
                        ("New", "Ctrl+N"), ("Open", "Ctrl+O"), ("Save", "Ctrl+S"),
                        ("Undo", "Ctrl+Z"), ("Redo", "Ctrl+Shift+Z"),
                        ("Copy", "Ctrl+C"), ("Paste", "Ctrl+V"), ("Cut", "Ctrl+X"),
                        ("Fit", "F"), ("ISO View", "0"),
                        ("Sketch Mode", "S"), ("Line", "L"), ("Circle", "C"),
                        ("Command Palette", "Ctrl+Shift+P"), ("Options", "Ctrl+,"),
                    ];
                    for (cmd, sc) in &shortcuts {
                        ui.horizontal(|ui| {
                            ui.label(*cmd);
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                ui.label(*sc);
                                if ui.small_button("Edit").clicked() {}
                            });
                        });
                    }
                });
            }
            2 => {
                // Toolbars
                ui.label("Toolbar Layout:");
                ui.checkbox(&mut true, "Show menu bar");
                ui.checkbox(&mut true, "Show ribbon");
                ui.checkbox(&mut true, "Show status bar");
                ui.checkbox(&mut true, "Show left panel (Browser)");
                ui.checkbox(&mut true, "Show right panel (Properties)");
                ui.checkbox(&mut false, "Show timeline panel");
            }
            3 => {
                // Quick Access Toolbar
                ui.label("Quick Access Toolbar items:");
                ui.checkbox(&mut true, "Undo");
                ui.checkbox(&mut true, "Redo");
                ui.checkbox(&mut true, "Save");
                ui.checkbox(&mut false, "Open");
                ui.checkbox(&mut false, "Print");
            }
            _ => {}
        }

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Apply").clicked() {}
            if ui.button("Reset").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// NC Code Viewer dialog (mockup 84).
///
/// Displays generated G-code with syntax highlighting.
/// Reads from ViewerApp.brepcad_cam_gcode.
fn render_nc_code_viewer_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    // In production, this would read from self.brepcad_cam_gcode.
    // For now, show a placeholder with sample G-code.
    let sample_gcode = "; BRepCAD G-code (Fanuc)\n\
; 1 operations\n\
G90 G21 G17\n\
G54\n\
M3 S1000\n\
; Op 1 : Profile\n\
G0 Z10\n\
G0 X0.000 Y0.000\n\
G1 Z-5.000 F100\n\
G1 X100.000 F200\n\
G1 Y80.000\n\
G1 X0.000\n\
G1 Y0.000\n\
G0 Z10\n\
M5\n\
M30\n";

    ui.vertical(|ui| {
        ui.heading("NC Code Viewer");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Dialect:");
            ui.label("Fanuc");
            ui.separator();
            ui.label("Lines: 16");
            ui.separator();
            ui.label("Est. time: 2.3 min");
        });

        ui.separator();

        // G-code with basic syntax highlighting via monospace + color
        egui::ScrollArea::vertical()
            .max_height(350.0)
            .show(ui, |ui| {
                for line in sample_gcode.lines() {
                    let color = if line.starts_with(';') {
                        egui::Color32::from_rgb(108, 112, 134) // Comment gray
                    } else if line.starts_with('G') || line.starts_with('M') {
                        egui::Color32::from_rgb(137, 180, 250) // G/M codes blue
                    } else if line.starts_with('X') || line.starts_with('Y') || line.starts_with('Z') {
                        egui::Color32::from_rgb(166, 227, 161) // Coordinates green
                    } else if line.starts_with('F') || line.starts_with('S') {
                        egui::Color32::from_rgb(249, 226, 175) // Feed/speed yellow
                    } else {
                        egui::Color32::from_rgb(205, 214, 244) // Default text
                    };
                    ui.label(egui::RichText::new(line)
                        .family(egui::FontFamily::Monospace)
                        .size(11.0)
                        .color(color));
                }
            });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Copy to Clipboard").clicked() {
                ui.ctx().copy_text(sample_gcode.to_string());
            }
            if ui.button("Save As...").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Print/Plot dialog (mockup 74).
///
/// Paper size, orientation, scale, preview.
fn render_print_plot_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut paper_size = 3; // 0=A0, 1=A1, 2=A2, 3=A3, 4=A4
    let mut orientation = 0; // 0=Portrait, 1=Landscape
    let mut scale = 1.0f32; // 1:1
    let mut copies = 1;
    let mut printer = "Default Printer".to_string();

    ui.vertical(|ui| {
        ui.heading("Print / Plot");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Printer:");
            ui.text_edit_singleline(&mut printer);
            ui.button("Browse...");
        });

        ui.separator();
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label("Paper Size:");
                egui::ComboBox::from_id_salt("paper_size_combo")
                    .selected_text(["A0 (841×1189)", "A1 (594×841)", "A2 (420×594)",
                                    "A3 (297×420)", "A4 (210×297)"][paper_size])
                    .show_ui(ui, |ui| {
                        for (i, label) in ["A0 (841×1189)", "A1 (594×841)", "A2 (420×594)",
                                            "A3 (297×420)", "A4 (210×297)"].iter().enumerate() {
                            ui.selectable_value(&mut paper_size, i, *label);
                        }
                    });
            });
            ui.vertical(|ui| {
                ui.label("Orientation:");
                ui.radio_value(&mut orientation, 0, "Portrait");
                ui.radio_value(&mut orientation, 1, "Landscape");
            });
        });

        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Scale:");
            ui.add(egui::Slider::new(&mut scale, 0.1..=10.0).text(": 1"));
            ui.label(format!("({:.1}:1)", scale));
        });

        ui.horizontal(|ui| {
            ui.label("Copies:");
            ui.add(egui::DragValue::new(&mut copies).range(1..=99));
        });

        ui.separator();
        ui.checkbox(&mut true, "Fit to paper");
        ui.checkbox(&mut false, "Print grid");
        ui.checkbox(&mut true, "Print title block");

        ui.separator();
        ui.label("Preview:");
        let preview_frame = egui::Frame::new()
            .fill(egui::Color32::from_rgb(0x1e, 0x1e, 0x2e))
            .stroke(egui::Stroke::new(1.0, egui::Color32::GRAY))
            .inner_margin(egui::Margin::symmetric(40, 30));
        preview_frame.show(ui, |ui| {
            ui.label(egui::RichText::new("📄 Drawing Preview")
                .size(14.0)
                .color(egui::Color32::from_rgb(0x89, 0xb4, 0xfa)));
            ui.label(egui::RichText::new(format!("{} × {}mm", 297, 420))
                .size(10.0)
                .color(egui::Color32::GRAY));
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Print").clicked() {}
            if ui.button("Export PDF").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Constraint Diagnostics dialog (mockup 60).
///
/// Shows sketch constraint status, violations, DOF count.
fn render_constraint_diagnostics_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    ui.vertical(|ui| {
        ui.heading("Constraint Diagnostics");
        ui.separator();

        // Summary
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label(egui::RichText::new("DOF: 2").strong());
                ui.label("Degrees of Freedom");
            });
            ui.separator();
            ui.vertical(|ui| {
                ui.label(egui::RichText::new("3").strong().color(egui::Color32::from_rgb(80, 180, 80)));
                ui.label("Constraints");
            });
            ui.separator();
            ui.vertical(|ui| {
                ui.label(egui::RichText::new("0").strong().color(egui::Color32::from_rgb(220, 80, 80)));
                ui.label("Violations");
            });
        });

        ui.separator();
        ui.label("Constraints:");
        egui::ScrollArea::vertical().max_height(150.0).show(ui, |ui| {
            let constraints = [
                ("✓", "Coincident", "P1 = P2", "OK"),
                ("✓", "Horizontal", "L1", "OK"),
                ("✓", "Vertical", "L2", "OK"),
                ("⚠", "Parallel", "L1 ∥ L3", "Over-constrained"),
            ];
            for (icon, ctype, desc, status) in &constraints {
                let color = if *icon == "✓" {
                    egui::Color32::from_rgb(80, 180, 80)
                } else {
                    egui::Color32::from_rgb(220, 180, 80)
                };
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(*icon).color(color));
                    ui.label(format!("{}: {}", ctype, desc));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(egui::RichText::new(*status).size(10.0).color(color));
                    });
                });
            }
        });

        ui.separator();
        ui.label("Suggestions:");
        ui.label("• Add 1 more constraint to fully define the sketch");
        ui.label("• Remove 'Parallel L1 ∥ L3' (redundant with existing)");

        ui.separator();
        if ui.button("Close").clicked() { *close = true; }
    });
    None
}

/// Macro Recorder dialog (mockup 70).
///
/// Records user actions to a script for replay.
fn render_macro_recorder_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut recording = false;
    let mut macro_name = "macro1".to_string();
    let mut macro_lang = 0; // 0=Python, 1=Lua

    ui.vertical(|ui| {
        ui.heading("Macro Recorder");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Name:");
            ui.text_edit_singleline(&mut macro_name);
        });

        ui.horizontal(|ui| {
            ui.label("Language:");
            ui.radio_value(&mut macro_lang, 0, "Python");
            ui.radio_value(&mut macro_lang, 1, "Lua");
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.add(egui::Button::new(if recording { "⏹ Stop" } else { "⏺ Record" }))
                .clicked() {
                recording = !recording;
            }
            if ui.button("▶ Play").clicked() {}
            if ui.button("⏸ Pause").clicked() {}
        });

        ui.separator();
        ui.label("Recorded Actions:");
        egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
            let actions = [
                "1. [00:00] Select Box primitive",
                "2. [00:01] Set width = 100.0",
                "3. [00:02] Set height = 80.0",
                "4. [00:03] Set depth = 60.0",
                "5. [00:04] Click OK",
                "6. [00:05] View → ISO",
            ];
            for action in &actions {
                ui.label(egui::RichText::new(*action)
                    .family(egui::FontFamily::Monospace)
                    .size(10.0));
            }
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Save Macro").clicked() {}
            if ui.button("Export Script").clicked() {}
            if ui.button("Clear").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}
