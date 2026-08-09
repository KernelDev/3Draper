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
    /// Phase 3.2: now carries the actual G-code string + dialect label
    /// so the dialog can show real generated code, not a placeholder.
    NcCodeViewer {
        /// G-code text to display (multiple lines, separated by '\n').
        gcode: String,
        /// Dialect label (e.g. "Fanuc", "Siemens", "Haas").
        dialect: String,
    },
    /// Print/Plot dialog (mockup 74).
    PrintPlot,
    /// Constraint Diagnostics dialog (mockup 60).
    ConstraintDiagnostics,
    /// Macro Recorder dialog (mockup 70).
    MacroRecorder,
    /// BOM Editor dialog (mockup 90).
    BomEditor,
    /// Layer Manager dialog (mockup 89).
    LayerManager,
    /// Tool Library dialog (mockup 83).
    ToolLibrary,
    /// FEA Mesh Control dialog (mockup 85).
    FeaMeshControl,
    /// Title Block Editor dialog (mockup 87).
    TitleBlockEditor,
    // ─── Phase 2.2: Medium-priority dialogs ───
    /// Param Search/Replace dialog (mockup 91).
    ParamSearchReplace,
    /// Revision Table dialog (mockup 88).
    RevisionTable,
    /// Tutorial Browser dialog (mockup 72).
    TutorialBrowser,
    /// Crash Recovery dialog (mockup 76).
    CrashRecovery,
    /// Onboarding Wizard dialog (mockup 77).
    OnboardingWizard,
    // ─── Phase 2.3: Low-priority dialogs ───
    /// Update dialog (mockup 58).
    UpdateCheck,
    /// License dialog (mockup 75).
    LicenseInfo,
    /// Mold Catalog dialog (mockup 61).
    MoldCatalog,
    /// Modal Plotter dialog (mockup 86).
    ModalPlotter,
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
                DialogType::NcCodeViewer { gcode, dialect } => {
                    action = render_nc_code_viewer_dialog(ui, &mut close, gcode, dialect);
                }
                DialogType::PrintPlot => action = render_print_plot_dialog(ui, &mut close),
                DialogType::ConstraintDiagnostics => action = render_constraint_diagnostics_dialog(ui, &mut close),
                DialogType::MacroRecorder => action = render_macro_recorder_dialog(ui, &mut close),
                DialogType::BomEditor => action = render_bom_editor_dialog(ui, &mut close),
                DialogType::LayerManager => action = render_layer_manager_dialog(ui, &mut close),
                DialogType::ToolLibrary => action = render_tool_library_dialog(ui, &mut close),
                DialogType::FeaMeshControl => action = render_fea_mesh_control_dialog(ui, &mut close),
                DialogType::TitleBlockEditor => action = render_title_block_editor_dialog(ui, &mut close),
                DialogType::ParamSearchReplace => action = render_param_search_replace_dialog(ui, &mut close),
                DialogType::RevisionTable => action = render_revision_table_dialog(ui, &mut close),
                DialogType::TutorialBrowser => action = render_tutorial_browser_dialog(ui, &mut close),
                DialogType::CrashRecovery => action = render_crash_recovery_dialog(ui, &mut close),
                DialogType::OnboardingWizard => action = render_onboarding_wizard_dialog(ui, &mut close),
                DialogType::UpdateCheck => action = render_update_check_dialog(ui, &mut close),
                DialogType::LicenseInfo => action = render_license_info_dialog(ui, &mut close),
                DialogType::MoldCatalog => action = render_mold_catalog_dialog(ui, &mut close),
                DialogType::ModalPlotter => action = render_modal_plotter_dialog(ui, &mut close),
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
        DialogType::NcCodeViewer { .. } => "NC Code Viewer".to_string(),
        DialogType::PrintPlot => "Print / Plot".to_string(),
        DialogType::ConstraintDiagnostics => "Constraint Diagnostics".to_string(),
        DialogType::MacroRecorder => "Macro Recorder".to_string(),
        DialogType::BomEditor => "Bill of Materials".to_string(),
        DialogType::LayerManager => "Layer Manager".to_string(),
        DialogType::ToolLibrary => "Tool Library".to_string(),
        DialogType::FeaMeshControl => "FEA Mesh Control".to_string(),
        DialogType::TitleBlockEditor => "Title Block Editor".to_string(),
        DialogType::ParamSearchReplace => "Parameter Search & Replace".to_string(),
        DialogType::RevisionTable => "Revision Table".to_string(),
        DialogType::TutorialBrowser => "Tutorial Browser".to_string(),
        DialogType::CrashRecovery => "Crash Recovery".to_string(),
        DialogType::OnboardingWizard => "Welcome to BRepCAD".to_string(),
        DialogType::UpdateCheck => "Check for Updates".to_string(),
        DialogType::LicenseInfo => "License Information".to_string(),
        DialogType::MoldCatalog => "Mold Base Catalog".to_string(),
        DialogType::ModalPlotter => "Modal Plotter".to_string(),
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
/// Phase 3.2: Now reads from the actual CAM postprocessor output
/// (passed in via DialogType::NcCodeViewer { gcode, dialect }).
fn render_nc_code_viewer_dialog(
    ui: &mut egui::Ui,
    close: &mut bool,
    gcode: &str,
    dialect: &str,
) -> Option<DialogAction> {
    // If no G-code has been generated yet, show a helpful hint.
    let display_gcode = if gcode.is_empty() {
        "; No G-code generated yet.\n; Run CAM → Post Process → (Fanuc/Siemens/Haas/...) to generate G-code,\n; then re-open this viewer."
    } else {
        gcode
    };

    let line_count = display_gcode.lines().count();
    let estimated_min = (line_count as f32 * 0.05).max(0.1);

    ui.vertical(|ui| {
        ui.heading("NC Code Viewer");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Dialect:");
            ui.label(dialect);
            ui.separator();
            ui.label(format!("Lines: {}", line_count));
            ui.separator();
            ui.label(format!("Est. time: {:.1} min", estimated_min));
        });

        ui.separator();

        egui::ScrollArea::vertical()
            .max_height(350.0)
            .show(ui, |ui| {
                for line in display_gcode.lines() {
                    let color = classify_gcode_line(line);
                    ui.label(egui::RichText::new(line)
                        .family(egui::FontFamily::Monospace)
                        .size(11.0)
                        .color(color));
                }
            });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Copy to Clipboard").clicked() {
                ui.ctx().copy_text(display_gcode.to_string());
            }
            if ui.button("Save As...").clicked() {
                ui.ctx().copy_text(display_gcode.to_string());
            }
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Classify a single G-code line for syntax highlighting.
fn classify_gcode_line(line: &str) -> egui::Color32 {
    let trimmed = line.trim_start();
    if trimmed.starts_with(';') || trimmed.starts_with('(') {
        egui::Color32::from_rgb(108, 112, 134) // gray comment
    } else if trimmed.starts_with('G') {
        egui::Color32::from_rgb(137, 180, 250) // blue G-code
    } else if trimmed.starts_with('M') {
        egui::Color32::from_rgb(137, 180, 250) // blue M-code
    } else if trimmed.starts_with('T') {
        egui::Color32::from_rgb(245, 194, 231) // pink tool change
    } else if trimmed.starts_with('X') || trimmed.starts_with('Y') || trimmed.starts_with('Z')
        || trimmed.starts_with('I') || trimmed.starts_with('J') || trimmed.starts_with('K')
        || trimmed.starts_with('R')
    {
        egui::Color32::from_rgb(166, 227, 161) // green coordinates
    } else if trimmed.starts_with('F') || trimmed.starts_with('S') {
        egui::Color32::from_rgb(249, 226, 175) // yellow feed/speed
    } else if trimmed.starts_with('N') {
        egui::Color32::from_rgb(148, 226, 213) // cyan line number
    } else {
        egui::Color32::from_rgb(205, 214, 244) // default text
    }
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

// ============================================================
// Phase 2.1: High-priority dialogs (mockups 90, 89, 83, 85, 87)
// ============================================================

/// BOM Editor dialog (mockup 90).
fn render_bom_editor_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    ui.vertical(|ui| {
        ui.heading("Bill of Materials");
        ui.separator();

        // BOM table
        egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
            egui::Grid::new("bom_grid").num_columns(5).striped(true).show(ui, |ui| {
                ui.label(egui::RichText::new("Item").strong());
                ui.label(egui::RichText::new("Part Number").strong());
                ui.label(egui::RichText::new("Description").strong());
                ui.label(egui::RichText::new("Qty").strong());
                ui.label(egui::RichText::new("Material").strong());
                ui.end_row();

                let bom_items = [
                    ("1", "BR-001", "Base Bracket", "2", "Aluminum 6061"),
                    ("2", "BR-002", "Mounting Plate", "1", "Steel 1045"),
                    ("3", "HW-100", "M6 Socket Head Cap Screw", "8", "Stainless 316"),
                    ("4", "HW-101", "M6 Flat Washer", "8", "Stainless 316"),
                    ("5", "BR-003", "Side Cover", "2", "ABS Plastic"),
                ];
                for (item, pn, desc, qty, mat) in &bom_items {
                    ui.label(*item);
                    ui.label(*pn);
                    ui.label(*desc);
                    ui.label(*qty);
                    ui.label(*mat);
                    ui.end_row();
                }
            });
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Export CSV").clicked() {}
            if ui.button("Export Excel").clicked() {}
            if ui.button("Add Item").clicked() {}
            if ui.button("Remove Item").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Layer Manager dialog (mockup 89).
fn render_layer_manager_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut layers = vec![
        ("Default", true, false, egui::Color32::from_rgb(255, 255, 255)),
        ("Construction", true, false, egui::Color32::from_rgb(100, 200, 255)),
        ("Dimensions", true, false, egui::Color32::from_rgb(255, 200, 100)),
        ("Hidden Lines", false, false, egui::Color32::from_rgb(150, 150, 150)),
        ("Annotations", true, true, egui::Color32::from_rgb(200, 100, 255)),
    ];

    ui.vertical(|ui| {
        ui.heading("Layer Manager");
        ui.separator();

        egui::Grid::new("layer_grid").num_columns(5).striped(true).show(ui, |ui| {
            ui.label(egui::RichText::new("Visible").strong());
            ui.label(egui::RichText::new("Lock").strong());
            ui.label(egui::RichText::new("Color").strong());
            ui.label(egui::RichText::new("Name").strong());
            ui.label(egui::RichText::new("Objects").strong());
            ui.end_row();

            for (name, visible, locked, color) in &mut layers {
                ui.checkbox(visible, "");
                ui.checkbox(locked, "");
                ui.color_edit_button_srgba(color);
                ui.text_edit_singleline(&mut name.to_string());
                ui.label(format!("{}", 10)); // Object count placeholder
                ui.end_row();
            }
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("New Layer").clicked() {}
            if ui.button("Delete").clicked() {}
            if ui.button("Move Up").clicked() {}
            if ui.button("Move Down").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Tool Library dialog (mockup 83).
fn render_tool_library_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    ui.vertical(|ui| {
        ui.heading("Tool Library");
        ui.separator();

        egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
            egui::Grid::new("tool_grid").num_columns(6).striped(true).show(ui, |ui| {
                ui.label(egui::RichText::new("ID").strong());
                ui.label(egui::RichText::new("Name").strong());
                ui.label(egui::RichText::new("Type").strong());
                ui.label(egui::RichText::new("Ø (mm)").strong());
                ui.label(egui::RichText::new("Length").strong());
                ui.label(egui::RichText::new("Flutes").strong());
                ui.end_row();

                let tools = [
                    ("T01", "6mm End Mill", "Flat End", "6.0", "50", "3"),
                    ("T02", "3mm End Mill", "Flat End", "3.0", "30", "2"),
                    ("T03", "6mm Ball Mill", "Ball End", "6.0", "50", "2"),
                    ("T04", "5mm Drill", "Drill", "5.0", "80", "2"),
                    ("T05", "10mm Face Mill", "Face", "10.0", "25", "4"),
                    ("T06", "1mm Engraver", "Engrave", "1.0", "15", "1"),
                ];
                for (id, name, ttype, dia, len, flutes) in &tools {
                    ui.label(*id);
                    ui.label(*name);
                    ui.label(*ttype);
                    ui.label(*dia);
                    ui.label(*len);
                    ui.label(*flutes);
                    ui.end_row();
                }
            });
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Add Tool").clicked() {}
            if ui.button("Edit").clicked() {}
            if ui.button("Delete").clicked() {}
            if ui.button("Import").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// FEA Mesh Control dialog (mockup 85).
fn render_fea_mesh_control_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut element_type = 0u32; // 0=Tet4, 1=Tet10, 2=Hex8, 3=Hex20
    let mut mesh_size = 2.0f32;
    let mut refine_curved = true;
    let mut quality_threshold = 0.3f32;
    let mut growth_ratio = 1.5f32;

    ui.vertical(|ui| {
        ui.heading("FEA Mesh Control");
        ui.separator();

        ui.label("Element Type:");
        ui.radio_value(&mut element_type, 0, "Tet4 (4-node tetrahedron)");
        ui.radio_value(&mut element_type, 1, "Tet10 (10-node tetrahedron)");
        ui.radio_value(&mut element_type, 2, "Hex8 (8-node hexahedron)");
        ui.radio_value(&mut element_type, 3, "Hex20 (20-node hexahedron)");

        ui.separator();
        ui.label("Mesh Parameters:");
        ui.add(egui::Slider::new(&mut mesh_size, 0.1..=50.0).text("Element size (mm)"));
        ui.checkbox(&mut refine_curved, "Refine curved surfaces");
        ui.add(egui::Slider::new(&mut quality_threshold, 0.01..=1.0).text("Min quality (aspect ratio)"));
        ui.add(egui::Slider::new(&mut growth_ratio, 1.0..=3.0).text("Growth ratio"));

        ui.separator();
        ui.label("Statistics:");
        ui.label("Nodes: — (click Generate)");
        ui.label("Elements: — (click Generate)");
        ui.label("Quality (avg): — ");

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Generate Mesh").clicked() {}
            if ui.button("Refine").clicked() {}
            if ui.button("Coarsen").clicked() {}
            if ui.button("Quality Check").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Title Block Editor dialog (mockup 87).
fn render_title_block_editor_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut title = "Bracket Assembly".to_string();
    let mut drawing_no = "DRW-2026-001".to_string();
    let mut scale = "1:2".to_string();
    let mut material = "Aluminum 6061-T6".to_string();
    let mut designer = "AI Agent".to_string();
    let mut date = "2026-08-09".to_string();
    let mut revision = "A".to_string();
    let mut sheet_size = "A3".to_string();
    let mut sheet_no = "1 of 1".to_string();
    let mut tolerance = "±0.1mm".to_string();
    let mut finish = "Anodized".to_string();

    ui.vertical(|ui| {
        ui.heading("Title Block Editor");
        ui.separator();

        egui::Grid::new("title_block_grid").num_columns(2).striped(true).show(ui, |ui| {
            ui.label("Title:");
            ui.text_edit_singleline(&mut title);
            ui.end_row();

            ui.label("Drawing No.:");
            ui.text_edit_singleline(&mut drawing_no);
            ui.end_row();

            ui.label("Scale:");
            ui.text_edit_singleline(&mut scale);
            ui.end_row();

            ui.label("Material:");
            ui.text_edit_singleline(&mut material);
            ui.end_row();

            ui.label("Designer:");
            ui.text_edit_singleline(&mut designer);
            ui.end_row();

            ui.label("Date:");
            ui.text_edit_singleline(&mut date);
            ui.end_row();

            ui.label("Revision:");
            ui.text_edit_singleline(&mut revision);
            ui.end_row();

            ui.label("Sheet Size:");
            ui.text_edit_singleline(&mut sheet_size);
            ui.end_row();

            ui.label("Sheet No.:");
            ui.text_edit_singleline(&mut sheet_no);
            ui.end_row();

            ui.label("Tolerance:");
            ui.text_edit_singleline(&mut tolerance);
            ui.end_row();

            ui.label("Finish:");
            ui.text_edit_singleline(&mut finish);
            ui.end_row();
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Apply to Drawing").clicked() {}
            if ui.button("Save Template").clicked() {}
            if ui.button("Load Template").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

// ============================================================
// Phase 2.2: Medium-priority dialogs
// ============================================================

use std::cell::{Cell, RefCell};

thread_local! {
    static PARAM_FIND: RefCell<String> = const { RefCell::new(String::new()) };
    static PARAM_REPLACE: RefCell<String> = const { RefCell::new(String::new()) };
    static PARAM_SCOPE: Cell<u8> = const { Cell::new(0) };
    static PARAM_MATCH_CASE: Cell<bool> = const { Cell::new(false) };
    static PARAM_USE_REGEX: Cell<bool> = const { Cell::new(false) };
    static PARAM_PREVIEW: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };

    static REV_ROWS: RefCell<Vec<(String, String, String, String, bool)>>
        = RefCell::new(vec![
            ("A".to_string(), "Initial release".to_string(),
             "2026-08-09".to_string(), "CAD User".to_string(), true),
        ]);
    static REV_NEW_REV: RefCell<String> = RefCell::new("B".to_string());
    static REV_NEW_DESC: RefCell<String> = const { RefCell::new(String::new()) };

    static TUT_CATEGORY: Cell<usize> = const { Cell::new(0) };
    static TUT_DIFFICULTY: Cell<usize> = const { Cell::new(0) };
    static TUT_SELECTED: Cell<usize> = const { Cell::new(0) };

    static ONB_STEP: Cell<u8> = const { Cell::new(0) };
    static ONB_UNITS: Cell<u8> = const { Cell::new(0) };
    static ONB_WORKSPACE: Cell<u8> = const { Cell::new(0) };
    static ONB_THEME: Cell<u8> = const { Cell::new(0) };
    static ONB_NAME: RefCell<String> = const { RefCell::new(String::new()) };

    static UPD_CHECKED: Cell<bool> = const { Cell::new(false) };
    static UPD_LATEST: RefCell<String> = const { RefCell::new(String::new()) };

    static MOLD_VENDOR: Cell<u8> = const { Cell::new(0) };
    static MOLD_SELECTED: Cell<usize> = const { Cell::new(0) };
}

/// Parameter Search & Replace dialog (mockup 91).
fn render_param_search_replace_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut find = PARAM_FIND.with(|c| c.borrow().clone());
    let mut replace = PARAM_REPLACE.with(|c| c.borrow().clone());
    let mut scope = PARAM_SCOPE.get();
    let mut match_case = PARAM_MATCH_CASE.get();
    let mut use_regex = PARAM_USE_REGEX.get();
    let mut preview: Vec<String> = PARAM_PREVIEW.with(|c| c.borrow().clone());

    ui.vertical(|ui| {
        ui.heading("Parameter Search & Replace");
        ui.separator();
        egui::Grid::new("param_sr_grid").num_columns(2).spacing([10.0, 8.0]).show(ui, |ui| {
            ui.label("Find:"); ui.text_edit_singleline(&mut find); ui.end_row();
            ui.label("Replace with:"); ui.text_edit_singleline(&mut replace); ui.end_row();
            ui.label("Scope:");
            ui.horizontal(|ui| {
                ui.radio_value(&mut scope, 0, "Current Sketch");
                ui.radio_value(&mut scope, 1, "Current Part");
                ui.radio_value(&mut scope, 2, "Entire Assembly");
            });
            ui.end_row();
        });
        ui.horizontal(|ui| {
            ui.checkbox(&mut match_case, "Match case");
            ui.checkbox(&mut use_regex, "Use regex");
        });
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Find All").clicked() {
                preview.clear();
                if !find.is_empty() {
                    preview.push(format!("Sketch1.width = 100 (matches '{}')", find));
                    preview.push(format!("Sketch1.height = 50 (matches '{}')", find));
                    preview.push(format!("Extrude1.depth = {}", find));
                }
            }
            if ui.button("Replace All").clicked() {}
        });
        ui.separator();
        ui.label(format!("Preview ({} matches):", preview.len()));
        egui::ScrollArea::vertical().max_height(180.0).show(ui, |ui| {
            if preview.is_empty() {
                ui.label("(no matches — click \"Find All\")");
            } else {
                for line in &preview { ui.label(line); }
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Apply").clicked() { *close = true; }
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    PARAM_FIND.with(|c| *c.borrow_mut() = find);
    PARAM_REPLACE.with(|c| *c.borrow_mut() = replace);
    PARAM_SCOPE.set(scope);
    PARAM_MATCH_CASE.set(match_case);
    PARAM_USE_REGEX.set(use_regex);
    PARAM_PREVIEW.with(|c| *c.borrow_mut() = preview);
    None
}

/// Revision Table dialog (mockup 88).
fn render_revision_table_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut rows = REV_ROWS.with(|c| c.borrow().clone());
    let mut new_rev = REV_NEW_REV.with(|c| c.borrow().clone());
    let mut new_desc = REV_NEW_DESC.with(|c| c.borrow().clone());

    ui.vertical(|ui| {
        ui.heading("Revision Table");
        ui.separator();
        egui::ScrollArea::vertical().max_height(280.0).show(ui, |ui| {
            egui::Grid::new("rev_grid").num_columns(6).spacing([8.0, 4.0]).striped(true).show(ui, |ui| {
                ui.heading("Rev"); ui.heading("Description"); ui.heading("Date");
                ui.heading("Author"); ui.heading("Approved"); ui.heading("Actions"); ui.end_row();
                let mut remove_idx = None;
                for (i, (rev, desc, date, author, approved)) in rows.iter_mut().enumerate() {
                    ui.label(rev.as_str()); ui.label(desc.as_str()); ui.label(date.as_str());
                    ui.label(author.as_str()); ui.checkbox(approved, "");
                    if ui.button("Delete").clicked() { remove_idx = Some(i); }
                    ui.end_row();
                }
                if let Some(idx) = remove_idx { rows.remove(idx); }
            });
        });
        ui.separator();
        ui.label("Add new revision:");
        egui::Grid::new("rev_new_grid").num_columns(4).show(ui, |ui| {
            ui.label("Rev:"); ui.text_edit_singleline(&mut new_rev);
            ui.label("Description:"); ui.text_edit_singleline(&mut new_desc); ui.end_row();
        });
        ui.horizontal(|ui| {
            if ui.button("Add Revision").clicked() {
                let next_rev = new_rev.clone();
                let next_desc = new_desc.clone();
                rows.push((next_rev, next_desc, "2026-08-09".to_string(),
                          "CAD User".to_string(), false));
                new_desc.clear();
                if let Some(c) = new_rev.chars().next() {
                    let next_char = ((c as u8) + 1) as char;
                    new_rev = next_char.to_string();
                }
            }
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    REV_ROWS.with(|c| *c.borrow_mut() = rows);
    REV_NEW_REV.with(|c| *c.borrow_mut() = new_rev);
    REV_NEW_DESC.with(|c| *c.borrow_mut() = new_desc);
    None
}

/// Tutorial Browser dialog (mockup 72).
fn render_tutorial_browser_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let categories = ["Getting Started", "Sketching", "Part Modeling",
                      "Assembly", "Drawing", "Sheet Metal", "CAM", "FEA"];
    let difficulties = ["All", "Beginner", "Intermediate", "Advanced"];
    let tutorials_per_category: &[&[(&str, &str, &str)]] = &[
        &[("Welcome Tour", "Beginner", "5 min — UI overview"),
          ("First Part", "Beginner", "10 min — sketch + extrude"),
          ("Save & Export", "Beginner", "5 min — STEP/STL/PDF")],
        &[("Basic Sketching", "Beginner", "8 min — lines, circles"),
          ("Constraints", "Intermediate", "12 min — coincident, tangent"),
          ("Dimensions", "Intermediate", "10 min — linear, angular")],
        &[("Extrude & Revolve", "Beginner", "10 min"),
          ("Loft & Sweep", "Intermediate", "15 min"),
          ("Fillets & Chamfers", "Beginner", "8 min")],
        &[("Mate Constraints", "Intermediate", "12 min"),
          ("Exploded Views", "Intermediate", "8 min"),
          ("Motion Study", "Advanced", "20 min")],
        &[("Drawing Templates", "Intermediate", "15 min"),
          ("Dimensions & Annotations", "Intermediate", "12 min"),
          ("HLR Views", "Advanced", "10 min")],
        &[("Base Flange", "Intermediate", "8 min"),
          ("Bends & Relief", "Intermediate", "12 min"),
          ("Flat Pattern", "Advanced", "10 min")],
        &[("2.5D Pocket", "Intermediate", "15 min"),
          ("3D Surfacing", "Advanced", "20 min"),
          ("Post Processing", "Intermediate", "8 min")],
        &[("Linear Static", "Intermediate", "15 min"),
          ("Modal Analysis", "Advanced", "20 min"),
          ("Mesh Quality", "Advanced", "10 min")],
    ];
    let mut category = TUT_CATEGORY.get();
    let mut difficulty = TUT_DIFFICULTY.get();
    let mut selected = TUT_SELECTED.get();
    let cat_tutorials = tutorials_per_category.get(category).copied().unwrap_or(&[]);

    ui.vertical(|ui| {
        ui.heading("Tutorial Browser");
        ui.separator();
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label("Category:");
                for (i, c) in categories.iter().enumerate() { ui.radio_value(&mut category, i, *c); }
            });
            ui.separator();
            ui.vertical(|ui| {
                ui.label("Difficulty:");
                for (i, d) in difficulties.iter().enumerate() { ui.radio_value(&mut difficulty, i, *d); }
            });
        });
        ui.separator();
        ui.label(format!("Tutorials ({} shown):", cat_tutorials.len()));
        egui::ScrollArea::vertical().max_height(250.0).show(ui, |ui| {
            for (i, (title, diff, duration)) in cat_tutorials.iter().enumerate() {
                let diff_match = difficulty == 0 || difficulties[difficulty] == *diff;
                if !diff_match { continue; }
                let is_selected = selected == i;
                if ui.selectable_label(is_selected,
                    format!("{} [{}] — {}", title, diff, duration)).clicked() { selected = i; }
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Start Tutorial").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    TUT_CATEGORY.set(category);
    TUT_DIFFICULTY.set(difficulty);
    TUT_SELECTED.set(selected);
    None
}

/// Crash Recovery dialog (mockup 76).
fn render_crash_recovery_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let auto_saves = [
        ("untitled_001.brep", "2026-08-09 14:32:01", "12 KB"),
        ("bracket_v2.brep", "2026-08-09 13:18:45", "47 KB"),
        ("assembly_test.brep", "2026-08-09 11:05:22", "120 KB"),
    ];
    ui.vertical(|ui| {
        ui.heading("Crash Recovery");
        ui.separator();
        ui.label("BRepCAD detected auto-saved files from a previous session.");
        ui.label("Select a file to recover, or discard to delete it permanently.");
        ui.separator();
        egui::Grid::new("crash_grid").num_columns(4).striped(true).show(ui, |ui| {
            ui.heading("File"); ui.heading("Saved"); ui.heading("Size"); ui.heading("Actions"); ui.end_row();
            for (name, time, size) in &auto_saves {
                ui.label(*name); ui.label(*time); ui.label(*size);
                ui.horizontal(|ui| {
                    if ui.button("Recover").clicked() {}
                    if ui.button("Discard").clicked() {}
                });
                ui.end_row();
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Recover All").clicked() {}
            if ui.button("Discard All").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

/// Onboarding Wizard dialog (mockup 77).
fn render_onboarding_wizard_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut step = ONB_STEP.get();
    let mut units = ONB_UNITS.get();
    let mut workspace = ONB_WORKSPACE.get();
    let mut theme = ONB_THEME.get();
    let mut name = ONB_NAME.with(|c| c.borrow().clone());
    if name.is_empty() { name = "User".to_string(); }

    let steps = ["Welcome", "Profile", "Units & Workspace", "Theme", "Done"];
    ui.vertical(|ui| {
        ui.heading("Welcome to BRepCAD");
        ui.label(format!("Step {} of {}: {}", step + 1, steps.len(), steps[step as usize]));
        ui.separator();
        let progress = (step as f32 + 1.0) / steps.len() as f32;
        ui.add(egui::ProgressBar::new(progress));
        ui.separator();
        match step {
            0 => {
                ui.label("Welcome to BRepCAD — a Rust-native 3D CAD/CAE/CAM system.");
                ui.label("This wizard will guide you through initial setup.");
                ui.label("Click Next to continue, or Skip to use defaults.");
            }
            1 => {
                ui.label("Profile");
                egui::Grid::new("onb_profile").show(ui, |ui| {
                    ui.label("Your name:"); ui.text_edit_singleline(&mut name); ui.end_row();
                });
            }
            2 => {
                ui.label("Units & Default Workspace");
                egui::Grid::new("onb_units").show(ui, |ui| {
                    ui.label("Default units:");
                    ui.horizontal(|ui| {
                        ui.radio_value(&mut units, 0, "mm");
                        ui.radio_value(&mut units, 1, "cm");
                        ui.radio_value(&mut units, 2, "m");
                        ui.radio_value(&mut units, 3, "inch");
                    });
                    ui.end_row();
                    ui.label("Default workspace:");
                    ui.horizontal(|ui| {
                        ui.radio_value(&mut workspace, 0, "Modeling");
                        ui.radio_value(&mut workspace, 1, "Sketch");
                        ui.radio_value(&mut workspace, 2, "Drawing");
                    });
                    ui.end_row();
                });
            }
            3 => {
                ui.label("Theme");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut theme, 0, "Dark");
                    ui.radio_value(&mut theme, 1, "Light");
                    ui.radio_value(&mut theme, 2, "System");
                });
            }
            _ => {
                ui.label("Setup complete!");
                ui.label(format!("Welcome, {}!", name));
                ui.label("Click Finish to start using BRepCAD.");
            }
        }
        ui.separator();
        ui.horizontal(|ui| {
            if step > 0 && ui.button("Back").clicked() { step -= 1; }
            if step < (steps.len() - 1) as u8 {
                if ui.button("Next").clicked() { step += 1; }
            } else {
                if ui.button("Finish").clicked() { *close = true; }
            }
            if ui.button("Skip").clicked() { *close = true; }
        });
    });
    ONB_STEP.set(step);
    ONB_UNITS.set(units);
    ONB_WORKSPACE.set(workspace);
    ONB_THEME.set(theme);
    ONB_NAME.with(|c| *c.borrow_mut() = name);
    None
}

// ============================================================
// Phase 2.3: Low-priority dialogs
// ============================================================

/// Update Check dialog (mockup 58).
fn render_update_check_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let mut checked = UPD_CHECKED.get();
    let latest = UPD_LATEST.with(|c| c.borrow().clone());
    let latest = if latest.is_empty() { "0.1.0".to_string() } else { latest };

    ui.vertical(|ui| {
        ui.heading("Check for Updates");
        ui.separator();
        egui::Grid::new("update_grid").show(ui, |ui| {
            ui.label("Current version:"); ui.label(env!("CARGO_PKG_VERSION")); ui.end_row();
            ui.label("Latest version:"); ui.label(latest.as_str()); ui.end_row();
        });
        ui.separator();
        if !checked {
            ui.label("Click \"Check Now\" to look for updates.");
        } else if latest == env!("CARGO_PKG_VERSION") {
            ui.colored_label(egui::Color32::from_rgb(166, 227, 161),
                "✓ You are running the latest version.");
        } else {
            ui.colored_label(egui::Color32::from_rgb(249, 226, 175),
                "⚠ A new version is available!");
            ui.label("Release notes:");
            egui::ScrollArea::vertical().max_height(150.0).show(ui, |ui| {
                ui.label("• Improved HLR performance (100× faster)");
                ui.label("• Added NURBS sweep along curve");
                ui.label("• New menu icons (Phase 1.2)");
                ui.label("• View toggles for shadows/AO (Phase 3.6)");
            });
        }
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Check Now").clicked() { checked = true; }
            if ui.button("Download").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    UPD_CHECKED.set(checked);
    UPD_LATEST.with(|c| *c.borrow_mut() = latest);
    None
}

/// License Information dialog (mockup 75).
fn render_license_info_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    ui.vertical(|ui| {
        ui.heading("License Information");
        ui.separator();
        ui.label("BRepCAD is licensed under the GNU GPL v3 or later.");
        ui.label("Copyright © 2026 KernelDev");
        ui.separator();
        ui.heading("Third-Party Components");
        egui::ScrollArea::vertical().max_height(280.0).show(ui, |ui| {
            egui::Grid::new("lic_grid").num_columns(2).striped(true).show(ui, |ui| {
                let components = [
                    ("egui", "MIT"), ("wgpu", "MIT/Apache-2.0"),
                    ("cgmath", "MIT"), ("rayon", "MIT/Apache-2.0"),
                    ("serde", "MIT/Apache-2.0"), ("Noto Sans SC", "SIL Open Font License"),
                ];
                for (name, lic) in components { ui.label(name); ui.label(lic); ui.end_row(); }
            });
        });
        ui.separator();
        if ui.button("View Full License").clicked() {}
        if ui.button("Close").clicked() { *close = true; }
    });
    None
}

/// Mold Catalog dialog (mockup 61).
fn render_mold_catalog_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let vendors = ["HASCO", "DME", "MISUMI", "LKM"];
    let catalog: &[(&str, &str, &str)] = &[
        ("K50", "200×250 mm", "Standard 2-plate"),
        ("K100", "300×400 mm", "Standard 2-plate"),
        ("K200", "400×500 mm", "Standard 2-plate"),
        ("K50-3P", "200×250 mm", "3-plate mold"),
        ("K100-3P", "300×400 mm", "3-plate mold"),
    ];
    let mut vendor = MOLD_VENDOR.get();
    let mut selected = MOLD_SELECTED.get();

    ui.vertical(|ui| {
        ui.heading("Mold Base Catalog");
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Vendor:");
            for (i, v) in vendors.iter().enumerate() { ui.radio_value(&mut vendor, i as u8, *v); }
        });
        ui.separator();
        ui.label(format!("Mold bases from {}:", vendors[vendor as usize]));
        egui::ScrollArea::vertical().max_height(220.0).show(ui, |ui| {
            egui::Grid::new("mold_grid").num_columns(4).striped(true).show(ui, |ui| {
                ui.heading("Code"); ui.heading("Size"); ui.heading("Type"); ui.heading("Select"); ui.end_row();
                for (i, (code, size, mtype)) in catalog.iter().enumerate() {
                    ui.label(*code); ui.label(*size); ui.label(*mtype);
                    ui.radio_value(&mut selected, i, ""); ui.end_row();
                }
            });
        });
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Insert into Assembly").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    MOLD_VENDOR.set(vendor);
    MOLD_SELECTED.set(selected);
    None
}

/// Modal Plotter dialog (mockup 86).
fn render_modal_plotter_dialog(ui: &mut egui::Ui, close: &mut bool) -> Option<DialogAction> {
    let modes: &[(u32, f64, &str)] = &[
        (1, 42.5, "Bending X"), (2, 87.3, "Bending Y"),
        (3, 145.8, "Torsion"), (4, 210.4, "Bending Z"),
        (5, 298.1, "Membrane"), (6, 387.5, "Higher mode"),
    ];
    ui.vertical(|ui| {
        ui.heading("Modal Plotter");
        ui.separator();
        ui.label("Natural frequencies (Hz) vs mode number:");
        ui.separator();
        let max_freq = modes.iter().map(|(_, f, _)| *f).fold(0.0_f64, f64::max);
        let plot_height = 18.0_f32;
        let plot_width = 380.0_f32;
        let (rect, _) = ui.allocate_exact_size(
            egui::vec2(plot_width, modes.len() as f32 * (plot_height + 4.0)),
            egui::Sense::hover(),
        );
        let painter = ui.painter();
        let start_x = rect.min.x;
        let start_y = rect.min.y;
        for (i, (n, freq, label)) in modes.iter().enumerate() {
            let y = start_y + i as f32 * (plot_height + 4.0) + plot_height * 0.5;
            painter.text(egui::pos2(start_x, y), egui::Align2::LEFT_CENTER,
                format!("Mode {}: {} ({:.1} Hz)", n, label, freq),
                egui::FontId::proportional(10.0),
                egui::Color32::from_rgb(205, 214, 244));
            let bar_width = (freq / max_freq * (plot_width as f64 - 180.0)) as f32;
            let bar_rect = egui::Rect::from_min_size(
                egui::pos2(start_x + 180.0, y - plot_height * 0.4),
                egui::vec2(bar_width, plot_height * 0.8),
            );
            painter.rect_filled(bar_rect, 2.0, egui::Color32::from_rgb(137, 180, 250));
        }
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Export CSV").clicked() {}
            if ui.button("Animate Mode").clicked() {}
            if ui.button("Close").clicked() { *close = true; }
        });
    });
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_classify_gcode_comment() {
        let color = classify_gcode_line("; comment");
        assert_ne!(color, egui::Color32::from_rgb(205, 214, 244));
    }

    #[test]
    fn test_classify_gcode_g_command() {
        assert_eq!(classify_gcode_line("G90 G21 G17"), egui::Color32::from_rgb(137, 180, 250));
    }

    #[test]
    fn test_classify_gcode_m_command() {
        assert_eq!(classify_gcode_line("M3 S1000"), egui::Color32::from_rgb(137, 180, 250));
    }

    #[test]
    fn test_classify_gcode_tool_change() {
        assert_eq!(classify_gcode_line("T1 M6"), egui::Color32::from_rgb(245, 194, 231));
    }

    #[test]
    fn test_classify_gcode_coordinates() {
        for coord in &["X10.0", "Y20.0", "Z-5.0", "I0.5", "J0.5", "R2.0", "K0.0"] {
            assert_eq!(classify_gcode_line(coord), egui::Color32::from_rgb(166, 227, 161));
        }
    }

    #[test]
    fn test_classify_gcode_feed_speed() {
        assert_eq!(classify_gcode_line("F200"), egui::Color32::from_rgb(249, 226, 175));
        assert_eq!(classify_gcode_line("S1000"), egui::Color32::from_rgb(249, 226, 175));
    }

    #[test]
    fn test_classify_gcode_line_number() {
        assert_eq!(classify_gcode_line("N10 G0 X0 Y0"), egui::Color32::from_rgb(148, 226, 213));
    }

    #[test]
    fn test_classify_gcode_default() {
        assert_eq!(classify_gcode_line("UNKNOWN_LINE"), egui::Color32::from_rgb(205, 214, 244));
    }

    #[test]
    fn test_classify_gcode_leading_whitespace() {
        assert_eq!(classify_gcode_line("    G90"), egui::Color32::from_rgb(137, 180, 250));
    }

    #[test]
    fn test_dialog_titles_phase2_2() {
        assert_eq!(dialog_title(&DialogType::ParamSearchReplace), "Parameter Search & Replace");
        assert_eq!(dialog_title(&DialogType::RevisionTable), "Revision Table");
        assert_eq!(dialog_title(&DialogType::TutorialBrowser), "Tutorial Browser");
        assert_eq!(dialog_title(&DialogType::CrashRecovery), "Crash Recovery");
        assert_eq!(dialog_title(&DialogType::OnboardingWizard), "Welcome to BRepCAD");
    }

    #[test]
    fn test_dialog_titles_phase2_3() {
        assert_eq!(dialog_title(&DialogType::UpdateCheck), "Check for Updates");
        assert_eq!(dialog_title(&DialogType::LicenseInfo), "License Information");
        assert_eq!(dialog_title(&DialogType::MoldCatalog), "Mold Base Catalog");
        assert_eq!(dialog_title(&DialogType::ModalPlotter), "Modal Plotter");
    }

    #[test]
    fn test_nc_code_dialog_with_data() {
        let dt = DialogType::NcCodeViewer {
            gcode: "G90 G21\nM3 S1000\nM30".to_string(),
            dialect: "Fanuc".to_string(),
        };
        assert_eq!(dialog_title(&dt), "NC Code Viewer");
    }

    #[test]
    fn test_nc_code_dialog_empty() {
        let dt = DialogType::NcCodeViewer {
            gcode: String::new(), dialect: String::new(),
        };
        assert_eq!(dialog_title(&dt), "NC Code Viewer");
    }

    #[test]
    fn test_dialog_type_default_is_none() {
        assert_eq!(DialogType::default(), DialogType::None);
        assert_eq!(dialog_title(&DialogType::default()), "");
    }

    #[test]
    fn test_dialog_type_equality() {
        assert_eq!(DialogType::RevisionTable, DialogType::RevisionTable);
        assert_ne!(DialogType::RevisionTable, DialogType::TutorialBrowser);
        assert_ne!(DialogType::MoldCatalog, DialogType::ModalPlotter);
    }

    #[test]
    fn test_dialog_type_clone_debug() {
        let dt = DialogType::OnboardingWizard;
        assert_eq!(dt, dt.clone());
        assert!(format!("{:?}", dt).contains("OnboardingWizard"));
    }
}
