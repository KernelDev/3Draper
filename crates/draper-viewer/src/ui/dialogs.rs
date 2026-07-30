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
    let mut values: Vec<f64> = params.iter().map(|p| p.1).collect();
    let mut result = None;

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
