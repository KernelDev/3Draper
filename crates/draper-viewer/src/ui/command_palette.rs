// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Command palette — fuzzy search palette triggered by Ctrl+Shift+P.
//!
//! VS Code-style fuzzy command search.

use eframe::egui;

/// Command palette state.
#[derive(Clone, Debug)]
pub struct CommandPalette {
    pub visible: bool,
    pub query: String,
    pub selected: usize,
}

impl Default for CommandPalette {
    fn default() -> Self {
        Self {
            visible: false,
            query: String::new(),
            selected: 0,
        }
    }
}

/// A command entry in the palette.
#[derive(Clone, Debug)]
pub struct Command {
    pub name: String,
    pub category: String,
    pub shortcut: String,
}

/// Get all available commands.
pub fn all_commands() -> Vec<Command> {
    vec![
        // File
        Command { name: "New".into(), category: "File".into(), shortcut: "Ctrl+N".into() },
        Command { name: "Open…".into(), category: "File".into(), shortcut: "Ctrl+O".into() },
        Command { name: "Save".into(), category: "File".into(), shortcut: "Ctrl+S".into() },
        Command { name: "Export STEP".into(), category: "File".into(), shortcut: "".into() },
        Command { name: "Export STL".into(), category: "File".into(), shortcut: "".into() },
        Command { name: "Import STEP".into(), category: "File".into(), shortcut: "".into() },
        // Edit
        Command { name: "Undo".into(), category: "Edit".into(), shortcut: "Ctrl+Z".into() },
        Command { name: "Redo".into(), category: "Edit".into(), shortcut: "Ctrl+Shift+Z".into() },
        Command { name: "Cut".into(), category: "Edit".into(), shortcut: "Ctrl+X".into() },
        Command { name: "Copy".into(), category: "Edit".into(), shortcut: "Ctrl+C".into() },
        Command { name: "Paste".into(), category: "Edit".into(), shortcut: "Ctrl+V".into() },
        Command { name: "Duplicate".into(), category: "Edit".into(), shortcut: "Ctrl+D".into() },
        // View
        Command { name: "Fit to View".into(), category: "View".into(), shortcut: "F".into() },
        Command { name: "ISO View".into(), category: "View".into(), shortcut: "0".into() },
        Command { name: "Front View".into(), category: "View".into(), shortcut: "1".into() },
        Command { name: "Top View".into(), category: "View".into(), shortcut: "2".into() },
        Command { name: "Right View".into(), category: "View".into(), shortcut: "3".into() },
        Command { name: "Wireframe".into(), category: "View".into(), shortcut: "".into() },
        Command { name: "Shaded".into(), category: "View".into(), shortcut: "".into() },
        Command { name: "Shaded + Edges".into(), category: "View".into(), shortcut: "".into() },
        // Insert
        Command { name: "Insert Box".into(), category: "Insert".into(), shortcut: "".into() },
        Command { name: "Insert Sphere".into(), category: "Insert".into(), shortcut: "".into() },
        Command { name: "Insert Cylinder".into(), category: "Insert".into(), shortcut: "".into() },
        Command { name: "Insert Cone".into(), category: "Insert".into(), shortcut: "".into() },
        Command { name: "Insert Torus".into(), category: "Insert".into(), shortcut: "".into() },
        Command { name: "Insert Sketch".into(), category: "Insert".into(), shortcut: "".into() },
        // Modify
        Command { name: "Boolean Union".into(), category: "Modify".into(), shortcut: "".into() },
        Command { name: "Boolean Subtract".into(), category: "Modify".into(), shortcut: "".into() },
        Command { name: "Boolean Intersect".into(), category: "Modify".into(), shortcut: "".into() },
        Command { name: "Fillet".into(), category: "Modify".into(), shortcut: "".into() },
        Command { name: "Chamfer".into(), category: "Modify".into(), shortcut: "".into() },
        Command { name: "Move".into(), category: "Modify".into(), shortcut: "M".into() },
        Command { name: "Rotate".into(), category: "Modify".into(), shortcut: "R".into() },
        Command { name: "Scale".into(), category: "Modify".into(), shortcut: "".into() },
        // Sketch
        Command { name: "Sketch Mode".into(), category: "Sketch".into(), shortcut: "S".into() },
        Command { name: "Line".into(), category: "Sketch".into(), shortcut: "L".into() },
        Command { name: "Circle".into(), category: "Sketch".into(), shortcut: "C".into() },
        Command { name: "Arc".into(), category: "Sketch".into(), shortcut: "A".into() },
        Command { name: "Rectangle".into(), category: "Sketch".into(), shortcut: "R".into() },
        Command { name: "Dimension".into(), category: "Sketch".into(), shortcut: "D".into() },
        // Tools
        Command { name: "Options…".into(), category: "Tools".into(), shortcut: "".into() },
        Command { name: "Customize…".into(), category: "Tools".into(), shortcut: "".into() },
        Command { name: "Plugins Manager…".into(), category: "Tools".into(), shortcut: "".into() },
        Command { name: "Scripting Console".into(), category: "Tools".into(), shortcut: "".into() },
        Command { name: "Performance Monitor".into(), category: "Tools".into(), shortcut: "".into() },
        // Measure
        Command { name: "Measure Distance".into(), category: "Inspect".into(), shortcut: "".into() },
        Command { name: "Measure Angle".into(), category: "Inspect".into(), shortcut: "".into() },
        Command { name: "Measure Area".into(), category: "Inspect".into(), shortcut: "".into() },
        Command { name: "Measure Volume".into(), category: "Inspect".into(), shortcut: "".into() },
        // Heal
        Command { name: "Heal: Stitch".into(), category: "Heal".into(), shortcut: "".into() },
        Command { name: "Heal: Gap Fill".into(), category: "Heal".into(), shortcut: "".into() },
        Command { name: "Heal: Fix Orientation".into(), category: "Heal".into(), shortcut: "".into() },
        Command { name: "Watertight Check".into(), category: "Inspect".into(), shortcut: "".into() },
        Command { name: "Manifold Check".into(), category: "Inspect".into(), shortcut: "".into() },
    ]
}

/// Render the command palette as a modal overlay.
/// Returns the selected command name if one was chosen.
pub fn render_command_palette(ctx: &egui::Context, palette: &mut CommandPalette) -> Option<String> {
    let mut selected_command = None;

    // Check for Ctrl+Shift+P or Cmd+Shift+P
    let toggle = ctx.input(|i| {
        (i.modifiers.ctrl || i.modifiers.command) && i.modifiers.shift && i.key_pressed(egui::Key::P)
    });
    if toggle {
        palette.visible = !palette.visible;
        if palette.visible {
            palette.query.clear();
            palette.selected = 0;
        }
    }

    if !palette.visible {
        return None;
    }

    // Modal area — centered at top
    let screen_rect = ctx.screen_rect();
    let popup_width = 600.0;
    let popup_x = screen_rect.center().x - popup_width / 2.0;
    let popup_y = screen_rect.top() + 40.0;

    egui::Area::new(egui::Id::new("command_palette"))
        .fixed_pos(egui::pos2(popup_x, popup_y))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style())
                .fill(egui::Color32::from_rgb(30, 35, 40))
                .stroke(egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(60, 70, 80)))
                .show(ui, |ui| {
                    ui.set_width(popup_width);

                    // Search input
                    let resp = ui.text_edit_singleline(&mut palette.query);
                    resp.request_focus();

                    ui.separator();

                    // Filter commands
                    let commands = all_commands();
                    let query_lower = palette.query.to_lowercase();
                    let filtered: Vec<&Command> = commands.iter()
                        .filter(|c| {
                            if query_lower.is_empty() { return true; }
                            c.name.to_lowercase().contains(&query_lower) ||
                            c.category.to_lowercase().contains(&query_lower)
                        })
                        .collect();

                    // Command list
                    egui::ScrollArea::vertical()
                        .max_height(300.0)
                        .show(ui, |ui| {
                            for (i, cmd) in filtered.iter().enumerate() {
                                let is_selected = i == palette.selected;
                                let bg = if is_selected {
                                    egui::Color32::from_rgb(10, 132, 255)
                                } else {
                                    egui::Color32::TRANSPARENT
                                };

                                let frame = egui::Frame::new()
                                    .fill(bg)
                                    .inner_margin(egui::Margin::symmetric(8, 4));

                                let row = ui.horizontal(|ui| {
                                    frame.show(ui, |ui| {
                                        ui.set_width(popup_width - 20.0);
                                        ui.horizontal(|ui| {
                                            ui.label(egui::RichText::new(&cmd.name).color(egui::Color32::WHITE));
                                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                if !cmd.shortcut.is_empty() {
                                                    ui.label(egui::RichText::new(&cmd.shortcut).small().color(egui::Color32::GRAY));
                                                }
                                                ui.label(egui::RichText::new(&cmd.category).small().color(egui::Color32::from_rgb(100, 120, 140)));
                                            });
                                        });
                                    });
                                });

                                let resp = &row.response;
                                if resp.clicked() {
                                    selected_command = Some(cmd.name.clone());
                                    palette.visible = false;
                                }
                                if resp.hovered() {
                                    palette.selected = i;
                                }
                            }
                        });

                    // Keyboard navigation
                    if ui.input(|i| i.key_pressed(egui::Key::ArrowDown)) {
                        if palette.selected + 1 < filtered.len() {
                            palette.selected += 1;
                        }
                    }
                    if ui.input(|i| i.key_pressed(egui::Key::ArrowUp)) {
                        if palette.selected > 0 {
                            palette.selected -= 1;
                        }
                    }
                    if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        if let Some(cmd) = filtered.get(palette.selected) {
                            selected_command = Some(cmd.name.clone());
                            palette.visible = false;
                        }
                    }
                    if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                        palette.visible = false;
                    }
                });
        });

    selected_command
}
