// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Dockable panels — Browser (Model Tree), Properties, and other panels.
//!
//! Roadmap UI Phase 5: Dockable panels.

use eframe::egui;

/// Panel tabs for the left dock.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LeftPanelTab {
    #[default]
    Tree,
    Layers,
    Selection,
}

/// Panel tabs for the right dock.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RightPanelTab {
    #[default]
    Properties,
    Constraints,
    Dimensions,
    Material,
}

/// Tree node for the model browser.
#[derive(Clone, Debug)]
pub struct TreeNode {
    pub name: String,
    pub node_type: String,
    pub visible: bool,
    pub selected: bool,
    pub children: Vec<TreeNode>,
}

/// Render the left dock panel (Browser/Model Tree).
pub fn render_left_panel(ctx: &egui::Context, active_tab: &mut LeftPanelTab, tree: &[TreeNode]) {
    egui::SidePanel::left("left_panel")
        .default_width(280.0)
        .min_width(200.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(active_tab, LeftPanelTab::Tree, "Tree");
                ui.selectable_value(active_tab, LeftPanelTab::Layers, "Layers");
                ui.selectable_value(active_tab, LeftPanelTab::Selection, "Selection");
            });
            ui.separator();

            // Filter box
            ui.horizontal(|ui| {
                ui.text_edit_singleline(&mut String::new());
                let _ = ui.button("Filter");
            });
            ui.separator();

            match active_tab {
                LeftPanelTab::Tree => render_tree(ui, tree),
                LeftPanelTab::Layers => render_layers(ui),
                LeftPanelTab::Selection => render_selection(ui),
            }
        });
}

fn render_tree(ui: &mut egui::Ui, nodes: &[TreeNode]) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        for node in nodes {
            render_tree_node(ui, node);
        }
    });
}

fn render_tree_node(ui: &mut egui::Ui, node: &TreeNode) {
    let icon = match node.node_type.as_str() {
        "assembly" => "📁",
        "body" | "solid" => "📦",
        "face" => "🔷",
        "edge" => "📏",
        "vertex" => "•",
        "sketch" => "✏️",
        "plane" => "▱",
        "mesh" => "🕸",
        "material" => "🎨",
        "study" => "📊",
        "cam" => "⚙",
        _ => "📄",
    };
    let label = format!("{} {} {}", icon,
        if node.visible { "👁" } else { "🚫" },
        node.name);
    if node.children.is_empty() {
        ui.selectable_label(node.selected, &label);
    } else {
        let resp = ui.selectable_label(node.selected, &label);
        if resp.clicked() {
            // Toggle expand/collapse could be added here
        }
        ui.indent(format!("tree_{}", node.name), |ui| {
            for child in &node.children {
                render_tree_node(ui, child);
            }
        });
    }
}

fn render_layers(ui: &mut egui::Ui) {
    ui.label("Layers:");
    let layers = ["0: Default", "1: Construction", "2: Dimensions", "3: Annotations"];
    for layer in &layers {
        ui.horizontal(|ui| {
            ui.checkbox(&mut true, "");
            ui.label(*layer);
        });
    }
}

fn render_selection(ui: &mut egui::Ui) {
    ui.label("No selection");
    ui.label("Click on entities in the viewport");
    ui.label("or tree to select them.");
}

/// Render the right dock panel (Properties).
pub fn render_right_panel(ctx: &egui::Context, active_tab: &mut RightPanelTab) {
    egui::SidePanel::right("right_panel")
        .default_width(280.0)
        .min_width(200.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(active_tab, RightPanelTab::Properties, "Props");
                ui.selectable_value(active_tab, RightPanelTab::Constraints, "Constraints");
                ui.selectable_value(active_tab, RightPanelTab::Dimensions, "Dimensions");
                ui.selectable_value(active_tab, RightPanelTab::Material, "Material");
            });
            ui.separator();

            match active_tab {
                RightPanelTab::Properties => render_properties(ui),
                RightPanelTab::Constraints => render_constraints(ui),
                RightPanelTab::Dimensions => render_dimensions(ui),
                RightPanelTab::Material => render_material(ui),
            }
        });
}

fn render_properties(ui: &mut egui::Ui) {
    ui.heading("Properties");
    ui.separator();
    ui.label("No entity selected");
    ui.label("");
    ui.label("Select a face, edge, or vertex");
    ui.label("to see its properties here.");

    // Example properties (shown when something is selected)
    ui.collapsing("General", |ui| {
        ui.label("Name: Face_1");
        ui.label("ID: 42");
        ui.label("Type: Plane");
    });
    ui.collapsing("Geometry", |ui| {
        ui.label("Area: 1250.00 mm²");
        ui.label("Normal: (0, 0, 1)");
        ui.label("Perimeter: 150.00 mm");
    });
    ui.collapsing("Appearance", |ui| {
        ui.color_edit_button_srgb(&mut [100, 150, 200]);
        ui.label("Color");
    });
}

fn render_constraints(ui: &mut egui::Ui) {
    ui.heading("Constraints");
    ui.separator();
    ui.label("No constraints");
    ui.label("Constraints appear here when");
    ui.label("editing a sketch.");
}

fn render_dimensions(ui: &mut egui::Ui) {
    ui.heading("Dimensions");
    ui.separator();
    ui.label("No dimensions");
    ui.label("Dimensions appear here when");
    ui.label("editing a sketch or drawing.");
}

fn render_material(ui: &mut egui::Ui) {
    ui.heading("Material");
    ui.separator();
    ui.label("No material assigned");
    let _ = ui.button("Assign Material…");
    ui.separator();
    ui.collapsing("Library", |ui| {
        let categories = ["Metals", "Plastics", "Ceramics", "Composites", "Wood", "Glass", "Custom"];
        for cat in &categories {
            ui.label(*cat);
        }
    });
}
