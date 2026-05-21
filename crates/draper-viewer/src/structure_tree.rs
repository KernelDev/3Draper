//! Structure tree display for STEP files.

use draper_step::ast::StructureNode;
use egui::*;

/// Show the structure tree of a STEP document.
pub fn show_structure_tree(ui: &mut Ui, tree: &StructureNode) {
    show_node(ui, tree, 0);
}

fn show_node(ui: &mut Ui, node: &StructureNode, depth: usize) {
    let icon = match node.type_name.as_str() {
        "PRODUCT" => "📦",
        "MANIFOLD_SOLID_BREP" => "🔲",
        "ADVANCED_FACE" => "🔷",
        "EDGE_LOOP" => "📐",
        "EDGE_CURVE" => "📏",
        "VERTEX_POINT" => "📍",
        "CARTESIAN_POINT" => "·",
        "CLOSED_SHELL" => "⬡",
        _ => "●",
    };

    let default_open = depth < 2;

    if node.children.is_empty() {
        ui.horizontal(|ui| {
            ui.add_space(depth as f32 * 16.0);
            ui.label(RichText::new(format!("{} ", icon)).size(12.0));
            ui.label(RichText::new(&node.name).size(12.0));
        });
    } else {
        ui.horizontal(|ui| {
            ui.add_space(depth as f32 * 16.0);
        });

        let id = egui::Id::new(format!("step_node_{}_{}", node.name, depth));

        egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, default_open)
            .show_header(ui, |ui| {
                ui.label(RichText::new(format!("{} ", icon)).size(12.0));
                ui.label(RichText::new(&node.name).size(12.0));
            })
            .body(|ui| {
                for child in &node.children {
                    show_node(ui, child, depth + 1);
                }
            });
    }
}
