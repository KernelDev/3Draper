//! Engine demo — builds and displays an inline-4 ICE model.
//!
//! Usage: cargo run --example engine_demo

use draper_core::engine::EngineModel;
use draper_core::Document;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    println!("=== 3Draper Engine Model Demo ===\n");
    println!("Building inline-4 internal combustion engine model...\n");

    let engine = EngineModel::build();

    let stats = Document::from_engine(engine).statistics();

    println!("Engine model statistics:");
    println!("  Topological vertices: {}", stats.total_vertices);
    println!("  Topological edges:    {}", stats.total_edges);
    println!("  Topological faces:    {}", stats.total_faces);
    println!("  Solids:               {}", stats.total_solids);
    println!("  Mesh triangles:       {}", stats.total_triangles);
    println!("  Mesh vertices:        {}", stats.total_mesh_vertices);

    println!("\nEngine model built successfully!");

    // List all parts
    let engine = EngineModel::build();
    println!("\nParts ({} total):", engine.part_names.len());
    let mut parts: Vec<_> = engine.part_names.iter().collect();
    parts.sort_by_key(|(_, name)| name.clone());
    for (id, name) in &parts {
        let color = engine.part_colors.get(id).unwrap_or(&[0.5, 0.5, 0.5]);
        println!("  [{}] {} (color: [{:.2}, {:.2}, {:.2}])", id, name, color[0], color[1], color[2]);
    }

    println!("\n=== Demo complete ===");
    println!("\nTo view the 3D model, run: cargo run -p draper-viewer");
    println!("Then open a STEP file, or modify the viewer to load the engine model.");
}
