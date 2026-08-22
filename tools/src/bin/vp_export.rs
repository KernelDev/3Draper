//! VP → STL + STEP export.
//!
//! Строит сложную деталь ("engine bracket") через VP graph:
//!
//! ```text
//!   ┌─────────┐     ┌──────────┐
//!   │ Box     │────▶│          │
//!   │ 80×40×20│     │ Boolean  │──▶ Fillet ──▶ BakeToDoc ──▶ Solid
//!   └─────────┘     │ Subtract │
//!   ┌─────────┐     │          │
//!   │ Cylinder│────▶│          │
//!   │ r=8 h=40│     └──────────┘
//!   └─────────┘          │
//!   ┌─────────┐          ▼
//!   │ Box     │────▶ Boolean Union
//!   │ 20×40×30│
//!   └─────────┘
//! ```
//!
//! Результат: кронштейн с отверстием, ребром жесткости и филе на рёбрах.

use draper_viewer::app::vp_evaluate_graph;
use draper_viewer::ui::workspaces::{NodeType, VpGraph};
use draper_mesh::{stl, triangulate_solid, TriangulationParams};
use draper_step::exporter::{export_step, write_step_file};

fn main() {
    env_logger::init();
    println!("=== VP → STL + STEP Export ===\n");

    let mut graph = VpGraph::new();

    // ── Node 1: Base plate (Box 80×40×20) ──
    let base_box = graph.add_node(
        NodeType::Box { width: 80.0, height: 40.0, depth: 20.0 },
        0.0, 0.0,
    );

    // ── Node 2: Cylinder cutter (r=8, h=40) — for the bolt hole ──
    let cyl_cutter = graph.add_node(
        NodeType::Cylinder { radius: 8.0, height: 40.0 },
        0.0, 100.0,
    );

    // ── Node 3: Move cylinder to center of plate (40, 20, -10) ──
    let move_cyl = graph.add_node(
        NodeType::Move { x: 40.0, y: 20.0, z: -10.0 },
        200.0, 100.0,
    );
    graph.connect(cyl_cutter, 0, move_cyl, 0);

    // ── Node 4: Boolean Subtract (base - cylinder) → plate with hole ──
    let plate_with_hole = graph.add_node(NodeType::BooleanSubtract, 400.0, 0.0);
    graph.connect(base_box, 0, plate_with_hole, 0);
    graph.connect(move_cyl, 0, plate_with_hole, 1);

    // ── Node 5: Rib (Box 20×40×30) — vertical reinforcement ──
    let rib_box = graph.add_node(
        NodeType::Box { width: 20.0, height: 40.0, depth: 30.0 },
        0.0, 200.0,
    );

    // ── Node 6: Move rib to center (30, 0, 20) ──
    let move_rib = graph.add_node(
        NodeType::Move { x: 30.0, y: 0.0, z: 20.0 },
        200.0, 200.0,
    );
    graph.connect(rib_box, 0, move_rib, 0);

    // ── Node 7: Boolean Union (plate_with_hole + rib) ──
    let bracket = graph.add_node(NodeType::BooleanUnion, 600.0, 0.0);
    graph.connect(plate_with_hole, 0, bracket, 0);
    graph.connect(move_rib, 0, bracket, 1);

    // ── Node 8: Fillet edges ──
    let fillet = graph.add_node(NodeType::Fillet { radius: 2.0 }, 800.0, 0.0);
    graph.connect(bracket, 0, fillet, 0);

    // ── Node 9: BakeToDoc ──
    let bake = graph.add_node(NodeType::BakeToDoc, 1000.0, 0.0);
    graph.connect(fillet, 0, bake, 0);

    println!("VP Graph: {} nodes, {} connections",
        graph.node_count(), graph.connection_count());

    // ── Evaluate ──
    println!("\nEvaluating VP graph...");
    let solid = vp_evaluate_graph(&graph);

    match &solid {
        Some(s) => {
            let n_faces = s.faces().len();
            println!("✓ VP evaluation produced a Solid with {} faces", n_faces);

            // Create examples/ directory
            let dir = "examples";
            std::fs::create_dir_all(dir).ok();

            // ── Export STEP ──
            println!("\nExporting to STEP...");
            let step_content = export_step(s, "engine_bracket");
            let step_path = format!("{}/engine_bracket.step", dir);
            match write_step_file(&step_content, &step_path) {
                Ok(_) => println!("✓ STEP exported: {} ({} bytes)", step_path, step_content.len()),
                Err(e) => eprintln!("✗ STEP export failed: {}", e),
            }

            // ── Triangulate ──
            println!("\nTriangulating solid...");
            let params = TriangulationParams::default();
            let mesh = triangulate_solid(s, &params);
            println!("✓ Mesh: {} vertices, {} triangles",
                mesh.vertex_count(), mesh.triangle_count());

            // ── Export STL (binary) ──
            println!("\nExporting to STL (binary)...");
            let stl_path = format!("{}/engine_bracket.stl", dir);
            match stl::write_stl_file(&mesh, &stl_path, true) {
                Ok(_) => println!("✓ STL exported: {}", &stl_path),
                Err(e) => eprintln!("✗ STL export failed: {}", e),
            }

            // ── Export STL (ASCII) ──
            println!("\nExporting to STL (ASCII)...");
            let stl_ascii_path = format!("{}/engine_bracket_ascii.stl", dir);
            match stl::write_stl_file(&mesh, &stl_ascii_path, false) {
                Ok(_) => println!("✓ STL (ASCII) exported: {}", &stl_ascii_path),
                Err(e) => eprintln!("✗ STL (ASCII) export failed: {}", e),
            }

            // ── Export OBJ ──
            println!("\nExporting to OBJ...");
            let obj_path = format!("{}/engine_bracket.obj", dir);
            match stl::write_obj_file(&mesh, &obj_path) {
                Ok(_) => println!("✓ OBJ exported: {}", &obj_path),
                Err(e) => eprintln!("✗ OBJ export failed: {}", e),
            }

            println!("\n=== Done! ===");
            println!("Files saved to {}/:", dir);
            println!("  - engine_bracket.step  (STEP AP214)");
            println!("  - engine_bracket.stl   (STL binary)");
            println!("  - engine_bracket_ascii.stl (STL ASCII)");
            println!("  - engine_bracket.obj   (Wavefront OBJ)");
        }
        None => {
            eprintln!("✗ VP evaluation returned None — no solid produced");
            std::process::exit(1);
        }
    }
}
