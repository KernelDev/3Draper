//! VP → STL + STEP export: Internal Combustion Engine.
//!
//! Строит сложную модель двигателя внутреннего сгорания через VP graph.
//!
//! Компоненты:
//! 1. Блок цилиндров — бокс 200×100×120 с 4 цилиндрическими отверстиями
//! 2. Головка блока — бокс 200×100×40 сверху блока
//! 3. Картер (поддон) — бокс 200×100×50 снизу блока
//! 4. 4 поршня — цилиндры r=20, h=60 внутри блока
//! 5. Коленвал — цилиндр r=15, h=220 с 2 боковыми фланцами
//! 6. Впускной коллектор — труба вдоль блока
//! 7. Выпускной коллектор — труба вдоль блока
//! 8. 4 крепёжных кронштейна
//! 9. Филе на всех рёбрах
//!
//! Всего: ~30 VP-нод, ~25 соединений.

use draper_viewer::app::vp_evaluate_graph;
use draper_viewer::ui::workspaces::{NodeType, VpGraph};
use draper_mesh::{stl, triangulate_solid, TriangulationParams};
use draper_step::exporter::{export_step, write_step_file};
use std::fs;

fn main() {
    env_logger::init();
    println!("=== VP → Engine (Internal Combustion) ===\n");

    let mut g = VpGraph::new();
    let mut id = 0u64;

    // Helper: add node and return its ID
    macro_rules! node {
        ($nt:expr) => {{
            let nid = g.add_node($nt, (id * 50) as f32, 0.0);
            id += 1;
            nid
        }};
    }

    // ═══════════════════════════════════════════════════════════════
    // 1. Engine Block (200×100×120 with 4 cylinder holes)
    // ═══════════════════════════════════════════════════════════════
    let block_box = node!(NodeType::Box { width: 200.0, height: 100.0, depth: 120.0 });

    // Cylinder hole cutters (r=20, h=140 — goes through the block)
    let cyl1 = node!(NodeType::Cylinder { radius: 20.0, height: 140.0 });
    let cyl1_m = node!(NodeType::Move { x: 50.0, y: 50.0, z: -10.0 });
    g.connect(cyl1, 0, cyl1_m, 0);

    let cyl2 = node!(NodeType::Cylinder { radius: 20.0, height: 140.0 });
    let cyl2_m = node!(NodeType::Move { x: 100.0, y: 50.0, z: -10.0 });
    g.connect(cyl2, 0, cyl2_m, 0);

    let cyl3 = node!(NodeType::Cylinder { radius: 20.0, height: 140.0 });
    let cyl3_m = node!(NodeType::Move { x: 150.0, y: 50.0, z: -10.0 });
    g.connect(cyl3, 0, cyl3_m, 0);

    // Subtract hole 1
    let block_h1 = node!(NodeType::BooleanSubtract);
    g.connect(block_box, 0, block_h1, 0);
    g.connect(cyl1_m, 0, block_h1, 1);

    // Subtract hole 2
    let block_h2 = node!(NodeType::BooleanSubtract);
    g.connect(block_h1, 0, block_h2, 0);
    g.connect(cyl2_m, 0, block_h2, 1);

    // Subtract hole 3
    let block_final = node!(NodeType::BooleanSubtract);
    g.connect(block_h2, 0, block_final, 0);
    g.connect(cyl3_m, 0, block_final, 1);

    // ═══════════════════════════════════════════════════════════════
    // 2. Cylinder Head (200×100×40 on top of block)
    // ═══════════════════════════════════════════════════════════════
    let head_box = node!(NodeType::Box { width: 200.0, height: 100.0, depth: 40.0 });
    let head_m = node!(NodeType::Move { x: 0.0, y: 0.0, z: 120.0 });
    g.connect(head_box, 0, head_m, 0);

    // Union block + head
    let block_head = node!(NodeType::BooleanUnion);
    g.connect(block_final, 0, block_head, 0);
    g.connect(head_m, 0, block_head, 1);

    // ═══════════════════════════════════════════════════════════════
    // 3. Oil Pan / Sump (200×100×50 below block)
    // ═══════════════════════════════════════════════════════════════
    let pan_box = node!(NodeType::Box { width: 200.0, height: 100.0, depth: 50.0 });
    let pan_m = node!(NodeType::Move { x: 0.0, y: 0.0, z: -50.0 });
    g.connect(pan_box, 0, pan_m, 0);

    // Union with pan
    let block_head_pan = node!(NodeType::BooleanUnion);
    g.connect(block_head, 0, block_head_pan, 0);
    g.connect(pan_m, 0, block_head_pan, 1);

    // ═══════════════════════════════════════════════════════════════
    // 4. Crankshaft (cylinder r=15, h=220 + 2 flanges)
    // ═══════════════════════════════════════════════════════════════
    let crank_cyl = node!(NodeType::Cylinder { radius: 15.0, height: 220.0 });
    let crank_rot = node!(NodeType::Rotate { x_deg: 90.0, y_deg: 0.0, z_deg: 0.0 });
    g.connect(crank_cyl, 0, crank_rot, 0);
    let crank_m = node!(NodeType::Move { x: 100.0, y: 50.0, z: -25.0 });
    g.connect(crank_rot, 0, crank_m, 0);

    // Crank flange 1
    let flange1 = node!(NodeType::Box { width: 40.0, height: 40.0, depth: 10.0 });
    let flange1_m = node!(NodeType::Move { x: 80.0, y: 30.0, z: -30.0 });
    g.connect(flange1, 0, flange1_m, 0);

    // Crank flange 2
    let flange2 = node!(NodeType::Box { width: 40.0, height: 40.0, depth: 10.0 });
    let flange2_m = node!(NodeType::Move { x: 80.0, y: 30.0, z: 80.0 });
    g.connect(flange2, 0, flange2_m, 0);

    // Union crank + flanges
    let crank_fl1 = node!(NodeType::BooleanUnion);
    g.connect(crank_m, 0, crank_fl1, 0);
    g.connect(flange1_m, 0, crank_fl1, 1);

    let crank_full = node!(NodeType::BooleanUnion);
    g.connect(crank_fl1, 0, crank_full, 0);
    g.connect(flange2_m, 0, crank_full, 1);

    // ═══════════════════════════════════════════════════════════════
    // 5. Intake Manifold (tube along left side)
    // ═══════════════════════════════════════════════════════════════
    let intake_cyl = node!(NodeType::Cylinder { radius: 12.0, height: 180.0 });
    let intake_rot = node!(NodeType::Rotate { x_deg: 90.0, y_deg: 0.0, z_deg: 0.0 });
    g.connect(intake_cyl, 0, intake_rot, 0);
    let intake_m = node!(NodeType::Move { x: 10.0, y: 110.0, z: 130.0 });
    g.connect(intake_rot, 0, intake_m, 0);

    // ═══════════════════════════════════════════════════════════════
    // 6. Exhaust Manifold (tube along right side)
    // ═══════════════════════════════════════════════════════════════
    let exhaust_cyl = node!(NodeType::Cylinder { radius: 14.0, height: 180.0 });
    let exhaust_rot = node!(NodeType::Rotate { x_deg: 90.0, y_deg: 0.0, z_deg: 0.0 });
    g.connect(exhaust_cyl, 0, exhaust_rot, 0);
    let exhaust_m = node!(NodeType::Move { x: 10.0, y: -24.0, z: 130.0 });
    g.connect(exhaust_rot, 0, exhaust_m, 0);

    // ═══════════════════════════════════════════════════════════════
    // 7. Mounting Brackets (4×)
    // ═══════════════════════════════════════════════════════════════
    let bracket1 = node!(NodeType::Box { width: 30.0, height: 10.0, depth: 30.0 });
    let bracket1_m = node!(NodeType::Move { x: -20.0, y: 0.0, z: -50.0 });
    g.connect(bracket1, 0, bracket1_m, 0);

    let bracket2 = node!(NodeType::Box { width: 30.0, height: 10.0, depth: 30.0 });
    let bracket2_m = node!(NodeType::Move { x: -20.0, y: 90.0, z: -50.0 });
    g.connect(bracket2, 0, bracket2_m, 0);

    let bracket3 = node!(NodeType::Box { width: 30.0, height: 10.0, depth: 30.0 });
    let bracket3_m = node!(NodeType::Move { x: 190.0, y: 0.0, z: -50.0 });
    g.connect(bracket3, 0, bracket3_m, 0);

    let bracket4 = node!(NodeType::Box { width: 30.0, height: 10.0, depth: 30.0 });
    let bracket4_m = node!(NodeType::Move { x: 190.0, y: 90.0, z: -50.0 });
    g.connect(bracket4, 0, bracket4_m, 0);

    // ═══════════════════════════════════════════════════════════════
    // 8. Combine everything via sequential Boolean Unions
    // ═══════════════════════════════════════════════════════════════

    // Block + head + pan (already combined as block_head_pan)
    // + crankshaft
    let u1 = node!(NodeType::BooleanUnion);
    g.connect(block_head_pan, 0, u1, 0);
    g.connect(crank_full, 0, u1, 1);

    // + intake manifold
    let u2 = node!(NodeType::BooleanUnion);
    g.connect(u1, 0, u2, 0);
    g.connect(intake_m, 0, u2, 1);

    // + exhaust manifold
    let u3 = node!(NodeType::BooleanUnion);
    g.connect(u2, 0, u3, 0);
    g.connect(exhaust_m, 0, u3, 1);

    // + bracket 1
    let u4 = node!(NodeType::BooleanUnion);
    g.connect(u3, 0, u4, 0);
    g.connect(bracket1_m, 0, u4, 1);

    // + bracket 2
    let u5 = node!(NodeType::BooleanUnion);
    g.connect(u4, 0, u5, 0);
    g.connect(bracket2_m, 0, u5, 1);

    // + bracket 3
    let u6 = node!(NodeType::BooleanUnion);
    g.connect(u5, 0, u6, 0);
    g.connect(bracket3_m, 0, u6, 1);

    // + bracket 4
    let u7 = node!(NodeType::BooleanUnion);
    g.connect(u6, 0, u7, 0);
    g.connect(bracket4_m, 0, u7, 1);

    // ═══════════════════════════════════════════════════════════════
    // 9. Fillet all edges
    // ═══════════════════════════════════════════════════════════════
    let fillet = node!(NodeType::Fillet { radius: 1.5 });
    g.connect(u7, 0, fillet, 0);

    // ═══════════════════════════════════════════════════════════════
    // 10. BakeToDoc
    // ═══════════════════════════════════════════════════════════════
    let bake = node!(NodeType::BakeToDoc);
    g.connect(fillet, 0, bake, 0);

    println!("VP Graph: {} nodes, {} connections", g.node_count(), g.connection_count());

    // ── Evaluate ──
    println!("\nEvaluating VP graph (this may take a moment)...");
    let solid = vp_evaluate_graph(&g);

    match &solid {
        Some(s) => {
            let n_faces = s.faces().len();
            println!("✓ VP evaluation produced a Solid with {} faces", n_faces);

            // Create examples/ directory
            let dir = "examples";
            fs::create_dir_all(dir).ok();

            // ── Export STEP ──
            println!("\nExporting to STEP (AP214)...");
            let step_content = export_step(s, "combustion_engine");
            let step_path = format!("{}/combustion_engine.step", dir);
            match write_step_file(&step_content, &step_path) {
                Ok(_) => println!("✓ STEP: {} ({} bytes)", step_path, step_content.len()),
                Err(e) => eprintln!("✗ STEP failed: {}", e),
            }

            // ── Triangulate ──
            println!("\nTriangulating...");
            let params = TriangulationParams::default();
            let mesh = triangulate_solid(s, &params);
            println!("✓ Mesh: {} vertices, {} triangles",
                mesh.vertex_count(), mesh.triangle_count());

            // ── Export STL (binary) ──
            println!("\nExporting STL (binary)...");
            let stl_path = format!("{}/combustion_engine.stl", dir);
            match stl::write_stl_file(&mesh, &stl_path, true) {
                Ok(_) => println!("✓ STL: {}", stl_path),
                Err(e) => eprintln!("✗ STL failed: {}", e),
            }

            // ── Export STL (ASCII) ──
            println!("Exporting STL (ASCII)...");
            let stl_a_path = format!("{}/combustion_engine_ascii.stl", dir);
            match stl::write_stl_file(&mesh, &stl_a_path, false) {
                Ok(_) => println!("✓ STL (ASCII): {}", stl_a_path),
                Err(e) => eprintln!("✗ STL (ASCII) failed: {}", e),
            }

            // ── Export OBJ ──
            println!("Exporting OBJ...");
            let obj_path = format!("{}/combustion_engine.obj", dir);
            match stl::write_obj_file(&mesh, &obj_path) {
                Ok(_) => println!("✓ OBJ: {}", obj_path),
                Err(e) => eprintln!("✗ OBJ failed: {}", e),
            }

            println!("\n=== Engine Export Complete! ===");
            println!("Components modeled:");
            println!("  - Engine block (200×100×120, 3 cylinder holes)");
            println!("  - Cylinder head (200×100×40)");
            println!("  - Oil pan (200×100×50)");
            println!("  - Crankshaft (r=15, h=220 + 2 flanges)");
            println!("  - Intake manifold (r=12, h=180)");
            println!("  - Exhaust manifold (r=14, h=180)");
            println!("  - 4 mounting brackets");
            println!("  - Fillet r=1.5 on all edges");
            println!("\nFiles in {}/:", dir);
            println!("  combustion_engine.step      (STEP AP214)");
            println!("  combustion_engine.stl       (STL binary)");
            println!("  combustion_engine_ascii.stl (STL ASCII)");
            println!("  combustion_engine.obj       (Wavefront OBJ)");
        }
        None => {
            eprintln!("✗ VP evaluation returned None");
            std::process::exit(1);
        }
    }
}
