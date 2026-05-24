//! Example: Generate an ICE engine model and export to STL/STEP.
//!
//! Run: cargo run --example engine_export

use draper_core::engine::{EngineConfig, build_engine};
use draper_core::Document;
use draper_topology::ShapeBuilder;
use draper_mesh::{triangulate_solid, TriangulationParams, stl::write_stl_file};
use draper_step::exporter::{export_step, write_step_file};

fn main() {
    env_logger::init();

    println!("=== 3Draper ICE Engine Generator ===\n");

    // === 1. Build the engine ===
    println!("Building ICE engine model...");
    let config = EngineConfig::default();
    let doc = build_engine(&config);

    println!("Engine configuration:");
    println!("  Bore: {} mm", config.bore);
    println!("  Stroke: {} mm", config.stroke);
    println!("  Cylinders: {}", config.cylinder_count);
    println!("  Con-rod length: {} mm", config.con_rod_length);
    println!("  Crank radius: {} mm", config.crank_radius);
    println!("  Deck height: {} mm", config.deck_height);
    println!();

    // === 2. Triangulate ===
    println!("Triangulating engine model...");
    let mesh = doc.triangulate();
    println!("  Vertices: {}", mesh.vertex_count());
    println!("  Triangles: {}", mesh.triangle_count());
    println!("  Surface area: {:.2} mm^2", mesh.surface_area());
    let (bbox_min, bbox_max) = mesh.bounding_box();
    println!("  Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})",
        bbox_min.x, bbox_min.y, bbox_min.z,
        bbox_max.x, bbox_max.y, bbox_max.z
    );
    println!();

    // === 3. Export to STL ===
    let stl_path = "/home/z/my-project/download/engine.stl";
    let mut mesh_with_normals = mesh.clone();
    mesh_with_normals.compute_face_normals();
    match write_stl_file(&mesh_with_normals, stl_path, true) {
        Ok(()) => println!("Exported binary STL: {}", stl_path),
        Err(e) => println!("STL export error: {}", e),
    }

    let stl_ascii_path = "/home/z/my-project/download/engine_ascii.stl";
    match write_stl_file(&mesh_with_normals, stl_ascii_path, false) {
        Ok(()) => println!("Exported ASCII STL: {}", stl_ascii_path),
        Err(e) => println!("STL export error: {}", e),
    }
    println!();

    // === 4. Export individual parts to STEP ===
    let solids = doc.solids();
    println!("Engine has {} solid parts:", solids.len());

    let part_names = [
        "Engine Block",
        "Piston 1", "Piston 2", "Piston 3", "Piston 4",
        "Crankshaft",
        "Con-Rod 1", "Con-Rod 2", "Con-Rod 3", "Con-Rod 4",
        "Cylinder Head",
        "Intake Valve 1", "Exhaust Valve 1",
        "Intake Valve 2", "Exhaust Valve 2",
        "Intake Valve 3", "Exhaust Valve 3",
        "Intake Valve 4", "Exhaust Valve 4",
        "Camshaft",
        "Oil Pan",
        "Flywheel",
    ];

    for (i, solid) in solids.iter().enumerate() {
        let name = part_names.get(i).unwrap_or(&"Unknown Part");
        let part_mesh = triangulate_solid(solid, &TriangulationParams::default());
        println!("  Part {}: {} ({} vertices, {} triangles)",
            i, name, part_mesh.vertex_count(), part_mesh.triangle_count());

        // Export each part to STEP
        let step_path = format!("/home/z/my-project/download/part_{}_{}.stp", i, name.to_lowercase().replace(' ', "_"));
        let step_content = export_step(solid, name);
        match write_step_file(&step_content, &step_path) {
            Ok(()) => {},
            Err(e) => println!("    STEP export error for {}: {}", name, e),
        }
    }
    println!();

    // === 5. Also export individual primitives ===
    println!("Exporting primitives...");

    let box_solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
    let box_mesh = triangulate_solid(&box_solid, &TriangulationParams::default());
    let mut box_mesh_n = box_mesh.clone();
    box_mesh_n.compute_face_normals();
    write_stl_file(&box_mesh_n, "/home/z/my-project/download/box.stl", true).unwrap();
    println!("  Box: {} vertices, {} triangles", box_mesh.vertex_count(), box_mesh.triangle_count());

    let cyl_solid = ShapeBuilder::make_cylinder(40.0, 100.0);
    let cyl_mesh = triangulate_solid(&cyl_solid, &TriangulationParams::default());
    let mut cyl_mesh_n = cyl_mesh.clone();
    cyl_mesh_n.compute_face_normals();
    write_stl_file(&cyl_mesh_n, "/home/z/my-project/download/cylinder.stl", true).unwrap();
    println!("  Cylinder: {} vertices, {} triangles", cyl_mesh.vertex_count(), cyl_mesh.triangle_count());

    let sphere_solid = ShapeBuilder::make_sphere(50.0);
    let sphere_mesh = triangulate_solid(&sphere_solid, &TriangulationParams::default());
    let mut sphere_mesh_n = sphere_mesh.clone();
    sphere_mesh_n.compute_face_normals();
    write_stl_file(&sphere_mesh_n, "/home/z/my-project/download/sphere.stl", true).unwrap();
    println!("  Sphere: {} vertices, {} triangles", sphere_mesh.vertex_count(), sphere_mesh.triangle_count());

    let torus_solid = ShapeBuilder::make_torus(50.0, 15.0);
    let torus_mesh = triangulate_solid(&torus_solid, &TriangulationParams::default());
    let mut torus_mesh_n = torus_mesh.clone();
    torus_mesh_n.compute_face_normals();
    write_stl_file(&torus_mesh_n, "/home/z/my-project/download/torus.stl", true).unwrap();
    println!("  Torus: {} vertices, {} triangles", torus_mesh.vertex_count(), torus_mesh.triangle_count());

    println!("\n=== Done! All files exported to /home/z/my-project/download/ ===");
}
