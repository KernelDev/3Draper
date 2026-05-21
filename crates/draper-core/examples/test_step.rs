//! Quick test for STEP file parsing with step-io.

use std::fs;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // Test SampleCube.step
    println!("=== Testing SampleCube.step ===");
    let content = fs::read_to_string("test/SampleCube.step").expect("Failed to read SampleCube.step");
    match draper_step::bridge::parse_step(&content) {
        Ok(parsed) => {
            println!("  Parsed successfully!");
            println!("  Model: {} points, {} directions, {} curves, {} surfaces",
                parsed.model.geometry.points.len(),
                parsed.model.geometry.directions.len(),
                parsed.model.geometry.curves.len(),
                parsed.model.geometry.surfaces.len(),
            );
            println!("  Topo: {} vertices, {} edges, {} wires, {} faces, {} shells, {} solids",
                parsed.model.topology.vertices.len(),
                parsed.model.topology.edges.len(),
                parsed.model.topology.wires.len(),
                parsed.model.topology.faces.len(),
                parsed.model.topology.shells.len(),
                parsed.model.topology.solids.len(),
            );
            println!("  Schema: {:?}", parsed.model.schema);

            // Test shape conversion
            let shape = draper_core::step_bridge::step_model_to_shape(&parsed.model);
            println!("  Shape: {} vertices, {} edges, {} faces, {} solids",
                shape.vertices().len(),
                shape.edges().len(),
                shape.faces().len(),
                shape.solids().len(),
            );
        }
        Err(e) => {
            println!("  ERROR: {}", e);
        }
    }

    println!();

    // Test 3.05.078.stp
    println!("=== Testing 3.05.078.stp ===");
    let content = fs::read_to_string("test/3.05.078.stp").expect("Failed to read 3.05.078.stp");
    match draper_step::bridge::parse_step(&content) {
        Ok(parsed) => {
            println!("  Parsed successfully!");
            println!("  Model: {} points, {} directions, {} curves, {} surfaces",
                parsed.model.geometry.points.len(),
                parsed.model.geometry.directions.len(),
                parsed.model.geometry.curves.len(),
                parsed.model.geometry.surfaces.len(),
            );
            println!("  Topo: {} vertices, {} edges, {} wires, {} faces, {} shells, {} solids",
                parsed.model.topology.vertices.len(),
                parsed.model.topology.edges.len(),
                parsed.model.topology.wires.len(),
                parsed.model.topology.faces.len(),
                parsed.model.topology.shells.len(),
                parsed.model.topology.solids.len(),
            );
            println!("  Schema: {:?}", parsed.model.schema);

            let shape = draper_core::step_bridge::step_model_to_shape(&parsed.model);
            println!("  Shape: {} vertices, {} edges, {} faces, {} solids",
                shape.vertices().len(),
                shape.edges().len(),
                shape.faces().len(),
                shape.solids().len(),
            );
        }
        Err(e) => {
            println!("  ERROR: {}", e);
        }
    }
}
