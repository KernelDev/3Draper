//! Test mesh generation for STEP files

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // Test SampleCube.step
    println!("=== Testing SampleCube.step ===");
    let content = std::fs::read_to_string("test/SampleCube.step").expect("Failed to read");
    let parsed = draper_step::bridge::parse_step(&content).expect("Failed to parse");
    let shape = draper_core::step_bridge::step_model_to_shape(&parsed.model);
    
    println!("Shape: {} vertices, {} edges, {} faces, {} solids",
        shape.vertices().len(),
        shape.edges().len(),
        shape.faces().len(),
        shape.solids().len(),
    );
    
    let mesh = draper_mesh::generate::generate_mesh(&shape, 32, 32);
    println!("Mesh: {} vertices, {} triangles (expected: 12 for a cube)", mesh.vertex_count(), mesh.triangle_count());
    
    if mesh.triangle_count() == 12 {
        println!("✓ Cube mesh is correct!");
    } else {
        println!("✗ Cube mesh is WRONG (expected 12 triangles)");
    }

    println!();

    // Test 3.05.078.stp
    println!("=== Testing 3.05.078.stp ===");
    let content = std::fs::read_to_string("test/3.05.078.stp").expect("Failed to read");
    let parsed = draper_step::bridge::parse_step(&content).expect("Failed to parse");
    let shape = draper_core::step_bridge::step_model_to_shape(&parsed.model);
    
    println!("Shape: {} vertices, {} edges, {} faces, {} solids",
        shape.vertices().len(),
        shape.edges().len(),
        shape.faces().len(),
        shape.solids().len(),
    );
    
    let mesh = draper_mesh::generate::generate_mesh(&shape, 32, 32);
    println!("Mesh: {} vertices, {} triangles", mesh.vertex_count(), mesh.triangle_count());
    
    // Print face details for debugging
    for (i, face) in shape.faces().iter().enumerate() {
        let surface_type = match &face.surface {
            Some(s) => format!("{:?}", std::mem::discriminant(s)),
            None => "None".to_string(),
        };
        let n_edges = if let Some(wire_id) = face.outer_wire {
            if let Some(draper_topology::entity::TopoShape::Wire(w)) = shape.get(wire_id) {
                w.edges.len()
            } else { 0 }
        } else { 0 };
        println!("  Face #{}: surface={}, edges={}, orient={}", i, surface_type, n_edges, face.orientation);
    }
}
