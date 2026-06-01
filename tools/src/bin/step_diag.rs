//! Full validation check

fn main() {
    // Initialize logger
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    
    let path = std::env::args().nth(1).unwrap_or("test/Zentralstaender.stp".to_string());
    let content = std::fs::read_to_string(&path).expect("read step file");
    let step_file = draper_step::parser::parse_step(&content).expect("parse step file");
    
    let (tree, instances) = draper_step::converter::step_structure_with_instances(&step_file);
    
    // Print assembly tree
    println!("=== Assembly Tree ===");
    print_tree(&tree, 0);
    println!();
    
    // Check for NaN/Inf
    let mut nan_count = 0;
    let mut inf_count = 0;
    let mut total_tris = 0;
    
    // Merge all instances
    let mut merged = draper_mesh::TriangleMesh::new();
    for inst in &instances {
        total_tris += inst.mesh.triangle_count();
        for v in &inst.mesh.vertices {
            if v.x.is_nan() || v.y.is_nan() || v.z.is_nan() { nan_count += 1; }
            if v.x.is_infinite() || v.y.is_infinite() || v.z.is_infinite() { inf_count += 1; }
        }
        merged.merge(&inst.mesh);
    }
    
    println!("Instances: {}", instances.len());
    println!("Total triangles: {}", total_tris);
    println!("Merged: {} vertices, {} triangles", merged.vertices.len(), merged.triangles.len());
    println!("NaN vertices: {}, Inf vertices: {}", nan_count, inf_count);
    
    // Bounding box
    let (bmin, bmax) = merged.bounding_box();
    println!("BBox: ({:.1},{:.1},{:.1}) - ({:.1},{:.1},{:.1})", 
        bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z);
    
    // Per-instance summary
    for (i, inst) in instances.iter().enumerate() {
        let tris = inst.mesh.triangle_count();
        if tris > 0 {
            let surface_types: Vec<String> = inst.faces.iter()
                .map(|f| f.surface_type.clone())
                .collect();
            let unique_types: Vec<String> = {
                let mut s = surface_types.clone();
                s.sort();
                s.dedup();
                s
            };
            println!("[{:2}] BREP#{:<5} {:>5} tris  faces: {}  types: {}", 
                i, inst.brep_id, tris, inst.faces.len(), unique_types.join(", "));
        }
    }
}

fn print_tree(node: &draper_step::AssemblyNode, depth: usize) {
    let indent = "  ".repeat(depth);
    let brep_str = match node.brep_id {
        Some(id) => format!(" BREP#{}", id),
        None => String::new(),
    };
    let inst_str = match node.instance_index {
        Some(idx) => format!(" [inst:{}]", idx),
        None => String::new(),
    };
    let child_str = if node.children.is_empty() { "" } else { &format!(" ({} children)", node.children.len()) };
    println!("{}{}{}{}{}", indent, node.name, brep_str, inst_str, child_str);
    for child in &node.children {
        print_tree(child, depth + 1);
    }
}
