//! Full validation check

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/Zentralstaender.stp".to_string());
    let content = std::fs::read_to_string(&path).expect("read step file");
    let step_file = draper_step::parser::parse_step(&content).expect("parse step file");
    
    let (tree, instances) = draper_step::converter::step_structure_with_instances(&step_file);
    
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
