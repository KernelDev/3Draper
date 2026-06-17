//! Test NURBS triangulation

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    
    let path = std::env::args().nth(1).unwrap_or("test/nist_complex_surface.stp".to_string());
    println!("Loading: {}", path);
    let content = std::fs::read_to_string(&path).expect("read");
    let step_file = draper_step::parser::parse_step(&content).expect("parse");
    
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    println!("Pending BREP instances: {}", pending.len());
    
    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);
    
    for (i, p) in pending.iter().enumerate().take(2) {
        println!("\n=== Instance {} BREP#{} '{}' ===", i, p.brep_id, p.name);
        if let Some(inst) = ctx.triangulate_pending(p) {
            println!("  Faces: {}", inst.faces.len());
            println!("  Vertices: {}", inst.mesh.vertices.len());
            println!("  Triangles: {}", inst.mesh.triangle_count());
            
            for (fi, face) in inst.faces.iter().enumerate().take(10) {
                let tris = face.triangle_range.1.saturating_sub(face.triangle_range.0);
                println!("    Face {}: surface={} tris={} bnd_pts={}", 
                    fi, face.surface_type, tris, face.outer_boundary.first().map(|v| v.len()).unwrap_or(0));
            }
            
            let (boundary, total) = count_edges(&inst.mesh);
            println!("  Edges: total={} boundary={} ({:.1}%)", 
                total, boundary, 100.0 * boundary as f64 / total.max(1) as f64);
        }
    }
}

fn count_edges(mesh: &draper_mesh::TriangleMesh) -> (usize, usize) {
    use std::collections::HashMap;
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        for i in 0..3 {
            let a = tri[i].min(tri[(i+1)%3]);
            let b = tri[i].max(tri[(i+1)%3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let boundary = edge_count.values().filter(|c| **c == 1).count();
    let total = edge_count.len();
    (boundary, total)
}
