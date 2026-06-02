use draper_step::{parse_step_file, step_structure_lazy, OwnedStepConversionContext};

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "test/as1-oc-214.stp".to_string());
    eprintln!("Loading {}...", path);
    
    match parse_step_file(&path) {
        Ok(step_file) => {
            let (_tree, pending) = step_structure_lazy(&step_file);
            eprintln!("{} pending instances", pending.len());
            
            let mut ctx = OwnedStepConversionContext::new(step_file);
            let mut total_tris = 0;
            let mut total_verts = 0;
            
            for (i, p) in pending.iter().enumerate() {
                if let Some(instance) = ctx.triangulate_pending(p) {
                    total_verts += instance.mesh.vertex_count();
                    total_tris += instance.mesh.triangle_count();
                    eprintln!("[{}] {} (brep={}): {}v {}t", i, p.name, p.brep_id, 
                        instance.mesh.vertex_count(), instance.mesh.triangle_count());
                    for face in &instance.faces {
                        let outer_pts: usize = face.outer_boundary.iter().map(|b| b.len()).sum();
                        let inner_holes = face.inner_boundaries.len();
                        if outer_pts <= 3 {
                            eprintln!("  ** LOW BOUNDARY: step_id={} type={} outer_pts={} holes={} forward={}",
                                face.step_face_id, face.surface_type, outer_pts, 
                                inner_holes, face.forward);
                        }
                    }
                } else {
                    eprintln!("[{}] {} (brep={}): FAILED TO TRIANGULATE", i, p.name, p.brep_id);
                }
            }
            eprintln!("\nTotal: {} vertices, {} triangles", total_verts, total_tris);
        }
        Err(e) => eprintln!("Error: {}", e),
    }
}
