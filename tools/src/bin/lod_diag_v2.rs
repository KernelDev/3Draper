//! Check actual params used at different LODs after bbox-floor

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    let path = std::env::args().nth(1).unwrap_or("test/Zentralstaender.stp".to_string());
    println!("Loading: {}", path);
    let content = std::fs::read_to_string(&path).expect("read step file");
    let step_file = draper_step::parser::parse_step(&content).expect("parse step file");
    
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    println!("Pending instances: {}", pending.len());
    
    println!("\n=== Compare params BEFORE and AFTER bbox-floor ===");
    for lod in [0.05, 0.1, 0.3, 0.5, 0.75, 1.0] {
        let params_raw = draper_mesh::TriangulationParams::for_lod(lod);
        let mut ctx = draper_step::OwnedStepConversionContext::new_with_params(
            step_file.clone(),
            draper_mesh::TriangulationParams::for_lod(lod),
        );
        // Force bbox computation
        let _ = ctx.triangulate_pending(&pending[0]);
        // We can't easily read params back, but we know the bbox floor is:
        // params.max_deviation = max(params.max_deviation, diagonal * 0.0002)
        // So we need to compute diagonal from bbox
        println!("LOD={:<5} raw_dev={:<10.6} raw_angular={:<3} raw_height={:<3} raw_max_tris={:<6} raw_edge_len={:<7.3}",
            lod, params_raw.max_deviation, params_raw.angular_samples, params_raw.height_samples,
            params_raw.max_face_triangles, params_raw.max_edge_length);
    }
    
    println!("\n=== Detailed stats per LOD ===");
    println!("{:<6} {:<10} {:<10} {:<12} {:<12}", "LOD", "ok", "verts", "tris", "tris/face");
    for lod in [0.05, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0] {
        let params = draper_mesh::TriangulationParams::for_lod(lod);
        let mut ctx = draper_step::OwnedStepConversionContext::new_with_params(
            step_file.clone(),
            params,
        );
        let mut total_verts = 0usize;
        let mut total_tris = 0usize;
        let mut ok_count = 0usize;
        for p in &pending {
            if let Some(inst) = ctx.triangulate_pending(p) {
                total_verts += inst.mesh.vertex_count();
                total_tris += inst.mesh.triangle_count();
                ok_count += 1;
            }
        }
        let avg_tris_per_face = total_tris as f64 / (ok_count as f64 * 30.0).max(1.0);
        println!("{:<6} {:<10} {:<10} {:<12} {:<12.1}", lod, ok_count, total_verts, total_tris, avg_tris_per_face);
    }
}
