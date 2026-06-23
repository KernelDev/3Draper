//! Verify LOD actually changes vertex/triangle counts when using new_with_params

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let path = std::env::args().nth(1).unwrap_or("test/Zentralstaender.stp".to_string());
    println!("Loading: {}", path);
    let content = std::fs::read_to_string(&path).expect("read step file");
    let step_file = draper_step::parser::parse_step(&content).expect("parse step file");

    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    println!("Pending instances: {}", pending.len());

    println!("\n=== LOD comparison (new_with_params) ===");
    println!("{:<10} {:<10} {:<12} {:<12} {:<10}", "LOD", "ok", "verts", "tris", "params.dev");
    for lod in [0.1, 0.3, 0.5, 0.75, 1.0] {
        let params = draper_mesh::TriangulationParams::for_lod(lod);
        let dev = params.max_deviation;
        let mut ctx = draper_step::OwnedStepConversionContext::new_with_params(
            step_file.clone(),
            params,
        );

        let mut total_verts = 0usize;
        let mut total_tris = 0usize;
        let mut ok_count = 0usize;
        let mut fail_count = 0usize;
        for p in &pending {
            match ctx.triangulate_pending(p) {
                Some(inst) => {
                    total_verts += inst.mesh.vertex_count();
                    total_tris += inst.mesh.triangle_count();
                    ok_count += 1;
                }
                None => fail_count += 1,
            }
        }
        println!(
            "{:<10} {:<10} {:<12} {:<12} {:<10.6}",
            lod, ok_count, total_verts, total_tris, dev
        );
    }

    println!("\n=== Old behavior (default params, simulating the bug) ===");
    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file.clone());
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
    println!("default: ok={} verts={} tris={}", ok_count, total_verts, total_tris);
}
