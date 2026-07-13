// Test LOD response on bolt.stp and full as1-oc-214.stp
use draper_step::{parser::parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::TriangulationParams;

fn test_file(path: &str, lods: &[f64]) {
    let content = std::fs::read_to_string(path).expect("read");
    let step_file = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step_file);

    println!("\n=== {} ({} BREPs) ===", path, pending.len());

    for &lod in lods {
        let params = TriangulationParams::for_lod(lod);
        let mut ctx = OwnedStepConversionContext::new_with_params(step_file.clone(), params);

        let mut total_tris = 0;
        let mut total_verts = 0;
        for p in &pending {
            if let Some(inst) = ctx.triangulate_pending(p) {
                total_tris += inst.mesh.triangle_count();
                total_verts += inst.mesh.vertex_count();
            }
        }
        println!("  LOD {:.2}: {} tris, {} verts, max_dev={:.4}",
            lod, total_tris, total_verts, TriangulationParams::for_lod(lod).max_deviation);
    }
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let lods = [0.1_f64, 0.3, 0.5, 0.75, 1.0];

    test_file("test/as1-oc-214_bolt.stp", &lods);
    test_file("test/as1-oc-214.stp", &lods);
}
