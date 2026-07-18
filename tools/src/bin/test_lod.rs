// Test LOD: triangulate 3.05.078.stp at different LOD levels and check triangle count
use draper_step::{parse_step, OwnedStepConversionContext, step_structure_lazy};
use draper_mesh::TriangulationParams;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/3.05.078.stp").expect("read");
    let step = parse_step(&content).expect("parse");

    for lod in &[0.1, 0.3, 0.5, 0.75, 1.0] {
        let params = TriangulationParams::for_lod(*lod);
        let max_dev = params.max_deviation;
        let mut ctx = OwnedStepConversionContext::new_with_params(step.clone(), params);
        let (_tree, pending) = step_structure_lazy(&step);
        let mut total_tris = 0;
        let mut total_verts = 0;
        for p in &pending {
            if let Some(inst) = ctx.triangulate_pending(p) {
                total_tris += inst.mesh.triangle_count();
                total_verts += inst.mesh.vertex_count();
            }
        }
        println!("LOD={:.2}: verts={} tris={} max_deviation={:.6}",
            lod, total_verts, total_tris, max_dev);
    }
}
