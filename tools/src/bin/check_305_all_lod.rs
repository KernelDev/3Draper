// Check 3.05.078.stp watertightness at ALL LOD levels
use draper_step::{parser::parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::{TriangulationParams, validate_watertight};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let path = "test/3.05.078.stp";
    let content = std::fs::read_to_string(path).expect("read");
    let step_file = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step_file);

    println!("=== {} ({} BREPs) ===", path, pending.len());

    for &lod in &[0.1_f64, 0.3, 0.5, 0.75, 1.0] {
        let params = TriangulationParams::for_lod(lod);
        let mut ctx = OwnedStepConversionContext::new_with_params(step_file.clone(), params);

        for p in &pending {
            if let Some(inst) = ctx.triangulate_pending(p) {
                let mesh = &inst.mesh;
                let report = validate_watertight(mesh, false);
                println!(
                    "LOD {:.2}: {} tris, {} verts, watertight={}, boundary={}, non_manifold={}, euler={}",
                    lod, mesh.triangle_count(), mesh.vertex_count(),
                    report.is_watertight(), report.boundary_edge_count,
                    report.non_manifold_edge_count, report.euler_characteristic
                );
            }
        }
    }
}
