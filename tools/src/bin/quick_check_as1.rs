// Quick check: as1-oc-214.stp watertightness
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/as1-oc-214.stp")
        .expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    let mut total_boundary = 0;
    let mut total_tris = 0;
    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            total_boundary += report.boundary_edge_count;
            total_tris += inst.mesh.triangle_count();
            if !report.is_watertight() {
                println!("BREP #{} ({}): v={} t={} boundary={}",
                    p.brep_id, p.name, inst.mesh.vertex_count(),
                    inst.mesh.triangle_count(), report.boundary_edge_count);
            }
        }
    }
    println!("\nTotal: {} tris, {} boundary edges, watertight={}",
        total_tris, total_boundary, total_boundary == 0);
}
