// Diagnostic: check face data for step_id=236 edge in 3.05.078.stp
// Uses the public API of draper-step (OwnedStepConversionContext)
use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = "test/3.05.078.stp";
    let data = std::fs::read_to_string(path).expect("Failed to read file");
    let step = parse_step(&data).expect("Failed to parse");

    // Get the pending BREP instances
    let (_tree, pending) = step_structure_lazy(&step);
    let mut ctx = OwnedStepConversionContext::new(step);

    // Triangulate each pending instance and dump info
    for p in &pending {
        if let Some(instance) = ctx.triangulate_pending(p) {
            let mesh = &instance.mesh;
            println!(
                "Instance '{}': {} vertices, {} triangles, brep_id={}",
                instance.name,
                mesh.vertices.len(),
                mesh.triangles.len(),
                instance.brep_id,
            );
        } else {
            println!("Instance '{}': triangulation failed (brep_id={})", p.name, p.brep_id);
        }
    }
}
