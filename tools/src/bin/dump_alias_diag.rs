// Diagnostic: check why edges #220 and #238 (same circle, different entities)
// are NOT being aliased in 3.05.078.stp
// Uses the public StepConversionContext API to triangulate, then inspects
// the edge cache stats and boundary edges.
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("read 3.05.078.stp");
    let step = parse_step(&content).expect("parse step");

    // Print the STEP entities for edges #220, #238, #222, #236
    for sid in &[220, 238, 222, 236] {
        if let Some(entity) = step.find_entity(*sid) {
            println!("#{} = {}({})", sid, entity.type_name, entity.params.iter()
                .map(|p| format!("{:?}", p))
                .collect::<Vec<_>>()
                .join(", "));
        }
    }

    // Print the CIRCLE entities #246, #274, #251, #272
    println!("\n--- Curve entities ---");
    for sid in &[246, 274, 251, 272] {
        if let Some(entity) = step.find_entity(*sid) {
            println!("#{} = {}({})", sid, entity.type_name, entity.params.iter()
                .map(|p| format!("{:?}", p))
                .collect::<Vec<_>>()
                .join(", "));
        }
    }

    // Print the AXIS2_PLACEMENT_3D entities
    println!("\n--- Axis2 placements ---");
    for sid in &[284, 318, 290, 316] {
        if let Some(entity) = step.find_entity(*sid) {
            println!("#{} = {}({})", sid, entity.type_name, entity.params.iter()
                .map(|p| format!("{:?}", p))
                .collect::<Vec<_>>()
                .join(", "));
        }
    }

    // Triangulate and check alias stats
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            println!("\n=== BREP #{}: {} faces ===", p.brep_id, inst.faces.len());
            break;
        }
    }
}
