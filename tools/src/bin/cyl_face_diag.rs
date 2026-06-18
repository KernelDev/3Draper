//! Diagnose cylinder faces in a STEP file that produce 0 triangles.

use draper_step::{parse_step, StepConversionContext, step_structure_lazy};
use log::LevelFilter;

fn main() {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .filter_module("draper_mesh::parametric_domain", LevelFilter::Debug)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).cloned().unwrap_or_else(|| "test/Zentralstaender.stp".to_string());
    
    println!("Loading: {}", path);
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    println!("{} BREP instances", pending.len());
    
    // Find a BREP with cylinder face failures (BREP #1070)
    let target_brep = pending.iter().find(|p| p.brep_id == 1070);
    if target_brep.is_none() {
        println!("BREP #1070 not found, listing all:");
        for p in &pending {
            println!("  BREP #{}: {}", p.brep_id, p.name);
        }
        return;
    }
    
    let pending = target_brep.unwrap();
    let ctx = StepConversionContext::new(&step);
    let inst = ctx.triangulate_pending(pending).expect("triangulate");
    println!("\nBREP #1070: {} verts, {} tris", inst.mesh.vertex_count(), inst.mesh.triangle_count());
}
