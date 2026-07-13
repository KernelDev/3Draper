// Diagnostic: inspect STEP#3084 triangulation in as1-oc-214.stp
// Also test LOD response on this file
use draper_step::{parser::parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::TriangulationParams;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let path = "test/as1-oc-214.stp";
    let content = std::fs::read_to_string(path).expect("read");
    let step_file = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step_file);

    println!("File: {} ({} BREPs)", path, pending.len());

    // Find which BREP contains STEP#3084
    for (i, p) in pending.iter().enumerate() {
        let name = &p.name;
        let brep_id = p.brep_id;
        if i < 5 || name.contains("plate") || name.contains("Plate") {
            println!("BREP {}: name='{}', brep_id={}", i, name, brep_id);
        }
    }

    // Test at multiple LODs
    for &lod in &[0.1_f64, 0.5, 1.0] {
        let params = TriangulationParams::for_lod(lod);
        let mut ctx = OwnedStepConversionContext::new_with_params(step_file.clone(), params);

        let mut total_tris = 0;
        let mut total_verts = 0;
        for (i, p) in pending.iter().enumerate() {
            if let Some(inst) = ctx.triangulate_pending(p) {
                total_tris += inst.mesh.triangle_count();
                total_verts += inst.mesh.vertex_count();
                if i < 5 {
                    println!("  LOD {:.1} BREP {}: {} tris, {} verts",
                        lod, i, inst.mesh.triangle_count(), inst.mesh.vertex_count());
                }
            }
        }
        println!("LOD {:.1}: total {} tris, {} verts", lod, total_tris, total_verts);
    }
}
