//! Export the worker-path mesh of a STEP file to OBJ (+ vertex normals) for
//! offline rendering/inspection. Mirrors as1_lod_repro but writes files.
//!
//! Usage: as1_export_obj <step_file> <lod> <out_prefix>

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::triangulate::SteinerBudgetProfile;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/as1-oc-214.stp");
    let lod: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0.75);
    let prefix = args.get(3).cloned().unwrap_or_else(|| "as1_mesh".to_string());

    let content = std::fs::read_to_string(path).expect("read step file");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);

    let mut ctx = OwnedStepConversionContext::new_with_lod_and_profile(
        step.clone(),
        lod,
        SteinerBudgetProfile::Desktop,
    );

    let obj_path = format!("{}.obj", prefix);
    let mut obj = String::new();
    let mut obj_v_count = 0usize;

    let mut total_v = 0usize;
    let mut total_t = 0usize;
    for (i, p) in pending.iter().enumerate() {
        let Some(inst) = ctx.triangulate_pending(p) else { continue };
        total_v += inst.mesh.vertex_count();
        total_t += inst.mesh.triangle_count();
        obj.push_str(&format!("o part_{}_{}\n", i, p.name.replace(' ', "_")));

        let v_offset = obj_v_count; // global index of this instance's first vertex (0-based)
        for v in &inst.mesh.vertices {
            obj.push_str(&format!("v {:.6} {:.6} {:.6}\n", v.x, v.y, v.z));
            obj_v_count += 1;
        }
        if let Some(normals) = &inst.mesh.normals {
            for n in normals {
                obj.push_str(&format!("vn {:.6} {:.6} {:.6}\n", n[0], n[1], n[2]));
            }
        } else {
            for _ in &inst.mesh.vertices {
                obj.push_str("vn 0 0 1\n");
            }
        }
        for t in &inst.mesh.triangles {
            let (a, b, c) = (
                t[0] as usize + v_offset + 1,
                t[1] as usize + v_offset + 1,
                t[2] as usize + v_offset + 1,
            );
            // v//vn (same indices)
            obj.push_str(&format!("f {}//{} {}//{} {}//{}\n", a, a, b, b, c, c));
        }
    }
    std::fs::write(&obj_path, obj).expect("write obj");
    println!("wrote {} (V={} T={})", obj_path, total_v, total_t);
}
