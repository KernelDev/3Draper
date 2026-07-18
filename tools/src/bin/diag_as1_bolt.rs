// Check as1-oc-214.stp bolt file — all BREPs
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = "/home/z/my-project/3Draper/test/as1-oc-214_bolt.stp";
    let content = std::fs::read_to_string(path).expect("read step");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    println!("Pending BREPs: {}", pending.len());

    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            println!("\n=== BREP #{} ({}) instance {} ===", p.brep_id, p.name, i);
            println!("  verts={} tris={} watertight={} boundary={}",
                inst.mesh.vertex_count(), inst.mesh.triangle_count(),
                report.is_watertight(), report.boundary_edge_count);

            if report.boundary_edge_count > 0 {
                let fids = inst.mesh.triangle_face_ids.as_ref();
                let mut edge_count_map: HashMap<(u32, u32), usize> = HashMap::new();
                for tri in &inst.mesh.triangles {
                    let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                    for (a, b) in edges {
                        let key = if a < b { (a, b) } else { (b, a) };
                        *edge_count_map.entry(key).or_insert(0) += 1;
                    }
                }

                let mut face_boundary_count: HashMap<u64, usize> = HashMap::new();
                for (edge, &count) in &edge_count_map {
                    if count == 1 {
                        for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                            let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                            for (a, b) in edges {
                                let key = if a < b { (a, b) } else { (b, a) };
                                if key == *edge {
                                    if let Some(fid) = fids.and_then(|f| f.get(ti).copied()) {
                                        *face_boundary_count.entry(fid).or_insert(0) += 1;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }

                let mut sorted: Vec<_> = face_boundary_count.into_iter().collect();
                sorted.sort_by(|a, b| b.1.cmp(&a.1));
                println!("  Faces with boundary edges:");
                for (fid, count) in sorted.iter().take(10) {
                    let fi = inst.faces.iter().find(|f| f.face_id == *fid);
                    if let Some(fi) = fi {
                        println!("    face_id={} (Step#{}, {}): {} boundary edges",
                            fid, fi.step_face_id, fi.surface_type, count);
                    }
                }
            }
        }
    }
}
