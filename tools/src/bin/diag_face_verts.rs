// Check how many boundary points each face has for shared edges
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/as1-oc-214_bolt.stp")
        .expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            for fi in &inst.faces {
                let (start, end) = fi.triangle_range;
                let mut verts: std::collections::HashSet<u32> = std::collections::HashSet::new();
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    for &vi in &tri { verts.insert(vi); }
                }
                println!("Face Step#{} {}: {} verts, {} tris",
                    fi.step_face_id, fi.surface_type, verts.len(), end - start);

                // Check z-levels
                let mut z_counts: std::collections::HashMap<i64, usize> = std::collections::HashMap::new();
                for &vi in &verts {
                    let v = inst.mesh.vertices[vi as usize];
                    let z_level = (v.z * 10.0).round() as i64;
                    *z_counts.entry(z_level).or_insert(0) += 1;
                }
                let mut levels: Vec<_> = z_counts.into_iter().collect();
                levels.sort_by_key(|(l, _)| *l);
                for (z_level, count) in &levels {
                    println!("  z={:.1}: {} verts", *z_level as f64 / 10.0, count);
                }
            }
            break;
        }
    }
}
