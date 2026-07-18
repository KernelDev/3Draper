// Find the EXACT vertices that differ between Plane and NURBS faces
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashSet;

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
            let f96 = inst.faces.iter().find(|f| f.step_face_id == 96).unwrap();
            let f192 = inst.faces.iter().find(|f| f.step_face_id == 192).unwrap();

            let (s96, e96) = f96.triangle_range;
            let (s192, e192) = f192.triangle_range;

            // Collect positions (not indices) from each face
            let mut v96_pos: HashSet<[u64; 3]> = HashSet::new();
            let mut v96_list: Vec<([f64; 3], u32)> = Vec::new();
            for i in s96..e96 {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    let v = inst.mesh.vertices[vi as usize];
                    let bits = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
                    if v96_pos.insert(bits) {
                        v96_list.push(([v.x, v.y, v.z], vi));
                    }
                }
            }

            let mut v192_pos: HashSet<[u64; 3]> = HashSet::new();
            let mut v192_list: Vec<([f64; 3], u32)> = Vec::new();
            for i in s192..e192 {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    let v = inst.mesh.vertices[vi as usize];
                    let bits = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
                    if v192_pos.insert(bits) {
                        v192_list.push(([v.x, v.y, v.z], vi));
                    }
                }
            }

            // Find vertices in v96 but NOT in v192
            println!("=== Vertices in Plane but NOT in NURBS ===");
            for (pos, vi) in &v96_list {
                let bits = [pos[0].to_bits(), pos[1].to_bits(), pos[2].to_bits()];
                if !v192_pos.contains(&bits) {
                    println!("  v{}: ({:.6}, {:.6}, {:.6})", vi, pos[0], pos[1], pos[2]);
                }
            }

            // Find vertices in v192 but NOT in v96
            println!("\n=== Vertices in NURBS but NOT in Plane ===");
            for (pos, vi) in &v192_list {
                let bits = [pos[0].to_bits(), pos[1].to_bits(), pos[2].to_bits()];
                if !v96_pos.contains(&bits) {
                    println!("  v{}: ({:.6}, {:.6}, {:.6})", vi, pos[0], pos[1], pos[2]);
                }
            }

            break;
        }
    }
}
