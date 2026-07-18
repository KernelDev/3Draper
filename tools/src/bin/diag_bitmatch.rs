// Check if strip triangulation boundary vertices are bit-identical to edge cache
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashMap;

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
            // Get face 1 (Step#96 Plane) and face 3 (Step#192 NURBS)
            let f96 = inst.faces.iter().find(|f| f.step_face_id == 96);
            let f192 = inst.faces.iter().find(|f| f.step_face_id == 192);

            if let (Some(f96), Some(f192)) = (f96, f192) {
                let (s96, e96) = f96.triangle_range;
                let (s192, e192) = f192.triangle_range;

                // Collect all vertices from each face, with their bit patterns
                let mut v96_bits: std::collections::HashSet<[u64; 3]> = std::collections::HashSet::new();
                let mut v96_list: Vec<([u64; 3], [f64; 3])> = Vec::new();
                for i in s96..e96 {
                    let tri = inst.mesh.triangles[i];
                    for &vi in &tri {
                        let v = inst.mesh.vertices[vi as usize];
                        let bits = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
                        if v96_bits.insert(bits) {
                            v96_list.push((bits, [v.x, v.y, v.z]));
                        }
                    }
                }

                let mut v192_bits: std::collections::HashSet<[u64; 3]> = std::collections::HashSet::new();
                let mut v192_list: Vec<([u64; 3], [f64; 3])> = Vec::new();
                for i in s192..e192 {
                    let tri = inst.mesh.triangles[i];
                    for &vi in &tri {
                        let v = inst.mesh.vertices[vi as usize];
                        let bits = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
                        if v192_bits.insert(bits) {
                            v192_list.push((bits, [v.x, v.y, v.z]));
                        }
                    }
                }

                // Find bit-identical vertices between the two faces
                let shared: Vec<&([u64; 3], [f64; 3])> = v96_list.iter()
                    .filter(|(bits, _)| v192_bits.contains(bits))
                    .collect();

                println!("Face #96 (Plane): {} unique vertices", v96_list.len());
                println!("Face #192 (NURBS): {} unique vertices", v192_list.len());
                println!("Bit-identical shared: {}", shared.len());

                // For shared vertices, print their positions
                for (i, (_, pos)) in shared.iter().enumerate().take(5) {
                    println!("  shared[{}]: ({:.6}, {:.6}, {:.6})", i, pos[0], pos[1], pos[2]);
                }

                // For non-shared vertices in v96, find closest in v192
                let mut near_count = 0;
                let mut near_dists: Vec<f64> = Vec::new();
                for (_, pos96) in &v96_list {
                    if v192_bits.contains(&[pos96[0].to_bits(), pos96[1].to_bits(), pos96[2].to_bits()]) {
                        continue; // Already shared
                    }
                    let mut best_d = f64::MAX;
                    for (_, pos192) in &v192_list {
                        let d = ((pos96[0]-pos192[0]).powi(2) + (pos96[1]-pos192[1]).powi(2) + (pos96[2]-pos192[2]).powi(2)).sqrt();
                        if d < best_d { best_d = d; }
                    }
                    if best_d < 1.0 {
                        near_count += 1;
                        near_dists.push(best_d);
                    }
                }
                if !near_dists.is_empty() {
                    near_dists.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    println!("Near-miss (not bit-identical, <1mm): {}", near_count);
                    println!("  dist: min={:.6} med={:.6} max={:.6}",
                        near_dists[0], near_dists[near_dists.len()/2], near_dists[near_dists.len()-1]);
                }
            }
            break;
        }
    }
}
