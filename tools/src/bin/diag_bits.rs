// Check bit-identity of edge cache points between Plane and NURBS faces
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/as1-oc-214_bolt.stp")
        .expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            // Get face boundaries
            let f96 = inst.faces.iter().find(|f| f.step_face_id == 96).unwrap();
            let f192 = inst.faces.iter().find(|f| f.step_face_id == 192).unwrap();

            // Check outer_boundary — these are sampled by sample_edges_to_polylines
            // (NOT the edge cache), so they won't match.
            // Instead, check the ACTUAL mesh vertices.

            let (s96, e96) = f96.triangle_range;
            let (s192, e192) = f192.triangle_range;

            // Collect all vertices from each face
            let mut v96: Vec<(u32, [f64; 3])> = Vec::new();
            let mut seen96: std::collections::HashSet<u32> = std::collections::HashSet::new();
            for i in s96..e96 {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    if seen96.insert(vi) {
                        let v = inst.mesh.vertices[vi as usize];
                        v96.push((vi, [v.x, v.y, v.z]));
                    }
                }
            }

            let mut v192: Vec<(u32, [f64; 3])> = Vec::new();
            let mut seen192: std::collections::HashSet<u32> = std::collections::HashSet::new();
            for i in s192..e192 {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    if seen192.insert(vi) {
                        let v = inst.mesh.vertices[vi as usize];
                        v192.push((vi, [v.x, v.y, v.z]));
                    }
                }
            }

            // For each v96 vertex, find the closest v192 vertex
            println!("=== Bit-identity check ===");
            let mut bit_identical = 0;
            let mut close = 0;
            let mut far = 0;
            for (_, p96) in &v96 {
                let mut best_d = f64::MAX;
                let mut best_bits = false;
                for (_, p192) in &v192 {
                    let dx = p96[0] - p192[0];
                    let dy = p96[1] - p192[1];
                    let dz = p96[2] - p192[2];
                    let d = (dx*dx + dy*dy + dz*dz).sqrt();
                    if d < best_d {
                        best_d = d;
                        best_bits = p96[0].to_bits() == p192[0].to_bits()
                            && p96[1].to_bits() == p192[1].to_bits()
                            && p96[2].to_bits() == p192[2].to_bits();
                    }
                }
                if best_bits {
                    bit_identical += 1;
                } else if best_d < 0.01 {
                    close += 1;
                } else {
                    far += 1;
                }
            }
            println!("  v96 vertices: {}", v96.len());
            println!("  v192 vertices: {}", v192.len());
            println!("  bit-identical: {}", bit_identical);
            println!("  close (<0.01mm): {}", close);
            println!("  far (>=0.01mm): {}", far);

            // Show first 5 vertices from each face with their bits
            println!("\n  v96 first 5 (with bits):");
            for (vi, p) in v96.iter().take(5) {
                println!("    vi={}: ({:.10},{:.10},{:.10}) bits=({:x},{:x},{:x})",
                    vi, p[0], p[1], p[2], p[0].to_bits(), p[1].to_bits(), p[2].to_bits());
            }
            println!("  v192 first 5 (with bits):");
            for (vi, p) in v192.iter().take(5) {
                println!("    vi={}: ({:.10},{:.10},{:.10}) bits=({:x},{:x},{:x})",
                    vi, p[0], p[1], p[2], p[0].to_bits(), p[1].to_bits(), p[2].to_bits());
            }

            break;
        }
    }
}
