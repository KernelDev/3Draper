//! Dump face_mesh_with_ids (BEFORE merge) for a face with a specific STEP face ID.
//! Usage: face_pre_merge_dump <file.stp> <step_face_id>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let target_step_id: i64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(803);

    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    for p in &pending {
        let ctx = StepConversionContext::new(&step);
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => continue,
        };

        for (face_idx, face) in inst.faces.iter().enumerate() {
            if face.step_face_id != target_step_id { continue; }

            let tri_start = face.triangle_range.0 as usize;
            let tri_end = face.triangle_range.1 as usize;
            let tris = tri_end - tri_start;
            println!("\n=== BREP #{} Face #{} (STEP #{}, surf={}) === tris={}, forward={}",
                p.brep_id, face_idx + 1, face.step_face_id, face.surface_type, tris, face.forward);

            // Check z-coords of all vertices in this face's triangles
            let mut z_min = f64::MAX;
            let mut z_max = f64::MIN;
            let mut bad_z_count = 0;  // z outside [-3.225, -3.213]
            let mut bad_z_examples: Vec<(u32, f64, f64, f64)> = Vec::new();
            let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
            for ti in tri_start..tri_end {
                let tri = inst.mesh.triangles[ti];
                for &vi in &tri {
                    if !seen.insert(vi) { continue; }
                    let v = inst.mesh.vertices[vi as usize];
                    z_min = z_min.min(v.z);
                    z_max = z_max.max(v.z);
                    if v.z < -3.225 || v.z > -3.213 {
                        bad_z_count += 1;
                        if bad_z_examples.len() < 20 {
                            bad_z_examples.push((vi, v.x, v.y, v.z));
                        }
                    }
                }
            }
            println!("Z range of vertices: [{:.4}, {:.4}]", z_min, z_max);
            println!("Vertices with z outside [-3.225, -3.213]: {}", bad_z_count);
            for (vi, x, y, z) in &bad_z_examples {
                println!("  v{}: ({:.4}, {:.4}, {:.4})", vi, x, y, z);
            }

            // Print all unique triangles with at least one bad-z vertex
            println!("\nTriangles with at least one bad-z vertex:");
            let mut count = 0;
            for ti in tri_start..tri_end {
                let tri = inst.mesh.triangles[ti];
                let vs = [
                    inst.mesh.vertices[tri[0] as usize],
                    inst.mesh.vertices[tri[1] as usize],
                    inst.mesh.vertices[tri[2] as usize],
                ];
                let has_bad = vs.iter().any(|v| v.z < -3.225 || v.z > -3.213);
                if has_bad {
                    println!("  t{}: v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4})",
                        ti - tri_start,
                        tri[0], vs[0].x, vs[0].y, vs[0].z,
                        tri[1], vs[1].x, vs[1].y, vs[1].z,
                        tri[2], vs[2].x, vs[2].y, vs[2].z);
                    count += 1;
                    if count >= 30 { break; }
                }
            }
            if count == 0 {
                println!("  (none)");
            }
        }
    }
}
