// Compare actual 3D vertex positions between Plane and NURBS faces
// for shared edges in the bolt
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
            for fi in &inst.faces {
                if fi.step_face_id == 96 || fi.step_face_id == 192 {
                    println!("\n=== Face Step#{} {} ===", fi.step_face_id, fi.surface_type);
                    // Print outer boundary
                    for (pi, polyline) in fi.outer_boundary.iter().enumerate() {
                        println!("  outer[{}] ({} pts):", pi, polyline.len());
                        for (i, p) in polyline.iter().enumerate().take(10) {
                            println!("    [{}]: ({:.10}, {:.10}, {:.10})", i, p.x, p.y, p.z);
                        }
                    }
                }
            }

            // Also check: for the merged mesh, compare vertices at the same 3D position
            // between face 1 (Step#96 Plane) and face 3 (Step#192 NURBS)
            let fids = inst.mesh.triangle_face_ids.as_ref();
            if let Some(fids) = fids {
                // Find vertices used by face 1 (Step#96)
                let face1_start = inst.faces.iter().find(|f| f.step_face_id == 96).map(|f| f.triangle_range.0).unwrap_or(0);
                let face1_end = inst.faces.iter().find(|f| f.step_face_id == 96).map(|f| f.triangle_range.1).unwrap_or(0);
                let face3_start = inst.faces.iter().find(|f| f.step_face_id == 192).map(|f| f.triangle_range.0).unwrap_or(0);
                let face3_end = inst.faces.iter().find(|f| f.step_face_id == 192).map(|f| f.triangle_range.1).unwrap_or(0);

                let face1_verts: std::collections::HashSet<u32> = (face1_start..face1_end)
                    .flat_map(|i| {
                        let tri = inst.mesh.triangles[i];
                        [tri[0], tri[1], tri[2]]
                    })
                    .collect();
                let face3_verts: std::collections::HashSet<u32> = (face3_start..face3_end)
                    .flat_map(|i| {
                        let tri = inst.mesh.triangles[i];
                        [tri[0], tri[1], tri[2]]
                    })
                    .collect();

                // Find the first face1 vertex and check if any face3 vertex is close
                println!("\n=== Cross-face vertex comparison ===");
                let mut found = 0;
                for &vi1 in face1_verts.iter() {
                    let v1 = inst.mesh.vertices[vi1 as usize];
                    for &vi3 in face3_verts.iter() {
                        let v3 = inst.mesh.vertices[vi3 as usize];
                        let d = ((v1.x-v3.x).powi(2) + (v1.y-v3.y).powi(2) + (v1.z-v3.z).powi(2)).sqrt();
                        if d < 1.0 {
                            println!("  face1 vi={} ({:.6},{:.6},{:.6}) → face3 vi={} ({:.6},{:.6},{:.6}) dist={:.6}",
                                vi1, v1.x, v1.y, v1.z, vi3, v3.x, v3.y, v3.z, d);
                            found += 1;
                            if found >= 5 { break; }
                        }
                    }
                    if found >= 5 { break; }
                }
                if found == 0 {
                    println!("  NO close vertices found between face1 and face3!");
                    // Show first 3 vertices from each face
                    println!("  Face1 first 3 verts:");
                    for &vi in face1_verts.iter().take(3) {
                        let v = inst.mesh.vertices[vi as usize];
                        println!("    vi={}: ({:.6},{:.6},{:.6})", vi, v.x, v.y, v.z);
                    }
                    println!("  Face3 first 3 verts:");
                    for &vi in face3_verts.iter().take(3) {
                        let v = inst.mesh.vertices[vi as usize];
                        println!("    vi={}: ({:.6},{:.6},{:.6})", vi, v.x, v.y, v.z);
                    }
                }
            }
            break;
        }
    }
}
