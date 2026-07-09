//! Dump 3D vertices and triangles for a face with a specific STEP face ID.
//! Usage: face_3d_by_step_id <file.stp> <step_face_id>

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

            // Print first 5 3D triangles
            println!("\n3D triangles (first 5):");
            for ti in tri_start..tri_end.min(tri_start + 5) {
                let tri = inst.mesh.triangles[ti];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                println!("  t{}: v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4})",
                    ti - tri_start, tri[0], v0.x, v0.y, v0.z,
                    tri[1], v1.x, v1.y, v1.z,
                    tri[2], v2.x, v2.y, v2.z);
            }

            // Compute 3D bbox
            let (mut min_x, mut max_x) = (f64::MAX, f64::MIN);
            let (mut min_y, mut max_y) = (f64::MAX, f64::MIN);
            let (mut min_z, mut max_z) = (f64::MAX, f64::MIN);
            for ti in tri_start..tri_end {
                let tri = inst.mesh.triangles[ti];
                for &vi in &tri {
                    let v = inst.mesh.vertices[vi as usize];
                    min_x = min_x.min(v.x); max_x = max_x.max(v.x);
                    min_y = min_y.min(v.y); max_y = max_y.max(v.y);
                    min_z = min_z.min(v.z); max_z = max_z.max(v.z);
                }
            }
            println!("\n3D bbox: x=[{:.4}, {:.4}] y=[{:.4}, {:.4}] z=[{:.4}, {:.4}]",
                min_x, max_x, min_y, max_y, min_z, max_z);

            // Triangle areas (full 3D area)
            let mut min_area = f64::MAX;
            let mut max_area: f64 = 0.0;
            let mut zero_area_count = 0;
            let mut n_long = 0;
            for ti in tri_start..tri_end {
                let tri = inst.mesh.triangles[ti];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                let cross = (
                    (v1.y - v0.y) * (v2.z - v0.z) - (v1.z - v0.z) * (v2.y - v0.y),
                    (v1.z - v0.z) * (v2.x - v0.x) - (v1.x - v0.x) * (v2.z - v0.z),
                    (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x),
                );
                let area = 0.5 * (cross.0*cross.0 + cross.1*cross.1 + cross.2*cross.2).sqrt();
                if area < 1e-12 {
                    zero_area_count += 1;
                } else {
                    min_area = min_area.min(area);
                    max_area = max_area.max(area);
                    if area > 0.1 { n_long += 1; }
                }
            }
            println!("\n3D triangle areas: min={:.6}, max={:.6}, zero_area={}, huge(>0.1)={}",
                min_area, max_area, zero_area_count, n_long);

            // Edge lengths
            let (mut min_edge, mut max_edge) = (f64::MAX, 0.0_f64);
            let mut long_edge_count = 0;
            for ti in tri_start..tri_end {
                let tri = inst.mesh.triangles[ti];
                let vs = [
                    inst.mesh.vertices[tri[0] as usize],
                    inst.mesh.vertices[tri[1] as usize],
                    inst.mesh.vertices[tri[2] as usize],
                ];
                for k in 0..3 {
                    let a = vs[k]; let b = vs[(k+1)%3];
                    let len = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt();
                    min_edge = min_edge.min(len);
                    max_edge = max_edge.max(len);
                    if len > 0.5 { long_edge_count += 1; }
                }
            }
            println!("Edge lengths: min={:.6}, max={:.6}, long_edges(>0.5)={}",
                min_edge, max_edge, long_edge_count);

            // Print triangles with long edges (suspicious)
            println!("\nTriangles with long edges (>0.3):");
            let mut count = 0;
            for ti in tri_start..tri_end {
                let tri = inst.mesh.triangles[ti];
                let vs = [
                    inst.mesh.vertices[tri[0] as usize],
                    inst.mesh.vertices[tri[1] as usize],
                    inst.mesh.vertices[tri[2] as usize],
                ];
                let mut max_len = 0.0;
                let mut long_k = 0;
                for k in 0..3 {
                    let a = vs[k]; let b = vs[(k+1)%3];
                    let len = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt();
                    if len > max_len {
                        max_len = len;
                        long_k = k;
                    }
                }
                if max_len > 0.3 {
                    println!("  t{}: v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4}) max_edge={:.4} (v{}-v{})",
                        ti - tri_start,
                        tri[0], vs[0].x, vs[0].y, vs[0].z,
                        tri[1], vs[1].x, vs[1].y, vs[1].z,
                        tri[2], vs[2].x, vs[2].y, vs[2].z,
                        max_len, tri[long_k], tri[(long_k+1)%3]);
                    count += 1;
                    if count >= 20 { break; }
                }
            }
            if count == 0 {
                println!("  (none)");
            }

            // Compute unique vertices in this face
            let mut unique_verts: std::collections::HashSet<u32> = std::collections::HashSet::new();
            for ti in tri_start..tri_end {
                let tri = inst.mesh.triangles[ti];
                unique_verts.insert(tri[0]);
                unique_verts.insert(tri[1]);
                unique_verts.insert(tri[2]);
            }
            println!("\nUnique vertices in face: {}", unique_verts.len());

            // Check for DUPLICATE vertices (same XYZ, different indices) — would indicate a watertightness issue
            let mut pos_to_indices: std::collections::HashMap<(i64,i64,i64), Vec<u32>> = std::collections::HashMap::new();
            for &vi in &unique_verts {
                let v = inst.mesh.vertices[vi as usize];
                let key = (
                    (v.x * 1e6).round() as i64,
                    (v.y * 1e6).round() as i64,
                    (v.z * 1e6).round() as i64,
                );
                pos_to_indices.entry(key).or_default().push(vi);
            }
            let n_dup_positions = pos_to_indices.values().filter(|v| v.len() > 1).count();
            println!("Duplicate positions (same XYZ, different vertex indices): {}", n_dup_positions);
            if n_dup_positions > 0 && n_dup_positions <= 10 {
                for (pos, idxs) in &pos_to_indices {
                    if idxs.len() > 1 {
                        println!("  pos=({:.4},{:.4},{:.4}) indices={:?}", pos.0 as f64 / 1e6, pos.1 as f64 / 1e6, pos.2 as f64 / 1e6, idxs);
                    }
                }
            }
        }
    }
}
