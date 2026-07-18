// Check if the edge cache produces identical points for the same step_id
// when called from different faces (Plane vs NURBS)
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::{Point3d, Surface};
use std::collections::HashMap;

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
            // For each face, collect the boundary vertices and their step_face_id
            let fids = inst.mesh.triangle_face_ids.as_ref();

            // For each unique vertex, find which faces use it
            let mut vertex_faces: HashMap<u32, std::collections::HashSet<u64>> = HashMap::new();
            for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                let fid = fids.and_then(|f| f.get(ti).copied()).unwrap_or(u64::MAX);
                for &vi in tri {
                    vertex_faces.entry(vi).or_default().insert(fid);
                }
            }

            // Find vertices used by exactly 1 face (boundary vertices)
            let mut boundary_by_face: HashMap<u64, Vec<(u32, Point3d)>> = HashMap::new();
            for (vi, faces) in &vertex_faces {
                if faces.len() == 1 {
                    let fid = faces.iter().next().unwrap();
                    let v = inst.mesh.vertices[*vi as usize];
                    boundary_by_face.entry(*fid).or_default().push((*vi, v));
                }
            }

            println!("\n=== BREP #{} ({}) ===", p.brep_id, p.name);
            for (fid, verts) in boundary_by_face.iter() {
                let fi = inst.faces.iter().find(|f| f.face_id == *fid);
                if let Some(fi) = fi {
                    println!("  Face Step#{} {} (face_id={}): {} boundary vertices",
                        fi.step_face_id, fi.surface_type, fid, verts.len());
                    // Show first 3 boundary vertices
                    for (vi, v) in verts.iter().take(3) {
                        println!("    vi={}: ({:.6}, {:.6}, {:.6})", vi, v.x, v.y, v.z);
                    }
                }
            }

            // Now check: for vertices at the same 3D position but different indices,
            // are they from different faces?
            let mut pos_to_verts: HashMap<[u64; 3], Vec<(u32, u64)>> = HashMap::new();
            for (vi, v) in inst.mesh.vertices.iter().enumerate() {
                let key = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
                let fid = fids.and_then(|f| {
                    for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                        if tri[0] == vi as u32 || tri[1] == vi as u32 || tri[2] == vi as u32 {
                            return f.get(ti).copied();
                        }
                    }
                    None
                }).unwrap_or(u64::MAX);
                pos_to_verts.entry(key).or_default().push((vi as u32, fid));
            }

            // Find positions with multiple vertices from different faces
            let mut multi_face_positions = 0;
            let mut near_match_positions = 0;
            for (_key, verts) in &pos_to_verts {
                let faces: std::collections::HashSet<u64> = verts.iter().map(|(_, f)| *f).collect();
                if faces.len() > 1 {
                    multi_face_positions += 1;
                }
            }

            // Also check near-matches (within merge_tol)
            let merge_tol = 0.06f64; // approximate merge_tol
            let mut near_matches = 0;
            let mut exact_matches = 0;
            let verts: Vec<(u32, Point3d, u64)> = inst.mesh.vertices.iter().enumerate().map(|(vi, v)| {
                let fid = fids.and_then(|f| {
                    for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                        if tri[0] == vi as u32 || tri[1] == vi as u32 || tri[2] == vi as u32 {
                            return f.get(ti).copied();
                        }
                    }
                    None
                }).unwrap_or(u64::MAX);
                (vi as u32, *v, fid)
            }).collect();

            for i in 0..verts.len() {
                for j in (i+1)..verts.len() {
                    let (vi, pi, fi) = &verts[i];
                    let (vj, pj, fj) = &verts[j];
                    if fi == fj { continue; } // same face
                    let d = ((pi.x-pj.x).powi(2) + (pi.y-pj.y).powi(2) + (pi.z-pj.z).powi(2)).sqrt();
                    if d < 1e-13 {
                        exact_matches += 1;
                    } else if d < merge_tol {
                        near_matches += 1;
                    }
                }
            }

            println!("\n  Multi-face positions (exact bit match): {}", multi_face_positions);
            println!("  Exact cross-face vertex matches: {}", exact_matches);
            println!("  Near cross-face vertex matches (<{:.3}mm): {}", merge_tol, near_matches);
            break;
        }
    }
}
