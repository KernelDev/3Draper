// Find the 13 boundary edges in bolt and check which faces they belong to
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
            let fids = inst.mesh.triangle_face_ids.as_ref().unwrap();

            // Build edge → triangle count map
            let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
            let mut edge_tris: HashMap<(u32, u32), Vec<(usize, u64)>> = HashMap::new();
            for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                let fid = fids.get(ti).copied().unwrap_or(u64::MAX);
                let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                for (a, b) in edges {
                    let key = if a < b { (a, b) } else { (b, a) };
                    *edge_count.entry(key).or_insert(0) += 1;
                    edge_tris.entry(key).or_default().push((ti, fid));
                }
            }

            // Find boundary edges (count == 1)
            let mut boundary_info: Vec<(u32, u32, [f64; 3], [f64; 3], u64)> = Vec::new();
            for (edge, &count) in &edge_count {
                if count == 1 {
                    let v0 = inst.mesh.vertices[edge.0 as usize];
                    let v1 = inst.mesh.vertices[edge.1 as usize];
                    let fid = edge_tris.get(edge).and_then(|v| v.first().map(|(_, f)| *f)).unwrap_or(u64::MAX);
                    boundary_info.push((edge.0, edge.1, [v0.x, v0.y, v0.z], [v1.x, v1.y, v1.z], fid));
                }
            }

            println!("=== BREP #{} ({}) ===", p.brep_id, p.name);
            println!("  {} boundary edges", boundary_info.len());

            // Group by face
            let mut by_face: HashMap<u64, Vec<_>> = HashMap::new();
            for info in &boundary_info {
                by_face.entry(info.4).or_default().push(info);
            }
            for (fid, edges) in by_face.iter() {
                let fi = inst.faces.iter().find(|f| f.face_id == *fid);
                let step_id = fi.map(|f| f.step_face_id).unwrap_or(0);
                let surf = fi.map(|f| f.surface_type.clone()).unwrap_or_default();
                println!("  Face {} (Step#{}, {}): {} boundary edges", fid, step_id, surf, edges.len());
                for e in edges.iter().take(5) {
                    let d = ((e.2[0]-e.3[0]).powi(2) + (e.2[1]-e.3[1]).powi(2) + (e.2[2]-e.3[2]).powi(2)).sqrt();
                    println!("    v{}→v{} len={:.4} pos=({:.2},{:.2},{:.2})→({:.2},{:.2},{:.2})",
                        e.0, e.1, d, e.2[0], e.2[1], e.2[2], e.3[0], e.3[1], e.3[2]);
                }
            }

            break;
        }
    }
}
