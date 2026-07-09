// Quick diagnostic: brick_thin.stp — why 4724 boundary edges?
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/brick_thin.stp")
        .expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            println!("=== BREP #{} ({}) ===", p.brep_id, p.name);
            println!("  v={} t={} watertight={} boundary={}",
                inst.mesh.vertex_count(), inst.mesh.triangle_count(),
                report.is_watertight(), report.boundary_edge_count);

            // Count faces by surface type
            let mut surf_counts: HashMap<String, usize> = HashMap::new();
            for fi in &inst.faces {
                let st = fi.surface_type.clone();
                *surf_counts.entry(st).or_insert(0) += 1;
            }
            for (st, count) in &surf_counts {
                println!("  {}: {} faces", st, count);
            }

            // Find boundary edges and their face IDs
            let fids = inst.mesh.triangle_face_ids.as_ref();
            let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
            for tri in &inst.mesh.triangles {
                let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                for (a, b) in edges {
                    let key = if a < b { (a, b) } else { (b, a) };
                    *edge_count.entry(key).or_insert(0) += 1;
                }
            }

            // Count boundary edges by face
            let mut face_boundary: HashMap<u64, usize> = HashMap::new();
            for (edge, &count) in &edge_count {
                if count == 1 {
                    for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                        let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                        for (a, b) in edges {
                            let key = if a < b { (a, b) } else { (b, a) };
                            if key == *edge {
                                if let Some(fid) = fids.and_then(|f| f.get(ti).copied()) {
                                    *face_boundary.entry(fid).or_insert(0) += 1;
                                }
                                break;
                            }
                        }
                    }
                }
            }

            let mut sorted: Vec<_> = face_boundary.into_iter().collect();
            sorted.sort_by(|a, b| b.1.cmp(&a.1));
            println!("\n  Top 10 faces by boundary edges:");
            for (fid, count) in sorted.iter().take(10) {
                let fi = inst.faces.iter().find(|f| f.face_id == *fid);
                if let Some(fi) = fi {
                    println!("    face_id={} (Step#{}, {}): {} boundary edges, forward={}",
                        fid, fi.step_face_id, fi.surface_type, count, fi.forward);
                }
            }

            // Check boundary edge lengths
            let mut lengths: Vec<f64> = Vec::new();
            for (edge, &count) in &edge_count {
                if count == 1 {
                    let v0 = inst.mesh.vertices[edge.0 as usize];
                    let v1 = inst.mesh.vertices[edge.1 as usize];
                    let d = ((v0.x-v1.x).powi(2) + (v0.y-v1.y).powi(2) + (v0.z-v1.z).powi(2)).sqrt();
                    lengths.push(d);
                }
            }
            lengths.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let n = lengths.len();
            if n > 0 {
                println!("\n  Boundary edge lengths ({} edges):", n);
                println!("    min={:.6} med={:.6} max={:.6}", lengths[0], lengths[n/2], lengths[n-1]);
                let fp = lengths.iter().filter(|&&l| l < 1e-6).count();
                let small = lengths.iter().filter(|&&l| l >= 1e-6 && l < 0.1).count();
                let med = lengths.iter().filter(|&&l| l >= 0.1 && l < 1.0).count();
                let large = lengths.iter().filter(|&&l| l >= 1.0).count();
                println!("    FP(<1e-6):{} small(<0.1):{} med(<1):{} large(>=1):{}", fp, small, med, large);
            }
            break;
        }
    }
}
