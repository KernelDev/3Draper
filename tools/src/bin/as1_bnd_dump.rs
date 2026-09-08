//! Dump boundary edge positions of a BREP instance mesh to find where the
//! open edges are (which feature: hole circles, seams, flats).

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::triangulate::SteinerBudgetProfile;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let lod: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(0.75);
    let target: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(63);

    let content = std::fs::read_to_string("test/as1-oc-214.stp").expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let mut ctx = OwnedStepConversionContext::new_with_lod_and_profile(
        step.clone(),
        lod,
        SteinerBudgetProfile::Desktop,
    );
    for p in &pending {
        let Some(inst) = ctx.triangulate_pending(p) else { continue };
        if p.brep_id != target { continue; }
        let mesh = &inst.mesh;
        let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &mesh.triangles {
            let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
            for (a, b) in edges {
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }
        let bnd: Vec<(u32, u32)> = edge_count.iter()
            .filter(|(_, &c)| c == 1)
            .map(|(&(a, b), _)| (a, b))
            .collect();
        println!("=== BREP #{} ({}) V={} T={} boundary_edges={} ===",
            p.brep_id, p.name, mesh.vertex_count(), mesh.triangle_count(), bnd.len());
        // Cluster by face ranges
        for f in &inst.faces {
            let (t0, t1) = f.triangle_range;
            let tri_ids: std::collections::HashSet<usize> = (t0..t1).collect();
            let mut face_bnd = 0usize;
            for &(a, b) in &bnd {
                // count if any adjacent triangle belongs to this face
                let mut ok = false;
                for tri in &mesh.triangles {
                    let has_a = tri[0] == a || tri[1] == a || tri[2] == a;
                    let has_b = tri[0] == b || tri[1] == b || tri[2] == b;
                    if has_a && has_b {
                        let fi = mesh.triangles.iter().position(|t| *t == *tri).unwrap_or(0);
                        let _ = fi;
                        ok = true;
                        break;
                    }
                }
                let _ = ok;
            }
            let _ = &tri_ids;
            let _ = &mut face_bnd;
        }
        // Simpler: print first 40 boundary edges with positions
        for (n, &(a, b)) in bnd.iter().take(40).enumerate() {
            let pa = mesh.vertices[a as usize];
            let pb = mesh.vertices[b as usize];
            let mid = [(pa.x+pb.x)/2.0, (pa.y+pb.y)/2.0, (pa.z+pb.z)/2.0];
            let len = ((pa.x-pb.x).powi(2)+(pa.y-pb.y).powi(2)+(pa.z-pb.z).powi(2)).sqrt();
            println!("  bnd[{:2}] ({:7.3},{:7.3},{:7.3})-({:7.3},{:7.3},{:7.3}) len={:6.3} mid=({:7.2},{:7.2},{:7.2})",
                n, pa.x, pa.y, pa.z, pb.x, pb.y, pb.z, len, mid[0], mid[1], mid[2]);
        }
        // length stats
        let mut lens: Vec<f64> = bnd.iter().map(|&(a, b)| {
            let pa = mesh.vertices[a as usize];
            let pb = mesh.vertices[b as usize];
            ((pa.x-pb.x).powi(2)+(pa.y-pb.y).powi(2)+(pa.z-pb.z).powi(2)).sqrt()
        }).collect();
        lens.sort_by(|x, y| x.partial_cmp(y).unwrap());
        if !lens.is_empty() {
            println!("  lengths: min={:.4} p25={:.4} med={:.4} p75={:.4} max={:.4}",
                lens[0], lens[lens.len()/4], lens[lens.len()/2], lens[3*lens.len()/4], lens[lens.len()-1]);
        }
        break;
    }
}
