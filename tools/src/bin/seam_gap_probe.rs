//! Diagnostic: measure T-junction seam geometry on as1-oc-214 instances
//! (nut #63, plate #3813). For each long boundary edge, find nearby
//! boundary vertices from OTHER (short) boundary edges, report their
//! distance to the segment — this gives the tolerance needed to snap
//! T-junction seams closed. Then sweep repair_t_junctions tolerances.

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::triangulate::SteinerBudgetProfile;
use draper_mesh::TriangleMesh;
use std::collections::HashMap;

fn edge_counts(mesh: &TriangleMesh) -> HashMap<(u32, u32), usize> {
    let mut m: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        let [a, b, c] = *tri;
        for (v0, v1) in [(a, b), (b, c), (c, a)] {
            let key = (v0.min(v1), v0.max(v1));
            *m.entry(key).or_insert(0) += 1;
        }
    }
    m
}

fn dist_point_seg(p: &[f64; 3], a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let ab = [b[0]-a[0], b[1]-a[1], b[2]-a[2]];
    let ap = [p[0]-a[0], p[1]-a[1], p[2]-a[2]];
    let ab2 = ab[0]*ab[0] + ab[1]*ab[1] + ab[2]*ab[2];
    if ab2 < 1e-20 { return f64::MAX; }
    let t = (ap[0]*ab[0] + ap[1]*ab[1] + ap[2]*ab[2]) / ab2;
    let t = t.clamp(0.0, 1.0);
    let q = [a[0]+t*ab[0], a[1]+t*ab[1], a[2]+t*ab[2]];
    ((p[0]-q[0]).powi(2) + (p[1]-q[1]).powi(2) + (p[2]-q[2]).powi(2)).sqrt()
}

fn main() {
    env_logger::builder().filter_level(log::LevelFilter::Info).init();

    let args: Vec<String> = std::env::args().skip(1).collect();
    let file = args.iter().find(|s| s.ends_with(".stp") || s.ends_with(".step"))
        .cloned().unwrap_or_else(|| "test/as1-oc-214.stp".to_string());
    let targets: Vec<i64> = args.iter()
        .filter(|s| !s.contains('.'))
        .filter_map(|s| s.parse().ok()).collect();

    let content = std::fs::read_to_string(&file).expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    println!("File: {} ({} BREP instances)", file, pending.len());

    let mut ctx = OwnedStepConversionContext::new_with_lod_and_profile(
        step.clone(), 1.0, SteinerBudgetProfile::Desktop,
    );

    for p in &pending {
        let Some(inst) = ctx.triangulate_pending(p) else { continue };
        if !targets.is_empty() && !targets.contains(&p.brep_id) { continue; }
        let mesh = &inst.mesh;

        let ec = edge_counts(mesh);
        let bnd: Vec<(u32, u32)> = ec.iter()
            .filter(|(_, &c)| c == 1).map(|(&(a, b), _)| (a, b)).collect();
        let bnd_verts: std::collections::HashSet<u32> =
            bnd.iter().flat_map(|&(a, b)| [a, b]).collect();

        println!("\n=== BREP #{} ({}) — {} verts, {} tris, {} boundary edges ===",
            p.brep_id, p.name, mesh.vertices.len(), mesh.triangles.len(), bnd.len());

        // Measure: long boundary edges vs nearby boundary vertices.
        let mut stats: Vec<f64> = Vec::new();
        let mut pairs = 0usize;
        for &(a, b) in &bnd {
            let pa = &mesh.vertices[a as usize];
            let pb = &mesh.vertices[b as usize];
            let len = ((pb.x-pa.x).powi(2) + (pb.y-pa.y).powi(2) + (pb.z-pa.z).powi(2)).sqrt();
            if len < 0.2 { continue; } // only long edges
            for &v in &bnd_verts {
                if v == a || v == b { continue; }
                let pv = &mesh.vertices[v as usize];
                let a3 = [pa.x, pa.y, pa.z];
                let b3 = [pb.x, pb.y, pb.z];
                let p3 = [pv.x, pv.y, pv.z];
                let d = dist_point_seg(&p3, &a3, &b3);
                if d < 0.5 {
                    pairs += 1;
                    stats.push(d);
                }
            }
        }
        if !stats.is_empty() {
            stats.sort_by(|x, y| x.partial_cmp(y).unwrap());
            let n = stats.len();
            let i90 = ((n as f64) * 0.9) as usize % n;
            println!("  long-edge near-vertices: {} hits, dist min={:.3e} p50={:.3e} p90={:.3e} max={:.3e}",
                pairs, stats[0], stats[n/2], stats[i90], stats[n-1]);
        } else {
            println!("  long-edge near-vertices: none (genuine holes, not T-junctions)");
        }

        // Sweep repair_t_junctions tolerances on fresh clones.
        for tol in [1e-8, 1e-6] {
            let mut m = mesh.clone();
            let before = edge_counts(&m);
            let b0 = before.values().filter(|&&c| c == 1).count();
            let nm0 = before.values().filter(|&&c| c > 2).count();
            let n = draper_mesh::repair_t_junctions(&mut m, tol);
            let after = edge_counts(&m);
            let b1 = after.values().filter(|&&c| c == 1).count();
            let nm1 = after.values().filter(|&&c| c > 2).count();
            println!("  tj tol={:.0e}: splits={} | bnd {}->{} | nonmanifold {}->{} | tris {}->{}",
                tol, n, b0, b1, nm0, nm1, mesh.triangles.len(), m.triangles.len());
        }
    }
}
