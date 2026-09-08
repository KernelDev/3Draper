//! Diagnostic: for a given STEP file + LOD, dump per-NURBS-face ACTUAL mesh
//! boundary loop lengths (segments around circles in the RENDERED mesh),
//! unique vertex counts, and chord-deviation estimates — to pinpoint where
//! the dense 185-point edge discretization gets dropped.

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::triangulate::SteinerBudgetProfile;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/as1-oc-214.stp");
    let lod: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0.75);
    let target_brep: i64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(759);

    let content = std::fs::read_to_string(path).expect("read step file");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);

    let mut ctx = OwnedStepConversionContext::new_with_lod_and_profile(
        step.clone(),
        lod,
        SteinerBudgetProfile::Desktop,
    );

    for p in &pending {
        let Some(inst) = ctx.triangulate_pending(p) else { continue };
        if p.brep_id != target_brep { continue; }
        println!("=== BREP #{} ({}) — LOD {} — V={} T={} ===",
            p.brep_id, p.name, lod, inst.mesh.vertex_count(), inst.mesh.triangle_count());

        for f in &inst.faces {
            let (t0, t1) = f.triangle_range;
            let tris = t1 - t0;
            if tris == 0 { continue; }

            // Unique vertices of this face's sub-mesh
            let mut used: std::collections::HashSet<u32> = std::collections::HashSet::new();
            // Edge → usage count within the face
            let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
            for i in t0..t1 {
                let tri = inst.mesh.triangles[i];
                used.insert(tri[0]); used.insert(tri[1]); used.insert(tri[2]);
                let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                for (a, b) in edges {
                    let key = if a < b { (a, b) } else { (b, a) };
                    *edge_count.entry(key).or_insert(0) += 1;
                }
            }

            // Boundary loops of the face sub-mesh (edges used exactly once)
            let boundary_edges: Vec<(u32, u32)> = edge_count
                .iter()
                .filter(|(_, &c)| c == 1)
                .map(|(&(a, b), _)| if a < b { (a, b) } else { (b, a) })
                .collect();
            // Chain boundary edges into loops
            let mut loops: Vec<usize> = Vec::new();
            let mut adj: HashMap<u32, Vec<u32>> = HashMap::new();
            for (a, b) in &boundary_edges {
                adj.entry(*a).or_default().push(*b);
                adj.entry(*b).or_default().push(*a);
            }
            let mut visited: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
            for (a, b) in &boundary_edges {
                for (s, e) in [(*a, *b), (*b, *a)] {
                    if visited.contains(&(s, e)) { continue; }
                    // walk loop
                    let mut len = 0usize;
                    let mut prev = s;
                    let mut cur = e;
                    visited.insert((s, e));
                    loop {
                        len += 1;
                        let nexts = adj.get(&cur).map(|v| v.as_slice()).unwrap_or(&[]);
                        let mut stepped = false;
                        for &n in nexts {
                            if n == prev { continue; }
                            if visited.contains(&(cur, n)) { continue; }
                            visited.insert((cur, n));
                            prev = cur;
                            cur = n;
                            stepped = true;
                            break;
                        }
                        if !stepped || cur == s { break; }
                    }
                    loops.push(len);
                }
            }
            loops.sort_unstable();
            loops.dedup();

            // min/max as arrays of f64
            let mut min = [f64::MAX; 3];
            let mut max = [f64::MIN; 3];
            for &vi in used.iter() {
                let v = inst.mesh.vertices[vi as usize];
                let c = [v.x, v.y, v.z];
                for k in 0..3 {
                    min[k] = min[k].min(c[k]);
                    max[k] = max[k].max(c[k]);
                }
            }

            println!(
                "  face #{:<6} {:<8} tris={:<5} uniqV={:<5} bnd_edges={:<5} loops={:?}  bbox=[{:.1},{:.1},{:.1}]..[{:.1},{:.1},{:.1}]",
                f.step_face_id, f.surface_type, tris, used.len(), boundary_edges.len(), loops,
                min[0], min[1], min[2], max[0], max[1], max[2]
            );

            // Polyline boundary (what the edge cache provided)
            let poly: Vec<usize> = f.outer_boundary.iter().map(|l| l.len())
                .chain(f.inner_boundaries.iter().map(|l| l.len()))
                .collect();
            println!("      edge-cache polyline loop sizes: {:?}", poly);
        }
    }
}
