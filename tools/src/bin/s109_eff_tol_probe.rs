//! s109 diagnostic: exact per-BREP eff_tol (full precision — the census
//! prints only 4 decimals, which flattened 2.26e-5 to "0.0000" and let a
//! wrong "2.49e-8" micro-tolerance claim into the s107 worklog), plus the
//! full vertex geometry of every >170 deg same-face pair on the target
//! BREP and the triangle ranges of the involved faces.
//! Read-only: mutates nothing. Conversion path replicates fold_face_probe
//! exactly (StepConversionContext::new + triangulate_pending per pending).

use draper_mesh::check_manifold;
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env()
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "test/brick_thin_round.stp".to_string());
    let target: i64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1498);

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        let Some(inst) = ctx.triangulate_pending(p) else {
            continue;
        };
        let eff_tol = ctx.brep_eff_tol(p.brep_id);
        let mesh = &inst.mesh;
        let report = check_manifold(mesh);
        println!(
            "BREP #{} ({}) V={} T={} bnd={} nm={} eff_tol={:?} ({:e})",
            p.brep_id,
            p.name,
            mesh.vertex_count(),
            mesh.triangle_count(),
            report.boundary_edge_count,
            report.non_manifold_edge_count,
            eff_tol,
            eff_tol.unwrap_or(f64::NAN),
        );
        if p.brep_id != target {
            continue;
        }

        // Face triangle ranges (for the double-attribution picture).
        println!("--- face ranges ---");
        let mut by_fid: Vec<_> = inst.faces.iter().cloned().collect();
        by_fid.sort_by_key(|f| f.triangle_range.0);
        for f in &by_fid {
            println!(
                "face {} {} range={:?} step_faces={:?}",
                f.face_id, f.surface_type, f.triangle_range, f.step_face_id
            );
        }

        // Same-face >170 deg pairs, full geometry.
        let Some(fids) = mesh.triangle_face_ids.as_ref() else {
            println!("no face ids");
            continue;
        };
        let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let [a, b, c] = *tri;
            for (v0, v1) in [(a, b), (b, c), (c, a)] {
                let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                edge_to_tris.entry(key).or_default().push(ti);
            }
        }
        let vpos = |i: u32| {
            let v = &mesh.vertices[i as usize];
            format!("({:+.6e},{:+.6e},{:+.6e})", v.x, v.y, v.z)
        };
        let mut n_pairs = 0usize;
        let mut keys: Vec<_> = edge_to_tris.keys().cloned().collect();
        keys.sort();
        for key in &keys {
            let tris = &edge_to_tris[&key];
            if tris.len() != 2 {
                continue;
            }
            let (t0, t1) = (tris[0], tris[1]);
            if fids[t0] != fids[t1] {
                continue;
            }
            // normals
            let nrm = |ti: usize| {
                let [a, b, c] = mesh.triangles[ti];
                let (va, vb, vc) = (
                    &mesh.vertices[a as usize],
                    &mesh.vertices[b as usize],
                    &mesh.vertices[c as usize],
                );
                let e1 = [vb.x - va.x, vb.y - va.y, vb.z - va.z];
                let e2 = [vc.x - va.x, vc.y - va.y, vc.z - va.z];
                let cr = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ];
                let l = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
                (cr, l)
            };
            let (n0, l0) = nrm(t0);
            let (n1, l1) = nrm(t1);
            if l0 < 1e-30 || l1 < 1e-30 {
                continue;
            }
            let dot = (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]) / (l0 * l1);
            let ang = dot.clamp(-1.0, 1.0).acos().to_degrees();
            if ang <= 170.0 {
                continue;
            }
            n_pairs += 1;
            println!(
                "--- PAIR {} face=({},{}) ang={:.4} tris=({},{}) edge=({},{})",
                n_pairs, fids[t0], fids[t1], ang, t0, t1, key.0, key.1
            );
            for ti in [t0, t1] {
                let [a, b, c] = mesh.triangles[ti];
                println!(
                    "  tri[{}] = [{} {} {}]  {} {} {}",
                    ti,
                    a,
                    b,
                    c,
                    vpos(a),
                    vpos(b),
                    vpos(c)
                );
            }
        }
        println!("--- total same-face >170 pairs: {}", n_pairs);

        // s109 item-2: chi decomposition (COLLECTOR #2860 documentation).
        let v = mesh.vertex_count();
        let t = mesh.triangle_count();
        let mut edge_count = 0usize;
        let mut boundary: Vec<(u32, u32)> = Vec::new();
        for key in &keys {
            let n = edge_to_tris[key].len();
            if n >= 1 {
                edge_count += 1;
            }
            if n == 1 {
                boundary.push(*key);
            }
        }
        // Boundary loops: directed hole-walk half-edges. For each boundary
        // edge {a,b} traversed a->b by its single triangle, the hole rim
        // walks b->a. Greedy edge-disjoint closed walks = boundary cycles
        // (pinch vertices may merge rims sharing a vertex — acceptable for
        // the chi documentation).
        use std::collections::BTreeSet;
        let mut dir_bnd: BTreeSet<(u32, u32)> = BTreeSet::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let [a, b, c] = *tri;
            for (v0, v1) in [(a, b), (b, c), (c, a)] {
                let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                if edge_to_tris[&key].len() == 1 && edge_to_tris[&key][0] == ti {
                    dir_bnd.insert((v1, v0));
                }
            }
        }
        let mut n_loops = 0usize;
        let mut open_chains = 0usize;
        let mut loop_sizes: Vec<usize> = Vec::new();
        while let Some(&e0) = dir_bnd.iter().next() {
            dir_bnd.remove(&e0);
            let (a, b) = e0;
            let mut size = 1usize;
            let mut cur = b;
            let mut closed = false;
            let mut guard = 0usize;
            while guard < 1_000_000 {
                guard += 1;
                let mut found = None;
                for &e in dir_bnd.iter() {
                    if e.0 == cur {
                        found = Some(e);
                        break;
                    }
                }
                match found {
                    Some(e) => {
                        dir_bnd.remove(&e);
                        size += 1;
                        cur = e.1;
                        if cur == a {
                            closed = true;
                            break;
                        }
                    }
                    None => break,
                }
            }
            if closed {
                n_loops += 1;
                loop_sizes.push(size);
            } else {
                open_chains += 1;
            }
        }
        loop_sizes.sort();
        println!(
            "--- chi decomposition: V={} T={} E={} chi={} bnd_edges={} bnd_cycles={} open_chains={} cycle_sizes={:?}",
            v,
            t,
            edge_count,
            v as i64 - edge_count as i64 + t as i64,
            boundary.len(),
            n_loops,
            open_chains,
            loop_sizes
        );
        // Connected components (vertex-connectivity via any edge).
        let mut parent: Vec<u32> = (0..v as u32).collect();
        fn find(p: &mut Vec<u32>, i: u32) -> u32 {
            if p[i as usize] == i {
                i
            } else {
                let r = find(p, p[i as usize]);
                p[i as usize] = r;
                r
            }
        }
        for key in &keys {
            let (a, b) = *key;
            if edge_to_tris[key].is_empty() {
                continue;
            }
            let ra = find(&mut parent, a);
            let rb = find(&mut parent, b);
            if ra != rb {
                parent[ra as usize] = rb;
            }
        }
        let mut comps: std::collections::BTreeSet<u32> = std::collections::BTreeSet::new();
        for ti in mesh.triangles.iter() {
            for &vv in ti {
                comps.insert(find(&mut parent, vv));
            }
        }
        println!("--- connected components: {}", comps.len());
    }
}
