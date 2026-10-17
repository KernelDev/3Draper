//! session-100 diagnostic: census the vertex/edge neighborhoods of the
//! born-folded cross-face pairs, to decide the seam-split clone scope.
//! For each queried vertex: list ALL triangles using it (tri idx, face id,
//! surface type). For each queried edge (v0,v1): list ALL triangles
//! containing both verts (manifold census) + their winding relationship.
//! Usage: s100_edge_census <file.stp> <brep_id> v<v1>[,v2,...] e<v0>:<v1>[,...]
//!   e.g. s100_edge_census test/drill_top.stp 47598 v543,544,551,542 e543:544
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "test/drill_top.stp".into());
    let brep_id: i64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(47598);
    let mut verts: Vec<u32> = Vec::new();
    let mut edges: Vec<(u32, u32)> = Vec::new();
    for a in &args[3..] {
        if let Some(rest) = a.strip_prefix('v') {
            for p in rest.split(',') {
                if let Ok(v) = p.parse::<u32>() {
                    verts.push(v);
                }
            }
        } else if let Some(rest) = a.strip_prefix('e') {
            for p in rest.split(',') {
                let ab: Vec<&str> = p.split(':').collect();
                if ab.len() == 2 {
                    if let (Ok(a), Ok(b)) = (ab[0].parse::<u32>(), ab[1].parse::<u32>()) {
                        edges.push((a, b));
                    }
                }
            }
        }
    }

    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    for p in &pending {
        if p.brep_id != brep_id {
            continue;
        }
        let ctx = StepConversionContext::new(&step);
        let Some(inst) = ctx.triangulate_pending(p) else {
            println!("BREP #{}: triangulate_pending None", brep_id);
            continue;
        };
        println!(
            "=== BREP #{} '{}' faces={} tris={} ===",
            brep_id,
            inst.name,
            inst.faces.len(),
            inst.mesh.triangles.len()
        );
        let mesh = &inst.mesh;
        let fids = mesh.triangle_face_ids.as_ref();
        let face_by_fid: std::collections::HashMap<u64, &draper_step::FaceInfo> =
            inst.faces.iter().map(|f| (f.face_id, f)).collect();
        let fkind = |ti: usize| -> (u64, String) {
            let fid = fids
                .and_then(|f| f.get(ti).copied())
                .unwrap_or(u64::MAX);
            let k = face_by_fid
                .get(&fid)
                .map(|f| f.surface_type.clone())
                .unwrap_or_else(|| "?".into());
            (fid, k)
        };
        let p3 = |vi: u32| -> (f64, f64, f64) {
            let v = &mesh.vertices[vi as usize];
            (v.x, v.y, v.z)
        };

        // vertex neighborhoods
        for &q in &verts {
            println!("\n---- VERTEX v{} at ({:.5},{:.5},{:.5}) ----", q, {
                let p = p3(q);
                p.0
            }, {
                let p = p3(q);
                p.1
            }, {
                let p = p3(q);
                p.2
            });
            let mut users: Vec<usize> = Vec::new();
            for (ti, tri) in mesh.triangles.iter().enumerate() {
                if tri.contains(&q) {
                    users.push(ti);
                }
            }
            println!("  used by {} tris:", users.len());
            for ti in users {
                let (fid, k) = fkind(ti);
                let tri = mesh.triangles[ti];
                println!(
                    "    T{} fid={} {} verts=({},{},{})",
                    ti, fid, k, tri[0], tri[1], tri[2]
                );
            }
        }

        // edge census
        for &(a, b) in &edges {
            let pa = p3(a);
            let pb = p3(b);
            let d = ((pa.0 - pb.0).powi(2) + (pa.1 - pb.1).powi(2) + (pa.2 - pb.2).powi(2)).sqrt();
            println!(
                "\n---- EDGE v{}:v{} len={:.5} ----",
                a, b, d
            );
            let mut adj: Vec<usize> = Vec::new();
            for (ti, tri) in mesh.triangles.iter().enumerate() {
                if tri.contains(&a) && tri.contains(&b) {
                    adj.push(ti);
                }
            }
            println!("  adjacent tris: {}", adj.len());
            // normal of each adjacent tri + traversal of the edge
            let mut info: Vec<(usize, f64, [f64; 3], u32, u32, bool)> = Vec::new();
            for &ti in &adj {
                let tri = mesh.triangles[ti];
                let (va, vb, vc) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
                let p0 = &mesh.vertices[va];
                let p1 = &mesh.vertices[vb];
                let p2 = &mesh.vertices[vc];
                let e1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
                let e2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];
                let n = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ];
                let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                // traversal: find the directed occurrence of (a,b) or (b,a)
                let pairs = [
                    (tri[0], tri[1]),
                    (tri[1], tri[2]),
                    (tri[2], tri[0]),
                ];
                let mut dir_ab = false;
                let mut found = false;
                for (x, y) in pairs {
                    if x == a && y == b {
                        dir_ab = true;
                        found = true;
                    } else if x == b && y == a {
                        dir_ab = false;
                        found = true;
                    }
                }
                let _ = found;
                info.push((ti, nl, n, tri[0], tri[1], dir_ab));
            }
            for (i, &(ti, nl, n, _, _, dir_ab)) in info.iter().enumerate() {
                let (fid, k) = fkind(ti);
                println!(
                    "    T{} fid={} {} |n|={:.2e} n=({:.3},{:.3},{:.3}) traverses {}{}",
                    ti,
                    fid,
                    k,
                    nl,
                    n[0] / nl.max(1e-30),
                    n[1] / nl.max(1e-30),
                    n[2] / nl.max(1e-30),
                    if dir_ab { "a->b" } else { "b->a" },
                    if nl < 1e-12 { " DEGENERATE" } else { "" }
                );
                // pairwise angles vs later entries
                for &(tj, _, n2, _, _, dir2) in info.iter().skip(i + 1) {
                    let nl2 = (n2[0] * n2[0] + n2[1] * n2[1] + n2[2] * n2[2]).sqrt();
                    if nl < 1e-12 || nl2 < 1e-12 {
                        continue;
                    }
                    let dot = (n[0] * n2[0] + n[1] * n2[1] + n[2] * n2[2]) / (nl * nl2);
                    let ang = dot.clamp(-1.0, 1.0).acos().to_degrees();
                    println!(
                    "      pair T{}/T{}: normal-angle={:.2}deg traversal={}",
                    ti,
                    tj,
                    ang,
                    if dir_ab == dir2 { "SAME" } else { "OPP" }
                );
                }
            }
        }
        return;
    }
    println!("brep {} not found", brep_id);
}
