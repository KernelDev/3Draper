//! session-41 diagnostic: replicate fix_inconsistent_winding Step-1 pair
//! scan (same-face, shared edge usage 2, normals >170°) on a converted
//! BREP mesh and dump the full geometry of every candidate pair — winding
//! consistency, apex same-side (genuine overlap) vs opposite-side
//! (tiling), areas, coords. Read-only: mutates nothing.

use draper_mesh::{check_manifold, TriangulationParams};
use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "test/as1-oc-214.stp".to_string());
    let target: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(63);

    let content = std::fs::read_to_string(&path).expect("read");
    let step_file = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step_file);

    let mut params = TriangulationParams::default();
    params.use_surface_canonical_cdt = true;

    let mut ctx = OwnedStepConversionContext::new_with_params(step_file, params);
    for p in &pending {
        let Some(inst) = ctx.triangulate_pending(p) else {
            continue;
        };
        if p.brep_id != target {
            continue;
        }
        let mesh = &inst.mesh;
        let report = check_manifold(mesh);
        println!(
            "=== BREP #{} ({}) V={} T={} bnd={} nm={} ===",
            p.brep_id,
            p.name,
            mesh.vertex_count(),
            mesh.triangle_count(),
            report.boundary_edge_count,
            report.non_manifold_edge_count
        );

        let face_ids = mesh.triangle_face_ids.as_ref().expect("face ids");
        let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let [a, b, c] = *tri;
            for (v0, v1) in [(a, b), (b, c), (c, a)] {
                let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                edge_to_tris.entry(key).or_default().push(ti);
            }
        }

        // fid -> step_face_id via face_infos ranges (best effort)
        let fid_to_step: HashMap<u64, i64> = inst
            .faces
            .iter()
            .map(|f| (f.face_id, f.step_face_id))
            .collect();

        let mut keys: Vec<_> = edge_to_tris.keys().copied().collect();
        keys.sort_unstable();
        let mut n_pairs = 0usize;
        for edge in keys {
            let tris = &edge_to_tris[&edge];
            if tris.len() != 2 {
                continue;
            }
            let fid0 = face_ids.get(tris[0]).copied().unwrap_or(0);
            let fid1 = face_ids.get(tris[1]).copied().unwrap_or(0);
            if fid0 != fid1 || fid0 == 0 {
                continue;
            }
            let t0 = mesh.triangles[tris[0]];
            let t1 = mesh.triangles[tris[1]];
            let n0 = tri_normal(mesh, t0);
            let n1 = tri_normal(mesh, t1);
            let (Some(n0), Some(n1)) = (n0, n1) else {
                continue;
            };
            let dot = n0.0 * n1.0 + n0.1 * n1.1 + n0.2 * n1.2;
            let l0 = n0.0.hypot(n0.1.hypot(n0.2));
            let l1 = n1.0.hypot(n1.1.hypot(n1.2));
            let cos = (dot / (l0 * l1)).clamp(-1.0, 1.0);
            let angle = cos.acos().to_degrees();
            if angle <= 170.0 {
                continue;
            }
            n_pairs += 1;

            // Winding consistency: both triangles traverse the shared
            // edge — consistent = opposite directions.
            let dir0 = edge_dir(t0, edge);
            let dir1 = edge_dir(t1, edge);
            let winding = if dir0 != dir1 {
                "CONSISTENT"
            } else {
                "INCONSISTENT"
            };

            // Apex same-side test: project both apexes onto the plane
            // through edge midpoint perpendicular to the edge; if the
            // apexes are on the same side -> genuine geometric overlap.
            let pa = mesh.vertices[edge.0 as usize];
            let pb = mesh.vertices[edge.1 as usize];
            let apex0 = apex_of(t0, edge);
            let apex1 = apex_of(t1, edge);
            let same_side = {
                let e = (pb.x - pa.x, pb.y - pa.y, pb.z - pa.z);
                // Build a plane containing the edge and apex0's normal side:
                // use the normal of t0 as the plane normal reference.
                let s0 = e.0 * (mesh.vertices[apex0 as usize].x - pa.x)
                    + e.1 * (mesh.vertices[apex0 as usize].y - pa.y)
                    + e.2 * (mesh.vertices[apex0 as usize].z - pa.z);
                let s1 = e.0 * (mesh.vertices[apex1 as usize].x - pa.x)
                    + e.1 * (mesh.vertices[apex1 as usize].y - pa.y)
                    + e.2 * (mesh.vertices[apex1 as usize].z - pa.z);
                // component along the edge is the same for both by
                // construction; compare the in-plane side via cross with e:
                let side = |vi: u32| -> (f64, f64, f64) {
                    let v = mesh.vertices[vi as usize];
                    let d = (v.x - pa.x, v.y - pa.y, v.z - pa.z);
                    // d × e (component along t0 normal-ish)
                    (
                        d.1 * e.2 - d.2 * e.1,
                        d.2 * e.0 - d.0 * e.2,
                        d.0 * e.1 - d.1 * e.0,
                    )
                };
                let sd0 = side(apex0);
                let sd1 = side(apex1);
                let dot_side = sd0.0 * sd1.0 + sd0.1 * sd1.1 + sd0.2 * sd1.2;
                let _ = (s0, s1);
                dot_side > 0.0
            };

            let a0 = tri_area(mesh, t0);
            let a1 = tri_area(mesh, t1);
            println!(
                "PAIR#{} edge=({:.4},{:.4},{:.4})-({:.4},{:.4},{:.4}) face={} winding={} apex_same_side={} angle={:.1} areas=({:.3e},{:.3e})",
                n_pairs,
                pa.x, pa.y, pa.z, pb.x, pb.y, pb.z,
                fid_to_step.get(&fid0).copied().unwrap_or(-1),
                winding,
                same_side,
                angle,
                a0, a1
            );
            for (label, t) in [("T0", t0), ("T1", t1)] {
                let v = |i: u32| {
                    let p = mesh.vertices[i as usize];
                    format!("({:.4},{:.4},{:.4})", p.x, p.y, p.z)
                };
                println!(
                    "  {}: {} {} {} area={:.3e}",
                    label,
                    v(t[0]),
                    v(t[1]),
                    v(t[2]),
                    tri_area(mesh, t)
                );
            }
        }
        println!("TOTAL candidate pairs: {}", n_pairs);
        break; // first matching BREP only
    }
}

fn tri_normal(mesh: &draper_mesh::TriangleMesh, t: [u32; 3]) -> Option<(f64, f64, f64)> {
    let a = mesh.vertices[t[0] as usize];
    let b = mesh.vertices[t[1] as usize];
    let c = mesh.vertices[t[2] as usize];
    let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
    let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
    let n = (
        e1.1 * e2.2 - e1.2 * e2.1,
        e1.2 * e2.0 - e1.0 * e2.2,
        e1.0 * e2.1 - e1.1 * e2.0,
    );
    let l = n.0.hypot(n.1.hypot(n.2));
    if l < 1e-15 {
        None
    } else {
        Some(n)
    }
}

fn tri_area(mesh: &draper_mesh::TriangleMesh, t: [u32; 3]) -> f64 {
    let n = tri_normal(mesh, t).unwrap_or((0.0, 0.0, 0.0));
    0.5 * n.0.hypot(n.1.hypot(n.2))
}

fn edge_dir(t: [u32; 3], edge: (u32, u32)) -> bool {
    // true if the triangle traverses the edge low->high
    for w in 0..3 {
        let v0 = t[w];
        let v1 = t[(w + 1) % 3];
        if (v0, v1) == edge {
            return true;
        }
    }
    false
}

fn apex_of(t: [u32; 3], edge: (u32, u32)) -> u32 {
    for &v in &t {
        if v != edge.0 && v != edge.1 {
            return v;
        }
    }
    t[0]
}
