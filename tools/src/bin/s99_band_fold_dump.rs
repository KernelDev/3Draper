//! session-99 diagnostic: attribute the 8 REAL drill fold pairs
//! (HOUSING 3 + HOUSING_MIRROR 5) to their band-strip emission.
//! For each fold pair (two final-mesh triangle indices):
//!   1. resolve each triangle's FaceInfo (via mesh.triangle_face_ids)
//!   2. match its (world→local) vertices to the face's emission UV
//!      triangles (FaceInfo.uv_triangles, built by projecting the
//!      emission verts back to UV) through Surface::point_at
//!   3. report: final shared verts, emission UV of both triangles,
//!      whether the pair shares 2 UV points at emission, the ANALYTIC
//!      dihedral at emission positions vs the FINAL dihedral.
//! Discriminates: EMISSION-FOLD (band guard hole) vs WELD-CREATED
//! (positions moved post-emission) vs MERGE-ADJACENCY (pair became
//! edge-neighbours only after vertex dedup).
//! Usage: s99_band_fold_dump <file.stp> <brep_id> <triA,triB> [...]
//! Vertices printed in WORLD and BREP-LOCAL (analytic) frames.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn inv_transform(m: &[[f64; 4]; 4], w: [f64; 3]) -> [f64; 3] {
    // world = R * local + t  (rows 0..3, cols 0..3; col 3 = t)
    let t = [m[0][3], m[1][3], m[2][3]];
    let r = [
        [m[0][0], m[0][1], m[0][2]],
        [m[1][0], m[1][1], m[1][2]],
        [m[2][0], m[2][1], m[2][2]],
    ];
    // Solve R * l = w - t by Gaussian elimination (3x3).
    let b = [w[0] - t[0], w[1] - t[1], w[2] - t[2]];
    let mut a = r;
    let mut rhs = b;
    for col in 0..3 {
        let mut piv = col;
        for row in col + 1..3 {
            if a[row][col].abs() > a[piv][col].abs() {
                piv = row;
            }
        }
        a.swap(col, piv);
        rhs.swap(col, piv);
        let d = a[col][col];
        if d.abs() < 1e-12 {
            return [f64::NAN; 3];
        }
        for row in 0..3 {
            if row != col {
                let f = a[row][col] / d;
                for c in 0..3 {
                    a[row][c] -= f * a[col][c];
                }
                rhs[row] -= f * rhs[col];
            }
        }
    }
    [
        rhs[0] / a[0][0],
        rhs[1] / a[1][1],
        rhs[2] / a[2][2],
    ]
}

fn tri_normal(p: [[f64; 3]; 3]) -> Option<[f64; 3]> {
    let e1 = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
    let e2 = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
    let n = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if l > 1e-15 {
        Some([n[0] / l, n[1] / l, n[2] / l])
    } else {
        None
    }
}

fn dihedral_deg(n1: [f64; 3], n2: [f64; 3]) -> f64 {
    let dot = (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]).clamp(-1.0, 1.0);
    dot.acos().to_degrees()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).cloned().unwrap_or_else(|| "test/drill_top.stp".into());
    let brep_id: i64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(47598);
    let pairs: Vec<(usize, usize)> = args[3..]
        .iter()
        .filter_map(|s| {
            let ab: Vec<&str> = s.split(',').collect();
            if ab.len() == 2 {
                Some((ab[0].parse().ok()?, ab[1].parse().ok()?))
            } else {
                None
            }
        })
        .collect();

    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
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
        let tr = inst.transform.as_ref();

        for (ta, tb) in &pairs {
            println!("\n---- PAIR T{} / T{} ----", ta, tb);
            let mut info = Vec::new();
            for &ti in &[*ta, *tb] {
                let tri = mesh.triangles[ti];
                let wpts: Vec<[f64; 3]> = tri
                    .iter()
                    .map(|&vi| {
                        let v = &mesh.vertices[vi as usize];
                        [v.x, v.y, v.z]
                    })
                    .collect();
                let lpts: Vec<[f64; 3]> = wpts
                    .iter()
                    .map(|w| match tr {
                        Some(m) => inv_transform(m, *w),
                        None => *w,
                    })
                    .collect();
                let fid = fids.and_then(|f| f.get(ti).copied()).unwrap_or(u64::MAX);
                let face = face_by_fid.get(&fid);
                println!(
                    "  T{} fid={} face={} step={} range=({},{})",
                    ti,
                    fid,
                    face.map(|f| f.surface_type.clone()).unwrap_or("?".into()),
                    face.map(|f| f.step_face_id).unwrap_or(-1),
                    face.map(|f| f.triangle_range.0).unwrap_or(0),
                    face.map(|f| f.triangle_range.1).unwrap_or(0),
                );
                for (w, l) in wpts.iter().zip(lpts.iter()) {
                    println!(
                        "    world=({:.5},{:.5},{:.5}) local=({:.5},{:.5},{:.5})",
                        w[0], w[1], w[2], l[0], l[1], l[2]
                    );
                }
                // match against emission uv triangles
                if let Some(f) = face {
                    let mut best: Option<(
                        usize,
                        f64,
                        [[f64; 3]; 3],
                        [draper_geometry::Point2d; 3],
                    )> = None;
                    for (k, uvt) in f.uv_triangles.iter().enumerate() {
                        let mut pts = [[0.0f64; 3]; 3];
                        for j in 0..3 {
                            let p3 = f.surface.point_at(uvt[j].u, uvt[j].v);
                            pts[j] = [p3.x, p3.y, p3.z];
                        }
                        // best permutation assignment: try all 6
                        let perms = [
                            [0usize, 1, 2],
                            [0, 2, 1],
                            [1, 0, 2],
                            [1, 2, 0],
                            [2, 0, 1],
                            [2, 1, 0],
                        ];
                        for pm in perms {
                            let d = (0..3)
                                .map(|j| {
                                    let a = &lpts[j];
                                    let b = &pts[pm[j]];
                                    ((a[0] - b[0]).powi(2)
                                        + (a[1] - b[1]).powi(2)
                                        + (a[2] - b[2]).powi(2))
                                    .sqrt()
                                })
                                .fold(0.0f64, f64::max);
                            if best.is_none() || d < best.as_ref().unwrap().1 {
                                best = Some((k, d, pts, *uvt));
                            }
                        }
                    }
                    if let Some((k, d, pts, uvs)) = best {
                        println!(
                            "  emission uv_tri #{} dmax={:.5} uv=({:.4},{:.4})({:.4},{:.4})({:.4},{:.4})",
                            k, d, uvs[0].u, uvs[0].v, uvs[1].u, uvs[1].v, uvs[2].u, uvs[2].v
                        );
                        println!(
                            "    analytic local=({:.5},{:.5},{:.5})({:.5},{:.5},{:.5})({:.5},{:.5},{:.5})",
                            pts[0][0], pts[0][1], pts[0][2],
                            pts[1][0], pts[1][1], pts[1][2],
                            pts[2][0], pts[2][1], pts[2][2]
                        );
                        info.push((fid, k, d, uvs, pts));
                    } else {
                        println!("  NO uv_triangles to match");
                    }
                }
            }
            // cross analysis
            if info.len() == 2 {
                let (_, _, d0, uv0, p0) = &info[0];
                let (_, _, d1, uv1, p1) = &info[1];
                // UV-shared points?
                let mut shared_uv = 0usize;
                for a in uv0 {
                    for b in uv1 {
                        if (a.u - b.u).abs() < 1e-9 && (a.v - b.v).abs() < 1e-9 {
                            shared_uv += 1;
                        }
                    }
                }
                let near_uv = (0..3)
                    .map(|i| {
                        (0..3)
                            .map(|j| {
                                ((uv0[i].u - uv1[j].u).hypot(uv0[i].v - uv1[j].v))
                            })
                            .fold(f64::INFINITY, f64::min)
                    })
                    .collect::<Vec<_>>();
                let n0 = tri_normal(*p0);
                let n1 = tri_normal(*p1);
                let ana = match (n0, n1) {
                    (Some(a), Some(b)) => dihedral_deg(a, b),
                    _ => f64::NAN,
                };
                // final dihedral
                let fw = |ti: usize| -> [[f64; 3]; 3] {
                    let t = mesh.triangles[ti];
                    let mut o = [[0.0; 3]; 3];
                    for j in 0..3 {
                        let v = &mesh.vertices[t[j] as usize];
                        o[j] = [v.x, v.y, v.z];
                    }
                    o
                };
                let fd = match (
                    tri_normal(fw(*ta)),
                    tri_normal(fw(*tb)),
                ) {
                    (Some(a), Some(b)) => dihedral_deg(a, b),
                    _ => f64::NAN,
                };
                println!("  ANALYSIS: shared_exact_uv={} near_uv_dists=[{:.2e},{:.2e},{:.2e}]",
                    shared_uv, near_uv[0], near_uv[1], near_uv[2]);
                println!("  ANALYSIS: analytic dihedral={:.2}°  final dihedral={:.2}°  dmax=({:.5},{:.5})",
                    ana, fd, d0, d1);
                let same_face = info[0].0 == info[1].0;
                if ana > 170.0 {
                    println!("  VERDICT: EMISSION-FOLD (analytic dihedral folded; band guard hole)");
                } else if shared_uv >= 2 {
                    println!("  VERDICT: WELD-CREATED (UV-shared edge, clean at emission, folded after vertex welding)");
                } else {
                    println!(
                        "  VERDICT: MERGE-ADJACENCY ({}; pair became neighbours at vertex dedup/merge)",
                        if same_face { "same face" } else { "cross face" }
                    );
                }
            }
        }
    }
}
