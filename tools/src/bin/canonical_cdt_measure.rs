// Diagnostic: measure the surface-level canonical CDT (Vision 2036 Phase 1,
// session-30) on real STEP files — per-BREP tris / boundary / non-manifold
// with `use_surface_canonical_cdt` OFF vs ON, plus merged totals.
//
// Usage: canonical_cdt_measure <file.stp> [more.stp ...]
//
// The flag is default-off; this tool flips it to quantify exactly what the
// canonical path changes (interior Steiner fill without cross-face boundary
// regressions — see ROADMAP_VISION_2036 §"100% watertight").

use draper_mesh::{TriangulationParams, check_manifold};
use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};

fn run(path: &str, canonical: bool) -> Vec<(String, i64, usize, usize, usize)> {
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => {
            println!("  ERROR reading {}: {}", path, e);
            return Vec::new();
        }
    };
    let step_file = match parse_step(&content) {
        Ok(f) => f,
        Err(e) => {
            println!("  ERROR parsing {}: {}", path, e);
            return Vec::new();
        }
    };
    let (_tree, pending) = step_structure_lazy(&step_file);

    let mut params = TriangulationParams::default();
    params.use_surface_canonical_cdt = canonical;

    let mut ctx = OwnedStepConversionContext::new_with_params(step_file, params);
    let mut out = Vec::new();
    let mut first_dumped = false;
    let target_brep: i64 = std::env::var("TARGET_BREP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(-1);
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = check_manifold(&inst.mesh);
            if canonical && p.brep_id == target_brep && std::env::var("DUMP_BND").is_ok() {
                use std::collections::HashMap;
                eprintln!(
                    "=== TARGET BREP #{} ({}) tris={} bnd={} ===",
                    p.brep_id, p.name, inst.mesh.triangle_count(), report.boundary_edge_count
                );
                for f in &inst.faces {
                    let (t0, t1) = f.triangle_range;
                    if t1 <= t0 {
                        eprintln!("  face #{} [{}] EMITS NOTHING", f.step_face_id, f.surface_type);
                        continue;
                    }
                    let mut fec: HashMap<(u32, u32), usize> = HashMap::new();
                    for ti in t0..t1 {
                        let tri = &inst.mesh.triangles[ti];
                        for i in 0..3 {
                            let a = tri[i].min(tri[(i + 1) % 3]);
                            let b = tri[i].max(tri[(i + 1) % 3]);
                            *fec.entry((a, b)).or_insert(0) += 1;
                        }
                    }
                    let lb = fec.values().filter(|&&c| c == 1).count();
                    eprintln!(
                        "  face #{} [{}] tris={} local_bnd={}",
                        f.step_face_id, f.surface_type, t1 - t0, lb
                    );
                }
                let mut ec: HashMap<(u32, u32), usize> = HashMap::new();
                for tri in &inst.mesh.triangles {
                    for i in 0..3 {
                        let a = tri[i].min(tri[(i + 1) % 3]);
                        let b = tri[i].max(tri[(i + 1) % 3]);
                        *ec.entry((a, b)).or_insert(0) += 1;
                    }
                }
                let mut bnd: Vec<(u32, u32)> = ec
                    .iter()
                    .filter(|(_, &c)| c == 1)
                    .map(|(&(a, b), _)| (a, b))
                    .collect();
                bnd.sort();
                eprintln!("  merged boundary edges: {}", bnd.len());
                let bits = |q: &draper_geometry::Point3d| {
                    [q.x.to_bits(), q.y.to_bits(), q.z.to_bits()]
                };
                for &(a, b) in bnd.iter().take(14) {
                    let pa = inst.mesh.vertices[a as usize];
                    let pb = inst.mesh.vertices[b as usize];
                    // owners
                    let mut owners = Vec::new();
                    for f in inst.faces.iter() {
                        let (t0, t1) = f.triangle_range;
                        let mut cnt = 0usize;
                        for ti in t0..t1 {
                            let tri = &inst.mesh.triangles[ti];
                            for k in 0..3 {
                                let ea = tri[k].min(tri[(k + 1) % 3]);
                                let eb = tri[k].max(tri[(k + 1) % 3]);
                                if (ea, eb) == (a, b) {
                                    cnt += 1;
                                }
                            }
                        }
                        if cnt > 0 {
                            owners.push(format!("#{}({})x{}", f.step_face_id, f.surface_type, cnt));
                        }
                    }
                    // loop-chord membership
                    let ka = bits(&pa);
                    let kb = bits(&pb);
                    let mut hit = String::new();
                    for f in inst.faces.iter() {
                        if !hit.is_empty() {
                            break;
                        }
                        let mut scan = |pl: &[draper_geometry::Point3d], tag: &str| {
                            if !pl.is_empty() && hit.is_empty() {
                                for w in 0..pl.len() {
                                    let w2 = (w + 1) % pl.len();
                                    if (bits(&pl[w]) == ka && bits(&pl[w2]) == kb)
                                        || (bits(&pl[w]) == kb && bits(&pl[w2]) == ka)
                                    {
                                        hit = format!(
                                            "#{} {}[{}]/{}",
                                            f.step_face_id, tag, w, pl.len()
                                        );
                                    }
                                }
                            }
                        };
                        for (bi, pl) in f.outer_boundary.iter().enumerate() {
                            scan(pl, &format!("outer{}", bi));
                        }
                        for (bi, pl) in f.inner_boundaries.iter().enumerate() {
                            scan(pl, &format!("inner{}", bi));
                        }
                    }
                    if hit.is_empty() {
                        hit = "NOT-A-LOOP-CHORD".to_string();
                    }
                    eprintln!(
                        "    ({:.4},{:.4},{:.4})-({:.4},{:.4},{:.4}) len={:.4} owners=[{}] chord={}",
                        pa.x, pa.y, pa.z, pb.x, pb.y, pb.z,
                        ((pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2) + (pa.z - pb.z).powi(2)).sqrt(),
                        owners.join(","), hit
                    );
                }
            }
            if canonical && report.boundary_edge_count > 0 && !first_dumped && std::env::var("DUMP_BND").is_ok() {
                first_dumped = true;
                eprintln!("=== boundary edges of {} (#{}) ===", p.name, p.brep_id);
                use std::collections::HashMap;
                let mut ec: HashMap<(u32, u32), usize> = HashMap::new();
                for tri in &inst.mesh.triangles {
                    for i in 0..3 {
                        let a = tri[i].min(tri[(i + 1) % 3]);
                        let b = tri[i].max(tri[(i + 1) % 3]);
                        *ec.entry((a, b)).or_insert(0) += 1;
                    }
                }
                for ((a, b), n) in ec {
                    if n == 1 {
                        let pa = inst.mesh.vertices[a as usize];
                        let pb = inst.mesh.vertices[b as usize];
                        eprintln!("  ({:.5},{:.5},{:.5})-({:.5},{:.5},{:.5}) len={:.5}",
                            pa.x, pa.y, pa.z, pb.x, pb.y, pb.z,
                            ((pa.x-pb.x).powi(2)+(pa.y-pb.y).powi(2)+(pa.z-pb.z).powi(2)).sqrt());
                    }
                }
            }
            out.push((
                p.name.clone(),
                p.brep_id,
                inst.mesh.triangle_count(),
                report.boundary_edge_count,
                report.non_manifold_edge_count,
            ));
        }
    }
    out
}

fn main() {
    eprintln!(
        "MESH-CRATE TAG: {}",
        draper_mesh::surface_canonical::CANON_BUILD_TAG
    );
    // Respect RUST_LOG when set (e.g. RUST_LOG=draper_mesh=info,draper_step=info
    // to see the canonical pre-pass built/dropped summaries and the group
    // rescue actions); default to errors only.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    let files: Vec<String> = std::env::args().skip(1).collect();
    let files = if files.is_empty() {
        vec![
            "test/drill_top.stp".to_string(),
            "test/as1-oc-214.stp".to_string(),
        ]
    } else {
        files
    };

    println!(
        "{:<44} {:>9} {:>9} {:>8} {:>8}   {:>9} {:>9} {:>8} {:>8}",
        "BREP", "tris-0", "tris-1", "bnd-0", "bnd-1", "nm-0", "nm-1", "d.tris", "d.bnd"
    );
    println!("{}", "-".repeat(125));

    for f in &files {
        println!("== {} ==", f);
        let base = run(f, false);
        let canon = run(f, true);

        let mut tot = [0usize; 4];
        for (i, (name, brep_id, tris0, bnd0, nm0)) in base.iter().enumerate() {
            let (tris1, bnd1, nm1) = canon
                .get(i)
                .map(|(_, _, t, b, n)| (*t, *b, *n))
                .unwrap_or((0, 0, 0));
            tot[0] += tris0;
            tot[1] += tris1;
            tot[2] += bnd0;
            tot[3] += bnd1;
            println!(
                "{:<32} #{:<10} {:>9} {:>9} {:>8} {:>8}   {:>9} {:>9} {:>+8} {:>+8}",
                name,
                brep_id,
                tris0,
                tris1,
                bnd0,
                bnd1,
                nm0,
                nm1,
                tris1 as i64 - *tris0 as i64,
                bnd1 as i64 - *bnd0 as i64
            );
        }
        println!(
            "{:<44} {:>9} {:>9} {:>8} {:>8}",
            "  TOTAL", tot[0], tot[1], tot[2], tot[3]
        );
    }
}
