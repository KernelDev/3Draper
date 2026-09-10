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
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = check_manifold(&inst.mesh);
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
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();
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
