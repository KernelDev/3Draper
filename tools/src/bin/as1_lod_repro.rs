//! Diagnostic: reproduce the worker-path triangulation of a STEP file at
//! multiple LODs and report per-face mesh density + boundary loop segment
//! counts, to diagnose visual "degradation" (octagonal circles).
//!
//! Usage: as1_lod_repro <step_file> [lod ...]
//! Defaults: 0.1 0.3 0.5 0.75 1.0

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::triangulate::SteinerBudgetProfile;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/as1-oc-214.stp");
    let lods: Vec<f64> = if args.len() > 2 {
        args[2..].iter().filter_map(|s| s.parse().ok()).collect()
    } else {
        vec![0.1, 0.3, 0.5, 0.75, 1.0]
    };

    let content = std::fs::read_to_string(path).expect("read step file");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);
    println!("File: {}  ({} BREP instances)", path, pending.len());

    for lod in lods {
        let mut ctx = OwnedStepConversionContext::new_with_lod_and_profile(
            step.clone(),
            lod,
            SteinerBudgetProfile::Desktop,
        );
        let mut total_v = 0usize;
        let mut total_t = 0usize;
        let mut rows: Vec<String> = Vec::new();

        for p in &pending {
            if let Some(inst) = ctx.triangulate_pending(p) {
                let v = inst.mesh.vertex_count();
                let t = inst.mesh.triangle_count();
                total_v += v;
                total_t += t;

                // Per-face stats: NURBS faces only (the curved ones).
                let mut nurbs_faces = 0usize;
                let mut nurbs_tris = 0usize;
                let mut loop_sizes: Vec<usize> = Vec::new();
                let mut face_rows: Vec<(i64, usize, usize)> = Vec::new(); // (step_face_id, tris, max loop pts)
                for f in &inst.faces {
                    if f.surface_type.contains("Nurbs") || f.surface_type.contains("Spline") {
                        nurbs_faces += 1;
                        let tris = f.triangle_range.1 - f.triangle_range.0;
                        nurbs_tris += tris;
                        let mut max_loop = 0usize;
                        for lp in &f.outer_boundary {
                            max_loop = max_loop.max(lp.len());
                        }
                        for lp in &f.inner_boundaries {
                            max_loop = max_loop.max(lp.len());
                        }
                        loop_sizes.push(max_loop);
                        face_rows.push((f.step_face_id, tris, max_loop));
                    }
                }

                let line = format!(
                    "  BREP #{:<4} {:<28} V={:<6} T={:<6} | NURBS faces: {:<3} tris={:<6} loops(min/med/max)={:?}",
                    p.brep_id,
                    p.name,
                    v,
                    t,
                    nurbs_faces,
                    nurbs_tris,
                    {
                        let mut s = loop_sizes.clone();
                        s.sort_unstable();
                        if s.is_empty() {
                            "-".to_string()
                        } else {
                            format!("{}/{}/{}", s[0], s[s.len() / 2], s[s.len() - 1])
                        }
                    }
                );
                rows.push(line);

                // Detail dump at this LOD for the first BREP with NURBS faces.
                if lod == 0.75 && !face_rows.is_empty() && rows.len() <= 2 {
                    for (fid, tris, lp) in face_rows.iter().take(40) {
                        println!("      face #{}: {} tris, max loop pts {}", fid, tris, lp);
                    }
                }
            }
        }

        println!("\n=== LOD {:.2} (max_deviation for this LOD) ===", lod);
        for r in &rows {
            println!("{}", r);
        }
        println!("  TOTAL: V={} T={}", total_v, total_t);
    }
}
