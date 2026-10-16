//! session-98 diagnostic: dump the emission UV triangles of the SLEEVE
//! spiral-groove wall faces (f93 STEP#26243, f147 STEP#27631, f152
//! STEP#27996) that SPAN the domain (u-width > threshold) — the fold
//! flap family. For each spanning triangle print UV + local 3D coords of
//! its verts, plus a compact census: how many spanning tris, their
//! u-widths, and the ring-index gap if the verts are ring points.
//! Also dumps the two chains' point counts for structure context.
//! Usage: s98_groove_uv_dump <file.stp> <brep_id> [u_thresh]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let brep_id: i64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(32629);
    let u_thresh: f64 = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.5);

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
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => {
                println!("BREP #{}: triangulate_pending None", brep_id);
                continue;
            }
        };
        println!("=== BREP #{} '{}' faces={} ===", brep_id, inst.name, inst.faces.len());

        for f in &inst.faces {
            let fid = f.step_face_id;
            if ![26243i64, 27631, 27996, 25173, 25188].contains(&fid) {
                continue;
            }
            // domain bbox from boundary
            let mut min_u = f64::MAX;
            let mut max_u = f64::MIN;
            let mut min_v = f64::MAX;
            let mut max_v = f64::MIN;
            for poly in &f.outer_uv_boundary {
                for pt in poly {
                    min_u = min_u.min(pt.u);
                    max_u = max_u.max(pt.u);
                    min_v = min_v.min(pt.v);
                    max_v = max_v.max(pt.v);
                }
            }
            println!(
                "\n--- face STEP#{} {} fwd={} tris=({},{}) uv_tris={}",
                fid,
                f.surface_type,
                f.forward,
                f.triangle_range.0,
                f.triangle_range.1,
                f.uv_triangles.len()
            );
            println!(
                "  DOMAIN u=[{:.5},{:.5}] v=[{:.5},{:.5}] width_u={:.4}",
                min_u, max_u, min_v, max_v, max_u - min_u
            );
            // v-level histogram of the boundary (top chain vs teeth)
            let mut levels: std::collections::BTreeMap<u64, usize> = Default::default();
            for poly in &f.outer_uv_boundary {
                for pt in poly {
                    let key = (pt.v * 1000.0).round() as i64;
                    *levels.entry(key.max(0) as u64).or_insert(0) += 1;
                }
            }
            let top_levels: Vec<_> = levels.iter().rev().take(6).collect();
            println!("  top v-levels (v_mm -> pts): {:?}", top_levels);

            // spanning triangles
            let mut spanning = 0usize;
            for (ti, t) in f.uv_triangles.iter().enumerate() {
                let tu = [t[0].u, t[1].u, t[2].u];
                let tv = [t[0].v, t[1].v, t[2].v];
                let w = tu.iter().cloned().fold(f64::MIN, f64::max)
                    - tu.iter().cloned().fold(f64::MAX, f64::min);
                let h = tv.iter().cloned().fold(f64::MIN, f64::max)
                    - tv.iter().cloned().fold(f64::MAX, f64::min);
                if w > u_thresh && (max_u - min_u) > 0.0 {
                    spanning += 1;
                    if spanning <= 12 {
                        println!(
                            "  SPAN tri#{} u-width={:.4} v-height={:.4} uv=({:.5},{:.5})({:.5},{:.5})({:.5},{:.5})",
                            ti, w, h, t[0].u, t[0].v, t[1].u, t[1].v, t[2].u, t[2].v
                        );
                    }
                }
            }
            println!("  spanning(u>{:.3}) tris: {}", u_thresh, spanning);
        }
    }
}
