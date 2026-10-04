//! session-75 diagnostic: compare the discretization of SHARED rim edges
//! between adjacent faces involved in REAL fold pairs (HM 57|58, 105|113,
//! 147|148, 7|226, 3|121). The consistent-triangulation pipeline promises
//! bit-identical points on shared STEP edges; if the two faces' wires carry
//! DIFFERENT point sequences on the shared stretch, the global weld has to
//! merge two different chains — T-junction repair then fans needles along
//! the rim (the s75 fold families).
//!
//! Usage: s75_rim_match <file.stp> [pending_idx] [fidA:fidB,...]
//! Default: drill_top.stp 4 57:58,105:113,147:148,7:226,3:121
//!
//! For each face pair:
//!   - list all wire polylines of both faces (id, pts, bbox, length)
//!   - find the best-overlapping polyline pair (max shared-point fraction)
//!   - report: |A∩B| / |A|, |A∩B| / |B| with tol 1e-7 (bit-exact-ish),
//!     plus nearest-point distance stats for the non-matching tail
//!   - dump the shared-chain sequences to /tmp/s75_rim_<fidA>_<fidB>.txt

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::Point3d;

fn dist(a: &Point3d, b: &Point3d) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn poly_len(p: &[Point3d]) -> f64 {
    p.windows(2).map(|w| dist(&w[0], &w[1])).sum()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "test/drill_top.stp".to_string());
    let idx: usize = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(4);
    let pairs: Vec<(u64, u64)> = args
        .get(3)
        .map(|s| {
            s.split(',')
                .filter_map(|p| {
                    let mut it = p.split(':');
                    let a = it.next()?.parse().ok()?;
                    let b = it.next()?.parse().ok()?;
                    Some((a, b))
                })
                .collect()
        })
        .unwrap_or_else(|| {
            vec![(57, 58), (105, 113), (147, 148), (7, 226), (3, 121)]
        });

    println!("Loading {} pending={}", path, idx);
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    let p = &pending[idx];
    let inst = ctx.triangulate_pending(p).expect("triangulate");

    for (fa, fb) in pairs {
        println!("\n===== faces {} | {} =====", fa, fb);
        let face_a = inst.faces.iter().find(|f| f.face_id == fa);
        let face_b = inst.faces.iter().find(|f| f.face_id == fb);
        let (Some(fa_info), Some(fb_info)) = (face_a, face_b) else {
            println!("  face not found");
            continue;
        };
        println!(
            "  A: {} step={} fwd={} polys(out/in)={}/{}",
            fa_info.surface_type,
            fa_info.step_face_id,
            fa_info.forward,
            fa_info.outer_boundary.len(),
            fa_info.inner_boundaries.len()
        );
        println!(
            "  B: {} step={} fwd={} polys(out/in)={}/{}",
            fb_info.surface_type,
            fb_info.step_face_id,
            fb_info.forward,
            fb_info.outer_boundary.len(),
            fb_info.inner_boundaries.len()
        );

        let polys_a: Vec<&Vec<Point3d>> = fa_info
            .outer_boundary
            .iter()
            .chain(fa_info.inner_boundaries.iter())
            .collect();
        let polys_b: Vec<&Vec<Point3d>> = fb_info
            .outer_boundary
            .iter()
            .chain(fb_info.inner_boundaries.iter())
            .collect();

        // report each polyline briefly
        for (label, polys) in [("A", &polys_a), ("B", &polys_b)] {
            for (i, poly) in polys.iter().enumerate() {
                if poly.len() < 2 {
                    continue;
                }
                println!(
                    "  {}[{}]: pts={} len={:.4} endpoints=({:.4},{:.4},{:.4})-({:.4},{:.4},{:.4})",
                    label,
                    i,
                    poly.len(),
                    poly_len(poly),
                    poly.first().unwrap().x,
                    poly.first().unwrap().y,
                    poly.first().unwrap().z,
                    poly.last().unwrap().x,
                    poly.last().unwrap().y,
                    poly.last().unwrap().z
                );
            }
        }

        // Whole wires are single closed polylines (all edges concatenated),
        // so endpoint matching is useless. Instead: point-set matching —
        // for every A-point find the nearest B-point; the shared rim is the
        // maximal run of A-points with nearest-B < tol. Report both sides.
        let wa = &polys_a[0];
        let wb = &polys_b[0];
        let near = |q: &Point3d, poly: &[Point3d]| -> f64 {
            poly.iter()
                .map(|p| dist(p, q))
                .fold(f64::INFINITY, f64::min)
        };
        let tol = 1e-6;
        let a_shared: Vec<usize> = wa
            .iter()
            .enumerate()
            .filter(|(_, q)| near(q, wb) < tol)
            .map(|(i, _)| i)
            .collect();
        let b_shared: Vec<usize> = wb
            .iter()
            .enumerate()
            .filter(|(_, q)| near(q, wa) < tol)
            .map(|(i, _)| i)
            .collect();
        println!(
            "  POINT-MATCH tol={:e}: A {}/{} pts on shared rim, B {}/{} pts on shared rim",
            tol,
            a_shared.len(),
            wa.len(),
            b_shared.len(),
            wb.len()
        );
        if a_shared.is_empty() {
            println!("  no shared rim points at all");
            continue;
        }
        // longest consecutive run of shared points in A
        let mut run = 1usize;
        let mut best_run = 1usize;
        for w in a_shared.windows(2) {
            if w[1] == w[0] + 1 {
                run += 1;
                best_run = best_run.max(run);
            } else {
                run = 1;
            }
        }
        println!(
            "  A shared-run max={}/{} (scattered over {} runs)",
            best_run,
            a_shared.len(),
            a_shared
                .windows(2)
                .filter(|w| w[1] != w[0] + 1)
                .count()
                + 1
        );
        // One-to-one pairing check: the counts must be EQUAL for a
        // bit-identical shared-edge discretization. A denser side means
        // the weld had to merge two different chains of the same rim.
        if a_shared.len() == b_shared.len() {
            println!(
                "  DISCRETIZATION MATCH: {} pts on both sides",
                a_shared.len()
            );
        } else {
            println!(
                "  *** DISCRETIZATION MISMATCH: A has {} rim pts, B has {} — weld must merge chains (diff {:+}) ***",
                a_shared.len(),
                b_shared.len(),
                a_shared.len() as i64 - b_shared.len() as i64
            );
        }
        // Dump both wires' shared-rim sequences with nearest distances.
        let out_path = format!("/tmp/s75_rim_{}_{}.txt", fa, fb);
        let mut out = String::new();
        out.push_str(&format!(
            "# A {} pts ({} shared), B {} pts ({} shared)\n",
            wa.len(),
            a_shared.len(),
            wb.len(),
            b_shared.len()
        ));
        for &i in &a_shared {
            let q = &wa[i];
            out.push_str(&format!(
                "A {:4} {:12.8} {:12.8} {:12.8} nearest_B {:.3e}\n",
                i, q.x, q.y, q.z,
                near(q, wb)
            ));
        }
        for &i in &b_shared {
            let q = &wb[i];
            out.push_str(&format!(
                "B {:4} {:12.8} {:12.8} {:12.8} nearest_A {:.3e}\n",
                i, q.x, q.y, q.z,
                near(q, wa)
            ));
        }
        let _ = std::fs::write(&out_path, out);
        println!("  rim sequences dumped to {}", out_path);
    }
}
