//! Dump triangulation data for a specific STEP face ID.
//! Usage: step_face_dump <file.stp> <step_face_id>
//! Example: step_face_dump test/drill_top.stp 803

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let target_step_id: i64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(803);

    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    println!("Found {} BREP(s) in {}", pending.len(), path);

    let mut found_count = 0;
    for p in &pending {
        let ctx = StepConversionContext::new(&step);
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => continue,
        };

        for (face_idx, face) in inst.faces.iter().enumerate() {
            if face.step_face_id == target_step_id {
                found_count += 1;
                let tris = face.triangle_range.1 - face.triangle_range.0;
                println!("\n=== BREP #{} Face #{} (STEP #{}, surf={}, forward={}) === tris={}",
                    p.brep_id, face_idx + 1, face.step_face_id, face.surface_type, face.forward, tris);

                // Print outer boundary UV loops
                for (li, loop_uv) in face.outer_uv_boundary.iter().enumerate() {
                    println!("\nouter UV loop {}: {} points", li, loop_uv.len());
                    let u_min = loop_uv.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                    let u_max = loop_uv.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                    let v_min = loop_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                    let v_max = loop_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                    println!("  U range: [{:.4}, {:.4}] (range={:.4})", u_min, u_max, u_max - u_min);
                    println!("  V range: [{:.4}, {:.4}] (range={:.4})", v_min, v_max, v_max - v_min);
                    println!("  period check: u_range/2pi={:.3}, v_range/2pi={:.3}",
                        (u_max-u_min)/(2.0*std::f64::consts::PI),
                        (v_max-v_min)/(2.0*std::f64::consts::PI));
                    // Print first 5, last 5
                    for (i, uv) in loop_uv.iter().enumerate().take(5) {
                        println!("  uv[{}]: ({:.4}, {:.4})", i, uv.u, uv.v);
                    }
                    if loop_uv.len() > 10 {
                        println!("  ...");
                        for i in (loop_uv.len() - 5)..loop_uv.len() {
                            println!("  uv[{}]: ({:.4}, {:.4})", i, loop_uv[i].u, loop_uv[i].v);
                        }
                    }
                }

                // Print inner boundaries (holes)
                for (hi, hole) in face.inner_uv_boundaries.iter().enumerate() {
                    for (li, loop_uv) in hole.iter().enumerate() {
                        println!("\nhole {} UV loop {}: {} points", hi, li, loop_uv.len());
                        let u_min = loop_uv.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                        let u_max = loop_uv.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                        let v_min = loop_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                        let v_max = loop_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                        println!("  U range: [{:.4}, {:.4}] (range={:.4})", u_min, u_max, u_max - u_min);
                        println!("  V range: [{:.4}, {:.4}] (range={:.4})", v_min, v_max, v_max - v_min);
                    }
                }

                // Print UV triangles — check for any issues
                println!("\nUV triangles: {}", face.uv_triangles.len());
                if !face.uv_triangles.is_empty() {
                    // Compute UV area of each triangle
                    let mut min_area = f64::MAX;
                    let mut max_area = 0.0_f64;
                    let mut total_area = 0.0_f64;
                    let mut n_zero = 0;
                    let mut n_huge = 0;
                    for tri in &face.uv_triangles {
                        let area = ((tri[1].u - tri[0].u) * (tri[2].v - tri[0].v)
                            - (tri[2].u - tri[0].u) * (tri[1].v - tri[0].v)).abs() * 0.5;
                        min_area = min_area.min(area);
                        max_area = max_area.max(area);
                        total_area += area;
                        if area < 1e-9 { n_zero += 1; }
                        if area > 1.0 { n_huge += 1; }
                    }
                    println!("  area: min={:.6}, max={:.6}, total={:.4}, avg={:.6}",
                        min_area, max_area, total_area, total_area / face.uv_triangles.len() as f64);
                    println!("  zero-area triangles (area<1e-9): {}", n_zero);
                    println!("  huge-area triangles (area>1.0): {}", n_huge);

                    // Print first 5 triangles
                    for (i, tri) in face.uv_triangles.iter().enumerate().take(5) {
                        println!("  tri[{}]: ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                            i, tri[0].u, tri[0].v, tri[1].u, tri[1].v, tri[2].u, tri[2].v);
                    }

                    // Print the largest 5 triangles (by UV area) — these are suspect
                    let mut tris_with_area: Vec<(usize, f64)> = face.uv_triangles.iter().enumerate()
                        .map(|(i, tri)| {
                            let area = ((tri[1].u - tri[0].u) * (tri[2].v - tri[0].v)
                                - (tri[2].u - tri[0].u) * (tri[1].v - tri[0].v)).abs() * 0.5;
                            (i, area)
                        }).collect();
                    tris_with_area.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
                    println!("\n  Largest 5 triangles by UV area:");
                    for (i, area) in tris_with_area.iter().take(5) {
                        let tri = &face.uv_triangles[*i];
                        println!("    tri[{}]: area={:.4} ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                            i, area, tri[0].u, tri[0].v, tri[1].u, tri[1].v, tri[2].u, tri[2].v);
                    }
                }
            }
        }
    }

    if found_count == 0 {
        println!("Face with STEP ID {} not found", target_step_id);
    }
}
