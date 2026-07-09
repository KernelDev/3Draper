// Dump the actual grid structure of Step#84 — how many rows, how many cols,
// and check if the grid is well-formed.
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("read 3.05.078.stp");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => continue,
        };

        for fi in &inst.faces {
            if fi.step_face_id != 84 { continue; }

            let (start, end) = fi.triangle_range;
            println!("\n=== Step#84 Cone forward={} ===", fi.forward);
            println!("  triangle_range: [{}, {})", start, end);

            // Collect all unique vertices with their (x, angle) coordinates
            let mut verts: Vec<(u32, f64, f64)> = Vec::new(); // (idx, x, angle_deg)
            let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    if seen.insert(vi) {
                        let v = inst.mesh.vertices[vi as usize];
                        let angle = v.z.atan2(v.y).to_degrees();
                        verts.push((vi, v.x, angle));
                    }
                }
            }
            println!("  Unique vertices: {}", verts.len());

            // Group by x-level (bottom, top, intermediate)
            let mut x_levels: HashMap<i64, Vec<(u32, f64)>> = HashMap::new();
            for (vi, x, angle) in &verts {
                let level = (x * 100.0).round() as i64; // 0.01mm buckets
                x_levels.entry(level).or_default().push((*vi, *angle));
            }
            let mut levels: Vec<_> = x_levels.into_iter().collect();
            levels.sort_by_key(|(l, _)| *l);
            println!("\n  X-levels ({} distinct):", levels.len());
            for (level, mut verts_at_level) in levels {
                let x = level as f64 / 100.0;
                verts_at_level.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
                let r = {
                    let vi = verts_at_level[0].0;
                    let v = inst.mesh.vertices[vi as usize];
                    (v.y * v.y + v.z * v.z).sqrt()
                };
                println!("    x={:.2} r={:.2}: {} verts, angles [{:.1}°, {:.1}°]",
                    x, r, verts_at_level.len(),
                    verts_at_level.first().unwrap().1,
                    verts_at_level.last().unwrap().1);
                // Show angle gaps
                let mut gaps: Vec<f64> = Vec::new();
                for i in 1..verts_at_level.len() {
                    gaps.push(verts_at_level[i].1 - verts_at_level[i-1].1);
                }
                if !gaps.is_empty() {
                    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    println!("      angle gaps: min={:.2}° median={:.2}° max={:.2}°",
                        gaps[0], gaps[gaps.len()/2], gaps[gaps.len()-1]);
                }
            }

            // For each triangle, check which x-levels it spans
            let x_level_of = |vi: u32| -> i64 {
                let v = inst.mesh.vertices[vi as usize];
                (v.x * 100.0).round() as i64
            };

            let mut same_row_tris = 0;
            let mut adjacent_row_tris = 0;
            let mut far_row_tris = 0;
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let l0 = x_level_of(tri[0]);
                let l1 = x_level_of(tri[1]);
                let l2 = x_level_of(tri[2]);
                let levels = vec![l0, l1, l2];
                let min_l = *levels.iter().min().unwrap();
                let max_l = *levels.iter().max().unwrap();
                let distinct = {
                    let mut s = levels.clone();
                    s.sort();
                    s.dedup();
                    s.len()
                };
                if distinct == 1 {
                    same_row_tris += 1;
                } else if distinct == 2 && (max_l - min_l) <= 243 { // within 2.43mm
                    adjacent_row_tris += 1;
                } else {
                    far_row_tris += 1;
                }
            }
            println!("\n  Triangle row structure:");
            println!("    same-row: {}", same_row_tris);
            println!("    adjacent-row (within 2.43mm): {}", adjacent_row_tris);
            println!("    far-row (>2.43mm): {}", far_row_tris);
        }

        break;
    }
}
