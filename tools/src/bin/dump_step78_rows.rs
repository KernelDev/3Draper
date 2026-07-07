// Check n_v and grid structure for Step#78
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
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
            if fi.step_face_id != 78 { continue; }
            let (start, end) = fi.triangle_range;
            println!("\n=== Step#78 forward={} tris=[{},{}) ===", fi.forward, start, end);

            // Count rows by x-level
            let mut x_counts: std::collections::HashMap<i64, usize> = std::collections::HashMap::new();
            let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    if seen.insert(vi) {
                        let v = inst.mesh.vertices[vi as usize];
                        let level = (v.x * 100.0).round() as i64;
                        *x_counts.entry(level).or_insert(0) += 1;
                    }
                }
            }
            let mut levels: Vec<_> = x_counts.into_iter().collect();
            levels.sort_by_key(|(l, _)| *l);
            println!("  X-levels (rows):");
            for (level, count) in &levels {
                println!("    x={:.2}: {} verts", *level as f64 / 100.0, count);
            }
            println!("  Total rows: {} (n_v+1 = {}, n_v = {})",
                levels.len(), levels.len(), levels.len().saturating_sub(1));

            // For each triangle, show which rows it connects
            println!("\n  Triangle row connectivity:");
            let mut row_pairs: std::collections::HashMap<(i64, i64), usize> = std::collections::HashMap::new();
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let xs: Vec<i64> = tri.iter().map(|&vi| {
                    let v = inst.mesh.vertices[vi as usize];
                    (v.x * 100.0).round() as i64
                }).collect();
                let mut sorted_xs = xs.clone();
                sorted_xs.sort();
                let key = (sorted_xs[0], sorted_xs[2]);
                *row_pairs.entry(key).or_insert(0) += 1;
            }
            let mut pairs: Vec<_> = row_pairs.into_iter().collect();
            pairs.sort_by_key(|((lo, hi), _)| (*lo, *hi));
            for ((lo, hi), count) in &pairs {
                println!("    x=[{:.2}, {:.2}]: {} tris", *lo as f64 / 100.0, *hi as f64 / 100.0, count);
            }
        }
        break;
    }
}
