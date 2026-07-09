// Compare Step#84 vs Step#78 — both are cone faces, should have similar structure
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

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

        for target_id in &[78i64, 84i64] {
            for fi in &inst.faces {
                if fi.step_face_id != *target_id { continue; }

                let (start, end) = fi.triangle_range;
                println!("\n=== Step#{} Cone forward={} ===", fi.step_face_id, fi.forward);
                println!("  triangle_range: [{}, {})", start, end);

                // Boundary analysis
                for (pi, polyline) in fi.outer_boundary.iter().enumerate() {
                    println!("  outer[{}] ({} pts):", pi, polyline.len());
                    // Group by x-level
                    let mut x_levels: std::collections::HashMap<i64, Vec<(f64, f64)>> = std::collections::HashMap::new();
                    for p in polyline {
                        let level = (p.x * 100.0).round() as i64;
                        let r = (p.y * p.y + p.z * p.z).sqrt();
                        let a = p.z.atan2(p.y).to_degrees();
                        x_levels.entry(level).or_default().push((r, a));
                    }
                    let mut levels: Vec<_> = x_levels.into_iter().collect();
                    levels.sort_by_key(|(l, _)| *l);
                    for (level, pts) in levels {
                        let x = level as f64 / 100.0;
                        let r_vals: Vec<f64> = pts.iter().map(|(r, _)| *r).collect();
                        let a_vals: Vec<f64> = pts.iter().map(|(_, a)| *a).collect();
                        let r_min = r_vals.iter().fold(f64::MAX, |a, &b| a.min(b));
                        let r_max = r_vals.iter().fold(f64::MIN, |a, &b| a.max(b));
                        let a_min = a_vals.iter().fold(f64::MAX, |a, &b| a.min(b));
                        let a_max = a_vals.iter().fold(f64::MIN, |a, &b| a.max(b));
                        println!("    x={:.2}: {} pts, r=[{:.2},{:.2}], angle=[{:.1}°,{:.1}°]",
                            x, pts.len(), r_min, r_max, a_min, a_max);
                    }
                }

                // Check: does the boundary contain the full circle (0° to 360°) or just half?
                println!("\n  Vertex angle distribution in mesh:");
                let mut angle_buckets: [usize; 36] = [0; 36]; // 10° buckets
                let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    for &vi in &tri {
                        if seen.insert(vi) {
                            let v = inst.mesh.vertices[vi as usize];
                            let a = v.z.atan2(v.y).to_degrees();
                            let a_norm = ((a % 360.0) + 360.0) % 360.0;
                            let bucket = (a_norm / 10.0) as usize % 36;
                            angle_buckets[bucket] += 1;
                        }
                    }
                }
                for (i, &count) in angle_buckets.iter().enumerate() {
                    if count > 0 {
                        println!("    [{:3.0}°, {:3.0}°): {}", i as f64 * 10.0, (i+1) as f64 * 10.0, count);
                    }
                }

                // Surface info
                if let draper_geometry::Surface::Cone(ref cone) = fi.surface {
                    println!("\n  Cone surface:");
                    println!("    radius={:.4}, half_angle={:.4}deg", cone.radius, cone.half_angle.to_degrees());
                    println!("    origin=({:.4},{:.4},{:.4})", cone.origin.x, cone.origin.y, cone.origin.z);
                    println!("    axis=({:.4},{:.4},{:.4})", cone.axis.x, cone.axis.y, cone.axis.z);
                    println!("    x_dir=({:.4},{:.4},{:.4})", cone.x_dir.x, cone.x_dir.y, cone.x_dir.z);
                }
            }
        }

        break;
    }
}
