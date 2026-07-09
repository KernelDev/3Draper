// Check if bottom and top ring points are at the same angles.
// If not, the quad grid will be twisted.
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

        for target_id in &[78i64, 84i64] {
            for fi in &inst.faces {
                if fi.step_face_id != *target_id { continue; }

                let (start, end) = fi.triangle_range;
                println!("\n=== Step#{} Cone forward={} ===", fi.step_face_id, fi.forward);

                // Collect vertices by x-level
                let mut x_levels: HashMap<i64, Vec<(u32, f64, f64)>> = HashMap::new();
                let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    for &vi in &tri {
                        if seen.insert(vi) {
                            let v = inst.mesh.vertices[vi as usize];
                            let level = (v.x * 100.0).round() as i64;
                            let r = (v.y * v.y + v.z * v.z).sqrt();
                            let a = v.z.atan2(v.y).to_degrees();
                            x_levels.entry(level).or_default().push((vi, r, a));
                        }
                    }
                }

                let mut levels: Vec<_> = x_levels.into_iter().collect();
                levels.sort_by_key(|(l, _)| *l);

                // Get bottom and top ring angles
                let bottom = levels.first().map(|(_, v)| v).cloned().unwrap_or_default();
                let top = levels.last().map(|(_, v)| v).cloned().unwrap_or_default();

                if !bottom.is_empty() && !top.is_empty() {
                    // Sort both by angle
                    let mut b_sorted = bottom.clone();
                    b_sorted.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
                    let mut t_sorted = top.clone();
                    t_sorted.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

                    println!("  Bottom ring: {} verts", b_sorted.len());
                    println!("    first 5 angles: {:?}", b_sorted.iter().take(5).map(|v| v.2).collect::<Vec<_>>());
                    println!("    last 5 angles:  {:?}", b_sorted.iter().rev().take(5).map(|v| v.2).collect::<Vec<_>>());

                    println!("  Top ring: {} verts", t_sorted.len());
                    println!("    first 5 angles: {:?}", t_sorted.iter().take(5).map(|v| v.2).collect::<Vec<_>>());
                    println!("    last 5 angles:  {:?}", t_sorted.iter().rev().take(5).map(|v| v.2).collect::<Vec<_>>());

                    // Check if angles match (bottom[i] vs top[i])
                    if b_sorted.len() == t_sorted.len() {
                        let mut max_diff = 0.0f64;
                        let mut mismatches = 0;
                        for i in 0..b_sorted.len() {
                            let diff = (b_sorted[i].2 - t_sorted[i].2).abs();
                            if diff > max_diff { max_diff = diff; }
                            if diff > 0.5 {
                                mismatches += 1;
                            }
                        }
                        println!("\n  Angle alignment (bottom[i] vs top[i]):");
                        println!("    max angle diff: {:.4}°", max_diff);
                        println!("    mismatches (>0.5°): {}/{}", mismatches, b_sorted.len());

                        // Show first 10 mismatches
                        if mismatches > 0 {
                            println!("    mismatch examples:");
                            let mut shown = 0;
                            for i in 0..b_sorted.len() {
                                let diff = (b_sorted[i].2 - t_sorted[i].2).abs();
                                if diff > 0.5 && shown < 10 {
                                    println!("      [{}]: bottom={:.4}° top={:.4}° diff={:.4}°",
                                        i, b_sorted[i].2, t_sorted[i].2, diff);
                                    shown += 1;
                                }
                            }
                        }
                    }
                }

                // Also check: for each triangle, compute the angular mismatch
                // between its 3 vertices (how much they deviate from being at
                // the same angular column)
                let mut twisted_quads = 0;
                let mut examples: Vec<(usize, f64, f64, f64)> = Vec::new();
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];
                    let a0 = v0.z.atan2(v0.y).to_degrees();
                    let a1 = v1.z.atan2(v1.y).to_degrees();
                    let a2 = v2.z.atan2(v2.y).to_degrees();

                    // Check if 2 vertices are at the same x-level and one is at a different level
                    let x0 = (v0.x * 100.0).round() as i64;
                    let x1 = (v1.x * 100.0).round() as i64;
                    let x2 = (v2.x * 100.0).round() as i64;

                    // If 2 verts at same x, check if their angles match the 3rd vert's angle
                    if x0 == x1 && x0 != x2 {
                        // v0 and v1 at same level, v2 at different level
                        let mid_a = (a0 + a1) / 2.0;
                        let diff = (mid_a - a2).abs();
                        if diff > 3.0 {
                            twisted_quads += 1;
                            if examples.len() < 10 {
                                examples.push((i, a0, a1, a2));
                            }
                        }
                    } else if x0 == x2 && x0 != x1 {
                        let mid_a = (a0 + a2) / 2.0;
                        let diff = (mid_a - a1).abs();
                        if diff > 3.0 {
                            twisted_quads += 1;
                            if examples.len() < 10 {
                                examples.push((i, a0, a1, a2));
                            }
                        }
                    } else if x1 == x2 && x1 != x0 {
                        let mid_a = (a1 + a2) / 2.0;
                        let diff = (mid_a - a0).abs();
                        if diff > 3.0 {
                            twisted_quads += 1;
                            if examples.len() < 10 {
                                examples.push((i, a0, a1, a2));
                            }
                        }
                    }
                }
                println!("\n  Twisted quads (angular mismatch > 3°): {}", twisted_quads);
                if !examples.is_empty() {
                    println!("    Examples:");
                    for (i, a0, a1, a2) in &examples {
                        println!("      tri[{}]: angles=({:.2}°, {:.2}°, {:.2}°)", i, a0, a1, a2);
                    }
                }
            }
        }

        break;
    }
}
