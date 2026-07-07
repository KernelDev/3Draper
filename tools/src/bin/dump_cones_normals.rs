// Check Step#84 vs Step#78 winding order and visual correctness
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

                // For each triangle, compute the face normal via cross product
                // and check if it points radially outward (away from axis) or inward.
                // Cone axis is (-1,0,0), so radial = (0, y, z) direction.
                // forward=false → normals should point INWARD (toward axis)
                // forward=true  → normals should point OUTWARD (away from axis)
                let mut outward_count = 0;
                let mut inward_count = 0;
                let mut wrong: Vec<(usize, [f64;3], [f64;3], [f64;3], f64)> = Vec::new();

                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];

                    // Cross product (v1-v0) × (v2-v0)
                    let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
                    let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
                    let nx = e1[1] * e2[2] - e1[2] * e2[1];
                    let ny = e1[2] * e2[0] - e1[0] * e2[2];
                    let nz = e1[0] * e2[1] - e1[1] * e2[0];

                    // Radial direction at centroid
                    let cx = (v0.x + v1.x + v2.x) / 3.0;
                    let cy = (v0.y + v1.y + v2.y) / 3.0;
                    let cz = (v0.z + v1.z + v2.z) / 3.0;
                    let dot = ny * cy + nz * cz; // dot with radial (0, y, z)

                    if dot > 0.0 {
                        outward_count += 1;
                        if fi.forward == false && wrong.len() < 5 {
                            // For forward=false, outward is WRONG
                            wrong.push((i, [v0.x, v0.y, v0.z], [v1.x, v1.y, v1.z], [v2.x, v2.y, v2.z], dot));
                        }
                    } else {
                        inward_count += 1;
                        if fi.forward == true && wrong.len() < 5 {
                            // For forward=true, inward is WRONG
                            wrong.push((i, [v0.x, v0.y, v0.z], [v1.x, v1.y, v1.z], [v2.x, v2.y, v2.z], dot));
                        }
                    }
                }

                println!("  Outward normals: {}", outward_count);
                println!("  Inward normals:  {}", inward_count);
                let expected = if fi.forward { "outward" } else { "inward" };
                println!("  Expected (forward={}): {}", fi.forward, expected);
                let wrong_total = if fi.forward { inward_count } else { outward_count };
                println!("  WRONG direction: {}", wrong_total);

                if !wrong.is_empty() {
                    println!("\n  Wrong-direction examples:");
                    for (i, p0, p1, p2, dot) in &wrong {
                        println!("    tri[{}] dot={:.4}: ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2})",
                            i, dot, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
                    }
                }

                // Also check: triangle area distribution
                let mut areas: Vec<f64> = Vec::new();
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];
                    let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
                    let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
                    let cx = e1[1] * e2[2] - e1[2] * e2[1];
                    let cy = e1[2] * e2[0] - e1[0] * e2[2];
                    let cz = e1[0] * e2[1] - e1[1] * e2[0];
                    let area = 0.5 * (cx*cx + cy*cy + cz*cz).sqrt();
                    areas.push(area);
                }
                areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let n = areas.len();
                println!("\n  Triangle area stats ({} tris):", n);
                println!("    min:    {:.4}", areas[0]);
                println!("    25%:    {:.4}", areas[n/4]);
                println!("    median: {:.4}", areas[n/2]);
                println!("    75%:    {:.4}", areas[3*n/4]);
                println!("    max:    {:.4}", areas[n-1]);
                let tiny = areas.iter().filter(|&&a| a < 0.1).count();
                let small = areas.iter().filter(|&&a| a >= 0.1 && a < 1.0).count();
                let normal = areas.iter().filter(|&&a| a >= 1.0 && a < 10.0).count();
                let large = areas.iter().filter(|&&a| a >= 10.0).count();
                println!("    tiny (<0.1): {} small (0.1-1): {} normal (1-10): {} large (>=10): {}",
                    tiny, small, normal, large);
            }
        }

        break;
    }
}
