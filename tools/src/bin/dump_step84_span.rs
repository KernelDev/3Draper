// Check the angular span of each triangle (the small arc, not the big gap)
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

        for fi in &inst.faces {
            if fi.step_face_id != 84 { continue; }

            let (start, end) = fi.triangle_range;
            println!("\n=== Step#84 Cone forward={} ===", fi.forward);

            let norm = |a: f64| ((a % (2.0 * std::f64::consts::PI)) + 2.0 * std::f64::consts::PI) % (2.0 * std::f64::consts::PI);
            let angle = |y: f64, z: f64| norm(z.atan2(y));

            // Angular span = the small arc covered by the 3 vertices
            // = 2π - max_gap
            let mut spans: Vec<f64> = Vec::new();
            let mut bad_count = 0;
            let mut examples: Vec<(usize, [f64; 3], [f64; 3], [f64; 3], f64)> = Vec::new();

            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];

                let a0 = angle(v0.y, v0.z);
                let a1 = angle(v1.y, v1.z);
                let a2 = angle(v2.y, v2.z);

                let mut angles = vec![a0, a1, a2];
                angles.sort_by(|a, b| a.partial_cmp(b).unwrap());

                let g1 = angles[1] - angles[0];
                let g2 = angles[2] - angles[1];
                let g3 = 2.0 * std::f64::consts::PI - (angles[2] - angles[0]);
                let max_gap = g1.max(g2).max(g3);
                let span = 2.0 * std::f64::consts::PI - max_gap;
                spans.push(span);

                // Bad if span > 90° (triangle covers too much angular range)
                if span > std::f64::consts::PI / 2.0 {
                    bad_count += 1;
                    if examples.len() < 10 {
                        examples.push((i,
                            [v0.x, v0.y, v0.z],
                            [v1.x, v1.y, v1.z],
                            [v2.x, v2.y, v2.z],
                            span.to_degrees()));
                    }
                }
            }

            spans.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let n = spans.len();
            println!("  Angular span statistics ({} triangles):", n);
            println!("    min:    {:.1}°", spans[0].to_degrees());
            println!("    25%:    {:.1}°", spans[n/4].to_degrees());
            println!("    median: {:.1}°", spans[n/2].to_degrees());
            println!("    75%:    {:.1}°", spans[3*n/4].to_degrees());
            println!("    max:    {:.1}°", spans[n-1].to_degrees());

            let lt5 = spans.iter().filter(|&&s| s < 5.0_f64.to_radians()).count();
            let lt10 = spans.iter().filter(|&&s| s < 10.0_f64.to_radians()).count();
            let lt30 = spans.iter().filter(|&&s| s < 30.0_f64.to_radians()).count();
            let lt60 = spans.iter().filter(|&&s| s < 60.0_f64.to_radians()).count();
            let lt90 = spans.iter().filter(|&&s| s < 90.0_f64.to_radians()).count();
            let lt180 = spans.iter().filter(|&&s| s < 180.0_f64.to_radians()).count();
            let ge180 = spans.iter().filter(|&&s| s >= 180.0_f64.to_radians()).count();
            println!("    <5°:   {} (good)", lt5);
            println!("    <10°:  {} (good)", lt10);
            println!("    <30°:  {}", lt30);
            println!("    <60°:  {}", lt60);
            println!("    <90°:  {}", lt90);
            println!("    <180°: {}", lt180);
            println!("    >=180°: {} (twisted)", ge180);

            println!("\n  Triangles with span > 90°: {}", bad_count);
            if !examples.is_empty() {
                println!("\n  Examples:");
                for (i, p0, p1, p2, span_deg) in &examples {
                    let a0 = p0[2].atan2(p0[1]).to_degrees();
                    let a1 = p1[2].atan2(p1[1]).to_degrees();
                    let a2 = p2[2].atan2(p2[1]).to_degrees();
                    println!("    tri[{}] span={:.1}°:", i, span_deg);
                    println!("      v0=({:.2},{:.2},{:.2}) angle={:.1}°", p0[0], p0[1], p0[2], a0);
                    println!("      v1=({:.2},{:.2},{:.2}) angle={:.1}°", p1[0], p1[1], p1[2], a1);
                    println!("      v2=({:.2},{:.2},{:.2}) angle={:.1}°", p2[0], p2[1], p2[2], a2);
                }
            }

            // Also check normal direction — for forward=false, normals should
            // point INWARD (toward axis). Compute face normal from cross product
            // and check if it points outward or inward.
            println!("\n  Normal direction check (forward=false → should point inward):");
            let mut outward = 0;
            let mut inward = 0;
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

                // Outward = normal points away from axis (radially outward)
                // Radial direction at centroid:
                let cx = (v0.x + v1.x + v2.x) / 3.0;
                let cy = (v0.y + v1.y + v2.y) / 3.0;
                let cz = (v0.z + v1.z + v2.z) / 3.0;
                // For cone with axis (-1,0,0), radial = (0, y, z) normalized
                let dot = ny * cy + nz * cz;
                if dot > 0.0 { outward += 1; }
                else { inward += 1; }
            }
            println!("    outward normals: {}", outward);
            println!("    inward normals:  {} (correct for forward=false)", inward);
        }

        break;
    }
}
