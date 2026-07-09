// Check: is Step#84 a half-cone (partial wrap, π) or full cone (2π)?
// The boundary has 2 CIRCLE edges + 2 LINE edges.
// - CIRCLE #271 (R=30.36) at axis (4.86,0,0), axis (-1,0,0)
// - CIRCLE #272 (R=35.22) at axis (0,0,0), axis (1,0,0)
// - LINE #249: from (0,35.22,0) to (4.86,30.36,0)
// - LINE #253: from (4.86,-30.36,0) to (0,-35.22,0)
//
// The two lines are at angle 0° and 180° — so this is a HALF cone (π wrap).
// But the diagnostic shows angles 0° to 180° — so only half.
//
// Question: is the triangulation covering the correct half?
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

            // Check which half is covered: +z half (angles 0° to 180°) or -z half?
            let mut pos_z_count = 0;
            let mut neg_z_count = 0;
            let mut pos_y_count = 0;
            let mut neg_y_count = 0;
            let mut both_signs_z = 0;
            let mut both_signs_y = 0;
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                let z_signs = [v0.z.signum(), v1.z.signum(), v2.z.signum()];
                let y_signs = [v0.y.signum(), v1.y.signum(), v2.y.signum()];
                let z_pos = z_signs.iter().filter(|&&s| s > 0.0).count();
                let z_neg = z_signs.iter().filter(|&&s| s < 0.0).count();
                let y_pos = y_signs.iter().filter(|&&s| s > 0.0).count();
                let y_neg = y_signs.iter().filter(|&&s| s < 0.0).count();
                if z_pos > 0 { pos_z_count += 1; }
                if z_neg > 0 { neg_z_count += 1; }
                if z_pos > 0 && z_neg > 0 { both_signs_z += 1; }
                if y_pos > 0 { pos_y_count += 1; }
                if y_neg > 0 { neg_y_count += 1; }
                if y_pos > 0 && y_neg > 0 { both_signs_y += 1; }
            }
            println!("  Triangles with +z vertex: {}", pos_z_count);
            println!("  Triangles with -z vertex: {}", neg_z_count);
            println!("  Triangles with BOTH +z and -z: {} (should be 0 for half-cone)", both_signs_z);
            println!("  Triangles with +y vertex: {}", pos_y_count);
            println!("  Triangles with -y vertex: {}", neg_y_count);
            println!("  Triangles with BOTH +y and -y: {}", both_signs_y);

            // Dump the boundary edges to see the actual boundary
            println!("\n  Boundary points (from face_info):");
            for (pi, polyline) in fi.outer_boundary.iter().enumerate() {
                println!("    outer[{}] ({} pts):", pi, polyline.len());
                let mut angles: Vec<f64> = Vec::new();
                for p in polyline {
                    let r = (p.y * p.y + p.z * p.z).sqrt();
                    let a = p.z.atan2(p.y).to_degrees();
                    angles.push(a);
                }
                // Show angle range
                let min_a = angles.iter().fold(f64::MAX, |a, &b| a.min(b));
                let max_a = angles.iter().fold(f64::MIN, |a, &b| a.max(b));
                println!("      angle range: [{:.1}°, {:.1}°]", min_a, max_a);
                // Show first 5 and last 5
                for (i, p) in polyline.iter().enumerate().take(5) {
                    let r = (p.y * p.y + p.z * p.z).sqrt();
                    let a = p.z.atan2(p.y).to_degrees();
                    println!("      [{}]: ({:.2},{:.2},{:.2}) r={:.2} angle={:.1}°", i, p.x, p.y, p.z, r, a);
                }
                println!("      ...");
                let n = polyline.len();
                for (i, p) in polyline.iter().enumerate().skip(n.saturating_sub(5)) {
                    let r = (p.y * p.y + p.z * p.z).sqrt();
                    let a = p.z.atan2(p.y).to_degrees();
                    println!("      [{}]: ({:.2},{:.2},{:.2}) r={:.2} angle={:.1}°", i, p.x, p.y, p.z, r, a);
                }
            }

            // Compare with Step#78 (the other cone face) — should cover the OTHER half
            if let Some(f78) = inst.faces.iter().find(|f| f.step_face_id == 78) {
                let (s, e) = f78.triangle_range;
                println!("\n  Step#78 (other cone) for comparison:");
                let mut angles78: Vec<f64> = Vec::new();
                for i in s..e {
                    let tri = inst.mesh.triangles[i];
                    for &vi in &tri {
                        let v = inst.mesh.vertices[vi as usize];
                        let a = v.z.atan2(v.y).to_degrees();
                        angles78.push(a);
                    }
                }
                let min_a = angles78.iter().fold(f64::MAX, |a, &b| a.min(b));
                let max_a = angles78.iter().fold(f64::MIN, |a, &b| a.max(b));
                println!("    angle range: [{:.1}°, {:.1}°]", min_a, max_a);
                println!("    forward={}", f78.forward);
            }
        }

        break;
    }
}
