// Check the 4 triangles that span the +z/-z boundary in Step#84
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

                // Find triangles that span the +z/-z boundary
                let mut spanning: Vec<(usize, [f64;9])> = Vec::new();
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];

                    let z_pos = [v0.z > 0.001, v1.z > 0.001, v2.z > 0.001];
                    let z_neg = [v0.z < -0.001, v1.z < -0.001, v2.z < -0.001];
                    let has_pos = z_pos.iter().any(|&b| b);
                    let has_neg = z_neg.iter().any(|&b| b);
                    if has_pos && has_neg {
                        spanning.push((i, [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z]));
                    }
                }

                println!("  Triangles spanning +z/-z: {}", spanning.len());
                for (i, pts) in &spanning {
                    let a0 = pts[2].atan2(pts[1]).to_degrees();
                    let a1 = pts[5].atan2(pts[4]).to_degrees();
                    let a2 = pts[8].atan2(pts[7]).to_degrees();
                    println!("    tri[{}]:", i);
                    println!("      v0=({:.2},{:.2},{:.2}) r={:.2} angle={:.1}°", pts[0], pts[1], pts[2], (pts[1]*pts[1]+pts[2]*pts[2]).sqrt(), a0);
                    println!("      v1=({:.2},{:.2},{:.2}) r={:.2} angle={:.1}°", pts[3], pts[4], pts[5], (pts[4]*pts[4]+pts[5]*pts[5]).sqrt(), a1);
                    println!("      v2=({:.2},{:.2},{:.2}) r={:.2} angle={:.1}°", pts[6], pts[7], pts[8], (pts[7]*pts[7]+pts[8]*pts[8]).sqrt(), a2);
                }

                // Also: check if any triangles are at angle ~0° or ~180° (the seam edges)
                // These would be triangles with a vertex at (x, +35.22, 0) or (x, -35.22, 0)
                let mut seam_tris: Vec<(usize, [f64;9])> = Vec::new();
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];
                    // Check if any vertex is at the seam (z ≈ 0, |y| > 30)
                    let at_seam = |v: &draper_geometry::Point3d| v.z.abs() < 0.5 && v.y.abs() > 30.0;
                    if at_seam(&v0) || at_seam(&v1) || at_seam(&v2) {
                        seam_tris.push((i, [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z]));
                    }
                }
                println!("\n  Triangles at seam (z≈0, |y|>30): {}", seam_tris.len());
                for (i, pts) in seam_tris.iter().take(10) {
                    let a0 = pts[2].atan2(pts[1]).to_degrees();
                    let a1 = pts[5].atan2(pts[4]).to_degrees();
                    let a2 = pts[8].atan2(pts[7]).to_degrees();
                    println!("    tri[{}]: angles=({:.1}°, {:.1}°, {:.1}°)", i, a0, a1, a2);
                    println!("      v0=({:.2},{:.2},{:.2})", pts[0], pts[1], pts[2]);
                    println!("      v1=({:.2},{:.2},{:.2})", pts[3], pts[4], pts[5]);
                    println!("      v2=({:.2},{:.2},{:.2})", pts[6], pts[7], pts[8]);
                }
            }
        }

        break;
    }
}
