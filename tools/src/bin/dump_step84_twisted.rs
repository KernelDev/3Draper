// Detailed check: for each triangle in Step#84, compute the angular range
// spanned by its 3 vertices. A "good" triangle should span a small angular
// range (<30°). A "twisted" triangle spans a large range (>90°) or crosses
// the seam.
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
            println!("  triangle_range: [{}, {})", start, end);

            // For each triangle, compute angles of its 3 vertices
            // Angle = atan2(z, y) on the yz-plane
            let angle = |y: f64, z: f64| z.atan2(y);

            let mut twisted_count = 0;
            let mut good_count = 0;
            let mut examples_twisted: Vec<(usize, [f64; 3], [f64; 3], [f64; 3])> = Vec::new();

            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];

                let a0 = angle(v0.y, v0.z);
                let a1 = angle(v1.y, v1.z);
                let a2 = angle(v2.y, v2.z);

                // Normalize to [0, 2π)
                let norm = |a: f64| ((a % (2.0 * std::f64::consts::PI)) + 2.0 * std::f64::consts::PI) % (2.0 * std::f64::consts::PI);
                let a0 = norm(a0);
                let a1 = norm(a1);
                let a2 = norm(a2);

                let angles = {
                    let mut v = vec![a0, a1, a2];
                    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    v
                };

                // Gaps between consecutive angles (including wraparound)
                let g1 = angles[1] - angles[0];
                let g2 = angles[2] - angles[1];
                let g3 = 2.0 * std::f64::consts::PI - (angles[2] - angles[0]);

                // Max gap — if the triangle spans a small angular range,
                // the max gap is large (the "outside" of the triangle's arc).
                // If twisted, the max gap is small (vertices spread around).
                let max_gap = g1.max(g2).max(g3);
                let max_gap_deg = max_gap.to_degrees();

                // A good triangle's max gap should be > 270° (vertices in a small arc)
                // A twisted triangle's max gap is < 180° (vertices spread around)
                if max_gap_deg < 180.0 {
                    twisted_count += 1;
                    if examples_twisted.len() < 10 {
                        examples_twisted.push((i,
                            [v0.x, v0.y, v0.z],
                            [v1.x, v1.y, v1.z],
                            [v2.x, v2.y, v2.z]));
                    }
                } else {
                    good_count += 1;
                }
            }

            println!("  Good triangles (max_gap > 180°): {}", good_count);
            println!("  Twisted triangles (max_gap < 180°): {}", twisted_count);

            if !examples_twisted.is_empty() {
                println!("\n  Twisted triangle examples:");
                for (i, p0, p1, p2) in &examples_twisted {
                    let a0 = p0[2].atan2(p0[1]).to_degrees();
                    let a1 = p1[2].atan2(p1[1]).to_degrees();
                    let a2 = p2[2].atan2(p2[1]).to_degrees();
                    println!("    tri[{}]:", i);
                    println!("      v0=({:.2},{:.2},{:.2}) angle={:.1}°", p0[0], p0[1], p0[2], a0);
                    println!("      v1=({:.2},{:.2},{:.2}) angle={:.1}°", p1[0], p1[1], p1[2], a1);
                    println!("      v2=({:.2},{:.2},{:.2}) angle={:.1}°", p2[0], p2[1], p2[2], a2);
                }
            }

            // Also check: how many triangles have all 3 vertices at the same x-level?
            // (bottom row, top row, or intermediate)
            let mut same_level = 0;
            let mut cross_level = 0;
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                let x_range = v0.x.max(v1.x).max(v2.x) - v0.x.min(v1.x).min(v2.x);
                if x_range < 0.001 {
                    same_level += 1;
                } else {
                    cross_level += 1;
                }
            }
            println!("\n  Triangles with all 3 vertices at same x: {}", same_level);
            println!("  Triangles spanning multiple x levels: {}", cross_level);
        }

        break;
    }
}
