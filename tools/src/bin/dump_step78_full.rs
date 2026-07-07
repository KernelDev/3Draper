// Detailed Step#78 triangulation dump — check actual 3D structure
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
            if fi.step_face_id != 78 { continue; }

            let (start, end) = fi.triangle_range;
            println!("\n=== Step#78 Cone forward={} ===", fi.forward);
            println!("  triangle_range: [{}, {})", start, end);
            println!("  UV triangles: {}", fi.uv_triangles.len());

            if let draper_geometry::Surface::Cone(ref cone) = fi.surface {
                println!("  Cone: radius={:.4}, half_angle={:.4}deg",
                    cone.radius, cone.half_angle.to_degrees());
                println!("  origin=({:.4},{:.4},{:.4})", cone.origin.x, cone.origin.y, cone.origin.z);
                println!("  axis=({:.4},{:.4},{:.4})", cone.axis.x, cone.axis.y, cone.axis.z);
                println!("  x_dir=({:.4},{:.4},{:.4})", cone.x_dir.x, cone.x_dir.y, cone.x_dir.z);
            }

            // Dump ALL 252 triangles with full 3D coords
            println!("\n  All {} triangles:", end - start);
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                let r0 = (v0.y * v0.y + v0.z * v0.z).sqrt();
                let r1 = (v1.y * v1.y + v1.z * v1.z).sqrt();
                let r2 = (v2.y * v2.y + v2.z * v2.z).sqrt();
                let a0 = v0.z.atan2(v0.y).to_degrees();
                let a1 = v1.z.atan2(v1.y).to_degrees();
                let a2 = v2.z.atan2(v2.y).to_degrees();
                println!("    tri[{}]: ({:.3},{:.3},{:.3})r={:.3}a={:.1}°  ({:.3},{:.3},{:.3})r={:.3}a={:.1}°  ({:.3},{:.3},{:.3})r={:.3}a={:.1}°",
                    i,
                    v0.x, v0.y, v0.z, r0, a0,
                    v1.x, v1.y, v1.z, r1, a1,
                    v2.x, v2.y, v2.z, r2, a2);
            }

            // Triangle area stats
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
            println!("\n  Area stats: min={:.4} med={:.4} max={:.4}",
                areas[0], areas[areas.len()/2], areas[areas.len()-1]);
        }

        break;
    }
}
