// Diagnostic: dump Step#84 triangulation details for 3.05.078.stp
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;

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

        println!("\n=== BREP #{}: {} verts, {} tris ===",
            p.brep_id, inst.mesh.vertices.len(), inst.mesh.triangles.len());

        let report = validate_watertight(&inst.mesh, true);
        println!("  watertight: {}", report.is_watertight());
        println!("  boundary edges: {}", report.boundary_edge_count);

        for fi in &inst.faces {
            if fi.step_face_id != 84 { continue; }

            let (start, end) = fi.triangle_range;
            println!("\n=== Step#84 Cone forward={} ===", fi.forward);
            println!("  triangle_range: [{}, {})", start, end);
            println!("  UV triangles: {}", fi.uv_triangles.len());

            if let draper_geometry::Surface::Cone(ref cone) = fi.surface {
                println!("  Cone: radius={:.4}, half_angle={:.4}deg, expanding={}",
                    cone.radius, cone.half_angle.to_degrees(), cone.expanding);
                println!("  origin=({:.4},{:.4},{:.4})", cone.origin.x, cone.origin.y, cone.origin.z);
                println!("  axis=({:.4},{:.4},{:.4})", cone.axis.x, cone.axis.y, cone.axis.z);
                println!("  x_dir=({:.4},{:.4},{:.4})", cone.x_dir.x, cone.x_dir.y, cone.x_dir.z);
                println!("  apex_v={:.4}", cone.apex_v());
            }

            // Dump outer boundary
            for (pi, polyline) in fi.outer_boundary.iter().enumerate() {
                println!("\n  outer[{}] ({} pts):", pi, polyline.len());
                for (i, p) in polyline.iter().enumerate().take(8) {
                    let r = (p.y * p.y + p.z * p.z).sqrt();
                    println!("    [{}]: ({:.4},{:.4},{:.4}) r={:.4}", i, p.x, p.y, p.z, r);
                }
            }

            // Dump vertex distribution
            let mut x_min = f64::MAX;
            let mut x_max = f64::MIN;
            let mut r_at_x0: Vec<f64> = Vec::new();
            let mut r_at_x486: Vec<f64> = Vec::new();
            let mut other: usize = 0;

            let fids = inst.mesh.triangle_face_ids.as_ref();
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    let v = inst.mesh.vertices[vi as usize];
                    x_min = x_min.min(v.x);
                    x_max = x_max.max(v.x);
                    let r = (v.y * v.y + v.z * v.z).sqrt();
                    if v.x.abs() < 0.001 {
                        r_at_x0.push(r);
                    } else if (v.x - 4.86).abs() < 0.001 {
                        r_at_x486.push(r);
                    } else {
                        other += 1;
                    }
                }
            }

            println!("\n  Vertex x range: [{:.4}, {:.4}]", x_min, x_max);
            println!("  Vertices at x=0: {} (R should be 35.22)", r_at_x0.len());
            println!("  Vertices at x=4.86: {} (R should be 30.36)", r_at_x486.len());
            println!("  Vertices at other x: {}", other);

            if !r_at_x0.is_empty() {
                let r_min = r_at_x0.iter().fold(f64::MAX, |a, &b| a.min(b));
                let r_max = r_at_x0.iter().fold(f64::MIN, |a, &b| a.max(b));
                println!("  R at x=0: [{:.4}, {:.4}]", r_min, r_max);
            }
            if !r_at_x486.is_empty() {
                let r_min = r_at_x486.iter().fold(f64::MAX, |a, &b| a.min(b));
                let r_max = r_at_x486.iter().fold(f64::MIN, |a, &b| a.max(b));
                println!("  R at x=4.86: [{:.4}, {:.4}]", r_min, r_max);
            }

            // Sample 10 triangles
            let total = end - start;
            println!("\n  Sampled 3D triangles (every {}):", (total / 10).max(1));
            let step_n = (total / 10).max(1);
            for k in 0..10 {
                let i = start + k * step_n;
                if i >= end { break; }
                let tri = inst.mesh.triangles[i];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                let r0 = (v0.y * v0.y + v0.z * v0.z).sqrt();
                let r1 = (v1.y * v1.y + v1.z * v1.z).sqrt();
                let r2 = (v2.y * v2.y + v2.z * v2.z).sqrt();
                println!("    3D_tri[{}]: ({:.2},{:.2},{:.2})r={:.2}  ({:.2},{:.2},{:.2})r={:.2}  ({:.2},{:.2},{:.2})r={:.2}",
                    i,
                    v0.x, v0.y, v0.z, r0,
                    v1.x, v1.y, v1.z, r1,
                    v2.x, v2.y, v2.z, r2);
            }

            // Check for triangles outside the cone surface
            // Cone: x goes from 0 (R=35.22) to 4.86 (R=30.36)
            // At any x, R should be 35.22 - (35.22-30.36) * x/4.86 = 35.22 - 4.86*x/4.86 = 35.22 - x*tan(45) = 35.22 - x
            // Wait, that's for half_angle=45 with axis along x. Actually R(x) = 32.79 - (x-2.43)*tan(45) = 32.79 - x + 2.43 = 35.22 - x
            // At x=0: R=35.22 ✓
            // At x=4.86: R=30.36 ✓
            println!("\n  Triangle validity check (R should be 35.22 - x for cone surface):");
            let mut on_cone = 0;
            let mut off_cone = 0;
            let mut max_dev = 0.0f64;
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                for &vi in &tri {
                    let v = inst.mesh.vertices[vi as usize];
                    let r = (v.y * v.y + v.z * v.z).sqrt();
                    let expected_r = 35.22 - v.x;
                    let dev = (r - expected_r).abs();
                    if dev > max_dev { max_dev = dev; }
                    if dev < 0.01 {
                        on_cone += 1;
                    } else {
                        off_cone += 1;
                    }
                }
            }
            println!("    on cone (dev<0.01): {}", on_cone);
            println!("    off cone (dev>=0.01): {}", off_cone);
            println!("    max deviation: {:.6}", max_dev);
        }

        break;
    }
}
