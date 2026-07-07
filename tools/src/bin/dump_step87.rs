// Diagnostic: dump ALL triangles for Step#87 in 3.05.078.stp
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("Failed to read 3.05.078.stp");
    let step = parse_step(&content).expect("parse step file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => continue,
        };

        for fi in &inst.faces {
            if fi.step_face_id != 87 { continue; }

            let (start, end) = fi.triangle_range;
            println!("\n=== Step#87 Plane forward={} ===", fi.forward);
            println!("  triangle_range: [{}, {})", start, end);
            println!("  UV triangles: {}", fi.uv_triangles.len());
            println!("  Surface: {:?}", fi.surface_type);

            // Dump plane info
            if let draper_geometry::Surface::Plane(ref plane) = fi.surface {
                println!("  Plane origin: ({:?}, {:?}, {:?})", plane.origin.x, plane.origin.y, plane.origin.z);
                println!("  Plane u_dir:   ({:?}, {:?}, {:?})", plane.u_dir.x, plane.u_dir.y, plane.u_dir.z);
                println!("  Plane v_dir:   ({:?}, {:?}, {:?})", plane.v_dir.x, plane.v_dir.y, plane.v_dir.z);
                println!("  Plane normal:  ({:?}, {:?}, {:?})", plane.normal.x, plane.normal.y, plane.normal.z);
            }

            // Dump UV boundary (outer + inner)
            println!("\n  UV outer boundary ({} polylines):", fi.outer_boundary.len());
            for (pi, polyline) in fi.outer_boundary.iter().enumerate() {
                println!("    Polyline {} ({} pts):", pi, polyline.len());
                for (i, p) in polyline.iter().enumerate().take(5) {
                    println!("      [{}]: ({:.4}, {:.4}, {:.4})", i, p.x, p.y, p.z);
                }
                if polyline.len() > 5 {
                    println!("      ... ({} total)", polyline.len());
                }
            }

            println!("\n  UV inner boundaries: {} loops", fi.inner_boundaries.len());
            for (li, loop_pts) in fi.inner_boundaries.iter().enumerate() {
                println!("    Loop {} ({} pts):", li, loop_pts.len());
                for (i, p) in loop_pts.iter().enumerate().take(5) {
                    println!("      [{}]: ({:.4}, {:.4}, {:.4})", i, p.x, p.y, p.z);
                }
                if loop_pts.len() > 5 {
                    println!("      ... ({} total)", loop_pts.len());
                }
            }

            // Dump first 5 UV triangles (Point2d)
            println!("\n  First 5 UV triangles:");
            for (i, tri) in fi.uv_triangles.iter().enumerate().take(5) {
                println!("    uv_tri[{}]: ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                    i, tri[0].u, tri[0].v, tri[1].u, tri[1].v, tri[2].u, tri[2].v);
            }

            // Compute radius of each 3D vertex from yz-plane origin (since the annulus is centered at (0,0,0) in yz)
            let mut outer_count = 0;
            let mut inner_count = 0;
            let mut other_count = 0;
            let mut radii = Vec::new();
            for i in start..end {
                let tri = inst.mesh.triangles[i];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];

                // Distance from (0,0,0) in yz-plane
                let r0 = (v0.y * v0.y + v0.z * v0.z).sqrt();
                let r1 = (v1.y * v1.y + v1.z * v1.z).sqrt();
                let r2 = (v2.y * v2.y + v2.z * v2.z).sqrt();
                radii.push((r0, r1, r2));

                let avg_r = (r0 + r1 + r2) / 3.0;
                if (avg_r - 37.5).abs() < 0.5 {
                    outer_count += 1;
                } else if (avg_r - 35.22).abs() < 0.5 {
                    inner_count += 1;
                } else {
                    other_count += 1;
                }
            }
            println!("\n  Triangle distribution by avg radius:");
            println!("    ~37.5 (outer): {}", outer_count);
            println!("    ~35.22 (inner): {}", inner_count);
            println!("    other (annulus fill): {}", other_count);

            // Sample 10 triangles across the range
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

            // Check 3D vertex radii distribution
            let mut r_hist = std::collections::HashMap::<u32, usize>::new();
            let mut vs_seen = std::collections::HashSet::new();
            for i in start..end {
                for &vi in &inst.mesh.triangles[i] {
                    if vs_seen.insert(vi) {
                        let v = inst.mesh.vertices[vi as usize];
                        let r = (v.y * v.y + v.z * v.z).sqrt();
                        let bucket = (r * 10.0).round() as u32; // 0.1mm buckets
                        *r_hist.entry(bucket).or_insert(0) += 1;
                    }
                }
            }
            println!("\n  Vertex radius histogram (0.1mm buckets):");
            let mut sorted: Vec<_> = r_hist.into_iter().collect();
            sorted.sort_by_key(|(b, _)| *b);
            for (bucket, count) in sorted {
                let r = bucket as f64 / 10.0;
                println!("    r={:.1}mm: {} vertices", r, count);
            }
        }
    }
}
