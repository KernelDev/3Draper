// Diagnostic: dump sphere face boundary UVs
// Run with: cargo run --bin sphere_uv_dump -- test/nist_sphere.stp

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/nist_sphere.stp".to_string());
    println!("Loading: {}", path);
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    println!("Total BREP instances: {}", pending.len());

    let ctx = StepConversionContext::new(&step);
    let p = &pending[0];
    let result = ctx.triangulate_pending(p);
    let inst = match result {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); std::process::exit(2); }
    };

    println!("\n=== Instance: {} (BREP #{}) ===", inst.name, p.brep_id);
    println!("Vertices: {}, Triangles: {}", inst.mesh.vertex_count(), inst.mesh.triangle_count());

    // Per-face summary
    for (fi, face) in inst.faces.iter().enumerate() {
        println!("\nFace #{}: {} tris, surface={}, forward={}",
            fi, face.triangle_count(), face.surface_type, face.forward);

        println!("  Outer boundary polylines: {}", face.outer_boundary.len());
        for (li, line) in face.outer_boundary.iter().enumerate() {
            println!("    Line {}: {} points", li, line.len());
            if line.len() <= 20 {
                for (i, p) in line.iter().enumerate() {
                    println!("      [{}] ({:.4}, {:.4}, {:.4})", i, p.x, p.y, p.z);
                }
            } else {
                for i in 0..5 {
                    let p = &line[i];
                    println!("      [{}] ({:.4}, {:.4}, {:.4})", i, p.x, p.y, p.z);
                }
                println!("      ... ({} more)", line.len() - 10);
                for i in (line.len()-5)..line.len() {
                    let p = &line[i];
                    println!("      [{}] ({:.4}, {:.4}, {:.4})", i, p.x, p.y, p.z);
                }
            }
        }
        println!("  Outer UV boundary polylines: {}", face.outer_uv_boundary.len());
        for (li, line) in face.outer_uv_boundary.iter().enumerate() {
            println!("    Line {}: {} points", li, line.len());
            // Compute UV bbox
            let (umin, umax) = line.iter().fold((f64::MAX, f64::MIN), |(mn,mx), p| (mn.min(p.u), mx.max(p.u)));
            let (vmin, vmax) = line.iter().fold((f64::MAX, f64::MIN), |(mn,mx), p| (mn.min(p.v), mx.max(p.v)));
            println!("      UV bbox: u=[{:.4},{:.4}] v=[{:.4},{:.4}]", umin, umax, vmin, vmax);
            if line.len() <= 20 {
                for (i, p) in line.iter().enumerate() {
                    println!("      [{}] (u={:.4}, v={:.4})", i, p.u, p.v);
                }
            } else {
                for i in 0..5 {
                    let p = &line[i];
                    println!("      [{}] (u={:.4}, v={:.4})", i, p.u, p.v);
                }
                println!("      ... ({} more)", line.len() - 10);
                for i in (line.len()-5)..line.len() {
                    let p = &line[i];
                    println!("      [{}] (u={:.4}, v={:.4})", i, p.u, p.v);
                }
            }
        }
    }
}
