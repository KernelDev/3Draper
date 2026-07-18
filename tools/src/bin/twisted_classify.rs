// Distinguish twisted quads from apex fan triangles
// Apex fan triangles: one vertex is shared by MANY triangles (the apex)
// Twisted quads: pairs of triangles with bad aspect ratio but no shared apex
use draper_step::{parser::parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::TriangulationParams;
use std::collections::HashMap;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let path = std::env::args().nth(1).unwrap_or("test/3.05.078.stp".to_string());
    let content = std::fs::read_to_string(&path).expect("read file");
    let step_file = parse_step(&content).expect("parse STEP");
    let (_tree, pending) = step_structure_lazy(&step_file);

    let params = TriangulationParams::for_lod(1.0);
    let mut ctx = OwnedStepConversionContext::new_with_params(step_file, params);

    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let mesh = &inst.mesh;
            if mesh.triangle_count() == 0 { continue; }

            // Count how many triangles each vertex appears in
            let mut vertex_tri_count: Vec<usize> = vec![0; mesh.vertex_count()];
            for tri in &mesh.triangles {
                for &v in tri {
                    vertex_tri_count[v as usize] += 1;
                }
            }

            // Classify triangles
            let mut fan_triangles = 0;  // triangles with an apex vertex (high tri count)
            let mut twisted_quads = 0;  // high aspect ratio, no apex vertex
            let mut normal_thin = 0;    // high aspect ratio, no apex, but might be legitimate
            let mut total = 0;

            for tri in &mesh.triangles {
                let a = mesh.vertices[tri[0] as usize];
                let b = mesh.vertices[tri[1] as usize];
                let c = mesh.vertices[tri[2] as usize];
                let ab = a.distance_to(&b);
                let bc = b.distance_to(&c);
                let ac = a.distance_to(&c);
                let max_side = ab.max(bc).max(ac);
                let min_side = ab.min(bc).min(ac);

                if max_side > 0.0 && min_side > 0.0 {
                    let ratio = max_side / min_side;
                    if ratio > 10.0 {
                        // Check if any vertex is an "apex" (shared by many triangles)
                        let max_tri_count = tri.iter().map(|&v| vertex_tri_count[v as usize]).max().unwrap_or(0);
                        if max_tri_count > 20 {
                            fan_triangles += 1;
                        } else if max_tri_count > 8 {
                            twisted_quads += 1;
                        } else {
                            normal_thin += 1;
                        }
                    }
                    total += 1;
                }
            }

            println!("BREP {}: {} tris, {} verts", i, mesh.triangle_count(), mesh.vertex_count());
            println!("  High aspect ratio (>10:1): {}", fan_triangles + twisted_quads + normal_thin);
            println!("    Fan triangles (apex, >20 tris/vertex): {} ({:.1}%)", fan_triangles, 100.0 * fan_triangles as f64 / total as f64);
            println!("    Twisted quads (>8 tris/vertex): {} ({:.1}%)", twisted_quads, 100.0 * twisted_quads as f64 / total as f64);
            println!("    Other thin (<8 tris/vertex): {} ({:.1}%)", normal_thin, 100.0 * normal_thin as f64 / total as f64);
        }
    }
}
