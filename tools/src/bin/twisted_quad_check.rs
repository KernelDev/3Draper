// Check if twisted quads are fixed on 3.05.078.stp cone (Step#78/Step#84)
// The cone has bottom ring R=30.36 and top ring R=35.22 from different
// CIRCLE entities. Previously, different angular samplings caused twisted quads.
// After variant A fix (pre_compute_circle_axis_n), both rings should have
// the same n → rectangular quads.
use draper_mesh::TriangulationParams;
use draper_step::{parser::parse_step, step_structure_lazy, OwnedStepConversionContext};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let path = std::env::args().nth(1).unwrap_or("test/3.05.078.stp".to_string());
    let content = std::fs::read_to_string(&path).expect("read file");
    let step_file = parse_step(&content).expect("parse STEP");
    let (_tree, pending) = step_structure_lazy(&step_file);

    println!("File: {} ({} BREPs)", path, pending.len());

    for &lod in &[0.5_f64, 1.0] {
        let params = TriangulationParams::for_lod(lod);
        let mut ctx = OwnedStepConversionContext::new_with_params(step_file.clone(), params);

        for (i, p) in pending.iter().enumerate() {
            if let Some(inst) = ctx.triangulate_pending(p) {
                let mesh = &inst.mesh;
                if mesh.triangle_count() == 0 {
                    continue;
                }

                // Check for twisted quads: a twisted quad has 2 triangles
                // where the diagonal is longer than the sides. We check
                // by looking at triangle aspect ratios.
                let mut twisted = 0;
                let mut total_checked = 0;
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
                            twisted += 1;
                        }
                        total_checked += 1;
                    }
                }

                // Count boundary edges
                let mut edge_count: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
                for tri in &mesh.triangles {
                    for i in 0..3 {
                        let a = tri[i].min(tri[(i + 1) % 3]);
                        let b = tri[i].max(tri[(i + 1) % 3]);
                        *edge_count.entry((a, b)).or_insert(0) += 1;
                    }
                }
                let boundary = edge_count.values().filter(|c| **c == 1).count();
                let non_manifold = edge_count.values().filter(|c| **c > 2).count();

                if twisted > 0 || boundary > 0 {
                    println!(
                        "LOD {:.1} BREP {}: {} tris, {} verts, {} twisted ({:.1}%), {} bnd, {} non-manifold",
                        lod, i, mesh.triangle_count(), mesh.vertex_count(),
                        twisted, 100.0 * twisted as f64 / total_checked as f64,
                        boundary, non_manifold
                    );
                }
            }
        }
    }
    println!("Done.");
}
