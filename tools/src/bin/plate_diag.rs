// Detailed diagnostic for STEP#3084 (plate top face with 3 holes)
use draper_step::{parser::parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::TriangulationParams;
use std::collections::HashMap;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let path = "test/as1-oc-214.stp";
    let content = std::fs::read_to_string(path).expect("read");
    let step_file = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step_file);

    // Find the plate BREP (BREP#1934)
    let plate_pending = pending.iter().find(|p| p.brep_id == 1934);
    if plate_pending.is_none() {
        eprintln!("BREP#1934 not found in pending list!");
        return;
    }
    let plate_pending = plate_pending.unwrap();

    println!("Found BREP#1934: name='{}'", plate_pending.name);

    let params = TriangulationParams::for_lod(1.0);
    let mut ctx = OwnedStepConversionContext::new_with_params(step_file.clone(), params);

    let start = std::time::Instant::now();
    if let Some(inst) = ctx.triangulate_pending(plate_pending) {
        let elapsed = start.elapsed();
        let mesh = &inst.mesh;
        println!("Plate: {} tris, {} verts, {:?}", mesh.triangle_count(), mesh.vertex_count(), elapsed);

        // Count boundary edges
        let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &mesh.triangles {
            for i in 0..3 {
                let a = tri[i].min(tri[(i + 1) % 3]);
                let b = tri[i].max(tri[(i + 1) % 3]);
                *edge_count.entry((a, b)).or_insert(0) += 1;
            }
        }
        let boundary = edge_count.values().filter(|c| **c == 1).count();
        let non_manifold = edge_count.values().filter(|c| **c > 2).count();
        let total_edges = edge_count.len();
        println!("  edges: {} total, {} boundary, {} non-manifold", total_edges, boundary, non_manifold);
    }
}
