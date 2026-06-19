//! Quick test: triangulate transmission_top.stp and count boundary edges
use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    let path = "test/transmission_top.stp";
    let t0 = Instant::now();
    
    let content = std::fs::read_to_string(path).expect("read");
    let step_file = draper_step::parser::parse_step(&content).expect("parse");
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    let breps = pending.len();
    println!("BREPs: {}", breps);
    
    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);
    let mut total_tris = 0;
    let mut merged = draper_mesh::TriangleMesh::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            total_tris += inst.mesh.triangle_count();
            merged.merge(&inst.mesh);
        }
    }
    
    // Count boundary edges
    use std::collections::HashMap;
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &merged.triangles {
        for i in 0..3 {
            let a = tri[i].min(tri[(i+1)%3]);
            let b = tri[i].max(tri[(i+1)%3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let boundary = edge_count.values().filter(|c| **c == 1).count();
    let total = edge_count.len();
    let pct = if total > 0 { 100.0 * boundary as f64 / total as f64 } else { 0.0 };
    let status = if boundary == 0 { "WATERTIGHT" } else if pct < 5.0 { "ok" } else if pct < 20.0 { "leaky" } else { "BAD" };
    
    println!("{:<35} {:>6} {:>8} {:>5}/{:<7} {:>5.1}%  {} (elapsed {:.1}s)",
             path, breps, total_tris, boundary, total, pct, status, t0.elapsed().as_secs_f64());
}
