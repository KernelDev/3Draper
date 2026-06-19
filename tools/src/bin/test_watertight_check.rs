//! Quick watertight check for specific files
use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let files: Vec<String> = if args.is_empty() {
        vec![
            "test/compressor-13920_top.stp".to_string(),
            "test/Zentralstaender.stp".to_string(),
            "test/3.05.078.stp".to_string(),
        ]
    } else {
        args
    };
    
    println!("{:<45} {:>6} {:>8} {:>14} {:>10}", "File", "BREPs", "Tris", "BndEdges", "Status");
    println!("{}", "-".repeat(95));
    
    for f in &files {
        let t0 = Instant::now();
        match test_file(f) {
            Ok((breps, tris, bnd, total)) => {
                let pct = if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 };
                let status = if bnd == 0 { "WATERTIGHT" } else if pct < 5.0 { "ok" } else if pct < 20.0 { "leaky" } else { "BAD" };
                println!("{:<45} {:>6} {:>8} {:>5}/{:<7} {:>5.1}%  {} ({:.1}s)",
                         f, breps, tris, bnd, total, pct, status, t0.elapsed().as_secs_f64());
            }
            Err(e) => {
                println!("{:<45} ERROR: {} ({:.1}s)", f, &e[..e.len().min(50)], t0.elapsed().as_secs_f64());
            }
        }
    }
}

fn test_file(path: &str) -> Result<(usize, usize, usize, usize), String> {
    let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let step_file = draper_step::parser::parse_step(&content).map_err(|e| e.to_string())?;
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    let breps = pending.len();
    if breps == 0 { return Ok((0, 0, 0, 0)); }
    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);
    let mut total_tris = 0;
    let mut merged = draper_mesh::TriangleMesh::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            total_tris += inst.mesh.triangle_count();
            merged.merge(&inst.mesh);
        }
    }
    let (bnd, total) = count_edges(&merged);
    Ok((breps, total_tris, bnd, total))
}

fn count_edges(mesh: &draper_mesh::TriangleMesh) -> (usize, usize) {
    use std::collections::HashMap;
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        for i in 0..3 {
            let a = tri[i].min(tri[(i+1)%3]);
            let b = tri[i].max(tri[(i+1)%3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let boundary = edge_count.values().filter(|c| **c == 1).count();
    let total = edge_count.len();
    (boundary, total)
}
