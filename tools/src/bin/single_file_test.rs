//! Test single STEP file for watertightness — used for quick iteration.

use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <step_file>", args[0]);
        std::process::exit(1);
    }
    let path = &args[1];

    let t0 = Instant::now();
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("ERROR: cannot read {}: {}", path, e);
            std::process::exit(2);
        }
    };

    let step_file = match draper_step::parser::parse_step(&content) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("ERROR: parse failed: {}", e);
            std::process::exit(3);
        }
    };

    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    let breps = pending.len();
    if breps == 0 {
        println!("{}: 0 BREP, 0 tris, 0.0s", path);
        return;
    }

    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);
    let mut total_tris = 0;
    let mut merged = draper_mesh::TriangleMesh::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            total_tris += inst.mesh.triangle_count();
            merged.merge(&inst.mesh);
        }
    }

    let elapsed = t0.elapsed().as_secs_f64();
    let (bnd, total) = count_edges(&merged);
    let pct = if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 };
    let status = if bnd == 0 { "WATERTIGHT" } else if pct < 5.0 { "ok" } else if pct < 20.0 { "leaky" } else { "BAD" };

    println!(
        "{:<50} {:>4} BREP {:>8} tris {:>6}/{:<6} ({:.2}%) {} in {:.2}s",
        path, breps, total_tris, bnd, total, pct, status, elapsed
    );
}

fn count_edges(mesh: &draper_mesh::TriangleMesh) -> (usize, usize) {
    use std::collections::HashMap;
    let mut edge_count: HashMap<[u32; 2], usize> = HashMap::new();
    for tri in &mesh.triangles {
        for i in 0..3 {
            let a = tri[i];
            let b = tri[(i + 1) % 3];
            let key = if a < b { [a, b] } else { [b, a] };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }
    let total = edge_count.len();
    let bnd = edge_count.values().filter(|c| **c == 1).count();
    (bnd, total)
}
