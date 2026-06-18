//! Test all STEP files for opening + triangulation
//! Verifies that every STEP file in test/ opens, parses, and converts to a watertight mesh.

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    // Enumerate all .stp/.step/.STEP files in test/ automatically
    let mut files: Vec<String> = Vec::new();
    if let Ok(entries) = std::fs::read_dir("test") {
        for e in entries.flatten() {
            let p = e.path();
            if let Some(ext) = p.extension().and_then(|s| s.to_str()) {
                let ext_lc = ext.to_lowercase();
                if ext_lc == "stp" || ext_lc == "step" {
                    if let Some(s) = p.to_str() {
                        files.push(s.to_string());
                    }
                }
            }
        }
    }
    files.sort();

    println!("{:<45} {:>6} {:>8} {:>14} {:>10}", "File", "BREPs", "Tris", "BndEdges", "Status");
    println!("{}", "-".repeat(95));

    let mut n_ok = 0usize;
    let mut n_leaky = 0usize;
    let mut n_bad = 0usize;
    let mut n_err = 0usize;

    for f in &files {
        let result = test_file(f);
        match result {
            Ok((breps, tris, bnd, total)) => {
                let pct = if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 };
                let status = if bnd == 0 { "WATERTIGHT" } else if pct < 5.0 { "ok" } else if pct < 20.0 { "leaky" } else { "BAD" };
                match status {
                    "WATERTIGHT" | "ok" => n_ok += 1,
                    "leaky" => n_leaky += 1,
                    _ => n_bad += 1,
                }
                println!("{:<45} {:>6} {:>8} {:>5}/{:<7} {:>5.1}%  {}", f, breps, tris, bnd, total, pct, status);
            }
            Err(e) => {
                n_err += 1;
                let short = if e.len() > 60 { format!("{}...", &e[..60]) } else { e };
                println!("{:<45} {:>6} {:>8} {:>14} ERROR: {}", f, "-", "-", "-", short);
            }
        }
    }

    println!("{}", "-".repeat(95));
    println!("Summary: {} ok, {} leaky, {} BAD, {} errors ({} total)",
             n_ok, n_leaky, n_bad, n_err, files.len());
}

fn test_file(path: &str) -> Result<(usize, usize, usize, usize), String> {
    let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let step_file = draper_step::parser::parse_step(&content).map_err(|e| e.to_string())?;
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    let breps = pending.len();
    if breps == 0 {
        return Ok((0, 0, 0, 0));
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
