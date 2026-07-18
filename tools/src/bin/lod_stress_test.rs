// Test all STEP files at multiple LOD levels to find LOD-sensitive bugs
use draper_mesh::TriangulationParams;
use draper_step::{parser::parse_step, step_structure_lazy, OwnedStepConversionContext};
use std::collections::HashMap;

fn count_edges(mesh: &draper_mesh::TriangleMesh) -> (usize, usize) {
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        for i in 0..3 {
            let a = tri[i].min(tri[(i + 1) % 3]);
            let b = tri[i].max(tri[(i + 1) % 3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let boundary = edge_count.values().filter(|c| **c == 1).count();
    let total = edge_count.len();
    (boundary, total)
}

fn test_file_at_lod(path: &str, lod: f64) -> Result<(usize, usize, usize), String> {
    let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let step_file = parse_step(&content).map_err(|e| e.to_string())?;
    let (_tree, pending) = step_structure_lazy(&step_file);
    if pending.is_empty() {
        return Ok((0, 0, 0));
    }
    let params = TriangulationParams::for_lod(lod);
    let mut ctx = OwnedStepConversionContext::new_with_params(step_file, params);
    let mut total_tris = 0;
    let mut merged = draper_mesh::TriangleMesh::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            total_tris += inst.mesh.triangle_count();
            merged.merge(&inst.mesh);
        }
    }
    let (bnd, total) = count_edges(&merged);
    Ok((total_tris, bnd, total))
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let test_dir = std::env::args().nth(1).unwrap_or("test".to_string());
    let mut files: Vec<String> = Vec::new();
    if let Ok(entries) = std::fs::read_dir(&test_dir) {
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

    println!("Testing {} files at LOD 0.1, 0.5, 1.0", files.len());
    println!("{:<42} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}", "File", "Tris@0.1", "Bnd@0.1", "Tris@0.5", "Bnd@0.5", "Tris@1.0", "Bnd@1.0");
    println!("{}", "-".repeat(110));

    let lods = [0.1_f64, 0.5, 1.0];
    let mut total_tris = [0usize; 3];
    let mut total_bnd = [0usize; 3];
    let mut issues = Vec::new();

    for f in &files {
        let short = std::path::Path::new(f).file_name().and_then(|s| s.to_str()).unwrap_or(f);
        let mut results = [None, None, None];
        for (i, &lod) in lods.iter().enumerate() {
            match test_file_at_lod(f, lod) {
                Ok((tris, bnd, _total)) => {
                    total_tris[i] += tris;
                    total_bnd[i] += bnd;
                    results[i] = Some((tris, bnd));
                }
                Err(e) => {
                    eprintln!("ERROR {}: {}", short, e);
                    issues.push(format!("{}: ERROR at LOD {}: {}", short, lod, e));
                }
            }
        }

        let row: Vec<String> = results.iter().map(|r| {
            match r {
                Some((t, b)) => format!("{:>5}/{:<4}", t, b),
                None => "  ERR   ".to_string(),
            }
        }).collect();
        println!("{:<42} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
            short,
            row[0].split('/').next().unwrap_or("").trim(), row[0].split('/').nth(1).unwrap_or("").trim(),
            row[1].split('/').next().unwrap_or("").trim(), row[1].split('/').nth(1).unwrap_or("").trim(),
            row[2].split('/').next().unwrap_or("").trim(), row[2].split('/').nth(1).unwrap_or("").trim(),
        );

        // Check for issues
        if let (Some(r0), Some(r1), Some(r2)) = (&results[0], &results[1], &results[2]) {
            // Triangle count should increase with LOD
            if r0.0 > r1.0 || r1.0 > r2.0 {
                issues.push(format!("{}: non-monotonic triangle count: {} → {} → {}", short, r0.0, r1.0, r2.0));
            }
            // Boundary edges should not explode at higher LOD
            if r2.1 > r0.1 * 2 && r2.1 > 50 {
                issues.push(format!("{}: boundary edges grow with LOD: {} → {} → {}", short, r0.1, r1.1, r2.1));
            }
        }
    }

    println!("{}", "-".repeat(110));
    println!("{:<42} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}", "TOTAL",
        total_tris[0], total_bnd[0], total_tris[1], total_bnd[1], total_tris[2], total_bnd[2]);

    println!("\n=== Issues found: {} ===", issues.len());
    for issue in &issues {
        println!("  ⚠️  {}", issue);
    }
}
