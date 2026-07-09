//! Benchmark runner for STEP-file triangulation pipelines.
//!
//! Inspired by the Autodesk Benchmark Collection (ABC) — a standard suite of
//! ~100 STEP files used to evaluate CAD geometry kernels. Since the full ABC
//! dataset is ~1 GB and licensing requires attribution, this tool runs the
//! same benchmark protocol on whatever STEP files are present in `test/`.
//!
//! ## Usage
//!
//! ```bash
//! # Benchmark all files in test/
//! cargo run --release --bin benchmark
//!
//! # Benchmark a specific directory (e.g., downloaded ABC subset)
//! cargo run --release --bin benchmark -- /path/to/abc_subset
//!
//! # Save results as CSV for tracking regressions over time
//! cargo run --release --bin benchmark -- --csv benchmark_results.csv
//! ```
//!
//! ## Output
//!
//! Per-file: BREP count, triangle count, boundary edge %, status, timing.
//! Aggregate: watertight rate, ok rate, leaky rate, BAD rate, total time,
//! triangles/sec, file count by status category.
//!
//! ## Adding the ABC dataset
//!
//! 1. Clone the ABC dataset from https://github.com/deepgeometry/abc-data
//!    (or download the STEP-file subset from https://abc-technology.org/)
//! 2. Place .stp / .step files in a directory (e.g., `test/abc/`)
//! 3. Run: `cargo run --release --bin benchmark -- test/abc/`
//! 4. Compare results against the baseline recorded in this repo's
//!    `docs/benchmark_baseline.md` (TBD).
//!
//! ## Status categories
//!
//! - WATERTIGHT: 0 boundary edges (perfect closed manifold)
//! - ok: <5% boundary edges (acceptable for visualization)
//! - leaky: 5-20% boundary edges (visible holes)
//! - BAD: >20% boundary edges (broken mesh)
//! - ERROR: parse/conversion failure

use std::collections::HashMap;
use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let args: Vec<String> = std::env::args().collect();
    let mut test_dir = "test".to_string();
    let mut csv_path: Option<String> = None;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--csv" => {
                i += 1;
                if i < args.len() {
                    csv_path = Some(args[i].clone());
                }
            }
            "--help" | "-h" => {
                println!("Usage: {} [test_dir] [--csv out.csv]", args[0]);
                println!("  test_dir: directory containing .stp/.step files (default: test)");
                println!("  --csv: write per-file results to CSV");
                return;
            }
            _ => {
                test_dir = args[i].clone();
            }
        }
        i += 1;
    }

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

    if files.is_empty() {
        eprintln!("No .stp/.step files found in {}", test_dir);
        std::process::exit(1);
    }

    println!("Benchmarking {} files from {}", files.len(), test_dir);
    println!("{:<48} {:>6} {:>8} {:>14} {:>8} {:>8}", "File", "BREPs", "Tris", "BndEdges", "Time(s)", "Status");
    println!("{}", "-".repeat(100));

    let mut results: Vec<FileResult> = Vec::new();
    let mut status_counts: HashMap<&'static str, usize> = HashMap::new();
    let mut total_tris = 0usize;
    let mut total_bnd = 0usize;
    let mut total_edges = 0usize;
    let overall_start = Instant::now();

    for f in &files {
        let file_start = Instant::now();
        let result = test_file(f);
        let elapsed = file_start.elapsed().as_secs_f64();

        match &result {
            Ok((breps, tris, bnd, total)) => {
                let pct = if *total > 0 { 100.0 * *bnd as f64 / *total as f64 } else { 0.0 };
                let status = if *bnd == 0 { "WATERTIGHT" }
                    else if pct < 5.0 { "ok" }
                    else if pct < 20.0 { "leaky" }
                    else { "BAD" };
                *status_counts.entry(status).or_insert(0) += 1;
                total_tris += tris;
                total_bnd += bnd;
                total_edges += total;
                println!("{:<48} {:>6} {:>8} {:>5}/{:<7} {:>5.1}% {:>7.2} {:>8}",
                    short_path(f), breps, tris, bnd, total, pct, elapsed, status);
                results.push(FileResult {
                    file: f.clone(),
                    breps: *breps,
                    tris: *tris,
                    bnd: *bnd,
                    total: *total,
                    elapsed,
                    status,
                });
            }
            Err(e) => {
                *status_counts.entry("ERROR").or_insert(0) += 1;
                let _short = if e.len() > 40 { format!("{}...", &e[..40]) } else { e.clone() };
                println!("{:<48} {:>6} {:>8} {:>14} {:>7.2} {:>8}", short_path(f), "-", "-", "-", elapsed, "ERROR");
                results.push(FileResult {
                    file: f.clone(),
                    breps: 0, tris: 0, bnd: 0, total: 0,
                    elapsed, status: "ERROR",
                });
            }
        }
    }

    let overall_elapsed = overall_start.elapsed().as_secs_f64();
    println!("{}", "-".repeat(100));
    println!("\n=== Aggregate ===");
    let n_files = files.len();
    let n_watertight = *status_counts.get("WATERTIGHT").unwrap_or(&0);
    let n_ok = *status_counts.get("ok").unwrap_or(&0);
    let n_leaky = *status_counts.get("leaky").unwrap_or(&0);
    let n_bad = *status_counts.get("BAD").unwrap_or(&0);
    let n_err = *status_counts.get("ERROR").unwrap_or(&0);
    let n_pass = n_watertight + n_ok;
    println!("Files:           {}", n_files);
    println!("WATERTIGHT:      {} ({:.1}%)", n_watertight, 100.0 * n_watertight as f64 / n_files as f64);
    println!("ok (<5% bnd):    {} ({:.1}%)", n_ok, 100.0 * n_ok as f64 / n_files as f64);
    println!("leaky (5-20%):   {} ({:.1}%)", n_leaky, 100.0 * n_leaky as f64 / n_files as f64);
    println!("BAD (>20%):      {} ({:.1}%)", n_bad, 100.0 * n_bad as f64 / n_files as f64);
    println!("ERROR:           {} ({:.1}%)", n_err, 100.0 * n_err as f64 / n_files as f64);
    println!("PASS RATE:       {}/{} ({:.1}%)", n_pass, n_files, 100.0 * n_pass as f64 / n_files as f64);
    println!();
    println!("Total triangles: {}", total_tris);
    println!("Total edges:     {} ({} boundary, {:.2}% overall)",
        total_edges, total_bnd, if total_edges > 0 { 100.0 * total_bnd as f64 / total_edges as f64 } else { 0.0 });
    println!("Total time:      {:.2}s", overall_elapsed);
    if overall_elapsed > 0.0 {
        println!("Throughput:      {:.0} triangles/sec, {:.2} files/sec",
            total_tris as f64 / overall_elapsed,
            n_files as f64 / overall_elapsed);
    }

    // Write CSV if requested
    if let Some(csv_path) = csv_path {
        match write_csv(&csv_path, &results) {
            Ok(_) => println!("\nCSV written to {}", csv_path),
            Err(e) => eprintln!("Failed to write CSV: {}", e),
        }
    }
}

struct FileResult {
    file: String,
    breps: usize,
    tris: usize,
    bnd: usize,
    total: usize,
    elapsed: f64,
    status: &'static str,
}

fn short_path(path: &str) -> &str {
    // Show just filename + immediate parent dir
    if let Some(idx) = path.rfind('/') {
        let parent_end = idx;
        if let Some(parent_start) = path[..parent_end].rfind('/') {
            &path[parent_start + 1..]
        } else {
            &path[..]
        }
    } else {
        path
    }
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
            let a = tri[i].min(tri[(i + 1) % 3]);
            let b = tri[i].max(tri[(i + 1) % 3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let boundary = edge_count.values().filter(|c| **c == 1).count();
    let total = edge_count.len();
    (boundary, total)
}

fn write_csv(path: &str, results: &[FileResult]) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::fs::File::create(path)?;
    writeln!(f, "file,breps,triangles,boundary_edges,total_edges,boundary_pct,elapsed_s,status")?;
    for r in results {
        let pct = if r.total > 0 { 100.0 * r.bnd as f64 / r.total as f64 } else { 0.0 };
        writeln!(f, "{},{},{},{},{},{:.4},{},{}",
            r.file, r.breps, r.tris, r.bnd, r.total, pct, r.elapsed, r.status)?;
    }
    Ok(())
}
