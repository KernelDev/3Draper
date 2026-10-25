//! Test all STEP files for opening + triangulation
//! Verifies that every STEP file in test/ opens, parses, and converts to a watertight mesh.
//!
//! s110: PER-FILE SUBPROCESS ISOLATION (default). The s105 archaeology
//! proved that per-file numbers measured in ONE process are
//! order-dependent: the Hamiltonian-path solver's process-global step
//! budget (GLOBAL_STEPS, parametric_domain.rs — 80M steps shared across
//! ALL faces of ALL files, never reset) is progressively consumed by
//! earlier files, so a late-alphabet file (transmission_top.stp) can
//! flip emission outcomes depending on what earlier files consumed —
//! the s105 "trans +255" was exactly this artifact (sphere band ON
//! changed Vulcan/comp/drill consumption; trans itself has ZERO sphere
//! grid calls; trans-first and trans-only runs give bit-identical
//! 63421 for both band states at the s105 commit). Default mode now
//! spawns `--single <file>` per file so each number is measured with a
//! fresh process state. DRAPPER_ALLFILES_INPROC=1 restores the legacy
//! in-process loop (comparison/forensics only).

use std::io::Read;
use std::process::Command;

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // Child mode: `all_files_test --single <path>` — one file, one
    // machine-readable result line, fresh process state.
    if let Some(pos) = args.iter().position(|a| a == "--single") {
        if let Some(path) = args.get(pos + 1) {
            env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
            match test_file(path) {
                Ok((breps, tris, bnd, total)) => {
                    let pct = if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 };
                    let status = if bnd == 0 { "WATERTIGHT" } else if pct < 5.0 { "ok" } else if pct < 20.0 { "leaky" } else { "BAD" };
                    println!("RESULT {} {} {} {} {} {:.1}", breps, tris, bnd, total, status, pct);
                }
                Err(e) => {
                    println!("ERROR {}", e.replace('\n', " "));
                }
            }
            return;
        }
    }

    // Parent mode (default): enumerate + spawn a child per file.
    let inproc = std::env::var("DRAPPER_ALLFILES_INPROC").as_deref() == Ok("1");

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
        // (breps, tris, bnd, total, status) — status computed by the
        // same rule in both modes.
        let outcome: Result<(usize, usize, usize, usize, String), String> = if inproc {
            test_file(f).map(|(breps, tris, bnd, total)| {
                let pct = if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 };
                let status = if bnd == 0 { "WATERTIGHT".to_string() } else if pct < 5.0 { "ok".to_string() } else if pct < 20.0 { "leaky".to_string() } else { "BAD".to_string() };
                (breps, tris, bnd, total, status)
            })
        } else {
            run_single(f)
        };
        match outcome {
            Ok((breps, tris, bnd, total, status)) => {
                let pct = if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 };
                match status.as_str() {
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
    println!("Summary: {} ok, {} leaky, {} BAD, {} errors ({} total){}",
             n_ok, n_leaky, n_bad, n_err, files.len(),
             if inproc { " [in-process legacy mode]" } else { " [per-file isolated]" });
}

/// Spawn `current_exe --single <path>` and parse the RESULT/ERROR line.
/// stdout is piped (we parse it); stderr is INHERITED so debug-log
/// lines (RUST_LOG) and the `=== FILE:` markers still reach the
/// parent's stderr in order.
fn run_single(path: &str) -> Result<(usize, usize, usize, usize, String), String> {
    use std::process::Stdio;
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    let mut child = Command::new(exe)
        .arg("--single")
        .arg(path)
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| e.to_string())?;
    let mut stdout = String::new();
    if let Some(mut pipe) = child.stdout.take() {
        use std::io::Read;
        pipe.read_to_string(&mut stdout).map_err(|e| e.to_string())?;
    }
    let status = child.wait().map_err(|e| e.to_string())?;
    for line in stdout.lines() {
        if let Some(rest) = line.strip_prefix("RESULT ") {
            let parts: Vec<&str> = rest.split_whitespace().collect();
            if parts.len() >= 5 {
                let breps: usize = parts[0].parse().map_err(|_| "bad RESULT line".to_string())?;
                let tris: usize = parts[1].parse().map_err(|_| "bad RESULT line".to_string())?;
                let bnd: usize = parts[2].parse().map_err(|_| "bad RESULT line".to_string())?;
                let total: usize = parts[3].parse().map_err(|_| "bad RESULT line".to_string())?;
                let status = parts[4].to_string();
                return Ok((breps, tris, bnd, total, status));
            }
        }
        if let Some(rest) = line.strip_prefix("ERROR ") {
            return Err(rest.to_string());
        }
    }
    Err(format!("no RESULT line from child (exit={:?})", status.code()))
}

fn test_file(path: &str) -> Result<(usize, usize, usize, usize), String> {
    // stderr file marker: attributes debug-log lines to files in both
    // modes (children inherit the parent's stderr).
    eprintln!("=== FILE: {}", path);
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
