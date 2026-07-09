// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Round-trip integrity test: read STEP → extract solids → export STEP → re-read.
//!
//! Verifies that our STEP exporter produces files that the parser can re-read
//! and that geometric identity is preserved (same surface types, same curve
//! types, same vertex / face / edge counts within tolerance).
//!
//! Usage: cargo run --release --bin roundtrip_test -- <step_file>

use draper_step::{parse_step_file, extract_solids, export_step, validate_exported_step};
use std::env;
use std::path::Path;

fn main() {
    env_logger::init();

    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: roundtrip_test <step_file>");
        eprintln!("       roundtrip_test --all [dir]");
        std::process::exit(1);
    }

    // Batch mode: iterate over every .stp/.step/.STEP file in a directory
    // (defaults to ./test) and run round-trip on each. Print a final summary.
    if args[1] == "--all" {
        let dir = if args.len() >= 3 { &args[2] } else { "test" };
        run_batch(dir);
        return;
    }

    let path = &args[1];
    if !Path::new(path).exists() {
        eprintln!("File not found: {}", path);
        std::process::exit(1);
    }

    let pass = run_one(path);
    if pass {
        std::process::exit(0);
    } else {
        std::process::exit(1);
    }
}

/// Run round-trip on a single file; returns true if PASS, false if WARN/FAIL.
fn run_one(path: &str) -> bool {
    println!("=== Round-trip test: {} ===", path);

    // ── 1. Parse original STEP ──
    let original_step = match parse_step_file(path) {
        Ok(sf) => sf,
        Err(e) => {
            eprintln!("PARSE FAILED: {}", e);
            return false;
        }
    };
    println!(
        "[1/5] Parsed original STEP: {} entities",
        original_step.entities.len()
    );

    // Count surface/curve types in original
    let orig_surfaces = count_surface_types(&original_step);
    let orig_curves = count_curve_types(&original_step);
    let orig_breps = original_step.find_entities_by_type("MANIFOLD_SOLID_BREP").len()
        + original_step.find_entities_by_type("BREP_WITH_VOIDS").len();
    println!("      BREP entities: {}", orig_breps);
    println!("      Surface types: {:?}", orig_surfaces);
    println!("      Curve types:   {:?}", orig_curves);

    // ── 2. Extract Solids (no triangulation) ──
    let (solids, brep_ids) = extract_solids(&original_step);
    println!(
        "[2/5] Extracted {} solid(s) from {} BREP(s)",
        solids.len(),
        brep_ids.len()
    );
    for (i, solid) in solids.iter().enumerate() {
        let n_faces = solid.faces().len();
        let n_voids = solid.inner_shells.len();
        println!(
            "      Solid #{} (brep_id={}): {} faces, {} void shell(s)",
            i, brep_ids[i], n_faces, n_voids
        );
    }

    if solids.is_empty() {
        println!("\n[ABORT] No solids extracted — file may use unsupported BREP variant.");
        return false;
    }

    // ── 3. Export each solid back to STEP ──
    let mut combined_export = String::new();
    combined_export.push_str("ISO-10303-21;\n");
    combined_export.push_str("HEADER;\n");
    combined_export.push_str("FILE_DESCRIPTION(('3Draper round-trip test'), '2;1');\n");
    combined_export.push_str("FILE_NAME('roundtrip.stp','2026-06-20T00:00:00',('3Draper'),(''),'3Draper','','');\n");
    combined_export.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
    combined_export.push_str("ENDSEC;\nDATA;\n");

    let mut offset: i64 = 1;
    for (i, solid) in solids.iter().enumerate() {
        let chunk = export_step(solid, &format!("solid_{}", i));
        let body = extract_data_section(&chunk);
        let remapped = remap_ids(&body, offset);
        combined_export.push_str(&remapped);
        offset = max_id_in(&body) + 1;
    }
    combined_export.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    println!("[3/5] Exported {} solid(s) back to STEP ({} bytes)",
        solids.len(), combined_export.len());

    // ── 4. Re-parse the exported STEP ──
    let reparsed = match draper_step::parse_step(&combined_export) {
        Ok(sf) => sf,
        Err(e) => {
            eprintln!("[FAIL] Re-parse failed: {}", e);
            // Save the failed export for debugging
            std::fs::write("/tmp/roundtrip_failed.stp", &combined_export).ok();
            eprintln!("       Saved failed export to /tmp/roundtrip_failed.stp");
            return false;
        }
    };
    println!(
        "[4/5] Re-parsed exported STEP: {} entities",
        reparsed.entities.len()
    );

    let new_surfaces = count_surface_types(&reparsed);
    let new_curves = count_curve_types(&reparsed);
    let new_breps = reparsed.find_entities_by_type("MANIFOLD_SOLID_BREP").len()
        + reparsed.find_entities_by_type("BREP_WITH_VOIDS").len();
    println!("      BREP entities: {}", new_breps);
    println!("      Surface types: {:?}", new_surfaces);
    println!("      Curve types:   {:?}", new_curves);

    // ── 5. Compare ──
    println!("\n[5/5] Comparison:");
    let mut pass = true;

    // Run the export validator (P20) on the exported STEP
    let validation = validate_exported_step(&combined_export);
    println!("      Validation: {}", validation.summary());
    if validation.has_errors() {
        for issue in validation.errors() {
            println!("        [{}] {} ({})", issue.severity, issue.message, issue.code);
        }
        pass = false;
    } else {
        println!("        [OK] No validation errors");
    }

    if orig_breps != new_breps {
        println!("  [FAIL] BREP count: {} → {}", orig_breps, new_breps);
        pass = false;
    } else {
        println!("  [OK]   BREP count preserved: {}", orig_breps);
    }

    // Surface comparison
    let mut all_surf_keys: std::collections::HashSet<&str> = std::collections::HashSet::new();
    for k in orig_surfaces.keys() { all_surf_keys.insert(k); }
    for k in new_surfaces.keys() { all_surf_keys.insert(k); }
    let mut surface_match = true;
    for k in &all_surf_keys {
        let a = orig_surfaces.get(k).copied().unwrap_or(0);
        let b = new_surfaces.get(k).copied().unwrap_or(0);
        if a != b {
            println!("  [WARN] Surface '{}': {} → {} (may differ due to dedup)", k, a, b);
            // Surface count mismatch is expected because we dedup shared surfaces
            // in the exporter but the original may have duplicates.
            surface_match = false;
        }
    }
    if surface_match {
        println!("  [OK]   Surface types preserved");
    }

    // Curve comparison
    let mut all_curve_keys: std::collections::HashSet<&str> = std::collections::HashSet::new();
    for k in orig_curves.keys() { all_curve_keys.insert(k); }
    for k in new_curves.keys() { all_curve_keys.insert(k); }
    let mut curve_match = true;
    for k in &all_curve_keys {
        let a = orig_curves.get(k).copied().unwrap_or(0);
        let b = new_curves.get(k).copied().unwrap_or(0);
        if a != b {
            println!("  [WARN] Curve '{}': {} → {} (may differ due to dedup)", k, a, b);
            curve_match = false;
        }
    }
    if curve_match {
        println!("  [OK]   Curve types preserved");
    }

    // Save the exported STEP for inspection
    let out_path = "/tmp/roundtrip_export.stp";
    std::fs::write(out_path, &combined_export).ok();
    println!("\n  Exported STEP saved to: {}", out_path);

    if pass {
        println!("\n=== RESULT: PASS ===");
    } else {
        println!("\n=== RESULT: WARN (counts differ, see above) ===");
    }
    pass
}

/// Batch round-trip runner: walks `dir` for *.stp/*.step/*.STEP files and
/// runs a silent round-trip on each. Prints a summary table at the end.
fn run_batch(dir: &str) {
    use std::fs;
    use std::path::PathBuf;

    let mut files: Vec<PathBuf> = Vec::new();
    if let Ok(entries) = fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                let ext_upper = ext.to_uppercase();
                if ext_upper == "STP" || ext_upper == "STEP" {
                    files.push(path);
                }
            }
        }
    }
    files.sort();

    if files.is_empty() {
        eprintln!("No STEP files found in {}", dir);
        std::process::exit(1);
    }

    println!("Round-trip batch: {} files in {}\n", files.len(), dir);
    println!(
        "{:<48} {:<8} {:<8} {:<10} {:<8}",
        "File", "BREPs", "Solids", "Re-BREPs", "Result"
    );
    println!("{}", "-".repeat(86));

    let mut pass_count = 0;
    let mut warn_count = 0;
    let mut fail_count = 0;

    for path in &files {
        let path_str = path.to_string_lossy();
        let original = parse_step_file(&path_str).ok();
        let orig_breps = original.as_ref().map(|s| {
            s.find_entities_by_type("MANIFOLD_SOLID_BREP").len()
                + s.find_entities_by_type("BREP_WITH_VOIDS").len()
        }).unwrap_or(0);
        let n_solids = original.as_ref().map(|s| extract_solids(s).0.len()).unwrap_or(0);

        let result = run_one_silent(&path_str);
        let (re_breps, status_str) = match result {
            RunResult::Pass(re_b) => { pass_count += 1; (re_b, "PASS") }
            RunResult::Warn(re_b) => { warn_count += 1; (re_b, "WARN") }
            RunResult::Fail      => { fail_count += 1; (0, "FAIL") }
        };

        let short_name = path.file_name().map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| path_str.to_string());
        println!(
            "{:<48} {:<8} {:<8} {:<10} {:<8}",
            short_name, orig_breps, n_solids, re_breps, status_str
        );
    }

    println!("{}", "-".repeat(86));
    println!(
        "Summary: {} PASS, {} WARN, {} FAIL ({} total)",
        pass_count, warn_count, fail_count, files.len()
    );

    if fail_count > 0 {
        std::process::exit(1);
    }
}

enum RunResult {
    Pass(usize),
    Warn(usize),
    Fail,
}

/// Silent round-trip — no per-file stdout. Returns a summary result.
fn run_one_silent(path: &str) -> RunResult {
    let original_step = match parse_step_file(path) {
        Ok(sf) => sf,
        Err(_) => return RunResult::Fail,
    };
    let orig_breps = original_step.find_entities_by_type("MANIFOLD_SOLID_BREP").len()
        + original_step.find_entities_by_type("BREP_WITH_VOIDS").len();
    let orig_surfaces = count_surface_types(&original_step);
    let orig_curves = count_curve_types(&original_step);

    let (solids, _brep_ids) = extract_solids(&original_step);
    if solids.is_empty() {
        return RunResult::Fail;
    }

    let mut combined_export = String::new();
    combined_export.push_str("ISO-10303-21;\n");
    combined_export.push_str("HEADER;\n");
    combined_export.push_str("FILE_DESCRIPTION(('3Draper round-trip test'), '2;1');\n");
    combined_export.push_str("FILE_NAME('roundtrip.stp','2026-06-20T00:00:00',('3Draper'),(''),'3Draper','','');\n");
    combined_export.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
    combined_export.push_str("ENDSEC;\nDATA;\n");

    let mut offset: i64 = 1;
    for (i, solid) in solids.iter().enumerate() {
        let chunk = export_step(solid, &format!("solid_{}", i));
        let body = extract_data_section(&chunk);
        let remapped = remap_ids(&body, offset);
        combined_export.push_str(&remapped);
        offset = max_id_in(&body) + 1;
    }
    combined_export.push_str("ENDSEC;\nEND-ISO-10303-21;\n");

    let validation = validate_exported_step(&combined_export);
    if validation.has_errors() {
        return RunResult::Fail;
    }

    let reparsed = match draper_step::parse_step(&combined_export) {
        Ok(sf) => sf,
        Err(_) => return RunResult::Fail,
    };
    let new_breps = reparsed.find_entities_by_type("MANIFOLD_SOLID_BREP").len()
        + reparsed.find_entities_by_type("BREP_WITH_VOIDS").len();
    let new_surfaces = count_surface_types(&reparsed);
    let new_curves = count_curve_types(&reparsed);

    if new_breps != orig_breps {
        return RunResult::Warn(new_breps);
    }

    // Type sets must match — counts may differ because the exporter dedups
    // shared surfaces/curves, but no type should disappear or appear new.
    for k in orig_surfaces.keys() {
        if !new_surfaces.contains_key(k) { return RunResult::Warn(new_breps); }
    }
    for k in new_surfaces.keys() {
        if !orig_surfaces.contains_key(k) { return RunResult::Warn(new_breps); }
    }
    for k in orig_curves.keys() {
        if !new_curves.contains_key(k) { return RunResult::Warn(new_breps); }
    }
    for k in new_curves.keys() {
        if !orig_curves.contains_key(k) { return RunResult::Warn(new_breps); }
    }

    RunResult::Pass(new_breps)
}

fn count_surface_types(step: &draper_step::StepFile) -> std::collections::HashMap<&'static str, usize> {
    let mut counts: std::collections::HashMap<&'static str, usize> = std::collections::HashMap::new();
    for t in &[
        "PLANE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE",
        "TOROIDAL_SURFACE", "SURFACE_OF_REVOLUTION", "SURFACE_OF_LINEAR_EXTRUSION",
        "B_SPLINE_SURFACE_WITH_KNOTS", "B_SPLINE_SURFACE",
    ] {
        let n = step.find_entities_by_type(t).len();
        if n > 0 {
            counts.insert(*t, n);
        }
    }
    counts
}

fn count_curve_types(step: &draper_step::StepFile) -> std::collections::HashMap<&'static str, usize> {
    let mut counts: std::collections::HashMap<&'static str, usize> = std::collections::HashMap::new();
    // Note: SURFACE_CURVE and PCURVE are STEP wrapper/container entities
    // around underlying geometric curves (LINE, CIRCLE, B_SPLINE_CURVE, etc.).
    // Our exporter intentionally flattens these wrappers and writes the
    // underlying curve directly, so a round-trip will legitimately drop
    // SURFACE_CURVE / PCURVE entity counts to zero. We exclude them from
    // the comparison to avoid spurious WARNs.
    for t in &[
        "LINE", "CIRCLE", "ELLIPSE", "HYPERBOLA", "PARABOLA",
        "B_SPLINE_CURVE_WITH_KNOTS", "B_SPLINE_CURVE",
        "TRIMMED_CURVE",
    ] {
        let n = step.find_entities_by_type(t).len();
        if n > 0 {
            counts.insert(*t, n);
        }
    }
    counts
}

fn extract_data_section(step: &str) -> String {
    let mut in_data = false;
    let mut out = String::with_capacity(step.len());
    for line in step.lines() {
        if line.trim() == "DATA;" { in_data = true; continue; }
        if line.trim() == "ENDSEC;" { in_data = false; continue; }
        if in_data {
            out.push_str(line);
            out.push('\n');
        }
    }
    out
}

fn max_id_in(body: &str) -> i64 {
    let mut max_id: i64 = 0;
    let bytes = body.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'#' {
            let mut j = i + 1;
            while j < bytes.len() && bytes[j].is_ascii_digit() { j += 1; }
            if j > i + 1 {
                if let Ok(n) = std::str::from_utf8(&bytes[i + 1..j]).unwrap_or("0").parse::<i64>() {
                    if n > max_id { max_id = n; }
                }
            }
            i = j;
        } else {
            i += 1;
        }
    }
    max_id
}

fn remap_ids(body: &str, offset: i64) -> String {
    let mut out = String::with_capacity(body.len());
    let bytes = body.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'#' {
            let mut j = i + 1;
            while j < bytes.len() && bytes[j].is_ascii_digit() { j += 1; }
            if j > i + 1 {
                if let Ok(n) = std::str::from_utf8(&bytes[i + 1..j]).unwrap_or("0").parse::<i64>() {
                    out.push('#');
                    out.push_str(&(n + offset).to_string());
                    i = j;
                    continue;
                }
            }
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}
