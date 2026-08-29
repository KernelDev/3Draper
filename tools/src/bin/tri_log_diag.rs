// Diagnostic runner with env_logger enabled: reproduces triangulate_solid_with_report
// with full log output to see which code path each face takes.
// Run: RUST_LOG=info cargo run --release --bin tri_log_diag -- test/nist_cone.stp

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::triangulate_solid_with_report;
use draper_mesh::triangulate::TriangulationParams;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/nist_cone.stp".to_string()
    );
    println!("Loading: {}", path);
    let step = match parse_step_file(&path) {
        Ok(s) => s,
        Err(e) => { eprintln!("Parse error: {}", e); return; }
    };
    let (solids, _) = extract_solids(&step);
    println!("Solids: {}", solids.len());

    for (si, solid) in solids.iter().enumerate() {
        println!("\n=== Solid #{} ===", si);
        let params = TriangulationParams::default();
        let result = triangulate_solid_with_report(solid, &params);
        println!("Report: {}", result.report.summary());
    }
}
