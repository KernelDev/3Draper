//! Test the slow NURBS-heavy files (drill_top, transmission_top)
//! These are known to take ~90s each due to NURBS projection

use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    
    let files: Vec<&str> = vec![
        "test/drill_top.stp",
        "test/transmission_top.stp",
    ];
    
    println!("{:<35} {:>6} {:>8} {:>8}", "File", "BREPs", "Tris", "Time_s");
    println!("{}", "-".repeat(65));
    
    let mut all_ok = true;
    for f in &files {
        let t0 = Instant::now();
        let result = test_file(f);
        let elapsed = t0.elapsed().as_secs_f64();
        match result {
            Ok((breps, tris)) => {
                println!("{:<35} {:>6} {:>8} {:>8.2}", f, breps, tris, elapsed);
            }
            Err(e) => {
                println!("{:<35} {:>6} {:>8} {:>8.2}  ERROR: {}", f, "-", "-", elapsed, e);
                all_ok = false;
            }
        }
    }
    
    println!("\n=== Summary ===");
    if all_ok {
        println!("✓ All slow files opened successfully");
    } else {
        println!("✗ Some slow files failed");
        std::process::exit(1);
    }
}

fn test_file(path: &str) -> Result<(usize, usize), String> {
    let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let step_file = draper_step::parser::parse_step(&content).map_err(|e| e.to_string())?;
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    let breps = pending.len();
    if breps == 0 {
        return Ok((0, 0));
    }
    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);
    let mut total_tris = 0;
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            total_tris += inst.mesh.triangle_count();
        }
    }
    Ok((breps, total_tris))
}
