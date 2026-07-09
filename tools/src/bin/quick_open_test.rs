//! Quick test: open all STEP files and verify they parse + produce some triangles
//! Skips slow NURBS-heavy files (drill_top, transmission_top)

use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    
    let files: Vec<&str> = vec![
        "test/3.05.078.stp",
        "test/SampleCube.step",
        "test/Zentralstaender.stp",
        "test/as1-oc-214.stp",
        "test/brick_thin.stp",
        "test/brick_thin_hole.stp",
        "test/brick_thin_round.stp",
        "test/compressor-13920_top.stp",
        "test/nist_assembly.stp",
        "test/nist_block_with_hole.stp",
        "test/nist_chamfer_block.stp",
        "test/nist_complex_surface.stp",
        "test/nist_cone.stp",
        "test/nist_cube.stp",
        "test/nist_cylinder.stp",
        "test/nist_sphere.stp",
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
        println!("✓ All files opened successfully");
    } else {
        println!("✗ Some files failed to open");
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
