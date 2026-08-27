// Diagnostic: investigate thin_annulus hang
// Run: cargo run -p draper-diag --release --bin annulus_diag -- test/synthetic/synth_thin_annulus.stp
//
// This runs with a 30-second timeout per face triangulation to identify
// which face causes the infinite loop.

use draper_step::{parse_step_file, extract_solids};
use draper_geometry::Surface;
use std::time::Instant;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/synthetic/synth_thin_annulus.stp".to_string()
    );
    println!("Loading: {}", path);
    let step = match parse_step_file(&path) {
        Ok(s) => { println!("Parse OK"); s },
        Err(e) => { eprintln!("Parse error: {}", e); return; }
    };
    println!("Extracting solids...");
    let (solids, _) = extract_solids(&step);
    println!("Solids: {}", solids.len());

    for (si, solid) in solids.iter().enumerate() {
        println!("\n=== Solid #{} ===", si);
        println!("Faces: {}", solid.faces().len());

        for (fi, face) in solid.faces().iter().enumerate() {
            let surf_name = face.surface.as_ref().map(|s| {
                match s {
                    Surface::Plane(_) => "Plane",
                    Surface::Sphere(_) => "Sphere",
                    Surface::Cylinder(_) => "Cylinder",
                    Surface::Cone(_) => "Cone",
                    Surface::Torus(_) => "Torus",
                    Surface::Nurbs(_) => "Nurbs",
                    Surface::Revolution(_) => "Revolution",
                    Surface::Extrusion(_) => "Extrusion",
                    _ => "Other",
                }
            }).unwrap_or("None");
            println!("\nFace {}: surface={}, edges={}", fi, surf_name, face.edges.len());

            // Print edge details
            for (ei, edge) in face.edges.iter().enumerate() {
                let edge_type = edge.curve.as_ref().map(|c| {
                    match c {
                        draper_geometry::Curve3d::Line(_) => "Line",
                        draper_geometry::Curve3d::Circle(_) => "Circle",
                        draper_geometry::Curve3d::Nurbs(_) => "Nurbs",
                        _ => "Other",
                    }
                }).unwrap_or("None");
                println!("    Edge {}: type={} param_range=({:.4},{:.4})",
                    ei, edge_type, edge.param_range.0, edge.param_range.1);
            }
        }
    }
}
