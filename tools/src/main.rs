//! Diagnostic tool for 3Draper — analyzes STEP files and prints per-face info.

use draper_step::*;
use draper_geometry::*;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("Usage: draper-diag <file.stp>");

    let data = std::fs::read_to_string(path).expect("Failed to read file");
    let step = draper_step::parser::parse_step(&data).expect("Failed to parse STEP");

    println!("=== STEP File Parsed ===");
    println!("Total entities: {}", step.entities.len());

    // Get detailed instances - this will also trigger the eprintln! debug logging
    let instances = step_to_detailed_instances(&step).unwrap_or_default();
    println!("\n=== Detailed Instances: {} ===", instances.len());

    for (ii, inst) in instances.iter().enumerate() {
        println!("\n--- Instance #{}: {} (BREP #{}) ---", ii, inst.name, inst.brep_id);
        println!("  Vertices: {}  Triangles: {}", inst.mesh.vertex_count(), inst.mesh.triangle_count());

        for (fi, face) in inst.faces.iter().enumerate() {
            let tris = face.triangle_range.1 - face.triangle_range.0;
            let status = if tris == 0 { " *** EMPTY ***" } else { "" };

            println!("  Face #{} [F#{}]: {} step_id={} forward={} tris={}{}",
                fi + 1, face.face_id, face.surface_type, face.step_face_id, face.forward, tris, status);

            // Print UV bounds
            let mut u_min = f64::MAX;
            let mut u_max = f64::MIN;
            let mut v_min = f64::MAX;
            let mut v_max = f64::MIN;
            for uv_loop in &face.outer_uv_boundary {
                for pt in uv_loop {
                    u_min = u_min.min(pt.u);
                    u_max = u_max.max(pt.u);
                    v_min = v_min.min(pt.v);
                    v_max = v_max.max(pt.v);
                }
            }
            if u_min < f64::MAX {
                println!("    UV bounds: U: {:.4}..{:.4}  V: {:.4}..{:.4}", u_min, u_max, v_min, v_max);
            }

            // Print boundary points
            for (li, loop_pts) in face.outer_boundary.iter().enumerate() {
                print!("    Outer loop {}: {} pts: ", li, loop_pts.len());
                for (i, p) in loop_pts.iter().enumerate() {
                    if i < 5 || i > loop_pts.len() - 3 {
                        print!("({:.3},{:.3},{:.3})", p.x, p.y, p.z);
                    } else if i == 5 {
                        print!("...");
                    }
                }
                println!();
            }

            // Print surface details
            match &face.surface {
                Surface::Plane(p) => {
                    println!("    Plane: origin=({:.3},{:.3},{:.3}) normal=({:.3},{:.3},{:.3})",
                        p.origin.x, p.origin.y, p.origin.z,
                        p.normal.x, p.normal.y, p.normal.z);
                }
                Surface::Cylinder(c) => {
                    println!("    Cylinder: origin=({:.3},{:.3},{:.3}) axis=({:.3},{:.3},{:.3}) r={:.4}",
                        c.origin.x, c.origin.y, c.origin.z,
                        c.axis.x, c.axis.y, c.axis.z, c.radius);
                }
                _ => {}
            }
        }
    }
}
