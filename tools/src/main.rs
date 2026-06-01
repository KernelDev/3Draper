//! Diagnostic tool for 3Draper — analyzes STEP files and tests hole cutting.

use draper_step::*;
use draper_geometry::*;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    
    if args.len() > 1 && args[1] == "--holes" {
        test_hole_cutting();
        return;
    }
    
    if args.len() < 2 {
        eprintln!("Usage: draper-diag <file.stp>");
        eprintln!("       draper-diag --holes");
        return;
    }

    let path = &args[1];
    let data = std::fs::read_to_string(path).expect("Failed to read file");
    let step = draper_step::parser::parse_step(&data).expect("Failed to parse STEP");

    println!("=== STEP File Parsed ===");
    println!("Total entities: {}", step.entities.len());

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

fn test_hole_cutting() {
    use draper_topology::ShapeBuilder;
    use draper_mesh::{triangulate_solid, TriangulationParams, cut_text_holes_in_mesh, TextSurface};
    
    env_logger::init();
    
    // Test 1: Box with hole on top face
    let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
    let params = TriangulationParams::default();
    let base_mesh = triangulate_solid(&solid, &params);
    eprintln!("Base mesh: {} vertices, {} triangles", base_mesh.vertex_count(), base_mesh.triangle_count());

    let mesh = cut_text_holes_in_mesh(
        &base_mesh,
        "3",
        &TextSurface::Plane { z: 30.0 },
        3.0,
        5.0,
        [0.15, 0.15, 0.2, 1.0],
    );
    eprintln!("Hole mesh: {} vertices, {} triangles", mesh.vertex_count(), mesh.triangle_count());

    if mesh.triangle_count() <= base_mesh.triangle_count() {
        eprintln!("BUG: Holes not being cut! Same or fewer triangles than base mesh.");
    } else {
        eprintln!("OK: Holes being cut — mesh has more triangles (hole insets added).");
    }
    
    // Count top face triangles before and after
    let base_top_tris: usize = base_mesh.triangles.iter().enumerate()
        .filter(|(_, tri)| {
            let z_avg = (base_mesh.vertices[tri[0] as usize].z + 
                        base_mesh.vertices[tri[1] as usize].z + 
                        base_mesh.vertices[tri[2] as usize].z) / 3.0;
            (z_avg - 30.0).abs() < 1.0
        })
        .count();
    eprintln!("Base top-face triangles (z≈30): {}", base_top_tris);
    
    // Test 2: Cylinder with hole
    let solid2 = ShapeBuilder::make_cylinder(40.0, 100.0);
    let base2 = triangulate_solid(&solid2, &params);
    eprintln!("\nCylinder base: {} vertices, {} triangles", base2.vertex_count(), base2.triangle_count());
    
    let mesh2 = cut_text_holes_in_mesh(
        &base2,
        "3",
        &TextSurface::Cylinder { radius: 40.0, height: 100.0 },
        2.5,
        5.0,
        [0.1, 0.15, 0.1, 1.0],
    );
    eprintln!("Cylinder hole mesh: {} vertices, {} triangles", mesh2.vertex_count(), mesh2.triangle_count());
    
    if mesh2.triangle_count() <= base2.triangle_count() {
        eprintln!("BUG: Cylinder holes not being cut!");
    } else {
        eprintln!("OK: Cylinder holes being cut.");
    }
}
