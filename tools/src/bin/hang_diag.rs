//! Diagnose which BREP instance hangs during triangulation.
//! Uses the lazy API to parse + build tree first, then triangulates one BREP at a time.

use std::time::Instant;

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    
    println!("Reading {}...", path);
    let content = std::fs::read_to_string(&path).expect("read step file");
    
    print!("Parsing STEP... ");
    let t0 = Instant::now();
    let step_file = draper_step::parser::parse_step(&content).expect("parse step file");
    println!("OK ({:.2}s, {} entities)", t0.elapsed().as_secs_f64(), step_file.entities.len());
    
    print!("Building lazy structure... ");
    let t1 = Instant::now();
    let (tree, pending) = draper_step::step_structure_lazy(&step_file);
    println!("OK ({:.2}s, {} pending BREPs)", t1.elapsed().as_secs_f64(), pending.len());
    
    // Show tree structure
    fn show_tree(node: &draper_step::AssemblyNode, depth: usize) {
        let indent = "  ".repeat(depth);
        let brep_str = match node.brep_id {
            Some(id) => format!(" BREP=#{}", id),
            None => String::new(),
        };
        let inst_str = match node.instance_index {
            Some(idx) => format!(" inst={}", idx),
            None => String::new(),
        };
        println!("{}{} (PD=#{}){}{}", indent, node.name, node.pd_id, brep_str, inst_str);
        for child in &node.children {
            show_tree(child, depth + 1);
        }
    }
    show_tree(&tree, 0);
    
    // Triangulate one at a time with timeout
    println!("\n=== Triangulating {} BREPs ===", pending.len());
    for (i, p) in pending.iter().enumerate() {
        print!("[{}/{}] Triangulating '{}' (BREP #{})... ", i+1, pending.len(), p.name, p.brep_id);
        let t2 = Instant::now();
        
        // Use a thread with timeout
        let result = std::thread::scope(|s| {
            let step_file_ref = &step_file;
            let pending_ref = p;
            let handle = s.spawn(move || {
                draper_step::triangulate_pending_instance(step_file_ref, pending_ref)
            });
            // Wait up to 30 seconds
            match handle.join() {
                Ok(Some(inst)) => {
                    let elapsed = t2.elapsed().as_secs_f64();
                    if inst.mesh.triangle_count() == 0 && inst.mesh.vertex_count() == 0 {
                        println!("EMPTY ({:.2}s)", elapsed);
                    } else {
                        println!("OK ({:.2}s, {}v, {}t, {} faces)", 
                            elapsed,
                            inst.mesh.vertex_count(),
                            inst.mesh.triangle_count(),
                            inst.faces.len());
                        // Show per-face info
                        for (fi, face) in inst.faces.iter().enumerate() {
                            let tris = face.triangle_range.1 - face.triangle_range.0;
                            if tris > 1000 {
                                println!("    Face #{}: {} tris — {} *** MANY TRIANGLES ***", fi+1, tris, face.surface_type);
                            } else if tris == 0 {
                                println!("    Face #{}: 0 tris — {} *** EMPTY ***", fi+1, face.surface_type);
                            }
                        }
                    }
                }
                Ok(None) => {
                    println!("FAILED ({:.2}s)", t2.elapsed().as_secs_f64());
                }
                Err(_) => {
                    println!("PANIC");
                }
            }
        });
        
        if t2.elapsed().as_secs() > 30 {
            println!("  TIMEOUT! Stopping.");
            break;
        }
    }
}
