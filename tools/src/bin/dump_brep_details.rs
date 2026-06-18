//! Dump per-BREP details for a STEP file with multiple BREPs
//! Usage: dump_brep_details <file.stp> [brep_index]

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <file.stp> [brep_index]", args[0]);
        std::process::exit(1);
    }
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let path = &args[1];
    let target_brep_idx: Option<usize> = args.get(2).and_then(|s| s.parse().ok());

    let content = std::fs::read_to_string(path).expect("read");
    let step_file = draper_step::parser::parse_step(&content).expect("parse");
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);

    println!("File: {}", path);
    println!("BREPs: {}", pending.len());

    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);

    for (idx, p) in pending.iter().enumerate() {
        if let Some(t) = target_brep_idx {
            if idx != t { continue; }
        }

        if let Some(inst) = ctx.triangulate_pending(p) {
            let (bnd, total) = count_edges(&inst.mesh);
            let pct = if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 };
            println!("\n=== BREP#{} (idx {}) ===", p.brep_id, idx);
            println!("  Tris: {}  Verts: {}  Bnd: {}/{} ({:.1}%)",
                inst.mesh.triangle_count(),
                inst.mesh.vertex_count(),
                bnd, total, pct);

            if bnd > 0 && bnd <= 50 {
                println!("  Boundary edges:");
                let edges = boundary_edges(&inst.mesh);
                for (i, (a, b)) in edges.iter().take(30).enumerate() {
                    let va = inst.mesh.vertices[*a as usize];
                    let vb = inst.mesh.vertices[*b as usize];
                    println!("    #{}: v{}({:.2},{:.2},{:.2}) - v{}({:.2},{:.2},{:.2})",
                        i, a, va.x, va.y, va.z, b, vb.x, vb.y, vb.z);
                }
                if edges.len() > 30 {
                    println!("    ... and {} more", edges.len() - 30);
                }
            } else if bnd > 50 {
                println!("  (too many boundary edges to print: {})", bnd);
            }
        }
    }
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

fn boundary_edges(mesh: &draper_mesh::TriangleMesh) -> Vec<(u32, u32)> {
    use std::collections::HashMap;
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        for i in 0..3 {
            let a = tri[i].min(tri[(i+1)%3]);
            let b = tri[i].max(tri[(i+1)%3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let mut out: Vec<(u32, u32)> = edge_count.iter()
        .filter(|(_, c)| **c == 1)
        .map(|(e, _)| *e)
        .collect();
    out.sort();
    out
}
