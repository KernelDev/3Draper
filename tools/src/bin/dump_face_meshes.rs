//! Dump per-face mesh diagnostics for a STEP file
//! Usage: dump_face_meshes <file.stp>

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <file.stp>", args[0]);
        std::process::exit(1);
    }
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let path = &args[1];
    let content = std::fs::read_to_string(path).expect("read");
    let step_file = draper_step::parser::parse_step(&content).expect("parse");
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);

    println!("File: {}", path);
    println!("BREPs: {}", pending.len());

    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);
    let mut global_merged = draper_mesh::TriangleMesh::new();

    for (idx, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let label = if pending.len() > 1 {
                format!("BREP#{} (instance {})", p.brep_id, idx)
            } else {
                format!("BREP#{}", p.brep_id)
            };
            println!("\n=== {} ===", label);
            println!("  Tris: {}  Verts: {}",
                inst.mesh.triangle_count(),
                inst.mesh.vertex_count());

            // Per-face mesh from triangle_face_ids
            if let Some(ref fids) = inst.mesh.triangle_face_ids {
                use std::collections::HashMap;
                let mut by_face: HashMap<u64, draper_mesh::TriangleMesh> = HashMap::new();
                for (tri_idx, &fid) in fids.iter().enumerate() {
                    let entry = by_face.entry(fid).or_insert_with(|| {
                        let mut m = draper_mesh::TriangleMesh::new();
                        // start with all vertices pre-populated so indices match
                        m.vertices = inst.mesh.vertices.clone();
                        m
                    });
                    entry.triangles.push(inst.mesh.triangles[tri_idx]);
                }
                let mut fids_sorted: Vec<u64> = by_face.keys().copied().collect();
                fids_sorted.sort_unstable();
                for fid in fids_sorted {
                    let m = &by_face[&fid];
                    let (bnd, total) = count_edges(m);
                    let status = if bnd == 0 { "OK" } else { "BND" };
                    println!("    face {:>3}: tris={:>4} bnd={:>3}/{:>3} {}",
                        fid, m.triangle_count(),
                        bnd, total, status);
                }
            }

            // Per-instance merged
            let (bnd, total) = count_edges(&inst.mesh);
            println!("  Instance merged: bnd={}/{} ({:.1}%)", bnd, total,
                if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 });

            global_merged.merge(&inst.mesh);
        }
    }

    println!("\n=== GLOBAL ===");
    let (bnd, total) = count_edges(&global_merged);
    println!("  Total tris: {}  Total verts: {}",
        global_merged.triangle_count(), global_merged.vertex_count());
    println!("  Boundary: {} / {} ({:.1}%)", bnd, total,
        if total > 0 { 100.0 * bnd as f64 / total as f64 } else { 0.0 });

    // Try to identify which edges are boundary
    if bnd > 0 {
        println!("\n  Boundary edges:");
        let edges = boundary_edges(&global_merged);
        for (i, (a, b)) in edges.iter().take(30).enumerate() {
            let va = global_merged.vertices[*a as usize];
            let vb = global_merged.vertices[*b as usize];
            println!("    #{}: v{}({:.2},{:.2},{:.2}) - v{}({:.2},{:.2},{:.2})",
                i, a, va.x, va.y, va.z, b, vb.x, vb.y, vb.z);
        }
        if edges.len() > 30 {
            println!("    ... and {} more", edges.len() - 30);
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
