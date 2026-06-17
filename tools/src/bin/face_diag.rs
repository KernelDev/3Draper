// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Detailed per-face diagnostic for a single BREP.
//!
//! Usage: cargo run --bin face_diag -- <step_file> [brep_index]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.iter().find(|a| !a.starts_with('-') && a.as_str() != args[0])
        .cloned()
        .unwrap_or_else(|| "test/as1-oc-214.stp".to_string());
    let brep_index: usize = args.iter()
        .filter(|a| !a.starts_with('-'))
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    println!("Loading: {} (BREP index {})", path, brep_index);
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    println!("Total BREP instances: {}", pending.len());

    if brep_index >= pending.len() {
        eprintln!("BREP index {} out of range (max {})", brep_index, pending.len() - 1);
        std::process::exit(1);
    }

    let ctx = StepConversionContext::new(&step);
    let p = &pending[brep_index];
    let result = ctx.triangulate_pending(p);
    let inst = match result {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); std::process::exit(2); }
    };

    println!("\n=== Instance: {} (BREP #{}) ===", inst.name, p.brep_id);
    println!("Vertices: {}, Triangles: {}", inst.mesh.vertex_count(), inst.mesh.triangle_count());

    // Per-face summary
    println!("\nFaces ({}):", inst.faces.len());
    println!("{:>4} {:>10} {:>10} {:>8} {:>10} {:>30}",
        "#", "surf", "tris", "forward", "step_id", "uv_bbox");
    for (fi, face) in inst.faces.iter().enumerate() {
        let tris = face.triangle_range.1 - face.triangle_range.0;
        // Get UV bbox from outer_uv_boundary
        let mut u_min = f64::MAX; let mut u_max = f64::MIN;
        let mut v_min = f64::MAX; let mut v_max = f64::MIN;
        for uv_loop in &face.outer_uv_boundary {
            for pt in uv_loop {
                u_min = u_min.min(pt.u); u_max = u_max.max(pt.u);
                v_min = v_min.min(pt.v); v_max = v_max.max(pt.v);
            }
        }
        let uv_str = if u_min.is_finite() {
            format!("[{:.2},{:.2}]x[{:.2},{:.2}]", u_min, u_max, v_min, v_max)
        } else {
            "(none)".to_string()
        };
        println!("{:>4} {:>10} {:>10} {:>8} {:>10} {:>30}",
            fi + 1, face.surface_type, tris, face.forward, face.step_face_id, uv_str);
    }

    // Detailed watertight analysis
    let report = draper_mesh::validate_watertight(&inst.mesh, true);
    println!("\nWatertight: {} (boundary={}, non-manifold={}, degenerate={})",
        if report.is_watertight() { "YES" } else { "NO" },
        report.boundary_edge_count,
        report.non_manifold_edge_count,
        report.degenerate_triangle_count);

    // Per-face degenerate triangle count
    println!("\nPer-face degenerate triangle analysis:");
    if let Some(face_ids) = &inst.mesh.triangle_face_ids {
        use std::collections::HashMap;
        let mut per_face_total: HashMap<u64, usize> = HashMap::new();
        let mut per_face_degen: HashMap<u64, usize> = HashMap::new();
        for (i, tri) in inst.mesh.triangles.iter().enumerate() {
            let fid = face_ids[i];
            *per_face_total.entry(fid).or_insert(0) += 1;
            // Check if degenerate (zero area)
            let v0 = inst.mesh.vertices[tri[0] as usize];
            let v1 = inst.mesh.vertices[tri[1] as usize];
            let v2 = inst.mesh.vertices[tri[2] as usize];
            let ex = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
            let fx = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
            let cross = (ex.1 * fx.2 - ex.2 * fx.1,
                         ex.2 * fx.0 - ex.0 * fx.2,
                         ex.0 * fx.1 - ex.1 * fx.0);
            let area_sq = cross.0 * cross.0 + cross.1 * cross.1 + cross.2 * cross.2;
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] || area_sq < 1e-20 {
                *per_face_degen.entry(fid).or_insert(0) += 1;
            }
        }
        let mut fids: Vec<u64> = per_face_total.keys().copied().collect();
        fids.sort();
        for fid in fids {
            let total = per_face_total[&fid];
            let degen = per_face_degen.get(&fid).copied().unwrap_or(0);
            println!("  Face {}: {} triangles, {} degenerate ({:.0}%)",
                fid, total, degen, 100.0 * degen as f64 / total as f64);
        }

        // For the first face with high degeneracy, dump first 5 degenerate triangles
        if let Some((&worst_fid, _)) = per_face_degen.iter().max_by_key(|(_, &v)| v) {
            let degen_count = per_face_degen[&worst_fid];
            if degen_count > 0 {
                println!("\n  First 5 degenerate triangles in Face {}:", worst_fid);
                let mut shown = 0;
                for (i, tri) in inst.mesh.triangles.iter().enumerate() {
                    if face_ids[i] != worst_fid { continue; }
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];
                    let ex = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    let fx = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    let cross = (ex.1 * fx.2 - ex.2 * fx.1,
                                 ex.2 * fx.0 - ex.0 * fx.2,
                                 ex.0 * fx.1 - ex.1 * fx.0);
                    let area_sq = cross.0 * cross.0 + cross.1 * cross.1 + cross.2 * cross.2;
                    let is_degen = tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] || area_sq < 1e-20;
                    if is_degen {
                        println!("    tri {}: indices=({},{},{}), v0=({:.3},{:.3},{:.3}), v1=({:.3},{:.3},{:.3}), v2=({:.3},{:.3},{:.3}), area_sq={:.2e}",
                            i, tri[0], tri[1], tri[2],
                            v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, area_sq);
                        shown += 1;
                        if shown >= 5 { break; }
                    }
                }
            }
        }
    }

    // Show a few boundary edges with their face_ids
    let face_ids = inst.mesh.triangle_face_ids.as_ref();
    if !report.boundary_edges.is_empty() {
        println!("\nBoundary edges (first 20):");
        for (i, (a, b)) in report.boundary_edges.iter().take(20).enumerate() {
            let pa = inst.mesh.vertices[*a as usize];
            let pb = inst.mesh.vertices[*b as usize];
            // Find which face owns this edge
            let mut owner_faces = Vec::new();
            if let Some(ids) = face_ids {
                for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                    let edges = [
                        (tri[0].min(tri[1]), tri[0].max(tri[1])),
                        (tri[1].min(tri[2]), tri[1].max(tri[2])),
                        (tri[2].min(tri[0]), tri[2].max(tri[0])),
                    ];
                    if edges.contains(&(*a, *b)) {
                        owner_faces.push(ids[ti]);
                    }
                }
            }
            println!("  {}: vertices ({}, {}), pos_a=({:.4},{:.4},{:.4}), pos_b=({:.4},{:.4},{:.4}), faces={:?}",
                i, a, b, pa.x, pa.y, pa.z, pb.x, pb.y, pb.z, owner_faces);
        }
    }

    // Show a few non-manifold edges
    if !report.non_manifold_edges.is_empty() {
        println!("\nNon-manifold edges (first 10):");
        for (i, (a, b, count)) in report.non_manifold_edges.iter().take(10).enumerate() {
            let pa = inst.mesh.vertices[*a as usize];
            let pb = inst.mesh.vertices[*b as usize];
            println!("  {}: vertices ({}, {}) count={}, pos_a=({:.4},{:.4},{:.4}), pos_b=({:.4},{:.4},{:.4})",
                i, a, b, count, pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
        }
    }

    // Find duplicate vertex positions (vertices with same 3D position but different indices)
    {
        use std::collections::HashMap;
        let mut pos_to_indices: HashMap<[u64; 3], Vec<u32>> = HashMap::new();
        for (i, v) in inst.mesh.vertices.iter().enumerate() {
            let key = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
            pos_to_indices.entry(key).or_default().push(i as u32);
        }
        let dup_count = pos_to_indices.values().filter(|v| v.len() > 1).count();
        println!("\nDuplicate vertex positions: {} (out of {} unique positions)",
            dup_count, pos_to_indices.len());

        // Show first 5 duplicate groups
        let dups: Vec<_> = pos_to_indices.iter().filter(|(_, v)| v.len() > 1).take(5).collect();
        for (key, indices) in dups {
            let v = inst.mesh.vertices[indices[0] as usize];
            println!("  pos=({:.4},{:.4},{:.4}): indices={:?} ({} duplicates)",
                v.x, v.y, v.z, indices, indices.len());
        }
    }

    // Count duplicate triangles (triangles with the same vertex set)
    {
        use std::collections::HashMap;
        let mut tri_count: HashMap<[u32; 3], usize> = HashMap::new();
        for tri in &inst.mesh.triangles {
            let mut sorted = [tri[0], tri[1], tri[2]];
            sorted.sort();
            *tri_count.entry(sorted).or_insert(0) += 1;
        }
        let dup_tris = tri_count.values().filter(|&&c| c > 1).count();
        let total_dup = tri_count.values().filter(|&&c| c > 1).map(|&c| c - 1).sum::<usize>();
        println!("\nDuplicate triangles: {} unique sets with duplicates, {} extra triangles",
            dup_tris, total_dup);

        // Show top 5 most-duplicated triangles
        let mut sorted_counts: Vec<_> = tri_count.iter().filter(|(_, &c)| c > 1).collect();
        sorted_counts.sort_by(|a, b| b.1.cmp(a.1));
        for (tri, count) in sorted_counts.iter().take(5) {
            // Find which face this triangle belongs to
            let face_id = if let Some(face_ids) = &inst.mesh.triangle_face_ids {
                let mut found = None;
                for (i, t) in inst.mesh.triangles.iter().enumerate() {
                    let mut s = [t[0], t[1], t[2]];
                    s.sort();
                    if &s == *tri {
                        found = Some(face_ids[i]);
                        break;
                    }
                }
                found
            } else { None };
            println!("  tri {:?}: {} occurrences (face {:?})", tri, count, face_id);
        }
    }
}
