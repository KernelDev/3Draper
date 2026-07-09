//! Per-BREP/per-face profiling for drill_top.stp at mobile LOD (0.5).
//!
//! Reports:
//! - Per-BREP: time, face count, triangle count, vertex count
//! - Per-face: surface type, triangle count, hole count
//! - Identifies the slowest BREP and the slowest face types

use std::time::Instant;
use std::collections::HashMap;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let lod_str = std::env::args().nth(2).unwrap_or("0.5".to_string());
    let lod: f64 = lod_str.parse().expect("lod must be a float");

    println!("Loading {} at LOD {} ...", path, lod);
    let t0 = Instant::now();
    let content = std::fs::read_to_string(&path).expect("read");
    let step_file = draper_step::parser::parse_step(&content).expect("parse");
    println!("  parsed in {:.2}s ({} entities)", t0.elapsed().as_secs_f64(), step_file.entities.len());

    let t1 = Instant::now();
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);
    println!("  structure built in {:.2}s ({} pending BREPs)", t1.elapsed().as_secs_f64(), pending.len());

    // Use LOD params (mobile-like)
    let mut ctx = draper_step::OwnedStepConversionContext::new_with_lod(step_file, lod);

    let mut total_tris = 0usize;
    let mut total_verts = 0usize;
    let mut total_faces = 0usize;
    let mut brep_timings: Vec<(String, f64, usize, usize, usize)> = Vec::new();
    let mut surface_type_totals: HashMap<String, (usize, usize)> = HashMap::new(); // type -> (face_count, tri_count)

    for (i, p) in pending.iter().enumerate() {
        let brep_start = Instant::now();
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => {
                println!("[{}/{}] '{}' FAILED", i+1, pending.len(), p.name);
                continue;
            }
        };
        let elapsed = brep_start.elapsed().as_secs_f64();
        let n_faces = inst.faces.len();
        let n_verts = inst.mesh.vertex_count();
        let n_tris = inst.mesh.triangle_count();

        // Per-surface-type breakdown
        let mut brep_surface_counts: HashMap<String, (usize, usize)> = HashMap::new();
        for face in &inst.faces {
            let tris = face.triangle_range.1 - face.triangle_range.0;
            let st = face.surface_type.split('(').next().unwrap_or(&face.surface_type).to_string();
            *brep_surface_counts.entry(st.clone()).or_insert((0, 0)) = (
                brep_surface_counts.get(&st).map(|x| x.0).unwrap_or(0) + 1,
                brep_surface_counts.get(&st).map(|x| x.1).unwrap_or(0) + tris,
            );
            *surface_type_totals.entry(st).or_insert((0, 0)) = (
                surface_type_totals.get(&st).map(|x| x.0).unwrap_or(0) + 1,
                surface_type_totals.get(&st).map(|x| x.1).unwrap_or(0) + tris,
            );
        }

        println!("\n[{}/{}] '{}' (BREP #{}) in {:.2}s — {} faces, {} verts, {} tris",
            i+1, pending.len(), inst.name, p.brep_id, elapsed, n_faces, n_verts, n_tris);

        // Sort by triangle count descending
        let mut sorted: Vec<_> = brep_surface_counts.iter().collect();
        sorted.sort_by(|a, b| b.1.1.cmp(&a.1.1));
        for (st, (fc, tc)) in &sorted {
            let avg_tris = if *fc > 0 { *tc / *fc } else { 0 };
            println!("    {:<12}: {:>4} faces, {:>6} tris (avg {} tris/face)", st, fc, tc, avg_tris);
        }

        // Faces with holes — count by surface type
        let faces_with_holes = inst.faces.iter().filter(|f| !f.inner_uv_boundaries.is_empty()).count();
        if faces_with_holes > 0 {
            println!("    Faces with holes: {}", faces_with_holes);
        }

        // Find the face with most triangles
        if let Some(max_face) = inst.faces.iter().max_by_key(|f| f.triangle_range.1 - f.triangle_range.0) {
            let max_tris = max_face.triangle_range.1 - max_face.triangle_range.0;
            if max_tris > 500 {
                println!("    *** Highest-triangle face: STEP #{} type={} tris={} ***",
                    max_face.step_face_id, max_face.surface_type, max_tris);
            }
        }

        total_tris += n_tris;
        total_verts += n_verts;
        total_faces += n_faces;
        brep_timings.push((inst.name.clone(), elapsed, n_faces, n_verts, n_tris));
    }

    println!("\n=== Summary (LOD {}) ===", lod);
    println!("Total: {} BREPs, {} faces, {} verts, {} tris",
        brep_timings.len(), total_faces, total_verts, total_tris);
    println!("\nPer-BREP timings (sorted by time desc):");
    let mut sorted = brep_timings.clone();
    sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    for (name, t, f, v, tr) in &sorted {
        let avg_face_ms = if *f > 0 { t * 1000.0 / *f as f64 } else { 0.0 };
        println!("  {:<35} {:>6.2}s  {:>4} faces  {:>6} verts  {:>6} tris  ({:.2}ms/face)",
            name, t, f, v, tr, avg_face_ms);
    }

    println!("\nSurface-type totals (across all BREPs):");
    let mut sorted_st: Vec<_> = surface_type_totals.iter().collect();
    sorted_st.sort_by(|a, b| b.1.1.cmp(&a.1.1));
    for (st, (fc, tc)) in &sorted_st {
        let avg_tris = if *fc > 0 { *tc / *fc } else { 0 };
        println!("  {:<12}: {:>4} faces, {:>6} tris (avg {} tris/face)", st, fc, tc, avg_tris);
    }

    // Estimate mobile (WASM) time: assume 5-10x slowdown vs desktop native.
    let total_time: f64 = brep_timings.iter().map(|x| x.1).sum();
    println!("\nTotal desktop time: {:.2}s", total_time);
    println!("Estimated mobile WASM time (5x slower): {:.1}s", total_time * 5.0);
    println!("Estimated mobile WASM time (10x slower): {:.1}s", total_time * 10.0);
    println!("Mobile timeout in viewer: 120s");

    let slowest_brep = sorted[0].1;
    println!("\nSlowest single BREP: {:.2}s (desktop)", slowest_brep);
    println!("  Estimated mobile freeze duration: {:.1}-{:.1}s", slowest_brep * 5.0, slowest_brep * 10.0);
}
