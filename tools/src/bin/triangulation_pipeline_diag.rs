// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Comprehensive triangulation pipeline diagnostics.
//!
//! Analyzes the triangulation pipeline to identify geometry corruption.

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.iter().filter(|a| !a.starts_with('-')).nth(1)
        .cloned()
        .unwrap_or_else(|| "test/3.05.078.stp".to_string());

    println!("========================================================");
    println!("TRIANGULATION PIPELINE DIAGNOSTIC");
    println!("========================================================");
    println!("\nLoading STEP file: {}", path);
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    println!("STEP file parsed: {} entities", step.entities.len());

    // ============================================================
    // DIAG 1: Triangulation and watertightness check
    // ============================================================
    println!("\n{}", "=".repeat(60));
    println!("DIAG 1: Triangulation Result Analysis");
    println!("{}", "=".repeat(60));

    let (_tree, pending) = step_structure_lazy(&step);
    println!("BRep instances: {}", pending.len());

    let mut ctx = OwnedStepConversionContext::new(step);
    let mut seen_breps: std::collections::HashSet<i64> = std::collections::HashSet::new();

    for (i, p) in pending.iter().enumerate() {
        if seen_breps.contains(&p.brep_id) { continue; }
        seen_breps.insert(p.brep_id);

        println!("\n--- BRep #{}: {} (instance #{}) ---", p.brep_id, p.name, i + 1);

        let result = ctx.triangulate_pending(p);
        match result {
            Some(inst) => {
                let report = validate_watertight(&inst.mesh, true);
                let is_wt = report.is_watertight();

                println!("  Vertices: {}, Triangles: {}", inst.mesh.vertex_count(), inst.mesh.triangle_count());
                println!("  Watertight: {}", if is_wt { "YES ✓" } else { "NO ✗" });
                println!("  Boundary edges: {}, Non-manifold: {}, Degenerate: {}",
                    report.boundary_edge_count, report.non_manifold_edge_count, report.degenerate_triangle_count);

                if !is_wt {
                    // Analyze boundary edge distances
                    let mut distances: Vec<f64> = Vec::new();
                    for (a, b) in &report.boundary_edges {
                        if (*a as usize) < inst.mesh.vertices.len() && (*b as usize) < inst.mesh.vertices.len() {
                            let va = inst.mesh.vertices[*a as usize];
                            let vb = inst.mesh.vertices[*b as usize];
                            let dx = va.x - vb.x;
                            let dy = va.y - vb.y;
                            let dz = va.z - vb.z;
                            let dist = (dx*dx + dy*dy + dz*dz).sqrt();
                            distances.push(dist);
                        }
                    }
                    if !distances.is_empty() {
                        distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
                        println!("  Boundary edge lengths: min={:.10}, median={:.10}, max={:.10}",
                            distances[0], distances[distances.len()/2], *distances.last().unwrap());

                        // Check if gaps are micro (FP drift) or macro (geometry issue)
                        let micro_gaps = distances.iter().filter(|&d| *d < 1e-8).count();
                        let small_gaps = distances.iter().filter(|&d| *d < 1e-4 && *d >= 1e-8).count();
                        let macro_gaps = distances.iter().filter(|&d| *d >= 1e-4).count();
                        println!("  Gap categories: micro(<1e-8): {}, small(1e-8..1e-4): {}, macro(>1e-4): {}",
                            micro_gaps, small_gaps, macro_gaps);

                        if macro_gaps > 0 {
                            println!("  ⚠️ MACRO GAPS DETECTED - geometry corruption likely!");
                            // Show macro gap examples
                            println!("  Macro gap examples:");
                            for (idx, (a, b)) in report.boundary_edges.iter().take(5).enumerate() {
                                if (*a as usize) < inst.mesh.vertices.len() && (*b as usize) < inst.mesh.vertices.len() {
                                    let va = inst.mesh.vertices[*a as usize];
                                    let vb = inst.mesh.vertices[*b as usize];
                                    let dx = va.x - vb.x;
                                    let dy = va.y - vb.y;
                                    let dz = va.z - vb.z;
                                    let dist = (dx*dx + dy*dy + dz*dz).sqrt();
                                    if dist >= 1e-4 {
                                        println!("    {}: v{} ({:.4},{:.4},{:.4}) → v{} ({:.4},{:.4},{:.4}) gap={:.6}",
                                            idx, a, va.x, va.y, va.z, b, vb.x, vb.y, vb.z, dist);
                                    }
                                }
                            }
                        }
                    }

                    // Check face overlap analysis
                    if !report.per_face_summary.is_empty() {
                        println!("\n  Faces with boundary edges:");
                        let mut bad_faces: Vec<_> = report.per_face_summary.iter()
                            .filter(|(_, s)| s.boundary_edge_count > 0)
                            .collect();
                        bad_faces.sort_by_key(|(_, s)| std::cmp::Reverse(s.boundary_edge_count));
                        for (fid, s) in bad_faces.iter().take(10) {
                            println!("    Face #{}: {} tris, {} boundary edges",
                                fid, s.triangle_count, s.boundary_edge_count);
                        }
                    }
                }

                // ============================================================
                // DIAG 2: Vertex position uniqueness analysis
                // ============================================================
                println!("\n{}", "-".repeat(40));
                println!("DIAG 2: Vertex Position Uniqueness");
                println!("{}", "-".repeat(40));

                // Count unique bit-identical positions
                let mut bit_identical_positions: HashMap<[u64; 3], Vec<u32>> = HashMap::new();
                for (i, v) in inst.mesh.vertices.iter().enumerate() {
                    let key = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
                    bit_identical_positions.entry(key).or_default().push(i as u32);
                }

                let positions_with_duplicates = bit_identical_positions.values()
                    .filter(|v| v.len() > 1)
                    .count();
                let duplicate_vertex_count = bit_identical_positions.values()
                    .filter(|v| v.len() > 1)
                    .map(|v| v.len() - 1)
                    .sum::<usize>();

                println!("  Unique bit-identical positions: {}", bit_identical_positions.len());
                println!("  Positions with duplicate indices: {}", positions_with_duplicates);
                println!("  Total duplicate indices: {}", duplicate_vertex_count);

                if duplicate_vertex_count > 0 {
                    println!("  ⚠️ Duplicate vertex indices - deduplication may have failed!");
                }

                // ============================================================
                // DIAG 3: Near-duplicate vertex analysis (tolerance-based)
                // ============================================================
                println!("\n{}", "-".repeat(40));
                println!("DIAG 3: Near-Duplicate Vertex Analysis");
                println!("{}", "-".repeat(40));

                let tol = 1e-6; // Edge cache rounding tolerance
                let mut near_dup_clusters: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
                for (i, v) in inst.mesh.vertices.iter().enumerate() {
                    let cx = ((v.x / tol).round()) as i64;
                    let cy = ((v.y / tol).round()) as i64;
                    let cz = ((v.z / tol).round()) as i64;
                    near_dup_clusters.entry((cx, cy, cz)).or_default().push(i);
                }

                let near_dup_count = near_dup_clusters.values()
                    .filter(|v| v.len() > 1)
                    .count();
                let near_dup_vertices = near_dup_clusters.values()
                    .filter(|v| v.len() > 1)
                    .map(|v| v.len())
                    .sum::<usize>();

                println!("  Near-duplicate clusters (tol={}): {}", tol, near_dup_count);
                println!("  Vertices in near-duplicate clusters: {}", near_dup_vertices);

                // Show example near-duplicate pairs
                if near_dup_count > 0 {
                    println!("  Example near-duplicate pairs:");
                    for (cluster, indices) in near_dup_clusters.iter().take(3) {
                        if indices.len() > 1 {
                            println!("    Cluster {:?}: {} vertices", cluster, indices.len());
                            for &idx in indices.iter().take(3) {
                                let v = inst.mesh.vertices[idx];
                                println!("      v{}: ({:.10},{:.10},{:.10})", idx, v.x, v.y, v.z);
                            }
                        }
                    }
                }
            }
            None => {
                println!("  TRIANGULATION FAILED ✗");
            }
        }
    }

    // ============================================================
    // Recommendations
    // ============================================================
    println!("\n{}", "=".repeat(60));
    println!("Recommendations");
    println!("{}", "=".repeat(60));
    println!("\nIf MACRO GAPS detected:");
    println!("  → Check healing pipeline for geometry modifications");
    println!("  → Verify alias registration for shared edges");
    println!("  → Run with RUST_LOG=debug for detailed logs");
    println!("\nIf duplicate vertices detected:");
    println!("  → Check merge_deduplicating tolerance");
    println!("  → Verify edge cache deterministic_round_point");
    println!("========================================================");
}