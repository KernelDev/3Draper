// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Detailed watertight diagnostics for STEP file triangulation.
//!
//! For each BRep solid, shows per-face details and analyzes WHY
//! the merged mesh is not watertight.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::{validate_watertight, TriangulationParams};
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let default_path = "test/as1-oc-214.stp".to_string();
    let path = if args.len() > 1 { args[1].clone() } else { default_path };

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    println!("STEP file parsed: {} entities", step.entities.len());

    let start = std::time::Instant::now();
    let (_tree, pending) = step_structure_lazy(&step);
    println!("Structure parsed: {} BRep instances ({:.1?})", pending.len(), start.elapsed());

    let ctx = StepConversionContext::new(&step);

    // Track unique BReps (same brep_id = same geometry)
    let mut seen_breps: std::collections::HashSet<i64> = std::collections::HashSet::new();

    for (i, p) in pending.iter().enumerate() {
        // Skip duplicate BReps (same geometry at different positions)
        if seen_breps.contains(&p.brep_id) {
            continue;
        }
        seen_breps.insert(p.brep_id);

        println!("{}", "=".repeat(80));
        println!("BRep #{}: {} (instance #{})", p.brep_id, p.name, i + 1);
        println!("{}", "=".repeat(80));

        let t_start = std::time::Instant::now();
        let result = ctx.triangulate_pending(p);
        let t_elapsed = t_start.elapsed();

        match result {
            Some(inst) => {
                let report = validate_watertight(&inst.mesh, true);
                let is_wt = report.is_watertight();
                let wt_str = if is_wt { "WATERTIGHT" } else { "NOT WATERTIGHT" };

                println!("  Vertices: {}, Triangles: {}, Time: {:.1?}",
                    inst.mesh.vertex_count(), inst.mesh.triangle_count(), t_elapsed);
                println!("  Watertight: {}", wt_str);
                println!("  Boundary edges: {}, Non-manifold edges: {}, Degenerate: {}",
                    report.boundary_edge_count, report.non_manifold_edge_count,
                    report.degenerate_triangle_count);
                println!("  Euler characteristic: {} (should be 2 for closed solid)", report.euler_characteristic);

                if !is_wt {
                    // Analyze boundary edges: find the 3D coordinates and distances
                    println!("\n  --- Boundary Edge Analysis ---");
                    let mut boundary_edge_distances: Vec<f64> = Vec::new();
                    let mut small_gap_count = 0usize;
                    let mut medium_gap_count = 0usize;
                    let mut large_gap_count = 0usize;

                    for &(a, b) in &report.boundary_edges {
                        if (a as usize) < inst.mesh.vertices.len() && (b as usize) < inst.mesh.vertices.len() {
                            let va = inst.mesh.vertices[a as usize];
                            let vb = inst.mesh.vertices[b as usize];
                            let dist = ((va.x - vb.x).powi(2) + (va.y - vb.y).powi(2) + (va.z - vb.z).powi(2)).sqrt();
                            boundary_edge_distances.push(dist);
                            if dist < 0.001 { small_gap_count += 1; }
                            else if dist < 0.01 { medium_gap_count += 1; }
                            else { large_gap_count += 1; }
                        }
                    }

                    if !boundary_edge_distances.is_empty() {
                        boundary_edge_distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
                        let min_d = boundary_edge_distances[0];
                        let max_d = *boundary_edge_distances.last().unwrap();
                        let median_d = boundary_edge_distances[boundary_edge_distances.len() / 2];
                        println!("  Boundary edge lengths: min={:.10}, median={:.10}, max={:.10}",
                            min_d, median_d, max_d);
                        println!("  Gap distribution: <0.001: {}, 0.001-0.01: {}, >0.01: {}",
                            small_gap_count, medium_gap_count, large_gap_count);

                        // Show first few boundary edges with their coordinates
                        println!("\n  First 10 boundary edges (vertex indices + 3D positions):");
                        for (idx, &(a, b)) in report.boundary_edges.iter().take(10).enumerate() {
                            if (a as usize) < inst.mesh.vertices.len() && (b as usize) < inst.mesh.vertices.len() {
                                let va = inst.mesh.vertices[a as usize];
                                let vb = inst.mesh.vertices[b as usize];
                                let dist = ((va.x - vb.x).powi(2) + (va.y - vb.y).powi(2) + (va.z - vb.z).powi(2)).sqrt();
                                println!("    [{}] {}→{}: ({:.4},{:.4},{:.4}) → ({:.4},{:.4},{:.4}) dist={:.10}",
                                    idx, a, b, va.x, va.y, va.z, vb.x, vb.y, vb.z, dist);
                            }
                        }
                    }

                    // Per-face analysis
                    if !report.per_face_summary.is_empty() {
                        let mut bad_faces: Vec<_> = report.per_face_summary.iter()
                            .filter(|(_, s)| s.boundary_edge_count > 0)
                            .collect();
                        bad_faces.sort_by_key(|(_, s)| std::cmp::Reverse(s.boundary_edge_count));

                        println!("\n  Faces with boundary edges (showing up to 10):");
                        for (fid, s) in bad_faces.iter().take(10) {
                            println!("    Face #{}: {} tris, {} boundary edges",
                                fid, s.triangle_count, s.boundary_edge_count);
                        }
                    }

                    // Non-manifold edge analysis
                    if !report.non_manifold_edges.is_empty() {
                        println!("\n  Non-manifold edges (first 10):");
                        for (idx, &(a, b, count)) in report.non_manifold_edges.iter().take(10).enumerate() {
                            if (a as usize) < inst.mesh.vertices.len() && (b as usize) < inst.mesh.vertices.len() {
                                let va = inst.mesh.vertices[a as usize];
                                let vb = inst.mesh.vertices[b as usize];
                                let dist = ((va.x - vb.x).powi(2) + (va.y - vb.y).powi(2) + (va.z - vb.z).powi(2)).sqrt();
                                println!("    [{}] {}→{}: shared by {} triangles, edge dist={:.10}",
                                    idx, a, b, count, dist);
                            }
                        }
                    }

                    // Duplicate vertex check
                    let mut vertex_buckets: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
                    let cell_size = 0.01;
                    for (i, v) in inst.mesh.vertices.iter().enumerate() {
                        let cx = (v.x / cell_size).floor() as i64;
                        let cy = (v.y / cell_size).floor() as i64;
                        let cz = (v.z / cell_size).floor() as i64;
                        vertex_buckets.entry((cx, cy, cz)).or_default().push(i);
                    }
                    let near_duplicate_count = vertex_buckets.values()
                        .map(|v| if v.len() > 1 { v.len() - 1 } else { 0 })
                        .sum::<usize>();
                    if near_duplicate_count > 0 {
                        println!("\n  Near-duplicate vertex clusters (within {:.3}): {} vertices could be merged",
                            cell_size, near_duplicate_count);
                    }
                }
            }
            None => {
                println!("  TRIANGULATION FAILED");
            }
        }
    }
}
