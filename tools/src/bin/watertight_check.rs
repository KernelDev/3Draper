// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Standalone watertight validation tool for STEP file triangulation.
//!
//! Loads a STEP file, triangulates each BRep solid individually,
//! and checks whether each solid produces a watertight mesh.
//!
//! Usage: cargo run --bin watertight_check [file.stp]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path_arg = args.iter().find(|a| !a.starts_with('-') && a.as_str() != args[0]);
    let default_path = "test/as1-oc-214.stp".to_string();
    let path = if let Some(p) = path_arg {
        p.clone()
    } else {
        println!("No file specified, defaulting to test/as1-oc-214.stp");
        default_path
    };

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    println!("STEP file parsed: {} entities", step.entities.len());

    // Use lazy loading to get per-BRep instances
    let start = std::time::Instant::now();
    let (_tree, pending) = step_structure_lazy(&step);
    let lazy_time = start.elapsed();
    println!("Structure parsed: {} BRep instances (lazy: {:.1?})", pending.len(), lazy_time);

    // Triangulate each BRep individually and check watertightness
    let ctx = StepConversionContext::new(&step);
    let mut total_vertices = 0usize;
    let mut total_triangles = 0usize;
    let mut watertight_count = 0usize;
    let mut non_watertight_count = 0usize;
    let mut fail_count = 0usize;

    println!("\n{:-<80}", "");
    println!("{:>4} {:>30} {:>8} {:>8} {:>10} {:>10} {:>10} {:>12}",
        "#", "Name", "Verts", "Tris", "Boundary", "NonManf", "Degenerate", "Watertight");
    println!("{:-<80}", "");

    for (i, p) in pending.iter().enumerate() {
        let triangulate_start = std::time::Instant::now();
        let result = ctx.triangulate_pending(p);
        let triangulate_time = triangulate_start.elapsed();

        match result {
            Some(inst) => {
                let report = validate_watertight(&inst.mesh, true);
                let is_wt = report.is_watertight();
                if is_wt {
                    watertight_count += 1;
                } else {
                    non_watertight_count += 1;
                }
                total_vertices += inst.mesh.vertex_count();
                total_triangles += inst.mesh.triangle_count();

                let wt_str = if is_wt { "YES" } else { "NO" };
                println!("{:>4} {:>30} {:>8} {:>8} {:>10} {:>10} {:>10} {:>12}  ({:.1?})",
                    i + 1,
                    inst.name,
                    inst.mesh.vertex_count(),
                    inst.mesh.triangle_count(),
                    report.boundary_edge_count,
                    report.non_manifold_edge_count,
                    report.degenerate_triangle_count,
                    wt_str,
                    triangulate_time);

                // If not watertight, show details about which faces have boundary edges
                if !is_wt && !report.per_face_summary.is_empty() {
                    let mut bad_faces: Vec<_> = report.per_face_summary.iter()
                        .filter(|(_, s)| s.boundary_edge_count > 0)
                        .collect();
                    bad_faces.sort_by_key(|(_, s)| std::cmp::Reverse(s.boundary_edge_count));
                    let n_show = bad_faces.len().min(5);
                    for (fid, s) in &bad_faces[..n_show] {
                        println!("      Face #{}: {} tris, {} boundary edges", fid, s.triangle_count, s.boundary_edge_count);
                    }
                    if bad_faces.len() > 5 {
                        println!("      ... and {} more faces with boundary edges", bad_faces.len() - 5);
                    }
                }
            }
            None => {
                fail_count += 1;
                println!("{:>4} {:>30} {:>8} {:>8} {:>10} {:>10} {:>10} {:>12}",
                    i + 1, "(triangulation failed)", "-", "-", "-", "-", "-", "FAIL");
            }
        }
    }

    println!("{:-<80}", "");
    println!("\nSummary:");
    println!("  Total BReps:    {}", pending.len());
    println!("  Watertight:     {} ({:.0}%)", watertight_count,
        100.0 * watertight_count as f64 / pending.len().max(1) as f64);
    println!("  Not watertight: {}", non_watertight_count);
    println!("  Failed:         {}", fail_count);
    println!("  Total vertices: {}", total_vertices);
    println!("  Total triangles: {}", total_triangles);

    if non_watertight_count > 0 || fail_count > 0 {
        std::process::exit(1);
    }
}
