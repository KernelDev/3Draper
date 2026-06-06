// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Standalone watertight check binary for STEP assemblies.
//!
//! Loads a STEP file, triangulates each solid using the CDT-based watertight
//! pipeline, and reports watertightness per part. Useful for validating mesh
//! quality on real-world CAD assemblies like as1-oc-214.stp.
//!
//! # Algorithm
//! Uses `triangulate_solid_watertight` which:
//! 1. Triangulates each face independently using the standard pipeline
//! 2. Unifies vertices across ALL faces using 3D spatial hashing with tolerance
//! 3. Remaps all triangle vertex indices to the unified vertex set
//! 4. Stitches remaining boundary edges with progressive tolerance sweep

use draper_step::converter::{step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::{
    validate_watertight,
    triangulate_solid_watertight, adaptive_merge_tolerance,
    TriangulationParams,
};
use std::env;
use std::path::Path;
use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info")
    ).init();

    let args: Vec<String> = env::args().collect();
    let step_path = if args.len() > 1 {
        args[1].clone()
    } else {
        // Default: as1-oc-214.stp in the test directory
        let default = env::current_dir()
            .unwrap_or_default()
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.join("test/as1-oc-214.stp"))
            .unwrap_or_default();
        if default.exists() {
            default.to_str().unwrap_or("test/as1-oc-214.stp").to_string()
        } else {
            "test/as1-oc-214.stp".to_string()
        }
    };

    // Parse --legacy flag to use old triangulation path
    let use_legacy = args.iter().any(|a| a == "--legacy");

    if !Path::new(&step_path).exists() {
        eprintln!("Error: STEP file not found: {}", step_path);
        eprintln!("Usage: watertight_check [path/to/file.stp] [--legacy]");
        std::process::exit(1);
    }

    let mode = if use_legacy { "LEGACY" } else { "CDT-WATERTIGHT" };
    println!("=== 3Draper Watertight Checker ({}) ===", mode);
    println!("Loading STEP file: {}", step_path);

    let parse_start = Instant::now();
    let step_data = std::fs::read(&step_path).expect("Failed to read STEP file");
    let step_file = draper_step::parser::parse_step(&String::from_utf8_lossy(&step_data))
        .expect("Failed to parse STEP file");
    println!("STEP file parsed in {:.2}s", parse_start.elapsed().as_secs_f64());

    // Build assembly structure without triangulation (fast)
    let structure_start = Instant::now();
    let (_tree, pending_instances) = step_structure_lazy(&step_file);
    println!("Assembly structure resolved in {:.2}s ({} instances)",
        structure_start.elapsed().as_secs_f64(), pending_instances.len());

    // Triangulate each instance and check watertightness
    let mut ctx = OwnedStepConversionContext::new(step_file);

    let mut total_watertight = 0usize;
    let mut total_not_watertight = 0usize;
    let mut total_empty = 0usize;
    let mut total_boundary_edges = 0usize;
    let mut total_non_manifold = 0usize;

    println!("\n{:<40} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>12}",
        "Part", "Verts", "Tris", "Edges", "Bound", "NonMan", "Euler", "Status");
    println!("{}", "-".repeat(108));

    for (_i, pending) in pending_instances.iter().enumerate() {
        let tri_start = Instant::now();

        if use_legacy {
            // Legacy path: use the standard STEP converter triangulation
            let instance = ctx.triangulate_pending(pending);
            match instance {
                Some(inst) => {
                    let tri_time = tri_start.elapsed();
                    let report = validate_watertight(&inst.mesh, true);
                    print_report(
                        &truncate_name(&inst.name, 40),
                        &report,
                        &inst.mesh,
                        tri_time,
                        &mut total_watertight,
                        &mut total_not_watertight,
                        &mut total_empty,
                        &mut total_boundary_edges,
                        &mut total_non_manifold,
                    );
                }
                None => {
                    total_empty += 1;
                    println!("{:<40} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>12}",
                        truncate_name(&pending.name, 40),
                        0, 0, 0, 0, 0, 0, "EMPTY");
                }
            }
        } else {
            // CDT watertight path: extract Solid topology and triangulate with
            // unified vertex merging + boundary edge stitching
            let solid = ctx.extract_solid_from_brep(pending.brep_id);
            match solid {
                Some(solid) => {
                    // Compute adaptive merge tolerance based on bounding box
                    // Factor 8e-3 (0.8%) with max 20.0 to merge shared-edge
                    // boundary vertices without over-merging within faces
                    let merge_tol = adaptive_merge_tolerance(&solid, 8e-3, 1e-3, 20.0);

                    let params = TriangulationParams::default();
                    let mesh = triangulate_solid_watertight(&solid, &params, merge_tol);

                    let tri_time = tri_start.elapsed();
                    let report = validate_watertight(&mesh, true);
                    print_report(
                        &truncate_name(&pending.name, 40),
                        &report,
                        &mesh,
                        tri_time,
                        &mut total_watertight,
                        &mut total_not_watertight,
                        &mut total_empty,
                        &mut total_boundary_edges,
                        &mut total_non_manifold,
                    );
                }
                None => {
                    total_empty += 1;
                    println!("{:<40} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>12}",
                        truncate_name(&pending.name, 40),
                        0, 0, 0, 0, 0, 0, "EMPTY");
                }
            }
        }
    }

    println!("{}", "-".repeat(108));
    println!("\n=== Summary ({}) ===", mode);
    println!("Total instances:  {}", pending_instances.len());
    println!("Watertight:       {} ({:.1}%)",
        total_watertight,
        100.0 * total_watertight as f64 / pending_instances.len().max(1) as f64);
    println!("Not watertight:   {}", total_not_watertight);
    println!("Empty/failed:     {}", total_empty);
    println!("Total boundary edges:   {}", total_boundary_edges);
    println!("Total non-manifold:     {}", total_non_manifold);

    if total_not_watertight > 0 {
        std::process::exit(1);
    }
}

fn print_report(
    name: &str,
    report: &draper_mesh::WatertightReport,
    mesh: &draper_mesh::TriangleMesh,
    tri_time: std::time::Duration,
    total_watertight: &mut usize,
    total_not_watertight: &mut usize,
    _total_empty: &mut usize,
    total_boundary_edges: &mut usize,
    total_non_manifold: &mut usize,
) {
    let status = if report.is_watertight() {
        *total_watertight += 1;
        "WATERTIGHT"
    } else {
        *total_not_watertight += 1;
        "LEAKY"
    };
    *total_boundary_edges += report.boundary_edge_count;
    *total_non_manifold += report.non_manifold_edge_count;

    println!("{:<40} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>12}  ({:.1}ms)",
        name,
        report.vertex_count,
        report.triangle_count,
        report.edge_count,
        report.boundary_edge_count,
        report.non_manifold_edge_count,
        report.euler_characteristic,
        status,
        tri_time.as_secs_f64() * 1000.0,
    );

    // If leaky, show some boundary edge details
    if !report.is_watertight() && !report.boundary_edges.is_empty() {
        let show_count = report.boundary_edges.len().min(5);
        for &(a, b) in &report.boundary_edges[..show_count] {
            let va = mesh.vertices.get(a as usize);
            let vb = mesh.vertices.get(b as usize);
            if let (Some(pa), Some(pb)) = (va, vb) {
                println!("  boundary edge: v{}({:.4},{:.4},{:.4}) - v{}({:.4},{:.4},{:.4})",
                    a, pa.x, pa.y, pa.z, b, pb.x, pb.y, pb.z);
            }
        }
        if report.boundary_edges.len() > 5 {
            println!("  ... and {} more boundary edges", report.boundary_edges.len() - 5);
        }
    }

    // Show non-manifold edge details
    if !report.non_manifold_edges.is_empty() {
        let show_count = report.non_manifold_edges.len().min(3);
        for &(a, b, count) in &report.non_manifold_edges[..show_count] {
            println!("  non-manifold edge: v{}-v{} (shared by {} triangles)", a, b, count);
        }
        if report.non_manifold_edges.len() > 3 {
            println!("  ... and {} more non-manifold edges", report.non_manifold_edges.len() - 3);
        }
    }
}

fn truncate_name(name: &str, max_len: usize) -> String {
    if name.len() <= max_len {
        name.to_string()
    } else {
        format!("...{}", &name[name.len() - max_len + 3..])
    }
}
