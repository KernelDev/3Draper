// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! C5 follow-up #1 — industrial-file performance bench.
//!
//! Loads a STEP file, extracts solids, triangulates each solid with
//! timing + boundary report. Usage:
//!
//! ```text
//! cargo run -p draper-step --release --example transmission_bench -- \
//!     test/transmission_top.stp [--nthreads N] [--verbose]
//! ```
//!
//! `--verbose` additionally prints per-solid face counts so the largest
//! faces can be correlated with the slowest solids.

use draper_mesh::triangulate::TriangulationParams;
use draper_mesh::triangulate_solid_with_report;
use draper_mesh::{triangulate_solid_face_with_cache, EdgeDiscretizationCache};
use draper_step::{extract_solids, parse_step_file};
use std::time::Instant;

fn main() {
    // Honor RUST_LOG (e.g. `RUST_LOG=draper_mesh=info`) so the bench can
    // surface mesh-pipeline instrumentation (fallback tiers, weld passes,
    // gap-filling) while benchmarking. Default: warnings only.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .iter()
        .find(|a| !a.starts_with("--"))
        .cloned()
        .unwrap_or_else(|| "test/transmission_top.stp".to_string());
    let verbose = args.iter().any(|a| a == "--verbose");
    let solid_filter: Option<usize> = args
        .iter()
        .position(|a| a == "--solid")
        .and_then(|p| args.get(p + 1))
        .and_then(|v| v.parse().ok());
    let face_bench = args.iter().any(|a| a == "--faces");
    if let Some(pos) = args.iter().position(|a| a == "--nthreads") {
        if let Some(v) = args.get(pos + 1) {
            if let Ok(n) = v.parse::<usize>() {
                rayon::ThreadPoolBuilder::new()
                    .num_threads(n)
                    .build_global()
                    .ok();
            }
        }
    }

    println!("=== industrial bench: {} ===", path);

    let t0 = Instant::now();
    let step_file = match parse_step_file(&path) {
        Ok(sf) => sf,
        Err(e) => {
            eprintln!("parse error: {e}");
            std::process::exit(1);
        }
    };
    let t_parse = t0.elapsed();

    let t1 = Instant::now();
    let (solids, _ids) = extract_solids(&step_file);
    let t_extract = t1.elapsed();

    println!(
        "parse:    {:>9.2?}   extract: {:>9.2?}   solids: {}",
        t_parse, t_extract, solids.len()
    );

    let params = TriangulationParams::default();

    // Per-face attribution mode: replicate the sequential pipeline manually
    // and time every face individually (no lib changes needed).
    if face_bench {
        if let Some(si) = solid_filter {
            face_bench_run(&solids[si], &params);
        } else {
            for (si, solid) in solids.iter().enumerate() {
                println!("── solid {si} ──");
                face_bench_run(solid, &params);
            }
        }
        return;
    }

    let mut total_boundary_pct = 0.0f64;
    let mut total_boundary_edges = 0usize;
    let mut total_edges = 0usize;
    let mut total_tris = 0usize;

    let t_all = Instant::now();
    let solid_filter_val = solid_filter;
    let solids_iter: Vec<&draper_topology::Solid> = match solid_filter {
        Some(si) => vec![&solids[si]],
        None => solids.iter().collect(),
    };
    for (idx, solid) in solids_iter.iter().enumerate() {
        let ts = Instant::now();
        let result = triangulate_solid_with_report(solid, &params);
        let dt = ts.elapsed();
        let r = &result.report;
        total_boundary_pct += r.boundary_pct;
        total_boundary_edges += r.boundary_edge_count;
        total_edges += r.edge_count;
        total_tris += result.mesh.triangle_count();
        let flag = if r.boundary_pct > 5.0 { "  <-- >5%" } else { "" };
        println!(
            "solid {:>3}: {:>9.2?}  tris {:>9}  verts {:>9}  boundary {:>6.2}% ({}/{}){}",
            solid_filter_val.unwrap_or(idx),
            dt,
            result.mesh.triangle_count(),
            result.mesh.vertex_count(),
            r.boundary_pct,
            r.boundary_edge_count,
            r.edge_count,
            flag
        );
        if verbose {
            let n_faces = solid.faces().len();
            println!("           faces: {n_faces}");
        }
    }
    let t_tri = t_all.elapsed();

    let overall_pct = if total_edges > 0 {
        total_boundary_edges as f64 / total_edges as f64 * 100.0
    } else {
        0.0
    };
    println!(
        "triangulation total: {:>9.2?}   triangles: {}   overall boundary: {:.2}%",
        t_tri, total_tris, overall_pct
    );
}

/// Time each face of `solid` through the store-resolved entry,
/// mirroring `triangulate_solid_sequential` (pre-populate + per-face).
fn face_bench_run(solid: &draper_topology::Solid, params: &TriangulationParams) {
    let mut cache = EdgeDiscretizationCache::new();
    let t0 = Instant::now();
    cache.pre_populate_for_solid(solid, 20);
    let t_pre = t0.elapsed();
    println!("  pre-populate: {t_pre:.2?}");

    let faces = solid.faces();
    let mut rows: Vec<(usize, f64, usize, &'static str, usize)> = Vec::new();
    for (fi, face) in faces.iter().enumerate() {
        let stype = face
            .surface
            .as_ref()
            .map(surface_name)
            .unwrap_or("None");
        // (C5 7.6b: store-resolved boundary — Face has no edge mirrors.)
        let nb = solid.resolve_face_edges(face).len();
        let t = Instant::now();
        let m = triangulate_solid_face_with_cache(solid, face, params, &mut cache);
        rows.push((fi, t.elapsed().as_secs_f64(), m.triangle_count(), stype, nb));
    }
    rows.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    let total: f64 = rows.iter().map(|r| r.1).sum();
    println!("  face-tris total: {total:.2?}  ({} faces)", rows.len());
    for (fi, dt, tris, stype, nb) in rows.iter().take(20) {
        println!(
            "  face {fi:>4}: {dt:>8.3}s  tris {tris:>7}  {stype:<10}  bnd-edges {nb}"
        );
    }
}

fn surface_name(s: &draper_geometry::Surface) -> &'static str {
    use draper_geometry::Surface::*;
    match s {
        Plane(_) => "Plane",
        Cylinder(_) => "Cylinder",
        Cone(_) => "Cone",
        Sphere(_) => "Sphere",
        Torus(_) => "Torus",
        Nurbs(_) => "Nurbs",
        Revolution(_) => "Revolution",
        Extrusion(_) => "Extrusion",
        Offset(_) => "Offset",
        Ruled(_) => "Ruled",
    }
}
