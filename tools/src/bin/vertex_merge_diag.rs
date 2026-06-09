// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Diagnose WHY vertices are not merging between faces.
//!
//! Usage: cargo run --bin vertex_merge_diag

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = "test/as1-oc-214.stp";
    println!("Loading: {}", path);
    let data = std::fs::read_to_string(path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");

    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    // Just check the first instance (nut)
    if let Some(inst) = ctx.triangulate_pending(&pending[0]) {
        let report = validate_watertight(&inst.mesh, true);
        println!("\nInstance: {} — {} verts, {} tris", inst.name, inst.mesh.vertex_count(), inst.mesh.triangle_count());
        println!("Watertight: {} — {} boundary, {} non-manifold", 
            report.is_watertight(), report.boundary_edge_count, report.non_manifold_edge_count);

        // Find the minimum distance between boundary edge vertices and any other vertex
        // This tells us if the vertices are "almost" shared but just outside merge tolerance
        let boundary_verts: Vec<u32> = report.boundary_edges.iter()
            .flat_map(|&(a, b)| [a, b])
            .collect::<std::collections::HashSet<_>>()
            .into_iter()
            .collect();

        println!("\n{} unique boundary vertices out of {} total", boundary_verts.len(), inst.mesh.vertex_count());

        // For each boundary vertex, find the closest non-identical vertex
        let mut min_dists = Vec::new();
        for &bv in &boundary_verts {
            let vp = inst.mesh.vertices[bv as usize];
            let mut min_dist = f64::MAX;
            for (j, other) in inst.mesh.vertices.iter().enumerate() {
                if j == bv as usize { continue; }
                let dx = vp.x - other.x;
                let dy = vp.y - other.y;
                let dz = vp.z - other.z;
                let d = (dx*dx + dy*dy + dz*dz).sqrt();
                if d < min_dist {
                    min_dist = d;
                }
            }
            min_dists.push(min_dist);
        }

        min_dists.sort_by(|a, b| a.partial_cmp(b).unwrap());

        println!("\nClosest non-identical vertex distances for boundary vertices:");
        println!("  min:    {:.2e}", min_dists.first().unwrap_or(&0.0));
        println!("  10th:   {:.2e}", min_dists.get(9).unwrap_or(&0.0));
        println!("  median: {:.2e}", min_dists.get(min_dists.len()/2).unwrap_or(&0.0));
        println!("  90th:   {:.2e}", min_dists.get(min_dists.len()*9/10).unwrap_or(&0.0));
        println!("  max:    {:.2e}", min_dists.last().unwrap_or(&0.0));

        // Count how many boundary vertices have a very close neighbor (< 1e-3)
        let close_count = min_dists.iter().filter(|&&d| d < 1e-3).count();
        println!("\n  {} of {} boundary verts have a neighbor within 1e-3", close_count, boundary_verts.len());

        let very_close_count = min_dists.iter().filter(|&&d| d < 1e-4).count();
        println!("  {} of {} boundary verts have a neighbor within 1e-4", very_close_count, boundary_verts.len());

        let micro_close_count = min_dists.iter().filter(|&&d| d < 1e-5).count();
        println!("  {} of {} boundary verts have a neighbor within 1e-5", micro_close_count, boundary_verts.len());

        // Bounding box info
        let (bmin, bmax) = inst.mesh.bounding_box();
        let dx = bmax.x - bmin.x;
        let dy = bmax.y - bmin.y;
        let dz = bmax.z - bmin.z;
        let diagonal = (dx*dx + dy*dy + dz*dz).sqrt();
        let merge_tol = (diagonal * 1e-6).max(1e-5).min(1e-2);
        println!("\nBounding box: ({:.2}, {:.2}, {:.2}) to ({:.2}, {:.2}, {:.2})", 
            bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z);
        println!("Diagonal: {:.4}", diagonal);
        println!("Current merge_tol: {:.2e}", merge_tol);
        println!("Suggested merge_tol (1e-4 * diag): {:.2e}", diagonal * 1e-4);
    }
}
