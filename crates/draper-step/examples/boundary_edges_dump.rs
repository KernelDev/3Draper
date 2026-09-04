// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! C5 follow-up #2 — residual boundary-edge diagnostics for synthetic files.
//!
//! Loads a STEP file, triangulates the first solid, and prints every mesh
//! edge used by exactly one triangle (boundary edge) with its endpoint
//! coordinates, so residual seam mismatches can be attributed to faces.
//!
//! ```text
//! cargo run -p draper-step --release --example boundary_edges_dump -- \
//!     test/synthetic/synth_cone.stp
//! ```

use draper_mesh::triangulate::TriangulationParams;
use draper_mesh::triangulate_solid_with_report;
use draper_step::{extract_solids, parse_step_file};
use std::collections::HashMap;

fn main() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .try_init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .first()
        .cloned()
        .unwrap_or_else(|| "test/synthetic/synth_cone.stp".to_string());

    let step_file = parse_step_file(&path).expect("parse failed");
    let (solids, _ids) = extract_solids(&step_file);
    println!("{}: {} solids", path, solids.len());

    let params = TriangulationParams::default();
    for (si, solid) in solids.iter().enumerate() {
        // Topology probe: per-face wire structure and coedge resolution
        for (fi, face) in solid.faces().iter().enumerate() {
            let outer_n = face
                .outer_wire
                .as_ref()
                .map(|w| w.coedges.len())
                .unwrap_or(0);
            let stype = face
                .surface
                .as_ref()
                .map(|s| {
                    use draper_geometry::Surface::*;
                    match s {
                        Plane(_) => "Plane",
                        Cylinder(_) => "Cylinder",
                        Sphere(_) => "Sphere",
                        Cone(_) => "Cone",
                        Torus(_) => "Torus",
                        Nurbs(_) => "Nurbs",
                        _ => "Other",
                    }
                })
                .unwrap_or("None");
            let holes = face
                .inner_wires
                .iter()
                .map(|w| {
                    // (C5 7.6b: store-resolved ids — Face has no edge
                    // mirrors; the store answers coedge resolution.)
                    w.coedges
                        .iter()
                        .filter(|c| solid.edge_store.instance_edge(c.edge).is_some())
                        .count()
                })
                .collect::<Vec<_>>();
            println!(
                "  face {fi}: {stype} forward={} outer_coedges={outer_n} inner_wires={} resolved_coedges_per_wire={holes:?}",
                face.forward,
                face.inner_wires.len()
            );
        }
        let result = triangulate_solid_with_report(solid, &params);
        let mesh = &result.mesh;
        let r = &result.report;
        println!(
            "solid {si}: tris={} verts={} boundary={}/{} ({:.2}%)",
            mesh.triangle_count(),
            mesh.vertex_count(),
            r.boundary_edge_count,
            r.edge_count,
            r.boundary_pct
        );

        // Count edge usage over triangles (sorted index pairs)
        let mut usage: HashMap<(u32, u32), u32> = HashMap::new();
        for idx in &mesh.triangles {
            let (a, b, c) = (idx[0], idx[1], idx[2]);
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let key = if u < v { (u, v) } else { (v, u) };
                *usage.entry(key).or_insert(0) += 1;
            }
        }

        let mut boundary: Vec<(u32, u32)> = usage
            .iter()
            .filter(|(_, &n)| n == 1)
            .map(|(&(u, v), _)| (u, v))
            .collect();
        boundary.sort_unstable();
        // Face-id histogram (if per-triangle face ids are present)
        if let Some(fids) = &mesh.triangle_face_ids {
            let mut hist: HashMap<u64, usize> = HashMap::new();
            for &f in fids {
                *hist.entry(f).or_insert(0) += 1;
            }
            let mut hist: Vec<(u64, usize)> = hist.into_iter().collect();
            hist.sort_unstable();
            println!("  per-face triangle counts (TopoId -> tris):");
            for (f, c) in hist {
                println!("    {:>20} -> {}", f, c);
            }
            // Face attribution of the triangle adjacent to each boundary edge
            println!("  boundary edge -> adjacent triangle face id:");
            for (u, v) in boundary.iter().take(8) {
                let owner = mesh
                    .triangles
                    .iter()
                    .zip(fids.iter())
                    .find(|(idx, _)| {
                        let set = [idx[0], idx[1], idx[2]];
                        let has_u = set.contains(u);
                        let has_v = set.contains(v);
                        has_u && has_v
                    })
                    .map(|(_, &f)| f);
                let pu = mesh.vertices[*u as usize];
                let pv = mesh.vertices[*v as usize];
                println!(
                    "    ({:>8.4},{:>8.4},{:>6.3}) - ({:>8.4},{:>8.4},{:>6.3}) -> face {:?}",
                    pu.x, pu.y, pu.z, pv.x, pv.y, pv.z, owner
                );
            }
        }

        println!("  boundary edges (used by exactly 1 triangle):");
        for (u, v) in boundary {
            let pu = mesh.vertices[u as usize];
            let pv = mesh.vertices[v as usize];
            println!(
                "    ({:>9.4},{:>9.4},{:>9.4}) - ({:>9.4},{:>9.4},{:>9.4})",
                pu.x, pu.y, pu.z, pv.x, pv.y, pv.z
            );
        }
    }
}
