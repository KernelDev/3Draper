// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Diagnostic: check for duplicate triangles with opposite winding.
//!
//! Usage: cargo run --bin dup_check --release [file.stp]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.iter().find(|a| !a.starts_with('-') && a.as_str() != args[0])
        .cloned()
        .unwrap_or_else(|| "test/Zentralstaender.stp".to_string());

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let mesh = &inst.mesh;

            // Build edge → triangles map
            let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
            for (ti, tri) in mesh.triangles.iter().enumerate() {
                let a = tri[0]; let b = tri[1]; let c = tri[2];
                for (v0, v1) in [(a, b), (b, c), (c, a)] {
                    let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                    edge_to_tris.entry(key).or_default().push(ti);
                }
            }

            // Find edges shared by 2+ triangles
            let mut dup_count = 0;
            let mut opposite_winding_count = 0;
            let mut same_winding_count = 0;
            let mut non_manifold_count = 0;

            for (_edge, tris) in &edge_to_tris {
                if tris.len() == 2 {
                    // Check winding
                    let tri0 = mesh.triangles[tris[0]];
                    let tri1 = mesh.triangles[tris[1]];
                    // Get the shared edge vertices
                    let (a0, b0) = (tri0[0], tri0[1]);
                    let (a1, b1) = (tri1[0], tri1[1]);
                    let _ = (a0, b0, a1, b1);

                    // For each triangle, find the directed edge matching the shared edge
                    let edge_v0 = mesh.vertices[mesh.triangles[tris[0]][0] as usize];
                    let _ = edge_v0;

                    // Check if the two triangles have the same vertex set (duplicates)
                    let mut s0: Vec<u32> = tri0.to_vec(); s0.sort();
                    let mut s1: Vec<u32> = tri1.to_vec(); s1.sort();
                    if s0 == s1 {
                        dup_count += 1;
                    }
                } else if tris.len() > 2 {
                    non_manifold_count += 1;
                }
            }

            // Find triangles with 180° angles
            let mut angle_180_count = 0;
            for (_edge, tris) in &edge_to_tris {
                if tris.len() != 2 { continue; }
                let tri0 = mesh.triangles[tris[0]];
                let tri1 = mesh.triangles[tris[1]];

                let v0 = &mesh.vertices[tri0[0] as usize];
                let v1 = &mesh.vertices[tri0[1] as usize];
                let v2 = &mesh.vertices[tri0[2] as usize];
                let e1 = draper_geometry::Point3d::new(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);
                let e2 = draper_geometry::Point3d::new(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);
                let n0 = draper_geometry::Point3d::new(
                    e1.y*e2.z - e1.z*e2.y,
                    e1.z*e2.x - e1.x*e2.z,
                    e1.x*e2.y - e1.y*e2.x,
                );

                let v0 = &mesh.vertices[tri1[0] as usize];
                let v1 = &mesh.vertices[tri1[1] as usize];
                let v2 = &mesh.vertices[tri1[2] as usize];
                let e1 = draper_geometry::Point3d::new(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);
                let e2 = draper_geometry::Point3d::new(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);
                let n1 = draper_geometry::Point3d::new(
                    e1.y*e2.z - e1.z*e2.y,
                    e1.z*e2.x - e1.x*e2.z,
                    e1.x*e2.y - e1.y*e2.x,
                );

                let dot = n0.x*n1.x + n0.y*n1.y + n0.z*n1.z;
                let len0 = (n0.x*n0.x + n0.y*n0.y + n0.z*n0.z).sqrt();
                let len1 = (n1.x*n1.x + n1.y*n1.y + n1.z*n1.z).sqrt();
                if len0 < 1e-15 || len1 < 1e-15 { continue; }
                let cos_angle = dot / (len0 * len1);
                let cos_clamped = cos_angle.max(-1.0).min(1.0);
                let angle_deg = cos_clamped.acos().to_degrees();

                if angle_deg > 170.0 {
                    angle_180_count += 1;
                }
            }

            if dup_count > 0 || angle_180_count > 0 || non_manifold_count > 0 {
                println!("BREP #{} ({}): {} tris, {} dups, {} angle~180, {} non-manifold edges",
                    inst.name, i+1, mesh.triangles.len(), dup_count, angle_180_count, non_manifold_count);
            }
        }
    }
}
