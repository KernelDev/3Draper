// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Detailed per-face analysis for a single STEP file.
//!
//! Usage: face_analysis <step.stp>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::{validate_watertight, TriangleMesh};

fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let mut vol = 0.0;
    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let cx = v1.y * v2.z - v1.z * v2.y;
        let cy = v1.z * v2.x - v1.x * v2.z;
        let cz = v1.x * v2.y - v1.y * v2.x;
        vol += v0.x * cx + v0.y * cy + v0.z * cz;
    }
    vol / 6.0
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = if args.len() >= 2 { &args[1] } else { "test/as1-oc-214_bolt.stp" };

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(path).expect("read");
    let step = parse_step(&data).expect("parse");
    println!("  entities: {}", step.entities.len());

    let (_tree, pending) = step_structure_lazy(&step);
    println!("  BRep instances: {}", pending.len());

    let ctx = StepConversionContext::new(&step);

    for (i, p) in pending.iter().enumerate() {
        let result = ctx.triangulate_pending(p);
        match result {
            Some(inst) => {
                let report = validate_watertight(&inst.mesh, true);
                let vol = mesh_volume(&inst.mesh).abs();
                println!("\n=== BRep #{}: name='{}' brep_id={} ===", i + 1, inst.name, inst.brep_id);
                println!("  verts={} tris={} vol={:.4} watertight={}",
                    inst.mesh.vertex_count(), inst.mesh.triangle_count(), vol, report.is_watertight());
                println!("  boundary={} non_manf={} degenerate={} duplicate={} χ={}",
                    report.boundary_edge_count, report.non_manifold_edge_count,
                    report.degenerate_triangle_count, report.duplicate_triangle_count,
                    report.euler_characteristic);

                // Print per-face summary
                println!("\n  Per-face summary (sorted by boundary edges):");
                let mut faces: Vec<_> = report.per_face_summary.iter().collect();
                faces.sort_by_key(|(_, s)| std::cmp::Reverse(s.boundary_edge_count));
                for (fid, s) in &faces {
                    println!("    Face #{:3}: {:5} tris, {:5} boundary edges", fid, s.triangle_count, s.boundary_edge_count);
                }
            }
            None => println!("BRep #{}: FAILED", i + 1),
        }
    }
}
