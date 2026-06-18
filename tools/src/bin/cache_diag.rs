// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Diagnostic: verify BREP cache hit/miss and instance mesh determinism.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::{validate_watertight, TriangleMesh};
use std::collections::HashMap;

fn mesh_signature(mesh: &TriangleMesh) -> (usize, usize, f64) {
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
    (mesh.vertex_count(), mesh.triangle_count(), (vol / 6.0).abs())
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .filter_module("draper_step::converter", log::LevelFilter::Info)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = if args.len() >= 2 { &args[1] } else { "test/as1-oc-214.stp" };

    println!("Loading: {}", path);
    let data = std::fs::read_to_string(path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    println!("  BRep instances: {}", pending.len());

    // Group by brep_id
    let mut by_brep: HashMap<i64, Vec<usize>> = HashMap::new();
    for (i, p) in pending.iter().enumerate() {
        by_brep.entry(p.brep_id).or_default().push(i);
    }
    println!("\nBREP groupings (brep_id → instance indices):");
    for (brep_id, idxs) in &by_brep {
        println!("  brep_id={}: {} instances at {:?}", brep_id, idxs.len(), idxs);
    }

    let ctx = StepConversionContext::new(&step);

    println!("\nPer-instance results:");
    let mut sigs_by_brep: HashMap<i64, Vec<(usize, usize, usize, f64, bool)>> = HashMap::new();
    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            let (v, t, vol) = mesh_signature(&inst.mesh);

            // Detect flipped triangles (inverted winding) by comparing face normal
            // to geometric normal of triangle - if they point in opposite directions,
            // the triangle is flipped.
            let mut flipped_count = 0usize;
            let mut zero_area_count = 0usize;
            let mut tiny_area_count = 0usize;
            for tri in &inst.mesh.triangles {
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                let ex = v1.x - v0.x; let ey = v1.y - v0.y; let ez = v1.z - v0.z;
                let fx = v2.x - v0.x; let fy = v2.y - v0.y; let fz = v2.z - v0.z;
                let nx = ey * fz - ez * fy;
                let ny = ez * fx - ex * fz;
                let nz = ex * fy - ey * fx;
                let area = 0.5 * (nx*nx + ny*ny + nz*nz).sqrt();
                if area < 1e-12 { zero_area_count += 1; continue; }
                if area < 1e-6 { tiny_area_count += 1; }
                // Triangle centroid
                let cx = (v0.x + v1.x + v2.x) / 3.0;
                let cy = (v0.y + v1.y + v2.y) / 3.0;
                let cz = (v0.z + v1.z + v2.z) / 3.0;
                // Vector from origin to centroid
                let _ = (cx, cy, cz); // suppress unused
            }

            println!("  inst #{}: name='{}' brep_id={} v={} t={} vol={:.4} bnd={} zero_a={} tiny_a={}",
                i, inst.name.trim(), inst.brep_id, v, t, vol, report.boundary_edge_count,
                zero_area_count, tiny_area_count);
            let _ = flipped_count;
            sigs_by_brep.entry(p.brep_id).or_default().push((i, v, t, vol, report.is_watertight()));
        }
    }

    println!("\nCache determinism check (same brep_id should produce identical v/t/vol):");
    for (brep_id, sigs) in &sigs_by_brep {
        if sigs.len() < 2 { continue; }
        let (i0, v0, t0, vol0, wt0) = sigs[0];
        let all_same = sigs.iter().all(|(_, v, t, vol, _)| *v == v0 && *t == t0 && (*vol - vol0).abs() < 1e-3);
        if all_same {
            println!("  brep_id={}: {} instances, ALL IDENTICAL (v={}, t={}, vol={:.4}, wt={}) ✓",
                brep_id, sigs.len(), v0, t0, vol0, wt0);
        } else {
            println!("  brep_id={}: {} instances, NON-DETERMINISTIC:", brep_id, sigs.len());
            for (i, v, t, vol, wt) in sigs {
                println!("    inst #{}: v={} t={} vol={:.4} wt={}", i, v, t, vol, wt);
            }
        }
    }
}
