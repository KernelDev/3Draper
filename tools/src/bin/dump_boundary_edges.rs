// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Dump the boundary edges of a bolt (or any STEP file) to OBJ for visual inspection.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;
use std::io::Write;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = if args.len() >= 2 { &args[1] } else { "test/as1-oc-214.stp" };
    let target_brep: Option<i64> = args.get(2).and_then(|s| s.parse().ok());

    println!("Loading: {}", path);
    let data = std::fs::read_to_string(path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let ctx = StepConversionContext::new(&step);

    let mut obj_path = std::path::PathBuf::from(path);
    obj_path.set_extension("boundary_edges.obj");
    let mut f = std::fs::File::create(&obj_path).expect("create obj");

    writeln!(f, "# Boundary edges dump from {}", path).unwrap();

    let mut obj_vert_idx: u32 = 1; // OBJ is 1-indexed

    for (i, p) in pending.iter().enumerate() {
        if let Some(target) = target_brep {
            if p.brep_id != target { continue; }
        }
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            if report.boundary_edge_count == 0 { continue; }

            println!("\nInst #{}: name='{}' brep_id={} bnd={}",
                i, inst.name.trim(), inst.brep_id, report.boundary_edge_count);

            // Write boundary edges as OBJ line segments
            // First write vertices, then write lines
            let start_vert = obj_vert_idx;
            for v in &inst.mesh.vertices {
                writeln!(f, "v {:.6} {:.6} {:.6}", v.x, v.y, v.z).unwrap();
                obj_vert_idx += 1;
            }
            for (a, b) in &report.boundary_edges {
                let va = start_vert + *a as u32;
                let vb = start_vert + *b as u32;
                writeln!(f, "l {} {}", va, vb).unwrap();
            }

            // Also print the boundary edges with positions for the first 20
            let mut shown = 0;
            for (a, b) in &report.boundary_edges {
                let pa = inst.mesh.vertices[*a as usize];
                let pb = inst.mesh.vertices[*b as usize];
                let len = ((pa.x-pb.x).powi(2) + (pa.y-pb.y).powi(2) + (pa.z-pb.z).powi(2)).sqrt();
                println!("  edge v{}→v{} len={:.4}  ({:.3},{:.3},{:.3}) → ({:.3},{:.3},{:.3})",
                    a, b, len, pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
                shown += 1;
                if shown >= 30 { break; }
            }
            if report.boundary_edge_count > 30 {
                println!("  ... ({} more)", report.boundary_edge_count - 30);
            }
        }
    }

    println!("\nWrote boundary edges to: {}", obj_path.display());
}
