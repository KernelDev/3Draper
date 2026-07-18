// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Diagnostic: verify sewing tolerance computation for a STEP file.
//!
//! Lists all VERTEX_POINT entities, finds near-coincident pairs, and
//! compares with the actual mesh boundary vertex pairs.
//!
//! Usage: cargo run --bin diag_sew_tol --release [file.stp]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.iter().find(|a| !a.starts_with('-') && a.as_str() != args[0])
        .cloned()
        .unwrap_or_else(|| "test/brick_thin_round.stp".to_string());

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    println!("STEP file parsed: {} entities", step.entities.len());

    // ── Step 1: List all VERTEX_POINT entities and their 3D coords ──────
    let mut vertex_points: Vec<(i64, draper_geometry::Point3d)> = Vec::new();
    for entity in &step.entities {
        if entity.type_name != "VERTEX_POINT" { continue; }
        let id = entity.id;
        // Find the cartesian_point reference
        for param in &entity.params {
            if let draper_step::schema::StepValue::Ref(cp_ref) = param {
                if let Some(cp_entity) = step.find_entity(*cp_ref) {
                    for cp_param in &cp_entity.params {
                        if let draper_step::schema::StepValue::List(coords) = cp_param {
                            if coords.len() >= 2 {
                                let x = match &coords[0] {
                                    draper_step::schema::StepValue::Float(f) => *f,
                                    draper_step::schema::StepValue::Integer(i) => *i as f64,
                                    _ => continue,
                                };
                                let y = match &coords[1] {
                                    draper_step::schema::StepValue::Float(f) => *f,
                                    draper_step::schema::StepValue::Integer(i) => *i as f64,
                                    _ => continue,
                                };
                                let z = if coords.len() >= 3 {
                                    match &coords[2] {
                                        draper_step::schema::StepValue::Float(f) => *f,
                                        draper_step::schema::StepValue::Integer(i) => *i as f64,
                                        _ => 0.0,
                                    }
                                } else { 0.0 };
                                vertex_points.push((id, draper_geometry::Point3d::new(x, y, z)));
                                break;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    println!("\n=== VERTEX_POINT entities: {} ===", vertex_points.len());

    // ── Step 2: Find near-coincident pairs (within 1% of bbox diagonal) ──
    let (bmin, bmax) = vertex_points.iter().fold(
        (draper_geometry::Point3d::new(f64::MAX, f64::MAX, f64::MAX),
         draper_geometry::Point3d::new(f64::MIN, f64::MIN, f64::MIN)),
        |(mn, mx), (_, p)| {
            (draper_geometry::Point3d::new(mn.x.min(p.x), mn.y.min(p.y), mn.z.min(p.z)),
             draper_geometry::Point3d::new(mx.x.max(p.x), mx.y.max(p.y), mx.z.max(p.z)))
        });
    let dx = bmax.x - bmin.x;
    let dy = bmax.y - bmin.y;
    let dz = bmax.z - bmin.z;
    let model_scale = (dx*dx + dy*dy + dz*dz).sqrt().max(1e-10);
    println!("Bounding box: ({:.4},{:.4},{:.4})..({:.4},{:.4},{:.4})  model_scale={:.4}",
        bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z, model_scale);

    let seed_tol = model_scale * 1e-2;
    let seed_tol_sq = seed_tol * seed_tol;
    println!("Seed tolerance: {:.4e} (1% of model_scale)", seed_tol);

    let mut near_pairs: Vec<(i64, i64, f64)> = Vec::new();
    for i in 0..vertex_points.len() {
        for j in (i+1)..vertex_points.len() {
            let (id1, p1) = &vertex_points[i];
            let (id2, p2) = &vertex_points[j];
            let ddx = p1.x - p2.x;
            let ddy = p1.y - p2.y;
            let ddz = p1.z - p2.z;
            let dist_sq = ddx*ddx + ddy*ddy + ddz*ddz;
            if dist_sq > 1e-30 && dist_sq < seed_tol_sq {
                near_pairs.push((*id1, *id2, dist_sq.sqrt()));
            }
        }
    }
    near_pairs.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
    println!("\n=== Near-coincident VERTEX_POINT pairs: {} ===", near_pairs.len());
    for (id1, id2, dist) in near_pairs.iter().take(20) {
        let p1 = vertex_points.iter().find(|(id, _)| id == id1).map(|(_, p)| p).unwrap();
        let p2 = vertex_points.iter().find(|(id, _)| id == id2).map(|(_, p)| p).unwrap();
        println!("  #{} ↔ #{}: dist={:.4e}  ({:.6},{:.6},{:.6}) ↔ ({:.6},{:.6},{:.6})",
            id1, id2, dist, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
    }
    if near_pairs.is_empty() {
        // Show the smallest distances anyway
        let mut all_pairs: Vec<(i64, i64, f64)> = Vec::new();
        for i in 0..vertex_points.len() {
            for j in (i+1)..vertex_points.len() {
                let (id1, p1) = &vertex_points[i];
                let (id2, p2) = &vertex_points[j];
                let ddx = p1.x - p2.x;
                let ddy = p1.y - p2.y;
                let ddz = p1.z - p2.z;
                let dist_sq = ddx*ddx + ddy*ddy + ddz*ddz;
                if dist_sq > 1e-30 {
                    all_pairs.push((*id1, *id2, dist_sq.sqrt()));
                }
            }
        }
        all_pairs.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
        println!("\n  No near-pairs found. Smallest 20 distances:");
        for (id1, id2, dist) in all_pairs.iter().take(20) {
            let p1 = vertex_points.iter().find(|(id, _)| id == id1).map(|(_, p)| p).unwrap();
            let p2 = vertex_points.iter().find(|(id, _)| id == id2).map(|(_, p)| p).unwrap();
            println!("  #{} ↔ #{}: dist={:.4e}  ({:.6},{:.6},{:.6}) ↔ ({:.6},{:.6},{:.6})",
                id1, id2, dist, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
        }
    }

    // ── Step 3: Triangulate and check watertightness ────────────────────
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    println!("\n=== Triangulation result ===");
    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, false);
            let is_wt = report.is_watertight();
            println!("BREP #{}: {} verts, {} tris, {} boundary, {} non-manifold, watertight={}",
                inst.name, inst.mesh.vertex_count(), inst.mesh.triangle_count(),
                report.boundary_edge_count, report.non_manifold_edge_count, is_wt);

            // ── Step 4: Find near-coincident MESH vertex pairs ──────────
            // The gap might be in the mesh, not in the VERTEX_POINT entities.
            let mut mesh_near_pairs: Vec<(u32, u32, f64)> = Vec::new();
            let mesh_tol = model_scale * 1e-2;
            let mesh_tol_sq = mesh_tol * mesh_tol;
            for i in 0..inst.mesh.vertices.len() {
                for j in (i+1)..inst.mesh.vertices.len() {
                    let p1 = &inst.mesh.vertices[i];
                    let p2 = &inst.mesh.vertices[j];
                    let ddx = p1.x - p2.x;
                    let ddy = p1.y - p2.y;
                    let ddz = p1.z - p2.z;
                    let dist_sq = ddx*ddx + ddy*ddy + ddz*ddz;
                    if dist_sq > 1e-30 && dist_sq < mesh_tol_sq {
                        mesh_near_pairs.push((i as u32, j as u32, dist_sq.sqrt()));
                    }
                }
            }
            mesh_near_pairs.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
            println!("\n  Near-coincident MESH vertex pairs (within {:.4e}): {}",
                mesh_tol, mesh_near_pairs.len());
            for (vi1, vi2, dist) in mesh_near_pairs.iter().take(10) {
                let p1 = &inst.mesh.vertices[*vi1 as usize];
                let p2 = &inst.mesh.vertices[*vi2 as usize];
                println!("    v{} ↔ v{}: dist={:.4e}  ({:.6},{:.6},{:.6}) ↔ ({:.6},{:.6},{:.6})",
                    vi1, vi2, dist, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
            }

            let _ = i;
        }
    }
}
