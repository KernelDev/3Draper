// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Check vertex point positions for specific VERTEX_POINT entities.
//!
//! Usage: cargo run --bin vertex_check

use draper_step::parse_step;
use draper_step::schema::{StepFile, StepValue};
use draper_geometry::point::Point3d;

/// Resolve a VERTEX_POINT entity to a 3D point.
fn resolve_vertex_point(step: &StepFile, vertex_id: i64) -> Option<Point3d> {
    let vertex_entity = step.find_entity(vertex_id)?;
    if vertex_entity.type_name != "VERTEX_POINT" {
        eprintln!("Entity #{} is not VERTEX_POINT, it's {}", vertex_id, vertex_entity.type_name);
        return None;
    }
    // VERTEX_POINT('', #point_ref)
    for param in &vertex_entity.params {
        if let StepValue::Ref(point_id) = param {
            if let Some(point) = resolve_cartesian_point(step, *point_id) {
                return Some(point);
            }
        }
    }
    None
}

/// Resolve a CARTESIAN_POINT entity to a 3D point.
fn resolve_cartesian_point(step: &StepFile, point_id: i64) -> Option<Point3d> {
    let entity = step.find_entity(point_id)?;
    for param in &entity.params {
        if let StepValue::List(coords) = param {
            let x = get_float(coords.get(0)?)?;
            let y = get_float(coords.get(1)?)?;
            let z = coords.get(2).and_then(|v| get_float(v)).unwrap_or(0.0);
            return Some(Point3d::new(x, y, z));
        }
    }
    None
}

fn get_float(value: &StepValue) -> Option<f64> {
    match value {
        StepValue::Float(f) => Some(*f),
        StepValue::Integer(i) => Some(*i as f64),
        _ => None,
    }
}

fn main() {
    let path = "test/as1-oc-214.stp";
    println!("Loading: {}", path);
    let data = std::fs::read_to_string(path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");

    // The vertex IDs we need to check:
    // - Plane face EDGE_CURVE #427 uses VERTEX_POINT #428 and #70
    // - NURBS face EDGE_CURVE #628 uses VERTEX_POINT #629 and #631
    let vertex_ids: [(i64, &str); 4] = [
        (70, "Plane face edge #427 vertex (end)"),
        (428, "Plane face edge #427 vertex (start)"),
        (629, "NURBS face edge #628 vertex (start)"),
        (631, "NURBS face edge #628 vertex (end)"),
    ];

    println!("\n=== VERTEX_POINT Positions ===\n");

    let mut positions: Vec<(i64, Point3d)> = Vec::new();

    for (id, desc) in &vertex_ids {
        match resolve_vertex_point(&step, *id) {
            Some(p) => {
                println!("  VERTEX_POINT #{} ({}) → ({:.15}, {:.15}, {:.15})", id, desc, p.x, p.y, p.z);
                positions.push((*id, p));
            }
            None => {
                eprintln!("  VERTEX_POINT #{} ({}) — FAILED to resolve!", id, desc);
            }
        }
    }

    // Also verify by printing the raw STEP entity
    println!("\n=== Raw STEP Entities ===\n");
    for (id, _) in &vertex_ids {
        if let Some(entity) = step.find_entity(*id) {
            println!("  #{} = {}({:?})", id, entity.type_name, entity.params);
        }
    }

    // Pairwise distance comparison
    println!("\n=== Pairwise Distances ===\n");
    for i in 0..positions.len() {
        for j in (i + 1)..positions.len() {
            let (id_a, pa) = positions[i];
            let (id_b, pb) = positions[j];
            let dx = pa.x - pb.x;
            let dy = pa.y - pb.y;
            let dz = pa.z - pb.z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            let close = if dist < 1e-3 { "CLOSE (< 1e-3)" } else if dist < 1e-1 { "NEAR (< 0.1)" } else { "FAR" };
            println!("  #{} ↔ #{}: distance = {:.6e}  [{}]", id_a, id_b, dist, close);
        }
    }

    // Summary
    println!("\n=== Summary ===\n");
    let min_dist = (0..positions.len())
        .flat_map(|i| (i+1..positions.len()).map(move |j| (i, j)))
        .map(|(i, j)| {
            let (_, pa) = positions[i];
            let (_, pb) = positions[j];
            let dx = pa.x - pb.x;
            let dy = pa.y - pb.y;
            let dz = pa.z - pb.z;
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .fold(f64::MAX, f64::min);

    println!("  Minimum pairwise distance: {:.6e}", min_dist);
    println!("  Typical STEP tolerance:    ~1e-6");
    println!("  Vertices are {}within tolerance of each other.", if min_dist < 1e-3 { "" } else { "NOT " });

    // Check cross-edge pairs specifically
    println!("\n=== Cross-Edge Comparison (Plane vs NURBS) ===\n");
    let plane_verts: Vec<(i64, Point3d)> = positions.iter().filter(|(id, _)| *id == 70 || *id == 428).cloned().collect();
    let nurbs_verts: Vec<(i64, Point3d)> = positions.iter().filter(|(id, _)| *id == 629 || *id == 631).cloned().collect();

    let mut closest_cross = f64::MAX;
    for (id_a, pa) in &plane_verts {
        for (id_b, pb) in &nurbs_verts {
            let dx = pa.x - pb.x;
            let dy = pa.y - pb.y;
            let dz = pa.z - pb.z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            println!("  Plane #{} ↔ NURBS #{}: distance = {:.6e}", id_a, id_b, dist);
            if dist < closest_cross {
                closest_cross = dist;
            }
        }
    }
    println!("\n  Closest cross-edge distance: {:.6e}", closest_cross);
    println!("  The Plane and NURBS faces do {}share geometrically close vertices.",
        if closest_cross < 1e-3 { "" } else { "NOT " });
}
