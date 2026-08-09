// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Diagnostic: check angles between adjacent triangles for outliers.
//!
//! For each interior edge (shared by exactly 2 triangles), compute the
//! dihedral angle between the two triangles' normals. Outliers (very sharp
//! angles) indicate triangulation errors — typically caused by:
//! - Triangles from different faces being incorrectly welded together
//! - T-junction repair creating bad triangles
//! - Mesh gap filling creating non-geometric triangles
//!
//! Usage: cargo run --bin angle_check --release [file.stp]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::Point3d;
use std::collections::HashMap;

fn triangle_normal(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> Option<Point3d> {
    let e1 = Point3d::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let e2 = Point3d::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    let nx = e1.y * e2.z - e1.z * e2.y;
    let ny = e1.z * e2.x - e1.x * e2.z;
    let nz = e1.x * e2.y - e1.y * e2.x;
    let len = (nx*nx + ny*ny + nz*nz).sqrt();
    if len < 1e-15 { return None; }
    Some(Point3d::new(nx/len, ny/len, nz/len))
}

fn dot(a: &Point3d, b: &Point3d) -> f64 {
    a.x*b.x + a.y*b.y + a.z*b.z
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.iter().find(|a| !a.starts_with('-') && a.as_str() != args[0])
        .cloned()
        .unwrap_or_else(|| "test/Zentralstaender.stp".to_string());

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    println!("STEP file parsed: {} entities", step.entities.len());

    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    let mut total_watertight = 0;
    let mut total_not_watertight = 0;
    let mut total_outlier_angles = 0;
    let mut total_interior_edges = 0;
    let mut total_sharp_angles = 0; // > 60 degrees
    let mut total_extreme_angles = 0; // > 90 degrees

    let mut per_brep_stats: Vec<(String, usize, usize, usize, f64)> = Vec::new();
    // (name, total_interior, sharp_count, extreme_count, max_angle_deg)

    println!("\n{:-<120}", "");
    println!("{:>4} {:>34} {:>10} {:>10} {:>10} {:>10} {:>12}",
        "#", "Name", "Interior", "Sharp>60", "Extreme>90", "MaxAngle°", "Outliers");
    println!("{:-<120}", "");

    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let mesh = &inst.mesh;
            let face_ids = mesh.triangle_face_ids.as_ref();

            // Build edge → triangles map (interior edges have exactly 2 triangles)
            let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
            for (ti, tri) in mesh.triangles.iter().enumerate() {
                let a = tri[0]; let b = tri[1]; let c = tri[2];
                for (v0, v1) in [(a, b), (b, c), (c, a)] {
                    let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                    edge_to_tris.entry(key).or_default().push(ti);
                }
            }

            // Compute triangle normals
            let normals: Vec<Option<Point3d>> = mesh.triangles.iter().map(|tri| {
                let v0 = &mesh.vertices[tri[0] as usize];
                let v1 = &mesh.vertices[tri[1] as usize];
                let v2 = &mesh.vertices[tri[2] as usize];
                triangle_normal(v0, v1, v2)
            }).collect();

            // For each interior edge (exactly 2 triangles), compute dihedral angle
            let mut angles: Vec<f64> = Vec::new();
            let mut max_angle: f64 = 0.0;
            let mut sharp_count = 0; // > 60 degrees
            let mut extreme_count = 0; // > 90 degrees
            let mut outlier_edges: Vec<(u32, u32, f64, [u32; 3], [u32; 3], u64, u64)> = Vec::new();

            for (edge, tris) in &edge_to_tris {
                if tris.len() != 2 { continue; }
                let n0 = match &normals[tris[0]] { Some(n) => n, None => continue };
                let n1 = match &normals[tris[1]] { Some(n) => n, None => continue };
                let d = dot(n0, n1);
                let d_clamped = d.max(-1.0).min(1.0);
                let angle_rad = d_clamped.acos();
                let angle_deg = angle_rad.to_degrees();
                angles.push(angle_deg);
                if angle_deg > max_angle { max_angle = angle_deg; }
                if angle_deg > 60.0 {
                    sharp_count += 1;
                    if angle_deg > 90.0 {
                        extreme_count += 1;
                        // Save details for outliers
                        let fid0 = face_ids.and_then(|ids| ids.get(tris[0]).copied()).unwrap_or(0);
                        let fid1 = face_ids.and_then(|ids| ids.get(tris[1]).copied()).unwrap_or(0);
                        outlier_edges.push((
                            edge.0, edge.1, angle_deg,
                            mesh.triangles[tris[0]],
                            mesh.triangles[tris[1]],
                            fid0, fid1,
                        ));
                    }
                }
            }

            let interior_count = angles.len();
            total_interior_edges += interior_count;
            total_sharp_angles += sharp_count;
            total_extreme_angles += extreme_count;

            // Use a simple outlier criterion: angles beyond 3σ from the mean
            let mean = if !angles.is_empty() {
                angles.iter().sum::<f64>() / angles.len() as f64
            } else { 0.0 };
            let variance = if angles.len() > 1 {
                angles.iter().map(|a| (a - mean).powi(2)).sum::<f64>() / angles.len() as f64
            } else { 0.0 };
            let std_dev = variance.sqrt();
            let threshold = mean + 3.0 * std_dev;
            let outlier_count = angles.iter().filter(|&&a| a > threshold && a > 30.0).count();
            total_outlier_angles += outlier_count;

            per_brep_stats.push((inst.name.clone(), interior_count, sharp_count, extreme_count, max_angle));

            let is_wt = interior_count > 0 && extreme_count == 0;
            if is_wt { total_watertight += 1; } else { total_not_watertight += 1; }

            println!("{:>4} {:>34} {:>10} {:>10} {:>10} {:>10.2} {:>12}",
                i + 1,
                inst.name,
                interior_count,
                sharp_count,
                extreme_count,
                max_angle,
                outlier_count,
            );

            // Print details for worst outliers
            if !outlier_edges.is_empty() {
                outlier_edges.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
                for (v0, v1, ang, t0, t1, fid0, fid1) in outlier_edges.iter().take(5) {
                    let p0 = &mesh.vertices[*v0 as usize];
                    let p1 = &mesh.vertices[*v1 as usize];
                    let mid = Point3d::new(
                        (p0.x + p1.x) / 2.0,
                        (p0.y + p1.y) / 2.0,
                        (p0.z + p1.z) / 2.0,
                    );
                    let same_face = if fid0 == fid1 { "SAME-FACE" } else { "cross-face" };
                    // Flag only truly extreme angles (>170°) as "BAD"
                    let flag = if *ang > 170.0 { " *** BAD ***" } else { "" };
                    println!("      outlier: edge({},{}) angle={:.2}° faces=[{},{}] {}{} midpoint=({:.3},{:.3},{:.3})  tris=[{},{},{}]/[{},{},{}]",
                        v0, v1, ang, fid0, fid1, same_face, flag,
                        mid.x, mid.y, mid.z,
                        t0[0], t0[1], t0[2],
                        t1[0], t1[1], t1[2],
                    );
                }
            }
        }
    }

    println!("{:-<120}", "");
    println!("\nSummary:");
    println!("  Total BREPs:            {}", pending.len());
    println!("  Interior edges total:   {}", total_interior_edges);
    println!("  Sharp angles (>60°):    {} ({:.2}%)", total_sharp_angles,
        100.0 * total_sharp_angles as f64 / total_interior_edges.max(1) as f64);
    println!("  Extreme angles (>90°):  {} ({:.2}%)", total_extreme_angles,
        100.0 * total_extreme_angles as f64 / total_interior_edges.max(1) as f64);
    println!("  Statistical outliers:   {}", total_outlier_angles);

    // Count truly extreme angles (>170°) — these are actual bugs
    // (back-to-back triangles with opposite normals)
    let mut truly_extreme = 0;
    for (_, _, _, _, max_ang) in &per_brep_stats {
        if *max_ang > 170.0 { truly_extreme += 1; }
    }

    if truly_extreme == 0 {
        println!("\n✓ PASS: No truly extreme angles (>170°) detected");
        std::process::exit(0);
    } else {
        println!("\n✗ FAIL: {} BREPs with truly extreme angles (>170°)", truly_extreme);
        std::process::exit(1);
    }
}
