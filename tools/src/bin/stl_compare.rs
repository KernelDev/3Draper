// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Compare STEP triangulation with a reference STL file.
//!
//! Usage: stl_compare <step.stp> <ref.stl>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::{validate_watertight, TriangleMesh};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
struct V3 { x: f64, y: f64, z: f64 }

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

fn count_edges(mesh: &TriangleMesh) -> (usize, usize, usize) {
    // returns (boundary, non_manifold, total_unique_edges)
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        for k in 0..3 {
            let a = tri[k].min(tri[(k+1)%3]);
            let b = tri[k].max(tri[(k+1)%3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let boundary = edge_count.values().filter(|&&c| c == 1).count();
    let non_manifold = edge_count.values().filter(|&&c| c > 2).count();
    let total = edge_count.len();
    (boundary, non_manifold, total)
}

fn bbox(mesh: &TriangleMesh) -> ([f64;3], [f64;3]) {
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for v in &mesh.vertices {
        if v.x < mn[0] { mn[0] = v.x; }
        if v.y < mn[1] { mn[1] = v.y; }
        if v.z < mn[2] { mn[2] = v.z; }
        if v.x > mx[0] { mx[0] = v.x; }
        if v.y > mx[1] { mx[1] = v.y; }
        if v.z > mx[2] { mx[2] = v.z; }
    }
    (mn, mx)
}

// Parse binary STL (returns vertices + triangles as TriangleMesh-like data)
fn parse_binary_stl(data: &[u8]) -> Option<(Vec<[f64;3]>, Vec<[u32;3]>)> {
    // Binary STL: 80-byte header, 4-byte triangle count, then per triangle:
    //   12 bytes normal, 36 bytes 3 floats vertices, 2 bytes attr
    if data.len() < 84 { return None; }
    let n_tris = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
    let expected = 84 + n_tris * 50;
    if data.len() < expected {
        eprintln!("STL size mismatch: have {}, expected {} for {} tris", data.len(), expected, n_tris);
        return None;
    }
    let mut verts: Vec<[f64;3]> = Vec::with_capacity(n_tris * 3);
    let mut tris: Vec<[u32;3]> = Vec::with_capacity(n_tris);
    let mut off = 84;
    for _ in 0..n_tris {
        off += 12; // skip normal
        let mut tri = [0u32; 3];
        for j in 0..3 {
            let bx = &data[off..off+12];
            let x = f32::from_le_bytes([bx[0],bx[1],bx[2],bx[3]]) as f64;
            let y = f32::from_le_bytes([bx[4],bx[5],bx[6],bx[7]]) as f64;
            let z = f32::from_le_bytes([bx[8],bx[9],bx[10],bx[11]]) as f64;
            tri[j] = verts.len() as u32;
            verts.push([x, y, z]);
            off += 12;
        }
        tris.push(tri);
        off += 2;
    }
    Some((verts, tris))
}

fn parse_ascii_stl(data: &str) -> Option<(Vec<[f64;3]>, Vec<[u32;3]>)> {
    let mut verts: Vec<[f64;3]> = Vec::new();
    let mut tris: Vec<[u32;3]> = Vec::new();
    let mut current: Vec<u32> = Vec::new();
    for line in data.lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() { continue; }
        if parts[0] == "vertex" && parts.len() >= 4 {
            let x = parts[1].parse::<f64>().ok()?;
            let y = parts[2].parse::<f64>().ok()?;
            let z = parts[3].parse::<f64>().ok()?;
            current.push(verts.len() as u32);
            verts.push([x, y, z]);
            if current.len() == 3 {
                tris.push([current[0], current[1], current[2]]);
                current.clear();
            }
        }
    }
    if tris.is_empty() { None } else { Some((verts, tris)) }
}

fn stl_volume_and_edges(verts: &[[f64;3]], tris: &[[u32;3]]) -> (f64, usize, usize) {
    let mut vol = 0.0;
    for t in tris {
        let v0 = verts[t[0] as usize];
        let v1 = verts[t[1] as usize];
        let v2 = verts[t[2] as usize];
        let cx = v1[1] * v2[2] - v1[2] * v2[1];
        let cy = v1[2] * v2[0] - v1[0] * v2[2];
        let cz = v1[0] * v2[1] - v1[1] * v2[0];
        vol += v0[0] * cx + v0[1] * cy + v0[2] * cz;
    }
    vol /= 6.0;

    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for t in tris {
        for k in 0..3 {
            let a = t[k].min(t[(k+1)%3]);
            let b = t[k].max(t[(k+1)%3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    let boundary = edge_count.values().filter(|&&c| c == 1).count();
    let non_manifold = edge_count.values().filter(|&&c| c > 2).count();
    (vol.abs(), boundary, non_manifold)
}

fn stl_bbox(verts: &[[f64;3]]) -> ([f64;3], [f64;3]) {
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for v in verts {
        for i in 0..3 {
            if v[i] < mn[i] { mn[i] = v[i]; }
            if v[i] > mx[i] { mx[i] = v[i]; }
        }
    }
    (mn, mx)
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: stl_compare <step.stp> <ref.stl>");
        std::process::exit(2);
    }
    let step_path = &args[1];
    let stl_path = &args[2];

    println!("=== STEP FILE: {} ===", step_path);
    let data = std::fs::read_to_string(step_path).expect("read step");
    let step = parse_step(&data).expect("parse step");
    println!("  entities: {}", step.entities.len());

    let start = std::time::Instant::now();
    let (_tree, pending) = step_structure_lazy(&step);
    println!("  BRep instances: {} (structure: {:.1?})", pending.len(), start.elapsed());

    let ctx = StepConversionContext::new(&step);

    let mut step_verts_total = 0usize;
    let mut step_tris_total = 0usize;
    let mut step_vol_total = 0.0;
    let mut step_boundary_total = 0usize;
    let mut step_nonmanf_total = 0usize;
    let mut step_watertight_count = 0usize;

    for (i, p) in pending.iter().enumerate() {
        let t0 = std::time::Instant::now();
        let result = ctx.triangulate_pending(p);
        let dt = t0.elapsed();
        match result {
            Some(inst) => {
                let report = validate_watertight(&inst.mesh, true);
                let is_wt = report.is_watertight();
                if is_wt { step_watertight_count += 1; }
                let vol = mesh_volume(&inst.mesh).abs();
                let (boundary, non_manifold, _total) = count_edges(&inst.mesh);
                let (mn, mx) = bbox(&inst.mesh);
                println!("  BRep #{}: name='{}' brep_id={} verts={} tris={} vol={:.4} boundary={} ({}%) non_manf={} wt={} ({:.1?})",
                    i + 1, inst.name, inst.brep_id, inst.mesh.vertex_count(), inst.mesh.triangle_count(),
                    vol, boundary,
                    if inst.mesh.triangle_count() > 0 { 100.0 * boundary as f64 / (inst.mesh.triangle_count() * 3) as f64 } else { 0.0 },
                    non_manifold, is_wt, dt);
                println!("    bbox: mn=[{:.3},{:.3},{:.3}] mx=[{:.3},{:.3},{:.3}]",
                    mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]);
                if !is_wt {
                    let mut bad_faces: Vec<_> = report.per_face_summary.iter()
                        .filter(|(_, s)| s.boundary_edge_count > 0)
                        .collect();
                    bad_faces.sort_by_key(|(_, s)| std::cmp::Reverse(s.boundary_edge_count));
                    for (fid, s) in bad_faces.iter().take(8) {
                        println!("    Face #{}: {} tris, {} boundary edges",
                            fid, s.triangle_count, s.boundary_edge_count);
                    }
                }
                step_verts_total += inst.mesh.vertex_count();
                step_tris_total += inst.mesh.triangle_count();
                step_vol_total += vol;
                step_boundary_total += boundary;
                step_nonmanf_total += non_manifold;
            }
            None => {
                println!("  BRep #{}: FAILED to triangulate", i + 1);
            }
        }
    }

    println!("\n--- STEP Totals ---");
    println!("  verts: {}, tris: {}, volume: {:.4}", step_verts_total, step_tris_total, step_vol_total);
    println!("  boundary edges: {} ({}%), non-manifold: {}",
        step_boundary_total,
        if step_tris_total > 0 { 100.0 * step_boundary_total as f64 / (step_tris_total * 3) as f64 } else { 0.0 },
        step_nonmanf_total);
    println!("  watertight: {}/{}", step_watertight_count, pending.len());

    // ============ STL REFERENCE ============
    println!("\n=== STL REFERENCE: {} ===", stl_path);
    let stl_data = std::fs::read(stl_path).expect("read stl");
    let (stl_verts, stl_tris) = if let Some((v, t)) = parse_binary_stl(&stl_data) {
        println!("  format: binary");
        (v, t)
    } else if let Ok(s) = std::str::from_utf8(&stl_data) {
        match parse_ascii_stl(s) {
            Some((v, t)) => { println!("  format: ascii"); (v, t) }
            None => { eprintln!("STL parse failed"); std::process::exit(3); }
        }
    } else {
        eprintln!("STL parse failed (neither binary nor ascii)"); std::process::exit(3);
    };
    println!("  verts: {}, tris: {}", stl_verts.len(), stl_tris.len());
    let (stl_vol, stl_boundary, stl_nonmanf) = stl_volume_and_edges(&stl_verts, &stl_tris);
    let (smn, smx) = stl_bbox(&stl_verts);
    println!("  volume: {:.4}", stl_vol);
    println!("  boundary edges: {} ({}%), non-manifold: {}",
        stl_boundary,
        if !stl_tris.is_empty() { 100.0 * stl_boundary as f64 / (stl_tris.len() * 3) as f64 } else { 0.0 },
        stl_nonmanf);
    println!("  bbox: mn=[{:.3},{:.3},{:.3}] mx=[{:.3},{:.3},{:.3}]",
        smn[0], smn[1], smn[2], smx[0], smx[1], smx[2]);

    // ============ COMPARISON ============
    println!("\n=== COMPARISON ===");
    let vol_ratio = if stl_vol > 0.0 { step_vol_total / stl_vol } else { 0.0 };
    println!("  Volume ratio (step/stl): {:.4}", vol_ratio);
    if (vol_ratio - 1.0).abs() < 0.01 {
        println!("  -> Volume MATCH (within 1%)");
    } else if (vol_ratio - 1.0).abs() < 0.05 {
        println!("  -> Volume CLOSE (within 5%)");
    } else {
        println!("  -> Volume MISMATCH (>5% off)");
    }

    let stl_wt = stl_boundary == 0;
    println!("  STL watertight: {}", stl_wt);
    println!("  STEP watertight: {}/{}", step_watertight_count, pending.len());
}
