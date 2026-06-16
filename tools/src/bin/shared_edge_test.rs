// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Test that shared edges between Plane and NURBS faces produce bit-identical 3D points.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.iter().filter(|a| !a.starts_with('-')).nth(1)
        .cloned()
        .unwrap_or_else(|| "test/as1-oc-214.stp".to_string());
    let brep_index: usize = args.iter()
        .filter(|a| !a.starts_with('-'))
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);

    println!("Loading: {} (BREP index {})", path, brep_index);
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let ctx = StepConversionContext::new(&step);
    let p = &pending[brep_index];
    let result = ctx.triangulate_pending(p);
    let inst = match result {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); std::process::exit(2); }
    };

    println!("\nInstance: {} (BREP #{})", inst.name, p.brep_id);
    println!("Vertices: {}, Triangles: {}", inst.mesh.vertex_count(), inst.mesh.triangle_count());

    // Build vertex position → list of (vertex_index, face_ids) map
    let mut pos_to_vertices: HashMap<[u64; 3], Vec<(u32, std::collections::HashSet<u64>)>> = HashMap::new();
    if let Some(face_ids) = &inst.mesh.triangle_face_ids {
        let mut vert_to_faces: HashMap<u32, std::collections::HashSet<u64>> = HashMap::new();
        for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
            let fid = face_ids[ti];
            for &v in tri.iter() {
                vert_to_faces.entry(v).or_default().insert(fid);
            }
        }
        for (i, v) in inst.mesh.vertices.iter().enumerate() {
            let key = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
            let faces = vert_to_faces.get(&(i as u32)).cloned().unwrap_or_default();
            pos_to_vertices.entry(key).or_default().push((i as u32, faces));
        }
    }

    // Count positions shared between faces
    let mut shared_count = 0;
    let mut multi_face_positions = 0;
    for (_, vertices) in &pos_to_vertices {
        let all_faces: std::collections::HashSet<u64> = vertices.iter()
            .flat_map(|(_, fs)| fs.iter().copied())
            .collect();
        if all_faces.len() > 1 {
            multi_face_positions += 1;
        }
        if vertices.len() > 1 {
            shared_count += 1;
        }
    }
    println!("\nVertex sharing analysis:");
    println!("  Total unique positions: {}", pos_to_vertices.len());
    println!("  Positions with multiple vertex indices: {}", shared_count);
    println!("  Positions shared between multiple faces: {}", multi_face_positions);

    // Find vertices that should be shared but aren't
    // Look at boundary edges and check if their endpoints are at positions shared with other faces
    let report = draper_mesh::validate_watertight(&inst.mesh, true);
    println!("\nBoundary edges: {}, Non-manifold: {}", report.boundary_edge_count, report.non_manifold_edge_count);

    let mut boundary_with_shared_pos = 0;
    let mut boundary_without_shared_pos = 0;
    for (a, b) in &report.boundary_edges {
        let pa = inst.mesh.vertices[*a as usize];
        let pb = inst.mesh.vertices[*b as usize];
        let key_a = [pa.x.to_bits(), pa.y.to_bits(), pa.z.to_bits()];
        let key_b = [pb.x.to_bits(), pb.y.to_bits(), pb.z.to_bits()];
        let a_shared = pos_to_vertices.get(&key_a).map(|v| v.len() > 1).unwrap_or(false);
        let b_shared = pos_to_vertices.get(&key_b).map(|v| v.len() > 1).unwrap_or(false);
        if a_shared || b_shared {
            boundary_with_shared_pos += 1;
        } else {
            boundary_without_shared_pos += 1;
        }
    }
    println!("  Boundary edges with at least 1 shared endpoint: {}", boundary_with_shared_pos);
    println!("  Boundary edges with NO shared endpoints: {}", boundary_without_shared_pos);

    // Sample first 5 boundary edges with positions
    println!("\nFirst 5 boundary edges:");
    for (i, (a, b)) in report.boundary_edges.iter().take(5).enumerate() {
        let pa = inst.mesh.vertices[*a as usize];
        let pb = inst.mesh.vertices[*b as usize];
        let key_a = [pa.x.to_bits(), pa.y.to_bits(), pa.z.to_bits()];
        let key_b = [pb.x.to_bits(), pb.y.to_bits(), pb.z.to_bits()];
        let a_faces: Vec<u64> = pos_to_vertices.get(&key_a)
            .map(|v| v.iter().flat_map(|(_, fs)| fs.iter().copied()).collect::<std::collections::HashSet<_>>().into_iter().collect())
            .unwrap_or_default();
        let b_faces: Vec<u64> = pos_to_vertices.get(&key_b)
            .map(|v| v.iter().flat_map(|(_, fs)| fs.iter().copied()).collect::<std::collections::HashSet<_>>().into_iter().collect())
            .unwrap_or_default();
        println!("  {}: v({},{}) pa=({:.4},{:.4},{:.4}) faces_a={:?} | pb=({:.4},{:.4},{:.4}) faces_b={:?}",
            i, a, b, pa.x, pa.y, pa.z, a_faces, pb.x, pb.y, pb.z, b_faces);
    }
}
