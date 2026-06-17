//! Diagnose why vertices aren't being deduplicated across faces in a BREP.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::{validate_watertight, mesh::VertexDedupMap};
use draper_geometry::Point3d;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).cloned().unwrap_or_else(|| "test/as1-oc-214.stp".to_string());
    let target_brep: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(63); // nut

    println!("Loading: {}", path);
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    println!("{} BREP instances", pending.len());

    let target = pending.iter().find(|p| p.brep_id == target_brep);
    if target.is_none() {
        println!("BREP #{} not found. Available:", target_brep);
        for p in &pending {
            println!("  BREP #{}: {}", p.brep_id, p.name);
        }
        return;
    }
    let pending = target.unwrap();

    let ctx = StepConversionContext::new(&step);
    let inst = ctx.triangulate_pending(pending).expect("triangulate");
    println!("\nBREP #{}: {} verts, {} tris", pending.brep_id, inst.mesh.vertex_count(), inst.mesh.triangle_count());

    let report = validate_watertight(&inst.mesh, true);
    println!("Watertight: {} (boundary={}, non-manifold={}, degen={})",
        report.is_watertight(), report.boundary_edge_count, report.non_manifold_edge_count, report.degenerate_triangle_count);

    // Collect all vertices with their face IDs
    let face_ids = inst.mesh.triangle_face_ids.as_ref();
    let mut vertex_to_faces: HashMap<u32, std::collections::HashSet<u64>> = HashMap::new();
    for (i, tri) in inst.mesh.triangles.iter().enumerate() {
        let fid = face_ids.and_then(|ids| ids.get(i).copied()).unwrap_or(0);
        for &v in tri {
            vertex_to_faces.entry(v).or_default().insert(fid);
        }
    }

    // Find vertices that appear in only 1 face (these are likely the "unique" ones
    // that should have been deduplicated)
    let single_face_verts: Vec<(u32, u64)> = vertex_to_faces.iter()
        .filter(|(_, faces)| faces.len() == 1)
        .map(|(&v, faces)| (v, *faces.iter().next().unwrap()))
        .collect();

    println!("\n{} vertices appear in only 1 face (should be 0 for a closed BREP)", single_face_verts.len());

    // For each single-face vertex, find the closest vertex from a DIFFERENT face
    let mut distance_buckets = [0usize; 10]; // [0, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1, 10, 100]
    let thresholds = [1e-15, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0];
    let mut closest_pairs: Vec<(f64, u32, u32)> = Vec::new();

    for &(v, face) in single_face_verts.iter().take(500) {
        let p = inst.mesh.vertices[v as usize];
        let mut closest_dist = f64::MAX;
        let mut closest_v = v;
        for &(v2, face2) in single_face_verts.iter() {
            if face2 == face { continue; } // Skip same face
            let p2 = inst.mesh.vertices[v2 as usize];
            let d = ((p.x - p2.x).powi(2) + (p.y - p2.y).powi(2) + (p.z - p2.z).powi(2)).sqrt();
            if d < closest_dist {
                closest_dist = d;
                closest_v = v2;
            }
        }
        if closest_dist < f64::MAX {
            for (i, &t) in thresholds.iter().enumerate() {
                if closest_dist <= t {
                    distance_buckets[i] += 1;
                    break;
                }
            }
            if closest_pairs.len() < 20 {
                closest_pairs.push((closest_dist, v, closest_v));
            }
        }
    }

    println!("\nDistance distribution (closest cross-face vertex):");
    let labels = ["0", "1e-6", "1e-5", "1e-4", "1e-3", "1e-2", "1e-1", "1", "10", "100"];
    for (i, count) in distance_buckets.iter().enumerate() {
        if *count > 0 {
            println!("  ≤{}: {} vertices", labels[i], count);
        }
    }

    println!("\nClosest cross-face vertex pairs (sample of 20):");
    closest_pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    for (d, v1, v2) in &closest_pairs {
        let p1 = inst.mesh.vertices[*v1 as usize];
        let p2 = inst.mesh.vertices[*v2 as usize];
        let f1 = vertex_to_faces.get(v1).and_then(|s| s.iter().next()).copied().unwrap_or(0);
        let f2 = vertex_to_faces.get(v2).and_then(|s| s.iter().next()).copied().unwrap_or(0);
        println!("  dist={:.2e}  v{} (face {}) ({:.3},{:.3},{:.3})  ↔  v{} (face {}) ({:.3},{:.3},{:.3})",
            d, v1, f1, p1.x, p1.y, p1.z, v2, f2, p2.x, p2.y, p2.z);
    }
}
