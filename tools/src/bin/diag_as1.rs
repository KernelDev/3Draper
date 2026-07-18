// Diagnostic: load as1-oc-214.stp and check watertightness + triangulation
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = "/home/z/my-project/test/as1-oc-214.stp";
    let content = std::fs::read_to_string(path).expect("read step file");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    println!("=== as1-oc-214.stp ===");
    println!("Pending BREPs: {}", pending.len());

    let mut total_boundary = 0;
    let mut total_interior = 0;
    let mut total_tris = 0;
    let mut total_verts = 0;

    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            let is_wt = report.is_watertight();
            total_boundary += report.boundary_edge_count;
            total_interior += report.interior_edge_count;
            total_tris += inst.mesh.triangle_count();
            total_verts += inst.mesh.vertex_count();

            println!("\n--- BREP #{} ({}) instance {} ---", p.brep_id, p.name, i);
            println!("  verts={} tris={} watertight={} boundary={} interior={}",
                inst.mesh.vertex_count(), inst.mesh.triangle_count(),
                is_wt, report.boundary_edge_count, report.interior_edge_count);

            if !is_wt && report.boundary_edge_count > 0 {
                // Show boundary edge length distribution
                let mut edge_count_map: HashMap<(u32, u32), usize> = HashMap::new();
                for tri in &inst.mesh.triangles {
                    let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                    for (a, b) in edges {
                        let key = if a < b { (a, b) } else { (b, a) };
                        *edge_count_map.entry(key).or_insert(0) += 1;
                    }
                }
                let mut lengths: Vec<f64> = Vec::new();
                for (edge, &count) in &edge_count_map {
                    if count == 1 {
                        let v0 = inst.mesh.vertices[edge.0 as usize];
                        let v1 = inst.mesh.vertices[edge.1 as usize];
                        let d = ((v0.x-v1.x).powi(2) + (v0.y-v1.y).powi(2) + (v0.z-v1.z).powi(2)).sqrt();
                        lengths.push(d);
                    }
                }
                if !lengths.is_empty() {
                    lengths.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    let n = lengths.len();
                    println!("  Boundary edge lengths: min={:.6} med={:.6} max={:.6}",
                        lengths[0], lengths[n/2], lengths[n-1]);
                    let fp = lengths.iter().filter(|&&l| l < 1e-6).count();
                    let small = lengths.iter().filter(|&&l| l >= 1e-6 && l < 0.1).count();
                    let medium = lengths.iter().filter(|&&l| l >= 0.1 && l < 1.0).count();
                    let large = lengths.iter().filter(|&&l| l >= 1.0).count();
                    println!("  FP drift (<1e-6): {} small (<0.1): {} medium (<1): {} large (>=1): {}",
                        fp, small, medium, large);
                }
            }
        }
    }

    println!("\n=== TOTAL ===");
    println!("  verts={} tris={} boundary={} interior={}", total_verts, total_tris, total_boundary, total_interior);
    println!("  watertight: {}", if total_boundary == 0 { "YES" } else { "NO" });
}
