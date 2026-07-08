// Detailed investigation of bolt (BREP#1190) in as1-oc-214.stp
// Check which faces share edges and why merge_deduplicating fails
use draper_step::{parse_step, step_structure_lazy, StepConversionContext, StepValue};
use draper_mesh::validate_watertight;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/as1-oc-214_bolt.stp")
        .expect("read");
    let step = parse_step(&content).expect("parse");

    // Build vertex-pair → edge_curves map
    let mut vp_to_edges: HashMap<(i64, i64), Vec<i64>> = HashMap::new();
    for entity in step.find_entities_by_type("EDGE_CURVE") {
        let mut vids = Vec::new();
        let mut curve_type = String::new();
        for param in &entity.params {
            if let StepValue::Ref(ref_id) = param {
                if let Some(re) = step.find_entity(*ref_id) {
                    if re.type_name == "VERTEX_POINT" {
                        vids.push(*ref_id);
                    }
                    if matches!(re.type_name.as_str(),
                        "CIRCLE" | "LINE" | "B_SPLINE_CURVE" | "B_SPLINE_CURVE_WITH_KNOTS" |
                        "RATIONAL_B_SPLINE_CURVE" | "TRIMMED_CURVE" | "SURFACE_CURVE" |
                        "ELLIPSE" | "PARABOLA" | "HYPERBOLA") {
                        curve_type = re.type_name.clone();
                    }
                }
            }
        }
        if vids.len() >= 2 {
            let vp = (vids[0].min(vids[1]), vids[0].max(vids[1]));
            vp_to_edges.entry(vp).or_default().push(entity.id);
            println!("EDGE_CURVE #{}: {} v1=#{} v2=#{}", entity.id, curve_type, vids[0], vids[1]);
        }
    }

    println!("\n=== Vertex pairs with multiple edges ===");
    for (vp, edges) in &vp_to_edges {
        if edges.len() >= 2 {
            println!("  ({}, {}): {} edges {:?}", vp.0, vp.1, edges.len(), edges);
        }
    }

    // Now triangulate and check which faces have boundary edges
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            println!("\n=== BREP #{} ({}) ===", p.brep_id, p.name);
            println!("  v={} t={} watertight={} boundary={}",
                inst.mesh.vertex_count(), inst.mesh.triangle_count(),
                report.is_watertight(), report.boundary_edge_count);

            // For each face, show surface type and step_id
            for fi in &inst.faces {
                println!("  Face Step#{} {} (face_id={}) forward={}",
                    fi.step_face_id, fi.surface_type, fi.face_id, fi.forward);
            }

            if report.boundary_edge_count > 0 {
                // Find boundary edges and which faces they belong to
                let fids = inst.mesh.triangle_face_ids.as_ref();
                let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
                for tri in &inst.mesh.triangles {
                    let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
                    for (a, b) in edges {
                        let key = if a < b { (a, b) } else { (b, a) };
                        *edge_count.entry(key).or_insert(0) += 1;
                    }
                }

                // For boundary edges, compute distance between the two vertices
                let mut dists: Vec<f64> = Vec::new();
                for (edge, &count) in &edge_count {
                    if count == 1 {
                        let v0 = inst.mesh.vertices[edge.0 as usize];
                        let v1 = inst.mesh.vertices[edge.1 as usize];
                        let d = ((v0.x-v1.x).powi(2) + (v0.y-v1.y).powi(2) + (v0.z-v1.z).powi(2)).sqrt();
                        dists.push(d);
                    }
                }
                dists.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let n = dists.len();
                if n > 0 {
                    println!("  Boundary edge lengths: min={:.6} med={:.6} max={:.6}",
                        dists[0], dists[n/2], dists[n-1]);
                    // Find closest vertex for first boundary edge
                    let mut closest_dists: Vec<f64> = Vec::new();
                    for (edge, &count) in &edge_count {
                        if count == 1 {
                            let v0 = inst.mesh.vertices[edge.0 as usize];
                            let mut best = f64::MAX;
                            for (vi, v) in inst.mesh.vertices.iter().enumerate() {
                                if vi as u32 == edge.0 || vi as u32 == edge.1 { continue; }
                                let d = ((v0.x-v.x).powi(2) + (v0.y-v.y).powi(2) + (v0.z-v.z).powi(2)).sqrt();
                                if d < best { best = d; }
                            }
                            closest_dists.push(best);
                        }
                    }
                    closest_dists.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    println!("  Closest vertex distances: min={:.6} med={:.6} max={:.6}",
                        closest_dists[0], closest_dists[closest_dists.len()/2],
                        closest_dists[closest_dists.len()-1]);
                }
            }
        }
    }
}
