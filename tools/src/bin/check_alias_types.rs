use draper_step::{parser::parse_step, StepConverter};
use draper_geometry::Curve3d;
use std::collections::HashMap;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    let content = std::fs::read_to_string("test/brick_thin_round.stp").expect("read");
    let step_file = parse_step(&content).expect("parse");
    let converter = StepConverter::new(&step_file);
    let (_tree, pending) = draper_step::step_structure_lazy(&step_file);

    for p in &pending {
        let fd = converter.extract_face_data(p.brep_id);
        let fd = match fd { Some(v) => v, None => continue };
        let mut vp_to_edges: HashMap<(i64, i64), Vec<(i64, String)>> = HashMap::new();
        {
            for (ei, edge) in fd.outer_edges.iter().enumerate() {
                let sid = fd.outer_edge_step_ids.get(ei).copied().unwrap_or(0);
                if sid == 0 { continue; }
                if let Some(vp) = converter.get_edge_curve_vertex_pair(sid) {
                    let curve_type = match &edge.curve {
                        Some(Curve3d::Line(_)) => "Line",
                        Some(Curve3d::Circle(_)) => "Circle",
                        Some(Curve3d::Nurbs(_)) => "Nurbs",
                        _ => "Other",
                    };
                    let key = if vp.0 < vp.1 { vp } else { (vp.1, vp.0) };
                    vp_to_edges.entry(key).or_default().push((sid, curve_type.to_string()));
                }
            }
        }

        let mut diff_type_count = 0;
        let mut same_type_count = 0;
        for (vp, edges) in &vp_to_edges {
            if edges.len() < 2 { continue; }
            let types: Vec<&str> = edges.iter().map(|(_, t)| t.as_str()).collect();
            let all_same_type = types.iter().all(|&t| t == types[0]);
            if !all_same_type {
                diff_type_count += 1;
                if diff_type_count <= 5 {
                    println!("VP {:?}: {} edges, types={:?}", vp, edges.len(), types);
                }
            } else {
                same_type_count += 1;
            }
        }
        println!("Same type pairs: {}, Different type pairs: {}", same_type_count, diff_type_count);
        break;
    }
}
