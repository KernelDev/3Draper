// Directly compare step_id=236 edge cache points between cone and plane face processing
use draper_step::{parse_step, step_structure_lazy, StepConversionContext, StepStructure};
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_topology::Edge;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = "test/3.05.078.stp";
    let data = std::fs::read_to_string(path).expect("Failed to read file");
    let step = parse_step(&data).expect("Failed to parse");
    
    // Get the face data for both faces
    let (tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    // Process just like the converter does
    for p in &pending {
        let face_data_list = ctx.prepare_face_data(p);
        
        // Find Face#84 (Cone) and Face#87 (Plane)
        for face_data in &face_data_list {
            let is_cone = matches!(&face_data.surface, draper_geometry::Surface::Cone(_));
            let is_plane = matches!(&face_data.surface, draper_geometry::Surface::Plane(_));
            
            // Check if this face has step_id=236 in its boundary
            let has_236_outer = face_data.outer_edge_step_ids.iter().any(|&id| id == 236);
            let has_236_inner = face_data.inner_edge_step_ids.iter().any(|ids| ids.iter().any(|&id| id == 236));
            
            if has_236_outer || has_236_inner {
                println!("Face step_id={}: surface={:?}, has_236_outer={}, has_236_inner={}, outer_step_ids={:?}, inner_step_ids={:?}",
                    face_data.step_face_id,
                    if is_cone { "Cone" } else if is_plane { "Plane" } else { "Other" },
                    has_236_outer, has_236_inner,
                    face_data.outer_edge_step_ids,
                    face_data.inner_edge_step_ids,
                );
                
                // Show the edges that have step_id=236
                for (ei, edge) in face_data.outer_edges.iter().enumerate() {
                    let sid = face_data.outer_edge_step_ids.get(ei).copied().unwrap_or(0);
                    if sid == 236 {
                        println!("  Outer edge[{}]: step_id=236, param_range=({:.4},{:.4}), curve_type={:?}",
                            ei, edge.param_range.0, edge.param_range.1,
                            edge.curve.as_ref().map(|c| std::mem::discriminant(c)));
                    }
                }
                for (li, inner_edges) in face_data.inner_edges.iter().enumerate() {
                    for (ei, edge) in inner_edges.iter().enumerate() {
                        let sid = face_data.inner_edge_step_ids.get(li)
                            .and_then(|ids| ids.get(ei)).copied().unwrap_or(0);
                        if sid == 236 {
                            println!("  Inner edge[{}][{}]: step_id=236, param_range=({:.4},{:.4})",
                                li, ei, edge.param_range.0, edge.param_range.1);
                        }
                    }
                }
            }
        }
    }
}
