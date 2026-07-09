use std::fs;
use draper_step::{StepConversionContext, StepFile, parser};

fn main() {
    let step_content = fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("Failed to read 3.05.078.stp");
    
    let step_file = parser::parse_step_streaming(&step_content)
        .expect("Failed to parse STEP file");
    
    println!("Parsed STEP file: {} entities", step_file.entities.len());
    
    // Create conversion context
    let ctx = StepConversionContext::new(&step_file);
    
    // Get detailed mesh instances
    let instances = ctx.triangulate_all_detailed();
    
    for (inst_idx, inst) in instances.iter().enumerate() {
        println!("\n=== Instance #{}: {} ===", inst_idx, inst.name);
        println!("  Mesh: {} vertices, {} triangles", inst.mesh.vertices.len(), inst.mesh.triangles.len());
        
        for fi in &inst.face_infos {
            let step_id = fi.step_face_id;
            if step_id == 87 || step_id == 78 {
                println!("\n  --- Face Step#{} {} (face_id={}, forward={}) ---",
                    step_id, fi.surface_type, fi.face_id, fi.forward);
                println!("  triangle_range: {:?}", fi.triangle_range);
                println!("  outer_uv_boundary: {} polylines", fi.outer_uv_boundary.len());
                for (i, poly) in fi.outer_uv_boundary.iter().enumerate() {
                    println!("    polyline {}: {} points", i, poly.len());
                    if poly.len() <= 10 {
                        for p in poly {
                            println!("      ({:.6}, {:.6})", p.u, p.v);
                        }
                    } else {
                        println!("      first: ({:.6}, {:.6}), last: ({:.6}, {:.6})",
                            poly[0].u, poly[0].v, poly.last().unwrap().u, poly.last().unwrap().v);
                    }
                }
                println!("  inner_uv_boundaries: {} hole groups", fi.inner_uv_boundaries.len());
                for (i, hole_group) in fi.inner_uv_boundaries.iter().enumerate() {
                    for (j, poly) in hole_group.iter().enumerate() {
                        println!("    hole {}-{}: {} points", i, j, poly.len());
                    }
                }
                println!("  uv_triangles: {} triangles", fi.uv_triangles.len());
                
                // Check for degenerate triangles
                let mut degenerate_count = 0;
                let mut zero_area_count = 0;
                for tri in &fi.uv_triangles {
                    let dx1 = tri[1].u - tri[0].u;
                    let dy1 = tri[1].v - tri[0].v;
                    let dx2 = tri[2].u - tri[0].u;
                    let dy2 = tri[2].v - tri[0].v;
                    let area = (dx1 * dy2 - dx2 * dy1).abs() / 2.0;
                    if area < 1e-10 {
                        degenerate_count += 1;
                    }
                    if area < 1e-20 {
                        zero_area_count += 1;
                    }
                }
                println!("  degenerate_triangles (area < 1e-10): {}", degenerate_count);
                println!("  zero_area_triangles (area < 1e-20): {}", zero_area_count);
                
                // Print first few triangles
                let print_limit = 10.min(fi.uv_triangles.len());
                println!("  first {} triangles:", print_limit);
                for (i, tri) in fi.uv_triangles.iter().take(print_limit).enumerate() {
                    println!("    tri[{}]: ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                        i, tri[0].u, tri[0].v, tri[1].u, tri[1].v, tri[2].u, tri[2].v);
                }
                
                // Print actual 3D triangles for this face
                let (start, end) = fi.triangle_range;
                if start < end && end <= inst.mesh.triangles.len() {
                    println!("  3D triangles ({} total):", end - start);
                    let tri3d_print = 5.min(end - start);
                    for i in start..start+tri3d_print {
                        let tri = inst.mesh.triangles[i];
                        let v0 = inst.mesh.vertices[tri[0] as usize];
                        let v1 = inst.mesh.vertices[tri[1] as usize];
                        let v2 = inst.mesh.vertices[tri[2] as usize];
                        println!("    tri3d[{}]: ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4})",
                            i, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
                    }
                }
            }
        }
    }
}
