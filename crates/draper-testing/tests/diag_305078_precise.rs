// Precise diagnostic for 3.05.078.stp using triangle_face_ids
use draper_step::{parse_step_file, step_to_detailed_instances_with_config, StepConversionConfig};
use draper_geometry::Surface;
use std::collections::HashMap;

#[test]
fn test_305078_precise() {
    let step_file = parse_step_file("../../test/3.05.078.stp")
        .expect("Failed to parse");
    let config = StepConversionConfig::default();
    let result = step_to_detailed_instances_with_config(&step_file, &config)
        .expect("Failed");

    for instance in &result {
        let mesh = &instance.mesh;
        
        // Build face_id → step_face_id mapping
        let mut face_id_to_step_id: HashMap<u64, i64> = HashMap::new();
        for fi in &instance.faces {
            face_id_to_step_id.insert(fi.face_id, fi.step_face_id);
        }
        
        // Build face_id → surface mapping
        let mut face_id_to_surface: HashMap<u64, Surface> = HashMap::new();
        for fi in &instance.faces {
            face_id_to_surface.insert(fi.face_id, fi.surface.clone());
        }
        
        // Use triangle_face_ids to group triangles by face
        let face_ids = mesh.triangle_face_ids.as_ref();
        
        // Count triangles per step_face_id
        let mut step_id_tri_count: HashMap<i64, usize> = HashMap::new();
        let mut step_id_off_surface: HashMap<i64, usize> = HashMap::new();
        let mut step_id_total_checked: HashMap<i64, usize> = HashMap::new();
        
        if let Some(fids) = face_ids {
            for (ti, &fid) in fids.iter().enumerate() {
                let step_id = face_id_to_step_id.get(&fid).copied().unwrap_or(-1);
                *step_id_tri_count.entry(step_id).or_insert(0) += 1;
                
                if step_id != 78 && step_id != 87 { continue; }
                
                // Check if this triangle is on the correct surface
                if let Some(surface) = face_id_to_surface.get(&fid) {
                    let tri = mesh.triangles[ti];
                    let v0 = mesh.vertices[tri[0] as usize];
                    let v1 = mesh.vertices[tri[1] as usize];
                    let v2 = mesh.vertices[tri[2] as usize];
                    
                    *step_id_total_checked.entry(step_id).or_insert(0) += 1;
                    
                    match surface {
                        Surface::Cone(cone) => {
                            for v in &[v0, v1, v2] {
                                let (u, v_param) = cone.project_point(v);
                                let p_recon = cone.point_at(u, v_param);
                                let dist = ((v.x - p_recon.x).powi(2) + (v.y - p_recon.y).powi(2) + (v.z - p_recon.z).powi(2)).sqrt();
                                if dist > 0.1 {
                                    *step_id_off_surface.entry(step_id).or_insert(0) += 1;
                                }
                            }
                        }
                        Surface::Plane(plane) => {
                            for v in &[v0, v1, v2] {
                                let dx = v.x - plane.origin.x;
                                let dy = v.y - plane.origin.y;
                                let dz = v.z - plane.origin.z;
                                let dist = (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).abs();
                                if dist > 0.01 {
                                    *step_id_off_surface.entry(step_id).or_insert(0) += 1;
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
        
        println!("=== Per-face triangle counts (using triangle_face_ids) ===");
        for fi in &instance.faces {
            let count = step_id_tri_count.get(&fi.step_face_id).copied().unwrap_or(0);
            let checked = step_id_total_checked.get(&fi.step_face_id).copied().unwrap_or(0);
            let off = step_id_off_surface.get(&fi.step_face_id).copied().unwrap_or(0);
            println!("  Step#{} ({}): tris_in_mesh={} checked={} off_surface={}",
                fi.step_face_id, fi.surface_type, count, checked, off);
        }
        
        // For cone face, print specific off-surface triangles
        if let Some(fids) = face_ids {
            println!("\n=== Cone face (Step#78) detailed ===");
            for (ti, &fid) in fids.iter().enumerate() {
                let step_id = face_id_to_step_id.get(&fid).copied().unwrap_or(-1);
                if step_id != 78 { continue; }
                
                if let Some(surface) = face_id_to_surface.get(&fid) {
                    if let Surface::Cone(cone) = surface {
                        let tri = mesh.triangles[ti];
                        let v0 = mesh.vertices[tri[0] as usize];
                        let v1 = mesh.vertices[tri[1] as usize];
                        let v2 = mesh.vertices[tri[2] as usize];
                        
                        for v in &[v0, v1, v2] {
                            let (u, v_param) = cone.project_point(v);
                            let p_recon = cone.point_at(u, v_param);
                            let dist = ((v.x - p_recon.x).powi(2) + (v.y - p_recon.y).powi(2) + (v.z - p_recon.z).powi(2)).sqrt();
                            if dist > 0.1 {
                                println!("  OFF-SURFACE tri[{}]: v=({:.4},{:.4},{:.4}) uv=({:.4},{:.4}) recon=({:.4},{:.4},{:.4}) dist={:.4}",
                                    ti, v.x, v.y, v.z, u, v_param, p_recon.x, p_recon.y, p_recon.z, dist);
                            }
                        }
                    }
                }
            }
        }
    }
}
