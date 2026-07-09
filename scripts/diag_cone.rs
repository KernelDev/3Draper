// Diagnostic script: analyze cone face #78 from 3.05.078.stp
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::{Surface, ConeSurface};

fn main() {
    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("Failed to read 3.05.078.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            for fi in &inst.faces {
                if fi.step_face_id == 78 || fi.step_face_id == 87 {
                    println!("\n=== Face Step#{} {} (face_id={}, forward={}) ===",
                        fi.step_face_id, fi.surface_type, fi.face_id, fi.forward);
                    
                    if let Surface::Cone(ref cone) = fi.surface {
                        println!("  Cone: origin=({:.4},{:.4},{:.4})", 
                            cone.origin.x, cone.origin.y, cone.origin.z);
                        println!("  Cone: axis=({:.4},{:.4},{:.4})",
                            cone.axis.x, cone.axis.y, cone.axis.z);
                        println!("  Cone: radius={:.6}, half_angle={:.6} rad ({:.2}°)",
                            cone.radius, cone.half_angle, cone.half_angle.to_degrees());
                        println!("  Cone: expanding={}, apex_v={:.6}", cone.expanding, cone.apex_v());
                        println!("  Cone: height={:.6}", cone.height());
                        println!("  Cone: x_dir=({:.4},{:.4},{:.4})",
                            cone.x_dir.x, cone.x_dir.y, cone.x_dir.z);
                        
                        // Check what the apex 3D point is
                        let apex_point = cone.point_at(0.0, cone.apex_v());
                        println!("  Apex 3D point: ({:.4},{:.4},{:.4})",
                            apex_point.x, apex_point.y, apex_point.z);
                        
                        // Check a few points on the cone
                        for v_test in [-2.43, -1.0, 0.0, 1.0, 2.43] {
                            let r = cone.radius_at(v_test);
                            let p = cone.point_at(0.0, v_test);
                            println!("  At v={:.2}: radius={:.6}, point=({:.4},{:.4},{:.4})",
                                v_test, r, p.x, p.y, p.z);
                        }
                    }
                    
                    // Analyze UV structure
                    let mut u_min = f64::MAX;
                    let mut u_max = f64::MIN;
                    let mut v_min = f64::MAX;
                    let mut v_max = f64::MIN;
                    for tri in &fi.uv_triangles {
                        for p in tri {
                            u_min = u_min.min(p.u);
                            u_max = u_max.max(p.u);
                            v_min = v_min.min(p.v);
                            v_max = v_max.max(p.v);
                        }
                    }
                    println!("  UV range: u=[{:.6}, {:.6}], v=[{:.6}, {:.6}]", u_min, u_max, v_min, v_max);
                    
                    // Check for v=0 row (apex)
                    let mut v0_triangles = 0;
                    for tri in &fi.uv_triangles {
                        for p in tri {
                            if p.v.abs() < 0.001 {
                                // This vertex is near v=0 (apex)
                            }
                        }
                        // Count triangles that span across v=0
                        let vs: Vec<f64> = tri.iter().map(|p| p.v).collect();
                        let has_neg = vs.iter().any(|v| *v < -0.001);
                        let has_pos = vs.iter().any(|v| *v > 0.001);
                        let has_zero = vs.iter().any(|v| v.abs() < 0.001);
                        if has_zero && (has_neg || has_pos) {
                            v0_triangles += 1;
                        }
                    }
                    println!("  Triangles touching v≈0 (apex): {}", v0_triangles);
                    println!("  Total triangles: {}", fi.uv_triangles.len());
                    
                    // Check 3D triangle range
                    let (start, end) = fi.triangle_range;
                    println!("  triangle_range: ({}, {}), mesh total: {} triangles",
                        start, end, inst.mesh.triangles.len());
                    if end > inst.mesh.triangles.len() {
                        println!("  *** BUG: triangle_range END exceeds mesh size! ***");
                    }
                    
                    // Check 3D degenerate triangles for this face
                    if end <= inst.mesh.triangles.len() && start < end {
                        let mut deg3d = 0;
                        for i in start..end {
                            let tri = inst.mesh.triangles[i];
                            let v0 = inst.mesh.vertices[tri[0] as usize];
                            let v1 = inst.mesh.vertices[tri[1] as usize];
                            let v2 = inst.mesh.vertices[tri[2] as usize];
                            let ab = (v0.x - v1.x).powi(2) + (v0.y - v1.y).powi(2) + (v0.z - v1.z).powi(2);
                            let bc = (v1.x - v2.x).powi(2) + (v1.y - v2.y).powi(2) + (v1.z - v2.z).powi(2);
                            let ac = (v0.x - v2.x).powi(2) + (v0.y - v2.y).powi(2) + (v0.z - v2.z).powi(2);
                            if ab < 1e-10 || bc < 1e-10 || ac < 1e-10 {
                                deg3d += 1;
                            }
                        }
                        println!("  3D degenerate triangles (edge² < 1e-10): {}", deg3d);
                    }
                }
            }
        }
    }
}
