use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

/// Resolve a test-data file relative to the workspace `test/` dir.
/// Robust to repo relocation: derived from CARGO_MANIFEST_DIR
/// (crates/draper-step -> workspace root), not from a hardcoded sandbox path.
fn test_file(name: &str) -> std::path::PathBuf {
    let mut dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    dir.pop(); // crates/draper-step -> crates
    dir.pop(); // crates -> workspace root
    dir.join("test").join(name)
}


/// Diagnostic for Step#78 (cone) and Step#87 (plane) 3D triangulation issues.
/// Checks:
/// 1. Whether the face triangulation has correct winding order
/// 2. Whether boundary vertices match between adjacent faces
/// 3. Whether there are duplicate/missing triangles
/// 4. Specific cone tube grid issues
#[test]
fn diag_step_78_87_detailed() {
    let content = std::fs::read_to_string(test_file("3.05.078.stp"))
        .expect("Failed to read 3.05.078.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    for (pi, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            eprintln!("=== Instance #{}: {} vertices, {} triangles ===", 
                pi, inst.mesh.vertices.len(), inst.mesh.triangles.len());
            
            // For each face, check detailed properties
            for fi in &inst.faces {
                let (start, end) = fi.triangle_range;
                if start >= end || end > inst.mesh.triangles.len() { continue; }
                
                // ─── Step#78 Cone analysis ───
                if fi.step_face_id == 78 {
                    eprintln!("\n=== Step#78 Cone (face_id={}, forward={}) ===", fi.face_id, fi.forward);
                    
                    if let draper_geometry::Surface::Cone(ref cone) = fi.surface {
                        eprintln!("  Cone: radius={:.4}, half_angle={:.2}deg, expanding={}, apex_v={:.4}",
                            cone.radius, cone.half_angle.to_degrees(), cone.expanding, cone.apex_v());
                        eprintln!("  origin=({:.4},{:.4},{:.4}), axis=({:.4},{:.4},{:.4})",
                            cone.origin.x, cone.origin.y, cone.origin.z,
                            cone.axis.x, cone.axis.y, cone.axis.z);
                    }
                    
                    // Check that 3D triangles form a valid cone surface
                    eprintln!("  3D triangles: {}", end - start);
                    
                    // Find vertices at boundary edges (shared with other faces)
                    let face_verts: std::collections::HashSet<u32> = (start..end)
                        .flat_map(|i| { let t = inst.mesh.triangles[i]; [t[0], t[1], t[2]] })
                        .collect();
                    
                    // Check for each boundary vertex, whether it's on the cone surface
                    let mut on_surface_count = 0;
                    let mut off_surface_count = 0;
                    for &vi in &face_verts {
                        let v = inst.mesh.vertices[vi as usize];
                        // Check if this point is on the cone surface
                        // The cone has: origin=(2.43,0,0), axis=(-1,0,0), half_angle=45deg
                        // A point on the cone surface satisfies:
                        // distance from axis / (v_param * tan(half_angle) + radius) ≈ 1
                        let dx = v.x - 2.43;
                        let dy = v.y - 0.0;
                        let dz = v.z - 0.0;
                        let v_param = -dx; // dot with axis (-1,0,0)
                        let r_actual = (dy*dy + dz*dz).sqrt();
                        let r_expected = 32.79 + v_param * (45.0_f64.to_radians().tan());
                        if r_expected > 0.0 && (r_actual - r_expected).abs() / r_expected < 0.01 {
                            on_surface_count += 1;
                        } else if r_expected <= 0.0 && r_actual < 0.1 {
                            on_surface_count += 1; // apex
                        } else {
                            off_surface_count += 1;
                            if off_surface_count <= 5 {
                                eprintln!("  OFF-SURFACE vertex vi={}: ({:.4},{:.4},{:.4}) r_actual={:.4} r_expected={:.4} v_param={:.4}",
                                    vi, v.x, v.y, v.z, r_actual, r_expected, v_param);
                            }
                        }
                    }
                    eprintln!("  On-surface vertices: {}/{}", on_surface_count, face_verts.len());
                    eprintln!("  Off-surface vertices: {}/{}", off_surface_count, face_verts.len());
                    
                    if off_surface_count > 0 {
                        eprintln!("  *** WARNING: {} vertices are off the cone surface!", off_surface_count);
                        eprintln!("  *** This means the 3D triangulation has vertices at wrong positions.");
                    }
                }
                
                // ─── Step#87 Plane analysis ───
                if fi.step_face_id == 87 {
                    eprintln!("\n=== Step#87 Plane (face_id={}, forward={}) ===", fi.face_id, fi.forward);
                    
                    // Check if all vertices lie on the expected plane
                    if let draper_geometry::Surface::Plane(ref plane) = fi.surface {
                        eprintln!("  Plane: origin=({:.4},{:.4},{:.4}) normal=({:.4},{:.4},{:.4})",
                            plane.origin.x, plane.origin.y, plane.origin.z,
                            plane.normal.x, plane.normal.y, plane.normal.z);
                        eprintln!("  u_dir=({:.4},{:.4},{:.4}) v_dir=({:.4},{:.4},{:.4})",
                            plane.u_dir.x, plane.u_dir.y, plane.u_dir.z,
                            plane.v_dir.x, plane.v_dir.y, plane.v_dir.z);
                        
                        // Check u_dir × v_dir direction
                        let cross_x = plane.u_dir.y * plane.v_dir.z - plane.u_dir.z * plane.v_dir.y;
                        let cross_y = plane.u_dir.z * plane.v_dir.x - plane.u_dir.x * plane.v_dir.z;
                        let cross_z = plane.u_dir.x * plane.v_dir.y - plane.u_dir.y * plane.v_dir.x;
                        eprintln!("  u_dir × v_dir = ({:.4},{:.4},{:.4})", cross_x, cross_y, cross_z);
                        eprintln!("  plane.normal = ({:.4},{:.4},{:.4})", plane.normal.x, plane.normal.y, plane.normal.z);
                        
                        let dot = cross_x * plane.normal.x + cross_y * plane.normal.y + cross_z * plane.normal.z;
                        eprintln!("  (u×v) · normal = {:.4} (should be >0 if they match)", dot);
                        
                        if dot < 0.0 {
                            eprintln!("  *** BUG: u_dir × v_dir points OPPOSITE to plane.normal!");
                            eprintln!("  *** This means the 2D projection INVERTS the polygon orientation.");
                            eprintln!("  *** The earcutr/ear-clip triangulation will produce triangles with WRONG winding.");
                            eprintln!("  *** The forward flag flip doesn't help because the 2D polygon is already inverted.");
                        }
                    }
                    
                    // Check 3D triangle normals
                    let mut correct_normal = 0;
                    let mut wrong_normal = 0;
                    for i in start..end {
                        let tri = inst.mesh.triangles[i];
                        let v0 = inst.mesh.vertices[tri[0] as usize];
                        let v1 = inst.mesh.vertices[tri[1] as usize];
                        let v2 = inst.mesh.vertices[tri[2] as usize];
                        
                        let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                        let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                        let nx = e1.1 * e2.2 - e1.2 * e2.1;
                        let ny = e1.2 * e2.0 - e1.0 * e2.2;
                        let nz = e1.0 * e2.1 - e1.1 * e2.0;
                        
                        // For forward=true, normal should match surface normal direction
                        // If surface normal is -x, then cross product should point in -x
                        if nx.abs() > 0.001 {
                            if fi.forward && nx < 0.0 { correct_normal += 1; }
                            else { wrong_normal += 1; }
                        }
                    }
                    eprintln!("  Triangles with correct normal: {}/{}", correct_normal, end - start);
                    eprintln!("  Triangles with wrong normal: {}/{}", wrong_normal, end - start);
                    
                    // Check: are 3D triangles the SAME as UV triangles?
                    // If UV count != 3D count, some triangles were removed
                    eprintln!("  UV triangles: {}, 3D triangles: {}", fi.uv_triangles.len(), end - start);
                    if fi.uv_triangles.len() != end - start {
                        eprintln!("  *** MISMATCH: {} triangles removed during post-processing",
                            fi.uv_triangles.len() as isize - (end - start) as isize);
                    }
                }
            }
        }
    }
}
