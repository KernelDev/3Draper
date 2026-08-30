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


/// Check: what is the plane normal for Step#87 and does it make sense?
#[test]
fn diag_step87_plane_normal() {
    let content = std::fs::read_to_string(test_file("3.05.078.stp"))
        .expect("Failed to read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            for fi in &inst.faces {
                if fi.step_face_id != 87 { continue; }
                
                eprintln!("\n=== Face Step#87 ===");
                eprintln!("  forward={}", fi.forward);
                eprintln!("  surface_type={}", fi.surface_type);
                
                // Print the surface details
                if let draper_geometry::Surface::Plane(ref plane) = fi.surface {
                    eprintln!("  plane.origin=({:.4},{:.4},{:.4})", plane.origin.x, plane.origin.y, plane.origin.z);
                    eprintln!("  plane.normal=({:.6},{:.6},{:.6})", plane.normal.x, plane.normal.y, plane.normal.z);
                    eprintln!("  plane.u_dir=({:.6},{:.6},{:.6})", plane.u_dir.x, plane.u_dir.y, plane.u_dir.z);
                    eprintln!("  plane.v_dir=({:.6},{:.6},{:.6})", plane.v_dir.x, plane.v_dir.y, plane.v_dir.z);
                }
                
                let (start, end) = fi.triangle_range;
                
                // Check face_normals 
                if let Some(ref face_normals) = inst.mesh.face_normals {
                    let fn0 = face_normals[start];
                    eprintln!("  face_normals[{}]=({:.6},{:.6},{:.6})", start, fn0[0], fn0[1], fn0[2]);
                }
                
                // Cross product of first triangle
                let tri = inst.mesh.triangles[start];
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                
                let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                let nx = e1.1 * e2.2 - e1.2 * e2.1;
                let ny = e1.2 * e2.0 - e1.0 * e2.2;
                let nz = e1.0 * e2.1 - e1.1 * e2.0;
                eprintln!("  cross_product[{}]=({:.6},{:.6},{:.6})", start, nx, ny, nz);
                
                // For a plane at x=0 with forward=true, the normal should point +x
                // But cross product says -x, and face_normals also says -x
                // This means either:
                // 1. The plane normal itself is -x (and forward=true doesn't negate it)
                // 2. Something went wrong in the CCW normalization + winding swap
                if let Some(ref face_normals) = inst.mesh.face_normals {
                    let an = face_normals[start];
                    eprintln!("\n  If plane is at x=0 and forward=true:");
                    eprintln!("    Expected face normal: +x (1,0,0)");
                    eprintln!("    Actual face normal: ({:.1},{:.1},{:.1})", an[0], an[1], an[2]);
                    eprintln!("    Cross product: ({:.1},{:.1},{:.1})", nx.signum(), ny.signum(), nz.signum());
                }
                
                // Check: does the face go through the converter path or the mesh path?
                // Also check all plane faces
            }
            
            // Check ALL plane faces
            for fi in &inst.faces {
                let (start, end) = fi.triangle_range;
                if start >= end { continue; }
                
                if let draper_geometry::Surface::Plane(ref plane) = fi.surface {
                    let fn0 = inst.mesh.face_normals.as_ref().map(|n| n[start]);
                    let tri = inst.mesh.triangles[start];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];
                    let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    let cx = e1.1 * e2.2 - e1.2 * e2.1;
                    let cy = e1.2 * e2.0 - e1.0 * e2.2;
                    let cz = e1.0 * e2.1 - e1.1 * e2.0;
                    
                    let expected_normal = if fi.forward { 
                        format!("({:.4},{:.4},{:.4})", plane.normal.x, plane.normal.y, plane.normal.z)
                    } else { 
                        format!("({:.4},{:.4},{:.4})", -plane.normal.x, -plane.normal.y, -plane.normal.z)
                    };
                    
                    eprintln!("Plane Step#{} forward={}: plane.normal=({:.4},{:.4},{:.4}) expected_face_normal={} face_normal={:?} cross=({:.4},{:.4},{:.4})",
                        fi.step_face_id, fi.forward, plane.normal.x, plane.normal.y, plane.normal.z,
                        expected_normal, fn0.map(|n| format!("({:.4},{:.4},{:.4})", n[0], n[1], n[2])),
                        cx, cy, cz);
                }
            }
        }
    }
}
