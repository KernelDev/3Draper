// Extended diagnostic test for 3.05.078.stp — detailed triangulation analysis
// Run: cargo test --package draper-testing --test diag_305078_v2 -- --nocapture 2>&1

use draper_step::{parse_step_file, step_to_detailed_instances_with_config, StepConversionConfig};
use draper_geometry::Surface;

#[test]
fn test_305078_detailed_diagnostic() {
    let step_file = parse_step_file("../../test/3.05.078.stp")
        .expect("Failed to parse 3.05.078.stp");

    let config = StepConversionConfig::default();
    let result = step_to_detailed_instances_with_config(&step_file, &config)
        .expect("Failed to triangulate");

    println!("=== 3.05.078.stp Detailed Diagnostic ===");

    for (instance_idx, instance) in result.iter().enumerate() {
        println!("\n--- Instance {} ---", instance_idx);
        println!("Triangle count: {}", instance.mesh.triangle_count());
        println!("Vertex count: {}", instance.mesh.vertex_count());

        for (fi, face_info) in instance.faces.iter().enumerate() {
            let step_id = face_info.step_face_id;
            
            // Only analyze key faces
            if step_id != 78 && step_id != 87 { continue; }
            
            let (tri_start, tri_end) = face_info.triangle_range;
            let tri_count = tri_end.saturating_sub(tri_start);

            println!("\n=== FACE step_id={} surface={:?} forward={} triangles={} ===",
                step_id, face_info.surface_type, face_info.forward, tri_count);

            // Analyze the surface
            {
                let surface = &face_info.surface;
                match surface {
                    Surface::Cone(cone) => {
                        println!("  Cone: origin=({:.4},{:.4},{:.4}) axis=({:.4},{:.4},{:.4})",
                            cone.origin.x, cone.origin.y, cone.origin.z,
                            cone.axis.x, cone.axis.y, cone.axis.z);
                        println!("  Cone: radius={:.4} half_angle={:.4} ({:.1}°) expanding={}",
                            cone.radius, cone.half_angle, cone.half_angle.to_degrees(), cone.expanding);
                        println!("  Cone: x_dir=({:.4},{:.4},{:.4})",
                            cone.x_dir.x, cone.x_dir.y, cone.x_dir.z);
                        let y_dir = cone.axis.cross(&cone.x_dir);
                        println!("  Cone: y_dir=({:.4},{:.4},{:.4}) (computed as axis × x_dir)",
                            y_dir.x, y_dir.y, y_dir.z);
                        println!("  Cone: apex_v={:.4} height={:.4}", cone.apex_v(), cone.height());
                        
                        // Verify point_at and project_point roundtrip
                        let test_points = [
                            (0.0, 0.0), (std::f64::consts::PI / 2.0, 0.0),
                            (std::f64::consts::PI, 0.0), (0.0, 2.43),
                            (0.0, -2.43), (std::f64::consts::PI, 2.43),
                        ];
                        println!("  Point_at / project_point roundtrip:");
                        for (u, v) in &test_points {
                            let p = cone.point_at(*u, *v);
                            let (u2, v2) = cone.project_point(&p);
                            let u_err = (u2 - *u).abs();
                            let v_err = (v2 - *v).abs();
                            // Handle u wrap-around
                            let u_err = if u_err > std::f64::consts::PI {
                                (u_err - 2.0 * std::f64::consts::PI).abs()
                            } else {
                                u_err
                            };
                            println!("    ({:.4},{:.4}) → 3d=({:.2},{:.2},{:.2}) → uv=({:.4},{:.4}) err=({:.2e},{:.2e})",
                                u, v, p.x, p.y, p.z, u2, v2, u_err, v_err);
                        }
                        
                        // Check boundary points are on the cone surface
                        println!("  Boundary points on-cone check (first 5):");
                        for (bi, bdry) in face_info.outer_boundary.iter().enumerate() {
                            for (pi, p) in bdry.iter().take(5).enumerate() {
                                let (u, v) = cone.project_point(p);
                                let p_recon = cone.point_at(u, v);
                                let dist = (p.x - p_recon.x).abs().max((p.y - p_recon.y).abs()).max((p.z - p_recon.z).abs());
                                let r_at_v = if cone.expanding {
                                    v * cone.half_angle.tan()
                                } else {
                                    (cone.radius + v * cone.half_angle.tan()).max(0.0)
                                };
                                let dx = p.x - cone.origin.x;
                                let dy = p.y - cone.origin.y;
                                let dz = p.z - cone.origin.z;
                                let radial_dist = (dx * dx + dy * dy + dz * dz).sqrt()
                                    - (v * v).sqrt(); // approximate
                                println!("    P{}: ({:.4},{:.4},{:.4}) uv=({:.4},{:.4}) recon_err={:.2e} r_at_v={:.4}",
                                    pi, p.x, p.y, p.z, u, v, dist, r_at_v);
                            }
                        }
                        
                        // Check normal directions
                        println!("  Normal check at u=0, v=0:");
                        let n = cone.normal_at(0.0, 0.0);
                        println!("    Analytical normal: ({:.6},{:.6},{:.6})", n.x, n.y, n.z);
                        let n_oriented = if face_info.forward { n.clone() } else {
                            draper_geometry::Direction3d::new(-n.x, -n.y, -n.z).unwrap_or(n.clone())
                        };
                        println!("    Oriented (forward={}): ({:.6},{:.6},{:.6})", face_info.forward, n_oriented.x, n_oriented.y, n_oriented.z);
                    }
                    Surface::Plane(plane) => {
                        println!("  Plane: origin=({:.4},{:.4},{:.4}) normal=({:.4},{:.4},{:.4})",
                            plane.origin.x, plane.origin.y, plane.origin.z,
                            plane.normal.x, plane.normal.y, plane.normal.z);
                        println!("  Plane: u_dir=({:.4},{:.4},{:.4}) v_dir=({:.4},{:.4},{:.4})",
                            plane.u_dir.x, plane.u_dir.y, plane.u_dir.z,
                            plane.v_dir.x, plane.v_dir.y, plane.v_dir.z);
                        
                        // Check boundary points are on the plane
                        println!("  Boundary points on-plane check:");
                        for (bi, bdry) in face_info.outer_boundary.iter().enumerate() {
                            let mut max_dist = 0.0f64;
                            for p in bdry {
                                let dx = p.x - plane.origin.x;
                                let dy = p.y - plane.origin.y;
                                let dz = p.z - plane.origin.z;
                                let dist = (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).abs();
                                max_dist = max_dist.max(dist);
                            }
                            println!("    Outer boundary {}: max distance from plane = {:.2e}", bi, max_dist);
                        }
                        for (bi, bdry) in face_info.inner_boundaries.iter().enumerate() {
                            let mut max_dist = 0.0f64;
                            for p in bdry {
                                let dx = p.x - plane.origin.x;
                                let dy = p.y - plane.origin.y;
                                let dz = p.z - plane.origin.z;
                                let dist = (dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z).abs();
                                max_dist = max_dist.max(dist);
                            }
                            println!("    Inner boundary {}: max distance from plane = {:.2e}", bi, max_dist);
                        }
                    }
                    _ => {}
                }
            }

            // Analyze UV triangles
            let uv_tri_count = face_info.uv_triangles.len();
            println!("\n  UV triangles: {}", uv_tri_count);
            
            // Check for degenerate UV triangles (zero area)
            let mut degen_count = 0;
            let mut huge_count = 0;
            for tri in &face_info.uv_triangles {
                let area2 = (tri[1].u - tri[0].u) * (tri[2].v - tri[0].v)
                          - (tri[1].v - tri[0].v) * (tri[2].u - tri[0].u);
                if area2.abs() < 1e-12 {
                    degen_count += 1;
                }
                if area2.abs() > 100.0 {
                    huge_count += 1;
                }
            }
            println!("  Degenerate UV triangles (area < 1e-12): {}", degen_count);
            println!("  Huge UV triangles (area > 100): {}", huge_count);

            // Check UV triangles coverage
            let mut u_min = f64::MAX;
            let mut u_max = f64::MIN;
            let mut v_min = f64::MAX;
            let mut v_max = f64::MIN;
            for tri in &face_info.uv_triangles {
                for pt in tri {
                    u_min = u_min.min(pt.u);
                    u_max = u_max.max(pt.u);
                    v_min = v_min.min(pt.v);
                    v_max = v_max.max(pt.v);
                }
            }
            println!("  UV triangle range: u=[{:.4},{:.4}] v=[{:.4},{:.4}]", u_min, u_max, v_min, v_max);

            // Check 3D triangles for correctness
            let vertices = &instance.mesh.vertices;
            let triangles = &instance.mesh.triangles;
            if tri_start < triangles.len() {
                let mut outside_surface_count = 0;
                let mut total_checked = 0;
                for ti in tri_start..tri_end.min(triangles.len()) {
                    let [i0, i1, i2] = triangles[ti];
                    if (i0 as usize) >= vertices.len() || (i1 as usize) >= vertices.len() || (i2 as usize) >= vertices.len() {
                        continue;
                    }
                    let v0 = vertices[i0 as usize];
                    let v1 = vertices[i1 as usize];
                    let v2 = vertices[i2 as usize];
                    
                    {
                        let surface = &face_info.surface;
                        total_checked += 1;
                        match surface {
                            Surface::Cone(cone) => {
                                // Check if vertices are on the cone surface
                                for v in &[v0, v1, v2] {
                                    let (u, v_param) = cone.project_point(v);
                                    let p_recon = cone.point_at(u, v_param);
                                    let dist = ((v.x - p_recon.x).powi(2) + (v.y - p_recon.y).powi(2) + (v.z - p_recon.z).powi(2)).sqrt();
                                    if dist > 0.1 {
                                        outside_surface_count += 1;
                                        if outside_surface_count <= 3 {
                                            println!("    OFF-SURFACE: v=({:.4},{:.4},{:.4}) uv=({:.4},{:.4}) recon=({:.4},{:.4},{:.4}) dist={:.4}",
                                                v.x, v.y, v.z, u, v_param, p_recon.x, p_recon.y, p_recon.z, dist);
                                        }
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
                                        outside_surface_count += 1;
                                        if outside_surface_count <= 3 {
                                            println!("    OFF-PLANE: v=({:.4},{:.4},{:.4}) dist={:.4}", v.x, v.y, v.z, dist);
                                        }
                                    }
                                }
                            }
                            _ => {}
                        }
                    }
                }
                if total_checked > 0 {
                    println!("  3D triangles checked: {}, off-surface: {} ({:.1}%)",
                        total_checked, outside_surface_count,
                        outside_surface_count as f64 / total_checked as f64 * 100.0);
                }
            }
        }
    }

    println!("\n=== Diagnostic Complete ===");
}
