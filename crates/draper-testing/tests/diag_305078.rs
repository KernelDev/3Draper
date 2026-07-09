// Diagnostic test for 3.05.078.stp — Plane#87 and Cone#78 triangulation issues
// Run: cargo test --package draper-testing --test diag_305078 -- --nocapture 2>&1

use draper_step::{parse_step_file, step_to_detailed_instances_with_config, StepConversionConfig};

#[test]
fn test_305078_diagnostic() {
    let step_file = parse_step_file("../../test/3.05.078.stp")
        .expect("Failed to parse 3.05.078.stp");

    println!("=== 3.05.078.stp Diagnostic ===");

    let config = StepConversionConfig::default();
    let result = step_to_detailed_instances_with_config(&step_file, &config)
        .expect("Failed to triangulate");

    println!("Total instances: {}", result.len());

    for (instance_idx, instance) in result.iter().enumerate() {
        println!("\n--- Instance {} ---", instance_idx);
        println!("Triangle count: {}", instance.mesh.triangle_count());
        println!("Vertex count: {}", instance.mesh.vertex_count());
        println!("Face count: {}", instance.faces.len());

        for (fi, face_info) in instance.faces.iter().enumerate() {
            let step_id = face_info.step_face_id;
            let forward = face_info.forward;
            let (tri_start, tri_end) = face_info.triangle_range;
            let tri_count = tri_end.saturating_sub(tri_start);

            println!(
                "  F#{}: step_id={} surface={:?} forward={} triangles=[{},{}]={} outer_bdry={} inner_bdry={}",
                fi, step_id, face_info.surface_type, forward,
                tri_start, tri_end, tri_count,
                face_info.outer_boundary.len(),
                face_info.inner_boundaries.len(),
            );

            // Highlight step IDs 78 and 87
            if step_id == 78 || step_id == 87 {
                println!("    *** KEY FACE (Step#{}) ***", step_id);

                // Print outer boundary info
                for (bi, bdry) in face_info.outer_boundary.iter().enumerate() {
                    println!("    Outer boundary {}: {} points", bi, bdry.len());
                    if bdry.len() <= 12 {
                        for (pi, p) in bdry.iter().enumerate() {
                            println!("      P{}: ({:.4}, {:.4}, {:.4})", pi, p.x, p.y, p.z);
                        }
                    } else {
                        println!("      First: ({:.4}, {:.4}, {:.4})", bdry[0].x, bdry[0].y, bdry[0].z);
                        println!("      Last:  ({:.4}, {:.4}, {:.4})", bdry.last().unwrap().x, bdry.last().unwrap().y, bdry.last().unwrap().z);
                    }
                }

                // Print inner boundary info (holes)
                for (bi, bdry) in face_info.inner_boundaries.iter().enumerate() {
                    println!("    Inner boundary (hole) {}: {} points", bi, bdry.len());
                    if bdry.len() <= 12 {
                        for (pi, p) in bdry.iter().enumerate() {
                            println!("      P{}: ({:.4}, {:.4}, {:.4})", pi, p.x, p.y, p.z);
                        }
                    } else {
                        println!("      First: ({:.4}, {:.4}, {:.4})", bdry[0].x, bdry[0].y, bdry[0].z);
                        println!("      Last:  ({:.4}, {:.4}, {:.4})", bdry.last().unwrap().x, bdry.last().unwrap().y, bdry.last().unwrap().z);
                    }
                }

                // Print UV boundary info
                for (bi, uv_bdry) in face_info.outer_uv_boundary.iter().enumerate() {
                    println!("    Outer UV boundary {}: {} points", bi, uv_bdry.len());
                    if uv_bdry.len() <= 8 {
                        for (pi, p) in uv_bdry.iter().enumerate() {
                            println!("      UV{}: ({:.6}, {:.6})", pi, p.u, p.v);
                        }
                    } else {
                        let u_min = uv_bdry.iter().map(|p| p.u).fold(f64::INFINITY, f64::min);
                        let u_max = uv_bdry.iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
                        let v_min = uv_bdry.iter().map(|p| p.v).fold(f64::INFINITY, f64::min);
                        let v_max = uv_bdry.iter().map(|p| p.v).fold(f64::NEG_INFINITY, f64::max);
                        println!("      UV range: u=[{:.4}, {:.4}] v=[{:.4}, {:.4}]", u_min, u_max, v_min, v_max);
                    }
                }

                for (bi, uv_holes) in face_info.inner_uv_boundaries.iter().enumerate() {
                    for (hi, uv_bdry) in uv_holes.iter().enumerate() {
                        println!("    Inner UV boundary (hole) {}.{}: {} points", bi, hi, uv_bdry.len());
                        if uv_bdry.len() <= 8 {
                            for (pi, p) in uv_bdry.iter().enumerate() {
                                println!("      UV{}: ({:.6}, {:.6})", pi, p.u, p.v);
                            }
                        } else {
                            let u_min = uv_bdry.iter().map(|p| p.u).fold(f64::INFINITY, f64::min);
                            let u_max = uv_bdry.iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
                            let v_min = uv_bdry.iter().map(|p| p.v).fold(f64::INFINITY, f64::min);
                            let v_max = uv_bdry.iter().map(|p| p.v).fold(f64::NEG_INFINITY, f64::max);
                            println!("      UV range: u=[{:.4}, {:.4}] v=[{:.4}, {:.4}]", u_min, u_max, v_min, v_max);
                        }
                    }
                }

                // Print UV triangles for the face
                let uv_tri_count = face_info.uv_triangles.len();
                println!("    UV triangles count: {}", uv_tri_count);
                if uv_tri_count <= 20 {
                    for (ti, tri) in face_info.uv_triangles.iter().enumerate() {
                        println!("      UVT{}: ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                            ti,
                            tri[0].u, tri[0].v,
                            tri[1].u, tri[1].v,
                            tri[2].u, tri[2].v,
                        );
                    }
                }

                // Print face normals for this face's triangles
                if let Some(ref face_normals) = instance.mesh.face_normals {
                    let fn_count = face_normals.len();
                    if tri_start < fn_count {
                        let fn_end = (tri_end).min(fn_count);
                        println!("    Face normals for this face (count={}):", fn_end - tri_start);
                        for fi in tri_start..fn_end.min(tri_start + 5) {
                            let n = face_normals[fi];
                            println!("      FN[{}]: ({:.4}, {:.4}, {:.4})", fi, n[0], n[1], n[2]);
                        }
                    }
                } else {
                    println!("    Face normals: NONE");
                }

                // Print vertex normals count vs vertex count
                if let Some(ref vnormals) = instance.mesh.normals {
                    println!("    Vertex normals: {} (vertices: {})", vnormals.len(), instance.mesh.vertices.len());
                } else {
                    println!("    Vertex normals: NONE (vertices: {})", instance.mesh.vertices.len());
                }

                // Print a few 3D triangles
                if tri_count > 0 {
                    let triangles = &instance.mesh.triangles;
                    let vertices = &instance.mesh.vertices;
                    let normals = instance.mesh.normals.as_ref();
                    let print_count = tri_count.min(5);
                    println!("    First {} 3D triangles:", print_count);
                    for ti in tri_start..(tri_start + print_count) {
                        if ti >= triangles.len() { break; }
                        let [i0, i1, i2] = triangles[ti];
                        let v0 = vertices[i0 as usize];
                        let v1 = vertices[i1 as usize];
                        let v2 = vertices[i2 as usize];

                        // Compute face normal from cross product
                        let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                        let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                        let nx = e1.1 * e2.2 - e1.2 * e2.1;
                        let ny = e1.2 * e2.0 - e1.0 * e2.2;
                        let nz = e1.0 * e2.1 - e1.1 * e2.0;
                        let len = (nx*nx + ny*ny + nz*nz).sqrt().max(1e-10);

                        let n0 = if let Some(norms) = normals {
                            if (i0 as usize) < norms.len() {
                                let n = norms[i0 as usize];
                                format!("n=({:.3},{:.3},{:.3})", n[0], n[1], n[2])
                            } else {
                                "n=OOB".to_string()
                            }
                        } else {
                            "n=None".to_string()
                        };

                        println!("      T{}: ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2}) cross=({:.3},{:.3},{:.3}) {}",
                            ti,
                            v0.x, v0.y, v0.z,
                            v1.x, v1.y, v1.z,
                            v2.x, v2.y, v2.z,
                            nx/len, ny/len, nz/len,
                            n0,
                        );
                    }
                }
            }
        }
    }

    println!("\n=== Diagnostic Complete ===");
}
