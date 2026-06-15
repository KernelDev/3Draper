// SPDX-License-Identifier: GPL-3.0-or-later
// Diagnostic test for NURBS triangulation issues.

use draper_step::{parse_step, step_to_mesh};
use draper_geometry::{Point3d, Surface};

fn read_nist_step(filename: &str) -> String {
    let path = format!("../../test/{}", filename);
    std::fs::read_to_string(&path).unwrap()
}

#[test]
fn test_nurbs_surface_evaluation() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .is_test(true)
        .try_init();

    let content = read_nist_step("nist_complex_surface.stp");
    let step = parse_step(&content).expect("parse failed");
    
    println!("\n========================================");
    println!("STEP PARSED: {} entities", step.entities.len());
    println!("========================================");

    let mesh = step_to_mesh(&step).expect("mesh failed");
    println!("\nCombined mesh: v={} t={}", mesh.vertex_count(), mesh.triangle_count());
    
    for (vi, v) in mesh.vertices.iter().enumerate() {
        println!("  v[{}] = ({:.6}, {:.6}, {:.6})", vi, v.x, v.y, v.z);
    }
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        // Compute triangle area
        let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let cx = e1.1 * e2.2 - e1.2 * e2.1;
        let cy = e1.2 * e2.0 - e1.0 * e2.2;
        let cz = e1.0 * e2.1 - e1.1 * e2.0;
        let area = (cx*cx + cy*cy + cz*cz).sqrt() * 0.5;
        println!("  t[{}] = [{}, {}, {}] area={:.6}", ti, tri[0], tri[1], tri[2], area);
    }
    
    // Check if any vertex has z > 1.0 (the NURBS surface should bulge upward)
    let max_z = mesh.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
    let min_z = mesh.vertices.iter().map(|v| v.z).fold(f64::MAX, f64::min);
    println!("\nZ range: [{:.4}, {:.4}]", min_z, max_z);
    println!("Expected: z should be >= 0.0 and the top face should have z >= 1.0");
    
    // The NURBS surface should have z values between 1.0 and ~1.5 at center
    // If all top-face vertices are at z=1.0, the surface isn't curved
    let top_face_verts: Vec<_> = mesh.vertices.iter().filter(|v| v.z > 0.9).collect();
    let top_z_values: Vec<f64> = top_face_verts.iter().map(|v| v.z).collect();
    println!("Top face z values: {:?}", top_z_values);
}

#[test]
fn test_nurbs_projection_accuracy() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .is_test(true)
        .try_init();

    // Create the same NURBS surface as in nist_complex_surface.stp
    // Degree 2×2, 3×3 control points, knots [0,0,0,1,1,1] × [0,0,0,1,1,1]
    let control_points = vec![
        vec![Point3d::new(0.0, 0.0, 1.0), Point3d::new(5.0, 0.0, 1.0), Point3d::new(10.0, 0.0, 1.0)],
        vec![Point3d::new(0.0, 5.0, 1.0), Point3d::new(5.0, 5.0, 3.0), Point3d::new(10.0, 5.0, 1.0)],
        vec![Point3d::new(0.0, 10.0, 1.0), Point3d::new(5.0, 10.0, 1.0), Point3d::new(10.0, 10.0, 1.0)],
    ];
    
    let n_u = 3; let n_v = 3;
    let nurbs = draper_geometry::NurbsSurface {
        u_degree: 2,
        v_degree: 2,
        control_points: control_points,
        u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        weights: vec![vec![1.0; n_v]; n_u],
        u_closed: false,
        v_closed: false,
    };
    
    let surface = Surface::Nurbs(nurbs.clone());
    
    println!("\n========================================");
    println!("NURBS SURFACE EVALUATION TEST");
    println!("========================================");
    println!("u_range: {:?}", nurbs.u_range());
    println!("v_range: {:?}", nurbs.v_range());
    
    // Test surface evaluation at key points
    println!("\n--- Surface evaluation ---");
    let test_uvs = vec![
        (0.0, 0.0), (0.5, 0.0), (1.0, 0.0),
        (0.0, 0.5), (0.5, 0.5), (1.0, 0.5),
        (0.0, 1.0), (0.5, 1.0), (1.0, 1.0),
    ];
    for (u, v) in &test_uvs {
        let p = surface.point_at(*u, *v);
        println!("  S({:.1},{:.1}) = ({:.6}, {:.6}, {:.6})", u, v, p.x, p.y, p.z);
    }
    
    // Test project_point for boundary points
    println!("\n--- Projection test (boundary points) ---");
    let test_points = vec![
        ("corner (0,0,1)", Point3d::new(0.0, 0.0, 1.0), 0.0, 0.0),
        ("corner (10,0,1)", Point3d::new(10.0, 0.0, 1.0), 1.0, 0.0),
        ("corner (10,10,1)", Point3d::new(10.0, 10.0, 1.0), 1.0, 1.0),
        ("corner (0,10,1)", Point3d::new(0.0, 10.0, 1.0), 0.0, 1.0),
        ("mid-x (5,0,1)", Point3d::new(5.0, 0.0, 1.0), 0.5, 0.0),
        ("mid-y (10,5,1)", Point3d::new(10.0, 5.0, 1.0), 1.0, 0.5),
        ("mid-x (5,10,1)", Point3d::new(5.0, 10.0, 1.0), 0.5, 1.0),
        ("mid-y (0,5,1)", Point3d::new(0.0, 5.0, 1.0), 0.0, 0.5),
    ];
    for (name, p, exp_u, exp_v) in &test_points {
        let (u, v) = surface.project_point(p);
        let reconstructed = surface.point_at(u, v);
        let error = p.distance_to(&reconstructed);
        let u_err = (u - exp_u).abs();
        let v_err = (v - exp_v).abs();
        println!("  {} -> UV=({:.6}, {:.6}) expected=({:.1}, {:.1}) err_uv=({:.2e},{:.2e}) recon_err={:.2e}",
            name, u, v, exp_u, exp_v, u_err, v_err, error);
    }
    
    // Test brute_force_project_point
    println!("\n--- Brute-force projection test ---");
    for (name, p, exp_u, exp_v) in &test_points {
        let (u, v) = draper_mesh::edge_cache::brute_force_project_point(&nurbs, p, 20);
        let reconstructed = surface.point_at(u, v);
        let error = p.distance_to(&reconstructed);
        let u_err = (u - exp_u).abs();
        let v_err = (v - exp_v).abs();
        println!("  {} -> UV=({:.6}, {:.6}) expected=({:.1}, {:.1}) err_uv=({:.2e},{:.2e}) recon_err={:.2e}",
            name, u, v, exp_u, exp_v, u_err, v_err, error);
    }
}
