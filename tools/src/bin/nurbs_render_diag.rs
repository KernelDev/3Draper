// Diagnostic: evaluate the NURBS Saddle surface at sample points and
// compare to the analytic saddle z = (x^2 - y^2) / 100 over [-50,+50]^2.
//
// Build with:
//   cargo run --release --bin nurbs_diag
//
// (assuming a [[bin]] entry is added to Cargo.toml; or just run as a test)

use draper_geometry::{NurbsSurface, Surface, Point3d as P3};

fn main() {
    // ---- Replicate load_nurbs_saddle exactly ----
    let control_points = vec![
        // row v=0 (y=-50)
        vec![P3::new(-50.0, -50.0,   0.0), P3::new(-17.0, -50.0, -28.0), P3::new( 17.0, -50.0, -28.0), P3::new( 50.0, -50.0,   0.0)],
        // row v=1 (y≈-17)
        vec![P3::new(-50.0, -17.0,  28.0), P3::new(-17.0, -17.0,  -6.0), P3::new( 17.0, -17.0,  -6.0), P3::new( 50.0, -17.0,  28.0)],
        // row v=2 (y≈+17)
        vec![P3::new(-50.0,  17.0,  28.0), P3::new(-17.0,  17.0,  -6.0), P3::new( 17.0,  17.0,  -6.0), P3::new( 50.0,  17.0,  28.0)],
        // row v=3 (y=+50)
        vec![P3::new(-50.0,  50.0,   0.0), P3::new(-17.0,  50.0, -28.0), P3::new( 17.0,  50.0, -28.0), P3::new( 50.0,  50.0,   0.0)],
    ];
    let weights = vec![vec![1.0; 4]; 4];
    let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let nurbs_surface = NurbsSurface::from_v_rows(
        3, 3, control_points, weights, u_knots, v_knots, false, false,
    );
    let surface = Surface::Nurbs(nurbs_surface.clone());

    let (u_min, u_max) = nurbs_surface.u_range();
    let (v_min, v_max) = nurbs_surface.v_range();
    println!("u_range = [{}, {}]", u_min, u_max);
    println!("v_range = [{}, {}]", v_min, v_max);
    println!();

    // Expected: x and y both map linearly: x = -50 + u*100, y = -50 + v*100.
    // (Because control points are at x ∈ {-50,-17,17,50} but the NURBS
    //  surface interpolates the corners exactly with clamped knots.)
    // So at u=v=0 → (x,y,z) = (-50,-50, 0).
    // At u=v=1 → (x,y,z) = (+50,+50, 0).
    // At u=v=0.5 → (x,y) = (0,0), z = (0-0)/100 = 0.
    // At u=1, v=0 → (x,y,z) = (+50,-50, (2500-2500)/100) = (50,-50,0).
    // At u=0.5, v=0 → (x,y) = (0,-50), z = (0-2500)/100 = -25.
    // At u=1, v=0.5 → (x,y) = (50,0), z = (2500-0)/100 = +25.

    println!("Surface evaluations (sample points):");
    println!("  (u, v)        → (x, y, z)        | expected (x, y, z)");
    let test_cases: &[(f64, f64, f64, f64, f64)] = &[
        // (u, v, exp_x, exp_y, exp_z)
        (0.0, 0.0, -50.0, -50.0, 0.0),
        (1.0, 0.0,  50.0, -50.0, 0.0),
        (1.0, 1.0,  50.0,  50.0, 0.0),
        (0.0, 1.0, -50.0,  50.0, 0.0),
        (0.5, 0.5,   0.0,   0.0, 0.0),
        (0.5, 0.0,   0.0, -50.0, -25.0),
        (1.0, 0.5,  50.0,   0.0,  25.0),
        (0.0, 0.5, -50.0,   0.0,  25.0),
        (0.5, 1.0,   0.0,  50.0, -25.0),
        (0.25, 0.25, -25.0, -25.0, 0.0),
        (0.75, 0.25,  25.0, -25.0, 0.0),
        (0.25, 0.75, -25.0,  25.0, 0.0),
        (0.75, 0.75,  25.0,  25.0, 0.0),
        (0.5, 0.25,   0.0, -25.0, -6.25),
        (0.5, 0.75,   0.0,  25.0, -6.25),
        (0.25, 0.5, -25.0,   0.0,  6.25),
        (0.75, 0.5,  25.0,   0.0,  6.25),
    ];
    let mut max_err_xyz = 0.0_f64;
    for &(u, v, ex, ey, ez) in test_cases {
        let p = surface.point_at(u, v);
        let err = ((p.x - ex).abs().max((p.y - ey).abs())).max((p.z - ez).abs());
        if err > max_err_xyz { max_err_xyz = err; }
        println!("  ({:.2}, {:.2}) → ({:8.3}, {:8.3}, {:8.3}) | ({}, {}, {})  err={:.3}",
                 u, v, p.x, p.y, p.z, ex, ey, ez, err);
    }
    println!();
    println!("Max X/Y/Z error across all sample points: {:.4}", max_err_xyz);
    println!();
    println!("Interpretation:");
    if max_err_xyz < 1.0 {
        println!("  ✓ NURBS surface evaluation is CORRECT (errors < 1.0).");
        println!("    → The 3D rendering bug is in TRIANGULATION, not surface eval.");
    } else {
        println!("  ✗ NURBS surface evaluation is WRONG (errors >= 1.0).");
        println!("    → The bug is in nurbs_surface_eval / from_v_rows / knot vector.");
    }

    // ---- Also test the boundary used by build_nurbs_surface_mesh ----
    println!();
    println!("Boundary sampling (steps=30, as in build_nurbs_surface_mesh):");
    let steps = 30;
    let (u_min, u_max) = nurbs_surface.u_range();
    let (v_min, v_max) = nurbs_surface.v_range();
    let mut boundary: Vec<P3> = Vec::new();
    // Bottom edge (v = v_min)
    for i in 0..=steps {
        let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u, v_min));
    }
    // Right edge (u = u_max)
    for i in 1..=steps {
        let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u_max, v));
    }
    // Top edge (v = v_max), reversed
    for i in (0..steps).rev() {
        let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u, v_max));
    }
    // Left edge (u = u_min), reversed
    for i in (1..steps).rev() {
        let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u_min, v));
    }
    println!("  Total boundary points: {}", boundary.len());
    println!("  First 3 (bottom edge, v=0):");
    for (i, p) in boundary.iter().take(3).enumerate() {
        println!("    [{}] = ({:.3}, {:.3}, {:.3})", i, p.x, p.y, p.z);
    }
    println!("  Expected first: (-50, -50, 0), then along bottom edge");
    println!("  Point at u=0.5, v=0 (idx 15): ({:.3}, {:.3}, {:.3}) — expected (0, -50, -25)",
             boundary[15].x, boundary[15].y, boundary[15].z);

    // ---- Check for NaN / Inf ----
    let mut nan_count = 0;
    let n_u_samples = 50;
    let n_v_samples = 50;
    for i in 0..=n_u_samples {
        for j in 0..=n_v_samples {
            let u = u_min + (u_max - u_min) * i as f64 / n_u_samples as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v_samples as f64;
            let p = surface.point_at(u, v);
            if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
                nan_count += 1;
                if nan_count <= 5 {
                    println!("  NaN/Inf at (u={:.3}, v={:.3}): ({}, {}, {})", u, v, p.x, p.y, p.z);
                }
            }
        }
    }
    println!();
    println!("NaN/Inf check ({}x{} = {} samples): {} NaN/Inf points",
             n_u_samples, n_v_samples, n_u_samples * n_v_samples, nan_count);

    // ---- Check the control_points layout after from_v_rows ----
    println!();
    println!("After from_v_rows, internal layout:");
    println!("  u_degree={}, v_degree={}", nurbs_surface.u_degree, nurbs_surface.v_degree);
    println!("  control_points.len() (n_u) = {}", nurbs_surface.control_points.len());
    println!("  control_points[0].len() (n_v) = {}", nurbs_surface.control_points[0].len());
    println!("  u_knots = {:?}", nurbs_surface.u_knots);
    println!("  v_knots = {:?}", nurbs_surface.v_knots);
    println!("  First few control_points[u][v] (expect corners):");
    for ui in 0..nurbs_surface.control_points.len() {
        for vi in 0..nurbs_surface.control_points[ui].len() {
            let p = &nurbs_surface.control_points[ui][vi];
            if (ui < 2 && vi < 2) || (ui >= nurbs_surface.control_points.len() - 2 && vi >= nurbs_surface.control_points[0].len() - 2) {
                println!("    cp[{}][{}] = ({:.1}, {:.1}, {:.1})", ui, vi, p.x, p.y, p.z);
            }
        }
    }
}
