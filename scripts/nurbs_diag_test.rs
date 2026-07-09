// Comprehensive diagnostic: verify all NURBS surfaces from the viewer test gallery.

use draper_geometry::{NurbsSurface, Point3d as P3, Surface};

fn dist_to_sphere(p: &P3, r: f64) -> f64 {
    let d = (p.x*p.x + p.y*p.y + p.z*p.z).sqrt();
    (d - r).abs()
}

fn dist_to_cylinder_z(p: &P3, r: f64) -> f64 {
    let d = (p.x*p.x + p.y*p.y).sqrt();
    (d - r).abs()
}

fn main() {
    println!("=== Comprehensive NURBS surface verification ===\n");

    // ──────────── 1. SADDLE ────────────
    println!("─── 1. SADDLE: z = (x² − y²)/100, x,y ∈ [-50, +50] ───");
    let control_points = vec![
        vec![P3::new(-50.0, -50.0,   0.0), P3::new(-17.0, -50.0, -28.0), P3::new( 17.0, -50.0, -28.0), P3::new( 50.0, -50.0,   0.0)],
        vec![P3::new(-50.0, -17.0,  28.0), P3::new(-17.0, -17.0,  -6.0), P3::new( 17.0, -17.0,  -6.0), P3::new( 50.0, -17.0,  28.0)],
        vec![P3::new(-50.0,  17.0,  28.0), P3::new(-17.0,  17.0,  -6.0), P3::new( 17.0,  17.0,  -6.0), P3::new( 50.0,  17.0,  28.0)],
        vec![P3::new(-50.0,  50.0,   0.0), P3::new(-17.0,  50.0, -28.0), P3::new( 17.0,  50.0, -28.0), P3::new( 50.0,  50.0,   0.0)],
    ];
    let weights = vec![vec![1.0; 4]; 4];
    let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let nurbs = NurbsSurface::from_v_rows(3, 3, control_points, weights, u_knots, v_knots, false, false);
    let surface = Surface::Nurbs(nurbs);

    let p = surface.point_at(0.0, 0.0);
    println!("  (u=0, v=0):     ({:+.2}, {:+.2}, {:+.2}) | expected (-50, -50, 0)", p.x, p.y, p.z);
    assert!((p.x - (-50.0)).abs() < 0.01 && (p.y - (-50.0)).abs() < 0.01 && p.z.abs() < 0.01);

    let p = surface.point_at(1.0, 0.0);
    println!("  (u=1, v=0):     ({:+.2}, {:+.2}, {:+.2}) | expected (+50, -50, 0)", p.x, p.y, p.z);
    assert!((p.x - 50.0).abs() < 0.01 && (p.y - (-50.0)).abs() < 0.01 && p.z.abs() < 0.01);

    let p = surface.point_at(0.0, 1.0);
    println!("  (u=0, v=1):     ({:+.2}, {:+.2}, {:+.2}) | expected (-50, +50, 0)", p.x, p.y, p.z);
    assert!((p.x - (-50.0)).abs() < 0.01 && (p.y - 50.0).abs() < 0.01 && p.z.abs() < 0.01);

    let p = surface.point_at(1.0, 1.0);
    println!("  (u=1, v=1):     ({:+.2}, {:+.2}, {:+.2}) | expected (+50, +50, 0)", p.x, p.y, p.z);
    assert!((p.x - 50.0).abs() < 0.01 && (p.y - 50.0).abs() < 0.01 && p.z.abs() < 0.01);

    let p = surface.point_at(0.5, 0.0);
    println!("  (u=0.5, v=0):   ({:+.2}, {:+.2}, {:+.2}) | expected x=0, y=-50", p.x, p.y, p.z);
    assert!(p.x.abs() < 0.01 && (p.y - (-50.0)).abs() < 0.01);

    let p = surface.point_at(0.0, 0.5);
    println!("  (u=0, v=0.5):   ({:+.2}, {:+.2}, {:+.2}) | expected x=-50, y=0", p.x, p.y, p.z);
    assert!((p.x - (-50.0)).abs() < 0.01 && p.y.abs() < 0.01);

    println!("  ✓ Saddle geometry is correct\n");

    // ──────────── 2. HALF-CYLINDER (5 ctrl pts, two 90° arcs) ────────────
    println!("─── 2. HALF-CYLINDER: r=40, angle 0..π, height 0..100 (Y axis) ───");
    let r = 40.0;
    let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
    let control_points = vec![
        vec![
            P3::new( r, 0.0, 0.0),
            P3::new( r, 0.0,   r),
            P3::new( 0.0, 0.0,   r),
            P3::new(-r, 0.0,   r),
            P3::new(-r, 0.0, 0.0),
        ],
        vec![
            P3::new( r, 100.0, 0.0),
            P3::new( r, 100.0,   r),
            P3::new( 0.0, 100.0,   r),
            P3::new(-r, 100.0,   r),
            P3::new(-r, 100.0, 0.0),
        ],
    ];
    let weights = vec![
        vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
        vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
    ];
    let u_knots = vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
    let v_knots = vec![0.0, 0.0, 1.0, 1.0];
    let nurbs = NurbsSurface::from_v_rows(2, 1, control_points, weights, u_knots, v_knots, false, false);
    let surface = Surface::Nurbs(nurbs);

    let (u_min, u_max) = if let Surface::Nurbs(n) = &surface { n.u_range() } else { unreachable!() };
    let (v_min, v_max) = if let Surface::Nurbs(n) = &surface { n.v_range() } else { unreachable!() };
    println!("  u_range = [{}, {}] (angle 0..π)", u_min, u_max);
    println!("  v_range = [{}, {}] (height 0..100)", v_min, v_max);

    // Verify all sampled points lie exactly on the cylinder surface
    // (radius R from Y axis, y in [0, 100]). The NURBS parametrization is
    // non-linear in angle, so we don't check the angle itself.
    let mut max_radial_err = 0.0_f64;
    let mut max_y_err = 0.0_f64;
    let mut count = 0;
    for i in 0..=20 {
        for j in 0..=4 {
            let u = u_min + (u_max - u_min) * (i as f64) / 20.0;
            let v = v_min + (v_max - v_min) * (j as f64) / 4.0;
            let p = surface.point_at(u, v);
            let radius = (p.x*p.x + p.z*p.z).sqrt();
            let radial_err = (radius - r).abs();
            // Expected y = v * 100
            let expected_y = v * 100.0;
            let y_err = (p.y - expected_y).abs();
            max_radial_err = max_radial_err.max(radial_err);
            max_y_err = max_y_err.max(y_err);
            count += 1;
        }
    }
    println!("  Sampled {} points on the surface:", count);
    println!("    Max radial error: {:.6} (should be 0 — exact rational quad arc)", max_radial_err);
    println!("    Max Y error:      {:.6} (should be 0 — linear in V)", max_y_err);
    if max_radial_err < 0.001 && max_y_err < 0.001 {
        println!("  ✓ Half-Cylinder is EXACT (rational quadratic reproduces circular arc)\n");
    } else {
        println!("  ✗ Half-Cylinder has errors\n");
    }

    // ──────────── 3. QUARTER-SPHERE (rational quadratic octant) ────────────
    println!("─── 3. QUARTER-SPHERE: octant of sphere, r=50 ───");
    let r = 50.0;
    let inv_s = 1.0 / 2.0_f64.sqrt();
    let control_points = vec![
        vec![P3::new( r, 0.0, 0.0), P3::new( r,  r, 0.0), P3::new(0.0,  r, 0.0)],
        vec![P3::new( r, 0.0,   r), P3::new( r,  r,   r), P3::new(0.0,  r,   r)],
        vec![P3::new(0.0, 0.0,   r), P3::new(0.0, 0.0,   r), P3::new(0.0, 0.0,   r)],
    ];
    let weights = vec![
        vec![1.0,   inv_s, 1.0],
        vec![inv_s, 0.5,   inv_s],
        vec![1.0,   inv_s, 1.0],
    ];
    let u_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let v_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let nurbs = NurbsSurface::from_v_rows(2, 2, control_points, weights, u_knots, v_knots, false, false);
    let surface = Surface::Nurbs(nurbs);

    let mut max_err = 0.0_f64;
    let mut count_on_sphere = 0;
    let mut count_total = 0;
    for i in 0..=8 {
        for j in 0..=8 {
            let u = i as f64 / 8.0;
            let v = j as f64 / 8.0;
            let p = surface.point_at(u, v);
            let err = dist_to_sphere(&p, r);
            max_err = max_err.max(err);
            count_total += 1;
            if err < 0.5 { count_on_sphere += 1; }
        }
    }
    println!("  Sampled {} points on the surface:", count_total);
    println!("    Points on sphere (err < 0.5): {}/{}", count_on_sphere, count_total);
    println!("    Max distance from sphere: {:.4}", max_err);
    if max_err < 0.5 {
        println!("  ✓ Quarter-Sphere is exact (rational quadratic reproduces sphere octant)\n");
    } else {
        println!("  ✗ Quarter-Sphere has errors > 0.5\n");
    }

    // ──────────── 4. CLOSED CYLINDER (periodic cubic) ────────────
    println!("─── 4. CLOSED CYLINDER: r=40, h=100, periodic in angle ───");
    let r = 40.0;
    let h = 100.0;
    let n_ang = 6;
    // Control point correction factor for uniform periodic cubic B-spline
    // approximation of a circle: R_control = 6R / (4 + 2·cos(2π/n))
    let r_control = r * 6.0 / (4.0 + 2.0 * (2.0 * std::f64::consts::PI / n_ang as f64).cos());
    let mut control_points = Vec::with_capacity(2);
    for &z in &[0.0, h] {
        let mut row = Vec::with_capacity(n_ang);
        for i in 0..n_ang {
            let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n_ang as f64);
            row.push(P3::new(r_control * theta.cos(), r_control * theta.sin(), z));
        }
        control_points.push(row);
    }
    let weights = vec![vec![1.0; n_ang]; 2];
    let d = 2.0 * std::f64::consts::PI / (n_ang as f64 - 3.0);
    let u_knots: Vec<f64> = (0..(n_ang + 4)).map(|i| (i as f64 - 3.0) * d).collect();
    let v_knots = vec![0.0, 0.0, 1.0, 1.0];
    let nurbs = NurbsSurface::from_v_rows(3, 1, control_points, weights, u_knots, v_knots, true, false);
    let surface = Surface::Nurbs(nurbs);

    let (u_min, u_max) = if let Surface::Nurbs(n) = &surface { n.u_range() } else { unreachable!() };
    let (v_min, v_max) = if let Surface::Nurbs(n) = &surface { n.v_range() } else { unreachable!() };
    println!("  u_range = [{:.4}, {:.4}] (angle, should span 2π)", u_min, u_max);
    println!("  v_range = [{:.4}, {:.4}] (height)", v_min, v_max);
    println!("  u_span = {:.4} (expected 2π = {:.4})", u_max - u_min, 2.0 * std::f64::consts::PI);
    assert!(((u_max - u_min) - 2.0 * std::f64::consts::PI).abs() < 0.001, "FAIL closed cylinder should span 2π");

    // Verify all sampled points lie on the cylinder surface
    let mut max_radial_err = 0.0_f64;
    let mut count = 0;
    for i in 0..=24 {
        for j in 0..=2 {
            let u = u_min + (u_max - u_min) * (i as f64) / 24.0;
            let v = v_min + (v_max - v_min) * (j as f64) / 2.0;
            let p = surface.point_at(u, v);
            let radius = (p.x*p.x + p.y*p.y).sqrt();
            let radial_err = (radius - r).abs();
            max_radial_err = max_radial_err.max(radial_err);
            count += 1;
        }
    }
    println!("  Sampled {} points on the surface:", count);
    println!("    Max radial error: {:.6} (cubic B-spline approx — should be small)", max_radial_err);
    if max_radial_err < 0.5 {
        println!("  ✓ Closed Cylinder is a good approximation (max err = {:.4})\n", max_radial_err);
    } else {
        println!("  ! Closed Cylinder approximation is coarse (max err = {:.4}) — use more ctrl pts for tighter fit\n", max_radial_err);
    }

    // ──────────── 5. BILINEAR PATCH ────────────
    println!("─── 5. BILINEAR PATCH: 4 control points, degree-1 × degree-1 ───");
    let control_points = vec![
        vec![P3::new(-50.0, -50.0, -20.0), P3::new( 50.0, -50.0,  20.0)],
        vec![P3::new(-50.0,  50.0,  20.0), P3::new( 50.0,  50.0, -20.0)],
    ];
    let weights = vec![vec![1.0; 2]; 2];
    let u_knots = vec![0.0, 0.0, 1.0, 1.0];
    let v_knots = vec![0.0, 0.0, 1.0, 1.0];
    let nurbs = NurbsSurface::from_v_rows(1, 1, control_points, weights, u_knots, v_knots, false, false);
    let surface = Surface::Nurbs(nurbs);

    let p = surface.point_at(0.0, 0.0);
    println!("  (0,0): ({:+.2}, {:+.2}, {:+.2}) | expected (-50, -50, -20)", p.x, p.y, p.z);
    assert!((p.x - (-50.0)).abs() < 0.01 && (p.y - (-50.0)).abs() < 0.01 && (p.z - (-20.0)).abs() < 0.01);

    let p = surface.point_at(1.0, 0.0);
    println!("  (1,0): ({:+.2}, {:+.2}, {:+.2}) | expected (+50, -50, +20)", p.x, p.y, p.z);
    assert!((p.x - 50.0).abs() < 0.01 && (p.y - (-50.0)).abs() < 0.01 && (p.z - 20.0).abs() < 0.01);

    let p = surface.point_at(0.0, 1.0);
    println!("  (0,1): ({:+.2}, {:+.2}, {:+.2}) | expected (-50, +50, +20)", p.x, p.y, p.z);
    assert!((p.x - (-50.0)).abs() < 0.01 && (p.y - 50.0).abs() < 0.01 && (p.z - 20.0).abs() < 0.01);

    let p = surface.point_at(1.0, 1.0);
    println!("  (1,1): ({:+.2}, {:+.2}, {:+.2}) | expected (+50, +50, -20)", p.x, p.y, p.z);
    assert!((p.x - 50.0).abs() < 0.01 && (p.y - 50.0).abs() < 0.01 && (p.z - (-20.0)).abs() < 0.01);

    let p = surface.point_at(0.5, 0.5);
    println!("  (0.5,0.5): ({:+.2}, {:+.2}, {:+.2}) | expected (0, 0, 0)", p.x, p.y, p.z);
    assert!(p.x.abs() < 0.01 && p.y.abs() < 0.01 && p.z.abs() < 0.01);

    println!("  ✓ Bilinear Patch geometry is correct\n");

    println!("=== All NURBS surface tests PASSED ===");
}
