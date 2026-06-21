//! Verify NURBS surface evaluation against known-good reference values.
//!
//! These reference values are computed by hand using the analytic form
//! of each surface (saddle, bump, etc.) and should be reproduced EXACTLY
//! by the rational B-spline evaluation.

use draper_geometry::{NurbsSurface, Point3d};
#[allow(unused_imports)]
use draper_geometry::Surface;

fn dist(a: Point3d, b: Point3d) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

fn main() {
    println!("=== TEST 1: Bilinear Patch (warped quad) ===");
    {
        let control_points_v_rows = vec![
            vec![Point3d::new(-50.0, -50.0, -20.0), Point3d::new( 50.0, -50.0,  20.0)],
            vec![Point3d::new(-50.0,  50.0,  20.0), Point3d::new( 50.0,  50.0, -20.0)],
        ];
        let weights = vec![vec![1.0; 2]; 2];
        let u_knots = vec![0.0, 0.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 1.0, 1.0];
        let nurbs = NurbsSurface::from_v_rows(
            1, 1, control_points_v_rows, weights, u_knots, v_knots, false, false,
        );

        let p_center = nurbs.point_at(0.5, 0.5);
        println!("  Center (u=0.5, v=0.5): expected (0, 0, 0), got ({:.4}, {:.4}, {:.4})",
            p_center.x, p_center.y, p_center.z);
        println!("  Error: {:.6}", dist(p_center, Point3d::new(0.0, 0.0, 0.0)));
    }

    println!("\n=== TEST 2: Saddle (corners only — clamped bicubic) ===");
    {
        let control_points = vec![
            vec![Point3d::new(-50.0, -50.0,   0.0), Point3d::new(-17.0, -50.0, -28.0), Point3d::new( 17.0, -50.0, -28.0), Point3d::new( 50.0, -50.0,   0.0)],
            vec![Point3d::new(-50.0, -17.0,  28.0), Point3d::new(-17.0, -17.0,  -6.0), Point3d::new( 17.0, -17.0,  -6.0), Point3d::new( 50.0, -17.0,  28.0)],
            vec![Point3d::new(-50.0,  17.0,  28.0), Point3d::new(-17.0,  17.0,  -6.0), Point3d::new( 17.0,  17.0,  -6.0), Point3d::new( 50.0,  17.0,  28.0)],
            vec![Point3d::new(-50.0,  50.0,   0.0), Point3d::new(-17.0,  50.0, -28.0), Point3d::new( 17.0,  50.0, -28.0), Point3d::new( 50.0,  50.0,   0.0)],
        ];
        let weights = vec![vec![1.0; 4]; 4];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let nurbs = NurbsSurface::from_v_rows(
            3, 3, control_points, weights, u_knots, v_knots, false, false,
        );

        // For clamped bicubic, corners are exact
        let p_00 = nurbs.point_at(0.0, 0.0);
        let p_10 = nurbs.point_at(1.0, 0.0);
        let p_01 = nurbs.point_at(0.0, 1.0);
        let p_11 = nurbs.point_at(1.0, 1.0);
        println!("  (0,0): expected (-50,-50, 0), got ({:.4}, {:.4}, {:.4})", p_00.x, p_00.y, p_00.z);
        println!("  (1,0): expected ( 50,-50, 0), got ({:.4}, {:.4}, {:.4})", p_10.x, p_10.y, p_10.z);
        println!("  (0,1): expected (-50, 50, 0), got ({:.4}, {:.4}, {:.4})", p_01.x, p_01.y, p_01.z);
        println!("  (1,1): expected ( 50, 50, 0), got ({:.4}, {:.4}, {:.4})", p_11.x, p_11.y, p_11.z);
    }

    println!("\n=== TEST 3: Half-Cylinder (rational quadratic 180° arc) ===");
    {
        let r = 40.0;
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        let control_points = vec![
            vec![
                Point3d::new( r, 0.0, 0.0),
                Point3d::new( r, 0.0,   r),
                Point3d::new( 0.0, 0.0,   r),
                Point3d::new(-r, 0.0,   r),
                Point3d::new(-r, 0.0, 0.0),
            ],
            vec![
                Point3d::new( r, 100.0, 0.0),
                Point3d::new( r, 100.0,   r),
                Point3d::new( 0.0, 100.0,   r),
                Point3d::new(-r, 100.0,   r),
                Point3d::new(-r, 100.0, 0.0),
            ],
        ];
        let weights = vec![
            vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
            vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
        ];
        let u_knots = vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 1.0, 1.0];
        let nurbs = NurbsSurface::from_v_rows(
            2, 1, control_points, weights, u_knots, v_knots, false, false,
        );

        // Check that every point on the arc is exactly at radius R
        let n_samples = 50;
        let mut max_err = 0.0_f64;
        for i in 0..=n_samples {
            let u = i as f64 / n_samples as f64;
            let p = nurbs.point_at(u, 0.0);
            let dist_xy = (p.x.powi(2) + p.z.powi(2)).sqrt();
            let err = (dist_xy - r).abs();
            if err > max_err {
                max_err = err;
            }
        }
        println!("  Max radius error across {} samples: {:.10} mm (should be 0 for exact circle)", n_samples, max_err);
    }

    println!("\n=== TEST 4: Quarter-Sphere (rational quad octant) ===");
    {
        let r = 50.0;
        let inv_s = 1.0 / 2.0_f64.sqrt();
        let control_points = vec![
            vec![Point3d::new( r, 0.0, 0.0), Point3d::new( r,  r, 0.0), Point3d::new(0.0,  r, 0.0)],
            vec![Point3d::new( r, 0.0,   r), Point3d::new( r,  r,   r), Point3d::new(0.0,  r,   r)],
            vec![Point3d::new(0.0, 0.0,   r), Point3d::new(0.0, 0.0,   r), Point3d::new(0.0, 0.0,   r)],
        ];
        let weights = vec![
            vec![1.0,   inv_s, 1.0],
            vec![inv_s, 0.5,   inv_s],
            vec![1.0,   inv_s, 1.0],
        ];
        let u_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let nurbs = NurbsSurface::from_v_rows(
            2, 2, control_points, weights, u_knots, v_knots, false, false,
        );

        let n_samples = 10;
        let mut max_err = 0.0_f64;
        for i in 0..=n_samples {
            for j in 0..=n_samples {
                let u = i as f64 / n_samples as f64;
                let v = j as f64 / n_samples as f64;
                let p = nurbs.point_at(u, v);
                let dist_from_origin = (p.x.powi(2) + p.y.powi(2) + p.z.powi(2)).sqrt();
                let err = (dist_from_origin - r).abs();
                if err > max_err {
                    max_err = err;
                }
            }
        }
        println!("  Max radius error across {}x{} samples: {:.10} mm (should be 0 for exact sphere)",
            n_samples, n_samples, max_err);
    }

    println!("\n=== TEST 5: Closed Cylinder (NEW: rational quadratic full circle) ===");
    {
        let r = 40.0;
        let h = 100.0;
        let (circle_pts, circle_weights, circle_knots) = NurbsSurface::full_circle_xy(r);

        let mut control_points = Vec::with_capacity(2);
        for &z in &[0.0, h] {
            let row: Vec<Point3d> = circle_pts.iter().map(|p| Point3d::new(p.x, p.y, z)).collect();
            control_points.push(row);
        }
        let weights = vec![circle_weights.clone(); 2];
        let u_knots: Vec<f64> = circle_knots.iter().map(|&k| k * 2.0 * std::f64::consts::PI).collect();
        let v_knots = vec![0.0, 0.0, 1.0, 1.0];

        let nurbs = NurbsSurface::from_v_rows(
            2, 1, control_points, weights, u_knots, v_knots, true, false,
        );

        let (u_min, u_max) = nurbs.u_range();
        let (v_min, v_max) = nurbs.v_range();
        println!("  u_range=({:.4}, {:.4}), expected (0, 2π={:.4})", u_min, u_max, 2.0*std::f64::consts::PI);
        println!("  v_range=({:.4}, {:.4}), expected (0, 1)", v_min, v_max);

        // Sample at 36 angles, at v=0 (bottom). Every point should be at radius R exactly.
        let n_samples = 36;
        let mut max_err = 0.0_f64;
        let mut max_z_err = 0.0_f64;
        for i in 0..=n_samples {
            let u = u_min + (u_max - u_min) * (i as f64) / (n_samples as f64);
            let p = nurbs.point_at(u, 0.0);
            let dist_xy = (p.x.powi(2) + p.y.powi(2)).sqrt();
            let err = (dist_xy - r).abs();
            let z_err = p.z.abs();
            if err > max_err { max_err = err; }
            if z_err > max_z_err { max_z_err = z_err; }
        }
        println!("  Max radius error: {:.10} mm (should be 0 for exact circle)", max_err);
        println!("  Max z error (should be 0 at v=0): {:.10} mm", max_z_err);

        // Check at v=1 (top), z should be h
        let p_top = nurbs.point_at(0.0, 1.0);
        println!("  Top point at (u=0, v=1): z={:.4} (expected {})", p_top.z, h);
    }

    println!("\n=== TEST 6: Surface of Revolution (NEW: exact rational quad circle in V) ===");
    {
        let profile_pts = vec![
            Point3d::new(40.0, 0.0,   0.0),
            Point3d::new(30.0, 0.0,  33.0),
            Point3d::new(50.0, 0.0,  66.0),
            Point3d::new(35.0, 0.0, 100.0),
        ];
        let profile_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let profile_weights = vec![1.0; 4];

        let nurbs = NurbsSurface::surface_of_revolution_z(
            &profile_pts,
            3,
            profile_knots,
            profile_weights,
            0.0,
            2.0 * std::f64::consts::PI,
            false,
        );

        let (u_min, u_max) = nurbs.u_range();
        let (v_min, v_max) = nurbs.v_range();
        println!("  u_range=({:.4}, {:.4}), expected (0, 1)", u_min, u_max);
        println!("  v_range=({:.4}, {:.4}), expected (0, 2π={:.4})  ← was 4.19 (240°) before fix",
            v_min, v_max, 2.0*std::f64::consts::PI);

        // Sample at 36 angles around the vase at u=0 (z=0, expected radius 40).
        // For a clamped cubic with these control points, at u=0 the curve passes
        // exactly through profile_pts[0] = (40, 0, 0), so radius should be 40.
        let n_samples = 36;
        let mut max_err = 0.0_f64;
        for i in 0..=n_samples {
            let v = v_min + (v_max - v_min) * (i as f64) / (n_samples as f64);
            let p = nurbs.point_at(0.0, v);
            let dist_xy = (p.x.powi(2) + p.y.powi(2)).sqrt();
            let err = (dist_xy - 40.0).abs();
            if err > max_err { max_err = err; }
            if i % 6 == 0 {
                println!("  v={:.4} ({:>3}°): point=({:>7.3}, {:>7.3}, {:>7.3}) |xy|={:.6}",
                    v, v * 180.0 / std::f64::consts::PI, p.x, p.y, p.z, dist_xy);
            }
        }
        println!("  Max radius error at u=0 (z=0): {:.10} mm (should be 0 for exact circle)", max_err);

        // At u=1 (z=100), expected radius 35.
        let p_top = nurbs.point_at(1.0, 0.0);
        let dist_xy = (p_top.x.powi(2) + p_top.y.powi(2)).sqrt();
        println!("  Top at u=1, v=0: |xy|={:.6} (expected 35)", dist_xy);
    }

    println!("\n=== TEST 7: full_circle_xy helper ===");
    {
        let r = 25.0;
        let (pts, weights, knots) = NurbsSurface::full_circle_xy(r);
        println!("  9 control points, {} weights, {} knots", weights.len(), knots.len());
        println!("  Weights: {:?}", weights);
        println!("  Knots:   {:?}", knots);

        // Build a NURBS curve from these and verify it's an exact circle.
        use draper_geometry::{NurbsCurve, Curve3d};
        let curve = Curve3d::Nurbs(NurbsCurve {
            degree: 2,
            control_points: pts,
            weights,
            knots,
        });

        let n_samples = 100;
        let mut max_err = 0.0_f64;
        for i in 0..=n_samples {
            let t = i as f64 / n_samples as f64;
            let p = curve.point_at(t);
            let dist = (p.x.powi(2) + p.y.powi(2)).sqrt();
            let err = (dist - r).abs();
            if err > max_err { max_err = err; }
        }
        println!("  Max radius error across {} samples: {:.12} mm (should be ~0)", n_samples, max_err);

        // Verify the curve passes through the 4 cardinal points
        let p_0 = curve.point_at(0.0);
        let p_q1 = curve.point_at(0.25);
        let p_q2 = curve.point_at(0.5);
        let p_q3 = curve.point_at(0.75);
        let p_1 = curve.point_at(1.0);
        println!("  At u=0.00: ({:>7.3}, {:>7.3}, {:>7.3})  expected ({}, 0, 0)", p_0.x, p_0.y, p_0.z, r);
        println!("  At u=0.25: ({:>7.3}, {:>7.3}, {:>7.3})  expected (0, {}, 0)", p_q1.x, p_q1.y, p_q1.z, r);
        println!("  At u=0.50: ({:>7.3}, {:>7.3}, {:>7.3})  expected ({}, 0, 0)", p_q2.x, p_q2.y, p_q2.z, -r);
        println!("  At u=0.75: ({:>7.3}, {:>7.3}, {:>7.3})  expected (0, {}, 0)", p_q3.x, p_q3.y, p_q3.z, -r);
        println!("  At u=1.00: ({:>7.3}, {:>7.3}, {:>7.3})  expected ({}, 0, 0)", p_1.x, p_1.y, p_1.z, r);
    }

    println!("\n=== SUMMARY ===");
    println!("All tests should show max errors < 1e-10 for exact rational constructions.");
}
