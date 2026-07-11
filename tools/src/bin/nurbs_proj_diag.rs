// Diagnosis: test NURBS projection on known-difficult surfaces
// 1. Create NURBS surfaces with various configurations
// 2. Test round-trip projection (point → project → check)
// 3. Identify which configurations cause failures
use draper_geometry::{NurbsSurface, Surface, Point3d};
use draper_mesh::edge_cache::{brute_force_project_point, adaptive_grid_size};

fn make_flat_nurbs() -> NurbsSurface {
    // Simple flat NURBS (degree 1x1, 2x2 control points) — should be trivial
    NurbsSurface {
        u_degree: 1,
        v_degree: 1,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 10.0, 0.0)],
            vec![Point3d::new(10.0, 0.0, 0.0), Point3d::new(10.0, 10.0, 0.0)],
        ],
        weights: vec![
            vec![1.0, 1.0],
            vec![1.0, 1.0],
        ],
        u_knots: vec![0.0, 0.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    }
}

fn make_cylinder_nurbs() -> NurbsSurface {
    // Cylinder-like NURBS (degree 2 in u, 1 in v)
    // 90° arc as a rational quadratic NURBS
    let r = 5.0;
    let k = 0.7071067811865476; // sqrt(2)/2
    NurbsSurface {
        u_degree: 2,
        v_degree: 1,
        control_points: vec![
            vec![
                Point3d::new(r, 0.0, 0.0),
                Point3d::new(r, 0.0, 10.0),
            ],
            vec![
                Point3d::new(r, r, 0.0),
                Point3d::new(r, r, 10.0),
            ],
            vec![
                Point3d::new(0.0, r, 0.0),
                Point3d::new(0.0, r, 10.0),
            ],
        ],
        weights: vec![
            vec![1.0, 1.0],
            vec![k, k],
            vec![1.0, 1.0],
        ],
        u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    }
}

fn make_wavy_nurbs() -> NurbsSurface {
    // Wavy NURBS with high curvature variation
    let n = 10;
    let mut cps = Vec::new();
    let mut weights = Vec::new();
    for i in 0..n {
        let mut row = Vec::new();
        let mut wrow = Vec::new();
        for j in 0..n {
            let x = i as f64 * 2.0;
            let y = j as f64 * 2.0;
            let z = (x * 0.5).sin() * (y * 0.5).cos() * 3.0;
            row.push(Point3d::new(x, y, z));
            wrow.push(1.0);
        }
        cps.push(row);
        weights.push(wrow);
    }
    let mut u_knots = vec![0.0, 0.0, 0.0, 0.0];
    for i in 1..n-3 {
        u_knots.push(i as f64 / (n - 3) as f64);
    }
    u_knots.extend(vec![1.0, 1.0, 1.0, 1.0]);
    let v_knots = u_knots.clone();

    NurbsSurface {
        u_degree: 3,
        v_degree: 3,
        control_points: cps,
        weights,
        u_knots,
        v_knots,
        u_closed: false,
        v_closed: false,
    }
}

fn test_surface(name: &str, nurbs: &NurbsSurface, grid_size: usize) {
    let surface = Surface::Nurbs(nurbs.clone());
    let (u_min, u_max) = nurbs.u_range();
    let (v_min, v_max) = nurbs.v_range();

    println!("\n=== {} ===", name);
    println!("  degree: {}/{}, cps: {}x{}, grid_size: {}",
        nurbs.u_degree, nurbs.v_degree,
        nurbs.control_points.len(), nurbs.control_points[0].len(),
        grid_size);

    let n_test = 20;
    let mut success = 0;
    let mut fail = 0;
    let mut errors: Vec<f64> = Vec::new();
    let mut worst = (0.0, 0.0, 0.0); // (err, u, v)

    for i in 0..n_test {
        for j in 0..n_test {
            let u_true = u_min + (u_max - u_min) * i as f64 / (n_test - 1) as f64;
            let v_true = v_min + (v_max - v_min) * j as f64 / (n_test - 1) as f64;
            let p = surface.point_at(u_true, v_true);

            let (u_proj, v_proj) = brute_force_project_point(nurbs, &p, grid_size);
            let p_proj = surface.point_at(u_proj, v_proj);
            let err = p.distance_to(&p_proj);

            if err < 1e-6 {
                success += 1;
            } else {
                fail += 1;
                errors.push(err);
                if err > worst.0 {
                    worst = (err, u_true, v_true);
                }
            }
        }
    }

    let total = success + fail;
    println!("  success: {}/{} ({:.1}%)", success, total, 100.0 * success as f64 / total as f64);
    println!("  fail:    {}/{} ({:.1}%)", fail, total, 100.0 * fail as f64 / total as f64);
    if !errors.is_empty() {
        let mean: f64 = errors.iter().sum::<f64>() / errors.len() as f64;
        let max: f64 = errors.iter().cloned().fold(0./0., f64::max);
        println!("  error: mean={:.2e}, max={:.2e}", mean, max);
        println!("  worst: err={:.2e} at UV=({:.4}, {:.4})", worst.0, worst.1, worst.2);
    }
}

fn main() {
    println!("NURBS Projection Diagnosis");

    let flat = make_flat_nurbs();
    let cyl = make_cylinder_nurbs();
    let wavy = make_wavy_nurbs();

    for &gs in &[16, 32, 64] {
        test_surface(&format!("Flat (grid={})", gs), &flat, gs);
        test_surface(&format!("Cylinder-arc (grid={})", gs), &cyl, gs);
        test_surface(&format!("Wavy (grid={})", gs), &wavy, gs);
    }

    println!("\n=== Adaptive grid size ===");
    for (name, n) in &[("Flat", &flat), ("Cylinder", &cyl), ("Wavy", &wavy)] {
        let (u_min, u_max) = n.u_range();
        let (v_min, v_max) = n.v_range();
        let gs = adaptive_grid_size(u_max - u_min, v_max - v_min);
        println!("  {}: adaptive_grid_size({:.3}, {:.3}) = {}", name, u_max - u_min, v_max - v_min, gs);
    }
}
