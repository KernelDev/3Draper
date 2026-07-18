// Extract and test all NURBS surfaces from a STEP file
// Identifies which surfaces cause projection failures
use draper_geometry::{Surface, Point3d};
use draper_mesh::edge_cache::{brute_force_project_point, nurbs_surface_hash};
use draper_step::parser::parse_step;
use draper_step::StepConverter;
use std::collections::HashMap;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();

    let path = std::env::args().nth(1).unwrap_or("test/transmission_top.stp".to_string());
    let content = std::fs::read_to_string(&path).expect("read file");
    let step_file = parse_step(&content).expect("parse STEP");

    // Find all B_SPLINE_SURFACE entities
    let bspline_entities = step_file.find_entities_by_type("B_SPLINE_SURFACE_WITH_KNOTS");
    println!("File: {}", path);
    println!("B_SPLINE_SURFACE_WITH_KNOTS entities: {}", bspline_entities.len());

    let converter = StepConverter::new(&step_file);

    let mut nurbs_surfaces: HashMap<u64, (draper_geometry::NurbsSurface, i64)> = HashMap::new();
    for entity in &bspline_entities {
        if let Some(Surface::Nurbs(n)) = converter.extract_bspline_surface(entity) {
            let h = nurbs_surface_hash(&n);
            nurbs_surfaces.entry(h).or_insert((n, entity.id));
        }
    }

    println!("Unique NURBS surfaces extracted: {}", nurbs_surfaces.len());
    println!();

    for (i, (h, (nurbs, step_id))) in nurbs_surfaces.iter().enumerate() {
        println!("=== NURBS #{} (hash={:x}, STEP#{}) ===", i, h, step_id);
        println!("  degree: {}/{}", nurbs.u_degree, nurbs.v_degree);
        println!("  control points: {}x{}", nurbs.control_points.len(), nurbs.control_points[0].len());
        println!("  u_knots: {}, v_knots: {}", nurbs.u_knots.len(), nurbs.v_knots.len());

        let (u_min, u_max) = nurbs.u_range();
        let (v_min, v_max) = nurbs.v_range();
        println!("  u_range: [{:.4}, {:.4}], v_range: [{:.4}, {:.4}]", u_min, u_max, v_min, v_max);
        let surface = Surface::Nurbs(nurbs.clone());

        // Bounding box of control points (rough surface extent)
        let mut bb_min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
        let mut bb_max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
        for row in &nurbs.control_points {
            for p in row {
                bb_min.x = bb_min.x.min(p.x);
                bb_min.y = bb_min.y.min(p.y);
                bb_min.z = bb_min.z.min(p.z);
                bb_max.x = bb_max.x.max(p.x);
                bb_max.y = bb_max.y.max(p.y);
                bb_max.z = bb_max.z.max(p.z);
            }
        }
        println!("  bbox: [{:.2},{:.2},{:.2}] to [{:.2},{:.2},{:.2}]",
            bb_min.x, bb_min.y, bb_min.z, bb_max.x, bb_max.y, bb_max.z);

        // Test projection on a grid
        let n_test = 15;
        let mut success = 0;
        let mut fail = 0;
        let mut errors: Vec<f64> = Vec::new();
        let mut worst = (0.0, 0.0, 0.0);

        for i in 0..n_test {
            for j in 0..n_test {
                let u_true = u_min + (u_max - u_min) * i as f64 / (n_test - 1) as f64;
                let v_true = v_min + (v_max - v_min) * j as f64 / (n_test - 1) as f64;
                let p = surface.point_at(u_true, v_true);

                let (u_proj, v_proj) = brute_force_project_point(nurbs, &p, 32);
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
        println!("  Projection test ({}x{} grid):", n_test, n_test);
        println!("    success: {}/{} ({:.1}%)", success, total, 100.0 * success as f64 / total as f64);
        println!("    fail:    {}/{} ({:.1}%)", fail, total, 100.0 * fail as f64 / total as f64);
        if !errors.is_empty() {
            let mean: f64 = errors.iter().sum::<f64>() / errors.len() as f64;
            let max: f64 = errors.iter().cloned().fold(0./0., f64::max);
            println!("    error: mean={:.2e}, max={:.2e}", mean, max);
            println!("    worst: err={:.2e} at UV=({:.4}, {:.4})", worst.0, worst.1, worst.2);
        }
        println!();
    }
}
