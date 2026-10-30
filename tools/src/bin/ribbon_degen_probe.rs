// ribbon_degen_probe — s118 satellite-(a) measurement probe.
//
// For every Nurbs face of the target BREP: degrees, cps dims, u/v ranges.
// For EXTREME-range faces (span > 100 — the s117 multi-coil ribbon class):
//   - grid-sample ANALYTIC |dS/du|, |dS/dv| (derivatives_at),
//   - the NUMERICAL partials is_degenerate_at uses (fixed eps = 1e-4),
//   - the resulting DegeneracyFlags at tolerance 1e-6 (what is_degenerate_uv
//     sees through the `_` branch) — counts of DU_ZERO / DV_ZERO / singular,
//   - the ribbon width: 3D distance P(u, v_min) → P(u, v_max) along u,
//   - a sweep of eps candidates (1e-4 … span*1e-3) showing at which eps the
//     numerical partials clear the 1e-6 tolerance (fix-(a) calibration).
//
// Usage: ribbon_degen_probe <file.stp> <brep_id>

use draper_geometry::Surface;
use draper_step::{parse_step, extract_solids};

fn vlen(x: f64, y: f64, z: f64) -> f64 {
    (x * x + y * y + z * z).sqrt()
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/8500-02_Vulcan.STEP");
    let brep_id: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(10220);

    let data = std::fs::read_to_string(path).expect("read STEP");
    let step = parse_step(&data).expect("parse STEP");
    // s118 bisect mode 2: full conversion, per-instance pre-merge tri counts
    // (the all_files_test metric) to localize emission-size changes.
    if std::env::var("DRAPPER_PROBE_CONVERT").is_ok() {
        let (_tree, pending) = draper_step::step_structure_lazy(&step);
        let ctx = draper_step::StepConversionContext::new(&step);
        let mut total = 0usize;
        for (i, p) in pending.iter().enumerate() {
            if let Some(inst) = ctx.triangulate_pending(p) {
                total += inst.mesh.triangle_count();
                println!("INST {} brep={} tris={}", i, inst.brep_id, inst.mesh.triangle_count());
            }
        }
        println!("TOTAL_PREMERGE {}", total);
        return;
    }
    let (solids, ids) = extract_solids(&step);
    println!(
        "# {} solids total",
        solids.len()
    );
    // s118 bisect mode: scan EVERY solid, list every Nurbs face with an
    // extreme parametric span (the fix-(a) trigger class).
    for (sidx, solid) in solids.iter().enumerate() {
        for (fi, face) in solid.faces().iter().enumerate() {
            let surface = match &face.surface {
                Some(s) => s.clone(),
                None => continue,
            };
            let nurbs = match &surface {
                Surface::Nurbs(n) => n,
                _ => continue,
            };
            let (u0, u1) = nurbs.u_range();
            let (v0, v1) = nurbs.v_range();
            let u_span = u1 - u0;
            let v_span = v1 - v0;
            if u_span > 100.0 || v_span > 100.0 {
                let n_cu = nurbs.control_points.len();
                let n_cv = if n_cu > 0 { nurbs.control_points[0].len() } else { 0 };
                println!(
                    "EXTREME solid#{} (BREP id {}) face #{fi}: Nurbs(deg={}/{}, cps={}x{}) u_span={:.1} v_span={:.4}",
                    sidx, ids[sidx], nurbs.u_degree, nurbs.v_degree, n_cu, n_cv, u_span, v_span
                );
            }
        }
    }
    let sidx = ids
        .iter()
        .position(|&id| id == brep_id)
        .unwrap_or_else(|| panic!("BREP {} not found among {} solids", brep_id, ids.len()));
    let solid = &solids[sidx];
    println!(
        "# BREP#{}: {} faces (of {} solids)",
        brep_id,
        solid.faces().len(),
        solids.len()
    );

    for (fi, face) in solid.faces().iter().enumerate() {
        let surface = match &face.surface {
            Some(s) => s.clone(),
            None => continue,
        };
        let nurbs = match &surface {
            Surface::Nurbs(n) => n,
            _ => continue,
        };
        let (u0, u1) = nurbs.u_range();
        let (v0, v1) = nurbs.v_range();
        let u_span = u1 - u0;
        let v_span = v1 - v0;
        let n_cu = nurbs.control_points.len();
        let n_cv = if n_cu > 0 { nurbs.control_points[0].len() } else { 0 };
        println!(
            "\nFACE #{fi}: Nurbs(deg={}/{}, cps={}x{}) u=[{:.4},{:.4}] span={:.1} v=[{:.4},{:.4}] span={:.4}",
            nurbs.u_degree, nurbs.v_degree, n_cu, n_cv, u0, u1, u_span, v0, v1, v_span
        );
        if u_span <= 100.0 && v_span <= 100.0 {
            println!("  (normal-range surface — not the ribbon class, skipping detail)");
            continue;
        }

        // Grid sample: 41 x 5 over the domain.
        let nu = 41;
        let nv = 5;
        let mut du_min = f64::INFINITY;
        let mut du_max: f64 = 0.0;
        let mut du_med: Vec<f64> = Vec::new();
        let mut dv_min = f64::INFINITY;
        let mut dv_max: f64 = 0.0;
        let mut dv_med: Vec<f64> = Vec::new();
        let mut n_du_zero = 0usize;
        let mut n_dv_zero = 0usize;
        let mut n_singular = 0usize;
        let mut n_pts = 0usize;
        let mut width_min = f64::INFINITY;
        let mut width_max: f64 = 0.0;
        for i in 0..nu {
            let u = u0 + (u1 - u0) * (i as f64) / ((nu - 1) as f64);
            // ribbon width at this u
            let pw0 = surface.point_at(u, v0);
            let pw1 = surface.point_at(u, v1);
            let w = vlen(pw1.x - pw0.x, pw1.y - pw0.y, pw1.z - pw0.z);
            width_min = width_min.min(w);
            width_max = width_max.max(w);
            for j in 0..nv {
                let v = v0 + (v1 - v0) * (j as f64) / ((nv - 1) as f64);
                let d = surface.derivatives_at(u, v);
                let dau = vlen(d.du.x, d.du.y, d.du.z);
                let dav = vlen(d.dv.x, d.dv.y, d.dv.z);
                du_min = du_min.min(dau);
                du_max = du_max.max(dau);
                du_med.push(dau);
                dv_min = dv_min.min(dav);
                dv_max = dv_max.max(dav);
                dv_med.push(dav);
                // replicate is_degenerate_at's numerical partials (eps = 1e-4)
                let eps = 1e-4_f64;
                let p = surface.point_at(u, v);
                let pu = surface.point_at(u + eps, v);
                let pv = surface.point_at(u, v + eps);
                let num_du = vlen(pu.x - p.x, pu.y - p.y, pu.z - p.z);
                let num_dv = vlen(pv.x - p.x, pv.y - p.y, pv.z - p.z);
                let du_zero = num_du < 1e-6;
                let dv_zero = num_dv < 1e-6;
                n_du_zero += du_zero as usize;
                n_dv_zero += dv_zero as usize;
                n_singular += (du_zero && dv_zero) as usize;
                n_pts += 1;
            }
        }
        du_med.sort_by(|a, b| a.partial_cmp(b).unwrap());
        dv_med.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!(
            "  ANALYTIC |dS/du|: min={:.3e} med={:.3e} max={:.3e} | |dS/dv|: min={:.3e} med={:.3e} max={:.3e}",
            du_min,
            du_med[du_med.len() / 2],
            du_max,
            dv_min,
            dv_med[dv_med.len() / 2],
            dv_max
        );
        println!(
            "  NUMERICAL eps=1e-4: DU_ZERO {}/{} DV_ZERO {}/{} SINGULAR {}/{}",
            n_du_zero, n_pts, n_dv_zero, n_pts, n_singular, n_pts
        );
        println!(
            "  RIBBON WIDTH (v_min→v_max 3D): min={:.3e} max={:.3e}",
            width_min, width_max
        );

        // eps sweep: at the domain center + corners, find the eps at which
        // the numerical u-partial clears 1e-6.
        for &(su, sv) in &[
            (0.5f64, 0.5f64),
            (0.01, 0.0),
            (0.99, 1.0),
            (0.5, 0.0),
            (0.5, 1.0),
        ] {
            let u = u0 + (u1 - u0) * su;
            let v = v0 + (v1 - v0) * sv;
            let p = surface.point_at(u, v);
            print!("  EPS-SWEEP (u={:.2}, v={:.3}):", u, v);
            let mut eps = 1e-4_f64;
            for _ in 0..7 {
                let pu = surface.point_at(u + eps, v);
                let pv = surface.point_at(u, v + eps);
                let num_du = vlen(pu.x - p.x, pu.y - p.y, pu.z - p.z);
                let num_dv = vlen(pv.x - p.x, pv.y - p.y, pv.z - p.z);
                print!(" eps={:.2e}:du={:.2e},dv={:.2e} |", eps, num_du, num_dv);
                eps *= 10.0;
            }
            println!();
        }
    }
    println!("\n# done");
}
