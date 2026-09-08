//! Sample the NURBS surface of specific faces on a UV grid to understand
//! the parametrization (folded? swapped? degenerate?).

use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::triangulate::SteinerBudgetProfile;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("test/as1-oc-214.stp").expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = pending.iter().find(|p| p.brep_id == 759).expect("rod");
    let mut ctx = OwnedStepConversionContext::new_with_lod_and_profile(
        step.clone(),
        0.75,
        SteinerBudgetProfile::Desktop,
    );
    let inst = ctx.triangulate_pending(p).expect("rod inst");

    for f in &inst.faces {
        if !f.surface_type.contains("Nurbs") { continue; }
        let s = &f.surface;
        let (u0, u1) = if let draper_geometry::Surface::Nurbs(n) = s { n.u_range() } else { (0.0, 1.0) };
        let (v0, v1) = if let draper_geometry::Surface::Nurbs(n) = s { n.v_range() } else { (0.0, 1.0) };
        println!("=== face #{} {} u=[{:.4},{:.4}] v=[{:.4},{:.4}] ===",
            f.step_face_id, f.surface_type, u0, u1, v0, v1);
        println!("  UV grid (rows: v from {:.2} to {:.2}, cols: u from {:.2} to {:.2}):", v0, v1, u0, u1);
        let vs = [0.0f64, 0.03, 1.0, 0.5, 0.97, 1.0];
        // sample: v fractions [0, 1/30, 1/30*?...] — include v=1.0 and v=29..30
        let v_samples: Vec<f64> = vec![v0, v0 + 1.0, v0 + (v1 - v0) * 0.5, v1 - 1.0, v1];
        let u_samples: Vec<f64> = vec![u0, u0 + 1.0, u0 + (u1 - u0) * 0.5, u1 - 1.0, u1];
        for &v in &v_samples {
            let mut row = String::new();
            for &u in &u_samples {
                let pt = s.point_at(u, v);
                row.push_str(&format!("({:8.3},{:8.3},{:8.3}) ", pt.x, pt.y, pt.z));
            }
            println!("  v={:9.4}: {}", v, row);
        }
        let _ = vs;
    }
}
