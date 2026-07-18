// Quick check: 3.05.078.stp watertightness + triangle quality
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/3.05.078.stp")
        .expect("read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let report = validate_watertight(&inst.mesh, true);
            println!("=== 3.05.078.stp BREP #{} ===", p.brep_id);
            println!("  verts={} tris={}", inst.mesh.vertex_count(), inst.mesh.triangle_count());
            println!("  watertight={} boundary={}", report.is_watertight(), report.boundary_edge_count);

            // Check triangle quality - compute aspect ratios
            let mut min_angle = std::f64::MAX;
            let mut sharp_count = 0;
            for tri in &inst.mesh.triangles {
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                // Edge lengths
                let a = ((v1.x-v0.x).powi(2) + (v1.y-v0.y).powi(2) + (v1.z-v0.z).powi(2)).sqrt();
                let b = ((v2.x-v1.x).powi(2) + (v2.y-v1.y).powi(2) + (v2.z-v1.z).powi(2)).sqrt();
                let c = ((v0.x-v2.x).powi(2) + (v0.y-v2.y).powi(2) + (v0.z-v2.z).powi(2)).sqrt();
                // Angles using law of cosines
                let ang_a = ((b*b + c*c - a*a) / (2.0*b*c)).clamp(-1.0, 1.0).acos();
                let ang_b = ((a*a + c*c - b*b) / (2.0*a*c)).clamp(-1.0, 1.0).acos();
                let ang_c = ((a*a + b*b - c*c) / (2.0*a*b)).clamp(-1.0, 1.0).acos();
                let min_a = ang_a.min(ang_b).min(ang_c);
                if min_a < min_angle { min_angle = min_a; }
                if min_a < 10.0_f64.to_radians() { sharp_count += 1; }
            }
            println!("  min triangle angle: {:.2}°", min_angle.to_degrees());
            println!("  sharp triangles (<10°): {} / {}", sharp_count, inst.mesh.triangles.len());
        }
    }
}
