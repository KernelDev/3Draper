// Print specific mesh vertices by index for the rod.
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/as1-oc-214_rod.stp".to_string());
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];
    let ctx = StepConversionContext::new(&step);
    let inst = match ctx.triangulate_pending(p) {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); return; }
    };

    let verts = &inst.mesh.vertices;
    println!("Mesh has {} vertices", verts.len());

    // Print vertices 0, 62, 63, 93, 125, 126, 156, 188 directly
    for vi in [0, 62, 63, 93, 125, 126, 156, 188, 189, 190] {
        if vi < verts.len() {
            let v = &verts[vi];
            println!("  vertex {}: ({:.6}, {:.6}, {:.6})", vi, v.x, v.y, v.z);
        }
    }

    // Find all vertices near (60, -75, 15) — the corner position
    let target = (60.0, -75.0, 15.0);
    let tol = 0.5;
    println!("\nVertices near ({}, {}, {}):", target.0, target.1, target.2);
    for (vi, v) in verts.iter().enumerate() {
        let dx = v.x - target.0;
        let dy = v.y - target.1;
        let dz = v.z - target.2;
        if dx.abs() < tol && dy.abs() < tol && dz.abs() < tol {
            println!("  vertex {}: ({:.6}, {:.6}, {:.6}) dist={:.6}", vi, v.x, v.y, v.z,
                (dx*dx + dy*dy + dz*dz).sqrt());
        }
    }

    // Find all vertices near (-140, -75, 15) — the other corner
    let target = (-140.0, -75.0, 15.0);
    println!("\nVertices near ({}, {}, {}):", target.0, target.1, target.2);
    for (vi, v) in verts.iter().enumerate() {
        let dx = v.x - target.0;
        let dy = v.y - target.1;
        let dz = v.z - target.2;
        if dx.abs() < tol && dy.abs() < tol && dz.abs() < tol {
            println!("  vertex {}: ({:.6}, {:.6}, {:.6}) dist={:.6}", vi, v.x, v.y, v.z,
                (dx*dx + dy*dy + dz*dz).sqrt());
        }
    }
}
