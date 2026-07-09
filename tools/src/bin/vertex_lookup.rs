// Print specific mesh vertices by index.
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/as1-oc-214_bolt.stp".to_string());
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

    // Print vertices near specific positions
    let targets = [
        (88.15, 52.50, -4.0),  // vertex 127 expected
        (87.99, 52.50, -4.0),  // vertex 126 expected
        (87.83, 42.50, -4.0),  // vertex 190 expected
        (87.99, 42.50, -4.0),  // vertex 189 expected
        (87.99, 52.50, 30.0),  // vertex 438 expected
        (87.99, 42.50, 30.0),  // vertex 501 expected
        (0.0, 5.0, 37.0),      // original coords UPPER top
        (0.0, -5.0, 37.0),     // original coords LOWER top
        (0.0, 5.0, 3.0),       // original coords UPPER bottom
        (0.0, -5.0, 3.0),      // original coords LOWER bottom
    ];

    let tol = 0.1;
    for (tx, ty, tz) in &targets {
        println!("\nTarget ({:.2}, {:.2}, {:.2}):", tx, ty, tz);
        let mut found = vec![];
        for (vi, v) in verts.iter().enumerate() {
            let dx = v.x - tx;
            let dy = v.y - ty;
            let dz = v.z - tz;
            if dx.abs() < tol && dy.abs() < tol && dz.abs() < tol {
                found.push((vi, v.x, v.y, v.z));
            }
        }
        // Also check transformed positions (87.99 + orig.x, 47.5 + orig.y, orig.z - 7)
        let (ox, oy, oz) = (tx - 87.99, ty - 47.5, tz + 7.0);
        for (vi, v) in verts.iter().enumerate() {
            let dx = v.x - ox;
            let dy = v.y - oy;
            let dz = v.z - oz;
            if dx.abs() < tol && dy.abs() < tol && dz.abs() < tol {
                found.push((vi, v.x, v.y, v.z));
            }
        }
        if found.is_empty() {
            println!("  NOT FOUND");
        } else {
            for (vi, x, y, z) in found {
                println!("  vertex {} at ({:.4}, {:.4}, {:.4})", vi, x, y, z);
            }
        }
    }

    // Print vertices 126, 127, 189, 190, 438, 501 directly
    println!("\nDirect vertex lookup:");
    for vi in [126, 127, 189, 190, 438, 501] {
        if vi < verts.len() {
            let v = &verts[vi];
            println!("  vertex {}: ({:.4}, {:.4}, {:.4})", vi, v.x, v.y, v.z);
        }
    }
}
