// Export 3.05.078.stp triangulation to STL for comparison
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::fs::File;
use std::io::Write;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("read 3.05.078.stp");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    let mut f = File::create("/home/z/my-project/download/3.05.078.stl").expect("create STL");
    writeln!(f, "solid 3Draper").unwrap();

    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            for tri in &inst.mesh.triangles {
                let v0 = inst.mesh.vertices[tri[0] as usize];
                let v1 = inst.mesh.vertices[tri[1] as usize];
                let v2 = inst.mesh.vertices[tri[2] as usize];
                // Compute normal
                let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                let nx = e1.1 * e2.2 - e1.2 * e2.1;
                let ny = e1.2 * e2.0 - e1.0 * e2.2;
                let nz = e1.0 * e2.1 - e1.1 * e2.0;
                let len = (nx*nx + ny*ny + nz*nz).sqrt();
                let (nx, ny, nz) = if len > 1e-20 { (nx/len, ny/len, nz/len) } else { (0.0, 0.0, 0.0) };
                writeln!(f, "  facet normal {} {} {}", nx, ny, nz).unwrap();
                writeln!(f, "    outer loop").unwrap();
                writeln!(f, "      vertex {} {} {}", v0.x, v0.y, v0.z).unwrap();
                writeln!(f, "      vertex {} {} {}", v1.x, v1.y, v1.z).unwrap();
                writeln!(f, "      vertex {} {} {}", v2.x, v2.y, v2.z).unwrap();
                writeln!(f, "    endloop").unwrap();
                writeln!(f, "  endfacet").unwrap();
            }
        }
    }
    writeln!(f, "endsolid 3Draper").unwrap();
    println!("STL exported to /home/z/my-project/download/3.05.078.stl");
}
