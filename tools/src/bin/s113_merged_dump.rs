//! s113: dump the MERGED per-BREP instance mesh (post-merge, post-repair)
//! with per-triangle face ids, for offline comparison against the census
//! pair indices and the emission TRI_INPUT dumps.
//! Usage: s113_merged_dump <file.stp> <brep_id> <out_prefix>
//! Writes <out_prefix>.mesh.txt: "v x y z" per vertex, "t a b c f" per
//! triangle (f = face id, 0-based label same as converter face_id).

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/8394-121_Spit-Fire.STEP");
    let brep_id: i64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(27977);
    let out_path = args
        .get(3)
        .cloned()
        .unwrap_or_else(|| format!("/tmp/s113_merged_{}.mesh.txt", brep_id));

    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .parse_default_env()
        .init();

    let data = std::fs::read_to_string(path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        if p.brep_id != brep_id {
            continue;
        }
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => {
                eprintln!("BREP {} failed to triangulate", brep_id);
                continue;
            }
        };
        let mesh = &inst.mesh;
        let fids = mesh.triangle_face_ids.as_ref();
        let mut out = String::with_capacity(1 << 22);
        out.push_str(&format!(
            "# brep {} verts {} tris {}\n",
            brep_id,
            mesh.vertices.len(),
            mesh.triangles.len()
        ));
        for v in &mesh.vertices {
            out.push_str(&format!("v {:.9} {:.9} {:.9}\n", v.x, v.y, v.z));
        }
        for (ti, t) in mesh.triangles.iter().enumerate() {
            let f = fids.map(|v| v[ti]).unwrap_or(0);
            out.push_str(&format!("t {} {} {}\t{}\n", t[0], t[1], t[2], f));
        }
        std::fs::write(&out_path, out).expect("write mesh");
        println!(
            "wrote {} (brep {}, {} verts, {} tris)",
            out_path,
            brep_id,
            mesh.vertices.len(),
            mesh.triangles.len()
        );
        break; // first instance only (instances share the cached BREP mesh)
    }
}
