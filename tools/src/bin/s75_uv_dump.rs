//! session-75 diagnostic #2: dump the UV domain (boundary polygons) and
//! UV triangulation of specific faces, to decide whether degenerate
//! rim-spanning triangles come from the per-face earcut (UV level) or
//! from weld/repair (3D level).
//!
//! Usage: s75_uv_dump <file.stp> <pending_idx> <fid[,fid...]>
//! Writes /tmp/s75_uv_<fid>.txt with polygon polylines + triangles.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::Point2d;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "test/drill_top.stp".to_string());
    let idx: usize = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(4);
    let fids: Vec<u64> = args
        .get(3)
        .map(|s| s.split(',').filter_map(|v| v.parse().ok()).collect())
        .unwrap_or_else(|| vec![57, 58, 105, 113]);

    println!("Loading {} pending={} faces={:?}", path, idx, fids);
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    let inst = ctx.triangulate_pending(&pending[idx]).expect("triangulate");

    for fid in fids {
        let Some(f) = inst.faces.iter().find(|f| f.face_id == fid) else {
            println!("face {} not found", fid);
            continue;
        };
        let path = format!("/tmp/s75_uv_{}.txt", fid);
        let mut out = String::new();
        out.push_str(&format!(
            "# face {} {} step={} fwd={} tris={}\n",
            fid,
            f.surface_type,
            f.step_face_id,
            f.forward,
            f.uv_triangles.len()
        ));
        for (i, poly) in f.outer_uv_boundary.iter().enumerate() {
            out.push_str(&format!("POLY outer {} {}\n", i, poly.len()));
            for p in poly {
                out.push_str(&format!("  o {:.8} {:.8}\n", p.u, p.v));
            }
        }
        for (i, polys) in f.inner_uv_boundaries.iter().enumerate() {
            for (j, poly) in polys.iter().enumerate() {
                out.push_str(&format!("POLY inner {}-{} {}\n", i, j, poly.len()));
                for p in poly {
                    out.push_str(&format!("  i {:.8} {:.8}\n", p.u, p.v));
                }
            }
        }
        let area2 = |t: &[Point2d; 3]| -> f64 {
            (t[1].u - t[0].u) * (t[2].v - t[0].v)
                - (t[1].v - t[0].v) * (t[2].u - t[0].u)
        };
        for (ti, t) in f.uv_triangles.iter().enumerate() {
            out.push_str(&format!(
                "TRI {:5} ({:.6},{:.6}) ({:.6},{:.6}) ({:.6},{:.6}) area2={:.3e}\n",
                ti,
                t[0].u,
                t[0].v,
                t[1].u,
                t[1].v,
                t[2].u,
                t[2].v,
                area2(t)
            ));
        }
        let _ = std::fs::write(&path, out);
        let zero_area = f
            .uv_triangles
            .iter()
            .filter(|t| area2(t).abs() < 1e-10)
            .count();
        let tiny_area = f
            .uv_triangles
            .iter()
            .filter(|t| area2(t).abs() < 1e-4)
            .count();
        println!(
            "face {}: {} uv tris ({} zero-area <1e-10, {} tiny <1e-4), dumped {}",
            fid,
            f.uv_triangles.len(),
            zero_area,
            tiny_area,
            path
        );
    }
}
