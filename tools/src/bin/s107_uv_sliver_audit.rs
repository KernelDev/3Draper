//! session-107 diagnostic: audit a face's emission UV triangles for
//! degenerate (zero-coverage) slivers — the (3,243) class where three
//! near-collinear boundary points along a straight trim (u=const,
//! jitter ~1e-8) produce a phantom triangle that covers no UV area but
//! still enters the merge/winding pipeline and manufactures a REAL
//! cross-face fold pair via the BFS ripple.
//!
//! For the requested (brep, face):
//!   1. dump every emission UV triangle's area, longest UV edge and the
//!      sliver ratio h/longest (apex height over the longest edge,
//!      normalized) — full f64 precision;
//!   2. build the face's UV edge-usage table (edges keyed by exact UV
//!      position pairs) and, for each degenerate candidate, report
//!      whether ALL THREE of its edges are covered by other triangles
//!      (drop-safe: removal opens no edge) or not;
//!   3. 3D diagnostics through Surface::point_at for the worst slivers.
//!
//! Usage: s107_uv_sliver_audit <file.stp> <brep_id> <face_id> [ratio_eps=1e-4] [max_report=12]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn key(p: &draper_geometry::Point2d) -> (u64, u64) {
    (p.u.to_bits(), p.v.to_bits())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "test/drill_top.stp".into());
    let brep_id: i64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(62542);
    let face_id: u64 = args
        .get(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(243);
    let ratio_eps: f64 = args
        .get(4)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1e-4);
    let max_report: usize = args
        .get(5)
        .and_then(|s| s.parse().ok())
        .unwrap_or(12);

    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    for p in &pending {
        if p.brep_id != brep_id {
            continue;
        }
        let ctx = StepConversionContext::new(&step);
        let Some(inst) = ctx.triangulate_pending(p) else {
            println!("BREP #{}: triangulate_pending None", brep_id);
            continue;
        };
        let mesh = &inst.mesh;
        let fids = mesh.triangle_face_ids.as_ref();
        let face = inst
            .faces
            .iter()
            .find(|f| f.face_id == face_id)
            .unwrap_or_else(|| {
                eprintln!("face {} not found in BREP #{}", face_id, brep_id);
                std::process::exit(2);
            });
        println!(
            "=== BREP #{} face {} {} forward={} uv_tris={} range=({},{}) ===",
            brep_id,
            face_id,
            face.surface_type,
            face.forward,
            face.uv_triangles.len(),
            face.triangle_range.0,
            face.triangle_range.1
        );

        // Edge usage among this face's emission triangles (UV-position keys).
        let mut edge_usage: std::collections::HashMap<((u64, u64), (u64, u64)), usize> =
            std::collections::HashMap::new();
        for t in &face.uv_triangles {
            let k: Vec<(u64, u64)> = t.iter().map(key).collect();
            for i in 0..3 {
                let a = k[i];
                let b = k[(i + 1) % 3];
                if a != b {
                    let e = if a < b { (a, b) } else { (b, a) };
                    *edge_usage.entry(e).or_insert(0) += 1;
                }
            }
        }

        // Per-triangle sliver ratio + drop-safety.
        let mut rows: Vec<(usize, f64, f64, f64, [usize; 3], bool)> = Vec::new();
        for (ti, t) in face.uv_triangles.iter().enumerate() {
            let (u0, v0) = (t[0].u, t[0].v);
            let (u1, v1) = (t[1].u, t[1].v);
            let (u2, v2) = (t[2].u, t[2].v);
            let cross = (u1 - u0) * (v2 - v0) - (u2 - u0) * (v1 - v0); // 2*area
            let d01 = ((u1 - u0).powi(2) + (v1 - v0).powi(2)).sqrt();
            let d12 = ((u2 - u1).powi(2) + (v2 - v1).powi(2)).sqrt();
            let d20 = ((u0 - u2).powi(2) + (v0 - v2).powi(2)).sqrt();
            let longest = d01.max(d12).max(d20);
            let ratio = if longest > 0.0 {
                cross.abs() / (longest * longest)
            } else {
                f64::INFINITY
            };
            // drop-safety: every edge used by >= 2 triangles (self + other)
            let k: Vec<(u64, u64)> = t.iter().map(key).collect();
            let mut safe = true;
            let mut usage = [0usize; 3];
            for i in 0..3 {
                let a = k[i];
                let b = k[(i + 1) % 3];
                if a == b {
                    continue;
                }
                let e = if a < b { (a, b) } else { (b, a) };
                let u = edge_usage.get(&e).copied().unwrap_or(0);
                usage[i] = u;
                if u < 2 {
                    safe = false;
                }
            }
            rows.push((ti, cross, longest, ratio, usage, safe));
        }

        rows.sort_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal));
        let n_degen = rows.iter().filter(|r| r.3 < ratio_eps).count();
        println!(
            "degenerate candidates (ratio < {:.0e}): {} of {}",
            ratio_eps,
            n_degen,
            face.uv_triangles.len()
        );
        for (ti, cross, longest, ratio, usage, safe) in rows.iter().take(max_report) {
            let t = &face.uv_triangles[*ti];
            println!(
                "  uv_tri#{ti} 2*area={cross:e} longest={longest:e} ratio={ratio:e} edge_usage={usage:?} drop_safe={safe}"
            );
            for p in t {
                println!("    ({:?}, {:?})", p.u, p.v);
            }
            // 3D footprint through the surface
            let p3: Vec<_> = t
                .iter()
                .map(|uv| face.surface.point_at(uv.u, uv.v))
                .collect();
            let e1 = (
                p3[1].x - p3[0].x,
                p3[1].y - p3[0].y,
                p3[1].z - p3[0].z,
            );
            let e2 = (
                p3[2].x - p3[0].x,
                p3[2].y - p3[0].y,
                p3[2].z - p3[0].z,
            );
            let n = (
                e1.1 * e2.2 - e1.2 * e2.1,
                e1.2 * e2.0 - e1.0 * e2.2,
                e1.0 * e2.1 - e1.1 * e2.0,
            );
            let a3 = 0.5 * (n.0 * n.0 + n.1 * n.1 + n.2 * n.2).sqrt();
            println!("    3D area={:e}", a3);
        }

        // Also: the final-mesh triangles of this face — count >170° same-face
        // pairs at emission (analytic, world frame) for context.
        if let Some(f) = fids {
            let (s, e) = face.triangle_range;
            let mut pairs = 0usize;
            let nrm = |t: &[u32; 3]| -> Option<[f64; 3]> {
                let a = &mesh.vertices[t[0] as usize];
                let b = &mesh.vertices[t[1] as usize];
                let c = &mesh.vertices[t[2] as usize];
                let e1 = [b.x - a.x, b.y - a.y, b.z - a.z];
                let e2 = [c.x - a.x, c.y - a.y, c.z - a.z];
                let n = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ];
                let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                if l > 1e-15 {
                    Some([n[0] / l, n[1] / l, n[2] / l])
                } else {
                    None
                }
            };
            // edge -> tris (final mesh, face-local range)
            let mut em: std::collections::HashMap<(u32, u32), Vec<usize>> =
                std::collections::HashMap::new();
            for ti in s..e {
                let t = &mesh.triangles[ti];
                for i in 0..3 {
                    let a = t[i];
                    let b = t[(i + 1) % 3];
                    let k = if a < b { (a, b) } else { (b, a) };
                    em.entry(k).or_default().push(ti);
                }
            }
            for ts in em.values() {
                if ts.len() != 2 {
                    continue;
                }
                if let (Some(n1), Some(n2)) = (nrm(&mesh.triangles[ts[0]]), nrm(&mesh.triangles[ts[1]]))
                {
                    let dot = (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]).clamp(-1.0, 1.0);
                    if dot.acos().to_degrees() > 170.0 {
                        pairs += 1;
                    }
                }
            }
            println!(
                "final-mesh face-local >170° same-face pairs (analytic normals): {}",
                pairs
            );
        }
        break;
    }
}
