//! session-97 diagnostic: dump UV domains + emission for GEAR faces
//! STEP #15826 (census 421) and #15893 (census 426) — two patches on the
//! SAME CYLINDRICAL_SURFACE (#15816/#15886, identical placement+radius).
//! Question: do their UV domains overlap (duplicate faces) or are they
//! disjoint-adjacent (emission spill / weld bug)?
//! Usage: s97_gear_uv_dump <file.stp> <brep_id> <face_id_a> <face_id_b>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn uv_bbox(polys: &[Vec<draper_geometry::Point2d>]) -> (f64, f64, f64, f64, usize) {
    let mut min_u = f64::MAX; let mut max_u = f64::MIN;
    let mut min_v = f64::MAX; let mut max_v = f64::MIN;
    let mut n = 0usize;
    for p in polys {
        for pt in p {
            min_u = min_u.min(pt.u); max_u = max_u.max(pt.u);
            min_v = min_v.min(pt.v); max_v = max_v.max(pt.v);
            n += 1;
        }
    }
    (min_u, max_u, min_v, max_v, n)
}

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let brep_id: i64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(16033);
    let id_a: i64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(15826);
    let id_b: i64 = std::env::args().nth(4).and_then(|s| s.parse().ok()).unwrap_or(15893);

    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    for p in &pending {
        if p.brep_id != brep_id { continue; }
        let ctx = StepConversionContext::new(&step);
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => { println!("BREP #{}: triangulate_pending returned None", brep_id); continue; }
        };

        println!("=== BREP #{} '{}' ===", brep_id, inst.name);
        for (target, label) in [(id_a, "A"), (id_b, "B")] {
            let fidx = inst.faces.iter().position(|f| f.step_face_id == target);
            let fi = match fidx {
                Some(i) => i,
                None => { println!("face STEP#{} NOT FOUND", target); continue; }
            };
            let f = &inst.faces[fi];
            let (dmin_u, dmax_u, dmin_v, dmax_v, dn) = uv_bbox(&f.outer_uv_boundary);
            let (emin_u, emax_u, emin_v, emax_v, en) = uv_bbox(
                &f.uv_triangles.iter().map(|t| vec![t[0], t[1], t[2]]).collect::<Vec<_>>(),
            );
            println!("\n--- {} face_idx={} STEP#{} {} fwd={} is_void={} tris=({},{})",
                label, fi + 1, f.step_face_id, f.surface_type, f.forward, f.is_void,
                f.triangle_range.0, f.triangle_range.1);
            println!("  DOMAIN  u=[{:.6},{:.6}] v=[{:.6},{:.6}] pts={}", dmin_u, dmax_u, dmin_v, dmax_v, dn);
            println!("  EMISSION u=[{:.6},{:.6}] v=[{:.6},{:.6}] tris_uv={}", emin_u, emax_u, emin_v, emax_v, en);

            // FULL boundary polylines (compact: u,v pairs, run-length of constant values)
            for (li, poly) in f.outer_uv_boundary.iter().enumerate() {
                println!("  bnd loop #{} pts={}:", li, poly.len());
                let mut line = String::new();
                for (i, pt) in poly.iter().enumerate() {
                    line.push_str(&format!("({:.4},{:.4})", pt.u, pt.v));
                    if line.len() > 100 {
                        println!("    [{}] {}", i, line);
                        line.clear();
                    }
                }
                if !line.is_empty() { println!("    [end] {}", line); }
            }
            // count inner holes
            println!("  holes={}", f.inner_uv_boundaries.len());
        }

        // UV overlap test between A and B domains
        let fa = inst.faces.iter().find(|f| f.step_face_id == id_a);
        let fb = inst.faces.iter().find(|f| f.step_face_id == id_b);
        if let (Some(a), Some(b)) = (fa, fb) {
            // Histogram of uv_triangles u-mid coverage in 12 angular bins
            for (f, label) in [(a, "A"), (b, "B")] {
                let mut bins = [0usize; 12];
                for t in &f.uv_triangles {
                    let um = (t[0].u + t[1].u + t[2].u) / 3.0;
                    let un = if um < 0.0 { um + 2.0 * std::f64::consts::PI } else { um };
                    let b = ((un / (2.0 * std::f64::consts::PI)) * 12.0) as usize;
                    let b = b.min(11);
                    bins[b] += 1;
                }
                println!("\n  {} uv_tri u-mid histogram (12 bins over [0,2π]): {:?}", label, bins);
                // Stray detail: triangles outside the face's own half.
                // A owns [π, 2π]; B owns [0, π]. Wrap-aware test.
                let (lo, hi) = if label == "A" {
                    (std::f64::consts::PI, 2.0 * std::f64::consts::PI)
                } else {
                    (0.0, std::f64::consts::PI)
                };
                for (ti, t) in f.uv_triangles.iter().enumerate() {
                    let us: Vec<f64> = t.iter().map(|p| {
                        let mut u = p.u;
                        while u < 0.0 { u += 2.0 * std::f64::consts::PI; }
                        while u >= 2.0 * std::f64::consts::PI { u -= 2.0 * std::f64::consts::PI; }
                        u
                    }).collect();
                    let um = (us[0] + us[1] + us[2]) / 3.0;
                    // foreign mid: A's mids must be in [π,2π], B's in [0,π]
                    let foreign = if label == "A" { um < std::f64::consts::PI } else { um > std::f64::consts::PI };
                    // out-of-domain v verts: v < 0.01 - eps or v > 0.22 + eps
                    let vout = t.iter().any(|p| p.v < 0.01 - 1e-9 || p.v > 0.22 + 1e-9);
                    if foreign || vout {
                        println!("    {} tri #{}{}{}: u=({:.6},{:.6},{:.6}) v=({:.4},{:.4},{:.4})",
                            label, ti, if foreign {" FOREIGN"} else {""}, if vout {" VOUT"} else {""},
                            us[0], us[1], us[2], t[0].v, t[1].v, t[2].v);
                    }
                }
            }
            // Merged mesh triangles: angular coverage around world Y axis (gear axis).
            // World coords of GEAR: use inst.mesh verts of each face's triangle_range.
            for (f, label) in [(a, "A"), (b, "B")] {
                let mut bins = [0usize; 12];
                let (t0, t1) = f.triangle_range;
                for ti in t0..t1 {
                    let tri = inst.mesh.triangles[ti];
                    let v0 = &inst.mesh.vertices[tri[0] as usize];
                    let v1 = &inst.mesh.vertices[tri[1] as usize];
                    let v2 = &inst.mesh.vertices[tri[2] as usize];
                    let cx = (v0.x + v1.x + v2.x) / 3.0;
                    let cz = (v0.z + v1.z + v2.z) / 3.0;
                    let ang = cz.atan2(cx); // [-π, π]
                    let a01 = if ang < 0.0 { ang + 2.0 * std::f64::consts::PI } else { ang };
                    let b = ((a01 / (2.0 * std::f64::consts::PI)) * 12.0) as usize;
                    bins[b.min(11)] += 1;
                }
                println!("  {} merged tris={} world-angle histogram: {:?}", label, t1 - t0, bins);
            }

            // Dump the census FOLD-OVER pair triangles by merged vertex ids.
            let targets: [(u32, u32, u32); 4] = [
                (1215, 123, 1217), (367, 1217, 123),
                (336, 1218, 92), (1216, 92, 1218),
            ];
            println!("\n=== FOLD-OVER PAIR WORLD DUMP (merged vertex ids) ===");
            for (i0, i1, i2) in targets {
                // find the triangle in the merged mesh (any rotation)
                let mut found = false;
                for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                    let v = [tri[0], tri[1], tri[2]];
                    // check cyclic match
                    for rot in 0..3 {
                        if v[rot] == i0 && v[(rot+1)%3] == i1 && v[(rot+2)%3] == i2 {
                            let p0 = &inst.mesh.vertices[i0 as usize];
                            let p1 = &inst.mesh.vertices[i1 as usize];
                            let p2 = &inst.mesh.vertices[i2 as usize];
                            println!("  tri#{} ({},{},{}): ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4})",
                                ti, i0, i1, i2,
                                p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
                            found = true;
                            break;
                        }
                    }
                    if found { break; }
                }
                if !found {
                    // maybe winding reversed: search (i0,i2,i1)
                    for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                        let v = [tri[0], tri[1], tri[2]];
                        for rot in 0..3 {
                            if v[rot] == i0 && v[(rot+1)%3] == i2 && v[(rot+2)%3] == i1 {
                                let p0 = &inst.mesh.vertices[i0 as usize];
                                let p1 = &inst.mesh.vertices[i1 as usize];
                                let p2 = &inst.mesh.vertices[i2 as usize];
                                println!("  tri#{} ({},{},{})*REV*: ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4})",
                                    ti, i0, i1, i2,
                                    p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
                                found = true;
                                break;
                            }
                        }
                        if found { break; }
                    }
                }
                if !found { println!("  ({},{},{}) NOT FOUND in merged mesh", i0, i1, i2); }
            }
            // Dump the individual vertices
            println!("\n=== VERTEX DUMP ===");
            for vi in [92u32, 123, 336, 367, 1215, 1216, 1217, 1218] {
                let p = &inst.mesh.vertices[vi as usize];
                println!("  v{}: ({:.4},{:.4},{:.4})", vi, p.x, p.y, p.z);
            }
        }
    }
}
