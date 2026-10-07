// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! s82: replay the PRODUCTION per-face CDT
//! (custom_cdt::triangulate_polygon_cdt) on a DRAPPER_DUMP_TRI_INPUT
//! dump and report quality metrics — the acceptance-gate blind spots
//! the s81 addendum identified:
//!   - interior drops (Steiner points on no triangle)
//!   - rim coverage (missing rim edges after repair_unused_ring_
//!     vertices — the s82 plan item 3)
//!   - non-rim boundary edges (one-sided interior seams)
//!   - signed-area census (negative = winding inversions)
//!   - overlap ratio (sum|A| vs polygon area — s82 plan item 2)
//!   - UV needle census (area/max_edge^2 < 1e-3 — fold-prone slivers)
//!
//! Usage: cdt_replay <dump1.txt> [dump2.txt ...]
//!   DRAPPER_CDT_DEBUG=1 — production drop census (ring/sliver/not_found)
//!   The production guard mode env (DRAPPER_CDT_GUARD_MODE) applies.

use std::collections::HashMap;

fn parse_dump(path: &str) -> Option<ParsedDump> {
    let data = std::fs::read_to_string(path).ok()?;
    let mut lines = data.lines();
    let header = lines.next()?;
    let mut n_boundary = 0usize;
    let mut label = String::new();
    for tok in header.split_whitespace() {
        if let Some(v) = tok.strip_prefix("n_boundary=") {
            n_boundary = v.parse().ok()?;
        } else if let Some(v) = tok.strip_prefix("label=") {
            label = v.to_string();
        }
    }
    // label may contain spaces: take the rest of the header after "label="
    if let Some(pos) = header.find("label=") {
        label = header[pos + "label=".len()..].trim().to_string();
    }

    let mut boundary: Vec<[f64; 2]> = Vec::new();
    let mut holes: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut interior: Vec<[f64; 2]> = Vec::new();
    let mut section = 0u8; // 1=bnd, 2=hole, 3=interior
    for line in lines {
        if line.starts_with("boundary") {
            section = 1;
        } else if line.starts_with("hole ") {
            holes.push(Vec::new());
            section = 2;
        } else if line.starts_with("interior") {
            section = 3;
        } else if let Some(rest) = line.strip_prefix("b ") {
            if section == 1 {
                let mut it = rest.split_whitespace();
                if let (Some(u), Some(v)) = (it.next(), it.next()) {
                    if let (Ok(u), Ok(v)) = (u.parse::<f64>(), v.parse::<f64>()) {
                        boundary.push([u, v]);
                    }
                }
            }
        } else if let Some(rest) = line.strip_prefix("h ") {
            if section == 2 {
                let mut it = rest.split_whitespace();
                if let (Some(u), Some(v)) = (it.next(), it.next()) {
                    if let (Ok(u), Ok(v)) = (u.parse::<f64>(), v.parse::<f64>()) {
                        holes.last_mut().map(|h| h.push([u, v]));
                    }
                }
            }
        } else if let Some(rest) = line.strip_prefix("i ") {
            if section == 3 {
                let mut it = rest.split_whitespace();
                if let (Some(u), Some(v)) = (it.next(), it.next()) {
                    if let (Ok(u), Ok(v)) = (u.parse::<f64>(), v.parse::<f64>()) {
                        interior.push([u, v]);
                    }
                }
            }
        }
    }
    if boundary.len() < 3 {
        return None;
    }
    let _ = n_boundary; // parsed from points directly
    Some(ParsedDump {
        label,
        boundary,
        holes,
        interior,
    })
}

struct ParsedDump {
    label: String,
    boundary: Vec<[f64; 2]>,
    holes: Vec<Vec<[f64; 2]>>,
    interior: Vec<[f64; 2]>,
}

fn area2(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Compact metrics for one pipeline stage (CDT_REPLAY_STAGED).
fn report_stage(
    stage: &str,
    d: &ParsedDump,
    all_2d: &[[f64; 2]],
    tris: &[[u32; 3]],
    n_ring_total: usize,
) {
    let n_b = d.boundary.len();
    let mut used = vec![false; all_2d.len()];
    for t in tris {
        for &i in t {
            if (i as usize) < used.len() {
                used[i as usize] = true;
            }
        }
    }
    let ring_unused = (0..n_ring_total).filter(|&i| !used[i]).count();
    let mut ecount: HashMap<(u32, u32), usize> = HashMap::new();
    for t in tris {
        for k in 0..3 {
            let a = t[k];
            let b = t[(k + 1) % 3];
            if a != b {
                *ecount.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
    }
    let mut rim_set = std::collections::HashSet::new();
    let mut rim_edges = 0usize;
    let mut idx = 0usize;
    for ring in std::iter::once(&d.boundary).chain(d.holes.iter()) {
        let n = ring.len();
        for i in 0..n {
            let j = (i + 1) % n;
            rim_set.insert((((idx + i) as u32).min((idx + j) as u32), ((idx + i) as u32).max((idx + j) as u32)));
            if ecount.contains_key(&((idx + i) as u32, (idx + j) as u32)) {
                rim_edges += 1;
            }
        }
        idx += n;
    }
    let extra_bnd = ecount
        .iter()
        .filter(|(e, &n)| n == 1 && !rim_set.contains(e))
        .count();
    let non_manifold = ecount.iter().filter(|(_, &n)| n > 2).count();
    let mut neg = 0usize;
    let mut zero = 0usize;
    let mut sum_abs = 0.0f64;
    let mut needles = 0usize;
    let mut areas: Vec<f64> = Vec::with_capacity(tris.len());
    for t in tris {
        let (a, b, c) = (
            all_2d[t[0] as usize],
            all_2d[t[1] as usize],
            all_2d[t[2] as usize],
        );
        let s = area2(a, b, c);
        if s < 0.0 {
            neg += 1;
        }
        if s == 0.0 {
            zero += 1;
        }
        let abs_half = s.abs() * 0.5;
        areas.push(abs_half);
        sum_abs += abs_half;
        let e = |p: [f64; 2], q: [f64; 2]| {
            let dx = p[0] - q[0];
            let dy = p[1] - q[1];
            dx * dx + dy * dy
        };
        let max_e2 = e(a, b).max(e(b, c)).max(e(c, a));
        if max_e2 > 0.0 && abs_half < 1e-3 * max_e2 {
            needles += 1;
        }
    }
    areas.sort_by(|x, y| x.partial_cmp(y).unwrap());
    let shoelace = |ring: &[[f64; 2]]| -> f64 {
        let mut s = 0.0;
        let n = ring.len();
        for i in 0..n {
            let j = (i + 1) % n;
            s += ring[i][0] * ring[j][1] - ring[j][0] * ring[i][1];
        }
        s * 0.5
    };
    let poly_area = (shoelace(&d.boundary).abs()
        - d.holes.iter().map(|h| shoelace(h).abs()).sum::<f64>())
    .abs();
    let overlap = (sum_abs - poly_area) / poly_area.max(1e-30);
    let _ = n_b;
    println!(
        "  [{}] tris={} ring_unused={} rim {}/{} extra_bnd={} nm={} neg={} zero={} overlap={:.4} needles={} area_max={:.3e}",
        stage,
        tris.len(),
        ring_unused,
        rim_edges,
        rim_set.len(),
        extra_bnd,
        non_manifold,
        neg,
        zero,
        overlap,
        needles,
        areas.last().copied().unwrap_or(0.0),
    );
    // s82 defect dump (CDT_REPLAY_DEFECTS=1): exact topology of the
    // damage — nm edges, one-sided non-rim edges, overlapping pairs.
    if std::env::var("CDT_REPLAY_DEFECTS").is_ok() {
        for (e, n) in ecount.iter() {
            if *n > 2 {
                println!(
                    "    NM edge ({},{}) x{} tris={:?}",
                    e.0,
                    e.1,
                    n,
                    tris
                        .iter()
                        .enumerate()
                        .filter(|(_, t)| {
                            t.contains(&e.0) && t.contains(&e.1)
                        })
                        .map(|(i, t)| (i, *t))
                        .collect::<Vec<_>>()
                );
            }
        }
        for (e, n) in ecount.iter() {
            if *n == 1 && !rim_set.contains(e) {
                println!(
                    "    1-SIDED edge ({},{}) tri={:?}",
                    e.0,
                    e.1,
                    tris.iter().find(|t| t.contains(&e.0) && t.contains(&e.1)).copied()
                );
            }
        }
        // brute-force overlapping pairs (segment-segment intersection
        // between triangle perimeters, sharing no vertex)
        let seg_int = |p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], p4: [f64; 2]| -> bool {
            let d1 = area2(p1, p2, p3);
            let d2 = area2(p1, p2, p4);
            let d3 = area2(p3, p4, p1);
            let d4 = area2(p3, p4, p2);
            ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))
        };
        let mut n_pairs = 0usize;
        for i in 0..tris.len() {
            for j in (i + 1)..tris.len() {
                let ti = tris[i];
                let tj = tris[j];
                // skip if they share an edge or vertex overlap handled elsewhere
                let shared = ti.iter().any(|v| tj.contains(v));
                if shared {
                    continue;
                }
                let pi = [
                    all_2d[ti[0] as usize],
                    all_2d[ti[1] as usize],
                    all_2d[ti[2] as usize],
                ];
                let pj = [
                    all_2d[tj[0] as usize],
                    all_2d[tj[1] as usize],
                    all_2d[tj[2] as usize],
                ];
                let mut hit = false;
                for a in 0..3 {
                    for b in 0..3 {
                        if seg_int(pi[a], pi[(a + 1) % 3], pj[b], pj[(b + 1) % 3]) {
                            hit = true;
                        }
                    }
                }
                if hit {
                    n_pairs += 1;
                    if n_pairs <= 12 {
                        println!(
                            "    OVERLAP tris {} {:?} (areas {:.3e}) x {} {:?} (areas {:.3e})",
                            i,
                            ti,
                            (area2(pi[0], pi[1], pi[2]) * 0.5).abs(),
                            j,
                            tj,
                            (area2(pj[0], pj[1], pj[2]) * 0.5).abs(),
                        );
                    }
                }
            }
        }
        println!("    overlapping disjoint pairs: {}", n_pairs);
    }
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env()
        .init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!("usage: cdt_replay <dump.txt> [...more dumps]");
        std::process::exit(2);
    }
    for path in &args {
        let Some(d) = parse_dump(path) else {
            eprintln!("{}: parse failed / too small", path);
            continue;
        };
        draper_mesh::parametric_domain::set_current_face_label(d.label.clone());
        let n_b = d.boundary.len();
        let n_ring_total: usize = n_b + d.holes.iter().map(|h| h.len()).sum::<usize>();
        let n_i = d.interior.len();

        let t0 = std::time::Instant::now();
        let no_interior = std::env::var("CDT_REPLAY_NO_INTERIOR").is_ok();
        let staged = std::env::var("CDT_REPLAY_STAGED").is_ok();
        let interior_arg: Vec<[f64; 2]> = if no_interior {
            Vec::new()
        } else {
            d.interior.clone()
        };
        let tris = if staged {
            // Stage-by-stage replay of the base pipeline (mirrors
            // triangulate_polygon_cdt's internals) to attribute
            // overlap / nm / rim-miss to the responsible stage.
            let mut all_2d: Vec<[f64; 2]> = d.boundary.clone();
            let mut hole_ranges: Vec<(usize, usize)> = Vec::new();
            for h in &d.holes {
                let s = all_2d.len();
                all_2d.extend_from_slice(h);
                hole_ranges.push((s, all_2d.len()));
            }
            let interior_start = all_2d.len();
            all_2d.extend_from_slice(&interior_arg);
            let mut coords: Vec<f64> = Vec::with_capacity(interior_start * 2);
            for p in &all_2d[..interior_start] {
                coords.push(p[0]);
                coords.push(p[1]);
            }
            let hole_idx: Vec<usize> = hole_ranges.iter().map(|&(s, _)| s).collect();
            let earcut = draper_mesh::earcut_adapter::triangulate_polygon_with_holes(
                &coords, &hole_idx,
            );
            let mut triangles: Vec<[u32; 3]> = earcut
                .chunks(3)
                .filter_map(|c| {
                    if c.len() < 3 {
                        return None;
                    }
                    let (a, b, ch) = (c[0] as u32, c[1] as u32, c[2] as u32);
                    if a == b || b == ch || a == ch {
                        return None;
                    }
                    Some([a, b, ch])
                })
                .collect();
            report_stage("A_earcut", &d, &all_2d, &triangles, n_ring_total);
            if let Ok(dir) = std::env::var("CDT_REPLAY_DUMP_STAGE_TRIS") {
                let _ = std::fs::create_dir_all(&dir);
                let mut out = String::new();
                for t in &triangles {
                    out.push_str(&format!("{} {} {}\n", t[0], t[1], t[2]));
                }
                let _ = std::fs::write(format!("{}/A_earcut.tris", dir), out);
            }
            draper_mesh::custom_cdt::repair_unused_ring_vertices(
                &all_2d,
                &mut triangles,
                d.boundary.len(),
                &hole_ranges,
            );
            report_stage("B_repair", &d, &all_2d, &triangles, n_ring_total);
            draper_mesh::custom_cdt::lawson_flip(
                &all_2d,
                &mut triangles,
                d.boundary.len(),
                &hole_ranges,
            );
            report_stage("C_lawson", &d, &all_2d, &triangles, n_ring_total);
            triangles
        } else {
            draper_mesh::custom_cdt::triangulate_polygon_cdt(
                &d.boundary,
                &d.holes,
                &interior_arg,
            )
        };
        let dt = t0.elapsed();

        // ---- vertex usage ----
        let mut used = vec![false; n_ring_total + n_i];
        for t in &tris {
            for &i in t {
                if (i as usize) < used.len() {
                    used[i as usize] = true;
                }
            }
        }
        let ring_unused = (0..n_ring_total).filter(|&i| !used[i]).count();
        let int_unused = (n_ring_total..n_ring_total + n_i)
            .filter(|&i| !used[i])
            .count();

        // ---- edge census ----
        let mut ecount: HashMap<(u32, u32), usize> = HashMap::new();
        for t in &tris {
            for k in 0..3 {
                let a = t[k];
                let b = t[(k + 1) % 3];
                if a != b {
                    *ecount.entry((a.min(b), a.max(b))).or_default() += 1;
                }
            }
        }
        let mut rim_set = std::collections::HashSet::new();
        let mut rim_edges = 0usize;
        let mut idx = 0usize;
        for ring in std::iter::once(&d.boundary).chain(d.holes.iter()) {
            let n = ring.len();
            for i in 0..n {
                let j = (i + 1) % n;
                rim_set.insert((((idx + i) as u32).min((idx + j) as u32), ((idx + i) as u32).max((idx + j) as u32)));
                if ecount.contains_key(&((idx + i) as u32, (idx + j) as u32)) {
                    rim_edges += 1;
                }
            }
            idx += n;
        }
        let n_rim_total = rim_set.len();
        let extra_bnd = ecount
            .iter()
            .filter(|(e, &n)| n == 1 && !rim_set.contains(e))
            .count();
        let non_manifold = ecount.iter().filter(|(_, &n)| n > 2).count();

        // ---- area / overlap / needle census ----
        let mut all_pts: Vec<[f64; 2]> = d.boundary.clone();
        for h in &d.holes {
            all_pts.extend_from_slice(h);
        }
        all_pts.extend_from_slice(&d.interior);
        let mut neg = 0usize;
        let mut zero = 0usize;
        let mut sum_abs = 0.0f64;
        let mut sum_abs_base = 0.0f64; // triangles using ONLY ring vertices
        let mut n_base = 0usize;
        let mut needles = 0usize;
        let mut areas: Vec<f64> = Vec::with_capacity(tris.len());
        for t in &tris {
            let (a, b, c) = (
                all_pts[t[0] as usize],
                all_pts[t[1] as usize],
                all_pts[t[2] as usize],
            );
            let s = area2(a, b, c);
            if s < 0.0 {
                neg += 1;
            }
            if s == 0.0 {
                zero += 1;
            }
            let abs_half = s.abs() * 0.5;
            areas.push(abs_half);
            sum_abs += abs_half;
            if (t[0] as usize) < n_ring_total
                && (t[1] as usize) < n_ring_total
                && (t[2] as usize) < n_ring_total
            {
                sum_abs_base += abs_half;
                n_base += 1;
            }
            // needle: area < 1e-3 * max_edge^2
            let e = |p: [f64; 2], q: [f64; 2]| {
                let dx = p[0] - q[0];
                let dy = p[1] - q[1];
                dx * dx + dy * dy
            };
            let max_e2 = e(a, b).max(e(b, c)).max(e(c, a));
            if max_e2 > 0.0 && abs_half < 1e-3 * max_e2 {
                needles += 1;
            }
        }
        areas.sort_by(|x, y| x.partial_cmp(y).unwrap());
        // polygon area (outer - holes), shoelace
        let shoelace = |ring: &[[f64; 2]]| -> f64 {
            let mut s = 0.0;
            let n = ring.len();
            for i in 0..n {
                let j = (i + 1) % n;
                s += ring[i][0] * ring[j][1] - ring[j][0] * ring[i][1];
            }
            s * 0.5
        };
        let poly_area = (shoelace(&d.boundary).abs()
            - d.holes.iter().map(|h| shoelace(h).abs()).sum::<f64>())
        .abs();
        let overlap = (sum_abs - poly_area) / poly_area.max(1e-30);

        println!(
            "{}: [{}] bnd={} holes={}/{} int={} tris={} | drops: ring_unused={} int_unused={} | rim {}/{} (miss {}) extra_bnd={} nm={} | neg={} zero={} overlap={:.4} base_overlap={:.4} ({} base tris, base_sum={:.6}) needles={} | area min/med/max = {:.3e}/{:.3e}/{:.3e} | {:.1}ms",
            path,
            d.label,
            n_b,
            d.holes.len(),
            d.holes.iter().map(|h| h.len()).sum::<usize>(),
            n_i,
            tris.len(),
            ring_unused,
            int_unused,
            rim_edges,
            n_rim_total,
            n_rim_total - rim_edges,
            extra_bnd,
            non_manifold,
            neg,
            zero,
            overlap,
            (sum_abs_base - poly_area) / poly_area.max(1e-30),
            n_base,
            sum_abs_base,
            needles,
            areas.first().copied().unwrap_or(0.0),
            if areas.is_empty() { 0.0 } else { areas[areas.len() / 2] },
            areas.last().copied().unwrap_or(0.0),
            dt.as_secs_f64() * 1000.0,
        );
    }
}
