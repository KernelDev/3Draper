// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! s81 diagnostic: replay the earcut adapter chain stage-by-stage on a
//! dumped FAN_GUARD UV ring and report which stage breaks the area
//! contract (2.4e-3 relative area loss on brep62542_f3_Plane m=842).
//!
//! Usage: earcut_replay /tmp/s81_fanring/brep62542_f3_Plane.txt

use std::collections::HashSet;

fn load(path: &str) -> (Vec<(f64, f64)>, usize, f64) {
    let txt = std::fs::read_to_string(path).expect("read ring file");
    let mut pts = Vec::new();
    let mut max_deg = 0usize;
    let mut thin = 0.0f64;
    for line in txt.lines() {
        if line.starts_with("m=") {
            for kv in line.split_whitespace() {
                if let Some(v) = kv.strip_prefix("max_deg=") {
                    max_deg = v.parse().unwrap();
                }
                if let Some(v) = kv.strip_prefix("thinness=") {
                    thin = v.parse().unwrap();
                }
            }
        } else if let Some(rest) = line.strip_prefix("p ") {
            let mut it = rest.split_whitespace();
            let u: f64 = it.next().unwrap().parse().unwrap();
            let v: f64 = it.next().unwrap().parse().unwrap();
            pts.push((u, v));
        }
    }
    (pts, max_deg, thin)
}

fn stats(name: &str, pts: &[(f64, f64)], flat: &[usize]) {
    let m = pts.len();
    let n_tris = flat.len() / 3;
    let mut area2 = 0.0f64;
    let mut n_neg = 0usize;
    let mut n_zero = 0usize;
    for t in flat.chunks_exact(3) {
        let (a, b, c) = (pts[t[0]], pts[t[1]], pts[t[2]]);
        let s = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
        area2 += s;
        if s < 0.0 {
            n_neg += 1;
        }
        if s == 0.0 {
            n_zero += 1;
        }
    }
    let mut ring2 = 0.0f64;
    for i in 0..m {
        let j = (i + 1) % m;
        ring2 += pts[i].0 * pts[j].1 - pts[j].0 * pts[i].1;
    }
    let rel = (area2 - ring2).abs() / ring2.abs().max(1e-30);
    // rim coverage + vertex usage
    let mut edges: HashSet<(usize, usize)> = HashSet::new();
    let mut used = vec![false; m];
    let mut deg = vec![0u32; m];
    for t in flat.chunks_exact(3) {
        for k in 0..3 {
            let a = t[k];
            let b = t[(k + 1) % 3];
            edges.insert((a.min(b), a.max(b)));
            used[a] = true;
            deg[a] += 1;
        }
    }
    let mut missing = 0usize;
    for i in 0..m {
        let a = i;
        let b = (i + 1) % m;
        if !edges.contains(&(a.min(b), a.max(b))) {
            missing += 1;
        }
    }
    let unused = used.iter().filter(|&&u| !u).count();
    let maxdeg = deg.iter().copied().max().unwrap_or(0);
    println!(
        "{:<28} tris={:<5} area_rel={:.3e} rim_missing={} unused={} max_deg={} neg={} zero={}",
        name,
        n_tris,
        rel,
        missing,
        unused,
        maxdeg,
        n_neg,
        n_zero
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("usage: earcut_replay <ring.txt>");
    let (pts, fan_max_deg, thin) = load(path);
    let m = pts.len();
    println!(
        "ring: m={} fan_max_deg={} thinness={:.6}",
        m, fan_max_deg, thin
    );

    let mut coords: Vec<f64> = Vec::with_capacity(m * 2);
    for p in &pts {
        coords.push(p.0);
        coords.push(p.1);
    }

    // Stage A: raw earcutr (via the s74 kill-switch path semantics)
    let raw: Vec<usize> = {
        let hole_indices_vec: Vec<usize> = vec![];
        let coords_vec: Vec<f64> = coords.clone();
        earcutr::earcut(&coords_vec, &hole_indices_vec, 2)
            .into_iter()
            .map(|i| i as usize)
            .collect()
    };
    stats("A raw earcutr", &pts, &raw);

    // Stage B: + repair_collinear_strips
    let repaired = draper_mesh::earcut_adapter::repair_collinear_strips(raw.clone(), &coords);
    stats("B +repair_collinear_strips", &pts, &repaired);

    // Stage C: + flip_zero_area_ears (full chain)
    let flipped = draper_mesh::earcut_adapter::flip_zero_area_ears(repaired.clone(), &coords);
    stats("C +flip_zero_area_ears", &pts, &flipped);

    // Alt 1: earcut int predicates
    if let Some(int_res) = draper_mesh::earcut_adapter::triangulate_with_earcut_int(&coords, &[]) {
        stats("D earcut_int", &pts, &int_res);
    } else {
        println!("D earcut_int                  FAILED (None)");
    }

    // Alt 2: i_triangle fallback
    let it_res =
        draper_mesh::earcut_adapter::triangulate_with_itriangle_fallback(&coords, &[]);
    stats("E i_triangle_fallback", &pts, &it_res);
}
