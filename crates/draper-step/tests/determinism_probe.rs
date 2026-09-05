// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Determinism probe: convert one STEP file to a mesh and print a stable
//! digest of the result. Run the SAME test twice in two separate `cargo
//! test` invocations (each process gets a different HashMap seed) and
//! compare the printed digests — a mismatch proves cross-process
//! nondeterminism (HashMap iteration order leaking into geometry).
//!
//! Usage:
//!   cargo test -p draper-step --test determinism_probe -- --nocapture
//! (run twice; compare the DIGEST lines)

use draper_step::{parse_step, step_to_mesh, step_to_mesh_instances, extract_solids};

/// FNV-1a over the vertex and triangle buffers — order-sensitive: any
/// reordering of vertices or triangles changes the digest.
fn mesh_digest(mesh: &draper_mesh::TriangleMesh) -> (u64, u64) {
    let mut h: u64 = 0xcbf29ce484222325;
    for v in &mesh.vertices {
        for c in [v.x, v.y, v.z] {
            h ^= c.to_bits();
            h = h.wrapping_mul(0x100000001b3);
        }
    }
    let mut g: u64 = 0xcbf29ce484222325;
    for t in &mesh.triangles {
        for i in [t[0], t[1], t[2]] {
            g ^= i as u64;
            g = g.wrapping_mul(0x100000001b3);
        }
    }
    (h, g)
}

#[test]
fn determinism_probe() {
    for name in [
        "brick_thin_hole.stp",
        "compressor-13920_top.stp",
        "as1-oc-214.stp",
    ] {
        let path = format!("../../test/{}", name);
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(_) => {
                println!("DIGEST {} SKIP (file not found)", name);
                continue;
            }
        };
        let step = match parse_step(&content) {
            Ok(s) => s,
            Err(e) => {
                println!("DIGEST {} PARSE_ERR {}", name, e);
                continue;
            }
        };
        match step_to_mesh(&step) {
            Ok(mesh) => {
                let (hv, ht) = mesh_digest(&mesh);
                println!(
                    "DIGEST {} v={} t={} hv={:016x} ht={:016x}",
                    name,
                    mesh.vertex_count(),
                    mesh.triangle_count(),
                    hv,
                    ht
                );
                assert!(
                    mesh.triangle_count() > 0,
                    "{}: step_to_mesh produced an empty mesh",
                    name
                );
            }
            Err(e) => println!("DIGEST {} MESH_ERR {}", name, e),
        }
        // Solid-level digest (before triangulation): face structure +
        // SORTED edge geometry — detects healing / rebuild_store
        // nondeterminism independent of mesh ordering.
        let (solids, brep_ids) = extract_solids(&step);
        for (si, solid) in solids.iter().enumerate() {
            let mut h: u64 = 0xcbf29ce484222325;
            let faces = solid.faces();
            let mut face_sig: Vec<(usize, usize, bool)> = faces
                .iter()
                .map(|f| {
                    let outer = f.outer_wire.as_ref().map(|w| w.coedges.len()).unwrap_or(0);
                    let inner: usize = f.inner_wires.iter().map(|w| w.coedges.len()).sum();
                    (outer, inner, f.forward)
                })
                .collect();
            face_sig.sort_unstable();
            for (outer, inner, fwd) in face_sig {
                h ^= outer as u64;
                h = h.wrapping_mul(0x100000001b3);
                h ^= inner as u64;
                h = h.wrapping_mul(0x100000001b3);
                h ^= fwd as u64;
                h = h.wrapping_mul(0x100000001b3);
            }
            let mut edge_sig: Vec<[Option<f64>; 8]> = Vec::new();
            for e in solid.edge_store.iter() {
                let pts = [
                    e.start_vertex_point.map(|p| p.x),
                    e.start_vertex_point.map(|p| p.y),
                    e.start_vertex_point.map(|p| p.z),
                    e.end_vertex_point.map(|p| p.x),
                    e.end_vertex_point.map(|p| p.y),
                    e.end_vertex_point.map(|p| p.z),
                    Some(e.param_range.0),
                    Some(e.param_range.1),
                ];
                edge_sig.push(pts);
            }
            edge_sig.sort_unstable_by(|a, b| {
                for (x, y) in a.iter().zip(b) {
                    match (x, y) {
                        (Some(x), Some(y)) => match x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal) {
                            std::cmp::Ordering::Equal => continue,
                            o => return o,
                        },
                        (None, None) => continue,
                        (None, Some(_)) => return std::cmp::Ordering::Less,
                        (Some(_), None) => return std::cmp::Ordering::Greater,
                    }
                }
                std::cmp::Ordering::Equal
            });
            for pts in edge_sig {
                for c in pts {
                    h ^= c.map(|v| v.to_bits()).unwrap_or(0xdead);
                    h = h.wrapping_mul(0x100000001b3);
                }
            }
            println!(
                "DIGEST {} solid[{}] brep={} faces={} edges={} hs={:016x}",
                name,
                si,
                brep_ids.get(si).copied().unwrap_or(-1),
                faces.len(),
                solid.edge_store.len(),
                h
            );
        }
        // Per-instance digests — isolates triangulation nondeterminism
        // (instance digests vary) from merge nondeterminism (instances
        // stable, merged mesh varies).
        if let Ok(instances) = step_to_mesh_instances(&step) {
            for (i, inst) in instances.iter().enumerate() {
                let (hv, ht) = mesh_digest(&inst.mesh);
                println!(
                    "DIGEST {} inst[{}] brep={} v={} t={} hv={:016x} ht={:016x}",
                    name,
                    i,
                    inst.brep_id,
                    inst.mesh.vertex_count(),
                    inst.mesh.triangle_count(),
                    hv,
                    ht
                );
            }
        }
        // Per-face digests from the detailed API — pinpoints exactly
        // which faces are nondeterministic (and their surface types).
        if let Ok(detailed) = draper_step::step_to_detailed_instances(&step) {
            for inst in &detailed {
                let (hv, ht) = mesh_digest(&inst.mesh);
                println!(
                    "DIGEST {} detail brep={} v={} t={} hv={:016x} ht={:016x}",
                    name,
                    inst.brep_id,
                    inst.mesh.vertex_count(),
                    inst.mesh.triangle_count(),
                    hv,
                    ht
                );
                let tris = &inst.mesh.triangles;
                let mut face_sig: Vec<(i64, String, usize, u64)> = Vec::new();
                for f in &inst.faces {
                    let (s, e) = f.triangle_range;
                    let mut g: u64 = 0xcbf29ce484222325;
                    if e > s {
                        for t in &tris[s..e] {
                            for idx in t {
                                g ^= *idx as u64;
                                g = g.wrapping_mul(0x100000001b3);
                            }
                        }
                    }
                    face_sig.push((f.step_face_id, f.surface_type.clone(), e - s, g));
                }
                face_sig.sort_unstable_by(|a, b| a.0.cmp(&b.0));
                for (fid, stype, ntri, g) in face_sig {
                    println!("DIGEST {} face brep={} id={} {} ntri={} g={:016x}", name, inst.brep_id, fid, stype, ntri, g);
                }
                // Boundary discretization lengths — pinpoints edge-cache
                // sampling nondeterminism (per-edge point counts).
                let mut bnd_sig: Vec<(i64, usize, usize)> = inst
                    .faces
                    .iter()
                    .filter(|f| f.step_face_id != 0)
                    .map(|f| {
                        let outer: usize =
                            f.outer_boundary.iter().map(|p| p.len()).sum();
                        let inner: usize =
                            f.inner_boundaries.iter().map(|p| p.len()).sum();
                        (f.step_face_id, outer, inner)
                    })
                    .collect();
                bnd_sig.sort_unstable();
                for (fid, outer, inner) in bnd_sig {
                    println!("DIGEST {} bnd brep={} id={} outer={} inner={}", name, inst.brep_id, fid, outer, inner);
                }
            }
        }
    }
}
