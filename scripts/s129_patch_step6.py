#!/usr/bin/env python3
"""s129: surgical patcher for refine_mesh_chord_error_uv (Step-6).

The rebuild/insertion blocks are DUPLICATED between refine_mesh_chord_error
(non-uv, 17590+) and refine_mesh_chord_error_uv (17886+). We patch ONLY the
_uv variant: for duplicated patterns, replace the SECOND occurrence; unique
patterns are replaced once. Idempotent: refuses to run twice (checks for the
s129 marker).
"""
import sys

PATH = "crates/draper-mesh/src/parametric_domain.rs"
src = open(PATH).read()
if "s129 Guard B: position-dedup the split vertex" in src:
    print("already patched — nothing to do")
    sys.exit(0)


def replace_nth(text, old, new, n=2):
    """Replace the n-th occurrence (1-based) of old with new."""
    pos = -1
    for _ in range(n):
        pos = text.find(old, pos + 1)
        if pos == -1:
            raise SystemExit(f"PATTERN NOT FOUND (occurrence #{n}):\n{old[:120]}")
    return text[:pos] + new + text[pos + len(old):]


def replace_once(text, old, new):
    return replace_nth(text, old, new, 1)


# ── 1. pos_map init + kill-switch (unique anchor: min_uv_dist block) ──
A1_OLD = """    let min_uv_dist = if is_nurbs {
        // For NURBS, use a fraction of the parameter range as the minimum.
        // This prevents infinite subdivision in areas with bad parameterization.
        let u_size = (nurb_u_max - nurb_u_min).max(1e-10);
        let v_size = (nurb_v_max - nurb_v_min).max(1e-10);
        u_size.min(v_size) * 0.01 // 1% of the smaller parameter range
    } else {
        1e-10 // Effectively no minimum for analytic surfaces
    };

    for _iter in 0..max_iterations {"""
A1_NEW = """    let min_uv_dist = if is_nurbs {
        // For NURBS, use a fraction of the parameter range as the minimum.
        // This prevents infinite subdivision in areas with bad parameterization.
        let u_size = (nurb_u_max - nurb_u_min).max(1e-10);
        let v_size = (nurb_v_max - nurb_v_min).max(1e-10);
        u_size.min(v_size) * 0.01 // 1% of the smaller parameter range
    } else {
        1e-10 // Effectively no minimum for analytic surfaces
    };

    // s129: position-dedup for Step-6 midpoints. A split midpoint that
    // lands (at deterministic-rounded resolution) on an EXISTING mesh
    // vertex must REUSE that vertex instead of adding a duplicate:
    // duplicate-position vertices fork the surrounding triangles'
    // shared edges into ONE-SIDED boundary edges (#27977 f7/f65 — the
    // accepted per-face CDT (639 tris, extra_bnd=0, folds 0) shipped as
    // 707 tris with an 11-edge one-sided notch; the notch edges and the
    // FOLD-OVERFAT census pairs are manufactured by Step-6 midpoints
    // hitting existing pinch-zone vertex positions). Kill-switch:
    // DRAPPER_CHORD_REFINE_DEDUP=0 → legacy behavior, bit-exact.
    let refine_dedup_on = std::env::var("DRAPPER_CHORD_REFINE_DEDUP")
        .map(|v| v != "0")
        .unwrap_or(true);
    let mut refine_pos_map: HashMap<[u64; 3], u32> = HashMap::new();
    if refine_dedup_on {
        for (vi, v) in mesh.vertices.iter().enumerate() {
            let rp = deterministic_round_point(*v);
            let key = [rp.x.to_bits(), rp.y.to_bits(), rp.z.to_bits()];
            refine_pos_map.entry(key).or_insert(vi as u32);
        }
    }

    for _iter in 0..max_iterations {"""
src = replace_once(src, A1_OLD, A1_NEW)

# ── 2. Guard A in the collection loop (2nd occurrence = _uv) ──
A2_OLD = """                if chord_error > max_deviation {
                    edges_to_split.insert(edge, u32::MAX); // Placeholder
                }"""
A2_NEW = """                if chord_error > max_deviation {
                    // s129 Guard A: never mark an edge whose split
                    // midpoint would land, at deterministic-rounded
                    // resolution, ON one of its endpoints — the products
                    // would be position-degenerate (pinch-zone class).
                    // Kill-switch DRAPPER_CHORD_REFINE_DEDUP=0 reverts.
                    if refine_dedup_on {
                        let rp = deterministic_round_point(p_surf);
                        let key = [rp.x.to_bits(), rp.y.to_bits(), rp.z.to_bits()];
                        let p0r = deterministic_round_point(p0);
                        let p1r = deterministic_round_point(p1);
                        let k0 = [p0r.x.to_bits(), p0r.y.to_bits(), p0r.z.to_bits()];
                        let k1 = [p1r.x.to_bits(), p1r.y.to_bits(), p1r.z.to_bits()];
                        if key == k0 || key == k1 {
                            continue;
                        }
                    }
                    edges_to_split.insert(edge, u32::MAX); // Placeholder
                }"""
src = replace_nth(src, A2_OLD, A2_NEW, 2)

# ── 3. Guard B: dedup the split vertex (unique insertion block) ──
A3_OLD = """            // For the split vertex, simply evaluate the surface at the averaged UV.
            // We do NOT use Newton-Raphson re-projection here because:
            // 1. UV averaging is already correct for well-parameterized surfaces
            // 2. Newton-Raphson costs ~469 de Boor iterations per edge split
            // 3. The chord error check already verified the averaged UV is reasonable
            // 4. For NURBS with bad parameterization, Newton often doesn't converge
            //    better than simple averaging anyway
            let p_surf = deterministic_round_point(surface.point_at(mid_u, mid_v));
            let n = surface.normal_at(mid_u, mid_v);

            let vi = mesh.add_vertex(p_surf);
            mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);

            // Store UV for the new vertex
            vertex_uvs.push(Point2d::new(mid_u, mid_v));

            // New vertices from chord-error refinement are NOT boundary vertices —
            // they're computed from surface.point_at(), not from the edge cache.
            // Marking them as non-boundary ensures that future refinement iterations
            // can still split edges involving these vertices.
            is_boundary_vertex.push(false);

            new_edges.insert(*edge, vi);
        }"""
A3_NEW = """            // For the split vertex, simply evaluate the surface at the averaged UV.
            // We do NOT use Newton-Raphson re-projection here because:
            // 1. UV averaging is already correct for well-parameterized surfaces
            // 2. Newton-Raphson costs ~469 de Boor iterations per edge split
            // 3. The chord error check already verified the averaged UV is reasonable
            // 4. For NURBS with bad parameterization, Newton often doesn't converge
            //    better than simple averaging anyway
            let p_surf = deterministic_round_point(surface.point_at(mid_u, mid_v));
            let n = surface.normal_at(mid_u, mid_v);

            // s129 Guard B: position-dedup the split vertex. Reuse an
            // existing mesh vertex at the same deterministic-rounded
            // position instead of adding a duplicate (a duplicate forks
            // shared edges into one-sided boundary edges). When reusing,
            // no vertex is added, so vertex_uvs / is_boundary_vertex stay
            // in sync with mesh.vertices (they are only pushed together
            // with add_vertex). Deterministic: lowest existing index
            // wins (map built in index order; split_order is sorted).
            let vi = if refine_dedup_on {
                let key = [p_surf.x.to_bits(), p_surf.y.to_bits(), p_surf.z.to_bits()];
                if let Some(&existing) = refine_pos_map.get(&key) {
                    existing
                } else {
                    let vi = mesh.add_vertex(p_surf);
                    mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                    // Store UV for the new vertex
                    vertex_uvs.push(Point2d::new(mid_u, mid_v));
                    // New vertices from chord-error refinement are NOT
                    // boundary vertices (computed from surface.point_at()).
                    is_boundary_vertex.push(false);
                    refine_pos_map.insert(key, vi);
                    vi
                }
            } else {
                let vi = mesh.add_vertex(p_surf);
                mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                // Store UV for the new vertex
                vertex_uvs.push(Point2d::new(mid_u, mid_v));
                // New vertices from chord-error refinement are NOT
                // boundary vertices (computed from surface.point_at()).
                is_boundary_vertex.push(false);
                vi
            };

            new_edges.insert(*edge, vi);
        }"""
src = replace_once(src, A3_OLD, A3_NEW)

# ── 4. rebuild: vkeys + push_nondeg macro (2nd occurrence = _uv) ──
A4_OLD = """        // Rebuild the triangle list, splitting triangles that have edges marked for subdivision
        let old_triangles = std::mem::take(&mut mesh.triangles);
        mesh.triangles.reserve(old_triangles.len());

        for tri in &old_triangles {"""
A4_NEW = """        // Rebuild the triangle list, splitting triangles that have edges marked for subdivision
        let old_triangles = std::mem::take(&mut mesh.triangles);
        mesh.triangles.reserve(old_triangles.len());

        // s129: position-degeneracy filter for subdivision products —
        // drop a product whose two corners share a deterministic-rounded
        // position (mirrors the Step-5 assembly filter; the deduped split
        // vertex may coincide with the triangle's own corner in pinch
        // zones). Kill-switch DRAPPER_CHORD_REFINE_DEDUP=0 disables.
        let vkeys: Vec<[u64; 3]> = mesh
            .vertices
            .iter()
            .map(|v| {
                let rp = deterministic_round_point(*v);
                [rp.x.to_bits(), rp.y.to_bits(), rp.z.to_bits()]
            })
            .collect();
        macro_rules! push_nondeg {
            ([$a:expr, $b:expr, $c:expr]) => {{
                let t = [$a, $b, $c];
                let bad = refine_dedup_on
                    && (vkeys[t[0] as usize] == vkeys[t[1] as usize]
                        || vkeys[t[1] as usize] == vkeys[t[2] as usize]
                        || vkeys[t[0] as usize] == vkeys[t[2] as usize]);
                if !bad {
                    mesh.triangles.push(t);
                }
            }};
        }

        for tri in &old_triangles {"""
src = replace_nth(src, A4_OLD, A4_NEW, 2)

# ── 5. 1-split pushes → push_nondeg! (2nd occurrence) ──
A5_OLD = """                if forward {
                    mesh.triangles.push([v0, vm, v2]);
                    mesh.triangles.push([vm, v1, v2]);
                } else {
                    mesh.triangles.push([v0, v2, vm]);
                    mesh.triangles.push([vm, v2, v1]);
                }"""
A5_NEW = """                if forward {
                    push_nondeg!([v0, vm, v2]);
                    push_nondeg!([vm, v1, v2]);
                } else {
                    push_nondeg!([v0, v2, vm]);
                    push_nondeg!([vm, v2, v1]);
                }"""
src = replace_nth(src, A5_OLD, A5_NEW, 2)

# ── 6. 2-split pushes → push_nondeg! (2nd occurrence) ──
A6_OLD = """                if (k0 + 1) % 3 == k1 {
                    if forward {
                        mesh.triangles.push([v0, vm0, vm1]);
                        mesh.triangles.push([vm0, v1, vm1]);
                        mesh.triangles.push([v0, vm1, v2]);
                    } else {
                        mesh.triangles.push([v0, vm1, vm0]);
                        mesh.triangles.push([vm0, vm1, v1]);
                        mesh.triangles.push([v0, v2, vm1]);
                    }
                } else {
                    if forward {
                        mesh.triangles.push([v0, vm0, v1]);
                        mesh.triangles.push([v2, vm1, vm0]);
                        mesh.triangles.push([vm0, v0, vm1]);
                    } else {
                        mesh.triangles.push([v0, v1, vm0]);
                        mesh.triangles.push([v2, vm0, vm1]);
                        mesh.triangles.push([vm0, vm1, v0]);
                    }
                }"""
A6_NEW = """                if (k0 + 1) % 3 == k1 {
                    if forward {
                        push_nondeg!([v0, vm0, vm1]);
                        push_nondeg!([vm0, v1, vm1]);
                        push_nondeg!([v0, vm1, v2]);
                    } else {
                        push_nondeg!([v0, vm1, vm0]);
                        push_nondeg!([vm0, vm1, v1]);
                        push_nondeg!([v0, v2, vm1]);
                    }
                } else {
                    if forward {
                        push_nondeg!([v0, vm0, v1]);
                        push_nondeg!([v2, vm1, vm0]);
                        push_nondeg!([vm0, v0, vm1]);
                    } else {
                        push_nondeg!([v0, v1, vm0]);
                        push_nondeg!([v2, vm0, vm1]);
                        push_nondeg!([vm0, vm1, v0]);
                    }
                }"""
src = replace_nth(src, A6_OLD, A6_NEW, 2)

# ── 7. 3-split pushes → push_nondeg! (2nd occurrence) ──
A7_OLD = """                if forward {
                    mesh.triangles.push([v0, vm0, vm2]);
                    mesh.triangles.push([vm0, v1, vm1]);
                    mesh.triangles.push([vm2, vm1, v2]);
                    mesh.triangles.push([vm0, vm1, vm2]);
                } else {
                    mesh.triangles.push([v0, vm2, vm0]);
                    mesh.triangles.push([vm0, vm1, v1]);
                    mesh.triangles.push([vm2, v2, vm1]);
                    mesh.triangles.push([vm0, vm2, vm1]);
                }"""
A7_NEW = """                if forward {
                    push_nondeg!([v0, vm0, vm2]);
                    push_nondeg!([vm0, v1, vm1]);
                    push_nondeg!([vm2, vm1, v2]);
                    push_nondeg!([vm0, vm1, vm2]);
                } else {
                    push_nondeg!([v0, vm2, vm0]);
                    push_nondeg!([vm0, vm1, v1]);
                    push_nondeg!([vm2, v2, vm1]);
                    push_nondeg!([vm0, vm2, vm1]);
                }"""
src = replace_nth(src, A7_OLD, A7_NEW, 2)

open(PATH, "w").write(src)
print("patched OK: 7 replacements in refine_mesh_chord_error_uv")
