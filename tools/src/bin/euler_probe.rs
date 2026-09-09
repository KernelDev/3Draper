//! Diagnostic: raw topological Euler analysis of a BREP instance from a STEP
//! file — no triangulation, pure entity-graph walk. Mirrors the §3.2
//! `validate_brep` Check 3 counting (V = unique VERTEX_POINT entity ids,
//! E = unique EDGE_CURVE entity ids, F = ADVANCED_FACE count) and then
//! diagnoses WHY χ is odd or > 2:
//!
//! 1. Vertex duplicate analysis — VERTEX_POINT entities at the same
//!    CARTESIAN_POINT position (tolerance grid) with different ids.
//! 2. Zero-length edges — EDGE_CURVE with coincident start/end vertices.
//! 3. Multi-edge vertex pairs — several EDGE_CURVEs between the same
//!    (unordered) vertex pair (legal: e.g. full circle split in two arcs,
//!    but inflates E).
//! 4. Face multiplicity per edge — boundary (1 face), manifold (2),
//!    non-manifold (>2).
//! 5. χ recomputed with position-deduplicated V and degenerate-edge-pruned E.
//!
//! Usage: euler_probe <file.stp> [brep_id ...]  (no ids = all BREPs)

use draper_step::parser::parse_step;
use draper_step::schema::{StepEntity, StepFile, StepValue};
use std::collections::HashMap;

fn refs_of(e: &StepEntity) -> Vec<i64> {
    let mut out = Vec::new();
    for p in &e.params {
        collect_refs(p, &mut out);
    }
    out
}

fn collect_refs(v: &StepValue, out: &mut Vec<i64>) {
    if let StepValue::Ref(r) = v {
        out.push(*r);
    } else if let StepValue::List(items) = v {
        for it in items {
            collect_refs(it, out);
        }
    }
}

/// Position of a VERTEX_POINT via its CARTESIAN_POINT.
fn vertex_position(step: &StepFile, vp_id: i64) -> Option<[f64; 3]> {
    let vp = step.find_entity(vp_id)?;
    for r in refs_of(vp) {
        if let Some(cp) = step.find_entity(r) {
            if cp.type_name == "CARTESIAN_POINT" {
                let mut xyz = [0.0f64; 3];
                let mut n = 0;
                for p in &cp.params {
                    if let StepValue::List(items) = p {
                        for it in items {
                            if let StepValue::Float(f) = it {
                                if n < 3 {
                                    xyz[n] = *f;
                                    n += 1;
                                }
                            }
                        }
                    }
                }
                if n >= 3 {
                    return Some(xyz);
                }
            }
        }
    }
    None
}

/// BREP → shell(s) → faces. Handles MANIFOLD_SOLID_BREP (single CLOSED_SHELL)
/// and BREP_WITH_VOIDS (outer + void shells).
fn brep_face_ids(step: &StepFile, brep_id: i64) -> (String, Vec<i64>) {
    let brep = step.find_entity(brep_id).expect("brep entity");
    let mut faces = Vec::new();
    for shell_ref in refs_of(brep) {
        if let Some(shell) = step.find_entity(shell_ref) {
            if shell.type_name.contains("SHELL") {
                for f in refs_of(shell) {
                    if let Some(face) = step.find_entity(f) {
                        if face.type_name == "ADVANCED_FACE" {
                            faces.push(f);
                        }
                    }
                }
            }
        }
    }
    (brep.type_name.clone(), faces)
}

/// Face → EDGE_CURVE ids (via FACE_OUTER_BOUND + FACE_BOUND + EDGE_LOOP +
/// ORIENTED_EDGE). Also counts loops per face.
fn face_edge_ids(step: &StepFile, face_id: i64) -> (Vec<i64>, usize) {
    let (edges, n_loops, _) = face_edge_orientations(step, face_id);
    (edges, n_loops)
}

/// Face → (edge ids, #loops, per-usage orientation flags).
/// Also counts VERTEX_LOOPs (loops with no ORIENTED_EDGEs — sphere poles).
fn face_edge_orientations(step: &StepFile, face_id: i64) -> (Vec<i64>, usize, Vec<(i64, bool)>) {
    let face = step.find_entity(face_id).expect("face entity");
    let mut edges = Vec::new();
    let mut n_loops = 0;
    let mut usage: Vec<(i64, bool)> = Vec::new();
    for bound_ref in refs_of(face) {
        if let Some(bound) = step.find_entity(bound_ref) {
            if bound.type_name == "FACE_BOUND" || bound.type_name == "FACE_OUTER_BOUND" {
                n_loops += 1;
                for loop_ref in refs_of(bound) {
                    if let Some(lp) = step.find_entity(loop_ref) {
                        if lp.type_name == "EDGE_LOOP" {
                            for oe_ref in refs_of(lp) {
                                if let Some(oe) = step.find_entity(oe_ref) {
                                    if oe.type_name.contains("ORIENTED_EDGE") {
                                        let mut orient = true;
                                        for p in &oe.params {
                                            if let StepValue::Enum(s) = p {
                                                orient = s == "T";
                                            }
                                        }
                                        for r in refs_of(oe) {
                                            if let Some(ec) = step.find_entity(r) {
                                                if ec.type_name == "EDGE_CURVE" {
                                                    edges.push(r);
                                                    usage.push((r, orient));
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        } else if lp.type_name == "VERTEX_LOOP" {
                            // Loop with no edges — a closed-surface face marker
                            // (sphere pole) or degenerate loop.
                        }
                    }
                }
            }
        }
    }
    (edges, n_loops, usage)
}

/// EDGE_CURVE → (start_vertex, end_vertex) ids.
fn edge_vertices(step: &StepFile, ec_id: i64) -> Option<(i64, i64)> {
    let ec = step.find_entity(ec_id)?;
    let mut vs = Vec::new();
    for p in &ec.params {
        if let StepValue::Ref(r) = p {
            if let Some(v) = step.find_entity(*r) {
                if v.type_name == "VERTEX_POINT" {
                    vs.push(*r);
                }
            }
        }
    }
    if vs.len() >= 2 {
        Some((vs[0], vs[1]))
    } else {
        None
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let file = args
        .iter()
        .find(|s| s.ends_with(".stp") || s.ends_with(".step"))
        .cloned()
        .unwrap_or_else(|| "test/drill_top.stp".to_string());
    let targets: Vec<i64> = args
        .iter()
        .filter(|s| !s.contains('.'))
        .filter_map(|s| s.parse().ok())
        .collect();

    let content = std::fs::read_to_string(&file).expect("read file");
    let step = parse_step(&content).expect("parse STEP");

    // All BREP entities in the file
    let breps: Vec<i64> = step
        .entities
        .iter()
        .filter(|e| e.type_name.contains("MANIFOLD_SOLID_BREP") || e.type_name.contains("BREP_WITH_VOIDS"))
        .map(|e| e.id)
        .filter(|id| targets.is_empty() || targets.contains(id))
        .collect();
    println!("File: {} — {} BREP(s) targeted", file, breps.len());

    for brep_id in breps {
        let (brep_type, faces) = brep_face_ids(&step, brep_id);
        if faces.is_empty() {
            println!("BREP #{} ({}): no faces found — skipped", brep_id, brep_type);
            continue;
        }
        let f = faces.len() as i64;
        // E and V as the validator counts them
        let mut edge_face_count: HashMap<i64, usize> = HashMap::new();
        let mut loops_per_face = Vec::new();
        // edge → Vec<(face_id, orientation)> — one entry per loop usage
        let mut edge_usages: HashMap<i64, Vec<(i64, bool)>> = HashMap::new();
        let mut vertex_loop_faces = Vec::new();
        let mut zero_edge_faces = Vec::new();
        for &face_id in &faces {
            let (edges, n_loops, usage) = face_edge_orientations(&step, face_id);
            loops_per_face.push(n_loops);
            if edges.is_empty() {
                zero_edge_faces.push(face_id);
            }
            for (ec, orient) in &usage {
                *edge_face_count.entry(*ec).or_insert(0) += 1;
                edge_usages.entry(*ec).or_default().push((face_id, *orient));
            }
            // detect VERTEX_LOOP bounds
            if let Some(face) = step.find_entity(face_id) {
                for bound_ref in refs_of(face) {
                    if let Some(bound) = step.find_entity(bound_ref) {
                        if bound.type_name == "FACE_BOUND" || bound.type_name == "FACE_OUTER_BOUND" {
                            for loop_ref in refs_of(bound) {
                                if let Some(lp) = step.find_entity(loop_ref) {
                                    if lp.type_name == "VERTEX_LOOP" {
                                        vertex_loop_faces.push((face_id, loop_ref));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        let e_count = edge_face_count.len() as i64;

        let mut vertex_ids: std::collections::HashSet<i64> = std::collections::HashSet::new();
        let mut edges_no_two = 0usize;
        let mut edge_vpair: HashMap<i64, (i64, i64)> = HashMap::new();
        for &ec_id in edge_face_count.keys() {
            match edge_vertices(&step, ec_id) {
                Some((a, b)) => {
                    vertex_ids.insert(a);
                    vertex_ids.insert(b);
                    edge_vpair.insert(ec_id, (a, b));
                }
                None => edges_no_two += 1,
            }
        }
        let v_count = vertex_ids.len() as i64;
        let chi = v_count - e_count + f;

        println!("\n============================================================");
        println!("BREP #{} ({}) — F={}, E={}, V={} → χ_naive={}", brep_id, brep_type, f, e_count, v_count, chi);
        println!("loops/face: min={}, max={}, faces with >1 loop={}",
            loops_per_face.iter().min().unwrap_or(&0),
            loops_per_face.iter().max().unwrap_or(&0),
            loops_per_face.iter().filter(|&&l| l > 1).count());
        // ── Correct BREP Euler: χ = V - E + F - H, H = inner loops ──
        // A face with k loops is a disk with (k-1) holes: χ(face) = 1-(k-1)
        // = 2-k, not 1. Summing: χ = V - E + Σ(2 - k_f) = V - E + 2F - L,
        // where L = total loops. Equivalently χ = V - E + F - H, H = L - F.
        {
            let l_total: i64 = loops_per_face.iter().map(|&l| l as i64).sum::<i64>()
                + vertex_loop_faces.len() as i64;
            let h = l_total - f;
            let chi_true = v_count - e_count + f - h;
            println!("L(total loops)={}, H(inner loops)={} → χ_correct = {}-{}+{}-{} = {}",
                l_total, h, v_count, e_count, f, h, chi_true);
            if chi % 2 != 0 && chi_true % 2 == 0 {
                println!("    ✓ odd χ_naive fully explained by uncounted inner loops ({} holes)", h);
            }
            if chi_true % 2 != 0 {
                println!("    ⚠ χ_correct STILL odd — genuine inconsistency");
            }
        }
        if !vertex_loop_faces.is_empty() {
            println!("VERTEX_LOOP faces (closed-surface / pole loops): {} — {:?}",
                vertex_loop_faces.len(), vertex_loop_faces.iter().take(5).collect::<Vec<_>>());
        }
        if !zero_edge_faces.is_empty() {
            println!("faces with ZERO edge usages (self-closed surfaces): {} — {:?}",
                zero_edge_faces.len(), zero_edge_faces.iter().take(5).collect::<Vec<_>>());
        }
        let bnd = edge_face_count.values().filter(|&&c| c == 1).count();
        let nm = edge_face_count.values().filter(|&&c| c > 2).count();
        println!("edge multiplicity: 1-face(boundary)={}, 2-face={}, >2-face(non-manifold)={}",
            bnd, edge_face_count.values().filter(|&&c| c == 2).count(), nm);
        if edges_no_two > 0 {
            println!("edges lacking 2 vertices: {}", edges_no_two);
        }

        // ── Diagnostic 0: per-edge usage orientation analysis ──
        let mut seam_same_face_opposite = 0usize;
        let mut bowtie_same_face_same = 0usize;
        let mut shared_diff_faces_opposite = 0usize;
        let mut bowtie_diff_faces_same = 0usize;
        let mut other_usage = 0usize;
        for (ec, usages) in &edge_usages {
            if usages.len() == 2 {
                let (f1, o1) = usages[0];
                let (f2, o2) = usages[1];
                if f1 == f2 {
                    if o1 != o2 {
                        seam_same_face_opposite += 1;
                    } else {
                        bowtie_same_face_same += 1;
                        if bowtie_same_face_same <= 5 {
                            println!("    ⚠ bowtie edge #{}: face #{} traverses TWICE in the SAME orientation", ec, f1);
                        }
                    }
                } else {
                    if o1 != o2 {
                        shared_diff_faces_opposite += 1;
                    } else {
                        bowtie_diff_faces_same += 1;
                        if bowtie_diff_faces_same <= 8 {
                            println!("    ⚠ suspect edge #{}: faces #{} #{} both orientation={}", ec, f1, f2, o1);
                        }
                    }
                }
            } else {
                other_usage += 1;
            }
        }
        println!("[0] edge usage orientation: seam(same face, opposite)={}, shared(2 faces, opposite)={}, \
            bowtie(same face, same)={}, suspect(2 faces, same)={}, other(count≠2)={}",
            seam_same_face_opposite, shared_diff_faces_opposite,
            bowtie_same_face_same, bowtie_diff_faces_same, other_usage);

        // ── Diagnostic 0b: connected components via shared edges ──
        // Union faces that share an edge (different faces). Seam-only faces
        // are separate components unless joined via vertices.
        {
            let mut parent: HashMap<i64, i64> = HashMap::new();
            fn find(parent: &mut HashMap<i64, i64>, x: i64) -> i64 {
                let p = *parent.get(&x).unwrap_or(&x);
                if p == x {
                    x
                } else {
                    let r = find(parent, p);
                    parent.insert(x, r);
                    r
                }
            }
            for face_id in &faces {
                parent.entry(*face_id).or_insert(*face_id);
            }
            for usages in edge_usages.values() {
                let distinct: Vec<i64> = {
                    let mut d: Vec<i64> = usages.iter().map(|(f, _)| *f).collect();
                    d.sort();
                    d.dedup();
                    d
                };
                for i in 1..distinct.len() {
                    let a = find(&mut parent, distinct[0]);
                    let b = find(&mut parent, distinct[i]);
                    if a != b {
                        parent.insert(a, b);
                    }
                }
            }
            // Also union faces sharing a vertex (via edge endpoints)
            let mut vertex_faces: HashMap<i64, Vec<i64>> = HashMap::new();
            for (ec, usages) in &edge_usages {
                if let Some(&(a, _b)) = edge_vpair.get(ec) {
                    for (face, _) in usages {
                        vertex_faces.entry(a).or_default().push(*face);
                    }
                }
            }
            for fs in vertex_faces.values() {
                let distinct: Vec<i64> = {
                    let mut d = fs.clone();
                    d.sort();
                    d.dedup();
                    d
                };
                for i in 1..distinct.len() {
                    let ra = find(&mut parent, distinct[0]);
                    let rb = find(&mut parent, distinct[i]);
                    if ra != rb {
                        parent.insert(ra, rb);
                    }
                }
            }
            let mut components: HashMap<i64, Vec<i64>> = HashMap::new();
            for face_id in &faces {
                let r = find(&mut parent, *face_id);
                components.entry(r).or_default().push(*face_id);
            }
            println!("[0b] connected components (faces): {}", components.len());
            // Per-component V, E, F, χ
            for (root, comp_faces) in components.iter().take(6) {
                let comp_set: std::collections::HashSet<i64> = comp_faces.iter().copied().collect();
                let mut c_v: std::collections::HashSet<i64> = std::collections::HashSet::new();
                let mut c_e: std::collections::HashSet<i64> = std::collections::HashSet::new();
                for (ec, usages) in &edge_usages {
                    if usages.iter().any(|(fc, _)| comp_set.contains(fc)) {
                        c_e.insert(*ec);
                        if let Some(&(a, b)) = edge_vpair.get(ec) {
                            c_v.insert(a);
                            c_v.insert(b);
                        }
                    }
                }
                let cchi = c_v.len() as i64 - c_e.len() as i64 + comp_faces.len() as i64;
                println!("    component root #{}: F={}, E={}, V={} → χ={}",
                    root, comp_faces.len(), c_e.len(), c_v.len(), cchi);
            }
        }

        // ── Diagnostic 1: duplicate vertex positions ──
        let mut pos_groups: HashMap<(i64, i64, i64), Vec<i64>> = HashMap::new();
        let mut no_pos = 0usize;
        for &vp in &vertex_ids {
            match vertex_position(&step, vp) {
                Some([x, y, z]) => {
                    // 1e-6 mm grid — same position ⇒ same key
                    let key = (
                        (x * 1e6).round() as i64,
                        (y * 1e6).round() as i64,
                        (z * 1e6).round() as i64,
                    );
                    pos_groups.entry(key).or_default().push(vp);
                }
                None => no_pos += 1,
            }
        }
        let dup_groups: Vec<&Vec<i64>> = pos_groups.values().filter(|g| g.len() > 1).collect();
        let unique_positions = pos_groups.len();
        println!("\n[1] vertex positions: {} entities → {} unique positions; {} duplicate groups, {} unresolvable",
            vertex_ids.len(), unique_positions, dup_groups.len(), no_pos);
        for (i, g) in dup_groups.iter().take(10).enumerate() {
            let [x, y, z] = vertex_position(&step, g[0]).unwrap();
            println!("    dup group {}: pos=({:.4}, {:.4}, {:.4}) ids={:?}", i, x, y, z, g.iter().take(8).collect::<Vec<_>>());
        }
        let v_eff = unique_positions as i64;

        // ── Diagnostic 2: zero-length edges ──
        let mut zero_len = 0usize;
        for (&ec_id, &(a, b)) in &edge_vpair {
            if a == b {
                zero_len += 1;
                if zero_len <= 5 {
                    println!("    zero-length edge #{} (same vertex #{} twice)", ec_id, a);
                }
                continue;
            }
            if let (Some(pa), Some(pb)) = (vertex_position(&step, a), vertex_position(&step, b)) {
                let d = ((pa[0]-pb[0]).powi(2) + (pa[1]-pb[1]).powi(2) + (pa[2]-pb[2]).powi(2)).sqrt();
                if d < 1e-6 {
                    zero_len += 1;
                }
            }
        }
        println!("[2] zero-length edges (same or coincident endpoints): {}", zero_len);

        // ── Diagnostic 3: multi-edge vertex pairs ──
        let mut pair_edges: HashMap<(i64, i64), Vec<i64>> = HashMap::new();
        for (&ec_id, &(a, b)) in &edge_vpair {
            let key = (a.min(b), a.max(b));
            pair_edges.entry(key).or_default().push(ec_id);
        }
        let multi: Vec<(&(i64, i64), &Vec<i64>)> = pair_edges.iter().filter(|(_, v)| v.len() > 1).collect();
        println!("[3] vertex pairs carrying >1 EDGE_CURVE: {} (pairs with 2 = full-circle split; >2 suspicious)",
            multi.len());
        for (k, v) in multi.iter().take(8) {
            if v.len() > 2 {
                println!("    suspicious: vertices {:?} share {} edges {:?}", k, v.len(), v);
            }
        }

        // ── Diagnostic 4: vertex incidence (isolated / dangling) ──
        let mut vertex_edge_inc: HashMap<i64, usize> = HashMap::new();
        for &(a, b) in edge_vpair.values() {
            *vertex_edge_inc.entry(a).or_insert(0) += 1;
            *vertex_edge_inc.entry(b).or_insert(0) += 1;
        }
        let d1 = vertex_edge_inc.values().filter(|&&c| c == 1).count();
        let d3p = vertex_edge_inc.values().filter(|&&c| c >= 3).count();
        println!("[4] vertex edge-incidence: degree-1(dangling)={}, degree-2={}, degree>=3={}",
            d1, vertex_edge_inc.values().filter(|&&c| c == 2).count(), d3p);

        // ── Diagnostic 5: χ after position-dedupe and degenerate pruning ──
        let e_eff = e_count - zero_len as i64;
        let chi_dedup = v_eff - e_count + f;
        let chi_both = v_eff - e_eff + f;
        println!("[5] χ recomputed: pos-dedup V={} → χ'={}; + zero-len-edge prune E={} → χ''={}",
            v_eff, chi_dedup, e_eff, chi_both);
        if chi % 2 != 0 {
            if chi_dedup % 2 != 0 && chi_both % 2 != 0 {
                println!("    ⚠ odd χ persists after dedupe — topology genuinely inconsistent (missing faces?)");
            } else {
                println!("    ✓ odd χ explained by duplicate vertex entities (counting artifact)");
            }
        }

        // ── Diagnostic 6: loop vertex consistency + vertex link (pinch) analysis ──
        // Walk each face's loops in traversed order; consecutive edges must
        // share the same VERTEX_POINT entity. Then build per-vertex link
        // graphs: nodes = (edge, face) usages at v; connections = loop
        // transitions at v. A manifold vertex has ONE link cycle; ≥2 cycles
        // = pinched (non-manifold) vertex — each pinch flips χ parity.
        {
            // loops: face → Vec<Vec<(ec, start_v, end_v)>> in traversal order
            let mut face_loops: HashMap<i64, Vec<Vec<(i64, i64, i64)>>> = HashMap::new();
            let mut loop_breaks: Vec<(i64, i64, f64)> = Vec::new(); // (face, ec_break, gap mm)
            for &face_id in &faces {
                let face = step.find_entity(face_id).expect("face entity");
                let mut loops = Vec::new();
                for bound_ref in refs_of(face) {
                    if let Some(bound) = step.find_entity(bound_ref) {
                        if bound.type_name == "FACE_BOUND" || bound.type_name == "FACE_OUTER_BOUND" {
                            for loop_ref in refs_of(bound) {
                                if let Some(lp) = step.find_entity(loop_ref) {
                                    if lp.type_name == "EDGE_LOOP" {
                                        let mut loop_path: Vec<(i64, i64, i64)> = Vec::new();
                                        for oe_ref in refs_of(lp) {
                                            if let Some(oe) = step.find_entity(oe_ref) {
                                                if oe.type_name.contains("ORIENTED_EDGE") {
                                                    let mut orient = true;
                                                    for p in &oe.params {
                                                        if let StepValue::Enum(s) = p {
                                                            orient = s == "T";
                                                        }
                                                    }
                                                    for r in refs_of(oe) {
                                                        if let Some(ec) = step.find_entity(r) {
                                                            if ec.type_name == "EDGE_CURVE" {
                                                                if let Some((a, b)) = edge_vertices(&step, r) {
                                                                    let (s, e) = if orient { (a, b) } else { (b, a) };
                                                                    loop_path.push((r, s, e));
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        // check consecutive vertex continuity
                                        for i in 0..loop_path.len() {
                                            let (_, _, end_i) = loop_path[i];
                                            let next_i = (i + 1) % loop_path.len();
                                            let (_, start_next, _) = loop_path[next_i];
                                            if end_i != start_next {
                                                let pa = vertex_position(&step, end_i);
                                                let pb = vertex_position(&step, start_next);
                                                let dist = match (pa, pb) {
                                                    (Some(a), Some(b)) => ((a[0]-b[0]).powi(2) + (a[1]-b[1]).powi(2) + (a[2]-b[2]).powi(2)).sqrt(),
                                                    _ => -1.0,
                                                };
                                                loop_breaks.push((face_id, loop_path[i].0, dist));
                                            }
                                        }
                                        loops.push(loop_path);
                                    }
                                }
                            }
                        }
                    }
                }
                face_loops.insert(face_id, loops);
            }
            println!("[6] loop vertex continuity: {} breaks (consecutive edges with different VERTEX_POINT entities)", loop_breaks.len());
            for (fc, ec, d) in loop_breaks.iter().take(8) {
                println!("    break: face #{} after edge #{} — next-start vertex differs (gap {:.6} mm)", fc, ec, d);
            }

            // Link analysis per vertex.
            // The link of vertex v: nodes = incident EDGE_CURVEs at v;
            // link edges = face corners at v (in face f's loop, consecutive
            // edges e_a → e_b at v contribute link edge e_a—e_b).
            // Manifold vertex ⟔ link is a single cycle; k≥2 components =
            // pinched vertex (k shells meeting at v) — flips χ parity.
            let mut v_incident: HashMap<i64, Vec<i64>> = HashMap::new(); // vertex → incident edges
            let mut link_edges: Vec<(i64, i64)> = Vec::new(); // (ec_a, ec_b) corners
            for (&face_id, loops) in &face_loops {
                for lp in loops {
                    let n = lp.len();
                    for i in 0..n {
                        let (ec_a, _, end_a) = lp[i];
                        let next = (i + 1) % n;
                        let (ec_b, start_b, _) = lp[next];
                        let v = end_a;
                        v_incident.entry(v).or_default().push(ec_a);
                        if v == start_b {
                            if ec_a != ec_b {
                                link_edges.push((ec_a, ec_b));
                            }
                        }
                    }
                }
            }
            // union-find over edge ids
            let mut ec_ids: Vec<i64> = edge_usages.keys().copied().collect();
            ec_ids.sort();
            let mut parent: HashMap<i64, i64> = HashMap::new();
            for e in &ec_ids {
                parent.insert(*e, *e);
            }
            fn uf_find(p: &mut HashMap<i64, i64>, x: i64) -> i64 {
                let px = *p.get(&x).unwrap();
                if px == x {
                    x
                } else {
                    let r = uf_find(p, px);
                    p.insert(x, r);
                    r
                }
            }
            for &(a, b) in &link_edges {
                let ra = uf_find(&mut parent, a);
                let rb = uf_find(&mut parent, b);
                if ra != rb {
                    parent.insert(ra, rb);
                }
            }
            // per-vertex: count distinct link components among incident edges
            let mut pinched: Vec<(i64, usize)> = Vec::new();
            let mut link_stats: HashMap<usize, usize> = HashMap::new();
            for (v, edges) in &v_incident {
                let mut roots: Vec<i64> = edges.iter().map(|e| uf_find(&mut parent, *e)).collect();
                roots.sort();
                roots.dedup();
                *link_stats.entry(roots.len()).or_default() += 1;
                if roots.len() >= 2 {
                    pinched.push((*v, roots.len()));
                }
            }
            let mut sorted_pinch = pinched.clone();
            sorted_pinch.sort_by_key(|(_, c)| std::cmp::Reverse(*c));
            println!("[7] vertex link components: {:?}", {
                let mut ks: Vec<_> = link_stats.iter().collect();
                ks.sort();
                ks.iter().map(|(k, c)| format!("{}comp×{}", k, c)).collect::<Vec<_>>()
            });
            println!("    pinched vertices (link ≥2 components): {} — these flip χ parity", sorted_pinch.len());
            for (v, c) in sorted_pinch.iter().take(12) {
                if let Some(pos) = vertex_position(&step, *v) {
                    println!("    pinch: vertex #{} at ({:.3}, {:.3}, {:.3}) — {} link components", v, pos[0], pos[1], pos[2], c);
                }
            }
            if chi % 2 != 0 && !sorted_pinch.is_empty() {
                println!("    → odd χ = {} EXPLAINED by {} pinched vertices (parity: {} mod 2 = {})",
                    chi, sorted_pinch.len(), sorted_pinch.len(), sorted_pinch.len() % 2);
            }
        }
    }
}
