// Diagnostic: chunked vs cached (non-chunked) converter path parity check.
//
// The two BREP triangulation paths in the STEP converter are manually
// mirrored code (`BrepSession::process_one_face`/`finalize` vs the inline
// face loop + post-loop of `triangulate_brep_detailed`). Any drift between
// the mirrors makes the SAME file produce DIFFERENT meshes depending on
// which path runs (WASM progressive viewer = chunked, native tools =
// cached) — the "6405 vs 6757" HOUSING boundary discrepancy (worklog
// session-29+, top of the canonical-CDT «Осталось» since session-37).
//
// This tool runs BOTH paths on native with default params (keep_ratio=1.0
// → decimation no-op) and diffs per-BREP:
//   - vertex/triangle counts
//   - boundary / non-manifold edge counts
//   - an order-sensitive FNV digest of vertices + triangle indices
//   - an order-INsensitive digest (sorted bit patterns) to separate
//     "same geometry, different vertex order" from "different geometry"
//
// Usage: chunked_cached_diff <file.stp> [more.stp ...]

use draper_mesh::{TriangulationParams, check_manifold};
use draper_step::{
    OwnedStepConversionContext, TriangulatePendingResult, parse_step, step_structure_lazy,
};

// Modes via env vars (both paths always run with the SAME mode):
//   ADAPTIVE=1  — adaptive_lod_enabled (per-face budgets; the chunked path
//                 historically never applied them — parity regression guard)
//   CANONICAL=1  — use_surface_canonical_cdt (the setup-order change moves
//                 seam aliasing BEFORE the canonical pre-pass on the cached
//                 path — parity + baseline-shift guard)
fn mode_params() -> TriangulationParams {
    let mut params = TriangulationParams::default();
    if std::env::var("ADAPTIVE").is_ok() {
        params.adaptive_lod_enabled = true;
    }
    if std::env::var("CANONICAL").is_ok() {
        params.use_surface_canonical_cdt = true;
    }
    params
}

struct Row {
    name: String,
    brep_id: i64,
    verts: usize,
    tris: usize,
    bnd: usize,
    nm: usize,
    digest: u64,
    sorted_digest: u64,
}

fn fnv_step(h: u64, v: u64) -> u64 {
    (h ^ v).wrapping_mul(0x100000001b3)
}

fn mesh_digest(mesh: &draper_mesh::TriangleMesh) -> (u64, u64) {
    // Order-sensitive: FNV-1a over vertex coords + triangle indices, in order.
    let mut h: u64 = 0xcbf29ce484222325;
    for v in &mesh.vertices {
        h = fnv_step(h, v.x.to_bits() as u64);
        h = fnv_step(h, v.y.to_bits() as u64);
        h = fnv_step(h, v.z.to_bits() as u64);
    }
    for t in &mesh.triangles {
        for i in 0..3 {
            h = fnv_step(h, t[i] as u64);
        }
    }
    // Order-insensitive: XOR-fold of per-vertex/per-triangle hashes (commutative).
    let mut s: u64 = 0x9e3779b97f4a7c15;
    for v in &mesh.vertices {
        let mut vh: u64 = 0xcbf29ce484222325;
        vh = fnv_step(vh, v.x.to_bits() as u64);
        vh = fnv_step(vh, v.y.to_bits() as u64);
        vh = fnv_step(vh, v.z.to_bits() as u64);
        s ^= vh.rotate_left((s >> 57) as u32);
        s = s.wrapping_mul(0x100000001b3);
    }
    for t in &mesh.triangles {
        let mut th: u64 = 0xcbf29ce484222325;
        for i in 0..3 {
            th = fnv_step(th, t[i] as u64);
        }
        s ^= th.rotate_left((s >> 57) as u32);
        s = s.wrapping_mul(0x100000001b3);
    }
    (h, s)
}

fn row_of(name: &str, brep_id: i64, mesh: &draper_mesh::TriangleMesh) -> Row {
    let report = check_manifold(mesh);
    let (digest, sorted_digest) = mesh_digest(mesh);
    Row {
        name: name.to_string(),
        brep_id,
        verts: mesh.vertex_count(),
        tris: mesh.triangle_count(),
        bnd: report.boundary_edge_count,
        nm: report.non_manifold_edge_count,
        digest,
        sorted_digest,
    }
}

/// Path A: cached / non-chunked (`triangulate_pending` →
/// `triangulate_brep_detailed_gated`, with the §1.2 watertight retry).
fn run_cached(content: &str) -> Vec<Row> {
    let step_file = match parse_step(content) {
        Ok(f) => f,
        Err(e) => {
            println!("  ERROR parsing: {}", e);
            return Vec::new();
        }
    };
    let (_tree, pending) = step_structure_lazy(&step_file);
    let params = mode_params();
    let mut ctx = OwnedStepConversionContext::new_with_params(step_file, params);
    let mut out = Vec::new();
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            out.push(row_of(&p.name, p.brep_id, &inst.mesh));
        }
    }
    out
}

/// Path B: chunked / progressive (`triangulate_pending_chunked` →
/// `prepare_brep_session` + `BrepSession`, no retry).
fn run_chunked(content: &str) -> Vec<Row> {
    let step_file = match parse_step(content) {
        Ok(f) => f,
        Err(e) => {
            println!("  ERROR parsing: {}", e);
            return Vec::new();
        }
    };
    let (_tree, pending) = step_structure_lazy(&step_file);
    let params = mode_params();
    let mut ctx = OwnedStepConversionContext::new_with_params(step_file, params);
    let mut out = Vec::new();
    for p in &pending {
        // Big chunk budget: process the whole BREP in one "frame". Native
        // default time limits are Duration::MAX (no face skips) — any diff
        // vs the cached path is pure algorithmic divergence, not timeouts.
        loop {
            match ctx.triangulate_pending_chunked(p, std::time::Duration::from_secs(600)) {
                TriangulatePendingResult::Done(inst) => {
                    if let Some(inst) = inst {
                        out.push(row_of(&p.name, p.brep_id, &inst.mesh));
                    }
                    break;
                }
                TriangulatePendingResult::InProgress { .. } => continue,
            }
        }
    }
    out
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("error")).init();
    println!(
        "mode: adaptive={} canonical={}",
        std::env::var("ADAPTIVE").is_ok(),
        std::env::var("CANONICAL").is_ok()
    );
    let files: Vec<String> = std::env::args().skip(1).collect();
    let files = if files.is_empty() {
        vec![
            "test/drill_top.stp".to_string(),
            "test/as1-oc-214.stp".to_string(),
        ]
    } else {
        files
    };

    let mut total_diffs = 0usize;
    for f in &files {
        let content = match std::fs::read_to_string(f) {
            Ok(c) => c,
            Err(e) => {
                println!("ERROR reading {}: {}", f, e);
                continue;
            }
        };
        println!("== {} ==", f);
        let cached = run_cached(&content);
        let chunked = run_chunked(&content);

        println!(
            "{:<34} {:>10} {:>10} {:>7} {:>7} | {:>10} {:>10} {:>7} {:>7} | {}",
            "BREP", "v-cache", "t-cache", "b-c", "nm-c", "v-chunk", "t-chunk", "b-h", "nm-h", "VERDICT"
        );
        println!("{}", "-".repeat(130));
        for (i, c) in cached.iter().enumerate() {
            let h = chunked.get(i);
            let (verdict, hv, ht, hb, hnm) = match h {
                Some(h) => {
                    let geo_same = c.digest == h.digest;
                    let set_same = c.sorted_digest == h.sorted_digest;
                    let counts_same =
                        c.verts == h.verts && c.tris == h.tris && c.bnd == h.bnd && c.nm == h.nm;
                    let v = if geo_same && counts_same {
                        "IDENTICAL"
                    } else if set_same && counts_same {
                        "ORDER-ONLY"
                    } else if counts_same {
                        "CONTENT-DIFF"
                    } else {
                        "DIVERGENT"
                    };
                    (v.to_string(), h.verts, h.tris, h.bnd, h.nm)
                }
                None => ("MISSING".to_string(), 0, 0, 0, 0),
            };
            if verdict != "IDENTICAL" {
                total_diffs += 1;
            }
            println!(
                "{:<32} #{:<9} {:>10} {:>10} {:>7} {:>7} | {:>10} {:>10} {:>7} {:>7} | {}",
                c.name, c.brep_id, c.verts, c.tris, c.bnd, c.nm, hv, ht, hb, hnm, verdict
            );
        }
        if chunked.len() > cached.len() {
            for h in &chunked[cached.len()..] {
                total_diffs += 1;
                println!(
                    "{:<32} #{:<9} {:>10} {:>10} {:>7} {:>7} | {:>10} {:>10} {:>7} {:>7} | EXTRA",
                    h.name, h.brep_id, 0, 0, 0, 0, h.verts, h.tris, h.bnd, h.nm
                );
            }
        }
    }
    println!();
    if total_diffs == 0 {
        println!("RESULT: all BREPs bit-identical between chunked and cached paths.");
    } else {
        println!("RESULT: {} BREP(s) diverge between chunked and cached paths.", total_diffs);
    }
}
