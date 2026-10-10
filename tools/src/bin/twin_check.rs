//! session-92 diagnostic: verify the COINCIDENT-twin hypothesis —
//! same-face triangle pairs whose shared interior edge runs in the
//! SAME direction through both triangles (a WINDING-FLIP census
//! candidate) and whose third vertices are (near-)coincident after
//! the weld. Such pairs are positional AREA DUPLICATES ("twins"):
//! removing one loses no coverage.
//!
//! For every same-face one-directional shared-edge pair, dump:
//!   - the two triangles (indices + vertex positions)
//!   - the distance between the two third vertices
//!   - whether the two vertex SETS are bit-identical positionally
//!   - areas and the min-height over the shared base
//! Read-only diagnostic; no mesh mutation.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::TriangleMesh;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env()
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args
        .iter()
        .skip(1)
        .find(|a| !a.starts_with('-'))
        .cloned()
        .unwrap_or_else(|| "test/transmission_top.stp".to_string());

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    let mut n_breps = 0usize;
    let mut total_pairs = 0usize;
    let mut twins_bit = 0usize;
    let mut twins_eps = 0usize; // third-vertex distance < 1e-9
    let mut twins_subtol = 0usize; // < 0.05 (typical res_tol scale)

    for p in pending.iter() {
        let Some(inst) = ctx.triangulate_pending(p) else {
            continue;
        };
        n_breps += 1;
        let mesh = &inst.mesh;
        let Some(fids) = mesh.triangle_face_ids.as_ref() else {
            continue;
        };
        let stats = scan_brep(mesh, fids, p.brep_id);
        total_pairs += stats.0;
        twins_bit += stats.1;
        twins_eps += stats.2;
        twins_subtol += stats.3;
    }

    println!(
        "\nTWIN-CHECK SUMMARY: {} breps, {} same-face one-directional pairs; \
         third-vertex coincident: bit={} eps(<1e-9)={} subtol(<0.05)={}",
        n_breps, total_pairs, twins_bit, twins_eps, twins_subtol
    );
}

fn edge_owners(mesh: &TriangleMesh, key: (u32, u32)) -> Vec<usize> {
    use std::collections::HashMap;
    let mut map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let (a, b, c) = (tri[0], tri[1], tri[2]);
        for (v0, v1) in [(a, b), (b, c), (c, a)] {
            let k = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            map.entry(k).or_default().push(ti);
        }
    }
    let k = if key.0 < key.1 { key } else { (key.1, key.0) };
    map.get(&k).cloned().unwrap_or_default()
}

fn scan_brep(mesh: &TriangleMesh, fids: &[u64], brep_id: i64) -> (usize, usize, usize, usize) {
    use std::collections::HashMap;
    let nv = mesh.vertices.len();
    let n_tris = mesh.triangles.len();
    let mut edge_map: HashMap<(u32, u32), Vec<(usize, u32, u32)>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let (a, b, c) = (tri[0], tri[1], tri[2]);
        for (v0, v1) in [(a, b), (b, c), (c, a)] {
            let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            edge_map.entry(key).or_default().push((ti, v0, v1));
        }
    }
    let mut edges: Vec<(&(u32, u32), &Vec<(usize, u32, u32)>)> = edge_map.iter().collect();
    edges.sort_unstable_by_key(|(e, _)| **e);

    let mut total = 0usize;
    let mut bit = 0usize;
    let mut eps = 0usize;
    let mut subtol = 0usize;
    let mut reported = 0usize;

    let dist = |a: u32, b: u32| -> f64 {
        let pa = &mesh.vertices[a as usize];
        let pb = &mesh.vertices[b as usize];
        (pa.x - pb.x).hypot((pa.y - pb.y)).hypot(pa.z - pb.z)
    };
    let area_of = |t: &[u32; 3]| -> f64 {
        let pa = &mesh.vertices[t[0] as usize];
        let pb = &mesh.vertices[t[1] as usize];
        let pc = &mesh.vertices[t[2] as usize];
        let e1 = [pb.x - pa.x, pb.y - pa.y, pb.z - pa.z];
        let e2 = [pc.x - pa.x, pc.y - pa.y, pc.z - pa.z];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
    };

    for (_edge, owners) in &edges {
        if owners.len() != 2 {
            continue;
        }
        let (t0, a0, b0) = owners[0];
        let (t1, a1, b1) = owners[1];
        if t0 >= n_tris || t1 >= n_tris {
            continue;
        }
        // same face?
        if fids.get(t0).copied().unwrap_or(u64::MAX)
            != fids.get(t1).copied().unwrap_or(u64::MAX - 1)
        {
            continue;
        }
        // one-directional (WINDING-FLIP topology)?
        if (a0, b0) != (a1, b1) {
            continue;
        }
        // extra forensics: how many triangles own this edge (non-manifold?),
        // and each triangle's winding sign relative to the face plane.
        let n_owners = edge_owners(mesh, (_edge.0, _edge.1)).len();
        let nz_of = |t: &[u32; 3]| -> f64 {
            let pa = &mesh.vertices[t[0] as usize];
            let pb = &mesh.vertices[t[1] as usize];
            let pc = &mesh.vertices[t[2] as usize];
            let e1 = [pb.x - pa.x, pb.y - pa.y, pb.z - pa.z];
            let e2 = [pc.x - pa.x, pc.y - pa.y, pc.z - pa.z];
            e1[0] * e2[1] - e1[1] * e2[0]
        };
        // third vertices
        let tri0 = mesh.triangles[t0];
        let tri1 = mesh.triangles[t1];
        let w0 = tri0.iter().find(|&&v| v != a0 && v != b0).copied();
        let w1 = tri1.iter().find(|&&v| v != a1 && v != b1).copied();
        let (Some(w0), Some(w1)) = (w0, w1) else { continue };
        if w0 as usize >= nv || w1 as usize >= nv {
            continue;
        }
        total += 1;
        let d = dist(w0, w1);
        let p0 = &mesh.vertices[w0 as usize];
        let p1 = &mesh.vertices[w1 as usize];
        let bits_eq = p0.x.to_bits() == p1.x.to_bits()
            && p0.y.to_bits() == p1.y.to_bits()
            && p0.z.to_bits() == p1.z.to_bits();
        if bits_eq {
            bit += 1;
        }
        if d < 1e-9 {
            eps += 1;
        }
        if d < 0.05 {
            subtol += 1;
        }
        // Report interesting cases (near-coincident, first few, or
        // everything on a forensics-targeted BREP)
        let want_all = std::env::var("TWIN_CHECK_BREP")
            .map(|v| v.parse::<i64>().map(|b| b == brep_id).unwrap_or(false))
            .unwrap_or(false);
        if want_all || (d < 1e-6 || reported < 8) && reported < 24 {
            reported += 1;
            println!(
                "TWIN brep#{} fid={} t{}=[{},{},{}] t{}=[{},{},{}] d_third={:.3e} bits={} areas=({:.5},{:.5}) edge_owners={} nz=({:.2e},{:.2e})",
                brep_id,
                fids.get(t0).copied().unwrap_or(0),
                t0, tri0[0], tri0[1], tri0[2],
                t1, tri1[0], tri1[1], tri1[2],
                d, bits_eq,
                area_of(&tri0), area_of(&tri1),
                n_owners,
                nz_of(&tri0), nz_of(&tri1)
            );
            println!(
                "     w{}=({:.6},{:.6},{:.6}) w{}=({:.6},{:.6},{:.6})",
                w0, p0.x, p0.y, p0.z, w1, p1.x, p1.y, p1.z
            );
        }
    }
    (total, bit, eps, subtol)
}
