//! session-44 diagnostic: classify interior-edge pairs with dihedral >170°
//! in a converted BREP mesh. For each pair dump: face ids, surface types,
//! face `forward`/`is_void` orientation flags, TOPOLOGICAL winding
//! consistency (does the shared edge appear in opposite directions in the
//! two triangles?), and the apex side test (same side = genuine geometric
//! fold-over overlap; opposite sides = proper tiling). Combined classes:
//!   TOPO-CONSISTENT + SAME-SIDE apexes  → FOLD-OVER (genuine overlap)
//!   TOPO-CONSISTENT + OPP-SIDE apexes   → CURVED-180 (needs geometric look)
//!   TOPO-FLIPPED   + OPP-SIDE apexes    → WINDING-FLIP (one face inverted)
//!   TOPO-FLIPPED   + SAME-SIDE apexes   → DOUBLE-BROKEN
//! Aggregate histogram per (surfaceA × surfaceB × class). Read-only.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::Point3d;
use std::collections::HashMap;

// Shared surface-justified dihedral exemption logic (also used by the
// angle_check release gate — single source of truth).
#[path = "../surf_exempt.rs"]
mod surf_exempt;

fn tri_normal(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> Option<Point3d> {
    let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
    let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
    let n = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if l < 1e-15 {
        return None;
    }
    Some(Point3d::new(n[0] / l, n[1] / l, n[2] / l))
}

fn tri_area(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
    let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
    let n = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
}

// surface_distance / surface_unit_normal / same_analytic_surface and the
// pair-level exemption live in the shared `surf_exempt` module.

/// Transform a surface from BREP-local space into world space by a 4×4
/// (assumed rigid: rotation + translation) matrix.
fn transform_surface(
    surf: &draper_geometry::Surface,
    m: &[[f64; 4]; 4],
) -> draper_geometry::Surface {
    use draper_geometry::Surface;
    let tp = |p: &Point3d| -> Point3d {
        Point3d::new(
            m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
            m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
            m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3],
        )
    };
    let norm_dir = |d: &draper_geometry::Direction3d| -> draper_geometry::Direction3d {
        let v = (
            m[0][0] * d.x + m[0][1] * d.y + m[0][2] * d.z,
            m[1][0] * d.x + m[1][1] * d.y + m[1][2] * d.z,
            m[2][0] * d.x + m[2][1] * d.y + m[2][2] * d.z,
        );
        let l = (v.0 * v.0 + v.1 * v.1 + v.2 * v.2).sqrt();
        if l < 1e-15 {
            return *d;
        }
        draper_geometry::Direction3d::new(v.0 / l, v.1 / l, v.2 / l)
            .unwrap_or(*d)
    };
    match surf {
        Surface::Plane(pl) => Surface::Plane(draper_geometry::Plane {
            origin: tp(&pl.origin),
            u_dir: norm_dir(&pl.u_dir),
            v_dir: norm_dir(&pl.v_dir),
            normal: norm_dir(&pl.normal),
        }),
        Surface::Cylinder(cy) => Surface::Cylinder(draper_geometry::CylinderSurface {
            origin: tp(&cy.origin),
            axis: norm_dir(&cy.axis),
            radius: cy.radius,
            x_dir: norm_dir(&cy.x_dir),
        }),
        Surface::Cone(co) => Surface::Cone(draper_geometry::ConeSurface {
            origin: tp(&co.origin),
            axis: norm_dir(&co.axis),
            half_angle: co.half_angle,
            radius: co.radius,
            x_dir: norm_dir(&co.x_dir),
            expanding: co.expanding,
        }),
        Surface::Torus(to) => Surface::Torus(draper_geometry::TorusSurface {
            center: tp(&to.center),
            axis: norm_dir(&to.axis),
            major_radius: to.major_radius,
            minor_radius: to.minor_radius,
            x_dir: norm_dir(&to.x_dir),
        }),
        other => other.clone(),
    }
}

fn main() {
    env_logger::builder()
        // session-52: default Warn, honor RUST_LOG on top (parse_default_env).
        // The previous filter(Some("RUST_LOG"), Warn) did NOT parse the env
        // var — it created a directive for a module literally named
        // "RUST_LOG", leaving the global default at Error: WARN lines were
        // silently suppressed without RUST_LOG, which masked the T-junction
        // explosion in solo runs and manufactured a false "non-determinism"
        // hypothesis (worklog-52 §2).
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env()
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args
        .iter()
        .skip(1)
        .find(|a| !a.starts_with('-'))
        .cloned()
        .unwrap_or_else(|| "test/Zentralstaender.stp".to_string());
    // Optional: pending-list index (row number in angle_check output) or
    // "all" (default) to scan every BREP.
    let target: String = args
        .iter()
        .skip(2)
        .find(|a| !a.starts_with('-'))
        .cloned()
        .unwrap_or_else(|| "all".to_string());

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    let mut grand: HashMap<String, usize> = HashMap::new();

    for (i, p) in pending.iter().enumerate() {
        if target != "all" {
            let want: usize = target.parse().unwrap_or(usize::MAX);
            if i != want {
                continue;
            }
        }
        let Some(inst) = ctx.triangulate_pending(p) else {
            continue;
        };
        // Session-74: per-BREP effective vertex-resolution tolerance
        // (merge + every weld pass). Fold pairs whose BOTH triangles are
        // thinner than this live under the mesh's guaranteed resolution —
        // sub-tolerance weld noise, not triangulation debt. Kill-switch:
        // DRAPPER_SUBTOL_EXEMPT=0 disables the classification.
        let subtol_enabled = std::env::var("DRAPPER_SUBTOL_EXEMPT")
            .map(|v| v != "0")
            .unwrap_or(true);
        let eff_tol = if subtol_enabled {
            ctx.brep_eff_tol(inst.brep_id)
        } else {
            None
        };
        if std::env::var("DRAPPER_DUMP_SURF_PARAMS").is_ok() {
            match inst.transform {
                Some(m) => println!(
                    "INST brep_idx={} {} transform=[{:.4} {:.4} {:.4} {:.4}; {:.4} {:.4} {:.4} {:.4}; {:.4} {:.4} {:.4} {:.4}]",
                    i, inst.name,
                    m[0][0], m[0][1], m[0][2], m[0][3],
                    m[1][0], m[1][1], m[1][2], m[1][3],
                    m[2][0], m[2][1], m[2][2], m[2][3]
                ),
                None => println!("INST brep_idx={} {} transform=identity", i, inst.name),
            }
        }
        let mesh = &inst.mesh;
        // session-47 diagnostics: dump the FINAL (post-repair) instance
        // mesh as OBJ + per-triangle face ids, for offline reconstruction
        // of fold-pair provenance at merged level.
        if let Ok(dir) = std::env::var("DRAPPER_DUMP_FINAL_OBJS") {
            let _ = std::fs::create_dir_all(&dir);
            let path = format!("{}/brep{}_{}.obj", dir, i, inst.name);
            let mut obj = String::with_capacity(1 << 16);
            for v in &mesh.vertices {
                obj.push_str(&format!("v {:.9} {:.9} {:.9}\n", v.x, v.y, v.z));
            }
            for t in &mesh.triangles {
                obj.push_str(&format!("f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1));
            }
            let _ = std::fs::write(&path, obj);
            let fids = mesh.triangle_face_ids.as_ref();
            let mut fmap = String::new();
            for (ti, t) in mesh.triangles.iter().enumerate() {
                let fid = fids.map(|f| f[ti]).unwrap_or(u64::MAX);
                fmap.push_str(&format!("t {} {}\n", ti, fid));
            }
            let _ = std::fs::write(&path.replace(".obj", ".fmap"), fmap);
            // session-67 diagnostics: face-id → surface type map, for
            // offline bnd census by surface class.
            let mut facemap = String::new();
            for f in &inst.faces {
                facemap.push_str(&format!(
                    "f {} {} {} {}\n",
                    f.face_id,
                    f.surface_type,
                    f.step_face_id,
                    f.forward
                ));
            }
            let _ = std::fs::write(&path.replace(".obj", ".facemap"), facemap);
        }
        let faces = &inst.faces;
        // face_id → FaceInfo (face ids are NOT guaranteed dense; the vec is
        // positional while ids may skip values).
        let face_by_id: HashMap<u64, &draper_step::FaceInfo> =
            faces.iter().map(|f| (f.face_id, f)).collect();
        let face_type = |fid: u64| -> String {
            face_by_id
                .get(&fid)
                .map(|f| {
                    format!(
                        "{}{}{}",
                        f.surface_type,
                        if f.forward { "" } else { "!fwd" },
                        if f.is_void { "/void" } else { "" }
                    )
                })
                .unwrap_or_else(|| {
                    if fid == u64::MAX {
                        "?MAX".to_string()
                    } else {
                        format!("?{}", fid)
                    }
                })
        };
        let face_step_id = |fid: u64| -> i64 {
            face_by_id.get(&fid).map(|f| f.step_face_id).unwrap_or(-1)
        };
        // World-space surface lookup: FaceInfo.surface is in BREP-LOCAL
        // coordinates while the mesh is transformed to world space. Apply
        // the instance transform before any geometric comparison.
        let xform: Option<[[f64; 4]; 4]> = inst.transform;
        let face_surface = |fid: u64| -> Option<draper_geometry::Surface> {
            let s = face_by_id.get(&fid).map(|f| f.surface.clone())?;
            Some(match xform {
                Some(m) => transform_surface(&s, &m),
                None => s,
            })
        };
        let fids = mesh.triangle_face_ids.as_ref();

        // edge (sorted) → list of (tri_idx, directed a, directed b)
        let mut edge_to_tris: HashMap<(u32, u32), Vec<(usize, u32, u32)>> = HashMap::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let (a, b, c) = (tri[0], tri[1], tri[2]);
            for (v0, v1) in [(a, b), (b, c), (c, a)] {
                let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                edge_to_tris.entry(key).or_default().push((ti, v0, v1));
            }
        }

        let normals: Vec<Option<Point3d>> = mesh
            .triangles
            .iter()
            .map(|tri| {
                tri_normal(
                    &mesh.vertices[tri[0] as usize],
                    &mesh.vertices[tri[1] as usize],
                    &mesh.vertices[tri[2] as usize],
                )
            })
            .collect();

        let mut brep_pairs = 0usize;
        let mut brep_exempt = 0usize;
        let mut brep_subtol = 0usize;
        let mut brep_knife = 0usize;
        let mut brep_coil = 0usize;
        let mut brep_junction = 0usize;
        let mut brep_thinface = 0usize;
        // session-121: per-(instance, face) cache of the rim polyline
        // far-loop min distance — the RIM-GAP witness input.
        let mut rim_far_cache: HashMap<u64, Option<f64>> = HashMap::new();
        // s132: per-(instance, face) cache of the WORLD-frame rim
        // polylines (outer + inner) — the RIM-COLLAPSE witness input
        // (triad-vertex-to-rim distances; the mesh vertices are in
        // the instance frame, the polylines are transformed by the
        // same xform the rim-gap path uses).
        let mut rim_poly_cache: HashMap<u64, Option<Vec<Vec<Point3d>>>> =
            HashMap::new();
        // session-123 recon: lazy per-BREP vertex → distinct-face-ids map
        // (the HUB witness input for the junction-sliver exemption —
        // built on first use, one pass over triangles + face ids).
        let mut vertex_faces: Option<HashMap<u32, std::collections::HashSet<u64>>> = None;
        // session-124 recon: lazily-built per-face patch metrics
        // (area, boundary perimeter, tri count) for the same-face
        // thin-pair witness (inherently-thin face class).
        let mut face_patch: Option<HashMap<u64, (f64, f64, usize)>> = None;
        // s132: lazily-built per-BREP vertex → incident-triangle count
        // (the weld-absorption witness input: a merged vertex that
        // absorbed a sub-tol duplicate carries the union of both
        // pre-weld stars — ~2x the regular lattice valence).
        let mut vert_tri_count: Option<HashMap<u32, usize>> = None;
        let mut hist: HashMap<String, usize> = HashMap::new();
        let mut involved_fids: std::collections::HashSet<u64> = std::collections::HashSet::new();

        for (edge, tris) in &edge_to_tris {
            if tris.len() != 2 {
                continue;
            }
            let (t0, da0, db0) = tris[0];
            let (t1, da1, db1) = tris[1];
            let (Some(n0), Some(n1)) = (&normals[t0], &normals[t1]) else {
                continue;
            };
            let dot = n0.x * n1.x + n0.y * n1.y + n0.z * n1.z;
            let ang = dot.clamp(-1.0, 1.0).acos().to_degrees();
            if ang <= 170.0 {
                continue;
            }
            brep_pairs += 1;

            // Topological winding consistency: the shared edge must be
            // traversed in OPPOSITE directions by the two triangles.
            let topo_consistent = (da0 == db1) && (db0 == da1);

            // Apex side test: both apexes relative to the shared edge line.
            let a = mesh.vertices[edge.0 as usize];
            let b = mesh.vertices[edge.1 as usize];
            let apex_of = |ti: usize, e0: u32, e1: u32| -> Point3d {
                let tri = &mesh.triangles[ti];
                for &v in tri {
                    if v != e0 && v != e1 {
                        return mesh.vertices[v as usize];
                    }
                }
                mesh.vertices[tri[0] as usize]
            };
            let c0 = apex_of(t0, edge.0, edge.1);
            let c1 = apex_of(t1, edge.0, edge.1);
            let e = [b.x - a.x, b.y - a.y, b.z - a.z];
            let side = |q: &Point3d| {
                let d = [q.x - a.x, q.y - a.y, q.z - a.z];
                [
                    e[1] * d[2] - e[2] * d[1],
                    e[2] * d[0] - e[0] * d[2],
                    e[0] * d[1] - e[1] * d[0],
                ]
            };
            let s0 = side(&c0);
            let s1 = side(&c1);
            let same_side = s0[0] * s1[0] + s0[1] * s1[1] + s0[2] * s1[2] > 0.0;

            let class = if topo_consistent {
                if same_side {
                    "FOLD-OVER"
                } else {
                    "CURVED-180"
                }
            } else if same_side {
                "DOUBLE-BROKEN"
            } else {
                "WINDING-FLIP"
            };

            let fid0 = fids.and_then(|ids| ids.get(t0).copied()).unwrap_or(0);
            let fid1 = fids.and_then(|ids| ids.get(t1).copied()).unwrap_or(0);
            involved_fids.insert(fid0);
            involved_fids.insert(fid1);
            let st0 = face_type(fid0);
            let st1 = face_type(fid1);

            // Range-owner check for EVERY pair (not just "?"): which face's
            // triangle_range contains each triangle. Reveals misattribution
            // and double-attribution (two faces' ranges covering the same
            // triangle index).
            let owner_of = |ti: usize| -> String {
                let mut owners = Vec::new();
                for f in faces {
                    if ti >= f.triangle_range.0 && ti < f.triangle_range.1 {
                        owners.push(format!("{}({})", f.face_id, f.surface_type));
                    }
                }
                if owners.is_empty() {
                    "post-repair".to_string()
                } else {
                    owners.join("+")
                }
            };
            let owner_note = format!(
                " owners=({},{})",
                owner_of(t0),
                owner_of(t1)
            );

            // Env-gated: dump full vertex coordinates of both triangles.
            if std::env::var("DRAPPER_DUMP_PAIR_VERTS").is_ok() {
                let pv = |tri: &[u32; 3]| -> String {
                    tri.iter()
                        .map(|&vi| {
                            let v = &mesh.vertices[vi as usize];
                            format!("({:.4},{:.4},{:.4})", v.x, v.y, v.z)
                        })
                        .collect::<Vec<_>>()
                        .join(" ")
                };
                println!(
                    "    VERTS T{}=[{}] T{}=[{}]",
                    t0,
                    pv(&mesh.triangles[t0]),
                    t1,
                    pv(&mesh.triangles[t1])
                );
            }

            let area0 = tri_area(
                &mesh.vertices[mesh.triangles[t0][0] as usize],
                &mesh.vertices[mesh.triangles[t0][1] as usize],
                &mesh.vertices[mesh.triangles[t0][2] as usize],
            );
            let area1 = tri_area(
                &mesh.vertices[mesh.triangles[t1][0] as usize],
                &mesh.vertices[mesh.triangles[t1][1] as usize],
                &mesh.vertices[mesh.triangles[t1][2] as usize],
            );

            // Coincidence test: distance of each triangle's centroid to the
            // OTHER face's surface. Both ≈ 0 → the two faces occupy the same
            // region of space (duplicate/coincident faces). Only one ≈ 0 →
            // tangential/crossing junction. None → unrelated surfaces.
            let centroid = |ti: usize| -> Point3d {
                let tri = &mesh.triangles[ti];
                let (va, vb, vc) = (
                    &mesh.vertices[tri[0] as usize],
                    &mesh.vertices[tri[1] as usize],
                    &mesh.vertices[tri[2] as usize],
                );
                Point3d::new(
                    (va.x + vb.x + vc.x) / 3.0,
                    (va.y + vb.y + vc.y) / 3.0,
                    (va.z + vb.z + vc.z) / 3.0,
                )
            };
            let g0 = centroid(t0);
            let g1 = centroid(t1);
            let surf0 = face_surface(fid0);
            let surf1 = face_surface(fid1);
            let d01 = surf1.as_ref().and_then(|s| surf_exempt::surface_distance(s, &g0));
            let d10 = surf0.as_ref().and_then(|s| surf_exempt::surface_distance(s, &g1));
            // Self-distances: sanity check (each triangle's centroid must be
            // near its OWN surface — chord sagitta level).
            let d00 = surf0.as_ref().and_then(|s| surf_exempt::surface_distance(s, &g0));
            let d11 = surf1.as_ref().and_then(|s| surf_exempt::surface_distance(s, &g1));
            // Env-gated: dump the two surfaces' analytic parameters.
            let surf_params = |fid: u64| -> String {
                use draper_geometry::Surface;
                match face_surface(fid) {
                    Some(Surface::Cylinder(cy)) => format!(
                        "cyl(o=({:.2},{:.2},{:.2}),ax=({:.3},{:.3},{:.3}),r={:.3})",
                        cy.origin.x, cy.origin.y, cy.origin.z, cy.axis.x, cy.axis.y, cy.axis.z, cy.radius
                    ),
                    Some(Surface::Torus(to)) => format!(
                        "tor(o=({:.2},{:.2},{:.2}),ax=({:.3},{:.3},{:.3}),R={:.3},r={:.3})",
                        to.center.x, to.center.y, to.center.z, to.axis.x, to.axis.y, to.axis.z,
                        to.major_radius, to.minor_radius
                    ),
                    Some(Surface::Plane(pl)) => format!(
                        "pln(o=({:.2},{:.2},{:.2}),n=({:.3},{:.3},{:.3}))",
                        pl.origin.x, pl.origin.y, pl.origin.z, pl.normal.x, pl.normal.y, pl.normal.z
                    ),
                    Some(Surface::Cone(co)) => format!(
                        "con(o=({:.2},{:.2},{:.2}),ax=({:.3},{:.3},{:.3}),ha={:.4},r={:.3})",
                        co.origin.x, co.origin.y, co.origin.z, co.axis.x, co.axis.y, co.axis.z,
                        co.half_angle, co.radius
                    ),
                    _ => String::new(),
                }
            };
            let params_note = if std::env::var("DRAPPER_DUMP_SURF_PARAMS").is_ok() {
                format!(" [{}] [{}]", surf_params(fid0), surf_params(fid1))
            } else {
                String::new()
            };
            let coincide = match (d01, d10) {
                (Some(a), Some(b)) => a < 1e-6 && b < 1e-6,
                _ => false,
            };
            let coincident_str = match (d01, d10) {
                (Some(a), Some(b)) => {
                    if coincide {
                        "COINCIDENT".to_string()
                    } else {
                        format!(
                            "d01={:.2e} d10={:.2e} self=({:.2e},{:.2e})",
                            a,
                            b,
                            d00.unwrap_or(f64::NAN),
                            d11.unwrap_or(f64::NAN)
                        )
                    }
                }
                _ => "n/a".to_string(),
            };

            // Sliver classification: min height of either triangle over the
            // shared edge base. Needles at tangent corners have height/base
            // < 1e-2; genuine fan overlaps are fat.
            let base_len = {
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let dz = b.z - a.z;
                (dx * dx + dy * dy + dz * dz).sqrt()
            };
            let point_line_dist = |p: &Point3d| -> f64 {
                // distance from p to line(a, b)
                let ab = [b.x - a.x, b.y - a.y, b.z - a.z];
                let ap = [p.x - a.x, p.y - a.y, p.z - a.z];
                let cr = [
                    ap[1] * ab[2] - ap[2] * ab[1],
                    ap[2] * ab[0] - ap[0] * ab[2],
                    ap[0] * ab[1] - ap[1] * ab[0],
                ];
                let l = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
                l / base_len.max(1e-30)
            };
            let h0 = point_line_dist(&c0);
            let h1 = point_line_dist(&c1);
            let sliver_class = if h0 < 1e-2 * base_len || h1 < 1e-2 * base_len {
                "SLIVER"
            } else {
                "FAT"
            };

            // ---- surface-justified dihedral exemption (session-46,
            // direction (c) of session-44, generalized) — see
            // surf_exempt.rs for the criterion and its guards. The
            // analytic surface normals are evaluated at the shared-edge
            // ENDPOINTS (bit-exact rim points) because the midpoint of a
            // chord of the tangency circle sits inside the circle and the
            // projected torus/cone normal there picks up a chord-sagitta
            // tilt (measured 3.8° on B_WELLE), masking exact tangency.
            let verdict = surf_exempt::evaluate(
                &mesh.vertices,
                &mesh.triangles[t0],
                &mesh.triangles[t1],
                ang,
                fid0,
                fid1,
                surf0.as_ref(),
                surf1.as_ref(),
            );
            let tangent_exempt = verdict.exempt;

            let sn_str = match (verdict.sn_rim, verdict.sn_mid) {
                (Some(a), Some(m)) => format!("snAng={:.3}/mid={:.3}", a, m),
                (Some(a), None) => format!("snAng={:.3}", a),
                (None, _) => "snAng=n/a".to_string(),
            };
            let exempt_str = if tangent_exempt { " TANGENT-EXEMPT" } else { "" };
            // Session-74: sub-tolerance weld-noise classification. A pair
            // whose BOTH apex heights are below the BREP's effective
            // vertex-resolution tolerance (the max of merge_tol and every
            // weld pass applied) is under the resolution the output mesh
            // guarantees — the thin lip/tab geometry cannot be represented
            // at this tolerance, so its 180° folds are weld noise.
            let subtol =
                !tangent_exempt && matches!(eff_tol, Some(tol) if h0 < tol && h1 < tol);
            if subtol {
                brep_subtol += 1;
            }
            let subtol_str = if subtol { " SUBTOL" } else { "" };
            // session-94: KNIFE-FIN (thin-wall pinch) exemption — a
            // cross-face FOLD-OVER pair that is two offset copies
            // stacked across a resolvable wall gap (the flattened-tube
            // pinch; transmission SPRING f2/f3). Not debt.
            // Kill-switch: DRAPPER_KNIFE_FIN=0.
            let (knife, knife_why) = if !tangent_exempt
                && !subtol
                && class == "FOLD-OVER"
                && fid0 != fid1
                && std::env::var("DRAPPER_KNIFE_FIN").as_deref() != Ok("0")
            {
                surf_exempt::knife_fin(
                    &mesh.vertices,
                    &mesh.triangles[t0],
                    &mesh.triangles[t1],
                    eff_tol.unwrap_or(0.0),
                )
            } else {
                (false, String::new())
            };
            if knife {
                brep_knife += 1;
            }
            let knife_str = if knife { " KNIFE-FIN" } else { "" };
            // session-121: COIL-TURN-WELD exemption — a same-face >170°
            // pair on a coiled ribbon (Nurbs with a degree-1 direction)
            // whose thin fold (max h < 2×eff_tol) is rooted in coil
            // geometry the merge tolerance cannot represent: adjacent
            // turns welded into one seam (f11 x f13 gap 0.0079 < 0.0156 →
            // BFS collateral) or the band / turn gap narrower than the
            // tolerance (f28 w=8.9e-3 — merge-doomed by geometry).
            // Kill-switch: DRAPPER_COIL_TURN_EXEMPT=0. Not debt.
            let (coil, coil_why, coil_star, coil_rim, coil_triad_on_rim) =
                if !tangent_exempt && !subtol && !knife && fid0 == fid1
                    && std::env::var("DRAPPER_COIL_TURN_EXEMPT").as_deref() != Ok("0")
                {
                    // s132 diagnostics: per-vertex triad-to-rim min
                    // distances (printed by the TRIAD_WITNESS dump).
                    let mut coil_triad_rim_dists: Option<Vec<(u32, f64, usize)>> = None;
                    // s132: build the vertex → incident-triangle count
                    // map on first use (one pass over the BREP's
                    // triangles — the weld-absorption diagnostic).
                    if vert_tri_count.is_none() {
                        let mut m: HashMap<u32, usize> = HashMap::new();
                        for tri in mesh.triangles.iter() {
                            for &v in tri.iter() {
                                *m.entry(v).or_default() += 1;
                            }
                        }
                        vert_tri_count = Some(m);
                    }
                    let vtc = vert_tri_count.as_ref().unwrap();
                    // witness (a) STAR: a cross-face interior edge at one
                    // of the pair's four triad vertices (the welded rail
                    // in the merged mesh at the pair).
                    let star = {
                        use std::collections::HashSet;
                        let mut triad: HashSet<u32> = HashSet::new();
                        for &v in mesh.triangles[t0].iter().chain(mesh.triangles[t1].iter()) {
                            triad.insert(v);
                        }
                        // Collect ALL cross-face star edges, then pick the
                        // DETERMINISTIC minimum key — edge_to_tris is a
                        // HashMap, and first-found would vary run to run
                        // (s117 lesson-5: diagnostics must be reproducible).
                        let mut founds: Vec<((u32, u32), (u64, u64))> = Vec::new();
                        for (ek, et) in &edge_to_tris {
                            if et.len() != 2 {
                                continue;
                            }
                            if !triad.contains(&ek.0) && !triad.contains(&ek.1) {
                                continue;
                            }
                            let (wa, _, _) = et[0];
                            let (wb, _, _) = et[1];
                            let fa = fids.and_then(|ids| ids.get(wa).copied()).unwrap_or(0);
                            let fb = fids.and_then(|ids| ids.get(wb).copied()).unwrap_or(0);
                            if fa != fb {
                                founds.push((*ek, (fa, fb)));
                            }
                        }
                        founds.sort_by_key(|(e, _)| *e);
                        founds.first().copied()
                    };
                    // witness (b) RIM-GAP: the owner face's rim polyline
                    // returns within eff_tol of itself at a FAR position
                    // of the loop (index distance >= n/4, wrap neighbours
                    // and the closing duplicate excluded). Cached per face.
                    let rim_far = if let Some(c) = rim_far_cache.get(&fid0) {
                        *c
                    } else {
                        let v = (|| -> Option<f64> {
                            let polys = rim_poly_cache.entry(fid0).or_insert_with(
                                || {
                                    let f = face_by_id.get(&fid0)?;
                                    let tf = |p: &Point3d| -> Point3d {
                                        match xform {
                                            Some(m) => Point3d::new(
                                                m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
                                                m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
                                                m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3],
                                            ),
                                            None => *p,
                                        }
                                    };
                                    Some(
                                        f.outer_boundary
                                            .iter()
                                            .chain(f.inner_boundaries.iter())
                                            .map(|poly| poly.iter().map(tf).collect())
                                            .collect(),
                                    )
                                },
                            );
                            let polys = polys.as_ref()?;
                            let mut self_min = f64::INFINITY;
                            for w in polys.iter() {
                                let n = w.len();
                                if n < 8 {
                                    continue;
                                }
                                let far = (n / 4).max(3);
                                for i in 0..n {
                                    for j in (i + far)..n {
                                        if i + n - j <= 2 {
                                            continue; // loop-closure neighbour
                                        }
                                        let d = (w[i].x - w[j].x).powi(2)
                                            + (w[i].y - w[j].y).powi(2)
                                            + (w[i].z - w[j].z).powi(2);
                                        if d < self_min {
                                            self_min = d;
                                        }
                                    }
                                }
                            }
                            if self_min.is_infinite() {
                                None
                            } else {
                                Some(self_min.sqrt())
                            }
                        })();
                        rim_far_cache.insert(fid0, v);
                        v
                    };
                    // witness (c) s132 RIM-COLLAPSE: the triad
                    // contains an ORPHAN vertex — ≤ 1 incident
                    // triangle (the repair cascade dismantled the
                    // duplicate zone; the pair is a post-repair
                    // remnant). The rim-distance per-vertex numbers
                    // are kept as diagnostics (the WITNESS dump) —
                    // the MEASURED f28 triad sits 0.024–0.050 off the
                    // rim, so rim proximity is NOT the signal.
                    let triad_orphan = {
                        let mut triad: Vec<u32> =
                            mesh.triangles[t0]
                                .iter()
                                .chain(mesh.triangles[t1].iter())
                                .copied()
                                .collect();
                        triad.sort_unstable();
                        triad.dedup();
                        // s132 diagnostics: per-vertex min rim
                        // distance + valence (printed by the WITNESS
                        // dump — the negative-control evidence).
                        let polys = rim_poly_cache.get(&fid0);
                        let eps = eff_tol.unwrap_or(0.0);
                        let mut dists: Vec<(u32, f64, usize)> = Vec::new();
                        let mut any_orphan = false;
                        for &vi in triad.iter() {
                            let vc = vtc.get(&vi).copied().unwrap_or(0);
                            if vc <= 1 {
                                any_orphan = true;
                            }
                            let mut best = f64::INFINITY;
                            if let Some(Some(polys)) = polys {
                                let p = &mesh.vertices[vi as usize];
                                for w in polys.iter() {
                                    let n = w.len();
                                    if n == 0 {
                                        continue;
                                    }
                                    // consecutive-pair segments (a
                                    // closed polyline carries its
                                    // closing duplicate, so the wrap
                                    // segment is covered too)
                                    for k in 0..n.saturating_sub(1) {
                                        let a = &w[k];
                                        let b = &w[k + 1];
                                        let abx = b.x - a.x;
                                        let aby = b.y - a.y;
                                        let abz = b.z - a.z;
                                        let apx = p.x - a.x;
                                        let apy = p.y - a.y;
                                        let apz = p.z - a.z;
                                        let ab2 = abx * abx + aby * aby + abz * abz;
                                        let t = if ab2 > 1e-30 {
                                            ((apx * abx + apy * aby + apz * abz) / ab2)
                                                .clamp(0.0, 1.0)
                                        } else {
                                            0.0
                                        };
                                        let dx = apx - t * abx;
                                        let dy = apy - t * aby;
                                        let dz = apz - t * abz;
                                        let d2 = dx * dx + dy * dy + dz * dz;
                                        if d2 < best {
                                            best = d2;
                                        }
                                    }
                                }
                            }
                            dists.push((vi, best.sqrt(), vc));
                        }
                        let _ = eps;
                        coil_triad_rim_dists = Some(dists);
                        any_orphan
                    };
                    let w = surf_exempt::CoilWitness {
                        star: star.is_some(),
                        star_edge: star.map(|(e, _)| e).unwrap_or((0, 0)),
                        star_faces: star.map(|(_, f)| f).unwrap_or((0, 0)),
                        rim_far,
                        triad_orphan,
                    };
                    let r = surf_exempt::coil_turn_weld(
                        h0,
                        h1,
                        eff_tol.unwrap_or(0.0),
                        surf0.as_ref(),
                        Some(&w),
                    );
                    (r.0, r.1, star, rim_far, coil_triad_rim_dists)
                } else {
                    (false, String::new(), None, None, None)
                };
            if coil {
                brep_coil += 1;
            }
            let coil_str = if coil { " COIL-TURN" } else { "" };
            // session-123: JUNCTION-SLIVER (cross-face multi-face-hub
            // duplicate coverage) — the s122-п.3 class. The witness
            // (lazy per-BREP vertex→distinct-faces map + two-sided
            // plane deviations) is computed for cross-face REAL pairs
            // past the coil cascade, or whenever the recon dump asks
            // for it. Kill-switch: DRAPPER_JUNCTION_SLIDER=0. Not debt.
            let junction_candidate = !tangent_exempt
                && !subtol
                && !knife
                && !coil
                && fid0 != fid1
                && std::env::var("DRAPPER_JUNCTION_SLIDER").as_deref() != Ok("0");
            // session-123 recon (read-only, env-gated): dump the
            // junction witness facts BEFORE deciding thresholds (the
            // s121 lesson-1 method: measure the planned witness
            // first — the accepted thresholds in surf_exempt are the
            // measured ones).
            let junction_recon = std::env::var("DRAPPER_DUMP_JUNCTION").is_ok()
                && !tangent_exempt
                && !subtol
                && !knife
                && !coil
                && fid0 != fid1;
            let mut junction_witness: Option<surf_exempt::JunctionWitness> = None;
            if junction_candidate || junction_recon {
                if vertex_faces.is_none() {
                    let mut vf: HashMap<u32, std::collections::HashSet<u64>> =
                        HashMap::new();
                    for (ti, tri) in mesh.triangles.iter().enumerate() {
                        let fid = fids
                            .and_then(|ids| ids.get(ti).copied())
                            .unwrap_or(u64::MAX);
                        for &v in tri.iter() {
                            vf.entry(v).or_default().insert(fid);
                        }
                    }
                    vertex_faces = Some(vf);
                }
                let vf = vertex_faces.as_ref().unwrap();
                // Deterministic hub pick: iterate the pair's vertices
                // in sorted order, keep the FIRST maximum (s117
                // lesson-5: diagnostics must be reproducible).
                let mut vs: Vec<u32> = mesh.triangles[t0]
                    .iter()
                    .chain(mesh.triangles[t1].iter())
                    .copied()
                    .collect();
                vs.sort_unstable();
                vs.dedup();
                let mut hub_max = 0usize;
                let mut hub_vertex = 0u32;
                for v in vs {
                    let c = vf.get(&v).map(|s| s.len()).unwrap_or(0);
                    if c > hub_max {
                        hub_max = c;
                        hub_vertex = v;
                    }
                }
                // Two-sided plane deviations: max distance of the
                // other triangle's vertices from this triangle's
                // plane.
                let plane_dev = |ta: usize, tb: usize| -> Option<f64> {
                    let tri_a = &mesh.triangles[ta];
                    let (pa, pb, pc) = (
                        &mesh.vertices[tri_a[0] as usize],
                        &mesh.vertices[tri_a[1] as usize],
                        &mesh.vertices[tri_a[2] as usize],
                    );
                    let e1 = [pb.x - pa.x, pb.y - pa.y, pb.z - pa.z];
                    let e2 = [pc.x - pa.x, pc.y - pa.y, pc.z - pa.z];
                    let n = [
                        e1[1] * e2[2] - e1[2] * e2[1],
                        e1[2] * e2[0] - e1[0] * e2[2],
                        e1[0] * e2[1] - e1[1] * e2[0],
                    ];
                    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                    if l < 1e-15 {
                        return None;
                    }
                    let mut d_max = 0.0f64;
                    for &v in mesh.triangles[tb].iter() {
                        let p = &mesh.vertices[v as usize];
                        let d = ((p.x - pa.x) * n[0]
                            + (p.y - pa.y) * n[1]
                            + (p.z - pa.z) * n[2])
                            .abs()
                            / l;
                        d_max = d_max.max(d);
                    }
                    Some(d_max)
                };
                let d10 = plane_dev(t0, t1); // t1 verts vs t0 plane
                let d01 = plane_dev(t1, t0); // t0 verts vs t1 plane
                let plane_dev = match (d10, d01) {
                    (Some(a), Some(b)) => Some([a, b]),
                    _ => None,
                };
                junction_witness = Some(surf_exempt::JunctionWitness {
                    hub_max,
                    hub_vertex,
                    plane_dev,
                });
                if junction_recon {
                    println!(
                        "    JUNCTION brep#{} {} faces=({},{}) types=({},{}) class={} ang={:.2} h=({:.4},{:.4}) areas=({:.6},{:.6}) tol={:.4} hub=v{}x{} plane_dev=({:?},{:?})",
                        inst.brep_id,
                        inst.name,
                        fid0,
                        fid1,
                        st0,
                        st1,
                        class,
                        ang,
                        h0,
                        h1,
                        area0,
                        area1,
                        eff_tol.unwrap_or(0.0),
                        hub_vertex,
                        hub_max,
                        d10,
                        d01
                    );
                }
            }
            let (junction, junction_why) = if junction_candidate {
                surf_exempt::junction_sliver(
                    h0,
                    h1,
                    eff_tol.unwrap_or(0.0),
                    junction_witness.as_ref(),
                )
            } else {
                (false, String::new())
            };
            if junction {
                brep_junction += 1;
            }
            let junction_str = if junction { " JUNCTION-SLIDER" } else { "" };
            // session-124: THIN-FACE exemption — a same-face >170° pair
            // on an INHERENTLY THIN face (patch mean width 2A/P under
            // 4×eff_tol): any triangulation of a legitimately thin
            // face is a chain of thin triangles; the pairs are
            // resolution-level geometry (the planar crescent analogue
            // of the coil-turn class). The #10203 f(11xx) family + the
            // Spit re-evaluation classes (#31356 f(3,3), #27977
            // f(30,30)). Kill-switch: DRAPPER_THIN_FACE_EXEMPT=0. Not
            // debt.
            let thinface_candidate = !tangent_exempt
                && !subtol
                && !knife
                && !coil
                && !junction
                && fid0 == fid1
                && std::env::var("DRAPPER_THIN_FACE_EXEMPT").as_deref() != Ok("0");
            // session-124 recon (read-only, env-gated): dump the face
            // patch's mean width (2A/P) vs eff_tol for same-face REAL
            // pairs — the witness facts BEFORE thresholds (the s121
            // lesson-1 method).
            let thinface_recon = std::env::var("DRAPPER_DUMP_THINFACE").is_ok()
                && !tangent_exempt
                && !subtol
                && !knife
                && !coil
                && fid0 == fid1;
            let mut thinface_witness: Option<surf_exempt::ThinFaceWitness> = None;
            if thinface_candidate || thinface_recon {
                if face_patch.is_none() {
                    // Build per-face patch metrics ONCE per BREP mesh:
                    // area sum + boundary-edge perimeter (edges used
                    // exactly once within the face's own triangles).
                    use std::collections::HashMap as Map;
                    let mut tri_by_face: Map<u64, Vec<usize>> = Map::new();
                    for (ti, tri) in mesh.triangles.iter().enumerate() {
                        let fid = fids
                            .and_then(|ids| ids.get(ti).copied())
                            .unwrap_or(u64::MAX);
                        tri_by_face.entry(fid).or_default().push(ti);
                    }
                    let mut fp: HashMap<u64, (f64, f64, usize)> = HashMap::new();
                    for (fid, tis) in tri_by_face {
                        let mut area = 0.0f64;
                        let mut edge_count: Map<(u32, u32), u32> = Map::new();
                        for &ti in &tis {
                            let tri = &mesh.triangles[ti];
                            let (a, b, c) = (
                                &mesh.vertices[tri[0] as usize],
                                &mesh.vertices[tri[1] as usize],
                                &mesh.vertices[tri[2] as usize],
                            );
                            area += tri_area(a, b, c);
                            for (v0, v1) in [
                                (tri[0], tri[1]),
                                (tri[1], tri[2]),
                                (tri[2], tri[0]),
                            ] {
                                let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                                *edge_count.entry(key).or_insert(0) += 1;
                            }
                        }
                        let mut perim = 0.0f64;
                        for (key, cnt) in edge_count {
                            if cnt == 1 {
                                let (p, q) = (
                                    &mesh.vertices[key.0 as usize],
                                    &mesh.vertices[key.1 as usize],
                                );
                                perim += ((p.x - q.x).powi(2)
                                    + (p.y - q.y).powi(2)
                                    + (p.z - q.z).powi(2))
                                .sqrt();
                            }
                        }
                        fp.insert(fid, (area, perim, tis.len()));
                    }
                    face_patch = Some(fp);
                }
                let fp = face_patch.as_ref().unwrap();
                if let Some(&(area_f, perim_f, ntris_f)) = fp.get(&fid0) {
                    let w = if perim_f > 1e-12 {
                        2.0 * area_f / perim_f
                    } else {
                        0.0
                    };
                    thinface_witness = Some(surf_exempt::ThinFaceWitness {
                        width: w,
                        area: area_f,
                        perim: perim_f,
                        tris: ntris_f,
                    });
                    if thinface_recon {
                        let tol = eff_tol.unwrap_or(0.0);
                        println!(
                            "    THINFACE brep#{} {} faces=({},{}) type={} class={} ang={:.2} h=({:.4},{:.4}) areas=({:.6},{:.6}) tol={:.4} patchA={:.6} patchP={:.4} w={:.4} w/tol={:.2} patch_tris={}",
                            inst.brep_id,
                            inst.name,
                            fid0,
                            fid1,
                            st0,
                            class,
                            ang,
                            h0,
                            h1,
                            area0,
                            area1,
                            tol,
                            area_f,
                            perim_f,
                            w,
                            if tol > 1e-12 { w / tol } else { f64::INFINITY },
                            ntris_f
                        );
                    }
                }
            }
            let (thinface, thinface_why) = if thinface_candidate {
                surf_exempt::thin_face(
                    h0,
                    h1,
                    eff_tol.unwrap_or(0.0),
                    thinface_witness.as_ref(),
                )
            } else {
                (false, String::new())
            };
            if thinface {
                brep_thinface += 1;
            }
            let thinface_str = if thinface { " THIN-FACE" } else { "" };
            // session-121 recon (read-only, env-gated): for each REAL
            // (non-exempt / non-subtol / non-knife / non-coil /
            // non-junction) pair, dump the triad's edges + the coil
            // witness values. Reuses the cascade-computed star /
            // rim-gap. s132: coil-EXEMPT same-face pairs also dump
            // (the weld-zone comparison population).
            if std::env::var("DRAPPER_DUMP_TRIAD_WITNESS").is_ok()
                && !tangent_exempt
                && !subtol
                && !knife
                && (!coil || fid0 == fid1)
                && !junction
            {
                let mut wit: Vec<String> = Vec::new();
                for ti in [t0, t1] {
                    let tri = &mesh.triangles[ti];
                    for (v0, v1) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                        let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                        let Some(et) = edge_to_tris.get(&key) else {
                            continue;
                        };
                        if et.len() != 2 {
                            continue; // boundary edge of the mesh
                        }
                        let (wa, _, _) = et[0];
                        let (wb, _, _) = et[1];
                        let fa = fids.and_then(|ids| ids.get(wa).copied()).unwrap_or(0);
                        let fb = fids.and_then(|ids| ids.get(wb).copied()).unwrap_or(0);
                        let shared = if key == *edge { " [shared]" } else { "" };
                        wit.push(format!(
                            "e({},{}) t=({},{}) f=({},{}){}{}",
                            v0,
                            v1,
                            wa,
                            wb,
                            fa,
                            fb,
                            if fa != fb { " CROSS-FACE" } else { "" },
                            shared
                        ));
                    }
                }
                let star_note = match coil_star {
                    Some((e, f)) => format!(
                        " || star e({},{}) f=({},{})",
                        e.0, e.1, f.0, f.1
                    ),
                    None => String::new(),
                };
                let poly_note = match coil_rim {
                    Some(g) => format!(" POLY far={:.5}", g),
                    None => " POLY far=n/a".to_string(),
                };
                // s132: the triad-on-rim distances (the RIM-COLLAPSE
                // witness input) — which vertex sits how far from the
                // owner rim + its incident-triangle count (the
                // weld-absorption diagnostic).
                let triad_note = match &coil_triad_on_rim {
                    Some(dists) => format!(
                        " TRIAD-RIM [{}]",
                        dists
                            .iter()
                            .map(|(v, d, c)| format!("v{}={:.4}/t{}", v, d, c))
                            .collect::<Vec<_>>()
                            .join(" ")
                    ),
                    None => String::new(),
                };
                // POLYLINE / RIM-GAP: printed from the cascade-computed
                // value (same far-loop rule the exemption uses: index
                // distance >= n/4, wrap neighbours and the closing
                // duplicate excluded).
                println!(
                    "    WITNESS brep#{} {} real-pair class={} tris=({:?},{:?}) faces=({},{}) types=({},{}) h=({:.4},{:.4}) tol={:.4}: {}{}{}{}",
                    inst.brep_id,
                    inst.name,
                    class,
                    mesh.triangles[t0],
                    mesh.triangles[t1],
                    fid0,
                    fid1,
                    st0,
                    st1,
                    h0,
                    h1,
                    eff_tol.unwrap_or(0.0),
                    wit.join(" | "),
                    star_note,
                    poly_note,
                    triad_note
                );
            }
            println!(
                "[{}{}{}{}{}{}{}]{} brep_idx={} {} BREP#{} ang={:.2} faces=({},{}) types=({},{}) step=({},{}) tris=({:?},{:?}) areas=({:.4},{:.4}) h=({:.4},{:.4}) {} {}{}{}{}{}{}{} mid=({:.2},{:.2},{:.2})",
                class,
                sliver_class,
                subtol_str,
                knife_str,
                coil_str,
                junction_str,
                thinface_str,
                exempt_str,
                i,
                inst.name,
                inst.brep_id,
                ang,
                fid0,
                fid1,
                st0,
                st1,
                face_step_id(fid0),
                face_step_id(fid1),
                mesh.triangles[t0],
                mesh.triangles[t1],
                area0,
                area1,
                h0,
                h1,
                sn_str,
                coincident_str,
                owner_note,
                if knife { &knife_why } else { "" },
                if coil { &coil_why } else { "" },
                if junction { &junction_why } else { "" },
                if thinface { &thinface_why } else { "" },
                params_note,
                (a.x + b.x) / 2.0,
                (a.y + b.y) / 2.0,
                (a.z + b.z) / 2.0,
            );

            *hist
                .entry(format!(
                    "{}{}{}{}{}{} | {} | {}",
                    if tangent_exempt { "EXEMPT+" } else { "" },
                    if subtol { "SUBTOL+" } else { "" },
                    if knife { "KNIFE-FIN+" } else { "" },
                    if coil { "COIL-TURN+" } else { "" },
                    if junction { "JUNCTION-SLIDER+" } else { "" },
                    sliver_class,
                    st0,
                    st1
                ))
                .or_insert(0) += 1;
            *grand
                .entry(format!(
                    "{}{}{}{}{}{}{}+{} | {} | {}",
                    if tangent_exempt { "TANGENT-EXEMPT " } else { "" },
                    if subtol { "SUBTOL " } else { "" },
                    if knife { "KNIFE-FIN " } else { "" },
                    if coil { "COIL-TURN " } else { "" },
                    if junction { "JUNCTION-SLIDER " } else { "" },
                    class,
                    if coincide { "+COIN" } else { "" },
                    sliver_class,
                    st0,
                    st1
                ))
                .or_insert(0) += 1;
            if tangent_exempt {
                brep_exempt += 1;
            }
        }

        if brep_pairs > 0 {
            // session-44 v3 follow-up: dump boundary-wire statistics for the
            // faces involved in fold pairs — do their wires dip off-surface
            // (e.g., into the end-cap plane)?
            if std::env::var("DRAPPER_DUMP_WIRES").is_ok() {
                use std::collections::BTreeSet;
                let involved: BTreeSet<u64> =
                    involved_fids.iter().copied().collect();
                for &fid in &involved {
                    let Some(f) = face_by_id.get(&fid) else { continue };
                    // Wire polylines and FaceInfo.surface are BOTH in
                    // BREP-local space — compare against the LOCAL surface.
                    let surf = f.surface.clone();
                    let mut n_off = 0usize;
                    let mut n_tot = 0usize;
                    let mut max_off = 0.0f64;
                    for poly in f.outer_boundary.iter().chain(f.inner_boundaries.iter()) {
                        for p in poly {
                            n_tot += 1;
                            if let Some(d) = surf_exempt::surface_distance(&surf, p) {
                                if d > 1e-4 {
                                    n_off += 1;
                                    max_off = max_off.max(d);
                                }
                            }
                        }
                    }
                    println!(
                        "WIRE face={} {} step={} polys={} pts={} off_surface={} max_off={:.3}",
                        fid,
                        f.surface_type,
                        f.step_face_id,
                        f.outer_boundary.len() + f.inner_boundaries.len(),
                        n_tot,
                        n_off,
                        max_off
                    );
                }
            }
            println!(
                "--- brep_idx={} {} BREP#{}: {} pairs >170° ({} tangent-exempt, {} sub-tol weld-noise, {} knife-fin pinch, {} coil-turn weld, {} junction-sliver, {} thin-face, {} real){}",
                i,
                inst.name,
                inst.brep_id,
                brep_pairs,
                brep_exempt,
                brep_subtol,
                brep_knife,
                brep_coil,
                brep_junction,
                brep_thinface,
                brep_pairs
                    - brep_exempt
                    - brep_subtol
                    - brep_knife
                    - brep_coil
                    - brep_junction
                    - brep_thinface,
                eff_tol
                    .map(|t| format!(" eff_tol={:.4}", t))
                    .unwrap_or_default()
            );
            let mut keys: Vec<_> = hist.iter().collect();
            keys.sort_by(|a, b| b.1.cmp(a.1));
            for (k, v) in keys {
                println!("    {:>50} : {}", k, v);
            }
        }
    }

    println!("\n===== GRAND HISTOGRAM (class | surfA | surfB) =====");
    let mut keys: Vec<_> = grand.iter().collect();
    keys.sort_by(|a, b| b.1.cmp(a.1));
    for (k, v) in keys {
        println!("  {:>60} : {}", k, v);
    }
}
