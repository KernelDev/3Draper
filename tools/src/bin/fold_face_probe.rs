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

/// Distance from a point to a surface (plane/cylinder/cone/torus analytic;
/// None for everything else). The surface must ALREADY be in the same
/// space as the point (world).
fn surface_distance(surf: &draper_geometry::Surface, p: &Point3d) -> Option<f64> {
    use draper_geometry::Surface;
    match surf {
        Surface::Plane(pl) => {
            let d = [
                p.x - pl.origin.x,
                p.y - pl.origin.y,
                p.z - pl.origin.z,
            ];
            Some((d[0] * pl.normal.x + d[1] * pl.normal.y + d[2] * pl.normal.z).abs())
        }
        Surface::Cylinder(cy) => {
            // axis: origin + t*axis; distance to axis minus radius
            let d = [p.x - cy.origin.x, p.y - cy.origin.y, p.z - cy.origin.z];
            let a = [cy.axis.x, cy.axis.y, cy.axis.z];
            let t = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let perp = [
                d[0] - t * a[0],
                d[1] - t * a[1],
                d[2] - t * a[2],
            ];
            Some(((perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt() - cy.radius).abs())
        }
        Surface::Cone(co) => {
            let d = [p.x - co.origin.x, p.y - co.origin.y, p.z - co.origin.z];
            let a = [co.axis.x, co.axis.y, co.axis.z];
            let t = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let perp = [
                d[0] - t * a[0],
                d[1] - t * a[1],
                d[2] - t * a[2],
            ];
            let r = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
            let tan = co.half_angle.tan();
            let expect = if co.expanding { co.radius + t * tan } else { co.radius - t * tan };
            Some((r - expect).abs())
        }
        Surface::Torus(to) => {
            // Torus: center, axis, major (ring) radius, minor radius.
            // Surface distance: |dist(P, ring circle) − minor_radius|.
            let d = [p.x - to.center.x, p.y - to.center.y, p.z - to.center.z];
            let a = [to.axis.x, to.axis.y, to.axis.z];
            let h = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let q = [d[0] - h * a[0], d[1] - h * a[1], d[2] - h * a[2]];
            let ql = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt();
            if ql < 1e-12 {
                return Some(to.minor_radius.abs());
            }
            let ring_dist = ((ql - to.major_radius).powi(2) + h * h).sqrt();
            Some((ring_dist - to.minor_radius).abs())
        }
        _ => None,
    }
}

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
        .filter_level(log::LevelFilter::Warn)
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
            let d01 = surf1.as_ref().and_then(|s| surface_distance(s, &g0));
            let d10 = surf0.as_ref().and_then(|s| surface_distance(s, &g1));
            // Self-distances: sanity check (each triangle's centroid must be
            // near its OWN surface — chord sagitta level).
            let d00 = surf0.as_ref().and_then(|s| surface_distance(s, &g0));
            let d11 = surf1.as_ref().and_then(|s| surface_distance(s, &g1));
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

            println!(
                "[{}{}] brep_idx={} {} BREP#{} ang={:.2} faces=({},{}) types=({},{}) step=({},{}) tris=({:?},{:?}) areas=({:.4},{:.4}) h=({:.4},{:.4}) {}{}{} mid=({:.2},{:.2},{:.2})",
                class,
                sliver_class,
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
                coincident_str,
                owner_note,
                params_note,
                (a.x + b.x) / 2.0,
                (a.y + b.y) / 2.0,
                (a.z + b.z) / 2.0,
            );

            *hist
                .entry(format!("{} | {}", sliver_class, st0))
                .or_insert(0) += 1;
            *hist.entry(format!("{} | {}", st0, st1)).or_insert(0) += 1;
            *grand
                .entry(format!(
                    "{}{}+{} | {} | {}",
                    class,
                    if coincide { "+COIN" } else { "" },
                    sliver_class,
                    st0,
                    st1
                ))
                .or_insert(0) += 1;
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
                            if let Some(d) = surface_distance(&surf, p) {
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
                "--- brep_idx={} {} BREP#{}: {} pairs >170°",
                i, inst.name, inst.brep_id, brep_pairs
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
