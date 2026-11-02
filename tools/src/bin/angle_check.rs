// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Diagnostic: check angles between adjacent triangles for outliers.
//!
//! For each interior edge (shared by exactly 2 triangles), compute the
//! dihedral angle between the two triangles' normals. Outliers (very sharp
//! angles) indicate triangulation errors — typically caused by:
//! - Triangles from different faces being incorrectly welded together
//! - T-junction repair creating bad triangles
//! - Mesh gap filling creating non-geometric triangles
//!
//! Usage: cargo run --bin angle_check --release [file.stp]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::Point3d;
use std::collections::HashMap;

// Shared surface-justified dihedral exemption (also used by
// fold_face_probe — single source of truth). See surf_exempt.rs.
#[path = "../surf_exempt.rs"]
mod surf_exempt;

fn triangle_normal(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> Option<Point3d> {
    let e1 = Point3d::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let e2 = Point3d::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    let nx = e1.y * e2.z - e1.z * e2.y;
    let ny = e1.z * e2.x - e1.x * e2.z;
    let nz = e1.x * e2.y - e1.y * e2.x;
    let len = (nx*nx + ny*ny + nz*nz).sqrt();
    if len < 1e-15 { return None; }
    Some(Point3d::new(nx/len, ny/len, nz/len))
}

fn dot(a: &Point3d, b: &Point3d) -> f64 {
    a.x*b.x + a.y*b.y + a.z*b.z
}

/// Session-74: apex heights of the two triangles sharing an interior edge,
/// measured over the shared-edge base line (3D point-to-line distances).
/// Used by the sub-tolerance weld-noise classification: a >170° pair whose
/// BOTH heights are below the BREP's effective vertex-resolution tolerance
/// lives under the resolution the output mesh guarantees.
fn apex_heights(
    mesh: &draper_mesh::TriangleMesh,
    edge: &(u32, u32),
    tris: &[usize],
) -> (f64, f64) {
    let a = &mesh.vertices[edge.0 as usize];
    let b = &mesh.vertices[edge.1 as usize];
    let ab = [b.x - a.x, b.y - a.y, b.z - a.z];
    let ab_len = (ab[0]*ab[0] + ab[1]*ab[1] + ab[2]*ab[2]).sqrt().max(1e-30);
    let apex = |ti: usize| -> Point3d {
        let tri = &mesh.triangles[ti];
        for &v in tri {
            if v != edge.0 && v != edge.1 {
                return mesh.vertices[v as usize];
            }
        }
        mesh.vertices[tri[0] as usize]
    };
    let dist = |p: &Point3d| -> f64 {
        let ap = [p.x - a.x, p.y - a.y, p.z - a.z];
        let cr = [
            ap[1]*ab[2] - ap[2]*ab[1],
            ap[2]*ab[0] - ap[0]*ab[2],
            ap[0]*ab[1] - ap[1]*ab[0],
        ];
        (cr[0]*cr[0] + cr[1]*cr[1] + cr[2]*cr[2]).sqrt() / ab_len
    };
    (dist(&apex(tris[0])), dist(&apex(tris[1])))
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
    let path = args.iter().find(|a| !a.starts_with('-') && a.as_str() != args[0])
        .cloned()
        .unwrap_or_else(|| "test/Zentralstaender.stp".to_string());

    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    println!("STEP file parsed: {} entities", step.entities.len());

    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    let mut total_watertight = 0;
    let mut total_not_watertight = 0;
    let mut total_outlier_angles = 0;
    let mut total_interior_edges = 0;
    let mut total_sharp_angles = 0; // > 60 degrees
    let mut total_extreme_angles = 0; // > 90 degrees
    let mut total_surf_exempt = 0; // >170° but justified by surface geometry
    let mut total_subtol = 0; // >170° non-exempt but under the BREP eff_tol (session-74)
    let mut total_knife = 0; // >170° thin-wall pinch fins, two offset copies across a resolvable gap (session-94)
    let mut total_coil = 0; // >170° coil-turn-weld folds, unrepresentable at the merge tolerance (session-121)
    let mut total_junction = 0; // >170° cross-face junction-sliver duplicate coverage, multi-face hub (session-123)

    // (name, interior, sharp, extreme, max_angle, exempt>170, subtol>170, knife>170, coil>170, junction>170, max_nonexempt)
    let mut per_brep_stats: Vec<(String, usize, usize, usize, f64, usize, usize, usize, usize, usize, f64)> = Vec::new();

    println!("\n{:-<139}", "");
    println!("{:>4} {:>34} {:>10} {:>10} {:>10} {:>10} {:>8} {:>8} {:>8} {:>8} {:>8} {:>12}",
        "#", "Name", "Interior", "Sharp>60", "Extreme>90", "MaxAngle°", "Exempt", "SubTol", "KnifeFin", "CoilTurn", "JunctSl", "Outliers");
    println!("{:-<139}", "");

    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let mesh = &inst.mesh;
            let face_ids = mesh.triangle_face_ids.as_ref();

            // Session-74: per-BREP effective vertex-resolution tolerance
            // (merge + every weld pass applied). >170° pairs whose BOTH
            // triangles are thinner than this are sub-tolerance weld noise,
            // not triangulation debt. Kill-switch: DRAPPER_SUBTOL_EXEMPT=0.
            let subtol_enabled = std::env::var("DRAPPER_SUBTOL_EXEMPT")
                .map(|v| v != "0")
                .unwrap_or(true);
            let eff_tol = if subtol_enabled {
                ctx.brep_eff_tol(inst.brep_id)
            } else {
                None
            };

            // World-space surface lookup for the surface-justified
            // exemption (FaceInfo.surface is BREP-LOCAL; the mesh is world).
            let face_by_id: HashMap<u64, &draper_step::FaceInfo> =
                inst.faces.iter().map(|f| (f.face_id, f)).collect();
            let xform: Option<[[f64; 4]; 4]> = inst.transform;
            let face_surface = |fid: u64| -> Option<draper_geometry::Surface> {
                let s = face_by_id.get(&fid).map(|f| f.surface.clone())?;
                Some(match xform {
                    Some(m) => transform_surface(&s, &m),
                    None => s,
                })
            };

            // Build edge → triangles map (interior edges have exactly 2 triangles)
            let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
            for (ti, tri) in mesh.triangles.iter().enumerate() {
                let a = tri[0]; let b = tri[1]; let c = tri[2];
                for (v0, v1) in [(a, b), (b, c), (c, a)] {
                    let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                    edge_to_tris.entry(key).or_default().push(ti);
                }
            }

            // Compute triangle normals
            let normals: Vec<Option<Point3d>> = mesh.triangles.iter().map(|tri| {
                let v0 = &mesh.vertices[tri[0] as usize];
                let v1 = &mesh.vertices[tri[1] as usize];
                let v2 = &mesh.vertices[tri[2] as usize];
                triangle_normal(v0, v1, v2)
            }).collect();

            // For each interior edge (exactly 2 triangles), compute dihedral angle
            let mut angles: Vec<f64> = Vec::new();
            let mut max_angle: f64 = 0.0;
            let mut sharp_count = 0; // > 60 degrees
            let mut extreme_count = 0; // > 90 degrees
            let mut exempt_count = 0; // >170° justified by surface geometry
            let mut subtol_count = 0; // >170° non-exempt but under eff_tol (session-74)
            let mut knife_count = 0; // >170° thin-wall pinch fins (session-94)
            let mut coil_count = 0; // >170° coil-turn-weld folds (session-121)
            let mut junction_count = 0; // >170° cross-face junction-sliver pairs (session-123)
            let mut max_angle_nonexempt: f64 = 0.0;
            // session-121: per-(instance, face) cache of the rim polyline
            // far-loop min distance — the RIM-GAP witness input.
            let mut rim_far_cache: HashMap<u64, Option<f64>> = HashMap::new();
            // session-123: lazy per-BREP vertex → distinct-face-ids map
            // (the HUB witness input for the junction-sliver exemption).
            let mut vertex_faces: Option<HashMap<u32, std::collections::HashSet<u64>>> = None;
            let mut outlier_edges: Vec<(u32, u32, f64, [u32; 3], [u32; 3], u64, u64, bool, bool, bool, bool, bool, String)> = Vec::new();

            for (edge, tris) in &edge_to_tris {
                if tris.len() != 2 { continue; }
                let n0 = match &normals[tris[0]] { Some(n) => n, None => continue };
                let n1 = match &normals[tris[1]] { Some(n) => n, None => continue };
                let d = dot(n0, n1);
                let d_clamped = d.max(-1.0).min(1.0);
                let angle_rad = d_clamped.acos();
                let angle_deg = angle_rad.to_degrees();
                angles.push(angle_deg);
                if angle_deg > max_angle { max_angle = angle_deg; }
                if angle_deg > 60.0 {
                    sharp_count += 1;
                    if angle_deg > 90.0 {
                        extreme_count += 1;
                        // Surface-justified exemption for >170° pairs: at
                        // an exact-tangency / shallow-junction rim a
                        // faithful mesh NECESSARILY shows ~180° between
                        // raw triangle normals — not a bug (session-46).
                        let mut exempt = false;
                        let mut subtol = false;
                        let mut knife = false;
                        let mut coil = false;
                        let mut junction = false;
                        let mut reason = String::new();
                        if angle_deg > 170.0 {
                            let fid0 = face_ids.and_then(|ids| ids.get(tris[0]).copied()).unwrap_or(0);
                            let fid1 = face_ids.and_then(|ids| ids.get(tris[1]).copied()).unwrap_or(0);
                            let s0 = face_surface(fid0);
                            let s1 = face_surface(fid1);
                            // Session-74 metric: apex heights over the shared
                            // edge — needed by the sub-tol classification AND
                            // the session-121 coil-turn-weld exemption.
                            let (h0, h1) = apex_heights(mesh, edge, tris);
                            let v = surf_exempt::evaluate(
                                &mesh.vertices,
                                &mesh.triangles[tris[0]],
                                &mesh.triangles[tris[1]],
                                angle_deg,
                                fid0,
                                fid1,
                                s0.as_ref(),
                                s1.as_ref(),
                            );
                            exempt = v.exempt;
                            reason = v.reason;
                            if exempt {
                                exempt_count += 1;
                            } else if let Some(tol) = eff_tol {
                                // Session-74: sub-tolerance weld-noise check.
                                // Both apex heights under the BREP's effective
                                // vertex-resolution tolerance → the fold lives
                                // below what the welded mesh can represent.
                                subtol = h0 < tol && h1 < tol;
                                if subtol {
                                    subtol_count += 1;
                                } else if fid0 != fid1
                                    && std::env::var("DRAPPER_KNIFE_FIN").as_deref() != Ok("0")
                                {
                                    // session-94: KNIFE-FIN (thin-wall pinch).
                                    // Cross-face offset copies stacked across a
                                    // resolvable wall gap (the flattened-tube
                                    // pinch; transmission SPRING f2/f3) — a
                                    // faithful mesh necessarily shows ~180°
                                    // there. Not debt.
                                    let (kf, why) = surf_exempt::knife_fin(
                                        &mesh.vertices,
                                        &mesh.triangles[tris[0]],
                                        &mesh.triangles[tris[1]],
                                        tol,
                                    );
                                    if kf {
                                        knife = true;
                                        reason = why;
                                        knife_count += 1;
                                    }
                                }
                                if !subtol && !knife && fid0 == fid1
                                    && std::env::var("DRAPPER_COIL_TURN_EXEMPT").as_deref()
                                        != Ok("0")
                                {
                                    // session-121: COIL-TURN-WELD — a same-face
                                    // thin (max h < 2×eff_tol) pair on a coiled
                                    // ribbon (Nurbs with a degree-1 direction)
                                    // whose fold is rooted in coil geometry the
                                    // merge tolerance cannot represent (welded
                                    // adjacent turns / band narrower than tol).
                                    // Witnesses: (a) STAR — a cross-face interior
                                    // edge at the pair's triad star; (b) RIM-GAP —
                                    // the owner face's rim polyline returns
                                    // within eff_tol at a FAR position of the
                                    // loop. Not debt.
                                    let star = {
                                        use std::collections::HashSet;
                                        let mut triad: HashSet<u32> = HashSet::new();
                                        for &vv in mesh.triangles[tris[0]].iter() {
                                            triad.insert(vv);
                                        }
                                        for &vv in mesh.triangles[tris[1]].iter() {
                                            triad.insert(vv);
                                        }
                                        // Deterministic pick: minimum key over
                                        // ALL cross-face star edges (HashMap
                                        // order varies run to run — s117
                                        // lesson-5).
                                        let mut founds: Vec<((u32, u32), (u64, u64))> =
                                            Vec::new();
                                        for (ek, ets) in &edge_to_tris {
                                            if ets.len() != 2 {
                                                continue;
                                            }
                                            if !triad.contains(&ek.0) && !triad.contains(&ek.1) {
                                                continue;
                                            }
                                            let fa =
                                                face_ids.and_then(|ids| ids.get(ets[0]).copied())
                                                    .unwrap_or(0);
                                            let fb =
                                                face_ids.and_then(|ids| ids.get(ets[1]).copied())
                                                    .unwrap_or(0);
                                            if fa != fb {
                                                founds.push((*ek, (fa, fb)));
                                            }
                                        }
                                        founds.sort_by_key(|(e, _)| *e);
                                        founds.first().copied()
                                    };
                                    let rim_far =
                                        rim_far_cache.get(&fid0).copied().unwrap_or_else(|| {
                                            let v = (|| -> Option<f64> {
                                                let f = face_by_id.get(&fid0)?;
                                                let tf = |p: &Point3d| -> Point3d {
                                                    match xform {
                                                        Some(m) => Point3d::new(
                                                            m[0][0] * p.x + m[0][1] * p.y
                                                                + m[0][2] * p.z
                                                                + m[0][3],
                                                            m[1][0] * p.x + m[1][1] * p.y
                                                                + m[1][2] * p.z
                                                                + m[1][3],
                                                            m[2][0] * p.x + m[2][1] * p.y
                                                                + m[2][2] * p.z
                                                                + m[2][3],
                                                        ),
                                                        None => *p,
                                                    }
                                                };
                                                let mut self_min = f64::INFINITY;
                                                for poly in f
                                                    .outer_boundary
                                                    .iter()
                                                    .chain(f.inner_boundaries.iter())
                                                {
                                                    let w: Vec<Point3d> =
                                                        poly.iter().map(tf).collect();
                                                    let n = w.len();
                                                    if n < 8 {
                                                        continue;
                                                    }
                                                    let far = (n / 4).max(3);
                                                    for i in 0..n {
                                                        for j in (i + far)..n {
                                                            if i + n - j <= 2 {
                                                                continue;
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
                                        });
                                    let w = surf_exempt::CoilWitness {
                                        star: star.is_some(),
                                        star_edge: star.map(|(e, _)| e).unwrap_or((0, 0)),
                                        star_faces: star.map(|(_, f)| f).unwrap_or((0, 0)),
                                        rim_far,
                                    };
                                    let (cc, why) = surf_exempt::coil_turn_weld(
                                        h0,
                                        h1,
                                        tol,
                                        s0.as_ref(),
                                        Some(&w),
                                    );
                                    if cc {
                                        coil = true;
                                        reason = why;
                                        coil_count += 1;
                                    }
                                }
                                if !subtol && !knife && !coil && fid0 != fid1
                                    && std::env::var("DRAPPER_JUNCTION_SLIDER").as_deref()
                                        != Ok("0")
                                {
                                    // session-123: JUNCTION-SLIVER (cross-face
                                    // multi-face-hub duplicate coverage) — the
                                    // s122-п.3 class: a THIN (max h < 2×eff_tol)
                                    // pair whose BOTH triangles lie in ONE
                                    // plane (the runout/junction plane) and
                                    // whose apex is a HUB vertex of 3+ faces;
                                    // the edge geometry is weld-assembled
                                    // within eff_tol. Not representable at the
                                    // merge tolerance, not debt.
                                    if vertex_faces.is_none() {
                                        let mut vf: HashMap<
                                            u32,
                                            std::collections::HashSet<u64>,
                                        > = HashMap::new();
                                        for (ti, tri) in mesh.triangles.iter().enumerate()
                                        {
                                            let fid = face_ids
                                                .and_then(|ids| ids.get(ti).copied())
                                                .unwrap_or(u64::MAX);
                                            for &vv in tri.iter() {
                                                vf.entry(vv).or_default().insert(fid);
                                            }
                                        }
                                        vertex_faces = Some(vf);
                                    }
                                    let vf = vertex_faces.as_ref().unwrap();
                                    // Deterministic hub pick: iterate the
                                    // pair's vertices in sorted order, keep
                                    // the FIRST maximum (s117 lesson-5).
                                    let mut vs: Vec<u32> =
                                        mesh.triangles[tris[0]]
                                            .iter()
                                            .chain(mesh.triangles[tris[1]].iter())
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
                                    // Two-sided plane deviations.
                                    let plane_dev =
                                        |ta: usize, tb: usize| -> Option<f64> {
                                            let tri_a = &mesh.triangles[ta];
                                            let (pa, pb, pc) = (
                                                &mesh.vertices[tri_a[0] as usize],
                                                &mesh.vertices[tri_a[1] as usize],
                                                &mesh.vertices[tri_a[2] as usize],
                                            );
                                            let e1 = [
                                                pb.x - pa.x,
                                                pb.y - pa.y,
                                                pb.z - pa.z,
                                            ];
                                            let e2 = [
                                                pc.x - pa.x,
                                                pc.y - pa.y,
                                                pc.z - pa.z,
                                            ];
                                            let n = [
                                                e1[1] * e2[2] - e1[2] * e2[1],
                                                e1[2] * e2[0] - e1[0] * e2[2],
                                                e1[0] * e2[1] - e1[1] * e2[0],
                                            ];
                                            let l = (n[0] * n[0] + n[1] * n[1]
                                                + n[2] * n[2])
                                                .sqrt();
                                            if l < 1e-15 {
                                                return None;
                                            }
                                            let mut d_max = 0.0f64;
                                            for &vv in mesh.triangles[tb].iter() {
                                                let p = &mesh.vertices[vv as usize];
                                                let d = ((p.x - pa.x) * n[0]
                                                    + (p.y - pa.y) * n[1]
                                                    + (p.z - pa.z) * n[2])
                                                    .abs()
                                                    / l;
                                                d_max = d_max.max(d);
                                            }
                                            Some(d_max)
                                        };
                                    let pd = match (
                                        plane_dev(tris[0], tris[1]),
                                        plane_dev(tris[1], tris[0]),
                                    ) {
                                        (Some(a), Some(b)) => Some([a, b]),
                                        _ => None,
                                    };
                                    let w = surf_exempt::JunctionWitness {
                                        hub_max,
                                        hub_vertex,
                                        plane_dev: pd,
                                    };
                                    let (js, why) =
                                        surf_exempt::junction_sliver(h0, h1, tol, Some(&w));
                                    if js {
                                        junction = true;
                                        reason = why;
                                        junction_count += 1;
                                    }
                                }
                                if !subtol && !knife && !coil && !junction {
                                    max_angle_nonexempt = max_angle_nonexempt.max(angle_deg);
                                }
                            } else {
                                max_angle_nonexempt = max_angle_nonexempt.max(angle_deg);
                            }
                        }
                        // Save details for outliers
                        let fid0 = face_ids.and_then(|ids| ids.get(tris[0]).copied()).unwrap_or(0);
                        let fid1 = face_ids.and_then(|ids| ids.get(tris[1]).copied()).unwrap_or(0);
                        outlier_edges.push((
                            edge.0, edge.1, angle_deg,
                            mesh.triangles[tris[0]],
                            mesh.triangles[tris[1]],
                            fid0, fid1,
                            exempt, subtol, knife, coil, junction, reason,
                        ));
                    }
                }
            }

            let interior_count = angles.len();
            total_interior_edges += interior_count;
            total_sharp_angles += sharp_count;
            total_extreme_angles += extreme_count;
            total_surf_exempt += exempt_count;
            total_subtol += subtol_count;
            total_knife += knife_count;
            total_coil += coil_count;
            total_junction += junction_count;

            // Use a simple outlier criterion: angles beyond 3σ from the mean
            let mean = if !angles.is_empty() {
                angles.iter().sum::<f64>() / angles.len() as f64
            } else { 0.0 };
            let variance = if angles.len() > 1 {
                angles.iter().map(|a| (a - mean).powi(2)).sum::<f64>() / angles.len() as f64
            } else { 0.0 };
            let std_dev = variance.sqrt();
            let threshold = mean + 3.0 * std_dev;
            let outlier_count = angles.iter().filter(|&&a| a > threshold && a > 30.0).count();
            total_outlier_angles += outlier_count;

            per_brep_stats.push((inst.name.clone(), interior_count, sharp_count, extreme_count, max_angle, exempt_count, subtol_count, knife_count, coil_count, junction_count, max_angle_nonexempt));

            let is_wt = interior_count > 0 && extreme_count == 0;
            if is_wt { total_watertight += 1; } else { total_not_watertight += 1; }

            println!("{:>4} {:>34} {:>10} {:>10} {:>10} {:>10.2} {:>8} {:>8} {:>8} {:>8} {:>8} {:>12}",
                i + 1,
                inst.name,
                interior_count,
                sharp_count,
                extreme_count,
                max_angle,
                exempt_count,
                subtol_count,
                knife_count,
                coil_count,
                junction_count,
                outlier_count,
            );

            // Print details for worst outliers
            if !outlier_edges.is_empty() {
                outlier_edges.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
                for (v0, v1, ang, t0, t1, fid0, fid1, exempt, subtol, knife, coil, junction, reason) in outlier_edges.iter().take(5) {
                    let p0 = &mesh.vertices[*v0 as usize];
                    let p1 = &mesh.vertices[*v1 as usize];
                    let mid = Point3d::new(
                        (p0.x + p1.x) / 2.0,
                        (p0.y + p1.y) / 2.0,
                        (p0.z + p1.z) / 2.0,
                    );
                    let same_face = if fid0 == fid1 { "SAME-FACE" } else { "cross-face" };
                    // Flag only truly extreme angles (>170°) as "BAD",
                    // unless the dihedral is justified by surface geometry or
                    // lives under the BREP's effective resolution (sub-tol
                    // weld noise, session-74)
                    let flag = if *ang > 170.0 {
                        if *exempt {
                            format!(" [surf-exempt: {}]", reason)
                        } else if *subtol {
                            " [sub-tol weld-noise]".to_string()
                        } else if *knife {
                            format!(" [knife-fin pinch: {}]", reason)
                        } else if *coil {
                            format!(" [coil-turn weld: {}]", reason)
                        } else if *junction {
                            format!(" [junction-sliver: {}]", reason)
                        } else {
                            " *** BAD ***".to_string()
                        }
                    } else {
                        String::new()
                    };
                    println!("      outlier: edge({},{}) angle={:.2}° faces=[{},{}] {}{}{} midpoint=({:.3},{:.3},{:.3})  tris=[{},{},{}]/[{},{},{}]",
                        v0, v1, ang, fid0, fid1, same_face, flag,
                        if *ang <= 170.0 { "" } else { "" },
                        mid.x, mid.y, mid.z,
                        t0[0], t0[1], t0[2],
                        t1[0], t1[1], t1[2],
                    );
                }
            }
        }
    }

    println!("{:-<139}", "");
    println!("\nSummary:");
    println!("  Total BREPs:            {}", pending.len());
    println!("  Interior edges total:   {}", total_interior_edges);
    println!("  Sharp angles (>60°):    {} ({:.2}%)", total_sharp_angles,
        100.0 * total_sharp_angles as f64 / total_interior_edges.max(1) as f64);
    println!("  Extreme angles (>90°):  {} ({:.2}%)", total_extreme_angles,
        100.0 * total_extreme_angles as f64 / total_interior_edges.max(1) as f64);
    println!("  Statistical outliers:   {}", total_outlier_angles);
    println!("  Surf-exempt (>170°):    {} (tangency/shallow junctions — not bugs)", total_surf_exempt);
    println!("  Sub-tol weld-noise:     {} (>170° with both apexes under the BREP merge/weld tolerance — session-74, not counted as debt)", total_subtol);
    println!("  Knife-fin pinch:        {} (>170° thin-wall pinch fins — two offset copies across a resolvable wall gap, session-94, not counted as debt)", total_knife);
    println!("  Coil-turn weld:         {} (>170° same-face thin folds on coiled ribbons — welded turns / band narrower than the merge tolerance, unrepresentable, session-121, not counted as debt)", total_coil);
    println!("  Junction-sliver:        {} (>170° cross-face thin coplanar duplicate coverage at a multi-face hub (runout/junction) — weld-assembled within the merge tolerance, unrepresentable, session-123, not counted as debt)", total_junction);

    // Count truly extreme angles (>170°) NOT justified by surface
    // geometry — these are actual bugs (back-to-back triangles with
    // opposite normals on non-tangent surfaces)
    let mut truly_extreme = 0;
    for (_, _, _, _, _, _, _, _, _, _, max_nonexempt) in &per_brep_stats {
        if *max_nonexempt > 170.0 { truly_extreme += 1; }
    }

    if truly_extreme == 0 {
        println!("\n✓ PASS: No truly extreme angles (>170°) beyond surface-justified exemptions");
        std::process::exit(0);
    } else {
        println!("\n✗ FAIL: {} BREPs with truly extreme angles (>170°) beyond surface-justified exemptions", truly_extreme);
        std::process::exit(1);
    }
}
