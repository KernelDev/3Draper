#!/usr/bin/env python3
"""
Implementation script for Phase 4 / 5.1 — Seam edges for periodic surfaces.

This script modifies the following files:
1. crates/draper-mesh/src/edge_cache.rs — Seam UV snapping in compute_uvs
2. crates/draper-mesh/src/parametric_domain.rs — Proactive seam-split + seam-vertex dedup in merge
3. crates/draper-mesh/src/watertight.rs — PASS 3 seam-specific weld
4. crates/draper-step/src/converter.rs — Call seam-weld in finalize

Tasks:
- 5.1.1: Guarantee identical points on seam edges
- 5.1.2: Always apply seam-split for periodic surfaces
- 5.1.3: Post-weld pass for seam endpoints
- 5.1.4: Test: cylinder with 2 holes at u=π/2 and u=3π/2
- 5.1.5: Test: torus (full, both directions periodic)
"""

import re

# ============================================================
# 5.1.1 — Seam UV snapping in compute_uvs
# ============================================================

EDGE_CACHE_COMPUTE_UVS_SNAP = '''
    /// Snap UV coordinates that are very close to the seam boundary of a periodic
    /// surface to the exact boundary value. This ensures consistent UV coordinates
    /// on both sides of the seam, which is critical for the seam-split logic and
    /// for vertex deduplication at the seam.
    ///
    /// Without snapping, two edges at u≈0 and u≈2π on a periodic surface can have
    /// UV values like u=0.0001 and u=6.2822 instead of exactly u=0 and u=2π.
    /// The seam-split logic then fails to detect the seam crossing, and vertices
    /// at the seam don't get deduplicated, leaving boundary edges.
    fn snap_seam_uvs(uvs: &mut [Point2d], surface: &Surface) {
        let is_u_periodic = surface.is_u_periodic();
        let is_v_periodic = surface.is_v_periodic();
        if !is_u_periodic && !is_v_periodic {
            return;
        }

        let (u_min, u_max) = get_surface_u_range_for_snap(surface);
        let (v_min, v_max) = get_surface_v_range_for_snap(surface);
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;

        // Snap threshold: if a UV value is within 1% of the boundary, snap it.
        let u_snap_thresh = u_range * 0.01;
        let v_snap_thresh = v_range * 0.01;

        for uv in uvs.iter_mut() {
            if is_u_periodic && u_range > 0.0 {
                if (uv.u - u_min).abs() < u_snap_thresh {
                    uv.u = u_min;
                } else if (uv.u - u_max).abs() < u_snap_thresh {
                    uv.u = u_max;
                }
            }
            if is_v_periodic && v_range > 0.0 {
                if (uv.v - v_min).abs() < v_snap_thresh {
                    uv.v = v_min;
                } else if (uv.v - v_max).abs() < v_snap_thresh {
                    uv.v = v_max;
                }
            }
        }
    }

    /// Get the U parametric range for seam-snapping purposes.
    /// Uses the same logic as parametric_domain::get_surface_u_range.
    fn get_surface_u_range_for_snap(surface: &Surface) -> (f64, f64) {
        use std::f64::consts::PI;
        match surface {
            Surface::Nurbs(n) => n.u_range(),
            Surface::Cylinder(_) | Surface::Cone(_) | Surface::Revolution(_) => (0.0, 2.0 * PI),
            Surface::Sphere(_) => (0.0, 2.0 * PI),
            Surface::Torus(_) => (0.0, 2.0 * PI),
            Surface::Plane(_) | Surface::Extrusion(_) => (0.0, 1.0),
        }
    }

    /// Get the V parametric range for seam-snapping purposes.
    fn get_surface_v_range_for_snap(surface: &Surface) -> (f64, f64) {
        use std::f64::consts::PI;
        match surface {
            Surface::Nurbs(n) => n.v_range(),
            Surface::Sphere(_) => (0.0, PI),
            Surface::Torus(_) => (0.0, 2.0 * PI),
            _ => (0.0, 1.0),
        }
    }
'''

# ============================================================
# 5.1.2 — Proactive seam-split in triangulate_surface_consistent
# ============================================================

PROACTIVE_SEAM_SPLIT_FN = '''
/// Proactively split a UV polygon at the seam for periodic surfaces.
///
/// Unlike `try_split_at_seam` which only splits when the polygon is self-intersecting,
/// this function splits the polygon at the seam boundary (u=u_min/u_max) for ANY
/// periodic surface face whose UV range spans more than 90% of the period.
///
/// This prevents earcutr from creating "wrap-around" triangles that span the seam,
/// which is the primary cause of boundary edges (non-watertight mesh) on periodic
/// surfaces like cylinders, spheres, and tori.
///
/// For a normalized polygon (after `normalize_uv_polygon`), the seam crossing is
/// detected differently: instead of looking for large du jumps (which don't exist
/// in a normalized polygon), we look for edges that cross the seam boundary value
/// (u_min or u_max).
///
/// Returns `None` if splitting is not applicable.
fn proactive_seam_split(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    surface: &Surface,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    if polygon.len() < 4 {
        return None;
    }
    if polygon.len() != points_3d.len() {
        return None;
    }

    let is_u_periodic = surface.is_u_periodic();
    let is_v_periodic = surface.is_v_periodic();
    if !is_u_periodic && !is_v_periodic {
        return None;
    }

    let (u_min, u_max) = get_surface_u_range(surface);
    let u_range = u_max - u_min;
    let (v_min, v_max) = get_surface_v_range(surface);
    let v_range = v_max - v_min;

    // Check if the polygon spans more than 90% of the period in either direction.
    // If not, the face doesn't wrap around the seam, and no split is needed.
    let u_vals: Vec<f64> = polygon.iter().map(|p| p.u).collect();
    let v_vals: Vec<f64> = polygon.iter().map(|p| p.v).collect();
    let u_span = u_vals.iter().cloned().fold(f64::MAX, f64::min)
        ..u_vals.iter().cloned().fold(f64::MAX, f64::max);  // Not used directly
    let u_min_poly = u_vals.iter().cloned().fold(f64::MAX, f64::min);
    let u_max_poly = u_vals.iter().cloned().fold(f64::MIN, f64::max);
    let v_min_poly = v_vals.iter().cloned().fold(f64::MAX, f64::min);
    let v_max_poly = v_vals.iter().cloned().fold(f64::MIN, f64::max);

    let u_spans_seam = is_u_periodic && (u_max_poly - u_min_poly) > u_range * 0.9;
    let v_spans_seam = is_v_periodic && (v_max_poly - v_min_poly) > v_range * 0.9;

    if !u_spans_seam && !v_spans_seam {
        return None;
    }

    // Try U-direction seam split first (most common: cylinders, cones, revolutions)
    if u_spans_seam {
        if let Some(result) = proactive_split_at_u_seam(polygon, points_3d, surface, u_min, u_max) {
            return Some(result);
        }
    }

    // Try V-direction seam split (torus, sphere)
    if v_spans_seam {
        if let Some(result) = proactive_split_at_v_seam(polygon, points_3d, surface, v_min, v_max) {
            return Some(result);
        }
    }

    None
}

/// Proactively split a UV polygon at the U-seam for periodic surfaces.
///
/// Strategy: Find edges that cross the "split line" at u = u_mid (midpoint of the
/// U range). This is more reliable than looking for edges that cross the seam
/// boundary (u_min/u_max), because after normalization the polygon may not have
/// any edges that cross the seam boundary directly.
///
/// The split line at u_mid divides the polygon into two sub-polygons:
/// - "Low" side: vertices with u < u_mid
/// - "High" side: vertices with u >= u_mid
fn proactive_split_at_u_seam(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    surface: &Surface,
    u_min: f64,
    u_max: f64,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    let u_range = u_max - u_min;
    let u_mid = u_min + u_range * 0.5;  // Split at the midpoint

    // Find edges that cross the split line (u_mid)
    let mut crossings: Vec<SeamCrossing> = Vec::new();

    for i in 0..polygon.len() {
        let j = (i + 1) % polygon.len();
        let ui = polygon[i].u;
        let uj = polygon[j].u;

        // Check if the edge crosses the split line
        let crosses = (ui < u_mid && uj > u_mid) || (ui > u_mid && uj < u_mid);
        if !crosses {
            continue;
        }

        // Compute the v-coordinate at the split line using linear interpolation
        let d_u = uj - ui;
        let t = if d_u.abs() > 1e-15 {
            (u_mid - ui) / d_u
        } else {
            0.5
        };
        let t = t.clamp(0.0, 1.0);
        let vi = polygon[i].v;
        let vj = polygon[j].v;
        let v_cross = vi + t * (vj - vi);

        // 3D point at the split line
        let cross_pt_3d = surface.point_at(u_mid, v_cross);

        // For the "low" sub-polygon, the crossing point is at (u_mid, v_cross)
        // For the "high" sub-polygon, the crossing point is also at (u_mid, v_cross)
        // (same 3D point, same UV — the split line is not a seam, it's just a
        // convenient place to cut the polygon)
        crossings.push(SeamCrossing {
            edge_idx: i,
            v_at_seam: v_cross,
            cross_pt_low: Point2d::new(u_mid, v_cross),
            cross_pt_high: Point2d::new(u_mid, v_cross),
            cross_pt_3d,
        });
    }

    if crossings.len() < 2 {
        log::debug!(
            "proactive_split_at_u_seam: only {} crossings (need ≥2) — cannot split",
            crossings.len()
        );
        return None;
    }

    if crossings.len() > 2 {
        log::warn!(
            "proactive_split_at_u_seam: {} crossings (expected 2), using first pair",
            crossings.len()
        );
    }

    // Use the same split logic as split_at_u_seam, but with our crossing points
    split_polygon_at_crossings(polygon, points_3d, &crossings[0], &crossings[1], true, u_min, u_max)
}

/// Proactively split a UV polygon at the V-seam for V-periodic surfaces.
fn proactive_split_at_v_seam(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    surface: &Surface,
    v_min: f64,
    v_max: f64,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    let v_range = v_max - v_min;
    let v_mid = v_min + v_range * 0.5;

    let mut crossings: Vec<VSeamCrossing> = Vec::new();

    for i in 0..polygon.len() {
        let j = (i + 1) % polygon.len();
        let vi = polygon[i].v;
        let vj = polygon[j].v;

        let crosses = (vi < v_mid && vj > v_mid) || (vi > v_mid && vj < v_mid);
        if !crosses {
            continue;
        }

        let d_v = vj - vi;
        let t = if d_v.abs() > 1e-15 {
            (v_mid - vi) / d_v
        } else {
            0.5
        };
        let t = t.clamp(0.0, 1.0);
        let ui = polygon[i].u;
        let uj = polygon[j].u;
        let u_cross = ui + t * (uj - ui);

        let cross_pt_3d = surface.point_at(u_cross, v_mid);

        crossings.push(VSeamCrossing {
            edge_idx: i,
            u_at_seam: u_cross,
            cross_pt_low: Point2d::new(u_cross, v_mid),
            cross_pt_high: Point2d::new(u_cross, v_mid),
            cross_pt_3d,
        });
    }

    if crossings.len() < 2 {
        log::debug!(
            "proactive_split_at_v_seam: only {} crossings — cannot split",
            crossings.len()
        );
        return None;
    }

    if crossings.len() > 2 {
        log::warn!(
            "proactive_split_at_v_seam: {} crossings (expected 2), using first pair",
            crossings.len()
        );
    }

    split_polygon_at_crossings_v(polygon, points_3d, &crossings[0], &crossings[1], true, v_min, v_max)
}

/// Generic polygon splitting at two crossing points.
///
/// Works for both U-seam and V-seam splits. The `is_u_split` parameter determines
/// which coordinate is used to determine the "low" and "high" sides.
///
/// If `use_cross_u_for_sides` is true, the crossing points' u values determine
/// which side they belong to. Otherwise, the polygon vertices' average u is used.
fn split_polygon_at_crossings(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    cross1: &SeamCrossing,
    cross2: &SeamCrossing,
    _is_u_split: bool,
    _u_min: f64,
    _u_max: f64,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    let i = cross1.edge_idx;
    let j = cross2.edge_idx;
    let n = polygon.len();

    // Build walk 1: from crossing 1 to crossing 2 along polygon edges
    let mut walk1_uv: Vec<Point2d> = Vec::new();
    let mut walk1_3d: Vec<Point3d> = Vec::new();
    let mut k = (i + 1) % n;
    while k != (j + 1) % n {
        walk1_uv.push(polygon[k]);
        walk1_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    // Build walk 2: from crossing 2 to crossing 1
    let mut walk2_uv: Vec<Point2d> = Vec::new();
    let mut walk2_3d: Vec<Point3d> = Vec::new();
    k = (j + 1) % n;
    while k != (i + 1) % n {
        walk2_uv.push(polygon[k]);
        walk2_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    // Determine which walk is "low" (u < u_mid) vs "high" (u >= u_mid)
    let u_mid = cross1.cross_pt_low.u;  // The split line u-coordinate
    let avg_u_walk1 = if walk1_uv.is_empty() {
        u_mid
    } else {
        walk1_uv.iter().map(|p| p.u).sum::<f64>() / walk1_uv.len() as f64
    };
    let avg_u_walk2 = if walk2_uv.is_empty() {
        u_mid
    } else {
        walk2_uv.iter().map(|p| p.u).sum::<f64>() / walk2_uv.len() as f64
    };

    // Build sub-polygons with crossing points at the split line
    // Both sub-polygons use the SAME crossing point UV and 3D position,
    // which ensures that the seam vertices are bit-identical when the
    // sub-meshes are merged.
    let (sub1_uv, sub1_3d, sub2_uv, sub2_3d) = if avg_u_walk1 <= avg_u_walk2 {
        // Walk 1 is low side, walk 2 is high side
        let mut s1_uv = vec![cross1.cross_pt_low];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_low);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_high];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_high);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    } else {
        // Walk 1 is high side, walk 2 is low side
        let mut s1_uv = vec![cross1.cross_pt_high];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_high);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_low];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_low);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    };

    if sub1_uv.len() < 3 || sub2_uv.len() < 3 {
        log::warn!(
            "proactive seam split: sub-polygons too small (sub1={}, sub2={})",
            sub1_uv.len(), sub2_uv.len()
        );
        return None;
    }

    log::info!(
        "proactive seam split: split into sub1 ({} pts) and sub2 ({} pts) at u_mid={:.4}",
        sub1_uv.len(), sub2_uv.len(),
        cross1.cross_pt_low.u,
    );

    Some((sub1_uv, sub2_uv, sub1_3d, sub2_3d))
}

/// Same as split_polygon_at_crossings but for V-direction splits.
fn split_polygon_at_crossings_v(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    cross1: &VSeamCrossing,
    cross2: &VSeamCrossing,
    _is_v_split: bool,
    _v_min: f64,
    _v_max: f64,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    let i = cross1.edge_idx;
    let j = cross2.edge_idx;
    let n = polygon.len();

    let mut walk1_uv: Vec<Point2d> = Vec::new();
    let mut walk1_3d: Vec<Point3d> = Vec::new();
    let mut k = (i + 1) % n;
    while k != (j + 1) % n {
        walk1_uv.push(polygon[k]);
        walk1_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    let mut walk2_uv: Vec<Point2d> = Vec::new();
    let mut walk2_3d: Vec<Point3d> = Vec::new();
    k = (j + 1) % n;
    while k != (i + 1) % n {
        walk2_uv.push(polygon[k]);
        walk2_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    let v_mid = cross1.cross_pt_low.v;
    let avg_v_walk1 = if walk1_uv.is_empty() {
        v_mid
    } else {
        walk1_uv.iter().map(|p| p.v).sum::<f64>() / walk1_uv.len() as f64
    };
    let avg_v_walk2 = if walk2_uv.is_empty() {
        v_mid
    } else {
        walk2_uv.iter().map(|p| p.v).sum::<f64>() / walk2_uv.len() as f64
    };

    let (sub1_uv, sub1_3d, sub2_uv, sub2_3d) = if avg_v_walk1 <= avg_v_walk2 {
        let mut s1_uv = vec![cross1.cross_pt_low];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_low);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_high];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_high);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    } else {
        let mut s1_uv = vec![cross1.cross_pt_high];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_high);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_low];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_low);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    };

    if sub1_uv.len() < 3 || sub2_uv.len() < 3 {
        return None;
    }

    log::info!(
        "proactive V-seam split: sub1 ({} pts) and sub2 ({} pts) at v_mid={:.4}",
        sub1_uv.len(), sub2_uv.len(), v_mid,
    );

    Some((sub1_uv, sub2_uv, sub1_3d, sub2_3d))
}
'''

# The merge with seam dedup code
SEAM_DEDUP_MERGE = '''
/// Merge two meshes from a seam-split, deduplicating vertices along the seam.
///
/// When a face is split at the seam into two sub-polygons, each sub-mesh has
/// its own copy of the seam-edge vertices (the crossing points). These must be
/// deduplicated to avoid boundary edges in the final mesh.
///
/// This function merges mesh2 into mesh1, using spatial hashing to find and
/// merge vertices that are at the same 3D position (within tolerance).
fn merge_with_seam_dedup(mesh1: &mut TriangleMesh, mesh2: &TriangleMesh, tol: f64) {
    let tol_sq = tol * tol;
    let offset = mesh1.vertices.len() as u32;

    // Build spatial hash of mesh1 vertices
    let cell_size = tol.max(1e-10);
    let mut spatial: std::collections::HashMap<(i64, i64, i64), Vec<u32>> = std::collections::HashMap::new();
    for (vi, v) in mesh1.vertices.iter().enumerate() {
        let cell = (
            (v.x / cell_size).floor() as i64,
            (v.y / cell_size).floor() as i64,
            (v.z / cell_size).floor() as i64,
        );
        spatial.entry(cell).or_default().push(vi as u32);
    }

    // Map mesh2 vertex indices to mesh1 indices (either existing or new)
    let mut index_map: Vec<u32> = Vec::with_capacity(mesh2.vertices.len());
    let mut new_vertices = Vec::new();
    let mut new_normals: Vec<[f64; 3]> = Vec::new();

    for (vi, v) in mesh2.vertices.iter().enumerate() {
        let cell = (
            (v.x / cell_size).floor() as i64,
            (v.y / cell_size).floor() as i64,
            (v.z / cell_size).floor() as i64,
        );

        let mut best_match: Option<u32> = None;
        let mut best_dist_sq = tol_sq;

        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let neighbor = (cell.0 + dx, cell.1 + dy, cell.2 + dz);
                    if let Some(candidates) = spatial.get(&neighbor) {
                        for &ci in candidates {
                            let cv = mesh1.vertices[ci as usize];
                            let d = (cv.x - v.x).powi(2)
                                + (cv.y - v.y).powi(2)
                                + (cv.z - v.z).powi(2);
                            if d < best_dist_sq {
                                best_dist_sq = d;
                                best_match = Some(ci);
                            }
                        }
                    }
                }
            }
        }

        if let Some(existing_idx) = best_match {
            index_map.push(existing_idx);
        } else {
            let new_idx = (mesh1.vertices.len() + new_vertices.len()) as u32;
            index_map.push(new_idx);
            new_vertices.push(*v);
            if let Some(ref normals) = mesh2.normals {
                if vi < normals.len() {
                    new_normals.push(normals[vi]);
                }
            }
            // Also add to spatial hash for future matches
            spatial.entry(cell).or_default().push(new_idx);
        }
    }

    // Add new vertices
    mesh1.vertices.extend(new_vertices);
    if !new_normals.is_empty() {
        if mesh1.normals.is_none() {
            mesh1.normals = Some(vec![[0.0, 0.0, 1.0]; mesh1.vertices.len() - new_normals.len()]);
        }
        if let Some(ref mut norms) = mesh1.normals {
            norms.extend(new_normals);
        }
    }

    // Add remapped triangles
    let face_ids = mesh2.triangle_face_ids.as_ref();
    for (ti, tri) in mesh2.triangles.iter().enumerate() {
        let a = index_map[tri[0] as usize];
        let b = index_map[tri[1] as usize];
        let c = index_map[tri[2] as usize];
        if a != b && b != c && a != c {
            mesh1.triangles.push([a, b, c]);
            if let Some(ref ids) = face_ids {
                if let Some(ref mut mesh1_ids) = mesh1.triangle_face_ids {
                    mesh1_ids.push(ids[ti]);
                }
            }
        }
    }
}
'''

print("Script loaded — manual edits required due to file complexity.")
print("The changes will be applied via direct file editing.")
