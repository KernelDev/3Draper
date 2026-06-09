// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Pre-processing for CDT: deduplication, vertex-to-edge snapping, intersection detection.

use super::CdtPoint;

const SENTINEL: u32 = u32::MAX;

/// Deduplicate points within tolerance using spatial hashing.
/// Returns (deduplicated points, remapping table: old_index → new_index).
pub fn deduplicate_points(points: &[CdtPoint], tolerance: f64) -> (Vec<CdtPoint>, Vec<u32>) {
    let n = points.len();
    if n == 0 {
        return (Vec::new(), Vec::new());
    }

    let tol_sq = tolerance * tolerance;
    let cell_size = tolerance.max(1e-15);
    
    // Spatial hash grid
    let mut grid: std::collections::HashMap<(i64, i64), Vec<usize>> = std::collections::HashMap::new();
    let mut remap: Vec<u32> = vec![SENTINEL; n];
    let mut dedup: Vec<CdtPoint> = Vec::new();

    for (i, p) in points.iter().enumerate() {
        let ci = (p.x / cell_size).floor() as i64;
        let cj = (p.y / cell_size).floor() as i64;

        // Check nearby cells for duplicate
        let mut found_dup = None;
        'outer: for di in -1i64..=1 {
            for dj in -1i64..=1 {
                let key = (ci + di, cj + dj);
                if let Some(indices) = grid.get(&key) {
                    for &j in indices {
                        let q = &dedup[j];
                        let dx = p.x - q.x;
                        let dy = p.y - q.y;
                        if dx * dx + dy * dy < tol_sq {
                            found_dup = Some(j);
                            break 'outer;
                        }
                    }
                }
            }
        }

        if let Some(j) = found_dup {
            remap[i] = j as u32;
        } else {
            let new_idx = dedup.len();
            dedup.push(*p);
            remap[i] = new_idx as u32;
            grid.entry((ci, cj)).or_default().push(new_idx);
        }
    }

    (dedup, remap)
}

/// Snap vertices that are within tolerance of a constraint edge.
/// Modifies points in-place (moving them onto the edge) and splits
/// constraint edges that have vertices snapped onto them.
///
/// This eliminates T-junctions by ensuring every vertex near a constraint
/// edge becomes a proper vertex ON that edge.
pub fn snap_vertices_to_constraints(
    points: &mut [CdtPoint],
    constraints: &mut Vec<[u32; 2]>,
    boundary_indices: &[u32],  // indices of boundary vertices (these are ON constraints, don't snap them)
    tolerance: f64,
) {
    let tol_sq = tolerance * tolerance;
    let boundary_set: std::collections::HashSet<u32> = boundary_indices.iter().copied().collect();
    
    // Build a set of constraint vertex indices
    let constraint_vertex_set: std::collections::HashSet<u32> = constraints
        .iter()
        .flat_map(|e| [e[0], e[1]])
        .collect();

    // For each non-constraint, non-boundary vertex, check distance to each constraint edge
    let mut new_constraints: Vec<[u32; 2]> = Vec::new();
    let mut constraints_to_remove: Vec<usize> = Vec::new();
    let mut new_point_index = points.len() as u32;

    // We'll collect snap events first, then apply them
    let mut snap_events: Vec<(u32, usize, f64)> = Vec::new(); // (vertex_idx, constraint_idx, t_param)

    for vi in 0..points.len() as u32 {
        if constraint_vertex_set.contains(&vi) || boundary_set.contains(&vi) {
            continue;
        }
        let px = points[vi as usize].x;
        let py = points[vi as usize].y;

        let mut best_dist_sq = tol_sq;
        let mut best_ci = None;
        let mut best_t = 0.0;

        for (ci, edge) in constraints.iter().enumerate() {
            let ax = points[edge[0] as usize].x;
            let ay = points[edge[0] as usize].y;
            let bx = points[edge[1] as usize].x;
            let by = points[edge[1] as usize].y;

            let dx = bx - ax;
            let dy = by - ay;
            let len_sq = dx * dx + dy * dy;
            if len_sq < 1e-30 {
                continue;
            }

            // Project P onto line AB
            let t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
            if t < 0.0 || t > 1.0 {
                continue; // Outside segment
            }

            // Distance from P to line at parameter t
            let proj_x = ax + t * dx;
            let proj_y = ay + t * dy;
            let dist_sq = (px - proj_x) * (px - proj_x) + (py - proj_y) * (py - proj_y);

            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_ci = Some(ci);
                best_t = t;
            }
        }

        if let Some(ci) = best_ci {
            snap_events.push((vi, ci, best_t));
        }
    }

    // Group snap events by constraint, sort by t parameter
    snap_events.sort_by(|a, b| a.1.cmp(&b.1).then(a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal)));

    // Apply snaps: for each constraint with snaps, split it
    let mut i = 0;
    while i < snap_events.len() {
        let ci = snap_events[i].1;
        let mut group: Vec<(u32, f64)> = Vec::new();
        while i < snap_events.len() && snap_events[i].1 == ci {
            group.push((snap_events[i].0, snap_events[i].2));
            i += 1;
        }

        if group.is_empty() {
            continue;
        }

        let edge = constraints[ci];
        let ax = points[edge[0] as usize].x;
        let ay = points[edge[0] as usize].y;
        let bx = points[edge[1] as usize].x;
        let by = points[edge[1] as usize].y;

        // Move snapped vertices onto the edge
        for &(vi, t) in &group {
            let proj_x = ax + t * (bx - ax);
            let proj_y = ay + t * (by - ay);
            points[vi as usize].x = proj_x;
            points[vi as usize].y = proj_y;
        }

        // Split the constraint edge into segments
        constraints_to_remove.push(ci);
        
        let mut chain = vec![edge[0]];
        for &(vi, _t) in &group {
            chain.push(vi);
        }
        chain.push(edge[1]);

        for j in 0..chain.len() - 1 {
            new_constraints.push([chain[j], chain[j + 1]]);
        }
    }

    // Remove old constraints and add new ones
    constraints_to_remove.sort();
    constraints_to_remove.dedup();
    constraints_to_remove.reverse();
    for ci in constraints_to_remove {
        constraints.remove(ci);
    }
    constraints.extend(new_constraints);
}

/// Find and resolve intersections between constraint edges.
/// When two constraints intersect at a non-shared vertex, the intersection
/// point is inserted and both constraints are split.
///
/// Returns newly created intersection points.
pub fn find_constraint_intersections(
    points: &mut Vec<CdtPoint>,
    constraints: &mut Vec<[u32; 2]>,
) -> Vec<u32> {
    let mut new_point_indices: Vec<u32> = Vec::new();
    let mut new_constraints: Vec<[u32; 2]> = Vec::new();
    let mut constraints_to_remove: Vec<usize> = Vec::new();

    let n = constraints.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let e1 = constraints[i];
            let e2 = constraints[j];

            // Skip if edges share an endpoint
            if e1[0] == e2[0] || e1[0] == e2[1] || e1[1] == e2[0] || e1[1] == e2[1] {
                continue;
            }

            let a1x = points[e1[0] as usize].x;
            let a1y = points[e1[0] as usize].y;
            let b1x = points[e1[1] as usize].x;
            let b1y = points[e1[1] as usize].y;
            let a2x = points[e2[0] as usize].x;
            let a2y = points[e2[0] as usize].y;
            let b2x = points[e2[1] as usize].x;
            let b2y = points[e2[1] as usize].y;

            if super::predicates::segments_intersect_proper(
                a1x, a1y, b1x, b1y, a2x, a2y, b2x, b2y,
            ) {
                let (ix, iy) = super::predicates::segment_intersection(
                    a1x, a1y, b1x, b1y, a2x, a2y, b2x, b2y,
                );

                let new_idx = points.len() as u32;
                points.push(CdtPoint {
                    x: ix,
                    y: iy,
                    original_index: SENTINEL,
                });
                new_point_indices.push(new_idx);

                // Split both constraints at the intersection
                constraints_to_remove.push(i);
                constraints_to_remove.push(j);
                new_constraints.push([e1[0], new_idx]);
                new_constraints.push([new_idx, e1[1]]);
                new_constraints.push([e2[0], new_idx]);
                new_constraints.push([new_idx, e2[1]]);
            }
        }
    }

    // Remove old constraints and add new ones
    constraints_to_remove.sort();
    constraints_to_remove.dedup();
    constraints_to_remove.reverse();
    for ci in constraints_to_remove {
        constraints.remove(ci);
    }
    constraints.extend(new_constraints);

    new_point_indices
}

/// Remap indices in boundary/hole arrays using a remapping table.
pub fn remap_indices(indices: &[u32], remap: &[u32]) -> Vec<u32> {
    indices
        .iter()
        .map(|&i| {
            if (i as usize) < remap.len() {
                remap[i as usize]
            } else {
                i
            }
        })
        .filter(|&i| i != SENTINEL)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dedup_identical() {
        let points = vec![
            CdtPoint { x: 0.0, y: 0.0, original_index: 0 },
            CdtPoint { x: 0.0, y: 0.0, original_index: 1 },
            CdtPoint { x: 1.0, y: 0.0, original_index: 2 },
        ];
        let (dedup, remap) = deduplicate_points(&points, 1e-10);
        assert_eq!(dedup.len(), 2, "Should merge identical points");
        assert_eq!(remap[0], remap[1], "Identical points should map to same index");
    }

    #[test]
    fn test_dedup_within_tolerance() {
        let points = vec![
            CdtPoint { x: 0.0, y: 0.0, original_index: 0 },
            CdtPoint { x: 1e-12, y: 1e-12, original_index: 1 },
        ];
        let (dedup, _remap) = deduplicate_points(&points, 1e-10);
        assert_eq!(dedup.len(), 1, "Points within tolerance should be merged");
    }

    #[test]
    fn test_dedup_no_dup() {
        let points = vec![
            CdtPoint { x: 0.0, y: 0.0, original_index: 0 },
            CdtPoint { x: 1.0, y: 0.0, original_index: 1 },
            CdtPoint { x: 0.0, y: 1.0, original_index: 2 },
        ];
        let (dedup, _remap) = deduplicate_points(&points, 1e-10);
        assert_eq!(dedup.len(), 3, "Distinct points should not be merged");
    }
}
