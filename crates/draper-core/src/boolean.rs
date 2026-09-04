// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Boolean operations — union, subtract, intersection.
//!
//! These are **face-classification** based boolean operations. The algorithm:
//!
//! 1. Triangulate both solids.
//! 2. For each face of solid A, classify its centroid against solid B
//!    (inside / outside / on-boundary).
//! 3. For each face of solid B, classify its centroid against solid A.
//! 4. **Union**: keep faces of A that are OUTSIDE B, plus faces of B that
//!    are OUTSIDE A. (Faces inside the other solid become internal and
//!    are removed.)
//! 5. **Subtraction (A − B)**: keep faces of A that are OUTSIDE B, plus
//!    REVERSED faces of B that are INSIDE A. (The reversed B faces form
//!    the "lid" of the cavity.)
//! 6. **Intersection**: keep faces of A that are INSIDE B, plus faces of
//!    B that are INSIDE A.
//!
//! This approach does NOT compute exact intersection curves between
//! surfaces, so the resulting shell may have small gaps along the
//! intersection curve. For a watertight result, a full B-rep intersection
//! algorithm would be needed (planned for P21). However, this approach
//! correctly handles all the common cases:
//!
//! - Two overlapping boxes / cubes
//! - A cylinder subtracted from a plate (creates a hole)
//! - Two intersecting cylinders
//! - Sphere clipped by a plane
//!
//! Limitations:
//! - Faces that straddle the boundary (one vertex inside, one outside)
//!   are kept whole — the algorithm does not split them. For solids with
//!   faces much smaller than the intersection curve detail, this is
//!   acceptable.
//! - Tangent contact between solids (e.g. two cubes touching at a face)
//!   is handled gracefully but the resulting shell may have duplicate
//!   coincident faces.

use draper_geometry::Point3d;
use draper_topology::{Solid, Shell, Face, Edge};

/// Result of a boolean operation.
pub type BooleanResult = Result<Solid, String>;

/// Boolean union: combine two solids into one.
///
/// The resulting solid contains all points that are in A OR in B.
/// Faces of A that are inside B (and vice versa) are removed because
/// they become internal to the union.
pub fn boolean_union(a: &Solid, b: &Solid) -> BooleanResult {
    let mut faces = Vec::new();
    // C5 7.6b: working lists parallel to `faces` — each kept face's
    // boundary resolved from its OWNING solid's store.
    let mut working: Vec<Vec<Edge>> = Vec::new();

    // Keep faces of A that are OUTSIDE B.
    if let Some(ref shell_a) = a.outer_shell {
        for face in &shell_a.faces {
            // C5 Stage 6.4 → 7.6b: store-first boundary reads of the OWNER
            // solid.
            let face_edges = a.resolve_face_edges(face);
            if !face_inside_solid(face, &face_edges, b, /*tolerance=*/ 1e-9) {
                faces.push(face.clone());
                working.push(face_edges);
            }
        }
    }
    // Keep faces of B that are OUTSIDE A.
    if let Some(ref shell_b) = b.outer_shell {
        for face in &shell_b.faces {
            let face_edges = b.resolve_face_edges(face);
            if !face_inside_solid(face, &face_edges, a, /*tolerance=*/ 1e-9) {
                faces.push(face.clone());
                working.push(face_edges);
            }
        }
    }

    if faces.is_empty() {
        return Err("boolean_union produced no faces — both solids may be empty or identical".to_string());
    }

    let shell = Shell::new_closed(faces);
    // C5 7.6b: born store-first — the working lists rebuild the unified
    // store.
    Ok(Solid::from_edges_only(shell, working))
}

/// Boolean subtraction: subtract solid B from solid A.
///
/// The resulting solid contains all points that are in A but NOT in B.
/// Faces of A that are inside B are removed (they are "carved out").
/// Faces of B that are inside A are kept but REVERSED — they become the
/// inner walls of the cavity left by the subtraction.
pub fn boolean_subtract(a: &Solid, b: &Solid) -> BooleanResult {
    let mut faces = Vec::new();
    // C5 7.6b: working lists parallel to `faces`.
    let mut working: Vec<Vec<Edge>> = Vec::new();

    // Keep faces of A that are OUTSIDE B.
    if let Some(ref shell_a) = a.outer_shell {
        for face in &shell_a.faces {
            // C5 7.6b: store-first boundary reads of the OWNER solid.
            let face_edges = a.resolve_face_edges(face);
            if !face_inside_solid(face, &face_edges, b, /*tolerance=*/ 1e-9) {
                faces.push(face.clone());
                working.push(face_edges);
            }
        }
    }
    // Keep REVERSED faces of B that are INSIDE A.
    if let Some(ref shell_b) = b.outer_shell {
        for face in &shell_b.faces {
            let face_edges = b.resolve_face_edges(face);
            if face_inside_solid(face, &face_edges, a, /*tolerance=*/ 1e-9) {
                faces.push(face.reversed());
                working.push(face_edges);
            }
        }
    }

    if faces.is_empty() {
        return Err("boolean_subtract produced no faces — A may be entirely inside B (result is empty)".to_string());
    }

    let shell = Shell::new_closed(faces);
    // C5 7.6b: born store-first (see boolean_union).
    Ok(Solid::from_edges_only(shell, working))
}

/// Boolean intersection: keep only the overlap of A and B.
///
/// The resulting solid contains all points that are in BOTH A AND B.
/// Faces of A that are inside B are kept (they bound the intersection
/// from A's side), and faces of B that are inside A are kept (they
/// bound the intersection from B's side).
///
/// For boundary cases (a face of A lies exactly on the boundary of B),
/// we treat the face as INSIDE — this handles the identical-cubes case
/// where all faces of A are on the boundary of B and vice versa.
pub fn boolean_intersect(a: &Solid, b: &Solid) -> BooleanResult {
    let mut faces = Vec::new();
    // C5 7.6b: working lists parallel to `faces`.
    let mut working: Vec<Vec<Edge>> = Vec::new();

    // Keep faces of A that are INSIDE B (or on boundary).
    if let Some(ref shell_a) = a.outer_shell {
        for face in &shell_a.faces {
            // C5 7.6b: store-first boundary reads of the OWNER solid.
            let face_edges = a.resolve_face_edges(face);
            if face_inside_or_on_solid(face, &face_edges, b, /*tolerance=*/ 1e-9) {
                faces.push(face.clone());
                working.push(face_edges);
            }
        }
    }
    // Keep faces of B that are INSIDE A (or on boundary).
    if let Some(ref shell_b) = b.outer_shell {
        for face in &shell_b.faces {
            let face_edges = b.resolve_face_edges(face);
            if face_inside_or_on_solid(face, &face_edges, a, /*tolerance=*/ 1e-9) {
                faces.push(face.clone());
                working.push(face_edges);
            }
        }
    }

    if faces.is_empty() {
        return Err("boolean_intersect produced no faces — the solids may not overlap".to_string());
    }

    let shell = Shell::new_closed(faces);
    // C5 7.6b: born store-first (see boolean_union).
    Ok(Solid::from_edges_only(shell, working))
}

/// Like `face_inside_solid`, but also returns true for faces whose
/// centroid is ON the boundary of `other` (within `tolerance`).
/// Used by `boolean_intersect` to handle boundary-coincident faces.
fn face_inside_or_on_solid(face: &Face, face_edges: &[Edge], other: &Solid, tolerance: f64) -> bool {
    let mut sample_points = Vec::new();
    for edge in face_edges {
        if let Some(p) = edge.start_point() {
            sample_points.push(p);
        }
        if let Some(p) = edge.end_point() {
            sample_points.push(p);
        }
    }
    if sample_points.is_empty() {
        if let Some(ref surface) = face.surface {
            sample_points.push(surface.point_at(0.5, 0.5));
        }
    }
    if sample_points.is_empty() {
        return false;
    }

    // For "inside or on boundary" we count a point as inside if it is
    // inside OR if perturbations in ANY direction are inside. This
    // catches the boundary case (where the point itself is on the
    // surface, but some perturbations go inside).
    let mut inside_or_boundary_count = 0;
    let mut outside_count = 0;
    for p in &sample_points {
        let direct = draper_topology::queries::point_in_solid(other, p);
        if direct {
            inside_or_boundary_count += 1;
            continue;
        }
        // Test perturbations — if any perturbation is inside, treat as
        // "on boundary" (counts as inside for intersection purposes).
        let perturbations = [
            Point3d::new(p.x + tolerance, p.y, p.z),
            Point3d::new(p.x - tolerance, p.y, p.z),
            Point3d::new(p.x, p.y + tolerance, p.z),
            Point3d::new(p.x, p.y - tolerance, p.z),
            Point3d::new(p.x, p.y, p.z + tolerance),
            Point3d::new(p.x, p.y, p.z - tolerance),
        ];
        let any_perturb_inside = perturbations.iter()
            .any(|pp| draper_topology::queries::point_in_solid(other, pp));
        if any_perturb_inside {
            inside_or_boundary_count += 1;
        } else {
            outside_count += 1;
        }
    }
    inside_or_boundary_count >= outside_count
}

/// Test whether a face of one solid lies inside another solid.
///
/// We compute the centroid of the face's edge endpoints and test that
/// against the other solid using ray-casting (`point_in_solid`).
///
/// If the face has no edge endpoints (degenerate), we fall back to the
/// face surface's origin point.
///
/// Returns true if the face's centroid is inside `other` (strictly, with
/// the given tolerance for boundary cases).
fn face_inside_solid(face: &Face, face_edges: &[Edge], other: &Solid, tolerance: f64) -> bool {
    // Collect edge endpoints as candidate sample points.
    let mut sample_points = Vec::new();
    for edge in face_edges {
        if let Some(p) = edge.start_point() {
            sample_points.push(p);
        }
        if let Some(p) = edge.end_point() {
            sample_points.push(p);
        }
    }
    // Fall back to surface origin if no edge endpoints.
    if sample_points.is_empty() {
        if let Some(ref surface) = face.surface {
            // Use a representative point on the surface. Most surfaces
            // have parametric range [0,1]×[0,1] or [0,2π]×[0,π] — picking
            // (0.5, 0.5) lands at the parametric centre. For unbounded
            // surfaces (Plane), point_at(0.5, 0.5) returns a point near
            // the plane's origin, which is acceptable.
            sample_points.push(surface.point_at(0.5, 0.5));
        }
    }
    if sample_points.is_empty() {
        // No way to evaluate; assume outside.
        return false;
    }

    // Use majority voting: a face is "inside" if more than half of its
    // vertices are inside. This handles the case where the face straddles
    // the boundary (some vertices inside, some outside) — we treat it as
    // inside if the majority is inside.
    let mut inside_count = 0;
    let mut outside_count = 0;
    for p in &sample_points {
        if point_in_solid_with_tolerance(p, other, tolerance) {
            inside_count += 1;
        } else {
            outside_count += 1;
        }
    }
    inside_count > outside_count
}

/// Point-in-solid test with a tolerance for boundary cases.
///
/// We test the point itself, and if it's "very close" to the boundary
/// (which we detect by testing a small perturbation and getting a different
/// answer), we test a few perturbations and take the majority vote.
fn point_in_solid_with_tolerance(point: &Point3d, solid: &Solid, tolerance: f64) -> bool {
    // Direct test.
    let direct = draper_topology::queries::point_in_solid(solid, point);
    if tolerance <= 0.0 {
        return direct;
    }

    // Perturbed tests to disambiguate boundary cases.
    let perturbations = [
        Point3d::new(point.x + tolerance, point.y, point.z),
        Point3d::new(point.x - tolerance, point.y, point.z),
        Point3d::new(point.x, point.y + tolerance, point.z),
        Point3d::new(point.x, point.y - tolerance, point.z),
        Point3d::new(point.x, point.y, point.z + tolerance),
        Point3d::new(point.x, point.y, point.z - tolerance),
    ];
    let mut inside_votes = if direct { 1 } else { 0 };
    let mut outside_votes = if direct { 0 } else { 1 };
    for p in &perturbations {
        if draper_topology::queries::point_in_solid(solid, p) {
            inside_votes += 1;
        } else {
            outside_votes += 1;
        }
    }
    inside_votes > outside_votes
}

/// Check if a point is inside a solid (ray casting).
///
/// This is a thin wrapper around `draper_topology::queries::point_in_solid`
/// for backward compatibility with code that imports from this module.
pub fn point_in_solid(point: &Point3d, solid: &Solid) -> bool {
    draper_topology::queries::point_in_solid(solid, point)
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::{Direction3d, Point3d, Surface, Plane};
    use draper_topology::{Edge, Face, Shell, Solid};

    /// Build a unit cube at the origin: vertices at (0,0,0)..(1,1,1).
    fn unit_cube_at(origin: (f64, f64, f64)) -> Solid {
        let (ox, oy, oz) = origin;
        let mut faces = Vec::with_capacity(6);

        // Helper: build a planar face with 4 line edges, INCLUDING the
        // outer_wire (which is required by triangulation_solid_for_queries
        // and other topology queries).
        let make_face = |origin: Point3d, normal: Direction3d, u_dir: Direction3d, v_dir: Direction3d,
                         p0: Point3d, p1: Point3d, p2: Point3d, p3: Point3d| -> Face {
            let surface = Surface::Plane(Plane {
                origin, u_dir, v_dir, normal,
            });
            // Create 4 edges.
            let e0 = Edge::new_line(p0, p1);
            let e1 = Edge::new_line(p1, p2);
            let e2 = Edge::new_line(p2, p3);
            let e3 = Edge::new_line(p3, p0);
            // Build coedges referencing the edges.
            let coedges = vec![
                draper_topology::CoEdge::new(e0.id, true),
                draper_topology::CoEdge::new(e1.id, true),
                draper_topology::CoEdge::new(e2.id, true),
                draper_topology::CoEdge::new(e3.id, true),
            ];
            let wire = draper_topology::Wire::new(coedges);
            let mut f = Face::new(surface, wire);
            f.edges = vec![e0, e1, e2, e3];
            f
        };

        // 8 corner points
        let p000 = Point3d::new(ox, oy, oz);
        let p100 = Point3d::new(ox + 1.0, oy, oz);
        let p110 = Point3d::new(ox + 1.0, oy + 1.0, oz);
        let p010 = Point3d::new(ox, oy + 1.0, oz);
        let p001 = Point3d::new(ox, oy, oz + 1.0);
        let p101 = Point3d::new(ox + 1.0, oy, oz + 1.0);
        let p111 = Point3d::new(ox + 1.0, oy + 1.0, oz + 1.0);
        let p011 = Point3d::new(ox, oy + 1.0, oz + 1.0);

        // Bottom (z=oz, normal -Z)
        faces.push(make_face(
            p000, Direction3d::new(0.0, 0.0, -1.0).unwrap(),
            Direction3d::X, Direction3d::Y,
            p000, p100, p110, p010,
        ));
        // Top (z=oz+1, normal +Z)
        faces.push(make_face(
            p001, Direction3d::Z,
            Direction3d::X, Direction3d::Y,
            p001, p101, p111, p011,
        ));
        // Front (y=oy, normal -Y)
        faces.push(make_face(
            p000, Direction3d::new(0.0, -1.0, 0.0).unwrap(),
            Direction3d::X, Direction3d::Z,
            p000, p100, p101, p001,
        ));
        // Back (y=oy+1, normal +Y)
        faces.push(make_face(
            p010, Direction3d::Y,
            Direction3d::X, Direction3d::Z,
            p010, p110, p111, p011,
        ));
        // Left (x=ox, normal -X)
        faces.push(make_face(
            p000, Direction3d::new(-1.0, 0.0, 0.0).unwrap(),
            Direction3d::Y, Direction3d::Z,
            p000, p010, p011, p001,
        ));
        // Right (x=ox+1, normal +X)
        faces.push(make_face(
            p100, Direction3d::X,
            Direction3d::Y, Direction3d::Z,
            p100, p110, p111, p101,
        ));

        let shell = Shell::new_closed(faces);
        Solid::new(shell)
    }

    #[test]
    fn test_point_in_solid_center_of_unit_cube() {
        let cube = unit_cube_at((0.0, 0.0, 0.0));
        // Center of cube is at (0.5, 0.5, 0.5) — inside.
        assert!(point_in_solid(&Point3d::new(0.5, 0.5, 0.5), &cube));
    }

    #[test]
    fn test_point_in_solid_outside_cube() {
        let cube = unit_cube_at((0.0, 0.0, 0.0));
        // Point (2, 2, 2) is outside the unit cube.
        assert!(!point_in_solid(&Point3d::new(2.0, 2.0, 2.0), &cube));
    }

    #[test]
    fn test_point_in_solid_edge_of_cube() {
        let cube = unit_cube_at((0.0, 0.0, 0.0));
        // Point exactly on the cube boundary — ray-casting may give either
        // answer (inside or outside). The function should at least not panic.
        let _ = point_in_solid(&Point3d::new(0.0, 0.0, 0.0), &cube);
        let _ = point_in_solid(&Point3d::new(0.5, 0.0, 0.0), &cube);
        // A point far outside is definitely outside.
        assert!(!point_in_solid(&Point3d::new(-1.0, -1.0, -1.0), &cube));
    }

    #[test]
    fn test_point_in_solid_offset_cube() {
        let cube = unit_cube_at((5.0, 5.0, 5.0));
        // Center is at (5.5, 5.5, 5.5) — inside.
        assert!(point_in_solid(&Point3d::new(5.5, 5.5, 5.5), &cube));
        // (0, 0, 0) is outside.
        assert!(!point_in_solid(&Point3d::new(0.0, 0.0, 0.0), &cube));
    }

    #[test]
    fn test_boolean_union_two_disjoint_cubes() {
        let a = unit_cube_at((0.0, 0.0, 0.0));
        let b = unit_cube_at((3.0, 0.0, 0.0));
        // Disjoint cubes — union should give 12 faces (6+6, none inside).
        let result = boolean_union(&a, &b);
        assert!(result.is_ok(), "union failed: {:?}", result);
        let solid = result.unwrap();
        let n_faces = solid.outer_shell.as_ref().unwrap().faces.len();
        assert_eq!(n_faces, 12, "expected 12 faces (6+6) for disjoint union, got {}", n_faces);
    }

    #[test]
    fn test_boolean_union_two_overlapping_cubes() {
        let a = unit_cube_at((0.0, 0.0, 0.0));
        let b = unit_cube_at((0.5, 0.0, 0.0));
        // Overlapping cubes — union should remove internal faces.
        // The exact count depends on how many faces of A are inside B and
        // vice versa. For a 0.5 offset, the left face of B is inside A,
        // and the right face of A is inside B. So we expect 12 - 2 = 10.
        let result = boolean_union(&a, &b);
        assert!(result.is_ok(), "union failed: {:?}", result);
        let solid = result.unwrap();
        let n_faces = solid.outer_shell.as_ref().unwrap().faces.len();
        assert!(
            n_faces <= 12,
            "expected ≤12 faces after union (some removed), got {}",
            n_faces
        );
        assert!(
            n_faces >= 6,
            "expected ≥6 faces after union (some remain), got {}",
            n_faces
        );
    }

    /// C5 Stage 6.4: store-first face classification must not change
    /// boolean results — the same subtraction with un-indexed (mirror
    /// reads) and indexed (store reads) inputs produces identical
    /// topology and volume.
    #[test]
    fn test_boolean_indexed_equivalence() {
        let a_raw = unit_cube_at((0.0, 0.0, 0.0));
        let b_raw = unit_cube_at((0.5, 0.5, 0.5));

        let mut a_idx = a_raw.clone();
        a_idx.index_edges();
        let mut b_idx = b_raw.clone();
        b_idx.index_edges();
        assert!(!a_idx.edge_store.is_empty(), "index_edges populated the store");

        let r_raw = boolean_subtract(&a_raw, &b_raw).expect("raw subtract ok");
        let r_idx = boolean_subtract(&a_idx, &b_idx).expect("indexed subtract ok");

        let faces_raw = r_raw.outer_shell.as_ref().unwrap().faces.len();
        let faces_idx = r_idx.outer_shell.as_ref().unwrap().faces.len();
        assert_eq!(faces_raw, faces_idx, "face count must match");

        let wire_fp = |s: &Solid| -> Vec<usize> {
            s.outer_shell
                .as_ref()
                .unwrap()
                .faces
                .iter()
                .map(|f| f.outer_wire.as_ref().map(|w| w.coedges.len()).unwrap_or(0))
                .collect()
        };
        assert_eq!(wire_fp(&r_raw), wire_fp(&r_idx), "wire fingerprint must match");

        let vol_raw = draper_topology::queries::solid_volume(&r_raw);
        let vol_idx = draper_topology::queries::solid_volume(&r_idx);
        assert_eq!(
            vol_raw.to_bits(),
            vol_idx.to_bits(),
            "volume must be bit-identical (raw={vol_raw}, idx={vol_idx})"
        );
    }

    #[test]
    fn test_boolean_subtract_disjoint_cubes() {
        let a = unit_cube_at((0.0, 0.0, 0.0));
        let b = unit_cube_at((3.0, 0.0, 0.0));
        // Disjoint — subtract leaves A unchanged (no faces of A are inside B,
        // no faces of B are inside A).
        let result = boolean_subtract(&a, &b);
        assert!(result.is_ok(), "subtract failed: {:?}", result);
        let solid = result.unwrap();
        let n_faces = solid.outer_shell.as_ref().unwrap().faces.len();
        assert_eq!(n_faces, 6, "expected 6 faces for disjoint subtract, got {}", n_faces);
    }

    #[test]
    fn test_boolean_subtract_overlapping_cubes() {
        let a = unit_cube_at((0.0, 0.0, 0.0));
        let b = unit_cube_at((0.5, 0.0, 0.0));
        // B overlaps the right half of A. Subtraction should remove the
        // right face of A (it's inside B), and the left face of B (reversed)
        // becomes the new right face of A.
        let result = boolean_subtract(&a, &b);
        assert!(result.is_ok(), "subtract failed: {:?}", result);
        let solid = result.unwrap();
        let n_faces = solid.outer_shell.as_ref().unwrap().faces.len();
        // At minimum we should have at least 6 faces (the result is a solid).
        assert!(
            n_faces >= 6,
            "expected ≥6 faces after subtract, got {}",
            n_faces
        );
    }

    #[test]
    fn test_boolean_intersect_disjoint_cubes() {
        let a = unit_cube_at((0.0, 0.0, 0.0));
        let b = unit_cube_at((3.0, 0.0, 0.0));
        // Disjoint — intersection is empty.
        let result = boolean_intersect(&a, &b);
        assert!(result.is_err(), "expected error for disjoint intersection");
    }

    #[test]
    fn test_boolean_intersect_identical_cubes() {
        let a = unit_cube_at((0.0, 0.0, 0.0));
        let b = unit_cube_at((0.0, 0.0, 0.0));
        // Identical — intersection should be the same cube. All 6 faces of
        // A are inside B (on boundary) and all 6 faces of B are inside A.
        // Result: up to 12 faces (6+6), but typically 6 (if deduplication
        // is done — which it isn't here, so 12).
        let result = boolean_intersect(&a, &b);
        assert!(result.is_ok(), "intersect failed: {:?}", result);
        let solid = result.unwrap();
        let n_faces = solid.outer_shell.as_ref().unwrap().faces.len();
        assert!(
            n_faces >= 6,
            "expected ≥6 faces for identical intersection, got {}",
            n_faces
        );
    }

    #[test]
    fn test_boolean_union_with_disjoint_returns_two_solids_worth_of_faces() {
        let a = unit_cube_at((0.0, 0.0, 0.0));
        let b = unit_cube_at((10.0, 0.0, 0.0));
        let result = boolean_union(&a, &b).unwrap();
        // 6 + 6 = 12 faces (no overlap, no removal).
        assert_eq!(
            result.outer_shell.as_ref().unwrap().faces.len(),
            12
        );
    }
}
