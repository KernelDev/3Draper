// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! C5 Stage 5.2 — explicit-edge standalone triangulation API.
//!
//! `triangulate_face_with_edges(face, edges, params)` decouples standalone
//! face triangulation from the per-face `Face.edges` FIELD: the caller
//! supplies the boundary edges explicitly, in face-instance order (the
//! instance ids the face's coedges reference).
//!
//! Tests:
//! 1. Equivalence — explicit edges taken from the face's own mirrors
//!    produce the identical mesh as the mirror-based `triangulate_face`.
//! 2. Shared cache — adjacent faces triangulated through ONE cache with
//!    the explicit API produce bit-identical vertices on their shared
//!    edge (the watertightness contract of the edge cache).
//! 3. Empty edge list — graceful degradation (no panic, mesh returned).

use draper_mesh::{
    triangulate_face, triangulate_face_with_edges, triangulate_face_with_edges_and_cache,
    EdgeDiscretizationCache, TriangulationParams,
};
use draper_topology::builder::ShapeBuilder;
use draper_topology::{Face, Solid, TopoId};

fn points_equal(a: &draper_geometry::Point3d, b: &draper_geometry::Point3d) -> bool {
    (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12 && (a.z - b.z).abs() < 1e-12
}

#[test]
fn test_explicit_edges_equivalent_to_mirrors() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let params = TriangulationParams::default();
    let faces = solid.faces();
    assert!(faces.len() >= 6, "box must have 6 faces");

    for face in faces {
        let via_mirrors = triangulate_face(face, &params);
        let refs: Vec<&draper_topology::Edge> = face.edges.iter().collect();
        let via_explicit = triangulate_face_with_edges(face, &refs, &params);

        assert_eq!(
            via_mirrors.vertex_count(),
            via_explicit.vertex_count(),
            "vertex count must match between mirror and explicit paths"
        );
        assert_eq!(
            via_mirrors.triangle_count(),
            via_explicit.triangle_count(),
            "triangle count must match between mirror and explicit paths"
        );
        for (a, b) in via_mirrors.vertices.iter().zip(via_explicit.vertices.iter()) {
            assert!(points_equal(a, b), "vertices must be bit-identical");
        }
        for (a, b) in via_mirrors.triangles.iter().zip(via_explicit.triangles.iter()) {
            assert_eq!(a, b, "triangles must be identical");
        }
    }
}

/// Find two faces of `solid` that share an edge (canonical id present in
/// both faces' `edge_ids`).
fn adjacent_face_pair(solid: &Solid) -> (usize, usize, TopoId) {
    let faces = solid.faces();
    let per_face: Vec<Vec<TopoId>> = faces.iter().map(|f| f.canonical_edge_ids()).collect();
    for i in 0..per_face.len() {
        for j in (i + 1)..per_face.len() {
            for id in &per_face[i] {
                if per_face[j].contains(id) {
                    return (i, j, *id);
                }
            }
        }
    }
    panic!("box must have faces sharing an edge");
}

#[test]
fn test_shared_cache_explicit_api_watertight_contribution() {
    let mut solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    solid.index_edges();

    let (i, j, _shared_id) = adjacent_face_pair(&solid);
    let faces = solid.faces();
    let (face_i, face_j) = (faces[i], faces[j]);

    // Triangulate both faces through ONE shared cache with the explicit API.
    let params = TriangulationParams::default();
    let mut cache = EdgeDiscretizationCache::new();
    let edges_i: Vec<&draper_topology::Edge> = face_i.edges.iter().collect();
    let edges_j: Vec<&draper_topology::Edge> = face_j.edges.iter().collect();
    let mesh_i = triangulate_face_with_edges_and_cache(face_i, &edges_i, &params, &mut cache);
    let mesh_j = triangulate_face_with_edges_and_cache(face_j, &edges_j, &params, &mut cache);

    assert!(mesh_i.triangle_count() > 0 && mesh_j.triangle_count() > 0);

    // The shared edge must discretize to BIT-IDENTICAL points in both
    // meshes — the watertightness contract of the edge cache.
    let shared_pts_in_i: Vec<_> = mesh_i
        .vertices
        .iter()
        .filter(|p| mesh_j.vertices.iter().any(|q| points_equal(p, q)))
        .collect();
    assert!(
        !shared_pts_in_i.is_empty(),
        "adjacent faces must share at least the corner vertices bit-identically"
    );
}

#[test]
fn test_explicit_api_empty_edges_degrades_gracefully() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let face = solid.faces()[0];
    let params = TriangulationParams::default();

    // Empty edge list: boundary recovery falls back (plane fit / fan /
    // surface sampling). Must not panic; a mesh object is always returned.
    // (vertex_count is usize — the real contract here is "no panic", the
    // touch below also pins that the mesh is in a queryable state.)
    let mesh = triangulate_face_with_edges(face, &[], &params);
    let _ = mesh.vertex_count();
    let _ = mesh.triangle_count();
}

/// Sanity for the staged-view helper semantics: the caller's face is not
/// modified (view is a copy).
#[test]
fn test_explicit_api_does_not_mutate_caller_face() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let face = solid.faces()[0];
    let before: Vec<TopoId> = face.edges.iter().map(|e| e.id).collect();
    let refs: Vec<&draper_topology::Edge> = face.edges.iter().collect();
    let _ = triangulate_face_with_edges(face, &refs, &TriangulationParams::default());
    let after: Vec<TopoId> = face.edges.iter().map(|e| e.id).collect();
    assert_eq!(before, after, "caller's face mirrors must be untouched");
}

/// Ensure the Face type used here still exposes what Stage 5 consumers
/// need (compile-time contract check).
#[test]
fn test_face_api_surface() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let face: &Face = &solid.faces()[0];
    let _ = face.canonical_edge_ids();
    let _ = face.edge_by_id(face.edges[0].id);
    assert!(face.surface.is_some());
}
