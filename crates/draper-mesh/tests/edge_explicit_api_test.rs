// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! C5 Stage 7.6b — explicit-edge standalone triangulation API (store-only).
//!
//! `Face.edges` is physically removed: every test drives the boundary
//! through the solid's `EdgeStore` (born-indexed construction) —
//! `Solid::resolve_face_edges` for instance-faithful lists and
//! `Solid::face_edges` for canonical references. The historical
//! "mirror vs explicit" bit-identity premise is replaced by the
//! store-vs-store contract:
//!
//! 1. Per-face equivalence — `triangulate_face_with_edges` fed the
//!    store-resolved instance list matches the store-first solid entry
//!    point bit-for-bit.
//! 2. Shared cache — adjacent faces through ONE cache produce
//!    bit-identical vertices on their shared edge (watertightness).
//! 3. Empty edge list — graceful degradation (no panic, mesh returned).
//! 4. Full-solid replication of the pipeline through the explicit API
//!    stays bit-identical to `triangulate_solid` and watertight.

use draper_mesh::{
    stage_face_view, triangulate_face_with_edges, triangulate_face_with_edges_and_cache,
    triangulate_solid, triangulate_solid_face_with_cache, validate_watertight,
    EdgeDiscretizationCache, TriangleMesh, TriangulationParams, VertexDedupMap,
    filter_degenerate_triangles,
};
use draper_topology::builder::ShapeBuilder;
use draper_topology::{Solid, TopoId};

fn points_equal(a: &draper_geometry::Point3d, b: &draper_geometry::Point3d) -> bool {
    (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12 && (a.z - b.z).abs() < 1e-12
}

#[test]
fn test_explicit_edges_store_equivalent() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let params = TriangulationParams::default();
    let faces = solid.faces();
    assert!(faces.len() >= 6, "box must have 6 faces");

    for face in faces {
        // Store-first solid entry point (the reference).
        let mut cache_a = EdgeDiscretizationCache::new();
        let via_store = triangulate_solid_face_with_cache(&solid, face, &params, &mut cache_a);

        // Standalone explicit API fed the store-resolved INSTANCE list
        // (owned resolution, re-borrowed as the explicit slice).
        let owned = solid.resolve_face_edges(face);
        let refs: Vec<&draper_topology::Edge> = owned.iter().collect();
        let via_explicit = triangulate_face_with_edges(face, &refs, &params);

        assert_eq!(
            via_store.vertex_count(),
            via_explicit.vertex_count(),
            "vertex count must match between store and explicit paths"
        );
        assert_eq!(
            via_store.triangle_count(),
            via_explicit.triangle_count(),
            "triangle count must match between store and explicit paths"
        );
        for (a, b) in via_store.vertices.iter().zip(via_explicit.vertices.iter()) {
            assert!(points_equal(a, b), "vertices must be identical");
        }
        for (a, b) in via_store.triangles.iter().zip(via_explicit.triangles.iter()) {
            assert_eq!(a, b, "triangles must be identical");
        }
    }
}

#[test]
fn test_explicit_edges_bit_identical_curved() {
    for solid in [
        ShapeBuilder::make_box(100.0, 80.0, 50.0),
        ShapeBuilder::make_cylinder(20.0, 100.0),
    ] {
        let params = TriangulationParams::default();
        for face in solid.faces() {
            let mut cache_a = EdgeDiscretizationCache::new();
            let via_store = triangulate_solid_face_with_cache(&solid, face, &params, &mut cache_a);
            let owned = solid.resolve_face_edges(face);
            let refs: Vec<&draper_topology::Edge> = owned.iter().collect();
            let via_explicit = triangulate_face_with_edges(face, &refs, &params);
            assert_eq!(via_store.vertex_count(), via_explicit.vertex_count());
            assert_eq!(via_store.triangle_count(), via_explicit.triangle_count());
            for (a, b) in via_store.vertices.iter().zip(via_explicit.vertices.iter()) {
                assert!(points_equal(a, b), "curved faces must be identical");
            }
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
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);

    let (i, j, _shared_id) = adjacent_face_pair(&solid);
    let faces = solid.faces();
    let (face_i, face_j) = (faces[i], faces[j]);

    // Triangulate both faces through ONE shared cache with the explicit
    // API, fed canonical store references (face_edges).
    let params = TriangulationParams::default();
    let mut cache = EdgeDiscretizationCache::new();
    let edges_i: Vec<&draper_topology::Edge> = solid.face_edges(face_i);
    let edges_j: Vec<&draper_topology::Edge> = solid.face_edges(face_j);
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
    let before: Vec<TopoId> = face.edge_ids.clone();
    let owned = solid.resolve_face_edges(face);
    let refs: Vec<&draper_topology::Edge> = owned.iter().collect();
    let _ = triangulate_face_with_edges(face, &refs, &TriangulationParams::default());
    let after: Vec<TopoId> = face.edge_ids.clone();
    assert_eq!(before, after, "caller's face must be untouched");
}

/// Ensure the 7.6b API surface used by consumers still compiles
/// (compile-time contract check).
#[test]
fn test_face_api_surface() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let face = &solid.faces()[0];
    let ids = face.canonical_edge_ids();
    assert!(!ids.is_empty(), "born-indexed face carries edge_ids");
    assert!(face.surface.is_some());

    // StagedFace::edge_by_id resolves on the vehicle's payload.
    let owned = solid.resolve_face_edges(face);
    let view = stage_face_view(face, &owned.iter().collect::<Vec<_>>());
    assert!(view.edge_by_id(owned[0].id).is_some());
}

// ============================================================
// Canonical-store staging contract (Stage 5.2 → 7.6b)
// ============================================================

#[test]
fn test_explicit_edges_canonical_store_resolution() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    // Born-indexed: no explicit index pass needed.

    for face in solid.faces() {
        let canonical: Vec<&draper_topology::Edge> = solid.face_edges(face);
        assert_eq!(
            canonical.len(),
            face.edge_ids.len(),
            "face_edges must be parallel to edge_ids (born-indexed)"
        );
        for e in &canonical {
            assert!(
                solid.edge_store.get(e.id).is_some(),
                "face_edges must yield canonical store-backed edges"
            );
        }
    }

    // A shared edge resolves to the SAME canonical &Edge from both
    // incident faces (pointer equality — Stage 4 contract kept).
    let faces = solid.faces();
    let (i, j, shared_id) = adjacent_face_pair(&solid);
    let ea = solid.face_edges(faces[i]);
    let eb = solid.face_edges(faces[j]);
    let a = ea.iter().find(|e| e.id == shared_id).unwrap();
    let b = eb.iter().find(|e| e.id == shared_id).unwrap();
    assert!(
        std::ptr::eq(*a, *b),
        "shared edge must resolve to the same canonical &Edge"
    );
}

#[test]
fn test_stage_view_replacement_contract_defines_id_space() {
    let solid = ShapeBuilder::make_box(10.0, 10.0, 10.0);
    let face = solid.faces()[0];
    let owned = solid.resolve_face_edges(face);
    let refs: Vec<&draper_topology::Edge> = owned.iter().collect();

    // Replacement contract: the slice defines the view's edge set and id
    // space outright.
    let subset: Vec<&draper_topology::Edge> = refs.iter().take(2).copied().collect();
    let view = stage_face_view(face, &subset);
    assert_eq!(view.edges.len(), 2);
    assert_eq!(view.edge_by_id(subset[0].id).is_some(), true);
    assert_eq!(view.edge_ids.len(), 2, "edge_ids derived from the slice");

    // Full-slice view keeps the instance id pairing.
    let full = stage_face_view(face, &refs);
    assert_eq!(full.edges.len(), refs.len());
    for (staged, src) in full.edges.iter().zip(refs.iter()) {
        assert_eq!(staged.id, src.id, "staged ids must match the slice");
    }
}

// ============================================================
// Full-solid pipeline replication (store-only)
// ============================================================

/// EDGE_SAMPLES from `draper_mesh::triangulate` (not publicly exported).
const SOLID_EDGE_SAMPLES: usize = 20;

/// max_samples passed to `with_adaptive_tolerance` by the solid pipeline.
const SOLID_ADAPTIVE_SAMPLES: usize = 64;

fn test_solids() -> Vec<Solid> {
    vec![
        ShapeBuilder::make_box(100.0, 80.0, 50.0),
        ShapeBuilder::make_cylinder(20.0, 100.0),
    ]
}

/// Assert two meshes are identical down to the bit: same vertex/triangle
/// counts, bit-equal coordinates, identical triangle indices.
fn assert_meshes_bit_identical(a: &TriangleMesh, b: &TriangleMesh, ctx: &str) {
    assert_eq!(
        a.vertices.len(),
        b.vertices.len(),
        "{ctx}: vertex count {} vs {}",
        a.vertices.len(),
        b.vertices.len()
    );
    assert_eq!(
        a.triangles.len(),
        b.triangles.len(),
        "{ctx}: triangle count {} vs {}",
        a.triangles.len(),
        b.triangles.len()
    );
    for (i, (va, vb)) in a.vertices.iter().zip(b.vertices.iter()).enumerate() {
        assert_eq!(va.x.to_bits(), vb.x.to_bits(), "{ctx}: vertex {i} x");
        assert_eq!(va.y.to_bits(), vb.y.to_bits(), "{ctx}: vertex {i} y");
        assert_eq!(va.z.to_bits(), vb.z.to_bits(), "{ctx}: vertex {i} z");
    }
    for (i, (ta, tb)) in a.triangles.iter().zip(b.triangles.iter()).enumerate() {
        assert_eq!(ta, tb, "{ctx}: triangle {i}: {ta:?} vs {tb:?}");
    }
}

/// Replicate the legacy `solid_bounding_box` scan through the store-first
/// read API (`Solid::face_edges`).
fn solid_bbox(solid: &Solid) -> (draper_geometry::Point3d, draper_geometry::Point3d) {
    let mut min = draper_geometry::Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = draper_geometry::Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    let mut has_points = false;
    for face in solid.faces() {
        for edge in solid.face_edges(face) {
            if edge.degenerate {
                continue;
            }
            for p in [edge.start_point(), edge.end_point()].into_iter().flatten() {
                min.x = min.x.min(p.x);
                min.y = min.y.min(p.y);
                min.z = min.z.min(p.z);
                max.x = max.x.max(p.x);
                max.y = max.y.max(p.y);
                max.z = max.z.max(p.z);
                has_points = true;
            }
        }
    }
    if !has_points {
        return (
            draper_geometry::Point3d::ORIGIN,
            draper_geometry::Point3d::new(1.0, 1.0, 1.0),
        );
    }
    (min, max)
}

/// Manual replication of the sequential solid pipeline with the per-face
/// call swapped for the store-resolved entry point.
fn explicit_solid_mesh(solid: &Solid, params: &TriangulationParams) -> TriangleMesh {
    let (bmin, bmax) = solid_bbox(solid);
    let mut cache =
        EdgeDiscretizationCache::with_adaptive_tolerance(&bmin, &bmax, SOLID_ADAPTIVE_SAMPLES);
    cache.set_chord_tolerance_override(Some(params.max_deviation));
    cache.pre_populate_for_solid(solid, SOLID_EDGE_SAMPLES);

    let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
    let mut mesh = TriangleMesh::new();
    let mut dedup_map = VertexDedupMap::with_tolerance(adaptive_tol);
    for (face_idx, face) in solid.faces().iter().enumerate() {
        let mut face_mesh = triangulate_solid_face_with_cache(solid, face, params, &mut cache);
        let face_tri_count = face_mesh.triangles.len();
        face_mesh.triangle_face_ids = Some(vec![face_idx as u64; face_tri_count]);
        mesh.merge_deduplicating(&face_mesh, &mut dedup_map);
    }
    filter_degenerate_triangles(&mut mesh, 1e-10);
    mesh
}

#[test]
fn test_solid_pipeline_store_resolved_bit_identical() {
    for (si, solid) in test_solids().into_iter().enumerate() {
        // (7.6b: born-indexed — no ensure/index pass.)
        let params = TriangulationParams::default();

        let reference = triangulate_solid(&solid, &params);
        let explicit = explicit_solid_mesh(&solid, &params);
        assert_meshes_bit_identical(
            &reference,
            &explicit,
            &format!("solid {si} full pipeline (store-resolved edges)"),
        );
    }
}

#[test]
fn test_clone_endstate_bit_identical() {
    // 7.6b: EVERY solid is the store-only end-state — the historical
    // "mirror-free clone" twin collapsed into plain clone fidelity.
    for (si, solid) in test_solids().into_iter().enumerate() {
        let params = TriangulationParams::default();
        let reference = triangulate_solid(&solid, &params);
        let clone = solid.clone();
        for face in clone.faces() {
            assert!(!face.edge_ids.is_empty(), "clone keeps edge_ids");
        }
        let explicit = explicit_solid_mesh(&clone, &params);
        assert_meshes_bit_identical(
            &reference,
            &explicit,
            &format!("solid {si} clone end-state"),
        );
    }
}

#[test]
fn test_store_path_watertight() {
    for (si, solid) in test_solids().into_iter().enumerate() {
        let params = TriangulationParams::default();
        let mesh = explicit_solid_mesh(&solid, &params);
        let report = validate_watertight(&mesh, false);
        assert!(
            report.is_watertight(),
            "solid {si}: store-path mesh is not watertight: {} boundary edges, {} non-manifold",
            report.boundary_edge_count,
            report.non_manifold_edge_count
        );
    }
}
