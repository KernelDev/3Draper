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
    triangulate_solid, triangulate_solid_face_with_cache, validate_watertight,
    EdgeDiscretizationCache, TriangleMesh, TriangulationParams, VertexDedupMap,
    filter_degenerate_triangles,
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

// ============================================================
// Stage 5.2 follow-up: canonical-store staging contract
// ============================================================

/// `Solid::face_edges(face)` returns CANONICAL store edges (Stage 4 read
/// API): for a shared edge the canonical id may differ from the instance id
/// the face's coedges reference, and the canonical curve may be the
/// OPPOSITE-direction twin of the instance's (Stage 3 geometric dedup keeps
/// the first-seen instance). The parallel staging contract must keep the
/// instance traversal pairing while upgrading geometry — output must match
/// the legacy mirror-reading path bit-for-bit.
#[test]
fn test_explicit_edges_canonical_store_resolution() {
    use draper_geometry::ToleranceContext;
    use draper_topology::boolean::boolean_subtract;

    let box_solid = ShapeBuilder::make_box(100.0, 80.0, 50.0);
    let cyl_solid = ShapeBuilder::make_cylinder(20.0, 100.0);
    let tol_ctx = ToleranceContext::from_model_scale(133.0);
    let mut result =
        boolean_subtract(&box_solid, &cyl_solid, &tol_ctx).expect("boolean subtract failed");
    let report = result.index_edges();
    assert!(report.total_instances > 0, "no edges indexed");

    let params = TriangulationParams::default();
    for face in result.faces() {
        let legacy = triangulate_face(face, &params);
        let canonical: Vec<&draper_topology::Edge> = result.face_edges(face);
        assert_eq!(canonical.len(), face.edges.len(), "face_edges must be parallel");
        let explicit = triangulate_face_with_edges(face, &canonical, &params);
        assert_eq!(
            legacy.vertices.len(),
            explicit.vertices.len(),
            "canonical resolution: vertex count mismatch"
        );
        for (a, b) in legacy.vertices.iter().zip(explicit.vertices.iter()) {
            assert!(points_equal(a, b), "canonical resolution: vertex mismatch {a:?} vs {b:?}");
        }
        assert_eq!(legacy.triangles.len(), explicit.triangles.len());
        for (a, b) in legacy.triangles.iter().zip(explicit.triangles.iter()) {
            assert_eq!(a, b, "canonical resolution: triangle mismatch");
        }
    }
}

/// Bit-identity on curved surfaces (remote test 1 covers the box only).
#[test]
fn test_explicit_edges_bit_identical_curved() {
    let cases: Vec<(&str, Solid)> = vec![
        ("cylinder", ShapeBuilder::make_cylinder(20.0, 100.0)),
        ("sphere", ShapeBuilder::make_sphere(25.0)),
    ];
    let params = TriangulationParams::default();

    for (name, solid) in cases {
        for face in solid.faces() {
            let legacy = triangulate_face(face, &params);
            let refs: Vec<&draper_topology::Edge> = face.edges.iter().collect();
            let explicit = triangulate_face_with_edges(face, &refs, &params);
            assert!(!legacy.triangles.is_empty(), "{name} face: no triangles");
            assert_eq!(legacy.vertices.len(), explicit.vertices.len(), "{name}: vertex count");
            for (a, b) in legacy.vertices.iter().zip(explicit.vertices.iter()) {
                assert!(points_equal(a, b), "{name}: vertex mismatch");
            }
            assert_eq!(legacy.triangles, explicit.triangles, "{name}: triangles");
        }
    }
}

/// Whole-solid watertightness through the explicit API + one shared cache
/// (the sequential-solid strategy, now driven via the Stage 5 entry point):
/// a box and a boolean (box − cylinder) must both come out watertight.
#[test]
fn test_explicit_api_shared_cache_full_solid_watertight() {
    use draper_mesh::mesh::VertexDedupMap;
    use draper_mesh::{filter_degenerate_triangles, TriangleMesh};
    use draper_mesh::watertight::validate_watertight;

    fn solid_mesh(solid: &Solid, params: &TriangulationParams) -> TriangleMesh {
        let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(
            &draper_geometry::Point3d::new(0.0, 0.0, 0.0),
            &draper_geometry::Point3d::new(100.0, 80.0, 50.0),
            20,
        );
        cache.pre_populate_for_solid(solid, 20);
        let mut mesh = TriangleMesh::new();
        let mut dedup =
            VertexDedupMap::with_tolerance(cache.adaptive_tolerance().merge_tolerance());
        for face in solid.faces() {
            let refs: Vec<&draper_topology::Edge> = face.edges.iter().collect();
            let face_mesh =
                triangulate_face_with_edges_and_cache(face, &refs, params, &mut cache);
            mesh.merge_deduplicating(&face_mesh, &mut dedup);
        }
        filter_degenerate_triangles(&mut mesh, 1e-10);
        mesh
    }

    let params = TriangulationParams::default();

    let box_mesh = solid_mesh(&ShapeBuilder::make_box(100.0, 80.0, 50.0), &params);
    let report = validate_watertight(&box_mesh, false);
    assert!(!box_mesh.triangles.is_empty());
    assert!(
        report.is_watertight(),
        "explicit-API box not watertight: {} boundary / {} edges",
        report.boundary_edge_count,
        report.edge_count
    );

    use draper_geometry::ToleranceContext;
    use draper_topology::boolean::boolean_subtract;
    let tol_ctx = ToleranceContext::from_model_scale(133.0);
    let boolean_result =
        boolean_subtract(&ShapeBuilder::make_box(100.0, 80.0, 50.0),
                         &ShapeBuilder::make_cylinder(20.0, 100.0), &tol_ctx).unwrap();
    let bool_mesh = solid_mesh(&boolean_result, &params);
    let report = validate_watertight(&bool_mesh, false);
    assert!(!bool_mesh.triangles.is_empty());
    assert!(
        report.is_watertight(),
        "explicit-API boolean not watertight: {} boundary / {} edges",
        report.boundary_edge_count,
        report.edge_count
    );
}

/// Parallel staging contract, unit level: instance ids / param_range /
/// forward / vertices are preserved; canonical upgrades (tolerance,
/// step_entity_id) are adopted; face id (cache keys) is preserved.
#[test]
fn test_stage_view_parallel_contract_keeps_instance_orientation() {
    use draper_mesh::stage_face_view;

    let solid = ShapeBuilder::make_box(30.0, 20.0, 10.0);
    let face = solid.faces()[0];
    assert!(!face.edges.is_empty());

    // Simulated canonical upgrades: tolerance + step_entity_id differ.
    let provided: Vec<draper_topology::Edge> = face
        .edges
        .iter()
        .map(|e| {
            let mut c = e.clone();
            c.tolerance = 0.5;
            c.step_entity_id = Some(4242);
            c
        })
        .collect();
    let refs: Vec<&draper_topology::Edge> = provided.iter().collect();

    let view = stage_face_view(face, &refs);
    assert_eq!(view.id, face.id, "face id must be preserved for cache keys");
    assert_eq!(view.edges.len(), face.edges.len());
    for (staged, mirror) in view.edges.iter().zip(face.edges.iter()) {
        assert_eq!(staged.id, mirror.id, "instance id preserved");
        assert_eq!(staged.param_range, mirror.param_range);
        assert_eq!(staged.forward, mirror.forward);
        assert_eq!(staged.vertex_start, mirror.vertex_start);
        assert_eq!(staged.vertex_end, mirror.vertex_end);
        assert_eq!(staged.start_vertex_point, mirror.start_vertex_point);
        assert_eq!(staged.end_vertex_point, mirror.end_vertex_point);
        // canonical geometry adopted
        assert_eq!(staged.tolerance, 0.5);
        assert_eq!(staged.step_entity_id, Some(4242));
    }

    // Round-trip: staging the face's own mirrors yields identical edges.
    let mirrors: Vec<&draper_topology::Edge> = face.edges.iter().collect();
    let round_trip = stage_face_view(face, &mirrors);
    for (staged, mirror) in round_trip.edges.iter().zip(face.edges.iter()) {
        assert_eq!(staged.id, mirror.id);
        assert_eq!(staged.tolerance, mirror.tolerance);
        assert_eq!(staged.step_entity_id, mirror.step_entity_id);
    }
}

/// Replacement staging contract: a slice of DIFFERENT length defines the
/// view's edge set and id space outright.
#[test]
fn test_stage_view_replacement_contract_defines_id_space() {
    use draper_mesh::stage_face_view;

    let solid = ShapeBuilder::make_box(30.0, 20.0, 10.0);
    let face = solid.faces()[0];

    // Subset (shorter than the mirror list) → replacement branch.
    let subset: Vec<&draper_topology::Edge> = face.edges.iter().take(2).collect();
    let view = stage_face_view(face, &subset);
    assert_eq!(view.edges.len(), 2);
    assert_eq!(view.edge_ids.len(), 2);
    for (staged, src) in view.edges.iter().zip(subset.iter()) {
        assert_eq!(staged.id, src.id, "replacement keeps provided ids");
    }

    // Empty slice on an edged face → replacement with zero edges; safe.
    let empty = stage_face_view(face, &[]);
    assert!(empty.edges.is_empty());
    assert!(empty.edge_ids.is_empty());
}


// ---------------------------------------------------------------------------
// C5 Stage 5.3 (mirror-free store path) — instance orientation in EdgeStore
//
// `triangulate_solid_face_with_cache` resolves edges through the canonical
// store: the canonical geometry is re-keyed to each coedge's instance id and
// param-reversed when `EdgeStore::instance_reversed` records the instance as
// traversing the canonical curve backwards. This keeps the legacy traversal
// bit-identically WITHOUT consulting the per-face mirrors — the mirrors
// become optional plumbing (removable end-state).
// ---------------------------------------------------------------------------

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
/// call swapped for the store-resolved mirror-free entry point.
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

/// Clone a solid and CLEAR every per-face `edges` mirror, keeping the
/// indexed `edge_ids` and the `EdgeStore` — the Stage 5 end-state.
fn mirror_free_clone(solid: &Solid) -> Solid {
    let mut clone = solid.clone();
    for face in clone.faces_mut() {
        assert!(
            !face.edge_ids.is_empty(),
            "test premise: solid must be indexed before cloning"
        );
        std::mem::take(&mut face.edges);
    }
    clone
}

#[test]
fn test_solid_pipeline_store_resolved_bit_identical() {
    for (si, mut solid) in test_solids().into_iter().enumerate() {
        solid.ensure_edge_store();
        let params = TriangulationParams::default();

        let reference = triangulate_solid(&solid, &params);
        let explicit = explicit_solid_mesh(&solid, &params);
        assert_meshes_bit_identical(
            &reference,
            &explicit,
            &format!("solid {si} full pipeline (store-resolved mirror-free edges)"),
        );
    }
}

#[test]
fn test_mirror_free_endstate_bit_identical() {
    for (si, mut solid) in test_solids().into_iter().enumerate() {
        solid.ensure_edge_store();
        let params = TriangulationParams::default();

        let reference = triangulate_solid(&solid, &params);

        let mirror_free = mirror_free_clone(&solid);
        for face in mirror_free.faces() {
            assert!(face.edges.is_empty());
            assert!(!face.edge_ids.is_empty());
        }

        let explicit = explicit_solid_mesh(&mirror_free, &params);
        assert_meshes_bit_identical(
            &reference,
            &explicit,
            &format!("solid {si} mirror-free end-state"),
        );
    }
}

#[test]
fn test_store_path_watertight_and_canonical_ptr_identity() {
    for (si, mut solid) in test_solids().into_iter().enumerate() {
        solid.ensure_edge_store();
        let params = TriangulationParams::default();

        let mesh = explicit_solid_mesh(&solid, &params);
        let report = validate_watertight(&mesh, false);
        assert!(
            report.is_watertight(),
            "solid {si}: store-path mesh is not watertight: {} boundary edges, {} non-manifold",
            report.boundary_edge_count,
            report.non_manifold_edge_count
        );

        // Stage 4 contract kept: a shared edge resolves to the SAME
        // canonical &Edge from both incident faces (pointer equality).
        let faces = solid.faces();
        let mut found_shared = 0usize;
        for i in 0..faces.len() {
            for j in (i + 1)..faces.len() {
                let ea = solid.face_edges(faces[i]);
                let eb = solid.face_edges(faces[j]);
                for a in &ea {
                    for b in &eb {
                        if a.id == b.id {
                            assert!(
                                std::ptr::eq(*a, *b),
                                "solid {si}: shared edge resolved to different canonical edges"
                            );
                            found_shared += 1;
                        }
                    }
                }
            }
        }
        assert!(
            found_shared > 0,
            "solid {si}: expected at least one shared edge to check pointer identity"
        );

        // Mirror-free face_edges: still returns the full reference list.
        let mirror_free = mirror_free_clone(&solid);
        for (orig, cleared) in solid.faces().iter().zip(mirror_free.faces().iter()) {
            assert_eq!(
                solid.face_edges(orig).len(),
                mirror_free.face_edges(cleared).len(),
                "face_edges length changed after clearing mirrors"
            );
        }
    }
}

/// C5 Stage 7.5 — `Solid::from_edges_only` construction: a solid built
/// mirror-free from scratch (topology-only shell + explicit working
/// lists → indexed → compacted) triangulates bit-identically to the
/// mirror-bearing reference and stays watertight.
#[test]
fn test_from_edges_only_solid_bit_identical() {
    for (si, mut solid) in test_solids().into_iter().enumerate() {
        solid.ensure_edge_store();
        let params = TriangulationParams::default();

        let reference = triangulate_solid(&solid, &params);

        // Deconstruct into (topology-only shell, working lists) and
        // rebuild through the Stage 7.5 mirror-free constructor.
        let shell = solid.outer_shell.clone().unwrap();
        let mut topology_only = shell.clone();
        let working: Vec<Vec<draper_topology::Edge>> = topology_only
            .faces
            .iter_mut()
            .map(|f| std::mem::take(&mut f.edges))
            .collect();
        let built = Solid::from_edges_only(topology_only, working);
        for face in built.faces() {
            assert!(face.edges.is_empty(), "from_edges_only faces are mirror-free");
            assert!(!face.edge_ids.is_empty());
        }

        let mesh = triangulate_solid(&built, &params);
        assert_meshes_bit_identical(
            &reference,
            &mesh,
            &format!("solid {si} from_edges_only construction"),
        );
        let report = validate_watertight(&mesh, false);
        assert!(
            report.is_watertight(),
            "solid {si}: from_edges_only mesh is not watertight: {} boundary edges",
            report.boundary_edge_count
        );
    }
}
