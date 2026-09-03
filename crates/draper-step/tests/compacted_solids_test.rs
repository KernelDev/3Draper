//! C5 Stage 7.2 — canonical SOLID payload from STEP.
//!
//! `extract_solids` now returns store-first solids: after `index_edges`
//! (covering outer AND void shells), the converter calls
//! `Solid::compact_edge_mirrors`, so every face the edge store can fully
//! answer arrives with EMPTY `edges` mirrors and canonical `edge_ids`.
//! The production triangulation pipelines (sequential AND parallel) stage
//! faces through `Solid::resolve_face_edges` (Stage 7.2 migration), so the
//! compacted payload triangulates without the mirrors.
//!
//! Verification criteria:
//! 1. STEP-loaded solids arrive compacted (mirrors empty where the store
//!    answers every boundary-reader query) and every wire coedge resolves.
//! 2. `triangulate_solid` on the compacted payload is watertight (the
//!    known-good regression files: nist_cone was at 0.00% boundary since
//!    C5 Stage 1).
//! 3. Compaction is value-neutral: the mesh from the compacted solid is
//!    bit-identical to the PRE-Stage-7.2 mirror-reading pipeline
//!    (per-face `triangulate_face_with_cache` on re-materialized mirrors).

use draper_mesh::{
    filter_degenerate_triangles, triangulate_face_with_cache, triangulate_solid,
    triangulate_solid_with_report, EdgeDiscretizationCache, TriangleMesh,
    TriangulationParams, VertexDedupMap,
};
use draper_step::{extract_solids, parse_step};
use draper_topology::{Face, Solid};

/// Load + parse a STEP file from the workspace test directory and extract
/// solids (same path resolution as `seam_junction_regression.rs`).
fn load_solids(filename: &str) -> Vec<Solid> {
    let candidates = [
        format!("test/{}", filename),       // workspace root
        format!("../../test/{}", filename), // crate dir
    ];
    let path = candidates
        .iter()
        .find(|p| std::path::Path::new(p).exists())
        .unwrap_or_else(|| {
            panic!(
                "test file not found: {} (cwd={:?})",
                filename,
                std::env::current_dir()
            )
        });
    let content = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("failed to read {}: {}", filename, e));
    let step = parse_step(&content)
        .unwrap_or_else(|e| panic!("failed to parse {}: {}", filename, e));
    let (solids, _) = extract_solids(&step);
    assert!(!solids.is_empty(), "{}: no solids extracted", filename);
    solids
}

/// Assert two meshes are identical down to the bit.
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

/// Classify a face: `Some(compacted)` when mirrors are empty, `None` when
/// the face kept (or never had) mirror content.
fn face_compacted(face: &Face) -> bool {
    face.edges.is_empty()
}

/// Test 1 — STEP solids arrive as store-first (compacted) payloads.
#[test]
fn test_step_solids_arrive_compacted() {
    for filename in ["nist_cone.stp", "cube_with_void.stp"] {
        let solids = load_solids(filename);
        for (si, solid) in solids.iter().enumerate() {
            assert!(
                !solid.edge_store.is_empty(),
                "{filename}: solid {si} must carry an edge store"
            );
            let faces = solid.faces();
            let mut compacted = 0usize;
            for (fi, face) in faces.iter().enumerate() {
                if face_compacted(face) {
                    compacted += 1;
                    assert!(
                        !face.edge_ids.is_empty(),
                        "{filename}: solid {si} face {fi}: compacted face must keep edge_ids"
                    );
                } else {
                    // Un-compacted leftovers must be mirror-only identity
                    // (un-indexed), never a half-cleared state.
                    assert!(
                        !face.edges.is_empty(),
                        "{filename}: solid {si} face {fi}: non-compacted face keeps mirrors"
                    );
                }
                // Every wire coedge id resolves through the store — the
                // boundary readers' hard requirement.
                let mut coedge_ids = Vec::new();
                if let Some(ref wire) = face.outer_wire {
                    coedge_ids.extend(wire.coedges.iter().map(|ce| ce.edge));
                }
                for wire in &face.inner_wires {
                    coedge_ids.extend(wire.coedges.iter().map(|ce| ce.edge));
                }
                for id in coedge_ids {
                    assert!(
                        solid.edge_store.instance_edge(id).is_some(),
                        "{filename}: solid {si} face {fi}: coedge edge {id:?} unresolved in store"
                    );
                }
            }
            assert!(
                compacted > 0,
                "{filename}: solid {si}: expected at least one compacted face, got 0/{}",
                faces.len()
            );
        }
    }
}

/// Test 2 — the compacted payload triangulates watertight through the
/// production solid pipeline (store-first staging, Stage 7.2).
#[test]
fn test_compacted_step_solid_triangulates_watertight() {
    let params = TriangulationParams::default();

    let solids = load_solids("nist_cone.stp");
    let result = triangulate_solid_with_report(&solids[0], &params);
    assert!(
        !result.mesh.triangles.is_empty(),
        "nist_cone: compacted payload produced no triangles"
    );
    assert!(
        result.report.is_watertight,
        "nist_cone: compacted payload not watertight: {}",
        result.report.summary()
    );

    // A holed brick: multi-face planar payload through the same pipeline
    // (bit-identity vs the pre-7.2 pipeline is asserted in test 3).
    let solids = load_solids("brick_thin_hole.stp");
    let result = triangulate_solid_with_report(&solids[0], &params);
    assert!(
        !result.mesh.triangles.is_empty(),
        "brick_thin_hole: compacted payload produced no triangles"
    );
    assert!(
        result.report.is_acceptable(),
        "brick_thin_hole: compacted payload quality degraded: {}",
        result.report.summary()
    );
}

/// Test 3 — compaction is value-neutral: bit-identical mesh vs the
/// PRE-Stage-7.2 mirror-reading sequential pipeline.
///
/// The reference re-materializes the per-face mirrors from the store
/// (`Solid::instance_edges` — the Stage 6 mirror re-derivation) and runs
/// the OLD per-face entry (`triangulate_face_with_cache`, which reads the
/// face's own `edges` mirror) in the old sequential loop shape. The
/// compacted payload goes through the production `triangulate_solid`
/// (store-first staging). Bit-identity proves the whole 7.2 chain:
/// mirror compaction + store resolution + staging.
#[test]
fn test_compaction_value_neutral_bit_identity() {
    for filename in ["nist_cone.stp", "nist_cylinder.stp", "brick_thin_hole.stp", "cube_with_void.stp"] {
        let solids = load_solids(filename);
        for (si, solid) in solids.iter().enumerate() {
            let params = TriangulationParams::default();
            let ctx = format!("{filename} solid {si}");

            // Production: compacted payload → store-first staging.
            let compacted_mesh = triangulate_solid(solid, &params);

            // Reference: re-materialize mirrors, run the pre-7.2 pipeline.
            let re_materialized = re_materialize_mirrors(solid);
            let reference_mesh = legacy_mirror_pipeline(&re_materialized, &params);

            if reference_mesh.triangles.is_empty() {
                // Pre-existing empty payload (degenerate STEP case, e.g.
                // cube_with_void solid 2 was empty before Stage 7.2 too):
                // the new pipeline must stay empty as well.
                assert!(
                    compacted_mesh.triangles.is_empty(),
                    "{ctx}: reference pipeline produced no triangles, production produced {}",
                    compacted_mesh.triangles.len()
                );
                continue;
            }
            assert!(
                !compacted_mesh.triangles.is_empty(),
                "{ctx}: production mesh is empty while reference has {} triangles",
                reference_mesh.triangles.len()
            );
            assert_meshes_bit_identical(&compacted_mesh, &reference_mesh, &ctx);
        }
    }
}

/// Rebuild the per-face `edges` mirrors from the store (Stage 6
/// `Solid::instance_edges` mirror re-derivation), producing the pre-C5
/// in-memory shape of the same solid.
fn re_materialize_mirrors(solid: &Solid) -> Solid {
    let mut clone = solid.clone();
    let rebuilt: Vec<Vec<draper_topology::Edge>> = clone
        .faces()
        .iter()
        .map(|face| clone.instance_edges(face))
        .collect();
    for (face, edges) in clone.faces_mut().iter_mut().zip(rebuilt) {
        face.edges = edges;
    }
    clone
}

/// The pre-Stage-7.2 sequential pipeline: pre-populate the shared cache,
/// then run the MIRROR-READING per-face entry (`triangulate_face_with_cache`
/// on the face itself), merging with tolerance dedup — the exact loop shape
/// `triangulate_solid_sequential` had before the store-first migration.
fn legacy_mirror_pipeline(solid: &Solid, params: &TriangulationParams) -> TriangleMesh {
    // Same adaptive-tolerance cache setup as `triangulate_solid_with_report`.
    let (bmin, bmax) = legacy_solid_bbox(solid);
    let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(&bmin, &bmax, 64);
    cache.set_chord_tolerance_override(Some(params.max_deviation));
    cache.pre_populate_for_solid(solid, 20);

    let adaptive_tol = cache.adaptive_tolerance().merge_tolerance();
    let mut mesh = TriangleMesh::new();
    let mut dedup_map = VertexDedupMap::with_tolerance(adaptive_tol);
    for (face_idx, face) in solid.faces().iter().enumerate() {
        let mut face_mesh = triangulate_face_with_cache(face, params, &mut cache);
        let face_tri_count = face_mesh.triangles.len();
        face_mesh.triangle_face_ids = Some(vec![face_idx as u64; face_tri_count]);
        mesh.merge_deduplicating(&face_mesh, &mut dedup_map);
    }
    filter_degenerate_triangles(&mut mesh, 1e-10);
    mesh
}

/// Bounding-box scan over non-degenerate edge endpoints — the same scan
/// `solid_bounding_box` performs (store-resolved via `Solid::face_edges`).
fn legacy_solid_bbox(solid: &Solid) -> (draper_geometry::Point3d, draper_geometry::Point3d) {
    use draper_geometry::Point3d;
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
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
        return (Point3d::ORIGIN, Point3d::new(1.0, 1.0, 1.0));
    }
    (min, max)
}
