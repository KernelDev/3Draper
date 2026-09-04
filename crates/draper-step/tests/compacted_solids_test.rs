//! C5 Stage 7.2 / 7.6b — canonical SOLID payload from STEP.
//!
//! `extract_solids` returns store-first solids: every face carries
//! canonical `edge_ids` and the edge store answers all boundary-reader
//! queries. `Face.edges` mirrors no longer exist (7.6b removed the field),
//! so the "compacted vs mirror-bearing" distinction collapsed — every
//! payload IS the compacted end-state.
//!
//! Verification criteria:
//! 1. STEP-loaded solids arrive store-first (edge_ids on every face,
//!    every wire coedge resolvable through the store).
//! 2. `triangulate_solid` on the payload is watertight (the known-good
//!    regression files: nist_cone was at 0.00% boundary since C5 Stage 1).
//! 3. Value-neutrality of the store chain: the production solid pipeline
//!    is bit-identical to a manual sequential replication that stages
//!    each face through `triangulate_solid_face_with_cache` (the
//!    store-resolved per-face entry) on the same payload.

use draper_mesh::{
    filter_degenerate_triangles, triangulate_solid, triangulate_solid_face_with_cache,
    triangulate_solid_with_report, EdgeDiscretizationCache, TriangleMesh,
    TriangulationParams, VertexDedupMap,
};
use draper_step::{extract_solids, parse_step};
use draper_topology::Solid;

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

/// Test 1 — STEP solids arrive as store-first payloads (7.6b: the
/// compacted end-state is the ONLY state — faces carry edge_ids and the
/// store answers every boundary-reader query).
#[test]
fn test_step_solids_arrive_store_first() {
    for filename in ["nist_cone.stp", "cube_with_void.stp"] {
        let solids = load_solids(filename);
        for (si, solid) in solids.iter().enumerate() {
            assert!(
                !solid.edge_store.is_empty(),
                "{filename}: solid {si} must carry an edge store"
            );
            let faces = solid.faces();
            for (fi, face) in faces.iter().enumerate() {
                assert!(
                    !face.edge_ids.is_empty(),
                    "{filename}: solid {si} face {fi}: store-first face must keep edge_ids"
                );
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
        }
    }
}

/// Test 2 — the store-first payload triangulates watertight through the
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
    // (bit-identity vs the manual replication is asserted in test 3).
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

/// Test 3 — the store chain is value-neutral: bit-identical mesh vs a
/// manual sequential replication staging every face through the
/// store-resolved per-face entry (`triangulate_solid_face_with_cache`)
/// with the same adaptive cache setup as `triangulate_solid_with_report`.
///
/// (7.6b: the historical reference re-materialized per-face mirrors and
/// ran the mirror-reading `triangulate_face_with_cache` — that path is
/// structurally gone; the manual store-first replication below proves the
/// same chain: store resolution + staging + merge.)
#[test]
fn test_store_chain_value_neutral_bit_identity() {
    for filename in [
        "nist_cone.stp",
        "nist_cylinder.stp",
        "brick_thin_hole.stp",
        "cube_with_void.stp",
    ] {
        let solids = load_solids(filename);
        for (si, solid) in solids.iter().enumerate() {
            let params = TriangulationParams::default();
            let ctx = format!("{filename} solid {si}");

            // Production pipeline.
            let production_mesh = triangulate_solid(solid, &params);

            // Manual sequential replication (store-first per-face entry).
            let reference_mesh = manual_store_pipeline(solid, &params);

            if reference_mesh.triangles.is_empty() {
                // Pre-existing empty payload (degenerate STEP case):
                // the new pipeline must stay empty as well.
                assert!(
                    production_mesh.triangles.is_empty(),
                    "{ctx}: reference pipeline produced no triangles, production produced {}",
                    production_mesh.triangles.len()
                );
                continue;
            }
            assert!(
                !production_mesh.triangles.is_empty(),
                "{ctx}: production mesh is empty while reference has {} triangles",
                reference_mesh.triangles.len()
            );
            assert_meshes_bit_identical(&production_mesh, &reference_mesh, &ctx);
        }
    }
}

/// Manual replication of the sequential solid pipeline with the per-face
/// call staged through the store-resolved entry point.
fn manual_store_pipeline(solid: &Solid, params: &TriangulationParams) -> TriangleMesh {
    // Same adaptive-tolerance cache setup as `triangulate_solid_with_report`.
    let (bmin, bmax) = store_solid_bbox(solid);
    let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(&bmin, &bmax, 64);
    cache.set_chord_tolerance_override(Some(params.max_deviation));
    cache.pre_populate_for_solid(solid, 20);

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

/// Bounding-box scan over non-degenerate edge endpoints — the same scan
/// `solid_bounding_box` performs (store-resolved via `Solid::face_edges`).
fn store_solid_bbox(solid: &Solid) -> (draper_geometry::Point3d, draper_geometry::Point3d) {
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
