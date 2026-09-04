//! C5 Stage 6.1 regression — store-first validation/queries consistency.
//!
//! Historically this suite compared "mirrors present" vs "mirrors cleared"
//! solids. C5 Stage 7.6b physically removed `Face::edges`, so the
//! mirror-free premise is structurally dead: every consumer reads the
//! `EdgeStore` + `edge_ids` and there is nothing to clear. The suite keeps
//! the parts that still carry regression value:
//!
//! 1. validation reports are DETERMINISTIC (stable fingerprints across
//!    independent evaluations of the same solid and of a clone);
//! 2. mutating validation / healing preserve the solid's store and
//!    `edge_ids` (the 7.6b "store survives re-shelling" contract);
//! 3. analytical query results (volume / area / bounding box) are
//!    bit-identical across clones.
//!
//! Clone fidelity is the modern stand-in for the old mirror-clearing
//! operation: `Solid::clone` copies store + edge_ids wholesale, so any
//! divergence between original and clone fingerprints would indicate
//! non-store data leaking back into the read paths.

use draper_topology::builder::ShapeBuilder;
use draper_topology::boolean::boolean_subtract;
use draper_topology::validation::{
    heal_solid, validate_solid, validate_solid_readonly, validate_topology,
    TopologyValidationConfig, TopologyValidationReport,
};
use draper_topology::validator::{validate_brep, validate_tolerance_consistency};
use draper_topology::queries::{solid_surface_area, solid_volume};
use draper_topology::traversal::solid_bounding_box;
use draper_geometry::ToleranceContext;
use draper_topology::Solid;

fn test_solids() -> Vec<Solid> {
    let mut solids = vec![
        ShapeBuilder::make_box(2.0, 2.0, 2.0),
        ShapeBuilder::make_cylinder(0.5, 2.0),
        ShapeBuilder::make_sphere(1.0),
    ];
    // Boolean result: box − cylinder — shared edges carry store aliases
    // and instance orientation flags (the Stage 5.3 machinery).
    let box_solid = ShapeBuilder::make_box(2.0, 2.0, 2.0);
    let cyl_solid = ShapeBuilder::make_cylinder_at(0.0, 0.0, 0.0, 0.5, 2.0);
    if let Ok(result) = boolean_subtract(&box_solid, &cyl_solid, &ToleranceContext::default()) {
        solids.push(result);
    }
    // C5 7.6b: builder and boolean solids are born-indexed (store built
    // during construction) — no explicit index pass is needed anymore.
    solids
}

/// Sorted issue fingerprints — HashMap-driven issue insertion order is not
/// deterministic across calls, but the issue SET must be.
fn sorted_issues(report: &TopologyValidationReport) -> Vec<String> {
    let mut fingerprints: Vec<String> = report
        .issues
        .iter()
        .map(|issue| format!("{:?}|{:?}|{}", issue.severity, issue.entity_id, issue.message))
        .collect();
    fingerprints.sort();
    fingerprints
}

fn validate_brep_fingerprint(solid: &Solid, config: &TopologyValidationConfig) -> String {
    let report = validate_brep(solid, config);
    let mut parts = vec![
        format!("faces={}", report.face_count),
        format!("edges={}", report.edge_count),
        format!("vertices={}", report.vertex_count),
        format!("no_outer_loop={}", report.faces_without_outer_loop),
        format!("bad_orientation={}", report.edges_with_bad_orientation),
        format!("dangling={}", report.dangling_edges),
        format!("euler={}", report.euler_characteristic),
        format!("errors={}", report.detailed.error_count),
        format!("warnings={}", report.detailed.warning_count),
        format!("infos={}", report.detailed.info_count),
    ];
    parts.extend(sorted_issues(&report.detailed));
    parts.join("\n")
}

fn validate_topology_fingerprint(solid: &Solid, config: &TopologyValidationConfig) -> String {
    let report = validate_topology(solid, config);
    let mut parts = vec![
        format!("errors={}", report.error_count),
        format!("warnings={}", report.warning_count),
        format!("infos={}", report.info_count),
    ];
    parts.extend(sorted_issues(&report));
    parts.join("\n")
}

fn tolerance_fingerprint(solid: &Solid) -> String {
    let report = validate_tolerance_consistency(solid);
    let mut messages = report.messages.clone();
    messages.sort();
    format!(
        "total={} face_exceeds={} edge_exceeds={} vertex_off={} msgs={:?}",
        report.total_violations,
        report.face_exceeds_shell,
        report.edge_exceeds_face,
        report.vertex_not_on_edge,
        messages
    )
}

#[test]
fn store_first_validation_reports_deterministic() {
    for solid in test_solids() {
        let config = TopologyValidationConfig::default();
        let brep_a = validate_brep_fingerprint(&solid, &config);
        let topo_a = validate_topology_fingerprint(&solid, &config);
        let tol_a = tolerance_fingerprint(&solid);
        let readonly_a = format!("{:?}", validate_solid_readonly(&solid));

        // The clone carries store + edge_ids wholesale; fingerprints must
        // not diverge between the original and the cloned evaluation.
        let clone = solid.clone();
        let brep_b = validate_brep_fingerprint(&clone, &config);
        let topo_b = validate_topology_fingerprint(&clone, &config);
        let tol_b = tolerance_fingerprint(&clone);
        let readonly_b = format!("{:?}", validate_solid_readonly(&clone));

        assert_eq!(
            brep_a, brep_b,
            "validate_brep report differs between solid and clone"
        );
        assert_eq!(
            topo_a, topo_b,
            "validate_topology report differs between solid and clone"
        );
        assert_eq!(
            tol_a, tol_b,
            "validate_tolerance_consistency report differs between solid and clone"
        );
        assert_eq!(
            readonly_a, readonly_b,
            "validate_solid_readonly errors differ between solid and clone"
        );
    }
}

#[test]
fn store_first_mutating_validation_and_heal_preserve_store() {
    for solid in test_solids() {
        // The sphere builder is legitimately edge-less (single full-surface
        // face) — its store is empty by construction, not by data loss.
        let has_edges = solid
            .faces()
            .iter()
            .any(|f| !f.edge_ids.is_empty());

        let mut a = solid.clone();
        let _errors_a = format!("{:?}", validate_solid(&mut a));
        let _heals_a = format!("{:?}", heal_solid(&mut a));
        assert_eq!(
            !a.edge_store.is_empty(),
            has_edges,
            "store must survive mutating validation/healing"
        );

        // The same through a clone: the store and face edge_ids survive
        // the re-shelling surgery inside heal_solid (7.6b preservation).
        let mut b = solid.clone();
        let _errors_b = format!("{:?}", validate_solid(&mut b));
        let _heals_b = format!("{:?}", heal_solid(&mut b));
        assert_eq!(
            !b.edge_store.is_empty(),
            has_edges,
            "heal_solid wiped the store of a cloned solid (preservation broken)"
        );
        if has_edges {
            assert!(
                !b.faces().iter().all(|f| f.edge_ids.is_empty()),
                "heal_solid wiped face edge_ids of a cloned solid"
            );
        }
    }
}

#[test]
fn store_first_analytical_queries_clone_stable() {
    for solid in test_solids() {
        let volume_a = solid_volume(&solid);
        let area_a = solid_surface_area(&solid);
        let (min_a, max_a) = solid_bounding_box(&solid, 8);

        let clone = solid.clone();
        let volume_b = solid_volume(&clone);
        let area_b = solid_surface_area(&clone);
        let (min_b, max_b) = solid_bounding_box(&clone, 8);

        assert!(
            volume_a.to_bits() == volume_b.to_bits(),
            "solid_volume differs between solid and clone: {} vs {}",
            volume_a,
            volume_b
        );
        assert!(
            area_a.to_bits() == area_b.to_bits(),
            "solid_surface_area differs between solid and clone: {} vs {}",
            area_a,
            area_b
        );
        assert_eq!(
            format!("{:?}", (min_a, max_a)),
            format!("{:?}", (min_b, max_b)),
            "solid_bounding_box differs between solid and clone"
        );
    }
}
