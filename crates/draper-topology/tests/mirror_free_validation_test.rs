//! C5 Stage 6.1 regression — mirror-free validation/queries equivalence.
//!
//! The Stage 6 read-path migration moved validation and query consumers
//! from per-face `Face.edges` mirrors to store-resolved instance-faithful
//! edges (`Solid::instance_edges` / `build_edge_map_store`). The contract
//! under test: after the migration, validation reports and analytical
//! query results must be IDENTICAL whether the mirrors are present or
//! fully cleared (the Stage 5 end-state — `edge_ids` + `EdgeStore` only).
//!
//! Mirrors being irrelevant to these consumers is the precondition for
//! the eventual physical removal of the `Face.edges` field (Stage 6.2+).

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
use draper_topology::{Shell, Solid};

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
    for solid in &mut solids {
        solid.index_edges();
    }
    solids
}

/// Clone with every `Face.edges` mirror cleared (`edge_ids` + store intact).
fn mirror_free_clone(solid: &Solid) -> Solid {
    let mut clone = solid.clone();
    let mut shells: Vec<&mut Shell> = Vec::new();
    if let Some(ref mut shell) = clone.outer_shell {
        shells.push(shell);
    }
    for shell in &mut clone.inner_shells {
        shells.push(shell);
    }
    for shell in shells {
        for face in &mut shell.faces {
            face.edges.clear();
        }
    }
    clone
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
fn mirror_free_validation_reports_identical() {
    for solid in test_solids() {
        let config = TopologyValidationConfig::default();
        let brep_a = validate_brep_fingerprint(&solid, &config);
        let topo_a = validate_topology_fingerprint(&solid, &config);
        let tol_a = tolerance_fingerprint(&solid);
        let readonly_a = format!("{:?}", validate_solid_readonly(&solid));

        let mirror_free = mirror_free_clone(&solid);
        // The mirrors really are cleared.
        for face in mirror_free.faces() {
            assert!(face.edges.is_empty());
        }
        let brep_b = validate_brep_fingerprint(&mirror_free, &config);
        let topo_b = validate_topology_fingerprint(&mirror_free, &config);
        let tol_b = tolerance_fingerprint(&mirror_free);
        let readonly_b = format!("{:?}", validate_solid_readonly(&mirror_free));

        assert_eq!(
            brep_a, brep_b,
            "validate_brep report differs on mirror-free solid"
        );
        assert_eq!(
            topo_a, topo_b,
            "validate_topology report differs on mirror-free solid"
        );
        assert_eq!(
            tol_a, tol_b,
            "validate_tolerance_consistency report differs on mirror-free solid"
        );
        assert_eq!(
            readonly_a, readonly_b,
            "validate_solid_readonly errors differ on mirror-free solid"
        );
    }
}

#[test]
fn mirror_free_mutating_validation_and_heal_identical() {
    for solid in test_solids() {
        // The sphere builder is legitimately edge-less (single full-surface
        // face) — its store is empty by construction, not by data loss.
        let has_edges = solid
            .faces()
            .iter()
            .any(|f| !f.edges.is_empty() || !f.edge_ids.is_empty());

        // With mirrors.
        let mut a = solid.clone();
        let errors_a = format!("{:?}", validate_solid(&mut a));
        let heals_a = format!("{:?}", heal_solid(&mut a));
        assert_eq!(
            !a.edge_store.is_empty(),
            has_edges,
            "store must survive re-indexing"
        );

        // Mirror-free: index_edges Pass 0 must preserve the serialized store
        // instead of wiping it by rebuilding from (absent) mirrors.
        let mut b = mirror_free_clone(&solid);
        let errors_b = format!("{:?}", validate_solid(&mut b));
        let heals_b = format!("{:?}", heal_solid(&mut b));
        assert_eq!(
            !b.edge_store.is_empty(),
            has_edges,
            "index_edges wiped the store of a mirror-free solid (Pass 0 preservation broken)"
        );
        if has_edges {
            assert!(
                !b.faces().iter().all(|f| f.edge_ids.is_empty()),
                "index_edges wiped face edge_ids of a mirror-free solid"
            );
        }

        assert_eq!(
            errors_a, errors_b,
            "validate_solid (mutating) errors differ on mirror-free solid"
        );
        assert_eq!(
            heals_a, heals_b,
            "heal_solid fixes differ on mirror-free solid"
        );
    }
}

#[test]
fn mirror_free_analytical_queries_identical() {
    for solid in test_solids() {
        let volume_a = solid_volume(&solid);
        let area_a = solid_surface_area(&solid);
        let (min_a, max_a) = solid_bounding_box(&solid, 8);

        let mirror_free = mirror_free_clone(&solid);
        let volume_b = solid_volume(&mirror_free);
        let area_b = solid_surface_area(&mirror_free);
        let (min_b, max_b) = solid_bounding_box(&mirror_free, 8);

        assert!(
            volume_a.to_bits() == volume_b.to_bits(),
            "solid_volume differs on mirror-free solid: {} vs {}",
            volume_a,
            volume_b
        );
        assert!(
            area_a.to_bits() == area_b.to_bits(),
            "solid_surface_area differs on mirror-free solid: {} vs {}",
            area_a,
            area_b
        );
        assert_eq!(
            format!("{:?}", (min_a, max_a)),
            format!("{:?}", (min_b, max_b)),
            "solid_bounding_box differs on mirror-free solid"
        );
    }
}
