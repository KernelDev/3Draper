// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Vision 2036 §1.4 item 1 — completeness of STEP tolerance extraction.
//!
//! Covers the extraction paths added/verified in §1.4:
//! 1. Inline `UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(v), #unit)` (regression).
//! 2. Reference-based uncertainty (`UNCERTAINTY_MEASURE_WITH_UNIT(#measure, ...)`).
//! 3. AP214 GD&T subtypes with inline values (`FLATNESS_TOLERANCE(..., 0.01, ...)`)
//!    — previously missed: only `starts_with("GEOMETRIC_TOLERANCE")` was matched.
//! 4. The AP242 reference chain
//!    `PERPENDICULARITY_TOLERANCE → MEASURE_REPRESENTATION_ITEM →
//!    LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(v), #unit)` (previously missed:
//!    `StepValue::Ref` was never resolved).
//! 5. Standalone `LENGTH_MEASURE_WITH_UNIT(1.0)` unit declarations are NOT
//!    tolerances (as1-oc-214_bolt.stp pattern).
//! 6. Datum/point references reachable from `*_TOLERANCE` never leak
//!    coordinates into the tolerance value.
//! 7. Angle-typed measures are rejected.
//! 8. Tightest value wins across sources.

use draper_step::{extract_step_tolerance, parse_step};

/// Wrap entity lines into a minimal ISO-10303-21 file.
fn step_with(entities: &[&str]) -> String {
    let mut s = String::new();
    s.push_str("ISO-10303-21;\nHEADER;\n");
    s.push_str("FILE_DESCRIPTION((''),'2;1');\n");
    s.push_str("FILE_NAME('tol.stp','2026-09-12T00:00:00',(''),(''),'','','');\n");
    s.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }'));\n");
    s.push_str("ENDSEC;\nDATA;\n");
    for e in entities {
        s.push_str(e);
        s.push('\n');
    }
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    s
}

#[test]
fn test_uncertainty_inline_regression() {
    let text = step_with(&[
        "#1 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.26E-006),#2,'','');",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, Some(2.26e-6));
}

#[test]
fn test_uncertainty_via_reference() {
    // Some writers emit the measure as a standalone referenced entity.
    let text = step_with(&[
        "#10 = LENGTH_MEASURE(0.005);",
        "#11 = UNCERTAINTY_MEASURE_WITH_UNIT(#10,#2,'','');",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, Some(0.005));
}

#[test]
fn test_gdt_subtype_inline_value() {
    // FLATNESS_TOLERANCE does NOT start with GEOMETRIC_TOLERANCE — the old
    // prefix rule missed it (test/gdt_test.stp pattern).
    let text = step_with(&[
        "#940 = FLATNESS_TOLERANCE('flatness','top surface flatness',0.01,#910);",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, Some(0.01));
}

#[test]
fn test_gdt_ap242_reference_chain() {
    // PERPENDICULARITY_TOLERANCE → MEASURE_REPRESENTATION_ITEM →
    // LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(v), #unit)
    let text = step_with(&[
        "#21 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.02),#22);",
        "#20 = MEASURE_REPRESENTATION_ITEM('tolerance value',#21);",
        "#941 = PERPENDICULARITY_TOLERANCE('perp','back',#20,#30,(#40));",
        "#30 = DATUM('A',#31);",
        "#31 = AXIS2_PLACEMENT_3D('',#32,#33,#34);",
        "#32 = CARTESIAN_POINT('',(1.5,2.5,3.5));",
        "#40 = DATUM('B',#41);",
        "#41 = CARTESIAN_POINT('',(7.5,8.5,9.5));",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, Some(0.02), "AP242 chain must resolve to 0.02");
}

#[test]
fn test_standalone_length_measure_with_unit_is_not_a_tolerance() {
    // as1-oc-214_bolt.stp: #5=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0),#4)
    // — a unit declaration (1 mm), not a tolerance. Must be ignored.
    let text = step_with(&[
        "#4 = (LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));",
        "#5 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0),#4);",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, None, "standalone LMWU(1.0) is a unit declaration");
}

#[test]
fn test_datum_coordinates_never_leak() {
    // The tolerance value sits AFTER a datum reference whose target carries
    // plausible-looking floats — the whitelist must skip the datum.
    let text = step_with(&[
        "#30 = DATUM('A',#31);",
        "#31 = CARTESIAN_POINT('',(0.25,0.5,0.75));",
        "#950 = CYLINDRICITY_TOLERANCE('cy','zone',#30,0.05);",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, Some(0.05), "must pick 0.05, never 0.25/0.5/0.75");
}

#[test]
fn test_angle_measures_rejected() {
    let text = step_with(&[
        "#50 = PLANE_ANGLE_MEASURE(0.01745);",
        "#51 = MEASURE_REPRESENTATION_ITEM('angle tol',#50);",
        "#960 = ANGULARITY_TOLERANCE('ang','zone',#51);",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, None, "plane-angle measures must be rejected");
}

#[test]
fn test_tightest_value_wins() {
    let text = step_with(&[
        "#1 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.01),#2,'','');",
        "#940 = FLATNESS_TOLERANCE('flatness','top',0.001,#910);",
        "#941 = PERPENDICULARITY_TOLERANCE('perp','back',0.02,#910);",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, Some(0.001));
}

#[test]
fn test_no_tolerance_metadata() {
    let text = step_with(&[
        "#1 = CARTESIAN_POINT('',(0.0,0.0,0.0));",
    ]);
    let file = parse_step(&text).expect("parse");
    let tol = extract_step_tolerance(&file);
    assert_eq!(tol, None);
}
