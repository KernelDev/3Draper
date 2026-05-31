// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.4 — Problematic Files
//!
//! Functions to create STEP files with known problems.
//! Tests that these parse and triangulate WITHOUT panicking.

/// Generate a STEP file with a zero-length (degenerate) edge.
/// Two vertices at the same point connected by a LINE edge of length 0.
pub fn make_degenerate_edge_step() -> String {
    r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test file with degenerate edge'),'2;1');
FILE_NAME('degenerate_edge.stp','2026-06-01',('KernelDev'),('KernelDev'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=SHAPE_REPRESENTATION_RELATIONSHIP('','',#2,#3);
#2=SHAPE_REPRESENTATION('',(#4),#100);
#3=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#5),#100);
#4=AXIS2_PLACEMENT_3D('',#6,$,$);
#5=MANIFOLD_SOLID_BREP('',#7);
#6=CARTESIAN_POINT('',(0.,0.,0.));
#7=CLOSED_SHELL('',(#8));
#8=ADVANCED_FACE('',(#9),#10,.T.);
#9=FACE_OUTER_BOUND('',#11,.T.);
#10=PLANE('',#12);
#11=EDGE_LOOP('',(#13));
#12=AXIS2_PLACEMENT_3D('',#14,#15,#16);
#13=ORIENTED_EDGE('',*,*,#17,.T.);
#14=CARTESIAN_POINT('',(0.,0.,0.));
#15=DIRECTION('',(0.,0.,1.));
#16=DIRECTION('',(1.,0.,0.));
#17=EDGE_CURVE('',#18,#18,#19,.T.);
#18=VERTEX_POINT('',#20);
#19=LINE('',#21,#22);
#20=CARTESIAN_POINT('',(0.,0.,0.));
#21=CARTESIAN_POINT('',(0.,0.,0.));
#22=VECTOR('',#23,0.);
#23=DIRECTION('',(1.,0.,0.));
#100=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#101))GLOBAL_UNIT_ASSIGNED_CONTEXT((#102,#103,#104))REPRESENTATION_CONTEXT('',''));
#101=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),'','');
#102=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#103=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#104=(NAMED_UNIT(*)SOLID_ANGLE_UNIT()SI_UNIT($,.STERADIAN.));
ENDSEC;
END-ISO-10303-21;
"#.to_string()
}

/// Generate a STEP file with a zero-radius cylinder (degenerate surface).
pub fn make_zero_radius_step() -> String {
    r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test file with zero radius cylinder'),'2;1');
FILE_NAME('zero_radius.stp','2026-06-01',('KernelDev'),('KernelDev'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=SHAPE_REPRESENTATION_RELATIONSHIP('','',#2,#3);
#2=SHAPE_REPRESENTATION('',(#4),#100);
#3=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#5),#100);
#4=AXIS2_PLACEMENT_3D('',#6,$,$);
#5=MANIFOLD_SOLID_BREP('',#7);
#6=CARTESIAN_POINT('',(0.,0.,0.));
#7=CLOSED_SHELL('',(#8));
#8=ADVANCED_FACE('',(#9),#10,.T.);
#9=FACE_OUTER_BOUND('',#11,.T.);
#10=CYLINDRICAL_SURFACE('',#12,0.);
#11=EDGE_LOOP('',(#13));
#12=AXIS2_PLACEMENT_3D('',#14,#15,#16);
#13=ORIENTED_EDGE('',*,*,#17,.T.);
#14=CARTESIAN_POINT('',(0.,0.,0.));
#15=DIRECTION('',(0.,0.,1.));
#16=DIRECTION('',(1.,0.,0.));
#17=EDGE_CURVE('',#18,#18,#19,.T.);
#18=VERTEX_POINT('',#20);
#19=LINE('',#21,#22);
#20=CARTESIAN_POINT('',(0.,0.,0.));
#21=CARTESIAN_POINT('',(0.,0.,0.));
#22=VECTOR('',#23,1.);
#23=DIRECTION('',(0.,0.,1.));
#100=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#101))GLOBAL_UNIT_ASSIGNED_CONTEXT((#102,#103,#104))REPRESENTATION_CONTEXT('',''));
#101=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),'','');
#102=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#103=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#104=(NAMED_UNIT(*)SOLID_ANGLE_UNIT()SI_UNIT($,.STERADIAN.));
ENDSEC;
END-ISO-10303-21;
"#.to_string()
}

/// Generate a STEP file that could produce NaN values.
/// Uses a sphere with radius=0 which produces degenerate geometry.
pub fn make_nan_surface_step() -> String {
    r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test file that could produce NaN'),'2;1');
FILE_NAME('nan_surface.stp','2026-06-01',('KernelDev'),('KernelDev'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=SHAPE_REPRESENTATION_RELATIONSHIP('','',#2,#3);
#2=SHAPE_REPRESENTATION('',(#4),#100);
#3=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#5),#100);
#4=AXIS2_PLACEMENT_3D('',#6,$,$);
#5=MANIFOLD_SOLID_BREP('',#7);
#6=CARTESIAN_POINT('',(0.,0.,0.));
#7=CLOSED_SHELL('',(#8));
#8=ADVANCED_FACE('',(#9),#10,.T.);
#9=FACE_OUTER_BOUND('',#11,.T.);
#10=SPHERICAL_SURFACE('',#12,0.);
#11=EDGE_LOOP('',(#13));
#12=AXIS2_PLACEMENT_3D('',#14,#15,#16);
#13=ORIENTED_EDGE('',*,*,#17,.T.);
#14=CARTESIAN_POINT('',(0.,0.,0.));
#15=DIRECTION('',(0.,0.,1.));
#16=DIRECTION('',(1.,0.,0.));
#17=EDGE_CURVE('',#18,#18,#19,.T.);
#18=VERTEX_POINT('',#20);
#19=LINE('',#21,#22);
#20=CARTESIAN_POINT('',(0.,0.,0.));
#21=CARTESIAN_POINT('',(0.,0.,0.));
#22=VECTOR('',#23,1.);
#23=DIRECTION('',(0.,0.,1.));
#100=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#101))GLOBAL_UNIT_ASSIGNED_CONTEXT((#102,#103,#104))REPRESENTATION_CONTEXT('',''));
#101=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),'','');
#102=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#103=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#104=(NAMED_UNIT(*)SOLID_ANGLE_UNIT()SI_UNIT($,.STERADIAN.));
ENDSEC;
END-ISO-10303-21;
"#.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_step::parse_step;

    /// Test that a STEP file with a degenerate edge can be parsed
    /// without panicking.
    #[test]
    fn test_degenerate_edge_step_parses_without_panic() {
        let content = make_degenerate_edge_step();
        let result = std::panic::catch_unwind(|| {
            let _ = parse_step(&content);
        });
        assert!(result.is_ok(), "Parsing degenerate edge STEP should not panic");
    }

    /// Test that a STEP file with zero-radius cylinder can be parsed
    /// without panicking.
    #[test]
    fn test_zero_radius_step_parses_without_panic() {
        let content = make_zero_radius_step();
        let result = std::panic::catch_unwind(|| {
            let _ = parse_step(&content);
        });
        assert!(result.is_ok(), "Parsing zero-radius STEP should not panic");
    }

    /// Test that a STEP file that could produce NaN can be parsed
    /// without panicking.
    #[test]
    fn test_nan_surface_step_parses_without_panic() {
        let content = make_nan_surface_step();
        let result = std::panic::catch_unwind(|| {
            let _ = parse_step(&content);
        });
        assert!(result.is_ok(), "Parsing NaN surface STEP should not panic");
    }

    /// Test that triangulation of problematic files does not panic.
    #[test]
    fn test_degenerate_edge_triangulate_no_panic() {
        let content = make_degenerate_edge_step();
        let result = std::panic::catch_unwind(|| {
            if let Ok(step_file) = parse_step(&content) {
                let _ = draper_step::step_to_mesh(&step_file);
            }
        });
        assert!(result.is_ok(), "Triangulating degenerate edge STEP should not panic");
    }
}
