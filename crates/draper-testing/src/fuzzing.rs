// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.5 — Fuzzing
//!
//! Random perturbation of surface parameters and STEP entity modifications.
//! Uses a simple LCG RNG (no external crate needed).

use draper_mesh::{TriangulationParams, triangulate_solid};
use draper_topology::Solid;

/// Outcome of a single fuzz iteration.
#[derive(Debug, Clone)]
pub enum FuzzOutcome {
    /// The fuzz input was processed successfully.
    Success,
    /// The fuzz input caused a panic (caught by catch_unwind).
    Panic,
    /// The fuzz input returned an error.
    Error,
    /// The fuzz input produced NaN or Inf values.
    NaN,
}

/// Result of a single fuzz iteration.
#[derive(Debug, Clone)]
pub struct FuzzResult {
    /// Which iteration this was.
    pub iteration: usize,
    /// Description of the fuzz input.
    pub input_desc: String,
    /// Outcome of the fuzz iteration.
    pub result: FuzzOutcome,
    /// Error message if the outcome was Error.
    pub error_msg: Option<String>,
}

/// Simple LCG (Linear Congruential Generator) RNG.
/// No external crate needed.
#[derive(Clone, Debug)]
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: if seed == 0 { 1 } else { seed } }
    }

    fn next_u64(&mut self) -> u64 {
        // Numerical Recipes LCG parameters
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        let bits = self.next_u64();
        // Map to [0, 1)
        (bits >> 11) as f64 / (1u64 << 53) as f64
    }

    fn next_range(&mut self, min: f64, max: f64) -> f64 {
        min + self.next_f64() * (max - min)
    }
}

/// Fuzz surface parameters by random perturbation.
/// Creates random surface geometries, builds Solids, triangulates them,
/// and checks for NaN/Inf.
pub fn fuzz_surface_params(iterations: usize) -> Vec<FuzzResult> {
    let mut rng = Lcg::new(42);
    let mut results = Vec::with_capacity(iterations);

    for i in 0..iterations {
        let surface_type = (rng.next_u64() % 5) as usize;
        let (solid, desc) = match surface_type {
            0 => {
                // Random sphere
                let radius = rng.next_range(0.01, 100.0);
                let solid = draper_topology::ShapeBuilder::make_sphere(radius);
                (solid, format!("Sphere(r={:.3})", radius))
            }
            1 => {
                // Random cylinder
                let radius = rng.next_range(0.01, 100.0);
                let height = rng.next_range(0.01, 100.0);
                let solid = draper_topology::ShapeBuilder::make_cylinder(radius, height);
                (solid, format!("Cylinder(r={:.3},h={:.3})", radius, height))
            }
            2 => {
                // Random cone
                let radius = rng.next_range(0.01, 100.0);
                let height = rng.next_range(0.01, 100.0);
                let half_angle = rng.next_range(0.01, 1.5);
                let solid = draper_topology::ShapeBuilder::make_cone(radius, height, half_angle);
                (solid, format!("Cone(r={:.3},h={:.3},ha={:.3})", radius, height, half_angle))
            }
            3 => {
                // Random torus
                let major = rng.next_range(0.5, 100.0);
                let minor = rng.next_range(0.01, major * 0.9);
                let solid = draper_topology::ShapeBuilder::make_torus(major, minor);
                (solid, format!("Torus(R={:.3},r={:.3})", major, minor))
            }
            4 => {
                // Random box
                let dx = rng.next_range(0.01, 100.0);
                let dy = rng.next_range(0.01, 100.0);
                let dz = rng.next_range(0.01, 100.0);
                let solid = draper_topology::ShapeBuilder::make_box(dx, dy, dz);
                (solid, format!("Box({:.3},{:.3},{:.3})", dx, dy, dz))
            }
            _ => unreachable!(),
        };

        let result = fuzz_triangulate_solid(i, &solid, &desc);
        results.push(result);
    }

    results
}

/// Fuzz STEP entity modifications by creating random STEP content.
pub fn fuzz_step_entities(iterations: usize) -> Vec<FuzzResult> {
    let mut rng = Lcg::new(123);
    let mut results = Vec::with_capacity(iterations);

    for i in 0..iterations {
        let radius = rng.next_range(0.001, 50.0);
        let height = rng.next_range(0.001, 50.0);

        // Generate a STEP file with random cylinder parameters
        let step_content = format!(
            r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Fuzz test'),'2;1');
FILE_NAME('fuzz.stp','2026-06-01',('KernelDev'),('KernelDev'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=SHAPE_REPRESENTATION_RELATIONSHIP('','',#2,#3);
#2=SHAPE_REPRESENTATION('',(#4),#100);
#3=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#5),#100);
#4=AXIS2_PLACEMENT_3D('',#6,$,$);
#5=MANIFOLD_SOLID_BREP('',#7);
#6=CARTESIAN_POINT('',(0.,0.,0.));
#7=CLOSED_SHELL('',(#8,#20,#30));
#8=ADVANCED_FACE('',(#9),#10,.T.);
#9=FACE_OUTER_BOUND('',#11,.T.);
#10=CYLINDRICAL_SURFACE('',#12,{radius});
#11=EDGE_LOOP('',(#13));
#12=AXIS2_PLACEMENT_3D('',#14,#15,#16);
#13=ORIENTED_EDGE('',*,*,#17,.T.);
#14=CARTESIAN_POINT('',(0.,0.,0.));
#15=DIRECTION('',(0.,0.,1.));
#16=DIRECTION('',(1.,0.,0.));
#17=EDGE_CURVE('',#18,#18,#19,.T.);
#18=VERTEX_POINT('',#21);
#19=CIRCLE('',#22,{radius});
#21=CARTESIAN_POINT('',(0.,0.,0.));
#22=AXIS2_PLACEMENT_3D('',#23,#24,#25);
#23=CARTESIAN_POINT('',(0.,0.,0.));
#24=DIRECTION('',(0.,0.,1.));
#25=DIRECTION('',(1.,0.,0.));
#20=ADVANCED_FACE('',(#26),#27,.T.);
#26=FACE_OUTER_BOUND('',#28,.T.);
#27=PLANE('',#29);
#28=EDGE_LOOP('',(#31));
#29=AXIS2_PLACEMENT_3D('',#32,#33,#34);
#31=ORIENTED_EDGE('',*,*,#35,.F.);
#32=CARTESIAN_POINT('',(0.,0.,0.));
#33=DIRECTION('',(0.,0.,1.));
#34=DIRECTION('',(1.,0.,0.));
#35=EDGE_CURVE('',#36,#36,#37,.T.);
#36=VERTEX_POINT('',#38);
#37=CIRCLE('',#39,{radius});
#38=CARTESIAN_POINT('',(0.,0.,0.));
#39=AXIS2_PLACEMENT_3D('',#40,#41,#42);
#40=CARTESIAN_POINT('',(0.,0.,0.));
#41=DIRECTION('',(0.,0.,1.));
#42=DIRECTION('',(1.,0.,0.));
#30=ADVANCED_FACE('',(#43),#44,.T.);
#43=FACE_OUTER_BOUND('',#45,.T.);
#44=PLANE('',#46);
#45=EDGE_LOOP('',(#47));
#46=AXIS2_PLACEMENT_3D('',#48,#49,#50);
#47=ORIENTED_EDGE('',*,*,#51,.T.);
#48=CARTESIAN_POINT('',(0.,0.,{height}));
#49=DIRECTION('',(0.,0.,1.));
#50=DIRECTION('',(1.,0.,0.));
#51=EDGE_CURVE('',#52,#52,#53,.T.);
#52=VERTEX_POINT('',#54);
#53=CIRCLE('',#55,{radius});
#54=CARTESIAN_POINT('',(0.,0.,{height}));
#55=AXIS2_PLACEMENT_3D('',#56,#57,#58);
#56=CARTESIAN_POINT('',(0.,0.,{height}));
#57=DIRECTION('',(0.,0.,1.));
#58=DIRECTION('',(1.,0.,0.));
#100=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#101))GLOBAL_UNIT_ASSIGNED_CONTEXT((#102,#103,#104))REPRESENTATION_CONTEXT('',''));
#101=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),'','');
#102=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#103=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#104=(NAMED_UNIT(*)SOLID_ANGLE_UNIT()SI_UNIT($,.STERADIAN.));
ENDSEC;
END-ISO-10303-21;
"#
        );

        let desc = format!("STEP_Cylinder(r={:.3},h={:.3})", radius, height);
        let desc_clone = desc.clone();

        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            match draper_step::parse_step(&step_content) {
                Ok(step_file) => {
                    match draper_step::step_to_mesh(&step_file) {
                        Ok(mesh) => {
                            // Check for NaN/Inf in vertices
                            let has_nan = mesh.vertices.iter().any(|v|
                                !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite()
                            );
                            if has_nan {
                                FuzzResult {
                                    iteration: i,
                                    input_desc: desc,
                                    result: FuzzOutcome::NaN,
                                    error_msg: Some("NaN/Inf in mesh vertices".to_string()),
                                }
                            } else {
                                FuzzResult {
                                    iteration: i,
                                    input_desc: desc,
                                    result: FuzzOutcome::Success,
                                    error_msg: None,
                                }
                            }
                        }
                        Err(e) => FuzzResult {
                            iteration: i,
                            input_desc: desc,
                            result: FuzzOutcome::Error,
                            error_msg: Some(e),
                        },
                    }
                }
                Err(e) => FuzzResult {
                    iteration: i,
                    input_desc: desc,
                    result: FuzzOutcome::Error,
                    error_msg: Some(format!("Parse error: {}", e)),
                },
            }
        }));

        match result {
            Ok(fuzz_result) => results.push(fuzz_result),
            Err(_) => results.push(FuzzResult {
                iteration: i,
                input_desc: desc_clone,
                result: FuzzOutcome::Panic,
                error_msg: Some("Panic during fuzz iteration".to_string()),
            }),
        }
    }

    results
}

/// Triangulate a solid and check for NaN/Inf — used by fuzz_surface_params.
fn fuzz_triangulate_solid(iteration: usize, solid: &Solid, desc: &str) -> FuzzResult {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let params = TriangulationParams::default();
        let mesh = triangulate_solid(solid, &params);

        // Check for NaN/Inf in vertices
        let has_nan = mesh.vertices.iter().any(|v|
            !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite()
        );

        if has_nan {
            FuzzOutcome::NaN
        } else {
            FuzzOutcome::Success
        }
    }));

    match result {
        Ok(outcome) => {
            let is_nan = matches!(outcome, FuzzOutcome::NaN);
            FuzzResult {
                iteration,
                input_desc: desc.to_string(),
                result: outcome,
                error_msg: if is_nan {
                    Some("NaN/Inf in mesh vertices".to_string())
                } else {
                    None
                },
            }
        }
        Err(_) => FuzzResult {
            iteration,
            input_desc: desc.to_string(),
            result: FuzzOutcome::Panic,
            error_msg: Some("Panic during triangulation".to_string()),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fuzz_surface_params_100_iterations() {
        let results = fuzz_surface_params(100);
        assert_eq!(results.len(), 100, "Should have 100 fuzz results");

        let panic_count = results.iter().filter(|r| matches!(r.result, FuzzOutcome::Panic)).count();
        assert_eq!(panic_count, 0, "No fuzz iteration should panic, but {} did", panic_count);
    }

    #[test]
    fn test_fuzz_step_entities_100_iterations() {
        let results = fuzz_step_entities(100);
        assert_eq!(results.len(), 100, "Should have 100 fuzz results");

        let panic_count = results.iter().filter(|r| matches!(r.result, FuzzOutcome::Panic)).count();
        assert_eq!(panic_count, 0, "No STEP fuzz iteration should panic, but {} did", panic_count);
    }
}
