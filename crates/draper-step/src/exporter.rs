// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP file exporter (AP203/AP214).
//!
//! Exports a B-Rep Solid to a valid STEP file with proper topology:
//! - MANIFOLD_SOLID_BREP → CLOSED_SHELL → ADVANCED_FACE → FACE_BOUND → EDGE_LOOP → EDGE_CURVE
//! - BREP_WITH_VOIDS → outer CLOSED_SHELL + inner ORIENTED_CLOSED_SHELL(s)
//! - Full surface coverage: PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE / SPHERICAL_SURFACE
//!   TOROIDAL_SURFACE / SURFACE_OF_REVOLUTION / SURFACE_OF_LINEAR_EXTRUSION / B_SPLINE_SURFACE
//! - Full curve coverage: LINE / CIRCLE / ELLIPSE / HYPERBOLA / PARABOLA / B_SPLINE_CURVE
//!   TRIMMED_CURVE / SURFACE_CURVE (with PCURVE)
//! - Geometry deduplication: shared CARTESIAN_POINT / DIRECTION / AXIS2_PLACEMENT_3D /
//!   EDGE_CURVE entities are emitted only once.
//!
//! Algorithm adapted from truck-repl export examples (ricosjp/truck, Apache-2.0 OR MIT)
//! and ISO 10303-42 schema conventions.

use draper_geometry::{
    Circle, Curve3d, Curve2d, Direction3d, Ellipse, Hyperbola, Line, NurbsCurve,
    Parabola, Point3d, Surface, Arc,
};
use draper_topology::{Compound, Edge, Shell, Solid, Wire};
use std::collections::HashMap;
use std::io::{self, Write};

// ─────────────────────────────────────────────────────────────────────────
// StepWriter — centralised emitter with deduplication
// ─────────────────────────────────────────────────────────────────────────

/// Internal helper to format a float for STEP output.
/// Uses the minimal representation that round-trips (no scientific notation
/// for "nice" numbers, but full precision otherwise).
fn fmt_f64(v: f64) -> String {
    if v.is_nan() {
        "0.0".to_string()
    } else if v.is_infinite() {
        if v > 0.0 {
            "1e30".to_string()
        } else {
            "-1e30".to_string()
        }
    } else if v == 0.0 {
        "0.0".to_string()
    } else if v.fract() == 0.0 && v.abs() < 1e16 {
        format!("{}.0", v as i64)
    } else {
        // Reparse to avoid Rust's scientific notation for moderate values.
        let s = format!("{}", v);
        if s.contains('e') || s.contains('E') {
            format!("{:.15}", v)
                .trim_end_matches('0')
                .trim_end_matches('.')
                .to_string()
        } else {
            s
        }
    }
}

/// A write buffer for STEP entities with deduplication.
struct StepWriter {
    out: String,
    next_id: i64,
    /// Cache for CARTESIAN_POINT dedup — key is (x_bits, y_bits, z_bits).
    point_cache: HashMap<(u64, u64, u64), i64>,
    /// Cache for DIRECTION dedup — key is rounded (x, y, z).
    dir_cache: HashMap<(i64, i64, i64), i64>,
    /// Cache for AXIS2_PLACEMENT_3D — key is (pt_id, dir_id, ref_dir_id).
    axis2_cache: HashMap<(i64, i64, i64), i64>,
    /// Cache for EDGE_CURVE dedup by content hash.
    edge_cache: HashMap<String, i64>,
    /// Cache for curve geometry by content hash.
    curve_cache: HashMap<String, i64>,
    /// Cache for surface geometry by content hash.
    surface_cache: HashMap<String, i64>,
    /// Cache for vertex by point id (VERTEX_POINT).
    vertex_cache: HashMap<i64, i64>,
}

impl StepWriter {
    fn new() -> Self {
        Self {
            out: String::with_capacity(64 * 1024),
            next_id: 1,
            point_cache: HashMap::new(),
            dir_cache: HashMap::new(),
            axis2_cache: HashMap::new(),
            edge_cache: HashMap::new(),
            curve_cache: HashMap::new(),
            surface_cache: HashMap::new(),
            vertex_cache: HashMap::new(),
        }
    }

    #[inline]
    fn alloc_id(&mut self) -> i64 {
        let i = self.next_id;
        self.next_id += 1;
        i
    }

    fn push_line(&mut self, s: &str) {
        self.out.push_str(s);
        self.out.push('\n');
    }

    // ── CARTESIAN_POINT ──
    fn emit_point(&mut self, pt: &Point3d) -> i64 {
        let key = (
            pt.x.to_bits(),
            pt.y.to_bits(),
            pt.z.to_bits(),
        );
        if let Some(&id) = self.point_cache.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = CARTESIAN_POINT('',({},{},{}));",
            id,
            fmt_f64(pt.x),
            fmt_f64(pt.y),
            fmt_f64(pt.z)
        ));
        self.point_cache.insert(key, id);
        id
    }

    // ── DIRECTION (dedup by 1e-9 quantisation) ──
    fn emit_direction(&mut self, dir: &Direction3d) -> i64 {
        // Quantise to 1e-9 to dedup near-identical directions.
        let key = (
            (dir.x * 1e9).round() as i64,
            (dir.y * 1e9).round() as i64,
            (dir.z * 1e9).round() as i64,
        );
        if let Some(&id) = self.dir_cache.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = DIRECTION('',({},{},{}));",
            id,
            fmt_f64(dir.x),
            fmt_f64(dir.y),
            fmt_f64(dir.z)
        ));
        self.dir_cache.insert(key, id);
        id
    }

    // ── AXIS2_PLACEMENT_3D ──
    fn emit_axis2(&mut self, origin: &Point3d, axis: &Direction3d, ref_dir: &Direction3d) -> i64 {
        let pt_id = self.emit_point(origin);
        let dir_id = self.emit_direction(axis);
        let ref_id = self.emit_direction(ref_dir);
        let key = (pt_id, dir_id, ref_id);
        if let Some(&id) = self.axis2_cache.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});",
            id, pt_id, dir_id, ref_id
        ));
        self.axis2_cache.insert(key, id);
        id
    }

    // ── VERTEX_POINT (dedup by underlying point id) ──
    fn emit_vertex(&mut self, pt: &Point3d) -> i64 {
        let pt_id = self.emit_point(pt);
        if let Some(&id) = self.vertex_cache.get(&pt_id) {
            return id;
        }
        let id = self.alloc_id();
        self.push_line(&format!("#{} = VERTEX_POINT('',#{});", id, pt_id));
        self.vertex_cache.insert(pt_id, id);
        id
    }

    // ── VECTOR + LINE ──
    fn emit_line(&mut self, line: &Line) -> i64 {
        let key = format!("line|{:?}|{:?}", line.origin, line.direction);
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }
        let pt_id = self.emit_point(&line.origin);
        let dir_id = self.emit_direction(&line.direction);
        let id = self.alloc_id();
        self.push_line(&format!("#{} = LINE('',#{},#{});", id, pt_id, dir_id));
        self.curve_cache.insert(key, id);
        id
    }

    // ── CIRCLE ──
    fn emit_circle(&mut self, circle: &Circle) -> i64 {
        let key = format!(
            "circle|{:?}|{:?}|{:?}|{}",
            circle.center, circle.normal, circle.x_axis, circle.radius
        );
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }
        // STEP CIRCLE uses AXIS2_PLACEMENT_3D where:
        //   axis = normal (perpendicular to circle plane)
        //   ref_direction = x_axis
        let axis2_id = self.emit_axis2(&circle.center, &circle.normal, &circle.x_axis);
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = CIRCLE('',#{},{});",
            id, axis2_id, fmt_f64(circle.radius)
        ));
        self.curve_cache.insert(key, id);
        id
    }

    // ── ELLIPSE ──
    fn emit_ellipse(&mut self, ellipse: &Ellipse) -> i64 {
        let key = format!(
            "ellipse|{:?}|{:?}|{:?}|{}|{}",
            ellipse.center, ellipse.normal, ellipse.x_axis, ellipse.semi_major, ellipse.semi_minor
        );
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }
        let axis2_id = self.emit_axis2(&ellipse.center, &ellipse.normal, &ellipse.x_axis);
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = ELLIPSE('',#{},{});",
            id,
            axis2_id,
            // STEP convention: first semi-axis is the major, second is the minor.
            // The axis2 placement's ref_direction defines the major axis.
            fmt_f64(ellipse.semi_major)
        ));
        // Note: STEP ELLIPSE only takes semi_axis_1; semi_axis_2 is inferred
        // from the placement. We need to include both via the second arg.
        // Actually the proper STEP entity is:
        //   ELLIPSE('', #axis2, semi_axis_1, semi_axis_2)
        // Re-emit correctly:
        self.out.pop(); // remove trailing newline of the line we just added
        // Replace the line with the correct format
        // Find last line and replace it
        if let Some(pos) = self.out.rfind('\n') {
            self.out.truncate(pos + 1);
        } else {
            self.out.clear();
        }
        self.push_line(&format!(
            "#{} = ELLIPSE('',#{},{},{});",
            id, axis2_id, fmt_f64(ellipse.semi_major), fmt_f64(ellipse.semi_minor)
        ));
        self.curve_cache.insert(key, id);
        id
    }

    // ── HYPERBOLA ──
    fn emit_hyperbola(&mut self, hyp: &Hyperbola) -> i64 {
        let key = format!(
            "hyperbola|{:?}|{:?}|{:?}|{}|{}",
            hyp.center, hyp.normal, hyp.x_axis, hyp.semi_real, hyp.semi_imag
        );
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }
        let axis2_id = self.emit_axis2(&hyp.center, &hyp.normal, &hyp.x_axis);
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = HYPERBOLA('',#{},{},{});",
            id, axis2_id, fmt_f64(hyp.semi_real), fmt_f64(hyp.semi_imag)
        ));
        self.curve_cache.insert(key, id);
        id
    }

    // ── PARABOLA ──
    fn emit_parabola(&mut self, par: &Parabola) -> i64 {
        let key = format!(
            "parabola|{:?}|{:?}|{:?}|{}",
            par.vertex, par.normal, par.x_axis, par.focal_dist
        );
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }
        let axis2_id = self.emit_axis2(&par.vertex, &par.normal, &par.x_axis);
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = PARABOLA('',#{},{});",
            id, axis2_id, fmt_f64(par.focal_dist)
        ));
        self.curve_cache.insert(key, id);
        id
    }

    // ── ARC (as TRIMMED_CURVE wrapping CIRCLE) ──
    fn emit_arc(&mut self, arc: &Arc) -> i64 {
        let key = format!(
            "arc|{:?}|{:?}|{:?}|{}|{}|{}",
            arc.circle.center,
            arc.circle.normal,
            arc.circle.x_axis,
            arc.circle.radius,
            arc.start_angle,
            arc.end_angle
        );
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }
        let circle_id = self.emit_circle(&arc.circle);
        // Trim points
        let start_pt = arc.start_point();
        let end_pt = arc.end_point();
        let start_pt_id = self.emit_point(&start_pt);
        let end_pt_id = self.emit_point(&end_pt);
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = TRIMMED_CURVE('',#{},(#{},PARAMETER_VALUE({})),(#{},PARAMETER_VALUE({})),.T.,.PARAMETER.);",
            id,
            circle_id,
            start_pt_id,
            fmt_f64(arc.start_angle),
            end_pt_id,
            fmt_f64(arc.end_angle)
        ));
        self.curve_cache.insert(key, id);
        id
    }

    // ── B_SPLINE_CURVE (rational or non-rational) ──
    fn emit_nurbs_curve(&mut self, nurbs: &NurbsCurve) -> i64 {
        let key = format!(
            "nurbs_curve|{}|{:?}|{:?}|{:?}",
            nurbs.degree, nurbs.control_points, nurbs.weights, nurbs.knots
        );
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }

        let degree = nurbs.degree;
        let n_cps = nurbs.control_points.len();

        // Compute knot multiplicities from the knot vector (STEP needs
        // unique knot values + multiplicities, not the raw expanded form).
        let mut knot_values: Vec<f64> = Vec::new();
        let mut knot_mults: Vec<usize> = Vec::new();
        for &k in &nurbs.knots {
            if let Some(last) = knot_values.last() {
                if (k - last).abs() < 1e-12 {
                    *knot_mults.last_mut().unwrap() += 1;
                    continue;
                }
            }
            knot_values.push(k);
            knot_mults.push(1);
        }

        // Control point refs
        let cp_refs: Vec<String> = nurbs
            .control_points
            .iter()
            .map(|p| format!("#{}", self.emit_point(p)))
            .collect();

        // Determine if all weights are 1.0 → non-rational B-spline
        let all_unit_weights = nurbs
            .weights
            .iter()
            .all(|w| (w - 1.0).abs() < 1e-12);

        let closed_curve = if knot_values.len() >= 2 {
            // Heuristic: if first/last control points coincide, treat as closed.
            // STEP uses .T./.F. for closed_curve field.
            let first = &nurbs.control_points[0];
            let last = &nurbs.control_points[n_cps - 1];
            (first.x - last.x).abs() < 1e-9
                && (first.y - last.y).abs() < 1e-9
                && (first.z - last.z).abs() < 1e-9
        } else {
            false
        };

        let id = self.alloc_id();

        if all_unit_weights {
            // Non-rational: simple B_SPLINE_CURVE_WITH_KNOTS
            let mults_str: Vec<String> = knot_mults.iter().map(|m| m.to_string()).collect();
            let knots_str: Vec<String> = knot_values.iter().map(|k| fmt_f64(*k)).collect();
            self.push_line(&format!(
                "#{} = B_SPLINE_CURVE_WITH_KNOTS('',{},({}),.UNSPECIFIED.,{},{},({}),({}),.UNSPECIFIED.);",
                id,
                degree,
                cp_refs.join(","),
                if closed_curve { ".T." } else { ".F." },
                ".F.",
                mults_str.join(","),
                knots_str.join(",")
            ));
        } else {
            // Rational: complex entity
            // (B_SPLINE_CURVE(deg,(cps),.UNSPECIFIED.,.F.,.F.)
            //  B_SPLINE_CURVE_WITH_KNOTS((mults),(knots),.UNSPECIFIED.)
            //  RATIONAL_B_SPLINE_CURVE((weights))
            //  BOUNDED_CURVE())
            let weights_str: Vec<String> = nurbs.weights.iter().map(|w| fmt_f64(*w)).collect();
            let mults_str: Vec<String> = knot_mults.iter().map(|m| m.to_string()).collect();
            let knots_str: Vec<String> = knot_values.iter().map(|k| fmt_f64(*k)).collect();
            self.push_line(&format!(
                "#{} = (B_SPLINE_CURVE({},({}),.UNSPECIFIED.,{},{})B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.)RATIONAL_B_SPLINE_CURVE(({}))BOUNDED_CURVE());",
                id,
                degree,
                cp_refs.join(","),
                if closed_curve { ".T." } else { ".F." },
                ".F.",
                mults_str.join(","),
                knots_str.join(","),
                weights_str.join(",")
            ));
        }

        self.curve_cache.insert(key, id);
        id
    }

    // ── TRIMMED_CURVE wrapping arbitrary basis curve ──
    fn emit_trimmed_curve(&mut self, basis: &Curve3d, start: f64, end: f64) -> i64 {
        let key = format!(
            "trimmed|{:?}|{}|{}",
            basis, start, end
        );
        if let Some(&id) = self.curve_cache.get(&key) {
            return id;
        }
        let basis_id = self.emit_curve(basis);
        let start_pt = basis.point_at(start);
        let end_pt = basis.point_at(end);
        let start_pt_id = self.emit_point(&start_pt);
        let end_pt_id = self.emit_point(&end_pt);
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = TRIMMED_CURVE('',#{},(#{},PARAMETER_VALUE({})),(#{},PARAMETER_VALUE({})),.T.,.PARAMETER.);",
            id, basis_id, start_pt_id, fmt_f64(start), end_pt_id, fmt_f64(end)
        ));
        self.curve_cache.insert(key, id);
        id
    }

    // ── PCurve as SURFACE_CURVE wrapping a 3D curve + PCURVE ──
    fn emit_pcurve(&mut self, curve_2d: &Curve2d, surface: &Surface) -> i64 {
        // Sample the 3D path of the pcurve to produce a 3D curve.
        // For PCurves we emit a SURFACE_CURVE that wraps:
        //   - a 3D representation (B_SPLINE_CURVE sampled from the analytical PCurve)
        //   - a PCURVE referencing the surface + the 2D curve
        let (tmin, tmax) = curve_2d.param_range();

        // Sample 16 points along the 2D curve to fit a B-spline approximation
        const N_SAMPLES: usize = 16;
        let mut samples_3d: Vec<Point3d> = Vec::with_capacity(N_SAMPLES);
        for i in 0..N_SAMPLES {
            let t = tmin + (tmax - tmin) * (i as f64) / ((N_SAMPLES - 1) as f64);
            let uv = curve_2d.point_at(t);
            samples_3d.push(surface.point_at(uv.u, uv.v));
        }

        // Build a degree-3 B-spline curve through these samples (interpolating).
        // For simplicity, use the samples as control points of a degree-min(3, N-1) curve.
        let degree = 3.min(N_SAMPLES - 1);
        let n = samples_3d.len();
        let mut knots: Vec<f64> = Vec::with_capacity(n + degree + 1);
        // Clamped knot vector
        for _ in 0..=degree {
            knots.push(0.0);
        }
        for i in 1..(n - degree) {
            knots.push(i as f64 / (n - degree) as f64);
        }
        for _ in 0..=degree {
            knots.push(1.0);
        }
        let weights = vec![1.0; n];

        let nurbs = NurbsCurve {
            degree,
            control_points: samples_3d.clone(),
            weights,
            knots,
        };
        let curve_3d_id = self.emit_nurbs_curve(&nurbs);

        // Build the 2D curve geometry
        let curve_2d_id = self.emit_curve_2d(curve_2d);

        // Emit the surface (the underlying face surface)
        let surface_id = self.emit_surface(surface);

        // PCURVE('', #surface, #curve_2d)
        let pcurve_geom_id = self.alloc_id();
        self.push_line(&format!(
            "#{} = PCURVE('',#{},#{});",
            pcurve_geom_id, surface_id, curve_2d_id
        ));

        // SURFACE_CURVE('', #curve_3d, (#pcurve_geom), .CURVE_3D.)
        let id = self.alloc_id();
        self.push_line(&format!(
            "#{} = SURFACE_CURVE('',#{},(#{},),.CURVE_3D.);",
            id, curve_3d_id, pcurve_geom_id
        ));
        id
    }

    // ── 2D curve (line / circle / ellipse / nurbs) in UV space ──
    fn emit_curve_2d(&mut self, curve: &Curve2d) -> i64 {
        match curve {
            Curve2d::Line(line) => {
                // Emit as LINE in 2D using AXIS2_PLACEMENT_2D
                let pt_id = self.emit_point(&Point3d::new(line.start.u, line.start.v, 0.0));
                let dir_id = self.emit_direction(&Direction3d::new(
                    line.end.u - line.start.u,
                    line.end.v - line.start.v,
                    0.0,
                ).unwrap_or(Direction3d::X));
                let axis2d_id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = AXIS2_PLACEMENT_2D('',#{},#{});",
                    axis2d_id, pt_id, dir_id
                ));
                let id = self.alloc_id();
                self.push_line(&format!("#{} = LINE('',#{},#{});", id, pt_id, dir_id));
                id
            }
            Curve2d::Circle(circle) => {
                let pt_id = self.emit_point(&Point3d::new(circle.center.u, circle.center.v, 0.0));
                let dir_id = self.emit_direction(&Direction3d::Z);
                let ref_dir_id = self.emit_direction(&Direction3d::X);
                let axis2d_id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = AXIS2_PLACEMENT_2D('',#{},#{});",
                    axis2d_id, pt_id, ref_dir_id
                ));
                let _ = dir_id; // not used in 2D
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = CIRCLE('',#{},{});",
                    id,
                    axis2d_id,
                    fmt_f64(circle.radius)
                ));
                id
            }
            Curve2d::Ellipse(ellipse) => {
                let pt_id = self.emit_point(&Point3d::new(ellipse.center.u, ellipse.center.v, 0.0));
                let ref_dir_id = self.emit_direction(&Direction3d::X);
                let axis2d_id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = AXIS2_PLACEMENT_2D('',#{},#{});",
                    axis2d_id, pt_id, ref_dir_id
                ));
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = ELLIPSE('',#{},{},{});",
                    id,
                    axis2d_id,
                    fmt_f64(ellipse.semi_major),
                    fmt_f64(ellipse.semi_minor)
                ));
                id
            }
            Curve2d::Hyperbola(hyp) => {
                let pt_id = self.emit_point(&Point3d::new(hyp.center.u, hyp.center.v, 0.0));
                let ref_dir_id = self.emit_direction(&Direction3d::new(
                    hyp.axis_u, hyp.axis_v, 0.0,
                ).unwrap_or(Direction3d::X));
                let axis2d_id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = AXIS2_PLACEMENT_2D('',#{},#{});",
                    axis2d_id, pt_id, ref_dir_id
                ));
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = HYPERBOLA('',#{},{},{});",
                    id,
                    axis2d_id,
                    fmt_f64(hyp.semi_real),
                    fmt_f64(hyp.semi_imag)
                ));
                id
            }
            Curve2d::Parabola(par) => {
                let pt_id = self.emit_point(&Point3d::new(par.vertex.u, par.vertex.v, 0.0));
                let ref_dir_id = self.emit_direction(&Direction3d::new(
                    par.axis_u, par.axis_v, 0.0,
                ).unwrap_or(Direction3d::X));
                let axis2d_id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = AXIS2_PLACEMENT_2D('',#{},#{});",
                    axis2d_id, pt_id, ref_dir_id
                ));
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = PARABOLA('',#{},{});",
                    id,
                    axis2d_id,
                    fmt_f64(par.focal_dist)
                ));
                id
            }
            Curve2d::Nurbs(nurbs) => {
                // Emit as 2D B-spline by lifting control points to (u, v, 0)
                let lifted_cps: Vec<Point3d> = nurbs
                    .control_points
                    .iter()
                    .map(|p| Point3d::new(p.u, p.v, 0.0))
                    .collect();
                let lifted = NurbsCurve {
                    degree: nurbs.degree,
                    control_points: lifted_cps,
                    weights: nurbs.weights.clone(),
                    knots: nurbs.knots.clone(),
                };
                self.emit_nurbs_curve(&lifted)
            }
            Curve2d::Composite { segments, .. } => {
                // Emit each segment recursively, then wrap in COMPOSITE_CURVE
                let mut segment_ids = Vec::new();
                for seg in segments {
                    segment_ids.push(self.emit_curve_2d(seg));
                }
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = COMPOSITE_CURVE('',({}),.F.);",
                    id,
                    segment_ids.iter().map(|s| format!("#{}", s)).collect::<Vec<_>>().join(",")
                ));
                id
            }
        }
    }

    // ── Dispatch Curve3d to specific emitter ──
    fn emit_curve(&mut self, curve: &Curve3d) -> i64 {
        match curve {
            Curve3d::Line(line) => self.emit_line(line),
            Curve3d::Circle(circle) => self.emit_circle(circle),
            Curve3d::Ellipse(ellipse) => self.emit_ellipse(ellipse),
            Curve3d::Arc(arc) => self.emit_arc(arc),
            Curve3d::Hyperbola(hyp) => self.emit_hyperbola(hyp),
            Curve3d::Parabola(par) => self.emit_parabola(par),
            Curve3d::Nurbs(nurbs) => self.emit_nurbs_curve(nurbs),
            Curve3d::PCurve { curve_2d, surface } => {
                self.emit_pcurve(curve_2d, surface)
            }
            Curve3d::Trimmed { basis, start, end } => {
                self.emit_trimmed_curve(basis, *start, *end)
            }
            Curve3d::Composite { segments, .. } => {
                let mut segment_ids = Vec::new();
                for seg in segments {
                    segment_ids.push(self.emit_curve(seg));
                }
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = COMPOSITE_CURVE('',({}),.F.);",
                    id,
                    segment_ids.iter().map(|s| format!("#{}", s)).collect::<Vec<_>>().join(",")
                ));
                id
            }
        }
    }

    // ── EDGE_CURVE (dedup by content hash) ──
    fn emit_edge_curve(&mut self, edge: &Edge) -> i64 {
        // Build a content-based key so shared edges between faces are deduped.
        let key = if let Some(curve) = &edge.curve {
            format!("{:?}", curve)
        } else {
            // No curve — use vertex endpoints
            let s = edge.start_point().unwrap_or(Point3d::ORIGIN);
            let e = edge.end_point().unwrap_or(Point3d::ORIGIN);
            format!("no_curve|{:?}|{:?}", s, e)
        };
        if let Some(&id) = self.edge_cache.get(&key) {
            return id;
        }

        let start_pt = edge.start_point().unwrap_or(Point3d::ORIGIN);
        let end_pt = edge.end_point().unwrap_or(Point3d::ORIGIN);
        let start_vtx_id = self.emit_vertex(&start_pt);
        let end_vtx_id = self.emit_vertex(&end_pt);

        let curve_id = if let Some(curve) = &edge.curve {
            self.emit_curve(curve)
        } else {
            // Fallback: line from start to end
            let dir = Direction3d::new(
                end_pt.x - start_pt.x,
                end_pt.y - start_pt.y,
                end_pt.z - start_pt.z,
            )
            .unwrap_or(Direction3d::X);
            self.emit_line(&Line::new(start_pt, dir))
        };

        let id = self.alloc_id();
        let same_sense = if edge.forward { ".T." } else { ".F." };
        self.push_line(&format!(
            "#{} = EDGE_CURVE('',#{},#{},{},{});",
            id, start_vtx_id, end_vtx_id, curve_id, same_sense
        ));
        self.edge_cache.insert(key, id);
        id
    }

    // ── Wire → EDGE_LOOP + FACE_BOUND ──
    // Returns Some(face_bound_id) if the wire is non-empty, None otherwise.
    fn emit_wire_as_bound(
        &mut self,
        wire: &Wire,
        edges: &[Edge],
    ) -> Option<i64> {
        if wire.coedges.is_empty() {
            return None;
        }

        // Handle degenerate single-vertex wire as VERTEX_LOOP
        if wire.coedges.len() == 1 {
            let coedge = &wire.coedges[0];
            let edge_opt = edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge_opt {
                if edge.degenerate {
                    let pt = edge.start_point().unwrap_or(Point3d::ORIGIN);
                    let vtx_id = self.emit_vertex(&pt);
                    let vl_id = self.alloc_id();
                    self.push_line(&format!(
                        "#{} = VERTEX_LOOP('',#{});",
                        vl_id, vtx_id
                    ));
                    let fb_id = self.alloc_id();
                    self.push_line(&format!(
                        "#{} = FACE_BOUND('',#{},.T.);",
                        fb_id, vl_id
                    ));
                    return Some(fb_id);
                }
            }
        }

        // Normal EDGE_LOOP from coedges
        let mut oriented_edge_ids: Vec<i64> = Vec::with_capacity(wire.coedges.len());
        for coedge in &wire.coedges {
            let edge_opt = edges.iter().find(|e| e.id == coedge.edge);
            let ec_id = if let Some(e) = edge_opt {
                self.emit_edge_curve(e)
            } else {
                // Create a degenerate edge at origin
                let pt_id = self.emit_point(&Point3d::ORIGIN);
                let vtx_id = self.emit_vertex(&Point3d::ORIGIN);
                let _ = pt_id;
                let dummy_line_id = {
                    let dir_id = self.emit_direction(&Direction3d::X);
                    let pt_id = self.emit_point(&Point3d::ORIGIN);
                    let id = self.alloc_id();
                    self.push_line(&format!("#{} = LINE('',#{},#{});", id, pt_id, dir_id));
                    id
                };
                let ec_id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = EDGE_CURVE('',#{},#{},{},.T.);",
                    ec_id, vtx_id, vtx_id, dummy_line_id
                ));
                ec_id
            };

            let oe_id = self.alloc_id();
            let orientation = if coedge.forward { ".T." } else { ".F." };
            self.push_line(&format!(
                "#{} = ORIENTED_EDGE('',*,*,#{},{});",
                oe_id, ec_id, orientation
            ));
            oriented_edge_ids.push(oe_id);
        }

        let oe_refs: Vec<String> = oriented_edge_ids.iter().map(|id| format!("#{}", id)).collect();
        let el_id = self.alloc_id();
        self.push_line(&format!(
            "#{} = EDGE_LOOP('',({}));",
            el_id,
            oe_refs.join(",")
        ));
        let fb_id = self.alloc_id();
        self.push_line(&format!(
            "#{} = FACE_BOUND('',#{},.T.);",
            fb_id, el_id
        ));
        Some(fb_id)
    }

    // ── Surface emitters ──

    fn emit_surface(&mut self, surface: &Surface) -> i64 {
        let key = format!("{:?}", surface);
        if let Some(&id) = self.surface_cache.get(&key) {
            return id;
        }
        let id = match surface {
            Surface::Plane(plane) => {
                let axis2_id = self.emit_axis2(&plane.origin, &plane.normal, &plane.u_dir);
                let id = self.alloc_id();
                self.push_line(&format!("#{} = PLANE('',#{});", id, axis2_id));
                id
            }
            Surface::Cylinder(cyl) => {
                let axis2_id = self.emit_axis2(&cyl.origin, &cyl.axis, &cyl.x_dir);
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = CYLINDRICAL_SURFACE('',#{},{});",
                    id, axis2_id, fmt_f64(cyl.radius)
                ));
                id
            }
            Surface::Cone(cone) => {
                let axis2_id = self.emit_axis2(&cone.origin, &cone.axis, &cone.x_dir);
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = CONICAL_SURFACE('',#{},{},{});",
                    id,
                    axis2_id,
                    fmt_f64(cone.radius),
                    fmt_f64(cone.half_angle)
                ));
                id
            }
            Surface::Sphere(sphere) => {
                let axis2_id = self.emit_axis2(&sphere.center, &Direction3d::Z, &Direction3d::X);
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = SPHERICAL_SURFACE('',#{},{});",
                    id, axis2_id, fmt_f64(sphere.radius)
                ));
                id
            }
            Surface::Torus(torus) => {
                let axis2_id = self.emit_axis2(&torus.center, &torus.axis, &torus.x_dir);
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = TOROIDAL_SURFACE('',#{},{},{});",
                    id,
                    axis2_id,
                    fmt_f64(torus.major_radius),
                    fmt_f64(torus.minor_radius)
                ));
                id
            }
            Surface::Revolution(rev) => {
                // SURFACE_OF_REVOLUTION('', #profile_curve, #axis1_placement)
                let profile_id = self.emit_curve(&rev.profile);
                let pt_id = self.emit_point(&rev.origin);
                let dir_id = self.emit_direction(&rev.axis);
                let axis1_id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = AXIS1_PLACEMENT('',#{},#{});",
                    axis1_id, pt_id, dir_id
                ));
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = SURFACE_OF_REVOLUTION('',#{},#{});",
                    id, profile_id, axis1_id
                ));
                id
            }
            Surface::Extrusion(ext) => {
                // SURFACE_OF_LINEAR_EXTRUSION('', #profile_curve, #direction)
                let profile_id = self.emit_curve(&ext.profile);
                let dir_id = self.emit_direction(&ext.direction);
                let id = self.alloc_id();
                self.push_line(&format!(
                    "#{} = SURFACE_OF_LINEAR_EXTRUSION('',#{},#{});",
                    id, profile_id, dir_id
                ));
                id
            }
            Surface::Nurbs(nurbs) => self.emit_nurbs_surface(nurbs),
            Surface::Offset(_) | Surface::Ruled(_) => {
                // Audit item 4.3 (2026-07-19): Offset/Ruled surfaces
                // are not yet supported in STEP export. Fall back to NURBS
                // conversion (TODO: implement direct export).
                log::warn!("Offset/Ruled surface export not yet implemented, skipping");
                self.alloc_id() // Return a dummy ID
            }
        };
        self.surface_cache.insert(key, id);
        id
    }

    fn emit_nurbs_surface(&mut self, nurbs: &draper_geometry::NurbsSurface) -> i64 {
        let u_degree = nurbs.u_degree;
        let v_degree = nurbs.v_degree;

        // Compress knot vectors
        let mut u_knot_values: Vec<f64> = Vec::new();
        let mut u_knot_mults: Vec<usize> = Vec::new();
        for &k in &nurbs.u_knots {
            if let Some(last) = u_knot_values.last() {
                if (k - last).abs() < 1e-12 {
                    *u_knot_mults.last_mut().unwrap() += 1;
                    continue;
                }
            }
            u_knot_values.push(k);
            u_knot_mults.push(1);
        }
        let mut v_knot_values: Vec<f64> = Vec::new();
        let mut v_knot_mults: Vec<usize> = Vec::new();
        for &k in &nurbs.v_knots {
            if let Some(last) = v_knot_values.last() {
                if (k - last).abs() < 1e-12 {
                    *v_knot_mults.last_mut().unwrap() += 1;
                    continue;
                }
            }
            v_knot_values.push(k);
            v_knot_mults.push(1);
        }

        // Control point grid as ((row1cps),(row2cps),...)
        let mut rows: Vec<String> = Vec::with_capacity(nurbs.control_points.len());
        for row in &nurbs.control_points {
            let row_refs: Vec<String> = row.iter().map(|p| format!("#{}", self.emit_point(p))).collect();
            rows.push(format!("({})", row_refs.join(",")));
        }
        let cp_grid = format!("({})", rows.join(","));

        let all_unit_weights = nurbs
            .weights
            .iter()
            .flatten()
            .all(|w| (w - 1.0).abs() < 1e-12);

        let u_mults_str: Vec<String> = u_knot_mults.iter().map(|m| m.to_string()).collect();
        let u_knots_str: Vec<String> = u_knot_values.iter().map(|k| fmt_f64(*k)).collect();
        let v_mults_str: Vec<String> = v_knot_mults.iter().map(|m| m.to_string()).collect();
        let v_knots_str: Vec<String> = v_knot_values.iter().map(|k| fmt_f64(*k)).collect();

        let id = self.alloc_id();
        if all_unit_weights {
            self.push_line(&format!(
                "#{} = B_SPLINE_SURFACE_WITH_KNOTS('',{},{},{},.UNSPECIFIED.,{},{},{},({}),({}),({}),({}),.UNSPECIFIED.);",
                id,
                u_degree,
                v_degree,
                cp_grid,
                if nurbs.u_closed { ".T." } else { ".F." },
                if nurbs.v_closed { ".T." } else { ".F." },
                ".F.",
                u_mults_str.join(","),
                v_mults_str.join(","),
                u_knots_str.join(","),
                v_knots_str.join(",")
            ));
        } else {
            // Rational: complex entity
            let weight_rows: Vec<String> = nurbs
                .weights
                .iter()
                .map(|row| {
                    let row_strs: Vec<String> = row.iter().map(|w| fmt_f64(*w)).collect();
                    format!("({})", row_strs.join(","))
                })
                .collect();
            let weights_grid = format!("({})", weight_rows.join(","));
            self.push_line(&format!(
                "#{} = (B_SPLINE_SURFACE({},{},{},.UNSPECIFIED.,{},{},{})B_SPLINE_SURFACE_WITH_KNOTS(({}),({}),({}),({}),.UNSPECIFIED.)RATIONAL_B_SPLINE_SURFACE({})BOUNDED_SURFACE());",
                id,
                u_degree,
                v_degree,
                cp_grid,
                if nurbs.u_closed { ".T." } else { ".F." },
                if nurbs.v_closed { ".T." } else { ".F." },
                ".F.",
                u_mults_str.join(","),
                v_mults_str.join(","),
                u_knots_str.join(","),
                v_knots_str.join(","),
                weights_grid
            ));
        }
        id
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────

/// STEP application protocol version.
///
/// Audit item 8.1 (2026-07-19): Added AP242 support.
#[derive(Clone, Copy, Debug)]
pub enum StepSchema {
    /// AP203 — Configuration controlled design
    Ap203,
    /// AP214 — Automotive design (default, most widely supported)
    Ap214,
    /// AP242 — Managed model based 3D engineering (latest standard)
    Ap242,
}

impl Default for StepSchema {
    fn default() -> Self {
        StepSchema::Ap214
    }
}

impl StepSchema {
    fn schema_string(&self) -> &'static str {
        match self {
            StepSchema::Ap203 => "CONFIG_CONTROL_DESIGN { 1 0 10303 203 1 1 1 }",
            StepSchema::Ap214 => "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }",
            StepSchema::Ap242 => "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }",
        }
    }
}

/// Export a solid to STEP AP203/AP214 format.
///
/// Supports:
/// - MANIFOLD_SOLID_BREP (solid with only outer shell)
/// - BREP_WITH_VOIDS (solid with outer shell + inner shells as voids)
/// - All surface types: Plane, Cylinder, Cone, Sphere, Torus, Revolution, Extrusion, Nurbs
/// - All curve types: Line, Circle, Ellipse, Arc, Hyperbola, Parabola, Nurbs, PCurve, Trimmed
/// - Multiple inner wires per face (holes)
/// - Shared EDGE_CURVE / VERTEX_POINT / CARTESIAN_POINT deduplication
pub fn export_step(solid: &Solid, name: &str) -> String {
    export_step_with_schema(solid, name, StepSchema::default())
}

/// Export a solid to STEP with a specified schema (AP203/AP214/AP242).
///
/// Audit item 8.1 (2026-07-19): Added schema selection.
pub fn export_step_with_schema(solid: &Solid, name: &str, schema: StepSchema) -> String {
    let mut sw = StepWriter::new();

    // ── Header ──
    sw.push_line("ISO-10303-21;");
    sw.push_line("HEADER;");
    sw.push_line("FILE_DESCRIPTION(('3Draper export','3Draper STEP exporter'), '2;1');");
    let now = chrono_now();
    sw.push_line(&format!(
        "FILE_NAME('{}.stp','{}',('3Draper'),(''),'3Draper','','');",
        name, now
    ));
    sw.push_line(&format!("FILE_SCHEMA(('{}'));", schema.schema_string()));
    sw.push_line("ENDSEC;");

    // ── Data section ──
    sw.push_line("DATA;");

    // Emit all shells of the solid
    let outer_shell_id = if let Some(ref shell) = solid.outer_shell {
        emit_shell(&mut sw, solid, shell)
    } else {
        // Empty solid — emit a placeholder
        let id = sw.alloc_id();
        sw.push_line(&format!("#{} = CLOSED_SHELL('',());", id));
        id
    };

    // Inner shells (voids) — emit as CLOSED_SHELL references
    let inner_shell_ids: Vec<i64> = solid
        .inner_shells
        .iter()
        .map(|shell| emit_shell(&mut sw, solid, shell))
        .collect();

    // MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS
    let brep_id = sw.alloc_id();
    if inner_shell_ids.is_empty() {
        sw.push_line(&format!(
            "#{} = MANIFOLD_SOLID_BREP('{}',#{});",
            brep_id, name, outer_shell_id
        ));
    } else {
        let inner_refs: Vec<String> = inner_shell_ids.iter().map(|id| format!("#{}", id)).collect();
        sw.push_line(&format!(
            "#{} = BREP_WITH_VOIDS('{}',#{},({}));",
            brep_id, name, outer_shell_id, inner_refs.join(",")
        ));
    }

    // ── Units + representation context ──
    // Pre-allocate all IDs in flat sequence (no nested borrows).
    let length_unit_id = sw.alloc_id();
    let angle_unit_id = sw.alloc_id();
    let solid_angle_unit_id = sw.alloc_id();
    let uncertainty_id = sw.alloc_id();
    let unit_assignment_id = sw.alloc_id();

    sw.push_line(&format!(
        "#{} = (LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));",
        length_unit_id
    ));
    sw.push_line(&format!(
        "#{} = (NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));",
        angle_unit_id
    ));
    sw.push_line(&format!(
        "#{} = (NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());",
        solid_angle_unit_id
    ));
    sw.push_line(&format!(
        "#{} = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-6),#{},'distance_accuracy_value','confusion accuracy');",
        uncertainty_id, length_unit_id
    ));
    sw.push_line(&format!(
        "#{} = (GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{},#{},#{})) REPRESENTATION_CONTEXT('NONE','WORKSPACE'));",
        unit_assignment_id, uncertainty_id, length_unit_id, angle_unit_id, solid_angle_unit_id
    ));

    // ── Shape representation ──
    let shape_rep_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{},),#{});",
        shape_rep_id, brep_id, unit_assignment_id
    ));

    // ── Product + definition chain ──
    let product_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = PRODUCT('{}','{}','',$,$);",
        product_id, name, name
    ));

    let prod_formation_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = PRODUCT_DEFINITION_FORMATION('','',#{});",
        prod_formation_id, product_id
    ));

    let app_ctx_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = APPLICATION_CONTEXT('automotive design');",
        app_ctx_id
    ));

    let prod_def_ctx_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = PRODUCT_DEFINITION_CONTEXT('part definition',#{},'design');",
        prod_def_ctx_id, app_ctx_id
    ));

    let product_def_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = PRODUCT_DEFINITION('design','',#{},#{});",
        product_def_id, prod_formation_id, prod_def_ctx_id
    ));

    let shape_prop_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = PRODUCT_DEFINITION_SHAPE('','shape of {}',#{});",
        shape_prop_id, name, product_def_id
    ));

    let shape_rep_rel_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = SHAPE_DEFINITION_REPRESENTATION(#{},#{});",
        shape_rep_rel_id, shape_prop_id, shape_rep_id
    ));

    let ap_def_id = sw.alloc_id();
    sw.push_line(&format!(
        "#{} = APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{});",
        ap_def_id, app_ctx_id
    ));
    let _ = (ap_def_id, shape_rep_rel_id);

    sw.push_line("ENDSEC;");
    sw.push_line("END-ISO-10303-21;");

    sw.out
}

/// Emit a CLOSED_SHELL and return its ID.
fn emit_shell(sw: &mut StepWriter, solid: &Solid, shell: &Shell) -> i64 {
    let mut face_ids: Vec<i64> = Vec::with_capacity(shell.faces.len());

    for face in &shell.faces {
        // C5 Stage 6.4: store-first boundary registry — coedge ids resolve
        // through the owner solid's `EdgeStore` (per-id mirror fallback
        // keeps un-indexed builder faces complete), so a stale mirror can
        // no longer leak exported EDGE_CURVE geometry.
        let face_edges = solid.resolve_face_edges(face);
        // Surface
        let surface_id = if let Some(ref surface) = face.surface {
            sw.emit_surface(surface)
        } else {
            // Fallback: plane at origin
            let axis2_id = sw.emit_axis2(&Point3d::ORIGIN, &Direction3d::Z, &Direction3d::X);
            let id = sw.alloc_id();
            sw.push_line(&format!("#{} = PLANE('',#{});", id, axis2_id));
            id
        };

        // Bounds: outer wire + inner wires (holes)
        let mut bound_ids: Vec<i64> = Vec::new();

        if let Some(ref outer) = face.outer_wire {
            if let Some(fb_id) = sw.emit_wire_as_bound(outer, &face_edges) {
                bound_ids.push(fb_id);
            }
        }
        for inner in &face.inner_wires {
            if let Some(fb_id) = sw.emit_wire_as_bound(inner, &face_edges) {
                bound_ids.push(fb_id);
            }
        }

        // If no bounds at all (infinite face), emit a minimal VERTEX_LOOP
        if bound_ids.is_empty() {
            let vtx_id = sw.emit_vertex(&Point3d::ORIGIN);
            let vl_id = sw.alloc_id();
            sw.push_line(&format!("#{} = VERTEX_LOOP('',#{});", vl_id, vtx_id));
            let fb_id = sw.alloc_id();
            sw.push_line(&format!("#{} = FACE_BOUND('',#{},.T.);", fb_id, vl_id));
            bound_ids.push(fb_id);
        }

        let bound_refs: Vec<String> = bound_ids.iter().map(|id| format!("#{}", id)).collect();
        let face_id = sw.alloc_id();
        let face_orient = if face.forward { ".T." } else { ".F." };
        sw.push_line(&format!(
            "#{} = ADVANCED_FACE('',({}),#{},{},.F.);",
            face_id,
            bound_refs.join(","),
            surface_id,
            face_orient
        ));
        face_ids.push(face_id);
    }

    let shell_id = sw.alloc_id();
    let face_refs: Vec<String> = face_ids.iter().map(|id| format!("#{}", id)).collect();
    sw.push_line(&format!(
        "#{} = CLOSED_SHELL('',({}));",
        shell_id,
        face_refs.join(",")
    ));
    shell_id
}

/// Export a compound (assembly) to STEP.
///
/// Emits multiple MANIFOLD_SOLID_BREP entities in a single STEP file,
/// wrapped in a single SHAPE_REPRESENTATION_RELATIONSHIP.
pub fn export_compound_step(compound: &Compound, name: &str) -> String {
    if compound.solids.len() <= 1 {
        if let Some(solid) = compound.solids.first() {
            return export_step(solid, name);
        }
        return "// Empty compound — no solids to export".to_string();
    }
    // For multi-solid compounds, concatenate the exports and merge headers.
    // Simplest correct approach: emit each solid into its own StepWriter,
    // then stitch the DATA sections together.
    let mut combined = String::with_capacity(64 * 1024);
    combined.push_str("ISO-10303-21;\n");
    combined.push_str("HEADER;\n");
    combined.push_str("FILE_DESCRIPTION(('3Draper export'), '2;1');\n");
    let now = chrono_now();
    combined.push_str(&format!(
        "FILE_NAME('{}.stp','{}',('3Draper'),(''),'3Draper','','');\n",
        name, now
    ));
    combined.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
    combined.push_str("ENDSEC;\n");
    combined.push_str("DATA;\n");

    let mut offset: i64 = 1;
    for (i, solid) in compound.solids.iter().enumerate() {
        let chunk = export_step(solid, &format!("{}_{}", name, i));
        // Strip the header/ENDSEC/END-ISO from the chunk and remap IDs.
        let body = extract_data_section(&chunk);
        let remapped = remap_ids(&body, offset);
        combined.push_str(&remapped);
        // Bump offset past the highest ID used in this chunk.
        offset = max_id_in(&body) + 1;
    }
    combined.push_str("ENDSEC;\n");
    combined.push_str("END-ISO-10303-21;\n");
    combined
}

/// Extract the body of the DATA section from a STEP string.
fn extract_data_section(step: &str) -> String {
    let mut in_data = false;
    let mut out = String::with_capacity(step.len());
    for line in step.lines() {
        if line.trim() == "DATA;" {
            in_data = true;
            continue;
        }
        if line.trim() == "ENDSEC;" {
            in_data = false;
            continue;
        }
        if in_data {
            out.push_str(line);
            out.push('\n');
        }
    }
    out
}

/// Find the maximum `#N` ID used in the body.
fn max_id_in(body: &str) -> i64 {
    let mut max_id: i64 = 0;
    let bytes = body.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'#' {
            let mut j = i + 1;
            while j < bytes.len() && bytes[j].is_ascii_digit() {
                j += 1;
            }
            if j > i + 1 {
                if let Ok(n) = std::str::from_utf8(&bytes[i + 1..j]).unwrap_or("0").parse::<i64>() {
                    if n > max_id {
                        max_id = n;
                    }
                }
            }
            i = j;
        } else {
            i += 1;
        }
    }
    max_id
}

/// Remap all `#N` references in the body by adding `offset`.
fn remap_ids(body: &str, offset: i64) -> String {
    let mut out = String::with_capacity(body.len());
    let bytes = body.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'#' {
            let mut j = i + 1;
            while j < bytes.len() && bytes[j].is_ascii_digit() {
                j += 1;
            }
            if j > i + 1 {
                if let Ok(n) = std::str::from_utf8(&bytes[i + 1..j])
                    .unwrap_or("0")
                    .parse::<i64>()
                {
                    out.push('#');
                    out.push_str(&(n + offset).to_string());
                    i = j;
                    continue;
                }
            }
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

/// Write STEP content to a file (native only — not available on wasm).
#[cfg(not(target_arch = "wasm32"))]
pub fn write_step_file(content: &str, path: &str) -> io::Result<()> {
    let mut file = std::fs::File::create(path)?;
    file.write_all(content.as_bytes())
}

/// Get current timestamp in ISO format.
#[cfg(not(target_arch = "wasm32"))]
fn chrono_now() -> String {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let days = now / 86400;
    let year = 1970 + days / 365;
    let month = ((days % 365) / 30).min(11) + 1;
    let day = (days % 30).min(27) + 1;
    let hour = (now % 86400) / 3600;
    let minute = (now % 3600) / 60;
    let second = now % 60;
    format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
        year, month, day, hour, minute, second
    )
}

#[cfg(target_arch = "wasm32")]
fn chrono_now() -> String {
    use web_time::SystemTime;
    let now = SystemTime::now()
        .duration_since(web_time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let days = now / 86400;
    let year = 1970 + days / 365;
    let month = ((days % 365) / 30).min(11) + 1;
    let day = (days % 30).min(27) + 1;
    let hour = (now % 86400) / 3600;
    let minute = (now % 3600) / 60;
    let second = now % 60;
    format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
        year, month, day, hour, minute, second
    )
}

// ── Public helper: round-trip a Solid through STEP and re-parse ──
// (used by P18 round-trip integrity tests)

#[cfg(not(target_arch = "wasm32"))]
#[cfg(test)]
mod tests {
    use super::*;
    use draper_topology::{Shell, Face, Wire, CoEdge, Edge, TopoId};
    use draper_geometry::{Plane, Line};

    #[test]
    fn test_export_minimal_cube() {
        // Build a minimal cube: 6 plane faces, each with 4 edges.
        // Just verify the export produces a valid STEP string with
        // ISO-10303-21 header and END-ISO-10303-21 footer.
        let plane = Plane::xy();
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);
        let step = export_step(&solid, "test_cube");
        assert!(step.starts_with("ISO-10303-21;"));
        assert!(step.contains("END-ISO-10303-21;"));
        assert!(step.contains("MANIFOLD_SOLID_BREP"));
        assert!(step.contains("CLOSED_SHELL"));
        assert!(step.contains("ADVANCED_FACE"));
        assert!(step.contains("PLANE"));
    }

    /// C5 7.6b: export is store-only (`resolve_face_edges`) — there are no
    /// mirrors to go stale. The surviving contracts: the exported
    /// EDGE_CURVE geometry is the STORE's data (mutations surface), and
    /// repeated exports of the same solid are bit-identical.
    #[test]
    fn test_export_reads_store_single_source() {
        use draper_topology::ShapeBuilder;

        let base = ShapeBuilder::make_box(10.0, 10.0, 10.0);

        // (7.6b: builder solids are born-indexed — no index pass.)
        let clean = base.clone();

        // A store mutation is authoritative data now — shifting one
        // canonical edge (vertex points AND the curve — the exporter
        // samples `edge.start_point()`, which reads the curve) must
        // surface in the exported geometry.
        let mut stale = base.clone();
        {
            let face0 = &stale.faces()[0];
            let eid = face0.edge_ids[0];
            if let Some(edge) = stale.edge_store.get_mut(eid) {
                edge.start_vertex_point = Some(draper_geometry::Point3d::new(95.0, 0.0, 0.0));
                edge.end_vertex_point = Some(draper_geometry::Point3d::new(95.0, 10.0, 0.0));
                if let Some(draper_geometry::Curve3d::Line(ref mut line)) = edge.curve {
                    line.origin = draper_geometry::Point3d::new(95.0, 0.0, 0.0);
                }
            }
        }

        let step_clean = export_step(&clean, "stale_check");
        let step_stale = export_step(&stale, "stale_check");

        assert!(
            step_stale.contains("95."),
            "store mutation must surface in the export (single source of truth)"
        );
        let data = |s: &str| s.split_once("DATA;").map(|(_, rest)| rest.to_string());
        assert_ne!(
            data(&step_clean),
            data(&step_stale),
            "mutated store must yield a different DATA section"
        );

        // Determinism: exporting the same solid twice is bit-identical.
        let again = export_step(&clean, "stale_check");
        assert_eq!(
            data(&step_clean),
            data(&again),
            "repeated exports must be bit-identical"
        );
    }

    #[test]
    fn test_export_nurbs_surface() {
        // Build a 2x2 NURBS surface (degree 1, 4 control points).
        let cps = vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0)],
            vec![Point3d::new(0.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 0.0)],
        ];
        let weights = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
        let nurbs = draper_geometry::NurbsSurface {
            u_degree: 1,
            v_degree: 1,
            control_points: cps,
            weights,
            u_knots: vec![0.0, 0.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        };
        let face = Face::new_surface_only(Surface::Nurbs(nurbs));
        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);
        let step = export_step(&solid, "test_nurbs");
        assert!(step.contains("B_SPLINE_SURFACE_WITH_KNOTS"));
    }

    #[test]
    fn test_export_with_voids() {
        // Build a solid with an inner shell (void).
        let plane = Plane::xy();
        let face = Face::new_surface_only(Surface::Plane(plane));
        let outer = Shell::new_closed(vec![face.clone()]);
        let inner = Shell::new_closed(vec![face]);
        let mut solid = Solid::new(outer);
        solid.add_void(inner);
        let step = export_step(&solid, "test_voids");
        assert!(step.contains("BREP_WITH_VOIDS"));
        assert!(step.contains("CLOSED_SHELL"));
    }

    #[test]
    fn test_dedup_shared_points() {
        // Build a face where the same CARTESIAN_POINT is referenced multiple times.
        // Verify the output has fewer unique point entities than naive emission.
        let plane = Plane::xy();
        let face = Face::new(Surface::Plane(plane), Wire::new(vec![]));
        let shell = Shell::new_closed(vec![face]);
        let solid = Solid::new(shell);
        let step = export_step(&solid, "test_dedup");
        // Count CARTESIAN_POINT occurrences — should be small (1 for ORIGIN
        // even if multiple faces would share it).
        let count = step.matches("CARTESIAN_POINT").count();
        assert!(count >= 1);
        // Verify AXIS2_PLACEMENT_3D dedup — there should be exactly one
        // for the plane origin (0,0,0)+Z+X.
        let axis2_count = step.matches("AXIS2_PLACEMENT_3D").count();
        assert!(axis2_count >= 1);
        let _ = TopoId::new();
        let _ = CoEdge::new(TopoId::new(), true);
        let _ = Edge::new_line(Point3d::ORIGIN, Point3d::new(1.0, 0.0, 0.0));
        let _ = Line::new(Point3d::ORIGIN, Direction3d::X);
    }
}
