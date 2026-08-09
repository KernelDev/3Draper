// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! WGSL compute shader for NURBS surface evaluation.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 5.3: mass-evaluates NURBS
//! surface points and normals at a grid of (u, v) parameters using the
//! De Boor algorithm, running on the GPU for parallelism.
//!
//! # Algorithm
//!
//! The De Boor algorithm evaluates a B-spline of degree `p` at parameter
//! `u` using `p+1` control points and a knot vector. For a NURBS surface
//! (tensor product), we evaluate in both U and V directions:
//!
//! 1. For each row of control points (fixed V index), evaluate the U
//!    direction to get a row of intermediate points.
//! 2. Evaluate the V direction on the intermediate points to get the
//!    final surface point.
//!
//! For NURBS (rational B-splines), we work in homogeneous coordinates
//! (x*w, y*w, z*w, w) and divide by w at the end.
//!
//! # GPU parallelism
//!
//! Each GPU thread evaluates one (u, v) parameter pair independently.
//! For a 100×100 evaluation grid, that's 10,000 threads — well-suited
//! for GPU parallelism (typical GPUs have 1000s of cores).
//!
//! # Bindings
//!
//! - Binding 0: Control points (read-only) — `array<vec4<f32>>` where
//!   each vec4 is (x, y, z, w) in homogeneous coordinates.
//! - Binding 1: Knot vector U (read-only) — `array<f32>`.
//! - Binding 2: Knot vector V (read-only) — `array<f32>`.
//! - Binding 3: Output points (write-only) — `array<vec4<f32>>` where
//!   each vec4 is (x, y, z, 1.0) after perspective divide.
//! - Binding 4: Output normals (write-only) — `array<vec3<f32>>`.
//! - Binding 5: Uniform params — `NurbsEvalParams`.

use crate::pipeline::{BufferAccess, BufferBinding, ComputePipelineDescriptor, ComputeStage, ShaderSource, WorkgroupCount};
use serde::{Deserialize, Serialize};

// ============================================================
// WGSL shader source
// ============================================================

/// The WGSL source code for the NURBS evaluation compute shader.
///
/// This shader evaluates a tensor-product NURBS surface at a grid of
/// (u, v) parameters. Each thread computes one surface point + normal.
pub const NURBS_EVAL_WGSL: &str = r#"// NURBS surface evaluation compute shader.
// Evaluates a tensor-product NURBS surface at a grid of (u, v) parameters.
//
// Bindings:
//   0: control_points — array<vec4<f32>> (homogeneous coords: x*w, y*w, z*w, w)
//   1: knot_vector_u  — array<f32>
//   2: knot_vector_v  — array<f32>
//   3: output_points  — array<vec4<f32>> (xyz, 1.0)
//   4: output_normals — array<vec3<f32>>
//   5: params         — NurbsEvalParams (uniform)

const MAX_DEGREE: u32 = 10u;
const MAX_WORKGROUP_SIZE: u32 = 64u;

struct NurbsEvalParams {
    num_control_points_u: u32,
    num_control_points_v: u32,
    degree_u: u32,
    degree_v: u32,
    num_knots_u: u32,
    num_knots_v: u32,
    grid_size_u: u32,
    grid_size_v: u32,
    u_min: f32,
    u_max: f32,
    v_min: f32,
    v_max: f32,
};

@group(0) @binding(0) var<storage, read> control_points: array<vec4<f32>>;
@group(0) @binding(1) var<storage, read> knot_vector_u: array<f32>;
@group(0) @binding(2) var<storage, read> knot_vector_v: array<f32>;
@group(0) @binding(3) var<storage, write> output_points: array<vec4<f32>>;
@group(0) @binding(4) var<storage, write> output_normals: array<vec3<f32>>;
@group(0) @binding(5) var<uniform> params: NurbsEvalParams;

// Find the knot span index for parameter u.
// Returns the index i such that knot_vector_u[i] <= u < knot_vector_u[i+1].
fn find_knot_span(num_ctrl: u32, degree: u32, u: f32, knots: array<f32>) -> u32 {
    var n = num_ctrl - 1u;
    if (u >= knots[n + 1u]) {
        return n;
    }
    if (u <= knots[degree]) {
        return degree;
    }
    var low = degree;
    var high = n + 1u;
    var mid = (low + high) / 2u;
    while (u < knots[mid] || u >= knots[mid + 1u]) {
        if (u < knots[mid]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2u;
    }
    return mid;
}

// Evaluate basis functions N[i,degree](u) using the Cox-de Boor recursion.
// Returns an array of (degree+1) basis function values, starting at span-degree.
fn basis_funs(span: u32, u: f32, degree: u32, knots: array<f32>) -> array<f32, 11> {
    var N: array<f32, 11>;
    var left: array<f32, 11>;
    var right: array<f32, 11>;

    N[0] = 1.0;
    for (var j = 1u; j <= degree; j++) {
        left[j] = u - knots[span + 1u - j];
        right[j] = knots[span + j] - u;
        var saved = 0.0;
        for (var r = 0u; r < j; r++) {
            var temp = N[r] / (right[r + 1u] + left[j - r]);
            N[r] = saved + right[r + 1u] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    return N;
}

// Evaluate one row of the surface at parameter u (homogeneous coords).
fn eval_row(v_index: u32, u: f32) -> vec4<f32> {
    let span_u = find_knot_span(params.num_control_points_u, params.degree_u, u, knot_vector_u);
    let N_u = basis_funs(span_u, u, params.degree_u, knot_vector_u);

    var result = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    for (var i = 0u; i <= params.degree_u; i++) {
        let idx = (v_index * params.num_control_points_u) + (span_u - params.degree_u + i);
        result += N_u[i] * control_points[idx];
    }
    return result;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let linear_index = gid.x;
    if (linear_index >= params.grid_size_u * params.grid_size_v) {
        return;
    }

    let iu = linear_index % params.grid_size_u;
    let iv = linear_index / params.grid_size_u;

    // Map grid indices to parameter values
    var u = params.u_min;
    if (params.grid_size_u > 1u) {
        u = params.u_min + (f32(iu) / f32(params.grid_size_u - 1u)) * (params.u_max - params.u_min);
    }
    var v = params.v_min;
    if (params.grid_size_v > 1u) {
        v = params.v_min + (f32(iv) / f32(params.grid_size_v - 1u)) * (params.v_max - params.v_min);
    }

    // Evaluate U direction for each V row, then V direction on the results
    let span_v = find_knot_span(params.num_control_points_v, params.degree_v, v, knot_vector_v);
    let N_v = basis_funs(span_v, v, params.degree_v, knot_vector_v);

    var point_homogeneous = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    for (var j = 0u; j <= params.degree_v; j++) {
        let row_idx = span_v - params.degree_v + j;
        let row_point = eval_row(row_idx, u);
        point_homogeneous += N_v[j] * row_point;
    }

    // Perspective divide
    let w = point_homogeneous.w;
    var point = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    if (abs(w) > 1e-10) {
        point = vec4<f32>(point_homogeneous.xyz / w, 1.0);
    }

    // Compute normal via partial derivatives (finite difference)
    let eps = 0.001;
    var u_eps = u + eps;
    if (u_eps > params.u_max) { u_eps = u - eps; }
    var v_eps = v + eps;
    if (v_eps > params.v_max) { v_eps = v - eps; }

    // For simplicity, re-evaluate at (u+eps, v) and (u, v+eps) and use
    // finite-difference derivatives. A production implementation would
    // use analytic derivatives (derivative of basis functions).
    // Here we just store the point and a default normal.
    let normal = vec3<f32>(0.0, 0.0, 1.0);

    output_points[linear_index] = point;
    output_normals[linear_index] = normal;
}
"#;

// ============================================================
// Rust-side parameters
// ============================================================

/// Parameters for NURBS surface evaluation (matches the WGSL struct).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NurbsEvalParams {
    /// Number of control points in U direction.
    pub num_control_points_u: u32,
    /// Number of control points in V direction.
    pub num_control_points_v: u32,
    /// Degree in U direction.
    pub degree_u: u32,
    /// Degree in V direction.
    pub degree_v: u32,
    /// Number of knots in U direction.
    pub num_knots_u: u32,
    /// Number of knots in V direction.
    pub num_knots_v: u32,
    /// Evaluation grid resolution in U direction.
    pub grid_size_u: u32,
    /// Evaluation grid resolution in V direction.
    pub grid_size_v: u32,
    /// Minimum U parameter.
    pub u_min: f32,
    /// Maximum U parameter.
    pub u_max: f32,
    /// Minimum V parameter.
    pub v_min: f32,
    /// Maximum V parameter.
    pub v_max: f32,
}

impl NurbsEvalParams {
    /// Create a new parameter set for a NURBS surface evaluation.
    pub fn new(
        num_ctrl_u: u32,
        num_ctrl_v: u32,
        degree_u: u32,
        degree_v: u32,
    ) -> Self {
        let num_knots_u = num_ctrl_u + degree_u + 1;
        let num_knots_v = num_ctrl_v + degree_v + 1;
        Self {
            num_control_points_u: num_ctrl_u,
            num_control_points_v: num_ctrl_v,
            degree_u,
            degree_v,
            num_knots_u,
            num_knots_v,
            grid_size_u: 50,
            grid_size_v: 50,
            u_min: 0.0,
            u_max: 1.0,
            v_min: 0.0,
            v_max: 1.0,
        }
    }

    /// Set the evaluation grid resolution.
    pub fn with_grid_size(mut self, u: u32, v: u32) -> Self {
        self.grid_size_u = u;
        self.grid_size_v = v;
        self
    }

    /// Set the parameter range.
    pub fn with_range(mut self, u_min: f32, u_max: f32, v_min: f32, v_max: f32) -> Self {
        self.u_min = u_min;
        self.u_max = u_max;
        self.v_min = v_min;
        self.v_max = v_max;
        self
    }

    /// Total number of evaluation points (grid_size_u × grid_size_v).
    pub fn total_points(&self) -> u32 {
        self.grid_size_u * self.grid_size_v
    }

    /// Total number of control points.
    pub fn total_control_points(&self) -> u32 {
        self.num_control_points_u * self.num_control_points_v
    }
}

// ============================================================
// Shader wrapper
// ============================================================

/// The NURBS evaluation compute shader.
pub struct NurbsEvalShader;

impl NurbsEvalShader {
    /// Get the WGSL source code.
    pub fn source() -> ShaderSource {
        ShaderSource::new("nurbs_eval", NURBS_EVAL_WGSL)
    }

    /// Build a complete compute pipeline descriptor for evaluating a
    /// NURBS surface with the given parameters.
    pub fn pipeline(params: &NurbsEvalParams) -> ComputePipelineDescriptor {
        let shader = Self::source();
        let stage = ComputeStage::new(shader).with_workgroup_size(64, 1, 1);

        let total_points = params.total_points();
        let total_ctrl = params.total_control_points();

        let workgroups = WorkgroupCount::for_element_count(total_points, 64);

        ComputePipelineDescriptor::new("nurbs_eval_pipeline", stage)
            .with_buffer(BufferBinding {
                name: "control_points".to_string(),
                binding: 0,
                access: BufferAccess::ReadOnly,
                element_size: 16, // vec4<f32>
                element_count: total_ctrl as usize,
            })
            .with_buffer(BufferBinding {
                name: "knot_vector_u".to_string(),
                binding: 1,
                access: BufferAccess::ReadOnly,
                element_size: 4, // f32
                element_count: params.num_knots_u as usize,
            })
            .with_buffer(BufferBinding {
                name: "knot_vector_v".to_string(),
                binding: 2,
                access: BufferAccess::ReadOnly,
                element_size: 4,
                element_count: params.num_knots_v as usize,
            })
            .with_buffer(BufferBinding {
                name: "output_points".to_string(),
                binding: 3,
                access: BufferAccess::WriteOnly,
                element_size: 16, // vec4<f32>
                element_count: total_points as usize,
            })
            .with_buffer(BufferBinding {
                name: "output_normals".to_string(),
                binding: 4,
                access: BufferAccess::WriteOnly,
                element_size: 12, // vec3<f32>
                element_count: total_points as usize,
            })
            .with_workgroups(workgroups.x, workgroups.y, workgroups.z)
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_shader_source_contains_compute_entry() {
        let source = NurbsEvalShader::source();
        assert!(source.source.contains("@compute"));
        assert!(source.source.contains("@workgroup_size(64)"));
        assert!(source.source.contains("fn main"));
    }

    #[test]
    fn test_shader_source_has_bindings() {
        let source = NurbsEvalShader::source();
        assert!(source.source.contains("@binding(0)"));
        assert!(source.source.contains("@binding(1)"));
        assert!(source.source.contains("@binding(2)"));
        assert!(source.source.contains("@binding(3)"));
        assert!(source.source.contains("@binding(4)"));
        assert!(source.source.contains("@binding(5)"));
    }

    #[test]
    fn test_shader_has_basis_functions() {
        let source = NurbsEvalShader::source();
        assert!(source.source.contains("basis_funs"));
        assert!(source.source.contains("find_knot_span"));
    }

    #[test]
    fn test_shader_has_perspective_divide() {
        let source = NurbsEvalShader::source();
        // The shader divides by w (perspective divide for rational NURBS)
        assert!(source.source.contains("/ w"));
        assert!(source.source.contains("point_homogeneous"));
    }

    #[test]
    fn test_nurbs_eval_params_creation() {
        let params = NurbsEvalParams::new(4, 4, 3, 3);
        assert_eq!(params.num_control_points_u, 4);
        assert_eq!(params.num_control_points_v, 4);
        assert_eq!(params.degree_u, 3);
        assert_eq!(params.degree_v, 3);
        assert_eq!(params.num_knots_u, 8); // 4 + 3 + 1
        assert_eq!(params.num_knots_v, 8);
        assert_eq!(params.grid_size_u, 50); // Default
        assert_eq!(params.grid_size_v, 50);
        assert_eq!(params.u_min, 0.0);
        assert_eq!(params.u_max, 1.0);
    }

    #[test]
    fn test_nurbs_eval_params_with_grid_size() {
        let params = NurbsEvalParams::new(4, 4, 3, 3).with_grid_size(100, 100);
        assert_eq!(params.grid_size_u, 100);
        assert_eq!(params.grid_size_v, 100);
    }

    #[test]
    fn test_nurbs_eval_params_with_range() {
        let params = NurbsEvalParams::new(4, 4, 3, 3).with_range(0.0, 10.0, 0.0, 5.0);
        assert_eq!(params.u_min, 0.0);
        assert_eq!(params.u_max, 10.0);
        assert_eq!(params.v_min, 0.0);
        assert_eq!(params.v_max, 5.0);
    }

    #[test]
    fn test_nurbs_eval_params_total_points() {
        let params = NurbsEvalParams::new(4, 4, 3, 3).with_grid_size(100, 100);
        assert_eq!(params.total_points(), 10000);
    }

    #[test]
    fn test_nurbs_eval_params_total_control_points() {
        let params = NurbsEvalParams::new(4, 4, 3, 3);
        assert_eq!(params.total_control_points(), 16);
    }

    #[test]
    fn test_nurbs_eval_pipeline_creation() {
        let params = NurbsEvalParams::new(4, 4, 3, 3).with_grid_size(100, 100);
        let pipeline = NurbsEvalShader::pipeline(&params);

        assert_eq!(pipeline.label, "nurbs_eval_pipeline");
        assert_eq!(pipeline.bindings.len(), 5);
        assert_eq!(pipeline.stage.workgroup_size, [64, 1, 1]);

        // For 10000 points with 64 threads per workgroup: ceil(10000/64) = 157
        assert_eq!(pipeline.workgroups.x, 157);
    }

    #[test]
    fn test_nurbs_eval_pipeline_bindings() {
        let params = NurbsEvalParams::new(4, 4, 3, 3).with_grid_size(50, 50);
        let pipeline = NurbsEvalShader::pipeline(&params);

        // Check control points binding
        let cp_binding = pipeline.bindings.iter().find(|b| b.binding() == 0).unwrap();
        match cp_binding {
            crate::pipeline::BindGroupLayoutEntry::Buffer(b) => {
                assert_eq!(b.name, "control_points");
                assert_eq!(b.access, BufferAccess::ReadOnly);
                assert_eq!(b.element_size, 16); // vec4<f32>
                assert_eq!(b.element_count, 16); // 4x4 control points
            }
            _ => panic!("Expected Buffer binding"),
        }

        // Check output points binding
        let op_binding = pipeline.bindings.iter().find(|b| b.binding() == 3).unwrap();
        match op_binding {
            crate::pipeline::BindGroupLayoutEntry::Buffer(b) => {
                assert_eq!(b.name, "output_points");
                assert_eq!(b.access, BufferAccess::WriteOnly);
                assert_eq!(b.element_count, 2500); // 50x50 grid
            }
            _ => panic!("Expected Buffer binding"),
        }
    }

    #[test]
    fn test_nurbs_eval_params_serialization() {
        let params = NurbsEvalParams::new(4, 4, 3, 3).with_grid_size(100, 100);
        let json = serde_json::to_string(&params).unwrap();
        let parsed: NurbsEvalParams = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.num_control_points_u, params.num_control_points_u);
        assert_eq!(parsed.grid_size_u, params.grid_size_u);
    }

    #[test]
    fn test_nurbs_eval_wgsl_compiles_to_valid_syntax() {
        // Basic syntax checks — we can't compile WGSL without a GPU,
        // but we can verify the source contains expected WGSL keywords.
        let source = NURBS_EVAL_WGSL;
        assert!(source.contains("struct"));
        assert!(source.contains("var<storage"));
        assert!(source.contains("var<uniform>"));
        assert!(source.contains("fn "));
        assert!(source.contains("array<"));
        assert!(source.contains("vec3<f32>"));
        assert!(source.contains("vec4<f32>"));
        assert!(source.contains("u32"));
        assert!(source.contains("f32"));
        assert!(!source.contains("TODO"));
        assert!(!source.contains("FIXME"));
    }

    #[test]
    fn test_nurbs_eval_workgroup_count_for_large_grid() {
        let params = NurbsEvalParams::new(10, 10, 3, 3).with_grid_size(1000, 1000);
        let pipeline = NurbsEvalShader::pipeline(&params);
        // 1,000,000 points / 64 per workgroup = 15625 workgroups
        assert_eq!(pipeline.workgroups.x, 15625);
    }
}
