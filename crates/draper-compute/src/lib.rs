// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-compute
//!
//! WebGPU compute shaders for accelerating heavy geometric computations.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 5.3: provides WGSL shader
//! source code and Rust bindings for GPU-accelerated:
//!
//! - **NURBS surface evaluation** — mass-evaluate NURBS surface points
//!   and normals at a grid of (u, v) parameters.
//! - **Mesh triangulation** — parallel ear-clipping or Marching Cubes
//!   for SDF-to-mesh conversion.
//!
//! # Design
//!
//! The crate is **shader-source-only**: it provides the WGSL source code
//! as embedded strings and a Rust API for describing the compute pipeline
//! (bind group layout, push constants, dispatch sizes). The actual GPU
//! device interaction is left to the caller (typically `draper-viewer`
//! via `wgpu`), so this crate has no hard dependency on `wgpu` and
//! compiles on platforms without GPU support (e.g., headless servers,
//! WASM without WebGPU).
//!
//! # Modules
//!
//! - `nurbs_eval` — WGSL shader for NURBS surface evaluation.
//! - `triangulate` — WGSL shader for parallel triangulation.
//! - `pipeline` — Rust types describing compute pipeline descriptors.

pub mod nurbs_eval;
pub mod triangulate;
pub mod pipeline;

pub use nurbs_eval::{NurbsEvalShader, NurbsEvalParams};
pub use triangulate::{TriangulateShader, TriangulateParams, TriangulateMethod};
pub use pipeline::{
    ComputePipelineDescriptor, BindGroupLayoutEntry, BufferBinding,
    ComputeStage, ShaderSource, WorkgroupCount,
};
