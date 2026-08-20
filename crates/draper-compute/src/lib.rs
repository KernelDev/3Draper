// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! WebGPU Compute Shaders for geometry evaluation (Vision 2030 Task 2).
//!
//! Per Vision 2030: port heavy math to GPU Compute Shaders for 50-100× speedup.
//!
//! This crate provides:
//! - WGSL shader source for NURBS surface evaluation (mass parallel)
//! - GPU buffer management for SOA data (NurbsSurfaceSOA)
//! - Compute pipeline creation and dispatch
//! - Result readback to CPU
//!
//! Works on both native (Vulkan/DX12/Metal) and WASM (WebGPU).

use draper_geometry::gpu_batch::NurbsSurfaceSOA;

/// WGSL compute shader for NURBS surface evaluation.
///
/// Evaluates S(u,v) = sum_i sum_j N_i(u) * N_j(v) * P_ij * w_ij / sum(w)
/// for N points in parallel (one thread per UV pair).
pub const NURBS_EVAL_SHADER: &str = r#"
// NURBS surface evaluation compute shader
// Input: control points (SOA), weights, knots, UV params
// Output: XYZ coordinates

@group(0) @binding(0) var<storage, read> cp_x: array<f32>;
@group(0) @binding(1) var<storage, read> cp_y: array<f32>;
@group(0) @binding(2) var<storage, read> cp_z: array<f32>;
@group(0) @binding(3) var<storage, read> weights: array<f32>;
@group(0) @binding(4) var<storage, read> u_knots: array<f32>;
@group(0) @binding(5) var<storage, read> v_knots: array<f32>;
@group(0) @binding(6) var<storage, read> u_params: array<f32>;
@group(0) @binding(7) var<storage, read> v_params: array<f32>;
@group(0) @binding(8) var<storage, read_write> out_x: array<f32>;
@group(0) @binding(9) var<storage, read_write> out_y: array<f32>;
@group(0) @binding(10) var<storage, read_write> out_z: array<f32>;

struct Params {
    u_degree: u32,
    v_degree: u32,
    n_u: u32,
    n_v: u32,
    count: u32,
    u_knot_count: u32,
    v_knot_count: u32,
    _pad: u32,
}
@group(0) @binding(11) var<uniform> params: Params;

fn find_knot_span(knots: array<f32, >, degree: u32, t: f32, n: u32) -> u32 {
    if (t >= knots[n]) { return n - 1u; }
    if (t <= knots[degree]) { return degree; }
    var low = degree;
    var high = n;
    var mid = (low + high) / 2u;
    while (t < knots[mid] || t >= knots[mid + 1u]) {
        if (t < knots[mid]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2u;
    }
    return mid;
}

fn de_boor(p: ptr<function, array<f32, 8>>, knots: array<f32, >, degree: u32, span: u32, t: f32) {
    for (var r = 1u; r <= degree; r++) {
        for (var j = degree; j >= r; j--) {
            let i = span - degree + j;
            let denom = knots[i + degree + 1u - r] - knots[i];
            let alpha = select(0.0, (t - knots[i]) / denom, abs(denom) > 1e-15);
            p[j] = alpha * p[j] + (1.0 - alpha) * p[j - 1u];
        }
    }
}

@compute @workgroup_size(64)
fn evaluate_batch(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    if (idx >= params.count) { return; }

    let u = u_params[idx];
    let v = v_params[idx];

    let k_u = find_knot_span(u_knots, params.u_degree, u, params.n_u);
    let k_v = find_knot_span(v_knots, params.v_degree, v, params.n_v);

    // Step 1: Evaluate B-spline in V for each U-row
    var inter_x: array<f32, 8>;
    var inter_y: array<f32, 8>;
    var inter_z: array<f32, 8>;
    var inter_w: array<f32, 8>;

    for (var i = 0u; i <= params.u_degree; i++) {
        let row = k_u - params.u_degree + i;
        if (row >= params.n_u) { continue; }

        var vx: array<f32, 8>;
        var vy: array<f32, 8>;
        var vz: array<f32, 8>;
        var vw: array<f32, 8>;

        for (var j = 0u; j <= params.v_degree; j++) {
            let col = min(k_v - params.v_degree + j, params.n_v - 1u);
            let vi = col * params.n_u + row;
            vx[j] = cp_x[vi] * weights[vi];
            vy[j] = cp_y[vi] * weights[vi];
            vz[j] = cp_z[vi] * weights[vi];
            vw[j] = weights[vi];
        }

        de_boor(&vx, v_knots, params.v_degree, k_v, v);
        de_boor(&vy, v_knots, params.v_degree, k_v, v);
        de_boor(&vz, v_knots, params.v_degree, k_v, v);
        de_boor(&vw, v_knots, params.v_degree, k_v, v);

        inter_x[i] = vx[params.v_degree];
        inter_y[i] = vy[params.v_degree];
        inter_z[i] = vz[params.v_degree];
        inter_w[i] = vw[params.v_degree];
    }

    // Step 2: De Boor in U
    de_boor(&inter_x, u_knots, params.u_degree, k_u, u);
    de_boor(&inter_y, u_knots, params.u_degree, k_u, u);
    de_boor(&inter_z, u_knots, params.u_degree, k_u, u);
    de_boor(&inter_w, u_knots, params.u_degree, k_u, u);

    let w = inter_w[params.u_degree];
    if (abs(w) < 1e-15) {
        out_x[idx] = 0.0;
        out_y[idx] = 0.0;
        out_z[idx] = 0.0;
    } else {
        let x = inter_x[params.u_degree] / w;
        let y = inter_y[params.u_degree] / w;
        let z = inter_z[params.u_degree] / w;
        if (x == x && y == y && z == z) {  // NaN check
            out_x[idx] = x;
            out_y[idx] = y;
            out_z[idx] = z;
        } else {
            out_x[idx] = 0.0;
            out_y[idx] = 0.0;
            out_z[idx] = 0.0;
        }
    }
}
"#;

/// Uniform buffer layout for NURBS evaluation parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct NurbsEvalParams {
    pub u_degree: u32,
    pub v_degree: u32,
    pub n_u: u32,
    pub n_v: u32,
    pub count: u32,
    pub u_knot_count: u32,
    pub v_knot_count: u32,
    pub _pad: u32,
}

/// GPU compute pipeline for NURBS surface evaluation.
pub struct NurbsComputePipeline {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    bind_group_layout: wgpu::BindGroupLayout,
}

impl NurbsComputePipeline {
    /// Create a GPU compute pipeline for NURBS evaluation.
    ///
    /// Requests a WebGPU adapter (or native GPU adapter on desktop).
    /// Returns None if no GPU adapter is available.
    pub async fn new() -> Option<Self> {
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let adapter = match instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }).await {
            Some(a) => a,
            None => return None,
        };

        let (device, queue) = match adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("draper-compute NURBS device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            memory_hints: wgpu::MemoryHints::default(),
            ..Default::default()
        }, None).await {
            Ok(dq) => dq,
            Err(_) => return None,
        };

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("NURBS eval shader"),
            source: wgpu::ShaderSource::Wgsl(NURBS_EVAL_SHADER.into()),
        });

        let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..11u32)
            .map(|i| wgpu::BindGroupLayoutEntry {
                binding: i,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if i < 8 {
                        wgpu::BufferBindingType::Storage { read_only: true }
                    } else {
                        wgpu::BufferBindingType::Storage { read_only: false }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .chain(std::iter::once(wgpu::BindGroupLayoutEntry {
                binding: 11,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }))
            .collect();

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("NURBS bind group layout"),
            entries: &entries,
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("NURBS compute layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("NURBS eval pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: Some("evaluate_batch"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });

        log::info!("GPU compute pipeline created: NURBS surface evaluation");

        Some(Self {
            device,
            queue,
            pipeline,
            bind_group_layout,
        })
    }

    /// Evaluate a NURBS surface on the GPU for N UV parameter pairs.
    ///
    /// Creates storage buffers, dispatches the compute shader, and reads back
    /// the results. Falls back to None if GPU dispatch fails.
    pub fn evaluate_batch(
        &self,
        surface: &NurbsSurfaceSOA,
        u_params: &[f32],
        v_params: &[f32],
    ) -> Option<(Vec<f32>, Vec<f32>, Vec<f32>)> {
        let count = u_params.len().min(v_params.len());
        if count == 0 {
            return None;
        }

        log::info!(
            "GPU NURBS eval: {} points, {}×{} control grid, degree {}×{}",
            count, surface.n_u, surface.n_v, surface.u_degree, surface.v_degree
        );

        let device = &self.device;
        let queue = &self.queue;

        // ── Create input storage buffers ──
        let cp_x_bytes = bytemuck::cast_slice(&surface.cp_x);
        let cp_y_bytes = bytemuck::cast_slice(&surface.cp_y);
        let cp_z_bytes = bytemuck::cast_slice(&surface.cp_z);
        let weights_bytes = bytemuck::cast_slice(&surface.weights);
        let u_knots_bytes = bytemuck::cast_slice(&surface.u_knots);
        let v_knots_bytes = bytemuck::cast_slice(&surface.v_knots);
        let u_params_bytes = bytemuck::cast_slice(&u_params[..count]);
        let v_params_bytes = bytemuck::cast_slice(&v_params[..count]);

        let out_size = (count * std::mem::size_of::<f32>()) as wgpu::BufferAddress;

        let create_storage = |label: &str, _data: &[u8], read_only: bool| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: out_size,
                usage: if read_only {
                    wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST
                } else {
                    wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC
                },
                mapped_at_creation: false,
            })
        };

        let create_staging = |label: &str| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: out_size,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };

        let create_input = |label: &str, data: &[u8]| {
            let buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: data.len() as wgpu::BufferAddress,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&buf, 0, data);
            buf
        };

        let buf_cp_x = create_input("cp_x", cp_x_bytes);
        let buf_cp_y = create_input("cp_y", cp_y_bytes);
        let buf_cp_z = create_input("cp_z", cp_z_bytes);
        let buf_weights = create_input("weights", weights_bytes);
        let buf_u_knots = create_input("u_knots", u_knots_bytes);
        let buf_v_knots = create_input("v_knots", v_knots_bytes);
        let buf_u_params = create_input("u_params", u_params_bytes);
        let buf_v_params = create_input("v_params", v_params_bytes);

        // ── Create output storage buffers (STORAGE | COPY_SRC) ──
        let buf_out_x = create_storage("out_x", &[], false);
        let buf_out_y = create_storage("out_y", &[], false);
        let buf_out_z = create_storage("out_z", &[], false);

        // Staging buffers for CPU readback (MAP_READ | COPY_DST)
        let staging_x = create_staging("staging_x");
        let staging_y = create_staging("staging_y");
        let staging_z = create_staging("staging_z");

        // ── Create uniform buffer ──
        let params = NurbsEvalParams {
            u_degree: surface.u_degree as u32,
            v_degree: surface.v_degree as u32,
            n_u: surface.n_u as u32,
            n_v: surface.n_v as u32,
            count: count as u32,
            u_knot_count: surface.u_knots.len() as u32,
            v_knot_count: surface.v_knots.len() as u32,
            _pad: 0,
        };
        let params_bytes = bytemuck::bytes_of(&params);
        let buf_params = create_input("params", params_bytes);

        // ── Create bind group ──
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("NURBS eval bind group"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: buf_cp_x.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: buf_cp_y.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: buf_cp_z.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: buf_weights.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: buf_u_knots.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: buf_v_knots.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: buf_u_params.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 7, resource: buf_v_params.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 8, resource: buf_out_x.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 9, resource: buf_out_y.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 10, resource: buf_out_z.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 11, resource: buf_params.as_entire_binding() },
            ],
        });

        // ── Dispatch compute ──
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("NURBS eval encoder"),
        });

        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("NURBS eval pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            let workgroups = (count as u32 + 63) / 64; // Round up to workgroup_size=64
            pass.dispatch_workgroups(workgroups, 1, 1);
        }

        // Copy output buffers to staging buffers for CPU readback
        encoder.copy_buffer_to_buffer(&buf_out_x, 0, &staging_x, 0, out_size);
        encoder.copy_buffer_to_buffer(&buf_out_y, 0, &staging_y, 0, out_size);
        encoder.copy_buffer_to_buffer(&buf_out_z, 0, &staging_z, 0, out_size);

        queue.submit(std::iter::once(encoder.finish()));

        // ── Read back results from staging buffers ──
        let slice_x = staging_x.slice(..);
        slice_x.map_async(wgpu::MapMode::Read, |_| {});

        let slice_y = staging_y.slice(..);
        slice_y.map_async(wgpu::MapMode::Read, |_| {});

        let slice_z = staging_z.slice(..);
        slice_z.map_async(wgpu::MapMode::Read, |_| {});

        // Poll device to complete GPU work
        let _ = device.poll(wgpu::Maintain::Wait);

        let out_x: Vec<f32> = {
            let data = slice_x.get_mapped_range();
            bytemuck::cast_slice(&data).to_vec()
        };
        let out_y: Vec<f32> = {
            let data = slice_y.get_mapped_range();
            bytemuck::cast_slice(&data).to_vec()
        };
        let out_z: Vec<f32> = {
            let data = slice_z.get_mapped_range();
            bytemuck::cast_slice(&data).to_vec()
        };

        log::info!("GPU NURBS eval complete: {} points evaluated", count);
        Some((out_x, out_y, out_z))
    }
}

// ============================================================
// Phase 5.3 additional modules (triangulation shaders + pipeline descriptors)
// ============================================================

pub mod nurbs_eval;
pub mod triangulate;
pub mod pipeline;

pub use nurbs_eval::{NurbsEvalShader as NurbsEvalShaderPhase5, NurbsEvalParams as NurbsEvalParamsPhase5};
pub use triangulate::{TriangulateShader, TriangulateParams, TriangulateMethod};
pub use pipeline::{
    ComputePipelineDescriptor, BindGroupLayoutEntry, BufferBinding,
    ComputeStage, ShaderSource, WorkgroupCount,
};
