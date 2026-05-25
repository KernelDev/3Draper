//! wgpu-based 3D renderer for draper-viewer.
//!
//! Renders the 3D scene to an offscreen texture (with depth buffer) in the
//! `prepare` callback, then blits the result to the egui render pass in the
//! `paint` callback. This is the standard approach for custom wgpu rendering
//! within egui, since egui's render pass does not include a depth attachment.

use std::sync::Arc;
use egui_wgpu::{CallbackTrait, CallbackResources, RenderState, ScreenDescriptor};
use egui::PaintCallbackInfo;
use wgpu::util::DeviceExt;

// ─── Vertex / Uniform types ──────────────────────────────────────────────

/// Vertex format for the 3D mesh: position + normal.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

impl MeshVertex {
    const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<MeshVertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![
            0 => Float32x3,
            1 => Float32x3,
        ],
    };
}

/// Uniform buffer for MVP matrices + lighting.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SceneUniforms {
    pub mvp: [[f32; 4]; 4],
    pub model: [[f32; 4]; 4],
    pub light_dir: [f32; 4],  // xyz = direction (normalized, FROM camera), w = ambient
    pub camera_pos: [f32; 4], // xyz = position, w = unused
}

// ─── Offscreen resources ─────────────────────────────────────────────────

/// All GPU resources needed for offscreen 3D rendering + blit.
pub struct SceneResources {
    // Mesh rendering
    pub mesh_pipeline: wgpu::RenderPipeline,
    pub wireframe_pipeline: Option<wgpu::RenderPipeline>,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
    pub uniform_buffer: wgpu::Buffer,
    pub mesh_bind_group: wgpu::BindGroup,

    // Bind group layouts (needed for resize recreation)
    pub mesh_bind_group_layout: wgpu::BindGroupLayout,
    pub blit_bind_group_layout: wgpu::BindGroupLayout,

    // Offscreen render target
    pub offscreen_color: wgpu::TextureView,
    pub offscreen_depth: wgpu::TextureView,
    pub offscreen_width: u32,
    pub offscreen_height: u32,

    // Blit (fullscreen quad to display offscreen texture in egui pass)
    pub blit_pipeline: wgpu::RenderPipeline,
    pub blit_bind_group: wgpu::BindGroup,
    pub blit_sampler: wgpu::Sampler,
}

/// Stored in CallbackResources so paint() can access the offscreen texture.
pub struct OffscreenResult {
    pub color_view: wgpu::TextureView,
}

// ─── SceneCallback ───────────────────────────────────────────────────────

/// The wgpu callback that renders the 3D scene.
pub struct SceneCallback {
    pub resources: Arc<std::sync::Mutex<Option<SceneResources>>>,
    pub wireframe: bool,
    pub viewport_width: u32,
    pub viewport_height: u32,
}

impl CallbackTrait for SceneCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        _screen_descriptor: &ScreenDescriptor,
        egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let mut guard = self.resources.lock().unwrap();
        let resources = match guard.as_mut() {
            Some(r) => r,
            None => return Vec::new(),
        };

        if resources.index_count == 0 {
            return Vec::new();
        }

        let w = self.viewport_width;
        let h = self.viewport_height;
        if w == 0 || h == 0 {
            return Vec::new();
        }

        // Resize offscreen textures if viewport size changed
        if w != resources.offscreen_width || h != resources.offscreen_height {
            resize_offscreen(resources, device, w, h);
        }

        // Render mesh to offscreen texture (with depth attachment)
        let mut mesh_pass = egui_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("3Draper offscreen mesh render"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &resources.offscreen_color,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.93, g: 0.95, b: 0.97, a: 1.0 }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &resources.offscreen_depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        let pipeline = if self.wireframe {
            resources.wireframe_pipeline.as_ref().unwrap_or(&resources.mesh_pipeline)
        } else {
            &resources.mesh_pipeline
        };

        mesh_pass.set_pipeline(pipeline);
        mesh_pass.set_bind_group(0, &resources.mesh_bind_group, &[]);
        mesh_pass.set_vertex_buffer(0, resources.vertex_buffer.slice(..));
        mesh_pass.set_index_buffer(resources.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        mesh_pass.draw_indexed(0..resources.index_count, 0, 0..1);

        drop(mesh_pass);

        // Store the offscreen result for paint()
        callback_resources.insert(OffscreenResult {
            color_view: resources.offscreen_color.clone(),
        });

        Vec::new()
    }

    fn paint(
        &self,
        _info: PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &CallbackResources,
    ) {
        let Some(_result) = callback_resources.get::<OffscreenResult>() else {
            return;
        };

        // Get the blit pipeline from resources
        let guard = self.resources.lock().unwrap();
        let Some(resources) = guard.as_ref() else {
            return;
        };

        // The blit pipeline has depth_stencil: None, so it's compatible
        // with egui's render pass (which has no depth attachment).
        render_pass.set_pipeline(&resources.blit_pipeline);
        render_pass.set_bind_group(0, &resources.blit_bind_group, &[]);
        render_pass.draw(0..3, 0..1); // Fullscreen triangle
    }
}

// ─── Shaders ─────────────────────────────────────────────────────────────

const MESH_SHADER_SRC: &str = r#"
struct Uniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    light_dir: vec4<f32>,
    camera_pos: vec4<f32>,
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) world_pos: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let world_pos = uniforms.model * vec4<f32>(in.position, 1.0);
    out.clip_position = uniforms.mvp * vec4<f32>(in.position, 1.0);
    out.world_normal = (uniforms.model * vec4<f32>(in.normal, 0.0)).xyz;
    out.world_pos = world_pos.xyz;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let normal = normalize(in.world_normal);
    // Primary light: headlight from camera direction
    let light_dir = normalize(uniforms.light_dir.xyz);
    let ambient = uniforms.light_dir.w;

    // Two-sided lighting: flip normal if it faces away from camera
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_pos);
    let effective_normal = select(-normal, normal, dot(normal, view_dir) >= 0.0);

    // Primary headlight (strong)
    let ndotl_primary = max(dot(effective_normal, light_dir), 0.0);
    let half_dir_primary = normalize(light_dir + view_dir);
    let ndoth_primary = max(dot(effective_normal, half_dir_primary), 0.0);
    let specular_primary = pow(ndoth_primary, 64.0) * 0.4;

    // Secondary fill light from below-left (softer)
    let fill_dir = normalize(vec3<f32>(-0.3, -0.5, 0.4));
    let ndotl_fill = max(dot(effective_normal, fill_dir), 0.0);

    // Tertiary rim/back light for edge definition
    let rim_dir = normalize(vec3<f32>(0.2, 0.4, -0.8));
    let ndotl_rim = max(dot(effective_normal, rim_dir), 0.0);
    let rim_factor = pow(1.0 - max(dot(effective_normal, view_dir), 0.0), 2.5) * 0.12;

    // Base color — blue-grey for CAD models (visible on light background)
    let base_color = vec3<f32>(0.48, 0.52, 0.58);

    // Combine lighting
    let diffuse = ndotl_primary * 0.50 + ndotl_fill * 0.25 + ndotl_rim * 0.12;
    let color = base_color * (ambient + diffuse) + vec3<f32>(1.0) * specular_primary + base_color * rim_factor;

    return vec4<f32>(color, 1.0);
}
"#;

const BLIT_SHADER_SRC: &str = r#"
@group(0) @binding(0) var offscreen_tex: texture_2d<f32>;
@group(0) @binding(1) var offscreen_sampler: sampler;

struct VertexOutput {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

// Fullscreen triangle: 3 vertices cover the entire screen
@vertex
fn vs_main(@builtin(vertex_index) vi: u32) -> VertexOutput {
    var out: VertexOutput;
    let positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
        vec2<f32>(-1.0,  3.0),
    );
    let uvs = array<vec2<f32>, 3>(
        vec2<f32>(0.0, 1.0),
        vec2<f32>(2.0, 1.0),
        vec2<f32>(0.0, -1.0),
    );
    out.clip_pos = vec4<f32>(positions[vi], 0.0, 1.0);
    out.uv = uvs[vi];
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return textureSample(offscreen_tex, offscreen_sampler, in.uv);
}
"#;

// ─── Pipeline creation helpers ────────────────────────────────────────────

fn create_mesh_pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::BindGroupLayout,
    label: &str,
    polygon_mode: wgpu::PolygonMode,
) -> wgpu::RenderPipeline {
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(label),
        bind_group_layouts: &[layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(&pipeline_layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_main"),
            buffers: &[MeshVertex::LAYOUT],
            compilation_options: Default::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_main"),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: Some(wgpu::BlendState {
                    color: wgpu::BlendComponent::REPLACE,
                    alpha: wgpu::BlendComponent::REPLACE,
                }),
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            // IMPORTANT: Disable backface culling. Our triangulation may produce
            // CW or CCW triangles depending on face.forward and surface type.
            // With culling enabled, half the model can be invisible.
            cull_mode: None,
            polygon_mode,
            unclipped_depth: false,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: true,
            // Use LessEqual instead of Less so that equal-depth faces are also drawn.
            // This helps with wireframe overlays and two-sided rendering.
            depth_compare: wgpu::CompareFunction::LessEqual,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: 1,
            mask: !0,
            alpha_to_coverage_enabled: false,
        },
        multiview: None,
        cache: None,
    })
}

fn create_blit_pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("3Draper blit pipeline layout"),
        bind_group_layouts: &[layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("3Draper blit pipeline"),
        layout: Some(&pipeline_layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_main"),
            buffers: &[],
            compilation_options: Default::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_main"),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: Some(wgpu::BlendState {
                    color: wgpu::BlendComponent::REPLACE,
                    alpha: wgpu::BlendComponent::REPLACE,
                }),
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: None,
            polygon_mode: wgpu::PolygonMode::Fill,
            unclipped_depth: false,
            conservative: false,
        },
        depth_stencil: None, // No depth for the blit — egui pass has no depth attachment
        multisample: wgpu::MultisampleState {
            count: 1,
            mask: !0,
            alpha_to_coverage_enabled: false,
        },
        multiview: None,
        cache: None,
    })
}

// ─── Offscreen texture management ────────────────────────────────────────

fn create_offscreen_textures(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::TextureView, wgpu::TextureView) {
    let color = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("3Draper offscreen color"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });

    let depth = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("3Draper offscreen depth"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });

    (
        color.create_view(&wgpu::TextureViewDescriptor::default()),
        depth.create_view(&wgpu::TextureViewDescriptor::default()),
    )
}

fn resize_offscreen(resources: &mut SceneResources, device: &wgpu::Device, width: u32, height: u32) {
    let (color_view, depth_view) = create_offscreen_textures(device, width, height);
    resources.offscreen_color = color_view;
    resources.offscreen_depth = depth_view;
    resources.offscreen_width = width;
    resources.offscreen_height = height;

    // Recreate blit bind group with new texture view
    resources.blit_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("3Draper blit bind group (resized)"),
        layout: &resources.blit_bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&resources.offscreen_color),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&resources.blit_sampler),
            },
        ],
    });
}

// ─── Public API ──────────────────────────────────────────────────────────

/// Initialize all GPU resources for the scene.
pub fn create_scene_resources(
    render_state: &RenderState,
    vertices: &[MeshVertex],
    indices: &[u32],
) -> SceneResources {
    let device = &render_state.device;
    let surface_format = render_state.target_format;

    // The offscreen color texture uses a fixed format independent of the surface.
    // The mesh pipeline must match the offscreen format, NOT the surface format,
    // because it renders into the offscreen texture.
    let offscreen_color_format = wgpu::TextureFormat::Rgba8UnormSrgb;

    // ── Mesh shader ──
    let mesh_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("3Draper mesh shader"),
        source: wgpu::ShaderSource::Wgsl(MESH_SHADER_SRC.into()),
    });

    // ── Mesh bind group layout (uniform buffer only) ──
    let mesh_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("3Draper mesh bind group layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: Some(std::num::NonZeroU64::new(std::mem::size_of::<SceneUniforms>() as u64).unwrap()),
                },
                count: None,
            },
        ],
    });

    // ── Mesh pipeline (renders into offscreen texture → use offscreen format) ──
    let mesh_pipeline = create_mesh_pipeline(
        device, offscreen_color_format, &mesh_shader, &mesh_bind_group_layout,
        "3Draper mesh pipeline (fill)",
        wgpu::PolygonMode::Fill,
    );

    let wireframe_pipeline = if device.features().contains(wgpu::Features::POLYGON_MODE_LINE) {
        Some(create_mesh_pipeline(
            device, offscreen_color_format, &mesh_shader, &mesh_bind_group_layout,
            "3Draper mesh pipeline (wireframe)",
            wgpu::PolygonMode::Line,
        ))
    } else {
        None
    };

    // ── Vertex / index buffers ──
    let vertex_buffer = create_vertex_buffer(device, vertices);
    let index_buffer = create_index_buffer(device, indices);
    let index_count = indices.len() as u32;

    // ── Uniform buffer ──
    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("3Draper uniform buffer"),
        contents: bytemuck::cast_slice(&[SceneUniforms {
            mvp: [[0.0; 4]; 4],
            model: [[0.0; 4]; 4],
            light_dir: [0.0, 0.0, 1.0, 0.25],
            camera_pos: [0.0, 0.0, 0.0, 0.0],
        }]),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });

    let mesh_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("3Draper mesh bind group"),
        layout: &mesh_bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            },
        ],
    });

    // ── Offscreen textures (initial size) ──
    let (offscreen_color, offscreen_depth) = create_offscreen_textures(device, 1280, 800);

    // ── Blit pipeline ──
    let blit_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("3Draper blit shader"),
        source: wgpu::ShaderSource::Wgsl(BLIT_SHADER_SRC.into()),
    });

    let blit_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("3Draper blit bind group layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    multisampled: false,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
    });

    // Blit pipeline renders into egui's render pass → use surface format
    let blit_pipeline = create_blit_pipeline(device, surface_format, &blit_shader, &blit_bind_group_layout);

    let blit_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("3Draper blit sampler"),
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });

    let blit_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("3Draper blit bind group"),
        layout: &blit_bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&offscreen_color),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&blit_sampler),
            },
        ],
    });

    SceneResources {
        mesh_pipeline,
        wireframe_pipeline,
        vertex_buffer,
        index_buffer,
        index_count,
        uniform_buffer,
        mesh_bind_group,
        mesh_bind_group_layout,
        blit_bind_group_layout,
        offscreen_color,
        offscreen_depth,
        offscreen_width: 1280,
        offscreen_height: 800,
        blit_pipeline,
        blit_bind_group,
        blit_sampler,
    }
}

fn create_vertex_buffer(device: &wgpu::Device, vertices: &[MeshVertex]) -> wgpu::Buffer {
    if vertices.is_empty() {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("3Draper vertex buffer (empty)"),
            contents: bytemuck::cast_slice(&[MeshVertex { position: [0.0; 3], normal: [0.0; 3] }]),
            usage: wgpu::BufferUsages::VERTEX,
        })
    } else {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("3Draper vertex buffer"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX,
        })
    }
}

fn create_index_buffer(device: &wgpu::Device, indices: &[u32]) -> wgpu::Buffer {
    if indices.is_empty() {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("3Draper index buffer (empty)"),
            contents: bytemuck::cast_slice(&[0u32]),
            usage: wgpu::BufferUsages::INDEX,
        })
    } else {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("3Draper index buffer"),
            contents: bytemuck::cast_slice(indices),
            usage: wgpu::BufferUsages::INDEX,
        })
    }
}

/// Update mesh data in GPU buffers.
pub fn update_mesh_buffers(
    resources: &mut SceneResources,
    device: &wgpu::Device,
    vertices: &[MeshVertex],
    indices: &[u32],
) {
    resources.vertex_buffer = create_vertex_buffer(device, vertices);
    resources.index_buffer = create_index_buffer(device, indices);
    resources.index_count = indices.len() as u32;
}

/// Update the uniform buffer.
pub fn update_uniforms(
    resources: &SceneResources,
    queue: &wgpu::Queue,
    uniforms: &SceneUniforms,
) {
    queue.write_buffer(&resources.uniform_buffer, 0, bytemuck::cast_slice(&[*uniforms]));
}
