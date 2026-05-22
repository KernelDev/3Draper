//! wgpu-based 3D renderer for draper-viewer.
//!
//! Uses a proper GPU pipeline with depth buffer and Phong lighting
//! for high-performance rendering of triangle meshes.

use std::sync::Arc;
use egui_wgpu::{CallbackTrait, CallbackResources, RenderState};
use egui::PaintCallbackInfo;
use wgpu::util::DeviceExt;

/// Vertex format: position + normal.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

impl MeshVertex {
    const DESC: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<MeshVertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![
            0 => Float32x3,  // position
            1 => Float32x3,  // normal
        ],
    };
}

/// Uniform buffer for MVP matrices.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SceneUniforms {
    pub mvp: [[f32; 4]; 4],
    pub model: [[f32; 4]; 4],
    pub light_dir: [f32; 4],  // w = ambient intensity
    pub camera_pos: [f32; 4], // w = unused
}

/// GPU resources for the 3D scene.
pub struct SceneResources {
    pub pipeline: wgpu::RenderPipeline,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
    pub uniform_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
    pub depth_texture: wgpu::Texture,
    pub depth_texture_view: wgpu::TextureView,
    pub wireframe_pipeline: wgpu::RenderPipeline,
}

/// The wgpu callback that renders the 3D scene.
pub struct SceneCallback {
    pub resources: Arc<std::sync::Mutex<Option<SceneResources>>>,
}

impl CallbackTrait for SceneCallback {
    fn paint(
        &self,
        info: PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &CallbackResources,
    ) {
        let resources_guard = self.resources.lock().unwrap();
        let resources = match resources_guard.as_ref() {
            Some(r) => r,
            None => return,
        };

        if resources.index_count == 0 {
            return;
        }

        let viewport = info.viewport_in_pixels();
        let width = viewport.width_px as u32;
        let height = viewport.height_px as u32;

        if width == 0 || height == 0 {
            return;
        }

        render_pass.set_pipeline(&resources.pipeline);
        render_pass.set_bind_group(0, &resources.bind_group, &[]);
        render_pass.set_vertex_buffer(0, resources.vertex_buffer.slice(..));
        render_pass.set_index_buffer(
            resources.index_buffer.slice(..),
            wgpu::IndexFormat::Uint32,
        );
        render_pass.draw_indexed(0..resources.index_count, 0, 0..1);
    }
}

/// Create the WGSL shader module.
fn create_shader_module(device: &wgpu::Device) -> wgpu::ShaderModule {
    device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("3Draper mesh shader"),
        source: wgpu::ShaderSource::Wgsl(SHADER_SRC.into()),
    })
}

const SHADER_SRC: &str = r#"
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
    let light_dir = normalize(uniforms.light_dir.xyz);
    let ambient = uniforms.light_dir.w;

    // Diffuse lighting
    let ndotl = max(dot(normal, light_dir), 0.0);

    // Specular (Blinn-Phong)
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_pos);
    let half_dir = normalize(light_dir + view_dir);
    let ndoth = max(dot(normal, half_dir), 0.0);
    let specular = pow(ndoth, 64.0) * 0.4;

    // Base color (steel blue)
    let base_color = vec3<f32>(0.35, 0.55, 0.78);

    let color = base_color * (ambient + ndotl * 0.7) + vec3<f32>(1.0, 1.0, 1.0) * specular;
    return vec4<f32>(color, 1.0);
}
"#;

/// Create the render pipeline.
fn create_pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    shader: &wgpu::ShaderModule,
    bind_group_layout: &wgpu::BindGroupLayout,
    label: &str,
    topology: wgpu::PrimitiveTopology,
    polygon_mode: wgpu::PolygonMode,
) -> wgpu::RenderPipeline {
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(label),
        bind_group_layouts: &[bind_group_layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(&pipeline_layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_main"),
            buffers: &[MeshVertex::DESC],
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
            topology,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: Some(wgpu::Face::Back),
            polygon_mode,
            unclipped_depth: false,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
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

/// Create or recreate depth texture for the given dimensions.
pub fn create_depth_texture(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("3Draper depth texture"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    (texture, view)
}

/// Initialize all GPU resources for the scene.
pub fn create_scene_resources(
    render_state: &RenderState,
    vertices: &[MeshVertex],
    indices: &[u32],
) -> SceneResources {
    let device = &render_state.device;
    let format = render_state.target_format;

    // Shader
    let shader = create_shader_module(device);

    // Bind group layout
    let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("3Draper uniform bind group layout"),
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

    // Vertex buffer
    let vertex_buffer = if vertices.is_empty() {
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
    };

    // Index buffer
    let index_buffer = if indices.is_empty() {
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
    };

    let index_count = indices.len() as u32;

    // Uniform buffer (will be updated every frame)
    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("3Draper uniform buffer"),
        contents: bytemuck::cast_slice(&[SceneUniforms {
            mvp: [[0.0; 4]; 4],
            model: [[0.0; 4]; 4],
            light_dir: [0.3, 0.5, 0.8, 0.2],
            camera_pos: [0.0, 0.0, 0.0, 0.0],
        }]),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });

    // Bind group
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("3Draper uniform bind group"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            },
        ],
    });

    // Depth texture (initial size, will be resized)
    let (depth_texture, depth_texture_view) = create_depth_texture(device, 1200, 800);

    // Pipelines
    let pipeline = create_pipeline(
        device, format, &shader, &bind_group_layout,
        "3Draper solid pipeline",
        wgpu::PrimitiveTopology::TriangleList,
        wgpu::PolygonMode::Fill,
    );

    let wireframe_pipeline = create_pipeline(
        device, format, &shader, &bind_group_layout,
        "3Draper wireframe pipeline",
        wgpu::PrimitiveTopology::TriangleList,
        wgpu::PolygonMode::Line,
    );

    SceneResources {
        pipeline,
        vertex_buffer,
        index_buffer,
        index_count,
        uniform_buffer,
        bind_group,
        depth_texture,
        depth_texture_view,
        wireframe_pipeline,
    }
}

/// Update the mesh data in GPU buffers.
pub fn update_mesh_buffers(
    resources: &mut SceneResources,
    device: &wgpu::Device,
    vertices: &[MeshVertex],
    indices: &[u32],
) {
    resources.vertex_buffer = if vertices.is_empty() {
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
    };

    resources.index_buffer = if indices.is_empty() {
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
    };

    resources.index_count = indices.len() as u32;
}

/// Update the uniform buffer with new MVP matrices.
pub fn update_uniforms(
    resources: &SceneResources,
    queue: &wgpu::Queue,
    uniforms: &SceneUniforms,
) {
    queue.write_buffer(&resources.uniform_buffer, 0, bytemuck::cast_slice(&[*uniforms]));
}

/// Resize depth texture if needed.
pub fn resize_depth_texture(
    resources: &mut SceneResources,
    device: &wgpu::Device,
    width: u32,
    height: u32,
) {
    if width == 0 || height == 0 {
        return;
    }
    let (dt, dtv) = create_depth_texture(device, width, height);
    resources.depth_texture = dt;
    resources.depth_texture_view = dtv;
}
