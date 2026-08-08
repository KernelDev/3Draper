// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Compute pipeline descriptors.
//!
//! These types describe the layout of a WebGPU compute pipeline without
//! depending on `wgpu`. The caller (typically `draper-viewer`) converts
//! these descriptors into actual `wgpu::ComputePipeline` objects.

use serde::{Deserialize, Serialize};

// ============================================================
// Shader source
// ============================================================

/// A reference to a WGSL shader source.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShaderSource {
    /// Human-readable name (e.g., "nurbs_eval", "triangulate_marching_cubes").
    pub name: String,
    /// The WGSL source code.
    pub source: String,
    /// Entry point function name (default: "main").
    pub entry_point: String,
}

impl ShaderSource {
    /// Create a new shader source reference.
    pub fn new(name: &str, source: &str) -> Self {
        Self {
            name: name.to_string(),
            source: source.to_string(),
            entry_point: "main".to_string(),
        }
    }
}

// ============================================================
// Buffer bindings
// ============================================================

/// How a buffer is accessed in a shader.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BufferAccess {
    /// Read-only access (storage buffer with `read`).
    ReadOnly,
    /// Write-only access (storage buffer with `write`).
    WriteOnly,
    /// Read-write access (storage buffer with `read_write`).
    ReadWrite,
    /// Uniform buffer.
    Uniform,
}

/// A buffer binding in a bind group.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BufferBinding {
    /// Human-readable name for debugging.
    pub name: String,
    /// Binding index (0, 1, 2, ...).
    pub binding: u32,
    /// Access mode.
    pub access: BufferAccess,
    /// Size of one element in bytes (e.g., 12 for vec3<f32>).
    pub element_size: usize,
    /// Number of elements in the buffer.
    pub element_count: usize,
}

impl BufferBinding {
    /// Total buffer size in bytes.
    pub fn total_size(&self) -> usize {
        self.element_size * self.element_count
    }
}

/// A bind group layout entry (shader-visible resource).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BindGroupLayoutEntry {
    /// A storage or uniform buffer.
    Buffer(BufferBinding),
    /// A texture (not used in compute shaders, but included for completeness).
    Texture {
        name: String,
        binding: u32,
    },
}

impl BindGroupLayoutEntry {
    /// Get the binding index.
    pub fn binding(&self) -> u32 {
        match self {
            BindGroupLayoutEntry::Buffer(b) => b.binding,
            BindGroupLayoutEntry::Texture { binding, .. } => *binding,
        }
    }
}

// ============================================================
// Compute pipeline
// ============================================================

/// The compute shader stage.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComputeStage {
    /// The shader source.
    pub shader: ShaderSource,
    /// Name of the entry point function.
    pub entry_point: String,
    /// Workgroup size (x, y, z). Default: (64, 1, 1).
    pub workgroup_size: [u32; 3],
}

impl ComputeStage {
    pub fn new(shader: ShaderSource) -> Self {
        Self {
            shader: shader.clone(),
            entry_point: shader.entry_point.clone(),
            workgroup_size: [64, 1, 1],
        }
    }

    /// Set the workgroup size.
    pub fn with_workgroup_size(mut self, x: u32, y: u32, z: u32) -> Self {
        self.workgroup_size = [x, y, z];
        self
    }
}

/// Number of workgroups to dispatch.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct WorkgroupCount {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

impl WorkgroupCount {
    pub fn new(x: u32, y: u32, z: u32) -> Self {
        Self { x, y, z }
    }

    /// Compute the workgroup count needed to cover `total_invocations`
    /// elements, given `workgroup_size` threads per workgroup.
    pub fn for_element_count(total_invocations: u32, workgroup_size: u32) -> Self {
        let groups = total_invocations.div_ceil(workgroup_size);
        Self { x: groups, y: 1, z: 1 }
    }
}

/// A complete compute pipeline descriptor.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComputePipelineDescriptor {
    /// Human-readable label for debugging.
    pub label: String,
    /// The compute stage (shader + entry point + workgroup size).
    pub stage: ComputeStage,
    /// Bind group layout entries (buffers, textures).
    pub bindings: Vec<BindGroupLayoutEntry>,
    /// Number of workgroups to dispatch.
    pub workgroups: WorkgroupCount,
}

impl ComputePipelineDescriptor {
    /// Create a new compute pipeline descriptor.
    pub fn new(label: &str, stage: ComputeStage) -> Self {
        Self {
            label: label.to_string(),
            stage,
            bindings: Vec::new(),
            workgroups: WorkgroupCount::new(1, 1, 1),
        }
    }

    /// Add a buffer binding.
    pub fn with_buffer(mut self, buffer: BufferBinding) -> Self {
        self.bindings.push(BindGroupLayoutEntry::Buffer(buffer));
        self
    }

    /// Set the workgroup dispatch count.
    pub fn with_workgroups(mut self, x: u32, y: u32, z: u32) -> Self {
        self.workgroups = WorkgroupCount::new(x, y, z);
        self
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
    fn test_shader_source_creation() {
        let shader = ShaderSource::new("test_shader", "@compute @workgroup_size(64) fn main() {}");
        assert_eq!(shader.name, "test_shader");
        assert_eq!(shader.entry_point, "main");
        assert!(shader.source.contains("@compute"));
    }

    #[test]
    fn test_buffer_binding_total_size() {
        let binding = BufferBinding {
            name: "positions".to_string(),
            binding: 0,
            access: BufferAccess::ReadOnly,
            element_size: 12, // vec3<f32>
            element_count: 1000,
        };
        assert_eq!(binding.total_size(), 12000);
    }

    #[test]
    fn test_buffer_access_variants() {
        assert_eq!(BufferAccess::ReadOnly, BufferAccess::ReadOnly);
        assert_ne!(BufferAccess::ReadOnly, BufferAccess::WriteOnly);
        assert_ne!(BufferAccess::ReadWrite, BufferAccess::Uniform);
    }

    #[test]
    fn test_bind_group_layout_entry_binding() {
        let buf_entry = BindGroupLayoutEntry::Buffer(BufferBinding {
            name: "test".to_string(),
            binding: 3,
            access: BufferAccess::ReadOnly,
            element_size: 4,
            element_count: 100,
        });
        assert_eq!(buf_entry.binding(), 3);

        let tex_entry = BindGroupLayoutEntry::Texture {
            name: "input_tex".to_string(),
            binding: 5,
        };
        assert_eq!(tex_entry.binding(), 5);
    }

    #[test]
    fn test_compute_stage_default_workgroup_size() {
        let shader = ShaderSource::new("test", "@compute @workgroup_size(64) fn main() {}");
        let stage = ComputeStage::new(shader);
        assert_eq!(stage.workgroup_size, [64, 1, 1]);
    }

    #[test]
    fn test_compute_stage_custom_workgroup_size() {
        let shader = ShaderSource::new("test", "@compute @workgroup_size(8, 8) fn main() {}");
        let stage = ComputeStage::new(shader).with_workgroup_size(8, 8, 1);
        assert_eq!(stage.workgroup_size, [8, 8, 1]);
    }

    #[test]
    fn test_workgroup_count_for_element_count() {
        // 1000 elements, 64 per workgroup → ceil(1000/64) = 16 workgroups
        let count = WorkgroupCount::for_element_count(1000, 64);
        assert_eq!(count.x, 16);
        assert_eq!(count.y, 1);
        assert_eq!(count.z, 1);
    }

    #[test]
    fn test_workgroup_count_exact_multiple() {
        // 1024 elements, 64 per workgroup → 16 workgroups (exact)
        let count = WorkgroupCount::for_element_count(1024, 64);
        assert_eq!(count.x, 16);
    }

    #[test]
    fn test_workgroup_count_one_extra() {
        // 1025 elements, 64 per workgroup → 17 workgroups (one extra)
        let count = WorkgroupCount::for_element_count(1025, 64);
        assert_eq!(count.x, 17);
    }

    #[test]
    fn test_compute_pipeline_descriptor_builder() {
        let shader = ShaderSource::new("test", "@compute @workgroup_size(64) fn main() {}");
        let stage = ComputeStage::new(shader);
        let pipeline = ComputePipelineDescriptor::new("test_pipeline", stage)
            .with_buffer(BufferBinding {
                name: "input".to_string(),
                binding: 0,
                access: BufferAccess::ReadOnly,
                element_size: 12,
                element_count: 100,
            })
            .with_buffer(BufferBinding {
                name: "output".to_string(),
                binding: 1,
                access: BufferAccess::WriteOnly,
                element_size: 12,
                element_count: 100,
            })
            .with_workgroups(10, 1, 1);

        assert_eq!(pipeline.label, "test_pipeline");
        assert_eq!(pipeline.bindings.len(), 2);
        assert_eq!(pipeline.workgroups.x, 10);
    }

    #[test]
    fn test_pipeline_descriptor_serialization() {
        let shader = ShaderSource::new("test", "fn main() {}");
        let stage = ComputeStage::new(shader);
        let pipeline = ComputePipelineDescriptor::new("test_pipeline", stage);

        let json = serde_json::to_string(&pipeline).unwrap();
        assert!(json.contains("test_pipeline"));
        assert!(json.contains("test"));

        let parsed: ComputePipelineDescriptor = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.label, pipeline.label);
    }
}
