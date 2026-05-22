//! 3Draper Viewer — high-performance 3D model viewer using egui/wgpu.

mod app;
mod camera;
mod renderer;

use std::sync::Arc;
use egui_wgpu::{WgpuSetup, WgpuSetupCreateNew};

fn main() {
    env_logger::init();

    let wgpu_setup = WgpuSetupCreateNew {
        // Request POLYGON_MODE_LINE feature for wireframe rendering
        device_descriptor: Arc::new(|adapter| {
            let base_limits = if adapter.get_info().backend == wgpu::Backend::Gl {
                wgpu::Limits::downlevel_webgl2_defaults()
            } else {
                wgpu::Limits::default()
            };

            // Request wireframe support if the adapter supports it
            let wireframe_feature = wgpu::Features::POLYGON_MODE_LINE;
            let supported = adapter.features();
            let required_features = supported & wireframe_feature;

            wgpu::DeviceDescriptor {
                label: Some("3Draper wgpu device"),
                required_features,
                required_limits: wgpu::Limits {
                    max_texture_dimension_2d: 8192,
                    ..base_limits
                },
                memory_hints: wgpu::MemoryHints::default(),
            }
        }),
        ..Default::default()
    };

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("3Draper Viewer"),
        wgpu_options: egui_wgpu::WgpuConfiguration {
            wgpu_setup: WgpuSetup::CreateNew(wgpu_setup),
            ..Default::default()
        },
        ..Default::default()
    };

    let _ = eframe::run_native(
        "3Draper Viewer",
        options,
        Box::new(|cc| {
            Ok(Box::new(app::ViewerApp::new(cc)))
        }),
    );
}
