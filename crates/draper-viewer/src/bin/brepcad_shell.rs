// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! BRepCAD UI Shell — wraps the existing draper-viewer ViewerApp with the
//! extended BRepCAD UI (21-menu bar + 15-tab ribbon + command palette + dialogs).
//!
//! This is a THIN WRAPPER: the actual 3D rendering, structure panel, face info,
//! selection, picking, NURBS gallery, STEP/STL/JSON import/export, manifold
//! checks, and all other existing functionality come from `ViewerApp`.
//! This binary only enables the extended menu/ribbon UI via the
//! `enable_brepcad_ui` flag on `ViewerApp`.
//!
//! Usage:
//!     cargo run --bin brepcad-shell

use std::sync::Arc;
use egui_wgpu::{WgpuSetup, WgpuSetupCreateNew};

fn main() -> Result<(), eframe::Error> {
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
                label: Some("BRepCAD wgpu device"),
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
            .with_inner_size([1400.0, 900.0])
            .with_title("BRepCAD — 3Draper-powered CAD/CAE/CAM"),
        wgpu_options: egui_wgpu::WgpuConfiguration {
            wgpu_setup: WgpuSetup::CreateNew(wgpu_setup),
            ..Default::default()
        },
        ..Default::default()
    };

    eframe::run_native(
        "BRepCAD",
        options,
        Box::new(|cc| {
            let mut app = draper_viewer::app::ViewerApp::new(cc);
            // Enable the BRepCAD extended UI (21-menu bar + 15-tab ribbon)
            app.enable_brepcad_ui = true;
            Ok(Box::new(app))
        }),
    )
}
