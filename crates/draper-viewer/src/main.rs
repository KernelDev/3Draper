//! 3Draper Viewer — high-performance 3D model viewer using egui/wgpu.
//!
//! Supports both native and web (wasm32) targets.

mod app;
mod camera;
mod renderer;

// ─── Native entry point ──────────────────────────────────────────────────

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    env_logger::init();

    use std::sync::Arc;
    use egui_wgpu::{WgpuSetup, WgpuSetupCreateNew};

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

// ─── Web (wasm32) entry point ────────────────────────────────────────────

#[cfg(target_arch = "wasm32")]
fn main() {
    // Web entry: use eframe's WebRunner
    // The actual startup is handled by the wasm_bindgen start function below.
    // This main() is never called on wasm — the #[wasm_bindgen(start)] function is.
}

#[cfg(target_arch = "wasm32")]
mod web_entry {
    use eframe::WebRunner;
    use wasm_bindgen::prelude::*;

    /// This is the entry point for the web version.
    /// It is called automatically when the wasm module is loaded.
    #[wasm_bindgen(start)]
    pub async fn start() {
        console_log::init_with_level(log::Level::Info).ok();

        // Configure wgpu to use WebGL2 as fallback when WebGPU is not available
        let web_options = eframe::WebOptions {
            wgpu_options: egui_wgpu::WgpuConfiguration {
                wgpu_setup: egui_wgpu::WgpuSetup::CreateNew(
                    egui_wgpu::WgpuSetupCreateNew {
                        instance_descriptor: wgpu::InstanceDescriptor {
                            // Try WebGPU first, fall back to WebGL2
                            backends: wgpu::Backends::BROWSER_WEBGPU | wgpu::Backends::GL,
                            ..Default::default()
                        },
                        ..Default::default()
                    }
                ),
                ..Default::default()
            },
            ..Default::default()
        };

        // Get the canvas element by ID
        let window = web_sys::window().expect("no window");
        let document = window.document().expect("no document");
        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("failed to find the_canvas_id")
            .unchecked_into::<web_sys::HtmlCanvasElement>();

        let runner = WebRunner::new();
        match runner
            .start(
                canvas,
                web_options,
                Box::new(|cc| Ok(Box::new(crate::app::ViewerApp::new(cc)))),
            )
            .await
        {
            Ok(()) => {}
            Err(e) => {
                let msg = format!("3Draper failed to start: {e:?}\n\nMake sure you're using a browser with WebGPU or WebGL2 support.");
                log::error!("{msg}");
                // Show error on the page
                if let Some(window) = web_sys::window() {
                    if let Some(document) = window.document() {
                        if let Some(body) = document.body() {
                            let error_div = document.create_element("div").unwrap();
                            error_div.set_inner_html(&format!(
                                "<div style='color:#ff6b6b;padding:20px;font-family:sans-serif;max-width:600px;margin:40px auto;'>\
                                <h2>3Draper — Rendering Error</h2>\
                                <p>{msg}</p>\
                                <p style='color:#888;font-size:14px;'>Try using Chrome 113+, Edge 113+, or Firefox Nightly with WebGPU enabled.</p>\
                                </div>"
                            ));
                            let _ = body.append_child(&error_div);
                            // Hide loading overlay
                            if let Some(loading) = document.get_element_by_id("loading") {
                                loading.set_attribute("style", "display:none").ok();
                            }
                        }
                    }
                }
            }
        }
    }
}
