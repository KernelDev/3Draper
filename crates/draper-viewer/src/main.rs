// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
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

    /// Show an error page in the browser, replacing the loading overlay.
    fn show_error_page(document: &web_sys::Document, html: &str) {
        if let Some(body) = document.body() {
            let error_div = document.create_element("div").unwrap();
            error_div.set_inner_html(&format!(
                "<div style='color:#ff6b6b;padding:20px;font-family:sans-serif;max-width:600px;margin:40px auto;'>\
                {}\
                </div>", html
            ));
            let _ = body.append_child(&error_div);
            // Hide loading overlay
            if let Some(loading) = document.get_element_by_id("loading") {
                loading.set_attribute("style", "display:none").ok();
            }
        }
    }

    /// This is the entry point for the web version.
    /// It is called automatically when the wasm module is loaded.
    #[wasm_bindgen(start)]
    pub async fn start() {
        console_log::init_with_level(log::Level::Info).ok();

        let window = web_sys::window().expect("no window");
        let document = window.document().expect("no document");
        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("failed to find the_canvas_id")
            .unchecked_into::<web_sys::HtmlCanvasElement>();

        // ── Feature detection: check if WebGL2/WebGPU is available BEFORE
        // attempting wgpu initialization. This prevents silent failures where
        // the canvas context is already taken or WebGL2 is not supported.
        //
        // NOTE: We probe using a temporary context check. If canvas.getContext
        // returns null for "webgl2", the browser doesn't support it or the
        // canvas is already in use by another context. We do NOT actually
        // take the context here — just check availability.
        // However, canvas.getContext("webgl2") will CREATE a context if one
        // doesn't exist yet. This is fine because wgpu will use it.
        let webgl2_available = canvas.get_context("webgl2")
            .map(|ctx| ctx.is_some())
            .unwrap_or(false);

        if !webgl2_available {
            // Try WebGL1 as a last resort (very limited but better than nothing)
            let webgl1_available = canvas.get_context("webgl")
                .map(|ctx| ctx.is_some())
                .unwrap_or(false);

            if !webgl1_available {
                let msg = "Neither WebGPU nor WebGL is available in this browser. \
                           The 3D viewer requires at least WebGL2 support.";
                log::error!("{}", msg);
                show_error_page(&document, &format!(
                    "<h2>3Draper — Graphics Not Available</h2>\
                     <p>{}</p>\
                     <p style='color:#888;font-size:14px;'>\
                     Try using Chrome 113+, Edge 113+, or Firefox with WebGL2 enabled.\
                     </p>", msg
                ));
                return;
            }
            log::warn!("WebGL2 not available, falling back to WebGL1 — rendering may be limited");
        }

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
                        device_descriptor: std::sync::Arc::new(|adapter| {
                            let base_limits = if adapter.get_info().backend == wgpu::Backend::Gl {
                                let mut limits = wgpu::Limits::downlevel_webgl2_defaults();
                                // WebGL2 on many browsers only supports 6-8 color attachments.
                                // Default downlevel_webgl2_defaults sets 8, but some only support 6.
                                // Clamp to the adapter's actual limit to avoid errors.
                                let adapter_limits = adapter.limits();
                                limits.max_color_attachments = limits.max_color_attachments.min(adapter_limits.max_color_attachments);
                                limits
                            } else {
                                wgpu::Limits::default()
                            };

                            wgpu::DeviceDescriptor {
                                label: Some("3Draper wgpu device (web)"),
                                required_features: wgpu::Features::default(),
                                required_limits: wgpu::Limits {
                                    max_texture_dimension_2d: 8192.min(base_limits.max_texture_dimension_2d),
                                    ..base_limits
                                },
                                memory_hints: wgpu::MemoryHints::default(),
                            }
                        }),
                        ..Default::default()
                    }
                ),
                ..Default::default()
            },
            ..Default::default()
        };

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
                let msg = format!("3Draper failed to start: {e:?}");
                log::error!("{msg}");
                show_error_page(&document, &format!(
                    "<h2>3Draper — Rendering Error</h2>\
                     <p>{msg}</p>\
                     <p style='color:#888;font-size:14px;'>\
                     Make sure you're using a browser with WebGPU or WebGL2 support.\
                     Try Chrome 113+, Edge 113+, or Firefox Nightly with WebGPU enabled.\
                     </p>"
                ));
            }
        }
    }
}
