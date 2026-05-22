//! 3Draper Viewer — high-performance 3D model viewer using egui/wgpu.

mod app;
mod camera;
mod renderer;

fn main() {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("3Draper Viewer"),
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
