//! 3Draper STEP Viewer
//!
//! A minimal viewer that can:
//! - Open STEP files (all AP versions)
//! - Display the file structure tree
//! - Render the 3D geometry
//! - Save STEP files

mod app;
mod render;
mod structure_tree;

fn main() -> eframe::Result<()> {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("3Draper — STEP Viewer"),
        ..Default::default()
    };

    eframe::run_native(
        "3Draper",
        options,
        Box::new(|cc| {
            Ok(Box::new(app::DraperViewer::new(cc)))
        }),
    )
}
