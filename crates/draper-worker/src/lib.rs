// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Thin wrapper crate for the STEP triangulation Web Worker.
//!
//! This crate simply re-exports `draper_wasm` with the `worker` feature
//! enabled. Building this crate with `--target wasm32-unknown-unknown`
//! produces `draper-worker.wasm` — a lightweight WASM binary containing
//! only STEP parsing and triangulation functions (no eframe/egui/wgpu).
//!
//! ## Build
//!
//! ```sh
//! cargo build -p draper-worker --release --target wasm32-unknown-unknown
//! wasm-bindgen --target web --no-typescript --out-dir dist/ target/wasm32-unknown-unknown/release/draper_worker.wasm
//! ```

// Re-export everything from draper-wasm (with worker feature enabled).
// The worker feature causes draper-wasm to only compile `src/worker.rs`,
// which exports `parse_step_worker`, `triangulate_brep_structured`, etc.
pub use draper_wasm::*;
