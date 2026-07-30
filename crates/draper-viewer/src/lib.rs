// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-viewer
//! High-performance 3D model viewer using egui/wgpu.

pub mod app;
pub mod camera;
pub mod renderer;
pub mod ui;

#[cfg(target_arch = "wasm32")]
pub mod cache;
