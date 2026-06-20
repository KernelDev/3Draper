// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-mesh
//! Mesh generation from B-Rep topology.
//!
//! Provides constrained Delaunay triangulation of B-Rep faces
//! and mesh output in various formats.

#![warn(clippy::unwrap_used)]
#![allow(ambiguous_glob_reexports)]

pub mod mesh;
pub mod triangulate;
pub mod stl;
pub mod manifold;
pub mod edge_cache;
pub mod adaptive;
pub mod parametric_domain;
pub mod parametric_division_2d;
pub mod poly_boundary;
pub mod subdivision;
pub mod certification;
pub mod text3d;
pub mod watertight;
pub mod custom_cdt;
pub mod earcut_adapter;
pub mod pmi_display;
pub mod wasm_api;
pub mod gdt_check;

#[cfg(feature = "export-3mf")]
pub mod export;
pub mod export_usd;

pub use mesh::*;
pub use triangulate::*;
pub use stl::*;
pub use manifold::*;
pub use edge_cache::*;
pub use text3d::*;
#[cfg(feature = "export-3mf")]
pub use export::*;
pub use certification::*;
pub use watertight::*;
pub use pmi_display::*;
pub use wasm_api::*;
pub use gdt_check::*;
pub use parametric_domain::reproject_nurbs_point;

// ============================================================
// WASM parallel threading support
// ============================================================

/// Initialize the rayon thread pool for WASM targets.
///
/// On native targets, rayon automatically spawns a thread pool.
/// On WASM, however, Web Workers must be explicitly created by the JavaScript
/// host and the rayon thread pool must be initialized via `wasm-bindgen-rayon`.
///
/// # Usage (WASM)
///
/// ```ignore
/// // In your JavaScript/WASM initialization:
/// import init, { initThreadPool } from './pkg/draper_mesh.js';
///
/// await init();
/// await initThreadPool(navigator.hardwareConcurrency);
/// ```
///
/// After calling this function, `triangulate_solid_parallel_arc()` and all
/// other rayon-based parallel functions will use the Web Worker pool.
///
/// # Native targets
///
/// On native targets, this function is a no-op — rayon's default thread pool
/// is used automatically.
///
/// # Requirements
///
/// WASM parallel threading requires:
/// 1. `SharedArrayBuffer` support in the browser (requires COOP/COEP headers)
/// 2. The `wasm-parallel` feature flag enabled in `draper-mesh`
/// 3. A web worker script that re-exports the wasm-bindgen-rayon builder
#[cfg(all(target_arch = "wasm32", feature = "wasm-parallel"))]
pub fn init_wasm_thread_pool() -> Result<(), String> {
    // Use navigator.hardwareConcurrency (via wasm-bindgen-rayon's default)
    // if num_threads is not explicitly set. The builder defaults to
    // the number of available logical cores, which is optimal for most workloads.
    wasm_bindgen_rayon::thread_pool_builder()
        .build_global()
        .map_err(|e| format!("Failed to initialize WASM thread pool: {:?}", e))
}

/// No-op on native targets — rayon's default pool is used.
#[cfg(not(all(target_arch = "wasm32", feature = "wasm-parallel")))]
pub fn init_wasm_thread_pool() -> Result<(), String> {
    Ok(())
}
