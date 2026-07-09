// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-wasm
//! JavaScript / WASM bindings for the 3Draper kernel.
//!
//! This crate exposes the kernel's full surface area (modeling, boolean,
//! GDT, STEP I/O, mesh export) to JavaScript via `wasm-bindgen`.
//!
//! ## Quick start
//!
//! ```js
//! import init, { Document, GdtType } from "./draper_wasm.js";
//! await init();
//! const doc = Document.new("my doc");
//! doc.addBox(100, 80, 60);
//! doc.filletEdge(0, 0, 5.0);          // fillet first manifold edge of solid 0
//! const mesh = doc.triangulate();
//! console.log(mesh.vertexCount, mesh.triangleCount);
//! const stl = mesh.exportStlBinary(); // Uint8Array
//! ```
//!
//! ## Worker mode
//!
//! When compiled with the `worker` feature, this crate only exports the
//! STEP parsing and triangulation functions needed by the Web Worker.
//! See `src/worker.rs` for the worker-specific API.

#![allow(clippy::unused_unit)]

// ─── Worker mode: only the worker module ────────────────────────────
#[cfg(feature = "worker")]
mod worker;

// ─── Normal mode: full bindings ─────────────────────────────────────
#[cfg(not(feature = "worker"))]
mod main_bindings;

#[cfg(not(feature = "worker"))]
#[cfg(test)]
#[cfg(not(target_arch = "wasm32"))]
mod tests;
