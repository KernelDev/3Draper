// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-cloud
//!
//! Cloud/Collaboration integration module for the 3Draper geometric kernel.
//!
//! This crate provides infrastructure for streaming, incremental loading,
//! server-side rendering, and collaborative editing of 3D CAD models.
//!
//! # Modules
//!
//! - **stream** — Progressive mesh streaming for large models (4.5.1)
//! - **incremental** — Incremental assembly loading with LOD (4.5.2)
//! - **server** — Server-side rendering infrastructure (4.5.3)
//! - **collab** — Conflict resolution for collaborative editing (4.5.4)

#![warn(clippy::unwrap_used)]

pub mod stream;
pub mod incremental;
pub mod server;
pub mod collab;

// Re-export primary types from each module for convenience.
pub use stream::{
    CancellationToken, ChunkCallback, MeshChunk, StreamConfig, StreamTriangulator,
    reassemble_chunks,
};

pub use incremental::{
    AssemblyNode, IncrementalLoader, LoadPriority, LodSpec,
};

pub use server::{
    BandwidthClass, CameraState, RenderRequest, RenderResponse, RenderServer,
    RenderSession, Viewport, WsMessage,
};

pub use collab::{
    CollabSession, ConflictResolution, Operation, OperationalTransform,
    PendingOperation, TransformedOperation, VersionVector,
};
