// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-ai
//! AI-assisted healing module for the 3Draper kernel.
//!
//! Provides:
//! - **Defect classification** (4.6.1): ML-based classification of mesh defects
//!   using rule-based decision trees and heuristic scoring.
//! - **Healing strategy selection** (4.6.2): Automatic selection of optimal
//!   healing strategies based on classified defects.
//! - **Predictive mesh optimization** (4.6.3): Predicts optimal triangulation
//!   parameters based on mesh quality analysis and target use-case.

#![warn(clippy::unwrap_used)]

pub mod classifier;
pub mod strategy;
pub mod predictive;

pub use classifier::*;
pub use strategy::*;
pub use predictive::*;
