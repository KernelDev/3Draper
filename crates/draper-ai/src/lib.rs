// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # draper-ai
//! AI-assisted design module for the 3Draper kernel.
//!
//! Provides:
//! - **Defect classification** (4.6.1): ML-based classification of mesh defects
//!   using rule-based decision trees and heuristic scoring.
//! - **Healing strategy selection** (4.6.2): Automatic selection of optimal
//!   healing strategies based on classified defects.
//! - **Predictive mesh optimization** (4.6.3): Predicts optimal triangulation
//!   parameters based on mesh quality analysis and target use-case.
//! - **Shape parser** (Phase 5.2): Natural-language → geometry actions.
//! - **Design reviewer** (Phase 5.2): Manufacturability analysis.
//! - **LLM client** (Phase 5.2): Pluggable LLM backend for prompt expansion.

#![warn(clippy::unwrap_used)]

pub mod classifier;
pub mod strategy;
pub mod predictive;
pub mod shape_parser;
pub mod design_reviewer;
pub mod llm;
pub mod healing_ml;
pub mod shape_from_text;
pub mod design_review;

pub use classifier::*;
pub use strategy::*;
pub use predictive::*;
pub use shape_parser::{GeometryAction, ParseError, ShapeParser};
pub use design_reviewer::{
    DesignReviewer as AiDesignReviewer, ReviewCategory, ReviewConfig, ReviewIssue, ReviewReport,
    ReviewSeverity, ReviewStats,
};
pub use llm::{
    HttpLlmConfig, LlmClient, LlmError, LlmResponse, MockLlmClient, SYSTEM_PROMPT,
    parse_with_llm,
};
pub use healing_ml::*;
pub use shape_from_text::*;
pub use design_review::*;
