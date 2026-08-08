// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! LLM client for natural-language → geometry expansion.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 5.2: provides an interface
//! to a Large Language Model (LLM) that can expand vague design requests
//! ("a bracket with 4 holes") into canonical prompts that the rule-based
//! `ShapeParser` can understand ("box 50x30x20 holes of diameter 5 fillet 2").
//!
//! # Architecture
//!
//! The `LlmClient` trait abstracts the LLM backend so that:
//!
//! - **In tests / WASM**: `MockLlmClient` returns canned responses
//!   (no network required).
//! - **In production**: `HttpLlmClient` sends a POST request to an
//!   OpenAI-compatible API endpoint (e.g., `http://localhost:11434/v1`
//!   for Ollama, or `https://api.openai.com/v1` for OpenAI).
//!
//! # Why trait + mock?
//!
//! 1. Tests don't depend on network availability.
//! 2. WASM builds can't make raw HTTP requests — they need `fetch()`.
//! 3. Different deployments may use different LLM providers.
//!
//! # Prompt design
//!
//! The system prompt instructs the LLM to:
//!
//! 1. Output ONLY canonical shape descriptions (no prose).
//! 2. Use units (mm, cm, m, in) explicitly.
//! 3. List each operation on its own line.
//! 4. Avoid ambiguous terms (use "box" not "block thing").
//!
//! Example LLM expansion:
//! ```text
//! Input:  "I need a mounting bracket for a stepper motor"
//! Output: "box 60x60x5 holes of diameter 5 fillet 2"
//! ```

use crate::shape_parser::{GeometryAction, ParseError, ShapeParser};
use serde::{Deserialize, Serialize};
use std::future::Future;
use std::pin::Pin;

// ============================================================
// LLM client trait
// ============================================================

/// A response from an LLM backend.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LlmResponse {
    /// The expanded text (canonical shape description).
    pub text: String,
    /// Model name that generated the response.
    pub model: String,
    /// Time taken in milliseconds.
    pub latency_ms: u64,
    /// Number of tokens in the prompt.
    pub prompt_tokens: usize,
    /// Number of tokens in the response.
    pub completion_tokens: usize,
}

/// Error from an LLM backend.
#[derive(Debug, Clone, thiserror::Error)]
pub enum LlmError {
    #[error("network error: {0}")]
    Network(String),

    #[error("API error: {status} {body}")]
    Api { status: u16, body: String },

    #[error("rate limited — retry after {retry_after_ms}ms")]
    RateLimited { retry_after_ms: u64 },

    #[error("timeout after {0}ms")]
    Timeout(u64),

    #[error("invalid response: {0}")]
    InvalidResponse(String),

    #[error("model not found: {0}")]
    ModelNotFound(String),
}

/// Future returned by LLM client methods.
pub type LlmFuture<'a, T> = Pin<Box<dyn Future<Output = Result<T, LlmError>> + Send + 'a>>;

/// Trait for LLM backends (mock, HTTP, WASM-fetch, etc.).
pub trait LlmClient: Send + Sync {
    /// Expand a vague design prompt into a canonical shape description.
    fn expand_prompt<'a>(&'a self, prompt: &'a str) -> LlmFuture<'a, LlmResponse>;

    /// Get the model name.
    fn model_name(&self) -> &str;
}

// ============================================================
// MockLlmClient (for tests / offline use)
// ============================================================

/// A mock LLM client that returns canned responses based on keyword matching.
///
/// Useful for testing the prompt → action pipeline without a real LLM.
pub struct MockLlmClient {
    /// Canned responses: (keyword_in_prompt, response_text).
    responses: Vec<(String, String)>,
    /// Default response if no keyword matches.
    default: String,
}

impl Default for MockLlmClient {
    fn default() -> Self {
        Self::new()
    }
}

impl MockLlmClient {
    pub fn new() -> Self {
        let mut client = Self {
            responses: Vec::new(),
            default: "box 10x10x10".to_string(),
        };
        // Pre-populate with common patterns
        client.add_pattern("bracket", "box 50x30x5 holes of diameter 5 fillet 2");
        client.add_pattern("mounting", "box 60x60x5 holes of diameter 5 fillet 2");
        client.add_pattern("plate", "box 100x100x5 fillet 1");
        client.add_pattern("shaft", "cylinder 10x100");
        client.add_pattern("rod", "cylinder 8x50");
        client.add_pattern("bearing", "cylinder 20x8");
        client.add_pattern("housing", "box 80x60x40 shell 2 fillet 3");
        client.add_pattern("gear", "cylinder 40x10");
        client.add_pattern("pulley", "cylinder 30x15");
        client.add_pattern("flange", "cylinder 50x5 holes of diameter 5");
        client.add_pattern("cover", "box 50x50x2 shell 1");
        client.add_pattern("cap", "box 30x30x5 fillet 2");
        client
    }

    /// Add a keyword → response mapping.
    pub fn add_pattern(&mut self, keyword: &str, response: &str) {
        self.responses.push((keyword.to_lowercase(), response.to_string()));
    }

    /// Set the default response when no pattern matches.
    pub fn set_default(&mut self, default: &str) {
        self.default = default.to_string();
    }
}

impl LlmClient for MockLlmClient {
    fn expand_prompt<'a>(&'a self, prompt: &'a str) -> LlmFuture<'a, LlmResponse> {
        let prompt_lower = prompt.to_lowercase();
        let text = self
            .responses
            .iter()
            .find(|(kw, _)| prompt_lower.contains(kw))
            .map(|(_, r)| r.clone())
            .unwrap_or_else(|| self.default.clone());

        Box::pin(async move {
            Ok(LlmResponse {
                text,
                model: "mock-llm-v1".to_string(),
                latency_ms: 1,
                prompt_tokens: prompt.len() / 4, // Rough estimate
                completion_tokens: 20,
            })
        })
    }

    fn model_name(&self) -> &str {
        "mock-llm-v1"
    }
}

// ============================================================
// HttpLlmClient config (not implemented — requires HTTP client)
// ============================================================

/// Configuration for an HTTP-based LLM client.
///
/// This struct is provided for documentation and future implementation.
/// A real implementation would use `reqwest` or `hyper` to POST to an
/// OpenAI-compatible `/v1/chat/completions` endpoint.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HttpLlmConfig {
    /// Base URL of the API (e.g., "http://localhost:11434/v1" for Ollama).
    pub base_url: String,
    /// API key (optional for local servers like Ollama).
    pub api_key: Option<String>,
    /// Model name (e.g., "llama3.2", "gpt-4o-mini").
    pub model: String,
    /// Maximum tokens in the response.
    pub max_tokens: usize,
    /// Temperature (0.0 = deterministic, 1.0 = creative).
    pub temperature: f64,
    /// Request timeout in milliseconds.
    pub timeout_ms: u64,
}

impl Default for HttpLlmConfig {
    fn default() -> Self {
        Self {
            base_url: "http://localhost:11434/v1".to_string(),
            api_key: None,
            model: "llama3.2".to_string(),
            max_tokens: 256,
            temperature: 0.2, // Low temperature for deterministic shape descriptions
            timeout_ms: 10_000,
        }
    }
}

/// The system prompt sent to the LLM to instruct it to output canonical
/// shape descriptions.
pub const SYSTEM_PROMPT: &str = r#"You are a CAD design assistant. Convert the user's request into a canonical shape description that the ShapeParser can understand.

Rules:
1. Output ONLY the shape description (no prose, no explanations).
2. Use lowercase.
3. Use units explicitly (mm, cm, m, in).
4. List each operation on a single line, space-separated.
5. Use these keywords only: box, cube, block, cylinder, rod, shaft, sphere, ball, cone, torus, ring, donut, holes, fillet, chamfer, shell, hollow.
6. Format dimensions as: shape WxHxD (e.g., "box 50x30x20").
7. For holes: "holes of diameter D" (D in mm).

Examples:
- "a bracket with 4 holes" → "box 50x30x5 holes of diameter 5 fillet 2"
- "a 100mm shaft" → "cylinder 10x100"
- "a small box" → "box 20x20x20"
- "a hollow sphere" → "sphere 30 shell 1"

Respond with ONLY the canonical description, nothing else."#;

// ============================================================
// High-level: parse prompt via LLM
// ============================================================

/// Parse a natural-language prompt into geometry actions, using an LLM
/// to expand vague requests first.
///
/// This is the main entry point for AI-driven geometry. The flow is:
///
/// 1. Try the rule-based `ShapeParser` directly. If it succeeds, return.
/// 2. If it fails (unknown shape, ambiguous), call the LLM to expand
///    the prompt into a canonical form.
/// 3. Parse the LLM's response with `ShapeParser`.
/// 4. If the LLM response also fails to parse, return the original error.
pub async fn parse_with_llm(
    parser: &ShapeParser,
    llm: &dyn LlmClient,
    prompt: &str,
) -> Result<Vec<GeometryAction>, ParseError> {
    // First, try direct parsing (fast path)
    match parser.parse(prompt) {
        Ok(actions) if !actions.is_empty() => return Ok(actions),
        Ok(_) => {} // Empty actions, fall through to LLM
        Err(_) => {} // Parse error, fall through to LLM
    }

    // Expand via LLM
    let llm_response = llm
        .expand_prompt(prompt)
        .await
        .map_err(|e| ParseError::Ambiguous(format!("LLM error: {}", e)))?;

    log::info!(
        "LLM expanded '{}' → '{}' (model: {}, {}ms)",
        prompt,
        llm_response.text,
        llm_response.model,
        llm_response.latency_ms
    );

    // Parse the expanded prompt
    parser.parse(&llm_response.text)
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_llm_default_patterns() {
        let client = MockLlmClient::new();
        // The mock client is async, but we can test it via block_on
        let rt = tokio::runtime::Runtime::new().unwrap();
        let response = rt.block_on(client.expand_prompt("I need a bracket")).unwrap();
        assert!(response.text.contains("box"));
        assert!(response.text.contains("holes"));
        assert_eq!(response.model, "mock-llm-v1");
    }

    #[test]
    fn test_mock_llm_mounting_pattern() {
        let client = MockLlmClient::new();
        let rt = tokio::runtime::Runtime::new().unwrap();
        let response = rt.block_on(client.expand_prompt("mounting plate")).unwrap();
        assert!(response.text.contains("box"));
        assert!(response.text.contains("60"));
    }

    #[test]
    fn test_mock_llm_shaft_pattern() {
        let client = MockLlmClient::new();
        let rt = tokio::runtime::Runtime::new().unwrap();
        let response = rt.block_on(client.expand_prompt("steel shaft")).unwrap();
        assert!(response.text.contains("cylinder"));
    }

    #[test]
    fn test_mock_llm_default_response() {
        let client = MockLlmClient::new();
        let rt = tokio::runtime::Runtime::new().unwrap();
        let response = rt.block_on(client.expand_prompt("xyz unknown thing")).unwrap();
        assert_eq!(response.text, "box 10x10x10");
    }

    #[test]
    fn test_mock_llm_custom_pattern() {
        let mut client = MockLlmClient::new();
        client.add_pattern("gizmo", "sphere 42");
        let rt = tokio::runtime::Runtime::new().unwrap();
        let response = rt.block_on(client.expand_prompt("make me a gizmo")).unwrap();
        assert_eq!(response.text, "sphere 42");
    }

    #[test]
    fn test_mock_llm_custom_default() {
        let mut client = MockLlmClient::new();
        client.set_default("cylinder 5x20");
        let rt = tokio::runtime::Runtime::new().unwrap();
        let response = rt.block_on(client.expand_prompt("unknown thing")).unwrap();
        assert_eq!(response.text, "cylinder 5x20");
    }

    #[test]
    fn test_mock_llm_model_name() {
        let client = MockLlmClient::new();
        assert_eq!(client.model_name(), "mock-llm-v1");
    }

    #[test]
    fn test_http_llm_config_default() {
        let config = HttpLlmConfig::default();
        assert_eq!(config.base_url, "http://localhost:11434/v1");
        assert_eq!(config.model, "llama3.2");
        assert_eq!(config.max_tokens, 256);
        assert!((config.temperature - 0.2).abs() < 1e-6);
        assert_eq!(config.timeout_ms, 10_000);
        assert!(config.api_key.is_none());
    }

    #[test]
    fn test_system_prompt_contains_rules() {
        assert!(SYSTEM_PROMPT.contains("box"));
        assert!(SYSTEM_PROMPT.contains("cylinder"));
        assert!(SYSTEM_PROMPT.contains("holes"));
        assert!(SYSTEM_PROMPT.contains("fillet"));
        assert!(SYSTEM_PROMPT.contains("lowercase"));
    }

    #[tokio::test]
    async fn test_parse_with_llm_direct_succeeds() {
        // A prompt that the rule-based parser can handle directly
        let parser = ShapeParser::new();
        let llm = MockLlmClient::new();

        let actions = parse_with_llm(&parser, &llm, "box 20x30x10").await.unwrap();
        assert_eq!(actions.len(), 1);
        assert!(matches!(actions[0], GeometryAction::CreateBox { .. }));
    }

    #[tokio::test]
    async fn test_parse_with_llm_falls_back_to_llm() {
        // A vague prompt that the rule-based parser can't handle
        let parser = ShapeParser::new();
        let llm = MockLlmClient::new();

        let actions = parse_with_llm(&parser, &llm, "I need a bracket for my project").await.unwrap();
        // MockLlmClient returns "box 50x30x5 holes of diameter 5 fillet 2" for "bracket"
        assert!(!actions.is_empty());
        assert!(matches!(actions[0], GeometryAction::CreateBox { .. }));
        // Should have holes (CreateCylinder + BooleanSubtract)
        assert!(actions.iter().any(|a| matches!(a, GeometryAction::CreateCylinder { .. })));
        assert!(actions.iter().any(|a| matches!(a, GeometryAction::BooleanSubtract)));
        // Should have fillet
        assert!(actions.iter().any(|a| matches!(a, GeometryAction::FilletAllEdges { .. })));
    }

    #[tokio::test]
    async fn test_parse_with_llm_unknown_prompt_uses_default() {
        let parser = ShapeParser::new();
        let llm = MockLlmClient::new();

        // "gibberish" won't match any mock pattern, so default "box 10x10x10" is used
        let actions = parse_with_llm(&parser, &llm, "gibberish that doesn't match").await.unwrap();
        assert_eq!(actions.len(), 1);
        assert!(matches!(actions[0], GeometryAction::CreateBox { size: [10.0, 10.0, 10.0], .. }));
    }

    #[test]
    fn test_llm_response_serialization() {
        let response = LlmResponse {
            text: "box 20x20x20".to_string(),
            model: "test-model".to_string(),
            latency_ms: 100,
            prompt_tokens: 10,
            completion_tokens: 5,
        };
        let json = serde_json::to_string(&response).unwrap();
        assert!(json.contains("test-model"));
        assert!(json.contains("box 20x20x20"));

        let parsed: LlmResponse = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.text, response.text);
        assert_eq!(parsed.model, response.model);
    }

    #[test]
    fn test_llm_error_display() {
        let err = LlmError::Network("connection refused".to_string());
        assert!(format!("{}", err).contains("connection refused"));

        let err = LlmError::Api { status: 500, body: "internal error".to_string() };
        assert!(format!("{}", err).contains("500"));
        assert!(format!("{}", err).contains("internal error"));

        let err = LlmError::RateLimited { retry_after_ms: 1000 };
        assert!(format!("{}", err).contains("1000"));

        let err = LlmError::Timeout(5000);
        assert!(format!("{}", err).contains("5000"));

        let err = LlmError::InvalidResponse("bad json".to_string());
        assert!(format!("{}", err).contains("bad json"));

        let err = LlmError::ModelNotFound("gpt-99".to_string());
        assert!(format!("{}", err).contains("gpt-99"));
    }
}
