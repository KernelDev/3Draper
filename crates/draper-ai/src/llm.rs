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
// HttpLlmClient — real HTTP client for Ollama/OpenAI
// ============================================================

/// An HTTP-based LLM client that sends requests to an OpenAI-compatible
/// `/v1/chat/completions` endpoint (Ollama, OpenAI, LM Studio, etc.).
///
/// Uses raw `std::net::TcpStream` for HTTP/1.1 — no external HTTP
/// dependency required. This keeps the WASM build clean (HTTP client
/// is only used on native; WASM falls back to MockLlmClient).
///
/// # Protocol
///
/// Sends a POST request with JSON body:
/// ```json
/// {
///   "model": "llama3.2",
///   "messages": [
///     {"role": "system", "content": "..."},
///     {"role": "user", "content": "..."}
///   ],
///   "max_tokens": 256,
///   "temperature": 0.2,
///   "stream": false
/// }
/// ```
///
/// Parses the JSON response to extract `choices[0].message.content`.
pub struct HttpLlmClient {
    config: HttpLlmConfig,
}

impl HttpLlmClient {
    /// Create a new HTTP LLM client with the given configuration.
    pub fn new(config: HttpLlmConfig) -> Self {
        Self { config }
    }

    /// Create a client with default Ollama configuration
    /// (http://localhost:11434/v1, model llama3.2).
    pub fn ollama() -> Self {
        Self::new(HttpLlmConfig::default())
    }

    /// Create a client for OpenAI API.
    pub fn openai(api_key: &str, model: &str) -> Self {
        Self::new(HttpLlmConfig {
            base_url: "https://api.openai.com/v1".to_string(),
            api_key: Some(api_key.to_string()),
            model: model.to_string(),
            ..Default::default()
        })
    }

    /// Get the configuration.
    pub fn config(&self) -> &HttpLlmConfig {
        &self.config
    }

    /// Build the JSON request body for the OpenAI chat completions API.
    fn build_request_body(&self, prompt: &str) -> String {
        let system = SYSTEM_PROMPT.replace('"', "\\\"").replace('\n', "\\n");
        let user = prompt.replace('"', "\\\"").replace('\n', "\\n");
        format!(
            r#"{{"model":"{}","messages":[{{"role":"system","content":"{}"}},{{"role":"user","content":"{}"}}],"max_tokens":{},"temperature":{},"stream":false}}"#,
            self.config.model, system, user, self.config.max_tokens, self.config.temperature
        )
    }

    /// Parse the JSON response from the LLM API.
    /// Extracts `choices[0].message.content` from the response.
    fn parse_response(json: &str) -> Result<String, LlmError> {
        // Simple JSON parsing without a full JSON parser — find the
        // "content" field in the first choice's message.
        // This is a minimal parser that works for OpenAI-compatible responses.
        let val: serde_json::Value = serde_json::from_str(json)
            .map_err(|e| LlmError::InvalidResponse(format!("JSON parse error: {}", e)))?;

        let content = val
            .get("choices")
            .and_then(|c| c.get(0))
            .and_then(|c| c.get("message"))
            .and_then(|m| m.get("content"))
            .and_then(|c| c.as_str())
            .ok_or_else(|| LlmError::InvalidResponse("Missing choices[0].message.content".to_string()))?;

        Ok(content.trim().to_string())
    }

    /// Perform a synchronous HTTP POST request to the LLM API.
    /// Returns the response body as a string.
    #[cfg(not(target_arch = "wasm32"))]
    fn http_post(&self, path: &str, body: &str) -> Result<String, LlmError> {
        use std::io::{Read, Write};
        use std::net::TcpStream;
        use std::time::Duration;

        // Parse base_url to extract host and port
        let url = &self.config.base_url;
        let url = url.strip_prefix("http://").or_else(|| url.strip_prefix("https://")).unwrap_or(url);
        let (host_port, _path) = url.split_once('/').unwrap_or((url, ""));

        // Connect
        let stream = TcpStream::connect(host_port)
            .map_err(|e| LlmError::Network(format!("Connection failed to {}: {}", host_port, e)))?;
        stream.set_read_timeout(Some(Duration::from_millis(self.config.timeout_ms)))
            .map_err(|e| LlmError::Network(format!("Set read timeout: {}", e)))?;
        stream.set_write_timeout(Some(Duration::from_millis(self.config.timeout_ms)))
            .map_err(|e| LlmError::Network(format!("Set write timeout: {}", e)))?;

        // Build HTTP request
        let auth_header = if let Some(ref key) = self.config.api_key {
            format!("Authorization: Bearer {}\r\n", key)
        } else {
            String::new()
        };

        let request = format!(
            "POST {} HTTP/1.1\r\nHost: {}\r\n{}Content-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            path, host_port, auth_header, body.len(), body
        );

        let mut stream = stream;
        stream.write_all(request.as_bytes())
            .map_err(|e| LlmError::Network(format!("Write request: {}", e)))?;

        // Read response
        let mut response = String::new();
        stream.read_to_string(&mut response)
            .map_err(|e| LlmError::Network(format!("Read response: {}", e)))?;

        // Extract body (skip HTTP headers)
        let body_start = response.find("\r\n\r\n")
            .ok_or_else(|| LlmError::InvalidResponse("No HTTP body separator".to_string()))?;
        let http_body = &response[body_start + 4..];

        // Check for chunked transfer encoding
        if response.to_lowercase().contains("transfer-encoding: chunked") {
            return Self::decode_chunked(http_body);
        }

        Ok(http_body.to_string())
    }

    /// Decode HTTP chunked transfer encoding.
    #[cfg(not(target_arch = "wasm32"))]
    fn decode_chunked(body: &str) -> Result<String, LlmError> {
        let mut result = String::new();
        let mut pos = 0;
        let bytes = body.as_bytes();

        while pos < bytes.len() {
            // Find the chunk size line
            let line_end = body[pos..].find("\r\n")
                .ok_or_else(|| LlmError::InvalidResponse("Malformed chunked encoding".to_string()))?;
            let size_str = &body[pos..pos + line_end];
            let chunk_size = usize::from_str_radix(size_str.trim(), 16)
                .map_err(|_| LlmError::InvalidResponse(format!("Invalid chunk size: {}", size_str)))?;

            if chunk_size == 0 {
                break; // End of chunks
            }

            pos += line_end + 2; // Skip size line + \r\n
            if pos + chunk_size > bytes.len() {
                return Err(LlmError::InvalidResponse("Chunk extends past body".to_string()));
            }
            result.push_str(&body[pos..pos + chunk_size]);
            pos += chunk_size + 2; // Skip chunk data + \r\n
        }

        Ok(result)
    }
}

impl LlmClient for HttpLlmClient {
    fn expand_prompt<'a>(&'a self, prompt: &'a str) -> LlmFuture<'a, LlmResponse> {
        let config = self.config.clone();
        let prompt_owned = prompt.to_string();

        Box::pin(async move {
            // On WASM, fall back to error (use MockLlmClient instead)
            #[cfg(target_arch = "wasm32")]
            {
                let _ = (config, prompt_owned);
                return Err(LlmError::Network("HTTP LLM not available on WASM — use MockLlmClient".to_string()));
            }

            #[cfg(not(target_arch = "wasm32"))]
            {
                let start = std::time::Instant::now();
                let body = HttpLlmClient {
                    config: config.clone(),
                };
                let request_body = body.build_request_body(&prompt_owned);
                let path = if config.base_url.ends_with("/v1") {
                    "/v1/chat/completions"
                } else if config.base_url.ends_with('/') {
                    "v1/chat/completions"
                } else {
                    "/v1/chat/completions"
                };

                let response_json = body.http_post(path, &request_body)?;
                let text = HttpLlmClient::parse_response(&response_json)?;
                let latency_ms = start.elapsed().as_millis() as u64;

                Ok(LlmResponse {
                    text,
                    model: config.model,
                    latency_ms,
                    prompt_tokens: prompt_owned.len() / 4,
                    completion_tokens: 50, // Approximate
                })
            }
        })
    }

    fn model_name(&self) -> &str {
        &self.config.model
    }
}

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

    // ─── HttpLlmClient tests ───

    #[test]
    fn test_http_llm_client_ollama_default() {
        let client = HttpLlmClient::ollama();
        assert_eq!(client.model_name(), "llama3.2");
        assert_eq!(client.config().base_url, "http://localhost:11434/v1");
        assert!(client.config().api_key.is_none());
    }

    #[test]
    fn test_http_llm_client_openai() {
        let client = HttpLlmClient::openai("sk-test123", "gpt-4o-mini");
        assert_eq!(client.model_name(), "gpt-4o-mini");
        assert_eq!(client.config().base_url, "https://api.openai.com/v1");
        assert_eq!(client.config().api_key.as_deref(), Some("sk-test123"));
    }

    #[test]
    fn test_http_llm_build_request_body() {
        let client = HttpLlmClient::ollama();
        let body = client.build_request_body("box 50x30x20");
        assert!(body.contains("\"model\":\"llama3.2\""));
        assert!(body.contains("\"messages\":["));
        assert!(body.contains("\"role\":\"system\""));
        assert!(body.contains("\"role\":\"user\""));
        assert!(body.contains("box 50x30x20"));
        assert!(body.contains("\"stream\":false"));
    }

    #[test]
    fn test_http_llm_parse_response_valid() {
        let json = r#"{"choices":[{"message":{"content":"box 50x30x5 holes of diameter 5 fillet 2"}}]}"#;
        let result = HttpLlmClient::parse_response(json).unwrap();
        assert_eq!(result, "box 50x30x5 holes of diameter 5 fillet 2");
    }

    #[test]
    fn test_http_llm_parse_response_missing_content() {
        let json = r#"{"choices":[]}"#;
        let result = HttpLlmClient::parse_response(json);
        assert!(result.is_err());
    }

    #[test]
    fn test_http_llm_parse_response_invalid_json() {
        let result = HttpLlmClient::parse_response("not json at all");
        assert!(result.is_err());
    }

    #[test]
    fn test_http_llm_parse_response_with_whitespace() {
        let json = r#"{"choices":[{"message":{"content":"  box 20x20x20  "}}]}"#;
        let result = HttpLlmClient::parse_response(json).unwrap();
        assert_eq!(result, "box 20x20x20"); // Trimmed
    }

    #[test]
    fn test_http_llm_connection_error() {
        // Try connecting to a port that's definitely not listening
        let client = HttpLlmClient::new(HttpLlmConfig {
            base_url: "http://127.0.0.1:1/v1".to_string(),
            timeout_ms: 500,
            ..Default::default()
        });
        let rt = tokio::runtime::Runtime::new().unwrap();
        let result = rt.block_on(client.expand_prompt("test"));
        assert!(result.is_err());
        // Should be a Network error, not a panic
        match result {
            Err(LlmError::Network(_)) => {} // Expected
            Err(e) => panic!("Expected Network error, got: {:?}", e),
            Ok(_) => panic!("Expected error, got Ok"),
        }
    }

    #[test]
    fn test_http_llm_model_name() {
        let client = HttpLlmClient::new(HttpLlmConfig {
            model: "custom-model".to_string(),
            ..Default::default()
        });
        assert_eq!(client.model_name(), "custom-model");
    }
}
