// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! HTTP server for the JSON API.
//!
//! Provides a REST-like HTTP interface for the 3Draper kernel.
//! Only available on native targets (not WASM).
//!
//! ## Endpoints
//! - `POST /api` — Execute a JSON API command
//! - `GET /health` — Health check
//! - `GET /api/help` — List available commands
//! - `POST /api/load_step` — Load STEP file
//! - `GET /api/stats` — Get model statistics
//! - `GET /api/mesh` — Get mesh data
//! - `GET /api/assembly` — Get assembly tree
//! - `GET /api/instances` — Get instances summary
//! - `GET /api/bbox` — Get bounding box

use crate::api::{ApiRequest, ApiResponse, JsonApi};
use std::sync::Mutex;

/// Run the JSON API HTTP server on the given port.
pub fn run_server(port: u16) -> Result<(), String> {
    let api = Mutex::new(JsonApi::new());
    let addr = format!("0.0.0.0:{}", port);

    let server = tiny_http::Server::http(&addr)
        .map_err(|e| format!("Failed to start server on {}: {}", addr, e))?;

    log::info!("3Draper JSON API server listening on {}", addr);
    println!("3Draper JSON API server listening on http://{}", addr);
    println!("Endpoints:");
    println!("  POST /api          — Execute JSON API command");
    println!("  GET  /health       — Health check");
    println!("  GET  /api/help     — List available commands");
    println!("  POST /api/load_step — Load STEP file (body = step text)");
    println!("  GET  /api/stats    — Model statistics");
    println!("  GET  /api/mesh     — Mesh data");
    println!("  GET  /api/assembly — Assembly tree");
    println!("  GET  /api/instances — Instances summary");
    println!("  GET  /api/bbox     — Bounding box");

    for request in server.incoming_requests() {
        handle_request(request, &api);
    }

    Ok(())
}

fn handle_request(request: tiny_http::Request, api: &Mutex<JsonApi>) {
    let path = request.url().to_string();
    let method = request.method().clone();

    let response_body = match (method, path.as_str()) {
        // Health check
        (tiny_http::Method::Get, "/health") => {
            serde_json::to_string(&ApiResponse::ok_msg("OK")).unwrap_or_default()
        }

        // API help
        (tiny_http::Method::Get, "/api/help") => {
            let mut api_guard = api.lock().unwrap();
            let response = api_guard.execute(ApiRequest::Help);
            serde_json::to_string(&response).unwrap_or_default()
        }

        // General API endpoint — command in JSON body
        (tiny_http::Method::Post, "/api") => {
            let mut body = String::new();
            if request.as_reader().read_to_string(&mut body).is_ok() {
                let mut api_guard = api.lock().unwrap();
                api_guard.execute_json(&body)
            } else {
                serde_json::to_string(&ApiResponse::err("Failed to read request body"))
                    .unwrap_or_default()
            }
        }

        // Load STEP file
        (tiny_http::Method::Post, "/api/load_step") => {
            let mut body = String::new();
            if request.as_reader().read_to_string(&mut body).is_ok() {
                let mut api_guard = api.lock().unwrap();
                let req = ApiRequest::LoadStep { content: body, heal: true };
                let response = api_guard.execute(req);
                serde_json::to_string(&response).unwrap_or_default()
            } else {
                serde_json::to_string(&ApiResponse::err("Failed to read request body"))
                    .unwrap_or_default()
            }
        }

        // Get stats
        (tiny_http::Method::Get, "/api/stats") => {
            let mut api_guard = api.lock().unwrap();
            let response = api_guard.execute(ApiRequest::GetStats);
            serde_json::to_string(&response).unwrap_or_default()
        }

        // Get mesh
        (tiny_http::Method::Get, "/api/mesh") => {
            let mut api_guard = api.lock().unwrap();
            let response = api_guard.execute(ApiRequest::GetMesh { instance_index: None });
            serde_json::to_string(&response).unwrap_or_default()
        }

        // Get assembly
        (tiny_http::Method::Get, "/api/assembly") => {
            let mut api_guard = api.lock().unwrap();
            let response = api_guard.execute(ApiRequest::GetAssembly);
            serde_json::to_string(&response).unwrap_or_default()
        }

        // Get instances
        (tiny_http::Method::Get, "/api/instances") => {
            let mut api_guard = api.lock().unwrap();
            let response = api_guard.execute(ApiRequest::GetInstances);
            serde_json::to_string(&response).unwrap_or_default()
        }

        // Get bounding box
        (tiny_http::Method::Get, "/api/bbox") => {
            let mut api_guard = api.lock().unwrap();
            let response = api_guard.execute(ApiRequest::GetBbox);
            serde_json::to_string(&response).unwrap_or_default()
        }

        // CORS preflight
        (tiny_http::Method::Options, _) => {
            "".to_string()
        }

        // 404
        _ => {
            serde_json::to_string(&ApiResponse::err(&format!("Not found: {} {}", method, path)))
                .unwrap_or_default()
        }
    };

    let response = tiny_http::Response::from_string(response_body)
        .with_header(
            "Content-Type: application/json".parse::<tiny_http::Header>().unwrap()
        )
        .with_header(
            "Access-Control-Allow-Origin: *".parse::<tiny_http::Header>().unwrap()
        )
        .with_header(
            "Access-Control-Allow-Methods: GET, POST, OPTIONS".parse::<tiny_http::Header>().unwrap()
        )
        .with_header(
            "Access-Control-Allow-Headers: Content-Type".parse::<tiny_http::Header>().unwrap()
        );

    let _ = request.respond(response);
}

use std::io::Read;
