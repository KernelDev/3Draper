// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Collaboration panel — integrates draper-cloud WebSocket CRDT sync into the UI.
//!
//! Per Phase 5.1 UI integration: provides an interactive panel where the user
//! can connect to a collaboration server, join a session, see who's online,
//! and broadcast geometry operations to other connected clients.
//!
//! # Layout
//!
//! ```text
//! ┌── Collaboration ──────────────────────┐
//! │ Server: [http://localhost:8080  ]     │
//! │ Session: [doc-1              ]        │
//! │ Replica: [alice              ]        │
//! │                                       │
//! │ [Connect] [Disconnect]               │
//! │                                       │
//! │ Status: Connected (3 peers online)    │
//! │                                       │
//! │ Peers:                                │
//! │   • alice (you)                       │
//! │   • bob                               │
//! │   • charlie                           │
//! │                                       │
//! │ Recent Operations:                    │
//! │   [alice] AddFace #42                 │
//! │   [bob] ModifyGeometry #17            │
//! │   [alice] FilletAllEdges R=2          │
//! │                                       │
//! │ [Send Test Op] [Clear Log]            │
//! └───────────────────────────────────────┘
//! ```

use eframe::egui;
use draper_cloud::{
    CollabServer, SyncMessage, Operation,
};
use std::sync::Arc;
use tokio::sync::mpsc;

/// State for the collaboration panel.
pub struct CollabPanelState {
    /// Server URL (for display — actual connection not yet wired).
    pub server_url: String,
    /// Session ID to join.
    pub session_id: String,
    /// Replica ID (user name).
    pub replica_id: String,
    /// Whether connected to the collaboration server.
    pub connected: bool,
    /// Number of peers in the session.
    pub peer_count: usize,
    /// List of peer replica IDs.
    pub peers: Vec<String>,
    /// Log of recent operations.
    pub op_log: Vec<OpLogEntry>,
    /// The collaboration server (in-process, for testing).
    /// In production, this would be a remote WebSocket connection.
    pub server: Arc<CollabServer>,
    /// Channel receiver for messages from the server.
    pub receiver: Option<mpsc::UnboundedReceiver<SyncMessage>>,
    /// Our own sender (for sending messages to ourselves if needed).
    pub sender: Option<mpsc::UnboundedSender<SyncMessage>>,
    /// Status message.
    pub status: String,
}

/// A log entry for a collaboration operation.
#[derive(Clone, Debug)]
pub struct OpLogEntry {
    pub timestamp: String,
    pub replica_id: String,
    pub operation_desc: String,
}

impl Default for CollabPanelState {
    fn default() -> Self {
        Self {
            server_url: "http://localhost:8080".to_string(),
            session_id: "doc-1".to_string(),
            replica_id: "alice".to_string(),
            connected: false,
            peer_count: 0,
            peers: Vec::new(),
            op_log: Vec::new(),
            server: Arc::new(CollabServer::new()),
            receiver: None,
            sender: None,
            status: "Disconnected — click Connect to join".to_string(),
        }
    }
}

impl CollabPanelState {
    /// Connect to the collaboration server (in-process for now).
    ///
    /// In production, this would open a WebSocket connection to the server URL.
    /// For testing, we use the in-process CollabServer with tokio channels.
    pub fn connect(&mut self) {
        if self.connected {
            self.status = "Already connected".to_string();
            return;
        }

        let (tx, rx) = mpsc::unbounded_channel();
        self.sender = Some(tx.clone());
        self.receiver = Some(rx);

        // Use a tokio runtime to call the async handle_message
        let server = self.server.clone();
        let session_id = self.session_id.clone();
        let replica_id = self.replica_id.clone();

        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build();

        match rt {
            Ok(runtime) => {
                let result = runtime.block_on(async {
                    server.handle_message(tx, SyncMessage::Join {
                        session_id,
                        replica_id,
                    }).await
                });

                if result.is_empty() {
                    self.status = "Connect failed — no response from server".to_string();
                    return;
                }

                // Check for Joined response
                for msg in &result {
                    if let SyncMessage::Joined { session_id, replica_id, version, history_len } = msg {
                        self.connected = true;
                        self.status = format!("Connected to session '{}' ({} history ops)", session_id, history_len);
                        let _ = (replica_id, version);
                    } else if let SyncMessage::Error { message } = msg {
                        self.status = format!("Error: {}", message);
                        return;
                    }
                }
            }
            Err(e) => {
                self.status = format!("Failed to create runtime: {}", e);
            }
        }
    }

    /// Disconnect from the collaboration server.
    pub fn disconnect(&mut self) {
        if !self.connected {
            self.status = "Not connected".to_string();
            return;
        }

        let server = self.server.clone();
        let session_id = self.session_id.clone();
        let replica_id = self.replica_id.clone();

        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build();

        if let Ok(runtime) = rt {
            let _ = runtime.block_on(async {
                server.handle_message(
                    self.sender.clone().unwrap_or_else(|| mpsc::unbounded_channel().0),
                    SyncMessage::Leave { session_id, replica_id },
                ).await
            });
        }

        self.connected = false;
        self.peers.clear();
        self.peer_count = 0;
        self.receiver = None;
        self.sender = None;
        self.status = "Disconnected".to_string();
    }

    /// Send a test operation to the collaboration server.
    pub fn send_test_operation(&mut self) {
        if !self.connected {
            self.status = "Not connected — click Connect first".to_string();
            return;
        }

        let server = self.server.clone();
        let session_id = self.session_id.clone();
        let replica_id = self.replica_id.clone();
        let sender = self.sender.clone();

        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build();

        if let Ok(runtime) = rt {
            let _ = runtime.block_on(async {
                if let Some(s) = sender {
                    let _ = server.handle_message(s, SyncMessage::LocalOp {
                        session_id,
                        replica_id: replica_id.clone(),
                        seq: 0, // Server will assign
                        operation: Operation::AddFace {
                            face_id: 100,
                            geometry: vec![1, 2, 3],
                        },
                    }).await;
                }
            });
        }

        // Add to local log
        let entry = OpLogEntry {
            timestamp: current_time_str(),
            replica_id: self.replica_id.clone(),
            operation_desc: "AddFace #100".to_string(),
        };
        self.op_log.push(entry);
        if self.op_log.len() > 50 {
            self.op_log.remove(0);
        }
        self.status = "Sent test operation (AddFace #100)".to_string();
    }

    /// Poll for incoming messages from the server.
    ///
    /// This should be called each frame to process any messages that the
    /// server has broadcast to us.
    pub fn poll_messages(&mut self) {
        let Some(rx) = &mut self.receiver else {
            return;
        };

        // Drain all available messages without blocking
        while let Ok(msg) = rx.try_recv() {
            match &msg {
                SyncMessage::RemoteOp { replica_id, operation, .. } => {
                    let entry = OpLogEntry {
                        timestamp: current_time_str(),
                        replica_id: replica_id.clone(),
                        operation_desc: format!("{:?}", operation),
                    };
                    self.op_log.push(entry);
                    if self.op_log.len() > 50 {
                        self.op_log.remove(0);
                    }
                }
                SyncMessage::Presence { replicas, .. } => {
                    self.peers = replicas.clone();
                    self.peer_count = replicas.len();
                }
                _ => {}
            }
        }
    }

    /// Clear the operation log.
    pub fn clear_log(&mut self) {
        self.op_log.clear();
        self.status = "Log cleared".to_string();
    }
}

/// Get a simple timestamp string for log entries.
fn current_time_str() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let h = (secs / 3600) % 24;
    let m = (secs / 60) % 60;
    let s = secs % 60;
    format!("{:02}:{:02}:{:02}", h, m, s)
}

/// Render the collaboration panel.
///
/// Returns `Some(operation)` if the user triggered an action that should
/// be applied to the local document (e.g., a remote operation was received).
pub fn render_collab_panel(ui: &mut egui::Ui, state: &mut CollabPanelState) -> Option<Operation> {
    let mut triggered_op = None;

    // Poll for incoming messages each frame
    state.poll_messages();

    ui.heading(egui::RichText::new("Collaboration").size(14.0).strong());
    ui.separator();

    // === Connection settings ===
    ui.label(egui::RichText::new("Server:").size(11.0));
    ui.add(
        egui::TextEdit::singleline(&mut state.server_url)
            .hint_text("http://localhost:8080")
            .desired_width(ui.available_width())
            .code_editor(),
    );

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("Session:").size(11.0));
        ui.add(
            egui::TextEdit::singleline(&mut state.session_id)
                .desired_width(80.0)
                .code_editor(),
        );
        ui.label(egui::RichText::new("Name:").size(11.0));
        ui.add(
            egui::TextEdit::singleline(&mut state.replica_id)
                .desired_width(80.0)
                .code_editor(),
        );
    });

    ui.add_space(4.0);

    // === Connect/Disconnect buttons ===
    ui.horizontal(|ui| {
        let connect_enabled = !state.connected && !state.session_id.is_empty() && !state.replica_id.is_empty();
        let disconnect_enabled = state.connected;
        if ui.add_enabled(connect_enabled, egui::Button::new("Connect")).clicked() {
            state.connect();
        }
        if ui.add_enabled(disconnect_enabled, egui::Button::new("Disconnect")).clicked() {
            state.disconnect();
        }
    });

    // === Status ===
    ui.add_space(2.0);
    let status_color = if state.connected {
        egui::Color32::from_rgb(80, 180, 80)
    } else {
        egui::Color32::from_rgb(150, 150, 150)
    };
    ui.label(egui::RichText::new(&state.status).size(11.0).color(status_color));

    // === Peers ===
    if state.connected && !state.peers.is_empty() {
        ui.add_space(4.0);
        ui.label(egui::RichText::new(format!("Peers ({}):", state.peer_count)).size(11.0).strong());
        egui::ScrollArea::vertical()
            .max_height(80.0)
            .show(ui, |ui| {
                for peer in &state.peers {
                    let is_me = peer == &state.replica_id;
                    let label = if is_me {
                        format!("• {} (you)", peer)
                    } else {
                        format!("• {}", peer)
                    };
                    ui.label(egui::RichText::new(label).size(10.0));
                }
            });
    }

    // === Operation log ===
    ui.add_space(4.0);
    ui.label(egui::RichText::new("Recent Operations:").size(11.0).strong());
    if state.op_log.is_empty() {
        ui.label(egui::RichText::new("(no operations yet)").size(10.0).weak());
    } else {
        egui::ScrollArea::vertical()
            .max_height(120.0)
            .show(ui, |ui| {
                for entry in state.op_log.iter().rev() {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(&entry.timestamp).size(9.0).weak().color(egui::Color32::from_rgb(120, 120, 120)));
                        ui.label(egui::RichText::new(format!("[{}]", entry.replica_id)).size(9.0).color(egui::Color32::from_rgb(100, 160, 220)));
                        ui.label(egui::RichText::new(&entry.operation_desc).size(9.0));
                    });
                }
            });
    }

    // === Action buttons ===
    ui.horizontal(|ui| {
        if ui.add_enabled(state.connected, egui::Button::new("Send Test Op")).clicked() {
            state.send_test_operation();
        }
        if ui.button("Clear Log").clicked() {
            state.clear_log();
        }
    });

    triggered_op
}

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_collab_panel_default_state() {
        let state = CollabPanelState::default();
        assert!(!state.connected);
        assert!(state.peers.is_empty());
        assert!(state.op_log.is_empty());
        assert!(state.receiver.is_none());
        assert!(state.sender.is_none());
        assert!(state.status.contains("Disconnected"));
    }

    #[test]
    fn test_connect_and_disconnect() {
        let mut state = CollabPanelState::default();
        state.connect();
        assert!(state.connected);
        assert!(state.status.contains("Connected"));
        state.disconnect();
        assert!(!state.connected);
        assert!(state.status.contains("Disconnected"));
    }

    #[test]
    fn test_connect_when_already_connected() {
        let mut state = CollabPanelState::default();
        state.connect();
        assert!(state.connected);
        state.connect(); // Second connect should not crash
        assert!(state.status.contains("Already connected"));
    }

    #[test]
    fn test_disconnect_when_not_connected() {
        let mut state = CollabPanelState::default();
        state.disconnect();
        assert!(state.status.contains("Not connected"));
    }

    #[test]
    fn test_send_test_operation_when_disconnected() {
        let mut state = CollabPanelState::default();
        state.send_test_operation();
        assert!(state.status.contains("Not connected"));
        assert!(state.op_log.is_empty());
    }

    #[test]
    fn test_send_test_operation_when_connected() {
        let mut state = CollabPanelState::default();
        state.connect();
        state.send_test_operation();
        assert!(state.op_log.len() == 1);
        assert!(state.status.contains("Sent test operation"));
    }

    #[test]
    fn test_clear_log() {
        let mut state = CollabPanelState::default();
        state.connect();
        state.send_test_operation();
        state.send_test_operation();
        assert_eq!(state.op_log.len(), 2);
        state.clear_log();
        assert!(state.op_log.is_empty());
    }

    #[test]
    fn test_current_time_str_format() {
        let ts = current_time_str();
        // Should match HH:MM:SS format
        assert_eq!(ts.len(), 8);
        let parts: Vec<&str> = ts.split(':').collect();
        assert_eq!(parts.len(), 3);
        assert!(parts[0].parse::<u32>().is_ok());
        assert!(parts[1].parse::<u32>().is_ok());
        assert!(parts[2].parse::<u32>().is_ok());
    }

    #[test]
    fn test_multiple_clients_join_same_session() {
        let server = Arc::new(CollabServer::new());
        let mut state1 = CollabPanelState {
            server: server.clone(),
            replica_id: "alice".to_string(),
            ..Default::default()
        };
        let mut state2 = CollabPanelState {
            server: server.clone(),
            replica_id: "bob".to_string(),
            ..Default::default()
        };

        state1.connect();
        state2.connect();

        // Both should be connected
        assert!(state1.connected);
        assert!(state2.connected);

        // Poll for presence updates
        state1.poll_messages();
        state2.poll_messages();

        // Note: presence updates may or may not have been received yet
        // depending on timing. The key assertion is that both connected.
    }

    #[test]
    fn test_duplicate_replica_id_rejected() {
        let server = Arc::new(CollabServer::new());
        let mut state1 = CollabPanelState {
            server: server.clone(),
            replica_id: "alice".to_string(),
            ..Default::default()
        };
        let mut state2 = CollabPanelState {
            server: server.clone(),
            replica_id: "alice".to_string(), // Same name
            ..Default::default()
        };

        state1.connect();
        assert!(state1.connected);

        state2.connect();
        // Should fail with "already in session" error
        assert!(!state2.connected);
        assert!(state2.status.contains("Error") || state2.status.contains("already"));
    }

    #[test]
    fn test_op_log_entry_creation() {
        let entry = OpLogEntry {
            timestamp: "12:34:56".to_string(),
            replica_id: "alice".to_string(),
            operation_desc: "AddFace #42".to_string(),
        };
        assert_eq!(entry.timestamp, "12:34:56");
        assert_eq!(entry.replica_id, "alice");
        assert_eq!(entry.operation_desc, "AddFace #42");
    }
}
