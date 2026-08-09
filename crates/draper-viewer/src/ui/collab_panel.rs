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

    // ─── Phase 8: Extended UI fields (mockup 73) ───

    /// Activity feed entries (recent collaborative actions).
    pub activity_feed: Vec<ActivityEntry>,
    /// Branch info for version control display.
    pub branch_info: BranchInfo,
    /// Storage usage: (used_bytes, total_bytes).
    pub storage_usage: (u64, u64),
    /// User color assignments for avatars (replica_id → color).
    pub user_colors: std::collections::HashMap<String, [u8; 3]>,
}

/// An entry in the activity feed.
#[derive(Clone, Debug)]
pub struct ActivityEntry {
    pub timestamp: String,
    pub user: String,
    pub action: String,
    pub target: String,
}

/// Branch information for version control display.
#[derive(Clone, Debug)]
pub struct BranchInfo {
    pub current_branch: String,
    pub commits_ahead: u32,
    pub commits_behind: u32,
    pub last_commit_msg: String,
}

impl Default for BranchInfo {
    fn default() -> Self {
        Self {
            current_branch: "main".to_string(),
            commits_ahead: 0,
            commits_behind: 0,
            last_commit_msg: "Initial commit".to_string(),
        }
    }
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
        let mut user_colors = std::collections::HashMap::new();
        user_colors.insert("alice".to_string(), [137, 180, 250]);   // Blue
        user_colors.insert("bob".to_string(), [166, 227, 161]);     // Green
        user_colors.insert("charlie".to_string(), [249, 226, 175]); // Yellow
        user_colors.insert("maria".to_string(), [203, 166, 247]);   // Mauve

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
            activity_feed: Vec::new(),
            branch_info: BranchInfo::default(),
            storage_usage: (4_200_000_000, 10_000_000_000), // 4.2 GB / 10 GB
            user_colors,
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

        // Add to activity feed
        self.add_activity(&self.replica_id.clone(), "editing", "Face #100");

        self.status = "Sent test operation (AddFace #100)".to_string();
    }

    /// Add an activity feed entry.
    pub fn add_activity(&mut self, user: &str, action: &str, target: &str) {
        let entry = ActivityEntry {
            timestamp: current_time_str(),
            user: user.to_string(),
            action: action.to_string(),
            target: target.to_string(),
        };
        self.activity_feed.push(entry);
        if self.activity_feed.len() > 30 {
            self.activity_feed.remove(0);
        }
    }

    /// Get the avatar color for a user.
    pub fn get_user_color(&self, user: &str) -> [u8; 3] {
        *self.user_colors.get(user).unwrap_or(&[180, 180, 180])
    }

    /// Get the avatar initials for a user name.
    pub fn get_initials(name: &str) -> String {
        let chars: Vec<char> = name.chars().collect();
        if chars.is_empty() {
            return "?".to_string();
        }
        if chars.len() >= 2 {
            format!("{}{}", chars[0].to_uppercase(), chars[1].to_uppercase())
        } else {
            chars[0].to_uppercase().to_string()
        }
    }

    /// Commit changes (simulated).
    pub fn commit(&mut self, message: &str) {
        self.branch_info.commits_ahead += 1;
        self.branch_info.last_commit_msg = message.to_string();
        self.add_activity(&self.replica_id.clone(), "committed", message);
        self.status = format!("Committed: '{}'", message);
    }

    /// Merge from remote (simulated).
    pub fn merge(&mut self) {
        self.branch_info.commits_behind = 0;
        self.branch_info.commits_ahead = 0;
        self.add_activity(&self.replica_id.clone(), "merged", "main");
        self.status = "Merged from main".to_string();
    }

    /// Pull from remote (simulated).
    pub fn pull(&mut self) {
        self.branch_info.commits_behind = 0;
        self.add_activity(&self.replica_id.clone(), "pulled", "main");
        self.status = "Pulled latest from main".to_string();
    }

    /// Format storage usage as human-readable string.
    pub fn storage_usage_str(&self) -> String {
        let (used, total) = self.storage_usage;
        format!("{:.1} GB / {:.1} GB", used as f64 / 1e9, total as f64 / 1e9)
    }

    /// Storage usage as a fraction (0.0 to 1.0).
    pub fn storage_fraction(&self) -> f32 {
        let (used, total) = self.storage_usage;
        if total == 0 { 0.0 } else { used as f32 / total as f32 }
    }

    /// Poll for incoming messages from the server.
    ///
    /// This should be called each frame to process any messages that the
    /// server has broadcast to us.
    pub fn poll_messages(&mut self) {
        let mut received_ops: Vec<(String, String)> = Vec::new();
        let mut received_presence: Option<Vec<String>> = None;

        if let Some(rx) = &mut self.receiver {
            // Drain all available messages without blocking
            while let Ok(msg) = rx.try_recv() {
                match &msg {
                    SyncMessage::RemoteOp { replica_id, operation, .. } => {
                        received_ops.push((replica_id.clone(), format!("{:?}", operation)));
                    }
                    SyncMessage::Presence { replicas, .. } => {
                        received_presence = Some(replicas.clone());
                    }
                    _ => {}
                }
            }
        }

        // Process received operations (outside the borrow of self.receiver)
        for (replica_id, op_desc) in received_ops {
            let entry = OpLogEntry {
                timestamp: current_time_str(),
                replica_id: replica_id.clone(),
                operation_desc: op_desc.clone(),
            };
            self.op_log.push(entry);
            if self.op_log.len() > 50 {
                self.op_log.remove(0);
            }
            // Add to activity feed
            self.add_activity(&replica_id, "modified", &op_desc);
        }

        // Update presence
        if let Some(replicas) = received_presence {
            self.peers = replicas.clone();
            self.peer_count = replicas.len();
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

    // === Peers with avatars ===
    if state.connected && !state.peers.is_empty() {
        ui.add_space(4.0);
        ui.label(egui::RichText::new(format!("Online ({}):", state.peer_count)).size(11.0).strong());
        egui::ScrollArea::vertical()
            .max_height(80.0)
            .show(ui, |ui| {
                for peer in &state.peers {
                    let is_me = peer == &state.replica_id;
                    let color = state.get_user_color(peer);
                    let initials = CollabPanelState::get_initials(peer);
                    ui.horizontal(|ui| {
                        // Avatar circle
                        let (rect, _) = ui.allocate_exact_size(
                            egui::vec2(18.0, 18.0),
                            egui::Sense::hover(),
                        );
                        ui.painter().circle_filled(
                            rect.center(),
                            8.0,
                            egui::Color32::from_rgb(color[0], color[1], color[2]),
                        );
                        ui.painter().text(
                            rect.center(),
                            egui::Align2::CENTER_CENTER,
                            &initials,
                            egui::FontId::proportional(8.0),
                            egui::Color32::BLACK,
                        );
                        // Online indicator (green dot)
                        ui.painter().circle_filled(
                            egui::pos2(rect.max.x - 2.0, rect.max.y - 2.0),
                            3.0,
                            egui::Color32::from_rgb(80, 200, 80),
                        );
                        // Name
                        let name_label = if is_me {
                            format!("{} (you)", peer)
                        } else {
                            peer.clone()
                        };
                        ui.label(egui::RichText::new(name_label).size(10.0));
                    });
                }
            });
    }

    // === Activity Feed ===
    ui.add_space(4.0);
    ui.label(egui::RichText::new("Activity:").size(11.0).strong());
    if state.activity_feed.is_empty() {
        ui.label(egui::RichText::new("(no recent activity)").size(10.0).weak());
    } else {
        egui::ScrollArea::vertical()
            .max_height(100.0)
            .show(ui, |ui| {
                for entry in state.activity_feed.iter().rev() {
                    let color = state.get_user_color(&entry.user);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(&entry.timestamp).size(9.0).weak().color(egui::Color32::from_rgb(120, 120, 120)));
                        ui.label(egui::RichText::new(&entry.user).size(9.0).color(egui::Color32::from_rgb(color[0], color[1], color[2])));
                        ui.label(egui::RichText::new(entry.action.clone()).size(9.0).color(egui::Color32::from_rgb(150, 150, 160)));
                        ui.label(egui::RichText::new(&entry.target).size(9.0));
                    });
                }
            });
    }

    // === Branch Management ===
    ui.add_space(4.0);
    ui.label(egui::RichText::new("Branch:").size(11.0).strong());
    ui.horizontal(|ui| {
        // Branch icon
        ui.label(egui::RichText::new("⎇").size(12.0).color(egui::Color32::from_rgb(0x89, 0xb4, 0xfa)));
        ui.label(egui::RichText::new(&state.branch_info.current_branch).size(10.0).strong());
        if state.branch_info.commits_ahead > 0 {
            ui.label(egui::RichText::new(format!("↑{}", state.branch_info.commits_ahead))
                .size(9.0).color(egui::Color32::from_rgb(80, 180, 80)));
        }
        if state.branch_info.commits_behind > 0 {
            ui.label(egui::RichText::new(format!("↓{}", state.branch_info.commits_behind))
                .size(9.0).color(egui::Color32::from_rgb(220, 180, 80)));
        }
    });
    ui.label(egui::RichText::new(format!("Last: {}", state.branch_info.last_commit_msg))
        .size(9.0).weak().color(egui::Color32::from_rgb(120, 120, 130)));

    // Branch action buttons
    ui.horizontal(|ui| {
        let mut commit_msg = String::new();
        // Commit button opens inline text
        if ui.button("Commit").clicked() {
            state.commit("Update model");
        }
        if ui.add_enabled(state.branch_info.commits_behind > 0, egui::Button::new("Pull")).clicked() {
            state.pull();
        }
        if ui.add_enabled(state.branch_info.commits_ahead > 0, egui::Button::new("Merge")).clicked() {
            state.merge();
        }
        let _ = commit_msg;
    });

    // === Storage Usage ===
    ui.add_space(4.0);
    ui.label(egui::RichText::new("Cloud Storage:").size(11.0).strong());
    let fraction = state.storage_fraction();
    let storage_color = if fraction > 0.9 {
        egui::Color32::from_rgb(220, 80, 80) // Red
    } else if fraction > 0.7 {
        egui::Color32::from_rgb(220, 180, 80) // Yellow
    } else {
        egui::Color32::from_rgb(80, 180, 80) // Green
    };
    ui.add(egui::ProgressBar::new(fraction)
        .fill(storage_color)
        .text(state.storage_usage_str()));

    // === Operation log ===
    ui.add_space(4.0);
    ui.label(egui::RichText::new("Operation Log:").size(11.0).strong());
    if state.op_log.is_empty() {
        ui.label(egui::RichText::new("(no operations yet)").size(10.0).weak());
    } else {
        egui::ScrollArea::vertical()
            .max_height(80.0)
            .show(ui, |ui| {
                for entry in state.op_log.iter().rev() {
                    let color = state.get_user_color(&entry.replica_id);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(&entry.timestamp).size(9.0).weak().color(egui::Color32::from_rgb(120, 120, 120)));
                        ui.label(egui::RichText::new(format!("[{}]", entry.replica_id)).size(9.0)
                            .color(egui::Color32::from_rgb(color[0], color[1], color[2])));
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

    // ─── Phase 8 tests ───

    #[test]
    fn test_activity_feed_default_empty() {
        let state = CollabPanelState::default();
        assert!(state.activity_feed.is_empty());
    }

    #[test]
    fn test_add_activity() {
        let mut state = CollabPanelState::default();
        state.add_activity("bob", "editing", "Sketch 3");
        assert_eq!(state.activity_feed.len(), 1);
        assert_eq!(state.activity_feed[0].user, "bob");
        assert_eq!(state.activity_feed[0].action, "editing");
        assert_eq!(state.activity_feed[0].target, "Sketch 3");
    }

    #[test]
    fn test_activity_feed_max_30() {
        let mut state = CollabPanelState::default();
        for i in 0..50 {
            state.add_activity("bob", "editing", &format!("Op {}", i));
        }
        assert_eq!(state.activity_feed.len(), 30);
    }

    #[test]
    fn test_user_colors() {
        let state = CollabPanelState::default();
        assert_eq!(state.get_user_color("alice"), [137, 180, 250]);
        assert_eq!(state.get_user_color("bob"), [166, 227, 161]);
        assert_eq!(state.get_user_color("unknown"), [180, 180, 180]); // Default gray
    }

    #[test]
    fn test_get_initials() {
        assert_eq!(CollabPanelState::get_initials("alice"), "AL");
        assert_eq!(CollabPanelState::get_initials("bob"), "BO");
        assert_eq!(CollabPanelState::get_initials("a"), "A");
        assert_eq!(CollabPanelState::get_initials(""), "?");
    }

    #[test]
    fn test_commit() {
        let mut state = CollabPanelState::default();
        assert_eq!(state.branch_info.commits_ahead, 0);
        state.commit("Add fillet");
        assert_eq!(state.branch_info.commits_ahead, 1);
        assert_eq!(state.branch_info.last_commit_msg, "Add fillet");
        assert!(state.status.contains("Committed"));
        assert_eq!(state.activity_feed.len(), 1);
    }

    #[test]
    fn test_merge() {
        let mut state = CollabPanelState::default();
        state.branch_info.commits_ahead = 3;
        state.branch_info.commits_behind = 1;
        state.merge();
        assert_eq!(state.branch_info.commits_ahead, 0);
        assert_eq!(state.branch_info.commits_behind, 0);
        assert!(state.status.contains("Merged"));
    }

    #[test]
    fn test_pull() {
        let mut state = CollabPanelState::default();
        state.branch_info.commits_behind = 5;
        state.pull();
        assert_eq!(state.branch_info.commits_behind, 0);
        assert!(state.status.contains("Pulled"));
    }

    #[test]
    fn test_storage_usage_str() {
        let state = CollabPanelState::default();
        let s = state.storage_usage_str();
        assert!(s.contains("4.2 GB"));
        assert!(s.contains("10.0 GB"));
    }

    #[test]
    fn test_storage_fraction() {
        let state = CollabPanelState::default();
        let frac = state.storage_fraction();
        assert!((frac - 0.42).abs() < 0.01);
    }

    #[test]
    fn test_storage_fraction_zero_total() {
        let mut state = CollabPanelState::default();
        state.storage_usage = (100, 0);
        assert_eq!(state.storage_fraction(), 0.0);
    }

    #[test]
    fn test_branch_info_default() {
        let bi = BranchInfo::default();
        assert_eq!(bi.current_branch, "main");
        assert_eq!(bi.commits_ahead, 0);
        assert_eq!(bi.commits_behind, 0);
        assert_eq!(bi.last_commit_msg, "Initial commit");
    }

    #[test]
    fn test_send_test_operation_adds_activity() {
        let mut state = CollabPanelState::default();
        state.connect();
        state.send_test_operation();
        assert!(!state.activity_feed.is_empty());
        assert_eq!(state.activity_feed[0].user, "alice");
        assert_eq!(state.activity_feed[0].action, "editing");
    }
}
