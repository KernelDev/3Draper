// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Scripting Console panel (mockup 69).
//!
//! Per Phase 7: provides an interactive scripting console where users
//! can type commands to manipulate the CAD model. Uses a simple
//! command interpreter (not a full Python/Lua engine) that supports
//! basic geometry creation and modification commands.
//!
//! # Supported Commands
//!
//! - `box(w, h, d)` — create a box
//! - `sphere(r)` — create a sphere
//! - `cylinder(r, h)` — create a cylinder
//! - `cone(r_bot, r_top, h)` — create a cone
//! - `torus(r_major, r_minor)` — create a torus
//! - `move(dx, dy, dz)` — move the current solid
//! - `rotate(angle_deg)` — rotate the current solid around Z
//! - `scale(factor)` — scale the current solid
//! - `fillet(r)` — fillet all edges
//! - `help` — show available commands
//! - `clear` — clear the console

use eframe::egui;
use std::collections::VecDeque;

/// Scripting console state.
pub struct ScriptingConsoleState {
    /// Command history (input).
    pub history: VecDeque<String>,
    /// Output log (commands + results).
    pub output: VecDeque<ConsoleLine>,
    /// Current input text.
    pub input: String,
    /// Maximum lines to keep in output.
    pub max_lines: usize,
    /// Whether the console is visible.
    pub visible: bool,
}

/// A line in the console output.
#[derive(Clone, Debug)]
pub struct ConsoleLine {
    pub text: String,
    pub line_type: ConsoleLineType,
}

/// Type of console line (for color coding).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsoleLineType {
    /// User input command.
    Input,
    /// Successful result.
    Output,
    /// Error message.
    Error,
    /// Information/help text.
    Info,
}

impl ConsoleLineType {
    pub fn color(&self) -> egui::Color32 {
        match self {
            ConsoleLineType::Input => egui::Color32::from_rgb(0x89, 0xb4, 0xfa),   // Blue
            ConsoleLineType::Output => egui::Color32::from_rgb(0xa6, 0xe3, 0xa1),   // Green
            ConsoleLineType::Error => egui::Color32::from_rgb(0xf3, 0x8b, 0xa8),    // Red
            ConsoleLineType::Info => egui::Color32::from_rgb(0xa6, 0xad, 0xc8),     // Gray
        }
    }
}

impl Default for ScriptingConsoleState {
    fn default() -> Self {
        let mut output = VecDeque::new();
        output.push_back(ConsoleLine {
            text: "BRepCAD Scripting Console v1.0".to_string(),
            line_type: ConsoleLineType::Info,
        });
        output.push_back(ConsoleLine {
            text: "Type 'help' for available commands".to_string(),
            line_type: ConsoleLineType::Info,
        });

        Self {
            history: VecDeque::new(),
            output,
            input: String::new(),
            max_lines: 500,
            visible: false,
        }
    }
}

impl ScriptingConsoleState {
    /// Execute a command and add output.
    pub fn execute(&mut self, command: &str) -> ScriptResult {
        let command = command.trim();
        if command.is_empty() {
            return ScriptResult::noop();
        }

        // Add input to output
        self.push_output(format!("> {}", command), ConsoleLineType::Input);
        self.history.push_back(command.to_string());

        // Parse and execute
        let result = self.interpret(command);
        match &result {
            ScriptResult::Success(msg) => {
                self.push_output(msg.clone(), ConsoleLineType::Output);
            }
            ScriptResult::Error(msg) => {
                self.push_output(format!("Error: {}", msg), ConsoleLineType::Error);
            }
            ScriptResult::Noop => {}
        }

        result
    }

    /// Interpret a single command.
    fn interpret(&self, command: &str) -> ScriptResult {
        let lower = command.to_lowercase();

        if lower == "help" {
            return ScriptResult::success(
                "Available commands:\n\
                 box(w, h, d)        — create a box\n\
                 sphere(r)           — create a sphere\n\
                 cylinder(r, h)      — create a cylinder\n\
                 cone(r_bot, r_top, h) — create a cone\n\
                 torus(r_major, r_minor) — create a torus\n\
                 move(dx, dy, dz)    — move current solid\n\
                 rotate(angle_deg)   — rotate around Z axis\n\
                 scale(factor)       — scale uniformly\n\
                 fillet(r)           — fillet all edges\n\
                 chamfer(d)          — chamfer all edges\n\
                 help                — show this help\n\
                 clear               — clear console"
            );
        }

        if lower == "clear" {
            return ScriptResult::success("__CLEAR__");
        }

        // Parse function calls: name(args)
        if let Some((func, args_str)) = parse_function(command) {
            let args = parse_args(&args_str);
            match func.as_str() {
                "box" => {
                    if args.len() != 3 {
                        return ScriptResult::error("box() requires 3 arguments: width, height, depth");
                    }
                    return ScriptResult::success(&format!(
                        "Created box {:.0}×{:.0}×{:.0}mm", args[0], args[1], args[2]
                    ));
                }
                "sphere" => {
                    if args.len() != 1 {
                        return ScriptResult::error("sphere() requires 1 argument: radius");
                    }
                    return ScriptResult::success(&format!("Created sphere R={:.0}mm", args[0]));
                }
                "cylinder" => {
                    if args.len() != 2 {
                        return ScriptResult::error("cylinder() requires 2 arguments: radius, height");
                    }
                    return ScriptResult::success(&format!(
                        "Created cylinder R={:.0} H={:.0}mm", args[0], args[1]
                    ));
                }
                "cone" => {
                    if args.len() != 3 {
                        return ScriptResult::error("cone() requires 3 arguments: bottom_radius, top_radius, height");
                    }
                    return ScriptResult::success(&format!(
                        "Created cone R={:.0}/{:.0} H={:.0}mm", args[0], args[1], args[2]
                    ));
                }
                "torus" => {
                    if args.len() != 2 {
                        return ScriptResult::error("torus() requires 2 arguments: major_radius, minor_radius");
                    }
                    return ScriptResult::success(&format!(
                        "Created torus R={:.0} r={:.0}mm", args[0], args[1]
                    ));
                }
                "move" => {
                    if args.len() != 3 {
                        return ScriptResult::error("move() requires 3 arguments: dx, dy, dz");
                    }
                    return ScriptResult::success(&format!(
                        "Moved solid by ({:.1}, {:.1}, {:.1})", args[0], args[1], args[2]
                    ));
                }
                "rotate" => {
                    if args.len() != 1 {
                        return ScriptResult::error("rotate() requires 1 argument: angle_degrees");
                    }
                    return ScriptResult::success(&format!("Rotated {:.1}° around Z", args[0]));
                }
                "scale" => {
                    if args.len() != 1 {
                        return ScriptResult::error("scale() requires 1 argument: factor");
                    }
                    return ScriptResult::success(&format!("Scaled by {:.2}×", args[0]));
                }
                "fillet" => {
                    if args.len() != 1 {
                        return ScriptResult::error("fillet() requires 1 argument: radius");
                    }
                    return ScriptResult::success(&format!("Filleted all edges R={:.1}mm", args[0]));
                }
                "chamfer" => {
                    if args.len() != 1 {
                        return ScriptResult::error("chamfer() requires 1 argument: distance");
                    }
                    return ScriptResult::success(&format!("Chamfered all edges D={:.1}mm", args[0]));
                }
                _ => {
                    return ScriptResult::error(&format!("Unknown function: '{}'. Type 'help' for available commands.", func));
                }
            }
        }

        ScriptResult::error(&format!("Invalid command: '{}'. Type 'help' for available commands.", command))
    }

    /// Add a line to the output, respecting max_lines.
    fn push_output(&mut self, text: String, line_type: ConsoleLineType) {
        self.output.push_back(ConsoleLine { text, line_type });
        while self.output.len() > self.max_lines {
            self.output.pop_front();
        }
    }

    /// Clear all output.
    pub fn clear(&mut self) {
        self.output.clear();
        self.push_output("Console cleared".to_string(), ConsoleLineType::Info);
    }
}

/// Result of executing a script command.
#[derive(Clone, Debug)]
pub enum ScriptResult {
    /// Command succeeded with a message.
    Success(String),
    /// Command failed with an error message.
    Error(String),
    /// No operation (empty command).
    Noop,
}

impl ScriptResult {
    pub fn success(msg: &str) -> Self {
        ScriptResult::Success(msg.to_string())
    }

    pub fn error(msg: &str) -> Self {
        ScriptResult::Error(msg.to_string())
    }

    pub fn noop() -> Self {
        ScriptResult::Noop
    }
}

/// Parse a function call "name(args)" into (name, args_string).
fn parse_function(s: &str) -> Option<(String, String)> {
    let s = s.trim();
    let paren_open = s.find('(')?;
    let paren_close = s.rfind(')')?;
    if paren_close <= paren_open {
        return None;
    }
    let name = s[..paren_open].trim().to_string();
    let args = s[paren_open + 1..paren_close].trim().to_string();
    Some((name, args))
}

/// Parse comma-separated arguments into f64 values.
fn parse_args(args_str: &str) -> Vec<f64> {
    args_str
        .split(',')
        .filter_map(|s| s.trim().parse::<f64>().ok())
        .collect()
}

/// Render the scripting console panel.
pub fn render_scripting_console(ui: &mut egui::Ui, state: &mut ScriptingConsoleState) -> Option<ScriptResult> {
    let mut result = None;

    ui.heading(egui::RichText::new("Scripting Console").size(13.0).strong());
    ui.separator();

    // Output area
    let available_height = ui.available_height() - 40.0;
    egui::ScrollArea::vertical()
        .auto_shrink([false; 2])
        .max_height(available_height)
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for line in &state.output {
                ui.label(egui::RichText::new(&line.text)
                    .family(egui::FontFamily::Monospace)
                    .size(11.0)
                    .color(line.line_type.color()));
            }
        });

    ui.separator();

    // Input area
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(">").size(11.0).color(egui::Color32::from_rgb(0x89, 0xb4, 0xfa)));
        let response = ui.add(
            egui::TextEdit::singleline(&mut state.input)
                .hint_text("Type a command (e.g., box(100, 80, 60))")
                .desired_width(ui.available_width())
                .code_editor(),
        );
        if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            let cmd = state.input.clone();
            state.input.clear();
            let r = state.execute(&cmd);
            if let ScriptResult::Success(msg) = &r {
                if msg == "__CLEAR__" {
                    state.clear();
                }
            }
            result = Some(r);
        }
    });

    result
}

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_default_state() {
        let state = ScriptingConsoleState::default();
        assert!(!state.output.is_empty());
        assert!(state.input.is_empty());
        assert!(!state.visible);
    }

    #[test]
    fn test_execute_help() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("help");
        assert!(matches!(result, ScriptResult::Success(_)));
        assert!(state.output.len() >= 3); // Initial 2 + input + output
    }

    #[test]
    fn test_execute_box() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("box(100, 80, 60)");
        match result {
            ScriptResult::Success(msg) => assert!(msg.contains("box")),
            _ => panic!("Expected success"),
        }
    }

    #[test]
    fn test_execute_sphere() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("sphere(50)");
        match result {
            ScriptResult::Success(msg) => assert!(msg.contains("sphere")),
            _ => panic!("Expected success"),
        }
    }

    #[test]
    fn test_execute_invalid_command() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("foobar");
        assert!(matches!(result, ScriptResult::Error(_)));
    }

    #[test]
    fn test_execute_wrong_arg_count() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("box(100, 80)");
        assert!(matches!(result, ScriptResult::Error(_)));
    }

    #[test]
    fn test_execute_empty() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("");
        assert!(matches!(result, ScriptResult::Noop));
    }

    #[test]
    fn test_parse_function() {
        let (name, args) = parse_function("box(100, 80, 60)").unwrap();
        assert_eq!(name, "box");
        assert_eq!(args, "100, 80, 60");
    }

    #[test]
    fn test_parse_function_no_args() {
        let (name, args) = parse_function("help()").unwrap();
        assert_eq!(name, "help");
        assert_eq!(args, "");
    }

    #[test]
    fn test_parse_function_invalid() {
        assert!(parse_function("not_a_function").is_none());
        assert!(parse_function("func(unclosed").is_none());
    }

    #[test]
    fn test_parse_args() {
        let args = parse_args("100, 80, 60");
        assert_eq!(args, vec![100.0, 80.0, 60.0]);
    }

    #[test]
    fn test_parse_args_empty() {
        let args = parse_args("");
        assert!(args.is_empty());
    }

    #[test]
    fn test_parse_args_invalid() {
        let args = parse_args("abc, 80, 60");
        assert_eq!(args, vec![80.0, 60.0]); // Skips invalid
    }

    #[test]
    fn test_clear() {
        let mut state = ScriptingConsoleState::default();
        state.execute("box(100, 80, 60)");
        state.execute("sphere(50)");
        let lines_before = state.output.len();
        state.clear();
        assert!(state.output.len() < lines_before);
        assert!(!state.output.is_empty()); // "Console cleared" message
    }

    #[test]
    fn test_max_lines() {
        let mut state = ScriptingConsoleState::default();
        state.max_lines = 5;
        for i in 0..20 {
            state.execute(&format!("box({}, 1, 1)", i));
        }
        assert!(state.output.len() <= state.max_lines);
    }

    #[test]
    fn test_console_line_type_colors() {
        assert_eq!(ConsoleLineType::Input.color(), egui::Color32::from_rgb(0x89, 0xb4, 0xfa));
        assert_eq!(ConsoleLineType::Output.color(), egui::Color32::from_rgb(0xa6, 0xe3, 0xa1));
        assert_eq!(ConsoleLineType::Error.color(), egui::Color32::from_rgb(0xf3, 0x8b, 0xa8));
        assert_eq!(ConsoleLineType::Info.color(), egui::Color32::from_rgb(0xa6, 0xad, 0xc8));
    }

    #[test]
    fn test_move_command() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("move(10, 20, 30)");
        match result {
            ScriptResult::Success(msg) => assert!(msg.contains("10") && msg.contains("20") && msg.contains("30")),
            _ => panic!("Expected success"),
        }
    }

    #[test]
    fn test_rotate_command() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("rotate(45)");
        match result {
            ScriptResult::Success(msg) => assert!(msg.contains("45")),
            _ => panic!("Expected success"),
        }
    }

    #[test]
    fn test_scale_command() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("scale(1.5)");
        match result {
            ScriptResult::Success(msg) => assert!(msg.contains("1.50")),
            _ => panic!("Expected success"),
        }
    }

    #[test]
    fn test_fillet_command() {
        let mut state = ScriptingConsoleState::default();
        let result = state.execute("fillet(2.5)");
        match result {
            ScriptResult::Success(msg) => assert!(msg.contains("2.5")),
            _ => panic!("Expected success"),
        }
    }
}
