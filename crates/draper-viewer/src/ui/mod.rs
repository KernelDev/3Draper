// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! UI module — modular UI components for BRepCAD.
//!
//! Roadmap UI Phase 0-2: Application shell, menu bar, ribbon tabs.

pub mod menubar;
pub mod ribbon;
pub mod statusbar;
pub mod panels;
pub mod context_menus;
pub mod command_palette;
pub mod view_modes;
pub mod dialogs;
pub mod core_engine;
pub mod sketch;
pub mod workspaces;
pub mod dispatcher;
pub mod ai_panel;
pub mod collab_panel;
pub mod dock;

use eframe::egui;

/// The active workspace/mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Workspace {
    Modeling,
    Sketch,
    VisualProgramming,
    Surface,
    SheetMetal,
    Assembly,
    Cam,
    Drawing,
    Simulation,
    Inspect,
    Ai,
}

impl Default for Workspace {
    fn default() -> Self { Workspace::Modeling }
}

/// Display style for the viewport.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DisplayStyle {
    Wireframe,
    Shaded,
    ShadedWithEdges,
}

impl Default for DisplayStyle {
    fn default() -> Self { DisplayStyle::ShadedWithEdges }
}

/// Global UI state shared across all components.
#[derive(Clone, Debug)]
pub struct UiState {
    /// Active workspace.
    pub workspace: Workspace,
    /// Active display style.
    pub display_style: DisplayStyle,
    /// Currently selected ribbon tab.
    pub active_ribbon: ribbon::RibbonTab,
    /// Mouse coordinates in 3D world space (from viewport).
    pub mouse_world: [f64; 3],
    /// Camera azimuth, elevation, distance.
    pub camera_info: [f32; 3],
    /// Active tool name.
    pub active_tool: String,
    /// FPS counter.
    pub fps: f32,
    /// Units (mm, cm, m, inch).
    pub units: String,
    /// Selected entity count.
    pub selection_count: usize,
    /// View orientation.
    pub view_orientation: String,
    /// Command palette state.
    pub command_palette: command_palette::CommandPalette,
    /// Marking menu visibility.
    pub marking_menu_visible: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            workspace: Workspace::Modeling,
            display_style: DisplayStyle::ShadedWithEdges,
            active_ribbon: ribbon::RibbonTab::Home,
            mouse_world: [0.0, 0.0, 0.0],
            camera_info: [35.0, 25.0, 480.0],
            active_tool: "Select".to_string(),
            fps: 60.0,
            units: "mm".to_string(),
            selection_count: 0,
            view_orientation: "ISO".to_string(),
            command_palette: Default::default(),
            marking_menu_visible: false,
        }
    }
}
