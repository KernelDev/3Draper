// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Docking system using egui-dock.

use egui_dock::{DockArea, DockState, Style, TabViewer, SurfaceIndex, NodeIndex};
use egui::Color32;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum DockTab {
    Viewport,
    Browser,
    Properties,
    Timeline,
    Log,
    VpCanvas,
}

pub struct DockStateHolder {
    pub state: DockState<DockTab>,
}

impl DockStateHolder {
    pub fn new() -> Self {
        let mut state = DockState::new(vec![DockTab::Viewport]);

        let [_left, _right] = state.split(
            (SurfaceIndex::main(), NodeIndex::root()),
            egui_dock::Split::Right,
            0.25,
            egui_dock::Node::leaf_with(vec![DockTab::Properties, DockTab::Timeline]),
        );

        let [_browser, _viewport] = state.split(
            (SurfaceIndex::main(), NodeIndex::root()),
            egui_dock::Split::Left,
            0.2,
            egui_dock::Node::leaf_with(vec![DockTab::Browser, DockTab::Log]),
        );

        Self { state }
    }

    pub fn show(&mut self, ctx: &egui::Context, tab_viewer: &mut impl TabViewer<Tab = DockTab>) {
        let mut style = Style::from_egui(ctx.style().as_ref());

        // Dark Catppuccin Mocha theme for dock
        style.dock_area_padding = Some(egui_dock::egui::Margin::same(2));
        style.tab_bar.bg_fill = Color32::from_rgb(0x11, 0x11, 0x1b);
        style.tab_bar.height = 26.0;

        // Tab colors
        let dark_bg = Color32::from_rgb(0x18, 0x18, 0x25);
        let active_bg = Color32::from_rgb(0x1e, 0x1e, 0x2e);
        let text_color = Color32::from_rgb(0xcd, 0xd6, 0xf4);
        let text_dim = Color32::from_rgb(0x6c, 0x70, 0x86);
        let accent = Color32::from_rgb(0x89, 0xb4, 0xfa);

        style.tab.active.bg_fill = active_bg;
        style.tab.active.text_color = text_color;
        style.tab.active.outline_color = accent;
        style.tab.inactive.bg_fill = dark_bg;
        style.tab.inactive.text_color = text_dim;
        style.tab.focused.bg_fill = active_bg;
        style.tab.focused.text_color = text_color;
        style.tab.focused.outline_color = accent;
        style.tab.hovered.bg_fill = Color32::from_rgb(0x31, 0x32, 0x44);
        style.tab.hovered.text_color = text_color;

        style.separator.width = 2.0;
        style.separator.color_idle = Color32::from_rgb(0x31, 0x32, 0x44);
        style.separator.color_hovered = Color32::from_rgb(0x45, 0x47, 0x5a);
        style.separator.color_dragged = accent;

        DockArea::new(&mut self.state)
            .style(style)
            .show(ctx, tab_viewer);
    }
}

impl Default for DockStateHolder {
    fn default() -> Self {
        Self::new()
    }
}
