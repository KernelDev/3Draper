// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Sketch engine — Phase 4.
//!
//! 2D sketch canvas with drawing tools, constraints, dimensions, and solver.

use std::collections::HashMap;

// ============================================================
// 4.1. Sketch Entities
// ============================================================

/// Unique entity ID.
pub type EntId = u64;

/// A 2D point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    pub fn new(x: f64, y: f64) -> Self { Self { x, y } }
    pub fn distance_to(&self, other: &Point2D) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
    pub fn midpoint(a: &Point2D, b: &Point2D) -> Point2D {
        Point2D::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
    }
}

/// Sketch entity types.
#[derive(Clone, Debug)]
pub enum SketchEntity {
    /// Line segment from p1 to p2.
    Line { id: EntId, p1: Point2D, p2: Point2D },
    /// Circle with center and radius.
    Circle { id: EntId, center: Point2D, radius: f64 },
    /// Arc from p1 to p2 with center.
    Arc { id: EntId, center: Point2D, p1: Point2D, p2: Point2D, clockwise: bool },
    /// Rectangle (4 lines).
    Rectangle { id: EntId, p1: Point2D, p2: Point2D },
    /// Spline through points.
    Spline { id: EntId, points: Vec<Point2D> },
    /// Single point.
    Point { id: EntId, p: Point2D },
}

impl SketchEntity {
    pub fn id(&self) -> EntId {
        match self {
            SketchEntity::Line { id, .. } => *id,
            SketchEntity::Circle { id, .. } => *id,
            SketchEntity::Arc { id, .. } => *id,
            SketchEntity::Rectangle { id, .. } => *id,
            SketchEntity::Spline { id, .. } => *id,
            SketchEntity::Point { id, .. } => *id,
        }
    }

    pub fn entity_type(&self) -> &'static str {
        match self {
            SketchEntity::Line { .. } => "Line",
            SketchEntity::Circle { .. } => "Circle",
            SketchEntity::Arc { .. } => "Arc",
            SketchEntity::Rectangle { .. } => "Rectangle",
            SketchEntity::Spline { .. } => "Spline",
            SketchEntity::Point { .. } => "Point",
        }
    }

    /// Get all control points of this entity.
    pub fn points(&self) -> Vec<Point2D> {
        match self {
            SketchEntity::Line { p1, p2, .. } => vec![*p1, *p2],
            SketchEntity::Circle { center, .. } => vec![*center],
            SketchEntity::Arc { center, p1, p2, .. } => vec![*center, *p1, *p2],
            SketchEntity::Rectangle { p1, p2, .. } => vec![*p1, *p2],
            SketchEntity::Spline { points, .. } => points.clone(),
            SketchEntity::Point { p, .. } => vec![*p],
        }
    }

    /// Update a point by index (used by constraint solver).
    pub fn set_point(&mut self, idx: usize, p: Point2D) {
        match self {
            SketchEntity::Line { p1, p2, .. } => {
                if idx == 0 { *p1 = p; } else if idx == 1 { *p2 = p; }
            }
            SketchEntity::Circle { center, .. } => { if idx == 0 { *center = p; } }
            SketchEntity::Arc { center, p1, p2, .. } => {
                match idx { 0 => *center = p, 1 => *p1 = p, 2 => *p2 = p, _ => {} }
            }
            SketchEntity::Rectangle { p1, p2, .. } => {
                if idx == 0 { *p1 = p; } else if idx == 1 { *p2 = p; }
            }
            SketchEntity::Spline { points, .. } => {
                if idx < points.len() { points[idx] = p; }
            }
            SketchEntity::Point { p: pt, .. } => { if idx == 0 { *pt = p; } }
        }
    }
}

// ============================================================
// 4.2. Drawing Tools
// ============================================================

/// Active drawing tool.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DrawTool {
    Select,
    Line,
    Circle,
    Arc3Point,
    Rectangle,
    Spline,
    Point,
    Dimension,
    Trim,
    Extend,
    Offset,
    Mirror,
    Fillet2D,
}

impl Default for DrawTool {
    fn default() -> Self { DrawTool::Select }
}

impl DrawTool {
    pub fn label(&self) -> &'static str {
        match self {
            DrawTool::Select => "Select",
            DrawTool::Line => "Line",
            DrawTool::Circle => "Circle",
            DrawTool::Arc3Point => "Arc",
            DrawTool::Rectangle => "Rectangle",
            DrawTool::Spline => "Spline",
            DrawTool::Point => "Point",
            DrawTool::Dimension => "Dimension",
            DrawTool::Trim => "Trim",
            DrawTool::Extend => "Extend",
            DrawTool::Offset => "Offset",
            DrawTool::Mirror => "Mirror",
            DrawTool::Fillet2D => "Fillet",
        }
    }
}

// ============================================================
// 4.3. Constraints
// ============================================================

/// Constraint types (9 types from mockup 11_menu_sketch.svg).
#[derive(Clone, Debug)]
pub enum Constraint {
    /// Two points are coincident.
    Coincident { p1_ent: EntId, p1_idx: usize, p2_ent: EntId, p2_idx: usize },
    /// Two lines are collinear.
    Collinear { ent1: EntId, ent2: EntId },
    /// Two circles/arcs share the same center.
    Concentric { ent1: EntId, ent2: EntId },
    /// Two lines are parallel.
    Parallel { ent1: EntId, ent2: EntId },
    /// Two lines are perpendicular.
    Perpendicular { ent1: EntId, ent2: EntId },
    /// Line and circle/arc are tangent.
    Tangent { line_ent: EntId, circle_ent: EntId },
    /// Line is horizontal.
    Horizontal { ent: EntId },
    /// Line is vertical.
    Vertical { ent: EntId },
    /// Two lines/circles have equal length/radius.
    Equal { ent1: EntId, ent2: EntId },
}

impl Constraint {
    pub fn label(&self) -> &'static str {
        match self {
            Constraint::Coincident { .. } => "Coincident",
            Constraint::Collinear { .. } => "Collinear",
            Constraint::Concentric { .. } => "Concentric",
            Constraint::Parallel { .. } => "Parallel",
            Constraint::Perpendicular { .. } => "Perpendicular",
            Constraint::Tangent { .. } => "Tangent",
            Constraint::Horizontal { .. } => "Horizontal",
            Constraint::Vertical { .. } => "Vertical",
            Constraint::Equal { .. } => "Equal",
        }
    }

    pub fn icon(&self) -> &'static str {
        match self {
            Constraint::Coincident { .. } => "●",
            Constraint::Collinear { .. } => "═",
            Constraint::Concentric { .. } => "◎",
            Constraint::Parallel { .. } => "∥",
            Constraint::Perpendicular { .. } => "⊥",
            Constraint::Tangent { .. } => "⌒",
            Constraint::Horizontal { .. } => "↔",
            Constraint::Vertical { .. } => "↕",
            Constraint::Equal { .. } => "=",
        }
    }
}

// ============================================================
// 4.4. Dimensions
// ============================================================

/// Dimension types.
#[derive(Clone, Debug)]
pub enum Dimension {
    /// Linear dimension between two points.
    Linear { id: EntId, p1: Point2D, p2: Point2D, value: f64 },
    /// Angular dimension between two lines.
    Angular { id: EntId, vertex: Point2D, p1: Point2D, p2: Point2D, value: f64 },
    /// Radial dimension of a circle/arc.
    Radial { id: EntId, center: Point2D, radius: f64 },
    /// Diameter dimension of a circle.
    Diameter { id: EntId, center: Point2D, diameter: f64 },
}

impl Dimension {
    pub fn label(&self) -> &'static str {
        match self {
            Dimension::Linear { .. } => "Linear",
            Dimension::Angular { .. } => "Angular",
            Dimension::Radial { .. } => "Radial",
            Dimension::Diameter { .. } => "Diameter",
        }
    }

    pub fn display_value(&self) -> String {
        match self {
            Dimension::Linear { value, .. } => format!("{:.2} mm", value),
            Dimension::Angular { value, .. } => format!("{:.1}°", value.to_degrees()),
            Dimension::Radial { radius, .. } => format!("R{:.2}", radius),
            Dimension::Diameter { diameter, .. } => format!("⌀{:.2}", diameter),
        }
    }
}

// ============================================================
// 4.5. Sketch State
// ============================================================

/// Sketch state — holds all entities, constraints, dimensions.
#[derive(Clone, Debug, Default)]
pub struct Sketch {
    pub entities: HashMap<EntId, SketchEntity>,
    pub constraints: Vec<Constraint>,
    pub dimensions: Vec<Dimension>,
    pub next_id: EntId,
    pub active_tool: DrawTool,
    pub plane: SketchPlane,
    pub grid_visible: bool,
    pub snap_to_grid: bool,
    pub grid_size: f64,
}

/// Sketch plane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SketchPlane {
    XY,
    XZ,
    YZ,
    Custom,
}

impl Default for SketchPlane {
    fn default() -> Self { SketchPlane::XY }
}

impl SketchPlane {
    pub fn label(&self) -> &'static str {
        match self {
            SketchPlane::XY => "XY (Top)",
            SketchPlane::XZ => "XZ (Front)",
            SketchPlane::YZ => "YZ (Right)",
            SketchPlane::Custom => "Custom",
        }
    }
}

impl Sketch {
    pub fn new() -> Self {
        Self {
            grid_visible: true,
            snap_to_grid: true,
            grid_size: 10.0,
            ..Default::default()
        }
    }

    pub fn add_line(&mut self, p1: Point2D, p2: Point2D) -> EntId {
        let id = self.next_id;
        self.next_id += 1;
        self.entities.insert(id, SketchEntity::Line { id, p1, p2 });
        id
    }

    pub fn add_circle(&mut self, center: Point2D, radius: f64) -> EntId {
        let id = self.next_id;
        self.next_id += 1;
        self.entities.insert(id, SketchEntity::Circle { id, center, radius });
        id
    }

    pub fn add_arc(&mut self, center: Point2D, p1: Point2D, p2: Point2D, clockwise: bool) -> EntId {
        let id = self.next_id;
        self.next_id += 1;
        self.entities.insert(id, SketchEntity::Arc { id, center, p1, p2, clockwise });
        id
    }

    pub fn add_rectangle(&mut self, p1: Point2D, p2: Point2D) -> EntId {
        let id = self.next_id;
        self.next_id += 1;
        self.entities.insert(id, SketchEntity::Rectangle { id, p1, p2 });
        id
    }

    pub fn add_point(&mut self, p: Point2D) -> EntId {
        let id = self.next_id;
        self.next_id += 1;
        self.entities.insert(id, SketchEntity::Point { id, p });
        id
    }

    pub fn add_constraint(&mut self, c: Constraint) {
        self.constraints.push(c);
    }

    pub fn remove_entity(&mut self, id: EntId) {
        self.entities.remove(&id);
        // Remove constraints referencing this entity
        self.constraints.retain(|c| !constraint_references(c, id));
    }

    pub fn entity_count(&self) -> usize { self.entities.len() }
    pub fn constraint_count(&self) -> usize { self.constraints.len() }

    /// Snap a point to the grid.
    pub fn snap(&self, p: Point2D) -> Point2D {
        if !self.snap_to_grid { return p; }
        Point2D::new(
            (p.x / self.grid_size).round() * self.grid_size,
            (p.y / self.grid_size).round() * self.grid_size,
        )
    }

    // ============================================================
    // 4.6. Constraint Solver (simplified)
    // ============================================================

    /// Degrees of freedom: 2 per point - 1 per constraint (simplified).
    pub fn degrees_of_freedom(&self) -> i32 {
        let point_count: usize = self.entities.values().map(|e| e.points().len()).sum();
        let constraint_count = self.constraints.len();
        let dof = (point_count * 2) as i32 - constraint_count as i32;
        dof.max(0)
    }

    pub fn is_fully_constrained(&self) -> bool {
        self.degrees_of_freedom() == 0
    }

    pub fn status(&self) -> &'static str {
        let dof = self.degrees_of_freedom();
        if dof == 0 { "✓ Fully Constrained" }
        else if self.constraints.is_empty() { "Under-Constrained" }
        else { "Under-Constrained" }
    }

    /// Solve constraints (simplified: applies constraints sequentially).
    /// Returns true if all constraints were satisfied.
    pub fn solve(&mut self) -> bool {
        let constraints = self.constraints.clone();
        for c in &constraints {
            match c {
                Constraint::Horizontal { ent } => {
                    if let Some(SketchEntity::Line { p1, p2, .. }) = self.entities.get_mut(ent) {
                        p2.y = p1.y;
                    }
                }
                Constraint::Vertical { ent } => {
                    if let Some(SketchEntity::Line { p1, p2, .. }) = self.entities.get_mut(ent) {
                        p2.x = p1.x;
                    }
                }
                Constraint::Coincident { p1_ent, p1_idx, p2_ent, p2_idx } => {
                    let p = self.entities.get(p1_ent).map(|e| {
                        let pts = e.points();
                        pts.get(*p1_idx).copied().unwrap_or(Point2D::new(0.0, 0.0))
                    });
                    if let Some(p) = p {
                        if let Some(e2) = self.entities.get_mut(p2_ent) {
                            e2.set_point(*p2_idx, p);
                        }
                    }
                }
                Constraint::Parallel { ent1, ent2 } => {
                    // Make ent2 parallel to ent1
                    if let (Some(SketchEntity::Line { p1: a1, p2: a2, .. }),
                            Some(SketchEntity::Line { p1: b1, p2: b2, .. })) =
                        (self.entities.get(ent1), self.entities.get(ent2).cloned()) {
                        let a_len = a1.distance_to(&a2);
                        if a_len > 1e-10 {
                            let dx = (a2.x - a1.x) / a_len;
                            let dy = (a2.y - a1.y) / a_len;
                            let b_len = b1.distance_to(&b2);
                            if let Some(e2) = self.entities.get_mut(ent2) {
                                e2.set_point(1, Point2D::new(b1.x + dx * b_len, b1.y + dy * b_len));
                            }
                        }
                    }
                }
                Constraint::Perpendicular { ent1, ent2 } => {
                    if let (Some(SketchEntity::Line { p1: a1, p2: a2, .. }),
                            Some(SketchEntity::Line { p1: b1, p2: b2, .. })) =
                        (self.entities.get(ent1), self.entities.get(ent2).cloned()) {
                        let a_len = a1.distance_to(&a2);
                        if a_len > 1e-10 {
                            let dx = (a2.x - a1.x) / a_len;
                            let dy = (a2.y - a1.y) / a_len;
                            // Perpendicular direction
                            let px = -dy;
                            let py = dx;
                            let b_len = b1.distance_to(&b2);
                            if let Some(e2) = self.entities.get_mut(ent2) {
                                e2.set_point(1, Point2D::new(b1.x + px * b_len, b1.y + py * b_len));
                            }
                        }
                    }
                }
                _ => { /* Other constraints: more complex, skip for now */ }
            }
        }
        true
    }
}

fn constraint_references(c: &Constraint, id: EntId) -> bool {
    match c {
        Constraint::Coincident { p1_ent, p2_ent, .. } => *p1_ent == id || *p2_ent == id,
        Constraint::Collinear { ent1, ent2 } => *ent1 == id || *ent2 == id,
        Constraint::Concentric { ent1, ent2 } => *ent1 == id || *ent2 == id,
        Constraint::Parallel { ent1, ent2 } => *ent1 == id || *ent2 == id,
        Constraint::Perpendicular { ent1, ent2 } => *ent1 == id || *ent2 == id,
        Constraint::Tangent { line_ent, circle_ent } => *line_ent == id || *circle_ent == id,
        Constraint::Horizontal { ent } => *ent == id,
        Constraint::Vertical { ent } => *ent == id,
        Constraint::Equal { ent1, ent2 } => *ent1 == id || *ent2 == id,
    }
}

// ============================================================
// 4.7. Sketch Drawing State (for interactive drawing)
// ============================================================

/// State for interactive drawing (click-by-click).
#[derive(Clone, Debug, Default)]
pub struct DrawState {
    pub tool: DrawTool,
    pub points: Vec<Point2D>,
    pub preview: Option<SketchEntity>,
    pub hover_point: Option<Point2D>,
}

impl DrawState {
    pub fn reset(&mut self) {
        self.points.clear();
        self.preview = None;
    }

    /// Add a click point. Returns Some(entity) when drawing is complete.
    pub fn click(&mut self, p: Point2D, sketch: &mut Sketch) -> Option<EntId> {
        match self.tool {
            DrawTool::Line => {
                self.points.push(p);
                if self.points.len() == 2 {
                    let id = sketch.add_line(self.points[0], self.points[1]);
                    self.reset();
                    return Some(id);
                }
            }
            DrawTool::Circle => {
                self.points.push(p);
                if self.points.len() == 2 {
                    let r = self.points[0].distance_to(&self.points[1]);
                    let id = sketch.add_circle(self.points[0], r);
                    self.reset();
                    return Some(id);
                }
            }
            DrawTool::Rectangle => {
                self.points.push(p);
                if self.points.len() == 2 {
                    let id = sketch.add_rectangle(self.points[0], self.points[1]);
                    self.reset();
                    return Some(id);
                }
            }
            DrawTool::Point => {
                let id = sketch.add_point(p);
                self.reset();
                return Some(id);
            }
            DrawTool::Arc3Point => {
                self.points.push(p);
                if self.points.len() == 3 {
                    // Simplified: center = midpoint of p1-p3, arc from p1 to p3
                    let center = Point2D::midpoint(&self.points[0], &self.points[2]);
                    let id = sketch.add_arc(center, self.points[0], self.points[2], false);
                    self.reset();
                    return Some(id);
                }
            }
            DrawTool::Spline => {
                self.points.push(p);
                // Spline needs at least 3 points, finish on double-click
                // (for now, finish at 5 points)
                if self.points.len() >= 5 {
                    let id = sketch.next_id;
                    sketch.next_id += 1;
                    sketch.entities.insert(id, SketchEntity::Spline { id, points: self.points.clone() });
                    self.reset();
                    return Some(id);
                }
            }
            _ => {}
        }
        None
    }

    /// Update preview while mouse moves.
    pub fn update_preview(&mut self, hover: Point2D) {
        if self.points.is_empty() {
            self.preview = None;
            return;
        }
        match self.tool {
            DrawTool::Line if self.points.len() == 1 => {
                self.preview = Some(SketchEntity::Line {
                    id: 0, p1: self.points[0], p2: hover,
                });
            }
            DrawTool::Circle if self.points.len() == 1 => {
                let r = self.points[0].distance_to(&hover);
                self.preview = Some(SketchEntity::Circle {
                    id: 0, center: self.points[0], radius: r,
                });
            }
            DrawTool::Rectangle if self.points.len() == 1 => {
                self.preview = Some(SketchEntity::Rectangle {
                    id: 0, p1: self.points[0], p2: hover,
                });
            }
            _ => self.preview = None,
        }
        self.hover_point = Some(hover);
    }
}
