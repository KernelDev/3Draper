// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Kinematic drag for interactive assembly editing.
//!
//! Per FLEXIBLE_EXECUTION_PLAN task C3: when the user drags a component
//! with the mouse, the assembly solver is re-invoked with the dragged
//! component's position "pinned" to the cursor. Other components move
//! to satisfy existing constraints, and collisions are reported.

use crate::{Assembly, AssemblyConstraint, AssemblyError, AssemblySolver, bvh};
use draper_geometry::Point3d;

/// Result of a kinematic drag step.
#[derive(Debug, Clone)]
pub enum DragResult {
    /// Drag succeeded — component moved to the new position, and all
    /// constraints remain satisfied.
    Ok {
        new_position: [f64; 3],
        iterations: usize,
        residual: f64,
        collisions: Vec<(usize, usize)>,
    },
    /// Drag would cause a collision — component stays at its previous position.
    CollisionDetected { a: usize, b: usize },
    /// Solver failed to converge — drag is rejected.
    Failed(AssemblyError),
}

/// A kinematic drag controller.
///
/// Created when the user starts dragging a component, updated on each
/// frame, and dropped when the user releases the mouse button.
pub struct KinematicDrag {
    /// The component being dragged.
    pub dragged_component: usize,
    /// The component's local-space point that was picked (anchor point).
    pub anchor_local: Point3d,
    /// Previous world position of the anchor (for rollback on collision).
    previous_anchor_world: Point3d,
    /// Internal solver instance (reused across frames for warm-starting).
    solver: AssemblySolver,
    /// Whether collisions should block the drag.
    pub collision_blocking: bool,
}

impl KinematicDrag {
    /// Begin dragging a component.
    pub fn new(dragged_component: usize, anchor_local: Point3d) -> Self {
        Self {
            dragged_component,
            anchor_local,
            previous_anchor_world: anchor_local,
            solver: AssemblySolver::new(),
            collision_blocking: true,
        }
    }

    /// Update the drag with a new target world position for the anchor point.
    ///
    /// This modifies the assembly in place: the dragged component is
    /// translated so that its anchor point is at `target_world`, and
    /// other free components are re-solved to satisfy existing constraints.
    ///
    /// If `collision_blocking` is true and the new position causes a
    /// collision, the assembly is rolled back to its previous state.
    pub fn update(
        &mut self,
        assembly: &mut Assembly,
        target_world: Point3d,
    ) -> DragResult {
        let snapshot = assembly.clone();

        let comp = match assembly.get_component(self.dragged_component) {
            Some(c) => c,
            None => {
                return DragResult::Failed(AssemblyError::ComponentNotFound(self.dragged_component));
            }
        };

        let current_anchor_world = comp.transform.transform_point(&self.anchor_local);
        let dx = target_world.x - current_anchor_world.x;
        let dy = target_world.y - current_anchor_world.y;
        let dz = target_world.z - current_anchor_world.z;

        let (tx, ty, tz) = comp.translation();
        if let Some(c) = assembly.get_component_mut(self.dragged_component) {
            c.set_translation(tx + dx, ty + dy, tz + dz);
            c.fixed = true;
        }

        let solve_result = self.solver.solve(assembly);

        if let Some(c) = assembly.get_component_mut(self.dragged_component) {
            c.fixed = false;
        }

        match solve_result {
            Ok(()) => {
                if self.collision_blocking {
                    let collisions = bvh::detect_collisions(&assembly.components);
                    if let Some(&(a, b)) = collisions.first() {
                        *assembly = snapshot;
                        return DragResult::CollisionDetected { a, b };
                    }
                }

                let comp = assembly.get_component(self.dragged_component).unwrap();
                let new_pos = comp.translation();
                self.previous_anchor_world = comp.transform.transform_point(&self.anchor_local);
                let collisions = bvh::detect_collisions(&assembly.components);
                DragResult::Ok {
                    new_position: [new_pos.0, new_pos.1, new_pos.2],
                    iterations: self.solver.max_iterations,
                    residual: 0.0,
                    collisions,
                }
            }
            Err(e) => {
                *assembly = snapshot;
                DragResult::Failed(e)
            }
        }
    }

    /// End the drag — runs a final solve.
    pub fn finish(self, assembly: &mut Assembly) -> Result<(), AssemblyError> {
        let mut solver = self.solver;
        solver.solve(assembly)
    }
}

/// Run a "drag" simulation by repeatedly calling `update` with a series
/// of target positions along a path.
pub fn simulate_drag_path(
    assembly: &mut Assembly,
    component: usize,
    anchor_local: Point3d,
    path: &[Point3d],
) -> (usize, usize) {
    let mut drag = KinematicDrag::new(component, anchor_local);
    let mut successes = 0;
    let mut collisions = 0;

    for target in path {
        match drag.update(assembly, *target) {
            DragResult::Ok { .. } => successes += 1,
            DragResult::CollisionDetected { .. } => collisions += 1,
            DragResult::Failed(_) => break,
        }
    }

    let _ = drag.finish(assembly);
    (successes, collisions)
}

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::{Component, bvh::BoundingBox};

    #[test]
    fn test_drag_simple_translation() {
        let mut asm = Assembly::new();
        asm.add_component(Component::new_fixed(0, "Base"));
        asm.add_component(Component::new(1, "Slider"));

        asm.add_constraint(AssemblyConstraint::Coincident {
            component_a: 0,
            point_a: Point3d::new(0.0, 0.0, 0.0),
            component_b: 1,
            point_b: Point3d::new(0.0, 0.0, 0.0),
        });

        let mut drag = KinematicDrag::new(1, Point3d::new(0.0, 0.0, 0.0));
        let result = drag.update(&mut asm, Point3d::new(5.0, 0.0, 0.0));
        match result {
            DragResult::Ok { new_position, .. } => {
                assert!((new_position[0] - 5.0).abs() < 1e-3);
            }
            _ => panic!("Drag should succeed: {:?}", result),
        }
    }

    #[test]
    fn test_drag_collision_blocking() {
        let mut asm = Assembly::new();
        let mut base = Component::new_fixed(0, "Base");
        base.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]));
        asm.add_component(base);

        let mut mover = Component::new(1, "Mover");
        mover.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [5.0, 5.0, 5.0]));
        mover.set_translation(20.0, 0.0, 0.0);
        asm.add_component(mover);

        let mut drag = KinematicDrag::new(1, Point3d::new(2.5, 2.5, 2.5));
        drag.collision_blocking = true;
        let result = drag.update(&mut asm, Point3d::new(5.0, 2.5, 2.5));

        match result {
            DragResult::CollisionDetected { a, b } => {
                assert_eq!(a.min(b), 0);
                assert_eq!(a.max(b), 1);
            }
            _ => panic!("Drag should be blocked by collision: {:?}", result),
        }
        let (tx, _, _) = asm.get_component(1).unwrap().translation();
        assert!((tx - 20.0).abs() < 1e-6, "Mover should be rolled back, tx={}", tx);
    }

    #[test]
    fn test_drag_no_collision_when_far() {
        let mut asm = Assembly::new();
        let mut base = Component::new_fixed(0, "Base");
        base.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]));
        asm.add_component(base);
        let mut mover = Component::new(1, "Mover");
        mover.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [5.0, 5.0, 5.0]));
        mover.set_translation(20.0, 0.0, 0.0);
        asm.add_component(mover);

        let mut drag = KinematicDrag::new(1, Point3d::new(2.5, 2.5, 2.5));
        let result = drag.update(&mut asm, Point3d::new(22.5, 2.5, 2.5));
        assert!(matches!(result, DragResult::Ok { .. }));
    }

    #[test]
    fn test_drag_nonexistent_component() {
        let mut asm = Assembly::new();
        asm.add_component(Component::new_fixed(0, "Base"));
        let mut drag = KinematicDrag::new(99, Point3d::new(0.0, 0.0, 0.0));
        let result = drag.update(&mut asm, Point3d::new(1.0, 1.0, 1.0));
        match result {
            DragResult::Failed(AssemblyError::ComponentNotFound(99)) => {}
            _ => panic!("Expected ComponentNotFound"),
        }
    }

    #[test]
    fn test_simulate_drag_path() {
        let mut asm = Assembly::new();
        asm.add_component(Component::new_fixed(0, "Base"));
        let mut mover = Component::new(1, "Mover");
        mover.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]));
        mover.set_translation(5.0, 0.0, 0.0);
        asm.add_component(mover);

        let path = vec![
            Point3d::new(4.0, 0.0, 0.0),
            Point3d::new(3.0, 0.0, 0.0),
            Point3d::new(2.0, 0.0, 0.0),
        ];
        let (successes, collisions) = simulate_drag_path(&mut asm, 1, Point3d::new(0.0, 0.0, 0.0), &path);
        assert_eq!(successes, 3);
        assert_eq!(collisions, 0);
    }
}
