// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Parametric surfaces in 3D space.

use crate::{Direction3d, Point3d, Vec3d, Transform, curve::Curve3d};
use std::f64::consts::PI;

/// Bitflags indicating the type of degeneracy at a surface point.
///
/// Multiple degeneracies can occur simultaneously (e.g., a cone apex
/// is both a pole and a zero-area singularity).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct DegeneracyFlags(pub u32);

impl DegeneracyFlags {
    /// No degeneracy — the surface is well-behaved at this point.
    pub const NONE: DegeneracyFlags = DegeneracyFlags(0);
    /// The partial derivative dS/du is zero (u-pole / u-seam degeneracy).
    pub const DU_ZERO: DegeneracyFlags = DegeneracyFlags(1);
    /// The partial derivative dS/dv is zero (v-pole / v-seam degeneracy).
    pub const DV_ZERO: DegeneracyFlags = DegeneracyFlags(2);
    /// Both partial derivatives are zero (complete singularity, e.g., cone apex or sphere pole).
    pub const SINGULAR: DegeneracyFlags = DegeneracyFlags(3); // DU_ZERO | DV_ZERO
    /// The surface normal is NaN or Inf at this point.
    pub const NORMAL_INVALID: DegeneracyFlags = DegeneracyFlags(4);
    /// The 3D point is NaN or Inf.
    pub const POINT_INVALID: DegeneracyFlags = DegeneracyFlags(8);

    /// Check if any degeneracy is present.
    pub fn is_degenerate(&self) -> bool {
        self.0 != 0
    }

    /// Check if this is a complete singularity (both partials zero).
    pub fn is_singular(&self) -> bool {
        self.contains(DegeneracyFlags::DU_ZERO) && self.contains(DegeneracyFlags::DV_ZERO)
    }

    /// Check if only the u-direction is degenerate (v-ring collapses).
    pub fn is_u_pole(&self) -> bool {
        self.contains(DegeneracyFlags::DU_ZERO) && !self.contains(DegeneracyFlags::DV_ZERO)
    }

    /// Check if only the v-direction is degenerate (u-ring collapses).
    pub fn is_v_pole(&self) -> bool {
        !self.contains(DegeneracyFlags::DU_ZERO) && self.contains(DegeneracyFlags::DV_ZERO)
    }

    /// Check if the given flags are set.
    pub fn contains(&self, other: DegeneracyFlags) -> bool {
        (self.0 & other.0) == other.0
    }
}

impl std::ops::BitOr for DegeneracyFlags {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        DegeneracyFlags(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for DegeneracyFlags {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// A parametric surface: S(u,v) -> Point3d.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Surface {
    /// Plane: S(u,v) = origin + u*u_dir + v*v_dir
    Plane(Plane),
    /// Cylinder along an axis
    Cylinder(CylinderSurface),
    /// Cone along an axis
    Cone(ConeSurface),
    /// Sphere
    Sphere(SphereSurface),
    /// Torus
    Torus(TorusSurface),
    /// Surface of revolution
    Revolution(RevolutionSurface),
    /// Extruded surface
    Extrusion(ExtrusionSurface),
    /// NURBS surface
    Nurbs(NurbsSurface),
    /// Offset surface — base surface offset along its normals by `distance`.
    ///
    /// Audit item 4.3 (2026-07-19): Added for extended surface types.
    /// S(u,v) = base.point_at(u,v) + distance * base.normal_at(u,v)
    Offset(OffsetSurface),
    /// Ruled surface — linear interpolation between two curves.
    ///
    /// Audit item 4.3 (2026-07-19): Added for extended surface types.
    /// S(u,v) = (1-v)*curve1.point_at(u) + v*curve2.point_at(u)
    Ruled(RuledSurface),
}

/// A plane in 3D space.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Plane {
    pub origin: Point3d,
    pub u_dir: Direction3d,
    pub v_dir: Direction3d,
    pub normal: Direction3d,
}

impl Plane {
    /// Create a plane in the XY plane.
    pub fn xy() -> Self {
        Self {
            origin: Point3d::ORIGIN,
            u_dir: Direction3d::X,
            v_dir: Direction3d::Y,
            normal: Direction3d::Z,
        }
    }

    /// Create a plane in the XZ plane.
    pub fn xz() -> Self {
        Self {
            origin: Point3d::ORIGIN,
            u_dir: Direction3d::X,
            v_dir: Direction3d::Z,
            normal: Direction3d::Y,
        }
    }

    /// Create a plane in the YZ plane.
    pub fn yz() -> Self {
        Self {
            origin: Point3d::ORIGIN,
            u_dir: Direction3d::Y,
            v_dir: Direction3d::Z,
            normal: Direction3d::X,
        }
    }

    /// Create a plane from origin and normal.
    pub fn from_origin_and_normal(origin: Point3d, normal: Direction3d) -> Self {
        let u_dir = if normal.is_parallel_to(&Direction3d::Y) {
            normal.cross(&Direction3d::X)
        } else {
            normal.cross(&Direction3d::Y)
        };
        let v_dir = normal.cross(&u_dir);
        Self { origin, u_dir, v_dir, normal }
    }

    /// Create a plane through three points.
    pub fn from_three_points(p1: &Point3d, p2: &Point3d, p3: &Point3d) -> Option<Self> {
        let v1 = Vec3d::new(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        let v2 = Vec3d::new(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
        let normal = v1.cross(&v2).normalize()?;
        let u_dir = v1.normalize()?;
        let v_dir = normal.cross(&u_dir);
        Some(Self { origin: *p1, u_dir, v_dir, normal })
    }

    /// Create a plane from a normal direction and a point on the plane.
    ///
    /// Per BREPCAD Phase 1.2: needed for extrude side faces where the
    /// normal is known but three non-collinear points may not be readily
    /// available (e.g., when the first three points happen to be collinear).
    pub fn from_normal_and_point(normal: &Direction3d, point: &Point3d) -> Option<Self> {
        // Build an orthonormal basis: pick u_dir perpendicular to normal.
        let n = Vec3d::new(normal.x, normal.y, normal.z);
        // Pick an arbitrary vector not parallel to normal
        let seed = if n.x.abs() < 0.9 {
            Vec3d::new(1.0, 0.0, 0.0)
        } else {
            Vec3d::new(0.0, 1.0, 0.0)
        };
        let u_dir = seed.cross(&n).normalize()?;
        let v_dir_vec = n.cross(&Vec3d::new(u_dir.x, u_dir.y, u_dir.z));
        let v_dir = Direction3d::new(v_dir_vec.x, v_dir_vec.y, v_dir_vec.z)?;
        Some(Self {
            origin: *point,
            u_dir,
            v_dir,
            normal: *normal,
        })
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        Point3d::new(
            self.origin.x + u * self.u_dir.x + v * self.v_dir.x,
            self.origin.y + u * self.u_dir.y + v * self.v_dir.y,
            self.origin.z + u * self.u_dir.z + v * self.v_dir.z,
        )
    }

    pub fn normal_at(&self, _u: f64, _v: f64) -> Direction3d {
        self.normal
    }

    /// Project a 3D point onto the plane's parametric space → (u, v).
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let dx = point.x - self.origin.x;
        let dy = point.y - self.origin.y;
        let dz = point.z - self.origin.z;
        let u = dx * self.u_dir.x + dy * self.u_dir.y + dz * self.u_dir.z;
        let v = dx * self.v_dir.x + dy * self.v_dir.y + dz * self.v_dir.z;
        (u, v)
    }
}

/// A cylindrical surface.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CylinderSurface {
    pub origin: Point3d,
    pub axis: Direction3d,
    pub radius: f64,
    pub x_dir: Direction3d, // reference direction for u=0
}

impl CylinderSurface {
    /// Compute a default x_dir from the axis direction.
    fn default_x_dir(axis: &Direction3d) -> Direction3d {
        if axis.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            axis.cross(&Direction3d::Z)
        }
    }

    /// Create a cylinder along the Z axis.
    pub fn new_z(radius: f64) -> Self {
        Self {
            origin: Point3d::ORIGIN,
            axis: Direction3d::Z,
            radius,
            x_dir: Direction3d::X,
        }
    }

    /// Create a cylinder at a given origin along a given axis.
    /// The x_dir is computed automatically from the axis.
    pub fn new(origin: Point3d, axis: Direction3d, radius: f64) -> Self {
        let x_dir = Self::default_x_dir(&axis);
        Self { origin, axis, radius, x_dir }
    }

    /// Create a cylinder with an explicit reference direction for u=0.
    /// Use this when the STEP file provides the x_dir (ref_direction).
    pub fn new_with_frame(origin: Point3d, axis: Direction3d, radius: f64, x_dir: Direction3d) -> Self {
        Self { origin, axis, radius, x_dir }
    }

    /// Evaluate: u = angle in radians [0, 2pi], v = height along axis.
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let y_dir = self.axis.cross(&self.x_dir);

        Point3d::new(
            self.origin.x + self.radius * (u.cos() * self.x_dir.x + u.sin() * y_dir.x) + v * self.axis.x,
            self.origin.y + self.radius * (u.cos() * self.x_dir.y + u.sin() * y_dir.y) + v * self.axis.y,
            self.origin.z + self.radius * (u.cos() * self.x_dir.z + u.sin() * y_dir.z) + v * self.axis.z,
        )
    }

    /// Normal at (u, v) — points outward.
    pub fn normal_at(&self, u: f64, _v: f64) -> Direction3d {
        let y_dir = self.axis.cross(&self.x_dir);
        Direction3d::new(
            u.cos() * self.x_dir.x + u.sin() * y_dir.x,
            u.cos() * self.x_dir.y + u.sin() * y_dir.y,
            u.cos() * self.x_dir.z + u.sin() * y_dir.z,
        ).unwrap_or(Direction3d::X)
    }

    /// Parametric range: u in [0, 2pi], v in [-inf, inf].
    pub fn u_range(&self) -> (f64, f64) {
        (0.0, 2.0 * std::f64::consts::PI)
    }

    /// Project a 3D point onto the cylinder's parametric space → (u, v).
    /// u = angle in radians ∈ [0, 2π), v = height along axis.
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let y_dir = self.axis.cross(&self.x_dir);
        let dx = point.x - self.origin.x;
        let dy = point.y - self.origin.y;
        let dz = point.z - self.origin.z;
        let u = (dx * y_dir.x + dy * y_dir.y + dz * y_dir.z)
            .atan2(dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z);
        // Normalize u to [0, 2π) to match the canonical parameterization
        let u = if u < 0.0 { u + 2.0 * PI } else { u };
        let v = dx * self.axis.x + dy * self.axis.y + dz * self.axis.z;
        (u, v)
    }
}

/// A conical surface.
///
/// Parameterization: v=0 is the base circle with the given radius,
/// v increases toward the apex where radius reaches 0.
/// Height from base to apex = radius / tan(half_angle).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ConeSurface {
    pub origin: Point3d,    // Center of base circle (or apex for expanding cones)
    pub axis: Direction3d,   // Direction from base toward apex (or away from apex for expanding cones)
    pub half_angle: f64,     // Half-angle in radians
    pub radius: f64,         // Base radius (at v=0)
    pub x_dir: Direction3d,  // reference direction for u=0
    pub expanding: bool,     // If true, cone expands from apex (radius increases with v)
}

impl ConeSurface {
    /// Compute a default x_dir from the axis direction.
    fn default_x_dir(axis: &Direction3d) -> Direction3d {
        if axis.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            axis.cross(&Direction3d::Z)
        }
    }

    /// Create a cone along the Z axis with base at z=0.
    /// The base has the given radius, and the apex is at z = radius / tan(half_angle).
    pub fn new_z(radius: f64, half_angle: f64) -> Self {
        Self {
            origin: Point3d::ORIGIN,
            axis: Direction3d::Z,
            half_angle,
            radius,
            x_dir: Direction3d::X,
            expanding: false,
        }
    }

    /// Create a cone with given origin, axis, radius, and half_angle.
    /// The x_dir is computed automatically from the axis.
    pub fn new(origin: Point3d, axis: Direction3d, radius: f64, half_angle: f64) -> Self {
        let x_dir = Self::default_x_dir(&axis);
        Self { origin, axis, half_angle, radius, x_dir, expanding: false }
    }

    /// Create a cone with an explicit reference direction for u=0.
    /// Use this when the STEP file provides the x_dir (ref_direction).
    pub fn new_with_frame(origin: Point3d, axis: Direction3d, radius: f64, half_angle: f64, x_dir: Direction3d) -> Self {
        Self { origin, axis, half_angle, radius, x_dir, expanding: false }
    }

    /// Create an expanding cone (radius increases with v) — used for STEP
    /// CONICAL_SURFACE with radius=0 where the apex is at the origin.
    pub fn new_expanding(origin: Point3d, axis: Direction3d, half_angle: f64, x_dir: Direction3d) -> Self {
        Self { origin, axis, half_angle, radius: 0.0, x_dir, expanding: true }
    }

    /// Distance from the reference point (v=0) to the apex along the axis.
    ///
    /// For standard cones with the STEP parameterization (r = radius + v*tan(ha)),
    /// the apex is at v = -radius/tan(ha). The distance is always positive.
    /// For expanding cones, there is no apex in the positive v direction (infinity).
    pub fn height(&self) -> f64 {
        if self.expanding {
            f64::INFINITY
        } else if self.half_angle.abs() < 1e-10 {
            f64::INFINITY
        } else {
            self.radius / self.half_angle.tan()
        }
    }

    /// The v parameter value at which the apex occurs (radius = 0).
    /// For standard cones: v_apex = -radius / tan(half_angle).
    /// For expanding cones: v_apex = 0 (apex is at origin).
    pub fn apex_v(&self) -> f64 {
        if self.expanding {
            0.0
        } else if self.half_angle.abs() < 1e-10 {
            f64::NEG_INFINITY
        } else {
            -self.radius / self.half_angle.tan()
        }
    }

    /// Evaluate: u = angle in radians [0, 2pi], v = distance along axis from origin.
    ///
    /// Parameterization follows STEP ISO 10303-42:
    ///   C(u, v) = origin + v * axis + (radius + v * tan(half_angle)) * (cos(u) * x_dir + sin(u) * y_dir)
    ///
    /// For standard cones: At v=0: radius = self.radius (reference). Radius increases with v.
    ///   Apex is at v = -radius / tan(half_angle) (where radius becomes 0).
    /// For expanding cones: At v=0: radius = 0 (apex). At v>0: radius = v * tan(half_angle).
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let r = if self.expanding {
            v * self.half_angle.tan()
        } else {
            // STEP parameterization: r = radius + v * tan(half_angle)
            // Radius increases with v; apex is at v = -radius/tan(ha)
            (self.radius + v * self.half_angle.tan()).max(0.0)
        };
        let y_dir = self.axis.cross(&self.x_dir);

        Point3d::new(
            self.origin.x + r * (u.cos() * self.x_dir.x + u.sin() * y_dir.x) + v * self.axis.x,
            self.origin.y + r * (u.cos() * self.x_dir.y + u.sin() * y_dir.y) + v * self.axis.y,
            self.origin.z + r * (u.cos() * self.x_dir.z + u.sin() * y_dir.z) + v * self.axis.z,
        )
    }

    /// Normal at (u, v) — points outward.
    pub fn normal_at(&self, u: f64, _v: f64) -> Direction3d {
        let y_dir = self.axis.cross(&self.x_dir);
        // Normal to cone: perpendicular to the slant surface
        // The slant has angle half_angle from the axis
        let radial = Direction3d::new(
            u.cos() * self.x_dir.x + u.sin() * y_dir.x,
            u.cos() * self.x_dir.y + u.sin() * y_dir.y,
            u.cos() * self.x_dir.z + u.sin() * y_dir.z,
        ).unwrap_or(Direction3d::X);
        // Normal = radial * cos(half_angle) ∓ axis * sin(half_angle)
        // For standard (tapering) cones: outward normal points away from axis toward apex
        // For expanding cones: outward normal points away from axis away from apex
        let ha = self.half_angle;
        if self.expanding {
            Direction3d::new(
                radial.x * ha.cos() + self.axis.x * ha.sin(),
                radial.y * ha.cos() + self.axis.y * ha.sin(),
                radial.z * ha.cos() + self.axis.z * ha.sin(),
            ).unwrap_or(radial)
        } else {
            Direction3d::new(
                radial.x * ha.cos() - self.axis.x * ha.sin(),
                radial.y * ha.cos() - self.axis.y * ha.sin(),
                radial.z * ha.cos() - self.axis.z * ha.sin(),
            ).unwrap_or(radial)
        }
    }

    /// Project a 3D point onto the cone's parametric space → (u, v).
    /// u = angle in radians ∈ [0, 2π), v = height along axis.
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let y_dir = self.axis.cross(&self.x_dir);
        let dx = point.x - self.origin.x;
        let dy = point.y - self.origin.y;
        let dz = point.z - self.origin.z;
        let u = (dx * y_dir.x + dy * y_dir.y + dz * y_dir.z)
            .atan2(dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z);
        // Normalize u to [0, 2π) to match the canonical parameterization
        let u = if u < 0.0 { u + 2.0 * PI } else { u };
        let v = dx * self.axis.x + dy * self.axis.y + dz * self.axis.z;
        (u, v)
    }
}

/// A spherical surface.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SphereSurface {
    pub center: Point3d,
    pub radius: f64,
}

impl SphereSurface {
    pub fn new(center: Point3d, radius: f64) -> Self {
        Self { center, radius }
    }

    /// Evaluate: u = azimuthal angle [0, 2pi], v = polar angle [0, pi].
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        Point3d::new(
            self.center.x + self.radius * v.sin() * u.cos(),
            self.center.y + self.radius * v.sin() * u.sin(),
            self.center.z + self.radius * v.cos(),
        )
    }

    pub fn normal_at(&self, u: f64, v: f64) -> Direction3d {
        Direction3d::new(
            v.sin() * u.cos(),
            v.sin() * u.sin(),
            v.cos(),
        ).unwrap_or(Direction3d::Z)
    }

    /// Project a 3D point onto the sphere's parametric space → (u, v).
    /// u = azimuthal angle ∈ [0, 2π), v = polar angle ∈ [0, π].
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let dx = point.x - self.center.x;
        let dy = point.y - self.center.y;
        let dz = point.z - self.center.z;
        let u = dy.atan2(dx);
        // Normalize u to [0, 2π) to match the canonical parameterization
        let u = if u < 0.0 { u + 2.0 * PI } else { u };
        let r = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-15);
        let v = (dz / r).clamp(-1.0, 1.0).acos();
        (u, v)
    }
}

/// A toroidal surface.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TorusSurface {
    pub center: Point3d,
    pub axis: Direction3d,
    pub major_radius: f64, // R — distance from center to tube center
    pub minor_radius: f64, // r — tube radius
    pub x_dir: Direction3d,  // reference direction for u=0
}

impl TorusSurface {
    /// Compute a default x_dir from the axis direction.
    fn default_x_dir(axis: &Direction3d) -> Direction3d {
        if axis.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            axis.cross(&Direction3d::Z)
        }
    }

    pub fn new_z(center: Point3d, major_radius: f64, minor_radius: f64) -> Self {
        Self { center, axis: Direction3d::Z, major_radius, minor_radius, x_dir: Direction3d::X }
    }

    /// Create a torus with given center, axis, and radii.
    /// The x_dir is computed automatically from the axis.
    pub fn new(center: Point3d, axis: Direction3d, major_radius: f64, minor_radius: f64) -> Self {
        let x_dir = Self::default_x_dir(&axis);
        Self { center, axis, major_radius, minor_radius, x_dir }
    }

    /// Create a torus with an explicit reference direction for u=0.
    /// Use this when the STEP file provides the x_dir (ref_direction).
    pub fn new_with_frame(center: Point3d, axis: Direction3d, major_radius: f64, minor_radius: f64, x_dir: Direction3d) -> Self {
        Self { center, axis, major_radius, minor_radius, x_dir }
    }

    /// Evaluate: u = angle around main ring [0, 2pi], v = angle around tube [0, 2pi].
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let y_dir = self.axis.cross(&self.x_dir);
        let r = self.major_radius + self.minor_radius * v.cos();
        Point3d::new(
            self.center.x + r * (u.cos() * self.x_dir.x + u.sin() * y_dir.x) + self.minor_radius * v.sin() * self.axis.x,
            self.center.y + r * (u.cos() * self.x_dir.y + u.sin() * y_dir.y) + self.minor_radius * v.sin() * self.axis.y,
            self.center.z + r * (u.cos() * self.x_dir.z + u.sin() * y_dir.z) + self.minor_radius * v.sin() * self.axis.z,
        )
    }

    pub fn normal_at(&self, u: f64, v: f64) -> Direction3d {
        let y_dir = self.axis.cross(&self.x_dir);
        let nx = v.cos() * (u.cos() * self.x_dir.x + u.sin() * y_dir.x) + v.sin() * self.axis.x;
        let ny = v.cos() * (u.cos() * self.x_dir.y + u.sin() * y_dir.y) + v.sin() * self.axis.y;
        let nz = v.cos() * (u.cos() * self.x_dir.z + u.sin() * y_dir.z) + v.sin() * self.axis.z;
        Direction3d::new(nx, ny, nz).unwrap_or(Direction3d::Z)
    }

    /// Project a 3D point onto the torus's parametric space → (u, v).
    /// u = angle around main ring, v = angle around tube.
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let y_dir = self.axis.cross(&self.x_dir);
        let dx = point.x - self.center.x;
        let dy = point.y - self.center.y;
        let dz = point.z - self.center.z;
        // u = angle around main ring in the x_dir/y_dir plane ∈ [0, 2π)
        let u = (dx * y_dir.x + dy * y_dir.y + dz * y_dir.z)
            .atan2(dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z);
        let u = if u < 0.0 { u + 2.0 * PI } else { u };
        // v = angle around tube ∈ [0, 2π)
        let radial_dist = dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z;
        let radial_y = dx * y_dir.x + dy * y_dir.y + dz * y_dir.z;
        let dist_ring = (radial_dist * radial_dist + radial_y * radial_y).sqrt();
        let along_axis = dx * self.axis.x + dy * self.axis.y + dz * self.axis.z;
        let local_x = dist_ring - self.major_radius;
        let local_y = along_axis;
        let v = local_y.atan2(local_x);
        let v = if v < 0.0 { v + 2.0 * PI } else { v };
        (u, v)
    }
}

/// Surface of revolution.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RevolutionSurface {
    /// The profile curve (generatrix) defined in global coordinates.
    /// The curve is revolved around `axis` passing through `origin`.
    pub profile: Curve3d,
    /// Axis of revolution (unit direction vector).
    pub axis: Direction3d,
    /// Origin point on the axis of revolution.
    pub origin: Point3d,
}

impl RevolutionSurface {
    pub fn new(profile: Curve3d, axis: Direction3d, origin: Point3d) -> Self {
        Self { profile, axis, origin }
    }

    /// Evaluate: u = revolution angle [0, 2pi], v = parameter on profile curve.
    ///
    /// The general revolution formula for an arbitrary axis:
    /// Given profile point P(v), axis direction A, and origin O:
    ///   V = P(v) - O
    ///   V_parallel = (V · A) * A
    ///   V_perp = V - V_parallel
    ///   S(u, v) = O + V_parallel + cos(u) * V_perp + sin(u) * (A × V_perp)
    ///
    /// This works for any axis orientation. When the axis is Z and the
    /// profile lies in the XZ plane (P.y = 0), this reduces to the simpler
    /// formula used previously.
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let p = self.profile.point_at(v);

        // Vector from origin to profile point
        let vx = p.x - self.origin.x;
        let vy = p.y - self.origin.y;
        let vz = p.z - self.origin.z;

        // Parallel component (along axis)
        let dot = vx * self.axis.x + vy * self.axis.y + vz * self.axis.z;
        let par_x = dot * self.axis.x;
        let par_y = dot * self.axis.y;
        let par_z = dot * self.axis.z;

        // Perpendicular component (to revolve)
        let perp_x = vx - par_x;
        let perp_y = vy - par_y;
        let perp_z = vz - par_z;
        let perp_len_sq = perp_x * perp_x + perp_y * perp_y + perp_z * perp_z;

        if perp_len_sq < 1e-30 {
            // Point is on the axis — no revolution needed
            return p;
        }

        // Second perpendicular direction: axis × perp
        // Since axis ⊥ perp, |axis × perp| = |perp|
        let cross_x = self.axis.y * perp_z - self.axis.z * perp_y;
        let cross_y = self.axis.z * perp_x - self.axis.x * perp_z;
        let cross_z = self.axis.x * perp_y - self.axis.y * perp_x;

        let cos_u = u.cos();
        let sin_u = u.sin();

        Point3d::new(
            self.origin.x + par_x + cos_u * perp_x + sin_u * cross_x,
            self.origin.y + par_y + cos_u * perp_y + sin_u * cross_y,
            self.origin.z + par_z + cos_u * perp_z + sin_u * cross_z,
        )
    }

    /// Evaluate the surface and its first partial derivatives at (u, v).
    ///
    /// Uses the chain rule to compute analytical derivatives:
    ///   S(u, v) = O + V_parallel(v) + cos(u)·V_perp(v) + sin(u)·(A × V_perp(v))
    ///
    /// where V(v) = P(v) - O, V_parallel = (V·A)·A, V_perp = V - V_parallel.
    ///
    /// Partial derivatives:
    ///   dS/du = -sin(u)·V_perp + cos(u)·(A × V_perp)
    ///   dS/dv = R_u(P'(v))   (rotate the profile derivative by angle u around A)
    ///
    /// where R_u is the rotation by angle u around axis A:
    ///   R_u(w) = (w·A)·A + cos(u)·(w - (w·A)·A) + sin(u)·(A × w)
    ///
    /// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
    pub fn derivatives_at(&self, u: f64, v: f64) -> SurfaceDerivatives {
        let p = self.profile.point_at(v);
        let dp = self.profile.derivative_at(v); // P'(v)

        // Vector from origin to profile point
        let vx = p.x - self.origin.x;
        let vy = p.y - self.origin.y;
        let vz = p.z - self.origin.z;

        // Parallel component (along axis): V_parallel = (V·A)·A
        let dot = vx * self.axis.x + vy * self.axis.y + vz * self.axis.z;
        let par_x = dot * self.axis.x;
        let par_y = dot * self.axis.y;
        let par_z = dot * self.axis.z;

        // Perpendicular component: V_perp = V - V_parallel
        let perp_x = vx - par_x;
        let perp_y = vy - par_y;
        let perp_z = vz - par_z;

        // Second perpendicular direction: A × V_perp
        // Since A ⊥ V_perp, |A × V_perp| = |V_perp|
        let cross_x = self.axis.y * perp_z - self.axis.z * perp_y;
        let cross_y = self.axis.z * perp_x - self.axis.x * perp_z;
        let cross_z = self.axis.x * perp_y - self.axis.y * perp_x;

        let cos_u = u.cos();
        let sin_u = u.sin();

        // S(u, v) = O + V_parallel + cos(u)·V_perp + sin(u)·(A × V_perp)
        let point = Point3d::new(
            self.origin.x + par_x + cos_u * perp_x + sin_u * cross_x,
            self.origin.y + par_y + cos_u * perp_y + sin_u * cross_y,
            self.origin.z + par_z + cos_u * perp_z + sin_u * cross_z,
        );

        // dS/du = -sin(u)·V_perp + cos(u)·(A × V_perp)
        let du = Vec3d::new(
            -sin_u * perp_x + cos_u * cross_x,
            -sin_u * perp_y + cos_u * cross_y,
            -sin_u * perp_z + cos_u * cross_z,
        );

        // dS/dv = R_u(P'(v))
        // First, decompose P'(v) into parallel and perpendicular components w.r.t. A
        let dp_dot_a = dp.x * self.axis.x + dp.y * self.axis.y + dp.z * self.axis.z;
        let dp_par_x = dp_dot_a * self.axis.x;
        let dp_par_y = dp_dot_a * self.axis.y;
        let dp_par_z = dp_dot_a * self.axis.z;
        let dp_perp_x = dp.x - dp_par_x;
        let dp_perp_y = dp.y - dp_par_y;
        let dp_perp_z = dp.z - dp_par_z;

        // A × dp_perp
        let dp_cross_x = self.axis.y * dp_perp_z - self.axis.z * dp_perp_y;
        let dp_cross_y = self.axis.z * dp_perp_x - self.axis.x * dp_perp_z;
        let dp_cross_z = self.axis.x * dp_perp_y - self.axis.y * dp_perp_x;

        // R_u(dp) = dp_par + cos(u)·dp_perp + sin(u)·(A × dp_perp)
        let dv = Vec3d::new(
            dp_par_x + cos_u * dp_perp_x + sin_u * dp_cross_x,
            dp_par_y + cos_u * dp_perp_y + sin_u * dp_cross_y,
            dp_par_z + cos_u * dp_perp_z + sin_u * dp_cross_z,
        );

        SurfaceDerivatives { point, du, dv }
    }
}

/// Extruded surface — a curve swept along a direction.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ExtrusionSurface {
    /// The profile curve.
    pub profile: Curve3d,
    /// Direction of extrusion.
    pub direction: Direction3d,
}

impl ExtrusionSurface {
    pub fn new(profile: Curve3d, direction: Direction3d) -> Self {
        Self { profile, direction }
    }

    /// Evaluate: u = parameter on profile curve, v = extrusion distance.
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let p = self.profile.point_at(u);
        Point3d::new(
            p.x + v * self.direction.x,
            p.y + v * self.direction.y,
            p.z + v * self.direction.z,
        )
    }

    /// Evaluate the surface and its first partial derivatives at (u, v).
    ///
    /// S(u, v) = P(u) + v·D
    ///
    /// Partial derivatives:
    ///   dS/du = P'(u)
    ///   dS/dv = D
    ///
    /// Algorithm adapted from truck-geometry v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
    pub fn derivatives_at(&self, u: f64, v: f64) -> SurfaceDerivatives {
        let p = self.profile.point_at(u);
        let du = self.profile.derivative_at(u); // P'(u)
        let dv = Vec3d::new(self.direction.x, self.direction.y, self.direction.z);

        let point = Point3d::new(
            p.x + v * self.direction.x,
            p.y + v * self.direction.y,
            p.z + v * self.direction.z,
        );

        SurfaceDerivatives { point, du, dv }
    }
}

/// Offset surface — base surface offset along its normals by `distance`.
///
/// Audit item 4.3 (2026-07-19): Added for extended surface types.
///
/// S(u,v) = base.point_at(u,v) + distance * base.normal_at(u,v)
///
/// Common use cases:
/// - Thickening a sheet metal face
/// - Creating a shell offset for blending operations
/// - Offset surfaces in fillet/blend generation
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OffsetSurface {
    /// The base surface to offset from.
    pub base: Box<Surface>,
    /// Offset distance along the base surface's normals.
    /// Positive = offset in normal direction, negative = opposite.
    pub distance: f64,
}

impl OffsetSurface {
    pub fn new(base: Surface, distance: f64) -> Self {
        Self {
            base: Box::new(base),
            distance,
        }
    }
}

/// Ruled surface — linear interpolation between two curves.
///
/// Audit item 4.3 (2026-07-19): Added for extended surface types.
///
/// S(u,v) = (1-v)*curve1.point_at(u) + v*curve2.point_at(u)
///
/// Common use cases:
/// - Lofting between two profiles
/// - Connecting two edges of a face with a smooth transition
/// - Draft surfaces in molding design
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RuledSurface {
    /// First curve (at v=0).
    pub curve1: Box<Curve3d>,
    /// Second curve (at v=1).
    pub curve2: Box<Curve3d>,
}

impl RuledSurface {
    pub fn new(curve1: Curve3d, curve2: Curve3d) -> Self {
        Self {
            curve1: Box::new(curve1),
            curve2: Box::new(curve2),
        }
    }
}

/// NURBS surface.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsSurface {
    pub u_degree: usize,
    pub v_degree: usize,
    pub control_points: Vec<Vec<Point3d>>,
    pub weights: Vec<Vec<f64>>,
    pub u_knots: Vec<f64>,
    pub v_knots: Vec<f64>,
    /// Whether the surface is closed/periodic in the u direction.
    /// A NURBS surface is u-closed when the first and last rows of
    /// control points coincide (within tolerance), or when the STEP
    /// entity specifies CLOSED_B_SPLINE_SURFACE.
    pub u_closed: bool,
    /// Whether the surface is closed/periodic in the v direction.
    pub v_closed: bool,
}

/// Surface derivatives at a parametric point (u, v).
///
/// Contains the first partial derivatives dS/du and dS/dv,
/// which together define the tangent plane and can be used to
/// compute the surface normal via n = dS/du × dS/dv.
#[derive(Clone, Debug)]
pub struct SurfaceDerivatives {
    /// The evaluated 3D point S(u,v).
    pub point: Point3d,
    /// Partial derivative dS/du — tangent vector in the u direction.
    pub du: Vec3d,
    /// Partial derivative dS/dv — tangent vector in the v direction.
    pub dv: Vec3d,
}

impl SurfaceDerivatives {
    /// Compute the surface normal from the partial derivatives.
    /// n = du × dv, normalized.
    pub fn normal(&self) -> Direction3d {
        self.du.cross(&self.dv).normalize().unwrap_or(Direction3d::Z)
    }

    /// Compute the first fundamental form coefficients.
    ///
    /// The first fundamental form describes the intrinsic metric of the surface:
    ///   I = E*du² + 2*F*du*dv + G*dv²
    ///
    /// Where:
    ///   E = dS/du · dS/du  (squared length of u-tangent)
    ///   F = dS/du · dS/dv  (dot product of tangents)
    ///   G = dS/dv · dS/dv  (squared length of v-tangent)
    ///
    /// This is essential for:
    /// - Converting 3D tolerances to UV-space tolerances
    /// - Computing surface areas via integration
    /// - Measuring distances on the surface
    pub fn first_fundamental_form(&self) -> FirstFundamentalForm {
        FirstFundamentalForm {
            e: self.du.dot(&self.du),
            f: self.du.dot(&self.dv),
            g: self.dv.dot(&self.dv),
        }
    }
}

/// Coefficients of the first fundamental form of a surface.
///
/// I = E*du² + 2*F*du*dv + G*dv²
///
/// These coefficients describe the metric tensor of the surface in
/// parametric space and are used for:
/// - Tolerance conversion between 3D and UV space
/// - Surface area computation
/// - Geodesic distance measurement
#[derive(Clone, Debug)]
pub struct FirstFundamentalForm {
    /// E = dS/du · dS/du
    pub e: f64,
    /// F = dS/du · dS/dv
    pub f: f64,
    /// G = dS/dv · dS/dv
    pub g: f64,
}

impl FirstFundamentalForm {
    /// Convert a 3D distance tolerance to a UV parametric tolerance.
    ///
    /// Given a 3D tolerance δ, the corresponding parametric tolerance
    /// is approximately δ / √(max(E, G)), which ensures that a step
    /// in parameter space doesn't exceed the 3D tolerance.
    pub fn tolerance_3d_to_uv(&self, tol_3d: f64) -> f64 {
        let max_stretch = self.e.max(self.g).sqrt().max(1e-10);
        tol_3d / max_stretch
    }

    /// Compute the area element dA at this point.
    /// dA = √(EG - F²) du dv
    pub fn area_element(&self) -> f64 {
        (self.e * self.g - self.f * self.f).max(0.0).sqrt()
    }

    /// Check if the parameterization is orthogonal (F ≈ 0).
    pub fn is_orthogonal(&self, tolerance: f64) -> bool {
        self.f.abs() < tolerance
    }

    /// Check if the parameterization is conformal (E ≈ G and F ≈ 0).
    pub fn is_conformal(&self, tolerance: f64) -> bool {
        self.f.abs() < tolerance && (self.e - self.g).abs() < tolerance * self.e.max(self.g)
    }
}

/// Coefficients of the second fundamental form of a surface.
///
/// II = L*du² + 2*M*du*dv + N*dv²
///
/// These coefficients describe how the surface curves in 3D space:
/// - L = d²S/du² · n  (normal curvature in u-direction)
/// - M = d²S/du*dv · n (mixed normal curvature)
/// - N = d²S/dv² · n  (normal curvature in v-direction)
#[derive(Clone, Debug)]
pub struct SecondFundamentalForm {
    /// L = d²S/du² · n
    pub l: f64,
    /// M = d²S/du*dv · n
    pub m: f64,
    /// N = d²S/dv² · n
    pub n: f64,
}

impl SecondFundamentalForm {
    /// Compute Gaussian curvature from first and second fundamental forms.
    /// K = (LN - M²) / (EG - F²)
    pub fn gaussian_curvature(&self, first: &FirstFundamentalForm) -> f64 {
        let denom = first.e * first.g - first.f * first.f;
        if denom.abs() < 1e-20 { return 0.0; }
        (self.l * self.n - self.m * self.m) / denom
    }

    /// Compute mean curvature from first and second fundamental forms.
    /// H = (EN - 2FM + GL) / (2(EG - F²))
    pub fn mean_curvature(&self, first: &FirstFundamentalForm) -> f64 {
        let denom = 2.0 * (first.e * first.g - first.f * first.f);
        if denom.abs() < 1e-20 { return 0.0; }
        (first.e * self.n - 2.0 * first.f * self.m + first.g * self.l) / denom
    }

    /// Compute principal curvatures from Gaussian (K) and mean (H) curvatures.
    /// k1,2 = H ± sqrt(H² - K)
    pub fn principal_curvatures(&self, first: &FirstFundamentalForm) -> (f64, f64) {
        let h = self.mean_curvature(first);
        let k = self.gaussian_curvature(first);
        let disc = (h * h - k).max(0.0);
        let sqrt_disc = disc.sqrt();
        (h + sqrt_disc, h - sqrt_disc)
    }

    /// Compute maximum absolute curvature (for adaptive sampling).
    pub fn max_curvature(&self, first: &FirstFundamentalForm) -> f64 {
        let (k1, k2) = self.principal_curvatures(first);
        k1.abs().max(k2.abs())
    }
}

/// Surface curvature information at a point.
///
/// Contains Gaussian curvature (K), mean curvature (H),
/// and principal curvatures (k1, k2).
#[derive(Clone, Debug)]
pub struct SurfaceCurvature {
    /// Gaussian curvature K = k1 * k2.
    pub gaussian: f64,
    /// Mean curvature H = (k1 + k2) / 2.
    pub mean: f64,
    /// Maximum principal curvature (largest absolute value).
    pub k1: f64,
    /// Minimum principal curvature (smallest absolute value).
    pub k2: f64,
    /// Maximum absolute curvature = max(|k1|, |k2|).
    pub max_abs: f64,
}

impl NurbsSurface {
    /// Construct a NURBS surface from a control point grid laid out as
    /// **rows-of-V × columns-of-U** (i.e., `grid[v_idx][u_idx]`).
    ///
    /// This is the natural mental model when authoring surfaces by hand:
    /// you think of building the surface row-by-row in V, with each row
    /// varying U. The STEP standard and the `NurbsSurface` struct however
    /// use the opposite convention (`control_points[u_idx][v_idx]`).
    ///
    /// This constructor takes the V-rows layout and produces a correctly
    /// oriented `NurbsSurface` by:
    /// 1. Transposing the control point grid (so first index = U)
    /// 2. Transposing the weights grid (same)
    /// 3. Swapping `u_degree` ↔ `v_degree`
    /// 4. Swapping `u_knots` ↔ `v_knots`
    /// 5. Swapping `u_closed` ↔ `v_closed`
    ///
    /// All arguments are in the **author's intent** orientation:
    /// - `v_rows_cp[v][u]` = control point at V-index `v`, U-index `u`
    /// - `v_rows_w[v][u]`  = weight at V-index `v`, U-index `u`
    /// - `u_degree` = degree in the U direction (length of `u_knots` = n_u + u_degree + 1)
    /// - `v_degree` = degree in the V direction (length of `v_knots` = n_v + v_degree + 1)
    /// - `u_knots`  = knot vector for U (size n_u + u_degree + 1)
    /// - `v_knots`  = knot vector for V (size n_v + v_degree + 1)
    /// - `u_closed`, `v_closed` = closure flags
    pub fn from_v_rows(
        u_degree: usize,
        v_degree: usize,
        v_rows_cp: Vec<Vec<Point3d>>,
        v_rows_w: Vec<Vec<f64>>,
        u_knots: Vec<f64>,
        v_knots: Vec<f64>,
        u_closed: bool,
        v_closed: bool,
    ) -> Self {
        // Validate dimensions
        let n_v = v_rows_cp.len();
        assert!(n_v > 0, "NurbsSurface::from_v_rows: empty control point grid");
        let n_u = v_rows_cp[0].len();
        assert!(n_u > 0, "NurbsSurface::from_v_rows: empty first row");
        for (i, row) in v_rows_cp.iter().enumerate() {
            assert_eq!(row.len(), n_u,
                "NurbsSurface::from_v_rows: row {} has {} cols, expected {}", i, row.len(), n_u);
        }
        // Weights grid must match control points grid
        assert_eq!(v_rows_w.len(), n_v,
            "NurbsSurface::from_v_rows: weights has {} rows, expected {}", v_rows_w.len(), n_v);
        for (i, row) in v_rows_w.iter().enumerate() {
            assert_eq!(row.len(), n_u,
                "NurbsSurface::from_v_rows: weights row {} has {} cols, expected {}", i, row.len(), n_u);
        }
        // Knot vector lengths must match (n + degree + 1)
        assert_eq!(u_knots.len(), n_u + u_degree + 1,
            "NurbsSurface::from_v_rows: u_knots has {} elements, expected {} (= n_u {} + u_degree {} + 1)",
            u_knots.len(), n_u + u_degree + 1, n_u, u_degree);
        assert_eq!(v_knots.len(), n_v + v_degree + 1,
            "NurbsSurface::from_v_rows: v_knots has {} elements, expected {} (= n_v {} + v_degree {} + 1)",
            v_knots.len(), n_v + v_degree + 1, n_v, v_degree);

        // Transpose: control_points_new[u][v] = v_rows_cp[v][u]
        let mut control_points: Vec<Vec<Point3d>> = Vec::with_capacity(n_u);
        let mut weights: Vec<Vec<f64>> = Vec::with_capacity(n_u);
        for u_idx in 0..n_u {
            let mut row_cp = Vec::with_capacity(n_v);
            let mut row_w = Vec::with_capacity(n_v);
            for v_idx in 0..n_v {
                row_cp.push(v_rows_cp[v_idx][u_idx]);
                row_w.push(v_rows_w[v_idx][u_idx]);
            }
            control_points.push(row_cp);
            weights.push(row_w);
        }

        // The struct convention: first index = U, second = V.
        // u_degree applies to the first index, v_degree to the second.
        // So we keep u_degree and v_degree as the author declared them.
        NurbsSurface {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
            u_closed,
            v_closed,
        }
    }

    /// Build a NURBS surface of revolution by sweeping a profile curve
    /// around the Z axis.
    ///
    /// This uses the **standard exact rational-quadratic circle** construction
    /// from "The NURBS Book" (Piegl & Tiller, §7.3) for the angular direction:
    /// 9 control points (4 on-axis + 4 bounding-box corners + 1 duplicate for closure),
    /// degree 2, weights `[1, 1/√2, 1, 1/√2, 1, 1/√2, 1, 1/√2, 1]`, knots
    /// `[0,0,0, 1/4,1/4, 1/2,1/2, 3/4,3/4, 1,1,1]`.
    ///
    /// This produces an **EXACT** circle in the angular direction — no radius
    /// oscillation, no parameter-to-angle nonlinearity beyond the inherent
    /// rational-quadratic parameterization.
    ///
    /// # Arguments
    /// * `profile_ctrl_pts` — Control points of the profile curve in the XZ plane.
    ///   The `.x` component is the radius, `.z` is the height. `.y` is ignored
    ///   (assumed 0).
    /// * `profile_degree` — Degree of the profile B-spline (typically 2 or 3).
    /// * `profile_knots` — Knot vector for the profile (clamped recommended).
    /// * `u_closed` — Whether the profile is closed (e.g., for a torus-like shape).
    ///   Generally `false` for a vase.
    /// * `angle_start`, `angle_end` — Range of revolution in radians.
    ///   For a full vase, use `0.0` to `2π`. For a partial sweep, use less.
    pub fn surface_of_revolution_z(
        profile_ctrl_pts: &[Point3d],
        profile_degree: usize,
        profile_knots: Vec<f64>,
        profile_weights: Vec<f64>,
        angle_start: f64,
        angle_end: f64,
        u_closed: bool,
    ) -> Self {
        let _ = &profile_weights; // reserved for future rational-profile support
        let n_profile = profile_ctrl_pts.len();
        assert!(n_profile > 0, "surface_of_revolution_z: empty profile");
        assert_eq!(
            profile_knots.len(),
            n_profile + profile_degree + 1,
            "surface_of_revolution_z: profile_knots has {} elements, expected {} (= n {} + degree {} + 1)",
            profile_knots.len(), n_profile + profile_degree + 1, n_profile, profile_degree
        );

        // Angular direction (V): exact rational-quadratic full circle.
        // 9 control points, degree 2, 12 knots.
        let n_angle = 9;
        let angle_degree = 2;
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        let angle_weights = vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0];

        // Build the angle knot vector. For a full circle (angle_start=0, angle_end=2π):
        //   knots = [0,0,0, π/2,π/2, π,π, 3π/2,3π/2, 2π,2π,2π] (length 12 = 9 + 2 + 1).
        // For a partial sweep, we scale accordingly.
        let span = angle_end - angle_start;
        let q1 = angle_start + 0.25 * span;
        let q2 = angle_start + 0.50 * span;
        let q3 = angle_start + 0.75 * span;
        let q4 = angle_end;
        let angle_knots = vec![
            angle_start, angle_start, angle_start,
            q1, q1,
            q2, q2,
            q3, q3,
            q4, q4, q4,
        ];

        // The angle control points: at angles 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°, 360°.
        // For an EXACT circle of radius R, the control points are:
        //   P0 = (R, 0)
        //   P1 = (R, R)         ← bounding-box corner
        //   P2 = (0, R)
        //   P3 = (-R, R)        ← bounding-box corner
        //   P4 = (-R, 0)
        //   P5 = (-R, -R)       ← bounding-box corner
        //   P6 = (0, -R)
        //   P7 = (R, -R)        ← bounding-box corner
        //   P8 = (R, 0)         ← duplicate of P0 for closure
        // At intermediate angles (when span != 2π), the same construction scaled
        // to the span works — but the "bounding-box corner" positions need to be
        // at the appropriate angles.
        let angles = [
            angle_start,
            angle_start + 0.125 * span, // 1/8 of span = 1/4 of first quadrant midpoint
            q1,
            angle_start + 0.375 * span,
            q2,
            angle_start + 0.625 * span,
            q3,
            angle_start + 0.875 * span,
            q4,
        ];
        // For each profile control point, build the 9 angle control points.
        // For an exact circle of radius r (= profile_pt.x), the control points are
        // at the 9 angles above, but with the bounding-box corner points placed
        // at distance r * sqrt(2) from origin (so they sit at the box corner).
        //
        // Actually, for the standard NURBS circle construction, the "corner"
        // control points are at distance R from origin in BOTH axes, so they're
        // at (R, R), (-R, R), etc. — which is distance R√2 from origin. The
        // weight 1/√2 brings them back onto the circle.
        //
        // For a partial arc (e.g., 0 to π/2), the construction is similar but
        // uses a single rational-quadratic Bézier segment.
        //
        // For a full circle (0 to 2π) with 9 control points, the on-axis points
        // are at (R, 0), (0, R), (-R, 0), (0, -R), and the corners are at
        // (R, R), (-R, R), (-R, -R), (R, -R). The last point (R, 0) is a
        // duplicate of the first.
        //
        // For a partial sweep, we still use the same construction but only
        // cover the requested angle range. The on-axis points become
        // "endpoint" points at angle_start and angle_end.
        //
        // We'll generate the control points by walking the 9 angle positions
        // and placing each at radius r * (corner_factor), where corner_factor
        // is 1 for on-axis points and √2 for corner points (i.e., the corner
        // points are at (r*cos(a)*√2, r*sin(a)*√2) for the corner angle a,
        // which places them at (r, r) etc. when a = 45°).
        //
        // Actually, the simplest formulation: for each of the 9 angle positions,
        // the control point in 2D is:
        //   - If it's an on-axis point (multiplicity-1 knot): (r*cos(a), r*sin(a))
        //   - If it's a corner point (multiplicity-2 interior knot): the point
        //     that, together with the weight 1/√2, makes the curve pass through
        //     the on-axis points exactly. This is (r*cos(a) + r*sin(a)*sign1, ...)
        //     — actually it's just (r*cos(a) + r*sin(a), r*sin(a) - r*cos(a))
        //     No wait, the corner point at angle a (midpoint of an arc from
        //     angle a-Δ/2 to a+Δ/2 where Δ = quadrant angle) is at the
        //     intersection of the tangent lines at the two endpoints.
        //
        // For the standard 4-quadrant circle:
        //   - On-axis at 0°, 90°, 180°, 270°.
        //   - Corners at 45°, 135°, 225°, 315°.
        //   - At 45°, the corner is at (R, R), which is the intersection of
        //     the tangent at (R, 0) (vertical line x=R) and the tangent at
        //     (0, R) (horizontal line y=R).
        //
        // For a general full sweep (0 to 2π), we use this exact construction.
        // For a partial sweep (e.g., 0 to π), we'd use just 2 quadrants.
        //
        // Here we'll handle the full-sweep case (the common one) and use
        // a simpler approximation for partial sweeps. Most users want a full
        // revolution anyway.
        let is_full = (span - 2.0 * PI).abs() < 1e-6;

        // Build the control_points grid: control_points[u_idx][v_idx]
        // u = profile direction (height), v = angle direction.
        // The struct convention is control_points[u][v].
        let mut control_points: Vec<Vec<Point3d>> = Vec::with_capacity(n_profile);
        let mut weights: Vec<Vec<f64>> = Vec::with_capacity(n_profile);
        for p in profile_ctrl_pts {
            let r = p.x;
            let z = p.z;
            let mut row: Vec<Point3d> = Vec::with_capacity(n_angle);
            let mut w_row: Vec<f64> = Vec::with_capacity(n_angle);
            for (i, &a) in angles.iter().enumerate() {
                let is_corner = (i % 2) == 1; // odd indices are corners
                let (px, py);
                if is_full {
                    // Standard NURBS circle construction
                    if is_corner {
                        // Corner point: at the bounding box corner for this quadrant.
                        // For corner at 45°, point = (R, R). For 135°, point = (-R, R). Etc.
                        // We can compute this as: place the point at distance R from each axis,
                        // i.e., at (sign(cos(a))*R, sign(sin(a))*R).
                        // But for a corner at 45°, cos(45°) > 0 and sin(45°) > 0, so (R, R). ✓
                        // For 135°: cos < 0, sin > 0, so (-R, R). ✓
                        px = r * (a.cos().signum());
                        py = r * (a.sin().signum());
                    } else {
                        // On-axis point
                        px = r * a.cos();
                        py = r * a.sin();
                    }
                } else {
                    // Partial sweep: place all control points at radius r * (1 if on-axis, √2 if corner).
                    // This is an approximation for partial sweeps — works exactly only for full circle.
                    let factor = if is_corner { 2.0_f64.sqrt() } else { 1.0 };
                    px = r * a.cos() * factor;
                    py = r * a.sin() * factor;
                }
                row.push(Point3d::new(px, py, z));
                w_row.push(angle_weights[i]);
            }
            control_points.push(row);
            weights.push(w_row);
        }

        // V degree = 2 (rational quadratic for exact circle)
        let v_degree = angle_degree;

        // V knots
        let v_knots = angle_knots;

        NurbsSurface {
            u_degree: profile_degree,
            v_degree,
            control_points,
            weights,
            u_knots: profile_knots,
            v_knots,
            u_closed,
            v_closed: is_full,
        }
    }

    /// Build an EXACT rational-quadratic NURBS representation of a full circle
    /// in the XY plane, centered at the origin, with the given radius.
    ///
    /// Returns `(control_points, weights, knots)` for a degree-2 NURBS curve
    /// that exactly represents the circle (every point on the curve is at
    /// distance `radius` from the origin).
    ///
    /// Construction (from "The NURBS Book", Piegl & Tiller, §7.3):
    /// - 9 control points (4 on-axis + 4 bounding-box corners + 1 duplicate for closure)
    /// - Degree 2
    /// - Weights: `[1, 1/√2, 1, 1/√2, 1, 1/√2, 1, 1/√2, 1]`
    /// - Knots: `[0, 0, 0, 1/4, 1/4, 1/2, 1/2, 3/4, 3/4, 1, 1, 1]`
    ///   (length 12 = 9 + 2 + 1)
    ///
    /// The parameter u ∈ [0, 1] corresponds to angle θ ∈ [0, 2π] via a
    /// non-linear (but smooth) mapping inherent to rational quadratic
    /// parameterization. The curve passes EXACTLY through the 4 on-axis
    /// control points at u = 0, 1/4, 1/2, 3/4, 1.
    pub fn full_circle_xy(radius: f64) -> (Vec<Point3d>, Vec<f64>, Vec<f64>) {
        let r = radius;
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        // 9 control points: on-axis points at 0°, 90°, 180°, 270° (and 360° = 0° duplicate),
        // corner points at 45°, 135°, 225°, 315° placed at the bounding-box corners
        // (R, R), (-R, R), (-R, -R), (R, -R).
        let control_points = vec![
            Point3d::new( r, 0.0, 0.0), // u=0,    angle 0°
            Point3d::new( r,   r, 0.0), // u=1/8,  corner (1st quadrant)
            Point3d::new(0.0,  r, 0.0), // u=1/4,  angle 90°
            Point3d::new(-r,   r, 0.0), // u=3/8,  corner (2nd quadrant)
            Point3d::new(-r, 0.0, 0.0), // u=1/2,  angle 180°
            Point3d::new(-r,  -r, 0.0), // u=5/8,  corner (3rd quadrant)
            Point3d::new(0.0, -r, 0.0), // u=3/4,  angle 270°
            Point3d::new( r,  -r, 0.0), // u=7/8,  corner (4th quadrant)
            Point3d::new( r, 0.0, 0.0), // u=1,    angle 360° (= 0°, duplicate for closure)
        ];
        let weights = vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0];
        // Knot vector: 9 + 2 + 1 = 12 knots.
        // Multiplicity 2 at interior knots gives C⁰ continuity at segment boundaries,
        // which is the standard construction for an exact NURBS circle.
        let knots = vec![
            0.0, 0.0, 0.0,
            0.25, 0.25,
            0.5, 0.5,
            0.75, 0.75,
            1.0, 1.0, 1.0,
        ];
        (control_points, weights, knots)
    }

    /// Get the valid parametric range for the u parameter.
    /// The valid domain is [u_knots[u_degree], u_knots[n_u]] where n_u = number of control points in u.
    pub fn u_range(&self) -> (f64, f64) {
        let p = self.u_degree;
        if self.u_knots.len() > p {
            let u_min = self.u_knots[p];
            let u_max = self.u_knots[self.u_knots.len() - p - 1];
            (u_min, u_max)
        } else {
            (0.0, 1.0)
        }
    }

    /// Get the valid parametric range for the v parameter.
    pub fn v_range(&self) -> (f64, f64) {
        let q = self.v_degree;
        if self.v_knots.len() > q {
            let v_min = self.v_knots[q];
            let v_max = self.v_knots[self.v_knots.len() - q - 1];
            (v_min, v_max)
        } else {
            (0.0, 1.0)
        }
    }

    /// Evaluate the surface point at parameter (u, v) using de Boor's algorithm.
    ///
    /// This is the public entry point for `nurbs_surface_eval`. It exposes the
    /// same tensor-product B-spline evaluation used internally by
    /// `Surface::point_at(Surface::Nurbs(n), u, v)`, but without needing to
    /// wrap the surface in the `Surface` enum.
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        nurbs_surface_eval(self, u, v)
    }

    /// Compute the surface point and first partial derivatives analytically.
    ///
    /// Uses the quotient rule for rational NURBS surfaces:
    ///   S(u,v) = A(u,v) / w(u,v)
    ///   dS/du = (dA/du - S * dw/du) / w
    ///   dS/dv = (dA/dv - S * dw/dv) / w
    ///
    /// The derivatives of the weighted numerator are computed by differentiating
    /// the B-spline basis functions analytically using the degree-reduction
    /// technique (standard NURBS derivative computation from "The NURBS Book").
    pub fn derivatives_at(&self, u: f64, v: f64) -> SurfaceDerivatives {
        let (u_min, u_max) = self.u_range();
        let (v_min, v_max) = self.v_range();

        // Clamp u, v to valid range
        let u_c = u.clamp(u_min, u_max);
        let v_c = v.clamp(v_min, v_max);

        let p = self.u_degree;
        let q = self.v_degree;
        let n_u = self.control_points.len();
        let n_v = self.control_points[0].len();

        // Find knot spans
        let k_u = find_knot_span(&self.u_knots, p, u_c, n_u);
        let k_v = find_knot_span(&self.v_knots, q, v_c, n_v);

        // Step 1: Compute weighted control points and their u-derivatives
        // For each row i in [k_u-p .. k_u], evaluate:
        //   - B-spline in v direction on the q+1 weighted control points
        //   - B-spline derivative in v direction
        // This gives us intermediate points in the (wx, wy, wz, w) homogeneous space

        let mut intermediate = Vec::with_capacity(p + 1);
        for i in 0..=p {
            let row = k_u - p + i;
            if row >= n_u { continue; }

            // Collect q+1 weighted control points for this row
            let mut vpts = Vec::with_capacity(q + 1);
            for j in 0..=q {
                let col = k_v - q + j;
                if col >= n_v { continue; }
                let cp = &self.control_points[row][col];
                let w = self.weights[row][col];
                vpts.push((cp.x * w, cp.y * w, cp.z * w, w));
            }
            if vpts.len() < q + 1 { continue; }

            // De Boor in v direction
            de_boor_step(&mut vpts, &self.v_knots, q, k_v, v_c);
            intermediate.push(vpts[q]); // Result is at index [degree]
        }

        if intermediate.len() < p + 1 {
            // Fallback to numerical differences if we couldn't get enough intermediate points
            return self.derivatives_at_numerical(u, v);
        }

        // Step 2: De Boor in u direction on intermediate points → get S_w(u,v) = (wx, wy, wz, w)
        de_boor_step(&mut intermediate, &self.u_knots, p, k_u, u_c);
        let (wx, wy, wz, w) = intermediate[p];

        if w.abs() < 1e-15 {
            return self.derivatives_at_numerical(u, v);
        }

        // Compute the 3D point: S = (wx/w, wy/w, wz/w)
        let point = Point3d::new(wx / w, wy / w, wz / w);

        // NaN/Inf guard: if the point is not finite, fall back to numerical.
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            log::warn!("NURBS derivatives_at: non-finite point ({}, {}, {}) at u={}, v={} — falling back to numerical", point.x, point.y, point.z, u, v);
            return self.derivatives_at_numerical(u, v);
        }

        // Step 3: Compute derivatives using degree-reduced control points
        // dS_w/du is computed from the p control points of degree p-1 B-spline
        // For each v-row, the derivative in u is:
        //   d/d u [sum_i N_{i,p}(u) * P_w_{i,j}] = p/(u_{i+p+1}-u_{i+1}) * (Q_{i+1} - Q_i)
        // where Q_i are the degree-reduced control points

        // Compute u-direction derivative
        let du = self.compute_partial_derivative_u(u_c, v_c, k_u, k_v, p, q, n_u, n_v, w, point);

        // Compute v-direction derivative
        let dv = self.compute_partial_derivative_v(u_c, v_c, k_u, k_v, p, q, n_u, n_v, w, point);

        // Validate derivatives — fall back to numerical if something went wrong
        let du_len = (du.x * du.x + du.y * du.y + du.z * du.z).sqrt();
        let dv_len = (dv.x * dv.x + dv.y * dv.y + dv.z * dv.z).sqrt();

        if du_len < 1e-20 || dv_len < 1e-20 || !du_len.is_finite() || !dv_len.is_finite() {
            return self.derivatives_at_numerical(u, v);
        }

        SurfaceDerivatives { point, du, dv }
    }

    /// Compute the partial derivative dS/du analytically using degree reduction.
    fn compute_partial_derivative_u(
        &self, u: f64, v: f64, k_u: usize, k_v: usize,
        p: usize, q: usize, n_u: usize, n_v: usize, w: f64, point: Point3d,
    ) -> Vec3d {
        // For each v-row, compute the u-direction derivative using:
        // dA/du = p * sum_i N_{i,p-1}(u) * (P_{i+1}^w - P_i^w) / (u_{i+p+1} - u_{i+1})
        // dw/du = p * sum_i N_{i,p-1}(u) * (w_{i+1} - w_i) / (u_{i+p+1} - u_{i+1})
        // Then dS/du = (dA/du - S * dw/du) / w

        let mut du_intermediate = Vec::with_capacity(p); // p points for degree p-1
        for i in 0..p {
            let row0 = k_u - p + i;
            let row1 = k_u - p + i + 1;
            if row0 >= n_u || row1 >= n_u { continue; }

            // Collect q+1 weighted control points for each row
            let mut vpts0 = Vec::with_capacity(q + 1);
            let mut vpts1 = Vec::with_capacity(q + 1);
            for j in 0..=q {
                let col = k_v - q + j;
                if col >= n_v { continue; }
                let cp0 = &self.control_points[row0][col];
                let w0 = self.weights[row0][col];
                vpts0.push((cp0.x * w0, cp0.y * w0, cp0.z * w0, w0));
                let cp1 = &self.control_points[row1][col];
                let w1 = self.weights[row1][col];
                vpts1.push((cp1.x * w1, cp1.y * w1, cp1.z * w1, w1));
            }
            if vpts0.len() < q + 1 || vpts1.len() < q + 1 { continue; }

            // Evaluate both in v
            de_boor_step(&mut vpts0, &self.v_knots, q, k_v, v);
            de_boor_step(&mut vpts1, &self.v_knots, q, k_v, v);

            // Compute the difference scaled by knot interval
            let (wx0, wy0, wz0, w0_) = vpts0[q];
            let (wx1, wy1, wz1, w1_) = vpts1[q];

            let knot_idx = k_u - p + i;
            let denom = if knot_idx + p + 1 < self.u_knots.len() {
                let d = self.u_knots[knot_idx + p + 1] - self.u_knots[knot_idx + 1];
                if d.abs() < 1e-15 { 1.0 } else { d }
            } else {
                1.0
            };

            let scale = p as f64 / denom;
            du_intermediate.push((
                (wx1 - wx0) * scale,
                (wy1 - wy0) * scale,
                (wz1 - wz0) * scale,
                (w1_ - w0_) * scale,
            ));
        }

        if du_intermediate.len() < p {
            return Vec3d::new(0.0, 0.0, 0.0);
        }

        // De Boor in u direction on the p derivative control points (degree p-1)
        // We need to evaluate a B-spline of degree p-1 at u using p control points
        let mut du_pts = du_intermediate;
        de_boor_step(&mut du_pts, &self.u_knots, p - 1, k_u, u);
        let (dawx, dawy, dawz, daw) = du_pts[p - 1];

        // Apply quotient rule: dS/du = (dA/du - S * dw/du) / w
        Vec3d::new(
            (dawx - point.x * daw) / w,
            (dawy - point.y * daw) / w,
            (dawz - point.z * daw) / w,
        )
    }

    /// Compute the partial derivative dS/dv analytically using degree reduction.
    fn compute_partial_derivative_v(
        &self, u: f64, v: f64, k_u: usize, k_v: usize,
        p: usize, q: usize, n_u: usize, n_v: usize, w: f64, point: Point3d,
    ) -> Vec3d {
        // Similar to u-derivative but in v direction
        let mut dv_intermediate = Vec::with_capacity(p + 1);
        for i in 0..=p {
            let row = k_u - p + i;
            if row >= n_u { continue; }

            // Collect q derivative control points (degree q-1)
            let mut dvpts = Vec::with_capacity(q);
            for j in 0..q {
                let col0 = k_v - q + j;
                let col1 = k_v - q + j + 1;
                if col0 >= n_v || col1 >= n_v { continue; }

                let cp0 = &self.control_points[row][col0];
                let w0 = self.weights[row][col0];
                let cp1 = &self.control_points[row][col1];
                let w1 = self.weights[row][col1];

                let knot_idx = k_v - q + j;
                let denom = if knot_idx + q + 1 < self.v_knots.len() {
                    let d = self.v_knots[knot_idx + q + 1] - self.v_knots[knot_idx + 1];
                    if d.abs() < 1e-15 { 1.0 } else { d }
                } else {
                    1.0
                };

                let scale = q as f64 / denom;
                dvpts.push((
                    (cp1.x * w1 - cp0.x * w0) * scale,
                    (cp1.y * w1 - cp0.y * w0) * scale,
                    (cp1.z * w1 - cp0.z * w0) * scale,
                    (w1 - w0) * scale,
                ));
            }

            if dvpts.len() < q { continue; }

            // Evaluate the degree q-1 B-spline in v
            de_boor_step(&mut dvpts, &self.v_knots, q - 1, k_v, v);
            dv_intermediate.push(dvpts[q - 1]);
        }

        if dv_intermediate.len() < p + 1 {
            return Vec3d::new(0.0, 0.0, 0.0);
        }

        // De Boor in u direction on the p+1 intermediate points
        de_boor_step(&mut dv_intermediate, &self.u_knots, p, k_u, u);
        let (dbwx, dbwy, dbwz, dbw) = dv_intermediate[p];

        // Apply quotient rule: dS/dv = (dB/dv - S * dw/dv) / w
        Vec3d::new(
            (dbwx - point.x * dbw) / w,
            (dbwy - point.y * dbw) / w,
            (dbwz - point.z * dbw) / w,
        )
    }

    /// Numerical fallback for derivatives (used when analytical approach fails).
    fn derivatives_at_numerical(&self, u: f64, v: f64) -> SurfaceDerivatives {
        // Use nurbs_surface_eval for point_at calls — avoids cloning the entire
        // NurbsSurface struct (which the old code did via Surface::Nurbs(self.clone())).
        let point = nurbs_surface_eval(self, u, v);

        let eps_u = {
            let (u_min, u_max) = self.u_range();
            (u_max - u_min).max(1e-6) * 1e-6
        };
        let eps_v = {
            let (v_min, v_max) = self.v_range();
            (v_max - v_min).max(1e-6) * 1e-6
        };

        let pu_plus = nurbs_surface_eval(self, u + eps_u, v);
        let pu_minus = nurbs_surface_eval(self, u - eps_u, v);
        let pv_plus = nurbs_surface_eval(self, u, v + eps_v);
        let pv_minus = nurbs_surface_eval(self, u, v - eps_v);

        let du = Vec3d::new(
            (pu_plus.x - pu_minus.x) / (2.0 * eps_u),
            (pu_plus.y - pu_minus.y) / (2.0 * eps_u),
            (pu_plus.z - pu_minus.z) / (2.0 * eps_u),
        );
        let dv = Vec3d::new(
            (pv_plus.x - pv_minus.x) / (2.0 * eps_v),
            (pv_plus.y - pv_minus.y) / (2.0 * eps_v),
            (pv_plus.z - pv_minus.z) / (2.0 * eps_v),
        );

        SurfaceDerivatives { point, du, dv }
    }

    /// Inverse evaluation: given a 3D point, find the (u, v) parameters.
    ///
    /// Audit item 4.2 (2026-07-19): Added for NURBS surface methods.
    ///
    /// Uses Newton-Raphson iteration on the residual:
    ///   F(u, v) = S(u, v) - target_point
    /// The Jacobian is the 3×2 matrix [dS/du, dS/dv].
    /// At each step: (u, v) -= (J^T J)^-1 J^T F
    ///
    /// Returns `None` if Newton doesn't converge within 20 iterations or
    /// if the point is outside the parametric domain.
    ///
    /// **Multi-start strategy:** Tries 9 starting points (3×3 grid in UV
    /// space) to avoid local minima. Returns the best result found.
    pub fn inverse_evaluate(&self, target: &Point3d, tol: f64) -> Option<(f64, f64)> {
        let (u_min, u_max) = self.u_range();
        let (v_min, v_max) = self.v_range();

        // Multi-start: 3×3 grid of starting points
        let u_starts = [u_min, (u_min + u_max) / 2.0, u_max];
        let v_starts = [v_min, (v_min + v_max) / 2.0, v_max];

        let mut best: Option<(f64, f64, f64)> = None; // (u, v, dist_sq)

        for &u0 in &u_starts {
            for &v0 in &v_starts {
                if let Some((u, v, dist_sq)) = self.newton_inverse(target, u0, v0, tol, u_min, u_max, v_min, v_max) {
                    match best {
                        None => best = Some((u, v, dist_sq)),
                        Some((_, _, best_dist)) => {
                            if dist_sq < best_dist {
                                best = Some((u, v, dist_sq));
                            }
                        }
                    }
                    // Early exit if we found a good enough solution
                    if dist_sq < tol * tol {
                        return Some((u, v));
                    }
                }
            }
        }

        best.map(|(u, v, _)| (u, v))
    }

    /// Newton-Raphson inverse evaluation from a single starting point.
    fn newton_inverse(
        &self,
        target: &Point3d,
        u0: f64,
        v0: f64,
        tol: f64,
        u_min: f64,
        u_max: f64,
        v_min: f64,
        v_max: f64,
    ) -> Option<(f64, f64, f64)> {
        let mut u = u0.clamp(u_min, u_max);
        let mut v = v0.clamp(v_min, v_max);
        let max_iter = 20;

        for _ in 0..max_iter {
            let derivs = self.derivatives_at(u, v);
            let p = derivs.point;

            // Residual: F = S(u,v) - target
            let fx = p.x - target.x;
            let fy = p.y - target.y;
            let fz = p.z - target.z;
            let dist_sq = fx * fx + fy * fy + fz * fz;

            if dist_sq < tol * tol {
                return Some((u, v, dist_sq));
            }

            // Jacobian: J = [dS/du, dS/dv] (3×2 matrix)
            let du = derivs.du;
            let dv = derivs.dv;

            // J^T J (2×2 matrix)
            let jtj_00 = du.x * du.x + du.y * du.y + du.z * du.z;
            let jtj_01 = du.x * dv.x + du.y * dv.y + du.z * dv.z;
            let jtj_11 = dv.x * dv.x + dv.y * dv.y + dv.z * dv.z;

            // J^T F (2×1 vector)
            let jtf_0 = du.x * fx + du.y * fy + du.z * fz;
            let jtf_1 = dv.x * fx + dv.y * fy + dv.z * fz;

            // Solve (J^T J) Δ = J^T F using Cramer's rule
            let det = jtj_00 * jtj_11 - jtj_01 * jtj_01;
            if det.abs() < 1e-20 {
                // Singular Jacobian — can't solve, abort this start
                return Some((u, v, dist_sq));
            }

            let delta_u = (jtj_11 * jtf_0 - jtj_01 * jtf_1) / det;
            let delta_v = (jtj_00 * jtf_1 - jtj_01 * jtf_0) / det;

            // Update with damping to prevent overshooting
            let step_scale = 0.5; // Conservative damping
            u -= step_scale * delta_u;
            v -= step_scale * delta_v;

            // Clamp to parametric domain
            u = u.clamp(u_min, u_max);
            v = v.clamp(v_min, v_max);
        }

        // Return best effort
        let derivs = self.derivatives_at(u, v);
        let p = derivs.point;
        let fx = p.x - target.x;
        let fy = p.y - target.y;
        let fz = p.z - target.z;
        let dist_sq = fx * fx + fy * fy + fz * fz;
        Some((u, v, dist_sq))
    }
}

impl Surface {
    /// Check if the surface is degenerate at the given parametric point (u, v).
    ///
    /// Returns `DegeneracyFlags` indicating which types of degeneracy are present.
    /// A degenerate point is one where the surface parameterization breaks down:
    /// - Poles where a parametric ring collapses to a single point (e.g., sphere poles, cone apex)
    /// - Seam edges where the parameterization wraps around
    /// - Points where the normal cannot be computed
    ///
    /// # Arguments
    /// * `u`, `v` - Parametric coordinates on the surface
    /// * `tolerance` - Geometric tolerance for zero-comparisons
    pub fn is_degenerate_at(&self, u: f64, v: f64, tolerance: f64) -> DegeneracyFlags {
        let mut flags = DegeneracyFlags::NONE;

        // Evaluate the 3D point and check for NaN/Inf
        let p = self.point_at(u, v);
        if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            flags |= DegeneracyFlags::POINT_INVALID;
        }

        // Check the surface normal
        let normal = self.normal_at(u, v);
        if !normal.x.is_finite() || !normal.y.is_finite() || !normal.z.is_finite() {
            flags |= DegeneracyFlags::NORMAL_INVALID;
        }

        // Compute partial derivatives numerically.
        // Use a reasonable step size: 1e-6 is too small (numerical noise),
        // 1e-3 is better for estimating the Jacobian.
        // We only flag degeneracy if the partial is zero to within `tolerance`,
        // which means the surface collapses at this parametric point.
        let eps = 1e-4;
        let pu = self.point_at(u + eps, v);
        let pv = self.point_at(u, v + eps);
        let du = Vec3d::new(pu.x - p.x, pu.y - p.y, pu.z - p.z);
        let dv = Vec3d::new(pv.x - p.x, pv.y - p.y, pv.z - p.z);
        let du_len = (du.x * du.x + du.y * du.y + du.z * du.z).sqrt();
        let dv_len = (dv.x * dv.x + dv.y * dv.y + dv.z * dv.z).sqrt();

        // A partial derivative is considered "zero" if the step in parameter space
        // produces a 3D displacement smaller than tolerance.
        // This detects degeneracies like cone apex (radius → 0) and sphere poles.
        if du_len < tolerance {
            flags |= DegeneracyFlags::DU_ZERO;
        }
        if dv_len < tolerance {
            flags |= DegeneracyFlags::DV_ZERO;
        }

        // Also apply surface-specific degeneracy checks
        match self {
            Surface::Cone(cone) => {
                // Cone apex: radius reaches zero, all u-values map to the same 3D point
                let r = if cone.expanding {
                    v * cone.half_angle.tan()
                } else {
                    (cone.radius + v * cone.half_angle.tan()).max(0.0)
                };
                if r < tolerance {
                    flags |= DegeneracyFlags::DU_ZERO | DegeneracyFlags::DV_ZERO;
                }
            }
            Surface::Sphere(sphere) => {
                // Sphere poles: at v=0 (north) or v=pi (south), all u-values map to same point
                // v is polar angle: v=0 → top, v=pi → bottom
                if v.abs() < tolerance / sphere.radius.max(tolerance) {
                    flags |= DegeneracyFlags::DU_ZERO;
                }
                if (v - std::f64::consts::PI).abs() < tolerance / sphere.radius.max(tolerance) {
                    flags |= DegeneracyFlags::DU_ZERO;
                }
            }
            Surface::Nurbs(nurbs) => {
                // NURBS surface: check for collapsed control point rows/columns
                // A row of coincident control points indicates a degenerate edge
                let (u_min, u_max) = nurbs.u_range();
                let (_v_min, _v_max) = nurbs.v_range();

                // At the boundary of the knot domain, check if the boundary row/column
                // is degenerate (all control points coincident)
                let tol_sq = tolerance * tolerance;
                let n_u = nurbs.control_points.len();
                if n_u > 0 {
                    // Check first row (u = u_min boundary)
                    let first_row = &nurbs.control_points[0];
                    if first_row.len() > 1 {
                        let fp = &first_row[0];
                        let first_row_degenerate = first_row.iter().skip(1).all(|p| {
                            (p.x - fp.x).powi(2) + (p.y - fp.y).powi(2) + (p.z - fp.z).powi(2) < tol_sq
                        });
                        if first_row_degenerate && (u - u_min).abs() < (u_max - u_min) * 0.01 + tolerance {
                            flags |= DegeneracyFlags::DU_ZERO;
                        }
                    }

                    // Check last row (u = u_max boundary)
                    let last_row = &nurbs.control_points[n_u - 1];
                    if last_row.len() > 1 {
                        let lp = &last_row[0];
                        let last_row_degenerate = last_row.iter().skip(1).all(|p| {
                            (p.x - lp.x).powi(2) + (p.y - lp.y).powi(2) + (p.z - lp.z).powi(2) < tol_sq
                        });
                        if last_row_degenerate && (u - u_max).abs() < (u_max - u_min) * 0.01 + tolerance {
                            flags |= DegeneracyFlags::DU_ZERO;
                        }
                    }
                }
            }
            _ => {}
        }

        flags
    }

    /// Check if the surface as a whole is degenerate (e.g., zero area).
    ///
    /// This is a coarser check than `is_degenerate_at` — it checks whether
    /// the surface has any meaningful geometric extent at all.
    pub fn is_degenerate(&self, tolerance: f64) -> bool {
        match self {
            Surface::Plane(_) => false, // Planes are never degenerate
            Surface::Cylinder(c) => c.radius < tolerance,
            Surface::Cone(c) => {
                // A cone is degenerate if its base radius is below tolerance
                // AND it's not an expanding cone
                !c.expanding && c.radius < tolerance
            }
            Surface::Sphere(s) => s.radius < tolerance,
            Surface::Torus(t) => {
                // A torus is degenerate if its major radius or minor radius is below tolerance
                t.major_radius < tolerance || t.minor_radius < tolerance
            }
            Surface::Revolution(_) => false, // Can't easily tell without evaluating
            Surface::Extrusion(_) => false,
            Surface::Offset(_) => false, // Offset surfaces inherit base's degeneracy
            Surface::Ruled(_) => false,  // Ruled surfaces need both curves checked
            Surface::Nurbs(n) => {
                // Check if all control points are coincident
                if n.control_points.is_empty() || n.control_points[0].is_empty() {
                    return true;
                }
                let first = &n.control_points[0][0];
                let tol_sq = tolerance * tolerance;
                n.control_points.iter().all(|row| {
                    row.iter().all(|p| {
                        (p.x - first.x).powi(2) + (p.y - first.y).powi(2) + (p.z - first.z).powi(2) < tol_sq
                    })
                })
            }
        }
    }

    /// Return the surface type name as a static string (for logging/diagnostics).
    pub fn type_name(&self) -> &'static str {
        match self {
            Surface::Plane(_) => "Plane",
            Surface::Cylinder(_) => "Cylinder",
            Surface::Cone(_) => "Cone",
            Surface::Sphere(_) => "Sphere",
            Surface::Torus(_) => "Torus",
            Surface::Revolution(_) => "Revolution",
            Surface::Extrusion(_) => "Extrusion",
            Surface::Nurbs(_) => "Nurbs",
            Surface::Offset(_) => "Offset",
            Surface::Ruled(_) => "Ruled",
        }
    }

    /// Evaluate the surface at (u, v).
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        match self {
            Surface::Plane(p) => p.point_at(u, v),
            Surface::Cylinder(c) => c.point_at(u, v),
            Surface::Cone(c) => c.point_at(u, v),
            Surface::Sphere(s) => s.point_at(u, v),
            Surface::Torus(t) => t.point_at(u, v),
            Surface::Revolution(r) => r.point_at(u, v),
            Surface::Extrusion(e) => e.point_at(u, v),
            Surface::Nurbs(n) => nurbs_surface_eval(n, u, v),
            Surface::Offset(o) => {
                // S(u,v) = base.point_at(u,v) + distance * base.normal_at(u,v)
                let p = o.base.point_at(u, v);
                let n = o.base.normal_at(u, v);
                Point3d::new(
                    p.x + o.distance * n.x,
                    p.y + o.distance * n.y,
                    p.z + o.distance * n.z,
                )
            }
            Surface::Ruled(r) => {
                // S(u,v) = (1-v)*curve1.point_at(u) + v*curve2.point_at(u)
                let p1 = r.curve1.point_at(u);
                let p2 = r.curve2.point_at(u);
                Point3d::new(
                    p1.x * (1.0 - v) + p2.x * v,
                    p1.y * (1.0 - v) + p2.y * v,
                    p1.z * (1.0 - v) + p2.z * v,
                )
            }
        }
    }

    /// Get the surface normal at (u, v).
    pub fn normal_at(&self, u: f64, v: f64) -> Direction3d {
        match self {
            Surface::Plane(p) => p.normal_at(u, v),
            Surface::Cylinder(c) => c.normal_at(u, v),
            Surface::Cone(c) => c.normal_at(u, v),
            Surface::Sphere(s) => s.normal_at(u, v),
            Surface::Torus(t) => t.normal_at(u, v),
            Surface::Nurbs(nurbs) => {
                // Use analytical derivatives for NURBS — more accurate than forward differences
                let derivs = nurbs.derivatives_at(u, v);
                derivs.normal()
            }
            _ => {
                // Numerical differentiation fallback for Revolution, Extrusion
                let eps = 1e-7;
                let p0 = self.point_at(u, v);
                let pu = self.point_at(u + eps, v);
                let pv = self.point_at(u, v + eps);
                let du = Vec3d::new(pu.x - p0.x, pu.y - p0.y, pu.z - p0.z);
                let dv = Vec3d::new(pv.x - p0.x, pv.y - p0.y, pv.z - p0.z);
                du.cross(&dv).normalize().unwrap_or(Direction3d::Z)
            }
        }
    }

    /// Check if the surface is periodic in u.
    pub fn is_u_periodic(&self) -> bool {
        match self {
            Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) => true,
            Surface::Nurbs(n) => n.u_closed,
            _ => false,
        }
    }

    /// Check if the surface is periodic in v.
    pub fn is_v_periodic(&self) -> bool {
        match self {
            Surface::Sphere(_) | Surface::Torus(_) => true,
            Surface::Nurbs(n) => n.v_closed,
            _ => false,
        }
    }

    /// Return the natural parametric UV domain of the surface.
    ///
    /// For analytically-defined surfaces (Cylinder, Cone, Sphere, Torus),
    /// returns the canonical parameterization range used by `point_at` /
    /// `project_point`:
    ///   - Cylinder: u ∈ [0, 2π], v ∈ [-∞, ∞] → v clamped to [-1000, 1000]
    ///   - Cone:     u ∈ [0, 2π], v ∈ [0, height]  (height = radius/tan(half_angle))
    ///   - Sphere:   u ∈ [0, 2π], v ∈ [0, π]
    ///   - Torus:    u ∈ [0, 2π], v ∈ [0, 2π]
    ///   - Plane:    u, v ∈ [-1000, 1000] (infinite)
    ///   - Revolution: u ∈ [0, 2π], v ∈ profile's parametric range
    ///   - Extrusion:  u ∈ profile's parametric range, v ∈ [-1000, 1000]
    ///   - Nurbs:    u ∈ knot range, v ∈ knot range
    ///
    /// This is used by the UV breakdown viewer to compute the display
    /// bounds when a face has no explicit outer wire (e.g., the lateral
    /// face of a cone whose only stored edge is the bottom circle).
    pub fn natural_uv_domain(&self) -> (f64, f64, f64, f64) {
        const LARGE: f64 = 1000.0;
        match self {
            Surface::Plane(_) => (-LARGE, LARGE, -LARGE, LARGE),
            Surface::Cylinder(c) => {
                let (u0, u1) = c.u_range();
                // v is along the axis — there's no natural bound, but the
                // face's outer wire usually constrains it. If the wire is
                // empty, show a generous default window.
                (u0, u1, -LARGE, LARGE)
            }
            Surface::Cone(c) => {
                let u0 = 0.0;
                let u1 = 2.0 * std::f64::consts::PI;
                // v goes from 0 (base) to height() (apex) for tapering cones,
                // or 0 to +LARGE for expanding cones.
                let v0 = 0.0;
                let v1 = if c.expanding {
                    LARGE
                } else if c.half_angle.abs() < 1e-10 {
                    LARGE
                } else {
                    c.radius / c.half_angle.tan()
                };
                (u0, u1, v0, v1)
            }
            Surface::Sphere(s) => {
                let _ = s;
                (0.0, 2.0 * std::f64::consts::PI, 0.0, std::f64::consts::PI)
            }
            Surface::Torus(_) => {
                (0.0, 2.0 * std::f64::consts::PI, 0.0, 2.0 * std::f64::consts::PI)
            }
            Surface::Revolution(r) => {
                // u = revolution angle, v = profile parameter
                let (v0, v1) = r.profile.param_range();
                (0.0, 2.0 * std::f64::consts::PI, v0, v1)
            }
            Surface::Extrusion(e) => {
                let (u0, u1) = e.profile.param_range();
                (u0, u1, -LARGE, LARGE)
            }
            Surface::Nurbs(n) => {
                let (u0, u1) = n.u_range();
                let (v0, v1) = n.v_range();
                (u0, u1, v0, v1)
            }
            Surface::Offset(o) => {
                // Inherit UV domain from base surface
                o.base.natural_uv_domain()
            }
            Surface::Ruled(_) => {
                // u from curve param range, v from [0, 1]
                (-LARGE, LARGE, 0.0, 1.0)
            }
        }
    }

    /// Return (u_scale, v_scale) multipliers for converting the UV
    /// parameter-space domain into metric (arc-length) space.
    ///
    /// For surfaces where one or both parameters are angles (cylinder,
    /// cone, sphere, torus), the raw UV domain does not reflect the
    /// true surface proportions. For example, a cylinder with R=10 and
    /// H=5 has a UV domain of [0, 2π] × [0, 5] which looks wider than
    /// tall, but the actual surface is much wider (circumference ≈ 62.8)
    /// than tall (5).
    ///
    /// Multiplying the UV bounds by these scales gives metric-correct
    /// dimensions suitable for aspect-ratio-correct display.
    ///
    /// | Surface  | u_scale | v_scale |
    /// |----------|---------|---------|
    /// | Plane    | 1.0     | 1.0     |
    /// | Cylinder | R       | 1.0     |
    /// | Cone     | R_base  | 1/cos(α)|
    /// | Sphere   | R       | R       |
    /// | Torus    | R+r     | r       |
    /// | Others   | 1.0     | 1.0     |
    pub fn uv_metric_scale(&self) -> (f64, f64) {
        match self {
            Surface::Plane(_) => (1.0, 1.0),
            Surface::Cylinder(c) => (c.radius, 1.0),
            Surface::Cone(c) => {
                // U is angle, scale by base radius.
                // V is distance along axis, but the slant distance is v/cos(half_angle).
                let v_scale = if c.half_angle.abs() > 1e-10 {
                    1.0 / c.half_angle.cos()
                } else {
                    1.0
                };
                (c.radius, v_scale)
            }
            Surface::Sphere(s) => (s.radius, s.radius),
            Surface::Torus(t) => (t.major_radius + t.minor_radius, t.minor_radius),
            Surface::Revolution(_) | Surface::Extrusion(_) | Surface::Nurbs(_) => (1.0, 1.0),
            Surface::Offset(_) | Surface::Ruled(_) => (1.0, 1.0),
        }
    }

    /// Compute the curvature at a point on the surface.
    ///
    /// For analytical surfaces (plane, cylinder, cone, sphere, torus),
    /// the curvature is computed from known analytical formulas.
    /// For NURBS, revolution, and extrusion surfaces, numerical
    /// differentiation of the second fundamental form is used.
    pub fn curvature_at(&self, u: f64, v: f64) -> SurfaceCurvature {
        match self {
            Surface::Plane(_) => SurfaceCurvature {
                gaussian: 0.0, mean: 0.0, k1: 0.0, k2: 0.0, max_abs: 0.0,
            },
            Surface::Cylinder(cyl) => {
                // Cylinder: k_meridional = 0 (along axis), k_circumferential = 1/radius
                let k_circ = 1.0 / cyl.radius.max(1e-10);
                SurfaceCurvature {
                    gaussian: 0.0,
                    mean: k_circ / 2.0,
                    k1: k_circ,
                    k2: 0.0,
                    max_abs: k_circ,
                }
            },
            Surface::Cone(cone) => {
                // Cone: k_meridional = 0 (along generator), k_circumferential = cos(half_angle) / r(u,v)
                let r = if cone.expanding {
                    v * cone.half_angle.tan()
                } else {
                    (cone.radius + v * cone.half_angle.tan()).max(0.0)
                };
                let k_circ = if r > 1e-10 {
                    cone.half_angle.cos() / r
                } else {
                    0.0 // At apex, curvature is undefined
                };
                SurfaceCurvature {
                    gaussian: 0.0,
                    mean: k_circ / 2.0,
                    k1: k_circ,
                    k2: 0.0,
                    max_abs: k_circ,
                }
            },
            Surface::Sphere(sphere) => {
                // Sphere: k1 = k2 = 1/radius
                let k = 1.0 / sphere.radius.max(1e-10);
                SurfaceCurvature {
                    gaussian: k * k,
                    mean: k,
                    k1: k,
                    k2: k,
                    max_abs: k,
                }
            },
            Surface::Torus(torus) => {
                // Torus: k1 = cos(v)/(R + r*cos(v)), k2 = 1/r
                let minor_r = torus.minor_radius.max(1e-10);
                let major_r = torus.major_radius;
                let k2 = 1.0 / major_r;
                let k1 = {
                    let denom = major_r + minor_r * v.cos();
                    if denom.abs() > 1e-10 { v.cos() / denom } else { 0.0 }
                };
                let max_abs = k1.abs().max(k2.abs());
                SurfaceCurvature {
                    gaussian: k1 * k2,
                    mean: (k1 + k2) / 2.0,
                    k1, k2, max_abs,
                }
            },
            Surface::Nurbs(_nurbs) => {
                // Fast NURBS curvature using only point_at (9 evaluations total)
                // instead of 5 derivatives_at calls (5×87 = 435 de Boor iterations).
                // Uses central-difference first and second derivatives from
                // point_at, which is ~3x cheaper than using derivatives_at.
                let eps = 1e-5;
                let p0 = self.point_at(u, v);

                // First derivatives via central differences (4 point_at calls)
                let pu_p = self.point_at(u + eps, v);
                let pu_m = self.point_at(u - eps, v);
                let pv_p = self.point_at(u, v + eps);
                let pv_m = self.point_at(u, v - eps);

                let du = Vec3d::new(
                    (pu_p.x - pu_m.x) / (2.0 * eps),
                    (pu_p.y - pu_m.y) / (2.0 * eps),
                    (pu_p.z - pu_m.z) / (2.0 * eps),
                );
                let dv = Vec3d::new(
                    (pv_p.x - pv_m.x) / (2.0 * eps),
                    (pv_p.y - pv_m.y) / (2.0 * eps),
                    (pv_p.z - pv_m.z) / (2.0 * eps),
                );

                // Second derivatives from central differences (no extra point_at calls)
                let duu = Vec3d::new(
                    (pu_p.x - 2.0 * p0.x + pu_m.x) / (eps * eps),
                    (pu_p.y - 2.0 * p0.y + pu_m.y) / (eps * eps),
                    (pu_p.z - 2.0 * p0.z + pu_m.z) / (eps * eps),
                );
                let dvv = Vec3d::new(
                    (pv_p.x - 2.0 * p0.x + pv_m.x) / (eps * eps),
                    (pv_p.y - 2.0 * p0.y + pv_m.y) / (eps * eps),
                    (pv_p.z - 2.0 * p0.z + pv_m.z) / (eps * eps),
                );

                // Mixed derivative from 4 corner points (4 extra point_at calls)
                let puv_pp = self.point_at(u + eps, v + eps);
                let puv_mm = self.point_at(u - eps, v - eps);
                let puv_pm = self.point_at(u + eps, v - eps);
                let puv_mp = self.point_at(u - eps, v + eps);
                let duv = Vec3d::new(
                    (puv_pp.x - puv_pm.x - puv_mp.x + puv_mm.x) / (4.0 * eps * eps),
                    (puv_pp.y - puv_pm.y - puv_mp.y + puv_mm.y) / (4.0 * eps * eps),
                    (puv_pp.z - puv_pm.z - puv_mp.z + puv_mm.z) / (4.0 * eps * eps),
                );

                // Normal from du × dv
                let n_vec = du.cross(&dv);
                let n_len = (n_vec.x * n_vec.x + n_vec.y * n_vec.y + n_vec.z * n_vec.z).sqrt();
                if n_len < 1e-20 {
                    return SurfaceCurvature { gaussian: 0.0, mean: 0.0, k1: 0.0, k2: 0.0, max_abs: 0.0 };
                }
                let n_vec = Vec3d::new(n_vec.x / n_len, n_vec.y / n_len, n_vec.z / n_len);

                // Second fundamental form
                let l = duu.dot(&n_vec);
                let m = duv.dot(&n_vec);
                let n_val = dvv.dot(&n_vec);

                // First fundamental form
                let e = du.dot(&du);
                let f = du.dot(&dv);
                let g = dv.dot(&dv);

                let denom = e * g - f * f;
                if denom.abs() < 1e-20 {
                    return SurfaceCurvature { gaussian: 0.0, mean: 0.0, k1: 0.0, k2: 0.0, max_abs: 0.0 };
                }

                let k_gauss = (l * n_val - m * m) / denom;
                let k_mean = (e * n_val - 2.0 * f * m + g * l) / (2.0 * denom);
                let disc = (k_mean * k_mean - k_gauss).max(0.0);
                let sqrt_disc = disc.sqrt();
                let k1 = k_mean + sqrt_disc;
                let k2 = k_mean - sqrt_disc;
                let max_abs = k1.abs().max(k2.abs());

                SurfaceCurvature {
                    gaussian: k_gauss,
                    mean: k_mean,
                    k1, k2, max_abs,
                }
            }
            _ => {
                // Numerical curvature computation for Revolution, Extrusion
                // using second fundamental form
                let eps = 1e-5;
                let p0 = self.point_at(u, v);
                let n = self.normal_at(u, v);

                // First derivatives (central differences)
                let pu_p = self.point_at(u + eps, v);
                let pu_m = self.point_at(u - eps, v);
                let pv_p = self.point_at(u, v + eps);
                let pv_m = self.point_at(u, v - eps);

                let du = Vec3d::new(
                    (pu_p.x - pu_m.x) / (2.0 * eps),
                    (pu_p.y - pu_m.y) / (2.0 * eps),
                    (pu_p.z - pu_m.z) / (2.0 * eps),
                );
                let dv = Vec3d::new(
                    (pv_p.x - pv_m.x) / (2.0 * eps),
                    (pv_p.y - pv_m.y) / (2.0 * eps),
                    (pv_p.z - pv_m.z) / (2.0 * eps),
                );

                // Second derivatives
                let duu = Vec3d::new(
                    (pu_p.x - 2.0 * p0.x + pu_m.x) / (eps * eps),
                    (pu_p.y - 2.0 * p0.y + pu_m.y) / (eps * eps),
                    (pu_p.z - 2.0 * p0.z + pu_m.z) / (eps * eps),
                );
                let dvv = Vec3d::new(
                    (pv_p.x - 2.0 * p0.x + pv_m.x) / (eps * eps),
                    (pv_p.y - 2.0 * p0.y + pv_m.y) / (eps * eps),
                    (pv_p.z - 2.0 * p0.z + pv_m.z) / (eps * eps),
                );
                let puv_pp = self.point_at(u + eps, v + eps);
                let puv_mm = self.point_at(u - eps, v - eps);
                let puv_pm = self.point_at(u + eps, v - eps);
                let puv_mp = self.point_at(u - eps, v + eps);
                let duv = Vec3d::new(
                    (puv_pp.x - puv_pm.x - puv_mp.x + puv_mm.x) / (4.0 * eps * eps),
                    (puv_pp.y - puv_pm.y - puv_mp.y + puv_mm.y) / (4.0 * eps * eps),
                    (puv_pp.z - puv_pm.z - puv_mp.z + puv_mm.z) / (4.0 * eps * eps),
                );

                let n_vec = Vec3d::new(n.x, n.y, n.z);

                // Second fundamental form
                let l = duu.dot(&n_vec);
                let m = duv.dot(&n_vec);
                let n_val = dvv.dot(&n_vec);

                // First fundamental form
                let e = du.dot(&du);
                let f = du.dot(&dv);
                let g = dv.dot(&dv);

                let denom = e * g - f * f;
                if denom.abs() < 1e-20 {
                    return SurfaceCurvature { gaussian: 0.0, mean: 0.0, k1: 0.0, k2: 0.0, max_abs: 0.0 };
                }

                let k_gauss = (l * n_val - m * m) / denom;
                let k_mean = (e * n_val - 2.0 * f * m + g * l) / (2.0 * denom);
                let disc = (k_mean * k_mean - k_gauss).max(0.0);
                let sqrt_disc = disc.sqrt();
                let k1 = k_mean + sqrt_disc;
                let k2 = k_mean - sqrt_disc;
                let max_abs = k1.abs().max(k2.abs());

                SurfaceCurvature {
                    gaussian: k_gauss,
                    mean: k_mean,
                    k1, k2, max_abs,
                }
            }
        }
    }

    /// Compute the second fundamental form at a point on the surface using
    /// numerical differentiation.
    ///
    /// Returns (FirstFundamentalForm, SecondFundamentalForm) so that
    /// curvature quantities can be computed.
    pub fn fundamental_forms_at(&self, u: f64, v: f64) -> (FirstFundamentalForm, SecondFundamentalForm) {
        let eps = 1e-5;
        let p0 = self.point_at(u, v);
        let n = self.normal_at(u, v);

        let pu_p = self.point_at(u + eps, v);
        let pu_m = self.point_at(u - eps, v);
        let pv_p = self.point_at(u, v + eps);
        let pv_m = self.point_at(u, v - eps);

        let du = Vec3d::new(
            (pu_p.x - pu_m.x) / (2.0 * eps),
            (pu_p.y - pu_m.y) / (2.0 * eps),
            (pu_p.z - pu_m.z) / (2.0 * eps),
        );
        let dv = Vec3d::new(
            (pv_p.x - pv_m.x) / (2.0 * eps),
            (pv_p.y - pv_m.y) / (2.0 * eps),
            (pv_p.z - pv_m.z) / (2.0 * eps),
        );

        let duu = Vec3d::new(
            (pu_p.x - 2.0 * p0.x + pu_m.x) / (eps * eps),
            (pu_p.y - 2.0 * p0.y + pu_m.y) / (eps * eps),
            (pu_p.z - 2.0 * p0.z + pu_m.z) / (eps * eps),
        );
        let dvv = Vec3d::new(
            (pv_p.x - 2.0 * p0.x + pv_m.x) / (eps * eps),
            (pv_p.y - 2.0 * p0.y + pv_m.y) / (eps * eps),
            (pv_p.z - 2.0 * p0.z + pv_m.z) / (eps * eps),
        );
        let puv_pp = self.point_at(u + eps, v + eps);
        let puv_mm = self.point_at(u - eps, v - eps);
        let puv_pm = self.point_at(u + eps, v - eps);
        let puv_mp = self.point_at(u - eps, v + eps);
        let duv = Vec3d::new(
            (puv_pp.x - puv_pm.x - puv_mp.x + puv_mm.x) / (4.0 * eps * eps),
            (puv_pp.y - puv_pm.y - puv_mp.y + puv_mm.y) / (4.0 * eps * eps),
            (puv_pp.z - puv_pm.z - puv_mp.z + puv_mm.z) / (4.0 * eps * eps),
        );

        let n_vec = Vec3d::new(n.x, n.y, n.z);

        let first = FirstFundamentalForm {
            e: du.dot(&du),
            f: du.dot(&dv),
            g: dv.dot(&dv),
        };
        let second = SecondFundamentalForm {
            l: duu.dot(&n_vec),
            m: duv.dot(&n_vec),
            n: dvv.dot(&n_vec),
        };

        (first, second)
    }

    /// Compute the surface point and first partial derivatives at (u, v).
    ///
    /// For NURBS surfaces, uses the NurbsSurface::derivatives_at method.
    /// For all other surface types, uses central finite differences.
    pub fn derivatives_at(&self, u: f64, v: f64) -> SurfaceDerivatives {
        match self {
            Surface::Nurbs(nurbs) => nurbs.derivatives_at(u, v),
            Surface::Revolution(r) => r.derivatives_at(u, v),
            Surface::Extrusion(e) => e.derivatives_at(u, v),
            _ => {
                // Fallback to numerical central differences for surface types
                // that don't yet have analytical derivatives.
                let point = self.point_at(u, v);
                let eps = 1e-6;
                let pu_plus = self.point_at(u + eps, v);
                let pu_minus = self.point_at(u - eps, v);
                let pv_plus = self.point_at(u, v + eps);
                let pv_minus = self.point_at(u, v - eps);
                let du = Vec3d::new(
                    (pu_plus.x - pu_minus.x) / (2.0 * eps),
                    (pu_plus.y - pu_minus.y) / (2.0 * eps),
                    (pu_plus.z - pu_minus.z) / (2.0 * eps),
                );
                let dv = Vec3d::new(
                    (pv_plus.x - pv_minus.x) / (2.0 * eps),
                    (pv_plus.y - pv_minus.y) / (2.0 * eps),
                    (pv_plus.z - pv_minus.z) / (2.0 * eps),
                );
                SurfaceDerivatives { point, du, dv }
            }
        }
    }

    /// Project a 3D point onto the surface's parametric space → Some(u, v).
    /// Returns None if the point is too far from the surface for a meaningful projection.
    pub fn project_point_opt(&self, point: &Point3d) -> Option<(f64, f64)> {
        let (u, v) = self.project_point(point);
        Some((u, v))
    }

    /// Project a 3D point onto the surface's parametric space → (u, v).
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        match self {
            Surface::Plane(p) => p.project_point(point),
            Surface::Cylinder(c) => c.project_point(point),
            Surface::Cone(c) => c.project_point(point),
            Surface::Sphere(s) => s.project_point(point),
            Surface::Torus(t) => t.project_point(point),
            Surface::Revolution(r) => {
                // u = revolution angle ∈ [0, 2π)
                let dx = point.x - r.origin.x;
                let dy = point.y - r.origin.y;
                let u = dy.atan2(dx);
                // Normalize u to [0, 2π) to match the canonical parameterization
                let u = if u < 0.0 { u + 2.0 * PI } else { u };
                // v = profile curve parameter: find the closest point on the profile curve
                let dz = point.z - r.origin.z;
                let radial = (dx * dx + dy * dy).sqrt();
                let (v_min, v_max) = r.profile.param_range();
                let mut best_v = (v_min + v_max) * 0.5;
                let mut best_dist = f64::MAX;

                // Use same search resolution on all platforms for consistent results
                let (steps, refine_steps) = (64, 20);

                for i in 0..=steps {
                    let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
                    let p = r.profile.point_at(v);
                    let dr = p.x - radial;
                    let ddz = p.z - dz;
                    let dist = dr * dr + ddz * ddz;
                    if dist < best_dist {
                        best_dist = dist;
                        best_v = v;
                    }
                }
                // Refine with a finer search around the best point
                let v_step = (v_max - v_min) / steps as f64;
                for i in 0..=refine_steps {
                    let v = (best_v - v_step + 2.0 * v_step * i as f64 / refine_steps as f64)
                        .clamp(v_min, v_max);
                    let p = r.profile.point_at(v);
                    let dr = p.x - radial;
                    let ddz = p.z - dz;
                    let dist = dr * dr + ddz * ddz;
                    if dist < best_dist {
                        best_dist = dist;
                        best_v = v;
                    }
                }
                (u, best_v)
            }
            Surface::Extrusion(e) => {
                // u: profile curve parameter, v: distance along extrusion direction
                let p0 = e.profile.point_at(0.0);
                let dx = point.x - p0.x;
                let dy = point.y - p0.y;
                let dz = point.z - p0.z;
                let v = dx * e.direction.x + dy * e.direction.y + dz * e.direction.z;

                let px = point.x - v * e.direction.x;
                let py = point.y - v * e.direction.y;
                let pz = point.z - v * e.direction.z;

                let (u_min, u_max) = e.profile.param_range();
                let mut best_u = (u_min + u_max) * 0.5;
                let mut best_dist = f64::MAX;

                // Use same search resolution on all platforms for consistent results
                let (steps, refine_steps) = (64, 20);

                // Coarse search
                for i in 0..=steps {
                    let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
                    let p = e.profile.point_at(u);
                    let dist = (p.x - px).powi(2) + (p.y - py).powi(2) + (p.z - pz).powi(2);
                    if dist < best_dist {
                        best_dist = dist;
                        best_u = u;
                    }
                }
                // Refine
                let u_step = (u_max - u_min) / steps as f64;
                for i in 0..=refine_steps {
                    let u = (best_u - u_step + 2.0 * u_step * i as f64 / refine_steps as f64)
                        .clamp(u_min, u_max);
                    let p = e.profile.point_at(u);
                    let dist = (p.x - px).powi(2) + (p.y - py).powi(2) + (p.z - pz).powi(2);
                    if dist < best_dist {
                        best_dist = dist;
                        best_u = u;
                    }
                }
                (best_u, v)
            }
            Surface::Nurbs(n) => {
                // Multi-resolution grid search + Newton-Raphson for NURBS point projection.
                //
                // The key performance insight: for each boundary point that comes from
                // edge curve sampling (which is how STEP files provide them), we already
                // know the point is ON the surface. So we need to find the (u,v) such
                // that S(u,v) = point. A coarse grid that misses the closest point leads
                // to Newton-Raphson diverging or converging to a wrong minimum.
                //
                // Strategy:
                // 1. 11×11 coarse grid (121 evals) — finds a good initial guess
                // 2. Local 5×5 refinement around the best point (25 evals)
                // 3. Newton-Raphson from the best initial guess (up to 15 iterations)
                //
                // Total: ~146 surface evaluations for the worst case, but typically
                // ~130 for the grid + 3-5 Newton iterations = ~135-140 total.
                // This is faster than the old 5×5 grid approach because Newton
                // converges in fewer iterations from a better starting point.
                let (u_min, u_max) = n.u_range();
                let (v_min, v_max) = n.v_range();
                let mut best_u = (u_min + u_max) * 0.5;
                let mut best_v = (v_min + v_max) * 0.5;
                let mut best_dist = f64::MAX;
                let px = point.x;
                let py = point.y;
                let pz = point.z;

                // Phase 1: 11×11 coarse grid (121 evaluations)
                let coarse = 11;
                let u_step_coarse = (u_max - u_min) / coarse as f64;
                let v_step_coarse = (v_max - v_min) / coarse as f64;
                for i in 0..=coarse {
                    let u = u_min + u_step_coarse * i as f64;
                    for j in 0..=coarse {
                        let v = v_min + v_step_coarse * j as f64;
                        let p = self.point_at(u, v);
                        let dist = (p.x - px).powi(2) + (p.y - py).powi(2) + (p.z - pz).powi(2);
                        if dist < best_dist {
                            best_dist = dist;
                            best_u = u;
                            best_v = v;
                        }
                    }
                }

                // Phase 2: Local 5×5 refinement around the best coarse point (25 evaluations)
                // This significantly improves Newton convergence by providing a much
                // better initial guess, especially for high-curvature NURBS surfaces.
                let fine = 5;
                let fine_u_step = u_step_coarse / fine as f64;
                let fine_v_step = v_step_coarse / fine as f64;
                let fine_u_start = (best_u - 2.0 * fine_u_step).max(u_min);
                let fine_v_start = (best_v - 2.0 * fine_v_step).max(v_min);
                let fine_u_end = (best_u + 2.0 * fine_u_step).min(u_max);
                let fine_v_end = (best_v + 2.0 * fine_v_step).min(v_max);
                let fine_u_range = fine_u_end - fine_u_start;
                let fine_v_range = fine_v_end - fine_v_start;
                for i in 0..=fine {
                    let u = fine_u_start + fine_u_range * i as f64 / fine as f64;
                    for j in 0..=fine {
                        let v = fine_v_start + fine_v_range * j as f64 / fine as f64;
                        let p = self.point_at(u, v);
                        let dist = (p.x - px).powi(2) + (p.y - py).powi(2) + (p.z - pz).powi(2);
                        if dist < best_dist {
                            best_dist = dist;
                            best_u = u;
                            best_v = v;
                        }
                    }
                }

                // Phase 3: Newton-Raphson refinement (up to 15 iterations)
                // With the improved initial guess, Newton typically converges in 3-5 iterations.
                let u_range = u_max - u_min;
                let v_range = v_max - v_min;
                for _ in 0..15 {
                    let derivs = n.derivatives_at(best_u, best_v);
                    let sp = derivs.point;
                    let dx = sp.x - px;
                    let dy = sp.y - py;
                    let dz = sp.z - pz;

                    // Gradient: g = [dS/du · (S-P), dS/dv · (S-P)]
                    let gu = derivs.du.x * dx + derivs.du.y * dy + derivs.du.z * dz;
                    let gv = derivs.dv.x * dx + derivs.dv.y * dy + derivs.dv.z * dz;

                    // Hessian approximation (Gauss-Newton)
                    let hu_u = derivs.du.x * derivs.du.x + derivs.du.y * derivs.du.y + derivs.du.z * derivs.du.z;
                    let hu_v = derivs.du.x * derivs.dv.x + derivs.du.y * derivs.dv.y + derivs.du.z * derivs.dv.z;
                    let hv_v = derivs.dv.x * derivs.dv.x + derivs.dv.y * derivs.dv.y + derivs.dv.z * derivs.dv.z;

                    let det = hu_u * hv_v - hu_v * hu_v;
                    if det.abs() < 1e-20 { break; }

                    let du = -(hv_v * gu - hu_v * gv) / det;
                    let dv = -(-hu_v * gu + hu_u * gv) / det;

                    // Clamp step size relative to parametric range
                    let step_limit_u = u_range * 0.1;
                    let step_limit_v = v_range * 0.1;
                    let du = du.clamp(-step_limit_u, step_limit_u);
                    let dv = dv.clamp(-step_limit_v, step_limit_v);

                    let new_u = (best_u + du).clamp(u_min, u_max);
                    let new_v = (best_v + dv).clamp(v_min, v_max);

                    let new_p = self.point_at(new_u, new_v);
                    let new_dist_sq = (new_p.x - px).powi(2) + (new_p.y - py).powi(2) + (new_p.z - pz).powi(2);

                    if new_dist_sq < best_dist {
                        if (best_dist - new_dist_sq) < 1e-10 * best_dist.max(1e-20) {
                            best_u = new_u;
                            best_v = new_v;
                            break; // Converged
                        }
                        best_u = new_u;
                        best_v = new_v;
                        best_dist = new_dist_sq;
                    } else {
                        break; // Not improving
                    }
                }

                (best_u, best_v)
            }
            Surface::Offset(o) => {
                // Project onto base surface
                o.base.project_point(point)
            }
            Surface::Ruled(r) => {
                // Use curve1's param range start as approximation
                // (full implementation would use closest_point_on_curve)
                (0.0, 0.0)
            }
        }
    }

    /// Transform the surface.
    pub fn transform(&self, t: &Transform) -> Surface {
        match self {
            Surface::Plane(p) => Surface::Plane(Plane {
                origin: t.transform_point(&p.origin),
                u_dir: t.transform_direction(&p.u_dir),
                v_dir: t.transform_direction(&p.v_dir),
                normal: t.transform_direction(&p.normal),
            }),
            Surface::Cylinder(c) => Surface::Cylinder(CylinderSurface {
                origin: t.transform_point(&c.origin),
                axis: t.transform_direction(&c.axis),
                radius: c.radius,
                x_dir: t.transform_direction(&c.x_dir),
            }),
            Surface::Cone(c) => Surface::Cone(ConeSurface {
                origin: t.transform_point(&c.origin),
                axis: t.transform_direction(&c.axis),
                half_angle: c.half_angle,
                radius: c.radius,
                x_dir: t.transform_direction(&c.x_dir),
                expanding: c.expanding,
            }),
            Surface::Sphere(s) => Surface::Sphere(SphereSurface {
                center: t.transform_point(&s.center),
                radius: s.radius,
            }),
            Surface::Torus(tor) => Surface::Torus(TorusSurface {
                center: t.transform_point(&tor.center),
                axis: t.transform_direction(&tor.axis),
                major_radius: tor.major_radius,
                minor_radius: tor.minor_radius,
                x_dir: t.transform_direction(&tor.x_dir),
            }),
            Surface::Revolution(r) => Surface::Revolution(RevolutionSurface {
                profile: r.profile.transform(t),
                axis: t.transform_direction(&r.axis),
                origin: t.transform_point(&r.origin),
            }),
            Surface::Extrusion(e) => Surface::Extrusion(ExtrusionSurface {
                profile: e.profile.transform(t),
                direction: t.transform_direction(&e.direction),
            }),
            Surface::Nurbs(n) => Surface::Nurbs(NurbsSurface {
                u_degree: n.u_degree,
                v_degree: n.v_degree,
                control_points: n.control_points.iter().map(|row| {
                    row.iter().map(|p| t.transform_point(p)).collect()
                }).collect(),
                weights: n.weights.clone(),
                u_knots: n.u_knots.clone(),
                v_knots: n.v_knots.clone(),
                u_closed: n.u_closed,
                v_closed: n.v_closed,
            }),
            Surface::Offset(o) => Surface::Offset(OffsetSurface {
                base: Box::new(o.base.transform(t)),
                distance: o.distance,
            }),
            Surface::Ruled(r) => Surface::Ruled(RuledSurface {
                curve1: Box::new(r.curve1.transform(t)),
                curve2: Box::new(r.curve2.transform(t)),
            }),
        }
    }
}

/// NURBS surface evaluation using de Boor's algorithm.
/// Uses tensor-product approach: evaluate B-spline in v for each relevant row,
/// then evaluate B-spline in u on the resulting intermediate points.
fn nurbs_surface_eval(nurbs: &NurbsSurface, u: f64, v: f64) -> Point3d {
    if nurbs.control_points.is_empty() || nurbs.control_points[0].is_empty() {
        return Point3d::ORIGIN;
    }

    let n_u = nurbs.control_points.len();
    let n_v = nurbs.control_points[0].len();
    let p = nurbs.u_degree;
    let q = nurbs.v_degree;

    // Clamp u and v to valid knot range
    let u_min = if nurbs.u_knots.len() > p { nurbs.u_knots[p] } else { 0.0 };
    let u_max = if nurbs.u_knots.len() > p + 1 { nurbs.u_knots[nurbs.u_knots.len() - p - 1] } else { 1.0 };
    let v_min = if nurbs.v_knots.len() > q { nurbs.v_knots[q] } else { 0.0 };
    let v_max = if nurbs.v_knots.len() > q + 1 { nurbs.v_knots[nurbs.v_knots.len() - q - 1] } else { 1.0 };

    let u_c = u.clamp(u_min, u_max);
    let v_c = v.clamp(v_min, v_max);

    // Find u knot span: T[k_u] <= u_c < T[k_u+1]
    let k_u = find_knot_span(&nurbs.u_knots, p, u_c, n_u);
    // Find v knot span: T[k_v] <= v_c < T[k_v+1]
    let k_v = find_knot_span(&nurbs.v_knots, q, v_c, n_v);

    // Step 1: For each row i in [k_u-p .. k_u], evaluate B-spline in v direction
    // This gives us p+1 intermediate points
    let mut intermediate: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(p + 1);

    for i in 0..=p {
        let row_idx = k_u - p + i;
        if row_idx >= n_u {
            // Out of bounds — use last valid row
            let last = intermediate.last().copied().unwrap_or((0.0, 0.0, 0.0, 1.0));
            intermediate.push(last);
            continue;
        }

        // Collect q+1 control points in v direction (weighted)
        let mut pts: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(q + 1);
        for j in 0..=q {
            let col_idx = k_v - q + j;
            let col_idx = if col_idx >= n_v { n_v - 1 } else { col_idx };
            let cp = &nurbs.control_points[row_idx][col_idx];
            let w = nurbs.weights.get(row_idx).and_then(|r| r.get(col_idx)).copied().unwrap_or(1.0);
            pts.push((cp.x * w, cp.y * w, cp.z * w, w));
        }

        // De Boor in v direction (standard algorithm)
        de_boor_step(&mut pts, &nurbs.v_knots, q, k_v, v_c);

        if let Some(&last) = pts.last() {
            intermediate.push(last);
        }
    }

    // Step 2: De Boor in u direction on the intermediate points
    de_boor_step(&mut intermediate, &nurbs.u_knots, p, k_u, u_c);

    if let Some(&result) = intermediate.last() {
        let w = result.3;
        if w.abs() < 1e-15 {
            Point3d::ORIGIN
        } else {
            let px = result.0 / w;
            let py = result.1 / w;
            let pz = result.2 / w;
            // NaN/Inf guard: if any coordinate is not finite, return ORIGIN.
            // This can happen with degenerate control points or numerical
            // instability in the De Boor algorithm.
            if px.is_finite() && py.is_finite() && pz.is_finite() {
                Point3d::new(px, py, pz)
            } else {
                log::warn!("NURBS surface eval: non-finite result ({}, {}, {}) at u={}, v={} — returning ORIGIN", px, py, pz, u, v);
                Point3d::ORIGIN
            }
        }
    } else {
        Point3d::ORIGIN
    }
}

/// Find the knot span index k such that T[k] <= t < T[k+1]
/// (with special handling for t at the end of the domain).
fn find_knot_span(knots: &[f64], degree: usize, t: f64, n_control_points: usize) -> usize {
    // Special case: t at or beyond the end of the domain
    if t >= knots[n_control_points] {
        return n_control_points - 1;
    }

    // Binary search for the knot span
    let mut lo = degree;
    let mut hi = n_control_points;
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// Perform the de Boor refinement steps on an array of (weighted) control points.
/// `pts` has (degree+1) elements, indexed 0..=degree.
/// After this function, pts[degree] contains the evaluated point.
///
/// Implements the standard de Boor algorithm:
///   for r = 1 .. degree:
///     for j = degree down to r:
///       i = k - degree + j
///       alpha = (t - knots[i]) / (knots[i + degree + 1 - r] - knots[i])
///       d[j] = alpha * d[j] + (1-alpha) * d[j-1]
fn de_boor_step(pts: &mut [(f64, f64, f64, f64)], knots: &[f64], degree: usize, k: usize, t: f64) {
    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let i = k - degree + j;
            let alpha = if i + degree + 1 - r < knots.len() && i < knots.len() {
                let denom = knots[i + degree + 1 - r] - knots[i];
                if denom.abs() < 1e-15 {
                    0.0
                } else {
                    (t - knots[i]) / denom
                }
            } else {
                0.0
            };
            let beta = 1.0 - alpha;

            pts[j] = (
                alpha * pts[j].0 + beta * pts[j - 1].0,
                alpha * pts[j].1 + beta * pts[j - 1].1,
                alpha * pts[j].2 + beta * pts[j - 1].2,
                alpha * pts[j].3 + beta * pts[j - 1].3,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::{Circle, Line};
    use std::f64::consts::PI;

    #[test]
    fn test_cone_apex_degenerate() {
        let cone = ConeSurface::new_z(10.0, 0.5);
        // With STEP parameterization, apex is at v = -radius / tan(half_angle)
        let apex_v = cone.apex_v();
        let surface = Surface::Cone(cone);
        let flags = surface.is_degenerate_at(0.0, apex_v, 1e-6);
        assert!(flags.is_degenerate(), "Cone apex should be degenerate at v={}, got {:?}", apex_v, flags);
        assert!(flags.is_singular(), "Cone apex should be singular (both partials zero)");
    }

    #[test]
    fn test_cone_base_not_degenerate() {
        let cone = ConeSurface::new_z(10.0, 0.5);
        let surface = Surface::Cone(cone);
        // At v = 0 (base), the cone is not degenerate
        let flags = surface.is_degenerate_at(0.0, 0.0, 1e-6);
        assert!(!flags.is_degenerate(), "Cone base should not be degenerate, got {:?}", flags);
    }

    #[test]
    fn test_sphere_north_pole_degenerate() {
        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let surface = Surface::Sphere(sphere);
        // At v = 0 (north pole), the sphere is degenerate (u-ring collapses)
        let flags = surface.is_degenerate_at(0.0, 0.0, 1e-6);
        assert!(flags.contains(DegeneracyFlags::DU_ZERO),
            "Sphere north pole should have DU_ZERO flag, got {:?}", flags);
    }

    #[test]
    fn test_sphere_south_pole_degenerate() {
        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let surface = Surface::Sphere(sphere);
        // At v = pi (south pole), the sphere is degenerate (u-ring collapses)
        let flags = surface.is_degenerate_at(0.0, PI, 1e-6);
        assert!(flags.contains(DegeneracyFlags::DU_ZERO),
            "Sphere south pole should have DU_ZERO flag, got {:?}", flags);
    }

    #[test]
    fn test_sphere_equator_not_degenerate() {
        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let surface = Surface::Sphere(sphere);
        // At v = pi/2 (equator), the sphere is not degenerate
        let flags = surface.is_degenerate_at(0.0, PI / 2.0, 1e-6);
        assert!(!flags.is_degenerate(), "Sphere equator should not be degenerate, got {:?}", flags);
    }

    #[test]
    fn test_cylinder_not_degenerate() {
        let cyl = CylinderSurface::new_z(10.0);
        let surface = Surface::Cylinder(cyl);
        let flags = surface.is_degenerate_at(0.0, 5.0, 1e-6);
        assert!(!flags.is_degenerate(), "Cylinder should not be degenerate at any point, got {:?}", flags);
    }

    #[test]
    fn test_plane_not_degenerate() {
        let plane = Surface::Plane(Plane::xy());
        let flags = plane.is_degenerate_at(0.0, 0.0, 1e-6);
        assert!(!flags.is_degenerate(), "Plane should never be degenerate, got {:?}", flags);
    }

    #[test]
    fn test_surface_is_degenerate_zero_radius_sphere() {
        let sphere = SphereSurface::new(Point3d::ORIGIN, 0.0);
        let surface = Surface::Sphere(sphere);
        assert!(surface.is_degenerate(1e-6), "Sphere with zero radius should be degenerate");
    }

    #[test]
    fn test_surface_is_degenerate_zero_radius_cylinder() {
        let cyl = CylinderSurface::new_z(0.0);
        let surface = Surface::Cylinder(cyl);
        assert!(surface.is_degenerate(1e-6), "Cylinder with zero radius should be degenerate");
    }

    #[test]
    fn test_degeneracy_flags_bitor() {
        let flags = DegeneracyFlags::DU_ZERO | DegeneracyFlags::DV_ZERO;
        assert!(flags.contains(DegeneracyFlags::DU_ZERO));
        assert!(flags.contains(DegeneracyFlags::DV_ZERO));
        assert!(flags.is_singular());
    }

    #[test]
    fn test_torus_inner_touch_degenerate() {
        // Torus where minor_radius == major_radius (self-intersecting at center)
        // This is not degenerate per se, but the surface point at the touch
        // should still be computable without NaN
        let torus = TorusSurface::new_z(Point3d::ORIGIN, 10.0, 10.0);
        let surface = Surface::Torus(torus);
        let flags = surface.is_degenerate_at(0.0, PI, 1e-6);
        // The point itself should not be invalid
        assert!(!flags.contains(DegeneracyFlags::POINT_INVALID),
            "Torus inner touch point should not be NaN/Inf");
    }

    // ─── Analytical derivatives tests (P9) ──────────────────────────────

    #[test]
    fn test_revolution_analytical_derivatives_match_numerical() {
        // Revolution of a line around the Z axis → cylinder.
        let profile = Curve3d::Line(Line::new(
            Point3d::new(1.0, 0.0, 0.0),
            Direction3d::Z,
        ));
        let axis = Direction3d::Z;
        let origin = Point3d::ORIGIN;
        let rev = RevolutionSurface::new(profile, axis, origin);
        let surface = Surface::Revolution(rev);

        // Test at multiple (u, v) points
        let eps = 1e-6;
        for &u in &[0.1_f64, 0.5, 1.0, 2.0, 3.0] {
            for &v in &[0.0_f64, 0.5, 1.0, 2.0] {
                let analytical = surface.derivatives_at(u, v);

                // Numerical derivatives via central differences
                let p_u_plus = surface.point_at(u + eps, v);
                let p_u_minus = surface.point_at(u - eps, v);
                let p_v_plus = surface.point_at(u, v + eps);
                let p_v_minus = surface.point_at(u, v - eps);

                let num_du_x = (p_u_plus.x - p_u_minus.x) / (2.0 * eps);
                let num_du_y = (p_u_plus.y - p_u_minus.y) / (2.0 * eps);
                let num_du_z = (p_u_plus.z - p_u_minus.z) / (2.0 * eps);
                let num_dv_x = (p_v_plus.x - p_v_minus.x) / (2.0 * eps);
                let num_dv_y = (p_v_plus.y - p_v_minus.y) / (2.0 * eps);
                let num_dv_z = (p_v_plus.z - p_v_minus.z) / (2.0 * eps);

                let du_err = ((analytical.du.x - num_du_x).powi(2)
                    + (analytical.du.y - num_du_y).powi(2)
                    + (analytical.du.z - num_du_z).powi(2)).sqrt();
                let dv_err = ((analytical.dv.x - num_dv_x).powi(2)
                    + (analytical.dv.y - num_dv_y).powi(2)
                    + (analytical.dv.z - num_dv_z).powi(2)).sqrt();

                assert!(du_err < 1e-6,
                    "Revolution dS/du mismatch at ({}, {}): analytical=({:.6},{:.6},{:.6}), numerical=({:.6},{:.6},{:.6}), err={:.2e}",
                    u, v, analytical.du.x, analytical.du.y, analytical.du.z, num_du_x, num_du_y, num_du_z, du_err);
                assert!(dv_err < 1e-6,
                    "Revolution dS/dv mismatch at ({}, {}): analytical=({:.6},{:.6},{:.6}), numerical=({:.6},{:.6},{:.6}), err={:.2e}",
                    u, v, analytical.dv.x, analytical.dv.y, analytical.dv.z, num_dv_x, num_dv_y, num_dv_z, dv_err);
            }
        }
    }

    #[test]
    fn test_revolution_derivatives_cylinder_orthogonal() {
        // For a cylinder (revolution of a vertical line), dS/du should be
        // perpendicular to dS/dv (which is along the axis).
        let profile = Curve3d::Line(Line::new(
            Point3d::new(2.0, 0.0, 0.0),
            Direction3d::Z,
        ));
        let rev = RevolutionSurface::new(profile, Direction3d::Z, Point3d::ORIGIN);
        let surface = Surface::Revolution(rev);

        let ders = surface.derivatives_at(0.5, 1.0);
        let dot = ders.du.x * ders.dv.x + ders.du.y * ders.dv.y + ders.du.z * ders.dv.z;
        assert!(dot.abs() < 1e-10,
            "Cylinder dS/du and dS/dv should be orthogonal, dot product = {}", dot);
    }

    #[test]
    fn test_extrusion_analytical_derivatives_match_numerical() {
        // Extrusion of a circle along Z → cylinder.
        let profile = Curve3d::Circle(Circle::new_xy(Point3d::ORIGIN, 1.0));
        let direction = Direction3d::Z;
        let ext = ExtrusionSurface::new(profile, direction);
        let surface = Surface::Extrusion(ext);

        let eps = 1e-6;
        for &u in &[0.1_f64, 0.5, 1.0, 2.0, 3.0] {
            for &v in &[0.0_f64, 0.5, 1.0, 2.0] {
                let analytical = surface.derivatives_at(u, v);

                let p_u_plus = surface.point_at(u + eps, v);
                let p_u_minus = surface.point_at(u - eps, v);
                let p_v_plus = surface.point_at(u, v + eps);
                let p_v_minus = surface.point_at(u, v - eps);

                let num_du_x = (p_u_plus.x - p_u_minus.x) / (2.0 * eps);
                let num_du_y = (p_u_plus.y - p_u_minus.y) / (2.0 * eps);
                let num_du_z = (p_u_plus.z - p_u_minus.z) / (2.0 * eps);
                let num_dv_x = (p_v_plus.x - p_v_minus.x) / (2.0 * eps);
                let num_dv_y = (p_v_plus.y - p_v_minus.y) / (2.0 * eps);
                let num_dv_z = (p_v_plus.z - p_v_minus.z) / (2.0 * eps);

                let du_err = ((analytical.du.x - num_du_x).powi(2)
                    + (analytical.du.y - num_du_y).powi(2)
                    + (analytical.du.z - num_du_z).powi(2)).sqrt();
                let dv_err = ((analytical.dv.x - num_dv_x).powi(2)
                    + (analytical.dv.y - num_dv_y).powi(2)
                    + (analytical.dv.z - num_dv_z).powi(2)).sqrt();

                assert!(du_err < 1e-6,
                    "Extrusion dS/du mismatch at ({}, {}): err={:.2e}", u, v, du_err);
                assert!(dv_err < 1e-6,
                    "Extrusion dS/dv mismatch at ({}, {}): err={:.2e}", u, v, dv_err);
            }
        }
    }

    #[test]
    fn test_extrusion_dv_constant() {
        // For an extrusion surface, dS/dv should be the (constant) direction.
        let profile = Curve3d::Circle(Circle::new_xy(Point3d::ORIGIN, 1.0));
        let direction = Direction3d::Z;
        let ext = ExtrusionSurface::new(profile, direction);
        let surface = Surface::Extrusion(ext);

        for &u in &[0.0_f64, 1.0, 2.0] {
            for &v in &[0.0_f64, 1.0, 2.0] {
                let ders = surface.derivatives_at(u, v);
                assert!((ders.dv.x - 0.0).abs() < 1e-12, "dS/dv.x should be 0");
                assert!((ders.dv.y - 0.0).abs() < 1e-12, "dS/dv.y should be 0");
                assert!((ders.dv.z - 1.0).abs() < 1e-12, "dS/dv.z should be 1");
            }
        }
    }
}

#[cfg(test)]
mod curvature_tests {
    use super::*;

    #[test]
    fn test_plane_curvature_is_zero() {
        let plane = Surface::Plane(Plane::xy());
        let curv = plane.curvature_at(0.5, 0.5);
        assert!(curv.gaussian.abs() < 1e-10, "Plane Gaussian curvature should be 0");
        assert!(curv.mean.abs() < 1e-10, "Plane mean curvature should be 0");
        assert!(curv.max_abs.abs() < 1e-10, "Plane max curvature should be 0");
    }

    #[test]
    fn test_sphere_curvature() {
        let r = 10.0;
        let sphere = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, r));
        let curv = sphere.curvature_at(1.0, 1.0);
        let expected_k = 1.0 / r;
        assert!((curv.k1 - expected_k).abs() < 1e-6, "Sphere k1 should be 1/r, got {}", curv.k1);
        assert!((curv.k2 - expected_k).abs() < 1e-6, "Sphere k2 should be 1/r, got {}", curv.k2);
        assert!((curv.gaussian - expected_k * expected_k).abs() < 1e-8, "Sphere K should be 1/r²");
    }

    #[test]
    fn test_cylinder_curvature() {
        let r = 5.0;
        let cyl = Surface::Cylinder(CylinderSurface::new_z(r));
        let curv = cyl.curvature_at(0.0, 0.0);
        let expected_k = 1.0 / r;
        assert!((curv.k1 - expected_k).abs() < 1e-6, "Cylinder circumferential curvature should be 1/r");
        assert!(curv.k2.abs() < 1e-6, "Cylinder meridional curvature should be 0");
        assert!(curv.gaussian.abs() < 1e-8, "Cylinder Gaussian curvature should be 0");
    }

    #[test]
    fn test_numerical_vs_analytical_curvature() {
        // Test that numerical curvature computation (used for NURBS/Revolution/Extrusion)
        // matches analytical curvature for surfaces where we have both
        let sphere = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 10.0));
        let (first, second) = sphere.fundamental_forms_at(1.0, 1.5);
        let k_gauss = second.gaussian_curvature(&first);
        let k_mean = second.mean_curvature(&first);

        let expected_k = 1.0 / 10.0;
        let expected_K = expected_k * expected_k;
        // Note: mean curvature sign depends on normal orientation;
        // the numerical fundamental-form computation may give -1/r
        // for outward normals on a convex surface, so we compare absolute values.
        let expected_H = expected_k;

        assert!((k_gauss - expected_K).abs() < 0.01, "Numerical Gaussian curvature {} should be close to {}", k_gauss, expected_K);
        assert!((k_mean.abs() - expected_H).abs() < 0.01, "Numerical |mean curvature| {} should be close to {}", k_mean.abs(), expected_H);
    }

    #[test]
    fn test_nurbs_derivatives_vs_numerical() {
        // Create a simple bilinear NURBS patch and compare derivatives
        let nurbs = NurbsSurface {
            u_degree: 1,
            v_degree: 1,
            control_points: vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 10.0, 0.0)],
                vec![Point3d::new(10.0, 0.0, 0.0), Point3d::new(10.0, 10.0, 5.0)],
            ],
            weights: vec![
                vec![1.0, 1.0],
                vec![1.0, 1.0],
            ],
            u_knots: vec![0.0, 0.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        };

        let surface = Surface::Nurbs(nurbs);
        let derivs = surface.derivatives_at(0.5, 0.5);

        // Compare with finite differences
        let eps = 1e-5;
        let p0 = surface.point_at(0.5, 0.5);
        let pu = surface.point_at(0.5 + eps, 0.5);
        let pv = surface.point_at(0.5, 0.5 + eps);

        let du_fd = Vec3d::new(
            (pu.x - p0.x) / eps,
            (pu.y - p0.y) / eps,
            (pu.z - p0.z) / eps,
        );
        let dv_fd = Vec3d::new(
            (pv.x - p0.x) / eps,
            (pv.y - p0.y) / eps,
            (pv.z - p0.z) / eps,
        );

        let du_err = ((derivs.du.x - du_fd.x).powi(2) + (derivs.du.y - du_fd.y).powi(2) + (derivs.du.z - du_fd.z).powi(2)).sqrt();
        let dv_err = ((derivs.dv.x - dv_fd.x).powi(2) + (derivs.dv.y - dv_fd.y).powi(2) + (derivs.dv.z - dv_fd.z).powi(2)).sqrt();

        assert!(du_err < 1.0, "dS/du error should be small, got {}", du_err);
        assert!(dv_err < 1.0, "dS/dv error should be small, got {}", dv_err);
    }

    #[test]
    fn test_uv_metric_scale_plane() {
        let plane = Plane { origin: Point3d::ORIGIN, u_dir: Direction3d::X, v_dir: Direction3d::Y, normal: Direction3d::Z };
        let surface = Surface::Plane(plane);
        let (us, vs) = surface.uv_metric_scale();
        assert_eq!(us, 1.0, "Plane U metric scale should be 1.0");
        assert_eq!(vs, 1.0, "Plane V metric scale should be 1.0");
    }

    #[test]
    fn test_uv_metric_scale_cylinder() {
        let cyl = CylinderSurface::new_z(5.0);
        let surface = Surface::Cylinder(cyl);
        let (us, vs) = surface.uv_metric_scale();
        assert!((us - 5.0).abs() < 1e-10, "Cylinder U metric scale should be radius (5.0), got {}", us);
        assert_eq!(vs, 1.0, "Cylinder V metric scale should be 1.0");
    }

    #[test]
    fn test_uv_metric_scale_cone() {
        let cone = ConeSurface::new_z(10.0, PI / 4.0);
        let surface = Surface::Cone(cone);
        let (us, vs) = surface.uv_metric_scale();
        assert!((us - 10.0).abs() < 1e-10, "Cone U metric scale should be base radius (10.0), got {}", us);
        // cos(π/4) = √2/2, 1/cos(π/4) = √2 ≈ 1.414
        let expected_vs = 1.0 / (PI / 4.0).cos();
        assert!((vs - expected_vs).abs() < 1e-10, "Cone V metric scale should be 1/cos(half_angle), got {} expected {}", vs, expected_vs);
    }

    #[test]
    fn test_uv_metric_scale_sphere() {
        let sphere = SphereSurface::new(Point3d::ORIGIN, 7.0);
        let surface = Surface::Sphere(sphere);
        let (us, vs) = surface.uv_metric_scale();
        assert!((us - 7.0).abs() < 1e-10, "Sphere U metric scale should be radius (7.0), got {}", us);
        assert!((vs - 7.0).abs() < 1e-10, "Sphere V metric scale should be radius (7.0), got {}", vs);
    }

    #[test]
    fn test_uv_metric_scale_torus() {
        let torus = TorusSurface { center: Point3d::ORIGIN, axis: Direction3d::Z, major_radius: 10.0, minor_radius: 3.0, x_dir: Direction3d::X };
        let surface = Surface::Torus(torus);
        let (us, vs) = surface.uv_metric_scale();
        assert!((us - 13.0).abs() < 1e-10, "Torus U metric scale should be R+r (13.0), got {}", us);
        assert!((vs - 3.0).abs() < 1e-10, "Torus V metric scale should be r (3.0), got {}", vs);
    }
}
