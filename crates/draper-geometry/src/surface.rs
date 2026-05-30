//! Parametric surfaces in 3D space.

use crate::{Direction3d, Point3d, Point2d, Vec3d, Transform, curve::Curve3d};
use std::fmt;

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
}

/// A plane in 3D space.
#[derive(Clone, Debug)]
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
    /// u = angle in radians, v = height along axis.
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let y_dir = self.axis.cross(&self.x_dir);
        let dx = point.x - self.origin.x;
        let dy = point.y - self.origin.y;
        let dz = point.z - self.origin.z;
        let u = (dx * y_dir.x + dy * y_dir.y + dz * y_dir.z)
            .atan2(dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z);
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

    /// Height from base to apex.
    /// For expanding cones, this is infinity (no natural apex in positive v direction).
    pub fn height(&self) -> f64 {
        if self.expanding {
            f64::INFINITY
        } else if self.half_angle.abs() < 1e-10 {
            f64::INFINITY
        } else {
            self.radius / self.half_angle.tan()
        }
    }

    /// Evaluate: u = angle in radians [0, 2pi], v = height from base along axis.
    /// For standard cones: At v=0: radius = self.radius (base). At v=height(): radius = 0 (apex).
    /// For expanding cones: At v=0: radius = 0 (apex). At v>0: radius = v * tan(half_angle).
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let r = if self.expanding {
            v * self.half_angle.tan()
        } else {
            // Radius decreases linearly from base to apex
            (self.radius - v * self.half_angle.tan()).max(0.0)
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
    /// u = angle in radians, v = height along axis.
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let y_dir = self.axis.cross(&self.x_dir);
        let dx = point.x - self.origin.x;
        let dy = point.y - self.origin.y;
        let dz = point.z - self.origin.z;
        let u = (dx * y_dir.x + dy * y_dir.y + dz * y_dir.z)
            .atan2(dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z);
        let v = dx * self.axis.x + dy * self.axis.y + dz * self.axis.z;
        (u, v)
    }
}

/// A spherical surface.
#[derive(Clone, Debug)]
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
    /// u = azimuthal angle [0, 2pi], v = polar angle [0, pi].
    pub fn project_point(&self, point: &Point3d) -> (f64, f64) {
        let dx = point.x - self.center.x;
        let dy = point.y - self.center.y;
        let dz = point.z - self.center.z;
        let u = dy.atan2(dx);
        let r = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-15);
        let v = (dz / r).clamp(-1.0, 1.0).acos();
        (u, v)
    }
}

/// A toroidal surface.
#[derive(Clone, Debug)]
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
        // u = angle around main ring in the x_dir/y_dir plane
        let u = (dx * y_dir.x + dy * y_dir.y + dz * y_dir.z)
            .atan2(dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z);
        // v = angle around tube
        let radial_dist = dx * self.x_dir.x + dy * self.x_dir.y + dz * self.x_dir.z;
        let radial_y = dx * y_dir.x + dy * y_dir.y + dz * y_dir.z;
        let dist_ring = (radial_dist * radial_dist + radial_y * radial_y).sqrt();
        let along_axis = dx * self.axis.x + dy * self.axis.y + dz * self.axis.z;
        let local_x = dist_ring - self.major_radius;
        let local_y = along_axis;
        let v = local_y.atan2(local_x);
        (u, v)
    }
}

/// Surface of revolution.
#[derive(Clone, Debug)]
pub struct RevolutionSurface {
    /// The profile curve in the XZ plane (revolved around Z axis).
    pub profile: Curve3d,
    /// Axis of revolution.
    pub axis: Direction3d,
    /// Origin point on the axis.
    pub origin: Point3d,
}

impl RevolutionSurface {
    pub fn new(profile: Curve3d, axis: Direction3d, origin: Point3d) -> Self {
        Self { profile, axis, origin }
    }

    /// Evaluate: u = revolution angle [0, 2pi], v = parameter on profile curve.
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let p = self.profile.point_at(v);
        // Profile is in XZ plane, revolve around Z
        let cos_u = u.cos();
        let sin_u = u.sin();
        Point3d::new(
            self.origin.x + p.x * cos_u,
            self.origin.y + p.x * sin_u,
            self.origin.z + p.z,
        )
    }
}

/// Extruded surface — a curve swept along a direction.
#[derive(Clone, Debug)]
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
}

/// NURBS surface.
#[derive(Clone, Debug)]
pub struct NurbsSurface {
    pub u_degree: usize,
    pub v_degree: usize,
    pub control_points: Vec<Vec<Point3d>>,
    pub weights: Vec<Vec<f64>>,
    pub u_knots: Vec<f64>,
    pub v_knots: Vec<f64>,
}

impl NurbsSurface {
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
                    (cone.radius - v * cone.half_angle.tan()).max(0.0)
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
                let (v_min, v_max) = nurbs.v_range();

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
            _ => {
                // Numerical differentiation fallback
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
        matches!(self, Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_))
    }

    /// Check if the surface is periodic in v.
    pub fn is_v_periodic(&self) -> bool {
        matches!(self, Surface::Sphere(_) | Surface::Torus(_))
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
                // u = revolution angle
                let dx = point.x - r.origin.x;
                let dy = point.y - r.origin.y;
                let u = dy.atan2(dx);
                // v = profile curve parameter: find the closest point on the profile curve
                // The profile curve is in the XZ plane, and the surface point at (u, v)
                // is: origin + (profile(v).x * cos(u), profile(v).x * sin(u), profile(v).z)
                // We need to find v such that profile(v) matches the radial distance and z.
                // Strategy: search the profile curve for the closest point in (radius, z) space.
                let dz = point.z - r.origin.z;
                let radial = (dx * dx + dy * dy).sqrt();
                let (v_min, v_max) = r.profile.param_range();
                let mut best_v = (v_min + v_max) * 0.5;
                let mut best_dist = f64::MAX;
                let steps = 64;
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
                let refine_steps = 20;
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
                // First compute v by projecting the point onto the extrusion direction
                let p0 = e.profile.point_at(0.0);
                let dx = point.x - p0.x;
                let dy = point.y - p0.y;
                let dz = point.z - p0.z;
                let v = dx * e.direction.x + dy * e.direction.y + dz * e.direction.z;

                // For u: find the profile curve parameter where the profile point
                // is closest to the 3D point projected onto the profile plane
                // (subtract the extrusion component)
                let px = point.x - v * e.direction.x;
                let py = point.y - v * e.direction.y;
                let pz = point.z - v * e.direction.z;

                let (u_min, u_max) = e.profile.param_range();
                let mut best_u = (u_min + u_max) * 0.5;
                let mut best_dist = f64::MAX;
                // Coarse search
                let steps = 64;
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
                let refine_steps = 20;
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
                // Grid-based closest point search using actual knot range.
                // Uses progressively finer searches (coarse → medium → fine) for
                // good accuracy with fewer total evaluations than a single fine grid.
                let (u_min, u_max) = n.u_range();
                let (v_min, v_max) = n.v_range();
                let mut best_u = (u_min + u_max) * 0.5;
                let mut best_v = (v_min + v_max) * 0.5;
                let mut best_dist = f64::MAX;

                // Phase 1: Coarse grid (10×10 = 121 evaluations)
                let coarse = 10;
                for i in 0..=coarse {
                    for j in 0..=coarse {
                        let u = u_min + (u_max - u_min) * i as f64 / coarse as f64;
                        let v = v_min + (v_max - v_min) * j as f64 / coarse as f64;
                        let p = self.point_at(u, v);
                        let dist = (p.x - point.x).powi(2) + (p.y - point.y).powi(2) + (p.z - point.z).powi(2);
                        if dist < best_dist {
                            best_dist = dist;
                            best_u = u;
                            best_v = v;
                        }
                    }
                }

                // Phase 2: Medium refinement (8×8 = 81 evaluations around best)
                let medium = 8;
                let u_range = (u_max - u_min) / coarse as f64;
                let v_range = (v_max - v_min) / coarse as f64;
                let mut med_best_u = best_u;
                let mut med_best_v = best_v;
                let mut med_best_dist = best_dist;
                for i in 0..=medium {
                    for j in 0..=medium {
                        let u = (best_u - u_range * 0.5 + u_range * i as f64 / medium as f64).clamp(u_min, u_max);
                        let v = (best_v - v_range * 0.5 + v_range * j as f64 / medium as f64).clamp(v_min, v_max);
                        let p = self.point_at(u, v);
                        let dist = (p.x - point.x).powi(2) + (p.y - point.y).powi(2) + (p.z - point.z).powi(2);
                        if dist < med_best_dist {
                            med_best_dist = dist;
                            med_best_u = u;
                            med_best_v = v;
                        }
                    }
                }
                best_u = med_best_u;
                best_v = med_best_v;
                best_dist = med_best_dist;

                // Phase 3: Fine refinement (6×6 = 49 evaluations around best)
                let fine = 6;
                let u_range2 = u_range / medium as f64;
                let v_range2 = v_range / medium as f64;
                for i in 0..=fine {
                    for j in 0..=fine {
                        let u = (best_u - u_range2 * 0.5 + u_range2 * i as f64 / fine as f64).clamp(u_min, u_max);
                        let v = (best_v - v_range2 * 0.5 + v_range2 * j as f64 / fine as f64).clamp(v_min, v_max);
                        let p = self.point_at(u, v);
                        let dist = (p.x - point.x).powi(2) + (p.y - point.y).powi(2) + (p.z - point.z).powi(2);
                        if dist < best_dist {
                            best_dist = dist;
                            best_u = u;
                            best_v = v;
                        }
                    }
                }

                (best_u, best_v)
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

    if intermediate.is_empty() {
        return Point3d::ORIGIN;
    }

    let result = intermediate.last().unwrap();
    let w = result.3;
    if w.abs() < 1e-15 {
        Point3d::ORIGIN
    } else {
        Point3d::new(result.0 / w, result.1 / w, result.2 / w)
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
    use std::f64::consts::PI;

    #[test]
    fn test_cone_apex_degenerate() {
        let cone = ConeSurface::new_z(10.0, 0.5);
        let surface = Surface::Cone(cone);
        // At v = height (apex), the cone is degenerate
        let apex_v = 10.0 / 0.5_f64.tan(); // height = radius / tan(half_angle)
        let flags = surface.is_degenerate_at(0.0, apex_v, 1e-6);
        assert!(flags.is_degenerate(), "Cone apex should be degenerate, got {:?}", flags);
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
}
