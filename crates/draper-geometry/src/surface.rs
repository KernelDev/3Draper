//! Parametric surfaces in 3D space.

use crate::{Direction3d, Point3d, Point2d, Vec3d, Transform, curve::Curve3d};
use std::fmt;

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
}

/// A cylindrical surface.
#[derive(Clone, Debug)]
pub struct CylinderSurface {
    pub origin: Point3d,
    pub axis: Direction3d,
    pub radius: f64,
}

impl CylinderSurface {
    /// Create a cylinder along the Z axis.
    pub fn new_z(radius: f64) -> Self {
        Self {
            origin: Point3d::ORIGIN,
            axis: Direction3d::Z,
            radius,
        }
    }

    /// Create a cylinder at a given origin along a given axis.
    pub fn new(origin: Point3d, axis: Direction3d, radius: f64) -> Self {
        Self { origin, axis, radius }
    }

    /// Evaluate: u = angle in radians [0, 2pi], v = height along axis.
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let x_dir = if self.axis.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            self.axis.cross(&Direction3d::Z)
        };
        let y_dir = self.axis.cross(&x_dir);

        Point3d::new(
            self.origin.x + self.radius * (u.cos() * x_dir.x + u.sin() * y_dir.x) + v * self.axis.x,
            self.origin.y + self.radius * (u.cos() * x_dir.y + u.sin() * y_dir.y) + v * self.axis.y,
            self.origin.z + self.radius * (u.cos() * x_dir.z + u.sin() * y_dir.z) + v * self.axis.z,
        )
    }

    /// Normal at (u, v) — points outward.
    pub fn normal_at(&self, u: f64, _v: f64) -> Direction3d {
        let x_dir = if self.axis.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            self.axis.cross(&Direction3d::Z)
        };
        let y_dir = self.axis.cross(&x_dir);
        Direction3d::new(
            u.cos() * x_dir.x + u.sin() * y_dir.x,
            u.cos() * x_dir.y + u.sin() * y_dir.y,
            u.cos() * x_dir.z + u.sin() * y_dir.z,
        ).unwrap_or(Direction3d::X)
    }

    /// Parametric range: u in [0, 2pi], v in [-inf, inf].
    pub fn u_range(&self) -> (f64, f64) {
        (0.0, 2.0 * std::f64::consts::PI)
    }
}

/// A conical surface.
///
/// Parameterization: v=0 is the base circle with the given radius,
/// v increases toward the apex where radius reaches 0.
/// Height from base to apex = radius / tan(half_angle).
#[derive(Clone, Debug)]
pub struct ConeSurface {
    pub origin: Point3d,    // Center of base circle
    pub axis: Direction3d,   // Direction from base toward apex
    pub half_angle: f64,     // Half-angle in radians
    pub radius: f64,         // Base radius (at v=0)
}

impl ConeSurface {
    /// Create a cone along the Z axis with base at z=0.
    /// The base has the given radius, and the apex is at z = radius / tan(half_angle).
    pub fn new_z(radius: f64, half_angle: f64) -> Self {
        Self {
            origin: Point3d::ORIGIN,
            axis: Direction3d::Z,
            half_angle,
            radius,
        }
    }

    /// Height from base to apex.
    pub fn height(&self) -> f64 {
        if self.half_angle.abs() < 1e-10 {
            f64::INFINITY
        } else {
            self.radius / self.half_angle.tan()
        }
    }

    /// Evaluate: u = angle in radians [0, 2pi], v = height from base along axis.
    /// At v=0: radius = self.radius (base). At v=height(): radius = 0 (apex).
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        // Radius decreases linearly from base to apex
        let r = (self.radius - v * self.half_angle.tan()).max(0.0);
        let x_dir = if self.axis.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            self.axis.cross(&Direction3d::Z)
        };
        let y_dir = self.axis.cross(&x_dir);

        Point3d::new(
            self.origin.x + r * (u.cos() * x_dir.x + u.sin() * y_dir.x) + v * self.axis.x,
            self.origin.y + r * (u.cos() * x_dir.y + u.sin() * y_dir.y) + v * self.axis.y,
            self.origin.z + r * (u.cos() * x_dir.z + u.sin() * y_dir.z) + v * self.axis.z,
        )
    }

    /// Normal at (u, v) — points outward.
    pub fn normal_at(&self, u: f64, _v: f64) -> Direction3d {
        let x_dir = if self.axis.is_parallel_to(&Direction3d::Z) {
            Direction3d::X
        } else {
            self.axis.cross(&Direction3d::Z)
        };
        let y_dir = self.axis.cross(&x_dir);
        // Normal to cone: perpendicular to the slant surface
        // The slant has angle half_angle from the axis
        let radial = Direction3d::new(
            u.cos() * x_dir.x + u.sin() * y_dir.x,
            u.cos() * x_dir.y + u.sin() * y_dir.y,
            u.cos() * x_dir.z + u.sin() * y_dir.z,
        ).unwrap_or(Direction3d::X);
        // Normal = radial * cos(half_angle) - axis * sin(half_angle)
        let ha = self.half_angle;
        Direction3d::new(
            radial.x * ha.cos() - self.axis.x * ha.sin(),
            radial.y * ha.cos() - self.axis.y * ha.sin(),
            radial.z * ha.cos() - self.axis.z * ha.sin(),
        ).unwrap_or(radial)
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
}

/// A toroidal surface.
#[derive(Clone, Debug)]
pub struct TorusSurface {
    pub center: Point3d,
    pub axis: Direction3d,
    pub major_radius: f64, // R — distance from center to tube center
    pub minor_radius: f64, // r — tube radius
}

impl TorusSurface {
    pub fn new_z(center: Point3d, major_radius: f64, minor_radius: f64) -> Self {
        Self { center, axis: Direction3d::Z, major_radius, minor_radius }
    }

    /// Evaluate: u = angle around main ring [0, 2pi], v = angle around tube [0, 2pi].
    pub fn point_at(&self, u: f64, v: f64) -> Point3d {
        let x_dir = Direction3d::X;
        let y_dir = Direction3d::Y;
        let r = self.major_radius + self.minor_radius * v.cos();
        Point3d::new(
            self.center.x + r * u.cos() * x_dir.x + r * u.sin() * y_dir.x,
            self.center.y + r * u.cos() * x_dir.y + r * u.sin() * y_dir.y,
            self.center.z + self.minor_radius * v.sin(),
        )
    }

    pub fn normal_at(&self, u: f64, v: f64) -> Direction3d {
        let x_dir = Direction3d::X;
        let y_dir = Direction3d::Y;
        let nx = v.cos() * u.cos() * x_dir.x + v.cos() * u.sin() * y_dir.x;
        let ny = v.cos() * u.cos() * x_dir.y + v.cos() * u.sin() * y_dir.y;
        let nz = v.sin();
        Direction3d::new(nx, ny, nz).unwrap_or(Direction3d::Z)
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

impl Surface {
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
            }),
            Surface::Cone(c) => Surface::Cone(ConeSurface {
                origin: t.transform_point(&c.origin),
                axis: t.transform_direction(&c.axis),
                half_angle: c.half_angle,
                radius: c.radius,
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

/// NURBS surface evaluation using de Boor's algorithm in 2D.
/// Evaluates by performing de Boor in the u-direction for each row of control points,
/// then de Boor in the v-direction on the resulting intermediate points.
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

    let u_clamped = u.clamp(u_min, u_max);
    let v_clamped = v.clamp(v_min, v_max);

    // Find u knot span
    let mut k_u = p;
    while k_u < nurbs.u_knots.len() - p - 1 && nurbs.u_knots[k_u + 1] <= u_clamped {
        k_u += 1;
    }
    k_u = k_u.min(n_u - 1);

    // Find v knot span
    let mut k_v = q;
    while k_v < nurbs.v_knots.len() - q - 1 && nurbs.v_knots[k_v + 1] <= v_clamped {
        k_v += 1;
    }
    k_v = k_v.min(n_v - 1);

    // For each row affected by the u knot span, evaluate a NURBS curve in u
    // This gives us p+1 intermediate control points in the v direction
    let mut intermediate: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(p + 1);

    for i in 0..=p {
        let row_idx = if k_u >= p { k_u - p + i } else { i.min(n_u - 1) };
        if row_idx >= n_u {
            continue;
        }

        // Collect control points and weights for this row
        let mut pts: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(q + 1);
        for j in 0..=q {
            let col_idx = if k_v >= q { k_v - q + j } else { j.min(n_v - 1) };
            if col_idx >= n_v {
                continue;
            }
            let cp = &nurbs.control_points[row_idx][col_idx];
            let w = nurbs.weights.get(row_idx).and_then(|r| r.get(col_idx)).copied().unwrap_or(1.0);
            pts.push((cp.x * w, cp.y * w, cp.z * w, w));
        }

        // De Boor in v direction
        for r in 1..pts.len() {
            for j in (r..pts.len()).rev() {
                let i0 = if k_v >= q { k_v - q + j } else { j }.min(nurbs.v_knots.len().saturating_sub(1));
                let i1 = (i0 + 1).min(nurbs.v_knots.len().saturating_sub(1));
                let denom_idx = (i0 + q - r + 1).min(nurbs.v_knots.len().saturating_sub(1));

                let d = nurbs.v_knots[denom_idx] - nurbs.v_knots[i1];
                let a = if d.abs() < 1e-15 { 0.0 } else { (v_clamped - nurbs.v_knots[i1]) / d };
                let b = 1.0 - a;

                pts[j] = (
                    a * pts[j].0 + b * pts[j - 1].0,
                    a * pts[j].1 + b * pts[j - 1].1,
                    a * pts[j].2 + b * pts[j - 1].2,
                    a * pts[j].3 + b * pts[j - 1].3,
                );
            }
        }

        if !pts.is_empty() {
            let last = pts.last().unwrap();
            intermediate.push(*last);
        }
    }

    // Now de Boor in u direction on the intermediate points
    for r in 1..intermediate.len() {
        for j in (r..intermediate.len()).rev() {
            let i0 = if k_u >= p { k_u - p + j } else { j }.min(nurbs.u_knots.len().saturating_sub(1));
            let i1 = (i0 + 1).min(nurbs.u_knots.len().saturating_sub(1));
            let denom_idx = (i0 + p - r + 1).min(nurbs.u_knots.len().saturating_sub(1));

            let d = nurbs.u_knots[denom_idx] - nurbs.u_knots[i1];
            let a = if d.abs() < 1e-15 { 0.0 } else { (u_clamped - nurbs.u_knots[i1]) / d };
            let b = 1.0 - a;

            intermediate[j] = (
                a * intermediate[j].0 + b * intermediate[j - 1].0,
                a * intermediate[j].1 + b * intermediate[j - 1].1,
                a * intermediate[j].2 + b * intermediate[j - 1].2,
                a * intermediate[j].3 + b * intermediate[j - 1].3,
            );
        }
    }

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
