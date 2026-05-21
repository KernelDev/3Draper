//! Surface primitives — parametric surfaces in 3D.

use crate::curve::BSplineCurve;
use crate::direction::Axis2Placement3D;
use crate::point::Point3;
use crate::transform::Transform3;

use serde::{Deserialize, Serialize};

/// A parametric surface in 3D. Parameters (u, v) map to 3D point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Surface {
    Plane(Plane),
    CylindricalSurface(CylindricalSurface),
    ConicalSurface(ConicalSurface),
    SphericalSurface(SphericalSurface),
    ToroidalSurface(ToroidalSurface),
    SurfaceOfRevolution(SurfaceOfRevolution),
    SurfaceOfLinearExtrusion(SurfaceOfLinearExtrusion),
    BSplineSurface(BSplineSurface),
    OffsetSurface(OffsetSurface),
    RectangularTrimmedSurface(RectangularTrimmedSurface),
}

impl Surface {
    /// Evaluate the surface at parameters (u, v).
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        match self {
            Surface::Plane(s) => s.point_at(u, v),
            Surface::CylindricalSurface(s) => s.point_at(u, v),
            Surface::ConicalSurface(s) => s.point_at(u, v),
            Surface::SphericalSurface(s) => s.point_at(u, v),
            Surface::ToroidalSurface(s) => s.point_at(u, v),
            Surface::SurfaceOfRevolution(s) => s.point_at(u, v),
            Surface::SurfaceOfLinearExtrusion(s) => s.point_at(u, v),
            Surface::BSplineSurface(s) => s.point_at(u, v),
            Surface::OffsetSurface(s) => s.point_at(u, v),
            Surface::RectangularTrimmedSurface(s) => s.basis_surface.point_at(
                s.u1 + u * (s.u2 - s.u1),
                s.v1 + v * (s.v2 - s.v1),
            ),
        }
    }

    /// Compute the surface normal at (u, v) via finite differences.
    pub fn normal_at(&self, u: f64, v: f64) -> crate::direction::Direction3 {
        let eps = 1e-5;
        let p = self.point_at(u, v);
        let pu = self.point_at(u + eps, v);
        let pv = self.point_at(u, v + eps);

        let du = pu - p;
        let dv = pv - p;

        let normal = du.cross(dv);
        crate::direction::Direction3::new(normal.x, normal.y, normal.z)
            .unwrap_or(crate::direction::Direction3::Z)
    }

    /// Get approximate bounding box by sampling the surface.
    pub fn bounding_box(&self, u_samples: usize, v_samples: usize) -> crate::point::BoundingBox3 {
        let mut bb = crate::point::BoundingBox3::empty();
        for i in 0..=u_samples {
            for j in 0..=v_samples {
                let u = i as f64 / u_samples as f64;
                let v = j as f64 / v_samples as f64;
                bb.extend(self.point_at(u, v));
            }
        }
        bb
    }

    pub fn transform(&self, tf: &Transform3) -> Surface {
        match self {
            Surface::Plane(s) => Surface::Plane(s.transform(tf)),
            Surface::CylindricalSurface(s) => Surface::CylindricalSurface(s.transform(tf)),
            Surface::ConicalSurface(s) => Surface::ConicalSurface(s.transform(tf)),
            Surface::SphericalSurface(s) => Surface::SphericalSurface(s.transform(tf)),
            Surface::ToroidalSurface(s) => Surface::ToroidalSurface(s.transform(tf)),
            Surface::SurfaceOfRevolution(s) => Surface::SurfaceOfRevolution(s.transform(tf)),
            Surface::SurfaceOfLinearExtrusion(s) => Surface::SurfaceOfLinearExtrusion(s.transform(tf)),
            Surface::BSplineSurface(s) => Surface::BSplineSurface(s.transform(tf)),
            Surface::OffsetSurface(s) => Surface::OffsetSurface(s.transform(tf)),
            Surface::RectangularTrimmedSurface(s) => Surface::RectangularTrimmedSurface(RectangularTrimmedSurface {
                basis_surface: Box::new(s.basis_surface.transform(tf)),
                u1: s.u1,
                u2: s.u2,
                v1: s.v1,
                v2: s.v2,
            }),
        }
    }
}

/// A plane defined by a position and normal.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Plane {
    pub axis: Axis2Placement3D,
}

impl Plane {
    pub fn new(axis: Axis2Placement3D) -> Self {
        Self { axis }
    }

    /// u and v are distances along the local X and Y axes from the origin.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let center = self.axis.location.to_dvec3();
        let x = self.axis.ref_direction.to_dvec3();
        let y = self.axis.y_direction().to_dvec3();
        Point3::from_dvec3(center + x * u + y * v)
    }

    pub fn transform(&self, tf: &Transform3) -> Plane {
        Plane {
            axis: Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
        }
    }
}

/// A cylindrical surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CylindricalSurface {
    pub axis: Axis2Placement3D,
    pub radius: f64,
}

impl CylindricalSurface {
    pub fn new(axis: Axis2Placement3D, radius: f64) -> Self {
        Self { axis, radius }
    }

    /// u = angle (0 to 2PI), v = distance along axis.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let center = self.axis.location.to_dvec3();
        let x = self.axis.ref_direction.to_dvec3();
        let y = self.axis.y_direction().to_dvec3();
        let z = self.axis.axis.to_dvec3();

        Point3::from_dvec3(
            center + x * (self.radius * u.cos()) + y * (self.radius * u.sin()) + z * v,
        )
    }

    pub fn transform(&self, tf: &Transform3) -> CylindricalSurface {
        CylindricalSurface {
            axis: Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
            radius: self.radius * tf.scale(),
        }
    }
}

/// A conical surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConicalSurface {
    pub axis: Axis2Placement3D,
    pub radius: f64,
    pub semi_angle: f64, // Half-angle of the cone
}

impl ConicalSurface {
    pub fn new(axis: Axis2Placement3D, radius: f64, semi_angle: f64) -> Self {
        Self { axis, radius, semi_angle }
    }

    /// u = angle (0 to 2PI), v = distance along axis.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let center = self.axis.location.to_dvec3();
        let x = self.axis.ref_direction.to_dvec3();
        let y = self.axis.y_direction().to_dvec3();
        let z = self.axis.axis.to_dvec3();

        let r = self.radius + v * self.semi_angle.tan();
        Point3::from_dvec3(
            center + x * (r * u.cos()) + y * (r * u.sin()) + z * v,
        )
    }

    pub fn transform(&self, tf: &Transform3) -> ConicalSurface {
        ConicalSurface {
            axis: Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
            radius: self.radius * tf.scale(),
            semi_angle: self.semi_angle,
        }
    }
}

/// A spherical surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SphericalSurface {
    pub axis: Axis2Placement3D,
    pub radius: f64,
}

impl SphericalSurface {
    pub fn new(axis: Axis2Placement3D, radius: f64) -> Self {
        Self { axis, radius }
    }

    /// u = azimuthal angle (0 to 2PI), v = polar angle (-PI/2 to PI/2).
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let center = self.axis.location.to_dvec3();
        let x = self.axis.ref_direction.to_dvec3();
        let y = self.axis.y_direction().to_dvec3();
        let z = self.axis.axis.to_dvec3();

        let r = self.radius;
        let cos_v = v.cos();
        Point3::from_dvec3(
            center + x * (r * cos_v * u.cos())
                + y * (r * cos_v * u.sin())
                + z * (r * v.sin()),
        )
    }

    pub fn transform(&self, tf: &Transform3) -> SphericalSurface {
        SphericalSurface {
            axis: Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
            radius: self.radius * tf.scale(),
        }
    }
}

/// A toroidal surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ToroidalSurface {
    pub axis: Axis2Placement3D,
    pub major_radius: f64,
    pub minor_radius: f64,
}

impl ToroidalSurface {
    pub fn new(axis: Axis2Placement3D, major_radius: f64, minor_radius: f64) -> Self {
        Self { axis, major_radius, minor_radius }
    }

    /// u = angle around the main axis (0 to 2PI),
    /// v = angle around the tube (0 to 2PI).
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let center = self.axis.location.to_dvec3();
        let x = self.axis.ref_direction.to_dvec3();
        let y = self.axis.y_direction().to_dvec3();
        let z = self.axis.axis.to_dvec3();

        let r = self.major_radius;
        let minor_r = self.minor_radius;

        Point3::from_dvec3(
            center
                + x * ((r + minor_r * v.cos()) * u.cos())
                + y * ((r + minor_r * v.cos()) * u.sin())
                + z * (minor_r * v.sin()),
        )
    }

    pub fn transform(&self, tf: &Transform3) -> ToroidalSurface {
        ToroidalSurface {
            axis: Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
            major_radius: self.major_radius * tf.scale(),
            minor_radius: self.minor_radius * tf.scale(),
        }
    }
}

/// A surface of revolution.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SurfaceOfRevolution {
    /// The profile curve that is revolved.
    pub generatrix: crate::curve::Curve,
    /// The axis of revolution.
    pub axis: Axis2Placement3D,
}

impl SurfaceOfRevolution {
    /// u = angle of revolution (0 to 2PI), v = parameter along the profile curve.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let profile_pt = self.generatrix.point_at(v);
        let center = self.axis.location.to_dvec3();
        let x = self.axis.ref_direction.to_dvec3();
        let y = self.axis.y_direction().to_dvec3();
        let z = self.axis.axis.to_dvec3();

        // Project profile point onto the axis coordinate system
        let rel = profile_pt.to_dvec3() - center;
        let along = rel.dot(z); // distance along axis
        let perp_x = rel.dot(x); // distance in X direction
        let perp_y = rel.dot(y); // distance in Y direction
        let radius = (perp_x * perp_x + perp_y * perp_y).sqrt();

        Point3::from_dvec3(
            center + x * (radius * u.cos()) + y * (radius * u.sin()) + z * along,
        )
    }

    pub fn transform(&self, tf: &Transform3) -> SurfaceOfRevolution {
        SurfaceOfRevolution {
            generatrix: self.generatrix.transform(tf),
            axis: Axis2Placement3D {
                location: tf.transform_point(self.axis.location),
                axis: tf.transform_direction(self.axis.axis),
                ref_direction: tf.transform_direction(self.axis.ref_direction),
            },
        }
    }
}

/// A surface of linear extrusion.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SurfaceOfLinearExtrusion {
    /// The profile curve that is extruded.
    pub generatrix: crate::curve::Curve,
    /// The direction of extrusion.
    pub direction: crate::direction::Direction3,
}

impl SurfaceOfLinearExtrusion {
    /// u = parameter along profile curve, v = distance along extrusion direction.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let profile_pt = self.generatrix.point_at(u);
        profile_pt + self.direction.to_dvec3() * v
    }

    pub fn transform(&self, tf: &Transform3) -> SurfaceOfLinearExtrusion {
        SurfaceOfLinearExtrusion {
            generatrix: self.generatrix.transform(tf),
            direction: tf.transform_direction(self.direction),
        }
    }
}

/// A B-Spline surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BSplineSurface {
    pub poles: Vec<Vec<Point3>>,
    pub u_knots: Vec<f64>,
    pub v_knots: Vec<f64>,
    pub u_multiplicities: Vec<u32>,
    pub v_multiplicities: Vec<u32>,
    pub u_degree: u32,
    pub v_degree: u32,
    pub u_periodic: bool,
    pub v_periodic: bool,
}

impl BSplineSurface {
    /// Evaluate using tensor product of De Boor's algorithm.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        // Evaluate each row at parameter u to get intermediate points,
        // then evaluate those at parameter v.
        let n_rows = self.poles.len();
        if n_rows == 0 {
            return Point3::ORIGIN;
        }

        let intermediate: Vec<Point3> = self
            .poles
            .iter()
            .map(|row| {
                let curve = BSplineCurve {
                    poles: row.clone(),
                    knots: self.u_knots.clone(),
                    multiplicities: self.u_multiplicities.clone(),
                    degree: self.u_degree,
                    periodic: self.u_periodic,
                };
                curve.point_at(u)
            })
            .collect();

        let v_curve = BSplineCurve {
            poles: intermediate,
            knots: self.v_knots.clone(),
            multiplicities: self.v_multiplicities.clone(),
            degree: self.v_degree,
            periodic: self.v_periodic,
        };
        v_curve.point_at(v)
    }

    pub fn transform(&self, tf: &Transform3) -> BSplineSurface {
        BSplineSurface {
            poles: self
                .poles
                .iter()
                .map(|row| row.iter().map(|p| tf.transform_point(*p)).collect())
                .collect(),
            u_knots: self.u_knots.clone(),
            v_knots: self.v_knots.clone(),
            u_multiplicities: self.u_multiplicities.clone(),
            v_multiplicities: self.v_multiplicities.clone(),
            u_degree: self.u_degree,
            v_degree: self.v_degree,
            u_periodic: self.u_periodic,
            v_periodic: self.v_periodic,
        }
    }
}

/// An offset surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffsetSurface {
    pub basis_surface: Box<Surface>,
    pub distance: f64,
}

impl OffsetSurface {
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let base_pt = self.basis_surface.point_at(u, v);
        let normal = self.basis_surface.normal_at(u, v);
        base_pt + normal.to_dvec3() * self.distance
    }

    pub fn transform(&self, tf: &Transform3) -> OffsetSurface {
        OffsetSurface {
            basis_surface: Box::new(self.basis_surface.transform(tf)),
            distance: self.distance * tf.scale(),
        }
    }
}

/// A rectangularly trimmed surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RectangularTrimmedSurface {
    pub basis_surface: Box<Surface>,
    pub u1: f64,
    pub u2: f64,
    pub v1: f64,
    pub v2: f64,
}
