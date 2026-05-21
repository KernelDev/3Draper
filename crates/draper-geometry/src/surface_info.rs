//! Surface periodicity and metric information.
//!
//! Provides analysis of surface parameterization: periodicity detection,
//! UV bounds, metric tensor computation, and curvature estimation.
//! This information drives the triangulation pipeline decisions:
//! seam handling, adaptive discretization, and quality control.

use crate::surface::Surface;

/// Information about a surface's parameterization.
#[derive(Debug, Clone)]
pub struct SurfaceInfo {
    /// Whether the surface is periodic in U direction.
    pub u_periodic: bool,
    /// Whether the surface is periodic in V direction.
    pub v_periodic: bool,
    /// U parameter range.
    pub u_range: (f64, f64),
    /// V parameter range.
    pub v_range: (f64, f64),
    /// Period in U direction (if periodic).
    pub u_period: f64,
    /// Period in V direction (if periodic).
    pub v_period: f64,
}

impl SurfaceInfo {
    /// Analyze a surface and extract its parameterization info.
    pub fn from_surface(surface: &Surface) -> Self {
        match surface {
            Surface::Plane(_) => SurfaceInfo {
                u_periodic: false,
                v_periodic: false,
                u_range: (-1e6, 1e6),
                v_range: (-1e6, 1e6),
                u_period: 0.0,
                v_period: 0.0,
            },
            Surface::CylindricalSurface(_) => SurfaceInfo {
                u_periodic: true,
                v_periodic: false,
                u_range: (0.0, 2.0 * std::f64::consts::PI),
                v_range: (-1e6, 1e6),
                u_period: 2.0 * std::f64::consts::PI,
                v_period: 0.0,
            },
            Surface::ConicalSurface(_) => SurfaceInfo {
                u_periodic: true,
                v_periodic: false,
                u_range: (0.0, 2.0 * std::f64::consts::PI),
                v_range: (-1e6, 1e6),
                u_period: 2.0 * std::f64::consts::PI,
                v_period: 0.0,
            },
            Surface::SphericalSurface(_) => SurfaceInfo {
                u_periodic: true,
                v_periodic: false,
                u_range: (0.0, 2.0 * std::f64::consts::PI),
                v_range: (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
                u_period: 2.0 * std::f64::consts::PI,
                v_period: 0.0,
            },
            Surface::ToroidalSurface(_) => SurfaceInfo {
                u_periodic: true,
                v_periodic: true,
                u_range: (0.0, 2.0 * std::f64::consts::PI),
                v_range: (0.0, 2.0 * std::f64::consts::PI),
                u_period: 2.0 * std::f64::consts::PI,
                v_period: 2.0 * std::f64::consts::PI,
            },
            Surface::SurfaceOfRevolution(_) => SurfaceInfo {
                u_periodic: true,
                v_periodic: false,
                u_range: (0.0, 2.0 * std::f64::consts::PI),
                v_range: (-1e3, 1e3),
                u_period: 2.0 * std::f64::consts::PI,
                v_period: 0.0,
            },
            Surface::SurfaceOfLinearExtrusion(_) => SurfaceInfo {
                u_periodic: false,
                v_periodic: false,
                u_range: (-1e3, 1e3),
                v_range: (-1e3, 1e3),
                u_period: 0.0,
                v_period: 0.0,
            },
            Surface::BSplineSurface(bs) => SurfaceInfo {
                u_periodic: bs.u_periodic,
                v_periodic: bs.v_periodic,
                u_range: (
                    bs.u_knots.first().copied().unwrap_or(0.0),
                    bs.u_knots.last().copied().unwrap_or(1.0),
                ),
                v_range: (
                    bs.v_knots.first().copied().unwrap_or(0.0),
                    bs.v_knots.last().copied().unwrap_or(1.0),
                ),
                u_period: if bs.u_periodic {
                    bs.u_knots.last().copied().unwrap_or(1.0) - bs.u_knots.first().copied().unwrap_or(0.0)
                } else {
                    0.0
                },
                v_period: if bs.v_periodic {
                    bs.v_knots.last().copied().unwrap_or(1.0) - bs.v_knots.first().copied().unwrap_or(0.0)
                } else {
                    0.0
                },
            },
            Surface::OffsetSurface(os) => SurfaceInfo::from_surface(&os.basis_surface),
            Surface::RectangularTrimmedSurface(rs) => {
                let base = SurfaceInfo::from_surface(&rs.basis_surface);
                SurfaceInfo {
                    u_range: (rs.u1, rs.u2),
                    v_range: (rs.v1, rs.v2),
                    ..base
                }
            }
        }
    }

    /// Returns true if the surface has any periodic direction.
    pub fn is_periodic(&self) -> bool {
        self.u_periodic || self.v_periodic
    }
}

/// Metric tensor information at a point on a surface.
///
/// The first fundamental form coefficients E, F, G describe how
/// the surface stretches and shears in parameter space:
/// - E = dS/du · dS/du (stretch in U)
/// - F = dS/du · dS/dv (shear between U and V)
/// - G = dS/dv · dS/dv (stretch in V)
///
/// The area element dA = sqrt(EG - F²) du dv.
/// The Jacobian determinant is sqrt(EG - F²).
#[derive(Debug, Clone, Copy)]
pub struct SurfaceMetric {
    /// E = |dS/du|²
    pub e: f64,
    /// F = dS/du · dS/dv
    pub f: f64,
    /// G = |dS/dv|²
    pub g: f64,
    /// sqrt(EG - F²) — area scaling factor
    pub area_element: f64,
    /// Maximum stretch factor (max eigenvalue of metric tensor)
    pub max_stretch: f64,
    /// Minimum stretch factor (min eigenvalue of metric tensor)
    pub min_stretch: f64,
    /// Anisotropy ratio = max_stretch / min_stretch (>= 1)
    pub anisotropy: f64,
}

impl SurfaceMetric {
    /// Compute the surface metric at a given UV point using finite differences.
    pub fn compute(surface: &Surface, u: f64, v: f64) -> Self {
        let h = 1e-6;

        let p = surface.point_at(u, v);
        let pu = surface.point_at(u + h, v);
        let pv = surface.point_at(u, v + h);

        let du = pu - p;
        let dv = pv - p;

        let e = du.dot(du);
        let f = du.dot(dv);
        let g = dv.dot(dv);

        let det = e * g - f * f;
        let area_element = det.max(0.0).sqrt();

        // Eigenvalues of the 2x2 metric tensor [E F; F G]
        let trace = e + g;
        let disc = ((e - g) * (e - g) + 4.0 * f * f).max(0.0).sqrt();
        let max_stretch = ((trace + disc) / 2.0).max(0.0).sqrt();
        let min_stretch = ((trace - disc) / 2.0).max(1e-20).sqrt();
        let anisotropy = max_stretch / min_stretch;

        SurfaceMetric {
            e,
            f,
            g,
            area_element,
            max_stretch,
            min_stretch,
            anisotropy,
        }
    }

    /// Compute the metric over a regular grid in UV space.
    pub fn compute_grid(
        surface: &Surface,
        u_range: (f64, f64),
        v_range: (f64, f64),
        n_u: usize,
        n_v: usize,
    ) -> Vec<Vec<SurfaceMetric>> {
        let mut grid = Vec::with_capacity(n_v);
        for j in 0..n_v {
            let mut row = Vec::with_capacity(n_u);
            let v = v_range.0 + (v_range.1 - v_range.0) * j as f64 / (n_v - 1).max(1) as f64;
            for i in 0..n_u {
                let u = u_range.0 + (u_range.1 - u_range.0) * i as f64 / (n_u - 1).max(1) as f64;
                row.push(SurfaceMetric::compute(surface, u, v));
            }
            grid.push(row);
        }
        grid
    }
}

/// Surface curvature information at a point.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceCurvature {
    /// Mean curvature.
    pub mean: f64,
    /// Gaussian curvature.
    pub gaussian: f64,
    /// Maximum absolute curvature (max of |k1|, |k2|).
    pub max_abs_curvature: f64,
}

impl SurfaceCurvature {
    /// Estimate surface curvature at a UV point using finite differences.
    pub fn estimate(surface: &Surface, u: f64, v: f64) -> Self {
        let h = 1e-4;

        let p = surface.point_at(u, v).to_dvec3();

        // First derivatives
        let pu = surface.point_at(u + h, v).to_dvec3();
        let pv = surface.point_at(u, v + h).to_dvec3();
        let du = (pu - p) / h;
        let dv = (pv - p) / h;

        // Second derivatives
        let puu = (surface.point_at(u + h, v).to_dvec3()
            - 2.0 * p
            + surface.point_at(u - h, v).to_dvec3())
            / (h * h);
        let pvv = (surface.point_at(u, v + h).to_dvec3()
            - 2.0 * p
            + surface.point_at(u, v - h).to_dvec3())
            / (h * h);
        let puv = (surface.point_at(u + h, v + h).to_dvec3()
            - surface.point_at(u + h, v - h).to_dvec3()
            - surface.point_at(u - h, v + h).to_dvec3()
            + surface.point_at(u - h, v - h).to_dvec3())
            / (4.0 * h * h);

        // Normal vector
        let normal = du.cross(dv);
        let n_len = normal.length();
        if n_len < 1e-20 {
            return SurfaceCurvature {
                mean: 0.0,
                gaussian: 0.0,
                max_abs_curvature: 0.0,
            };
        }
        let n = normal / n_len;

        // Second fundamental form
        let l = puu.dot(n);
        let m = puv.dot(n);
        let nn = pvv.dot(n);

        // First fundamental form
        let e = du.dot(du);
        let f = du.dot(dv);
        let g = dv.dot(dv);

        let det_fg = e * g - f * f;
        if det_fg.abs() < 1e-30 {
            return SurfaceCurvature {
                mean: 0.0,
                gaussian: 0.0,
                max_abs_curvature: 0.0,
            };
        }

        let gaussian = (l * nn - m * m) / det_fg;
        let mean = (e * nn - 2.0 * f * m + g * l) / (2.0 * det_fg);

        // Principal curvatures
        let disc = (mean * mean - gaussian).max(0.0).sqrt();
        let k1 = mean + disc;
        let k2 = mean - disc;
        let max_abs = k1.abs().max(k2.abs());

        SurfaceCurvature {
            mean,
            gaussian,
            max_abs_curvature: max_abs,
        }
    }
}

/// Represents a UV domain for a face, including trimming information.
#[derive(Debug, Clone)]
pub struct UVDomain {
    /// U parameter range.
    pub u_range: (f64, f64),
    /// V parameter range.
    pub v_range: (f64, f64),
    /// Whether the surface is periodic in U.
    pub u_periodic: bool,
    /// Whether the surface is periodic in V.
    pub v_periodic: bool,
}

impl UVDomain {
    pub fn from_surface_info(info: &SurfaceInfo) -> Self {
        Self {
            u_range: info.u_range,
            v_range: info.v_range,
            u_periodic: info.u_periodic,
            v_periodic: info.v_periodic,
        }
    }

    /// Normalize a UV point to be within the parameter range.
    /// For periodic surfaces, wraps the coordinate.
    pub fn normalize_uv(&self, u: f64, v: f64) -> (f64, f64) {
        let u = if self.u_periodic && self.u_range.1 > self.u_range.0 {
            let period = self.u_range.1 - self.u_range.0;
            let mut u = u;
            while u < self.u_range.0 {
                u += period;
            }
            while u > self.u_range.1 {
                u -= period;
            }
            u
        } else {
            u.clamp(self.u_range.0, self.u_range.1)
        };

        let v = if self.v_periodic && self.v_range.1 > self.v_range.0 {
            let period = self.v_range.1 - self.v_range.0;
            let mut v = v;
            while v < self.v_range.0 {
                v += period;
            }
            while v > self.v_range.1 {
                v -= period;
            }
            v
        } else {
            v.clamp(self.v_range.0, self.v_range.1)
        };

        (u, v)
    }
}
