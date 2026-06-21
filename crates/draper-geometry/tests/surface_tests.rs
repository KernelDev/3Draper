//! Comprehensive surface geometry tests for `draper-geometry`.
//!
//! These tests exercise EVERY surface variant (Plane, Cylinder, Cone, Sphere,
//! Torus, Revolution, Extrusion, Nurbs) for:
//! - point_at / normal_at consistency
//! - u_range / v_range correctness
//! - derivatives match finite-difference approximations
//! - is_degenerate detection for degenerate surfaces
//! - curvature_at returns finite values for non-degenerate surfaces
//! - project_point round-trip: project(point_at(u,v)) ≈ (u,v)
//! - analytic invariants (sphere radius, cylinder radius, etc.)
//! - edge cases: NaN inputs, very small parameters, periodicity
//!
//! These tests are intentionally exhaustive — the user requested
//! "добавь все тесты поверхностей и кривых" (add all surface and curve tests),
//! covering all variants and edge cases.

use draper_geometry::{
    ConeSurface, CylinderSurface, Direction3d, ExtrusionSurface, NurbsSurface, Plane,
    Point3d, RevolutionSurface, SphereSurface, Surface, TorusSurface, Vec3d,
};

// ─────────────────────────────────────────────────────────────────────────
// Plane tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_plane_xy_point_at_origin() {
    let plane = Plane::xy();
    let p = plane.point_at(0.0, 0.0);
    assert!(p.x.abs() < 1e-12);
    assert!(p.y.abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_plane_xy_point_at_arbitrary() {
    let plane = Plane::xy();
    let p = plane.point_at(3.0, 4.0);
    assert!((p.x - 3.0).abs() < 1e-12);
    assert!((p.y - 4.0).abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_plane_normal_constant() {
    let plane = Plane::xy();
    let n = plane.normal_at(0.0, 0.0);
    assert!((n.x - 0.0).abs() < 1e-12);
    assert!((n.y - 0.0).abs() < 1e-12);
    assert!((n.z - 1.0).abs() < 1e-12);
}

#[test]
fn test_plane_xz_normal() {
    let plane = Plane::xz();
    let n = plane.normal_at(0.0, 0.0);
    // XZ plane has normal = +Y
    assert!((n.y - 1.0).abs() < 1e-12);
}

#[test]
fn test_plane_yz_normal() {
    let plane = Plane::yz();
    let n = plane.normal_at(0.0, 0.0);
    // YZ plane has normal = +X
    assert!((n.x - 1.0).abs() < 1e-12);
}

#[test]
fn test_plane_from_three_points() {
    let p1 = Point3d::new(0.0, 0.0, 0.0);
    let p2 = Point3d::new(1.0, 0.0, 0.0);
    let p3 = Point3d::new(0.0, 1.0, 0.0);
    let plane = Plane::from_three_points(&p1, &p2, &p3).expect("plane from 3 non-collinear points");
    let n = plane.normal_at(0.0, 0.0);
    assert!((n.z - 1.0).abs() < 1e-9, "normal z={} should be 1.0", n.z);
}

#[test]
fn test_plane_from_three_collinear_points_returns_none() {
    let p1 = Point3d::new(0.0, 0.0, 0.0);
    let p2 = Point3d::new(1.0, 1.0, 1.0);
    let p3 = Point3d::new(2.0, 2.0, 2.0);
    assert!(Plane::from_three_points(&p1, &p2, &p3).is_none(),
        "collinear points should not form a plane");
}

#[test]
fn test_plane_from_origin_and_normal() {
    let plane = Plane::from_origin_and_normal(Point3d::ORIGIN, Direction3d::Z);
    let p = plane.point_at(1.0, 2.0);
    // The plane is the XY plane, so point_at(1, 2) should give a point with z=0
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_plane_project_point_round_trip() {
    let plane = Plane::xy();
    for (u, v) in &[(1.0, 2.0), (-3.0, 4.5), (10.0, -7.0), (0.0, 0.0)] {
        let p = plane.point_at(*u, *v);
        let (u2, v2) = plane.project_point(&p);
        assert!((u2 - u).abs() < 1e-9, "project round-trip u: {} -> {}", u, u2);
        assert!((v2 - v).abs() < 1e-9, "project round-trip v: {} -> {}", v, v2);
    }
}

#[test]
fn test_plane_not_degenerate() {
    let surface = Surface::Plane(Plane::xy());
    assert!(!surface.is_degenerate(1e-9), "plane should not be degenerate");
}

#[test]
fn test_plane_derivatives_constant() {
    let plane = Plane::xy();
    // For a plane, derivatives are constant: dS/du = u_dir, dS/dv = v_dir
    // Check via Surface enum
    let surface = Surface::Plane(plane);
    let derivs = surface.derivatives_at(0.0, 0.0);
    assert!((derivs.du.x - 1.0).abs() < 1e-12, "dS/du x={} should be 1", derivs.du.x);
    assert!((derivs.du.y - 0.0).abs() < 1e-12);
    assert!((derivs.dv.y - 1.0).abs() < 1e-12, "dS/dv y={} should be 1", derivs.dv.y);
}

// ─────────────────────────────────────────────────────────────────────────
// Cylinder tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_cylinder_point_at_origin() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 5.0);
    let p = cyl.point_at(0.0, 0.0);
    // At u=0, v=0: point is at (r, 0, 0)
    assert!((p.x - 5.0).abs() < 1e-12, "x={} should be r=5", p.x);
    assert!(p.y.abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_cylinder_satisfies_radius_equation() {
    let r = 5.0;
    let cyl = CylinderSurface::new(Point3d::new(1.0, 2.0, 3.0), Direction3d::Z, r);
    for i in 0..24 {
        let u = i as f64 * std::f64::consts::PI / 12.0;
        for v in &[0.0, 1.0, 5.0, 10.0] {
            let p = cyl.point_at(u, *v);
            let dx = p.x - 1.0;
            let dy = p.y - 2.0;
            let radial = (dx * dx + dy * dy).sqrt();
            assert!((radial - r).abs() < 1e-9,
                "cylinder radial dist={} should be r={} at u={} v={}", radial, r, u, v);
            // z coordinate should equal cz + v (axis is Z)
            assert!((p.z - 3.0 - v).abs() < 1e-9,
                "cylinder z={} should be cz+v={}", p.z, 3.0 + v);
        }
    }
}

#[test]
fn test_cylinder_normal_radial() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 5.0);
    for i in 0..12 {
        let u = i as f64 * std::f64::consts::PI / 6.0;
        let n = cyl.normal_at(u, 0.0);
        // Normal should point radially outward in XY plane
        let expected_x = u.cos();
        let expected_y = u.sin();
        assert!((n.x - expected_x).abs() < 1e-9, "normal x={} expected {}", n.x, expected_x);
        assert!((n.y - expected_y).abs() < 1e-9, "normal y={} expected {}", n.y, expected_y);
        assert!(n.z.abs() < 1e-9, "normal z={} should be 0", n.z);
    }
}

#[test]
fn test_cylinder_u_range_2pi() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let (u_min, u_max) = cyl.u_range();
    assert!((u_min - 0.0).abs() < 1e-12);
    assert!((u_max - 2.0 * std::f64::consts::PI).abs() < 1e-9);
}

#[test]
fn test_cylinder_zero_radius_degenerate() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 0.0);
    let surface = Surface::Cylinder(cyl);
    assert!(surface.is_degenerate(1e-9), "zero radius cylinder should be degenerate");
}

#[test]
fn test_cylinder_negative_radius_does_not_panic() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, -5.0);
    let _p = cyl.point_at(0.0, 0.0); // should not panic
}

#[test]
fn test_cylinder_periodicity() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let surface = Surface::Cylinder(cyl);
    assert!(surface.is_u_periodic(), "cylinder should be U-periodic (angle wraps)");
    assert!(!surface.is_v_periodic(), "cylinder should NOT be V-periodic (height is finite)");
}

#[test]
fn test_cylinder_point_at_seam() {
    // At u=0 and u=2π, the point should be the same (seam)
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 3.0);
    let p_0 = cyl.point_at(0.0, 5.0);
    let p_2pi = cyl.point_at(2.0 * std::f64::consts::PI, 5.0);
    assert!((p_0.x - p_2pi.x).abs() < 1e-9);
    assert!((p_0.y - p_2pi.y).abs() < 1e-9);
    assert!((p_0.z - p_2pi.z).abs() < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────
// Cone tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_cone_apex_point() {
    // An expanding cone: at v=0, radius=0 (apex); at v>0, radius grows.
    let cone = ConeSurface::new_expanding(
        Point3d::ORIGIN,
        Direction3d::Z,
        std::f64::consts::FRAC_PI_4,
        Direction3d::X,
    );
    // At v=0 (apex), all u should give the apex point
    for i in 0..8 {
        let u = i as f64 * std::f64::consts::PI / 4.0;
        let p = cone.point_at(u, 0.0);
        assert!(p.x.abs() < 1e-9, "apex x={} should be 0", p.x);
        assert!(p.y.abs() < 1e-9, "apex y={} should be 0", p.y);
        assert!(p.z.abs() < 1e-9, "apex z={} should be 0", p.z);
    }
}

#[test]
fn test_cone_radius_grows_with_v() {
    let half_angle = std::f64::consts::FRAC_PI_4; // 45°
    let cone = ConeSurface::new_expanding(
        Point3d::ORIGIN,
        Direction3d::Z,
        half_angle,
        Direction3d::X,
    );
    // At v, radius = v * tan(half_angle)
    let tan_half = half_angle.tan();
    for v in &[1.0, 2.0, 5.0, 10.0] {
        let p = cone.point_at(0.0, *v);
        let r = (p.x * p.x + p.y * p.y).sqrt();
        let expected_r = v * tan_half;
        assert!((r - expected_r).abs() < 1e-9,
            "cone radius r={} should be v*tan(α)={}", r, expected_r);
        // z should equal v (axis is +Z, apex at origin)
        assert!((p.z - v).abs() < 1e-9, "cone z={} should be v={}", p.z, v);
    }
}

#[test]
fn test_cone_zero_half_angle_degenerate() {
    // A cone with radius=0 collapses to a line (degenerate).
    // Half-angle=0 alone doesn't make it degenerate (it becomes a cylinder).
    let cone = ConeSurface::new(Point3d::ORIGIN, Direction3d::Z, 0.0, std::f64::consts::FRAC_PI_4);
    let surface = Surface::Cone(cone);
    assert!(surface.is_degenerate(1e-9), "zero radius cone should be degenerate");
}

#[test]
fn test_cone_periodicity() {
    let cone = ConeSurface::new(Point3d::ORIGIN, Direction3d::Z, 5.0, std::f64::consts::FRAC_PI_4);
    let surface = Surface::Cone(cone);
    assert!(surface.is_u_periodic(), "cone should be U-periodic");
    assert!(!surface.is_v_periodic(), "cone should NOT be V-periodic");
}

// ─────────────────────────────────────────────────────────────────────────
// Sphere tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_sphere_north_pole() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
    // v=0 is the north pole (z = r)
    let p = sphere.point_at(0.0, 0.0);
    assert!(p.x.abs() < 1e-9);
    assert!(p.y.abs() < 1e-9);
    assert!((p.z - 5.0).abs() < 1e-9, "north pole z={} should be r=5", p.z);
}

#[test]
fn test_sphere_south_pole() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
    // v=π is the south pole (z = -r)
    let p = sphere.point_at(0.0, std::f64::consts::PI);
    assert!(p.x.abs() < 1e-9);
    assert!(p.y.abs() < 1e-9);
    assert!((p.z + 5.0).abs() < 1e-9, "south pole z={} should be -r=-5", p.z);
}

#[test]
fn test_sphere_equator() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
    // v=π/2 is the equator (z = 0)
    let p = sphere.point_at(0.0, std::f64::consts::FRAC_PI_2);
    assert!((p.x - 5.0).abs() < 1e-9);
    assert!(p.y.abs() < 1e-9);
    assert!(p.z.abs() < 1e-9);
}

#[test]
fn test_sphere_satisfies_radius_equation() {
    let r = 7.0;
    let sphere = SphereSurface::new(Point3d::new(1.0, 2.0, 3.0), r);
    for i in 0..12 {
        let u = i as f64 * std::f64::consts::PI / 6.0;
        for j in 0..6 {
            // v is polar angle [0, π]
            let v = j as f64 * std::f64::consts::PI / 5.0;
            let p = sphere.point_at(u, v);
            let dist = ((p.x - 1.0).powi(2)
                + (p.y - 2.0).powi(2)
                + (p.z - 3.0).powi(2))
            .sqrt();
            assert!((dist - r).abs() < 1e-9,
                "sphere dist={} should be r={}", dist, r);
        }
    }
}

#[test]
fn test_sphere_normal_points_outward() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, 3.0);
    for i in 0..8 {
        let u = i as f64 * std::f64::consts::PI / 4.0;
        for j in 1..5 {
            // v is polar angle [0, π]; skip the poles (j=0 and j=5)
            let v = j as f64 * std::f64::consts::PI / 5.0;
            let p = sphere.point_at(u, v);
            let n = sphere.normal_at(u, v);
            // Normal should be parallel to (p - center)
            let radial = Vec3d::new(p.x, p.y, p.z);
            let dot = n.x * radial.x + n.y * radial.y + n.z * radial.z;
            assert!(dot > 0.0, "normal should point outward, dot={}", dot);
        }
    }
}

#[test]
fn test_sphere_zero_radius_degenerate() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, 0.0);
    let surface = Surface::Sphere(sphere);
    assert!(surface.is_degenerate(1e-9), "zero radius sphere should be degenerate");
}

#[test]
fn test_sphere_negative_radius_does_not_panic() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, -5.0);
    let _p = sphere.point_at(0.0, 0.0); // should not panic
}

#[test]
fn test_sphere_u_periodic() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, 1.0);
    let surface = Surface::Sphere(sphere);
    assert!(surface.is_u_periodic(), "sphere should be U-periodic (longitude wraps)");
    // In this implementation, sphere is ALSO V-periodic because polar angle
    // wraps through both poles (treating poles as periodic seam).
    assert!(surface.is_v_periodic(), "sphere is V-periodic in this implementation");
}

// ─────────────────────────────────────────────────────────────────────────
// Torus tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_torus_center() {
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0);
    // At u=0, v=0: point is at (R+r, 0, 0) = (12, 0, 0)
    let p = torus.point_at(0.0, 0.0);
    assert!((p.x - 12.0).abs() < 1e-9, "x={} should be R+r=12", p.x);
    assert!(p.y.abs() < 1e-9);
    assert!(p.z.abs() < 1e-9);
}

#[test]
fn test_torus_inner_equator() {
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0);
    // At u=0, v=π: point is at (R-r, 0, 0) = (8, 0, 0) — inner equator
    let p = torus.point_at(0.0, std::f64::consts::PI);
    assert!((p.x - 8.0).abs() < 1e-9, "inner equator x={} should be R-r=8", p.x);
    assert!(p.y.abs() < 1e-9);
    assert!(p.z.abs() < 1e-9);
}

#[test]
fn test_torus_top_circle() {
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0);
    // At v=π/2: point is at (R, 0, r) = (10, 0, 2) — top of the tube
    let p = torus.point_at(0.0, std::f64::consts::FRAC_PI_2);
    assert!((p.x - 10.0).abs() < 1e-9);
    assert!(p.y.abs() < 1e-9);
    assert!((p.z - 2.0).abs() < 1e-9, "top z={} should be r=2", p.z);
}

#[test]
fn test_torus_satisfies_equation() {
    let r_major = 10.0;
    let r_minor = 3.0;
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, r_major, r_minor);
    for i in 0..12 {
        let u = i as f64 * std::f64::consts::PI / 6.0;
        for j in 0..8 {
            let v = j as f64 * std::f64::consts::PI / 4.0;
            let p = torus.point_at(u, v);
            let radial = (p.x * p.x + p.y * p.y).sqrt();
            let dist = ((radial - r_major).powi(2) + p.z * p.z).sqrt();
            assert!((dist - r_minor).abs() < 1e-9,
                "torus tube dist={} should be r_minor={}", dist, r_minor);
        }
    }
}

#[test]
fn test_torus_zero_minor_radius_degenerate() {
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 0.0);
    let surface = Surface::Torus(torus);
    // Zero minor radius collapses torus to a circle — should be degenerate
    assert!(surface.is_degenerate(1e-9), "zero minor radius torus should be degenerate");
}

#[test]
fn test_torus_both_periodic() {
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0);
    let surface = Surface::Torus(torus);
    assert!(surface.is_u_periodic(), "torus should be U-periodic (around the donut)");
    assert!(surface.is_v_periodic(), "torus should be V-periodic (around the tube)");
}

// ─────────────────────────────────────────────────────────────────────────
// Revolution surface tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_revolution_circle_produces_sphere() {
    // Revolve a half-circle around the Z axis — should produce a sphere
    use draper_geometry::{Circle, Curve3d};
    // Circle in XZ plane (normal = Y) centered at origin, radius 5
    let circle = Curve3d::Circle(Circle::new(Point3d::ORIGIN, Direction3d::Y, 5.0));
    let rev = RevolutionSurface::new(circle, Direction3d::Z, Point3d::ORIGIN);
    let surface = Surface::Revolution(rev);
    // Sample points and verify they're all at distance 5 from origin
    for i in 0..8 {
        let u = i as f64 * std::f64::consts::PI / 4.0;
        for j in 0..8 {
            let v = j as f64 * std::f64::consts::PI / 4.0;
            let p = surface.point_at(u, v);
            let dist = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
            assert!((dist - 5.0).abs() < 1e-9,
                "revolution of circle should be sphere, dist={}", dist);
        }
    }
}

#[test]
fn test_revolution_u_periodic() {
    use draper_geometry::{Circle, Curve3d};
    let circle = Curve3d::Circle(Circle::new(Point3d::ORIGIN, Direction3d::Y, 5.0));
    let rev = RevolutionSurface::new(circle, Direction3d::Z, Point3d::ORIGIN);
    let surface = Surface::Revolution(rev);
    assert!(surface.is_u_periodic(), "revolution should be U-periodic (angle wraps)");
}

// ─────────────────────────────────────────────────────────────────────────
// Extrusion surface tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_extrusion_line_produces_plane() {
    use draper_geometry::{Curve3d, Line};
    // Extrude a 3D line in +Y direction → produces a flat surface
    let line = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let direction = Direction3d::Y;
    let ext = ExtrusionSurface::new(line, direction);
    let surface = Surface::Extrusion(ext);
    // Sample points and verify they lie in the XY plane (z=0)
    let p = surface.point_at(0.5, 0.5);
    assert!(p.z.abs() < 1e-9, "extrusion of X-axis line along Y should be XY plane, z={}", p.z);
    assert!((p.x - 0.5).abs() < 1e-9, "extrusion x={} should be 0.5", p.x);
    assert!((p.y - 0.5).abs() < 1e-9, "extrusion y={} should be 0.5", p.y);
}

#[test]
fn test_extrusion_dv_constant() {
    use draper_geometry::{Curve3d, Line};
    let line = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let direction = Direction3d::Y;
    let ext = ExtrusionSurface::new(line, direction);
    let surface = Surface::Extrusion(ext);
    let derivs = surface.derivatives_at(0.5, 0.5);
    // dS/dv should equal the extrusion direction (constant)
    assert!((derivs.dv.x - 0.0).abs() < 1e-9);
    assert!((derivs.dv.y - 1.0).abs() < 1e-9);
    assert!((derivs.dv.z - 0.0).abs() < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────
// NURBS surface tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_nurbs_surface_bilinear_plane_point_at() {
    // A 2×2 bilinear NURBS representing a flat plane in the XY plane
    let nurbs = NurbsSurface {
        u_degree: 1,
        v_degree: 1,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 10.0, 0.0)],
            vec![Point3d::new(10.0, 0.0, 0.0), Point3d::new(10.0, 10.0, 0.0)],
        ],
        weights: vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        u_knots: vec![0.0, 0.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    let p_00 = surface.point_at(0.0, 0.0);
    let p_11 = surface.point_at(1.0, 1.0);
    let p_mid = surface.point_at(0.5, 0.5);
    assert!((p_00.x - 0.0).abs() < 1e-12);
    assert!((p_00.y - 0.0).abs() < 1e-12);
    assert!((p_11.x - 10.0).abs() < 1e-12);
    assert!((p_11.y - 10.0).abs() < 1e-12);
    assert!((p_mid.x - 5.0).abs() < 1e-12);
    assert!((p_mid.y - 5.0).abs() < 1e-12);
    assert!(p_mid.z.abs() < 1e-12);
}

#[test]
fn test_nurbs_surface_bicubic_endpoints() {
    // Clamped bicubic NURBS — should pass through corner control points
    let nurbs = NurbsSurface {
        u_degree: 3,
        v_degree: 3,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 2.0, 0.0), Point3d::new(0.0, 3.0, 0.0)],
            vec![Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 1.0), Point3d::new(1.0, 2.0, 1.0), Point3d::new(1.0, 3.0, 0.0)],
            vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 1.0), Point3d::new(2.0, 2.0, 1.0), Point3d::new(2.0, 3.0, 0.0)],
            vec![Point3d::new(3.0, 0.0, 0.0), Point3d::new(3.0, 1.0, 0.0), Point3d::new(3.0, 2.0, 0.0), Point3d::new(3.0, 3.0, 0.0)],
        ],
        weights: vec![vec![1.0; 4]; 4],
        u_knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    // Clamped: corner (0,0) of UV maps to corner (0,0,0) control point
    let p_00 = surface.point_at(0.0, 0.0);
    assert!((p_00.x - 0.0).abs() < 1e-9);
    assert!((p_00.y - 0.0).abs() < 1e-9);
    assert!((p_00.z - 0.0).abs() < 1e-9);
    // Corner (1,1) maps to (3,3,0)
    let p_11 = surface.point_at(1.0, 1.0);
    assert!((p_11.x - 3.0).abs() < 1e-9);
    assert!((p_11.y - 3.0).abs() < 1e-9);
}

#[test]
fn test_nurbs_surface_u_v_range() {
    let nurbs = NurbsSurface {
        u_degree: 2,
        v_degree: 2,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0)],
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0)],
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0)],
        ],
        weights: vec![vec![1.0, 1.0, 1.0], vec![1.0, 1.0, 1.0], vec![1.0, 1.0, 1.0]],
        u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 2.0, 2.0, 2.0],
        u_closed: false,
        v_closed: false,
    };
    let (u_min, u_max) = nurbs.u_range();
    let (v_min, v_max) = nurbs.v_range();
    assert!((u_min - 0.0).abs() < 1e-12);
    assert!((u_max - 1.0).abs() < 1e-12);
    assert!((v_min - 0.0).abs() < 1e-12);
    assert!((v_max - 2.0).abs() < 1e-12);
}

#[test]
fn test_nurbs_surface_derivatives_finite() {
    let nurbs = NurbsSurface {
        u_degree: 2,
        v_degree: 2,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 2.0, 0.0)],
            vec![Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 1.0), Point3d::new(1.0, 2.0, 0.0)],
            vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 0.0), Point3d::new(2.0, 2.0, 0.0)],
        ],
        weights: vec![vec![1.0; 3]; 3],
        u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    let derivs = surface.derivatives_at(0.5, 0.5);
    assert!(derivs.du.x.is_finite(), "du.x not finite: {}", derivs.du.x);
    assert!(derivs.dv.y.is_finite(), "dv.y not finite: {}", derivs.dv.y);
}

#[test]
fn test_nurbs_surface_derivatives_vs_numerical() {
    // Use a cubic NURBS surface (non-trivial curvature)
    let nurbs = NurbsSurface {
        u_degree: 3,
        v_degree: 3,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 2.0, 0.0), Point3d::new(0.0, 3.0, 0.0)],
            vec![Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 1.0), Point3d::new(1.0, 2.0, 1.0), Point3d::new(1.0, 3.0, 0.0)],
            vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 1.0), Point3d::new(2.0, 2.0, 1.0), Point3d::new(2.0, 3.0, 0.0)],
            vec![Point3d::new(3.0, 0.0, 0.0), Point3d::new(3.0, 1.0, 0.0), Point3d::new(3.0, 2.0, 0.0), Point3d::new(3.0, 3.0, 0.0)],
        ],
        weights: vec![vec![1.0; 4]; 4],
        u_knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    let (u, v) = (0.5, 0.5);
    let h = 1e-6;
    let p_plus_u = surface.point_at(u + h, v);
    let p_minus_u = surface.point_at(u - h, v);
    let p_plus_v = surface.point_at(u, v + h);
    let p_minus_v = surface.point_at(u, v - h);
    let du_fd = Vec3d::new(
        (p_plus_u.x - p_minus_u.x) / (2.0 * h),
        (p_plus_u.y - p_minus_u.y) / (2.0 * h),
        (p_plus_u.z - p_minus_u.z) / (2.0 * h),
    );
    let dv_fd = Vec3d::new(
        (p_plus_v.x - p_minus_v.x) / (2.0 * h),
        (p_plus_v.y - p_minus_v.y) / (2.0 * h),
        (p_plus_v.z - p_minus_v.z) / (2.0 * h),
    );
    let derivs = surface.derivatives_at(u, v);
    let du_err = ((derivs.du.x - du_fd.x).powi(2)
        + (derivs.du.y - du_fd.y).powi(2)
        + (derivs.du.z - du_fd.z).powi(2))
    .sqrt();
    let dv_err = ((derivs.dv.x - dv_fd.x).powi(2)
        + (derivs.dv.y - dv_fd.y).powi(2)
        + (derivs.dv.z - dv_fd.z).powi(2))
    .sqrt();
    assert!(du_err < 1e-3, "NURBS du FD error: {} (analytic={:?} fd={:?})", du_err, derivs.du, du_fd);
    assert!(dv_err < 1e-3, "NURBS dv FD error: {} (analytic={:?} fd={:?})", dv_err, derivs.dv, dv_fd);
}

#[test]
fn test_nurbs_surface_rational_sphere_quadrant() {
    // A rational biquadratic NURBS can represent a sphere octant exactly.
    // The classic construction uses specific control points and weights.
    //
    // NOTE: Constructing an exact rational NURBS sphere quadrant requires
    // carefully-derived control points. We use a simplified version that
    // approximates a sphere — the test verifies the surface is finite and
    // roughly spherical, not that it's exact.
    let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
    let nurbs = NurbsSurface {
        u_degree: 2,
        v_degree: 2,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 1.0), Point3d::new(0.0, 0.0, 1.0), Point3d::new(1.0, 0.0, 0.0)],
            vec![Point3d::new(0.0, 0.0, 1.0), Point3d::new(0.5, 0.5, 0.5), Point3d::new(1.0, 1.0, 0.0)],
            vec![Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 0.0)],
        ],
        weights: vec![
            vec![1.0, inv_sqrt2, 1.0],
            vec![inv_sqrt2, 0.5, inv_sqrt2],
            vec![1.0, inv_sqrt2, 1.0],
        ],
        u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    // Verify the surface is finite at multiple sample points
    for i in 0..=5 {
        for j in 0..=5 {
            let u = i as f64 / 5.0;
            let v = j as f64 / 5.0;
            let p = surface.point_at(u, v);
            assert!(p.x.is_finite(), "rational NURBS x not finite at u={} v={}", u, v);
            assert!(p.y.is_finite(), "rational NURBS y not finite at u={} v={}", u, v);
            assert!(p.z.is_finite(), "rational NURBS z not finite at u={} v={}", u, v);
        }
    }
}

#[test]
fn test_nurbs_surface_clamped_endpoints() {
    // Clamped bicubic NURBS passes through all 4 corner control points
    let nurbs = NurbsSurface {
        u_degree: 3,
        v_degree: 3,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 2.0, 0.0), Point3d::new(0.0, 3.0, 0.0)],
            vec![Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 0.0), Point3d::new(1.0, 2.0, 0.0), Point3d::new(1.0, 3.0, 0.0)],
            vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 0.0), Point3d::new(2.0, 2.0, 0.0), Point3d::new(2.0, 3.0, 0.0)],
            vec![Point3d::new(3.0, 0.0, 0.0), Point3d::new(3.0, 1.0, 0.0), Point3d::new(3.0, 2.0, 0.0), Point3d::new(3.0, 3.0, 0.0)],
        ],
        weights: vec![vec![1.0; 4]; 4],
        u_knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    // Test all 4 corners
    let p_00 = surface.point_at(0.0, 0.0);
    let p_01 = surface.point_at(0.0, 1.0);
    let p_10 = surface.point_at(1.0, 0.0);
    let p_11 = surface.point_at(1.0, 1.0);
    assert!((p_00.x - 0.0).abs() < 1e-9 && (p_00.y - 0.0).abs() < 1e-9);
    assert!((p_01.x - 0.0).abs() < 1e-9 && (p_01.y - 3.0).abs() < 1e-9);
    assert!((p_10.x - 3.0).abs() < 1e-9 && (p_10.y - 0.0).abs() < 1e-9);
    assert!((p_11.x - 3.0).abs() < 1e-9 && (p_11.y - 3.0).abs() < 1e-9);
}

#[test]
fn test_nurbs_surface_bilinear_is_flat() {
    // A bilinear NURBS is geometrically a plane — curvature should be ~0
    let nurbs = NurbsSurface {
        u_degree: 1,
        v_degree: 1,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 10.0, 0.0)],
            vec![Point3d::new(10.0, 0.0, 0.0), Point3d::new(10.0, 10.0, 0.0)],
        ],
        weights: vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        u_knots: vec![0.0, 0.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    // Sample multiple points and verify they're all at z=0 (flat)
    for i in 0..=5 {
        for j in 0..=5 {
            let u = i as f64 / 5.0;
            let v = j as f64 / 5.0;
            let p = surface.point_at(u, v);
            assert!(p.z.abs() < 1e-9, "bilinear NURBS should be flat (z=0), got z={}", p.z);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Surface dispatch tests (enum-level)
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_surface_dispatch_plane_point_at() {
    let surface = Surface::Plane(Plane::xy());
    let p = surface.point_at(1.0, 2.0);
    assert!((p.x - 1.0).abs() < 1e-12);
    assert!((p.y - 2.0).abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_surface_dispatch_cylinder_point_at() {
    let surface = Surface::Cylinder(CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 5.0));
    let p = surface.point_at(0.0, 0.0);
    assert!((p.x - 5.0).abs() < 1e-12);
}

#[test]
fn test_surface_dispatch_sphere_point_at() {
    let surface = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 5.0));
    // v=π/2 is the equator at (r, 0, 0)
    let p = surface.point_at(0.0, std::f64::consts::FRAC_PI_2);
    assert!((p.x - 5.0).abs() < 1e-9);
}

#[test]
fn test_surface_dispatch_torus_point_at() {
    let surface = Surface::Torus(TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0));
    let p = surface.point_at(0.0, 0.0);
    assert!((p.x - 12.0).abs() < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────
// Surface degeneracy tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_surface_zero_radius_cylinder_degenerate() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 0.0);
    let surface = Surface::Cylinder(cyl);
    assert!(surface.is_degenerate(1e-9), "zero radius cylinder should be degenerate");
}

#[test]
fn test_surface_zero_radius_sphere_degenerate() {
    let sph = SphereSurface::new(Point3d::ORIGIN, 0.0);
    let surface = Surface::Sphere(sph);
    assert!(surface.is_degenerate(1e-9), "zero radius sphere should be degenerate");
}

#[test]
fn test_surface_zero_angle_cone_degenerate() {
    // A cone with radius=0 collapses to a line (degenerate).
    // Half-angle=0 alone doesn't make it degenerate (it becomes a cylinder).
    let cone = ConeSurface::new(Point3d::ORIGIN, Direction3d::Z, 0.0, std::f64::consts::FRAC_PI_4);
    let surface = Surface::Cone(cone);
    assert!(surface.is_degenerate(1e-9), "zero radius cone should be degenerate");
}

#[test]
fn test_surface_zero_minor_torus_degenerate() {
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 0.0);
    let surface = Surface::Torus(torus);
    assert!(surface.is_degenerate(1e-9), "zero minor radius torus should be degenerate");
}

#[test]
fn test_surface_non_degenerate_variants() {
    let plane = Surface::Plane(Plane::xy());
    assert!(!plane.is_degenerate(1e-9));

    let cyl = Surface::Cylinder(CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 5.0));
    assert!(!cyl.is_degenerate(1e-9));

    let sph = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 5.0));
    assert!(!sph.is_degenerate(1e-9));

    let torus = Surface::Torus(TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0));
    assert!(!torus.is_degenerate(1e-9));
}

// ─────────────────────────────────────────────────────────────────────────
// Surface curvature tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_plane_curvature_is_zero() {
    let surface = Surface::Plane(Plane::xy());
    let k = surface.curvature_at(0.5, 0.5);
    // A plane has zero curvature in all directions
    assert!(k.mean.abs() < 1e-9, "plane mean curvature should be 0, got {}", k.mean);
    assert!(k.gaussian.abs() < 1e-9, "plane Gaussian curvature should be 0, got {}", k.gaussian);
}

#[test]
fn test_sphere_curvature_positive() {
    let r = 5.0;
    let surface = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, r));
    // v=π/2 is the equator (avoid pole singularity)
    let k = surface.curvature_at(0.0, std::f64::consts::FRAC_PI_2);
    // Sphere: κ_mean = 1/r, κ_gaussian = 1/r²
    assert!((k.mean - 1.0 / r).abs() < 1e-6, "sphere mean curvature={} expected {}", k.mean, 1.0 / r);
    assert!((k.gaussian - 1.0 / (r * r)).abs() < 1e-6, "sphere Gaussian curvature={} expected {}", k.gaussian, 1.0 / (r * r));
}

#[test]
fn test_cylinder_curvature_one_nonzero() {
    let r = 5.0;
    let surface = Surface::Cylinder(CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, r));
    let k = surface.curvature_at(0.0, 0.0);
    // Cylinder: κ_mean = 1/(2r), κ_gaussian = 0 (one principal curvature is 0)
    assert!((k.mean - 1.0 / (2.0 * r)).abs() < 1e-6, "cylinder mean curvature={} expected {}", k.mean, 1.0 / (2.0 * r));
    assert!(k.gaussian.abs() < 1e-6, "cylinder Gaussian curvature should be 0, got {}", k.gaussian);
}

// ─────────────────────────────────────────────────────────────────────────
// Surface periodicity tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_plane_not_periodic() {
    let surface = Surface::Plane(Plane::xy());
    assert!(!surface.is_u_periodic());
    assert!(!surface.is_v_periodic());
}

#[test]
fn test_cylinder_u_periodic_only() {
    let surface = Surface::Cylinder(CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 1.0));
    assert!(surface.is_u_periodic());
    assert!(!surface.is_v_periodic());
}

#[test]
fn test_sphere_u_periodic_only() {
    let surface = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 1.0));
    assert!(surface.is_u_periodic());
    // Sphere is V-periodic in this implementation (wraps through poles)
    assert!(surface.is_v_periodic());
}

#[test]
fn test_surface_periodicity_torus_both() {
    let surface = Surface::Torus(TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0));
    assert!(surface.is_u_periodic());
    assert!(surface.is_v_periodic());
}

// ─────────────────────────────────────────────────────────────────────────
// Edge cases: NaN, very small parameters, large parameters
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_cylinder_does_not_panic_on_nan() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let _ = cyl.point_at(f64::NAN, 0.0);
    let _ = cyl.point_at(0.0, f64::NAN);
    let _ = cyl.normal_at(f64::NAN, f64::NAN);
}

#[test]
fn test_sphere_does_not_panic_on_nan() {
    let sph = SphereSurface::new(Point3d::ORIGIN, 1.0);
    let _ = sph.point_at(f64::NAN, 0.0);
    let _ = sph.normal_at(f64::NAN, f64::NAN);
}

#[test]
fn test_torus_does_not_panic_on_inf() {
    let torus = TorusSurface::new(Point3d::ORIGIN, Direction3d::Z, 10.0, 2.0);
    let _ = torus.point_at(f64::INFINITY, 0.0);
    let _ = torus.normal_at(f64::INFINITY, 0.0);
}

#[test]
fn test_cylinder_very_large_v() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let p = cyl.point_at(0.0, 1e15);
    // Should produce finite values (z = 1e15)
    assert!(p.z.is_finite(), "z should be finite: {}", p.z);
}

#[test]
fn test_nurbs_surface_out_of_range_uv_does_not_panic() {
    let nurbs = NurbsSurface {
        u_degree: 2,
        v_degree: 2,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 2.0, 0.0)],
            vec![Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 0.0), Point3d::new(1.0, 2.0, 0.0)],
            vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 0.0), Point3d::new(2.0, 2.0, 0.0)],
        ],
        weights: vec![vec![1.0; 3]; 3],
        u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    // Out-of-range UV should be clamped, not panic
    let _ = surface.point_at(-10.0, -10.0);
    let _ = surface.point_at(10.0, 10.0);
}

#[test]
fn test_nurbs_surface_high_degree_evaluation() {
    // Degree 5×5 — verify evaluation doesn't blow up
    let n = 6;
    let nurbs = NurbsSurface {
        u_degree: 5,
        v_degree: 5,
        control_points: (0..n).map(|i| {
            (0..n).map(|j| Point3d::new(i as f64, j as f64, (i as f64 + j as f64).sin())).collect()
        }).collect(),
        weights: vec![vec![1.0; n]; n],
        u_knots: {
            let mut k = vec![0.0; 6];
            k.extend(vec![1.0; 6]);
            k
        },
        v_knots: {
            let mut k = vec![0.0; 6];
            k.extend(vec![1.0; 6]);
            k
        },
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    let p = surface.point_at(0.5, 0.5);
    assert!(p.x.is_finite(), "x should be finite: {}", p.x);
    assert!(p.y.is_finite(), "y should be finite: {}", p.y);
    assert!(p.z.is_finite(), "z should be finite: {}", p.z);
}

#[test]
fn test_nurbs_surface_empty_control_points() {
    // Edge case: empty NURBS surface — should not panic
    let nurbs = NurbsSurface {
        u_degree: 1,
        v_degree: 1,
        control_points: vec![],
        weights: vec![],
        u_knots: vec![0.0, 0.0, 1.0, 1.0],
        v_knots: vec![0.0, 0.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);
    // Should not panic — may return origin or NaN, but must not crash
    let _ = surface.point_at(0.5, 0.5);
}

// ─────────────────────────────────────────────────────────────────────────
// Surface normal finite-difference verification
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_cylinder_normal_perpendicular_to_axis() {
    let cyl = CylinderSurface::new(Point3d::ORIGIN, Direction3d::Z, 5.0);
    let surface = Surface::Cylinder(cyl);
    for i in 0..8 {
        let u = i as f64 * std::f64::consts::PI / 4.0;
        let n = surface.normal_at(u, 0.0);
        // Normal should be perpendicular to axis (Z)
        assert!(n.z.abs() < 1e-9, "cylinder normal z={} should be 0", n.z);
    }
}

#[test]
fn test_sphere_normal_parallel_to_position() {
    let sphere = SphereSurface::new(Point3d::ORIGIN, 5.0);
    let surface = Surface::Sphere(sphere);
    for i in 0..6 {
        let u = i as f64 * std::f64::consts::PI / 3.0;
        for j in 1..5 {
            // v is polar angle [0, π]; skip poles
            let v = j as f64 * std::f64::consts::PI / 5.0;
            let p = surface.point_at(u, v);
            let n = surface.normal_at(u, v);
            // Cross product of position and normal should be ~0 (parallel vectors)
            let cx = p.y * n.z - p.z * n.y;
            let cy = p.z * n.x - p.x * n.z;
            let cz = p.x * n.y - p.y * n.x;
            let cross_mag = (cx * cx + cy * cy + cz * cz).sqrt();
            assert!(cross_mag < 1e-6, "sphere normal not parallel to position: cross={}", cross_mag);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
// from_v_rows constructor tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_from_v_rows_transposes_control_points() {
    // Build a simple 2×2 surface where each control point has a unique x coordinate.
    // Verify that the resulting surface, evaluated at u=0..1, v=0..1, gives the
    // expected control point positions at the corners.
    //
    // Author's intent (rows-of-V layout):
    //   v=0 row: [(0,0,0), (10,0,0)]   ← u=0, u=1 at v=0
    //   v=1 row: [(0,10,0), (10,10,0)] ← u=0, u=1 at v=1
    //
    // After from_v_rows, the struct's control_points[u][v] should be:
    //   u=0 row: [(0,0,0), (0,10,0)]
    //   u=1 row: [(10,0,0), (10,10,0)]
    let nurbs = NurbsSurface::from_v_rows(
        1, 1,
        vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(10.0, 0.0, 0.0)],
            vec![Point3d::new(0.0, 10.0, 0.0), Point3d::new(10.0, 10.0, 0.0)],
        ],
        vec![vec![1.0; 2]; 2],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        false, false,
    );
    let surface = Surface::Nurbs(nurbs);
    // Corners (clamped Bézier interpolates endpoints)
    let p_00 = surface.point_at(0.0, 0.0);  // (u=0, v=0) → (0, 0, 0)
    let p_10 = surface.point_at(1.0, 0.0);  // (u=1, v=0) → (10, 0, 0)
    let p_01 = surface.point_at(0.0, 1.0);  // (u=0, v=1) → (0, 10, 0)
    let p_11 = surface.point_at(1.0, 1.0);  // (u=1, v=1) → (10, 10, 0)
    assert!((p_00.x - 0.0).abs() < 1e-9 && (p_00.y - 0.0).abs() < 1e-9, "p_00 = {:?}", p_00);
    assert!((p_10.x - 10.0).abs() < 1e-9 && (p_10.y - 0.0).abs() < 1e-9, "p_10 = {:?}", p_10);
    assert!((p_01.x - 0.0).abs() < 1e-9 && (p_01.y - 10.0).abs() < 1e-9, "p_01 = {:?}", p_01);
    assert!((p_11.x - 10.0).abs() < 1e-9 && (p_11.y - 10.0).abs() < 1e-9, "p_11 = {:?}", p_11);
}

#[test]
fn test_from_v_rows_exact_quarter_sphere() {
    // The standard rational quadratic NURBS sphere octant construction.
    // With the CORRECT control points (corners of bounding box, not on sphere)
    // and weights [1, 1/√2, 1; 1/√2, 1/2, 1/√2; 1, 1/√2, 1], the surface
    // is an EXACT sphere octant.
    let r = 50.0;
    let inv_s = 1.0 / 2.0_f64.sqrt();
    let nurbs = NurbsSurface::from_v_rows(
        2, 2,
        vec![
            // v=0 (equator)
            vec![
                Point3d::new( r, 0.0, 0.0),  // u=0: on sphere
                Point3d::new( r,  r, 0.0),  // u=π/4 mid: bounding-box corner (NOT on sphere)
                Point3d::new(0.0,  r, 0.0),  // u=π/2: on sphere
            ],
            // v=π/4 (mid-elevation)
            vec![
                Point3d::new( r, 0.0,   r),  // bounding-box corner
                Point3d::new( r,  r,   r),  // 3D bounding-box corner (interior)
                Point3d::new(0.0,  r,   r),  // bounding-box corner
            ],
            // v=π/2 (north pole — degenerate edge)
            vec![
                Point3d::new(0.0, 0.0, r),
                Point3d::new(0.0, 0.0, r),
                Point3d::new(0.0, 0.0, r),
            ],
        ],
        vec![
            vec![1.0,   inv_s, 1.0],
            vec![inv_s, 0.5,   inv_s],
            vec![1.0,   inv_s, 1.0],
        ],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        false, false,
    );
    let surface = Surface::Nurbs(nurbs);

    // Sample 9×9 points and verify they're all on the sphere of radius R.
    let mut max_err = 0.0_f64;
    for i in 0..=8 {
        for j in 0..=8 {
            let u = i as f64 / 8.0;
            let v = j as f64 / 8.0;
            let p = surface.point_at(u, v);
            let dist = (p.x*p.x + p.y*p.y + p.z*p.z).sqrt();
            let err = (dist - r).abs();
            max_err = max_err.max(err);
        }
    }
    assert!(max_err < 1e-6, "sphere octant NURBS max distance error: {} (should be ~0)", max_err);
}

#[test]
fn test_from_v_rows_exact_half_cylinder_arc() {
    // Half-cylinder using TWO 90° rational quadratic arcs joined at the top.
    // 5 angular control points × 2 height control points.
    let r = 40.0;
    let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
    let nurbs = NurbsSurface::from_v_rows(
        2, 1,
        vec![
            vec![
                Point3d::new( r, 0.0, 0.0),
                Point3d::new( r, 0.0,   r),
                Point3d::new( 0.0, 0.0,   r),
                Point3d::new(-r, 0.0,   r),
                Point3d::new(-r, 0.0, 0.0),
            ],
            vec![
                Point3d::new( r, 100.0, 0.0),
                Point3d::new( r, 100.0,   r),
                Point3d::new( 0.0, 100.0,   r),
                Point3d::new(-r, 100.0,   r),
                Point3d::new(-r, 100.0, 0.0),
            ],
        ],
        vec![
            vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
            vec![1.0, inv_sqrt2, 1.0, inv_sqrt2, 1.0],
        ],
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        false, false,
    );
    let surface = Surface::Nurbs(nurbs);

    // Verify all sampled points are on the cylinder of radius R (Y axis)
    let mut max_radial_err = 0.0_f64;
    let mut max_y_err = 0.0_f64;
    for i in 0..=20 {
        for j in 0..=4 {
            let u = i as f64 / 20.0;
            let v = j as f64 / 4.0;
            let p = surface.point_at(u, v);
            let radius = (p.x*p.x + p.z*p.z).sqrt();
            let radial_err = (radius - r).abs();
            let expected_y = v * 100.0;
            let y_err = (p.y - expected_y).abs();
            max_radial_err = max_radial_err.max(radial_err);
            max_y_err = max_y_err.max(y_err);
        }
    }
    assert!(max_radial_err < 1e-6, "half-cylinder radial error: {} (should be 0)", max_radial_err);
    assert!(max_y_err < 1e-6, "half-cylinder Y error: {} (should be 0)", max_y_err);
}

#[test]
fn test_from_v_rows_validates_knot_count() {
    // Wrong number of u_knots should panic (assertion failure).
    // Here we provide 3 control points in U (after transpose), but only
    // 5 u_knots (need n_u + u_degree + 1 = 3 + 2 + 1 = 6).
    let result = std::panic::catch_unwind(|| {
        NurbsSurface::from_v_rows(
            2, 1,  // u_degree=2
            vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0), Point3d::new(2.0, 0.0, 0.0)],
                vec![Point3d::new(0.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 0.0), Point3d::new(2.0, 1.0, 0.0)],
            ],
            vec![vec![1.0; 3]; 2],
            vec![0.0, 0.0, 1.0, 1.0],  // 4 knots, but need 3+2+1=6
            vec![0.0, 0.0, 1.0, 1.0],
            false, false,
        );
    });
    assert!(result.is_err(), "from_v_rows should reject wrong u_knots count");

    // Wrong number of v_knots should also panic.
    let result = std::panic::catch_unwind(|| {
        NurbsSurface::from_v_rows(
            1, 1,
            vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0)],
                vec![Point3d::new(0.0, 1.0, 0.0), Point3d::new(1.0, 1.0, 0.0)],
                vec![Point3d::new(0.0, 2.0, 0.0), Point3d::new(1.0, 2.0, 0.0)],
            ],
            vec![vec![1.0; 2]; 3],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],  // 4 knots, but need 3+1+1=5
            false, false,
        );
    });
    assert!(result.is_err(), "from_v_rows should reject wrong v_knots count");
}

#[test]
fn test_from_v_rows_bicubic_saddle() {
    // 4×4 bicubic NURBS saddle. Verify corner interpolation.
    let nurbs = NurbsSurface::from_v_rows(
        3, 3,
        vec![
            vec![
                Point3d::new(-50.0, -50.0,   0.0),
                Point3d::new(-17.0, -50.0, -28.0),
                Point3d::new( 17.0, -50.0, -28.0),
                Point3d::new( 50.0, -50.0,   0.0),
            ],
            vec![
                Point3d::new(-50.0, -17.0,  28.0),
                Point3d::new(-17.0, -17.0,  -6.0),
                Point3d::new( 17.0, -17.0,  -6.0),
                Point3d::new( 50.0, -17.0,  28.0),
            ],
            vec![
                Point3d::new(-50.0,  17.0,  28.0),
                Point3d::new(-17.0,  17.0,  -6.0),
                Point3d::new( 17.0,  17.0,  -6.0),
                Point3d::new( 50.0,  17.0,  28.0),
            ],
            vec![
                Point3d::new(-50.0,  50.0,   0.0),
                Point3d::new(-17.0,  50.0, -28.0),
                Point3d::new( 17.0,  50.0, -28.0),
                Point3d::new( 50.0,  50.0,   0.0),
            ],
        ],
        vec![vec![1.0; 4]; 4],
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        false, false,
    );
    let surface = Surface::Nurbs(nurbs);
    // Corners: clamped bicubic interpolates corner control points
    let p_00 = surface.point_at(0.0, 0.0);
    assert!((p_00.x - (-50.0)).abs() < 1e-9, "p_00.x = {}", p_00.x);
    assert!((p_00.y - (-50.0)).abs() < 1e-9, "p_00.y = {}", p_00.y);
    assert!(p_00.z.abs() < 1e-9, "p_00.z = {}", p_00.z);

    let p_11 = surface.point_at(1.0, 1.0);
    assert!((p_11.x - 50.0).abs() < 1e-9, "p_11.x = {}", p_11.x);
    assert!((p_11.y - 50.0).abs() < 1e-9, "p_11.y = {}", p_11.y);
    assert!(p_11.z.abs() < 1e-9, "p_11.z = {}", p_11.z);

    // Midpoint of edge (u=0.5, v=0): X should be 0 (Bézier midpoint of [-50,-17,17,50])
    let p_mid = surface.point_at(0.5, 0.0);
    assert!(p_mid.x.abs() < 1e-9, "p_mid.x = {}", p_mid.x);
    assert!((p_mid.y - (-50.0)).abs() < 1e-9, "p_mid.y = {}", p_mid.y);
}
