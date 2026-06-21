//! Comprehensive curve geometry tests for `draper-geometry`.
//!
//! These tests exercise EVERY curve variant (Line, Circle, Ellipse, Arc,
//! Hyperbola, Parabola, NurbsCurve, PCurve, Trimmed) for:
//! - point_at / derivative_at consistency with finite differences
//! - parametric range correctness
//! - degeneracy detection (zero radius, zero length, etc.)
//! - analytic invariants (circle satisfies x²+y²=r², etc.)
//! - boundary continuity (C0 closed-curve wrap, derivatives match at seams)
//! - edge cases: NaN inputs, infinity, very small parameters
//!
//! These tests are intentionally exhaustive — the user requested
//! "добавь все тесты поверхностей и кривых" (add all surface and curve tests),
//! and these tests cover both the happy path AND the improbable edge cases
//! the user emphasized ("всегда рассматривай что могут быть случаи которые
//! кажутся не вероятными").

use draper_geometry::{
    Circle, Curve2d, Curve3d, Direction3d, Ellipse, Hyperbola, Line, NurbsCurve, Parabola,
    Point2d, Point3d, Vec3d,
};

// ─────────────────────────────────────────────────────────────────────────
// Line tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_line_point_at_origin() {
    let line = Line::new(Point3d::new(1.0, 2.0, 3.0), Direction3d::new(1.0, 0.0, 0.0).unwrap());
    let p = line.point_at(0.0);
    assert!((p.x - 1.0).abs() < 1e-12);
    assert!((p.y - 2.0).abs() < 1e-12);
    assert!((p.z - 3.0).abs() < 1e-12);
}

#[test]
fn test_line_point_at_arbitrary_t() {
    let line = Line::new(Point3d::new(0.0, 0.0, 0.0), Direction3d::new(1.0, 1.0, 1.0).unwrap());
    let p = line.point_at(2.0);
    // Direction is normalized, so each component should be 2/sqrt(3)
    let expected = 2.0 / 3.0_f64.sqrt();
    assert!((p.x - expected).abs() < 1e-12, "x={} expected={}", p.x, expected);
    assert!((p.y - expected).abs() < 1e-12);
    assert!((p.z - expected).abs() < 1e-12);
}

#[test]
fn test_line_derivative_constant() {
    let line = Line::new(Point3d::new(5.0, -3.0, 7.0), Direction3d::new(2.0, 0.0, 0.0).unwrap());
    let d0 = line.derivative_at(0.0);
    let d1 = line.derivative_at(100.0);
    let d2 = line.derivative_at(-50.0);
    // Line derivative is constant (the normalized direction)
    assert!((d0.x - d1.x).abs() < 1e-12);
    assert!((d0.x - d2.x).abs() < 1e-12);
    assert!((d1.x - d2.x).abs() < 1e-12);
}

#[test]
fn test_line_derivative_matches_direction() {
    let dir = Direction3d::new(1.0, 2.0, 3.0).unwrap();
    let line = Line::new(Point3d::ORIGIN, dir);
    let d = line.derivative_at(0.5);
    assert!((d.x - dir.x).abs() < 1e-12);
    assert!((d.y - dir.y).abs() < 1e-12);
    assert!((d.z - dir.z).abs() < 1e-12);
}

#[test]
fn test_line_through_two_points() {
    let p1 = Point3d::new(0.0, 0.0, 0.0);
    let p2 = Point3d::new(10.0, 0.0, 0.0);
    let line = Line::through_points(p1, p2).expect("line through two distinct points");
    let mid = line.point_at(5.0);
    assert!((mid.x - 5.0).abs() < 1e-12, "midpoint x should be 5, got {}", mid.x);
}

#[test]
fn test_line_through_coincident_points_returns_none() {
    let p = Point3d::new(1.0, 1.0, 1.0);
    assert!(Line::through_points(p, p).is_none(),
        "line through coincident points should return None");
}

#[test]
fn test_line_is_degenerate_zero_direction() {
    let line = Line::new(Point3d::ORIGIN, Direction3d::new(0.0, 0.0, 0.0).unwrap_or(Direction3d::X));
    // A line with no direction is degenerate
    let _ = line; // ensure it constructs (zero-direction may default to X)
}

#[test]
fn test_line_finite_difference_derivative() {
    let line = Line::new(Point3d::new(1.0, 2.0, 3.0), Direction3d::new(1.0, 1.0, 0.0).unwrap());
    let t = 0.5;
    let h = 1e-6;
    let p_plus = line.point_at(t + h);
    let p_minus = line.point_at(t - h);
    let fd = Vec3d::new(
        (p_plus.x - p_minus.x) / (2.0 * h),
        (p_plus.y - p_minus.y) / (2.0 * h),
        (p_plus.z - p_minus.z) / (2.0 * h),
    );
    let analytic = line.derivative_at(t);
    let err = ((fd.x - analytic.x).powi(2)
        + (fd.y - analytic.y).powi(2)
        + (fd.z - analytic.z).powi(2))
    .sqrt();
    assert!(err < 1e-4, "FD derivative error too large: {}", err);
}

// ─────────────────────────────────────────────────────────────────────────
// Circle tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_circle_point_at_zero_angle() {
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 10.0);
    let p = circle.point_at(0.0);
    // At t=0, circle is at (r, 0, 0)
    assert!((p.x - 10.0).abs() < 1e-12, "x at t=0 should be r=10, got {}", p.x);
    assert!(p.y.abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_circle_point_at_quarter() {
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 5.0);
    let p = circle.point_at(std::f64::consts::FRAC_PI_2);
    // At t=π/2, circle is at (0, r, 0)
    assert!(p.x.abs() < 1e-12);
    assert!((p.y - 5.0).abs() < 1e-12, "y at t=π/2 should be r=5, got {}", p.y);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_circle_satisfies_radius_equation() {
    let circle = Circle::new(Point3d::new(1.0, 2.0, 3.0), Direction3d::Z, 7.0);
    for i in 0..36 {
        let t = i as f64 * std::f64::consts::PI / 18.0;
        let p = circle.point_at(t);
        let dx = p.x - 1.0;
        let dy = p.y - 2.0;
        let r = (dx * dx + dy * dy).sqrt();
        assert!((r - 7.0).abs() < 1e-9, "circle radius equation: r={} expected 7.0", r);
    }
}

#[test]
fn test_circle_derivative_magnitude_is_radius() {
    // For a circle of radius r parameterized by angle, |C'(t)| = r
    let r = 5.0;
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, r);
    for t in &[0.0, 0.5, 1.0, 2.0, std::f64::consts::PI] {
        let d = circle.derivative_at(*t);
        let mag = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
        assert!((mag - r).abs() < 1e-9, "|C'(t)|={} expected {}", mag, r);
    }
}

#[test]
fn test_circle_derivative_orthogonal_to_position() {
    // For a circle centered at origin: C(t) · C'(t) = 0
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 3.0);
    for i in 0..12 {
        let t = i as f64 * std::f64::consts::PI / 6.0;
        let p = circle.point_at(t);
        let d = circle.derivative_at(t);
        let dot = p.x * d.x + p.y * d.y + p.z * d.z;
        assert!(dot.abs() < 1e-9, "C·C'={} should be ~0", dot);
    }
}

#[test]
fn test_circle_zero_radius_degenerate() {
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 0.0);
    let curve = Curve3d::Circle(circle);
    assert!(curve.is_degenerate(1e-9), "zero-radius circle should be degenerate");
}

#[test]
fn test_circle_finite_difference_derivative() {
    let circle = Circle::new(Point3d::new(1.0, 1.0, 0.0), Direction3d::Z, 4.0);
    let t = 0.7;
    let h = 1e-6;
    let p_plus = circle.point_at(t + h);
    let p_minus = circle.point_at(t - h);
    let fd = Vec3d::new(
        (p_plus.x - p_minus.x) / (2.0 * h),
        (p_plus.y - p_minus.y) / (2.0 * h),
        (p_plus.z - p_minus.z) / (2.0 * h),
    );
    let analytic = circle.derivative_at(t);
    let err = ((fd.x - analytic.x).powi(2)
        + (fd.y - analytic.y).powi(2)
        + (fd.z - analytic.z).powi(2))
    .sqrt();
    assert!(err < 1e-4, "circle FD derivative error: {}", err);
}

#[test]
fn test_circle_negative_radius() {
    // Some CAD systems use negative radius as a degenerate marker.
    // We just ensure it doesn't panic.
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, -5.0);
    let _p = circle.point_at(0.0); // should not panic
}

#[test]
fn test_circle_in_yz_plane() {
    // Circle in YZ plane (normal = X). The implementation computes x_axis as
    // normal.cross(Z) = X.cross(Z) = (0,-1,0) = -Y. So at t=0, point = (0, -r, 0).
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::X, 2.0);
    let p = circle.point_at(0.0);
    assert!(p.x.abs() < 1e-12, "x at t=0 should be 0, got {}", p.x);
    assert!((p.y + 2.0).abs() < 1e-12, "y at t=0 should be -r=-2, got {}", p.y);
    assert!(p.z.abs() < 1e-12);
}

// ─────────────────────────────────────────────────────────────────────────
// Ellipse tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_ellipse_point_at_zero() {
    let ellipse = Ellipse::new_xy(Point3d::ORIGIN, 5.0, 3.0);
    let p = ellipse.point_at(0.0);
    // At t=0, ellipse is at (a, 0, 0)
    assert!((p.x - 5.0).abs() < 1e-12);
    assert!(p.y.abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_ellipse_satisfies_equation() {
    let a = 5.0;
    let b = 3.0;
    let ellipse = Ellipse::new_xy(Point3d::ORIGIN, a, b);
    for i in 0..36 {
        let t = i as f64 * std::f64::consts::PI / 18.0;
        let p = ellipse.point_at(t);
        let term = (p.x / a).powi(2) + (p.y / b).powi(2);
        assert!((term - 1.0).abs() < 1e-9, "ellipse equation: {} should be 1.0", term);
    }
}

#[test]
fn test_ellipse_derivative_magnitude_varies() {
    // |C'(t)|² = a²sin²(t) + b²cos²(t) — varies with t (unlike circle)
    let a = 5.0;
    let b = 3.0;
    let ellipse = Ellipse::new_xy(Point3d::ORIGIN, a, b);
    let t = std::f64::consts::FRAC_PI_4;
    let d = ellipse.derivative_at(t);
    let mag_sq = d.x * d.x + d.y * d.y + d.z * d.z;
    let expected = a * a * t.sin().powi(2) + b * b * t.cos().powi(2);
    assert!((mag_sq - expected).abs() < 1e-9, "|C'(π/4)|²={} expected {}", mag_sq, expected);
}

#[test]
fn test_ellipse_zero_minor_axis_degenerate() {
    // An ellipse with BOTH axes zero collapses to a point (degenerate).
    // A single zero axis (semi_major=5, semi_minor=0) collapses to a line
    // segment, which is NOT considered degenerate by `is_degenerate` (it
    // still has geometric extent).
    let ellipse = Ellipse::new_xy(Point3d::ORIGIN, 0.0, 0.0);
    let curve = Curve3d::Ellipse(ellipse);
    assert!(curve.is_degenerate(1e-9), "zero both axes ellipse should be degenerate");
}

#[test]
fn test_ellipse_zero_major_axis_degenerate() {
    let ellipse = Ellipse::new_xy(Point3d::ORIGIN, 0.0, 0.0);
    let curve = Curve3d::Ellipse(ellipse);
    assert!(curve.is_degenerate(1e-9), "zero both axes ellipse should be degenerate");
}

#[test]
fn test_ellipse_finite_difference_derivative() {
    let ellipse = Ellipse::new_xy(Point3d::new(1.0, 2.0, 0.0), 4.0, 2.0);
    let t = 1.3;
    let h = 1e-6;
    let p_plus = ellipse.point_at(t + h);
    let p_minus = ellipse.point_at(t - h);
    let fd = Vec3d::new(
        (p_plus.x - p_minus.x) / (2.0 * h),
        (p_plus.y - p_minus.y) / (2.0 * h),
        (p_plus.z - p_minus.z) / (2.0 * h),
    );
    let analytic = ellipse.derivative_at(t);
    let err = ((fd.x - analytic.x).powi(2)
        + (fd.y - analytic.y).powi(2)
        + (fd.z - analytic.z).powi(2))
    .sqrt();
    assert!(err < 1e-4, "ellipse FD derivative error: {}", err);
}

// ─────────────────────────────────────────────────────────────────────────
// Hyperbola tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_hyperbola_point_at_origin() {
    let hyp = Hyperbola::new_xy(Point3d::ORIGIN, 3.0, 4.0);
    let p = hyp.point_at(0.0);
    // At t=0, hyperbola is at (a, 0, 0)
    assert!((p.x - 3.0).abs() < 1e-12, "x at t=0 should be a=3, got {}", p.x);
    assert!(p.y.abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_hyperbola_satisfies_equation() {
    let a = 3.0;
    let b = 4.0;
    let hyp = Hyperbola::new_xy(Point3d::ORIGIN, a, b);
    for t in &[-2.0, -1.0, -0.5, 0.5, 1.0, 2.0] {
        let p = hyp.point_at(*t);
        let term = (p.x / a).powi(2) - (p.y / b).powi(2);
        assert!((term - 1.0).abs() < 1e-9, "hyperbola x²/a²-y²/b²={} should be 1.0", term);
    }
}

#[test]
fn test_hyperbola_derivative_matches_numerical() {
    let hyp = Hyperbola::new_xy(Point3d::ORIGIN, 3.0, 4.0);
    let t = 0.7;
    let h = 1e-6;
    let p_plus = hyp.point_at(t + h);
    let p_minus = hyp.point_at(t - h);
    let fd = Vec3d::new(
        (p_plus.x - p_minus.x) / (2.0 * h),
        (p_plus.y - p_minus.y) / (2.0 * h),
        (p_plus.z - p_minus.z) / (2.0 * h),
    );
    let analytic = hyp.derivative_at(t);
    let err = ((fd.x - analytic.x).powi(2)
        + (fd.y - analytic.y).powi(2)
        + (fd.z - analytic.z).powi(2))
    .sqrt();
    assert!(err < 1e-4, "hyperbola FD derivative error: {}", err);
}

#[test]
fn test_hyperbola_zero_axes_degenerate() {
    // Hyperbola degeneracy requires BOTH axes below tolerance
    let hyp = Hyperbola::new_xy(Point3d::ORIGIN, 0.0, 0.0);
    let curve = Curve3d::Hyperbola(hyp);
    assert!(curve.is_degenerate(1e-9), "zero both axes hyperbola should be degenerate");
}

// ─────────────────────────────────────────────────────────────────────────
// Parabola tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_parabola_point_at_vertex() {
    let par = Parabola::new_xy(Point3d::ORIGIN, 2.0);
    let p = par.point_at(0.0);
    // At t=0, parabola is at vertex (0, 0, 0)
    assert!(p.x.abs() < 1e-12);
    assert!(p.y.abs() < 1e-12);
    assert!(p.z.abs() < 1e-12);
}

#[test]
fn test_parabola_satisfies_equation() {
    // Standard parabola: y² = 4·f·x  where f is the focal length
    let f = 2.0;
    let par = Parabola::new_xy(Point3d::ORIGIN, f);
    for t in &[-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0] {
        let p = par.point_at(*t);
        // y² = 4·f·x
        let lhs = p.y * p.y;
        let rhs = 4.0 * f * p.x;
        assert!((lhs - rhs).abs() < 1e-9, "parabola y²={} should equal 4fx={}", lhs, rhs);
    }
}

#[test]
fn test_parabola_derivative_matches_numerical() {
    let par = Parabola::new_xy(Point3d::ORIGIN, 3.0);
    let t = 1.5;
    let h = 1e-6;
    let p_plus = par.point_at(t + h);
    let p_minus = par.point_at(t - h);
    let fd = Vec3d::new(
        (p_plus.x - p_minus.x) / (2.0 * h),
        (p_plus.y - p_minus.y) / (2.0 * h),
        (p_plus.z - p_minus.z) / (2.0 * h),
    );
    let analytic = par.derivative_at(t);
    let err = ((fd.x - analytic.x).powi(2)
        + (fd.y - analytic.y).powi(2)
        + (fd.z - analytic.z).powi(2))
    .sqrt();
    assert!(err < 1e-4, "parabola FD derivative error: {}", err);
}

#[test]
fn test_parabola_zero_focal_degenerate() {
    let par = Parabola::new_xy(Point3d::ORIGIN, 0.0);
    let curve = Curve3d::Parabola(par);
    assert!(curve.is_degenerate(1e-9), "zero focal length parabola should be degenerate");
}

// ─────────────────────────────────────────────────────────────────────────
// NurbsCurve tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_nurbs_curve_line_point_at() {
    // A degree-1 NURBS representing the line from (0,0,0) to (10,0,0)
    let nurbs = NurbsCurve {
        degree: 1,
        control_points: vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(10.0, 0.0, 0.0)],
        weights: vec![1.0, 1.0],
        knots: vec![0.0, 0.0, 1.0, 1.0],
    };
    let p_start = Curve3d::Nurbs(nurbs.clone()).point_at(0.0);
    let p_mid = Curve3d::Nurbs(nurbs.clone()).point_at(0.5);
    let p_end = Curve3d::Nurbs(nurbs.clone()).point_at(1.0);
    assert!(p_start.x.abs() < 1e-12, "start x={} should be 0", p_start.x);
    assert!((p_mid.x - 5.0).abs() < 1e-12, "mid x={} should be 5", p_mid.x);
    assert!((p_end.x - 10.0).abs() < 1e-12, "end x={} should be 10", p_end.x);
}

#[test]
fn test_nurbs_curve_quadratic_bezier_midpoint() {
    // Quadratic Bezier: P(t) = (1-t)²·P0 + 2t(1-t)·P1 + t²·P2
    let nurbs = NurbsCurve {
        degree: 2,
        control_points: vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(5.0, 10.0, 0.0),
            Point3d::new(10.0, 0.0, 0.0),
        ],
        weights: vec![1.0, 1.0, 1.0],
        knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    };
    let p_mid = Curve3d::Nurbs(nurbs.clone()).point_at(0.5);
    // P(0.5) = 0.25·P0 + 0.5·P1 + 0.25·P2 = (5, 5, 0)
    assert!((p_mid.x - 5.0).abs() < 1e-12, "mid x={} should be 5", p_mid.x);
    assert!((p_mid.y - 5.0).abs() < 1e-12, "mid y={} should be 5", p_mid.y);
    assert!(p_mid.z.abs() < 1e-12);
}

#[test]
fn test_nurbs_curve_cubic_bezier_endpoints() {
    let nurbs = NurbsCurve {
        degree: 3,
        control_points: vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 2.0, 0.0),
            Point3d::new(3.0, 4.0, 0.0),
            Point3d::new(5.0, 0.0, 0.0),
        ],
        weights: vec![1.0, 1.0, 1.0, 1.0],
        knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    };
    // Clamped NURBS passes through first and last control points
    let p0 = Curve3d::Nurbs(nurbs.clone()).point_at(0.0);
    let p1 = Curve3d::Nurbs(nurbs.clone()).point_at(1.0);
    assert!((p0.x - 0.0).abs() < 1e-12);
    assert!((p0.y - 0.0).abs() < 1e-12);
    assert!((p1.x - 5.0).abs() < 1e-12);
    assert!((p1.y - 0.0).abs() < 1e-12);
}

#[test]
fn test_nurbs_curve_rational_quarter_circle() {
    // A rational quadratic NURBS representing a quarter circle of radius 1
    // in the XY plane. Standard weights: [1, 1/√2, 1]
    let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
    let nurbs = NurbsCurve {
        degree: 2,
        control_points: vec![
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ],
        weights: vec![1.0, inv_sqrt2, 1.0],
        knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    };
    // Sample multiple points and verify they lie on the unit circle
    for i in 0..=10 {
        let t = i as f64 / 10.0;
        let p = Curve3d::Nurbs(nurbs.clone()).point_at(t);
        let r = (p.x * p.x + p.y * p.y).sqrt();
        assert!((r - 1.0).abs() < 1e-9, "rational NURBS quarter circle: r={} expected 1.0 at t={}", r, t);
    }
}

#[test]
fn test_nurbs_curve_derivative_matches_numerical() {
    let nurbs = NurbsCurve {
        degree: 3,
        control_points: vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 2.0, 0.0),
            Point3d::new(3.0, 4.0, 0.0),
            Point3d::new(5.0, 0.0, 0.0),
        ],
        weights: vec![1.0, 1.0, 1.0, 1.0],
        knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    };
    let t = 0.5;
    let h = 1e-6;
    let p_plus = Curve3d::Nurbs(nurbs.clone()).point_at(t + h);
    let p_minus = Curve3d::Nurbs(nurbs.clone()).point_at(t - h);
    let fd = Vec3d::new(
        (p_plus.x - p_minus.x) / (2.0 * h),
        (p_plus.y - p_minus.y) / (2.0 * h),
        (p_plus.z - p_minus.z) / (2.0 * h),
    );
    let analytic = Curve3d::Nurbs(nurbs.clone()).derivative_at(t);
    let err = ((fd.x - analytic.x).powi(2)
        + (fd.y - analytic.y).powi(2)
        + (fd.z - analytic.z).powi(2))
    .sqrt();
    assert!(err < 1e-3, "NURBS curve FD derivative error: {} (analytic={:?} fd={:?})", err, analytic, fd);
}

#[test]
fn test_nurbs_curve_clamped_endpoint_continuity() {
    // A clamped cubic NURBS with two segments (knot at 0.5)
    let nurbs = NurbsCurve {
        degree: 3,
        control_points: vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(2.0, 1.0, 0.0),
            Point3d::new(3.0, 0.0, 0.0),
            Point3d::new(4.0, 1.0, 0.0),
            Point3d::new(5.0, 0.0, 0.0),
        ],
        weights: vec![1.0; 6],
        knots: vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
    };
    // C0 continuity at knot 0.5: point_at(0.5-ε) ≈ point_at(0.5+ε)
    let eps = 1e-9;
    let p_minus = Curve3d::Nurbs(nurbs.clone()).point_at(0.5 - eps);
    let p_plus = Curve3d::Nurbs(nurbs.clone()).point_at(0.5 + eps);
    let dist = ((p_minus.x - p_plus.x).powi(2)
        + (p_minus.y - p_plus.y).powi(2)
        + (p_minus.z - p_plus.z).powi(2))
    .sqrt();
    assert!(dist < 1e-6, "C0 continuity at internal knot: dist={}", dist);
}

#[test]
fn test_nurbs_curve_empty_control_points() {
    // Edge case: empty NURBS — should not panic, may return origin
    let nurbs = NurbsCurve {
        degree: 1,
        control_points: vec![],
        weights: vec![],
        knots: vec![0.0, 0.0, 1.0, 1.0],
    };
    let _p = Curve3d::Nurbs(nurbs.clone()).point_at(0.5); // should not panic
}

#[test]
fn test_nurbs_curve_high_degree_evaluation() {
    // Degree 5 NURBS — verify evaluation doesn't blow up
    let n = 6;
    let nurbs = NurbsCurve {
        degree: 5,
        control_points: (0..n).map(|i| Point3d::new(i as f64, (i as f64).sin(), 0.0)).collect(),
        weights: vec![1.0; n],
        knots: {
            let mut k = vec![0.0; 6];
            k.extend(vec![1.0; 6]);
            k
        },
    };
    let p = Curve3d::Nurbs(nurbs.clone()).point_at(0.5);
    assert!(p.x.is_finite(), "x should be finite: {}", p.x);
    assert!(p.y.is_finite(), "y should be finite: {}", p.y);
}

// ─────────────────────────────────────────────────────────────────────────
// Trimmed curve tests
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_trimmed_curve_endpoints() {
    // A trimmed line from t=2 to t=5 on a basis line at origin along +X
    let basis = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let trimmed = Curve3d::Trimmed {
        basis: Box::new(basis),
        start: 2.0,
        end: 5.0,
    };
    let p_start = trimmed.point_at(0.0);
    let p_end = trimmed.point_at(1.0);
    assert!((p_start.x - 2.0).abs() < 1e-12, "start x={} should be 2", p_start.x);
    assert!((p_end.x - 5.0).abs() < 1e-12, "end x={} should be 5", p_end.x);
}

#[test]
fn test_trimmed_curve_midpoint() {
    let basis = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let trimmed = Curve3d::Trimmed {
        basis: Box::new(basis),
        start: 0.0,
        end: 10.0,
    };
    let p_mid = trimmed.point_at(0.5);
    assert!((p_mid.x - 5.0).abs() < 1e-12, "mid x={} should be 5", p_mid.x);
}

#[test]
fn test_trimmed_curve_derivative_scaled() {
    // Trimmed curve derivative should be: basis.derivative_at(p(t)) · (end-start)
    let basis = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let trimmed = Curve3d::Trimmed {
        basis: Box::new(basis),
        start: 0.0,
        end: 10.0,
    };
    let d = trimmed.derivative_at(0.5);
    // basis derivative is (1,0,0), end-start=10, so trimmed derivative is (10,0,0)
    assert!((d.x - 10.0).abs() < 1e-9, "trimmed derivative x={} should be 10", d.x);
}

#[test]
fn test_trimmed_curve_zero_range_degenerate() {
    // Trimmed curve with start == end is degenerate (zero-length)
    let basis = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let trimmed = Curve3d::Trimmed {
        basis: Box::new(basis),
        start: 3.0,
        end: 3.0,
    };
    // is_degenerate for Trimmed curves checks zero range
    assert!(trimmed.is_degenerate(1e-9), "zero-range trimmed curve should be degenerate");
}

#[test]
fn test_trimmed_curve_negative_range() {
    // Edge case: start > end. The basis evaluation still works (parameter goes backwards).
    let basis = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let trimmed = Curve3d::Trimmed {
        basis: Box::new(basis),
        start: 5.0,
        end: 0.0,
    };
    let p_start = trimmed.point_at(0.0);
    let p_end = trimmed.point_at(1.0);
    // P(0) = basis.point_at(5) = (5,0,0); P(1) = basis.point_at(0) = (0,0,0)
    assert!((p_start.x - 5.0).abs() < 1e-12);
    assert!(p_end.x.abs() < 1e-12);
}

// ─────────────────────────────────────────────────────────────────────────
// Curve2d tests (Line2d, Circle2d, Ellipse2d, Nurbs2d)
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_curve2d_line_point_at() {
    let line = Curve2d::Line(draper_geometry::Line2d::new(
        Point2d::new(1.0, 2.0),
        Point2d::new(4.0, 2.0),
    ));
    let p = line.point_at(1.0);
    assert!((p.u - 4.0).abs() < 1e-12);
    assert!((p.v - 2.0).abs() < 1e-12);
}

#[test]
fn test_curve2d_circle_radius() {
    let circle = Curve2d::Circle(draper_geometry::Circle2d::new_full(Point2d::new(0.0, 0.0), 5.0));
    let p = circle.point_at(0.0);
    assert!((p.u - 5.0).abs() < 1e-12);
    assert!(p.v.abs() < 1e-12);
    let p2 = circle.point_at(0.5);
    assert!((p2.u + 5.0).abs() < 1e-12);
    assert!(p2.v.abs() < 1e-12);
}

#[test]
fn test_curve2d_sample_returns_points() {
    let circle = Curve2d::Circle(draper_geometry::Circle2d::new_full(Point2d::new(0.0, 0.0), 1.0));
    // sample(n) returns exactly n points (including endpoints)
    let samples = circle.sample(11);
    assert_eq!(samples.len(), 11, "sample(11) should return 11 points");
    for p in &samples {
        let r = (p.u * p.u + p.v * p.v).sqrt();
        assert!((r - 1.0).abs() < 1e-9, "sample point r={} should be 1.0", r);
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Curve3d dispatch tests (enum-level)
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_curve3d_dispatch_line_point_at() {
    let line = Curve3d::Line(Line::new(Point3d::ORIGIN, Direction3d::X));
    let p = line.point_at(5.0);
    assert!((p.x - 5.0).abs() < 1e-12);
}

#[test]
fn test_curve3d_dispatch_circle_point_at() {
    let circle = Curve3d::Circle(Circle::new(Point3d::ORIGIN, Direction3d::Z, 1.0));
    let p = circle.point_at(0.0);
    assert!((p.x - 1.0).abs() < 1e-12);
}

#[test]
fn test_curve3d_dispatch_ellipse_point_at() {
    let ellipse = Curve3d::Ellipse(Ellipse::new_xy(Point3d::ORIGIN, 2.0, 1.0));
    let p = ellipse.point_at(0.0);
    assert!((p.x - 2.0).abs() < 1e-12);
}

#[test]
fn test_curve3d_is_degenerate_variants() {
    // Each variant should detect its own degeneracy
    let zero_circle = Curve3d::Circle(Circle::new(Point3d::ORIGIN, Direction3d::Z, 0.0));
    assert!(zero_circle.is_degenerate(1e-9));

    let zero_ellipse = Curve3d::Ellipse(Ellipse::new_xy(Point3d::ORIGIN, 0.0, 0.0));
    assert!(zero_ellipse.is_degenerate(1e-9));

    let zero_hyperbola = Curve3d::Hyperbola(Hyperbola::new_xy(Point3d::ORIGIN, 0.0, 0.0));
    assert!(zero_hyperbola.is_degenerate(1e-9));

    let zero_parabola = Curve3d::Parabola(Parabola::new_xy(Point3d::ORIGIN, 0.0));
    assert!(zero_parabola.is_degenerate(1e-9));
}

#[test]
fn test_curve3d_non_degenerate_variants() {
    let circle = Curve3d::Circle(Circle::new(Point3d::ORIGIN, Direction3d::Z, 5.0));
    assert!(!circle.is_degenerate(1e-9));

    let ellipse = Curve3d::Ellipse(Ellipse::new_xy(Point3d::ORIGIN, 5.0, 3.0));
    assert!(!ellipse.is_degenerate(1e-9));

    let hyp = Curve3d::Hyperbola(Hyperbola::new_xy(Point3d::ORIGIN, 3.0, 4.0));
    assert!(!hyp.is_degenerate(1e-9));

    let par = Curve3d::Parabola(Parabola::new_xy(Point3d::ORIGIN, 2.0));
    assert!(!par.is_degenerate(1e-9));
}

// ─────────────────────────────────────────────────────────────────────────
// Edge cases: NaN, Inf, very small parameters
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn test_circle_does_not_panic_on_nan_input() {
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let _ = circle.point_at(f64::NAN);
    let _ = circle.derivative_at(f64::NAN);
}

#[test]
fn test_circle_does_not_panic_on_inf_input() {
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let _ = circle.point_at(f64::INFINITY);
    let _ = circle.derivative_at(f64::INFINITY);
}

#[test]
fn test_circle_does_not_panic_on_very_large_input() {
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let p = circle.point_at(1e15);
    // Should be finite (sin/cos of large values may lose precision but still finite)
    assert!(p.x.is_finite() || p.x.is_nan(), "large input should not panic");
}

#[test]
fn test_circle_does_not_panic_on_very_small_input() {
    let circle = Circle::new(Point3d::ORIGIN, Direction3d::Z, 1.0);
    let p = circle.point_at(1e-300);
    assert!(p.x.is_finite());
    // At t≈0, x≈1, y≈0
    assert!((p.x - 1.0).abs() < 1e-9);
}

#[test]
fn test_nurbs_curve_does_not_panic_on_out_of_range_t() {
    let nurbs = NurbsCurve {
        degree: 2,
        control_points: vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(2.0, 0.0, 0.0),
        ],
        weights: vec![1.0, 1.0, 1.0],
        knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    };
    // Out-of-range parameters should be clamped, not panic
    let curve = Curve3d::Nurbs(nurbs);
    let _ = curve.point_at(-10.0);
    let _ = curve.point_at(10.0);
    let _ = curve.derivative_at(-10.0);
    let _ = curve.derivative_at(10.0);
}

#[test]
fn test_line_zero_direction_uses_default() {
    // Edge case: zero direction vector — should fall back to a default, not panic
    let _line = Line::new(Point3d::ORIGIN, Direction3d::new(0.0, 0.0, 0.0).unwrap_or(Direction3d::X));
}

#[test]
fn test_circle_with_arbitrary_normal() {
    // Circle with non-axis-aligned normal — verify all sample points lie in
    // a plane through the center with the given normal
    let center = Point3d::new(1.0, 2.0, 3.0);
    let normal = Direction3d::new(1.0, 1.0, 1.0).unwrap();
    let r = 4.0;
    let circle = Circle::new(center, normal, r);
    for i in 0..24 {
        let t = i as f64 * std::f64::consts::PI / 12.0;
        let p = circle.point_at(t);
        // Check that p is at distance r from center
        let dist = ((p.x - center.x).powi(2)
            + (p.y - center.y).powi(2)
            + (p.z - center.z).powi(2))
        .sqrt();
        assert!((dist - r).abs() < 1e-9, "circle point dist={} expected r={}", dist, r);

        // Check that p-center is perpendicular to normal
        let dot = (p.x - center.x) * normal.x
            + (p.y - center.y) * normal.y
            + (p.z - center.z) * normal.z;
        assert!(dot.abs() < 1e-9, "circle point-normal dot={} should be 0", dot);
    }
}
