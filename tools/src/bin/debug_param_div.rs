//! Debug: trace parameter_division_2d output for a NURBS surface.

use draper_geometry::{NurbsSurface, Surface};
use std::f64::consts::PI;

fn main() {
    // Construct a sample NURBS surface that mimics the nist_complex_surface case:
    // 3x3 control points, deg 2/2, UV range [0,1]x[0,1], near-flat.
    //
    // We can't easily extract the exact NURBS from the STEP file here, but
    // we can demonstrate the algorithm on a known case.
    let _ = test_near_flat_nurbs();
    let _ = test_cylinder();
    let _ = test_sphere();
}

fn test_near_flat_nurbs() {
    // 3x3 control points, deg 2/2. Center point raised 2 units (like
    // nist_complex_surface.stp #45: control points have z=1 everywhere
    // except center which is z=3).
    let control_points = vec![
        vec![
            draper_geometry::Point3d::new(0.0, 0.0, 1.0),
            draper_geometry::Point3d::new(5.0, 0.0, 1.0),
            draper_geometry::Point3d::new(10.0, 0.0, 1.0),
        ],
        vec![
            draper_geometry::Point3d::new(0.0, 5.0, 1.0),
            draper_geometry::Point3d::new(5.0, 5.0, 3.0),
            draper_geometry::Point3d::new(10.0, 5.0, 1.0),
        ],
        vec![
            draper_geometry::Point3d::new(0.0, 10.0, 1.0),
            draper_geometry::Point3d::new(5.0, 10.0, 1.0),
            draper_geometry::Point3d::new(10.0, 10.0, 1.0),
        ],
    ];
    let weights = vec![vec![1.0; 3]; 3];
    let u_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let v_knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let nurbs = NurbsSurface {
        u_degree: 2,
        v_degree: 2,
        control_points,
        weights,
        u_knots,
        v_knots,
        u_closed: false,
        v_closed: false,
    };
    let surface = Surface::Nurbs(nurbs);

    println!("\n=== Saddle NURBS 3x3 deg 2/2 (matches nist_complex_surface) ===");
    for tol in &[0.0001, 0.001, 0.01, 0.05, 0.1, 0.5, 1.0] {
        let (us, vs) = draper_mesh::parametric_division_2d::parameter_division_2d(
            &surface, (0.0, 1.0), (0.0, 1.0), *tol, 64,
        );
        let interior = draper_mesh::parametric_division_2d::interior_steiner_points(
            &us, &vs, (0.0, 1.0), (0.0, 1.0), 1e-9,
        );
        println!("  tol={:8.4}: u_knots={} v_knots={} interior_pts={}",
                 tol, us.len(), vs.len(), interior.len());
    }
}

fn test_cylinder() {
    use draper_geometry::CylinderSurface;
    let cyl = Surface::Cylinder(CylinderSurface::new_z(1.0));
    println!("\n=== Cylinder r=1, full 2π sweep ===");
    for tol in &[0.0001, 0.001, 0.01, 0.1] {
        let (us, vs) = draper_mesh::parametric_division_2d::parameter_division_2d(
            &cyl, (0.0, 2.0 * PI), (0.0, 1.0), *tol, 64,
        );
        let interior = draper_mesh::parametric_division_2d::interior_steiner_points(
            &us, &vs, (0.0, 2.0 * PI), (0.0, 1.0), 1e-9,
        );
        println!("  tol={:8.4}: u_knots={} v_knots={} interior_pts={}",
                 tol, us.len(), vs.len(), interior.len());
    }
}

fn test_sphere() {
    use draper_geometry::{SphereSurface, Point3d};
    let sph = Surface::Sphere(SphereSurface::new(Point3d::ORIGIN, 1.0));
    println!("\n=== Sphere r=1 ===");
    for tol in &[0.0001, 0.001, 0.01, 0.1] {
        let (us, vs) = draper_mesh::parametric_division_2d::parameter_division_2d(
            &sph, (0.0, 2.0 * PI), (0.0, PI), *tol, 64,
        );
        let interior = draper_mesh::parametric_division_2d::interior_steiner_points(
            &us, &vs, (0.0, 2.0 * PI), (0.0, PI), 1e-9,
        );
        println!("  tol={:8.4}: u_knots={} v_knots={} interior_pts={}",
                 tol, us.len(), vs.len(), interior.len());
    }
}
