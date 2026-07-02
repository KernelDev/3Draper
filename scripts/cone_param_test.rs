// Quick test to verify cone parameterization
// Expected STEP behavior: r = radius + v * tan(half_angle)
// Current code: r = radius - v * tan(half_angle) for standard (non-expanding) cones

fn main() {
    // Cone from 3.05.078.stp Step#78:
    // origin = (2.43, 0, 0), axis = (-1, 0, 0), x_dir = (0, 1, 0)
    // radius = 32.79, half_angle = π/4

    let pi = std::f64::consts::PI;
    let half_angle = pi / 4.0; // 0.7854
    let radius = 32.79;
    let tan_ha = half_angle.tan(); // = 1.0

    // STEP parameterization: C(u,v) = origin + v*axis + (radius + v*tan(ha)) * (cos(u)*x_dir + sin(u)*y_dir)
    // At u=0:
    // C(0, v) = (2.43 - v, radius + v, 0)

    // v=0: (2.43, 32.79, 0) — base circle
    // v=2.43: (0, 35.22, 0) — outer circle vertex #247
    // v=-2.43: (4.86, 30.36, 0) — inner circle vertex #248

    println!("=== STEP cone parameterization (r = radius + v * tan(ha)) ===");
    for v in [-2.43_f64, 0.0, 2.43] {
        let r = radius + v * tan_ha;
        let x = 2.43 - v; // origin.x + v * axis.x
        let y = r; // at u=0: cos(0)*x_dir.y * r = r
        println!("v={:.2}: point=({:.2}, {:.2}, 0), r={:.2}", v, x, y, r);
    }

    println!("\n=== Current code parameterization (r = radius - v * tan(ha)) ===");
    for v in [-2.43_f64, 0.0, 2.43] {
        let r = (radius - v * tan_ha).max(0.0);
        let x = 2.43 - v;
        let y = r;
        println!("v={:.2}: point=({:.2}, {:.2}, 0), r={:.2}", v, x, y, r);
    }

    println!("\n=== Expected 3D points from STEP file ===");
    println!("Vertex #247 (outer, r=35.22): (0.00, 35.22, 0)");
    println!("Vertex #248 (inner, r=30.36): (4.86, 30.36, 0)");
}
