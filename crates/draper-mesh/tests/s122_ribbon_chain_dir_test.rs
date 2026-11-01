//! s122: contiguous-direction chain projection + per-face drift gate.
//!
//! MEASURED ROOT CAUSE (s122, forensics EDGEPTS on Vulcan #10220): every
//! drifted ribbon edge has its raw discretization REVERSED relative to the
//! face-loop traversal, so the legacy chain seed (previous edge's last UV)
//! sits a full coil-bundle diameter (~0.73) away from its target and the
//! 2D Newton lands on a neighbouring coil branch (v off by ±0.098, u off
//! by whole turns) while every 3D-contiguous edge projects at ~1e-7.
//!
//! `ribbon_chain_reversal` detects the reversed arrays (the chain must
//! walk them back-to-front); `ribbon_drift_gate` is the per-face failure
//! threshold of the s119-п.1 plan (p90 vs median reprojection error) that
//! triggers the independent-projection fallback in the converter.

use draper_geometry::Point3d;
use draper_mesh::{ribbon_chain_reversal, ribbon_drift_gate};

#[test]
fn s122_chain_reversal_detection() {
    // FWD array: raw first continues the previous edge's end.
    let prev = Point3d::new(0.0, 0.0, 0.0);
    let fwd = vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0)];
    assert!(!ribbon_chain_reversal(&prev, &fwd));

    // REV array: raw LAST continues the previous edge's end (the #10220
    // pattern — e.g. f25 e9: raw first one coil-bundle away, raw last
    // 0.012 from the shared vertex).
    let rev = vec![Point3d::new(0.73, 0.1, 0.0), Point3d::new(0.01, 0.0, 0.0)];
    assert!(ribbon_chain_reversal(&prev, &rev));

    // GAP array: neither end contiguous (loop wrap / after a notch) —
    // keep the legacy forward chaining (strictly-closer test).
    let gap = vec![Point3d::new(5.0, 5.0, 5.0), Point3d::new(6.0, 5.0, 5.0)];
    assert!(!ribbon_chain_reversal(&prev, &gap));

    // Equidistant ends (2-point edge seen from the middle): ties are
    // NOT reversals.
    let tie = vec![Point3d::new(-1.0, 0.0, 0.0), Point3d::new(1.0, 0.0, 0.0)];
    assert!(!ribbon_chain_reversal(&prev, &tie));

    // Degenerate arrays are never reversed.
    assert!(!ribbon_chain_reversal(&prev, &[]));
    assert!(!ribbon_chain_reversal(&prev, &[Point3d::new(1.0, 1.0, 1.0)]));
}

#[test]
fn s122_drift_gate_thresholds() {
    // Clean ribbon loop: all points ~1e-7 (the measured f25/f27 rails).
    let clean: Vec<f64> = (0..200).map(|i| 1e-7 * (1.0 + (i % 7) as f64 * 0.1)).collect();
    assert!(!ribbon_drift_gate(&clean));

    // f27-class drift: 21% of points coil-jumped to ~6e-2 while the rest
    // sit at 1e-7 — p90 lands on the drifted population → fires.
    let mut drifted: Vec<f64> = vec![1e-7; 158];
    drifted.extend(vec![6.0e-2; 42]);
    drifted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!(ribbon_drift_gate(&drifted));

    // f25-notch-class drift: a sub-population at 5-6e-2 among the rail
    // points at 1e-7. 15% of the loop (the notch walls + seam segments)
    // — p90 lands on the drifted population → fires. (At exactly 10%
    // the p90 index still sits in the clean population — the gate
    // deliberately sees only the top decile.)
    let mut notch: Vec<f64> = vec![1e-7; 1700];
    notch.extend(vec![5.2e-2; 300]);
    notch.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!(ribbon_drift_gate(&notch));

    // UNIFORMLY sloppy curve (all points at the same moderate error —
    // e.g. a curve lying 1e-2 off the surface everywhere): p90 == median
    // → consistent, NOT drift → does not fire.
    let uniform: Vec<f64> = vec![1.0e-2; 100];
    assert!(!ribbon_drift_gate(&uniform));

    // Boundary cases.
    assert!(!ribbon_drift_gate(&[]));
    assert!(!ribbon_drift_gate(&[1e-7]));
    // Median floor: a single-point population at exactly 1e-6 median
    // with p90 at 1e-5 is 10× — fires.
    let mut spiky: Vec<f64> = vec![1e-6; 90];
    spiky.extend(vec![1e-5; 10]);
    spiky.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!(ribbon_drift_gate(&spiky));
}
