// Check what effective_chord_tolerance returns in the STEP path
// and whether the edge cache override is set
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_geometry::tolerance::ToleranceContext;

fn main() {
    // Simulate what the STEP path does
    let bbox_min = draper_geometry::Point3d::new(-37.5, -37.5, -5.0);
    let bbox_max = draper_geometry::Point3d::new(44.0, 37.5, 37.5);
    let tol_ctx = ToleranceContext::from_bounding_box(&bbox_min, &bbox_max);
    println!("ToleranceContext: model_scale={:.4} absolute={:.6}", tol_ctx.model_scale, tol_ctx.absolute);

    let cache = EdgeDiscretizationCache::with_tolerance(tol_ctx.clone(), 64);
    println!("effective_chord_tolerance (no override): {:.6}", cache.effective_chord_tolerance());
    println!("adaptive_tol.chord_tolerance(): {:.6}", tol_ctx.model_scale * 1e-5);

    // With override (what triangulate_solid does)
    let mut cache2 = EdgeDiscretizationCache::with_tolerance(tol_ctx.clone(), 64);
    cache2.set_chord_tolerance_override(Some(0.01)); // LOD 1.0
    println!("\nWith override=0.01: effective={:.6}", cache2.effective_chord_tolerance());

    let mut cache3 = EdgeDiscretizationCache::with_tolerance(tol_ctx.clone(), 64);
    cache3.set_chord_tolerance_override(Some(0.04)); // LOD 0.5
    println!("With override=0.04: effective={:.6}", cache3.effective_chord_tolerance());
}
