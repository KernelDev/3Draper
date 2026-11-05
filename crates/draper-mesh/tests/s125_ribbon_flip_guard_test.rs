//! s125: ribbon-scoped merge self flip-weld refusal.
//!
//! MEASURED ROOT CAUSE (s125 forensics, Vulcan #10220 f27): ribbon-coil
//! faces carry near-duplicate polygon points from ADJACENT COIL TURNS
//! (inter-turn pitch 0.014-0.018 < merge_tol 0.02). The merge tolerance
//! path welds a triangle APEX into a NEIGHBORING TURN's point slot,
//! moving it 0.7-0.9×tol ACROSS the shared edge and manufacturing
//! same-face FOLD-OVER pairs (the f27 runout residue: 2 FAT + 1 SLIVER
//! REAL pairs + 20 sub-tol). Refusing the self flip-weld on
//! ribbon-flagged faces kills all of them (Vulcan 10→7 REAL).
//!
//! The UNSCOPED broad guard ("1", all faces) is NOT a net win: on the
//! #10203 thin crescents the same welds drop needle triangles as
//! same-face duplicates (cleanup), so the broad guard SURFACES 5 new
//! REAL pairs there (net 10→10) — the ribbon scope keeps the crescents
//! untouched (measured: all non-#10220 census lines bit-identical).
//!
//! Scenario geometry (mirrors the measured mechanism): a thin triangle
//! (A, B, V) with apex V just above its base, and an unused "orphan"
//! point O just BELOW the base within merge_tol of V. Without the
//! guard, V tolerance-welds into O's slot (O is processed first, added
//! as a new vertex of the same merge call = SELF weld) and the apex
//! lands on the far side of the base — orientation flip.

use draper_geometry::Point3d;
use draper_mesh::mesh::VertexDedupMap;
use draper_mesh::parametric_domain::{
    clear_ribbon_merge_guard, set_ribbon_merge_guard,
};
use draper_mesh::TriangleMesh;
use std::sync::Mutex;

/// Env-var flips are process-global — serialize the env-manipulating
/// scenarios (the s98 pattern).
static S125_ENV_LOCK: Mutex<()> = Mutex::new(());

/// The incoming "face": vertices [O, A, B, V] with ONE thin triangle
/// (A, B, V) whose apex V sits 0.005 above the base A-B; the orphan O
/// sits 0.005 BELOW the base, 0.01 from V (inside merge_tol 0.02).
/// O comes FIRST so the merge adds it as the target slot (self weld).
fn incoming_face() -> TriangleMesh {
    let mut m = TriangleMesh::new();
    let o = m.add_vertex(Point3d::new(0.5, -0.005, 0.0)); // 0: orphan slot
    let a = m.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // 1: base start
    let b = m.add_vertex(Point3d::new(1.0, 0.0, 0.0)); // 2: base end
    let v = m.add_vertex(Point3d::new(0.5, 0.005, 0.0)); // 3: apex (above)
    assert_eq!((o, a, b, v), (0, 1, 2, 3));
    // The only triangle: apex ABOVE the base, CCW around +z.
    m.add_triangle_arr([a, b, v]);
    m.triangle_face_ids = Some(vec![7; 1]);
    m
}

/// Whether the merged mesh's surviving triangle still has its apex at
/// V's position (guard refused the weld) or at O's position (weld
/// happened, apex dragged across the base).
fn apex_is_at_v(mesh: &TriangleMesh) -> Option<bool> {
    let v = Point3d::new(0.5, 0.005, 0.0);
    let o = Point3d::new(0.5, -0.005, 0.0);
    let tri = mesh.triangles.first()?;
    let apex_idx = tri[2] as usize; // the triangle was added as (a, b, v)
    let apex = mesh.vertices[apex_idx];
    let dv = (apex.x - v.x).abs() + (apex.y - v.y).abs() + (apex.z - v.z).abs();
    let d_o = (apex.x - o.x).abs() + (apex.y - o.y).abs() + (apex.z - o.z).abs();
    Some(dv < d_o)
}

#[test]
fn s125_ribbon_self_flip_weld_scenarios() {
    let _guard = S125_ENV_LOCK.lock().unwrap();

    // ── 1. DEFAULT (env unset): ribbon-flagged face → weld REFUSED ──
    std::env::remove_var("DRAPPER_MERGE_FLIP_GUARD");
    {
        let mut target = TriangleMesh::new();
        let mut dedup = VertexDedupMap::with_tolerance(0.02);
        set_ribbon_merge_guard();
        target.merge_deduplicating(&incoming_face(), &mut dedup);
        clear_ribbon_merge_guard();
        assert_eq!(
            apex_is_at_v(&target),
            Some(true),
            "ribbon-scoped default: the self flip-weld must be refused"
        );
    }

    // ── 2. KILL-SWITCH (=0): ribbon flag set → weld RESTORED ────────
    std::env::set_var("DRAPPER_MERGE_FLIP_GUARD", "0");
    {
        let mut target = TriangleMesh::new();
        let mut dedup = VertexDedupMap::with_tolerance(0.02);
        set_ribbon_merge_guard();
        target.merge_deduplicating(&incoming_face(), &mut dedup);
        clear_ribbon_merge_guard();
        assert_eq!(
            apex_is_at_v(&target),
            Some(false),
            "kill-switch =0: bit-identical pre-s125 behavior (weld happens)"
        );
    }

    // ── 3. DEFAULT, NON-ribbon face → weld happens (baseline) ───────
    std::env::remove_var("DRAPPER_MERGE_FLIP_GUARD");
    {
        let mut target = TriangleMesh::new();
        let mut dedup = VertexDedupMap::with_tolerance(0.02);
        // no set_ribbon_merge_guard: the incoming face is not a ribbon
        target.merge_deduplicating(&incoming_face(), &mut dedup);
        assert_eq!(
            apex_is_at_v(&target),
            Some(false),
            "non-ribbon face under the default scope: weld unchanged"
        );
    }

    // ── 4. BROAD mode ("1", all faces) → self weld refused too ──────
    std::env::set_var("DRAPPER_MERGE_FLIP_GUARD", "1");
    {
        let mut target = TriangleMesh::new();
        let mut dedup = VertexDedupMap::with_tolerance(0.02);
        target.merge_deduplicating(&incoming_face(), &mut dedup);
        assert_eq!(
            apex_is_at_v(&target),
            Some(true),
            "broad mode 1: the s62 self-flip refusal on every face"
        );
    }

    // restore the default state for the rest of the process
    std::env::remove_var("DRAPPER_MERGE_FLIP_GUARD");
}
