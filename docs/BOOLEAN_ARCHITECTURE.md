# B-Rep Boolean Operations — Architecture Recommendations

Based on research of OpenCASCADE, Manifold, Cork, and other CAD kernels.

## Current State

### What Works
- Box - Box boolean: ✓ WATERTIGHT (0 boundary edges)
- Plane-plane intersection: exact Line curves
- Plane-cylinder intersection: exact Circle curves + PCurves
- Shared edge mechanism: both faces reference same Edge ID
- Face classification: multi-sample point-in-solid test

### What Doesn't Work Yet
- Box - Cylinder: 195 boundary edges (not watertight)
- Root cause: `triangulate_cylinder_tube_from_boundary` creates a regular
  grid that doesn't share vertices with the plane face's hole triangulation

## The Core Problem

The cylinder lateral face is triangulated by `triangulate_cylinder_tube_from_boundary`:
1. Collects 3D points from edge cache (shared edge = Circle, N points)
2. Splits into bottom_ring and top_ring, sorted by atan2(u)
3. Creates a grid: n_v+1 rows × n_u columns
4. Bottom row (j=0): uses cached bottom_ring points
5. Top row (j=n_v): uses cached top_ring points
6. Intermediate rows: `cyl.point_at(u, v)` — NEW points not in cache

The plane face (top/bottom of box) triangulates the intersection circle as a hole:
1. Collects 3D points from edge cache (same shared edge, same N points)
2. Passes to earcutr as a hole polyline
3. earcutr creates triangles connecting outer boundary to hole boundary

**The vertices should match** because both use the same edge cache entry.
**But they don't** because:
- The cylinder sorts points by atan2, reordering them
- The plane keeps points in parameter order (t=0 to t=2π)
- After sorting, the cylinder's vertex order differs from the plane's
- VertexDedupMap matches by coordinates, so order shouldn't matter
- BUT: the cylinder may create intermediate grid points at different v values
  that don't exist in the plane face

## Solution Architecture (OCCT-Inspired)

### Phase 1: PCurve-Based Triangulation (Immediate Fix)

Make the cylinder face triangulation use PCurves to evaluate boundary points
in UV space, ensuring identical 3D points with the plane face.

```
For each CoEdge with curve_2d (PCurve):
  - Evaluate PCurve at N parameter values → UV points
  - Map UV to 3D via surface.point_at(u, v)
  - Use these 3D points for triangulation
  - Result: bit-identical to plane face's hole points
```

**Implementation**: Modify `triangulate_cylinder_tube_from_boundary` to:
1. Check if CoEdges have `curve_2d` (PCurve)
2. If yes, evaluate PCurve to get UV points
3. Map UV → 3D via `cyl.point_at(u, v)`
4. Use these as boundary points (instead of cache + atan2 sort)

### Phase 2: UV-Space Face Splitting (Medium Term)

Implement proper UV-space face splitting like OCCT's BuilderFace:

1. **Intersection curves** produce PCurves on both surfaces
2. **Face builder** inserts PCurves as new wire edges in UV domain
3. **2D region computation** splits the UV domain into regions
4. Each region becomes a new face with proper UV boundaries

This eliminates the need for `split_boundary_into_rings_with_u` and
`atan2` sorting — the UV order is determined by the PCurve, not by
post-hoc projection.

### Phase 3: Common Block / Same-Domain Map (Long Term)

Implement OCCT's Common Block mechanism:

1. When multiple edges coincide (same vertices, same geometry),
   they form a Common Block
2. All edges in a Common Block resolve to a single "same-domain" (SD) edge
3. Both faces reference this one SD edge → topological watertightness

This handles cases where the boolean creates edges that should be shared
but have different TopoIds (e.g., two box faces that coincide after a fuse).

### Phase 4: PaveFiller-Style Intersection Part (Future)

Restructure the boolean pipeline into two clear phases:

**Intersection Part (IP)**:
1. VV: vertex-vertex interferences
2. VE: vertex-edge interferences
3. EE: edge-edge interferences
4. VF: vertex-face interferences
5. EF: edge-face interferences (pave blocks)
6. FF: face-face interferences (section edges + PCurves)
7. BuildSplitEdges: split edges at interference points
8. BuildSectionEdges: create shared section edges
9. MakePCurves: compute PCurves for all section edges
10. ProcessDE: handle degenerate edges (seams, poles)

**Building Part (BP)**:
1. BuildSplitFaces: reconstruct faces from split edges + PCurves
2. ClassifyFaces: determine IN/OUT/ON for each face piece
3. BuildSolid: assemble classified faces into result solid

## Key Design Decisions

### 1. PCurve Storage
- Store `Curve2d` on `CoEdge.curve_2d` (already exists!)
- For plane⊥cylinder: PCurve on cylinder = `Line2d((0, h), (2π, h))`
- For plane⊥cylinder: PCurve on plane = `Circle2d(center_uv, radius)`

### 2. Edge Discretization
- Cache by Edge ID (already works)
- When PCurve is present, compute UVs from PCurve (already works in `compute_uvs`)
- 3D points come from the analytic Curve3d (Circle, Line) — identical for all faces

### 3. Face Triangulation
- Plane faces: earcutr with holes (already works)
- Cylinder faces: must use PCurve-based UV evaluation
- Key change: `triangulate_cylinder_tube_from_boundary` should evaluate
  boundary from PCurve, not from `atan2` projection

### 4. Watertightness Guarantee
- **Structural**: shared Edge ID → same cache entry → same 3D points
- **PCurve-based**: both faces evaluate the same PCurve → same UV → same 3D
- **Vertex dedup**: safety net for floating-point drift

## Test Cases

### Must Pass (Watertight)
1. Box - Box (parallel faces) — ✓ PASSES
2. Box - Cylinder (perpendicular) — current focus
3. Box - Sphere
4. Cylinder - Cylinder (perpendicular axes)
5. Cylinder - Cylinder (parallel axes)
6. Sphere - Box
7. Torus - Box
8. Complex: Gear (cylinder + circular array - cylinder hole)

### Should Pass (Visual Quality)
1. Fillet edge (radius varying)
2. Chamfer edge
3. Shell (hollow solid)
4. Draft (tapered faces)
