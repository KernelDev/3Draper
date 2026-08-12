# B-Rep Boolean Operations — Reference Books & Resources

## Core Books

### 1. **Computational Geometry: Algorithms and Applications**
- Authors: Mark de Berg, Otfried Cheong, Marc van Kreveld, Mark Overmars
- Publisher: Springer, 3rd Edition (2008)
- ISBN: 978-3540779735
- **Why**: Foundational text on computational geometry — polygon triangulation, point location, arrangement of lines, Voronoi diagrams. Essential for understanding UV-space face splitting and 2D boolean operations.
- **Key chapters**: Ch 2 (Line Segment Intersection), Ch 3 (Polygon Triangulation), Ch 8 (Arrangements and Duality)

### 2. **Geometric Tools for Computer Graphics**
- Authors: Philip Schneider, David Eberly
- Publisher: Morgan Kaufmann (2002)
- ISBN: 978-1558605947
- **Why**: Comprehensive reference on distance computations, intersection algorithms (triangle-triangle, plane-cylinder, plane-sphere), and geometric primitives. Directly applicable to surface-surface intersection (SSI).
- **Key chapters**: Ch 5 (Distance Methods), Ch 6 (Intersection Methods), Ch 11 (Curves)

### 3. **Curves and Surfaces for Computer Aided Geometric Design: A Practical Guide**
- Author: Gerald Farin
- Publisher: Academic Press, 5th Edition (2001)
- ISBN: 978-1558607378
- **Why**: The definitive reference on NURBS, Bézier curves, B-splines, and surface parameterization. Essential for understanding PCurves, surface trimming, and parametric domains.
- **Key chapters**: Ch 9 (B-Splines), Ch 10 (NURBS), Ch 12 (Geometric Concepts), Ch 20 (Surfaces

### 4. **The NURBS Book**
- Authors: Les Piegl, Wayne Tiller
- Publisher: Springer, 2nd Edition (1997)
- ISBN: 978-3540615453
- **Why**: The "bible" of NURBS — covers surface representation, knot insertion, refinement, and intersection algorithms. Used by all major CAD kernels (OCCT, Parasolid, ACIS).
- **Key chapters**: Ch 4 (B-Spline Curves), Ch 5 (NURBS Curves), Ch 8 (NURBS Surfaces), Ch 10 (Curve/Surface Intersections)

### 5. **Computer Graphics: Principles and Practice**
- Authors: John F. Hughes, Andries van Dam, Morgan McGuire, David F. Sklar, James D. Foley, Steven K. Feiner, Kurt Akeley
- Publisher: Addison-Wesley, 3rd Edition (2013)
- ISBN: 978-0321399526
- **Why**: Broad coverage of mesh operations, transformation matrices, and rendering pipeline. Chapter on mesh operations covers watertightness, manifold detection, and edge collapse.
- **Key chapters**: Ch 2 (Miscellaneous Math), Ch 7 (Spatial Data Structures), Ch 12 (Meshes), Ch 15 (Curve and Surface Modeling)

### 6. **Geometric Modeling**
- Author: Michael E. Mortenson
- Publisher: Industrial Press, 3rd Edition (2006)
- ISBN: 978-0831132989
- **Why**: Covers B-Rep modeling, boundary representation, boolean operations, and topological data structures. Explains the half-edge data structure and Euler operators used in CAD kernels.
- **Key chapters**: Ch 2 (Geometric Elements), Ch 5 (Boundary Models), Ch 7 (Boolean Operations), Ch 9 (Topological Data Structures)

### 7. **Solid Modeling**
- Authors: Christoph M. Hoffmann, Joan R. Rossignac
- Publisher: Springer (1996, out of print but available online)
- **Why**: Comprehensive treatment of B-Rep, CSG, and boolean operations on solids. Covers the theory behind face splitting, edge classification, and shell assembly.
- **Key topics**: Non-manifold topology, boolean operation algorithms, feature-based modeling

### 8. **An Introduction to NURBS: With Historical Perspective**
- Author: David F. Rogers
- Publisher: Morgan Kaufmann (2000)
- ISBN: 978-1558606692
- **Why**: More accessible than The NURBS Book, with practical code examples. Good for understanding surface parameterization and PCurves.

## Advanced Papers & Technical References

### 9. **OpenCASCADE Source Code & Documentation**
- URL: https://dev.opencascade.org/doc/overview/html/specification__boolean_operations.html
- **Why**: The most comprehensive open-source B-Rep boolean implementation. Study the PaveFiller, Common Block, and BuilderFace algorithms.
- **Key files**: BOPAlgo_PaveFiller, BOPAlgo_Builder, BOPTools_AlgoTools, IntPatch_Intersection

### 10. **"A Fast Triangle-Triangle Intersection Test" (Möller 1997)**
- Journal: Journal of Graphics Tools, Vol 2, No 2
- **Why**: The standard algorithm for triangle-triangle intersection used in mesh booleans (Cork, Blender, Manifold).

### 11. **"Interactive & Robust Mesh Booleans" (EMBER paper, 2023)**
- Authors: lacoste et al., RWTH Aachen
- URL: https://graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans.pdf
- **Why**: State-of-the-art mesh boolean using exact predicates and winding numbers. Explains why mesh booleans are robust but B-Rep is exact.

### 12. **"Solid Modeling" by Satyaki Mahapatra (Lecture Notes)**
- URL: https://nptel.ac.in/courses/112/105/112105239/
- **Why**: NPTEL course on solid modeling covering B-Rep, CSG, and boolean operations with Indian Institute of Technology depth.

## Rust-Specific Resources

### 13. **Manifold Library (Rust-compatible via FFI)**
- URL: https://github.com/elalish/manifold
- **Why**: Modern mesh boolean library with winding-number approach. Can be used as a fallback for pathological cases.

### 14. **CGAL (Computational Geometry Algorithms Library)**
- URL: https://www.cgal.org/
- **Why**: Reference implementation of exact-predicate boolean operations. Study the 2D Boolean Operations package for UV-space face splitting.

### 15. **"Computer-Aided Geometric Design" Journal**
- Publisher: Elsevier
- **Why**: Academic journal with latest research on surface intersection, boolean operations, and topology repair. Search for "B-rep boolean" and "surface-surface intersection".

## Specific Topics to Study

### Surface-Surface Intersection (SSI)
- **Book**: Farin Ch 12, Piegl Ch 10
- **OCCT**: IntPatch_Intersection, IntTools_FaceFace
- **Key concept**: Marching squares on surface meshes + Newton-Raphson refinement → analytic curves (GLine) or B-spline approximations (WLine)

### PCurves (2D Curves on Surface)
- **Book**: Farin Ch 12 (surface parameterization)
- **OCCT**: IntTools_Curve (3D curve + 2 PCurves + tolerance)
- **Key concept**: Each face evaluates the intersection edge in its own UV space via PCurve → identical 3D points guaranteed

### Face Splitting in UV Space
- **Book**: de Berg Ch 2 (line segment intersection), Ch 3 (polygon triangulation)
- **OCCT**: BOPAlgo_BuilderFace
- **Key concept**: The face's boundary is a set of 2D loops in UV; intersection edges add new loops; 2D region computation splits the face

### Common Blocks / Shared Edges
- **OCCT**: BOPDS_CommonBlock, myShapesSD
- **Key concept**: When multiple edges coincide, they resolve to a single "same-domain" (SD) representative. Both faces reference this one edge object → topological watertightness

### Cylinder Seam Handling
- **Book**: Piegl Ch 8 (periodic NURBS surfaces)
- **OCCT**: ProcessDE step in PaveFiller
- **Key concept**: Cylinder U∈[0,2π) wraps periodically; the seam (u=0=2π) must be handled as a degenerate edge

## Recommended Reading Order

1. **Start**: Farin Ch 12 (surface parameterization) — understand UV space
2. **Next**: de Berg Ch 2-3 — 2D polygon operations
3. **Then**: OCCT Boolean spec — see how it all fits together
4. **Deep dive**: Piegl Ch 10 — NURBS intersection
5. **Practice**: Study OCCT source (BOPAlgo_PaveFiller)
6. **Fallback**: Manifold wiki — mesh boolean theory
