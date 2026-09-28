#!/usr/bin/env python3
"""s64 diag: find OVERLAPPING triangles in the CDT output (post-rescue dump).

Two consistently-wound triangles on a developable surface can only show
~180° dihedrals if they geometrically OVERLAP (double coverage). Detect:
pairs of triangles with positive signed area whose interiors intersect
(edge crossing or containment).
"""
import sys

def load(path):
    bnd, holes_pts, interior, tris = [], [], [], []
    mode = None
    for line in open(path):
        line = line.strip()
        if line.startswith("type="):
            continue
        if line in ("boundary", "interior", "tris"):
            mode = line
            continue
        if line.startswith("hole ") or not line:
            continue
        p = line.split()
        if p[0] == "b":
            bnd.append((float(p[1]), float(p[2])))
        elif p[0] == "h":
            holes_pts.append((float(p[1]), float(p[2])))
        elif p[0] == "i":
            interior.append((float(p[1]), float(p[2])))
        elif p[0] == "t":
            tris.append((int(p[1]), int(p[2]), int(p[3])))
    return bnd, holes_pts, interior, tris

def seg_int(p1, p2, p3, p4, eps=1e-12):
    def d(o, a, b):
        return (b[0] - o[0]) * (a[1] - o[1]) - (b[1] - o[1]) * (a[0] - o[0])
    d1, d2 = d(p3, p4, p1), d(p3, p4, p2)
    d3, d4 = d(p1, p2, p3), d(p1, p2, p4)
    return ((d1 > eps) != (d2 > eps)) and ((d3 > eps) != (d4 > eps))

def tri_overlap(A, B):
    """A, B = [(x,y)*3]. True if interiors intersect (edge crossing)."""
    for i in range(3):
        a1, a2 = A[i], A[(i + 1) % 3]
        for j in range(3):
            b1, b2 = B[j], B[(j + 1) % 3]
            if seg_int(a1, a2, b1, b2):
                return True
    return False

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/s64_tri_fix/tri_0000_Cylinder.txt"
    bnd, holes_pts, interior, tris = load(path)
    n_b = len(bnd)
    all_uv = bnd + holes_pts + interior
    pts = [all_uv[t[0]] for t in []]  # noqa
    T = [(all_uv[t[0]], all_uv[t[1]], all_uv[t[2]]) for t in tris]
    print(f"{path}: {len(tris)} tris")
    # edge-usage census (find non-manifold in CDT output)
    from collections import defaultdict
    ecnt = defaultdict(int)
    for t in tris:
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            ecnt[(min(a, b), max(a, b))] += 1
    nm = [e for e, n in ecnt.items() if n > 2]
    bnd_e = [e for e, n in ecnt.items() if n == 1]
    print(f"edge usage: >2: {len(nm)}, ==1 (region boundary): {len(bnd_e)}")
    # overlaps: brute force with early bbox filter (n~300 -> 45k pairs, ok)
    overlaps = []
    for i in range(len(T)):
        ax = [p[0] for p in T[i]]; ay = [p[1] for p in T[i]]
        for j in range(i + 1, len(T)):
            bx = [p[0] for p in T[j]]; by = [p[1] for p in T[j]]
            if min(ax) > max(bx) or max(ax) < min(bx) or min(ay) > max(by) or max(ay) < min(by):
                continue
            if tri_overlap(T[i], T[j]):
                overlaps.append((i, j))
    print(f"OVERLAPPING triangle pairs: {len(overlaps)}")
    for i, j in overlaps[:10]:
        print(f"  tri{i} {tris[i]} x tri{j} {tris[j]}")

if __name__ == "__main__":
    main()
