#!/usr/bin/env python3
"""Session-56: TRUE overlap-area census of >170° FOLD-OVER pairs.

For each topo-consistent same-side pair (the probe's FOLD-OVER class),
compute the actual 2D overlap area of the two triangles (projection onto
the plane of the first triangle's normal). True fold-overs have positive
overlap; degenerate/classification noise has ~0.
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

obj = Path(sys.argv[1])
verts, tris = [], []
for line in obj.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))
fid_of_tri = {}
for line in obj.with_suffix(".fmap").read_text().splitlines():
    if line.startswith("t "):
        _, ti, fid = line.split()
        fid_of_tri[int(ti)] = int(fid)


def sub(p, q): return (p[0]-q[0], p[1]-q[1], p[2]-q[2])
def cross(p, q): return (p[1]*q[2]-p[2]*q[1], p[2]*q[0]-p[0]*q[2], p[0]*q[1]-p[1]*q[0])
def dot(p, q): return p[0]*q[0]+p[1]*q[1]+p[2]*q[2]
def norm(p): return math.sqrt(dot(p, p))


def tri_normal(t):
    n = cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]]))
    l = norm(n)
    return (n[0]/l, n[1]/l, n[2]/l) if l > 1e-30 else None


def project(pt, origin, n, u_axis, v_axis):
    d = sub(pt, origin)
    return (dot(d, u_axis), dot(d, v_axis))


def poly_clip_overlap(A, B):
    """Sutherland-Hodgman: clip polygon A by polygon B, return area.
    Both polygons are normalized to CCW orientation first."""
    def signed_area(P):
        s = 0.0
        for i in range(len(P)):
            s += P[i][0]*P[(i+1) % len(P)][1] - P[(i+1) % len(P)][0]*P[i][1]
        return s / 2
    if signed_area(A) < 0:
        A = A[::-1]
    if signed_area(B) < 0:
        B = B[::-1]
    def inside(p, a, b):
        return (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0]) >= 0
    def intersect(p1, p2, a, b):
        d1 = (b[0]-a[0])*(p1[1]-a[1]) - (b[1]-a[1])*(p1[0]-a[0])
        d2 = (b[0]-a[0])*(p2[1]-a[1]) - (b[1]-a[1])*(p2[0]-a[0])
        t = d1 / (d1 - d2) if abs(d1-d2) > 1e-30 else 0.0
        return (p1[0]+t*(p2[0]-p1[0]), p1[1]+t*(p2[1]-p1[1]))
    out = list(A)
    nb = len(B)
    for i in range(nb):
        a, b = B[i], B[(i+1) % nb]
        inp = out
        out = []
        if not inp:
            break
        for j in range(len(inp)):
            p, q = inp[j], inp[(j+1) % len(inp)]
            pin = inside(p, a, b)
            qin = inside(q, a, b)
            if pin:
                out.append(p)
                if not qin:
                    out.append(intersect(p, q, a, b))
            elif qin:
                out.append(intersect(p, q, a, b))
    if len(out) < 3:
        return 0.0
    s = 0.0
    for i in range(len(out)):
        s += out[i][0]*out[(i+1) % len(out)][1] - out[(i+1) % len(out)][0]*out[i][1]
    return abs(s)/2


edge_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(tris):
    for e in ((a, b), (b, c), (c, a)):
        edge_tris[(min(e), max(e))].append((ti, e[0], e[1]))

normals = [tri_normal(t) for t in tris]
fold_pairs = 0
true_overlap = 0.0
overlap_per_face = defaultdict(float)
count_per_face = defaultdict(int)
for (e, tl) in edge_tris.items():
    if len(tl) != 2:
        continue
    (t0, a0, b0), (t1, a1, b1) = tl
    n0, n1 = normals[t0], normals[t1]
    if not n0 or not n1:
        continue
    ang = math.degrees(math.acos(max(-1, min(1, dot(n0, n1)))))
    if ang <= 170.0:
        continue
    topo_consistent = (a0 == b1) and (b0 == a1)
    if not topo_consistent:
        continue
    a, b = verts[e[0]], verts[e[1]]
    ed = sub(b, a)

    def apex_side(ti):
        for v in tris[ti]:
            if v != e[0] and v != e[1]:
                return cross(ed, sub(verts[v], a)), tris[ti]
        return (0, 0, 0), tris[ti]

    s0, tri0 = apex_side(t0)
    s1, tri1 = apex_side(t1)
    if dot(s0, s1) <= 0:
        continue  # opposite side: CURVED-180, benign
    fold_pairs += 1
    # overlap area in the plane of t0
    origin = verts[tri0[0]]
    u_axis = sub(verts[tri0[1]], origin)
    u_axis = tuple(x / norm(u_axis) for x in u_axis)
    v_axis = cross(n0, u_axis)
    A = [project(verts[v], origin, n0, u_axis, v_axis) for v in tri0]
    B = [project(verts[v], origin, n0, u_axis, v_axis) for v in tri1]
    ov = poly_clip_overlap(A, B)
    if ov > 1e-12:
        true_overlap += ov
        fid = fid_of_tri.get(t0, -1)
        overlap_per_face[fid] += ov
        count_per_face[fid] += 1

print(f"file: {obj.name}")
print(f"same-side topo-consistent >170° pairs: {fold_pairs}")
print(f"pairs with REAL 2D overlap (>1e-12 mm²): {sum(count_per_face.values())}")
print(f"total overlap area: {true_overlap:.3e} mm²")
top = sorted(overlap_per_face.items(), key=lambda kv: -kv[1])[:8]
for fid, ar in top:
    print(f"   face {fid}: {count_per_face[fid]} overlapping pairs, area {ar:.3e} mm²")
