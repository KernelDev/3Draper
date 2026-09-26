#!/usr/bin/env python3
"""Session-57: split final-mesh FOLD-OVER pairs into same-face vs
cross-face, with real 2D overlap area for each class.

Pre-merge FO=0 (FACEFOLD session-57 extension) proves per-face
triangulations contain NO real overlaps — so every final FOLD-OVER pair
is either created at merge/post-merge stages (same-face) or is a
cross-face pair (two different faces nearly tangent).

Usage: sameface_split.py <dir-with-obj+fmap> [--ov]
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

d = Path(sys.argv[1])
obj = next(d.glob("*.obj"))
fmap = obj.with_suffix(".fmap")

verts, tris = [], []
for line in obj.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))
fid_of_tri = {}
for line in fmap.read_text().splitlines():
    if line.startswith("t "):
        _, ti, fid = line.split()
        fid_of_tri[int(ti)] = int(fid)


def sub(p, q):
    return (p[0] - q[0], p[1] - q[1], p[2] - q[2])


def cross(p, q):
    return (p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0])


def dot(p, q):
    return p[0] * q[0] + p[1] * q[1] + p[2] * q[2]


def norm(p):
    return math.sqrt(dot(p, p))


def tri_normal(t):
    n = cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]]))
    l = norm(n)
    return (n[0] / l, n[1] / l, n[2] / l) if l > 1e-30 else None


def clip_area(a, b):
    """2D overlap area of two triangles (Sutherland-Hodgman)."""
    e1 = sub(a[1], a[0])
    e2 = sub(a[2], a[0])
    n = cross(e1, e2)
    nl = norm(n)
    if nl < 1e-15:
        return None
    n = tuple(x / nl for x in n)
    ul = norm(e1)
    if ul < 1e-15:
        return None
    u = tuple(x / ul for x in e1)
    v = cross(n, u)

    def to2(p):
        dd = sub(p, a[0])
        return (dot(dd, u), dot(dd, v))

    pa = [to2(a[0]), to2(a[1]), to2(a[2])]
    pb = [to2(b[0]), to2(b[1]), to2(b[2])]
    signed = (pa[1][0] - pa[0][0]) * (pa[2][1] - pa[0][1]) - (pa[1][1] - pa[0][1]) * (pa[2][0] - pa[0][0])
    clip = pa if signed >= 0 else pa[::-1]
    subject = pb[:]
    for i in range(3):
        if not subject:
            break
        ca, cb = clip[i], clip[(i + 1) % 3]

        def sidef(p):
            return (cb[0] - ca[0]) * (p[1] - ca[1]) - (cb[1] - ca[1]) * (p[0] - ca[0])

        out = []
        n_s = len(subject)
        for j in range(n_s):
            cur, nxt = subject[j], subject[(j + 1) % n_s]
            sc, sn = sidef(cur), sidef(nxt)
            if sc >= 0:
                out.append(cur)
            if (sc > 0 and sn < 0) or (sc < 0 and sn > 0):
                t = sc / (sc - sn)
                out.append((cur[0] + t * (nxt[0] - cur[0]), cur[1] + t * (nxt[1] - cur[1])))
        subject = out
    if len(subject) < 3:
        return 0.0
    area = 0.0
    for i in range(len(subject)):
        j = (i + 1) % len(subject)
        area += subject[i][0] * subject[j][1] - subject[j][0] * subject[i][1]
    return abs(area) * 0.5


normals = [tri_normal(t) for t in tris]
edge_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(tris):
    for e in ((a, b), (b, c), (c, a)):
        edge_tris[(min(e), max(e))].append((ti, e[0], e[1]))

cls_counts = defaultdict(int)
cls_area = defaultdict(float)
cls_pairs_faces = defaultdict(int)  # for cross-face: which face pairs
ov_examples = defaultdict(list)
for (e, tl) in edge_tris.items():
    if len(tl) != 2:
        continue
    (t0, a0, b0), (t1, a1, b1) = tl
    n0, n1 = normals[t0], normals[t1]
    if not n0 or not n1:
        continue
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(n0, n1)))))
    if ang <= 170.0:
        continue
    f0, f1 = fid_of_tri.get(t0, -1), fid_of_tri.get(t1, -1)
    topo_consistent = (a0 == b1) and (b0 == a1)
    a, b = verts[e[0]], verts[e[1]]
    ed = sub(b, a)

    def apex_side(ti, e0, e1):
        for v in tris[ti]:
            if v != e0 and v != e1:
                return cross(ed, sub(verts[v], a))
        return (0.0, 0.0, 0.0)

    s0, s1 = apex_side(t0, e[0], e[1]), apex_side(t1, e[0], e[1])
    dd = dot(s0, s1)
    same_side = dd > 0
    if not topo_consistent:
        cls = "WINDING-FLIP" if same_side else "DOUBLE-BROKEN"
    elif same_side:
        cls = "FO-sameface" if f0 == f1 else "FO-crossface"
    else:
        cls = "CURVED-180-same" if f0 == f1 else "CURVED-180-cross"
    cls_counts[cls] += 1
    if cls.startswith("FO"):
        area = clip_area(
            [verts[x] for x in tris[t0]], [verts[x] for x in tris[t1]]
        )
        if area is not None:
            cls_area[cls] += area
            if len(ov_examples[cls]) < 5 or area > ov_examples[cls][-1][0]:
                ov_examples[cls].append((area, f0, f1, t0, t1, e))
                ov_examples[cls].sort(reverse=True)
                ov_examples[cls] = ov_examples[cls][:5]
    if cls == "FO-crossface":
        cls_pairs_faces[(min(f0, f1), max(f0, f1))] += 1

print("class                                count      total_ov_area(mm^2)")
for cls in sorted(cls_counts, key=lambda c: -cls_counts[c]):
    print(f"{cls:<28} {cls_counts[cls]:>8}   {cls_area[cls]:.4e}")

print("\nTop cross-face FO pairs (faceA,faceB):")
for (fa, fb), n in sorted(cls_pairs_faces.items(), key=lambda kv: -kv[1])[:15]:
    print(f"  faces ({fa},{fb}): {n}")

for cls, ex in ov_examples.items():
    print(f"\nlargest {cls} overlaps (area, face0, face1, tri0, tri1, edge):")
    for area, f0, f1, t0, t1, e in ex:
        print(f"  {area:.4e}  f{f0}/f{f1}  t{t0}/t{t1}  edge {e}")
