#!/usr/bin/env python3
"""s131: stage tracking of the #10220 f(28,28) REAL pair (h=0.0124,0.0847).

The pair triangles are MERGE-ASSEMBLED (verts = f28 emission rim verts
v117/v409/v72/v92, but the triples exist in NO emission). Bisect the
per-BREP repair chain (DRAPPER_DUMP_STAGE_OBJS stage dumps, local
frame) to find the stage where the pair first appears, and the face
attribution (fmap) of its triangles at that stage.

Target (LOCAL frame, from inverse INST of census VERTS, 4-dp print):
  shared edge A=(-0.2766, 7.0586, 2.2745) B=(-0.1684, 7.0701, 2.3829)
  apex0 P=(-0.2216, 7.0567, 2.3426)  h=0.0124
  apex1 Q=(-0.2207, 7.0120, 2.4192)  h=0.0847
  mid  M=(-0.2236, 7.0655, 2.3250)
"""
import os, math, re
from collections import defaultdict

DUMP = "/tmp/s131_stages10220"
STAGES = [
    "b10220-d-after-merge",
    "b10220-d-after-weld",
    "b10220-d-after-tj",
    "b10220-d-after-gapfill",
    "b10220-d-after-winding",
    "d-after-flapclean",
    "d-after-bornfold-split",
    "d-after-postweldwind",
]
PASSES = ["p0", "p1"]

A = (-0.2766, 7.0586, 2.2745)
B = (-0.1684, 7.0701, 2.3829)
P = (-0.2216, 7.0567, 2.3426)
Q = (-0.2207, 7.0120, 2.4192)
MID = (-0.2236, 7.0655, 2.3250)
HT = (0.0124, 0.0847)


def d(p, q):
    return math.dist(p, q)


def load_obj(path):
    verts, tris = [], []
    for ln in open(path, errors="replace"):
        if ln.startswith("v "):
            _, x, y, z = ln.split()
            verts.append((float(x), float(y), float(z)))
        elif ln.startswith("f "):
            _, i, j, k = ln.split()
            tris.append((int(i) - 1, int(j) - 1, int(k) - 1))
    return verts, tris


def load_fmap(path):
    fm = {}
    if not os.path.exists(path):
        return fm
    for ln in open(path, errors="replace"):
        if ln.startswith("t "):
            _, ti, fid = ln.split()
            fm[int(ti)] = int(fid)
    return fm


def tri_normal(verts, t):
    (a, b, c) = (verts[t[0]], verts[t[1]], verts[t[2]])
    ab = [b[i] - a[i] for i in range(3)]
    ac = [c[i] - a[i] for i in range(3)]
    n = [ab[1] * ac[2] - ab[2] * ac[1],
         ab[2] * ac[0] - ab[0] * ac[2],
         ab[0] * ac[1] - ab[1] * ac[0]]
    l = math.sqrt(sum(x * x for x in n))
    return [x / l for x in n] if l > 1e-18 else None


def scan_stage(passname, stage):
    obj = f"{DUMP}/{passname}_{stage}.obj"
    fmap = f"{DUMP}/{passname}_{stage}.fmap"
    if not os.path.exists(obj):
        return None
    verts, tris = load_obj(obj)
    fm = load_fmap(fmap)
    edge_tris = defaultdict(list)
    for ti, t in enumerate(tris):
        for k in range(3):
            x, y = t[k], t[(k + 1) % 3]
            if x != y:
                edge_tris[(min(x, y), max(x, y))].append(ti)
    hits = []
    for (x, y), ts_ in edge_tris.items():
        if len(ts_) != 2:
            continue
        t0, t1 = tris[ts_[0]], tris[ts_[1]]
        n0, n1 = tri_normal(verts, t0), tri_normal(verts, t1)
        if not n0 or not n1:
            continue
        dot = max(-1.0, min(1.0, sum(n0[i] * n1[i] for i in range(3))))
        ang = math.degrees(math.acos(dot))
        if ang <= 170.0:
            continue
        # apexes
        s0, s1 = set(t0), set(t1)
        shared = s0 & s1
        if len(shared) != 2:
            continue
        ap0 = list(s0 - shared)[0]
        ap1 = list(s1 - shared)[0]
        a, b = verts[x], verts[y]
        ab = [b[i] - a[i] for i in range(3)]
        abl = math.sqrt(sum(v * v for v in ab))
        if abl < 1e-12:
            continue

        def line_dist(p):
            ap = [p[i] - a[i] for i in range(3)]
            cr = [ap[1] * ab[2] - ap[2] * ab[1],
                  ap[2] * ab[0] - ap[0] * ab[2],
                  ap[0] * ab[1] - ap[1] * ab[0]]
            return math.sqrt(sum(v * v for v in cr)) / abl

        h0, h1 = line_dist(verts[ap0]), line_dist(verts[ap1])
        mid = tuple((verts[x][i] + verts[y][i]) / 2 for i in range(3))
        # match: heights (unordered) within 15%+2e-3 AND midpoint near MID
        hs = sorted((h0, h1))
        ht = sorted(HT)
        h_ok = all(abs(hs[i] - ht[i]) <= 0.15 * ht[i] + 2e-3 for i in range(2))
        pos_ok = d(mid, MID) < 0.08
        if h_ok and pos_ok:
            hits.append({
                "edge": (x, y), "tris": (ts_[0], ts_[1]),
                "t_ids": (t0, t1), "h": (h0, h1), "ang": ang,
                "fids": (fm.get(ts_[0]), fm.get(ts_[1])),
                "apex_pos": (verts[ap0], verts[ap1]),
            })
    return len(tris), hits


print(f"Target pair local: shared A={A} B={B}")
print(f"  apex P={P} (h=0.0124)  Q={Q} (h=0.0847)  MID={MID}")
print()
for p in PASSES:
    for st in STAGES:
        r = scan_stage(p, st)
        if r is None:
            print(f"{p} {st}: (no dump)")
            continue
        ntris, hits = r
        if not hits:
            print(f"{p} {st}: {ntris} tris, pair ABSENT")
        else:
            for h in hits:
                print(f"{p} {st}: {ntris} tris, pair PRESENT "
                      f"tris={h['tris']} fids={h['fids']} "
                      f"h=({h['h'][0]:.4f},{h['h'][1]:.4f}) ang={h['ang']:.2f} "
                      f"edge_mid=({h['edge'][0]},{h['edge'][1]})")
    print()
