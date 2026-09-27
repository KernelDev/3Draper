#!/usr/bin/env python3
"""Session-62: correlate the 18 FOLD-OVER pairs (GEAR f1/f12, after-merge
stage) with TOLWELD points to prove/disprove the 'merge welding creates the
pairs' mechanism.

Inputs:
  /tmp/stages/p0_d-after-merge.obj+fmap   (pairs source)
  /tmp/welds.txt                          (TOLWELD lines: idx d p q)

For each pair edge endpoint / apex, find TOLWELD entries whose q (existing,
kept vertex) matches within eps. Also verify: does the shared edge chain form
a contiguous 'zipper' along welded vertices?
"""
import sys, math
from collections import defaultdict

sys.path.insert(0, '/home/z/my-project/scripts')
from stage_pair_track import load_obj, load_fmap, normal

OBJ = '/tmp/stages/p0_d-after-merge.obj'
FMAP = '/tmp/stages/p0_d-after-merge.fmap'
WELDS = '/tmp/welds.txt'
EPS = 1e-7

v, tris = load_obj(OBJ)
fm = load_fmap(FMAP)

# target faces
FACES = {1, 12}

edge_map = defaultdict(list)
for ti, tri in enumerate(tris):
    a, b, c = tri
    for v0, v1 in ((a, b), (b, c), (c, a)):
        key = (v0, v1) if v0 < v1 else (v1, v0)
        edge_map[key].append((ti, v0, v1))

pairs = []
for edge, lst in edge_map.items():
    if len(lst) != 2:
        continue
    (t0, da0, db0), (t1, da1, db1) = lst
    f0, f1 = fm.get(t0), fm.get(t1)
    if f0 != f1 or f0 not in FACES:
        continue
    n0, n1 = normal(v, tris[t0]), normal(v, tris[t1])
    if not n0 or not n1:
        continue
    dot = sum(n0[i]*n1[i] for i in range(3))
    ang = math.degrees(math.acos(max(-1, min(1, dot))))
    if ang <= 170:
        continue
    pairs.append((edge, t0, t1, tris[t0], tris[t1], f0))

print(f'pairs: {len(pairs)}')

# load welds: TOLWELDerge] ->IDX d=D p=(px,py,pz) q=(qx,qy,qz)
welds = []
with open(WELDS) as f:
    for line in f:
        if 'TOLWELD' not in line:
            continue
        try:
            arrow = line.split('->')[1]
            idx = int(arrow.split()[0])
            d = float(line.split('d=')[1].split()[0])
            p = tuple(float(x) for x in line.split('p=(')[1].split(')')[0].split(','))
            q = tuple(float(x) for x in line.split('q=(')[1].split(')')[0].split(','))
            welds.append((idx, d, p, q))
        except (IndexError, ValueError):
            pass
print(f'welds loaded: {len(welds)}')

def match_weld(pt, eps=1e-7):
    hits = []
    for (idx, d, p, q) in welds:
        dq = math.dist(pt, q)
        dp = math.dist(pt, p)
        if dq < eps or dp < eps:
            hits.append((idx, d, p, q, 'q' if dq < eps else 'p'))
    return hits

# analyze each pair
for (edge, t0, t1, tri0, tri1, fid) in sorted(pairs, key=lambda x: (x[5], x[0])):
    e0, e1 = v[edge[0]], v[edge[1]]
    apexes = []
    for tri, t in ((tri0, t0), (tri1, t1)):
        s = set(tri) - set(edge)
        apexes.append(v[s.pop()] if s else e0)
    print(f'\nf{fid} edge{edge} T{t0}{tri0} T{t1}{tri1}')
    for name, pt in (('E0', e0), ('E1', e1), ('A0', apexes[0]), ('A1', apexes[1])):
        hits = match_weld(pt)
        if hits:
            for (idx, d, p, q, which) in hits[:2]:
                print(f'  {name} {tuple(round(x,5) for x in pt)} == weld-{which} idx={idx} d={d:.6f}')
    # distance between apexes and edge
    ex = tuple(e1[i]-e0[i] for i in range(3))
    exl = math.sqrt(sum(x*x for x in ex))
    for name, ap in (('A0', apexes[0]), ('A1', apexes[1])):
        d = tuple(ap[i]-e0[i] for i in range(3))
        cr = (ex[1]*d[2]-ex[2]*d[1], ex[2]*d[0]-ex[0]*d[2], ex[0]*d[1]-ex[1]*d[0])
        h = math.sqrt(sum(x*x for x in cr))/exl
        print(f'  {name} height_above_edge={h:.6f}')
