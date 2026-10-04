#!/usr/bin/env python3
"""s73: map LUNE fold triangles to band/region structure.

Parses DRAPPER_LUNE_DUMP2 "[LUNE fold LABEL] ang=.. t1=(u,v)#i .. t2=(u,v)#i"
lines for ONE face (first pb round only), plus the matching [LUNE state]
block, and reports WHERE the folds live:
  - v-band (between which connector levels)
  - u-region (left flank / mid corridor / right flank / left tab)
  - shared-edge geometry (length, orientation)
"""
import re
import sys
from collections import Counter

err_path = sys.argv[1]
face = sys.argv[2]          # e.g. "brep32629_f49"
max_round = int(sys.argv[3]) if len(sys.argv) > 3 else 1

fold_re = re.compile(
    r"\[LUNE fold ([^\]]+)\] ang=([\d.]+) "
    r"t1=\(([-\d.e]+),([-\d.e]+)\)#(\d+)\(([-\d.e]+),([-\d.e]+)\)#(\d+)\(([-\d.e]+),([-\d.e]+)\)#(\d+) "
    r"t2=\(([-\d.e]+),([-\d.e]+)\)#(\d+)\(([-\d.e]+),([-\d.e]+)\)#(\d+)\(([-\d.e]+),([-\d.e]+)\)#(\d+)"
)
state_re = re.compile(r"\[LUNE state ([^\]]+)\] n=(\d+) k_bands=(\d+) anchors_l=\[([\d, ]+)\] anchors_r=\[([\d, ]+)\]")

folds = []          # (round, ang, pts[6])
state = None
round_seen = 0
prev_label = None
with open(err_path) as f:
    for line in f:
        m = fold_re.match(line)
        if m:
            lab = m.group(1)
            if face not in lab:
                continue
            if prev_label is not None and lab != prev_label:
                pass
            vals = [float(x) for x in m.groups()[1:2]]
            pts = []
            g = m.groups()
            for i in range(6):
                u = float(g[2 + 3 * i])
                v = float(g[3 + 3 * i])
                idx = int(g[4 + 3 * i])
                pts.append((u, v, idx))
            folds.append((vals[0], pts))
            continue
        m = state_re.match(line)
        if m and face in m.group(1):
            if state is None:
                state = m

if not folds:
    print(f"no folds for {face}")
    sys.exit(0)

print(f"face={face} total fold lines={len(folds)}")
if state:
    al = [int(x) for x in state.group(4).split(',')]
    ar = [int(x) for x in state.group(5).split(',')]
    print(f"state: n={state.group(2)} k_bands={state.group(3)} anchors_l={al} anchors_r={ar}")

# group fold pairs by shared edge = the common index pair of t1/t2
def shared_edge(p1, p2):
    s1 = {(p1[i][2], p1[(i+1)%3][2]) for i in range(3)}
    s2 = {(p2[i][2], p2[(i+1)%3][2]) for i in range(3)}
    s1c = {frozenset(e) for e in s1}
    s2c = {frozenset(e) for e in s2}
    common = s1c & s2c
    return list(common)[0] if common else None

edges = Counter()
edge_geo = {}
for ang, pts in folds:
    t1, t2 = pts[:3], pts[3:]
    e = shared_edge(t1, t2)
    if e is None:
        continue
    key = tuple(sorted(e))
    edges[key] += 1
    # geometry of the shared edge: find the two points
    pts_by_idx = {p[2]: (p[0], p[1]) for p in pts}
    if key[0] in pts_by_idx and key[1] in pts_by_idx:
        (u0, v0), (u1, v1) = pts_by_idx[key[0]], pts_by_idx[key[1]]
        edge_geo[key] = (u0, v0, u1, v1, ang)

print(f"unique fold edges: {len(edges)}")
# cluster by location: v-band and u-region
regions = Counter()
for key, cnt in edges.items():
    g = edge_geo.get(key)
    if not g:
        continue
    u0, v0, u1, v1, ang = g
    umid, vmid = (u0+u1)/2, (v0+v1)/2
    du, dv = abs(u1-u0), abs(v1-v0)
    if vmid < 0.02:
        band = "band0 (tab/bottom, v<0.02)"
    elif vmid < 0.24:
        band = "band1 (0.02..0.24)"
    elif vmid < 0.49:
        band = "band2 (0.24..0.49)"
    elif vmid < 0.74:
        band = "band3 (0.49..0.74)"
    else:
        band = "band4 (0.74..1.0)"
    if umid < 0.14:
        reg = "u<0.14 (tab/left of step)"
    elif umid < 0.37:
        reg = "left flank (0.14..0.37)"
    elif umid < 0.63:
        reg = "mid (0.37..0.63)"
    elif umid < 0.85:
        reg = "right flank (0.63..0.85)"
    else:
        reg = "far right (>0.85)"
    orient = "V-edge" if dv > du else "U-edge"
    regions[(band, reg, orient)] += cnt

print("\nband x region x edge-orientation (fold counts):")
for (band, reg, orient), cnt in sorted(regions.items(), key=lambda kv: -kv[1]):
    print(f"  {cnt:5d}  {band:28s} {reg:28s} {orient}")

print("\ntop-20 fold edges by repeat count:")
for key, cnt in edges.most_common(20):
    g = edge_geo.get(key)
    if g:
        u0, v0, u1, v1, ang = g
        print(f"  x{cnt:4d} edge #{key[0]}-#{key[1]}: ({u0:.3f},{v0:.3f})-({u1:.3f},{v1:.3f}) du={abs(u1-u0):.3f} dv={abs(v1-v0):.3f}")

# v-profile of fold midpoints
vs = Counter()
for key, cnt in edges.items():
    g = edge_geo.get(key)
    if g:
        u0, v0, u1, v1, ang = g
        b = int(((v0+v1)/2) * 20)
        vs[b] += cnt
print("\nv-midpoint histogram (bins of 0.05 v):")
for b in sorted(vs):
    print(f"  v {b*0.05:5.2f}-{b*0.05+0.05:5.2f}: {vs[b]}")
