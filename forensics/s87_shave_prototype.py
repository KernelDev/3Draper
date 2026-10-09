#!/usr/bin/env python3
"""s87: offline prototype of the FAN_GUARD degree-shaving on the
synthetic arc-trapezoid ring — mirrors the Rust logic (sorted star,
convexity, no-dup-edge, degree caps, 2 flips) to prove the test
geometry exercises the shaving path before writing the Rust test."""
from collections import defaultdict
import sys

FAN_MAX = int(sys.argv[1]) if len(sys.argv) > 1 else 16
SRC = sys.argv[2] if len(sys.argv) > 2 else "/tmp/s87_replay2/s87_arc_ring_altC.txt"

pts = []
tris = []
for line in open(SRC):
    if line.startswith("p "):
        _, u, v = line.split()
        pts.append((float(u), float(v)))
    elif line.startswith("t "):
        _, a, b, c = line.split()
        tris.append((int(a), int(b), int(c)))
m = len(pts)
deg = [0] * m
for t in tris:
    for v in t:
        deg[v] += 1
vmax = max(range(m), key=lambda i: deg[i])
need = deg[vmax] - FAN_MAX + 1
print(f"m={m} v*={vmax} alt_deg={deg[vmax]} fan={FAN_MAX} need={need}")
print("top-6 degrees:", sorted(deg, reverse=True)[:6])

edge_tris = defaultdict(list)
for ti, t in enumerate(tris):
    for k in range(3):
        a, b = t[k], t[(k + 1) % 3]
        if a != b:
            edge_tris[(min(a, b), max(a, b))].append(ti)


def cr(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def is_rim(a, b):
    lo, hi = min(a, b), max(a, b)
    return hi == lo + 1 or (lo == 0 and hi == m - 1)


remaining = need
flips = 0
for _ in range(need + 2):
    if remaining == 0:
        break
    star = sorted(
        (e, tis[0], tis[1])
        for e, tis in edge_tris.items()
        if len(tis) == 2 and vmax in e and not is_rim(*e)
    )
    progressed = False
    for (e, ti1, ti2) in star:
        a, b = e
        t1, t2 = tris[ti1], tris[ti2]
        c = next(x for x in t1 if x not in (a, b))
        d = next(x for x in t2 if x not in (a, b))
        if edge_tris.get((min(c, d), max(c, d))):
            continue  # flip edge already exists
        s1 = cr(pts[a], pts[c], pts[d])
        s2 = cr(pts[b], pts[c], pts[d])
        s3 = cr(pts[c], pts[a], pts[b])
        s4 = cr(pts[d], pts[a], pts[b])
        if not (s1 * s2 < -1e-12 and s3 * s4 < -1e-12):
            continue
        if deg[c] + 1 >= FAN_MAX or deg[d] + 1 >= FAN_MAX:
            continue
        tris[ti1] = (c, a, d)
        tris[ti2] = (c, d, b)
        # orient replacements to the partners' stored signs (s81)
        def sgn(t):
            return cr(pts[t[0]], pts[t[1]], pts[t[2]])
        w1, w2 = sgn(t1), sgn(t2)
        if sgn(tris[ti1]) * w1 < 0:
            tris[ti1] = (tris[ti1][0], tris[ti1][2], tris[ti1][1])
        if sgn(tris[ti2]) * w2 < 0:
            tris[ti2] = (tris[ti2][0], tris[ti2][2], tris[ti2][1])
        t1_new, t2_new = tris[ti1], tris[ti2]
        for t in (t1, t2):
            for k in range(3):
                x, y = t[k], t[(k + 1) % 3]
                if x != y:
                    v = edge_tris[(min(x, y), max(x, y))]
                    if ti1 in v:
                        v.remove(ti1)
                    if ti2 in v:
                        v.remove(ti2)
                deg[x] -= 1
        for t, slot in ((t1_new, ti1), (t2_new, ti2)):
            for k in range(3):
                x, y = t[k], t[(k + 1) % 3]
                if x != y:
                    edge_tris[(min(x, y), max(x, y))].append(slot)
                deg[x] += 1
        flips += 1
        remaining -= 1
        progressed = True
        break
    if not progressed:
        break

final_max = max(deg)
print(f"flips={flips} remaining={remaining} final_max={final_max} (need < {FAN_MAX})")


def area2(ts):
    return sum(cr(pts[t[0]], pts[t[1]], pts[t[2]]) for t in ts)


ring2 = sum(
    pts[i][0] * pts[(i + 1) % m][1] - pts[(i + 1) % m][0] * pts[i][1]
    for i in range(m)
)
print(f"area_rel={abs(area2(tris) - ring2) / abs(ring2):.2e}")
es = set()
for t in tris:
    for k in range(3):
        a, b = t[k], t[(k + 1) % 3]
        es.add((min(a, b), max(a, b)))
missing = sum(
    1
    for i in range(m)
    if (min(i, (i + 1) % m), max(i, (i + 1) % m)) not in es
)
unused = sum(1 for i in range(m) if deg[i] == 0)
print(f"rim_missing={missing} unused={unused}")
print("VERDICT:", "SHAVABLE" if remaining == 0 and final_max < FAN_MAX and missing == 0 and unused == 0 else "NOT SHAVABLE")
