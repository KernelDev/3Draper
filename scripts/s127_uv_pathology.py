#!/usr/bin/env python3
"""s127 п.1b: UV polygon pathology analysis for brep10212 faces.

For a Cylinder param (u, v): u = angle (period 2π), v = axis position.
Pathologies to measure on the OUTER ring (and holes):
  1. u-range: full 2π wrap? seam-crossing jumps (du > π between consecutive)?
  2. self-intersections of the ring as 2D polyline (in u with wrap and in raw u)
  3. v-range
  4. duplicate/coinincident points
  5. signed area (orientation) of outer ring and each hole
  6. spikes: edges hugely longer than median
"""
import math, sys
from collections import defaultdict

def load_uv(path):
    outer, holes = [], defaultdict(list)
    for l in open(path):
        if l.startswith("#"):
            continue
        parts = l.split()
        if not parts:
            continue
        if parts[0] == "o":
            outer.append((float(parts[1]), float(parts[2])))
        elif parts[0].startswith("h"):
            holes[int(parts[0][1:])].append((float(parts[1]), float(parts[2])))
    return outer, [holes[k] for k in sorted(holes)]

def seg_int(p1, p2, p3, p4):
    """proper segment intersection test (excluding shared endpoints)"""
    d1x, d1y = p2[0]-p1[0], p2[1]-p1[1]
    d2x, d2y = p4[0]-p3[0], p4[1]-p3[1]
    den = d1x*d2y - d1y*d2x
    if abs(den) < 1e-15:
        return False
    t = ((p3[0]-p1[0])*d2y - (p3[1]-p1[1])*d2x) / den
    s = ((p3[0]-p1[0])*d1y - (p3[1]-p1[1])*d1x) / den
    eps = 1e-12
    return (eps < t < 1-eps) and (eps < s < 1-eps)

def analyze(path, name):
    outer, holes = load_uv(path)
    print(f"== {name} ({path.split('/')[-1]}): outer={len(outer)} holes={[len(h) for h in holes]}")
    us = [p[0] for p in outer]; vs = [p[1] for p in outer]
    print(f"   outer u: [{min(us):.4f}, {max(us):.4f}] span={max(us)-min(us):.4f} "
          f"(2pi={2*math.pi:.4f})  v: [{min(vs):.4f}, {max(vs):.4f}] span={max(vs)-min(vs):.4f}")
    # consecutive du (wrapped and raw)
    jumps, raw_du = [], []
    n = len(outer)
    for i in range(n):
        j = (i+1) % n
        du = outer[j][0] - outer[i][0]
        raw_du.append(abs(du))
        wdu = (du + math.pi) % (2*math.pi) - math.pi
        if abs(wdu) > math.pi/2:
            jumps.append((i, outer[i], outer[j], wdu))
    med = sorted(raw_du)[len(raw_du)//2]
    big = [d for d in raw_du if d > max(3*med, 0.5)]
    print(f"   consecutive |du|: median={med:.5f}, >90deg wrap-jumps={len(jumps)}, >3*med={len(big)}")
    for i, a, b, wdu in jumps[:4]:
        print(f"      jump @{i}: ({a[0]:.3f},{a[1]:.3f}) -> ({b[0]:.3f},{b[1]:.3f}) du={wdu:.3f}")
    # self-intersection count (raw u space)
    P = outer
    ints = 0
    samples = []
    for i in range(n):
        a1, a2 = P[i], P[(i+1) % n]
        for k in range(i+1, n):
            if k == i or (k+1) % n == i or k == (i+1) % n or (k+1) % n == (i+1) % n:
                continue
            b1, b2 = P[k], P[(k+1) % n]
            if seg_int(a1, a2, b1, b2):
                ints += 1
                if len(samples) < 4:
                    samples.append((i, k, a1, a2, b1, b2))
    print(f"   self-intersections (raw u): {ints}")
    for i, k, a1, a2, b1, b2 in samples:
        print(f"      edge {i}({a1[0]:.2f},{a1[1]:.2f})->({a2[0]:.2f},{a2[1]:.2f}) X "
              f"edge {k}({b1[0]:.2f},{b1[1]:.2f})->({b2[0]:.2f},{b2[1]:.2f})")
    # signed area outer
    A = 0.0
    for i in range(n):
        j = (i+1) % n
        A += P[i][0]*P[j][1] - P[j][0]*P[i][1]
    print(f"   outer signed area (raw u): {A/2:.4f}  ({'CCW' if A>0 else 'CW'})")
    for hi, h in enumerate(holes):
        Ah = 0.0
        m = len(h)
        for i in range(m):
            j = (i+1) % m
            Ah += h[i][0]*h[j][1] - h[j][0]*h[i][1]
        hus = [p[0] for p in h]
        hvs = [p[1] for p in h]
        print(f"   hole{hi}: n={m} u:[{min(hus):.3f},{max(hus):.3f}] v:[{min(hvs):.3f},{max(hvs):.3f}] "
              f"area={Ah/2:.4f} ({'CCW' if Ah>0 else 'CW'})")
    # duplicate points
    seen = defaultdict(int)
    for p in outer:
        seen[(round(p[0], 6), round(p[1], 6))] += 1
    dups = {k: v for k, v in seen.items() if v > 1}
    if dups:
        print(f"   DUPLICATE outer points: {len(dups)} e.g. {list(dups.items())[:3]}")
    # hole-in-hole / hole-outside-outer containment (bbox test)
    for hi, h in enumerate(holes):
        inside = all(min(us)-1e-9 <= p[0] <= max(us)+1e-9 and
                     min(vs)-1e-9 <= p[1] <= max(vs)+1e-9 for p in h)
        if not inside:
            print(f"   hole{hi} NOT inside outer bbox!")
    return outer, holes

if __name__ == "__main__":
    base = "forensics/s127/uv10212"
    analyze(f"{base}/uvpoly_1721.txt", "brep10212 f2 pass1")
    analyze(f"{base}/uvpoly_1728.txt", "brep10212 f19 pass1")
