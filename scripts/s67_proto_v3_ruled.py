#!/usr/bin/env python3
"""s67 PROTOTYPE v3: RULED BAND for f125 — no Steiner, no chain.

f125-class cylinder patches: ruled in v (axis direction). The mesh =
monotone two-pointer band (s65 machinery) between the two cached
u-monotone rim chains (arc bottom, spline top) + the two side lines
as band ends. ~56 triangles, watertight by construction.

Checks: edge usage, 3D area vs true, min angle, fold pairs, chord
error vs max_deviation.
"""
import math
import re
from collections import defaultdict

R = 0.125
POS = (-2.54, 0.375, -4.349967143371)
AX = (0.0, 0.0, -1.0)
XDIR = (-1.0, 0.0, 0.0)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


YDIR = cross(AX, XDIR)


def uv_to_3d(u_deg, v):
    u = math.radians(u_deg)
    d = (math.cos(u)*XDIR[0] + math.sin(u)*YDIR[0],
         math.cos(u)*XDIR[1] + math.sin(u)*YDIR[1],
         math.cos(u)*XDIR[2] + math.sin(u)*YDIR[2])
    return (POS[0] + R*d[0] + v*AX[0], POS[1] + R*d[1] + v*AX[1],
            POS[2] + R*d[2] + v*AX[2])


def main():
    data = open("/home/z/my-project/3Draper/test/drill_top.stp",
                errors="replace").read()
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*?)\)\s*;",
                         data, re.S):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    def nums(a):
        return [float(x) for x in
                re.findall(r"-?\d+\.?\d*(?:E[-+]?\d+)?", a)]

    def refs(a):
        return [int(x) for x in re.findall(r"#(\d+)", a)]

    def pt3(r):
        return tuple(nums(ents[r][1])[:3])

    a_ = ents[35059][1]
    deg = int(nums(a_.split(",", 2)[1])[0])
    cps = [pt3(r) for r in refs(a_)
           if r in ents and ents[r][0] == "CARTESIAN_POINT"]
    lists = re.findall(r"\(([\d.E,\-+\s]+)\)", a_)
    mult = [int(x) for x in lists[-2].split(",")]
    kn = [float(x) for x in lists[-1].split(",")]
    knots = []
    for m_, k in zip(mult, kn):
        knots += [k] * m_
    u0, u1 = knots[deg], knots[len(cps)]

    def spline3(t):
        u = u0 + (u1 - u0) * t
        span = deg
        while span < len(cps) - 1 and knots[span + 1] <= u:
            span += 1
        d = list(cps[span - deg:span + 1])
        for r in range(1, deg + 1):
            for j in range(len(d) - 1):
                k0, k1 = knots[span - deg + j + r], knots[span + 1 + j]
                a2 = 0.0 if k1 <= k0 else (u - k0) / (k1 - k0)
                d[j] = tuple(d[j][q]*(1-a2) + d[j+1][q]*a2 for q in range(3))
            d.pop()
        return d[0]

    def uv3(p):
        rel = (p[0]-POS[0], p[1]-POS[1], p[2]-POS[2])
        v = sum(rel[k]*AX[k] for k in range(3))
        rx = sum(rel[k]*XDIR[k] for k in range(3))
        ry = sum(rel[k]*YDIR[k] for k in range(3))
        return (math.degrees(math.atan2(ry, rx)), v)

    # rim chains (cached discretization reproduced)
    ARC_N = 17
    bottom = [(90.0*k/(ARC_N-1), -1.2) for k in range(ARC_N)]
    top = [uv3(spline3(1.0 - t/40.0)) for t in range(41)]
    top = [q for k, q in enumerate(top)
           if k == 0 or math.hypot((q[0]-top[k-1][0])*R*math.pi/180,
                                   q[1]-top[k-1][1]) > 1e-9]
    top.reverse()   # both chains u-ascending: bottom 0->90, top 0->90
    print(f"bottom {len(bottom)} pts, top {len(top)} pts")

    # two-pointer monotone band by u (s65 machinery)
    tris = []
    i = j = 0
    nb, nt = len(bottom), len(top)
    while i + 1 < nb or j + 1 < nt:
        if j + 1 >= nt:
            tris.append((("B", i), ("B", i+1), ("T", j))); i += 1
        elif i + 1 >= nb:
            tris.append((("T", j), ("T", j+1), ("B", i))); j += 1
        elif bottom[i+1][0] - bottom[i][0] <= top[j+1][0] - top[j][0]:
            tris.append((("B", i), ("B", i+1), ("T", j))); i += 1
        else:
            tris.append((("T", j), ("T", j+1), ("B", i))); j += 1
    print(f"band triangles: {len(tris)}")

    def pt_of(tag, k):
        return bottom[k] if tag == "B" else top[k]

    # VERIFY 1: edge usage — rim edges 1-used, others 2-used
    edge_use = defaultdict(int)
    for t in tris:
        ks = [(x[0], x[1]) for x in t]
        for x, y in ((ks[0], ks[1]), (ks[1], ks[2]), (ks[2], ks[0])):
            edge_use[(x, y) if x > y else (y, x)] += 1
    def canon(e):
        return e if e[0] > e[1] else (e[1], e[0])
    rim_edges = {canon((("B", k), ("B", k+1))) for k in range(nb-1)}
    rim_edges |= {canon((("T", k), ("T", k+1))) for k in range(nt-1)}
    # side lines: (B0,T0) and (B_last,T_last) are rim edges too
    rim_edges |= {canon((("B", 0), ("T", 0)))}
    rim_edges |= {canon((("B", nb-1), ("T", nt-1)))}
    bad = []
    for e, n in edge_use.items():
        if e in rim_edges:
            if n != 1:
                bad.append((e, n, "rim!=1"))
        else:
            if n != 2:
                bad.append((e, n, "interior!=2"))
    print(f"edge violations: {len(bad)}")
    for b in bad[:6]:
        print("   ", b)

    # VERIFY 2: 3D area vs true integral
    def area3(t):
        pts = [uv_to_3d(*pt_of(*x)) for x in t]
        a, b, c = pts
        ab = tuple(b[k]-a[k] for k in range(3))
        ac = tuple(c[k]-a[k] for k in range(3))
        cr = cross(ab, ac)
        return 0.5*math.sqrt(sum(x*x for x in cr))
    tot = sum(area3(t) for t in tris)
    # true area: integrate v_span(u) * R du
    true_area = 0.0
    for k in range(len(top)-1):
        u_a, u_b = top[k][0], top[k+1][0]
        # trapezoid of (rim_v - (-1.2)) over u
        h_a = top[k][1] + 1.2
        h_b = top[k+1][1] + 1.2
        true_area += 0.5*(h_a + h_b) * abs(u_b - u_a) * R * math.pi/180
    print(f"3D area: band {tot:.6f} vs true {true_area:.6f} "
          f"({100*tot/true_area:.2f}%)")

    # VERIFY 3: min angle + fold pairs
    def normal3(t):
        pts = [uv_to_3d(*pt_of(*x)) for x in t]
        a, b, c = pts
        ab = tuple(b[k]-a[k] for k in range(3))
        ac = tuple(c[k]-a[k] for k in range(3))
        cr = cross(ab, ac)
        l = math.sqrt(sum(x*x for x in cr))
        return None if l < 1e-15 else tuple(x/l for x in cr)
    worst = 90.0
    for t in tris:
        pts = [uv_to_3d(*pt_of(*x)) for x in t]
        for q in range(3):
            a, b, c = pts[q], pts[(q+1) % 3], pts[(q+2) % 3]
            v1 = tuple(b[k]-a[k] for k in range(3))
            v2 = tuple(c[k]-a[k] for k in range(3))
            n1 = math.sqrt(sum(x*x for x in v1))
            n2 = math.sqrt(sum(x*x for x in v2))
            if n1 < 1e-12 or n2 < 1e-12:
                worst = 0.0
                continue
            cs = sum(v1[k]*v2[k] for k in range(3))/(n1*n2)
            worst = min(worst, math.degrees(math.acos(
                max(-1, min(1, cs)))))
    print(f"min 3D angle: {worst:.2f} deg")
    edge_tris = defaultdict(list)
    for ti, t in enumerate(tris):
        ks = [(x[0], x[1]) for x in t]
        for x, y in ((ks[0], ks[1]), (ks[1], ks[2]), (ks[2], ks[0])):
            edge_tris[(x, y) if x > y else (y, x)].append(ti)
    folds = 0
    for e, ts_ in edge_tris.items():
        if len(ts_) != 2:
            continue
        n1, n2 = normal3(tris[ts_[0]]), normal3(tris[ts_[1]])
        if n1 and n2:
            ang = math.degrees(math.acos(max(-1, min(1,
                sum(n1[k]*n2[k] for k in range(3))))))
            if ang > 170:
                folds += 1
    print(f"same-face fold pairs: {folds}")

    # VERIFY 4: chord error in u (arc step)
    du = 90.0/(ARC_N-1)
    err = R * (1 - math.cos(math.radians(du/2)))
    print(f"u chord error (arc step {du:.1f} deg): {err:.6f} "
          f"vs max_deviation 0.01")

    # VERIFY 5: all rim pts used
    used = set()
    for t in tris:
        used.update((x[0], x[1]) for x in t)
    missing = ([("B", k) for k in range(nb) if ("B", k) not in used]
               + [("T", k) for k in range(nt) if ("T", k) not in used])
    print(f"unused rim pts: {len(missing)}")


if __name__ == "__main__":
    main()
