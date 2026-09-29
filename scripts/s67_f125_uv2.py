#!/usr/bin/env python3
"""s67 diag 11b: f125 UV polygon with REAL knots; full polygon
self-intersection check + slit endpoint localization.

Polygon: LINE u=180 (v -0.3123..-1.2), ARC v=-1.2 (u 90..180),
LINE u=90 (v -1.2..-0.1475), SPLINE rim (90..180, v varies).
The slit in the mesh serpents through the domain — where does the
polygon nearly touch itself?
"""
import math
import re

STP = "/home/z/my-project/3Draper/test/drill_top.stp"


def parse():
    data = open(STP, errors="replace").read()
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*?)\)\s*;",
                         data, re.S):
        ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents


def refs(a):
    return [int(x) for x in re.findall(r"#(\d+)", a)]


def nums(a):
    return [float(x) for x in
            re.findall(r"-?\d+\.?\d*(?:E[-+]?\d+)?", a)]


def pt3(ents, ref):
    return tuple(nums(ents[ref][1])[:3])


def spline_uv_samples(ents, cref, uv, n=100):
    t, a = ents[cref]
    deg = int(nums(a.split(",", 2)[1])[0])
    cps = [pt3(ents, r) for r in refs(a)
           if r in ents and ents[r][0] == "CARTESIAN_POINT"]
    # multiplicities + knots: two trailing parenthesized number lists
    lists = re.findall(r"\(([\d.E,\-+\s]+)\)", a)
    mult = [int(x) for x in lists[-2].split(",")]
    kn = [float(x) for x in lists[-1].split(",")]
    knots = []
    for m_, k in zip(mult, kn):
        knots += [k] * m_
    assert len(knots) == len(cps) + deg + 1, \
        (len(knots), len(cps), deg)
    pts = []
    u0, u1 = knots[deg], knots[len(cps)]
    for i in range(n + 1):
        u = u0 + (u1 - u0) * i / n
        span = deg
        while span < len(cps) - 1 and knots[span + 1] <= u:
            span += 1
        d = list(cps[span - deg:span + 1])
        for r in range(1, deg + 1):
            for j in range(len(d) - 1):
                k0, k1 = knots[span - deg + j + r], knots[span + 1 + j]
                a_ = 0.0 if k1 <= k0 else (u - k0) / (k1 - k0)
                d[j] = tuple(d[j][q] * (1 - a_) + d[j + 1][q] * a_
                             for q in range(3))
            d.pop()
        pts.append(uv(d[0]))
    return pts, cps, deg


def seg_int(p1, p2, p3, p4):
    """2D segment intersection, returns t params or None."""
    d1 = (p2[0] - p1[0], p2[1] - p1[1])
    d2 = (p4[0] - p3[0], p4[1] - p3[1])
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-15:
        return None
    t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / den
    s = ((p3[0] - p1[0]) * d1[1] - (p3[1] - p1[1]) * d1[0]) / den
    if 0 <= t <= 1 and 0 <= s <= 1:
        return t, s
    return None


def main():
    ents = parse()
    pos = pt3(ents, 42118)
    d = pt3(ents, 42119)
    dl = math.sqrt(sum(x * x for x in d))
    d = tuple(x / dl for x in d)

    def uv(p):
        rel = tuple(p[k] - pos[k] for k in range(3))
        v = sum(rel[k] * d[k] for k in range(3))
        rad = tuple(rel[k] - v * d[k] for k in range(3))
        return (math.atan2(rad[1], rad[0]), v)

    # polygon: lineA(180), arc(v=-1.2), lineB(90), spline rim
    poly = []
    # line A: (180°, -0.3123) -> (180°, -1.2)
    poly.append((math.pi, -0.3123))
    poly.append((math.pi, -1.2))
    # arc: u 180 -> 90 at v=-1.2
    for i in range(1, 9):
        u = math.pi - (math.pi / 2) * i / 8
        poly.append((u, -1.2))
    # line B: (90°, -1.2) -> (90°, -0.1475)
    poly.append((math.pi / 2, -0.1475))
    # spline rim: 90° -> 180° (back to start)
    sp, cps, deg = spline_uv_samples(ents, 35059, uv, 100)
    # unwrap u along the spline
    us = [p[0] for p in sp]
    for i in range(1, len(us)):
        while us[i] - us[i-1] > math.pi:
            us[i] -= 2 * math.pi
        while us[i] - us[i-1] < -math.pi:
            us[i] += 2 * math.pi
    sp = [(us[i], sp[i][1]) for i in range(len(sp))]
    # orient: start at 90° end
    if sp[0][0] > sp[-1][0]:
        sp = sp[::-1]
    poly += sp[:-1]
    print(f"polygon: {len(poly)} verts")
    vs_ = [p[1] for p in sp]
    us_ = [math.degrees(p[0]) for p in sp]
    print(f"spline rim: u[{min(us_):.2f}°,{max(us_):.2f}°] "
          f"v[{min(vs_):.4f},{max(vs_):.4f}]")

    # self-intersection check (skip adjacent segments)
    n = len(poly)
    hits = []
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            b1, b2 = poly[j], poly[(j + 1) % n]
            r = seg_int(a1, a2, b1, b2)
            if r:
                hits.append((i, j, r))
    print(f"self-intersections: {len(hits)}")
    for h in hits[:10]:
        i, j, (t, s) = h
        p = (poly[i][0] + t * (poly[(i+1) % n][0] - poly[i][0]),
             poly[i][1] + t * (poly[(i+1) % n][1] - poly[i][1]))
        print(f"  seg{i} x seg{j} at uv "
              f"({math.degrees(p[0]):.2f}°, {p[1]:.4f})")

    # near-touch: min distance between non-adjacent segments
    best = (1e9, None)
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            b1, b2 = poly[j], poly[(j + 1) % n]
            # coarse: point-to-point then refine
            for (p, q) in ((a1, b1), (a1, b2), (a2, b1), (a2, b2)):
                dd = math.dist(p, q)
                if dd < best[0]:
                    best = (dd, (i, j, p, q))
    print(f"min vertex-pair dist non-adjacent: {best[0]:.6f} "
          f"(seg{best[1][0]} x seg{best[1][1]})")


if __name__ == "__main__":
    main()
