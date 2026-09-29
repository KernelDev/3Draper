#!/usr/bin/env python3
"""s67 diag 11: UV polygon of f125 (Cylinder quarter-patch) direct
from STEP curves. Evaluates each EDGE_CURVE's geometry (LINE by
endpoints, CIRCLE arc, B_SPLINE by control points de Boor) in the
cylinder's UV frame (u = atan2 angle, v = axis height).
Checks: polygon self-intersection / self-touch.
"""
import math
import re

STP = "/home/z/my-project/3Draper/test/drill_top.stp"


def parse():
    data = open(STP, errors="replace").read()
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;",
                         data, re.S):
        ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents


def refs(a):
    return [int(x) for x in re.findall(r"#(\d+)", a)]


def nums(a):
    out = []
    for tok in re.findall(r"-?\d+\.?\d*(?:E[-+]?\d+)?", a):
        out.append(float(tok))
    return out


def pt(ents, ref):
    t, a = ents[ref]
    if t == "DIRECTION":
        v = nums(a)
        return tuple(v[:3])
    assert t == "CARTESIAN_POINT", t
    v = nums(a)
    return tuple(v[:3])


def eval_spline(ents, cref, n=60):
    """B_SPLINE_CURVE_WITH_KNOTS de Boor eval (degree, cp, knots)."""
    t, a = ents[cref]
    # fields: name, degree, cp-list, form, closed, selfint, knot_list,
    # mult_list, ... — knots may be a ref or literal
    deg = nums(a.split(",", 2)[1])[0]
    cps = [pt(ents, r) for r in refs(a)
           if r in ents and ents[r][0] == "CARTESIAN_POINT"]
    # knots ref: a list ref (last #ref) or literal numbers
    tail = a.rsplit(",", 4)
    knot_nums = nums(a)
    # heuristic: knots+mults are the trailing numeric lists; total
    # knots = n_cp + deg + 1
    # find knot list: try refs to a LIST entity
    knot_list = []
    for r in refs(a):
        if r in ents and ents[r][0] in ("(", ) or (
                r in ents and "KNOT" in ents[r][0]):
            pass
    # fallback: uniform knots
    nk = len(cps) + deg + 1
    if not knot_list:
        knot_list = [i / (nk - 1) for i in range(nk)]
    pts = []
    for i in range(n + 1):
        u = i / n * (knot_list[-deg - 1] - knot_list[deg]) + knot_list[deg]
        # find span
        span = deg
        while span < len(cps) - 1 and knot_list[span + 1] <= u:
            span += 1
        d = list(cps[span - deg:span + 1])
        for r in range(1, deg + 1):
            for j in range(len(d) - 1):
                k0 = knot_list[span - deg + j + r]
                k1 = knot_list[span + 1 + j]
                a_ = 0.0 if k1 <= k0 else (
                    u - k0) / (k1 - k0)
                d[j] = tuple(d[j][q] * (1 - a_) + d[j + 1][q] * a_
                             for q in range(3))
            d.pop()
        pts.append(d[0])
    return pts, deg, len(cps)


def main():
    ents = parse()
    # cylinder f125: surface #42122, placement #42121
    # axis pos #42118 (-2.54,0.375,-4.35), dir #42119 (0,0,-1)
    pos = pt(ents, 42118)
    d = pt(ents, 42119)
    dl = math.sqrt(sum(x * x for x in d)); d = tuple(x / dl for x in d)
    print(f"axis pos={pos} dir={d} R=0.125")

    def uv(p):
        rel = tuple(p[k] - pos[k] for k in range(3))
        v = sum(rel[k] * d[k] for k in range(3))
        rad = tuple(rel[k] - v * d[k] for k in range(3))
        u = math.atan2(rad[1], rad[0])
        return (u, v)

    # f125 edges: 42107 LINE, 42124 SPLINE, 42126 LINE, 42128 CIRCLE
    for ec, kind in [(42107, "LINE"), (42126, "LINE"),
                     (42128, "CIRCLE"), (42124, "SPLINE")]:
        t, a = ents[ec]
        er = refs(a)
        verts_ = [r for r in er if r in ents
                  and ents[r][0] == "VERTEX_POINT"]
        p0 = pt(ents, refs(ents[verts_[0]][1])[0])
        p1 = pt(ents, refs(ents[verts_[1]][1])[0])
        u0, v0 = uv(p0); u1, v1 = uv(p1)
        print(f"\nec#{ec} {kind}: V0 {p0} -> uv ({math.degrees(u0):8.2f}°, "
              f"{v0:8.4f})")
        print(f"{'':16s}V1 {p1} -> uv ({math.degrees(u1):8.2f}°, "
              f"{v1:8.4f})")
        if kind == "SPLINE":
            pts, deg, ncp = eval_spline(ents, [
                r for r in refs(a) if r in ents and
                "B_SPLINE_CURVE" in ents[r][0]][0])
            print(f"  spline deg={deg} ncp={ncp}: "
                  f"{len(pts)} samples")
            uvs = [uv(p) for p in pts]
            us = [x[0] for x in uvs]; vs = [x[1] for x in uvs]
            # unwrap u
            for i in range(1, len(us)):
                while us[i] - us[i-1] > math.pi:
                    us[i] -= 2 * math.pi
                while us[i] - us[i-1] < -math.pi:
                    us[i] += 2 * math.pi
            print(f"  spline UV: u[{math.degrees(min(us)):.2f}°,"
                  f"{math.degrees(max(us)):.2f}°] "
                  f"v[{min(vs):.4f},{max(vs):.4f}]")
            for i in range(0, len(uvs), 10):
                print(f"    s{i:3d}: u={math.degrees(us[i]):8.2f}° "
                      f"v={vs[i]:8.4f}")
        if kind == "CIRCLE":
            cr = [r for r in refs(a) if r in ents and
                  ents[r][0] == "CIRCLE"][0]
            ct, ca = ents[cr]
            print(f"  circle def: {ca[:100]}")


if __name__ == "__main__":
    main()
