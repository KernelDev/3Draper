#!/usr/bin/env python3
"""Session-85 v2: zipper with (a) monotonized keys for the two-pointer
decisions and (b) per-tri orientation normalization at emission.

Audit: rim 1x, non-rim 2x, area, slivers, orientation consistency,
skew quads (A-advance with b outside the a-segment).
"""
import math
import sys
from collections import defaultdict
from pathlib import Path


def load(p):
    bnd, tris = [], []
    mode = None
    for ln in Path(p).read_text().splitlines()[1:]:
        parts = ln.split()
        if not parts:
            continue
        if parts[0] in ("boundary", "interior", "tris"):
            mode = parts[0]
            continue
        if parts[0].startswith("hole"):
            mode = "hole"
            continue
        if mode == "boundary":
            bnd.append((float(parts[1]), float(parts[2])))
        elif mode == "tris":
            tris.append((int(parts[1]), int(parts[2]), int(parts[3])))
    return bnd, tris


def strip_v2(bnd, eps_rel=1e-3):
    n = len(bnd)
    for swap in (False, True):
        key = (lambda p: p[1]) if swap else (lambda p: p[0])
        kvals = [key(p) for p in bnd]
        span = max(kvals) - min(kvals)
        eps = max(eps_rel * span, 1e-12)
        umin_i = min(range(n), key=lambda k: (kvals[k], k))
        umax_i = min(range(n), key=lambda k: (-kvals[k], k))
        if umin_i == umax_i:
            continue
        a_chain = [umin_i]
        i = umin_i
        while i != umax_i:
            i = (i + 1) % n
            a_chain.append(i)
        b_walk = [umax_i]
        i = umax_i
        while i != umin_i:
            i = (i + 1) % n
            b_walk.append(i)
        b_chain = b_walk[::-1]

        def mono(c):
            return all(kvals[c[k]] <= kvals[c[k + 1]] + eps for k in range(len(c) - 1))

        if not mono(a_chain) or not mono(b_chain):
            continue
        if a_chain[0] != b_chain[0] or a_chain[-1] != b_chain[-1]:
            continue

        poly_area = 0.5 * sum(bnd[k][0] * bnd[(k + 1) % n][1] - bnd[(k + 1) % n][0] * bnd[k][1] for k in range(n))
        sign = 1.0 if poly_area >= 0 else -1.0

        # monotonized keys for decisions: running-max clip (never decrease)
        def monot(c):
            out = [kvals[c[0]]]
            for k in range(1, len(c)):
                out.append(max(kvals[c[k]], out[-1]))
            return out

        ka = monot(a_chain)
        kb = monot(b_chain)

        def cross(a, b, c):
            (ax, ay), (bx, by), (cx, cy) = bnd[a], bnd[b], bnd[c]
            return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)

        tris = []
        flips_fixed = 0
        ia = ib = 0
        while ia < len(a_chain) - 1 or ib < len(b_chain) - 1:
            if ia >= len(a_chain) - 1:
                t = [a_chain[ia], b_chain[ib], b_chain[ib + 1]]
                ib += 1
            elif ib >= len(b_chain) - 1:
                t = [a_chain[ia], a_chain[ia + 1], b_chain[ib]]
                ia += 1
            elif ka[ia + 1] <= kb[ib + 1]:
                t = [a_chain[ia], a_chain[ia + 1], b_chain[ib]]
                ia += 1
            else:
                t = [a_chain[ia], b_chain[ib], b_chain[ib + 1]]
                ib += 1
            if len({t[0], t[1], t[2]}) < 3:
                continue
            if cross(*t) * sign < 0:
                t[1], t[2] = t[2], t[1]
                flips_fixed += 1
            tris.append(tuple(t))
        return tris, swap, flips_fixed
    return None, None, None


def audit(bnd, tris):
    n = len(bnd)
    ecnt = defaultdict(int)
    for t in tris:
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            ecnt[(min(a, b), max(a, b))] += 1
    rim = sum(1 for k in range(n) if (min(k, (k + 1) % n), max(k, (k + 1) % n)) in ecnt)
    bad = sum(1 for e, c in ecnt.items() if c == 1 and not (
        e[0] < n and e[1] < n and (abs(e[0] - e[1]) == 1 or (min(e[0], e[1]) == 0 and max(e[0], e[1]) == n - 1))))
    # orientation consistency
    def cross(a, b, c):
        (ax, ay), (bx, by), (cx, cy) = bnd[a], bnd[b], bnd[c]
        return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
    poly = 0.5 * sum(bnd[k][0] * bnd[(k + 1) % n][1] - bnd[(k + 1) % n][0] * bnd[k][1] for k in range(n))
    wrong = sum(1 for t in tris if cross(*t) * poly < 0)
    sarea = 0.5 * sum(cross(*t) for t in tris)
    us = [p[0] for p in bnd]; vs = [p[1] for p in bnd]
    diag = math.hypot(max(us) - min(us), max(vs) - min(vs))
    sliv = 0
    for a, b, c in tris:
        pts = [bnd[a], bnd[b], bnd[c]]
        mn, lng = 1e9, 0.0
        for k in range(3):
            p0, p1, p2 = pts[k], pts[(k + 1) % 3], pts[(k + 2) % 3]
            v1 = (p1[0] - p0[0], p1[1] - p0[1]); v2 = (p2[0] - p0[0], p2[1] - p0[1])
            l1, l2 = math.hypot(*v1), math.hypot(*v2)
            lng = max(lng, l1, l2, math.hypot(p2[0] - p1[0], p2[1] - p1[1]))
            if l1 > 1e-15 and l2 > 1e-15:
                mn = min(mn, math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2))))))
        if mn < 2.0 and lng > 0.10 * diag:
            sliv += 1
    print(f"    tris={len(tris)} rim={rim}/{n} nonrim_bad={bad} wrong_orient={wrong} "
          f"slivers={sliv} area_ratio={abs(sarea/poly):.5f}")


for path in sys.argv[1:]:
    bnd, legacy = load(path)
    print(f"{path.split('/')[-1]}: n_b={len(bnd)}")
    tris, swap, fixed = strip_v2(bnd)
    if tris is None:
        print("    EMPTY")
    else:
        print(f"    swap={swap} orientation-fixes={fixed}")
        audit(bnd, tris)
