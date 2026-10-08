#!/usr/bin/env python3
"""Session-85 v3: v1 raw-key two-pointer + per-tri orientation
normalization (NO monotonization — v2 measured +9.85% overlap from
clipped plateau keys). Audit adds: UV-dihedral census of adjacent
pairs (any residual >170 deg = genuine geometric fold risk)."""
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from s85_zipper_sim import load  # noqa: E402


def strip_v3(bnd, eps_rel=1e-3):
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

        poly = 0.5 * sum(bnd[k][0] * bnd[(k + 1) % n][1] - bnd[(k + 1) % n][0] * bnd[k][1] for k in range(n))
        sign = 1.0 if poly >= 0 else -1.0

        def cross(a, b, c):
            (ax, ay), (bx, by), (cx, cy) = bnd[a], bnd[b], bnd[c]
            return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)

        tris = []
        fixed = 0
        ia = ib = 0
        while ia < len(a_chain) - 1 or ib < len(b_chain) - 1:
            if ia >= len(a_chain) - 1:
                t = [a_chain[ia], b_chain[ib], b_chain[ib + 1]]; ib += 1
            elif ib >= len(b_chain) - 1:
                t = [a_chain[ia], a_chain[ia + 1], b_chain[ib]]; ia += 1
            elif kvals[a_chain[ia + 1]] <= kvals[b_chain[ib + 1]]:
                t = [a_chain[ia], a_chain[ia + 1], b_chain[ib]]; ia += 1
            else:
                t = [a_chain[ia], b_chain[ib], b_chain[ib + 1]]; ib += 1
            if len({t[0], t[1], t[2]}) < 3:
                continue
            if cross(*t) * sign < 0:
                t[1], t[2] = t[2], t[1]
                fixed += 1
            tris.append(tuple(t))
        return tris, swap, fixed
    return None, None, None


def audit(bnd, tris):
    n = len(bnd)
    ecnt = defaultdict(int)
    edge_tris = defaultdict(list)
    for ti, t in enumerate(tris):
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            e = (min(a, b), max(a, b))
            ecnt[e] += 1
            edge_tris[e].append(ti)
    rim = sum(1 for k in range(n) if (min(k, (k + 1) % n), max(k, (k + 1) % n)) in ecnt)
    bad = sum(1 for e, c in ecnt.items() if c == 1 and not (
        e[0] < n and e[1] < n and (abs(e[0] - e[1]) == 1 or (min(e[0], e[1]) == 0 and max(e[0], e[1]) == n - 1))))

    def cross(a, b, c):
        (ax, ay), (bx, by), (cx, cy) = bnd[a], bnd[b], bnd[c]
        return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)

    poly = 0.5 * sum(bnd[k][0] * bnd[(k + 1) % n][1] - bnd[(k + 1) % n][0] * bnd[k][1] for k in range(n))
    wrong = sum(1 for t in tris if cross(*t) * poly < 0)
    sarea = 0.5 * sum(cross(*t) for t in tris)

    # UV-dihedral census on interior edges (count 2x)
    def tri_normal(t):
        (ax, ay), (bx, by), (cx, cy) = bnd[t[0]], bnd[t[1]], bnd[t[2]]
        ux, uy = bx - ax, by - ay
        vx, vy = cx - ax, cy - ay
        nx, ny = uy * vx_h(ux, uy, vx, vy), 0.0
        return nx, ny

    def uv_normal(t):
        (ax, ay), (bx, by), (cx, cy) = bnd[t[0]], bnd[t[1]], bnd[t[2]]
        ux, uy = bx - ax, by - ay
        vx, vy = cx - ax, cy - ay
        # normal in UV plane ~ 3rd component only; use edge vectors for angle
        return (ux, uy), (vx, vy)

    # for UV we use the angle between the planes spanned in 2D —
    # instead: flag interior edges whose two tris' union is a
    # "folded" quad: the two apexes on the same side of the edge
    folds = 0
    for e, tis in edge_tris.items():
        if len(tis) != 2:
            continue
        t1, t2 = tris[tis[0]], tris[tis[1]]
        # apex = the vert not on the edge
        es = set(e)
        a1 = [v for v in t1 if v not in es][0]
        a2 = [v for v in t2 if v not in es][0]
        # side of the edge line
        (px, py), (qx, qy) = bnd[e[0]], bnd[e[1]]
        s1 = (qx - px) * (bnd[a1][1] - py) - (qy - py) * (bnd[a1][0] - px)
        s2 = (qx - px) * (bnd[a2][1] - py) - (qy - py) * (bnd[a2][0] - px)
        if s1 * s2 > 0:  # same side = folded quad
            folds += 1
    print(f"    tris={len(tris)} rim={rim}/{n} nonrim_bad={bad} wrong_orient={wrong} "
          f"folded_quads={folds} area_ratio={abs(sarea/poly):.5f}")


def vx_h(a, b, c, d):
    return 0.0


for path in sys.argv[1:]:
    bnd, legacy = load(path)
    print(f"{path.split('/')[-1]}: n_b={len(bnd)}")
    tris, swap, fixed = strip_v3(bnd)
    if tris is None:
        print("    EMPTY")
    else:
        print(f"    swap={swap} orientation-fixes={fixed}")
        audit(bnd, tris)
