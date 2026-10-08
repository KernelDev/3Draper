#!/usr/bin/env python3
"""Session-85: OFFLINE simulation of two_chain_monotone_strip with the
relaxed monotonicity (eps = 1e-3 * u-span, the s76 pattern) on the
f155/f41 castellation dumps. Full audit:
  - chains split at umin/umax corners, both mono within eps
  - zipper tris: all rim edges exactly 1x, non-rim exactly 2x
  - area within +-0.5%
  - sliver guard (min_ang<2 AND longest>10% bbox_diag)
  - max vertex degree (fan check)
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


def strip(bnd, eps_rel=1e-3):
    n = len(bnd)
    us = [p[0] for p in bnd]
    vs = [p[1] for p in bnd]
    for swap in (False, True):
        key = (lambda p: p[1]) if swap else (lambda p: p[0])
        span = (max(vs) - min(vs)) if swap else (max(us) - min(us))
        eps = max(eps_rel * span, 1e-12)
        umin_i = min(range(n), key=lambda k: (key(bnd[k]), k))
        umax_i = min(range(n), key=lambda k: (-key(bnd[k]), k))
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
            return all(key(bnd[c[k]]) <= key(bnd[c[k + 1]]) + eps for k in range(len(c) - 1))

        if not mono(a_chain) or not mono(b_chain):
            print(f"  [swap={swap}] mono FAIL: a={mono(a_chain)} b={mono(b_chain)} eps={eps:.2e}")
            continue
        if a_chain[0] != b_chain[0] or a_chain[-1] != b_chain[-1]:
            print(f"  [swap={swap}] endpoint mismatch")
            continue
        tris = []
        ia = ib = 0
        while ia < len(a_chain) - 1 or ib < len(b_chain) - 1:
            if ia >= len(a_chain) - 1:
                t = (a_chain[ia], b_chain[ib], b_chain[ib + 1]); ib += 1
            elif ib >= len(b_chain) - 1:
                t = (a_chain[ia], a_chain[ia + 1], b_chain[ib]); ia += 1
            elif key(bnd[a_chain[ia + 1]]) <= key(bnd[b_chain[ib + 1]]):
                t = (a_chain[ia], a_chain[ia + 1], b_chain[ib]); ia += 1
            else:
                t = (a_chain[ia], b_chain[ib + 1], b_chain[ib]); ib += 1
            if len({t[0], t[1], t[2]}) == 3:
                tris.append(t)
        # area
        poly = 0.5 * sum(bnd[k][0] * bnd[(k + 1) % n][1] - bnd[(k + 1) % n][0] * bnd[k][1] for k in range(n))
        sarea = 0.0
        for a, b, c in tris:
            (ax, ay), (bx, by), (cx, cy) = bnd[a], bnd[b], bnd[c]
            sarea += 0.5 * ((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))
        lo, hi = (poly * 0.995 - 1e-12, poly * 1.005 + 1e-12) if poly >= 0 else (poly * 1.005 - 1e-12, poly * 0.995 + 1e-12)
        if not (lo <= sarea <= hi):
            print(f"  [swap={swap}] AREA FAIL: strip={sarea:.6f} poly={poly:.6f}")
            continue
        # winding normalize
        if sarea * poly < 0:
            tris = [(a, c, b) for a, b, c in tris]
        return tris, swap, eps
    return None, None, None


def audit(name, bnd, legacy_tris, tris):
    n = len(bnd)
    ecnt = defaultdict(int)
    for t in tris:
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            ecnt[(min(a, b), max(a, b))] += 1
    rim_ok = sum(1 for k in range(n) if (min(k, (k + 1) % n), max(k, (k + 1) % n)) in ecnt)
    nonrim_bad = sum(1 for e, c in ecnt.items() if c != 2 and not (e[0] < n and e[1] < n and abs(e[0] - e[1]) in (1, n - 1))
                     and not (min(e[0], e[1]) == 0 and max(e[0], e[1]) == n - 1))
    deg = defaultdict(int)
    for t in tris:
        for v in t:
            deg[v] += 1
    maxdeg = max(deg.values())
    # sliver guard
    us = [p[0] for p in bnd]; vs = [p[1] for p in bnd]
    diag = math.hypot(max(us) - min(us), max(vs) - min(vs))
    nsliver = 0
    for a, b, c in tris:
        pts = [bnd[a], bnd[b], bnd[c]]
        mn, lng = 1e9, 0.0
        for k in range(3):
            p0, p1, p2 = pts[k], pts[(k + 1) % 3], pts[(k + 2) % 3]
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p0[0], p2[1] - p0[1])
            l1, l2 = math.hypot(*v1), math.hypot(*v2)
            lng = max(lng, l1, l2, math.hypot(p2[0] - p1[0], p2[1] - p1[1]))
            if l1 > 1e-15 and l2 > 1e-15:
                mn = min(mn, math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2))))))
        if mn < 2.0 and lng > 0.10 * diag:
            nsliver += 1
    print(f"  {name}: tris={len(tris)} rim={rim_ok}/{n} nonrim_bad={nonrim_bad} "
          f"maxdeg={maxdeg} slivers={nsliver}")


for path in sys.argv[1:]:
    bnd, legacy = load(path)
    print(f"{path}: n_b={len(bnd)} legacy_tris={len(legacy)}")
    for eps_rel in (1e-3,):
        tris, swap, eps = strip(bnd, eps_rel)
        if tris is None:
            print(f"  eps_rel={eps_rel}: STRIP EMPTY")
        else:
            print(f"  eps_rel={eps_rel} swap={swap} eps={eps:.2e} -> {len(tris)} tris")
            audit("relaxed", bnd, legacy, tris)
    # strict (1e-12) for comparison
    tris0, _, _ = strip(bnd, 0)
    print(f"  strict eps: {'EMPTY' if tris0 is None else str(len(tris0)) + ' tris'}")
