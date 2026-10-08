#!/usr/bin/env python3
"""Session-85 v4: STACK-BASED monotone polygon triangulation (de Berg
ch.3) for the wobble-tolerant path. The two-pointer (s65, bit-frozen)
stays for strict-mono faces.

Sweep u-increasing over the merged chain vertices; same-chain pops
with per-chain convexity sign (lower chain walked in boundary
direction: left turn = clippable; upper chain walked reversed: right
turn = clippable); opposite-chain fans. Per-tri orientation
normalized to the polygon sign. FULL AUDIT as the guard."""
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from s85_zipper_sim import load  # noqa: E402


def strip_v4(bnd, eps_rel=1e-3):
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

        # which chain is "lower" in the key-perpendicular sense?
        # sample midpoints: compare the perpendicular coordinate
        perp = (lambda p: p[0]) if swap else (lambda p: p[1])
        amid = bnd[a_chain[len(a_chain) // 2]]
        bmid = bnd[b_chain[len(b_chain) // 2]]
        # chain tag: 0 = lower (smaller perp), 1 = upper
        a_tag = 0 if perp(amid) <= perp(bmid) else 1
        b_tag = 1 - a_tag

        # merged u-increasing sequence of (idx, tag), excluding the
        # duplicated endpoints; endpoints re-added at the ends
        ia, ib = 1, 1  # skip the shared umin vertex
        merged = []
        while ia < len(a_chain) - 1 or ib < len(b_chain) - 1:
            if ib >= len(b_chain) - 1 or (ia < len(a_chain) - 1 and kvals[a_chain[ia]] <= kvals[b_chain[ib]]):
                merged.append((a_chain[ia], a_tag)); ia += 1
            else:
                merged.append((b_chain[ib], b_tag)); ib += 1

        def cross3(p, q, r):
            (px, py), (qx, qy), (rx, ry) = bnd[p], bnd[q], bnd[r]
            return (qx - px) * (ry - py) - (rx - px) * (qy - py)

        tris = []
        fixed = 0

        def emit(vi, w, x):
            nonlocal fixed
            t = [vi, w, x]
            if len(set(t)) < 3:
                return
            if cross3(*t) * sign < 0:
                t[1], t[2] = t[2], t[1]
                fixed += 1
            tris.append(tuple(t))

        # stack entries: (idx, tag) — start with the umin corner vertex
        # (session-85 fix: without it the first fan cuts the vmin corner
        # off — the two vmin rim edges go missing, replaced by a
        # corner-cutting diagonal)
        if not merged:
            # degenerate: one chain has no interior verts
            continue
        stack = [(umin_i, a_tag)]
        for vi, tag in merged:
            if not stack:
                stack.append((vi, tag))
                continue
            top_idx, top_tag = stack[-1]
            if tag != top_tag:
                # opposite chain: fan across the whole stack
                # emit (vi, stack[k], stack[k+1]) for all k
                fan = [s[0] for s in stack]
                for k in range(len(fan) - 1):
                    emit(vi, fan[k], fan[k + 1])
                stack = [stack[-1], (vi, tag)]
            else:
                # same chain: pop clippable ears
                while len(stack) >= 2:
                    w = stack[-1][0]
                    x = stack[-2][0]
                    c = cross3(x, w, vi)  # turn at w from x->w->vi
                    # clippable: lower chain (boundary-direction walk):
                    # left turn (c>0); upper chain (reversed walk):
                    # right turn (c<0)
                    clippable = (c > 0) if (tag == 0) else (c < 0)
                    if clippable:
                        emit(vi, w, x)
                        stack.pop()
                    else:
                        break
                stack.append((vi, tag))
        # close with the umax vertex (opposite chain to everything left)
        vmax = a_chain[-1]
        fan = [s[0] for s in stack]
        for k in range(len(fan) - 1):
            emit(vmax, fan[k], fan[k + 1])
        return tris, swap, fixed
    return None, None, None


def audit(bnd, tris, verbose=True):
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
    folds = 0
    for e, tis in edge_tris.items():
        if len(tis) != 2:
            continue
        t1, t2 = tris[tis[0]], tris[tis[1]]
        es = set(e)
        a1 = [v for v in t1 if v not in es]
        a2 = [v for v in t2 if v not in es]
        if not a1 or not a2:
            continue
        (px, py), (qx, qy) = bnd[e[0]], bnd[e[1]]
        s1 = (qx - px) * (bnd[a1[0]][1] - py) - (qy - py) * (bnd[a1[0]][0] - px)
        s2 = (qx - px) * (bnd[a2[0]][1] - py) - (qy - py) * (bnd[a2[0]][0] - px)
        if s1 * s2 > 0:
            folds += 1
    deg = defaultdict(int)
    for t in tris:
        for v in t:
            deg[v] += 1
    if verbose:
        print(f"    tris={len(tris)} rim={rim}/{n} nonrim_bad={bad} wrong_orient={wrong} "
              f"folded_quads={folds} maxdeg={max(deg.values())} area_ratio={abs(sarea/poly):.6f}")
    return rim == n and bad == 0 and wrong == 0 and folds == 0 and 0.995 <= abs(sarea / poly) <= 1.005


for path in sys.argv[1:]:
    bnd, legacy = load(path)
    print(f"{path.split('/')[-1]}: n_b={len(bnd)}")
    tris, swap, fixed = strip_v4(bnd)
    if tris is None:
        print("    EMPTY")
    else:
        print(f"    swap={swap} orientation-fixes={fixed}")
        ok = audit(bnd, tris)
        print(f"    AUDIT: {'PASS' if ok else 'FAIL'}")
