#!/usr/bin/env python3
"""session-55 brick-chain prototype (pre-implementation measurement).

Parses fold_face_probe DRAPPER_DUMP_TRI_INPUT dumps (interior = chain
append order), rebuilds the P2 complement ring for chain variants and
measures the P2 cell map (connected components = strips) in 3D through
the torus surface.

Variants:
  comb   — the dumped order as-is (session-54 aniso v3 validation:
           expect ~NV teeth, full-span along the run axis)
  tower  — K-cell capped runs stacked in u-slabs (the brick): every
           chain step = 1 lattice cell; P2 strips must collapse to
           K-cell bricks along the low-curvature axis.

Usage:
  brick_proto.py <dump.txt> <R> <r> [K]
"""
import sys
from collections import deque


def parse_dump(path):
    bnd, interior, hdr = [], [], ""
    sec = None
    with open(path) as f:
        for line in f:
            if line.startswith("type="):
                hdr = line.strip()
                continue
            t = line.split()
            if not t:
                continue
            if t[0] in ("boundary", "hole", "interior", "tris"):
                sec = t[0]
                continue
            if sec == "boundary" and t[0] == "b":
                bnd.append((float(t[1]), float(t[2])))
            elif sec == "interior" and t[0] == "i":
                interior.append((float(t[1]), float(t[2])))
    return hdr, bnd, interior


def cluster(vals):
    vals = sorted(vals)
    if not vals:
        return []
    span = vals[-1] - vals[0]
    tol = max(span * 1e-9, 1e-12)
    uniq = [vals[0]]
    for a, b in zip(vals, vals[1:]):
        if b - a > tol:
            uniq.append(b)
    return uniq


def point_in_poly(x, y, poly):
    c = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if xin > x:
                c = not c
    return c


def segs_intersect(p1, p2, p3, p4):
    def orient(a, b, c):
        v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if v > 1e-15:
            return 1
        if v < -1e-15:
            return -1
        return 0

    def on_seg(a, b, c):
        return (
            min(a[0], b[0]) - 1e-15 <= c[0] <= max(a[0], b[0]) + 1e-15
            and min(a[1], b[1]) - 1e-15 <= c[1] <= max(a[1], b[1]) + 1e-15
        )

    d1 = orient(p3, p4, p1)
    d2 = orient(p3, p4, p2)
    d3 = orient(p1, p2, p3)
    d4 = orient(p1, p2, p4)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return True
    # touching/colinear does not count as a crossing for simplicity
    # purposes here (shared endpoints are expected between adjacent
    # edges); strict interior crossings only.
    return False


def poly_simple(poly):
    n = len(poly)
    segs = [(poly[i], poly[(i + 1) % n]) for i in range(n)]
    for i in range(n):
        a1, a2 = segs[i]
        for j in range(i + 1, n):
            if j == i:
                continue
            # skip adjacent edges (share an endpoint)
            if j == (i + 1) % n or (j + 1) % n == i or (i == 0 and j == n - 1):
                continue
            b1, b2 = segs[j]
            if segs_intersect(a1, a2, b1, b2):
                return False, (i, j)
    return True, None


def poly_area(poly):
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def torus3d(u, v, R, r):
    import math

    w = R + r * math.cos(v)
    return (w * math.cos(u), w * math.sin(u), r * math.sin(v))


def analyse(name, bnd, chain, us, vs, R, r, draw=False):
    """chain = interior order; P2 = [b0, chain rev, b-1]."""
    ring = [bnd[0]] + chain[::-1] + [bnd[-1]]
    p1ring = bnd + chain
    ok2, bad2 = poly_simple(ring)
    ok1, bad1 = poly_simple(p1ring)
    a2 = abs(poly_area(ring))
    # cell map
    nu, nv = len(us) - 1, len(vs) - 1
    grid = [[False] * nu for _ in range(nv)]
    for iv in range(nv):
        vc = (vs[iv] + vs[iv + 1]) / 2
        for iu in range(nu):
            uc = (us[iu] + us[iu + 1]) / 2
            grid[iv][iu] = point_in_poly(uc, vc, ring)
    # components (4-connectivity)
    seen = [[False] * nu for _ in range(nv)]
    comps = []
    for iv in range(nv):
        for iu in range(nu):
            if grid[iv][iu] and not seen[iv][iu]:
                q = deque([(iv, iu)])
                seen[iv][iu] = True
                cells = []
                while q:
                    y, x = q.popleft()
                    cells.append((y, x))
                    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        yy, xx = y + dy, x + dx
                        if (
                            0 <= yy < nv
                            and 0 <= xx < nu
                            and grid[yy][xx]
                            and not seen[yy][xx]
                        ):
                            seen[yy][xx] = True
                            q.append((yy, xx))
                comps.append(cells)
    # per-component 3D metrics: STRAIGHT RUNS (maximal consecutive
    # cell runs per row / per column) — the ribbon-chord proxy. A
    # component's bounding box can span the domain while every straight
    # segment stays brick-sized (labyrinth of connected bricks), and
    # the s54 discriminator was the straight-strip length.
    import math

    max_run_u = 0.0  # longest straight run along u (3D)
    max_run_v = 0.0  # longest straight run along v (3D)
    for cells in comps:
        rows = {}
        cols = {}
        for y, x in cells:
            rows.setdefault(y, []).append(x)
            cols.setdefault(x, []).append(y)
        for y, xs in rows.items():
            xs.sort()
            run = 1
            for a, b in zip(xs, xs[1:]):
                if b == a + 1:
                    run += 1
                else:
                    max_run_u = max(
                        max_run_u, run * (us[1] - us[0]) * (R + r * math.cos(vs[y]))
                    )
                    run = 1
            max_run_u = max(
                max_run_u, run * (us[1] - us[0]) * (R + r * math.cos(vs[y]))
            )
        for x, ys in cols.items():
            ys.sort()
            run = 1
            for a, b in zip(ys, ys[1:]):
                if b == a + 1:
                    run += 1
                else:
                    max_run_v = max(max_run_v, run * (vs[1] - vs[0]) * r)
                    run = 1
            max_run_v = max(max_run_v, run * (vs[1] - vs[0]) * r)
    stats = []
    for cells in comps:
        stats.append((len(cells), 0, 0, 0, 0))
    stats.sort(key=lambda s: -s[0])
    print(f"--- {name}: P1_simple={ok1} P2_simple={ok2} area={a2:.3e}")
    if not ok2:
        print(f"    P2 self-intersect at edges {bad2}")
    ncell = sum(s[0] for s in stats)
    print(
        f"    P2 cells={ncell}/{nu*nv} components={len(stats)} "
        f"max_run_u={max_run_u:.4f} max_run_v={max_run_v:.4f} "
        f"sag_u={max_run_u**2/(8*(R+r)):.2e} sag_v={max_run_v**2/(8*r):.2e}"
    )
    if draw:
        print("    P2 map (v rows, top=v_hi):")
        for iv in range(nv - 1, -1, -1):
            print("      " + "".join("#" if grid[iv][iu] else "." for iu in range(nu)))
    return ok1 and ok2 and a2 > 0, stats


def build_tower(us, vs, K, s0):
    """Brick/tower chain: runs of <=K cells along u inside disjoint
    u-slabs of K+1 points; slabs traversed as a boustrophedon along v.
    Every chain step = 1 lattice cell (horizontal inside runs and slab
    transitions, vertical at run turns). K >= nu-1 degenerates to the
    session-54 row comb. Returns None on inconsistency."""
    nu, nv = len(us), len(vs)
    slabs = []
    a = 0
    while a < nu:
        b = min(a + K, nu - 1)
        slabs.append((a, b))
        a = b + 1
    s0_iu, s0_iv = s0
    if s0_iu >= nu // 2:
        order = list(range(len(slabs) - 1, -1, -1))
    else:
        order = list(range(len(slabs)))
    up = s0_iv < nv // 2  # first slab traversed toward the far v end
    chain = []
    prev = None  # (a, b) of previous slab in traversal order
    for t, si in enumerate(order):
        a, b = slabs[si]
        entry = None
        if t == 0:
            entry = s0_iu
            if entry not in (a, b):
                return None
        rows = range(0, nv) if up else range(nv - 1, -1, -1)
        for row in rows:
            if entry is None:
                # first row of a later slab: enter from the column
                # adjacent to the previous slab's exit
                if prev is not None and a == prev[1] + 1:
                    entry = a
                elif prev is not None and b == prev[0] - 1:
                    entry = b
                else:
                    return None
            if entry == a:
                run = range(a, b + 1)
                exit_col = b
            elif entry == b:
                run = range(b, a - 1, -1)
                exit_col = a
            else:
                return None
            chain.extend((us[c], vs[row]) for c in run)
            entry = exit_col
        prev = (a, b)
        up = not up
    return chain


def build_hybrid(us, vs, chunk=4):
    """session-55 hybrid brick chain.

    Structure (verified by hand on 23x23):
    - rows 0..2*NB-1 in 2-row bands; each band = a double zigzag:
      down-right zigzag alternating the two rows over consecutive
      4-point chunks, then a return zigzag R->L alternating the
      BOTTOM row's missing chunks with the TOP row's missing chunks
      (turns = 1 vertical + (1,1) diagonals). All steps local.
    - the remaining bottom rows (23 - 2*NB, here 3) as a tower:
      u-slabs of `chunk` points, boustrophedon over the rows, runs
      alternate direction per row (slab transitions = 1 col).
    Ends: s0 = (row 0, u_lo); s_end = (bottom-1, u_hi) when the
    slab count is even. Returns None on ragged lattices.
    """
    nu, nv = len(us), len(vs)
    if nv < 7 or nv % 2 == 0:
        # prototype: odd row count (last 3 rows = tower); the Rust
        # version generalizes to even counts (2-row tower).
        return None
    chain = []

    def run(row, a, b):
        """emit points (row, a..b) inclusive; returns nothing."""
        if a <= b:
            chain.extend((us[c], vs[row]) for c in range(a, b + 1))
        else:
            chain.extend((us[c], vs[row]) for c in range(a, b - 1, -1))

    # chunk column ranges: [0..3],[4..7],...,[20..22]
    chunks = []
    a = 0
    while a < nu:
        b = min(a + chunk - 1, nu - 1)
        chunks.append((a, b))
        a = b + 1
    nch = len(chunks)  # 6
    n_bands = (nv - 3) // 2  # 10 bands cover rows 0..19
    # 2-row double-zigzag bands
    for k in range(n_bands):
        A, B = 2 * k, 2 * k + 1
        # zigzag down-right: A even chunks, B odd chunks
        for i in range(nch):
            r = A if i % 2 == 0 else B
            a, b = chunks[i]
            run(r, a, b)
        # return R->L: A odd chunks, B even chunks (the missing ones)
        # first turn: vertical from (B, nu-1) to (A, nu-1)
        for i in range(nch - 1, -1, -1):
            r = A if i % 2 == 1 else B
            a, b = chunks[i]
            run(r, b, a)
    # bottom tower: rows n_bands*2 .. nv-1 (3 rows), slabs = chunks
    rows = list(range(n_bands * 2, nv))  # [20, 21, 22]
    up = True
    for si in range(len(chunks)):
        a, b = chunks[si]
        rr = rows if up else rows[::-1]
        first = True
        for row in rr:
            if first:
                # enter from the transition side
                run(row, a, b)
                first = False
                last_end = b
            else:
                if last_end == b:
                    run(row, b, a)
                    last_end = a
                else:
                    run(row, a, b)
                    last_end = b
        up = not up
    return chain


def main():
    path = sys.argv[1]
    R, r = float(sys.argv[2]), float(sys.argv[3])
    K = int(sys.argv[4]) if len(sys.argv) > 4 else 3
    hdr, bnd, interior = parse_dump(path)
    print(hdr)
    us = cluster([p[0] for p in interior])
    vs = cluster([p[1] for p in interior])
    print(f"lattice {len(us)}x{len(vs)} boundary={len(bnd)}")
    # closure
    print(f"ring_start={bnd[0]} ring_last={bnd[-1]}")
    # comb validation (dumped order)
    analyse("comb(dumped)", bnd, interior, us, vs, R, r, draw=True)
    # tower bricks from each corner
    corners = {
        "lo/lo": (0, 0),
        "lo/hi": (0, len(vs) - 1),
        "hi/lo": (len(us) - 1, 0),
        "hi/hi": (len(us) - 1, len(vs) - 1),
    }
    for cname, (ciu, civ) in corners.items():
        ch = build_tower(us, vs, K, (ciu, civ))
        if ch is None or len(ch) != len(interior) or len(set(ch)) != len(interior):
            print(
                f"tower({cname},K={K}): BAD COVERAGE "
                f"{0 if ch is None else len(set(ch))} != {len(interior)}"
            )
            continue
        analyse(f"tower({cname},K={K})", bnd, ch, us, vs, R, r, draw=(cname == "lo/lo"))


if __name__ == "__main__":
    main()
