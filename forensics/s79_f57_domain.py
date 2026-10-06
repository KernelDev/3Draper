#!/usr/bin/env python3
"""s79: f57 (HM Plane) domain shape analysis — thinness, PCA chains,
monotonicity — to see if degen_strip_zipper would accept it."""
import math
from collections import defaultdict

OBJ = "/tmp/s79_objs/brep62542_f57_s54645_Plane.obj"


def load():
    verts, tris = [], []
    with open(OBJ) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                a, b, c = [int(x) for x in line.split()[1:4]]
                tris.append((a - 1, b - 1, c - 1))
    return verts, tris


def main():
    verts, tris = load()
    print(f"f57: {len(verts)} verts, {len(tris)} tris")

    # boundary edges = edges used exactly once
    cnt = defaultdict(int)
    for t in tris:
        for k in range(3):
            e = (min(t[k], t[(k + 1) % 3]), max(t[k], t[(k + 1) % 3]))
            cnt[e] += 1
    bnd = [e for e, c in cnt.items() if c == 1]
    print(f"boundary edges: {len(bnd)}")

    # chain the boundary into a cycle
    adj = defaultdict(list)
    for a, b in bnd:
        adj[a].append(b)
        adj[b].append(a)
    start = next(iter(adj))
    cycle = [start]
    prev = None
    cur = start
    while True:
        nxt = [x for x in adj[cur] if x != prev]
        if not nxt:
            break
        prev, cur = cur, nxt[0]
        if cur == start:
            break
        cycle.append(cur)
    print(f"boundary cycle: {len(cycle)} verts")

    # project onto the plane's PCA axes (use y,z since x is ~const)
    pts = [verts[i] for i in cycle]
    n = len(pts)
    cy = sum(p[1] for p in pts) / n
    cz = sum(p[2] for p in pts) / n
    # PCA 2x2 on (y,z)
    syy = sum((p[1] - cy) ** 2 for p in pts) / n
    szz = sum((p[2] - cz) ** 2 for p in pts) / n
    syz = sum((p[1] - cy) * (p[2] - cz) for p in pts) / n
    # main axis eigen
    tr = syy + szz
    det = syy * szz - syz * syz
    disc = max(tr * tr / 4 - det, 0.0)
    lam = tr / 2 + math.sqrt(disc)
    # eigenvector for lam
    # (syy-lam)vy + syz*vz = 0
    vy, vz = syz, lam - syy
    ln = math.hypot(vy, vz) or 1.0
    vy, vz = vy / ln, vz / ln
    print(f"PCA main axis (y,z): ({vy:.4f},{vz:.4f}), "
          f"lam1={lam:.4f} lam2={det/lam:.4f}")

    # t-projections
    ts = [(p[1] - cy) * vy + (p[2] - cz) * vz for p in pts]
    i_min = ts.index(min(ts))
    i_max = ts.index(max(ts))
    print(f"t range: [{min(ts):.3f}, {max(ts):.3f}]")

    # two chains: i_min..i_max and i_max..i_min (through wrap)
    def chain(a, b):
        if a <= b:
            return list(range(a, b + 1))
        return list(range(a, n)) + list(range(0, b + 1))

    c1 = chain(i_min, i_max)
    c2 = chain(i_max, i_min)
    t1 = [ts[i] for i in c1]
    t2 = [ts[i] for i in c2]

    def monotone(tt):
        viol = 0
        maxback = 0.0
        trange = max(tt) - min(tt)
        for k in range(1, len(tt)):
            d = tt[k] - tt[k - 1]
            if d < 0:
                viol += 1
                maxback = max(maxback, -d)
        return viol, maxback, trange

    v1, b1, r1 = monotone(t1)
    v2, b2, r2 = monotone(t2)
    print(f"chain1 (len {len(c1)}): {v1} backtracks, "
          f"max back={b1:.4f}, range={r1:.3f}")
    print(f"chain2 (len {len(c2)}): {v2} backtracks, "
          f"max back={b2:.4f}, range={r2:.3f}")

    # thinness
    area2 = 0.0
    perim = 0.0
    for i in range(n):
        j = (i + 1) % n
        p0, p1 = pts[i], pts[j]
        area2 += (p0[1] - 0) * (p1[2] - 0) - (p1[1] - 0) * (p0[2] - 0)
        perim += math.dist(p0, p1)
    semi = perim / 2
    thinness = abs(area2) / 2 / (semi * semi)
    print(f"area={abs(area2)/2:.4f} perim={perim:.4f} "
          f"thinness={thinness:.4f} (gate 0.12)")

    # deg distribution of the fan
    deg = defaultdict(int)
    for t in tris:
        for v in t:
            deg[v] += 1
    top = sorted(deg.items(), key=lambda kv: -kv[1])[:5]
    print("top degrees:", top)


if __name__ == "__main__":
    main()
