#!/usr/bin/env python3
"""s67 PROTOTYPE: grid + monotone-band triangulation for f125-class
cylinder patches (replaces the earcutr spike-chain for this class).

Chains (UV, cylinder frame u=deg, v):
  - lineA: u=0, v -1.2 .. -0.3123 (3 pts, cached edge discretization)
  - arc:   v=-1.2, u 0..90 (16 pts)
  - lineB: u=90, v -1.2 .. -0.1475 (2 pts)
  - spline rim: u 90..0, v varies (23 pts)
Lattice: n_u=12 columns (7.5°..82.5°), n_v=64 rows, filtered inside.

Checks:
  1. every non-rim edge used exactly 2x (face-local watertight)
  2. total area == shoelace area of the rim polygon (coverage)
  3. sliver guard: min triangle angle
  4. count vs legacy (1060 tris, 700 bnd)
"""
import math
import re
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP = BASE + ".obj", BASE + ".fmap"


def load_mesh():
    verts, tris = [], []
    for line in open(OBJ):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(t) - 1 for t in line.split()[1:4])
            tris.append((a, b, c))
    fid_of = {}
    for line in open(FMAP):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of[int(ti)] = int(fid)
    return verts, tris, fid_of


def load_rim_uv():
    """Rim chains in UV from the ACTUAL mesh rim verts (the cached
    discretization — must be preserved bit-exact)."""
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

    # spline #35059 eval
    a = ents[35059][1]
    deg = int(nums(a.split(",", 2)[1])[0])
    cps = [pt3(r) for r in refs(a)
           if r in ents and ents[r][0] == "CARTESIAN_POINT"]
    lists = re.findall(r"\(([\d.E,\-+\s]+)\)", a)
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
                a_ = 0.0 if k1 <= k0 else (u - k0) / (k1 - k0)
                d[j] = tuple(d[j][q] * (1 - a_) + d[j + 1][q] * a_
                             for q in range(3))
            d.pop()
        return d[0]

    POS = (-2.54, 0.375, -4.349967143371)
    AX = (0.0, 0.0, -1.0)

    def uv3(p):
        rel = tuple(p[k] - POS[k] for k in range(3))
        v = sum(rel[k] * AX[k] for k in range(3))
        rad = tuple(rel[k] - v * AX[k] for k in range(3))
        return (math.degrees(math.atan2(rad[1], -rad[0])), v)

    # rim pts from the mesh (f125 rim verts)
    verts, tris, fid_of = load_mesh()
    FID = 125
    my = set()
    for t, f in fid_of.items():
        if f == FID:
            for vi in tris[t]:
                my.add(vi)
    rim = []
    for vi in my:
        px, py, pz = verts[vi][2] - 2.325, verts[vi][0], verts[vi][1] - 5.4
        u, v = uv3((px, py, pz))
        if (u < 0.4 or u > 89.6 or v < -1.19
                or abs(v - uv3(spline3(0)) [1]) < 5):
            rim.append((u, v))
    # better: categorize precisely
    cats = {"A": [], "arc": [], "B": [], "spline": []}
    su = [uv3(spline3(t / 200.)) for t in range(201)]
    su = sorted(su)

    def rim_v_at(u):
        best = min(su, key=lambda p: abs(p[0] - u))
        return best[1]

    for vi in my:
        px, py, pz = verts[vi][2] - 2.325, verts[vi][0], verts[vi][1] - 5.4
        u, v = uv3((px, py, pz))
        if v < -1.19:
            cats["arc"].append((u, v))
        elif u < 0.4:
            cats["A"].append((u, v))
        elif u > 89.6:
            cats["B"].append((u, v))
        elif abs(v - rim_v_at(u)) < 0.02:
            cats["spline"].append((u, v))
    for k in cats:
        cats[k].sort()
    return cats, su


def zipper(top, bottom):
    """Two-pointer monotone strip between two u-monotone chains.
    top/bottom: lists of (u, v), both sorted by u, same endpooints.
    Returns triangles as index pairs into combined vert list."""
    tris = []
    i, j = 0, 0
    # chains: A = top (index i), B = bottom (index j)
    while i + 1 < len(top) or j + 1 < len(bottom):
        if j + 1 >= len(bottom):
            tris.append(("A", i, "A", i + 1, "B", j)); i += 1; continue
        if i + 1 >= len(top):
            tris.append(("B", j, "B", j + 1, "A", i)); j += 1; continue
        du_a = top[i + 1][0] - top[i][0]
        du_b = bottom[j + 1][0] - bottom[j][0]
        if du_a <= du_b:
            tris.append(("A", i, "A", i + 1, "B", j)); i += 1
        else:
            tris.append(("B", j, "B", j + 1, "A", i)); j += 1
    return tris


def main():
    cats, su = load_rim_uv()
    print("rim chains:",
          {k: len(v) for k, v in cats.items()})
    A = sorted(cats["A"], key=lambda p: -p[1])   # u=0, v desc
    arc = sorted(cats["arc"])                     # u asc at v=-1.2
    B = sorted(cats["B"], key=lambda p: p[1])     # u=90, v asc
    spl = sorted(cats["spline"], reverse=True)    # u desc (90..0)
    # domain: u in [0, 90], v in [-1.2, rim_v(u)]
    def rim_v_at(u):
        best = min(su, key=lambda p: abs(p[0] - u))
        # linear interp between neighbors
        su_s = sorted(su)
        import bisect
        i = bisect.bisect_left(su_s, (u, -9))
        if i == 0: return su_s[0][1]
        if i >= len(su_s): return su_s[-1][1]
        t = (u - su_s[i-1][0]) / (su_s[i][0] - su_s[i-1][0] + 1e-12)
        return su_s[i-1][1]*(1-t) + su_s[i][1]*t

    # lattice (n_u=12, n_v=64 interior per algorithm)
    n_u, n_v = 12, 64
    cols = [90.0 * i / n_u for i in range(1, n_u)]
    rows = [-1.2 + 1.0628 * j / n_v for j in range(1, n_v)]
    lat = [(u, v) for u in cols for v in rows
           if v < rim_v_at(u) - 1e-9 and v > -1.2 + 1e-9]
    print(f"lattice: {len(cols)} cols x {len(rows)} rows -> "
          f"{len(lat)} inside")

    # build vertex list: rim A, arc, B, spline, lattice
    V = ([(p, "rim") for p in A] + [(p, "rim") for p in arc]
         + [(p, "rim") for p in B] + [(p, "rim") for p in spl]
         + [(p, "lat") for p in lat])
    idx = {("rim", p): k for k, (p, t) in enumerate(V[:len(A)+len(arc)+len(B)+len(spl)])}
    # simpler: index by position in each chain
    iA = lambda k: k
    iArc = lambda k: len(A) + k
    iB = lambda k: len(A) + len(arc) + k
    iSpl = lambda k: len(A) + len(arc) + len(B) + k
    iLat = lambda u, v: len(A)+len(arc)+len(B)+len(spl) + lat.index((u, v))

    tris = []
    # 1. bottom band: arc (u asc) vs first lattice row (v = rows[0],
    #    only cols where row0 < rim) — both u-monotone
    # chain bottom: arc points; chain top: lattice row-0 points
    def band(ch1, ch2, make):
        """ch1/ch2: [(u, v, key)] u-sorted; make(tri) appends."""
        i = j = 0
        while i + 1 < len(ch1) or j + 1 < len(ch2):
            if j + 1 >= len(ch2):
                make((ch1[i][2], ch1[i+1][2], ch2[j][2])); i += 1
            elif i + 1 >= len(ch1):
                make((ch2[j][2], ch2[j+1][2], ch1[i][2])); j += 1
            elif ch1[i+1][0] - ch1[i][0] <= ch2[j+1][0] - ch2[j][0]:
                make((ch1[i][2], ch1[i+1][2], ch2[j][2])); i += 1
            else:
                make((ch2[j][2], ch2[j+1][2], ch1[i][2])); j += 1

    # lattice grid rows per column-set: group lattice by v
    byrow = defaultdict(list)
    for (u, v) in lat:
        byrow[v].append(u)
    rowvs = sorted(byrow)
    # interior grid: for each pair of adjacent rows, zipper cols
    for r in range(len(rowvs) - 1):
        ch1 = [(u, rowvs[r], iLat(u, rowvs[r]))
               for u in sorted(byrow[rowvs[r]])]
        ch2 = [(u, rowvs[r+1], iLat(u, rowvs[r+1]))
               for u in sorted(byrow[rowvs[r+1]])]
        band(ch1, ch2, lambda t: tris.append(t))
    # bottom band: arc vs first row
    ch_arc = [(p[0], p[1], iArc(k)) for k, p in enumerate(arc)]
    ch_r0 = [(u, rowvs[0], iLat(u, rowvs[0]))
             for u in sorted(byrow[rowvs[0]])]
    band(ch_arc, ch_r0, lambda t: tris.append(t))
    # top band: last row vs spline (u asc both)
    ch_rlast = [(u, rowvs[-1], iLat(u, rowvs[-1]))
                for u in sorted(byrow[rowvs[-1]])]
    ch_spl = [(p[0], p[1], iSpl(k)) for k, p in
              enumerate(sorted(spl))]
    band(ch_rlast, ch_spl, lambda t: tris.append(t))
    # left band: lineA (v-monotone) vs first column of each row
    colA = [(min(byrow[v]), v, iLat(min(byrow[v]), v))
            for v in rowvs]
    # vertical zipper: advance by v
    chA = [(p[1], p[0], iA(k)) for k, p in enumerate(A)]  # (v, u, idx)
    chL = [(p[1], p[0], p[2]) for p in colA]
    i = j = 0
    while i + 1 < len(chA) or j + 1 < len(chL):
        if j + 1 >= len(chL):
            tris.append((chA[i][2], chA[i+1][2], chL[j][2])); i += 1
        elif i + 1 >= len(chA):
            tris.append((chL[j][2], chL[j+1][2], chA[i][2])); j += 1
        elif chA[i+1][0] - chA[i][0] <= chL[j+1][0] - chL[j][0]:
            tris.append((chA[i][2], chA[i+1][2], chL[j][2])); i += 1
        else:
            tris.append((chL[j][2], chL[j+1][2], chA[i][2])); j += 1
    # right band: lineB vs last column
    colB = [(max(byrow[v]), v, iLat(max(byrow[v]), v))
            for v in rowvs]
    chBv = [(p[1], p[0], iB(k)) for k, p in enumerate(B)]
    chR = [(p[1], p[0], p[2]) for p in colB]
    i = j = 0
    while i + 1 < len(chBv) or j + 1 < len(chR):
        if j + 1 >= len(chR):
            tris.append((chBv[i][2], chBv[i+1][2], chR[j][2])); i += 1
        elif i + 1 >= len(chBv):
            tris.append((chR[j][2], chR[j+1][2], chBv[i][2])); j += 1
        elif chBv[i+1][0] - chBv[i][0] <= chR[j+1][0] - chR[j][0]:
            tris.append((chBv[i][2], chBv[i+1][2], chR[j][2])); i += 1
        else:
            tris.append((chR[j][2], chR[j+1][2], chBv[i][2])); j += 1
    # corner regions: between arc ends and lineA/lineB bottoms and the
    # first lattice row's ends — handled by the bands' endpoints.
    print(f"triangles: {len(tris)}")

    # VERIFY: edge usage
    edge_use = defaultdict(int)
    P = [p for (p, _) in V]
    for t in tris:
        a, b, c = t
        for x, y in ((a, b), (b, c), (c, a)):
            edge_use[(min(x, y), max(x, y))] += 1
    rim_idx = set(range(len(A) + len(arc) + len(B) + len(spl)))
    bad = {e: n for e, n in edge_use.items()
           if n != 2 and not (e[0] in rim_idx or e[1] in rim_idx)}
    print(f"non-rim edges with usage != 2: {len(bad)}")
    for e, n in list(bad.items())[:10]:
        print(f"   {e}: {n}")
    # missing lattice verts (not in any triangle)?
    used = set()
    for t in tris:
        used.update(t)
    miss = [k for k in range(len(V)) if k not in used]
    print(f"unused verts: {len(miss)}")

    # area check (shoelace in (arc-len, v) metric: u_deg*0.125*rad)
    def area(p):
        s = 0.0
        for k in range(len(p)):
            x1, y1 = p[k][0] * 0.125 * math.pi / 180, p[k][1]
            x2, y2 = p[(k + 1) % len(p)][0] * 0.125 * math.pi / 180, \
                p[(k + 1) % len(p)][1]
            s += x1 * y2 - x2 * y1
        return abs(s) / 2
    poly = ([p for p in A] + [p for p in arc] + [p for p in B]
            + [p for p in sorted(spl)])
    target = area(poly)
    tot = 0.0
    for t in tris:
        (x1, y1), (x2, y2), (x3, y3) = (
            (P[t[0]][0] * 0.125 * math.pi / 180, P[t[0]][1]),
            (P[t[1]][0] * 0.125 * math.pi / 180, P[t[1]][1]),
            (P[t[2]][0] * 0.125 * math.pi / 180, P[t[2]][1]))
        tot += abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)) / 2
    print(f"area: mesh {tot:.6f} vs polygon {target:.6f} "
          f"({100*tot/target:.2f}%)")

    # sliver check: min angle
    worst = 90.0
    for t in tris[:]:
        pts = [(P[k][0] * 0.125 * math.pi / 180, P[k][1]) for k in t]
        angs = []
        for i in range(3):
            a, b, c = pts[i], pts[(i+1) % 3], pts[(i+2) % 3]
            v1 = (b[0]-a[0], b[1]-a[1]); v2 = (c[0]-a[0], c[1]-a[1])
            n1 = math.hypot(*v1); n2 = math.hypot(*v2)
            if n1 < 1e-12 or n2 < 1e-12:
                angs.append(0.0); continue
            cs = (v1[0]*v2[0] + v1[1]*v2[1]) / (n1 * n2)
            angs.append(math.degrees(math.acos(
                max(-1, min(1, cs)))))
        worst = min(worst, min(angs))
    print(f"min triangle angle: {worst:.2f}°")


if __name__ == "__main__":
    main()
