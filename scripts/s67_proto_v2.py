#!/usr/bin/env python3
"""s67 PROTOTYPE v2: CLOSED-ANNULUS zipper design for f125.

Design:
  - rim polygon = ONE closed chain (cached pts, CCW in UV)
  - lattice = regular grid strictly inside; its HULL = closed chain
    (walk the outermost points of the grid: bottom row L→R, right
    col B→T, top row R→L, left col T→B)
  - interior: grid quads
  - annulus: closed two-pointer zipper between rim and hull chains
  (corner-free by construction; handles ANY density mismatch)

Checks: edge usage, area, min angle, and 3D fold simulation
(same-face adjacent dihedral > 170 deg on the true cylinder).
"""
import math
import re
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP = BASE + ".obj", BASE + ".fmap"


def load_all():
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


POS = (-2.54, 0.375, -4.349967143371)
AX = (0.0, 0.0, -1.0)
XDIR = (-1.0, 0.0, 0.0)   # u=0 direction (ref_dir)
R = 0.125


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


YDIR = cross(AX, XDIR)


def to_local(p):
    return (p[2] - 2.325, p[0], p[1] - 5.4)


def uv3(p):
    rel = (p[0]-POS[0], p[1]-POS[1], p[2]-POS[2])
    v = sum(rel[k]*AX[k] for k in range(3))
    rx = sum(rel[k]*XDIR[k] for k in range(3))
    ry = sum(rel[k]*YDIR[k] for k in range(3))
    return (math.degrees(math.atan2(ry, rx)), v)


def uv_to_3d(u_deg, v):
    u = math.radians(u_deg)
    d = (math.cos(u)*XDIR[0] + math.sin(u)*YDIR[0],
         math.cos(u)*XDIR[1] + math.sin(u)*YDIR[1],
         math.cos(u)*XDIR[2] + math.sin(u)*YDIR[2])
    return (POS[0] + R*d[0] + v*AX[0],
            POS[1] + R*d[1] + v*AX[1],
            POS[2] + R*d[2] + v*AX[2])


def main():
    verts, tris, fid_of = load_all()
    FID = 125
    my = set()
    for t, f in fid_of.items():
        if f == FID:
            for vi in tris[t]:
                my.add(vi)

    # rim polygon in CCW order: use the mesh rim verts, ordered by
    # walking the domain boundary: lineA up, spline 90->0 ... build
    # from categories
    # (reuse the categorization; then order CCW)
    arc, A, B, spl = [], [], [], []
    # spline samples for classification
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

    su = sorted(uv3(spline3(t/200.)) for t in range(201))

    def rim_v_at(u):
        import bisect
        i = bisect.bisect_left([p[0] for p in su], u)
        if i == 0: return su[0][1]
        if i >= len(su): return su[-1][1]
        t = (u - su[i-1][0]) / (su[i][0] - su[i-1][0] + 1e-12)
        return su[i-1][1]*(1-t) + su[i][1]*t

    for vi in my:
        u, v = uv3(to_local(verts[vi]))
        if v < -1.19:
            arc.append((u, v))
        elif u < 0.4:
            A.append((u, v))
        elif u > 89.6:
            B.append((u, v))
        elif abs(v - rim_v_at(u)) < 0.02:
            spl.append((u, v))
    # CLEAN rim from STEP geometry (prototype: uniform steps; the
    # Rust impl uses the cached edge discretization bit-exact)
    ARC_N = 17
    arc = [(90.0 * k / (ARC_N - 1), -1.2) for k in range(ARC_N)]
    A = [(0.0, -0.3123)]              # lineA top -> (0,-1.2)=arc[0]
    B = [(90.0, -1.2), (90.0, -0.1475)]  # lineB: bottom corner + top
    spl = [uv3(spline3(1.0 - t / 40.0)) for t in range(41)]
    spl = [q for q in spl]
    # dedup consecutive
    spl = [q for k, q in enumerate(spl)
           if k == 0 or math.hypot(
               (q[0]-spl[k-1][0])*R*math.pi/180,
               q[1]-spl[k-1][1]) > 1e-6]
    rim = arc + B + spl + A   # spl is already u-descending
    print(f"rim closed chain: {len(rim)} pts "
          f"(A{len(A)} arc{len(arc)} B{len(B)} spl{len(spl)})")

    # lattice
    n_u, n_v = 12, 64
    cols = [90.0 * i / n_u for i in range(1, n_u)]
    rows = [-1.2 + 1.0628 * j / n_v for j in range(1, n_v)]
    grid = {}
    for u in cols:
        for v in rows:
            if v < rim_v_at(u) - 1e-9 and v > -1.2 + 1e-9:
                grid[(u, v)] = True
    lat = sorted(grid)
    print(f"lattice: {len(lat)} pts")

    # lattice hull: walk border points CCW
    bycol = defaultdict(list)
    for (u, v) in lat:
        bycol[u].append(v)
    colus = sorted(bycol)
    tris = []

    def add(a, b, c):
        tris.append((a, b, c))

    def gi(u, v):
        return ("L", u, v)

    hull = []
    hull += [gi(u, min(bycol[u])) for u in colus]           # bottom L→R
    hull += [gi(colus[-1], v) for v in sorted(bycol[colus[-1]])][1:-1]  # right up
    hull += [gi(u, max(bycol[u])) for u in reversed(colus)]  # top R→L
    hull += [gi(colus[0], v) for v in reversed(sorted(bycol[colus[0]]))][1:-1]  # left DOWN
    print(f"hull: {len(hull)} pts")

    # interior grid quads: between adjacent columns, adjacent rows

    for cu in range(len(colus) - 1):
        u1_, u2_ = colus[cu], colus[cu + 1]
        vs1 = sorted(bycol[u1_])
        vs2 = sorted(bycol[u2_])
        i = j = 0
        while i + 1 < len(vs1) or j + 1 < len(vs2):
            if j + 1 >= len(vs2):
                add(gi(u1_, vs1[i+1]), gi(u1_, vs1[i]), gi(u2_, vs2[j])); i += 1
            elif i + 1 >= len(vs1):
                add(gi(u2_, vs2[j]), gi(u2_, vs2[j+1]), gi(u1_, vs1[i])); j += 1
            elif vs1[i+1] - vs1[i] <= vs2[j+1] - vs2[j]:
                add(gi(u1_, vs1[i+1]), gi(u1_, vs1[i]), gi(u2_, vs2[j])); i += 1
            else:
                add(gi(u2_, vs2[j]), gi(u2_, vs2[j+1]), gi(u1_, vs1[i])); j += 1
    print(f"interior tris: {len(tris)}")

    # annulus zipper: closed chains cut at aligned start points:
    # P = rim keys + first (nr+1), Q = hull keys + first (nh+1)
    # open two-pointer between them; cut edge (P0,Q0) used 2x.
    # annulus zipper: angular progress around the hull centroid
    # (both chains star-shaped w.r.t. it -> no crossing)
    cx = sum(h[1] for h in hull) / len(hull) * R * math.pi / 180
    cy = sum(h[2] for h in hull) / len(hull)

    def cumangk(ch):
        out = [0.0]
        for k in range(1, len(ch)):
            a0 = math.atan2(ch[k-1][2] - cy,
                            ch[k-1][1]*R*math.pi/180 - cx)
            a1 = math.atan2(ch[k][2] - cy,
                            ch[k][1]*R*math.pi/180 - cx)
            d = a1 - a0
            if d < -math.pi:
                d += 2 * math.pi
            elif d > math.pi:
                d -= 2 * math.pi
            if d < 0:
                d = 0.0   # radial tie / tiny backward eps: no progress
            out.append(out[-1] + d)
        return out

    nr = len(rim)
    k0 = 0
    for k in range(nr):
        if rim[k][0] < 0.5 and rim[k][1] < -1.15:
            k0 = k
            break
    rim = rim[k0:] + rim[:k0]
    P = [("R", p[0], p[1]) for p in rim]
    P.append(P[0])
    Q = list(hull)
    Q.append(Q[0])
    m_, n_ = len(P) - 1, len(Q) - 1
    rl, hl = cumangk(P), cumangk(Q)
    Lr, Lh = rl[-1], hl[-1]
    i = j = 0
    while i < m_ or j < n_:
        if j >= n_:
            add(P[i], P[i+1], Q[n_])
            i += 1
        elif i >= m_:
            add(Q[j+1], Q[j], P[m_])
            j += 1
        else:
            da = (rl[i+1] - rl[i]) / Lr
            db = (hl[j+1] - hl[j]) / Lh
            if da <= db:
                add(P[i], P[i+1], Q[j])
                i += 1
            else:
                add(Q[j+1], Q[j], P[i])
                j += 1
    print(f"total tris: {len(tris)}")

    # ---- VERIFY ----
    def key_of(t):
        return t
    edge_use = defaultdict(int)
    for t in tris:
        ks = [key_of(x) for x in t]
        for x, y in ((ks[0], ks[1]), (ks[1], ks[2]), (ks[2], ks[0])):
            edge_use[(x, y) if x > y else (y, x)] += 1
    rim_keys = {("R", p[0], p[1]) for p in rim}
    bad = {e: n for e, n in edge_use.items()
           if n != 2 and not (e[0] in rim_keys or e[1] in rim_keys)}
    print(f"non-rim edges usage != 2: {len(bad)}")
    for e, n in list(bad.items())[:8]:
        print("   ", e, n)
    used = set()
    for t in tris:
        used.update(t)
    unused_rim = [p for p in rim if ("R", p[0], p[1]) not in used]
    unused_lat = [p for p in lat if gi(*p) not in used]
    print(f"unused rim pts: {len(unused_rim)}, unused lattice: "
          f"{len(unused_lat)}")

    # area
    def tri_area_3d(t):
        import itertools
        pts = [uv_to_3d(x[1], x[2]) for x in t]
        a, b, c = pts
        ab = tuple(b[k]-a[k] for k in range(3))
        ac = tuple(c[k]-a[k] for k in range(3))
        cr = cross(ab, ac)
        return 0.5 * math.sqrt(sum(x*x for x in cr))
    tot = sum(tri_area_3d(t) for t in tris)
    # rim polygon area (3D fan from centroid)
    c3 = uv_to_3d(45, -0.7)
    pt = sum(tri_area_3d((("R", rim[k][0], rim[k][1]),
                          ("R", rim[(k+1) % nr][0], rim[(k+1) % nr][1]),
                          ("X", 45, -0.7))) for k in range(nr))
    print(f"3D area: mesh {tot:.5f} vs rim-polygon {pt:.5f} "
          f"({100*tot/pt:.2f}%)")

    # fold simulation: same-face adjacent dihedrals
    edge_tris = defaultdict(list)
    for ti, t in enumerate(tris):
        ks = [key_of(x) for x in t]
        for x, y in ((ks[0], ks[1]), (ks[1], ks[2]), (ks[2], ks[0])):
            edge_tris[(x, y) if x > y else (y, x)].append(ti)
    folds = 0
    for e, ts_ in edge_tris.items():
        if len(ts_) != 2:
            continue
        n1 = tri_normal_3d(tris[ts_[0]])
        n2 = tri_normal_3d(tris[ts_[1]])
        if n1 and n2:
            ang = math.degrees(math.acos(max(-1, min(1,
                sum(n1[k]*n2[k] for k in range(3))))))
            if ang > 170:
                folds += 1
    print(f"same-face fold pairs (>170 deg): {folds}")

    # min angle
    worst = 90.0
    for t in tris:
        pts = [uv_to_3d(x[1], x[2]) for x in t]
        for i in range(3):
            a, b, c = pts[i], pts[(i+1) % 3], pts[(i+2) % 3]
            v1 = tuple(b[k]-a[k] for k in range(3))
            v2 = tuple(c[k]-a[k] for k in range(3))
            n1 = math.sqrt(sum(x*x for x in v1))
            n2 = math.sqrt(sum(x*x for x in v2))
            if n1 < 1e-12 or n2 < 1e-12:
                worst = 0.0; continue
            cs = sum(v1[k]*v2[k] for k in range(3)) / (n1*n2)
            worst = min(worst, math.degrees(math.acos(
                max(-1, min(1, cs)))))
    print(f"min 3D triangle angle: {worst:.2f} deg")


def tri_normal_3d(t):
    pts = [uv_to_3d(x[1], x[2]) for x in t]
    a, b, c = pts
    ab = tuple(b[k]-a[k] for k in range(3))
    ac = tuple(c[k]-a[k] for k in range(3))
    cr = cross(ab, ac)
    l = math.sqrt(sum(x*x for x in cr))
    if l < 1e-15:
        return None
    return tuple(x/l for x in cr)


if __name__ == "__main__":
    main()
