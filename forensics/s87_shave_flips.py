#!/usr/bin/env python3
"""s87: degree-shaving feasibility on the HOUSING f3 alt mesh — can 2
local convex edge flips at the max-degree vertex bring alt_max 108
below the fan's 107 while keeping every guard contract?"""
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "forensics/s87_replay/brep47598_f3_Plane_altC.txt"
FAN_MAX = 107

pts = []
tris = []
for line in open(path):
    if line.startswith("p "):
        _, u, v = line.split()
        pts.append((float(u), float(v)))
    elif line.startswith("t "):
        _, a, b, c = line.split()
        tris.append((int(a), int(b), int(c)))
m = len(pts)
n = len(tris)
print(f"m={m} tris={n}")

# degrees (tri count per vertex)
deg = [0] * m
for t in tris:
    for v in t:
        deg[v] += 1
order = sorted(range(m), key=lambda i: -deg[i])
print("top degrees:", [(i, deg[i]) for i in order[:8]])
vmax = order[0]
print(f"max vertex v*={vmax} deg={deg[vmax]} (fan max={FAN_MAX}, need <= {FAN_MAX-1})")

# edge -> tris; rim set
edge_tris = defaultdict(list)
for ti, t in enumerate(tris):
    for k in range(3):
        a, b = t[k], t[(k + 1) % 3]
        edge_tris[(min(a, b), max(a, b))].append(ti)
rim = set()
for i in range(m):
    j = (i + 1) % m
    rim.add((min(i, j), max(i, j)))

interior = {e: tis for e, tis in edge_tris.items() if e not in rim and len(tis) == 2}
print(f"interior edges (2 tris, non-rim): {len(interior)}")
multi = [(e, len(tis)) for e, tis in edge_tris.items() if len(tis) > 2]
print(f"edges with >2 tris (non-manifold): {len(multi)} {multi[:5]}")

def cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def try_flip(e):
    """Return (ok, new_tris_pair, delta) for flipping interior edge e."""
    tis = interior[e]
    t1, t2 = tris[tis[0]], tris[tis[1]]
    a, b = e
    # apexes
    c = next(x for x in t1 if x != a and x != b)
    d = next(x for x in t2 if x != a and x != b)
    # quad cyclic order: walk t1 as a->?->b->? then t2
    # convexity: a and c on opposite sides of line b-d? Standard check:
    # the quad (c, d, ...) must be convex: both diagonals' apexes on
    # opposite sides.
    s_ac = cross(pts[b], pts[c], pts[d])  # side of c/d relative... use quad a-c-b-d? do direct:
    # flip edge (a,b) -> edge (c,d) valid iff quad c-a-d-b (cyclic around) is strictly convex
    # i.e. a,b on opposite sides of cd AND c,d on opposite sides of ab
    s1 = cross(pts[a], pts[c], pts[d])
    s2 = cross(pts[b], pts[c], pts[d])
    s3 = cross(pts[c], pts[a], pts[b])
    s4 = cross(pts[d], pts[a], pts[b])
    eps = 1e-12
    if not (s1 * s2 < -eps and s3 * s4 < -eps):
        return None
    # new tris with winding matching the originals: t1=(a,?,b) or (a,b,?)
    # normalize: ensure replacement tris oriented like t1/t2
    def orient(tri, want_sign):
        s = cross(pts[tri[0]], pts[tri[1]], pts[tri[2]])
        if (s > 0) == (want_sign > 0):
            return tri
        return (tri[0], tri[2], tri[1])
    # t1 contains a,b,c; t2 contains a,b,d. New: (a, c, d)?? no —
    # quad is c-a-d-b? Actually the two new tris are (c, d, ...) hmm:
    # new tris cover the quad with diagonal (c,d): (c, a, d) and (c, d, b)
    s_t1 = cross(pts[t1[0]], pts[t1[1]], pts[t1[2]])
    n1 = orient((c, a, d), s_t1)
    n2 = orient((c, d, b), s_t1)
    # degenerate check
    if abs(cross(pts[n1[0]], pts[n1[1]], pts[n1[2]])) < 1e-18:
        return None
    if abs(cross(pts[n2[0]], pts[n2[1]], pts[n2[2]])) < 1e-18:
        return None
    return (tis[0], tis[1], n1, n2, (c, d))

# candidate flips incident to v*
cands = []
for e, tis in interior.items():
    if vmax in e:
        r = try_flip(e)
        if r:
            ti1, ti2, n1, n2, _ = r
            # apexes gain +1 each: c,d
            c, d = n1[0], n1[2] if n1[1] != vmax else None
            cands.append((e, r))
print(f"\nflippable interior edges at v*: {len(cands)}")
for e, r in cands[:10]:
    ti1, ti2, n1, n2, _ = r
    c = next(x for x in n1 if x not in e)
    d = next(x for x in n2 if x not in e)
    print(f"  flip {e}: +1 to c={c}(deg {deg[c]}) d={d}(deg {deg[d]}); v* {deg[vmax]}->{deg[vmax]-1}")

# double-flip search: two distinct v*-edges, all +1 vertices stay <= FAN_MAX-2
print("\ndouble-flip search (v* -> <= 106, all affected <= 106):")
found = 0
for i in range(len(cands)):
    for j in range(i + 1, len(cands)):
        (e1, r1), (e2, r2) = cands[i], cands[j]
        if e1 == e2:
            continue
        apexes = set()
        for r in (r1, r2):
            ti1, ti2, n1, n2, _ = r
            for x in set(n1) | set(n2):
                if x not in r[4] and x != vmax:
                    pass
        # collect c,d per flip
        cd = []
        for (e, r) in ((e1, r1), (e2, r2)):
            ti1, ti2, n1, n2, (c, d) = r
            cd.extend([c, d])
        # simulate degree deltas
        dd = defaultdict(int)
        dd[vmax] = -2
        for x in cd:
            dd[x] += 1
        ok = all(deg[v] + delta <= FAN_MAX - 1 for v, delta in dd.items())
        if ok:
            found += 1
            if found <= 6:
                print(f"  OK: flip {e1} + {e2}; deltas {dict(dd)}")
print(f"valid double-flips: {found}")
