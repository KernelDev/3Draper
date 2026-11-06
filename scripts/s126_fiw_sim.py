#!/usr/bin/env python3
"""s126: replicate fix_inconsistent_winding's BFS exactly (Rust source:
watertight.rs 1306-1425) on the after-gapfill mesh of b10212 (p21),
reproduce the 1772-flip set, and identify WHICH neighbor caused the
t103(pair T1865) flip — reconstructing the cascade path to its origin.
"""
import os
from collections import defaultdict, deque

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDIR = f"{BASE}/forensics/s126/stages2"

def load(path):
    vs, fs = [], []
    fmap = {}
    for l in open(path):
        if l.startswith("v "):
            _, x, y, z = l.split()
            vs.append((float(x), float(y), float(z)))
        elif l.startswith("f "):
            _, a, b, c = l.split()
            fs.append((int(a)-1, int(b)-1, int(c)-1))
    for l in open(path.replace(".obj", ".fmap")):
        _, t, fid = l.split()
        fmap[int(t)] = int(fid)
    return vs, fs, fmap

vs, fs, fmap = load(f"{SDIR}/p21_b10212-d-after-gapfill.obj")
n = len(fs)
print(f"after-gapfill: {n} tris, {len(vs)} verts")

# edge -> tris (insertion order like Rust HashMap — but adjacency built
# from SORTED edges, so sort)
edge_to_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(fs):
    for v0, v1 in ((a, b), (b, c), (c, a)):
        key = (min(v0, v1), max(v0, v1))
        edge_to_tris[key].append(ti)

adjacency_edges = sorted(edge_to_tris.keys())
tri_adj = defaultdict(list)
for edge in adjacency_edges:
    tris = edge_to_tris[edge]
    if len(tris) == 2:
        tri_adj[tris[0]].append((tris[1], edge[0], edge[1]))
        tri_adj[tris[1]].append((tris[0], edge[0], edge[1]))

def edge_dir(tri, ev0, ev1):
    """get_edge_direction: returns the FROM vertex of the directed edge
    as traversed by tri (a,b,c): edges (a,b),(b,c),(c,a)."""
    a, b, c = tri
    for x, y in ((a, b), (b, c), (c, a)):
        if (x == ev0 and y == ev1) or (x == ev1 and y == ev0):
            return x
    return None

def normal(vs, tri):
    a, b, c = (vs[i] for i in tri)
    ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
    return (uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx)

visited = [False]*n
flips = {}   # ti -> (from_tri, reason)
for start in range(n):
    if visited[start]:
        continue
    q = deque([start])
    visited[start] = True
    while q:
        ti = q.popleft()
        tri = fs[ti]
        neigh = tri_adj.get(ti, ())
        for nidx, ev0, ev1 in neigh:
            if visited[nidx]:
                continue
            visited[nidx] = True
            ntri = fs[nidx]
            rd = edge_dir(tri, ev0, ev1)
            nd = edge_dir(ntri, ev0, ev1)
            if rd is None or nd is None:
                q.append(nidx)
                continue
            consistent = rd != nd
            reason = None
            if not consistent:
                reason = "dir"
            else:
                n0 = normal(vs, tri)
                n1 = normal(vs, ntri)
                d = n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2]
                l0 = (n0[0]**2+n0[1]**2+n0[2]**2)**.5
                l1 = (n1[0]**2+n1[1]**2+n1[2]**2)**.5
                if l0 > 1e-15 and l1 > 1e-15:
                    ang = (max(-1.0, min(1.0, d/(l0*l1)))).__import__(
                        "math").acos() if False else None
                    import math
                    ang = math.degrees(math.acos(max(-1.0, min(1.0, d/(l0*l1)))))
                    if ang > 170.0:
                        reason = f"normals {ang:.1f}"
            if reason:
                # flip neighbor (swap last two)
                fs[nidx] = (ntri[0], ntri[2], ntri[1])
                flips[nidx] = (ti, reason)
            q.append(nidx)

print(f"simulated flips: {len(flips)} (Rust reported 1772)")

# compare with actual after-winding
_, fs_w, _ = load(f"{SDIR}/p21_b10212-d-after-winding.obj")
mismatch = sum(1 for a, b in zip(fs, fs_w) if a != b)
print(f"sim vs actual after-winding mismatches: {mismatch}")

# the pair triangle t103: which vertex positions?
# anchors at after-gapfill: 53->1379 90->1052 91->1051 54->1380
A = {53: 1379, 90: 1052, 91: 1051, 54: 1380}
t103 = [i for i, t in enumerate(fs) if set(t) == {A[53], A[90], A[91]}]
t104 = [i for i, t in enumerate(fs) if set(t) == {A[53], A[54], A[90]}]
print(f"\nt103 idx {t103}, t104 idx {t104} (post-sim winding)")
for i in t103:
    print(f"  t{i}: sim={fs[i]} actual={fs_w[i]}")
    if i in flips:
        cause, reason = flips[i]
        print(f"    FLIPPED while visiting t{cause} (face {fmap.get(cause)}),"
              f" reason={reason}; t{cause} winding={fs[cause]}")
        print(f"    shared edge via tri_adj of t{cause}:")
# walk the flip chain backwards from t103's cause
if t103 and t103[0] in flips:
    chain = []
    cur = t103[0]
    while cur in flips:
        cause, reason = flips[cur]
        chain.append((cur, fmap.get(cur), cause, fmap.get(cause), reason))
        cur = cause
        if len(chain) > 12:
            break
    print("\n  flip chain (deepest last):")
    for c in chain:
        print(f"    t{c[0]} (f{c[1]}) <- caused by t{c[2]} (f{c[3]}): {c[4]}")

# ── post-analysis: roots (trigger-2), pre-FIW consistency ─────────
print("\n=== FIW roots (trigger-2 normals) ===")
roots = [(ti, cause, r) for ti, (cause, r) in flips.items()
         if r.startswith("normals")]
print(f"trigger-2 roots: {len(roots)}")
for ti, cause, r in sorted(roots)[:15]:
    print(f"  t{ti} (f{fmap.get(ti)}) <- t{cause} (f{fmap.get(cause)}): {r}")

# pre-FIW mesh: same-direction usage-2 edges (trigger-1 candidates
# BEFORE any flip — nonzero means real pre-existing inconsistency)
vs2, fs2, fmap2 = load(f"{SDIR}/p21_b10212-d-after-gapfill.obj")
e2 = defaultdict(list)
for ti, (a, b, c) in enumerate(fs2):
    for v0, v1 in ((a, b), (b, c), (c, a)):
        e2[(min(v0, v1), max(v0, v1))].append(ti)
same_dir = 0
anti_pairs = 0
for (ea, eb), tris in e2.items():
    if len(tris) != 2:
        continue
    d0 = edge_dir(fs2[tris[0]], ea, eb)
    d1 = edge_dir(fs2[tris[1]], ea, eb)
    if d0 is None or d1 is None:
        continue
    if d0 == d1:
        same_dir += 1
    else:
        n0 = normal(vs2, fs2[tris[0]])
        n1 = normal(vs2, fs2[tris[1]])
        d = n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2]
        l0 = (n0[0]**2+n0[1]**2+n0[2]**2)**.5
        l1 = (n1[0]**2+n1[1]**2+n1[2]**2)**.5
        import math
        if l0 > 1e-15 and l1 > 1e-15:
            ang = math.degrees(math.acos(max(-1.0, min(1.0, d/(l0*l1)))))
            if ang > 170.0:
                anti_pairs += 1
print(f"\npre-FIW (after-gapfill): same-direction usage-2 edges: {same_dir}")
print(f"pre-FIW: consistent-dir + >170 normals pairs (trigger-2 candidates): {anti_pairs}")

# ── breakdown of the 832 same-direction edges ────────────────────
print("\n=== same-direction edge breakdown ===")
from collections import Counter
sf = Counter()
xf = Counter()
facedges = Counter()
for (ea, eb), tris in e2.items():
    if len(tris) != 2:
        continue
    d0 = edge_dir(fs2[tris[0]], ea, eb)
    d1 = edge_dir(fs2[tris[1]], ea, eb)
    if d0 is None or d1 is None or d0 != d1:
        continue
    f0, f1 = fmap2.get(tris[0], -1), fmap2.get(tris[1], -1)
    if f0 == f1:
        sf[f0] += 1
    else:
        xf[(min(f0, f1), max(f0, f1))] += 1
    facedges[f0] += 1
    facedges[f1] += 1
print("same-face conflicts by face:", dict(sf))
print("cross-face conflicts by pair:", dict(xf))
print("per-face involvement:", dict(facedges.most_common(10)))

# ── A/B: FIW with trigger-2 DISABLED ─────────────────────────────
print("\n=== SIM: FIW trigger-2 disabled ===")
vs3, fs3, fmap3 = load(f"{SDIR}/p21_b10212-d-after-gapfill.obj")
n3 = len(fs3)
e3 = defaultdict(list)
for ti, (a, b, c) in enumerate(fs3):
    for v0, v1 in ((a, b), (b, c), (c, a)):
        e3[(min(v0, v1), max(v0, v1))].append(ti)
adj3 = defaultdict(list)
for edge in sorted(e3.keys()):
    tris = e3[edge]
    if len(tris) == 2:
        adj3[tris[0]].append((tris[1], edge[0], edge[1]))
        adj3[tris[1]].append((tris[0], edge[0], edge[1]))
visited3 = [False]*n3
flips3 = {}
for start in range(n3):
    if visited3[start]:
        continue
    q = deque([start])
    visited3[start] = True
    while q:
        ti = q.popleft()
        tri = fs3[ti]
        for nidx, ev0, ev1 in adj3.get(ti, ()):
            if visited3[nidx]:
                continue
            visited3[nidx] = True
            ntri = fs3[nidx]
            rd = edge_dir(tri, ev0, ev1)
            nd = edge_dir(ntri, ev0, ev1)
            if rd is None or nd is None:
                q.append(nidx)
                continue
            if rd == nd:  # trigger 1 only
                fs3[nidx] = (ntri[0], ntri[2], ntri[1])
                flips3[nidx] = ti
            q.append(nidx)
print(f"flips (trigger-1 only): {len(flips3)}")
# census on the result
same_cnt = 0
fold_pairs = []
EFF_TOL = 0.0161
for (ea, eb), tris in e3.items():
    if len(tris) != 2:
        continue
    d0 = edge_dir(fs3[tris[0]], ea, eb)
    d1 = edge_dir(fs3[tris[1]], ea, eb)
    if d0 is None or d1 is None:
        continue
    n0 = normal(vs3, fs3[tris[0]])
    n1 = normal(vs3, fs3[tris[1]])
    d = n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2]
    l0 = (n0[0]**2+n0[1]**2+n0[2]**2)**.5
    l1 = (n1[0]**2+n1[1]**2+n1[2]**2)**.5
    import math
    ang = math.degrees(math.acos(max(-1.0, min(1.0, d/(l0*l1))))) if l0>1e-15 and l1>1e-15 else 0.0
    if d0 == d1:
        same_cnt += 1
        print(f"  WF pair: t{tris[0]}/t{tris[1]} faces=({fmap3.get(tris[0])},{fmap3.get(tris[1])})")
    elif ang > 170.0:
        # fold pair (honest): compute h over shared edge
        elen = math.dist(vs3[ea], vs3[eb])
        def tarea(t):
            a, b, c = (vs3[i] for i in t)
            ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
            vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
            cx, cy, cz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
            return 0.5*math.sqrt(cx*cx+cy*cy+cz*cz)
        h0 = 2*tarea(fs3[tris[0]])/elen if elen > 1e-12 else 9e9
        h1 = 2*tarea(fs3[tris[1]])/elen if elen > 1e-12 else 9e9
        real = max(h0, h1) >= EFF_TOL
        fold_pairs.append((tris[0], tris[1], fmap3.get(tris[0]), fmap3.get(tris[1]), ang, h0, h1, real))
print(f"result: same-direction (WF) edges: {same_cnt}")
print(f"fold pairs (FOLD-OVER class): {len(fold_pairs)}")
for t0, t1, f0, f1, ang, h0, h1, real in fold_pairs:
    print(f"  f({f0},{f1}) t{t0}/t{t1} ang={ang:.1f} h=({h0:.4f},{h1:.4f}) "
          f"{'REAL' if real else 'subtol'}")
