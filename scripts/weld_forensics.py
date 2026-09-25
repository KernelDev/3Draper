#!/usr/bin/env python3
"""Session-56 forensics: correlate WELD[P*] dump pairs with the f198-family
region of drill_top HOUSING_MIRROR and verify the collapse mechanism.

Inputs (produced by):
  DRAPPER_CHAIN_COMPLEMENT=1 DRAPPER_DUMP_WELDS=1 \
  DRAPPER_DUMP_FINAL_OBJS=<dir> fold_face_probe test/drill_top.stp 4

Usage:
  python3 scripts/weld_forensics.py <obj_dir> <weld_log> [face_ids...]
"""
import re
import sys
from collections import defaultdict
from pathlib import Path

obj_dir = Path(sys.argv[1])
weld_log = Path(sys.argv[2])
face_ids = set(int(x) for x in sys.argv[3:]) or {198, 200, 202, 204, 206}

# ── Load the final OBJ + fmap ────────────────────────────────────────
obj_file = next(obj_dir.glob("brep4_*.obj"))
fmap_file = obj_file.with_suffix(".fmap")

verts = []
tris = []
for line in obj_file.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))

fid_of_tri = {}
for line in fmap_file.read_text().splitlines():
    if line.startswith("t "):
        _, ti, fid = line.split()
        fid_of_tri[int(ti)] = int(fid)

print(f"OBJ: {len(verts)} verts, {len(tris)} tris, fmap: {len(fid_of_tri)}")

# ── Per-face vertex sets + edge usage for the target faces ──────────
face_verts = defaultdict(set)
tri_face = []
for ti, (a, b, c) in enumerate(tris):
    fid = fid_of_tri.get(ti, -1)
    tri_face.append(fid)
    face_verts[fid].update((a, b, c))

# Same-face usage-N edge census on the FINAL mesh (post-weld)
edge_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(tris):
    for e in ((a, b), (b, c), (c, a)):
        key = (min(e), max(e))
        edge_tris[key].append(ti)

usage_hist = defaultdict(int)
sameface_u4 = defaultdict(list)   # fid -> [edges]
for (e, tl) in edge_tris.items():
    n = len(tl)
    usage_hist[n] += 1
    if n >= 3:
        fids = {tri_face[t] for t in tl}
        if len(fids) == 1:
            sameface_u4[list(fids)[0]].append(e)

print("\nedge usage histogram:", dict(sorted(usage_hist.items())))
for fid in sorted(face_ids):
    nv = len(face_verts[fid])
    u4 = len(sameface_u4.get(fid, []))
    print(f"  face {fid}: {nv} verts, {sum(1 for t,f in enumerate(tri_face) if f==fid)} tris, same-face usage>=3 edges: {u4}")
tot_u4 = sum(len(v) for v in sameface_u4.values())
print(f"TOTAL same-face usage>=3 edges (all faces): {tot_u4}")

# ── Spatial hash of target-face vertices for weld position lookup ───
cell = 0.05
grid = defaultdict(list)
for fid in face_ids:
    for vi in face_verts[fid]:
        x, y, z = verts[vi]
        grid[(int(x // cell), int(y // cell), int(z // cell))].append((vi, fid))

def locate(px, py, pz, eps=1e-4):
    """Return (vertex_index, face_id) if (px,py,pz) sits on a target-face
    vertex within eps, else None."""
    cx, cy, cz = int(px // cell), int(py // cell), int(pz // cell)
    best = None
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for (vi, fid) in grid.get((cx + dx, cy + dy, cz + dz), ()):
                    x, y, z = verts[vi]
                    d2 = (x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2
                    if d2 <= eps * eps and (best is None or d2 < best[0]):
                        best = (d2, vi, fid)
    return (best[1], best[2]) if best else None

# ── Parse the weld log (LAST run only: keep lines after the last
#    "Loading" / re-parse; positions repeat between runs) ────────────
pat = re.compile(
    r"WELD\[(P\d)\] (\d+)->(\d+) d=([0-9.e+-]+) "
    r"p=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\) q=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)"
)
welds = []
for line in weld_log.read_text(errors="replace").splitlines():
    m = pat.search(line)
    if m:
        welds.append({
            "pass": m.group(1),
            "src": int(m.group(2)), "dst": int(m.group(3)),
            "d": float(m.group(4)),
            "p": (float(m.group(5)), float(m.group(6)), float(m.group(7))),
            "q": (float(m.group(8)), float(m.group(9)), float(m.group(10))),
        })
print(f"\nweld dump lines: {len(welds)} ({defaultdict(int, [ (w['pass'], 0) for w in welds ])})")

# Only the LAST pass-1 dump per (src,dst) pair — the second triangulation
# run is the one that produced the dumped OBJ (vertex indices differ, so
# match by POSITION).
# Split into two runs by the duplicate position sequence:
# take the second half if the first half's positions all reappear.
def run_split(ws):
    half = len(ws) // 2
    first = {(round(w["p"][0], 5), round(w["p"][1], 5), round(w["p"][2], 5)) for w in ws[:half]}
    second = ws[half:]
    in_first = sum(
        1 for w in second
        if (round(w["p"][0], 5), round(w["p"][1], 5), round(w["p"][2], 5)) in first
    )
    return second if in_first > len(second) * 0.5 else ws

welds = run_split(welds)
print(f"final-run welds: {len(welds)}")

# ── Correlate: welds whose p or q lands on a target-face vertex ─────
hit_p = defaultdict(int)
hit_q = defaultdict(int)
hit_welds = []
for w in welds:
    lp = locate(*w["p"])
    lq = locate(*w["q"])
    if lp:
        hit_p[(w["pass"], lp[1])] += 1
    if lq:
        hit_q[(w["pass"], lq[1])] += 1
    if lp or lq:
        hit_welds.append((w, lp, lq))

print(f"\nwelds touching target faces: {len(hit_welds)}")
print("  by pass (p on face):", dict(hit_p))
print("  by pass (q on face):", dict(hit_q))

# Distances of the face-touching welds
ds = [w["d"] for (w, _, _) in hit_welds]
if ds:
    ds.sort()
    print(f"  distances: min={ds[0]:.2e} med={ds[len(ds)//2]:.2e} max={ds[-1]:.2e}")

# ── Union-find on POSITIONS: do multiple target-face vertices chain
#    into one root through welds? ────────────────────────────────────
pos_key = lambda p: (round(p[0], 5), round(p[1], 5), round(p[2], 5))
parent = {}
def find(x):
    parent.setdefault(x, x)
    while parent[x] != x:
        parent[x] = parent[parent[x]]
        x = parent[x]
    return x

for w in welds:
    a, b = find(pos_key(w["p"])), find(pos_key(w["q"]))
    if a != b:
        parent[a] = b

roots = defaultdict(set)
for fid in face_ids:
    for vi in face_verts[fid]:
        r = find(pos_key(verts[vi]))
        roots[r].add((vi, fid))

# Roots hosting 2+ distinct target-face vertices = collapsed clusters
multi = {r: members for r, members in roots.items() if len(members) >= 2}
n_collapsed_verts = sum(len(m) for m in multi.values())
print(f"\nunion-find clusters hosting 2+ target-face vertices: {len(multi)}")
print(f"  total target-face vertices in such clusters: {n_collapsed_verts}")
# cross-face (within-family) chains
mixed = {r: m for r, m in multi.items() if len({f for _, f in m}) > 1}
print(f"  clusters mixing DIFFERENT faces: {len(mixed)}")
for r, m in list(multi.items())[:6]:
    print(f"   cluster @ {r}: {sorted(m, key=lambda t: t[1])[:8]}{' ...' if len(m) > 8 else ''}")

# ── Are the same-face usage>=3 edges incident to collapsed roots? ───
if multi:
    collapsed_verts = {vi for m in multi.values() for (vi, _) in m}
    inc = 0
    for fid in face_ids:
        for e in sameface_u4.get(fid, []):
            if e[0] in collapsed_verts or e[1] in collapsed_verts:
                inc += 1
    print(f"\nsame-face usage>=3 edges (target faces) incident to collapsed verts: {inc}")
