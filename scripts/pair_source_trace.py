#!/usr/bin/env python3
"""Session-62: trace after-merge pair triangles back to their per-face
source meshes (by coordinates), across ALL GEAR faces.

For each >170° same-face pair (f1/f12) in p0_d-after-merge:
  - take both triangles' 3D vertex coords
  - find matching triangles (within eps) in every /tmp/faceobjs/*.obj
  - report which faces contributed each triangle of the pair
Reveals whether pairs are intra-face (impossible — per-face census clean)
or assembled from TWO different faces / duplicate emissions.
"""
import sys, math, glob
from collections import defaultdict

sys.path.insert(0, '/home/z/my-project/scripts')
from stage_pair_track import load_obj, load_fmap

MERGE_OBJ = '/tmp/stages/p0_d-after-merge.obj'
MERGE_FMAP = '/tmp/stages/p0_d-after-merge.fmap'
FACE_DIR = '/tmp/faceobjs'
EPS = 1e-6
FACES = {1, 12}

v, tris = load_obj(MERGE_OBJ)
fm = load_fmap(MERGE_FMAP)

# per-face meshes (pass p1 overwrote pass p0 in FACE_DIR — same geometry
# for a deterministic pipeline; verified separately)
face_meshes = {}
for path in sorted(glob.glob(f'{FACE_DIR}/*.obj')):
    name = path.split('/')[-1].replace('.obj', '')
    fv, ft = load_obj(path)
    # index by rounded coordinate for fast lookup
    lut = {}
    for i, p in enumerate(fv):
        lut.setdefault(tuple(round(c, 7) for c in p), []).append(i)
    face_meshes[name] = (lut, fv, ft)

def find_in_face(lut, fv, ft, tri_pts):
    """Find triangles in face mesh matching all 3 coords (any order)."""
    idx_sets = []
    for p in tri_pts:
        key = tuple(round(c, 7) for c in p)
        idx_sets.append(lut.get(key, []))
    if any(len(s) == 0 for s in idx_sets):
        return []
    hits = []
    for a in idx_sets[0]:
        for b in idx_sets[1]:
            for c in idx_sets[2]:
                if len({a, b, c}) < 3:
                    continue
                for ti, t in enumerate(ft):
                    if {t[0], t[1], t[2]} == {a, b, c}:
                        hits.append((ti, t))
    return hits

# collect pairs (same census as before)
edge_map = defaultdict(list)
for ti, tri in enumerate(tris):
    a, b, c = tri
    for v0, v1 in ((a, b), (b, c), (c, a)):
        key = (v0, v1) if v0 < v1 else (v1, v0)
        edge_map[key].append((ti, v0, v1))

def nrm(tri):
    a3, b3, c3 = (v[i] for i in tri)
    e1 = [b3[i]-a3[i] for i in range(3)]
    e2 = [c3[i]-a3[i] for i in range(3)]
    n = (e1[1]*e2[2]-e1[2]*e2[1], e1[2]*e2[0]-e1[0]*e2[2], e1[0]*e2[1]-e1[1]*e2[0])
    ln = math.sqrt(sum(x*x for x in n))
    return tuple(x/ln for x in n) if ln > 1e-30 else None

pairs = []
for edge, lst in edge_map.items():
    if len(lst) != 2:
        continue
    (t0, _, _), (t1, _, _) = lst
    f0, f1 = fm.get(t0), fm.get(t1)
    if f0 != f1 or f0 not in FACES:
        continue
    n0, n1 = nrm(tris[t0]), nrm(tris[t1])
    dot = sum(n0[i]*n1[i] for i in range(3))
    if math.degrees(math.acos(max(-1, min(1, dot)))) <= 170:
        continue
    pairs.append((edge, t0, t1, f0))

print(f'{len(pairs)} pairs to trace')
for (edge, t0, t1, fid) in pairs:
    print(f'\nf{fid} edge{edge} T{t0}{tris[t0]} T{t1}{tris[t1]}')
    for t in (t0, t1):
        pts = [v[i] for i in tris[t]]
        found = []
        for name, (lut, fv, ft) in face_meshes.items():
            hits = find_in_face(lut, fv, ft, pts)
            if hits:
                found.append((name, len(hits), hits[0][1]))
        tag = ' | '.join(f'{n}×{c}' for n, c, _ in found) if found else 'NOT FOUND in any face mesh'
        print(f'  T{t}: {tag}')
