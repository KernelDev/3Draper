#!/usr/bin/env python3
"""s114: track the f71 emission triangle (435,454,453) through the
stage OBJ dumps of BREP#27977 (first pass) to localize the stage where
its winding flips."""
import glob
import os

# 1. Load f71 first-pass TRI_INPUT (tri_0127) and extract the target
#    triangle's 3D coordinates. s113 format: 'b/i u v x y z' + triangles
#    section with 't a b c' indices into the combined (boundary+interior+
#    appended) list.
tri_path = 'forensics/s113/tri2_nurbs/tri_0127_Nurbs.txt'
pts = []  # (u, v, x, y, z)
tris = []
mode = None
for ln in open(tri_path):
    w = ln.split()
    if not w:
        continue
    if w[0] == 'type=' or w[0].startswith('type'):
        continue
    if w[0] == 'boundary':
        mode = 'b'
        continue
    if w[0] == 'interior':
        mode = 'i'
        continue
    if w[0] == 'tris':
        mode = 't'
        continue
    if w[0] == 'appended':
        mode = 'a'
        continue
    if mode in ('b', 'i', 'a') and w[0] in ('b', 'i', 'a'):
        # 'b u v x y z'
        if len(w) >= 6:
            pts.append(tuple(float(x) for x in w[1:6]))

    if mode == 't' and w[0] == 't' and len(w) >= 4:
        tris.append((int(w[1]), int(w[2]), int(w[3])))

print(f'f71 tri dump: {len(pts)} pts, {len(tris)} tris')
TARGET = (435, 454, 453)
tgt = None
for t in tris:
    if sorted(t) == sorted(TARGET):
        tgt = t
        break
if tgt is None:
    # maybe indices differ; search area 0.0878 triangle from s113
    print('exact target not found; searching by area 0.0878')
    import math
    def area3(t):
        a = pts[t[0]][2:5]; b = pts[t[1]][2:5]; c = pts[t[2]][2:5]
        ab = [b[i]-a[i] for i in range(3)]
        ac = [c[i]-a[i] for i in range(3)]
        cx = [ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]]
        return 0.5*math.sqrt(sum(x*x for x in cx))
    cand = [(t, area3(t)) for t in tris]
    cand.sort(key=lambda x: abs(x[1]-0.0878))
    print('closest areas:', [(t, round(a, 4)) for t, a in cand[:5]])
    tgt = cand[0][0]
print('target triangle:', tgt, [pts[i] for i in tgt])
target_pos = [tuple(round(pts[i][2 + k], 6) for k in range(3)) for i in tgt]

# 2. Scan stage OBJs of the first pass with matching tris count
def load_obj(path):
    vs = []
    ts = []
    for ln in open(path):
        if ln.startswith('v '):
            w = ln.split()
            vs.append((float(w[1]), float(w[2]), float(w[3])))
        elif ln.startswith('f '):
            w = ln.split()
            ts.append((int(w[1])-1, int(w[2])-1, int(w[3])-1))
    return vs, ts

def load_fmap(path):
    m = {}
    for ln in open(path):
        w = ln.split()
        if len(w) == 2 and w[0] == 't':
            m[int(w[1])] = int(w[2])
    return m

def find_by_pos(vs, pos, tol=1e-5):
    idxs = []
    for i, v in enumerate(vs):
        if all(abs(v[k]-pos[k]) < tol for k in range(3)):
            idxs.append(i)
    return idxs

stages = ['d-after-merge', 'd-after-weld', 'd-after-tj', 'd-after-gapfill',
          'd-after-winding', 'd-after-postweldwind']
print()
print('Tracking target triangle through stages (pass with ~13899 tris):')
for p in sorted(glob.glob('/tmp/my-project/stobjs/p*_d-after-merge.obj')):
    base = os.path.basename(p)
    prefix = base.split('_d-after-')[0]
    vs, ts = load_obj(p)
    if not (13000 < len(ts) < 14500):
        continue
    print(f'== {prefix}: {len(ts)} tris (BREP#27977 first pass candidate)')
    fmap = load_fmap(p.replace('.obj', '.fmap'))
    for stage in stages:
        sp = f'/tmp/my-project/stobjs/{prefix}_{stage}.obj'
        if not os.path.exists(sp):
            continue
        svs, sts = load_obj(sp)
        sfmap = load_fmap(sp.replace('.obj', '.fmap'))
        # locate the three target vertices
        vids = []
        for pos in target_pos:
            cands = find_by_pos(svs, pos)
            vids.append(cands[0] if cands else None)
        if any(v is None for v in vids):
            print(f'   {stage:24s}: vertices not found (welded away?)')
            continue
        sv = set(vids)
        found = []
        for ti, t in enumerate(sts):
            if set(t) == sv:
                found.append((ti, t, sfmap.get(ti)))

        def signed_normal(p3):
            a, b, c = p3
            ab = [b[i]-a[i] for i in range(3)]
            ac = [c[i]-a[i] for i in range(3)]
            return (ab[1]*ac[2]-ab[2]*ac[1],
                    ab[2]*ac[0]-ab[0]*ac[2],
                    ab[0]*ac[1]-ab[1]*ac[0])

        # emission orientation (from the TRI_INPUT 3D columns)
        emit_n = signed_normal([pts[i][2:5] for i in tgt])
        for ti, t, fid in found:
            stage_n = signed_normal([svs[v] for v in t])
            dot = sum(emit_n[k]*stage_n[k] for k in range(3))
            orientation = 'SAME' if dot > 0 else 'FLIPPED(anti)'
            print(f'   {stage:24s}: tri#{ti} {t} face={fid} -> {orientation}')
        if not found:
            print(f'   {stage:24s}: no triangle on these 3 verts (split?)')
