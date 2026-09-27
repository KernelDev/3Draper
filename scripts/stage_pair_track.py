#!/usr/bin/env python3
"""Session-62 diagnostics: track WINDING-FLIP / FOLD-OVER same-face pairs
across pipeline stage dumps for GEAR faces 1 and 12 (drill_top.stp).

For each stage OBJ+fmap:
  - edge -> [tris] (exactly 2)
  - ang between normals > 170 deg
  - topo_consistent = edge traversed opposite directions
  - same_side = apexes on same side of edge line
  - class: FOLD-OVER (consistent+same), WINDING-FLIP (inconsistent+opp),
           CURVED-180 (consistent+opp), DOUBLE-BROKEN (inconsistent+same)
Prints per-stage census for faces 1/12 + overall, and per-pair detail.
"""
import sys, os, math
from collections import defaultdict

def load_obj(path):
    verts, tris = [], []
    with open(path) as f:
        for line in f:
            if line.startswith('v '):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith('f '):
                p = line.split()
                tris.append((int(p[1])-1, int(p[2])-1, int(p[3])-1))
    return verts, tris

def load_fmap(path):
    m = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 3 and p[0] == 't':
                m[int(p[1])] = int(p[2])
    return m

def normal(v, tri):
    a, b, c = (v[i] for i in tri)
    e1 = tuple(b[i]-a[i] for i in range(3))
    e2 = tuple(c[i]-a[i] for i in range(3))
    n = (e1[1]*e2[2]-e1[2]*e2[1], e1[2]*e2[0]-e1[0]*e2[2], e1[0]*e2[1]-e1[1]*e2[0])
    ln = math.sqrt(sum(x*x for x in n))
    if ln < 1e-30:
        return None
    return tuple(x/ln for x in n)

def analyze(stage_dir, prefix, target_faces=None, detail=True):
    import glob as _glob
    stages = ['after-merge', 'after-weld', 'after-tj', 'after-gapfill', 'after-winding']
    for st in stages:
        obj = os.path.join(stage_dir, f'{prefix}-{st}.obj')
        fmap = os.path.join(stage_dir, f'{prefix}-{st}.fmap')
        if not os.path.exists(obj):
            cands = sorted(_glob.glob(os.path.join(stage_dir, f'{prefix}*{st}.obj')))
            if not cands:
                continue
            obj, fmap = cands[0], cands[0][:-4] + '.fmap'
        v, tris = load_obj(obj)
        fm = load_fmap(fmap) if os.path.exists(fmap) else {}
        edge_map = defaultdict(list)
        for ti, tri in enumerate(tris):
            a, b, c = tri
            for v0, v1 in ((a,b),(b,c),(c,a)):
                key = (v0, v1) if v0 < v1 else (v1, v0)
                edge_map[key].append((ti, v0, v1))
        census = defaultdict(int)
        details = []
        for edge, lst in edge_map.items():
            if len(lst) != 2:
                continue
            (t0, da0, db0), (t1, da1, db1) = lst
            f0, f1 = fm.get(t0), fm.get(t1)
            if f0 != f1 or f0 is None:
                continue
            if target_faces and f0 not in target_faces:
                continue
            n0, n1 = normal(v, tris[t0]), normal(v, tris[t1])
            if n0 is None or n1 is None:
                continue
            dot = sum(n0[i]*n1[i] for i in range(3))
            ang = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
            if ang <= 170.0:
                continue
            topo_consistent = (da0 == db1) and (db0 == da1)
            # apex side test
            a, b = v[edge[0]], v[edge[1]]
            e = tuple(b[i]-a[i] for i in range(3))
            def apex(ti):
                s = set(tris[ti])
                for x in edge:
                    s.discard(x)
                return v[s.pop()] if s else a
            def side(q):
                d = tuple(q[i]-a[i] for i in range(3))
                return (e[1]*d[2]-e[2]*d[1], e[2]*d[0]-e[0]*d[2], e[0]*d[1]-e[1]*d[0])
            s0, s1 = side(apex(t0)), side(apex(t1))
            same_side = sum(s0[i]*s1[i] for i in range(3)) > 0
            if topo_consistent and same_side: cls = 'FOLD-OVER'
            elif topo_consistent and not same_side: cls = 'CURVED-180'
            elif not topo_consistent and same_side: cls = 'DOUBLE-BROKEN'
            else: cls = 'WINDING-FLIP'
            census[cls] += 1
            details.append((st, cls, f0, edge, t0, t1, tris[t0], tris[t1]))
        total = sum(census.values())
        tag = f'faces {sorted(target_faces)}' if target_faces else 'all faces'
        print(f'{prefix}_{st}: {tag} same-face pairs>170: {total} ' +
              ' '.join(f'{k}={census[k]}' for k in sorted(census)))
        if detail:
            for d in details:
                print(f'    {d[1]} f{d[2]} edge{d[3]} T{d[4]}{d[6]} T{d[5]}{d[7]}')

if __name__ == '__main__':
    stage_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/stages'
    faces = set(int(x) for x in sys.argv[2].split(',')) if len(sys.argv) > 2 else None
    for prefix in sys.argv[3].split(',') if len(sys.argv) > 3 else ['p0_d', 'p1_d']:
        analyze(stage_dir, prefix, faces)
        print()
