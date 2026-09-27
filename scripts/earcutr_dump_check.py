#!/usr/bin/env python3
"""Session-62: earcutr dump forensics for GEAR f1/f12 (drill_top).

Checks:
  1. Index correspondence: projecting all_3d[i] onto (u_dir,v_dir) must
     reproduce coords[2i:2i+2] — if not, the 2D winding check was computed
     for DIFFERENT triangles than the 3D ones emitted.
  2. Per-triangle 2D signed area (sa2 from dump) vs 3D normal·plane_n.
  3. Same-face adjacent-pair census on the EMITTED triangles (3D) —
     FOLD-OVER / WINDING-FLIP classification, pre-merge.
"""
import sys, math
from collections import defaultdict

def load(path):
    meta = {}
    verts = []   # (idx, u, v, x, y, z)
    tris = []    # (ti, a, b, c, sa2)
    with open(path) as f:
        for line in f:
            p = line.rstrip('\n').split('\t')
            if p[0] == 'meta':
                for kv in p[1:]:
                    k, _, v = kv.partition('=')
                    meta[k] = v
            elif p[0] == 'v':
                verts.append((int(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6])))
            elif p[0] == 't':
                tris.append((int(p[1]), int(p[2]), int(p[3]), int(p[4]), float(p[5].split('=')[1])))
    return meta, verts, tris

def main(path):
    meta, verts, tris = load(path)
    print(f'=== {path} ===')
    print(f'meta: {meta}')
    fwd = meta.get('forward') == 'true'
    pn = tuple(float(x) for x in meta['plane_n'].strip('()').split(','))
    V = {v[0]: v for v in verts}

    # 1. correspondence check: reproject 3D->2D requires plane origin/basis
    # which we do NOT have. Instead: consistency test — for each triangle,
    # compare sa2 (2D area sign from coords) with the 3D signed volume sign
    # relative to plane normal. If indices match, sign(sa2) must equal
    # sign(normal·pn) for forward, and be negated for !forward.
    agree = disagree = zero = 0
    for (ti, a, b, c, sa2) in tris:
        pa, pb, pc = V[a], V[b], V[c]
        e1 = [pb[i+3]-pa[i+3] for i in range(3)]
        e2 = [pc[i+3]-pa[i+3] for i in range(3)]
        n = (e1[1]*e2[2]-e1[2]*e2[1], e1[2]*e2[0]-e1[0]*e2[2], e1[0]*e2[1]-e1[1]*e2[0])
        dot = sum(n[i]*pn[i] for i in range(3))
        # expected: forward -> 3D normal along +pn with CCW 2D (sa2>0)
        exp_pos = fwd
        if abs(sa2) < 1e-18 or abs(dot) < 1e-18:
            zero += 1
            continue
        if (sa2 > 0) == (dot > 0) == exp_pos or (sa2 > 0) != (dot > 0):
            pass
        # simpler: does sign(sa2) predict sign(dot) consistently for ALL tris?
        if ((sa2 > 0) == (dot > 0)):
            agree += 1
        else:
            disagree += 1
    print(f'sign(sa2) vs sign(n3d·pn): agree={agree} disagree={disagree} zero={zero} (forward={fwd})')

    # 2. How many triangles have 3D normal OPPOSITE to the face normal
    # (face normal = pn if forward else -pn)?
    face_n = pn if fwd else tuple(-x for x in pn)
    inverted = 0
    for (ti, a, b, c, sa2) in tris:
        pa, pb, pc = V[a], V[b], V[c]
        e1 = [pb[i+3]-pa[i+3] for i in range(3)]
        e2 = [pc[i+3]-pa[i+3] for i in range(3)]
        n = (e1[1]*e2[2]-e1[2]*e2[1], e1[2]*e2[0]-e1[0]*e2[2], e1[0]*e2[1]-e1[1]*e2[0])
        dot = sum(n[i]*face_n[i] for i in range(3))
        if dot < 0:
            inverted += 1
    print(f'3D-inverted triangles (normal against face normal): {inverted}/{len(tris)}')

    # 3. adjacent-pair census on emitted triangles
    edge_map = defaultdict(list)
    for (ti, a, b, c, sa2) in tris:
        for v0, v1 in ((a, b), (b, c), (c, a)):
            key = (min(v0, v1), max(v0, v1))
            edge_map[key].append((ti, v0, v1))
    census = defaultdict(int)
    for edge, lst in edge_map.items():
        if len(lst) != 2:
            continue
        (t0, da0, db0), (t1, da1, db1) = lst
        topo = (da0 == db1) and (db0 == da1)
        # normals
        def nrm(ti):
            tt = tris[ti]
            pa, pb, pc = V[tt[1]], V[tt[2]], V[tt[3]]
            e1 = [pb[i+3]-pa[i+3] for i in range(3)]
            e2 = [pc[i+3]-pa[i+3] for i in range(3)]
            n = (e1[1]*e2[2]-e1[2]*e2[1], e1[2]*e2[0]-e1[0]*e2[2], e1[0]*e2[1]-e1[1]*e2[0])
            ln = math.sqrt(sum(x*x for x in n))
            return tuple(x/ln for x in n) if ln > 1e-30 else None
        n0, n1 = nrm(t0), nrm(t1)
        if not n0 or not n1:
            continue
        dot = sum(n0[i]*n1[i] for i in range(3))
        ang = math.degrees(math.acos(max(-1, min(1, dot))))
        if ang <= 170:
            continue
        a3, b3 = V[edge[0]], V[edge[1]]
        a = a3[3:6]; b = b3[3:6]
        e = [b[i]-a[i] for i in range(3)]
        def apex(ti):
            tt = tris[ti]
            s = {tt[1], tt[2], tt[3]} - {edge[0], edge[1]}
            return V[s.pop()][3:6] if s else a
        def side(q):
            d = [q[i]-a[i] for i in range(3)]
            return (e[1]*d[2]-e[2]*d[1], e[2]*d[0]-e[0]*d[2], e[0]*d[1]-e[1]*d[0])
        s0, s1 = side(apex(t0)), side(apex(t1))
        same = sum(s0[i]*s1[i] for i in range(3)) > 0
        if topo and same: cls = 'FOLD-OVER'
        elif topo: cls = 'CURVED-180'
        elif same: cls = 'DOUBLE-BROKEN'
        else: cls = 'WINDING-FLIP'
        census[cls] += 1
    print('pair census (emitted, pre-merge): ' +
          ' '.join(f'{k}={v}' for k, v in sorted(census.items())))

if __name__ == '__main__':
    for path in sys.argv[1:]:
        main(path)
        print()
