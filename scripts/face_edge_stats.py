#!/usr/bin/env python3
"""Per-face 3D edge length stats from final OBJ + fmap, vs merge tolerance.

Usage: python3 face_edge_stats.py <dump_dir> <prefix> [face_ids...]
"""
import sys, os, collections, math

def load(base, prefix):
    obj = fmap = None
    for f in os.listdir(base):
        if f.startswith(prefix) and f.endswith('.obj'):
            obj = os.path.join(base, f)
        if f.startswith(prefix) and f.endswith('.fmap'):
            fmap = os.path.join(base, f)
    verts, tris = [], []
    with open(obj) as fh:
        for line in fh:
            if line.startswith('v '):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith('f '):
                p = line.split()[1:]
                tris.append((int(p[0])-1, int(p[1])-1, int(p[2])-1))
    fid = {}
    with open(fmap) as fh:
        for line in fh:
            if line.startswith('t '):
                _, ti, f = line.split()
                fid[int(ti)] = int(f)
    return verts, tris, fid

def main():
    verts, tris, fid = load(sys.argv[1], sys.argv[2])
    # bbox diag
    xs = [v[0] for v in verts]; ys = [v[1] for v in verts]; zs = [v[2] for v in verts]
    diag = math.dist((min(xs),min(ys),min(zs)), (max(xs),max(ys),max(zs)))
    tol = diag * 1e-6
    print(f"bbox diag={diag:.4f}  merge_tol(1e-6)={tol:.3e}")
    want = set(int(a) for a in sys.argv[3:]) if len(sys.argv) > 3 else None
    # per-face edge lengths
    face_edges = collections.defaultdict(set)
    for ti, (a, b, c) in enumerate(tris):
        f = fid.get(ti, -1)
        for u, v in ((a,b), (b,c), (c,a)):
            if u > v: u, v = v, u
            face_edges[f].add((u, v))
    stats = []
    for f, es in face_edges.items():
        if want is not None and f not in want:
            continue
        ls = sorted(math.dist(verts[u], verts[v]) for u, v in es)
        n = len(ls)
        stats.append((f, n, ls[0], ls[max(0,n//100)], ls[n//2], ls[-1]))
    stats.sort(key=lambda s: s[2])  # by min edge
    print(f"{'face':>6} {'n_edg':>6} {'min':>10} {'p1':>10} {'med':>10} {'max':>10}  min<tol?")
    for f, n, mn, p1, med, mx in stats[:25]:
        print(f"f{f:<5} {n:>6} {mn:>10.3e} {p1:>10.3e} {med:>10.3e} {mx:>10.3e}  {'*** BELOW TOL' if mn < tol else ''}")
    print("...")
    for f, n, mn, p1, med, mx in stats[-5:]:
        print(f"f{f:<5} {n:>6} {mn:>10.3e} {p1:>10.3e} {med:>10.3e} {mx:>10.3e}")

if __name__ == '__main__':
    main()
