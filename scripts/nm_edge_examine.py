#!/usr/bin/env python3
"""Examine usage-4 edges on given faces: the 4 triangles around each edge.

Usage: python3 nm_edge_examine.py <dump_dir> <prefix> <face_id> [max_show]
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

def area(v, t):
    a, b, c = v[t[0]], v[t[1]], v[t[2]]
    ab = tuple(b[i]-a[i] for i in range(3))
    ac = tuple(c[i]-a[i] for i in range(3))
    cx = ab[1]*ac[2]-ab[2]*ac[1]; cy = ab[2]*ac[0]-ab[0]*ac[2]; cz = ab[0]*ac[1]-ab[1]*ac[0]
    return 0.5*math.sqrt(cx*cx+cy*cy+cz*cz)

def cent(v, t):
    a, b, c = v[t[0]], v[t[1]], v[t[2]]
    return tuple((a[i]+b[i]+c[i])/3 for i in range(3))

def main():
    verts, tris, fid = load(sys.argv[1], sys.argv[2])
    face = int(sys.argv[3])
    max_show = int(sys.argv[4]) if len(sys.argv) > 4 else 5
    edge_tris = collections.defaultdict(list)
    for ti, t in enumerate(tris):
        if fid.get(ti) != face:
            continue
        for k in range(3):
            a, b = t[k], t[(k+1) % 3]
            key = (min(a,b), max(a,b))
            edge_tris[key].append(ti)
    nm = {e: tis for e, tis in edge_tris.items() if len(tis) >= 3}
    print(f"face f{face}: {len(edge_tris)} edges, {len(nm)} with usage>=3")
    # classify: exact duplicate triangles vs distinct
    n_exact_dup = 0; n_distinct = 0
    shown = 0
    for e, tis in sorted(nm.items()):
        ts = [tris[ti] for ti in tis]
        sigs = collections.Counter(tuple(sorted(t)) for t in ts)
        if any(c >= 2 for c in sigs.values()):
            n_exact_dup += 1
            kind = "EXACT-DUP"
        else:
            n_distinct += 1
            kind = "DISTINCT"
        if shown < max_show:
            print(f"\n--- edge {e} usage={len(tis)} {kind}")
            for ti in tis:
                t = tris[ti]
                print(f"  tri {ti}: v={t} area={area(verts,t):.3e} cent=({cent(verts,t)[0]:.4f},{cent(verts,t)[1]:.4f},{cent(verts,t)[2]:.4f})")
            # pairwise centroid distances
            cs = [cent(verts, tris[ti]) for ti in tis]
            for i in range(len(cs)):
                for j in range(i+1, len(cs)):
                    d = math.dist(cs[i], cs[j])
                    if d < 0.05:
                        print(f"    centroid dist tri{tis[i]}-tri{tis[j]}: {d:.3e}  << NEAR-COINCIDENT")
            shown += 1
    print(f"\nsummary: exact-dup edges={n_exact_dup}, distinct edges={n_distinct}")

if __name__ == '__main__':
    main()
