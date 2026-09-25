#!/usr/bin/env python3
"""Edge-usage analysis of a final OBJ dump + fmap (per-face attribution).

Usage: python3 edge_usage.py <dir> <obj_name_prefix>
Prints: usage histogram, bnd/NM totals, per-face top boundary-edge faces,
and (if a second dir is given) a diff of per-face bnd counts.
"""
import sys, os, collections

def load(base):
    # find obj + fmap by prefix
    obj = fmap = None
    for f in os.listdir(base):
        if f.startswith(sys.argv[2]) and f.endswith('.obj'):
            obj = os.path.join(base, f)
        if f.startswith(sys.argv[2]) and f.endswith('.fmap'):
            fmap = os.path.join(base, f)
    assert obj and fmap, f"files not found in {base} for prefix {sys.argv[2]}"
    verts = []
    tris = []
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

def analyze(base):
    verts, tris, fid = load(base)
    usage = collections.Counter()          # edge -> count
    face_bnd = collections.Counter()       # face -> #usage-1 edges (attributed to both faces)
    face_nm = collections.Counter()
    tri_by_face = collections.Counter()
    for ti, (a, b, c) in enumerate(tris):
        f = fid.get(ti, -1)
        tri_by_face[f] += 1
        for u, v in ((a,b), (b,c), (c,a)):
            if u > v: u, v = v, u
            usage[(u, v)] += 1
    hist = collections.Counter(usage.values())
    for (u, v), n in usage.items():
        if n == 1:
            for f in set([fid.get(t, -1) for t, t3 in enumerate(tris) if False]):
                pass
    # per-face attribution of boundary edges: a usage-1 edge belongs to the
    # single triangle that contains it -> its face
    for ti, (a, b, c) in enumerate(tris):
        f = fid.get(ti, -1)
        for u, v in ((a,b), (b,c), (c,a)):
            if u > v: u, v = v, u
            if usage[(u, v)] == 1:
                face_bnd[f] += 1
            elif usage[(u, v)] >= 3:
                face_nm[f] += 1  # counted once per containing triangle; divide later
    return {
        'verts': len(verts), 'tris': len(tris),
        'usage_hist': dict(sorted(hist.items())),
        'bnd': hist[1], 'nm_edges': sum(n for k, n in hist.items() if k >= 3),
        'face_bnd': face_bnd, 'face_nm': face_nm, 'tri_by_face': tri_by_face,
    }

def main():
    a = analyze(sys.argv[1])
    print(f"=== {sys.argv[1]} ===")
    print(f"verts={a['verts']} tris={a['tris']}")
    print(f"usage hist: {a['usage_hist']}")
    print(f"boundary(usage1)={a['bnd']}  nm(usage>=3)={a['nm_edges']}")
    top = a['face_bnd'].most_common(12)
    print("top bnd faces:", ", ".join(f"f{f}:{n}" for f, n in top))
    topn = [(f, n) for f, n in a['face_nm'].most_common(8) if n >= 3]
    print("top nm faces:", ", ".join(f"f{f}:{n//3 if n%3==0 else n}" for f, n in topn))
    if len(sys.argv) > 3:
        b = analyze(sys.argv[3])
        print(f"=== {sys.argv[3]} ===")
        print(f"verts={b['verts']} tris={b['tris']}")
        print(f"usage hist: {b['usage_hist']}")
        print(f"boundary(usage1)={b['bnd']}  nm(usage>=3)={b['nm_edges']}")
        top = b['face_bnd'].most_common(12)
        print("top bnd faces:", ", ".join(f"f{f}:{n}" for f, n in top))
        topn = [(f, n) for f, n in b['face_nm'].most_common(8) if n >= 3]
        print("top nm faces:", ", ".join(f"f{f}:{n//3 if n%3==0 else n}" for f, n in topn))
        # diff: bnd reduction per face
        diff = {}
        for f in set(a['face_bnd']) | set(b['face_bnd']):
            d = b['face_bnd'].get(f, 0) - a['face_bnd'].get(f, 0)
            if d != 0:
                diff[f] = d
        neg = sorted(diff.items(), key=lambda x: x[1])[:15]
        pos = sorted(diff.items(), key=lambda x: -x[1])[:10]
        print("bnd REDUCTION per face (top):", ", ".join(f"f{f}:{d}" for f, d in neg))
        print("bnd INCREASE per face (top):", ", ".join(f"f{f}:{+d}" for f, d in pos))
        # NM diff
        nmdiff = {}
        for f in set(a['face_nm']) | set(b['face_nm']):
            d = b['face_nm'].get(f, 0) - a['face_nm'].get(f, 0)
            if abs(d) >= 9:
                nmdiff[f] = d
        print("nm delta per face (|d|>=3 edges):", ", ".join(f"f{f}:{d//3}" for f, d in sorted(nmdiff.items(), key=lambda x: -x[1])[:12]))

if __name__ == '__main__':
    main()
