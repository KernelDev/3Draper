#!/usr/bin/env python3
"""s127 п.1a/1b: analyze RAW cylinder-face emission for winding defects.

Input: forensics/s126/fm10212/mesh_brep10212_f{N}_Cylinder_a{A}.txt
 (a0..a19 = first pass, a20..a39 = retry; s126 established a0..a19 = shipped p21)

Outputs per file:
  - verts/tris, grid test (is (i,j) ring lattice?)
  - directed edge usage: edges used twice in SAME direction (winding conflict)
    vs OPPOSITE direction (consistent manifold)
  - conflict edge structure: which triangle indices, periodicity of indices
  - normal orientation consistency: signed volume of the emission
"""
import re, sys
from collections import defaultdict

def load(path):
    verts, tris = [], []
    for l in open(path):
        if l.startswith("v "):
            parts = l.split()
            verts.append(tuple(float(x) for x in parts[2:5]))
        elif l.startswith("t "):
            parts = l.split()
            idx = [int(x) for x in parts[2:5]]
            tris.append(tuple(idx))
    return verts, tris

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def analyze(path):
    verts, tris = load(path)
    print(f"== {path.split('/')[-1]}: verts={len(verts)} tris={len(tris)}")
    # directed edge usage
    used = defaultdict(list)  # (a,b) sorted-free directed edge -> [tri indices]
    for ti, t in enumerate(tris):
        for k in range(3):
            a, b = t[k], t[(k+1) % 3]
            used[(a, b)].append(ti)
    same_dir = []   # same-direction usage-2
    opp_dir = 0     # consistent
    boundary = 0
    conflict_tris = set()
    for (a, b), tls in used.items():
        if len(tls) == 2:
            # opposite usage would have (b,a) present too
            if (b, a) in used and used[(b, a)]:
                opp_dir += 1
            else:
                same_dir.append(((a, b), tls))
                conflict_tris.update(tls)
        elif len(tls) == 1:
            if (b, a) not in used:
                boundary += 1
    print(f"   directed edges: opp-consistent={opp_dir} SAME-DIR={len(same_dir)} "
          f"(boundary={boundary}, conflict tris={len(conflict_tris)})")
    if same_dir:
        # structure: indices of the edges + triangle index deltas
        idxs = sorted(set(a for (a, b), _ in same_dir) | set(b for (a, b), _ in same_dir))
        print(f"   conflict edge vertex idx range: {min(idxs)}..{max(idxs)}, distinct={len(idxs)}")
        tis = sorted(conflict_tris)
        if len(tis) >= 3:
            diffs = [tis[i+1]-tis[i] for i in range(len(tis)-1)]
            from collections import Counter
            dc = Counter(diffs)
            print(f"   conflict tri indices: {tis[0]}..{tis[-1]}, step histogram: "
                  f"{dict(sorted(dc.items(), key=lambda kv: -kv[1])[:6])}")
        # signed volume (orientation coherence proxy)
        vol = 0.0
        cx = sum(v[0] for v in verts)/len(verts)
        cy = sum(v[1] for v in verts)/len(verts)
        cz = sum(v[2] for v in verts)/len(verts)
        c = (cx, cy, cz)
        for t in tris:
            n = cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]]))
            vol += n[0]*(verts[t[0]][0]-c[0]) + n[1]*(verts[t[0]][1]-c[1]) + n[2]*(verts[t[0]][2]-c[2])
        print(f"   signed volume (6V) = {vol:.6f}  ({'pos' if vol>0 else 'neg' if vol<0 else 'zero'})")
        # sample 3 conflict edges with geometry
        for (a, b), tls in same_dir[:3]:
            print(f"   sample conflict edge ({a}->{b}) tris={tls}: "
                  f"p_a=({verts[a][0]:.3f},{verts[a][1]:.3f},{verts[a][2]:.3f}) "
                  f"p_b=({verts[b][0]:.3f},{verts[b][1]:.3f},{verts[b][2]:.3f})")
    return len(same_dir)

if __name__ == "__main__":
    base = "forensics/s126/fm10212"
    for arg in sys.argv[1:]:
        analyze(f"{base}/{arg}")
