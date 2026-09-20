#!/usr/bin/env python3
"""Analyze STRIPEMIT dumps: find fold-over pairs (shared edge, same-side
apexes, dihedral >170 deg) within each strip's emitted triangles."""
import math, re, sys

def parse(path):
    strips = []  # list of dict(header=..., tris=[(p1,p2,p3)])
    cur = None
    emit_re = re.compile(r"STRIPEMIT: \(([-\d.]+),([-\d.]+),([-\d.]+)\) \(([-\d.]+),([-\d.]+),([-\d.]+)\) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")
    hdr_re = re.compile(r"STRIPDUMP: bnd=(\d+)")
    with open(path) as f:
        for line in f:
            m = hdr_re.search(line)
            if m:
                if cur and cur["tris"]:
                    strips.append(cur)
                cur = {"header": line.strip(), "tris": []}
                continue
            m = emit_re.search(line)
            if m and cur is not None:
                v = [float(x) for x in m.groups()]
                cur["tris"].append(((v[0],v[1],v[2]),(v[3],v[4],v[5]),(v[6],v[7],v[8])))
    if cur and cur["tris"]:
        strips.append(cur)
    return strips

def sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def dot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def norm(a):
    l = math.sqrt(dot(a,a))
    return (a[0]/l,a[1]/l,a[2]/l) if l > 1e-15 else None

def tri_normal(t):
    n = cross(sub(t[1],t[0]), sub(t[2],t[0]))
    l = math.sqrt(dot(n,n))
    if l < 1e-12: return None, 0.0
    return (n[0]/l,n[1]/l,n[2]/l), 0.5*l

def key(p, nd=4):
    return (round(p[0],nd), round(p[1],nd), round(p[2],nd))

def analyze(strips):
    total_pairs = 0
    for si, s in enumerate(strips):
        tris = s["tris"]
        # edge map: edge key (sorted pair of vertex keys) -> list of tri indices + which apex
        edges = {}
        for ti, t in enumerate(tris):
            ks = [key(p) for p in t]
            for i in range(3):
                a, b = ks[i], ks[(i+1)%3]
                e = (min(a,b), max(a,b))
                apex = ks[(i+2)%3]
                edges.setdefault(e, []).append((ti, apex))
        pairs = []
        for e, lst in edges.items():
            if len(lst) != 2: continue
            (t0, ap0), (t1, ap1) = lst
            n0, a0 = tri_normal(tris[t0])
            n1, a1 = tri_normal(tris[t1])
            if n0 is None or n1 is None: continue
            cosang = dot(n0, n1)
            ang = math.degrees(math.acos(max(-1, min(1, cosang))))
            # apex same side: both apexes on same side of the edge line
            pA, pB = e[0], e[1]
            ap0p = tris[t0][ ( [key(p) for p in tris[t0] ].index(ap0) ) ]
            ap1p = tris[t1][ ( [key(p) for p in tris[t1] ].index(ap1) ) ]
            ed = sub(pB, pA)
            s0 = cross(ed, sub(ap0p, pA))
            s1 = cross(ed, sub(ap1p, pA))
            same_side = dot(s0, s1) > 0
            if ang > 170 and same_side:
                pairs.append((t0, t1, ang, a0, a1, pA, pB, ap0p, ap1p))
        if pairs:
            total_pairs += len(pairs)
            print(f"=== strip #{si}: {s['header']} — {len(tris)} tris, {len(pairs)} FOLD PAIRS ===")
            for (t0, t1, ang, a0, a1, pA, pB, ap0p, ap1p) in pairs[:8]:
                print(f"  pair tri[{t0}](area={a0:.2f}) x tri[{t1}](area={a1:.2f}) dihedral={ang:.1f}")
                print(f"    shared edge: {pA} — {pB}")
                print(f"    apex0: ({ap0p[0]:.5f},{ap0p[1]:.5f},{ap0p[2]:.5f})")
                print(f"    apex1: ({ap1p[0]:.5f},{ap1p[1]:.5f},{ap1p[2]:.5f})")
    print(f"\nTOTAL strips={len(strips)}, fold pairs={total_pairs}")

if __name__ == "__main__":
    strips = parse(sys.argv[1] if len(sys.argv) > 1 else "/home/z/my-project/scripts/strip_dump.txt")
    analyze(strips)
