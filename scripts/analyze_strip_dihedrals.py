#!/usr/bin/env python3
"""Session-43: full dihedral analysis of STRIPEMIT dumps — both fold
families (same-side FOLD-OVER / opposite-side INVERTED), winding
consistency between adjacent triangles, and the dihedral histogram."""
import math, re, sys
from collections import defaultdict

def parse(path):
    strips, cur = [], None
    emit_re = re.compile(r"STRIPEMIT: \(([-\d.]+),([-\d.]+),([-\d.]+)\) \(([-\d.]+),([-\d.]+),([-\d.]+)\) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")
    for line in open(path):
        if line.startswith("STRIPDUMP: bnd="):
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

def sub(a,b): return (a[0]-b[0],a[1]-b[1],a[2]-b[2])
def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def dot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def tri_normal(t):
    n = cross(sub(t[1],t[0]), sub(t[2],t[0]))
    l = math.sqrt(dot(n,n))
    return (n[0]/l,n[1]/l,n[2]/l) if l > 1e-15 else None

def key(p, nd=4):
    return (round(p[0],nd),round(p[1],nd),round(p[2],nd))

def analyze(path):
    for si, s in enumerate(parse(path)):
        tris = s["tris"]
        # edge -> list of (tri_idx, oriented_dir) where oriented_dir is the
        # traversal direction of the edge IN THAT TRIANGLE (a->b as visited)
        edges = defaultdict(list)
        for ti, t in enumerate(tris):
            ks = [key(p) for p in t]
            for i in range(3):
                a, b = ks[i], ks[(i+1)%3]
                edges[(min(a,b),max(a,b))].append((ti, a, b))
        hist = defaultdict(int)
        same_side_170 = 0
        opp_side_170 = 0
        winding_bad = 0
        winding_ok = 0
        for e, owners in edges.items():
            if len(owners) != 2:
                continue
            (t0,a0,b0),(t1,a1,b1) = owners
            # winding consistency: the shared edge must be traversed in
            # OPPOSITE directions by the two triangles (a->b vs b->a)
            if a0 == a1:  # same direction = inconsistent winding
                winding_bad += 1
            else:
                winding_ok += 1
            n0, n1 = tri_normal(tris[t0]), tri_normal(tris[t1])
            if n0 is None or n1 is None:
                continue
            cosang = dot(n0,n1)
            ang = math.degrees(math.acos(max(-1,min(1,cosang))))
            hist[int(ang//10)*10] += 1
            if ang > 170:
                pA, pB = e
                ap0 = tris[t0][[key(p) for p in tris[t0]].index([x for x in (key(tris[t0][0]),key(tris[t0][1]),key(tris[t0][2])) if x not in (pA,pB)][0])]
                # simpler: apex = the vertex not on the edge
                def apex(t):
                    ks = [key(p) for p in t]
                    return t[[i for i,k in enumerate(ks) if k not in (pA,pB)][0]]
                ap0p, ap1p = apex(tris[t0]), apex(tris[t1])
                ed = sub(pB,pA)
                s0 = cross(ed, sub(ap0p,pA))
                s1 = cross(ed, sub(ap1p,pA))
                if dot(s0,s1) > 0:
                    same_side_170 += 1
                else:
                    opp_side_170 += 1
        total_pairs = sum(hist.values())
        print(f"=== strip #{si}: {len(tris)} tris, {total_pairs} usage-2 pairs ===")
        print(f"  winding consistent: {winding_ok}, INCONSISTENT: {winding_bad}")
        print(f"  >170 same-side (FOLD-OVER): {same_side_170}")
        print(f"  >170 opposite-side (INVERTED): {opp_side_170}")
        print(f"  dihedral histogram (10-deg bins): {dict(sorted(hist.items()))}")

if __name__ == "__main__":
    analyze(sys.argv[1] if len(sys.argv) > 1 else "/tmp/nut_strip2.log")
