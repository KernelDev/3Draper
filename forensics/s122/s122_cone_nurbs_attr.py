#!/usr/bin/env python3
"""s122-п.3: attribute the #31359 Cone×Nurbs (7,11) pair.

Local-frame analysis of the junction between the Cone face (s14541)
and the Nurbs ribbon face (s14543): for every junction-edge segment
(A,B) shared by both faces' emissions, find each face's triangle on
that segment and measure the third vertex's height over the line AB.
Target signature (instance frame, rotation-invariant): cone-side
height 0.0011, nurbs-side height 0.0163.
"""
import itertools, math, sys

def load_obj(path):
    verts, tris = [], []
    for line in open(path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
            tris.append((a, b, c))
    return verts, tris

def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def norm(a):
    return math.sqrt(sum(x*x for x in a))

def height_over_line(p, a, b):
    return norm(cross(sub(p, a), sub(b, a))) / norm(sub(b, a))

d = "/tmp/my-project/forensics_s122/faceobjs"
v7, t7 = load_obj(f"{d}/brep31359_f7_s14541_Cone.obj")
v11, t11 = load_obj(f"{d}/brep31359_f11_s14543_Nurbs_deg_3_1__cps_105x2_.obj")

set7 = set(v7)
shared = [v for v in v11 if v in set7]
print(f"shared junction verts: {len(shared)}")

# index maps
idx7 = {v: i for i, v in enumerate(v7)}
idx11 = {v: i for i, v in enumerate(v11)}

# triangles by edge (undirected)
def edge_map(tris):
    em = {}
    for t in tris:
        for a, b in itertools.combinations(t, 2):
            em.setdefault(frozenset((a, b)), []).append(t)
    return em

em7, em11 = edge_map(t7), edge_map(t11)

# for every pair of shared vertices connected in BOTH faces:
pairs_seen = 0
for va in shared:
    for vb in shared:
        if va >= vb:
            continue
        ia, ib = idx7.get(va), idx7.get(vb)
        ja, jb = idx11.get(va), idx11.get(vb)
        if ia is None or ib is None or ja is None or jb is None:
            continue
        k7, k11 = frozenset((ia, ib)), frozenset((ja, jb))
        if k7 not in em7 or k11 not in em11:
            continue
        pairs_seen += 1
        seg_len = norm(sub(vb, va))
        for (ta, tb, tc) in em7[k7]:
            third = v7[tc] if tc not in (ia, ib) else None
            if third:
                h = height_over_line(third, va, vb)
                if h < 0.05:
                    print(f"CONE  seg|AB|={seg_len:.4f} tri=({ta},{tb},{tc}) h_third={h:.4f} third=({third[0]:.4f},{third[1]:.4f},{third[2]:.4f})")
        for (ta, tb, tc) in em11[k11]:
            third = v11[tc] if tc not in (ja, jb) else None
            if third:
                h = height_over_line(third, va, vb)
                if h < 0.05:
                    print(f"NURBS seg|AB|={seg_len:.4f} tri=({ta},{tb},{tc}) h_third={h:.4f} third=({third[0]:.4f},{third[1]:.4f},{third[2]:.4f})")
print(f"junction segments connected in both faces: {pairs_seen}")
