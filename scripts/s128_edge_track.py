#!/usr/bin/env python3
"""s128 probe: track the #10220 f(28,28) REAL pair shared edge through
every stage dump + final, listing ALL triangles on that edge (the fold
partners) with apex heights. Decides birth/death stage of the fold pair.
Local frame (stage dumps) — instance pair verts mapped via INST^-1.
"""
import re, os, glob, math

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FM = f"{BASE}/forensics/s128/fm10220"
text = open(f"{FM}/census_pv.txt").read()

inst_m = None
for m in re.finditer(
        r"INST brep_idx=(\d+) (\S+)[^\[]*transform=\[([^\]]+)\]", text):
    if m.group(2).startswith("NAUO_29477"):
        nums = [float(x) for x in m.group(3).replace(";", "").split()]
        inst_m = [nums[0:4], nums[4:8], nums[8:12]]
        break
R = [inst_m[i][0:3] for i in range(3)]
T = [inst_m[i][3] for i in range(3)]
a, b, c = R[0]; d, e, f_ = R[1]; g, h, i_ = R[2]
det = (a*(e*i_ - f_*h) - b*(d*i_ - f_*g) + c*(d*h - e*g))
def inv_apply(p):
    q = [p[k]-T[k] for k in range(3)]
    return (((e*i_ - f_*h)*q[0] - (b*i_ - c*h)*q[1] + (b*f_ - c*e)*q[2]) / det,
            (-(d*i_ - f_*g)*q[0] + (a*i_ - c*g)*q[1] - (a*f_ - c*d)*q[2]) / det,
            ((d*h - e*g)*q[0] - (a*h - b*g)*q[1] + (a*e - b*d)*q[2]) / det)

# REAL pair verts (instance frame), from VERTS T2989/T2993:
A = inv_apply((-1.1730, 5.7395, 0.6665))   # shared edge end 1 (v1966)
B = inv_apply((-1.0648, 5.6550, 0.7354))   # shared edge end 2 (v2081)
C0 = inv_apply((-1.1180, 5.6815, 0.7021))  # T2989 apex (h=0.0124)
C1 = inv_apply((-1.1171, 5.5929, 0.7065))  # T2993 apex (h=0.0847)
print(f"shared edge local: A={A}\n                B={B}")
print(f"apex T2989 local: {C0}\napex T2993 local: {C1}\n")

def dist2(p, q):
    return (p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2

def edge_triangles(svs, sts, thr2=1e-4):  # (1e-2)^2 = 1e-4? no: (1e-2)^2. Use 4e-6 (2e-3)^2
    thr2 = 4e-6
    ia = min(range(len(svs)), key=lambda w: dist2(svs[w], A)) if svs else None
    ib = min(range(len(svs)), key=lambda w: dist2(svs[w], B)) if svs else None
    if ia is None or dist2(svs[ia], A) > thr2 or dist2(svs[ib], B) > thr2:
        return None, ia, ib
    out = []
    for ti, t in enumerate(sts):
        s = set(t)
        if ia in s and ib in s:
            apex = next(w for w in t if w not in (ia, ib))
            out.append((ti, apex))
    return out, ia, ib

def apex_h(p):
    # distance from p to line(A,B)
    ab = [B[0]-A[0], B[1]-A[1], B[2]-A[2]]
    ap = [p[0]-A[0], p[1]-A[1], p[2]-A[2]]
    cr = [ap[1]*ab[2]-ap[2]*ab[1], ap[2]*ab[0]-ap[0]*ab[2], ap[0]*ab[1]-ap[1]*ab[0]]
    l = math.sqrt(cr[0]**2+cr[1]**2+cr[2]**2)
    base = math.sqrt(ab[0]**2+ab[1]**2+ab[2]**2)
    return l/base

order = sorted(glob.glob(f"{FM}/stages/*.obj"), key=os.path.getmtime)
order.append(f"{FM}/final/brep7_NAUO_29477 (BREP#10220).obj")
for path in order:
    svs, sts = [], []
    for l in open(path):
        if l.startswith("v "):
            _, x, y, z = l.split(); svs.append((float(x), float(y), float(z)))
        elif l.startswith("f "):
            _, p, q, r = l.split(); sts.append((int(p)-1, int(q)-1, int(r)-1))
    tris, ia, ib = edge_triangles(svs, sts)
    name = os.path.basename(path).replace(".obj", "")
    if tris is None:
        print(f"{name}: edge verts NOT found (A d={math.sqrt(dist2(svs[ia],A)) if svs else -1:.4f})"
              if svs else f"{name}: empty")
        continue
    print(f"{name}: {len(sts)} tris; edge tris: "
          + ", ".join(f"#{ti} apex h={apex_h(svs[ap]):.4f}" for ti, ap in tris))
