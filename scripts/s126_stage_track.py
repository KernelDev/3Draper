#!/usr/bin/env python3
"""s126: track the #10212 f8 pair triangles (positions of emission
v53/v90/v91/v54, LOCAL frame) through the detailed-path stage dumps
(after-merge -> after-weld -> after-tj -> after-gapfill -> after-winding)
plus the final instance OBJ, and report the WINDING ORDER at each stage.

The pair in the final merged instance mesh is T1865=(v53,v91,v90) vs the
emission t103=(v53,v90,v91) — REVERSED. Find the stage where the order
flips (suspect: fix_inconsistent_winding BFS cascade, s114 mechanism).
"""
import os, re, sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDIR = f"{BASE}/forensics/s126/stages2"

# pair anchor positions (local frame, from f8 emission a7)
V53 = (-1.003702985, -3.675000000, -2.289668168)
V90 = (-1.014296737, -3.675000000, -2.312320724)
V91 = (-0.957775728, -3.675000000, -2.336298494)
V54 = (-1.059044589, -3.675000000, -2.264602517)
ANCHORS = {"53": V53, "90": V90, "91": V91, "54": V54}

def load_obj(path):
    vs, fs = [], []
    for l in open(path):
        if l.startswith("v "):
            _, x, y, z = l.split()
            vs.append((float(x), float(y), float(z)))
        elif l.startswith("f "):
            _, a, b, c = l.split()
            fs.append((int(a)-1, int(b)-1, int(c)-1))  # OBJ 1-based -> 0
    return vs, fs

def match(vs, p, tol=1e-6):
    for i, q in enumerate(vs):
        if (abs(q[0]-p[0]) < tol and abs(q[1]-p[1]) < tol
                and abs(q[2]-p[2]) < tol):
            return i
    # relaxed pass
    for i, q in enumerate(vs):
        if (abs(q[0]-p[0]) < 2e-3 and abs(q[1]-p[1]) < 2e-3
                and abs(q[2]-p[2]) < 2e-3):
            return i
    return None

STAGES = ["after-merge", "after-weld", "after-tj", "after-gapfill",
          "after-winding"]
for stage in STAGES:
    for p in ("p21", "p22"):
        path = f"{SDIR}/{p}_b10212-d-{stage}.obj"
        if not os.path.exists(path):
            continue
        vs, fs = load_obj(path)
        idx = {k: match(vs, v) for k, v in ANCHORS.items()}
        if any(i is None for i in idx.values()):
            print(f"{p} {stage}: anchors missing {idx}")
            continue
        # find triangles containing the pair sets
        s1 = {idx["53"], idx["90"], idx["91"]}   # t103 set
        s2 = {idx["53"], idx["54"], idx["90"]}   # t104 set
        hit1 = [t for t in fs if set(t) == s1]
        hit2 = [t for t in fs if set(t) == s2]
        # also: which verts have multiple triangles (usage counts)
        def tri_at(set_):
            return [t for t in fs if set(t) == set_]
        print(f"{p} {stage}: verts n={len(vs)} tris={len(fs)}")
        print(f"   anchors: " + " ".join(
            f"{k}->v{i}" for k, i in idx.items()))
        if hit1:
            for t in hit1:
                # order as (53,90,91)? or (53,91,90)?
                print(f"   t103-set present: winding={t} "
                      f"({'53,90,91 (emission)' if (t[0]==idx['53'] and t[1]==idx['90']) or (t[1]==idx['53'] and t[2]==idx['90']) or (t[2]==idx['53'] and t[0]==idx['90']) else '53,91,90 (REVERSED?)'})")
        else:
            print("   t103-set ABSENT")
        if hit2:
            for t in hit2:
                print(f"   t104-set present: winding={t}")
        else:
            print("   t104-set ABSENT")

# final instance mesh (from the FINAL_OBJS dump)
print("\n=== final instance mesh (brep20 NAUO_29592) ===")
FIN = f"{BASE}/forensics/s126/final/brep20_NAUO_29592 (BREP#10212).obj"
import math
# instance frame: apply forward transform M*local+t for anchor compare
Rm = [[1.0, 0.0, 0.0089], [0.0, 1.0, 0.0], [-0.0089, 0.0, 1.0]]
Tv = [0.001, 0.001, -0.0009]
def fwd(p):
    return tuple(sum(Rm[r][c]*p[c] for c in range(3)) + Tv[r]
                 for r in range(3))
AINST = {k: fwd(v) for k, v in ANCHORS.items()}
vs, fs = load_obj(FIN)
idx = {}
for k, p in AINST.items():
    idx[k] = match(vs, p, tol=2e-3)
print("anchors(instance): " + " ".join(f"{k}->v{i}" for k, i in idx.items()))
if all(i is not None for i in idx.values()):
    s1 = {idx["53"], idx["90"], idx["91"]}
    s2 = {idx["53"], idx["54"], idx["90"]}
    for t in fs:
        if set(t) == s1:
            print(f"   t103-set: winding={t}")
        if set(t) == s2:
            print(f"   t104-set: winding={t}")
