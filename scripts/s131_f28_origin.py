#!/usr/bin/env python3
"""s131 forensics: #10220 f(28,28) REAL pair provenance (h=0.0124,0.0847).

Method (s125/s126): census VERTS are INSTANCE frame; FACE_MESH
emissions are LOCAL frame. Map instance->local via inverse INST
transform, then locate each pair vertex and the pair triangles in
the f25/f28 emissions (both attempts) to classify:
  EMISSION-BORN (verts+tri in one face emission)
  CROSS-WELD ASSEMBLED (verts from BOTH faces' emissions, tri born
  at merge weld)
  NOT-IN-EMISSION (positions created by merge-level processes)
"""
import re, os, glob, math

EMDIR = "/tmp/s131_fm10220"
# INST NAUO_29477 (from SURF_PARAMS dump, printed 4-dp)
M = [[1.0000, -0.0000, 0.0000, -0.8964],
     [0.0000, 0.5465, -0.8375, 3.7869],
     [0.0000, 0.8375, 0.5465, -6.4881]]

# REAL pair (census VERTS line, instance frame)
T0 = [(-1.1730, 5.7395, 0.6665), (-1.0648, 5.6550, 0.7354), (-1.1180, 5.6815, 0.7021)]
T1 = [(-1.1730, 5.7395, 0.6665), (-1.1171, 5.5929, 0.7065), (-1.0648, 5.6550, 0.7354)]

# ── inverse transform (rigid) ─────────────────────────────────────────
R = [row[0:3] for row in M]
T = [row[3] for row in M]
a, b, c = R[0]; d, e, f_ = R[1]; g, h, i_ = R[2]
det = (a*(e*i_ - f_*h) - b*(d*i_ - f_*g) + c*(d*h - e*g))
def inv_apply(p):
    q = [p[k]-T[k] for k in range(3)]
    return (((e*i_ - f_*h)*q[0] - (b*i_ - c*h)*q[1] + (b*f_ - c*e)*q[2]) / det,
            (-(d*i_ - f_*g)*q[0] + (a*i_ - c*g)*q[1] - (a*f_ - c*d)*q[2]) / det,
            ((d*h - e*g)*q[0] - (a*h - b*g)*q[1] + (a*e - b*d)*q[2]) / det)

def dist2(p, q):
    return (p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2

# ── load emissions of f25/f28 (both attempts) ─────────────────────────
emissions = {}  # (fid, att) -> (verts, tris)
for path in sorted(glob.glob(f"{EMDIR}/mesh_brep10220_f*_Nurbs*.txt")):
    base = os.path.basename(path)
    m = re.match(r"mesh_brep10220_f(\d+)_Nurbs.*_a(\d+)\.txt", base)
    if not m:
        continue
    fid, att = int(m.group(1)), int(m.group(2))
    vs, ts = [], []
    for l in open(path):
        if l.startswith("v "):
            parts = l.split()
            # format: "v <index> <x> <y> <z>"
            vs.append((float(parts[2]), float(parts[3]), float(parts[4])))
        elif l.startswith("t "):
            parts = l.split()
            # format: "t <index> <a> <b> <c>"
            ts.append((int(parts[2]), int(parts[3]), int(parts[4])))
    emissions[(fid, att)] = (vs, ts)
    print(f"loaded {base}: {len(vs)} verts, {len(ts)} tris")

print()
THRESH2 = (2e-3 ** 2)  # vertex match threshold

for tname, tri in (("T2989", T0), ("T2993", T1)):
    print(f"=== {tname} (REAL pair, instance frame)")
    lts = [inv_apply(p) for p in tri]
    for pi, (ip, lp) in enumerate(zip(tri, lts)):
        best = (1e9, None)
        for (fid, att), (vs, ts) in sorted(emissions.items()):
            for vi, q in enumerate(vs):
                dd = dist2(lp, q)
                if dd < best[0]:
                    best = (dd, f"f{fid} a{att} v{vi}")
        print(f"  v{pi} inst{ip} -> local ({lp[0]:.4f},{lp[1]:.4f},{lp[2]:.4f}) "
              f"nearest d={math.sqrt(best[0]):.4f} ({best[1]})")
    # triangle presence in a single face emission
    tset, nm = lts, tname
    if True:
        found = None
        for (fid, att), (vs, ts) in sorted(emissions.items()):
            idxs = []
            ok = True
            for lp in tset:
                vi, bd = None, THRESH2
                for w, q in enumerate(vs):
                    dd = dist2(lp, q)
                    if dd < bd:
                        vi, bd = w, dd
                if vi is None:
                    ok = False
                    break
                idxs.append(vi)
            if not ok:
                continue
            s = set(idxs)
            for ti, t in enumerate(ts):
                if set(t) == s:
                    found = (fid, att, ti, idxs)
                    break
            if found:
                break
        if found:
            print(f"  {nm}: EMISSION tri f{found[0]} a{found[1]} t{found[2]} idxs={found[3]}")
        else:
            print(f"  {nm}: NO single-face emission triangle (cross-weld or merge-born)")

# shared edge + apex geometry summary (local frame)
print()
ap0, ap1 = inv_apply(T0[2]), inv_apply(T1[1])
e0, e1 = inv_apply(T0[0]), inv_apply(T0[1])
print(f"shared edge (local): {e0} .. {e1}")
print(f"apex0 (h=0.0124) local: {tuple(round(x,4) for x in ap0)}")
print(f"apex1 (h=0.0847) local: {tuple(round(x,4) for x in ap1)}")
print(f"apex-apex 3D distance: {math.sqrt(dist2(ap0, ap1)):.4f}")
