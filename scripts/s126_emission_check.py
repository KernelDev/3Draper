#!/usr/bin/env python3
"""s126 forensics: stage bisection for #10212 REAL pairs
(f(8,8)/f(12,12) Plane WINDING-FLIP FAT + f(19,19) Cylinder FAT).

Method (s125): census VERTS are INSTANCE frame; FACE_MESH emissions are
LOCAL frame. Map instance->local via inverse INST transform, then:
  - pair verts found + triangles present  -> EMISSION-BORN
  - pair verts found + triangles absent   -> MERGE-ASSEMBLED (s125 pattern)
  - pair verts ~0.01-0.02 from emission   -> cross-weld (active->orphan slot)
  - pair verts nowhere                   -> positions not in any emission

VERTS line PRECEDES its census line (census printed at loop end).
Emissions: BOTH attempts (a0..a19 first pass, a20..a39 retry pass) —
the shipped BREP mesh is the retry's if the retry was accepted (s125
lesson was the reverse case; here we cannot know which shipped, so
match both).
"""
import re, os, glob, math
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CENSUS = f"{BASE}/forensics/s126/fm10212/census_pv.txt"
EMDIR = f"{BASE}/forensics/s126/fm10212"

text = open(CENSUS).read()

# ── 1. INST transform for brep_idx=20 (NAUO_29592, BREP#10212) ────────
inst_m = None
for m in re.finditer(
        r"INST brep_idx=(\d+) (\S+)[^\[]*transform=\[([^\]]+)\]", text):
    if m.group(2).startswith("NAUO_29592"):
        nums = [float(x) for x in m.group(3).replace(";", "").split()]
        inst_m = [nums[0:4], nums[4:8], nums[8:12]]
        break
assert inst_m, "INST transform for NAUO_29592 not found"
print(f"INST NAUO_29592: {inst_m}")

R = [inst_m[i][0:3] for i in range(3)]
T = [inst_m[i][3] for i in range(3)]
a, b, c = R[0]; d, e, f_ = R[1]; g, h, i_ = R[2]
det = (a*(e*i_ - f_*h) - b*(d*i_ - f_*g) + c*(d*h - e*g))
def inv_apply(p):
    q = [p[k]-T[k] for k in range(3)]
    return (((e*i_ - f_*h)*q[0] - (b*i_ - c*h)*q[1] + (b*f_ - c*e)*q[2]) / det,
            (-(d*i_ - f_*g)*q[0] + (a*i_ - c*g)*q[1] - (a*f_ - c*d)*q[2]) / det,
            ((d*h - e*g)*q[0] - (a*h - b*g)*q[1] + (a*e - b*d)*q[2]) / det)

# ── 2. emission files: {face_id: [(attempt, verts, tris)]} ───────────
emissions = defaultdict(list)  # fid -> list of (att, verts, tris, path)
for path in sorted(glob.glob(f"{EMDIR}/mesh_brep10212_f*.txt")):
    m = re.match(r"mesh_brep10212_f(\d+)_([A-Za-z]+)_a(\d+)\.txt",
                 os.path.basename(path))
    if not m:
        continue
    fid, att = int(m.group(1)), int(m.group(3))
    vs, ts = [], []
    for l in open(path):
        if l.startswith("v "):
            _, _, x, y, z = l.split()
            vs.append((float(x), float(y), float(z)))
        elif l.startswith("t "):
            _, _, p, q, r = l.split()
            ts.append((int(p), int(q), int(r)))
    emissions[fid].append((att, vs, ts, path))
print(f"emissions: {sorted(emissions)} "
      f"(attempts per face: {[len(v) for v in emissions.values()]})")

def dist2(p, q):
    return (p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2

# ── 3. pair blocks ────────────────────────────────────────────────────
lines = text.splitlines()
verts_re = re.compile(
    r"\s*VERTS T(\d+)=\[\(([^)]*)\) \(([^)]*)\) \(([^)]*)\)\] "
    r"T(\d+)=\[\(([^)]*)\) \(([^)]*)\) \(([^)]*)\)\]")
pp = lambda s: tuple(float(x) for x in s.split(","))

blocks = []
for li, ln in enumerate(lines):
    m = verts_re.match(ln)
    if not m or li+1 >= len(lines):
        continue
    cl = lines[li+1]
    if "BREP#10212" not in cl or not cl.startswith("["):
        continue
    blocks.append((int(m.group(1)), int(m.group(5)),
                   [pp(m.group(i)) for i in (2, 3, 4)],
                   [pp(m.group(i)) for i in (6, 7, 8)], cl.strip()))
print(f"parsed {len(blocks)} #10212 pair blocks\n")

for t0i, t1i, t0, t1, cl in blocks:
    cls = re.match(r"\[([A-Z-]+)( [A-Z-]+)?\]", cl)
    is_real = cls and not cls.group(2)
    fm = re.search(r"faces=\((\d+),", cl)
    fid_txt = fm.group(1) if fm else "?"
    print(f"=== T{t0i}/T{t1i} f({fid_txt},{fid_txt}) "
          f"{'REAL' if is_real else 'sub/exempt'}")
    for tri, nm in ((t0, f"T{t0i}"), (t1, f"T{t1i}")):
        for p in tri:
            lp = inv_apply(p)
            # nearest over ALL faces, all attempts
            best = (1e9, None, None, None)
            for f, lst in emissions.items():
                for att, vs, ts, path in lst:
                    for vi, q in enumerate(vs):
                        dd = dist2(lp, q)
                        if dd < best[0]:
                            best = (dd, f, att, vi)
            dn = math.sqrt(best[0])
            print(f"  {nm} inst{p} -> local "
                  f"({lp[0]:.4f},{lp[1]:.4f},{lp[2]:.4f}) "
                  f"nearest d={dn:.4f} (f{best[1]} a{best[2]} v{best[3]})")
    # triangle presence: all 3 verts within 2e-3 in ONE face emission,
    # then exact set match among that face's tris
    for tri, nm in ((t0, f"T{t0i}"), (t1, f"T{t1i}")):
        lts = [inv_apply(p) for p in tri]
        found = None
        for f, lst in emissions.items():
            for att, vs, ts, path in lst:
                idxs = []
                ok = True
                for lp in lts:
                    vi, bd = None, 4e-6  # 2e-3 squared
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
                        found = (f, att, ti, idxs)
                        break
                if found:
                    break
        if found:
            print(f"  {nm}: EMISSION tri f{found[0]} a{found[1]} "
                  f"t{found[2]} idxs={found[3]}")
        else:
            print(f"  {nm}: NO emission triangle")
    print()
