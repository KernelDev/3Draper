#!/usr/bin/env python3
"""s128 forensics: #10220 f(28,28) REAL pair — emission path attribution.

REAL pair (Vulcan, BREP#10220 = brep_idx 7 NAUO_29477, instanced):
  [FOLD-OVERFAT] faces=(28,28) tris=([1966, 2081, 2340],[1966, 1945, 2081])
  T2989/T2993, h=(0.0124,0.0847), ang=180.00, snAng=n/a, mid=(-1.12,5.70,0.70)
  owners = f25(Nurbs)+f28(Nurbs) — overlapping triangle_range (post-repair zones)
Companions on the same face 28 (context for the ribbon anatomy):
  SUBTOL near-miss: tris=([2012,1995,2056],[2032,1995,2012]) h=(0.0076,0.0104)
  COIL-TURN exempt : tris=([1986,2082,1945],[1986,2083,2082]) rim-gap 0.01105

Method (s125/s126): census VERTS = INSTANCE frame; FACE_MESH emissions and
STAGE_OBJS dumps = LOCAL frame. Map instance->local via inverse INST, then:
  - pair verts found in emission + triangles present -> EMISSION-BORN
  - pair verts found + triangles absent -> MERGE-ASSEMBLED (s125 pattern)
  - pair verts ~0.01-0.02 off -> cross-weld (active->orphan slot)
  - nowhere -> positions not in any emission
Stage bisection: stages/p{0,1}_{b}?{stage}.obj(+.fmap) — find in which stage
the pair triangles first appear (attribution to a repair stage).
"""
import re, os, glob, math
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FM = f"{BASE}/forensics/s128/fm10220"
CENSUS = f"{FM}/census_pv.txt"

text = open(CENSUS).read()

# ── 1. INST transform for brep_idx=7 (NAUO_29477, BREP#10220) ──────────
inst_m = None
for m in re.finditer(
        r"INST brep_idx=(\d+) (\S+)[^\[]*transform=\[([^\]]+)\]", text):
    if m.group(2).startswith("NAUO_29477"):
        nums = [float(x) for x in m.group(3).replace(";", "").split()]
        inst_m = [nums[0:4], nums[4:8], nums[8:12]]
        break
assert inst_m, "INST transform for NAUO_29477 not found"
print(f"INST NAUO_29477: {inst_m}")

R = [inst_m[i][0:3] for i in range(3)]
T = [inst_m[i][3] for i in range(3)]
a, b, c = R[0]; d, e, f_ = R[1]; g, h, i_ = R[2]
det = (a*(e*i_ - f_*h) - b*(d*i_ - f_*g) + c*(d*h - e*g))
print(f"det={det:.6f}")
def inv_apply(p):
    q = [p[k]-T[k] for k in range(3)]
    return (((e*i_ - f_*h)*q[0] - (b*i_ - c*h)*q[1] + (b*f_ - c*e)*q[2]) / det,
            (-(d*i_ - f_*g)*q[0] + (a*i_ - c*g)*q[1] - (a*f_ - c*d)*q[2]) / det,
            ((d*h - e*g)*q[0] - (a*h - b*g)*q[1] + (a*e - b*d)*q[2]) / det)

def dist2(p, q):
    return (p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2

# ── 2. emission files for faces of brep10220 ──────────────────────────
emissions = defaultdict(list)  # fid -> [(att, verts, tris, path)]
for path in sorted(glob.glob(f"{FM}/fm/mesh_brep10220_f*.txt")):
    m = re.match(r"mesh_brep10220_f(\d+)_(.+)_a(\d+)\.txt",
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
      f"(attempts per face: { {k: len(v) for k, v in emissions.items()} })\n")

# ── 3. pair blocks: VERTS line + census line for faces=(28,28) ─────────
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
    if "BREP#10220" not in cl or not cl.startswith("["):
        continue
    fm_ = re.search(r"faces=\((\d+),", cl)
    fid_txt = int(fm_.group(1)) if fm_ else -1
    if fid_txt != 28:          # only face-28 pairs (REAL + companions)
        continue
    cls = re.match(r"\[([A-Z-]+)( [A-Z-]+)?\]", cl)
    tag = cls.group(1) + (cls.group(2) or "")
    blocks.append((int(m.group(1)), int(m.group(5)),
                   [pp(m.group(i)) for i in (2, 3, 4)],
                   [pp(m.group(i)) for i in (6, 7, 8)], tag, cl.strip()))
print(f"face-28 pair blocks: {len(blocks)}\n")

def nearest_emission(lp):
    best = (1e9, None, None, None)
    for f, lst in emissions.items():
        for att, vs, ts, path in lst:
            for vi, q in enumerate(vs):
                dd = dist2(lp, q)
                if dd < best[0]:
                    best = (dd, f, att, vi)
    return best

def tri_in_emissions(lts):
    for f, lst in emissions.items():
        for att, vs, ts, path in lst:
            idxs, ok = [], True
            for lp in lts:
                vi, bd = None, 4e-6  # (2e-3)^2
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
                    return (f, att, ti, idxs)
    return None

# ── 4. attribution per block ──────────────────────────────────────────
for t0i, t1i, t0, t1, tag, cl in blocks:
    is_real = "SUBTOL" not in tag and "COIL-TURN" not in tag \
        and "THIN-FACE" not in tag
    mark = "REAL" if is_real else "exempt"
    print(f"=== T{t0i}/T{t1i} {mark} [{tag}]")
    for tri, nm in ((t0, f"T{t0i}"), (t1, f"T{t1i}")):
        for p in tri:
            lp = inv_apply(p)
            dd, f, att, vi = nearest_emission(lp)
            dn = math.sqrt(dd)
            print(f"  {nm} inst{p} -> local ({lp[0]:.4f},{lp[1]:.4f},{lp[2]:.4f})"
                  f" nearest d={dn:.4f} (f{f} a{att} v{vi})")
    for tri, nm in ((t0, f"T{t0i}"), (t1, f"T{t1i}")):
        lts = [inv_apply(p) for p in tri]
        found = tri_in_emissions(lts)
        if found:
            print(f"  {nm}: EMISSION tri f{found[0]} a{found[1]} t{found[2]}"
                  f" idxs={found[3]}")
        else:
            print(f"  {nm}: NO emission triangle")
    print()

# ── 5. UV polygon of face 28 (ribbon anatomy) ──────────────────────────
print("=== UVPOLY for brep10220_f28 ===")
for path in sorted(glob.glob(f"{FM}/uv/uvpoly_*.txt")):
    head = open(path).readline()
    if "label=brep10220_f28" not in head and "brep10220_f28" not in head:
        continue
    pts_o, holes = [], defaultdict(list)
    for l in open(path):
        if l.startswith("o "):
            _, u, v = l.split(); pts_o.append((float(u), float(v)))
        elif l.startswith("h"):
            hi, u, v = l.split()[0], float(l.split()[1]), float(l.split()[2])
            holes[hi].append((u, v))
    if pts_o:
        us = [p[0] for p in pts_o]; vs = [p[1] for p in pts_o]
        print(f"{os.path.basename(path)}: {head.strip()}")
        print(f"  outer={len(pts_o)} pts, u=[{min(us):.4f},{max(us):.4f}] "
              f"v=[{min(vs):.4f},{max(vs):.4f}], holes={len(holes)}"
              + ("".join(f"\n    hole {k}: n={len(v_)} u=[{min(p[0] for p in v_):.4f},"
                         f"{max(p[0] for p in v_):.4f}] v=[{min(p[1] for p in v_):.4f},"
                         f"{max(p[1] for p in v_):.4f}]"
                         for k, v_ in holes.items())))

# ── 6. FINAL mesh: pair presence + face attribution ────────────────────
print("\n=== FINAL (brep7_NAUO_29477) check ===")
fobj = f"{FM}/final/brep7_NAUO_29477 (BREP#10220).obj"
ffmap = f"{FM}/final/brep7_NAUO_29477 (BREP#10220).fmap"
fvs, fts = [], []
for l in open(fobj):
    if l.startswith("v "):
        _, x, y, z = l.split(); fvs.append((float(x), float(y), float(z)))
fmap = {}
for l in open(ffmap):
    _, ti, fid = l.split(); fmap[int(ti)] = int(fid)
for l in open(fobj):
    if l.startswith("f "):
        _, p, q, r = l.split(); fts.append((int(p)-1, int(q)-1, int(r)-1))
print(f"final mesh: {len(fvs)} verts, {len(fts)} tris")
# REAL pair: triangle indices T2989/T2993 in the INSTANCE mesh = FINAL mesh
for ti in (2989, 2993):
    if ti < len(fts):
        tri = fts[ti]
        print(f"  T{ti}: verts={[fvs[v] for v in tri][:1]}... fmap fid={fmap.get(ti)}"
              f" (vert coords: {[tuple(round(x,4) for x in fvs[v]) for v in tri]})")
    else:
        print(f"  T{ti}: beyond final mesh ({len(fts)} tris)")

# ── 7. stage bisection for the REAL pair (local frame) ────────────────
print("\n=== STAGE bisection (REAL pair, local frame) ===")
real_block = next((b for b in blocks if "SUBTOL" not in b[4]
                   and "COIL-TURN" not in b[4]), None)
if real_block:
    t0l = [inv_apply(p) for p in real_block[2]]
    t1l = [inv_apply(p) for p in real_block[3]]
    for path in sorted(glob.glob(f"{FM}/stages/*.obj"),
                       key=lambda p: os.path.getmtime(p)):
        svs, sts = [], []
        for l in open(path):
            if l.startswith("v "):
                _, x, y, z = l.split(); svs.append((float(x), float(y), float(z)))
            elif l.startswith("f "):
                _, p, q, r = l.split(); sts.append((int(p)-1, int(q)-1, int(r)-1))
        def find_tri(lts):
            idxs = []
            for lp in lts:
                vi, bd = None, 4e-6
                for w, q in enumerate(svs):
                    dd = dist2(lp, q)
                    if dd < bd:
                        vi, bd = w, dd
                if vi is None:
                    return None
                idxs.append(vi)
            s = set(idxs)
            for ti, t in enumerate(sts):
                if set(t) == s:
                    return (ti, idxs)
            return None
        r0, r1 = find_tri(t0l), find_tri(t1l)
        name = os.path.basename(path).replace(".obj", "")
        if r0 or r1:
            print(f"  {name}: T0={'tri#'+str(r0[0]) if r0 else 'absent'}"
                  f" T1={'tri#'+str(r1[0]) if r1 else 'absent'}")
        else:
            print(f"  {name}: pair absent")
