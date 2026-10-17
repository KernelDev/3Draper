#!/usr/bin/env python3
"""s99: track the 8 REAL fold pairs of drill (HOUSING 3 + HM 5) through
pipeline stages (d-after-merge -> d-after-postweldwind) to attribute
each pair to EMISSION vs WELD/REPAIR.

Method (s98 lessons):
  - final OBJs are WORLD coords; stage OBJs are BREP-LOCAL.
  - HOUSING/HM instance transform: wx=ly, wy=lz+5.4, wz=lx+2.325
    => local: lx=wz-2.325, ly=wx, lz=wy-5.4
  - match triangle geometry by nearest-vertex (welding moves verts
    by up to ~merge_tol, so allow fuzzy match, record distances).
"""
import re
import sys
from pathlib import Path

LOG = Path("/tmp/s99_drill_full.txt")
STAGE_DIR = Path("/tmp/s99/stages")
FINAL_DIR = Path("/tmp/s99/final")

# brep -> (passes, w2l suffix)
BREP_PASSES = {3: (6, 7), 4: (8, 9)}
W2L = lambda p: (p[2] - 2.325, p[0], p[1] - 5.4)

STAGES = ["d-after-merge", "d-after-weld", "d-after-winding", "d-after-tj",
          "d-after-gapfill", "d-after-flapclean", "d-after-postweldwind"]

pair_re = re.compile(
    r"^\[([A-Z-]+)\s*(SUBTOL)?\] brep_idx=(\d+) (\S+) .*?"
    r"faces=\((\d+),(\d+)\).*?tris=\((\[[^\]]*\]),(\[[^\]]*\])\)")
verts_re = re.compile(r"VERTS T(\d+)=\[(.*?)\] T(\d+)=\[(.*?)\]")
pt_re = re.compile(r"\(([-\d.e]+),([-\d.e]+),([-\d.e]+)\)")


def parse_pts(s):
    return [tuple(float(g) for g in m.groups()) for m in pt_re.finditer(s)]


def load_obj(path):
    vs, ts = [], []
    for ln in path.read_text().splitlines():
        if ln.startswith("v "):
            _, x, y, z = ln.split()
            vs.append((float(x), float(y), float(z)))
        elif ln.startswith("f "):
            a, b, c = (int(t.split("/")[0]) - 1 for t in ln.split()[1:4])
            ts.append((a, b, c))
    return vs, ts


def load_fmap(path):
    fm = {}
    for ln in path.read_text().splitlines():
        parts = ln.split()
        if len(parts) == 3 and parts[0] == "t":
            fm[int(parts[1])] = int(parts[2])
    return fm


def match_tri(stage_vs, tri_local, tol):
    """Find stage triangles whose 3 verts are within tol of tri_local
    (any vertex order). Return list of (tri_idx, max_dist)."""
    # nearest stage vertex for each of our 3 points
    idxs = []
    for p in tri_local:
        best, bd = -1, 1e18
        for i, q in enumerate(stage_vs):
            d = sum((a - b) ** 2 for a, b in zip(p, q))
            if d < bd:
                bd, best = d, i
        idxs.append((best, bd ** 0.5))
    return idxs


def stage_dihedral(obj_path, l0, l1):
    """Load stage obj once; find the two triangles by nearest-vertex match;
    return (dihedral_deg_as_wound, d0, d1, i0, i1) or None."""
    vs, ts = load_obj(obj_path)
    res = []
    for tri_l in (l0, l1):
        idxs = match_tri(vs, tri_l, 0.05)
        key = tuple(sorted(i for i, _ in idxs))
        hit = None
        for ti, t in enumerate(ts):
            if tuple(sorted(t)) == key:
                hit = (ti, t)
                break
        res.append((hit, max(d for _, d in idxs)))
    if res[0][0] is None or res[1][0] is None:
        return None
    import math
    def normal(t):
        p = [vs[t[0]], vs[t[1]], vs[t[2]]]
        e1 = [p[1][k] - p[0][k] for k in range(3)]
        e2 = [p[2][k] - p[0][k] for k in range(3)]
        n = [e1[1] * e2[2] - e1[2] * e2[1],
             e1[2] * e2[0] - e1[0] * e2[2],
             e1[0] * e2[1] - e1[1] * e2[0]]
        l = math.sqrt(sum(x * x for x in n))
        return [x / l for x in n] if l > 1e-15 else None
    n0 = normal(res[0][0][1])
    n1 = normal(res[1][0][1])
    if n0 is None or n1 is None:
        return None
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(n0, n1))))
    return (math.degrees(math.acos(dot)), res[0][1], res[1][1],
            res[0][0][0], res[1][0][0])


def main():
    lines = LOG.read_text().splitlines()
    pairs = []
    for i, ln in enumerate(lines):
        m = pair_re.match(ln)
        if not m:
            continue
        tag, subtol, brep, name, f0, f1, _, _ = m.groups()
        if subtol or tag.startswith("KNIFE") or tag.startswith("TANGENT"):
            continue
        brep = int(brep)
        if brep not in BREP_PASSES:
            continue
        vm = None
        for j in range(i + 1, min(i + 4, len(lines))):
            vm = verts_re.search(lines[j])
            if vm:
                break
        if not vm:
            continue
        t0, pts0, t1, pts1 = vm.groups()
        pairs.append(dict(brep=brep, name=name, faces=(int(f0), int(f1)),
                          tag=tag, t=(int(t0), int(t1)),
                          tris=(pts0, pts1),
                          v0=parse_pts(pts0), v1=parse_pts(pts1)))

    print(f"REAL pairs found: {len(pairs)}")
    for p in pairs:
        print(f"\n=== brep{p['brep']} {p['name']} faces={p['faces']} "
              f"tag={p['tag']} T={p['t']}")
        # local coords
        l0 = [W2L(v) for v in p["v0"]]
        l1 = [W2L(v) for v in p["v1"]]
        for pv in (p0 for p0 in (l0, l1)):
            print("   local:", " ".join(f"({x:.4f},{y:.4f},{z:.4f})" for x, y, z in pv))
        # track through stages (winding-aware: as-stored dihedral per stage)
        for ps_idx, pass_no in enumerate(BREP_PASSES[p["brep"]]):
            row = []
            for st in STAGES:
                obj = STAGE_DIR / f"p{pass_no}_{st}.obj"
                if not obj.exists():
                    row.append("--")
                    continue
                r = stage_dihedral(obj, l0, l1)
                if r is None:
                    row.append("notri")
                else:
                    ang, d0, d1, i0, i1 = r
                    row.append(f"{ang:6.1f}")
            print(f"   pass{pass_no} dihedral: " + " ".join(
                f"{st.replace('d-after-', '')[:8]}={r}" for st, r in zip(STAGES, row)))


_tri_cache = {}


def _tris_cache(obj_path):
    if obj_path not in _tri_cache:
        _, ts = load_obj(obj_path)
        _tri_cache[obj_path] = ts
    return _tri_cache[obj_path]


if __name__ == "__main__":
    main()
