#!/usr/bin/env python3
"""s127 п.1a: correlate UVPOLY dumps with brep10212 face emissions.

Method: for each Cylinder UVPOLY with holes, fit the cylinder from the
face emission (SURF_PARAMS would be better but we do geometric match):
emission verts -> guess axis via PCA is overkill.

Simpler robust method: for a cylinder param (u,v): x = o + r*(cos u * X + sin u * Y) + v * A.
The UVPOLY outer ring maps to the emission boundary IF we know (o, r, A, X, Y).
Instead of fitting, use INVARIANTS: chord lengths. Consecutive UV points
(u_i, v_i), (u_{i+1}, v_{i+1}): 3D chord^2 = r^2 * du_wrap^2 + dv^2 (orthonormal
frame, du wrapped to [-pi,pi]). So the multiset of (du, dv) pairs of the UVPOLY
must be consistent with SOME cylinder; and the emission boundary walk gives
3D chords. Match ratio: 3D chord lengths vs sqrt(r^2 du^2 + dv^2) for a range
of r — find best r per candidate; the true face's polygon fits with tiny
residual for ONE r, and its v-range matches emission extent along axis.

Even simpler decisive test: face 2's emission boundary has 728 directed
boundary edges (364 undirected). The UVPOLY outer+holes vertex count for
the true face must be <= emission verts and the 3D boundary total length
must match sum of chord lengths. We fit r by least squares on total length.
"""
import glob, math, re, sys
from collections import defaultdict

def load_uv(path):
    outer, holes = [], defaultdict(list)
    for l in open(path):
        if l.startswith("#"):
            continue
        parts = l.split()
        if not parts:
            continue
        if parts[0] == "o":
            outer.append((float(parts[1]), float(parts[2])))
        elif parts[0].startswith("h"):
            holes[int(parts[0][1:])].append((float(parts[1]), float(parts[2])))
    return outer, [holes[k] for k in sorted(holes)]

def wrap_pi(x):
    return (x + math.pi) % (2 * math.pi) - math.pi

def uv_chords(pts):
    out = []
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        du = wrap_pi(pts[j][0] - pts[i][0])
        dv = pts[j][1] - pts[i][1]
        out.append((du, dv))
    return out

def load_emission_boundary(path):
    """boundary undirected edge lengths + total + vert count"""
    verts, tris = [], []
    for l in open(path):
        if l.startswith("v "):
            p = l.split()
            verts.append(tuple(float(x) for x in p[2:5]))
        elif l.startswith("t "):
            p = l.split()
            tris.append((int(p[2]), int(p[3]), int(p[4])))
    used = defaultdict(int)
    for t in tris:
        for k in range(3):
            a, b = t[k], t[(k+1) % 3]
            used[(a, b)] += 1
    bnd = set()
    for (a, b), c in used.items():
        if c == 1 and (b, a) not in used:
            bnd.add((min(a, b), max(a, b)))
    lengths = []
    for a, b in bnd:
        dx = verts[a][0]-verts[b][0]; dy = verts[a][1]-verts[b][1]; dz = verts[a][2]-verts[b][2]
        lengths.append(math.sqrt(dx*dx+dy*dy+dz*dz))
    return len(verts), len(bnd), sum(lengths)

def fit_radius(chords, target_len):
    """find r minimizing |sum sqrt(r^2 du^2 + dv^2) - target_len|"""
    lo, hi = 0.01, 50.0
    def total(r):
        return sum(math.sqrt(r*r*du*du + dv*dv) for du, dv in chords)
    for _ in range(80):
        mid = (lo + hi) / 2
        if total(mid) < target_len:
            lo = mid
        else:
            hi = mid
    r = (lo + hi) / 2
    return r, total(r)

if __name__ == "__main__":
    uvdir = "forensics/s127/uv10212"
    emis = {
        "f2":  "forensics/s126/fm10212/mesh_brep10212_f2_Cylinder_a1.txt",
        "f3":  "forensics/s126/fm10212/mesh_brep10212_f3_Cylinder_a2.txt",
        "f4":  "forensics/s126/fm10212/mesh_brep10212_f4_Cylinder_a3.txt",
        "f19": "forensics/s126/fm10212/mesh_brep10212_f19_Cylinder_a18.txt",
    }
    einfo = {}
    for k, p in emis.items():
        einfo[k] = load_emission_boundary(p)
        print(f"{k}: verts={einfo[k][0]} boundary_edges={einfo[k][1]} total_bnd_len={einfo[k][2]:.4f}")
    print()
    # candidates: Cylinder UVPOLYs
    for path in sorted(glob.glob(f"{uvdir}/uvpoly_*.txt")):
        head = open(path).readline()
        if 'surface="Cylinder"' not in head:
            continue
        outer, holes = load_uv(path)
        if not holes:
            continue
        chords = uv_chords(outer)
        for h in holes:
            chords += uv_chords(h)
        for k, (nv, nb, tl) in einfo.items():
            r, tot = fit_radius(chords, tl)
            # residual per boundary edge
            per_edge = abs(tot - tl) / max(nb, 1)
            if per_edge < 0.02 and len(outer) + (len(holes) if holes else 0) <= nv:
                print(f"{path.split('/')[-1]} -> {k}: outer={len(outer)} holes={len(holes)} "
                      f"fit_r={r:.4f} tot={tot:.3f} target={tl:.3f} per-edge-resid={per_edge:.5f}")
