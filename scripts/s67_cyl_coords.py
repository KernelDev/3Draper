#!/usr/bin/env python3
"""s67 diag 8: cylindrical coords of chain verts around each face's
own surface axis. Shows WHERE on the surface the open chain sits
(rim? mid-band? seam?) and which curve it follows.
"""
import math
import re
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP, FACEMAP = (BASE + ".obj", BASE + ".fmap",
                      BASE + ".facemap")
STP = "/home/z/my-project/3Draper/test/drill_top.stp"
# fid -> (axis_pos, axis_dir, R_major, r_minor)
TARGETS = [125, 214, 212, 127, 158, 160, 162, 164, 166,
           196, 198, 200, 202, 204, 236, 238, 130, 261, 14, 12, 69]


def step_surfaces():
    data = open(STP, errors="replace").read()
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;",
                         data, re.S):
        ents[int(m.group(1))] = (m.group(2), m.group(3))
    out = {}
    fid2step = {}
    for line in open(FACEMAP):
        if line.startswith("f "):
            p = line.split()
            fid2step[int(p[1])] = int(p[-2])
    for fid, sid in fid2step.items():
        if sid not in ents or fid not in TARGETS:
            continue
        refs = [int(x) for x in re.findall(r"#(\d+)", ents[sid][1])]
        for r in refs:
            if r in ents:
                t, a = ents[r]
                if t in ("CYLINDRICAL_SURFACE", "TOROIDAL_SURFACE"):
                    inner = [int(x) for x in
                             re.findall(r"#(\d+)", a)]
                    ax = ents.get(inner[0], (None, None))
                    # AXIS2_PLACEMENT: location point, direction
                    loc_ref = [int(x) for x in
                               re.findall(r"#(\d+)", ax[1])]
                    loc = ents.get(loc_ref[0], (None, "0.,0.,0."))
                    dirr = ents.get(loc_ref[1], (None, "0.,0.,1."))
                    pos = tuple(float(v) for v in
                                re.findall(r"-?\d+\.?\d*E?-?\d*",
                                           loc[1].replace("E", "E")))
                    d = tuple(float(v) for v in
                              re.findall(r"-?\d+\.?\d*E?-?\d*",
                                         dirr[1].replace("E", "E")))
                    nums = [float(v) for v in
                            re.findall(r"-?\d+\.?\d*E?-?\d*",
                                       a.split(",", 2)[-1])]
                    out[fid] = (pos, d, nums, t)
    return out


def main():
    verts, tris = [], []
    for line in open(OBJ):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(t) - 1 for t in line.split()[1:4])
            tris.append((a, b, c))
    fid_of = {}
    for line in open(FMAP):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of[int(ti)] = int(fid)
    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            edge_tris[(min(u, v), max(u, v))].append(ti)
    face_bnd = defaultdict(list)
    for e, ts in edge_tris.items():
        if len(ts) == 1:
            face_bnd[fid_of.get(ts[0], -1)].append(e)

    surf = step_surfaces()
    # instance transform (world = M*local):  xw=y, yw=z+5.4, zw=x+2.325
    def to_local(p):
        return (p[2] - 2.325, p[0], p[1] - 5.4)
    for fid in TARGETS:
        if fid not in surf:
            continue
        (pos, d, nums, st) = surf[fid]
        dl = math.sqrt(sum(x * x for x in d))
        d = tuple(x / dl for x in d)
        # pos/d are already in BREP-LOCAL coords — do NOT transform
        edges = face_bnd.get(fid, [])
        vs = set()
        for (u, v) in edges:
            vs.add(u); vs.add(v)
        rs, zs, angs = [], [], []
        for vi in vs:
            p = to_local(verts[vi])
            rel = tuple(p[k] - pos[k] for k in range(3))
            z = sum(rel[k] * d[k] for k in range(3))
            radial = tuple(rel[k] - z * d[k] for k in range(3))
            r = math.sqrt(sum(x * x for x in radial))
            rs.append(r); zs.append(z)
            angs.append(math.atan2(radial[1], radial[0]))
        rs.sort(); zs.sort()
        span = max(angs) - min(angs)
        Rmaj = nums[0] if st.startswith("TOROID") else nums[0]
        rmin = (nums[1] if st.startswith("TOROID") else None)
        print(f"f{fid:4d} {st[:9]:9s} R={Rmaj}"
              + (f" r={rmin}" if rmin else "")
              + f" | {len(vs):4d} chain-verts: "
              f"r[{rs[0]:.4f},{rs[-1]:.4f}] "
              f"z[{zs[0]:.4f},{zs[-1]:.4f}] "
              f"ang-span={math.degrees(span if span < 6.2 else span - 2*math.pi if span > 6.2 else span):.1f}°")


if __name__ == "__main__":
    main()
