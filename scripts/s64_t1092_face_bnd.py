#!/usr/bin/env python3
"""s64 diag: per-face mesh boundary of #1092 f29/f32/f3/f12 vs STEP curves.

For each dumped per-face mesh (LOCAL frame, matches STEP):
- boundary edges/verts
- classify each boundary vertex: which STEP edge curve it lies on
  (circle#5684/ellipse#5753/circle#5737/lines..., quarter-arc check)
- census per curve: n_verts, param range (angle on circle)
"""
import math
import re
import sys
from collections import defaultdict

D = "/tmp/s64_faces"

def load(path):
    verts, tris = [], []
    for line in open(path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            tris.append(tuple(int(p) - 1 for p in line.split()[1:4]))
    return verts, tris

def bnd_edges(tris):
    cnt = defaultdict(int)
    for (a, b, c) in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            e = (u, v) if u < v else (v, u)
            cnt[e] += 1
    return [e for e, n in cnt.items() if n == 1], cnt

def main():
    faces = {
        "f29": "brep1092_f29_s1831_Cylinder.obj",
        "f32": "brep1092_f32_s1834_Cylinder.obj",
        "f3": "brep1092_f3_s1805_Plane.obj",
        "f12": "brep1092_f12_s1814_Plane.obj",
        "f1": "brep1092_f1_s1803_Plane.obj",
    }
    # STEP curves (local frame): from surface placements
    # f29 cylinder: axis@ (17,40,42) dir (0,1,0), ref (0,0,1), r=3
    # circle#5684: CIRCLE('',#7384,3.) — get placement
    data = open("/home/z/my-project/3Draper/test/Zentralstaender.stp",
                encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    def vec(ref):
        t = ents[ref]
        nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+(?:E[-+]?[0-9]+)?", t[1])
        return tuple(float(x) for x in nums[:3])

    def circle_params(curve_id):
        # CIRCLE('',#axis2, r)
        t = ents[curve_id]
        refs = [int(x) for x in re.findall(r"#(\d+)", t[1])]
        ax = ents[refs[0]]
        arefs = [int(x) for x in re.findall(r"#(\d+)", ax[1])]
        loc = vec(arefs[0]); axis = vec(arefs[1]); refd = vec(arefs[2])
        r = float(re.findall(r"[-+]?[0-9]*\.?[0-9]+", t[1].split(",")[-1])[0])
        return loc, axis, refd, r

    curves = {}
    for cid, name in [(821, "circle#5684"), (839, "circle#5737"),
                      (824, "circle#5690"), (841, "circle#5741"),
                      (458, "ellipse#5753"), (6230, "line#5673"),
                      (6245, "line#5694"), (6273, "line#5733")]:
        t = ents[cid]
        if t[0] == "CIRCLE":
            loc, axis, refd, r = circle_params(cid)
            curves[name] = ("CIRCLE", loc, axis, refd, r)
            print(f"{name}: CIRCLE loc={loc} axis={axis} ref={refd} r={r}")
        elif t[0] == "ELLIPSE":
            refs = [int(x) for x in re.findall(r"#(\d+)", t[1])]
            ax = ents[refs[0]]
            arefs = [int(x) for x in re.findall(r"#(\d+)", ax[1])]
            loc = vec(arefs[0]); axis = vec(arefs[1]); refd = vec(arefs[2])
            nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", t[1])
            curves[name] = ("ELLIPSE", loc, axis, refd,
                            float(nums[-2]), float(nums[-1]))
            print(f"{name}: ELLIPSE loc={loc} axis={axis} ref={refd} "
                  f"a={nums[-2]} b={nums[-1]}")
        elif t[0] == "LINE":
            refs = [int(x) for x in re.findall(r"#(\d+)", t[1])]
            p = vec(refs[0])
            dref = ents[refs[1]]
            if dref[0] == "VECTOR":
                inner = [int(x) for x in re.findall(r"#(\d+)", dref[1])][0]
                dref = ents[inner]
            nums_d = re.findall(r"[-+]?[0-9]*\.?[0-9]+(?:E[-+]?[0-9]+)?", dref[1])
            d = tuple(float(x) for x in nums_d[:3])
            # normalize
            n = math.sqrt(sum(x * x for x in d)) or 1.0
            d = tuple(x / n for x in d)
            curves[name] = ("LINE", p, d)
            print(f"{name}: LINE p={p} d={d}")

    def angle_on_circle(name, pt):
        typ, loc, axis, refd, r = curves[name]
        # project pt-loc onto plane, get angle from refd
        rel = [pt[i] - loc[i] for i in range(3)]
        # basis: refd (u), refd × axis (v)
        u = refd
        v = tuple(axis[i] * refd[(i + 1) % 3] - axis[(i + 1) % 3] * refd[i]
                  for i in range(3))
        # v = axis × u
        v = (axis[1] * u[2] - axis[2] * u[1],
             axis[2] * u[0] - axis[0] * u[2],
             axis[0] * u[1] - axis[1] * u[0])
        dot_u = sum(rel[i] * u[i] for i in range(3))
        dot_v = sum(rel[i] * v[i] for i in range(3))
        ang = math.degrees(math.atan2(dot_v, dot_u)) % 360
        dist_c = math.sqrt(sum(x * x for x in rel) -
                           (sum(rel[i] * axis[i] for i in range(3)) ** 2))
        return ang, dist_c

    for fname, ffile in faces.items():
        verts, tris = load(f"{D}/{ffile}")
        bed, cnt = bnd_edges(tris)
        bv = sorted({v for e in bed for v in e})
        print(f"\n== {fname}: v={len(verts)} t={len(tris)} bnd_edges={len(bed)} bnd_verts={len(bv)}")
        if not bv:
            continue
        # classify each boundary vertex to nearest curve
        cls = defaultdict(list)
        for v in bv:
            pt = verts[v]
            best, bestd = None, 1e9
            for name, c in curves.items():
                if c[0] == "CIRCLE":
                    _, loc, axis, refd, r = c
                    rel = [pt[i] - loc[i] for i in range(3)]
                    axial = sum(rel[i] * axis[i] for i in range(3))
                    radial = math.sqrt(max(0, sum(x * x for x in rel) - axial * axial))
                    d = abs(radial - r) + abs(axial) * 0.1
                elif c[0] == "ELLIPSE":
                    _, loc, axis, refd, a, b = c
                    rel = [pt[i] - loc[i] for i in range(3)]
                    axial = sum(rel[i] * axis[i] for i in range(3))
                    radial = math.sqrt(max(0, sum(x * x for x in rel) - axial * axial))
                    d = abs(radial - max(a, b)) + abs(axial) * 0.1
                else:
                    _, p, dd = c
                    rel = [pt[i] - p[i] for i in range(3)]
                    tpar = sum(rel[i] * dd[i] for i in range(3))
                    perp2 = sum(x * x for x in rel) - tpar * tpar
                    d = math.sqrt(max(0, perp2))
                if d < bestd:
                    bestd, best = d, name
            if bestd < 0.3:
                cls[best].append(pt)
            else:
                cls["?"].append(pt)
        for name in sorted(cls):
            pts = cls[name]
            if name in curves and curves[name][0] == "CIRCLE":
                angs = sorted(angle_on_circle(name, p)[0] for p in pts)
                rr = [angle_on_circle(name, p)[1] for p in pts]
                print(f"  {name}: {len(pts)} verts, angle [{angs[0]:.1f}..{angs[-1]:.1f}]deg, |r-r_c|={max(abs(x - curves[name][4]) for x in rr):.4f}")
            else:
                print(f"  {name}: {len(pts)} verts")

if __name__ == "__main__":
    main()
