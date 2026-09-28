#!/usr/bin/env python3
"""s64 diag: tessellation comparison of shared arc circle#5684 between
f29 (cylinder) and f3 (plane) — per-face meshes, local frame.

Also circle#5737 (f29/f12) and the f32 mirrors.  For each shared circle,
collect per-face boundary verts lying EXACTLY on the circle (radial
residual < 1e-6), print their angle lists, and diff.
"""
import math
import re
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

def bnd_verts(tris):
    cnt = defaultdict(int)
    for (a, b, c) in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            e = (u, v) if u < v else (v, u)
            cnt[e] += 1
    bv = set()
    for e, n in cnt.items():
        if n == 1:
            bv.add(e[0]); bv.add(e[1])
    return bv

def main():
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
        t = ents[curve_id]
        refs = [int(x) for x in re.findall(r"#(\d+)", t[1])]
        ax = ents[refs[0]]
        arefs = [int(x) for x in re.findall(r"#(\d+)", ax[1])]
        return vec(arefs[0]), vec(arefs[1]), vec(arefs[2])

    circles = {  # name -> (loc, axis, refd, r)
        "c5684(f3|f29)": (821, 3.0),
        "c5690(f3|f32)": (824, 3.0),
        "c5737(f12|f29)": (839, 3.0),
        "c5741(f12|f32)": (841, 3.0),
    }
    C = {}
    for name, (cid, r) in circles.items():
        loc, axis, refd = circle_params(cid)
        C[name] = (loc, axis, refd, r)
        print(f"{name}: loc={loc} axis={axis} ref={refd} r={r}")

    def ang(name, pt):
        loc, axis, refd, r = C[name]
        rel = [pt[i] - loc[i] for i in range(3)]
        axial = sum(rel[i] * axis[i] for i in range(3))
        u = refd
        v = (axis[1] * u[2] - axis[2] * u[1],
             axis[2] * u[0] - axis[0] * u[2],
             axis[0] * u[1] - axis[1] * u[0])
        du = sum(rel[i] * u[i] for i in range(3))
        dv = sum(rel[i] * v[i] for i in range(3))
        a = math.degrees(math.atan2(dv, du)) % 360
        radial = math.sqrt(max(0.0, sum(x * x for x in rel) - axial * axial))
        return a, radial, axial

    faces = {
        "f29": "brep1092_f29_s1831_Cylinder.obj",
        "f32": "brep1092_f32_s1834_Cylinder.obj",
        "f3": "brep1092_f3_s1805_Plane.obj",
        "f12": "brep1092_f12_s1814_Plane.obj",
    }
    per_face_arc = {}
    for fname, ffile in faces.items():
        verts, tris = load(f"{D}/{ffile}")
        allv = verts  # classify ALL verts (not just boundary)
        per_face_arc[fname] = {}
        for name in C:
            hits = []
            for pt in allv:
                a, radial, axial = ang(name, pt)
                if abs(radial - 3.0) < 1e-4 and abs(axial) < 1e-4:
                    hits.append(round(a, 3))
            hits.sort()
            per_face_arc[fname][name] = hits
            if hits:
                print(f"\n{fname} on {name}: {len(hits)} verts exact-on-circle")
                print(f"  angles: {hits}")

    # diff the shared arcs
    print("\n=== SHARED ARC TESSELLATION DIFF ===")
    for fa, fb, name in [("f29", "f3", "c5684(f3|f29)"),
                         ("f32", "f3", "c5690(f3|f32)"),
                         ("f29", "f12", "c5737(f12|f29)"),
                         ("f32", "f12", "c5741(f12|f32)")]:
        A = set(per_face_arc[fa][name])
        B = set(per_face_arc[fb][name])
        common = sorted(A & B)
        onlyA = sorted(A - B)
        onlyB = sorted(B - A)
        print(f"\n{name}: {fa}={len(A)} {fb}={len(B)} common={len(common)} "
              f"only{fa}={len(onlyA)} only{fb}={len(onlyB)}")
        if onlyA:
            print(f"  only {fa}: {onlyA[:12]}{'...' if len(onlyA) > 12 else ''}")
        if onlyB:
            print(f"  only {fb}: {onlyB[:12]}{'...' if len(onlyB) > 12 else ''}")

if __name__ == "__main__":
    main()
