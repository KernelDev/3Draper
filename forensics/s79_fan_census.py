#!/usr/bin/env python3
"""s79: fan census on the class-A Plane faces (HM + HOUSING per-face
OBJs): max vertex degree, pole count, thinness — confirms the
ear_clip mega-fan pattern across the family."""
import math
import os
from collections import defaultdict

DIR = "/tmp/s79_objs"
TARGETS = {
    "brep62542_f3_Plane": "HM f3 (3,121)=30",
    "brep62542_f31_Plane": "HM f31 (26,31)",
    "brep62542_f57_Plane": "HM f57 (57,58)=43",
    "brep62542_f147_Plane": "HM f147 (147,148)=30",
    "brep47598_f144_Plane": "HOUS f144 (144,145)=22",
    "brep47598_f178_Plane": "HOUS f178 (49,178)=37",
    "brep47598_f12_Plane": "HOUS f12 (3,12)mirror",
}


def analyze(path, label):
    verts, tris = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                a, b, c = [int(x) for x in line.split()[1:4]]
                tris.append((a - 1, b - 1, c - 1))
    m = len(verts)
    deg = defaultdict(int)
    for t in tris:
        for v in t:
            deg[v] += 1
    top = sorted(deg.items(), key=lambda kv: -kv[1])[:3]
    # thinness (project on best-fit plane via two largest PCA axes —
    # for plane faces the mesh IS planar: use x,z or x,y by extent)
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    ext = [(max(xs)-min(xs), 'x'), (max(ys)-min(ys), 'y'),
           (max(zs)-min(zs), 'z')]
    ext.sort()
    # drop the thinnest axis
    a1, a2 = ext[1][1], ext[2][1]
    idx = {'x': 0, 'y': 1, 'z': 2}
    p2 = [(v[idx[a1]], v[idx[a2]]) for v in verts]
    area2 = 0.0
    perim = 0.0
    n = len(p2)
    for i in range(n):
        j = (i + 1) % n
        area2 += p2[i][0]*p2[j][1] - p2[j][0]*p2[i][1]
        perim += math.dist(p2[i], p2[j])
    semi = perim / 2
    thin = abs(area2) / 2 / (semi * semi) if semi > 0 else 0
    print(f"{label}: m={m} tris={len(tris)} "
          f"top-deg={[(d, v) for v, d in top]} "
          f"thinness~{thin:.4f} axes={a1}{a2}")


for key, label in TARGETS.items():
    # find the file
    for fn in os.listdir(DIR):
        if fn.startswith(key.split("_Plane")[0]) and "Plane" in fn:
            analyze(os.path.join(DIR, fn), label)
            break
    else:
        print(f"{label}: OBJ NOT FOUND")
