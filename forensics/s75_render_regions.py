#!/usr/bin/env python3
"""s75: visualize the (57,58) and (105,113) fold regions of HOUSING_MIRROR.

Uses the final OBJ (post-repair mesh) + facemap (triangle -> face id).
Renders the junction neighborhoods to see the fan-fold geometry.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
for f in ["/usr/share/fonts/truetype/chinese/NotoSansSC-Regular.ttf",
          "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]:
    try:
        fm.fontManager.addfont(f)
    except Exception:
        pass
import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Noto Sans SC"]
plt.rcParams["axes.unicode_minus"] = False

BASE = "/tmp/s75_objs/brep4_HOUSING_MIRROR (BREP#62542)"

verts = []
tris = []
with open(BASE + ".obj") as f:
    for line in f:
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:4]]
            tris.append(idx)
verts = np.array(verts)
tris = np.array(tris)

# fmap: "t <tri_idx> <face_id>"
tri_face = np.zeros(len(tris), dtype=int)
with open(BASE + ".fmap") as f:
    for line in f:
        if line.startswith("t "):
            _, t, fid = line.split()
            tri_face[int(t)] = int(fid)
print(f"verts={len(verts)} tris={len(tris)} faces={tri_face.max()}")

def face_tris(fid):
    return np.where(tri_face == fid)[0]

def render(fid_a, fid_b, title, fname, focus=None, view="xy"):
    ta, tb = face_tris(fid_a), face_tris(fid_b)
    print(f"face {fid_a}: {len(ta)} tris, face {fid_b}: {len(tb)} tris")
    fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    for ax, (i, j) in zip(axes, [(0, 1), (0, 2)]):
        for tids, color, label, alpha, zord in [
            (ta, "tab:blue", f"face {fid_a}", 0.9, 3),
            (tb, "tab:red", f"face {fid_b}", 0.9, 3),
        ]:
            sel = [t for t in tids
                   if focus is None or
                   (abs(verts[tris[t]][:, i].mean() - focus[0]) < focus[2]
                    and abs(verts[tris[t]][:, j].mean() - focus[1]) < focus[3])]
            for t in sel:
                tri = verts[tris[t]]
                ax.fill(tri[:, i], tri[:, j], color=color, alpha=alpha,
                        lw=0.1, zorder=zord)
        # mark the fan apex / junction vertices if given
        for (px, py, pz), mk, lab in MARKS.get((fid_a, fid_b), []):
            ax.plot([px, py][0] if i == 0 else (py if i == 1 else pz),
                    pz if j == 2 else (py if j == 1 else py),
                    marker=mk, color="black", ms=10, zorder=5)
        ax.set_xlabel("XYZ"[i]); ax.set_ylabel("XYZ"[j])
        ax.set_aspect("equal")
        ax.set_title(f"{title} — {'XY' if (i,j)==(0,1) else 'XZ'} projection")
        ax.legend(handles=[plt.Rectangle((0,0),1,1,color="tab:blue",alpha=.8),
                           plt.Rectangle((0,0),1,1,color="tab:red",alpha=.8)],
                  labels=[f"face {fid_a}", f"face {fid_b}"])
    fig.savefig(fname, dpi=110)
    plt.close(fig)
    print(f"saved {fname}")

MARKS = {}

# (57,58): fan apex at (-1.1092,-0.3497,-0.1000); rim chain near
# (-0.85..-1.04, -1.16..-0.58, 3.51..4.27). Focus on x -1.35..-0.7,
# y -1.5..0.2 full z.
MARKS[(57, 58)] = [
    ((-1.1092, -0.3497, -0.1000), "*", "fan apex 8209"),
]
render(57, 58, "HM faces 57(Plane) vs 58(Nurbs 4x10)",
       "forensics/s75_hm_57_58.png")

# (105,113): folds along x=-0.69, y 1.1..4.5, z 2.0..2.7
MARKS[(105, 113)] = []
render(105, 113, "HM faces 105(Plane) vs 113(Nurbs 9x9)",
       "forensics/s75_hm_105_113.png")
