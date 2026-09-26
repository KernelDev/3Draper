#!/usr/bin/env python3
"""session-61: plot the GEAR f18/f20 cone-band UV rings."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
fm.fontManager.addfont('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Noto Sans SC']
plt.rcParams['axes.unicode_minus'] = False

def load(path):
    ring, lat = [], []
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) != 3:
                continue
            kind, u, v = parts[0], float(parts[1]), float(parts[2])
            if kind == "ring":
                ring.append((u, v))
            elif kind == "lat":
                lat.append((u, v))
    return ring, lat

fig, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
for ax, fid in zip(axes, (18, 20)):
    ring, lat = load(f"/tmp/ring_brep16033_f{fid}_Cone.tsv")
    us = [p[0] for p in ring] + [ring[0][0]]
    vs = [p[1] for p in ring] + [ring[0][1]]
    ax.plot(us, vs, "-", lw=0.6, color="#1a6faf", label=f"ring ({len(ring)} pts)")
    if lat:
        ax.plot([p[0] for p in lat], [p[1] for p in lat], "o", ms=4,
                color="#d62728", label=f"lattice ({len(lat)} pts)")
    ax.set_title(f"GEAR f{fid} Cone — UV ring")
    ax.set_xlabel("u (azimuth)")
    ax.set_ylabel("v (profile height)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
fig.savefig("/tmp/gear_f18_f20_rings.png", dpi=110)
print("saved /tmp/gear_f18_f20_rings.png")

# zoom on 2 teeth of f18
fig2, ax2 = plt.subplots(figsize=(12, 4), constrained_layout=True)
ring, lat = load("/tmp/ring_brep16033_f18_Cone.tsv")
us = [p[0] for p in ring] + [ring[0][0]]
vs = [p[1] for p in ring] + [ring[0][1]]
ax2.plot(us, vs, "-", lw=0.8, color="#1a6faf")
ax2.plot([p[0] for p in lat], [p[1] for p in lat], "o", ms=5, color="#d62728")
ax2.set_xlim(0.0, 0.9)
ax2.set_title("GEAR f18 — zoom: first 2 teeth (u in [0, 0.9])")
ax2.set_xlabel("u")
ax2.set_ylabel("v")
ax2.grid(True, alpha=0.3)
fig2.savefig("/tmp/gear_f18_zoom.png", dpi=110)
print("saved /tmp/gear_f18_zoom.png")
