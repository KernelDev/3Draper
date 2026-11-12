#!/usr/bin/env python3
"""s132: ring + interior lattice overlay plots (f65, f6)."""
BASE = "/home/z/my-project/scripts/s132/cdtfail"

def parse(path):
    sec = None
    bnd, ints = [], []
    for ln in open(path):
        ln = ln.rstrip("\n")
        if ln == "BOUNDARY": sec = "b"; continue
        if ln == "INTERIOR": sec = "i"; continue
        if ln in ("HOLES", "LEGACY", "CDT"): sec = None; continue
        if not ln or ln.startswith(("n_boundary", "HOLE")): continue
        if sec == "b":
            p = ln.split(); bnd.append((float(p[0]), float(p[1])))
        elif sec == "i":
            p = ln.split(); ints.append((float(p[0]), float(p[1])))
    return bnd, ints

def plot(path, name, W=110, H=34):
    bnd, ints = parse(path)
    us = [p[0] for p in bnd + ints]; vs = [p[1] for p in bnd + ints]
    umin, umax, vmin, vmax = min(us), max(us), min(vs), max(vs)
    print(f"=== {name}  u[{umin:.3f},{umax:.3f}] v[{vmin:.3f},{vmax:.3f}] "
          f"bnd={len(bnd)} int={len(ints)}")
    grid = [[" "]*W for _ in range(H)]
    for (u, v) in ints:
        x = int((u-umin)/(umax-umin+1e-12)*(W-1))
        y = int((1-(v-vmin)/(vmax-vmin+1e-12))*(H-1))
        grid[y][x] = "."
    n = len(bnd)
    step = max(1, n // 30)
    for i, (u, v) in enumerate(bnd):
        x = int((u-umin)/(umax-umin+1e-12)*(W-1))
        y = int((1-(v-vmin)/(vmax-vmin+1e-12))*(H-1))
        grid[y][x] = "#"
    for i in range(0, n, step):
        u, v = bnd[i]
        x = int((u-umin)/(umax-umin+1e-12)*(W-1))
        y = int((1-(v-vmin)/(vmax-vmin+1e-12))*(H-1))
        grid[y][x] = "0123456789"[(i//step) % 10]
    print("    " + "\n    ".join("".join(r) for r in grid))
    print()

for nm in ("brep27977_f65_Torus", "brep27977_f6_Torus"):
    plot(f"{BASE}/{nm}.cdtfail.txt", nm)
