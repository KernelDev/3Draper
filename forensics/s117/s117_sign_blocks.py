#!/usr/bin/env python3
"""s117: sign-block census of UV polygons for Spit-Fire #27977 Torus faces
(f6=audit#5, f7=audit#6, f65=audit#64, 0-based audit fi = census face - 1).
Mirrors band_ring_split's sgn/blocks logic (custom_cdt.rs):
  sgn(i): dv = v[i+1]-v[i]; >veps -> +1, < -veps -> -1, else 0
  non-neutral cyclic sequence -> rotate to block head -> sign blocks
  band class = exactly 2 blocks with opposite signs.
Also reports the v-extent of each block and the u-monotonicity of the
walls, to judge whether a multi-run extension is feasible.
"""
import re
import sys

POLY_RE = re.compile(r"POLY\s+\d+\s+u=\s*([-\d.eE+]+)\s+v=\s*([-\d.eE+]+)")

def parse_faces(path):
    faces = {}
    cur = None
    for line in open(path):
        if line.startswith("=== FACE #"):
            cur = int(line.split("#")[1].split()[0])
            faces[cur] = []
        elif line.startswith("POLY") and cur is not None:
            m = POLY_RE.match(line)
            if m:
                faces[cur].append((float(m.group(1)), float(m.group(2))))
    return faces

def blocks_of(pts):
    n = len(pts)
    vmin = min(v for _, v in pts)
    vmax = max(v for _, v in pts)
    vspan = vmax - vmin
    if vspan <= 0:
        return None, None, None
    veps = 1e-9 * vspan
    def sgn(i):
        dv = pts[(i + 1) % n][1] - pts[i][1]
        if dv > veps: return 1
        if dv < -veps: return -1
        return 0
    first = next((i for i in range(n) if sgn(i) != 0), None)
    if first is None:
        return 0, vspan, None
    seq = [((first + k) % n, sgn((first + k) % n)) for k in range(n)]
    seq = [(i, s) for i, s in seq if s != 0]
    if len(seq) < 4:
        return 0, vspan, None
    rot_at = 0
    if seq[-1][1] == seq[0][1]:
        last_t = 0
        for k in range(1, len(seq)):
            if seq[k][1] != seq[k - 1][1]:
                last_t = k
        rot_at = last_t
    rot = seq[rot_at:] + seq[:rot_at]
    blocks = []
    s = 0
    for k in range(1, len(rot)):
        if rot[k][1] != rot[k - 1][1]:
            blocks.append((rot[s][1], rot[s][0], rot[k - 1][0]))
            s = k
    blocks.append((rot[s][1], rot[s][0], rot[len(rot) - 1][0]))
    return len(blocks), vspan, blocks

def face_report(name, pts):
    nb, vspan, blocks = blocks_of(pts)
    us = [u for u, _ in pts]
    vs = [v for _, v in pts]
    print(f"\n{name}: n={len(pts)} u=[{min(us):.4f},{max(us):.4f}] "
          f"v=[{min(vs):.4f},{max(vs):.4f}] vspan={vspan:.4f}")
    print(f"  sign blocks = {nb}")
    if blocks:
        for bi, (s, i0, i1) in enumerate(blocks):
            # wall = points from block start edge to block end edge +1
            chain = []
            i = i0
            chain.append(pts[i])
            while i != i1:
                i = (i + 1) % len(pts)
                chain.append(pts[i])
            wall_v = [v for _, v in chain]
            wall_u = [u for u, _ in chain]
            mono_u = all(wall_u[k + 1] > wall_u[k] for k in range(len(wall_u) - 1))
            mono_ud = all(wall_u[k + 1] < wall_u[k] for k in range(len(wall_u) - 1))
            print(f"    block{bi}: sign={s:+d} edges {i0}..{i1} wall_pts={len(chain)} "
                  f"v[{min(wall_v):.4f},{max(wall_v):.4f}] "
                  f"u[{min(wall_u):.4f},{max(wall_u):.4f}] "
                  f"u-mono asc={mono_u} desc={mono_ud}")

faces = parse_faces("forensics/s117/uvaudit_27977.txt")
for fi, census_id in [(5, 6), (6, 7), (64, 65)]:
    if fi in faces:
        face_report(f"audit FACE #{fi} (census f{census_id})", faces[fi])
