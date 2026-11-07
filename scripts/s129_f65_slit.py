#!/usr/bin/env python3
"""s129 п.1: #27977 f65 — decisive emission-vs-CDT analysis.

Inputs (from the live s129 dump):
  TRI   = forensics/s129/fm27977/tri/tri_0043_Torus.txt  (f65 first pass:
          279 bnd + 349 interior, 639 tris — the ASSEMBLED triangle_indices
          BEFORE Step5/5.5/6)
  FM    = forensics/s129/fm27977/fm/mesh_brep27977_f65_Torus_a64.txt
          (the SHIPPED first-pass emission: 495 verts / 707 tris)

Question: where do +68 tris come from, and which code created the
one-sided notch (11 non-rim boundary edges)? Method:
  1. Build position-keyed point sets from both files.
  2. Classify a64 boundary edges: rim (both endpoints match TRI boundary
     'b' points) vs slit (non-rim).
  3. For every a64 triangle, check membership in the CDT tri set
     (position-keyed triple). Report: CDT-native / ADDED (not in CDT).
  4. For ADDED triangles: check whether their vertices are CDT points
     (b/i) or NEW positions (Step6 midpoints / Step5.5 gap-fill).
"""
import re, sys
from collections import defaultdict

BASE = "/home/z/my-project/3Draper"
TRI = f"{BASE}/forensics/s129/fm27977/tri/tri_0043_Torus.txt"
FM = f"{BASE}/forensics/s129/fm27977/fm/mesh_brep27977_f65_Torus_a64.txt"


def pk(x, y, z):
    return (round(x, 9), round(y, 9), round(z, 9))


# ── parse TRI_INPUT: sections boundary / holes / interior / appended / tris
tri_pts = {}          # idx -> poskey  (index into all_uv)
tri_bnd_keys = set()  # poskeys of boundary 'b' points
tri_int_keys = set()
cdt_tris = set()      # frozenset of 3 poskeys
section = None
for ln in open(TRI):
    if ln.startswith("type="):
        print("TRI header:", ln.strip())
        continue
    if ln.startswith("boundary"):
        section = "b"; continue
    if ln.startswith("holes"):
        section = "h"; continue
    if ln.startswith("interior"):
        section = "i"; continue
    if ln.startswith("appended"):
        section = "a"; continue
    if ln.startswith("tris"):
        section = "t"; continue
    if section == "b" and ln[0] == "b":
        _, u, v, x, y, z = ln.split()
        tri_bnd_keys.add(pk(float(x), float(y), float(z)))
    elif section == "i" and ln[0] == "i":
        _, u, v, x, y, z = ln.split()
        tri_int_keys.add(pk(float(x), float(y), float(z)))
    elif section == "a" and ln[0] == "a":
        _, u, v, x, y, z = ln.split()
        tri_int_keys.add(pk(float(x), float(y), float(z)))  # appended = interior-class
    elif section == "t" and ln[0] == "t":
        _, a, b, c = ln.split()
        # indices into all_uv: need positions — TRI_INPUT 'tris' section
        # is index-only; we resolve via the ordering b(+holes)+i+appended.
        pass

# Re-parse with index tracking (all_uv = boundary(279) then interior/appended)
order = []
section = None
for ln in open(TRI):
    if ln.startswith("boundary"):
        section = "b"; continue
    if ln.startswith("interior"):
        section = "i"; continue
    if ln.startswith("appended"):
        section = "a"; continue
    if ln.startswith("tris"):
        section = "t"; continue
    if ln.startswith(("holes", "type=")):
        continue
    if section == "b" and ln.startswith("b "):
        _, u, v, x, y, z = ln.split()
        order.append(pk(float(x), float(y), float(z)))
    elif section in ("i", "a") and ln[0] in ("i", "a"):
        _, u, v, x, y, z = ln.split()
        order.append(pk(float(x), float(y), float(z)))
    elif section == "t" and ln.startswith("t "):
        parts = ln.split()
        a, b, c = int(parts[1]), int(parts[2]), int(parts[3])
        try:
            ka, kb, kc = order[a], order[b], order[c]
            cdt_tris.add(frozenset((ka, kb, kc)))
        except IndexError:
            pass
n_bnd_tri = sum(1 for p in order if p in tri_bnd_keys)
print(f"TRI: all_uv={len(order)} (bnd_keys={len(tri_bnd_keys)}, "
      f"int_keys={len(tri_int_keys)}), cdt_tris={len(cdt_tris)}")

# ── parse FACE_MESH emission a64
em_verts = {}   # idx -> poskey
em_tris = []    # (ti, (ia, ib, ic))
vpat = re.compile(r"^v\s+(\d+)\s+(\S+)\s+(\S+)\s+(\S+)")
tpat = re.compile(r"^t\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)")
for ln in open(FM):
    m = vpat.match(ln)
    if m:
        em_verts[int(m.group(1))] = pk(
            float(m.group(2)), float(m.group(3)), float(m.group(4)))
        continue
    m = tpat.match(ln)
    if m:
        em_tris.append((int(m.group(1)),
                        (int(m.group(2)), int(m.group(3)), int(m.group(4)))))
print(f"EMISSION a64: verts={len(em_verts)} tris={len(em_tris)}")

# ── boundary edges of the emission
use = defaultdict(list)
for ti, (a, b, c) in em_tris:
    for x, y in ((a, b), (b, c), (c, a)):
        use[(min(x, y), max(x, y))].append(ti)
bnd = [e for e, ts in use.items() if len(ts) == 1]
rim, slit = [], []
for e in bnd:
    ka, kb = em_verts[e[0]], em_verts[e[1]]
    if ka in tri_bnd_keys and kb in tri_bnd_keys:
        rim.append(e)
    else:
        slit.append(e)
print(f"emission boundary edges: {len(bnd)} = rim {len(rim)} + SLIT {len(slit)}")
for e in sorted(slit):
    ts = use[e][0]
    ka, kb = em_verts[e[0]], em_verts[e[1]]
    src_a = "bnd" if ka in tri_bnd_keys else ("int" if ka in tri_int_keys else "NEW")
    src_b = "bnd" if kb in tri_bnd_keys else ("int" if kb in tri_int_keys else "NEW")
    print(f"  slit edge ({e[0]},{e[1]}): tri#{ts} ends {src_a}/{src_b}")

# ── classify all emission triangles: CDT-native vs ADDED
added, native = [], 0
for ti, (a, b, c) in em_tris:
    key = frozenset((em_verts[a], em_verts[b], em_verts[c]))
    if key in cdt_tris:
        native += 1
    else:
        added.append((ti, (a, b, c)))
print(f"triangles: CDT-native={native}, ADDED={len(added)}")
newv = [v for v, k in em_verts.items()
        if k not in tri_bnd_keys and k not in tri_int_keys]
print(f"emission verts not in CDT point set (Step6 midpoints / gap-fill): "
      f"{len(newv)}")
# which triangles contain NEW verts?
newset = set(newv)
n_added_with_new = sum(1 for ti, (a, b, c) in added
                       if a in newset or b in newset or c in newset)
print(f"ADDED triangles containing a NEW vertex: {n_added_with_new}/"
      f"{len(added)}")
# ADDED triangles purely from CDT points (recombination?)
pure = [(ti, t) for ti, t in added
        if not (t[0] in newset or t[1] in newset or t[2] in newset)]
print(f"ADDED triangles from CDT points only (recombined): {len(pure)}")
for ti, (a, b, c) in pure[:10]:
    print(f"  recombined tri#{ti}: {a} {b} {c}")
