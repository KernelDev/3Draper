#!/usr/bin/env python3
"""s104 item-1: A/B diff of uv-audit dumps (emission level).

Compares /tmp/s104/A/uvaudit_step<id>.txt vs B for every id: header
counts (bnd/tris/verts), POLY rows (uv+xyz), TRI rows, and reports
which faces differ and in what way (polygon only / triangle emission /
vertex positions).
"""
import sys
from pathlib import Path

dA = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/s104/A")
dB = Path(sys.argv[2] if len(sys.argv) > 2 else "/tmp/s104/B")


def load(p: Path):
    header = None
    poly = []
    tris = []
    other = []
    for line in p.read_text(errors="replace").splitlines():
        if line.startswith("# UVAUDIT"):
            header = line
        elif line.startswith("POLY"):
            poly.append(line)
        elif line.startswith("TRI"):
            tris.append(line)
        else:
            other.append(line)
    return header, poly, tris, other


for pa in sorted(dA.glob("uvaudit_step*.txt")):
    pb = dB / pa.name
    if not pb.exists():
        print(f"{pa.name}: MISSING in B")
        continue
    ha, polya, trisa, othera = load(pa)
    hb, polyb, trisb, otherb = load(pb)
    diffs = []
    if ha != hb:
        diffs.append(f"HEADER A[{ha}] B[{hb}]")
    if polya != polyb:
        npd = sum(1 for x, y in zip(polya, polyb) if x != y)
        diffs.append(f"POLY {len(polya)}->{len(polyb)} rows, {npd} changed in-place")
    if trisa != trisb:
        ntd = sum(1 for x, y in zip(trisa, trisb) if x != y)
        diffs.append(f"TRIS {len(trisa)}->{len(trisb)} rows, {ntd} changed in-place")
    if othera != otherb:
        diffs.append(f"OTHER {len(othera)}->{len(otherb)} rows")
    if diffs:
        print(f"=== {pa.name} DIFFERS ===")
        for d in diffs:
            print(f"  {d}")
print("(files without output above are bit-identical)")
