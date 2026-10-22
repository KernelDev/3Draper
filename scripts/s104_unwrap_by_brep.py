#!/usr/bin/env python3
"""s104 item-1: attribute UV_SEAM_UNWRAP events to BREPs (marker-based).

Reads the RUST_LOG=info fold_face_probe output (transmission, run with
DRAPPER_UV_SEAM_UNWRAP_TYPES=sphere,cylinder) and distributes the
`UV_SEAM_UNWRAP: face #N surface=Discriminant(K): M samples re-branched`
events across BREPs using the preceding `BREP #N ...` / `ENTER
triangulate_brep_detailed: BREP #N` markers. Also dedups the multiple
conversion passes (same face seen per pass) keeping one event per
(brep, face, samples) triple.
"""
import re
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "forensics/s104/trans_sc.txt"
brep_re = re.compile(r"ENTER triangulate_brep_detailed: BREP #(\d+)")
unwrap_re = re.compile(
    r"UV_SEAM_UNWRAP: face #(\d+) surface=Discriminant\((\d+)\): (\d+) samples re-branched"
)

KIND = {0: "Plane", 1: "Cylinder", 2: "Cone", 3: "Sphere", 4: "Torus"}

cur = None
events = defaultdict(set)  # brep -> set((face, kind, samples))
order = []
for line in open(path, errors="replace"):
    m = brep_re.search(line)
    if m:
        cur = int(m.group(1))
        if cur not in order:
            order.append(cur)
        continue
    m = unwrap_re.search(line)
    if m and cur is not None:
        face, kind, samples = int(m.group(1)), int(m.group(2)), int(m.group(3))
        events[cur].add((face, kind, samples))

target = int(sys.argv[2]) if len(sys.argv) > 2 else None
for brep in order:
    if target is not None and brep != target:
        continue
    evs = sorted(events.get(brep, ()))
    if not evs:
        continue
    print(f"\n== BREP #{brep}: {len(evs)} unique unwrap faces ==")
    for face, kind, samples in evs:
        print(f"  face #{face} {KIND.get(kind, '?')}: {samples} samples")
