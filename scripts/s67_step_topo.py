#!/usr/bin/env python3
"""s67 diag 7: STEP topology of HOUSING debt faces.

From facemap: fid -> step_face_id. Then parse the STEP file:
 - the ADVANCED_FACE entity (bounds, surface ref)
 - the surface geometry (CYLINDER/PLANE/TORUS...)
 - the EDGE_CURVEs of its bounds with curve types + vertices
Answers: (a) exact surface params; (b) which edges are shared with
the neighbor debt faces; (c) are the 0.05-spaced chains on ONE face
(two edges of the same face) or on shared edges.
"""
import re
from collections import defaultdict

STP = "/home/z/my-project/3Draper/test/drill_top.stp"
FACEMAP = ("/home/z/my-project/scripts/s67_objs/"
           "brep3_HOUSING (BREP#47598).facemap")
WANT_NAMES = {125, 214, 212, 127, 158, 160, 162, 164, 166, 236, 238,
              130, 14, 12, 196, 198, 200, 202, 204, 261, 69}


def main():
    fid2step = {}
    for line in open(FACEMAP):
        if line.startswith("f "):
            parts = line.split()
            fid2step[int(parts[1])] = int(parts[-2])
    step2fid = {v: k for k, v in fid2step.items()}

    data = open(STP, errors="replace").read()
    # entity id -> (type, args) raw
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;",
                         data, re.S):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    def ref_ids(args):
        return [int(x) for x in re.findall(r"#(\d+)", args)]

    for fid in sorted(WANT_NAMES):
        sid = fid2step.get(fid)
        if sid is None or sid not in ents:
            continue
        typ, args = ents[sid]
        refs = ref_ids(args)
        # ADVANCED_FACE(name, bounds, surface, same_sense)
        surf_ref = refs[0] if typ == "ADVANCED_FACE" else None
        # find surface: 3rd direct arg in the raw args list order
        # parse positional: name,bounds,surface,same_sense
        pos = [p.strip() for p in
               re.findall(r"'[^']*'|\.(T|F)\.|#[\d]+", args)]
        sref = None
        m2 = re.findall(r"#(\d+)", args.split(".T.")[-1]
                        if ".T." in args else args)
        # simpler: surface = last #ref before .T./.F. at depth... use
        # the ORIENTED_EDGE list: bounds are EDGE_LOOPs of ORIENTED_EDGE
        print(f"\n=== fid={fid} STEP#{sid} {typ} ===")
        # bounds: first-level #refs are EDGE_LOOP ids (via FACE_BOUND)
        loops = [r for r in refs if r in ents and
                 ents[r][0] in ("EDGE_LOOP", "VERTEX_LOOP")]
        surf = [r for r in refs if r in ents and ents[r][0] in
                ("CYLINDRICAL_SURFACE", "PLANE", "TOROIDAL_SURFACE",
                 "SPHERICAL_SURFACE", "B_SPLINE_SURFACE_WITH_KNOTS",
                 "CONICAL_SURFACE", "SURFACE_OF_REVOLUTION",
                 "SURFACE_OF_LINEAR_EXTRUSION")]
        print(f"  loops={[l for l in loops]}, surface-refs={surf}")
        for s in surf:
            st, sa = ents[s]
            print(f"  SURFACE #{s} {st}: "
                  f"{sa[:150]}")
        for lp in loops[:6]:
            lt, la = ents[lp]
            oes = [r for r in ref_ids(la) if r in ents and
                   ents[r][0] == "ORIENTED_EDGE"]
            curve_types = defaultdict(int)
            shared_with = []
            for oe in oes:
                oe_refs = ref_ids(ents[oe][1])
                ec = [r for r in oe_refs if r in ents and
                      ents[r][0] == "EDGE_CURVE"]
                for e in ec:
                    et, ea = ents[e][1] if False else ents[e]
                    e_refs = ref_ids(ea)
                    ct = [r for r in e_refs if r in ents and
                          ents[r][0] in
                          ("LINE", "CIRCLE", "ELLIPSE", "B_SPLINE_CURVE_WITH_KNOTS",
                           "B_SPLINE_CURVE", "POLYLINE")]
                    for c in ct:
                        curve_types[ents[c][0]] += 1
            print(f"  loop#{lp}: {len(oes)} oriented-edges, "
                  f"curves={dict(curve_types)}")


if __name__ == "__main__":
    main()
