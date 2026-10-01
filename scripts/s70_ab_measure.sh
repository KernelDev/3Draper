#!/usr/bin/env bash
# s70 A/B: NURBS_FILLET_BAND (default ON) vs =0 (s69 baseline).
set -u
cd /home/z/my-project/3Draper
OUT=/home/z/my-project/scripts/s70_ab
mkdir -p "$OUT"
FILES=(
  test/Zentralstaender.stp
  test/drill_top.stp
  test/compressor-13920_top.stp
  test/as1-oc-214.stp
  test/transmission_top.stp
  test/brick_thin_round.stp
  test/brick_thin.stp
  test/brick_thin_hole.stp
)
for f in "${FILES[@]}"; do
  name=$(basename "$f" .stp)
  echo "=== $name ==="
  DRAPPER_NURBS_FILLET_BAND=0 ./target/release/fold_face_probe "$f" \
    > "$OUT/${name}.off.out" 2> "$OUT/${name}.off.err" || true
  ./target/release/fold_face_probe "$f" \
    > "$OUT/${name}.on.out" 2> "$OUT/${name}.on.err" || true
  rg -o "BREP#\d+: \d+ pairs.*" "$OUT/${name}.off.out" > "$OUT/${name}.off.pairs" 2>/dev/null || true
  rg -o "BREP#\d+: \d+ pairs.*" "$OUT/${name}.on.out" > "$OUT/${name}.on.pairs" 2>/dev/null || true
  echo "-- pairs diff (off vs on):"
  diff "$OUT/${name}.off.pairs" "$OUT/${name}.on.pairs" && echo "   IDENTICAL" || true
  rg "not watertight" "$OUT/${name}.off.err" | sort > "$OUT/${name}.off.wt" 2>/dev/null || true
  rg "not watertight" "$OUT/${name}.on.err" | sort > "$OUT/${name}.on.wt" 2>/dev/null || true
  echo "-- watertight lines diff (off vs on):"
  diff "$OUT/${name}.off.wt" "$OUT/${name}.on.wt" && echo "   IDENTICAL" || true
  echo "-- NURBS_FILLET_BAND rescues accepted:"
  rg -c "NURBS_FILLET_BAND rescue" "$OUT/${name}.on.err" || echo "   0"
done
echo "ALL DONE"
