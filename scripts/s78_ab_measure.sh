#!/usr/bin/env bash
# s78 A/B: NURBS_SAIL_BAND (default ON, rim-ratio 0.35) vs =0 (s77 baseline).
set -u
cd /home/z/my-project/3Draper
OUT=/home/z/my-project/scripts/s78_ab
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
  DRAPPER_NURBS_SAIL_BAND=0 ./target/release/fold_face_probe "$f" \
    > "$OUT/${name}.off.out" 2> "$OUT/${name}.off.err" || true
  ./target/release/fold_face_probe "$f" \
    > "$OUT/${name}.on.out" 2> "$OUT/${name}.on.err" || true
  grep -oE "BREP#[0-9]+: [0-9]+ pairs.*real\)" "$OUT/${name}.off.out" > "$OUT/${name}.off.pairs" || true
  grep -oE "BREP#[0-9]+: [0-9]+ pairs.*real\)" "$OUT/${name}.on.out" > "$OUT/${name}.on.pairs" || true
  echo "-- pairs diff (off vs on):"
  diff "$OUT/${name}.off.pairs" "$OUT/${name}.on.pairs" && echo "   IDENTICAL" || true
  grep "not watertight" "$OUT/${name}.off.err" | sort > "$OUT/${name}.off.wt" 2>/dev/null || true
  grep "not watertight" "$OUT/${name}.on.err" | sort > "$OUT/${name}.on.wt" 2>/dev/null || true
  echo "-- watertight diff:"
  diff "$OUT/${name}.off.wt" "$OUT/${name}.on.wt" && echo "   IDENTICAL" || true
done
