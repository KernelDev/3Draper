#!/usr/bin/env bash
# Determinism gate — formal CI check (Vision 2036 §9 quality engineering).
#
# Runs the draper-step determinism probe N times (default 2) in SEPARATE
# `cargo test` invocations: each process gets a fresh HashMap seed, so any
# hash-order leakage into geometry (vertex/triangle ordering, healing
# decisions, edge-store rebuilds) shows up as digest drift between runs.
# Every DIGEST line the probe prints (mesh digests, solid-level digests,
# per-face digests, instance digests — 667 lines for the current probe
# set) must be byte-identical across runs.
#
# Usage:
#   scripts/determinism_gate.sh [runs]        # default runs = 2
#   DETERMINISM_PROFILE=release scripts/determinism_gate.sh
#
# Exit codes:
#   0 — all runs produced identical digests (gate PASSED)
#   1 — nondeterminism detected (digest drift) or a probe error
#       (MESH_ERR / PARSE_ERR) in any run
#   2 — setup problem: no DIGEST lines produced (probe skipped or the
#       test files are missing — check the LFS checkout)
#
# Probe source: crates/draper-step/tests/determinism_probe.rs
# The probe converts three industrial STEP files (brick_thin_hole.stp,
# compressor-13920_top.stp, as1-oc-214.stp from the test/ directory)
# through the full STEP->heal->solid->mesh pipeline and prints FNV-1a
# digests of the vertex/triangle buffers, the solid topology signature
# (sorted face/edge geometry), per-face triangle digests, and the
# instanced/detailed mesh paths.

set -u

RUNS="${1:-2}"
if [ "$RUNS" -lt 2 ]; then
    echo "determinism_gate: need at least 2 runs to compare, got $RUNS" >&2
    exit 2
fi

PROFILE="${DETERMINISM_PROFILE:-debug}"
case "$PROFILE" in
    debug)   PROFILE_ARGS="" ;;
    release) PROFILE_ARGS="--release" ;;
    *) echo "determinism_gate: unknown DETERMINISM_PROFILE '$PROFILE' (debug|release)" >&2; exit 2 ;;
esac

# Repo root (script lives in scripts/).
cd "$(dirname "$0")/.." || exit 2

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

echo "=== Determinism gate: $RUNS runs (profile: $PROFILE) ==="

for run in $(seq 1 "$RUNS"); do
    OUT="$WORKDIR/run_$run.txt"
    echo "--- run $run / $RUNS ---"
    # stderr is build noise; digests go to stdout via --nocapture.
    if ! cargo test $PROFILE_ARGS -p draper-step --test determinism_probe \
            -- --nocapture 2>/dev/null | grep '^DIGEST' > "$OUT"; then
        echo "determinism_gate: run $run: cargo test failed" >&2
        exit 1
    fi

    # Probe errors inside the digests are gate failures (a MESH_ERR or
    # PARSE_ERR means the pipeline degraded, deterministically or not).
    if grep -qE 'MESH_ERR|PARSE_ERR' "$OUT"; then
        echo "determinism_gate: run $run: probe reported pipeline errors:" >&2
        grep -E 'MESH_ERR|PARSE_ERR' "$OUT" >&2
        exit 1
    fi

    LINES=$(wc -l < "$OUT")
    if [ "$LINES" -eq 0 ]; then
        echo "determinism_gate: run $run produced no DIGEST lines." >&2
        echo "  Is the test/ directory checked out (LFS)?" >&2
        exit 2
    fi
    echo "    $LINES digest lines"
done

# Compare every run against the first.
BASE="$WORKDIR/run_1.txt"
STATUS=0
for run in $(seq 2 "$RUNS"); do
    OUT="$WORKDIR/run_$run.txt"
    if ! diff -u "$BASE" "$OUT" > "$WORKDIR/diff_$run.txt"; then
        echo "DETERMINISM GATE FAILED: run $run differs from run 1:" >&2
        head -n 40 "$WORKDIR/diff_$run.txt" >&2
        STATUS=1
    fi
done

if [ "$STATUS" -ne 0 ]; then
    echo "determinism_gate: cross-run digest drift — nondeterminism detected." >&2
    exit 1
fi

echo "=== DETERMINISM GATE PASSED: $RUNS runs × $(wc -l < "$BASE") digest lines, byte-identical ==="
exit 0
