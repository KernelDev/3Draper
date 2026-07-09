# Benchmark Baseline

This document records the baseline benchmark results for the 3Draper triangulation
pipeline. Re-run the benchmark with:

```bash
cargo run --release --bin benchmark -- --csv benchmark_baseline.csv
```

## Current Baseline (2026-06-19, after P0-P3)

**Test corpus:** 24 STEP files in `test/` (NIST primitives, as1-oc-214 assembly
+ sub-parts, brick_thin variants, industrial parts: Spit-Fire, Vulcan, Zentralstaender,
compressor, transmission_top, drill_top, 3.05.078, SampleCube).

### Aggregate

| Metric | Value |
|---|---|
| Files | 24 |
| WATERTIGHT (0 boundary edges) | 17 (70.8%) |
| ok (<5% boundary edges) | 7 (29.2%) |
| leaky (5-20% boundary edges) | 0 (0.0%) |
| BAD (>20% boundary edges) | 0 (0.0%) |
| ERROR (parse/conversion) | 0 (0.0%) |
| **PASS RATE** | **24/24 (100%)** |
| Total triangles | 661,261 |
| Total edges | 987,938 (6,350 boundary, 0.64% overall) |
| Total time | 21.75s |
| Throughput | 30,404 triangles/sec, 1.10 files/sec |

### Per-file results

See `benchmark_baseline.csv` for the full per-file breakdown.

## Adding the ABC Dataset

The [Autodesk Benchmark Collection (ABC)](https://abc-technology.org/) is a
standard suite of ~1M CAD models used to evaluate geometry kernels. For
practical benchmarking we use the curated STEP-file subset (~100 files)
distributed by [deepgeometry/abc-data](https://github.com/deepgeometry/abc-data).

### Setup

1. Download the ABC STEP subset:
   ```bash
   git clone --depth 1 https://github.com/deepgeometry/abc-data /tmp/abc-data
   # Or download individual files from https://abc-technology.org/data/
   ```

2. Place `.step` files in `test/abc/` (or any directory of your choice).

3. Run the benchmark:
   ```bash
   cargo run --release --bin benchmark -- test/abc/ --csv benchmark_abc.csv
   ```

4. Compare against this baseline:
   - **Pass rate:** Should be >=95% (vs 100% on the curated 24-file corpus).
     ABC files have higher geometric complexity, so a 5% leaky/BAD rate is acceptable.
   - **Throughput:** Should be >=10,000 triangles/sec on average hardware.
   - **Per-file time:** Should be <10s for 95% of files; <60s for 99%.

### ABC categorization

ABC files are categorized by feature complexity:
- **Simple** (planar + cylindrical faces): expected WATERTIGHT
- **Medium** (NURBS + fillets): expected ok
- **Complex** (B-splines, swept surfaces, self-intersecting UV): expected ok with
  fallback path; some may be leaky due to STEP parser limitations

If a complex file produces BAD output (>20% boundary edges), run:
```bash
cargo run --release --bin single_file_test -- test/abc/<problem_file>.step
```
and inspect the diagnostic logs (RUST_LOG=warn) to identify the failing path.

## Regression Tracking

After any change to `draper-mesh` or `draper-step`, re-run the benchmark and
compare:

- **Pass rate** must not decrease.
- **Total triangle count** should not increase by more than 10% (mesh density
  regression).
- **Total time** should not increase by more than 20% (performance regression).
- **Per-file status** must not regress (WATERTIGHT -> ok is tolerable;
  WATERTIGHT/ok -> leaky/BAD is a regression).

The CSV output enables machine-readable diff between runs:
```bash
diff <(cut -d, -f1,8 old.csv | sort) <(cut -d, -f1,8 new.csv | sort)
```
