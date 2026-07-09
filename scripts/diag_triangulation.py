#!/usr/bin/env python3
"""
Diagnostic: Load a STEP file via draper-step, dump face triangulation details,
and check if cross-product normals match analytical face normals.
"""
import subprocess
import sys

# We'll write a Rust test instead since the pipeline is Rust-based
# This script just invokes it

print("Running Rust diagnostic test...")
result = subprocess.run(
    ["cargo", "test", "--package", "draper-step", "--test", "diag_3d_view", "--", "--nocapture"],
    capture_output=True, text=True, cwd="/home/z/my-project"
)
print(result.stdout[-2000:] if len(result.stdout) > 2000 else result.stdout)
print(result.stderr[-1000:] if len(result.stderr) > 1000 else result.stderr)
