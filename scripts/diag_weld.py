#!/usr/bin/env python3
"""Diagnostic: analyze which triangles degenerate during welding for 3.05.078.stp"""
import subprocess, re

# Run the existing Rust test with extra env to get more detail
result = subprocess.run(
    ["cargo", "test", "-p", "draper-step", "--test", "diag_3d_view", "--", "--nocapture"],
    capture_output=True, text=True, timeout=120,
    cwd="/home/z/my-project"
)
print("STDOUT:", result.stdout[-3000:] if len(result.stdout) > 3000 else result.stdout)
print("STDERR:", result.stderr[-3000:] if len(result.stderr) > 3000 else result.stderr)
