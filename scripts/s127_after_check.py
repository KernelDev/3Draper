#!/usr/bin/env python3
"""s127: winding-conflict check on the POST-FIX emissions (same metric as
s127_emission_winding.analyze but importable/quiet)."""
import sys
sys.path.insert(0, "scripts")
from s127_emission_winding import analyze

base = "forensics/s127/fm_after"
for arg in sys.argv[1:]:
    analyze(f"{base}/{arg}")
