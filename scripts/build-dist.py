#!/usr/bin/env python3
"""Build the gh-pages dist directory for 3Draper.

This script replaces trunk's html processing:
1. Reads source index.html
2. Replaces the <link data-trunk rel="rust" ...> tag with a proper module script
3. Writes the processed index.html to dist/
4. Creates .nojekyll marker file
"""

from pathlib import Path
import re
import shutil
import sys

VIEWER_DIR = Path("/home/z/my-project/crates/draper-viewer")
DIST_DIR = VIEWER_DIR / "dist"
SRC_HTML = VIEWER_DIR / "index.html"

# Trunk replacement: this is what trunk 0.21 generates for
# <link data-trunk rel="rust" data-wasm-opt="2" data-cargo-no-default-features data-cargo-features="web-deploy" />
TRUNK_REPLACEMENT = """<script type="module">
import init, * as bindings from './draper-viewer.js';
const wasm = await init('./draper-viewer_bg.wasm');
window.wasmBindings = bindings;
dispatchEvent(new CustomEvent("TrunkApplicationStarted", {detail: {wasm}}));
</script>"""


def main() -> int:
    if not SRC_HTML.exists():
        print(f"ERROR: source html not found: {SRC_HTML}", file=sys.stderr)
        return 1
    if not DIST_DIR.exists():
        print(f"ERROR: dist dir not found: {DIST_DIR}", file=sys.stderr)
        return 1
    if not (DIST_DIR / "draper-viewer.js").exists():
        print(f"ERROR: draper-viewer.js not built; run wasm-bindgen first", file=sys.stderr)
        return 1
    if not (DIST_DIR / "draper-viewer_bg.wasm").exists():
        print(f"ERROR: draper-viewer_bg.wasm not built; run wasm-bindgen first", file=sys.stderr)
        return 1

    html = SRC_HTML.read_text(encoding="utf-8")

    # Replace the <link data-trunk rel="rust" .../> tag (single self-closing tag).
    # Match across attributes; trunk emits a single line.
    pattern = re.compile(
        r'<link\s+data-trunk\s+rel="rust"[^>]*/>',
        re.MULTILINE | re.DOTALL,
    )
    new_html, n = pattern.subn(TRUNK_REPLACEMENT, html)
    if n == 0:
        print("WARNING: did not find <link data-trunk rel=\"rust\" ...> in index.html", file=sys.stderr)
    elif n > 1:
        print(f"WARNING: found {n} trunk rust link tags; expected 1", file=sys.stderr)
    print(f"Replaced {n} trunk link tag(s).")

    # Write processed index.html to dist.
    out_html = DIST_DIR / "index.html"
    out_html.write_text(new_html, encoding="utf-8")
    print(f"Wrote {out_html} ({len(new_html)} bytes)")

    # Create .nojekyll so GitHub Pages doesn't strip _-prefixed files.
    nojekyll = DIST_DIR / ".nojekyll"
    nojekyll.write_text("", encoding="utf-8")
    print(f"Wrote {nojekyll}")

    # Copy worker JS files from viewer directory
    for js_file in ["worker.js", "worker-bridge.js"]:
        src = VIEWER_DIR / js_file
        if src.exists():
            dst = DIST_DIR / js_file
            import shutil
            shutil.copy2(src, dst)
            print(f"Copied {src} -> {dst} ({dst.stat().st_size} bytes)")
        else:
            print(f"WARNING: {src} not found — worker will not be available")

    # Summary
    print("\nDist contents:")
    for p in sorted(DIST_DIR.iterdir()):
        print(f"  {p.name:40s} {p.stat().st_size:>10d} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
